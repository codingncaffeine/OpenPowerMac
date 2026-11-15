using System.Collections.Concurrent;
using System.Runtime.InteropServices;
using Avalonia.Platform;

namespace OpenPowerMac.Gui.Platform;

/// <summary>
/// Captured mouse and keyboard for a window on X11 (including Xwayland under a
/// Wayland desktop) -- the recipe SDL and GLFW use for relative mouse mode and
/// keyboard grab:
///
/// - The Mac is fed XInput2 raw motion: device counts before pointer
///   acceleration, which keep arriving wherever the pointer is.
/// - The pointer is grabbed and confined to the window with the cursor hidden
///   through XFixes. Xwayland turns exactly that pair, a hidden cursor and a
///   confined pointer, into the compositor's pointer lock.
/// - The keyboard is grabbed, so the desktop's own shortcuts (the Meta key,
///   Alt+Tab) go to the Mac instead. This is the X11 counterpart of the Windows
///   build's low-level keyboard hook. Under Xwayland the grab is forwarded to
///   the compositor; the window is first marked as allowed to grab, which is
///   what GNOME asks of an X11 client before it honors one.
///
/// One thread owns a private X connection. The UI only posts Lock and Unlock and
/// collects the accumulated motion; buttons and keys taken by the grab are handed
/// to callbacks, which run on that thread. No Xlib call is ever made from two
/// threads, and Avalonia's own connection is untouched.
/// </summary>
internal sealed class X11InputCapture : IDisposable
{
    public enum State
    {
        /// <summary>The grabs are held: the Mac owns the mouse and keyboard.</summary>
        Engaged,
        /// <summary>The grabs could not be taken; nothing is captured.</summary>
        Refused,
    }

    private const int KeyPress = 2, KeyRelease = 3, ButtonPress = 4, ButtonRelease = 5;
    private const int ClientMessage = 33, GenericEvent = 35;
    private const int XI_RawMotion = 17;
    private const int XIAllMasterDevices = 1;
    private const int XIPointerEmulated = 1 << 16;
    private const uint ButtonPressMask = 1 << 2, ButtonReleaseMask = 1 << 3;
    private const nint SubstructureNotifyMask = 1 << 19, SubstructureRedirectMask = 1 << 20;
    private const int GrabModeAsync = 1;
    private const int GrabSuccess = 0;
    private const short POLLIN = 1;
    private const int XEventSize = 192;               // sizeof(XEvent) on 64-bit

    [StructLayout(LayoutKind.Sequential)]
    private struct XIEventMask
    {
        public int deviceid;
        public int mask_len;
        public IntPtr mask;
    }

    [StructLayout(LayoutKind.Sequential)]
    private struct PollFd
    {
        public int fd;
        public short events;
        public short revents;
    }

    private const string X11 = "libX11.so.6";
    private const string Xi = "libXi.so.6";
    private const string Xfixes = "libXfixes.so.3";

    [DllImport(X11)] private static extern IntPtr XOpenDisplay(IntPtr name);
    [DllImport(X11)] private static extern int XCloseDisplay(IntPtr dpy);
    [DllImport(X11)] private static extern nuint XDefaultRootWindow(IntPtr dpy);
    [DllImport(X11)] private static extern int XConnectionNumber(IntPtr dpy);
    [DllImport(X11)] private static extern int XPending(IntPtr dpy);
    [DllImport(X11)] private static extern int XNextEvent(IntPtr dpy, IntPtr ev);
    [DllImport(X11)] private static extern int XFlush(IntPtr dpy);
    [DllImport(X11)] private static extern int XSync(IntPtr dpy, int discard);
    [DllImport(X11)] private static extern int XQueryExtension(IntPtr dpy, string name, out int opcode, out int evt, out int err);
    [DllImport(X11)] private static extern int XGetEventData(IntPtr dpy, IntPtr cookie);
    [DllImport(X11)] private static extern void XFreeEventData(IntPtr dpy, IntPtr cookie);
    [DllImport(X11)] private static extern int XWarpPointer(IntPtr dpy, nuint src, nuint dest, int sx, int sy, uint sw, uint sh, int dx, int dy);
    [DllImport(X11)] private static extern int XGetGeometry(IntPtr dpy, nuint drawable, out nuint root, out int x, out int y,
                                                            out uint width, out uint height, out uint border, out uint depth);
    [DllImport(X11)] private static extern int XGrabPointer(IntPtr dpy, nuint window, int ownerEvents, uint eventMask,
                                                            int pointerMode, int keyboardMode, nuint confineTo, nuint cursor, nuint time);
    [DllImport(X11)] private static extern int XUngrabPointer(IntPtr dpy, nuint time);
    [DllImport(X11)] private static extern int XGrabKeyboard(IntPtr dpy, nuint window, int ownerEvents,
                                                             int pointerMode, int keyboardMode, nuint time);
    [DllImport(X11)] private static extern int XUngrabKeyboard(IntPtr dpy, nuint time);
    [DllImport(X11)] private static extern nuint XInternAtom(IntPtr dpy, string name, int onlyIfExists);
    [DllImport(X11)] private static extern int XSendEvent(IntPtr dpy, nuint window, int propagate, nint eventMask, IntPtr ev);
    [DllImport(X11)] private static extern int XkbSetDetectableAutoRepeat(IntPtr dpy, int detectable, out int supported);
    [DllImport(Xi)] private static extern int XIQueryVersion(IntPtr dpy, ref int major, ref int minor);
    [DllImport(Xi)] private static extern int XISelectEvents(IntPtr dpy, nuint window, ref XIEventMask masks, int count);
    [DllImport(Xfixes)] private static extern int XFixesQueryExtension(IntPtr dpy, out int evt, out int err);
    [DllImport(Xfixes)] private static extern int XFixesQueryVersion(IntPtr dpy, ref int major, ref int minor);
    [DllImport(Xfixes)] private static extern void XFixesHideCursor(IntPtr dpy, nuint window);
    [DllImport(Xfixes)] private static extern void XFixesShowCursor(IntPtr dpy, nuint window);
    [DllImport("libc")] private static extern int poll([In, Out] PollFd[] fds, nuint nfds, int timeout);
    [DllImport("libc")] private static extern int pipe([Out] int[] fds);
    [DllImport("libc")] private static extern nint read(int fd, byte[] buf, nuint count);
    [DllImport("libc")] private static extern nint write(int fd, byte[] buf, nuint count);
    [DllImport("libc")] private static extern int close(int fd);

    private enum Command { Lock, Unlock, Quit }

    private readonly nuint _window;
    private readonly Action<string> _log;
    private readonly Action<int, bool> _onButton;     // X button number (1 left, 2 middle, 3 right), pressed
    private readonly Action<int, bool> _onKey;        // evdev key code, pressed
    private readonly Action<State> _onState;
    private readonly Thread _thread;
    private readonly ConcurrentQueue<Command> _commands = new();
    private readonly int _wakeRead = -1, _wakeWrite = -1;   // the UI pokes the thread's poll() through this pipe
    private readonly object _motionLock = new();
    private double _dx, _dy;
    private volatile bool _wantLocked;                // raw motion counts from the moment a lock is asked for

    // Thread-owned state.
    private IntPtr _dpy;
    private nuint _root;
    private int _xiOpcode;
    private bool _grabbed;
    private readonly PollFd[] _pollFds = new PollFd[2];
    private readonly byte[] _wakeBuffer = new byte[64];

    // Instruments, reset at every lock: what the capture actually delivered.
    private long _rawPackets, _buttons, _keys;

    private X11InputCapture(nuint window, Action<string> log, Action<int, bool> onButton,
                            Action<int, bool> onKey, Action<State> onState)
    {
        _window = window;
        _log = log;
        _onButton = onButton;
        _onKey = onKey;
        _onState = onState;
        int[] fds = new int[2];
        if (pipe(fds) == 0)
        {
            _wakeRead = fds[0];
            _wakeWrite = fds[1];
        }
        _thread = new Thread(Run) { IsBackground = true, Name = "opm-input-capture" };
    }

    /// <summary>A capture for this window, or null when it is not an X11 window
    /// or the X server lacks XInput 2.1 or XFixes 4. The callbacks run on the
    /// capture's own thread.</summary>
    public static X11InputCapture? TryCreate(IPlatformHandle? handle, Action<string> log,
                                             Action<int, bool> onButton, Action<int, bool> onKey,
                                             Action<State> onState)
    {
        if (handle is null || handle.HandleDescriptor != "XID") return null;
        var capture = new X11InputCapture((nuint)(ulong)handle.Handle, log, onButton, onKey, onState);
        using var ready = new ManualResetEventSlim();
        bool ok = false;
        capture._thread.Start((Action<bool>)(result => { ok = result; ready.Set(); }));
        ready.Wait();
        if (ok) return capture;
        capture.Dispose();
        return null;
    }

    /// <summary>Ask for the capture. <see cref="State.Engaged"/> or
    /// <see cref="State.Refused"/> follows through the state callback.</summary>
    public void Lock()
    {
        lock (_motionLock) { _dx = _dy = 0; }
        Interlocked.Exchange(ref _rawPackets, 0);
        Interlocked.Exchange(ref _buttons, 0);
        Interlocked.Exchange(ref _keys, 0);
        _wantLocked = true;
        Post(Command.Lock);
    }

    public void Unlock()
    {
        _wantLocked = false;
        Post(Command.Unlock);
    }

    /// <summary>Motion gathered since the last call, in device counts.</summary>
    public void TakeMotion(out double dx, out double dy)
    {
        lock (_motionLock)
        {
            dx = _dx; dy = _dy;
            _dx = _dy = 0;
        }
    }

    public string Stats() =>
        $"raw={Interlocked.Read(ref _rawPackets)} buttons={Interlocked.Read(ref _buttons)} "
        + $"keys={Interlocked.Read(ref _keys)}";

    private void Post(Command command)
    {
        _commands.Enqueue(command);
        if (_wakeWrite >= 0) write(_wakeWrite, new byte[] { 1 }, 1);
    }

    private void Run(object? state)
    {
        var started = (Action<bool>)state!;
        _dpy = XOpenDisplay(IntPtr.Zero);
        if (_dpy == IntPtr.Zero)
        {
            _log("input: capture unavailable -- could not open the X display");
            started(false);
            return;
        }
        IntPtr ev = Marshal.AllocHGlobal(XEventSize);
        IntPtr maskBytes = Marshal.AllocHGlobal(4);
        try
        {
            // XInput 2.1 is what keeps raw events arriving while the pointer is grabbed.
            int major = 2, minor = 2;
            if (XQueryExtension(_dpy, "XInputExtension", out _xiOpcode, out _, out _) == 0 ||
                XIQueryVersion(_dpy, ref major, ref minor) != 0 || major < 2 || (major == 2 && minor < 1))
            {
                _log("input: capture unavailable -- the X server has no XInput 2.1");
                started(false);
                return;
            }
            int fixesMajor = 4, fixesMinor = 0;
            if (XFixesQueryExtension(_dpy, out _, out _) == 0 ||
                XFixesQueryVersion(_dpy, ref fixesMajor, ref fixesMinor) == 0 || fixesMajor < 4)
            {
                _log("input: capture unavailable -- the X server has no XFixes 4");
                started(false);
                return;
            }
            _root = XDefaultRootWindow(_dpy);
            // A held key then arrives as repeated presses with no releases between
            // them, so the receiver can tell a repeat from a new press.
            XkbSetDetectableAutoRepeat(_dpy, 1, out _);

            // Raw events are delivered on the root window only. Selected once and
            // left on; they are counted only while a lock is wanted.
            Marshal.WriteInt32(maskBytes, 1 << XI_RawMotion);
            var mask = new XIEventMask { deviceid = XIAllMasterDevices, mask_len = 4, mask = maskBytes };
            XISelectEvents(_dpy, _root, ref mask, 1);
            XFlush(_dpy);

            int fd = XConnectionNumber(_dpy);
            _log($"input: X11 capture ready (XInput {major}.{minor}, XFixes {fixesMajor}.{fixesMinor}, "
                 + $"window 0x{(ulong)_window:X})");
            started(true);

            while (true)
            {
                Wait(fd, 50);
                Drain(ev);
                while (_commands.TryDequeue(out Command command))
                {
                    switch (command)
                    {
                        case Command.Lock: Engage(fd, ev); break;
                        case Command.Unlock: Release(); break;
                        case Command.Quit: Release(); return;
                    }
                }
            }
        }
        catch (Exception ex)
        {
            _log("input: capture thread failed -- " + ex.Message);
        }
        finally
        {
            Marshal.FreeHGlobal(maskBytes);
            Marshal.FreeHGlobal(ev);
            XCloseDisplay(_dpy);   // the server drops this client's grabs and cursor hide with it
            _dpy = IntPtr.Zero;
        }
    }

    private void Engage(int fd, IntPtr ev)
    {
        if (_grabbed || !_wantLocked) return;

        // GNOME honors an X11 client's keyboard grab only for a window that has
        // announced it may grab; other desktops ignore the message.
        nuint mayGrab = XInternAtom(_dpy, "_XWAYLAND_MAY_GRAB_KEYBOARD", 0);
        IntPtr message = Marshal.AllocHGlobal(XEventSize);
        try
        {
            for (int i = 0; i < XEventSize; i += 8) Marshal.WriteInt64(message, i, 0);
            Marshal.WriteInt32(message, 0, ClientMessage);
            Marshal.WriteInt64(message, 32, (long)(ulong)_window);
            Marshal.WriteInt64(message, 40, (long)(ulong)mayGrab);
            Marshal.WriteInt32(message, 48, 32);
            Marshal.WriteInt64(message, 56, 1);
            XSendEvent(_dpy, _root, 0, SubstructureNotifyMask | SubstructureRedirectMask, message);
        }
        finally { Marshal.FreeHGlobal(message); }

        // The cursor is hidden per screen, so the root window is always a valid target.
        XFixesHideCursor(_dpy, _root);
        XFlush(_dpy);

        // The click that asked for the capture is usually still held, and the
        // toolkit's implicit grab for that press refuses ours until the button
        // comes up -- so the grab is retried for a while rather than given up on.
        int pointer = -1, keyboard = -1;
        for (int attempt = 0; attempt < 200 && _wantLocked; attempt++)
        {
            if (pointer != GrabSuccess)
                pointer = XGrabPointer(_dpy, _window, 0, ButtonPressMask | ButtonReleaseMask,
                                       GrabModeAsync, GrabModeAsync, _window, 0, 0);
            if (pointer == GrabSuccess && keyboard != GrabSuccess)
                keyboard = XGrabKeyboard(_dpy, _window, 0, GrabModeAsync, GrabModeAsync, 0);
            if (pointer == GrabSuccess && keyboard == GrabSuccess) break;
            Wait(fd, 10);
            Drain(ev);
        }
        if (pointer != GrabSuccess || keyboard != GrabSuccess || !_wantLocked)
        {
            if (pointer == GrabSuccess) XUngrabPointer(_dpy, 0);
            if (keyboard == GrabSuccess) XUngrabKeyboard(_dpy, 0);
            XFixesShowCursor(_dpy, _root);
            XSync(_dpy, 0);
            if (_wantLocked)
            {
                _log($"input: capture refused (pointer grab {GrabName(pointer)}, keyboard grab {GrabName(keyboard)})");
                _onState(State.Refused);
            }
            return;
        }
        _grabbed = true;
        WarpToCenter();
        XSync(_dpy, 0);
        _onState(State.Engaged);
    }

    private void Release()
    {
        if (!_grabbed) return;
        _grabbed = false;
        WarpToCenter();   // the pointer reappears in the middle of the Mac's screen
        XUngrabKeyboard(_dpy, 0);
        XUngrabPointer(_dpy, 0);
        XFixesShowCursor(_dpy, _root);
        XSync(_dpy, 0);
    }

    private void WarpToCenter()
    {
        if (XGetGeometry(_dpy, _window, out _, out _, out _, out uint w, out uint h, out _, out _) != 0 && w > 0 && h > 0)
            XWarpPointer(_dpy, 0, _window, 0, 0, 0, 0, (int)(w / 2), (int)(h / 2));
    }

    private static string GrabName(int result) => result switch
    {
        0 => "ok", 1 => "AlreadyGrabbed", 2 => "InvalidTime", 3 => "NotViewable", 4 => "Frozen",
        _ => "not tried",
    };

    private void Wait(int fd, int timeoutMs)
    {
        _pollFds[0] = new PollFd { fd = fd, events = POLLIN };
        _pollFds[1] = new PollFd { fd = _wakeRead, events = POLLIN };
        // Xlib may already hold events it read off the socket while waiting for a reply.
        if (XPending(_dpy) > 0) timeoutMs = 0;
        poll(_pollFds, _wakeRead >= 0 ? 2u : 1u, timeoutMs);
        if (_wakeRead >= 0 && (_pollFds[1].revents & POLLIN) != 0)
            read(_wakeRead, _wakeBuffer, (nuint)_wakeBuffer.Length);
    }

    private void Drain(IntPtr ev)
    {
        while (XPending(_dpy) > 0)
        {
            XNextEvent(_dpy, ev);
            int type = Marshal.ReadInt32(ev, 0);
            switch (type)
            {
                case GenericEvent:
                    // XGenericEventCookie: extension @32, evtype @36, data @48.
                    if (Marshal.ReadInt32(ev, 32) != _xiOpcode || Marshal.ReadInt32(ev, 36) != XI_RawMotion) break;
                    if (XGetEventData(_dpy, ev) == 0) break;
                    try
                    {
                        if (_wantLocked) ReadRawMotion(Marshal.ReadIntPtr(ev, 48));
                    }
                    finally { XFreeEventData(_dpy, ev); }
                    break;
                case ButtonPress or ButtonRelease:
                    // These reach this connection only through the grab. XButtonEvent.button @84.
                    if (!_grabbed) break;
                    Interlocked.Increment(ref _buttons);
                    _onButton(Marshal.ReadInt32(ev, 84), type == ButtonPress);
                    break;
                case KeyPress or KeyRelease:
                    // XKeyEvent.keycode @84. An X keycode is the kernel's key code plus 8.
                    if (!_grabbed) break;
                    Interlocked.Increment(ref _keys);
                    _onKey(Marshal.ReadInt32(ev, 84) - 8, type == KeyPress);
                    break;
            }
        }
    }

    // XIRawEvent: flags @60, valuators.mask_len @64, valuators.mask @72, raw_values @88.
    // The mask names which axes the event carries; their values are packed in
    // mask order. Axis 0 is X and axis 1 is Y on every pointer.
    private void ReadRawMotion(IntPtr raw)
    {
        if ((Marshal.ReadInt32(raw, 60) & XIPointerEmulated) != 0) return;   // made up from a touch, not a mouse
        int maskLen = Marshal.ReadInt32(raw, 64);
        IntPtr maskPtr = Marshal.ReadIntPtr(raw, 72);
        IntPtr values = Marshal.ReadIntPtr(raw, 88);
        if (maskLen <= 0 || maskPtr == IntPtr.Zero || values == IntPtr.Zero) return;
        double dx = 0, dy = 0;
        int index = 0;
        for (int axis = 0; axis < maskLen * 8 && axis <= 1; axis++)
        {
            byte b = Marshal.ReadByte(maskPtr, axis / 8);
            if ((b & (1 << (axis % 8))) == 0) continue;
            double value = BitConverter.Int64BitsToDouble(Marshal.ReadInt64(values, index * 8));
            if (axis == 0) dx = value;
            else dy = value;
            index++;
        }
        Interlocked.Increment(ref _rawPackets);
        lock (_motionLock) { _dx += dx; _dy += dy; }
    }

    public void Dispose()
    {
        _wantLocked = false;
        Post(Command.Quit);
        // The thread lets go of the grabs before the window goes away.
        if (_thread.IsAlive) _thread.Join(2000);
        if (_wakeRead >= 0) close(_wakeRead);
        if (_wakeWrite >= 0) close(_wakeWrite);
    }
}
