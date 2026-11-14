using System.Diagnostics;
using System.Runtime.InteropServices;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Interactivity;
using Avalonia.Media.Imaging;
using Avalonia.Platform;
using Avalonia.Platform.Storage;
using Avalonia.Threading;
using OpenPowerMac.Gui.Dialogs;
using OpenPowerMac.Gui.Platform;
using OpenPowerMac.Shell;

namespace OpenPowerMac.Gui;

/// <summary>The Linux window. Behaviour follows the Windows front end
/// (shell/MainWindow.xaml.cs) menu for menu; the machine itself is driven by
/// the shared <see cref="MachineSession"/>, which owns every capi call on its
/// own thread. This class only queues input and draws what the session
/// publishes, so nothing slow ever runs on the UI thread.</summary>
public partial class MainWindow : Window
{
    private readonly ShellSettings _settings = ShellSettings.Load();
    private readonly TerminalBuffer _term = new();
    private readonly DispatcherTimer _timer;
    private MachineSession? _session;
    private WriteableBitmap? _bmp;
    private X11InputCapture? _grab;
    private bool _closeWhenStopped;

    private static readonly Cursor HiddenCursor = new(StandardCursorType.None);

    public MainWindow()
    {
        InitializeComponent();
        miAutoBoot.IsChecked = _settings.AutoBoot;
        miSound.IsChecked = _settings.Sound;
        UpdateTitle();
        UpdateNoVideoHint();
        RestoreWindowLayout();

        // 60 Hz: the frame is copied only when the session published a new
        // one, and pointer travel gathered by the capture is handed over on
        // every tick, so the guest's cursor moves at the display's rate.
        _timer = new DispatcherTimer(TimeSpan.FromMilliseconds(16), DispatcherPriority.Render, OnTimer);
        _timer.Start();

        WireInput();

        Opened += (_, _) =>
        {
            IPlatformHandle? handle = TryGetPlatformHandle();
            Log.Line($"window: platform handle {handle?.HandleDescriptor ?? "(none)"}, scaling {RenderScaling:0.##}");
            // The capture's callbacks arrive on its own thread; everything
            // they do is posted back here.
            _grab = X11InputCapture.TryCreate(handle, Log.Line,
                (button, down) => Dispatcher.UIThread.Post(() => OnGrabButton(button, down)),
                (code, down) => Dispatcher.UIThread.Post(() => OnGrabKey(code, down)),
                state => Dispatcher.UIThread.Post(() => OnGrabState(state)));
            if (_grab is null)
                Log.Line("input: no X11 capture on this backend; captured motion is measured over the window only");
        };
        Closing += (_, e) =>
        {
            // The machine writes its disk image as it runs: let it stop and
            // close the image before the process goes, rather than killing the
            // thread mid-write.
            if (_session is { Running: true } s)
            {
                e.Cancel = true;
                _closeWhenStopped = true;
                s.Stop();
                txtState.Text = "stopping…";
                return;
            }
            ReleasePointer();
            _grab?.Dispose();
            _grab = null;
            _timer.Stop();
            // The window is really going now. Written here and not left to
            // the pending background save: the process is about to end.
            _layoutSave?.Stop();
            CaptureWindowLayout();
            _settings.Save();
        };
    }

    // ---- the window as the user left it ----
    //
    // Size, whether it was maximized, and the height of the console pane are
    // put back at the next launch. The size remembered is the NORMAL one:
    // while the window is maximized its size is the screen's, and saving
    // that would make "restore" a window as big as the screen.

    private DispatcherTimer? _layoutSave;
    private Size _normalSize;
    private bool _layoutReady;

    private void RestoreWindowLayout()
    {
        if (_settings.WindowWidth >= MinWidth && _settings.WindowHeight >= MinHeight)
        {
            // Bounded, so a damaged settings file cannot ask for a window no
            // screen can show; the window manager fits it to the real screen.
            Width = Math.Min(_settings.WindowWidth, 16384);
            Height = Math.Min(_settings.WindowHeight, 16384);
        }
        if (_settings.ConsoleHeight > 0)
        {
            RowDefinition console = grdMain.RowDefinitions[2];
            console.Height = new GridLength(Math.Max(_settings.ConsoleHeight, console.MinHeight));
        }
        _normalSize = new Size(Width, Height);
        if (_settings.WindowMaximized)
            WindowState = WindowState.Maximized;

        // Changes are saved a moment after they stop arriving: a drag sends a
        // stream of sizes and only the last one is worth a write.
        _layoutSave = new DispatcherTimer(TimeSpan.FromMilliseconds(800), DispatcherPriority.Background,
            (_, _) =>
            {
                _layoutSave!.Stop();
                CaptureWindowLayout();
                _settings.SaveInBackground();
            });
        _layoutSave.Stop();
        Opened += (_, _) => _layoutReady = true;
        PropertyChanged += (_, e) =>
        {
            if (!_layoutReady)
                return;
            if (e.Property == ClientSizeProperty || e.Property == WindowStateProperty)
                TouchLayout();
        };
        grdMain.RowDefinitions[2].PropertyChanged += (_, e) =>
        {
            if (_layoutReady && e.Property == RowDefinition.HeightProperty)
                TouchLayout();
        };
    }

    private void TouchLayout()
    {
        _layoutSave!.Stop();
        _layoutSave.Start();
    }

    private void CaptureWindowLayout()
    {
        // The normal size is read HERE, once the changes have stopped, and
        // only while the window is in its normal state. On the way to
        // maximized the new size and the new state arrive as two events in
        // an order the window manager picks, and a size taken between them
        // would be the screen's.
        if (WindowState == WindowState.Normal)
            _normalSize = ClientSize;
        // Minimized is not a state to come back to; it counts as whatever
        // the window was before.
        if (WindowState != WindowState.Minimized)
            _settings.WindowMaximized = WindowState == WindowState.Maximized;
        if (_normalSize.Width >= MinWidth && _normalSize.Height >= MinHeight)
        {
            _settings.WindowWidth = Math.Round(_normalSize.Width);
            _settings.WindowHeight = Math.Round(_normalSize.Height);
        }
        GridLength console = grdMain.RowDefinitions[2].Height;
        if (console.IsAbsolute && console.Value > 0)
            _settings.ConsoleHeight = Math.Round(console.Value);
    }

    // ---- guest input ----

    private bool _captured;
    private Point? _lastMouse;
    private double _pendX, _pendY;
    private uint _buttons;

    // What one capture delivered, logged when it ends: a pointer that "does
    // not move" is either nothing arriving from the grab or nothing reaching
    // the guest, and these two numbers say which.
    private long _sentReports, _sentDx, _sentDy, _sentButtons;
    // OPM_INPUT_TRACE=1 logs every report handed to the guest.
    private static readonly bool InputTrace = Environment.GetEnvironmentVariable("OPM_INPUT_TRACE") == "1";

    // Everything the guest currently believes is held down, so it can be let
    // go when this window stops receiving key-ups.
    private readonly HashSet<uint> _held = new();

    private void WireInput()
    {
        Deactivated += (_, _) =>
        {
            // Alt-tabbing away with the pointer swallowed would leave it hidden
            // and pinned, and a modifier held at that moment would stay held.
            ReleasePointer();
            ReleaseHeldKeys();
        };

        imgScreen.PointerMoved += OnScreenPointerMoved;
        imgScreen.PointerPressed += OnScreenPointerPressed;

        // While captured, buttons are taken at the window before anything under
        // the hidden pointer sees them. With the X11 grab engaged the grab
        // delivers them instead (OnGrabButton) and none arrive here.
        AddHandler(PointerPressedEvent, OnWindowPointerButton, RoutingStrategies.Tunnel);
        AddHandler(PointerReleasedEvent, OnWindowPointerButton, RoutingStrategies.Tunnel);

        // Keys are taken before the focused control sees them, so Tab and the
        // arrows reach the guest instead of moving focus around the window.
        AddHandler(KeyDownEvent, OnGuestKeyDown, RoutingStrategies.Tunnel);
        AddHandler(KeyUpEvent, OnGuestKeyUp, RoutingStrategies.Tunnel);

        AddHandler(DragDrop.DragOverEvent, OnDisplayDragOver);
        AddHandler(DragDrop.DropEvent, OnDisplayDrop);
    }

    private static uint ButtonMask(PointerPointProperties p) =>
        (p.IsLeftButtonPressed ? 1u : 0u) | (p.IsRightButtonPressed ? 2u : 0u);

    /// <summary>Take the pointer: hide it and route raw travel to the guest.
    /// Middle-click or Esc gives it back.</summary>
    private void CapturePointer()
    {
        if (_captured || _session is not { Running: true })
            return;
        _captured = true;
        _pendX = _pendY = 0;
        _sentReports = _sentDx = _sentDy = _sentButtons = 0;
        _lastMouse = null;
        Cursor = HiddenCursor;
        imgScreen.Cursor = HiddenCursor;
        _grab?.Lock();
        UpdateCaptureHint();
    }

    private void ReleasePointer()
    {
        if (!_captured)
            return;
        FlushMouse();
        _captured = false;
        _grab?.Unlock();
        // Once the keyboard grab is gone, the releases of keys still held go
        // to whatever has focus, not to us.
        ReleaseHeldKeys();
        if (_buttons != 0 && _session is { Running: true } s)
            s.SendMouse(0, 0, 0);
        _buttons = 0;
        _lastMouse = null;
        Cursor = null;
        imgScreen.Cursor = null;
        UpdateCaptureHint();
        Log.Line($"input: capture released — sent {_sentReports} motion reports, travel {_sentDx}/{_sentDy}, "
                 + $"{_sentButtons} button reports; source {(_grab is null ? "window pointer" : "X11 raw (" + _grab.Stats() + ")")}");
    }

    private bool _hintCaptured, _hintVisible;

    private void UpdateCaptureHint()
    {
        bool visible = _session is { Running: true };
        if (visible != _hintVisible)
        {
            _hintVisible = visible;
            brdCapture.IsVisible = visible;
        }
        if (_captured != _hintCaptured)
        {
            _hintCaptured = _captured;
            txtCapture.Text = _captured
                ? "Pointer captured — middle-click (or Esc) to release"
                : "Click the screen to capture the pointer";
        }
    }

    // Travel, not position: a USB boot mouse is a RELATIVE device. Nothing is
    // metered (see the Windows shell's note on why a meter made the pointer
    // slow); only the sub-unit remainder is carried, because a HID report
    // holds whole counts.
    private void SendTravel(double dx, double dy)
    {
        _pendX += dx;
        _pendY += dy;
        FlushMouse();
    }

    private void FlushMouse()
    {
        if (_grab is not null && _captured)
        {
            _grab.TakeMotion(out double rx, out double ry);
            _pendX += rx;
            _pendY += ry;
        }
        if (_session is not { Running: true } s)
        {
            _pendX = _pendY = 0;
            return;
        }
        int dx = (int)Math.Truncate(_pendX);
        int dy = (int)Math.Truncate(_pendY);
        if (dx == 0 && dy == 0)
            return;
        _pendX -= dx;
        _pendY -= dy;
        if (_captured)
        {
            _sentReports++;
            _sentDx += dx;
            _sentDy += dy;
        }
        if (InputTrace)
            Log.Line($"input trace: mouse {dx},{dy} buttons {_buttons}");
        s.SendMouse(dx, dy, _buttons);
    }

    private void OnScreenPointerMoved(object? sender, PointerEventArgs e)
    {
        if (_session is not { Running: true })
            return;
        Point p = e.GetPosition(imgScreen);
        if (_captured)
        {
            // With the X11 grab the raw device counts arrive through the
            // capture; this path is the fallback for a backend without it.
            if (_grab is null && _lastMouse is { } last)
                SendTravel(p.X - last.X, p.Y - last.Y);
            _lastMouse = p;
            return;
        }
        // Uncaptured: still track, so the guest follows the pointer before the
        // user commits to capturing. The image is letterboxed and scaled, so a
        // window pixel is not a guest pixel.
        if (_lastMouse is { } prev && _bmp != null && imgScreen.Bounds.Width > 0 && imgScreen.Bounds.Height > 0)
        {
            double sx = _bmp.PixelSize.Width / imgScreen.Bounds.Width;
            double sy = _bmp.PixelSize.Height / imgScreen.Bounds.Height;
            _buttons = ButtonMask(e.GetCurrentPoint(imgScreen).Properties);
            SendTravel(Math.Round((p.X - prev.X) * sx), Math.Round((p.Y - prev.Y) * sy));
        }
        _lastMouse = p;
    }

    private void OnScreenPointerPressed(object? sender, PointerPressedEventArgs e)
    {
        if (_captured || _session is not { Running: true } s)
            return;
        imgScreen.Focus();
        PointerPointProperties props = e.GetCurrentPoint(imgScreen).Properties;
        e.Handled = true;
        // The middle button is the capture toggle and never reaches the guest.
        if (props.IsMiddleButtonPressed)
        {
            CapturePointer();
            return;
        }
        // The click that captures is also the guest's click, as on Windows.
        CapturePointer();
        _buttons = ButtonMask(props);
        s.SendMouse(0, 0, _buttons);
    }

    private void OnWindowPointerButton(object? sender, PointerEventArgs e)
    {
        if (!_captured || _session is not { Running: true } s)
            return;
        e.Handled = true;
        PointerPointProperties props = e.GetCurrentPoint(this).Properties;
        if (e is PointerPressedEventArgs && props.IsMiddleButtonPressed)
        {
            ReleasePointer();
            return;
        }
        uint mask = ButtonMask(props);
        if (mask == _buttons)
            return;
        FlushMouse();
        _buttons = mask;
        // Zero motion, current buttons: press and release are each a report.
        s.SendMouse(0, 0, _buttons);
    }

    // ---- input taken by the X11 grab (its thread, posted here) ----

    private void OnGrabButton(int button, bool down)
    {
        if (!_captured || _session is not { Running: true } s)
            return;
        uint bit = button switch { 1 => 1u, 3 => 2u, _ => 0u };
        if (button == 2)
        {
            if (down) ReleasePointer();
            return;
        }
        if (bit == 0)
            return;   // wheel and extra buttons: a boot mouse has two
        FlushMouse();
        _buttons = down ? _buttons | bit : _buttons & ~bit;
        _sentButtons++;
        if (InputTrace)
            Log.Line($"input trace: button {button} {(down ? "down" : "up")} -> buttons {_buttons}");
        s.SendMouse(0, 0, _buttons);
    }

    private void OnGrabKey(int evdev, bool down)
    {
        if (!_captured)
            return;
        if (evdev == HidKeys.EvdevEscape)
        {
            if (down) ReleasePointer();
            return;
        }
        GuestKey(HidKeys.UsageForEvdev(evdev), down);
    }

    private void OnGrabState(X11InputCapture.State state)
    {
        if (!_captured)
            return;
        if (state == X11InputCapture.State.Engaged)
        {
            Log.Line("input: capture engaged — pointer locked, keyboard grabbed");
            return;
        }
        // Something else holds the pointer or keyboard. Let go rather than sit
        // in a half capture that hides the pointer without holding it.
        ReleasePointer();
        txtState.Text = "could not capture input";
    }

    /// <summary>One key transition for the guest. A USB keyboard reports a key
    /// once and the OS decides how to repeat it, so host repeats (a second
    /// down for a key already held) are not forwarded.</summary>
    private void GuestKey(uint usage, bool down)
    {
        if (usage == 0 || _session is not { Running: true } s)
            return;
        if (down ? !_held.Add(usage) : !_held.Remove(usage))
            return;
        s.SendKeyEvent(usage, down);
    }

    private void OnGuestKeyDown(object? sender, KeyEventArgs e)
    {
        // Esc releases the pointer before anything else looks at the key.
        if (_captured && e.Key == Key.Escape)
        {
            ReleasePointer();
            e.Handled = true;
            return;
        }
        // Alt-F4 belongs to the host: a window that cannot be closed from the
        // keyboard is a trap.
        if (e.Key == Key.F4 && e.KeyModifiers.HasFlag(KeyModifiers.Alt))
            return;
        // The serial box is a host control, and an open menu is being driven
        // from the keyboard; both keep their own keys.
        if (txtInput.IsFocused || MenuIsOpen)
            return;
        if (_session is not { Running: true })
            return;
        uint usage = HidKeys.Usage(e.PhysicalKey);
        if (usage == 0)
            return;
        GuestKey(usage, true);
        // Taken, so Alt does not open the menu bar and Tab does not walk focus.
        e.Handled = true;
    }

    // A menu opened with the mouse leaves Menu.IsOpen to the top-level item's
    // own state, so the submenus are asked too.
    private bool MenuIsOpen =>
        MainMenu.IsOpen || MainMenu.Items.OfType<MenuItem>().Any(m => m.IsSubMenuOpen);

    private void OnGuestKeyUp(object? sender, KeyEventArgs e)
    {
        uint usage = HidKeys.Usage(e.PhysicalKey);
        if (usage == 0 || !_held.Contains(usage))
            return;
        GuestKey(usage, false);
        e.Handled = true;
    }

    /// <summary>Let go of everything the guest thinks is down.</summary>
    private void ReleaseHeldKeys()
    {
        if (_held.Count == 0)
            return;
        if (_session is { Running: true } s)
            foreach (uint usage in _held)
                s.SendKeyEvent(usage, false);
        _held.Clear();
    }

    // ---- machine control ----

    private void OnStart(object? sender, RoutedEventArgs e)
    {
        if (_session != null)
            return;
        if (!File.Exists(_settings.RomPath) && !ChooseRom())
            return;
        var session = new MachineSession();
        session.Ended += reason => Dispatcher.UIThread.Post(() => OnSessionEnded(session, reason));
        _session = session;
        session.Start(_settings);
        miStart.IsEnabled = false;
        miStop.IsEnabled = true;
        btnSend.IsEnabled = true;
        txtState.Text = "running";
    }

    private void OnStop(object? sender, RoutedEventArgs e)
    {
        _session?.Stop();
        txtState.Text = "stopping…";
    }

    private void OnSessionEnded(MachineSession session, string reason)
    {
        ReleasePointer();
        ReleaseHeldKeys();
        // The worker's last messages may still be queued: render them.
        while (session.ConsoleQ.TryDequeue(out var chunk))
        {
            _term.Feed(chunk);
            Log.Console(chunk);
        }
        RenderConsole(force: true);
        _session = null;
        miStart.IsEnabled = true;
        miStop.IsEnabled = false;
        btnSend.IsEnabled = false;
        txtState.Text = reason;
        UpdateCaptureHint();
        if (_closeWhenStopped)
            Close();
    }

    // ---- serial console ----

    private void OnSend(object? sender, RoutedEventArgs e) => SendLine();

    private void OnInputKeyDown(object? sender, KeyEventArgs e)
    {
        if (e.Key == Key.Enter)
        {
            SendLine();
            e.Handled = true;
        }
    }

    private void SendLine()
    {
        if (_session == null)
            return;
        _session.SendSerial(txtInput.Text ?? "");
        txtInput.Text = "";
    }

    private long _consoleShownAt;

    // The pane is re-laid out on every assignment, so it is refreshed at most
    // ten times a second however fast the firmware talks.
    private void RenderConsole(bool force = false)
    {
        long now = Environment.TickCount64;
        if (!force && now - _consoleShownAt < 100)
            return;
        if (!_term.TakeDirty())
            return;
        _consoleShownAt = now;
        string text = _term.Render();
        txtConsole.Text = text;
        txtConsole.CaretIndex = text.Length;
    }

    // ---- frame + console pump ----

    private long _statsShownAt;

    private void OnTimer(object? sender, EventArgs e)
    {
        var s = _session;
        if (s != null)
            while (s.ConsoleQ.TryDequeue(out var chunk))
            {
                _term.Feed(chunk);
                Log.Console(chunk);
            }
        RenderConsole();

        UpdateCaptureHint();
        FlushMouse();
        if (_captured && s is not { Running: true })
            ReleasePointer();   // the machine stopped under us

        if (s == null)
            return;

        lock (s.FrameLock)
        {
            if (s.FrameDirty)
            {
                s.FrameDirty = false;
                int w = s.FrameW, h = s.FrameH;
                if (_bmp == null || _bmp.PixelSize.Width != w || _bmp.PixelSize.Height != h)
                {
                    _bmp?.Dispose();
                    _bmp = new WriteableBitmap(new PixelSize(w, h), new Vector(96, 96),
                                               PixelFormat.Bgra8888, AlphaFormat.Opaque);
                    imgScreen.Source = _bmp;
                    pnlNoVideo.IsVisible = false;
                    txtVideo.Text = $"{w}×{h}";
                }
                using (ILockedFramebuffer fb = _bmp.Lock())
                {
                    int stride = w * 4;
                    if (fb.RowBytes == stride)
                        Marshal.Copy(s.Frame, 0, fb.Address, stride * h);
                    else
                        for (int y = 0; y < h; y++)
                            Marshal.Copy(s.Frame, y * stride, fb.Address + y * fb.RowBytes, stride);
                }
                imgScreen.InvalidateVisual();
            }
        }

        long now = Environment.TickCount64;
        if (now - _statsShownAt < 250)
            return;
        _statsShownAt = now;
        long executed = Interlocked.Read(ref s.StatExecuted);
        txtInsns.Text = executed >= 1_000_000_000
            ? $"{executed / 1e9:F2} B insns"
            : $"{executed / 1e6:F1} M insns";
        long mipsX10 = Interlocked.Read(ref s.StatMipsX10);
        txtMips.Text = mipsX10 > 0 ? $"{mipsX10 / 10.0:F1} MIPS" : "";
        txtPc.Text = $"pc {s.StatPc:X8}";
        // Dropped frames are shown because a stutter and a silent machine are
        // otherwise the same observation.
        long dropped = Interlocked.Read(ref s.StatAudioDropped);
        long starved = Interlocked.Read(ref s.StatAudioStarved);
        string audioNote = starved > 0 ? $" ({starved:N0} starved)"
                         : dropped > 0 ? $" ({dropped:N0} dropped)"
                         : "";
        txtAudio.Text = s.StatAudioRate == 0 ? "" : $"audio {s.StatAudioRate / 1000.0:F1} kHz{audioNote}";
    }

    // ---- file pickers ----

    private void Notice(string text, string caption, MessageBoxImage image = MessageBoxImage.Information) =>
        MessageBox.Show(this, text, caption, MessageBoxButton.OK, image);

    private bool ChooseRom()
    {
        string? path = FilePicker.Open(this, "Choose the Sawtooth Boot ROM (1 MB)",
                                       "Boot ROM (*.rom)|*.rom|All files (*.*)|*.*", _settings.RomPath);
        if (path == null)
            return false;
        _settings.RomPath = path;
        _settings.Save();
        UpdateTitle();
        return true;
    }

    private void OnChooseRom(object? sender, RoutedEventArgs e) => ChooseRom();

    private void OnChooseCd(object? sender, RoutedEventArgs e)
    {
        string? path = FilePicker.Open(this, "Choose a CD image (ISO/Toast/CDR, cue+bin, or UDIF dmg)",
            "CD images (*.iso;*.toast;*.img;*.cdr;*.cue;*.bin;*.dmg;*.mdf)"
            + "|*.iso;*.toast;*.img;*.cdr;*.cue;*.bin;*.dmg;*.mdf|All files (*.*)|*.*",
            _settings.CdPath);
        if (path == null)
            return;
        // A running machine follows this too: the disc goes into its drive and
        // Mac OS mounts it, the way it would a disc put in the tray.
        _settings.CdPath = path;
        _settings.Save();
        UpdateTitle();
    }

    // Eject rather than "detach": a Mac with a bootable CD in the drive boots
    // the CD, so taking it out is how you tell it to boot the disk.
    private void OnEjectCd(object? sender, RoutedEventArgs e)
    {
        // A running machine follows this: the disc comes out now unless Mac OS
        // has it mounted, and the console says which (MachineSession.Run).
        _settings.CdPath = "";
        _settings.Save();
        UpdateTitle();
    }

    private void OnChooseShared(object? sender, RoutedEventArgs e)
    {
        string? folder = FilePicker.OpenFolder(this,
            "Choose a folder to share with the guest (mounted read-only at every start)",
            _settings.SharedFolderPath);
        if (folder == null)
            return;
        _settings.SharedFolderPath = folder;
        _settings.Save();
        UpdateTitle();
        if (_session is { Running: true })
            Notice("Shared folder set.\n\nThe volume is built when the machine starts, so it appears "
                   + "the next time you start it.", "Shared Folder");
    }

    private void OnClearShared(object? sender, RoutedEventArgs e)
    {
        _settings.SharedFolderPath = "";
        _settings.Save();
        UpdateTitle();
    }

    // The drop box. Files dropped on the display go to the guest without a
    // restart; see MachineSession.RequestDrop for the hand-over.
    private void OnDisplayDragOver(object? sender, DragEventArgs e)
    {
        bool ok = _session is { Running: true } && e.DataTransfer.Contains(DataFormat.File);
        e.DragEffects = ok ? DragDropEffects.Copy : DragDropEffects.None;
        e.Handled = true;
    }

    private void OnDisplayDrop(object? sender, DragEventArgs e)
    {
        e.Handled = true;
        if (e.DataTransfer.TryGetFiles() is not { } items)
            return;
        var paths = items.Select(i => i.TryGetLocalPath()).OfType<string>().ToList();
        if (paths.Count == 0)
            return;
        if (_session is not { Running: true } s)
        {
            Notice("Start the machine first — the drop box hands files to a running guest.", "Drop Box");
            return;
        }
        s.RequestDrop(paths);
    }

    private void OnOpenDropFolder(object? sender, RoutedEventArgs e)
    {
        string folder = MachineSession.DropBoxFolder(_settings);
        // Process start and folder creation are file work: off the UI thread.
        Task.Run(() =>
        {
            try
            {
                Directory.CreateDirectory(folder);
                var psi = new ProcessStartInfo("xdg-open") { UseShellExecute = false };
                psi.ArgumentList.Add(folder);
                Process.Start(psi)?.Dispose();
            }
            catch (Exception ex)
            {
                Dispatcher.UIThread.Post(() =>
                    Notice($"Could not open {folder}\n\n{ex.Message}", "Drop Box", MessageBoxImage.Warning));
            }
        });
    }

    private void OnChooseDropFolder(object? sender, RoutedEventArgs e)
    {
        string? folder = FilePicker.OpenFolder(this,
            "Choose the drop box folder (dropped files are copied here and handed to the guest)",
            MachineSession.DropBoxFolder(_settings));
        if (folder == null)
            return;
        _settings.DropBoxPath = folder;
        _settings.Save();
        NoteDropFolderChange();
    }

    private void OnDefaultDropFolder(object? sender, RoutedEventArgs e)
    {
        _settings.DropBoxPath = "";
        _settings.Save();
        NoteDropFolderChange();
    }

    private void NoteDropFolderChange()
    {
        if (_session is { Running: true })
            Notice("Drop box folder set.\n\nThe running machine keeps using the folder it started with; "
                   + "the new one is used from the next start.", "Drop Box");
    }

    private void OnDiagnostics(object? sender, RoutedEventArgs e)
    {
        if (_session is { Running: true } s)
            s.RequestDiagnostics();
        else
            Notice("The machine is not running.", "Diagnostics");
    }

    private void OnSaveSnapshot(object? sender, RoutedEventArgs e)
    {
        if (_session is not { Running: true } s)
        {
            Notice("The machine is not running.", "Snapshot");
            return;
        }
        string? path = FilePicker.Save(this, "Save a snapshot of this exact moment",
            "Machine snapshot (*.snap)|*.snap|All files (*.*)|*.*", "moment.snap", ".snap");
        if (path != null)
            s.RequestSnapshot(path);
    }

    private void OnChooseAti(object? sender, RoutedEventArgs e)
    {
        string? path = FilePicker.Open(this, "Choose the ATI Rage 128 Pro FCode ROM",
            "Card ROM (*.rom;*.bin)|*.rom;*.bin|All files (*.*)|*.*", _settings.AtiRomPath);
        if (path == null)
            return;
        // Refused here, at the moment of the mistake: a wrong file in this
        // slot gives a machine that boots and shows nothing. Reading the file
        // is file work, so it happens off the UI thread.
        string why = "";
        if (!ModalLoop.Wait(Task.Run(() => MachineSession.LooksLikeCardRom(path, out why))))
        {
            Notice($"{Path.GetFileName(path)} is not a graphics card ROM: {why}.\n\n"
                   + "Choose the ATI Rage 128 Pro card's own ROM (about 64 to 128 KB). "
                   + "The 1 MB boot ROM goes in File ▸ Choose Boot ROM… instead.",
                   "ATI card ROM", MessageBoxImage.Warning);
            return;
        }
        _settings.AtiRomPath = path;
        _settings.Save();
        UpdateTitle();
        UpdateNoVideoHint();
    }

    private const string HintReady =
        "Machine ▸ Start boots the Sawtooth ROM.\nThe firmware talks on the serial console below until the display comes up.";

    /// <summary>What the empty screen area says. With no usable card ROM the
    /// display never comes up, and a machine that is running blind must not
    /// look like one that is merely still booting.</summary>
    private void UpdateNoVideoHint()
    {
        string ati = _settings.AtiRomPath;
        if (string.IsNullOrWhiteSpace(ati))
        {
            txtHint.Text = "No ATI card ROM is chosen, so the machine will have no picture.\n"
                           + "File ▸ Choose ATI Card ROM… (the Rage 128 Pro card's own ROM).";
            return;
        }
        txtHint.Text = HintReady;
        // The check reads the file; do it off the UI thread and come back.
        Task.Run(() => MachineSession.LooksLikeCardRom(ati, out string why) ? null : why)
            .ContinueWith(t =>
            {
                if (t.Result is { } why && ati == _settings.AtiRomPath)
                    txtHint.Text = $"The ATI card ROM is not a card ROM ({why}),\nso the machine will have no picture. "
                                   + "File ▸ Choose ATI Card ROM…";
            }, TaskScheduler.FromCurrentSynchronizationContext());
    }

    // ---- hard disks ----
    //
    // The machine opens its disk at opm_create, so everything here decides
    // what the NEXT start boots.

    private readonly List<MenuItem> _hdRecentItems = new();

    private static string DescribeMb(long mb) =>
        mb >= 1024 && mb % 1024 == 0 ? $"{mb / 1024} GB" : $"{mb} MB";

    private static string DescribeBytes(long bytes) =>
        bytes >= 1L << 30 ? $"{bytes / (double)(1L << 30):0.#} GB"
                          : $"{bytes / (double)(1L << 20):0.#} MB";

    private static string DescribeDisk(string path)
    {
        try
        {
            var fi = new FileInfo(path);
            return fi.Exists ? $"{fi.Name}  ({DescribeBytes(fi.Length)})"
                             : $"{Path.GetFileName(path)}  (missing)";
        }
        catch { return Path.GetFileName(path); }
    }

    private void OnHardDiskOpened(object? sender, RoutedEventArgs e)
    {
        // Submenus below this one raise the same event on the way up.
        if (!ReferenceEquals(e.Source, miHd))
            return;
        foreach (var mi in _hdRecentItems)
            miHd.Items.Remove(mi);
        _hdRecentItems.Clear();

        int at = miHd.Items.IndexOf(miHdSep) + 1;
        foreach (string path in _settings.RecentHds)
        {
            string p = path;
            var mi = new MenuItem
            {
                Header = DescribeDisk(p),
                ToggleType = MenuItemToggleType.CheckBox,
                IsChecked = string.Equals(p, _settings.HdPath, StringComparison.Ordinal),
            };
            ToolTip.SetTip(mi, p);
            mi.Click += (_, _) => AttachHd(p, "attached");
            miHd.Items.Insert(at++, mi);
            _hdRecentItems.Add(mi);
        }
        miHdSep.IsVisible = _settings.RecentHds.Count > 0;
        miHdDetach.IsEnabled = !string.IsNullOrWhiteSpace(_settings.HdPath);
    }

    private void OnNewHd(object? sender, RoutedEventArgs e)
    {
        if (sender is not MenuItem { Tag: string tag } || !long.TryParse(tag, out long mb))
            return;
        string? path = FilePicker.Save(this, $"New {DescribeMb(mb)} hard disk image",
            "Disk images (*.img)|*.img|All files (*.*)|*.*",
            $"macos9_{(mb >= 1024 && mb % 1024 == 0 ? $"{mb / 1024}gb" : $"{mb}mb")}.img", ".img",
            _settings.HdPath,
            // Ours below, because the desktop's prompt does not say how big the
            // thing it is about to destroy is.
            overwritePrompt: false);
        if (path == null)
            return;
        if (File.Exists(path))
        {
            long existing;
            try { existing = new FileInfo(path).Length; } catch { existing = 0; }
            if (MessageBox.Show(this,
                    $"{Path.GetFileName(path)} already exists ({DescribeBytes(existing)}).\n\n"
                    + $"Replace it with a blank {DescribeMb(mb)} disk? Anything installed on it is lost.",
                    "Replace disk image?", MessageBoxButton.YesNo, MessageBoxImage.Warning)
                != MessageBoxResult.Yes)
                return;
        }
        try
        {
            // SPARSE, as on Windows, and for the same reasons: creation is
            // instant at 120 GB, and the guest's first write at the END of the
            // disk (HFS+'s alternate volume header) has nothing to zero-fill
            // before it. On ext4, btrfs and xfs a length set without writing is
            // a hole, so SetLength alone is the whole job here.
            using var fs = new FileStream(path, FileMode.Create, FileAccess.Write);
            fs.SetLength(mb * 1024L * 1024L);
        }
        catch (Exception ex)
        {
            Notice($"Could not create the disk image:\n\n{ex.Message}", "New disk image", MessageBoxImage.Error);
            return;
        }
        // A sparse image is a promise the host may not keep: if the volume runs
        // out while Mac OS writes, the guest sees a damaged disk. Say so once.
        try
        {
            DriveInfo? drive = DriveFor(path);
            long want = mb * 1024L * 1024L;
            if (drive != null && drive.AvailableFreeSpace < want)
                Notice($"The disk is {DescribeMb(mb)} but {drive.RootDirectory.FullName} has only "
                       + $"{DescribeBytes(drive.AvailableFreeSpace)} free.\n\n"
                       + "The image is sparse, so it only uses what the guest actually writes — this is "
                       + "fine unless the guest fills it. If this volume runs out while Mac OS is writing, "
                       + "the guest sees it as a damaged disk.", "New disk image");
        }
        catch { /* a courtesy; failing to give it is not fatal */ }
        AttachHd(path, $"created a blank {DescribeMb(mb)} disk and attached");
    }

    /// <summary>The mounted volume a path lives on: the longest mount point
    /// that prefixes it. (DriveInfo of the path's root is always "/".)</summary>
    private static DriveInfo? DriveFor(string path)
    {
        string full = Path.GetFullPath(path);
        return DriveInfo.GetDrives()
            .Where(d => d.IsReady && full.StartsWith(d.RootDirectory.FullName.TrimEnd('/') + "/", StringComparison.Ordinal))
            .OrderByDescending(d => d.RootDirectory.FullName.Length)
            .FirstOrDefault();
    }

    private void OnChooseHd(object? sender, RoutedEventArgs e)
    {
        string? path = FilePicker.Open(this, "Choose a hard disk image",
            "Disk images (*.img;*.dsk;*.hfv)|*.img;*.dsk;*.hfv|All files (*.*)|*.*", _settings.HdPath);
        if (path != null)
            AttachHd(path, "attached");
    }

    private void OnDetachHd(object? sender, RoutedEventArgs e) => AttachHd("", "detached the hard disk");

    private void AttachHd(string path, string what)
    {
        _settings.AttachHd(path);
        _settings.Save();
        UpdateTitle();
        if (_session is { Running: true })
            Notice($"Shell {what}.\n\nThe machine is running, and it opened its disk when it started — "
                   + "this takes effect the next time you start it.", "Hard disk");
    }

    private void OnAutoBootToggled(object? sender, RoutedEventArgs e)
    {
        _settings.AutoBoot = miAutoBoot.IsChecked;
        _settings.Save();
    }

    private void OnMemoryOpened(object? sender, RoutedEventArgs e)
    {
        if (!ReferenceEquals(e.Source, miRam))
            return;
        foreach (var item in miRam.Items.OfType<MenuItem>())
            item.IsChecked = item.Tag is string t && uint.TryParse(t, out uint mb) && mb == _settings.RamMb;
    }

    private void OnSetRam(object? sender, RoutedEventArgs e)
    {
        if (sender is not MenuItem { Tag: string tag } || !uint.TryParse(tag, out uint mb) || mb == _settings.RamMb)
            return;
        _settings.RamMb = mb;
        _settings.Save();
        if (_session is { Running: true })
            Notice($"Memory set to {(mb >= 1024 ? $"{mb / 1024.0:0.#} GB" : $"{mb} MB")}.\n\n"
                   + "The machine is running, and it sized its DIMMs when it started — this takes effect "
                   + "the next time you start it.", "Memory");
    }

    // Takes effect on the next Start; the codec is drained either way.
    private void OnSoundToggled(object? sender, RoutedEventArgs e)
    {
        _settings.Sound = miSound.IsChecked;
        _settings.Save();
    }

    private void OnExit(object? sender, RoutedEventArgs e) => Close();

    private void UpdateTitle()
    {
        string t = "OpenPowerMac";
        if (!string.IsNullOrWhiteSpace(_settings.RomPath))
            t += " — " + Path.GetFileNameWithoutExtension(_settings.RomPath);
        if (!string.IsNullOrWhiteSpace(_settings.CdPath))
            t += " + " + Path.GetFileName(_settings.CdPath);
        t += string.IsNullOrWhiteSpace(_settings.HdPath)
                 ? "  [no HD]"
                 : "  [" + Path.GetFileName(_settings.HdPath) + "]";
        Title = t;
    }
}
