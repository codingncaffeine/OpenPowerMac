using System.Collections.Concurrent;
using System.Diagnostics;
using System.Text;

namespace OpenPowerMac.Shell;

/// <summary>Owns the machine and its run thread. Every capi call happens on
/// the worker — the UI talks through queues and published snapshots, so the
/// single-threaded core never sees a cross-thread call.</summary>
public sealed class MachineSession
{
    private Thread? _thread;
    private volatile bool _stop;
    private readonly ConcurrentQueue<string> _serialQ = new();
    private readonly ConcurrentQueue<string> _keyQ = new();
    private readonly ConcurrentQueue<(int dx, int dy, uint buttons)> _mouseQ = new();

    /// <summary>Raw console bytes drained from the machine (VT stream).</summary>
    public readonly ConcurrentQueue<string> ConsoleQ = new();

    // Latest video frame, guarded by FrameLock. Sized for the largest mode
    // the CRTC reports (2048x1536 BGRA) so it never reallocates.
    public readonly object FrameLock = new();
    public readonly byte[] Frame = new byte[2048 * 1536 * 4];
    public int FrameW, FrameH;
    public bool FrameDirty;

    // Stats published after every chunk (read racily by the UI timer).
    public long StatExecuted;
    public long StatMipsX10;
    public int StatPc;
    public volatile bool Running;

    /// <summary>Raised on the worker when the session ends (reason text).</summary>
    public event Action<string>? Ended;

    public bool Start(ShellSettings s)
    {
        if (_thread != null)
            return false;
        _stop = false;
        _thread = new Thread(() => Run(s)) { IsBackground = true, Name = "opm-run" };
        _thread.Start();
        return true;
    }

    public void Stop() => _stop = true;

    /// <summary>Queue a line for the serial console. ';' splits into
    /// separate lines (the Forth-bridge convention); a CR is appended.</summary>
    public void SendSerial(string line) =>
        _serialQ.Enqueue(line.Replace(';', '\r') + "\r");

    /// <summary>Queue typed text for the USB keyboard (usb@8).</summary>
    public void SendKeys(string text) => _keyQ.Enqueue(text);

    /// <summary>Queue a USB mouse report (usb@9): relative motion plus the
    /// button mask held at the time. Every capi call runs on the worker, so
    /// UI events only ever enqueue.</summary>
    public void SendMouse(int dx, int dy, uint buttons) =>
        _mouseQ.Enqueue((dx, dy, buttons));

    private void Run(ShellSettings s)
    {
        Running = true;
        IntPtr m = IntPtr.Zero;
        string reason = "stopped";
        try
        {
            m = Native.opm_create(s.RomPath,
                                  string.IsNullOrWhiteSpace(s.CdPath) ? null : s.CdPath,
                                  string.IsNullOrWhiteSpace(s.HdPath) ? null : s.HdPath,
                                  string.IsNullOrWhiteSpace(s.AtiRomPath) ? null : s.AtiRomPath,
                                  s.RamMb, s.FastTb);
            if (m == IntPtr.Zero)
            {
                ConsoleQ.Enqueue("\n[shell] opm_create failed — check the ROM path (1 MB Sawtooth Boot ROM)\n");
                reason = "create failed";
                return;
            }
            if (!string.IsNullOrWhiteSpace(s.AtiRomPath) && s.AtiAt > 0)
                Native.opm_ati_at(m, s.AtiAt);

            // Report the hard disk too. Without it in this line there was no
            // way to tell a machine booting from disk apart from one that
            // never had a disk at all — and that is the whole question when
            // the screen shows a flashing "?" folder.
            ConsoleQ.Enqueue($"[shell] machine up — {s.RamMb} MB, fast-tb {s.FastTb}" +
                             (string.IsNullOrWhiteSpace(s.HdPath) ? ", NO HD" : ", HD attached") +
                             (string.IsNullOrWhiteSpace(s.CdPath) ? "" : ", CD attached") +
                             (string.IsNullOrWhiteSpace(s.AtiRomPath) ? "" : $", ATI at {s.AtiAt / 1_000_000}M") + "\n");

            bool scriptPending = s.AutoBoot && !string.IsNullOrWhiteSpace(s.BootScript);
            var conBuf = new byte[65536];
            var shot = new byte[Frame.Length];
            ulong chunk = 1_000_000;
            var sw = Stopwatch.StartNew();
            long lastShotMs = 0, lastStatMs = 0, lastStatInsns = 0;
            ulong executed = 0;

            while (!_stop)
            {
                long t0 = sw.ElapsedMilliseconds;
                ulong before = executed;
                executed = Native.opm_run(m, chunk);
                long dt = sw.ElapsedMilliseconds - t0;

                if (executed == before)
                {
                    ConsoleQ.Enqueue($"\n[shell] machine halted @{executed:N0}\n");
                    reason = "halted";
                    break;
                }

                // Aim each chunk at ~15 ms so serial input and stop stay snappy.
                if (dt < 8) chunk = Math.Min(chunk * 2, 40_000_000);
                else if (dt > 30) chunk = Math.Max(chunk / 2, 100_000);

                while (_serialQ.TryDequeue(out var text))
                    Native.opm_serial(m, text);

                while (_keyQ.TryDequeue(out var keys))
                    Native.opm_key(m, keys);

                while (_mouseQ.TryDequeue(out var mv))
                    Native.opm_mouse(m, mv.dx, mv.dy, mv.buttons);

                if (scriptPending && executed >= s.ScriptAt)
                {
                    Native.opm_serial(m, s.BootScript.Replace(';', '\r') + "\r");
                    scriptPending = false;
                    ConsoleQ.Enqueue($"\n[shell] boot script injected @{executed:N0}\n");
                }

                uint n;
                while ((n = Native.opm_console(m, conBuf, (uint)conBuf.Length)) > 0)
                {
                    ConsoleQ.Enqueue(Encoding.Latin1.GetString(conBuf, 0, (int)n));
                    if (n + 1 < conBuf.Length)
                        break;
                }

                long now = sw.ElapsedMilliseconds;
                if (now - lastShotMs >= 33)
                {
                    lastShotMs = now;
                    int r = Native.opm_screen(m, shot, (uint)shot.Length, out uint w, out uint h);
                    if (r == 1)
                    {
                        lock (FrameLock)
                        {
                            Buffer.BlockCopy(shot, 0, Frame, 0, (int)(w * h * 4));
                            FrameW = (int)w;
                            FrameH = (int)h;
                            FrameDirty = true;
                        }
                    }
                }

                Interlocked.Exchange(ref StatExecuted, (long)executed);
                Interlocked.Exchange(ref StatPc, (int)Native.opm_pc(m));
                if (now - lastStatMs >= 500)
                {
                    long insns = (long)executed - lastStatInsns;
                    Interlocked.Exchange(ref StatMipsX10,
                        now > lastStatMs ? insns / (now - lastStatMs) / 100 : 0);
                    lastStatMs = now;
                    lastStatInsns = (long)executed;
                }
            }
        }
        finally
        {
            if (m != IntPtr.Zero)
                Native.opm_destroy(m);
            Running = false;
            _thread = null;
            Ended?.Invoke(reason);
        }
    }
}
