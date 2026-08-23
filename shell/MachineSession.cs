using System.Collections.Concurrent;
using System.Diagnostics;
using System.IO;
using System.Text;

namespace OpenPowerMac.Shell;

/// <summary>Owns the machine and its run thread. Every capi call happens on
/// the worker — the UI talks through queues and published snapshots, so the
/// single-threaded core never sees a cross-thread call.</summary>
public sealed class MachineSession
{
    private Thread? _thread;
    private volatile bool _stop;
    // Owned and touched only by the worker, like every other capi resource.
    private readonly WaveOutSink _wave = new();
    // OPM_AUDIO_WAV=path: record what the APP played, which is a different
    // timeline from what g4run's --wav-out records. See AudioCapture.
    private AudioCapture? _capture;
    private readonly ConcurrentQueue<string> _serialQ = new();
    private readonly ConcurrentQueue<string> _keyQ = new();
    private readonly ConcurrentQueue<(uint usage, uint down)> _keyEvQ = new();
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
    /// <summary>Sample rate the guest's codec is running at (0 = silent).</summary>
    public int StatAudioRate;
    /// <summary>Frames the host sink could not take. Non-zero means a stutter
    /// the user can hear, which is otherwise indistinguishable from a guest
    /// that simply stopped playing.</summary>
    public long StatAudioDropped;
    /// <summary>Times the speaker ran dry in the middle of a sound. This is
    /// the number that says a pop came from starvation rather than from the
    /// samples, and nothing could tell them apart before it existed.</summary>
    public long StatAudioStarved;

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

    /// <summary>Queue one key going down or coming up, as a HID usage code —
    /// what a person's keyboard actually sends. Usages 0xE0-0xE7 are the
    /// modifiers; the guest applies its own layout to the rest.</summary>
    public void SendKeyEvent(uint usage, bool down) =>
        _keyEvQ.Enqueue((usage, down ? 1u : 0u));

    /// <summary>Queue a USB mouse report (usb@9): relative motion plus the
    /// button mask held at the time. Every capi call runs on the worker, so
    /// UI events only ever enqueue.</summary>
    public void SendMouse(int dx, int dy, uint buttons) =>
        _mouseQ.Enqueue((dx, dy, buttons));

    // 🩺 Ask for a state report. ⚠ It is a REQUEST rather than a call because
    // the machine belongs to the run thread: reading its devices from the UI
    // thread while that thread is inside opm_run is a data race, and the one
    // moment anybody wants this is the moment the machine looks stuck — which
    // is exactly when racing it would turn a diagnosable stall into a crash.
    // The loop honours it between chunks and the text arrives on ConsoleQ.
    private volatile bool _diagWanted;
    // Sample A, held while the machine runs one more chunk so B can be
    // compared against it. See the capture site for why one sample is not
    // enough.
    private bool _diagPending;
    private string _diagA = "";
    private ulong _diagAInsns;
    private uint _diagAPc;
    public void RequestDiagnostics() => _diagWanted = true;

    // 📸 Save the machine, same request discipline and the same reason: the
    // machine belongs to the run thread. Serialising it from the UI thread
    // mid-chunk would race every device in it.
    //
    // ⭐ What this is FOR: a rendering defect that only exists inside a
    // running game costs a human launch per iteration — boot, mount,
    // launch, play to the broken frame. One snapshot at that frame hands
    // the whole reproduction to the headless harness, and every later
    // iteration is a resume measured in seconds.
    private volatile string? _snapWanted;
    public void RequestSnapshot(string path) => _snapWanted = path;

    private void Run(ShellSettings s)
    {
        Running = true;
        IntPtr m = IntPtr.Zero;
        string reason = "stopped";
        // The packed shared-folder image, when this Start built one — a full
        // copy of the folder's content, so it is deleted again at stop.
        string? sharedImg = null;
        try
        {
            // 📁 The shared folder rides the CD slot: packed fresh into a
            // classic HFS image at every Start, so the folder is the single
            // source of truth and the guest mounts it like an inserted
            // disc. An explicitly chosen CD image keeps the slot — the
            // console says which happened either way.
            string? cdPath =
                string.IsNullOrWhiteSpace(s.CdPath) ? null : s.CdPath;
            if (!string.IsNullOrWhiteSpace(s.SharedFolderPath))
            {
                if (cdPath != null)
                {
                    ConsoleQ.Enqueue("[shell] shared folder skipped — the "
                                     + "CD slot has an image; eject the CD "
                                     + "to mount the folder\n");
                }
                else
                {
                    var img = Path.Combine(Path.GetTempPath(),
                                           "opm_shared.img");
                    var volName = Path.GetFileName(
                        s.SharedFolderPath.TrimEnd('\\', '/'));
                    if (string.IsNullOrWhiteSpace(volName))
                        volName = "Shared";
                    var err = new StringBuilder(512);
                    if (Native.opm_hfs_build(s.SharedFolderPath, img,
                                             volName, err, 512) != 0)
                    {
                        cdPath = img;
                        sharedImg = img;
                        long mb = 0;
                        try { mb = new FileInfo(img).Length >> 20; }
                        catch { }
                        ConsoleQ.Enqueue(
                            $"[shell] shared folder mounted read-only as "
                            + $"“{volName}” ({mb} MB image): "
                            + $"{s.SharedFolderPath}\n");
                        // On success the buffer carries warnings — files
                        // the share had to leave out.
                        if (err.Length > 0)
                            ConsoleQ.Enqueue($"[shell] shared folder: {err}\n");
                    }
                    else
                    {
                        ConsoleQ.Enqueue("[shell] shared folder NOT mounted: "
                                         + $"{err}\n");
                        // A refused build must not leave an earlier run's
                        // image behind to be mistaken for this folder.
                        try { File.Delete(img); } catch { }
                    }
                }
            }
            m = Native.opm_create(s.RomPath,
                                  cdPath,
                                  string.IsNullOrWhiteSpace(s.HdPath) ? null : s.HdPath,
                                  string.IsNullOrWhiteSpace(s.AtiRomPath) ? null : s.AtiRomPath,
                                  s.RamMb, s.FastTb);
            if (m == IntPtr.Zero)
            {
                ConsoleQ.Enqueue("\n[shell] opm_create failed — check the ROM path (1 MB Sawtooth Boot ROM)\n");
                reason = "create failed";
                return;
            }
            // A ROM dump assembled from Apple's firmware updater carries a
            // template system-configuration block that describes some other
            // board; the core factory-configures it as a Sawtooth and says
            // so here, because the alternative was a machine that sat at
            // power-on forever with nothing on the console explaining why.
            {
                var noteBuf = new byte[1024];
                uint noteLen = Native.opm_rom_note(m, noteBuf, (uint)noteBuf.Length);
                if (noteLen > 0)
                    ConsoleQ.Enqueue("[shell] "
                        + System.Text.Encoding.UTF8.GetString(noteBuf, 0, (int)noteLen)
                        + "\n");
            }
            if (!string.IsNullOrWhiteSpace(s.AtiRomPath) && s.AtiAt > 0)
                Native.opm_ati_at(m, s.AtiAt);

            // Pace the timebase from the host clock. This is what makes the
            // guest's clock true — see ShellSettings.Realtime — and it also
            // makes the machine independent of how fast this host happens to
            // be, which instruction pacing cannot be.
            if (s.Realtime)
                Native.opm_set_realtime(m, 1);

            // Report the CD from the DRIVE, not from the setting: attach can
            // refuse the image (a cue sheet naming a missing .bin, a .dmg in
            // a codec the core does not carry), and a status line keyed off
            // the path would report a CD the machine never got.
            string cdNote = "";
            if (cdPath != null)
            {
                if (Native.opm_cd_present(m) != 0)
                {
                    cdNote = ", CD attached";
                }
                else
                {
                    var cdErr = new byte[512];
                    uint cdErrLen = Native.opm_cd_error(m, cdErr, (uint)cdErr.Length);
                    ConsoleQ.Enqueue("[shell] CD attach REFUSED: "
                                     + System.Text.Encoding.UTF8.GetString(cdErr, 0, (int)cdErrLen)
                                     + $"\n[shell]   image: {cdPath}\n");
                    cdNote = ", CD REFUSED (empty drive)";
                }
            }

            // Report the hard disk too. Without it in this line there was no
            // way to tell a machine booting from disk apart from one that
            // never had a disk at all — and that is the whole question when
            // the screen shows a flashing "?" folder.
            ConsoleQ.Enqueue($"[shell] machine up — {s.RamMb} MB, " +
                             (s.Realtime ? "clock paced from the host (25 MHz)"
                                         : $"fast-tb {s.FastTb}") +
                             (string.IsNullOrWhiteSpace(s.HdPath) ? ", NO HD" : ", HD attached") +
                             cdNote +
                             // AtiAt is the instruction at which the card
                             // becomes visible in PCI config space, and
                             // printing it in millions turned the shipping
                             // value — 1, meaning visible from reset, which
                             // is what you want — into "ATI at 0M". That
                             // reads as a card sitting idle, and it is not
                             // what the number means.
                             (string.IsNullOrWhiteSpace(s.AtiRomPath) ? ""
                              : s.AtiAt <= 1 ? ", ATI from reset"
                              : $", ATI hidden until {s.AtiAt:N0}") + "\n");

            bool scriptPending = s.AutoBoot && !string.IsNullOrWhiteSpace(s.BootScript);
            var conBuf = new byte[65536];
            var shot = new byte[Frame.Length];
            // A quarter of a second of 44.1 kHz stereo, which is more than the
            // codec can have produced between two chunks — the drain is never
            // the thing that falls behind.
            var pcm = new byte[44100 * 4 / 4];
            _capture = AudioCapture.FromEnvironment();
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

                // ⛔ ASK THE MACHINE, DO NOT INFER IT. This used to read
                // `executed == before`, and that stopped being a halt the
                // moment the run loop learned to wait: an idle guest at the
                // Finder legitimately executes NOTHING for a whole call,
                // because opm_run spends the time off the processor waiting
                // for the guest's next deadline. Inferring from the
                // instruction count would have stopped the machine the first
                // time the user left it sitting at the desktop.
                if (Native.opm_halted(m) != 0)
                {
                    ConsoleQ.Enqueue($"\n[shell] machine halted @{executed:N0}\n");
                    reason = "halted";
                    break;
                }

                // Aim each chunk at ~15 ms so serial input and stop stay snappy.
                // ⚠ Only grow it on a call that actually RAN. An idle call
                // returns after its own wait bound having executed nothing, and
                // treating that as "there was time to spare" would ratchet the
                // chunk to its ceiling for a machine doing no work at all.
                if (dt < 8 && executed != before) chunk = Math.Min(chunk * 2, 40_000_000);
                else if (dt > 30) chunk = Math.Max(chunk / 2, 100_000);

                // Between chunks, never during one — see RequestDiagnostics.
                // ⭐ TWO SAMPLES, ONE CLICK. A single capture cannot say whether
                // a stopped-looking machine is spinning or truly stopped, and
                // asking a person to click twice at the right moment is asking
                // them to remember it during the one run that reproduced the
                // bug — which each cost about ten minutes. So the first request
                // takes sample A and arms a second; the next trip round the
                // loop takes B, and the file carries both plus the delta.
                // Between chunks, for the reason above: the snapshot walks
                // every device in the machine.
                if (_snapWanted is string snapPath)
                {
                    _snapWanted = null;
                    var eb = new byte[512];
                    int ok = Native.opm_snapshot_save(
                        m, snapPath, eb, (uint)eb.Length);
                    if (ok == 1)
                    {
                        long bytes = 0;
                        try { bytes = new FileInfo(snapPath).Length; }
                        catch { }
                        ConsoleQ.Enqueue(
                            $"\r\n[snapshot saved at {Native.opm_executed(m)}" +
                            $" instructions: {snapPath} ({bytes / 1048576} MB)]\r\n");
                    }
                    else
                    {
                        int errLen = Array.IndexOf(eb, (byte)0);
                        ConsoleQ.Enqueue("\r\n[snapshot FAILED: " +
                            System.Text.Encoding.ASCII.GetString(
                                eb, 0, errLen < 0 ? eb.Length : errLen) +
                            "]\r\n");
                    }
                }
                if (_diagWanted)
                {
                    _diagWanted = false;
                    var ab = new byte[65536];
                    uint an = Native.opm_diag(m, ab, (uint)ab.Length);
                    _diagA = System.Text.Encoding.ASCII.GetString(ab, 0, (int)an);
                    _diagAInsns = Native.opm_executed(m);
                    _diagAPc = Native.opm_pc(m);
                    _diagPending = true;
                }
                else if (_diagPending)
                {
                    _diagPending = false;
                    var db = new byte[65536];
                    uint dn = Native.opm_diag(m, db, (uint)db.Length);
                    ulong bInsns = Native.opm_executed(m);
                    uint bPc = Native.opm_pc(m);
                    string verdict = bInsns == _diagAInsns
                        ? "STOPPED DEAD — the instruction count did not move between samples"
                        : $"RUNNING — {bInsns - _diagAInsns:N0} instructions between samples"
                          + (bPc == _diagAPc ? ", same pc (a tight loop)" : ", pc moved");
                    string text =
                        $"[sample A] pc={_diagAPc:x8} executed={_diagAInsns:N0}\n"
                        + _diagA
                        + $"\n[sample B] pc={bPc:x8} executed={bInsns:N0}\n"
                        + System.Text.Encoding.ASCII.GetString(db, 0, (int)dn)
                        + $"\n===== VERDICT: {verdict} =====\n";
                    ConsoleQ.Enqueue(text);
                    // ⭐ AND TO A FILE. A wedge is reported by someone reading a
                    // scrolling text pane and retyping what they see, which is
                    // the step where the useful half gets dropped. Writing it
                    // out means the report can be handed over whole.
                    try
                    {
                        string dir = Path.Combine(
                            Environment.GetFolderPath(
                                Environment.SpecialFolder.LocalApplicationData),
                            "OpenPowerMac");
                        Directory.CreateDirectory(dir);
                        string path = Path.Combine(
                            dir, $"diag-{DateTime.Now:yyyyMMdd-HHmmss}.txt");
                        File.WriteAllText(path, text);
                        ConsoleQ.Enqueue($"[shell] diagnostics written to {path}\n");
                    }
                    catch (Exception ex)
                    {
                        ConsoleQ.Enqueue($"[shell] could not write diagnostics file: {ex.Message}\n");
                    }
                }

                while (_serialQ.TryDequeue(out var text))
                    Native.opm_serial(m, text);

                while (_keyQ.TryDequeue(out var keys))
                    Native.opm_key(m, keys);

                while (_keyEvQ.TryDequeue(out var ke))
                    Native.opm_key_event(m, ke.usage, ke.down);

                while (_mouseQ.TryDequeue(out var mv))
                    Native.opm_mouse(m, mv.dx, mv.dy, mv.buttons);

                if (scriptPending && executed >= s.ScriptAt)
                {
                    Native.opm_serial(m, s.BootScript.Replace(';', '\r') + "\r");
                    scriptPending = false;
                    ConsoleQ.Enqueue($"\n[shell] boot script injected @{executed:N0}\n");
                }

                // Audio. Drained EVERY chunk whether or not anything is
                // listening: the codec's queue is bounded and drops its
                // oldest, so leaving it alone would make the machine skip
                // rather than merely stay quiet. The samples are big-endian —
                // see opm_audio — and the sink swaps them on the way to the
                // device.
                if (!s.Sound)
                {
                    // Still drained, just not played: the codec's queue is
                    // bounded and drops its oldest, so leaving it alone makes
                    // the machine skip rather than merely stay quiet.
                    while (Native.opm_audio(m, pcm, (uint)pcm.Length) > 0) { }
                }
                else if (_wave.IsOpen || _wave.Open((int)Native.opm_audio_rate(m)))
                {
                    int rate = (int)Native.opm_audio_rate(m);
                    if (rate != _wave.Rate)
                        _wave.Open(rate);
                    // ⭐ Take only what the device can hold. The overflow's
                    // right home is the codec's own six-second queue, not the
                    // floor: draining more than the sink can take and dropping
                    // the excess punches a hole in the waveform, and a hole is
                    // heard as a pop.
                    int room = _wave.FreeBytes;
                    while (room >= 4)
                    {
                        uint want = (uint)Math.Min(room, pcm.Length);
                        uint got = Native.opm_audio(m, pcm, want);
                        if (got == 0)
                            break;
                        _wave.Write(pcm, (int)got, bigEndian: true);
                        _capture?.Write(pcm, (int)got, rate, bigEndian: true);
                        room -= (int)got;
                    }
                }
                StatAudioRate = _wave.IsOpen ? _wave.Rate : 0;
                Interlocked.Exchange(ref StatAudioDropped, _wave.DroppedFrames);
                Interlocked.Exchange(ref StatAudioStarved, _wave.Starvations);

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
            _wave.Close();
            _capture?.Dispose();
            _capture = null;
            if (m != IntPtr.Zero)
                Native.opm_destroy(m);
            // The image is rebuilt at every Start, so deleting it loses
            // nothing — and a temp file the size of the shared folder
            // should not outlive the machine that was reading it. After
            // opm_destroy: the ATAPI cell holds the file open until then.
            if (sharedImg != null)
                try { File.Delete(sharedImg); } catch { }
            Running = false;
            StatAudioRate = 0;
            _thread = null;
            Ended?.Invoke(reason);
        }
    }
}
