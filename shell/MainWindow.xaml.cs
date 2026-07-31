using System.IO;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Threading;
using Microsoft.Win32;

namespace OpenPowerMac.Shell;

public partial class MainWindow : Window
{
    private readonly ShellSettings _settings = ShellSettings.Load();
    private readonly TerminalBuffer _term = new();
    private readonly DispatcherTimer _timer;
    private MachineSession? _session;
    private WriteableBitmap? _bmp;

    public MainWindow()
    {
        InitializeComponent();
        WindowTheming.ApplyDarkTitleBar(this);
        miAutoBoot.IsChecked = _settings.AutoBoot;
        miSound.IsChecked = _settings.Sound;
        UpdateTitle();
        _timer = new DispatcherTimer(DispatcherPriority.Render)
        {
            Interval = TimeSpan.FromMilliseconds(33),
        };
        _timer.Tick += OnTimer;
        _timer.Start();

        // Guest input. The screen is the pointer surface: motion is sent as
        // the delta since the last report, because a USB boot mouse is a
        // RELATIVE device, and the button mask rides along with every motion
        // so a drag keeps its button down. Keys are taken at the window so
        // the guest gets them without the image having to hold focus.
        imgScreen.MouseMove += OnScreenMouseMove;
        imgScreen.MouseDown += OnScreenMouseButton;
        imgScreen.MouseUp += OnScreenMouseButton;
        PreviewKeyDown += OnGuestKeyDown;
        PreviewKeyUp += OnGuestKeyUp;
        // Alt-tabbing away with the pointer swallowed would leave the host
        // cursor hidden and pinned with no window to click in — and any
        // modifier held at that moment would still be held on the way back,
        // silently changing the meaning of every key after it.
        Deactivated += (_, _) =>
        {
            ReleasePointer();
            ReleaseHeldKeys();
        };
    }

    // ---- guest input ----

    private Point? _lastMouse;
    private bool _captured;
    // The warp we perform ourselves raises its own MouseMove; treat the report
    // that lands back on the anchor as our own echo, not as travel.
    private Point _anchorScreen;

    private static uint ButtonMask(MouseEventArgs e) =>
        (e.LeftButton == MouseButtonState.Pressed ? 1u : 0u) |
        (e.RightButton == MouseButtonState.Pressed ? 2u : 0u);

    /// <summary>Take the pointer: hide the host cursor and route raw travel to
    /// the guest. Middle-click gives it back.</summary>
    private void CapturePointer()
    {
        if (_captured || _session is not { Running: true })
            return;
        _captured = true;
        Mouse.Capture(imgScreen);
        imgScreen.Cursor = Cursors.None;
        RecentrePointer();
        UpdateCaptureHint();
    }

    private void ReleasePointer()
    {
        if (!_captured)
            return;
        _captured = false;
        Mouse.Capture(null);
        imgScreen.Cursor = Cursors.Arrow;
        UpdateCaptureHint();
    }

    private void RecentrePointer()
    {
        if (imgScreen.ActualWidth <= 0 || imgScreen.ActualHeight <= 0)
            return;
        Point mid = imgScreen.PointToScreen(
            new Point(imgScreen.ActualWidth / 2, imgScreen.ActualHeight / 2));
        _anchorScreen = mid;
        Native.SetCursorPos((int)mid.X, (int)mid.Y);
    }

    // Only touched when something actually changes: this is called from the
    // 33 ms UI tick as well as the capture edges, and reassigning Text every
    // frame would invalidate the visual tree thirty times a second for nothing.
    private bool _hintCaptured, _hintVisible;

    private void UpdateCaptureHint()
    {
        bool visible = _session is { Running: true };
        if (visible != _hintVisible)
        {
            _hintVisible = visible;
            brdCapture.Visibility =
                visible ? Visibility.Visible : Visibility.Collapsed;
        }
        if (_captured != _hintCaptured)
        {
            _hintCaptured = _captured;
            txtCapture.Text = _captured
                ? "Pointer captured — middle-click (or Esc) to release"
                : "Click the screen to capture the pointer";
        }
    }

    // Travel, not position. While captured the host pointer is pinned to the
    // centre and every move is measured from there, so the guest gets
    // unbounded relative motion and the two cursors cannot drift apart.
    //
    // ⛔⛔ TRAVEL IS NOT METERED, AND THE METER THAT USED TO BE HERE IS WHY THE
    // POINTER WAS SLOW.
    //
    // It held the motion and released at most 10 units on each 33 ms tick,
    // which is a hard ceiling of about **303 units per second whatever the
    // hand does** — a 400 CPI mouse moved at a leisurely five inches a second
    // delivers 2,000. Everything above the ceiling was not dropped but
    // deferred, so a flick kept gliding after the hand stopped: the pointer
    // was smooth and sluggish for the same reason, and no amount of extra
    // physical movement could beat the cap.
    //
    // ⭐ The meter's own justification is what gives it away. It existed to
    // keep travel PER DRAIN "well inside the linear part of the acceleration
    // curve" — that is, to deliberately suppress the Cursor Device Manager's
    // acceleration. But that curve is the thing that makes a mouse feel right:
    // move fast, hand the CDM a large delta for that drain, get high gain and
    // a pointer that crosses the screen. Bounding the delta pins the curve at
    // its low end forever, which is precisely "needs far more physical
    // movement than it should".
    //
    // ⚠ It was also tuned against a machine whose clock ran 1.93x fast (the
    // comment quoted "the guest drains ~47 times a host second"). The clock is
    // real time now and the guest drains at 60 Hz, so the premise is gone as
    // well as the reasoning. A real USB mouse hands its host whatever it
    // accumulated between polls and lets the OS curve decide; so do we. The
    // USB cell already accumulates between the guest's own polls, which is the
    // only re-accumulation that models anything.
    //
    // What is left is a sub-unit remainder, because sensitivity may be
    // fractional and a HID report carries whole counts. Nothing is discarded.
    private double _pendX, _pendY;
    private uint _pendButtons;

    // 1:1 with the host pointer. The guest has a Mouse control panel and that
    // is the right place to set tracking speed — a constant here would be this
    // host's feel baked into the machine.
    private const double kSensitivity = 1.0;

    private void SendTravel(double dx, double dy, uint buttons)
    {
        _pendX += dx * kSensitivity;
        _pendY += dy * kSensitivity;
        _pendButtons = buttons;
        FlushMouse();
    }

    /// <summary>Hand over whole counts as they arrive, keeping the sub-unit
    /// remainder. Also called from the UI tick, so a fraction left by the last
    /// movement is not stranded when the hand stops.</summary>
    private void FlushMouse()
    {
        if (_session is not { Running: true } s)
            return;
        int dx = (int)Math.Truncate(_pendX);
        int dy = (int)Math.Truncate(_pendY);
        if (dx == 0 && dy == 0)
            return;
        _pendX -= dx;
        _pendY -= dy;
        s.SendMouse(dx, dy, _pendButtons);
    }

    private void OnScreenMouseMove(object sender, MouseEventArgs e)
    {
        if (_session is not { Running: true })
            return;

        if (_captured)
        {
            Point now = imgScreen.PointToScreen(e.GetPosition(imgScreen));
            int dx = (int)Math.Round(now.X - _anchorScreen.X);
            int dy = (int)Math.Round(now.Y - _anchorScreen.Y);
            if (dx == 0 && dy == 0)
                return; // our own re-centring echo
            SendTravel(dx, dy, ButtonMask(e));
            RecentrePointer();
            return;
        }

        // Uncaptured: still track, so the guest follows the pointer before the
        // user commits to capturing. The image is letterboxed and scaled, so a
        // window pixel is not a guest pixel.
        Point p = e.GetPosition(imgScreen);
        if (_lastMouse is { } prev && imgScreen.ActualWidth > 0 &&
            imgScreen.ActualHeight > 0 && _bmp != null)
        {
            double sx = _bmp.PixelWidth / imgScreen.ActualWidth;
            double sy = _bmp.PixelHeight / imgScreen.ActualHeight;
            int dx = (int)Math.Round((p.X - prev.X) * sx);
            int dy = (int)Math.Round((p.Y - prev.Y) * sy);
            if (dx != 0 || dy != 0)
                SendTravel(dx, dy, ButtonMask(e));
        }
        _lastMouse = p;
    }

    private void OnScreenMouseButton(object sender, MouseButtonEventArgs e)
    {
        if (_session is not { Running: true } s)
            return;
        imgScreen.Focus();

        // The middle button is the capture toggle and never reaches the guest:
        // Mac OS 9 is a one-button world, and a machine that can swallow your
        // pointer has to offer an unmistakable way out.
        if (e.ChangedButton == MouseButton.Middle)
        {
            if (e.ButtonState == MouseButtonState.Pressed)
            {
                if (_captured) ReleasePointer(); else CapturePointer();
            }
            e.Handled = true;
            return;
        }

        if (!_captured && e.ButtonState == MouseButtonState.Pressed)
            CapturePointer();

        // Zero motion, current buttons: press and release are each a report
        // of their own, which is what the guest edge-detects on.
        s.SendMouse(0, 0, ButtonMask(e));
    }

    // Everything the guest currently believes is held down, so it can be let
    // go when this window stops receiving key-ups.
    private readonly HashSet<uint> _held = new();

    /// <summary>The key WPF really means. While Alt is down every keystroke
    /// arrives as Key.System with the true key in SystemKey, so reading e.Key
    /// alone loses every Alt combination.</summary>
    private static Key RealKey(KeyEventArgs e) =>
        e.Key == Key.System ? e.SystemKey : e.Key;

    private void OnGuestKeyDown(object sender, KeyEventArgs e)
    {
        var key = RealKey(e);
        // Escape releases the pointer before anything else looks at the key.
        // Middle-click is the advertised way out, but a mouse without a middle
        // button must not be able to trap you.
        if (_captured && key == Key.Escape)
        {
            ReleasePointer();
            e.Handled = true;
            return;
        }
        // Alt-F4 belongs to the host. Every other key is swallowed below, and
        // a window that cannot be closed from the keyboard is a trap.
        if (key == Key.F4 &&
            (Keyboard.Modifiers & ModifierKeys.Alt) != 0)
            return;
        // The serial box is a host control, not the guest: while it holds
        // focus it keeps its own typing.
        if (Keyboard.FocusedElement == txtInput)
            return;
        if (_session is not { Running: true } s)
            return;
        var usage = HidKeys.Usage(key);
        if (usage == 0)
            return;
        // A USB keyboard does not auto-repeat — it reports the key once and
        // the OS decides how to repeat it. Forwarding the host's repeats would
        // stack a second press on a key the guest already holds.
        if (!e.IsRepeat)
        {
            _held.Add(usage);
            s.SendKeyEvent(usage, true);
        }
        // Taken, so WPF does not also read it as menu access or focus
        // navigation: Alt would open the menu bar and Tab would walk it.
        e.Handled = true;
    }

    private void OnGuestKeyUp(object sender, KeyEventArgs e)
    {
        if (Keyboard.FocusedElement == txtInput)
            return;
        if (_session is not { Running: true } s)
            return;
        var usage = HidKeys.Usage(RealKey(e));
        if (usage == 0)
            return;
        _held.Remove(usage);
        s.SendKeyEvent(usage, false);
        e.Handled = true;
    }

    /// <summary>Let go of everything the guest thinks is down. A key-up that
    /// lands somewhere else — alt-tab, a stopped machine — otherwise leaves it
    /// held forever.</summary>
    private void ReleaseHeldKeys()
    {
        if (_held.Count == 0)
            return;
        if (_session is { Running: true } s)
            foreach (var usage in _held)
                s.SendKeyEvent(usage, false);
        _held.Clear();
    }

    // ---- machine control ----

    private void OnStart(object sender, RoutedEventArgs e)
    {
        if (_session != null)
            return;
        if (!File.Exists(_settings.RomPath))
        {
            if (!ChooseRom())
                return;
        }
        var session = new MachineSession();
        session.Ended += reason => Dispatcher.BeginInvoke(() => OnSessionEnded(session, reason));
        _session = session;
        session.Start(_settings);
        miStart.IsEnabled = false;
        miStop.IsEnabled = true;
        btnSend.IsEnabled = true;
        txtState.Text = "running";
    }

    private void OnStop(object sender, RoutedEventArgs e)
    {
        _session?.Stop();
        txtState.Text = "stopping…";
    }

    private void OnSessionEnded(MachineSession session, string reason)
    {
        ReleaseHeldKeys();
        // The worker's last messages may still be queued — render them.
        while (session.ConsoleQ.TryDequeue(out var chunk))
            _term.Feed(chunk);
        if (_term.TakeDirty())
        {
            txtConsole.Text = _term.Render();
            txtConsole.ScrollToEnd();
        }
        _session = null;
        miStart.IsEnabled = true;
        miStop.IsEnabled = false;
        btnSend.IsEnabled = false;
        txtState.Text = reason;
    }

    // ---- serial console ----

    private void OnSend(object sender, RoutedEventArgs e) => SendLine();

    private void OnInputKeyDown(object sender, KeyEventArgs e)
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
        _session.SendSerial(txtInput.Text);
        txtInput.Clear();
    }

    // ---- frame + console pump ----

    private void OnTimer(object? sender, EventArgs e)
    {
        var s = _session;
        bool conDirty = false;
        while ((s?.ConsoleQ ?? StaticEmptyQ).TryDequeue(out var chunk))
        {
            _term.Feed(chunk);
            conDirty = true;
        }
        if (conDirty && _term.TakeDirty())
        {
            txtConsole.Text = _term.Render();
            txtConsole.ScrollToEnd();
        }

        UpdateCaptureHint();
        FlushMouse();
        if (_captured && s is not { Running: true })
            ReleasePointer(); // the machine stopped under us

        if (s == null)
            return;

        lock (s.FrameLock)
        {
            if (s.FrameDirty)
            {
                s.FrameDirty = false;
                int w = s.FrameW, h = s.FrameH;
                if (_bmp == null || _bmp.PixelWidth != w || _bmp.PixelHeight != h)
                {
                    _bmp = new WriteableBitmap(w, h, 96, 96, PixelFormats.Bgra32, null);
                    imgScreen.Source = _bmp;
                    pnlNoVideo.Visibility = Visibility.Collapsed;
                    txtVideo.Text = $"{w}×{h}";
                }
                _bmp.WritePixels(new Int32Rect(0, 0, w, h), s.Frame, w * 4, 0);
            }
        }

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
        txtAudio.Text = s.StatAudioRate == 0
            ? ""
            : $"audio {s.StatAudioRate / 1000.0:F1} kHz{audioNote}";
    }

    private static readonly System.Collections.Concurrent.ConcurrentQueue<string> StaticEmptyQ = new();

    // ---- file pickers ----

    private bool ChooseRom()
    {
        var dlg = new OpenFileDialog
        {
            Title = "Choose the Sawtooth Boot ROM (1 MB)",
            Filter = "Boot ROM (*.rom)|*.rom|All files (*.*)|*.*",
        };
        if (dlg.ShowDialog(this) != true)
            return false;
        _settings.RomPath = dlg.FileName;
        _settings.Save();
        UpdateTitle();
        return true;
    }

    private void OnChooseRom(object sender, RoutedEventArgs e) => ChooseRom();

    private void OnChooseCd(object sender, RoutedEventArgs e)
    {
        var dlg = new OpenFileDialog
        {
            Title = "Choose a CD image (Mac OS 9 install/restore ISO)",
            Filter = "CD images (*.iso;*.toast;*.img)|*.iso;*.toast;*.img|All files (*.*)|*.*",
        };
        if (dlg.ShowDialog(this) != true)
            return;
        _settings.CdPath = dlg.FileName;
        _settings.Save();
        UpdateTitle();
    }

    // Eject rather than "detach", because that is the decision being made: a
    // Mac with a bootable CD in the drive boots the CD, so taking it out is
    // how you tell it to boot the disk you just installed onto. Open Firmware
    // is where that preference lives, and this is the honest way to express it
    // without teaching the shell to rewrite NVRAM.
    private void OnEjectCd(object sender, RoutedEventArgs e)
    {
        _settings.CdPath = "";
        _settings.Save();
        UpdateTitle();
        if (_session is { Running: true })
            MessageBox.Show(
                this,
                "CD ejected.\n\nThe machine is running and opened its media "
                + "when it started — this takes effect the next time you "
                + "start it.",
                "Eject CD", MessageBoxButton.OK, MessageBoxImage.Information);
    }

    // 🩺 The report a stall needs, delivered to the console pane so it can be
    // read and copied. Asked for on the machine's own thread — see
    // MachineSession.RequestDiagnostics.
    private void OnDiagnostics(object sender, RoutedEventArgs e)
    {
        if (_session is { Running: true } s)
            s.RequestDiagnostics();
        else
            MessageBox.Show(this, "The machine is not running.", "Diagnostics",
                            MessageBoxButton.OK, MessageBoxImage.Information);
    }

    private void OnChooseAti(object sender, RoutedEventArgs e)
    {
        var dlg = new OpenFileDialog
        {
            Title = "Choose the ATI Rage 128 Pro FCode ROM",
            Filter = "Card ROM (*.rom;*.bin)|*.rom;*.bin|All files (*.*)|*.*",
        };
        if (dlg.ShowDialog(this) != true)
            return;
        _settings.AtiRomPath = dlg.FileName;
        _settings.Save();
        UpdateTitle();
    }

    // ---- hard disks ----
    //
    // ⚠ THE MACHINE OPENS ITS DISK AT opm_create, so everything here decides
    // what the NEXT start will boot. Changing the disk under a running machine
    // is the one case that needs saying out loud, because it otherwise looks
    // like the menu did nothing.

    // The history entries currently inserted into the submenu. Held so they
    // can be removed before the next rebuild — the alternative is recognising
    // them by their content, which breaks the first time a disk is called
    // "Detach".
    private readonly List<MenuItem> _hdRecentItems = new();

    private static string DescribeMb(long mb) =>
        mb >= 1024 && mb % 1024 == 0 ? $"{mb / 1024} GB" : $"{mb} MB";

    private static string DescribeBytes(long bytes) =>
        bytes >= 1L << 30 ? $"{bytes / (double)(1L << 30):0.#} GB"
                          : $"{bytes / (double)(1L << 20):0.#} MB";

    // What a disk is, said in one line: its name and how big it is. A missing
    // file says so rather than being hidden — a disk on a drive that is not
    // plugged in right now is still the disk you meant.
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

    private void OnHardDiskOpened(object sender, RoutedEventArgs e)
    {
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
                IsCheckable = true,
                IsChecked = string.Equals(p, _settings.HdPath,
                                          StringComparison.OrdinalIgnoreCase),
                ToolTip = p,
            };
            mi.Click += (_, _) => AttachHd(p, "attached");
            miHd.Items.Insert(at++, mi);
            _hdRecentItems.Add(mi);
        }
        miHdSep.Visibility = _settings.RecentHds.Count > 0
                                 ? Visibility.Visible : Visibility.Collapsed;
        miHdDetach.IsEnabled = !string.IsNullOrWhiteSpace(_settings.HdPath);
    }

    private void OnNewHd(object sender, RoutedEventArgs e)
    {
        if (sender is not MenuItem mi || mi.Tag is not string tag ||
            !long.TryParse(tag, out long mb))
            return;
        var dlg = new SaveFileDialog
        {
            Title = $"New {DescribeMb(mb)} hard disk image",
            Filter = "Disk images (*.img)|*.img|All files (*.*)|*.*",
            FileName = $"macos9_{(mb >= 1024 && mb % 1024 == 0 ? $"{mb / 1024}gb" : $"{mb}mb")}.img",
            DefaultExt = ".img",
            AddExtension = true,
            // Ours below, because Windows' own prompt does not say how big the
            // thing it is about to destroy is — and the point of this menu is
            // that several disks are in play at once.
            OverwritePrompt = false,
        };
        if (dlg.ShowDialog(this) != true)
            return;
        string path = dlg.FileName;
        if (File.Exists(path))
        {
            long existing;
            try { existing = new FileInfo(path).Length; } catch { existing = 0; }
            if (MessageBox.Show(
                    this,
                    $"{Path.GetFileName(path)} already exists ({DescribeBytes(existing)}).\n\n" +
                    $"Replace it with a blank {DescribeMb(mb)} disk? Anything " +
                    "installed on it is lost.",
                    "Replace disk image?", MessageBoxButton.YesNo,
                    MessageBoxImage.Warning,
                    MessageBoxResult.No) != MessageBoxResult.Yes)
                return;
        }
        try
        {
            // SetLength does not write the bytes: on NTFS it allocates the
            // clusters and moves the file's length, but leaves the VALID DATA
            // LENGTH at zero, and the unwritten region reads back as zeros —
            // which is exactly what a factory-fresh drive looks like.
            //
            // ⛔⛔ AND THAT IS A TRAP, SO THE LAST BYTE IS WRITTEN HERE ON
            // PURPOSE. The first write PAST the valid-data mark makes NTFS
            // zero-fill everything before it, synchronously. Drive Setup puts
            // HFS+'s alternate volume header at the very END of the disk, so
            // the guest's first initialise stalls the whole machine for the
            // time it takes to zero the entire image — measured 352 ms for
            // 2 GB on an NVMe here, inside ONE emulated instruction, and
            // seconds on a slower disk. That is long enough to blow a driver
            // timeout, and it surfaces to the user as "the disk has errors"
            // rather than as anything to do with the host filesystem.
            //
            // ⭐ So pay it once, HERE, where a pause is expected and harmless,
            // and the emulated machine never sees it. Deliberately NOT solved
            // by marking the file sparse: sparse would be faster still and
            // cost only the space in use, but then the image can fail to write
            // mid-install because the HOST volume filled up — and a disk that
            // dies halfway through an OS install is a far worse failure than a
            // slow New Disk Image.
            // Zeroing a large image is not instant on every disk, and an
            // unresponsive window with no cursor change reads as a hang.
            Mouse.OverrideCursor = Cursors.Wait;
            try
            {
                using var fs = new FileStream(path, FileMode.Create, FileAccess.Write);
                long bytes = mb * 1024L * 1024L;
                fs.SetLength(bytes);
                fs.Seek(bytes - 1, SeekOrigin.Begin);
                fs.WriteByte(0);
                fs.Flush();
            }
            finally { Mouse.OverrideCursor = null; }
        }
        catch (Exception ex)
        {
            MessageBox.Show(this, $"Could not create the disk image:\n\n{ex.Message}",
                            "New disk image", MessageBoxButton.OK,
                            MessageBoxImage.Error);
            return;
        }
        AttachHd(path, $"created a blank {DescribeMb(mb)} disk and attached");
    }

    private void OnChooseHd(object sender, RoutedEventArgs e)
    {
        var dlg = new OpenFileDialog
        {
            Title = "Choose a hard disk image",
            Filter = "Disk images (*.img;*.dsk;*.hfv)|*.img;*.dsk;*.hfv|All files (*.*)|*.*",
        };
        if (dlg.ShowDialog(this) != true)
            return;
        AttachHd(dlg.FileName, "attached");
    }

    private void OnDetachHd(object sender, RoutedEventArgs e) =>
        AttachHd("", "detached the hard disk");

    private void AttachHd(string path, string what)
    {
        _settings.AttachHd(path);
        _settings.Save();
        UpdateTitle();
        if (_session is { Running: true })
            MessageBox.Show(
                this,
                $"Shell {what}.\n\nThe machine is running, and it opened its "
                + "disk when it started — this takes effect the next time you "
                + "start it.",
                "Hard disk", MessageBoxButton.OK, MessageBoxImage.Information);
    }

    private void OnAutoBootToggled(object sender, RoutedEventArgs e)
    {
        _settings.AutoBoot = miAutoBoot.IsChecked;
        _settings.Save();
    }

    // Takes effect on the next Start. The guest's codec runs either way — the
    // samples are drained whether or not anyone is listening, because the
    // queue is bounded — so this only decides whether they reach a speaker,
    // and it cannot change how the machine behaves.
    private void OnSoundToggled(object sender, RoutedEventArgs e)
    {
        _settings.Sound = miSound.IsChecked;
        _settings.Save();
    }

    private void OnExit(object sender, RoutedEventArgs e) => Close();

    private void UpdateTitle()
    {
        string t = "OpenPowerMac";
        if (!string.IsNullOrWhiteSpace(_settings.RomPath))
            t += " — " + Path.GetFileNameWithoutExtension(_settings.RomPath);
        if (!string.IsNullOrWhiteSpace(_settings.CdPath))
            t += " + " + Path.GetFileName(_settings.CdPath);
        // Which disk this machine will boot. With several test disks in play
        // at once — a blank one, last attempt's half-installed one, the good
        // one — "which am I about to start?" is a question the title should
        // answer without opening a menu.
        t += string.IsNullOrWhiteSpace(_settings.HdPath)
                 ? "  [no HD]"
                 : "  [" + Path.GetFileName(_settings.HdPath) + "]";
        Title = t;
    }

    protected override void OnClosed(EventArgs e)
    {
        _timer.Stop();
        _session?.Stop();
        base.OnClosed(e);
    }
}
