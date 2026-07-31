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
    // Travel is METERED rather than handed over as it arrives, and the reason
    // is specific. The Cursor Device Manager applies an acceleration curve to
    // whatever has accumulated by the time its vertical-blank task drains it,
    // and that curve is steep — measured 1.20x gain at 17 units per drain
    // against 1.71x at 35. Simply splitting a delta into small reports does
    // NOTHING, because the USB cell re-accumulates them (there are thousands
    // of polls between drains) and the guest still sees one lump. What has to
    // be bounded is travel PER DRAIN, so the meter has to work in time: hold
    // the travel here and release at most kUnitsPerTick of it on each 33 ms
    // tick. Nothing is discarded — a fast flick glides over several frames
    // instead of teleporting, and total distance is preserved exactly.
    private double _pendX, _pendY;
    private uint _pendButtons;

    // Sensitivity. The host pointer moves in screen pixels and the guest is a
    // 640-wide desktop, so 1:1 is far too fast once the CDM's gain is applied.
    private const double kSensitivity = 0.55;
    // Ceiling on units released per tick. Ticks run at 30 Hz and the guest
    // drains ~47 times a host second, so this bounds per-drain travel to well
    // inside the linear part of the acceleration curve.
    private const int kUnitsPerTick = 10;

    private void SendTravel(double dx, double dy, uint buttons)
    {
        _pendX += dx * kSensitivity;
        _pendY += dy * kSensitivity;
        _pendButtons = buttons;
    }

    /// <summary>Release a bounded slice of the held travel. Called from the UI
    /// tick so the guest receives a steady stream rather than one lump.</summary>
    private void PumpMouse()
    {
        if (_session is not { Running: true } s)
            return;
        if (Math.Abs(_pendX) < 1 && Math.Abs(_pendY) < 1)
            return;

        // Scale both axes by the same factor so the direction of travel is
        // preserved; clamping them independently would bend a diagonal flick.
        double mag = Math.Max(Math.Abs(_pendX), Math.Abs(_pendY));
        double take = Math.Min(1.0, kUnitsPerTick / mag);
        int dx = (int)Math.Truncate(_pendX * take);
        int dy = (int)Math.Truncate(_pendY * take);
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
        PumpMouse();
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
        txtAudio.Text = s.StatAudioRate == 0
            ? ""
            : dropped > 0
                ? $"audio {s.StatAudioRate / 1000.0:F1} kHz ({dropped:N0} dropped)"
                : $"audio {s.StatAudioRate / 1000.0:F1} kHz";
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
        Title = t;
    }

    protected override void OnClosed(EventArgs e)
    {
        _timer.Stop();
        _session?.Stop();
        base.OnClosed(e);
    }
}
