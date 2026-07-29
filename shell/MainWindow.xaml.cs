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
        PreviewTextInput += OnGuestTextInput;
    }

    // ---- guest input ----

    private Point? _lastMouse;

    private static uint ButtonMask(MouseEventArgs e) =>
        (e.LeftButton == MouseButtonState.Pressed ? 1u : 0u) |
        (e.RightButton == MouseButtonState.Pressed ? 2u : 0u) |
        (e.MiddleButton == MouseButtonState.Pressed ? 4u : 0u);

    // The image is letterboxed and scaled, so a window pixel is not a guest
    // pixel; scale the delta by guest resolution over rendered size, or a
    // fast flick moves the guest cursor a fraction of the distance.
    private void OnScreenMouseMove(object sender, MouseEventArgs e)
    {
        if (_session is not { Running: true } s)
            return;
        Point p = e.GetPosition(imgScreen);
        if (_lastMouse is { } prev && imgScreen.ActualWidth > 0 &&
            imgScreen.ActualHeight > 0 && _bmp != null)
        {
            double sx = _bmp.PixelWidth / imgScreen.ActualWidth;
            double sy = _bmp.PixelHeight / imgScreen.ActualHeight;
            int dx = (int)Math.Round((p.X - prev.X) * sx);
            int dy = (int)Math.Round((p.Y - prev.Y) * sy);
            if (dx != 0 || dy != 0)
                s.SendMouse(dx, dy, ButtonMask(e));
        }
        _lastMouse = p;
    }

    private void OnScreenMouseButton(object sender, MouseButtonEventArgs e)
    {
        if (_session is not { Running: true } s)
            return;
        imgScreen.Focus();
        // Zero motion, current buttons: press and release are each a report
        // of their own, which is what the guest edge-detects on.
        s.SendMouse(0, 0, ButtonMask(e));
    }

    private void OnGuestKeyDown(object sender, KeyEventArgs e)
    {
        if (_session is not { Running: true } s)
            return;
        // Return never arrives as text input, so it needs taking here.
        if (e.Key == Key.Return)
        {
            s.SendKeys("\r");
            e.Handled = true;
        }
    }

    private void OnGuestTextInput(object sender, TextCompositionEventArgs e)
    {
        if (_session is { Running: true } s && !string.IsNullOrEmpty(e.Text))
            s.SendKeys(e.Text);
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
