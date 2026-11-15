using Avalonia;

namespace OpenPowerMac.Gui;

internal static class Program
{
    // Nothing that touches Avalonia may run before BuildAvaloniaApp: the
    // platform is not initialized yet.
    [STAThread]
    public static void Main(string[] args)
    {
        Platform.NativeLibs.Install();
        BuildAvaloniaApp().StartWithClassicDesktopLifetime(args);
    }

    public static AppBuilder BuildAvaloniaApp() =>
        AppBuilder.Configure<App>()
            .UsePlatformDetect()
            .WithInterFont()
            .LogToTrace();
}
