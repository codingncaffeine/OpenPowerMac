using System.Collections.Concurrent;

namespace OpenPowerMac.Gui;

/// <summary>The front end's own log: platform events (input capture, sound
/// library, window) that are not the machine's console. Written to
/// $XDG_STATE_HOME/OpenPowerMac/gui.log (~/.local/state by default), the
/// machine's console beside it in console.log, on a
/// background writer, so a line never costs the UI thread a disk write.</summary>
internal static class Log
{
    private static readonly BlockingCollection<(bool console, string text)> Queue = new(4096);
    private static StreamWriter? _writer, _console;

    public static string Path { get; private set; } = "";

    public static void Init()
    {
        string state = Environment.GetEnvironmentVariable("XDG_STATE_HOME") is { Length: > 0 } x
            ? x
            : System.IO.Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile),
                                     ".local", "state");
        string dir = System.IO.Path.Combine(state, "OpenPowerMac");
        Path = System.IO.Path.Combine(dir, "gui.log");
        try
        {
            Directory.CreateDirectory(dir);
            _writer = new StreamWriter(Path, append: false) { AutoFlush = true };
            _console = new StreamWriter(System.IO.Path.Combine(dir, "console.log"), append: false) { AutoFlush = true };
        }
        catch { _writer = null; }
        new Thread(Pump) { IsBackground = true, Name = "opm-log" }.Start();
    }

    public static void Line(string text) =>
        Queue.TryAdd((false, $"{DateTime.Now:HH:mm:ss.fff} {text}"));

    /// <summary>The machine's serial console, raw, as the pane receives it:
    /// console.log beside gui.log, so a boot can be read whole after the fact.</summary>
    public static void Console(string chunk) => Queue.TryAdd((true, chunk));

    private static void Pump()
    {
        foreach (var (console, text) in Queue.GetConsumingEnumerable())
        {
            try
            {
                if (console)
                {
                    _console?.Write(text);
                    continue;
                }
                System.Console.Error.WriteLine(text);
                _writer?.WriteLine(text);
            }
            catch { }
        }
    }
}
