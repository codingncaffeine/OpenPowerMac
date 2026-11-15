using System.Reflection;
using System.Runtime.InteropServices;

namespace OpenPowerMac.Gui.Platform;

/// <summary>
/// Where the app's native imports come from on Linux.
///
/// A plain [DllImport("SDL3")] only looks for libSDL3.so, the name a -dev package
/// provides; Debian and Ubuntu runtime packages ship libSDL3.so.0 alone, and the
/// release archives carry their own copy beside the app. So SDL3 resolves to the
/// bundled library first, then the versioned name, then the bare one. "libc" gets
/// the same treatment because the C library's file name differs between glibc and
/// musl and neither has an unversioned runtime name.
/// </summary>
internal static class NativeLibs
{
    private static string _sdl3 = "";

    [DllImport("SDL3")] private static extern int SDL_GetVersion();

    public static void Install() =>
        NativeLibrary.SetDllImportResolver(typeof(NativeLibs).Assembly, Resolve);

    private static IntPtr Resolve(string name, Assembly assembly, DllImportSearchPath? searchPath)
    {
        if (name == "SDL3")
        {
            foreach (string candidate in new[]
                     { Path.Combine(AppContext.BaseDirectory, "libSDL3.so.0"), "libSDL3.so.0", "libSDL3.so" })
                if (NativeLibrary.TryLoad(candidate, out IntPtr handle)) { _sdl3 = candidate; return handle; }
        }
        else if (name == "libc")
        {
            foreach (string candidate in new[] { "libc.so.6", "libc.musl-x86_64.so.1", "libc.musl-aarch64.so.1" })
                if (NativeLibrary.TryLoad(candidate, out IntPtr handle)) return handle;
        }
        return IntPtr.Zero;   // the runtime's own search
    }

    /// <summary>Calls one harmless SDL3 function through the same import the sound
    /// output uses and says what happened, for the log: a machine with no usable
    /// SDL3 otherwise shows nothing until a Mac is started and stays silent.</summary>
    public static string DescribeSdl3()
    {
        try
        {
            int v = SDL_GetVersion();
            return $"SDL3: loaded {_sdl3} version {v / 1000000}.{v / 1000 % 1000}.{v % 1000} (audio import ok)";
        }
        catch (Exception ex) when (ex is DllNotFoundException or EntryPointNotFoundException)
        {
            return "SDL3: not available, the Mac will have no sound: " + ex.Message.Split('\n')[0];
        }
    }

    /// <summary>Loads the emulator core the way its imports will and says what
    /// happened, for the log: a package whose core cannot load on this system
    /// otherwise shows nothing until a Mac is started.</summary>
    public static string DescribeCore()
    {
        if (!NativeLibrary.TryLoad("opmcapi", typeof(NativeLibs).Assembly, null, out IntPtr handle))
            return "core: libopmcapi.so did not load, no Mac can be started";
        string beside = Path.Combine(AppContext.BaseDirectory, "libopmcapi.so");
        return NativeLibrary.TryGetExport(handle, "opm_create", out _)
            ? $"core: loaded {(File.Exists(beside) ? beside : "libopmcapi.so")} (machine import ok)"
            : "core: libopmcapi.so loaded, but opm_create is not in it";
    }

    public static string AppVersion =>
        typeof(NativeLibs).Assembly.GetCustomAttribute<AssemblyInformationalVersionAttribute>()
            ?.InformationalVersion.Split('+')[0] ?? "0.0.0";
}
