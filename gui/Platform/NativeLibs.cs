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

    public static void Install()
    {
        NativeLibrary.SetDllImportResolver(typeof(NativeLibs).Assembly, Resolve);
        BindHarfBuzzToItself();
    }

    [DllImport("libc", EntryPoint = "dlopen")] private static extern IntPtr dlopen_c(string path, int flags);
    [DllImport("libdl.so.2", EntryPoint = "dlopen")] private static extern IntPtr dlopen_dl(string path, int flags);
    private const int RTLD_NOW = 2;

    /// <summary>
    /// Loads the app's own HarfBuzz (libHarfBuzzSharp.so, the text shaper) before
    /// anything else, with every call it makes resolved at once.
    ///
    /// That library offers the same hb_* names as the system's libharfbuzz and
    /// reaches its own functions through them. FreeType 2.14 and later opens the
    /// system's libharfbuzz for the whole process the first time fontconfig has
    /// it read a font file (a font the font cache has not seen yet is enough).
    /// A libHarfBuzzSharp.so loaded after that, or loaded earlier with its calls
    /// still to be resolved at first use, has them resolved to the system's
    /// copy: another version with other structures, and the app dies in
    /// hb_font_create while its window opens. Loaded first and resolved in
    /// full, there is no other copy to resolve to, and the runtime's own load
    /// later finds this one. The runtime itself asks for resolution at first
    /// use, which is why the load is made here, through dlopen.
    /// </summary>
    private static void BindHarfBuzzToItself()
    {
        if (!OperatingSystem.IsLinux())
            return;
        // Where the runtime will look, in its order: the folders the host worked
        // out from the app's dependency list, then the app's own.
        var folders = new List<string>();
        if (AppContext.GetData("NATIVE_DLL_SEARCH_DIRECTORIES") is string listed)
            folders.AddRange(listed.Split(Path.PathSeparator, StringSplitOptions.RemoveEmptyEntries));
        folders.Add(AppContext.BaseDirectory);
        foreach (string folder in folders)
        {
            string path = Path.Combine(folder, "libHarfBuzzSharp.so");
            if (!File.Exists(path))
                continue;
            IntPtr handle = IntPtr.Zero;
            try { handle = dlopen_c(path, RTLD_NOW); }
            catch (EntryPointNotFoundException)
            {
                // glibc before 2.34 keeps dlopen in libdl.
                try { handle = dlopen_dl(path, RTLD_NOW); }
                catch (Exception ex) when (ex is DllNotFoundException or EntryPointNotFoundException) { }
            }
            catch (DllNotFoundException) { }
            HarfBuzzNote = handle != IntPtr.Zero
                ? $"text shaper: own HarfBuzz opened first ({path})"
                : $"text shaper: own HarfBuzz could NOT be opened first ({path})";
            return;
        }
    }

    /// <summary>What <see cref="BindHarfBuzzToItself"/> did, for the log (which
    /// does not exist yet when it runs).</summary>
    public static string HarfBuzzNote { get; private set; } = "text shaper: own HarfBuzz not found beside the app";

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
