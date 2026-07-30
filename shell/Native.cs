using System.Runtime.InteropServices;

namespace OpenPowerMac.Shell;

/// <summary>P/Invoke surface over opmcapi.dll (the core's C ABI).</summary>
internal static class Native
{
    private const string Dll = "opmcapi";

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern IntPtr opm_create(string romPath, string? cdPath,
                                           string? hdPath, string? atiRomPath,
                                           uint ramMb, uint fastTb);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void opm_destroy(IntPtr m);

    // Defer PCI visibility of the ATI card: the practiced boot hides it past
    // OF's console choice (~228M insns) so the serial console stays owned.
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void opm_ati_at(IntPtr m, ulong insn);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern ulong opm_run(IntPtr m, ulong insns);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern void opm_serial(IntPtr m, string text);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern uint opm_console(IntPtr m, byte[] buf, uint cap);

    // USB HID: keyboard on usb@8, mouse on usb@9. Motion is RELATIVE, so the
    // caller sends the delta since its last report, and resends `buttons`
    // while a button is held.
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern void opm_key(IntPtr m, string text);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void opm_mouse(IntPtr m, int dx, int dy, uint buttons);

    // 1 = frame filled (BGRA, w*h*4), 0 = size query only, -1 = no picture.
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern int opm_screen(IntPtr m, byte[]? bgra, uint cap,
                                        out uint w, out uint h);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern ulong opm_executed(IntPtr m);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern uint opm_pc(IntPtr m);

    // ---- pointer capture (Win32) ----
    //
    // A USB mouse is a RELATIVE device, so the guest's cursor and the host's
    // are two independent pointers. They drift apart the moment the guest
    // applies its own acceleration or pins at a screen edge, and then aiming
    // the host pointer at the Apple menu leaves the guest's somewhere else.
    // The cure is to stop having two of them: hide the host pointer, warp it
    // back to the centre after every move, and feed the guest the raw travel.
    // The guest's cursor becomes the only one on screen.
    [DllImport("user32.dll")]
    [return: MarshalAs(UnmanagedType.Bool)]
    public static extern bool SetCursorPos(int x, int y);
}
