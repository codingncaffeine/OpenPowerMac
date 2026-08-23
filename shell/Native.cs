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

    // Pack a host folder into a classic HFS image for the CD slot — the
    // shared-folder transfer path. UTF-8 marshalling explicitly: names with
    // ™ and friends must survive, and the ANSI default mangles them into
    // bytes the path layer rejects.
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern int opm_hfs_build(
        [MarshalAs(UnmanagedType.LPUTF8Str)] string folder,
        [MarshalAs(UnmanagedType.LPUTF8Str)] string outPath,
        [MarshalAs(UnmanagedType.LPUTF8Str)] string volName,
        [MarshalAs(UnmanagedType.LPUTF8Str)] System.Text.StringBuilder err,
        uint errCap);

    // Defer PCI visibility of the ATI card: the practiced boot hides it past
    // OF's console choice (~228M insns) so the serial console stays owned.
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void opm_ati_at(IntPtr m, ulong insn);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern ulong opm_run(IntPtr m, ulong insns);

    // Advance the timebase from the host clock (25 MHz) rather than from the
    // instruction count, anchored to wherever the machine is now. fastTb is
    // ignored while this is on.
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void opm_set_realtime(IntPtr m, int on);

    // Times the machine could not keep up and the debt was forgiven. Many
    // slips means it is not running at real time and its tick rate is a lie.
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern ulong opm_rt_slips(IntPtr m);

    // Whether the machine STOPPED — which is not the same question as whether
    // it executed anything. An idle guest legitimately runs no instructions
    // for a whole opm_run call, because the machine spends the time waiting
    // for its next deadline instead of spinning.
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern int opm_halted(IntPtr m);

    // Why is this machine not doing anything? See opm_diag — processor,
    // interrupt controller and both ATA cells, in one report.
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern uint opm_diag(IntPtr m, byte[] buf, uint cap);

    // Save the whole machine mid-run, so a defect that only appears inside a
    // running game can be handed to the headless harness instead of being
    // reproduced by hand on every iteration. See opm_snapshot_save.
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern int opm_snapshot_save(
        IntPtr m, [MarshalAs(UnmanagedType.LPStr)] string path, byte[] err,
        uint errCap);

    // Nanoseconds spent off the host processor since the machine was created.
    // Over elapsed time it is the share of a core the emulator is NOT burning.
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern ulong opm_idle_ns(IntPtr m);

    // Whether a CD really made it into the drive, and if not, why the image
    // was refused. opm_create attaches silently, so a cue sheet naming a
    // missing .bin — or a .dmg compressed with a codec the core does not
    // carry — would otherwise boot an EMPTY drive with nothing saying so.
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern int opm_cd_present(IntPtr m);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern uint opm_cd_error(IntPtr m, byte[] buf, uint cap);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern uint opm_rom_note(IntPtr m, byte[] buf, uint cap);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern void opm_serial(IntPtr m, string text);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern uint opm_console(IntPtr m, byte[] buf, uint cap);

    // USB HID: keyboard on usb@8, mouse on usb@9. Motion is RELATIVE, so the
    // caller sends the delta since its last report, and resends `buttons`
    // while a button is held.
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern void opm_key(IntPtr m, string text);

    // One key going down or coming up, as a HID usage. Usages 0xE0-0xE7 are
    // the modifiers. This is the path a person's keyboard takes; opm_key is
    // for scripted text.
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void opm_key_event(IntPtr m, uint usage, uint down);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void opm_mouse(IntPtr m, int dx, int dy, uint buttons);

    // Drain PCM the guest handed the sound codec: 16-bit signed stereo,
    // BIG-ENDIAN, interleaved. Returns bytes (a multiple of 4). The codec
    // queues about six seconds and then drops its oldest, so this has to be
    // polled whether or not anything is listening — otherwise the machine
    // skips rather than merely staying quiet.
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern uint opm_audio(IntPtr m, byte[] outBuf, uint cap);

    // The rate the guest's Sound Control register selects. Not a constant:
    // the guest may change it, and the host device is re-opened when it does.
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern uint opm_audio_rate(IntPtr m);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern ulong opm_audio_played(IntPtr m);

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
