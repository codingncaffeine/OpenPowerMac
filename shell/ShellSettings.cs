using System.IO;
using System.Text.Json;

namespace OpenPowerMac.Shell;

/// <summary>Persisted shell configuration. The timing values and boot script
/// are the practiced Sawtooth recipe: the card stays PCI-invisible until OF
/// has taken the serial console, then the script builds the display/USB nodes
/// over the Forth bridge and boots.</summary>
public sealed class ShellSettings
{
    public string RomPath { get; set; } = "";
    public string CdPath { get; set; } = "";

    // Not optional in practice: every boot that reaches Mac OS does so from
    // the hard disk. Without one the machine only reaches the firmware.
    public string HdPath { get; set; } = "";

    // Disks used before, most-recent-first. Testing an OS install means
    // keeping several around — a blank one to initialise, the half-installed
    // one from the last attempt, the good one — and switching between them by
    // hand through a file picker every time is how the wrong disk gets
    // overwritten. Capped when written; entries whose file has gone are
    // dropped as the menu is built, not here, so a disk on a disconnected
    // drive survives being unplugged once.
    public List<string> RecentHds { get; set; } = new();

    // How many to keep. Enough for a working set of test disks, few enough
    // that the menu stays readable.
    public const int MaxRecentHds = 8;

    /// <summary>Make <paramref name="path"/> the attached disk and move it to
    /// the front of the history. Passing an empty path detaches without
    /// disturbing the history — "no disk" is a state, not a disk.</summary>
    public void AttachHd(string path)
    {
        HdPath = path ?? "";
        if (string.IsNullOrWhiteSpace(path))
            return;
        RecentHds.RemoveAll(p => string.Equals(p, path,
                                               StringComparison.OrdinalIgnoreCase));
        RecentHds.Insert(0, path);
        if (RecentHds.Count > MaxRecentHds)
            RecentHds.RemoveRange(MaxRecentHds, RecentHds.Count - MaxRecentHds);
    }

    // The shared folder: packed into a classic HFS image at every Start and
    // attached in the CD slot, where the guest mounts it like an inserted
    // disc. Read-only by design — the folder is the single source of truth.
    // An explicitly chosen CD image wins the slot; the console says so.
    public string SharedFolderPath { get; set; } = "";

    // Required for any picture. Without the card's FCode there is no display
    // node, so the OS has nothing to bind a driver to.
    public string AtiRomPath { get; set; } = "";

    // 1536 MB: three 512 MB DIMMs, the most a Sawtooth takes and the most Mac
    // OS 9 can use. The guest sizes this from the SPD, so the figure here only
    // has to MATCH what the DIMMs advertise — it does not set it.
    public uint RamMb { get; set; } = 64;

    // Pace the guest's timebase from the HOST CLOCK — 25 MHz, exactly what a
    // Sawtooth runs — instead of from the instruction count. With it on the
    // machine's own 60 Hz chain emits 60 Ticks per host second by
    // construction, on any host, and Mac OS gets (this host's MIPS)/60
    // instructions to spend on each tick instead of a number FastTb fixed in
    // advance. Turn it off only for a deterministic run.
    public bool Realtime { get; set; } = true;

    // Extra timebase cycles per instruction, used when Realtime is off: the
    // machine advances the timebase (1 + FastTb)/4 ticks per instruction.
    //
    // ⚠⚠ 60 WAS THE VALUE HERE UNTIL 2026-07-30 AND IT NO LONGER BOOTS. Now
    // that the KeyLargo timer is answered the guest's clock is correct, and
    // Mac OS spends about 32,000 emulated instructions on each 60 Hz tick —
    // the 68K VBL chain, the Time Manager, CrsrTask. FastTb 60 leaves it
    // 416,666 x 4/61 = 27,300, less than the work costs, so the machine
    // services ticks back to back and the boot dies at 2.5 G instructions
    // with a uniform grey screen. Measured on cold boots scored by distinct
    // scanlines: 60 fails; 30, 15, 8, 4 and 1 all reach the desktop with the
    // recorded baseline's 462 of 480, 292 disk commands and 1,261,505 bytes
    // painted.
    //
    // 4 is the value for a ~24 MIPS loop (1.06x real). This shell's loop runs
    // nearer 55, so instruction pacing here would run guest time about 2.8x
    // fast — which is the whole reason Realtime is the default.
    public uint FastTb { get; set; } = 4;

    // The card must be visible from the start. Held invisible until 236M it
    // misses Open Firmware's PCI probe entirely, its FCode never runs, and no
    // display node is built — the screen then stays blank no matter how well
    // everything downstream works.
    public ulong AtiAt { get; set; } = 1;

    // Play the guest's audio on the host. On by default — a Mac that makes no
    // sound is a broken Mac, and the very first thing this path carries is
    // the boot ROM's startup chime. Off leaves the codec running and simply
    // stops draining it to a speaker, so turning it off cannot change the
    // machine's behaviour.
    public bool Sound { get; set; } = true;

    public ulong ScriptAt { get; set; } = 240_000_000;
    public bool AutoBoot { get; set; } = true;

    // Empty by default. The machine auto-boots on its own now that mac-io
    // +0x61 is modelled as an input, so Open Firmware honours NVRAM and never
    // stops at the prompt — there is nothing to type at, and injecting the old
    // recipe into a booting machine only disturbs it.
    public string BootScript { get; set; } = "";

    private static string FilePath => Path.Combine(
        Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData),
        "OpenPowerMac", "shell.json");

    public static ShellSettings Load()
    {
        try
        {
            if (File.Exists(FilePath))
            {
                var s = JsonSerializer.Deserialize<ShellSettings>(
                            File.ReadAllText(FilePath)) ?? new ShellSettings();
                // ⚠ Every settings file written before 2026-07-30 carries
                // FastTb 60, and 60 no longer reaches the OS: it leaves Mac OS
                // less time per 60 Hz tick than its own tick work costs, so
                // the machine services ticks back to back and stops at a grey
                // screen. Changing the default alone does not reach an
                // install that already saved one — hence the clamp here.
                if (s.FastTb > 30)
                    s.FastTb = 4;
                // The disk this file already names belongs in the history —
                // every settings file written before the disk menu existed has
                // an HdPath and no history at all, and a history that opens
                // without the disk you are actually booting in it reads as
                // broken.
                if (!string.IsNullOrWhiteSpace(s.HdPath) &&
                    !s.RecentHds.Contains(s.HdPath, StringComparer.OrdinalIgnoreCase))
                    s.RecentHds.Insert(0, s.HdPath);
                return s;
            }
        }
        catch { /* fall through to defaults */ }
        return new ShellSettings();
    }

    public void Save()
    {
        try
        {
            Directory.CreateDirectory(Path.GetDirectoryName(FilePath)!);
            File.WriteAllText(FilePath, JsonSerializer.Serialize(
                this, new JsonSerializerOptions { WriteIndented = true }));
        }
        catch { /* settings loss is not fatal */ }
    }
}
