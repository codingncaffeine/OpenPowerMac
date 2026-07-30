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

    // Required for any picture. Without the card's FCode there is no display
    // node, so the OS has nothing to bind a driver to.
    public string AtiRomPath { get; set; } = "";

    // 1536 MB: three 512 MB DIMMs, the most a Sawtooth takes and the most Mac
    // OS 9 can use. The guest sizes this from the SPD, so the figure here only
    // has to MATCH what the DIMMs advertise — it does not set it.
    public uint RamMb { get; set; } = 64;

    // Extra timebase cycles per instruction. 60 is a COMPROMISE and both
    // directions off it are worse — measured, one variable, same run length:
    //
    //   fast-tb  60 -> 11.0 MIPS, 857,259 exceptions, the OS sets its own
    //                  32bpp mode and paints 1,261,505 bytes
    //   fast-tb 300 ->  8.7 MIPS, 4,997,821 exceptions, and the 68K world
    //                  never even starts inside the same 3.3 B instructions
    //
    // Raising it is the decrementer storm: the guest spends so much of itself
    // in exception handling that it stops making progress. Lowering it slows
    // guest time further, and guest time is ALREADY slow — 3.9 ticks per host
    // second against a real 60, or about 12 once the Open Firmware era, where
    // the counter is not running, is discounted.
    //
    // (An earlier note here claimed 60 ran guest time seven times fast and
    // that 7 landed near real time. The instrument says otherwise in the
    // direction that matters, and the instrument wins.)
    public uint FastTb { get; set; } = 60;

    // The card must be visible from the start. Held invisible until 236M it
    // misses Open Firmware's PCI probe entirely, its FCode never runs, and no
    // display node is built — the screen then stays blank no matter how well
    // everything downstream works.
    public ulong AtiAt { get; set; } = 1;

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
                return JsonSerializer.Deserialize<ShellSettings>(
                           File.ReadAllText(FilePath)) ?? new ShellSettings();
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
