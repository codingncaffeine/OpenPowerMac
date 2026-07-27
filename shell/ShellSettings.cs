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
    public string AtiRomPath { get; set; } = "";
    public uint RamMb { get; set; } = 256;
    public uint FastTb { get; set; } = 60;
    public ulong AtiAt { get; set; } = 236_000_000;
    public ulong ScriptAt { get; set; } = 240_000_000;
    public bool AutoBoot { get; set; } = true;

    // ';' becomes CR on injection (one Forth line per segment).
    public string BootScript { get; set; } =
        @""" /pci@f0000000"" select-dev;10 8000 probe-pci-device;8000 10 probe-pci-device;unselect-dev;" +
        @"dev /pci@f0000000/pci1002,5046@10;"" ATY,Rage128Pd"" device-name;"" display"" device-type;" +
        @""" ATY,Rage128Pd"" encode-string "" compatible"" property;" +
        @""" /pci@f2000000"" select-dev;3000000 to pci-probe-request;unselect-dev;probe-pci;" +
        @"dev /pci@f2000000/pci106b,19@18;"" usb"" device-name;"" usb"" device-type;" +
        @"dev /pci@f2000000/pci106b,19@19;"" usb"" device-name;"" usb"" device-type;mac-boot";

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
