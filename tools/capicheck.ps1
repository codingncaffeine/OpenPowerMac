# Drive opmcapi.dll — the library the APP runs — the way the app drives it,
# and score it the way g4run is scored.
#
# g4run and the capi are two different machines assembled from the same parts,
# and nothing checked that they were assembled the same way. They were not:
# the capi's stamp wiring was missing the hard disk, so AtaCell::tick() never
# fired, the drive raised BSY on its first command and held it forever, and
# the app had no working disk at all while every g4run boot and all four ctest
# targets passed. This script is the missing gate.
#
#   tools/capicheck.ps1                     # pace from the host clock (app default)
#   tools/capicheck.ps1 -Seconds 60         # how long to run
#   tools/capicheck.ps1 -FastTb 4 -Realtime:$false
#
# It reports the same two numbers the boot gate uses — DISTINCT SCANLINES of
# the framebuffer, which separates a desktop from a grey screen, and the
# timebase rate against real time, which says whether pacing works.
param(
    [int]$Seconds = 45,
    [uint32]$FastTb = 4,
    [switch]$Realtime = $true,
    [string]$Dll = "$PSScriptRoot\..\shell\bin\Release\net11.0-windows\opmcapi.dll",
    [string]$Rom = "I:\Visual Studio Projects\scratch\openpowermac\roms\newworld\sawtooth_4.2.8f1_stock.rom",
    [string]$Cd  = "C:\Users\gamer\Downloads\PowerMacG4.iso",
    [string]$Hd  = "",
    [string]$Ati = "I:\Visual Studio Projects\scratch\openpowermac\ati\ati_oem_rage128pro_136_agp_full.rom"
)

$ErrorActionPreference = 'Stop'
if (-not (Test-Path $Dll)) { throw "no DLL at $Dll — build the shell first" }

# A private copy, because the app holds the shipped one open while it runs.
$tmp = Join-Path $env:TEMP ("opmcapi_check_" + [guid]::NewGuid().ToString('N') + ".dll")
Copy-Item $Dll $tmp

Add-Type -TypeDefinition @"
using System;
using System.Runtime.InteropServices;
public static class Opm {
    [DllImport("$($tmp.Replace('\','\\'))", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern IntPtr opm_create(string rom, string cd, string hd, string ati, uint ramMb, uint fastTb);
    [DllImport("$($tmp.Replace('\','\\'))", CallingConvention = CallingConvention.Cdecl)]
    public static extern void opm_set_realtime(IntPtr m, int on);
    [DllImport("$($tmp.Replace('\','\\'))", CallingConvention = CallingConvention.Cdecl)]
    public static extern ulong opm_run(IntPtr m, ulong insns);
    [DllImport("$($tmp.Replace('\','\\'))", CallingConvention = CallingConvention.Cdecl)]
    public static extern ulong opm_tb(IntPtr m);
    [DllImport("$($tmp.Replace('\','\\'))", CallingConvention = CallingConvention.Cdecl)]
    public static extern ulong opm_rt_slips(IntPtr m);
    [DllImport("$($tmp.Replace('\','\\'))", CallingConvention = CallingConvention.Cdecl)]
    public static extern ulong opm_idle_ns(IntPtr m);
    [DllImport("$($tmp.Replace('\','\\'))", CallingConvention = CallingConvention.Cdecl)]
    public static extern int opm_screen(IntPtr m, byte[] bgra, uint cap, out uint w, out uint h);
    [DllImport("$($tmp.Replace('\','\\'))", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern void opm_serial(IntPtr m, string text);
    [DllImport("$($tmp.Replace('\','\\'))", CallingConvention = CallingConvention.Cdecl)]
    public static extern void opm_destroy(IntPtr m);
}
"@

$m = [Opm]::opm_create($Rom, $Cd, ($Hd -eq "" ? $null : $Hd), $Ati, 64, $FastTb)
if ($m -eq [IntPtr]::Zero) { throw "opm_create failed (ROM must be a 1 MB Sawtooth boot ROM)" }
if ($Realtime) { [Opm]::opm_set_realtime($m, 1) }
Write-Host ("-- capi machine up: fast-tb {0}, realtime {1}" -f $FastTb, $Realtime)

# The same Forth the app's boot script types, and at the same point.
$script = '" /pci@f0000000" select-dev;10 8000 probe-pci-device;8000 10 probe-pci-device;unselect-dev;dev /pci@f0000000/pci1002,5046@10;" ATY,Rage128Pd" device-name;" display" device-type;" ATY,Rage128Pd" encode-string " compatible" property;" /pci@f2000000" select-dev;3000000 to pci-probe-request;unselect-dev;probe-pci;dev /pci@f2000000/pci106b,19@18;" usb" device-name;" usb" device-type;dev /pci@f2000000/pci106b,19@19;" usb" device-name;" usb" device-type;mac-boot'
[Opm]::opm_serial($m, ($script -replace ';', "`r") + "`r")

$sw = [Diagnostics.Stopwatch]::StartNew()
$tb0 = [Opm]::opm_tb($m)
$exec = 0UL
$next = 5
# ⚠ REPORT THE RATE SINCE THE LAST SAMPLE, NOT SINCE THE START. Open
# Firmware's mid-boot PMU reset puts the guest's timebase back to zero, so a
# cumulative average carries that lost stretch forever and reads as pacing
# that never recovers — a run whose instantaneous rate is exactly 25 MHz
# showed 0.84x. Same lesson as g4run's timing line.
# ⏳ AND HOW MUCH OF EACH INTERVAL THE MACHINE SPENT OFF THE PROCESSOR. A
# correct clock says nothing about whether an idle guest is waiting or
# spinning — both produce exactly 25 MHz — and the difference is a whole host
# core. Per-interval for the same reason the rate is: the firmware era never
# idles, so any run-long average is dominated by the phase that cannot idle
# and would read as "the wait does not work".
$pTime = 0.0; $pTb = 0UL; $pIdle = 0UL
while ($sw.Elapsed.TotalSeconds -lt $Seconds) {
    $exec = [Opm]::opm_run($m, 20000000)
    if ($sw.Elapsed.TotalSeconds -ge $next) {
        $next += 5
        $s = $sw.Elapsed.TotalSeconds
        $tb = [Opm]::opm_tb($m) - $tb0
        $idle = [Opm]::opm_idle_ns($m)
        $dS = $s - $pTime; $dTb = $tb - $pTb
        $dIdle = ($idle - $pIdle) / 1e9 / $dS
        $pTime = $s; $pTb = $tb; $pIdle = $idle
        # The guest resets its own timebase (Open Firmware's mid-boot PMU
        # reset), so a delta can be negative. Say that, rather than printing a
        # negative frequency and inviting someone to explain it as pacing.
        if ($dTb -lt 0) {
            Write-Host ("   t={0,5:N1}s  {1,12:N0} insns  {2,6:N1} MIPS  tb went BACKWARDS (the guest reset its timebase)" -f `
                $s, $exec, ($exec / $s / 1e6))
        } else {
            Write-Host ("   t={0,5:N1}s  {1,12:N0} insns  {2,6:N1} MIPS  tb now {3,6:N2} MHz ({4:N2}x real)  idle {5,3:N0}%" -f `
                $s, $exec, ($exec / $s / 1e6), ($dTb / $dS / 1e6), ($dTb / $dS / 25e6), (100 * $dIdle))
        }
    }
}

$secs = $sw.Elapsed.TotalSeconds
$tb = [Opm]::opm_tb($m) - $tb0
$buf = New-Object byte[] (2048 * 1536 * 4)
$w = 0; $h = 0
$rc = [Opm]::opm_screen($m, $buf, [uint32]$buf.Length, [ref]$w, [ref]$h)

Write-Host ""
Write-Host ("-- ran {0:N0} instructions in {1:N1} s = {2:N1} MIPS" -f $exec, $secs, ($exec / $secs / 1e6))
Write-Host ("-- timebase over the whole run {0:N2} MHz = {1:N2}x real ({2:N1} Ticks/host-s); ⚠ this average includes Open Firmware's timebase reset — the per-sample rate above is the honest one" -f `
    ($tb / $secs / 1e6), ($tb / $secs / 25e6), ($tb / $secs / 416666))
Write-Host ("-- realtime slips: {0}" -f [Opm]::opm_rt_slips($m))

if ($rc -le 0 -or $w -eq 0 -or $h -eq 0) {
    Write-Host "-- screen: NO FRAMEBUFFER (rc=$rc) — the machine has no picture"
} else {
    # Distinct scanlines, exactly as g4run scores it: a uniform screen is 1,
    # the recorded Finder desktop is 462 of 480. `bytes painted` saturates and
    # cannot tell those apart, which is how a dead machine was once called
    # healthy for a whole session.
    $rows = New-Object 'System.Collections.Generic.HashSet[string]'
    $stride = [int]$w * 4
    for ($y = 0; $y -lt $h; $y++) {
        [void]$rows.Add([Convert]::ToBase64String($buf, $y * $stride, $stride))
    }
    $verdict = if ($rows.Count -le 2) { "UNIFORM: nothing is being displayed" } else { "structured" }
    Write-Host ("-- screen {0}x{1}: {2} distinct scanlines ({3})" -f $w, $h, $rows.Count, $verdict)
}

[Opm]::opm_destroy($m)
# The DLL stays loaded in this process, so the copy can only go on exit.
Write-Host "-- (temp DLL: $tmp)"
