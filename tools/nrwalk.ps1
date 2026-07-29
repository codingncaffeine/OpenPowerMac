# Walk Mac OS's Name Registry out of a raw RAM dump.
#
# The registry is the linked structure the Expansion Bus Manager builds from
# Open Firmware's device tree, so it is the only way to see what the OS
# actually believes about a device once the console has moved to the screen
# and the interactive Forth route is closed.
#
# Layout, read off the dump rather than assumed:
#
#   EMDN  node      +8 parent, +12 next sibling, +16/+20 child head/tail,
#                   +24/+28 property head/tail, +32 name (inline C string)
#   EMPN  property  +8 owning node, +12 next, +16 length, +20 value,
#                   +24 name (inline C string)
#
# Both lists are CIRCULAR: the last element points back at the head.
# Every pointer is LOGICAL: PA = logical + 0x4000.
#
#   pwsh tools/nrwalk.ps1 -Ram ..\scratch\openpowermac\s16_nr.ram [-Match usb]
#
param(
    [Parameter(Mandatory=$true)][string]$Ram,
    [string]$Match = '',          # only print nodes whose path matches
    [switch]$NamesOnly,           # one line per node
    [int]$Bias = 0x4000
)

$ErrorActionPreference = 'Stop'
$bytes = [System.IO.File]::ReadAllBytes($Ram)
Write-Host "-- $Ram : $($bytes.Length) bytes, logical->PA bias 0x$('{0:x}' -f $Bias)"

function Be32([int]$pa) {
    if ($pa -lt 0 -or $pa + 4 -gt $bytes.Length) { return -1 }
    return [int64]$bytes[$pa] * 16777216 + [int64]$bytes[$pa+1] * 65536 +
           [int64]$bytes[$pa+2] * 256 + [int64]$bytes[$pa+3]
}
function Tag([int]$pa) {
    if ($pa -lt 0 -or $pa + 4 -gt $bytes.Length) { return '' }
    return [System.Text.Encoding]::ASCII.GetString($bytes, $pa, 4)
}
function CStr([int]$pa, [int]$cap = 64) {
    if ($pa -lt 0 -or $pa -ge $bytes.Length) { return '' }
    $e = $pa
    while ($e -lt $bytes.Length -and $e - $pa -lt $cap -and $bytes[$e] -ne 0) { $e++ }
    return [System.Text.Encoding]::ASCII.GetString($bytes, $pa, $e - $pa)
}
function ToPa([int64]$logical) {
    if ($logical -le 0) { return -1 }
    $pa = $logical + $Bias
    if ($pa -lt 0 -or $pa -ge $bytes.Length) { return -1 }   # MMIO, not RAM
    return [int]$pa
}

# Show a value as text when it is printable and NUL-terminated, which is how
# OF stores names and `compatible` (a NUL-separated list).
function ValueText([int]$pa, [int]$len) {
    if ($pa -lt 0 -or $len -le 0 -or $len -gt 512) { return $null }
    if ($pa + $len -gt $bytes.Length) { return $null }
    for ($i = 0; $i -lt $len; $i++) {
        $b = $bytes[$pa + $i]
        if ($b -ne 0 -and ($b -lt 0x20 -or $b -ge 0x7f)) { return $null }
    }
    if ($bytes[$pa + $len - 1] -ne 0) { return $null }
    $s = [System.Text.Encoding]::ASCII.GetString($bytes, $pa, $len - 1)
    return ($s -replace "`0", '|')
}
function ValueHex([int]$pa, [int]$len) {
    if ($pa -lt 0) { return '<null value ptr>' }
    if ($len -le 0) { return '<empty>' }
    $n = [Math]::Min($len, 80)
    if ($pa + $n -gt $bytes.Length) { return '<out of range>' }
    $sb = New-Object System.Text.StringBuilder
    for ($i = 0; $i -lt $n; $i++) {
        [void]$sb.Append('{0:x2}' -f $bytes[$pa + $i])
        if (($i % 4) -eq 3) { [void]$sb.Append(' ') }
    }
    if ($n -lt $len) { [void]$sb.Append('...') }
    return $sb.ToString().Trim()
}

# Find nodes by scanning for the tag: a node whose parent chain is broken
# still matters, and a scan cannot miss one.
$nodes = New-Object System.Collections.ArrayList
for ($pa = 0; $pa + 40 -lt $bytes.Length; $pa += 4) {
    if ($bytes[$pa] -ne 0x45) { continue }               # 'E'
    if ($bytes[$pa+1] -ne 0x4d -or $bytes[$pa+2] -ne 0x44 -or $bytes[$pa+3] -ne 0x4e) { continue }
    [void]$nodes.Add($pa)
}
Write-Host "-- $($nodes.Count) EMDN nodes"

# Node names first, so a path can be built from the parent links.
$nameOf = @{}
foreach ($npa in $nodes) { $nameOf[$npa] = CStr ($npa + 32) }
function PathOf([int]$npa) {
    $parts = @()
    $cur = $npa
    $guard = 0
    while ($cur -ge 0 -and $guard -lt 24) {
        $parts = ,($nameOf[$cur]) + $parts
        $cur = ToPa (Be32 ($cur + 8))
        $guard++
    }
    return ($parts -join '/')
}

$shown = 0
foreach ($npa in $nodes) {
    $path = PathOf $npa
    if ($Match -and $path -notmatch $Match) { continue }
    $shown++

    $props = New-Object System.Collections.ArrayList
    $head = ToPa (Be32 ($npa + 24))
    $p = $head
    $guard = 0
    while ($p -ge 0 -and $guard -lt 400) {
        if ((Tag $p) -ne 'EMPN') { break }
        [void]$props.Add([pscustomobject]@{
            Name = CStr ($p + 24)
            Len  = [int](Be32 ($p + 16))
            Vpa  = ToPa (Be32 ($p + 20))
        })
        $p = ToPa (Be32 ($p + 12))
        if ($p -eq $head) { break }          # circular list
        $guard++
    }

    Write-Host ""
    Write-Host ("=== {0}   node@{1:x8}   {2} properties" -f $path, $npa, $props.Count)
    if ($NamesOnly) { continue }
    foreach ($pr in $props) {
        $t = ValueText $pr.Vpa $pr.Len
        $v = if ($null -ne $t) { '"' + $t + '"' } else { ValueHex $pr.Vpa $pr.Len }
        Write-Host ("    {0,-30} {1,4}  {2}" -f $pr.Name, $pr.Len, $v)
    }
}
Write-Host ""
Write-Host "-- $shown node(s) shown"
