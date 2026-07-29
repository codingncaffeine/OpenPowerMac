# Scan the boot ROM for PowerPC instructions matching a pattern.
#
# The firmware decompiler reads Open Firmware's Forth dictionary; this reads
# the ROM's own PowerPC. Two questions it answers directly: who branches to a
# given address, and which routine stores through a base register loaded with
# a given constant.
#
#   pwsh tools/romscan.ps1 -Rom <file> -BranchTo 0xfff80184
#   pwsh tools/romscan.ps1 -Rom <file> -LisBase 0x0010 -StoreDisp 4
#
param(
    # int64, because PowerShell reads 0xfff00000 as a NEGATIVE int32 and a
    # uint32 parameter then refuses the only address anyone will ever pass.
    [Parameter(Mandatory=$true)][string]$Rom,
    [int64]$Base = 0xfff00000,
    [int64]$BranchTo = 0,
    [int]$LisBase = -1,        # e.g. 0x0010 for `lis rX,0x0010`
    [int]$StoreDisp = -1,      # displacement to look for after that lis
    [int]$Window = 64,         # instructions to look ahead
    [int]$Spr = -1             # every mfspr/mtspr of this SPR (1017 = L2CR)
)

$ErrorActionPreference = 'Stop'
$b = [System.IO.File]::ReadAllBytes($Rom)
$n = [int]($b.Length / 4)
Write-Host "-- $Rom : $($b.Length) bytes at $('{0:x8}' -f $Base)"

$w = New-Object 'uint32[]' $n
for ($i = 0; $i -lt $n; $i++) {
    $o = $i * 4
    $w[$i] = ([uint32]$b[$o] -shl 24) -bor ([uint32]$b[$o+1] -shl 16) -bor
             ([uint32]$b[$o+2] -shl 8) -bor [uint32]$b[$o+3]
}
function VA([int]$i) { return [int64]($Base + $i * 4) }

if ($BranchTo -ne 0) {
    Write-Host "-- branches to $('{0:x8}' -f $BranchTo):"
    $hits = 0
    for ($i = 0; $i -lt $n; $i++) {
        $ins = $w[$i]
        $op = $ins -shr 26
        $tgt = 0; $kind = ''
        if ($op -eq 18) {                      # b / bl / ba / bla
            $li = [int64]($ins -band 0x03FFFFFC)
            if ($li -band 0x02000000) { $li = $li - 0x04000000 }
            $aa = ($ins -band 2) -ne 0
            $lk = ($ins -band 1) -ne 0
            $tgt = if ($aa) { [int64]$li } else { [int64]((VA $i) + $li) }
            $kind = if ($lk) { 'bl' } else { 'b ' }
        } elseif ($op -eq 16) {                # bc / bcl
            $bd = [int64]($ins -band 0x0000FFFC)
            if ($bd -band 0x8000) { $bd = $bd - 0x10000 }
            $aa = ($ins -band 2) -ne 0
            $lk = ($ins -band 1) -ne 0
            $tgt = if ($aa) { [int64]$bd } else { [int64]((VA $i) + $bd) }
            $kind = if ($lk) { 'bcl' } else { 'bc ' }
        } else { continue }
        if ($tgt -eq $BranchTo) {
            Write-Host ("   {0:x8}  {1} {2:x8}" -f (VA $i), $kind, $tgt)
            $hits++
        }
    }
    if ($hits -eq 0) { Write-Host "   <none>" }
}

if ($Spr -ge 0) {
    # The SPR number is encoded with its halves SWAPPED in bits 11-20, which
    # is why a naive search for "1017" finds nothing.
    $enc = (($Spr -band 31) -shl 5) -bor (($Spr -shr 5) -band 31)
    $want = [uint32](($enc -shl 11))
    Write-Host ("-- mfspr/mtspr of SPR {0} (encoded field {1:x3}):" -f $Spr, $enc)
    $hits = 0
    for ($i = 0; $i -lt $n; $i++) {
        $ins = $w[$i]
        if (($ins -shr 26) -ne 31) { continue }
        if (($ins -band 0x001FF800) -ne $want) { continue }
        $xo = ($ins -shr 1) -band 0x3FF
        $kind = if ($xo -eq 339) { 'mfspr' } elseif ($xo -eq 467) { 'mtspr' } else { continue }
        Write-Host ("   {0:x8}  {1} r{2}" -f (VA $i), $kind, (($ins -shr 21) -band 31))
        $hits++
    }
    if ($hits -eq 0) { Write-Host "   <none>" }
}

if ($LisBase -ge 0) {
    Write-Host ("-- `lis rX,{0:x4}` followed within {1} insns by a store at disp {2}:" -f `
        $LisBase, $Window, $StoreDisp)
    # addis rD,0,IMM  ==  lis: opcode 15, rA field == 0
    $storeOps = @{ 36 = 'stw'; 38 = 'stb'; 44 = 'sth'; 37 = 'stwu'; 39 = 'stbu'; 45 = 'sthu' }
    $hits = 0
    for ($i = 0; $i -lt $n; $i++) {
        $ins = $w[$i]
        if (($ins -shr 26) -ne 15) { continue }
        if ((($ins -shr 16) -band 31) -ne 0) { continue }       # rA must be 0
        if (($ins -band 0xFFFF) -ne $LisBase) { continue }
        $rd = ($ins -shr 21) -band 31
        for ($k = 1; $k -le $Window -and $i + $k -lt $n; $k++) {
            $s = $w[$i + $k]
            $sop = [int]($s -shr 26)
            if (-not $storeOps.ContainsKey($sop)) { continue }
            if ((($s -shr 16) -band 31) -ne $rd) { continue }
            $d = [int]($s -band 0xFFFF)
            if ($d -band 0x8000) { $d = $d - 0x10000 }
            if ($StoreDisp -ge 0 -and $d -ne $StoreDisp) { continue }
            Write-Host ("   {0:x8} lis r{1},{2:x4}   ->   {3:x8} {4} r{5},{6}(r{1})" -f `
                (VA $i), $rd, $LisBase, (VA ($i + $k)), $storeOps[$sop],
                (($s -shr 21) -band 31), $d)
            $hits++
        }
    }
    if ($hits -eq 0) { Write-Host "   <none>" }
}
