# Disassembler oracle: diff ppcrun --disasm against llvm-objdump on an ELF.
# Tokens are normalized (register prefixes stripped, numbers canonicalized)
# so style differences don't count as divergence — only decode/mnemonic/operand
# disagreements do.
param(
  [Parameter(Mandatory)][string]$Elf,
  [string]$Ppcrun = "$PSScriptRoot\..\build\tools\ppcrun\Release\ppcrun.exe",
  [string]$Objdump = 'C:\msys64\mingw64\bin\llvm-objdump.exe',
  [int]$ShowMismatches = 25
)

function Normalize([string]$mnem, [string]$ops) {
  $ops = ($ops -split '<')[0]   # drop "<symbol+0x..>" annotations
  $toks = @()
  foreach ($t in ($ops -split '[,\s()]+' | Where-Object { $_ -ne '' })) {
    $tt = $t
    if ($tt -match '^(cr|r|f|v)(\d+)$') { $tt = $Matches[2] }
    if ($tt -match '^(-?)0x([0-9a-fA-F]+)$') {
      $v = [convert]::ToUInt64($Matches[2], 16)
      if ($Matches[1] -eq '-') { $tt = "-$v" } else { $tt = "$v" }
    }
    $toks += $tt
  }
  return "$mnem $($toks -join ' ')"
}

# llvm-objdump lines: "  100000: 7c 22 1a 14   add 1, 2, 3"
$ref = @{}
& $Objdump -d --mcpu=7400 --mattr=+altivec $Elf | ForEach-Object {
  if ($_ -match '^\s*([0-9a-f]+):\s+([0-9a-f]{2}\s[0-9a-f]{2}\s[0-9a-f]{2}\s[0-9a-f]{2})\s+(\S+)\s*(.*)$') {
    $addr = [convert]::ToUInt32($Matches[1], 16)
    $ref[$addr] = Normalize $Matches[3] $Matches[4]
  }
}

# ppcrun lines: "  100000: 7c221a14  add 1, 2, 3"
$mine = @{}
& $Ppcrun --disasm --style llvm $Elf | ForEach-Object {
  if ($_ -match '^\s*([0-9a-f]+):\s+([0-9a-f]{8})\s+(\S+)\s*(.*)$') {
    $addr = [convert]::ToUInt32($Matches[1], 16)
    $mine[$addr] = Normalize $Matches[3] $Matches[4]
  }
}

$total = 0; $match = 0; $unk = 0; $mismatches = @()
foreach ($addr in $ref.Keys) {
  if (-not $mine.ContainsKey($addr)) { continue }
  $total++
  $a = $ref[$addr]; $b = $mine[$addr]
  if ($b -like '.long*' -or $a -like '*unknown*') { $unk++; continue }
  if ($a -eq $b) { $match++ }
  else { $mismatches += ('{0:x8}: ref[{1}]  mine[{2}]' -f $addr, $a, $b) }
}

$pct = if ($total -gt 0) { [math]::Round(100.0 * $match / ($total - $unk), 2) } else { 0 }
Write-Host "compared $total insns: $match match, $($total - $unk - $match) differ, $unk unknown-side ($pct% agreement)"
$mismatches | Select-Object -First $ShowMismatches | ForEach-Object { Write-Host $_ }
if ($pct -lt 99.0) { exit 1 } else { exit 0 }
