# Differential oracle: run guest ELFs under ppcrun and compare stdout against
# the host build of the same C source. Bit-identical or bust.
param(
  [string]$OracleDir = "$PSScriptRoot\..\build\oracle",
  [string]$Ppcrun = "$PSScriptRoot\..\build\tools\ppcrun\Release\ppcrun.exe",
  [string]$Filter = '[fi]*' # int* + fp* by default; mix* waits for P5
)

$fail = 0
$ran = 0
foreach ($elf in Get-ChildItem $OracleDir -File | Where-Object { $_.Name -like "$Filter.elf" }) {
  $base = $elf.BaseName -replace '-O[02]$', ''
  $hostExe = Join-Path $OracleDir "$base-host.exe"
  if (-not (Test-Path $hostExe)) { continue }
  $ran++
  $guestOut = & $Ppcrun --max 200000000 $elf.FullName 2>$null | Out-String
  $guestCode = $LASTEXITCODE
  $hostOut = & $hostExe | Out-String
  if ($guestCode -ne 0) {
    Write-Host "FAIL $($elf.BaseName): guest exit code $guestCode"
    & $Ppcrun --max 200000000 $elf.FullName 2>&1 1>$null | Select-Object -Last 6
    $fail++
  } elseif ($guestOut -ne $hostOut) {
    Write-Host "FAIL $($elf.BaseName): output differs"
    Write-Host "--- guest ---"; Write-Host $guestOut
    Write-Host "--- host ----"; Write-Host $hostOut
    $fail++
  } else {
    Write-Host "PASS $($elf.BaseName)"
  }
}
Write-Host "$ran run, $fail failed"
exit $(if ($fail -gt 0 -or $ran -eq 0) { 1 } else { 0 })
