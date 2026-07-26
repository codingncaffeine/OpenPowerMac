# Build and run the supervisor test kernel; assert its exact output.
param(
  [string]$Repo = "$PSScriptRoot\..",
  [string]$Ppcrun = "$PSScriptRoot\..\build\tools\ppcrun\Release\ppcrun.exe"
)

$env:Path = 'C:\msys64\mingw64\bin;' + $env:Path
$out = Join-Path $Repo 'build\super'
New-Item -ItemType Directory -Force $out | Out-Null

& 'C:\msys64\mingw64\bin\clang.exe' --target=powerpc-none-eabi -mcpu=7400 `
    -nostdlib -ffreestanding -fuse-ld=lld `
    "-Wl,-T,$Repo\tests\oracle\link.ld" '-Wl,--build-id=none' `
    "$Repo\tests\super\kernel1.S" -o "$out\kernel1.elf"
if ($LASTEXITCODE -ne 0) { throw 'kernel1 build failed' }

$got = (& $Ppcrun --max 10000000 "$out\kernel1.elf" 2>$null | Out-String) -replace "`r", ''
$want = "S1`nI1`nT1`nD3`nA1`nOK`n"
if ($got -eq $want) {
  Write-Host 'PASS kernel1'
  exit 0
} else {
  Write-Host "FAIL kernel1`n--- got ---`n$got--- want ---`n$want"
  & $Ppcrun --max 10000000 "$out\kernel1.elf" 2>&1 1>$null | Select-Object -Last 8
  exit 1
}
