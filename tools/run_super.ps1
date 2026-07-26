# Build and run the supervisor test kernels; assert their exact output.
param(
  [string]$Repo = "$PSScriptRoot\..",
  [string]$Ppcrun = "$PSScriptRoot\..\build\tools\ppcrun\Release\ppcrun.exe"
)

$env:Path = 'C:\msys64\mingw64\bin;' + $env:Path
$out = Join-Path $Repo 'build\super'
New-Item -ItemType Directory -Force $out | Out-Null

$kernels = @(
  @{ Name = 'kernel1'; Want = "S1`nI1`nT1`nD3`nA1`nOK`n" },
  @{ Name = 'kernel2'; Want = "M1`nC1`nP1`nF1`nOK`n" },
  @{ Name = 'kernel3'; Want = "N1`nT1`nOK`n" }
)

$fail = 0
foreach ($k in $kernels) {
  $name = $k.Name
  & 'C:\msys64\mingw64\bin\clang.exe' --target=powerpc-none-eabi -mcpu=7400 `
      -nostdlib -ffreestanding -fuse-ld=lld `
      "-Wl,-T,$Repo\tests\oracle\link.ld" '-Wl,--build-id=none' `
      "$Repo\tests\super\$name.S" -o "$out\$name.elf"
  if ($LASTEXITCODE -ne 0) { throw "$name build failed" }

  $got = (& $Ppcrun --max 10000000 "$out\$name.elf" 2>$null | Out-String) -replace "`r", ''
  if ($got -eq $k.Want) {
    Write-Host "PASS $name"
  } else {
    Write-Host "FAIL $name`n--- got ---`n$got--- want ---`n$($k.Want)"
    & $Ppcrun --max 10000000 "$out\$name.elf" 2>&1 1>$null | Select-Object -Last 8
    $fail++
  }
}
exit $fail
