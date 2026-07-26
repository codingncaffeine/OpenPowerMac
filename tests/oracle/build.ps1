# Build the oracle corpus for ppc32be with MSYS2 clang + lld.
# Usage: build.ps1 [-OutDir <dir>]
param(
  [string]$OutDir = "$PSScriptRoot\..\..\build\oracle"
)

$clang = 'C:\msys64\mingw64\bin\clang.exe'
$objcopy = 'C:\msys64\mingw64\bin\llvm-objcopy.exe'
$src = "$PSScriptRoot\src"
$ld = "$PSScriptRoot\link.ld"

New-Item -ItemType Directory -Force $OutDir | Out-Null

$common = @(
  '--target=powerpc-none-eabi', '-mcpu=7400', '-maltivec',
  '-ffreestanding', '-nostdlib', '-fno-builtin',
  '-fuse-ld=lld', "-Wl,-T,$ld", '-Wl,--build-id=none'
)

foreach ($c in Get-ChildItem $src -Filter *.c) {
  foreach ($opt in '-O0', '-O2') {
    $name = "$($c.BaseName)$opt"
    $elf = Join-Path $OutDir "$name.elf"
    & $clang @common $opt $c.FullName -o $elf
    if ($LASTEXITCODE -ne 0) { throw "clang failed on $name" }
    & $objcopy -O binary $elf (Join-Path $OutDir "$name.bin")
    if ($LASTEXITCODE -ne 0) { throw "objcopy failed on $name" }
    Write-Host "built $name"
  }
}
