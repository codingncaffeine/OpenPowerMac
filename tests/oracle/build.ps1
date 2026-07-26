# Build the oracle corpus: guest ELFs/flats (ppc32be, MSYS2 clang+lld) and
# host reference executables (same C, -DHOST, MSYS2 gcc).
# Files named int*.c are built -mno-altivec (pure integer phases).
param(
  [string]$OutDir = "$PSScriptRoot\..\..\build\oracle"
)

$env:Path = 'C:\msys64\mingw64\bin;' + $env:Path  # gcc/clang need their DLLs
$clang = 'C:\msys64\mingw64\bin\clang.exe'
$gcc = 'C:\msys64\mingw64\bin\gcc.exe'
$objcopy = 'C:\msys64\mingw64\bin\llvm-objcopy.exe'
$src = "$PSScriptRoot\src"
$ld = "$PSScriptRoot\link.ld"

New-Item -ItemType Directory -Force $OutDir | Out-Null

foreach ($c in Get-ChildItem $src -Filter *.c) {
  $vecFlag = if ($c.BaseName -like 'int*') { '-mno-altivec' } else { '-maltivec' }
  foreach ($opt in '-O0', '-O2') {
    $name = "$($c.BaseName)$opt"
    $elf = Join-Path $OutDir "$name.elf"
    & $clang --target=powerpc-none-eabi -mcpu=7400 $vecFlag `
        -ffreestanding -nostdlib -fno-builtin -fuse-ld=lld `
        "-Wl,-T,$ld" '-Wl,--build-id=none' $opt $c.FullName -o $elf
    if ($LASTEXITCODE -ne 0) { throw "clang failed on $name" }
    & $objcopy -O binary $elf (Join-Path $OutDir "$name.bin")
    if ($LASTEXITCODE -ne 0) { throw "objcopy failed on $name" }
    Write-Host "built $name"
  }
  $hostExe = Join-Path $OutDir "$($c.BaseName)-host.exe"
  & $gcc -DHOST -O2 $c.FullName -o $hostExe
  if ($LASTEXITCODE -ne 0) { throw "host gcc failed on $($c.BaseName)" }
  Write-Host "built $($c.BaseName)-host"
}
