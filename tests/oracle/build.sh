#!/usr/bin/env bash
# Linux corpus build (CI oracle leg): guest via GNU powerpc-linux-gnu-gcc —
# a second, unrelated compiler reading the same ISA — host via the native cc.
set -euo pipefail

out="${1:-build/oracle}"
here="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$out"

for c in "$here"/src/*.c; do
  b="$(basename "$c" .c)"
  vec="-maltivec -mabi=altivec"
  case "$b" in int*) vec="" ;; esac
  for opt in -O0 -O2; do
    powerpc-linux-gnu-gcc -mcpu=7400 $vec -ffreestanding -nostdlib -fno-builtin \
      -Wl,-T,"$here/link.ld" -Wl,--build-id=none $opt "$c" -o "$out/$b$opt.elf"
    echo "built $b$opt"
  done
  cc -DHOST -O2 "$c" -o "$out/$b-host"
  echo "built $b-host"
done
