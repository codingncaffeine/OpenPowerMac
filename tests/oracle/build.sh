#!/usr/bin/env bash
# Linux corpus build (CI oracle leg): guest via GNU powerpc-linux-gnu-gcc —
# a second, unrelated compiler reading the same ISA — host via the native cc.
set -euo pipefail

out="${1:-build/oracle}"
here="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$out"

for c in "$here"/src/*.c; do
  b="$(basename "$c" .c)"
  # int*/fp* = scalar phases. -mcpu=7400 alone implies -maltivec in GNU gcc,
  # and its -O2 auto-vectorizer will happily emit AltiVec for scalar loops —
  # so vector codegen must be disabled explicitly (as build.ps1 does) until
  # the P5 executors exist. FP corpus additionally forbids fma contraction on
  # both legs so guest and host compute the same unfused sequence.
  vec="-maltivec -mabi=altivec"
  case "$b" in int* | fp*) vec="-mno-altivec" ;; esac
  for opt in -O0 -O2; do
    powerpc-linux-gnu-gcc -mcpu=7400 $vec -ffp-contract=off \
      -ffreestanding -nostdlib -fno-builtin \
      -Wl,-T,"$here/link.ld" -Wl,--build-id=none $opt "$c" -o "$out/$b$opt.elf"
    echo "built $b$opt"
  done
  cc -DHOST -O2 -ffp-contract=off "$c" -o "$out/$b-host"
  echo "built $b-host"
done
