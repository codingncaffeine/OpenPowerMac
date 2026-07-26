#!/usr/bin/env bash
# Linux oracle runner: guest (ppcrun) output must be byte-identical to the
# host build of the same C source.
set -uo pipefail

dir="${1:-build/oracle}"
ppcrun="${2:-build/tools/ppcrun/ppcrun}"
filter="${3:-[fim]*}" # int* + fp* + mix* (scalar and vector corpus)

fail=0
ran=0
for elf in "$dir"/$filter.elf; do
  [ -e "$elf" ] || continue
  base="$(basename "$elf" .elf)"
  base="${base%-O0}"
  base="${base%-O2}"
  hostexe="$dir/$base-host"
  [ -x "$hostexe" ] || continue
  ran=$((ran + 1))
  guest="$("$ppcrun" --max 200000000 "$elf" 2>/dev/null)"
  code=$?
  host="$("$hostexe")"
  if [ $code -ne 0 ]; then
    echo "FAIL $(basename "$elf"): guest exit $code"
    "$ppcrun" --max 200000000 "$elf" 2>&1 >/dev/null | head -12
    echo "-- disassembly ($(basename "$elf")):"
    "$ppcrun" --disasm "$elf"
    fail=$((fail + 1))
  elif [ "$guest" != "$host" ]; then
    echo "FAIL $(basename "$elf"): output differs"
    diff <(echo "$host") <(echo "$guest") | head -20
    fail=$((fail + 1))
  else
    echo "PASS $(basename "$elf")"
  fi
done
echo "$ran run, $fail failed"
[ $fail -eq 0 ] && [ $ran -gt 0 ]
