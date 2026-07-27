#!/bin/bash
# Annotate 68K trace addresses with the Mac OS ROM's own MacsBug symbols.
#
# Classic 68K code carries a symbol name after each function's RTS, so the
# loaded Mac OS ROM in a RAM dump is self-describing. Build a table of
# (address, name) from the dump, then rewrite every ffdxxxxx / ffcxxxxx
# address in a log to "addr<Symbol+offset>", which turns hex archaeology
# into readable flow.
#
#   tools/symbolize.sh <ram-dump> <logfile> [rom-va-base] [rom-pa-base]
#
# Defaults match this machine: the Mac OS ROM is staged at PA 0x00c00000
# and runs at VA 0xffc00000.
set -e
DUMP=${1:?usage: symbolize.sh <ram-dump> <log> [va-base] [pa-base]}
LOG=${2:?usage: symbolize.sh <ram-dump> <log> [va-base] [pa-base]}
VA=${3:-0xffc00000}
PA=${4:-0x00c00000}
LEN=${5:-$((0x200000))}

SYMS=$(mktemp)
trap 'rm -f "$SYMS"' EXIT

# A MacsBug symbol is a length byte with the high bit set followed by the
# name, sitting just past a function's RTS. Accept plausible identifiers
# and keep the address they were found at.
dd if="$DUMP" bs=1 skip=$((PA)) count=$((LEN)) 2>/dev/null |
  grep -aboE '[A-Za-z_][A-Za-z0-9_.]{3,30}' |
  while IFS=: read -r off name; do
    prev=$(dd if="$DUMP" bs=1 skip=$((PA + off - 1)) count=1 2>/dev/null |
           od -An -tu1 | tr -d ' \n')
    [ -n "$prev" ] && [ "$prev" -ge 128 ] &&
      printf '%d %s\n' $((VA + off)) "$name"
  done | sort -n > "$SYMS"

printf 'symbol table: %s entries\n' "$(wc -l < "$SYMS")" >&2

# Rewrite each ROM address in the log to addr<Symbol+offset>.
awk -v symfile="$SYMS" '
  BEGIN {
    n = 0
    while ((getline line < symfile) > 0) {
      split(line, f, " ")
      addr[n] = f[1] + 0; name[n] = f[2]; n++
    }
  }
  function lookup(a,   lo, hi, mid, best) {
    lo = 0; hi = n - 1; best = -1
    while (lo <= hi) {
      mid = int((lo + hi) / 2)
      if (addr[mid] <= a) { best = mid; lo = mid + 1 } else { hi = mid - 1 }
    }
    if (best < 0) return ""
    return sprintf("<%s+0x%x>", name[best], a - addr[best])
  }
  {
    out = ""
    while (match($0, /ff[cd][0-9a-f]{5}/)) {
      pre = substr($0, 1, RSTART - 1)
      hex = substr($0, RSTART, RLENGTH)
      $0  = substr($0, RSTART + RLENGTH)
      out = out pre hex lookup(strtonum("0x" hex))
    }
    print out $0
  }
' "$LOG"
