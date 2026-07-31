#!/bin/bash
# THE STATE DIFFERENTIAL: the same window run two ways, compared as whole
# MACHINES.
#
# Every "this is transparent" claim in this tree has so far been settled by
# running with the machinery off and comparing four report lines — stop pc,
# timebase, disk commands, painted bytes. That is a real gate and it is a
# COARSE one: four numbers out of a 256 MB machine, and only at the end of a
# run. It cannot tell you that a block executor got a condition register wrong
# in a routine whose result is later overwritten.
#
# This compares everything. --fingerprint-every N makes g4run print FNV-1a over
# the complete serialized machine — RAM, every register, every device cell — at
# exact instruction counts, and the batch is clamped to land on each mark so
# both runs are photographed at the same instruction. The first differing line
# names the window the divergence happened in; narrow it by re-running that
# window with a smaller mark.
#
#   tools/diffstate.sh --max 200000000 --every 20000000
#   tools/diffstate.sh --from ../scratch/openpowermac/desk28_ft4.snap \
#                      --max 15200000000 --every 25000000
#
# Anything after the recognised options is passed to BOTH runs. Run B always
# adds the controls (--no-batch --no-line-exec), so what is being compared is
# "the machine as it ships" against "the machine as it was before any of this".
set -uo pipefail
cd "$(dirname "$0")/.." || exit 1

FROM=""
MAX=200000000
EVERY=20000000
while [ $# -gt 0 ]; do
    case "$1" in
        --from)  FROM="$2"; shift 2 ;;
        --max)   MAX="$2";  shift 2 ;;
        --every) EVERY="$2"; shift 2 ;;
        *) break ;;
    esac
done

OUT="${TMPDIR:-/tmp}/opm-diffstate.$$"
mkdir -p "$OUT" || exit 1
RESUME=()
[ -n "$FROM" ] && RESUME=(--resume-from "$FROM")

# ⚠ The two runs must not share the writable disk image: a boot writes to it,
# and the second run would start from a volume the first one had already
# changed. Each gets its own copy.
HD_SRC="${OPM_HD:-../scratch/openpowermac/hd.img}"
cp "$HD_SRC" "$OUT/a.img" && cp "$HD_SRC" "$OUT/b.img" || exit 1

echo "== A: as it ships"
OPM_HD="$OUT/a.img" ./tools/run.sh --max "$MAX" --bench \
    --fingerprint-every "$EVERY" "${RESUME[@]}" "$@" \
    > "$OUT/a.log" 2>&1
echo "== B: --no-batch --no-line-exec"
OPM_HD="$OUT/b.img" ./tools/run.sh --max "$MAX" --bench \
    --fingerprint-every "$EVERY" --no-batch --no-line-exec \
    "${RESUME[@]}" "$@" > "$OUT/b.log" 2>&1

grep '^FP @' "$OUT/a.log" > "$OUT/a.fp"
grep '^FP @' "$OUT/b.log" > "$OUT/b.fp"
A=$(wc -l < "$OUT/a.fp")
B=$(wc -l < "$OUT/b.fp")
echo "-- $A marks vs $B"
if [ "$A" -eq 0 ] || [ "$B" -eq 0 ]; then
    echo "DIFFSTATE INCONCLUSIVE: a run produced no marks at all"
    tail -3 "$OUT/a.log" "$OUT/b.log"
    exit 2
fi
if diff -q "$OUT/a.fp" "$OUT/b.fp" > /dev/null; then
    echo "IDENTICAL: $A whole-machine fingerprints match"
    head -1 "$OUT/a.fp"; tail -1 "$OUT/a.fp"
    rm -rf "$OUT"
    exit 0
fi
echo "DIVERGED — first difference:"
diff "$OUT/a.fp" "$OUT/b.fp" | head -6
echo "-- logs kept in $OUT"
exit 1
