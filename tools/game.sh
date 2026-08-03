#!/bin/bash
# THE IN-GAME WINDOW, parameterized — run.sh's sibling for the 3D title.
#
# run.sh exists because retyping the boot command shifted the timeline by
# ~100M instructions whenever a character moved. The game window has the same
# problem and worse consequences: it resumes a SNAPSHOT, so a wrong --ram or a
# shared HD image does not fail loudly, it produces a different machine that
# still prints plausible numbers. Every session since 37 has retyped this out
# of a pickup file. Now it is a fixed point.
#
#   tools/game.sh                       # moment2 + 200M, per-run HD copy
#   tools/game.sh --bench               # …with the speed line
#   OPM_SNAP=moment1 tools/game.sh      # the other capture
#   OPM_PLUS=50000000 tools/game.sh     # a shorter window
#   OPM_TAG=b tools/game.sh             # a second concurrent run (own HD copy)
#
# ⚠ RAM MUST BE 1536 — a snapshot carries its RAM size and resuming at another
# is not the same machine. ⚠ THE HD IMAGE IS WRITABLE, so every run gets its
# own copy; two runs sharing one corrupt each other silently.
# Extra flags are appended verbatim.
cd "$(dirname "$0")/.." || exit 1

SNAP_DIR="${OPM_SNAP_DIR:-/d/Open Power Mac Snapshots}"
SNAP_NAME="${OPM_SNAP:-moment2}"
SNAP="$SNAP_DIR/$SNAP_NAME.snap"
[ -f "$SNAP" ] || { echo "FAIL: snapshot missing: $SNAP (set OPM_SNAP/OPM_SNAP_DIR)"; exit 1; }

# The instruction count each capture was taken at — the resume point. --max is
# absolute, so the window is AT + PLUS.
case "$SNAP_NAME" in
    moment1) AT=15524809144 ;;
    moment2) AT=14701959456 ;;
    *) AT="${OPM_AT:-}" ;;
esac
[ -n "$AT" ] || { echo "FAIL: unknown snapshot $SNAP_NAME — set OPM_AT to its instruction count"; exit 1; }
PLUS="${OPM_PLUS:-200000000}"

ROM="${OPM_ROM:-/c/Users/gamer/Downloads/Open Power Mac/ROM 4.2.8f1 G4 AGP Sawtooth Stock/ROMs/@FFF00000 len-100000 All.rom}"
CD="${OPM_CD:-/c/Users/gamer/Downloads/nanosaur/nanosaur_hfs.img}"
HD_SRC="${OPM_HD_SRC:-/d/OpenPowerMac Drives/macos9_2gb.img}"
HD="${OPM_HD:-/d/OpenPowerMac Drives/_iter_hd${OPM_TAG:+_$OPM_TAG}.img}"
for f in "$ROM" "$CD" "$HD_SRC"; do
    [ -f "$f" ] || { echo "FAIL: missing media: $f"; exit 1; }
done
# Fresh disk per run unless the caller supplied one it owns.
#
# ⚠ FOR A SPEED SERIES, SUPPLY THE IMAGE (OPM_HD) AND COPY IT ONCE YOURSELF.
# This copy is 2 GB, and Windows finishes writing it back DURING the run that
# follows — a 3-second measurement window next to 2 GB of writeback read as a
# 20% regression once, and the tell was that the control run (which happened
# to land after the writeback drained) was rock steady while the run under
# test swung 59-74 MIPS. The window's own determinism is what makes reuse
# safe: every run of it reports the same tb advance to the tick whatever the
# image holds, because the guest is not touching the disk here.
if [ -z "${OPM_HD:-}" ]; then
    cp "$HD_SRC" "$HD" || exit 1
fi

exec "${OPM_BIN:-./build/tools/g4run/Release/g4run.exe}" \
 --rom "$ROM" \
 --ram 1536 --exc 0 --fast-tb "${OPM_FAST_TB:-4}" \
 --hd "$HD" \
 --cd "$CD" \
 --ati-rom ../scratch/openpowermac/ati/ati_oem_rage128pro_136_agp_full.rom \
 --ati-at 1 \
 --resume-from "$SNAP" \
 --max "$((AT + PLUS))" \
 "$@"
