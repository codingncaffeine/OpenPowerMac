#!/bin/bash
# THE ACCELERATED-DESKTOP GATE: boot the INSTALLED disk, with NO CD, and score
# the machine on its system-call rate.
#
# This gate did not exist before session 31, and without it the desktop hang
# looked like an interactive bug for two sessions. What it measures:
#
#   healthy boot/desktop   1 SYSCALL per ~20,000 instructions
#   the hang               1 SYSCALL per ~152        (~140x, measured twice)
#
# The hang is Mac OS 9's ATI Graphics Accelerator waiting on a Rage 128 2D
# engine; see _plans/R128_ENGINE_HANDOFF.md. It reproduces HEADLESSLY, on its
# own, between 8.50 G and 8.75 G instructions.
#
# ⚠⚠ THE IMAGE IS COPIED ON EVERY RUN AND THAT IS NOT OPTIONAL. Mac OS writes
# to the disk while it boots, so a second run against the same copy is a
# DIFFERENT MACHINE -- one such run silently failed to reproduce the bug and
# cost twenty minutes. The copy is what makes this a fixed point, so it lives
# in the script rather than in a note somebody has to remember.
#
# ⚠ NO --cd. Open Firmware prefers a bootable CD, so leaving run.sh's --cd in
# boots the INSTALLER instead of the installed system, and the bug is not on
# that path.
#
# ⚠ hd.img DOES NOT REPRODUCE IT: it has no ATI Graphics Accelerator and the
# full install does. A disk that boots is not a disk that boots the same
# software -- and the two images carry different Mac OS ROM builds, so their
# baselines are not comparable to each other either.
#
#   tools/hanggate.sh                            # the gate, to 12 G
#   tools/hanggate.sh --ati-engine-log 400000    # + the engine command stream
#   OPM_MAX=9000000000 tools/hanggate.sh         # a shorter window
#
set -uo pipefail
cd "$(dirname "$0")/.." || exit 1

SRC="${OPM_INSTALLED:-/d/OpenPowerMac Drives/macos9_2gb.img}"
COPY="${OPM_HANG_COPY:-../scratch/openpowermac/macos9_hang.img}"

[ -f "$SRC" ] || { echo "FAIL: no installed disk at $SRC"; exit 1; }

echo "-- copying $SRC -> $COPY (mandatory: the guest writes as it boots)"
cp "$SRC" "$COPY" || { echo "FAIL: copy"; exit 1; }

# Same Forth as run.sh: name the ATI and USB nodes so Open Firmware's probe
# publishes them, then boot. Kept in sync by hand -- if run.sh's script
# changes, this one has to change with it or the two timelines diverge.
DEFAULT_SERIAL='" /pci@f0000000" select-dev;10 8000 probe-pci-device;8000 10 probe-pci-device;unselect-dev;dev /pci@f0000000/pci1002,5046@10;" ATY,Rage128Pd" device-name;" display" device-type;" ATY,Rage128Pd" encode-string " compatible" property;" /pci@f2000000" select-dev;3000000 to pci-probe-request;unselect-dev;probe-pci;dev /pci@f2000000/pci106b,19@18;" usb" device-name;" usb" device-type;dev /pci@f2000000/pci106b,19@19;" usb" device-name;" usb" device-type;mac-boot'

exec "${OPM_BIN:-./build/tools/g4run/Release/g4run.exe}" \
 --rom "../scratch/openpowermac/roms/newworld/sawtooth_4.2.8f1_stock.rom" \
 --exc 0 --fast-tb "${OPM_FAST_TB:-4}" \
 --hd "$COPY" \
 --ati-rom "../scratch/openpowermac/ati/ati_oem_rage128pro_136_agp_full.rom" \
 --ati-at "${OPM_ATI_AT:-1}" \
 --serial-input "${OPM_SERIAL-$DEFAULT_SERIAL}" \
 --max "${OPM_MAX:-12000000000}" --heartbeat "${OPM_HEARTBEAT:-250000000}" \
 "$@"
