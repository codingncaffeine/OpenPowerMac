#!/bin/bash
# Does the boot still reach the same landmarks?
#
# ctest covers the CPU exhaustively and the machine not at all. A device
# change can move the whole boot without breaking a single test: the DD7 fix
# moved it 830 MILLION instructions earlier, which silently disarmed every
# instrument gated on an instruction count, and the runs that followed were
# read as evidence. This asserts the landmarks and prints where each one now
# lands, so a shift is announced rather than discovered three runs later.
#
#   tools/milestones.sh [max-instructions]
#
# Needs the shelf images; it is a local check, not a CI one — CI has no ROM.
set -e
cd "$(dirname "$0")/.."

ROM=../scratch/openpowermac/roms/newworld/sawtooth_4.2.8f1_stock.rom
CD=/c/Users/gamer/Downloads/PowerMacG4.iso
ATI=../scratch/openpowermac/ati/ati_oem_rage128pro_136_agp_full.rom
G4=./build/tools/g4run/Release/g4run.exe
MAX=${1:-3000000000}

for f in "$ROM" "$CD" "$ATI" "$G4"; do
  [ -e "$f" ] || { echo "MISSING: $f"; exit 2; }
done

SCRIPT='" /pci@f0000000" select-dev;10 8000 probe-pci-device;8000 10 probe-pci-device;unselect-dev;dev /pci@f0000000/pci1002,5046@10;" ATY,Rage128Pd" device-name;" display" device-type;" ATY,Rage128Pd" encode-string " compatible" property;" /pci@f2000000" select-dev;3000000 to pci-probe-request;unselect-dev;probe-pci;dev /pci@f2000000/pci106b,19@18;" usb" device-name;" usb" device-type;dev /pci@f2000000/pci106b,19@19;" usb" device-name;" usb" device-type;mac-boot'

LOG=$(mktemp)
trap 'rm -f "$LOG"' EXIT
echo "-- running the flagship recipe to $MAX ..."
# ⚠ --fast-tb 60 does not reach the OS any more: the KeyLargo timer is now
# answered, so the guest's clock is correct and 60 leaves Mac OS less time per
# 60 Hz tick than its own tick work costs. 4 is the measured value. ⚠ --ati-at
# 236000000 is likewise stale — the reveal has to land before Open Firmware's
# PCI probe and the boot timeline moved past that count, which produces a
# machine with a working boot and no picture at all.
"$G4" --rom "$ROM" --max "$MAX" --exc 0 --fast-tb 4 --cd "$CD" \
  --ati-rom "$ATI" --ati-at 1 --serial-input "$SCRIPT" \
  --arm-at-park 200 --dump-structs > "$LOG" 2>&1 || true

fail=0
# name | pattern | whether absence is fatal
check() {
  local name=$1 pat=$2 fatal=$3
  local line
  line=$(grep -am1 -- "$pat" "$LOG" || true)
  if [ -n "$line" ]; then
    printf '  %-28s OK   %s\n' "$name" "$(printf '%s' "$line" | cut -c1-96)"
  else
    printf '  %-28s %s  (pattern: %s)\n' "$name" \
      "$([ "$fatal" = fatal ] && echo MISSING || echo absent )" "$pat"
    [ "$fatal" = fatal ] && fail=1
  fi
  return 0
}

echo "-- landmarks:"
check "OF banner"            "Welcome to Open Firmware"   fatal
check "device tree probed"   "pci-probe-request"          soft
check "68K world running"    "driver base"                fatal
check "park reached"         "ARMED on park"              fatal
check "ATA manager calls"    "ATAFN 90"                   fatal
check "driver registration"  "ATAFN 85"                   fatal
check "bus registration"     "ATAFN 93"                   fatal
check "on-disc driver call"  "^DRV>"                      fatal
check "drive queue element"  "drive\[0\]"                 soft
check "display modeset"      "ati crtc"                   soft
check "_DrvrInstall reached" 'ATRAP \$A03d'               soft
check "_AddDrive reached"    'ATRAP \$A04e'               soft

echo "-- timing / position:"
grep -a "^-- timing:" "$LOG" || true
grep -a "^-- executed" "$LOG" || true
grep -a "^-- ARMED on park" "$LOG" || true
grep -a "^-- driver base" "$LOG" || true
grep -a "^   DrvQHdr" "$LOG" || true

if [ "$fail" = 0 ]; then
  echo "-- MILESTONES OK"
else
  echo "-- MILESTONES FAILED: a landmark the boot used to reach is gone."
  echo "   Compare against the last known-good run before trusting any"
  echo "   instrument output from this build."
fi
exit $fail
