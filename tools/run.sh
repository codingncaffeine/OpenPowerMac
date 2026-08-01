#!/bin/bash
# The flagship boot run, parameterized. Extra flags are appended verbatim.
#
# Every dig this session begins by retyping a 12-line command with a 700-char
# Forth script in it, and each accidental edit to that script shifts the whole
# timeline by ~100M instructions and silently invalidates comparisons against
# earlier runs. Keeping it in one file makes the baseline a fixed point.
#
# Overrides: OPM_ATI_AT (when the card's FCode ROM appears — before Open
# Firmware's PCI probe it runs and OF switches its console to the screen;
# after, it never runs at all), OPM_SERIAL (the script typed at OF),
# OPM_HD, and OPM_BIN (the executable — point it at g4prof to profile).
# The disk image is WRITABLE, so two runs sharing it corrupt each
# other: point a short query run at a copy when a long boot is in flight.
#
# ⚠ OPM_ATI_AT DEFAULTED TO 236000000 UNTIL 2026-07-30 AND THAT PRODUCED A
# BLIND MACHINE. The reveal has to land before Open Firmware's PCI probe, the
# boot timeline moved past that count, and every run through this script came
# back with `ati bars: reg=00000000` and zero framebuffer writes — healthy in
# every other respect, so it read as a finding rather than as a harness fault.
# The app has always used 1 (shell.json's AtiAt); so does this now.
cd "$(dirname "$0")/.." || exit 1

DEFAULT_SERIAL='" /pci@f0000000" select-dev;10 8000 probe-pci-device;8000 10 probe-pci-device;unselect-dev;dev /pci@f0000000/pci1002,5046@10;" ATY,Rage128Pd" device-name;" display" device-type;" ATY,Rage128Pd" encode-string " compatible" property;" /pci@f2000000" select-dev;3000000 to pci-probe-request;unselect-dev;probe-pci;dev /pci@f2000000/pci106b,19@18;" usb" device-name;" usb" device-type;dev /pci@f2000000/pci106b,19@19;" usb" device-name;" usb" device-type;mac-boot'

# ⚠ THE CD MUST EXIST OR THIS IS A DIFFERENT MACHINE. Open Firmware boots
# the CD when one is present (hanggate.sh's note), and hd.img alone is NOT
# bootable — so when the ISO went missing from ~/Downloads the "baseline"
# silently became a question-mark boot whose paint/Ticks numbers still
# matched, and a session was spent bisecting a machine change that had
# nothing to do with it. g4run only prints "cd attach FAILED" and carries
# on; this stops the run instead.
# 2026-08-01: the user consolidated the loose Downloads media into
# "Downloads/Open Power Mac/" — PowerMacG4.iso, the 9.2.2 universal
# install, the restore DVD and the 8.6 material all live there now.
OPM_CD="${OPM_CD:-/c/Users/gamer/Downloads/Open Power Mac/PowerMacG4.iso}"
[ -f "$OPM_CD" ] || { echo "FAIL: boot CD missing: $OPM_CD (set OPM_CD)"; exit 1; }

# ⚠ --fast-tb WAS 60 UNTIL 2026-07-30 AND IS NOW 4. With the KeyLargo timer
# answered the guest's clock is correct, and 60 leaves Mac OS only ~27,300
# emulated instructions to do ~32,000 instructions of 60 Hz work in: the
# machine services ticks back to back and the boot dies at 2.5 G with a blank
# screen. 4 gives it 333,000 and lands the timebase at 1.06x real. See the
# note on SawtoothBus::klTimerOn.
exec "${OPM_BIN:-./build/tools/g4run/Release/g4run.exe}" \
 --rom "../scratch/openpowermac/roms/newworld/sawtooth_4.2.8f1_stock.rom" \
 --exc 0 --fast-tb "${OPM_FAST_TB:-4}" \
 --cd "$OPM_CD" \
 --hd "${OPM_HD:-../scratch/openpowermac/hd.img}" \
 --ati-rom "../scratch/openpowermac/ati/ati_oem_rage128pro_136_agp_full.rom" \
 --ati-at "${OPM_ATI_AT:-1}" \
 --serial-input "${OPM_SERIAL-$DEFAULT_SERIAL}" \
 "$@"
