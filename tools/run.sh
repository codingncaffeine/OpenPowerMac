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
# after, it never runs at all) and OPM_SERIAL (the script typed at OF).
cd "$(dirname "$0")/.." || exit 1

DEFAULT_SERIAL='" /pci@f0000000" select-dev;10 8000 probe-pci-device;8000 10 probe-pci-device;unselect-dev;dev /pci@f0000000/pci1002,5046@10;" ATY,Rage128Pd" device-name;" display" device-type;" ATY,Rage128Pd" encode-string " compatible" property;" /pci@f2000000" select-dev;3000000 to pci-probe-request;unselect-dev;probe-pci;dev /pci@f2000000/pci106b,19@18;" usb" device-name;" usb" device-type;dev /pci@f2000000/pci106b,19@19;" usb" device-name;" usb" device-type;mac-boot'

exec ./build/tools/g4run/Release/g4run.exe \
 --rom "../scratch/openpowermac/roms/newworld/sawtooth_4.2.8f1_stock.rom" \
 --exc 0 --fast-tb 60 \
 --cd "/c/Users/gamer/Downloads/PowerMacG4.iso" \
 --hd "../scratch/openpowermac/hd.img" \
 --ati-rom "../scratch/openpowermac/ati/ati_oem_rage128pro_136_agp_full.rom" \
 --ati-at "${OPM_ATI_AT:-236000000}" \
 --serial-input "${OPM_SERIAL:-$DEFAULT_SERIAL}" \
 "$@"
