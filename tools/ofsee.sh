#!/bin/bash
# Ask Open Firmware's own dictionary a question and print only the answer.
#
# Every --of-see / --of-find / --of-callers / --of-refs run has to boot the
# machine to the arm point first, and that boot prints a screenful of ring
# buffers and cell watches before the section anyone wanted. This runs the
# query and shows the of-* sections alone.
#
#   tools/ofsee.sh [OPM_ATI_AT=..] -- <g4run of-flags...>
#   OPM_ARM=N   arm the name table at instruction N (default 207090000)
#   OPM_ROM=..  ROM image (default serialcons.rom)
cd "$(dirname "$0")/.." || exit 1

ARM="${OPM_ARM:-207090000}"
ROM="${OPM_ROM:-../scratch/openpowermac/serialcons.rom}"
OUT="../scratch/openpowermac/logs/ofsee.log"
mkdir -p "$(dirname "$OUT")"

OPM_ATI_AT="${OPM_ATI_AT:-1}" OPM_SERIAL="${OPM_SERIAL-}" \
  ./tools/run.sh --rom "$ROM" --max "$((ARM + 10000))" \
  --trace-of "$ARM" 0 "$@" > "$OUT" 2>&1

awk '/^-- of-(see|find|callers|refs|trace armed)/ {p=1} /^-- last exceptions/ {p=0} p' "$OUT"
