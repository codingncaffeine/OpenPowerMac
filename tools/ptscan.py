#!/usr/bin/env python3
"""How does the guest map its devices? Reads a RAM dump and prints every valid
entry of the hashed page table whose physical page is at or above 0x80000000,
with the WIMG bits it carries.

    g4run ... --dump-ram ram.bin --dump-ram-at N
    tools/ptscan.py ram.bin [SDR1]          SDR1 as printed by g4run's "-- mmu:" line
                                             (default 0fc0001f, the 256 MB machine)

A device page must be cache-inhibited (wimg -I-- or -IMG). One that is not is
being reached through the data cache: its registers read stale and its stores
wait in a cache line (see g4run --cache-watch, which catches the access itself).
"""
import struct, sys

if len(sys.argv) < 2:
    sys.exit(__doc__)
ram = open(sys.argv[1], "rb").read()
sdr1 = int(sys.argv[2], 16) if len(sys.argv) > 2 else 0x0FC0001F
org, mask = sdr1 & 0xFFFF0000, sdr1 & 0x1FF
size = (mask + 1) << 16
if org + size > len(ram):
    sys.exit(f"ptscan: the hash table at {org:08x} (+{size:#x}) is outside this dump; pass the run's SDR1")

def wimg_name(w):
    return "".join(c if w & b else "-" for c, b in zip("WIMG", (8, 4, 2, 1)))

rows = {}
for off in range(org, org + size, 8):
    w0, w1 = struct.unpack_from(">II", ram, off)
    if not (w0 & 0x80000000):
        continue
    rpn = w1 >> 12
    if rpn < 0x80000:
        continue
    vsid, secondary, api = (w0 >> 7) & 0xFFFFFF, (w0 >> 6) & 1, w0 & 0x3F
    pteg = (off - org) >> 6
    hash_ = pteg if not secondary else (~pteg & ((size >> 6) - 1))
    page = (api << 10) | ((hash_ ^ vsid) & 0x3FF)       # page index within the segment
    rows[(rpn, vsid, page)] = ((w1 >> 3) & 0xF, w1 & 3, w0, w1)

cached = 0
for (rpn, vsid, page), (wimg, pp, w0, w1) in sorted(rows.items()):
    flag = "" if wimg & 4 else "   <== CACHEABLE"
    cached += not (wimg & 4)
    print(f"pa {rpn:05x}000  vsid {vsid:06x} page {page:04x}  wimg={wimg_name(wimg)} pp={pp}  "
          f"pte {w0:08x} {w1:08x}{flag}")
print(f"{len(rows)} device-page entries, {cached} cacheable")
