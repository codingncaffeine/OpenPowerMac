#!/usr/bin/env python3
"""Find the Mac OS kernel's area table in a RAM dump and print the entries
around a device's.

    g4run ... --dump-ram ram.bin --dump-ram-at N
    tools/pmscan.py ram.bin [HEX]     HEX = the first bytes of the entry to look for
                                      (default 40003fff9400: the Rage 128's 64 MB
                                      framebuffer at 94000000, page 4000 of its segment)

Each entry is 8 bytes: the first page within the segment, the page count
minus one, and the physical base with the mapping's flags in its low 12 bits
(03a = an I/O area, cache-inhibited; a00/a01 = unmapped/available). A dump
holds two copies: the one the boot loader built, low in memory, and the
kernel's live one near the top of RAM. An entry that differs between them
was changed at run time, which is what a cache-mode request does.
"""
import struct, sys

if len(sys.argv) < 2:
    sys.exit(__doc__)
ram = open(sys.argv[1], "rb").read()
pat = bytes.fromhex(sys.argv[2]) if len(sys.argv) > 2 else bytes.fromhex("40003fff9400")
hits, at = [], 0
while True:
    at = ram.find(pat, at)
    if at < 0:
        break
    hits.append(at)
    at += 1
print(f"{len(hits)} hit(s) for {pat.hex()}: " + " ".join(f"{h:08x}" for h in hits[:12]))
seen = set()
for h in hits:
    if any(abs(h - s) < 8 * 24 for s in seen):
        continue                      # the paired entry of a table already printed
    seen.add(h)
    print(f"--- table around {h:08x}")
    for off in range(h - 8 * 14, h + 8 * 10, 8):
        if off < 0 or off + 8 > len(ram):
            continue
        first, count, base = struct.unpack_from(">HHI", ram, off)
        print(f"  {off:08x}: page {first:04x} +{count + 1:5x} pages  phys {base & 0xFFFFF000:08x} "
              f"flags {base & 0xFFF:03x}{'   <==' if off == h else ''}")
