#!/usr/bin/env python3
"""Turn the guest half of a `g4run --profile-host HZ FILE` run into functions.

    tools/guestprof.py FILE.guest [--top N] [--dis N] [--ppcdis BIN]

FILE.guest holds "pc hits" lines: where the guest's pc was at each host
sample, so the shares are of HOST time. FILE.guestmem, written beside it by
the same run, holds the guest's code at those pcs as "virtual page, 4096
bytes" records, read through the guest's own address translation (an
address is not a place in memory: Mac OS maps its system heap elsewhere, so
a dump of RAM cannot be read at a pc). A pc whose page is not in it is
grouped by page and marked.

A function is taken to run from the word after the previous `blr` (and its
traceback table, if one follows) to its own last `blr`; a name is printed
when the code carries a traceback table with one. --dis N prints the first
N instructions of each listed function with tools/ppcdis, each with its
share of the samples.
"""
import argparse
import collections
import os
import struct
import subprocess
import tempfile

ap = argparse.ArgumentParser()
ap.add_argument("guest")
ap.add_argument("--top", type=int, default=25)
ap.add_argument("--dis", type=int, default=0)
ap.add_argument("--ppcdis", default=os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "build", "tools", "ppcdis", "ppcdis"))
a = ap.parse_args()

hits = {}
for ln in open(a.guest):
    if ln.startswith("#"):
        continue
    pc, n = ln.split()
    hits[int(pc, 16)] = int(n)
total = sum(hits.values())

pages = {}
with open(a.guest + "mem", "rb") as f:
    while True:
        h = f.read(4)
        if len(h) < 4:
            break
        pages[struct.unpack(">I", h)[0] >> 12] = f.read(4096)
BLR = b"\x4e\x80\x00\x20"
SPAN = 0x4000  # how far a function is looked for on either side of a pc


def mem(at, n):
    """Up to n bytes at virtual `at`, cut short at the first page not held."""
    out = b""
    while n > 0:
        pg = pages.get(at >> 12)
        if pg is None:
            break
        take = min(n, 4096 - (at & 0xFFF))
        out += pg[at & 0xFFF:(at & 0xFFF) + take]
        at += take
        n -= take
    return out


def back(at, n):
    """Up to n bytes ending at virtual `at`, cut at the first page not held."""
    lo = at
    while lo > at - n and ((lo - 1) >> 12) in pages:
        lo = max(at - n, ((lo - 1) >> 12) << 12)
    return lo, mem(lo, at - lo)


def table(at):
    """A traceback table at `at` (after its zero word): (length, name)."""
    t = mem(at, 8)
    if len(t) < 8:
        return 0, ""
    f1, f2 = t[2], t[3]
    p = at + 8
    if t[6] + (t[7] >> 1):
        p += 4
    if f1 & 0x20:
        p += 4
    if f2 & 0x80:
        p += 4
    if f1 & 0x08:
        c = mem(p, 4)
        n = struct.unpack(">I", c)[0] if len(c) == 4 else 0
        p += 4 + 4 * min(n, 16)
    name = ""
    if f2 & 0x40:
        c = mem(p, 2)
        nl = struct.unpack(">H", c)[0] if len(c) == 2 else 0
        if 0 < nl < 128:
            nm = mem(p + 2, nl)
            if len(nm) == nl and all(32 <= ch < 127 for ch in nm):
                name = nm.decode("ascii")
        p += 2 + min(nl, 255)
    if f2 & 0x20:
        p += 1
    return ((p + 3) & ~3) - at, name


def bounds(pc):
    lo = pc & ~3
    floor, b = back(lo, SPAN)
    start = floor
    for k in range(len(b) - 4, -1, -4):
        if b[k:k + 4] == BLR:
            start = floor + k + 4
            break
    if mem(start, 4) == b"\0\0\0\0":
        n, _ = table(start + 4)
        if 0 < n < 512 and start + 4 + n <= lo:
            start += 4 + n
    fwd = mem(lo, SPAN)
    end = lo + 4
    for k in range(0, len(fwd) - 3, 4):
        if fwd[k:k + 4] == BLR:
            end = lo + k + 4
            break
    return start, end


funcs = collections.Counter()
info = {}
recent = []
for pc, n in sorted(hits.items()):
    if (pc >> 12) not in pages:
        funcs[("page", pc & ~0xFFF)] += n
        continue
    hit = next((b for b in recent[-4:] if b[0] <= pc <= b[1]), None)
    if hit is None:
        hit = bounds(pc)
        recent.append(hit)
    funcs[("fn",) + hit] += n
    info.setdefault(hit, collections.Counter())[pc] += n

print(f"{total} samples, {len(hits)} distinct pcs, {len(funcs)} functions or pages")
run = 0
for key, n in funcs.most_common(a.top):
    run += n
    if key[0] == "page":
        print(f"{100 * n / total:6.2f}% {100 * run / total:6.2f}%  page {key[1]:08x} (not in the code dump)")
        continue
    start, end = key[1], key[2]
    hot = info[(start, end)].most_common(1)[0]
    nm = table(end + 4)[1] if mem(end, 4) == b"\0\0\0\0" else ""
    print(f"{100 * n / total:6.2f}% {100 * run / total:6.2f}%  {start:08x}-{end:08x} "
          f"({(end - start) // 4:5d} insns)  hottest pc {hot[0]:08x} ({100 * hot[1] / total:.2f}%)  {nm}")
    if a.dis:
        cnt = min(a.dis, (end - start) // 4)
        with tempfile.NamedTemporaryFile(suffix=".bin") as tf:
            tf.write(mem(start, cnt * 4))
            tf.flush()
            out = subprocess.run([a.ppcdis, tf.name, "--at", "0", "--va", hex(start),
                                  "--count", str(cnt)], capture_output=True, text=True).stdout
        for ln in out.splitlines():
            try:
                at = int(ln.split(":")[0], 16)
            except ValueError:
                continue
            h = info[(start, end)].get(at + 4, 0)  # pc is one ahead while a handler runs
            print(f"        {100 * h / total:5.2f}  {ln}" if h else f"               {ln}")
