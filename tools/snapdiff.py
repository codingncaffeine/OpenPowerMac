#!/usr/bin/env python3
"""Which parts of two machine snapshots differ.

A fingerprint (g4run --fingerprint-every) says two machines differ; this says
WHERE. A snapshot is a header and then tagged sections (machine/include/opm/
snapshot.hpp: four tag bytes, a 64-bit length, the payload); each pair of
same-tag sections is compared, and for a section that differs the report is
how many bytes, in how many runs, and where the first ones are.

    tools/snapdiff.py A.snap B.snap [--runs N]

Memory gets one more question. A byte of RAM under a cache line the
processor holds MODIFIED is not what the processor reads there: it reads its
cache. So when the two snapshots' caches are identical, a difference in RAM
under such a line is not a difference between the two machines, and the
report separates those bytes from the ones that are.

Exit status 0 when the two are identical, 1 when they differ only in memory
the processor's caches shadow, 2 when they differ in anything else.
"""
import argparse, mmap, struct, sys

ap = argparse.ArgumentParser()
ap.add_argument("a")
ap.add_argument("b")
ap.add_argument("--runs", type=int, default=6, help="differing runs to list per section")
args = ap.parse_args()

def sections(buf):
    out, at = [], 16                      # magic, version, layout digest
    while at + 12 <= len(buf):
        tag = bytes(buf[at:at + 4]).decode("latin-1")
        (n,) = struct.unpack_from("<Q", buf, at + 4)
        if at + 12 + n > len(buf):
            sys.exit(f"snapdiff: section {tag!r} at {at} runs past the end of the file")
        out.append((tag, at + 12, n))
        at += 12 + n
    return out

def diff_runs(x, y):
    """(count of differing bytes, list of (offset, length) runs) between two equal-length buffers."""
    total, runs, CH = 0, [], 1 << 20
    for base in range(0, len(x), CH):
        cx, cy = x[base:base + CH], y[base:base + CH]
        if cx == cy:
            continue
        i, n = 0, len(cx)
        while i < n:
            if cx[i] != cy[i]:
                j = i
                while j < n and cx[j] != cy[j]:
                    j += 1
                total += j - i
                if runs and runs[-1][0] + runs[-1][1] == base + i:
                    runs[-1] = (runs[-1][0], runs[-1][1] + j - i)
                else:
                    runs.append((base + i, j - i))
                i = j
            else:
                i += 1
    return total, runs

def modified_lines(buf, secs):
    """Base addresses of the 32-byte lines a snapshot's caches hold modified."""
    lines = set()
    for tag, off, n in secs:
        if tag == "L1D ":
            at = off + 4                                  # the replacement clock
            for k in range(128 * 8):
                v, d = buf[at], buf[at + 1]
                (t,) = struct.unpack_from("<I", buf, at + 2)
                if v and d:
                    lines.add((t << 12) | ((k // 8) << 5))
                at += 42
        elif tag == "L2  ":
            (count,) = struct.unpack_from("<Q", buf, off + 8)
            at = off + 16
            for _ in range(count):
                v, d = buf[at], buf[at + 1]
                (t,) = struct.unpack_from("<I", buf, at + 2)
                if v and d:
                    lines.add(t << 5)
                at += 42
    return lines

fa, fb = open(args.a, "rb"), open(args.b, "rb")
A = mmap.mmap(fa.fileno(), 0, access=mmap.ACCESS_READ)
B = mmap.mmap(fb.fileno(), 0, access=mmap.ACCESS_READ)
if A[:16] != B[:16]:
    print("header differs (magic, version or layout digest): not the same build's snapshots")
sa, sb = sections(A), sections(B)
same = len(sa) == len(sb)
hard = not same        # a difference that is not shadowed memory
shadow = modified_lines(A, sa)
caches_same = all(A[oa:oa + na] == B[ob:ob + nb]
                  for (ta, oa, na), (tb, ob, nb) in zip(sa, sb) if ta in ("L1D ", "L2  ") and ta == tb)
for k in range(max(len(sa), len(sb))):
    if k >= len(sa) or k >= len(sb):
        t = sa[k] if k < len(sa) else sb[k]
        print(f"  {t[0]!r}: only in {'A' if k < len(sa) else 'B'}")
        same = False
        continue
    (ta, oa, na), (tb, ob, nb) = sa[k], sb[k]
    if ta != tb:
        print(f"  section {k}: {ta!r} in A, {tb!r} in B")
        same = False
        continue
    if na != nb:
        print(f"  {ta!r}: {na} bytes in A, {nb} in B")
        same = False
        continue
    xa, xb = A[oa:oa + na], B[ob:ob + nb]
    if xa == xb:
        print(f"  {ta!r}: {na:>12,} bytes  identical")
        continue
    same = False
    total, runs = diff_runs(xa, xb)
    print(f"  {ta!r}: {na:>12,} bytes  DIFFERS: {total:,} bytes in {len(runs):,} run(s)")
    if ta == "MEM " and caches_same:
        # RAM starts 8 bytes into the section, after its own length.
        (ram,) = struct.unpack_from("<Q", xa, 0)
        under, bare, bare_runs = 0, 0, []
        for off, n in runs:
            for b in range(off, off + n):
                pa = b - 8
                if 0 <= pa < ram and (pa & ~31) in shadow:
                    under += 1
                else:
                    bare += 1
                    if not bare_runs or bare_runs[-1][0] + bare_runs[-1][1] != b:
                        bare_runs.append((b, 1))
                    else:
                        bare_runs[-1] = (bare_runs[-1][0], bare_runs[-1][1] + 1)
        print(f"        {under:,} of those bytes are under a line the caches hold modified "
              f"({len(shadow):,} such lines, the same in both): the processor does not read them")
        print(f"        {bare:,} are not" + ("" if not bare else ": the machines DIFFER there"))
        if bare:
            hard = True
        runs = bare_runs
    else:
        hard = True
    for off, n in runs[:args.runs]:
        print(f"        +{off:#010x} len {n:<6} A={xa[off:off + min(n, 16)].hex()} B={xb[off:off + min(n, 16)].hex()}")
print("identical" if same else "DIFFERENT" if hard else
      "the same machine: they differ only in memory the caches shadow")
sys.exit(0 if same else 2 if hard else 1)
