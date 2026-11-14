#!/usr/bin/env python3
"""Turn a `g4run --profile-host HZ FILE` dump into functions and source lines.

    tools/hostprof.py FILE BINARY [--top N] [--lines FUNCTION] [--inline]

The dump holds "module offset hits" lines. Offsets in the program itself are
resolved with addr2line against BINARY, so build BINARY with debug information
to get source lines (it does not change the generated code):

    cmake -S . -B build-sym -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-g
    cmake --build build-sym --target g4run

Without --inline a sample is charged to the function that was COMPILED there,
which after cross-unit inlining is often a big caller. With --inline it is
charged to the innermost inlined function, which is the source-level answer.
--lines FUNCTION lists the hottest source lines charged to FUNCTION (a
substring match on the name shown by the listing).
"""
import argparse
import collections
import subprocess
import sys


def resolve(binary, offsets):
    """offset -> list of (function, file:line) frames, innermost first."""
    out = {}
    if not offsets:
        return out
    p = subprocess.run(
        ["addr2line", "-f", "-C", "-i", "-a", "-e", binary] + ["0x%x" % o for o in offsets],
        capture_output=True, text=True, check=True)
    cur = None
    lines = p.stdout.splitlines()
    i = 0
    while i < len(lines):
        ln = lines[i]
        if ln.startswith("0x"):
            cur = int(ln, 16)
            out[cur] = []
            i += 1
            continue
        fn = ln
        where = lines[i + 1] if i + 1 < len(lines) else "?"
        where = where.split(" (discriminator")[0]
        out[cur].append((fn, where))
        i += 2
    return out


def short(fn):
    # Drop the argument list; keep the qualified name.
    fn = fn.replace("(anonymous namespace)::", "")
    depth = 0
    for k, ch in enumerate(fn):
        if ch == "<":
            depth += 1
        elif ch == ">":
            depth -= 1
        elif ch == "(" and depth == 0:
            return fn[:k]
    return fn


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dump")
    ap.add_argument("binary")
    ap.add_argument("--top", type=int, default=40)
    ap.add_argument("--lines", metavar="FUNCTION")
    ap.add_argument("--inline", action="store_true")
    a = ap.parse_args()

    self_hits = {}
    other = collections.Counter()
    total = 0
    for ln in open(a.dump):
        if ln.startswith("#"):
            print(ln.strip())
            continue
        mod, off, hits = ln.rsplit(None, 2)
        hits = int(hits)
        total += hits
        if mod == "self":
            self_hits[int(off, 16)] = hits
        else:
            other[mod.rsplit("/", 1)[-1]] += hits
    if not total:
        sys.exit("no samples")

    frames = resolve(a.binary, sorted(self_hits))
    by_fn = collections.Counter()
    by_line = collections.Counter()
    for off, hits in self_hits.items():
        fr = frames.get(off) or [("??", "?")]
        inner, outer = fr[0], fr[-1]
        name = short(inner[0] if a.inline else outer[0])
        by_fn[name] += hits
        if a.lines and a.lines in name:
            by_line[(short(inner[0]), inner[1].rsplit("/", 1)[-1])] += hits
    for mod, hits in other.items():
        by_fn["[" + mod + "]"] += hits

    print("%d samples; charged to the %s function" % (total, "innermost inlined" if a.inline else "compiled"))
    acc = 0
    for name, hits in by_fn.most_common(a.top):
        acc += hits
        print("%6.2f%% %6.2f%%  %s" % (100.0 * hits / total, 100.0 * acc / total, name))
    if a.lines:
        print("\nhottest lines charged to *%s*:" % a.lines)
        for (fn, where), hits in by_line.most_common(a.top):
            print("%6.2f%%  %-28s %s" % (100.0 * hits / total, where, fn))


if __name__ == "__main__":
    main()
