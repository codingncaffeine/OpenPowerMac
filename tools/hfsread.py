#!/usr/bin/env python3
"""Read files out of a classic HFS volume image, without mounting it.

    tools/hfsread.py IMAGE ls [PATH]            list a folder (default: the root)
    tools/hfsread.py IMAGE find TEXT            every catalog entry whose name contains TEXT
    tools/hfsread.py IMAGE cat PATH OUT         write PATH's data fork to OUT
    tools/hfsread.py IMAGE rsrc PATH OUT        write PATH's resource fork to OUT

PATH is colon-separated from the volume's root, as the Mac writes it:
"Applications:Graphics:Sample Picture". IMAGE may be a bare HFS volume, or a disk image with
an Apple partition map, in which case the first Apple_HFS partition is read.
The image is opened read-only.

What this is for: ground truth. A picture the guest draws wrongly can be
compared against the picture as it is stored on the disc, and that needs the
file out of the image. Nothing else on this host reads classic HFS.

Format per Inside Macintosh: Files, chapter 2 (the master directory block,
the catalog and extents-overflow B*-trees). HFS only, not HFS Plus.
"""
import struct
import sys


class Hfs:
    def __init__(self, path):
        self.f = open(path, "rb")
        self.base = self._find_volume()
        mdb = self._at(self.base + 1024, 162)
        if mdb[0:2] != b"BD":
            sys.exit("hfsread: no HFS volume found (HFS Plus and other formats are not read)")
        self.al_blk_size = struct.unpack_from(">I", mdb, 20)[0]
        self.al_blk_start = struct.unpack_from(">H", mdb, 28)[0]
        self.name = mdb[37:37 + mdb[36]].decode("mac_roman")
        xt_size = struct.unpack_from(">I", mdb, 130)[0]
        xt_ext = self._extents(mdb, 134)
        ct_size = struct.unpack_from(">I", mdb, 146)[0]
        ct_ext = self._extents(mdb, 150)
        # The extents-overflow file never overflows itself (IM: Files 2-75).
        self.xt = self._read_extents(xt_ext, xt_size)
        self.overflow = self._load_overflow()
        self.ct = self._read_fork(4, 0x00, ct_ext, ct_size)
        self.entries = self._load_catalog()

    # -- raw access ---------------------------------------------------------
    def _at(self, off, n):
        self.f.seek(off)
        return self.f.read(n)

    def _find_volume(self):
        if self._at(1024, 2) == b"BD":
            return 0
        if self._at(0, 2) == b"ER":
            n = 1
            k = 1
            while k <= n:
                e = self._at(512 * k, 512)
                if e[0:2] != b"PM":
                    break
                n = struct.unpack_from(">I", e, 4)[0]
                start = struct.unpack_from(">I", e, 8)[0]
                kind = e[48:80].split(b"\0")[0]
                if kind == b"Apple_HFS":
                    return start * 512
                k += 1
        sys.exit("hfsread: neither a bare HFS volume nor a partition map with an Apple_HFS partition")

    @staticmethod
    def _extents(buf, off):
        return [struct.unpack_from(">HH", buf, off + 4 * k) for k in range(3)]

    def _blocks(self, start, count):
        return self._at(self.base + self.al_blk_start * 512 + start * self.al_blk_size,
                        count * self.al_blk_size)

    def _read_extents(self, exts, size):
        out = b"".join(self._blocks(s, c) for s, c in exts if c)
        return out[:size]

    def _read_fork(self, cnid, fork, exts, size):
        """A fork's bytes: its first three extents, then the overflow file's."""
        out = bytearray()
        done = 0
        for s, c in exts:
            if c:
                out += self._blocks(s, c)
                done += c
        while len(out) < size:
            more = self.overflow.get((fork, cnid, done))
            if not more:
                break
            for s, c in more:
                if c:
                    out += self._blocks(s, c)
                    done += c
        return bytes(out[:size])

    # -- B*-trees -----------------------------------------------------------
    @staticmethod
    def _leaf_records(tree):
        if len(tree) < 512:
            return
        node = struct.unpack_from(">I", tree, 14 + 10)[0]  # bthFNode: first leaf
        seen = set()
        while node and node not in seen:
            seen.add(node)
            n = tree[node * 512:(node + 1) * 512]
            if len(n) < 512:
                break
            flink = struct.unpack_from(">I", n, 0)[0]
            nrecs = struct.unpack_from(">H", n, 10)[0]
            for r in range(nrecs):
                a = struct.unpack_from(">H", n, 510 - 2 * r)[0]
                b = struct.unpack_from(">H", n, 508 - 2 * r)[0]
                yield n[a:b]
            node = flink

    def _load_overflow(self):
        out = {}
        for rec in self._leaf_records(self.xt):
            if len(rec) < 20 or rec[0] != 7:
                continue
            fork, cnid, fabn = struct.unpack_from(">BIH", rec, 1)
            out[(fork, cnid, fabn)] = self._extents(rec, 8)
        return out

    def _load_catalog(self):
        """parent directory id -> {name: record}"""
        dirs = {}
        for rec in self._leaf_records(self.ct):
            klen = rec[0]
            if klen < 6:
                continue
            parent = struct.unpack_from(">I", rec, 2)[0]
            name = rec[7:7 + rec[6]].decode("mac_roman")
            d = 1 + klen
            d += d & 1  # the data is word-aligned after the key
            data = rec[d:]
            if not data:
                continue
            kind = data[0]
            if kind == 1:  # directory
                dirs.setdefault(parent, {})[name] = ("dir", struct.unpack_from(">I", data, 6)[0])
            elif kind == 2:  # file
                dirs.setdefault(parent, {})[name] = (
                    "file",
                    {
                        "type": data[4:8], "creator": data[8:12],
                        "cnid": struct.unpack_from(">I", data, 20)[0],
                        "dlen": struct.unpack_from(">I", data, 26)[0],
                        "rlen": struct.unpack_from(">I", data, 36)[0],
                        "dext": self._extents(data, 74),
                        "rext": self._extents(data, 86),
                    })
        return dirs

    # -- by path ------------------------------------------------------------
    def lookup(self, path):
        cur = ("dir", 2)  # the root directory's id
        for part in [p for p in path.split(":") if p]:
            if cur[0] != "dir":
                sys.exit(f"hfsread: {part!r} is inside something that is not a folder")
            here = self.entries.get(cur[1], {})
            hit = [n for n in here if n.lower() == part.lower()]
            if not hit:
                sys.exit(f"hfsread: no {part!r} here; there is: " + ", ".join(sorted(here)) [:600])
            cur = here[hit[0]]
        return cur

    def fork(self, rec, resource):
        if resource:
            return self._read_fork(rec["cnid"], 0xFF, rec["rext"], rec["rlen"])
        return self._read_fork(rec["cnid"], 0x00, rec["dext"], rec["dlen"])


def show(name, e):
    if e[0] == "dir":
        print(f"  {'<folder>':>24}  {name}")
    else:
        r = e[1]
        tc = (r["type"] + b"/" + r["creator"]).decode("mac_roman", "replace")
        print(f"  {r['dlen']:>10} {r['rlen']:>9}r  {tc}  {name}")


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    vol = Hfs(sys.argv[1])
    cmd = sys.argv[2]
    if cmd == "ls":
        e = vol.lookup(sys.argv[3] if len(sys.argv) > 3 else "")
        if e[0] != "dir":
            sys.exit("hfsread: not a folder")
        print(f"volume {vol.name!r}; sizes are data fork, resource fork")
        for name, sub in sorted(vol.entries.get(e[1], {}).items(), key=lambda kv: kv[0].lower()):
            show(name, sub)
    elif cmd == "find" and len(sys.argv) > 3:
        want = sys.argv[3].lower()
        for parent, here in vol.entries.items():
            for name, sub in here.items():
                if want in name.lower():
                    print(f"in folder id {parent}:", end="")
                    show(name, sub)
    elif cmd in ("cat", "rsrc") and len(sys.argv) > 4:
        e = vol.lookup(sys.argv[3])
        if e[0] != "file":
            sys.exit("hfsread: not a file")
        data = vol.fork(e[1], cmd == "rsrc")
        open(sys.argv[4], "wb").write(data)
        print(f"{len(data)} bytes -> {sys.argv[4]}")
    else:
        sys.exit(__doc__)


if __name__ == "__main__":
    main()
