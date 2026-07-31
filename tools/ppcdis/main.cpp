// ppcdis — disassemble PowerPC out of a flat binary: a RAM dump, a ROM, a
// driver image.
//
// 📓 WHY THIS EXISTS. Every dig into guest code so far has ended the same way:
// an instrument prints sixteen hex words and PowerPC gets decoded by hand in
// the reply. That is slow, it is where the mistakes go, and it caps how far a
// lead can be followed — session 30 read one six-instruction stub by hand and
// stopped there, because the caller was another sixteen words away and the
// callee another after that. g4run's --dis already does this correctly, but it
// only disassembles a LIVE machine, so every follow-up question costs a fresh
// fifteen-minute boot. A RAM dump is the same bytes and answers questions for
// as long as it is on disk.
//
// The address model is deliberately dumb: a flat file, an offset, and a
// virtual address to LABEL that offset with. Nothing here walks BATs or page
// tables — the report that produced the address already resolved it, and
// guessing a second time is how a live Open Firmware address once got
// disassembled as a data table. Paste the pa the diagnostic printed.
//
//   ppcdis ram.bin --at 0xcd66e4 --va 0xffcd66f4 --count 24
//   ppcdis ram.bin --find 3800ffff,44000002        # word sequence -> offsets
//   ppcdis ram.bin --at 0xc00 --count 64           # va defaults to the offset
//
// --find takes big-endian 32-bit words because that is the unit every other
// instrument in this project prints in, so a pattern can be copied straight
// out of a diagnostic capture without re-splitting it into bytes.

#include "opm/insn.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

using namespace opm;

namespace {

std::vector<u8> readFile(const char* path)
{
    FILE* f = fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "ppcdis: cannot open %s\n", path);
        exit(2);
    }
    fseek(f, 0, SEEK_END);
    const long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    std::vector<u8> v(n > 0 ? static_cast<size_t>(n) : 0u);
    if (!v.empty() && fread(v.data(), 1, v.size(), f) != v.size()) {
        fprintf(stderr, "ppcdis: short read on %s\n", path);
        exit(2);
    }
    fclose(f);
    return v;
}

u32 be32(const u8* p) { return (u32(p[0]) << 24) | (u32(p[1]) << 16) | (u32(p[2]) << 8) | p[3]; }

u16 be16(const u8* p) { return static_cast<u16>((u32(p[0]) << 8) | p[1]); }

void usage()
{
    fprintf(stderr,
            "usage: ppcdis FILE [--at OFF] [--va VA] [--count N] [--find W,W,..]\n"
            "                   [--pef-scan [--near VA]] [--pa-base PA --va-base VA]\n"
            "  --at OFF     byte offset into FILE (for a RAM dump: the physical address)\n"
            "  --va VA      virtual address to label OFF with, so branch targets read\n"
            "               in guest terms; defaults to OFF\n"
            "  --count N    instructions to print (default 32)\n"
            "  --find W,..  find a sequence of big-endian 32-bit words; print offsets\n"
            "  --callers VA find every direct b/bl whose target is VA\n"
            "  --pef-scan   find 'Joy!peff' containers and list sections + exports\n"
            "  --near VA    with --pef-scan: print only the exports at or below VA,\n"
            "               i.e. answer \"which routine is this address in?\"\n"
            "  --pa-base/--va-base  map file offsets to guest addresses:\n"
            "               VA = vaBase + (offset - paBase). Defaults to identity.\n");
    exit(2);
}

// 📞 WHO CALLS THIS? — scan for branches whose target is a given address.
//
// The reason this exists: an instrument reports the address a machine is
// spinning at, and the next question is always "and what called it?", which a
// single sample cannot answer because lr only holds the innermost caller. A
// PowerPC branch encodes its target as a displacement from itself, so every
// direct caller of a routine is findable by arithmetic over a flat image —
// no symbols, no debugger, no re-running the machine. Cross-fragment calls go
// through glue and land here as a caller of the glue, so the chain is walked
// one hop at a time rather than in one shot.
//
// ⚠ It finds DIRECT branches only. Mac OS reaches most things through
// TVectors and `bctr`, and those are invisible here — so "no callers" means
// "nobody branches to it directly", never "nothing calls it".
void findCallers(const std::vector<u8>& buf, u64 paBase, u64 vaBase, u32 target)
{
    auto toVa = [&](u64 off) -> u32 { return static_cast<u32>(vaBase + (off - paBase)); };
    size_t hits = 0;
    char text[128];
    for (size_t off = 0; off + 4 <= buf.size(); off += 4) {
        const u32 w = be32(&buf[off]);
        const u32 op = w >> 26;
        u32 tgt = 0;
        if (op == 18) { // b / bl / ba / bla
            i32 d = static_cast<i32>(w & 0x03FFFFFCu);
            if (d & 0x02000000)
                d |= static_cast<i32>(0xFC000000u);
            tgt = (w & 2u) ? static_cast<u32>(d) : toVa(off) + static_cast<u32>(d);
        } else if (op == 16) { // bc / bcl
            i32 d = static_cast<i32>(w & 0x0000FFFCu);
            if (d & 0x8000)
                d |= static_cast<i32>(0xFFFF0000u);
            tgt = (w & 2u) ? static_cast<u32>(d) : toVa(off) + static_cast<u32>(d);
        } else
            continue;
        if (tgt != target)
            continue;
        disassemble(w, toVa(off), text, sizeof text, Style::Gnu);
        printf("%08x: %08x  %s\n", toVa(off), w, text);
        ++hits;
    }
    printf("-- %zu direct branch(es) to %08x (indirect calls through TVectors "
           "are NOT findable this way)\n",
           hits, target);
}

// ---- PEF ------------------------------------------------------------------
//
// 📓 WHY A PEF PARSER LIVES IN A DISASSEMBLER. Every address this project has
// chased through Mac OS's own code has been a bare number: `ffcd66ec` was
// followed for a whole session as "a NanoKernel stub near the display driver's
// driver-ref", which turned out to be a coincidence and cost the lead. Mac OS
// ROM code is PEF fragments mapped in place, and a PEF carries its exported
// symbol NAMES right there in the image -- so an address can be turned into a
// name from the same bytes the machine is executing, without a symbol file,
// without the internet, and without guessing from what an address is near.
//
// Layout per Apple's "Mac OS Runtime Architectures", ch. 'PEF Structure'. All
// fields big-endian.
//
// ⚠ SECTIONS ARE VALIDATED; EXPORT NAMES ARE NOT, AND THIS CODE REFUSES RATHER
// THAN GUESSES. Measured on the ATI NDRV (romndrv.pef, 6 exports): the export
// KEY table decodes perfectly at container offset 0x6da -- six (nameLength,
// hash) pairs matching debugSet/debugGet/TheDriverDescription/DoDriverIO/
// TheDriverPowerCapabilities/debugIO -- and the hash table sits 16 bytes below
// it at 0x6ca with chain counts 3+0+2+1 = 6, exactly the declared export
// count. But the loader header's exportHashOffset (0x628 -> 0x6a8) points 0x22
// bytes BELOW that, into the string table, and the strings themselves start
// 0xa bytes past loaderStringsOffset. Two independent offsets are short by
// different constants, so something about how these offsets are anchored is
// not understood yet -- and the symbol table then does not fit in the bytes
// that remain. Until that is resolved the guards below drop every candidate
// and the tool reports "0 export(s)", which is a false negative and safe. It
// must not become a false name: an address labelled with the wrong routine is
// worse than an address labelled with nothing.
constexpr u32 kPefTag1 = 0x4A6F7921u; // 'Joy!'
constexpr u32 kPefTag2 = 0x70656666u; // 'peff'

const char* sectionKindName(u8 k)
{
    switch (k) {
    case 0: return "code";
    case 1: return "data";
    case 2: return "pidata";
    case 3: return "const";
    case 4: return "loader";
    case 5: return "debug";
    case 6: return "exec-data";
    case 7: return "exception";
    case 8: return "traceback";
    default: return "?";
    }
}

// One export, resolved to the address it has in the running machine.
struct Export {
    u32 va;
    u32 container;
    std::string name; // by value: a vector of strings reallocates, and a
                      // c_str() taken before that is a dangling read
};

void scanPef(const std::vector<u8>& buf, u64 paBase, u64 vaBase, bool haveNear, u32 nearVa)
{
    auto toVa = [&](u64 off) -> u32 { return static_cast<u32>(vaBase + (off - paBase)); };
    std::vector<Export> exports;
    size_t containers = 0;

    for (size_t off = 0; off + 40 <= buf.size(); off += 4) {
        if (be32(&buf[off]) != kPefTag1 || be32(&buf[off + 4]) != kPefTag2)
            continue;
        ++containers;
        const u32 sectionCount = be16(&buf[off + 32]);
        if (!haveNear)
            printf("== PEF @%08llx (va %08x) arch=%08x sections=%u\n",
                   static_cast<unsigned long long>(off), toVa(off),
                   be32(&buf[off + 8]), sectionCount);

        u64 loaderOff = 0;
        // Section i's bytes live at containerOffset from the container start.
        // A ROM fragment is used IN PLACE, so that is also where the machine
        // executes it -- which is what makes an address nameable at all.
        std::vector<std::pair<u32, u32>> secVa(sectionCount, {0, 0});
        for (u32 s = 0; s < sectionCount; ++s) {
            const size_t sh = off + 40 + size_t(s) * 28;
            if (sh + 28 > buf.size())
                break;
            const u32 totalSize = be32(&buf[sh + 8]);
            const u32 containerLength = be32(&buf[sh + 16]);
            const u32 containerOffset = be32(&buf[sh + 20]);
            const u8 kind = buf[sh + 24];
            const u32 va = toVa(off + containerOffset);
            secVa[s] = {va, totalSize};
            if (!haveNear)
                printf("--   sec %u %-9s va %08x..%08x  len %u\n", s,
                       sectionKindName(kind), va, va + totalSize,
                       containerLength);
            if (kind == 4) {
                loaderOff = off + containerOffset;
            }
        }
        if (!loaderOff || loaderOff + 56 > buf.size())
            continue;
        const u8* ld = &buf[static_cast<size_t>(loaderOff)];
        const u32 stringsOff = be32(ld + 40);
        const u32 hashOff = be32(ld + 44);
        const u32 hashPower = be32(ld + 48);
        const u32 expCount = be32(ld + 52);
        if (hashPower > 24 || expCount > 200000u)
            continue; // not a loader section we understand; do not invent one
        // hash table (2^power u32) then expCount key words, then the symbols.
        const u64 symOff = loaderOff + hashOff + (u64(1) << hashPower) * 4 + u64(expCount) * 4;
        for (u32 e = 0; e < expCount; ++e) {
            const u64 so = symOff + u64(e) * 10;
            if (so + 10 > buf.size())
                break;
            const u32 classAndName = be32(&buf[static_cast<size_t>(so)]);
            const u32 value = be32(&buf[static_cast<size_t>(so) + 4]);
            const auto sec = static_cast<std::int16_t>(be16(&buf[static_cast<size_t>(so) + 8]));
            const u64 nameOff = loaderOff + stringsOff + (classAndName & 0x00FFFFFFu);
            if (nameOff + 1 >= buf.size() || sec < 0 || u32(sec) >= sectionCount)
                continue;
            // The loader string table is length-prefixed nowhere: names run to
            // the next entry. Read a bounded, printable run.
            std::string nm;
            for (u64 p = nameOff; p < buf.size() && nm.size() < 64; ++p) {
                const u8 c = buf[static_cast<size_t>(p)];
                if (c < 0x20 || c > 0x7E)
                    break;
                nm.push_back(static_cast<char>(c));
            }
            if (nm.empty())
                continue;
            exports.push_back({secVa[sec].first + value, toVa(off), nm});
        }
    }

    if (haveNear) {
        // "Which routine is this?" -- the nearest exports AT OR BELOW the
        // address. Printing the nearest in both directions reads as if the
        // answer might be the one above, and it never is: a symbol above the
        // address belongs to the NEXT routine.
        std::vector<const Export*> below;
        for (const Export& e : exports)
            if (e.va <= nearVa)
                below.push_back(&e);
        std::sort(below.begin(), below.end(),
                  [](const Export* a, const Export* b) { return a->va < b->va; });
        printf("-- %zu container(s), %zu export(s); nearest at or below %08x:\n",
               containers, exports.size(), nearVa);
        const size_t show = below.size() < 8 ? below.size() : 8;
        for (size_t i = below.size() - show; i < below.size(); ++i)
            printf("   %08x  +%-8u %s\n", below[i]->va, nearVa - below[i]->va,
                   below[i]->name.c_str());
        if (below.empty())
            printf("   (none -- no exported symbol precedes it in any container)\n");
    } else {
        for (const Export& e : exports)
            printf("   %08x  %s\n", e.va, e.name.c_str());
        printf("-- %zu container(s), %zu export(s)\n", containers, exports.size());
    }
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 2)
        usage();
    const char* path = argv[1];
    u64 at = 0;
    u64 va = 0;
    bool vaSet = false;
    u32 count = 32;
    const char* find = nullptr;
    bool pefScan = false, haveNear = false, haveCallers = false;
    u32 callers = 0;
    u32 nearVa = 0;
    u64 paBase = 0, vaBase = 0;

    for (int i = 2; i < argc; ++i) {
        auto next = [&]() -> const char* {
            if (i + 1 >= argc)
                usage();
            return argv[++i];
        };
        if (!strcmp(argv[i], "--at"))
            at = strtoull(next(), nullptr, 0);
        else if (!strcmp(argv[i], "--va")) {
            va = strtoull(next(), nullptr, 0);
            vaSet = true;
        } else if (!strcmp(argv[i], "--count"))
            count = static_cast<u32>(strtoul(next(), nullptr, 0));
        else if (!strcmp(argv[i], "--find"))
            find = next();
        else if (!strcmp(argv[i], "--pef-scan"))
            pefScan = true;
        else if (!strcmp(argv[i], "--callers")) {
            callers = static_cast<u32>(strtoul(next(), nullptr, 0));
            haveCallers = true;
        }
        else if (!strcmp(argv[i], "--near")) {
            nearVa = static_cast<u32>(strtoul(next(), nullptr, 0));
            haveNear = true;
        } else if (!strcmp(argv[i], "--pa-base"))
            paBase = strtoull(next(), nullptr, 0);
        else if (!strcmp(argv[i], "--va-base"))
            vaBase = strtoull(next(), nullptr, 0);
        else
            usage();
    }

    const std::vector<u8> buf = readFile(path);

    if (haveCallers) {
        findCallers(buf, paBase, vaBase, callers);
        return 0;
    }
    if (pefScan) {
        scanPef(buf, paBase, vaBase, haveNear, nearVa);
        return 0;
    }

    if (find) {
        std::vector<u32> pat;
        for (const char* p = find; *p;) {
            pat.push_back(static_cast<u32>(strtoul(p, nullptr, 16)));
            const char* c = strchr(p, ',');
            if (!c)
                break;
            p = c + 1;
        }
        if (pat.empty())
            usage();
        // Word-aligned only: every producer of these patterns is aligned code,
        // and an unaligned "hit" in a 64 MB dump is noise by construction.
        size_t hits = 0;
        for (size_t off = 0; off + pat.size() * 4 <= buf.size(); off += 4) {
            bool ok = true;
            for (size_t k = 0; k < pat.size(); ++k)
                if (be32(&buf[off + k * 4]) != pat[k]) {
                    ok = false;
                    break;
                }
            if (!ok)
                continue;
            printf("%08llx\n", static_cast<unsigned long long>(off));
            ++hits;
        }
        printf("-- %zu hit(s) of %zu word(s) in %s\n", hits, pat.size(), path);
        return hits ? 0 : 1;
    }

    if (!vaSet)
        va = at;
    if (at >= buf.size()) {
        fprintf(stderr, "ppcdis: offset %llx past end of %s (%zu bytes)\n",
                static_cast<unsigned long long>(at), path, buf.size());
        return 2;
    }

    char text[128];
    for (u32 n = 0; n < count; ++n) {
        const u64 off = at + u64(n) * 4;
        if (off + 4 > buf.size())
            break;
        const u32 w = be32(&buf[static_cast<size_t>(off)]);
        const u32 pc = static_cast<u32>(va + u64(n) * 4);
        disassemble(w, pc, text, sizeof text, Style::Gnu);
        printf("%08x: %08x  %s\n", pc, w, text);
    }
    return 0;
}
