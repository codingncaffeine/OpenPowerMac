// A classic HFS volume, written from scratch — the host side of the shared
// folder. Format per Inside Macintosh: Files ("Data Organization on Volumes"):
// boot blocks, the Master Directory Block, the volume bitmap, and two
// B*-trees (catalog, extents overflow) over contiguous fork extents. This
// builder writes read-only volumes built fresh from a folder, which removes
// every hard part of HFS as a LIVE filesystem: nothing is ever inserted into
// a B-tree — the trees are packed sorted, bottom-up, in one pass; every fork
// is one contiguous extent, so the extents overflow tree is always empty;
// and the free-space story is "whatever is left after the last fork".
//
// ⚠ Name ordering is the one place a from-scratch HFS writer can be subtly
// wrong: the guest looks records up with ITS comparator (RelString), so the
// builder's sort must agree with it or a record becomes unfindable while
// plainly present. ASCII names fold case-insensitively exactly as RelString
// does; bytes past 0x7F are ordered by raw value here, which diverges from
// the full MacRoman relative table only when two names in one folder
// straddle one of its quirks — kept, with the divergence stated rather than
// hidden. Names are transcoded to MacRoman with a small table for the Latin
// range (™ © é …); anything unmappable becomes '_'.

#include "opm/hfs.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <vector>

namespace opm {
namespace {

namespace fs = std::filesystem;
using u8 = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;

// C++20 moved the u8 path conversions to char8_t; these keep the API's
// plain-UTF-8 std::string contract without tripping the deprecation of
// fs::u8path.
fs::path pathFromUtf8(const std::string& s)
{
    return fs::path(std::u8string(s.begin(), s.end()));
}
std::string utf8Of(const fs::path& p)
{
    const std::u8string u = p.u8string();
    return std::string(u.begin(), u.end());
}

void put16(std::vector<u8>& b, size_t at, u16 v)
{
    b[at] = static_cast<u8>(v >> 8);
    b[at + 1] = static_cast<u8>(v);
}
void put32(std::vector<u8>& b, size_t at, u32 v)
{
    b[at] = static_cast<u8>(v >> 24);
    b[at + 1] = static_cast<u8>(v >> 16);
    b[at + 2] = static_cast<u8>(v >> 8);
    b[at + 3] = static_cast<u8>(v);
}
u16 get16(const u8* p) { return static_cast<u16>((p[0] << 8) | p[1]); }
u32 get32(const u8* p)
{
    return (static_cast<u32>(p[0]) << 24) | (static_cast<u32>(p[1]) << 16) |
           (static_cast<u32>(p[2]) << 8) | p[3];
}

// Host time → HFS time (seconds since 1904-01-01). HFS dates are nominally
// local time; UTC is close enough for a transfer volume.
u32 hfsNow()
{
    const u64 unixNow = static_cast<u64>(time(nullptr));
    return static_cast<u32>(unixNow + 2082844800ull);
}

// UTF-8 → MacRoman, best effort: ASCII verbatim, a table for the Latin-1 +
// punctuation range Mac names actually use, '_' for the rest. ':' is the
// HFS path separator and never legal in a name.
u8 macRomanOf(u32 cp)
{
    if (cp < 0x80)
        return cp == ':' ? '_' : static_cast<u8>(cp);
    switch (cp) {
    case 0x00C4: return 0x80; // Ä
    case 0x00C5: return 0x81; // Å
    case 0x00C7: return 0x82; // Ç
    case 0x00C9: return 0x83; // É
    case 0x00D1: return 0x84; // Ñ
    case 0x00D6: return 0x85; // Ö
    case 0x00DC: return 0x86; // Ü
    case 0x00E1: return 0x87; // á
    case 0x00E0: return 0x88; // à
    case 0x00E2: return 0x89; // â
    case 0x00E4: return 0x8A; // ä
    case 0x00E3: return 0x8B; // ã
    case 0x00E5: return 0x8C; // å
    case 0x00E7: return 0x8D; // ç
    case 0x00E9: return 0x8E; // é
    case 0x00E8: return 0x8F; // è
    case 0x00EA: return 0x90; // ê
    case 0x00EB: return 0x91; // ë
    case 0x00ED: return 0x92; // í
    case 0x00EC: return 0x93; // ì
    case 0x00EE: return 0x94; // î
    case 0x00EF: return 0x95; // ï
    case 0x00F1: return 0x96; // ñ
    case 0x00F3: return 0x97; // ó
    case 0x00F2: return 0x98; // ò
    case 0x00F4: return 0x99; // ô
    case 0x00F6: return 0x9A; // ö
    case 0x00F5: return 0x9B; // õ
    case 0x00FA: return 0x9C; // ú
    case 0x00F9: return 0x9D; // ù
    case 0x00FB: return 0x9E; // û
    case 0x00FC: return 0x9F; // ü
    case 0x2020: return 0xA0; // †
    case 0x00B0: return 0xA1; // °
    case 0x00A2: return 0xA2; // ¢
    case 0x00A3: return 0xA3; // £
    case 0x00A7: return 0xA4; // §
    case 0x2022: return 0xA5; // •
    case 0x00B6: return 0xA6; // ¶
    case 0x00DF: return 0xA7; // ß
    case 0x00AE: return 0xA8; // ®
    case 0x00A9: return 0xA9; // ©
    case 0x2122: return 0xAA; // ™
    case 0x00B4: return 0xAB; // ´
    case 0x00A8: return 0xAC; // ¨
    case 0x00C6: return 0xAE; // Æ
    case 0x00D8: return 0xAF; // Ø
    case 0x2013: return 0xD0; // –
    case 0x2014: return 0xD1; // —
    case 0x201C: return 0xD2; // “
    case 0x201D: return 0xD3; // ”
    case 0x2018: return 0xD4; // ‘
    case 0x2019: return 0xD5; // ’
    default: return '_';
    }
}

std::string toMacRoman(const std::u32string& in)
{
    std::string out;
    for (char32_t c : in)
        out.push_back(static_cast<char>(macRomanOf(static_cast<u32>(c))));
    if (out.size() > 31)
        out.resize(31); // Str31: the HFS name ceiling
    return out;
}

std::u32string decodeUtf8(const std::string& s)
{
    std::u32string out;
    for (size_t i = 0; i < s.size();) {
        const u8 c = static_cast<u8>(s[i]);
        u32 cp = 0xFFFD;
        size_t n = 1;
        if (c < 0x80) {
            cp = c;
        } else if ((c >> 5) == 6 && i + 1 < s.size()) {
            cp = ((c & 0x1Fu) << 6) | (s[i + 1] & 0x3Fu);
            n = 2;
        } else if ((c >> 4) == 14 && i + 2 < s.size()) {
            cp = ((c & 0x0Fu) << 12) | ((s[i + 1] & 0x3Fu) << 6) |
                 (s[i + 2] & 0x3Fu);
            n = 3;
        } else if ((c >> 3) == 30 && i + 3 < s.size()) {
            cp = ((c & 0x07u) << 18) | ((s[i + 1] & 0x3Fu) << 12) |
                 ((s[i + 2] & 0x3Fu) << 6) | (s[i + 3] & 0x3Fu);
            n = 4;
        }
        out.push_back(static_cast<char32_t>(cp));
        i += n;
    }
    return out;
}

// RelString's ASCII behaviour: case-insensitive, otherwise byte order.
u8 foldByte(u8 c) { return c >= 'a' && c <= 'z' ? c - 32 : c; }
int nameCompare(const std::string& a, const std::string& b)
{
    const size_t n = a.size() < b.size() ? a.size() : b.size();
    for (size_t i = 0; i < n; ++i) {
        const u8 fa = foldByte(static_cast<u8>(a[i]));
        const u8 fb = foldByte(static_cast<u8>(b[i]));
        if (fa != fb)
            return fa < fb ? -1 : 1;
    }
    if (a.size() != b.size())
        return a.size() < b.size() ? -1 : 1;
    return 0;
}

struct OSType {
    char c[4] = {'?', '?', '?', '?'};
};
OSType ost(const char* s)
{
    OSType t;
    std::memcpy(t.c, s, 4);
    return t;
}

// One item bound for the catalog.
struct Item {
    bool isDir = false;
    u32 cnid = 0, parent = 0;
    std::string mac;      // MacRoman name
    fs::path host;        // source (files only)
    bool macBinary = false;
    u64 dataOff = 0;      // MacBinary: fork offsets inside the host file
    u64 rsrcOff = 0;
    u64 dataLen = 0, rsrcLen = 0;
    bool adsRsrc = false; // resource fork from the :rsrc alternate stream
    OSType type, creator;
    u16 fdFlags = 0;
    u32 crDate = 0, mdDate = 0;
    u16 valence = 0; // dirs
    // layout results
    u16 dataStart = 0, dataBlocks = 0, rsrcStart = 0, rsrcBlocks = 0;
};

bool readAll(const fs::path& p, u64 off, u64 len, std::vector<u8>& out)
{
    std::ifstream f(p, std::ios::binary);
    if (!f)
        return false;
    f.seekg(static_cast<std::streamoff>(off));
    out.resize(static_cast<size_t>(len));
    f.read(reinterpret_cast<char*>(out.data()),
           static_cast<std::streamsize>(len));
    return static_cast<u64>(f.gcount()) == len;
}

u16 crc16x(const u8* p, size_t n) // XMODEM, MacBinary II's checksum
{
    u16 crc = 0;
    for (size_t i = 0; i < n; ++i) {
        crc ^= static_cast<u16>(p[i]) << 8;
        for (int k = 0; k < 8; ++k)
            crc = (crc & 0x8000u) ? static_cast<u16>((crc << 1) ^ 0x1021u)
                                  : static_cast<u16>(crc << 1);
    }
    return crc;
}

// MacBinary II sniff + decode into the Item. The container download sites
// use; a valid one supplies everything HFS wants back.
bool tryMacBinary(const fs::path& p, u64 fileSize, Item& it)
{
    if (fileSize < 128)
        return false;
    std::vector<u8> h;
    if (!readAll(p, 0, 128, h))
        return false;
    if (h[0] != 0 || h[74] != 0 || h[82] != 0)
        return false;
    const u8 nameLen = h[1];
    if (nameLen < 1 || nameLen > 63)
        return false;
    const u32 dl = get32(&h[83]), rl = get32(&h[87]);
    if (dl > 0x7FFFFFFFu || rl > 0x7FFFFFFFu)
        return false;
    const u64 need = 128u + ((dl + 127u) & ~127u) + rl;
    if (need > fileSize + 127u) // rsrc padding may or may not be present
        return false;
    const u16 crc = get16(&h[124]);
    if (crc != 0 && crc != crc16x(h.data(), 124))
        return false;
    if (crc == 0 && (h[122] || h[123]))
        return false; // claims MBII but carries no checksum: don't trust it
    std::string nm(reinterpret_cast<char*>(&h[2]), nameLen);
    for (char& c : nm)
        if (c == ':')
            c = '_';
    it.mac = nm.size() > 31 ? nm.substr(0, 31) : nm;
    std::memcpy(it.type.c, &h[65], 4);
    std::memcpy(it.creator.c, &h[69], 4);
    it.fdFlags = static_cast<u16>((h[73] << 8) | h[101]);
    it.fdFlags &= static_cast<u16>(~0x0001u); // never carry isOnDesk in
    it.crDate = get32(&h[91]);
    it.mdDate = get32(&h[95]);
    it.macBinary = true;
    it.dataOff = 128;
    it.dataLen = dl;
    it.rsrcOff = 128 + ((dl + 127u) & ~127u);
    it.rsrcLen = rl;
    return true;
}

OSType typeForExt(std::string ext, OSType& creator)
{
    for (char& c : ext)
        c = static_cast<char>(tolower(static_cast<unsigned char>(c)));
    struct M {
        const char *e, *t, *c;
    };
    static const M kMap[] = {
        {".sit", "SITD", "SIT!"}, {".sea", "APPL", "aust"},
        {".hqx", "TEXT", "BnHq"}, {".txt", "TEXT", "ttxt"},
        {".jpg", "JPEG", "ogle"}, {".jpeg", "JPEG", "ogle"},
        {".gif", "GIFf", "ogle"}, {".png", "PNGf", "ogle"},
        {".pdf", "PDF ", "CARO"}, {".mov", "MooV", "TVOD"},
        {".mp3", "MPG3", "TVOD"}, {".img", "dImg", "ddsk"},
        {".smi", "APPL", "oneb"}, {".pct", "PICT", "ogle"},
    };
    for (const auto& m : kMap)
        if (ext == m.e) {
            creator = ost(m.c);
            return ost(m.t);
        }
    creator = ost("????");
    return ost("BINA");
}

bool skipName(const std::string& n)
{
    if (!n.empty() && n[0] == '.')
        return true; // dotfiles, AppleDouble, .DS_Store, .journal, .Trashes
    std::string l = n;
    for (char& c : l)
        c = static_cast<char>(tolower(static_cast<unsigned char>(c)));
    return l == "desktop.ini" || l == "thumbs.db";
}

// A catalog record, key + payload, ready to pack.
struct Rec {
    u32 parent = 0;
    std::string name; // MacRoman
    std::vector<u8> payload;
};

std::vector<u8> keyBytes(u32 parent, const std::string& name, bool padFixed)
{
    // keyLen excludes its own byte. Index keys occupy the fixed maximum so
    // every implementation's reader agrees where the pointer lives.
    std::vector<u8> k;
    const u8 kl = padFixed ? 37u : static_cast<u8>(6 + name.size());
    k.push_back(kl);
    k.push_back(0); // reserved
    k.push_back(static_cast<u8>(parent >> 24));
    k.push_back(static_cast<u8>(parent >> 16));
    k.push_back(static_cast<u8>(parent >> 8));
    k.push_back(static_cast<u8>(parent));
    k.push_back(static_cast<u8>(name.size()));
    for (char c : name)
        k.push_back(static_cast<u8>(c));
    while (k.size() < static_cast<size_t>(kl) + 1u)
        k.push_back(0);
    if (k.size() & 1u)
        k.push_back(0); // records start on even boundaries
    return k;
}

// ── B*-tree packing (bottom-up over sorted records) ───────────────────────
struct BTree {
    std::vector<std::vector<u8>> nodes; // 512 each, node 0 = header
    u32 root = 0, firstLeaf = 0, lastLeaf = 0, leafRecs = 0;
    u16 depth = 0;
};

std::vector<u8> makeNode(u8 type, u8 height,
                         const std::vector<std::vector<u8>>& recs, u32 fLink,
                         u32 bLink)
{
    std::vector<u8> n(512, 0);
    put32(n, 0, fLink);
    put32(n, 4, bLink);
    n[8] = type; // 0xFF leaf, 0 index, 1 header
    n[9] = height;
    put16(n, 10, static_cast<u16>(recs.size()));
    size_t at = 14;
    size_t offSlot = 512 - 2; // record 0's offset lives in the last word
    for (const auto& r : recs) {
        put16(n, offSlot, static_cast<u16>(at));
        std::memcpy(&n[at], r.data(), r.size());
        at += r.size();
        offSlot -= 2;
    }
    put16(n, offSlot, static_cast<u16>(at)); // free-space offset
    return n;
}

// Pack records into 512-byte nodes; each node's usable space must also hold
// the offsets array (2 bytes per record + 2 for free space).
std::vector<std::vector<std::vector<u8>>>
packLevel(const std::vector<std::vector<u8>>& recs)
{
    std::vector<std::vector<std::vector<u8>>> nodes;
    std::vector<std::vector<u8>> cur;
    size_t used = 0;
    for (const auto& r : recs) {
        const size_t withThis = 14 + used + r.size() + 2 * (cur.size() + 2);
        if (!cur.empty() && withThis > 512) {
            nodes.push_back(cur);
            cur.clear();
            used = 0;
        }
        cur.push_back(r);
        used += r.size();
    }
    if (!cur.empty())
        nodes.push_back(cur);
    return nodes;
}

// `fileNodes` is the node count the catalog FILE holds (its allocation-block
// size / 512): the B-tree header must describe exactly that many nodes, with
// the unbuilt tail counted free, or Mac OS's mount-time consistency check
// calls the volume unreadable — measured as the "Mac OS Standard 19.7 MB"
// initialise dialog: partition found, size right, volume refused.
BTree buildCatalogTree(std::vector<Rec>& recs, u32 fileNodes)
{
    std::sort(recs.begin(), recs.end(), [](const Rec& a, const Rec& b) {
        if (a.parent != b.parent)
            return a.parent < b.parent;
        return nameCompare(a.name, b.name) < 0;
    });
    BTree t;
    std::vector<std::vector<u8>> leafRecs;
    for (const auto& r : recs) {
        auto k = keyBytes(r.parent, r.name, false);
        k.insert(k.end(), r.payload.begin(), r.payload.end());
        leafRecs.push_back(std::move(k));
    }
    t.leafRecs = static_cast<u32>(leafRecs.size());
    auto leafGroups = packLevel(leafRecs);
    // Node numbering: 0 header, then leaves, then each index level, root
    // last. Leaves link forward/backward.
    t.nodes.emplace_back(); // header placeholder
    const u32 firstLeaf = 1;
    struct Child {
        u32 node;
        std::vector<u8> firstKey;
    };
    std::vector<Child> level;
    for (size_t i = 0; i < leafGroups.size(); ++i) {
        const u32 num = firstLeaf + static_cast<u32>(i);
        const u32 fl = i + 1 < leafGroups.size() ? num + 1 : 0;
        const u32 bl = i ? num - 1 : 0;
        t.nodes.push_back(makeNode(0xFF, 1, leafGroups[i], fl, bl));
        // Recover the first record's raw key (variable form) for the parent.
        const auto& fr = leafGroups[i][0];
        const u8 kl = fr[0];
        std::vector<u8> key(fr.begin(), fr.begin() + kl + 1);
        level.push_back({num, std::move(key)});
    }
    t.firstLeaf = firstLeaf;
    t.lastLeaf = firstLeaf + static_cast<u32>(leafGroups.size()) - 1;
    u8 height = 1;
    while (level.size() > 1) {
        ++height;
        std::vector<std::vector<u8>> idxRecs;
        for (const auto& c : level) {
            // Re-encode the child's first key at the fixed index width.
            const u32 parent = get32(&c.firstKey[2]);
            std::string nm(reinterpret_cast<const char*>(&c.firstKey[7]),
                           c.firstKey[6]);
            auto k = keyBytes(parent, nm, true);
            k.push_back(static_cast<u8>(c.node >> 24));
            k.push_back(static_cast<u8>(c.node >> 16));
            k.push_back(static_cast<u8>(c.node >> 8));
            k.push_back(static_cast<u8>(c.node));
            idxRecs.push_back(std::move(k));
        }
        auto groups = packLevel(idxRecs);
        std::vector<Child> next;
        size_t at = 0;
        for (const auto& g : groups) {
            const u32 num = static_cast<u32>(t.nodes.size());
            t.nodes.push_back(makeNode(0, height, g, 0, 0));
            next.push_back({num, level[at].firstKey});
            at += g.size();
        }
        level = std::move(next);
    }
    t.root = level.empty() ? 0 : level[0].node;
    t.depth = height;
    // The header node: descriptor + header record (106 bytes) + 128-byte
    // user data + the 256-byte node-use map, offsets [14, 120, 248], free
    // space at 504 — the classic fixed layout.
    std::vector<u8> hdr(512, 0);
    hdr[8] = 1; // ndHdrNode
    put16(hdr, 10, 3);
    put16(hdr, 510, 14);
    put16(hdr, 508, 120);
    put16(hdr, 506, 248);
    put16(hdr, 504, 504); // free space (none: the map runs to 503)
    put16(hdr, 14, t.depth);
    put32(hdr, 16, t.root);
    put32(hdr, 20, t.leafRecs);
    put32(hdr, 24, t.firstLeaf);
    put32(hdr, 28, t.lastLeaf);
    put16(hdr, 32, 512); // node size
    put16(hdr, 34, 37);  // max key length
    put32(hdr, 36, fileNodes);
    put32(hdr, 40, fileNodes - static_cast<u32>(t.nodes.size()));
    for (size_t i = 0; i < t.nodes.size(); ++i)
        hdr[248 + i / 8] |= static_cast<u8>(0x80u >> (i % 8));
    t.nodes[0] = std::move(hdr);
    return t;
}

// "712 MB" / "3.81 GB" — sizes in messages a user has to act on.
std::string sizeText(u64 bytes)
{
    char buf[32];
    if (bytes >= 1000ull * 1000 * 1000)
        std::snprintf(buf, sizeof buf, "%.2f GB",
                      static_cast<double>(bytes) / 1e9);
    else
        std::snprintf(buf, sizeof buf, "%.0f MB",
                      static_cast<double>(bytes) / 1e6);
    return buf;
}

std::vector<u8> emptyExtentsTree(u32 fileNodes)
{
    std::vector<u8> hdr(512, 0);
    hdr[8] = 1;
    put16(hdr, 10, 3);
    put16(hdr, 510, 14);
    put16(hdr, 508, 120);
    put16(hdr, 506, 248);
    put16(hdr, 504, 504);
    put16(hdr, 32, 512);
    put16(hdr, 34, 7); // extents key length
    put32(hdr, 36, fileNodes);
    put32(hdr, 40, fileNodes - 1u);
    hdr[248] = 0x80; // node 0 in use
    return hdr;
}

} // namespace

bool hfsBuildImage(const std::string& folder, const std::string& outPath,
                   const std::string& volName, std::string& err,
                   std::string* warn, const HfsLimits& limits) try {
    err.clear();
    const fs::path root = pathFromUtf8(folder);
    std::error_code ec;
    if (!fs::is_directory(root, ec)) {
        err = "not a folder: " + folder;
        return false;
    }
    // ── Scan ─────────────────────────────────────────────────────────────
    std::vector<Item> items;
    u32 nextCnid = 16;
    const u32 now = hfsNow();
    struct Pending {
        fs::path dir;
        u32 cnid;
    };
    std::vector<Pending> stack{{root, 2u}};
    u64 totalBytes = 0;
    while (!stack.empty()) {
        const Pending cur = stack.back();
        stack.pop_back();
        // Names must be unique per folder AFTER MacRoman folding; a clash
        // is refused with both names so the user renames one, rather than
        // one file silently shadowing another.
        std::map<std::string, std::string> seen;
        for (fs::directory_iterator it(cur.dir, ec), end; it != end;
             it.increment(ec)) {
            if (ec) {
                err = "scan failed in " + utf8Of(cur.dir);
                return false;
            }
            const fs::path& p = it->path();
            const std::string host8 = utf8Of(p.filename());
            if (skipName(host8))
                continue;
            Item item;
            item.parent = cur.cnid;
            item.crDate = item.mdDate = now;
            if (it->is_directory(ec)) {
                item.isDir = true;
                item.cnid = nextCnid++;
                item.mac = toMacRoman(decodeUtf8(host8));
                item.host = p;
                stack.push_back({p, item.cnid});
            } else if (it->is_regular_file(ec)) {
                const u64 sz = static_cast<u64>(it->file_size(ec));
                item.cnid = nextCnid++;
                item.host = p;
                bool mb = false;
                {
                    std::string l = host8;
                    for (char& c : l)
                        c = static_cast<char>(
                            tolower(static_cast<unsigned char>(c)));
                    if (l.size() > 4 && l.compare(l.size() - 4, 4, ".bin") == 0)
                        mb = tryMacBinary(p, sz, item);
                }
                if (!mb) {
                    item.mac = toMacRoman(decodeUtf8(host8));
                    item.dataOff = 0;
                    item.dataLen = sz;
                    // The resource fork, when 7-Zip (or another Mac-aware
                    // extractor) left it beside the data as an alternate
                    // stream.
                    fs::path rs = p;
                    rs += ":rsrc";
                    std::error_code rec2;
                    const u64 rl = static_cast<u64>(fs::file_size(rs, rec2));
                    if (!rec2 && rl > 0) {
                        item.adsRsrc = true;
                        item.rsrcLen = rl;
                    }
                    item.type = typeForExt(utf8Of(p.extension()),
                                           item.creator);
                    // A PEF executable copied bare still deserves to launch.
                    std::vector<u8> magic;
                    if (item.dataLen >= 8 && readAll(p, 0, 8, magic) &&
                        std::memcmp(magic.data(), "Joy!peff", 8) == 0) {
                        item.type = ost("APPL");
                    }
                }
                // An HFS fork length is a signed 32-bit field: a file this
                // big cannot exist on the volume at all — its record would
                // carry a negative size. Left out loudly; the share still
                // builds and mounts without it.
                if (item.dataLen > limits.maxForkBytes ||
                    item.rsrcLen > limits.maxForkBytes) {
                    if (warn)
                        *warn += "left out \"" + host8 + "\" (" +
                                 sizeText(item.dataLen + item.rsrcLen) +
                                 ") — an HFS fork tops out at 2 GB\n";
                    continue;
                }
                totalBytes += item.dataLen + item.rsrcLen;
            } else {
                continue; // symlinks and oddities
            }
            if (item.mac.empty())
                item.mac = "_";
            auto clash = seen.find(item.mac);
            if (clash != seen.end()) {
                err = "name clash after Mac conversion: '" + clash->second +
                      "' and '" + host8 + "' both become '" + item.mac + "'";
                return false;
            }
            seen[item.mac] = host8;
            items.push_back(std::move(item));
        }
    }
    if (items.empty()) {
        err = warn && !warn->empty() ? "nothing was shareable: " + *warn
                                     : "the folder is empty — nothing to share";
        return false;
    }
    // ── Volume geometry ──────────────────────────────────────────────────
    // Allocation blocks are 16-bit, so the block size scales with content.
    // Slack: trees + bitmap + rounding, padded generously — free space on a
    // read-only volume costs nothing but image bytes.
    u32 abSize = 512;
    const u64 want = totalBytes + (totalBytes >> 3) + (2u << 20);
    while (static_cast<u64>(abSize) * 65000u < want)
        abSize *= 2;
    auto blocksFor = [&](u64 bytes) {
        return static_cast<u32>((bytes + abSize - 1) / abSize);
    };
    // ── Catalog records ──────────────────────────────────────────────────
    // Layout order in the allocation area: catalog file, extents file, then
    // every fork in item order.
    u32 nFiles = 0, nDirs = 0;
    std::map<u32, u16> valence; // parent → children
    for (const auto& it : items)
        ++valence[it.parent];
    u16 rootFiles = 0, rootDirs = 0;
    for (const auto& it : items) {
        if (it.parent == 2u) {
            if (it.isDir)
                ++rootDirs;
            else
                ++rootFiles;
        }
        if (it.isDir)
            ++nDirs;
        else
            ++nFiles;
    }
    // Fork placement, after the two tree files whose sizes need the record
    // count first — so build records with placeholder extents, then patch.
    // Simpler: compute tree size bounds first (records are size-known),
    // place forks, then emit records once.
    std::vector<Rec> recs;
    // Root directory record under the root parent, plus its thread.
    {
        Rec r;
        r.parent = 1;
        r.name = toMacRoman(decodeUtf8(volName)).substr(0, 27);
        r.payload.assign(70, 0);
        r.payload[0] = 1; // directory record
        put16(r.payload, 4, static_cast<u16>(valence[2u]));
        put32(r.payload, 6, 2);
        put32(r.payload, 10, now);
        put32(r.payload, 14, now);
        recs.push_back(r);
        Rec th;
        th.parent = 2;
        th.name = "";
        th.payload.assign(46, 0);
        th.payload[0] = 3; // directory thread
        put32(th.payload, 10, 1);
        th.payload[14] = static_cast<u8>(r.name.size());
        std::memcpy(&th.payload[15], r.name.data(), r.name.size());
        recs.push_back(th);
    }
    for (const auto& it : items) {
        if (!it.isDir)
            continue;
        Rec r;
        r.parent = it.parent;
        r.name = it.mac;
        r.payload.assign(70, 0);
        r.payload[0] = 1;
        put16(r.payload, 4, valence.count(it.cnid)
                                ? static_cast<u16>(valence[it.cnid])
                                : 0u);
        put32(r.payload, 6, it.cnid);
        put32(r.payload, 10, it.crDate);
        put32(r.payload, 14, it.mdDate);
        recs.push_back(r);
        Rec th;
        th.parent = it.cnid;
        th.name = "";
        th.payload.assign(46, 0);
        th.payload[0] = 3;
        put32(th.payload, 10, it.parent);
        th.payload[14] = static_cast<u8>(it.mac.size());
        std::memcpy(&th.payload[15], it.mac.data(), it.mac.size());
        recs.push_back(th);
    }
    // Files get their records after fork placement (extents go inside), so
    // reserve the size math now: a file record with key ≈ 8+31+102 bytes.
    // Compute an upper bound for catalog bytes to place the tree file.
    size_t recBytesUB = 0;
    for (const auto& r : recs)
        recBytesUB += 8 + r.name.size() + r.payload.size() + 4;
    for (const auto& it : items)
        if (!it.isDir)
            recBytesUB += 8 + it.mac.size() + 102 + 4;
    // Nodes hold ~480 usable bytes; triple the estimate for index levels
    // and rounding, then round to whole nodes.
    const u32 catNodesUB =
        static_cast<u32>((recBytesUB + 479) / 480 * 2 + 8);
    const u32 catBlocks = blocksFor(static_cast<u64>(catNodesUB) * 512u);
    const u32 extBlocks = blocksFor(512);
    u32 nextBlock = catBlocks + extBlocks;
    std::vector<Item*> files;
    for (auto& it : items)
        if (!it.isDir)
            files.push_back(&it);
    for (Item* f : files) {
        f->dataStart = static_cast<u16>(nextBlock);
        f->dataBlocks = static_cast<u16>(blocksFor(f->dataLen));
        nextBlock += f->dataBlocks;
        f->rsrcStart = static_cast<u16>(nextBlock);
        f->rsrcBlocks = static_cast<u16>(blocksFor(f->rsrcLen));
        nextBlock += f->rsrcBlocks;
        if (nextBlock > 65000u) {
            err = "content outgrew the volume model — raise the size law";
            return false;
        }
    }
    const u32 usedBlocks = nextBlock;
    const u32 totalBlocks = usedBlocks + 64; // visible free slack
    // The file record, Inside Macintosh's exact layout: type/pad/flags/typ,
    // FInfo at 4, CNID at 20, data fork (start/logical/physical) at 24, the
    // resource fork triple at 34, three dates at 44, FXInfo at 56, clump at
    // 72, then the two first-extent records at 74 and 86, reserved to 102.
    for (const Item* f : files) {
        Rec r;
        r.parent = f->parent;
        r.name = f->mac;
        r.payload.assign(102, 0);
        r.payload[0] = 2; // file record
        std::memcpy(&r.payload[4], f->type.c, 4);
        std::memcpy(&r.payload[8], f->creator.c, 4);
        put16(r.payload, 12, f->fdFlags);
        put32(r.payload, 20, f->cnid);
        put16(r.payload, 24, f->dataBlocks ? f->dataStart : 0);
        put32(r.payload, 26, static_cast<u32>(f->dataLen));
        put32(r.payload, 30, static_cast<u32>(f->dataBlocks) * abSize);
        put16(r.payload, 34, f->rsrcBlocks ? f->rsrcStart : 0);
        put32(r.payload, 36, static_cast<u32>(f->rsrcLen));
        put32(r.payload, 40, static_cast<u32>(f->rsrcBlocks) * abSize);
        put32(r.payload, 44, f->crDate ? f->crDate : now);
        put32(r.payload, 48, f->mdDate ? f->mdDate : now);
        if (f->dataBlocks) {
            put16(r.payload, 74, f->dataStart);
            put16(r.payload, 76, f->dataBlocks);
        }
        if (f->rsrcBlocks) {
            put16(r.payload, 86, f->rsrcStart);
            put16(r.payload, 88, f->rsrcBlocks);
        }
        recs.push_back(std::move(r));
    }
    // ── Trees ────────────────────────────────────────────────────────────
    const u32 catFileNodes =
        static_cast<u32>(static_cast<u64>(catBlocks) * abSize / 512u);
    if (catFileNodes > 2048u) {
        // The header node's 256-byte map covers 2048 nodes; a share this
        // big needs map nodes this builder does not write.
        err = "too many catalog nodes for the header map — split the share";
        return false;
    }
    BTree cat = buildCatalogTree(recs, catFileNodes);
    if (cat.nodes.size() * 512u > static_cast<u64>(catBlocks) * abSize) {
        err = "catalog outgrew its reservation — raise the estimate";
        return false;
    }
    // ── Emit ─────────────────────────────────────────────────────────────
    //
    // ⚠ The volume does NOT sit bare at sector 0. Mac OS 9's CD probe was
    // MEASURED refusing both a bare HFS+ and a bare classic-HFS volume with
    // the "unreadable … ProDOS 0K" initialise dialog; what every pressed
    // Mac disc carries — and what the same probe then accepts — is a Driver
    // Descriptor Map at block 0 and an Apple Partition Map naming an
    // Apple_HFS partition. The wrapper written here mirrors a real one
    // field for field (map at blocks 1..63, partition at 64, HFS status
    // 0x40000033); everything inside the partition is partition-relative,
    // so the volume itself is unchanged by the dress.
    const u32 kPartBase = 64; // DDM + the customary 63-block map
    const u32 bmSectors = (totalBlocks + 4095u) / 4096u;
    const u32 alBlSt = 3u + bmSectors;
    const u64 volSectors = static_cast<u64>(alBlSt) +
                           static_cast<u64>(totalBlocks) * (abSize / 512u) +
                           2u;
    // Whole image padded to the ATAPI 2048 grain; the tail is a free
    // partition when the pad leaves one.
    u64 imageSectors = kPartBase + volSectors;
    imageSectors = (imageSectors + 3u) & ~3ull;
    const u64 freeSectors = imageSectors - kPartBase - volSectors;
    // ⚠ The guest is the ceiling here, not HFS. Classic Mac OS positions
    // block I/O with SIGNED 32-BIT BYTE OFFSETS, so nothing past the 2 GB
    // line of a volume is reachable — measured on a 10.8 GB share, which
    // MOUNTED and BROWSED perfectly (the catalog sits at the front) and
    // then answered every Finder copy with an error, because every fork
    // the user wanted lay past the line. The worst failure shape: visible
    // success hiding certain failure. Refused here, before gigabytes get
    // written to the host's disk.
    if (imageSectors * 512u > limits.maxImageBytes) {
        err = "the folder needs a " + sizeText(imageSectors * 512u) +
              " volume and the guest's classic CD path can only address " +
              sizeText(limits.maxImageBytes) +
              " — share a subfolder, or a few titles at a time";
        return false;
    }
    std::ofstream out(pathFromUtf8(outPath), std::ios::binary | std::ios::trunc);
    if (!out) {
        err = "cannot write " + outPath;
        return false;
    }
    // A failure past this point leaves a partial image; remove it so a
    // caller (or the CD slot) can never mount half a volume.
    auto failOut = [&](std::string msg) {
        out.close();
        std::error_code rc;
        fs::remove(pathFromUtf8(outPath), rc);
        err = std::move(msg);
        return false;
    };
    auto seekAbs = [&](u64 s) {
        out.seekp(static_cast<std::streamoff>(s * 512u));
    };
    auto seekSector = [&](u64 s) { seekAbs(kPartBase + s); };
    auto seekBlock = [&](u32 b, u64 plus) {
        out.seekp(static_cast<std::streamoff>(
            (static_cast<u64>(kPartBase) + alBlSt) * 512u +
            static_cast<u64>(b) * abSize + plus));
    };
    // Grow the file to its final size first so sparse seeks are safe.
    seekAbs(imageSectors - 1);
    {
        std::vector<u8> z(512, 0);
        out.write(reinterpret_cast<char*>(z.data()), 512);
    }
    // Driver Descriptor Map, block 0: signature, 512-byte blocks, total.
    {
        std::vector<u8> ddm(512, 0);
        put16(ddm, 0, 0x4552); // 'ER'
        put16(ddm, 2, 512);
        put32(ddm, 4, static_cast<u32>(imageSectors));
        seekAbs(0);
        out.write(reinterpret_cast<char*>(ddm.data()), 512);
    }
    // The partition map. Entry layout: sig 'PM', map entry count at 4,
    // physical start at 8, block count at 12, name at 16, type at 48,
    // logical data start at 80, data count at 84, status at 88.
    {
        const u32 nEntries = freeSectors ? 3u : 2u;
        auto entry = [&](u32 slot, const char* name, const char* type,
                         u32 start, u32 count, u32 status) {
            std::vector<u8> e(512, 0);
            put16(e, 0, 0x504D); // 'PM'
            put32(e, 4, nEntries);
            put32(e, 8, start);
            put32(e, 12, count);
            std::snprintf(reinterpret_cast<char*>(&e[16]), 32, "%s", name);
            std::snprintf(reinterpret_cast<char*>(&e[48]), 32, "%s", type);
            put32(e, 84, count);
            put32(e, 88, status);
            seekAbs(slot);
            out.write(reinterpret_cast<char*>(e.data()), 512);
        };
        entry(1, "Apple", "Apple_partition_map", 1, kPartBase - 1, 0x3u);
        const std::string vn8 = toMacRoman(decodeUtf8(volName)).substr(0, 27);
        entry(2, vn8.c_str(), "Apple_HFS", kPartBase,
              static_cast<u32>(volSectors), 0x40000033u);
        if (freeSectors)
            entry(3, "Extra", "Apple_Free",
                  static_cast<u32>(kPartBase + volSectors),
                  static_cast<u32>(freeSectors), 0);
    }
    // MDB
    std::vector<u8> mdb(512, 0);
    const std::string vn = toMacRoman(decodeUtf8(volName)).substr(0, 27);
    put16(mdb, 0, 0x4244); // 'BD'
    put32(mdb, 2, now);
    put32(mdb, 6, now);
    put16(mdb, 10, 0x0100); // cleanly unmounted
    put16(mdb, 12, rootFiles);
    put16(mdb, 14, 3); // bitmap starts at sector 3
    put16(mdb, 16, static_cast<u16>(usedBlocks));
    put16(mdb, 18, static_cast<u16>(totalBlocks));
    put32(mdb, 20, abSize);
    put32(mdb, 24, abSize); // clump
    put16(mdb, 28, static_cast<u16>(alBlSt));
    put32(mdb, 30, nextCnid);
    put16(mdb, 34, static_cast<u16>(totalBlocks - usedBlocks));
    mdb[36] = static_cast<u8>(vn.size());
    std::memcpy(&mdb[37], vn.data(), vn.size());
    put32(mdb, 74, abSize);  // extents clump
    put32(mdb, 78, abSize);  // catalog clump
    put16(mdb, 82, rootDirs);
    put32(mdb, 84, nFiles);
    put32(mdb, 88, nDirs);
    put32(mdb, 130, extBlocks * abSize); // extents file size
    put16(mdb, 134, static_cast<u16>(catBlocks)); // extents file start
    put16(mdb, 136, static_cast<u16>(extBlocks));
    put32(mdb, 146, catBlocks * abSize); // catalog file size
    put16(mdb, 150, 0);                  // catalog starts at block 0
    put16(mdb, 152, static_cast<u16>(catBlocks));
    seekSector(2);
    out.write(reinterpret_cast<char*>(mdb.data()), 512);
    // Alternate MDB, second-to-last sector.
    seekSector(volSectors - 2);
    out.write(reinterpret_cast<char*>(mdb.data()), 512);
    // Bitmap: blocks 0..usedBlocks-1 in use.
    {
        std::vector<u8> bm(bmSectors * 512u, 0);
        for (u32 i = 0; i < usedBlocks; ++i)
            bm[i / 8] |= static_cast<u8>(0x80u >> (i % 8));
        seekSector(3);
        out.write(reinterpret_cast<char*>(bm.data()),
                  static_cast<std::streamsize>(bm.size()));
    }
    // Catalog + extents trees.
    for (size_t i = 0; i < cat.nodes.size(); ++i) {
        seekBlock(0, i * 512u);
        out.write(reinterpret_cast<char*>(cat.nodes[i].data()), 512);
    }
    {
        auto ext = emptyExtentsTree(
            static_cast<u32>(static_cast<u64>(extBlocks) * abSize / 512u));
        seekBlock(catBlocks, 0);
        out.write(reinterpret_cast<char*>(ext.data()), 512);
    }
    // Forks, streamed.
    auto streamFork = [&](const fs::path& p, u64 off, u64 len, u32 block,
                          bool ads) -> bool {
        if (!len)
            return true;
        fs::path src = p;
        if (ads)
            src += ":rsrc";
        std::ifstream in(src, std::ios::binary);
        if (!in)
            return false;
        in.seekg(static_cast<std::streamoff>(off));
        seekBlock(block, 0);
        std::vector<char> buf(1u << 20);
        u64 left = len;
        while (left) {
            const u64 take =
                left < buf.size() ? left : static_cast<u64>(buf.size());
            in.read(buf.data(), static_cast<std::streamsize>(take));
            if (static_cast<u64>(in.gcount()) != take)
                return false;
            out.write(buf.data(), static_cast<std::streamsize>(take));
            left -= take;
        }
        return true;
    };
    for (const Item* f : files) {
        if (!streamFork(f->host, f->macBinary ? f->dataOff : 0, f->dataLen,
                        f->dataStart, false))
            return failOut("read failed: " + utf8Of(f->host));
        if (f->rsrcLen) {
            if (!streamFork(f->host, f->macBinary ? f->rsrcOff : 0,
                            f->rsrcLen, f->rsrcStart,
                            !f->macBinary && f->adsRsrc))
                return failOut("resource fork read failed: " + utf8Of(f->host));
        }
    }
    // The image is already a 2048 multiple by construction (imageSectors is
    // rounded to fours), so the ATAPI slot serves it as-is.
    out.flush();
    if (!out)
        return failOut("write failed: " + outPath);
    return true;
} catch (const std::exception& e) {
    // The path layer throws on things like ill-encoded names; a build
    // function's contract is an error string, never a fail-fast. The capi
    // and the shell lean on exactly this.
    err = std::string("hfs build failed: ") + e.what();
    return false;
}

} // namespace opm
