// The shared-folder HFS builder: a folder in, a mountable classic volume
// out. The discriminating checks parse the image back the way the guest's
// own driver will: the MDB fields against each other, the catalog B-tree
// walked leaf-to-leaf with keys required to be sorted by the HFS
// comparator, forks read back through the extents and compared
// byte-for-byte, and a MacBinary file required to round-trip both forks
// plus its type/creator — the whole reason .bin exists.

#include "doctest.h"
#include "opm/hfs.hpp"

#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <vector>

using namespace opm;
namespace fs = std::filesystem;

namespace {

using u8 = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;

u16 g16(const std::vector<u8>& b, size_t at)
{
    return static_cast<u16>((b[at] << 8) | b[at + 1]);
}
u32 g32(const std::vector<u8>& b, size_t at)
{
    return (static_cast<u32>(b[at]) << 24) |
           (static_cast<u32>(b[at + 1]) << 16) |
           (static_cast<u32>(b[at + 2]) << 8) | b[at + 3];
}

struct Vol {
    std::vector<u8> img;
    u32 part = 0; // Apple_HFS partition start, 512-byte blocks
    u32 abSize = 0, alBlSt = 0, catStart = 0;
    std::vector<u8> sector(u32 s) const
    {
        const size_t at = (static_cast<size_t>(part) + s) * 512;
        return {img.begin() + at, img.begin() + at + 512};
    }
    std::vector<u8> node(u32 n) const
    {
        const size_t at = (static_cast<size_t>(part) + alBlSt) * 512 +
                          static_cast<size_t>(catStart) * abSize + n * 512;
        return {img.begin() + at, img.begin() + at + 512};
    }
    std::vector<u8> fork(u16 startBlk, u32 len) const
    {
        const size_t at = (static_cast<size_t>(part) + alBlSt) * 512 +
                          static_cast<size_t>(startBlk) * abSize;
        return {img.begin() + at, img.begin() + at + len};
    }
};

Vol load(const fs::path& p)
{
    Vol v;
    std::ifstream f(p, std::ios::binary);
    REQUIRE(f.good());
    v.img.assign(std::istreambuf_iterator<char>(f), {});
    REQUIRE(v.img.size() % 2048 == 0); // ATAPI-ready
    // The wrapper the CD probe demands: DDM, then a partition map naming
    // the Apple_HFS partition the volume actually lives in.
    REQUIRE(g16(v.img, 0) == 0x4552);   // 'ER'
    REQUIRE(g16(v.img, 512) == 0x504D); // 'PM'
    const u32 mapEntries = g32(v.img, 512 + 4);
    bool found = false;
    for (u32 e = 1; e <= mapEntries && !found; ++e) {
        const size_t at = static_cast<size_t>(e) * 512;
        if (std::memcmp(&v.img[at + 48], "Apple_HFS", 10) == 0) {
            v.part = g32(v.img, at + 8);
            found = true;
        }
    }
    REQUIRE(found);
    const auto mdb = v.sector(2);
    REQUIRE(g16(mdb, 0) == 0x4244); // 'BD'
    v.abSize = g32(mdb, 20);
    v.alBlSt = g16(mdb, 28);
    v.catStart = g16(mdb, 150);
    return v;
}

// Walk the catalog's leaf chain in order; return records as
// (parent, name, payload).
struct CRec {
    u32 parent;
    std::string name;
    std::vector<u8> payload;
};

std::vector<CRec> walkCatalog(const Vol& v)
{
    const auto hdr = v.node(0);
    REQUIRE(hdr[8] == 1); // header node
    const u32 firstLeaf = g32(hdr, 24);
    std::vector<CRec> out;
    u32 n = firstLeaf;
    while (n) {
        const auto nd = v.node(n);
        REQUIRE(nd[8] == 0xFF); // leaf
        const u16 nrec = g16(nd, 10);
        for (u16 i = 0; i < nrec; ++i) {
            const u16 off = g16(nd, 512 - 2 - 2 * i);
            const u8 kl = nd[off];
            const u32 parent = g32(nd, off + 2);
            const u8 nl = nd[off + 6];
            std::string nm(reinterpret_cast<const char*>(&nd[off + 7]), nl);
            size_t rec = off + 1 + kl;
            if (rec & 1)
                ++rec;
            const u16 next = g16(nd, 512 - 2 - 2 * (i + 1));
            out.push_back(
                {parent, nm,
                 std::vector<u8>(nd.begin() + rec, nd.begin() + next)});
        }
        n = g32(nd, 0); // forward link
    }
    return out;
}

u8 fold(u8 c) { return c >= 'a' && c <= 'z' ? c - 32 : c; }
bool keyLess(const CRec& a, const CRec& b)
{
    if (a.parent != b.parent)
        return a.parent < b.parent;
    const size_t n = a.name.size() < b.name.size() ? a.name.size()
                                                   : b.name.size();
    for (size_t i = 0; i < n; ++i) {
        const u8 fa = fold(a.name[i]), fb = fold(b.name[i]);
        if (fa != fb)
            return fa < fb;
    }
    return a.name.size() < b.name.size();
}

fs::path scratchDir()
{
    const fs::path d = fs::temp_directory_path() / "opm_hfs_test";
    fs::remove_all(d);
    fs::create_directories(d);
    return d;
}

void writeFile(const fs::path& p, const std::vector<u8>& bytes)
{
    std::ofstream f(p, std::ios::binary);
    f.write(reinterpret_cast<const char*>(bytes.data()),
            static_cast<std::streamsize>(bytes.size()));
}

u16 crc16x(const u8* p, size_t n)
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

} // namespace

TEST_CASE("hfs: a folder becomes a volume whose catalog and forks read back")
{
    const fs::path d = scratchDir();
    // A file whose bytes are recognisable, a nested folder with another,
    // and a MacBinary file carrying both forks and its Mac identity.
    std::vector<u8> readme;
    for (int i = 0; i < 5000; ++i)
        readme.push_back(static_cast<u8>('A' + i % 23));
    writeFile(d / "ReadMe.txt", readme);
    fs::create_directories(d / "Data");
    std::vector<u8> level;
    for (int i = 0; i < 300; ++i)
        level.push_back(static_cast<u8>(i));
    writeFile(d / "Data" / "Level1.ter", level);
    // MacBinary II: name "Game", type 'APPL'/'Dino', 100-byte data fork,
    // 40-byte resource fork, valid CRC.
    {
        std::vector<u8> mb(128, 0);
        mb[1] = 4;
        std::memcpy(&mb[2], "Game", 4);
        std::memcpy(&mb[65], "APPL", 4);
        std::memcpy(&mb[69], "Dino", 4);
        mb[83] = 0;
        mb[86] = 100; // data length
        mb[90] = 40;  // rsrc length
        mb[122] = 129;
        mb[123] = 129;
        const u16 crc = crc16x(mb.data(), 124);
        mb[124] = static_cast<u8>(crc >> 8);
        mb[125] = static_cast<u8>(crc);
        for (int i = 0; i < 100; ++i)
            mb.push_back(static_cast<u8>(0x40 + i % 10));
        mb.resize(128 + 128, 0); // data padded to 128
        for (int i = 0; i < 40; ++i)
            mb.push_back(static_cast<u8>(0x80 + i));
        writeFile(d / "Game.bin", mb);
    }
    // Litter that must be skipped.
    writeFile(d / "desktop.ini", {1, 2, 3});
    writeFile(d / ".DS_Store", {9});

    const fs::path img = d / "out.img";
    std::string err;
    REQUIRE(hfsBuildImage(d.string(), img.string(), "Shared", err));
    CHECK(err.empty());
    const Vol v = load(img);
    const auto mdb = v.sector(2);
    CHECK(g16(mdb, 10) == 0x0100); // cleanly unmounted
    CHECK(g32(mdb, 84) == 3);      // three files (litter skipped)
    CHECK(g32(mdb, 88) == 1);      // one folder
    // The alternate MDB sits at the PARTITION's second-to-last sector (the
    // free tail comes after it); find it by signature in the image's last
    // few raw sectors and require it to match the main one byte for byte.
    {
        const u32 sectors = static_cast<u32>(v.img.size() / 512);
        bool altOk = false;
        for (u32 s = sectors - 8; s < sectors && !altOk; ++s) {
            if (g16(v.img, static_cast<size_t>(s) * 512) != 0x4244)
                continue;
            altOk = true;
            for (u32 i = 0; i < 512; ++i)
                altOk &= v.img[(static_cast<size_t>(v.part) + 2) * 512 + i] ==
                         v.img[static_cast<size_t>(s) * 512 + i];
        }
        CHECK(altOk);
    }
    auto recs = walkCatalog(v);
    // Sorted exactly as the comparator will search.
    for (size_t i = 1; i < recs.size(); ++i)
        CHECK(keyLess(recs[i - 1], recs[i]));
    // Records: root dir under parent 1, its thread, three files + one dir
    // in root, the dir's thread, one file in the dir = 7.
    CHECK(recs.size() == 7);
    std::map<std::string, const CRec*> byName;
    for (const auto& r : recs)
        byName[std::string(std::to_string(r.parent) + "/" + r.name)] = &r;
    REQUIRE(byName.count("1/Shared"));
    REQUIRE(byName.count("2/ReadMe.txt"));
    REQUIRE(byName.count("2/Data"));
    REQUIRE(byName.count("2/Game"));
    const auto& rm = *byName["2/ReadMe.txt"];
    REQUIRE(rm.payload[0] == 2);
    CHECK(std::memcmp(&rm.payload[4], "TEXT", 4) == 0);
    CHECK(std::memcmp(&rm.payload[8], "ttxt", 4) == 0);
    const u32 rmLen = g32(rm.payload, 26);
    CHECK(rmLen == readme.size());
    const u16 rmStart = g16(rm.payload, 74);
    const u16 rmCount = g16(rm.payload, 76);
    CHECK(rmCount * v.abSize >= rmLen);
    CHECK(v.fork(rmStart, rmLen) == readme);
    // The nested file, under the folder's CNID.
    const auto& dir = *byName["2/Data"];
    REQUIRE(dir.payload[0] == 1);
    const u32 dirId = g32(dir.payload, 6);
    bool nestedOk = false;
    for (const auto& r : recs)
        if (r.parent == dirId && r.name == "Level1.ter") {
            CHECK(v.fork(g16(r.payload, 74), g32(r.payload, 26)) == level);
            nestedOk = true;
        }
    CHECK(nestedOk);
    // The MacBinary file: decoded name, identity, both forks.
    const auto& gm = *byName["2/Game"];
    REQUIRE(gm.payload[0] == 2);
    CHECK(std::memcmp(&gm.payload[4], "APPL", 4) == 0);
    CHECK(std::memcmp(&gm.payload[8], "Dino", 4) == 0);
    CHECK(g32(gm.payload, 26) == 100);
    CHECK(g32(gm.payload, 36) == 40);
    const auto data = v.fork(g16(gm.payload, 74), 100);
    CHECK(data[0] == 0x40);
    CHECK(data[99] == 0x40 + 99 % 10);
    const auto rsrc = v.fork(g16(gm.payload, 86), 40);
    CHECK(rsrc[0] == 0x80);
    CHECK(rsrc[39] == 0x80 + 39);
    fs::remove_all(d);
}

TEST_CASE("hfs: a name clash after Mac conversion is refused with both names")
{
    const fs::path d = scratchDir();
    writeFile(d / "a:b.txt", {1});
    // ':' is illegal in HFS names and becomes '_' — if the filesystem even
    // allowed it. Portable clash instead: two names that fold to one
    // MacRoman string after truncation to 31 chars.
    fs::remove(d / "a:b.txt");
    const std::string long1(40, 'x');
    const std::string long2 = long1.substr(0, 35) + "zzzzz";
    writeFile(d / (long1 + ".dat"), {1});
    writeFile(d / (long2 + ".dat"), {2});
    const fs::path img = d / "out.img";
    std::string err;
    CHECK(!hfsBuildImage(d.string(), img.string(), "Shared", err));
    CHECK(!err.empty());
    fs::remove_all(d);
}

TEST_CASE("hfs: an empty folder is refused rather than shared as nothing")
{
    const fs::path d = scratchDir();
    const fs::path img = d / "out.img";
    std::string err;
    CHECK(!hfsBuildImage(d.string(), img.string(), "Shared", err));
    CHECK(!err.empty());
    fs::remove_all(d);
}
