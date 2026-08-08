// The CD image layer: where LBA N's 2048 user bytes live is the WHOLE
// difference between a .iso, a raw .bin rip with its .cue sheet, and a
// Disk Utility .dmg — and every one of these mappings is exercised against
// files built byte-by-byte right here, so a wrong offset fails in this
// binary rather than as a guest that mounts garbage.
//
// The zlib stream embedded in the UDIF case was produced by an INDEPENDENT
// implementation (.NET DeflateStream), so the inflate here is checked
// against something other than itself.

#include "doctest.h"
#include "opm/ata.hpp"
#include "opm/cdimage.hpp"

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace opm;

namespace {

bool writeAll(const char* path, const std::vector<u8>& bytes)
{
    FILE* f = fopen(path, "wb");
    if (!f)
        return false;
    const bool ok =
        bytes.empty() || fwrite(bytes.data(), 1, bytes.size(), f) ==
                             bytes.size();
    fclose(f);
    return ok;
}

bool writeText(const char* path, const std::string& text)
{
    return writeAll(path,
                    std::vector<u8>(text.begin(), text.end()));
}

// The user-data byte at position j of LBA n, for every synthetic disc.
u8 pay(u64 n, u32 j) { return static_cast<u8>(n * 7 + j * 3 + 1); }

// One raw 2352-byte sector: sync mark, header, patterned payload, junk EDC.
void rawSector(std::vector<u8>& out, u64 lba, u8 mode)
{
    static const u8 sync[12] = {0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
    out.insert(out.end(), sync, sync + 12);
    out.push_back(static_cast<u8>(lba / 4500)); // fake MSF, unread by us
    out.push_back(static_cast<u8>((lba / 75) % 60));
    out.push_back(static_cast<u8>(lba % 75));
    out.push_back(mode);
    const u32 off = mode == 2 ? 24u : 16u;
    for (u32 j = 16; j < 16 + 8 && mode == 2; ++j)
        out.push_back(0xA5); // mode 2 subheader before the user data
    for (u32 j = 0; j < 2048; ++j)
        out.push_back(pay(lba, j));
    while (out.size() % 2352)
        out.push_back(0xEC); // EDC/ECC filler
    (void)off;
}

std::string base64(const std::vector<u8>& in)
{
    static const char* k =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string s;
    size_t i = 0;
    for (; i + 2 < in.size(); i += 3) {
        const u32 v = (u32(in[i]) << 16) | (u32(in[i + 1]) << 8) | in[i + 2];
        s += k[v >> 18];
        s += k[(v >> 12) & 63];
        s += k[(v >> 6) & 63];
        s += k[v & 63];
    }
    if (i + 1 == in.size()) {
        const u32 v = u32(in[i]) << 16;
        s += k[v >> 18];
        s += k[(v >> 12) & 63];
        s += "==";
    } else if (i + 2 == in.size()) {
        const u32 v = (u32(in[i]) << 16) | (u32(in[i + 1]) << 8);
        s += k[v >> 18];
        s += k[(v >> 12) & 63];
        s += k[(v >> 6) & 63];
        s += '=';
    }
    return s;
}

void be32at(std::vector<u8>& v, size_t at, u32 x)
{
    v[at] = static_cast<u8>(x >> 24);
    v[at + 1] = static_cast<u8>(x >> 16);
    v[at + 2] = static_cast<u8>(x >> 8);
    v[at + 3] = static_cast<u8>(x);
}

void be64at(std::vector<u8>& v, size_t at, u64 x)
{
    be32at(v, at, static_cast<u32>(x >> 32));
    be32at(v, at + 4, static_cast<u32>(x));
}

// A zlib stream produced by .NET's DeflateStream (an independent DEFLATE
// implementation), decompressing to 512 bytes of
// "OpenPowerMac reads discs. " x19 + "OpenPowerMac reads".
const u8 kZlib512[] = {
    0x78, 0x9c, 0xf3, 0x2f, 0x48, 0xcd, 0x0b, 0xc8, 0x2f, 0x4f, 0x2d,
    0xf2, 0x4d, 0x4c, 0x56, 0x28, 0x4a, 0x4d, 0x4c, 0x29, 0x56, 0x48,
    0xc9, 0x2c, 0x4e, 0x2e, 0xd6, 0x53, 0xf0, 0x1f, 0x95, 0x51, 0x18,
    0xfe, 0x61, 0x00, 0x00, 0xaf, 0x71, 0xb9, 0x39};

std::vector<u8> zlibPlain512()
{
    std::string t;
    for (int k = 0; k < 19; ++k)
        t += "OpenPowerMac reads discs. ";
    t += "OpenPowerMac reads";
    return std::vector<u8>(t.begin(), t.end());
}

// Deliver one 12-byte CDB the way the driver does: byte-count limit,
// PACKET, then six 16-bit writes of the data register. The limit matters:
// it is per-DRQ-chunk, and a driver that skips it inherits the previous
// reply's size as its transfer granularity.
void sendPacket(AtaCell& c, const u8* cdb)
{
    c.write(0x060, 0x00, 1);
    c.write(0x040, 0xFE, 1);
    c.write(0x050, 0xFF, 1);
    c.write(0x070, 0xA0, 1);
    for (u32 k = 0; k < 6; ++k)
        c.write(0x000, (u32(cdb[2 * k]) << 8) | cdb[2 * k + 1], 2);
}

std::vector<u8> drainData(AtaCell& c, u32 n)
{
    std::vector<u8> v;
    for (u32 k = 0; k < n; ++k)
        v.push_back(static_cast<u8>(c.read(0x000, 1)));
    return v;
}

} // namespace

TEST_CASE("plain 2048 image: identity mapping, exact snap size")
{
    const char* p = "opm_cdimg_plain.iso";
    std::vector<u8> img;
    for (u64 n = 0; n < 5; ++n)
        for (u32 j = 0; j < 2048; ++j)
            img.push_back(pay(n, j));
    img.resize(img.size() + 700, 0x77); // a straggler tail, not a sector
    REQUIRE(writeAll(p, img));

    CdImage cd;
    REQUIRE(cd.open(p));
    CHECK(std::string(cd.shape()) == "2048");
    CHECK(cd.blocks() == 5);
    // Snapshots recorded the EXACT file size long before other formats
    // existed; a rounded value would refuse every one of them.
    CHECK(cd.snapBytes() == img.size());
    REQUIRE(cd.tracks().size() == 1);
    CHECK(!cd.tracks()[0].audio);

    u8 buf[2048];
    REQUIRE(cd.readBlock(3, buf));
    for (u32 j = 0; j < 2048; ++j)
        REQUIRE(buf[j] == pay(3, j));
    REQUIRE(cd.readBlock(9, buf)); // runout
    for (u32 j = 0; j < 2048; ++j)
        REQUIRE(buf[j] == 0);
    remove(p);
}

TEST_CASE("raw 2352 image: recognised by sync mark whatever the name")
{
    std::vector<u8> img;
    for (u64 n = 0; n < 6; ++n)
        rawSector(img, n, 1);
    for (const char* p : {"opm_cdimg_raw.bin", "opm_cdimg_raw.cdr"}) {
        REQUIRE(writeAll(p, img));
        CdImage cd;
        REQUIRE(cd.open(p));
        CHECK(std::string(cd.shape()) == "raw");
        CHECK(cd.blocks() == 6);
        u8 buf[2048];
        for (u64 n = 0; n < 6; ++n) {
            REQUIRE(cd.readBlock(n, buf));
            for (u32 j = 0; j < 2048; j += 97)
                REQUIRE(buf[j] == pay(n, j));
        }
        remove(p);
    }
}

TEST_CASE("raw 2352 mode 2: user data behind the subheader")
{
    const char* p = "opm_cdimg_m2.bin";
    std::vector<u8> img;
    for (u64 n = 0; n < 3; ++n)
        rawSector(img, n, 2);
    REQUIRE(writeAll(p, img));
    CdImage cd;
    REQUIRE(cd.open(p));
    CHECK(cd.blocks() == 3);
    u8 buf[2048];
    REQUIRE(cd.readBlock(1, buf));
    for (u32 j = 0; j < 2048; j += 61)
        REQUIRE(buf[j] == pay(1, j));
    remove(p);
}

TEST_CASE("cue+bin, one data track: the Warcraft shape")
{
    const char* cue = "opm_cdimg_wc3.cue";
    const char* bin = "opm_cdimg_wc3.bin";
    std::vector<u8> img;
    for (u64 n = 0; n < 7; ++n)
        rawSector(img, n, 1);
    REQUIRE(writeAll(bin, img));
    REQUIRE(writeText(cue, "FILE \"opm_cdimg_wc3.bin\" BINARY\n"
                           "  TRACK 01 MODE1/2352\n"
                           "    INDEX 01 00:00:00\n"));
    CdImage cd;
    REQUIRE(cd.open(cue));
    CHECK(std::string(cd.shape()) == "cue");
    CHECK(cd.blocks() == 7);
    REQUIRE(cd.tracks().size() == 1);
    CHECK(cd.tracks()[0].startLba == 0);
    u8 buf[2048];
    for (u64 n = 0; n < 7; ++n) {
        REQUIRE(cd.readBlock(n, buf));
        for (u32 j = 0; j < 2048; j += 89)
            REQUIRE(buf[j] == pay(n, j));
    }
    remove(cue);
    remove(bin);
}

TEST_CASE("cue mixed mode: audio refuses, stored pregap reads as zeros")
{
    const char* cue = "opm_cdimg_mix.cue";
    const char* bin = "opm_cdimg_mix.bin";
    std::vector<u8> img;
    for (u64 n = 0; n < 8; ++n)
        rawSector(img, n, 1); // data track content
    for (u64 n = 8; n < 14; ++n) // 2 stored pregap + 4 audio sectors
        img.insert(img.end(), 2352, 0x55);
    REQUIRE(writeAll(bin, img));
    REQUIRE(writeText(cue, "FILE \"opm_cdimg_mix.bin\" BINARY\n"
                           "  TRACK 01 MODE1/2352\n"
                           "    INDEX 01 00:00:00\n"
                           "  TRACK 02 AUDIO\n"
                           "    INDEX 00 00:00:08\n"
                           "    INDEX 01 00:00:10\n"));
    CdImage cd;
    REQUIRE(cd.open(cue));
    REQUIRE(cd.tracks().size() == 2);
    CHECK(cd.tracks()[0].startLba == 0);
    CHECK(cd.tracks()[0].contentSectors == 8);
    CHECK(cd.tracks()[1].startLba == 10);
    CHECK(cd.tracks()[1].audio);
    CHECK(cd.blocks() == 14);

    u8 buf[2048];
    REQUIRE(cd.readBlock(3, buf));
    for (u32 j = 0; j < 2048; j += 89)
        REQUIRE(buf[j] == pay(3, j));
    // The audio track's stored pregap is not the data track's data.
    REQUIRE(cd.readBlock(8, buf));
    for (u32 j = 0; j < 2048; ++j)
        REQUIRE(buf[j] == 0);
    CHECK(!cd.audioAt(8));
    // Inside the audio track READ(10) must refuse.
    CHECK(cd.audioAt(11));
    CHECK(!cd.readBlock(11, buf));
    remove(cue);
    remove(bin);
}

TEST_CASE("cue PREGAP inserts disc sectors the file does not carry")
{
    const char* cue = "opm_cdimg_pg.cue";
    const char* bin = "opm_cdimg_pg.bin";
    std::vector<u8> img;
    for (u64 n = 0; n < 4; ++n)
        rawSector(img, n, 1);
    REQUIRE(writeAll(bin, img));
    REQUIRE(writeText(cue, "FILE \"opm_cdimg_pg.bin\" BINARY\n"
                           "  TRACK 01 MODE1/2352\n"
                           "    PREGAP 00:00:02\n"
                           "    INDEX 01 00:00:00\n"));
    CdImage cd;
    REQUIRE(cd.open(cue));
    REQUIRE(cd.tracks().size() == 1);
    CHECK(cd.tracks()[0].startLba == 2);
    CHECK(cd.blocks() == 6);
    u8 buf[2048];
    REQUIRE(cd.readBlock(0, buf)); // inside the virtual pregap
    for (u32 j = 0; j < 2048; ++j)
        REQUIRE(buf[j] == 0);
    REQUIRE(cd.readBlock(2, buf));
    for (u32 j = 0; j < 2048; j += 89)
        REQUIRE(buf[j] == pay(0, j));
    remove(cue);
    remove(bin);
}

TEST_CASE("cue refusals say what refused")
{
    const char* cue = "opm_cdimg_bad.cue";
    CdImage cd;

    REQUIRE(writeText(cue, "FILE \"opm_cdimg_missing.bin\" BINARY\n"
                           "  TRACK 01 MODE1/2352\n"
                           "    INDEX 01 00:00:00\n"));
    CHECK(!cd.open(cue));
    CHECK(cd.why().find("opm_cdimg_missing.bin") != std::string::npos);

    REQUIRE(writeText(cue, "FILE \"x.bin\" BINARY\n"
                           "  TRACK 01 MODE1/2352\n"));
    CHECK(!cd.open(cue));
    CHECK(cd.why().find("INDEX 01") != std::string::npos);

    REQUIRE(writeText(cue, "FILE \"x.wav\" WAVE\n"
                           "  TRACK 01 AUDIO\n"
                           "    INDEX 01 00:00:00\n"));
    CHECK(!cd.open(cue));
    CHECK(cd.why().find("BINARY") != std::string::npos);
    remove(cue);
}

TEST_CASE("UDIF dmg: zlib, raw, zero and hole chunks all land")
{
    const char* p = "opm_cdimg_test.dmg";
    // Device: 4 sectors of 512 — zlib text, raw pattern, zero chunk, hole.
    std::vector<u8> fork(kZlib512, kZlib512 + sizeof kZlib512);
    std::vector<u8> rawSec;
    for (u32 j = 0; j < 512; ++j)
        rawSec.push_back(static_cast<u8>(j * 5 + 9));
    fork.insert(fork.end(), rawSec.begin(), rawSec.end());

    std::vector<u8> mish(204 + 5 * 40, 0);
    be32at(mish, 0, 0x6D697368u); // 'mish'
    be32at(mish, 4, 1);
    be64at(mish, 8, 0);  // first sector of this table
    be64at(mish, 16, 3); // sectors the chunks below cover
    be64at(mish, 24, 0); // data offset
    be32at(mish, 200, 5);
    auto chunk = [&](u32 idx, u32 type, u64 sec, u64 cnt, u64 off,
                     u64 len) {
        const size_t at = 204 + idx * 40;
        be32at(mish, at, type);
        be64at(mish, at + 8, sec);
        be64at(mish, at + 16, cnt);
        be64at(mish, at + 24, off);
        be64at(mish, at + 32, len);
    };
    chunk(0, 0x7FFFFFFEu, 0, 0, 0, 0); // comment
    chunk(1, 0x80000005u, 0, 1, 0, sizeof kZlib512);
    chunk(2, 0x00000001u, 1, 1, sizeof kZlib512, 512);
    chunk(3, 0x00000000u, 2, 1, 0, 0);
    chunk(4, 0xFFFFFFFFu, 3, 0, 0, 0); // terminator

    const std::string xml =
        "<?xml version=\"1.0\"?><plist><dict><key>resource-fork</key>"
        "<dict><key>blkx</key><array><dict><key>Data</key><data>\n" +
        base64(mish) +
        "\n</data></dict></array></dict></dict></plist>";

    std::vector<u8> file(fork);
    file.insert(file.end(), xml.begin(), xml.end());
    std::vector<u8> koly(512, 0);
    memcpy(koly.data(), "koly", 4);
    be32at(koly, 4, 4);   // version
    be32at(koly, 8, 512); // header size
    be64at(koly, 24, 0);  // data fork offset
    be64at(koly, 32, fork.size());
    be64at(koly, 216, fork.size()); // XML offset
    be64at(koly, 224, xml.size());
    be64at(koly, 492, 4); // device sectors, hole included
    file.insert(file.end(), koly.begin(), koly.end());
    REQUIRE(writeAll(p, file));

    CdImage cd;
    REQUIRE_MESSAGE(cd.open(p), cd.why());
    CHECK(std::string(cd.shape()) == "udif");
    CHECK(cd.blocks() == 1);
    u8 buf[2048];
    REQUIRE(cd.readBlock(0, buf));
    const std::vector<u8> text = zlibPlain512();
    REQUIRE(memcmp(buf, text.data(), 512) == 0);
    REQUIRE(memcmp(buf + 512, rawSec.data(), 512) == 0);
    for (u32 j = 1024; j < 2048; ++j)
        REQUIRE(buf[j] == 0);
    remove(p);
}

TEST_CASE("bare HFS+ volume gets the virtual partition dress")
{
    // Disk Utility's "Master CD" images are bare HFS+ — no Driver
    // Descriptor Map, no partition map — and Mac OS 9's CD probe refuses
    // exactly that with the "unreadable … ProDOS 0K" dialog. The layer
    // dresses such a volume the way a pressed disc is dressed.
    const char* p = "opm_cdimg_bare.cdr";
    std::vector<u8> img(300 * 1024, 0); // 150 blocks = 600 512-sectors
    img[1024] = 0x48;                   // 'H+', version 4
    img[1025] = 0x2B;
    img[1027] = 0x04;
    for (u32 j = 0; j < 2048; ++j)
        img[2 * 2048 + j] = pay(2, j); // a marker block
    REQUIRE(writeAll(p, img));
    CdImage cd;
    REQUIRE(cd.open(p));
    CHECK(std::string(cd.shape()) == "2048+apm");
    CHECK(cd.blocks() == 16 + 150);
    // The identity a snapshot records is the FILE, not the dress.
    CHECK(cd.snapBytes() == img.size());
    u8 b[2048];
    REQUIRE(cd.readBlock(0, b));
    CHECK(b[0] == 'E');
    CHECK(b[1] == 'R');
    CHECK(((u32(b[2]) << 8) | b[3]) == 512);
    CHECK(((u32(b[4]) << 24) | (u32(b[5]) << 16) | (u32(b[6]) << 8) |
           b[7]) == 64 + 600);
    CHECK(b[512] == 'P');
    CHECK(b[513] == 'M');
    CHECK(memcmp(&b[512 + 48], "Apple_partition_map", 19) == 0);
    CHECK(memcmp(&b[1024 + 48], "Apple_HFS", 10) == 0);
    CHECK(((u32(b[1024 + 8]) << 24) | (u32(b[1024 + 9]) << 16) |
           (u32(b[1024 + 10]) << 8) | b[1024 + 11]) == 64);
    CHECK(((u32(b[1024 + 12]) << 24) | (u32(b[1024 + 13]) << 16) |
           (u32(b[1024 + 14]) << 8) | b[1024 + 15]) == 600);
    // The volume itself begins at LBA 16, byte-for-byte the file.
    REQUIRE(cd.readBlock(16, b));
    CHECK(b[1024] == 0x48);
    CHECK(b[1025] == 0x2B);
    REQUIRE(cd.readBlock(18, b));
    for (u32 j = 0; j < 2048; j += 97)
        REQUIRE(b[j] == pay(2, j));
    remove(p);
}

TEST_CASE("dressed and hybrid images keep their own structures")
{
    const char* p = "opm_cdimg_nodress.iso";
    // A real partition map at block 0: untouched.
    {
        std::vector<u8> img(64 * 1024, 0);
        img[0] = 'E';
        img[1] = 'R';
        img[1024] = 0x48;
        img[1025] = 0x2B;
        REQUIRE(writeAll(p, img));
        CdImage cd;
        REQUIRE(cd.open(p));
        CHECK(std::string(cd.shape()) == "2048");
        CHECK(cd.blocks() == 32);
    }
    // An ISO/HFS hybrid (PVD at 32768): mounts by its own structures.
    {
        std::vector<u8> img(40 * 2048, 0);
        img[1024] = 0x48;
        img[1025] = 0x2B;
        img[32768] = 0x01;
        memcpy(&img[32769], "CD001", 5);
        REQUIRE(writeAll(p, img));
        CdImage cd;
        REQUIRE(cd.open(p));
        CHECK(std::string(cd.shape()) == "2048");
        CHECK(cd.blocks() == 40);
    }
    remove(p);
}

TEST_CASE("ATAPI protocol over a mixed-mode cue: TOC, reads, refusal")
{
    const char* cue = "opm_cdimg_ata.cue";
    const char* bin = "opm_cdimg_ata.bin";
    std::vector<u8> img;
    for (u64 n = 0; n < 8; ++n)
        rawSector(img, n, 1);
    for (u64 n = 8; n < 14; ++n)
        img.insert(img.end(), 2352, 0x55);
    REQUIRE(writeAll(bin, img));
    REQUIRE(writeText(cue, "FILE \"opm_cdimg_ata.bin\" BINARY\n"
                           "  TRACK 01 MODE1/2352\n"
                           "    INDEX 01 00:00:00\n"
                           "  TRACK 02 AUDIO\n"
                           "    INDEX 00 00:00:08\n"
                           "    INDEX 01 00:00:10\n"));
    AtaCell cd;
    REQUIRE(cd.attachIso(cue));

    // READ TOC, LBA form: both tracks and the lead-out, real addresses.
    {
        const u8 cdb[12] = {0x43, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0};
        sendPacket(cd, cdb);
        REQUIRE((cd.read(0x070, 1) & 0x08) != 0); // DRQ: data waiting
        const u32 n = (cd.read(0x050, 1) << 8) | cd.read(0x040, 1);
        REQUIRE(n == 4 + 3 * 8);
        const std::vector<u8> toc = drainData(cd, n);
        CHECK(toc[2] == 1);
        CHECK(toc[3] == 2);
        CHECK(toc[5] == 0x14); // data track
        CHECK(toc[6] == 1);
        CHECK(toc[11] == 0);   // track 1 at LBA 0
        CHECK(toc[13] == 0x10); // audio track
        CHECK(toc[14] == 2);
        CHECK(toc[19] == 10);  // track 2 at LBA 10
        CHECK(toc[22] == 0xAA);
        CHECK(toc[27] == 14);  // lead-out
    }
    // READ TOC, MSF form: track 2 at 00:02:10 (LBA 10 + the 150 offset).
    {
        const u8 cdb[12] = {0x43, 2, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0};
        sendPacket(cd, cdb);
        const u32 n = (cd.read(0x050, 1) << 8) | cd.read(0x040, 1);
        const std::vector<u8> toc = drainData(cd, n);
        CHECK(toc[17] == 0);
        CHECK(toc[18] == 2);
        CHECK(toc[19] == 10);
    }
    // READ CAPACITY: last block 13, 2048-byte blocks.
    {
        const u8 cdb[12] = {0x25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        sendPacket(cd, cdb);
        const std::vector<u8> cap = drainData(cd, 8);
        CHECK(cap[3] == 13);
        CHECK(cap[6] == 0x08);
    }
    // READ(10) of data delivers the payload.
    {
        const u8 cdb[12] = {0x28, 0, 0, 0, 0, 2, 0, 0, 1, 0, 0, 0};
        sendPacket(cd, cdb);
        const std::vector<u8> d = drainData(cd, 2048);
        for (u32 j = 0; j < 2048; j += 89)
            REQUIRE(d[j] == pay(2, j));
    }
    // READ(10) spanning data into audio: the data sectors arrive, then the
    // command dies with CHECK CONDITION instead of serving silence.
    {
        const u8 cdb[12] = {0x28, 0, 0, 0, 0, 6, 0, 0, 6, 0, 0, 0};
        sendPacket(cd, cdb);
        const u32 n = (cd.read(0x050, 1) << 8) | cd.read(0x040, 1);
        REQUIRE(n == 4 * 2048); // clamped at the audio boundary
        const std::vector<u8> d = drainData(cd, n);
        for (u32 j = 0; j < 2048; j += 97)
            REQUIRE(d[j] == pay(6, j));
        for (u32 j = 0; j < 2048; ++j)
            REQUIRE(d[2 * 2048 + j] == 0); // the stored pregap
        REQUIRE((cd.read(0x070, 1) & 0x01) != 0); // ERR
        const u8 sense[12] = {0x03, 0, 0, 0, 18, 0, 0, 0, 0, 0, 0, 0};
        sendPacket(cd, sense);
        const std::vector<u8> s = drainData(cd, 18);
        CHECK(s[0] == 0x70);
        CHECK(s[2] == 0x05); // ILLEGAL REQUEST
    }
    // READ(10) straight into audio refuses immediately.
    {
        const u8 cdb[12] = {0x28, 0, 0, 0, 0, 11, 0, 0, 1, 0, 0, 0};
        sendPacket(cd, cdb);
        REQUIRE((cd.read(0x070, 1) & 0x01) != 0);
        REQUIRE((cd.read(0x070, 1) & 0x08) == 0); // and no data phase
    }
    remove(cue);
    remove(bin);
}
