// The ATA cell's PIO data path, and in particular the BYTE ORDER of an
// IDENTIFY response.
//
// This deserves a test rather than a boot because the whole mount dig has
// been conducted against identity data nobody ever read back. The on-disc
// Apple CD driver whitelists drive models by string; a previous session
// changed our model to a real "MATSHITA CD-ROM CR-175" and the boot did not
// change, which was recorded as "the whitelist is not the cause". That
// conclusion is only sound if the string the GUEST sees is the string we
// think we wrote — and a byte-swapped identity fails a whitelist no matter
// what text is in it, so the experiment would have looked identical either
// way.
//
// ATA words carry the first character in bits 15:8. A 16-bit read of the
// data register must therefore return 'M' in the high byte of word 27.

#include "doctest.h"
#include "opm/ata.hpp"

#include <cstdio>
#include <string>
#include <vector>

using namespace opm;

namespace {

const char* kIso = "opm_ata_test.iso";

bool makeIso()
{
    FILE* f = fopen(kIso, "wb");
    if (!f)
        return false;
    std::vector<u8> buf(64u * 1024u);
    for (size_t k = 0; k < buf.size(); ++k)
        buf[k] = static_cast<u8>(k * 13u + 5u);
    const bool ok = fwrite(buf.data(), 1, buf.size(), f) == buf.size();
    fclose(f);
    return ok;
}

// Drain 256 words of an IDENTIFY the way a driver does — one 16-bit read of
// the data register at a time — and apply the transport's byte swap.
//
// The mac-io ATA data register is byte-reversed on the 60x side, so the Mac
// driver reads it with a byte-swapping load and the buffer it builds in
// memory is the little-endian image. Measured directly with g4run's --peek
// on the ATA Manager's identify buffer: the guest's copy of a big-endian
// buffer came out reversed throughout — geometry 16383/16/63 arrived as
// ff3f/1000/3f00 and the serial "OPM00000001" as "PO0M000000 1", and
// .ATALoad then divided by a zero field and sad-macked. This models the
// same path, so the test asserts what the GUEST sees rather than what the
// cell happens to return.
u16 hostSwap(u16 v) { return static_cast<u16>((v >> 8) | (v << 8)); }

std::vector<u16> readIdentify(AtaCell& c)
{
    std::vector<u16> w;
    for (u32 k = 0; k < 256; ++k)
        w.push_back(hostSwap(static_cast<u16>(c.read(0x000, 2))));
    return w;
}

// An ATA string field: first character in bits 15:8 of the first word.
std::string ataText(const std::vector<u16>& w, u32 first, u32 words)
{
    std::string s;
    for (u32 k = 0; k < words && first + k < w.size(); ++k) {
        s.push_back(static_cast<char>(w[first + k] >> 8));
        s.push_back(static_cast<char>(w[first + k] & 0xFF));
    }
    while (!s.empty() && s.back() == ' ')
        s.pop_back();
    return s;
}

} // namespace

TEST_CASE("ATAPI IDENTIFY reads back as the drive we claim to be")
{
    REQUIRE(makeIso());
    AtaCell cd;
    REQUIRE(cd.attachIso(kIso));

    cd.write(0x060, 0x00, 1); // select device 0
    cd.write(0x070, 0xA1, 1); // IDENTIFY PACKET DEVICE
    const std::vector<u16> w = readIdentify(cd);

    // Word 0: ATAPI, CD-ROM, removable, 12-byte CDB.
    CHECK(w[0] == 0x8580);

    // The model string is what the on-disc Apple CD driver matches against.
    const std::string model = ataText(w, 27, 20);
    INFO("model as the guest reads it: [" << model << "]");
    CHECK(model == "MATSHITA CD-ROM CR-175");

    const std::string firmware = ataText(w, 23, 4);
    INFO("firmware: [" << firmware << "]");
    CHECK(firmware == "7T02");

    remove(kIso);
}

TEST_CASE("ATA disk IDENTIFY reports a capacity the host can read")
{
    REQUIRE(makeIso());
    AtaCell hd;
    REQUIRE(hd.attachDisk(kIso));

    hd.write(0x060, 0x00, 1);
    hd.write(0x070, 0xEC, 1); // IDENTIFY DEVICE
    const std::vector<u16> w = readIdentify(hd);

    CHECK(w[0] == 0x0040); // ATA device, non-removable
    CHECK((w[49] & 0x0200) != 0); // LBA supported

    // Words 60-61 are the LBA sector count, low word first. A 64 KiB image
    // is 128 sectors; reading them in the wrong order gives 0x00800000.
    const u32 lba = (u32(w[61]) << 16) | w[60];
    INFO("LBA capacity words: 60=" << w[60] << " 61=" << w[61]);
    CHECK(lba == 128u);

    remove(kIso);
}

// CHS addressing, which is how Mac OS's ATA disk driver talks to a disk.
//
// Bit 6 of the device register picks the mode and BOTH are live: Open
// Firmware sets it (dev = 0xE0) and addresses by LBA, Mac OS clears it
// (dev = 0xA0), programs the translation with INITIALIZE DEVICE PARAMETERS
// and addresses by cylinder/head/sector. Reading the task file as LBA in
// both cases is an off-by-one at the sector that decides everything: the
// driver's first read is CHS 0/0/1, which is LBA 0 — the Driver Descriptor
// Record — and it was served block 1 instead. No 'ER' signature, no driver,
// no disk, for the whole of the boot.
namespace {

const char* kChsIso = "opm_ata_chs.iso";

// 256 sectors whose FIRST BYTE is the sector's own LBA. The other test
// image's pattern repeats every 256 bytes, so every sector of it begins
// with the same byte and could not tell one sector from another.
bool makeSectorIso()
{
    FILE* f = fopen(kChsIso, "wb");
    if (!f)
        return false;
    std::vector<u8> buf(256u * 512u, 0);
    for (u32 s = 0; s < 256u; ++s) {
        buf[s * 512u] = static_cast<u8>(s);
        buf[s * 512u + 1u] = static_cast<u8>(~s);
    }
    const bool ok = fwrite(buf.data(), 1, buf.size(), f) == buf.size();
    fclose(f);
    return ok;
}

// Issue a one-sector read and return the sector the drive actually served.
u32 sectorRead(AtaCell& hd, u8 dev, u8 cylHi, u8 cylLo, u8 sec)
{
    hd.write(0x060, dev, 1);
    hd.write(0x020, 0x01, 1); // one sector
    hd.write(0x030, sec, 1);
    hd.write(0x040, cylLo, 1);
    hd.write(0x050, cylHi, 1);
    hd.write(0x070, 0x20, 1); // READ SECTOR(S)
    const u32 first = hd.read(0x000, 1);
    const u32 second = hd.read(0x000, 1);
    CHECK(second == ((~first) & 0xFFu)); // it really is a sector header
    return first;
}

} // namespace

TEST_CASE("an ATA disk addresses in CHS when the device register says so")
{
    REQUIRE(makeSectorIso());
    AtaCell hd;
    REQUIRE(hd.attachDisk(kChsIso));
    // No stamp, so commands run as they are written: the deferred-command
    // window is a separate property with its own test below.

    // The power-on translation is the one IDENTIFY advertises — 16 heads,
    // 63 sectors per track — so a driver that reads words 3/6 and programs
    // exactly that changes nothing.
    CHECK(sectorRead(hd, 0xA0, 0, 0, 1) == 0u);  // C0/H0/S1 = LBA 0
    CHECK(sectorRead(hd, 0xA0, 0, 0, 2) == 1u);  // sector numbers are 1-based
    CHECK(sectorRead(hd, 0xA1, 0, 0, 1) == 63u); // head 1 = one track in

    // LBA mode is unaffected: the same task file means something else.
    CHECK(sectorRead(hd, 0xE0, 0, 0, 1) == 1u);
    CHECK(sectorRead(hd, 0xE0, 0, 0, 0) == 0u);

    // INITIALIZE DEVICE PARAMETERS re-programs it. Sector count is sectors
    // per track; the device register's low nibble is the MAXIMUM head
    // number, one less than the head count.
    hd.write(0x020, 32, 1);
    hd.write(0x060, 0xA3, 1); // 4 heads
    hd.write(0x070, 0x91, 1);
    CHECK(sectorRead(hd, 0xA0, 0, 0, 1) == 0u);
    CHECK(sectorRead(hd, 0xA1, 0, 0, 1) == 32u); // one track is now 32
    CHECK(sectorRead(hd, 0xA3, 0, 0, 1) == 96u); // the last of four heads
    CHECK(sectorRead(hd, 0xA0, 0, 1, 1) == 128u); // one cylinder = 4 x 32

    // …and IDENTIFY then reports the translation it agreed to.
    hd.write(0x060, 0xA0, 1);
    hd.write(0x070, 0xEC, 1);
    const std::vector<u16> w = readIdentify(hd);
    CHECK(w[55] == 4u);
    CHECK(w[56] == 32u);
    CHECK(w[3] == 16u); // the DEFAULT geometry is untouched
    CHECK(w[6] == 63u);

    remove(kChsIso);
}

// A command must not complete in zero time.
//
// Open Firmware's ATA driver writes the command register, then arms its
// interrupt path some hundreds of instructions later, then waits. A drive
// that finishes instantly has already asserted and lost its interrupt by the
// time the driver is listening — measured on the boot disk, eight 16 KiB
// READ MULTIPLEs succeeded and the ninth hung forever with DRQ asserted and
// the driver polling. The device must hold BSY across the arming window.
TEST_CASE("an ATA command asserts BSY and completes only after its delay")
{
    REQUIRE(makeIso());
    AtaCell hd;
    REQUIRE(hd.attachDisk(kIso));

    u64 clock = 0;
    hd.stamp = &clock;
    hd.cmdDelay_ = 100;

    hd.write(0x060, 0xE0, 1); // LBA mode, device 0
    hd.write(0x020, 0x01, 1); // one sector
    hd.write(0x030, 0x00, 1);
    hd.write(0x040, 0x00, 1);
    hd.write(0x050, 0x00, 1);
    hd.write(0x070, 0x20, 1); // READ SECTOR(S)

    // Immediately after the write the drive is busy, not ready, not DRQ.
    CHECK((hd.read(0x160, 1) & 0x80u) != 0); // BSY via alt-status
    CHECK((hd.read(0x160, 1) & 0x08u) == 0); // no DRQ yet
    CHECK(!hd.irqLine());

    for (clock = 1; clock < 100; ++clock)
        hd.tick();
    CHECK((hd.read(0x160, 1) & 0x80u) != 0); // still busy one tick short

    clock = 100;
    hd.tick();
    CHECK((hd.read(0x160, 1) & 0x80u) == 0);
    CHECK((hd.read(0x160, 1) & 0x08u) != 0); // DRQ: data is ready
    CHECK(hd.irqLine());                     // and INTRQ is asserted

    remove(kIso);
}
