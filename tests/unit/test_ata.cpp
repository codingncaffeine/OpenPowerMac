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

// Drain 256 words of an IDENTIFY response the way a driver does: one 16-bit
// read of the data register at a time.
std::vector<u16> readIdentify(AtaCell& c)
{
    std::vector<u16> w;
    for (u32 k = 0; k < 256; ++k)
        w.push_back(static_cast<u16>(c.read(0x000, 2)));
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
