// The boot ROM's system-configuration block (fff03f00). A dump read from a
// Power Mac G4 (AGP Graphics) carries that board's block and must be left
// exactly as it is — snapshots digest the ROM as loaded, and an untouched
// block is what keeps every existing snapshot loadable. A dump assembled
// from Apple's firmware-update package carries a template describing some
// other board (I2S audio, a 66 MHz bus); HWInit then polls mac-io +0x10000
// forever. That block is rewritten as a Sawtooth's, with the dump's own ROM
// version and date kept and the Adler-32 trailer recomputed.

#include "doctest.h"
#include "opm/sawtooth.hpp"

#include <string>
#include <vector>

using namespace opm;

namespace {

constexpr u32 kSys = SawtoothBus::kSysBlock;

std::vector<u8> blankRom()
{
    return std::vector<u8>(SawtoothBus::kRomSize, 0xFF);
}

// The block a 4.2.8f1 dump read from a real Sawtooth carries (the two such
// dumps on the shelf agree byte for byte).
void putSawtoothBlock(std::vector<u8>& rom)
{
    static const u8 k[0x80] = {
        0xc9, 0x9c, 0x20, 0xc1, 0x00, 0x04, 0x28, 0xf1,
        0x20, 0x01, 0x10, 0x11, 0x01, 0xda, 0x46, 0xa0,
        0x00, 0x00, 0x00, 0x01, 0x05, 0xf0, 0x3e, 0x4d,
        0x03, 0xf9, 0x40, 0xaa, 0x03, 0xf9, 0x40, 0xaa,
        0x02, 0xee, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff,
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
        0xff, 0xff, 0xff, 0xff, 0x15, 0x3b, 0x2f, 0x80,
    };
    for (u32 i = 0; i < 0x80; ++i)
        rom[kSys + i] = k[i];
}

// The head of the template block the 4.2.8f1 firmware-update image carries:
// audio type 0x20 (I2S), a 66.58 MHz bus, the L2 probe disabled by bit 4 of
// +0e. The tail is left as the updater left it — not a Sawtooth's either.
void putUpdaterTemplate(std::vector<u8>& rom)
{
    static const u8 head[0x28] = {
        0xc9, 0x9c, 0x10, 0x81, 0x00, 0x04, 0x28, 0xf1,
        0x20, 0x01, 0x10, 0x11, 0x01, 0x00, 0x30, 0xc0,
        0x00, 0x00, 0x00, 0x00, 0x03, 0xf7, 0xec, 0x51,
        0x03, 0xf7, 0xec, 0x51, 0x01, 0xfb, 0xf6, 0x29,
        0x03, 0xf7, 0xec, 0x51, 0xc9, 0x2c, 0x08, 0x01,
    };
    for (u32 i = 0; i < 0x80; ++i)
        rom[kSys + i] = 0x00;
    for (u32 i = 0; i < 0x28; ++i)
        rom[kSys + i] = head[i];
    rom[kSys + 0x7c] = 0x30; // its own trailer, valid for ITS bytes
    rom[kSys + 0x7d] = 0xe7;
    rom[kSys + 0x7e] = 0x34;
    rom[kSys + 0x7f] = 0x1d;
}

u32 be32(const std::vector<u8>& rom, u32 off)
{
    return (u32(rom[off]) << 24) | (u32(rom[off + 1]) << 16) |
           (u32(rom[off + 2]) << 8) | rom[off + 3];
}

} // namespace

TEST_CASE("sys block: the trailer is Adler-32 of the body")
{
    // Three real dumps, three trailers; the 4.2.8f1 one is the block above.
    std::vector<u8> rom = blankRom();
    putSawtoothBlock(rom);
    CHECK(SawtoothBus::sysBlockChecksum(rom.data() + kSys) == 0x153b2f80u);
    // The 3.2.4f1 ship ROM's block differs in version, date and +4d only.
    rom[kSys + 0x05] = 0x03; rom[kSys + 0x06] = 0x24;
    rom[kSys + 0x09] = 0x00; rom[kSys + 0x0a] = 0x02; rom[kSys + 0x0b] = 0x17;
    rom[kSys + 0x4d] = 0xff;
    CHECK(SawtoothBus::sysBlockChecksum(rom.data() + kSys) == 0x3db43071u);
}

TEST_CASE("sys block: a real Sawtooth's block is recognised and left alone")
{
    std::vector<u8> rom = blankRom();
    putSawtoothBlock(rom);
    const std::vector<u8> before = rom;
    CHECK(SawtoothBus::sysBlockIsSawtooth(rom));
    CHECK(SawtoothBus::factoryConfigure(rom).empty());
    CHECK(rom == before);

    // Through the bus: no note, and the same bytes on the wire.
    SawtoothBus bus(8u << 20, rom);
    CHECK(bus.romNote().empty());
    CHECK(bus.read32(SawtoothBus::kRomBase + kSys + 0x14) == 0x05f03e4du);
}

TEST_CASE("sys block: the updater's template is factory-configured")
{
    std::vector<u8> rom = blankRom();
    putUpdaterTemplate(rom);
    CHECK_FALSE(SawtoothBus::sysBlockIsSawtooth(rom));

    const std::string note = SawtoothBus::factoryConfigure(rom);
    CHECK_FALSE(note.empty());
    CHECK(note.find("I2S") != std::string::npos);
    CHECK(note.find("66.57 MHz") != std::string::npos);

    // Now a Sawtooth's: Screamer, 100 MHz, the L2 probe enabled again.
    CHECK(SawtoothBus::sysBlockIsSawtooth(rom));
    CHECK((rom[kSys + 0x0e] & 0xe0) == 0x40);
    CHECK((rom[kSys + 0x0e] & 0x10) == 0);
    CHECK(be32(rom, kSys + 0x14) == 0x05f03e4du);
    CHECK(be32(rom, kSys + 0x20) == 0x02ee0000u);
    // The dump's own version and date survive...
    CHECK(be32(rom, kSys + 0x04) == 0x000428f1u);
    CHECK(be32(rom, kSys + 0x08) == 0x20011011u);
    // ...and the trailer is right for the bytes now there, which for a
    // 4.2.8f1 dump is exactly the real machine's block.
    CHECK(be32(rom, kSys + 0x7c) == SawtoothBus::sysBlockChecksum(rom.data() + kSys));
    CHECK(be32(rom, kSys + 0x7c) == 0x153b2f80u);
    // Nothing outside the block moved: tst and NVRAM stay as the dump had them.
    for (u32 i = kSys + 0x80; i < kSys + 0x100; ++i)
        CHECK(rom[i] == 0xFF);

    // A second pass is a no-op.
    CHECK(SawtoothBus::factoryConfigure(rom).empty());
}

TEST_CASE("sys block: an erased block becomes a Sawtooth's with a valid trailer")
{
    std::vector<u8> rom = blankRom(); // the whole flash 0xFF, block included
    const std::string note = SawtoothBus::factoryConfigure(rom);
    CHECK_FALSE(note.empty());
    CHECK(note.find("no block") != std::string::npos);
    CHECK(SawtoothBus::sysBlockIsSawtooth(rom));
    // No plausible version/date to keep: the canonical ones are used.
    CHECK(be32(rom, kSys + 0x04) == 0x000428f1u);
    CHECK(be32(rom, kSys + 0x7c) == SawtoothBus::sysBlockChecksum(rom.data() + kSys));
}

TEST_CASE("sys block: the bus configures at construction and reports it")
{
    std::vector<u8> rom = blankRom();
    putUpdaterTemplate(rom);
    SawtoothBus bus(8u << 20, rom);
    CHECK_FALSE(bus.romNote().empty());
    CHECK(bus.read32(SawtoothBus::kRomBase + kSys + 0x14) == 0x05f03e4du);
    CHECK((bus.read8(SawtoothBus::kRomBase + kSys + 0x0e) & 0xe0) == 0x40);
}
