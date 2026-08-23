#include "opm/sawtooth.hpp"

#include <cstdio>

namespace opm {

// A 512 MB PC100 SDRAM DIMM, served over Keywest. Three slots are populated
// (see slavePresent) for 1536 MB — the most a Sawtooth accepts, the most Mac
// OS 9 can use, and the most this address map has room for: RAM then ends at
// 0x60000000, still clear of the 0x78000000 sizing window and the device
// region above it.
//
// The ROM sizes memory from the SPD GEOMETRY rather than by probing — the
// sizing-window probe census is zero across a whole boot. The two independent
// readings of these bytes agree, which matters because which one the ROM uses
// is not established, and a disagreement would surface as a plausible wrong
// total rather than as a failure:
//
//   geometry  2^13 rows x 2^10 cols x 4 banks x 8 bytes x 2 ranks = 512 MB
//   byte 31   density per rank in 4 MB units, 0x40 = 64 x 4 MB    = 256 MB,
//             and there are two ranks                             = 512 MB
//
// ⚠ THE ROM VALIDATES THIS AND REFUSES GEOMETRY IT DOES NOT LIKE. Encoding the
// same 512 MB as a ONE-rank module with 11 column bits (byte 4 = 0x0B,
// byte 31 = 0x80) fails memory init outright: the machine never reaches Open
// Firmware and blinks error code 1 instead. That blink lands by 17 M
// instructions, which makes it the cheapest test in the tree for this file —
// a wrong SPD is visible in seconds rather than after a boot. Two ranks of
// 13x10 is also how real PC100 512 MB modules are built. Checksum is computed
// at serve time.
static constexpr u8 kSpd[64] = {
    0x80, 0x08, 0x04, 0x0C, 0x09, 0x01, 0x40, 0x00,
    0x01, 0x0A, 0x07, 0x00, 0x80, 0x08, 0x00, 0x01,
    0x0F, 0x04, 0x0C, 0x01, 0x01, 0x00, 0x0E, 0x0A,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x00,
};

// The DIMMs are DERIVED FROM THE ALLOCATION, never fixed.
//
// The SPD used to be a compile-time constant while the RAM size was a user
// setting, so the two could disagree — and a guest told it has more memory
// than exists is a machine that dies somewhere far from the cause. Deriving
// both the slot count and each module's geometry from ram_.size() makes that
// disagreement unrepresentable, and makes memory a runtime choice.
//
// Only these four geometries are offered, because the ROM VALIDATES the SPD
// and refuses what it dislikes: a 512 MB module encoded as one rank of 13x11
// fails memory init outright and blinks error code 1 without ever reaching
// Open Firmware. Each row below is a shape real PC100 modules were built in.
SawtoothBus::Dimms SawtoothBus::dimms() const
{
    struct Shape { u32 mb, rows, cols, ranks, dens; };
    static constexpr Shape kShapes[] = {
        {  64, 12,  9, 1, 0x10 },
        { 128, 12, 10, 1, 0x20 },
        { 256, 13, 10, 1, 0x40 },
        { 512, 13, 10, 2, 0x40 }, // two ranks: 2 x 256 MB, as real ones are
    };
    const u32 totalMb = static_cast<u32>(ram_.size() >> 20);
    // Fewest modules that can carry it, largest module last: a Sawtooth has
    // four slots, and the address map has room for three 512s before RAM would
    // reach the 0x78000000 sizing window.
    for (u32 n = 1; n <= 3; ++n)
        for (const Shape& s : kShapes)
            if (s.mb * n == totalMb)
                return {n, s.mb, static_cast<u8>(s.rows),
                        static_cast<u8>(s.cols), static_cast<u8>(s.ranks),
                        static_cast<u8>(s.dens)};
    return {1, 64, 12, 9, 1, 0x10}; // unrepresentable size: one 64 MB module
}

u8 SawtoothBus::spdByte() const
{
    const Dimms d = dimms();
    u8 spd[64];
    for (u32 k = 0; k < 64; ++k)
        spd[k] = kSpd[k];
    spd[3] = d.rows;
    spd[4] = d.cols;
    spd[5] = d.ranks;
    spd[31] = d.density;

    const u8 idx = i2c_[0].sub & 63u;
    if (idx == 63) {
        u32 sum = 0;
        for (u32 k = 0; k < 63; ++k)
            sum += spd[k];
        return static_cast<u8>(sum);
    }
    return spd[idx];
}

// The processor module carries its own I2C descriptor at slave 0xAC, and the
// boot ROM reads the backside cache's geometry out of it.
//
// It first PROBES for the module, at fff85220: unless bit 4 of the config
// byte at 0xFFF03F0E says otherwise, it reads register 0x80 and requires the
// signature 0xC9. Only then does it clear cr3[EQ] and take the I2C route;
// on a NAK, or any other value, it falls back to a descriptor in the ROM's
// own config block — which in this ROM is empty ([0xFFF03F40] is 0), so the
// fallback finds nothing and reports no cache at all. (The config block
// opens with the bytes `c9 9c`: the signature, and 156, the register the
// descriptor starts at.)
//
// The descriptor itself is read at fff8551c, four registers:
//
//   156  must read 1, or the descriptor is not believed
//   157  log2 of the cache size, accepted only in 18..21 (256 KB .. 2 MB)
//   159  the backside clock ratio: low nibble 1 = whole steps, 2 = half
//        steps; high nibble the ratio in those units
//   160  timing: bits 4-5 L2OH, bit 6 write-through, low nibble the RAM type
//
// From those the ROM builds an L2CR by hand (fff85264): L2SIZ out of the
// table at fff85490 — 01 02 03 00 for 256K/512K/1M/2M, which is the MPC7400
// encoding in um7400 Table 3-12 — and L2CLK out of the table at fff85484
// indexed by ratio×2, where 4 selects ÷2. It writes L2CR, runs the
// invalidate, and fff854a4 then sets L2E.
//
// With nothing at 0xAC that read NAKs at the address phase, the ROM bails,
// and the machine ends up describing no L2 at all: the info block it hands
// Open Firmware keeps memory-test junk where the cache code belongs, OF's
// `l2-cache` word CASEs on it, matches none of 3/5/6/7/8, and builds no
// l2-cache node. Mac OS then finds no cache described anywhere and puts up
// "the built-in memory test has detected a problem with cache memory".
//
// A Power Mac G4 AGP ships 1 MB of backside L2 running at half the core
// clock.
u8 SawtoothBus::cacheRomByte() const
{
    switch (i2c_[0].sub) {
    case 0x80: return 0xC9; // "there is a processor module here"
    case 156: return 1;     // the descriptor is present
    case 157: return 20;    // 2^20 = 1 MB
    case 159: return 0x42;  // half-step units, 4 of them = ÷2
    case 160: return 0x00;  // L2OH 0, copy-back, synchronous burst SRAM
    default: return 0xFF;
    }
}

// ---------------------------------------------------------------------------
// The system-configuration block — the board's factory data in the flash.
//
// New World boot ROMs keep 0x80 bytes at fff03f00 (the updater calls the
// section "sys") that HWInit reads to learn which board it is running on, and
// 0x80 more at fff03f80 ("tst": the unit's serial number and Ethernet
// address) ahead of the two NVRAM copies at fff04000/fff06000. A ROM READ
// FROM A MACHINE carries all three; a ROM ASSEMBLED FROM THE FIRMWARE UPDATER
// carries the updater's template "sys" block — which, in the 4.2.8f1
// package, describes a board with I2S audio and a 66 MHz bus — an empty "tst"
// and erased NVRAM. The template sent HWInit down its I2S path, where it
// waits on a status bit at mac-io +0x10000 that nothing here will ever raise:
// 999 million reads of one register, and a machine that "does not boot".
//
// The block as every real Sawtooth dump carries it (3.2.4f1 ship, 4.2.8f1 x2 —
// identical apart from the ROM version/date words, one byte at +4d and the
// checksum). Field meanings are what HWInit's reads of it show:
//
//   +00  c9 9c        magic
//   +02  20 c1        flags; HWInit keys one presence test off the top nibble
//   +04  00 04 28 f1  ROM version, BCD (4.2.8f1)   - kept from the dump
//   +08  20 01 10 11  ROM date, BCD (2001-10-11)   - kept from the dump
//   +0c  01 da 46 a0  board/product id; bits 7-5 of +0e select the AUDIO
//                     hardware: 0x40 = Screamer on DAVbus (mac-io +14000,
//                     the cell this machine has), 0x20 = I2S (+10000),
//                     0x00 = none, 0xe0 = decided by +12
//   +10  00 00 00 01
//   +14  05 f0 3e 4d  60x bus clock, Hz (99,499,597 ~ 100 MHz)
//   +18  03 f9 40 aa  66,666,666 Hz  } the 66 MHz side
//   +1c  03 f9 40 aa  66,666,666 Hz  }
//   +20  02 ee 00 00  49,152,000 Hz = 1024 x 48 kHz, the audio master clock
//   +25, +30, +40..+44, +48..+4d, +51, +52, +5c  read by HWInit; all zero
//   +7c  Adler-32 of +00..+7b, big-endian
static constexpr u8 kSawtoothSys[SawtoothBus::kSysBlockSize] = {
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

u32 SawtoothBus::sysBlockChecksum(const u8* block)
{
    u32 a = 1, b = 0;
    for (u32 k = 0; k < kSysBlockSize - 4; ++k) {
        a = (a + block[k]) % 65521u;
        b = (b + a) % 65521u;
    }
    return (b << 16) | a;
}

// "Describes this board" is judged on the fields HWInit branches on and this
// machine cannot satisfy any other way: the magic, the audio hardware and
// the bus clock. Anything else a real Sawtooth's block might vary in is left
// alone — the block is that machine's, and it boots.
bool SawtoothBus::sysBlockIsSawtooth(const std::vector<u8>& rom)
{
    if (rom.size() < kSysBlock + kSysBlockSize)
        return false;
    const u8* s = rom.data() + kSysBlock;
    if (s[0] != 0xc9 || s[1] != 0x9c)
        return false;
    if ((s[0x0e] & 0xe0) != 0x40)
        return false;
    const u32 bus = (u32(s[0x14]) << 24) | (u32(s[0x15]) << 16) |
                    (u32(s[0x16]) << 8) | s[0x17];
    return bus >= 90000000u && bus <= 110000000u;
}

std::string SawtoothBus::factoryConfigure(std::vector<u8>& rom)
{
    if (rom.size() < kSysBlock + kSysBlockSize || sysBlockIsSawtooth(rom))
        return "";
    u8* s = rom.data() + kSysBlock;
    // Say what was there before it goes.
    char was[160];
    const bool hadMagic = s[0] == 0xc9 && s[1] == 0x9c;
    const u32 bus = (u32(s[0x14]) << 24) | (u32(s[0x15]) << 16) |
                    (u32(s[0x16]) << 8) | s[0x17];
    const char* audio = "none";
    switch (s[0x0e] & 0xe0) {
    case 0x20: audio = "I2S at mac-io +10000"; break;
    case 0x40: audio = "Screamer on DAVbus"; break;
    case 0x00: audio = "none"; break;
    default: audio = "unspecified"; break;
    }
    if (hadMagic)
        snprintf(was, sizeof was,
                 "audio type %02x (%s), bus clock %u.%02u MHz",
                 s[0x0e] & 0xe0, audio, bus / 1000000u,
                 (bus / 10000u) % 100u);
    else
        snprintf(was, sizeof was, "no block (%02x %02x ...)", s[0], s[1]);

    // The dump's own ROM version and date stay; everything that describes
    // the board becomes a Sawtooth's, and the checksum is recomputed over
    // the result.
    u8 keep[8];
    for (u32 k = 0; k < 8; ++k)
        keep[k] = s[4 + k];
    for (u32 k = 0; k < kSysBlockSize; ++k)
        s[k] = kSawtoothSys[k];
    const bool versionPlausible = keep[0] == 0x00 && keep[4] == 0x20;
    if (versionPlausible)
        for (u32 k = 0; k < 8; ++k)
            s[4 + k] = keep[k];
    const u32 sum = sysBlockChecksum(s);
    s[0x7c] = static_cast<u8>(sum >> 24);
    s[0x7d] = static_cast<u8>(sum >> 16);
    s[0x7e] = static_cast<u8>(sum >> 8);
    s[0x7f] = static_cast<u8>(sum);

    std::string note = "boot ROM: the system-configuration block at fff03f00 "
                       "did not describe a Power Mac G4 (AGP Graphics) - ";
    note += was;
    note += " - so it was factory-configured as one (Screamer audio on "
            "DAVbus, 100 MHz bus). A ROM assembled from Apple's firmware "
            "updater carries a template here; one read from a machine "
            "carries that machine's.";
    return note;
}

} // namespace opm
