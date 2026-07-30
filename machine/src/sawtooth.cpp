#include "opm/sawtooth.hpp"

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

} // namespace opm
