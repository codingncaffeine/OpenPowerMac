#include "opm/sawtooth.hpp"

namespace opm {

// One PC-SDRAM DIMM in slot 1, served over Keywest: the same JEDEC
// geometry the Gossamer SPD work pinned (12 rows, 9 columns, x64 —
// 64 MB), checksum computed at serve time.
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

u8 SawtoothBus::spdByte() const
{
    const u8 idx = i2c_[0].sub & 63u;
    if (idx == 63) {
        u32 sum = 0;
        for (u32 k = 0; k < 63; ++k)
            sum += kSpd[k];
        return static_cast<u8>(sum);
    }
    return kSpd[idx];
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
