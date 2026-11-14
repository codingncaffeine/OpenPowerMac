// The Rage 128's two register apertures, as the bus and the chip report them.
//
// RAGE 128 PRO Register Reference Guide 2.2: the chip has two identical copies
// of its memory-mapped register aperture so that a PowerMac driver can mark
// one of them cacheable. Aperture 0 is at REG_BASE (PCI config 0x18, a 16 KB
// window); aperture 1 is at CONFIG_REG_1_BASE (MMR 0x10C), a 31:13 field
// whose lowest bit is hardwired to one; CONFIG_REG_APER_SIZE (MMR 0x110) is
// the size of EACH aperture, 0x2000.
//
// A driver that follows the guide caches [CONFIG_REG_1_BASE, +SIZE). If those
// two registers describe the whole window instead of its upper half, the
// driver caches aperture 0 as well and none of its register stores reach the
// chip — so the values are pinned here, together with the aliasing that makes
// the upper half usable at all.

#include "doctest.h"
#include "opm/sawtooth.hpp"

#include <memory>
#include <vector>

using namespace opm;

namespace {

u32 bswap(u32 v)
{
    return (v >> 24) | ((v >> 8) & 0xFF00u) | ((v << 8) & 0xFF0000u) |
           (v << 24);
}

// One PCI config write to the card (AGP bus, the slot the Rage sits in). The
// latch and the data window are little-endian devices.
void cardCfgWrite(SawtoothBus& bus, u32 reg, u32 v)
{
    bus.write32(0xF0800000u, bswap(0x00010000u | reg));
    bus.write32(0xF0C00000u, bswap(v));
}
u32 cardCfgRead(SawtoothBus& bus, u32 reg)
{
    bus.write32(0xF0800000u, bswap(0x00010000u | reg));
    return bswap(bus.read32(0xF0C00000u));
}

std::unique_ptr<SawtoothBus> makeBus()
{
    std::vector<u8> rom(SawtoothBus::kRomSize, 0);
    return std::make_unique<SawtoothBus>(8u << 20, rom);
}

constexpr u32 kReg = 0x90000000u; // where Open Firmware puts the window

} // namespace

TEST_CASE("r128: CONFIG_REG_1_BASE is aperture 1, the upper half of the window")
{
    auto bus = makeBus();
    cardCfgWrite(*bus, 0x18, kReg);
    // Registers are little-endian on the bus.
    CHECK(bswap(bus->read32(kReg + 0x010Cu)) == kReg + 0x2000u);
    CHECK(bswap(bus->read32(kReg + 0x0110u)) == 0x00002000u);
    // The same two answers through aperture 1 itself.
    CHECK(bswap(bus->read32(kReg + 0x2000u + 0x010Cu)) == kReg + 0x2000u);
    CHECK(bswap(bus->read32(kReg + 0x2000u + 0x0110u)) == 0x00002000u);
}

TEST_CASE("r128: the two register apertures are the same registers")
{
    auto bus = makeBus();
    cardCfgWrite(*bus, 0x18, kReg);
    // BIOS_0_SCRATCH (0x0010) is plain storage. Written through one
    // aperture, read through the other, both ways, with DIFFERENT values so
    // a pair of independent cells cannot pass.
    bus->write32(kReg + 0x0010u, 0x11223344u);
    CHECK(bus->read32(kReg + 0x2000u + 0x0010u) == 0x11223344u);
    bus->write32(kReg + 0x2000u + 0x0010u, 0xA5A55A5Au);
    CHECK(bus->read32(kReg + 0x0010u) == 0xA5A55A5Au);
}

TEST_CASE("r128: the framebuffer BAR reads back prefetchable, the register BAR not")
{
    // MEM_BASE CFG:0x10 has PREFETCH_EN (bit 3, read-only, 1) and decodes
    // 64 MB; REG_BASE CFG:0x18 has no such bit and decodes 16 KB. Sizing is
    // the all-ones write every PCI probe performs.
    auto bus = makeBus();
    CHECK((cardCfgRead(*bus, 0x10) & 0xFu) == 0x8u); // before any write
    cardCfgWrite(*bus, 0x10, 0xFFFFFFFFu);
    CHECK(cardCfgRead(*bus, 0x10) == 0xFC000008u);
    cardCfgWrite(*bus, 0x10, 0x94000000u);
    CHECK(cardCfgRead(*bus, 0x10) == 0x94000008u);
    CHECK(bus->atiFbBar() == 0x94000000u);
    cardCfgWrite(*bus, 0x18, 0xFFFFFFFFu);
    CHECK(cardCfgRead(*bus, 0x18) == 0xFFFFC000u);
}
