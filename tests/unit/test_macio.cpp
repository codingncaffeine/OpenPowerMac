// Heathrow interrupt controller: two {event, enable, ack, level} blocks of
// little-endian 32-bit registers (+0x20 = IRQs 0-31, +0x10 = IRQs 32-63),
// level-holding sources re-latching after ack. The VIA's IRQ output feeds
// source 18; delivery to the CPU line is event & enable.

#include "doctest.h"
#include "opm/macio.hpp"

using namespace opm;

namespace {

u32 rd32(MacIo& m, u32 off)
{
    return u32(m.read8(off)) | (u32(m.read8(off + 1)) << 8) |
           (u32(m.read8(off + 2)) << 16) | (u32(m.read8(off + 3)) << 24);
}
void wr32(MacIo& m, u32 off, u32 v)
{
    m.write8(off, static_cast<u8>(v));
    m.write8(off + 1, static_cast<u8>(v >> 8));
    m.write8(off + 2, static_cast<u8>(v >> 16));
    m.write8(off + 3, static_cast<u8>(v >> 24));
}

// Raise the VIA's IRQ output (IFR & IER nonzero): drives source 18.
void raiseVia(MacIo& m)
{
    m.write8(0x16000 + (2 << 9), 0x30);  // DDRB: TIP/TACK are outputs
    m.write8(0x16000 + (14 << 9), 0x84); // IER: set SR enable
    m.write8(0x16000 + (11 << 9), 0x1C); // ACR: shift-out mode
    m.write8(0x16000 + (10 << 9), 0x01); // SR: stage a byte
    m.write8(0x16000 + (0 << 9), 0x18);  // ORB: TIP asserts, clocks it
}

// Drop the level again (reading the SR clears the flag).
void dropVia(MacIo& m)
{
    (void)m.read8(0x16000 + (10 << 9));
}

} // namespace

TEST_CASE("heathrow pic: latch, enable gating, ack, level re-latch")
{
    MacIo m;
    CHECK_FALSE(m.irqAsserted());

    raiseVia(m);
    // Event latched in block 0 bit 18; nothing delivered while masked.
    CHECK((rd32(m, 0x20) & (1u << MacIo::kIrqVia)) != 0);
    CHECK((rd32(m, 0x2C) & (1u << MacIo::kIrqVia)) != 0); // live level
    CHECK_FALSE(m.irqAsserted());

    wr32(m, 0x24, 1u << MacIo::kIrqVia); // enable source 18
    CHECK(rd32(m, 0x24) == (1u << MacIo::kIrqVia));
    CHECK(m.irqAsserted());

    // Ack while the VIA still holds its line: the event re-latches.
    wr32(m, 0x28, 1u << MacIo::kIrqVia);
    CHECK((rd32(m, 0x20) & (1u << MacIo::kIrqVia)) != 0);
    CHECK(m.irqAsserted());

    // Drop the level, ack again: stays clear.
    dropVia(m);
    CHECK((rd32(m, 0x2C) & (1u << MacIo::kIrqVia)) == 0);
    wr32(m, 0x28, 1u << MacIo::kIrqVia);
    CHECK((rd32(m, 0x20) & (1u << MacIo::kIrqVia)) == 0);
    CHECK_FALSE(m.irqAsserted());
}

TEST_CASE("heathrow pic: byte lanes and the high block")
{
    MacIo m;
    // Enable register accepts per-byte little-endian writes.
    m.write8(0x24 + 2, 0x04); // bit 18 lives in byte lane 2
    CHECK(rd32(m, 0x24) == 0x00040000u);
    // The +0x10 block is a separate register set (IRQs 32-63).
    wr32(m, 0x14, 0xA5A50FF0u);
    CHECK(rd32(m, 0x14) == 0xA5A50FF0u);
    CHECK(rd32(m, 0x10) == 0);  // no high-block events modeled yet
    CHECK(rd32(m, 0x18) == 0);  // ack reads as zero
}
