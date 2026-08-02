// The Mac OS sound driver's pacing protocol, pinned: its interrupt handler
// ZEROES the codec's Frame Count register, arms a DBDMA program that
// LOAD_QUADs that register before each OUTPUT slice, and sizes its next mix
// by the count the engine deposited in the descriptor. Session 39 found both
// halves unimplemented — the write ignored, the op absorbed — which starved
// the mixer to one 512-frame slice per fallback kick in every Mac OS title
// while the ROM's polled chime stayed perfect.
#include "doctest.h"

#include "opm/sawtooth.hpp"

#include <vector>

using namespace opm;

namespace {

constexpr size_t kRam = 8u << 20;

std::vector<u8> quietRom()
{
    return std::vector<u8>(SawtoothBus::kRomSize, 0);
}

// Little-endian stores, the byte order DBDMA descriptors live in.
void putLe32(SawtoothBus& bus, u32 pa, u32 v)
{
    for (u32 k = 0; k < 4; ++k)
        bus.write8(pa + k, static_cast<u8>(v >> (8 * k)));
}

u32 ramLe32(const SawtoothBus& bus, u32 pa)
{
    const auto& r = bus.ram();
    return r[pa] | (r[pa + 1] << 8) | (r[pa + 2] << 16) |
           (static_cast<u32>(r[pa + 3]) << 24);
}

// Native value -> the byte-reversed image a little-endian register file
// carries on this big-endian bus (the guest's stwbrx/lwbrx pair).
u32 img(u32 native)
{
    return (native >> 24) | ((native >> 8) & 0xFF00u) |
           ((native << 8) & 0xFF0000u) | (native << 24);
}

constexpr u32 kSndCtrl = 0xF3008800u; // audio-out DBDMA channel
constexpr u32 kFrameCnt = 0xF3014050u;

} // namespace

TEST_CASE("awacs: frame count is writable and re-bases the timebase epoch")
{
    SawtoothBus bus(kRam, quietRom());

    bus.serviceDevices(1000000);
    // A counter nobody has zeroed keeps its boot epoch.
    const u32 before = img(bus.read32(kFrameCnt));
    CHECK(before == 1000000ull * 44100 / 25000000); // 1764

    // The driver's per-cycle zeroing must take, and count from "now".
    bus.write32(kFrameCnt, img(0));
    CHECK(img(bus.read32(kFrameCnt)) == 0);

    // 2.5 M timebase ticks at 44.1 kHz on a 25 MHz timebase = 4410 frames.
    bus.serviceDevices(1000000 + 2500000);
    CHECK(img(bus.read32(kFrameCnt)) == 4410);

    // A non-zero write is a base, not a reset.
    bus.write32(kFrameCnt, img(100));
    bus.serviceDevices(1000000 + 2500000 + 2500000);
    CHECK(img(bus.read32(kFrameCnt)) == 100 + 4410);
}

TEST_CASE("dbdma: LOAD_QUAD deposits the source quadlet in cmdDep")
{
    SawtoothBus bus(kRam, quietRom());
    bus.serviceDevices(1000000);
    bus.write32(kFrameCnt, img(0));
    bus.serviceDevices(1000000 + 2500000); // counter now reads 4410

    // The driver's own shape: LOAD_QUAD (key SYSTEM) of Frame Count, then
    // STOP. cmdDep starts as a sentinel the engine must overwrite.
    const u32 d = 0x00040000u;
    putLe32(bus, d + 0x0, 0x56000004u); // op 5, key 6, req 4
    putLe32(bus, d + 0x4, kFrameCnt);
    putLe32(bus, d + 0x8, 0xCAFEBABEu);
    putLe32(bus, d + 0xC, 0);
    putLe32(bus, d + 0x10, 0x70000000u); // STOP
    putLe32(bus, d + 0x14, 0);
    putLe32(bus, d + 0x18, 0);
    putLe32(bus, d + 0x1C, 0);

    bus.write32(kSndCtrl + 0x0C, img(d));          // commandPtrLo
    bus.write32(kSndCtrl + 0x00, img(0x80008000)); // RUN

    // The quadlet the driver reads back IS the frame count at execution.
    CHECK(ramLe32(bus, d + 0x8) == 4410);
    // xferStatus says the command retired.
    CHECK((ramLe32(bus, d + 0xC) >> 16) == 0x8400u);
    // STOP parked the channel with RUN still set.
    const u32 status = img(bus.read32(kSndCtrl + 0x00));
    CHECK((status & 0x8000u) != 0);
    CHECK((status & 0x0400u) == 0);
}

TEST_CASE("dbdma: LOAD_QUAD from RAM is a byte-exact copy")
{
    SawtoothBus bus(kRam, quietRom());

    const u32 src = 0x00050000u;
    bus.write8(src + 0, 0x11);
    bus.write8(src + 1, 0x22);
    bus.write8(src + 2, 0x33);
    bus.write8(src + 3, 0x44);

    const u32 d = 0x00041000u;
    putLe32(bus, d + 0x0, 0x56000004u);
    putLe32(bus, d + 0x4, src);
    putLe32(bus, d + 0x8, 0xCAFEBABEu);
    putLe32(bus, d + 0xC, 0);
    putLe32(bus, d + 0x10, 0x70000000u); // STOP
    putLe32(bus, d + 0x14, 0);
    putLe32(bus, d + 0x18, 0);
    putLe32(bus, d + 0x1C, 0);

    bus.write32(kSndCtrl + 0x0C, img(d));
    bus.write32(kSndCtrl + 0x00, img(0x80008000));

    const auto& r = bus.ram();
    CHECK(r[d + 0x8] == 0x11);
    CHECK(r[d + 0x9] == 0x22);
    CHECK(r[d + 0xA] == 0x33);
    CHECK(r[d + 0xB] == 0x44);
}
