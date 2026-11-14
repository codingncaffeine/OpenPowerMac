// A system reset, as the PMU's reset command performs it: every chip back at
// power-on values, around the RAM, the flash and the discs that a reset does
// not touch (SawtoothBus::afterSystemReset).
//
// The failure this pins: a guest's Restart used to restart the processor
// only. Every device stayed as Mac OS had programmed it, Open Firmware's
// second pass took an external interrupt from one of them, and the machine
// ended at a firmware prompt instead of booting. So the cases here program
// devices the way a running OS leaves them and require the reset to undo it,
// while what must survive survives.

#include "doctest.h"
#include "opm/sawtooth.hpp"

#include <cstdio>
#include <memory>
#include <vector>

using namespace opm;

namespace {

constexpr size_t kRam = 8u << 20;
const char* kDisk = "opm_reset_hd.img";
const char* kDisc = "opm_reset_cd.iso";

u32 bswap(u32 v)
{
    return (v >> 24) | ((v >> 8) & 0xFF00u) | ((v << 8) & 0xFF0000u) |
           (v << 24);
}
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
bool makeFile(const char* path, u8 tag, size_t bytes)
{
    FILE* f = fopen(path, "wb");
    if (!f)
        return false;
    std::vector<u8> buf(bytes, tag);
    const bool ok = fwrite(buf.data(), 1, buf.size(), f) == buf.size();
    fclose(f);
    return ok;
}
std::unique_ptr<SawtoothBus> makeBus()
{
    std::vector<u8> rom(SawtoothBus::kRomSize, 0);
    return std::make_unique<SawtoothBus>(kRam, rom);
}

} // namespace

TEST_CASE("system reset: devices return to power-on values")
{
    auto bus = makeBus();
    // What a running OS leaves behind: the card's apertures assigned, its
    // vertical-blank interrupt armed, a cursor position written.
    cardCfgWrite(*bus, 0x18, 0x90000000u);
    cardCfgWrite(*bus, 0x10, 0x94000000u);
    REQUIRE(bus->atiRegBar() == 0x90000000u);
    REQUIRE(bus->atiFbBar() == 0x94000000u);
    bus->write32(0x90000040u, bswap(0x00000001u)); // GEN_INT_CNTL: VBLANK
    bus->write32(0x90000010u, 0x11223344u);        // BIOS_0_SCRATCH
    REQUIRE(bus->read32(0x90000010u) == 0x11223344u);

    bus = SawtoothBus::afterSystemReset(std::move(bus));

    // PCI RST#: the apertures are unassigned again and their sizing bits
    // are the chip's own.
    CHECK(bus->atiRegBar() == 0u);
    CHECK(bus->atiFbBar() == 0u);
    CHECK(cardCfgRead(*bus, 0x18) == 0u);
    CHECK(cardCfgRead(*bus, 0x10) == 0x00000008u);
    // ...and once the firmware assigns them again, the registers behind
    // them hold nothing of the old session.
    cardCfgWrite(*bus, 0x18, 0x90000000u);
    CHECK(bus->read32(0x90000040u) == 0u);
    CHECK(bus->read32(0x90000010u) == 0u);
}

TEST_CASE("system reset: RAM and the flash keep their contents")
{
    auto bus = makeBus();
    bus->write32(0x00020000u, 0xDEADBEEFu);
    bus->write32(0x007FFFF0u, 0x0BADF00Du);
    const std::vector<u8> flashWas = bus->flash();
    const size_t ramWas = bus->ram().size();

    bus = SawtoothBus::afterSystemReset(std::move(bus));

    CHECK(bus->ram().size() == ramWas);
    CHECK(bus->read32(0x00020000u) == 0xDEADBEEFu);
    CHECK(bus->read32(0x007FFFF0u) == 0x0BADF00Du);
    CHECK(bus->flash() == flashWas);
}

TEST_CASE("system reset: the discs stay in the drives")
{
    REQUIRE(makeFile(kDisk, 0x5A, 1u << 20));
    REQUIRE(makeFile(kDisc, 0xC3, 64u * 2048u));
    {
        auto bus = makeBus();
        REQUIRE(bus->attachHd(kDisk));
        REQUIRE(bus->attachCd(kDisc));
        bus->seatDrop();
        REQUIRE(bus->hd().present());
        REQUIRE(bus->cd().mediaPresent());

        bus = SawtoothBus::afterSystemReset(std::move(bus));

        CHECK(bus->hd().present());
        CHECK(bus->hd().isDisk());
        CHECK(bus->hd().mediaPath() == kDisk);
        CHECK(bus->cd().present());
        CHECK(bus->cd().mediaPresent());
        CHECK(bus->cd().mediaPath() == kDisc);
        // The drop drive was seated with an empty tray, and still is.
        CHECK(bus->drop().present());
        CHECK(!bus->drop().mediaPresent());

        // A disc the guest ejected is not pushed back in by a reset.
        bus->cd().hostEject();
        REQUIRE(!bus->cd().mediaPresent());
        bus = SawtoothBus::afterSystemReset(std::move(bus));
        CHECK(bus->cd().present());
        CHECK(!bus->cd().mediaPresent());
    }
    remove(kDisk);
    remove(kDisc);
}

TEST_CASE("system reset: the front end's wiring is carried across")
{
    auto bus = makeBus();
    u64 executed = 12345;
    u32 pc = 0xFFF00100u;
    bus->setStamp(&executed);
    bus->setPcRef(&pc);
    bus->atiVisibleAt = 7;

    bus = SawtoothBus::afterSystemReset(std::move(bus));

    CHECK(bus->stamp == &executed);
    CHECK(bus->pcRef == &pc);
    CHECK(bus->atiVisibleAt == 7u);
}
