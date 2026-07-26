#pragma once
// Arc 2 M0: the Gossamer (beige Power Mac G3) address map, instrumented.
//
// Physical map modeled so far:
//   0x00000000 .. ramSize     system RAM (Grackle-controlled banks, flat here)
//   0xF3000000 .. +1 MB       "Heathrow" mac-io window — stub: reads 0, logged
//   0xFEC00000 / 0xFEE00000   Grackle PCI CONFIG_ADDR / CONFIG_DATA — stub:
//                             address latch stored, data reads all-ones
//                             (master abort: no PCI devices yet), logged
//   0xFFC00000 .. 4 GB        4 MB boot ROM (reset vector 0xFFF00100 inside)
//   anything else             unmapped: reads all-ones, writes dropped, logged
//
// RECEIPTS: unmapped/PCI-hole reads return all-ones (PCI master-abort
// convention); unknown mac-io registers read as zero (quiet device) — both
// deterministic stubs to be replaced by real models in M1/M2. Every touched
// stub address is logged (deduped, capped) — the log IS the M0 deliverable.

#include "opm/bus.hpp"

#include <cstddef>
#include <map>
#include <vector>

namespace opm {

class GossamerBus final : public Bus {
public:
    GossamerBus(size_t ramBytes, std::vector<u8> rom);

    struct Touch {
        u64 reads = 0, writes = 0;
        u32 lastWrite = 0;
    };
    const std::map<u32, Touch>& stubLog() const { return stubLog_; }
    u64 romWrites() const { return romWrites_; }

    u8 read8(u32 pa) override;
    u16 read16(u32 pa) override;
    u32 read32(u32 pa) override;
    u64 read64(u32 pa) override;
    void write8(u32 pa, u8 v) override;
    void write16(u32 pa, u16 v) override;
    void write32(u32 pa, u32 v) override;
    void write64(u32 pa, u64 v) override;

private:
    static constexpr u32 kRomBase = 0xFFC00000u;
    static constexpr u32 kMacIoBase = 0xF3000000u;
    static constexpr u32 kMacIoSize = 0x00100000u;
    static constexpr u32 kConfigAddr = 0xFEC00000u;
    static constexpr u32 kConfigData = 0xFEE00000u;

    u32 readWord(u32 pa);        // aligned-word backend
    void writeWord(u32 pa, u32 v);
    void logStub(u32 pa, bool write, u32 v);

    std::vector<u8> ram_;
    std::vector<u8> rom_;
    u32 configAddr_ = 0;
    std::map<u32, Touch> stubLog_;
    u64 romWrites_ = 0;
};

} // namespace opm
