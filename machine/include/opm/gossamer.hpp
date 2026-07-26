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

#include "opm/ati.hpp"
#include "opm/bus.hpp"
#include "opm/macio.hpp"
#include "opm/pci.hpp"

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
    MacIo& macio() { return macio_; }
    PciConfig& pci() { return pci_; }
    AtiRage& ati() { return ati_; }

    // Debug instrument: watch a physical window (the OS's hash table) and
    // record every word write into it, stamped with *stamp.
    struct HtabWr {
        u64 at;
        u32 pa, oldW, newW;
    };
    u32 htabWatchBase = 0, htabWatchSize = 0;
    const u64* stamp = nullptr;
    const std::vector<HtabWr>& htabLog() const { return htabLog_; }

    // Debug instrument: every byte written into Grackle (dev 0) config
    // space, stamped — the memory-init story in write order.
    struct CfgWr {
        u64 at;
        u32 reg;
        u8 val;
    };
    const std::vector<CfgWr>& cfgLog() const { return cfgLog_; }

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
    static constexpr u32 kRomAlias = 0xFF800000u; // 8 MB decode mirrors the 4 MB ROM
    static constexpr u32 kRomMask = 0x003FFFFFu;
    static constexpr u32 kMacIoBase = 0xF3000000u;
    static constexpr u32 kMacIoSize = 0x00100000u;
    static constexpr u32 kConfigAddr = 0xFEC00000u;
    static constexpr u32 kConfigData = 0xFEE00000u;

    u32 readWord(u32 pa);        // aligned-word backend
    void writeWord(u32 pa, u32 v);
    void logStub(u32 pa, bool write, u32 v);

    // Grackle 60x memory decode: banks from the config registers, gated by
    // MEMGO; a claimed access yields an offset into the backing DIMM with
    // wrap aliasing. Unclaimed 60x-memory accesses master-abort.
    bool ramClaim(u32 pa, u32& off);
    PciConfig::RamBank banks_[8];
    u32 bankGen_ = ~0u;
    bool memGo_ = false;

public:
    // Experiment lever: pretend bank 0 spans the DIMM regardless of the
    // config registers (counterfactual isolation of the enable question).
    bool forceBank0 = false;

private:

    // MPC106 internally-controlled L2 (UM ch.5 / PICR1[CF_L2_MP] +
    // PICR2[L2_EN]): a direct-mapped write-back lookaside cache on the 60x
    // memory space. Load-bearing pre-DRAM: HWInit enables it right before
    // the memory-init module runs, and the module's page tables and
    // workspace live in it (dcbf pushes land here) before any bank exists.
    struct Ml2Line {
        bool v = false, d = false;
        u32 tag = 0;
        u8 b[32] = {};
    };
    std::vector<Ml2Line> ml2_;
    u32 ml2Lines_ = 0;
    bool ml2On_ = false;

public:
    // Instrumentation: dirty inline-L2 castouts that had no DRAM to land in
    // (data genuinely lost), plus the touched-line census.
    u64 ml2LostCastouts = 0, ml2Fills = 0;
    std::vector<std::pair<u64, u32>> ml2LossLog; // (stamp, line PA)

private:
    void memCfgRefresh(); // banks + MEMGO + inline-L2 state from config
    Ml2Line* ml2Route(u32 pa);  // burst path: hit or allocate-with-fill
    Ml2Line* ml2Lookup(u32 pa); // single-beat path: hit only, no allocate

public:
    // Burst transactions allocate in the inline L2; single-beat accesses
    // (the plain accessors) only hit it (MPC106 UM Table 5-2).
    void readLine32(u32 pa, u8* out) override;
    void writeLine32(u32 pa, const u8* b) override;

private:

    bool atiWindow(u32 pa, u32& off) const;
    bool atiIoWindow(u32 pa, u32& off) const;

    std::vector<u8> ram_;
    std::vector<u8> rom_;
    MacIo macio_;
    PciConfig pci_;
    AtiRage ati_;
    void logCfgWrite(u32 lane, u8 v);

    u32 configAddr_ = 0;
    std::map<u32, Touch> stubLog_;
    std::vector<HtabWr> htabLog_;
    std::vector<CfgWr> cfgLog_;
    u64 romWrites_ = 0;
};

} // namespace opm
