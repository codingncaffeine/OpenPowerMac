#pragma once
#include "types.hpp"
#include "bus.hpp"
#include "insn.hpp"
#include <map>
#include <string>

namespace opm {

// Full architected state of the MPC7400. Reset values per UM Table 2-18
// ("Settings Caused by Hard Reset"); fields the table calls undefined are
// zero-initialized here for determinism.
struct CpuState {
    u32 gpr[32]{};
    u64 fpr[32]{};      // raw IEEE-754 double bit patterns
    V128 vr[32]{};

    u32 pc = 0xFFF00100u;
    u32 cr = 0, xer = 0, lr = 0, ctr = 0;
    u32 msr = 0x00000040u;      // only IP set at HRESET
    u32 fpscr = 0;
    u32 vscr = 0, vrsave = 0;

    u32 srr0 = 0, srr1 = 0;
    u32 sprg[4]{};
    u32 dar = 0, dsisr = 0, sdr1 = 0, ear = 0, pir = 0;
    u32 sr[16]{};
    u32 ibatu[4]{}, ibatl[4]{};
    u32 dbatu[4]{}, dbatl[4]{};

    u32 hid0 = 0, hid1 = 0;
    u32 msscr0 = 0x00400000u, msscr1 = 0;
    u32 l2cr = 0, ictc = 0;
    u32 thrm[3]{};
    u32 iabr = 0, dabr = 0, bamr = 0;
    u32 mmcr0 = 0, mmcr1 = 0, pmc[4]{}, siar = 0, sdar = 0;

    u64 tb = 0;
    u32 dec = 0xFFFFFFFFu;
    u32 pvr = 0x000C0209u;      // MPC7400 silicon rev 2.9 (errata Table 2)

    bool resvValid = false;
    u32 resvAddr = 0;           // 32-byte-granule base
};

struct Cpu {
    CpuState st;
    Bus* bus = nullptr;

    // Census of decode gaps hit at runtime: mnemonic (or raw word) -> count.
    std::map<std::string, u64> unimplemented;
    std::map<u32, u64> unknownWords;

    void attach(Bus& b) { bus = &b; }
    void reset() { st = CpuState{}; }

    // Execute up to n instructions; returns the number actually executed.
    // (P0: decode + census only; handlers land per implementation phase.)
    u64 run(u64 n);
    void step();
};

} // namespace opm
