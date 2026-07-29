#pragma once
#include "types.hpp"

namespace opm {

// Cache coherency for bus masters. On a 60x bus every DMA transaction is
// SNOOPED: the address goes out on the bus, the processor's L1 and backside
// L2 check their tags, a modified line is pushed so the master reads current
// data, and a master WRITE kills the processor's stale copy. New World Macs
// depend on that — Open Firmware builds a DBDMA descriptor list with ordinary
// cached stores and arms the channel without flushing anything.
//
// A DMA engine that reads and writes host memory directly models a machine
// with no snooping at all, and the symptom is not subtle: measured on the
// boot disk, the first DBDMA read fetched its descriptor list out of RAM
// while Open Firmware's stores were still sitting dirty in the L1, got
// power-on junk (0x287ec3ff), transferred 512 bytes to a garbage address and
// marked the channel DEAD. The second transfer's list read correctly only
// because the line had been evicted by then — and its data landed in memory
// underneath a cache line the firmware then read stale.
struct SnoopSink {
    virtual ~SnoopSink() = default;
    // A bus master is about to READ this range: push modified lines.
    virtual void snoopRead(u32 pa, u32 len) = 0;
    // A bus master is about to WRITE this range: push, then invalidate.
    virtual void snoopWrite(u32 pa, u32 len) = 0;
};

// Physical-address bus. The 7400 has a 32-bit physical address space (52-bit
// virtual, translated by the MMU before reaching here). All multi-byte
// accesses are big-endian: implementations assemble from big-endian byte
// storage. Alignment has already been handled/faulted by the CPU layer.
class Bus {
public:
    virtual ~Bus() = default;

    // Wired by the front end to the processor. Null means "no snooping",
    // which is only correct for a machine with no caches.
    SnoopSink* snoop = nullptr;
    void snoopBeforeDmaRead(u32 pa, u32 len)
    {
        if (snoop)
            snoop->snoopRead(pa, len);
    }
    void snoopBeforeDmaWrite(u32 pa, u32 len)
    {
        if (snoop)
            snoop->snoopWrite(pa, len);
    }

    virtual u8  read8(u32 pa) = 0;
    virtual u16 read16(u32 pa) = 0;
    virtual u32 read32(u32 pa) = 0;
    virtual u64 read64(u32 pa) = 0;

    virtual void write8(u32 pa, u8 v) = 0;
    virtual void write16(u32 pa, u16 v) = 0;
    virtual void write32(u32 pa, u32 v) = 0;
    virtual void write64(u32 pa, u64 v) = 0;

    // Burst (cache-line) transactions, the 60x TBST-asserted class. Chipset
    // caches allocate on these but not on the single-beat accessors above
    // (MPC106 UM Table 5-2). pa is 32-byte aligned; the default falls back
    // to word ops for buses that don't care about the distinction.
    virtual void readLine32(u32 pa, u8* out)
    {
        for (u32 k = 0; k < 32; k += 4) {
            const u32 w = read32(pa + k);
            out[k] = static_cast<u8>(w >> 24);
            out[k + 1] = static_cast<u8>(w >> 16);
            out[k + 2] = static_cast<u8>(w >> 8);
            out[k + 3] = static_cast<u8>(w);
        }
    }
    virtual void writeLine32(u32 pa, const u8* b)
    {
        for (u32 k = 0; k < 32; k += 4)
            write32(pa + k, (u32(b[k]) << 24) | (u32(b[k + 1]) << 16) |
                                (u32(b[k + 2]) << 8) | u32(b[k + 3]));
    }
};

} // namespace opm
