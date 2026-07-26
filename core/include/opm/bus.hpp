#pragma once
#include "types.hpp"

namespace opm {

// Physical-address bus. The 7400 has a 32-bit physical address space (52-bit
// virtual, translated by the MMU before reaching here). All multi-byte
// accesses are big-endian: implementations assemble from big-endian byte
// storage. Alignment has already been handled/faulted by the CPU layer.
class Bus {
public:
    virtual ~Bus() = default;

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
