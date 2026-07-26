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
};

} // namespace opm
