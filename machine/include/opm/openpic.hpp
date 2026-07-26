#pragma once
#include "opm/types.hpp"

#include <vector>

namespace opm {

// KeyLargo's embedded OpenPIC (the tree's interrupt-controller@40000,
// 256 KB window at mac-io +0x40000). Little-endian register file:
//   +0x01000.. global config (stored, minimally interpreted)
//   +0x010a0.. IPI vector/priority 0-3
//   +0x010e0   spurious vector
//   +0x10000 + n*0x20: per-source vector/priority (bit31 MASK, bit30
//            ACTIVITY, bit22 SENSE level, bits 19:16 priority, 7:0
//            vector) and +0x10 destination
//   +0x20080   per-CPU task priority
//   +0x200a0   IACK: read = highest pending unmasked vector (acks it)
//   +0x200b0   EOI
// Devices raise/lower level lines by source number; the CPU line is the
// OR of pending sources above the task priority.
class OpenPic {
public:
    static constexpr u32 kSources = 64;

    u32 read(u32 off, u32 len);       // off relative to +0x40000, LE regs
    void write(u32 off, u32 v, u32 len);

    void setLine(u32 src, bool level);
    bool cpuLine() const;

    struct Ev {
        u64 at;
        char kind; // 'v' vp write, 'a' iack, 'e' eoi, 'r' raise
        u32 val;
    };
    std::vector<Ev> log;
    const u64* stamp = nullptr;

private:
    u32 iack();
    int highestPending() const;

    u32 vp_[kSources] = {};   // vector/priority as the guest wrote them
    u32 dest_[kSources] = {};
    bool line_[kSources] = {};
    bool pending_[kSources] = {};
    u32 taskPri_ = 0xF; // reset: everything masked until lowered
    u32 spurious_ = 0xFF;
    int inService_ = -1;
    u32 global_[0x400] = {}; // +0x1000 region raw store (LE words)
};

} // namespace opm
