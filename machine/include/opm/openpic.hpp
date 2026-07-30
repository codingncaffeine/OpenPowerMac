#pragma once
#include "opm/types.hpp"

#include <vector>

namespace opm {

struct SnapWriter;
struct SnapReader;

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
    // Why is a raised source not being delivered? The line count alone
    // cannot say: a source can raise a thousand times and never reach the
    // CPU because its vector-priority still has the mask bit set, or its
    // priority does not beat the task priority, or an earlier interrupt is
    // still in service and was never EOId.
    void dumpState() const;
    // --vbl-trace N: print the first N iack/eoi events. `log` is capped at 512
    // AND snapshotted, so on a resume its tail sits in Open Firmware's era and
    // it cannot answer "which iack went un-EOI'd". A static, so sizeof(OpenPic)
    // — and every snapshot — is untouched.
    static void setTrace(int n);
    struct Unc { u64 at; u32 off; u32 val; };
    std::vector<Unc> unclaimed;

    struct Ev {
        u64 at;
        char kind; // 'v' vp write, 'a' iack, 'e' eoi, 'r' raise
        u32 val;
    };
    std::vector<Ev> log;    // v/a/e only — raises would flood the cap
    u64 raiseCount[64] = {}; // per-source raise tally, uncapped
    const u64* stamp = nullptr;

    void snapSave(SnapWriter& w) const;
    void snapLoad(SnapReader& r);

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
