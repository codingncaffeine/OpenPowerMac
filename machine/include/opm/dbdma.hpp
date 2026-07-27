#pragma once
#include "opm/types.hpp"

#include <vector>

namespace opm {

class Bus;
class AtaCell;
struct SnapWriter;
struct SnapReader;

// Apple DBDMA channel (KeyLargo lineage), one 0x100 register window:
//   +0x00 channelControl (write: mask in 31:16, values in 15:0)
//   +0x04 channelStatus  (RUN 0x8000, PAUSE 0x4000, FLUSH 0x2000,
//                         WAKE 0x1000, DEAD 0x0800, ACTIVE 0x0400,
//                         BT 0x0100, s7..s0 low byte)
//   +0x0C commandPtrLo · +0x10 interruptSelect · +0x14 branchSelect
//   +0x18 waitSelect
// Registers are little-endian on the bus; the BE-composed access is
// swapped at the edge. The engine walks the 16-byte little-endian
// descriptors at commandPtr: op in word0 bits 31:28 (OUTPUT_MORE=0,
// OUTPUT_LAST=1, INPUT_MORE=2, INPUT_LAST=3, STORE_QUAD=4, LOAD_QUAD=5,
// NOP=6, STOP=7), reqCount in 15:0, word1 = address, word3 receives
// {xferStatus, resCount}. INPUT ops pull real bytes from the attached
// ATA cell's current data phase into RAM — the CD's DMA read path.
class DbdmaChannel {
public:
    u32 read(u32 off, u32 len);
    void write(u32 off, u32 v, u32 len);

    void wake(); // device has fresh data: resume a standing list

    bool irqLine() const { return irq_; }

    Bus* dmaBus = nullptr;
    AtaCell* ata = nullptr;
    const u64* stamp = nullptr;
    const u32* pcRef = nullptr;

    struct Ev {
        u64 at;
        u32 kind, a, b; // 0=ctl 1=desc 2=input 3=stop 4=dead
    };
    std::vector<Ev> log;

    // Snapshot; the bus and ATA-cell pointers are wired by the machine's
    // constructor and stay valid across a load.
    void snapSave(SnapWriter& w) const;
    void snapLoad(SnapReader& r);

private:
    static u32 swap32(u32 v)
    {
        return (v >> 24) | ((v >> 8) & 0xFF00u) | ((v << 8) & 0xFF0000u) |
               (v << 24);
    }
    void run();
    void note(u32 kind, u32 a, u32 b);

    u32 status_ = 0, cmdPtr_ = 0, intSel_ = 0, brSel_ = 0, waitSel_ = 0;
    bool irq_ = false;
};

} // namespace opm
