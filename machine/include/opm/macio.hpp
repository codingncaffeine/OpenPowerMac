#pragma once
// "Heathrow" mac-io ASIC (Arc 2 M2): the I/O core of the Gossamer board.
// Modeled so far: the interrupt controller, the VIA (with T1/T2 timers and
// the shift-register Cuda transport), and a flat byte backing store for
// everything else in the 512 KB window — feature-control registers read
// back what was written, the NVRAM region simply persists, and every access
// to an unmodeled register is logged for the boot trace.
//
// VIA-Cuda transport wiring (port B): TREQ = bit 3 (input from Cuda),
// TACK = bit 4 (output), TIP = bit 5 (output); the shift register moves the
// bytes, IFR bit 2 (SR complete) paces each one. Tier-4 behavioral protocol,
// pinned empirically against the boot ROM.

#include "opm/bus.hpp"
#include "opm/cuda.hpp"
#include "opm/types.hpp"

#include <map>
#include <vector>

namespace opm {

class MacIo {
public:
    MacIo();

    u8 read8(u32 off);
    void write8(u32 off, u8 v);
    void tick(); // advance VIA timers / transport pacing

    bool irqAsserted() const;
    Cuda& cuda() { return cuda_; }

    // Heathrow interrupt sources (bit numbers within the 64-source space),
    // from real-hardware logs (tier-4/5): MESH=12, IDE0/1=13/14, ESCC
    // A/B=15/16, VIA/Cuda=18. NOT 31: the enable bit 31 HWInit sets at
    // boot belongs to a quiet source (NMI/programmer-switch class) —
    // wiring the VIA there delivered into the nanokernel's still-null
    // per-vector context table ([SPRG3+0x500]) and killed the OF era.
    static constexpr u32 kIrqMesh = 12, kIrqIde0 = 13, kIrqIde1 = 14;
    static constexpr u32 kIrqSccA = 15, kIrqSccB = 16, kIrqVia = 18;

    struct Touch {
        u64 reads = 0, writes = 0;
        u8 lastWrite = 0;
    };
    const std::map<u32, Touch>& unmodeledLog() const { return log_; }

    struct ViaOp {
        u8 write, reg, val;
    };
    const std::vector<ViaOp>& viaTrace() const { return viaTrace_; }

    struct XferByte {
        u8 toCuda, val; // direction + byte value
    };
    const std::vector<XferByte>& xferLog() const { return xferLog_; }

    const u64* stamp = nullptr; // debug: instruction counter for traces

    struct PicOp {
        u64 at;
        u8 write, off, val;
    };
    const std::vector<PicOp>& picTrace() const { return picTrace_; }

    struct SndOp {
        u64 at;
        u8 write;
        u32 off;
        u8 val;
        u32 pc;
    };
    const std::vector<SndOp>& sndTrace() const { return sndTrace_; }
    const u32* pcRef = nullptr; // debug: live PC for traces

    void debugState(char* out, size_t cap) const;

private:
    // VIA registers (offset 0x16000, stride 0x200).
    enum {
        vORB = 0, vORA = 1, vDDRB = 2, vDDRA = 3,
        vT1CL = 4, vT1CH = 5, vT1LL = 6, vT1LH = 7,
        vT2CL = 8, vT2CH = 9, vSR = 10, vACR = 11,
        vPCR = 12, vIFR = 13, vIER = 14, vORAnh = 15,
    };
    u8 viaRead(u32 reg);
    void viaWrite(u32 reg, u8 v);
    void setIfr(u8 bits);
    void clearIfr(u8 bits);
    void cudaClockByte(); // one SR transfer paced by a TACK edge
    void updateTreq();

    // Heathrow interrupt controller (pic.c-corroborated layout): two
    // {event, enable, ack, level} blocks of 32-bit LE registers — IRQs
    // 0-31 at +0x20..0x2C, IRQs 32-63 at +0x10..0x1C. Level-holding
    // sources re-latch their event bit while asserted.
    u32 picLevels(u32 blk) const;
    void picLatch();
    u8 picRead(u32 off);
    bool picWrite(u32 off, u8 v); // true if the offset belonged to the PIC

    u32 picEvent_[2] = {};  // [0] = IRQs 0-31, [1] = IRQs 32-63
    u32 picEnable_[2] = {};
    std::vector<PicOp> picTrace_;
    std::vector<SndOp> sndTrace_;

    // DBDMA channels (mac-io +0x8000, one 0x100 window each): the standard
    // register file — control (mask<<16|bits write protocol), live status,
    // command pointer. Enough engine for drivers that program a channel
    // and verify RUN|ACTIVE in the status.
    struct Dbdma {
        u32 status = 0; // RUN 0x8000, PAUSE 0x4000, FLUSH 0x2000,
                        // WAKE 0x1000, DEAD 0x0800, ACTIVE 0x0400
        u32 cmdPtr = 0;
        u32 intSel = 0, brSel = 0, waitSel = 0;
    };
    Dbdma dbdma_[16];
    u32 ctrlPend_[16] = {}; // lane-assembly latch for control writes
    u8 dbdmaRead(u32 off);
    bool dbdmaWrite(u32 off, u8 v);
    void dbdmaRun(u32 chan); // walk the command list until STOP

public:
    // The DMA engine fetches descriptors from system memory; the owning
    // bus provides the path (set once at machine construction).
    Bus* dmaBus = nullptr;

private:

    Cuda cuda_;
    std::vector<u8> store_;

    u8 via_[16] = {};
    u8 ifr_ = 0, ier_ = 0;
    u16 t1_ = 0xFFFF, t2_ = 0xFFFF;
    bool t1Running_ = true, t2Running_ = false;
    u32 viaDivider_ = 0;

    // Cuda transport state.
    std::vector<u8> hostPkt_;
    u32 respIndex_ = 0;
    u32 respDelay_ = 0; // VIA ticks before a queued response asserts TREQ
    u32 syncPulse_ = 0; // VIA ticks before the sync-negate SR pulse lands
    bool sending_ = false;   // Cuda -> host transfer in progress
    bool receiving_ = false; // host -> Cuda transfer in progress
    bool syncing_ = false;   // null transaction: TIP asserted while idle
    bool treq_ = false;      // Cuda asserting TREQ (active state, not wire level)

    // ESCC (Z8530) channels B/A: register pointer + WR file per channel.
    u32 esccPtr_[2] = {};
    u8 esccWr_[2][16] = {};

    std::map<u32, Touch> log_;
    std::vector<ViaOp> viaTrace_;
    std::vector<XferByte> xferLog_;
};

} // namespace opm
