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
    bool sending_ = false;   // Cuda -> host transfer in progress
    bool receiving_ = false; // host -> Cuda transfer in progress
    bool syncing_ = false;   // null transaction: TIP asserted while idle
    bool treq_ = false;      // Cuda asserting TREQ (active state, not wire level)

    std::map<u32, Touch> log_;
    std::vector<ViaOp> viaTrace_;
    std::vector<XferByte> xferLog_;
};

} // namespace opm
