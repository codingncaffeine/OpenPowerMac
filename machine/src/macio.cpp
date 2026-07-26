#include "opm/macio.hpp"

#include <cstdio>

namespace opm {

namespace {

inline constexpr u32 kViaBase = 0x16000u;
inline constexpr u32 kViaEnd = 0x18000u;
inline constexpr u32 kIntEvents = 0x20u; // second-cell events/mask/clear/levels
inline constexpr u32 kIntMask = 0x24u;
inline constexpr u32 kIntClear = 0x28u;
inline constexpr u32 kIntLevels = 0x2Cu;

// Port B Cuda lines.
inline constexpr u8 bTREQ = 0x08;
inline constexpr u8 bTACK = 0x10;
inline constexpr u8 bTIP = 0x20;

// IFR bits.
inline constexpr u8 iSR = 0x04;
inline constexpr u8 iT2 = 0x20;
inline constexpr u8 iT1 = 0x40;

// Heathrow interrupt bit for the VIA/Cuda (RECEIPT: empirical wiring; the
// ROM polls the VIA directly during early boot, so only the OS-visible
// number matters and can be re-pinned then).
inline constexpr u32 kIrqVia = 1u << 18;

inline constexpr size_t kLogCap = 512;

} // namespace

MacIo::MacIo() : store_(0x100000, 0)
{
    via_[vORB] = bTREQ; // TREQ negated (high) at rest
}

bool MacIo::irqAsserted() const
{
    // Real chain: the VIA's IRQ output is IFR&IER; premature delivery sends
    // the ROM through not-yet-installed vectors (observed: NULL dispatch).
    return (ifr_ & ier_ & 0x7F) != 0;
}

void MacIo::setIfr(u8 bits)
{
    ifr_ |= bits & 0x7F;
}
void MacIo::clearIfr(u8 bits)
{
    ifr_ &= static_cast<u8>(~(bits & 0x7F));
}

void MacIo::updateTreq()
{
    const u8 old = via_[vORB];
    if (treq_)
        via_[vORB] &= static_cast<u8>(~bTREQ); // asserted = low
    else
        via_[vORB] |= bTREQ;
    // TREQ also drives a VIA control pin: its falling (assert) edge latches
    // an IFR flag the ROM's service task can poll (PCR=0: negative edges).
    if ((old & bTREQ) && !(via_[vORB] & bTREQ))
        setIfr(0x10); // CB1
}

// One byte moves between the SR and the Cuda, paced by a TACK transition
// (or TIP assertion for the first byte of a Cuda->host transfer).
void MacIo::cudaClockByte()
{
    const bool tip = (via_[vORB] & bTIP) == 0; // active low
    if (!tip)
        return;
    const bool shiftOut = (via_[vACR] & 0x10u) != 0; // any SR output mode
    if (shiftOut) {
        // Host -> Cuda byte already in SR.
        receiving_ = true;
        hostPkt_.push_back(via_[vSR]);
        if (xferLog_.size() < 200)
            xferLog_.push_back({1, via_[vSR]});
        setIfr(iSR);
        return;
    }
    // Cuda -> host: load next response byte into SR.
    if (sending_ && cuda_.hasResponse()) {
        const std::vector<u8>& r = cuda_.response();
        if (respIndex_ < r.size()) {
            via_[vSR] = r[respIndex_++];
            if (xferLog_.size() < 200)
                xferLog_.push_back({0, via_[vSR]});
            setIfr(iSR);
            // TREQ negates before the last byte so the host knows to stop.
            if (respIndex_ >= r.size()) {
                treq_ = false;
                updateTreq();
            }
        }
    }
}

u8 MacIo::viaRead(u32 reg)
{
    switch (reg) {
    case vORB: {
        return via_[vORB];
    }
    case vORA:
    case vORAnh:
        return via_[vORA];
    case vDDRB:
        return via_[vDDRB];
    case vDDRA:
        return via_[vDDRA];
    case vT1CL:
        clearIfr(iT1);
        return static_cast<u8>(t1_);
    case vT1CH:
        return static_cast<u8>(t1_ >> 8);
    case vT1LL:
        return via_[vT1LL];
    case vT1LH:
        return via_[vT1LH];
    case vT2CL:
        clearIfr(iT2);
        return static_cast<u8>(t2_);
    case vT2CH:
        return static_cast<u8>(t2_ >> 8);
    case vSR:
        clearIfr(iSR);
        return via_[vSR];
    case vACR:
        return via_[vACR];
    case vPCR:
        return via_[vPCR];
    case vIFR: {
        u8 v = ifr_;
        if (ifr_ & ier_ & 0x7F)
            v |= 0x80;
        return v;
    }
    case vIER:
        return static_cast<u8>(ier_ | 0x80);
    default:
        return 0;
    }
}

void MacIo::viaWrite(u32 reg, u8 v)
{
    switch (reg) {
    case vORB: {
        const u8 old = via_[vORB];
        // Only output bits (per DDRB) change; TREQ is an input.
        const u8 outMask = via_[vDDRB];
        via_[vORB] = static_cast<u8>((via_[vORB] & ~outMask) | (v & outMask));
        updateTreq();
        if (((old ^ via_[vORB]) & (bTIP | bTACK)) && xferLog_.size() < 200)
            xferLog_.push_back({3, via_[vORB]}); // tag 3: TIP/TACK edge

        const bool tipNow = (via_[vORB] & bTIP) == 0;
        const bool tipWas = (old & bTIP) == 0;
        const bool tackFlipped = ((old ^ via_[vORB]) & bTACK) != 0;

        if (tipNow && !tipWas) {
            // Transfer opens. If Cuda has a response queued, this is the
            // host coming to collect it: load the first byte immediately.
            if (cuda_.hasResponse()) {
                sending_ = true;
                respIndex_ = 0;
                const std::vector<u8>& r = cuda_.response();
                via_[vSR] = r[respIndex_++];
                if (xferLog_.size() < 200)
                    xferLog_.push_back({0, via_[vSR]});
                setIfr(iSR);
                if (respIndex_ >= r.size()) {
                    treq_ = false;
                    updateTreq();
                }
            } else if (via_[vACR] & 0x10u) {
                // Shift-out: the host staged byte 0 in the SR before
                // asserting TIP — the TIP edge clocks it (observed framing).
                receiving_ = true;
                hostPkt_.clear();
                hostPkt_.push_back(via_[vSR]);
                if (xferLog_.size() < 200)
                    xferLog_.push_back({1, via_[vSR]});
                setIfr(iSR);
            } else {
                // TIP asserted while idle in shift-in mode: the sync/null
                // transaction — Cuda acknowledges by asserting TREQ, the
                // host's TACK edge completes it.
                syncing_ = true;
                treq_ = true;
                updateTreq();
                setIfr(iSR);
            }
        } else if (!tipNow && tipWas) {
            // Transfer closes.
            if (receiving_) {
                receiving_ = false;
                if (!hostPkt_.empty()) {
                    cuda_.hostPacket(hostPkt_);
                    hostPkt_.clear();
                    if (cuda_.hasResponse())
                        respDelay_ = 256; // RECEIPT: the MCU "thinks" for a
                                          // while; the host wants to see the
                                          // bus idle before the response TREQ
                }
            }
            if (sending_) {
                sending_ = false;
                cuda_.consumeResponse();
                respIndex_ = 0;
                treq_ = false;
                updateTreq();
            }
            syncing_ = false;
            setIfr(iSR); // edge on close paces the ROM's final wait
        } else if (tipNow && tackFlipped) {
            if (syncing_) {
                treq_ = false; // sync acknowledged and released
                updateTreq();
                setIfr(iSR);
            } else {
                cudaClockByte();
            }
        } else if (!tipNow && tackFlipped && !sending_ && !receiving_ &&
                   (!cuda_.hasResponse() || respDelay_ != 0)) {
            // Sync stepping (observed in the boot ROM): with TIP negated,
            // TACK edges are acknowledged by mirroring TREQ. The final
            // TACK-assert of cuda_init parks here until the host releases.
            treq_ = (via_[vORB] & bTACK) == 0;
            updateTreq();
            setIfr(iSR);
        }
        return;
    }
    case vORA:
    case vORAnh:
        via_[vORA] = v;
        return;
    case vDDRB:
        via_[vDDRB] = v;
        return;
    case vDDRA:
        via_[vDDRA] = v;
        return;
    case vT1CL:
    case vT1LL:
        via_[vT1LL] = v;
        return;
    case vT1LH:
        via_[vT1LH] = v;
        return;
    case vT1CH:
        via_[vT1CH] = v;
        t1_ = static_cast<u16>((v << 8) | via_[vT1LL]);
        t1Running_ = true;
        clearIfr(iT1);
        return;
    case vT2CL:
        via_[vT2CL] = v;
        return;
    case vT2CH:
        t2_ = static_cast<u16>((v << 8) | via_[vT2CL]);
        t2Running_ = true;
        clearIfr(iT2);
        return;
    case vSR:
        via_[vSR] = v;
        clearIfr(iSR);
        if (xferLog_.size() < 200)
            xferLog_.push_back({2, v}); // tag 2: SR write (host staging)
        // In shift-out mode with a transfer open, writing SR is the byte;
        // the ROM then flips TACK to clock it across.
        return;
    case vACR:
        via_[vACR] = v;
        return;
    case vPCR:
        via_[vPCR] = v;
        return;
    case vIFR:
        clearIfr(v);
        return;
    case vIER:
        if (v & 0x80)
            ier_ |= v & 0x7F;
        else
            ier_ &= static_cast<u8>(~(v & 0x7F));
        return;
    default:
        return;
    }
}

void MacIo::tick()
{
    // VIA timebase ~ CPU/8 (RECEIPT: pacing ratio; boot timing loops only
    // need steady movement, and the divider can be re-pinned any time).
    if (++viaDivider_ < 8)
        return;
    viaDivider_ = 0;
    if (respDelay_ && --respDelay_ == 0 && !sending_ && !receiving_ &&
        cuda_.hasResponse()) {
        treq_ = true; // the delayed response now requests the bus
        updateTreq();
    }
    if (t1Running_) {
        if (t1_ == 0) {
            setIfr(iT1);
            if (via_[vACR] & 0x40) { // free-run: reload from latches
                t1_ = static_cast<u16>((via_[vT1LH] << 8) | via_[vT1LL]);
            } else {
                t1Running_ = false;
                t1_ = 0xFFFF;
            }
        } else {
            --t1_;
        }
    }
    if (t2Running_) {
        if (t2_ == 0) {
            setIfr(iT2);
            t2Running_ = false;
            t2_ = 0xFFFF;
        } else {
            --t2_;
        }
    }
}

void MacIo::debugState(char* out, size_t cap) const
{
    snprintf(out, cap,
             "via: ORB=%02x DDRB=%02x ACR=%02x IFR=%02x IER=%02x SR=%02x "
             "treq=%d send=%d recv=%d respPending=%d",
             via_[vORB], via_[vDDRB], via_[vACR], ifr_, ier_, via_[vSR],
             treq_ ? 1 : 0, sending_ ? 1 : 0, receiving_ ? 1 : 0,
             cuda_.hasResponse() ? 1 : 0);
}

u8 MacIo::read8(u32 off)
{
    if (off >= kViaBase && off < kViaEnd) {
        const u32 reg = (off - kViaBase) >> 9;
        return viaRead(reg); // reads flood; the write timeline tells the story
    }
    // Interrupt-controller events: expose the VIA source in both candidate
    // banks (+0x10 and +0x20 event words) and both candidate bits (LE 18
    // and 31) so the wake handler's reads pin the real layout empirically.
    if (off >= 0x10 && off < 0x30) {
        if (log_.size() < kLogCap || log_.count(off))
            ++log_[off].reads;
        const bool pend = (ifr_ & 0x04) != 0;
        if (pend && (off == 0x12 || off == 0x22))
            return 0x04; // LE byte 2: bit 18
        if (pend && (off == 0x13 || off == 0x23))
            return 0x80; // LE byte 3: bit 31
        return store_[off];
    }
    if (log_.size() < kLogCap || log_.count(off))
        ++log_[off].reads;
    return store_[off];
}

void MacIo::write8(u32 off, u8 v)
{
    if (off >= kViaBase && off < kViaEnd) {
        const u32 reg = (off - kViaBase) >> 9;
        if (viaTrace_.size() < 2000)
            viaTrace_.push_back({1, static_cast<u8>(reg), v});
        viaWrite(reg, v);
        return;
    }
    if (off == kIntClear || off == kIntClear + 1 || off == kIntClear + 2 ||
        off == kIntClear + 3) {
        return; // write-only clear; events are derived live
    }
    if (log_.size() < kLogCap || log_.count(off)) {
        Touch& t = log_[off];
        ++t.writes;
        t.lastWrite = v;
    }
    store_[off] = v;
}

} // namespace opm
