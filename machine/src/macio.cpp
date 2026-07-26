#include "opm/macio.hpp"

#include <cstdio>

namespace opm {

namespace {

inline constexpr u32 kViaBase = 0x16000u;
inline constexpr u32 kViaEnd = 0x18000u;

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

// The mac-io INT line to the CPU: any latched event whose enable bit is
// set, in either 32-source block. Until the OS programs an enable mask,
// nothing is delivered — which is also why the early ROM (which only ever
// polls the VIA directly) sees a quiet line.
bool MacIo::irqAsserted() const
{
    return ((picEvent_[0] & picEnable_[0]) |
            (picEvent_[1] & picEnable_[1])) != 0;
}

// Live level per block: sources that hold their line while asserted.
u32 MacIo::picLevels(u32 blk) const
{
    if (blk != 0)
        return 0;
    u32 lvl = 0;
    if ((ifr_ & ier_ & 0x7Fu) != 0)
        lvl |= 1u << kIrqVia; // the VIA's IRQ output feeds source 18
    return lvl;
}

// A held level keeps its event bit latched: acking while the source is
// still asserted re-latches immediately (level-triggered input).
void MacIo::picLatch()
{
    picEvent_[0] |= picLevels(0);
    picEvent_[1] |= picLevels(1);
}

// Register file: 32-bit little-endian words served per byte lane.
//   +0x10 event[1]  +0x14 enable[1]  +0x18 ack[1]  +0x1C level[1]
//   +0x20 event[0]  +0x24 enable[0]  +0x28 ack[0]  +0x2C level[0]
u8 MacIo::picRead(u32 off)
{
    const u32 blk = off < 0x20u ? 1u : 0u;
    const u32 reg = (off >> 2) & 3u; // 0 event, 1 enable, 2 ack, 3 level
    const u32 lane = off & 3u;
    u32 v = 0;
    switch (reg) {
    case 0: v = picEvent_[blk]; break;
    case 1: v = picEnable_[blk]; break;
    case 2: v = 0; break; // ack is write-only
    default: v = picLevels(blk); break;
    }
    return static_cast<u8>(v >> (8 * lane));
}

bool MacIo::picWrite(u32 off, u8 v)
{
    const u32 blk = off < 0x20u ? 1u : 0u;
    const u32 reg = (off >> 2) & 3u;
    const u32 lane = off & 3u;
    const u32 bits = u32(v) << (8 * lane);
    if (picTrace_.size() < 512)
        picTrace_.push_back({stamp ? *stamp : 0, 1, static_cast<u8>(off), v});
    switch (reg) {
    case 1:
        picEnable_[blk] =
            (picEnable_[blk] & ~(0xFFu << (8 * lane))) | bits;
        return true;
    case 2:
        picEvent_[blk] &= ~bits; // write-1-to-clear
        picLatch();              // held levels re-assert at once
        return true;
    default:
        return true; // event/level are read-only; writes are absorbed
    }
}

// DBDMA register file: 32-bit little-endian words served per byte lane.
// channelControl writes carry a mask in bits 31:16 and values in 15:0
// (Apple DBDMA convention); channelStatus reflects the run state — RUN
// implies ACTIVE here (the modeled engine "keeps up" instantly). FLUSH
// and WAKE complete immediately and read back clear.
u8 MacIo::dbdmaRead(u32 off)
{
    Dbdma& ch = dbdma_[(off >> 8) & 15u];
    const u32 reg = (off >> 2) & 0x3Fu;
    const u32 lane = off & 3u;
    u32 v = 0;
    switch (reg) {
    case 1: v = ch.status; break;      // +0x04 channelStatus
    case 3: v = ch.cmdPtr; break;      // +0x0C commandPtrLo
    case 4: v = ch.intSel; break;      // +0x10 interruptSelect
    case 5: v = ch.brSel; break;       // +0x14 branchSelect
    case 6: v = ch.waitSel; break;     // +0x18 waitSelect
    default: break;
    }
    return static_cast<u8>(v >> (8 * lane));
}

bool MacIo::dbdmaWrite(u32 off, u8 v)
{
    Dbdma& ch = dbdma_[(off >> 8) & 15u];
    const u32 reg = (off >> 2) & 0x3Fu;
    const u32 lane = off & 3u;
    const u32 bits = u32(v) << (8 * lane);
    const u32 clear = 0xFFu << (8 * lane);
    switch (reg) {
    case 0: { // channelControl: assemble the LE word lane by lane, apply
              // when the high (mask) half has been written (lane 3).
        u32& w = ctrlPend_[(off >> 8) & 15u];
        w = (w & ~clear) | bits;
        if (lane == 3u) {
            const u32 mask = w >> 16, val = w & 0xFFFFu;
            const u32 wasRun = ch.status & 0x8000u;
            ch.status = (ch.status & ~mask) | (val & mask);
            ch.status &= ~0x3000u; // FLUSH/WAKE self-complete
            if (ch.status & 0x8000u) {
                ch.status |= 0x0400u; // RUN -> ACTIVE
                if (!wasRun)
                    dbdmaRun((off >> 8) & 15u); // engine starts now
            } else {
                ch.status &= ~0x0400u;
            }
            w = 0;
        }
        return true;
    }
    case 3: ch.cmdPtr = (ch.cmdPtr & ~clear) | bits; return true;
    case 4: ch.intSel = (ch.intSel & ~clear) | bits; return true;
    case 5: ch.brSel = (ch.brSel & ~clear) | bits; return true;
    case 6: ch.waitSel = (ch.waitSel & ~clear) | bits; return true;
    default: return true; // status is read-only; others absorbed
    }
}

// The DBDMA engine proper: walk the little-endian command list from
// cmdPtr, executing until STOP. Descriptors are 16 bytes: word0 holds the
// operation in bits 31:28 (0/1 OUTPUT, 2/3 INPUT, 4 STORE_QUAD,
// 5 LOAD_QUAD, 6 NOP, 7 STOP) with reqCount in 15:0; word1 = address;
// word3 receives {xferStatus, resCount} writeback. Sound output data has
// no audio backend yet — samples are consumed into the void, which is
// exactly what the boot beep needs to complete. RECEIPT: branches, waits
// and interrupt-select conditions are not yet honored (none appear in the
// ROM's beep list); revisit when a driver uses them.
void MacIo::dbdmaRun(u32 chan)
{
    Dbdma& ch = dbdma_[chan];
    if (!dmaBus)
        return;
    auto rd32le = [&](u32 pa) {
        const u32 be = dmaBus->read32(pa);
        return ((be & 0xFFu) << 24) | ((be & 0xFF00u) << 8) |
               ((be >> 8) & 0xFF00u) | (be >> 24);
    };
    auto wr32le = [&](u32 pa, u32 v) {
        dmaBus->write32(pa, ((v & 0xFFu) << 24) | ((v & 0xFF00u) << 8) |
                                ((v >> 8) & 0xFF00u) | (v >> 24));
    };
    for (u32 steps = 0; steps < 4096; ++steps) {
        const u32 w0 = rd32le(ch.cmdPtr);
        const u32 op = w0 >> 28;
        if (op == 7) { // STOP: the list is done, the channel goes idle
            ch.status &= ~0x0400u;
            return;
        }
        // OUTPUT/INPUT/NOP/QUAD ops: data movement is absorbed; report the
        // transfer complete in the descriptor's status word.
        wr32le(ch.cmdPtr + 12, (0x8400u << 16) | 0u);
        ch.cmdPtr += 16;
    }
    // List never reached STOP within bounds: leave ACTIVE standing.
}

void MacIo::setIfr(u8 bits)
{
    ifr_ |= bits & 0x7F;
    picLatch(); // the VIA IRQ level feeds interrupt source 18
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
    case vORAnh: {
        // Port A: output bits echo the latch, input bits are board straps
        // (machine identification). RECEIPT: strap value under empirical
        // pinning — the 68K hardware census reads these to choose its
        // per-board config table.
        const u8 straps = 0xFF;
        return static_cast<u8>((via_[vORA] & via_[vDDRA]) |
                               (straps & ~via_[vDDRA]));
    }
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
                        respDelay_ = 65536; // RECEIPT: MCU think time. The
                                            // original 256 raced the
                                            // interrupt-driven era: the
                                            // reply's TREQ fired before the
                                            // requester registered its wait
                                            // record (null-context resume).
                                            // ~ms-scale latency is also what
                                            // the real 68HC05 exhibits.
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
            // TACK edges are acknowledged by mirroring TREQ. The assert
            // edge answers immediately; the negate edge's SR pulse arrives
            // a little later, like the MCU it models — the host drains the
            // shift register first and then polls IFR for that pulse.
            treq_ = (via_[vORB] & bTACK) == 0;
            updateTreq();
            if (treq_)
                setIfr(iSR);
            else
                syncPulse_ = 64; // deferred completion pulse
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
        picLatch(); // unmasking with a flag pending raises source 18
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
    if (syncPulse_ && --syncPulse_ == 0)
        setIfr(iSR); // the sync-negate completion pulse lands

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
    // ESCC (Z8530), MacRISC layout: ch B ctrl/data +0x13000/+0x13010,
    // ch A ctrl/data +0x13020/+0x13030. RR0 is live STATUS, not a latch —
    // serving the written byte back made OF's console drain read a stale
    // WR command with bit 0 set: an endless phantom-input stream its boot
    // script treated as a break -> abort (pinned empirically). Rx idle +
    // Tx always-empty is the honest quiet-line answer.
    if (off >= 0x13000 && off < 0x13040) {
        const u32 ch = (off >> 5) & 1u; // 0 = B, 1 = A
        if (off & 0x10u)
            return 0; // data: Rx FIFO empty (undefined byte; RECEIPT zeros)
        const u32 p = esccPtr_[ch] & 15u;
        esccPtr_[ch] = 0; // any ctrl access resets the pointer
        switch (p) {
        case 0: return 0x04; // RR0: Tx buffer empty, no Rx char, no breaks
        case 1: return 0x01; // RR1: all sent
        default: return esccWr_[ch][p]; // benign read-back of the WR file
        }
    }
    if (off >= 0x10 && off < 0x30) {
        if (log_.size() < kLogCap || log_.count(off))
            ++log_[off].reads;
        return picRead(off);
    }
    // AWACS/Screamer codec status (+0x14020, LE dword): the revision field
    // in bits 15:12 identifies the codec — 3 = Screamer, which is what the
    // sound init probes for before bringing the driver up. RECEIPT: value
    // pinned to the revision check; the rest of the status reads zero.
    if (off >= 0x14020 && off < 0x14024) {
        const u32 codecStat = 0x00403100u;
        const u8 v = static_cast<u8>(codecStat >> (8 * (off - 0x14020)));
        if ((stamp && *stamp > 100000000ull) && sndTrace_.size() < 512)
            sndTrace_.push_back(
                {*stamp, 0, off, v, pcRef ? *pcRef : 0});
        return v;
    }
    if (off >= 0x8000 && off < 0x9000) {
        const u8 v = dbdmaRead(off);
        if (sndTrace_.size() < 512)
            sndTrace_.push_back(
                {stamp ? *stamp : 0, 0, off, v, pcRef ? *pcRef : 0});
        return v;
    }
    if (off >= 0x14000 && off < 0x15000) // sound codec space
        if (sndTrace_.size() < 512)
            sndTrace_.push_back({stamp ? *stamp : 0, 0, off, store_[off], pcRef ? *pcRef : 0});
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
    if (off >= 0x10 && off < 0x30) {
        if (log_.size() < kLogCap || log_.count(off)) {
            Touch& t = log_[off];
            ++t.writes;
            t.lastWrite = v;
        }
        picWrite(off, v);
        return;
    }
    if (off >= 0x8000 && off < 0x9000) {
        if (sndTrace_.size() < 512)
            sndTrace_.push_back(
                {stamp ? *stamp : 0, 1, off, v, pcRef ? *pcRef : 0});
        dbdmaWrite(off, v);
        return;
    }
    if (off >= 0x13000 && off < 0x13040) {
        const u32 ch = (off >> 5) & 1u;
        if (off & 0x10u)
            return; // Tx data: swallowed, transmitter is always ready
        u32& ptr = esccPtr_[ch];
        if (ptr == 0) {
            ptr = v & 7u;
            if (((v >> 3) & 7u) == 1u)
                ptr |= 8u; // WR0 "point high" command
        } else {
            esccWr_[ch][ptr & 15u] = v;
            ptr = 0;
        }
        return;
    }
    if ((off >= 0x14000 && off < 0x15000) ||
        (off >= 0x8000 && off < 0x9000))
        if (sndTrace_.size() < 512)
            sndTrace_.push_back({stamp ? *stamp : 0, 1, off, v, pcRef ? *pcRef : 0});
    if (log_.size() < kLogCap || log_.count(off)) {
        Touch& t = log_[off];
        ++t.writes;
        t.lastWrite = v;
    }
    store_[off] = v;
}

} // namespace opm
