#include "opm/dbdma.hpp"

#include "opm/bus.hpp"

#include <vector>

namespace opm {

static constexpr u32 kRun = 0x8000, kPause = 0x4000, kFlush = 0x2000,
                     kWake = 0x1000, kDead = 0x0800, kActive = 0x0400;

// File-static so sizeof(DbdmaChannel) — which is in the snapshot layout
// digest — does not move. Only the two audio channels are watched.
static const DbdmaChannel* g_irqWatch[2] = {nullptr, nullptr};
static DbdmaIrqStats g_irqStats[2];

static int irqSlot(const DbdmaChannel* c)
{
    if (c == g_irqWatch[0])
        return 0;
    if (c == g_irqWatch[1])
        return 1;
    return -1;
}

void dbdmaWatchIrq(const DbdmaChannel* out, const DbdmaChannel* in)
{
    g_irqWatch[0] = out;
    g_irqWatch[1] = in;
    g_irqStats[0] = {};
    g_irqStats[1] = {};
}

const DbdmaIrqStats& dbdmaIrqStats(u32 slot) { return g_irqStats[slot & 1u]; }

void dbdmaNotePulse(const DbdmaChannel* ch)
{
    const int s = irqSlot(ch);
    if (s >= 0)
        ++g_irqStats[s].pulsed;
}

void DbdmaChannel::note(u32 kind, u32 a, u32 b)
{
    if (stamp && *stamp < logFrom)
        return;
    if (log.size() >= 4096)
        log.erase(log.begin(), log.begin() + 2048);
    log.push_back({stamp ? *stamp : 0, kind, a, b});
}

u32 DbdmaChannel::read(u32 off, u32 len)
{
    u32 native = 0;
    switch ((off >> 2) & 0x3Fu) {
    case 0: // control reads back as status per the convention
    case 1: native = status_; break;
    case 3: native = cmdPtr_; break;
    case 4: native = intSel_; break;
    case 5: native = brSel_; break;
    case 6: native = waitSel_; break;
    default: break;
    }
    if (len == 4)
        return swap32(native);
    u32 r = 0;
    for (u32 k = 0; k < len; ++k)
        r = (r << 8) | ((native >> (8 * ((off + k) & 3u))) & 0xFFu);
    return r;
}

void DbdmaChannel::write(u32 off, u32 v, u32 len)
{
    u32 native;
    if (len == 4)
        native = swap32(v);
    else {
        // lane-merge against the current register image
        native = read(off & ~3u, 4);
        native = swap32(native);
        for (u32 k = 0; k < len; ++k) {
            const u32 lane = (off + k) & 3u;
            native = (native & ~(0xFFu << (8 * lane))) |
                     (((v >> (8 * (len - 1 - k))) & 0xFFu) << (8 * lane));
        }
    }
    switch ((off >> 2) & 0x3Fu) {
    case 0: { // channelControl: mask/value halves
        const u32 mask = native >> 16, val = native & 0xFFFFu;
        const u32 wasRun = status_ & kRun;
        status_ = (status_ & ~mask) | (val & mask);
        note(0, native, status_);
        // The interrupt is a LEVEL, and until this line existed nothing ever
        // lowered it. On the hardware the condition lives in channelStatus
        // and the driver clears it by writing channelControl, which is what
        // every DBDMA interrupt handler does before it looks at anything
        // else. A level nothing lowers is an interrupt storm the moment a
        // driver enables the source — which is exactly what a sound driver
        // does, and why this could stay wrong for as long as ATA was the
        // only user (its driver reads the condition from the cell's +0x300
        // latch instead).
        if (irq_) {
            const int s = irqSlot(this);
            if (s >= 0)
                ++g_irqStats[s].dropCtl;
        }
        irq_ = false;
        if (status_ & kFlush) {
            status_ &= ~kFlush;
            // ⭐⭐ AND ACTUALLY FLUSH, WHICH THIS NEVER DID. Clearing the bit
            // was the whole of it, while the comment claimed it "drops the
            // standing list state" — it left that state exactly where it was,
            // so a channel parked mid-descriptor stayed RUN|ACTIVE on the same
            // descriptor for ever.
            //
            // ⛔ That hung the Mac OS 9 boot during extension loading. A serial
            // driver programs SCC receive B (+0x8700) with an INPUT_MORE for
            // 768 bytes, gets nothing because no device is attached to that
            // channel, and aborts the pending receive the way DBDMA says to:
            // set FLUSH and wait for the channel to let go. Measured at the
            // hang: 2,467 writes of 0x20002000 to ChannelControl, one every
            // ~26 instructions, against a channel answering RUN|ACTIVE every
            // single time.
            //
            // A flush means "give up what you are holding", so the descriptor
            // in progress ends here — its status word takes the residual it
            // did NOT transfer, and the channel stops being busy on it.
            // Anything else tells a driver its transfer is still coming when
            // nothing is ever going to arrive.
            if ((status_ & kActive) && cmdPtr_ && dmaBus) {
                // ⛔⛔ SNOOP FIRST, exactly as the run loop does before it
                // touches a descriptor. This line went in without it and the
                // omission cost a boot: the status word landed in RAM
                // underneath a LIVE, STALE cache line, so the driver polling
                // that descriptor read xferStatus=0000 for ever while memory
                // held 8400 — a completion the guest could not see. The
                // diagnostic read RAM and reported the descriptor complete,
                // which is precisely the disagreement that named it.
                //
                // It covers the read below as well as the write: the engine
                // must not take a request count out of a line the processor
                // has dirty either.
                dmaBus->snoopBeforeDmaWrite(cmdPtr_, 16);
                const u32 req = swap32(dmaBus->read32(cmdPtr_)) & 0xFFFFu;
                dmaBus->write32(cmdPtr_ + 12,
                                swap32((0x8400u << 16) |
                                       ((req - xferDone_) & 0xFFFFu)));
                xferDone_ = 0;
                status_ &= ~kActive;
                note(3, cmdPtr_, status_);
            }
        }
        if (status_ & kWake) {
            status_ &= ~kWake;
            if (status_ & kRun)
                run();
        }
        if ((status_ & kRun) && !wasRun) {
            status_ &= ~kDead;
            status_ |= kActive;
            xferDone_ = 0; // a fresh list starts at the head of a descriptor
            run();
        }
        if (!(status_ & kRun)) {
            status_ &= ~kActive;
            xferDone_ = 0;
        }
        break;
    }
    case 3:
        // commandPtrLo is ignored while the channel is ACTIVE, and the low
        // four bits are ignored always (descriptors are 16-byte aligned).
        // A driver retargets a running channel by clearing RUN first; a
        // write that lands mid-list would otherwise teleport the engine.
        if (status_ & kActive) {
            note(6, native, status_);
            break;
        }
        cmdPtr_ = native & ~15u;
        break;
    case 4: intSel_ = native; break;
    case 5: brSel_ = native; break;
    case 6: waitSel_ = native; break;
    default: break;
    }
}

void DbdmaChannel::wake()
{
    if ((status_ & kRun) && (status_ & kActive))
        run();
}

void DbdmaChannel::run()
{
    if (!dmaBus)
        return;
    auto rd32le = [&](u32 pa) {
        const u32 be = dmaBus->read32(pa);
        return swap32(be);
    };
    auto wr32le = [&](u32 pa, u32 x) { dmaBus->write32(pa, swap32(x)); };
    // A descriptor list that BRANCHES is a ring, and an audio driver's output
    // list is one: without a bound, a ring whose device has no room would be
    // walked forever inside a single guest instruction. Any descriptor that
    // moves a byte resets the count, so an ATA list — which never branches at
    // all — cannot reach it.
    u32 idle = 0;
    for (u32 steps = 0; steps < 65536; ++steps) {
        if (!(status_ & kRun))
            return;
        // The descriptor is both read (command, address, literal) and
        // written (xferStatus/resCount), so snoop it as a master WRITE:
        // push whatever the processor still holds dirty, then drop its
        // copy. Open Firmware builds this list with ordinary cached stores
        // and arms the channel without a single dcbf — on real hardware
        // the 60x snoop covers it, and without this the engine reads
        // power-on junk and marks itself DEAD.
        dmaBus->snoopBeforeDmaWrite(cmdPtr_, 16);
        const u32 w0 = rd32le(cmdPtr_);
        const u32 op = w0 >> 28;
        const u32 req = w0 & 0xFFFFu;
        const u32 addr = rd32le(cmdPtr_ + 4);
        note(1, cmdPtr_, w0);
        // Park on this descriptor and wait for a wake: a streaming device
        // took part of it and still owes the rest.
        bool stall = false;
        u32 movedNow = 0;
        switch (op) {
        case 7: // STOP
            status_ &= ~kActive;
            xferDone_ = 0;
            note(3, cmdPtr_, 0);
            return;
        case 2:
        case 3: { // INPUT_MORE / INPUT_LAST: device -> memory
            u32 moved = 0;
            const u32 want = req - xferDone_;
            const u32 at = addr + xferDone_;
            if (dev) {
                std::vector<u8> tmp(want);
                moved = dev->dmaTake(tmp.data(), want);
                // The processor's copy of the destination is about to be
                // wrong. Without this the bytes land in RAM underneath a
                // live cache line and the driver reads back whatever it
                // had before the transfer — a DMA that "worked" and
                // delivered nothing.
                dmaBus->snoopBeforeDmaWrite(at, moved);
                for (u32 k = 0; k < moved; ++k)
                    dmaBus->write8(at + k, tmp[k]);
            }
            note(2, at, moved);
            movedNow = moved;
            xferDone_ += moved;
            if (xferDone_ < req) {
                // Device has no more data right now: report what moved,
                // stay on this descriptor and wait for a wake.
                if (xferDone_ == 0)
                    return; // nothing yet — retry on wake
                if (dev && dev->dmaStreams()) {
                    stall = true;
                    break;
                }
                wr32le(cmdPtr_ + 12, (0x8400u << 16) | (req - xferDone_));
            } else {
                wr32le(cmdPtr_ + 12, (0x8400u << 16) | 0u);
            }
            break;
        }
        case 0:
        case 1: { // OUTPUT_MORE / OUTPUT_LAST: memory -> device
            if (dev && dev->dmaWriteSink()) {
                const u32 want = req - xferDone_;
                const u32 at = addr + xferDone_;
                // The source is a buffer the guest built with ordinary
                // cached stores, so push whatever the processor still holds
                // dirty before reading it — the read side of the same
                // coherency truth the descriptor fetch above depends on.
                dmaBus->snoopBeforeDmaRead(at, want);
                std::vector<u8> tmp(want);
                for (u32 k = 0; k < want; ++k)
                    tmp[k] = dmaBus->read8(at + k);
                const u32 moved = dev->dmaGive(tmp.data(), want);
                note(2, at, moved);
                movedNow = moved;
                xferDone_ += moved;
                if (xferDone_ == 0)
                    return; // no write phase open yet — retry on wake
                if (xferDone_ < req && dev->dmaStreams()) {
                    stall = true;
                    break;
                }
                wr32le(cmdPtr_ + 12, (0x8400u << 16) | (req - xferDone_));
                break;
            }
            // An ATAPI device's packet and its payload go through the task
            // file in this machine, so its OUTPUT descriptors carry nothing
            // the drive has not already been handed: absorb and complete.
            wr32le(cmdPtr_ + 12, (0x8400u << 16) | 0u);
            break;
        }
        case 4: { // STORE_QUAD: literal (word2) -> addr, width by req
            const u32 lit = rd32le(cmdPtr_ + 8);
            dmaBus->snoopBeforeDmaWrite(addr, req ? req : 4u);
            note(5, addr, lit);
            if (req >= 4)
                wr32le(addr, lit);
            else if (req == 2)
                dmaBus->write16(addr,
                                static_cast<u16>(((lit & 0xFFu) << 8) |
                                                 ((lit >> 8) & 0xFFu)));
            else
                dmaBus->write8(addr, static_cast<u8>(lit));
            wr32le(cmdPtr_ + 12, (0x8400u << 16) | 0u);
            break;
        }
        case 5: { // LOAD_QUAD: read the quadlet at `address` — a DEVICE
            // REGISTER is a legal target — and deposit it in the
            // descriptor's OWN cmdDep word, where the driver reads it.
            //
            // ⭐ THIS OP IS THE MAC OS SOUND DRIVER'S CLOCK. Its interrupt
            // handler ZEROES the codec's Frame Count register, arms a
            // two-slice program with a LOAD_QUAD of that register before
            // each OUTPUT, and sizes its next mix by the count the engine
            // wrote back — a hardware timestamp of playback progress
            // embedded in the command stream. Absorbing the op answered
            // every cycle with "0 frames have played": the mixer produced
            // one 512-frame slice per fallback kick against ~60 of silence,
            // which the ear knows as sputtering rough garbage in every
            // title while the ROM's polled chime stayed perfect.
            //
            // The copy is BYTE-ORDER-BLIND on purpose: read32/write32 both
            // carry the bus's byte image, so the register's little-endian
            // image lands in the little-endian descriptor exactly as the
            // real engine's quadlet transfer does.
            const u32 size = (req & 4u) ? 4u : (req & 2u) ? 2u : 1u;
            dmaBus->snoopBeforeDmaRead(addr, size);
            u32 img = 0;
            if (size == 4)
                img = dmaBus->read32(addr);
            else if (size == 2)
                img = (dmaBus->read8(addr) << 24) |
                      (dmaBus->read8(addr + 1) << 16);
            else
                img = dmaBus->read8(addr) << 24;
            dmaBus->write32(cmdPtr_ + 8, img);
            note(8, addr, img);
            wr32le(cmdPtr_ + 12, (0x8400u << 16) | 0u);
            break;
        }
        case 6: // NOP
            wr32le(cmdPtr_ + 12, (0x8400u << 16) | 0u);
            break;
        default:
            status_ |= kDead;
            status_ &= ~kActive;
            note(4, cmdPtr_, w0);
            return;
        }
        if (stall) {
            // The device took part of this descriptor and still owes the
            // rest. Stay here; a wake resumes at addr + xferDone_. The
            // status word is deliberately NOT written — the descriptor is
            // not finished, and a driver that read a residual here would be
            // told a transfer ended that has not.
            return;
        }
        xferDone_ = 0;
        // Interrupt control. The descriptor is a LITTLE-ENDIAN struct —
        //   u16 reqCount · u8 cmdBits · u8 cmdKey · u32 address · …
        // so in the assembled word cmdKey is 31:24 (cmd = key>>4) and
        // cmdBits is 23:16, and inside cmdBits w=1:0, b=3:2, i=5:4. The
        // interrupt select therefore sits at word bits 21:20 — NOT at
        // 23:22, which is where the spec's big-endian bit numbering puts
        // it and where reading it costs you the branch field instead.
        // It applies to every command, not only the _LAST forms.
        // (01/10 are conditional on the channel's s0-s7 against intSelect;
        // no list in this machine uses them, so they read as "always".)
        if (((w0 >> 20) & 3u) != 0) {
            irq_ = true;
            const int s = irqSlot(this);
            if (s >= 0)
                ++g_irqStats[s].set;
        }
        // Branch, at word bits 19:18 by the same arithmetic. It is what
        // turns a list into a RING, which is what a continuous audio stream
        // needs and what no ATA list has ever used — the branch target is
        // cmdDep, word2, the same word STORE_QUAD/LOAD_QUAD use as their
        // literal, so those two are excluded rather than trusted.
        const u32 br = (op == 4 || op == 5) ? 0u : ((w0 >> 18) & 3u);
        bool take = br == 3u; // 11 = always
        if (br == 1u || br == 2u) {
            // 01 branch if the channel's s0-s7 match branchSelect, 10 if
            // they do not. Both halves of the select register are used:
            // mask in 23:16, value in 7:0.
            const u32 mask = (brSel_ >> 16) & 0xFFu, val = brSel_ & 0xFFu;
            const bool cond = ((status_ & 0xFFu) & mask) == (val & mask);
            take = br == 1u ? cond : !cond;
        }
        if (take) {
            const u32 target = rd32le(cmdPtr_ + 8) & ~15u;
            note(7, cmdPtr_, target);
            cmdPtr_ = target;
        } else {
            cmdPtr_ += 16;
        }
        if (movedNow)
            idle = 0;
        else if (++idle > 512)
            return; // a ring going round without moving a byte: park
    }
}

} // namespace opm
