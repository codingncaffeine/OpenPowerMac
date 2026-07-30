#include "opm/dbdma.hpp"

#include "opm/ata.hpp"
#include "opm/bus.hpp"

namespace opm {

static constexpr u32 kRun = 0x8000, kPause = 0x4000, kFlush = 0x2000,
                     kWake = 0x1000, kDead = 0x0800, kActive = 0x0400;

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
        if (status_ & kFlush) {
            // flush completes immediately: drop the standing list state
            status_ &= ~kFlush;
        }
        if (status_ & kWake) {
            status_ &= ~kWake;
            if (status_ & kRun)
                run();
        }
        if ((status_ & kRun) && !wasRun) {
            status_ &= ~kDead;
            status_ |= kActive;
            run();
        }
        if (!(status_ & kRun))
            status_ &= ~kActive;
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
        switch (op) {
        case 7: // STOP
            status_ &= ~kActive;
            note(3, cmdPtr_, 0);
            return;
        case 2:
        case 3: { // INPUT_MORE / INPUT_LAST: device -> memory
            u32 moved = 0;
            if (ata) {
                std::vector<u8> tmp(req);
                moved = ata->dmaTake(tmp.data(), req);
                // The processor's copy of the destination is about to be
                // wrong. Without this the bytes land in RAM underneath a
                // live cache line and the driver reads back whatever it
                // had before the transfer — a DMA that "worked" and
                // delivered nothing.
                dmaBus->snoopBeforeDmaWrite(addr, moved);
                for (u32 k = 0; k < moved; ++k)
                    dmaBus->write8(addr + k, tmp[k]);
            }
            note(2, addr, moved);
            if (moved < req) {
                // Device has no more data right now: report what moved,
                // stay on this descriptor and wait for a wake.
                if (moved == 0)
                    return; // nothing yet — retry on wake
                wr32le(cmdPtr_ + 12, (0x8400u << 16) | (req - moved));
            } else {
                wr32le(cmdPtr_ + 12, (0x8400u << 16) | 0u);
            }
            break;
        }
        case 0:
        case 1: { // OUTPUT_MORE / OUTPUT_LAST: memory -> device
            if (ata && ata->dmaWriteSink()) {
                // The source is a buffer the guest built with ordinary
                // cached stores, so push whatever the processor still holds
                // dirty before reading it — the read side of the same
                // coherency truth the descriptor fetch above depends on.
                dmaBus->snoopBeforeDmaRead(addr, req);
                std::vector<u8> tmp(req);
                for (u32 k = 0; k < req; ++k)
                    tmp[k] = dmaBus->read8(addr + k);
                const u32 moved = ata->dmaGive(tmp.data(), req);
                note(2, addr, moved);
                if (moved == 0)
                    return; // no write phase open yet — retry on wake
                wr32le(cmdPtr_ + 12, (0x8400u << 16) | (req - moved));
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
        case 5: // LOAD_QUAD: absorbed
        case 6: // NOP
            wr32le(cmdPtr_ + 12, (0x8400u << 16) | 0u);
            break;
        default:
            status_ |= kDead;
            status_ &= ~kActive;
            note(4, cmdPtr_, w0);
            return;
        }
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
        if (((w0 >> 20) & 3u) != 0)
            irq_ = true;
        cmdPtr_ += 16;
    }
}

} // namespace opm
