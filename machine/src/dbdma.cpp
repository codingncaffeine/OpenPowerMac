#include "opm/dbdma.hpp"

#include "opm/ata.hpp"
#include "opm/bus.hpp"

namespace opm {

static constexpr u32 kRun = 0x8000, kPause = 0x4000, kFlush = 0x2000,
                     kWake = 0x1000, kDead = 0x0800, kActive = 0x0400;

void DbdmaChannel::note(u32 kind, u32 a, u32 b)
{
    if (log.size() < 2048)
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
        cmdPtr_ = native;
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
        case 1: { // OUTPUT: memory -> device (CDB/data writes)
            // The ATAPI packet and write data go through the task file
            // in this machine; absorb and complete.
            wr32le(cmdPtr_ + 12, (0x8400u << 16) | 0u);
            break;
        }
        case 4: { // STORE_QUAD: literal (word2) -> addr, width by req
            const u32 lit = rd32le(cmdPtr_ + 8);
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
        // interrupt-select: fire on _LAST ops when the condition field
        // requests it (coarse: any nonzero select interrupts on LAST).
        if ((op == 1 || op == 3) &&
            ((w0 >> 20) & 3u) != 0) // 'i' field: interrupt condition
            irq_ = true;
        cmdPtr_ += 16;
    }
}

} // namespace opm
