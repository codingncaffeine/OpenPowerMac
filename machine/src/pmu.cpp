#include "opm/pmu.hpp"

namespace opm {

// 6522 register index from the cell offset (stride 0x200).
static u32 regOf(u32 off) { return (off >> 9) & 0xFu; }

u8 PmuVia::read(u32 off, u64 now)
{
    switch (regOf(off)) {
    case 0: { // ORB/IRB: output bits read back, input bits from the PMU
        u8 in = 0;
        if (ack_)
            in |= 0x08u;
        return static_cast<u8>((orb_ & ddrb_) | (in & ~ddrb_));
    }
    case 1:
    case 15:
        return static_cast<u8>((ora_ & ddra_) | (0x00u & ~ddra_));
    case 2: return ddrb_;
    case 3: return ddra_;
    case 4: return t1cl_;
    case 5: return t1ch_;
    case 6: return t1ll_;
    case 7: return t1lh_;
    case 8: return t2cl_;
    case 9: return t2ch_;
    case 10:
        if (log.size() < 4096)
            log.push_back({now, 'r', sr_});
        return sr_;
    case 11: return acr_;
    case 12: return pcr_;
    case 13: return ifr_;
    default: return ier_;
    }
}

void PmuVia::write(u32 off, u8 v, u64 now)
{
    switch (regOf(off)) {
    case 0: {
        orb_ = v;
        const bool req = (v & 0x10u) != 0;
        if (req != lastReq_) {
            lastReq_ = req;
            if (log.size() < 4096)
                log.push_back({now, 'e', static_cast<u8>(req ? 1 : 0)});
            reqEdge(!req, now); // asserted = line driven low
        }
        break;
    }
    case 1:
    case 15: ora_ = v; break;
    case 2: ddrb_ = v; break;
    case 3: ddra_ = v; break;
    case 4: t1cl_ = v; break;
    case 5: t1ch_ = v; break;
    case 6: t1ll_ = v; break;
    case 7: t1lh_ = v; break;
    case 8: t2cl_ = v; break;
    case 9: t2ch_ = v; break;
    case 10: sr_ = v; break;
    case 11: acr_ = v; break;
    case 12: pcr_ = v; break;
    case 13: ifr_ = static_cast<u8>(ifr_ & ~v); break;
    default:
        if (v & 0x80u)
            ier_ = static_cast<u8>(ier_ | (v & 0x7Fu));
        else
            ier_ = static_cast<u8>(ier_ & ~v);
        break;
    }
}

// Request edges clock one byte each full assert/release cycle. The shift
// direction comes from the ACR shift-register mode: out-modes latch the
// host's SR byte, in-modes load the next reply byte for the host to read.
void PmuVia::reqEdge(bool asserted, u64 now)
{
    const bool shiftOut = (acr_ & 0x10u) != 0; // SR mode bit 4: 1 = out
    if (asserted) {
        if (shiftOut) {
            if (lastDirIn_)
                frame_.clear(); // a receive phase ended: new command frame
            frame_.push_back(sr_);
            if (log.size() < 4096)
                log.push_back({now, frame_.size() == 1 ? 'c' : 'd', sr_});
        } else {
            sr_ = nextReply();
        }
        lastDirIn_ = !shiftOut;
        ack_ = false; // byte taken/ready: ack follows request
    } else {
        ack_ = true;
        if (!shiftOut && reply_.empty() && replyAt_ >= reply_.size()) {
            // receive drained; conversation idles until the next command
        }
        if (shiftOut && !frame_.empty())
            buildReply(); // (re)compute what this frame would answer
    }
}

u8 PmuVia::nextReply()
{
    if (replyAt_ < reply_.size())
        return reply_[replyAt_++];
    return 0x00u;
}

// Frame → reply. Founding placeholder: unknown commands answer a single
// zero byte; specific commands earn real replies as the boot demands
// them (the event log shows exactly what it asked).
void PmuVia::buildReply()
{
    reply_.clear();
    replyAt_ = 0;
    if (frame_.empty())
        return;
    switch (frame_[0]) {
    case 0xEA: // GET_VERSION lineage: a PMU99-class version byte
        reply_ = {0x0C};
        break;
    default:
        reply_ = {0x00};
        break;
    }
}

} // namespace opm
