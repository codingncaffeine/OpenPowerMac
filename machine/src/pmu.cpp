#include "opm/pmu.hpp"

namespace opm {

// 6522 register index from the cell offset (stride 0x200).
static u32 regOf(u32 off) { return (off >> 9) & 0xFu; }

// The 6522 timers count down at the classic VIA rate (~783 kHz). With
// the timebase wired (tbRef) that is TB/32 — the authentic ratio against
// the 25 MHz timebase, so harness TB compression scales VIA time too.
// Fallback without a TB: one tick every ~128 instructions. The Toolbox
// calibrates its tick chain against T1/T2 — dead or skewed timers stall
// or stretch every timed wait in the OS.
static constexpr u64 kInsnsPerViaTick = 128;

u64 PmuVia::vclk(u64 now) const
{
    return tbRef ? (*tbRef >> 5) : (now / kInsnsPerViaTick);
}

u16 PmuVia::timerNow(u16 loaded, u64 loadedAtV, u64 now) const
{
    const u64 ticks = vclk(now) - loadedAtV;
    return static_cast<u16>(loaded - static_cast<u16>(ticks));
}

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
        return static_cast<u8>((ora_ & ddra_) | (portAIn & ~ddra_));
    case 2: return ddrb_;
    case 3: return ddra_;
    case 4:
        ifr_ = static_cast<u8>(ifr_ & ~0x40u); // T1 low read clears T1 int
        return static_cast<u8>(timerNow(t1Load_, t1At_, now));
    case 5: return static_cast<u8>(timerNow(t1Load_, t1At_, now) >> 8);
    case 6: return t1ll_;
    case 7: return t1lh_;
    case 8:
        ifr_ = static_cast<u8>(ifr_ & ~0x20u); // T2 low read clears T2 int
        return static_cast<u8>(timerNow(t2Load_, t2At_, now));
    case 9: return static_cast<u8>(timerNow(t2Load_, t2At_, now) >> 8);
    case 10:
        if (log.size() < 65536)
            log.push_back({now, 'r', sr_});
        return sr_;
    case 11: return acr_;
    case 12: return pcr_;
    case 13: {
        // expiry flags computed from the free-running counts
        u8 f = ifr_;
        if (vclk(now) - t1At_ >= u64(t1Load_) + 1)
            f |= 0x40u;
        if (vclk(now) - t2At_ >= u64(t2Load_) + 1)
            f |= 0x20u;
        if (f & ier_ & 0x7Fu)
            f |= 0x80u;
        return f;
    }
    default: return ier_;
    }
}

void PmuVia::write(u32 off, u8 v, u64 now, PmuHost* host)
{
    switch (regOf(off)) {
    case 0: {
        orb_ = v;
        const bool req = (v & 0x10u) != 0;
        if (req != lastReq_) {
            lastReq_ = req;
            if (log.size() < 65536)
                log.push_back({now, 'e', static_cast<u8>(req ? 1 : 0)});
            reqEdge(!req, now, host); // asserted = line driven low
        }
        break;
    }
    case 1:
    case 15: ora_ = v; break;
    case 2: ddrb_ = v; break;
    case 3: ddra_ = v; break;
    case 4: t1ll_ = v; break; // T1CL write = latch low
    case 5: // T1CH write loads and starts T1
        t1Load_ = static_cast<u16>((u16(v) << 8) | t1ll_);
        t1At_ = vclk(now);
        ifr_ = static_cast<u8>(ifr_ & ~0x40u);
        break;
    case 6: t1ll_ = v; break;
    case 7: t1lh_ = v; break;
    case 8: t2cl_ = v; break; // latch low
    case 9: // T2CH write loads and starts T2
        t2Load_ = static_cast<u16>((u16(v) << 8) | t2cl_);
        t2At_ = vclk(now);
        ifr_ = static_cast<u8>(ifr_ & ~0x20u);
        break;
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
void PmuVia::reqEdge(bool asserted, u64 now, PmuHost* host)
{
    const bool shiftOut = (acr_ & 0x10u) != 0; // SR mode bit 4: 1 = out
    if (asserted) {
        if (shiftOut) {
            if (lastDirIn_)
                frame_.clear(); // a receive phase ended: new command frame
            frame_.push_back(sr_);
            // PMU_RESET is acted on when the COMMAND BYTE lands, not when a
            // reply is built: `reset-all` sends it and never reads back,
            // so a handler in buildReply() is never reached.
            if (frame_.size() == 1 && sr_ == 0xD0u)
                resetRequest = true;
            if (log.size() < 65536)
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
            buildReply(now, host); // (re)compute what this frame would answer
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
void PmuVia::buildReply(u64 now, PmuHost* host)
{
    reply_.clear();
    replyAt_ = 0;
    if (frame_.empty())
        return;
    // The clock is read in timebase, which the bus keeps current through
    // tbRef; `now` is the instruction stamp and only stands in without one.
    const u64 tb = tbRef ? *tbRef : now;
    switch (frame_[0]) {
    case 0x38: // READ_RTC: four bytes, seconds since 1904, high byte first
        if (host && host->rtcSet) {
            const u32 t = host->rtcNow(tb);
            reply_ = {static_cast<u8>(t >> 24), static_cast<u8>(t >> 16),
                      static_cast<u8>(t >> 8), static_cast<u8>(t)};
        } else {
            reply_ = {0x00}; // no clock set: see PmuHost
        }
        break;
    case 0x30: // SET_RTC: the same four bytes, from the guest
        // This runs after every byte of the frame; the fifth completes it.
        // ⚠ AND ENDS IT. The command has no reply, so no receive phase
        // follows to mark the frame over, and the next command's byte would
        // be taken for a sixth data byte of this one — a READ_RTC straight
        // after a set answered nothing. The length is the command's own:
        // four bytes in, none out.
        if (frame_.size() == 5) {
            if (host)
                host->rtcSetTo((u32(frame_[1]) << 24) |
                                   (u32(frame_[2]) << 16) |
                                   (u32(frame_[3]) << 8) | u32(frame_[4]),
                               tb);
            frame_.clear();
            return;
        }
        reply_ = {0x00};
        break;
    case 0xEA: // GET_VERSION lineage: a PMU99-class version byte
        reply_ = {0x0C};
        break;
    case 0xDF:
        // Observed framing: DF 03 00 <data> <clock> cycling through the
        // four line combinations — a 2-wire bus bit-banged through the
        // PMU. With no slave modeled yet, the lines read back as driven
        // (open bus, pull-ups): echo them.
        if (frame_.size() >= 5)
            reply_ = {frame_[3], frame_[4]};
        else
            reply_ = {0x00, 0x00};
        break;
    default:
        reply_ = {0x00};
        break;
    }
}

} // namespace opm
