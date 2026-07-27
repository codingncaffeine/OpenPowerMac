#pragma once
#include "opm/types.hpp"

#include <vector>

namespace opm {

// KeyLargo VIA cell + PMU99 behind it (the Sawtooth's system-management
// path: power, ADB, RTC, NVRAM-adjacent services). The VIA face is the
// classic 6522 register file at stride 0x200 across mac-io +0x16000..
// +0x17FFF. The PMU conversation is the port-B REQ/ACK byte handshake
// with data through the shift register:
//
//   port B bit 4 = host request (output, DDRB confirms: ROM writes 0x10)
//   port B bit 3 = PMU ack     (input)
//
// Founding model: the PMU acks every byte (ack follows request with a
// small delay), latches host bytes on request-assert edges, serves reply
// bytes on receive edges, and LOGS everything — the boot ROM's framing
// behavior against this shell is what pins the command set. Replies are
// generated for the commands the boot demands, added as they surface.
class PmuVia {
public:
    u8 read(u32 off, u64 now);        // off: 0..0x1FFF within the cell
    void write(u32 off, u8 v, u64 now);

    struct Ev {
        u64 at;
        char kind; // 'c' cmd, 'd' data byte in, 'r' reply served, 'e' edge
        u8 val;
    };
    std::vector<Ev> log;

    bool ackLevel() const { return ack_; }

    // VIA port A input levels: board-identity straps on real machines
    // (open lines float high through pull-ups; grounded straps encode
    // the board). The boot ROM folds these into product-code/product-id
    // and gates per-board behavior (the f2 slot-names incl. USB!) on it.
    u8 portAIn = 0x00;

    // Timebase pointer: when wired, the VIA timers count TB/32 (the real
    // 7400-side ratio — TB at bus/4 ≈ 25 MHz against the VIA's ~783 kHz
    // phi2), so --fast-tb compresses VIA time and TB time uniformly. The
    // OS calibrates its tick chain against these timers; pacing them per
    // instruction while the TB runs compressed splits the machine into
    // two clocks and stretches every tick-timed wait by the same factor.
    const u64* tbRef = nullptr;

private:
    void reqEdge(bool asserted, u64 now);
    u8 nextReply();

    // VIA registers
    u8 orb_ = 0, ora_ = 0, ddrb_ = 0, ddra_ = 0;
    u8 t1ll_ = 0, t1lh_ = 0, t2cl_ = 0;
    u16 t1Load_ = 0xFFFF, t2Load_ = 0xFFFF;
    u64 t1At_ = 0, t2At_ = 0; // in VIA-tick units (vclk)
    u64 vclk(u64 now) const;
    u16 timerNow(u16 loaded, u64 loadedAtV, u64 now) const;
    u8 sr_ = 0, acr_ = 0, pcr_ = 0, ifr_ = 0, ier_ = 0;

    // PMU engine
    bool ack_ = true;      // idle level: high (ROM polls for set at start)
    bool lastReq_ = true;  // idle high (ORB reset + bit4 as written = 0x10)
    bool lastDirIn_ = false; // direction of the previous byte edge
    std::vector<u8> frame_;   // bytes received since frame start
    std::vector<u8> reply_;   // queued reply bytes
    u32 replyAt_ = 0;
    void buildReply();
};

} // namespace opm
