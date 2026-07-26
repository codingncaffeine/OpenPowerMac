#include "opm/cuda.hpp"

namespace opm {

namespace {

// Packet types on the wire.
inline constexpr u8 kAdb = 0;
inline constexpr u8 kPseudo = 1;
inline constexpr u8 kError = 2;

// Pseudo (Cuda) commands the boot path exercises.
inline constexpr u8 cWarmStart = 0x00;
inline constexpr u8 cAutopoll = 0x01;
inline constexpr u8 cGetTime = 0x03;
inline constexpr u8 cGetPram = 0x07;
inline constexpr u8 cSetTime = 0x09;
inline constexpr u8 cSetPram = 0x0C;
inline constexpr u8 cMsReset = 0x0D;
inline constexpr u8 cResetSystem = 0x11;
inline constexpr u8 cSetAutoRate = 0x14;
inline constexpr u8 cSetDeviceList = 0x19;
inline constexpr u8 cGetDeviceList = 0x1A;
inline constexpr u8 cOneSecond = 0x1B;

// RECEIPT: the RTC base is a fixed instant (2000-01-01 00:00:00 in the
// Mac 1904 epoch) plus whatever SET_TIME establishes — deterministic runs,
// adjustable clock. 2000-01-01 = 3029529600 seconds after 1904-01-01.
inline constexpr u32 kRtcBase = 3029529600u;

} // namespace

void Cuda::hostPacket(const std::vector<u8>& pkt)
{
    if (pkt.empty())
        return;
    const u32 key = (static_cast<u32>(pkt[0]) << 8) |
                    (pkt.size() > 1 ? pkt[1] : 0u);
    ++cmdLog_[key];
    switch (pkt[0]) {
    case kAdb:
        adb(pkt);
        break;
    case kPseudo:
        pseudo(pkt);
        break;
    default:
        reply({kError, 2, pkt[0]}); // bad packet type
        break;
    }
}

void Cuda::pseudo(const std::vector<u8>& pkt)
{
    const u8 cmd = pkt.size() > 1 ? pkt[1] : 0xFF;
    // Replies echo {type, 0, cmd} then carry data. The zero is the flag
    // byte the drivers skip; pinned empirically against the boot ROM.
    std::vector<u8> r{kPseudo, 0, cmd};
    switch (cmd) {
    case cGetTime: {
        const u32 t = kRtcBase + timeOffset_;
        r.push_back(static_cast<u8>(t >> 24));
        r.push_back(static_cast<u8>(t >> 16));
        r.push_back(static_cast<u8>(t >> 8));
        r.push_back(static_cast<u8>(t));
        break;
    }
    case cSetTime:
        if (pkt.size() >= 6) {
            const u32 t = (u32(pkt[2]) << 24) | (u32(pkt[3]) << 16) |
                          (u32(pkt[4]) << 8) | u32(pkt[5]);
            timeOffset_ = t - kRtcBase;
        }
        break;
    case cGetPram: {
        const u8 addr = pkt.size() > 2 ? pkt[2] : 0;
        r.push_back(pram[addr]);
        break;
    }
    case cSetPram: {
        if (pkt.size() > 3)
            pram[pkt[2]] = pkt[3];
        break;
    }
    case cAutopoll:
        autopoll_ = pkt.size() > 2 && pkt[2] != 0;
        break;
    case cWarmStart:
    case cMsReset:
    case cResetSystem:
    case cSetAutoRate:
    case cSetDeviceList:
    case cGetDeviceList:
    case cOneSecond:
    default:
        // Acknowledge unknowns with a bare echo; the command log tells us
        // when something real is being asked for.
        break;
    }
    reply(std::move(r));
}

void Cuda::adb(const std::vector<u8>& pkt)
{
    const u8 cmd = pkt.size() > 1 ? pkt[1] : 0;
    const u8 addr = cmd >> 4;
    const u8 op = (cmd >> 2) & 3; // 0=reset/flush, 2=listen, 3=talk
    const u8 regn = cmd & 3;

    // Stub ADB bus: keyboard at 2, mouse at 3, nothing else. Replies are
    // {0, flags, cmd, data...}; flag bit 0 = timeout (no such device).
    const bool present = addr == 2 || addr == 3;
    if (op == 3) { // talk
        if (!present) {
            reply({kAdb, 0x01, cmd}); // timeout
            return;
        }
        if (regn == 3) {
            // Device handler info: address in the high nibble, handler ID.
            const u8 handler = addr == 2 ? 0x02 : 0x01;
            reply({kAdb, 0, cmd, static_cast<u8>(0x60 | addr), handler});
            return;
        }
        reply({kAdb, 0x01, cmd}); // registers with no data: timeout-style
        return;
    }
    reply({kAdb, 0, cmd}); // listens/flush: plain ack
}

} // namespace opm
