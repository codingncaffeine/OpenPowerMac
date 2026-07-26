#pragma once
// Cuda MCU, protocol-level HLE (Arc 2 M2). Transport-agnostic packet engine:
// the VIA layer hands it complete host packets and pulls response bytes.
//
// Sources: Apple developer notes describe the Cuda's role; the byte protocol
// is tier-4 behavioral knowledge (documented across decades of driver
// implementations — read, never copied) pinned empirically against the boot
// ROM, which is the consumer that matters. Every command is logged so the
// ROM itself tells us what to implement next.

#include "opm/types.hpp"

#include <cstdio>
#include <map>
#include <vector>

namespace opm {

class Cuda {
public:
    // A complete packet from the host (type byte first).
    void hostPacket(const std::vector<u8>& pkt);

    bool hasResponse() const { return !resp_.empty(); }
    const std::vector<u8>& response() const { return resp_; }
    void consumeResponse() { resp_.clear(); }

    // Command census for the boot trace.
    const std::map<u32, u64>& commandLog() const { return cmdLog_; }

    u8 pram[256] = {};

private:
    void pseudo(const std::vector<u8>& pkt);
    void adb(const std::vector<u8>& pkt);
    void reply(std::vector<u8> r) { resp_ = std::move(r); }

    std::vector<u8> resp_;
    std::map<u32, u64> cmdLog_;
    bool autopoll_ = false;
    u32 timeOffset_ = 0; // SET_TIME delta against the fixed base
};

} // namespace opm
