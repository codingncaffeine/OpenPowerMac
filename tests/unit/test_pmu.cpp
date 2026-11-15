// The PMU's clock chip, spoken to the way the guest speaks to it: one byte
// per request/acknowledge cycle through the VIA's shift register.
//
// READ_RTC (0x38) answers four bytes, seconds since midnight on 1 January
// 1904, high byte first; SET_RTC (0x30) takes the same four. The chip itself
// is the board's PmuHost — see opm/pmu.hpp for why it is not in the PMU.

#include "doctest.h"
#include "opm/pmu.hpp"
#include "opm/sawtooth.hpp"

#include <memory>
#include <vector>

using namespace opm;

namespace {

constexpr u32 kOrb = 0u * 0x200u, kSr = 10u * 0x200u, kAcr = 11u * 0x200u;

struct Rig {
    PmuVia pmu;
    PmuHost host;
    u64 tb = 0;
    Rig() { pmu.tbRef = &tb; }

    // One byte to the PMU: shift register in output mode, the byte, then
    // the request line down and up.
    void send(u8 v, PmuHost* h)
    {
        pmu.write(kAcr, 0x1C, 0, h);
        pmu.write(kSr, v, 0, h);
        pmu.write(kOrb, 0x00, 0, h);
        pmu.write(kOrb, 0x10, 0, h);
    }
    // One byte from it: input mode, request down, read, request up.
    u8 recv(PmuHost* h)
    {
        pmu.write(kAcr, 0x0C, 0, h);
        pmu.write(kOrb, 0x00, 0, h);
        const u8 v = pmu.read(kSr, 0);
        pmu.write(kOrb, 0x10, 0, h);
        return v;
    }
    u32 readRtc(PmuHost* h)
    {
        send(0x38, h);
        u32 v = 0;
        for (int k = 0; k < 4; ++k)
            v = (v << 8) | recv(h);
        return v;
    }
};

} // namespace

TEST_CASE("pmu clock: READ_RTC answers the seconds it was set to, and counts")
{
    Rig r;
    r.host.rtcSetTo(0xE71D2A00u, 1000);
    r.tb = 1000;
    CHECK(r.readRtc(&r.host) == 0xE71D2A00u);
    // Ninety seconds of a 25 MHz timebase later, and one tick short of the
    // ninety-first.
    r.tb = 1000 + 25000000ull * 91 - 1;
    CHECK(r.readRtc(&r.host) == 0xE71D2A00u + 90u);
    r.tb += 1;
    CHECK(r.readRtc(&r.host) == 0xE71D2A00u + 91u);
}

TEST_CASE("pmu clock: a chip that was never set answers zero")
{
    // The founding model's answer, which every recorded run was made with:
    // the guest starts up on 1 January 1904.
    Rig r;
    r.tb = 25000000ull * 500;
    CHECK(r.readRtc(&r.host) == 0u);
    CHECK(!r.host.rtcSet);
    // …and so does a PMU with no chip behind it at all.
    Rig bare;
    CHECK(bare.readRtc(nullptr) == 0u);
}

TEST_CASE("pmu clock: SET_RTC from the guest sets the chip")
{
    Rig r;
    r.tb = 25000000ull * 7;
    r.send(0x30, &r.host);
    for (const u8 b : {u8(0xB2), u8(0x4C), u8(0x10), u8(0x05)})
        r.send(b, &r.host);
    CHECK(r.host.rtcSet);
    CHECK(r.readRtc(&r.host) == 0xB24C1005u);
    r.tb += 25000000ull * 3600;
    CHECK(r.readRtc(&r.host) == 0xB24C1005u + 3600u);
    // A set that stopped a byte short changed nothing.
    Rig part;
    part.send(0x30, &part.host);
    for (const u8 b : {u8(1), u8(2), u8(3)})
        part.send(b, &part.host);
    CHECK(!part.host.rtcSet);
}

TEST_CASE("pmu clock: the chip keeps time through a system reset")
{
    // A reset is a new board, and the timebase the clock is read against
    // starts again at zero on it. The chip does not.
    std::vector<u8> rom(SawtoothBus::kRomSize, 0);
    auto bus = std::make_unique<SawtoothBus>(8u << 20, rom);
    bus->pmuHost().rtcSetTo(0xE71D2A00u, 0);
    bus->serviceDevices(25000000ull * 120); // two minutes in
    auto next = SawtoothBus::afterSystemReset(std::move(bus));
    REQUIRE(next);
    CHECK(next->pmuHost().rtcSet);
    CHECK(next->pmuHost().rtcNow(0) == 0xE71D2A00u + 120u);
    CHECK(next->pmuHost().rtcNow(25000000ull * 5) == 0xE71D2A00u + 125u);
}

// ── Framing ──────────────────────────────────────────────────────────────
//
// A command the PMU answers is closed by the answer being read. A command it
// does not answer is closed by its own last byte, and the PMU has to know
// where that is, or the command that follows is taken for more data.

TEST_CASE("pmu framing: a command with no answer ends with its last byte")
{
    Rig r;
    r.host.rtcSetTo(0x12345678u, 0);
    // Twenty bytes of parameter RAM, written and not answered…
    r.send(0x31, &r.host);
    for (int k = 0; k < 20; ++k)
        r.send(static_cast<u8>(0xA0 + k), &r.host);
    // …and the next byte is a command again: the clock answers.
    CHECK(r.readRtc(&r.host) == 0x12345678u);
    // The same with a count byte in front of the data (0x20 takes one).
    r.send(0x20, &r.host);
    r.send(0x02, &r.host);
    r.send(0x38, &r.host); // data, though it has a command's value
    r.send(0x7E, &r.host); // and so is this
    CHECK(!r.host.powerOff);
    CHECK(r.readRtc(&r.host) == 0x12345678u);
    // A command of one byte, no data and no answer.
    r.send(0x21, &r.host);
    CHECK(r.readRtc(&r.host) == 0x12345678u);
}

TEST_CASE("pmu framing: the lengths of the commands a guest has been seen to "
          "send")
{
    CHECK(pmuCommandLen(0x30).send == 4);
    CHECK(pmuCommandLen(0x30).reply == 0);
    CHECK(pmuCommandLen(0x31).send == 20);
    CHECK(pmuCommandLen(0x31).reply == 0);
    CHECK(pmuCommandLen(0x38).send == 0);
    CHECK(pmuCommandLen(0x38).reply == 4);
    CHECK(pmuCommandLen(0x39).reply == 20);
    CHECK(pmuCommandLen(0x7E).send == 4);
    CHECK(pmuCommandLen(0x7E).reply == 1);
    CHECK(pmuCommandLen(0xD0).send == 0);
    CHECK(pmuCommandLen(0xD0).reply == 0);
    // The regular part of the table: counted data, and a counted answer in
    // the upper half of each row of sixteen.
    CHECK(pmuCommandLen(0x04).send == -1);
    CHECK(pmuCommandLen(0x04).reply == 0);
    CHECK(pmuCommandLen(0xBA).send == -1);
    CHECK(pmuCommandLen(0xBA).reply == -1);
}

TEST_CASE("pmu power: the power-off command, with its signature, turns the "
          "machine off")
{
    // As Mac OS sends it at Shut Down: the parameter RAM first, read and
    // written back, then the power-off.
    Rig r;
    r.send(0x39, &r.host);
    for (int k = 0; k < 20; ++k)
        r.recv(&r.host);
    r.send(0x31, &r.host);
    for (int k = 0; k < 20; ++k)
        r.send(0x00, &r.host);
    CHECK(!r.host.powerOff);
    r.send(0x7E, &r.host);
    for (const char ch : {'M', 'A', 'T'})
        r.send(static_cast<u8>(ch), &r.host);
    CHECK(!r.host.powerOff); // not until the signature is whole
    r.send(static_cast<u8>('T'), &r.host);
    CHECK(r.host.powerOff);

    // Without the signature it is not a power-off.
    Rig wrong;
    wrong.send(0x7E, &wrong.host);
    for (const char ch : {'M', 'A', 'T', 'S'})
        wrong.send(static_cast<u8>(ch), &wrong.host);
    CHECK(!wrong.host.powerOff);

    // A PMU with nothing behind it takes the command and nothing happens.
    Rig bare;
    bare.send(0x7E, nullptr);
    for (const char ch : {'M', 'A', 'T', 'T'})
        bare.send(static_cast<u8>(ch), nullptr);
    CHECK(!bare.host.powerOff);
}
