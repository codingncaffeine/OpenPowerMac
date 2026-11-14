// The guest's clock under host pacing (HostPacer::sync): it follows the host's,
// and a stretch of host time in which no guest instruction ran is handed over
// as the guest executes, not as one jump.
//
// The failure this pins: the card's command engine draws a frame's triangles
// inside one register write, about 10 ms of host time. Handed over in one
// step, that is a processor that stopped while the sound hardware played on;
// the driver's two 11.6 ms slices both ended before its interrupt handler ran
// once, and a fifth of a game's sound came out as silence.

#include "doctest.h"
#include "opm/hostclock.hpp"

#include <chrono>

using namespace opm;

namespace {

using Clock = std::chrono::steady_clock;

// A processor with its clock where a pacer can own it, and a pacer anchored
// at a moment the test controls.
struct Rig {
    Cpu cpu;
    HostPacer pacer;
    Clock::time_point t0;
    u64 executed = 1000000;

    Rig()
    {
        cpu.insnCycles = 0; // the host owns the timebase
        cpu.st.tb = 5000000;
        pacer.anchor(cpu);
        t0 = pacer.base;
        // The first sync after an anchor has no instruction count to measure
        // from; take it here so every case starts with one.
        pacer.syncAt(cpu, t0, executed, 0);
    }
    Clock::time_point at(u64 ns) const
    {
        return t0 + std::chrono::nanoseconds(ns);
    }
    u64 tb() const { return cpu.st.tb - 5000000; }
};

constexpr u64 kMs = 1000000;        // ns
constexpr u64 kTbPerMs = 25000;     // 25 MHz

} // namespace

TEST_CASE("pacer: a machine that keeps up has the host's clock")
{
    Rig r;
    // 4096 instructions every 40 us is 102 MIPS: each step is 1000 ticks and
    // the allowance is four times that.
    for (u64 i = 1; i <= 500; ++i) {
        r.executed += 4096;
        r.pacer.syncAt(r.cpu, r.at(i * 40000), r.executed, 0);
        REQUIRE(r.tb() == i * 1000);
    }
    CHECK(r.pacer.held == 0);
    CHECK(r.pacer.slips == 0);
    CHECK(r.pacer.forgivenTb == 0);
}

TEST_CASE("pacer: host time with no instructions is repaid as they run")
{
    Rig r;
    // 10 ms pass inside one instruction (a device doing its work).
    r.executed += 1;
    const u64 first = r.pacer.syncAt(r.cpu, r.at(10 * kMs), r.executed, 0);
    CHECK(first == 1); // one instruction earned one tick, not 250,000
    CHECK(r.pacer.held == 1);

    // The guest then runs at 102 MIPS. No step may outrun the instructions
    // executed since the one before, and the clock must reach the host's.
    u64 ns = 10 * kMs, steps = 0;
    for (;;) {
        ns += 40000;
        r.executed += 4096;
        const u64 d = r.pacer.syncAt(r.cpu, r.at(ns), r.executed, 0);
        REQUIRE(d <= 4096 * HostPacer::kTbPerInsn);
        ++steps;
        if (r.tb() == ns / HostPacer::kNsPerTick)
            break;
        REQUIRE(steps < 1000);
    }
    // 250,000 ticks of debt at a net 3,096 a step.
    CHECK(steps > 70);
    CHECK(steps < 90);
    CHECK(r.pacer.slips == 0);
    CHECK(r.pacer.forgivenTb == 0);
    // The furthest behind it was found: the gap, plus the one step of host
    // time that went by before the second look.
    CHECK(r.pacer.maxDebtTb == 10 * kTbPerMs - 1 + 1000);
}

TEST_CASE("pacer: an asleep core goes straight to its next deadline")
{
    Rig r;
    // Nothing executed, 10 ms gone, the next deadline 1 ms of clock away.
    const u64 due = r.cpu.st.tb + kTbPerMs;
    CHECK(r.pacer.syncAt(r.cpu, r.at(10 * kMs), r.executed, due) == kTbPerMs);
    CHECK(r.cpu.st.tb == due);
    // ...and to the one after, without an instruction in between.
    CHECK(r.pacer.syncAt(r.cpu, r.at(10 * kMs), r.executed, due + 3 * kTbPerMs) ==
          3 * kTbPerMs);
    // No deadline at all: the whole of what is left.
    CHECK(r.pacer.syncAt(r.cpu, r.at(10 * kMs), r.executed, ~0ull) ==
          6 * kTbPerMs);
    CHECK(r.tb() == 10 * kTbPerMs);
    CHECK(r.pacer.forgivenTb == 0);
}

TEST_CASE("pacer: a deadline already reached does not stop the clock")
{
    Rig r;
    // Asleep, the deadline is now, nothing ran: the clock still moves.
    const u64 before = r.cpu.st.tb;
    CHECK(r.pacer.syncAt(r.cpu, r.at(kMs), r.executed, before) ==
          HostPacer::kMinStepTb);
    CHECK(r.cpu.st.tb > before);
}

TEST_CASE("pacer: what the guest cannot repay is forgiven, and counted")
{
    Rig r;
    // 200 ms with one instruction: 50 ms may stay owed, the rest is lost.
    r.executed += 1;
    r.pacer.syncAt(r.cpu, r.at(200 * kMs), r.executed, 0);
    CHECK(r.pacer.slips == 1);
    CHECK(r.pacer.forgivenTb == 200 * kTbPerMs - 1 - HostPacer::kCatchupTb);
    // The 50 ms is then repaid like any other debt.
    u64 ns = 200 * kMs;
    for (int i = 0; i < 2000 && r.pacer.held < 300; ++i) {
        ns += 40000;
        r.executed += 4096;
        r.pacer.syncAt(r.cpu, r.at(ns), r.executed, 0);
    }
    CHECK(r.pacer.slips == 1); // nothing more was lost on the way
}

TEST_CASE("pacer: the control hands the gap over in one step")
{
    Rig r;
    r.pacer.smooth = false;
    r.executed += 1;
    CHECK(r.pacer.syncAt(r.cpu, r.at(10 * kMs), r.executed, 0) == 10 * kTbPerMs);
    CHECK(r.pacer.held == 0);
    // ...and a gap past the cap in one step of the cap, the rest forgiven.
    r.executed += 1;
    CHECK(r.pacer.syncAt(r.cpu, r.at(210 * kMs), r.executed, 0) ==
          HostPacer::kCatchupTb);
    CHECK(r.pacer.slips == 1);
    CHECK(r.pacer.forgivenTb == 200 * kTbPerMs - HostPacer::kCatchupTb);
}

TEST_CASE("pacer: a clock the guest set is paced from where it was put")
{
    Rig r;
    r.cpu.st.tb += 12345678; // mtspr TBL/TBU
    r.executed += 4096;
    CHECK(r.pacer.syncAt(r.cpu, r.at(kMs), r.executed, 0) == 0);
    CHECK(r.pacer.rebases == 1);
    const u64 from = r.cpu.st.tb;
    r.executed += 4096;
    r.pacer.syncAt(r.cpu, r.at(kMs + 40000), r.executed, 0);
    CHECK(r.cpu.st.tb == from + 1000);
}
