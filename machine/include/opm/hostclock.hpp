#pragma once
// Real-time pacing: the guest's timebase, driven by the HOST clock.
//
// WHY THIS EXISTS AS A SEPARATE THING. A Power Mac's timebase runs at 25 MHz —
// the 100 MHz bus over four — and for nine sessions this emulator derived it
// from the instruction count instead, one processor cycle per instruction. That
// is a rate, not a clock: it makes the guest's second a function of how fast
// THIS host happens to be. `--realtime` and `opm_set_realtime` were built to
// fix that and they worked, by topping the timebase up from the wall clock
// whenever the instruction-derived value fell behind.
//
// ⛔ AND THEN THE EMULATOR GOT FAST AND THE TOP-UP STOPPED FIRING. The
// architectural advance is MIPS/4 timebase ticks per host second, which is
// 25 MHz at exactly 100 MIPS; past that the floor is above the target and a
// pacer that can only ADD time has nothing left to do. Measured at the Finder
// desktop, 2026-07-31: 203.6 MIPS, timebase 50.90 MHz, 2.04x real, zero
// top-ups in a 25-second window — and 203.6/4 = 50.9 to three digits, so the
// architectural advance was not part of the clock, it WAS the clock. The
// guest's 60 Hz tick chain ran at 122.5 Hz.
//
// ⭐ SO THE FIX IS NOT A BETTER TOP-UP, IT IS OWNERSHIP. Cpu::insnCycles goes
// to zero and executing instructions stops advancing the timebase at all;
// everything below is then the only thing that moves it. The guest's clock is
// right at any throughput, and it stays right the next time this emulator gets
// faster — which is the property the fix had to have, because the last three
// sessions each made the old bug bigger.
//
// ⚠ IT LIVES IN ONE HEADER FOR THE REASON pace.hpp DOES. The app and the
// measuring tool are two machines assembled from the same parts, and the last
// two times a piece of run-loop wiring was written out twice, one copy was
// missing the hard disk and the other diagnosed a real-time defect from an
// instruction-paced capture. Both front ends call this.

#include "opm/cpu.hpp"

#include <chrono>

namespace opm {

// ⏳ A WAIT THAT IS ACTUALLY AS LONG AS IT WAS ASKED FOR.
//
// ⛔⛔ std::this_thread::sleep_for IS NOT USABLE HERE, AND IT IS WRONG IN BOTH
// DIRECTIONS AT ONCE. Measured on this machine, 200 iterations each:
//
//     sleep_for( 250 us)  ->   0.000 ms      returns without waiting at all
//     sleep_for( 500 us)  ->   0.000 ms
//     sleep_for(1000 us)  ->  15.677 ms      the default scheduler quantum
//     sleep_for(2000 us)  ->  15.566 ms
//     high-res timer, ask 250/500/1000/2000 us -> 0.50/1.03/1.44/2.45 ms
//
// Below a millisecond the portable ask does not wait; at a millisecond it
// waits fifteen and a half. An idle machine here wants to wait about ONE, so
// the portable call would have overshot the guest's next deadline by 15x and
// the emulator would have serviced its USB frames at 64 Hz instead of 1000 —
// which reads as a sluggish pointer, not as a bad sleep.
//
// So: a Windows high-resolution waitable timer (CREATE_WAITABLE_TIMER_HIGH_-
// RESOLUTION, 1803+), nanosleep elsewhere. Both still overshoot — about
// 0.45 ms here — so the overshoot is MEASURED AT RUNTIME and subtracted from
// the next ask, rather than being a constant tuned against one host. If the
// high-resolution timer is unavailable the learned overshoot grows until this
// stops waiting at all, which degrades to the spin it replaced instead of
// wedging the machine on a 15 ms quantum.
class HostWait {
  public:
    HostWait() = default;
    ~HostWait();
    HostWait(const HostWait&) = delete;
    HostWait& operator=(const HostWait&) = delete;

    // Wait for AT MOST `ns`, aiming to come back just before it rather than
    // after. Returns what it actually waited. Waking early is free — the
    // caller is a loop and will ask again — so every choice here is biased
    // that way.
    u64 wait(u64 ns);

    // Instrumentation: a wait that never happens and a wait that never ends
    // look identical from outside the loop.
    u64 waits = 0;        // times it actually waited
    u64 skipped = 0;      // times the deadline was too near to be worth it
    u64 waitedNs = 0;     // total waited
    u64 overshootNs = 0;  // the learned cost of asking, in ns

    // Not worth a system call: two of these fit in one batch of instructions,
    // so the run loop comes back around soon enough on its own.
    static constexpr u64 kMinAskNs = 50000; // 50 us

    // The longest this will ever wait in one go, and it is NOT a pacing
    // constant — nothing about the guest's clock depends on it, because a wait
    // that ends early just sends the caller round the loop again. It is a
    // backstop against a machine with no deadline at all (see the clamp in
    // pace.hpp, where an unbounded deadline would otherwise overflow), and it
    // is the ceiling on how long opm_run can sit inside one call while the
    // shell is waiting to hand it a keystroke. An idle Mac has a USB frame due
    // every millisecond, so in ordinary running this never binds.
    static constexpr u64 kMaxAskNs = 2000000; // 2 ms

  private:
    void* h_ = nullptr; // the timer, created on first use
    bool tried_ = false;
};

// The guest's timebase, anchored to a host time_point and advanced from it.
//
// ⚠ PACE AGAINST A FIXED ORIGIN. The anchor is set once when real time is
// turned on and then left alone; a pacer that re-anchors on every sample is
// integrating its own error, and one that re-anchors when it falls behind was
// tried on 2026-07-31, measured at 0.39x real with 13,320 slips, and reverted.
// The origin moves in exactly one place — a stall long enough that the guest
// cannot be told the truth about it — and that place counts itself.
struct HostPacer {
    std::chrono::steady_clock::time_point base{};
    u64 tbBase = 0;
    u64 slips = 0;

    // 25 MHz = bus/4: one timebase tick every 40 ns.
    static constexpr u64 kNsPerTick = 40;
    // The most time that is ever handed over in one go, and the reason it is
    // 50 ms rather than the 1 ms it used to be.
    //
    // The cap exists so a host stall — this process descheduled, a synchronous
    // disk read inside one emulated instruction — is not delivered to the guest
    // as one enormous jump. What it must NOT do is fire in ordinary running:
    // every time it does, the origin moves and the guest quietly loses that
    // time, which is what the old 1 ms cap did 263 times in a 400 s boot.
    //
    // 1 ms was chosen when a sample happened every 1024 instructions and the
    // top-up was a correction. It is far too tight now that this is the whole
    // clock: an idle machine deliberately waits about a millisecond at a time,
    // so the cap would have fired on nearly every wake and the pacer would have
    // spent the boot forgiving time it had not lost. 50 ms is longer than any
    // scheduling hitch or disk read this emulator produces and shorter than a
    // stall a person would call a freeze.
    static constexpr u64 kCatchupTb = 1250000;

    // How many instructions a run loop may hand to one batch under host
    // pacing. It is not a correctness bound — nothing in the machine can fall
    // due inside a batch once executing stops advancing the clock — it is how
    // STALE a device's idea of "now" is allowed to get, because the timebase
    // stands still for the length of one. At the Finder desktop this machine
    // breaks its own batch every 7,741 instructions anyway (a device access, a
    // write to DEC, the core going to sleep), so 4096 rarely binds; where it
    // does it is ~20 µs of host time at 200 MIPS, far finer than the 1 ms USB
    // frame or the 16.7 ms vertical blank.
    static constexpr u64 kBatchInsns = 4096;

    // Anchor to where the machine's clock ACTUALLY is, not to zero. A resumed
    // or already-running machine is billions of ticks in, and pacing
    // "0 + elapsed" against it leaves the target behind the timebase forever,
    // so the clock never advances at all. Both front ends learned this the
    // same way.
    void anchor(const Cpu& cpu)
    {
        base = std::chrono::steady_clock::now();
        tbBase = cpu.st.tb;
    }

    // Move the guest's clock to where the host says it should be. Returns the
    // timebase delta applied, which is zero when the host clock has not
    // advanced a whole tick since the last call.
    //
    // ⚠ THROUGH Cpu::tick, NOT BY ASSIGNMENT. tick() is what decrements the
    // decrementer and asks for its interrupt, so a pacer that wrote st.tb
    // directly would advance the clock and stop the 60 Hz chain in the same
    // stroke. One tick() of n cycles raises the decrementer at most once
    // however large n is — see the closed form there — so a big delta is a
    // late interrupt, never a burst of them.
    u64 sync(Cpu& cpu)
    {
        const auto now = std::chrono::steady_clock::now();
        const u64 ns = static_cast<u64>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(now - base)
                .count());
        const u64 want = tbBase + ns / kNsPerTick;
        if (want <= cpu.st.tb)
            return 0;
        u64 delta = want - cpu.st.tb;
        if (delta > kCatchupTb) {
            delta = kCatchupTb;
            base = now;
            tbBase = cpu.st.tb + delta;
            ++slips;
        }
        cpu.tick(static_cast<u32>(delta * cpu.cyclesPerTbTick));
        return delta;
    }
};

} // namespace opm
