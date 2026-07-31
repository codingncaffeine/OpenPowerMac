#pragma once
// How the machine advances when the processor has nothing to do.
//
// WHY THIS EXISTS. Measured at the Finder desktop, 300 M instructions:
// 192,528,873 of them — 64% — executed NO instruction at all. Mac OS puts the
// core to sleep (MSR[POW]) whenever it is idle, and Cpu::step's nap arm ticks
// the clock and returns. The emulator was spending most of a host core
// stepping one asleep instruction at a time, and the profiler charged the
// whole of it to "other", where it sat unexplained for two sessions.
//
// WHY IT IS EXACT AND NOT AN APPROXIMATION. Nothing can wake a sleeping core
// except an enabled interrupt, and there are only four sources:
//   - the decrementer   — set by Cpu::tick, so Cpu::stepsUntilDec bounds it;
//   - an external line  — set only when the machine's devices are serviced,
//                         and SawtoothBus publishes conservative lower bounds
//                         on when a device could next do anything;
//   - SMI and the performance monitor — neither can change while no
//                         instruction is executing.
// Stopping at the nearest of those bounds leaves the machine in exactly the
// state the one-at-a-time loop would have produced: same timebase, same
// decrementer, same pending flags, same instruction count. It is a skip, not
// a shortcut.
//
// ⚠ IT LIVES HERE, IN ONE HEADER, BECAUSE THE APP AND THE MEASURING TOOL ARE
// TWO DIFFERENT MACHINES ASSEMBLED FROM THE SAME PARTS. The last time a piece
// of run-loop wiring was written out twice, the copy in the C API was missing
// the hard disk and the app had no working disk for weeks while every g4run
// boot passed. One definition, called from both.

#include "opm/cpu.hpp"
#include "opm/hostclock.hpp"
#include "opm/prof.hpp"
#include "opm/sawtooth.hpp"

namespace opm {

// The slow half must stay OUT of the caller, or the compiler folds it back in
// and the per-instruction question is expensive again. Measured in the
// firmware era, where the core never sleeps and the answer is always no:
// 59.0 MIPS with the whole thing written as one function MSVC declined to
// inline, 61.4 split like this, 63.7 with --no-nap-skip.
//
// ⚠ SO THE QUESTION IS NOT FREE: about 4% where nothing ever sleeps, against
// 3.1 s -> 2.3 s for the same window where things do. That trade is worth
// taking — the firmware era is fifteen seconds of a boot and the desktop is
// where the machine actually lives — but it is a trade, and --no-nap-skip is
// how the next reader checks it rather than trusting this comment.
#if defined(_MSC_VER)
#define OPM_PACE_COLD __declspec(noinline)
#else
#define OPM_PACE_COLD __attribute__((noinline))
#endif

// ⚠ INSTRUCTION-PACED LOOPS ONLY. This advances the timebase itself, so a
// caller whose timebase already comes from the host clock (--realtime,
// opm_set_realtime) must not use it — the clock would advance twice. An idle
// machine under real-time pacing wants to SLEEP to the earliest deadline
// instead. That is now BUILDABLE — every device deadline is in timebase, so
// "when could anything happen" converts straight to host nanoseconds — and it
// is not built: nothing in this file yields the host processor.
//
// Steps the caller may charge to its instruction count WITHOUT calling
// Cpu::step,
// because the core is asleep and provably cannot wake inside them. Returns 0
// when the core is awake, when a wake is already pending, or when there is no
// room to skip — in which case the caller just steps normally.
//
// `budget` is how many steps the caller still wants to run at all.
// ⚠ SPLIT, AND THE SPLIT IS MEASURED. As one function this is too big for
// MSVC to inline, so the machine paid a call with two pointer arguments on
// every instruction — 61.9 -> 59.0 MIPS in the firmware era, where the core
// never sleeps at all and the answer is always "no". The question asked per
// instruction has to be one load and one branch; everything else belongs
// behind it.
OPM_PACE_COLD inline u64 napSkipSlow(Cpu& cpu, const SawtoothBus& bus,
                                     u64 budget)
{
    OPM_MARK(Loop);
    if (budget < 2)
        return 0;
    // An enabled interrupt already waiting: step() must run and take it.
    if ((cpu.st.msr & msr::EE) &&
        (cpu.smiPending || cpu.extIrqLine || cpu.decPending || cpu.pmPending))
        return 0;
    const u32 cyclesPerStep = cpu.insnCycles + cpu.extraCycles;
    // Zero is the host clock owning the timebase, and this function ADVANCES
    // the timebase — the two cannot both be true. The callers know that and
    // exclude it, but the header refuses as well: a skip that charged asleep
    // steps to a host-paced clock would advance it twice, and it would read as
    // a pacing bug rather than as a wiring one. hostIdleWait is the equivalent
    // over there — it waits instead of skipping, because there IS nothing to
    // skip when time comes from outside.
    if (!cyclesPerStep)
        return 0;
    u64 n = budget;
    const u64 dec = cpu.stepsUntilDec(cyclesPerStep);
    if (dec < n)
        n = dec;
    // The device deadline, in steps. It is a timebase value and converts
    // through the same cycles-per-step the clock advances at. There used to be
    // a second one in instructions, for ATA commands; a machine has one clock.
    const u64 dueTb = bus.deviceDueTb();
    if (dueTb > cpu.st.tb) {
        const u64 cycles = (dueTb - cpu.st.tb) * cpu.cyclesPerTbTick;
        const u64 s = cycles / cyclesPerStep; // floor: never overshoot
        if (s < n)
            n = s;
    } else {
        return 0;
    }
    if (n < 2)
        return 0;
    cpu.clockAdvance(n, cyclesPerStep);
    cpu.napSkipped += n;
    return n;
}

inline u64 napSkip(Cpu& cpu, const SawtoothBus& bus, u64 budget)
{
    if (!cpu.napping)
        return 0;
    return napSkipSlow(cpu, bus, budget);
}

// ---- the same question, for a machine whose clock comes from outside -------
//
// The nap skip's answer is "charge those steps and move on", because there the
// timebase is made of steps: skipping them IS letting time pass. Under
// host-clock pacing time passes whether or not this process does anything, so
// the answer is the opposite one — WAIT, and let the host's clock carry the
// guest's forward while this thread is off the processor.
//
// ⚠ THIS IS THE THIRD FACE OF THE PACING BUG, and it is the one nobody was
// looking at: the app burned a whole core while the guest sat idle at the
// Finder, because nothing in this file ever yielded. It reads as a D7 shell
// problem and it is not; it is this loop.
//
// ⛔ IT IS NOT "SLEEP WHEN AHEAD", WHICH WAS TRIED AND REVERTED. That threw
// away time the guest had already earned, fought the pacer's own catch-up, and
// measured 0.39x real with 13,320 slips. This waits only when the machine has
// NOTHING TO DO — the core is asleep and no enabled interrupt is waiting — and
// only up to the first moment something could happen. The guest cannot tell
// the difference between this and a processor that was simply idle, which is
// what it is modelling.
//
// Returns the nanoseconds waited (0 when the machine had work, or when the
// deadline was too near to be worth a system call).
OPM_PACE_COLD inline u64 hostIdleWaitSlow(const Cpu& cpu,
                                          const SawtoothBus& bus,
                                          HostWait& w)
{
    OPM_MARK(Loop);
    // An enabled interrupt is already waiting: the machine has work this
    // instant, whatever its core is doing.
    if ((cpu.st.msr & msr::EE) &&
        (cpu.smiPending || cpu.extIrqLine || cpu.decPending || cpu.pmPending))
        return 0;
    // The two deadlines, in the one unit that survives the clock coming from
    // outside. Nothing else can wake a sleeping core — see the note at the top
    // of this file, which enumerates the four sources and disposes of the
    // other two.
    u64 dueTb = bus.deviceDueTb();
    const u64 dec = cpu.tbUntilDec();
    if (dec != ~0ull && cpu.st.tb + dec < dueTb)
        dueTb = cpu.st.tb + dec;
    if (dueTb <= cpu.st.tb)
        return 0; // already due: service it, do not wait on it
    // ⚠ CLAMPED, FOR TWO UNRELATED REASONS THAT MEET AT THE SAME LINE. A
    // machine with nothing due at all leaves dueTb at ~0, and the multiply
    // would overflow into a wait measured in centuries; and this runs INSIDE
    // opm_run, which the shell needs back every few milliseconds to pump the
    // keyboard, the mouse and the audio drain. The clamp costs nothing when it
    // does not bind — waking early is free, because the caller is a loop and
    // simply asks again — so it is compared in TIMEBASE, before the multiply
    // that would have overflowed.
    const u64 tbLeft = dueTb - cpu.st.tb;
    return w.wait(tbLeft > HostWait::kMaxAskNs / HostPacer::kNsPerTick
                      ? HostWait::kMaxAskNs
                      : tbLeft * HostPacer::kNsPerTick);
}

inline u64 hostIdleWait(const Cpu& cpu, const SawtoothBus& bus, HostWait& w)
{
    if (!cpu.napping)
        return 0;
    return hostIdleWaitSlow(cpu, bus, w);
}

// ---- the batch ------------------------------------------------------------
//
// The same reasoning as the nap skip, for a processor that is AWAKE. There the
// question was "how many steps can I skip"; here it is "how many can I run
// before anything outside the processor could possibly matter" — and it has
// the same two answers, because they are the same two deadlines.
//
// What a batch buys is the clock. Cpu::tick is 13% of the desktop profile and
// it runs on every emulated instruction to answer a question that only has a
// new answer when the timebase crosses the bus divisor or the decrementer goes
// negative; the device gate is another 2.8% of asking a question whose answer
// is cached. Over a run of instructions that nothing can observe, both are
// computed once. See the contract on Cpu::pendCycles.
//
// ⚠ THIS IS A LOWER BOUND AND IT HAS TO STAY ONE. Every clamp here is a claim
// that nothing can happen sooner; a wrong one does not crash, it delivers an
// interrupt late and moves the whole timeline. --no-batch is the control.
inline u64 batchSteps(const Cpu& cpu, const SawtoothBus& bus, u64 budget)
{
    if (budget < 2)
        return budget;
    const u32 cyclesPerStep = cpu.insnCycles + cpu.extraCycles;
    // ⏱ THE HOST CLOCK OWNS THE TIMEBASE, so executing instructions does not
    // advance it and neither deadline below can be reached by running. Nothing
    // bounds the batch except the caller's budget and the things that end it
    // from the inside — a device access, a write to DEC or the timebase, the
    // core going to sleep. The caller's budget is what carries the clock
    // sample, and it is the caller that sizes it. See Cpu::insnCycles.
    if (!cyclesPerStep)
        return budget;
    u64 n = budget;
    // The decrementer's own deadline: run to the step where tick() would have
    // asked for the interrupt, and no further. Exact, not approximate — this
    // is the clamp that lets the clock be applied in one piece.
    const u64 dec = cpu.stepsUntilDec(cyclesPerStep);
    if (dec < n)
        n = dec;
    // The nearest timed device event, converted through the same cycles per
    // step the clock advances at. A device the GUEST touches is not in here
    // and does not need to be: that access ends the batch from the inside
    // (Bus::deviceClock).
    const u64 dueTb = bus.deviceDueTb();
    if (dueTb <= cpu.st.tb)
        return 1; // already due: service it before running anything
    const u64 cycles = (dueTb - cpu.st.tb) * cpu.cyclesPerTbTick;
    const u64 s = cycles / cyclesPerStep; // floor: never overshoot
    if (s < n)
        n = s;
    return n ? n : 1;
}

// Run one batch: size it, run it, and report how many instructions it was.
// The caller services devices afterwards, exactly as it does after a step.
// ⚠ `executed` IS ADVANCED HERE, per instruction, not by the caller. See the
// note on Cpu::runSteps: the machine's instruction counter is wired into every
// device cell and three harness features are timed against it, so it may not
// sit still for the length of a batch.
inline u64 runBatch(Cpu& cpu, SawtoothBus& bus, u64& executed, u64 budget)
{
    u64 n;
    {
        OPM_MARK(Loop); // the sizing is the loop's cost, not the machine's
        n = batchSteps(cpu, bus, budget);
    }
    return cpu.runSteps(n, executed);
}

} // namespace opm
