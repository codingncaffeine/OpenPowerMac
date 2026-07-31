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
    if (budget < 2)
        return 0;
    // An enabled interrupt already waiting: step() must run and take it.
    if ((cpu.st.msr & msr::EE) &&
        (cpu.smiPending || cpu.extIrqLine || cpu.decPending || cpu.pmPending))
        return 0;
    const u32 cyclesPerStep = 1u + cpu.extraCycles;
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

} // namespace opm
