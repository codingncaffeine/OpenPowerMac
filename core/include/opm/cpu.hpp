#pragma once
#include "types.hpp"
#include "bus.hpp"
#include "insn.hpp"
#include "bits.hpp" // the line byte helpers the inline load/store use
#include "jit.hpp"  // compiled-line cache; pure derived state, see the header
#include "prof.hpp" // markers on the inline execution path; compiled away
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace opm {

// Full architected state of the MPC7400. Reset values per UM Table 2-18
// ("Settings Caused by Hard Reset"); fields the table calls undefined are
// zero-initialized here for determinism.
struct CpuState {
    u32 gpr[32]{};
    u64 fpr[32]{};      // raw IEEE-754 double bit patterns
    V128 vr[32]{};

    u32 pc = 0xFFF00100u;
    u32 cr = 0, xer = 0, lr = 0, ctr = 0;
    u32 msr = 0x00000040u;      // only IP set at HRESET
    u32 fpscr = 0;
    u32 vscr = 0, vrsave = 0;

    u32 srr0 = 0, srr1 = 0;
    u32 sprg[4]{};
    u32 dar = 0, dsisr = 0, sdr1 = 0, ear = 0, pir = 0;
    u32 sr[16]{};
    u32 ibatu[4]{}, ibatl[4]{};
    u32 dbatu[4]{}, dbatl[4]{};

    u32 hid0 = 0, hid1 = 0;
    u32 msscr0 = 0x00400000u, msscr1 = 0;
    u32 l2cr = 0, ictc = 0;
    u32 thrm[3]{};
    u32 iabr = 0, dabr = 0, bamr = 0;
    u32 mmcr0 = 0, mmcr1 = 0, pmc[4]{}, siar = 0, sdar = 0;

    u64 tb = 0;
    u32 dec = 0xFFFFFFFFu;
    u32 pvr = 0x000C0209u;      // MPC7400 silicon rev 2.9 (errata Table 2)

    bool resvValid = false;
    u32 resvAddr = 0;           // 32-byte-granule base
};

// MSR bit masks (32-bit, PPC numbering bit 0 = MSB). Verified: VEC = bit 6
// (UM Table 2-1); layout per MPCPRG; PM position pinned pending P6 sweep.
namespace msr {
inline constexpr u32 VEC = 0x02000000u;
inline constexpr u32 POW = 0x00040000u;
inline constexpr u32 ILE = 0x00010000u;
inline constexpr u32 EE  = 0x00008000u;
inline constexpr u32 PR  = 0x00004000u;
inline constexpr u32 FP  = 0x00002000u;
inline constexpr u32 ME  = 0x00001000u;
inline constexpr u32 FE0 = 0x00000800u;
inline constexpr u32 SE  = 0x00000400u;
inline constexpr u32 BE  = 0x00000200u;
inline constexpr u32 FE1 = 0x00000100u;
inline constexpr u32 IP  = 0x00000040u;
inline constexpr u32 IR  = 0x00000020u;
inline constexpr u32 DR  = 0x00000010u;
inline constexpr u32 PM  = 0x00000004u;
inline constexpr u32 RI  = 0x00000002u;
inline constexpr u32 LE  = 0x00000001u;
// Bits the 7400 implements (mtmsr/rfi mask).
inline constexpr u32 VALID = VEC | POW | ILE | EE | PR | FP | ME | FE0 | SE |
                             BE | FE1 | IP | IR | DR | PM | RI | LE;
} // namespace msr

// Exception vectors (offsets; base 0xFFF00000 when MSR[IP], else 0).
enum class Exc : u32 {
    SystemReset = 0x00100,
    MachineCheck = 0x00200,
    Dsi = 0x00300,
    Isi = 0x00400,
    External = 0x00500,
    Alignment = 0x00600,
    Program = 0x00700,
    FpUnavailable = 0x00800,
    Decrementer = 0x00900,
    SystemCall = 0x00C00,
    Trace = 0x00D00,
    PerfMon = 0x00F00,
    VecUnavailable = 0x00F20,
    Iabr = 0x01300,
    Smi = 0x01400,
    VecAssist = 0x01600,
    Thermal = 0x01700,
};

// Program-exception SRR1 cause bits.
inline constexpr u32 kSrr1ProgFpEnabled = 0x00100000u;
inline constexpr u32 kSrr1ProgIllegal = 0x00080000u;
inline constexpr u32 kSrr1ProgPrivileged = 0x00040000u;
inline constexpr u32 kSrr1ProgTrap = 0x00020000u;

struct Cpu {
    CpuState st;
    Bus* bus = nullptr;

    // Census of decode gaps hit at runtime: mnemonic (or raw word) -> count.
    std::map<std::string, u64> unimplemented;
    std::map<u32, u64> unknownWords;

    // Async lines / pending state.
    bool extIrqLine = false;   // level-sensitive external interrupt
    bool smiPending = false;
    bool decPending = false;
    bool pmPending = false;    // performance monitor (PMC MSB with PMXE)
    bool raisedThisStep = false;
    bool napping = false;      // MSR[POW] + HID0 nap/doze/sleep: TB ticks,
                               // no instructions until an enabled interrupt
    u32 curInsn = 0;           // instruction being executed (LE align image)
    u32 cycleAccum = 0;
    u32 cyclesPerTbTick = 4;   // TB = bus clock / 4; provisional 1 cycle/insn
    // Cycles the HARNESS adds to every instruction on top of the
    // architectural one (--fast-tb). It used to be a second tick() call from
    // the machine loop, and two calls per instruction is one more than the
    // model needs: across an instruction boundary tick(1) then tick(n) and a
    // single tick(1+n) leave TB, DEC and the pending flag identical, and
    // nothing observes them in between. The one merged away was the call that
    // crossed the divisor EVERY time, so it did the full arithmetic; the
    // architectural one alone crosses it once in four.
    //
    // ⚠ EQUIVALENCE IS LOAD-BEARING and it is not just the common path. A
    // step that delivers an async exception executes no instruction and
    // returns without the architectural tick — so it must still take
    // `extraCycles`, or the harness's clock silently stops for the length of
    // every interrupt and the timeline drifts away from every recorded
    // baseline.
    u32 extraCycles = 0;

    // ⏱⏱ THE ARCHITECTURAL CYCLE ITSELF — one per instruction, and ZERO when
    // the HOST CLOCK owns the timebase.
    //
    // A processor cycle per instruction is what makes the guest's clock a
    // function of how many instructions this emulator got through, and that is
    // a rate: tb advances `insnCycles / cyclesPerTbTick` per instruction, so
    // the emulator's own throughput is a FLOOR under the guest's clock. At the
    // architectural 1 and the bus divisor of 4 that floor is MIPS/4 — 25 MHz
    // at 100 MIPS, which is exactly the rate a Sawtooth's timebase runs at.
    //
    // That was invisible while the machine was slow. It is not now. Measured
    // at the Finder desktop on 2026-07-31: 203.6 MIPS, timebase 50.90 MHz,
    // 2.04x real, and 203.6/4 = 50.9 to three digits — the architectural
    // advance was the WHOLE clock and the host-paced top-up in the run loops
    // never fired once. The guest's 60 Hz chain ran at 122.5 Hz.
    //
    // ⭐ So under real-time pacing this is zero, and the host clock is the
    // only thing that advances the timebase. Then the guest's clock is right
    // at ANY speed — and stays right the next time this emulator gets faster,
    // which is the property the fix has to have. It also stops being a lie in
    // the other direction: a machine retiring 200 M instructions per guest
    // second now REPORTS 200 M instead of pretending to be a 100 M machine
    // whose seconds are half as long.
    //
    // ⚠ Instruction pacing — every measurement, every A/B run, every snapshot
    // proof — keeps the architectural 1 and is untouched.
    u32 insnCycles = 1;

    // Timebase/decrementer accounting. Whether the guest's 60 Hz tick is
    // driven off the decrementer is a question the whole boot hangs on —
    // Ticks ($016A) advanced once in 1.45 billion instructions — and it is
    // answerable only by counting what the guest actually does with DEC:
    // how often it reloads it, with what period, and how many 0x900s it
    // takes. Guessing from the interrupt total alone cannot separate "never
    // programmed" from "programmed far too long".
    u64 decWrites = 0;         // mtspr DEC by the guest
    u64 decIrqs = 0;           // 0x900 exceptions actually delivered
    // 0x500s delivered. A periodic global that moves at a fixed rate is being
    // driven by SOMETHING periodic, and the two candidates -- the decrementer
    // and a device line -- are told apart by which counter advanced between
    // two of its stores. Counting only the total says a wake-up happened and
    // names no source.
    u64 extIrqs = 0;
    u32 decLastWrite = 0;      // last value written
    u64 decLastWriteTb = 0;    // TB at that write
    u64 decMinPeriod = ~0ull;  // smallest reload seen (TB ticks)
    // ...and WHO programs it, with what. The guest's 60 Hz chain is
    // timebase-paced and runs 43x slow, so some routine is computing a period
    // in timebase units and getting it wrong; a total says that happened and
    // names nobody. Keyed by the writing pc — bounded by code sites rather
    // than by traffic — with the last value each site wrote, because the VALUE
    // is the whole question. r24 comes along because a write from the
    // NanoKernel's 68K emulator reports a pc that names nothing.
    struct DecSite {
        u64 hits = 0;
        u32 lastVal = 0, lastLr = 0, lastR24 = 0;
    };
    std::map<u32, DecSite> decByPc;

    // Derived from cyclesPerTbTick and refreshed whenever it changes (it is a
    // public field and the snapshot restores it). Kept on Cpu, never in
    // CpuState: the snapshot's layout digest hashes sizeof(CpuState), so
    // caches live out here and cost no snapshot.
    u32 tbDivSeen = 0, tbShift = 0;

    // Advance the timebase by `cycles` processor cycles.
    //
    // ⚠ THIS IS ON THE HOTTEST PATH IN THE PROGRAM and it used to be a LOOP.
    // The harness calls it twice per instruction — tick(1) from step(), then
    // tick(--fast-tb) from the machine loop, 60 by default — so with the
    // architectural bus/4 divisor it ran about sixteen iterations per emulated
    // instruction, each one a subtract, a compare and a branch. The profiler
    // put 16.1% of the whole machine's host time in here, more than the
    // instruction handlers themselves at 7.2%. Closed form gives the identical
    // sequence of TB and DEC values at every instruction boundary.
    void tick(u32 cycles)
    {
        cycleAccum += cycles;
        if (cycleAccum < cyclesPerTbTick)
            return;
        if (tbDivSeen != cyclesPerTbTick) {
            tbDivSeen = cyclesPerTbTick;
            tbShift = 32; // 32 = "not a power of two", use the divide
            for (u32 k = 0; k < 32; ++k)
                if (cyclesPerTbTick == (1u << k)) {
                    tbShift = k;
                    break;
                }
        }
        u32 n;
        if (tbShift < 32) {
            n = cycleAccum >> tbShift;
            cycleAccum &= cyclesPerTbTick - 1u;
        } else {
            n = cyclesPerTbTick ? cycleAccum / cyclesPerTbTick : 0u;
            cycleAccum -= n * cyclesPerTbTick;
        }
        st.tb += n;
        const u32 old = st.dec;
        st.dec = old - n;
        // The decrementer asks for an interrupt on the step where its MSB goes
        // 0 -> 1. Over n decrements from `old` that is exactly the step where
        // the count passes below zero, so it happens iff the count started
        // non-negative and n > old. From a negative start it would have to
        // fall through 2^31 first and then wrap, which needs n >= 2^31 — a
        // number of cycles no caller passes.
        if (!(old & 0x80000000u) && n > old)
            decPending = true;
    }

    // How many steps of `cyclesPerStep` cycles each can pass before the
    // decrementer's MSB goes 0 -> 1 and asks for its interrupt. ~0 when it
    // has already gone negative, because it will not ask again until the
    // guest reloads it — and the guest cannot, if no instruction is running.
    //
    // This is the clamp that makes skipping a run of asleep steps EXACT
    // rather than approximate: the skip stops on the step where tick() would
    // have raised the interrupt, so the state afterwards is the state the
    // one-at-a-time loop would have produced, tick for tick.
    // The same deadline in the machine's own units. ~0 when the decrementer
    // has already gone negative, because it will not ask again until the guest
    // reloads it — and a sleeping core cannot. tick() requests the interrupt on
    // the tick where the count passes below zero, which is one past the count
    // it holds now.
    //
    // This is the one a HOST-PACED loop needs: there is no instruction rate to
    // convert through when the clock comes from outside, so "when could this
    // machine next need me" has to be answerable in timebase alone.
    u64 tbUntilDec() const
    {
        if (st.dec & 0x80000000u)
            return ~0ull;
        return u64(st.dec) + 1;
    }

    u64 stepsUntilDec(u32 cyclesPerStep) const
    {
        if ((st.dec & 0x80000000u) || !cyclesPerStep || !cyclesPerTbTick)
            return ~0ull;
        const u64 need = static_cast<u64>(st.dec) + 1ull; // TB ticks to go
        const u64 total = need * cyclesPerTbTick;         // cycles for them
        const u64 cycles = total > cycleAccum ? total - cycleAccum : 0ull;
        return (cycles + cyclesPerStep - 1ull) / cyclesPerStep;
    }

    // Advance the clock by `steps` steps' worth of cycles without executing
    // anything. Chunked because tick() takes a 32-bit cycle count and a nap
    // can legitimately span millions of steps.
    void clockAdvanceCycles(u64 cycles)
    {
        while (cycles) {
            const u32 c = cycles > 0x3F000000ull
                              ? 0x3F000000u
                              : static_cast<u32>(cycles);
            cycles -= c;
            tick(c);
        }
    }
    void clockAdvance(u64 steps, u32 cyclesPerStep)
    {
        clockAdvanceCycles(steps * static_cast<u64>(cyclesPerStep));
    }

    // ---- the clock, and who owns it --------------------------------------
    //
    // ⏱ CHARGED PER INSTRUCTION, APPLIED PER BATCH. tick() is the second most
    // expensive thing in this emulator after the instruction handlers — 13% of
    // the desktop profile — and it runs to answer the same question every
    // time: has the timebase crossed the bus divisor, and has the decrementer
    // gone negative. Over a run of instructions in which nothing can OBSERVE
    // the clock, the answer only has to be computed once.
    //
    // So step() CHARGES cycles and a batch APPLIES them. The contract that
    // makes this exact rather than approximate has three parts, and all three
    // are enforced somewhere a reader can find:
    //
    //   1. the batch may not span anything that could change its own bounds —
    //      Cpu::batchBreak, set by a device access (Bus::deviceClock), by a
    //      write to DEC or the timebase, and by the core going to sleep;
    //   2. the batch may not span the decrementer's deadline — the caller
    //      sizes it with stepsUntilDec (machine/include/opm/pace.hpp);
    //   3. anything that READS the clock syncs it first — the guest through
    //      mfspr/mftb/mtspr, the devices through Bus::deviceNow, the harness
    //      through tbNow().
    // Miss one and the machine keeps running with a clock that is behind,
    // which is not a crash, it is a wrong measurement — so `--no-batch` is
    // the control, and the equivalence is proved the way the nap skip was:
    // same stop pc, same timebase to the tick, same guest Ticks.
    u64 pendCycles = 0;
    bool batching = false;   // a caller owns the clock advance: see runSteps
    bool batchBreak = false; // …and something happened that ends its batch
    void charge(u32 cycles)
    {
        pendCycles += cycles;
        if (!batching)
            syncClock();
    }
    void syncClock()
    {
        const u64 c = pendCycles;
        if (!c)
            return;
        pendCycles = 0;
        clockAdvanceCycles(c);
    }
    // The timebase as it stands RIGHT NOW, charged cycles included. Pure: it
    // computes what syncClock would produce without producing it, so an
    // instrument or a device can read the clock without perturbing the run.
    u64 tbNow() const
    {
        if (!pendCycles || !cyclesPerTbTick)
            return st.tb;
        return st.tb + (cycleAccum + pendCycles) / cyclesPerTbTick;
    }
    // Execute up to n instructions with the clock charged but not applied.
    // Stops early on batchBreak or halt. See pace.hpp for how n is chosen —
    // a caller that picks it wrongly does not crash, it delivers an interrupt
    // late.
    //
    // ⚠ `stamp` IS THE MACHINE'S INSTRUCTION COUNTER AND IT IS ADVANCED HERE,
    // one per instruction, NOT by the caller afterwards. It has to be live
    // inside the batch: the bus hands it to every device cell, three harness
    // features are timed against it (the display's config-space visibility
    // window, the paced serial injection, the ATA traffic logs), and every
    // instrument in g4run quotes it. Adding a batch's worth at the end left
    // all of them reading a count that was up to a batch behind — a lying
    // instrument first, and a real behavior difference second. The state
    // differential (tools/diffstate.sh) is what caught it.
    u64 runSteps(u64 n, u64& stamp);

    // Set when execution cannot continue (pre-P2 stand-in for the exception
    // model: traps, sc, illegal ops halt with a reason instead of vectoring).
    bool halted = false;
    std::string haltReason;

    // Dispatch tables, cached at construction (see the constructor). Not in
    // CpuState: instrument- and performance-only state lives on Cpu so that
    // no snapshot layout depends on it.
    const Handler* dispFn = nullptr; // handler per ISA row
    const u8* dispPre = nullptr;     // pre-dispatch gate bits per ISA row
    // The gate bits: everything step() has to check about an ISA row BEFORE
    // it can dispatch. Zero, the overwhelmingly common value, means
    // "dispatch, no checks".
    static constexpr u8 kPreIll = 1u;  // architected, not implemented on the 7400
    static constexpr u8 kPrePriv = 2u; // supervisor-only
    static constexpr u8 kPreFp = 4u;   // needs MSR[FP]
    static constexpr u8 kPreVec = 8u;  // needs MSR[VEC]

    Cpu();
    // ⏱ ONE WIRING SITE. The bus is handed the processor's clock here rather
    // than by each front end, for the reason SawtoothBus::setStamp exists: the
    // last piece of run-loop wiring that was written out twice was missing the
    // hard disk in one copy, and the app ran for weeks without one.
    void attach(Bus& b)
    {
        bus = &b;
        b.deviceClock = [](void* p) -> u64 {
            Cpu& c = *static_cast<Cpu*>(p);
            c.batchBreak = true;
            return c.tbNow();
        };
        b.deviceClockCtx = this;
    }
    void reset()
    {
        st = CpuState{};
        halted = false;
        haltReason.clear();
        extIrqLine = smiPending = decPending = pmPending = false;
        napping = false;
        cycleAccum = 0;
        // The clock restarts at zero, so cycles charged against the old one
        // are not owed against the new one.
        pendCycles = 0;
        batchBreak = true;
        fetchDrop();
        if (jit)
            jit->dropAll(); // compiled lines are derived from dropped lines
        tlbFlushAll();
        dmDropAll();
        cacheModeUpdate(); // HID0 is clear again: the full model
    }
    void halt(std::string reason)
    {
        halted = true;
        haltReason = std::move(reason);
        batchBreak = true; // the line executor tests one flag, not two
    }

    // Execute up to n instructions; returns the number actually executed.
    u64 run(u64 n);
    void step();
    // The dispatch tail of step(): everything from "we have the instruction
    // word and its ISA row" onwards — the gates, the handler, the pc, the
    // clock, the performance monitor and the trace. step() reaches it through
    // the fetch path; the line executor reaches it with the word and row read
    // straight out of a resident block. ⚠ ONE COPY, DELIBERATELY: two
    // execution paths that each implement the dispatch rules is the shape of
    // bug this project has already paid for twice, and the second copy is
    // always the one nobody measures.
    void execRow(u32 insn, u32 row);
    // The FALL-THROUGH of execRow — an ordinary instruction with a bound
    // handler and no gate — is inline in runSteps' inner loop, because MSVC
    // will not inline a function the size of execRow and the line loop was
    // paying a call with two arguments on every emulated instruction. It
    // duplicates no rule: every decision (illegal, privileged, FP and vector
    // availability, the unimplemented halt, the performance monitor, the
    // trace) stays here, and the loop defers to it.

    // Vector to an exception per the UM ch.4 model: SRR0/SRR1 composition,
    // MSR transition (clears VEC/POW/EE/PR/FP/FE0/SE/BE/FE1/PM/IR/DR/RI,
    // LE<-ILE, keeps ME/IP/ILE), PC to base|vector.
    // srr0 = address to save; extra = exception-specific SRR1 bits (1-4,10-15).
    void raiseExc(Exc v, u32 srr0, u32 extra);

    // 📓 A ring of the exceptions most recently taken. See raiseExc: one look
    // at a stopped machine cannot distinguish a handler entered ONCE from a
    // fault being re-taken forever, and that is the whole question when a guest
    // freezes inside one. The same vector at the same DAR, over and over, is
    // the second — visibly, without another ten-minute run to find out.
    //
    // ⚠ Deliberately on Cpu and NOT in CpuState: the snapshot layout digest
    // hashes sizeof(CpuState), so putting it there would invalidate every
    // snapshot in the project to gain an instrument. Same rule the caches obey.
    //
    // 📞 r0 AND lr ARE HERE FOR `sc`. A system call's SELECTOR is in r0 and its
    // caller is in lr, and without them a ring of thirty-two SYSCALLs reads as
    // one repeated address with nothing to say about what was being ASKED —
    // which is exactly where session 30's desktop hang stalled. They cost two
    // stores on a path that already writes six fields, and for every other
    // vector they are free context about who was running.
    struct ExcRec {
        u32 vec, srr0, srr1, dar, dsisr, r0, lr;
        u64 tb;
    };
    static constexpr u32 kExcRing = 32; // power of two: the index masks
    ExcRec excRing[kExcRing]{};
    u64 excRingAt = 0; // total ever raised; & (kExcRing-1) is the next slot

    // 📊 …AND A TALLY PER VECTOR, because thirty-two entries cannot tell a
    // storm from a coincidence. A ring full of one vector might be the machine
    // drowning in it or might be the last microsecond of an otherwise healthy
    // run; only the counts say which, and the rate against the instruction
    // count says whether it is pathological at all. Indexed by vector >> 8, so
    // 0x300 is [3] and 0xF20 shares [15] with 0xF00 — close enough for a
    // histogram, and it keeps the array to sixteen words.
    u64 excByVec[16]{};

    void setExternalIrq(bool level) { extIrqLine = level; }
    void raiseSmi() { smiPending = true; }

    bool userMode() const { return (st.msr & msr::PR) != 0; }

    // TLBs (UM ch.5: separate 128-entry, 2-way set-associative I and D
    // TLBs, 64 sets indexed by EA[14-19], LRU replacement per set). This is
    // REQUIRED micro-architectural state, not an optimization: the Old
    // World ROM's regime teardown wipes the hash table while relying on
    // warm TLB entries to keep its own code, I/O, and pager mappings alive
    // until it issues tlbie — walk-per-access dies exactly where real
    // silicon survives (pinned empirically, macrun boot trace 2026-07-26).
    // Lives outside CpuState: visible only through the staleness contract;
    // SST state deliberately excludes it.
    struct TlbEntry {
        bool v = false, c = false;
        u32 vsid = 0, pi = 0, rpn = 0, wimg = 0, pp = 0;
    };
    TlbEntry itlb[64][2], dtlb[64][2];
    // Element [64] of each is not a set: it is where the translation memos
    // (see DmLine below) write when a translation came from no TLB entry at
    // all, so that their replay of "the other way is now least recently used"
    // needs no branch. The 64 real sets are the only ones the snapshot
    // carries.
    u8 itlbLru[65] = {}, dtlbLru[65] = {};
    bool mmuProbe = false; // instrumentation: bypass TLB and R/C writeback
    // Harness convention (ppcrun flat rig only — never the machine): treat
    // real-mode accesses at/above this base as cache-inhibited so MMIO
    // (putc/exit) keeps device semantics with the caches enabled. The
    // architecture makes real mode cacheable (WIMG 0b0011); real software
    // must map I/O with I=1, and the machine path models exactly that.
    u32 realModeInhibitBase = 0;
    void tlbInvalidateClass(u32 ea); // tlbie: both ways, both TLBs
    void tlbFlushAll();

    // L1 data cache (UM ch.3: 32 KB, 8-way, 128 sets, 32-byte blocks,
    // write-back with write-allocate; LRU stands in for the PLRU tree —
    // architecturally invisible). Load-bearing the same way the TLBs are:
    // the Gossamer ROM runs its early world out of dcbz-conjured cache
    // lines (cache-as-RAM) before the memory controller is enabled, so
    // those lines must live and hit without any bus fill.
    //
    // ⭐ THE LOOKUP AND THE DATA LIVE APART, and that is a measurement, not a
    // preference. Every guest memory access searches a set, and s42 counted
    // 65.8 M lookups examining 4.57 ways each over one 200 M-instruction
    // in-game window. With the tag, the valid bit and the LRU age sitting
    // INSIDE the 44-byte line, that search walked ~200 bytes across four host
    // cache lines and dragged each way's 32 data bytes into L1 to read 12
    // bytes of metadata. `DIdx` puts all eight ways' metadata in ONE 64-byte
    // line, so a lookup touches one line and compares eight tags at once.
    //
    // ⚠⚠ sizeof(DLine) IS PINNED AT 44 AND THE static_assert IS THE GUARD.
    // snapshot.cpp's layoutDigest() hashes it, so a different value refuses
    // EVERY snapshot ever taken — including the in-game captures this project
    // measures against, which cannot be regenerated without replaying the
    // game. The L1D stream is written field by field and did not change here,
    // so the digest must not change either: the reserved bytes hold the size
    // while the fields move.
    struct DLine {
        u8 b[32] = {};
        bool d = false; // dirty; tag, valid and age are in DIdx below
        u8 rsv[11] = {};
    };
    static_assert(sizeof(DLine) == 44,
                  "snapshot layoutDigest() hashes sizeof(DLine)");
    // alignas is the whole point — an entry that straddled two host cache
    // lines would cost the second line this change exists to stop touching.
    // MSVC's C4324 just reports that it padded Cpu to honor it, which is
    // what was asked for.
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4324)
#endif
    struct alignas(64) DIdx {
        u32 tv[8] = {};  // (tag << 1) | valid, so one compare answers both
        u32 age[8] = {}; // LRU stamp, u32 and allowed to wrap as before
    };
    DIdx l1x[128];
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
    DLine l1d[128][8];
    u32 l1dClock = 0;
    bool dceOn() const { return (st.hid0 & 0x00004000u) != 0; }
    u64 memRead(u32 pa, u32 len, u32 wimg);
    void memWrite(u32 pa, u32 len, u64 v, u32 wimg);

    // Physical data watchpoint. Bus::watchPa only sees traffic that reaches
    // the bus, so an ordinary cached store to a low-memory global — Ticks,
    // a DCE field, a queue header — is invisible to it: the write lands in
    // an L1 line and the watch never fires, which reads exactly like "the
    // guest never wrote it". This sits on the store path itself, so it sees
    // the write whether or not the line is ever cast out.
    u32 wpPa = 0, wpEnd = 0; // inclusive byte range; wpEnd 0 disables
    // The pc of a store is often a shared primitive and names nothing: Open
    // Firmware's Forth is subroutine-threaded, so every `!` compiled into the
    // dictionary reports the same ff80b2ac. The LR is the return address
    // inside the word that CALLED it, which is the answer the question
    // wanted; without it a watchpoint says a value changed but not by whom.
    struct WpHit {
        u32 pc, pa, val, len, lr;
        u64 tb;
        // ⚠ The INSTRUCTION COUNT, not only the timebase. Every other
        // instrument in this project — heartbeats, snapshots, --bp-from,
        // --trace-from — is keyed on instructions, and a watch that reports
        // only tb cannot be lined up against any of them without interpolating
        // a rate that is not even constant. Correlating a store against when
        // the machine changed behavior is the whole use of this log.
        u64 at;
        // r24 is the 68K program counter inside the NanoKernel's emulator.
        // A store made by emulated 68K code reports a pc in 0x68xxxxxx --
        // the emulator's own dispatch, which names nothing and is the same
        // handful of addresses for every 68K store in the system. The 68K
        // routine that actually made it is only in r24, and low-memory
        // globals like Ticks are written almost exclusively from there.
        u32 r24;
        // What had woken the machine by the time of this store. The gap
        // between two stores to a periodic global is a rate; the gap between
        // these two counters over the same interval is its SOURCE.
        u64 dec, ext;
    };
    std::vector<WpHit> wpLog;
    u32 wpMax = 64;
    // Which bytes of the watched range were EVER written, and from where.
    // A watch aimed at one field answers "was it written?" with silence, and
    // silence is not an answer: the same question asked of the bus watch
    // reported zero hits for a word that provably changed. Watching the whole
    // RECORD and censusing it by address puts the control INSIDE the run —
    // a neighboring field known to change is the proof the watch was armed
    // and looking, so the silence of the field under test means what it says.
    // The log cannot do this job: it is capped, so a busy neighbor fills it
    // and the quiet field's absence becomes an artifact of the cap.
    // The LENGTH belongs with the count: a dcbz writes 32 bytes from one
    // address, so a census that recorded only the address would report a
    // block-zero of the record as a store to its first field and every other
    // field as untouched.
    struct WpSpan {
        u64 n = 0;
        u32 len = 0; // widest store seen at this address
    };
    std::map<u32, WpSpan> wpByPa; // first byte of each store -> count, width
    std::map<u32, u64> wpByPc;    // storing pc -> count
    u64 wpHits = 0;               // total, including those past wpMax
    // Record a store landing in the watched range; returns whether it did.
    // Shared by the ordinary store path and by dcbz, which writes 32 zero
    // bytes and reaches no other write path in this model.
    bool wpNote(u32 pa, u32 len, u64 v);
    // --wp-from N: ignore stores before instruction N. A 64-entry log fills
    // with Open Firmware and boot-payload memsets thousands of times before
    // the OS era starts, so an ungated watch reports the WRONG WINDOW rather
    // than nothing -- the same lie a capped log tells about absence.
    u64 wpFrom = 0;
    const u64* wpStamp = nullptr;
    // READ watchpoint (--rp): a census of which code READS a range, keyed by
    // the reading pc. Stores answer "who changed this"; only reads answer "does
    // anything consume it", which is the question a device buffer poses. These
    // live on Cpu and not in CpuState, so they are instrument state and no
    // snapshot layout depends on them (layoutDigest hashes sizeof(CpuState)).
    u32 rpPa = 0, rpEnd = 0; // inclusive byte range; rpEnd 0 disables
    u64 rpHits = 0;
    std::map<u32, u64> rpByPc;
    // ...and by LR. ffcf4598 is a shared copy primitive: its pc names nothing,
    // exactly like Open Firmware's `!`. The LR is the return address inside the
    // routine that CALLED it, which is the answer the question wanted.
    std::map<u32, u64> rpByLr;
    // CACHED-DEVICE watch (--cache-watch): which code reaches a physical
    // window THROUGH THE DATA CACHE, and under what mapping. A device's
    // registers must be mapped cache-inhibited: reached through the cache, a
    // read returns whatever the line held when it was filled and a store
    // sits in the line until it is cast out, so an interrupt status is never
    // seen to change and a written register never reaches the chip. From the
    // device's side that looks like a driver that is not there, and its only
    // trace is whole 32-byte rows of registers read and written equally
    // often. Noted on the line FILL, which is the miss path, so the watch
    // costs the hit path nothing. Instrument state, not snapshotted.
    u32 cwPa = 0, cwEnd = 0; // inclusive byte range; cwEnd 0 disables
    u64 cwFrom = 0;          // ignore fills before this instruction
    u64 cwFills = 0, cwPushes = 0;
    struct CwHit {
        u32 pc, lr, pa, wimg, msr;
        int bat;    // the DBAT whose block holds pa and is valid, or -1 = page table
        bool write; // the access that missed was a store
        u64 at;
    };
    std::vector<CwHit> cwLog; // the first 64
    std::map<u32, u64> cwByPc;
    void cwNote(u32 pa, u32 wimg, bool write);
    // DIAGNOSTIC, NOT MACHINE TRUTH (--uncached-from). Treat every access at
    // or above this physical address as cache-inhibited whatever the guest's
    // mapping says. It supplies the control for the watch above: if a device
    // that looked dead comes alive with its window forced uncached, the
    // mapping is the whole defect and the device model is exonerated.
    u32 diagUncachedBase = 0;
    // DIAGNOSTIC, NOT MACHINE TRUTH. Substitute wpForce for the value of any
    // 32-bit store landing exactly on wpPa. This exists to supply a positive
    // control: a chain that is decoded but not yet fixed can be proved end to
    // end by forcing the one cell it turns on, the way --em294-rts does for
    // the USB shim. Anything it makes work is evidence, never a fix.
    u32 wpForce = 0;
    bool wpForceSet = false;
    // ---- instruction fetch block buffer ----------------------------------
    //
    // Instructions are read in sequence: eight of every eight fetches land in
    // the same 32-byte block. Serving them one word at a time meant an
    // eight-way L1 tag sweep, an L2 probe and a bus read PER INSTRUCTION, and
    // the profiler charged 20% of the machine to that. This is the I-cache the
    // model never had, one block deep.
    //
    // ⚠ IT IS KEYED ON THE PHYSICAL ADDRESS AND IT MUST NEVER OUTLIVE ITS
    // DATA. The project's fetch rule is that a fetch sees a superset of what
    // real split caches would show — never less — so every path that can
    // change what memory or the caches hold for a block drops it: stores,
    // dcbz/dcbf/dcbi, castouts and pushes, L2 installs of modified data, L2
    // invalidates, the flush-alls, bus-master snoops, reset and snapshot
    // restore. A block is only ever filled from the L1, from the L2, or from
    // storage the bus agrees is memory (Bus::memoryAt).
    //
    // Lives on Cpu, never in CpuState: the snapshot's layout digest hashes
    // sizeof(CpuState), so caches out here cost no snapshot compatibility.
    // ⚠ ONE BLOCK IS NOT ENOUGH, MEASURED. A single buffer hit only 66.9% of
    // fetches against the 87.5% straight-line execution implies: a call and
    // its return alternate between two blocks and evict each other every time,
    // and Open Firmware's Forth is subroutine-threaded, so it does almost
    // nothing else. Sixty-four blocks direct-mapped covers 2 KB of code for
    // 2.3 KB of storage and costs exactly the same one tag compare.
    //
    // ⚠⚠ AND SIXTY-FOUR WAS NOT ENOUGH EITHER — MEASURED AT THE DESKTOP,
    // WHICH IS A DIFFERENT MACHINE FROM THE FIRMWARE. Open Firmware runs
    // tight threaded loops and hit 97.2% on 64 lines; Mac OS at the Finder
    // has a working set that 2 KB cannot hold, and the same 64 lines hit
    // 84.5% with 16.5 M fills coming straight off the bus. Sweep, resumed
    // from the desktop, 300 M instructions each:
    //
    //     lines   coverage   hit     fills from memory   MIPS
    //        64       2 KB   84.6%          16,523,991   55.2
    //       256       8 KB   90.5%          10,196,535   62.5
    //      1024      32 KB   96.3%           3,958,051   72.0
    //     4096     128 KB   99.0%           1,080,691   80.2
    //
    // 4096 cost 208 KB and is where that curve flattened, at the desktop.
    //
    // ⚠⚠⚠ AND A GAME IS A THIRD MACHINE. In a 3D game's level 4096 lines
    // hold 128 KB of a code working set near 2 MB: 9 M fills per 1.27 G
    // instructions, each a block read and eight decodes. Measured in two
    // games, real time, interleaved against 4096:
    //
    //     lines   coverage   speed
    //     16384     512 KB   +4.1%, +3.8%
    //     65536       2 MB   +5.9%, +6.0%
    //
    // What made the step possible is the layout below. The words and their
    // decodes, the handlers and the gates are on the heap; what stays in the
    // processor object is each slot's TAG (fetchBase), four bytes a line, so
    // the object is no bigger than it was with 4096 whole lines in it (it
    // lives on callers' stacks) and a compiled block still checks its line
    // at a fixed distance from the object.
    static constexpr u32 kFetchLines = 65536;
    static constexpr u16 kNoRow = 0xFFFFu; // decodes to nothing: illegal
    struct FetchLine {
        // The DECODE of each of the eight words, as an index into kIsa. Decode
        // is a pure function of the word, so it belongs with the word: it was
        // being recomputed on every execution of every instruction, and the
        // profiler charged 22% of the machine to it. Filled with the block and
        // dropped with the block, so it can never outlive the bytes it
        // describes.
        u16 row[8];
        // The eight instruction WORDS, already assembled. Held as bytes they
        // cost four loads, three shifts and three ors on every hit, and after
        // the caches above landed the fetch path was 42.7% of the machine and
        // the biggest item in it. The block is assembled once per fill and
        // read about eighteen times, so the work belongs in the fill.
        u32 w[8];
    };
    // The block each slot holds: its base, or 1 — which no 32-byte base can
    // be. Set by the constructor (fetchDrop).
    u32 fetchBase[kFetchLines];
    // ⚠ ALLOCATED, NOT INITIALIZED, and the same for the handlers and gates
    // below. A slot's words, rows, handlers and gate mean something only
    // while its tag names a block, and the fill that sets the tag writes all
    // of them first; nothing reads a slot whose tag does not match. Seven
    // megabytes zeroed in every constructor was nothing to a machine that
    // makes one processor, and minutes to a test tool that makes one per
    // test vector.
    std::unique_ptr<FetchLine[]> fetchLine =
        std::make_unique_for_overwrite<FetchLine[]>(kFetchLines);
    // ⭐ THE HANDLER OF EACH WORD, BOUND WHEN THE BLOCK IS FILLED. The line
    // executor used to get from a word to its handler in three loads and
    // three tests per instruction — the row, the gate byte for the row, the
    // handler for the row. Those are functions of the word, so they belong
    // with the word: fetchFn[slot * 8 + k] is the handler, or null for a word
    // that needs execRow's decisions (illegal, privileged, unimplemented).
    // A word that only needs MSR[FP] or MSR[VEC] keeps its handler, and
    // fetchGate[slot] says which words those are — bit k for FP, bit 8 + k
    // for VEC — so the executor tests the MSR once per line.
    //
    // Beside the lines rather than inside them: the JIT addresses FetchLine
    // by layout, and 264 KB of pointers do not belong in a Cpu that tests
    // put on the stack. Filled and dropped with the line, never on their own.
    std::unique_ptr<Handler[]> fetchFn =
        std::make_unique_for_overwrite<Handler[]>(size_t(kFetchLines) * 8u);
    std::unique_ptr<u16[]> fetchGate =
        std::make_unique_for_overwrite<u16[]>(kFetchLines);

    // ---- instruction translation cache -----------------------------------
    //
    // One page. Straight-line execution stays inside a 4 KB page for a
    // thousand instructions, and re-deriving the answer meant a four-entry BAT
    // sweep and a TLB probe every time.
    //
    // Validity is checked against the inputs themselves wherever that is
    // cheap — MSR's translation and privilege bits, and the segment register
    // for this address — and against a GENERATION for the two that are not:
    // the BATs and SDR1 (bumped in mtspr) and tlbie/tlbia (bumped in the TLB
    // invalidators). Deliberately used only on the fetch path, so an
    // instrument running with mmuProbe never sees it.
    u64 mmuGen = 0, xlGen = ~0ull;
    u32 xlPage = 0, xlPa = 0, xlMsr = 0, xlSr = 0;
    void xlDrop() { xlGen = ~0ull; }
    // ---- the data path's memo ---------------------------------------------
    //
    // A guest load or store is a translation and then a data-cache access,
    // and both are models with state of their own: a TLB hit makes the other
    // way of its set the least recently used, a cache hit stamps the line's
    // age, a store marks the line modified and drops any fetched copy of it.
    // The answer to "where does this address live" hardly ever changes
    // between two accesses to the same 32 bytes, yet every access re-derived
    // it: a BAT sweep or a page-cache compare, a TLB probe, an eight-way tag
    // search. That re-derivation was 40% of the processor thread in a game.
    //
    // The memo remembers the answer and REPLAYS the state changes. A hit does
    // exactly what translate() followed by memRead()/memWrite() would have
    // done to the machine — the LRU write, the age stamp, the dirty bit, the
    // fetch drop, the bytes — and nothing else, so the machine with the memo
    // is the machine without it, bit for bit. `dxlCacheOff` (--no-dxlate) is
    // the control: it walks every access, and the two must end identical.
    //
    // ⚠ WHAT A HIT TAKES ON TRUST, AND WHAT RETIRES IT. An entry is one EA
    // line (32 bytes) and holds where it translated to and where that block
    // sits in the L1. Each input to that answer is covered by exactly one of:
    //
    //   MSR[DR], [PR], [LE]      in the key, compared on every hit. Entries are
    //                            never made in little-endian mode, so the hit
    //                            path owes no munging and no alignment check.
    //   segment registers, data  `dmEpoch`, also in the key. runSteps compares
    //   BATs, HID0, DABR         them against a copy on entry (dmEnter) and
    //                            retires every entry if one moved. Every writer
    //                            of those ends its batch, so a batch never runs
    //                            on with a stale epoch.
    //   the TLB entry used       `dtlbStamp`, one counter per entry, bumped by
    //                            everything that rewrites or invalidates that
    //                            entry: a reload, the first-store C-bit rewalk,
    //                            tlbie, tlbia.
    //   the L1 line              compared on every hit: the way's (tag|valid)
    //                            word. Eviction, dcbi, dcbf, a snoop kill and
    //                            the flash invalidate all change it.
    //   write permission, C bit  an entry is good for stores only if a store
    //                            made it (wkey): the walk that store took set C
    //                            and proved the protection.
    //
    // ⚠ IT IS LIVE ONLY INSIDE runSteps. `dmLive` is the dead value everywhere
    // else, so step() called by a test rig or an instrumented loop takes the
    // full path, and so does everything while a watchpoint, a read census,
    // the cached-device watch, the uncached diagnostic or an MMU probe is
    // armed: those live on the full path and must see every access.
    //
    // Never made for a cache-inhibited or write-through page, with the data
    // cache off, or for an access that crosses a line. Lives outside
    // CpuState, like every cache here: no snapshot carries it.
    static constexpr u32 kDmLines = 4096; // at most 1024 can be valid at once
    static constexpr u32 kDmPages = 256;
    static constexpr u32 kDmMsr = msr::DR | msr::PR | msr::LE;
    static constexpr u32 kDmOff = 2u;   // dmLive outside a batch: matches no key
    static constexpr u32 kDmNever = 4u; // a stored key that nothing matches
    static constexpr u32 kDmEpochStep = 0x8000u; // above every kDmMsr bit
    struct DmLine {
        u32 eaLine = ~0u; // ea >> 5, or ~0: no line is numbered that
        u32 rkey = kDmNever, wkey = kDmNever;
        u32 paLine = 0; // pa >> 5
        u32 tv = 0;     // what l1x[set].tv[way] must still read
        u32 stamp = 0;  // what dtlbStamp[slot] must still read
        u8 set = 0, way = 0;
        u8 slot = 128;  // dtlb entry (set * 2 + way), or 128: none was used
        u8 lruIdx = 64, lruVal = 0; // the LRU write a TLB hit makes
        u8 rsv[3] = {};
    };
    struct DmPage {
        u32 page = ~0u; // ea >> 12, or ~0
        u32 rkey = kDmNever, wkey = kDmNever;
        u32 paPage = 0; // pa & ~0xFFF
        u32 stamp = 0;
        u8 wimg = 0, slot = 128, lruIdx = 64, lruVal = 0;
    };
    // Heap-owned: 134 KB that a Cpu on somebody's stack should not carry.
    std::unique_ptr<DmLine[]> dmLine = std::make_unique<DmLine[]>(kDmLines);
    std::unique_ptr<DmPage[]> dmPage = std::make_unique<DmPage[]>(kDmPages);
    u32 dtlbStamp[129] = {}; // [128] never moves: "no TLB entry was used"
    u32 dmEpoch = kDmEpochStep;
    u32 dmLive = kDmOff;
    // The translation path of the last successful DATA translate(): the dtlb
    // entry it used, or 128. xlateData passes it on to the entry it fills.
    u8 dmSlot = 128;
    // The inputs dmEpoch stands for, as they were when it was last current.
    u32 dmSigSr[16] = {}, dmSigBat[8] = {}, dmSigHid0 = 0, dmSigDabr = 0;
    // Census, profiling build only.
    u64 dmHits = 0, dmMisses = 0, dmPageHits = 0, dmPageMisses = 0,
        dmEpochs = 0;
    void dmDropAll();
    void dmEnter(); // runSteps: decide whether the memo may be used
    void dmNoteLine(u32 ea, u32 len, u32 pa, u32 wimg, bool write);

    // ---- memory read directly ---------------------------------------------
    //
    // The L1 and L2 above are models of where a block of memory is held, and
    // a guest can only tell that from memory itself in a handful of ways:
    // by running with the data cache off or locked, by the L2's test and
    // data-only modes, by throwing a modified block away, by mapping a
    // device cacheable. The firmware does several of those on its way up
    // (it sizes and tests the L2 with them). An operating system that has
    // the caches in their NORMAL mode does none: every modified block gets
    // back to memory, and what a load returns is what the last store put
    // there. For such a guest the two arrays are a third of the processor's
    // work that changes no answer.
    //
    // So while HID0 has the data cache enabled and unlocked, and L2CR has
    // the L2 enabled with no test, data-only or invalidate bit, and the bus
    // offers its memory (Bus::directRam), the machine is in DIRECT mode:
    //
    //   - main memory is the truth. Entering the mode writes every modified
    //     block of main memory back and takes all of main memory's blocks
    //     out of both arrays; while the mode lasts none is ever put in.
    //   - a load or a store to main memory is the bytes at dmRam + pa. So
    //     are the page table's own reads and writes, a fetch fill, a dcbz.
    //     dcbf and dcbst find nothing to push; a bus master's snoop finds
    //     nothing to push and only drops fetched code it writes over.
    //   - EVERYTHING ELSE IS THE MODEL, UNCHANGED: the TLBs with their
    //     replacement order and the page table's R and C bits, protection,
    //     DABR, the reservation, and every block that is NOT main memory. A
    //     device or the ROM mapped cacheable still lives in the L1 and L2
    //     exactly as before, so a block conjured with dcbz, filled and
    //     pushed at a device with dcbf arrives as one burst, as it must.
    //
    // Leaving the mode needs nothing: the arrays hold no block of main
    // memory, which is a state the model can simply continue from.
    //
    // ⚠ WHAT A GUEST COULD SEE, each counted (directReport):
    //   - dcbi, a flash invalidate (HID0[DCFI], L2CR[L2I]) or a reset throws
    //     modified blocks away in the model; here memory already has them.
    //   - switching the data cache or the L2 off without flushing first
    //     leaves the model reading memory that is behind the cache; here
    //     memory is current.
    //   - a modified block of a cacheable DEVICE leaves the L1 when it is
    //     pushed or when other blocks crowd it out. Main memory no longer
    //     crowds anything out, so an unpushed one stays longer.
    // Software that depends on any of these depends on losing a store or on
    // when a cache happens to evict, which no cache promises.
    //
    // `exactCaches` (g4run --exact-caches, opm_set_exact_caches) keeps the
    // model for everything: the control. The two must agree on every
    // register, the TLBs, every device and on memory as it reads with all
    // modified blocks written back (snapshotFingerprints' `guest`).
    //
    // dmRamSize is zero outside the mode, so "is this main memory, read
    // directly" is the one compare in ramDirect().
    static constexpr u32 kHid0Dce = 0x00004000u, kHid0Dlock = 0x00001000u;
    static constexpr u32 kL2crE = 0x80000000u, kL2crDo = 0x00400000u,
                         kL2crI = 0x00200000u, kL2crTs = 0x00040000u;
    bool exactCaches = false;
    bool dmDirect = false;
    u8* dmRam = nullptr;
    u32 dmRamSize = 0;
    bool ramDirect(u32 pa, u32 len) const
    {
        return pa < dmRamSize && len <= dmRamSize - pa;
    }
    // Decide the mode from the registers and the bus, and switch. Called by
    // the writers of HID0 and L2CR, before every batch, on reset and after a
    // snapshot is loaded.
    void cacheModeUpdate();
    u64 directEnters = 0, directLeaves = 0; // mode switches
    u64 directPushed = 0;    // modified blocks written back on entering
    u64 directDcbi = 0;      // dcbi on main memory while direct
    u64 directFlashInv = 0;  // DCFI or L2I while direct
    std::string directReport() const;

    // The direct mode's memo: one entry per page of main memory, in front of
    // readV/writeV. It holds what the line memo above holds, less the L1's
    // part: where the page translated to, as a host address, and the TLB
    // entry that answer came from. A hit replays the TLB set's LRU write and
    // moves the bytes. Covered exactly as the line memo's inputs are (the
    // key, the epoch, dtlbStamp, a store rides only an entry a store made);
    // the mode itself is in the epoch, so no entry outlives a switch. Made
    // only for a page that lies wholly in main memory, never in
    // little-endian mode or with DABR armed, and live only inside runSteps.
    static constexpr u32 kDpPages = 4096;
    struct DpPage {
        u32 page = ~0u; // ea >> 12, or ~0
        u32 rkey = kDmNever, wkey = kDmNever;
        u32 stamp = 0;      // what dtlbStamp[slot] must still read
        u8* host = nullptr; // the page's first byte
        u32 paPage = 0;     // pa & ~0xFFF: a store drops fetched code there
        u8 slot = 128, lruIdx = 64, lruVal = 0, rsv = 0;
    };
    std::unique_ptr<DpPage[]> dpPage = std::make_unique<DpPage[]>(kDpPages);
    u64 dpHits = 0, dpMisses = 0; // census, profiling build only
    void dpNotePage(u32 ea, u32 pa, bool write);

    // ---- …and the instruction side's ------------------------------------
    //
    // The same idea for the page an instruction is fetched from. The fetch
    // path keeps ONE page's translation (xlPage below), so every branch that
    // leaves the page re-translated: a BAT sweep, a TLB probe, and a trip
    // through step() to do it — 27.7 M of them in a 1.2 G-instruction game
    // window. ImPage remembers 256 pages, and a hit replays the one thing the
    // instruction-side translate() does to the machine, the ITLB set's LRU
    // write. Its inputs are covered exactly as the data memo's are: MSR's
    // IR/PR/LE in the key, the segment registers and instruction BATs by the
    // shared epoch, the ITLB entry by `itlbStamp`. Same switch, same rule
    // about being live only inside runSteps.
    static constexpr u32 kImPages = 256;
    static constexpr u32 kImMsr = msr::IR | msr::PR | msr::LE;
    struct ImPage {
        u32 page = ~0u; // ea >> 12, or ~0
        u32 key = kDmNever;
        u32 paPage = 0;
        u32 stamp = 0;
        u8 slot = 128, lruIdx = 64, lruVal = 0, rsv = 0;
    };
    std::unique_ptr<ImPage[]> imPage = std::make_unique<ImPage[]>(kImPages);
    u32 itlbStamp[129] = {};
    u8 imSlot = 128; // as dmSlot, for the last successful FETCH translate()
    u32 dmSigIbat[8] = {};
    u64 imHits = 0, imMisses = 0; // census, profiling build only
    // The page of `ea`, if the memo holds it: on a hit the one-page fetch
    // translation (xlPage and friends) is moved to that page exactly as
    // fetchDecoded would have moved it, and the page's physical base comes
    // back. Never walks and never raises; a miss leaves everything alone.
    bool fetchPageProbe(u32 ea, u32& paPage)
    {
        if (dmLive == kDmOff)
            return false;
        const u32 page = ea >> 12;
        const ImPage& m = imPage[page & (kImPages - 1u)];
        if (m.page != page || m.key != (dmLive | (st.msr & kImMsr)) ||
            itlbStamp[m.slot] != m.stamp)
            return false;
        OPM_COUNT(imHits);
        itlbLru[m.lruIdx] = m.lruVal;
        xlGen = mmuGen;
        xlPage = page;
        xlMsr = st.msr & kImMsr;
        xlSr = st.sr[ea >> 28];
        xlPa = m.paPage;
        paPage = m.paPage;
        return true;
    }
    bool fetchXlate(u32 ea, u32& pa); // the fetch path's translation front
    // DIAGNOSTIC (--no-icache): bypass the fetch block cache, the cached
    // decode and this translation cache all at once, restoring the fetch path
    // byte for byte as it was before they existed. See Cpu::fetchDecoded.
    bool fetchCacheOff = false;
    // DIAGNOSTIC (--no-dxlate): switch the translation memos off — the data
    // memo above and the instruction page memo — so every data access is
    // translated and looked up in the L1 in full and every change of
    // instruction page is walked. Same rule as --no-icache: a cache is a
    // claim, and the claim's control run needs a switch. The two runs must
    // write the same fingerprints.
    bool dxlCacheOff = false;
    // Census, counted only in the profiling build. A cache is a bet on
    // locality and a bet has to be settled with a number.
    u64 fetchHits = 0, fetchFillsL1 = 0, fetchFillsL2 = 0, fetchFillsMem = 0,
        fetchUncached = 0;
    // ⚠ WHY A MISS HAPPENED, WHICH IS A DIFFERENT QUESTION FROM HOW MANY.
    // A block cache can miss because it is too small for the working set, or
    // because something threw the whole thing away. Those have opposite fixes:
    // the first wants more lines, the second wants a narrower invalidation —
    // and enlarging a cache that is being wiped just makes the wipe cost more.
    // icbi drops EVERY line here although the architecture gives it an
    // address, and every DMA snoop does the same.
    u64 fetchDropIcbi = 0, fetchDropSnoop = 0, fetchDropFlush = 0;
    // Steps that executed NO instruction because the core was in nap/doze:
    // the clock advances, the fetch path is never entered, and the step still
    // counts. Without this, a machine that is asleep and a machine that is
    // working are the same MIPS number, and the fetch hit rate is computed
    // over a denominator that has nothing to do with the step count.
    u64 napSteps = 0;
    // Steps charged to the instruction count without calling step(), because
    // the core was asleep and could not wake inside them. Counted in every
    // build, not just the profiling one: it is the difference between "this
    // machine is fast" and "this machine is asleep", and the MIPS figure
    // cannot tell them apart on its own. See machine/include/opm/pace.hpp.
    u64 napSkipped = 0;
    // Instructions executed straight out of a resident block, without the
    // per-instruction fetch path — see runSteps. The interesting number is
    // this divided by the number of times a line was entered, which is the
    // average run of straight-line code this machine actually executes.
    u64 lineInsns = 0, lineEntries = 0;
    static u32 fetchSlot(u32 pa) { return (pa >> 5) & (kFetchLines - 1u); }

    // ⭐ THE BLOCK COVERING `ea`, IF IT IS ALREADY HERE. A pure probe: the
    // translation cache must hit and the block must be resident. It never
    // walks, never fills, never reads the bus and never raises — so a caller
    // that gets null has lost nothing by asking, and can take the ordinary
    // fetch path, which fills the block as a side effect and leaves the next
    // probe hitting.
    //
    // This is what makes a line executor cheap: every line below runs once per
    // LINE here and once per INSTRUCTION in step(). What it must not do is
    // outlive its inputs — see Cpu::runSteps, where the invariant is that
    // every writer of MSR, a segment register, the BATs, SDR1, the TLB or the
    // block itself ends the batch.
    FetchLine* fetchBlockFast(u32 ea, u32& word)
    {
        if (fetchCacheOff)
            return nullptr;
        if (st.msr & msr::LE)
            ea ^= 4u; // PEM 3.1.4.4, as in fetchDecoded
        if (xlGen != mmuGen || xlPage != (ea >> 12) ||
            xlMsr != (st.msr & (msr::IR | msr::PR | msr::LE)) ||
            xlSr != st.sr[ea >> 28])
            return nullptr;
        const u32 pa = xlPa | (ea & 0xFFFu);
        const u32 slot = fetchSlot(pa);
        if (fetchBase[slot] != (pa & ~31u))
            return nullptr;
        word = (pa >> 2) & 7u;
        return &fetchLine[slot];
    }
    // DIAGNOSTIC (--no-line-exec): take one instruction per trip through the
    // run loop, the way the machine did before whole blocks were executed in
    // one go. The control for runSteps' inner loop.
    bool lineExecOff = false;
    // ⭐ THE JIT (opm/jit.hpp, core/src/jit.cpp): each fetch line compiles to
    // one native block and runSteps enters it instead of the line executor's
    // inner loop. Heap-owned and created on first use, so a Cpu that never
    // runs batched code pays a pointer; --no-jit is the control, exactly as
    // --no-line-exec is for the line executor. Pure derived state: never
    // snapshotted, dropped on reset, re-keyed by the refill hook.
    std::unique_ptr<JitCache> jit;
    //
    // ⚠ ON BY DEFAULT ONLY WHERE IT HAS MEASURED FASTER. Built with MSVC the
    // JIT took the in-game window from 68.8 to 85.6 MIPS. Built with GCC on
    // Linux the same blocks run it at 86.4 against 89.0 for the line
    // executor alone (two interleaved pairs, real time, 97.0% of instructions
    // native, no bails): that compiler's interpreter is already where the
    // blocks get to, and a block pays a call for every memory access. So it
    // is compiled, tested and available there (g4run --jit) and off unless
    // asked for.
#if defined(_WIN32)
    bool jitOn = true;
#else
    bool jitOn = false;
#endif
    // DIAGNOSTIC (--no-jit-chain): compile blocks without chain sites, so
    // every block exit returns to the dispatcher as v1 did. The control for
    // Stage B, exactly as --no-jit is for the JIT itself.
    bool jitChainOff = false;
    // --jit-tsc, carried here because the cache is created on first use and
    // the harness sets its flags before the machine runs.
    bool jitTscOn = false;
    // DIAGNOSTIC (--no-jit-inline-mem): compile every load and store as a
    // call to its shim, as before memory could be read directly. The control
    // for the inline form (Compiler::memProbe in jit.cpp).
    bool jitInlineMemOff = false;
    // DIAGNOSTIC (--no-jit-direct): compile fallbacks as the execRow shim
    // call instead of a direct call to the bound handler. The control for the
    // direct-call lowering, which is the only way its speed claim can be
    // settled same-binary.
    bool jitDirectOff = false;
    // The running batch's budget bound (runSteps' `until`), published here so
    // the trampoline can pin it in a register: a chained hop re-checks
    // `until - stamp >= 8` without returning to C++. Written by jitRunLine on
    // every enter; meaningless between enters.
    u64 jitUntil = 0;
    // Block-to-block transfers that skipped the dispatcher (emitted code
    // increments this at every successful chain hop). The measure of Stage B:
    // insns / (enters + hops) is the true scaffold amortization.
    u64 jitChainHops = 0;
    static_assert(JitCache::kLines == kFetchLines,
                  "the JIT cache is slot-parallel to the fetch lines");
    void fetchDrop()
    {
        for (u32& b : fetchBase)
            b = 1;
    }
    void fetchDropAt(u32 pa)
    {
        u32& b = fetchBase[fetchSlot(pa)];
        if (b == (pa & ~31u))
            b = 1;
    }
    // icbi: every block that could be the one at this EFFECTIVE address. A
    // line is kept by physical address and icbi is not translated here, but
    // translation leaves the offset in the page alone, and the slot's low
    // bits are that offset's block number: so one slot in every 128.
    void fetchDropInPage(u32 ea)
    {
        for (u32 s = (ea >> 5) & 127u; s < kFetchLines; s += 128u)
            fetchBase[s] = 1;
    }
    void fetchDropRange(u32 pa, u32 len)
    {
        fetchDropAt(pa);
        if (((pa ^ (pa + len - 1u)) & ~31u) != 0)
            fetchDropAt(pa + len - 1u);
    }

    bool l1dPeek32(u32 pa, u32& w); // fetch path coherence peek
    bool l1dReadLine(u32 pa, u8* out); // whole-block variant, same precedence
    void dcbzLine(u32 pa);          // allocate + zero, no fill
    void dcbClean(u32 pa, bool invalidate); // dcbst / dcbf
    void dcbKill(u32 pa);                   // dcbi
    void l1dFlushAll(bool writeback);
    // SnoopSink::publish: modified lines copied out to memory, state untouched.
    void publishModified(u8* ram, u32 ramSize) const;
    // Snoop response for a bus master (see SnoopSink in bus.hpp). Pushes
    // every modified line covering [pa, pa+len) all the way to MEMORY —
    // not into the L2, which a DMA engine cannot see either — and
    // invalidates the processor's copy as well when the master is writing.
    void snoopPush(u32 pa, u32 len, bool invalidate);

    // Backside L2 (UM ch.3.7: L2CR-governed, 256K-2MB, 2-way here). The
    // boot ROM runs its pre-DRAM world in it via L2CR[L2TS]: dcbf/dcbst
    // pushes land in the L2 marked valid instead of dying on the bus (UM
    // 2-17/L2TS verbatim) — hash tables and sizing scratch live there
    // until real memory exists. Castouts allocate; fills probe L2 first.
    struct L2Line {
        bool v = false, d = false;
        u32 tag = 0; // pa >> 5 (line number)
        u32 age = 0;
        u8 b[32] = {};
    };
    std::vector<L2Line> l2;
    u32 l2Sets = 0, l2Clock = 0;
    // Set index without a division. Every L2CR size the 7400 offers gives a
    // power-of-two set count (256K..2M over 32-byte lines, 2 ways), and this
    // index is computed on every instruction FETCH — a runtime `%` there is a
    // hardware divide on the hottest path in the program. Cached rather than
    // computed in l2Resize() so that a snapshot which restores l2Sets
    // directly cannot leave a stale mask behind; a non-power-of-two count
    // simply falls back to the divide.
    u32 l2SetsSeen = 0, l2SetMask = 0;
    u32 l2SetOf(u32 pa)
    {
        if (l2SetsSeen != l2Sets) {
            l2SetsSeen = l2Sets;
            l2SetMask = (l2Sets && (l2Sets & (l2Sets - 1u)) == 0) ? l2Sets - 1u
                                                                  : 0u;
        }
        const u32 line = pa >> 5;
        return l2SetMask ? (line & l2SetMask) : (line % l2Sets);
    }
    bool l2On() const { return (st.l2cr & 0x80000000u) != 0; }
    void l2Resize();                 // per L2CR[L2SIZ]
    L2Line* l2Find(u32 pa);
    void l2Install(u32 pa, const u8* bytes, bool dirty);
    bool l2ReadLine(u32 pa, u8* out);
    // The same block for the fetch buffer, with the L2's own bookkeeping
    // left alone: an instruction fetch must not reorder the replacement
    // policy of a cache it only looks into, any more than l1dReadLine does.
    bool l2PeekLine(u32 pa, u8* out);
    void l2Invalidate(u32 pa);
    void l2WipeAll();                // L2I global invalidate
    bool l2Peek32(u32 pa, u32& w);
    void l2FlushAll(bool writeback); // instrumentation/harness coherence

    // DIAGNOSTIC census: accesses that bypass the caches (cache-inhibited,
    // write-through, or HID0[DCE] clear) while the L2 holds a line for the
    // same block. Each one is a place where the L2 and memory can disagree,
    // and "the L2 is enabled and the machine misbehaves" needs to be told
    // apart from "the L2 is enabled and something reads round it". Counts by
    // the accessing pc, so it is bounded by code sites rather than traffic.
    // Lives on Cpu, not CpuState, so no snapshot is invalidated.
    u64 l2SkewR = 0, l2SkewW = 0;
    std::map<u32, u64> l2SkewByPc;
    void noteL2Skew(u32 pa, bool write);

    // DIAGNOSTIC, NOT MACHINE TRUTH. Keep every architectural effect of the
    // L2 — L2CR programs, L2E sets, the sizing and the invalidate all happen
    // and read back — while the array holds nothing: every lookup misses and
    // every allocation goes straight to memory. It separates "the guest
    // behaves differently because the L2 is switched on" from "the guest
    // behaves differently because our L2 serves the wrong bytes", which no
    // amount of staring at the model can tell apart.
    bool l2Inert = false;

    // Thermal Assist Unit: recompute THRM1/THRM2 TIN+TIV from their thresholds
    // and THRM3[E]. Called on every mtspr to a THRM register, which is what
    // restarts a comparison (um7400 2.1.5.6). Adding a method, not a field, so
    // existing snapshots stay valid.
    void thrmUpdate();

    // MMU (mmu.cpp). translate() raises ISI/DSI itself on failure. wimg, if
    // requested, receives the access's WIMG nibble (W=8,I=4,M=2,G=1; real
    // mode reads as 0b0011 per PEM 7.2).
    bool translate(u32 ea, bool write, bool fetch, u32& pa, u32* wimg = nullptr);
    // translate() through the memo's page table (see DmPage above). Guest
    // data accesses only — instruments and probes keep calling translate().
    bool xlateData(u32 ea, bool write, u32& pa, u32& wimg);
    // ⭐ A GUEST LOAD / STORE. The direct mode's hit is here, inline, so
    // that a handler's load is a table probe and a byte swap. Everything
    // else is readVSlow/writeVSlow: the line memo's hit when the cache model
    // is in the path (see DmLine for what that replays), then the first
    // touch of a page, a page crossing, a device, a fault — the full path,
    // which is also the only path there is outside runSteps.
    bool readV(u32 ea, u32 len, u64& out)
    {
        const u32 page = ea >> 12;
        const DpPage& m = dpPage[page & (kDpPages - 1u)];
        if (m.page == page && m.rkey == (dmLive | (st.msr & kDmMsr)) &&
            (ea & 0xFFFu) + len <= 0x1000u && dtlbStamp[m.slot] == m.stamp) {
            OPM_MARK(Read);
            OPM_COUNT(dpHits);
            dtlbLru[m.lruIdx] = m.lruVal;
            out = lineLoadBE(m.host + (ea & 0xFFFu), len);
            return true;
        }
        return readVSlow(ea, len, out);
    }
    bool writeV(u32 ea, u32 len, u64 v)
    {
        const u32 page = ea >> 12;
        const DpPage& m = dpPage[page & (kDpPages - 1u)];
        if (m.page == page && m.wkey == (dmLive | (st.msr & kDmMsr)) &&
            (ea & 0xFFFu) + len <= 0x1000u && dtlbStamp[m.slot] == m.stamp) {
            OPM_MARK(Write);
            OPM_COUNT(dpHits);
            dtlbLru[m.lruIdx] = m.lruVal;
            // the store may land on fetched code
            fetchDropRange(m.paPage | (ea & 0xFFFu), len);
            lineStoreBE(m.host + (ea & 0xFFFu), len, v);
            return true;
        }
        return writeVSlow(ea, len, v);
    }
    bool readVSlow(u32 ea, u32 len, u64& out);
    bool writeVSlow(u32 ea, u32 len, u64 v);
    bool fetch32(u32 ea, u32& insn);
    // Fetch AND decode in one step, through the caches above. `row` comes back
    // as an index into kIsa, or kNoRow when the word decodes to nothing.
    bool fetchDecoded(u32 ea, u32& insn, u32& row);
    bool leAlignCheck(u32 ea, u32 len); // LE mode: misaligned -> alignment exc

    bool readV8(u32 ea, u32& v)  { u64 t; if (!readV(ea, 1, t)) return false; v = static_cast<u32>(t); return true; }
    bool readV16(u32 ea, u32& v) { u64 t; if (!readV(ea, 2, t)) return false; v = static_cast<u32>(t); return true; }
    bool readV32(u32 ea, u32& v) { u64 t; if (!readV(ea, 4, t)) return false; v = static_cast<u32>(t); return true; }
    bool writeV8(u32 ea, u32 v)  { return writeV(ea, 1, v & 0xFFu); }
    bool writeV16(u32 ea, u32 v) { return writeV(ea, 2, v & 0xFFFFu); }
    bool writeV32(u32 ea, u32 v) { return writeV(ea, 4, v); }

    // --- state helpers shared by executors ---
    void setCr0(u32 val)
    {
        const i32 s = static_cast<i32>(val);
        u32 f = (s < 0) ? 8u : (s > 0) ? 4u : 2u;
        f |= (st.xer >> 31) & 1u; // SO
        st.cr = (st.cr & 0x0FFFFFFFu) | (f << 28);
    }
    void setCrField(u32 field, u32 nibble)
    {
        const u32 sh = (7u - field) * 4u;
        st.cr = (st.cr & ~(0xFu << sh)) | ((nibble & 0xFu) << sh);
    }
    u32 crField(u32 field) const { return (st.cr >> ((7u - field) * 4u)) & 0xFu; }
    void setCa(bool ca)
    {
        st.xer = ca ? (st.xer | 0x20000000u) : (st.xer & ~0x20000000u);
    }
    bool ca() const { return (st.xer & 0x20000000u) != 0; }
    void setOv(bool ov)
    {
        if (ov)
            st.xer |= 0xC0000000u; // OV + sticky SO
        else
            st.xer &= ~0x40000000u;
    }
};

// Hands a Bus its snoop responder. Kept as a separate object rather than
// making Cpu itself a SnoopSink: a vtable would change sizeof(Cpu), and the
// snapshot format's layout digest is computed from exactly that.
struct CpuSnoop final : SnoopSink {
    Cpu* cpu = nullptr;
    void snoopRead(u32 pa, u32 len) override;
    void snoopWrite(u32 pa, u32 len) override;
    void publish(u8* ram, u32 ramSize) override;
    bool memoryCurrent() const override { return cpu && cpu->dmDirect; }
};

// Binds every implemented handler into the dispatch used by Cpu::step.
// Idempotent; called from the Cpu constructor.
void bindHandlers();
void bindFpuHandlers(); // fpu.cpp's registrations, called by bindHandlers()
void bindVecHandlers(); // altivec.cpp's registrations, called by bindHandlers()

// Handler slot for an ISA row (parallel to kIsa; see decode()).
Handler handlerFor(const InsnDesc* d);
void setHandler(const char* mnem, Handler fn);

} // namespace opm
