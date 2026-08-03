// Execution loop and handler binding.
//
// step(): fetch, decode, pre-increment PC (so handlers see NIA in st.pc and
// compute CIA as st.pc - 4), dispatch. Undecodable or not-yet-implemented
// instructions halt with a census entry — the pre-P2 stand-in for the
// program-exception path.

#include "opm/cpu.hpp"
#include "opm/bits.hpp"
#include "opm/prof.hpp"
#include <cstring>
#include <vector>

namespace opm {

namespace {

std::vector<Handler>& slots()
{
    static std::vector<Handler> v(kIsaCount, nullptr);
    return v;
}

// Everything step() has to check about an ISA row BEFORE it can dispatch,
// precomputed once per row.
//
// These are properties of the row and never change, but they used to be
// recomputed for every emulated instruction: isFpInsn and isVecInsn are each a
// switch over the operand pattern, so two switches ran on the hot path forever
// to answer a question the table already knew. The profiler charged 15.1% of
// the machine to "decode" — twice what the instruction handlers themselves
// cost. Zero, the overwhelmingly common value, means "dispatch, no checks".
enum : u8 {
    kPreIll = 1u,  // architected but not implemented on the 7400
    kPrePriv = 2u, // supervisor-only
    kPreFp = 4u,   // needs MSR[FP]
    kPreVec = 8u,  // needs MSR[VEC]
};

std::vector<u8>& preGates()
{
    static std::vector<u8> v = [] {
        std::vector<u8> g(kIsaCount, 0u);
        for (size_t i = 0; i < kIsaCount; ++i) {
            u8 b = 0;
            if (kIsa[i].flags & FL_ILL7400)
                b |= kPreIll;
            if (kIsa[i].flags & FL_PRIV)
                b |= kPrePriv;
            if (isFpInsn(kIsa[i]))
                b |= kPreFp;
            if (isVecInsn(kIsa[i]))
                b |= kPreVec;
            g[i] = b;
        }
        return g;
    }();
    return v;
}

} // namespace

Handler handlerFor(const InsnDesc* d)
{
    return slots()[static_cast<size_t>(d - kIsa)];
}

void setHandler(const char* mnem, Handler fn)
{
    for (size_t i = 0; i < kIsaCount; ++i) {
        if (std::strcmp(kIsa[i].mnem, mnem) == 0) {
            slots()[i] = fn;
            return;
        }
    }
}

Cpu::Cpu()
{
    bindHandlers();
    // Cache the dispatch tables on the object. They are function-local
    // statics, so every use costs a thread-safe-initialization guard check,
    // and step() used them twice per instruction. Neither vector is resized
    // after binding, so the pointers stay valid for the life of the process.
    dispFn = slots().data();
    dispPre = preGates().data();
}

void Cpu::step()
{
    raisedThisStep = false;

    // Nap/doze/sleep: no instructions execute, the time base keeps ticking,
    // and an enabled interrupt wakes the core into its handler (MSR[POW]
    // cleared by the exception's MSR transition).
    if (napping) {
        const bool wake = (st.msr & msr::EE) &&
                          (smiPending || extIrqLine || decPending || pmPending);
        if (!wake) {
            OPM_COUNT(napSteps);
            charge(insnCycles + extraCycles);
            return;
        }
        napping = false;
    }

    // Async exceptions at the instruction boundary, EE-gated.
    // Priority per UM ch.4: SMI, then external, then decrementer.
    //
    // No instruction executes on these paths, so there is no architectural
    // tick — but the harness's extra cycles are the machine loop's, not the
    // instruction's, and the loop used to add them after step() returned
    // whatever it did. Dropping them here would stop the compressed clock for
    // the length of every interrupt, which on this machine is half a million
    // of them per boot.
    if (st.msr & msr::EE) {
        if (smiPending) {
            smiPending = false;
            raiseExc(Exc::Smi, st.pc, 0);
            charge(extraCycles);
            return;
        }
        if (extIrqLine) {
            ++extIrqs;
            raiseExc(Exc::External, st.pc, 0);
            charge(extraCycles);
            return;
        }
        if (decPending) {
            decPending = false;
            ++decIrqs;
            raiseExc(Exc::Decrementer, st.pc, 0);
            charge(extraCycles);
            return;
        }
        if (pmPending) {
            pmPending = false;
            raiseExc(Exc::PerfMon, st.pc, 0);
            charge(extraCycles);
            return;
        }
    }

    const u32 cia = st.pc;

    // IABR: address breakpoint on the instruction about to execute.
    // IABR[30]=enable, IABR[31]=translation mode must equal MSR[IR].
    if ((st.iabr & 2u) && ((cia ^ st.iabr) & 0xFFFFFFFCu) == 0 &&
        ((st.iabr & 1u) != 0) == ((st.msr & msr::IR) != 0)) {
        raiseExc(Exc::Iabr, cia, 0);
        charge(insnCycles + extraCycles);
        return;
    }

    u32 insn, row;
    OPM_PH(Fetch);
    if (!fetchDecoded(cia, insn, row)) { // ISI raised by translate()
        charge(insnCycles + extraCycles);
        return;
    }
    execRow(insn, row);
}

// The dispatch tail. Reached from step() with the word off the fetch path, and
// from runSteps' line executor with the word read straight out of a resident
// block — the rules below are the same rules either way, which is the point of
// there being one of them.
void Cpu::execRow(u32 insn, u32 row)
{
    raisedThisStep = false;
    const u32 cia = st.pc;
    OPM_PH(Decode);
    curInsn = insn;
    if (row == kNoRow) {
        ++unknownWords[insn];
        raiseExc(Exc::Program, cia, kSrr1ProgIllegal);
        charge(insnCycles + extraCycles);
        return;
    }
    const InsnDesc* d = kIsa + row;
    // The pre-dispatch gates, in the architecture's priority order, read out
    // of the table instead of recomputed. Almost every instruction gates on
    // nothing, so the common path is one load and one branch.
    if (const u8 gate = dispPre[row]) {
        if (gate & kPreIll) {
            raiseExc(Exc::Program, cia, kSrr1ProgIllegal);
            charge(insnCycles + extraCycles);
            return;
        }
        if ((gate & kPrePriv) && userMode()) {
            raiseExc(Exc::Program, cia, kSrr1ProgPrivileged);
            charge(insnCycles + extraCycles);
            return;
        }
        if ((gate & kPreFp) && !(st.msr & msr::FP)) {
            raiseExc(Exc::FpUnavailable, cia, 0);
            charge(insnCycles + extraCycles);
            return;
        }
        if ((gate & kPreVec) && !(st.msr & msr::VEC)) {
            raiseExc(Exc::VecUnavailable, cia, 0);
            charge(insnCycles + extraCycles);
            return;
        }
    }

    const Handler fn = dispFn[row];
    if (!fn) {
        ++unimplemented[d->mnem];
        halt(std::string("unimplemented: ") + d->mnem);
        return;
    }

    st.pc += 4;
    const u32 fallThrough = st.pc;
    OPM_PH(Exec);
    fn(*this, insn, *d);
    OPM_PH(Tick);
    charge(insnCycles + extraCycles);

    // Performance monitor, minimal-honest: PMC1/PMC2 count cycles (event 1,
    // one per instruction at the provisional 1 cycle/insn rate) or completed
    // instructions (event 2) unless globally frozen; a counter's MSB going
    // 0->1 with PMC1CE/PMCnCE and PMXE requests the (EE-gated) interrupt.
    // MMCR0 == 0 is the state this machine spends its whole life in: no event
    // is selected, so both counter arms below are dead code, and the shifts
    // and compares that prove it ran on every emulated instruction. Testing
    // the register against zero first is exactly equivalent — selector 0 is
    // "count nothing" — and it is one compare instead of eight instructions.
    if (st.mmcr0 && !(st.mmcr0 & 0x80000000u)) { // FC
        const u32 sel1 = (st.mmcr0 >> 6) & 0x7Fu;
        const u32 sel2 = st.mmcr0 & 0x3Fu;
        const u32 pmxe = st.mmcr0 & 0x04000000u;
        if (sel1 == 1 || sel1 == 2) {
            const u32 old = st.pmc[0];
            st.pmc[0] += 1;
            if (!(old & 0x80000000u) && (st.pmc[0] & 0x80000000u) && pmxe &&
                (st.mmcr0 & 0x00008000u)) // PMC1CE
                pmPending = true;
        }
        if (sel2 == 1 || sel2 == 2) {
            const u32 old = st.pmc[1];
            st.pmc[1] += 1;
            if (!(old & 0x80000000u) && (st.pmc[1] & 0x80000000u) && pmxe &&
                (st.mmcr0 & 0x00004000u)) // PMCnCE
                pmPending = true;
        }
    }

    // ⚠ THE PHASE HAS TO BE HANDED BACK HERE. Everything after this point —
    // the trace check, the return, and then the run loop's own per-instruction
    // bookkeeping until the next Decode marker — is the LOOP's cost, not the
    // clock's. Leaving Tick set through it billed the line executor's exit
    // tests to tick+perfmon and read 36.1% on a machine whose clock advance is
    // now three instructions. Third time this project has been lied to by a
    // marker that outlived the work it named; see the note in prof.hpp.
    OPM_PH(Loop);
    if (halted || raisedThisStep)
        return;

    // Trace (SE: every completed instruction; BE: taken branches). The 7400
    // does not trace isync; rfi is a context-synchronizing return.
    if (st.msr & (msr::SE | msr::BE)) {
        const bool isRfi = d->kind == Xk::X19 && d->xo == 50;
        const bool isIsync = d->kind == Xk::X19 && d->xo == 150;
        const bool branchTaken =
            (d->pat == Pat::B || d->pat == Pat::BC || d->pat == Pat::BCLR ||
             d->pat == Pat::BCCTR) &&
            st.pc != fallThrough;
        if (!isRfi && !isIsync &&
            ((st.msr & msr::SE) || ((st.msr & msr::BE) && branchTaken)))
            raiseExc(Exc::Trace, st.pc, 0);
    }
}

u64 Cpu::run(u64 n)
{
    u64 i = 0;
    for (; i < n && !halted; ++i)
        step();
    return i;
}

// A batch: n instructions with the clock charged once at the end instead of
// once per instruction. See the contract on Cpu::pendCycles, and pace.hpp for
// how a caller sizes n.
//
// ⚠ THE EARLY EXIT IS THE WHOLE CORRECTNESS ARGUMENT. Everything the caller
// proved about this run of instructions was proved before the first one ran:
// that no device could act, that the decrementer could not fire, that nothing
// would read the clock behind the loop's back. Any of those becoming false
// sets batchBreak, and the batch ends on the instruction that did it — so the
// machine is never left running past the point where the reasoning stopped
// applying.
// ⭐ THE LINE EXECUTOR. Straight-line code is executed out of the block the
// fetch cache is already holding: the block's 32 bytes carry the eight
// instruction words AND their decoded ISA rows, so a run of instructions
// inside one costs an array index each instead of a translation-cache check, a
// slot index, a tag compare and two loads. That per-instruction fetch work is
// 14.7% of the desktop profile with another 7.6% in the translation cache, and
// it is the whole of what a block executor can remove — the handlers
// themselves (35.1%) and memory (7.5%) are untouched, because a translated
// block would still have to call exactly the same helpers.
//
// ⚠ WHAT THE INNER LOOP IS ALLOWED TO ASSUME, AND WHY EACH ONE HOLDS. It runs
// with the fetch path skipped, so every input to that path has to be
// unchanged, and every check step() would have made per instruction has to be
// either made here or provably invariant:
//
//   the block is still ours   checked every instruction (fl->base). Every
//                             writer of memory or of a cache drops the line —
//                             stores, dcbz/dcbf/dcbi, castouts, dirty L2
//                             installs, snoops, icbi, reset — and that same
//                             contract is what the four-way control already
//                             proved transparent.
//   the mapping is unchanged  MSR, the segment registers, the BATs, SDR1 and
//                             the TLB all end the batch when written; see the
//                             batchBreak sites.
//   the pc fell through       checked every instruction. A branch or an
//                             exception leaves the line by definition.
//   no async is pending       hoisted: nothing inside a batch can raise one
//                             (see the contract on pendCycles), and the one
//                             thing that could — enabling MSR[EE] with a line
//                             already asserted — is an mtmsr, which ends it.
//   no nap, no IABR, no
//   trace, no perfmon         hoisted: all four are checked once per line and
//                             hand the whole line back to step() if live.
u64 Cpu::runSteps(u64 n, u64& stamp)
{
    batchBreak = false;
    batching = true;
    const u64 first = stamp;
    // ⚠ `i` IS THE CALLER'S OWN COUNTER, not a local copy written back at the
    // end: see the note on the declaration. Counting here and publishing later
    // is free, and it hands every device in the machine an instruction number
    // that is up to a batch stale.
    u64& i = stamp;
    const u64 until = first + n;
    while (i < until) {
        // The conditions the inner loop hoists. Rare enough that paying for
        // the check once per line is free, and live rarely enough that the
        // fallback costs nothing measurable: iabr and the trace bits are debug
        // features, mmcr0 is zero for this machine's whole life, and a napping
        // core belongs to the nap skip.
        const bool slow =
            lineExecOff || napping || st.mmcr0 || (st.iabr & 2u) ||
            (st.msr & (msr::SE | msr::BE)) ||
            ((st.msr & msr::EE) &&
             (smiPending || extIrqLine || decPending || pmPending));
        u32 word = 0;
        FetchLine* fl;
        {
            // The probe is the fetch path — bill it there. Left unmarked it
            // inherited whatever phase the previous instruction ended in, and
            // the one piece of real work the line executor still does per line
            // showed up as loop overhead.
            OPM_MARK(Fetch);
            fl = slow ? nullptr : fetchBlockFast(st.pc, word);
        }
        if (!fl) {
            // No block, or something the line loop may not assume away. One
            // instruction the ordinary way — which also FILLS the block, so
            // the next trip round finds it.
            step();
            ++i;
            if (batchBreak || halted)
                break;
            continue;
        }
        // ⭐ THE JIT. Same window, same hoists, same exits as the inner loop
        // below — see opm/jit.hpp. Only when a whole line's worth of budget
        // remains: a block never checks the budget per instruction, so it
        // must be unable to overrun `until` (fpClamp exactness rides on
        // that), and the interpreter finishes any sub-line tail. LE mode is
        // excluded because a block bakes big-endian next-pc immediates.
        if (jitOn && until - i >= 8u && !(st.msr & msr::LE)) {
            OPM_PH(Exec);
            const bool ran =
                jitRunLine(*this, static_cast<u32>(fl - fetchLine), word, i,
                           until);
            OPM_PH(Loop);
            if (ran) {
                if (batchBreak || halted)
                    break;
                continue;
            }
        }
        OPM_COUNT(lineEntries);
        const u32 base = fl->base;
        for (;;) {
            const u32 cia = st.pc;
            const u32 insn = fl->w[word];
            const u32 row = fl->row[word];
            OPM_COUNT(lineInsns);
            if (!execFast(insn, row))
                execRow(insn, row);
            ++i;
            if (batchBreak || halted)
                goto done;
            if (i >= until || st.pc != cia + 4u || ++word == 8u ||
                fl->base != base)
                break;
        }
    }
done:
    batching = false;
    {
        OPM_MARK(Tick); // the batch's whole clock advance, in one place
        syncClock();
    }
    return i - first;
}

} // namespace opm
