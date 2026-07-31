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
            tick(1 + extraCycles);
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
            tick(extraCycles);
            return;
        }
        if (extIrqLine) {
            ++extIrqs;
            raiseExc(Exc::External, st.pc, 0);
            tick(extraCycles);
            return;
        }
        if (decPending) {
            decPending = false;
            ++decIrqs;
            raiseExc(Exc::Decrementer, st.pc, 0);
            tick(extraCycles);
            return;
        }
        if (pmPending) {
            pmPending = false;
            raiseExc(Exc::PerfMon, st.pc, 0);
            tick(extraCycles);
            return;
        }
    }

    const u32 cia = st.pc;

    // IABR: address breakpoint on the instruction about to execute.
    // IABR[30]=enable, IABR[31]=translation mode must equal MSR[IR].
    if ((st.iabr & 2u) && ((cia ^ st.iabr) & 0xFFFFFFFCu) == 0 &&
        ((st.iabr & 1u) != 0) == ((st.msr & msr::IR) != 0)) {
        raiseExc(Exc::Iabr, cia, 0);
        tick(1 + extraCycles);
        return;
    }

    u32 insn, row;
    OPM_PH(Fetch);
    if (!fetchDecoded(cia, insn, row)) { // ISI raised by translate()
        tick(1 + extraCycles);
        return;
    }
    OPM_PH(Decode);
    curInsn = insn;
    if (row == kNoRow) {
        ++unknownWords[insn];
        raiseExc(Exc::Program, cia, kSrr1ProgIllegal);
        tick(1 + extraCycles);
        return;
    }
    const InsnDesc* d = kIsa + row;
    // The pre-dispatch gates, in the architecture's priority order, read out
    // of the table instead of recomputed. Almost every instruction gates on
    // nothing, so the common path is one load and one branch.
    if (const u8 gate = dispPre[row]) {
        if (gate & kPreIll) {
            raiseExc(Exc::Program, cia, kSrr1ProgIllegal);
            tick(1 + extraCycles);
            return;
        }
        if ((gate & kPrePriv) && userMode()) {
            raiseExc(Exc::Program, cia, kSrr1ProgPrivileged);
            tick(1 + extraCycles);
            return;
        }
        if ((gate & kPreFp) && !(st.msr & msr::FP)) {
            raiseExc(Exc::FpUnavailable, cia, 0);
            tick(1 + extraCycles);
            return;
        }
        if ((gate & kPreVec) && !(st.msr & msr::VEC)) {
            raiseExc(Exc::VecUnavailable, cia, 0);
            tick(1 + extraCycles);
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
    tick(1 + extraCycles);

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

} // namespace opm
