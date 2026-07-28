// Execution loop and handler binding.
//
// step(): fetch, decode, pre-increment PC (so handlers see NIA in st.pc and
// compute CIA as st.pc - 4), dispatch. Undecodable or not-yet-implemented
// instructions halt with a census entry — the pre-P2 stand-in for the
// program-exception path.

#include "opm/cpu.hpp"
#include "opm/bits.hpp"
#include <cstring>
#include <vector>

namespace opm {

namespace {

std::vector<Handler>& slots()
{
    static std::vector<Handler> v(kIsaCount, nullptr);
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
            tick(1);
            return;
        }
        napping = false;
    }

    // Async exceptions at the instruction boundary, EE-gated.
    // Priority per UM ch.4: SMI, then external, then decrementer.
    if (st.msr & msr::EE) {
        if (smiPending) {
            smiPending = false;
            raiseExc(Exc::Smi, st.pc, 0);
            return;
        }
        if (extIrqLine) {
            raiseExc(Exc::External, st.pc, 0);
            return;
        }
        if (decPending) {
            decPending = false;
            ++decIrqs;
            raiseExc(Exc::Decrementer, st.pc, 0);
            return;
        }
        if (pmPending) {
            pmPending = false;
            raiseExc(Exc::PerfMon, st.pc, 0);
            return;
        }
    }

    const u32 cia = st.pc;

    // IABR: address breakpoint on the instruction about to execute.
    // IABR[30]=enable, IABR[31]=translation mode must equal MSR[IR].
    if ((st.iabr & 2u) && ((cia ^ st.iabr) & 0xFFFFFFFCu) == 0 &&
        ((st.iabr & 1u) != 0) == ((st.msr & msr::IR) != 0)) {
        raiseExc(Exc::Iabr, cia, 0);
        tick(1);
        return;
    }

    u32 insn;
    if (!fetch32(cia, insn)) { // ISI raised by translate()
        tick(1);
        return;
    }
    curInsn = insn;
    const InsnDesc* d = decode(insn);
    if (!d) {
        ++unknownWords[insn];
        raiseExc(Exc::Program, cia, kSrr1ProgIllegal);
        tick(1);
        return;
    }
    if (d->flags & FL_ILL7400) {
        raiseExc(Exc::Program, cia, kSrr1ProgIllegal);
        tick(1);
        return;
    }
    if ((d->flags & FL_PRIV) && userMode()) {
        raiseExc(Exc::Program, cia, kSrr1ProgPrivileged);
        tick(1);
        return;
    }
    if (isFpInsn(*d) && !(st.msr & msr::FP)) {
        raiseExc(Exc::FpUnavailable, cia, 0);
        tick(1);
        return;
    }
    if (isVecInsn(*d) && !(st.msr & msr::VEC)) {
        raiseExc(Exc::VecUnavailable, cia, 0);
        tick(1);
        return;
    }

    const Handler fn = handlerFor(d);
    if (!fn) {
        ++unimplemented[d->mnem];
        halt(std::string("unimplemented: ") + d->mnem);
        return;
    }

    st.pc += 4;
    const u32 fallThrough = st.pc;
    fn(*this, insn, *d);
    tick(1);

    // Performance monitor, minimal-honest: PMC1/PMC2 count cycles (event 1,
    // one per instruction at the provisional 1 cycle/insn rate) or completed
    // instructions (event 2) unless globally frozen; a counter's MSB going
    // 0->1 with PMC1CE/PMCnCE and PMXE requests the (EE-gated) interrupt.
    if (!(st.mmcr0 & 0x80000000u)) { // FC
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
