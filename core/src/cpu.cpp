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
            raiseExc(Exc::Decrementer, st.pc, 0);
            return;
        }
    }

    const u32 cia = st.pc;
    const u32 insn = bus->read32(cia);
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
