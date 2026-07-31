// Exception vectoring per UM ch.4 / PEM ch.6.
//
// SRR1 composition: bits 0,5-9,16-31 copy from MSR; bits 1-4,10-15 carry
// exception-specific status (the `extra` argument).
// MSR transition: VEC/POW/EE/PR/FP/FE0/SE/BE/FE1/PM/IR/DR/RI cleared,
// LE <- ILE, ME/IP/ILE preserved. Machine check additionally clears ME.

#include "opm/cpu.hpp"
#include "opm/prof.hpp"

namespace opm {

void Cpu::raiseExc(Exc v, u32 srr0, u32 extra)
{
    // Scoped, because this is reached from an async check at the top of step()
    // — before the fetch marker, so its cost used to land in whatever phase
    // the machine loop left set — and from inside a handler, where the
    // enclosing phase has to come back. See the note on prof::Ph::Exc.
    OPM_MARK(Exc);
    raisedThisStep = true;
    st.srr0 = srr0;
    st.srr1 = (st.msr & 0x87C0FFFFu) | (extra & 0x783F0000u);

    u32 m = st.msr & (msr::ME | msr::IP | msr::ILE);
    if (st.msr & msr::ILE)
        m |= msr::LE;
    if (v == Exc::MachineCheck)
        m &= ~msr::ME;
    st.msr = m;

    const u32 base = (st.msr & msr::IP) ? 0xFFF00000u : 0x00000000u;
    st.pc = base | static_cast<u32>(v);

    // 📓 THE LAST FEW EXCEPTIONS, kept so a stopped machine can be asked what
    // it has been doing rather than only where it ended up. One sample of a
    // guest sitting in a handler cannot say whether it arrived once and stayed,
    // or is re-taking the same fault forever — and those are opposite bugs. A
    // ring of the same vector at the same DAR is the second one, visibly.
    //
    // ⚠ On Cpu, never in CpuState: the snapshot layout digest hashes
    // CpuState's size, so putting it there would kill every existing snapshot
    // to gain an instrument.
    ExcRec& r = excRing[excRingAt & (kExcRing - 1u)];
    r = {static_cast<u32>(v), srr0, st.srr1, st.dar, st.dsisr, st.tb};
    ++excRingAt;
    ++excByVec[(static_cast<u32>(v) >> 8) & 15u];
}

} // namespace opm
