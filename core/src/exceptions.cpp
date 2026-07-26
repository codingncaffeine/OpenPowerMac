// Exception vectoring per UM ch.4 / PEM ch.6.
//
// SRR1 composition: bits 0,5-9,16-31 copy from MSR; bits 1-4,10-15 carry
// exception-specific status (the `extra` argument).
// MSR transition: VEC/POW/EE/PR/FP/FE0/SE/BE/FE1/PM/IR/DR/RI cleared,
// LE <- ILE, ME/IP/ILE preserved. Machine check additionally clears ME.

#include "opm/cpu.hpp"

namespace opm {

void Cpu::raiseExc(Exc v, u32 srr0, u32 extra)
{
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
}

} // namespace opm
