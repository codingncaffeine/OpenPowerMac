// Execution loop. At P0 this is fetch + decode + census: every decoded
// instruction whose handler is not yet bound lands in the `unimplemented`
// census, undecodable words in `unknownWords`. Handlers bind per phase.

#include "opm/cpu.hpp"
#include "opm/bits.hpp"

namespace opm {

void Cpu::step()
{
    const u32 insn = bus->read32(st.pc);
    const InsnDesc* d = decode(insn);
    if (!d) {
        ++unknownWords[insn];
    } else if (!d->fn) {
        ++unimplemented[d->mnem];
    } else {
        st.pc += 4;
        d->fn(*this, insn, *d);
        return;
    }
    st.pc += 4;
}

u64 Cpu::run(u64 n)
{
    for (u64 i = 0; i < n; ++i)
        step();
    return n;
}

} // namespace opm
