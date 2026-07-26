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
    const u32 insn = bus->read32(st.pc);
    const InsnDesc* d = decode(insn);
    if (!d) {
        ++unknownWords[insn];
        halt("unknown instruction word");
        return;
    }
    const Handler fn = handlerFor(d);
    if (!fn) {
        ++unimplemented[d->mnem];
        halt(std::string("unimplemented: ") + d->mnem);
        return;
    }
    st.pc += 4;
    fn(*this, insn, *d);
    st.tb += 1; // provisional 1 tick per instruction until the P2 clock model
}

u64 Cpu::run(u64 n)
{
    u64 i = 0;
    for (; i < n && !halted; ++i)
        step();
    return i;
}

} // namespace opm
