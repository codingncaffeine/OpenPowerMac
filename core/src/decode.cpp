// Decode dispatch tables, built once from the ISA table (kIsa).
//
// Space layout (see Xk in insn.hpp): primary opcode selects either a direct
// D/I/B/M-form instruction or a sub-table keyed by the extended opcode. XO-form
// integer arithmetic (FL_OE) registers both the OE=0 and OE=1 slots. VXR-form
// AltiVec compares register both record-bit halves of the 11-bit VX index.

#include "opm/insn.hpp"
#include "opm/bits.hpp"
#include <array>

namespace opm {

namespace {

struct Tables {
    std::array<const InsnDesc*, 64>   pri{};
    std::array<const InsnDesc*, 1024> x19{};
    std::array<const InsnDesc*, 1024> x31{};
    std::array<const InsnDesc*, 1024> x63{};
    std::array<const InsnDesc*, 32>   a59{};
    std::array<const InsnDesc*, 32>   a63{};
    std::array<const InsnDesc*, 64>   va{};
    std::array<const InsnDesc*, 2048> vx{};
};

Tables buildTables()
{
    Tables t{};
    for (size_t n = 0; n < kIsaCount; ++n) {
        const InsnDesc& d = kIsa[n];
        switch (d.kind) {
        case Xk::PRI:  t.pri[d.primary] = &d; break;
        case Xk::X19:  t.x19[d.xo] = &d; break;
        case Xk::X31:
            t.x31[d.xo] = &d;
            if (d.flags & FL_OE)
                t.x31[d.xo + 512u] = &d;
            break;
        case Xk::A59:  t.a59[d.xo] = &d; break;
        case Xk::A63:  t.a63[d.xo] = &d; break;
        case Xk::X63:  t.x63[d.xo] = &d; break;
        case Xk::VA4:  t.va[d.xo] = &d; break;
        case Xk::VX4:  t.vx[d.xo] = &d; break;
        case Xk::VXR4:
            t.vx[d.xo] = &d;
            t.vx[d.xo + 1024u] = &d;
            break;
        }
    }
    return t;
}

const Tables& tables()
{
    static const Tables t = buildTables();
    return t;
}

} // namespace

const InsnDesc* decode(u32 insn)
{
    const Tables& t = tables();
    const u32 op = f_primary(insn);
    switch (op) {
    case 4: {
        if (const InsnDesc* d = t.va[f_xo6v(insn)])
            return d;
        return t.vx[f_xo11v(insn)];
    }
    case 19: return t.x19[f_xo10(insn)];
    case 31: return t.x31[f_xo10(insn)];
    case 59: return t.a59[f_xo5(insn)];
    case 63: {
        const u32 xo5 = f_xo5(insn);
        if (xo5 >= 16u)
            return t.a63[xo5];
        return t.x63[f_xo10(insn)];
    }
    default: return t.pri[op];
    }
}

} // namespace opm
