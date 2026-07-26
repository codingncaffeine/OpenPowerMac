// FPU executors (P4): arithmetic, fused multiply-add, compares, converts,
// estimates, moves, and the FPSCR instruction set. Numeric results come from
// the deterministic softfloat core (softfp.cpp, PEM Appendix D models);
// this file owns the FPSCR state machine of PEM 3.3.6:
//   - sticky exception bits + FX on any 0->1 transition,
//   - FR/FI rewritten by every arithmetic/rounding/conversion op,
//   - FPRF from the delivered result (not set when the op is suppressed
//     by an enabled invalid/zero-divide, per the fadd page's rule),
//   - VX/FEX recomputed as derived ORs after every update,
//   - FEX && MSR[FE0|FE1] -> floating-point enabled program exception,
//     precise (7400: no imprecise modes, UM 4.6.7), evaluated after every
//     computational and move-to-FPSCR instruction.
//
// RECEIPTS: mffs/fctiw upper word = 0xFFF80000 (undefined); FPRF unchanged
// by fctiw (undefined); CR1 (Rc) updates even when the op traps; mcrfs does
// not evaluate the enabled-exception trap (it only clears bits).

#include "opm/cpu.hpp"
#include "opm/bits.hpp"
#include "softfp.hpp"

namespace opm {

namespace {

// FPSCR bit masks (PPC bit 0 = MSB).
inline constexpr u32 fFX = 0x80000000u;
inline constexpr u32 fFEX = 0x40000000u;
inline constexpr u32 fVX = 0x20000000u;
inline constexpr u32 fOX = 0x10000000u;
inline constexpr u32 fUX = 0x08000000u;
inline constexpr u32 fZX = 0x04000000u;
inline constexpr u32 fXX = 0x02000000u;
inline constexpr u32 fVXSNAN = 0x01000000u;
inline constexpr u32 fVXISI = 0x00800000u;
inline constexpr u32 fVXIDI = 0x00400000u;
inline constexpr u32 fVXZDZ = 0x00200000u;
inline constexpr u32 fVXIMZ = 0x00100000u;
inline constexpr u32 fVXVC = 0x00080000u;
inline constexpr u32 fFR = 0x00040000u;
inline constexpr u32 fFI = 0x00020000u;
inline constexpr u32 fFPRF = 0x0001F000u;
inline constexpr u32 fVXSOFT = 0x00000400u;
inline constexpr u32 fVXSQRT = 0x00000200u;
inline constexpr u32 fVXCVI = 0x00000100u;
inline constexpr u32 fVE = 0x00000080u;
inline constexpr u32 fOE = 0x00000040u;
inline constexpr u32 fUE = 0x00000020u;
inline constexpr u32 fZE = 0x00000010u;
inline constexpr u32 fXE = 0x00000008u;
inline constexpr u32 fNI = 0x00000004u;
inline constexpr u32 fRN = 0x00000003u;

inline constexpr u32 fVxAll = fVXSNAN | fVXISI | fVXIDI | fVXZDZ | fVXIMZ |
                              fVXVC | fVXSOFT | fVXSQRT | fVXCVI;
inline constexpr u32 fExcAll = fFX | fOX | fUX | fZX | fXX | fVxAll;

// Derived bits: VX = OR of the invalid bits; FEX = OR of enabled exceptions.
void refresh(u32& f)
{
    f = (f & ~fVX) | ((f & fVxAll) ? fVX : 0);
    const bool fex = ((f & fVX) && (f & fVE)) || ((f & fOX) && (f & fOE)) ||
                     ((f & fUX) && (f & fUE)) || ((f & fZX) && (f & fZE)) ||
                     ((f & fXX) && (f & fXE));
    f = (f & ~fFEX) | (fex ? fFEX : 0);
}

sf::Env env(const Cpu& c)
{
    const u32 f = c.st.fpscr;
    return {f & fRN, (f & fOE) != 0, (f & fUE) != 0, (f & fNI) != 0};
}

// FPRF codes per PEM Table 3-10 (C || FPCC). Single-precision ops classify
// against single thresholds: a result below 2^-126 is a ±denormalized
// number architecturally even though the FPR holds it as a normal double.
u32 classify(u64 v, bool sgl)
{
    const bool s = (v >> 63) != 0;
    const u32 e = static_cast<u32>(v >> 52) & 0x7FFu;
    const u64 m = v & 0x000FFFFFFFFFFFFFull;
    if (e == 0x7FF)
        return m ? 0x11u : (s ? 0x09u : 0x05u);
    if (e == 0 && m == 0)
        return s ? 0x12u : 0x02u;
    if (e < (sgl ? 897u : 1u))
        return s ? 0x18u : 0x14u;
    return s ? 0x08u : 0x04u;
}

enum class Fprf { None, Dbl, Sgl };

void setFprf(u32& f, u64 result, Fprf k)
{
    f = (f & ~fFPRF) | (classify(result, k == Fprf::Sgl) << 12);
}

void fpRc(Cpu& c, u32 i)
{
    if (f_rcbit(i))
        c.setCrField(1, c.st.fpscr >> 28); // FX FEX VX OX
}

void trapIfEnabled(Cpu& c)
{
    if ((c.st.fpscr & fFEX) && (c.st.msr & (msr::FE0 | msr::FE1)))
        c.raiseExc(Exc::Program, c.st.pc - 4, kSrr1ProgFpEnabled);
}

u32 mapFlags(u32 sfl)
{
    u32 exc = 0;
    if (sfl & sf::kOx) exc |= fOX;
    if (sfl & sf::kUx) exc |= fUX;
    if (sfl & sf::kZx) exc |= fZX;
    if (sfl & sf::kXx) exc |= fXX;
    if (sfl & sf::kVxsnan) exc |= fVXSNAN;
    if (sfl & sf::kVxisi) exc |= fVXISI;
    if (sfl & sf::kVxidi) exc |= fVXIDI;
    if (sfl & sf::kVxzdz) exc |= fVXZDZ;
    if (sfl & sf::kVximz) exc |= fVXIMZ;
    if (sfl & sf::kVxcvi) exc |= fVXCVI;
    if (sfl & sf::kVxsqrt) exc |= fVXSQRT;
    return exc;
}

// Common tail for arithmetic/rounding/conversion ops.
void commit(Cpu& c, u32 i, const sf::R& r, Fprf fprf)
{
    u32& f = c.st.fpscr;
    const u32 exc = mapFlags(r.fl);
    const bool suppress = ((exc & fVxAll) && (f & fVE)) ||
                          ((exc & fZX) && (f & fZE));
    if (exc & ~f)
        f |= fFX; // some exception bit transitions 0 -> 1
    f |= exc;
    f &= ~(fFR | fFI);
    if (r.fl & sf::kXx)
        f |= fFI;
    if (r.fl & sf::kFr)
        f |= fFR;
    if (!suppress) {
        c.st.fpr[f_rt(i)] = r.bits;
        if (fprf != Fprf::None)
            setFprf(f, r.bits, fprf);
    }
    refresh(f);
    fpRc(c, i);
    trapIfEnabled(c);
}

inline u64 A(Cpu& c, u32 i) { return c.st.fpr[f_ra(i)]; }
inline u64 B(Cpu& c, u32 i) { return c.st.fpr[f_rb(i)]; }
inline u64 C_(Cpu& c, u32 i) { return c.st.fpr[f_rc_vec(i)]; }

// ---- arithmetic ------------------------------------------------------------

void h_fadd(Cpu& c, u32 i, const InsnDesc&)  { commit(c, i, sf::add(A(c, i), B(c, i), env(c), sf::Tgt::Dbl), Fprf::Dbl); }
void h_fadds(Cpu& c, u32 i, const InsnDesc&) { commit(c, i, sf::add(A(c, i), B(c, i), env(c), sf::Tgt::Sgl), Fprf::Sgl); }
void h_fsub(Cpu& c, u32 i, const InsnDesc&)  { commit(c, i, sf::sub(A(c, i), B(c, i), env(c), sf::Tgt::Dbl), Fprf::Dbl); }
void h_fsubs(Cpu& c, u32 i, const InsnDesc&) { commit(c, i, sf::sub(A(c, i), B(c, i), env(c), sf::Tgt::Sgl), Fprf::Sgl); }
void h_fmul(Cpu& c, u32 i, const InsnDesc&)  { commit(c, i, sf::mul(A(c, i), C_(c, i), env(c), sf::Tgt::Dbl), Fprf::Dbl); }
void h_fmuls(Cpu& c, u32 i, const InsnDesc&) { commit(c, i, sf::mul(A(c, i), C_(c, i), env(c), sf::Tgt::Sgl), Fprf::Sgl); }
void h_fdiv(Cpu& c, u32 i, const InsnDesc&)  { commit(c, i, sf::div(A(c, i), B(c, i), env(c), sf::Tgt::Dbl), Fprf::Dbl); }
void h_fdivs(Cpu& c, u32 i, const InsnDesc&) { commit(c, i, sf::div(A(c, i), B(c, i), env(c), sf::Tgt::Sgl), Fprf::Sgl); }

#define MADD(NAME, TGT, NEGADD, NEGRES)                                       \
    void NAME(Cpu& c, u32 i, const InsnDesc&)                                 \
    {                                                                         \
        commit(c, i,                                                          \
               sf::madd(A(c, i), C_(c, i), B(c, i), env(c), sf::Tgt::TGT,     \
                        NEGADD, NEGRES),                                      \
               Fprf::TGT);                                                    \
    }
MADD(h_fmadd, Dbl, false, false)
MADD(h_fmadds, Sgl, false, false)
MADD(h_fmsub, Dbl, true, false)
MADD(h_fmsubs, Sgl, true, false)
MADD(h_fnmadd, Dbl, false, true)
MADD(h_fnmadds, Sgl, false, true)
MADD(h_fnmsub, Dbl, true, true)
MADD(h_fnmsubs, Sgl, true, true)
#undef MADD

void h_frsp(Cpu& c, u32 i, const InsnDesc&) { commit(c, i, sf::rsp(B(c, i), env(c)), Fprf::Sgl); }
void h_fres(Cpu& c, u32 i, const InsnDesc&) { commit(c, i, sf::res(B(c, i), env(c)), Fprf::Sgl); }
void h_frsqrte(Cpu& c, u32 i, const InsnDesc&) { commit(c, i, sf::rsqrte(B(c, i), env(c)), Fprf::Dbl); }

void h_fctiw(Cpu& c, u32 i, const InsnDesc&)
{
    commit(c, i, sf::ctiw(B(c, i), c.st.fpscr & fRN), Fprf::None);
}
void h_fctiwz(Cpu& c, u32 i, const InsnDesc&)
{
    commit(c, i, sf::ctiw(B(c, i), 1u), Fprf::None);
}

// ---- select / moves --------------------------------------------------------

inline bool isNan(u64 v)
{
    return ((v >> 52) & 0x7FFu) == 0x7FFu && (v & 0x000FFFFFFFFFFFFFull) != 0;
}

void h_fsel(Cpu& c, u32 i, const InsnDesc&)
{
    const u64 a = A(c, i);
    const bool geZero = !isNan(a) && (!(a >> 63) || (a << 1) == 0);
    c.st.fpr[f_rt(i)] = geZero ? C_(c, i) : B(c, i);
    fpRc(c, i); // fsel alters no FPSCR bits (PEM D.5.4)
}

void h_fmr(Cpu& c, u32 i, const InsnDesc&)   { c.st.fpr[f_rt(i)] = B(c, i); fpRc(c, i); }
void h_fneg(Cpu& c, u32 i, const InsnDesc&)  { c.st.fpr[f_rt(i)] = B(c, i) ^ 0x8000000000000000ull; fpRc(c, i); }
void h_fabs(Cpu& c, u32 i, const InsnDesc&)  { c.st.fpr[f_rt(i)] = B(c, i) & 0x7FFFFFFFFFFFFFFFull; fpRc(c, i); }
void h_fnabs(Cpu& c, u32 i, const InsnDesc&) { c.st.fpr[f_rt(i)] = B(c, i) | 0x8000000000000000ull; fpRc(c, i); }

// ---- compares --------------------------------------------------------------

u32 fcmp(u64 a, u64 b)
{
    if (isNan(a) || isNan(b))
        return 1u; // unordered
    const bool za = (a << 1) == 0, zb = (b << 1) == 0;
    if (za && zb)
        return 2u; // +0 == -0
    const bool sa = (a >> 63) != 0, sb = (b >> 63) != 0;
    if (za)
        return sb ? 4u : 8u; // 0 vs x: greater if x negative
    if (zb)
        return sa ? 8u : 4u;
    if (sa != sb)
        return sa ? 8u : 4u;
    const u64 ma = a & 0x7FFFFFFFFFFFFFFFull, mb = b & 0x7FFFFFFFFFFFFFFFull;
    if (ma == mb)
        return 2u;
    const bool magLess = ma < mb;
    return (magLess != sa) ? 8u : 4u; // sign-aware magnitude order
}

inline bool isSNan(u64 v) { return isNan(v) && !((v >> 51) & 1u); }

void compare(Cpu& c, u32 i, bool ordered)
{
    const u64 a = A(c, i), b = B(c, i);
    const u32 cc = fcmp(a, b);
    u32& f = c.st.fpscr;
    f = (f & ~0x0000F000u) | (cc << 12); // FPCC (C bit untouched)
    c.setCrField(f_crfd(i), cc);
    u32 exc = 0;
    if (isSNan(a) || isSNan(b)) {
        exc |= fVXSNAN;
        if (ordered && !(f & fVE))
            exc |= fVXVC;
    } else if (ordered && (isNan(a) || isNan(b))) {
        exc |= fVXVC;
    }
    if (exc) {
        if (exc & ~f)
            f |= fFX;
        f |= exc;
    }
    refresh(f);
    trapIfEnabled(c);
}

void h_fcmpu(Cpu& c, u32 i, const InsnDesc&) { compare(c, i, false); }
void h_fcmpo(Cpu& c, u32 i, const InsnDesc&) { compare(c, i, true); }

// ---- FPSCR instructions ----------------------------------------------------

void h_mffs(Cpu& c, u32 i, const InsnDesc&)
{
    c.st.fpr[f_rt(i)] = 0xFFF8000000000000ull | c.st.fpscr;
    fpRc(c, i);
}

void h_mcrfs(Cpu& c, u32 i, const InsnDesc&)
{
    u32& f = c.st.fpscr;
    const u32 crfS = f_crfs(i);
    const u32 sh = (7u - crfS) * 4u;
    c.setCrField(f_crfd(i), (f >> sh) & 0xFu);
    // Copied exception bits (except FEX/VX) are cleared.
    const u32 fieldMask = 0xFu << sh;
    f &= ~(fieldMask & fExcAll & ~(fFEX | fVX));
    refresh(f);
}

void h_mtfsfi(Cpu& c, u32 i, const InsnDesc&)
{
    u32& f = c.st.fpscr;
    const u32 crfD = f_crfd(i);
    const u32 imm = ppcbits(i, 16, 19);
    const u32 sh = (7u - crfD) * 4u;
    u32 mask = 0xFu << sh;
    if (crfD == 0)
        mask &= ~(fFEX | fVX); // FEX/VX are never explicitly writable
    f = (f & ~mask) | ((imm << sh) & mask);
    refresh(f);
    fpRc(c, i);
    trapIfEnabled(c);
}

void h_mtfsf(Cpu& c, u32 i, const InsnDesc&)
{
    u32& f = c.st.fpscr;
    const u32 fm = f_fm(i);
    const u32 v = static_cast<u32>(B(c, i));
    u32 mask = 0;
    for (u32 k = 0; k < 8; ++k)
        if (fm & (0x80u >> k))
            mask |= 0xFu << ((7u - k) * 4u);
    mask &= ~(fFEX | fVX);
    f = (f & ~mask) | (v & mask);
    refresh(f);
    fpRc(c, i);
    trapIfEnabled(c);
}

void mtfsb(Cpu& c, u32 i, bool set)
{
    u32& f = c.st.fpscr;
    const u32 crbD = f_rt(i); // bit number in PPC order
    const u32 bit = 1u << (31u - crbD);
    if (bit & (fFEX | fVX)) { // not explicitly alterable
        fpRc(c, i);
        return;
    }
    if (set) {
        // mtfsb1 is not exempt from the implicit-FX rule: setting an
        // exception bit 0->1 sets FX too.
        if ((bit & fExcAll & ~fFX) && !(f & bit))
            f |= fFX;
        f |= bit;
    } else {
        f &= ~bit;
    }
    refresh(f);
    fpRc(c, i);
    trapIfEnabled(c);
}

void h_mtfsb0(Cpu& c, u32 i, const InsnDesc&) { mtfsb(c, i, false); }
void h_mtfsb1(Cpu& c, u32 i, const InsnDesc&) { mtfsb(c, i, true); }

} // namespace

void bindFpuHandlers()
{
    setHandler("fadd", h_fadd);
    setHandler("fadds", h_fadds);
    setHandler("fsub", h_fsub);
    setHandler("fsubs", h_fsubs);
    setHandler("fmul", h_fmul);
    setHandler("fmuls", h_fmuls);
    setHandler("fdiv", h_fdiv);
    setHandler("fdivs", h_fdivs);
    setHandler("fmadd", h_fmadd);
    setHandler("fmadds", h_fmadds);
    setHandler("fmsub", h_fmsub);
    setHandler("fmsubs", h_fmsubs);
    setHandler("fnmadd", h_fnmadd);
    setHandler("fnmadds", h_fnmadds);
    setHandler("fnmsub", h_fnmsub);
    setHandler("fnmsubs", h_fnmsubs);
    setHandler("frsp", h_frsp);
    setHandler("fres", h_fres);
    setHandler("frsqrte", h_frsqrte);
    setHandler("fctiw", h_fctiw);
    setHandler("fctiwz", h_fctiwz);
    setHandler("fsel", h_fsel);
    setHandler("fmr", h_fmr);
    setHandler("fneg", h_fneg);
    setHandler("fabs", h_fabs);
    setHandler("fnabs", h_fnabs);
    setHandler("fcmpu", h_fcmpu);
    setHandler("fcmpo", h_fcmpo);
    setHandler("mffs", h_mffs);
    setHandler("mcrfs", h_mcrfs);
    setHandler("mtfsfi", h_mtfsfi);
    setHandler("mtfsf", h_mtfsf);
    setHandler("mtfsb0", h_mtfsb0);
    setHandler("mtfsb1", h_mtfsb1);
}

} // namespace opm
