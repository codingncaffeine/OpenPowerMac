// Deterministic software FP per PEM Appendix D models. See softfp.hpp.
//
// Internal conventions:
//   - Finite nonzero significands are held normalized with the leading 1 in
//     bit 63 ("norm63"); the value is sig * 2^(exp-63), i.e. 1.x * 2^exp.
//   - Rounding follows PEM D.1: guard/round/sticky relative to the kept
//     precision (53 or 24), increment per the Round Float model.
//   - The fused path follows D.2: exact 106-bit product, 128-bit aligned
//     add with jam bit, single rounding.
//
// RECEIPTS (architecturally undefined, pinned deterministically here):
//   - Overflow-disabled default sets FR=1 when the default is infinity,
//     FR=0 when it is the format's max (PEM: FR undefined).
//   - fnmadd/fnmsub do not negate a NaN result (PEM D.2 note conflicts with
//     the NaN-propagation rule; silicon convention).
//   - fres/frsqrte are correctly-rounded compositions (well inside the
//     1/256 and 1/32 bounds), not the silicon lookup tables — ledger row.
//     Their FR/FI are cleared (PEM: undefined).
//   - storeSingle with exponent below 874 continues the D.7 denormalize
//     loop to its fixed point: a signed zero (PEM: WORD undefined).
//   - NI flush-to-zero keeps the flags the IEEE path computed.

#include "softfp.hpp"
#include <bit>
#include <cmath>

#if defined(_M_X64) || defined(__x86_64__)
#define OPM_SF_X64 1
#include <immintrin.h>
#if defined(_MSC_VER)
#include <intrin.h>
#endif
#else
#define OPM_SF_X64 0
#endif

namespace opm::sf {

u32 gHostFastOff = 0;
u64 gFastHits = 0, gFastMiss = 0;
u64 gFastHitsD = 0, gFastMissD = 0;

#if OPM_SF_X64
// The fused path of the fast case wants a HARDWARE fma: the C runtime's
// fmaf is a correctly-rounded software routine on MSVC's default arch, and
// paying a libcall per fmadds is how the fast path measured no faster than
// the model it bypassed. FMA3 is checked once; a host without it simply
// leaves the madd family to the model (adds and muls need no fma at all).
static bool haveFma3()
{
#if defined(_MSC_VER)
    int info[4] = {};
    __cpuid(info, 1);
    return (info[2] & (1 << 12)) != 0;
#else
    return __builtin_cpu_supports("fma");
#endif
}
static const bool kHaveFma = haveFma3();
#if defined(__GNUC__)
__attribute__((target("fma")))
#endif
static float hwFmaf(float a, float b, float c)
{
    return _mm_cvtss_f32(
        _mm_fmadd_ss(_mm_set_ss(a), _mm_set_ss(b), _mm_set_ss(c)));
}
#if defined(__GNUC__)
__attribute__((target("fma")))
#endif
static double hwFmad(double a, double b, double c)
{
    return _mm_cvtsd_f64(
        _mm_fmadd_sd(_mm_set_sd(a), _mm_set_sd(b), _mm_set_sd(c)));
}
#endif

// One name for "a*b+c with a single rounding", whatever the host is. On x64
// this is the FMA3 unit and the caller has already checked kHaveFma; anywhere
// else it is the C runtime's correctly-rounded routine, which is right but
// not fast — and the double fast path below is a speed measure, so a host
// without FMA3 simply leaves those rows with the model.
inline double fmaExact(double a, double b, double c)
{
#if OPM_SF_X64
    return hwFmad(a, b, c);
#else
    return std::fma(a, b, c);
#endif
}

namespace {

inline constexpr u64 kSignBit = 0x8000000000000000ull;
inline constexpr u64 kQNaNGen = 0x7FF8000000000000ull;
inline constexpr u64 kOne = 0x3FF0000000000000ull;
inline constexpr u64 kInf = 0x7FF0000000000000ull;
inline constexpr u64 kMaxDbl = 0x7FEFFFFFFFFFFFFFull;
inline constexpr u64 kMaxSglAsDbl = 0x47EFFFFFE0000000ull;

enum Cls : u8 { cZero, cFin, cInf, cQNaN, cSNaN };

struct Un {
    Cls c;
    bool s;
    i32 e;   // unbiased exponent of the leading 1 (cFin only)
    u64 m;   // norm63 significand (cFin only)
};

Un unpack(u64 f)
{
    Un u;
    u.s = (f >> 63) != 0;
    const u32 e = static_cast<u32>(f >> 52) & 0x7FFu;
    const u64 frac = f & 0x000FFFFFFFFFFFFFull;
    if (e == 0x7FF) {
        u.c = frac == 0 ? cInf : ((frac >> 51) ? cQNaN : cSNaN);
        u.e = 0;
        u.m = 0;
        return u;
    }
    if (e == 0) {
        if (frac == 0) {
            u.c = cZero;
            u.e = 0;
            u.m = 0;
            return u;
        }
        const int lz = std::countl_zero(frac);
        u.c = cFin;
        u.m = frac << lz;
        u.e = (63 - lz) - 1074;
        return u;
    }
    u.c = cFin;
    u.m = (frac | (1ull << 52)) << 11;
    u.e = static_cast<i32>(e) - 1023;
    return u;
}

inline u64 quiet(u64 nan) { return nan | (1ull << 51); }
inline u64 packZero(bool s) { return s ? kSignBit : 0; }
inline u64 packInf(bool s) { return kInf | (s ? kSignBit : 0); }

// Pack an exact norm63 value as a double (handles double-format denorms).
u64 packExact(bool sign, i32 vexp, u64 nsig)
{
    const u64 s = sign ? kSignBit : 0;
    if (vexp < -1022) {
        const u32 sh = static_cast<u32>(-1011 - vexp); // nsig * 2^(vexp+1011)
        return s | (sh >= 64 ? 0 : nsig >> sh);
    }
    return s | (static_cast<u64>(vexp + 1023) << 52) | ((nsig << 1) >> 12);
}

// PEM D.1 rounding + PEM 3.3.6.2 overflow/underflow flow.
// Value in: sig norm63, true exponent exp, extra sticky below bit 0.
u64 roundPack(bool sign, i32 exp, u64 sig, bool sticky, const Env& env,
              Tgt t, u32& fl)
{
    const int p = t == Tgt::Sgl ? 24 : 53;
    const i32 emin = t == Tgt::Sgl ? -126 : -1022;
    const i32 emax = t == Tgt::Sgl ? 127 : 1023;
    const i32 scale = t == Tgt::Sgl ? 192 : 1536;

    const bool tiny = exp < emin; // tininess detected before rounding
    if (tiny) {
        if (env.ue) {
            exp += scale;
            fl |= kUx;
        } else {
            const u32 sh = static_cast<u32>(emin - exp);
            if (sh >= 64) {
                sticky = sticky || sig != 0;
                sig = 0;
            } else if (sh) {
                sticky = sticky || (sig << (64 - sh)) != 0;
                sig >>= sh;
            }
            exp = emin;
        }
    }

    const u64 lsb = (sig >> (64 - p)) & 1u;
    const u64 g = (sig >> (63 - p)) & 1u;
    const u64 r = (sig >> (62 - p)) & 1u;
    const bool x = sticky || (sig & ((1ull << (62 - p)) - 1u)) != 0;
    const bool inx = g || r || x;
    u32 inc = 0;
    switch (env.rn) {
    case 0: inc = g && (r || x || lsb); break;         // nearest-even
    case 1: inc = 0; break;                            // toward zero
    case 2: inc = !sign && inx; break;                 // toward +inf
    case 3: inc = sign && inx; break;                  // toward -inf
    }
    u64 kept = (sig >> (64 - p)) + inc;
    if (kept >> p) {
        kept >>= 1;
        exp += 1;
    }
    if (inx)
        fl |= kXx;
    if (inc)
        fl |= kFr;
    if (tiny && !env.ue && inx)
        fl |= kUx;

    if (!tiny && exp > emax) { // overflow detected after rounding
        fl |= kOx;
        if (env.oe) {
            exp -= scale;
        } else {
            fl |= kXx;
            const bool toInf = env.rn == 0 || (env.rn == 2 && !sign) ||
                               (env.rn == 3 && sign);
            if (toInf)
                fl |= kFr; // RECEIPT: FR undefined -> tracks the default
            else
                fl &= ~kFr;
            if (toInf)
                return packInf(sign);
            return (sign ? kSignBit : 0) |
                   (t == Tgt::Sgl ? kMaxSglAsDbl : kMaxDbl);
        }
    }

    if (kept == 0)
        return packZero(sign); // tiny value rounded away

    const int m = 63 - std::countl_zero(kept);
    const i32 vexp = exp - (p - 1) + m;
    if (env.ni && vexp < emin) // NI: denormalized result flushes to zero
        return packZero(sign);
    return packExact(sign, vexp, kept << std::countl_zero(kept));
}

void mul64to128(u64 a, u64 b, u64& hi, u64& lo)
{
    const u64 a0 = static_cast<u32>(a), a1 = a >> 32;
    const u64 b0 = static_cast<u32>(b), b1 = b >> 32;
    const u64 p00 = a0 * b0, p01 = a0 * b1, p10 = a1 * b0, p11 = a1 * b1;
    u64 mid = p01 + p10;
    const u64 midCarry = mid < p01 ? (1ull << 32) : 0;
    lo = p00 + (mid << 32);
    hi = p11 + (mid >> 32) + midCarry + (lo < p00 ? 1 : 0);
}

struct U128 {
    u64 hi, lo;
};

inline U128 shl128(U128 v, u32 n)
{
    if (n == 0) return v;
    if (n >= 64) return {n >= 128 ? 0 : v.lo << (n - 64), 0};
    return {(v.hi << n) | (v.lo >> (64 - n)), v.lo << n};
}

// Right shift with jam: bits shifted out OR into `jam`.
inline U128 shr128jam(U128 v, u32 n, bool& jam)
{
    if (n == 0) return v;
    if (n >= 128) {
        jam = jam || v.hi != 0 || v.lo != 0;
        return {0, 0};
    }
    if (n >= 64) {
        jam = jam || v.lo != 0 || (n > 64 && (v.hi << (128 - n)) != 0);
        return {0, v.hi >> (n - 64)};
    }
    jam = jam || (v.lo << (64 - n)) != 0;
    return {v.hi >> n, (v.lo >> n) | (v.hi << (64 - n))};
}

inline int clz128(U128 v)
{
    return v.hi ? std::countl_zero(v.hi) : 64 + std::countl_zero(v.lo);
}

inline bool ge128(U128 a, U128 b)
{
    return a.hi != b.hi ? a.hi > b.hi : a.lo >= b.lo;
}

inline U128 sub128(U128 a, U128 b)
{
    return {a.hi - b.hi - (a.lo < b.lo ? 1 : 0), a.lo - b.lo};
}

inline U128 add128(U128 a, U128 b, bool& carry)
{
    U128 r{a.hi + b.hi, a.lo + b.lo};
    if (r.lo < a.lo) {
        if (++r.hi == 0)
            carry = true;
    }
    if (r.hi < a.hi)
        carry = true;
    return r;
}

// The one generic path: rounded( (a*c) [+/- b] ), optionally negated.
// Handles fadd/fsub (c = 1.0), fmul (no addend), and the fmadd family.
// NaN precedence is frA, frB, frC (PEM 3.3.4); only frsp truncates payloads.
// ⭐ THE SINGLE-TARGET HOST FAST PATH. Measured in-game (s40): the JIT left
// ~27% of the window in softfp arithmetic, and the census top was all
// single-precision (fmadds/fmuls/fadds/fnmsubs/fsubs). For SINGLE ops with
// single-valued operands the host can produce the model's exact answer:
//
//   result   float add / float mul-of-exact-double-product / fmaf are each
//            the correct rounding of the exact value — for the pure product,
//            (float)(double(sa)*double(sc)) has NO double-rounding hazard
//            because the 24x24-bit product is EXACT in a double.
//   flags    FI ("this op was inexact") and FR ("the fraction was
//            incremented") need sign-exact knowledge of exact - result.
//            With S,T = 2Sum(pd, db) (exact by the 2Sum lemma; no double
//            over/underflow is possible with single-range inputs), and
//            d = S - r0d (EXACT: S and r0d are 53- and 24-bit roundings of
//            the same value, so they agree to a factor [1-2^-23, 1+2^-23]
//            and Sterbenz applies; the S==0 and r0==0 corners are handled
//            separately): when d != 0, |d| >= ulp53(S) > ulp53(S)/2 >= |T|,
//            so sign(exact - r0) = sign(d) and it cannot cancel to zero;
//            when d == 0, exact - r0 = T exactly. No fenv, no MXCSR.
//
// Everything else — NaN/Inf/denormal operands, non-nearest rounding,
// enabled-exception scaling, NI, overflow, denormal-or-underflowed results,
// operands that are not single-valued (whose Sgl result the model defines) —
// bails into the model below, which remains the single source of truth.
bool hostSglFast(u64 fa, u64 fc, u64 fb, bool hasAddend, bool negResult,
                 R& out)
{
    const auto nzD = [](u64 x) {
        const u32 e = static_cast<u32>(x >> 52) & 0x7FFu;
        return e ? e != 0x7FFu : (x & ~kSignBit) == 0;
    };
    if (!nzD(fa) || !nzD(fc) || (hasAddend && !nzD(fb)))
        return false;
    const double da = std::bit_cast<double>(fa);
    const double dc = std::bit_cast<double>(fc);
    const double db = hasAddend ? std::bit_cast<double>(fb) : 0.0;
    const float sa = static_cast<float>(da);
    const float sc = static_cast<float>(dc);
    const float sb = static_cast<float>(db);
    if (static_cast<double>(sa) != da || static_cast<double>(sc) != dc ||
        (hasAddend && static_cast<double>(sb) != db))
        return false; // not single-valued: the model's territory
    const double pd = static_cast<double>(sa) * static_cast<double>(sc);
    float r0;
    double S, T;
    if (hasAddend) {
        // Correctly rounded single of the exact a*c+b. A true multiplier of
        // one (every add/sub arrives as a*1+b) needs no fma — float add is
        // native and correctly rounded; the genuine madd family takes the
        // FMA3 unit, and a host without FMA3 leaves it to the model.
        if (sc == 1.0f) {
            r0 = sa + sb;
        } else {
#if OPM_SF_X64
            if (!kHaveFma)
                return false;
            r0 = hwFmaf(sa, sc, sb);
#else
            r0 = std::fmaf(sa, sc, sb);
#endif
        }
        S = pd + db; // 2Sum: S + T == pd + db exactly
        const double bv = S - pd;
        T = (pd - (S - bv)) + (db - bv);
    } else {
        r0 = static_cast<float>(pd); // exact product: one rounding only
        S = pd;
        T = 0.0;
    }
    const u32 rbits = std::bit_cast<u32>(r0);
    const u32 rexp = (rbits >> 23) & 0xFFu;
    if (rexp == 0xFFu)
        return false; // overflowed: the model owns OX and the defaults
    if (rexp == 0u && (rbits & 0x007FFFFFu))
        return false; // denormal result: tininess is the model's business
    // ⚠ ANY 2Sum RESIDUE BAILS. T != 0 means the addend reaches below the
    // 53-bit window of the sum — jam territory in the model's aligned-add,
    // where BOTH deliverables leave ideal arithmetic behind: the jam is an
    // unsigned sticky, so a far-below addend breaks a product tie upward
    // where true fused rounding (fmaf) breaks it by the addend's sign
    // (result bits differ by one ulp), and FR follows the window's own
    // increment, not the magnitude. Measured, not theorized: the
    // differential produced both families on its first runs. With T == 0
    // the exact value IS the double S, the model's 106-bit accumulator
    // holds it entirely, and model == ideal — provably.
    if (T != 0.0)
        return false;
    const double r0d = static_cast<double>(r0); // exact widening
    const double resid = S - r0d; // exact: Sterbenz (S, r0d agree to 2^-23)
    if (r0 == 0.0f && resid != 0.0)
        return false; // underflowed to zero: UX belongs to the model
    u32 fl = 0;
    if (resid != 0.0) {
        fl |= kXx;
        if ((resid < 0.0) != ((rbits >> 31) != 0))
            fl |= kFr; // rounded away from zero
    }
    u64 bits = std::bit_cast<u64>(static_cast<double>(r0));
    if (negResult)
        bits ^= kSignBit;
    out = {bits, fl};
    return true;
}

// ⭐ THE DOUBLE-TARGET HOST FAST PATH. s41's census sorted the fallback by
// host CYCLES and the double rows were the top of what was left: fsub 216
// cycles a call, fmsub 193, fmadd 190, fmul 184, against 114-130 for the
// single rows the path above had already lowered. They had no fast path at
// all, because the single path's admission does not generalize: it accepts
// when the exact result FITS IN A DOUBLE, and a double product is 106 bits
// and never does.
//
// So this one admits on THE MODEL'S OWN GEOMETRY instead. fmaCore below
// accumulates into a 128-bit window with the product at the top and folds
// whatever the alignment shift pushes past the bottom into `jam`, an
// UNSIGNED sticky. While nothing falls off, that window holds the exact
// value and rounds it exactly once — which is the definition of the host's
// own add/mul/fma, so the bits agree by construction. Once something does
// fall off, the model deliberately parts company with ideal arithmetic (s40
// measured both families that produces), and we bail to it.
//
// Three admissions, all on raw fields:
//   operands  ±0, or normal inside an exponent band chosen so that every
//             residue this path computes is itself a NORMAL double — the
//             product residue is a multiple of 2^(ea+ec-104), the 2Sum
//             residues of 2^(e-52) — and so that the product cannot
//             overflow. Anything else is the model's.
//   window    the addend's lowest set bit and the product's lowest set bit
//             must both survive the alignment shift. Counting each one's
//             TRAILING ZEROS is what lets `a*1.0 + b` and single-valued
//             operands keep the wide window they deserve instead of the
//             106-bit worst case.
//   result    normal, and at least one binade above the tininess boundary
//             (the model detects tininess BEFORE rounding, so a result that
//             only reached 2^-1022 by rounding up is underflow to it) — or
//             an exact zero, which every zero-operand shape delivers.
//
// The flags come from the residue exactly as in the single path: FI is
// "residue != 0", FR is "the residue disagrees in sign with the result",
// i.e. the rounding went away from zero. Getting the residue EXACTLY is the
// whole difficulty, and it is three different jobs:
//   mul    err = fma(a,c,-r) — the classic 2Product residue, exact.
//   add    the 2Sum residue (an add arrives here as a product by one, and
//          the model's own accumulator makes it exactly that).
//   fused  the error of an FMA needs two doubles in general. Their SUM is
//          computed here as one double e1 = fl(g + a2) via the standard
//          decomposition (Boldo & Muller, "Exact and Approximated Error of
//          the FMA", 2011): the second double is bounded by half an ulp of
//          the first, so it can move neither e1's sign nor its zeroness —
//          and sign and zeroness are all FI and FR ever ask.
inline constexpr u32 kDblELo = 564;  // 2^-459 : ea+ec >= -918, residues normal
inline constexpr u32 kDblEHi = 1532; // 2^+509 : ea+ec <= 1022, no overflow

// Admitted operand: a normal double inside the band, or an exact zero.
inline bool admitD(u64 x)
{
    const u32 e = u32(x >> 52) & 0x7FFu;
    return (e - kDblELo <= kDblEHi - kDblELo) || (x << 1) == 0;
}

// Lowest bit position the model's 128-bit accumulator can hold a nonzero
// product bit in. mul64to128 puts the product of the two 53-bit significands
// at bit 22 and up, and each factor's trailing zeros raise that floor one for
// one; the normalization shift can only raise it further, so this is a lower
// bound on where the product's information actually lives.
inline u32 lowBitProd(u64 fa, u64 fc)
{
    const u64 ma = (fa & 0x000FFFFFFFFFFFFFull) | (1ull << 52);
    const u64 mc = (fc & 0x000FFFFFFFFFFFFFull) | (1ull << 52);
    return 22u + u32(std::countr_zero(ma)) + u32(std::countr_zero(mc));
}

// Lowest bit position the model's accumulator holds a nonzero bit of a
// SINGLE operand in: its 53-bit significand starts at bit 75 and its
// trailing zeros raise that floor one for one.
inline u32 lowBitOne(u64 f)
{
    return 75u + u32(std::countr_zero((f & 0x000FFFFFFFFFFFFFull) | (1ull << 52)));
}

// The delivered result, admitted and flagged. Shared by all three shapes
// because it is the same question for each: did the model own this result,
// and what did the rounding do to it.
inline bool finishD(double r0, double resid, bool negResult, R& out)
{
    const u64 rb = std::bit_cast<u64>(r0);
    const u32 re = u32(rb >> 52) & 0x7FFu;
    if (re < 2u || re == 0x7FFu) {
        // Everything the model owns: overflow to infinity, a denormal or
        // underflowed result, and the binade where tininess-before-rounding
        // can disagree with the delivered exponent. An EXACT zero is ours —
        // it is what every zero-operand shape produces and its sign already
        // matches the model's rules.
        if (!(re == 0u && (rb << 1) == 0 && resid == 0.0))
            return false;
    }
    u32 fl = 0;
    if (resid != 0.0) {
        fl |= kXx;
        if ((resid < 0.0) != ((rb >> 63) != 0))
            fl |= kFr; // rounded away from zero
    }
    out = {negResult ? rb ^ kSignBit : rb, fl};
    return true;
}

// ⚠ ONE FUNCTION PER SHAPE, and that is a speed decision with a measurement
// behind it. The first cut of this was a single entry taking `hasAddend` and
// testing `fc == 1.0` at run time — and fmul measured 82 host cycles a call
// against fmuls' 58, for two floating-point instructions of actual work. The
// callers below know their shape at compile time; handing that knowledge over
// is what turns the admission into straight-line code.

// a * c.
bool dblMulFast(u64 fa, u64 fc, R& out)
{
#if OPM_SF_X64
    if (!kHaveFma)
        return false; // the residue needs a real fma; the model keeps the row
#endif
    if (!admitD(fa) || !admitD(fc))
        return false;
    const double da = std::bit_cast<double>(fa);
    const double dc = std::bit_cast<double>(fc);
    const double r0 = da * dc;
    return finishD(r0, fmaExact(da, dc, -r0), false, out);
}

// a + b, which the model computes as a*1.0 + b: the product is exactly `a`,
// so the whole operation is a 2Sum and its residue is the answer.
bool dblAddFast(u64 fa, u64 fb, R& out)
{
    if (!admitD(fa) || !admitD(fb))
        return false;
    if ((fa << 1) != 0 && (fb << 1) != 0) {
        const i32 d = i32(u32(fa >> 52) & 0x7FFu) - i32(u32(fb >> 52) & 0x7FFu);
        if (d > i32(lowBitOne(fb)) || d < -i32(lowBitOne(fa)))
            return false; // a bit would fall off the accumulator and be jammed
    }
    const double da = std::bit_cast<double>(fa);
    const double db = std::bit_cast<double>(fb);
    const double r0 = da + db;
    const double bv = r0 - da;
    return finishD(r0, (da - (r0 - bv)) + (db - bv), false, out);
}

// ±((a * c) + b).
bool dblFmaFast(u64 fa, u64 fc, u64 fb, bool negResult, R& out)
{
#if OPM_SF_X64
    if (!kHaveFma)
        return false;
#endif
    if (!admitD(fa) || !admitD(fc) || !admitD(fb))
        return false;
    if ((fa << 1) != 0 && (fc << 1) != 0 && (fb << 1) != 0) {
        // d = the model's product exponent minus the addend's. The product
        // exponent is ea+ec or ea+ec+1 (the normalization shift), so both
        // have to pass.
        const i32 d = i32(u32(fa >> 52) & 0x7FFu) + i32(u32(fc >> 52) & 0x7FFu) -
                      i32(u32(fb >> 52) & 0x7FFu) - 1023;
        if (d + 1 > i32(lowBitOne(fb)) || d < -i32(lowBitProd(fa, fc)))
            return false;
    }
    const double da = std::bit_cast<double>(fa);
    const double dc = std::bit_cast<double>(fc);
    const double db = std::bit_cast<double>(fb);
    const double u1 = da * dc;              // the exact product as u1 + u2
    const double u2 = fmaExact(da, dc, -u1);
    if (u2 == 0.0) {
        // An exact product turns the fused operation into a plain add — and
        // this is the common case in a title whose operands came out of lfs,
        // because a 24x24-bit product fits a double with room to spare.
        const double r0 = u1 + db;
        const double bv = r0 - u1;
        return finishD(r0, (u1 - (r0 - bv)) + (db - bv), negResult, out);
    }
    const double r0 = fmaExact(da, dc, db);
    // a*c + b == u1 + (a1 + a2) == (b1 + b2) + a2, so the residue is
    // (b1 - r0) + b2 + a2 — and every step of that is exact.
    const double a1 = db + u2, av = a1 - db;
    const double a2 = (db - (a1 - av)) + (u2 - av);
    const double b1 = u1 + a1, bv = b1 - u1;
    const double b2 = (u1 - (b1 - bv)) + (a1 - bv);
    return finishD(r0, ((b1 - r0) + b2) + a2, negResult, out);
}

// The gate every double fast path shares: round-to-nearest, no enabled
// exception scaling, no flush-to-zero, and the control not thrown.
inline bool plainD(const Env& e, Tgt t)
{
    return t == Tgt::Dbl && e.rn == 0 && !e.oe && !e.ue && !e.ni &&
           !(gHostFastOff & kFastOffDbl);
}

R fmaCore(u64 fa, u64 fc, u64 fb, bool hasAddend, bool negAdd, bool negResult,
          const Env& env, Tgt t)
{
    if (hasAddend && negAdd)
        fb ^= kSignBit;

    if (t == Tgt::Sgl && env.rn == 0 && !env.oe && !env.ue && !env.ni &&
        !(gHostFastOff & kFastOffSgl)) {
        R fast;
        if (hostSglFast(fa, fc, fb, hasAddend, negResult, fast)) {
            ++gFastHits;
            return fast;
        }
        ++gFastMiss;
    }

    const Un a = unpack(fa), c = unpack(fc), b = unpack(fb);
    u32 fl = 0;

    // Product-invalid (inf * 0) is flagged even when a NaN is also present.
    const bool imz = (a.c == cInf && c.c == cZero) ||
                     (a.c == cZero && c.c == cInf);
    if (imz)
        fl |= kVximz;

    if (a.c == cSNaN || (hasAddend && b.c == cSNaN) || c.c == cSNaN)
        fl |= kVxsnan;
    if (a.c == cQNaN || a.c == cSNaN)
        return {quiet(fa), fl};
    if (hasAddend && (b.c == cQNaN || b.c == cSNaN))
        return {quiet(fb ^ (negAdd ? kSignBit : 0)), fl}; // undo the sign flip
    if (c.c == cQNaN || c.c == cSNaN)
        return {quiet(fc), fl};
    if (imz)
        return {kQNaNGen, fl};

    // Product specials.
    const bool ps = a.s != c.s;
    if (a.c == cInf || c.c == cInf) {
        if (hasAddend && b.c == cInf && b.s != ps) {
            fl |= kVxisi;
            return {kQNaNGen, fl};
        }
        u64 out = packInf(ps);
        if (negResult)
            out ^= kSignBit;
        return {out, fl};
    }
    if (hasAddend && b.c == cInf) {
        u64 out = packInf(b.s);
        if (negResult)
            out ^= kSignBit;
        return {out, fl};
    }

    const bool prodZero = a.c == cZero || c.c == cZero;
    if (prodZero && (!hasAddend || b.c == cZero)) {
        // Exact zero: like-signed zeros keep the sign; unlike-signed sums
        // are +0 except in round-toward--inf (IEEE / PEM 3.3.2).
        bool sign;
        if (!hasAddend)
            sign = ps;
        else if (ps == b.s)
            sign = ps;
        else
            sign = env.rn == 3;
        if (negResult)
            sign = !sign;
        return {packZero(sign), fl};
    }

    U128 acc;   // value = acc * 2^(exp-127), leading 1 in bit 127
    i32 exp;
    bool sign;
    bool jam = false;

    if (prodZero) { // pure addend
        acc = {b.m, 0};
        exp = b.e;
        sign = b.s;
    } else {
        u64 hi, lo;
        mul64to128(a.m, c.m, hi, lo);
        acc = {hi, lo};
        exp = a.e + c.e + 1;
        if (!(acc.hi >> 63)) {
            acc = shl128(acc, 1);
            exp -= 1;
        }
        sign = ps;
        if (hasAddend && b.c != cZero) {
            U128 bacc{b.m, 0};
            i32 bexp = b.e;
            const i32 d = exp - bexp;
            if (d > 0) {
                bacc = shr128jam(bacc, static_cast<u32>(d), jam);
            } else if (d < 0) {
                acc = shr128jam(acc, static_cast<u32>(-d), jam);
                exp = bexp;
            }
            if (sign == b.s) {
                bool carry = false;
                acc = add128(acc, bacc, carry);
                if (carry) {
                    acc = shr128jam(acc, 1, jam);
                    acc.hi |= 1ull << 63;
                    exp += 1;
                }
            } else {
                // Effective subtraction. Jam can only be set when |d| > 1,
                // and then no left normalization beyond one position is
                // needed, so shifting zeros in below is exact.
                if (ge128(bacc, acc)) {
                    U128 diff = sub128(bacc, acc);
                    acc = diff;
                    sign = b.s;
                } else {
                    acc = sub128(acc, bacc);
                }
                if (acc.hi == 0 && acc.lo == 0 && !jam) {
                    bool zsign = env.rn == 3;
                    if (negResult)
                        zsign = !zsign;
                    return {packZero(zsign), fl};
                }
                const int lz = clz128(acc);
                if (lz) {
                    acc = shl128(acc, static_cast<u32>(lz));
                    exp -= lz;
                }
            }
        }
    }

    if (negResult)
        sign = !sign;
    const bool sticky = jam || acc.lo != 0;
    const u64 bits = roundPack(sign, exp, acc.hi, sticky, env, t, fl);
    return {bits, fl};
}

} // namespace

R add(u64 a, u64 b, const Env& e, Tgt t)
{
    if (plainD(e, t)) {
        R f;
        if (dblAddFast(a, b, f)) { ++gFastHitsD; return f; }
        ++gFastMissD;
    }
    return fmaCore(a, kOne, b, true, false, false, e, t);
}
R sub(u64 a, u64 b, const Env& e, Tgt t)
{
    if (plainD(e, t)) {
        R f;
        if (dblAddFast(a, b ^ kSignBit, f)) { ++gFastHitsD; return f; }
        ++gFastMissD;
    }
    return fmaCore(a, kOne, b, true, true, false, e, t);
}
R mul(u64 a, u64 c, const Env& e, Tgt t)
{
    if (plainD(e, t)) {
        R f;
        if (dblMulFast(a, c, f)) { ++gFastHitsD; return f; }
        ++gFastMissD;
    }
    return fmaCore(a, c, 0, false, false, false, e, t);
}
R madd(u64 a, u64 c, u64 b, const Env& e, Tgt t, bool negAdd, bool negResult)
{
    if (plainD(e, t)) {
        R f;
        if (dblFmaFast(a, c, negAdd ? b ^ kSignBit : b, negResult, f)) {
            ++gFastHitsD;
            return f;
        }
        ++gFastMissD;
    }
    return fmaCore(a, c, b, true, negAdd, negResult, e, t);
}

R div(u64 fa, u64 fb, const Env& env, Tgt t)
{
    const Un a = unpack(fa), b = unpack(fb);
    u32 fl = 0;
    if (a.c == cSNaN || b.c == cSNaN)
        fl |= kVxsnan;
    if (a.c == cQNaN || a.c == cSNaN)
        return {quiet(fa), fl};
    if (b.c == cQNaN || b.c == cSNaN)
        return {quiet(fb), fl};

    const bool s = a.s != b.s;
    if (a.c == cInf) {
        if (b.c == cInf) {
            fl |= kVxidi;
            return {kQNaNGen, fl};
        }
        return {packInf(s), fl};
    }
    if (b.c == cInf)
        return {packZero(s), fl};
    if (b.c == cZero) {
        if (a.c == cZero) {
            fl |= kVxzdz;
            return {kQNaNGen, fl};
        }
        fl |= kZx;
        return {packInf(s), fl};
    }
    if (a.c == cZero)
        return {packZero(s), fl};

    // 56 quotient bits by shift-subtract; remainder feeds sticky. After each
    // step rem < b.m, so 2*rem < 2^65 — the dropped top bit is recovered via
    // `top` (2*rem >= 2^64 > b.m always implies the quotient bit is 1, and
    // the mod-2^64 subtraction is still exact since 2*rem - b.m < b.m).
    u64 q = 0, rem = a.m;
    if (rem >= b.m) {
        rem -= b.m;
        q = 1;
    }
    for (int i = 0; i < 55; ++i) {
        const bool top = (rem >> 63) != 0;
        rem <<= 1;
        q <<= 1;
        if (top || rem >= b.m) {
            rem -= b.m;
            q |= 1;
        }
    }
    const bool sticky = rem != 0;
    const int lz = std::countl_zero(q); // 8 or 9
    const u64 nsig = q << lz;
    const i32 exp = (a.e - b.e) + (8 - lz);
    u32 rfl = fl;
    const u64 bits = roundPack(s, exp, nsig, sticky, env, t, rfl);
    return {bits, rfl};
}

R rsp(u64 fb, const Env& env)
{
    // frsp is 2.0M calls and 123 cycles each in-game (s41 --jit-tsc), and it
    // is the cheapest of the three fast paths to justify: the host's own
    // double->float narrowing IS the rounding the model performs, and the
    // residue is exact by Sterbenz — the two agree to within 2^-24, so their
    // difference is representable. Admitted only for a normal input landing
    // on a normal single at least one binade above the tininess boundary;
    // the model keeps NaNs, infinities, zeros, and everything that under- or
    // overflows the single format.
    if (env.rn == 0 && !env.oe && !env.ue && !env.ni &&
        !(gHostFastOff & kFastOffDbl)) {
        const u32 e = u32(fb >> 52) & 0x7FFu;
        if (e - 1u < 0x7FEu) {
            const double d = std::bit_cast<double>(fb);
            const float s = static_cast<float>(d);
            const u32 sb = std::bit_cast<u32>(s);
            const u32 se = (sb >> 23) & 0xFFu;
            if (se >= 2u && se != 0xFFu) {
                const double sd = static_cast<double>(s);
                const double resid = d - sd; // exact: Sterbenz
                u32 fl = 0;
                if (resid != 0.0) {
                    fl |= kXx;
                    if ((resid < 0.0) != ((sb >> 31) != 0))
                        fl |= kFr;
                }
                ++gFastHitsD;
                return {std::bit_cast<u64>(sd), fl};
            }
        }
        ++gFastMissD;
    }
    const Un b = unpack(fb);
    u32 fl = 0;
    if (b.c == cSNaN)
        fl |= kVxsnan;
    if (b.c == cQNaN || b.c == cSNaN) // frsp truncates the payload (PEM 3.3.4)
        return {quiet(fb) & 0xFFFFFFFFE0000000ull, fl};
    if (b.c == cInf || b.c == cZero)
        return {fb, fl};
    const u64 bits = roundPack(b.s, b.e, b.m, false, env, Tgt::Sgl, fl);
    return {bits, fl};
}

R ctiw(u64 fb, u32 rn)
{
    const Un b = unpack(fb);
    u32 fl = 0;
    const u64 hi = 0xFFF8000000000000ull; // RECEIPT: undefined upper word

    if (b.c == cQNaN || b.c == cSNaN) {
        fl |= kVxcvi;
        if (b.c == cSNaN)
            fl |= kVxsnan;
        return {hi | 0x80000000ull, fl};
    }
    if (b.c == cInf || b.e > 62) { // certainly out of range
        fl |= kVxcvi;
        return {hi | (b.s ? 0x80000000ull : 0x7FFFFFFFull), fl};
    }
    if (b.c == cZero)
        return {hi, fl};

    u64 mag;
    bool inx = false, inc = false;
    if (b.e < 0) {
        mag = 0;
        // fraction only: g = bit at 2^-1, rest sticky
        const u64 g = b.e == -1 ? (b.m >> 63) : 0;
        const bool x = b.e == -1 ? (b.m << 1) != 0 : b.m != 0;
        inx = g || x;
        if (inx) {
            switch (rn) {
            case 0: inc = g && x; break; // halfway (g=1,x=0) rounds to even 0
            case 1: inc = false; break;
            case 2: inc = !b.s; break;
            case 3: inc = b.s; break;
            }
        }
        mag += inc ? 1 : 0;
    } else {
        const u32 sh = static_cast<u32>(63 - b.e);
        mag = sh >= 64 ? 0 : b.m >> sh;
        const u64 low = sh == 0 ? 0 : b.m << (64 - sh);
        const u64 g = low >> 63;
        const bool x = (low << 1) != 0;
        inx = g || x;
        if (inx) {
            switch (rn) {
            case 0: inc = g && (x || (mag & 1)); break;
            case 1: inc = false; break;
            case 2: inc = !b.s; break;
            case 3: inc = b.s; break;
            }
        }
        mag += inc ? 1 : 0;
    }

    const u64 limit = b.s ? 0x80000000ull : 0x7FFFFFFFull;
    if (mag > limit) {
        fl = (fl & kVxsnan) | kVxcvi; // FR/FI cleared on invalid convert
        return {hi | (b.s ? 0x80000000ull : 0x7FFFFFFFull), fl};
    }
    if (inx)
        fl |= kXx;
    if (inc)
        fl |= kFr;
    const u32 res32 = b.s ? static_cast<u32>(0u - static_cast<u32>(mag))
                          : static_cast<u32>(mag);
    return {hi | res32, fl};
}

R res(u64 fb, const Env& env)
{
    const Un b = unpack(fb);
    u32 fl = 0;
    if (b.c == cSNaN)
        fl |= kVxsnan;
    if (b.c == cQNaN || b.c == cSNaN)
        return {quiet(fb) & 0xFFFFFFFFE0000000ull, fl};
    if (b.c == cInf)
        return {packZero(b.s), fl};
    if (b.c == cZero) {
        fl |= kZx;
        return {packInf(b.s), fl};
    }
    Env e2 = env; // estimates round to nearest regardless of RN (RECEIPT)
    e2.rn = 0;
    R r = div(kOne, fb, e2, Tgt::Sgl);
    r.fl = (r.fl & (kOx | kUx)) | fl; // FR/FI cleared, XX unaffected (PEM)
    return r;
}

namespace {

// q = floor(sqrt(sigma' * 2^110)) where sigma' = m/2^63 (doubled when the
// exponent was odd) — i.e. sqrt(value) = (q/2^55) * 2^((e-odd)/2). Classic
// two-bits-per-step digit recurrence: radicand = m << (63+odd) aligned so 56
// steps consume exactly its 112 value bits; remainder nonzero -> sticky.
u64 sqrtSig(u64 m, bool oddExp, bool& sticky)
{
    U128 rad = shl128({0, m}, 63u + (oddExp ? 1u : 0u));
    U128 rem{0, 0};
    u64 q = 0;
    for (int i = 0; i < 56; ++i) {
        const u64 top2 = rad.hi >> 62;
        rad = shl128(rad, 2);
        rem = shl128(rem, 2);
        rem.lo |= top2;
        const u64 trial = (q << 2) | 1; // appending a 1 bit costs 4q+1
        if (ge128(rem, {0, trial})) {
            rem = sub128(rem, {0, trial});
            q = (q << 1) | 1;
        } else {
            q <<= 1;
        }
    }
    sticky = rem.hi != 0 || rem.lo != 0;
    return q;
}

} // namespace

R rsqrte(u64 fb, const Env& env)
{
    const Un b = unpack(fb);
    u32 fl = 0;
    if (b.c == cSNaN)
        fl |= kVxsnan;
    if (b.c == cQNaN || b.c == cSNaN)
        return {quiet(fb), fl};
    if (b.c == cZero) {
        fl |= kZx;
        return {packInf(b.s), fl};
    }
    if (b.s) { // negative, nonzero (includes -inf)
        fl |= kVxsqrt;
        return {kQNaNGen, fl};
    }
    if (b.c == cInf)
        return {0, fl}; // +0

    // sqrt to double precision (RN), then reciprocal to double (RN) —
    // deterministic composition, well inside the 1/32 architectural bound.
    const bool odd = (b.e & 1) != 0;
    const i32 half = (b.e - (odd ? 1 : 0)) / 2;
    bool sticky = false;
    const u64 q = sqrtSig(b.m, odd, sticky); // sqrt(sigma') in [1,2): 56 bits
    const int lz = std::countl_zero(q);      // always 8
    Env e2 = env;
    e2.rn = 0;
    u32 sfl = 0;
    const u64 root = roundPack(false, half + (8 - lz), q << lz, sticky, e2,
                               Tgt::Dbl, sfl);
    R r = div(kOne, root, e2, Tgt::Dbl);
    r.fl = fl; // FR/FI undefined (cleared); XX unaffected (PEM)
    return r;
}

u64 loadSingle(u32 w)
{
    const u64 s = static_cast<u64>(w >> 31) << 63;
    const u32 e = (w >> 23) & 0xFFu;
    const u32 frac = w & 0x7FFFFFu;
    if (e == 255)
        return s | 0x7FF0000000000000ull | (static_cast<u64>(frac) << 29);
    if (e == 0) {
        if (frac == 0)
            return s;
        const int m = 31 - std::countl_zero(frac); // position of leading 1
        const i32 vexp = m - 149;
        const u64 mant =
            (static_cast<u64>(frac) << (52 - m)) & 0x000FFFFFFFFFFFFFull;
        return s | (static_cast<u64>(vexp + 1023) << 52) | mant;
    }
    return s | (static_cast<u64>(e + 896) << 52) |
           (static_cast<u64>(frac) << 29);
}

u32 storeSingle(u64 f)
{
    const u32 sign = static_cast<u32>(f >> 63) << 31;
    const u32 e = static_cast<u32>(f >> 52) & 0x7FFu;
    if (e > 896 || (f & 0x7FFFFFFFFFFFFFFFull) == 0)
        return (static_cast<u32>(f >> 62) << 30) |
               (static_cast<u32>(f >> 29) & 0x3FFFFFFFu);
    if (e >= 874) {
        const u64 frac = (1ull << 52) | (f & 0x000FFFFFFFFFFFFFull);
        const u32 sh = 896 + 1 - e; // denormalize until exp = -126
        return sign | (static_cast<u32>(frac >> (sh + 29)) & 0x7FFFFFu);
    }
    return sign; // RECEIPT: model's loop fixed point (WORD undefined per PEM)
}

} // namespace opm::sf
