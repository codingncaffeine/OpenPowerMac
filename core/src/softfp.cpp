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

namespace opm::sf {

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
R fmaCore(u64 fa, u64 fc, u64 fb, bool hasAddend, bool negAdd, bool negResult,
          const Env& env, Tgt t)
{
    if (hasAddend && negAdd)
        fb ^= kSignBit;

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
    return fmaCore(a, kOne, b, true, false, false, e, t);
}
R sub(u64 a, u64 b, const Env& e, Tgt t)
{
    return fmaCore(a, kOne, b, true, true, false, e, t);
}
R mul(u64 a, u64 c, const Env& e, Tgt t)
{
    return fmaCore(a, c, 0, false, false, false, e, t);
}
R madd(u64 a, u64 c, u64 b, const Env& e, Tgt t, bool negAdd, bool negResult)
{
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
