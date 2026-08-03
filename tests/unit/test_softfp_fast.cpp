// The single-target host fast path (softfp.cpp hostSglFast) against the
// PEM-model path, on the same inputs, demanding identical {bits, flags}.
// The fast path's whole claim is "bails unless provably identical", and a
// claim like that is settled by adversarial volume: random operands, the
// tie-boundary mantissas where double rounding would bite, cancellation
// pairs, signed zeros, denormals, infinities, NaNs, and raw doubles that
// are not single-valued at all (the bail's own territory).

#include "doctest.h"
#include "../../core/src/softfp.hpp"

#include <cstdio>

using namespace opm;
using namespace opm::sf;

namespace {

struct Rng {
    u64 s = 0xC0DEC0DEC0DEC0DEull;
    u32 next()
    {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        return u32(s >> 32);
    }
};

u32 pickSingleBits(Rng& r)
{
    switch (r.next() % 8u) {
    case 0: return r.next(); // anything at all
    case 1: // normal, full-range exponent
        return (r.next() & 0x807FFFFFu) | (((r.next() % 254u) + 1u) << 23);
    case 2: return r.next() & 0x80000000u;                // ±0
    case 3: return (r.next() & 0x80000000u) | 0x7F800000u; // ±inf
    case 4: return (r.next() & 0x80000000u) | 0x7F800000u | (r.next() & 0x7FFFFFu); // NaN space
    case 5: return r.next() & 0x807FFFFFu;                // denormal / zero
    case 6: // mid-band exponents: products stay normal, sums cancel often
        return (r.next() & 0x80000000u) |
               (((r.next() % 24u) + 115u) << 23) | (r.next() & 0x7FFFFFu);
    default: { // tie-boundary mantissas
        static const u32 m[] = {0x7FFFFFu, 0x400000u, 0x3FFFFFu, 0x000001u,
                                0x555555u, 0x7FFFFEu};
        return (r.next() & 0x80000000u) |
               (((r.next() % 254u) + 1u) << 23) | m[r.next() % 6u];
    }
    }
}

u64 pickOperand(Rng& r)
{
    // Three in four are genuine single-valued doubles (the machine's own
    // steady state after lfs/single arithmetic); the rest are raw doubles,
    // which the fast path must hand to the model untouched.
    if (r.next() % 4u)
        return loadSingle(pickSingleBits(r));
    return (u64(r.next()) << 32) | r.next();
}

} // namespace

TEST_CASE("softfp: single-target host fast path == model on {bits, flags}")
{
    const Env env{0, false, false, false};
    Rng r;
    u64 bad = 0;
    for (u32 it = 0; it < 300000 && bad < 8; ++it) {
        const u64 a = pickOperand(r);
        const u64 c = pickOperand(r);
        const u64 b = pickOperand(r);
        for (u32 op = 0; op < 7u; ++op) {
            R fast{}, model{};
            for (int pass = 0; pass < 2; ++pass) {
                gHostFastOff = pass ? (kFastOffSgl | kFastOffDbl) : 0u;
                R& out = pass ? model : fast;
                switch (op) {
                case 0: out = add(a, b, env, Tgt::Sgl); break;
                case 1: out = sub(a, b, env, Tgt::Sgl); break;
                case 2: out = mul(a, c, env, Tgt::Sgl); break;
                case 3: out = madd(a, c, b, env, Tgt::Sgl, false, false); break;
                case 4: out = madd(a, c, b, env, Tgt::Sgl, true, false); break;
                case 5: out = madd(a, c, b, env, Tgt::Sgl, false, true); break;
                default: out = madd(a, c, b, env, Tgt::Sgl, true, true); break;
                }
            }
            gHostFastOff = 0;
            if (fast.bits != model.bits || fast.fl != model.fl) {
                ++bad;
                char why[192];
                snprintf(why, sizeof why,
                         "op=%u a=%016llx c=%016llx b=%016llx "
                         "fast=%016llx/%x model=%016llx/%x",
                         op, (unsigned long long)a, (unsigned long long)c,
                         (unsigned long long)b,
                         (unsigned long long)fast.bits, fast.fl,
                         (unsigned long long)model.bits, model.fl);
                MESSAGE(why);
            }
        }
    }
    CHECK(bad == 0);
}

namespace {

// Double-target operands. The single generator above is useless here: three
// in four of its values are single-valued, and the double path's whole
// difficulty lives in the 53-bit corners a single can't reach — a product
// that needs all 106 bits, an addend that lands just inside or just outside
// the model's 128-bit alignment window, a cancellation that leaves the
// residue to carry the sign.
u64 pickDouble(Rng& r, i32 expLo, i32 expHi)
{
    const u64 sign = u64(r.next() & 1u) << 63;
    switch (r.next() % 10u) {
    case 0: return (u64(r.next()) << 32) | r.next(); // anything at all
    case 1: return sign;                             // ±0
    case 2: return sign | 0x7FF0000000000000ull;     // ±inf
    case 3: return sign | 0x7FF0000000000000ull |
                   (u64(r.next() & 0x7FFFFu) << 32) | r.next(); // NaN space
    case 4: return sign | (u64(r.next() & 0xFFFFFu) << 32) | r.next(); // denormal
    case 5: { // single-valued: the machine's own steady state after lfs
        const u32 w = (r.next() & 0x807FFFFFu) | (((r.next() % 200u) + 30u) << 23);
        return loadSingle(w);
    }
    case 6: { // few significant bits: exact products, wide alignment windows
        const u32 e = u32(expLo + i32(r.next() % u32(expHi - expLo + 1)));
        return sign | (u64(e) << 52) |
               (u64(r.next() % 64u) << (46 + (r.next() % 7u)));
    }
    case 7: { // tie-boundary mantissas
        static const u64 m[] = {0xFFFFFFFFFFFFFull, 0x8000000000000ull,
                                0x7FFFFFFFFFFFFull, 0x0000000000001ull,
                                0x5555555555555ull, 0xFFFFFFFFFFFFEull};
        const u32 e = u32(expLo + i32(r.next() % u32(expHi - expLo + 1)));
        return sign | (u64(e) << 52) | m[r.next() % 6u];
    }
    default: { // full-mantissa normal in the requested exponent band
        const u32 e = u32(expLo + i32(r.next() % u32(expHi - expLo + 1)));
        return sign | (u64(e) << 52) |
               (((u64(r.next()) << 32) | r.next()) & 0x000FFFFFFFFFFFFFull);
    }
    }
}

// ⭐ THE POSITIVE CONTROL. A fast path that bailed on every input would pass
// every differential in this file trivially — the two arms would both be the
// model. So each case below asserts a floor on how often the path was
// actually TAKEN, read from the counters the machine itself reports.
struct Taken {
    u64 h0, m0;
    Taken() : h0(gFastHitsD), m0(gFastMissD) {}
    double rate() const
    {
        const u64 h = gFastHitsD - h0, m = gFastMissD - m0;
        return h + m ? 100.0 * double(h) / double(h + m) : 0.0;
    }
};

// Every arithmetic shape the fast path can be reached through, both ways,
// on one operand triple. Counts (and prints the first few) disagreements.
void diffTriple(u64 a, u64 c, u64 b, Tgt t, u32& bad)
{
    const Env env{0, false, false, false};
    for (u32 op = 0; op < 8u; ++op) {
        R fast{}, model{};
        for (int pass = 0; pass < 2; ++pass) {
            gHostFastOff = pass ? (kFastOffSgl | kFastOffDbl) : 0u;
            R& out = pass ? model : fast;
            switch (op) {
            case 0: out = add(a, b, env, t); break;
            case 1: out = sub(a, b, env, t); break;
            case 2: out = mul(a, c, env, t); break;
            case 3: out = madd(a, c, b, env, t, false, false); break;
            case 4: out = madd(a, c, b, env, t, true, false); break;
            case 5: out = madd(a, c, b, env, t, false, true); break;
            case 6: out = madd(a, c, b, env, t, true, true); break;
            default: out = rsp(b, env); break;
            }
        }
        gHostFastOff = 0;
        if (fast.bits != model.bits || fast.fl != model.fl) {
            if (++bad <= 8) {
                char why[192];
                snprintf(why, sizeof why,
                         "op=%u a=%016llx c=%016llx b=%016llx "
                         "fast=%016llx/%x model=%016llx/%x",
                         op, (unsigned long long)a, (unsigned long long)c,
                         (unsigned long long)b,
                         (unsigned long long)fast.bits, fast.fl,
                         (unsigned long long)model.bits, model.fl);
                MESSAGE(why);
            }
        }
    }
}

} // namespace

TEST_CASE("softfp: double-target host fast path == model on {bits, flags}")
{
    const Taken taken;
    Rng r{0x9E3779B97F4A7C15ull};
    u32 bad = 0;
    // Three exponent regimes, because the fast path's guards are all about
    // exponents: the mid band where it should almost always take, the band
    // edges where the residue stops being a normal double, and the extremes
    // where products overflow and results go denormal.
    static const i32 lo[] = {1000, 1, 500, 1023, 560, 1500};
    static const i32 hi[] = {1046, 2046, 1546, 1023, 570, 1540};
    for (u32 band = 0; band < 6u; ++band)
        for (u32 it = 0; it < 40000; ++it) {
            const u64 a = pickDouble(r, lo[band], hi[band]);
            const u64 c = pickDouble(r, lo[band], hi[band]);
            const u64 b = pickDouble(r, lo[band], hi[band]);
            diffTriple(a, c, b, Tgt::Dbl, bad);
        }
    CHECK(bad == 0);
    // A quarter of the generated operands are NaN/inf/denormal on purpose,
    // two of the six bands sit outside the exponent admission entirely, and
    // `rsp` counts here too while none of these values are in single range —
    // so this floor is deliberately low. It exists to fail loudly if the path
    // ever stops taking at all. The floor that speaks to the machine's own
    // population is the next case.
    CHECK(taken.rate() > 15.0);
}

TEST_CASE("softfp: double fast path takes on the machine's own population")
{
    // ⭐ The control that matters. Ordinary doubles — the exponents and
    // mantissas a 3D title's transform and lighting arithmetic actually
    // produces — must go down the fast path essentially always. Without this,
    // a guard regression that quietly bailed on everything would still pass
    // every differential above, because both arms would be the model.
    const Taken taken;
    Rng r{0x14057B7EF767814Full};
    u32 bad = 0;
    // Ordinary finite normals only — no NaN, no infinity, no denormal, no
    // wild exponent. This is the population, so this is the rate.
    const auto ordinary = [&r]() {
        return (u64(r.next() & 1u) << 63) |
               (u64(1013u + r.next() % 21u) << 52) |
               (((u64(r.next()) << 32) | r.next()) & 0x000FFFFFFFFFFFFFull);
    };
    for (u32 it = 0; it < 20000; ++it) {
        const u64 a = ordinary();
        const u64 c = ordinary();
        const u64 b = ordinary();
        for (u32 op = 0; op < 7u; ++op) { // every shape but rsp
            R fast{}, model{};
            const Env env{0, false, false, false};
            for (int pass = 0; pass < 2; ++pass) {
                gHostFastOff = pass ? (kFastOffSgl | kFastOffDbl) : 0u;
                R& o = pass ? model : fast;
                switch (op) {
                case 0: o = add(a, b, env, Tgt::Dbl); break;
                case 1: o = sub(a, b, env, Tgt::Dbl); break;
                case 2: o = mul(a, c, env, Tgt::Dbl); break;
                case 3: o = madd(a, c, b, env, Tgt::Dbl, false, false); break;
                case 4: o = madd(a, c, b, env, Tgt::Dbl, true, false); break;
                case 5: o = madd(a, c, b, env, Tgt::Dbl, false, true); break;
                default: o = madd(a, c, b, env, Tgt::Dbl, true, true); break;
                }
            }
            gHostFastOff = 0;
            if (fast.bits != model.bits || fast.fl != model.fl)
                ++bad;
        }
    }
    CHECK(bad == 0);
    CHECK(taken.rate() > 99.0); // measured 99.5%
}

TEST_CASE("softfp: double fast path across the alignment window")
{
    // The admission that is genuinely the model's own geometry: an addend
    // more than 75 bits below the product (or a product more than its own
    // trailing-zero count below the addend) has bits jammed off the bottom of
    // fmaCore's 128-bit accumulator, and there the model is deliberately not
    // ideal arithmetic. Walk the addend's exponent right across both edges.
    const Taken taken;
    Rng r{0x2545F4914F6CDD1Dull};
    u32 bad = 0;
    for (u32 it = 0; it < 4000; ++it) {
        const u64 a = pickDouble(r, 1010, 1036);
        const u64 c = pickDouble(r, 1010, 1036);
        for (i32 shift = -140; shift <= 140; shift += 1) {
            u64 b = pickDouble(r, 1023, 1023);
            const i32 e = i32((b >> 52) & 0x7FFu) + shift;
            if (e < 1 || e > 2046)
                continue;
            b = (b & 0x800FFFFFFFFFFFFFull) | (u64(u32(e)) << 52);
            diffTriple(a, c, b, Tgt::Dbl, bad);
        }
    }
    CHECK(bad == 0);
    CHECK(taken.rate() > 20.0); // both sides of the window get walked
}

TEST_CASE("softfp: double fast path on cancellation and exact sums")
{
    // Residues that carry the whole answer: near-cancelling pairs (where FR
    // and FI are decided by bits far below the result) and sums that are
    // exactly representable (where the fast path must report FI=0 and hand
    // back a bit-identical zero or sum).
    const Taken taken;
    Rng r{0xD1B54A32D192ED03ull};
    u32 bad = 0;
    for (u32 it = 0; it < 60000; ++it) {
        const u64 a = pickDouble(r, 1000, 1046);
        const u64 c = pickDouble(r, 1000, 1046);
        // b chosen to nearly annihilate a*c: same magnitude, opposite sign,
        // perturbed in the low bits.
        const i32 pe = i32((a >> 52) & 0x7FFu) + i32((c >> 52) & 0x7FFu) - 1023;
        u64 b = a;
        if (pe >= 1 && pe <= 2046)
            b = ((a ^ 0x8000000000000000ull) & 0x800FFFFFFFFFFFFFull) |
                (u64(u32(pe)) << 52);
        b ^= u64(r.next() % 8u); // and off the exact cancellation by a few ulps
        diffTriple(a, c, b, Tgt::Dbl, bad);
        diffTriple(a, a, a ^ 0x8000000000000000ull, Tgt::Dbl, bad);
    }
    CHECK(bad == 0);
    CHECK(taken.rate() > 40.0); // rsp bails throughout: nothing here is in single range
}

TEST_CASE("softfp: fast path bails outside round-to-nearest and plain env")
{
    // Non-RN roundings and enabled exceptions never take the fast path;
    // both switch states must therefore agree trivially. A cheap fence
    // against a guard regressing.
    Rng r{0xFEEDFACEull};
    for (u32 it = 0; it < 20000; ++it) {
        // NI (flush-to-zero) is in the sweep too: it changes the delivered
        // result for denormals, so it must reach the model like the rest.
        const Env env{r.next() % 4u, (r.next() & 1u) != 0,
                      (r.next() & 1u) != 0, (r.next() & 1u) != 0};
        if (env.rn == 0 && !env.oe && !env.ue && !env.ni)
            continue; // that is the plain env, where the fast path belongs
        const u64 a = pickOperand(r);
        const u64 c = pickOperand(r);
        const u64 b = pickDouble(r, 1, 2046);
        static const Tgt kTgts[2] = {Tgt::Sgl, Tgt::Dbl};
        for (const Tgt t : kTgts) {
            gHostFastOff = 0;
            const R f = madd(a, c, b, env, t, false, false);
            const R fr = rsp(b, env);
            gHostFastOff = kFastOffSgl | kFastOffDbl;
            const R m = madd(a, c, b, env, t, false, false);
            const R mr = rsp(b, env);
            gHostFastOff = 0;
            CHECK(f.bits == m.bits);
            CHECK(f.fl == m.fl);
            CHECK(fr.bits == mr.bits);
            CHECK(fr.fl == mr.fl);
        }
    }
}
