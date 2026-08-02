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
                gHostFastOff = pass == 1;
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
            gHostFastOff = false;
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

TEST_CASE("softfp: fast path bails outside round-to-nearest and plain env")
{
    // Non-RN roundings and enabled exceptions never take the fast path;
    // both switch states must therefore agree trivially. A cheap fence
    // against a guard regressing.
    Rng r{0xFEEDFACEull};
    for (u32 it = 0; it < 20000; ++it) {
        const Env env{(r.next() % 3u) + 1u, (r.next() & 1u) != 0,
                      (r.next() & 1u) != 0, false};
        const u64 a = pickOperand(r);
        const u64 c = pickOperand(r);
        const u64 b = pickOperand(r);
        gHostFastOff = false;
        const R f = madd(a, c, b, env, Tgt::Sgl, false, false);
        gHostFastOff = true;
        const R m = madd(a, c, b, env, Tgt::Sgl, false, false);
        gHostFastOff = false;
        CHECK(f.bits == m.bits);
        CHECK(f.fl == m.fl);
    }
}
