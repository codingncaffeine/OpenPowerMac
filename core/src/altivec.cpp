// AltiVec executors (P5): all 162 vector instructions. Element numbering is
// big-endian (element 0 = V128.b[0] = the byte at the lowest EA of an aligned
// load), per the AltiVec PEM. Integer semantics per PEM ch.4/6:
//   - saturating forms clamp and set VSCR[SAT] only when clamping changed a
//     value (borderline-equal results do not set SAT);
//   - modulo forms never touch SAT;
//   - compares with Rc set CR6[0] = all-true (8), CR6[2] = all-false (2).
// Vector float lanes reuse the deterministic softfloat (softfp.cpp) at
// single precision: always round-to-nearest, no FPSCR interaction, no
// exceptions — QNaN results replace invalid operations. VSCR[NJ]=1 flushes
// denormalized inputs AND results to signed zero (PEM 3.2.1.2).
//
// RECEIPTS: vsl/vsr use vB byte 15's low 3 bits (architecturally all bytes
// must agree, else boundedly undefined); estimate instructions are
// correctly-rounded compositions, not the silicon tables (ledger rows);
// vlogefp/vexptefp use a fixed cubic minimax on the mantissa, inside the
// architectural 1/32 absolute / 3/4096 relative bounds.

#include "opm/cpu.hpp"
#include "opm/bits.hpp"
#include "softfp.hpp"

#include <bit>
#include <cstring>

namespace opm {

namespace {

// VSCR bits (32-bit register: NJ = bit 15, SAT = bit 31 in PPC numbering).
inline constexpr u32 vNJ = 0x00010000u;
inline constexpr u32 vSAT = 0x00000001u;

inline V128& VD(Cpu& c, u32 i) { return c.st.vr[f_rt(i)]; }
inline const V128& VA(Cpu& c, u32 i) { return c.st.vr[f_ra(i)]; }
inline const V128& VB(Cpu& c, u32 i) { return c.st.vr[f_rb(i)]; }
inline const V128& VC(Cpu& c, u32 i) { return c.st.vr[f_rc_vec(i)]; }

inline u16 gh(const V128& v, int i)
{
    return static_cast<u16>((v.b[2 * i] << 8) | v.b[2 * i + 1]);
}
inline u32 gw(const V128& v, int i)
{
    return (u32(v.b[4 * i]) << 24) | (u32(v.b[4 * i + 1]) << 16) |
           (u32(v.b[4 * i + 2]) << 8) | u32(v.b[4 * i + 3]);
}
inline void ph(V128& v, int i, u16 x)
{
    v.b[2 * i] = static_cast<u8>(x >> 8);
    v.b[2 * i + 1] = static_cast<u8>(x);
}
inline void pw(V128& v, int i, u32 x)
{
    v.b[4 * i] = static_cast<u8>(x >> 24);
    v.b[4 * i + 1] = static_cast<u8>(x >> 16);
    v.b[4 * i + 2] = static_cast<u8>(x >> 8);
    v.b[4 * i + 3] = static_cast<u8>(x);
}

inline void sat(Cpu& c) { c.st.vscr |= vSAT; }

inline u8 satU8(Cpu& c, i64 x) { if (x < 0) { sat(c); return 0; } if (x > 255) { sat(c); return 255; } return (u8)x; }
inline i8 satS8(Cpu& c, i64 x) { if (x < -128) { sat(c); return -128; } if (x > 127) { sat(c); return 127; } return (i8)x; }
inline u16 satU16(Cpu& c, i64 x) { if (x < 0) { sat(c); return 0; } if (x > 65535) { sat(c); return 65535; } return (u16)x; }
inline i16 satS16(Cpu& c, i64 x) { if (x < -32768) { sat(c); return -32768; } if (x > 32767) { sat(c); return 32767; } return (i16)x; }
inline u32 satU32(Cpu& c, i64 x) { if (x < 0) { sat(c); return 0; } if (x > 0xFFFFFFFFll) { sat(c); return 0xFFFFFFFFu; } return (u32)x; }
inline i32 satS32(Cpu& c, i64 x)
{
    if (x < -0x80000000ll) { sat(c); return static_cast<i32>(0x80000000u); }
    if (x > 0x7FFFFFFFll) { sat(c); return 0x7FFFFFFF; }
    return (i32)x;
}

// ---- memory ----------------------------------------------------------------
// lvx/stvx truncate the EA to 16-byte alignment; element loads/stores place
// the addressed element at its natural position within the target register
// and leave the rest undefined — RECEIPT: we zero the rest.

inline u32 eaVX(Cpu& c, u32 i)
{
    return (f_ra(i) ? c.st.gpr[f_ra(i)] : 0) + c.st.gpr[f_rb(i)];
}

void h_lvx(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 ea = eaVX(c, i) & ~15u;
    V128 v;
    for (int k = 0; k < 16; ++k) {
        u32 byte;
        if (!c.readV8(ea + static_cast<u32>(k), byte))
            return;
        v.b[k] = static_cast<u8>(byte);
    }
    VD(c, i) = v;
}

void h_stvx(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 ea = eaVX(c, i) & ~15u;
    const V128 v = c.st.vr[f_rt(i)];
    for (int k = 0; k < 16; ++k)
        if (!c.writeV8(ea + static_cast<u32>(k), v.b[k]))
            return;
}

void h_lvebx(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 ea = eaVX(c, i);
    u32 byte;
    if (!c.readV8(ea, byte))
        return;
    V128 v{};
    v.b[ea & 15u] = static_cast<u8>(byte);
    VD(c, i) = v;
}
void h_lvehx(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 ea = eaVX(c, i) & ~1u;
    u32 half;
    if (!c.readV16(ea, half))
        return;
    V128 v{};
    ph(v, (ea & 15u) >> 1, static_cast<u16>(half));
    VD(c, i) = v;
}
void h_lvewx(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 ea = eaVX(c, i) & ~3u;
    u32 word;
    if (!c.readV32(ea, word))
        return;
    V128 v{};
    pw(v, (ea & 15u) >> 2, word);
    VD(c, i) = v;
}
void h_stvebx(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 ea = eaVX(c, i);
    c.writeV8(ea, c.st.vr[f_rt(i)].b[ea & 15u]);
}
void h_stvehx(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 ea = eaVX(c, i) & ~1u;
    c.writeV16(ea, gh(c.st.vr[f_rt(i)], (ea & 15u) >> 1));
}
void h_stvewx(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 ea = eaVX(c, i) & ~3u;
    c.writeV32(ea, gw(c.st.vr[f_rt(i)], (ea & 15u) >> 2));
}

// lvsl/lvsr produce the permute control for unaligned access sequences.
void h_lvsl(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 sh = eaVX(c, i) & 15u;
    V128 v;
    for (int k = 0; k < 16; ++k)
        v.b[k] = static_cast<u8>(sh + static_cast<u32>(k));
    VD(c, i) = v;
}
void h_lvsr(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 sh = eaVX(c, i) & 15u;
    V128 v;
    for (int k = 0; k < 16; ++k)
        v.b[k] = static_cast<u8>(16 - sh + static_cast<u32>(k));
    VD(c, i) = v;
}

// ---- permute / select / splat / merge --------------------------------------

void h_vperm(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 a = VA(c, i), b = VB(c, i), ctl = VC(c, i);
    V128 v;
    for (int k = 0; k < 16; ++k) {
        const u8 s = ctl.b[k] & 0x1Fu;
        v.b[k] = s < 16 ? a.b[s] : b.b[s - 16];
    }
    VD(c, i) = v;
}

void h_vsel(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 a = VA(c, i), b = VB(c, i), m = VC(c, i);
    V128 v;
    for (int k = 0; k < 16; ++k)
        v.b[k] = static_cast<u8>((a.b[k] & ~m.b[k]) | (b.b[k] & m.b[k]));
    VD(c, i) = v;
}

void h_vspltb(Cpu& c, u32 i, const InsnDesc&)
{
    const u8 x = VB(c, i).b[f_ra(i) & 15u];
    V128 v;
    std::memset(v.b, x, 16);
    VD(c, i) = v;
}
void h_vsplth(Cpu& c, u32 i, const InsnDesc&)
{
    const u16 x = gh(VB(c, i), f_ra(i) & 7u);
    V128 v;
    for (int k = 0; k < 8; ++k)
        ph(v, k, x);
    VD(c, i) = v;
}
void h_vspltw(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 x = gw(VB(c, i), f_ra(i) & 3u);
    V128 v;
    for (int k = 0; k < 4; ++k)
        pw(v, k, x);
    VD(c, i) = v;
}
void h_vspltisb(Cpu& c, u32 i, const InsnDesc&)
{
    V128 v;
    std::memset(v.b, static_cast<u8>(sext5(f_simm5(i))), 16);
    VD(c, i) = v;
}
void h_vspltish(Cpu& c, u32 i, const InsnDesc&)
{
    V128 v;
    for (int k = 0; k < 8; ++k)
        ph(v, k, static_cast<u16>(sext5(f_simm5(i))));
    VD(c, i) = v;
}
void h_vspltisw(Cpu& c, u32 i, const InsnDesc&)
{
    V128 v;
    for (int k = 0; k < 4; ++k)
        pw(v, k, static_cast<u32>(sext5(f_simm5(i))));
    VD(c, i) = v;
}

#define MERGE(NAME, N, GET, PUT, BASE)                                        \
    void NAME(Cpu& c, u32 i, const InsnDesc&)                                 \
    {                                                                         \
        const V128 a = VA(c, i), b = VB(c, i);                                \
        V128 v;                                                               \
        for (int k = 0; k < N; ++k) {                                         \
            PUT(v, 2 * k, GET(a, BASE + k));                                  \
            PUT(v, 2 * k + 1, GET(b, BASE + k));                              \
        }                                                                     \
        VD(c, i) = v;                                                         \
    }
inline u8 gb8(const V128& v, int i) { return v.b[i]; }
inline void pb8(V128& v, int i, u8 x) { v.b[i] = x; }
MERGE(h_vmrghb, 8, gb8, pb8, 0)
MERGE(h_vmrglb, 8, gb8, pb8, 8)
MERGE(h_vmrghh, 4, gh, ph, 0)
MERGE(h_vmrglh, 4, gh, ph, 4)
MERGE(h_vmrghw, 2, gw, pw, 0)
MERGE(h_vmrglw, 2, gw, pw, 2)
#undef MERGE

void h_vsldoi(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 a = VA(c, i), b = VB(c, i);
    const u32 sh = f_vsh(i);
    V128 v;
    for (u32 k = 0; k < 16; ++k) {
        const u32 s = k + sh;
        v.b[k] = s < 16 ? a.b[s] : b.b[s - 16];
    }
    VD(c, i) = v;
}

// ---- whole-register shifts -------------------------------------------------

void h_vslo(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 a = VA(c, i);
    const u32 shb = (VB(c, i).b[15] >> 3) & 0xFu;
    V128 v{};
    for (u32 k = 0; k + shb < 16; ++k)
        v.b[k] = a.b[k + shb];
    VD(c, i) = v;
}
void h_vsro(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 a = VA(c, i);
    const u32 shb = (VB(c, i).b[15] >> 3) & 0xFu;
    V128 v{};
    for (u32 k = shb; k < 16; ++k)
        v.b[k] = a.b[k - shb];
    VD(c, i) = v;
}
void h_vsl(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 a = VA(c, i);
    const u32 sh = VB(c, i).b[15] & 7u; // RECEIPT: byte 15's low 3 bits
    V128 v;
    for (int k = 0; k < 16; ++k) {
        const u8 next = k < 15 ? a.b[k + 1] : 0;
        v.b[k] = static_cast<u8>((a.b[k] << sh) | (sh ? next >> (8 - sh) : 0));
    }
    VD(c, i) = v;
}
void h_vsr(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 a = VA(c, i);
    const u32 sh = VB(c, i).b[15] & 7u;
    V128 v;
    for (int k = 0; k < 16; ++k) {
        const u8 prev = k > 0 ? a.b[k - 1] : 0;
        v.b[k] = static_cast<u8>((a.b[k] >> sh) | (sh ? prev << (8 - sh) : 0));
    }
    VD(c, i) = v;
}

// ---- element rotates / shifts ----------------------------------------------

#define PERLANE_B(NAME, EXPR)                                                 \
    void NAME(Cpu& c, u32 i, const InsnDesc&)                                 \
    {                                                                         \
        const V128 a = VA(c, i), b = VB(c, i);                                \
        V128 v;                                                               \
        for (int k = 0; k < 16; ++k) {                                        \
            const u32 x = a.b[k];                                             \
            const u32 n = b.b[k] & 7u;                                        \
            (void)x; (void)n;                                                 \
            v.b[k] = static_cast<u8>(EXPR);                                   \
        }                                                                     \
        VD(c, i) = v;                                                         \
    }
#define PERLANE_H(NAME, EXPR)                                                 \
    void NAME(Cpu& c, u32 i, const InsnDesc&)                                 \
    {                                                                         \
        const V128 a = VA(c, i), b = VB(c, i);                                \
        V128 v;                                                               \
        for (int k = 0; k < 8; ++k) {                                         \
            const u32 x = gh(a, k);                                           \
            const u32 n = gh(b, k) & 15u;                                     \
            (void)x; (void)n;                                                 \
            ph(v, k, static_cast<u16>(EXPR));                                 \
        }                                                                     \
        VD(c, i) = v;                                                         \
    }
#define PERLANE_W(NAME, EXPR)                                                 \
    void NAME(Cpu& c, u32 i, const InsnDesc&)                                 \
    {                                                                         \
        const V128 a = VA(c, i), b = VB(c, i);                                \
        V128 v;                                                               \
        for (int k = 0; k < 4; ++k) {                                         \
            const u32 x = gw(a, k);                                           \
            const u32 n = gw(b, k) & 31u;                                     \
            (void)x; (void)n;                                                 \
            pw(v, k, static_cast<u32>(EXPR));                                 \
        }                                                                     \
        VD(c, i) = v;                                                         \
    }

PERLANE_B(h_vrlb, (x << n) | (x >> ((8 - n) & 7)))
PERLANE_H(h_vrlh, (x << n) | (x >> ((16 - n) & 15)))
PERLANE_W(h_vrlw, n ? ((x << n) | (x >> (32 - n))) : x)
PERLANE_B(h_vslb, x << n)
PERLANE_H(h_vslh, x << n)
PERLANE_W(h_vslw, x << n)
PERLANE_B(h_vsrb, x >> n)
PERLANE_H(h_vsrh, x >> n)
PERLANE_W(h_vsrw, x >> n)
PERLANE_B(h_vsrab, static_cast<u32>(static_cast<i32>(static_cast<i8>(x)) >> n))
PERLANE_H(h_vsrah, static_cast<u32>(static_cast<i32>(static_cast<i16>(x)) >> n))
PERLANE_W(h_vsraw, static_cast<u32>(static_cast<i32>(x) >> n))
#undef PERLANE_B
#undef PERLANE_H
#undef PERLANE_W

// ---- logical ---------------------------------------------------------------

#define VLOGIC(NAME, EXPR)                                                    \
    void NAME(Cpu& c, u32 i, const InsnDesc&)                                 \
    {                                                                         \
        const V128 a = VA(c, i), b = VB(c, i);                                \
        V128 v;                                                               \
        for (int k = 0; k < 16; ++k)                                          \
            v.b[k] = static_cast<u8>(EXPR);                                   \
        VD(c, i) = v;                                                         \
    }
VLOGIC(h_vand, a.b[k] & b.b[k])
VLOGIC(h_vandc, a.b[k] & ~b.b[k])
VLOGIC(h_vor, a.b[k] | b.b[k])
VLOGIC(h_vnor, ~(a.b[k] | b.b[k]))
VLOGIC(h_vxor, a.b[k] ^ b.b[k])
#undef VLOGIC

// ---- integer add / sub -----------------------------------------------------

#define VARITH_B(NAME, EXPR)                                                  \
    void NAME(Cpu& c, u32 i, const InsnDesc&)                                 \
    {                                                                         \
        const V128 a = VA(c, i), b = VB(c, i);                                \
        V128 v;                                                               \
        for (int k = 0; k < 16; ++k)                                          \
            v.b[k] = static_cast<u8>(EXPR);                                   \
        VD(c, i) = v;                                                         \
    }
#define VARITH_H(NAME, EXPR)                                                  \
    void NAME(Cpu& c, u32 i, const InsnDesc&)                                 \
    {                                                                         \
        const V128 a = VA(c, i), b = VB(c, i);                                \
        V128 v;                                                               \
        for (int k = 0; k < 8; ++k)                                           \
            ph(v, k, static_cast<u16>(EXPR));                                 \
        VD(c, i) = v;                                                         \
    }
#define VARITH_W(NAME, EXPR)                                                  \
    void NAME(Cpu& c, u32 i, const InsnDesc&)                                 \
    {                                                                         \
        const V128 a = VA(c, i), b = VB(c, i);                                \
        V128 v;                                                               \
        for (int k = 0; k < 4; ++k)                                           \
            pw(v, k, static_cast<u32>(EXPR));                                 \
        VD(c, i) = v;                                                         \
    }

VARITH_B(h_vaddubm, a.b[k] + b.b[k])
VARITH_H(h_vadduhm, gh(a, k) + gh(b, k))
VARITH_W(h_vadduwm, gw(a, k) + gw(b, k))
VARITH_B(h_vsububm, a.b[k] - b.b[k])
VARITH_H(h_vsubuhm, gh(a, k) - gh(b, k))
VARITH_W(h_vsubuwm, gw(a, k) - gw(b, k))

VARITH_B(h_vaddubs, satU8(c, a.b[k] + b.b[k]))
VARITH_H(h_vadduhs, satU16(c, static_cast<i64>(gh(a, k)) + gh(b, k)))
VARITH_W(h_vadduws, satU32(c, static_cast<i64>(gw(a, k)) + gw(b, k)))
VARITH_B(h_vaddsbs, satS8(c, static_cast<i8>(a.b[k]) + static_cast<i8>(b.b[k])))
VARITH_H(h_vaddshs, satS16(c, static_cast<i64>(static_cast<i16>(gh(a, k))) + static_cast<i16>(gh(b, k))))
VARITH_W(h_vaddsws, satS32(c, static_cast<i64>(static_cast<i32>(gw(a, k))) + static_cast<i32>(gw(b, k))))
VARITH_B(h_vsububs, satU8(c, a.b[k] - b.b[k]))
VARITH_H(h_vsubuhs, satU16(c, static_cast<i64>(gh(a, k)) - gh(b, k)))
VARITH_W(h_vsubuws, satU32(c, static_cast<i64>(gw(a, k)) - gw(b, k)))
VARITH_B(h_vsubsbs, satS8(c, static_cast<i8>(a.b[k]) - static_cast<i8>(b.b[k])))
VARITH_H(h_vsubshs, satS16(c, static_cast<i64>(static_cast<i16>(gh(a, k))) - static_cast<i16>(gh(b, k))))
VARITH_W(h_vsubsws, satS32(c, static_cast<i64>(static_cast<i32>(gw(a, k))) - static_cast<i32>(gw(b, k))))

VARITH_W(h_vaddcuw, (static_cast<u64>(gw(a, k)) + gw(b, k)) >> 32)
VARITH_W(h_vsubcuw, static_cast<u64>(gw(a, k)) >= gw(b, k) ? 1u : 0u)

// ---- avg / min / max -------------------------------------------------------

VARITH_B(h_vavgub, (a.b[k] + b.b[k] + 1) >> 1)
VARITH_H(h_vavguh, (gh(a, k) + gh(b, k) + 1) >> 1)
VARITH_W(h_vavguw, (static_cast<u64>(gw(a, k)) + gw(b, k) + 1) >> 1)
VARITH_B(h_vavgsb, (static_cast<i8>(a.b[k]) + static_cast<i8>(b.b[k]) + 1) >> 1)
VARITH_H(h_vavgsh, (static_cast<i16>(gh(a, k)) + static_cast<i16>(gh(b, k)) + 1) >> 1)
VARITH_W(h_vavgsw, (static_cast<i64>(static_cast<i32>(gw(a, k))) + static_cast<i32>(gw(b, k)) + 1) >> 1)

VARITH_B(h_vminub, a.b[k] < b.b[k] ? a.b[k] : b.b[k])
VARITH_H(h_vminuh, gh(a, k) < gh(b, k) ? gh(a, k) : gh(b, k))
VARITH_W(h_vminuw, gw(a, k) < gw(b, k) ? gw(a, k) : gw(b, k))
VARITH_B(h_vmaxub, a.b[k] > b.b[k] ? a.b[k] : b.b[k])
VARITH_H(h_vmaxuh, gh(a, k) > gh(b, k) ? gh(a, k) : gh(b, k))
VARITH_W(h_vmaxuw, gw(a, k) > gw(b, k) ? gw(a, k) : gw(b, k))
VARITH_B(h_vminsb, static_cast<i8>(a.b[k]) < static_cast<i8>(b.b[k]) ? a.b[k] : b.b[k])
VARITH_H(h_vminsh, static_cast<i16>(gh(a, k)) < static_cast<i16>(gh(b, k)) ? gh(a, k) : gh(b, k))
VARITH_W(h_vminsw, static_cast<i32>(gw(a, k)) < static_cast<i32>(gw(b, k)) ? gw(a, k) : gw(b, k))
VARITH_B(h_vmaxsb, static_cast<i8>(a.b[k]) > static_cast<i8>(b.b[k]) ? a.b[k] : b.b[k])
VARITH_H(h_vmaxsh, static_cast<i16>(gh(a, k)) > static_cast<i16>(gh(b, k)) ? gh(a, k) : gh(b, k))
VARITH_W(h_vmaxsw, static_cast<i32>(gw(a, k)) > static_cast<i32>(gw(b, k)) ? gw(a, k) : gw(b, k))

#undef VARITH_B
#undef VARITH_H
#undef VARITH_W

// ---- multiplies ------------------------------------------------------------
// Even/odd forms: element pairs by big-endian numbering (even = 0,2,..).

#define VMUL(NAME, LANES, GET, CAST, PUTW, OUT, ODD)                          \
    void NAME(Cpu& c, u32 i, const InsnDesc&)                                 \
    {                                                                         \
        const V128 a = VA(c, i), b = VB(c, i);                                \
        V128 v;                                                               \
        for (int k = 0; k < LANES; ++k)                                       \
            PUTW(v, k,                                                        \
                 static_cast<OUT>(CAST(GET(a, 2 * k + ODD)) *                 \
                                  CAST(GET(b, 2 * k + ODD))));                \
        VD(c, i) = v;                                                         \
    }
inline i32 cSB(u8 x) { return static_cast<i8>(x); }
inline i32 cUB(u8 x) { return x; }
inline i32 cSH(u16 x) { return static_cast<i16>(x); }
inline i32 cUH(u16 x) { return x; }
VMUL(h_vmulesb, 8, gb8, cSB, ph, u16, 0)
VMUL(h_vmuleub, 8, gb8, cUB, ph, u16, 0)
VMUL(h_vmulesh, 4, gh, cSH, pw, u32, 0)
VMUL(h_vmuleuh, 4, gh, cUH, pw, u32, 0)
VMUL(h_vmulosb, 8, gb8, cSB, ph, u16, 1)
VMUL(h_vmuloub, 8, gb8, cUB, ph, u16, 1)
VMUL(h_vmulosh, 4, gh, cSH, pw, u32, 1)
VMUL(h_vmulouh, 4, gh, cUH, pw, u32, 1)
#undef VMUL

void h_vmladduhm(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 a = VA(c, i), b = VB(c, i), cc = VC(c, i);
    V128 v;
    for (int k = 0; k < 8; ++k)
        ph(v, k, static_cast<u16>(gh(a, k) * gh(b, k) + gh(cc, k)));
    VD(c, i) = v;
}
void h_vmhaddshs(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 a = VA(c, i), b = VB(c, i), cc = VC(c, i);
    V128 v;
    for (int k = 0; k < 8; ++k) {
        const i32 prod = static_cast<i16>(gh(a, k)) * static_cast<i16>(gh(b, k));
        const i32 x = (prod >> 15) + static_cast<i16>(gh(cc, k));
        ph(v, k, static_cast<u16>(satS16(c, x)));
    }
    VD(c, i) = v;
}
void h_vmhraddshs(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 a = VA(c, i), b = VB(c, i), cc = VC(c, i);
    V128 v;
    for (int k = 0; k < 8; ++k) {
        const i32 prod = static_cast<i16>(gh(a, k)) * static_cast<i16>(gh(b, k));
        const i32 x = ((prod + 0x4000) >> 15) + static_cast<i16>(gh(cc, k));
        ph(v, k, static_cast<u16>(satS16(c, x)));
    }
    VD(c, i) = v;
}

// msum family: per-word dot products accumulated with the vC word.
void h_vmsumubm(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 a = VA(c, i), b = VB(c, i), cc = VC(c, i);
    V128 v;
    for (int k = 0; k < 4; ++k) {
        u32 s = gw(cc, k);
        for (int j = 0; j < 4; ++j)
            s += static_cast<u32>(a.b[4 * k + j]) * b.b[4 * k + j];
        pw(v, k, s);
    }
    VD(c, i) = v;
}
void h_vmsummbm(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 a = VA(c, i), b = VB(c, i), cc = VC(c, i);
    V128 v;
    for (int k = 0; k < 4; ++k) {
        i32 s = static_cast<i32>(gw(cc, k));
        for (int j = 0; j < 4; ++j)
            s += static_cast<i8>(a.b[4 * k + j]) * b.b[4 * k + j];
        pw(v, k, static_cast<u32>(s));
    }
    VD(c, i) = v;
}
void h_vmsumuhm(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 a = VA(c, i), b = VB(c, i), cc = VC(c, i);
    V128 v;
    for (int k = 0; k < 4; ++k) {
        u32 s = gw(cc, k);
        for (int j = 0; j < 2; ++j)
            s += static_cast<u32>(gh(a, 2 * k + j)) * gh(b, 2 * k + j);
        pw(v, k, s);
    }
    VD(c, i) = v;
}
void h_vmsumshm(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 a = VA(c, i), b = VB(c, i), cc = VC(c, i);
    V128 v;
    for (int k = 0; k < 4; ++k) {
        i32 s = static_cast<i32>(gw(cc, k));
        for (int j = 0; j < 2; ++j)
            s += static_cast<i16>(gh(a, 2 * k + j)) * static_cast<i16>(gh(b, 2 * k + j));
        pw(v, k, static_cast<u32>(s));
    }
    VD(c, i) = v;
}
void h_vmsumuhs(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 a = VA(c, i), b = VB(c, i), cc = VC(c, i);
    V128 v;
    for (int k = 0; k < 4; ++k) {
        u64 s = gw(cc, k);
        for (int j = 0; j < 2; ++j)
            s += static_cast<u64>(gh(a, 2 * k + j)) * gh(b, 2 * k + j);
        pw(v, k, satU32(c, static_cast<i64>(s)));
    }
    VD(c, i) = v;
}
void h_vmsumshs(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 a = VA(c, i), b = VB(c, i), cc = VC(c, i);
    V128 v;
    for (int k = 0; k < 4; ++k) {
        i64 s = static_cast<i32>(gw(cc, k));
        for (int j = 0; j < 2; ++j)
            s += static_cast<i16>(gh(a, 2 * k + j)) * static_cast<i16>(gh(b, 2 * k + j));
        pw(v, k, static_cast<u32>(satS32(c, s)));
    }
    VD(c, i) = v;
}

// sum-across family.
void h_vsumsws(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 a = VA(c, i), b = VB(c, i);
    i64 s = static_cast<i32>(gw(b, 3));
    for (int k = 0; k < 4; ++k)
        s += static_cast<i32>(gw(a, k));
    V128 v{};
    pw(v, 3, static_cast<u32>(satS32(c, s)));
    VD(c, i) = v;
}
void h_vsum2sws(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 a = VA(c, i), b = VB(c, i);
    V128 v{};
    for (int k = 0; k < 2; ++k) {
        i64 s = static_cast<i32>(gw(b, 2 * k + 1));
        s += static_cast<i32>(gw(a, 2 * k));
        s += static_cast<i32>(gw(a, 2 * k + 1));
        pw(v, 2 * k + 1, static_cast<u32>(satS32(c, s)));
    }
    VD(c, i) = v;
}
void h_vsum4sbs(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 a = VA(c, i), b = VB(c, i);
    V128 v;
    for (int k = 0; k < 4; ++k) {
        i64 s = static_cast<i32>(gw(b, k));
        for (int j = 0; j < 4; ++j)
            s += static_cast<i8>(a.b[4 * k + j]);
        pw(v, k, static_cast<u32>(satS32(c, s)));
    }
    VD(c, i) = v;
}
void h_vsum4shs(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 a = VA(c, i), b = VB(c, i);
    V128 v;
    for (int k = 0; k < 4; ++k) {
        i64 s = static_cast<i32>(gw(b, k));
        for (int j = 0; j < 2; ++j)
            s += static_cast<i16>(gh(a, 2 * k + j));
        pw(v, k, static_cast<u32>(satS32(c, s)));
    }
    VD(c, i) = v;
}
void h_vsum4ubs(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 a = VA(c, i), b = VB(c, i);
    V128 v;
    for (int k = 0; k < 4; ++k) {
        u64 s = gw(b, k);
        for (int j = 0; j < 4; ++j)
            s += a.b[4 * k + j];
        pw(v, k, satU32(c, static_cast<i64>(s)));
    }
    VD(c, i) = v;
}

// ---- compares (Rc -> CR6) --------------------------------------------------

void setCr6(Cpu& c, u32 i, bool allTrue, bool allFalse)
{
    if (f_vrcbit(i))
        c.setCrField(6, (allTrue ? 8u : 0u) | (allFalse ? 2u : 0u));
}

#define VCMP(NAME, LANES, GET, PUT, TEST, ONES)                               \
    void NAME(Cpu& c, u32 i, const InsnDesc&)                                 \
    {                                                                         \
        const V128 a = VA(c, i), b = VB(c, i);                                \
        V128 v;                                                               \
        bool allT = true, allF = true;                                        \
        for (int k = 0; k < LANES; ++k) {                                     \
            const bool t = (TEST);                                            \
            (t ? allF : allT) = false;                                        \
            PUT(v, k, t ? ONES : 0);                                          \
        }                                                                     \
        VD(c, i) = v;                                                         \
        setCr6(c, i, allT, allF);                                             \
    }
VCMP(h_vcmpequb, 16, gb8, pb8, a.b[k] == b.b[k], 0xFFu)
VCMP(h_vcmpequh, 8, gh, ph, gh(a, k) == gh(b, k), 0xFFFFu)
VCMP(h_vcmpequw, 4, gw, pw, gw(a, k) == gw(b, k), 0xFFFFFFFFu)
VCMP(h_vcmpgtub, 16, gb8, pb8, a.b[k] > b.b[k], 0xFFu)
VCMP(h_vcmpgtuh, 8, gh, ph, gh(a, k) > gh(b, k), 0xFFFFu)
VCMP(h_vcmpgtuw, 4, gw, pw, gw(a, k) > gw(b, k), 0xFFFFFFFFu)
VCMP(h_vcmpgtsb, 16, gb8, pb8, static_cast<i8>(a.b[k]) > static_cast<i8>(b.b[k]), 0xFFu)
VCMP(h_vcmpgtsh, 8, gh, ph, static_cast<i16>(gh(a, k)) > static_cast<i16>(gh(b, k)), 0xFFFFu)
VCMP(h_vcmpgtsw, 4, gw, pw, static_cast<i32>(gw(a, k)) > static_cast<i32>(gw(b, k)), 0xFFFFFFFFu)
#undef VCMP

// ---- pack / unpack ---------------------------------------------------------

void h_vpkuhum(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 a = VA(c, i), b = VB(c, i);
    V128 v;
    for (int k = 0; k < 8; ++k) {
        v.b[k] = static_cast<u8>(gh(a, k));
        v.b[k + 8] = static_cast<u8>(gh(b, k));
    }
    VD(c, i) = v;
}
void h_vpkuwum(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 a = VA(c, i), b = VB(c, i);
    V128 v;
    for (int k = 0; k < 4; ++k) {
        ph(v, k, static_cast<u16>(gw(a, k)));
        ph(v, k + 4, static_cast<u16>(gw(b, k)));
    }
    VD(c, i) = v;
}

#define VPKS(NAME, LANES, GETSRC, PUTDST, SATFN, CAST)                        \
    void NAME(Cpu& c, u32 i, const InsnDesc&)                                 \
    {                                                                         \
        const V128 a = VA(c, i), b = VB(c, i);                                \
        V128 v;                                                               \
        for (int k = 0; k < LANES; ++k) {                                     \
            PUTDST(v, k, SATFN(c, CAST(GETSRC(a, k))));                       \
            PUTDST(v, k + LANES, SATFN(c, CAST(GETSRC(b, k))));               \
        }                                                                     \
        VD(c, i) = v;                                                         \
    }
inline void pb8u(V128& v, int i, u8 x) { v.b[i] = x; }
inline void pb8s(V128& v, int i, i8 x) { v.b[i] = static_cast<u8>(x); }
inline void phs(V128& v, int i, i16 x) { ph(v, i, static_cast<u16>(x)); }
inline void phu(V128& v, int i, u16 x) { ph(v, i, x); }
inline i64 cU16(u16 x) { return x; }
inline i64 cS16(u16 x) { return static_cast<i16>(x); }
inline i64 cU32(u32 x) { return x; }
inline i64 cS32(u32 x) { return static_cast<i32>(x); }
VPKS(h_vpkuhus, 8, gh, pb8u, satU8, cU16)
VPKS(h_vpkshus, 8, gh, pb8u, satU8, cS16)
VPKS(h_vpkshss, 8, gh, pb8s, satS8, cS16)
VPKS(h_vpkuwus, 4, gw, phu, satU16, cU32)
VPKS(h_vpkswus, 4, gw, phu, satU16, cS32)
VPKS(h_vpkswss, 4, gw, phs, satS16, cS32)
#undef VPKS

void h_vpkpx(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 a = VA(c, i), b = VB(c, i);
    V128 v;
    for (int k = 0; k < 4; ++k) {
        const u32 wa = gw(a, k), wb = gw(b, k);
        const u16 pa = static_cast<u16>(((wa >> 9) & 0xFC00u) |
                                        ((wa >> 6) & 0x03E0u) |
                                        ((wa >> 3) & 0x001Fu));
        const u16 pb = static_cast<u16>(((wb >> 9) & 0xFC00u) |
                                        ((wb >> 6) & 0x03E0u) |
                                        ((wb >> 3) & 0x001Fu));
        ph(v, k, pa);
        ph(v, k + 4, pb);
    }
    VD(c, i) = v;
}

void h_vupkhsb(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 b = VB(c, i);
    V128 v;
    for (int k = 0; k < 8; ++k)
        ph(v, k, static_cast<u16>(static_cast<i16>(static_cast<i8>(b.b[k]))));
    VD(c, i) = v;
}
void h_vupklsb(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 b = VB(c, i);
    V128 v;
    for (int k = 0; k < 8; ++k)
        ph(v, k, static_cast<u16>(static_cast<i16>(static_cast<i8>(b.b[k + 8]))));
    VD(c, i) = v;
}
void h_vupkhsh(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 b = VB(c, i);
    V128 v;
    for (int k = 0; k < 4; ++k)
        pw(v, k, static_cast<u32>(static_cast<i32>(static_cast<i16>(gh(b, k)))));
    VD(c, i) = v;
}
void h_vupklsh(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 b = VB(c, i);
    V128 v;
    for (int k = 0; k < 4; ++k)
        pw(v, k, static_cast<u32>(static_cast<i32>(static_cast<i16>(gh(b, k + 4)))));
    VD(c, i) = v;
}
void h_vupkhpx(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 b = VB(c, i);
    V128 v;
    for (int k = 0; k < 4; ++k) {
        const u16 p = gh(b, k);
        pw(v, k, (static_cast<u32>(p & 0x8000u) ? 0xFF000000u : 0u) |
                     ((p & 0x7C00u) << 6) | ((p & 0x03E0u) << 3) | (p & 0x1Fu));
    }
    VD(c, i) = v;
}
void h_vupklpx(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 b = VB(c, i);
    V128 v;
    for (int k = 0; k < 4; ++k) {
        const u16 p = gh(b, k + 4);
        pw(v, k, (static_cast<u32>(p & 0x8000u) ? 0xFF000000u : 0u) |
                     ((p & 0x7C00u) << 6) | ((p & 0x03E0u) << 3) | (p & 0x1Fu));
    }
    VD(c, i) = v;
}

// ---- VSCR ------------------------------------------------------------------

void h_mfvscr(Cpu& c, u32 i, const InsnDesc&)
{
    V128 v{};
    pw(v, 3, c.st.vscr);
    VD(c, i) = v;
}
void h_mtvscr(Cpu& c, u32 i, const InsnDesc&)
{
    c.st.vscr = gw(VB(c, i), 3) & (vNJ | vSAT);
}

// ---- vector float ----------------------------------------------------------
// Lanes are IEEE single stored as single bit patterns; computation reuses the
// deterministic softfloat at Tgt::Sgl through a double widening (exact for
// all singles). Always round-to-nearest; no exceptions; QNaN results for
// invalid operations; NJ=1 flushes denormal inputs and results.

inline bool sglDenorm(u32 w) { return (w & 0x7F800000u) == 0 && (w & 0x007FFFFFu) != 0; }
inline u32 flushIn(const Cpu& c, u32 w)
{
    return ((c.st.vscr & vNJ) && sglDenorm(w)) ? (w & 0x80000000u) : w;
}
inline u32 flushOut(const Cpu& c, u32 w)
{
    return ((c.st.vscr & vNJ) && sglDenorm(w)) ? (w & 0x80000000u) : w;
}

inline u64 up(u32 w) { return sf::loadSingle(w); }
inline u32 dn(u64 d) { return sf::storeSingle(d); } // exact for single results

inline sf::Env vecEnv()
{
    return {0u, false, false, false}; // RN, no traps, NJ handled at lane level
}

#define VFP3(NAME, CALL)                                                      \
    void NAME(Cpu& c, u32 i, const InsnDesc&)                                 \
    {                                                                         \
        const V128 a = VA(c, i), b = VB(c, i);                                \
        V128 v;                                                               \
        for (int k = 0; k < 4; ++k) {                                         \
            const u64 x = up(flushIn(c, gw(a, k)));                           \
            const u64 y = up(flushIn(c, gw(b, k)));                           \
            (void)x; (void)y;                                                 \
            pw(v, k, flushOut(c, dn((CALL).bits)));                           \
        }                                                                     \
        VD(c, i) = v;                                                         \
    }
VFP3(h_vaddfp, sf::add(x, y, vecEnv(), sf::Tgt::Sgl))
VFP3(h_vsubfp, sf::sub(x, y, vecEnv(), sf::Tgt::Sgl))
#undef VFP3

void h_vmaddfp(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 a = VA(c, i), b = VB(c, i), cc = VC(c, i);
    V128 v;
    for (int k = 0; k < 4; ++k) {
        const u64 x = up(flushIn(c, gw(a, k)));
        const u64 y = up(flushIn(c, gw(cc, k)));
        const u64 z = up(flushIn(c, gw(b, k)));
        pw(v, k, flushOut(c, dn(sf::madd(x, y, z, vecEnv(), sf::Tgt::Sgl,
                                         false, false)
                                    .bits)));
    }
    VD(c, i) = v;
}
void h_vnmsubfp(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 a = VA(c, i), b = VB(c, i), cc = VC(c, i);
    V128 v;
    for (int k = 0; k < 4; ++k) {
        const u64 x = up(flushIn(c, gw(a, k)));
        const u64 y = up(flushIn(c, gw(cc, k)));
        const u64 z = up(flushIn(c, gw(b, k)));
        pw(v, k, flushOut(c, dn(sf::madd(x, y, z, vecEnv(), sf::Tgt::Sgl,
                                         true, true)
                                    .bits)));
    }
    VD(c, i) = v;
}

inline bool sglNan(u32 w)
{
    return (w & 0x7F800000u) == 0x7F800000u && (w & 0x007FFFFFu) != 0;
}
inline u32 sglQuiet(u32 w) { return w | 0x00400000u; }

// min/max: if either operand is a NaN the result is a QNaN (PEM ch.6).
#define VMINMAX(NAME, CMP)                                                    \
    void NAME(Cpu& c, u32 i, const InsnDesc&)                                 \
    {                                                                         \
        const V128 a = VA(c, i), b = VB(c, i);                                \
        V128 v;                                                               \
        for (int k = 0; k < 4; ++k) {                                         \
            const u32 wa = flushIn(c, gw(a, k)), wb = flushIn(c, gw(b, k));   \
            u32 out;                                                          \
            if (sglNan(wa))                                                   \
                out = sglQuiet(wa);                                           \
            else if (sglNan(wb))                                              \
                out = sglQuiet(wb);                                           \
            else                                                              \
                out = (CMP) ? wa : wb;                                        \
            pw(v, k, out);                                                    \
        }                                                                     \
        VD(c, i) = v;                                                         \
    }
inline bool sglGreater(u32 x, u32 y)
{
    // ordered compare on single bit patterns (zeros equal regardless of sign)
    const bool zx = (x << 1) == 0, zy = (y << 1) == 0;
    if (zx && zy)
        return false;
    const bool sx = x >> 31, sy = y >> 31;
    if (zx)
        return sy;
    if (zy)
        return !sx;
    if (sx != sy)
        return sy;
    const u32 mx = x & 0x7FFFFFFFu, my = y & 0x7FFFFFFFu;
    return (mx > my) != sx;
}
VMINMAX(h_vmaxfp, sglGreater(wa, wb))
VMINMAX(h_vminfp, sglGreater(wb, wa))
#undef VMINMAX

#define VFCMP(NAME, LANETEST)                                                 \
    void NAME(Cpu& c, u32 i, const InsnDesc&)                                 \
    {                                                                         \
        const V128 a = VA(c, i), b = VB(c, i);                                \
        V128 v;                                                               \
        bool allT = true, allF = true;                                        \
        for (int k = 0; k < 4; ++k) {                                         \
            const u32 wa = flushIn(c, gw(a, k)), wb = flushIn(c, gw(b, k));   \
            const bool nan = sglNan(wa) || sglNan(wb);                        \
            const bool t = !nan && (LANETEST);                                \
            (t ? allF : allT) = false;                                        \
            pw(v, k, t ? 0xFFFFFFFFu : 0u);                                   \
        }                                                                     \
        VD(c, i) = v;                                                         \
        setCr6(c, i, allT, allF);                                             \
    }
inline bool sglEqual(u32 x, u32 y)
{
    if ((x << 1) == 0 && (y << 1) == 0)
        return true;
    return x == y;
}
VFCMP(h_vcmpeqfp, sglEqual(wa, wb))
VFCMP(h_vcmpgtfp, sglGreater(wa, wb))
VFCMP(h_vcmpgefp, sglEqual(wa, wb) || sglGreater(wa, wb))
#undef VFCMP

// vcmpbfp: bounds check; lane bits [le -bound, ge +bound] cleared per test;
// CR6[2] (Rc form) = all lanes fully in bounds.
void h_vcmpbfp(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 a = VA(c, i), b = VB(c, i);
    V128 v;
    bool allIn = true;
    for (int k = 0; k < 4; ++k) {
        const u32 wa = flushIn(c, gw(a, k)), wb = flushIn(c, gw(b, k));
        u32 le, ge;
        if (sglNan(wa) || sglNan(wb)) {
            le = ge = 1u; // out of bounds both ways
        } else {
            const u32 nb = wb ^ 0x80000000u; // -bound
            le = sglGreater(wa, wb) ? 1u : 0u;           // a > bound
            ge = sglGreater(nb, wa) ? 1u : 0u;           // a < -bound
        }
        if (le | ge)
            allIn = false;
        pw(v, k, (le << 31) | (ge << 30));
    }
    VD(c, i) = v;
    if (f_vrcbit(i))
        c.setCrField(6, allIn ? 2u : 0u);
}

// ---- float conversions / rounds --------------------------------------------

void h_vcfux(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 b = VB(c, i);
    const u32 scale = f_ra(i) & 31u;
    V128 v;
    for (int k = 0; k < 4; ++k) {
        const u32 w = gw(b, k);
        if (w == 0) {
            pw(v, k, 0);
            continue;
        }
        // exact integer -> double, then adjust exponent by -scale, round to single
        u64 d = 0;
        const int m = 31 - std::countl_zero(w);
        const u64 mant = (static_cast<u64>(w) << (52 - m)) & 0x000FFFFFFFFFFFFFull;
        d = (static_cast<u64>(1023 + m - static_cast<int>(scale)) << 52) | mant;
        u32 fl = 0;
        (void)fl;
        pw(v, k, dn(sf::rsp(d, vecEnv()).bits));
    }
    VD(c, i) = v;
}
void h_vcfsx(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 b = VB(c, i);
    const u32 scale = f_ra(i) & 31u;
    V128 v;
    for (int k = 0; k < 4; ++k) {
        const i32 sw = static_cast<i32>(gw(b, k));
        if (sw == 0) {
            pw(v, k, 0);
            continue;
        }
        const u32 mag = sw < 0 ? 0u - static_cast<u32>(sw) : static_cast<u32>(sw);
        const int m = 31 - std::countl_zero(mag);
        const u64 mant = (static_cast<u64>(mag) << (52 - m)) & 0x000FFFFFFFFFFFFFull;
        u64 d = (static_cast<u64>(1023 + m - static_cast<int>(scale)) << 52) | mant;
        if (sw < 0)
            d |= 0x8000000000000000ull;
        pw(v, k, dn(sf::rsp(d, vecEnv()).bits));
    }
    VD(c, i) = v;
}

// float -> fixed with saturation: value * 2^scale, truncate toward zero.
void ctx(Cpu& c, u32 i, bool sgn)
{
    const V128 b = VB(c, i);
    const u32 scale = f_ra(i) & 31u;
    V128 v;
    for (int k = 0; k < 4; ++k) {
        const u32 w = flushIn(c, gw(b, k));
        u32 out;
        if (sglNan(w)) {
            out = 0; // all NaNs convert to zero (PEM 3.2.5.4); SAT untouched
        } else {
            const u64 d = up(w);
            const bool neg = (d >> 63) != 0;
            const u32 e = static_cast<u32>(d >> 52) & 0x7FFu;
            const u64 frac = (d & 0x000FFFFFFFFFFFFFull) | (e ? (1ull << 52) : 0);
            const i32 exp = static_cast<i32>(e) - 1023 + static_cast<i32>(scale);
            u64 mag;
            if (e == 0 && frac == 0) {
                mag = 0;
            } else if (exp < 0) {
                mag = 0;
            } else if (exp > 62) {
                mag = ~0ull;
            } else {
                mag = exp >= 52 ? frac << (exp - 52) : frac >> (52 - exp);
            }
            if (sgn) {
                if (!neg)
                    out = mag > 0x7FFFFFFFull ? (sat(c), 0x7FFFFFFFu)
                                              : static_cast<u32>(mag);
                else
                    out = mag > 0x80000000ull
                              ? (sat(c), 0x80000000u)
                              : static_cast<u32>(0u - static_cast<u32>(mag));
            } else {
                if (neg)
                    out = mag ? (sat(c), 0u) : 0u;
                else
                    out = mag > 0xFFFFFFFFull ? (sat(c), 0xFFFFFFFFu)
                                              : static_cast<u32>(mag);
            }
        }
        pw(v, k, out);
    }
    VD(c, i) = v;
}
void h_vctuxs(Cpu& c, u32 i, const InsnDesc&) { ctx(c, i, false); }
void h_vctsxs(Cpu& c, u32 i, const InsnDesc&) { ctx(c, i, true); }

// vrfin/vrfiz/vrfip/vrfim: round to integral single.
void vrfi(Cpu& c, u32 i, u32 rn)
{
    const V128 b = VB(c, i);
    V128 v;
    for (int k = 0; k < 4; ++k) {
        const u32 w = flushIn(c, gw(b, k));
        u32 out;
        if (sglNan(w)) {
            out = sglQuiet(w);
        } else {
            const u64 d = up(w);
            const u32 e = static_cast<u32>(d >> 52) & 0x7FFu;
            if (e >= 1023 + 23) { // already integral (or inf)
                out = w;
            } else {
                // round the double at integer granularity, then to single
                const bool neg = (d >> 63) != 0;
                const u64 frac = (d & 0x000FFFFFFFFFFFFFull) |
                                 (e ? (1ull << 52) : 0);
                const i32 exp = static_cast<i32>(e) - 1023;
                u64 ip;
                bool half = false, rest = false;
                if (exp < -1) {
                    ip = 0;
                    half = exp == -1;
                    rest = (frac << 1) != 0 || exp < -1;
                    if (exp == -1) {
                        half = true;
                        rest = (frac & 0x000FFFFFFFFFFFFFull) != 0;
                    } else {
                        half = false;
                        rest = frac != 0;
                    }
                } else {
                    const u32 fbits = 52 - static_cast<u32>(exp);
                    ip = frac >> fbits;
                    const u64 below = frac << (64 - fbits);
                    half = (below >> 63) != 0;
                    rest = (below << 1) != 0;
                }
                bool inc = false;
                switch (rn) {
                case 0: inc = half && (rest || (ip & 1)); break;
                case 1: inc = false; break;
                case 2: inc = !neg && (half || rest); break;
                case 3: inc = neg && (half || rest); break;
                }
                ip += inc ? 1 : 0;
                if (ip == 0) {
                    out = w & 0x80000000u; // signed zero
                } else {
                    const int m = 63 - std::countl_zero(ip);
                    const u64 mant =
                        (ip << (52 - m)) & 0x000FFFFFFFFFFFFFull;
                    u64 rd = (static_cast<u64>(1023 + m) << 52) | mant;
                    if (neg)
                        rd |= 0x8000000000000000ull;
                    out = dn(rd); // integral values are single-exact here
                }
            }
        }
        pw(v, k, out);
    }
    VD(c, i) = v;
}
void h_vrfin(Cpu& c, u32 i, const InsnDesc&) { vrfi(c, i, 0); }
void h_vrfiz(Cpu& c, u32 i, const InsnDesc&) { vrfi(c, i, 1); }
void h_vrfip(Cpu& c, u32 i, const InsnDesc&) { vrfi(c, i, 2); }
void h_vrfim(Cpu& c, u32 i, const InsnDesc&) { vrfi(c, i, 3); }

// ---- estimates (LEDGER: deterministic, not silicon tables) -----------------

void h_vrefp(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 b = VB(c, i);
    V128 v;
    for (int k = 0; k < 4; ++k) {
        const u32 w = flushIn(c, gw(b, k));
        u32 out;
        if (sglNan(w))
            out = sglQuiet(w);
        else
            out = flushOut(c, dn(sf::res(up(w), vecEnv()).bits));
        pw(v, k, out);
    }
    VD(c, i) = v;
}
void h_vrsqrtefp(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 b = VB(c, i);
    V128 v;
    for (int k = 0; k < 4; ++k) {
        const u32 w = flushIn(c, gw(b, k));
        u32 out;
        if (sglNan(w)) {
            out = sglQuiet(w);
        } else {
            const u64 r = sf::rsqrte(up(w), vecEnv()).bits;
            u32 fl = 0;
            (void)fl;
            out = flushOut(c, dn(sf::rsp(r, vecEnv()).bits));
        }
        pw(v, k, out);
    }
    VD(c, i) = v;
}

// log2/2^x estimates: exponent part exact, fractional part via a fixed cubic
// on the mantissa — deterministic, inside the architectural bounds.
void h_vlogefp(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 b = VB(c, i);
    V128 v;
    for (int k = 0; k < 4; ++k) {
        const u32 w = flushIn(c, gw(b, k));
        u32 out;
        if (sglNan(w)) {
            out = sglQuiet(w);
        } else if ((w << 1) == 0) {
            out = 0xFF800000u; // log2(±0) = -inf
        } else if (w >> 31) {
            out = 0x7FC00000u; // log2(negative) = QNaN
        } else if (w == 0x7F800000u) {
            out = w; // log2(+inf) = +inf
        } else {
            const u64 d = up(w);
            const i32 e = static_cast<i32>((d >> 52) & 0x7FF) - 1023;
            // t in [0,1): mantissa fraction; log2(1+t) ~ cubic minimax
            const u64 frac = d & 0x000FFFFFFFFFFFFFull;
            const i64 t = static_cast<i64>(frac >> 22); // Q30
            // c1*t + c2*t^2 + c3*t^3, Q30 coefficients
            const i64 c1 = 1549082004, c2 = -623693658, c3 = 148656453;
            const i64 t2 = (t * t) >> 30;
            const i64 t3 = (t2 * t) >> 30;
            const i64 lf = (c1 * t + c2 * t2 + c3 * t3) >> 30; // Q30
            // value = e + lf/2^30, build as double then round to single
            const i64 q = (static_cast<i64>(e) << 30) + lf;    // Q30
            if (q == 0) {
                out = 0;
            } else {
                const bool neg = q < 0;
                const u64 mag = static_cast<u64>(neg ? -q : q);
                const int m = 63 - std::countl_zero(mag);
                const u64 mant = (mag << (52 - m)) & 0x000FFFFFFFFFFFFFull;
                u64 rd = (static_cast<u64>(1023 + m - 30) << 52) | mant;
                if (neg)
                    rd |= 0x8000000000000000ull;
                out = dn(sf::rsp(rd, vecEnv()).bits);
            }
        }
        pw(v, k, flushOut(c, out));
    }
    VD(c, i) = v;
}
void h_vexptefp(Cpu& c, u32 i, const InsnDesc&)
{
    const V128 b = VB(c, i);
    V128 v;
    for (int k = 0; k < 4; ++k) {
        const u32 w = flushIn(c, gw(b, k));
        u32 out;
        if (sglNan(w)) {
            out = sglQuiet(w);
        } else if (w == 0x7F800000u) {
            out = w; // 2^inf = inf
        } else if (w == 0xFF800000u) {
            out = 0; // 2^-inf = +0
        } else {
            const u64 d = up(w);
            const bool neg = (d >> 63) != 0;
            const u32 e = static_cast<u32>(d >> 52) & 0x7FF;
            const i32 exp = static_cast<i32>(e) - 1023;
            i64 q; // value in Q30
            if (e == 0) {
                q = 0;
            } else if (exp > 8) {
                q = neg ? -(200ll << 30) : (200ll << 30); // saturates to 0/inf
            } else if (exp < -31) {
                q = 0; // magnitude below Q30 resolution: 2^~0 = 1
            } else {
                const u64 frac = (d & 0x000FFFFFFFFFFFFFull) | (1ull << 52);
                const u64 mag = exp >= 0 ? (frac >> (52 - 30 - exp))
                                         : (frac >> (52 - 30)) >> (-exp);
                q = neg ? -static_cast<i64>(mag) : static_cast<i64>(mag);
            }
            const i64 ip = q >> 30;          // floor
            const i64 t = q & ((1ll << 30) - 1); // frac in [0,1) Q30
            // 2^t ~ 1 + c1 t + c2 t^2 + c3 t^3 (Q30)
            const i64 c1 = 744822652, c2 = 258141149, c3 = 71624073;
            const i64 t2 = (t * t) >> 30;
            const i64 t3 = (t2 * t) >> 30;
            const i64 mf = (1ll << 30) + ((c1 * t + c2 * t2 + c3 * t3) >> 30);
            // result = mf * 2^ip
            const i64 rexp = ip + 0; // mf in [1,2) Q30
            if (rexp < -150) {
                out = 0;
            } else if (rexp > 128) {
                out = 0x7F800000u;
            } else {
                const u64 mag = static_cast<u64>(mf); // Q30, in [2^30, 2^31)
                const int m = 63 - std::countl_zero(mag);
                const u64 mant = (mag << (52 - m)) & 0x000FFFFFFFFFFFFFull;
                const i64 de = rexp + (m - 30);
                u64 rd = (static_cast<u64>(1023 + de) << 52) | mant;
                out = dn(sf::rsp(rd, vecEnv()).bits);
            }
        }
        pw(v, k, flushOut(c, out));
    }
    VD(c, i) = v;
}

} // namespace

void bindVecHandlers()
{
    setHandler("lvx", h_lvx);
    setHandler("lvxl", h_lvx);
    setHandler("stvx", h_stvx);
    setHandler("stvxl", h_stvx);
    setHandler("lvebx", h_lvebx);
    setHandler("lvehx", h_lvehx);
    setHandler("lvewx", h_lvewx);
    setHandler("stvebx", h_stvebx);
    setHandler("stvehx", h_stvehx);
    setHandler("stvewx", h_stvewx);
    setHandler("lvsl", h_lvsl);
    setHandler("lvsr", h_lvsr);

    setHandler("vperm", h_vperm);
    setHandler("vsel", h_vsel);
    setHandler("vsldoi", h_vsldoi);
    setHandler("vspltb", h_vspltb);
    setHandler("vsplth", h_vsplth);
    setHandler("vspltw", h_vspltw);
    setHandler("vspltisb", h_vspltisb);
    setHandler("vspltish", h_vspltish);
    setHandler("vspltisw", h_vspltisw);
    setHandler("vmrghb", h_vmrghb);
    setHandler("vmrglb", h_vmrglb);
    setHandler("vmrghh", h_vmrghh);
    setHandler("vmrglh", h_vmrglh);
    setHandler("vmrghw", h_vmrghw);
    setHandler("vmrglw", h_vmrglw);

    setHandler("vslo", h_vslo);
    setHandler("vsro", h_vsro);
    setHandler("vsl", h_vsl);
    setHandler("vsr", h_vsr);
    setHandler("vrlb", h_vrlb);
    setHandler("vrlh", h_vrlh);
    setHandler("vrlw", h_vrlw);
    setHandler("vslb", h_vslb);
    setHandler("vslh", h_vslh);
    setHandler("vslw", h_vslw);
    setHandler("vsrb", h_vsrb);
    setHandler("vsrh", h_vsrh);
    setHandler("vsrw", h_vsrw);
    setHandler("vsrab", h_vsrab);
    setHandler("vsrah", h_vsrah);
    setHandler("vsraw", h_vsraw);

    setHandler("vand", h_vand);
    setHandler("vandc", h_vandc);
    setHandler("vor", h_vor);
    setHandler("vnor", h_vnor);
    setHandler("vxor", h_vxor);

    setHandler("vaddubm", h_vaddubm);
    setHandler("vadduhm", h_vadduhm);
    setHandler("vadduwm", h_vadduwm);
    setHandler("vsububm", h_vsububm);
    setHandler("vsubuhm", h_vsubuhm);
    setHandler("vsubuwm", h_vsubuwm);
    setHandler("vaddubs", h_vaddubs);
    setHandler("vadduhs", h_vadduhs);
    setHandler("vadduws", h_vadduws);
    setHandler("vaddsbs", h_vaddsbs);
    setHandler("vaddshs", h_vaddshs);
    setHandler("vaddsws", h_vaddsws);
    setHandler("vsububs", h_vsububs);
    setHandler("vsubuhs", h_vsubuhs);
    setHandler("vsubuws", h_vsubuws);
    setHandler("vsubsbs", h_vsubsbs);
    setHandler("vsubshs", h_vsubshs);
    setHandler("vsubsws", h_vsubsws);
    setHandler("vaddcuw", h_vaddcuw);
    setHandler("vsubcuw", h_vsubcuw);

    setHandler("vavgub", h_vavgub);
    setHandler("vavguh", h_vavguh);
    setHandler("vavguw", h_vavguw);
    setHandler("vavgsb", h_vavgsb);
    setHandler("vavgsh", h_vavgsh);
    setHandler("vavgsw", h_vavgsw);
    setHandler("vminub", h_vminub);
    setHandler("vminuh", h_vminuh);
    setHandler("vminuw", h_vminuw);
    setHandler("vmaxub", h_vmaxub);
    setHandler("vmaxuh", h_vmaxuh);
    setHandler("vmaxuw", h_vmaxuw);
    setHandler("vminsb", h_vminsb);
    setHandler("vminsh", h_vminsh);
    setHandler("vminsw", h_vminsw);
    setHandler("vmaxsb", h_vmaxsb);
    setHandler("vmaxsh", h_vmaxsh);
    setHandler("vmaxsw", h_vmaxsw);

    setHandler("vmulesb", h_vmulesb);
    setHandler("vmuleub", h_vmuleub);
    setHandler("vmulesh", h_vmulesh);
    setHandler("vmuleuh", h_vmuleuh);
    setHandler("vmulosb", h_vmulosb);
    setHandler("vmuloub", h_vmuloub);
    setHandler("vmulosh", h_vmulosh);
    setHandler("vmulouh", h_vmulouh);
    setHandler("vmladduhm", h_vmladduhm);
    setHandler("vmhaddshs", h_vmhaddshs);
    setHandler("vmhraddshs", h_vmhraddshs);
    setHandler("vmsumubm", h_vmsumubm);
    setHandler("vmsummbm", h_vmsummbm);
    setHandler("vmsumuhm", h_vmsumuhm);
    setHandler("vmsumshm", h_vmsumshm);
    setHandler("vmsumuhs", h_vmsumuhs);
    setHandler("vmsumshs", h_vmsumshs);
    setHandler("vsumsws", h_vsumsws);
    setHandler("vsum2sws", h_vsum2sws);
    setHandler("vsum4sbs", h_vsum4sbs);
    setHandler("vsum4shs", h_vsum4shs);
    setHandler("vsum4ubs", h_vsum4ubs);

    setHandler("vcmpequb", h_vcmpequb);
    setHandler("vcmpequh", h_vcmpequh);
    setHandler("vcmpequw", h_vcmpequw);
    setHandler("vcmpgtub", h_vcmpgtub);
    setHandler("vcmpgtuh", h_vcmpgtuh);
    setHandler("vcmpgtuw", h_vcmpgtuw);
    setHandler("vcmpgtsb", h_vcmpgtsb);
    setHandler("vcmpgtsh", h_vcmpgtsh);
    setHandler("vcmpgtsw", h_vcmpgtsw);

    setHandler("vpkuhum", h_vpkuhum);
    setHandler("vpkuwum", h_vpkuwum);
    setHandler("vpkuhus", h_vpkuhus);
    setHandler("vpkuwus", h_vpkuwus);
    setHandler("vpkshus", h_vpkshus);
    setHandler("vpkswus", h_vpkswus);
    setHandler("vpkshss", h_vpkshss);
    setHandler("vpkswss", h_vpkswss);
    setHandler("vpkpx", h_vpkpx);
    setHandler("vupkhsb", h_vupkhsb);
    setHandler("vupklsb", h_vupklsb);
    setHandler("vupkhsh", h_vupkhsh);
    setHandler("vupklsh", h_vupklsh);
    setHandler("vupkhpx", h_vupkhpx);
    setHandler("vupklpx", h_vupklpx);

    setHandler("mfvscr", h_mfvscr);
    setHandler("mtvscr", h_mtvscr);

    setHandler("vaddfp", h_vaddfp);
    setHandler("vsubfp", h_vsubfp);
    setHandler("vmaddfp", h_vmaddfp);
    setHandler("vnmsubfp", h_vnmsubfp);
    setHandler("vmaxfp", h_vmaxfp);
    setHandler("vminfp", h_vminfp);
    setHandler("vcmpeqfp", h_vcmpeqfp);
    setHandler("vcmpgtfp", h_vcmpgtfp);
    setHandler("vcmpgefp", h_vcmpgefp);
    setHandler("vcmpbfp", h_vcmpbfp);
    setHandler("vcfux", h_vcfux);
    setHandler("vcfsx", h_vcfsx);
    setHandler("vctuxs", h_vctuxs);
    setHandler("vctsxs", h_vctsxs);
    setHandler("vrfin", h_vrfin);
    setHandler("vrfiz", h_vrfiz);
    setHandler("vrfip", h_vrfip);
    setHandler("vrfim", h_vrfim);
    setHandler("vrefp", h_vrefp);
    setHandler("vrsqrtefp", h_vrsqrtefp);
    setHandler("vlogefp", h_vlogefp);
    setHandler("vexptefp", h_vexptefp);
}

} // namespace opm
