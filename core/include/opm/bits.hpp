#pragma once
#include "types.hpp"
#include <cstring>

// PowerPC bit numbering: bit 0 is the MOST significant bit of a 32-bit word,
// bit 31 the least. Every field extraction in the codebase goes through these
// helpers so the convention lives in exactly one place.

namespace opm {

// Value of the inclusive PPC-numbered bit range [hi..lo] of a 32-bit word.
constexpr u32 ppcbits(u32 w, unsigned hi, unsigned lo)
{
    const unsigned width = lo - hi + 1u;
    const u32 mask = (width >= 32u) ? 0xFFFFFFFFu : ((1u << width) - 1u);
    return (w >> (31u - lo)) & mask;
}

// Single PPC-numbered bit (0 = MSB).
constexpr u32 ppcbit(u32 w, unsigned bit)
{
    return (w >> (31u - bit)) & 1u;
}

// A mask with PPC-numbered bits [hi..lo] set. Supports the rlwinm wrap case
// (mb > me handled by callers composing two masks).
constexpr u32 ppcmask(unsigned mb, unsigned me)
{
    const u32 x = 0xFFFFFFFFu >> mb;
    const u32 y = (me == 31u) ? 0xFFFFFFFFu : ~(0xFFFFFFFFu >> (me + 1u));
    return (mb <= me) ? (x & y) : (x | y);
}

constexpr i32 sext16(u32 v) { return static_cast<i32>(static_cast<i16>(v & 0xFFFFu)); }
constexpr i32 sext26(u32 v) { return (static_cast<i32>(v << 6)) >> 6; }
constexpr i32 sext14(u32 v) { return (static_cast<i32>(v << 18)) >> 18; }
constexpr i32 sext5(u32 v)  { return (static_cast<i32>(v << 27)) >> 27; }

// Common instruction fields, named by their PEM positions.
constexpr u32 f_primary(u32 i) { return ppcbits(i, 0, 5); }
constexpr u32 f_rt(u32 i)      { return ppcbits(i, 6, 10); }
constexpr u32 f_ra(u32 i)      { return ppcbits(i, 11, 15); }
constexpr u32 f_rb(u32 i)      { return ppcbits(i, 16, 20); }
constexpr u32 f_rc_vec(u32 i)  { return ppcbits(i, 21, 25); }  // VA-form vC
constexpr u32 f_d(u32 i)       { return ppcbits(i, 16, 31); }
constexpr u32 f_xo10(u32 i)    { return ppcbits(i, 21, 30); }
constexpr u32 f_xo9(u32 i)     { return ppcbits(i, 22, 30); }
constexpr u32 f_xo5(u32 i)     { return ppcbits(i, 26, 30); }
constexpr u32 f_xo6v(u32 i)    { return ppcbits(i, 26, 31); }  // VA-form
constexpr u32 f_xo11v(u32 i)   { return ppcbits(i, 21, 31); }  // VX-form
constexpr u32 f_rcbit(u32 i)   { return ppcbit(i, 31); }
constexpr u32 f_vrcbit(u32 i)  { return ppcbit(i, 21); }  // VXR-form Rc
constexpr u32 f_oebit(u32 i)   { return ppcbit(i, 21); }
constexpr u32 f_lkbit(u32 i)   { return ppcbit(i, 31); }
constexpr u32 f_aabit(u32 i)   { return ppcbit(i, 30); }
constexpr u32 f_crfd(u32 i)    { return ppcbits(i, 6, 8); }
constexpr u32 f_crfs(u32 i)    { return ppcbits(i, 11, 13); }
constexpr u32 f_sh(u32 i)      { return ppcbits(i, 16, 20); }
constexpr u32 f_mb(u32 i)      { return ppcbits(i, 21, 25); }
constexpr u32 f_me(u32 i)      { return ppcbits(i, 26, 30); }
constexpr u32 f_spr(u32 i)     { return (ppcbits(i, 16, 20) << 5) | ppcbits(i, 11, 15); }
constexpr u32 f_crm(u32 i)     { return ppcbits(i, 12, 19); }
constexpr u32 f_fm(u32 i)      { return ppcbits(i, 7, 14); }
constexpr u32 f_nb(u32 i)      { return ppcbits(i, 16, 20); }
constexpr u32 f_to(u32 i)      { return ppcbits(i, 6, 10); }
constexpr u32 f_sr(u32 i)      { return ppcbits(i, 12, 15); }
constexpr u32 f_simm5(u32 i)   { return ppcbits(i, 11, 15); }
constexpr u32 f_vsh(u32 i)     { return ppcbits(i, 22, 25); }  // vsldoi SH

// Alignment-exception DSISR opcode image per PEM Table 6-14 (cross-checked
// against the Table 6-15 inverse mapping). Shared by the executors and the
// LE-mode alignment path in the virtual accessors.
constexpr u32 alignDsisr(u32 i)
{
    u32 d = (ppcbits(i, 6, 10) << 5) | ppcbits(i, 11, 15);
    if (f_primary(i) == 31u)
        d |= (ppcbits(i, 29, 30) << 15) | (ppcbit(i, 25) << 14) |
             (ppcbits(i, 21, 24) << 10);
    else
        d |= (ppcbit(i, 5) << 14) | (ppcbits(i, 1, 4) << 10);
    return d;
}

// ⭐ THE LINE'S BYTES, AS ONE ACCESS INSTEAD OF `len` OF THEM. A cache line
// holds the guest's memory in guest (big-endian) order, so assembling a value
// out of it was written the obvious way — `v = (v << 8) | b[o + i]` — and
// that loop is a runtime loop, because `len` reaches memRead as a parameter.
// Every 32-bit load in the machine therefore cost four dependent
// load-shift-or steps, and every store four shift-and-store steps, on the
// HOTTEST path this emulator has (mem ops are 15.9% of the instructions the
// game executes and a far larger share of its time).
//
// These do the identical thing in one unaligned host access plus a byte swap.
// Identical, not equivalent-in-practice: the caller has already proved the
// access lies inside the 32-byte block ((pa & 31) + len <= 32), the bytes are
// the same bytes in the same order, and a byte swap is exactly what
// "big-endian assembly on a little-endian host" means. The default arm keeps
// the general loop, so a length nobody passes today still behaves.
inline u64 lineLoadBE(const u8* p, u32 len)
{
    switch (len) {
    case 1:
        return p[0];
    case 2: {
        u16 v;
        std::memcpy(&v, p, 2);
        return u16((v >> 8) | (v << 8));
    }
    case 4: {
        u32 v;
        std::memcpy(&v, p, 4);
        return ((v & 0xFFu) << 24) | ((v & 0xFF00u) << 8) |
               ((v >> 8) & 0xFF00u) | (v >> 24);
    }
    case 8: {
        u64 v;
        std::memcpy(&v, p, 8);
        v = ((v & 0x00FF00FF00FF00FFull) << 8) |
            ((v >> 8) & 0x00FF00FF00FF00FFull);
        v = ((v & 0x0000FFFF0000FFFFull) << 16) |
            ((v >> 16) & 0x0000FFFF0000FFFFull);
        return (v << 32) | (v >> 32);
    }
    default: {
        u64 v = 0;
        for (u32 i = 0; i < len; ++i)
            v = (v << 8) | p[i];
        return v;
    }
    }
}
inline void lineStoreBE(u8* p, u32 len, u64 v)
{
    switch (len) {
    case 1:
        p[0] = u8(v);
        return;
    case 2: {
        const u16 t = u16((u16(v) >> 8) | (u16(v) << 8));
        std::memcpy(p, &t, 2);
        return;
    }
    case 4: {
        const u32 s = u32(v);
        const u32 t = ((s & 0xFFu) << 24) | ((s & 0xFF00u) << 8) |
                      ((s >> 8) & 0xFF00u) | (s >> 24);
        std::memcpy(p, &t, 4);
        return;
    }
    case 8: {
        u64 t = ((v & 0x00FF00FF00FF00FFull) << 8) |
                ((v >> 8) & 0x00FF00FF00FF00FFull);
        t = ((t & 0x0000FFFF0000FFFFull) << 16) |
            ((t >> 16) & 0x0000FFFF0000FFFFull);
        t = (t << 32) | (t >> 32);
        std::memcpy(p, &t, 8);
        return;
    }
    default:
        for (u32 i = 0; i < len; ++i)
            p[i] = static_cast<u8>(v >> (8 * (len - 1 - i)));
        return;
    }
}

} // namespace opm
