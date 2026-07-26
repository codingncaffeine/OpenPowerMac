#pragma once
#include "types.hpp"

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

} // namespace opm
