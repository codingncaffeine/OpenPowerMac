#pragma once
// Deterministic software floating point for the FPU executors (P4).
//
// Pure integer arithmetic implementing the PEM's execution models directly:
// Appendix D.1 (guard/round/sticky accumulator), D.2 (106-bit fused
// multiply-add), D.4 (integer conversion), D.6/D.7 (single-precision
// load/store conversion), with the exception-condition flows of PEM 3.3.6
// (tininess before rounding, overflow after; OE/UE=1 exponent scaling by
// 192/1536). No host FP, no fenv: results are bit-identical on every
// compiler by construction — this retires the plan's host-FP determinism
// risk instead of mitigating it.
//
// Ops return the DISABLED-exception result (QNaN / ±inf / default overflow
// value); the executor suppresses the register write for enabled invalid /
// zero-divide per PEM 3.3.6.1. Enabled overflow/underflow scaling is done
// here because those cases do deliver a (scaled) result.

#include "opm/types.hpp"

namespace opm::sf {

// Per-op status flags (mapped to FPSCR by the executor).
inline constexpr u32 kOx     = 1u << 0;
inline constexpr u32 kUx     = 1u << 1;
inline constexpr u32 kZx     = 1u << 2;
inline constexpr u32 kXx     = 1u << 3;  // this op's inexact (FI)
inline constexpr u32 kFr     = 1u << 4;  // fraction incremented
inline constexpr u32 kVxsnan = 1u << 5;
inline constexpr u32 kVxisi  = 1u << 6;
inline constexpr u32 kVxidi  = 1u << 7;
inline constexpr u32 kVxzdz  = 1u << 8;
inline constexpr u32 kVximz  = 1u << 9;
inline constexpr u32 kVxcvi  = 1u << 10;
inline constexpr u32 kVxsqrt = 1u << 11;
inline constexpr u32 kVxAny  = kVxsnan | kVxisi | kVxidi | kVxzdz | kVximz |
                               kVxcvi | kVxsqrt;

struct R {
    u64 bits;
    u32 fl;
};

struct Env {
    u32 rn;        // FPSCR[RN]: 0 nearest, 1 zero, 2 +inf, 3 -inf
    bool oe, ue;   // FPSCR[OE]/[UE]: enabled -> scaled results per PEM
    bool ni;       // FPSCR[NI]: denormalized results flush to signed zero
};

enum class Tgt { Dbl, Sgl };

R add(u64 a, u64 b, const Env& e, Tgt t);           // a + b
R sub(u64 a, u64 b, const Env& e, Tgt t);           // a - b
R mul(u64 a, u64 c, const Env& e, Tgt t);           // a * c
R div(u64 a, u64 b, const Env& e, Tgt t);           // a / b
R madd(u64 a, u64 c, u64 b, const Env& e, Tgt t,    // ±((a*c) ± b)
       bool negAdd, bool negResult);
R rsp(u64 b, const Env& e);                         // frsp
R ctiw(u64 b, u32 rn);                              // fctiw[z]: low 32 = int
R res(u64 b, const Env& e);                         // fres (deterministic)
R rsqrte(u64 b, const Env& e);                      // frsqrte (deterministic)

u64 loadSingle(u32 w);   // PEM D.6 (exact format conversion)
u32 storeSingle(u64 f);  // PEM D.7 (truncating conversion)

// The host fast paths' off switches (see softfp.cpp), one bit each. The
// differential test runs every case both ways and demands the same
// {bits, flags}; g4run exposes them as --no-fp-fast[-sgl|-dbl] so a speed
// series can A/B either path inside ONE binary, which is the only kind of
// comparison this project trusts. The machine never sets them.
inline constexpr u32 kFastOffSgl = 1u;
inline constexpr u32 kFastOffDbl = 2u;
extern u32 gHostFastOff;
// Whether the single fast path computes the fused family on this host (it
// needs a hardware fused multiply-add); otherwise the model has those rows.
bool hostHasFma();
extern u64 gFastHits, gFastMiss;   // single-target: taken vs bailed
extern u64 gFastHitsD, gFastMissD; // double-target (incl. frsp); g4run reports both
// Why a fast path declined (the multiply/add/fused ones; divide keeps its
// own count in the misses above). kFastWhyName is the same list in words.
enum : u32 {
    kWhySglOperand,   // an operand is a NaN, an infinity or a denormal
    kWhySglNotSingle, // an operand is not a single's value
    kWhySglOverflow,  // the result overflowed
    kWhySglDenormal,  // the result is a denormal
    kWhySglResidue,   // the addend reaches below the product's 53 bits
    kWhySglToZero,    // the result underflowed to zero
    kWhyDblOperand,   // an operand is outside the admitted band
    kWhyDblAddWindow, // an add whose smaller operand falls off the window
    kWhyDblFmaWindow, // a fused op whose addend or product does
    kWhyDblTiny,      // the result is tiny, a denormal, or an inexact zero
    kWhyDblOverflow,  // the result overflowed
    kWhyNoFma,        // this host has no fused multiply-add
    kFastWhyN
};
extern u64 gFastWhy[kFastWhyN];
inline constexpr const char* kFastWhyName[kFastWhyN] = {
    "single: operand NaN/infinity/denormal",
    "single: operand not a single's value",
    "single: result overflowed",
    "single: result denormal",
    "single: addend below the product's 53 bits",
    "single: result underflowed to zero",
    "double: operand denormal/NaN/infinity/out of band",
    "double: add, smaller operand falls off the window",
    "double: fused, addend or product falls off the window",
    "double: result tiny or denormal",
    "double: result overflowed",
    "no fused multiply-add on this host",
};

} // namespace opm::sf
