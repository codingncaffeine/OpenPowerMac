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

// The single-target host fast path's off switch (see softfp.cpp). Exists for
// the differential test, which runs every case both ways and demands the
// same {bits, flags}; the machine never touches it.
extern bool gHostFastOff;
extern u64 gFastHits, gFastMiss; // taken vs bailed; g4run reports them

} // namespace opm::sf
