#pragma once
#include "types.hpp"

namespace opm {

// Which opcode space an instruction's extended opcode lives in.
enum class Xk : u8 {
    PRI,   // primary opcode only (D/I/B/M/SC forms)
    X19,   // opcode 19, 10-bit XO (bits 21-30)
    X31,   // opcode 31, 10-bit XO (bits 21-30); FL_OE rows also register xo+512
    A59,   // opcode 59, 5-bit XO (bits 26-30), A-form single-precision FP
    A63,   // opcode 63, 5-bit XO (bits 26-30), A-form double-precision FP
    X63,   // opcode 63, 10-bit XO (bits 21-30), X-form FP
    VA4,   // opcode 4, 6-bit XO (bits 26-31), VA-form AltiVec
    VX4,   // opcode 4, 11-bit XO (bits 21-31), VX-form AltiVec
    VXR4,  // opcode 4, 10-bit XO (bits 22-31) + Rc at bit 21; registers both halves
};

// Operand pattern — drives the disassembler (and documents the syntax).
enum class Pat : u8 {
    NONE,
    RT_RA_RB,   // add rD,rA,rB
    RT_RA,      // neg rD,rA
    RT_RA_SI,   // addi rD,rA,SIMM
    RA_RS_RB,   // and rA,rS,rB
    RA_RS,      // cntlzw rA,rS
    RA_RS_UI,   // ori rA,rS,UIMM
    RA_RS_SH,   // srawi rA,rS,SH
    RT_D_RA,    // lwz rD,d(rA)
    RS_D_RA,    // stw rS,d(rA)
    FRT_D_RA,   // lfs frD,d(rA)
    FRS_D_RA,   // stfs frS,d(rA)
    RS_RA_RB,   // stwx rS,rA,rB
    FRT_RA_RB,  // lfsx frD,rA,rB
    FRS_RA_RB,  // stfsx frS,rA,rB
    CMP_SI,     // cmpi crfD,L,rA,SIMM
    CMP_UI,     // cmpli crfD,L,rA,UIMM
    CMP_RB,     // cmp crfD,L,rA,rB
    TW_RB,      // tw TO,rA,rB
    TW_SI,      // twi TO,rA,SIMM
    B,          // b target
    BC,         // bc BO,BI,target
    BCLR,       // bclr BO,BI
    BCCTR,      // bcctr BO,BI
    CRB3,       // crand crbD,crbA,crbB
    MCRF,       // mcrf crfD,crfS
    RLWINM,     // rlwinm rA,rS,SH,MB,ME
    RLWNM,      // rlwnm rA,rS,rB,MB,ME
    MTCRF,      // mtcrf CRM,rS
    MFCR,       // mfcr rD
    MFSPR,      // mfspr rD,spr
    MTSPR,      // mtspr spr,rS
    MFTB,       // mftb rD,tbr
    MFMSR,      // mfmsr rD
    MTMSR,      // mtmsr rS
    MTSR,       // mtsr SR,rS
    MFSR,       // mfsr rD,SR
    MTSRIN,     // mtsrin rS,rB
    MFSRIN,     // mfsrin rD,rB
    RB_ONLY,    // tlbie rB
    RA_RB,      // dcbz rA,rB
    LSWI,       // lswi rD,rA,NB
    MCRXR,      // mcrxr crfD
    SC,         // sc
    FP2,        // fmr frD,frB
    FP3,        // fadd frD,frA,frB
    FP3C,       // fmul frD,frA,frC
    FP4,        // fmadd frD,frA,frC,frB
    FCMP,       // fcmpu crfD,frA,frB
    MTFSF,      // mtfsf FM,frB
    MTFSFI,     // mtfsfi crfD,IMM
    MTFSB,      // mtfsb0 crbD
    MCRFS,      // mcrfs crfD,crfS
    MFFS,       // mffs frD
    VX3,        // vaddubm vD,vA,vB
    VX2B,       // vupkhsb vD,vB
    VX_SPLAT,   // vspltb vD,vB,UIMM
    VX_SPLATIS, // vspltisb vD,SIMM
    MFVSCR,     // mfvscr vD
    MTVSCR,     // mtvscr vB
    VA4P,       // vperm vD,vA,vB,vC
    VA_MADD,    // vmaddfp vD,vA,vC,vB
    VSLDOI,     // vsldoi vD,vA,vB,SH
    VD_RA_RB,   // lvx vD,rA,rB
    VS_RA_RB,   // stvx vS,rA,rB
    DST,        // dst rA,rB,STRM
    DSS,        // dss STRM (dssall: no operands)
};

// Flags.
inline constexpr u16 FL_RC      = 1u << 0;  // has record (Rc) variant, mnem + "."
inline constexpr u16 FL_OE      = 1u << 1;  // has overflow-enable variant, mnem + "o"
inline constexpr u16 FL_LK      = 1u << 2;  // branch link bit, mnem + "l"
inline constexpr u16 FL_AA      = 1u << 3;  // branch absolute bit, mnem + "a"
inline constexpr u16 FL_PRIV    = 1u << 4;  // supervisor-only
inline constexpr u16 FL_ILL7400 = 1u << 5;  // architected but NOT implemented on the
                                            // MPC7400 (fsqrt/fsqrts/tlbia): decodes for
                                            // the disassembler, executes as an illegal-
                                            // instruction program exception
inline constexpr u16 FL_TBIT    = 1u << 6;  // dst/dstst: bit 6 selects transient (+"t")
inline constexpr u16 FL_ABIT    = 1u << 7;  // dss: bit 6 selects dssall
inline constexpr u16 FL_VRC     = 1u << 8;  // VXR-form record bit at bit 21

struct Cpu; // fwd
struct InsnDesc;

// Executor signature (bound from P1 onward; null = not yet implemented).
using Handler = void (*)(Cpu&, u32 insn, const InsnDesc&);

struct InsnDesc {
    const char* mnem;
    u8 primary;
    Xk kind;
    u16 xo;
    Pat pat;
    u16 flags;
    Handler fn;
};

// The single source of truth, transcribed from the MPC7400/7410 User's Manual
// (Appendix A), the PEM (MPCFPE32B Appendix A), and the AltiVec PEM.
extern const InsnDesc kIsa[];
extern const size_t kIsaCount;

// Decode a 32-bit instruction word; nullptr = no such instruction (illegal).
const InsnDesc* decode(u32 insn);

enum class Style : u8 { Gnu, Llvm }; // rN/fN/vN prefixes vs bare numbers

// Disassemble one instruction. Returns bytes written (excl. NUL).
// Unknown words render as ".long 0x????????".
int disassemble(u32 insn, u32 pc, char* out, size_t cap, Style style);

// Facility classification (drives FP/AltiVec-unavailable gating).
inline bool isFpInsn(const InsnDesc& d)
{
    switch (d.pat) {
    case Pat::FRT_D_RA: case Pat::FRS_D_RA: case Pat::FRT_RA_RB:
    case Pat::FRS_RA_RB: case Pat::FP2: case Pat::FP3: case Pat::FP3C:
    case Pat::FP4: case Pat::FCMP: case Pat::MTFSF: case Pat::MTFSFI:
    case Pat::MTFSB: case Pat::MCRFS: case Pat::MFFS:
        return true;
    default:
        return false;
    }
}

// AltiVec-unavailable applies to everything touching VRs/VSCR except the
// data-stream hints (verified: dst/dstst/dss run with MSR[VEC]=0, and VRSAVE
// is never gated).
inline bool isVecInsn(const InsnDesc& d)
{
    if (d.flags & (FL_TBIT | FL_ABIT))
        return false;
    switch (d.pat) {
    case Pat::VX3: case Pat::VX2B: case Pat::VX_SPLAT: case Pat::VX_SPLATIS:
    case Pat::MFVSCR: case Pat::MTVSCR: case Pat::VA4P: case Pat::VA_MADD:
    case Pat::VSLDOI: case Pat::VD_RA_RB: case Pat::VS_RA_RB:
        return true;
    default:
        return false;
    }
}

} // namespace opm
