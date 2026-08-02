// The JIT — a native x86-64 transcription of Cpu::runSteps' line executor.
// The design and the correctness argument live in _plans/JIT_PLAN.md; the
// header (opm/jit.hpp) carries the short form. What matters while reading
// this file:
//
//   ⭐ NOTHING HERE INVENTS SEMANTICS. Every inlined instruction is a
//   transcription of the bound handler in exec_int.cpp (named in each
//   emitter); everything else calls execRow — the one dispatch tail — and
//   then performs the line executor's own exit tests. Memory goes through
//   shims over readV*/writeV*, so the MMU, the L1/L2 models, the bus, the
//   device clocks and the batch-break contract all run unchanged, and a
//   --jit machine writes the same whole-machine fingerprint as a --no-jit
//   machine at any instruction count.
//
//   ⚠ THE EXIT TESTS ARE THE LINE EXECUTOR'S, NEITHER MORE NOR FEWER. The
//   interpreter does not re-check EE/SE/MMCR0 in the middle of a line, so
//   the blocks do not either — interrupt-delivery and trace latency stay
//   identical to the tick. What IS checked per instruction is exactly what
//   runSteps checks: the pc fell through, the batch did not break, the
//   machine did not halt, and the fetch line under the block is still
//   resident (fl.base — the SMC contract). An early exit to the dispatcher
//   is always safe — the hoisted conditions are batch invariants, so the
//   dispatcher re-derives the same answers — but a skipped check never is,
//   which is why every path out of an instruction runs the full tail.
//
//   ⚠ WIN64 ABI THROUGHOUT. Blocks run with rsp 16-aligned and a 40-byte
//   outgoing area (32 shadow + 8 alignment) reserved by the trampoline, and
//   call C++ shims with RCX/RDX/R8 arguments. Pinned callee-saved registers:
//   r13 = Cpu*, r14 = &stamp, r15 = insnCycles + extraCycles (constant for
//   the length of a batch — only the harness writes those fields, never the
//   guest). r12 stages the effective address across the one shim call of an
//   update-form access. Blocks never touch XMM.

#include "opm/jit.hpp"

#include "opm/bits.hpp"
#include "opm/cpu.hpp"
#include "opm/insn.hpp"
#include "softfp.hpp" // the FP load/store shims use the same D.6/D.7 models

#include <cstring>
#include <vector>

#if defined(_MSC_VER)
#include <intrin.h>
#elif defined(__x86_64__)
#include <x86intrin.h>
#endif

#if defined(_WIN32) && (defined(_M_X64) || defined(__x86_64__))
#define OPM_JIT_HOST 1
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#define OPM_JIT_HOST 0
#endif

namespace opm {
namespace {

#if OPM_JIT_HOST

// ---- what gets inlined -----------------------------------------------------

enum class Lo : u8 {
    None = 0,
    Addi, Addis, Addic, AddicRc, Subfic, Mulli,
    Ori, Oris, Xori, Xoris, AndiRc, AndisRc,
    Cmpi, Cmpli, Cmp, Cmpl,
    Rlwinm, Rlwimi, Rlwnm,
    Add, Addc, Adde, Addze, Addme, Subf, Subfc, Subfe, Subfze, Subfme, Neg,
    Mullw, Mulhw, Mulhwu,
    And, Andc, Or, Orc, Xor, Eqv, Nor, Nand,
    Slw, Srw, Srawi, Cntlzw, Extsb, Extsh,
    B, Bc, Bclr, Bcctr,
    Lwz, Lbz, Lhz, Lha, Lwzx, Lbzx, Lhzx, Lhax,
    Lwzu, Lbzu, Lhzu, Lhau, Lwzux, Lbzux, Lhzux, Lhaux,
    Lwbrx, Lhbrx,
    Stw, Stb, Sth, Stwx, Stbx, Sthx,
    Stwu, Stbu, Sthu, Stwux, Stbux, Sthux,
    Stwbrx, Sthbrx,
    Mfspr, Mtspr,
    // FP memory + the register move: measured 43% of the in-game window ran
    // through execRow, and the census read lfs/stfs/lfd/stfd/fmr at the top
    // alongside the softfp arithmetic. The arithmetic IS softfp and stays a
    // fallback; the loads, stores and moves are ordinary memory/register
    // work and inline like their integer siblings (behind an MSR[FP] head
    // test whose cold side is the ordinary execRow path).
    Lfs, Lfsu, Lfsx, Lfsux, Lfd, Lfdu, Lfdx, Lfdux,
    Stfs, Stfsu, Stfsx, Stfsux, Stfd, Stfdu, Stfdx, Stfdux,
    Fmr,
};

// Row -> lowering, keyed by the ISA table's own mnemonics so the mapping can
// never drift from decode. Rows not named here fall back to execRow.
const u8* loTable()
{
    static const std::vector<u8> t = [] {
        std::vector<u8> v(kIsaCount, static_cast<u8>(Lo::None));
        auto set = [&](const char* mn, Lo lo) {
            for (size_t i = 0; i < kIsaCount; ++i)
                if (std::strcmp(kIsa[i].mnem, mn) == 0) {
                    v[i] = static_cast<u8>(lo);
                    return;
                }
        };
        set("addi", Lo::Addi);     set("addis", Lo::Addis);
        set("addic", Lo::Addic);   set("addic.", Lo::AddicRc);
        set("subfic", Lo::Subfic); set("mulli", Lo::Mulli);
        set("ori", Lo::Ori);       set("oris", Lo::Oris);
        set("xori", Lo::Xori);     set("xoris", Lo::Xoris);
        set("andi.", Lo::AndiRc);  set("andis.", Lo::AndisRc);
        set("cmpi", Lo::Cmpi);     set("cmpli", Lo::Cmpli);
        set("cmp", Lo::Cmp);       set("cmpl", Lo::Cmpl);
        set("rlwinm", Lo::Rlwinm); set("rlwimi", Lo::Rlwimi);
        set("rlwnm", Lo::Rlwnm);
        set("add", Lo::Add);       set("addc", Lo::Addc);
        set("adde", Lo::Adde);     set("addze", Lo::Addze);
        set("addme", Lo::Addme);   set("subf", Lo::Subf);
        set("subfc", Lo::Subfc);   set("subfe", Lo::Subfe);
        set("subfze", Lo::Subfze); set("subfme", Lo::Subfme);
        set("neg", Lo::Neg);       set("mullw", Lo::Mullw);
        set("mulhw", Lo::Mulhw);   set("mulhwu", Lo::Mulhwu);
        set("and", Lo::And);       set("andc", Lo::Andc);
        set("or", Lo::Or);         set("orc", Lo::Orc);
        set("xor", Lo::Xor);       set("eqv", Lo::Eqv);
        set("nor", Lo::Nor);       set("nand", Lo::Nand);
        set("slw", Lo::Slw);       set("srw", Lo::Srw);
        set("srawi", Lo::Srawi);   set("cntlzw", Lo::Cntlzw);
        set("extsb", Lo::Extsb);   set("extsh", Lo::Extsh);
        set("b", Lo::B);           set("bc", Lo::Bc);
        set("bclr", Lo::Bclr);     set("bcctr", Lo::Bcctr);
        set("lwz", Lo::Lwz);       set("lbz", Lo::Lbz);
        set("lhz", Lo::Lhz);       set("lha", Lo::Lha);
        set("lwzx", Lo::Lwzx);     set("lbzx", Lo::Lbzx);
        set("lhzx", Lo::Lhzx);     set("lhax", Lo::Lhax);
        set("lwzu", Lo::Lwzu);     set("lbzu", Lo::Lbzu);
        set("lhzu", Lo::Lhzu);     set("lhau", Lo::Lhau);
        set("lwzux", Lo::Lwzux);   set("lbzux", Lo::Lbzux);
        set("lhzux", Lo::Lhzux);   set("lhaux", Lo::Lhaux);
        set("lwbrx", Lo::Lwbrx);   set("lhbrx", Lo::Lhbrx);
        set("stw", Lo::Stw);       set("stb", Lo::Stb);
        set("sth", Lo::Sth);       set("stwx", Lo::Stwx);
        set("stbx", Lo::Stbx);     set("sthx", Lo::Sthx);
        set("stwu", Lo::Stwu);     set("stbu", Lo::Stbu);
        set("sthu", Lo::Sthu);     set("stwux", Lo::Stwux);
        set("stbux", Lo::Stbux);   set("sthux", Lo::Sthux);
        set("stwbrx", Lo::Stwbrx); set("sthbrx", Lo::Sthbrx);
        set("mfspr", Lo::Mfspr);   set("mtspr", Lo::Mtspr);
        set("lfs", Lo::Lfs);       set("lfsu", Lo::Lfsu);
        set("lfsx", Lo::Lfsx);     set("lfsux", Lo::Lfsux);
        set("lfd", Lo::Lfd);       set("lfdu", Lo::Lfdu);
        set("lfdx", Lo::Lfdx);     set("lfdux", Lo::Lfdux);
        set("stfs", Lo::Stfs);     set("stfsu", Lo::Stfsu);
        set("stfsx", Lo::Stfsx);   set("stfsux", Lo::Stfsux);
        set("stfd", Lo::Stfd);     set("stfdu", Lo::Stfdu);
        set("stfdx", Lo::Stfdx);   set("stfdux", Lo::Stfdux);
        set("fmr", Lo::Fmr);
        return v;
    }();
    return t.data();
}

// ---- the C++ the emitted code calls ----------------------------------------
//
// Loads return the zero-extended value, or -1 on a fault (a 32-bit value
// zero-extends below 2^32, so -1 is unreachable as data). Stores return 0 or
// -1. On -1 the exception has already been raised by readV/writeV — pc is at
// the vector — and the emitted code takes the fault stub: charge, count,
// exit. That is the interpreter's own order: execFast charges after a
// faulting handler too, and the destination register is left unwritten.

i64 shimRead8(Cpu* c, u32 ea)  { ++c->jit->memOps; u32 v; return c->readV8(ea, v)  ? static_cast<i64>(v) : -1; }
i64 shimRead16(Cpu* c, u32 ea) { ++c->jit->memOps; u32 v; return c->readV16(ea, v) ? static_cast<i64>(v) : -1; }
i64 shimRead32(Cpu* c, u32 ea) { ++c->jit->memOps; u32 v; return c->readV32(ea, v) ? static_cast<i64>(v) : -1; }
i64 shimWrite8(Cpu* c, u32 ea, u32 v)  { ++c->jit->memOps; return c->writeV8(ea, v)  ? 0 : -1; }
i64 shimWrite16(Cpu* c, u32 ea, u32 v) { ++c->jit->memOps; return c->writeV16(ea, v) ? 0 : -1; }
i64 shimWrite32(Cpu* c, u32 ea, u32 v) { ++c->jit->memOps; return c->writeV32(ea, v) ? 0 : -1; }
void shimExecRow(Cpu* c, u32 insn, u32 row)
{
    ++c->jit->fallbacks; // the census: which rows still cost an execRow call
    ++c->jit->fbByRow[row < 1024u ? row : 1023u];
    c->execRow(insn, row);
}

// FP memory shims — transcriptions of exec_int.cpp's FLOAD/FSTORE bodies:
// word-alignment fault first (UM 4.6.6, DSISR from the image), then readV/
// writeV, then the FPR write with the PEM D.6/D.7 single<->double models.
// The rA update of the u-forms stays in emitted code, off the staged EA.
i64 shimAlignF(Cpu* c, u32 ea, u32 insn)
{
    c->st.dar = ea;
    c->st.dsisr = alignDsisr(insn);
    c->raiseExc(Exc::Alignment, c->st.pc - 4, 0);
    return -1;
}
i64 shimLfs(Cpu* c, u32 ea, u32 insn)
{
    if (ea & 3u)
        return shimAlignF(c, ea, insn);
    u64 v;
    if (!c->readV(ea, 4, v))
        return -1;
    c->st.fpr[f_rt(insn)] = sf::loadSingle(static_cast<u32>(v));
    return 0;
}
i64 shimLfd(Cpu* c, u32 ea, u32 insn)
{
    if (ea & 3u)
        return shimAlignF(c, ea, insn);
    u64 v;
    if (!c->readV(ea, 8, v))
        return -1;
    c->st.fpr[f_rt(insn)] = v;
    return 0;
}
i64 shimStfs(Cpu* c, u32 ea, u32 insn)
{
    if (ea & 3u)
        return shimAlignF(c, ea, insn);
    return c->writeV(ea, 4, sf::storeSingle(c->st.fpr[f_rt(insn)])) ? 0 : -1;
}
i64 shimStfd(Cpu* c, u32 ea, u32 insn)
{
    if (ea & 3u)
        return shimAlignF(c, ea, insn);
    return c->writeV(ea, 8, c->st.fpr[f_rt(insn)]) ? 0 : -1;
}

// ---- byte emitter ----------------------------------------------------------

enum : u32 { RAX = 0, RCX = 1, RDX = 2, R8 = 8, R9 = 9, R10 = 10, R12 = 12 };
enum : u8 { CC_E = 4, CC_NE = 5, CC_Z = 4, CC_NZ = 5 };

struct Emit {
    u8* base;
    size_t cap;
    size_t at;
    bool ok = true;

    void b(u8 v)
    {
        if (at < cap)
            base[at++] = v;
        else
            ok = false;
    }
    void bytes(std::initializer_list<u8> l)
    {
        for (u8 v : l)
            b(v);
    }
    void i32le(u32 v) { b(u8(v)); b(u8(v >> 8)); b(u8(v >> 16)); b(u8(v >> 24)); }
    void i64le(u64 v) { i32le(u32(v)); i32le(u32(v >> 32)); }

    // op reg32, [r13+disp] (or /digit in `reg`). r13 as base is rm=101 with
    // mod=01/10 — no RIP quirk (mod=00 only), no SIB (rm=100 only). disp8
    // when it fits: the GPR file sits in the first 128 bytes of Cpu, so the
    // bulk of every block's bytes shrink by three — and at a multi-megabyte
    // code working set the I-cache footprint is a first-order cost.
    void disp8or32(u32 reg, i32 disp)
    {
        if (disp >= -128 && disp <= 127) {
            b(u8(0x40 | ((reg & 7) << 3) | 5));
            b(u8(i8(disp)));
        } else {
            b(u8(0x80 | ((reg & 7) << 3) | 5));
            i32le(u32(disp));
        }
    }
    void mem13(u8 op, u32 reg, i32 disp)
    {
        b(u8(0x40 | ((reg & 8) ? 4 : 0) | 1)); // REX.B for r13 (+R if reg>=8)
        b(op);
        disp8or32(reg, disp);
    }
    void mem13_0F(u8 op, u32 reg, i32 disp)
    {
        b(u8(0x40 | ((reg & 8) ? 4 : 0) | 1));
        b(0x0F);
        b(op);
        disp8or32(reg, disp);
    }

    void loadR(u32 reg, i32 disp) { mem13(0x8B, reg, disp); }   // mov r32,[m]
    void storeR(i32 disp, u32 reg) { mem13(0x89, reg, disp); }  // mov [m],r32
    void storeI(i32 disp, u32 imm)                              // mov [m],imm
    {
        mem13(0xC7, 0, disp);
        i32le(imm);
    }

    void addEaxI(u32 v) { b(0x05); i32le(v); }
    void andEaxI(u32 v) { b(0x25); i32le(v); }
    void orEaxI(u32 v)  { b(0x0D); i32le(v); }
    void xorEaxI(u32 v) { b(0x35); i32le(v); }
    void cmpEaxI(u32 v) { b(0x3D); i32le(v); }
    void movEaxI(u32 v) { b(0xB8); i32le(v); }
    void movEdxI(u32 v) { b(0xBA); i32le(v); }

    size_t j8(u8 cc)
    {
        b(u8(0x70 | cc));
        b(0);
        return at - 1;
    }
    size_t jmp8()
    {
        b(0xEB);
        b(0);
        return at - 1;
    }
    // rel32 forward jumps for paths too long for rel8 (the FP gate skips a
    // whole emitted execRow arm).
    size_t j32(u8 cc)
    {
        b(0x0F);
        b(u8(0x80 | cc));
        i32le(0);
        return at - 4;
    }
    size_t jmp32f()
    {
        b(0xE9);
        i32le(0);
        return at - 4;
    }
    void patch32(size_t pos)
    {
        const i64 rel = i64(at) - i64(pos + 4);
        if (pos + 4 <= cap) {
            base[pos] = u8(rel);
            base[pos + 1] = u8(rel >> 8);
            base[pos + 2] = u8(rel >> 16);
            base[pos + 3] = u8(rel >> 24);
        }
    }
    // 64-bit op reg, [r13+disp] — the FPR file lives past disp8 range.
    void mem13W(u8 op, u32 reg, i32 disp)
    {
        b(u8(0x48 | ((reg & 8) ? 4 : 0) | 1)); // REX.W + B
        b(op);
        disp8or32(reg, disp);
    }
    void patch8(size_t pos)
    {
        const i64 rel = i64(at) - i64(pos + 1);
        if (rel < -128 || rel > 127) {
            ok = false;
            return;
        }
        if (pos < cap)
            base[pos] = u8(i8(rel));
    }

    void jmpTo(u32 dst)
    {
        b(0xE9);
        i32le(u32(i64(dst) - (i64(at) + 4)));
    }
    void jccTo(u8 cc, u32 dst)
    {
        b(0x0F);
        b(u8(0x80 | cc));
        i32le(u32(i64(dst) - (i64(at) + 4)));
    }

    void callAbs(const void* fn)
    {
        b(0x48);
        b(0xB8);
        i64le(reinterpret_cast<u64>(fn)); // mov rax, imm64
        b(0xFF);
        b(0xD0);                          // call rax
    }
};

// ---- the compiler ----------------------------------------------------------

struct Compiler {
    Emit e;
    const Cpu& c;
    const JitCache::Offs& o;
    u32 epi;
    u32 paBase;
    u32 vaBase;
    i32 flBase; // r13-relative disp of fetchLine[slot].base

    i32 gpr(u32 r) const { return o.gpr0 + i32(r * 4u); }
    i32 fpr(u32 r) const { return o.fpr0 + i32(r * 8u); }

    // -- shared fragments ----------------------------------------------------

    // charge(insnCycles + extraCycles) + ++stamp — the execFast tail, in its
    // direct form: two read-modify-writes. Used on the call-dominated paths
    // (memory ops, fallbacks), where two RMWs are noise.
    void chargeInc()
    {
        e.bytes({0x4D, 0x01, 0xBD});   // add [r13+pend], r15
        e.i32le(u32(o.pend));
        e.bytes({0x49, 0xFF, 0x06});   // inc qword [r14]
    }
    // ⭐ THE BATCHED FORM, for the pure-register instructions. Three
    // serialized memory RMWs per instruction (pc, pendCycles, stamp) cost
    // more than the instructions they book-kept — measured: the JIT beat the
    // interpreter by 2% with them and the whole point without them. Between
    // observation points nothing can read st.pc, the stamp or pendCycles —
    // the only readers are the shims (device clocks, event logs, exception
    // srr0) and the dispatcher — so instructions completed since the last
    // flush ride in rbx (callee-saved, zeroed by the trampoline, preserved
    // by every shim), and a flush settles all three in one go at exactly the
    // points where the interpreter's values are observable: before every
    // shim call, and on every exit. The flushed values are bit-identical to
    // per-instruction accounting — under batching the interpreter itself
    // only accumulates pendCycles and defers the clock to the batch edge.
    void bump() { e.bytes({0x48, 0xFF, 0xC3}); } // inc rbx
    void flushNoPc()
    {
        e.bytes({0x49, 0x01, 0x1E});         // add [r14], rbx
        e.bytes({0x48, 0x89, 0xD8});         // mov rax, rbx
        e.bytes({0x49, 0x0F, 0xAF, 0xC7});   // imul rax, r15
        e.bytes({0x49, 0x01, 0x85});         // add [r13+pend], rax
        e.i32le(u32(o.pend));
        e.bytes({0x31, 0xDB});               // xor ebx, ebx
    }
    void flushPc(u32 pcImm)
    {
        e.storeI(o.pc, pcImm);
        flushNoPc();
    }
    void flCheck() // the block under our feet is still the compiled block
    {
        e.mem13(0x81, 7, flBase);      // cmp dword [r13+d], imm32
        e.i32le(paBase);
        e.jccTo(CC_NE, epi);
    }
    void byteCheck(i32 disp) // exit if a bool member went true
    {
        e.mem13(0x80, 7, disp);        // cmp byte [r13+d], imm8
        e.b(0);
        e.jccTo(CC_NE, epi);
    }
    void pcCheck(u32 nextVa) // exit unless the pc fell through
    {
        e.loadR(RAX, o.pc);
        e.cmpEaxI(nextVa);
        e.jccTo(CC_NE, epi);
    }

    // CR field from the FLAGS of a just-executed cmp/test: call zero3()
    // BEFORE the compare (xor clobbers flags), crInsert after. Mirrors
    // setCr0/setCrField plus the SO bit from XER, bit for bit.
    void zero3()
    {
        e.bytes({0x45, 0x31, 0xC0});   // xor r8d, r8d
        e.bytes({0x45, 0x31, 0xC9});   // xor r9d, r9d
        e.bytes({0x45, 0x31, 0xD2});   // xor r10d, r10d
    }
    void crInsert(u32 field, bool sgn)
    {
        e.bytes({0x41, 0x0F, sgn ? u8(0x9C) : u8(0x92), 0xC0}); // setl/setb r8b
        e.bytes({0x41, 0x0F, sgn ? u8(0x9F) : u8(0x97), 0xC1}); // setg/seta r9b
        e.bytes({0x41, 0x0F, 0x94, 0xC2});                      // sete  r10b
        e.bytes({0x41, 0xC1, 0xE0, 0x03}); // shl r8d, 3  (LT -> 8)
        e.bytes({0x41, 0xC1, 0xE1, 0x02}); // shl r9d, 2  (GT -> 4)
        e.bytes({0x45, 0x01, 0xC8});       // add r8d, r9d
        e.bytes({0x45, 0x01, 0xD2});       // add r10d, r10d (EQ -> 2)
        e.bytes({0x45, 0x01, 0xD0});       // add r8d, r10d
        e.loadR(RDX, o.xer);
        e.bytes({0xC1, 0xEA, 0x1F});       // shr edx, 31 (SO)
        e.bytes({0x41, 0x01, 0xD0});       // add r8d, edx
        e.loadR(RAX, o.cr);
        const u32 sh = (7u - field) * 4u;
        e.andEaxI(~(0xFu << sh));
        if (sh)
            e.bytes({0x41, 0xC1, 0xE0, u8(sh)}); // shl r8d, sh
        e.bytes({0x44, 0x09, 0xC0});       // or eax, r8d
        e.storeR(o.cr, RAX);
    }
    void setCr0FromEax() // result still in eax; crInsert only clobbers it
    {                    // after the setcc trio captured the flags
        zero3();
        e.bytes({0x85, 0xC0}); // test eax, eax
        crInsert(0, true);
    }
    void caFromEdx() // XER[CA] <- edx (0/1); mirrors setCa
    {
        e.loadR(R9, o.xer);          // mov r9d, [r13+xer]
        e.bytes({0x41, 0x81, 0xE1}); // and r9d, ~CA
        e.i32le(~0x20000000u);
        e.bytes({0xC1, 0xE2, 0x1D}); // shl edx, 29
        e.bytes({0x41, 0x09, 0xD1}); // or r9d, edx
        e.storeR(o.xer, R9);         // mov [r13+xer], r9d
    }

    // -- per-instruction tails ----------------------------------------------

    void aluTail(bool rc)
    {
        if (rc)
            setCr0FromEax();
        bump();
    }

    // -- fallback: one call into the one dispatch tail -----------------------
    //
    // Entering with st.pc == CIA is the block invariant (every inlined
    // instruction stores its own next pc; the dispatcher enters only when
    // st.pc is the entry word's VA), and execRow does its own pc increment,
    // charge, perfmon and trace work — so the only additions here are the
    // line executor's own post-instruction tests, in its own order of
    // effect: count the instruction, then exit on anything that ends the
    // run of assumptions.
    void fallback(u32 insn, u32 row, u32 nextVa)
    {
        flushPc(nextVa - 4); // execRow's contract: st.pc == CIA at entry
        e.bytes({0x4C, 0x89, 0xE9}); // mov rcx, r13
        e.movEdxI(insn);
        e.bytes({0x41, 0xB8});       // mov r8d, imm32
        e.i32le(row);
        e.callAbs(reinterpret_cast<const void*>(&shimExecRow));
        e.bytes({0x49, 0xFF, 0x06}); // inc qword [r14] (execRow charged)
        byteCheck(o.halted);
        byteCheck(o.batchBreak);
        pcCheck(nextVa);
        flCheck();
    }

    // -- loads / stores ------------------------------------------------------
    //
    // kind: 0=byte 1=half 2=word; xform selects the rb index form; sext /
    // brx are per-op decorations applied to the shim's zero-extended value.
    // The fault stub replicates execFast's order on a faulting handler:
    // no destination write, charge anyway, count, exit (pc is at the
    // vector). After a SUCCESSFUL access the batch may have been broken (a
    // device access ends the batch from the inside) and the access may have
    // dropped our own fetch line (a store into the line, or a castout), so
    // both are tested — the same two things the line executor tests.

    void emitEaD(u32 insn, bool update)
    {
        const u32 ra = f_ra(insn);
        const u32 simm = u32(sext16(f_d(insn)));
        if (update) { // ea staged in r12d, survives the call
            e.bytes({0x45, 0x8B, 0xA5}); // mov r12d, [r13+gpr ra]
            e.i32le(u32(gpr(ra)));
            e.bytes({0x41, 0x81, 0xC4}); // add r12d, imm32
            e.i32le(simm);
            e.bytes({0x44, 0x89, 0xE2}); // mov edx, r12d
            return;
        }
        if (ra) {
            e.loadR(RDX, gpr(ra));
            e.bytes({0x81, 0xC2}); // add edx, imm32
            e.i32le(simm);
        } else {
            e.movEdxI(simm);
        }
    }
    void emitEaX(u32 insn, bool update)
    {
        const u32 ra = f_ra(insn), rb = f_rb(insn);
        if (update) {
            e.bytes({0x45, 0x8B, 0xA5}); // mov r12d, [r13+gpr ra]
            e.i32le(u32(gpr(ra)));
            e.bytes({0x45, 0x03, 0xA5}); // add r12d, [r13+gpr rb]
            e.i32le(u32(gpr(rb)));
            e.bytes({0x44, 0x89, 0xE2}); // mov edx, r12d
            return;
        }
        if (ra) {
            e.loadR(RDX, gpr(ra));
            e.bytes({0x41, 0x03, 0x95}); // add edx, [r13+gpr rb]
            e.i32le(u32(gpr(rb)));
        } else {
            e.loadR(RDX, gpr(rb));
        }
    }

    void load(u32 insn, u32 nextVa, u32 kind, bool xform, bool update,
              bool sext, bool brx)
    {
        flushPc(nextVa); // handlers see NIA; readV can observe all three
        e.storeI(o.curInsn, insn); // a faulting access builds DSISR from it
        if (xform)
            emitEaX(insn, update);
        else
            emitEaD(insn, update);
        e.bytes({0x4C, 0x89, 0xE9}); // mov rcx, r13
        const void* shim = kind == 0 ? reinterpret_cast<const void*>(&shimRead8)
                         : kind == 1 ? reinterpret_cast<const void*>(&shimRead16)
                                     : reinterpret_cast<const void*>(&shimRead32);
        e.callAbs(shim);
        e.bytes({0x48, 0x83, 0xF8, 0xFF}); // cmp rax, -1
        const size_t okJ = e.j8(CC_NE);
        chargeInc();                       // fault: charge + count, no write
        e.jmpTo(epi);
        e.patch8(okJ);
        if (sext)
            e.b(0x98); // cwde — lha's sext16 (only the halfword forms set it)
        if (brx) {
            if (kind == 2)
                e.bytes({0x0F, 0xC8}); // bswap eax
            else
                e.bytes({0x66, 0xC1, 0xC8, 0x08}); // ror ax, 8
        }
        e.storeR(gpr(f_rt(insn)), RAX);
        if (update)
            e.bytes({0x45, 0x89, 0xA5}), e.i32le(u32(gpr(f_ra(insn))));
        chargeInc();
        byteCheck(o.batchBreak);
        flCheck();
    }

    void store(u32 insn, u32 nextVa, u32 kind, bool xform, bool update,
               bool brx)
    {
        flushPc(nextVa); // handlers see NIA; writeV can observe all three
        e.storeI(o.curInsn, insn);
        if (xform)
            emitEaX(insn, update);
        else
            emitEaD(insn, update);
        e.bytes({0x45, 0x8B, 0x85}); // mov r8d, [r13+gpr rs]
        e.i32le(u32(gpr(f_rt(insn))));
        if (brx) {
            if (kind == 2)
                e.bytes({0x41, 0x0F, 0xC8}); // bswap r8d
            else
                e.bytes({0x66, 0x41, 0xC1, 0xC8, 0x08}); // ror r8w, 8
        }
        e.bytes({0x4C, 0x89, 0xE9}); // mov rcx, r13
        const void* shim = kind == 0 ? reinterpret_cast<const void*>(&shimWrite8)
                         : kind == 1 ? reinterpret_cast<const void*>(&shimWrite16)
                                     : reinterpret_cast<const void*>(&shimWrite32);
        e.callAbs(shim);
        e.bytes({0x48, 0x83, 0xF8, 0xFF}); // cmp rax, -1
        const size_t okJ = e.j8(CC_NE);
        chargeInc();                       // fault: charge + count, no rA update
        e.jmpTo(epi);
        e.patch8(okJ);
        if (update)
            e.bytes({0x45, 0x89, 0xA5}), e.i32le(u32(gpr(f_ra(insn))));
        chargeInc();
        byteCheck(o.batchBreak);
        flCheck();
    }

    // -- FP loads / stores / moves -------------------------------------------
    //
    // Shape: MSR[FP] head test — the warm side runs the shim exactly like an
    // integer memory op; the cold side (FP unavailable) is a full emitted
    // execRow arm, which raises FpUnavailable the interpreter's own way and
    // keeps making progress (an exit back to the dispatcher would re-enter
    // this same block forever). Both arms rejoin at the next segment.
    void fpMem(u32 insn, u32 row, u32 nextVa, bool isStore, bool single,
               bool xform, bool update)
    {
        e.loadR(RAX, o.msr);
        e.b(0xA9); // test eax, imm32
        e.i32le(0x2000u); // msr::FP
        const size_t fbJ = e.j32(CC_Z);
        flushPc(nextVa); // the shim can fault: pc/stamp/cycles must be true
        if (xform)
            emitEaX(insn, update);
        else
            emitEaD(insn, update);
        e.bytes({0x41, 0xB8}); // mov r8d, insn (the shim decodes frt/DSISR)
        e.i32le(insn);
        e.bytes({0x4C, 0x89, 0xE9}); // mov rcx, r13
        const void* shim =
            isStore ? (single ? reinterpret_cast<const void*>(&shimStfs)
                              : reinterpret_cast<const void*>(&shimStfd))
                    : (single ? reinterpret_cast<const void*>(&shimLfs)
                              : reinterpret_cast<const void*>(&shimLfd));
        e.callAbs(shim);
        e.bytes({0x48, 0x83, 0xF8, 0xFF}); // cmp rax, -1
        const size_t okJ = e.j8(CC_NE);
        chargeInc(); // fault: charge + count, no rA update
        e.jmpTo(epi);
        e.patch8(okJ);
        if (update) {
            e.bytes({0x45, 0x89, 0xA5}); // mov [gpr ra], r12d
            e.i32le(u32(gpr(f_ra(insn))));
        }
        chargeInc();
        byteCheck(o.batchBreak);
        flCheck();
        const size_t joinJ = e.jmp32f();
        e.patch32(fbJ); // FP unavailable: the interpreter's own path
        fallback(insn, row, nextVa);
        e.patch32(joinJ);
    }

    // -- branches ------------------------------------------------------------
    //
    // bcTaken transcribed: the CTR decrement happens whether or not the
    // branch is taken (bo bit 2 clear), LR is written whether or not the
    // branch is taken (LK set), and the verdict composes ctrOk AND condOk.
    // bo/bi are immediates, so the composition specializes at compile time.
    enum { kAlways = 0, kTestR8, kTestCl, kTestAnd };

    int condPrep(u32 bo, u32 bi, bool usesCtr)
    {
        const bool ctrArm = usesCtr && !(bo & 4u);
        const bool condArm = !(bo & 16u);
        if (ctrArm) {
            e.bytes({0x45, 0x31, 0xC0}); // xor r8d, r8d
            e.mem13(0x83, 5, o.ctr);     // sub dword [r13+ctr], 1
            e.b(1);
            e.bytes({0x41, 0x0F, 0x95, 0xC0}); // setnz r8b (ctr != 0)
            if (bo & 2u)
                e.bytes({0x41, 0x80, 0xF0, 0x01}); // xor r8b, 1
        }
        if (condArm) {
            e.bytes({0x31, 0xC9}); // xor ecx, ecx
            e.loadR(RDX, o.cr);
            e.bytes({0x0F, 0xBA, 0xE2, u8(31u - bi)}); // bt edx, 31-bi
            e.bytes({0x0F, 0x92, 0xC1});               // setc cl
            if (!(bo & 8u))
                e.bytes({0x80, 0xF1, 0x01}); // xor cl, 1
        }
        if (ctrArm && condArm)
            return kTestAnd;
        if (ctrArm)
            return kTestR8;
        if (condArm)
            return kTestCl;
        return kAlways;
    }
    size_t condJump(int v) // emit the final test; SIZE_MAX = always taken
    {
        if (v == kAlways)
            return size_t(-1);
        if (v == kTestAnd) {
            e.bytes({0x41, 0x20, 0xC8}); // and r8b, cl
            e.bytes({0x45, 0x84, 0xC0}); // test r8b, r8b
        } else if (v == kTestR8) {
            e.bytes({0x45, 0x84, 0xC0});
        } else {
            e.bytes({0x84, 0xC9}); // test cl, cl
        }
        return e.j8(CC_Z);
    }

    // -- the driver ----------------------------------------------------------

    void word(u32 k, u32 insn, u32 row);
};

void Compiler::word(u32 k, u32 insn, u32 row)
{
    const u32 cia = vaBase + 4u * k;
    const u32 nextVa = cia + 4u;

    // Anything the line executor would hand to execRow's slow paths goes to
    // execRow here too: unknown words (kNoRow), gated rows (privileged, FP,
    // AltiVec, 7400-illegal), unbound handlers, and OE forms of the OE-capable
    // arithmetic (setOv is rare enough that transcription buys nothing).
    if (row >= kIsaCount) {
        fallback(insn, row, nextVa);
        return;
    }
    const InsnDesc& d = kIsa[row];
    // FP rows carry exactly the FP-availability gate, and the FP emitters
    // handle that themselves with the MSR head test — so they may proceed.
    // Anything else gated (privileged, 7400-illegal, AltiVec) is execRow's
    // business, as is a row with no bound handler.
    const bool fpGateOnly = isFpInsn(d) && !(d.flags & (FL_PRIV | FL_ILL7400));
    if ((c.dispPre[row] && !fpGateOnly) || !c.dispFn[row]) {
        fallback(insn, row, nextVa);
        return;
    }
    if ((d.flags & FL_OE) && f_oebit(insn)) {
        fallback(insn, row, nextVa);
        return;
    }
    const Lo lo = static_cast<Lo>(loTable()[row]);
    const bool rc = (d.flags & FL_RC) && f_rcbit(insn);
    const u32 rt = f_rt(insn), ra = f_ra(insn), rb = f_rb(insn);

    switch (lo) {
    default:
    case Lo::None:
        fallback(insn, row, nextVa);
        return;

    // ---- D-form arithmetic (h_addi family) --------------------------------
    case Lo::Addi:
    case Lo::Addis: {
        const u32 imm = lo == Lo::Addi ? u32(sext16(f_d(insn)))
                                       : (f_d(insn) << 16);
        if (ra) {
            e.loadR(RAX, gpr(ra));
            e.addEaxI(imm);
            e.storeR(gpr(rt), RAX);
        } else {
            e.storeI(gpr(rt), imm);
        }
        bump();
        return;
    }
    case Lo::Addic:
    case Lo::AddicRc: {
        e.bytes({0x31, 0xD2}); // xor edx, edx
        e.loadR(RAX, gpr(ra));
        e.addEaxI(u32(sext16(f_d(insn))));
        e.bytes({0x0F, 0x92, 0xC2}); // setc dl
        e.storeR(gpr(rt), RAX);
        caFromEdx();
        aluTail(lo == Lo::AddicRc);
        return;
    }
    case Lo::Subfic: {
        e.bytes({0x31, 0xD2}); // xor edx, edx
        e.movEaxI(u32(sext16(f_d(insn))));
        e.mem13(0x2B, RAX, gpr(ra)); // sub eax, [ra]
        e.bytes({0x0F, 0x93, 0xC2}); // setnc dl (CA = no borrow)
        e.storeR(gpr(rt), RAX);
        caFromEdx();
        bump();
        return;
    }
    case Lo::Mulli:
        e.mem13(0x69, RAX, gpr(ra)); // imul eax, [ra], imm32
        e.i32le(u32(sext16(f_d(insn))));
        e.storeR(gpr(rt), RAX);
        bump();
        return;

    // ---- logical immediates (h_ori family; source is rS = rt field) -------
    case Lo::Ori:
    case Lo::Oris:
    case Lo::Xori:
    case Lo::Xoris:
    case Lo::AndiRc:
    case Lo::AndisRc: {
        const bool hi = lo == Lo::Oris || lo == Lo::Xoris || lo == Lo::AndisRc;
        const u32 imm = hi ? (f_d(insn) << 16) : f_d(insn);
        e.loadR(RAX, gpr(rt));
        if (lo == Lo::Ori || lo == Lo::Oris)
            e.orEaxI(imm);
        else if (lo == Lo::Xori || lo == Lo::Xoris)
            e.xorEaxI(imm);
        else
            e.andEaxI(imm);
        e.storeR(gpr(ra), RAX);
        aluTail(lo == Lo::AndiRc || lo == Lo::AndisRc); // andi./andis. always
        return;
    }

    // ---- compares (cmpS/cmpU; the register reads are direct, not gpr0) ----
    case Lo::Cmpi:
    case Lo::Cmpli: {
        e.loadR(RAX, gpr(ra));
        zero3();
        e.cmpEaxI(lo == Lo::Cmpi ? u32(sext16(f_d(insn))) : f_d(insn));
        crInsert(f_crfd(insn), lo == Lo::Cmpi);
        bump();
        return;
    }
    case Lo::Cmp:
    case Lo::Cmpl: {
        e.loadR(RAX, gpr(ra));
        e.loadR(RDX, gpr(rb));
        zero3();
        e.bytes({0x39, 0xD0}); // cmp eax, edx
        crInsert(f_crfd(insn), lo == Lo::Cmp);
        bump();
        return;
    }

    // ---- rotates (h_rlwinm family; masks and shifts are immediates) -------
    case Lo::Rlwinm: {
        e.loadR(RAX, gpr(rt));
        if (f_sh(insn))
            e.bytes({0xC1, 0xC0, u8(f_sh(insn))}); // rol eax, sh
        e.andEaxI(ppcmask(f_mb(insn), f_me(insn)));
        e.storeR(gpr(ra), RAX);
        aluTail(rc);
        return;
    }
    case Lo::Rlwimi: {
        const u32 m = ppcmask(f_mb(insn), f_me(insn));
        e.loadR(RAX, gpr(rt));
        if (f_sh(insn))
            e.bytes({0xC1, 0xC0, u8(f_sh(insn))});
        e.andEaxI(m);
        e.loadR(RDX, gpr(ra));
        e.bytes({0x81, 0xE2}); // and edx, ~m
        e.i32le(~m);
        e.bytes({0x09, 0xD0}); // or eax, edx
        e.storeR(gpr(ra), RAX);
        aluTail(rc);
        return;
    }
    case Lo::Rlwnm: {
        e.loadR(RCX, gpr(rb));
        e.bytes({0x83, 0xE1, 0x1F}); // and ecx, 31
        e.loadR(RAX, gpr(rt));
        e.bytes({0xD3, 0xC0});       // rol eax, cl
        e.andEaxI(ppcmask(f_mb(insn), f_me(insn)));
        e.storeR(gpr(ra), RAX);
        aluTail(rc);
        return;
    }

    // ---- XO arithmetic (h_add family) -------------------------------------
    case Lo::Add:
        e.loadR(RAX, gpr(ra));
        e.mem13(0x03, RAX, gpr(rb));
        e.storeR(gpr(rt), RAX);
        aluTail(rc);
        return;
    case Lo::Subf: // rt = rb - ra
        e.loadR(RAX, gpr(rb));
        e.mem13(0x2B, RAX, gpr(ra));
        e.storeR(gpr(rt), RAX);
        aluTail(rc);
        return;
    case Lo::Neg:
        e.loadR(RAX, gpr(ra));
        e.bytes({0xF7, 0xD8});
        e.storeR(gpr(rt), RAX);
        aluTail(rc);
        return;
    case Lo::Addc:
        e.bytes({0x31, 0xD2});
        e.loadR(RAX, gpr(ra));
        e.mem13(0x03, RAX, gpr(rb));
        e.bytes({0x0F, 0x92, 0xC2}); // setc dl
        e.storeR(gpr(rt), RAX);
        caFromEdx();
        aluTail(rc);
        return;
    case Lo::Subfc: // rt = rb - ra, CA = no borrow
        e.bytes({0x31, 0xD2});
        e.loadR(RAX, gpr(rb));
        e.mem13(0x2B, RAX, gpr(ra));
        e.bytes({0x0F, 0x93, 0xC2}); // setnc dl
        e.storeR(gpr(rt), RAX);
        caFromEdx();
        aluTail(rc);
        return;
    case Lo::Adde:
    case Lo::Addze:
    case Lo::Addme:
    case Lo::Subfe:
    case Lo::Subfze:
    case Lo::Subfme: {
        const bool sub =
            lo == Lo::Subfe || lo == Lo::Subfze || lo == Lo::Subfme;
        e.bytes({0x31, 0xD2});       // xor edx, edx
        e.loadR(RCX, o.xer);
        e.bytes({0x0F, 0xBA, 0xE1, 0x1D}); // bt ecx, 29 -> CF = CA
        e.loadR(RAX, gpr(ra));
        if (sub)
            e.bytes({0xF7, 0xD0}); // not eax (flags preserved)
        if (lo == Lo::Adde || lo == Lo::Subfe) {
            e.mem13(0x13, RAX, gpr(rb)); // adc eax, [rb]
        } else if (lo == Lo::Addze || lo == Lo::Subfze) {
            e.bytes({0x83, 0xD0, 0x00}); // adc eax, 0
        } else {
            e.bytes({0x83, 0xD0, 0xFF}); // adc eax, -1
        }
        e.bytes({0x0F, 0x92, 0xC2}); // setc dl
        e.storeR(gpr(rt), RAX);
        caFromEdx();
        aluTail(rc);
        return;
    }
    case Lo::Mullw:
        e.loadR(RAX, gpr(ra));
        e.mem13_0F(0xAF, RAX, gpr(rb)); // imul eax, [rb]
        e.storeR(gpr(rt), RAX);
        aluTail(rc);
        return;
    case Lo::Mulhw:
    case Lo::Mulhwu:
        e.loadR(RAX, gpr(ra));
        e.mem13(0xF7, lo == Lo::Mulhw ? 5u : 4u, gpr(rb)); // imul/mul [rb]
        e.bytes({0x89, 0xD0});                             // mov eax, edx
        e.storeR(gpr(rt), RAX);
        aluTail(rc);
        return;

    // ---- X logicals (LOGIC macro family; source is rS = rt field) ---------
    case Lo::And:
    case Lo::Or:
    case Lo::Xor:
    case Lo::Eqv:
    case Lo::Nor:
    case Lo::Nand: {
        e.loadR(RAX, gpr(rt));
        const u8 op = (lo == Lo::And || lo == Lo::Nand) ? 0x23
                    : (lo == Lo::Xor || lo == Lo::Eqv)  ? 0x33
                                                        : 0x0B;
        e.mem13(op, RAX, gpr(rb));
        if (lo == Lo::Eqv || lo == Lo::Nor || lo == Lo::Nand)
            e.bytes({0xF7, 0xD0}); // not eax
        e.storeR(gpr(ra), RAX);
        aluTail(rc);
        return;
    }
    case Lo::Andc:
    case Lo::Orc:
        e.loadR(RAX, gpr(rb));
        e.bytes({0xF7, 0xD0}); // not eax
        e.mem13(lo == Lo::Andc ? 0x23 : 0x0B, RAX, gpr(rt));
        e.storeR(gpr(ra), RAX);
        aluTail(rc);
        return;

    // ---- shifts / extends --------------------------------------------------
    case Lo::Slw:
    case Lo::Srw:
        e.bytes({0x31, 0xD2}); // xor edx, edx
        e.loadR(RCX, gpr(rb));
        e.loadR(RAX, gpr(rt));
        e.bytes({0xD3, lo == Lo::Slw ? u8(0xE0) : u8(0xE8)}); // shl/shr eax,cl
        e.bytes({0xF7, 0xC1, 0x20, 0x00, 0x00, 0x00});        // test ecx, 32
        e.bytes({0x0F, 0x45, 0xC2});                          // cmovnz eax,edx
        e.storeR(gpr(ra), RAX);
        aluTail(rc);
        return;
    case Lo::Srawi: {
        const u32 n = f_sh(insn);
        if (!n) { // shift 0: result = rS, CA = 0 (h_srawi: n != 0 required)
            e.loadR(RAX, gpr(rt));
            e.storeR(gpr(ra), RAX);
            e.bytes({0x31, 0xD2}); // xor edx, edx
            caFromEdx();
            aluTail(rc);
            return;
        }
        e.loadR(RAX, gpr(rt));
        e.bytes({0x89, 0xC2});             // mov edx, eax (the lost bits)
        e.bytes({0xC1, 0xF8, u8(n)});      // sar eax, n
        e.storeR(gpr(ra), RAX);
        e.bytes({0x89, 0xC1});             // mov ecx, eax
        e.bytes({0xC1, 0xE9, 0x1F});       // shr ecx, 31 (sign)
        e.bytes({0x45, 0x31, 0xC0});       // xor r8d, r8d
        e.bytes({0x81, 0xE2});             // and edx, (1<<n)-1  (sets ZF)
        e.i32le((1u << n) - 1u);
        e.bytes({0x41, 0x0F, 0x95, 0xC0}); // setnz r8b
        e.bytes({0x44, 0x21, 0xC1});       // and ecx, r8d
        e.bytes({0x89, 0xCA});             // mov edx, ecx
        caFromEdx();
        aluTail(rc);
        return;
    }
    case Lo::Cntlzw: {
        e.loadR(RAX, gpr(rt));
        e.bytes({0x0F, 0xBD, 0xD0}); // bsr edx, eax
        const size_t zJ = e.j8(CC_Z);
        e.movEaxI(31);
        e.bytes({0x29, 0xD0}); // sub eax, edx
        const size_t dJ = e.jmp8();
        e.patch8(zJ);
        e.movEaxI(32);
        e.patch8(dJ);
        e.storeR(gpr(ra), RAX);
        aluTail(rc);
        return;
    }
    case Lo::Extsb:
    case Lo::Extsh:
        e.mem13_0F(lo == Lo::Extsb ? 0xBE : 0xBF, RAX, gpr(rt)); // movsx
        e.storeR(gpr(ra), RAX);
        aluTail(rc);
        return;

    // ---- branches (h_b / h_bc / h_bclr / h_bcctr) --------------------------
    case Lo::B: {
        const u32 disp = u32(sext26(ppcbits(insn, 6, 29) << 2));
        const u32 target = f_aabit(insn) ? disp : cia + disp;
        if (f_lkbit(insn))
            e.storeI(o.lr, nextVa);
        bump();
        if (target != nextVa) {
            e.storeI(o.pc, target);
            flushNoPc();
            e.jmpTo(epi);
        }
        return; // b .+4 falls through to the next segment, pc deferred
    }
    case Lo::Bc: {
        const u32 disp = u32(sext14(ppcbits(insn, 16, 29)) * 4);
        const u32 target = f_aabit(insn) ? disp : cia + disp;
        const int v = condPrep(ppcbits(insn, 6, 10), ppcbits(insn, 11, 15),
                               true);
        if (f_lkbit(insn))
            e.storeI(o.lr, nextVa); // written whether or not taken
        const size_t nt = condJump(v);
        bump();
        e.storeI(o.pc, target);
        flushNoPc();
        e.jmpTo(epi);
        if (nt != size_t(-1)) {
            e.patch8(nt);
            bump(); // not taken: fall through, pc deferred
        }
        return;
    }
    case Lo::Bclr:
    case Lo::Bcctr: {
        e.loadR(RAX, lo == Lo::Bclr ? o.lr : o.ctr);
        e.bytes({0x83, 0xE0, 0xFC}); // and eax, ~3
        const int v = condPrep(ppcbits(insn, 6, 10), ppcbits(insn, 11, 15),
                               lo == Lo::Bclr); // bcctr never touches CTR
        if (f_lkbit(insn))
            e.storeI(o.lr, nextVa);
        const size_t nt = condJump(v);
        bump();
        e.storeR(o.pc, RAX);
        flushNoPc();
        e.jmpTo(epi);
        if (nt != size_t(-1)) {
            e.patch8(nt);
            bump();
        }
        return;
    }

    // ---- loads / stores ----------------------------------------------------
    case Lo::Lwz:   load(insn, nextVa, 2, false, false, false, false); return;
    case Lo::Lbz:   load(insn, nextVa, 0, false, false, false, false); return;
    case Lo::Lhz:   load(insn, nextVa, 1, false, false, false, false); return;
    case Lo::Lha:   load(insn, nextVa, 1, false, false, true, false); return;
    case Lo::Lwzx:  load(insn, nextVa, 2, true, false, false, false); return;
    case Lo::Lbzx:  load(insn, nextVa, 0, true, false, false, false); return;
    case Lo::Lhzx:  load(insn, nextVa, 1, true, false, false, false); return;
    case Lo::Lhax:  load(insn, nextVa, 1, true, false, true, false); return;
    case Lo::Lwzu:  load(insn, nextVa, 2, false, true, false, false); return;
    case Lo::Lbzu:  load(insn, nextVa, 0, false, true, false, false); return;
    case Lo::Lhzu:  load(insn, nextVa, 1, false, true, false, false); return;
    case Lo::Lhau:  load(insn, nextVa, 1, false, true, true, false); return;
    case Lo::Lwzux: load(insn, nextVa, 2, true, true, false, false); return;
    case Lo::Lbzux: load(insn, nextVa, 0, true, true, false, false); return;
    case Lo::Lhzux: load(insn, nextVa, 1, true, true, false, false); return;
    case Lo::Lhaux: load(insn, nextVa, 1, true, true, true, false); return;
    case Lo::Lwbrx: load(insn, nextVa, 2, true, false, false, true); return;
    case Lo::Lhbrx: load(insn, nextVa, 1, true, false, false, true); return;
    case Lo::Stw:   store(insn, nextVa, 2, false, false, false); return;
    case Lo::Stb:   store(insn, nextVa, 0, false, false, false); return;
    case Lo::Sth:   store(insn, nextVa, 1, false, false, false); return;
    case Lo::Stwx:  store(insn, nextVa, 2, true, false, false); return;
    case Lo::Stbx:  store(insn, nextVa, 0, true, false, false); return;
    case Lo::Sthx:  store(insn, nextVa, 1, true, false, false); return;
    case Lo::Stwu:  store(insn, nextVa, 2, false, true, false); return;
    case Lo::Stbu:  store(insn, nextVa, 0, false, true, false); return;
    case Lo::Sthu:  store(insn, nextVa, 1, false, true, false); return;
    case Lo::Stwux: store(insn, nextVa, 2, true, true, false); return;
    case Lo::Stbux: store(insn, nextVa, 0, true, true, false); return;
    case Lo::Sthux: store(insn, nextVa, 1, true, true, false); return;
    case Lo::Stwbrx: store(insn, nextVa, 2, true, false, true); return;
    case Lo::Sthbrx: store(insn, nextVa, 1, true, false, true); return;

    // ---- FP loads / stores / register move ---------------------------------
    case Lo::Lfs:   fpMem(insn, row, nextVa, false, true, false, false); return;
    case Lo::Lfsu:  fpMem(insn, row, nextVa, false, true, false, true); return;
    case Lo::Lfsx:  fpMem(insn, row, nextVa, false, true, true, false); return;
    case Lo::Lfsux: fpMem(insn, row, nextVa, false, true, true, true); return;
    case Lo::Lfd:   fpMem(insn, row, nextVa, false, false, false, false); return;
    case Lo::Lfdu:  fpMem(insn, row, nextVa, false, false, false, true); return;
    case Lo::Lfdx:  fpMem(insn, row, nextVa, false, false, true, false); return;
    case Lo::Lfdux: fpMem(insn, row, nextVa, false, false, true, true); return;
    case Lo::Stfs:  fpMem(insn, row, nextVa, true, true, false, false); return;
    case Lo::Stfsu: fpMem(insn, row, nextVa, true, true, false, true); return;
    case Lo::Stfsx: fpMem(insn, row, nextVa, true, true, true, false); return;
    case Lo::Stfsux: fpMem(insn, row, nextVa, true, true, true, true); return;
    case Lo::Stfd:  fpMem(insn, row, nextVa, true, false, false, false); return;
    case Lo::Stfdu: fpMem(insn, row, nextVa, true, false, false, true); return;
    case Lo::Stfdx: fpMem(insn, row, nextVa, true, false, true, false); return;
    case Lo::Stfdux: fpMem(insn, row, nextVa, true, false, true, true); return;
    case Lo::Fmr: { // h_fmr: fpr[frt] = fpr[frb]; Rc (CR1 from FPSCR) falls back
        if (f_rcbit(insn)) {
            fallback(insn, row, nextVa);
            return;
        }
        e.loadR(RAX, o.msr);
        e.b(0xA9); // test eax, imm32
        e.i32le(0x2000u);
        const size_t fbJ = e.j32(CC_Z);
        e.mem13W(0x8B, RAX, fpr(f_rb(insn))); // mov rax, [fpr frb]
        e.mem13W(0x89, RAX, fpr(f_rt(insn))); // mov [fpr frt], rax
        bump();
        const size_t joinJ = e.jmp32f();
        e.patch32(fbJ);
        fallback(insn, row, nextVa);
        e.patch32(joinJ);
        return;
    }

    // ---- SPR fast forms (h_mfspr/h_mtspr, LR and CTR only) -----------------
    case Lo::Mfspr: {
        const u32 spr = f_spr(insn);
        if (spr != 8 && spr != 9) {
            fallback(insn, row, nextVa);
            return;
        }
        e.loadR(RAX, spr == 8 ? o.lr : o.ctr);
        e.storeR(gpr(rt), RAX);
        bump();
        return;
    }
    case Lo::Mtspr: {
        const u32 spr = f_spr(insn);
        if (spr != 8 && spr != 9) {
            fallback(insn, row, nextVa);
            return;
        }
        e.loadR(RAX, gpr(rt));
        e.storeR(spr == 8 ? o.lr : o.ctr, RAX);
        bump();
        return;
    }
    }
}

#endif // OPM_JIT_HOST

} // namespace

// ---- JitCache --------------------------------------------------------------

JitCache::JitCache()
{
#if OPM_JIT_HOST
    arena_ = static_cast<u8*>(VirtualAlloc(nullptr, kArenaBytes,
                                           MEM_COMMIT | MEM_RESERVE,
                                           PAGE_EXECUTE_READWRITE));
    cap_ = arena_ ? kArenaBytes : 0;
#endif
}

JitCache::~JitCache()
{
#if OPM_JIT_HOST
    if (arena_)
        VirtualFree(arena_, 0, MEM_RELEASE);
#endif
}

void JitCache::dropAll()
{
    for (JitLine& l : line)
        l.base = 1;
    std::memset(rr, 0, sizeof rr);
}

void JitCache::bind(Cpu& c)
{
#if OPM_JIT_HOST
    if (!arena_)
        return;
    // Displacements measured off the live object. Cpu is not standard-layout
    // (it owns std::map members), so offsetof is off the table; the object
    // itself is not.
    char* b = reinterpret_cast<char*>(&c);
    auto d32 = [&](const void* p) {
        return i32(reinterpret_cast<const char*>(p) - b);
    };
    offs.gpr0 = d32(&c.st.gpr[0]);
    offs.pc = d32(&c.st.pc);
    offs.cr = d32(&c.st.cr);
    offs.xer = d32(&c.st.xer);
    offs.lr = d32(&c.st.lr);
    offs.ctr = d32(&c.st.ctr);
    offs.pend = d32(&c.pendCycles);
    offs.curInsn = d32(&c.curInsn);
    offs.halted = d32(&c.halted);
    offs.batchBreak = d32(&c.batchBreak);
    offs.msr = d32(&c.st.msr);
    offs.fpr0 = d32(&c.st.fpr[0]);
    offs.fetchLine0 = d32(&c.fetchLine[0].base);
    offs.flStride = i32(sizeof(Cpu::FetchLine));

    // The trampoline and the shared epilogue, emitted like everything else.
    // enter(entry=rcx, cpu=rdx, stamp=r8, cyc=r9). Eight callee-saved pushes
    // plus the 40-byte outgoing area leave rsp 16-aligned inside a block, so
    // a block's `call shim` puts the callee at the ABI's rsp%16==8, with the
    // reserved bytes serving as its shadow space.
    Emit e{arena_, cap_, 0};
    e.bytes({0x53, 0x55, 0x56, 0x57});                     // push rbx/rbp/rsi/rdi
    e.bytes({0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57}); // push r12..r15
    e.bytes({0x48, 0x83, 0xEC, 0x28});                     // sub rsp, 40
    e.bytes({0x49, 0x89, 0xD5});                           // mov r13, rdx
    e.bytes({0x4D, 0x89, 0xC6});                           // mov r14, r8
    e.bytes({0x4D, 0x89, 0xCF});                           // mov r15, r9
    e.bytes({0x31, 0xDB});                                 // xor ebx, ebx
    e.bytes({0xFF, 0xE1});                                 // jmp rcx
    epilogue_ = u32(e.at);
    e.bytes({0x48, 0x83, 0xC4, 0x28});                     // add rsp, 40
    e.bytes({0x41, 0x5F, 0x41, 0x5E, 0x41, 0x5D, 0x41, 0x5C}); // pop r15..r12
    e.bytes({0x5F, 0x5E, 0x5D, 0x5B});                     // pop rdi/rsi/rbp/rbx
    e.b(0xC3);
    firstCode_ = e.at;
    at_ = e.at;
    enterFn = reinterpret_cast<EnterFn>(reinterpret_cast<void*>(arena_));
    live_ = e.ok;
#else
    (void)c;
#endif
}

bool JitCache::compileLine(Cpu& c, u32 slot, u32 paBase, const u32* w,
                           const u16* rows, u32 vaBase)
{
#if OPM_JIT_HOST
    if (!ready())
        return false;
    if (cap_ - at_ < 8192) { // a full arena: drop everything, start over
        dropAll();
        at_ = firstCode_;
        ++resets;
    }
    // The victim way: an invalid way if one exists, else round-robin. The
    // ways are what keep a fetch-slot conflict from being a compile storm —
    // see the field comment in the header.
    JitLine* ways = &line[slot * kWays];
    u32 way = kWays;
    for (u32 k = 0; k < kWays; ++k)
        if (ways[k].base == 1u) {
            way = k;
            break;
        }
    if (way == kWays) {
        way = rr[slot];
        rr[slot] = u8((rr[slot] + 1u) % kWays);
    }
    JitLine& jl = ways[way];
    jl.base = 1;
    Compiler cp{Emit{arena_, cap_, at_},
                c,
                offs,
                epilogue_,
                paBase,
                vaBase,
                i32(i64(offs.fetchLine0) + i64(slot) * offs.flStride)};
    for (u32 k = 0; k < 8; ++k) {
        jl.off[k] = u32(cp.e.at);
        cp.word(k, w[k], rows[k]);
    }
    // Word 7's fallthrough is the line end: settle the deferred pc (the word
    // after the line) and the batched counters, then leave. Every other path
    // out of the block flushed at its own exit.
    cp.flushPc(vaBase + 32u);
    cp.e.jmpTo(epilogue_);
    if (!cp.e.ok)
        return false; // ran out mid-compile; the next touch retries fresh
    at_ = cp.e.at;
    std::memcpy(jl.srcW, w, 32);
    jl.va = vaBase;
    jl.base = paBase;
    ++compiles;
    return true;
#else
    (void)c; (void)slot; (void)paBase; (void)w; (void)rows; (void)vaBase;
    return false;
#endif
}

// ---- the two doors ---------------------------------------------------------

bool jitRunLine(Cpu& c, u32 slot, u32 word, u64& stamp)
{
#if OPM_JIT_HOST
    if (!c.jit) {
        c.jit = std::make_unique<JitCache>();
        c.jit->bind(c);
        if (!c.jit->ready()) {
            c.jitOn = false; // no executable memory: interpreter it is
            return false;
        }
    }
    JitCache& J = *c.jit;
    const Cpu::FetchLine& fl = c.fetchLine[slot];
    const u32 vaBase = c.st.pc & ~31u;
    JitLine* ways = &J.line[slot * JitCache::kWays];
    JitLine* jl = nullptr;
    for (u32 k = 0; k < JitCache::kWays; ++k)
        if (ways[k].base == fl.base && ways[k].va == vaBase) {
            jl = &ways[k];
            break;
        }
    if (!jl) {
        if (!J.compileLine(c, slot, fl.base, fl.w, fl.row, vaBase)) {
            ++J.bails;
            return false;
        }
        for (u32 k = 0; k < JitCache::kWays; ++k)
            if (ways[k].base == fl.base && ways[k].va == vaBase) {
                jl = &ways[k];
                break;
            }
        if (!jl) { // compile succeeded but the way is unfindable: impossible,
            ++J.bails; // but a wrong branch here must fail safe
            return false;
        }
    }
    ++J.enters;
    const u64 before = stamp;
    J.enterFn(J.arena_ + jl->off[word], &c, &stamp,
              u64(c.insnCycles) + c.extraCycles);
    J.insns += stamp - before;
    return true;
#else
    (void)c; (void)slot; (void)word; (void)stamp;
    return false;
#endif
}

void jitNoteRefill(Cpu& c, u32 slot, u32 base, const u32* w)
{
    if (!c.jit)
        return;
    // Only the ways holding THIS base are in question. A refill of a
    // different base is a fetch-slot conflict, not an invalidation — the
    // other ways' code bytes did not change, and dropping them was measured
    // at 293k recompiles per 200M in-game instructions. A way whose base
    // matches keeps its block only when the refilled content is identical;
    // the mismatch case is both live self-modifying code AND code that was
    // modified while its line was out of residency, because the refill
    // always precedes the next execution.
    JitLine* ways = &c.jit->line[slot * JitCache::kWays];
    for (u32 k = 0; k < JitCache::kWays; ++k) {
        JitLine& jl = ways[k];
        if (jl.base != base || jl.base == 1u)
            continue;
        if (std::memcmp(jl.srcW, w, 32) == 0) {
            ++c.jit->refillKeeps;
        } else {
            jl.base = 1u;
            ++c.jit->refillDrops;
        }
    }
}

} // namespace opm

