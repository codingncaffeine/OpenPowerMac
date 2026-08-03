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

// A chain site's rel32 states (Stage B). The site is `jmp rel32`; relative to
// its end sit, in fixed layout, a 12-byte parking block (dec hops + jmp
// epilogue) and then the resolver thunk. So rel32 = 0 parks the site on the
// dispatcher exit (backref capacity exhausted — permanent until the block
// recompiles), rel32 = 12 lands on the thunk (initial state, and the severed
// state: re-resolve on next traversal), and anything else is a live link to
// the successor's segment. The emitter, the resolver and sever() all speak
// this encoding, so it lives here, once.
inline constexpr i32 kChainRelPark = 0;
inline constexpr i32 kChainRelThunk = 12;

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
    // s42: with the softfp arithmetic lowered, the by-cycles census turned up
    // four rows that are pure REGISTER work still paying a call out of the
    // block — 7% of what was left of the fallback between them, for a sign
    // bit and two condition-register field moves.
    Fneg, Fabs, Fnabs, Mcrf, Mtcrf,
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
        set("fneg", Lo::Fneg);   set("fabs", Lo::Fabs);
        set("fnabs", Lo::Fnabs);
        set("mcrf", Lo::Mcrf);   set("mtcrf", Lo::Mtcrf);
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

// The --jit-tsc timer, in the shape the shims can use without duplicating
// their bodies: construct at entry, destruct at return, charge nothing when
// the flag is off.
struct ShimTsc {
    JitCache& j;
    u64* bin;
    u64 t0;
    ShimTsc(Cpu* c, u64* b)
        : j(*c->jit), bin(b), t0(j.tscOn ? __rdtsc() : 0)
    {
    }
    ~ShimTsc()
    {
        if (j.tscOn)
            *bin += __rdtsc() - t0;
    }
};

i64 shimRead8(Cpu* c, u32 ea)  { ++c->jit->memOps; ShimTsc t(c, &c->jit->tscMem); u32 v; return c->readV8(ea, v)  ? static_cast<i64>(v) : -1; }
i64 shimRead16(Cpu* c, u32 ea) { ++c->jit->memOps; ShimTsc t(c, &c->jit->tscMem); u32 v; return c->readV16(ea, v) ? static_cast<i64>(v) : -1; }
i64 shimRead32(Cpu* c, u32 ea) { ++c->jit->memOps; ShimTsc t(c, &c->jit->tscMem); u32 v; return c->readV32(ea, v) ? static_cast<i64>(v) : -1; }
i64 shimWrite8(Cpu* c, u32 ea, u32 v)  { ++c->jit->memOps; ShimTsc t(c, &c->jit->tscMem); return c->writeV8(ea, v)  ? 0 : -1; }
i64 shimWrite16(Cpu* c, u32 ea, u32 v) { ++c->jit->memOps; ShimTsc t(c, &c->jit->tscMem); return c->writeV16(ea, v) ? 0 : -1; }
i64 shimWrite32(Cpu* c, u32 ea, u32 v) { ++c->jit->memOps; ShimTsc t(c, &c->jit->tscMem); return c->writeV32(ea, v) ? 0 : -1; }
// execRow's TAIL, for the direct-call path below: the two rare pieces of
// per-instruction bookkeeping that follow a handler. Reached only when the
// emitted code's own tests say one of them is live (mmcr0 selecting an event,
// or MSR[SE|BE] tracing), so on the machine's actual path this is never
// called — but when it is, it does exactly what cpu.cpp does, in order.
void shimFbTail(Cpu* c, u32 insn, u32 row, u32 fallThrough)
{
    if (c->st.mmcr0 && !(c->st.mmcr0 & 0x80000000u)) { // FC
        const u32 sel1 = (c->st.mmcr0 >> 6) & 0x7Fu;
        const u32 sel2 = c->st.mmcr0 & 0x3Fu;
        const u32 pmxe = c->st.mmcr0 & 0x04000000u;
        if (sel1 == 1 || sel1 == 2) {
            const u32 old = c->st.pmc[0];
            c->st.pmc[0] += 1;
            if (!(old & 0x80000000u) && (c->st.pmc[0] & 0x80000000u) && pmxe &&
                (c->st.mmcr0 & 0x00008000u))
                c->pmPending = true;
        }
        if (sel2 == 1 || sel2 == 2) {
            const u32 old = c->st.pmc[1];
            c->st.pmc[1] += 1;
            if (!(old & 0x80000000u) && (c->st.pmc[1] & 0x80000000u) && pmxe &&
                (c->st.mmcr0 & 0x00004000u))
                c->pmPending = true;
        }
    }
    if (c->halted || c->raisedThisStep)
        return;
    if (c->st.msr & (msr::SE | msr::BE)) {
        const InsnDesc& d = kIsa[row];
        const bool isRfi = d.kind == Xk::X19 && d.xo == 50;
        const bool isIsync = d.kind == Xk::X19 && d.xo == 150;
        const bool branchTaken =
            (d.pat == Pat::B || d.pat == Pat::BC || d.pat == Pat::BCLR ||
             d.pat == Pat::BCCTR) &&
            c->st.pc != fallThrough;
        if (!isRfi && !isIsync &&
            ((c->st.msr & msr::SE) ||
             ((c->st.msr & msr::BE) && branchTaken)))
            c->raiseExc(Exc::Trace, c->st.pc, 0);
    }
    (void)insn;
}

void shimExecRow(Cpu* c, u32 insn, u32 row)
{
    JitCache& J = *c->jit;
    ++J.fallbacks; // the census: which rows still cost an execRow call
    const u32 slot = row < 1024u ? row : 1023u;
    ++J.fbByRow[slot];
    if (!J.tscOn) {
        c->execRow(insn, row);
        return;
    }
    const u64 t0 = __rdtsc();
    c->execRow(insn, row);
    const u64 d = __rdtsc() - t0;
    J.tscFb += d;
    J.tscByRow[slot] += d;
}

// The chain resolver (JIT_PLAN §7 Stage B), called from a site's thunk the
// first time an unlinked site is traversed: find the successor's compiled
// block, patch the site's rel32 to its entry, and record the backref so
// invalidation can sever the link. The traversal that called this still
// exits to the dispatcher — the link pays from the NEXT one — so a resolve
// is never a hop (the thunk decrements the hop counter the site charged).
// A miss (successor not compiled yet) retries next traversal; the
// dispatcher compiles the successor in between, so a site links on its
// second traversal in the common case. Patching its own block's bytes here
// is safe: the rel32 was consumed by the jump that brought us in, and no
// other block's code is touched.
void opmJitChainResolve(Cpu* c, u32 siteOff, u32 tgtPaLine, u32 tgtVa)
{
    JitCache& J = *c->jit;
    ++J.chainResolves;
    const u32 slot = Cpu::fetchSlot(tgtPaLine);
    JitLine* ways = &J.line[slot * JitCache::kWays];
    const u32 vaLine = tgtVa & ~31u;
    for (u32 k = 0; k < JitCache::kWays; ++k) {
        JitLine& jl = ways[k];
        if (jl.base != tgtPaLine || jl.va != vaLine)
            continue;
        if (jl.nBref >= JitLine::kBrefs) {
            // No room to record the unlink — and a link that cannot be
            // severed is a use-after-invalidate waiting to happen. Park the
            // site on the dispatcher exit instead, permanently (until the
            // SITE's own block recompiles): a parked traversal costs what
            // v1 cost, and the counter names how often it happens.
            ++J.chainGiveups;
            const i32 rel = kChainRelPark;
            std::memcpy(J.arena_ + siteOff, &rel, 4);
            return;
        }
        jl.bref[jl.nBref++] = siteOff;
        const i32 rel = i32(jl.off[(tgtVa >> 2) & 7u]) - i32(siteOff + 4u);
        std::memcpy(J.arena_ + siteOff, &rel, 4);
        ++J.chainLinks;
        return;
    }
    ++J.chainMisses;
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
    ++c->jit->fpMemOps;
    ShimTsc t(c, &c->jit->tscMem);
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
    ++c->jit->fpMemOps;
    ShimTsc t(c, &c->jit->tscMem);
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
    ++c->jit->fpMemOps;
    ShimTsc t(c, &c->jit->tscMem);
    if (ea & 3u)
        return shimAlignF(c, ea, insn);
    return c->writeV(ea, 4, sf::storeSingle(c->st.fpr[f_rt(insn)])) ? 0 : -1;
}
i64 shimStfd(Cpu* c, u32 ea, u32 insn)
{
    ++c->jit->fpMemOps;
    ShimTsc t(c, &c->jit->tscMem);
    if (ea & 3u)
        return shimAlignF(c, ea, insn);
    return c->writeV(ea, 8, c->st.fpr[f_rt(insn)]) ? 0 : -1;
}

// ---- byte emitter ----------------------------------------------------------

enum : u32 { RAX = 0, RCX = 1, RDX = 2, R8 = 8, R9 = 9, R10 = 10, R12 = 12 };
enum : u8 { CC_B = 2, CC_E = 4, CC_NE = 5, CC_Z = 4, CC_NZ = 5 };

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
    // Patch a stored rel32 to land on arena offset dstOff — the compile-time
    // fixup for forward intra-line chains (runtime patching belongs to the
    // resolver, not the emitter).
    void patchRel32To(size_t pos, u32 dstOff)
    {
        const i64 rel = i64(dstOff) - i64(pos + 4);
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
    i32 flBase;         // r13-relative disp of fetchLine[slot].base
    const u32* segOff;  // jl.off — valid for words already emitted
    bool chain;         // Stage B on (compile chain sites at eligible exits)
    bool direct;        // call bound handlers directly (else the execRow shim)
    JitCache* jc;       // for the census counters' absolute addresses
    // Forward intra-line chain jumps: the target segment's offset is not
    // known until its word is emitted, so compileLine patches these after
    // the loop. {position of the rel32, target word}.
    std::vector<std::pair<size_t, u32>> fixes;

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

    // -- Stage B: chain points (JIT_PLAN §7) ---------------------------------
    //
    // A chain point replaces one dispatcher round trip: it is emitted at
    // every exit whose successor is known at compile time AND lies in the
    // same 4K page (taken direct b/bc; the line-end fallthrough). The gate
    // below is a transcription of everything the dispatcher would have
    // checked between this exit and re-entering compiled code — any failure
    // exits to the epilogue, which is always safe (the dispatcher re-derives
    // the same answers).
    //
    // ⚠ WHY SAME-PAGE IS THE WHOLE MAPPING ARGUMENT. This block was entered
    // through the dispatcher, whose fetchBlockFast proved the one-page fetch
    // translation CURRENT for this page (xlGen==mmuGen, xlMsr, xlSr, and
    // xlPa == this block's PA page). The xl* cache itself cannot change
    // during a chained run — only the fetch path writes it, and blocks never
    // fetch — and every writer of the mapping's inputs either breaks the
    // batch (mtsr/mtsrin, SDR1, the BATs, tlbie/tlbia — verified setters,
    // tested after every instruction that can set them) or leaves the block
    // by pc (rfi), EXCEPT a plain mtmsr, which the interpreter only catches
    // at its next fetchBlockFast via the xlMsr compare. That one hole is
    // closed here the same way: msr & (IR|PR|LE) against xlMsr, per hop. So
    // within one unbroken batch, a same-page target's PA line is the
    // compile-time constant (own PA page | target's page offset), and a
    // cross-page target — which would need the full xlate check re-emitted —
    // is simply not chained.
    //
    // ⚠ THE BUDGET BOUND RIDES IN RBP (Cpu::jitUntil, loaded by the
    // trampoline): a hop requires until - stamp >= 8, the dispatcher's own
    // JIT-entry condition, and a block runs at most 8 instructions between
    // checks — so a chained run can never overrun the batch, which is what
    // fpClamp exactness rides on. Preconditions at every chain point: pc
    // stored (immediate), counters flushed (rbx == 0) — every caller sits
    // right after flushPc/flushNoPc.

    void hopInc()
    {
        e.bytes({0x49, 0xFF, 0x85}); // inc qword [r13+chainHops]
        e.i32le(u32(o.chainHops));
    }
    void hopDec() // the cold arms of a site take back the hop it charged
    {
        e.bytes({0x49, 0xFF, 0x8D}); // dec qword [r13+chainHops]
        e.i32le(u32(o.chainHops));
    }

    void chainChecks() // the dispatcher's own gate, in test form
    {
        byteCheck(o.halted);      // post-run break test
        byteCheck(o.batchBreak);  // (provably false here, checked anyway)
        byteCheck(o.lineExecOff); // `slow`, term by term
        byteCheck(o.napping);
        e.mem13(0x83, 7, o.mmcr0); // cmp dword [r13+mmcr0], 0
        e.b(0);
        e.jccTo(CC_NE, epi);
        e.mem13(0xF6, 0, o.iabr);  // test byte [r13+iabr], 2
        e.b(2);
        e.jccTo(CC_NE, epi);
        e.loadR(RAX, o.msr);
        e.b(0xA9);                 // test eax, SE|BE
        e.i32le(msr::SE | msr::BE);
        e.jccTo(CC_NE, epi);
        e.bytes({0x89, 0xC2});     // mov edx, eax
        e.bytes({0x81, 0xE2});     // and edx, IR|PR|LE
        e.i32le(msr::IR | msr::PR | msr::LE);
        e.mem13(0x3B, RDX, o.xlMsr); // cmp edx, [r13+xlMsr] — the mtmsr hole
        e.jccTo(CC_NE, epi);
        e.b(0xA9);                 // test eax, EE
        e.i32le(msr::EE);
        const size_t noEe = e.j8(CC_Z);
        if (o.pend4Ok) {           // the four pending bools, one dword
            e.mem13(0x83, 7, o.pend4);
            e.b(0);
            e.jccTo(CC_NE, epi);
        } else {
            byteCheck(o.extIrq);
            byteCheck(o.smi);
            byteCheck(o.decP);
            byteCheck(o.pmP);
        }
        e.patch8(noEe);
        e.bytes({0x48, 0x89, 0xE8});       // mov rax, rbp (until)
        e.bytes({0x49, 0x2B, 0x06});       // sub rax, [r14]
        e.bytes({0x48, 0x83, 0xF8, 0x08}); // cmp rax, 8
        e.jccTo(CC_B, epi);                // budget < a line: dispatcher
    }

    // Intra-line hop: target segment in this same block. No residency or
    // linking machinery — the block's own validity covers it (mem ops and
    // fallbacks re-check fl.base at their own tails) — but the FULL gate
    // runs, because the interpreter passes through the dispatcher on every
    // taken branch, intra-line included.
    void chainIntra(u32 tgtWord, u32 curWord)
    {
        chainChecks();
        hopInc();
        if (tgtWord <= curWord) { // emitted already (self-loop included)
            e.jmpTo(segOff[tgtWord]);
        } else {
            e.b(0xE9);
            fixes.push_back({e.at, tgtWord});
            e.i32le(0);
        }
    }

    // Cross-line hop, same page: successor residency check + the patchable
    // site. Layout after the site's rel32 is fixed (kChainRel*): a parking
    // block, then the resolver thunk. Initial rel32 = thunk.
    void chainCross(u32 tgtVa)
    {
        const u32 tgtPaLine = (paBase & ~0xFFFu) | (tgtVa & 0xFE0u);
        const u32 slot = (tgtPaLine >> 5) & (JitCache::kLines - 1u);
        const i32 flD = i32(i64(o.fetchLine0) + i64(slot) * o.flStride);
        chainChecks();
        e.mem13(0x81, 7, flD); // cmp dword [fl'], paLine' — resident + same
        e.i32le(tgtPaLine);
        e.jccTo(CC_NE, epi);
        hopInc();
        e.b(0xE9); // the site
        const u32 siteOff = u32(e.at);
        e.i32le(u32(kChainRelThunk));
        const size_t parkAt = e.at;
        hopDec();      // parking block: not a hop after all
        e.jmpTo(epi);
        // ⚠ THE PARKING BLOCK'S LENGTH *IS* kChainRelThunk. The site's three
        // states are encoded as rel32 values (0 = park, 12 = thunk, anything
        // else = a live link), so the thunk only sits at +12 while these two
        // instructions are exactly twelve bytes. Nothing else would notice if
        // an emitter helper changed size — the site would simply jump into
        // the middle of an instruction — so it is checked, not assumed.
        if (e.at - parkAt != size_t(kChainRelThunk))
            e.ok = false;
        hopDec();      // thunk: a resolve is not a hop either
        e.movEdxI(siteOff);
        e.bytes({0x41, 0xB8}); // mov r8d, paLine'
        e.i32le(tgtPaLine);
        e.bytes({0x41, 0xB9}); // mov r9d, tgtVa
        e.i32le(tgtVa);
        e.bytes({0x4C, 0x89, 0xE9}); // mov rcx, r13
        e.callAbs(reinterpret_cast<const void*>(&opmJitChainResolve));
        e.jmpTo(epi);
    }

    // The one door: chain if eligible, else the v1 exit. Same-line targets
    // are intra (a line never spans pages); same-page targets cross-chain;
    // anything else — and everything under --no-jit-chain — exits to the
    // dispatcher exactly as v1 did.
    void chainOrExit(u32 target, u32 curWord)
    {
        if (chain && (target & ~31u) == vaBase)
            chainIntra((target >> 2) & 7u, curWord);
        else if (chain && (target >> 12) == (vaBase >> 12))
            chainCross(target);
        else
            e.jmpTo(epi);
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

    // ⭐⭐ THE DIRECT-CALL FALLBACK — execRow, specialized to the row.
    //
    // MEASURED (--jit-tsc): fallbacks are 45.9% of the in-game window's block
    // time and the top eleven rows are ALL floating-point arithmetic, at
    // 110-158 host cycles each even though 96% of them take the softfp host
    // fast path. The arithmetic is not what costs that. execRow's PER-CALL
    // SCAFFOLD is: a load of dispPre[row] and its gate ladder, a load of
    // dispFn[row] and an INDIRECT CALL through it (79 live fallback rows in
    // this window — the indirect predictor cannot hold them), charge(), the
    // perfmon test and the trace test. Every one of those is either known
    // when the block is compiled or already batched in rbx.
    //
    // So: call the bound handler DIRECTLY, with execRow's own work
    // transcribed around it in execRow's own order. Nothing is skipped —
    // the two rare tails (perfmon, trace) keep their exact code in
    // shimFbTail behind the same tests execRow makes, and the gate ladder
    // is re-emitted for the one gate this path admits.
    //
    // Admitted rows (checked by the caller): a bound handler, and either no
    // pre-dispatch gate at all or the FP gate alone — the same predicate
    // that lets the FP loads inline, with the same cold arm (the generic
    // fallback, which raises FpUnavailable exactly as execRow would).
    //
    // ⚠ `keepBatch` SKIPS THE COUNTER FLUSH, and that is a claim about the
    // handler, not a general one. It is passed only for the FP compute rows,
    // whose handlers live entirely in fpu.cpp: that file touches st.fpr,
    // st.fpscr, the CR and (in trapIfEnabled) st.pc, and NOTHING else —
    // no readV/writeV, no bus, no device clock, no pendCycles, no tbNow. So
    // nothing there can observe the instruction count or the charged cycles,
    // and leaving both in rbx across the call is invisible. The one clock
    // thing an FP handler can reach, raiseExc's exception-ring record of
    // st.tb, reads the RAW timebase — which the interpreter leaves equally
    // un-advanced mid-batch — so both machines record the same value.
    void fallbackDirect(u32 insn, u32 row, u32 nextVa, const void* fn,
                        bool fpGate, bool keepBatch)
    {
        size_t coldJ = size_t(-1);
        if (fpGate) { // execRow's kPreFp arm, tested before anything is written
            e.loadR(RAX, o.msr);
            e.b(0xA9); // test eax, msr::FP
            e.i32le(0x2000u);
            coldJ = e.j32(CC_Z);
        }
        // The census the shim used to keep. It is what named this cost in the
        // first place, so it survives the lowering: two absolute increments,
        // ~4 uops against the ~50 the direct call saves.
        e.bytes({0x48, 0xB8}); // mov rax, imm64 &fallbacks
        e.i64le(reinterpret_cast<u64>(&jc->fallbacks));
        e.bytes({0x48, 0xFF, 0x00}); // inc qword [rax]
        e.bytes({0x48, 0xB8});       // mov rax, imm64 &fbByRow[row]
        e.i64le(reinterpret_cast<u64>(&jc->fbByRow[row < 1024u ? row : 1023u]));
        e.bytes({0x48, 0xFF, 0x00});
        // execRow's own preamble, in its order.
        e.mem13(0xC6, 0, o.raisedThis); // mov byte [r13+raisedThisStep], 0
        e.b(0);
        e.storeI(o.curInsn, insn);
        if (keepBatch)
            e.storeI(o.pc, nextVa); // execRow's `st.pc += 4`, counters batched
        else
            flushPc(nextVa);
        e.bytes({0x4C, 0x89, 0xE9}); // mov rcx, r13
        e.movEdxI(insn);
        e.bytes({0x49, 0xB8});       // mov r8, imm64 &kIsa[row]
        e.i64le(reinterpret_cast<u64>(&kIsa[row]));
        e.callAbs(fn);               // DIRECT: no dispFn indirection
        if (keepBatch)
            bump();      // execRow's charge(), batched like every inlined op
        else
            chargeInc(); // …or settled now, for a handler that could observe
        // The two rare tails, behind execRow's own conditions.
        e.mem13(0x83, 7, o.mmcr0); // cmp dword [r13+mmcr0], 0
        e.b(0);
        const size_t tailJ = e.j32(CC_NE);
        e.loadR(RAX, o.msr);
        e.b(0xA9); // test eax, SE|BE
        e.i32le(msr::SE | msr::BE);
        const size_t tail2J = e.j32(CC_NE);
        const size_t doneJ = e.jmp32f();
        e.patch32(tailJ);
        e.patch32(tail2J);
        if (keepBatch)
            flushNoPc(); // the tail can raise: settle before it observes
        e.bytes({0x4C, 0x89, 0xE9}); // mov rcx, r13
        e.movEdxI(insn);
        e.bytes({0x41, 0xB8});       // mov r8d, row
        e.i32le(row);
        e.bytes({0x41, 0xB9});       // mov r9d, fallThrough (== nextVa)
        e.i32le(nextVa);
        e.callAbs(reinterpret_cast<const void*>(&shimFbTail));
        e.patch32(doneJ);
        // The line executor's exit tests, exactly as the generic path runs
        // them. (The instruction count is already carried: bump/chargeInc
        // above did what the generic path's `inc [r14]` does after execRow.)
        byteCheck(o.halted);
        byteCheck(o.batchBreak);
        pcCheck(nextVa);
        flCheck();
        if (coldJ != size_t(-1)) {
            const size_t joinJ = e.jmp32f();
            e.patch32(coldJ);
            fallback(insn, row, nextVa); // FP unavailable: the generic path
            e.patch32(joinJ);
        }
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
    // Emit the final test; SIZE_MAX = always taken. `wide` (⚠ not `far`:
    // windows.h #defines that away) widens the not-taken jump to rel32 — a
    // chained taken arm (checks + site + thunk) is far past rel8 range.
    size_t condJump(int v, bool wide = false)
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
        return wide ? e.j32(CC_Z) : e.j8(CC_Z);
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
    // From here the row has a bound handler and at most the FP gate, so any
    // path that ends in execRow can go through the direct call instead. The
    // batch may ride across the call only for the FP COMPUTE patterns — see
    // fallbackDirect's note; the four FP memory patterns are excluded because
    // their handlers do touch memory (and are inlined anyway).
    const bool fpCompute =
        isFpInsn(d) &&
        (d.pat == Pat::FP2 || d.pat == Pat::FP3 || d.pat == Pat::FP3C ||
         d.pat == Pat::FP4 || d.pat == Pat::FCMP || d.pat == Pat::MTFSF ||
         d.pat == Pat::MTFSFI || d.pat == Pat::MTFSB || d.pat == Pat::MCRFS ||
         d.pat == Pat::MFFS);
    // ⚠ --jit-tsc COMPILES THE SHIM PATH. The rdtsc split brackets the shim
    // call; the direct path has no call to bracket, and emitting rdtsc inline
    // would measure a shape that is not the one shipping either. So the
    // instrument keeps measuring what a fallback costs THROUGH execRow —
    // still the right diagnostic for "what would lowering this row buy" —
    // and the lowering's own win is settled by the same-binary MIPS A/B
    // against --no-jit-direct, like every other speed claim here.
    const auto fallbackBest = [&] {
        if (!direct || jc->tscOn) {
            fallback(insn, row, nextVa);
            return;
        }
        fallbackDirect(insn, row, nextVa,
                       reinterpret_cast<const void*>(c.dispFn[row]),
                       c.dispPre[row] != 0, fpCompute);
    };
    if ((d.flags & FL_OE) && f_oebit(insn)) {
        fallbackBest();
        return;
    }
    const Lo lo = static_cast<Lo>(loTable()[row]);
    const bool rc = (d.flags & FL_RC) && f_rcbit(insn);
    const u32 rt = f_rt(insn), ra = f_ra(insn), rb = f_rb(insn);

    switch (lo) {
    default:
    case Lo::None:
        fallbackBest();
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
            chainOrExit(target, k);
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
        const bool chained = chain && target != nextVa &&
                             (target >> 12) == (vaBase >> 12);
        const size_t nt = condJump(v, chained);
        bump();
        e.storeI(o.pc, target);
        flushNoPc();
        if (chained)
            chainOrExit(target, k);
        else
            e.jmpTo(epi);
        if (nt != size_t(-1)) {
            if (chained)
                e.patch32(nt);
            else
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
    case Lo::Fmr:
    case Lo::Fneg:
    case Lo::Fabs:
    case Lo::Fnabs: {
        // fpr[frt] = fpr[frb], with the sign bit moved or left alone. Rc
        // (CR1 from the FPSCR) falls back; the MSR[FP] cold arm is execRow's,
        // exactly as for the FP loads and stores.
        if (f_rcbit(insn)) {
            fallbackBest();
            return;
        }
        e.loadR(RAX, o.msr);
        e.b(0xA9); // test eax, imm32
        e.i32le(0x2000u);
        const size_t fbJ = e.j32(CC_Z);
        e.mem13W(0x8B, RAX, fpr(f_rb(insn))); // mov rax, [fpr frb]
        if (lo == Lo::Fneg)
            e.bytes({0x48, 0x0F, 0xBA, 0xF8, 0x3F}); // btc rax, 63
        else if (lo == Lo::Fabs)
            e.bytes({0x48, 0x0F, 0xBA, 0xF0, 0x3F}); // btr rax, 63
        else if (lo == Lo::Fnabs)
            e.bytes({0x48, 0x0F, 0xBA, 0xE8, 0x3F}); // bts rax, 63
        e.mem13W(0x89, RAX, fpr(f_rt(insn))); // mov [fpr frt], rax
        bump();
        const size_t joinJ = e.jmp32f();
        e.patch32(fbJ);
        fallback(insn, row, nextVa);
        e.patch32(joinJ);
        return;
    }

    // ---- condition-register field moves (h_mcrf, h_mtcrf) ------------------
    case Lo::Mcrf: { // cr[crfD] = cr[crfS]
        const u32 shD = (7u - f_crfd(insn)) * 4u;
        const u32 shS = (7u - f_crfs(insn)) * 4u;
        e.loadR(RAX, o.cr);
        e.bytes({0x89, 0xC2}); // mov edx, eax
        if (shS)
            e.bytes({0xC1, 0xEA, u8(shS)}); // shr edx, shS
        e.bytes({0x83, 0xE2, 0x0F});        // and edx, 15
        if (shD)
            e.bytes({0xC1, 0xE2, u8(shD)}); // shl edx, shD
        e.andEaxI(~(0xFu << shD));
        e.bytes({0x09, 0xD0}); // or eax, edx
        e.storeR(o.cr, RAX);
        bump();
        return;
    }
    case Lo::Mtcrf: { // the masked fields of cr take rS's bits
        u32 mask = 0;
        for (u32 f = 0; f < 8; ++f)
            if (f_crm(insn) & (0x80u >> f))
                mask |= 0xFu << ((7u - f) * 4u);
        if (mask) {
            e.loadR(RAX, gpr(f_rt(insn)));
            e.andEaxI(mask);
            e.loadR(RDX, o.cr);
            e.bytes({0x81, 0xE2}); // and edx, imm32
            e.i32le(~mask);
            e.bytes({0x09, 0xD0}); // or eax, edx
            e.storeR(o.cr, RAX);
        }
        bump();
        return;
    }

    // ---- SPR fast forms (h_mfspr/h_mtspr, LR and CTR only) -----------------
    case Lo::Mfspr: {
        const u32 spr = f_spr(insn);
        if (spr != 8 && spr != 9) {
            fallbackBest();
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
            fallbackBest();
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
    for (JitLine& l : line) {
        l.base = 1;
        // Links die with the population: after a wholesale drop no block can
        // be entered (the dispatcher probes ways and finds none), so chained
        // jumps between the dead blocks are unreachable and need no arena
        // rewrites — but the backref lists must not survive into the lines'
        // next lives, where the offsets would point at unrelated code.
        l.nBref = 0;
    }
    std::memset(rr, 0, sizeof rr);
}

void JitCache::sever(JitLine& jl)
{
#if OPM_JIT_HOST
    for (u32 k = 0; k < jl.nBref; ++k) {
        const u32 s = jl.bref[k];
        if (size_t(s) + 4 <= cap_) {
            const i32 rel = kChainRelThunk; // back to "resolve me"
            std::memcpy(arena_ + s, &rel, 4);
            ++chainSevers;
        }
    }
#endif
    jl.nBref = 0;
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
    // Chain-gate fields (Stage B). The four pending bools are declared
    // adjacently; when the measured offsets confirm it, one dword compare
    // covers them all — measured, not assumed, so a reordered declaration
    // degrades to four byte tests instead of a wrong gate.
    offs.lineExecOff = d32(&c.lineExecOff);
    offs.napping = d32(&c.napping);
    offs.mmcr0 = d32(&c.st.mmcr0);
    offs.iabr = d32(&c.st.iabr);
    offs.xlMsr = d32(&c.xlMsr);
    offs.raisedThis = d32(&c.raisedThisStep);
    offs.extIrq = d32(&c.extIrqLine);
    offs.smi = d32(&c.smiPending);
    offs.decP = d32(&c.decPending);
    offs.pmP = d32(&c.pmPending);
    offs.pend4 = offs.extIrq;
    offs.pend4Ok = offs.smi == offs.extIrq + 1 &&
                   offs.decP == offs.extIrq + 2 &&
                   offs.pmP == offs.extIrq + 3;
    offs.until = d32(&c.jitUntil);
    offs.chainHops = d32(&c.jitChainHops);

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
    e.bytes({0x48, 0x8B, 0xAA});                           // mov rbp, [rdx+until]
    e.i32le(u32(offs.until));                              // (the budget bound)
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
    sever(jl); // way reuse: chains into the OLD block must re-resolve
    jl.base = 1;
    Compiler cp{Emit{arena_, cap_, at_},
                c,
                offs,
                epilogue_,
                paBase,
                vaBase,
                i32(i64(offs.fetchLine0) + i64(slot) * offs.flStride),
                jl.off,
                !c.jitChainOff,
                !c.jitDirectOff,
                this,
                {}}; // fixes: named, because gcc requires every member here
    for (u32 k = 0; k < 8; ++k) {
        jl.off[k] = u32(cp.e.at);
        cp.word(k, w[k], rows[k]);
    }
    // Word 7's fallthrough is the line end: settle the deferred pc (the word
    // after the line) and the batched counters, then chain to the next line
    // — the other half of every dispatcher round trip — or leave. Every
    // other path out of the block flushed at its own exit.
    cp.flushPc(vaBase + 32u);
    cp.chainOrExit(vaBase + 32u, 7u);
    // Forward intra-line chain targets, now that every segment has a home.
    for (const auto& f : cp.fixes)
        cp.e.patchRel32To(f.first, jl.off[f.second]);
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

bool jitRunLine(Cpu& c, u32 slot, u32 word, u64& stamp, u64 until)
{
#if OPM_JIT_HOST
    if (!c.jit) {
        c.jit = std::make_unique<JitCache>();
        c.jit->bind(c);
        c.jit->tscOn = c.jitTscOn;
        if (!c.jit->ready()) {
            c.jitOn = false; // no executable memory: interpreter it is
            return false;
        }
    }
    JitCache& J = *c.jit;
    const u64 tsc0 = J.tscOn ? __rdtsc() : 0;
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
    c.jitUntil = until; // the trampoline pins it in rbp for the chain hops
    const u64 before = stamp;
    if (J.tscOn) {
        // The split the report's "cycles per …" lines are built from:
        // tscProbe is everything this function did to find the block (the
        // dispatcher's own overhead), tscNative is the emitted code itself.
        const u64 t1 = __rdtsc();
        J.tscProbe += t1 - tsc0;
        J.enterFn(J.arena_ + jl->off[word], &c, &stamp,
                  u64(c.insnCycles) + c.extraCycles);
        J.tscNative += __rdtsc() - t1;
    } else {
        J.enterFn(J.arena_ + jl->off[word], &c, &stamp,
                  u64(c.insnCycles) + c.extraCycles);
    }
    J.insns += stamp - before;
    return true;
#else
    (void)c; (void)slot; (void)word; (void)stamp; (void)until;
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
            ++c.jit->refillKeeps; // identical bytes: block AND its links live
        } else {
            c.jit->sever(jl); // unlink chains into it BEFORE it stops being
            jl.base = 1u;     // the block for these bytes
            ++c.jit->refillDrops;
        }
    }
}

} // namespace opm

