// Integer, branch, CR, load/store, and system-plumbing executors (P1).
// Semantics per the PEM instruction chapter; 7400 specifics per the UM.
//
// Deterministic choices for architecturally-undefined results are marked
// RECEIPT — to be pinned against silicon if hardware capture happens:
//   - divw/divwu overflow and divide-by-zero produce rD = 0.

#include "opm/cpu.hpp"
#include "opm/bits.hpp"
#include "softfp.hpp"
#include <bit>
#include <string>

namespace opm {

namespace {

// ---- small helpers ---------------------------------------------------------

u32 gpr0(Cpu& c, u32 ra) { return ra ? c.st.gpr[ra] : 0; }

u32 eaD(Cpu& c, u32 insn) { return gpr0(c, f_ra(insn)) + static_cast<u32>(sext16(f_d(insn))); }
u32 eaX(Cpu& c, u32 insn) { return gpr0(c, f_ra(insn)) + c.st.gpr[f_rb(insn)]; }

void recordRc(Cpu& c, u32 insn, u32 val)
{
    if (f_rcbit(insn))
        c.setCr0(val);
}

// a + b + cin with carry-out.
u32 addc3(Cpu& c, u32 a, u32 b, u32 cin, bool setCa, bool setOv, u32 insn)
{
    const u64 s = static_cast<u64>(a) + b + cin;
    const u32 r = static_cast<u32>(s);
    if (setCa)
        c.setCa((s >> 32) != 0);
    if (setOv && f_oebit(insn))
        c.setOv((((a ^ r) & (b ^ r)) >> 31) != 0);
    return r;
}

bool crBit(const Cpu& c, u32 bit) { return ((c.st.cr >> (31u - bit)) & 1u) != 0; }
void setCrBit(Cpu& c, u32 bit, bool v)
{
    const u32 m = 1u << (31u - bit);
    c.st.cr = v ? (c.st.cr | m) : (c.st.cr & ~m);
}

void cmpS(Cpu& c, u32 crf, i32 a, i32 b)
{
    u32 f = (a < b) ? 8u : (a > b) ? 4u : 2u;
    f |= (c.st.xer >> 31) & 1u;
    c.setCrField(crf, f);
}
void cmpU(Cpu& c, u32 crf, u32 a, u32 b)
{
    u32 f = (a < b) ? 8u : (a > b) ? 4u : 2u;
    f |= (c.st.xer >> 31) & 1u;
    c.setCrField(crf, f);
}

bool trapCond(u32 to, i32 a, i32 b)
{
    if ((to & 16u) && a < b) return true;
    if ((to & 8u) && a > b) return true;
    if ((to & 4u) && a == b) return true;
    if ((to & 2u) && static_cast<u32>(a) < static_cast<u32>(b)) return true;
    if ((to & 1u) && static_cast<u32>(a) > static_cast<u32>(b)) return true;
    return false;
}

// ---- arithmetic (D-form) ---------------------------------------------------

void h_addi(Cpu& c, u32 i, const InsnDesc&)
{
    c.st.gpr[f_rt(i)] = gpr0(c, f_ra(i)) + static_cast<u32>(sext16(f_d(i)));
}
void h_addis(Cpu& c, u32 i, const InsnDesc&)
{
    c.st.gpr[f_rt(i)] = gpr0(c, f_ra(i)) + (f_d(i) << 16);
}
void h_addic(Cpu& c, u32 i, const InsnDesc&)
{
    c.st.gpr[f_rt(i)] = addc3(c, c.st.gpr[f_ra(i)], static_cast<u32>(sext16(f_d(i))), 0, true, false, i);
}
void h_addicRc(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 r = addc3(c, c.st.gpr[f_ra(i)], static_cast<u32>(sext16(f_d(i))), 0, true, false, i);
    c.st.gpr[f_rt(i)] = r;
    c.setCr0(r);
}
void h_subfic(Cpu& c, u32 i, const InsnDesc&)
{
    c.st.gpr[f_rt(i)] = addc3(c, ~c.st.gpr[f_ra(i)], static_cast<u32>(sext16(f_d(i))), 1, true, false, i);
}
void h_mulli(Cpu& c, u32 i, const InsnDesc&)
{
    c.st.gpr[f_rt(i)] = static_cast<u32>(
        static_cast<i64>(static_cast<i32>(c.st.gpr[f_ra(i)])) * sext16(f_d(i)));
}

// ---- logical immediates ----------------------------------------------------

void h_ori(Cpu& c, u32 i, const InsnDesc&)   { c.st.gpr[f_ra(i)] = c.st.gpr[f_rt(i)] | f_d(i); }
void h_oris(Cpu& c, u32 i, const InsnDesc&)  { c.st.gpr[f_ra(i)] = c.st.gpr[f_rt(i)] | (f_d(i) << 16); }
void h_xori(Cpu& c, u32 i, const InsnDesc&)  { c.st.gpr[f_ra(i)] = c.st.gpr[f_rt(i)] ^ f_d(i); }
void h_xoris(Cpu& c, u32 i, const InsnDesc&) { c.st.gpr[f_ra(i)] = c.st.gpr[f_rt(i)] ^ (f_d(i) << 16); }
void h_andiRc(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 r = c.st.gpr[f_rt(i)] & f_d(i);
    c.st.gpr[f_ra(i)] = r;
    c.setCr0(r);
}
void h_andisRc(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 r = c.st.gpr[f_rt(i)] & (f_d(i) << 16);
    c.st.gpr[f_ra(i)] = r;
    c.setCr0(r);
}

// ---- compares / traps ------------------------------------------------------

void h_cmpi(Cpu& c, u32 i, const InsnDesc&)  { cmpS(c, f_crfd(i), static_cast<i32>(c.st.gpr[f_ra(i)]), sext16(f_d(i))); }
void h_cmpli(Cpu& c, u32 i, const InsnDesc&) { cmpU(c, f_crfd(i), c.st.gpr[f_ra(i)], f_d(i)); }
void h_cmp(Cpu& c, u32 i, const InsnDesc&)   { cmpS(c, f_crfd(i), static_cast<i32>(c.st.gpr[f_ra(i)]), static_cast<i32>(c.st.gpr[f_rb(i)])); }
void h_cmpl(Cpu& c, u32 i, const InsnDesc&)  { cmpU(c, f_crfd(i), c.st.gpr[f_ra(i)], c.st.gpr[f_rb(i)]); }
void h_twi(Cpu& c, u32 i, const InsnDesc&)
{
    if (trapCond(f_to(i), static_cast<i32>(c.st.gpr[f_ra(i)]), sext16(f_d(i))))
        c.raiseExc(Exc::Program, c.st.pc - 4, kSrr1ProgTrap);
}
void h_tw(Cpu& c, u32 i, const InsnDesc&)
{
    if (trapCond(f_to(i), static_cast<i32>(c.st.gpr[f_ra(i)]), static_cast<i32>(c.st.gpr[f_rb(i)])))
        c.raiseExc(Exc::Program, c.st.pc - 4, kSrr1ProgTrap);
}

// ---- rotates ---------------------------------------------------------------

u32 rotl(u32 v, u32 n) { return std::rotl(v, static_cast<int>(n & 31u)); }

void h_rlwinm(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 r = rotl(c.st.gpr[f_rt(i)], f_sh(i)) & ppcmask(f_mb(i), f_me(i));
    c.st.gpr[f_ra(i)] = r;
    recordRc(c, i, r);
}
void h_rlwnm(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 r = rotl(c.st.gpr[f_rt(i)], c.st.gpr[f_rb(i)] & 31u) & ppcmask(f_mb(i), f_me(i));
    c.st.gpr[f_ra(i)] = r;
    recordRc(c, i, r);
}
void h_rlwimi(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 m = ppcmask(f_mb(i), f_me(i));
    const u32 r = (rotl(c.st.gpr[f_rt(i)], f_sh(i)) & m) | (c.st.gpr[f_ra(i)] & ~m);
    c.st.gpr[f_ra(i)] = r;
    recordRc(c, i, r);
}

// ---- branches --------------------------------------------------------------

void h_b(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 cia = c.st.pc - 4;
    const i32 disp = sext26(ppcbits(i, 6, 29) << 2);
    if (f_lkbit(i))
        c.st.lr = c.st.pc;
    c.st.pc = f_aabit(i) ? static_cast<u32>(disp) : cia + static_cast<u32>(disp);
}

bool bcTaken(Cpu& c, u32 bo, u32 bi)
{
    if (!(bo & 4u))
        --c.st.ctr;
    const bool ctrOk = (bo & 4u) || ((c.st.ctr != 0) != ((bo & 2u) != 0));
    const bool condOk = (bo & 16u) || (crBit(c, bi) == ((bo & 8u) != 0));
    return ctrOk && condOk;
}

void h_bc(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 cia = c.st.pc - 4;
    const bool taken = bcTaken(c, ppcbits(i, 6, 10), ppcbits(i, 11, 15));
    if (f_lkbit(i))
        c.st.lr = c.st.pc; // LR is written whether or not the branch is taken
    if (taken) {
        const i32 disp = sext14(ppcbits(i, 16, 29)) * 4;
        c.st.pc = f_aabit(i) ? static_cast<u32>(disp) : cia + static_cast<u32>(disp);
    }
}
void h_bclr(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 target = c.st.lr & ~3u;
    const bool taken = bcTaken(c, ppcbits(i, 6, 10), ppcbits(i, 11, 15));
    if (f_lkbit(i))
        c.st.lr = c.st.pc;
    if (taken)
        c.st.pc = target;
}
void h_bcctr(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 bo = ppcbits(i, 6, 10);
    const bool condOk = (bo & 16u) || (crBit(c, ppcbits(i, 11, 15)) == ((bo & 8u) != 0));
    if (f_lkbit(i))
        c.st.lr = c.st.pc;
    if (condOk)
        c.st.pc = c.st.ctr & ~3u;
}

void h_sc(Cpu& c, u32, const InsnDesc&)
{
    // SRR0 gets the address of the instruction AFTER sc.
    c.raiseExc(Exc::SystemCall, c.st.pc, 0);
}
void h_rfi(Cpu& c, u32, const InsnDesc&)
{
    c.st.msr = c.st.srr1 & msr::VALID;
    c.st.pc = c.st.srr0 & ~3u;
}

// ---- XO-form arithmetic ----------------------------------------------------

void h_add(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 r = addc3(c, c.st.gpr[f_ra(i)], c.st.gpr[f_rb(i)], 0, false, true, i);
    c.st.gpr[f_rt(i)] = r;
    recordRc(c, i, r);
}
void h_addc(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 r = addc3(c, c.st.gpr[f_ra(i)], c.st.gpr[f_rb(i)], 0, true, true, i);
    c.st.gpr[f_rt(i)] = r;
    recordRc(c, i, r);
}
void h_adde(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 r = addc3(c, c.st.gpr[f_ra(i)], c.st.gpr[f_rb(i)], c.ca() ? 1u : 0u, true, true, i);
    c.st.gpr[f_rt(i)] = r;
    recordRc(c, i, r);
}
void h_addme(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 r = addc3(c, c.st.gpr[f_ra(i)], 0xFFFFFFFFu, c.ca() ? 1u : 0u, true, true, i);
    c.st.gpr[f_rt(i)] = r;
    recordRc(c, i, r);
}
void h_addze(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 r = addc3(c, c.st.gpr[f_ra(i)], 0, c.ca() ? 1u : 0u, true, true, i);
    c.st.gpr[f_rt(i)] = r;
    recordRc(c, i, r);
}
void h_subf(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 r = addc3(c, ~c.st.gpr[f_ra(i)], c.st.gpr[f_rb(i)], 1, false, true, i);
    c.st.gpr[f_rt(i)] = r;
    recordRc(c, i, r);
}
void h_subfc(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 r = addc3(c, ~c.st.gpr[f_ra(i)], c.st.gpr[f_rb(i)], 1, true, true, i);
    c.st.gpr[f_rt(i)] = r;
    recordRc(c, i, r);
}
void h_subfe(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 r = addc3(c, ~c.st.gpr[f_ra(i)], c.st.gpr[f_rb(i)], c.ca() ? 1u : 0u, true, true, i);
    c.st.gpr[f_rt(i)] = r;
    recordRc(c, i, r);
}
void h_subfme(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 r = addc3(c, ~c.st.gpr[f_ra(i)], 0xFFFFFFFFu, c.ca() ? 1u : 0u, true, true, i);
    c.st.gpr[f_rt(i)] = r;
    recordRc(c, i, r);
}
void h_subfze(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 r = addc3(c, ~c.st.gpr[f_ra(i)], 0, c.ca() ? 1u : 0u, true, true, i);
    c.st.gpr[f_rt(i)] = r;
    recordRc(c, i, r);
}
void h_neg(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 a = c.st.gpr[f_ra(i)];
    const u32 r = 0u - a;
    if (f_oebit(i))
        c.setOv(a == 0x80000000u);
    c.st.gpr[f_rt(i)] = r;
    recordRc(c, i, r);
}
void h_mullw(Cpu& c, u32 i, const InsnDesc&)
{
    const i64 p = static_cast<i64>(static_cast<i32>(c.st.gpr[f_ra(i)])) *
                  static_cast<i32>(c.st.gpr[f_rb(i)]);
    const u32 r = static_cast<u32>(p);
    if (f_oebit(i))
        c.setOv(p != static_cast<i32>(r));
    c.st.gpr[f_rt(i)] = r;
    recordRc(c, i, r);
}
void h_mulhw(Cpu& c, u32 i, const InsnDesc&)
{
    const i64 p = static_cast<i64>(static_cast<i32>(c.st.gpr[f_ra(i)])) *
                  static_cast<i32>(c.st.gpr[f_rb(i)]);
    const u32 r = static_cast<u32>(static_cast<u64>(p) >> 32);
    c.st.gpr[f_rt(i)] = r;
    recordRc(c, i, r);
}
void h_mulhwu(Cpu& c, u32 i, const InsnDesc&)
{
    const u64 p = static_cast<u64>(c.st.gpr[f_ra(i)]) * c.st.gpr[f_rb(i)];
    const u32 r = static_cast<u32>(p >> 32);
    c.st.gpr[f_rt(i)] = r;
    recordRc(c, i, r);
}
void h_divw(Cpu& c, u32 i, const InsnDesc&)
{
    const i32 a = static_cast<i32>(c.st.gpr[f_ra(i)]);
    const i32 b = static_cast<i32>(c.st.gpr[f_rb(i)]);
    const bool bad = (b == 0) || (a == static_cast<i32>(0x80000000u) && b == -1);
    const u32 r = bad ? 0u : static_cast<u32>(a / b); // RECEIPT: undefined result -> 0
    if (f_oebit(i))
        c.setOv(bad);
    c.st.gpr[f_rt(i)] = r;
    recordRc(c, i, r);
}
void h_divwu(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 a = c.st.gpr[f_ra(i)];
    const u32 b = c.st.gpr[f_rb(i)];
    const bool bad = b == 0;
    const u32 r = bad ? 0u : a / b; // RECEIPT: undefined result -> 0
    if (f_oebit(i))
        c.setOv(bad);
    c.st.gpr[f_rt(i)] = r;
    recordRc(c, i, r);
}

// ---- X-form logicals / shifts ---------------------------------------------

#define LOGIC(NAME, EXPR)                                                     \
    void NAME(Cpu& c, u32 i, const InsnDesc&)                                 \
    {                                                                         \
        const u32 s = c.st.gpr[f_rt(i)];                                      \
        const u32 b = c.st.gpr[f_rb(i)];                                      \
        (void)b;                                                              \
        const u32 r = (EXPR);                                                 \
        c.st.gpr[f_ra(i)] = r;                                                \
        recordRc(c, i, r);                                                    \
    }

LOGIC(h_and, s & b)
LOGIC(h_andcx, s & ~b)
LOGIC(h_or, s | b)
LOGIC(h_orc, s | ~b)
LOGIC(h_xor, s ^ b)
LOGIC(h_eqv, ~(s ^ b))
LOGIC(h_nor, ~(s | b))
LOGIC(h_nand, ~(s & b))
#undef LOGIC

void h_slw(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 n = c.st.gpr[f_rb(i)] & 63u;
    const u32 r = n >= 32 ? 0u : c.st.gpr[f_rt(i)] << n;
    c.st.gpr[f_ra(i)] = r;
    recordRc(c, i, r);
}
void h_srw(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 n = c.st.gpr[f_rb(i)] & 63u;
    const u32 r = n >= 32 ? 0u : c.st.gpr[f_rt(i)] >> n;
    c.st.gpr[f_ra(i)] = r;
    recordRc(c, i, r);
}
void h_sraw(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 n = c.st.gpr[f_rb(i)] & 63u;
    const i32 s = static_cast<i32>(c.st.gpr[f_rt(i)]);
    u32 r;
    bool ca;
    if (n >= 32) {
        r = static_cast<u32>(s >> 31);
        ca = s < 0;
    } else {
        r = static_cast<u32>(s >> n);
        ca = s < 0 && n != 0 && (static_cast<u32>(s) & ((1u << n) - 1u)) != 0;
    }
    c.setCa(ca);
    c.st.gpr[f_ra(i)] = r;
    recordRc(c, i, r);
}
void h_srawi(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 n = f_sh(i);
    const i32 s = static_cast<i32>(c.st.gpr[f_rt(i)]);
    const u32 r = static_cast<u32>(s >> n);
    c.setCa(s < 0 && n != 0 && (static_cast<u32>(s) & ((1u << n) - 1u)) != 0);
    c.st.gpr[f_ra(i)] = r;
    recordRc(c, i, r);
}
void h_cntlzw(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 r = static_cast<u32>(std::countl_zero(c.st.gpr[f_rt(i)]));
    c.st.gpr[f_ra(i)] = r;
    recordRc(c, i, r);
}
void h_extsb(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 r = static_cast<u32>(static_cast<i32>(static_cast<i8>(c.st.gpr[f_rt(i)])));
    c.st.gpr[f_ra(i)] = r;
    recordRc(c, i, r);
}
void h_extsh(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 r = static_cast<u32>(static_cast<i32>(static_cast<i16>(c.st.gpr[f_rt(i)])));
    c.st.gpr[f_ra(i)] = r;
    recordRc(c, i, r);
}

// ---- loads / stores --------------------------------------------------------

// EAKIND is eaD or eaX; READER a readV* member; XF maps the raw value v to
// the register value. A translation fault bails before any register write.
#define LOAD(NAME, EAKIND, READER, XF)                                        \
    void NAME(Cpu& c, u32 i, const InsnDesc&)                                 \
    {                                                                         \
        u32 v;                                                                \
        if (!c.READER(EAKIND(c, i), v))                                       \
            return;                                                           \
        c.st.gpr[f_rt(i)] = (XF);                                             \
    }

LOAD(h_lwz, eaD, readV32, v)
LOAD(h_lbz, eaD, readV8, v)
LOAD(h_lhz, eaD, readV16, v)
LOAD(h_lha, eaD, readV16, static_cast<u32>(sext16(v)))
LOAD(h_lwzx, eaX, readV32, v)
LOAD(h_lbzx, eaX, readV8, v)
LOAD(h_lhzx, eaX, readV16, v)
LOAD(h_lhax, eaX, readV16, static_cast<u32>(sext16(v)))
#undef LOAD

void updRa(Cpu& c, u32 insn, u32 ea) { c.st.gpr[f_ra(insn)] = ea; }

// RECEIPT: DSISR image fields that are architecturally undefined for a form
// (dcbz's [22-26], non-update [27-31]) are filled from the image anyway.
void raiseAlign(Cpu& c, u32 insn, u32 ea)
{
    c.st.dar = ea;
    c.st.dsisr = alignDsisr(insn);
    c.raiseExc(Exc::Alignment, c.st.pc - 4, 0);
}

// Update forms write neither rD nor rA when the access faults.
#define LOADU(NAME, EAKIND, READER, XF)                                       \
    void NAME(Cpu& c, u32 i, const InsnDesc&)                                 \
    {                                                                         \
        const u32 ea = EAKIND(c, i);                                          \
        u32 v;                                                                \
        if (!c.READER(ea, v))                                                 \
            return;                                                           \
        c.st.gpr[f_rt(i)] = (XF);                                             \
        updRa(c, i, ea);                                                      \
    }

LOADU(h_lwzu, eaD, readV32, v)
LOADU(h_lbzu, eaD, readV8, v)
LOADU(h_lhzu, eaD, readV16, v)
LOADU(h_lhau, eaD, readV16, static_cast<u32>(sext16(v)))
LOADU(h_lwzux, eaX, readV32, v)
LOADU(h_lbzux, eaX, readV8, v)
LOADU(h_lhzux, eaX, readV16, v)
LOADU(h_lhaux, eaX, readV16, static_cast<u32>(sext16(v)))
#undef LOADU

void h_stw(Cpu& c, u32 i, const InsnDesc&)   { c.writeV32(eaD(c, i), c.st.gpr[f_rt(i)]); }
void h_stb(Cpu& c, u32 i, const InsnDesc&)   { c.writeV8(eaD(c, i), c.st.gpr[f_rt(i)]); }
void h_sth(Cpu& c, u32 i, const InsnDesc&)   { c.writeV16(eaD(c, i), c.st.gpr[f_rt(i)]); }
void h_stwx(Cpu& c, u32 i, const InsnDesc&)  { c.writeV32(eaX(c, i), c.st.gpr[f_rt(i)]); }
void h_stbx(Cpu& c, u32 i, const InsnDesc&)  { c.writeV8(eaX(c, i), c.st.gpr[f_rt(i)]); }
void h_sthx(Cpu& c, u32 i, const InsnDesc&)  { c.writeV16(eaX(c, i), c.st.gpr[f_rt(i)]); }
void h_stwu(Cpu& c, u32 i, const InsnDesc&)  { const u32 ea = eaD(c, i); if (c.writeV32(ea, c.st.gpr[f_rt(i)])) updRa(c, i, ea); }
void h_stbu(Cpu& c, u32 i, const InsnDesc&)  { const u32 ea = eaD(c, i); if (c.writeV8(ea, c.st.gpr[f_rt(i)])) updRa(c, i, ea); }
void h_sthu(Cpu& c, u32 i, const InsnDesc&)  { const u32 ea = eaD(c, i); if (c.writeV16(ea, c.st.gpr[f_rt(i)])) updRa(c, i, ea); }
void h_stwux(Cpu& c, u32 i, const InsnDesc&) { const u32 ea = eaX(c, i); if (c.writeV32(ea, c.st.gpr[f_rt(i)])) updRa(c, i, ea); }
void h_stbux(Cpu& c, u32 i, const InsnDesc&) { const u32 ea = eaX(c, i); if (c.writeV8(ea, c.st.gpr[f_rt(i)])) updRa(c, i, ea); }
void h_sthux(Cpu& c, u32 i, const InsnDesc&) { const u32 ea = eaX(c, i); if (c.writeV16(ea, c.st.gpr[f_rt(i)])) updRa(c, i, ea); }

void h_lwbrx(Cpu& c, u32 i, const InsnDesc&)
{
    u32 v;
    if (!c.readV32(eaX(c, i), v))
        return;
    c.st.gpr[f_rt(i)] = ((v & 0xFFu) << 24) | ((v & 0xFF00u) << 8) |
                        ((v >> 8) & 0xFF00u) | (v >> 24);
}
void h_lhbrx(Cpu& c, u32 i, const InsnDesc&)
{
    u32 v;
    if (!c.readV16(eaX(c, i), v))
        return;
    c.st.gpr[f_rt(i)] = ((v & 0xFFu) << 8) | ((v >> 8) & 0xFFu);
}
void h_stwbrx(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 v = c.st.gpr[f_rt(i)];
    c.writeV32(eaX(c, i), ((v & 0xFFu) << 24) | ((v & 0xFF00u) << 8) |
                              ((v >> 8) & 0xFF00u) | (v >> 24));
}
void h_sthbrx(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 v = c.st.gpr[f_rt(i)];
    c.writeV16(eaX(c, i), ((v & 0xFFu) << 8) | ((v >> 8) & 0xFFu));
}

// Multi-access instructions perform their accesses in sequence; a fault
// partway leaves earlier accesses done (architecturally permitted — the
// instruction restarts from scratch after the handler resolves the fault).
void h_lmw(Cpu& c, u32 i, const InsnDesc&)
{
    u32 ea = eaD(c, i);
    if ((ea & 3u) || (c.st.msr & msr::LE)) { // LE: multiples always fault
        raiseAlign(c, i, ea);
        return;
    }
    for (u32 r = f_rt(i); r <= 31; ++r, ea += 4) {
        u32 v;
        if (!c.readV32(ea, v))
            return;
        c.st.gpr[r] = v;
    }
}
void h_stmw(Cpu& c, u32 i, const InsnDesc&)
{
    u32 ea = eaD(c, i);
    if ((ea & 3u) || (c.st.msr & msr::LE)) {
        raiseAlign(c, i, ea);
        return;
    }
    for (u32 r = f_rt(i); r <= 31; ++r, ea += 4)
        if (!c.writeV32(ea, c.st.gpr[r]))
            return;
}

void loadString(Cpu& c, u32 rt, u32 ea, u32 n)
{
    u32 r = rt;
    u32 sh = 24;
    if (n)
        c.st.gpr[r] = 0;
    while (n--) {
        u32 v;
        if (!c.readV8(ea++, v))
            return;
        c.st.gpr[r] |= v << sh;
        if (sh == 0) {
            sh = 24;
            r = (r + 1) & 31u;
            if (n)
                c.st.gpr[r] = 0;
        } else {
            sh -= 8;
        }
    }
}
void storeString(Cpu& c, u32 rs, u32 ea, u32 n)
{
    u32 r = rs;
    u32 sh = 24;
    while (n--) {
        if (!c.writeV8(ea++, c.st.gpr[r] >> sh))
            return;
        if (sh == 0) {
            sh = 24;
            r = (r + 1) & 31u;
        } else {
            sh -= 8;
        }
    }
}
// String ops in little-endian mode raise alignment (UM 4.6.6, verified).
bool leStringFault(Cpu& c, u32 i, u32 ea)
{
    if (!(c.st.msr & msr::LE))
        return false;
    raiseAlign(c, i, ea);
    return true;
}
void h_lswi(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 ea = gpr0(c, f_ra(i));
    if (leStringFault(c, i, ea))
        return;
    const u32 n = f_nb(i) ? f_nb(i) : 32u;
    loadString(c, f_rt(i), ea, n);
}
void h_stswi(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 ea = gpr0(c, f_ra(i));
    if (leStringFault(c, i, ea))
        return;
    const u32 n = f_nb(i) ? f_nb(i) : 32u;
    storeString(c, f_rt(i), ea, n);
}
void h_lswx(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 ea = eaX(c, i);
    if (leStringFault(c, i, ea))
        return;
    loadString(c, f_rt(i), ea, c.st.xer & 0x7Fu);
}
void h_stswx(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 ea = eaX(c, i);
    if (leStringFault(c, i, ea))
        return;
    storeString(c, f_rt(i), ea, c.st.xer & 0x7Fu);
}

// ---- FP loads / stores -----------------------------------------------------
// The 7400 raises an alignment exception for any FP load/store whose EA is
// not word-aligned (UM 4.6.6); a word-aligned lfd/stfd splits in hardware.
// Single forms convert via the PEM D.6/D.7 models.

#define FLOAD(NAME, EAKIND, CONV, LEN, UPD)                                   \
    void NAME(Cpu& c, u32 i, const InsnDesc&)                                 \
    {                                                                         \
        const u32 ea = EAKIND(c, i);                                          \
        if (ea & 3u) {                                                        \
            raiseAlign(c, i, ea);                                             \
            return;                                                           \
        }                                                                     \
        u64 v;                                                                \
        if (!c.readV(ea, LEN, v))                                             \
            return;                                                           \
        c.st.fpr[f_rt(i)] = CONV;                                             \
        if (UPD)                                                              \
            updRa(c, i, ea);                                                  \
    }

FLOAD(h_lfs, eaD, sf::loadSingle(static_cast<u32>(v)), 4, false)
FLOAD(h_lfsu, eaD, sf::loadSingle(static_cast<u32>(v)), 4, true)
FLOAD(h_lfsx, eaX, sf::loadSingle(static_cast<u32>(v)), 4, false)
FLOAD(h_lfsux, eaX, sf::loadSingle(static_cast<u32>(v)), 4, true)
FLOAD(h_lfd, eaD, v, 8, false)
FLOAD(h_lfdu, eaD, v, 8, true)
FLOAD(h_lfdx, eaX, v, 8, false)
FLOAD(h_lfdux, eaX, v, 8, true)
#undef FLOAD

#define FSTORE(NAME, EAKIND, VAL, LEN, UPD)                                   \
    void NAME(Cpu& c, u32 i, const InsnDesc&)                                 \
    {                                                                         \
        const u32 ea = EAKIND(c, i);                                          \
        if (ea & 3u) {                                                        \
            raiseAlign(c, i, ea);                                             \
            return;                                                           \
        }                                                                     \
        if (!c.writeV(ea, LEN, VAL))                                          \
            return;                                                           \
        if (UPD)                                                              \
            updRa(c, i, ea);                                                  \
    }

FSTORE(h_stfs, eaD, sf::storeSingle(c.st.fpr[f_rt(i)]), 4, false)
FSTORE(h_stfsu, eaD, sf::storeSingle(c.st.fpr[f_rt(i)]), 4, true)
FSTORE(h_stfsx, eaX, sf::storeSingle(c.st.fpr[f_rt(i)]), 4, false)
FSTORE(h_stfsux, eaX, sf::storeSingle(c.st.fpr[f_rt(i)]), 4, true)
FSTORE(h_stfd, eaD, c.st.fpr[f_rt(i)], 8, false)
FSTORE(h_stfdu, eaD, c.st.fpr[f_rt(i)], 8, true)
FSTORE(h_stfdx, eaX, c.st.fpr[f_rt(i)], 8, false)
FSTORE(h_stfdux, eaX, c.st.fpr[f_rt(i)], 8, true)
FSTORE(h_stfiwx, eaX, c.st.fpr[f_rt(i)] & 0xFFFFFFFFull, 4, false)
#undef FSTORE

void h_lwarx(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 ea = eaX(c, i);
    if (ea & 3u) {
        raiseAlign(c, i, ea);
        return;
    }
    u32 pa;
    if (!c.translate(ea, false, false, pa))
        return; // faulting lwarx establishes no reservation
    c.st.resvValid = true;
    c.st.resvAddr = pa & ~31u; // reservation granule is physical (bus-snooped)
    c.st.gpr[f_rt(i)] = c.bus->read32(pa);
}
void h_stwcx(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 ea = eaX(c, i);
    if (ea & 3u) {
        raiseAlign(c, i, ea);
        return;
    }
    // Translation/protection is checked even when no reservation exists (PEM
    // 7.6.3: the protection-violation row applies to a non-storing stwcx.).
    // A DSI here leaves the reservation intact — the instruction restarts.
    // RECEIPT: a translating-but-not-storing stwcx. sets R and C via the
    // table walk; Table 7-17 row 8 permits this ("Maybe").
    u32 pa;
    if (!c.translate(ea, true, false, pa))
        return;
    // UM: the reservation is non-specific with respect to this processor —
    // a stwcx. succeeds (and always clears) regardless of address match.
    const bool ok = c.st.resvValid;
    if (ok)
        c.bus->write32(pa, c.st.gpr[f_rt(i)]);
    c.st.resvValid = false;
    c.setCrField(0, (ok ? 2u : 0u) | ((c.st.xer >> 31) & 1u));
}

// External control: word-aligned only (UM 4.6.6); EAR[E]=0 raises a DSI
// with DSISR[11] (UM Table 5-4). RECEIPT: with EAR[E]=1 the access behaves
// as a normal translated load/store — there is no external-control device
// at the core level, so the system bus is the device.
void h_eciwx(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 ea = eaX(c, i);
    if (ea & 3u) {
        raiseAlign(c, i, ea);
        return;
    }
    if (!(c.st.ear & 0x80000000u)) {
        c.st.dar = ea;
        c.st.dsisr = 0x00100000u;
        c.raiseExc(Exc::Dsi, c.st.pc - 4, 0);
        return;
    }
    u32 v;
    if (!c.readV32(ea, v))
        return;
    c.st.gpr[f_rt(i)] = v;
}
void h_ecowx(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 ea = eaX(c, i);
    if (ea & 3u) {
        raiseAlign(c, i, ea);
        return;
    }
    if (!(c.st.ear & 0x80000000u)) {
        c.st.dar = ea;
        c.st.dsisr = 0x00100000u | 0x02000000u;
        c.raiseExc(Exc::Dsi, c.st.pc - 4, 0);
        return;
    }
    c.writeV32(ea, c.st.gpr[f_rt(i)]);
}

// ---- CR field ops ----------------------------------------------------------

void h_mcrf(Cpu& c, u32 i, const InsnDesc&) { c.setCrField(f_crfd(i), c.crField(f_crfs(i))); }

#define CROP(NAME, EXPR)                                                      \
    void NAME(Cpu& c, u32 i, const InsnDesc&)                                 \
    {                                                                         \
        const bool a = crBit(c, f_ra(i));                                     \
        const bool b = crBit(c, f_rb(i));                                     \
        (void)b;                                                              \
        setCrBit(c, f_rt(i), (EXPR));                                         \
    }
CROP(h_crand, a && b)
CROP(h_cror, a || b)
CROP(h_crxor, a != b)
CROP(h_crnand, !(a && b))
CROP(h_crnor, !(a || b))
CROP(h_creqv, a == b)
CROP(h_crandc, a && !b)
CROP(h_crorc, a || !b)
#undef CROP

void h_mfcr(Cpu& c, u32 i, const InsnDesc&) { c.st.gpr[f_rt(i)] = c.st.cr; }
void h_mtcrf(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 crm = f_crm(i);
    const u32 v = c.st.gpr[f_rt(i)];
    for (u32 f = 0; f < 8; ++f) {
        if (crm & (0x80u >> f)) {
            const u32 sh = (7u - f) * 4u;
            c.st.cr = (c.st.cr & ~(0xFu << sh)) | (v & (0xFu << sh));
        }
    }
}
void h_mcrxr(Cpu& c, u32 i, const InsnDesc&)
{
    c.setCrField(f_crfd(i), c.st.xer >> 28);
    c.st.xer &= 0x0FFFFFFFu;
}

// ---- SPR / MSR / SR moves --------------------------------------------------

u32* sprPtr(Cpu& c, u32 spr)
{
    CpuState& s = c.st;
    switch (spr) {
    case 1: return &s.xer;
    case 8: return &s.lr;
    case 9: return &s.ctr;
    case 18: return &s.dsisr;
    case 19: return &s.dar;
    case 22: return &s.dec;
    case 25: return &s.sdr1;
    case 26: return &s.srr0;
    case 27: return &s.srr1;
    case 256: return &s.vrsave;
    case 272: case 273: case 274: case 275: return &s.sprg[spr - 272];
    case 282: return &s.ear;
    case 287: return &s.pvr;
    case 528: case 530: case 532: case 534: return &s.ibatu[(spr - 528) / 2];
    case 529: case 531: case 533: case 535: return &s.ibatl[(spr - 529) / 2];
    case 536: case 538: case 540: case 542: return &s.dbatu[(spr - 536) / 2];
    case 537: case 539: case 541: case 543: return &s.dbatl[(spr - 537) / 2];
    case 936: case 952: return &s.mmcr0;
    case 940: case 956: return &s.mmcr1;
    case 937: case 953: return &s.pmc[0];
    case 938: case 954: return &s.pmc[1];
    case 941: case 957: return &s.pmc[2];
    case 942: case 958: return &s.pmc[3];
    case 939: case 955: return &s.siar;
    case 943: case 959: return &s.sdar;
    case 935: return &s.bamr; // UBAMR: user-level read access to BAMR
    case 951: return &s.bamr;
    case 1008: return &s.hid0;
    case 1009: return &s.hid1;
    case 1010: return &s.iabr;
    case 1013: return &s.dabr;
    case 1014: return &s.msscr0;
    case 1015: return &s.msscr1;
    case 1017: return &s.l2cr;
    case 1019: return &s.ictc;
    case 1020: case 1021: case 1022: return &s.thrm[spr - 1020];
    case 1023: return &s.pir;
    default: return nullptr;
    }
}

// User-mode readable SPRs; everything else is supervisor-only.
bool sprUserReadable(u32 spr)
{
    switch (spr) {
    case 1: case 8: case 9: case 256: // XER/LR/CTR/VRSAVE
    case 935: // UBAMR
    case 936: case 937: case 938: case 939: case 940: case 941: case 942:
    case 943: // performance-monitor user copies
        return true;
    default:
        return false;
    }
}
bool sprUserWritable(u32 spr)
{
    return spr == 1 || spr == 8 || spr == 9 || spr == 256;
}

// User access to an UNDEFINED SPR: privileged exception if the encoded
// SPR[0] bit (n & 0x10) is set, illegal otherwise (PEM 6.4.7).
bool sprAccessFault(Cpu& c, u32 spr, bool known, bool userAllowed)
{
    if (c.userMode()) {
        if (!known && !(spr & 0x10u)) {
            c.raiseExc(Exc::Program, c.st.pc - 4, kSrr1ProgIllegal);
            return true;
        }
        if (!userAllowed) {
            c.raiseExc(Exc::Program, c.st.pc - 4, kSrr1ProgPrivileged);
            return true;
        }
    }
    if (!known) {
        c.raiseExc(Exc::Program, c.st.pc - 4, kSrr1ProgIllegal);
        return true;
    }
    return false;
}

void h_mfspr(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 spr = f_spr(i);
    const bool tb = spr == 268 || spr == 269;
    u32* p = tb ? nullptr : sprPtr(c, spr);
    if (sprAccessFault(c, spr, tb || p != nullptr,
                       tb || sprUserReadable(spr)))
        return;
    if (spr == 268) { c.st.gpr[f_rt(i)] = static_cast<u32>(c.st.tb); return; }
    if (spr == 269) { c.st.gpr[f_rt(i)] = static_cast<u32>(c.st.tb >> 32); return; }
    c.st.gpr[f_rt(i)] = *p;
}
void h_mtspr(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 spr = f_spr(i);
    const u32 v = c.st.gpr[f_rt(i)];
    const bool special = spr == 22 || spr == 284 || spr == 285 || spr == 287;
    if (sprAccessFault(c, spr,
                       special || sprPtr(c, spr) != nullptr,
                       sprUserWritable(spr)))
        return;
    if (spr == 1017) { // L2CR: invalidate completes instantly, L2IP reads 0
        c.st.l2cr = v & ~1u;
        if (v & 0x00200000u)
            c.l2WipeAll(); // L2I global invalidate
        if (v & 0x80000000u)
            c.l2Resize(); // L2E: size the model per L2SIZ
        return;
    }
    if (spr == 22) { // DEC: MSB 0->1 by any means requests the exception
        const u32 old = c.st.dec;
        c.st.dec = v;
        if (!(old & 0x80000000u) && (v & 0x80000000u))
            c.decPending = true;
        return;
    }
    if (spr == 284) { c.st.tb = (c.st.tb & 0xFFFFFFFF00000000ull) | v; return; }
    if (spr == 285) { c.st.tb = (c.st.tb & 0xFFFFFFFFull) | (static_cast<u64>(v) << 32); return; }
    if (spr == 287) return; // PVR is mfspr-only
    if (spr == 1008) { // HID0: honor cache-control transitions
        const u32 old = c.st.hid0;
        c.st.hid0 = v;
        if (~old & v & 0x00000400u)
            c.l1dFlushAll(false); // DCFI 0->1: flash invalidate, no wb
        return;
    }
    *sprPtr(c, spr) = v;
}
void h_mftb(Cpu& c, u32 i, const InsnDesc&)
{
    const u32 tbr = f_spr(i);
    if (tbr == 269)
        c.st.gpr[f_rt(i)] = static_cast<u32>(c.st.tb >> 32);
    else
        c.st.gpr[f_rt(i)] = static_cast<u32>(c.st.tb);
}

void h_mfmsr(Cpu& c, u32 i, const InsnDesc&) { c.st.gpr[f_rt(i)] = c.st.msr; }
void h_mtmsr(Cpu& c, u32 i, const InsnDesc&)
{
    c.st.msr = c.st.gpr[f_rt(i)] & msr::VALID;
    // MSR[POW] with a HID0 power mode selected enters nap/doze/sleep: no
    // further instructions until an enabled interrupt (TB keeps ticking).
    if ((c.st.msr & msr::POW) &&
        (c.st.hid0 & 0x00E00000u)) // DOZE | NAP | SLEEP
        c.napping = true;
}
void h_mtsr(Cpu& c, u32 i, const InsnDesc&)  { c.st.sr[f_sr(i)] = c.st.gpr[f_rt(i)]; }
void h_mfsr(Cpu& c, u32 i, const InsnDesc&)  { c.st.gpr[f_rt(i)] = c.st.sr[f_sr(i)]; }
void h_mtsrin(Cpu& c, u32 i, const InsnDesc&) { c.st.sr[c.st.gpr[f_rb(i)] >> 28] = c.st.gpr[f_rt(i)]; }
void h_mfsrin(Cpu& c, u32 i, const InsnDesc&) { c.st.gpr[f_rt(i)] = c.st.sr[c.st.gpr[f_rb(i)] >> 28]; }

// ---- cache / sync / hints --------------------------------------------------

void h_nop(Cpu&, u32, const InsnDesc&) {}
void h_tlbie(Cpu& c, u32 i, const InsnDesc&)
{
    c.tlbInvalidateClass(c.st.gpr[f_rb(i)]);
}
void h_dcbz(Cpu& c, u32 i, const InsnDesc&)
{
    // dcbz raises an alignment exception when the data cache is disabled
    // (HID0[DCE]=0) or the target is write-through/cache-inhibited
    // (UM 4.6.6 / Table 5-4; xnu relies on the W/I case). RECEIPT: the
    // DCE check precedes translation.
    const u32 ea = eaX(c, i) & ~31u; // 32-byte block: never crosses a page
    if (!(c.st.hid0 & 0x00004000u)) { // DCE
        raiseAlign(c, i, ea);
        return;
    }
    u32 pa, wimg;
    if (!c.translate(ea, true, false, pa, &wimg))
        return;
    if (wimg & 0xCu) { // W or I
        raiseAlign(c, i, ea);
        return;
    }
    c.dcbzLine(pa); // zeros exist in the cache only until written back
}

void h_dcbf(Cpu& c, u32 i, const InsnDesc&)
{
    u32 pa;
    if (c.translate(eaX(c, i) & ~31u, false, false, pa))
        c.dcbClean(pa, true);
}
void h_dcbst(Cpu& c, u32 i, const InsnDesc&)
{
    u32 pa;
    if (c.translate(eaX(c, i) & ~31u, false, false, pa))
        c.dcbClean(pa, false);
}
void h_dcbi(Cpu& c, u32 i, const InsnDesc&)
{
    u32 pa;
    if (c.translate(eaX(c, i) & ~31u, true, false, pa))
        c.dcbKill(pa);
}

} // namespace

void bindHandlers()
{
    static bool bound = false;
    if (bound)
        return;
    bound = true;

    setHandler("addi", h_addi);
    setHandler("addis", h_addis);
    setHandler("addic", h_addic);
    setHandler("addic.", h_addicRc);
    setHandler("subfic", h_subfic);
    setHandler("mulli", h_mulli);
    setHandler("ori", h_ori);
    setHandler("oris", h_oris);
    setHandler("xori", h_xori);
    setHandler("xoris", h_xoris);
    setHandler("andi.", h_andiRc);
    setHandler("andis.", h_andisRc);
    setHandler("cmpi", h_cmpi);
    setHandler("cmpli", h_cmpli);
    setHandler("cmp", h_cmp);
    setHandler("cmpl", h_cmpl);
    setHandler("twi", h_twi);
    setHandler("tw", h_tw);
    setHandler("rlwinm", h_rlwinm);
    setHandler("rlwnm", h_rlwnm);
    setHandler("rlwimi", h_rlwimi);
    setHandler("b", h_b);
    setHandler("bc", h_bc);
    setHandler("bclr", h_bclr);
    setHandler("bcctr", h_bcctr);
    setHandler("sc", h_sc);
    setHandler("rfi", h_rfi);

    setHandler("add", h_add);
    setHandler("addc", h_addc);
    setHandler("adde", h_adde);
    setHandler("addme", h_addme);
    setHandler("addze", h_addze);
    setHandler("subf", h_subf);
    setHandler("subfc", h_subfc);
    setHandler("subfe", h_subfe);
    setHandler("subfme", h_subfme);
    setHandler("subfze", h_subfze);
    setHandler("neg", h_neg);
    setHandler("mullw", h_mullw);
    setHandler("mulhw", h_mulhw);
    setHandler("mulhwu", h_mulhwu);
    setHandler("divw", h_divw);
    setHandler("divwu", h_divwu);

    setHandler("and", h_and);
    setHandler("andc", h_andcx);
    setHandler("or", h_or);
    setHandler("orc", h_orc);
    setHandler("xor", h_xor);
    setHandler("eqv", h_eqv);
    setHandler("nor", h_nor);
    setHandler("nand", h_nand);
    setHandler("slw", h_slw);
    setHandler("srw", h_srw);
    setHandler("sraw", h_sraw);
    setHandler("srawi", h_srawi);
    setHandler("cntlzw", h_cntlzw);
    setHandler("extsb", h_extsb);
    setHandler("extsh", h_extsh);

    setHandler("lwz", h_lwz);
    setHandler("lbz", h_lbz);
    setHandler("lhz", h_lhz);
    setHandler("lha", h_lha);
    setHandler("lwzx", h_lwzx);
    setHandler("lbzx", h_lbzx);
    setHandler("lhzx", h_lhzx);
    setHandler("lhax", h_lhax);
    setHandler("lwzu", h_lwzu);
    setHandler("lbzu", h_lbzu);
    setHandler("lhzu", h_lhzu);
    setHandler("lhau", h_lhau);
    setHandler("lwzux", h_lwzux);
    setHandler("lbzux", h_lbzux);
    setHandler("lhzux", h_lhzux);
    setHandler("lhaux", h_lhaux);
    setHandler("stw", h_stw);
    setHandler("stb", h_stb);
    setHandler("sth", h_sth);
    setHandler("stwx", h_stwx);
    setHandler("stbx", h_stbx);
    setHandler("sthx", h_sthx);
    setHandler("stwu", h_stwu);
    setHandler("stbu", h_stbu);
    setHandler("sthu", h_sthu);
    setHandler("stwux", h_stwux);
    setHandler("stbux", h_stbux);
    setHandler("sthux", h_sthux);
    setHandler("lwbrx", h_lwbrx);
    setHandler("lhbrx", h_lhbrx);
    setHandler("stwbrx", h_stwbrx);
    setHandler("sthbrx", h_sthbrx);
    setHandler("lmw", h_lmw);
    setHandler("stmw", h_stmw);
    setHandler("lswi", h_lswi);
    setHandler("stswi", h_stswi);
    setHandler("lswx", h_lswx);
    setHandler("stswx", h_stswx);
    setHandler("lwarx", h_lwarx);
    setHandler("stwcx.", h_stwcx);
    setHandler("eciwx", h_eciwx);
    setHandler("ecowx", h_ecowx);

    setHandler("lfs", h_lfs);
    setHandler("lfsu", h_lfsu);
    setHandler("lfsx", h_lfsx);
    setHandler("lfsux", h_lfsux);
    setHandler("lfd", h_lfd);
    setHandler("lfdu", h_lfdu);
    setHandler("lfdx", h_lfdx);
    setHandler("lfdux", h_lfdux);
    setHandler("stfs", h_stfs);
    setHandler("stfsu", h_stfsu);
    setHandler("stfsx", h_stfsx);
    setHandler("stfsux", h_stfsux);
    setHandler("stfd", h_stfd);
    setHandler("stfdu", h_stfdu);
    setHandler("stfdx", h_stfdx);
    setHandler("stfdux", h_stfdux);
    setHandler("stfiwx", h_stfiwx);

    setHandler("mcrf", h_mcrf);
    setHandler("crand", h_crand);
    setHandler("cror", h_cror);
    setHandler("crxor", h_crxor);
    setHandler("crnand", h_crnand);
    setHandler("crnor", h_crnor);
    setHandler("creqv", h_creqv);
    setHandler("crandc", h_crandc);
    setHandler("crorc", h_crorc);
    setHandler("mfcr", h_mfcr);
    setHandler("mtcrf", h_mtcrf);
    setHandler("mcrxr", h_mcrxr);

    setHandler("mfspr", h_mfspr);
    setHandler("mtspr", h_mtspr);
    setHandler("mftb", h_mftb);
    setHandler("mfmsr", h_mfmsr);
    setHandler("mtmsr", h_mtmsr);
    setHandler("mtsr", h_mtsr);
    setHandler("mfsr", h_mfsr);
    setHandler("mtsrin", h_mtsrin);
    setHandler("mfsrin", h_mfsrin);

    setHandler("sync", h_nop);
    setHandler("isync", h_nop);
    setHandler("eieio", h_nop);
    setHandler("dcbf", h_dcbf);
    setHandler("dcbst", h_dcbst);
    setHandler("dcbt", h_nop);
    setHandler("dcbtst", h_nop);
    setHandler("dcba", h_nop);
    setHandler("dcbi", h_dcbi);
    setHandler("icbi", h_nop);
    setHandler("dcbz", h_dcbz);
    setHandler("dst", h_nop);
    setHandler("dstst", h_nop);
    setHandler("dss", h_nop);
    setHandler("tlbie", h_tlbie);
    setHandler("tlbsync", h_nop);

    bindFpuHandlers();
    bindVecHandlers();
}

} // namespace opm
