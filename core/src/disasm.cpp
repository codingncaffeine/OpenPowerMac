// Disassembler. Prints the standard mnemonics plus the common simplified
// mnemonics (PEM Appendix F / UM §2.3.7) so output can be diffed against
// llvm-objdump as a tier-3 decode oracle. Style::Gnu prints r/f/v register
// prefixes for human traces; Style::Llvm prints bare numbers.

#include "opm/insn.hpp"
#include "opm/bits.hpp"
#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace opm {

namespace {

struct Out {
    char* p;
    size_t cap;
    size_t len = 0;

    void put(const char* fmt, ...)
    {
        if (len >= cap)
            return;
        va_list ap;
        va_start(ap, fmt);
        const int n = vsnprintf(p + len, cap - len, fmt, ap);
        va_end(ap);
        if (n > 0)
            len += static_cast<size_t>(n);
        if (len > cap - 1)
            len = cap - 1;
    }
};

const char* condName(u32 bi, bool positive)
{
    static const char* const pos[4] = {"lt", "gt", "eq", "so"};
    static const char* const neg[4] = {"ge", "le", "ne", "ns"};
    return positive ? pos[bi & 3u] : neg[bi & 3u];
}

struct Ctx {
    u32 insn;
    u32 pc;
    Style style;
    Out out;

    void reg(u32 n)  { out.put(style == Style::Gnu ? "r%u" : "%u", n); }
    void freg(u32 n) { out.put(style == Style::Gnu ? "f%u" : "%u", n); }
    void vreg(u32 n) { out.put(style == Style::Gnu ? "v%u" : "%u", n); }
    void crf(u32 n)  { out.put("cr%u", n); }
    void imm(i32 v)  { out.put("%d", v); }
    void uimm(u32 v) { out.put("%u", v); }
    void target(i32 disp)
    {
        const u32 t = f_aabit(insn) ? static_cast<u32>(disp)
                                    : pc + static_cast<u32>(disp);
        out.put("0x%x", t);
    }
};

// Mnemonic with suffix letters per flags and instruction bits.
void mnem(Ctx& c, const InsnDesc& d)
{
    char buf[24];
    size_t n = 0;
    for (const char* s = d.mnem; *s && n < sizeof(buf) - 5; ++s)
        buf[n++] = *s;
    if ((d.flags & FL_TBIT) && ppcbit(c.insn, 6))
        buf[n++] = 't';
    if ((d.flags & FL_OE) && f_oebit(c.insn))
        buf[n++] = 'o';
    if ((d.flags & FL_LK) && f_lkbit(c.insn))
        buf[n++] = 'l';
    if ((d.flags & FL_AA) && f_aabit(c.insn))
        buf[n++] = 'a';
    if ((d.flags & FL_RC) && f_rcbit(c.insn))
        buf[n++] = '.';
    if ((d.flags & FL_VRC) && ppcbit(c.insn, 21))
        buf[n++] = '.';
    buf[n] = 0;
    c.out.put("%s ", buf);
}

// ---- Branch alias printing -------------------------------------------------

// kind: 0 = bc (has target), 1 = bclr, 2 = bcctr. Returns true if printed.
bool printBranch(Ctx& c, int kind)
{
    const u32 bo = ppcbits(c.insn, 6, 10);
    const u32 bi = ppcbits(c.insn, 11, 15);
    const bool lk = f_lkbit(c.insn) != 0;
    const bool dec = (bo & 4u) == 0;
    const bool condCheck = (bo & 16u) == 0;
    const bool hintY = (bo & 1u) != 0;
    const i32 disp = kind == 0 ? sext14(ppcbits(c.insn, 16, 29)) * 4 : 0;

    const char* tail = kind == 1 ? "lr" : kind == 2 ? "ctr" : "";

    char m[24];
    if (dec && !condCheck) {
        if (kind == 2)
            return false; // bcctr with decrement is invalid — print raw
        snprintf(m, sizeof m, "b%s%s%s", (bo & 2u) ? "dz" : "dnz", tail, lk ? "l" : "");
    } else if (!dec && condCheck) {
        const bool ifTrue = (bo & 8u) != 0;
        if (c.style == Style::Llvm) {
            // llvm-objdump prints the generic bt/bf form with a numeric CR bit.
            snprintf(m, sizeof m, "b%c%s%s", ifTrue ? 't' : 'f', tail, lk ? "l" : "");
            c.out.put("%s", m);
            if (hintY)
                c.out.put(kind == 0 ? (disp < 0 ? "-" : "+") : "+");
            c.out.put(" %u", bi);
            if (kind == 0) {
                c.out.put(", ");
                c.target(disp);
            }
            return true;
        }
        snprintf(m, sizeof m, "b%s%s%s", condName(bi, ifTrue), tail, lk ? "l" : "");
    } else if (!dec && !condCheck) {
        // Branch always.
        if (kind == 0)
            return false; // plain "bc 20,..." — leave raw
        snprintf(m, sizeof m, "b%s%s", tail, lk ? "l" : "");
    } else {
        return false; // decrement + condition forms: print raw bc
    }

    c.out.put("%s", m);
    if (hintY)
        c.out.put(kind == 0 ? (disp < 0 ? "-" : "+") : "+");

    bool first = true;
    if (!dec && condCheck && (bi / 4u) != 0) {
        c.out.put(" ");
        c.crf(bi / 4u);
        first = false;
    }
    if (kind == 0) {
        c.out.put(first ? " " : ", ");
        c.target(disp);
    }
    return true;
}

// ---- rlwinm aliases --------------------------------------------------------

bool printRlwinmAlias(Ctx& c)
{
    if (f_rcbit(c.insn))
        return false; // keep record forms raw for clarity
    const u32 ra = f_ra(c.insn), rs = f_rt(c.insn);
    const u32 sh = f_sh(c.insn), mb = f_mb(c.insn), me = f_me(c.insn);
    const bool full = c.style == Style::Gnu; // llvm-objdump only aliases slwi/srwi

    const char* m = nullptr;
    u32 n = 0;
    if (mb == 0 && me == 31 && sh != 0) {
        m = "rotlwi"; n = sh;
    } else if (mb == 0 && me == 31 - sh && sh != 0) {
        m = "slwi"; n = sh;
    } else if (me == 31 && mb != 0 && sh == 32 - mb) {
        m = "srwi"; n = mb;
    } else if (sh == 0 && me == 31 && mb != 0) {
        m = "clrlwi"; n = mb;
    } else if (full && sh == 0 && mb == 0 && me != 31) {
        // llvm-objdump stops short of clrrwi; keep it for human (Gnu) output.
        m = "clrrwi"; n = 31 - me;
    } else {
        return false;
    }
    c.out.put("%s ", m);
    c.reg(ra); c.out.put(", "); c.reg(rs); c.out.put(", ");
    c.uimm(n);
    return true;
}

// ---- SPR aliases -----------------------------------------------------------

const char* sprShortName(u32 spr)
{
    switch (spr) {
    case 1: return "xer";
    case 8: return "lr";
    case 9: return "ctr";
    default: return nullptr;
    }
}

} // namespace

int disassemble(u32 insn, u32 pc, char* outBuf, size_t cap, Style style)
{
    if (cap == 0)
        return 0;
    Ctx c{insn, pc, style, Out{outBuf, cap}};
    const InsnDesc* dp = decode(insn);
    if (!dp) {
        c.out.put(".long 0x%08x", insn);
        return static_cast<int>(c.out.len);
    }
    const InsnDesc& d = *dp;

    const u32 rt = f_rt(insn), ra = f_ra(insn), rb = f_rb(insn);
    const i32 si = sext16(f_d(insn));
    const u32 ui = f_d(insn);

    // ---- aliases ----
    if (d.primary == 24 && rt == 0 && ra == 0 && ui == 0) { // ori 0,0,0
        c.out.put("nop");
        return static_cast<int>(c.out.len);
    }
    if (d.primary == 14 && ra == 0) { // addi -> li
        c.out.put("li "); c.reg(rt); c.out.put(", "); c.imm(si);
        return static_cast<int>(c.out.len);
    }
    if (d.primary == 15 && ra == 0) { // addis -> lis
        c.out.put("lis "); c.reg(rt); c.out.put(", "); c.imm(si);
        return static_cast<int>(c.out.len);
    }
    if (d.kind == Xk::X31 && d.xo == 444 && rt == rb) { // or -> mr
        c.out.put(f_rcbit(insn) ? "mr. " : "mr ");
        c.reg(ra); c.out.put(", "); c.reg(rt);
        return static_cast<int>(c.out.len);
    }
    if (d.kind == Xk::X31 && d.xo == 124 && rt == rb) { // nor -> not
        c.out.put(f_rcbit(insn) ? "not. " : "not ");
        c.reg(ra); c.out.put(", "); c.reg(rt);
        return static_cast<int>(c.out.len);
    }
    if (d.kind == Xk::X31 && d.xo == 4 && rt == 31 && ra == 0 && rb == 0) { // tw 31,0,0
        c.out.put("trap");
        return static_cast<int>(c.out.len);
    }
    if (d.kind == Xk::X31 && (d.xo == 40 || d.xo == 8) && !f_oebit(insn)) {
        // subf -> sub / subfc -> subc, with minuend/subtrahend operand order.
        c.out.put(d.xo == 40 ? (f_rcbit(insn) ? "sub. " : "sub ")
                             : (f_rcbit(insn) ? "subc. " : "subc "));
        c.reg(rt); c.out.put(", "); c.reg(rb); c.out.put(", "); c.reg(ra);
        return static_cast<int>(c.out.len);
    }
    if (d.pat == Pat::BC && printBranch(c, 0))
        return static_cast<int>(c.out.len);
    if (d.pat == Pat::BCLR && printBranch(c, 1))
        return static_cast<int>(c.out.len);
    if (d.pat == Pat::BCCTR && printBranch(c, 2))
        return static_cast<int>(c.out.len);
    if (d.pat == Pat::RLWINM && d.primary == 21 && printRlwinmAlias(c))
        return static_cast<int>(c.out.len);
    if (d.pat == Pat::CRB3) {
        const u32 bt = rt, ba = ra, bb = rb;
        if (d.xo == 193 && bt == ba && ba == bb) { c.out.put("crclr %u", bt); return static_cast<int>(c.out.len); }
        if (d.xo == 289 && bt == ba && ba == bb) { c.out.put("crset %u", bt); return static_cast<int>(c.out.len); }
        if (d.xo == 449 && ba == bb) { c.out.put("crmove %u, %u", bt, ba); return static_cast<int>(c.out.len); }
        if (d.xo == 33 && ba == bb)  { c.out.put("crnot %u, %u", bt, ba); return static_cast<int>(c.out.len); }
    }

    // ---- generic patterns ----
    switch (d.pat) {
    case Pat::NONE:
        c.out.put("%s", d.mnem);
        break;
    case Pat::RT_RA_RB:
        mnem(c, d); c.reg(rt); c.out.put(", "); c.reg(ra); c.out.put(", "); c.reg(rb);
        break;
    case Pat::RT_RA:
        mnem(c, d); c.reg(rt); c.out.put(", "); c.reg(ra);
        break;
    case Pat::RT_RA_SI:
        mnem(c, d); c.reg(rt); c.out.put(", "); c.reg(ra); c.out.put(", "); c.imm(si);
        break;
    case Pat::RA_RS_RB:
        mnem(c, d); c.reg(ra); c.out.put(", "); c.reg(rt); c.out.put(", "); c.reg(rb);
        break;
    case Pat::RA_RS:
        mnem(c, d); c.reg(ra); c.out.put(", "); c.reg(rt);
        break;
    case Pat::RA_RS_UI:
        mnem(c, d); c.reg(ra); c.out.put(", "); c.reg(rt); c.out.put(", "); c.uimm(ui);
        break;
    case Pat::RA_RS_SH:
        mnem(c, d); c.reg(ra); c.out.put(", "); c.reg(rt); c.out.put(", "); c.uimm(f_sh(insn));
        break;
    case Pat::RT_D_RA:
    case Pat::RS_D_RA:
        mnem(c, d); c.reg(rt); c.out.put(", %d(", si); c.reg(ra); c.out.put(")");
        break;
    case Pat::FRT_D_RA:
    case Pat::FRS_D_RA:
        mnem(c, d); c.freg(rt); c.out.put(", %d(", si); c.reg(ra); c.out.put(")");
        break;
    case Pat::RS_RA_RB:
        mnem(c, d); c.reg(rt); c.out.put(", "); c.reg(ra); c.out.put(", "); c.reg(rb);
        break;
    case Pat::FRT_RA_RB:
    case Pat::FRS_RA_RB:
        mnem(c, d); c.freg(rt); c.out.put(", "); c.reg(ra); c.out.put(", "); c.reg(rb);
        break;
    case Pat::CMP_SI:
    case Pat::CMP_UI:
    case Pat::CMP_RB: {
        const bool lbit = ppcbit(insn, 10) != 0;
        const char* base = d.pat == Pat::CMP_SI ? (lbit ? "cmpdi" : "cmpwi")
                        : d.pat == Pat::CMP_UI ? (lbit ? "cmpldi" : "cmplwi")
                        : d.xo == 0            ? (lbit ? "cmpd" : "cmpw")
                                               : (lbit ? "cmpld" : "cmplw");
        c.out.put("%s ", base);
        if (f_crfd(insn) != 0) { c.crf(f_crfd(insn)); c.out.put(", "); }
        c.reg(ra); c.out.put(", ");
        if (d.pat == Pat::CMP_SI) c.imm(si);
        else if (d.pat == Pat::CMP_UI) c.uimm(ui);
        else c.reg(rb);
        break;
    }
    case Pat::TW_RB:
        mnem(c, d); c.uimm(f_to(insn)); c.out.put(", "); c.reg(ra); c.out.put(", "); c.reg(rb);
        break;
    case Pat::TW_SI:
        mnem(c, d); c.uimm(f_to(insn)); c.out.put(", "); c.reg(ra); c.out.put(", "); c.imm(si);
        break;
    case Pat::B:
        mnem(c, d); c.target(sext26(ppcbits(insn, 6, 29) << 2));
        break;
    case Pat::BC:
        mnem(c, d);
        c.uimm(ppcbits(insn, 6, 10)); c.out.put(", ");
        c.uimm(ppcbits(insn, 11, 15)); c.out.put(", ");
        c.target(sext14(ppcbits(insn, 16, 29)) * 4);
        break;
    case Pat::BCLR:
    case Pat::BCCTR:
        mnem(c, d);
        c.uimm(ppcbits(insn, 6, 10)); c.out.put(", ");
        c.uimm(ppcbits(insn, 11, 15));
        break;
    case Pat::CRB3:
        mnem(c, d); c.uimm(rt); c.out.put(", "); c.uimm(ra); c.out.put(", "); c.uimm(rb);
        break;
    case Pat::MCRF:
        mnem(c, d); c.crf(f_crfd(insn)); c.out.put(", "); c.crf(f_crfs(insn));
        break;
    case Pat::RLWINM:
        mnem(c, d); c.reg(ra); c.out.put(", "); c.reg(rt); c.out.put(", ");
        c.uimm(f_sh(insn)); c.out.put(", "); c.uimm(f_mb(insn)); c.out.put(", "); c.uimm(f_me(insn));
        break;
    case Pat::RLWNM:
        mnem(c, d); c.reg(ra); c.out.put(", "); c.reg(rt); c.out.put(", ");
        c.reg(rb); c.out.put(", "); c.uimm(f_mb(insn)); c.out.put(", "); c.uimm(f_me(insn));
        break;
    case Pat::MTCRF:
        if (f_crm(insn) == 0xFFu) {
            c.out.put("mtcr "); c.reg(rt);
        } else {
            mnem(c, d); c.uimm(f_crm(insn)); c.out.put(", "); c.reg(rt);
        }
        break;
    case Pat::MFCR:
        mnem(c, d); c.reg(rt);
        break;
    case Pat::MFSPR: {
        const u32 spr = f_spr(insn);
        if (const char* nm = sprShortName(spr)) {
            c.out.put("mf%s ", nm); c.reg(rt);
        } else {
            mnem(c, d); c.reg(rt); c.out.put(", %u", spr);
        }
        break;
    }
    case Pat::MTSPR: {
        const u32 spr = f_spr(insn);
        if (const char* nm = sprShortName(spr)) {
            c.out.put("mt%s ", nm); c.reg(rt);
        } else {
            mnem(c, d); c.out.put("%u, ", spr); c.reg(rt);
        }
        break;
    }
    case Pat::MFTB: {
        const u32 tbr = f_spr(insn);
        if (tbr == 269) { c.out.put("mftbu "); c.reg(rt); }
        else if (tbr == 268) { c.out.put("mftb "); c.reg(rt); }
        else { mnem(c, d); c.reg(rt); c.out.put(", %u", tbr); }
        break;
    }
    case Pat::MFMSR:
        mnem(c, d); c.reg(rt);
        break;
    case Pat::MTMSR:
        mnem(c, d); c.reg(rt);
        break;
    case Pat::MTSR:
        mnem(c, d); c.uimm(f_sr(insn)); c.out.put(", "); c.reg(rt);
        break;
    case Pat::MFSR:
        mnem(c, d); c.reg(rt); c.out.put(", "); c.uimm(f_sr(insn));
        break;
    case Pat::MTSRIN:
        mnem(c, d); c.reg(rt); c.out.put(", "); c.reg(rb);
        break;
    case Pat::MFSRIN:
        mnem(c, d); c.reg(rt); c.out.put(", "); c.reg(rb);
        break;
    case Pat::RB_ONLY:
        mnem(c, d); c.reg(rb);
        break;
    case Pat::RA_RB:
        mnem(c, d); c.reg(ra); c.out.put(", "); c.reg(rb);
        break;
    case Pat::LSWI:
        mnem(c, d); c.reg(rt); c.out.put(", "); c.reg(ra); c.out.put(", "); c.uimm(f_nb(insn));
        break;
    case Pat::MCRXR:
        mnem(c, d); c.crf(f_crfd(insn));
        break;
    case Pat::SC:
        c.out.put("sc");
        break;
    case Pat::FP2:
        mnem(c, d); c.freg(rt); c.out.put(", "); c.freg(rb);
        break;
    case Pat::FP3:
        mnem(c, d); c.freg(rt); c.out.put(", "); c.freg(ra); c.out.put(", "); c.freg(rb);
        break;
    case Pat::FP3C:
        mnem(c, d); c.freg(rt); c.out.put(", "); c.freg(ra); c.out.put(", "); c.freg(f_rc_vec(insn));
        break;
    case Pat::FP4:
        mnem(c, d); c.freg(rt); c.out.put(", "); c.freg(ra); c.out.put(", ");
        c.freg(f_rc_vec(insn)); c.out.put(", "); c.freg(rb);
        break;
    case Pat::FCMP:
        mnem(c, d); c.crf(f_crfd(insn)); c.out.put(", "); c.freg(ra); c.out.put(", "); c.freg(rb);
        break;
    case Pat::MTFSF:
        mnem(c, d); c.uimm(f_fm(insn)); c.out.put(", "); c.freg(rb);
        break;
    case Pat::MTFSFI:
        mnem(c, d); c.crf(f_crfd(insn)); c.out.put(", "); c.uimm(ppcbits(insn, 16, 19));
        break;
    case Pat::MTFSB:
        mnem(c, d); c.uimm(rt);
        break;
    case Pat::MCRFS:
        mnem(c, d); c.crf(f_crfd(insn)); c.out.put(", "); c.crf(f_crfs(insn));
        break;
    case Pat::MFFS:
        mnem(c, d); c.freg(rt);
        break;
    case Pat::VX3:
        mnem(c, d); c.vreg(rt); c.out.put(", "); c.vreg(ra); c.out.put(", "); c.vreg(rb);
        break;
    case Pat::VX2B:
        mnem(c, d); c.vreg(rt); c.out.put(", "); c.vreg(rb);
        break;
    case Pat::VX_SPLAT:
        mnem(c, d); c.vreg(rt); c.out.put(", "); c.vreg(rb); c.out.put(", "); c.uimm(ra);
        break;
    case Pat::VX_SPLATIS:
        mnem(c, d); c.vreg(rt); c.out.put(", "); c.imm(sext5(f_simm5(insn)));
        break;
    case Pat::MFVSCR:
        mnem(c, d); c.vreg(rt);
        break;
    case Pat::MTVSCR:
        mnem(c, d); c.vreg(rb);
        break;
    case Pat::VA4P:
        mnem(c, d); c.vreg(rt); c.out.put(", "); c.vreg(ra); c.out.put(", ");
        c.vreg(rb); c.out.put(", "); c.vreg(f_rc_vec(insn));
        break;
    case Pat::VA_MADD:
        mnem(c, d); c.vreg(rt); c.out.put(", "); c.vreg(ra); c.out.put(", ");
        c.vreg(f_rc_vec(insn)); c.out.put(", "); c.vreg(rb);
        break;
    case Pat::VSLDOI:
        mnem(c, d); c.vreg(rt); c.out.put(", "); c.vreg(ra); c.out.put(", ");
        c.vreg(rb); c.out.put(", "); c.uimm(f_vsh(insn));
        break;
    case Pat::VD_RA_RB:
    case Pat::VS_RA_RB:
        mnem(c, d); c.vreg(rt); c.out.put(", "); c.reg(ra); c.out.put(", "); c.reg(rb);
        break;
    case Pat::DST:
        mnem(c, d); c.reg(ra); c.out.put(", "); c.reg(rb); c.out.put(", "); c.uimm(ppcbits(insn, 9, 10));
        break;
    case Pat::DSS:
        if (ppcbit(insn, 6)) {
            c.out.put("dssall");
        } else {
            c.out.put("dss "); c.uimm(ppcbits(insn, 9, 10));
        }
        break;
    }

    return static_cast<int>(c.out.len);
}

} // namespace opm
