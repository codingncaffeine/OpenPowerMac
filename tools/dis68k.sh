#!/bin/bash
# A 68000/68020 disassembler for reading the Mac OS ROM's 68K code.
#
# We have a PowerPC disassembler (ppcrun --disasm) but had nothing for 68K, so
# every 68K decode in the ATA dig was done by reading hex by hand — which
# produced a wrong JSR base once and three watches placed on addresses that
# are never instruction boundaries.
#
# The first version of this script covered the control-flow opcodes and
# GUESSED the length of everything else, treating one-word forms as one word
# whether or not they carried extension words. In the region it was tested on
# that happened to stay aligned; where it does not, every following address
# silently shifts and the disassembly becomes fiction that reads like fact.
# So this version DERIVES every length from the effective-address fields:
#
#   mode 0-4        no extension words
#   mode 5          one (d16,An)
#   mode 6, 7/3     one brief extension word, or the full 68020 format,
#                   whose base and outer displacements are counted properly
#   mode 7/0        one   ((xxx).w)
#   mode 7/1        two   ((xxx).l)
#   mode 7/2        one   (d16,PC)
#   mode 7/4        one or two, per operation size (immediate)
#
# and every instruction reports the raw words it consumed, so a decode can be
# checked against the bytes without leaving the listing. The run ends with an
# accounting line: instructions decoded, bytes consumed against bytes asked
# for, and how many words could not be decoded. Drift becomes visible instead
# of silent.
#
#   tools/dis68k.sh <ram-dump> <va> [count-bytes] [--strict]
#   tools/dis68k.sh --selftest
#
# <va> is a ROM virtual address (ffcxxxxx/ffdxxxxx); the Mac OS ROM is staged
# at PA 0x00c00000, so PA = VA - 0xFF000000. --strict turns a window that
# ends mid-instruction into a non-zero exit.
set -e

AWKSRC=$(mktemp)
trap 'rm -f "$AWKSRC"' EXIT
cat > "$AWKSRC" <<'AWKEOF'
function sx8(v)  { return v >= 128 ? v - 256 : v }
function sx16(v) { return v >= 32768 ? v - 65536 : v }
function sx32(v) { return v >= 2147483648 ? v - 4294967296 : v }
function hx(v)   { return sprintf("%08x", v < 0 ? v + 4294967296 : v) }
function szBytes(s) { return s == 0 ? 1 : s == 1 ? 2 : 4 }
function sznB(b)    { return b == 1 ? "b" : b == 2 ? "w" : "l" }
function szn2(s)    { return sznB(szBytes(s)) }
function dcw(op)    { ADV = 1; UNK++; return sprintf("dc.w    $%04x", op) }

# MOVEM register mask. For a -(An) destination the mask is reversed: bit 0
# names A7 rather than D0.
function reglist(m, rev,   i, r, o, have) {
    for (i = 0; i < 16; i++) {
        if (!and(m, lshift(1, i)))
            continue
        r = rev ? 15 - i : i
        have[r] = 1
    }
    o = ""
    for (r = 0; r < 16; r++)
        if (have[r])
            o = o (o ? "/" : "") (r < 8 ? "d" r : "a" (r - 8))
    return o ? o : "-"
}

# Indexed effective address at word index i. Sets EAN.
function idxea(i, base,   e, xr, xt, xsz, sc, bdsz, iis, n, d8) {
    e = W[i]
    xr  = and(rshift(e, 12), 7)
    xt  = and(rshift(e, 15), 1)
    xsz = and(rshift(e, 11), 1)
    sc  = lshift(1, and(rshift(e, 9), 3))
    if (and(e, 256) == 0) {                  # brief extension word
        EAN = 1
        d8 = sx8(and(e, 255))
        return sprintf("%d(%s,%s%d.%s*%d)", d8, base, xt ? "a" : "d", xr,
                       xsz ? "l" : "w", sc)
    }
    bdsz = and(rshift(e, 4), 3)              # base displacement size
    iis  = and(e, 7)                         # index/indirect selection
    if (bdsz == 0) { EAN = 1; UNK++; return "(reserved-ext)" }
    n = 1
    if (bdsz == 2) n = n + 1
    else if (bdsz == 3) n = n + 2
    if (and(iis, 3) == 2) n = n + 1
    else if (and(iis, 3) == 3) n = n + 2
    EAN = n
    return sprintf("(%s,full-ext:%dw)", base, n)
}

# Effective address; i is the word index of its FIRST extension word, sz the
# operation size in bytes. Sets EAN to the number of words consumed.
function ea(m, r, sz, i,   tgt) {
    EAN = 0
    if (m == 0) return "d" r
    if (m == 1) return "a" r
    if (m == 2) return "(a" r ")"
    if (m == 3) return "(a" r ")+"
    if (m == 4) return "-(a" r ")"
    if (m == 5) { EAN = 1; return sprintf("%d(a%d)", sx16(W[i]), r) }
    if (m == 6) return idxea(i, "a" r)
    if (m != 7) { UNK++; return "?" }
    if (r == 0) { EAN = 1; return sprintf("($%04x).w", W[i]) }
    if (r == 1) { EAN = 2; return sprintf("($%04x%04x).l", W[i], W[i + 1]) }
    if (r == 2) {
        EAN = 1
        tgt = VA + i * 2 + sx16(W[i])
        return sprintf("%d(pc){%s}", sx16(W[i]), hx(tgt))
    }
    if (r == 3) return idxea(i, "pc")
    if (r == 4) {
        if (sz == 4) { EAN = 2; return sprintf("#$%04x%04x", W[i], W[i+1]) }
        EAN = 1
        if (sz == 1) return sprintf("#$%02x", and(W[i], 255))
        return sprintf("#$%04x", W[i])
    }
    UNK++
    return "?"
}

# --- group decoders. Each sets ADV to the words the instruction occupies. ---

function dis0(i, op,   m, r, sz, o, nm, dn, imm, immw, dst) {
    m = and(rshift(op, 3), 7); r = and(op, 7)
    if (and(op, 0x0138) == 0x0108) {         # MOVEP d16(Ay),Dx and back
        ADV = 2
        return sprintf("movep.%s %d(a%d),d%d%s",
                       and(op, 64) ? "l" : "w", sx16(W[i+1]), r,
                       and(rshift(op, 9), 7),
                       and(op, 128) ? " (reversed)" : "")
    }
    if (and(op, 0x0100)) {                   # BTST/BCHG/BCLR/BSET Dn,<ea>
        dn = and(rshift(op, 9), 7)
        nm = substr("btstbchgbclrbset", 1 + 4 * and(rshift(op, 6), 3), 4)
        o = ea(m, r, m == 0 ? 4 : 1, i + 1); ADV = 1 + EAN
        return sprintf("%-7s d%d,%s", nm, dn, o)
    }
    if (and(op, 0x0F00) == 0x0800) {         # static bit number
        nm = substr("btstbchgbclrbset", 1 + 4 * and(rshift(op, 6), 3), 4)
        imm = W[i + 1]
        o = ea(m, r, m == 0 ? 4 : 1, i + 2); ADV = 2 + EAN
        return sprintf("%-7s #%d,%s", nm, and(imm, 31), o)
    }
    sz = and(rshift(op, 6), 3)
    dn = and(rshift(op, 9), 7)
    nm = IMM[dn + 1]
    if (sz == 3) {                           # 68020 forms sharing group 0
        if (and(op, 0xF9C0) == 0x00C0) {     # CHK2/CMP2: own extension word
            o = ea(m, r, 1, i + 2); ADV = 2 + EAN
            return sprintf("%-7s %s,ext=$%04x",
                           and(W[i+1], 2048) ? "chk2" : "cmp2", o, W[i+1])
        }
        if (and(op, 0xF9C0) == 0x08C0) {     # CAS / CAS2
            if (r == 4 && m == 7) { ADV = 3; return "cas2" }
            o = ea(m, r, 1, i + 2); ADV = 2 + EAN
            return sprintf("cas     %s,ext=$%04x", o, W[i+1])
        }
        return dcw(op)
    }
    if (and(op, 0xFF00) == 0x0E00) {         # MOVES: own extension word
        o = ea(m, r, szBytes(sz), i + 2); ADV = 2 + EAN
        return sprintf("%-7s %s,ext=$%04x", "moves." szn2(sz), o, W[i+1])
    }
    if (nm == "?") return dcw(op)
    if (m == 7 && r == 4 && (dn == 0 || dn == 1 || dn == 5)) {
        ADV = 2                              # ORI/ANDI/EORI to CCR or SR
        return sprintf("%-7s #$%04x,%s", nm, W[i + 1], sz == 0 ? "ccr" : "sr")
    }
    immw = (sz == 2) ? 2 : 1
    imm = (sz == 2) ? (W[i+1] * 65536 + W[i+2]) : W[i+1]
    o = ea(m, r, szBytes(sz), i + 1 + immw); ADV = 1 + immw + EAN
    return sprintf("%-7s #$%x,%s", nm "." szn2(sz), imm, o)
}

function disMove(i, op, g,   szb, sm, sr, dm, dr, src, dst, e1) {
    szb = (g == 1) ? 1 : (g == 3) ? 2 : 4
    sm = and(rshift(op, 3), 7); sr = and(op, 7)
    dr = and(rshift(op, 9), 7); dm = and(rshift(op, 6), 7)
    src = ea(sm, sr, szb, i + 1); e1 = EAN
    dst = ea(dm, dr, szb, i + 1 + e1); ADV = 1 + e1 + EAN
    return sprintf("%-7s %s,%s", (dm == 1 ? "movea." : "move.") sznB(szb),
                   src, dst)
}

function dis4(i, op,   m, r, sz, o, nm, mask, hi) {
    m = and(rshift(op, 3), 7); r = and(op, 7)
    if (op == 0x4AFC) return "illegal"
    if (op == 0x4E70) return "reset"
    if (op == 0x4E71) return "nop"
    if (op == 0x4E72) { ADV = 2; return sprintf("stop    #$%04x", W[i+1]) }
    if (op == 0x4E73) return "rte"
    if (op == 0x4E75) return "rts"
    if (op == 0x4E76) return "trapv"
    if (op == 0x4E77) return "rtr"
    if (and(op, 0xFFF0) == 0x4E40) return sprintf("trap    #%d", and(op, 15))
    if (and(op, 0xFFF8) == 0x4E50) {
        ADV = 2; return sprintf("link    a%d,#%d", r, sx16(W[i+1]))
    }
    if (op == 0x4E74) { ADV = 2; return sprintf("rtd     #%d", sx16(W[i+1])) }
    if (and(op, 0xFFF8) == 0x4E58) return sprintf("unlk    a%d", r)
    if (and(op, 0xFFF8) == 0x4808) {         # LINK.L: 32-bit displacement
        ADV = 3
        return sprintf("link.l  a%d,#%d", r, sx32(W[i+1] * 65536 + W[i+2]))
    }
    if (and(op, 0xFFF8) == 0x4848) return sprintf("bkpt    #%d", r)
    if (and(op, 0xFF80) == 0x4C00) {         # MULU.L/MULS.L, DIVU.L/DIVS.L
        o = ea(m, r, 4, i + 2); ADV = 2 + EAN
        return sprintf("%-7s %s,ext=$%04x",
                       (and(op, 64) ? "div" : "mul") \
                       (and(W[i+1], 2048) ? "s.l" : "u.l"), o, W[i+1])
    }
    if (and(op, 0xFFF8) == 0x4E60) return sprintf("move    a%d,usp", r)
    if (and(op, 0xFFF8) == 0x4E68) return sprintf("move    usp,a%d", r)
    if (and(op, 0xFFFE) == 0x4E7A) {
        ADV = 2
        return sprintf("movec   %s ext=$%04x", and(op, 1) ? "->cr" : "cr->",
                       W[i+1])
    }
    if (and(op, 0xFFC0) == 0x4E80) {
        o = ea(m, r, 4, i + 1); ADV = 1 + EAN; return sprintf("jsr     %s", o)
    }
    if (and(op, 0xFFC0) == 0x4EC0) {
        o = ea(m, r, 4, i + 1); ADV = 1 + EAN; return sprintf("jmp     %s", o)
    }
    if (and(op, 0xFFF8) == 0x4880) return sprintf("ext.w   d%d", r)
    if (and(op, 0xFFF8) == 0x48C0) return sprintf("ext.l   d%d", r)
    if (and(op, 0xFFF8) == 0x49C0) return sprintf("extb.l  d%d", r)
    if (and(op, 0xFFF8) == 0x4840) return sprintf("swap    d%d", r)
    if (and(op, 0xFB80) == 0x4880) {         # MOVEM
        mask = W[i + 1]
        o = ea(m, r, 4, i + 2); ADV = 2 + EAN
        nm = "movem." (and(op, 64) ? "l" : "w")
        if (and(op, 0x0400))
            return sprintf("%-7s %s,%s", nm, o, reglist(mask, 0))
        return sprintf("%-7s %s,%s", nm, reglist(mask, m == 4), o)
    }
    if (and(op, 0xFFC0) == 0x4800) {
        o = ea(m, r, 1, i + 1); ADV = 1 + EAN; return sprintf("nbcd    %s", o)
    }
    if (and(op, 0xFFC0) == 0x4840) {
        o = ea(m, r, 4, i + 1); ADV = 1 + EAN; return sprintf("pea     %s", o)
    }
    if (and(op, 0xFFC0) == 0x4AC0) {
        o = ea(m, r, 1, i + 1); ADV = 1 + EAN; return sprintf("tas     %s", o)
    }
    if (and(op, 0xFF00) == 0x4A00) {
        sz = and(rshift(op, 6), 3)
        if (sz == 3) return dcw(op)
        o = ea(m, r, szBytes(sz), i + 1); ADV = 1 + EAN
        return sprintf("%-7s %s", "tst." szn2(sz), o)
    }
    if (and(op, 0xF1C0) == 0x41C0) {
        o = ea(m, r, 4, i + 1); ADV = 1 + EAN
        return sprintf("lea     %s,a%d", o, and(rshift(op, 9), 7))
    }
    if (and(op, 0xF1C0) == 0x4180 || and(op, 0xF1C0) == 0x4100) {
        sz = and(op, 0x0080) ? 2 : 4
        o = ea(m, r, sz, i + 1); ADV = 1 + EAN
        return sprintf("%-7s %s,d%d", sz == 2 ? "chk.w" : "chk.l", o,
                       and(rshift(op, 9), 7))
    }
    hi = and(op, 0xFF00)
    if (hi == 0x4000 || hi == 0x4200 || hi == 0x4400 || hi == 0x4600) {
        sz = and(rshift(op, 6), 3)
        if (sz == 3) {                       # move sr/ccr forms
            nm = hi == 0x4000 ? "move    sr," : hi == 0x4200 ? "move    ccr," :
                 hi == 0x4400 ? "move    " : "move    "
            o = ea(m, r, 2, i + 1); ADV = 1 + EAN
            if (hi == 0x4400) return sprintf("move    %s,ccr", o)
            if (hi == 0x4600) return sprintf("move    %s,sr", o)
            return sprintf("%s%s", nm, o)
        }
        nm = hi == 0x4000 ? "negx" : hi == 0x4200 ? "clr" :
             hi == 0x4400 ? "neg" : "not"
        o = ea(m, r, szBytes(sz), i + 1); ADV = 1 + EAN
        return sprintf("%-7s %s", nm "." szn2(sz), o)
    }
    return dcw(op)
}

function dis5(i, op,   m, r, sz, cc, o, d, tgt) {
    m = and(rshift(op, 3), 7); r = and(op, 7)
    sz = and(rshift(op, 6), 3)
    cc = and(rshift(op, 8), 15)
    if (sz == 3) {
        if (m == 1) {
            ADV = 2
            tgt = VA + (i + 1) * 2 + sx16(W[i + 1])
            return sprintf("db%-5s d%d,%s", CCS[cc + 1], r, hx(tgt))
        }
        if (m == 7 && r >= 2 && r <= 4) {     # TRAPcc: no effective address
            ADV = (r == 2) ? 2 : (r == 3) ? 3 : 1
            if (r == 2) return sprintf("trap%-3s #$%04x", CCS[cc+1], W[i+1])
            if (r == 3) return sprintf("trap%-3s #$%04x%04x", CCS[cc+1],
                                       W[i+1], W[i+2])
            return sprintf("trap%s", CCS[cc + 1])
        }
        o = ea(m, r, 1, i + 1); ADV = 1 + EAN
        return sprintf("s%-6s %s", CCS[cc + 1], o)
    }
    d = and(rshift(op, 9), 7); if (d == 0) d = 8
    o = ea(m, r, szBytes(sz), i + 1); ADV = 1 + EAN
    return sprintf("%-7s #%d,%s",
                   (and(op, 256) ? "subq." : "addq.") szn2(sz), d, o)
}

function dis6(i, op,   cc, d, tgt) {
    cc = and(rshift(op, 8), 15); d = and(op, 255)
    if (d == 0)        { ADV = 2; tgt = VA + (i+1)*2 + sx16(W[i+1]) }
    else if (d == 255) { ADV = 3
                         tgt = VA + (i+1)*2 + sx32(W[i+1]*65536 + W[i+2]) }
    else               { ADV = 1; tgt = VA + (i+1)*2 + sx8(d) }
    return sprintf("b%-6s %s", CCB[cc + 1], hx(tgt))
}

function disOr(i, op,   m, r, dn, opm, sz, o) {
    m = and(rshift(op, 3), 7); r = and(op, 7); dn = and(rshift(op, 9), 7)
    opm = and(rshift(op, 6), 7)
    if (and(op, 0xF1F0) == 0x8140 || and(op, 0xF1F0) == 0x8180) {
        ADV = 2                              # PACK / UNPK: adjustment word
        return sprintf("%-7s %s,%s,#$%04x",
                       and(op, 0x0040) ? "pack" : "unpk",
                       and(op, 8) ? sprintf("-(a%d)", r) : sprintf("d%d", r),
                       and(op, 8) ? sprintf("-(a%d)", dn) : sprintf("d%d", dn),
                       W[i + 1])
    }
    if (and(op, 0xF1F0) == 0x8100)
        return sprintf("sbcd    %s,%s",
                       and(op, 8) ? sprintf("-(a%d)", r) : sprintf("d%d", r),
                       and(op, 8) ? sprintf("-(a%d)", dn) : sprintf("d%d", dn))
    if (opm == 3 || opm == 7) {
        o = ea(m, r, 2, i + 1); ADV = 1 + EAN
        return sprintf("%-7s %s,d%d", opm == 3 ? "divu.w" : "divs.w", o, dn)
    }
    sz = and(opm, 3)
    o = ea(m, r, szBytes(sz), i + 1); ADV = 1 + EAN
    if (opm < 4) return sprintf("%-7s %s,d%d", "or." szn2(sz), o, dn)
    return sprintf("%-7s d%d,%s", "or." szn2(sz), dn, o)
}

function disAnd(i, op,   m, r, dn, opm, sz, o) {
    m = and(rshift(op, 3), 7); r = and(op, 7); dn = and(rshift(op, 9), 7)
    opm = and(rshift(op, 6), 7)
    if (and(op, 0xF1F8) == 0xC140) return sprintf("exg     d%d,d%d", dn, r)
    if (and(op, 0xF1F8) == 0xC148) return sprintf("exg     a%d,a%d", dn, r)
    if (and(op, 0xF1F8) == 0xC188) return sprintf("exg     d%d,a%d", dn, r)
    if (and(op, 0xF1F0) == 0xC100)
        return sprintf("abcd    %s,%s",
                       and(op, 8) ? sprintf("-(a%d)", r) : sprintf("d%d", r),
                       and(op, 8) ? sprintf("-(a%d)", dn) : sprintf("d%d", dn))
    if (opm == 3 || opm == 7) {
        o = ea(m, r, 2, i + 1); ADV = 1 + EAN
        return sprintf("%-7s %s,d%d", opm == 3 ? "mulu.w" : "muls.w", o, dn)
    }
    sz = and(opm, 3)
    o = ea(m, r, szBytes(sz), i + 1); ADV = 1 + EAN
    if (opm < 4) return sprintf("%-7s %s,d%d", "and." szn2(sz), o, dn)
    return sprintf("%-7s d%d,%s", "and." szn2(sz), dn, o)
}

function disAddSub(i, op, nm,   m, r, dn, opm, sz, o) {
    m = and(rshift(op, 3), 7); r = and(op, 7); dn = and(rshift(op, 9), 7)
    opm = and(rshift(op, 6), 7)
    if (opm == 3 || opm == 7) {
        o = ea(m, r, opm == 3 ? 2 : 4, i + 1); ADV = 1 + EAN
        return sprintf("%-7s %s,a%d", nm "a." (opm == 3 ? "w" : "l"), o, dn)
    }
    sz = and(opm, 3)
    if (opm >= 4 && and(op, 0x0030) == 0)    # ADDX/SUBX
        return sprintf("%-7s %s,%s", nm "x." szn2(sz),
                       and(op, 8) ? sprintf("-(a%d)", r) : sprintf("d%d", r),
                       and(op, 8) ? sprintf("-(a%d)", dn) : sprintf("d%d", dn))
    o = ea(m, r, szBytes(sz), i + 1); ADV = 1 + EAN
    if (opm < 4) return sprintf("%-7s %s,d%d", nm "." szn2(sz), o, dn)
    return sprintf("%-7s d%d,%s", nm "." szn2(sz), dn, o)
}

function disCmp(i, op,   m, r, dn, opm, sz, o) {
    m = and(rshift(op, 3), 7); r = and(op, 7); dn = and(rshift(op, 9), 7)
    opm = and(rshift(op, 6), 7)
    if (and(op, 0xF138) == 0xB108)
        return sprintf("%-7s (a%d)+,(a%d)+", "cmpm." szn2(and(opm, 3)), r, dn)
    if (opm == 3 || opm == 7) {
        o = ea(m, r, opm == 3 ? 2 : 4, i + 1); ADV = 1 + EAN
        return sprintf("%-7s %s,a%d", opm == 3 ? "cmpa.w" : "cmpa.l", o, dn)
    }
    sz = and(opm, 3)
    o = ea(m, r, szBytes(sz), i + 1); ADV = 1 + EAN
    if (opm < 4) return sprintf("%-7s %s,d%d", "cmp." szn2(sz), o, dn)
    return sprintf("%-7s d%d,%s", "eor." szn2(sz), dn, o)
}

function shname(tp, left) {
    return (tp == 0 ? "as" : tp == 1 ? "ls" : tp == 2 ? "rox" : "ro") \
           (left ? "l" : "r")
}

# 68020 bitfield ops: one extension word of their own PLUS whatever the
# effective address needs. The Mac OS ROM does use them (bfextu appears in
# the .ATALoad install path), and reading one as two shift instructions
# happens to consume the same two words — which is precisely how a decoder
# stays plausible while being wrong.
function disBf(i, op, e, tp, nm, dn, off, wid, o, fld, ot, wt) {
    e = W[i + 1]
    tp = and(rshift(op, 8), 7)
    nm = BF[tp + 1]
    dn  = and(rshift(e, 12), 7)
    off = and(rshift(e, 6), 31)
    wid = and(e, 31)
    if (and(e, 2048)) ot = sprintf("d%d", and(off, 7))
    else              ot = sprintf("%d", off)
    if (and(e, 32))   wt = sprintf("d%d", and(wid, 7))
    else              wt = sprintf("%d", wid ? wid : 32)
    fld = sprintf("{%s:%s}", ot, wt)
    o = ea(and(rshift(op, 3), 7), and(op, 7), 1, i + 2); ADV = 2 + EAN
    if (tp == 7) return sprintf("%-7s d%d,%s%s", nm, dn, o, fld)
    if (tp == 1 || tp == 3 || tp == 5)
        return sprintf("%-7s %s%s,d%d", nm, o, fld, dn)
    return sprintf("%-7s %s%s", nm, o, fld)
}

function disShift(i, op,   sz, cr, o, nm) {
    sz = and(rshift(op, 6), 3)
    if (sz == 3) {
        if (and(op, 0x0800)) return disBf(i, op)
        nm = shname(and(rshift(op, 9), 3), and(op, 256))
        o = ea(and(rshift(op, 3), 7), and(op, 7), 2, i + 1); ADV = 1 + EAN
        return sprintf("%-7s %s", nm ".w", o)
    }
    cr = and(rshift(op, 9), 7)
    nm = shname(and(rshift(op, 3), 3), and(op, 256))
    if (and(op, 32))
        return sprintf("%-7s d%d,d%d", nm "." szn2(sz), cr, and(op, 7))
    return sprintf("%-7s #%d,d%d", nm "." szn2(sz), cr ? cr : 8, and(op, 7))
}

function dis(i,   op, g) {
    op = W[i]
    ADV = 1
    g = rshift(op, 12)
    if (g == 0) return dis0(i, op)
    if (g == 1 || g == 2 || g == 3) return disMove(i, op, g)
    if (g == 4) return dis4(i, op)
    if (g == 5) return dis5(i, op)
    if (g == 6) return dis6(i, op)
    if (g == 7) {
        if (and(op, 256)) return dcw(op)
        return sprintf("moveq   #%d,d%d", sx8(and(op, 255)),
                       and(rshift(op, 9), 7))
    }
    if (g == 8)  return disOr(i, op)
    if (g == 9)  return disAddSub(i, op, "sub")
    if (g == 10) return sprintf("_trap   $%04x", op)
    if (g == 11) return disCmp(i, op)
    if (g == 12) return disAnd(i, op)
    if (g == 13) return disAddSub(i, op, "add")
    if (g == 14) return disShift(i, op)
    return dcw(op)                           # F-line
}

BEGIN {
    split("ra sr hi ls cc cs ne eq vc vs pl mi ge lt gt le", CCB, " ")
    split("t f hi ls cc cs ne eq vc vs pl mi ge lt gt le", CCS, " ")
    split("ori andi subi addi ? eori cmpi ?", IMM, " ")
    split("bftst bfextu bfchg bfexts bfclr bfffo bfset bfins", BF, " ")
    UNK = 0
}
{
    n = int(length($0) / 4)
    for (i = 0; i < n; i++)
        W[i] = strtonum("0x" substr($0, i * 4 + 1, 4))
    i = 0
    count = 0
    tail = 0
    while (i < n) {
        a = VA + i * 2
        t = dis(i)
        if (i + ADV > n) {
            printf "  %08x: %-24s ; TRUNCATED: needs %d words, %d left\n",
                   a, t, ADV, n - i
            tail = (n - i) * 2
            break
        }
        raw = ""
        for (k = 0; k < ADV; k++)
            raw = raw sprintf("%04x ", W[i + k])
        printf "  %08x: %-24s %s\n", a, raw, t
        i += ADV
        count++
    }
    printf "-- %d instructions, %d of %d bytes consumed",
           count, i * 2, n * 2
    if (tail) printf " (%d-byte tail is mid-instruction)", tail
    if (UNK) printf ", %d undecoded word(s)", UNK
    printf "\n"
    if (tail && STRICT) {
        printf "-- LENGTH CHECK FAILED: the window does not end on an "
        printf "instruction boundary\n"
        exit 1
    }
}
AWKEOF

if [ "${1:-}" = "--selftest" ]; then
  # Hand-verified encodings with hand-counted lengths. These are the check
  # the first version of this script did not have: if the effective-address
  # length logic drifts, the byte total stops matching and this fails.
  HEX="4e56fff8"        # link    a6,#-8                       4
  HEX+="48e73038"       # movem.l d2-d4/a2-a4,-(sp)            4
  HEX+="102eff78"       # move.b  -136(a6),d0                  4  <- the trap
  HEX+="0c400010"       # cmpi.w  #$10,d0                      4
  HEX+="6712"           # beq     +0x12                        2
  HEX+="4eba0102"       # jsr     d16(pc)                      4
  HEX+="4eb900c01234"   # jsr     ($00c01234).l                6
  HEX+="2f2e000c"       # move.l  12(a6),-(sp)                 4
  HEX+="51c9fffc"       # dbf     d1,back                      4
  HEX+="7e00"           # moveq   #0,d7                        2
  HEX+="a71e"           # _trap   $A71E                        2
  HEX+="4e75"           # rts                                  2
  HEX+="41fa0020"       # lea     d16(pc),a0                   4
  HEX+="13c000001234"   # move.b  d0,($00001234).l             6
  HEX+="0c6e000cff78"   # cmpi.w  #$c,-136(a6)                 6
  HEX+="207900c00000"   # movea.l ($00c00000).l,a0             6
  HEX+="4cdf0c04"       # movem.l (sp)+,d2/a2-a3               4
  HEX+="60000100"       # bra.w                                4
  HEX+="4a80"           # tst.l   d0                           2
  HEX+="b1ec0008"       # cmpa.l  8(a4),a0                     4
  HEX+="e588"           # lsl.l   #2,d0                        2
  HEX+="48781234"       # pea     ($1234).w                    4
  HEX+="303c0012"       # move.w  #$12,d0                      4
  HEX+="2f3c00000100"   # move.l  #$100,-(sp)                  6
  # 68020 forms the Mac OS ROM actually uses, and whose lengths the first
  # version of this script got wrong. bfextu appears in .ATALoad.
  HEX+="e9d30242"       # bfextu  (a3){9:2},d0                 4
  HEX+="4c001000"       # mulu.l  d0,ext                       4
  HEX+="4e740008"       # rtd     #8                           4
  HEX+="480800000010"   # link.l  a0,#16                       6
  EXPECT_INSNS=28
  EXPECT_BYTES=112
  OUT=$(printf '%s' "$HEX" |
        awk -v VA=$((0xffd90000)) -v STRICT=1 -f "$AWKSRC")
  echo "$OUT"
  GOT=$(printf '%s' "$OUT" | sed -n 's/^-- \([0-9]*\) instructions, \([0-9]*\) of.*/\1 \2/p')
  set -- $GOT
  FAIL=0
  [ "${1:-}" = "$EXPECT_INSNS" ] || { echo "SELFTEST FAIL: $1 instructions, expected $EXPECT_INSNS"; FAIL=1; }
  [ "${2:-}" = "$EXPECT_BYTES" ] || { echo "SELFTEST FAIL: $2 bytes, expected $EXPECT_BYTES"; FAIL=1; }
  printf '%s' "$OUT" | grep -q 'move.b  -136(a6),d0' ||
    { echo "SELFTEST FAIL: 102e ff78 did not decode as move.b -136(a6),d0"; FAIL=1; }
  printf '%s' "$OUT" | grep -q 'undecoded' &&
    { echo "SELFTEST FAIL: undecoded words in a known-good stream"; FAIL=1; }
  [ "$FAIL" = 0 ] && echo "SELFTEST PASSED: $EXPECT_INSNS instructions, $EXPECT_BYTES bytes, all lengths derived"
  exit $FAIL
fi

DUMP=${1:?usage: dis68k.sh <ram-dump> <va> [bytes] [--strict] | --selftest}
VA=$((${2:?usage: dis68k.sh <ram-dump> <va> [bytes] [--strict]}))
N=${3:-96}
STRICT=0
[ "${4:-}" = "--strict" ] && STRICT=1
PA=$((VA - 0xFF000000))

xxd -s "$PA" -l "$N" -p "$DUMP" | tr -d '\n' |
  awk -v VA="$VA" -v STRICT="$STRICT" -f "$AWKSRC"
