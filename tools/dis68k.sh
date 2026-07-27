#!/bin/bash
# A 68K disassembler for the opcodes this ROM actually uses.
#
# We have a PowerPC disassembler (ppcrun --disasm) but had nothing for 68K,
# so every 68K decode in the ATA dig was done by reading hex by hand — which
# produced a wrong JSR base once and three watches placed on addresses that
# are never instruction boundaries. This covers the forms that appear in the
# Mac OS ROM's driver code and prints "dc.w" for anything it does not know,
# so an unrecognised word is visible rather than silently mis-decoded.
#
#   tools/dis68k.sh <ram-dump> <va> [count-bytes]
#
# <va> is a ROM virtual address (ffcxxxxx/ffdxxxxx); the Mac OS ROM is
# staged at PA 0x00c00000, so PA = VA - 0xFF000000.
set -e
DUMP=${1:?usage: dis68k.sh <ram-dump> <va> [bytes]}
VA=$((${2:?usage: dis68k.sh <ram-dump> <va> [bytes]}))
N=${3:-96}
PA=$((VA - 0xFF000000))

xxd -s "$PA" -l "$N" -p "$DUMP" | tr -d '\n' | tr 'a-f' 'A-F' |
awk -v va="$VA" '
function hex(v) { return sprintf("%x", v) }
function w(i) { return strtonum("0x" substr(s, i*4+1, 4)) }
function sx16(v) { return v >= 32768 ? v - 65536 : v }
function sx8(v)  { return v >= 128 ? v - 256 : v }
function reglist(m,   r, o, i) {
    o = ""
    for (i = 0; i < 16; i++) if (m % 2**(i+1) >= 2**i)
        o = o (o ? "/" : "") (i < 8 ? "d" i : "a" (i-8))
    return o ? o : "-"
}
BEGIN { }
{
  s = $0
  n = length(s) / 4
  i = 0
  while (i < n) {
    op = w(i); a = va + i*2; t = ""; adv = 1
    if (op == 0x4E75) t = "rts"
    else if (op == 0x4E71) t = "nop"
    else if (op == 0x4E5E) t = "unlk a6"
    else if (op == 0x4E56) { t = sprintf("link a6,#%d", sx16(w(i+1))); adv = 2 }
    else if (op == 0x48E7) { t = sprintf("movem.l %s,-(sp)", reglist(w(i+1))); adv = 2 }
    else if (op == 0x4CDF) { t = sprintf("movem.l (sp)+,%s", reglist(w(i+1))); adv = 2 }
    else if (op == 0x4CEE) { t = sprintf("movem.l %d(a6),%s", sx16(w(i+2)), reglist(w(i+1))); adv = 3 }
    else if (op == 0x4EBA) { t = sprintf("jsr     %x(pc)   ; -> ffd9%04x", sx16(w(i+1)), (a + 2 + sx16(w(i+1))) % 65536); adv = 2 }
    else if (op == 0x4EFA) { t = sprintf("jmp     %x(pc)   ; -> ffd9%04x", sx16(w(i+1)), (a + 2 + sx16(w(i+1))) % 65536); adv = 2 }
    else if (op == 0x4EB9) { t = sprintf("jsr     $%04x%04x", w(i+1), w(i+2)); adv = 3 }
    else if (op == 0x4EF9) { t = sprintf("jmp     $%04x%04x", w(i+1), w(i+2)); adv = 3 }
    else if (op == 0x4EFB) { t = "jmp     (d8,pc,xn)"; adv = 2 }
    else if (int(op/16)*16 == 0x4EA8) { t = sprintf("jsr     %d(a%d)", sx16(w(i+1)), op % 8); adv = 2 }
    else if (op == 0x486E) { t = sprintf("pea     %d(a6)", sx16(w(i+1))); adv = 2 }
    else if (op == 0x41FA) { t = sprintf("lea     %x(pc),a0 ; -> ffd9%04x", sx16(w(i+1)), (a + 2 + sx16(w(i+1))) % 65536); adv = 2 }
    else if (op == 0x48C0) t = "ext.l   d0"
    else if (op == 0x4845) t = "swap    d5"
    else if (op == 0x51C9) { t = sprintf("dbf     d1,%x", (a + 2 + sx16(w(i+1))) % 65536); adv = 2 }
    else if (int(op/256) == 0x70) t = sprintf("moveq   #%d,d0", sx8(op % 256))
    else if (int(op/256) == 0x72) t = sprintf("moveq   #%d,d1", sx8(op % 256))
    else if (int(op/256) == 0x7E) t = sprintf("moveq   #%d,d7", sx8(op % 256))
    else if (int(op/4096) == 0x6) {
        cc = int(op/256) % 16; d = op % 256
        split("ra sr hi ls cc cs ne eq vc vs pl mi ge lt gt le", cn, " ")
        if (d == 0) { t = sprintf("b%-6s %x", cn[cc+1], (a + 2 + sx16(w(i+1))) % 65536); adv = 2 }
        else t = sprintf("b%-6s %x", cn[cc+1], (a + 2 + sx8(d)) % 65536)
    }
    else if (int(op/4096) == 0x1) t = "move.b  ..."
    else if (int(op/4096) == 0x2) t = "move.l  ..."
    else if (int(op/4096) == 0x3) t = "move.w  ..."
    else if (op == 0x0C00) { t = sprintf("cmpi.b  #%d,d0", w(i+1) % 256); adv = 2 }
    else if (int(op/256) == 0x0C) { t = sprintf("cmpi    #$%04x", w(i+1)); adv = 2 }
    else if (int(op/256) == 0x0A || int(op/256) == 0xA0 ||
             int(op/4096) == 0xA) t = sprintf("_trap   $%04x", op)
    else if (int(op/256) == 0x4A) t = "tst     ..."
    else t = sprintf("dc.w    $%04x", op)
    printf "  %08x: %-22s ; %04x\n", a, t, op
    i += adv
  }
}
'
