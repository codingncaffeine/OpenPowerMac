/* Demand-paging test kernel (P3 exit proof), xnu-shaped:
 * BAT0 identity-maps the first 256 MB (bl=0x7ff) for code/data, DBAT1 maps
 * the MMIO window at 0xF0000000 cache-inhibited+guarded, all 16 SRs get
 * vsid=index, SDR1 points at a zeroed hashed page table — then IR/DR go on.
 *
 *   M: load from unmapped VA 0x40000000 -> DSI (miss); the handler, running
 *      real-mode (exception entry clears IR/DR), builds the PTE and rfi's;
 *      the retried load returns the seeded marker.
 *   C: store to that page sets C; the store lands at the translated PA and
 *      the PTE reads back with R|C set.
 *   P: PP=3 page: load fine, store -> DSI (protection), handler skips it,
 *      target stays unchanged.
 *   F: branch to unmapped VA 0x60000000 -> ISI (miss); handler builds the
 *      PTE; retried fetch executes "li r9,7; blr" placed there beforehand.
 *
 * Expected stdout: M1 C1 P1 F1 OK   (one per line)
 *
 * Registers: r31 MMIO base, r20/r21/r22 = miss/prot/isi counters,
 * r14/r15/r16 test VAs, r28-r30 handler scratch, r3-r8 main scratch.
 */

.set SPR_DSISR, 18
.set SPR_DAR,   19
.set SPR_SDR1,  25
.set SPR_SRR0,  26
.set SPR_SRR1,  27
.set SPR_CTR,    9
.set SPR_IBAT0U, 528
.set SPR_IBAT0L, 529
.set SPR_DBAT0U, 536
.set SPR_DBAT0L, 537
.set SPR_DBAT1U, 538
.set SPR_DBAT1L, 539

.globl _start
_start:
    lis 31, 0xF000             /* MMIO base */

    /* vectors to 0x00000000: clear MSR[IP] */
    mfmsr 3
    li 4, 0x40
    andc 3, 3, 4
    mtmsr 3

    /* install "ba handler" words at the DSI/ISI vectors */
    lis 5, dsi_handler@ha
    addi 5, 5, dsi_handler@l
    addis 5, 5, 0x4800
    addi 5, 5, 2
    li 6, 0x0300
    stw 5, 0(6)

    lis 5, isi_handler@ha
    addi 5, 5, isi_handler@l
    addis 5, 5, 0x4800
    addi 5, 5, 2
    li 6, 0x0400
    stw 5, 0(6)

    /* SDR1: hashed table at physical 0x90000, HTABMASK=0 (RAM is zeroed) */
    lis 3, 0x0009
    mtspr SPR_SDR1, 3

    /* all 16 segment registers: vsid = index, Ks=Kp=T=N=0 */
    li 3, 0
srloop:
    rlwinm 4, 3, 28, 0, 3      /* rB = index << 28 */
    mtsrin 3, 4
    addi 3, 3, 1
    cmpwi 3, 16
    blt srloop

    /* seed physical pages while still in real mode:
       0x200000 = demand-page marker, 0x201000 = read-only page content,
       0x202000 = "li r9,7; blr" for the fetch test */
    lis 3, 0x0020
    lis 4, 0x600D
    ori 4, 4, 0xF00D
    stw 4, 0(3)
    lis 3, 0x0020
    ori 3, 3, 0x1000
    lis 4, 0x0BAD
    ori 4, 4, 0xC0DE
    stw 4, 0(3)
    lis 3, 0x0020
    ori 3, 3, 0x2000
    lis 4, 0x3920               /* li r9,7 = 0x39200007 */
    ori 4, 4, 0x0007
    stw 4, 0(3)
    lis 4, 0x4E80               /* blr = 0x4E800020 */
    ori 4, 4, 0x0020
    stw 4, 4(3)

    /* BAT0 I+D: identity, BEPI 0, BL 0x7FF (256 MB), Vs, RW */
    li 3, 0x1FFE
    li 4, 0x0002
    mtspr SPR_IBAT0U, 3
    mtspr SPR_IBAT0L, 4
    mtspr SPR_DBAT0U, 3
    mtspr SPR_DBAT0L, 4
    /* DBAT1: MMIO window 0xF0000000, 256 MB, cache-inhibited + guarded */
    lis 3, 0xF000
    ori 3, 3, 0x1FFE
    mtspr SPR_DBAT1U, 3
    lis 3, 0xF000
    ori 3, 3, 0x002A
    mtspr SPR_DBAT1L, 3

    li 20, 0                   /* DSI miss count */
    li 21, 0                   /* DSI protection count */
    li 22, 0                   /* ISI count */

    /* translation on */
    mfmsr 3
    ori 3, 3, 0x0030           /* IR|DR */
    mtmsr 3
    isync

    /* --- M: demand-paged load --- */
    lis 14, 0x4000             /* VA 0x40000000: vsid 4, pageIndex 0 */
    lwz 3, 0(14)
    lis 5, 0x600D
    ori 5, 5, 0xF00D
    cmpw 3, 5
    bne fail
    li 3, 'M'
    mr 4, 20
    bl print2

    /* --- C: store sets C; translation redirects to PA 0x200000 --- */
    lis 6, 0x1111
    ori 6, 6, 0x1111
    stw 6, 4(14)
    lis 7, 0x0020              /* the same page via the identity BAT */
    lwz 8, 4(7)
    cmpw 8, 6
    bne fail
    lis 7, 0x0009
    ori 7, 7, 0x0104           /* PTE w1 for the demand page */
    lwz 8, 0(7)
    andi. 8, 8, 0x0180         /* R|C */
    cmpwi 8, 0x0180
    bne fail
    li 3, 'C'
    li 4, 1
    bl print2

    /* --- P: read-only page (PP=3, key 0): load ok, store faults+skipped --- */
    lis 7, 0x0009
    ori 7, 7, 0x0140           /* PTEG for vsid 5, pageIndex 0 (hash 5) */
    lis 8, 0x8000
    ori 8, 8, 0x0280           /* w0 = V | vsid 5 | api 0 */
    stw 8, 0(7)
    lis 8, 0x0020
    ori 8, 8, 0x1003           /* w1 = RPN 0x201000 | PP 3 */
    stw 8, 4(7)
    lis 15, 0x5000             /* VA 0x50000000 */
    lwz 3, 0(15)
    lis 5, 0x0BAD
    ori 5, 5, 0xC0DE
    cmpw 3, 5
    bne fail
    stw 6, 0(15)               /* -> DSI protection, handler skips */
    lwz 3, 0(15)
    cmpw 3, 5                  /* unchanged */
    bne fail
    li 3, 'P'
    mr 4, 21
    bl print2

    /* --- F: fetch from unmapped VA -> ISI -> handler maps -> executes --- */
    li 9, 0
    lis 16, 0x6000             /* VA 0x60000000: vsid 6, pageIndex 0 */
    mtspr SPR_CTR, 16
    bctrl                      /* li r9,7; blr */
    cmpwi 9, 7
    bne fail
    li 3, 'F'
    mr 4, 22
    bl print2

    li 3, 'O'
    stb 3, 0(31)
    li 3, 'K'
    stb 3, 0(31)
    li 3, 10
    stb 3, 0(31)

    li 4, 0
    stw 4, 4(31)               /* MMIO exit 0 */
hang:
    b hang

print2:                        /* r3 = letter, r4 = small count */
    stb 3, 0(31)
    addi 4, 4, 0x30
    stb 4, 0(31)
    li 5, 10
    stb 5, 0(31)
    blr

fail:                          /* works translated (DBAT1) or real-mode */
    li 3, 'N'
    stb 3, 0(31)
    li 3, 'O'
    stb 3, 0(31)
    li 3, 10
    stb 3, 0(31)
    li 4, 1
    stw 4, 4(31)
1:
    b 1b

dsi_handler:                   /* real mode: exception entry cleared IR/DR */
    mfspr 29, SPR_DSISR
    andis. 30, 29, 0x4000      /* miss? */
    bne 1f
    andis. 30, 29, 0x0800      /* protection? */
    bne 2f
    b fail
1:                             /* only VA 0x40000000 may demand-fault */
    mfspr 30, SPR_DAR
    lis 28, 0x4000
    cmpw 30, 28
    bne fail
    lis 28, 0x0009
    ori 28, 28, 0x0100         /* PTEG for vsid 4, pageIndex 0 (hash 4) */
    lis 30, 0x8000
    ori 30, 30, 0x0200         /* w0 = V | vsid 4 | api 0 */
    stw 30, 0(28)
    lis 30, 0x0020
    ori 30, 30, 0x0002         /* w1 = RPN 0x200000 | PP 2 */
    stw 30, 4(28)
    addi 20, 20, 1
    rfi                        /* retry the load, now mapped */
2:
    mfspr 30, SPR_SRR0
    addi 30, 30, 4             /* skip the store */
    mtspr SPR_SRR0, 30
    addi 21, 21, 1
    rfi

isi_handler:
    mfspr 29, SPR_SRR1
    andis. 30, 29, 0x4000      /* fetch miss? */
    beq fail
    mfspr 30, SPR_SRR0
    lis 28, 0x6000             /* must be the F-test target */
    cmpw 30, 28
    bne fail
    lis 28, 0x0009
    ori 28, 28, 0x0180         /* PTEG for vsid 6, pageIndex 0 (hash 6) */
    lis 30, 0x8000
    ori 30, 30, 0x0300         /* w0 = V | vsid 6 | api 0 */
    stw 30, 0(28)
    lis 30, 0x0020
    ori 30, 30, 0x2002         /* w1 = RPN 0x202000 | PP 2 */
    stw 30, 4(28)
    addi 22, 22, 1
    rfi                        /* retry the fetch */
