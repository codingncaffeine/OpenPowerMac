/* Supervisor test kernel (P2 exit proof).
 * Clears MSR[IP], installs absolute-branch handlers at the low vectors,
 * then exercises: sc, illegal (tlbia), trap (twi), alignment (lmw odd),
 * and decrementer preemption of a spin loop with rfi resume.
 * Output via the ppcrun MMIO rig; expected stdout:
 *   S1 I1 T1 D3 A1 OK   (one per line)
 */

.set SPR_SRR0, 26
.set SPR_SRR1, 27
.set SPR_DEC, 22

.globl _start
_start:
    lis 1, 0x00F0              /* stack */
    lis 31, 0xF000             /* MMIO base */

    /* vectors to 0x00000000: clear MSR[IP] */
    mfmsr 3
    li 4, 0x40
    andc 3, 3, 4
    mtmsr 3

    /* install "ba handler" words at the vectors */
    lis 5, sc_handler@ha
    addi 5, 5, sc_handler@l
    addis 5, 5, 0x4800
    addi 5, 5, 2
    li 6, 0x0C00
    stw 5, 0(6)

    lis 5, prog_handler@ha
    addi 5, 5, prog_handler@l
    addis 5, 5, 0x4800
    addi 5, 5, 2
    li 6, 0x0700
    stw 5, 0(6)

    lis 5, dec_handler@ha
    addi 5, 5, dec_handler@l
    addis 5, 5, 0x4800
    addi 5, 5, 2
    li 6, 0x0900
    stw 5, 0(6)

    lis 5, align_handler@ha
    addi 5, 5, align_handler@l
    addis 5, 5, 0x4800
    addi 5, 5, 2
    li 6, 0x0600
    stw 5, 0(6)

    li 20, 0                   /* sc count */
    li 21, 0                   /* illegal count */
    li 22, 0                   /* trap count */
    li 23, 0                   /* dec ticks */
    li 25, 0                   /* alignment count */

    /* --- sc --- */
    sc

    /* --- illegal: tlbia (not implemented on the 7400) --- */
    .long 0x7C0002E4

    /* --- trap --- */
    twi 31, 0, 0

    /* --- alignment: lmw from an odd address --- */
    lis 9, 0
    ori 9, 9, 0x2001
    lmw 29, 0(9)

    /* --- decrementer preemption of a spin loop --- */
    li 10, 0x100
    mtspr SPR_DEC, 10
    mfmsr 3
    ori 3, 3, 0x8000           /* EE */
    mtmsr 3
    li 24, 0
spin:
    addi 24, 24, 1
    cmpwi 23, 3
    blt spin
    mfmsr 3
    li 4, 0
    ori 4, 4, 0x8000
    andc 3, 3, 4               /* EE off */
    mtmsr 3

    /* --- report --- */
    li 3, 'S'
    mr 4, 20
    bl print2
    li 3, 'I'
    mr 4, 21
    bl print2
    li 3, 'T'
    mr 4, 22
    bl print2
    li 3, 'D'
    mr 4, 23
    bl print2
    li 3, 'A'
    mr 4, 25
    bl print2

    li 3, 'O'
    stb 3, 0(31)
    li 3, 'K'
    stb 3, 0(31)
    li 3, 10
    stb 3, 0(31)

    li 4, 0
    stw 4, 4(31)               /* MMIO exit */
hang:
    b hang

print2:                        /* r3 = letter, r4 = small count */
    stb 3, 0(31)
    addi 4, 4, 0x30
    stb 4, 0(31)
    li 5, 10
    stb 5, 0(31)
    blr

sc_handler:
    addi 20, 20, 1
    rfi                        /* SRR0 already holds the next instruction */

prog_handler:
    mfspr 10, SPR_SRR1
    andis. 11, 10, 0x0008      /* illegal bit */
    beq 1f
    addi 21, 21, 1
1:
    andis. 11, 10, 0x0002      /* trap bit */
    beq 2f
    addi 22, 22, 1
2:
    mfspr 10, SPR_SRR0
    addi 10, 10, 4             /* skip the faulting instruction */
    mtspr SPR_SRR0, 10
    rfi

dec_handler:
    addi 23, 23, 1
    li 10, 0x100
    mtspr SPR_DEC, 10
    rfi

align_handler:
    addi 25, 25, 1
    mfspr 10, SPR_SRR0
    addi 10, 10, 4
    mtspr SPR_SRR0, 10
    rfi
