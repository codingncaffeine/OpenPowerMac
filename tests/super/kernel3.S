/* Chip-layer test kernel (P6): nap mode with the time base ticking.
 * Installs a DEC handler, programs a short decrementer, enables EE, sets
 * HID0[NAP], then sets MSR[POW] — the core sleeps (no instructions) until
 * the decrementer wakes it into the handler; rfi resumes after the mtmsr.
 * The TB delta across the nap proves time advanced while asleep.
 * Expected stdout: N1 T1 OK   (one per line)
 */

.set SPR_SRR0, 26
.set SPR_SRR1, 27
.set SPR_DEC, 22
.set SPR_HID0, 1008
.set SPR_TBL_READ, 268

.globl _start
_start:
    lis 31, 0xF000             /* MMIO base */

    mfmsr 3                    /* vectors low */
    li 4, 0x40
    andc 3, 3, 4
    mtmsr 3

    lis 5, dec_handler@ha
    addi 5, 5, dec_handler@l
    addis 5, 5, 0x4800
    addi 5, 5, 2
    li 6, 0x0900
    stw 5, 0(6)

    li 23, 0                   /* dec count */

    mftb 20                    /* TB before nap */

    li 10, 0x80
    mtspr SPR_DEC, 10

    mfmsr 3
    ori 3, 3, 0x8000           /* EE */
    mtmsr 3

    mfspr 5, SPR_HID0
    oris 5, 5, 0x0040          /* HID0[NAP] */
    mtspr SPR_HID0, 5

    mfmsr 3
    oris 3, 3, 0x0004          /* MSR[POW]: enter nap */
    mtmsr 3

    /* resumes here after the DEC wake + rfi */
    mftb 21
    subf 22, 20, 21            /* TB delta across the nap */

    li 3, 'N'
    mr 4, 23
    bl print2

    li 3, 'T'
    li 4, 1
    cmpwi 22, 0x20             /* >= 32 TB ticks slept */
    bge 1f
    li 4, 0
1:
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

dec_handler:
    addi 23, 23, 1
    lis 10, 0x7FFF             /* park the decrementer far away */
    mtspr SPR_DEC, 10
    rfi
