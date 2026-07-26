/* Dual-target shim for oracle corpus programs.
 * Guest build (default): freestanding ppc32be, output via ppcrun's MMIO rig.
 * Host build (-DHOST): normal hosted build; same cmain(), stdout via stdio.
 * A program's observable output must be bit-identical on both. */
#pragma once

void cmain(void);

#ifdef HOST
#include <stdio.h>
static void putc1(char c) { putchar(c); }
static void done(unsigned code) { (void)code; }
int main(void)
{
    cmain();
    return 0;
}
#else
#define MMIO_PUTC (*(volatile unsigned char*)0xF0000000u)
#define MMIO_EXIT (*(volatile unsigned int*)0xF0000004u)
static void putc1(char c) { MMIO_PUTC = (unsigned char)c; }
static void done(unsigned code) { MMIO_EXIT = code; }
/* Freestanding: the compiler lowers aggregate inits/copies to these. */
void* memset(void* d, int c, __SIZE_TYPE__ n)
{
    unsigned char* p = (unsigned char*)d;
    while (n--)
        *p++ = (unsigned char)c;
    return d;
}
void* memcpy(void* d, const void* s, __SIZE_TYPE__ n)
{
    unsigned char* p = (unsigned char*)d;
    const unsigned char* q = (const unsigned char*)s;
    while (n--)
        *p++ = *q++;
    return d;
}
__asm__(".globl _start\n"
        "_start:\n"
        "  lis 1, 0x00F0\n"
        "  li 0, 0\n"
        "  bl cmain\n"
        "  lis 3, 0xF000\n"
        "  li 4, 0\n"
        "  stw 4, 4(3)\n"
        "1: b 1b\n");
#endif

static void puthex(unsigned int v)
{
    int i;
    for (i = 28; i >= 0; i -= 4) {
        unsigned d = (v >> i) & 0xFu;
        putc1((char)(d < 10 ? '0' + d : 'a' + (d - 10)));
    }
    putc1('\n');
}
