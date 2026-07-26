/* Oracle corpus #1 — a grab-bag of integer, float, and AltiVec code shapes.
 * Compiled freestanding for ppc32be; also runs under ppcrun from P1 on:
 * prints results via the MMIO console and exits via the MMIO exit port. */

#include <altivec.h>

#define MMIO_PUTC (*(volatile unsigned char*)0xF0000000u)
#define MMIO_EXIT (*(volatile unsigned int*)0xF0000004u)

__asm__(".globl _start\n"
        "_start:\n"
        "  lis 1, 0x00F0\n"
        "  li 0, 0\n"
        "  bl cmain\n"
        "1: b 1b\n");

static void putc1(char c) { MMIO_PUTC = (unsigned char)c; }

static void puthex(unsigned int v)
{
    for (int i = 28; i >= 0; i -= 4) {
        unsigned d = (v >> i) & 0xF;
        putc1((char)(d < 10 ? '0' + d : 'a' + d - 10));
    }
    putc1('\n');
}

static unsigned int crc32(const unsigned char* p, unsigned n)
{
    unsigned int crc = 0xFFFFFFFFu;
    while (n--) {
        crc ^= *p++;
        for (int k = 0; k < 8; k++)
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1)));
    }
    return ~crc;
}

static int sieve(int limit)
{
    static unsigned char comp[4096];
    int count = 0;
    for (int i = 0; i < limit; i++)
        comp[i] = 0;
    for (int i = 2; i < limit; i++) {
        if (!comp[i]) {
            count++;
            for (int j = i + i; j < limit; j += i)
                comp[j] = 1;
        }
    }
    return count;
}

static unsigned long long mixmul(unsigned a, unsigned b)
{
    unsigned long long p = (unsigned long long)a * b;
    p ^= p >> 17;
    p *= 0x9E3779B97F4A7C15ull;
    return p;
}

static double fmix(double x, float y)
{
    double a = x * 1.5 + y;
    double b = a / 3.25 - x * y;
    float s = (float)(a + b) * 0.5f;
    return a * b + s;
}

static int vsum(void)
{
    vector signed int acc = vec_splat_s32(0);
    vector signed int step = vec_splat_s32(3);
    for (int i = 0; i < 64; i++)
        acc = vec_add(acc, step);
    vector signed int sh = vec_sld(acc, acc, 8);
    acc = vec_add(acc, sh);
    sh = vec_sld(acc, acc, 4);
    acc = vec_add(acc, sh);
    return vec_extract(acc, 0);
}

void cmain(void)
{
    static unsigned char buf[256];
    for (unsigned i = 0; i < sizeof buf; i++)
        buf[i] = (unsigned char)(i * 7 + 3);

    puthex(crc32(buf, sizeof buf));
    puthex((unsigned)sieve(4096));
    unsigned long long m = mixmul(0xDEADBEEFu, 0x12345679u);
    puthex((unsigned)(m >> 32));
    puthex((unsigned)m);
    double f = fmix(3.75, 2.5f);
    unsigned long long fb;
    __builtin_memcpy(&fb, &f, 8);
    puthex((unsigned)(fb >> 32));
    puthex((unsigned)fb);
    puthex((unsigned)vsum());

    MMIO_EXIT = 0;
}
