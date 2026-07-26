/* Oracle corpus #1 — integer, float, and AltiVec code shapes together.
 * Guest-runnable once P4 (FPU) and P5 (AltiVec) land; disassembler corpus
 * from P0 on. */
#ifndef HOST
#include <altivec.h>
#endif
#include "shim.h"

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
#ifdef HOST
    /* Scalar reference of the AltiVec kernel below. */
    int acc[4] = {0, 0, 0, 0};
    for (int i = 0; i < 64; i++)
        for (int l = 0; l < 4; l++)
            acc[l] += 3;
    int t[4];
    for (int l = 0; l < 4; l++)
        t[l] = acc[l] + acc[(l + 2) & 3];
    int r[4];
    for (int l = 0; l < 4; l++)
        r[l] = t[l] + t[(l + 1) & 3];
    return r[0];
#else
    vector signed int acc = vec_splat_s32(0);
    vector signed int step = vec_splat_s32(3);
    for (int i = 0; i < 64; i++)
        acc = vec_add(acc, step);
    vector signed int sh = vec_sld(acc, acc, 8);
    acc = vec_add(acc, sh);
    sh = vec_sld(acc, acc, 4);
    acc = vec_add(acc, sh);
    return vec_extract(acc, 0);
#endif
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

    done(0);
}
