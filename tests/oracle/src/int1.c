/* Oracle corpus: integer-only torture. No floats, no vectors. */
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
    static unsigned char comp[8192];
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

static void qsort_u32(unsigned int* a, int lo, int hi)
{
    if (lo >= hi)
        return;
    unsigned int p = a[(lo + hi) / 2];
    int i = lo, j = hi;
    while (i <= j) {
        while (a[i] < p) i++;
        while (a[j] > p) j--;
        if (i <= j) {
            unsigned int t = a[i]; a[i] = a[j]; a[j] = t;
            i++; j--;
        }
    }
    qsort_u32(a, lo, j);
    qsort_u32(a, i, hi);
}

static unsigned int xorshift(unsigned int* s)
{
    unsigned int x = *s;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return *s = x;
}

static unsigned int mixdiv(unsigned int seed)
{
    unsigned int s = seed | 1u;
    unsigned int acc = 0;
    for (int i = 0; i < 512; i++) {
        unsigned int a = xorshift(&s);
        unsigned int b = (xorshift(&s) & 0xFFFFu) + 1u; /* never zero */
        acc += a / b + a % b;
        int sa = (int)a >> 3;
        int sb = (int)(b | 1u);
        acc ^= (unsigned int)(sa / sb) + (unsigned int)(sa % sb);
        acc = (acc << 7) | (acc >> 25);
    }
    return acc;
}

static unsigned long long wide(unsigned int seed)
{
    unsigned long long acc = 0x0123456789ABCDEFull;
    unsigned int s = seed;
    for (int i = 0; i < 256; i++) {
        unsigned int a = xorshift(&s);
        acc += (unsigned long long)a * 0x9E3779B9u;
        acc ^= acc >> 29;
        acc *= 0xBF58476D1CE4E5B9ull;
    }
    return acc;
}

static int classify(unsigned int v)
{
    switch (v & 15u) { /* dense switch -> jump table + bctr */
    case 0: return 3;
    case 1: return 141;
    case 2: return 59;
    case 3: return 26;
    case 4: return 53;
    case 5: return 58;
    case 6: return 97;
    case 7: return 93;
    case 8: return 23;
    case 9: return 84;
    case 10: return 62;
    case 11: return 64;
    case 12: return 33;
    case 13: return 83;
    case 14: return 27;
    default: return 95;
    }
}

static int fib(int n)
{
    return n < 2 ? n : fib(n - 1) + fib(n - 2);
}

void cmain(void)
{
    static unsigned char buf[512];
    static unsigned int arr[257];
    unsigned int s = 0xC0FFEE01u;

    for (unsigned i = 0; i < sizeof buf; i++)
        buf[i] = (unsigned char)(xorshift(&s) >> 13);
    puthex(crc32(buf, sizeof buf));

    puthex((unsigned)sieve(8192));

    for (int i = 0; i < 257; i++)
        arr[i] = xorshift(&s);
    qsort_u32(arr, 0, 256);
    unsigned int h = 0;
    for (int i = 0; i < 257; i++)
        h = h * 31u + arr[i];
    puthex(h);

    puthex(mixdiv(0xDEAD4EEDu));

    unsigned long long w = wide(0x5EED5EEDu);
    puthex((unsigned)(w >> 32));
    puthex((unsigned)w);

    int cls = 0;
    for (int i = 0; i < 64; i++)
        cls += classify(xorshift(&s)) * (i + 1);
    puthex((unsigned)cls);

    puthex((unsigned)fib(20));

    done(0);
}
