/* Oracle corpus: string/memory ops and bit tricks (integer only). */
#include "shim.h"

static void* mymemcpy(void* d, const void* s, unsigned n)
{
    unsigned char* dp = (unsigned char*)d;
    const unsigned char* sp = (const unsigned char*)s;
    while (n--)
        *dp++ = *sp++;
    return d;
}

static void mymemset(void* d, int v, unsigned n)
{
    unsigned char* dp = (unsigned char*)d;
    while (n--)
        *dp++ = (unsigned char)v;
}

static unsigned mystrlen(const char* s)
{
    const char* p = s;
    while (*p)
        p++;
    return (unsigned)(p - s);
}

static int mystrcmp(const char* a, const char* b)
{
    while (*a && *a == *b) {
        a++;
        b++;
    }
    return (unsigned char)*a - (unsigned char)*b;
}

static unsigned popcount(unsigned v)
{
    v = v - ((v >> 1) & 0x55555555u);
    v = (v & 0x33333333u) + ((v >> 2) & 0x33333333u);
    return (((v + (v >> 4)) & 0x0F0F0F0Fu) * 0x01010101u) >> 24;
}

static unsigned fnv1a(const void* data, unsigned n)
{
    const unsigned char* p = (const unsigned char*)data;
    unsigned h = 2166136261u;
    while (n--) {
        h ^= *p++;
        h *= 16777619u;
    }
    return h;
}

static int bsearch_u32(const unsigned* a, int n, unsigned key)
{
    int lo = 0, hi = n - 1;
    while (lo <= hi) {
        int mid = (lo + hi) >> 1;
        if (a[mid] == key)
            return mid;
        if (a[mid] < key)
            lo = mid + 1;
        else
            hi = mid - 1;
    }
    return -1;
}

void cmain(void)
{
    static char text[300];
    static unsigned nums[128];
    static unsigned char scratch[600];

    const char* words[4] = {"macintosh", "powerpc", "altivec", "sawtooth"};
    unsigned pos = 0;
    for (int i = 0; i < 24; i++) {
        const char* w = words[(i * 7 + 3) & 3];
        unsigned len = mystrlen(w);
        mymemcpy(text + pos, w, len);
        pos += len;
        text[pos++] = '-';
    }
    text[pos] = 0;
    puthex(fnv1a(text, pos));
    puthex(mystrlen(text));
    puthex((unsigned)mystrcmp(text, text + 10));

    mymemset(scratch, 0xA5, sizeof scratch);
    scratch[17] = 3;
    scratch[599] = 7;
    puthex(fnv1a(scratch, sizeof scratch));

    unsigned pc = 0;
    for (unsigned v = 1; v < 4096; v += 7)
        pc += popcount(v * 2654435761u);
    puthex(pc);

    unsigned seed = 0x1234ABCDu;
    for (int i = 0; i < 128; i++) {
        seed = seed * 1103515245u + 12345u;
        nums[i] = (unsigned)i * 65537u + (seed >> 20);
    }
    int found = 0;
    for (int i = 0; i < 128; i += 3)
        if (bsearch_u32(nums, 128, nums[i]) == i)
            found++;
    puthex((unsigned)found);

    done(0);
}
