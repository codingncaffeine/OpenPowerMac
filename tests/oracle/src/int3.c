/* Oracle corpus: heapsort, histograms, rotate/carry chains (integer only). */
#include "shim.h"

static unsigned rotl32(unsigned v, unsigned n) { return (v << n) | (v >> (32 - n)); }

static void sift(unsigned* a, int start, int end)
{
    int root = start;
    while (root * 2 + 1 <= end) {
        int child = root * 2 + 1;
        if (child + 1 <= end && a[child] < a[child + 1])
            child++;
        if (a[root] < a[child]) {
            unsigned t = a[root];
            a[root] = a[child];
            a[child] = t;
            root = child;
        } else {
            return;
        }
    }
}

static void heapsort_u32(unsigned* a, int n)
{
    for (int start = (n - 2) / 2; start >= 0; start--)
        sift(a, start, n - 1);
    for (int end = n - 1; end > 0; end--) {
        unsigned t = a[end];
        a[end] = a[0];
        a[0] = t;
        sift(a, 0, end - 1);
    }
}

void cmain(void)
{
    static unsigned data[500];
    static unsigned hist[16];

    unsigned s = 0xFACEB00Cu;
    for (int i = 0; i < 500; i++) {
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        data[i] = s;
        hist[s & 15u]++;
    }

    unsigned hh = 0;
    for (int i = 0; i < 16; i++)
        hh = rotl32(hh, 5) + hist[i];
    puthex(hh);

    heapsort_u32(data, 500);
    unsigned ordered = 1;
    for (int i = 1; i < 500; i++)
        if (data[i - 1] > data[i])
            ordered = 0;
    puthex(ordered);
    puthex(data[0]);
    puthex(data[250]);
    puthex(data[499]);

    /* carry-chain torture: 128-bit accumulate built from 32-bit adds */
    unsigned acc0 = 0, acc1 = 0, acc2 = 0, acc3 = 0;
    for (int i = 0; i < 500; i++) {
        unsigned lo = data[i] * 0x9E3779B9u;
        unsigned old = acc0;
        acc0 += lo;
        unsigned c0 = acc0 < old;
        old = acc1;
        acc1 += (data[i] >> 3) + c0;
        unsigned c1 = acc1 < old || (c0 && acc1 == old);
        old = acc2;
        acc2 += c1 + (lo >> 29);
        acc3 += rotl32(acc2 ^ lo, (data[i] & 31u) ? (data[i] & 31u) : 1u);
    }
    puthex(acc0);
    puthex(acc1);
    puthex(acc2);
    puthex(acc3);

    done(0);
}
