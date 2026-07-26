/* Oracle corpus: FP arithmetic only (+,-,*,/ and conversions) — results are
 * printed as bit patterns and must match the host build exactly. No libm, no
 * generated-NaN cases (host default QNaN differs), no fma contraction
 * (-ffp-contract=off on both legs). */
#include "shim.h"

static void putbits(double d)
{
    union {
        double f;
        unsigned long long u;
    } v;
    v.f = d;
    puthex((unsigned int)(v.u >> 32));
    puthex((unsigned int)v.u);
}

static void putbitsf(float f)
{
    union {
        float f;
        unsigned int u;
    } v;
    v.f = f;
    puthex(v.u);
}

void cmain(void)
{
    /* harmonic and alternating sums: fdiv/fadd/fsub chains */
    double h = 0.0, alt = 0.0;
    for (int k = 1; k <= 50; k++) {
        h += 1.0 / k;
        alt += (k & 1) ? 1.0 / k : -1.0 / k;
    }
    putbits(h);
    putbits(alt);

    /* product ladder */
    double p = 1.0;
    for (int k = 1; k <= 40; k++)
        p *= 1.0 + 1.0 / (k * k);
    putbits(p);

    /* Newton reciprocal of 7 without division in the loop */
    double x = 0.1;
    for (int i = 0; i < 6; i++)
        x = x * (2.0 - 7.0 * x);
    putbits(x);

    /* Horner polynomial at a few points */
    for (int i = -3; i <= 3; i++) {
        double t = i * 0.37;
        putbits(((2.5 * t - 1.25) * t + 0.375) * t - 7.0);
    }

    /* underflow ladder: normals -> denorms -> zero */
    double u = 1.0;
    for (int i = 0; i < 1120; i++) {
        u *= 0.5;
        if (i % 100 == 99)
            putbits(u);
    }

    /* overflow ladder: -> inf, and inf arithmetic */
    double o = 1.0;
    for (int i = 0; i < 1100; i++)
        o *= 2.0;
    putbits(o);
    putbits(o + 1.0e300);
    putbits(1.0 / o);

    /* single precision: same shapes through fadds/fmuls/fdivs */
    float hf = 0.0f, pf = 1.0f;
    for (int k = 1; k <= 30; k++) {
        hf += 1.0f / k;
        pf *= 1.0f + 1.0f / (k + 3);
    }
    putbitsf(hf);
    putbitsf(pf);

    float uf = 1.0f;
    for (int i = 0; i < 160; i++)
        uf *= 0.5f;
    putbitsf(uf); /* single denorm territory */

    /* conversions: fctiwz via C casts, frsp via (float), lfs/stfs shapes */
    double c1 = -2.75, c2 = 2.5, c3 = 1234567.891;
    puthex((unsigned int)(int)(c1 * 3.0));
    puthex((unsigned int)(int)c2);
    puthex((unsigned int)(int)(c3 / 7.0));
    putbitsf((float)h);
    putbitsf((float)(1.0 / 3.0));
    putbits((double)(float)0.1);

    /* comparisons drive control flow */
    unsigned cmp = 0;
    double vals[5] = {-1.5, 0.0, 0.25, 3.0, -0.0};
    for (int i = 0; i < 5; i++)
        for (int j = 0; j < 5; j++) {
            cmp = (cmp << 2) | (vals[i] < vals[j] ? 2u : 0u) |
                  (vals[i] == vals[j] ? 1u : 0u);
            if (((i * 5 + j) % 16) == 15) {
                puthex(cmp);
                cmp = 0;
            }
        }
    puthex(cmp);

    done(0);
}
