/* Oracle corpus: FP #2 — kernels with heavier data flow: 4x4 matrix chains,
 * fixed-iteration escape counts, mixed int/float interleave. Arithmetic only;
 * -ffp-contract=off on both legs. */
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

static double m[4][4], r[4][4];

static void matmul(void)
{
    double t[4][4];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            double s = 0.0;
            for (int k = 0; k < 4; k++)
                s += r[i][k] * m[k][j];
            t[i][j] = s;
        }
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            r[i][j] = t[i][j];
}

void cmain(void)
{
    /* seed a rotation-ish matrix deterministically */
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            m[i][j] = (i == j) ? 0.75 : 0.125 * (i - j);
            r[i][j] = (i == j) ? 1.0 : 0.0;
        }
    for (int n = 0; n < 12; n++)
        matmul();
    for (int i = 0; i < 4; i++)
        putbits(r[i][(i + 1) & 3]);

    /* escape-time counts: pure compare/multiply/add, integer results */
    unsigned acc = 0;
    for (int gy = -2; gy <= 2; gy++) {
        for (int gx = -2; gx <= 2; gx++) {
            double cx = gx * 0.31, cy = gy * 0.27;
            double zx = 0.0, zy = 0.0;
            int it = 0;
            while (it < 30 && zx * zx + zy * zy < 4.0) {
                double nx = zx * zx - zy * zy + cx;
                zy = 2.0 * zx * zy + cy;
                zx = nx;
                it++;
            }
            acc = acc * 31 + (unsigned)it;
        }
    }
    puthex(acc);

    /* float accumulators interleaved with integer state */
    float fs = 0.0f;
    unsigned lcg = 12345u;
    for (int i = 0; i < 200; i++) {
        lcg = lcg * 1103515245u + 12345u;
        int step = (int)(lcg >> 27) - 16;
        fs += (float)step / 32.0f;
        if ((i & 63) == 63) {
            union {
                float f;
                unsigned u;
            } v;
            v.f = fs;
            puthex(v.u);
        }
    }

    /* division stress: quotient chains at varied magnitudes */
    double q = 1.0e9;
    for (int i = 1; i <= 40; i++)
        q = q / (1.0 + i * 0.125) + 3.0 / i;
    putbits(q);

    done(0);
}
