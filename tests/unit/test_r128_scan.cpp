// The scan-out truth table: CRTC pixel formats 3 (15 bpp ARGB1555),
// 4 (16 bpp RGB565) and 5 (24 bpp RGB888), plus the two that already
// worked (8 bpp CLUT, 32 bpp xRGB).
//
// This deserves pinning by synthetic framebuffer rather than by boot,
// because the failure it guards against was INVISIBLE from a boot: both
// harnesses refused every format except 8 and 32 bpp, and a game that
// switched the display to 15/16 bpp (DrawSprocket's standard move) played
// to a black app window while the machine underneath was alive and
// beeping. The conversion also has a paid-for byte-order lesson in it —
// the Mac writes big-endian pixels, and reading the 32 bpp lanes as
// little-endian rendered a gray desktop olive. These cases assert the
// exact BGRA bytes so neither lesson can regress silently.
//
// Every pixel reaches the screen through the DAC's table and then the
// monitor (see "The DAC's table" in opm/r128.hpp), so the format cases load
// the table that changes nothing and expect the monitor's level for each
// channel; the cases after them are about the table itself.

#include "doctest.h"
#include "opm/r128.hpp"

#include <cmath>
#include <cstdlib>

using namespace opm;

namespace {

// An 8×1 display: H_TOTAL_DISP display field 0 → (0+1)*8 px, V field 0 →
// 1 line, pitch8 = 1 → rowBytes = 8 * bypp, top-left at VRAM 0. setReg is
// the harness poke (native form), the same store peek() answers from.
void crtc8x1(R128Cell& c, u32 fmt)
{
    c.setReg(0x0050, 0x02000000u | (fmt << 8)); // CRTC_GEN_CNTL: enable + fmt
    c.setReg(0x0200, 0);                        // CRTC_H_TOTAL_DISP
    c.setReg(0x0208, 0);                        // CRTC_V_TOTAL_DISP
    c.setReg(0x022C, 1);                        // CRTC_PITCH (8-px units)
    c.setReg(0x0224, 0);                        // CRTC_OFFSET
}

// One converted pixel out of an 8×1 row as (b,g,r,a).
struct Px {
    u8 b, g, r, a;
};
Px px(const u8* row, u32 x)
{
    return {row[x * 4 + 0], row[x * 4 + 1], row[x * 4 + 2], row[x * 4 + 3]};
}

// Load the DAC's table the way a guest does: PALETTE_INDEX, then 256
// auto-incrementing stores to PALETTE_DATA. The register file is
// little-endian on the bus, so a store arrives byte-reversed. `entry(i)` is
// 0x00RRGGBB.
template <typename F> void loadTable(R128Cell& c, F entry)
{
    auto be = [](u32 v) {
        return (v >> 24) | ((v >> 8) & 0xFF00u) | ((v << 8) & 0xFF0000u) |
               (v << 24);
    };
    c.write(0x00B0, be(0u), 4);
    for (u32 i = 0; i < 256; ++i)
        c.write(0x00B4, be(entry(i) & 0x00FFFFFFu), 4);
}
// The table that changes nothing: entry i holds i in all three channels.
void identity(R128Cell& c)
{
    loadTable(c, [](u32 i) { return i * 0x010101u; });
}
// What the screen shows for a level leaving the DAC.
u8 mon(u32 level) { return r128MonitorCurve()[level & 0xFFu]; }

// The first `n` bytes of video memory scanned out as one 8x1 row.
struct Row {
    u8 bgra[8 * 4] = {};
};
Row scan(R128Cell& c, const u8* fb, u32 n)
{
    for (u32 i = 0; i < n; ++i)
        c.vram[i] = fb[i];
    Row r;
    const R128Scan s = r128ScanDecode(c);
    r128ScanRow(c, s, 0, r.bgra);
    return r;
}

} // namespace

TEST_CASE("r128 scan: decode maps formats to bytes-per-pixel and geometry")
{
    R128Cell c;
    crtc8x1(c, 3);
    R128Scan s = r128ScanDecode(c);
    CHECK(s.enabled);
    CHECK(s.fmt == 3u);
    CHECK(s.w == 8u);
    CHECK(s.h == 1u);
    CHECK(s.bypp == 2u);
    CHECK(s.rowBytes == 16u);

    crtc8x1(c, 4);
    CHECK(r128ScanDecode(c).bypp == 2u);
    crtc8x1(c, 5);
    CHECK(r128ScanDecode(c).bypp == 3u);
    CHECK(r128ScanDecode(c).rowBytes == 24u);
    crtc8x1(c, 2);
    CHECK(r128ScanDecode(c).bypp == 1u);
    crtc8x1(c, 6);
    CHECK(r128ScanDecode(c).bypp == 4u);

    // A format the table cannot scan stays a visible refusal: bypp 0.
    crtc8x1(c, 1);
    CHECK(r128ScanDecode(c).bypp == 0u);

    // CRTC disabled: geometry still decodes, enabled says no.
    c.setReg(0x0050, 3u << 8);
    CHECK_FALSE(r128ScanDecode(c).enabled);
}

TEST_CASE("r128 scan: the monitor curve is a 2.6 tube shown in sRGB")
{
    // The numbers in the source against their formula, to a level either
    // way (a host's pow may round the last place differently, which is why
    // the numbers are kept and not computed).
    const u8* m = r128MonitorCurve();
    CHECK(m[0] == 0);
    CHECK(m[255] == 255);
    for (u32 t = 0; t < 256; ++t) {
        const double light = std::pow(t / 255.0, 2.6);
        const double enc = light <= 0.0031308
                               ? 12.92 * light
                               : 1.055 * std::pow(light, 1.0 / 2.4) - 0.055;
        CAPTURE(t);
        CHECK(std::abs(static_cast<int>(m[t]) -
                       static_cast<int>(std::lround(255.0 * enc))) <= 1);
        if (t)
            CHECK(m[t] >= m[t - 1]);
    }
    // The look that was chosen: Mac OS's ramp holds 158 at 128, and a 50%
    // gray is shown as 146.
    CHECK(m[158] == 146);
}

TEST_CASE("r128 scan: fmt 3 (ARGB1555) big-endian halfwords to BGRA")
{
    R128Cell c;
    crtc8x1(c, 3);
    identity(c);
    // Big-endian halfwords in VRAM: pure red 0x7C00, green 0x03E0, blue
    // 0x001F, mid gray 0x4210 (all three 5-bit fields 0x10).
    const u8 fb[8] = {0x7C, 0x00, 0x03, 0xE0, 0x00, 0x1F, 0x42, 0x10};
    const Row r = scan(c, fb, 8);
    // A 5-bit channel is the table index with zeros below: full scale is
    // entry 0xF8, and 0x10 is entry 0x80.
    Px p = px(r.bgra, 0);
    CHECK(p.r == mon(0xF8)); CHECK(p.g == 0x00); CHECK(p.b == 0x00); CHECK(p.a == 0xFF);
    p = px(r.bgra, 1);
    CHECK(p.r == 0x00); CHECK(p.g == mon(0xF8)); CHECK(p.b == 0x00);
    p = px(r.bgra, 2);
    CHECK(p.r == 0x00); CHECK(p.g == 0x00); CHECK(p.b == mon(0xF8));
    p = px(r.bgra, 3);
    CHECK(p.r == mon(0x80)); CHECK(p.g == mon(0x80)); CHECK(p.b == mon(0x80));
}

TEST_CASE("r128 scan: fmt 4 (RGB565) big-endian halfwords to BGRA")
{
    R128Cell c;
    crtc8x1(c, 4);
    identity(c);
    // Red 0xF800, green 0x07E0 (the 6-bit field — a 555 misdecode would
    // put its top bit in red), blue 0x001F, mixed 0x8410.
    const u8 fb[8] = {0xF8, 0x00, 0x07, 0xE0, 0x00, 0x1F, 0x84, 0x10};
    const Row r = scan(c, fb, 8);
    Px p = px(r.bgra, 0);
    CHECK(p.r == mon(0xF8)); CHECK(p.g == 0x00); CHECK(p.b == 0x00);
    p = px(r.bgra, 1);
    CHECK(p.r == 0x00); CHECK(p.g == mon(0xFC)); CHECK(p.b == 0x00);
    p = px(r.bgra, 2);
    CHECK(p.r == 0x00); CHECK(p.g == 0x00); CHECK(p.b == mon(0xF8));
    // 0x8410: r5=0x10 is entry 0x80, g6=0x20 is entry 0x80, b5=0x10 too.
    p = px(r.bgra, 3);
    CHECK(p.r == mon(0x80)); CHECK(p.g == mon(0x80)); CHECK(p.b == mon(0x80));
}

TEST_CASE("r128 scan: fmt 5 (RGB888) is R,G,B — the 32 bpp order minus x")
{
    R128Cell c;
    crtc8x1(c, 5);
    identity(c);
    const u8 fb[6] = {0x11, 0x22, 0x33, 0xAA, 0xBB, 0xCC};
    const Row r = scan(c, fb, 6);
    Px p = px(r.bgra, 0);
    CHECK(p.r == mon(0x11)); CHECK(p.g == mon(0x22)); CHECK(p.b == mon(0x33));
    p = px(r.bgra, 1);
    CHECK(p.r == mon(0xAA)); CHECK(p.g == mon(0xBB)); CHECK(p.b == mon(0xCC));
}

TEST_CASE("r128 scan: fmt 6 keeps the olive lesson — byte 0 is the x lane")
{
    R128Cell c;
    crtc8x1(c, 6);
    identity(c);
    // The 50% gray that once rendered olive: 00 80 80 80 big-endian xRGB.
    // The second pixel's unused lane is full and must not reach a channel.
    const u8 fb[8] = {0x00, 0x80, 0x80, 0x80, 0xFF, 0x12, 0x34, 0x56};
    const Row r = scan(c, fb, 8);
    Px p = px(r.bgra, 0);
    CHECK(p.r == mon(0x80)); CHECK(p.g == mon(0x80)); CHECK(p.b == mon(0x80));
    p = px(r.bgra, 1);
    CHECK(p.r == mon(0x12)); CHECK(p.g == mon(0x34)); CHECK(p.b == mon(0x56));
}

TEST_CASE("r128 scan: fmt 2 reads the DAC palette")
{
    R128Cell c;
    crtc8x1(c, 2);
    // Entry 7 = (R=0x10, G=0x20, B=0x30), every other entry white: the
    // pixel is the index, and the entry's three channels go their own way.
    loadTable(c, [](u32 i) { return i == 7u ? 0x00102030u : 0x00FFFFFFu; });
    const u8 fb[2] = {7, 8};
    const Row r = scan(c, fb, 2);
    Px p = px(r.bgra, 0);
    CHECK(p.r == mon(0x10)); CHECK(p.g == mon(0x20)); CHECK(p.b == mon(0x30));
    CHECK(p.a == 0xFF);
    p = px(r.bgra, 1);
    CHECK(p.r == 0xFF); CHECK(p.g == 0xFF); CHECK(p.b == 0xFF);
}

TEST_CASE("r128 scan: a narrow channel indexes the table with zeros below it")
{
    // The entries a 5-bit and a 6-bit full-scale channel reach are 0xF8 and
    // 0xFC. A table that holds something else just above them tells the
    // chip's indexing from one that fills the low bits with the high ones.
    R128Cell c;
    crtc8x1(c, 4);
    loadTable(c, [](u32 i) {
        return i == 0xF8u ? 0x00C80000u   // red, as a 5-bit channel finds it
               : i == 0xFCu ? 0x00006400u // green, as the 6-bit one does
               : i == 0xFFu ? 0x000A0A0Au // where replicated bits would land
                            : 0u;
    });
    const u8 fb[4] = {0xF8, 0x00, 0x07, 0xE0}; // full red, full green
    const Row r = scan(c, fb, 4);
    CHECK(px(r.bgra, 0).r == mon(0xC8));
    CHECK(px(r.bgra, 1).g == mon(0x64));
}

TEST_CASE("r128 scan: the table is applied in the direct-color modes, each "
          "channel through its own third")
{
    // Before, a 15/16/32 bpp pixel went to the screen as stored, and a
    // game's fade (the ramp loaded again and again, a step darker each
    // time) was invisible. A ramp at half height must halve what leaves the
    // DAC; a table with the channels crossed must cross them.
    R128Cell c;
    crtc8x1(c, 6);
    const u8 fb[8] = {0x00, 0xC8, 0x64, 0x32, 0x00, 0xFF, 0xFF, 0xFF};
    loadTable(c, [](u32 i) { return (i / 2u) * 0x010101u; });
    Row r = scan(c, fb, 8);
    Px p = px(r.bgra, 0);
    CHECK(p.r == mon(0x64)); CHECK(p.g == mon(0x32)); CHECK(p.b == mon(0x19));
    p = px(r.bgra, 1);
    CHECK(p.r == mon(0x7F)); CHECK(p.g == mon(0x7F)); CHECK(p.b == mon(0x7F));
    // All the way down: black, whatever the pixel.
    loadTable(c, [](u32) { return 0u; });
    r = scan(c, fb, 8);
    p = px(r.bgra, 1);
    CHECK(p.r == 0); CHECK(p.g == 0); CHECK(p.b == 0);
    // Red's third holds the index, green's its complement, blue's nothing.
    loadTable(c, [](u32 i) { return (i << 16) | ((255u - i) << 8); });
    r = scan(c, fb, 8);
    p = px(r.bgra, 0);
    CHECK(p.r == mon(0xC8)); CHECK(p.g == mon(255u - 0x64u)); CHECK(p.b == 0);
}

TEST_CASE("r128 scan: Mac OS's ramp shows a 50% gray as 146")
{
    // The ramp Mac OS loads, as read from a running system (every eighth
    // entry; it is close to a 1/1.44 power). The entry a 0x80 channel
    // selects holds 0x9E.
    R128Cell c;
    crtc8x1(c, 6);
    loadTable(c, [](u32 i) { return i == 0x80u ? 0x009E9E9Eu : 0u; });
    const u8 fb[4] = {0x00, 0x80, 0x80, 0x80};
    const Row r = scan(c, fb, 4);
    const Px p = px(r.bgra, 0);
    CHECK(p.r == 146); CHECK(p.g == 146); CHECK(p.b == 146);
}

TEST_CASE("r128 scan: hardware cursor composites over the new formats")
{
    R128Cell c;
    crtc8x1(c, 4);
    identity(c);
    // A visible 16-bpp row of mid gray.
    for (u32 i = 0; i < 16; i += 2) {
        c.vram[i] = 0x84;
        c.vram[i + 1] = 0x10;
    }
    // Cursor enabled, bitmap at 0x1000: row 0 has AND=0 for every column,
    // XOR bit set for column 0 only → color 1 at the hotspot column,
    // color 0 for the rest. Position the cursor at x=2.
    c.setReg(0x0050, c.peek(0x0050) | 0x00010000u);
    c.setReg(0x0260, 0x1000);
    c.setReg(0x0264, 2u << 16); // CUR_HORZ_VERT_POSN: x=2, y=0
    c.setReg(0x0268, 0);
    c.setReg(0x026C, 0x00112233u); // color 0: R=11 G=22 B=33
    c.setReg(0x0270, 0x00FF0000u); // color 1: pure red
    for (u32 k = 0; k < 8; ++k)
        c.vram[0x1000 + k] = 0x00; // AND: all select colors
    c.vram[0x1008] = 0x80;         // XOR: col 0 → color 1
    for (u32 k = 1; k < 8; ++k)
        c.vram[0x1008 + k] = 0x00;
    auto shown = [&c] {
        Row r;
        const R128Scan s = r128ScanDecode(c);
        r128ScanRow(c, s, 0, r.bgra);
        r128ScanCursor(c, s, r.bgra);
        return r;
    };
    Row r = shown();
    // x=0,1: untouched framebuffer gray.
    Px p = px(r.bgra, 0);
    CHECK(p.r == mon(0x80)); CHECK(p.g == mon(0x80)); CHECK(p.b == mon(0x80));
    // x=2: cursor color 1 (stored 00RRGGBB).
    p = px(r.bgra, 2);
    CHECK(p.r == 0xFF); CHECK(p.g == 0x00); CHECK(p.b == 0x00);
    // x=3: cursor color 0.
    p = px(r.bgra, 3);
    CHECK(p.r == mon(0x11)); CHECK(p.g == mon(0x22)); CHECK(p.b == mon(0x33));
    // The cursor's colors do not pass the table ("always 24 bpp,
    // independent of graphics mode"): with the picture faded to black the
    // pointer is still there, at its own colors.
    loadTable(c, [](u32) { return 0u; });
    r = shown();
    CHECK(px(r.bgra, 0).r == 0);
    p = px(r.bgra, 2);
    CHECK(p.r == 0xFF); CHECK(p.g == 0x00); CHECK(p.b == 0x00);
    p = px(r.bgra, 3);
    CHECK(p.r == mon(0x11)); CHECK(p.g == mon(0x22)); CHECK(p.b == mon(0x33));
}
