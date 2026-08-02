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
// little-endian rendered a grey desktop olive. These cases assert the
// exact BGRA bytes so neither lesson can regress silently.

#include "doctest.h"
#include "opm/r128.hpp"

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

TEST_CASE("r128 scan: fmt 3 (ARGB1555) big-endian halfwords to BGRA")
{
    R128Cell c;
    crtc8x1(c, 3);
    // Big-endian halfwords in VRAM: pure red 0x7C00, green 0x03E0, blue
    // 0x001F, mid grey 0x4210 (all three 5-bit fields 0x10).
    const u8 fb[8] = {0x7C, 0x00, 0x03, 0xE0, 0x00, 0x1F, 0x42, 0x10};
    for (u32 i = 0; i < 8; ++i)
        c.vram[i] = fb[i];
    const R128Scan s = r128ScanDecode(c);
    u8 row[8 * 4] = {};
    r128ScanRow(c, s, 0, row);
    // Full-scale 0x1F must come out 0xFF (bit replication), not 0xF8.
    Px p = px(row, 0);
    CHECK(p.r == 0xFF); CHECK(p.g == 0x00); CHECK(p.b == 0x00); CHECK(p.a == 0xFF);
    p = px(row, 1);
    CHECK(p.r == 0x00); CHECK(p.g == 0xFF); CHECK(p.b == 0x00);
    p = px(row, 2);
    CHECK(p.r == 0x00); CHECK(p.g == 0x00); CHECK(p.b == 0xFF);
    // 0x10 → (0x10<<3) | (0x10>>2) = 0x84 in every lane.
    p = px(row, 3);
    CHECK(p.r == 0x84); CHECK(p.g == 0x84); CHECK(p.b == 0x84);
}

TEST_CASE("r128 scan: fmt 4 (RGB565) big-endian halfwords to BGRA")
{
    R128Cell c;
    crtc8x1(c, 4);
    // Red 0xF800, green 0x07E0 (the 6-bit field — a 555 misdecode would
    // put its top bit in red), blue 0x001F, mixed 0x8410.
    const u8 fb[8] = {0xF8, 0x00, 0x07, 0xE0, 0x00, 0x1F, 0x84, 0x10};
    for (u32 i = 0; i < 8; ++i)
        c.vram[i] = fb[i];
    const R128Scan s = r128ScanDecode(c);
    u8 row[8 * 4] = {};
    r128ScanRow(c, s, 0, row);
    Px p = px(row, 0);
    CHECK(p.r == 0xFF); CHECK(p.g == 0x00); CHECK(p.b == 0x00);
    p = px(row, 1);
    CHECK(p.r == 0x00); CHECK(p.g == 0xFF); CHECK(p.b == 0x00);
    p = px(row, 2);
    CHECK(p.r == 0x00); CHECK(p.g == 0x00); CHECK(p.b == 0xFF);
    // 0x8410: r5=0x10→0x84, g6=0x20→0x82, b5=0x10→0x84.
    p = px(row, 3);
    CHECK(p.r == 0x84); CHECK(p.g == 0x82); CHECK(p.b == 0x84);
}

TEST_CASE("r128 scan: fmt 5 (RGB888) is R,G,B — the 32 bpp order minus x")
{
    R128Cell c;
    crtc8x1(c, 5);
    const u8 fb[6] = {0x11, 0x22, 0x33, 0xAA, 0xBB, 0xCC};
    for (u32 i = 0; i < 6; ++i)
        c.vram[i] = fb[i];
    const R128Scan s = r128ScanDecode(c);
    u8 row[8 * 4] = {};
    r128ScanRow(c, s, 0, row);
    Px p = px(row, 0);
    CHECK(p.r == 0x11); CHECK(p.g == 0x22); CHECK(p.b == 0x33);
    p = px(row, 1);
    CHECK(p.r == 0xAA); CHECK(p.g == 0xBB); CHECK(p.b == 0xCC);
}

TEST_CASE("r128 scan: fmt 6 keeps the olive lesson — byte 0 is the x lane")
{
    R128Cell c;
    crtc8x1(c, 6);
    // The 50% grey that once rendered olive: 00 80 80 80 big-endian xRGB.
    const u8 fb[8] = {0x00, 0x80, 0x80, 0x80, 0x00, 0x12, 0x34, 0x56};
    for (u32 i = 0; i < 8; ++i)
        c.vram[i] = fb[i];
    const R128Scan s = r128ScanDecode(c);
    u8 row[8 * 4] = {};
    r128ScanRow(c, s, 0, row);
    Px p = px(row, 0);
    CHECK(p.r == 0x80); CHECK(p.g == 0x80); CHECK(p.b == 0x80);
    p = px(row, 1);
    CHECK(p.r == 0x12); CHECK(p.g == 0x34); CHECK(p.b == 0x56);
}

TEST_CASE("r128 scan: fmt 2 reads the DAC palette")
{
    R128Cell c;
    crtc8x1(c, 2);
    // Program palette entry 7 = (R=10,G=20,B=30) through the guest's own
    // path: PALETTE_INDEX, then auto-incrementing PALETTE_DATA. The
    // register file is little-endian on the bus, so the store arrives
    // byte-reversed — same convention as every write in test_r128_2d.
    auto be = [](u32 v) {
        return (v >> 24) | ((v >> 8) & 0xFF00u) | ((v << 8) & 0xFF0000u) |
               (v << 24);
    };
    c.write(0x00B0, be(7u), 4);
    c.write(0x00B4, be(0x00102030u), 4);
    c.vram[0] = 7;
    const R128Scan s = r128ScanDecode(c);
    u8 row[8 * 4] = {};
    r128ScanRow(c, s, 0, row);
    const Px p = px(row, 0);
    CHECK(p.r == 0x10); CHECK(p.g == 0x20); CHECK(p.b == 0x30);
    CHECK(p.a == 0xFF);
}

TEST_CASE("r128 scan: hardware cursor composites over the new formats")
{
    R128Cell c;
    crtc8x1(c, 4);
    // A visible 16-bpp row of mid grey.
    for (u32 i = 0; i < 16; i += 2) {
        c.vram[i] = 0x84;
        c.vram[i + 1] = 0x10;
    }
    // Cursor enabled, bitmap at 0x1000: row 0 has AND=0 for every column,
    // XOR bit set for column 0 only → colour 1 at the hotspot column,
    // colour 0 for the rest. Position the cursor at x=2.
    c.setReg(0x0050, c.peek(0x0050) | 0x00010000u);
    c.setReg(0x0260, 0x1000);
    c.setReg(0x0264, 2u << 16); // CUR_HORZ_VERT_POSN: x=2, y=0
    c.setReg(0x0268, 0);
    c.setReg(0x026C, 0x00112233u); // colour 0: R=11 G=22 B=33
    c.setReg(0x0270, 0x00FF0000u); // colour 1: pure red
    for (u32 k = 0; k < 8; ++k)
        c.vram[0x1000 + k] = 0x00; // AND: all select colours
    c.vram[0x1008] = 0x80;         // XOR: col 0 → colour 1
    for (u32 k = 1; k < 8; ++k)
        c.vram[0x1008 + k] = 0x00;
    const R128Scan s = r128ScanDecode(c);
    u8 row[8 * 4] = {};
    r128ScanRow(c, s, 0, row);
    r128ScanCursor(c, s, row);
    // x=0,1: untouched framebuffer grey.
    Px p = px(row, 0);
    CHECK(p.r == 0x84); CHECK(p.g == 0x82); CHECK(p.b == 0x84);
    // x=2: cursor colour 1 (stored 00RRGGBB).
    p = px(row, 2);
    CHECK(p.r == 0xFF); CHECK(p.g == 0x00); CHECK(p.b == 0x00);
    // x=3: cursor colour 0.
    p = px(row, 3);
    CHECK(p.r == 0x11); CHECK(p.g == 0x22); CHECK(p.b == 0x33);
}
