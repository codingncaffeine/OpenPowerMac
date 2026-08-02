// The 3D pipeline: packets in, shaded Z-tested pixels out.
//
// Every case here is a discriminating one in the stage-1 house style: the
// quad-adjacency case fails if the fill rule double-draws or gaps a shared
// edge, the perspective case fails if texturing is affine, the Z case fails
// if the compare or the write is wrong, the walker case fails if the GART
// path or the index decode is wrong, and the ring case fails if the fetch,
// the wrap or the read-pointer write-back is wrong. Formats and register
// fields are SDK-G04000 App F / ch 6 with bit positions from the Linux DRM
// header — the same shelf r128_3d.cpp is built from.

#include "doctest.h"
#include "opm/r128.hpp"

#include <cstring>
#include <vector>

using namespace opm;

namespace {

u32 be(u32 v)
{
    return (v >> 24) | ((v >> 8) & 0xFF00u) | ((v << 8) & 0xFF0000u) |
           (v << 24);
}

u32 f2u(float f)
{
    u32 u;
    std::memcpy(&u, &f, 4);
    return u;
}

// A rig with 8 MB of "system RAM", a GART table at 1 MB, and helpers that
// program the engine through the same paths the guest uses: MMIO stores for
// registers, FIFO words for packets.
struct Rig3d {
    R128Cell c;
    std::vector<u8> ram;
    static constexpr u32 kGart = 0x00100000u;

    Rig3d() : ram(8u << 20, 0) { r128CceReset(); }

    void put32(u32 at, u32 v) // little-endian, as the guest builds tables
    {
        ram[at] = static_cast<u8>(v);
        ram[at + 1] = static_cast<u8>(v >> 8);
        ram[at + 2] = static_cast<u8>(v >> 16);
        ram[at + 3] = static_cast<u8>(v >> 24);
    }
    void mapPage(u32 agpOff, u32 pa)
    {
        put32(kGart + (agpOff >> 12) * 4u, (pa & 0xFFFFF000u) | 1u);
    }
    void fifo(u32 w)
    {
        r128CceFifoWord(c, be(w), 4, ram.data(),
                        static_cast<u32>(ram.size()), kGart, 0, nullptr);
    }
    void reg(u32 off, u32 v) { c.write(off, be(v), 4); }

    // 64-px-wide 32bpp destination at VRAM 0, scissor wide open over 64x64.
    void dst32()
    {
        reg(0x146Cu, (1u << 30) | (1u << 28) | 0x0Fu | (6u << 8));
        reg(0x142Cu, (0u >> 5) | ((64u / 8u) << 21));
        reg(0x16ECu, 0);
        reg(0x16F0u, (63u << 16) | 63u);
    }
    void dst16()
    {
        reg(0x146Cu, (1u << 30) | (1u << 28) | 0x0Fu | (4u << 8));
        reg(0x142Cu, (0u >> 5) | ((64u / 8u) << 21));
        reg(0x16ECu, 0);
        reg(0x16F0u, (63u << 16) | 63u);
    }
    void openGate() { reg(0x1CA0u, 2u << 8); }
    void fpuSolidBoth() { reg(0x071Cu, (3u << 1) | (3u << 3) | (2u << 5)); }

    u32 pix32(u32 x, u32 y) const
    {
        const size_t at = (static_cast<size_t>(y) * 64u + x) * 4u;
        return (static_cast<u32>(c.vram[at]) << 24) |
               (static_cast<u32>(c.vram[at + 1]) << 16) |
               (static_cast<u32>(c.vram[at + 2]) << 8) | c.vram[at + 3];
    }
    u32 pix16(u32 x, u32 y) const
    {
        const size_t at = (static_cast<size_t>(y) * 64u + x) * 2u;
        return (static_cast<u32>(c.vram[at]) << 8) | c.vram[at + 1];
    }

    // Submit a 3D_RNDR_GEN_PRIM with inline vertices (ring walk).
    void prim(u32 fmt, u32 primType, u32 nVerts,
              const std::vector<u32>& vtxWords)
    {
        const u32 bodyN = 2u + static_cast<u32>(vtxWords.size());
        fifo(0xC0002500u | ((bodyN - 1u) << 16));
        fifo(fmt);
        fifo(primType | (3u << 4) | (nVerts << 16));
        for (u32 w : vtxWords)
            fifo(w);
    }
    // One RHW|DIFFUSE_ARGB vertex (fmt 0x9, five DWORDs).
    static void vtx(std::vector<u32>& out, float x, float y, float z,
                    u32 argb, float rhw = 1.0f)
    {
        out.push_back(f2u(x));
        out.push_back(f2u(y));
        out.push_back(f2u(z));
        out.push_back(f2u(rhw));
        out.push_back(argb);
    }
};

} // namespace

TEST_CASE("3d: two triangles sharing an edge cover a quad exactly once")
{
    Rig3d r;
    r.dst32();
    r.openGate();
    r.fpuSolidBoth();
    const auto before = r128EngStats();
    std::vector<u32> v;
    // Both wound clockwise on screen; the shared diagonal must be drawn by
    // exactly one of them — the top-left rule's whole job.
    Rig3d::vtx(v, 0, 0, 0.5f, 0xFF0000FFu);
    Rig3d::vtx(v, 32, 0, 0.5f, 0xFF0000FFu);
    Rig3d::vtx(v, 0, 32, 0.5f, 0xFF0000FFu);
    Rig3d::vtx(v, 32, 0, 0.5f, 0xFF0000FFu);
    Rig3d::vtx(v, 32, 32, 0.5f, 0xFF0000FFu);
    Rig3d::vtx(v, 0, 32, 0.5f, 0xFF0000FFu);
    r.prim(0x9u, 4u, 6u, v); // triangle list
    const auto& e = r128EngStats();
    CHECK(e.tris - before.tris == 2);
    // Exactly 32*32 pixels written: a double-drawn diagonal would be 1055,
    // a gapped one 993.
    CHECK(e.triPixels - before.triPixels == 1024);
    CHECK(r.pix32(0, 0) == 0xFF0000FFu);
    CHECK(r.pix32(31, 31) == 0xFF0000FFu);
    CHECK(r.pix32(15, 16) == 0xFF0000FFu);
    CHECK(r.pix32(32, 0) == 0u);  // right edge exclusive
    CHECK(r.pix32(0, 32) == 0u);  // bottom edge exclusive
}

TEST_CASE("3d: Gouraud interpolation lands the corner colours and the blend")
{
    Rig3d r;
    r.dst32();
    r.openGate();
    r.fpuSolidBoth();
    std::vector<u32> v;
    Rig3d::vtx(v, 0, 0, 0.5f, 0xFFC80000u);  // red 200
    Rig3d::vtx(v, 32, 0, 0.5f, 0xFF00C800u); // green 200
    Rig3d::vtx(v, 0, 32, 0.5f, 0xFF0000C8u); // blue 200
    r.prim(0x9u, 4u, 3u, v);
    // Pixel (0,0) center (0.5,0.5): weights 62/64, 1/64, 1/64.
    const u32 p = r.pix32(0, 0);
    const int pr = (p >> 16) & 0xFF, pg = (p >> 8) & 0xFF, pb = p & 0xFF;
    CHECK(pr >= 193);
    CHECK(pr <= 195);
    CHECK(pg >= 2);
    CHECK(pg <= 4);
    CHECK(pb >= 2);
    CHECK(pb <= 4);
}

TEST_CASE("3d: Z-less rejects the farther triangle wherever they overlap")
{
    Rig3d r;
    r.dst32();
    r.openGate();
    r.fpuSolidBoth();
    // Z16 buffer at 128 KB, 64-px pitch, initialised to the far plane.
    for (u32 i = 0; i < 64u * 64u * 2u; ++i)
        r.c.vram[0x20000u + i] = 0xFF;
    r.reg(0x1C90u, 0x20000u);
    r.reg(0x1C94u, 64u / 8u);
    r.reg(0x1C98u, (0u << 1) | (1u << 4)); // 16-bit, Z_TEST_LESS
    r.reg(0x1C9Cu, 0x3u);                  // Z_ENABLE | Z_WRITE_ENABLE
    std::vector<u32> nearTri, farTri;
    Rig3d::vtx(nearTri, 0, 0, 0.25f, 0xFF00FF00u);
    Rig3d::vtx(nearTri, 32, 0, 0.25f, 0xFF00FF00u);
    Rig3d::vtx(nearTri, 0, 32, 0.25f, 0xFF00FF00u);
    Rig3d::vtx(farTri, 0, 0, 0.75f, 0xFFFF0000u);
    Rig3d::vtx(farTri, 32, 0, 0.75f, 0xFFFF0000u);
    Rig3d::vtx(farTri, 0, 32, 0.75f, 0xFFFF0000u);
    r.prim(0x9u, 4u, 3u, nearTri);
    r.prim(0x9u, 4u, 3u, farTri); // farther, submitted second: must lose
    CHECK(r.pix32(4, 4) == 0xFF00FF00u);
    // And the Z buffer holds the near value, proving the write happened.
    const u32 zAt = 0x20000u + (4u * 64u + 4u) * 2u;
    const u32 z = (static_cast<u32>(r.c.vram[zAt]) << 8) | r.c.vram[zAt + 1];
    CHECK(z == 16384); // 0.25 * 65535 rounded
}

TEST_CASE("3d: alpha blend SRCALPHA/INVSRCALPHA mixes source over the dest")
{
    Rig3d r;
    r.dst32();
    r.openGate();
    r.fpuSolidBoth();
    std::vector<u32> bg, tri;
    Rig3d::vtx(bg, 0, 0, 0.5f, 0xFF0000FFu); // opaque blue background
    Rig3d::vtx(bg, 32, 0, 0.5f, 0xFF0000FFu);
    Rig3d::vtx(bg, 0, 32, 0.5f, 0xFF0000FFu);
    r.prim(0x9u, 4u, 3u, bg);
    // Now blend half-transparent red over it.
    r.reg(0x1C9Cu, 1u << 9); // ALPHA_ENABLE
    r.reg(0x1CA0u, (2u << 8) | (4u << 16) | (5u << 20)); // SRCA / INVSRCA
    Rig3d::vtx(tri, 0, 0, 0.5f, 0x80FF0000u);
    Rig3d::vtx(tri, 32, 0, 0.5f, 0x80FF0000u);
    Rig3d::vtx(tri, 0, 32, 0.5f, 0x80FF0000u);
    r.prim(0x9u, 4u, 3u, tri);
    const u32 p = r.pix32(4, 4);
    const int pr = (p >> 16) & 0xFF, pb = p & 0xFF;
    CHECK(pr >= 127); // 255*128/255
    CHECK(pr <= 129);
    CHECK(pb >= 126); // 255*127/255
    CHECK(pb <= 128);
}

TEST_CASE("3d: alpha test rejects below the reference")
{
    Rig3d r;
    r.dst32();
    r.openGate();
    r.fpuSolidBoth();
    r.reg(0x1C9Cu, 1u << 10); // ALPHA_TEST_ENABLE
    // Pass only if source alpha > 128.
    r.reg(0x1CA0u, (2u << 8) | (5u << 24) | 128u);
    const auto before = r128EngStats();
    std::vector<u32> v;
    Rig3d::vtx(v, 0, 0, 0.5f, 0x64FF0000u); // alpha 100: rejected
    Rig3d::vtx(v, 32, 0, 0.5f, 0x64FF0000u);
    Rig3d::vtx(v, 0, 32, 0.5f, 0x64FF0000u);
    r.prim(0x9u, 4u, 3u, v);
    CHECK(r128EngStats().triPixels - before.triPixels == 0);
    CHECK(r.pix32(4, 4) == 0u);
}

TEST_CASE("3d: vertex fog carries the fragment to the fog colour")
{
    Rig3d r;
    r.dst32();
    r.openGate();
    r.fpuSolidBoth();
    r.reg(0x1C9Cu, 1u << 7);        // FOG_ENABLE, vertex fog (misc bit14=0)
    r.reg(0x1CACu, 0x0000FF00u);    // fog colour: green
    // Format RHW|DIFFUSE_ARGB|SPEC_FRGB: the FRGB dword's top byte is the
    // fog factor — zero means fully fogged.
    std::vector<u32> v;
    auto vtxFog = [&](float x, float y) {
        v.push_back(f2u(x));
        v.push_back(f2u(y));
        v.push_back(f2u(0.5f));
        v.push_back(f2u(1.0f));
        v.push_back(0xFFFF0000u); // red diffuse
        v.push_back(0x00000000u); // fog 0, no specular
    };
    vtxFog(0, 0);
    vtxFog(32, 0);
    vtxFog(0, 32);
    r.prim(0x9u | 0x40u, 4u, 3u, v);
    const u32 p = r.pix32(4, 4);
    CHECK(((p >> 16) & 0xFFu) == 0);   // red fogged away
    CHECK(((p >> 8) & 0xFFu) == 255);  // to the fog green
}

TEST_CASE("3d: the fog table is written through the index/data pair and used")
{
    Rig3d r;
    r.dst32();
    r.openGate();
    r.fpuSolidBoth();
    r.reg(0x1C9Cu, 1u << 7);
    r.reg(0x1CA0u, (2u << 8) | (1u << 14)); // FOG_TABLE_EN
    r.reg(0x1CACu, 0x000000FFu);            // fog colour: blue
    // Table: everything fully fogged, through the auto-incrementing pair.
    r.reg(0x1A14u, 0);
    for (u32 i = 0; i < 256u; ++i)
        r.reg(0x1A18u, 0);
    std::vector<u32> v;
    Rig3d::vtx(v, 0, 0, 0.5f, 0xFFFF0000u);
    Rig3d::vtx(v, 32, 0, 0.5f, 0xFFFF0000u);
    Rig3d::vtx(v, 0, 32, 0.5f, 0xFFFF0000u);
    r.prim(0x9u, 4u, 3u, v);
    CHECK(r.pix32(4, 4) == 0xFF0000FFu);
}

TEST_CASE("3d: backface culling honours the winding and the front-face rule")
{
    Rig3d r;
    r.dst32();
    r.openGate();
    // FRONT_DIR_CW (bit0=0), BACKFACE_CULL (0<<1), FRONTFACE_SOLID (3<<3).
    r.reg(0x071Cu, (0u << 1) | (3u << 3) | (2u << 5));
    const auto before = r128EngStats();
    std::vector<u32> cw, ccw;
    Rig3d::vtx(cw, 0, 0, 0.5f, 0xFF0000FFu);
    Rig3d::vtx(cw, 32, 0, 0.5f, 0xFF0000FFu);
    Rig3d::vtx(cw, 0, 32, 0.5f, 0xFF0000FFu);
    Rig3d::vtx(ccw, 0, 0, 0.5f, 0xFFFF0000u);
    Rig3d::vtx(ccw, 0, 32, 0.5f, 0xFFFF0000u);
    Rig3d::vtx(ccw, 32, 0, 0.5f, 0xFFFF0000u);
    r.prim(0x9u, 4u, 3u, cw);  // front: drawn
    r.prim(0x9u, 4u, 3u, ccw); // back: culled
    const auto& e = r128EngStats();
    CHECK(e.tris - before.tris == 1);
    CHECK(e.triCulled - before.triCulled == 1);
    CHECK(r.pix32(4, 4) == 0xFF0000FFu);
}

TEST_CASE("3d: a triangle strip alternates winding without alternating faces")
{
    Rig3d r;
    r.dst32();
    r.openGate();
    // Culling ON: if the strip's second triangle were face-tested with the
    // raw winding it would vanish.
    r.reg(0x071Cu, (0u << 1) | (3u << 3) | (2u << 5));
    const auto before = r128EngStats();
    std::vector<u32> v;
    Rig3d::vtx(v, 0, 0, 0.5f, 0xFF00FFFFu);
    Rig3d::vtx(v, 32, 0, 0.5f, 0xFF00FFFFu);
    Rig3d::vtx(v, 0, 32, 0.5f, 0xFF00FFFFu);
    Rig3d::vtx(v, 32, 32, 0.5f, 0xFF00FFFFu);
    r.prim(0x9u, 6u, 4u, v); // strip: two triangles, a full quad
    const auto& e = r128EngStats();
    CHECK(e.tris - before.tris == 2);
    CHECK(e.triCulled - before.triCulled == 0);
    CHECK(e.triPixels - before.triPixels == 1024);
}

TEST_CASE("3d: nearest-neighbour texturing maps the quadrants of a 2x2")
{
    Rig3d r;
    r.dst32();
    r.openGate();
    r.fpuSolidBoth();
    // 2x2 RGB565 texture at VRAM 512 KB: red green / blue white.
    auto t16 = [&](u32 i, u32 v) {
        r.c.vram[0x80000u + i * 2u] = static_cast<u8>(v >> 8);
        r.c.vram[0x80000u + i * 2u + 1u] = static_cast<u8>(v);
    };
    t16(0, 0xF800u); // (0,0) red
    t16(1, 0x07E0u); // (1,0) green
    t16(2, 0x001Fu); // (0,1) blue
    t16(3, 0xFFFFu); // (1,1) white
    r.reg(0x1C9Cu, 1u << 4);              // TEXMAP_ENABLE, light fn disable
    r.reg(0x1CB0u, 4u << 16);             // 565, nearest, wrap
    r.reg(0x1CB8u, 0x111u);               // pitch 2, size 2, height 2
    r.reg(0x1CC0u, 0x80000u); // slot 1: offsets are size-indexed (2^1 px)
    std::vector<u32> v;
    auto vtxSt = [&](float x, float y, float s, float t) {
        v.push_back(f2u(x));
        v.push_back(f2u(y));
        v.push_back(f2u(0.5f));
        v.push_back(f2u(1.0f));
        v.push_back(0xFFFFFFFFu);
        v.push_back(f2u(s));
        v.push_back(f2u(t));
    };
    // A 16x16 quad mapped 0..1 in both axes.
    vtxSt(0, 0, 0, 0);
    vtxSt(16, 0, 1, 0);
    vtxSt(0, 16, 0, 1);
    vtxSt(16, 0, 1, 0);
    vtxSt(16, 16, 1, 1);
    vtxSt(0, 16, 0, 1);
    r.prim(0x9u | 0x80u, 4u, 6u, v);
    CHECK(r.pix32(3, 3) == 0xFFFF0000u);   // red quadrant
    CHECK(r.pix32(12, 3) == 0xFF00FF00u);  // green (565 expands exactly)
    CHECK(r.pix32(3, 12) == 0xFF0000FFu);  // blue
    CHECK(r.pix32(12, 12) == 0xFFFFFFFFu); // white
}

TEST_CASE("3d: bilinear filtering blends adjacent texels")
{
    Rig3d r;
    r.dst32();
    r.openGate();
    r.fpuSolidBoth();
    // 2x1 Y8 texture: black then white.
    r.c.vram[0x80000u] = 0;
    r.c.vram[0x80001u] = 255;
    r.reg(0x1C9Cu, 1u << 4);
    // Y8 (fmt 8), mag bilinear, clamp S+T so the sample cannot wrap.
    r.reg(0x1CB0u, (8u << 16) | (1u << 4) | (2u << 8) | (2u << 11));
    r.reg(0x1CB8u, (0u << 8) | (1u << 4) | 1u); // pitch 2, w 2, h 1
    r.reg(0x1CC0u, 0x80000u); // slot 1: size-indexed (2^1 px wide)
    std::vector<u32> v;
    auto vtxSt = [&](float x, float y, float s, float t) {
        v.push_back(f2u(x));
        v.push_back(f2u(y));
        v.push_back(f2u(0.5f));
        v.push_back(f2u(1.0f));
        v.push_back(0xFFFFFFFFu);
        v.push_back(f2u(s));
        v.push_back(f2u(t));
    };
    vtxSt(0, 0, 0, 0);
    vtxSt(8, 0, 1, 0);
    vtxSt(0, 8, 0, 1);
    vtxSt(8, 0, 1, 0);
    vtxSt(8, 8, 1, 1);
    vtxSt(0, 8, 0, 1);
    r.prim(0x9u | 0x80u, 4u, 6u, v);
    // Pixel (3,y): s = 3.5/8 = 0.4375 → texel space 0.875 − 0.5 = 0.375
    // between black and white → 95.6.
    const u32 p = r.pix32(3, 3);
    const int g = (p >> 8) & 0xFF;
    CHECK(g >= 94);
    CHECK(g <= 98);
}

TEST_CASE("3d: perspective correction divides by the interpolated rhw")
{
    Rig3d r;
    r.dst32();
    r.openGate();
    r.fpuSolidBoth();
    // A 4x1 gradient texture with strongly distinguishable texels.
    auto t16 = [&](u32 i, u32 v) {
        r.c.vram[0x80000u + i * 2u] = static_cast<u8>(v >> 8);
        r.c.vram[0x80000u + i * 2u + 1u] = static_cast<u8>(v);
    };
    t16(0, 0xF800u); // red
    t16(1, 0x07E0u);
    t16(2, 0x001Fu);
    t16(3, 0xFFFFu);
    r.reg(0x1C9Cu, 1u << 4);
    r.reg(0x1CB0u, (4u << 16) | (2u << 8) | (2u << 11)); // 565 nearest clamp
    r.reg(0x1CB8u, (0u << 8) | (2u << 4) | 2u);          // pitch 4, w 4, h 1
    r.reg(0x1CC4u, 0x80000u); // slot 2: size-indexed (2^2 px wide)
    std::vector<u32> v;
    auto vtxW = [&](float x, float y, float s, float rhw) {
        v.push_back(f2u(x));
        v.push_back(f2u(y));
        v.push_back(f2u(0.5f));
        v.push_back(f2u(rhw));
        v.push_back(0xFFFFFFFFu);
        v.push_back(f2u(s));
        v.push_back(f2u(0.0f));
    };
    // Left edge at w=1, right vertex at w=4 (rhw 0.25). At pixel (8,0) the
    // affine s would be 0.53 → texel 2 (blue); the perspective-correct s is
    // 0.22 → texel 0 (red). The colour names which path ran.
    vtxW(0, 0, 0, 1.0f);
    vtxW(16, 0, 1, 0.25f);
    vtxW(0, 16, 0, 1.0f);
    r.prim(0x9u | 0x80u, 4u, 3u, v);
    CHECK(r.pix32(8, 0) == 0xFFFF0000u);
}

TEST_CASE("3d: CI8 texels read through the palette LOAD_PALETTE filled")
{
    Rig3d r;
    r.dst32();
    r.openGate();
    r.fpuSolidBoth();
    // LOAD_PALETTE, 256 entries: index i -> blue ramp; entry 7 = pure red.
    r.fifo(0xC0002C00u | ((257u - 1u) << 16));
    r.fifo(2u); // 8bpp palette
    for (u32 i = 0; i < 256u; ++i)
        r.fifo(i == 7u ? 0x00FF0000u : 0x000000FFu);
    // 2x2 CI8 texture: indices 7,7 / 0,0.
    r.c.vram[0x80000u] = 7;
    r.c.vram[0x80001u] = 7;
    r.c.vram[0x80002u] = 0;
    r.c.vram[0x80003u] = 0;
    r.reg(0x1C9Cu, 1u << 4);
    r.reg(0x1CB0u, 2u << 16); // CI8, nearest, wrap
    r.reg(0x1CB8u, 0x111u);
    r.reg(0x1CC0u, 0x80000u); // slot 1: size-indexed (2^1 px)
    std::vector<u32> v;
    auto vtxSt = [&](float x, float y, float s, float t) {
        v.push_back(f2u(x));
        v.push_back(f2u(y));
        v.push_back(f2u(0.5f));
        v.push_back(f2u(1.0f));
        v.push_back(0xFFFFFFFFu);
        v.push_back(f2u(s));
        v.push_back(f2u(t));
    };
    vtxSt(0, 0, 0, 0);
    vtxSt(16, 0, 1, 0);
    vtxSt(0, 16, 0, 1);
    r.prim(0x9u | 0x80u, 4u, 3u, v);
    CHECK(r.pix32(2, 2) == 0xFFFF0000u); // palette entry 7
}

// ⭐⭐ THE OFFSET SLOTS ARE INDEXED BY LEVEL SIZE, NOT BY MIP NUMBER.
//
// Measured from Nanosaur's own register traffic: TEX_SIZE_PITCH_C cycled
// 0x666/0x777/0x888 while the driver wrote ONLY PRIM_TEX_6/7/8_OFFSET_C —
// one slot per texture, the slot number equal to log2 of the texture's
// size, PRIM_TEX_0_OFFSET_C never once. The model used to read the base
// level from slot 0, so every texel of every surface came from VRAM
// address zero: the whole 3D world drew black with junk flecks and not
// one counter tripped (the fetches were all "valid"). This case is shaped
// exactly like that evidence: slot-log2(size) holds the texture, slot 0
// holds ZERO, and the pixel must come out textured anyway.
TEST_CASE("3d: the texture base is read from the size-indexed offset slot, "
          "not slot 0")
{
    Rig3d r;
    r.dst32();
    r.openGate();
    r.fpuSolidBoth();
    // 4x4 solid-red 565 texture at VRAM 512 KB.
    for (u32 i = 0; i < 16u; ++i) {
        r.c.vram[0x80000u + i * 2u] = 0xF8;
        r.c.vram[0x80000u + i * 2u + 1u] = 0x00;
    }
    r.reg(0x1C9Cu, 1u << 4);
    r.reg(0x1CB0u, (4u << 16) | (2u << 8) | (2u << 11)); // 565 nearest clamp
    r.reg(0x1CB8u, 0x222u); // pitch 4, w 4, h 4 → log2 size 2
    r.reg(0x1CBCu, 0u);        // slot 0: EMPTY, as in the live capture
    r.reg(0x1CC4u, 0x80000u);  // slot 2 = log2(4): the actual texture
    std::vector<u32> v;
    auto vtxSt = [&](float x, float y, float s, float t) {
        v.push_back(f2u(x));
        v.push_back(f2u(y));
        v.push_back(f2u(0.5f));
        v.push_back(f2u(1.0f));
        v.push_back(0xFFFFFFFFu);
        v.push_back(f2u(s));
        v.push_back(f2u(t));
    };
    vtxSt(0, 0, 0, 0);
    vtxSt(16, 0, 1, 0);
    vtxSt(0, 16, 0, 1);
    r.prim(0x9u | 0x80u, 4u, 3u, v);
    CHECK(r.pix32(3, 3) == 0xFFFF0000u); // textured, NOT black-from-slot-0
}

// The tiling-mode bits ride the offset's top two bits (SDK F.13: 0 linear,
// 1 host-tiled, 2/3 via a SURFACE range). Nanosaur sets mode 3 with every
// SURFACE register zeroed — linear in fact — and 62 M samples per capture
// must not each count as "unimplemented". A LIVE surface range covering
// the address is the only honest trigger.
TEST_CASE("3d: tiling-mode bits with no live SURFACE range sample linear "
          "and count nothing")
{
    Rig3d r;
    r.dst32();
    r.openGate();
    r.fpuSolidBoth();
    for (u32 i = 0; i < 16u; ++i) {
        r.c.vram[0x80000u + i * 2u] = 0xF8;
        r.c.vram[0x80000u + i * 2u + 1u] = 0x00;
    }
    r.reg(0x1C9Cu, 1u << 4);
    r.reg(0x1CB0u, (4u << 16) | (2u << 8) | (2u << 11));
    r.reg(0x1CB8u, 0x222u);
    r.reg(0x1CC4u, 0xC0000000u | 0x80000u); // mode 3, as Nanosaur programs
    const auto before = r128EngStats();
    std::vector<u32> v;
    auto vtxSt = [&](float x, float y, float s, float t) {
        v.push_back(f2u(x));
        v.push_back(f2u(y));
        v.push_back(f2u(0.5f));
        v.push_back(f2u(1.0f));
        v.push_back(0xFFFFFFFFu);
        v.push_back(f2u(s));
        v.push_back(f2u(t));
    };
    vtxSt(0, 0, 0, 0);
    vtxSt(16, 0, 1, 0);
    vtxSt(0, 16, 0, 1);
    r.prim(0x9u | 0x80u, 4u, 3u, v);
    CHECK(r.pix32(3, 3) == 0xFFFF0000u);
    CHECK(r128EngStats().texUnimpl - before.texUnimpl == 0);
}

TEST_CASE("3d: an AGP-resident texture is fetched through the GART")
{
    Rig3d r;
    r.dst32();
    r.openGate();
    r.fpuSolidBoth();
    // Texels live in system RAM at 2 MB, reached via AGP page 0x300.
    r.mapPage(0x00300000u, 0x00200000u);
    // One 565 texel, red, in MEMORY byte order (MSB first — the convention
    // the whole engine stores pixels in).
    r.ram[0x00200000u] = 0xF8;
    r.ram[0x00200001u] = 0x00;
    r.reg(0x1C9Cu, 1u << 4);
    r.reg(0x1CB0u, (4u << 16) | (2u << 8) | (2u << 11));
    r.reg(0x1CB8u, 0x000u); // 1x1
    r.reg(0x1CBCu, 0x02000000u + 0x00300000u); // the AGP window + offset
    const auto before = r128EngStats();
    std::vector<u32> v;
    auto vtxSt = [&](float x, float y, float s, float t) {
        v.push_back(f2u(x));
        v.push_back(f2u(y));
        v.push_back(f2u(0.5f));
        v.push_back(f2u(1.0f));
        v.push_back(0xFFFFFFFFu);
        v.push_back(f2u(s));
        v.push_back(f2u(t));
    };
    vtxSt(0, 0, 0, 0);
    vtxSt(8, 0, 1, 0);
    vtxSt(0, 8, 0, 1);
    r.prim(0x9u | 0x80u, 4u, 3u, v);
    const auto& e = r128EngStats();
    CHECK(e.texGartMiss - before.texGartMiss == 0);
    CHECK(r.pix32(2, 2) == 0xFFFF0000u);
}

TEST_CASE("3d: the vertex walker draws from a GART buffer, indexed, and the "
          "bundle continues it")
{
    Rig3d r;
    r.dst32();
    r.openGate();
    r.fpuSolidBoth();
    // Vertex buffer in system RAM at 3 MB via AGP page 0x400: four
    // RHW|ARGB vertices of a quad.
    r.mapPage(0x00400000u, 0x00300000u);
    auto vbVtx = [&](u32 i, float x, float y, u32 argb) {
        const u32 at = 0x00300000u + i * 5u * 4u;
        r.put32(at, f2u(x));
        r.put32(at + 4u, f2u(y));
        r.put32(at + 8u, f2u(0.5f));
        r.put32(at + 12u, f2u(1.0f));
        r.put32(at + 16u, argb);
    };
    vbVtx(0, 0, 0, 0xFFFF00FFu);
    vbVtx(1, 32, 0, 0xFFFF00FFu);
    vbVtx(2, 0, 32, 0xFFFF00FFu);
    vbVtx(3, 32, 32, 0xFFFF00FFu);
    const auto before = r128EngStats();
    // INDX_PRIM: header, then VLOFF, VSIZE, FORMAT, CNTL and two index
    // DWORDs — triangle 0,1,2 by explicit indices (walk 1), the odd
    // index's high word padded with 0 exactly as F.26 prescribes.
    r.fifo(0xC0002300u | ((6u - 1u) << 16));
    r.fifo(0x00400000u);
    r.fifo(4u);
    r.fifo(0x9u);
    r.fifo(4u | (1u << 4) | (3u << 16));
    r.fifo((1u << 16) | 0u);
    r.fifo((0u << 16) | 2u);
    const auto& e1 = r128EngStats();
    CHECK(e1.tris - before.tris == 1);
    CHECK(e1.vtxGartMiss - before.vtxGartMiss == 0);
    CHECK(r.pix32(4, 4) == 0xFFFF00FFu);
    // NEXT_VERTEX_BUNDLE: the other half of the quad, indices 1,3,2.
    r.fifo(0xC0002E00u | ((2u - 1u) << 16));
    r.fifo((3u << 16) | 1u);
    r.fifo((0u << 16) | 2u);
    const auto& e2 = r128EngStats();
    CHECK(e2.tris - before.tris == 2);
    CHECK(e2.triPixels - before.triPixels == 1024);
}

TEST_CASE("3d: the ring fetches packets through the GART, wraps, and writes "
          "the read pointer back")
{
    Rig3d r;
    // Ring in system RAM at 4 MB via AGP page 0x500; 8 DWORDs (log2qw 2).
    r.mapPage(0x00500000u, 0x00400000u);
    // Two Type-0 single-register packets, laid so the second wraps: slots
    // 6,7 then 0,1.
    r.put32(0x00400000u + 6u * 4u, 0x00000578u); // GUI_SCRATCH_REG0
    r.put32(0x00400000u + 7u * 4u, 0xAAAA5555u);
    r.put32(0x00400000u + 0u * 4u, 0x00000579u); // GUI_SCRATCH_REG1
    r.put32(0x00400000u + 1u * 4u, 0x1234ABCDu);
    r.reg(0x0700u, 0x00500000u);         // PM4_BUFFER_OFFSET (AGP)
    r.reg(0x0704u, (2u << 28) | 2u);     // 192BM mode, 8-DWORD ring
    r.reg(0x070Cu, 0x00600000u);         // DL_RPTR_ADDR in system RAM
    r.reg(0x0710u, 6u);                  // RPTR at the wrap's edge
    r.reg(0x0714u, 2u);                  // WPTR past it
    r.reg(0x07FCu, 1u << 30);            // PM4_MICRO_CNTL FREERUN
    const auto before = r128EngStats();
    r128CceRingKick(r.c, r.ram.data(), static_cast<u32>(r.ram.size()),
                    Rig3d::kGart, 0, nullptr);
    const auto& e = r128EngStats();
    CHECK(e.cceRingKicks - before.cceRingKicks == 1);
    CHECK(e.cceRingWords - before.cceRingWords == 4);
    CHECK(r.c.peek(0x15E0u) == 0xAAAA5555u);
    CHECK(r.c.peek(0x15E4u) == 0x1234ABCDu);
    CHECK(r.c.peek(0x0710u) == 2u); // RPTR caught up
    // The write-back the SDK documents: the new RPTR, little-endian, at
    // DL_RPTR_ADDR.
    CHECK(r.ram[0x00600000u] == 2);
    CHECK(r.ram[0x00600001u] == 0);
}

TEST_CASE("3d: the SCALE_3D_FN write gate drops context writes when steered "
          "off and passes them when steered to TEXMAP_SHADE")
{
    Rig3d r;
    const auto before = r128EngStats();
    // Steer the function away from TEXMAP_SHADE: the gate closes.
    r.reg(0x1CA0u, 0u << 8);
    r.reg(0x1C98u, 0xDEADu);
    CHECK(r.c.peek(0x1C98u) == 0u); // dropped
    CHECK(r128EngStats().gated3d - before.gated3d == 1);
    // A primitive is declined too, and counted.
    std::vector<u32> v;
    Rig3d::vtx(v, 0, 0, 0.5f, 0xFF0000FFu);
    Rig3d::vtx(v, 32, 0, 0.5f, 0xFF0000FFu);
    Rig3d::vtx(v, 0, 32, 0.5f, 0xFF0000FFu);
    r.dst32();
    r.fpuSolidBoth();
    r.prim(0x9u, 4u, 3u, v);
    CHECK(r128EngStats().prim3dDecline - before.prim3dDecline == 1);
    // Open the gate: the same write lands.
    r.reg(0x1CA0u, 2u << 8);
    r.reg(0x1C98u, 0xDEADu);
    CHECK(r.c.peek(0x1C98u) == 0xDEADu);
    r.prim(0x9u, 4u, 3u, v);
    CHECK(r.pix32(4, 4) == 0xFF0000FFu);
}

// ⭐ THE WRITE GATE CLOSES ON ZERO, NOT ON "ANYTHING BUT TEXMAP_SHADE".
//
// RRG, MISC_3D_STATE_CNTL: "if this field is set to 0, many 3D/Front-End
// Scalar/Setup Engine registers are NOT writeable. Hence this field should
// be written to a NON-ZERO value prior to trying to write any other
// 3D/Front-End Scalar registers." So SCALE_3D_FN = 1 (Scaling) leaves the
// context block writeable exactly as 2 does.
//
// This model used to close the gate for every function except 2, so every
// context register a driver programmed while the pipe sat in Scaling was
// dropped on the floor — 4,392 of them in the first real Nanosaur session,
// and the primitives afterwards drew with whatever state happened to be
// left. Nothing announced it: the writes were counted, not reported.
//
// ⚠ Drawing still requires TEXMAP_SHADE — that is a separate gate on the
// primitive path, and the case above pins it. Writeable ≠ renderable.
TEST_CASE("3d: SCALE_3D_FN = Scaling leaves the context block writeable")
{
    Rig3d r;
    const auto before = r128EngStats();
    r.reg(0x1CA0u, 1u << 8); // 1 = Scaling: non-zero, so writes must land
    r.reg(0x1C98u, 0xBEEFu);
    CHECK(r.c.peek(0x1C98u) == 0xBEEFu);
    CHECK(r128EngStats().gated3d - before.gated3d == 0); // nothing dropped

    // ...and zero in BOTH registers still closes it, so the documented lock
    // is not lost.
    r.reg(0x1A00u, 0u);
    r.reg(0x1CA0u, 0u << 8);
    r.reg(0x1C98u, 0x1234u);
    CHECK(r.c.peek(0x1C98u) == 0xBEEFu); // unchanged: the write was dropped
    CHECK(r128EngStats().gated3d - before.gated3d == 1);
}

// ⭐ THE REAL SHAPE OF THE NANOSAUR BUG: two registers disagreeing.
//
// The driver ran with MISC_3D_STATE_CNTL = 00510200 (SCALE_3D_FN = 2,
// TEXMAP_SHADE) and SCALE_3D_CNTL = 80000000, whose 7:6 field is 0. With
// the gate steered by whichever register was written last, that zero closed
// it — and the block it guards holds PRIM_TEX_0_OFFSET_C, so the texture
// base addresses never landed. Every texel came from VRAM offset 0 and
// every surface rendered black with scattered bright flecks.
TEST_CASE("3d: a zero function in one register does not close the gate the "
          "other one opened")
{
    Rig3d r;
    const auto before = r128EngStats();
    r.reg(0x1CA0u, 2u << 8);   // MISC: TEXMAP_SHADE — the guest means 3D
    r.reg(0x1A00u, 0x80000000u); // SCALE_3D_CNTL: 7:6 reads 0
    // The texture base must still land, which is the whole point.
    r.reg(0x1CBCu, 0x00123450u); // PRIM_TEX_0_OFFSET_C
    CHECK(r.c.peek(0x1CBCu) == 0x00123450u);
    CHECK(r128EngStats().gated3d - before.gated3d == 0);
}

TEST_CASE("3d: a 565 destination quantises and stores big-endian pixels")
{
    Rig3d r;
    r.dst16();
    r.openGate();
    r.fpuSolidBoth();
    std::vector<u32> v;
    Rig3d::vtx(v, 0, 0, 0.5f, 0xFFFF8000u); // R 255 G 128 B 0
    Rig3d::vtx(v, 32, 0, 0.5f, 0xFFFF8000u);
    Rig3d::vtx(v, 0, 32, 0.5f, 0xFFFF8000u);
    r.prim(0x9u, 4u, 3u, v);
    CHECK(r.pix16(4, 4) == ((31u << 11) | (32u << 5) | 0u));
}

TEST_CASE("3d: stencil counts through Z-pass and gates a later draw")
{
    Rig3d r;
    r.dst32();
    r.openGate();
    r.fpuSolidBoth();
    // A 24-bit Z + 8-bit stencil surface at 128 KB, cells S=0, Z=max.
    for (u32 i = 0; i < 64u * 64u; ++i) {
        const u32 at = 0x20000u + i * 4u;
        r.c.vram[at] = 0x00;
        r.c.vram[at + 1] = 0xFF;
        r.c.vram[at + 2] = 0xFF;
        r.c.vram[at + 3] = 0xFF;
    }
    r.reg(0x1C90u, 0x20000u);
    r.reg(0x1C94u, 64u / 8u);
    // 24-bit Z, stencil ALWAYS, ZPASS op INC.
    r.reg(0x1C98u, (1u << 1) | (7u << 12) | (3u << 20));
    r.reg(0x1C9Cu, 1u << 3); // STENCIL_ENABLE (Z off)
    r.reg(0x1D40u, 0xFFu << 16 | 0xFFu << 24);
    std::vector<u32> v;
    Rig3d::vtx(v, 0, 0, 0.5f, 0xFF808080u);
    Rig3d::vtx(v, 32, 0, 0.5f, 0xFF808080u);
    Rig3d::vtx(v, 0, 32, 0.5f, 0xFF808080u);
    r.prim(0x9u, 4u, 3u, v);
    CHECK(r.c.vram[0x20000u + (4u * 64u + 4u) * 4u] == 1); // S incremented
    // Second draw passes only where stencil == 0 — outside the first
    // triangle. Inside must stay untouched.
    r.reg(0x1C98u, (1u << 1) | (3u << 12)); // STENCIL_TEST_EQUAL, ref 0
    r.reg(0x1D40u, (0xFFu << 16) | (0xFFu << 24) | 0u);
    std::vector<u32> w;
    Rig3d::vtx(w, 0, 0, 0.5f, 0xFF00FF00u);
    Rig3d::vtx(w, 64, 0, 0.5f, 0xFF00FF00u);
    Rig3d::vtx(w, 0, 64, 0.5f, 0xFF00FF00u);
    r.prim(0x9u, 4u, 3u, w);
    CHECK(r.pix32(4, 4) == 0xFF808080u);  // stencil 1: rejected
    CHECK(r.pix32(40, 4) == 0xFF00FF00u); // stencil 0: drawn
}

TEST_CASE("3d: points and lines land with their attributes")
{
    Rig3d r;
    r.dst32();
    r.openGate();
    r.fpuSolidBoth();
    const auto before = r128EngStats();
    std::vector<u32> pt, ln;
    Rig3d::vtx(pt, 10, 10, 0.5f, 0xFFFFFFFFu);
    r.prim(0x9u, 1u, 1u, pt); // one point
    Rig3d::vtx(ln, 0, 20, 0.5f, 0xFFFF0000u);
    Rig3d::vtx(ln, 9, 20, 0.5f, 0xFFFF0000u);
    r.prim(0x9u, 2u, 2u, ln); // one horizontal line
    const auto& e = r128EngStats();
    CHECK(e.points3d - before.points3d == 1);
    CHECK(e.lines3d - before.lines3d == 1);
    CHECK(r.pix32(10, 10) == 0xFFFFFFFFu);
    CHECK(r.pix32(0, 20) == 0xFFFF0000u);
    CHECK(r.pix32(9, 20) == 0xFFFF0000u);
    CHECK(r.pix32(5, 20) == 0xFFFF0000u);
}
