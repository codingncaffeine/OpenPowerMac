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
#include "opm/bus.hpp" // SnoopSink
#include "opm/r128.hpp"

#include <cstring>
#include <string>
#include <utility>
#include <memory>
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
    // Who the card asks before it reads system memory. None, as on the
    // engine's own thread: texels are then read in place. A sink that does
    // not say memory is current puts every texel back on the byte-by-byte
    // path.
    SnoopSink* snoop = nullptr;
    void fifo(u32 w)
    {
        r128CceFifoWord(c, be(w), 4, ram.data(),
                        static_cast<u32>(ram.size()), kGart, 0, snoop);
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

TEST_CASE("3d: Gouraud interpolation lands the corner colors and the blend")
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
    // Z16 buffer at 128 KB, 64-px pitch, initialized to the far plane.
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

TEST_CASE("3d: vertex fog carries the fragment to the fog color")
{
    Rig3d r;
    r.dst32();
    r.openGate();
    r.fpuSolidBoth();
    r.reg(0x1C9Cu, 1u << 7);        // FOG_ENABLE, vertex fog (misc bit14=0)
    r.reg(0x1CACu, 0x0000FF00u);    // fog color: green
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
    r.reg(0x1CACu, 0x000000FFu);            // fog color: blue
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

TEST_CASE("3d: backface culling honors the winding and the front-face rule")
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

TEST_CASE("3d: nearest-neighbor texturing maps the quadrants of a 2x2")
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

// TEX_CNTL_C bit 13: the texel's one alpha bit as a cutout. A driver sets it
// with blending and the alpha test both off for textures that carry a shape,
// and the texels outside the shape are not drawn at all — not their color,
// and not their depth, or whatever is drawn behind the quad afterwards would
// be cut to its rectangle anyway.
TEST_CASE("3d: the texel alpha mask draws nothing where the alpha bit is clear")
{
    for (const bool mask : {true, false}) {
        Rig3d r;
        r.dst32();
        r.openGate();
        r.fpuSolidBoth();
        for (u32 i = 0; i < 64u * 64u * 2u; ++i)
            r.c.vram[0x20000u + i] = 0xFF; // Z16 at 128 KB, far plane
        r.reg(0x1C90u, 0x20000u);
        r.reg(0x1C94u, 64u / 8u);
        r.reg(0x1C98u, (0u << 1) | (1u << 4)); // 16-bit, Z_TEST_LESS
        // 2x2 ARGB1555 at VRAM 512 KB: opaque red, clear black /
        //                              clear green, opaque white.
        auto t16 = [&](u32 i, u32 v) {
            r.c.vram[0x80000u + i * 2u] = static_cast<u8>(v >> 8);
            r.c.vram[0x80000u + i * 2u + 1u] = static_cast<u8>(v);
        };
        t16(0, 0xFC00u);
        t16(1, 0x0000u);
        t16(2, 0x03E0u);
        t16(3, 0xFFFFu);
        r.reg(0x1CB0u, 3u << 16); // ARGB1555, nearest, wrap
        r.reg(0x1CB8u, 0x111u);
        r.reg(0x1CC0u, 0x80000u);
        // Z on, Z write on, texture on, and the mask or not.
        r.reg(0x1C9Cu, 0x3u | (1u << 4) | (mask ? 1u << 13 : 0u));
        std::vector<u32> v;
        auto vtxSt = [&](float x, float y, float s, float t) {
            v.push_back(f2u(x));
            v.push_back(f2u(y));
            v.push_back(f2u(0.25f));
            v.push_back(f2u(1.0f));
            v.push_back(0xFFFFFFFFu);
            v.push_back(f2u(s));
            v.push_back(f2u(t));
        };
        vtxSt(0, 0, 0, 0);
        vtxSt(16, 0, 1, 0);
        vtxSt(0, 16, 0, 1);
        vtxSt(16, 0, 1, 0);
        vtxSt(16, 16, 1, 1);
        vtxSt(0, 16, 0, 1);
        r.prim(0x9u | 0x80u, 4u, 6u, v);
        // Then something behind the quad, untextured: it shows wherever the
        // quad left the depth alone.
        r.reg(0x1C9Cu, 0x3u);
        std::vector<u32> behind;
        for (const auto& [x, y] : {std::pair{0.f, 0.f}, {16.f, 0.f}, {0.f, 16.f},
                                   {16.f, 0.f}, {16.f, 16.f}, {0.f, 16.f}})
            Rig3d::vtx(behind, x, y, 0.75f, 0xFF123456u);
        r.prim(0x9u, 4u, 6u, behind);

        CAPTURE(mask);
        CHECK(r.pix32(3, 3) == 0xFFFF0000u);   // opaque red, in front
        CHECK(r.pix32(12, 12) == 0xFFFFFFFFu); // opaque white, in front
        if (mask) {
            CHECK(r.pix32(12, 3) == 0xFF123456u); // cut away: what is behind
            CHECK(r.pix32(3, 12) == 0xFF123456u);
        } else {
            CHECK(r.pix32(12, 3) == 0x00000000u); // the texel's own black
            CHECK(r.pix32(3, 12) == 0x0000FF00u); // …and its own green
        }
    }
}

// The LOD bias is TEX_CNTL_C 31:24 (r128_reg.h), the pipeline's register. The
// top byte of a unit's own control register is something else — the datatype
// of its palette entries — and a driver that sets it must get no bias for it.
TEST_CASE("3d: the LOD bias comes from TEX_CNTL_C, not the unit's register")
{
    // A 2x2 red base with a 1x1 blue mip, drawn one texel to a pixel: level
    // 0 unless something biases it.
    auto draw = [](u32 texCntl, u32 unitCntl) {
        Rig3d r;
        r.dst32();
        r.openGate();
        r.fpuSolidBoth();
        auto t16 = [&](u32 at, u32 v) {
            r.c.vram[at] = static_cast<u8>(v >> 8);
            r.c.vram[at + 1u] = static_cast<u8>(v);
        };
        for (u32 i = 0; i < 4; ++i)
            t16(0x80000u + i * 2u, 0xF800u); // red
        t16(0x90000u, 0x001Fu);              // blue
        r.reg(0x1C9Cu, (1u << 4) | texCntl);
        r.reg(0x1CB0u, (4u << 16) | unitCntl); // 565
        r.reg(0x1CB8u, 0x0111u);  // 2x2, mips down to 1x1
        r.reg(0x1CC0u, 0x80000u); // slot 1: the 2-texel level
        r.reg(0x1CBCu, 0x90000u); // slot 0: the 1-texel level
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
        vtxSt(2, 0, 1, 0);
        vtxSt(0, 2, 0, 1);
        vtxSt(2, 0, 1, 0);
        vtxSt(2, 2, 1, 1);
        vtxSt(0, 2, 0, 1);
        r.prim(0x9u | 0x80u, 4u, 6u, v);
        return r.pix32(0, 0);
    };
    const u32 red = 0xFFFF0000u, blue = 0xFF0000FFu;
    const u32 nearestMip = 2u << 1, trilinear = 5u << 1;
    CHECK(draw(0, nearestMip) == red);
    // One whole level of bias, 4.4 fixed point, in the pipeline's register.
    CHECK(draw(0x10u << 24, nearestMip) == blue);
    // The palette datatype in the unit's register: no bias, so a trilinear
    // sample at level 0 is all level 0.
    CHECK(draw(0, trilinear | (1u << 24)) == red);
    CHECK(draw(0, trilinear | (2u << 24)) == red);
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

TEST_CASE("3d: perspective correction divides premultiplied S by rhw")
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
    t16(1, 0x07E0u); // green
    t16(2, 0x001Fu);
    t16(3, 0xFFFFu);
    r.reg(0x1C9Cu, 1u << 4);
    r.reg(0x1CB0u, (4u << 16) | (2u << 8) | (2u << 11)); // 565 nearest clamp
    r.reg(0x1CB8u, (0u << 8) | (2u << 4) | 2u);          // pitch 4, w 4, h 1
    r.reg(0x1CC4u, 0x80000u); // slot 2: size-indexed (2^2 px wide)
    r.reg(0x1BC4u, 1u << 9);  // SETUP_CNTL: TEXTURE_ST_DIRECT, the Mac's
    std::vector<u32> v;
    auto vtxW = [&](float x, float y, float u, float rhw) {
        v.push_back(f2u(x));
        v.push_back(f2u(y));
        v.push_back(f2u(0.5f));
        v.push_back(f2u(rhw));
        v.push_back(0xFFFFFFFFu);
        v.push_back(f2u(u * rhw)); // the stream carries u·rhw (uOverW),
        v.push_back(f2u(0.0f));    // the measured FTLVERTEX contract
    };
    // u runs 0→1 across the span; left edge at w=1, right vertex at w=4
    // (rhw 0.25), so the stream numerators are 0 and 0.25. At pixel (12,0):
    // numerator 0.195, rhw 0.414 → perspective-correct u = 0.47 → texel 1
    // (green). Skipping the divide reads the numerator as u: 0.195 → texel
    // 0 (red). Re-multiplying by rhw (the old double-multiply defect):
    // 0.049/0.414 → texel 0 as well. The color names the one right path.
    vtxW(0, 0, 0, 1.0f);
    vtxW(16, 0, 1, 0.25f);
    vtxW(0, 16, 0, 1.0f);
    r.prim(0x9u | 0x80u, 4u, 3u, v);
    CHECK(r.pix32(12, 0) == 0xFF00FF00u);
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
// Measured from a game's own register traffic: TEX_SIZE_PITCH_C cycled
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
// 1 host-tiled, 2/3 via a SURFACE range). The game sets mode 3 with every
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
    r.reg(0x1CC4u, 0xC0000000u | 0x80000u); // mode 3, as the game programs
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
    // One 565 texel, red (0xF800), as the chip holds it: system memory is
    // read with no aperture in between, so the low byte comes first.
    r.ram[0x00200000u] = 0x00;
    r.ram[0x00200001u] = 0xF8;
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
// dropped on the floor — 4,392 of them in the first real game session,
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

// ⭐ THE REAL SHAPE OF THE BUG: two registers disagreeing.
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

TEST_CASE("3d: a 565 destination quantizes and stores big-endian pixels")
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

// ── Dithering (SDK 6-53; r128_3d.cpp "Dithering" says what is and is not
// claimed) ────────────────────────────────────────────────────────────────

namespace {

// A flat 30x8 rectangle of one color, as two triangles, into the 565
// destination. Returns the sums over its 240 pixels of each channel put
// back on the 0..255 scale.
struct Sums {
    u32 r = 0, g = 0, b = 0;
};
Sums flatRect(Rig3d& r, u32 argb)
{
    std::vector<u32> v;
    Rig3d::vtx(v, 0, 0, 0.5f, argb);
    Rig3d::vtx(v, 30, 0, 0.5f, argb);
    Rig3d::vtx(v, 0, 8, 0.5f, argb);
    Rig3d::vtx(v, 30, 0, 0.5f, argb);
    Rig3d::vtx(v, 30, 8, 0.5f, argb);
    Rig3d::vtx(v, 0, 8, 0.5f, argb);
    r.prim(0x9u, 4u, 6u, v);
    Sums s;
    for (u32 y = 0; y < 8; ++y)
        for (u32 x = 0; x < 30; ++x) {
            const u32 p = r.pix16(x, y);
            s.r += ((p >> 11) & 0x1Fu) * 8u;
            s.g += ((p >> 5) & 0x3Fu) * 4u;
            s.b += (p & 0x1Fu) * 8u;
        }
    return s;
}

constexpr u32 kTexCntlC = 0x1C9Cu, kScale3dCntl = 0x1A00u;
constexpr u32 kDitherEn = 1u << 8;

} // namespace

TEST_CASE("3d: error diffusion stores a flat color's mean, where cutting the "
          "low bits stored it too dark")
{
    // Red 100, green 101, blue 7 of 255: none of them a level the 565
    // destination has.
    const u32 color = 0xFF646507u;
    {
        Rig3d r;
        r.dst16();
        r.openGate();
        r.fpuSolidBoth();
        const Sums cut = flatRect(r, color); // dithering off: as before
        CHECK(cut.r == 240u * 96u);
        CHECK(cut.g == 240u * 100u);
        CHECK(cut.b == 0u);
        CHECK(r.pix16(3, 3) == ((12u << 11) | (25u << 5) | 0u));
    }
    Rig3d r;
    r.dst16();
    r.openGate();
    r.fpuSolidBoth();
    r.reg(kTexCntlC, kDitherEn); // SCALE_3D_CNTL is zero: error diffusion
    const Sums d = flatRect(r, color);
    // What is stored plus what the register still holds is exactly what was
    // asked for, so the sums are short by less than one step.
    CHECK(d.r <= 240u * 100u);
    CHECK(d.r > 240u * 100u - 8u);
    CHECK(d.g <= 240u * 101u);
    CHECK(d.g > 240u * 101u - 4u);
    CHECK(d.b <= 240u * 7u);
    CHECK(d.b > 240u * 7u - 8u);
    // And it is a dither: both neighboring levels are in the picture.
    bool low = false, high = false;
    for (u32 x = 0; x < 30; ++x) {
        const u32 r5 = (r.pix16(x, 0) >> 11) & 0x1Fu;
        low = low || r5 == 12u;
        high = high || r5 == 13u;
        CHECK((r5 == 12u || r5 == 13u));
    }
    CHECK(low);
    CHECK(high);
}

TEST_CASE("3d: DITHER_INIT resets the error where each scanline begins")
{
    // A right triangle with its upright edge at x = 0: every row's span
    // starts there. Red 101 leaves 5 of 8 in the register per pixel, so the
    // pattern along a row depends on what the row began with.
    auto draw = [](Rig3d& r, u32 scale3d) {
        r.dst16();
        r.openGate();
        r.fpuSolidBoth();
        r.reg(kScale3dCntl, scale3d);
        r.reg(kTexCntlC, kDitherEn);
        std::vector<u32> v;
        Rig3d::vtx(v, 0, 0, 0.5f, 0xFF650000u);
        Rig3d::vtx(v, 30, 0, 0.5f, 0xFF650000u);
        Rig3d::vtx(v, 0, 30, 0.5f, 0xFF650000u);
        r.prim(0x9u, 4u, 3u, v);
    };
    auto rowsAlike = [](const Rig3d& r) {
        for (u32 y = 1; y < 12; ++y)
            for (u32 x = 0; x < 12; ++x)
                if (r.pix16(x, y) != r.pix16(x, 0))
                    return false;
        return true;
    };
    Rig3d reset;
    draw(reset, 1u << 3); // DITHER_INIT = 1
    CHECK(rowsAlike(reset));
    Rig3d kept;
    draw(kept, 0u); // DITHER_INIT = 0: the register runs on from row to row
    CHECK(!rowsAlike(kept));
}

TEST_CASE("3d: table dither and a 32 bpp destination are drawn as with "
          "dithering off")
{
    const u32 color = 0xFF646507u;
    Rig3d table;
    table.dst16();
    table.openGate();
    table.fpuSolidBoth();
    table.reg(kScale3dCntl, 1u << 1); // SCALE_DITHER = 1: the table method
    table.reg(kTexCntlC, kDitherEn);
    const Sums t = flatRect(table, color);
    CHECK(t.r == 240u * 96u);
    CHECK(t.g == 240u * 100u);
    CHECK(t.b == 0u);

    Rig3d deep;
    deep.dst32();
    deep.openGate();
    deep.fpuSolidBoth();
    deep.reg(kTexCntlC, kDitherEn);
    std::vector<u32> v;
    Rig3d::vtx(v, 0, 0, 0.5f, color);
    Rig3d::vtx(v, 30, 0, 0.5f, color);
    Rig3d::vtx(v, 0, 8, 0.5f, color);
    deep.prim(0x9u, 4u, 3u, v);
    for (u32 x = 0; x < 8; ++x)
        CHECK(deep.pix32(x, 1) == color);
}

// ── Sub-pixel precision (r128_3d.cpp snapVertex) ──────────────────────────

namespace {

constexpr u32 kFpuSetup = 0x071Cu, kWindowXyOffset = 0x1BCCu;
constexpr u32 kFpuBoth = (3u << 1) | (3u << 3) | (2u << 5); // fpuSolidBoth's
constexpr u32 kFpuSub4 = 1u << 7, kFpuNearest = 1u << 15;

// A rectangle from the origin to (right, 8) in solid blue, as two triangles
// that share the upright edge at x = right.
void rectTo(Rig3d& r, float right)
{
    std::vector<u32> v;
    Rig3d::vtx(v, 0, 0, 0.5f, 0xFF0000FFu);
    Rig3d::vtx(v, right, 0, 0.5f, 0xFF0000FFu);
    Rig3d::vtx(v, 0, 8, 0.5f, 0xFF0000FFu);
    Rig3d::vtx(v, right, 0, 0.5f, 0xFF0000FFu);
    Rig3d::vtx(v, right, 8, 0.5f, 0xFF0000FFu);
    Rig3d::vtx(v, 0, 8, 0.5f, 0xFF0000FFu);
    r.prim(0x9u, 4u, 6u, v);
}

} // namespace

TEST_CASE("3d: a corner is placed on the sub-pixel grid the guest selects")
{
    // The edge is asked for at x = 10.7. Pixel 10's center is 10.5: inside
    // an edge at 10.7 or 10.6875 or 10.75, and ON an edge at 10.5, which a
    // right-hand edge does not take.
    {
        Rig3d r; // two fractional bits, cut: the edge is at 10.5
        r.dst32();
        r.openGate();
        r.reg(kFpuSetup, kFpuBoth);
        rectTo(r, 10.7f);
        CHECK(r.pix32(9, 3) == 0xFF0000FFu);
        CHECK(r.pix32(10, 3) == 0u);
    }
    {
        Rig3d r; // four fractional bits: 10.6875
        r.dst32();
        r.openGate();
        r.reg(kFpuSetup, kFpuBoth | kFpuSub4);
        rectTo(r, 10.7f);
        CHECK(r.pix32(10, 3) == 0xFF0000FFu);
        CHECK(r.pix32(11, 3) == 0u);
    }
    {
        Rig3d r; // two bits, rounded to the nearest step: 10.75
        r.dst32();
        r.openGate();
        r.reg(kFpuSetup, kFpuBoth | kFpuNearest);
        rectTo(r, 10.7f);
        CHECK(r.pix32(10, 3) == 0xFF0000FFu);
        CHECK(r.pix32(11, 3) == 0u);
    }
}

TEST_CASE("3d: two triangles that share snapped corners still meet exactly")
{
    // The shared diagonal runs between corners that are not on the grid.
    // Each pixel of the rectangle must be drawn once: a gap would leave a
    // zero, and a pixel drawn by both would show in the count.
    Rig3d r;
    r.dst32();
    r.openGate();
    r.reg(kFpuSetup, kFpuBoth);
    const auto before = r128EngStats();
    std::vector<u32> v;
    const float l = 1.3f, t = 2.6f, rt = 21.9f, b = 13.1f;
    Rig3d::vtx(v, l, t, 0.5f, 0xFF0000FFu);
    Rig3d::vtx(v, rt, t, 0.5f, 0xFF0000FFu);
    Rig3d::vtx(v, l, b, 0.5f, 0xFF0000FFu);
    Rig3d::vtx(v, rt, t, 0.5f, 0xFF0000FFu);
    Rig3d::vtx(v, rt, b, 0.5f, 0xFF0000FFu);
    Rig3d::vtx(v, l, b, 0.5f, 0xFF0000FFu);
    r.prim(0x9u, 4u, 6u, v);
    // Snapped: x from 1.25 to 21.75, y from 2.5 to 13.0. Centers taken:
    // x 1..21 (1.5 .. 21.5), y 2..12 (2.5 on the top edge, which a top edge
    // takes, to 12.5).
    u32 drawn = 0;
    for (u32 y = 0; y < 20; ++y)
        for (u32 x = 0; x < 30; ++x) {
            const bool inside = x >= 1 && x <= 21 && y >= 2 && y <= 12;
            CAPTURE(x);
            CAPTURE(y);
            CHECK(r.pix32(x, y) == (inside ? 0xFF0000FFu : 0u));
            drawn += inside;
        }
    CHECK(r128EngStats().triPixels - before.triPixels == drawn);
}

TEST_CASE("3d: the window offset is in the vertices' own fixed point")
{
    // X = 8, Y = 4 in the two halves of WINDOW_XY_OFFSET (guide: S.12.2 or
    // S.11.4 with the sub-pixel amount).
    auto draw = [](Rig3d& r, u32 fpu) {
        r.dst32();
        r.openGate();
        r.reg(kFpuSetup, fpu);
        r.reg(kWindowXyOffset, (8u << 16) | 4u);
        std::vector<u32> v;
        Rig3d::vtx(v, 0, 0, 0.5f, 0xFF0000FFu);
        Rig3d::vtx(v, 6, 0, 0.5f, 0xFF0000FFu);
        Rig3d::vtx(v, 0, 6, 0.5f, 0xFF0000FFu);
        r.prim(0x9u, 4u, 3u, v);
    };
    Rig3d two; // two fractional bits: 2 pixels across, 1 down
    draw(two, kFpuBoth);
    CHECK(two.pix32(0, 0) == 0u);
    CHECK(two.pix32(1, 1) == 0u);
    CHECK(two.pix32(2, 0) == 0u);
    CHECK(two.pix32(2, 1) == 0xFF0000FFu);
    CHECK(two.pix32(6, 1) == 0xFF0000FFu); // the far corner moved with it
    Rig3d four; // four bits: half a pixel across, a quarter down
    draw(four, kFpuBoth | kFpuSub4);
    CHECK(four.pix32(0, 0) == 0xFF0000FFu);
    CHECK(four.pix32(6, 1) == 0u);
    // A negative offset is negative: -4 steps across is one pixel left.
    Rig3d neg;
    neg.dst32();
    neg.openGate();
    neg.reg(kFpuSetup, kFpuBoth);
    neg.reg(kWindowXyOffset, 0xFFFC0000u);
    std::vector<u32> v;
    Rig3d::vtx(v, 4, 0, 0.5f, 0xFF0000FFu);
    Rig3d::vtx(v, 10, 0, 0.5f, 0xFF0000FFu);
    Rig3d::vtx(v, 4, 6, 0.5f, 0xFF0000FFu);
    neg.prim(0x9u, 4u, 3u, v);
    CHECK(neg.pix32(2, 0) == 0u);
    CHECK(neg.pix32(3, 0) == 0xFF0000FFu);
}

// ── The remaining texture settings ────────────────────────────────────────

namespace {

// A 16x16 right triangle textured 0..1 from a 2x2 texture at 0x80000, with
// the given format word for PRIM_TEX_CNTL_C.
void texturedTriangle(Rig3d& r, u32 texCntl)
{
    r.reg(0x1C9Cu, 1u << 4);  // TEXMAP_ENABLE
    r.reg(0x1CB0u, texCntl);  // format, nearest, wrap
    r.reg(0x1CB8u, 0x111u);   // pitch 2, 2 wide, 2 high
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
    vtxSt(32, 0, 2, 0);
    vtxSt(0, 32, 0, 2);
    r.prim(0x9u | 0x80u, 4u, 3u, v);
}

} // namespace

TEST_CASE("3d: a CI4 texel is a byte, its index in the low four bits")
{
    Rig3d r;
    r.dst32();
    r.openGate();
    r.fpuSolidBoth();
    // LOAD_PALETTE, 16 entries: entry 7 red, the rest blue.
    r.fifo(0xC0002C00u | ((17u - 1u) << 16));
    r.fifo(1u); // 4 bpp palette
    for (u32 i = 0; i < 16u; ++i)
        r.fifo(i == 7u ? 0x00FF0000u : 0x000000FFu);
    // 2x2: indices 7,7 / 0,0, with junk in the unused high bits. Read two
    // texels a byte, the second texel of row 0 would be 0xF: blue.
    r.c.vram[0x80000u] = 0xF7;
    r.c.vram[0x80001u] = 0xA7;
    r.c.vram[0x80002u] = 0x30;
    r.c.vram[0x80003u] = 0x00;
    texturedTriangle(r, 1u << 16);
    CHECK(r.pix32(3, 3) == 0xFFFF0000u);  // texel (0,0)
    CHECK(r.pix32(12, 3) == 0xFFFF0000u); // texel (1,0)
    CHECK(r.pix32(3, 12) == 0xFF0000FFu); // texel (0,1)
}

TEST_CASE("3d: the 16-bit alpha formats carry alpha above an index or a "
          "gray level")
{
    {
        Rig3d r; // 10: a:pseudocolor 8:8
        r.dst32();
        r.openGate();
        r.fpuSolidBoth();
        r.fifo(0xC0002C00u | ((257u - 1u) << 16));
        r.fifo(2u);
        for (u32 i = 0; i < 256u; ++i)
            r.fifo(i == 7u ? 0x00FF0000u : 0x000000FFu);
        const u8 tex[8] = {0x80, 7, 0xFF, 0, 0x10, 0, 0x20, 7};
        for (u32 i = 0; i < 8; ++i)
            r.c.vram[0x80000u + i] = tex[i];
        const auto before = r128EngStats().texUnimpl;
        texturedTriangle(r, 10u << 16);
        CHECK(r128EngStats().texUnimpl == before);
        CHECK(r.pix32(3, 3) == 0x80FF0000u);  // alpha 0x80, palette entry 7
        CHECK(r.pix32(12, 3) == 0xFF0000FFu); // alpha 0xFF, entry 0
        CHECK(r.pix32(3, 12) == 0x100000FFu);
    }
    {
        Rig3d r; // 13: a:gray 8:8
        r.dst32();
        r.openGate();
        r.fpuSolidBoth();
        const u8 tex[8] = {0x40, 0xC8, 0xFF, 0x10, 0x00, 0xFF, 0x80, 0x80};
        for (u32 i = 0; i < 8; ++i)
            r.c.vram[0x80000u + i] = tex[i];
        const auto before = r128EngStats().texUnimpl;
        texturedTriangle(r, 13u << 16);
        CHECK(r128EngStats().texUnimpl == before);
        CHECK(r.pix32(3, 3) == 0x40C8C8C8u);
        CHECK(r.pix32(12, 3) == 0xFF101010u);
        CHECK(r.pix32(3, 12) == 0x00FFFFFFu);
    }
    {
        Rig3d r; // 0: VQ has no code book in any manual, and is still declined
        r.dst32();
        r.openGate();
        r.fpuSolidBoth();
        const auto before = r128EngStats().texUnimpl;
        texturedTriangle(r, 0u << 16);
        CHECK(r128EngStats().texUnimpl > before);
    }
}

TEST_CASE("3d: with TEXTURE_ST_MULT_W the chip multiplies plain S and T by W")
{
    // The perspective case again, the stream carrying plain u this time and
    // SETUP_CNTL's bit 9 clear: the same pixel must come out. Left as they
    // are, plain coordinates would be divided by rhw once too often: at
    // pixel (12,0) u = 0.195 / 0.414 = 0.47 (texel 1, green) becomes
    // 0.75 / 0.414, past the last texel (clamped: white).
    Rig3d r;
    r.dst32();
    r.openGate();
    r.fpuSolidBoth();
    auto t16 = [&](u32 i, u32 v) {
        r.c.vram[0x80000u + i * 2u] = static_cast<u8>(v >> 8);
        r.c.vram[0x80000u + i * 2u + 1u] = static_cast<u8>(v);
    };
    t16(0, 0xF800u);
    t16(1, 0x07E0u);
    t16(2, 0x001Fu);
    t16(3, 0xFFFFu);
    r.reg(0x1C9Cu, 1u << 4);
    r.reg(0x1CB0u, (4u << 16) | (2u << 8) | (2u << 11));
    r.reg(0x1CB8u, (0u << 8) | (2u << 4) | 2u);
    r.reg(0x1CC4u, 0x80000u);
    r.reg(0x1BC4u, 0u); // SETUP_CNTL: TEXTURE_ST_MULT_W
    std::vector<u32> v;
    auto vtxW = [&](float x, float y, float u, float rhw) {
        v.push_back(f2u(x));
        v.push_back(f2u(y));
        v.push_back(f2u(0.5f));
        v.push_back(f2u(rhw));
        v.push_back(0xFFFFFFFFu);
        v.push_back(f2u(u)); // plain
        v.push_back(f2u(0.0f));
    };
    vtxW(0, 0, 0, 1.0f);
    vtxW(16, 0, 1, 0.25f);
    vtxW(0, 16, 0, 1.0f);
    r.prim(0x9u | 0x80u, 4u, 3u, v);
    CHECK(r.pix32(12, 0) == 0xFF00FF00u);
}

// ---- the rasterizer's shortcuts against what they stand for ----------------

TEST_CASE("3d: a triangle covers exactly the pixels its three edges pass")
{
    // The rasterizer finds each row's run of pixels from where the edges
    // cross it, and visits nothing else. What it must draw is still the
    // definition: a pixel whose center passes all three edge functions, an
    // edge through a center counting when it is a top or a left one. Corners
    // sit on the quarter-pixel grid the chip uses, so centers on an edge are
    // common and every product here is exact.
    struct Xs {
        u64 s;
        u32 next()
        {
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            return static_cast<u32>(s >> 32);
        }
    } rng{0x7A51A9E5ull};
    u32 drawn = 0, onEdge = 0;
    // One card for all of them (it carries 32 MB of video memory, which is
    // not worth making four hundred times): the 64x64 target is wiped
    // before each triangle.
    Rig3d r;
    r.dst32();
    r.openGate();
    r.fpuSolidBoth();
    for (u32 iter = 0; iter < 400; ++iter) {
        std::memset(r.c.vram.data(), 0, 64u * 64u * 4u);
        double x[3], y[3];
        for (int k = 0; k < 3; ++k) {
            // -8..72 in quarter pixels: corners off every side of the
            // 64x64 scissor, slivers, and small triangles alike.
            const u32 span = iter % 3u == 0u ? 64u : 320u;
            const u32 base = iter % 3u == 0u ? 100u + (rng.next() % 64u) : 0u;
            x[k] = (static_cast<double>((base + rng.next() % span) % 320u) -
                    32.0) / 4.0;
            y[k] = (static_cast<double>((base + rng.next() % span) % 320u) -
                    32.0) / 4.0;
        }
        std::vector<u32> v;
        for (int k = 0; k < 3; ++k)
            Rig3d::vtx(v, static_cast<float>(x[k]), static_cast<float>(y[k]),
                       0.5f, 0xFFFFFFFFu);
        r.prim(0x9u, 4u, 3u, v);

        double area2 = (x[1] - x[0]) * (y[2] - y[0]) -
                       (x[2] - x[0]) * (y[1] - y[0]);
        if (area2 < 0.0) {
            std::swap(x[1], x[2]);
            std::swap(y[1], y[2]);
        }
        u32 wrong = 0;
        for (u32 py = 0; py < 64; ++py)
            for (u32 px = 0; px < 64; ++px) {
                const double cx = px + 0.5, cy = py + 0.5;
                bool in = area2 != 0.0;
                for (int k = 0; k < 3 && in; ++k) {
                    const double ax = x[k], ay = y[k];
                    const double bx = x[(k + 1) % 3], by = y[(k + 1) % 3];
                    const double ev = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
                    const bool topLeft = (by - ay == 0.0 && bx - ax > 0.0) ||
                                         by - ay < 0.0;
                    if (ev == 0.0)
                        ++onEdge;
                    in = ev > 0.0 || (ev == 0.0 && topLeft);
                }
                const bool got = r.pix32(px, py) != 0u;
                wrong += got != in;
                drawn += got;
            }
        CAPTURE(iter);
        CHECK(wrong == 0u);
    }
    // The comparison was of something: pixels were drawn, and centers did
    // fall exactly on edges.
    CHECK(drawn > 20000u);
    CHECK(onEdge > 500u);
}

TEST_CASE("3d: a texel's bytes are high first in video memory and low first "
          "in AGP space")
{
    // The chip's memory is little-endian and the big-endian aperture swaps
    // what a host stores in video memory; AGP space is system memory, read
    // with nothing in between. So one texture drawn from each place, its
    // texels' bytes the other way round in AGP space, is one picture: for
    // every format of more than a byte, on the byte-by-byte path and in
    // place, point sampled and filtered.
    struct NeverCurrent final : SnoopSink {
        void snoopRead(u32, u32) override {}
        void snoopWrite(u32, u32) override {}
    } sink;
    u32 cases = 0;
    for (u32 fmt : {3u, 4u, 5u, 6u, 10u, 11u, 12u, 13u, 14u, 15u}) {
        // A texel's size, and the size of the unit its bytes are ordered
        // in: the packed 4:2:2 formats hold two texels in four bytes.
        const u32 n = fmt == 5u ? 3u : fmt == 6u || fmt == 14u ? 4u : 2u;
        const u32 unit = fmt == 11u || fmt == 12u ? 4u : n;
        for (u32 bilinear = 0; bilinear < 2; ++bilinear)
            for (u32 byBytes = 0; byBytes < 2; ++byBytes) {
                u32 hash[2] = {0, 0};
                u64 lit[2] = {0, 0};
                for (u32 agp = 0; agp < 2; ++agp) {
                    Rig3d r;
                    if (byBytes)
                        r.snoop = &sink;
                    r.dst32();
                    r.openGate();
                    r.fpuSolidBoth();
                    r.mapPage(0x00300000u, 0x00200000u);
                    // A palette for the format that indexes one: every
                    // entry its own color (LOAD_PALETTE, 256 entries).
                    if (fmt == 10u) {
                        r.fifo(0xC0002C00u | ((257u - 1u) << 16));
                        r.fifo(2u);
                        for (u32 i = 0; i < 256u; ++i)
                            r.fifo(0xFF000000u | (i << 16) |
                                   ((255u - i) << 8) | (i * 7u & 0xFFu));
                    }
                    // An 8x8 texture, no two bytes of a unit alike.
                    for (u32 k = 0; k < 64u * n; ++k) {
                        const u8 b = static_cast<u8>(k * 53u + fmt * 29u + 7u);
                        if (!agp)
                            r.c.vram[0x80000u + k] = b;
                        else
                            r.ram[0x00200000u + (k / unit) * unit +
                                  (unit - 1u - k % unit)] = b;
                    }
                    r.reg(0x1C9Cu, 1u << 4);
                    r.reg(0x1CB0u, (fmt << 16) | (bilinear << 1) |
                                       (bilinear << 4));
                    r.reg(0x1CB8u, (3u << 8) | (3u << 4) | 3u); // 8x8
                    r.reg(0x1CC8u, agp ? 0x02000000u + 0x00300000u
                                       : 0x80000u); // slot 3
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
                    vtxSt(64, 0, 1, 0);
                    vtxSt(0, 64, 0, 1);
                    vtxSt(64, 0, 1, 0);
                    vtxSt(64, 64, 1, 1);
                    vtxSt(0, 64, 0, 1);
                    r.prim(0x9u | 0x80u, 4u, 6u, v);
                    for (u32 y = 0; y < 64u; ++y)
                        for (u32 x = 0; x < 64u; ++x) {
                            const u32 p = r.pix32(x, y);
                            hash[agp] = hash[agp] * 16777619u ^ p;
                            lit[agp] += (p & 0x00FFFFFFu) != 0u;
                        }
                }
                CAPTURE(fmt);
                CAPTURE(bilinear);
                CAPTURE(byBytes);
                CHECK(hash[0] == hash[1]);
                // And it was a picture: most of the quad is not black.
                CHECK(lit[0] > 2000u);
                ++cases;
            }
    }
    CHECK(cases == 40u);
}

TEST_CASE("3d: a 1555 texel in AGP space, by the bytes a driver stores")
{
    // Four texels of a light map as a driver laid them down in system
    // memory: 21 04, c4 18, a3 14, 82 10. Low byte first they are 0x0421,
    // 0x18C4, 0x14A3 and 0x1082: red, green and blue levels of 31 of
    // (1,1,1), (6,6,4), (5,5,3) and (4,4,2), a dim warm ramp. High byte
    // first the same bytes are four unrelated colors.
    Rig3d r;
    r.dst32();
    r.openGate();
    r.fpuSolidBoth();
    r.mapPage(0x00300000u, 0x00200000u);
    const u8 tex[8] = {0x21, 0x04, 0xC4, 0x18, 0xA3, 0x14, 0x82, 0x10};
    for (u32 i = 0; i < 8u; ++i)
        r.ram[0x00200000u + i] = tex[i];
    r.reg(0x1C9Cu, 1u << 4);
    r.reg(0x1CB0u, (3u << 16) | (2u << 8) | (2u << 11)); // 1555, point
    r.reg(0x1CB8u, (0u << 8) | (2u << 4) | 2u); // 4 wide, 1 high
    r.reg(0x1CC4u, 0x02000000u + 0x00300000u);  // slot 2
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
    vtxSt(64, 0, 1, 0);
    vtxSt(0, 64, 0, 1);
    vtxSt(64, 0, 1, 0);
    vtxSt(64, 64, 1, 1);
    vtxSt(0, 64, 0, 1);
    r.prim(0x9u | 0x80u, 4u, 6u, v);
    const auto rgb = [](u32 red, u32 green, u32 blue) {
        return ((red * 255u / 31u) << 16) | ((green * 255u / 31u) << 8) |
               (blue * 255u / 31u);
    };
    CHECK((r.pix32(8, 8) & 0x00FFFFFFu) == rgb(1, 1, 1));
    CHECK((r.pix32(24, 8) & 0x00FFFFFFu) == rgb(6, 6, 4));
    CHECK((r.pix32(40, 8) & 0x00FFFFFFu) == rgb(5, 5, 3));
    CHECK((r.pix32(56, 8) & 0x00FFFFFFu) == rgb(4, 4, 2));
}

namespace {
struct AskEveryTime final : SnoopSink { // never says memory is current
    u64 asked = 0;
    void snoopRead(u32, u32) override { ++asked; }
    void snoopWrite(u32, u32) override {}
};
} // namespace

TEST_CASE("3d: texels read in place are the texels read byte by byte")
{
    // The same textured quad drawn twice: once with nobody to ask before
    // system memory is read (texels come from a pointer, four at a time for
    // a bilinear sample), once with a snoop sink (every texel byte by byte,
    // the path that was the only one). Every format the pointer path knows,
    // each addressing rule, both filters, a texture in video memory and one
    // in AGP space lying across two pages: video memory must end the same,
    // and so must the counts.
    u32 cases = 0;
    // Two cards for all the cases: each case programs everything it draws
    // with and wipes the 64x64 target first.
    AskEveryTime sink;
    Rig3d inPlace, bytes;
    bytes.snoop = &sink;
    for (u32 fmt : {3u, 4u, 6u, 15u})
        for (u32 wrap = 0; wrap < 4; ++wrap)
            for (u32 bilinear = 0; bilinear < 2; ++bilinear)
                for (u32 agp = 0; agp < 2; ++agp) {
                    sink.asked = 0;
                    u64 samples[2] = {};
                    int which = 0;
                    for (Rig3d* r : {&inPlace, &bytes}) {
                        std::memset(r->c.vram.data(), 0, 64u * 64u * 4u);
                        r->dst32();
                        r->openGate();
                        r->fpuSolidBoth();
                        // An 8x8 texture of every byte value in turn, at
                        // video memory 0x80000, or at AGP offset 0x300F80:
                        // the last 128 bytes of one page and on into the
                        // next, which are far apart in memory.
                        const u32 n = fmt == 6u ? 4u : 2u;
                        r->mapPage(0x00300000u, 0x00200000u);
                        r->mapPage(0x00301000u, 0x00500000u);
                        for (u32 k = 0; k < 64u * n; ++k) {
                            const u8 b = static_cast<u8>(k * 37u + fmt * 11u);
                            if (!agp)
                                r->c.vram[0x80000u + k] = b;
                            else if (0xF80u + k < 0x1000u)
                                r->ram[0x00200F80u + k] = b;
                            else
                                r->ram[0x00500000u + (0xF80u + k - 0x1000u)] = b;
                        }
                        r->reg(0x1C9Cu, 1u << 4);
                        r->reg(0x1CB0u, (fmt << 16) | (bilinear << 1) |
                                            (bilinear << 4) | (wrap << 8) |
                                            (wrap << 11));
                        r->reg(0x1CB8u, (3u << 8) | (3u << 4) | 3u); // 8x8
                        r->reg(0x1CC8u, agp ? 0x02000000u + 0x00300F80u
                                            : 0x80000u); // slot 3
                        r->reg(0x1D38u, 0x80402010u); // the border color
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
                        // S and T run from -0.7 to 1.9 across the quad: off
                        // both ends of the map, so the rule decides.
                        vtxSt(0, 0, -0.7f, -0.7f);
                        vtxSt(64, 0, 1.9f, -0.7f);
                        vtxSt(0, 64, -0.7f, 1.9f);
                        vtxSt(64, 0, 1.9f, -0.7f);
                        vtxSt(64, 64, 1.9f, 1.9f);
                        vtxSt(0, 64, -0.7f, 1.9f);
                        const u64 before = r128EngStats().texSamples;
                        r->prim(0x9u | 0x80u, 4u, 6u, v);
                        samples[which++] = r128EngStats().texSamples - before;
                    }
                    CAPTURE(fmt);
                    CAPTURE(wrap);
                    CAPTURE(bilinear);
                    CAPTURE(agp);
                    CHECK(std::memcmp(inPlace.c.vram.data(), bytes.c.vram.data(),
                                      64u * 64u * 4u) == 0);
                    CHECK(samples[0] == samples[1]);
                    // Sampled all over (the border rule fetches nothing
                    // beyond the map's edge, so not every pixel).
                    CHECK(samples[0] > 500u);
                    // The byte path did ask (it is the other path), and only
                    // where there is system memory to ask about.
                    CHECK((sink.asked > 0) == (agp != 0));
                    // Not one flat color: the texture was really sampled.
                    CHECK(inPlace.pix32(5, 5) != inPlace.pix32(40, 23));
                    ++cases;
                }
    CHECK(cases == 64u);
}

namespace {
// What a draw changed in the engine's counts.
struct DrawCounts {
    u64 pixels, samples, offVram, unimpl, crewTris, crewPixels;
    static DrawCounts now()
    {
        const R128EngStats& e = r128EngStats();
        return {e.triPixels, e.texSamples, e.offVram, e.texUnimpl, e.crewTris,
                e.crewPixels};
    }
    DrawCounts since(const DrawCounts& b) const
    {
        return {pixels - b.pixels,     samples - b.samples,
                offVram - b.offVram,   unimpl - b.unimpl,
                crewTris - b.crewTris, crewPixels - b.crewPixels};
    }
};
} // namespace

TEST_CASE("3d: triangles waiting in the queue land in the order they were "
          "sent, whatever comes between them")
{
    // One long stream, executed in one go as a driver's indirect buffers are:
    // some 3,500 triangles, large and small, each packet under its own state
    // (depth, blending, fog by table, a paletted texture in video memory, a
    // 1555 one in AGP space), and between them everything a stream can do
    // that the triangles before it must be drawn for: a 2D fill, a fog table
    // and a palette loaded again, a line, a write to a scratch register, the
    // destination's pitch changed and changed back; and in the middle 1,600
    // triangles with nothing between them, more than the queue has places
    // for, sent faster than they can be drawn. Drawn on one thread it is
    // the picture; on four and on nine it must be that picture byte for byte,
    // depth cells included, with the same counts. Every pixel here depends on
    // what was drawn before it and on the state of its own packet, so a
    // triangle out of its place, drawn twice, dropped, or drawn under a
    // neighbor's state is a different picture.
    constexpr u32 kW = 640, kH = 256;
    constexpr u32 kZAt = 0x00100000u, kCi8At = 0x00180000u;
    constexpr u32 kCompared = 0x00200000u;
    constexpr u32 kChunkWords = 0x3F00u;
    u32 seed = 0x1B873593u;
    auto rnd = [&]() {
        seed = seed * 1664525u + 1013904223u;
        return seed >> 8;
    };
    auto frac = [&]() { return static_cast<float>(rnd() & 0xFFFFu) / 65535.0f; };
    // The stream, in pieces no longer than an indirect buffer may be.
    std::vector<std::vector<u32>> chunks(1);
    auto room = [&](size_t words) {
        if (chunks.back().size() + words > kChunkWords)
            chunks.emplace_back();
    };
    auto reg = [&](u32 off, u32 v) {
        room(2);
        chunks.back().push_back(off >> 2);
        chunks.back().push_back(v);
    };
    u32 triangles = 0, forced = 0;
    auto tris = [&](u32 count, float least, float span) {
        room(3u + count * 21u);
        auto& w = chunks.back();
        w.push_back(0xC0002500u | ((2u + count * 21u - 1u) << 16));
        w.push_back(0x9u | 0x80u);
        w.push_back(4u | (3u << 4) | ((count * 3u) << 16));
        for (u32 t = 0; t < count; ++t) {
            const float cx = -20.0f + 680.0f * frac();
            const float cy = -20.0f + 296.0f * frac();
            const float reach = least + span * frac();
            for (u32 k = 0; k < 3u; ++k) {
                w.push_back(f2u(cx + reach * (frac() - 0.5f)));
                w.push_back(f2u(cy + reach * (frac() - 0.5f)));
                w.push_back(f2u(frac()));
                w.push_back(f2u(1.0f));
                w.push_back(rnd() << 8 | (rnd() & 0xFFu));
                w.push_back(f2u(-0.5f + 3.0f * frac()));
                w.push_back(f2u(-0.5f + 3.0f * frac()));
            }
            ++triangles;
        }
    };
    auto fogTable = [&](u32 salt) {
        reg(0x1A14u, 0);
        room(257);
        chunks.back().push_back((0x1A18u >> 2) | (1u << 15) | (255u << 16));
        for (u32 i = 0; i < 256u; ++i)
            chunks.back().push_back((i * 3u + salt) & 0xFFu);
        ++forced;
    };
    auto palette = [&](u32 salt) {
        room(258);
        chunks.back().push_back(0xC0002C00u | (256u << 16));
        chunks.back().push_back(2u);
        for (u32 i = 0; i < 256u; ++i)
            chunks.back().push_back(0xFF000000u | ((i * 5u + salt) & 0xFFu) << 16 |
                                    ((i * 11u + salt * 3u) & 0xFFu) << 8 |
                                    ((255u - i + salt) & 0xFFu));
        ++forced;
    };
    auto target = [&](u32 pitch8) {
        reg(0x1C80u, pitch8 << 21);
        reg(0x1C88u, 0);
        reg(0x1C8Cu, ((kH - 1u) << 16) | (pitch8 * 8u - 1u));
    };
    fogTable(7u);
    palette(1u);
    for (u32 n = 0; n < 2000u; ++n) {
        const bool run = n >= 700u && n < 1500u; // nothing between these
        // This packet's state: all of it plain 3D state, none of it a reason
        // to wait for the triangles before.
        const u32 pick = rnd();
        const bool ci8 = (pick >> 3) & 1u;
        reg(0x1C9Cu, (pick & 1u ? 0x3u : 0u) | (pick & 2u ? 1u << 4 : 0u) |
                         (pick & 4u ? 1u << 7 : 0u) |
                         ((pick >> 4) & 3u ? 1u << 9 : 0u));
        reg(0x1CA0u, (2u << 8) | (1u << 14) | (4u << 16) | (5u << 20));
        reg(0x1CB0u, ci8 ? (2u << 16) : (3u << 16) | (1u << 1) | (1u << 4));
        reg(0x1CB8u, ci8 ? 0x6666u : 0x5555u); // one map each, no smaller
        reg(0x1CACu, rnd() & 0x00FFFFFFu);
        if (run)
            tris(2u, 30.0f, 100.0f);
        else if (n == 0u || ((pick >> 8) & 7u) < 2u)
            tris(1u + (pick >> 6) % 2u, 60.0f, 300.0f);
        else
            tris(1u + (pick >> 6) % 2u, 3.0f, 22.0f);
        if (run)
            continue;
        // Between the triangles, now and then.
        if (n % 200u == 37u) { // a 2D fill across the middle of the picture
            room(8);
            auto& w = chunks.back();
            w.push_back(0xC0009100u | (6u << 16));
            w.push_back(2u | 8u | (13u << 4) | (6u << 8) | (0xF0u << 16));
            w.push_back((kW / 8u) << 21);
            w.push_back(0);
            w.push_back(((kH - 1u) << 16) | (kW - 1u));
            w.push_back(0x00102030u + n);
            w.push_back((60u + n / 10u) << 16 | 100u);
            w.push_back((140u + n / 10u) << 16 | 540u);
            ++forced;
        }
        if (n % 200u == 83u)
            fogTable(n);
        if (n % 200u == 121u)
            palette(n);
        if (n % 200u == 150u) { // a line from corner to corner
            room(13);
            auto& w = chunks.back();
            w.push_back(0xC0002500u | ((2u + 10u - 1u) << 16));
            w.push_back(0x9u);
            w.push_back(2u | (3u << 4) | (2u << 16));
            for (u32 k = 0; k < 2u; ++k) {
                w.push_back(f2u(k ? 630.0f : 5.0f));
                w.push_back(f2u(k ? 250.0f : 3.0f));
                w.push_back(f2u(0.25f));
                w.push_back(f2u(1.0f));
                w.push_back(0xFFFFFFFFu - n);
            }
            ++forced;
        }
        if (n % 200u == 170u) {
            reg(0x15E0u, n); // the marker a driver polls for
            ++forced;
        }
        // The same memory as a narrower picture for a while: its rows are
        // not the rows the waiting triangles were dealt by.
        if (n % 400u == 190u) {
            target(40u);
            ++forced;
        }
        if (n % 400u == 215u) {
            target(kW / 8u);
            ++forced;
        }
    }
    CHECK(triangles > 3000u);
    CHECK(chunks.size() >= 5u);
    struct Run {
        std::unique_ptr<Rig3d> rig;
        DrawCounts got;
        u64 settles = 0, refused = 0;
    };
    Run runs[3];
    const int threadsOf[3] = {1, 4, 9};
    for (int k = 0; k < 3; ++k) {
        runs[k].rig = std::make_unique<Rig3d>();
        Rig3d& r = *runs[k].rig;
        r128SetRasterThreads(threadsOf[k]);
        r128CceReset();
        u32 fill = 0x85EBCA6Bu;
        for (u32 i = 0; i < kCompared; ++i) {
            fill = fill * 1664525u + 1013904223u;
            r.c.vram[i] = static_cast<u8>(fill >> 24);
        }
        // The 1555 texture in AGP space, and the stream after it: a master
        // buffer that dispatches each piece in turn.
        r.mapPage(0x00300000u, 0x00200000u);
        for (u32 i = 0; i < 0x1000u; ++i)
            r.ram[0x00200000u + i] = static_cast<u8>(i * 13u + 5u);
        std::vector<u32> master;
        for (size_t c = 0; c < chunks.size(); ++c) {
            const u32 at = 0x00600000u + static_cast<u32>(c) * 0x10000u;
            for (u32 pg = 0; pg < 0x10000u; pg += 0x1000u)
                r.mapPage(at + pg, at + pg);
            for (size_t i = 0; i < chunks[c].size(); ++i)
                r.put32(at + static_cast<u32>(i) * 4u, chunks[c][i]);
            master.push_back(0x000101CEu); // PM4_IW_INDOFF, PM4_IW_INDSIZE
            master.push_back(at);
            master.push_back(static_cast<u32>(chunks[c].size()));
        }
        r.mapPage(0x00700000u, 0x00700000u);
        for (size_t i = 0; i < master.size(); ++i)
            r.put32(0x00700000u + static_cast<u32>(i) * 4u, master[i]);
        // A 640-wide 32-bit target at 0, depth at 1 MB, both textures' slots.
        r.reg(0x146Cu, (1u << 30) | (1u << 28) | 0x0Fu | (6u << 8));
        r.reg(0x142Cu, (kW / 8u) << 21);
        r.reg(0x16ECu, 0);
        r.reg(0x16F0u, ((kH - 1u) << 16) | (kW - 1u));
        r.reg(0x1CA0u, 2u << 8);
        r.fpuSolidBoth();
        r.reg(0x1C90u, kZAt);
        r.reg(0x1C94u, kW / 8u);
        r.reg(0x1C98u, 1u << 4);
        r.reg(0x1CB4u, 3u | (4u << 4) | (4u << 10) | (3u << 14) | (6u << 18) |
                           (2u << 25));
        r.reg(0x1CBCu + 4u * 6u, kCi8At);
        r.reg(0x1CBCu + 4u * 5u, 0x02000000u + 0x00300000u);
        r.reg(0x0738u, 0x00700000u);
        const DrawCounts before = DrawCounts::now();
        const u64 settles = r128EngStats().rowSettles;
        const u64 refused = r128EngStats().rowRefused;
        r128CceIndirect(r.c, static_cast<u32>(master.size()), r.ram.data(),
                        static_cast<u32>(r.ram.size()), Rig3d::kGart, 0,
                        nullptr);
        runs[k].got = DrawCounts::now().since(before);
        runs[k].settles = r128EngStats().rowSettles - settles;
        runs[k].refused = r128EngStats().rowRefused - refused;
    }
    r128SetRasterThreads(0);
    for (int k = 1; k < 3; ++k) {
        CAPTURE(threadsOf[k]);
        CHECK(std::memcmp(runs[0].rig->c.vram.data(),
                          runs[k].rig->c.vram.data(), kCompared) == 0);
        CHECK(runs[0].got.pixels == runs[k].got.pixels);
        CHECK(runs[0].got.samples == runs[k].got.samples);
        // Most triangles waited in the queue (not those of the narrow
        // picture that reach past its pitch), and the queue was
        // drawn out where something made it, not at every packet: the
        // stream holds 2,000 packets of state and some forty reasons.
        CHECK(runs[k].got.crewTris * 10u > triangles * 7u);
        CHECK(runs[k].settles >= forced / 2u);
        CHECK(runs[k].settles <= forced + 8u);
    }
    // One thread queues nothing; and the picture is a picture.
    CHECK(runs[0].got.crewTris == 0u);
    CHECK(runs[0].settles == 0u);
    CHECK(runs[0].got.pixels > 500000u);
}

TEST_CASE("3d: a triangle's rows drawn by several threads are the rows drawn "
          "by one")
{
    // The same scene of large overlapping triangles on two cards, one drawing
    // every row on the calling thread and one queueing the rows of a large
    // triangle to four. Video memory must end the same byte for byte, depth
    // and stencil cells included, and so must every count the pixel path
    // keeps. The scene is drawn with each kind of state a pixel can depend
    // on; the last cases are the ones where rows are NOT independent, and
    // there the four threads must not have been used at all.
    constexpr u32 kW = 512, kH = 256;
    constexpr u32 kZAt = 0x00100000u, kTex1 = 0x00200000u;
    constexpr u32 kAll = 0x00400000u; // everything a case can touch, compared
    struct Case {
        const char* what;
        u32 texCntl;  // TEX_CNTL_C
        u32 misc;     // MISC_3D_STATE_CNTL above the gate's field
        u32 zSten;    // Z_STEN_CNTL_C
        u32 prim, sec; // the two units' control registers
        u32 zOff;
        u32 tex1At;
        bool dst16;
        bool crew;    // may the rows go to several threads?
    };
    const u32 kTri = 5u << 1, kBil = (1u << 1) | (1u << 4);
    const u32 k1555 = 3u << 16, k8888 = 6u << 16;
    const u32 kBlend = (4u << 16) | (5u << 20);
    const Case cases[] = {
        {"shaded only", 0, 0, 0, 0, 0, kZAt, kTex1, false, true},
        {"depth, 16 bits", 0x3u, 0, 1u << 4, 0, 0, kZAt, kTex1, false, true},
        {"blended", 1u << 9, kBlend, 0, 0, 0, kZAt, kTex1, false, true},
        {"one texture with smaller maps, stencil, blended",
         0x3u | (1u << 3) | (1u << 4) | (1u << 9), kBlend,
         (1u << 1) | (1u << 4) | (7u << 12) | (3u << 20), k1555 | kTri | (1u << 4),
         0, kZAt, kTex1, false, true},
        {"two textures, specular, fog, alpha test",
         0x3u | (1u << 4) | (1u << 5) | (1u << 7) | (1u << 10) | (1u << 11),
         (4u << 24) | 0x20u, 1u << 4, k1555 | kTri | (1u << 4),
         k8888 | kBil | 1u | (2u << 8) | (2u << 11), kZAt, kTex1, false, true},
        {"two textures, a level bias, nearest map",
         0x3u | (1u << 4) | (1u << 5) | (0x18u << 24), 0, 1u << 4,
         k1555 | (2u << 1), k8888 | 1u, kZAt, kTex1, false, true},
        {"depth cells partly beyond video memory", 0x3u, 0, 1u << 4, 0, 0,
         (32u << 20) - 0x18000u, kTex1, false, true},
        // Rows that are not independent.
        {"depth buffer lying in the destination", 0x3u, 0, 1u << 4, 0, 0,
         kW * 4u * 64u, kTex1, false, false},
        {"a texture lying in the destination", 0x3u | (1u << 4), 0, 1u << 4,
         k1555 | kBil, 0, kZAt, kW * 4u * 100u, false, false},
        {"error diffusion", 1u << 8, 0, 0, 0, 0, kZAt, kTex1, true, false},
    };
    // The scene: twelve triangles, corners anywhere in and a little around
    // the target, every attribute its own number.
    u32 seed = 0x2545F491u;
    auto rnd = [&]() {
        seed = seed * 1664525u + 1013904223u;
        return seed >> 8;
    };
    auto frac = [&]() { return static_cast<float>(rnd() & 0xFFFFu) / 65535.0f; };
    std::vector<u32> scene;
    for (u32 k = 0; k < 12u * 3u; ++k) {
        const float rhw = 0.3f + 1.7f * frac();
        scene.push_back(f2u(-40.0f + 600.0f * frac())); // x
        scene.push_back(f2u(-30.0f + 320.0f * frac())); // y
        scene.push_back(f2u(frac()));                   // z
        scene.push_back(f2u(rhw));
        scene.push_back(rnd() << 8 | (rnd() & 0xFFu)); // diffuse
        scene.push_back(rnd() << 8 | (rnd() & 0xFFu)); // fog and specular
        scene.push_back(f2u((-1.0f + 4.0f * frac()) * rhw)); // s, t: times
        scene.push_back(f2u((-1.0f + 4.0f * frac()) * rhw)); // rhw, as sent
        scene.push_back(f2u((-0.5f + 2.0f * frac()) * rhw));
        scene.push_back(f2u((-0.5f + 2.0f * frac()) * rhw));
    }
    Rig3d one, many;
    u32 ran = 0;
    for (const Case& k : cases) {
        DrawCounts got[2] = {};
        int which = 0;
        for (Rig3d* r : {&one, &many}) {
            r128SetRasterThreads(r == &one ? 1 : 4);
            // The engine's own state is one for the process (the dither's
            // error register among it): each card starts from none.
            r128CceReset();
            // Memory as a case finds it: every byte its own, so that a cell
            // read in the wrong order is a different number.
            u32 fill = 0x9E3779B9u;
            for (u32 i = 0; i < kAll; ++i) {
                fill = fill * 1664525u + 1013904223u;
                r->c.vram[i] = static_cast<u8>(fill >> 24);
            }
            for (u32 i = (32u << 20) - 0x20000u; i < (32u << 20); ++i)
                r->c.vram[i] = static_cast<u8>(i * 7u);
            r->mapPage(0x00300000u, 0x00200000u);
            r->mapPage(0x00301000u, 0x00500000u);
            for (u32 i = 0; i < 0x1000u; ++i) {
                r->ram[0x00200000u + i] = static_cast<u8>(i * 13u + 5u);
                r->ram[0x00500000u + i] = static_cast<u8>(i * 29u + 3u);
            }
            // A 512-wide target at 0, 32 bits or 16.
            r->reg(0x146Cu, (1u << 30) | (1u << 28) | 0x0Fu |
                                ((k.dst16 ? 4u : 6u) << 8));
            r->reg(0x142Cu, (kW / 8u) << 21);
            r->reg(0x16ECu, 0);
            r->reg(0x16F0u, ((kH - 1u) << 16) | (kW - 1u));
            r->reg(0x1CA0u, (2u << 8) | k.misc);
            r->fpuSolidBoth();
            r->reg(0x1C90u, k.zOff);
            r->reg(0x1C94u, kW / 8u);
            r->reg(0x1C98u, k.zSten);
            r->reg(0x1D40u, 0xFFu << 16 | 0xFFu << 24);
            r->reg(0x1C9Cu, k.texCntl);
            r->reg(0x1CACu, 0x00406080u); // the fog color
            // Unit 1: 64x64 with every smaller map, in video memory. Unit 2:
            // 32x32, one map, in AGP space across two pages.
            r->reg(0x1CB0u, k.prim);
            r->reg(0x1CB4u, 0);
            r->reg(0x1D00u, k.sec);
            r->reg(0x1D04u, 1u);
            r->reg(0x1CB8u, 0x0666u | (0x5555u << 16));
            for (u32 l = 0; l <= 6u; ++l)
                r->reg(0x1CBCu + 4u * (6u - l), k.tex1At + l * 0x4000u);
            r->reg(0x1D08u + 4u * 5u, 0x02000000u + 0x00300F80u);
            const DrawCounts before = DrawCounts::now();
            r->prim(0x1u | 0x8u | 0x40u | 0x80u | 0x100u, 4u, 36u, scene);
            got[which++] = DrawCounts::now().since(before);
        }
        const std::string what = k.what;
        CAPTURE(what);
        CHECK(std::memcmp(one.c.vram.data(), many.c.vram.data(), kAll) == 0);
        CHECK(std::memcmp(one.c.vram.data() + (32u << 20) - 0x20000u,
                          many.c.vram.data() + (32u << 20) - 0x20000u,
                          0x20000u) == 0);
        CHECK(got[0].pixels == got[1].pixels);
        CHECK(got[0].samples == got[1].samples);
        CHECK(got[0].offVram == got[1].offVram);
        CHECK(got[0].unimpl == got[1].unimpl);
        // The scene was drawn, and by whom: one thread never hands a row
        // over, four take most of the pixels wherever rows are independent
        // and next to none of them where they are not.
        const bool cellsBeyond = k.zOff > (16u << 20);
        CHECK(got[0].pixels > (cellsBeyond ? 5000u : 20000u));
        CHECK((got[0].offVram > 0u) == cellsBeyond);
        CHECK(got[0].crewTris == 0u);
        if (k.crew) {
            CHECK(got[1].crewTris >= 6u);
            CHECK(got[1].crewPixels * 2u > got[1].pixels);
        } else if (k.dst16) {
            CHECK(got[1].crewTris == 0u); // no row of a dithered triangle
        } else {
            // Only a triangle that lies wholly clear of the other buffer:
            // one small one in this scene, twelve if nothing were checked.
            CHECK(got[1].crewTris <= 2u);
            CHECK(got[1].crewPixels * 10u < got[1].pixels);
        }
        ++ran;
    }
    r128SetRasterThreads(0);
    CHECK(ran == 10u);
}

TEST_CASE("3d: a texture taller than wide has its width in the pitch field")
{
    // TEX_SIZE_PITCH_C for a 2-wide, 4-tall texture as a driver writes it:
    // pitch 2^1, size 2^2 (the larger dimension: the offset slot and the
    // number of maps), height 2^2. Eight texels, each its own color, drawn
    // one texel to a 4x4 block of pixels; reading the size as the width
    // would take every second column from the next row.
    Rig3d r;
    r.dst32();
    r.openGate();
    r.fpuSolidBoth();
    const u32 texel[8] = {0xF800u, 0x07E0u,  // row 0: red, green
                          0x001Fu, 0xFFFFu,  // row 1: blue, white
                          0xFFE0u, 0xF81Fu,  // row 2: yellow, magenta
                          0x07FFu, 0x0000u}; // row 3: cyan, black
    for (u32 i = 0; i < 8; ++i) {
        r.c.vram[0x80000u + i * 2u] = static_cast<u8>(texel[i] >> 8);
        r.c.vram[0x80000u + i * 2u + 1u] = static_cast<u8>(texel[i]);
    }
    r.reg(0x1C9Cu, 1u << 4);
    r.reg(0x1CB0u, (4u << 16) | (1u << 7)); // 565, nearest, one map
    r.reg(0x1CB8u, 0x0221u);                // pitch 2, size 4, height 4
    r.reg(0x1CC4u, 0x80000u);               // slot 2: the size's
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
    vtxSt(0, 0, 0, 0); // 8x16 pixels over the whole map
    vtxSt(8, 0, 1, 0);
    vtxSt(0, 16, 0, 1);
    vtxSt(8, 0, 1, 0);
    vtxSt(8, 16, 1, 1);
    vtxSt(0, 16, 0, 1);
    r.prim(0x9u | 0x80u, 4u, 6u, v);
    const u32 want[8] = {0xFFFF0000u, 0xFF00FF00u, 0xFF0000FFu, 0xFFFFFFFFu,
                         0xFFFFFF00u, 0xFFFF00FFu, 0xFF00FFFFu, 0xFF000000u};
    for (u32 ty = 0; ty < 4; ++ty)
        for (u32 tx = 0; tx < 2; ++tx) {
            CAPTURE(tx);
            CAPTURE(ty);
            CHECK(r.pix32(tx * 4u + 1u, ty * 4u + 1u) == want[ty * 2u + tx]);
            CHECK(r.pix32(tx * 4u + 2u, ty * 4u + 2u) == want[ty * 2u + tx]);
        }
}
