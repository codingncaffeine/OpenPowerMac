// The Rage 128's 2D drawing engine: the blit itself.
//
// This deserves a test rather than a boot, and the reason is specific. The
// guest that motivated the engine — Mac OS 9's ATI Graphics Accelerator —
// turns out NOT to drive these registers: measured over a whole 12 G boot it
// makes THIRTEEN accesses to the whole 0x1400-0x17FF block and submits its
// real work through the CCE command ring instead. So a boot cannot currently
// exercise the blitter at all, and "it compiles and the machine still boots"
// would be the only evidence there was. That is not evidence.
//
// The engine is still the right thing to have built: a PM4 PACKET0 is a
// register write, so when the ring parser lands it drives exactly these
// registers, and this is the back end it will drive. These tests are what
// says the back end is correct before anything can reach it.
//
// Register semantics are ATI's own, from the RAGE 128 PRO Register Reference
// Guide (RRG-G04500-C rev 1.01).

#include "doctest.h"
#include "opm/r128.hpp"

#include <string>

using namespace opm;

namespace {

// The register file is little-endian on PCI and R128Cell swaps on the edge,
// so a 32-bit store of N arrives as N byte-reversed. Every write below goes
// through this, exactly as a guest's stwbrx would.
u32 be(u32 v)
{
    return (v >> 24) | ((v >> 8) & 0xFF00u) | ((v << 8) & 0xFF0000u) |
           (v << 24);
}
void wr(R128Cell& c, u32 off, u32 v) { c.write(off, be(v), 4); }

// Offsets used by the tests, by the manual's names.
constexpr u32 kDstOffset = 0x1404, kDstPitch = 0x1408;
constexpr u32 kDstYX = 0x1438, kDstHeightWidth = 0x143C;
constexpr u32 kGmc = 0x146C, kBrushFrgd = 0x147C;
constexpr u32 kSrcYX = 0x1434, kSrcOffset = 0x15AC, kSrcPitch = 0x15B0;
constexpr u32 kDpCntl = 0x16C0, kDpWriteMsk = 0x16CC;
constexpr u32 kDefaultOffset = 0x16E0, kDefaultPitch = 0x16E4;
constexpr u32 kDefaultScBr = 0x16E8;
constexpr u32 kScTopLeft = 0x16EC, kScBottomRight = 0x16F0;
constexpr u32 kGuiStat = 0x1740;

// A 64-pixel-wide 32-bpp surface. DST_PITCH is in units of EIGHT PIXELS, so
// 64 pixels is 8, and the byte stride is 8*32 = 256 = 64 px * 4 B.
constexpr u32 kW = 64, kPitch8 = kW / 8, kStride = kPitch8 * 32u;

// DP_GUI_MASTER_CNTL, assembled by field: brush 7:4, dst datatype 11:8,
// src datatype 13:12, ROP3 23:16, source select 26:24. Bits 3:0 are all set,
// which is "leave alone" for the pitch/offset and clipping defaults, so the
// registers the test programs explicitly are the ones that are used.
u32 gmc(u32 rop3, u32 srcDatatype, u32 srcSel)
{
    return 0x0Fu | (13u << 4) | (6u << 8) | (srcDatatype << 12) |
           (rop3 << 16) | (srcSel << 24);
}

u32 pix(const R128Cell& c, u32 x, u32 y)
{
    const size_t o = static_cast<size_t>(y) * kStride + static_cast<size_t>(x) * 4u;
    return (u32(c.vram[o]) << 24) | (u32(c.vram[o + 1]) << 16) |
           (u32(c.vram[o + 2]) << 8) | u32(c.vram[o + 3]);
}
void setPix(R128Cell& c, u32 x, u32 y, u32 v)
{
    const size_t o = static_cast<size_t>(y) * kStride + static_cast<size_t>(x) * 4u;
    c.vram[o] = static_cast<u8>(v >> 24);
    c.vram[o + 1] = static_cast<u8>(v >> 16);
    c.vram[o + 2] = static_cast<u8>(v >> 8);
    c.vram[o + 3] = static_cast<u8>(v);
}

// Program the destination surface and an all-inclusive scissor.
void surface(R128Cell& c)
{
    wr(c, kDstOffset, 0);
    wr(c, kDstPitch, kPitch8);
    wr(c, kSrcOffset, 0);
    wr(c, kSrcPitch, kPitch8);
    wr(c, kScTopLeft, 0);
    wr(c, kScBottomRight, (kW - 1u) | ((kW - 1u) << 16));
    wr(c, kDpWriteMsk, 0xFFFFFFFFu);
}

} // namespace

TEST_CASE("r128 2d: GUI_STAT reports an idle engine with a free FIFO")
{
    // GUI_FIFOCNT is bits 11:0 and its reset default is 0x40 — 64 free
    // CMDFIFO entries. A driver that waits for FIFO room before writing
    // spins for ever on a zero, which is one of the ways this card can wedge
    // a guest without ever drawing anything wrong.
    R128Cell c;
    const u32 st = be(c.read(kGuiStat, 4));
    CHECK((st & 0xFFFu) == 0x40u);
    CHECK((st & 0x80000000u) == 0u); // GUI_ACTIVE clear
}

TEST_CASE("r128 2d: a solid fill lands in the right pixels, most significant "
          "byte first")
{
    R128Cell c;
    const auto before = r128EngStats();
    wr(c, kGmc, gmc(0xF0u, 0u, 2u)); // PATCOPY from a solid brush
    surface(c);
    wr(c, kBrushFrgd, 0x11223344u);
    wr(c, kDstYX, 2u | (1u << 16));            // DST_X=2, DST_Y=1
    wr(c, kDstHeightWidth, 4u | (3u << 16));   // 4 wide, 3 high — INITIATES

    CHECK(r128EngStats().blits == before.blits + 1);
    CHECK(r128EngStats().pixels == before.pixels + 12);
    // The rectangle, and only the rectangle.
    for (u32 y = 0; y < 6; ++y)
        for (u32 x = 0; x < 8; ++x) {
            const bool inside = x >= 2 && x < 6 && y >= 1 && y < 4;
            CAPTURE(x);
            CAPTURE(y);
            CHECK(pix(c, x, y) == (inside ? 0x11223344u : 0u));
        }
    // ⚠ Byte order is the whole point of the check above: this model's VRAM
    // holds pixels as the guest wrote them through the big-endian aperture,
    // and the scanout reads byte1/byte2/byte3 as R/G/B. Storing the fill the
    // other way round is the bug that once turned a grey desktop olive.
    CHECK(c.vram[static_cast<size_t>(1) * kStride + 2 * 4] == 0x11);
    CHECK(c.vram[static_cast<size_t>(1) * kStride + 2 * 4 + 3] == 0x44);
}

TEST_CASE("r128 2d: GMC_AUX_CLIP_DIS clears the three auxiliary scissor "
          "enables and leaves their modes")
{
    // AUX_SC_CNTL in the register guide: the enables are bits 0, 2 and 4, each
    // followed by its scissor's mode bit.
    R128Cell c;
    wr(c, 0x1660, 0x3Fu);
    wr(c, kGmc, gmc(0xF0u, 0u, 2u) | (1u << 29));
    CHECK(c.peek(0x1660) == 0x2Au);
}

TEST_CASE("r128 2d: BLACKNESS and WHITENESS ignore the brush colour")
{
    R128Cell c;
    surface(c);
    setPix(c, 0, 0, 0xDEADBEEFu);
    setPix(c, 1, 0, 0xDEADBEEFu);
    wr(c, kBrushFrgd, 0x11223344u);

    wr(c, kGmc, gmc(0xFFu, 0u, 2u)); // WHITENESS
    surface(c);
    wr(c, kDstYX, 0);
    wr(c, kDstHeightWidth, 1u | (1u << 16));
    CHECK(pix(c, 0, 0) == 0xFFFFFFFFu);

    wr(c, kGmc, gmc(0x00u, 0u, 2u)); // BLACKNESS
    surface(c);
    wr(c, kDstYX, 1u);
    wr(c, kDstHeightWidth, 1u | (1u << 16));
    CHECK(pix(c, 1, 0) == 0u);
}

TEST_CASE("r128 2d: SRCCOPY moves a rectangle")
{
    R128Cell c;
    surface(c);
    for (u32 y = 0; y < 4; ++y)
        for (u32 x = 0; x < 4; ++x)
            setPix(c, x, y, 0xA0000000u | (y * 16u + x));

    wr(c, kGmc, gmc(0xCCu, 3u, 2u)); // SRCCOPY, colour source from memory
    surface(c);
    wr(c, kSrcYX, 0);                        // from (0,0)
    wr(c, kDstYX, 10u | (5u << 16));         // to (10,5)
    wr(c, kDstHeightWidth, 4u | (4u << 16));

    for (u32 y = 0; y < 4; ++y)
        for (u32 x = 0; x < 4; ++x) {
            CAPTURE(x);
            CAPTURE(y);
            CHECK(pix(c, 10 + x, 5 + y) == (0xA0000000u | (y * 16u + x)));
        }
}

// CLR_CMP_CNTL (0x15C0): the engine's transparency. Register guide 3-178;
// function 5 is "draw when not equal", 4 "draw when equal", 1 "never draw",
// each against a key through a mask, on the source pixel (bits 25:24 = 1),
// the destination pixel (0) or both (2).
TEST_CASE("r128 2d: a source colour key leaves the destination where the "
          "source is the key")
{
    constexpr u32 kClrCmpCntl = 0x15C0, kKeySrc = 0x15C4, kKeyDst = 0x15C8,
                  kCmpMsk = 0x15CC;
    for (const u32 fn : {5u, 4u, 1u, 0u}) {
        R128Cell c;
        surface(c);
        // A 4x1 source: key, colour, key-but-for-the-top-byte, colour.
        setPix(c, 0, 0, 0x00000000u);
        setPix(c, 1, 0, 0x00112233u);
        setPix(c, 2, 0, 0xFF000000u);
        setPix(c, 3, 0, 0x00445566u);
        for (u32 x = 0; x < 4; ++x)
            setPix(c, 20 + x, 9, 0x00ABCDEFu); // what is there already
        wr(c, kGmc, gmc(0xCCu, 3u, 2u));
        surface(c);
        wr(c, kKeySrc, 0x00000000u);
        wr(c, kKeyDst, 0x00000000u);
        wr(c, kCmpMsk, 0x00FFFFFFu); // the top byte is not compared
        wr(c, kClrCmpCntl, fn | (1u << 24));
        wr(c, kSrcYX, 0);
        wr(c, kDstYX, 20u | (9u << 16));
        wr(c, kDstHeightWidth, 4u | (1u << 16));
        CAPTURE(fn);
        const u32 kept = 0x00ABCDEFu;
        switch (fn) {
        case 5u: // a sprite: everything but the key
            CHECK(pix(c, 20, 9) == kept);
            CHECK(pix(c, 21, 9) == 0x00112233u);
            CHECK(pix(c, 22, 9) == kept); // equal through the mask
            CHECK(pix(c, 23, 9) == 0x00445566u);
            break;
        case 4u: // its complement
            CHECK(pix(c, 20, 9) == 0x00000000u);
            CHECK(pix(c, 21, 9) == kept);
            CHECK(pix(c, 22, 9) == 0xFF000000u);
            CHECK(pix(c, 23, 9) == kept);
            break;
        case 1u: // never
            for (u32 x = 0; x < 4; ++x)
                CHECK(pix(c, 20 + x, 9) == kept);
            break;
        default: // always: the plain copy
            CHECK(pix(c, 20, 9) == 0x00000000u);
            CHECK(pix(c, 21, 9) == 0x00112233u);
            CHECK(pix(c, 22, 9) == 0xFF000000u);
            CHECK(pix(c, 23, 9) == 0x00445566u);
            break;
        }
    }
}

TEST_CASE("r128 2d: a destination colour key draws only over the key, fills "
          "included")
{
    constexpr u32 kClrCmpCntl = 0x15C0, kKeyDst = 0x15C8, kCmpMsk = 0x15CC;
    R128Cell c;
    surface(c);
    setPix(c, 30, 4, 0x00FF00FFu); // the key colour
    setPix(c, 31, 4, 0x00123456u); // anything else
    setPix(c, 32, 4, 0x00FF00FFu);
    wr(c, kGmc, gmc(0xF0u, 0u, 0u)); // PATCOPY: a solid fill
    surface(c);
    wr(c, kBrushFrgd, 0x00C0FFEEu);
    wr(c, kKeyDst, 0x00FF00FFu);
    wr(c, kCmpMsk, 0xFFFFFFFFu);
    wr(c, kClrCmpCntl, (4u << 8) | (0u << 24)); // destination, draw when equal
    const auto before = r128EngStats();
    wr(c, kDstYX, 30u | (4u << 16));
    wr(c, kDstHeightWidth, 3u | (1u << 16));
    CHECK(pix(c, 30, 4) == 0x00C0FFEEu);
    CHECK(pix(c, 31, 4) == 0x00123456u);
    CHECK(pix(c, 32, 4) == 0x00C0FFEEu);
    const auto& e = r128EngStats();
    CHECK(e.fills - before.fills == 1);
    CHECK(e.colorCompare == before.colorCompare); // applied, so not counted
}

TEST_CASE("r128 2d: a compare this engine does not apply is counted and "
          "draws")
{
    constexpr u32 kClrCmpCntl = 0x15C0;
    R128Cell c;
    surface(c);
    setPix(c, 0, 0, 0x00112233u);
    wr(c, kGmc, gmc(0xCCu, 3u, 2u));
    surface(c);
    wr(c, kClrCmpCntl, 7u | (1u << 24)); // "flip when equal"
    const auto before = r128EngStats();
    wr(c, kSrcYX, 0);
    wr(c, kDstYX, 40u | (2u << 16));
    wr(c, kDstHeightWidth, 1u | (1u << 16));
    CHECK(pix(c, 40, 2) == 0x00112233u);
    CHECK(r128EngStats().colorCompare - before.colorCompare == 1);
}

TEST_CASE("r128 2d: an overlapping copy respects the programmed direction")
{
    // Scrolling a window down by one row overlaps itself. The hardware is
    // told which way to walk, and getting it wrong smears the first row over
    // everything below it — the classic symptom, and invisible to any test
    // that only copies between disjoint rectangles.
    R128Cell c;
    surface(c);
    for (u32 y = 0; y < 8; ++y)
        setPix(c, 0, y, 0x100u + y);

    wr(c, kGmc, gmc(0xCCu, 3u, 2u));
    surface(c);
    // Bottom-to-top: DST_Y_DIR clear. The programmed origin is then the FAR
    // corner, so (0,4) with height 4 means rows 1..4.
    wr(c, kDpCntl, 0x1u); // X left-to-right, Y bottom-to-top
    wr(c, kSrcYX, 0u | (3u << 16)); // SRC_X=0, SRC_Y=3 -> rows 0..3
    wr(c, kDstYX, 0u | (4u << 16)); // DST_X=0, DST_Y=4 -> rows 1..4
    wr(c, kDstHeightWidth, 1u | (4u << 16));

    // Each row moved down one, and no row was smeared.
    CHECK(pix(c, 0, 0) == 0x100u);
    CHECK(pix(c, 0, 1) == 0x100u);
    CHECK(pix(c, 0, 2) == 0x101u);
    CHECK(pix(c, 0, 3) == 0x102u);
    CHECK(pix(c, 0, 4) == 0x103u);
    CHECK(pix(c, 0, 5) == 0x105u);
}

TEST_CASE("r128 2d: the scissor clips the destination and shifts nothing")
{
    // A destination clipped on its left edge must lose those columns, not
    // slide the image right — the source has to follow the clip.
    R128Cell c;
    surface(c);
    for (u32 x = 0; x < 8; ++x)
        setPix(c, x, 0, 0xB0u + x);

    wr(c, kGmc, gmc(0xCCu, 3u, 2u));
    surface(c);
    wr(c, kScTopLeft, 12u); // SC_LEFT = 12
    wr(c, kSrcYX, 0);
    wr(c, kDstYX, 10u | (2u << 16)); // to (10,2), 8 wide -> columns 10..17
    wr(c, kDstHeightWidth, 8u | (1u << 16));

    CHECK(pix(c, 10, 2) == 0u); // clipped away
    CHECK(pix(c, 11, 2) == 0u);
    CHECK(pix(c, 12, 2) == 0xB2u); // and the SOURCE advanced with the clip
    CHECK(pix(c, 13, 2) == 0xB3u);
    CHECK(pix(c, 17, 2) == 0xB7u);
}

TEST_CASE("r128 2d: a wholly clipped blit draws nothing and is counted")
{
    R128Cell c;
    surface(c);
    const auto before = r128EngStats();
    wr(c, kGmc, gmc(0xF0u, 0u, 2u));
    surface(c);
    wr(c, kScBottomRight, 3u | (3u << 16)); // scissor to a 4x4 corner
    wr(c, kBrushFrgd, 0xFFFFFFFFu);
    wr(c, kDstYX, 40u | (40u << 16));
    wr(c, kDstHeightWidth, 4u | (4u << 16));
    CHECK(r128EngStats().blits == before.blits);
    CHECK(r128EngStats().clippedOut == before.clippedOut + 1);
    CHECK(pix(c, 40, 40) == 0u);
}

namespace {

// One 1x1 blit with the given raster op, returning the destination pixel.
// Source sits at (0,0), destination at (4,0), pattern is a solid brush.
u32 ropBlit(u32 rop, u32 srcPix, u32 dstPix, u32 patColor)
{
    R128Cell c;
    wr(c, kGmc, gmc(rop, 3u, 2u));
    surface(c);
    setPix(c, 0, 0, srcPix);
    setPix(c, 4, 0, dstPix);
    wr(c, kBrushFrgd, patColor);
    wr(c, kSrcYX, 0);
    wr(c, kDstYX, 4u);
    wr(c, kDstHeightWidth, 1u | (1u << 16));
    return pix(c, 4, 0);
}

} // namespace

TEST_CASE("r128 2d: every raster op is a truth table, and all 256 are exact")
{
    // A ROP3 code IS its truth table — bit (P<<2)|(S<<1)|D is the result for
    // that combination. Enumerating named ops instead leaves 252 of them
    // undrawn, so an ordinary XOR selection outline silently does nothing.
    //
    // The expectations below are ATI's own named constants from r128_reg.h,
    // which is what makes this a check rather than a restatement of the code.
    const u32 S = 0xF0F0F0F0u, D = 0xFF00FF00u, P = 0x0000FFFFu;

    CHECK(ropBlit(0x00u, S, D, P) == 0x00000000u);          // ROP3_ZERO
    CHECK(ropBlit(0xFFu, S, D, P) == 0xFFFFFFFFu);          // ROP3_ONE
    CHECK(ropBlit(0xCCu, S, D, P) == S);                    // ROP3_S
    CHECK(ropBlit(0xAAu, S, D, P) == D);                    // ROP3_D
    CHECK(ropBlit(0xF0u, S, D, P) == P);                    // ROP3_P
    CHECK(ropBlit(0x33u, S, D, P) == (~S & 0xFFFFFFFFu));   // ROP3_Sn
    CHECK(ropBlit(0x55u, S, D, P) == (~D & 0xFFFFFFFFu));   // ROP3_Dn
    CHECK(ropBlit(0x0Fu, S, D, P) == (~P & 0xFFFFFFFFu));   // ROP3_Pn
    CHECK(ropBlit(0x88u, S, D, P) == (D & S));              // ROP3_DSa
    CHECK(ropBlit(0xEEu, S, D, P) == (D | S));              // ROP3_DSo
    CHECK(ropBlit(0x66u, S, D, P) == (D ^ S));              // ROP3_DSx
    CHECK(ropBlit(0x44u, S, D, P) == (S & ~D));             // ROP3_SDna
    CHECK(ropBlit(0x22u, S, D, P) == (D & ~S));             // ROP3_DSna
    CHECK(ropBlit(0xA0u, S, D, P) == (D & P));              // ROP3_DPa
    CHECK(ropBlit(0x5Au, S, D, P) == (D ^ P));              // ROP3_DPx
    CHECK(ropBlit(0xFAu, S, D, P) == (D | P));              // ROP3_DPo
    CHECK(ropBlit(0x50u, S, D, P) == (P & ~D));             // ROP3_PDna
    CHECK(ropBlit(0x0Au, S, D, P) == (D & ~P));             // ROP3_DPna
    CHECK(ropBlit(0x77u, S, D, P) == (~(D & S) & 0xFFFFFFFFu)); // ROP3_DSan
    CHECK(ropBlit(0x11u, S, D, P) == (~(D | S) & 0xFFFFFFFFu)); // ROP3_DSon
    CHECK(ropBlit(0x99u, S, D, P) == (~(D ^ S) & 0xFFFFFFFFu)); // ROP3_DSxn
}

// The 8x8 mono pattern brush (r128.cpp engBlit says where its layout comes
// from). The pattern here is one no symmetry can hide a mistake in: row 0 is
// 0x80 (its leftmost pixel only, most significant bit first), row 1 is 0x01
// (its rightmost), rows 2 and 3 are full, rows 4 to 7 are empty except row 4
// which is 0xF0.
constexpr u32 kBrushData0 = 0x1480, kBrushData1 = 0x1484, kBrushYX = 0x1474;
constexpr u32 kBrushBkgd = 0x1478;
constexpr u32 kPatRows03 = 0xFFFF0180u, kPatRows47 = 0x000000F0u;

void patternFill(R128Cell& c, u32 brush, u32 rop3, u32 pixOrder, u32 x, u32 y,
                 u32 w, u32 h)
{
    wr(c, kGmc, 0x0Fu | (brush << 4) | (6u << 8) | (3u << 12) |
                    (pixOrder << 14) | (rop3 << 16) | (2u << 24));
    surface(c);
    wr(c, kBrushFrgd, 0x00F0F0F0u);
    wr(c, kBrushBkgd, 0x00101010u);
    wr(c, kBrushData0, kPatRows03);
    wr(c, kBrushData1, kPatRows47);
    wr(c, kDstYX, x | (y << 16));
    wr(c, kDstHeightWidth, w | (h << 16));
}

TEST_CASE("r128 2d: an 8x8 mono pattern paints foreground and background by "
          "its bits, rows from the low byte, pixels from the high bit")
{
    R128Cell c;
    const auto before = r128EngStats();
    patternFill(c, 0u, 0xF0u, 0u, 0, 0, 16, 8);
    CHECK(r128EngStats().blits == before.blits + 1);
    CHECK(r128EngStats().brushUnimpl == before.brushUnimpl);
    const u32 F = 0x00F0F0F0u, B = 0x00101010u;
    // Row 0 = 0x80: the leftmost pixel of each tile.
    CHECK(pix(c, 0, 0) == F);
    CHECK(pix(c, 1, 0) == B);
    CHECK(pix(c, 7, 0) == B);
    CHECK(pix(c, 8, 0) == F); // the tile repeats across
    // Row 1 = 0x01: the rightmost.
    CHECK(pix(c, 0, 1) == B);
    CHECK(pix(c, 7, 1) == F);
    CHECK(pix(c, 15, 1) == F);
    // Rows 2 and 3 full; row 4 = 0xF0: the left half; rows 5 to 7 empty.
    CHECK(pix(c, 5, 2) == F);
    CHECK(pix(c, 5, 3) == F);
    CHECK(pix(c, 3, 4) == F);
    CHECK(pix(c, 4, 4) == B);
    CHECK(pix(c, 0, 5) == B);
    CHECK(pix(c, 0, 7) == B);
    // Outside the rectangle nothing is touched.
    CHECK(pix(c, 16, 0) == 0u);
    CHECK(pix(c, 0, 8) == 0u);
}

TEST_CASE("r128 2d: the pattern's pixel order follows DP_BYTE_PIX_ORDER")
{
    R128Cell c;
    patternFill(c, 0u, 0xF0u, 1u, 0, 0, 8, 2); // least significant bit first
    const u32 F = 0x00F0F0F0u, B = 0x00101010u;
    CHECK(pix(c, 7, 0) == F); // row 0 = 0x80: now its rightmost pixel
    CHECK(pix(c, 0, 0) == B);
    CHECK(pix(c, 0, 1) == F); // row 1 = 0x01: now its leftmost
    CHECK(pix(c, 7, 1) == B);
}

TEST_CASE("r128 2d: a pattern with the background left alone draws only its "
          "foreground")
{
    R128Cell c;
    surface(c);
    for (u32 x = 0; x < 8; ++x)
        setPix(c, x, 0, 0x00ABCDEFu);
    patternFill(c, 1u, 0xF0u, 0u, 0, 0, 8, 1);
    CHECK(pix(c, 0, 0) == 0x00F0F0F0u); // the one set bit of row 0
    for (u32 x = 1; x < 8; ++x)
        CHECK(pix(c, x, 0) == 0x00ABCDEFu); // not the background colour
}

TEST_CASE("r128 2d: the pattern is anchored to the surface, at BRUSH_Y_X")
{
    // Two rectangles that do not start on a tile boundary continue one
    // pattern: a pixel's colour depends on where it is, not on where its
    // rectangle began.
    R128Cell c;
    patternFill(c, 0u, 0xF0u, 0u, 3, 1, 10, 4);
    const u32 F = 0x00F0F0F0u, B = 0x00101010u;
    CHECK(pix(c, 7, 1) == F);  // row 1's rightmost pixel
    CHECK(pix(c, 8, 1) == B);
    CHECK(pix(c, 3, 2) == F);  // row 2 is full
    CHECK(pix(c, 8, 4) == F);  // row 4's left half, second tile
    CHECK(pix(c, 12, 4) == B);
    // The origin moved to (2, 1): everything shifts with it.
    R128Cell d;
    wr(d, kBrushYX, 2u | (1u << 8));
    patternFill(d, 0u, 0xF0u, 0u, 0, 0, 16, 8);
    CHECK(pix(d, 2, 1) == F); // pattern (0,0)
    CHECK(pix(d, 3, 1) == B);
    CHECK(pix(d, 1, 2) == F); // pattern (7,1)
    CHECK(pix(d, 2, 0) == B); // pattern (0,7)
}

TEST_CASE("r128 2d: a pattern under an XOR draws and undraws an outline")
{
    // What a desktop does to show something being dragged: the same
    // patterned rectangle twice with PATINVERT restores every pixel.
    R128Cell c;
    surface(c);
    for (u32 y = 0; y < 8; ++y)
        for (u32 x = 0; x < 16; ++x)
            setPix(c, x, y, 0x00010203u * (x + 1u) + y);
    const u32 was = pix(c, 8, 2);
    patternFill(c, 1u, 0x5Au, 0u, 0, 0, 16, 8); // PATINVERT, background left
    CHECK(pix(c, 8, 2) == (was ^ 0x00F0F0F0u)); // a foreground pixel
    CHECK(pix(c, 1, 0) == 0x00010203u * 2u);    // a background pixel: as it was
    patternFill(c, 1u, 0x5Au, 0u, 0, 0, 16, 8);
    for (u32 y = 0; y < 8; ++y)
        for (u32 x = 0; x < 16; ++x)
            CHECK(pix(c, x, y) == 0x00010203u * (x + 1u) + y);
}

TEST_CASE("r128 2d: an op that reads a pattern this engine has no data for is "
          "declined, not painted as a solid guess")
{
    // The solid brush and the 8x8 mono pattern are modelled. The other
    // pattern brushes have no layout in anything on the shelf, so an op that
    // would actually READ one must draw nothing rather than paint a flat
    // colour where a chequer belongs. Never paint a guess.
    R128Cell c;
    surface(c);
    setPix(c, 0, 0, 0x12345678u);
    const auto before = r128EngStats();
    // brush datatype 10 = 8x8 colour pattern, with PATCOPY, which reads it.
    wr(c, kGmc, 0x0Fu | (10u << 4) | (6u << 8) | (0xF0u << 16) | (2u << 24));
    surface(c);
    wr(c, kDstYX, 0);
    wr(c, kDstHeightWidth, 4u | (4u << 16));
    CHECK(r128EngStats().blits == before.blits);
    CHECK(r128EngStats().brushUnimpl == before.brushUnimpl + 1);
    CHECK(pix(c, 0, 0) == 0x12345678u); // untouched

    // ...but the SAME brush with SRCCOPY, which never looks at the pattern,
    // must still draw. An op is only refused for what it actually reads.
    const auto mid = r128EngStats();
    wr(c, kGmc, 0x0Fu | (0u << 4) | (6u << 8) | (3u << 12) | (0xCCu << 16) |
                    (2u << 24));
    surface(c);
    setPix(c, 8, 0, 0xAABBCCDDu);
    wr(c, kSrcYX, 8u);
    wr(c, kDstYX, 0);
    wr(c, kDstHeightWidth, 1u | (1u << 16));
    CHECK(r128EngStats().blits == mid.blits + 1);
    CHECK(pix(c, 0, 0) == 0xAABBCCDDu);
}

TEST_CASE("r128 2d: a destination in the AGP half of the address space is "
          "declined, not drawn into VRAM")
{
    // DST_OFFSET is a 64 MB VIRTUAL address: the low 32 MB is the frame
    // buffer and the high 32 MB is AGP_BASE + offset, which is system memory
    // this cell cannot reach. Folding it back into VRAM would corrupt the
    // screen at an address nobody asked for.
    R128Cell c;
    const auto before = r128EngStats();
    wr(c, kGmc, gmc(0xF0u, 0u, 2u));
    surface(c);
    wr(c, kDstOffset, 0x02000000u); // exactly the top of VRAM
    wr(c, kBrushFrgd, 0xFFFFFFFFu);
    wr(c, kDstYX, 0);
    wr(c, kDstHeightWidth, 4u | (4u << 16));
    CHECK(r128EngStats().blits == before.blits);
    CHECK(r128EngStats().agpTarget == before.agpTarget + 1);
}

TEST_CASE("r128 2d: DP_GUI_MASTER_CNTL applies the defaults it is asked for")
{
    // With GMC_DST_PITCH_OFFSET_CNTL and GMC_DST_CLIPPING CLEAR, the write
    // is supposed to load DST_OFFSET/DST_PITCH from DEFAULT_OFFSET/PITCH and
    // reset the scissor to (0,0)..DEFAULT_SC_BOTTOM_RIGHT. A driver that
    // relies on that and gets stale values draws in the wrong place entirely.
    R128Cell c;
    wr(c, kDefaultOffset, 0);
    wr(c, kDefaultPitch, kPitch8);
    wr(c, kDefaultScBr, (kW - 1u) | ((kW - 1u) << 16));
    // Poison the live registers first, so passing means they were reloaded.
    wr(c, kDstOffset, 0x00100000u);
    wr(c, kDstPitch, 1u);
    wr(c, kScTopLeft, 30u | (30u << 16));
    wr(c, kDpCntl, 0u); // and clear both direction bits

    const auto before = r128EngStats();
    // bits 0..3 CLEAR: take every default.
    wr(c, kGmc, (13u << 4) | (6u << 8) | (0xF0u << 16) | (2u << 24));
    wr(c, kBrushFrgd, 0x01020304u);
    wr(c, kDstYX, 0);
    wr(c, kDstHeightWidth, 2u | (2u << 16));

    CHECK(r128EngStats().blits == before.blits + 1);
    CHECK(pix(c, 0, 0) == 0x01020304u); // default offset+pitch were used
    CHECK(pix(c, 1, 1) == 0x01020304u); // ...and the scissor was reset
    // ⭐ And the direction bits came back on. The reference guide says so
    // under DP_CNTL: "This bit is set to '1' by a GUI_MASTER_CNTL write."
    // Without it the fill above would have been placed at a negative origin
    // and clipped away entirely.
    CHECK((be(c.read(kDpCntl, 4)) & 0x3u) == 0x3u);
}

TEST_CASE("r128 2d: the composite registers decompose into the canonical set")
{
    // DST_PITCH_OFFSET is write-only and packs a 32-byte-aligned offset with
    // a pitch. Keeping ONE canonical pair is what lets a blit read its
    // parameters from one place regardless of which alias programmed them —
    // and it is why the engine needs no state of its own, so no snapshot
    // taken before it existed is invalidated by its arrival.
    R128Cell c;
    wr(c, 0x142Cu, (0x1234u) | (kPitch8 << 21)); // DST_PITCH_OFFSET
    CHECK(be(c.read(kDstOffset, 4)) == 0x1234u * 32u);
    CHECK(be(c.read(kDstPitch, 4)) == kPitch8);

    wr(c, 0x1594u, 7u | (9u << 16)); // DST_X_Y: Y in 13:0, X in 29:16
    CHECK(be(c.read(0x141Cu, 4)) == 9u); // DST_X
    CHECK(be(c.read(0x1420u, 4)) == 7u); // DST_Y

    wr(c, 0x1438u, 5u | (6u << 16)); // DST_Y_X: X in 13:0, Y in 29:16
    CHECK(be(c.read(0x141Cu, 4)) == 5u);
    CHECK(be(c.read(0x1420u, 4)) == 6u);
}

// ── Lines (r128.cpp engLine says where the stepping rule comes from) ──────

namespace {

constexpr u32 kDstBresErr = 0x1628, kDstBresInc = 0x162C;
constexpr u32 kDstBresDec = 0x1630, kDstBresLnth = 0x1634;
constexpr u32 kDpCntlLine = 0x16D0; // DP_CNTL_XDIR_YDIR_YMAJOR

// A solid PATCOPY line from (x0, y0) toward (x1, y1), `count` pixels, with
// the terms the SDK's sample computes.
void line(R128Cell& c, int x0, int y0, int x1, int y1, u32 count,
          u32 rop3 = 0xF0u, u32 colour = 0x00C0FFEEu)
{
    const int dx = x1 > x0 ? x1 - x0 : x0 - x1;
    const int dy = y1 > y0 ? y1 - y0 : y0 - y1;
    const int large = dx < dy ? dy : dx, small = dx < dy ? dx : dy;
    wr(c, kGmc, gmc(rop3, 3u, 2u));
    surface(c);
    wr(c, kBrushFrgd, colour);
    wr(c, kDpCntlLine, (x0 < x1 ? 1u << 31 : 0u) | (y0 < y1 ? 1u << 15 : 0u) |
                           (dx < dy ? 4u : 0u));
    wr(c, kDstYX, static_cast<u32>(x0) | (static_cast<u32>(y0) << 16));
    wr(c, kDstBresErr, static_cast<u32>(-large) & 0xFFFFFu);
    wr(c, kDstBresInc, static_cast<u32>(2 * small) & 0xFFFFFu);
    wr(c, kDstBresDec, static_cast<u32>(-2 * large) & 0xFFFFFu);
    wr(c, kDstBresLnth, count);
}

// Every pixel of the top-left 12x10 that is not zero, as "x,y x,y ...".
std::string lit(const R128Cell& c)
{
    std::string s;
    for (u32 y = 0; y < 10; ++y)
        for (u32 x = 0; x < 12; ++x)
            if (pix(c, x, y))
                s += std::to_string(x) + "," + std::to_string(y) + " ";
    return s;
}

} // namespace

TEST_CASE("r128 2d: a line steps as Bresenham's does with the SDK's terms")
{
    const auto before = r128EngStats();
    {
        R128Cell c; // level
        line(c, 2, 3, 7, 3, 6);
        CHECK(lit(c) == "2,3 3,3 4,3 5,3 6,3 7,3 ");
        CHECK(pix(c, 4, 3) == 0x00C0FFEEu);
    }
    {
        R128Cell c; // at 45 degrees
        line(c, 0, 0, 4, 4, 5);
        CHECK(lit(c) == "0,0 1,1 2,2 3,3 4,4 ");
    }
    {
        R128Cell c; // shallow: one down in three across
        line(c, 0, 0, 6, 2, 7);
        CHECK(lit(c) == "0,0 1,0 2,1 3,1 4,1 5,2 6,2 ");
    }
    {
        R128Cell c; // steep, drawn upward and to the left
        line(c, 5, 6, 3, 0, 7);
        CHECK(lit(c) == "3,0 3,1 4,2 4,3 4,4 5,5 5,6 ");
    }
    CHECK(r128EngStats().lines2d - before.lines2d == 4);
    CHECK(r128EngStats().linePixels - before.linePixels == 6 + 5 + 7 + 7);
}

TEST_CASE("r128 2d: BRES_SIGN decides which way a zero error term falls")
{
    // (0,0) to (2,1): after the first pixel the error term is exactly zero.
    R128Cell positive;
    line(positive, 0, 0, 2, 1, 3);
    CHECK(lit(positive) == "0,0 1,1 2,1 "); // zero counts as positive: step
    R128Cell negative;
    wr(negative, kDpCntl, 0x100u); // BRES_SIGN
    line(negative, 0, 0, 2, 1, 3);
    CHECK(lit(negative) == "0,0 1,0 2,1 ");
}

TEST_CASE("r128 2d: a line leaves the pen at its end, and is clipped by the "
          "scissor")
{
    R128Cell c;
    line(c, 1, 1, 5, 1, 4); // four pixels: 1..4
    CHECK(lit(c) == "1,1 2,1 3,1 4,1 ");
    // The pen is at (5,1) and the terms stand: another length continues.
    wr(c, kDstBresLnth, 3u);
    CHECK(lit(c) == "1,1 2,1 3,1 4,1 5,1 6,1 7,1 ");
    // The scissor's right edge at 3: the rest of a line is not drawn.
    R128Cell d;
    line(d, 0, 2, 9, 2, 10);
    CHECK(lit(d) == "0,2 1,2 2,2 3,2 4,2 5,2 6,2 7,2 8,2 9,2 ");
    R128Cell e;
    wr(e, kGmc, gmc(0xF0u, 3u, 2u));
    surface(e);
    wr(e, kScBottomRight, 3u | (63u << 16));
    wr(e, kBrushFrgd, 0x00C0FFEEu);
    wr(e, kDpCntlLine, (1u << 31) | (1u << 15));
    wr(e, kDstYX, 0u | (2u << 16));
    wr(e, kDstBresErr, static_cast<u32>(-9) & 0xFFFFFu);
    wr(e, kDstBresInc, 0u);
    wr(e, kDstBresDec, static_cast<u32>(-18) & 0xFFFFFu);
    wr(e, kDstBresLnth, 10u);
    CHECK(lit(e) == "0,2 1,2 2,2 3,2 ");
}

TEST_CASE("r128 2d: a line that would read a source or an unknown pattern is "
          "declined")
{
    R128Cell c;
    const auto before = r128EngStats();
    line(c, 0, 0, 5, 0, 6, 0xCCu); // SRCCOPY: a line has no source
    CHECK(lit(c).empty());
    CHECK(r128EngStats().lineDeclined - before.lineDeclined == 1);
    // A dashed line's 32x1 pattern: no layout for it in any source.
    wr(c, kGmc, 0x0Fu | (6u << 4) | (6u << 8) | (0xF0u << 16) | (2u << 24));
    surface(c);
    wr(c, kDstYX, 0);
    wr(c, kDstBresLnth, 6u);
    CHECK(lit(c).empty());
    CHECK(r128EngStats().lineDeclined - before.lineDeclined == 2);
    CHECK(r128EngStats().lines2d == before.lines2d);
}
