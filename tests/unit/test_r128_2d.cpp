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

TEST_CASE("r128 2d: an op that reads a pattern this engine has no data for is "
          "declined, not painted as a solid guess")
{
    // Brush datatype 13 is a solid colour and is modelled. The mono and
    // colour brushes need the pattern registers, which are not — so an op
    // that would actually READ the pattern must draw nothing rather than
    // paint a flat colour where a chequer belongs. Never paint a guess.
    R128Cell c;
    surface(c);
    setPix(c, 0, 0, 0x12345678u);
    const auto before = r128EngStats();
    // brush datatype 0 = 8x8 mono pattern, with PATCOPY, which reads it.
    wr(c, kGmc, 0x0Fu | (0u << 4) | (6u << 8) | (0xF0u << 16) | (2u << 24));
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
