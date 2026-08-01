// The CCE's PIO packet path: FIFO words in, register writes out.
//
// Measured 2026-08-01 on the first boot with a working .AGP driver: the ATI
// Resource Manager configures PM4_BUFFER_CNTL mode 7 and submits through
// PM4_FIFO_DATA_EVEN/ODD — two Type-0 packets, the first dispatching a
// 24-DWord INDIRECT buffer through the Uni-N GART, the second writing a
// 64-bit UpTime fence into GUI_SCRATCH_REG0/1, which the ARM then polls
// 43.7 million times. The whole desktop hang chain ends at whether these
// packets execute, so the parser gets its discriminating cases here, seeded
// with the exact words the guest was measured submitting.
//
// Packet formats are SDK-G04000 App F: Type-0 = TYPE 31:30 | COUNT 29:16
// (N-1) | ONE_REG_WR 15 | BASE_INDEX 10:0; Type-1 = two indexed registers;
// Type-2 = filler; Type-3 = COUNT (N-1) | IT_OPCODE 15:8, body skipped and
// tallied until an opcode is actually observed and implemented.

#include "doctest.h"
#include "opm/r128.hpp"

#include <vector>

using namespace opm;

namespace {

u32 be(u32 v)
{
    return (v >> 24) | ((v >> 8) & 0xFF00u) | ((v << 8) & 0xFF0000u) |
           (v << 24);
}

// A little machine: 4 MB of "system RAM", a GART table at 1 MB mapping AGP
// page 0x101 (the measured indirect offset 0x101000) to the page at 2 MB.
struct CceRig {
    R128Cell c;
    std::vector<u8> ram;
    static constexpr u32 kGartBase = 0x00100000u;
    static constexpr u32 kBodyPa = 0x00200000u;
    static constexpr u32 kAgpOff = 0x00101000u;

    CceRig() : ram(4u << 20, 0)
    {
        put32(kGartBase + (kAgpOff >> 12) * 4u, kBodyPa | 1u);
    }
    void put32(u32 at, u32 v) // little-endian, as the guest builds tables
    {
        ram[at] = static_cast<u8>(v);
        ram[at + 1] = static_cast<u8>(v >> 8);
        ram[at + 2] = static_cast<u8>(v >> 16);
        ram[at + 3] = static_cast<u8>(v >> 24);
    }
    void body(std::initializer_list<u32> words)
    {
        u32 at = kBodyPa;
        for (u32 w : words) {
            put32(at, w);
            at += 4;
        }
    }
    void fifo(u32 word)
    {
        r128CceFifoWord(c, be(word), 4, ram.data(),
                        static_cast<u32>(ram.size()), kGartBase, nullptr);
    }
};

} // namespace

TEST_CASE("cce: the measured six-DWORD submission lands the fence")
{
    CceRig r;
    const auto before = r128EngStats();
    // A 24-DWord indirect body of Type-2 fillers: content-free, but the
    // fetch count and the parse count both have to come out exact.
    {
        u32 at = CceRig::kBodyPa;
        for (int i = 0; i < 24; ++i, at += 4)
            r.put32(at, 0x80000000u);
    }
    // The submission, verbatim from the engine log @7,915,918,911.
    r.fifo(0x000101CEu); // Type-0, 2 regs @ 0x0738
    r.fifo(0x00101000u); //   PM4_IW_INDOFF
    r.fifo(0x00000018u); //   PM4_IW_INDSIZE = 24 -> dispatch
    r.fifo(0x00010578u); // Type-0, 2 regs @ 0x15E0
    r.fifo(0xD9C4E0D7u); //   GUI_SCRATCH_REG0
    r.fifo(0x00000001u); //   GUI_SCRATCH_REG1
    const auto& e = r128EngStats();
    CHECK(e.cceWords - before.cceWords == 6);
    CHECK(e.ccePkt0 - before.ccePkt0 == 2);
    CHECK(e.cceIndFetch - before.cceIndFetch == 1);
    CHECK(e.cceIndWords - before.cceIndWords == 24);
    CHECK(e.ccePkt2 - before.ccePkt2 == 24);
    CHECK(e.cceGartMiss == before.cceGartMiss);
    // The fence is the point: the ARM polls these two registers.
    CHECK(r.c.peek(0x15E0u) == 0xD9C4E0D7u);
    CHECK(r.c.peek(0x15E4u) == 0x00000001u);
}

TEST_CASE("cce: a packet split across FIFO writes waits for its body")
{
    CceRig r;
    const auto before = r128EngStats();
    r.fifo(0x00010578u); // header alone: 2 data words owed
    CHECK(r128EngStats().ccePkt0 == before.ccePkt0);
    CHECK(r.c.peek(0x15E0u) != 0x11111111u);
    r.fifo(0x11111111u);
    CHECK(r128EngStats().ccePkt0 == before.ccePkt0); // still one word short
    r.fifo(0x22222222u);
    CHECK(r128EngStats().ccePkt0 - before.ccePkt0 == 1);
    CHECK(r.c.peek(0x15E0u) == 0x11111111u);
    CHECK(r.c.peek(0x15E4u) == 0x22222222u);
}

TEST_CASE("cce: ONE_REG_WR streams every word into the same register")
{
    CceRig r;
    // Type-0, COUNT=2 (3 words), ONE_REG_WR set, base 0x15E0.
    r.fifo(0x00028578u | (1u << 15));
    r.fifo(0xAAAAAAAAu);
    r.fifo(0xBBBBBBBBu);
    r.fifo(0xCCCCCCCCu);
    CHECK(r.c.peek(0x15E0u) == 0xCCCCCCCCu); // last write wins
    CHECK(r.c.peek(0x15E4u) != 0xBBBBBBBBu); // neighbour untouched
}

TEST_CASE("cce: Type-1 writes its two indexed registers")
{
    CceRig r;
    // REG_INDEX1 = 0x578 (0x15E0), REG_INDEX2 = 0x579 (0x15E4).
    r.fifo(0x40000000u | (0x579u << 11) | 0x578u);
    r.fifo(0x12345678u);
    r.fifo(0x9ABCDEF0u);
    CHECK(r.c.peek(0x15E0u) == 0x12345678u);
    CHECK(r.c.peek(0x15E4u) == 0x9ABCDEF0u);
    CHECK(r128EngStats().ccePkt1 >= 1);
}

TEST_CASE("cce: Type-3 is skipped exactly and the stream stays in step")
{
    CceRig r;
    const auto before = r128EngStats();
    // Type-3, opcode 0x11, COUNT=1 -> 2-DWord body.
    r.fifo(0xC0001100u | (1u << 16));
    r.fifo(0xDEADBEEFu);
    r.fifo(0xFEEDFACEu);
    // If the skip miscounted, this Type-0 would be swallowed as body.
    r.fifo(0x00000578u); // 1 reg @ 0x15E0
    r.fifo(0x77777777u);
    const auto& e = r128EngStats();
    CHECK(e.ccePkt3 - before.ccePkt3 == 1);
    CHECK(r128CceP3Skipped().count(0x11u) == 1);
    CHECK(r.c.peek(0x15E0u) == 0x77777777u);
}

TEST_CASE("cce: an invalid GART entry declines the fetch and counts it")
{
    CceRig r;
    const auto before = r128EngStats();
    // Point the indirect window at an AGP page with NO valid entry.
    r.fifo(0x000101CEu);
    r.fifo(0x00303000u); // idx 0x303: entry is zero
    r.fifo(0x00000004u);
    const auto& e = r128EngStats();
    CHECK(e.cceGartMiss > before.cceGartMiss);
    CHECK(e.cceIndWords == before.cceIndWords);
    // The FIFO stream itself must survive the decline.
    r.fifo(0x00000578u);
    r.fifo(0x55555555u);
    CHECK(r.c.peek(0x15E0u) == 0x55555555u);
}

TEST_CASE("cce: PAINT (0x91) fills its rectangles through SETTINGS")
{
    CceRig r;
    const auto before = r128EngStats();
    // GUI_CONTROL: DST_PITCH_OFFSET supplied (b1), DST clipping supplied
    // (b3), solid brush (13), 32bpp dst (6), PATCOPY (0xF0).
    const u32 gc = 2u | 8u | (13u << 4) | (6u << 8) | (0xF0u << 16);
    // Type-3 PAINT, 7-DWord body: settings (5) + one rect (2).
    r.fifo(0xC0009100u | (6u << 16));
    r.fifo(gc);
    r.fifo(0x00800000u);  // DST_PITCH_OFFSET: pitch8=4 (32 px), offset 0
    r.fifo(0x00000000u);  // SC_TOP_LEFT
    r.fifo(0x001F001Fu);  // SC_BOT_RITE (31,31)
    r.fifo(0x00AA5500u);  // FRGD colour (solid brush packet)
    r.fifo(0x00010002u);  // TOP=1 LEFT=2
    r.fifo(0x00050006u);  // BOTM=5 RITE=6 -> 4x4 exclusive
    const auto& e = r128EngStats();
    CHECK(e.ccePkt3 - before.ccePkt3 == 1);
    CHECK(e.fills - before.fills == 1);
    CHECK(e.pixels - before.pixels == 16);
    // And the op must NOT appear in the skipped map as new.
    // (It may exist from other cases' tallies only if they used 0x91 —
    // none do.)
    CHECK(r128CceP3Skipped().count(0x91u) == 0);
}

TEST_CASE("cce: PAINT_MULTI (0x9A) takes half-swapped coordinates")
{
    CceRig r;
    const auto before = r128EngStats();
    const u32 gc = 2u | 8u | (13u << 4) | (6u << 8) | (0xF0u << 16);
    r.fifo(0xC0009A00u | (6u << 16)); // 7-DWord body
    r.fifo(gc);
    r.fifo(0x00800000u); // DST_PITCH_OFFSET
    r.fifo(0x00000000u); // SC_TOP_LEFT
    r.fifo(0x001F001Fu); // SC_BOT_RITE
    r.fifo(0x00123456u); // FRGD
    r.fifo(0x00020001u); // DST_X=2 | DST_Y=1
    r.fifo(0x00040003u); // DST_W=4 | DST_H=3
    const auto& e = r128EngStats();
    CHECK(e.fills - before.fills == 1);
    CHECK(e.pixels - before.pixels == 12);
    CHECK(r128CceP3Skipped().count(0x9Au) == 0);
}

TEST_CASE("cce: HOSTDATA_BLT (0x94) expands a mono glyph MSB-first")
{
    CceRig r;
    const auto before = r128EngStats();
    // No brush (15), 32bpp dst, mono OPAQUE source, SRCCOPY.
    const u32 gc = 2u | 8u | (15u << 4) | (6u << 8) | (0u << 12) |
                   (0xCCu << 16);
    r.fifo(0xC0009400u | (10u << 16)); // 11-DWord body
    r.fifo(gc);
    r.fifo(0x00800000u); // DST_PITCH_OFFSET: pitch8=4, offset 0
    r.fifo(0x00000000u); // SC_TOP_LEFT
    r.fifo(0x001F001Fu); // SC_BOT_RITE
    r.fifo(0x00FFFFFFu); // FRGD (white)
    r.fifo(0x00000000u); // BKGD (black)
    r.fifo(0x00010001u); // BaseY=1 | BaseX=1
    r.fifo(0x00080008u); // HEIGHT=8 | WIDTH=8
    r.fifo(0x00000002u); // NUMBER = 2 DWORDs
    r.fifo(0xF0F0F0F0u); // rows: 11110000 repeated
    r.fifo(0x0F0F0F0Fu);
    const auto& e = r128EngStats();
    CHECK(e.blits - before.blits == 1);
    CHECK(e.pixels - before.pixels == 64); // opaque: every pixel written
    CHECK(r128CceP3Skipped().count(0x94u) == 0);
    // Row 0 of the glyph lands at y=1: MSB-first, 0xF0... -> pixels 1..4 of
    // the row are foreground, 5..8 background. Stride = 4 px8 * 8 * 4 B.
    const size_t stride = 4u * 8u * 4u;
    CHECK(r.c.vram[stride * 1u + 1u * 4u + 1u] == 0xFF); // (1,1) fg
    CHECK(r.c.vram[stride * 1u + 5u * 4u + 1u] == 0x00); // (5,1) bk
}

TEST_CASE("cce: HOSTDATA_BLT colour keeps the guest's MEMORY byte order")
{
    CceRig r;
    const auto before = r128EngStats();
    // The contract under test is byte ORDER, so the packet goes through
    // the indirect path: the raster bytes are planted in guest RAM in
    // framebuffer order and must land in VRAM in that same order. The
    // GART fetch composes DWORDs little-endian, so an extractor that
    // walks them MSB-first reverses every pixel — white 00 FF FF FF
    // becomes FF FF FF 00, which scans out yellow; that bug shipped once
    // and the user's eyes caught it on the menu bar.
    const u32 gc = 2u | 8u | (15u << 4) | (6u << 8) | (3u << 12) |
                   (0xCCu << 16);
    // Whole Type-3 packet in the indirect body; put32 is little-endian,
    // so memory bytes [A1 B2 C3 D4] are put32(0xD4C3B2A1).
    r.body({
        0xC0009400u | (12u << 16), // header: 13-DWord body
        gc,
        0x00800000u,               // DST_PITCH_OFFSET
        0x00000000u,               // SC_TOP_LEFT
        0x001F001Fu,               // SC_BOT_RITE
        0x00000000u,               // FRGD (ineffective for colour)
        0x00000000u,               // BKGD (ineffective)
        0x00020003u,               // BaseY=2 | BaseX=3
        0x00020002u,               // HEIGHT=2 | WIDTH=2
        0x00000004u,               // NUMBER = 4 DWORDs (2x2 @ 32bpp)
        0xD4C3B2A1u,               // memory bytes A1 B2 C3 D4
        0xD8C7B6A5u,               // memory bytes A5 B6 C7 D8
        0x44332211u,               // memory bytes 11 22 33 44
        0x88776655u,               // memory bytes 55 66 77 88
    });
    r.fifo(0x000101CEu); // Type-0, 2 regs @ 0x0738: dispatch the body
    r.fifo(0x00101000u);
    r.fifo(0x0000000Eu); // 14 DWORDs
    const auto& e = r128EngStats();
    CHECK(e.blits - before.blits == 1);
    CHECK(e.pixels - before.pixels == 4);
    CHECK(e.hostData == before.hostData); // no decline
    const size_t stride = 4u * 8u * 4u;
    // (3,2) holds the first pixel, bytes exactly as planted in RAM.
    CHECK(r.c.vram[stride * 2u + 3u * 4u + 0u] == 0xA1);
    CHECK(r.c.vram[stride * 2u + 3u * 4u + 1u] == 0xB2);
    CHECK(r.c.vram[stride * 2u + 3u * 4u + 2u] == 0xC3);
    CHECK(r.c.vram[stride * 2u + 3u * 4u + 3u] == 0xD4);
    // (4,2) is the second pixel, (4,3) the fourth.
    CHECK(r.c.vram[stride * 2u + 4u * 4u + 0u] == 0xA5);
    CHECK(r.c.vram[stride * 3u + 4u * 4u + 0u] == 0x55);
}

TEST_CASE("cce: an indirect body drives the 2D engine through PACKET0")
{
    CceRig r;
    const auto before = r128EngStats();
    // Body: a solid fill, the same programming test_r128_2d proves against
    // the blitter directly — DP_GUI_MASTER_CNTL for a PATCOPY fill, brush
    // colour, then DST_Y_X / DST_HEIGHT_WIDTH which initiates the draw.
    // 32bpp dst (datatype 6), brush 13 (solid frgd), ROP3 0xF0 PATCOPY.
    const u32 gmc = 0x0Fu | (13u << 4) | (6u << 8) | (0xF0u << 16);
    r.body({
        0x00000000u | (0x146Cu >> 2), gmc,          // DP_GUI_MASTER_CNTL
        0x00000000u | (0x1404u >> 2), 0x00000000u,  // DST_OFFSET
        0x00000000u | (0x1408u >> 2), 0x00000004u,  // DST_PITCH (32 px)
        0x00000000u | (0x16ECu >> 2), 0x00000000u,  // SC_TOP_LEFT
        0x00000000u | (0x16F0u >> 2), 0x001F001Fu,  // SC_BOTTOM_RIGHT
        0x00000000u | (0x147Cu >> 2), 0x00FF00FFu,  // DP_BRUSH_FRGD_CLR
        0x00000000u | (0x1438u >> 2), 0x00000000u,  // DST_Y_X = 0,0
        0x00000000u | (0x143Cu >> 2), 0x00040004u,  // 4 rows of 4 px
    });
    r.fifo(0x000101CEu);
    r.fifo(0x00101000u);
    r.fifo(0x00000010u); // 16 DWORDs
    const auto& e = r128EngStats();
    CHECK(e.cceIndWords - before.cceIndWords == 16);
    CHECK(e.fills - before.fills == 1);
    CHECK(e.pixels - before.pixels == 16);
}
