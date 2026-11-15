// The frame recorder: what reaches the card over a few frames goes into a
// file, and the file replayed into a fresh card must leave it exactly as the
// recorded one stood, at every frame edge and at the end. The control is a
// recording with one value changed, which must be told apart.

#include "doctest.h"
#include "opm/r128rec.hpp"

#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

using namespace opm;
namespace fs = std::filesystem;

namespace {

u32 be(u32 v)
{
    return (v >> 24) | ((v >> 8) & 0xFF00u) | ((v << 8) & 0xFF0000u) |
           (v << 24);
}

constexpr u32 kDstOffset = 0x1404, kDstPitch = 0x1408;
constexpr u32 kDstYX = 0x1438, kDstHeightWidth = 0x143C;
constexpr u32 kGmc = 0x146C, kBrushFrgd = 0x147C;
constexpr u32 kDpWriteMsk = 0x16CC;
constexpr u32 kScTopLeft = 0x16EC, kScBottomRight = 0x16F0;
constexpr u32 kBmAddr = 0x0A50, kGuiScratch0 = 0x15E0;
constexpr u32 kW = 64, kH = 8, kPitch8 = kW / 8, kStride = kPitch8 * 32u;

// A card showing a 64x8 picture in millions of colors from the start of
// video memory, and a megabyte of system memory beside it.
struct Rig {
    R128Cell c;
    std::vector<u8> ram;
    Rig() : ram(1u << 20, 0)
    {
        c.setReg(0x0050, 0x02000000u | (6u << 8));    // CRTC on, 32 bpp
        c.setReg(0x0200, (kW / 8u - 1u) << 16);       // CRTC_H_TOTAL_DISP
        c.setReg(0x0208, (kH - 1u) << 16);            // CRTC_V_TOTAL_DISP
        c.setReg(0x022C, kPitch8);                    // CRTC_PITCH
        c.setReg(0x0224, 0);                          // CRTC_OFFSET
    }
    CceMem mem()
    {
        return CceMem{ram.data(), static_cast<u32>(ram.size()), 0, 0, nullptr};
    }
    // A store to a register, as the bus carries it out.
    void reg(u32 off, u32 v) { r128GuestRegWrite(c, off, be(v), 4, mem()); }
    // A store by the processor into video memory, as the bus carries it out.
    void fb(u32 off, u32 v)
    {
        if (r128RecOn())
            r128RecFbWrite(off, v, 4);
        for (u32 k = 0; k < 4; ++k)
            c.vram[off + k] = static_cast<u8>(v >> (8u * (3u - k)));
    }
    void le32(u32 at, u32 v)
    {
        for (u32 k = 0; k < 4; ++k)
            ram[at + k] = static_cast<u8>(v >> (8u * k));
    }
    void fillScreen(u32 color)
    {
        reg(kGmc, 0x0Fu | (13u << 4) | (6u << 8) | (0xF0u << 16) | (2u << 24));
        reg(kDstOffset, 0);
        reg(kDstPitch, kPitch8);
        reg(kScTopLeft, 0);
        reg(kScBottomRight, (kW - 1u) | ((kH - 1u) << 16));
        reg(kDpWriteMsk, 0xFFFFFFFFu);
        reg(kBrushFrgd, color);
        reg(kDstYX, 0);
        reg(kDstHeightWidth, kW | (kH << 16)); // the store that draws
    }
    // What a program does in one frame here: read two words out of system
    // memory by bus mastering, paint a pixel itself, then wipe the screen.
    void frame(u32 word0, u32 color)
    {
        le32(0x1000, kGuiScratch0);
        le32(0x1004, 0x2000);
        le32(0x2000, word0);
        le32(0x2004, ~word0);
        reg(kBmAddr, 0x1000);
        fb(5 * kStride + 12, word0 ^ 0x00FF00FFu);
        fillScreen(color);
    }
};

fs::path recPath(const char* name)
{
    return fs::temp_directory_path() / name;
}

std::vector<u8> slurp(const fs::path& p)
{
    std::ifstream f(p, std::ios::binary);
    return {std::istreambuf_iterator<char>(f), {}};
}

} // namespace

TEST_CASE("r128 rec: a replayed recording leaves the card as the live one stood")
{
    const fs::path file = recPath("opm_rec_test.frec");
    Rig live;
    live.fillScreen(0x00102030u); // before the recording: part of its start
    std::string err;
    REQUIRE(r128RecStart(live.c, live.mem(), 2, file.string(), err));
    CHECK(r128RecOn());
    live.frame(0xAABBCCDDu, 0x00405060u);
    CHECK(r128RecStatus().frames == 1);
    CHECK(r128RecOn());
    live.frame(0x01020304u, 0x00708090u);
    // The second wipe was the second frame edge: the recording ended itself.
    CHECK(!r128RecOn());
    const R128RecStatus st = r128RecStatus();
    CHECK(st.finished);
    CHECK(st.frames == 2);
    CHECK(st.pages >= 2); // the descriptor's page and the data's
    CHECK(st.note == "complete");

    R128Cell again;
    std::vector<char> kinds;
    std::vector<u32> shownAtEdge;
    R128ReplayReport rep;
    REQUIRE(r128Replay(
        file.string(), again,
        [&](u32, char kind, const R128Cell& c) {
            kinds.push_back(kind);
            shownAtEdge.push_back((u32(c.vram[1]) << 16) |
                                  (u32(c.vram[2]) << 8) | c.vram[3]);
        },
        rep, err));
    CHECK(rep.edgesWanted == 2);
    CHECK(rep.edges == 2);
    CHECK(rep.edgesEqual == 2);
    CHECK(rep.endRecorded);
    CHECK(rep.endEqual);
    CHECK(again.vram == live.c.vram);
    // Each edge was heard BEFORE its wipe: the frame it ends is still shown.
    REQUIRE(kinds.size() == 2);
    CHECK(kinds[0] == 'C');
    CHECK(shownAtEdge[0] == 0x102030u);
    CHECK(shownAtEdge[1] == 0x405060u);
    // The words the engine read out of system memory came with the file.
    CHECK(again.peek(kGuiScratch0) == 0x01020304u);
    CHECK(again.peek(kGuiScratch0 + 4) == ~0x01020304u);
    CHECK(again.peek(kGuiScratch0) == live.c.peek(kGuiScratch0));

    // CONTROL: one color changed in the file. The first edge stands before
    // that color is drawn and still matches; the second shows it and must
    // not.
    std::vector<u8> bytes = slurp(file);
    const u8 pat[9] = {'W', 0x7C, 0x14, 0, 0, 0x00, 0x40, 0x50, 0x60};
    size_t hit = 0, hits = 0;
    for (size_t i = 0; i + 9 <= bytes.size(); ++i)
        if (!std::memcmp(&bytes[i], pat, 9)) {
            hit = i;
            ++hits;
        }
    REQUIRE(hits == 1);
    bytes[hit + 7] ^= 0x01;
    const fs::path bent = recPath("opm_rec_test_bent.frec");
    {
        std::ofstream f(bent, std::ios::binary);
        f.write(reinterpret_cast<const char*>(bytes.data()),
                static_cast<std::streamsize>(bytes.size()));
    }
    R128Cell wrong;
    R128ReplayReport bad;
    REQUIRE(r128Replay(bent.string(), wrong, R128EdgeFn{}, bad, err));
    CHECK(bad.edges == 2);
    CHECK(bad.edgesEqual == 1);
    fs::remove(file);
    fs::remove(bent);
}

TEST_CASE("r128 rec: a recording stopped early is a whole file")
{
    const fs::path file = recPath("opm_rec_test_short.frec");
    Rig live;
    std::string err;
    REQUIRE(r128RecStart(live.c, live.mem(), 5, file.string(), err));
    std::string second;
    CHECK(!r128RecStart(live.c, live.mem(), 1, file.string(), second));
    CHECK(second.find("already") != std::string::npos);
    live.frame(0x55667788u, 0x00111111u);
    r128RecStop(live.c, "stopped on request");
    CHECK(!r128RecOn());
    CHECK(r128RecStatus().frames == 1);
    CHECK(r128RecStatus().note == "stopped on request");
    R128Cell again;
    R128ReplayReport rep;
    REQUIRE(r128Replay(file.string(), again, R128EdgeFn{}, rep, err));
    CHECK(rep.edges == 1);
    CHECK(rep.edgesEqual == 1);
    CHECK(rep.endEqual);
    CHECK(again.vram == live.c.vram);
    fs::remove(file);
}

TEST_CASE("r128 rec: a file that is not a recording is refused, not replayed")
{
    const fs::path file = recPath("opm_rec_test_not.frec");
    {
        std::ofstream f(file, std::ios::binary);
        f << "this is not a recording";
    }
    R128Cell c;
    R128ReplayReport rep;
    std::string err;
    CHECK(!r128Replay(file.string(), c, R128EdgeFn{}, rep, err));
    CHECK(err.find("not a frame recording") != std::string::npos);
    fs::remove(file);
}
