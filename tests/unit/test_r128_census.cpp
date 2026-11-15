// The census of what a guest asks the card for and does not get: control
// bits set outside what the model decodes, and values of multi-valued
// settings that draws used and the model draws some other way. The checks
// read the reports the diagnostics print.

#include "doctest.h"
#include "opm/r128.hpp"

#include <string>

using namespace opm;

namespace {

u32 be(u32 v)
{
    return (v >> 24) | ((v >> 8) & 0xFF00u) | ((v << 8) & 0xFF0000u) |
           (v << 24);
}
void wr(R128Cell& c, u32 off, u32 v) { c.write(off, be(v), 4); }

constexpr u32 kDstYX = 0x1438, kDstHeightWidth = 0x143C;
constexpr u32 kGmc = 0x146C, kBrushFrgd = 0x147C, kDpCntl = 0x16C0;
constexpr u32 kDefaultOffset = 0x16E0, kDefaultPitch = 0x16E4;
constexpr u32 kDefaultScBr = 0x16E8;

// A 4x3 fill with the given brush type, the 32 bpp destination, PATCOPY.
void fill(R128Cell& c, u32 brush, u32 extra = 0)
{
    wr(c, kDefaultOffset, 0);
    wr(c, kDefaultPitch, 8);
    wr(c, kDefaultScBr, 63u | (63u << 16));
    wr(c, kGmc, (brush << 4) | (6u << 8) | (3u << 12) | (0xF0u << 16) |
                    (2u << 24) | extra);
    wr(c, kBrushFrgd, 0x00445566u);
    wr(c, kDstYX, 0);
    wr(c, kDstHeightWidth, 4u | (3u << 16));
}

bool has(const std::string& text, const char* what)
{
    return text.find(what) != std::string::npos;
}

// The part of the settings report that lists what is not drawn.
std::string notDrawn(const std::string& report)
{
    return report.substr(0, report.find("-- every setting drawn with"));
}

} // namespace

TEST_CASE("r128 census: a control bit outside the decode mask is reported")
{
    r128RegEverReset();
    R128Cell c;
    wr(c, kDpCntl, 0x00000003u); // the two directions: both decoded
    CHECK(has(r128UndecodedReport(c), "none, in the registers that are audited"));
    wr(c, kDpCntl, 0x0000000Bu); // ...and DST_X_TILE, which is not
    const std::string r = r128UndecodedReport(c);
    CHECK(has(r, "16c0 DP_CNTL"));
    CHECK(has(r, "NOT DECODED: 00000008"));
    CHECK(!has(r, "none, in the registers that are audited"));
    r128RegEverReset();
}

TEST_CASE("r128 census: the registers audited in the second pass have masks")
{
    // Each of these printed as "not audited" before; a mask of zero would
    // report every bit a guest sets.
    for (u32 off : {0x146Cu, 0x16C0u, 0x16C4u, 0x1A00u, 0x1BC4u, 0x1C80u,
                    0x1C84u, 0x1C88u, 0x1C8Cu}) {
        u32 mask = 0;
        CAPTURE(off);
        CHECK(r128RegDecodedBits(off, mask));
        CHECK(mask != 0u);
    }
    u32 gmc = 0, gmcC = 0;
    REQUIRE(r128RegDecodedBits(0x146Cu, gmc));
    REQUIRE(r128RegDecodedBits(0x1C84u, gmcC));
    CHECK(gmc == gmcC); // one control under two names
}

TEST_CASE("r128 census: a setting's value is named when a draw uses it, and "
          "marked when the model does not draw it")
{
    r128SettingsReset();
    CHECK(has(r128SettingsReport(), "nothing has been drawn"));
    R128Cell c;
    fill(c, 13u, 1u << 27); // a solid brush, the 3D function left alone
    std::string r = r128SettingsReport();
    CHECK(has(r, "2D brush"));
    CHECK(has(r, "solid (13)"));
    CHECK(has(r, "32 bpp (6)"));
    CHECK(has(notDrawn(r), "none"));
    CHECK(!has(r, "NOT DRAWN"));

    // An 8x8 color pattern brush: the engine declines the fill, and the
    // report says what was asked for.
    const u64 declined = r128EngStats().brushUnimpl;
    fill(c, 10u, 1u << 27);
    CHECK(r128EngStats().brushUnimpl == declined + 1);
    r = r128SettingsReport();
    CHECK(has(notDrawn(r), "2D brush"));
    CHECK(has(notDrawn(r), "8x8 color (10)"));
    CHECK(!has(notDrawn(r), "solid (13)")); // drawn: not in the short list
    CHECK(has(r, "8x8 color (10) NOT DRAWN"));

    // A value of ZERO is a value like any other: the master control's bit
    // 27 was written as one so far, and only that is listed.
    CHECK(has(r, "leave it (1)"));
    CHECK(!has(r, "clear it (0)"));
    fill(c, 13u, 0);
    r = r128SettingsReport();
    CHECK(has(r, "clear it (0); leave it (1)"));

    r128SettingsReset();
    CHECK(has(r128SettingsReport(), "nothing has been drawn"));
}
