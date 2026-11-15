// What a guest asks the card for and does not get: see "The census" in
// opm/r128.hpp. This file holds the table of multi-valued settings and the two
// reports the diagnostics print.

#include "opm/r128.hpp"

#include <cstdio>
#include <cstring>
#include <string>

namespace opm {

u32 gR128SettingSeen[static_cast<size_t>(R128Setting::Count)] = {};

namespace {

// One multi-valued setting: what it is called, which of its values this
// model draws as the manuals describe (bit k = value k), and the values'
// names in order, '|' between them (an empty name = the manuals give none).
//
// ⚠ `drawn` IS A SECOND STATEMENT OF WHAT THE DECODERS DO, like the bit
// masks: change a decoder, change its line here. Sources for the names: the
// SDK's chapter 6 tables (6-3 to 6-26), the register guide's
// DP_GUI_MASTER_CNTL, and r128_reg.h for the fields the books leave out.
struct Setting {
    const char* name;
    u32 drawn;
    const char* values;
};

constexpr const char* kCompare =
    "never|less|less or equal|equal|greater or equal|greater|not equal|always";
constexpr const char* kStencilOp =
    "keep|zero|replace|increment|decrement|invert";
constexpr const char* kBlend =
    "zero|one|source colour|1 - source colour|source alpha|1 - source alpha|"
    "destination alpha|1 - destination alpha|destination colour|"
    "1 - destination colour|source alpha saturate|both source alpha|"
    "both 1 - source alpha";
constexpr const char* kCombine =
    "off|copy|copy input|modulate|modulate x2|modulate x4|add|add signed|"
    "blend by vertex alpha|blend by texture alpha|blend by constant alpha|"
    "blend premultiplied|blend by previous|blend premultiplied, inverse|"
    "add signed x2|blend by constant colour";
constexpr const char* kCombineAlpha =
    "off|copy|copy input|modulate|modulate x2|modulate x4|add|add signed|"
    "||||||add signed x2";
constexpr const char* kMinFilter =
    "nearest|linear|nearest map, nearest|nearest map, linear|"
    "two maps, nearest|trilinear";
constexpr const char* kClamp = "wrap|mirror|clamp|border colour";
constexpr const char* kTexFormat =
    "VQ|CI4|CI8|ARGB1555|RGB565|RGB888|ARGB8888|RGB332|Y8|RGB8|aCI88|"
    "YVYU422|VYUY422|alpha and grey 8:8|AYUV444|ARGB4444";
constexpr const char* kColorFactor =
    "constant colour|1 - constant colour|||texture colour|"
    "1 - texture colour|texture alpha|1 - texture alpha|previous colour";
constexpr const char* kInputFactor =
    "||constant colour|constant alpha|vertex colour|vertex alpha|||"
    "previous colour|previous alpha";
constexpr const char* kAlphaFactor = "||||||texture alpha|1 - texture alpha";
constexpr const char* kInputFactorA =
    "|constant alpha|vertex alpha||previous alpha";
constexpr const char* kPalEntries = "RGB565|ARGB1555|ARGB4444";
constexpr const char* kFace = "cull|points|lines|solid";

constexpr u32 bit(u32 k) { return 1u << k; }
constexpr u32 upTo(u32 k) { return (1u << (k + 1u)) - 1u; } // values 0..k

// In R128Setting's order.
constexpr Setting kSettings[] = {
    // ── the 2D engine (engBlit) ──────────────────────────────────────────
    {"2D destination format", upTo(9) & ~3u,
     "||8 bpp palette|15 bpp|16 bpp|24 bpp|32 bpp|8 bpp RGB332|Y8|RGB8||"
     "VYUY422|YVYU422||AYUV444|ARGB4444"},
    {"2D brush", bit(13) | bit(15),
     "8x8 mono|8x8 mono, see-through|8x1 mono|8x1 mono, see-through|"
     "1x8 mono|1x8 mono, see-through|32x1 mono|32x1 mono, see-through|"
     "32x32 mono|32x32 mono, see-through|8x8 colour|8x1 colour|1x8 colour|"
     "solid||solid, from the packet parser"},
    {"2D source format", bit(3), "mono|mono, see-through||colour"},
    {"2D source, through the registers", bit(2),
     "||video memory|host data|host data, byte aligned"},
    {"2D rectangle tiling", bit(0), "off|across|down|across and down"},
    // DP_GUI_MASTER_CNTL bit 27, at the write. Zero is an ACTION: clear the
    // 3D function and the depth and stencil enables. It is not taken, and
    // counted as drawn all the same: nothing the 2D engine draws reads those,
    // and the Mac driver, which asks for it with every 2D operation, sets
    // them again before it draws in 3D. MEASURED on six recorded scenes of
    // two games: taking the action through this model's write gate
    // (r128Eng3dWrite) changed no picture, and refused 16 to 20 of the
    // driver's own 3D register writes per three frames, because each burst
    // carries the depth and texture registers ahead of the function. A
    // driver that ships cannot be losing those on the chip, so the gate as
    // modelled is stricter than the chip toward this sequence, and the
    // action stays out until something says what the chip does there.
    {"2D master control, on the 3D function", bit(0) | bit(1),
     "clear it|leave it"},
    // ── the 3D pipeline (gatherPipe) ─────────────────────────────────────
    {"3D destination format", bit(3) | bit(4) | bit(6),
     "||8 bpp palette|15 bpp|16 bpp|24 bpp|32 bpp|8 bpp RGB332|Y8|RGB8"},
    {"depth buffer width", upTo(2), "16 bit|24 bit|32 bit"},
    {"depth test", upTo(7), kCompare},
    {"stencil test", upTo(7), kCompare},
    {"stencil, on stencil fail", upTo(5), kStencilOp},
    {"stencil, on depth pass", upTo(5), kStencilOp},
    {"stencil, on depth fail", upTo(5), kStencilOp},
    {"blend, source factor", upTo(12), kBlend},
    {"blend, destination factor", upTo(10), kBlend},
    {"blend, how the two are combined", upTo(3),
     "add, clamp|add|subtract, clamp|subtract"},
    {"alpha test", upTo(7), kCompare},
    // The manual gives no table for the second method: it is drawn as if
    // dithering were off (r128_3d.cpp, "Dithering").
    {"dither method", bit(0), "error diffusion|table"},
    {"dither, error at the start of a line", upTo(1), "kept|reset"},
    {"vertex position precision", upTo(1),
     "quarter pixel|sixteenth of a pixel"},
    {"texture coordinates", bit(1), "multiplied by W|as given"},
    {"shading", upTo(3), "constant colour|flat|Gouraud|Gouraud"},
    {"back faces", upTo(3), kFace},
    {"front faces", upTo(3), kFace},
    // TEX_CNTL_C bit 21. Nothing says what the card does with it, so "on"
    // is not drawn; no draw of either game studied has it on.
    {"texture anti-alias bit", bit(0), "off|on"},
    // CLR_CMP_FCN_3D (guide, MISC_3D_STATE_CNTL_REG 31:30): the texel that
    // equals the key colour is not drawn, whatever this says.
    {"texture colour key", bit(3),
     "never|always|where the texel differs from the key|"
     "where the texel equals the key"},
    {"edge anti-aliasing", bit(0), "off|on"},
    {"supersampling", bit(0), "off|on"},
    // Table 6-15 has no state 11 or 13.
    {"lighting, colour", upTo(15) & ~(bit(11) | bit(13)), kCombine},
    {"lighting, alpha", upTo(7), kCombineAlpha},
    // ── the two texture units ────────────────────────────────────────────
#define OPM_UNIT(N)                                                           \
    {"texture " N " minify filter", upTo(5), kMinFilter},                     \
    /* Table 6-5 names six states: the even ones nearest, the odd linear. */  \
    {"texture " N " magnify filter", upTo(5),                                 \
     "nearest|linear|nearest|linear|nearest|linear"},                         \
    {"texture " N " edge, across", upTo(3), kClamp},                          \
    {"texture " N " edge, down", upTo(3), kClamp},                            \
    {"texture " N " format", upTo(15) & ~(bit(0) | bit(10) | bit(13)),        \
     kTexFormat},                                                             \
    /* The palette is used as it was loaded, whatever this says. */           \
    {"texture " N " palette entries", 0, kPalEntries},                        \
    {"texture " N " colour combine", upTo(15), kCombine},                     \
    /* r128_reg.h's "1 - constant colour" is drawn as the texture colour. */  \
    {"texture " N " colour factor",                                           \
     bit(0) | bit(4) | bit(5) | bit(6) | bit(7) | bit(8), kColorFactor},      \
    {"texture " N " colour input",                                            \
     bit(2) | bit(3) | bit(4) | bit(5) | bit(8) | bit(9), kInputFactor},      \
    {"texture " N " alpha combine", upTo(7) | bit(14), kCombineAlpha},        \
    {"texture " N " alpha factor", bit(6) | bit(7), kAlphaFactor},            \
    {"texture " N " alpha input", bit(1) | bit(2) | bit(4), kInputFactorA}
    OPM_UNIT("1"),
    OPM_UNIT("2"),
#undef OPM_UNIT
};
static_assert(sizeof kSettings / sizeof kSettings[0] ==
                  static_cast<size_t>(R128Setting::Count),
              "kSettings and R128Setting must list the same settings");

std::string valueName(const Setting& s, u32 value)
{
    const char* p = s.values;
    for (u32 k = 0; k < value && p; ++k) {
        p = std::strchr(p, '|');
        if (p)
            ++p;
    }
    std::string name;
    if (p) {
        const char* e = std::strchr(p, '|');
        name.assign(p, e ? static_cast<size_t>(e - p) : std::strlen(p));
    }
    char num[16];
    std::snprintf(num, sizeof num, name.empty() ? "value %u" : " (%u)", value);
    return name.empty() ? std::string(num) + ", which the manuals do not name"
                        : name + num;
}

} // namespace

void r128SettingsReset()
{
    std::memset(gR128SettingSeen, 0, sizeof gR128SettingSeen);
}

std::string r128SettingsReport()
{
    std::string missing, all;
    char line[160];
    for (size_t i = 0; i < static_cast<size_t>(R128Setting::Count); ++i) {
        const Setting& s = kSettings[i];
        const u32 seen = gR128SettingSeen[i];
        if (!seen)
            continue;
        std::string every, lacking;
        for (u32 v = 0; v < 32; ++v) {
            if (!(seen & bit(v)))
                continue;
            const bool drawn = (s.drawn & bit(v)) != 0;
            const std::string nm = valueName(s, v);
            every += (every.empty() ? "" : "; ") + nm +
                     (drawn ? "" : " NOT DRAWN");
            if (!drawn)
                lacking += (lacking.empty() ? "" : "; ") + nm;
        }
        std::snprintf(line, sizeof line, "--   %-34s ", s.name);
        all += line + every + "\n";
        if (!lacking.empty())
            missing += line + lacking + "\n";
    }
    std::string out =
        "-- settings drawn with that this model does not draw as the manuals "
        "say:\n";
    out += missing.empty() ? "--   none\n" : missing;
    out += "-- every setting drawn with, by value:\n";
    out += all.empty() ? "--   nothing has been drawn\n" : all;
    return out;
}

std::string r128UndecodedReport(const R128Cell& a)
{
    std::string miss, unaudited;
    char b[200];
    // The second texture unit's registers are written with defaults by a
    // driver that never switches the unit on (TEX_CNTL_C bit 5); what they
    // hold then asks for nothing.
    const bool secOn = (r128RegEverSet(0x1C9Cu) & 0x20u) != 0;
    for (const auto& [off, n] : a.writeCount) {
        (void)n;
        if (off < 0x0700u || off > 0x1DFFu)
            continue;
        if (!secOn && off >= 0x1D00u && off <= 0x1D30u)
            continue;
        const u32 ever = r128RegEverSet(off);
        u32 mask = 0;
        const char* nm = r128RegName(off);
        if (r128RegDecodedBits(off, mask)) {
            if (ever & ~mask) {
                std::snprintf(b, sizeof b,
                              "--   %04x %-28s set %08x, of which NOT "
                              "DECODED: %08x\n",
                              off, nm ? nm : "(unnamed)", ever, ever & ~mask);
                miss += b;
            }
        } else if (off >= 0x1A00u && ever) {
            std::snprintf(b, sizeof b, " %04x=%08x", off, ever);
            unaudited += b;
        }
    }
    std::string out = "-- control bits the guest set that this model does "
                      "not decode:\n";
    out += miss.empty() ? "--   none, in the registers that are audited\n"
                        : miss;
    if (!unaudited.empty())
        out += "--   not audited (bits ever set):" + unaudited + "\n";
    return out;
}

} // namespace opm
