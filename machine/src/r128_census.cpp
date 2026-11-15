// What a guest asks the card for and does not get: see "The census" in
// opm/r128.hpp. This file holds the table of multi-valued settings and the two
// reports the diagnostics print.

#include "opm/r128.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <map>
#include <mutex>
#include <string>
#include <tuple>
#include <vector>

namespace opm {

u32 gR128SettingSeen[static_cast<size_t>(R128Setting::Count)] = {};
bool gR128TexCensusOn = false;

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
    "zero|one|source color|1 - source color|source alpha|1 - source alpha|"
    "destination alpha|1 - destination alpha|destination color|"
    "1 - destination color|source alpha saturate|both source alpha|"
    "both 1 - source alpha";
constexpr const char* kCombine =
    "off|copy|copy input|modulate|modulate x2|modulate x4|add|add signed|"
    "blend by vertex alpha|blend by texture alpha|blend by constant alpha|"
    "blend premultiplied|blend by previous|blend premultiplied, inverse|"
    "add signed x2|blend by constant color";
constexpr const char* kCombineAlpha =
    "off|copy|copy input|modulate|modulate x2|modulate x4|add|add signed|"
    "||||||add signed x2";
constexpr const char* kMinFilter =
    "nearest|linear|nearest map, nearest|nearest map, linear|"
    "two maps, nearest|trilinear";
constexpr const char* kClamp = "wrap|mirror|clamp|border color";
constexpr const char* kTexFormat =
    "VQ|CI4|CI8|ARGB1555|RGB565|RGB888|ARGB8888|RGB332|Y8|RGB8|aCI88|"
    "YVYU422|VYUY422|alpha and gray 8:8|AYUV444|ARGB4444";
constexpr const char* kColorFactor =
    "constant color|1 - constant color|||texture color|"
    "1 - texture color|texture alpha|1 - texture alpha|previous color";
constexpr const char* kInputFactor =
    "||constant color|constant alpha|vertex color|vertex alpha|||"
    "previous color|previous alpha";
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
    {"2D brush", bit(0) | bit(1) | bit(13) | bit(15),
     "8x8 mono|8x8 mono, see-through|8x1 mono|8x1 mono, see-through|"
     "1x8 mono|1x8 mono, see-through|32x1 mono|32x1 mono, see-through|"
     "32x32 mono|32x32 mono, see-through|8x8 color|8x1 color|1x8 color|"
     "solid||solid, from the packet parser"},
    {"2D source format", bit(0) | bit(1) | bit(3),
     "mono|mono, see-through||color"},
    {"2D source, through the registers", bit(2) | bit(3) | bit(4),
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
    // modeled is stricter than the chip toward this sequence, and the
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
    {"texture coordinates", upTo(1),
     "plain, for the chip to multiply by W|already multiplied by W"},
    {"shading", upTo(3), "constant color|flat|Gouraud|Gouraud"},
    {"back faces", upTo(3), kFace},
    {"front faces", upTo(3), kFace},
    // TEX_CNTL_C bit 21. Nothing says what the card does with it, so "on"
    // is not drawn; no draw of either game studied has it on.
    {"texture anti-alias bit", bit(0), "off|on"},
    // CLR_CMP_FCN_3D (guide, MISC_3D_STATE_CNTL_REG 31:30): the texel that
    // equals the key color is not drawn, whatever this says.
    {"texture color key", bit(3),
     "never|always|where the texel differs from the key|"
     "where the texel equals the key"},
    {"edge anti-aliasing", bit(0), "off|on"},
    {"supersampling", bit(0), "off|on"},
    // Table 6-15 has no state 11 or 13.
    {"lighting, color", upTo(15) & ~(bit(11) | bit(13)), kCombine},
    {"lighting, alpha", upTo(7), kCombineAlpha},
    // ── the two texture units ────────────────────────────────────────────
#define OPM_UNIT(N)                                                           \
    {"texture " N " minify filter", upTo(5), kMinFilter},                     \
    /* Table 6-5 names six states: the even ones nearest, the odd linear. */  \
    {"texture " N " magnify filter", upTo(5),                                 \
     "nearest|linear|nearest|linear|nearest|linear"},                         \
    {"texture " N " edge, across", upTo(3), kClamp},                          \
    {"texture " N " edge, down", upTo(3), kClamp},                            \
    {"texture " N " format", upTo(15) & ~bit(0), kTexFormat},                 \
    /* The palette is used as it was loaded, whatever this says. */           \
    {"texture " N " palette entries", 0, kPalEntries},                        \
    {"texture " N " color combine", upTo(15), kCombine},                     \
    /* r128_reg.h's "1 - constant color" is drawn as the texture color. */  \
    {"texture " N " color factor",                                           \
     bit(0) | bit(4) | bit(5) | bit(6) | bit(7) | bit(8), kColorFactor},      \
    {"texture " N " color input",                                            \
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

// ── The texture census ────────────────────────────────────────────────────
//
// Every distinct texture a packet of primitives is drawn with, and what can
// be read off its own bytes about how they are laid down. A 16-bit texel is
// two bytes and nothing in the stream says which is the high one; a picture,
// though, is smoother read the right way round, because the wrong way puts
// the low bits of green beside the high bits of red. So the top level's
// first rows are decoded both ways and the channel differences between
// horizontal neighbors are summed for each: the smaller sum is the order the
// texture was written in. A flat texture says nothing and is listed so.
namespace {

struct TexKey {
    u32 unit, where, base, w, h, fmt; // where: 0 video memory, 1 AGP space
    bool operator<(const TexKey& o) const
    {
        return std::tie(unit, where, base, w, h, fmt) <
               std::tie(o.unit, o.where, o.base, o.w, o.h, o.fmt);
    }
};
struct TexFacts {
    u64 packets = 0, order = 0;
    u32 texCntl = 0, unitCntl = 0, dpDatatype = 0;
    u64 roughHighFirst = 0, roughLowFirst = 0;
    // 32-bit texels: how many have 0x00 or 0xFF as their first byte and as
    // their last, which is where an alpha channel shows.
    u64 flatFirst = 0, flatLast = 0, texels = 0;
    // The whole top level: how many of its bytes are not zero. A texture
    // that was never filled draws black and looks like a lighting fault.
    u64 bytes = 0, bytesSet = 0;
    bool unread = false;
    // The top level as the pipeline decodes it, three bytes a texel, when
    // pictures were asked for.
    std::vector<u8> rgb;
};
bool gTexPictures = false;
std::mutex gTexLock;
std::map<TexKey, TexFacts> gTexSeen;

constexpr u8 kTexelBytes[16] = {1, 1, 1, 2, 2, 3, 4, 1, 1, 1, 2, 2, 2, 2, 4, 2};

void texRoughness(const R128TexSeen& t, TexFacts& f)
{
    const u32 rows = t.h < 32u ? t.h : 32u;
    const u32 cols = t.w < 64u ? t.w : 64u;
    const u32 bytes = t.fmt == 6u ? 4u : 2u;
    for (u32 y = 0; y < rows; ++y) {
        u32 prev[2] = {0, 0};
        for (u32 x = 0; x < cols; ++x) {
            u8 b[4] = {0, 0, 0, 0};
            const u32 at = t.base + (y * t.pitch + x) * bytes;
            for (u32 k = 0; k < bytes; ++k)
                if (!t.byteAt(t.ctx, at + k, b[k])) {
                    f.unread = true;
                    return;
                }
            ++f.texels;
            if (bytes == 4u) {
                f.flatFirst += b[0] == 0u || b[0] == 0xFFu;
                f.flatLast += b[3] == 0u || b[3] == 0xFFu;
                continue;
            }
            const u32 v[2] = {(static_cast<u32>(b[0]) << 8) | b[1],
                              (static_cast<u32>(b[1]) << 8) | b[0]};
            if (x) {
                for (int o = 0; o < 2; ++o) {
                    u32 d = 0;
                    const auto ch = [&](u32 shift, u32 mask) {
                        const int a0 = static_cast<int>((v[o] >> shift) & mask);
                        const int a1 =
                            static_cast<int>((prev[o] >> shift) & mask);
                        d += static_cast<u32>(a0 > a1 ? a0 - a1 : a1 - a0);
                    };
                    if (t.fmt == 4u) { // 565: green halved to weigh as red
                        ch(11, 31);
                        ch(6, 31);
                        ch(0, 31);
                    } else if (t.fmt == 15u) {
                        ch(12, 15);
                        ch(8, 15);
                        ch(4, 15);
                        ch(0, 15);
                    } else {
                        ch(10, 31);
                        ch(5, 31);
                        ch(0, 31);
                    }
                    (o ? f.roughLowFirst : f.roughHighFirst) += d;
                }
            }
            prev[0] = v[0];
            prev[1] = v[1];
        }
    }
}

// A verdict needs a margin: a fifth less rough one way than the other.
int texVerdict(const TexFacts& f)
{
    if (f.unread)
        return 3;
    const u64 hi = f.roughHighFirst, lo = f.roughLowFirst;
    if (hi * 5u < lo * 4u)
        return 0; // high byte first
    if (lo * 5u < hi * 4u)
        return 1; // low byte first
    return 2;     // no telling
}

} // namespace

void r128TextureSeen(const R128TexSeen& t)
{
    std::lock_guard<std::mutex> lock(gTexLock);
    const TexKey key{t.unit, t.agp ? 1u : 0u, t.base, t.w, t.h, t.fmt};
    auto [it, fresh] = gTexSeen.try_emplace(key);
    TexFacts& f = it->second;
    ++f.packets;
    if (!fresh)
        return;
    f.order = gTexSeen.size();
    f.texCntl = t.texCntl;
    f.unitCntl = t.unitCntl;
    f.dpDatatype = t.dpDatatype;
    if (t.fmt == 3u || t.fmt == 4u || t.fmt == 15u || t.fmt == 6u)
        texRoughness(t, f);
    const u32 n = kTexelBytes[t.fmt & 15u];
    for (u32 y = 0; y < t.h && !f.unread; ++y)
        for (u32 x = 0; x < t.w * n; ++x) {
            u8 b = 0;
            if (!t.byteAt(t.ctx, t.base + y * t.pitch * n + x, b)) {
                f.unread = true;
                break;
            }
            ++f.bytes;
            f.bytesSet += b != 0u;
        }
    if (!gTexPictures || f.unread)
        return;
    f.rgb.assign(static_cast<size_t>(t.w) * t.h * 3u, 0);
    for (u32 y = 0; y < t.h; ++y)
        for (u32 x = 0; x < t.w; ++x)
            t.texelAt(t.ctx, x, y,
                      &f.rgb[(static_cast<size_t>(y) * t.w + x) * 3u]);
}

void r128TextureCensus(bool on, bool pictures)
{
    std::lock_guard<std::mutex> lock(gTexLock);
    gR128TexCensusOn = on;
    gTexPictures = on && pictures;
    gTexSeen.clear();
}

std::vector<R128TexturePicture> r128TexturePictures()
{
    std::lock_guard<std::mutex> lock(gTexLock);
    std::vector<R128TexturePicture> out;
    for (const auto& [k, f] : gTexSeen) {
        if (f.rgb.empty())
            continue;
        char name[96];
        std::snprintf(name, sizeof name, "%03llu-unit%u-%s-%08x-%ux%u-fmt%u",
                      static_cast<unsigned long long>(f.order), k.unit + 1u,
                      k.where ? "agp" : "vram", k.base, k.w, k.h, k.fmt);
        out.push_back({name, k.w, k.h, f.rgb});
    }
    return out;
}

std::string r128TextureReport()
{
    std::lock_guard<std::mutex> lock(gTexLock);
    std::string out = "-- textures drawn with, and the byte order a 16-bit "
                      "one reads smoother in:\n";
    if (gTexSeen.empty())
        return out + "--   none (or the census is off)\n";
    static const char* const kWhere[2] = {"video memory", "AGP space"};
    static const char* const kVerdict[4] = {"high byte first",
                                            "low byte first", "no telling",
                                            "not readable"};
    // The totals first, split by where the texture lies and by the control
    // bits a driver might be choosing the order with.
    std::map<std::tuple<u32, u32, u32, u32, int>, u32> totals;
    for (const auto& [k, f] : gTexSeen) {
        if (k.fmt == 6u)
            continue;
        ++totals[{k.where, (f.texCntl >> 21) & 1u, (f.dpDatatype >> 29) & 1u,
                  (f.unitCntl >> 15) & 1u, texVerdict(f)}];
    }
    char b[280];
    for (const auto& [k, n] : totals) {
        std::snprintf(b, sizeof b,
                      "--   %-12s TEX_CNTL_C.21=%u DP_DATATYPE.29=%u "
                      "unit cntl.15=%u: %4u 16-bit texture(s) %s\n",
                      kWhere[std::get<0>(k)], std::get<1>(k), std::get<2>(k),
                      std::get<3>(k), n, kVerdict[std::get<4>(k)]);
        out += b;
    }
    std::vector<std::pair<TexKey, const TexFacts*>> rows;
    for (const auto& [k, f] : gTexSeen)
        rows.push_back({k, &f});
    std::sort(rows.begin(), rows.end(), [](const auto& x, const auto& y) {
        return x.second->order < y.second->order;
    });
    size_t shown = 0;
    for (const auto& [k, fp] : rows) {
        const TexFacts& f = *fp;
        if (++shown > 48u) {
            std::snprintf(b, sizeof b, "--   ... and %zu more\n",
                          rows.size() - 48u);
            out += b;
            break;
        }
        const unsigned long long set =
            f.bytes ? f.bytesSet * 100u / f.bytes : 0u;
        if (k.fmt == 6u)
            std::snprintf(b, sizeof b,
                          "--   unit %u %-12s %08x %4ux%-4u fmt %2u, %6llu "
                          "packet(s), %3llu%% of its bytes set: of %llu "
                          "texels 00/FF in the first byte %llu, in the last "
                          "%llu\n",
                          k.unit + 1u, kWhere[k.where], k.base, k.w, k.h,
                          k.fmt, static_cast<unsigned long long>(f.packets),
                          set, static_cast<unsigned long long>(f.texels),
                          static_cast<unsigned long long>(f.flatFirst),
                          static_cast<unsigned long long>(f.flatLast));
        else
            std::snprintf(b, sizeof b,
                          "--   unit %u %-12s %08x %4ux%-4u fmt %2u, %6llu "
                          "packet(s), %3llu%% of its bytes set: rough %llu "
                          "high first, %llu low first: %s\n",
                          k.unit + 1u, kWhere[k.where], k.base, k.w, k.h,
                          k.fmt, static_cast<unsigned long long>(f.packets),
                          set,
                          static_cast<unsigned long long>(f.roughHighFirst),
                          static_cast<unsigned long long>(f.roughLowFirst),
                          kVerdict[texVerdict(f)]);
        out += b;
    }
    return out;
}

} // namespace opm
