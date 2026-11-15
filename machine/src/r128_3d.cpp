// The Rage 128's 3D pipeline: the two primitive packets, the vertex walker,
// and a software rasterizer over the state the guest programs through the
// CCE context registers.
//
// Sources, and the clean-room boundary they sit inside: packet formats and
// render-state semantics are SDK-G04000 App F.24-F.27 and ch 6.6 (r128sdk.txt
// 6160-7500, 14572-15100); register offsets and bit positions are the Linux
// DRM's r128_reg.h and the DRI driver's observed programming order, which are
// facts about the hardware. No code is ported from either.
//
// The pipeline, in the order a fragment travels it: FTLVERTEX decode (the
// flexible screen-space vertex — the guest does T&L on the CPU, so there is
// NO transform stage here; ⚠ its S,T arrive PREMULTIPLIED by rhw — see the
// measurement at rasterTriangle's plane setup) → triangle setup with the D3D
// top-left fill rule → per-pixel interpolation (colors screen-linear,
// textures resolved by the per-pixel divide through rhw) → two
// texture-combine units → texture lighting → specular add → fog (vertex or
// table) → chroma key → alpha test → stencil → Z → alpha blend → plane mask
// → destination format.
//
// ⚠ CONVENTIONS THIS FILE MUST AGREE WITH, all inherited from the 2D engine:
// a pixel in VRAM is stored MOST SIGNIFICANT BYTE FIRST (the big-endian
// aperture the Mac paints through; getting this wrong once made a gray
// desktop olive), the Z buffer uses the same byte order because the guest
// CLEARS Z with 2D PAINT ops through the same store path, and every counter
// lives in a file-static so sizeof(R128Cell) — and with it every snapshot —
// never moves. A texture in AGP space is the exception to the first: its
// texels are least significant byte first (texelLowFirst).
//
// Known simplifications, each visible rather than silent: dithering rounds
// instead of patterning (DITHER_ENABLE is honored as "convert with
// rounding"; the two documented dither algorithms are not reproduced),
// error-diffusion state is not modeled, tiled textures and Z (bit flags
// observed) are counted and read as linear, and VQ/CI16 texel formats
// decline. Lines use a DDA rather than the hardware's undocumented exact
// rule; adjacency of LINES is not pixel-exact against real silicon.

#include "opm/bus.hpp"
#include "opm/r128.hpp"
#include "opm/r128rec.hpp"
#include "opm/snapshot.hpp"

#include <atomic>
#include <chrono>
#include <climits>
#include <cmath>
#include <condition_variable>
// The fragment path works on a pixel's four channels at once (F4, below):
// with SSE2 where the target has it, which every x86-64 does, and as plain
// loops elsewhere. Lane by lane the two are the same IEEE operations in the
// same order, so they draw the same picture.
#if !defined(OPM_F4_SCALAR) && \
    (defined(__SSE2__) || defined(_M_X64) || defined(_M_AMD64))
#define OPM_F4_SSE 1
#include <emmintrin.h>
#else
#define OPM_F4_SSE 0
#endif
#if defined(_MSC_VER)
#define OPM_FORCE_INLINE __forceinline
// "marked as __forceinline not inlined": the request is a hint about speed,
// and a compiler that declines it has still built the right program.
#pragma warning(disable : 4714)
// "structure was padded due to alignment specifier": the row queue's lanes
// are a processor cache line apart on purpose.
#pragma warning(disable : 4324)
#else
#define OPM_FORCE_INLINE inline __attribute__((always_inline))
#endif
#if defined(_MSC_VER)
#define OPM_NOINLINE __declspec(noinline)
#else
#define OPM_NOINLINE __attribute__((noinline))
#endif
#if defined(__x86_64__) || defined(_M_X64) || defined(_M_AMD64)
#define OPM_ROWS_X86 1
#include <xmmintrin.h>
#else
#define OPM_ROWS_X86 0
#endif
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

namespace opm {

// The stats object lives in r128.cpp; both halves of the engine count into
// the same struct so the report is one table.
R128EngStats& r128EngStatsMut();

namespace {

// ── Registers this unit reads (offsets from r128_reg.h; layouts per the
// SDK's field tables where the books redact them) ─────────────────────────
constexpr u32 kFpuSetup = 0x071C;        // PM4_VC_FPU_SETUP
constexpr u32 kScale3dCntl = 0x1A00;     // SCALE_3D_CNTL
constexpr u32 kSetupCntl = 0x1BC4;       // SETUP_CNTL
constexpr u32 kFogTableIndex = 0x1A14;   // SDK ch 6 example's own offsets
constexpr u32 kFogTableData = 0x1A18;
constexpr u32 kWindowXyOffset = 0x1BCC;  // x 31:20, y 15:4
constexpr u32 kZOffsetC = 0x1C90;
constexpr u32 kZPitchC = 0x1C94;         // pitch/8 in 9:0, Z_TILE bit 16
constexpr u32 kZStenCntlC = 0x1C98;
constexpr u32 kTexCntlC = 0x1C9C;
constexpr u32 kMisc3dStateCntl = 0x1CA0; // REF_ALPHA 7:0, SCALE_3D_FN 9:8,
                                         // ALPHA_COMB 13:12, FOG_TABLE 14,
                                         // BLND_SRC 19:16, BLND_DST 23:20,
                                         // ALPHA_TEST 26:24
constexpr u32 kTexClrCmpClrC = 0x1CA4;
constexpr u32 kTexClrCmpMskC = 0x1CA8;
constexpr u32 kFogColorC = 0x1CAC;       // RGB 888
constexpr u32 kPrimTexCntlC = 0x1CB0;
constexpr u32 kPrimTexCombineCntlC = 0x1CB4;
constexpr u32 kTexSizePitchC = 0x1CB8;   // unit0 15:0, unit1 31:16
constexpr u32 kPrimTex0OffsetC = 0x1CBC; // ..10 at +4 each
constexpr u32 kSecTexCntlC = 0x1D00;
constexpr u32 kSecTexCombineCntlC = 0x1D04;
constexpr u32 kSecTex0OffsetC = 0x1D08;  // ..10 at +4 each
constexpr u32 kConstantColorC = 0x1D34;  // A 31:24, R 23:16, G 15:8, B 7:0
constexpr u32 kPrimTexBorderColorC = 0x1D38;
constexpr u32 kSecTexBorderColorC = 0x1D3C;
constexpr u32 kStenRefMaskC = 0x1D40;    // ref 7:0, mask 23:16, wrmask 31:24
constexpr u32 kPlane3dMaskC = 0x1D44;
constexpr u32 kDpDatatype = 0x16C4;      // canonical, engWrite-normalized
constexpr u32 kDstOffset = 0x1404, kDstPitch = 0x1408;
constexpr u32 kScLeft = 0x1640, kScRight = 0x1644;
constexpr u32 kScTop = 0x1648, kScBottom = 0x164C;
constexpr u32 kEngAddrMask = 0x03FFFFF0u;
// The card's internal address split: offsets at or above 32 MB are the AGP
// aperture — system memory through the GART. r128_reg.h pins the constant
// (R128_AGP_TEX_OFFSET 0x02000000) and the 2D engine already refuses such
// destinations; textures are where this model finally honors them.
constexpr u32 kAgpWindow = 0x02000000u;

// ── File-static engine state (NEVER members: snapshot rule) ───────────────
//
// A snapshot taken between an INDX_PRIM and its NEXT_VERTEX_BUNDLE, or
// between fog-table words, resumes without this — the same accepted caveat
// as the parser's staging FIFO, and minted snapshots are quiescent.
// f=255 is "no fog" in the blend equation, so an unwritten table must read
// 255 from the very first machine of the process — g4run never calls the
// reset, and a zero-initialized table would fog that machine solid.
struct FogTable {
    u8 t[256];
    FogTable() { std::memset(t, 0xFF, sizeof t); }
};
FogTable gFog;
u32 gPalette[2][256]; // LOAD_PALETTE targets; CI4 uses entries 0..15
// The write gate. -1 = no guest has ever steered SCALE_3D_FN: writes pass,
// because the DRI's own first context burst writes Z/TEX registers BEFORE
// the MISC word in the same Packet-0 — a strict power-on gate would drop
// exactly the init sequence the shipping drivers use. 0 = a guest set the
// function to something other than TEXMAP_SHADE: the block drops writes, the
// documented behavior a driver may rely on. 1 = open.
int gGate3d = -1;
// The vertex-walker continuation latch for NEXT_VERTEX_BUNDLE (F.27): the
// bundle carries only indices, "rendered in the same manner as those of the
// previous 3D_RNDR_GEN_INDX_PRIM packet".
struct WalkLatch {
    bool valid = false;
    u32 vloff = 0, vsize = 0, fmt = 0, cntl = 0;
} gWalk;
// ── Dithering (SDK 6-53) ─────────────────────────────────────────────────
//
// With TEX_CNTL_C:DITHER_EN set, a 15 or 16 bpp destination is dithered by
// one of two methods, SCALE_3D_CNTL:SCALE_DITHER choosing: 0 = error
// diffusion, 1 = a table lookup. The manual gives no table, so table dither
// stays as it was (the color rounded to 8 bits and cut to the channel's
// width) and the census marks it. Error diffusion it describes by its
// register: "the current contents of the error register" are used at the
// start of a scanline, or the error is reset to zero there, as DITHER_INIT
// says. One register, carried along the line: the dither is one-dimensional.
//
// This is that mechanism in its plainest form, and NOT a claim to match the
// chip pixel for pixel, because three things are not stated anywhere:
//   - the register's width: taken as the bits the channel drops (three for a
//     5-bit channel, two for the 6-bit green), which is what an adder that
//     keeps its remainder holds;
//   - what happens at full scale: the sum is held at 255;
//   - the order pixels meet the register: this rasterizer's own, each
//     primitive's rows top to bottom and left to right, the register going
//     from one primitive to the next, and only a pixel that is written
//     moving it.
// What it does guarantee is what dithering is for: along a span the mean of
// what is stored equals the color asked for, where cutting the low bits
// stored a level too dark by half a step on average.
struct DitherError {
    u32 r = 0, g = 0, b = 0;
} gDither;

int gLogBudget = 24; // first-N prints for declines, so a busy path cannot spam
// --tri-dump: a print budget for real triangles, plus the VC_FORMAT/prim of
// the packet currently being walked (rasterTriangle cannot see the packet
// header, and the format says whether RHW was even in the stream).
int gTriDump = 0;
u32 gDumpFmt = 0, gDumpPrim = 0;
u64 gTriSeq = 0; // triangles seen since reset, so dump lines are addressable

R128EngStats& eng() { return r128EngStatsMut(); }

// ── The decoded vertex ────────────────────────────────────────────────────
struct Vtx {
    float x = 0, y = 0, z = 0, rhw = 1;
    float ca = 255, cr = 255, cg = 255, cb = 255; // diffuse, 0..255
    float sr = 0, sg = 0, sb = 0, fog = 255;      // specular + fog alpha
    float s1 = 0, t1 = 0, s2 = 0, t2 = 0;
};

u32 vtxDwords(u32 fmt)
{
    u32 n = 3; // x, y, z always present (F.24.3)
    if (fmt & 0x001u) ++n;      // RHW
    if (fmt & 0x002u) n += 3;   // DIFFUSE_BGR floats
    if (fmt & 0x004u) ++n;      // DIFFUSE_A float
    if (fmt & 0x008u) ++n;      // DIFFUSE_ARGB dword
    if (fmt & 0x010u) n += 3;   // SPEC_BGR floats
    if (fmt & 0x020u) ++n;      // SPEC_F float
    if (fmt & 0x040u) ++n;      // SPEC_FRGB dword
    if (fmt & 0x080u) n += 2;   // S_T
    if (fmt & 0x100u) n += 2;   // S2_T2
    if (fmt & 0x200u) ++n;      // RHW2 ("not used for DirectX")
    return n;
}

float asF32(u32 w)
{
    float f;
    std::memcpy(&f, &w, 4);
    return f;
}

// Decode one FTLVERTEX from `w` (already LE-composed stream dwords), field
// order exactly Table F-45.
void decodeVtx(const u32* w, u32 fmt, Vtx& v)
{
    u32 i = 0;
    v.x = asF32(w[i++]);
    v.y = asF32(w[i++]);
    v.z = asF32(w[i++]);
    if (fmt & 0x001u)
        v.rhw = asF32(w[i++]);
    if (fmt & 0x002u) { // OpenGL-format floats, 0..1, order B, G, R
        v.cb = asF32(w[i++]) * 255.0f;
        v.cg = asF32(w[i++]) * 255.0f;
        v.cr = asF32(w[i++]) * 255.0f;
    }
    if (fmt & 0x004u)
        v.ca = asF32(w[i++]) * 255.0f;
    if (fmt & 0x008u) {
        const u32 d = w[i++];
        v.ca = static_cast<float>(d >> 24);
        v.cr = static_cast<float>((d >> 16) & 0xFFu);
        v.cg = static_cast<float>((d >> 8) & 0xFFu);
        v.cb = static_cast<float>(d & 0xFFu);
    }
    if (fmt & 0x010u) {
        v.sb = asF32(w[i++]) * 255.0f;
        v.sg = asF32(w[i++]) * 255.0f;
        v.sr = asF32(w[i++]) * 255.0f;
    }
    if (fmt & 0x020u)
        v.fog = asF32(w[i++]) * 255.0f;
    if (fmt & 0x040u) { // FRGB: fog 31:24, then R, G, B
        const u32 d = w[i++];
        v.fog = static_cast<float>(d >> 24);
        v.sr = static_cast<float>((d >> 16) & 0xFFu);
        v.sg = static_cast<float>((d >> 8) & 0xFFu);
        v.sb = static_cast<float>(d & 0xFFu);
    }
    if (fmt & 0x080u) {
        v.s1 = asF32(w[i++]);
        v.t1 = asF32(w[i++]);
    }
    if (fmt & 0x100u) {
        v.s2 = asF32(w[i++]);
        v.t2 = asF32(w[i++]);
    }
    // RHW2 consumed by the caller via vtxDwords; nothing here reads it.
    if (fmt & 0x080u) {
        // The coordinate-convention census — see R128EngStats. Guarded
        // against NaN/garbage floats so one bad vertex cannot poison the
        // extrema forever.
        auto& e = eng();
        if (v.s1 > -1e6f && v.s1 < 1e6f) {
            if (v.s1 < e.stMinS) e.stMinS = v.s1;
            if (v.s1 > e.stMaxS) e.stMaxS = v.s1;
        }
        if (v.t1 > -1e6f && v.t1 < 1e6f) {
            if (v.t1 < e.stMinT) e.stMinT = v.t1;
            if (v.t1 > e.stMaxT) e.stMaxT = v.t1;
        }
    }
}

// ── Per-packet pipeline state, gathered once ──────────────────────────────
struct TexUnit {
    bool on = false;
    u32 cntl = 0, comb = 0;
    u32 fmt = 0;              // PRIMARY_DATATYPE
    u32 wTex = 0, hTex = 0, pitchTex = 0; // base-level texels
    u32 maxLevel = 0;
    u32 offs[11] = {};
    // Per level: does the tiling mode make this level's reads honest-count
    // material (mode 1, or mode 2/3 with a LIVE surface range covering the
    // base)? Decided ONCE here — the old per-fetch surface scan was twelve
    // register-map lookups per texel, and the profiler billed a third of
    // the machine's whole in-game host time to this engine.
    bool tileCount[11] = {};
    u32 border = 0;           // ARGB
    bool useSecondSt = false; // SEC_SRC_SEL_ST
};

// What a packet's primitives are drawn with: the state, gathered once.
struct PipeState {
    R128Cell* c = nullptr;
    CceMem m;
    // destination
    u32 bypp = 0, dstType = 0, dstOff = 0, dstStride = 0;
    int scL = 0, scR = 0, scT = 0, scB = 0;
    // The window origin added to every vertex, in pixels: it carries the
    // vertices' own sub-pixel bits, so it need not be whole.
    double winX = 0, winY = 0;
    // Vertex positions reach the setup engine in fixed point, with two
    // fractional bits or four: the steps in a pixel, and whether the
    // conversion rounds to the nearest step or cuts. See snapVertex.
    float subSteps = 4.0f;
    bool subNearest = false;
    // SETUP_CNTL bit 9 clear (r128_reg.h TEXTURE_ST_MULT_W): the vertices'
    // S and T are plain, and the chip multiplies them by W itself.
    bool stTimesW = false;
    u32 planeMask = 0xFFFFFFFFu;
    // Z / stencil
    bool zEn = false, zWr = false, stEn = false;
    u32 zw = 16;              // 16/24/32
    u32 zTest = 7, zOff = 0, zStride = 0;
    u32 stTest = 7, stSFail = 0, stZPass = 0, stZFail = 0;
    u32 stRef = 0, stMask = 0xFF, stWrMask = 0xFF;
    // shading / blending / fog
    u32 fpu = 0;              // PM4_VC_FPU_SETUP
    u32 texCntl = 0, misc = 0;
    bool blendEn = false, alphaTestEn = false, fogEn = false, fogTable = false;
    bool specEn = false, chromaEn = false, ditherEn = false;
    // Dithering by error diffusion is in force (see gDither), and whether
    // the error is cleared where a scanline of a primitive begins.
    bool ditherDiffuse = false, ditherReset = false;
    bool alphaMask = false;   // TEX_CNTL_C bit 13: see shadeFragment
    u32 blendSrc = 1, blendDst = 0, blendComb = 0, alphaOp = 7, refAlpha = 0;
    float fogR = 0, fogG = 0, fogB = 0;
    u32 chromaClr = 0, chromaMsk = 0;
    float constA = 255, constR = 255, constG = 255, constB = 255;
    // The same two colors as F4 loads them: alpha, red, green, blue (the
    // fog color has no alpha).
    float constC[4] = {255, 255, 255, 255}, fogC[4] = {0, 0, 0, 0};
    TexUnit tex[2];
    bool texAny = false;
    // ── Texel memory, decided once a packet (texSpan) ──
    // Video memory as a pointer and its end; whether system memory may be
    // read in place; and the pages of AGP space this packet's texels have
    // been found in so far, each walked through the GART once.
    const u8* vram = nullptr;
    u32 vramEnd = 0;
    bool fastMem = false;
};

// That state as one thread draws from it: with what the thread counts and
// what it has found of texel memory, which are its own. A thread drawing
// queued triangles takes each one's state into its own record (RowQueue) and
// keeps the rest.
struct Pipe : PipeState {
    // What the pixel path counts, kept here and added to the engine's
    // statistics when the packet is drawn (flushTally).
    struct Tally {
        u64 triPixels = 0, offVram = 0, texSamples = 0, texUnimpl = 0;
    };
    mutable Tally tally;
    // Set in the records several threads draw from at once: the rare paths
    // that touch state all threads share take a lock then (texByte).
    bool crowd = false;
    // Which copy of this record's state the row queue holds (RowQueue), once
    // a triangle of this packet has been queued.
    mutable u64 queued = ~0ull;
    static constexpr u32 kAgpPages = 64;
    struct AgpPage {
        u32 page = ~0u;
        const u8* host = nullptr; // null: not plainly there, ask byte by byte
    };
    mutable AgpPage agp[kAgpPages];
};

int sx14c(u32 v)
{
    const int t = static_cast<int>(v & 0x3FFFu);
    return t >= 0x2000 ? t - 0x4000 : t;
}

u32 log2Field(u32 v, u32 shift) { return (v >> shift) & 0xFu; }

// The value census (opm/r128.hpp): every multi-valued setting this packet's
// primitives are drawn with, each only when it is in force.
void noteSettings(const R128Cell& c, const Pipe& P)
{
    using S = R128Setting;
    const u32 zs = c.peek(kZStenCntlC);
    if (P.zEn) {
        r128SettingUsed(S::ZWidth, (zs >> 1) & 3u);
        r128SettingUsed(S::ZTest, P.zTest);
    }
    if (P.stEn) {
        r128SettingUsed(S::StencilTest, P.stTest);
        r128SettingUsed(S::StencilFail, P.stSFail);
        r128SettingUsed(S::StencilZPass, P.stZPass);
        r128SettingUsed(S::StencilZFail, P.stZFail);
    }
    if (P.blendEn) {
        r128SettingUsed(S::BlendSrc, P.blendSrc);
        r128SettingUsed(S::BlendDst, P.blendDst);
        r128SettingUsed(S::BlendCombine, P.blendComb);
    }
    if (P.alphaTestEn)
        r128SettingUsed(S::AlphaTest, P.alphaOp);
    if (P.ditherEn) {
        const u32 sc = c.peek(kScale3dCntl);
        r128SettingUsed(S::DitherMethod, (sc >> 1) & 1u);
        r128SettingUsed(S::DitherInit, (sc >> 3) & 1u);
    }
    r128SettingUsed(S::SubPixel, (P.fpu >> 7) & 1u);
    r128SettingUsed(S::EdgeAntiAlias, (P.fpu >> 10) & 1u);
    r128SettingUsed(S::Supersample, (P.fpu >> 11) & 1u);
    if (P.fpu) { // zero is the walker's "draw everything, as given"
        r128SettingUsed(S::Shading, (P.fpu >> 5) & 3u);
        r128SettingUsed(S::BackFace, (P.fpu >> 1) & 3u);
        r128SettingUsed(S::FrontFace, (P.fpu >> 3) & 3u);
    }
    r128SettingUsed(S::TexAntiAlias, (P.texCntl >> 21) & 1u);
    if (!P.texAny)
        return;
    if (P.chromaEn) {
        // The driver here writes the function in SCALE_3D_CNTL's top bits;
        // the guide places the field in MISC_3D_STATE_CNTL_REG.
        const u32 fnScale = c.peek(kScale3dCntl) >> 30;
        r128SettingUsed(S::TexColorKey, fnScale ? fnScale : P.misc >> 30);
    }
    r128SettingUsed(S::TexCoords, (c.peek(kSetupCntl) >> 9) & 1u);
    r128SettingUsed(S::LightColor, (P.texCntl >> 14) & 0xFu);
    r128SettingUsed(S::LightAlpha, (P.texCntl >> 18) & 7u);
    for (int u = 0; u < 2; ++u) {
        const TexUnit& t = P.tex[u];
        if (!t.on)
            continue;
        const auto at = [u](S first, u32 k) {
            return static_cast<S>(static_cast<u32>(first) + k +
                                  (u ? static_cast<u32>(S::Tex2MinFilter) -
                                           static_cast<u32>(S::Tex1MinFilter)
                                     : 0u));
        };
        r128SettingUsed(at(S::Tex1MinFilter, 0), (t.cntl >> 1) & 7u);
        r128SettingUsed(at(S::Tex1MagFilter, 0), (t.cntl >> 4) & 7u);
        r128SettingUsed(at(S::Tex1ClampS, 0), (t.cntl >> 8) & 3u);
        r128SettingUsed(at(S::Tex1ClampT, 0), (t.cntl >> 11) & 3u);
        r128SettingUsed(at(S::Tex1Format, 0), t.fmt);
        if (t.fmt == 1u || t.fmt == 2u)
            r128SettingUsed(at(S::Tex1PalEntries, 0), (t.cntl >> 24) & 3u);
        const u32 comb = t.comb & 0xFu, combA = (t.comb >> 14) & 0xFu;
        r128SettingUsed(at(S::Tex1Combine, 0), comb);
        if (comb) {
            r128SettingUsed(at(S::Tex1ColorFactor, 0), (t.comb >> 4) & 0xFu);
            r128SettingUsed(at(S::Tex1ColorInput, 0), (t.comb >> 10) & 0xFu);
        }
        r128SettingUsed(at(S::Tex1CombineAlpha, 0), combA);
        if (combA) {
            r128SettingUsed(at(S::Tex1AlphaFactor, 0), (t.comb >> 18) & 0xFu);
            r128SettingUsed(at(S::Tex1AlphaInput, 0), (t.comb >> 25) & 7u);
        }
    }
}

// The texture census's view of a packet (opm/r128.hpp), defined after the
// texel reader it lends out.
void noteTextures(const R128Cell& c, const Pipe& P);

bool gatherPipe(R128Cell& c, const CceMem& m, Pipe& P)
{
    P.c = &c;
    P.m = m;
    P.vram = c.vram.data();
    P.vramEnd = c.vram.size() < kAgpWindow ? static_cast<u32>(c.vram.size())
                                           : kAgpWindow;
    // System memory in place: not under a frame recording, which is told of
    // every read, and not while a read has to be snooped against the
    // processor's caches first. On the engine's own thread and in a replay
    // there is nothing to ask; a processor that reads memory directly says
    // so (SnoopSink::memoryCurrent).
    P.fastMem = m.ram && !r128RecOn() &&
                (!m.snoop || m.snoop->memoryCurrent());
    const u32 dt = c.peek(kDpDatatype) & 0xFu;
    r128SettingUsed(R128Setting::Dst3d, dt);
    switch (dt) {
    case 3u: case 4u: P.bypp = 2; break; // aRGB1555 / RGB565
    case 6u: P.bypp = 4; break;          // aRGB8888 (X byte stored)
    default:
        ++eng().badBpp; // 8-bpp and YUV destinations are not rasterized
        return false;
    }
    P.dstType = dt;
    P.dstOff = c.peek(kDstOffset) & kEngAddrMask;
    P.dstStride = (c.peek(kDstPitch) & 0x3FFu) * 8u * P.bypp;
    if (!P.dstStride) {
        ++eng().zeroPitch;
        return false;
    }
    if (P.dstOff >= (32u << 20)) {
        ++eng().agpTarget; // rendering INTO system memory: not modeled
        return false;
    }
    P.scL = sx14c(c.peek(kScLeft));
    P.scR = sx14c(c.peek(kScRight));
    P.scT = sx14c(c.peek(kScTop));
    P.scB = sx14c(c.peek(kScBottom));
    const u32 pm = c.peek(kPlane3dMaskC);
    P.planeMask = pm ? pm : 0xFFFFFFFFu; // reset-zero must not mean "write
                                         // nothing", the 2D mask's own rule
    P.fpu = c.peek(kFpuSetup);
    // PM4_VC_FPU_SETUP bit 7 (r128_reg.h FPU_SUB_PIX_2BITS / _4BITS) and bit
    // 15 (FPU_ROUND_TRUNCATE / _NEAREST).
    P.subSteps = (P.fpu >> 7) & 1u ? 16.0f : 4.0f;
    P.subNearest = (P.fpu >> 15) & 1u;
    P.stTimesW = !((c.peek(kSetupCntl) >> 9) & 1u);
    // WINDOW_XY_OFFSET (guide 3-248): X in 31:16 and Y in 15:0, signed, "in
    // S.12.2 format or S.11.4 format depending on SUB_PIX_AMNT", which is to
    // say in the vertices' own fixed point. (It was read as whole pixels
    // from the top twelve bits of each half: right only at four fractional
    // bits, and a quarter of the offset at two, which is what the Mac driver
    // runs with.)
    const u32 win = c.peek(kWindowXyOffset);
    P.winX = static_cast<double>(static_cast<int16_t>(win >> 16)) / P.subSteps;
    P.winY = static_cast<double>(static_cast<int16_t>(win & 0xFFFFu)) /
             P.subSteps;
    P.texCntl = c.peek(kTexCntlC);
    P.misc = c.peek(kMisc3dStateCntl);
    // Z / stencil
    P.zEn = P.texCntl & 0x1u;
    P.zWr = (P.texCntl >> 1) & 0x1u;
    P.stEn = (P.texCntl >> 3) & 0x1u;
    const u32 zs = c.peek(kZStenCntlC);
    P.zw = ((zs >> 1) & 3u) == 0u ? 16u : ((zs >> 1) & 3u) == 1u ? 24u : 32u;
    P.zTest = (zs >> 4) & 7u;
    P.stTest = (zs >> 12) & 7u;
    P.stSFail = (zs >> 16) & 7u;
    P.stZPass = (zs >> 20) & 7u;
    P.stZFail = (zs >> 24) & 7u;
    P.zOff = c.peek(kZOffsetC) & kEngAddrMask;
    const u32 zp = c.peek(kZPitchC);
    P.zStride = (zp & 0x3FFu) * 8u * (P.zw == 16u ? 2u : 4u);
    if ((zp >> 16) & 1u)
        ++eng().zTile; // tiled Z read as linear — counted, not silent
    const u32 srm = c.peek(kStenRefMaskC);
    P.stRef = srm & 0xFFu;
    P.stMask = (srm >> 16) & 0xFFu;
    P.stWrMask = (srm >> 24) & 0xFFu;
    // blend / test / fog
    P.blendEn = (P.texCntl >> 9) & 0x1u;
    P.alphaTestEn = (P.texCntl >> 10) & 0x1u;
    P.specEn = (P.texCntl >> 11) & 0x1u;
    P.chromaEn = (P.texCntl >> 12) & 0x1u;
    P.alphaMask = (P.texCntl >> 13) & 0x1u;
    P.fogEn = (P.texCntl >> 7) & 0x1u;
    P.ditherEn = (P.texCntl >> 8) & 0x1u;
    {
        const u32 sc = c.peek(kScale3dCntl);
        P.ditherDiffuse = P.ditherEn && !((sc >> 1) & 1u) && P.bypp == 2u;
        P.ditherReset = (sc >> 3) & 1u;
    }
    P.fogTable = (P.misc >> 14) & 0x1u;
    P.blendSrc = (P.misc >> 16) & 0xFu;
    P.blendDst = (P.misc >> 20) & 0xFu;
    P.blendComb = (P.misc >> 12) & 3u;
    P.alphaOp = (P.misc >> 24) & 7u;
    P.refAlpha = P.misc & 0xFFu;
    const u32 fc = c.peek(kFogColorC);
    P.fogR = static_cast<float>((fc >> 16) & 0xFFu);
    P.fogG = static_cast<float>((fc >> 8) & 0xFFu);
    P.fogB = static_cast<float>(fc & 0xFFu);
    P.chromaClr = c.peek(kTexClrCmpClrC);
    P.chromaMsk = c.peek(kTexClrCmpMskC);
    const u32 cc = c.peek(kConstantColorC);
    P.constA = static_cast<float>(cc >> 24);
    P.constR = static_cast<float>((cc >> 16) & 0xFFu);
    P.constG = static_cast<float>((cc >> 8) & 0xFFu);
    P.constB = static_cast<float>(cc & 0xFFu);
    P.constC[0] = P.constA;
    P.constC[1] = P.constR;
    P.constC[2] = P.constG;
    P.constC[3] = P.constB;
    P.fogC[1] = P.fogR;
    P.fogC[2] = P.fogG;
    P.fogC[3] = P.fogB;
    // texture units
    const u32 sp = c.peek(kTexSizePitchC);
    for (int u = 0; u < 2; ++u) {
        TexUnit& t = P.tex[u];
        t.cntl = c.peek(u ? kSecTexCntlC : kPrimTexCntlC);
        t.comb = c.peek(u ? kSecTexCombineCntlC : kPrimTexCombineCntlC);
        t.on = u ? ((P.texCntl >> 5) & 1u) && ((P.texCntl >> 4) & 1u)
                 : ((P.texCntl >> 4) & 1u);
        const u32 f = u ? (sp >> 16) : sp;
        // ⭐ THE WIDTH IS THE PITCH FIELD; THE "SIZE" FIELD IS THE LARGER OF
        // THE TWO DIMENSIONS. r128_reg.h names the four fields PITCH, SIZE,
        // HEIGHT and MIN_SIZE and says no more. A driver's own traffic
        // settles it: over three frames of a game, 3,469 uses of a unit,
        // SIZE was max(PITCH, HEIGHT) every time, and in 1,012 of them the
        // texture was taller than wide (PITCH below SIZE): 2 by 16, 8 by 16,
        // 32 by 128. So SIZE counts the maps (it is the offset slot of the
        // largest, below, and with MIN_SIZE the number of smaller ones) and
        // the width in texels is the pitch. Reading SIZE as the width was
        // right for every square or wide texture and scaled S by the height
        // for a tall one: its rows ran into each other, which on a light map
        // is bands of every color.
        t.pitchTex = 1u << log2Field(f, 0);
        t.wTex = t.pitchTex;
        t.hTex = 1u << log2Field(f, 8);
        const u32 szl2 = log2Field(f, 4), minl2 = log2Field(f, 12);
        t.maxLevel = szl2 > minl2 ? szl2 - minl2 : 0;
        if (t.maxLevel > 10u)
            t.maxLevel = 10u;
        t.fmt = (t.cntl >> 16) & 0xFu;
        // ⭐⭐ THE OFFSET SLOTS ARE INDEXED BY LEVEL SIZE, NOT BY MIP NUMBER.
        //
        // Measured from a game's own traffic (the diag register census):
        // TEX_SIZE_PITCH_C cycles 0x666/0x777/0x888 while the driver writes
        // ONLY PRIM_TEX_6/7/8_OFFSET_C — one slot per texture, the slot
        // number equal to log2 of the texture's size. So the base of a
        // 2^k-texel texture lives in slot k, its first mip in slot k-1, and
        // slot 0 holds a 1×1 level. A model that read the base from slot 0
        // sampled VRAM address zero for every texel of every surface: the
        // whole 3D world rendered black with junk flecks, 156 million
        // fetches per capture, no counter tripped. The SDK's "base texture
        // in TEX_0_OFFSET" prose reads like mip-indexing and is not.
        //
        // offs[] stays mip-indexed for the sampler; the remap happens here:
        // level l of a base-size-2^szl2 texture reads slot szl2 - l.
        for (u32 l = 0; l < 11u; ++l) {
            const u32 slot = szl2 >= l ? szl2 - l : 0u;
            t.offs[l] = c.peek(
                (u ? kSecTex0OffsetC : kPrimTex0OffsetC) + 4u * slot);
            // Tiling-mode census, hoisted from the texel fetch (see the
            // TexUnit note): mode 1 always counts; mode 2/3 counts only
            // when a live SURFACE0-3 range covers the level's base.
            const u32 tmode = (t.offs[l] >> 30) & 3u;
            t.tileCount[l] = tmode == 1u;
            if (tmode >= 2u) {
                const u32 lvBase = t.offs[l] & 0x07FFFFFFu;
                for (u32 sfc = 0; sfc < 4u; ++sfc) {
                    const u32 info = c.peek(0x0B0Cu + sfc * 0x10u);
                    const u32 lo = c.peek(0x0B04u + sfc * 0x10u);
                    const u32 hi = c.peek(0x0B08u + sfc * 0x10u);
                    if (info && hi > lo && lvBase >= lo && lvBase < hi) {
                        t.tileCount[l] = true;
                        break;
                    }
                }
            }
        }
        t.border = c.peek(u ? kSecTexBorderColorC : kPrimTexBorderColorC);
        t.useSecondSt = u && (t.cntl & 1u);
    }
    P.texAny = P.tex[0].on;
    noteSettings(c, P);
    if (gR128TexCensusOn)
        noteTextures(c, P);
    return true;
}

// ── VRAM/AGP texel bytes ──────────────────────────────────────────────────
//
// One abstraction so every format decodes from CONSECUTIVE BYTES exactly as
// the guest laid them down: below the AGP window the byte comes from VRAM,
// above it from system memory through the GART (fetched as the LE dword the
// bus would return, then the addressed lane extracted — memory byte order is
// preserved either way).
//
// ⭐ WHICH BYTE OF A TEXEL COMES FIRST DEPENDS ON WHERE THE TEXTURE LIES.
// The chip's own memory is little-endian (SDK 2.5.4 gives every pixel
// format "in the little endian representation"), and a big-endian host sees
// video memory the right way round only because the linear aperture swaps
// on the way in and out (register guide, CONFIG_CNTL APER_0/1_ENDIAN: "big
// endian: 16 bpp swapping", "32 bpp swapping"; the second copy of the
// aperture exists "to allow each to be independently marked as big-endian
// or little-endian in the PowerMac environment"). This model keeps video
// memory as the host wrote it, most significant byte first, and does not
// swap: the convention at the top of this file. A texture in AGP space is
// in system memory, which the chip reads itself with no aperture between,
// so a driver has to lay its texels down as the chip holds them: LEAST
// significant byte first. A game's own traffic says the same: of the
// 16-bit textures it drew with in one session, the 25 in video memory that
// were not flat all read smoother high byte first and the 62 in AGP space
// all low byte first, the same control bits set for both (the texture
// census, r128TextureReport). Read high byte first, those light maps were
// bands of every color.
//
// So a texel of several bytes is read in the order texelLowFirst says, in
// all three places a texel is decoded: fetchTexelBytes, fetchTexel and the
// four texels of sampleLevel. The 1555 format is the one traffic has shown
// in AGP space; the others follow from the same rule.
inline bool texelLowFirst(u32 base) { return base >= kAgpWindow; }
// A texel of two or of four bytes as a number, from a pointer to its first.
// Both orders are one load of the same bytes, so the choice costs a select
// and not a second way of addressing them.
OPM_FORCE_INLINE u32 texel16(const u8* p, bool low)
{
    const u32 lo = static_cast<u32>(p[0]) | (static_cast<u32>(p[1]) << 8);
    const u32 hi = (static_cast<u32>(p[0]) << 8) | p[1];
    return low ? lo : hi;
}
OPM_FORCE_INLINE u32 texel32(const u8* p, bool low)
{
    const u32 lo = static_cast<u32>(p[0]) | (static_cast<u32>(p[1]) << 8) |
                   (static_cast<u32>(p[2]) << 16) |
                   (static_cast<u32>(p[3]) << 24);
    const u32 hi = (static_cast<u32>(p[0]) << 24) |
                   (static_cast<u32>(p[1]) << 16) |
                   (static_cast<u32>(p[2]) << 8) | p[3];
    return low ? lo : hi;
}
bool texByte(const Pipe& P, u32 addr, u8& out)
{
    if (addr < kAgpWindow) {
        const auto& vram = P.c->vram;
        if (addr >= vram.size())
            return false;
        out = vram[addr];
        return true;
    }
    // The walk counts its misses and logs its first few in state every
    // thread shares; rows drawn by several threads at once come through
    // here only for a texel across two pages or on an unmapped one.
    static std::mutex shared;
    std::unique_lock<std::mutex> lock(shared, std::defer_lock);
    if (P.crowd)
        lock.lock();
    u32 w = 0;
    if (!r128CceGartRead(P.m, (addr - kAgpWindow) & ~3u, w)) {
        ++eng().texGartMiss;
        return false;
    }
    out = static_cast<u8>(w >> (8u * (addr & 3u)));
    return true;
}

struct Rgba {
    float a = 255, r = 0, g = 0, b = 0;
};

float clamp255(float v) { return v < 0 ? 0.0f : v > 255.0f ? 255.0f : v; }

// ── Four channels at once ─────────────────────────────────────────────────
//
// A color in the order Rgba holds it: alpha, red, green, blue. Everything
// the fragment path does to a color it does to each channel alike, or to the
// three colors alike and to alpha apart; written a channel at a time that
// was four of every operation and, worse, four small stores followed by one
// wide load wherever the compiler had joined them up. Here a color stays in
// one register from the texels to the pixel.
//
// ⚠ EACH OPERATION IS THE SCALAR ONE, PER LANE: no fused multiply-add, no
// reciprocal, no reordering. clamped() is clamp255 with its comparisons the
// same way round, so that a NaN and a negative zero come out as they did.
static_assert(sizeof(Rgba) == 16, "F4 loads an Rgba as four floats");
struct F4 {
#if OPM_F4_SSE
    __m128 v;
    static F4 set(float a, float r, float g, float b)
    {
        return {_mm_set_ps(b, g, r, a)};
    }
    static F4 splat(float x) { return {_mm_set1_ps(x)}; }
    static F4 of(const Rgba& c) { return {_mm_loadu_ps(&c.a)}; }
    static F4 of(const float c[4]) { return {_mm_loadu_ps(c)}; }
    // static_cast<float>(m[k] * x + c[k]) in each lane: four attribute
    // planes stepped to one pixel, in double as the planes are held.
    static F4 affine(const double m[4], double x, const double c[4])
    {
        const __m128d vx = _mm_set1_pd(x);
        const __m128 lo = _mm_cvtpd_ps(_mm_add_pd(
            _mm_mul_pd(_mm_loadu_pd(m), vx), _mm_loadu_pd(c)));
        const __m128 hi = _mm_cvtpd_ps(_mm_add_pd(
            _mm_mul_pd(_mm_loadu_pd(m + 2), vx), _mm_loadu_pd(c + 2)));
        return {_mm_movelh_ps(lo, hi)};
    }
    void store(float out[4]) const { _mm_storeu_ps(out, v); }
    float a() const { return _mm_cvtss_f32(v); }
    F4 withA(float x) const { return {_mm_move_ss(v, _mm_set_ss(x))}; }
    F4 clamped() const
    {
        // max(0, v) is "0 > v ? 0 : v" and min(255, t) is "255 < t ? 255 :
        // t": clamp255's two tests, each keeping its second operand when
        // the comparison is false.
        return {_mm_min_ps(_mm_set1_ps(255.0f),
                           _mm_max_ps(_mm_setzero_ps(), v))};
    }
    // static_cast<u32>(lane + 0.5f) for a CLAMPED color: a level 0..255, and
    // 0 for a NaN, which is what that cast makes of one.
    void levels(u32 out[4]) const
    {
        const __m128i i = _mm_and_si128(
            _mm_cvttps_epi32(_mm_add_ps(v, _mm_set1_ps(0.5f))),
            _mm_set1_epi32(0x7FFFFFFF));
        _mm_storeu_si128(reinterpret_cast<__m128i*>(out), i);
    }
    friend F4 operator+(F4 x, F4 y) { return {_mm_add_ps(x.v, y.v)}; }
    friend F4 operator-(F4 x, F4 y) { return {_mm_sub_ps(x.v, y.v)}; }
    friend F4 operator*(F4 x, F4 y) { return {_mm_mul_ps(x.v, y.v)}; }
    friend F4 operator/(F4 x, F4 y) { return {_mm_div_ps(x.v, y.v)}; }
#else
    float v[4];
    static F4 set(float a, float r, float g, float b) { return {{a, r, g, b}}; }
    static F4 splat(float x) { return {{x, x, x, x}}; }
    static F4 of(const Rgba& c) { return {{c.a, c.r, c.g, c.b}}; }
    static F4 of(const float c[4]) { return {{c[0], c[1], c[2], c[3]}}; }
    static F4 affine(const double m[4], double x, const double c[4])
    {
        return {{static_cast<float>(m[0] * x + c[0]),
                 static_cast<float>(m[1] * x + c[1]),
                 static_cast<float>(m[2] * x + c[2]),
                 static_cast<float>(m[3] * x + c[3])}};
    }
    void store(float out[4]) const
    {
        for (int k = 0; k < 4; ++k)
            out[k] = v[k];
    }
    float a() const { return v[0]; }
    F4 withA(float x) const { return {{x, v[1], v[2], v[3]}}; }
    F4 clamped() const
    {
        return {{clamp255(v[0]), clamp255(v[1]), clamp255(v[2]),
                 clamp255(v[3])}};
    }
    void levels(u32 out[4]) const
    {
        for (int k = 0; k < 4; ++k)
            out[k] = v[k] == v[k] ? static_cast<u32>(v[k] + 0.5f) : 0u;
    }
    friend F4 operator+(F4 x, F4 y)
    {
        return {{x.v[0] + y.v[0], x.v[1] + y.v[1], x.v[2] + y.v[2],
                 x.v[3] + y.v[3]}};
    }
    friend F4 operator-(F4 x, F4 y)
    {
        return {{x.v[0] - y.v[0], x.v[1] - y.v[1], x.v[2] - y.v[2],
                 x.v[3] - y.v[3]}};
    }
    friend F4 operator*(F4 x, F4 y)
    {
        return {{x.v[0] * y.v[0], x.v[1] * y.v[1], x.v[2] * y.v[2],
                 x.v[3] * y.v[3]}};
    }
    friend F4 operator/(F4 x, F4 y)
    {
        return {{x.v[0] / y.v[0], x.v[1] / y.v[1], x.v[2] / y.v[2],
                 x.v[3] / y.v[3]}};
    }
#endif
};

Rgba yuvToRgb(float y, float u, float v, float a)
{
    // BT.601 full-swing, the convention every overlay-era part used.
    Rgba o;
    o.a = a;
    o.r = clamp255(y + 1.402f * (v - 128.0f));
    o.g = clamp255(y - 0.344136f * (u - 128.0f) - 0.714136f * (v - 128.0f));
    o.b = clamp255(y + 1.772f * (u - 128.0f));
    return o;
}

// Fetch and decode ONE texel at integer coordinates (already wrapped),
// mip level l. False = unreadable (off VRAM / GART miss) — the border
// color stands in, which keeps a bad fetch visible without a crash.
//
// Byte by byte through texByte: every format, every corner (a texel across
// the end of video memory or across two pages of AGP space, a page the GART
// does not map, a read that has to be snooped or recorded). fetchTexel below
// answers the common formats from a pointer and comes here for the rest.
bool fetchTexelBytes(const Pipe& P, const TexUnit& t, u32 l, u32 x, u32 y,
                     Rgba& out)
{
    const u32 pitch = t.pitchTex >> l ? t.pitchTex >> l : 1u;
    const u32 base = t.offs[l] & 0x07FFFFFFu; // 31:30 are tiling flags
    // 31:30 texture mapping mode (SDK F.13): 0 linear, 1 tiled by the host
    // application, 2/3 "stored in a tiled surface" — i.e. swizzled only
    // where a SURFACE0-3 range register says so. A mode-3 offset with every
    // SURFACE register zeroed is linear in fact (measured); counting every
    // such sample as unimplemented would bury the real signal under tens of
    // millions of false positives per capture. The live-surface decision is
    // made once per packet in gatherPipe; here it is one flag.
    if (t.tileCount[l])
        ++P.tally.texUnimpl;
    ++P.tally.texSamples;
    auto b8 = [&](u32 at, u8& v) { return texByte(P, at, v); };
    u8 b0 = 0, b1 = 0, b2 = 0, b3 = 0;
    // The n bytes of a texel at `at`, into b0.. with the most significant
    // in b0 wherever the texture lies (texelLowFirst). Memory is read in
    // rising order either way, so a miss is counted at the same byte.
    const bool low = texelLowFirst(base);
    auto rd = [&](u32 at, u32 n) {
        u8* const b[4] = {&b0, &b1, &b2, &b3};
        for (u32 k = 0; k < n; ++k)
            if (!texByte(P, at + k, *b[low ? n - 1u - k : k]))
                return false;
        return true;
    };
    switch (t.fmt) {
    case 2u: { // CI8 through the CCE palette
        if (!b8(base + y * pitch + x, b0))
            return false;
        const u32 e = gPalette[((t.cntl >> 20) & 3u) == 2u ? 1 : 0][b0];
        out.a = static_cast<float>(e >> 24);
        out.r = static_cast<float>((e >> 16) & 0xFFu);
        out.g = static_cast<float>((e >> 8) & 0xFFu);
        out.b = static_cast<float>(e & 0xFFu);
        // A palette loaded without alpha reads 0 in 31:24; textures still
        // need to be opaque, matching the 2D palette convention.
        if (!(e >> 24))
            out.a = 255;
        return true;
    }
    case 1u: { // CI4
        // A texel is a BYTE whose low four bits are the index (SDK F-20,
        // the source datatype table the texture formats share: "4 bpp
        // pseudocolor. Upper 4 bits of the byte are unused"). It was read
        // as two texels a byte, which no source says.
        if (!b8(base + y * pitch + x, b0))
            return false;
        const u32 idx = b0 & 0xFu;
        const u32 e = gPalette[((t.cntl >> 20) & 3u) == 2u ? 1 : 0][idx];
        out.a = 255;
        out.r = static_cast<float>((e >> 16) & 0xFFu);
        out.g = static_cast<float>((e >> 8) & 0xFFu);
        out.b = static_cast<float>(e & 0xFFu);
        return true;
    }
    case 3u: { // ARGB1555
        const u32 at = base + (y * pitch + x) * 2u;
        if (!rd(at, 2u))
            return false;
        const u32 p = (static_cast<u32>(b0) << 8) | b1;
        out.a = (p & 0x8000u) ? 255.0f : 0.0f;
        out.r = static_cast<float>(((p >> 10) & 0x1Fu) * 255u / 31u);
        out.g = static_cast<float>(((p >> 5) & 0x1Fu) * 255u / 31u);
        out.b = static_cast<float>((p & 0x1Fu) * 255u / 31u);
        return true;
    }
    case 4u: { // RGB565
        const u32 at = base + (y * pitch + x) * 2u;
        if (!rd(at, 2u))
            return false;
        const u32 p = (static_cast<u32>(b0) << 8) | b1;
        out.a = 255;
        out.r = static_cast<float>(((p >> 11) & 0x1Fu) * 255u / 31u);
        out.g = static_cast<float>(((p >> 5) & 0x3Fu) * 255u / 63u);
        out.b = static_cast<float>((p & 0x1Fu) * 255u / 31u);
        return true;
    }
    case 5u: { // RGB888, three bytes, R the most significant
        const u32 at = base + (y * pitch + x) * 3u;
        if (!rd(at, 3u))
            return false;
        out.a = 255;
        out.r = b0;
        out.g = b1;
        out.b = b2;
        return true;
    }
    case 6u: { // ARGB8888
        const u32 at = base + (y * pitch + x) * 4u;
        if (!rd(at, 4u))
            return false;
        out.a = b0;
        out.r = b1;
        out.g = b2;
        out.b = b3;
        return true;
    }
    case 7u: { // RGB332
        if (!b8(base + y * pitch + x, b0))
            return false;
        out.a = 255;
        out.r = static_cast<float>(((b0 >> 5) & 7u) * 255u / 7u);
        out.g = static_cast<float>(((b0 >> 2) & 7u) * 255u / 7u);
        out.b = static_cast<float>((b0 & 3u) * 255u / 3u);
        return true;
    }
    case 8u:  // Y8
    case 9u: { // RGB8 gray
        if (!b8(base + y * pitch + x, b0))
            return false;
        out.a = 255;
        out.r = out.g = out.b = b0;
        return true;
    }
    case 11u:  // YVYU 422: pairs [Y0 V Y1 U], most significant byte first
    case 12u: { // VYUY 422: pairs [V Y0 U Y1]
        const u32 at = base + (y * pitch + (x & ~1u)) * 2u;
        if (!rd(at, 4u))
            return false;
        float Y, U, V;
        if (t.fmt == 11u) {
            Y = (x & 1u) ? b2 : b0;
            V = b1;
            U = b3;
        } else {
            Y = (x & 1u) ? b3 : b1;
            V = b0;
            U = b2;
        }
        out = yuvToRgb(Y, U, V, 255.0f);
        return true;
    }
    case 14u: { // AYUV 444
        const u32 at = base + (y * pitch + x) * 4u;
        if (!rd(at, 4u))
            return false;
        out = yuvToRgb(b1, b2, b3, static_cast<float>(b0));
        return true;
    }
    case 15u: { // ARGB4444
        const u32 at = base + (y * pitch + x) * 2u;
        if (!rd(at, 2u))
            return false;
        const u32 p = (static_cast<u32>(b0) << 8) | b1;
        out.a = static_cast<float>(((p >> 12) & 0xFu) * 17u);
        out.r = static_cast<float>(((p >> 8) & 0xFu) * 17u);
        out.g = static_cast<float>(((p >> 4) & 0xFu) * 17u);
        out.b = static_cast<float>((p & 0xFu) * 17u);
        return true;
    }
    // The two 16-bit formats with an alpha byte (SDK F-20: 10 is "16 bpp
    // a:pseudocolor (8:8)", 13 is "16 bpp a:RGB8 greyscale (8:8)"): the
    // 16-bit texel has alpha in its high byte, as the 1555 texel has its
    // alpha bit on top, and a palette index or a gray level in its low one.
    case 10u: {
        const u32 at = base + (y * pitch + x) * 2u;
        if (!rd(at, 2u))
            return false;
        const u32 e = gPalette[((t.cntl >> 20) & 3u) == 2u ? 1 : 0][b1];
        out.a = static_cast<float>(b0);
        out.r = static_cast<float>((e >> 16) & 0xFFu);
        out.g = static_cast<float>((e >> 8) & 0xFFu);
        out.b = static_cast<float>(e & 0xFFu);
        return true;
    }
    case 13u: {
        const u32 at = base + (y * pitch + x) * 2u;
        if (!rd(at, 2u))
            return false;
        out.a = static_cast<float>(b0);
        out.r = out.g = out.b = static_cast<float>(b1);
        return true;
    }
    default: // VQ (0): "2 bpp", and no word anywhere on its code book
        ++P.tally.texUnimpl;
        return false;
    }
}

// The texture census's view of this pipeline (opm/r128.hpp): each unit that
// is on, and the two readers the census looks at its top level through,
// which are the readers every sample uses. Out of line, and the census's own
// work in r128_census.cpp: nothing of it is in the way of the pixel path.
OPM_NOINLINE void noteTextures(const R128Cell& c, const Pipe& P)
{
    struct Lent {
        const Pipe* P;
        const TexUnit* t;
    };
    for (u32 u = 0; u < 2u; ++u) {
        const TexUnit& t = P.tex[u];
        if (!t.on)
            continue;
        Lent lent{&P, &t};
        R128TexSeen s;
        s.unit = u;
        s.base = t.offs[0] & 0x07FFFFFFu;
        s.agp = s.base >= kAgpWindow;
        s.w = t.wTex;
        s.h = t.hTex;
        s.pitch = t.pitchTex;
        s.fmt = t.fmt;
        s.texCntl = P.texCntl;
        s.unitCntl = t.cntl;
        s.dpDatatype = c.peek(kDpDatatype);
        s.ctx = &lent;
        s.byteAt = [](void* x, u32 addr, u8& out) {
            return texByte(*static_cast<Lent*>(x)->P, addr, out);
        };
        s.texelAt = [](void* x, u32 tx, u32 ty, u8 rgb[3]) {
            const Lent& k = *static_cast<Lent*>(x);
            // The pixel path's counts are put back as they were: a census
            // must not move what it reports on.
            const Pipe::Tally kept = k.P->tally;
            Rgba o;
            const bool ok = fetchTexelBytes(*k.P, *k.t, 0, tx, ty, o);
            k.P->tally = kept;
            if (ok) {
                rgb[0] = static_cast<u8>(clamp255(o.r));
                rgb[1] = static_cast<u8>(clamp255(o.g));
                rgb[2] = static_cast<u8>(clamp255(o.b));
            }
            return ok;
        };
        r128TextureSeen(s);
    }
}

// ── Texels read in place ──────────────────────────────────────────────────
//
// A sample is one to eight texels and a texel was one to four calls of
// texByte, each a bounds test and, in AGP space, a walk of the GART: that
// was half of the engine's time in a game. A texel's bytes are consecutive
// wherever they lie, so they are found once: video memory by one bounds
// test, a page of AGP space by one walk a packet (Pipe::agp; nothing the
// walk reads can change while a packet is drawn). Whatever is not plainly
// there (see fetchTexelBytes) says no, and the byte-by-byte path answers as
// it always did, counters and all.

// The page of AGP space as the walk in r128.cpp (cceGartRead) finds it, or
// null where that walk would not succeed for every byte of the page.
const u8* texAgpPage(const Pipe& P, u32 page)
{
    const CceMem& m = P.m;
    u32 agp = page << 12;
    const u32 aper = m.aperBase & 0xFFFFF000u;
    if (aper) {
        if (agp < aper)
            return nullptr;
        agp -= aper;
    }
    const u32 base = m.gartBase & ~0xFFFu;
    const u64 ea = static_cast<u64>(base) + static_cast<u64>(agp >> 12) * 4u;
    if (!base || ea + 4u > m.ramSize)
        return nullptr;
    const u8* e = m.ram + ea;
    const u32 entry = u32(e[0]) | (u32(e[1]) << 8) | (u32(e[2]) << 16) |
                      (u32(e[3]) << 24);
    const u64 pa = entry & 0xFFFFF000u;
    if (!(entry & 1u) || pa + 0x1000u > m.ramSize)
        return nullptr;
    return m.ram + pa;
}

inline bool texSpan(const Pipe& P, u32 addr, u32 n, const u8*& p)
{
    if (addr < kAgpWindow) {
        if (static_cast<u64>(addr) + n > P.vramEnd)
            return false;
        p = P.vram + addr;
        return true;
    }
    if (!P.fastMem)
        return false;
    const u32 agp = addr - kAgpWindow;
    if ((agp ^ (agp + n - 1u)) >> 12)
        return false; // the texel lies across two pages
    Pipe::AgpPage& e = P.agp[(agp >> 12) & (Pipe::kAgpPages - 1u)];
    if (e.page != (agp >> 12)) {
        e.page = agp >> 12;
        e.host = texAgpPage(P, agp >> 12);
    }
    if (!e.host)
        return false;
    p = e.host + (agp & 0xFFFu);
    return true;
}

// A 5-, 6- or 4-bit channel as the level of 255 the byte path computes for
// it: the same integer division, done once.
struct ChannelLevels {
    float k5[32], k6[64], k4[16];
    ChannelLevels()
    {
        for (u32 i = 0; i < 32; ++i)
            k5[i] = static_cast<float>(i * 255u / 31u);
        for (u32 i = 0; i < 64; ++i)
            k6[i] = static_cast<float>(i * 255u / 63u);
        for (u32 i = 0; i < 16; ++i)
            k4[i] = static_cast<float>(i * 17u);
    }
};
const ChannelLevels kLevels;

inline bool fetchTexel(const Pipe& P, const TexUnit& t, u32 l, u32 x, u32 y,
                       Rgba& out)
{
    const u32 pitch = t.pitchTex >> l ? t.pitchTex >> l : 1u;
    const u32 base = t.offs[l] & 0x07FFFFFFu;
    const bool low = texelLowFirst(base);
    const u8* p;
    switch (t.fmt) {
    case 3u: { // ARGB1555
        if (!texSpan(P, base + (y * pitch + x) * 2u, 2u, p))
            break;
        const u32 v = texel16(p, low);
        out.a = (v & 0x8000u) ? 255.0f : 0.0f;
        out.r = kLevels.k5[(v >> 10) & 0x1Fu];
        out.g = kLevels.k5[(v >> 5) & 0x1Fu];
        out.b = kLevels.k5[v & 0x1Fu];
        goto counted;
    }
    case 4u: { // RGB565
        if (!texSpan(P, base + (y * pitch + x) * 2u, 2u, p))
            break;
        const u32 v = texel16(p, low);
        out.a = 255;
        out.r = kLevels.k5[(v >> 11) & 0x1Fu];
        out.g = kLevels.k6[(v >> 5) & 0x3Fu];
        out.b = kLevels.k5[v & 0x1Fu];
        goto counted;
    }
    case 6u: { // ARGB8888
        if (!texSpan(P, base + (y * pitch + x) * 4u, 4u, p))
            break;
        const u32 v = texel32(p, low);
        out.a = static_cast<float>(v >> 24);
        out.r = static_cast<float>((v >> 16) & 0xFFu);
        out.g = static_cast<float>((v >> 8) & 0xFFu);
        out.b = static_cast<float>(v & 0xFFu);
        goto counted;
    }
    case 15u: { // ARGB4444
        if (!texSpan(P, base + (y * pitch + x) * 2u, 2u, p))
            break;
        const u32 v = texel16(p, low);
        out.a = kLevels.k4[(v >> 12) & 0xFu];
        out.r = kLevels.k4[(v >> 8) & 0xFu];
        out.g = kLevels.k4[(v >> 4) & 0xFu];
        out.b = kLevels.k4[v & 0xFu];
        goto counted;
    }
    default:
        break;
    }
    return fetchTexelBytes(P, t, l, x, y, out);
counted: // what fetchTexelBytes counts before it reads
    if (t.tileCount[l])
        ++P.tally.texUnimpl;
    ++P.tally.texSamples;
    return true;
}

// floor() of a float as an int, without the library call: the truncation,
// less one where that went up. Out of an int's range and for a NaN it is
// what the cast of floor() gave.
inline int floorInt(float f)
{
    const int i = static_cast<int>(f);
    return (static_cast<float>(i) > f && i != INT_MIN) ? i - 1 : i;
}

// Wrap one coordinate per the unit's clamp mode. Returns false when border
// color applies instead of a fetch. A level's size is a power of two, so
// the remainder of the division is a mask (for a negative coordinate too:
// the mask of the two's complement is the positive remainder).
inline bool wrapPow2(u32 mode, int i, u32 n, u32& out)
{
    switch (mode) {
    case 1u: { // mirror
        const u32 q = static_cast<u32>(i) & (2u * n - 1u);
        out = q < n ? q : 2u * n - 1u - q;
        return true;
    }
    case 2u: // clamp to edge
        out = i < 0 ? 0u
                    : static_cast<u32>(i) >= n ? n - 1u : static_cast<u32>(i);
        return true;
    case 3u: // border color beyond the edge
        if (i < 0 || static_cast<u32>(i) >= n)
            return false;
        out = static_cast<u32>(i);
        return true;
    default: // wrap (tile)
        out = static_cast<u32>(i) & (n - 1u);
        return true;
    }
}

Rgba borderColor(const TexUnit& t)
{
    Rgba o;
    o.a = static_cast<float>(t.border >> 24);
    o.r = static_cast<float>((t.border >> 16) & 0xFFu);
    o.g = static_cast<float>((t.border >> 8) & 0xFFu);
    o.b = static_cast<float>(t.border & 0xFFu);
    return o;
}

// A texel's number as a color, for the formats read in place: the levels
// fetchTexel gives, in a register.
OPM_FORCE_INLINE F4 texelColor1555(u32 v)
{
    return F4::set((v & 0x8000u) ? 255.0f : 0.0f, kLevels.k5[(v >> 10) & 0x1Fu],
                   kLevels.k5[(v >> 5) & 0x1Fu], kLevels.k5[v & 0x1Fu]);
}
OPM_FORCE_INLINE F4 texelColor565(u32 v)
{
    return F4::set(255.0f, kLevels.k5[(v >> 11) & 0x1Fu],
                   kLevels.k6[(v >> 5) & 0x3Fu], kLevels.k5[v & 0x1Fu]);
}
OPM_FORCE_INLINE F4 texelColor8888(u32 v)
{
    return F4::set(static_cast<float>(v >> 24),
                   static_cast<float>((v >> 16) & 0xFFu),
                   static_cast<float>((v >> 8) & 0xFFu),
                   static_cast<float>(v & 0xFFu));
}
OPM_FORCE_INLINE F4 texelColor4444(u32 v)
{
    return F4::set(kLevels.k4[(v >> 12) & 0xFu], kLevels.k4[(v >> 8) & 0xFu],
                   kLevels.k4[(v >> 4) & 0xFu], kLevels.k4[v & 0xFu]);
}

// Sample one unit at (s,t) in texture space (0..1 per tile), mip level l,
// point or bilinear.
F4 sampleLevel(const Pipe& P, const TexUnit& t, u32 l, float s, float tt,
               bool bilinear)
{
    const u32 w = t.wTex >> l ? t.wTex >> l : 1u;
    const u32 h = t.hTex >> l ? t.hTex >> l : 1u;
    const u32 clampS = (t.cntl >> 8) & 3u;
    const u32 clampT = (t.cntl >> 11) & 3u;
    // ⚠⚠ S/T ARE NORMALIZED [0,1] PER TILE — and the ST CENSUS DOES NOT
    // SETTLE IT. Reading texel-space out of the census extrema (s up to
    // 126.7 with a 128-texel texture bound) was wrong, and the user's eyes
    // rejected it: the picture went blacker and flatter, so this reverted.
    //
    // Why the census cannot decide: it reports GLOBAL extrema over every
    // vertex in the session, not the span of any one triangle, and the two
    // readings are each self-consistent with it. Normalized coordinates
    // with tiling factors up to ~126 (a ground plane repeating its texture)
    // produce exactly the same extrema as texel coordinates over a
    // 128-texel map. The discriminator is the PER-TRIANGLE span against
    // the bound size, which needs a snapshot to measure repeatably —
    // see opm_snapshot_save.
    const float fx = s * static_cast<float>(w) - (bilinear ? 0.5f : 0.0f);
    const float fy = tt * static_cast<float>(h) - (bilinear ? 0.5f : 0.0f);
    const int x0 = floorInt(fx);
    const int y0 = floorInt(fy);
    if (!bilinear) {
        u32 xi, yi;
        Rgba o;
        if (!wrapPow2(clampS, x0, w, xi) || !wrapPow2(clampT, y0, h, yi) ||
            !fetchTexel(P, t, l, xi, yi, o))
            return F4::of(borderColor(t));
        return F4::of(o);
    }
    const float ax = fx - static_cast<float>(x0);
    const float ay = fy - static_cast<float>(y0);
    F4 q00 = F4::splat(0.0f), q01 = q00, q10 = q00, q11 = q00;
    // ⭐ THE FOUR TEXELS READ IN PLACE, when all four are inside the map's
    // own rules (no border color) and plainly in memory (texSpan), in one of
    // the formats fetchTexel answers from a pointer. They are decoded
    // straight into registers. Anything else takes the general way below,
    // texel by texel, which is also what this replaces: the same four
    // texels, the same counts.
    u32 xs[2], ys[2];
    const bool inMap = wrapPow2(clampS, x0, w, xs[0]) &&
                       wrapPow2(clampS, x0 + 1, w, xs[1]) &&
                       wrapPow2(clampT, y0, h, ys[0]) &&
                       wrapPow2(clampT, y0 + 1, h, ys[1]);
    bool direct = false;
    if (inMap) {
        const u32 pitch = t.pitchTex >> l ? t.pitchTex >> l : 1u;
        const u32 base = t.offs[l] & 0x07FFFFFFu;
        const u32 n = t.fmt == 6u ? 4u : 2u;
        const auto at = [&](u32 x, u32 y) { return base + (y * pitch + x) * n; };
        const u8 *p00 = nullptr, *p01 = nullptr, *p10 = nullptr,
                 *p11 = nullptr;
        switch (t.fmt) {
        case 3u: case 4u: case 6u: case 15u:
            // The four are nearly always neighbors, a row apart, with no
            // edge of the map between them: then one stretch of memory
            // holds them all and is found once. Where it is not plainly
            // there as a whole (the end of video memory, two pages of AGP
            // space), each is found for itself, as before.
            if (xs[1] == xs[0] + 1u && ys[1] == ys[0] + 1u &&
                texSpan(P, at(xs[0], ys[0]), (pitch + 2u) * n, p00)) {
                p01 = p00 + n;
                p10 = p00 + pitch * n;
                p11 = p10 + n;
                direct = true;
            } else {
                direct = texSpan(P, at(xs[0], ys[0]), n, p00) &&
                         texSpan(P, at(xs[1], ys[0]), n, p01) &&
                         texSpan(P, at(xs[0], ys[1]), n, p10) &&
                         texSpan(P, at(xs[1], ys[1]), n, p11);
            }
            break;
        default:
            break;
        }
        if (direct) {
            // Each texel as a number in the order its memory keeps
            // (texelLowFirst), then into a register (texelColor*, forced in
            // line: four calls a sample is what this path is here to avoid).
            const bool low = texelLowFirst(base);
            switch (t.fmt) {
            case 3u: // ARGB1555
                q00 = texelColor1555(texel16(p00, low));
                q01 = texelColor1555(texel16(p01, low));
                q10 = texelColor1555(texel16(p10, low));
                q11 = texelColor1555(texel16(p11, low));
                break;
            case 4u: // RGB565
                q00 = texelColor565(texel16(p00, low));
                q01 = texelColor565(texel16(p01, low));
                q10 = texelColor565(texel16(p10, low));
                q11 = texelColor565(texel16(p11, low));
                break;
            case 6u: // ARGB8888
                q00 = texelColor8888(texel32(p00, low));
                q01 = texelColor8888(texel32(p01, low));
                q10 = texelColor8888(texel32(p10, low));
                q11 = texelColor8888(texel32(p11, low));
                break;
            default: // 15: ARGB4444
                q00 = texelColor4444(texel16(p00, low));
                q01 = texelColor4444(texel16(p01, low));
                q10 = texelColor4444(texel16(p10, low));
                q11 = texelColor4444(texel16(p11, low));
                break;
            }
            if (t.tileCount[l]) // four fetches' worth, as fetchTexel counts
                P.tally.texUnimpl += 4;
            P.tally.texSamples += 4;
        }
    }
    if (!direct) {
        Rgba q[2][2];
        for (int dy = 0; dy < 2; ++dy)
            for (int dx = 0; dx < 2; ++dx) {
                u32 xi, yi;
                if (!wrapPow2(clampS, x0 + dx, w, xi) ||
                    !wrapPow2(clampT, y0 + dy, h, yi) ||
                    !fetchTexel(P, t, l, xi, yi, q[dy][dx]))
                    q[dy][dx] = borderColor(t);
            }
        q00 = F4::of(q[0][0]);
        q01 = F4::of(q[0][1]);
        q10 = F4::of(q[1][0]);
        q11 = F4::of(q[1][1]);
    }
    const F4 fax = F4::splat(ax);
    const F4 top = q00 + (q01 - q00) * fax;
    const F4 bot = q10 + (q11 - q10) * fax;
    return top + (bot - top) * F4::splat(ay);
}

F4 sampleTex(const Pipe& P, const TexUnit& t, float s, float tt, float lod)
{
    // LOD bias: TEX_CNTL_C 31:24, signed 4.4 fixed point. The field width is
    // r128_reg.h's; the fixed-point split is this model's reading — revisit
    // against real traffic if mip selection looks uniformly off by a power.
    // ⚠ TEX_CNTL_C, the pipeline's register — not the unit's own control
    // register, whose top byte is the palette entries' datatype. This read
    // the unit's until the undecoded-bits census pointed at it: a driver
    // that sets "1555 palette entries" there was getting a sixteenth of a
    // level of bias for it.
    const int biasRaw = static_cast<int>((P.texCntl >> 24) & 0xFFu);
    lod += static_cast<float>(biasRaw >= 128 ? biasRaw - 256 : biasRaw) /
           16.0f;
    const bool mips = !((t.cntl >> 7) & 1u) && t.maxLevel > 0;
    const u32 minFn = (t.cntl >> 1) & 7u;
    const u32 magFn = (t.cntl >> 4) & 7u;
    if (!mips) {
        // No mip chain: the largest map serves both directions. Odd filter
        // states are bilinear in both tables (6-4/6-5), even are nearest.
        const bool bil = lod > 0.0f ? (minFn & 1u) != 0 : (magFn & 1u) != 0;
        return sampleLevel(P, t, 0, s, tt, bil);
    }
    if (lod <= 0.0f) // magnify: Table 6-5, largest map always
        return sampleLevel(P, t, 0, s, tt, (magFn & 1u) != 0);
    if (lod > static_cast<float>(t.maxLevel))
        lod = static_cast<float>(t.maxLevel);
    switch (minFn) {
    case 0u: return sampleLevel(P, t, 0, s, tt, false);
    case 1u: return sampleLevel(P, t, 0, s, tt, true);
    case 2u:
    case 3u: { // nearest map
        const u32 l = static_cast<u32>(lod + 0.5f) > t.maxLevel
                          ? t.maxLevel
                          : static_cast<u32>(lod + 0.5f);
        return sampleLevel(P, t, l, s, tt, minFn == 3u);
    }
    case 4u: // "1x1 filtering": the smallest map
        return sampleLevel(P, t, t.maxLevel, s, tt, false);
    default: { // 5: trilinear
        const u32 l0 = static_cast<u32>(lod);
        const u32 l1 = l0 + 1u > t.maxLevel ? t.maxLevel : l0 + 1u;
        const float fr = lod - static_cast<float>(l0);
        const F4 a = sampleLevel(P, t, l0, s, tt, true);
        const F4 b = sampleLevel(P, t, l1, s, tt, true);
        return a + (b - a) * F4::splat(fr);
    }
    }
}

// ── The texture-combine unit (Tables 6-7 .. 6-16) ─────────────────────────
//
// factor selections resolve to RGBA operands; the combine function folds the
// two into the unit's output. `tex` is this unit's sample, `interp` the
// iterated vertex color, `prev` the previous unit's output (secondary only).
// The three colors go through these as one value; a factor's alpha lane is
// whatever fell there, because alpha has its own selections below and takes
// nothing from here.
inline F4 pickColorFactor(const Pipe& P, u32 sel, F4 tex, F4 interp, F4 prev)
{
    const F4 k255 = F4::splat(255.0f);
    switch (sel) {
    case 0u: return F4::of(P.constC);
    case 1u: return k255 - F4::of(P.constC);
    case 4u: return tex;
    case 5u: return k255 - tex;
    case 6u: return F4::splat(tex.a());
    case 7u: return F4::splat(255.0f - tex.a());
    case 8u: return prev;
    default: return interp; // unnamed selections read as the iterator
    }
}

inline F4 pickInputFactor(const Pipe& P, u32 sel, F4 interp, F4 prev)
{
    switch (sel) {
    case 2u: return F4::of(P.constC);
    case 3u: return F4::splat(P.constA);
    case 4u: return interp;
    case 5u: return F4::splat(interp.a());
    case 8u: return prev;
    case 9u: return F4::splat(prev.a());
    default: return interp;
    }
}

// The combine function on the three colors: cf the color factor, inf the
// input factor, texC the unit's texel, cfConst the constant color.
inline F4 combColors(u32 fn, F4 cf, F4 inf, float texA, float intA,
                     float constA, F4 texC, F4 cfConst)
{
    const F4 k255 = F4::splat(255.0f);
    switch (fn) {
    case 1u: return cf;
    case 2u: return inf;
    case 3u: return cf * inf / k255;
    case 4u: return cf * inf / F4::splat(127.5f);
    case 5u: return cf * inf / F4::splat(63.75f);
    case 6u: return cf + inf;
    case 7u: return cf + inf - F4::splat(128.0f);
    case 8u:
        return (cf * F4::splat(intA) + inf * F4::splat(255.0f - intA)) / k255;
    case 9u:
        return (cf * F4::splat(texA) + inf * F4::splat(255.0f - texA)) / k255;
    case 10u:
        return (cf * F4::splat(constA) + inf * F4::splat(255.0f - constA)) /
               k255;
    case 11u: return cf + inf * F4::splat(255.0f - texA) / k255;
    case 12u:
        return (cf * F4::splat(texA) + inf * F4::splat(255.0f - texA)) / k255;
    case 13u: return cf + inf * F4::splat(texA) / k255;
    case 14u: return (cf + inf - F4::splat(128.0f)) * F4::splat(2.0f);
    case 15u: return (cf * cfConst + inf * (k255 - cfConst)) / k255;
    default: return texC; // 0: disabled — the unit passes its texel through
    }
}

inline F4 combineUnit(const Pipe& P, const TexUnit& t, F4 tex, F4 interp,
                      F4 prev)
{
    const u32 comb = t.comb & 0xFu;
    const u32 cfSel = (t.comb >> 4) & 0xFu;
    const u32 inSel = (t.comb >> 10) & 0xFu;
    const u32 combA = (t.comb >> 14) & 0xFu;
    const u32 afSel = (t.comb >> 18) & 0xFu;
    const u32 inASel = (t.comb >> 25) & 0x7u;
    const float texA = tex.a(), intA = interp.a();
    const F4 cf = pickColorFactor(P, cfSel, tex, interp, prev);
    const F4 inf = pickInputFactor(P, inSel, interp, prev);
    const float af = afSel == 7u ? 255.0f - texA : texA;
    const float inA = inASel == 1u ? P.constA
                      : inASel == 4u ? prev.a()
                                     : intA;
    const F4 o = combColors(comb, cf, inf, texA, intA, P.constA, tex,
                            F4::of(P.constC))
                     .clamped();
    float a;
    switch (combA) {
    case 1u: a = af; break;
    case 2u: a = inA; break;
    case 3u: a = af * inA / 255.0f; break;
    case 4u: a = af * inA / 127.5f; break;
    case 5u: a = af * inA / 63.75f; break;
    case 6u: a = af + inA; break;
    case 7u: a = af + inA - 128.0f; break;
    case 14u: a = (af + inA - 128.0f) * 2.0f; break;
    default: a = texA; break; // 0: disabled — texture alpha through
    }
    return o.withA(clamp255(a));
}

// TEX_LIGHT_FN / ALPHA_LIGHT_FN (Tables 6-15/6-16): the post-multitexture
// combine of the texture result with the iterated color. The color field
// reads 4 bits wide (17:14) — the SDK's table runs to state 15 and the gap
// to ALPHA_LIGHT at 18 is exactly four bits; r128_reg.h names only 0..7 and
// both readings agree there.
inline F4 texLight(const Pipe& P, F4 tex, F4 interp)
{
    const u32 fn = (P.texCntl >> 14) & 0xFu;
    const u32 fnA = (P.texCntl >> 18) & 0x7u;
    const float texA = tex.a(), intA = interp.a();
    const F4 o = combColors(fn, tex, interp, texA, intA, P.constA, tex,
                            F4::of(P.constC))
                     .clamped();
    float a;
    switch (fnA) {
    case 1u: a = texA; break;
    case 2u: a = intA; break;
    case 3u: a = texA * intA / 255.0f; break;
    case 4u: a = clamp255(texA * intA / 127.5f); break;
    case 5u: a = clamp255(texA * intA / 63.75f); break;
    case 6u: a = clamp255(texA + intA); break;
    case 7u: a = clamp255(texA + intA - 128.0f); break;
    default: a = texA; break;
    }
    return o.withA(a);
}

// ── Destination pixels (MSB-first, the engine-wide convention) ────────────
inline u32 dstRead(const Pipe& P, u64 at)
{
    const u8* p = P.vram + at;
    if (P.bypp == 2u)
        return (static_cast<u32>(p[0]) << 8) | p[1];
    return (static_cast<u32>(p[0]) << 24) | (static_cast<u32>(p[1]) << 16) |
           (static_cast<u32>(p[2]) << 8) | p[3];
}

inline void dstWrite(const Pipe& P, u64 at, u32 v)
{
    if (P.planeMask != 0xFFFFFFFFu)
        v = (dstRead(P, at) & ~P.planeMask) | (v & P.planeMask);
    if (r128PixelWatched(at, P.bypp))
        r128PixelWatchNote(
            "3d",
            "triangle %llu: TEX_CNTL %08x MISC %08x, texture %ux%u format %u "
            "cntl %08x combine %08x, wrote %x",
            static_cast<unsigned long long>(gTriSeq), P.texCntl, P.misc,
            P.tex[0].wTex, P.tex[0].hTex, P.tex[0].fmt, P.tex[0].cntl,
            P.tex[0].comb, v);
    u8* p = P.c->vram.data() + at;
    if (P.bypp == 2u) {
        p[0] = static_cast<u8>(v >> 8);
        p[1] = static_cast<u8>(v);
    } else {
        p[0] = static_cast<u8>(v >> 24);
        p[1] = static_cast<u8>(v >> 16);
        p[2] = static_cast<u8>(v >> 8);
        p[3] = static_cast<u8>(v);
    }
}

inline F4 dstDecode(const Pipe& P, u32 v)
{
    if (P.dstType == 4u)
        return F4::set(255.0f, kLevels.k5[(v >> 11) & 0x1Fu],
                       kLevels.k6[(v >> 5) & 0x3Fu], kLevels.k5[v & 0x1Fu]);
    if (P.dstType == 3u)
        return F4::set((v & 0x8000u) ? 255.0f : 0.0f,
                       kLevels.k5[(v >> 10) & 0x1Fu],
                       kLevels.k5[(v >> 5) & 0x1Fu], kLevels.k5[v & 0x1Fu]);
    return F4::set(static_cast<float>(v >> 24),
                   static_cast<float>((v >> 16) & 0xFFu),
                   static_cast<float>((v >> 8) & 0xFFu),
                   static_cast<float>(v & 0xFFu));
}

// A scanline of a primitive begins: DITHER_INIT decides what the error
// register starts it with.
inline void ditherLineStart(const Pipe& P)
{
    if (P.ditherDiffuse && P.ditherReset)
        gDither = DitherError{};
}

// One channel through the error register: the level plus what the last
// pixel left, the channel taking the top bits and the register the rest.
inline u32 ditherStep(u32 level, u32& carried, u32 dropBits)
{
    u32 sum = level + carried;
    if (sum > 255u)
        sum = 255u;
    carried = sum & ((1u << dropBits) - 1u);
    return sum >> dropBits;
}

inline u32 dstEncode(const Pipe& P, F4 c)
{
    u32 lv[4]; // each channel clamped and rounded: alpha, red, green, blue
    c.clamped().levels(lv);
    const u32 a = lv[0], r = lv[1], g = lv[2], b = lv[3];
    if (P.ditherDiffuse) {
        const u32 r5 = ditherStep(r, gDither.r, 3);
        const u32 g56 = ditherStep(g, gDither.g, P.dstType == 4u ? 2u : 3u);
        const u32 b5 = ditherStep(b, gDither.b, 3);
        return P.dstType == 4u ? (r5 << 11) | (g56 << 5) | b5
                               : ((a >> 7) << 15) | (r5 << 10) | (g56 << 5) |
                                     b5;
    }
    if (P.dstType == 4u)
        return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
    if (P.dstType == 3u)
        return ((a >> 7) << 15) | ((r >> 3) << 10) | ((g >> 3) << 5) |
               (b >> 3);
    return (a << 24) | (r << 16) | (g << 8) | b;
}

// ── Z / stencil cells ─────────────────────────────────────────────────────
inline bool zAddr(const Pipe& P, int px, int py, u64& at)
{
    const u32 zb = P.zw == 16u ? 2u : 4u;
    at = static_cast<u64>(P.zOff) +
         static_cast<u64>(py) * P.zStride + static_cast<u64>(px) * zb;
    return P.zStride != 0 && at + zb <= P.c->vram.size();
}

inline u32 zRead(const Pipe& P, u64 at)
{
    const u8* p = P.vram + at;
    if (P.zw == 16u)
        return (static_cast<u32>(p[0]) << 8) | p[1];
    return (static_cast<u32>(p[0]) << 24) | (static_cast<u32>(p[1]) << 16) |
           (static_cast<u32>(p[2]) << 8) | p[3];
}

inline void zStore(const Pipe& P, u64 at, u32 v)
{
    u8* p = P.c->vram.data() + at;
    if (P.zw == 16u) {
        p[0] = static_cast<u8>(v >> 8);
        p[1] = static_cast<u8>(v);
    } else {
        p[0] = static_cast<u8>(v >> 24);
        p[1] = static_cast<u8>(v >> 16);
        p[2] = static_cast<u8>(v >> 8);
        p[3] = static_cast<u8>(v);
    }
}

inline bool cmp8(u32 fn, u32 src, u32 ref)
{
    switch (fn) {
    case 0u: return false;
    case 1u: return src < ref;
    case 2u: return src <= ref;
    case 3u: return src == ref;
    case 4u: return src >= ref;
    case 5u: return src > ref;
    case 6u: return src != ref;
    default: return true;
    }
}

inline u32 stencilOp(u32 op, u32 cur, u32 ref)
{
    switch (op) {
    case 1u: return 0;
    case 2u: return ref;
    case 3u: return cur == 0xFFu ? 0xFFu : cur + 1u; // saturating, the safer
    case 4u: return cur == 0u ? 0u : cur - 1u;       // reading of inc/dec
    case 5u: return (~cur) & 0xFFu;
    default: return cur;
    }
}

// ── Blend factors (Tables 6-17/6-18, enumerants per r128_reg.h) ───────────
inline F4 blendFactor(u32 sel, F4 s, F4 d, bool isSrc)
{
    const F4 k255 = F4::splat(255.0f);
    switch (sel) {
    case 0u: return F4::splat(0.0f);
    case 2u: return s;
    case 3u: return k255 - s;
    case 4u: return F4::splat(s.a());
    case 5u: return F4::splat(255.0f - s.a());
    case 6u: return F4::splat(d.a());
    case 7u: return F4::splat(255.0f - d.a());
    case 8u: return d;
    case 9u: return k255 - d;
    case 10u: { // SRCALPHASAT: (f,f,f,1), f = min(As, 1-Ad)
        const float sa = s.a(), da = d.a();
        const float m = sa < 255 - da ? sa : 255 - da;
        return F4::splat(m).withA(255.0f);
    }
    case 11u: // BOTHSRCALPHA: src=As, forces dst=1-As (caller handles pair)
        return isSrc ? F4::splat(s.a()) : F4::splat(255.0f - s.a());
    case 12u: // BOTHINVSRCALPHA
        return isSrc ? F4::splat(255.0f - s.a()) : F4::splat(s.a());
    default: return k255; // 1: ONE
    }
}

// ── Interpolated fragment attributes handed to the shader ─────────────────
struct Frag {
    // The two colors first: they are held to a register's alignment, and
    // eight floats after them leave no padding anywhere.
    F4 diffuse = F4::set(255, 0, 0, 0), spec = F4::set(255, 0, 0, 0);
    float z = 0;
    float fog = 255;
    float s1 = 0, t1 = 0, s2 = 0, t2 = 0; // perspective-resolved
    float lod1 = 0, lod2 = 0;
};

// The whole per-pixel back end. False = fragment rejected before the write.
// In line in the triangle's pixel loop, where it is nearly all of the work;
// points and lines call the one copy below.
OPM_FORCE_INLINE bool shadeFragmentIn(const Pipe& P, int px, int py,
                                      const Frag& fr)
{
    const F4 diffuse = fr.diffuse;
    F4 color = diffuse;
    if (P.texAny) {
        const TexUnit& t0 = P.tex[0];
        const F4 tex0 = sampleTex(P, t0, fr.s1, fr.t1, fr.lod1);
        if (P.chromaEn) {
            // Texture chroma key: a texel matching CLR_CMP through the mask
            // kills the fragment. Compared in ARGB8888 space.
            float c[4];
            tex0.store(c);
            const u32 t32 = (static_cast<u32>(c[0] + 0.5f) << 24) |
                            (static_cast<u32>(c[1] + 0.5f) << 16) |
                            (static_cast<u32>(c[2] + 0.5f) << 8) |
                            static_cast<u32>(c[3] + 0.5f);
            if ((t32 & P.chromaMsk) == (P.chromaClr & P.chromaMsk))
                return false;
        }
        // ⭐ THE TEXEL'S ALPHA AS A CUTOUT (TEX_CNTL_C bit 13). r128_reg.h
        // names the bit's two values ALPHA_IN_TEX_COMPLETE_A and
        // ALPHA_IN_TEX_LSB_A; the SDK's field tables stop at bit 10 and
        // neither register guide prints the register. What it does is
        // pinned by a driver and a picture: the Mac driver sets it, with
        // ALPHA_EN and ALPHA_TST_EN both clear, for the draws whose ARGB1555
        // texels carry a shape — flat menu buttons, each a quad — and on the
        // real machine the texels with their alpha bit clear are not drawn.
        // With the bit ignored every such quad drew its whole rectangle, the
        // cut-away part in the texels' own black.
        //
        // So: the texture's alpha is one bit, and a sample whose bit is
        // clear writes nothing — no color, no depth. The sample's alpha is
        // taken after filtering and rounded to that bit, which is what
        // leaves a bilinear sample on the shape's edge its darkened rim.
        // ⚠ That last choice is this model's: nothing here says whether the
        // chip decides from the filtered alpha or from the nearest texel's.
        if (P.alphaMask && tex0.a() < 127.5f)
            return false;
        F4 out = combineUnit(P, t0, tex0, diffuse, diffuse);
        if (P.tex[1].on) {
            const TexUnit& t1 = P.tex[1];
            const float s = t1.useSecondSt ? fr.s2 : fr.s1;
            const float tt = t1.useSecondSt ? fr.t2 : fr.t1;
            const F4 tex1 =
                sampleTex(P, t1, s, tt, t1.useSecondSt ? fr.lod2 : fr.lod1);
            out = combineUnit(P, t1, tex1, diffuse, out);
        }
        color = texLight(P, out, diffuse);
    }
    if (P.specEn) // the three colors; alpha is not lit
        color = (color + fr.spec).clamped().withA(color.a());
    if (P.fogEn) {
        // f=255 is "all primitive color". Vertex fog interpolates the
        // specular alpha (SDK's own definition); table fog indexes the
        // 256-entry table by z.
        float f;
        if (P.fogTable) {
            int idx = static_cast<int>(fr.z * 255.0f + 0.5f);
            idx = idx < 0 ? 0 : idx > 255 ? 255 : idx;
            f = static_cast<float>(gFog.t[idx]);
        } else {
            f = fr.fog;
        }
        color = ((color * F4::splat(f) +
                  F4::of(P.fogC) * F4::splat(255.0f - f)) /
                 F4::splat(255.0f))
                    .withA(color.a());
    }
    if (P.alphaTestEn &&
        !cmp8(P.alphaOp, static_cast<u32>(clamp255(color.a()) + 0.5f),
              P.refAlpha))
        return false;
    // Z + stencil, evaluated together because the stencil result depends on
    // the Z verdict (S8 shares the Z24 cell).
    u64 za = 0;
    bool zPass = true;
    const bool wantZ = P.zEn || (P.stEn && P.zw == 24u);
    if (wantZ) {
        if (!zAddr(P, px, py, za)) {
            ++P.tally.offVram;
            return false;
        }
        const u32 cell = zRead(P, za);
        const u32 zdst = P.zw == 24u ? cell & 0xFFFFFFu : cell;
        double zf = fr.z;
        zf = zf < 0.0 ? 0.0 : zf > 1.0 ? 1.0 : zf;
        const u32 zsrc =
            P.zw == 16u
                ? static_cast<u32>(zf * 65535.0 + 0.5)
                : P.zw == 24u ? static_cast<u32>(zf * 16777215.0 + 0.5)
                              : static_cast<u32>(zf * 4294967295.0);
        if (P.stEn && P.zw == 24u) {
            const u32 sCur = cell >> 24;
            const bool sPass =
                cmp8(P.stTest, P.stRef & P.stMask, sCur & P.stMask);
            zPass = !P.zEn || cmp8(P.zTest, zsrc, zdst);
            const u32 op = !sPass ? P.stSFail
                                  : zPass ? P.stZPass : P.stZFail;
            const u32 sNew = stencilOp(op, sCur, P.stRef);
            const u32 sOut =
                (sCur & ~P.stWrMask) | (sNew & P.stWrMask);
            u32 zOut = zdst;
            if (sPass && zPass && P.zEn && P.zWr)
                zOut = zsrc & 0xFFFFFFu;
            zStore(P, za, (sOut << 24) | zOut);
            if (!sPass || !zPass)
                return false;
        } else if (P.zEn) {
            zPass = cmp8(P.zTest, zsrc, zdst);
            if (!zPass)
                return false;
            if (P.zWr)
                zStore(P, za, P.zw == 24u ? (cell & 0xFF000000u) |
                                                (zsrc & 0xFFFFFFu)
                                          : zsrc);
        }
    }
    const u64 at = static_cast<u64>(P.dstOff) +
                   static_cast<u64>(py) * P.dstStride +
                   static_cast<u64>(px) * P.bypp;
    if (at + P.bypp > P.c->vram.size()) {
        ++P.tally.offVram;
        return false;
    }
    if (P.blendEn) {
        const F4 dst = dstDecode(P, dstRead(P, at));
        const F4 fs = blendFactor(P.blendSrc, color, dst, true);
        // The BOTH* source selections force the destination factor; the
        // guest's own dst field is ignored then (Table 6-17's wording).
        const F4 fd = (P.blendSrc == 11u || P.blendSrc == 12u)
                          ? blendFactor(P.blendSrc, color, dst, false)
                          : blendFactor(P.blendDst, color, dst, true);
        const F4 k255 = F4::splat(255.0f);
        F4 v = P.blendComb >= 2u ? (color * fs - dst * fd) / k255
                                 : (color * fs + dst * fd) / k255;
        if (!(P.blendComb & 1u))
            v = v.clamped();
        color = v;
    }
    dstWrite(P, at, dstEncode(P, color));
    ++P.tally.triPixels;
    return true;
}

bool shadeFragment(const Pipe& P, int px, int py, const Frag& fr)
{
    return shadeFragmentIn(P, px, py, fr);
}

// ── Triangle rasterizer ───────────────────────────────────────────────────
//
// Screen-space edge functions, D3D top-left fill rule, pixel centers at
// +0.5. Colors interpolate screen-linear (the era's hardware did not
// perspective-correct Gouraud); texture S,T arrive as premultiplied
// numerators, interpolate screen-linearly alongside rhw, and divide per
// pixel unless the unit disables perspective.
void rasterPoint(const Pipe& P, const Vtx& v);
void rasterLine(const Pipe& P, const Vtx& a, const Vtx& b);

struct Plane {
    double a = 0, b = 0, c0 = 0;
    double at(double x, double y) const { return a * x + b * y + c0; }
};

Plane planeOf(double x0, double y0, double x1, double y1, double x2,
              double y2, double v0, double v1, double v2, double inv2a)
{
    Plane p;
    p.a = (v0 * (y1 - y2) + v1 * (y2 - y0) + v2 * (y0 - y1)) * inv2a;
    p.b = (v0 * (x2 - x1) + v1 * (x0 - x2) + v2 * (x1 - x0)) * inv2a;
    // Anchor at vertex 0: c = v0 - a*x0 - b*y0.
    p.c0 = v0 - p.a * x0 - p.b * y0;
    return p;
}

// One edge of a triangle as a function of the pixel center, and everything
// else about a triangle its rows are drawn from: found once (rasterTriangle),
// read by every row.
struct TriEdge {
    double a, b, c; // e = a*x + b*y + c, interior > 0 for CW/y-down
    bool topLeft;
};

struct TriSetup {
    TriEdge e[3];
    Plane pz, pa, pr, pg, pb, psr, psg, psb, pfog, pw, ps1, pt1, ps2, pt2;
    // The four colors' planes side by side, and the specular three, for
    // F4::affine: one step along the row for all of them.
    double colA[4], specA[4];
    bool persp0, persp1, wantSpec, wantFog, lodLive;
    bool lodSignOnly, lodNoBias; // see where the level is computed
    int minX, maxX;
};

// Rows yFrom..yTo of a triangle, every yStep-th. Nothing a row draws depends
// on another row of the same triangle except through what both may touch in
// memory and the dither's error register; RowQueue::admit says when neither
// can happen, and then the rows may be drawn in any order, or at once.
void drawRows(const Pipe& P, const TriSetup& T, int yFrom, int yTo,
              int yStep = 1)
{
    // Copies, on purpose: pixels are stored through byte pointers, which a
    // compiler has to assume might be T itself, and it would read every
    // plane again after every pixel.
    const TriEdge e[3] = {T.e[0], T.e[1], T.e[2]};
    const Plane pz = T.pz, pa = T.pa, pr = T.pr, pg = T.pg, pb = T.pb;
    const Plane psr = T.psr, psg = T.psg, psb = T.psb, pfog = T.pfog;
    const Plane pw = T.pw, ps1 = T.ps1, pt1 = T.pt1, ps2 = T.ps2, pt2 = T.pt2;
    const double colA[4] = {T.colA[0], T.colA[1], T.colA[2], T.colA[3]};
    const double specA[4] = {T.specA[0], T.specA[1], T.specA[2], T.specA[3]};
    const bool persp0 = T.persp0, persp1 = T.persp1;
    const bool wantSpec = T.wantSpec, wantFog = T.wantFog;
    const bool lodLive = T.lodLive;
    const bool lodSignOnly = T.lodSignOnly, lodNoBias = T.lodNoBias;
    const int minX = T.minX, maxX = T.maxX;
    for (int py = yFrom; py <= yTo; py += yStep) {
        const double cy = py + 0.5;
        ditherLineStart(P);
        // Per-row bases: value at (cx,cy) = plane.a * cx + rowBase. One
        // multiply-add per attribute per pixel instead of two multiplies
        // and two adds. (The EDGE functions below keep their original
        // form on purpose: reassociating them could flip an exactly-zero
        // verdict and move rasterization coverage by a pixel.)
        const double zRB = pz.b * cy + pz.c0;
        const double aRB = pa.b * cy + pa.c0;
        const double rRB = pr.b * cy + pr.c0;
        const double gRB = pg.b * cy + pg.c0;
        const double bRB = pb.b * cy + pb.c0;
        const double srRB = psr.b * cy + psr.c0;
        const double sgRB = psg.b * cy + psg.c0;
        const double sbRB = psb.b * cy + psb.c0;
        const double fogRB = pfog.b * cy + pfog.c0;
        const double wRB = pw.b * cy + pw.c0;
        const double s1RB = ps1.b * cy + ps1.c0;
        const double t1RB = pt1.b * cy + pt1.c0;
        const double s2RB = ps2.b * cy + ps2.c0;
        const double t2RB = pt2.b * cy + pt2.c0;
        const double colRB[4] = {aRB, rRB, gRB, bRB};
        const double specRB[4] = {0.0, srRB, sgRB, sbRB};
        // ⭐ THE ROW'S PIXELS, FOUND WITHOUT VISITING THE REST OF THE BOX.
        // A pixel is inside when all three edge functions pass at its
        // center. Along a row each is e.a * cx + (a row's constant): in
        // floating point as in arithmetic that never turns back as cx
        // grows (a product by a fixed number, and a sum with one, both
        // round in order), so the pixels an edge passes are everything from
        // some pixel on (e.a > 0), everything up to one (e.a < 0), or the
        // whole row or none of it (e.a == 0). The three together are one
        // run, lo..hi. Each end is guessed from the edge's crossing and then
        // settled by `inside`, which is the function itself, written as it
        // always was; the guess only decides how few steps that takes. The
        // same pixels as testing every one in the box, in the same order.
        const auto inside = [&](int k, int px) {
            const double cx = px + 0.5;
            const double ev = e[k].a * cx + e[k].b * cy + e[k].c;
            return ev > 0.0 || (ev == 0.0 && e[k].topLeft);
        };
        int lo = minX, hi = maxX;
        for (int k = 0; k < 3 && lo <= hi; ++k) {
            const double a = e[k].a;
            if (a == 0.0) {
                if (!inside(k, lo))
                    hi = lo - 1;
                continue;
            }
            // Where the edge crosses this row, as a pixel index; anything
            // that is not a number inside the row becomes one of its ends.
            const double cross = -(e[k].b * cy + e[k].c) / a - 0.5;
            if (a > 0.0) { // passes from some pixel on
                int g = !(cross > lo) ? lo
                        : cross > hi ? hi + 1
                                     : static_cast<int>(cross) + 1;
                while (g > lo && inside(k, g - 1))
                    --g;
                while (g <= hi && !inside(k, g))
                    ++g;
                lo = g;
            } else { // passes up to some pixel (and a NaN passes nowhere)
                int g = !(cross >= lo) ? lo - 1
                        : cross > hi ? hi
                                     : static_cast<int>(cross);
                while (g < hi && inside(k, g + 1))
                    ++g;
                while (g >= lo && !inside(k, g))
                    --g;
                hi = g;
            }
        }
        for (int px = lo; px <= hi; ++px) {
            const double cx = px + 0.5;
            Frag fr;
            fr.z = static_cast<float>(pz.a * cx + zRB);
            fr.diffuse = F4::affine(colA, cx, colRB).clamped();
            if (wantSpec) // its alpha lane is nothing: specular has none
                fr.spec = F4::affine(specA, cx, specRB).clamped();
            if (wantFog)
                fr.fog = clamp255(static_cast<float>(pfog.a * cx + fogRB));
            const double w = pw.a * cx + wRB;
            const double invW = (persp0 || persp1) && w != 0.0 ? 1.0 / w : 1.0;
            if (P.texAny) {
                const double s1n = ps1.a * cx + s1RB;
                const double t1n = pt1.a * cx + t1RB;
                fr.s1 = static_cast<float>(persp0 ? s1n * invW : s1n);
                fr.t1 = static_cast<float>(persp0 ? t1n * invW : t1n);
                if (P.tex[1].on) {
                    const double s2n = ps2.a * cx + s2RB;
                    const double t2n = pt2.a * cx + t2RB;
                    fr.s2 = static_cast<float>(persp1 ? s2n * invW : s2n);
                    fr.t2 = static_cast<float>(persp1 ? t2n * invW : t2n);
                }
            }
            // Per-pixel LOD from the analytic derivatives of s,t: for
            // S = Sn/W with Sn, W screen-affine,
            // dS/dx = (Sn'x·W − Sn·W'x)/W².
            if (lodLive) {
                const double sw = static_cast<double>(P.tex[0].wTex);
                const double sh = static_cast<double>(P.tex[0].hTex);
                double dsx, dsy, dtx, dty;
                if (persp0 && w != 0.0) {
                    const double w2 = 1.0 / (w * w);
                    const double s1n = ps1.a * cx + s1RB;
                    const double t1n = pt1.a * cx + t1RB;
                    dsx = (ps1.a * w - s1n * pw.a) * w2;
                    dsy = (ps1.b * w - s1n * pw.b) * w2;
                    dtx = (pt1.a * w - t1n * pw.a) * w2;
                    dty = (pt1.b * w - t1n * pw.b) * w2;
                } else {
                    dsx = ps1.a; dsy = ps1.b; dtx = pt1.a; dty = pt1.b;
                }
                const double rx = dsx * sw * dsx * sw + dtx * sh * dtx * sh;
                const double ry = dsy * sw * dsy * sw + dty * sh * dty * sh;
                const double rho2 = rx > ry ? rx : ry;
                // The level is half the logarithm, and the logarithm is the
                // dearest thing in this block. sampleTex asks two questions
                // of it: is it above zero (which filter, and whether a
                // smaller map is used at all), and, only then and only with
                // smaller maps to choose from, what it is. Above zero is
                // rho2 above one, exactly: a logarithm rounds to zero for no
                // other number. So wherever the answer to the first settles
                // it, a level of the same sign stands in. Not with a bias,
                // which moves the question off zero.
                if (!(rho2 > 0.0))
                    fr.lod1 = 0.0f;
                else if (lodSignOnly)
                    fr.lod1 = rho2 > 1.0 ? 1.0f : 0.0f;
                else if (lodNoBias && rho2 <= 1.0)
                    fr.lod1 = 0.0f;
                else
                    fr.lod1 = static_cast<float>(0.5 * std::log2(rho2));
                if (P.tex[1].on)
                    fr.lod2 = fr.lod1; // second set: same footprint model
            }
            shadeFragmentIn(P, px, py, fr);
        }
    }
}

// ── Triangles queued, their rows dealt to lanes ───────────────────────────
//
// At a high resolution drawing is nearly all of the card's work, and a guest
// that waits for the card waits for one thread. A triangle's rows share
// nothing with each other (admit, below, says when), so rows can be drawn at
// once; and a row of the screen shares nothing with another row whatever
// triangle draws it. So the screen's rows are dealt to LANES, two rows at a
// time, lane after lane and round again, and a triangle is not drawn where
// it is met but queued. A lane is drawn by one thread at a time, whichever
// takes it, and always in the order of the queue: within a row the triangles
// land in the order the guest sent them, as they always did. Between rows
// the order in time differs, which nothing can see. The pixels are the same
// pixels with the same values.
//
// The thread that executes the packets goes on decoding and setting up while
// the others draw. It joins them only where something is about to look at
// what they produce or change what they read (settle): a 2D operation, a
// register that is not plain 3D state, a palette or fog table load, a
// primitive that cannot be queued, the end of the stream. Every exit of the
// packet loop settles, so outside it the queue is always empty.
//
// There are more lanes than threads, and no lane is any thread's own: one
// that is slow, or that the host has taken off its processor, holds up the
// few triangles of the one lane it has and nothing else.
//
// A queued triangle carries its packet's state (one copy a packet), so later
// state writes do not reach it. Each thread draws from its own record: the
// triangle's state taken into it, its own counts, added up when the queue
// settles, and its own cache of texture pages. The threads are started once
// and kept; with nothing to take they spin briefly, because triangles come
// in runs, and then sleep.

inline void rowsRelax()
{
#if OPM_ROWS_X86
    _mm_pause();
#else
    std::this_thread::yield();
#endif
}

// How many threads draw queued triangles, the one executing the packets
// among them (it draws while it settles). 0 = not set: OPM_RASTER_THREADS,
// or a number from the host's processors. 1 = nothing is queued.
std::atomic<int> gRowThreads{0};

int rowThreadsDefault()
{
    if (const char* v = std::getenv("OPM_RASTER_THREADS")) {
        const int n = std::atoi(v);
        if (n >= 1)
            return n;
    }
    // The processor's thread is busy already, and this one between the
    // settles: a small host keeps to that, and a large one keeps a few
    // processors for everything else. Measured on sixteen cores with two
    // textures at 1024x768: three frames drawn in 204 ms on one thread, 38
    // on six, 23.5 on twelve.
    const unsigned hw = std::thread::hardware_concurrency();
    return hw >= 32u ? 12 : hw >= 24u ? 8 : hw >= 12u ? 6 : hw >= 8u ? 4
           : hw >= 4u ? 2 : 1;
}

class RowQueue {
  public:
    static constexpr int kMax = 16;
    // Never destroyed: its threads live as long as the process, and one that
    // is asleep when the process leaves must not find its lock gone.
    static RowQueue& get()
    {
        static RowQueue* const q = new RowQueue;
        return *q;
    }
    static int threads()
    {
        static const int byDefault = rowThreadsDefault();
        const int set = gRowThreads.load(std::memory_order_relaxed);
        const int n = set > 0 ? set : byDefault;
        return n > kMax ? kMax : n;
    }

    // Whether this triangle may wait in the queue behind what is there:
    // what one of its rows writes no other row reads or writes, and the same
    // holds between it and every triangle already waiting.
    bool admit(const Pipe& P, int maxX, int minY, int maxY)
    {
        // The error a dithered pixel leaves for the next crosses rows, the
        // pixel watch and the triangle dump report in the order of drawing,
        // and memory that is not read in place is asked for under a lock.
        if (P.ditherDiffuse || gR128PwHi != 0u || !P.fastMem || gTriDump > 0)
            return false;
        const u64 cols = static_cast<u64>(maxX) + 1u;
        // The destination's rows, and the depth buffer's, each within its
        // own stride.
        if (minY < 0 || cols * P.bypp > P.dstStride)
            return false;
        Reach r = reach_;
        const bool zUsed = P.zEn || (P.stEn && P.zw == 24u);
        const u64 zb = P.zw == 16u ? 2u : 4u;
        if (zUsed && cols * zb > P.zStride)
            return false;
        // One destination and one depth buffer for everything waiting: a row
        // of either is the same lane's whichever triangle touches it.
        if (pending_) {
            if (r.dstOff != P.dstOff || r.dstStride != P.dstStride ||
                r.bypp != P.bypp)
                return false;
            if (zUsed && r.zSeen &&
                (r.zOff != P.zOff || r.zStride != P.zStride || r.zb != zb))
                return false;
        } else {
            r = Reach{};
            r.dstOff = P.dstOff;
            r.dstStride = P.dstStride;
            r.bypp = P.bypp;
        }
        const auto widen = [](u64& lo, u64& hi, u64 a, u64 b) {
            if (lo == hi) {
                lo = a;
                hi = b;
            } else {
                lo = a < lo ? a : lo;
                hi = b > hi ? b : hi;
            }
        };
        widen(r.dLo, r.dHi,
              static_cast<u64>(P.dstOff) +
                  static_cast<u64>(minY) * P.dstStride,
              static_cast<u64>(P.dstOff) +
                  (static_cast<u64>(maxY) + 1u) * P.dstStride);
        if (zUsed) {
            r.zSeen = true;
            r.zOff = P.zOff;
            r.zStride = P.zStride;
            r.zb = zb;
            widen(r.zLo, r.zHi,
                  static_cast<u64>(P.zOff) +
                      static_cast<u64>(minY) * P.zStride,
                  static_cast<u64>(P.zOff) +
                      (static_cast<u64>(maxY) + 1u) * P.zStride);
        }
        // Every texture in video memory that anything waiting reads, as one
        // stretch. A level's extent is taken generously: its rows at their
        // pitch, four bytes a texel. System memory is never the frame
        // buffer.
        if (P.texAny)
            for (const TexUnit& t : P.tex) {
                if (!t.on)
                    continue;
                for (u32 l = 0; l <= t.maxLevel && l < 11u; ++l) {
                    const u64 base = t.offs[l] & 0x07FFFFFFu;
                    if (base >= kAgpWindow)
                        continue;
                    const u64 pitch = t.pitchTex >> l ? t.pitchTex >> l : 1u;
                    const u64 h = t.hTex >> l ? t.hTex >> l : 1u;
                    widen(r.tLo, r.tHi, base, base + (pitch * h + 4u) * 4u);
                }
            }
        // The three stretches apart from each other: what is drawn, the
        // depths, and what is read.
        const auto meet = [](u64 aLo, u64 aHi, u64 bLo, u64 bHi) {
            return aLo < bHi && bLo < aHi;
        };
        if (meet(r.dLo, r.dHi, r.zLo, r.zHi) ||
            meet(r.dLo, r.dHi, r.tLo, r.tHi) ||
            meet(r.zLo, r.zHi, r.tLo, r.tHi))
            return false;
        reach_ = r;
        return true;
    }

    // Queue an admitted triangle. One caller at a time: whoever executes the
    // packets.
    void add(const Pipe& P, const TriSetup& T, int minY, int maxY)
    {
        if (!pending_) {
            lone_ = 0.0;
            const int want = threads() - 1;
            if (static_cast<int>(crew_.size()) < want) {
                std::lock_guard<std::mutex> lk(mu_);
                while (static_cast<int>(crew_.size()) < want) {
                    crew_.emplace_back([this] { work(); });
                    crew_.back().detach();
                }
            }
        }
        // Room: a place in the ring is free once every lane has passed the
        // job that was in it. Short of room, the caller draws too.
        while (posted_ - passed_ >= kJobs) {
            u64 least = posted_;
            for (const Lane& l : lane_) {
                const u64 d = l.done.load(std::memory_order_acquire);
                least = d < least ? d : least;
            }
            passed_ = least;
            if (posted_ - passed_ >= kJobs && !pull(mine_, posted_))
                rowsRelax();
        }
        if (P.queued == ~0ull) {
            pipes_[pipeAt_ & (kJobs - 1u)] = P; // its state alone
            P.queued = pipeAt_++;
        }
        Job& j = jobs_[posted_ & (kJobs - 1u)];
        j.pipe = P.queued;
        j.batch = batch_;
        j.minY = minY;
        j.maxY = maxY;
#if OPM_ROWS_X86
        j.csr = _mm_getcsr();
#endif
        j.T = T;
        pending_ = true;
        gR128RowsPending = true;
        ++posted_;
        postedSeen_.store(posted_);
        if (sleepers_.load() > 0) {
            // Through the lock, so that a thread between its last look at
            // the queue and its sleep cannot miss this.
            { std::lock_guard<std::mutex> lk(mu_); }
            wake_.notify_all();
        }
    }

    // Draw with the others until everything queued is drawn, and take the
    // counts.
    void settle()
    {
        if (!pending_)
            return;
        const auto t0 = std::chrono::steady_clock::now();
#if OPM_ROWS_X86
        const unsigned csr = _mm_getcsr();
        mine_.csr = csr;
#endif
        for (int spins = 0;;) {
            if (pull(mine_, posted_)) {
                spins = 0;
                continue;
            }
            bool all = true;
            for (const Lane& l : lane_)
                if (l.done.load(std::memory_order_acquire) != posted_) {
                    all = false;
                    break;
                }
            if (all)
                break;
            // What is left is in lanes other threads hold.
            if (++spins < 20000)
                rowsRelax();
            else
                std::this_thread::yield();
        }
#if OPM_ROWS_X86
        _mm_setcsr(csr);
#endif
        passed_ = posted_;
        auto& es = eng();
        ++es.rowSettles;
        es.rowSettleNs += static_cast<u64>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - t0)
                .count());
        // Every lane stands at what was posted, so nobody is writing these.
        for (Lane& l : lane_) {
            es.triPixels += l.tally.triPixels;
            es.crewPixels += l.tally.triPixels;
            es.offVram += l.tally.offVram;
            es.texSamples += l.tally.texSamples;
            es.texUnimpl += l.tally.texUnimpl;
            l.tally = Pipe::Tally{};
        }
        pending_ = false;
        gR128RowsPending = false;
        ++batch_;
    }

    bool pending() const { return pending_; }
    // Whether an empty queue should be started for a triangle of about
    // `pixels`. Waking a thread costs the caller several microseconds, which
    // one small triangle does not repay; so: for a large one, for a small
    // one if a drawing thread is awake with nothing to do, and for a small
    // one once the small ones drawn here since the queue was last empty
    // come to a large one between them.
    bool worth(double pixels)
    {
        lone_ += pixels;
        if (lone_ >= kStartPixels)
            return true;
        return pixels >= kStartHotPixels && !crew_.empty() &&
               sleepers_.load(std::memory_order_relaxed) <
                   static_cast<int>(crew_.size());
    }

  private:
    // Powers of two: a job's place is its number's low bits, a band's lane
    // its number's.
    static constexpr u64 kJobs = 1024;
    static constexpr u32 kLanes = 32;
    static constexpr double kStartPixels = 2048.0, kStartHotPixels = 64.0;
    static constexpr int kBandShift = 1; // rows to a band, as a shift
    // Jobs a thread draws of one lane before it lets the lane go and looks
    // for the one furthest behind.
    static constexpr u64 kTake = 64;
    struct Job {
        u64 pipe = 0;
        u32 batch = 0;
        int minY = 0, maxY = 0;
        unsigned csr = 0;
        TriSetup T;
    };
    // What everything waiting draws into and reads from, in video memory.
    struct Reach {
        u32 dstOff = 0, dstStride = 0, bypp = 0;
        bool zSeen = false;
        u32 zOff = 0, zStride = 0;
        u64 zb = 0;
        u64 dLo = 0, dHi = 0, zLo = 0, zHi = 0, tLo = 0, tHi = 0;
    };
    struct alignas(64) Lane {
        // Taken by one thread at a time. `done`, the jobs this lane has
        // passed, and `tally`, what was counted drawing them, are the
        // holder's to write: the counts before the number.
        std::atomic<u32> held{0};
        std::atomic<u64> done{0};
        Pipe::Tally tally;
    };
    // One thread's own: the record it draws from, which state is in it, the
    // batch its page cache belongs to, and where it looks for a lane next.
    struct Hand {
        Pipe pipe;
        u64 pipeHeld = ~0ull;
        u32 batch = ~0u;
        u32 next = 0;
        unsigned csr = 0;
    };

    // Take a lane that is behind and that nobody holds, and draw what waits
    // in it, up to kTake jobs. False if there was none to take.
    bool pull(Hand& h, u64 posted)
    {
        for (u32 i = 0; i < kLanes; ++i) {
            const u32 c = (h.next + i) & (kLanes - 1u);
            Lane& lane = lane_[c];
            if (lane.done.load(std::memory_order_relaxed) >= posted ||
                lane.held.exchange(1u, std::memory_order_acquire) != 0u)
                continue;
            u64 at = lane.done.load(std::memory_order_relaxed);
            if (at >= posted) { // it caught up between the look and the take
                lane.held.store(0u, std::memory_order_release);
                continue;
            }
            const u64 end = posted - at > kTake ? at + kTake : posted;
            h.pipe.tally = Pipe::Tally{};
            for (; at < end; ++at) {
                const Job& j = jobs_[at & (kJobs - 1u)];
                // This lane's bands of the triangle: band b is rows
                // b << kBandShift onward, in lane b mod kLanes.
                const int first = j.minY >> kBandShift;
                int b = first + static_cast<int>(
                                    (c - static_cast<u32>(first)) &
                                    (kLanes - 1u));
                if ((b << kBandShift) > j.maxY)
                    continue;
                if (h.pipeHeld != j.pipe) {
                    static_cast<PipeState&>(h.pipe) =
                        pipes_[j.pipe & (kJobs - 1u)];
                    h.pipe.crowd = true;
                    h.pipeHeld = j.pipe;
                }
                // Texture pages found stay found while a batch lasts:
                // nothing a walk reads changes under a stream.
                if (h.batch != j.batch) {
                    h.batch = j.batch;
                    for (Pipe::AgpPage& e : h.pipe.agp)
                        e = Pipe::AgpPage{};
                }
#if OPM_ROWS_X86
                // The caller's rounding and exception masks, whatever they
                // are: the same arithmetic on every thread.
                if (h.csr != j.csr) {
                    h.csr = j.csr;
                    _mm_setcsr(j.csr);
                }
#endif
                for (; (b << kBandShift) <= j.maxY;
                     b += static_cast<int>(kLanes)) {
                    const int top = b << kBandShift;
                    const int bottom = top + (1 << kBandShift) - 1;
                    drawRows(h.pipe, j.T, top < j.minY ? j.minY : top,
                             bottom > j.maxY ? j.maxY : bottom);
                }
            }
            lane.tally.triPixels += h.pipe.tally.triPixels;
            lane.tally.offVram += h.pipe.tally.offVram;
            lane.tally.texSamples += h.pipe.tally.texSamples;
            lane.tally.texUnimpl += h.pipe.tally.texUnimpl;
            lane.done.store(at, std::memory_order_release);
            lane.held.store(0u, std::memory_order_release);
            h.next = c + 1u;
            return true;
        }
        return false;
    }

    void work()
    {
        auto hand = std::make_unique<Hand>();
        Hand& h = *hand;
#if OPM_ROWS_X86
        h.csr = _mm_getcsr();
#endif
        for (;;) {
            const u64 posted = postedSeen_.load(std::memory_order_acquire);
            if (pull(h, posted))
                continue;
            // Nothing to take. Awake for a moment: triangles come in runs.
            const auto until = std::chrono::steady_clock::now() + kAwake;
            bool more = false;
            while (!more) {
                for (int i = 0; i < 32; ++i)
                    rowsRelax();
                more = pull(h, postedSeen_.load(std::memory_order_acquire));
                if (std::chrono::steady_clock::now() >= until)
                    break;
            }
            if (more)
                continue;
            std::unique_lock<std::mutex> lk(mu_);
            sleepers_.fetch_add(1);
            wake_.wait(lk, [&] { return postedSeen_.load() != posted; });
            sleepers_.fetch_sub(1);
        }
    }

    std::mutex mu_;
    std::condition_variable wake_;
    std::vector<std::thread> crew_; // grown by the caller only
    std::atomic<int> sleepers_{0};
    // The caller's: jobs posted, jobs every lane is known to have passed,
    // states copied, the batch's number, whether anything waits, and what it
    // reaches.
    u64 posted_ = 0, passed_ = 0, pipeAt_ = 0;
    u32 batch_ = 0;
    bool pending_ = false;
    double lone_ = 0.0; // pixels drawn by the caller since the queue emptied
    Reach reach_;
    Hand mine_; // the caller's own, for when it draws
    // `posted_` again, for the threads.
    alignas(64) std::atomic<u64> postedSeen_{0};
    Lane lane_[kLanes];
    std::vector<Job> jobs_ = std::vector<Job>(kJobs);
    std::vector<PipeState> pipes_ = std::vector<PipeState>(kJobs);
    static constexpr std::chrono::microseconds kAwake{200};
};

// Everything queued is drawn before the caller goes on (see RowQueue).
inline void rowsSettle()
{
    if (gR128RowsPending)
        RowQueue::get().settle();
}

// The pixel path's counts, into the engine's statistics: once a packet.
void flushTally(const Pipe& P)
{
    auto& es = eng();
    es.triPixels += P.tally.triPixels;
    es.offVram += P.tally.offVram;
    es.texSamples += P.tally.texSamples;
    es.texUnimpl += P.tally.texUnimpl;
    P.tally = Pipe::Tally{};
}

void rasterTriangle(const Pipe& P, Vtx v0, Vtx v1, Vtx v2, bool flipFacing)
{
    // Facing. In y-down screen space a clockwise triangle has positive
    // doubled area; PM4_VC_FPU_SETUP bit 0 names which winding is front,
    // bits 2:1 / 4:3 what to do with back / front faces (Table 6-22).
    double area2 = (static_cast<double>(v1.x) - v0.x) * (v2.y - v0.y) -
                   (static_cast<double>(v2.x) - v0.x) * (v1.y - v0.y);
    if (area2 == 0.0) {
        ++eng().triDegen;
        return;
    }
    // The per-triangle S/T span against the bound texture size — the
    // measurement that tells normalized coordinates from texel ones. Taken
    // before any culling so the sample is of what the guest SENT.
    if (P.texAny && P.tex[0].wTex) {
        const float sMin = std::min(v0.s1, std::min(v1.s1, v2.s1));
        const float sMax = std::max(v0.s1, std::max(v1.s1, v2.s1));
        const float tMin = std::min(v0.t1, std::min(v1.t1, v2.t1));
        const float tMax = std::max(v0.t1, std::max(v1.t1, v2.t1));
        const float span = std::max(sMax - sMin, tMax - tMin);
        if (span >= 0.0f && span < 1e6f) {
            auto& e = eng();
            int b = 0;
            if (span > 0.0f) {
                const float l2 = std::log2(span);
                b = static_cast<int>(std::floor(l2)) + 5; // 2^-5 -> 0
                if (b < 0) b = 0;
                if (b > 11) b = 11;
            }
            ++e.triSpanBucket[b];
            e.triSpanTexW += P.tex[0].wTex;
            ++e.triSpanN;
        }
    }
    ++gTriSeq;
    if (gTriDump > 0 && P.texAny) {
        --gTriDump;
        const TexUnit& t0 = P.tex[0];
        printf("-- tri3d %llu fmt=%03x prim=%u t0 %ux%u p%u f=%u cn=%08x "
               "of=%08x tc=%08x fpu=%08x misc=%08x\n",
               static_cast<unsigned long long>(gTriSeq), gDumpFmt, gDumpPrim,
               t0.wTex, t0.hTex, t0.pitchTex, t0.fmt, t0.cntl, t0.offs[0],
               P.texCntl, P.fpu, P.misc);
        for (const Vtx* v : {&v0, &v1, &v2})
            printf("-- tv %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g\n",
                   v->x, v->y, v->z, v->rhw, v->s1, v->t1, v->s2, v->t2);
    }
    const bool ccwFront = (P.fpu & 1u) != 0;
    bool front = (area2 > 0.0) != ccwFront; // area>0 = screen-CW
    if (flipFacing)
        front = !front; // strips alternate winding; facing does not
    // An FPU_SETUP of exactly zero would cull every face; no driver means
    // that, and an unprogrammed register reads zero here. Draw solid.
    const u32 fn =
        P.fpu == 0u ? 3u : front ? (P.fpu >> 3) & 3u : (P.fpu >> 1) & 3u;
    if (fn == 0u) {
        ++eng().triCulled;
        return;
    }
    if (fn == 1u) { // degrade to its three vertices (Table 6-22)
        rasterPoint(P, v0);
        rasterPoint(P, v1);
        rasterPoint(P, v2);
        return;
    }
    if (fn == 2u) { // degrade to its three edges
        rasterLine(P, v0, v1);
        rasterLine(P, v1, v2);
        rasterLine(P, v2, v0);
        return;
    }
    if (area2 < 0.0) {
        Vtx t = v1;
        v1 = v2;
        v2 = t;
        area2 = -area2;
    }
    const double x0 = v0.x + P.winX, y0 = v0.y + P.winY;
    const double x1 = v1.x + P.winX, y1 = v1.y + P.winY;
    const double x2 = v2.x + P.winX, y2 = v2.y + P.winY;
    int minX = static_cast<int>(std::floor(std::fmin(x0, std::fmin(x1, x2))));
    int maxX = static_cast<int>(std::ceil(std::fmax(x0, std::fmax(x1, x2))));
    int minY = static_cast<int>(std::floor(std::fmin(y0, std::fmin(y1, y2))));
    int maxY = static_cast<int>(std::ceil(std::fmax(y0, std::fmax(y1, y2))));
    if (minX < P.scL) minX = P.scL;
    if (maxX > P.scR) maxX = P.scR;
    if (minY < P.scT) minY = P.scT;
    if (maxY > P.scB) maxY = P.scB;
    if (minX < 0) minX = 0;
    if (minY < 0) minY = 0;
    if (minX > maxX || minY > maxY) {
        ++eng().triDegen;
        return;
    }
    TriSetup T;
    auto mkEdge = [](double ax, double ay, double bx, double by) {
        // e = (bx-ax)(y-ay) - (by-ay)(x-ax), expanded to a*x + b*y + c.
        TriEdge ed;
        ed.a = ay - by;
        ed.b = bx - ax;
        ed.c = -(ed.a * ax + ed.b * ay);
        // D3D top-left rule in y-down CW terms: a top edge runs rightward
        // horizontally, a left edge runs upward.
        const double dx = bx - ax, dy = by - ay;
        ed.topLeft = (dy == 0.0 && dx > 0.0) || dy < 0.0;
        return ed;
    };
    T.e[0] = mkEdge(x0, y0, x1, y1);
    T.e[1] = mkEdge(x1, y1, x2, y2);
    T.e[2] = mkEdge(x2, y2, x0, y0);
    const double inv2a = 1.0 / area2;
    // Attribute planes (screen-affine). The stream's S,T already ARE the
    // s·rhw / t·rhw numerators (receipt below); rhw gets its own plane and
    // the per-pixel divide resolves u,v unless the unit disables
    // perspective.
    T.pz = planeOf(x0, y0, x1, y1, x2, y2, v0.z, v1.z, v2.z, inv2a);
    T.pa = planeOf(x0, y0, x1, y1, x2, y2, v0.ca, v1.ca, v2.ca, inv2a);
    T.pr = planeOf(x0, y0, x1, y1, x2, y2, v0.cr, v1.cr, v2.cr, inv2a);
    T.pg = planeOf(x0, y0, x1, y1, x2, y2, v0.cg, v1.cg, v2.cg, inv2a);
    T.pb = planeOf(x0, y0, x1, y1, x2, y2, v0.cb, v1.cb, v2.cb, inv2a);
    T.psr = planeOf(x0, y0, x1, y1, x2, y2, v0.sr, v1.sr, v2.sr, inv2a);
    T.psg = planeOf(x0, y0, x1, y1, x2, y2, v0.sg, v1.sg, v2.sg, inv2a);
    T.psb = planeOf(x0, y0, x1, y1, x2, y2, v0.sb, v1.sb, v2.sb, inv2a);
    T.pfog = planeOf(x0, y0, x1, y1, x2, y2, v0.fog, v1.fog, v2.fog, inv2a);
    T.pw = planeOf(x0, y0, x1, y1, x2, y2, v0.rhw, v1.rhw, v2.rhw, inv2a);
    T.persp0 = P.tex[0].on && !((P.tex[0].cntl >> 14) & 1u);
    T.persp1 = P.tex[1].on && !((P.tex[1].cntl >> 14) & 1u);
    // ⭐⭐ THE VERTEX'S S,T ARE ALREADY PREMULTIPLIED BY RHW — they are the
    // screen-linear NUMERATORS (u·rhw, v·rhw), not plain u,v, so the setup
    // stage must NOT multiply by rhw again. Measured from a shipping RAVE
    // title's live stream (--tri-dump, 900 triangles off a snapshot): raw
    // s against the content's 0.2 UV lattice is random (mean distance
    // 0.246 vs 0.25 chance) while s/rhw lands EXACTLY on it (0, 0.4, 0.6,
    // 1.0; 41% of interior vertices within 1e-3, the rest being model UVs
    // and screen-edge vertices the guest's own clipper interpolated) — and
    // the stream's global max of s equals the global max of rhw to three
    // decimals (126.7), which is what s = u·rhw with u ≤ ~1 looks like.
    // This is RAVE's own vertex contract (uOverW/vOverW) passed through by
    // the Mac driver. The SDK's F-45 calls the field "the u-coordinate"
    // and documents no premultiply anywhere — the stream outranks the
    // prose. A double multiply divided every coordinate by w once more,
    // which under the title's clamp-to-edge modes smeared edge texels over
    // the whole world: texture BYTES provably right, picture wrong
    // everywhere.
    //
    // Iteration is therefore the same for both states of the perspective
    // bit — the plane interpolates the stream value as given — and
    // TEX_PERSPECTIVE_DISABLE only controls the per-pixel divide below.
    T.ps1 = planeOf(x0, y0, x1, y1, x2, y2,
                     v0.s1, v1.s1, v2.s1, inv2a);
    T.pt1 = planeOf(x0, y0, x1, y1, x2, y2,
                     v0.t1, v1.t1, v2.t1, inv2a);
    T.ps2 = planeOf(x0, y0, x1, y1, x2, y2,
                     v0.s2, v1.s2, v2.s2, inv2a);
    T.pt2 = planeOf(x0, y0, x1, y1, x2, y2,
                     v0.t2, v1.t2, v2.t2, inv2a);
    ++eng().tris;
    // What this triangle's fragments actually consume, decided once. spec
    // and fog planes feed nothing when their stage is off; the second set
    // feeds nothing without a second unit; and the LOD derivative block —
    // which carries a log2 per pixel — steers the sampler ONLY when mips
    // are live or the min/mag filters disagree on bilinearity (sampleTex's
    // no-mip path reads nothing else out of it). A shipping title runs
    // mips-off with both filters bilinear, so the whole block is dead
    // weight there.
    T.wantSpec = P.specEn;
    T.wantFog = P.fogEn && !P.fogTable;
    T.lodLive =
        P.tex[0].on &&
        ((!((P.tex[0].cntl >> 7) & 1u) && P.tex[0].maxLevel > 0) ||
         (((P.tex[0].cntl >> 1) & 1u) != ((P.tex[0].cntl >> 4) & 1u)));
    // What sampleTex needs of the level (drawRows): with no bias, only
    // whether it is above zero wherever no unit has smaller maps to choose
    // from.
    const auto mips = [](const TexUnit& t) {
        return t.on && !((t.cntl >> 7) & 1u) && t.maxLevel > 0;
    };
    T.lodNoBias = ((P.texCntl >> 24) & 0xFFu) == 0u;
    T.lodSignOnly = T.lodNoBias && !mips(P.tex[0]) && !mips(P.tex[1]);
    const double colA[4] = {T.pa.a, T.pr.a, T.pg.a, T.pb.a};
    const double specA[4] = {0.0, T.psr.a, T.psg.a, T.psb.a};
    for (int k = 0; k < 4; ++k) {
        T.colA[k] = colA[k];
        T.specA[k] = specA[k];
    }
    T.minX = minX;
    T.maxX = maxX;
    // Several threads for a triangle large enough to repay waking them, or
    // one much smaller while some are awake anyway (they stay so for a
    // moment after a job, and a scene's triangles come in runs); and only
    // where its rows share nothing.
    if (RowQueue::threads() > 1) {
        RowQueue& q = RowQueue::get();
        bool start = q.pending();
        if (!start) {
            const double box = (static_cast<double>(maxX) - minX + 1.0) *
                               (static_cast<double>(maxY) - minY + 1.0);
            // Once something waits, whatever can wait behind it does, at
            // any size: the order within a row is the order of the queue.
            start = q.worth(area2 * 0.5 < box ? area2 * 0.5 : box);
        }
        if (start && q.admit(P, maxX, minY, maxY)) {
            q.add(P, T, minY, maxY);
            ++eng().crewTris;
            return;
        }
        if (q.pending()) {
            ++eng().rowRefused;
            q.settle(); // this one is drawn here, behind everything waiting
        }
    }
    drawRows(P, T, minY, maxY);
}

// ── Lines and points ──────────────────────────────────────────────────────
void rasterPoint(const Pipe& P, const Vtx& v)
{
    rowsSettle();
    const int px = static_cast<int>(std::floor(v.x + P.winX));
    const int py = static_cast<int>(std::floor(v.y + P.winY));
    if (px < P.scL || px > P.scR || py < P.scT || py > P.scB || px < 0 ||
        py < 0)
        return;
    Frag fr;
    fr.z = v.z;
    fr.diffuse = F4::set(v.ca, v.cr, v.cg, v.cb);
    fr.spec = F4::set(255, v.sr, v.sg, v.sb);
    fr.fog = v.fog;
    // Stream S,T are premultiplied numerators (rasterTriangle's receipt);
    // resolving a single vertex is one divide by its own rhw.
    const float iw = v.rhw != 0.0f ? 1.0f / v.rhw : 1.0f;
    const bool persp0 = P.tex[0].on && !((P.tex[0].cntl >> 14) & 1u);
    const bool persp1 = P.tex[1].on && !((P.tex[1].cntl >> 14) & 1u);
    fr.s1 = persp0 ? v.s1 * iw : v.s1;
    fr.t1 = persp0 ? v.t1 * iw : v.t1;
    fr.s2 = persp1 ? v.s2 * iw : v.s2;
    fr.t2 = persp1 ? v.t2 * iw : v.t2;
    ++eng().points3d;
    shadeFragment(P, px, py, fr);
}

void rasterLine(const Pipe& P, const Vtx& a, const Vtx& b)
{
    rowsSettle();
    // A DDA over the major axis. The hardware's exact traversal rule is not
    // documented in anything on the shelf; this draws the segment with
    // correct endpoints and attribute interpolation, which is what a game's
    // wireframe or UI line needs. Documented divergence, not a secret one.
    const float dx = b.x - a.x, dy = b.y - a.y;
    const float steps = std::fmax(std::fabs(dx), std::fabs(dy));
    const int n = static_cast<int>(steps);
    ++eng().lines3d;
    if (n == 0) {
        rasterPoint(P, a);
        return;
    }
    const bool persp0 = P.tex[0].on && !((P.tex[0].cntl >> 14) & 1u);
    const bool persp1 = P.tex[1].on && !((P.tex[1].cntl >> 14) & 1u);
    for (int i = 0; i <= n; ++i) {
        const float f = static_cast<float>(i) / static_cast<float>(n);
        Vtx v;
        v.x = a.x + dx * f;
        v.y = a.y + dy * f;
        v.z = a.z + (b.z - a.z) * f;
        v.ca = a.ca + (b.ca - a.ca) * f;
        v.cr = a.cr + (b.cr - a.cr) * f;
        v.cg = a.cg + (b.cg - a.cg) * f;
        v.cb = a.cb + (b.cb - a.cb) * f;
        v.sr = a.sr + (b.sr - a.sr) * f;
        v.sg = a.sg + (b.sg - a.sg) * f;
        v.sb = a.sb + (b.sb - a.sb) * f;
        v.fog = a.fog + (b.fog - a.fog) * f;
        // Numerators and rhw interpolate screen-linearly along the line;
        // the divide per step is what makes the texture perspective-true
        // (stream S,T are premultiplied — rasterTriangle's receipt).
        v.s1 = a.s1 + (b.s1 - a.s1) * f;
        v.t1 = a.t1 + (b.t1 - a.t1) * f;
        v.s2 = a.s2 + (b.s2 - a.s2) * f;
        v.t2 = a.t2 + (b.t2 - a.t2) * f;
        v.rhw = a.rhw + (b.rhw - a.rhw) * f;
        const int px = static_cast<int>(std::floor(v.x + P.winX));
        const int py = static_cast<int>(std::floor(v.y + P.winY));
        if (px < P.scL || px > P.scR || py < P.scT || py > P.scB ||
            px < 0 || py < 0)
            continue;
        Frag fr;
        fr.z = v.z;
        fr.diffuse = F4::set(v.ca, v.cr, v.cg, v.cb);
        fr.spec = F4::set(255, v.sr, v.sg, v.sb);
        fr.fog = v.fog;
        const float iw = v.rhw != 0.0f ? 1.0f / v.rhw : 1.0f;
        fr.s1 = persp0 ? v.s1 * iw : v.s1;
        fr.t1 = persp0 ? v.t1 * iw : v.t1;
        fr.s2 = persp1 ? v.s2 * iw : v.s2;
        fr.t2 = persp1 ? v.t2 * iw : v.t2;
        shadeFragment(P, px, py, fr);
    }
}

// ── Shading-mode application + primitive assembly ─────────────────────────
//
// PM4_VC_FPU_SETUP bits 6:5: 0 solid (CONSTANT_COLOR_C), 1 flat, 2/3
// Gouraud; bit 14 picks the D3D (first) or OpenGL (last) provoking vertex
// for flat shading (SDK Table 6-21 and its prose).
void applyShading(const Pipe& P, Vtx& v0, Vtx& v1, Vtx& v2)
{
    const u32 fcn = (P.fpu >> 5) & 3u;
    if (fcn == 0u && P.fpu != 0u) {
        for (Vtx* v : {&v0, &v1, &v2}) {
            v->ca = P.constA;
            v->cr = P.constR;
            v->cg = P.constG;
            v->cb = P.constB;
        }
    } else if (fcn == 1u) {
        const Vtx& src = (P.fpu >> 14) & 1u ? v2 : v0;
        for (Vtx* v : {&v0, &v1, &v2}) {
            v->ca = src.ca;
            v->cr = src.cr;
            v->cg = src.cg;
            v->cb = src.cb;
            v->sr = src.sr;
            v->sg = src.sg;
            v->sb = src.sb;
        }
    }
}

// A vertex source the assembler pulls from: either the inline packet body
// (RING walk) or the GART-resident vertex buffer (LIST/IND walks).
struct VtxSource {
    const Pipe* P = nullptr;
    // inline
    const u32* words = nullptr;
    u32 wordCount = 0;
    // buffer
    u32 vloff = 0, vsize = 0;
    bool buffered = false;
    u32 fmt = 0, stride = 0;

    bool fetch(u32 index, Vtx& out) const
    {
        if (!buffered) {
            const u64 at = static_cast<u64>(index) * stride;
            if (at + stride > wordCount)
                return false;
            decodeVtx(words + at, fmt, out);
            ++eng().vtxFetched;
            return true;
        }
        if (vsize && index >= vsize)
            return false; // an index past PM4_VC_VSIZE is a driver bug —
                          // refuse rather than wander the GART
        u32 tmp[20];
        if (stride > 20u)
            return false;
        for (u32 i = 0; i < stride; ++i)
            if (!r128CceGartRead(P->m,
                                 vloff + index * stride * 4u + i * 4u,
                                 tmp[i])) {
                ++eng().vtxGartMiss;
                return false;
            }
        decodeVtx(tmp, fmt, out);
        ++eng().vtxFetched;
        return true;
    }
};

// A vertex's position as the setup engine receives it: in fixed point with
// two or four fractional bits, so a corner sits on a quarter or a sixteenth
// of a pixel and nowhere between. r128_reg.h names the two choices in
// PM4_VC_FPU_SETUP (and SETUP_CNTL repeats the first for a setup engine
// driven through its registers), and the register guide gives the formats
// S.12.2 and S.11.4 for the window offset that is added to them. Corners
// shared by two triangles land on the same step, so the pair still meets
// without a gap; what changes against exact positions is which pixel centers
// an edge takes when it passes within a step of them.
void snapVertex(const Pipe& P, Vtx& v)
{
    const auto snap = [&P](float at) {
        const float steps = at * P.subSteps;
        return (P.subNearest ? std::floor(steps + 0.5f) : std::trunc(steps)) /
               P.subSteps;
    };
    v.x = snap(v.x);
    v.y = snap(v.y);
    // Texture coordinates. The rasterizer works on S and T ALREADY
    // multiplied by W (the numerators of the perspective divide), which is
    // how the Mac driver sends them, with SETUP_CNTL's bit 9 set
    // (TEXTURE_ST_DIRECT: use them as they are). With the bit clear
    // (TEXTURE_ST_MULT_W, the setting the Mesa driver's context carries)
    // the stream has them plain and the multiply is the chip's.
    if (P.stTimesW) {
        v.s1 *= v.rhw;
        v.t1 *= v.rhw;
        v.s2 *= v.rhw;
        v.t2 *= v.rhw;
    }
}

// Assemble and draw every primitive the packet describes. `idx` maps
// primitive-order position to vertex index (identity for LIST/RING walks,
// the packed index list for the IND walk).
void assemble(const Pipe& P, const VtxSource& src, u32 primType, u32 count,
              const u32* indexWords, bool indexed)
{
    auto vertexAt = [&](u32 i, Vtx& v) {
        u32 vi = i;
        if (indexed) {
            const u32 w = indexWords[i >> 1];
            vi = (i & 1u) ? (w >> 16) : (w & 0xFFFFu);
        }
        if (!src.fetch(vi, v))
            return false;
        snapVertex(P, v);
        return true;
    };
    switch (primType) {
    case 1u: // points
        for (u32 i = 0; i < count; ++i) {
            Vtx v;
            if (!vertexAt(i, v))
                return;
            rasterPoint(P, v);
        }
        break;
    case 2u: // independent lines
        for (u32 i = 0; i + 2u <= count; i += 2u) {
            Vtx a, b;
            if (!vertexAt(i, a) || !vertexAt(i + 1u, b))
                return;
            rasterLine(P, a, b);
        }
        break;
    case 3u: // polyline
        for (u32 i = 0; i + 2u <= count; ++i) {
            Vtx a, b;
            if (!vertexAt(i, a) || !vertexAt(i + 1u, b))
                return;
            rasterLine(P, a, b);
        }
        break;
    case 4u: // triangle list
        for (u32 i = 0; i + 3u <= count; i += 3u) {
            Vtx a, b, c2;
            if (!vertexAt(i, a) || !vertexAt(i + 1u, b) ||
                !vertexAt(i + 2u, c2))
                return;
            applyShading(P, a, b, c2);
            rasterTriangle(P, a, b, c2, false);
        }
        break;
    case 5u: // fan: vertex 1 shared by every triangle (F.25.5)
        for (u32 i = 0; count >= 3u && i <= count - 3u; ++i) {
            Vtx a, b, c2;
            if (!vertexAt(0, a) || !vertexAt(i + 1u, b) ||
                !vertexAt(i + 2u, c2))
                return;
            applyShading(P, a, b, c2);
            rasterTriangle(P, a, b, c2, false);
        }
        break;
    case 6u: // strip: winding alternates, facing does not
    case 7u: // "type-2 triangles" (walker): body shape matches a strip; no
             // fuller definition exists on the shelf, so a strip is the
             // reading — logged when first seen rather than silently alike
        if (primType == 7u && gLogBudget > 0) {
            --gLogBudget;
            printf("-- cce 3d: prim type 7 (type-2 tri) drawn as strip\n");
        }
        for (u32 i = 0; count >= 3u && i <= count - 3u; ++i) {
            Vtx a, b, c2;
            if (!vertexAt(i, a) || !vertexAt(i + 1u, b) ||
                !vertexAt(i + 2u, c2))
                return;
            applyShading(P, a, b, c2);
            rasterTriangle(P, a, b, c2, (i & 1u) != 0);
        }
        break;
    default:
        ++eng().prim3dDecline;
        if (gLogBudget > 0) {
            --gLogBudget;
            printf("-- cce 3d: prim type %u declined\n", primType);
        }
        break;
    }
}

bool pipeReady(R128Cell& c, const CceMem& m, Pipe& P)
{
    // The 3D function gate: SCALE_3D_FN = TEXMAP_SHADE in either register
    // enables the pipe (§6.6.1 names MISC; SCALE_3D_CNTL carries the same
    // field at 7:6). A guest that steered the function elsewhere gets a
    // counted decline, not a half-drawn guess. A machine where no guest
    // ever wrote either register draws — unit rigs and minimal streams
    // should not need a ceremony the packet format does not require.
    if (gGate3d == 0) {
        ++eng().prim3dDecline;
        if (gLogBudget > 0) {
            --gLogBudget;
            printf("-- cce 3d: primitive declined, SCALE_3D_FN not "
                   "TEXMAP_SHADE (misc=%08x scale=%08x)\n",
                   c.peek(kMisc3dStateCntl), c.peek(kScale3dCntl));
        }
        return false;
    }
    return gatherPipe(c, m, P);
}

// On the engine's own thread the register lock is held when a packet starts.
// A draw reads its state out of the Pipe gathered at its head and touches
// video memory and counters, never a register, so the lock is let go for its
// length: the machine's thread can then reach the registers that have nothing
// to do with the engine (the vertical-blank pair, the cursor) while a large
// triangle is being filled.
struct RegLend {
    bool on;
    explicit RegLend(bool w) : on(w)
    {
        if (on)
            r128AsyncRegRelease();
    }
    ~RegLend()
    {
        if (on)
            r128AsyncRegTake();
    }
    RegLend(const RegLend&) = delete;
    RegLend& operator=(const RegLend&) = delete;
};

} // namespace

// ── Entry points from the parser and the register engine ──────────────────

void r128Cce3dReset()
{
    std::memset(gFog.t, 0xFF, sizeof gFog.t); // f=255: no fog
    std::memset(gPalette, 0, sizeof gPalette);
    gDither = DitherError{};
    gWalk = WalkLatch{};
    gGate3d = -1;
    gLogBudget = 24;
    gDumpFmt = 0;
    gDumpPrim = 0;
    gTriSeq = 0;
    // gTriDump deliberately survives reset: the flag is armed before the
    // machine boots and the machine resets on the way up.
}

// The 3D unit's share of the engine state a frame recording carries.
void r128Cce3dStateSave(SnapWriter& w)
{
    w.raw(gFog.t, sizeof gFog.t);
    w.arr32(gPalette[0], 256);
    w.arr32(gPalette[1], 256);
    w.b(gWalk.valid);
    w.u32v(gWalk.vloff);
    w.u32v(gWalk.vsize);
    w.u32v(gWalk.fmt);
    w.u32v(gWalk.cntl);
    w.i32v(gGate3d);
    w.u32v(gDither.r);
    w.u32v(gDither.g);
    w.u32v(gDither.b);
}
void r128Cce3dStateLoad(SnapReader& r)
{
    r.raw(gFog.t, sizeof gFog.t);
    r.arr32(gPalette[0], 256);
    r.arr32(gPalette[1], 256);
    gWalk.valid = r.b();
    gWalk.vloff = r.u32v();
    gWalk.vsize = r.u32v();
    gWalk.fmt = r.u32v();
    gWalk.cntl = r.u32v();
    gGate3d = r.i32v();
    // A recording made before the error register existed ends here.
    gDither = DitherError{};
    if (r.ok && r.p < r.end) {
        gDither.r = r.u32v();
        gDither.g = r.u32v();
        gDither.b = r.u32v();
    }
}

int r128Gate3dState() { return gGate3d; }

void r128Set3dTriDump(int n) { gTriDump = n; }

void r128SetRasterThreads(int n)
{
    // Taken up where the queue next starts from empty.
    gRowThreads.store(n < 0 ? 0 : n, std::memory_order_relaxed);
}
int r128RasterThreads() { return RowQueue::threads(); }

bool gR128RowsPending = false;
void r128RowsFlush() { RowQueue::get().settle(); }

// Which bits of each 3D state register this file reads — the second statement
// of gatherPipe and the samplers that r128RegDecodedBits is built from. A bit
// a guest sets outside its register's mask is a feature it asked for and did
// not get. ⚠ Change a decoder, change its line here.
bool r128Reg3dDecodedBits(u32 off, u32& mask)
{
    // The texture offset slots. A driver writes all eleven in one burst and
    // means only the slot of the texture's size (see gatherPipe); the others
    // carry whatever was in its buffer, so "bits ever set" says nothing
    // here. The tiling mode in the top two bits is counted where a level is
    // fetched.
    if ((off >= kPrimTex0OffsetC && off <= kPrimTex0OffsetC + 40u) ||
        (off >= kSecTex0OffsetC && off <= kSecTex0OffsetC + 40u)) {
        mask = 0xFFFFFFFFu;
        return true;
    }
    switch (off) {
    // PM4_VC_FPU_SETUP: front direction, back and front face functions, the
    // shade function, the sub-pixel precision, the flat-shade vertex, the
    // rounding of a position. Bit 8 selects 3D mode, which is the only mode
    // this file is reached in.
    case kFpuSetup: mask = 0x0000C1FFu; return true;
    // Z_STEN_CNTL_C: depth width and test, the stencil test and its three
    // operations.
    case kZStenCntlC: mask = 0x07777076u; return true;
    // Z_PITCH_C: the pitch, and the tile bit (read as linear, counted).
    case kZPitchC: mask = 0x000103FFu; return true;
    // TEX_CNTL_C: Z enable and write, stencil, the two texture enables, fog,
    // dither, alpha blend and test, specular, chroma key, the texel alpha
    // mask, the two lighting functions, the LOD bias. Bit 23 flushes the
    // texel cache, which this model does not have. Bit 21 (r128_reg.h
    // ANTI_ALIAS; the books are silent) is in the value census, which says
    // whether a DRAW was made with it: one game sets it between two screens
    // and clears it before it draws anything.
    case kTexCntlC: mask = 0xFFBFFFBBu; return true;
    // MISC_3D_STATE_CNTL: reference alpha, the 3D function, the alpha
    // combine, table fog, the two blend factors, the alpha test.
    case kMisc3dStateCntl: mask = 0x07FF73FFu; return true;
    // PRIM/SEC_TEX_CNTL_C: the two filters, mip disable, the clamp modes,
    // perspective disable, the datatype, the palette select; the secondary
    // unit's coordinate select. Bits 25:24 are the palette entries' own
    // datatype: a pseudo-color texel is read through the palette at the
    // width it was loaded with whatever they say (a simplification the
    // header lists), and no other format looks at a palette.
    case kPrimTexCntlC: mask = 0x033F5BFEu; return true;
    case kSecTexCntlC: mask = 0x033F5BFFu; return true;
    // The combine registers: color function and its two factors, alpha
    // function and its two factors.
    case kPrimTexCombineCntlC:
    case kSecTexCombineCntlC: mask = 0x0E3FFCFFu; return true;
    // SCALE_3D_CNTL (r128_reg.h; the books give two of its fields): the 3D
    // function, which gates the block, and the texel cache's four settings,
    // which have nothing to act on here. The dither method (1) and the error
    // reset (3) are in the value census, as is the color key function the
    // Mac driver writes in the top two bits. Not read: 4 (24-bit rounding),
    // 8 (scaler pixel replication), 10 (a YUV mode), and 12 to 29, which
    // repeat MISC_3D_STATE_CNTL's fields: this model takes those from that
    // register alone.
    case kScale3dCntl: mask = 0xC0000AEEu; return true;
    // SETUP_CNTL (r128_reg.h): everything the header names. The setup engine
    // is driven here by the vertex walker, which takes the shading and the
    // primitive from PM4_VC_FPU_SETUP and the packet; the two fields that
    // change a picture (9, how texture coordinates are given, and 19, the
    // sub-pixel precision) are in the value census.
    case kSetupCntl: mask = 0x800FC3FDu; return true;
    case kTexSizePitchC:
    case kFogColorC:
    case kConstantColorC:
    case kPrimTexBorderColorC:
    case kSecTexBorderColorC:
    case kTexClrCmpClrC:
    case kTexClrCmpMskC:
    case kStenRefMaskC:
    case kPlane3dMaskC:
    case kZOffsetC: mask = 0xFFFFFFFFu; return true;
    default: return false;
    }
}

bool r128Eng3dWrite(R128Cell& c, u32 off, u32 v)
{
    if (off == kFogTableIndex) {
        c.setReg(kFogTableIndex, v & 0xFFu);
        return true;
    }
    if (off == kFogTableData) {
        // Post-incrementing table write, the SDK ch 6 example's contract.
        const u32 idx = c.peek(kFogTableIndex) & 0xFFu;
        gFog.t[idx] = static_cast<u8>(v);
        c.setReg(kFogTableIndex, (idx + 1u) & 0xFFu);
        return true;
    }
    if (off == kMisc3dStateCntl || off == kScale3dCntl) {
        // Both carry SCALE_3D_FN (9:8 in MISC, 7:6 in SCALE_3D_CNTL); the
        // last write steers the gate. Stored normally by the caller.
        //
        // ⭐ THE GATE CLOSES ON ZERO, NOT ON "NOT 2". The RRG's
        // MISC_3D_STATE_CNTL table is explicit: "if this field is set to 0,
        // many 3D/Front-End Scalar/Setup Engine registers are NOT writeable.
        // Hence this field should be written to a NON-ZERO value prior to
        // trying to write any other 3D/Front-End Scalar registers." So
        // 1 = Scaling leaves the block writeable exactly as 2 does.
        //
        // This model closed the gate whenever the function was not 2, which
        // silently DROPPED every context write a driver made while the pipe
        // sat in Scaling — 4,392 of them in the first real game session,
        // the only refusal in the whole capture. Dropped state does not
        // announce itself: the primitives still draw, using whatever the
        // registers held before.
        // ⭐⭐ AND IT IS STEERED BY BOTH REGISTERS TOGETHER, NOT BY WHICHEVER
        // WAS WRITTEN LAST. Measured in a game: the driver runs with
        // MISC_3D_STATE_CNTL = 00510200 (SCALE_3D_FN = 2, TEXMAP_SHADE — it
        // is plainly doing 3D, 314k triangles of it) while SCALE_3D_CNTL
        // reads 80000000, whose 7:6 field is 0. Under "last write wins" that
        // zero slammed the gate shut, and the block it guards holds
        // PRIM_TEX_0_OFFSET and its secondary — so the TEXTURE BASE
        // ADDRESSES never landed and stayed at 0. Every texel then came from
        // VRAM offset 0, which is 87% zero bytes: black surfaces with bright
        // flecks where something else happens to live, over correctly
        // shaped, correctly wound, fully textured geometry. 21,980 writes
        // went in the bin in one session.
        //
        // The lock releases once the pipe has been given a function at all,
        // which is the same "either register" convention the primitive gate
        // below already uses.
        const u32 miscV = off == kMisc3dStateCntl ? v : c.peek(kMisc3dStateCntl);
        const u32 scaleV = off == kScale3dCntl ? v : c.peek(kScale3dCntl);
        const u32 fnMisc = (miscV >> 8) & 3u, fnScale = (scaleV >> 6) & 3u;
        gGate3d = (fnMisc != 0u || fnScale != 0u) ? 1 : 0;
        return false; // fall through to the plain store
    }
    // The write-gated 3D context block. MISC itself stays writable — it IS
    // the gate — and the four cells at 0x1C80-0x1C8C are the 2D-shared
    // context registers handled by the alias switch.
    if (off >= kZOffsetC && off <= kPlane3dMaskC && off != kMisc3dStateCntl) {
        if (gGate3d == 0) {
            ++eng().gated3d;
            return true; // dropped, the documented behavior
        }
        c.setReg(off, v);
        return true;
    }
    return false;
}

bool r128Cce3dOp(R128Cell& c, u32 op, const u32* body, u32 n, const CceMem& m)
{
    if (op == 0x2Cu) { // LOAD_PALETTE (F.21)
        if (n < 1u)
            return false;
        rowsSettle(); // triangles still waiting read the palette (the packet
                      // loop has settled already; this is the rule, here)
        const u32 kind = body[0];
        const u32 count = kind == 1u ? 16u : kind == 2u ? 256u : 0u;
        if (!count || n < 1u + count)
            return false;
        // Which palette RAM this load targets is not in the packet; the
        // texel path selects via PRIM_TEX_CNTL's palette field. Both are
        // loaded — the observable difference needs hardware nothing on the
        // shelf documents.
        for (u32 i = 0; i < count; ++i) {
            gPalette[0][i] = body[1u + i];
            gPalette[1][i] = body[1u + i];
        }
        return true;
    }
    if (op == 0x25u) { // 3D_RNDR_GEN_PRIM: inline FTLVERTEX stream
        if (n < 2u)
            return false;
        Pipe P;
        if (!pipeReady(c, m, P))
            return false;
        const u32 fmt = body[0], cntl = body[1];
        const u32 walk = (cntl >> 4) & 3u;
        if (walk != 3u) {
            // The SDK pairs GEN_PRIM with the ring walk; a walker selection
            // here has no vertex buffer to read.
            ++eng().prim3dDecline;
            return false;
        }
        gDumpFmt = fmt;
        gDumpPrim = cntl & 0xFu;
        const u32 stride = vtxDwords(fmt);
        u32 count = cntl >> 16;
        const u32 avail = (n - 2u) / stride;
        if (count > avail)
            count = avail; // header count bounds the body; never wander
        VtxSource src;
        src.P = &P;
        src.words = body + 2u;
        src.wordCount = n - 2u;
        src.fmt = fmt;
        src.stride = stride;
        RegLend lend(m.worker);
        assemble(P, src, cntl & 0xFu, count, nullptr, false);
        flushTally(P);
        return true;
    }
    if (op == 0x23u || op == 0x2Eu) { // INDX_PRIM / NEXT_VERTEX_BUNDLE
        u32 vloff, vsize, fmt, cntl;
        const u32* idxWords;
        u32 idxAvail;
        if (op == 0x23u) {
            if (n < 4u)
                return false;
            vloff = body[0];
            vsize = body[1];
            fmt = body[2];
            cntl = body[3];
            idxWords = body + 4u;
            idxAvail = n - 4u;
            gWalk = {true, vloff, vsize, fmt, cntl};
        } else {
            // F.27: same manner as the previous 0x23. Without one, the
            // bundle has no vertex buffer at all.
            if (!gWalk.valid) {
                ++eng().prim3dDecline;
                return false;
            }
            vloff = gWalk.vloff;
            vsize = gWalk.vsize;
            fmt = gWalk.fmt;
            cntl = gWalk.cntl;
            idxWords = body;
            idxAvail = n;
        }
        Pipe P;
        if (!pipeReady(c, m, P))
            return false;
        gDumpFmt = fmt;
        gDumpPrim = cntl & 0xFu;
        const u32 walk = (cntl >> 4) & 3u;
        const u32 stride = vtxDwords(fmt);
        if (stride > 20u || walk == 0u || walk == 3u ||
            (op == 0x2Eu && walk != 1u)) {
            // A bundle IS more indices (F-54 numbers its fields as a
            // continuation of the F.26 packet), so it only exists for the
            // indexed walk.
            ++eng().prim3dDecline;
            return false;
        }
        VtxSource src;
        src.P = &P;
        src.buffered = true;
        src.vloff = vloff;
        src.vsize = vsize;
        src.fmt = fmt;
        src.stride = stride;
        u32 count;
        if (op == 0x2Eu) {
            // The bundle's count is its own payload: every index it carries.
            // ⚠ An odd number of real indices is padded with a zero high
            // word (F.27); a driver submits whole primitives per bundle, so
            // the pad lands in an incomplete tail the assembler drops. A
            // bundle that strands partial primitives was garbage on real
            // hardware too.
            count = idxAvail * 2u;
        } else {
            count = cntl >> 16;
            if (walk == 1u) { // indexed: two WORD indices per DWORD (F.26)
                const u32 fit = idxAvail * 2u;
                if (count > fit)
                    count = fit; // header-bounded, the pad stays out
            } else if (vsize && count > vsize) {
                count = vsize;
            }
        }
        RegLend lend(m.worker);
        assemble(P, src, cntl & 0xFu, count, idxWords, walk == 1u);
        flushTally(P);
        return true;
    }
    return false;
}

} // namespace opm
