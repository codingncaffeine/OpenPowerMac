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
// NO transform stage here) → triangle setup with the D3D top-left fill rule →
// per-pixel interpolation (colors screen-linear, textures perspective-correct
// through rhw) → two texture-combine units → texture lighting → specular add
// → fog (vertex or table) → chroma key → alpha test → stencil → Z → alpha
// blend → plane mask → destination format.
//
// ⚠ CONVENTIONS THIS FILE MUST AGREE WITH, all inherited from the 2D engine:
// a pixel in VRAM is stored MOST SIGNIFICANT BYTE FIRST (the big-endian
// aperture the Mac paints through; getting this wrong once made a grey
// desktop olive), the Z buffer uses the same byte order because the guest
// CLEARS Z with 2D PAINT ops through the same store path, and every counter
// lives in a file-static so sizeof(R128Cell) — and with it every snapshot —
// never moves.
//
// Known simplifications, each visible rather than silent: dithering rounds
// instead of patterning (DITHER_ENABLE is honoured as "convert with
// rounding"; the two documented dither algorithms are not reproduced),
// error-diffusion state is not modelled, tiled textures and Z (bit flags
// observed) are counted and read as linear, and VQ/CI16 texel formats
// decline. Lines use a DDA rather than the hardware's undocumented exact
// rule; adjacency of LINES is not pixel-exact against real silicon.

#include "opm/bus.hpp"
#include "opm/r128.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>

namespace opm {

// The stats object lives in r128.cpp; both halves of the engine count into
// the same struct so the report is one table.
R128EngStats& r128EngStatsMut();

namespace {

// ── Registers this unit reads (offsets from r128_reg.h; layouts per the
// SDK's field tables where the books redact them) ─────────────────────────
constexpr u32 kFpuSetup = 0x071C;        // PM4_VC_FPU_SETUP
constexpr u32 kScale3dCntl = 0x1A00;     // SCALE_3D_CNTL
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
constexpr u32 kDpDatatype = 0x16C4;      // canonical, engWrite-normalised
constexpr u32 kDstOffset = 0x1404, kDstPitch = 0x1408;
constexpr u32 kScLeft = 0x1640, kScRight = 0x1644;
constexpr u32 kScTop = 0x1648, kScBottom = 0x164C;
constexpr u32 kEngAddrMask = 0x03FFFFF0u;
// The card's internal address split: offsets at or above 32 MB are the AGP
// aperture — system memory through the GART. r128_reg.h pins the constant
// (R128_AGP_TEX_OFFSET 0x02000000) and the 2D engine already refuses such
// destinations; textures are where this model finally honours them.
constexpr u32 kAgpWindow = 0x02000000u;

// ── File-static engine state (NEVER members: snapshot rule) ───────────────
//
// A snapshot taken between an INDX_PRIM and its NEXT_VERTEX_BUNDLE, or
// between fog-table words, resumes without this — the same accepted caveat
// as the parser's staging FIFO, and minted snapshots are quiescent.
// f=255 is "no fog" in the blend equation, so an unwritten table must read
// 255 from the very first machine of the process — g4run never calls the
// reset, and a zero-initialised table would fog that machine solid.
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
// documented behaviour a driver may rely on. 1 = open.
int gGate3d = -1;
// The vertex-walker continuation latch for NEXT_VERTEX_BUNDLE (F.27): the
// bundle carries only indices, "rendered in the same manner as those of the
// previous 3D_RNDR_GEN_INDX_PRIM packet".
struct WalkLatch {
    bool valid = false;
    u32 vloff = 0, vsize = 0, fmt = 0, cntl = 0;
} gWalk;
int gLogBudget = 24; // first-N prints for declines, so a busy path cannot spam

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
}

// ── Per-packet pipeline state, gathered once ──────────────────────────────
struct TexUnit {
    bool on = false;
    u32 cntl = 0, comb = 0;
    u32 fmt = 0;              // PRIMARY_DATATYPE
    u32 wTex = 0, hTex = 0, pitchTex = 0; // base-level texels
    u32 maxLevel = 0;
    u32 offs[11] = {};
    u32 border = 0;           // ARGB
    bool useSecondSt = false; // SEC_SRC_SEL_ST
};

struct Pipe {
    R128Cell* c = nullptr;
    CceMem m;
    // destination
    u32 bypp = 0, dstType = 0, dstOff = 0, dstStride = 0;
    int scL = 0, scR = 0, scT = 0, scB = 0;
    int winX = 0, winY = 0;
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
    u32 blendSrc = 1, blendDst = 0, blendComb = 0, alphaOp = 7, refAlpha = 0;
    float fogR = 0, fogG = 0, fogB = 0;
    u32 chromaClr = 0, chromaMsk = 0;
    float constA = 255, constR = 255, constG = 255, constB = 255;
    TexUnit tex[2];
    bool texAny = false;
};

int sx14c(u32 v)
{
    const int t = static_cast<int>(v & 0x3FFFu);
    return t >= 0x2000 ? t - 0x4000 : t;
}

u32 log2Field(u32 v, u32 shift) { return (v >> shift) & 0xFu; }

bool gatherPipe(R128Cell& c, const CceMem& m, Pipe& P)
{
    P.c = &c;
    P.m = m;
    const u32 dt = c.peek(kDpDatatype) & 0xFu;
    switch (dt) {
    case 3u: case 4u: P.bypp = 2; break; // aRGB1555 / RGB565
    case 6u: P.bypp = 4; break;          // aRGB8888 (X byte stored)
    default:
        ++eng().badBpp; // 8-bpp and YUV destinations are not rasterised
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
        ++eng().agpTarget; // rendering INTO system memory: not modelled
        return false;
    }
    P.scL = sx14c(c.peek(kScLeft));
    P.scR = sx14c(c.peek(kScRight));
    P.scT = sx14c(c.peek(kScTop));
    P.scB = sx14c(c.peek(kScBottom));
    const u32 win = c.peek(kWindowXyOffset);
    // 12-bit fields, sign-extended: a window origin is non-negative on a Mac
    // desktop but the field format allows the guard band.
    P.winX = static_cast<int>((win >> 20) & 0xFFFu);
    if (P.winX >= 0x800) P.winX -= 0x1000;
    P.winY = static_cast<int>((win >> 4) & 0xFFFu);
    if (P.winY >= 0x800) P.winY -= 0x1000;
    const u32 pm = c.peek(kPlane3dMaskC);
    P.planeMask = pm ? pm : 0xFFFFFFFFu; // reset-zero must not mean "write
                                         // nothing", the 2D mask's own rule
    P.fpu = c.peek(kFpuSetup);
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
    P.fogEn = (P.texCntl >> 7) & 0x1u;
    P.ditherEn = (P.texCntl >> 8) & 0x1u;
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
    // texture units
    const u32 sp = c.peek(kTexSizePitchC);
    for (int u = 0; u < 2; ++u) {
        TexUnit& t = P.tex[u];
        t.cntl = c.peek(u ? kSecTexCntlC : kPrimTexCntlC);
        t.comb = c.peek(u ? kSecTexCombineCntlC : kPrimTexCombineCntlC);
        t.on = u ? ((P.texCntl >> 5) & 1u) && ((P.texCntl >> 4) & 1u)
                 : ((P.texCntl >> 4) & 1u);
        const u32 f = u ? (sp >> 16) : sp;
        t.pitchTex = 1u << log2Field(f, 0);
        t.wTex = 1u << log2Field(f, 4);
        t.hTex = 1u << log2Field(f, 8);
        const u32 szl2 = log2Field(f, 4), minl2 = log2Field(f, 12);
        t.maxLevel = szl2 > minl2 ? szl2 - minl2 : 0;
        if (t.maxLevel > 10u)
            t.maxLevel = 10u;
        t.fmt = (t.cntl >> 16) & 0xFu;
        // ⭐⭐ THE OFFSET SLOTS ARE INDEXED BY LEVEL SIZE, NOT BY MIP NUMBER.
        //
        // Measured from Nanosaur's own traffic (the diag register census):
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
        }
        t.border = c.peek(u ? kSecTexBorderColorC : kPrimTexBorderColorC);
        t.useSecondSt = u && (t.cntl & 1u);
    }
    P.texAny = P.tex[0].on;
    return true;
}

// ── VRAM/AGP texel bytes ──────────────────────────────────────────────────
//
// One abstraction so every format decodes from CONSECUTIVE BYTES exactly as
// the guest laid them down: below the AGP window the byte comes from VRAM,
// above it from system memory through the GART (fetched as the LE dword the
// bus would return, then the addressed lane extracted — memory byte order is
// preserved either way).
bool texByte(const Pipe& P, u32 addr, u8& out)
{
    if (addr < kAgpWindow) {
        const auto& vram = P.c->vram;
        if (addr >= vram.size())
            return false;
        out = vram[addr];
        return true;
    }
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
// colour stands in, which keeps a bad fetch visible without a crash.
bool fetchTexel(const Pipe& P, const TexUnit& t, u32 l, u32 x, u32 y,
                Rgba& out)
{
    const u32 pitch = t.pitchTex >> l ? t.pitchTex >> l : 1u;
    const u32 base = t.offs[l] & 0x07FFFFFFu; // 31:30 are tiling flags
    // 31:30 texture mapping mode (SDK F.13): 0 linear, 1 tiled by the host
    // application, 2/3 "stored in a tiled surface" — i.e. swizzled only
    // where a SURFACE0-3 range register says so. Nanosaur sets mode 3 with
    // every SURFACE register zeroed, so its textures are linear in fact;
    // counting every such sample as unimplemented would have buried the
    // real signal under 60 M false positives per capture. Mode 1, and mode
    // 2/3 with a LIVE surface range covering the address, remain honestly
    // counted: reading those linear scrambles the texture.
    const u32 tmode = (t.offs[l] >> 30) & 3u;
    if (tmode == 1u) {
        ++eng().texUnimpl;
    } else if (tmode >= 2u) {
        for (u32 sfc = 0; sfc < 4u; ++sfc) {
            const u32 info = P.c->peek(0x0B0Cu + sfc * 0x10u);
            const u32 lo = P.c->peek(0x0B04u + sfc * 0x10u);
            const u32 hi = P.c->peek(0x0B08u + sfc * 0x10u);
            if (info && hi > lo && base >= lo && base < hi) {
                ++eng().texUnimpl;
                break;
            }
        }
    }
    ++eng().texSamples;
    auto b8 = [&](u32 at, u8& v) { return texByte(P, at, v); };
    u8 b0, b1, b2, b3;
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
    case 1u: { // CI4: two texels per byte, low nibble first
        if (!b8(base + y * pitch / 2u + x / 2u, b0))
            return false;
        const u32 idx = (x & 1u) ? (b0 >> 4) : (b0 & 0xFu);
        const u32 e = gPalette[((t.cntl >> 20) & 3u) == 2u ? 1 : 0][idx];
        out.a = 255;
        out.r = static_cast<float>((e >> 16) & 0xFFu);
        out.g = static_cast<float>((e >> 8) & 0xFFu);
        out.b = static_cast<float>(e & 0xFFu);
        return true;
    }
    case 3u: { // ARGB1555, MSB-first bytes
        const u32 at = base + (y * pitch + x) * 2u;
        if (!b8(at, b0) || !b8(at + 1u, b1))
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
        if (!b8(at, b0) || !b8(at + 1u, b1))
            return false;
        const u32 p = (static_cast<u32>(b0) << 8) | b1;
        out.a = 255;
        out.r = static_cast<float>(((p >> 11) & 0x1Fu) * 255u / 31u);
        out.g = static_cast<float>(((p >> 5) & 0x3Fu) * 255u / 63u);
        out.b = static_cast<float>((p & 0x1Fu) * 255u / 31u);
        return true;
    }
    case 5u: { // RGB888, three bytes, R first (MSB-first convention)
        const u32 at = base + (y * pitch + x) * 3u;
        if (!b8(at, b0) || !b8(at + 1u, b1) || !b8(at + 2u, b2))
            return false;
        out.a = 255;
        out.r = b0;
        out.g = b1;
        out.b = b2;
        return true;
    }
    case 6u: { // ARGB8888
        const u32 at = base + (y * pitch + x) * 4u;
        if (!b8(at, b0) || !b8(at + 1u, b1) || !b8(at + 2u, b2) ||
            !b8(at + 3u, b3))
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
    case 9u: { // RGB8 grey
        if (!b8(base + y * pitch + x, b0))
            return false;
        out.a = 255;
        out.r = out.g = out.b = b0;
        return true;
    }
    case 11u:  // YVYU 422: pairs [Y0 V Y1 U] in MSB-first byte order
    case 12u: { // VYUY 422: pairs [V Y0 U Y1]
        const u32 at = base + (y * pitch + (x & ~1u)) * 2u;
        if (!b8(at, b0) || !b8(at + 1u, b1) || !b8(at + 2u, b2) ||
            !b8(at + 3u, b3))
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
        if (!b8(at, b0) || !b8(at + 1u, b1) || !b8(at + 2u, b2) ||
            !b8(at + 3u, b3))
            return false;
        out = yuvToRgb(b1, b2, b3, static_cast<float>(b0));
        return true;
    }
    case 15u: { // ARGB4444
        const u32 at = base + (y * pitch + x) * 2u;
        if (!b8(at, b0) || !b8(at + 1u, b1))
            return false;
        const u32 p = (static_cast<u32>(b0) << 8) | b1;
        out.a = static_cast<float>(((p >> 12) & 0xFu) * 17u);
        out.r = static_cast<float>(((p >> 8) & 0xFu) * 17u);
        out.g = static_cast<float>(((p >> 4) & 0xFu) * 17u);
        out.b = static_cast<float>((p & 0xFu) * 17u);
        return true;
    }
    default: // VQ (0), CI16 (10), reserved (13): counted, borders drawn
        ++eng().texUnimpl;
        return false;
    }
}

// Wrap one coordinate per the unit's clamp mode. Returns false when border
// colour applies instead of a fetch.
bool wrapCoord(u32 mode, int i, u32 n, u32& out)
{
    const int size = static_cast<int>(n);
    switch (mode) {
    case 1u: { // mirror
        int p = i % (2 * size);
        if (p < 0)
            p += 2 * size;
        out = static_cast<u32>(p < size ? p : 2 * size - 1 - p);
        return true;
    }
    case 2u: // clamp to edge
        out = static_cast<u32>(i < 0 ? 0 : i >= size ? size - 1 : i);
        return true;
    case 3u: // border colour beyond the edge
        if (i < 0 || i >= size)
            return false;
        out = static_cast<u32>(i);
        return true;
    default: { // wrap (tile)
        int p = i % size;
        if (p < 0)
            p += size;
        out = static_cast<u32>(p);
        return true;
    }
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

// Sample one unit at (s,t) in texture space (0..1 per tile), mip level l,
// point or bilinear.
Rgba sampleLevel(const Pipe& P, const TexUnit& t, u32 l, float s, float tt,
                 bool bilinear)
{
    const u32 w = t.wTex >> l ? t.wTex >> l : 1u;
    const u32 h = t.hTex >> l ? t.hTex >> l : 1u;
    const u32 clampS = (t.cntl >> 8) & 3u;
    const u32 clampT = (t.cntl >> 11) & 3u;
    const float fx = s * static_cast<float>(w) - (bilinear ? 0.5f : 0.0f);
    const float fy = tt * static_cast<float>(h) - (bilinear ? 0.5f : 0.0f);
    const int x0 = static_cast<int>(std::floor(fx));
    const int y0 = static_cast<int>(std::floor(fy));
    if (!bilinear) {
        u32 xi, yi;
        Rgba o;
        if (!wrapCoord(clampS, x0, w, xi) || !wrapCoord(clampT, y0, h, yi) ||
            !fetchTexel(P, t, l, xi, yi, o))
            return borderColor(t);
        return o;
    }
    const float ax = fx - static_cast<float>(x0);
    const float ay = fy - static_cast<float>(y0);
    Rgba q[2][2];
    for (int dy = 0; dy < 2; ++dy)
        for (int dx = 0; dx < 2; ++dx) {
            u32 xi, yi;
            if (!wrapCoord(clampS, x0 + dx, w, xi) ||
                !wrapCoord(clampT, y0 + dy, h, yi) ||
                !fetchTexel(P, t, l, xi, yi, q[dy][dx]))
                q[dy][dx] = borderColor(t);
        }
    Rgba o;
    auto lerp2 = [&](float c00, float c01, float c10, float c11) {
        const float top = c00 + (c01 - c00) * ax;
        const float bot = c10 + (c11 - c10) * ax;
        return top + (bot - top) * ay;
    };
    o.a = lerp2(q[0][0].a, q[0][1].a, q[1][0].a, q[1][1].a);
    o.r = lerp2(q[0][0].r, q[0][1].r, q[1][0].r, q[1][1].r);
    o.g = lerp2(q[0][0].g, q[0][1].g, q[1][0].g, q[1][1].g);
    o.b = lerp2(q[0][0].b, q[0][1].b, q[1][0].b, q[1][1].b);
    return o;
}

Rgba sampleTex(const Pipe& P, const TexUnit& t, float s, float tt, float lod)
{
    // LOD bias: TEX_CNTL_C 31:24, signed 4.4 fixed point. The field width is
    // r128_reg.h's; the fixed-point split is this model's reading — revisit
    // against real traffic if mip selection looks uniformly off by a power.
    const int biasRaw = static_cast<int>((t.cntl >> 24) & 0xFFu);
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
        const Rgba a = sampleLevel(P, t, l0, s, tt, true);
        const Rgba b = sampleLevel(P, t, l1, s, tt, true);
        Rgba o;
        o.a = a.a + (b.a - a.a) * fr;
        o.r = a.r + (b.r - a.r) * fr;
        o.g = a.g + (b.g - a.g) * fr;
        o.b = a.b + (b.b - a.b) * fr;
        return o;
    }
    }
}

// ── The texture-combine unit (Tables 6-7 .. 6-16) ─────────────────────────
//
// factor selections resolve to RGBA operands; the combine function folds the
// two into the unit's output. `tex` is this unit's sample, `interp` the
// iterated vertex colour, `prev` the previous unit's output (secondary only).
Rgba pickColorFactor(const Pipe& P, u32 sel, const Rgba& tex,
                     const Rgba& interp, const Rgba& prev)
{
    Rgba o;
    switch (sel) {
    case 0u: o.r = P.constR; o.g = P.constG; o.b = P.constB; break;
    case 1u:
        o.r = 255 - P.constR; o.g = 255 - P.constG; o.b = 255 - P.constB;
        break;
    case 4u: o = tex; break;
    case 5u: o.r = 255 - tex.r; o.g = 255 - tex.g; o.b = 255 - tex.b; break;
    case 6u: o.r = o.g = o.b = tex.a; break;
    case 7u: o.r = o.g = o.b = 255 - tex.a; break;
    case 8u: o = prev; break;
    default: o = interp; break; // unnamed selections read as the iterator
    }
    return o;
}

Rgba pickInputFactor(const Pipe& P, u32 sel, const Rgba& interp,
                     const Rgba& prev)
{
    Rgba o;
    switch (sel) {
    case 2u: o.r = P.constR; o.g = P.constG; o.b = P.constB; break;
    case 3u: o.r = o.g = o.b = P.constA; break;
    case 4u: o = interp; break;
    case 5u: o.r = o.g = o.b = interp.a; break;
    case 8u: o = prev; break;
    case 9u: o.r = o.g = o.b = prev.a; break;
    default: o = interp; break;
    }
    return o;
}

float combChannel(u32 fn, float cf, float inf, float texA, float intA,
                  float constA, float texC, float cfConst)
{
    switch (fn) {
    case 1u: return cf;
    case 2u: return inf;
    case 3u: return cf * inf / 255.0f;
    case 4u: return cf * inf / 127.5f;
    case 5u: return cf * inf / 63.75f;
    case 6u: return cf + inf;
    case 7u: return cf + inf - 128.0f;
    case 8u: return (cf * intA + inf * (255.0f - intA)) / 255.0f;
    case 9u: return (cf * texA + inf * (255.0f - texA)) / 255.0f;
    case 10u: return (cf * constA + inf * (255.0f - constA)) / 255.0f;
    case 11u: return cf + inf * (255.0f - texA) / 255.0f;
    case 12u: return (cf * texA + inf * (255.0f - texA)) / 255.0f;
    case 13u: return cf + inf * texA / 255.0f;
    case 14u: return (cf + inf - 128.0f) * 2.0f;
    case 15u: return (cf * cfConst + inf * (255.0f - cfConst)) / 255.0f;
    default: return texC; // 0: disabled — the unit passes its texel through
    }
}

Rgba combineUnit(const Pipe& P, const TexUnit& t, const Rgba& tex,
                 const Rgba& interp, const Rgba& prev)
{
    const u32 comb = t.comb & 0xFu;
    const u32 cfSel = (t.comb >> 4) & 0xFu;
    const u32 inSel = (t.comb >> 10) & 0xFu;
    const u32 combA = (t.comb >> 14) & 0xFu;
    const u32 afSel = (t.comb >> 18) & 0xFu;
    const u32 inASel = (t.comb >> 25) & 0x7u;
    const Rgba cf = pickColorFactor(P, cfSel, tex, interp, prev);
    const Rgba inf = pickInputFactor(P, inSel, interp, prev);
    const float af = afSel == 7u ? 255.0f - tex.a : tex.a;
    const float inA = inASel == 1u ? P.constA
                      : inASel == 4u ? prev.a
                                     : interp.a;
    Rgba o;
    o.r = clamp255(combChannel(comb, cf.r, inf.r, tex.a, interp.a, P.constA,
                               tex.r, P.constR));
    o.g = clamp255(combChannel(comb, cf.g, inf.g, tex.a, interp.a, P.constA,
                               tex.g, P.constG));
    o.b = clamp255(combChannel(comb, cf.b, inf.b, tex.a, interp.a, P.constA,
                               tex.b, P.constB));
    switch (combA) {
    case 1u: o.a = af; break;
    case 2u: o.a = inA; break;
    case 3u: o.a = af * inA / 255.0f; break;
    case 4u: o.a = af * inA / 127.5f; break;
    case 5u: o.a = af * inA / 63.75f; break;
    case 6u: o.a = af + inA; break;
    case 7u: o.a = af + inA - 128.0f; break;
    case 14u: o.a = (af + inA - 128.0f) * 2.0f; break;
    default: o.a = tex.a; break; // 0: disabled — texture alpha through
    }
    o.a = clamp255(o.a);
    return o;
}

// TEX_LIGHT_FN / ALPHA_LIGHT_FN (Tables 6-15/6-16): the post-multitexture
// combine of the texture result with the iterated colour. The colour field
// reads 4 bits wide (17:14) — the SDK's table runs to state 15 and the gap
// to ALPHA_LIGHT at 18 is exactly four bits; r128_reg.h names only 0..7 and
// both readings agree there.
Rgba texLight(const Pipe& P, const Rgba& tex, const Rgba& interp)
{
    const u32 fn = (P.texCntl >> 14) & 0xFu;
    const u32 fnA = (P.texCntl >> 18) & 0x7u;
    Rgba o;
    o.r = clamp255(combChannel(fn, tex.r, interp.r, tex.a, interp.a,
                               P.constA, tex.r, P.constR));
    o.g = clamp255(combChannel(fn, tex.g, interp.g, tex.a, interp.a,
                               P.constA, tex.g, P.constG));
    o.b = clamp255(combChannel(fn, tex.b, interp.b, tex.a, interp.a,
                               P.constA, tex.b, P.constB));
    switch (fnA) {
    case 1u: o.a = tex.a; break;
    case 2u: o.a = interp.a; break;
    case 3u: o.a = tex.a * interp.a / 255.0f; break;
    case 4u: o.a = clamp255(tex.a * interp.a / 127.5f); break;
    case 5u: o.a = clamp255(tex.a * interp.a / 63.75f); break;
    case 6u: o.a = clamp255(tex.a + interp.a); break;
    case 7u: o.a = clamp255(tex.a + interp.a - 128.0f); break;
    default: o.a = tex.a; break;
    }
    return o;
}

// ── Destination pixels (MSB-first, the engine-wide convention) ────────────
u32 dstRead(const Pipe& P, u64 at)
{
    u32 v = 0;
    for (u32 k = 0; k < P.bypp; ++k)
        v = (v << 8) | P.c->vram[at + k];
    return v;
}

void dstWrite(const Pipe& P, u64 at, u32 v)
{
    if (P.planeMask != 0xFFFFFFFFu)
        v = (dstRead(P, at) & ~P.planeMask) | (v & P.planeMask);
    for (u32 k = 0; k < P.bypp; ++k)
        P.c->vram[at + k] = static_cast<u8>(v >> (8u * (P.bypp - 1u - k)));
}

Rgba dstDecode(const Pipe& P, u32 v)
{
    Rgba o;
    if (P.dstType == 4u) {
        o.a = 255;
        o.r = static_cast<float>(((v >> 11) & 0x1Fu) * 255u / 31u);
        o.g = static_cast<float>(((v >> 5) & 0x3Fu) * 255u / 63u);
        o.b = static_cast<float>((v & 0x1Fu) * 255u / 31u);
    } else if (P.dstType == 3u) {
        o.a = (v & 0x8000u) ? 255.0f : 0.0f;
        o.r = static_cast<float>(((v >> 10) & 0x1Fu) * 255u / 31u);
        o.g = static_cast<float>(((v >> 5) & 0x1Fu) * 255u / 31u);
        o.b = static_cast<float>((v & 0x1Fu) * 255u / 31u);
    } else {
        o.a = static_cast<float>(v >> 24);
        o.r = static_cast<float>((v >> 16) & 0xFFu);
        o.g = static_cast<float>((v >> 8) & 0xFFu);
        o.b = static_cast<float>(v & 0xFFu);
    }
    return o;
}

u32 dstEncode(const Pipe& P, const Rgba& c)
{
    const u32 r = static_cast<u32>(clamp255(c.r) + 0.5f);
    const u32 g = static_cast<u32>(clamp255(c.g) + 0.5f);
    const u32 b = static_cast<u32>(clamp255(c.b) + 0.5f);
    const u32 a = static_cast<u32>(clamp255(c.a) + 0.5f);
    if (P.dstType == 4u)
        return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
    if (P.dstType == 3u)
        return ((a >> 7) << 15) | ((r >> 3) << 10) | ((g >> 3) << 5) |
               (b >> 3);
    return (a << 24) | (r << 16) | (g << 8) | b;
}

// ── Z / stencil cells ─────────────────────────────────────────────────────
bool zAddr(const Pipe& P, int px, int py, u64& at)
{
    const u32 zb = P.zw == 16u ? 2u : 4u;
    at = static_cast<u64>(P.zOff) +
         static_cast<u64>(py) * P.zStride + static_cast<u64>(px) * zb;
    return P.zStride != 0 && at + zb <= P.c->vram.size();
}

u32 zRead(const Pipe& P, u64 at)
{
    const u32 zb = P.zw == 16u ? 2u : 4u;
    u32 v = 0;
    for (u32 k = 0; k < zb; ++k)
        v = (v << 8) | P.c->vram[at + k];
    return v;
}

void zStore(const Pipe& P, u64 at, u32 v)
{
    const u32 zb = P.zw == 16u ? 2u : 4u;
    for (u32 k = 0; k < zb; ++k)
        P.c->vram[at + k] = static_cast<u8>(v >> (8u * (zb - 1u - k)));
}

bool cmp8(u32 fn, u32 src, u32 ref)
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

u32 stencilOp(u32 op, u32 cur, u32 ref)
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
void blendFactor(u32 sel, const Rgba& s, const Rgba& d, bool isSrc,
                 float f[4])
{
    auto set = [&](float r, float g, float b, float a) {
        f[0] = r; f[1] = g; f[2] = b; f[3] = a;
    };
    switch (sel) {
    case 0u: set(0, 0, 0, 0); break;
    case 2u: set(s.r, s.g, s.b, s.a); break;
    case 3u: set(255 - s.r, 255 - s.g, 255 - s.b, 255 - s.a); break;
    case 4u: set(s.a, s.a, s.a, s.a); break;
    case 5u: set(255 - s.a, 255 - s.a, 255 - s.a, 255 - s.a); break;
    case 6u: set(d.a, d.a, d.a, d.a); break;
    case 7u: set(255 - d.a, 255 - d.a, 255 - d.a, 255 - d.a); break;
    case 8u: set(d.r, d.g, d.b, d.a); break;
    case 9u: set(255 - d.r, 255 - d.g, 255 - d.b, 255 - d.a); break;
    case 10u: { // SRCALPHASAT: (f,f,f,1), f = min(As, 1-Ad)
        const float m = s.a < 255 - d.a ? s.a : 255 - d.a;
        set(m, m, m, 255);
        break;
    }
    case 11u: // BOTHSRCALPHA: src=As, forces dst=1-As (caller handles pair)
        if (isSrc) set(s.a, s.a, s.a, s.a);
        else set(255 - s.a, 255 - s.a, 255 - s.a, 255 - s.a);
        break;
    case 12u: // BOTHINVSRCALPHA
        if (isSrc) set(255 - s.a, 255 - s.a, 255 - s.a, 255 - s.a);
        else set(s.a, s.a, s.a, s.a);
        break;
    default: set(255, 255, 255, 255); break; // 1: ONE
    }
}

// ── Interpolated fragment attributes handed to the shader ─────────────────
struct Frag {
    float z = 0;
    Rgba diffuse, spec;
    float fog = 255;
    float s1 = 0, t1 = 0, s2 = 0, t2 = 0; // perspective-resolved
    float lod1 = 0, lod2 = 0;
};

// The whole per-pixel back end. False = fragment rejected before the write.
bool shadeFragment(const Pipe& P, int px, int py, const Frag& fr)
{
    Rgba color = fr.diffuse;
    if (P.texAny) {
        const TexUnit& t0 = P.tex[0];
        const Rgba tex0 = sampleTex(P, t0, fr.s1, fr.t1, fr.lod1);
        if (P.chromaEn) {
            // Texture chroma key: a texel matching CLR_CMP through the mask
            // kills the fragment. Compared in ARGB8888 space.
            const u32 t32 = (static_cast<u32>(tex0.a + 0.5f) << 24) |
                            (static_cast<u32>(tex0.r + 0.5f) << 16) |
                            (static_cast<u32>(tex0.g + 0.5f) << 8) |
                            static_cast<u32>(tex0.b + 0.5f);
            if ((t32 & P.chromaMsk) == (P.chromaClr & P.chromaMsk))
                return false;
        }
        Rgba out = combineUnit(P, t0, tex0, fr.diffuse, fr.diffuse);
        if (P.tex[1].on) {
            const TexUnit& t1 = P.tex[1];
            const float s = t1.useSecondSt ? fr.s2 : fr.s1;
            const float tt = t1.useSecondSt ? fr.t2 : fr.t1;
            const Rgba tex1 =
                sampleTex(P, t1, s, tt, t1.useSecondSt ? fr.lod2 : fr.lod1);
            out = combineUnit(P, t1, tex1, fr.diffuse, out);
        }
        color = texLight(P, out, fr.diffuse);
    }
    if (P.specEn) {
        color.r = clamp255(color.r + fr.spec.r);
        color.g = clamp255(color.g + fr.spec.g);
        color.b = clamp255(color.b + fr.spec.b);
    }
    if (P.fogEn) {
        // f=255 is "all primitive colour". Vertex fog interpolates the
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
        color.r = (color.r * f + P.fogR * (255.0f - f)) / 255.0f;
        color.g = (color.g * f + P.fogG * (255.0f - f)) / 255.0f;
        color.b = (color.b * f + P.fogB * (255.0f - f)) / 255.0f;
    }
    if (P.alphaTestEn &&
        !cmp8(P.alphaOp, static_cast<u32>(clamp255(color.a) + 0.5f),
              P.refAlpha))
        return false;
    // Z + stencil, evaluated together because the stencil result depends on
    // the Z verdict (S8 shares the Z24 cell).
    u64 za = 0;
    bool zPass = true;
    const bool wantZ = P.zEn || (P.stEn && P.zw == 24u);
    if (wantZ) {
        if (!zAddr(P, px, py, za)) {
            ++eng().offVram;
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
        ++eng().offVram;
        return false;
    }
    if (P.blendEn) {
        const Rgba dst = dstDecode(P, dstRead(P, at));
        float fs[4], fd[4];
        blendFactor(P.blendSrc, color, dst, true, fs);
        // The BOTH* source selections force the destination factor; the
        // guest's own dst field is ignored then (Table 6-17's wording).
        if (P.blendSrc == 11u || P.blendSrc == 12u)
            blendFactor(P.blendSrc, color, dst, false, fd);
        else
            blendFactor(P.blendDst, color, dst, true, fd);
        const bool sub = P.blendComb >= 2u;
        const bool clamp = !(P.blendComb & 1u);
        auto ch = [&](float sc, float dc, float sf, float df) {
            float v = sub ? (sc * sf - dc * df) / 255.0f
                          : (sc * sf + dc * df) / 255.0f;
            if (clamp)
                v = clamp255(v);
            return v;
        };
        color.r = ch(color.r, dst.r, fs[0], fd[0]);
        color.g = ch(color.g, dst.g, fs[1], fd[1]);
        color.b = ch(color.b, dst.b, fs[2], fd[2]);
        color.a = ch(color.a, dst.a, fs[3], fd[3]);
    }
    dstWrite(P, at, dstEncode(P, color));
    ++eng().triPixels;
    return true;
}

// ── Triangle rasterizer ───────────────────────────────────────────────────
//
// Screen-space edge functions, D3D top-left fill rule, pixel centers at
// +0.5. Colours interpolate screen-linear (the era's hardware did not
// perspective-correct Gouraud); textures interpolate s·rhw, t·rhw, rhw
// linearly and divide per pixel unless the unit disables perspective.
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
    struct Edge {
        double a, b, c; // e = a*x + b*y + c, interior > 0 for CW/y-down
        bool topLeft;
    } e[3];
    auto mkEdge = [](double ax, double ay, double bx, double by) {
        // e = (bx-ax)(y-ay) - (by-ay)(x-ax), expanded to a*x + b*y + c.
        Edge ed;
        ed.a = ay - by;
        ed.b = bx - ax;
        ed.c = -(ed.a * ax + ed.b * ay);
        // D3D top-left rule in y-down CW terms: a top edge runs rightward
        // horizontally, a left edge runs upward.
        const double dx = bx - ax, dy = by - ay;
        ed.topLeft = (dy == 0.0 && dx > 0.0) || dy < 0.0;
        return ed;
    };
    e[0] = mkEdge(x0, y0, x1, y1);
    e[1] = mkEdge(x1, y1, x2, y2);
    e[2] = mkEdge(x2, y2, x0, y0);
    const double inv2a = 1.0 / area2;
    // Attribute planes (screen-affine). Textures build s·rhw / t·rhw / rhw
    // numerator planes for perspective; a unit with perspective disabled
    // uses plain s,t planes instead.
    const Plane pz = planeOf(x0, y0, x1, y1, x2, y2, v0.z, v1.z, v2.z, inv2a);
    const Plane pa = planeOf(x0, y0, x1, y1, x2, y2, v0.ca, v1.ca, v2.ca, inv2a);
    const Plane pr = planeOf(x0, y0, x1, y1, x2, y2, v0.cr, v1.cr, v2.cr, inv2a);
    const Plane pg = planeOf(x0, y0, x1, y1, x2, y2, v0.cg, v1.cg, v2.cg, inv2a);
    const Plane pb = planeOf(x0, y0, x1, y1, x2, y2, v0.cb, v1.cb, v2.cb, inv2a);
    const Plane psr = planeOf(x0, y0, x1, y1, x2, y2, v0.sr, v1.sr, v2.sr, inv2a);
    const Plane psg = planeOf(x0, y0, x1, y1, x2, y2, v0.sg, v1.sg, v2.sg, inv2a);
    const Plane psb = planeOf(x0, y0, x1, y1, x2, y2, v0.sb, v1.sb, v2.sb, inv2a);
    const Plane pfog = planeOf(x0, y0, x1, y1, x2, y2, v0.fog, v1.fog, v2.fog, inv2a);
    const Plane pw = planeOf(x0, y0, x1, y1, x2, y2, v0.rhw, v1.rhw, v2.rhw, inv2a);
    const bool persp0 = P.tex[0].on && !((P.tex[0].cntl >> 14) & 1u);
    const bool persp1 = P.tex[1].on && !((P.tex[1].cntl >> 14) & 1u);
    const Plane ps1 = planeOf(x0, y0, x1, y1, x2, y2,
                              persp0 ? v0.s1 * v0.rhw : v0.s1,
                              persp0 ? v1.s1 * v1.rhw : v1.s1,
                              persp0 ? v2.s1 * v2.rhw : v2.s1, inv2a);
    const Plane pt1 = planeOf(x0, y0, x1, y1, x2, y2,
                              persp0 ? v0.t1 * v0.rhw : v0.t1,
                              persp0 ? v1.t1 * v1.rhw : v1.t1,
                              persp0 ? v2.t1 * v2.rhw : v2.t1, inv2a);
    const Plane ps2 = planeOf(x0, y0, x1, y1, x2, y2,
                              persp1 ? v0.s2 * v0.rhw : v0.s2,
                              persp1 ? v1.s2 * v1.rhw : v1.s2,
                              persp1 ? v2.s2 * v2.rhw : v2.s2, inv2a);
    const Plane pt2 = planeOf(x0, y0, x1, y1, x2, y2,
                              persp1 ? v0.t2 * v0.rhw : v0.t2,
                              persp1 ? v1.t2 * v1.rhw : v1.t2,
                              persp1 ? v2.t2 * v2.rhw : v2.t2, inv2a);
    ++eng().tris;
    for (int py = minY; py <= maxY; ++py) {
        const double cy = py + 0.5;
        for (int px = minX; px <= maxX; ++px) {
            const double cx = px + 0.5;
            bool in = true;
            for (int k = 0; k < 3 && in; ++k) {
                const double ev = e[k].a * cx + e[k].b * cy + e[k].c;
                in = ev > 0.0 || (ev == 0.0 && e[k].topLeft);
            }
            if (!in)
                continue;
            Frag fr;
            fr.z = static_cast<float>(pz.at(cx, cy));
            fr.diffuse.a = clamp255(static_cast<float>(pa.at(cx, cy)));
            fr.diffuse.r = clamp255(static_cast<float>(pr.at(cx, cy)));
            fr.diffuse.g = clamp255(static_cast<float>(pg.at(cx, cy)));
            fr.diffuse.b = clamp255(static_cast<float>(pb.at(cx, cy)));
            fr.spec.r = clamp255(static_cast<float>(psr.at(cx, cy)));
            fr.spec.g = clamp255(static_cast<float>(psg.at(cx, cy)));
            fr.spec.b = clamp255(static_cast<float>(psb.at(cx, cy)));
            fr.fog = clamp255(static_cast<float>(pfog.at(cx, cy)));
            const double w = pw.at(cx, cy);
            const double invW = (persp0 || persp1) && w != 0.0 ? 1.0 / w : 1.0;
            fr.s1 = static_cast<float>(persp0 ? ps1.at(cx, cy) * invW
                                              : ps1.at(cx, cy));
            fr.t1 = static_cast<float>(persp0 ? pt1.at(cx, cy) * invW
                                              : pt1.at(cx, cy));
            fr.s2 = static_cast<float>(persp1 ? ps2.at(cx, cy) * invW
                                              : ps2.at(cx, cy));
            fr.t2 = static_cast<float>(persp1 ? pt2.at(cx, cy) * invW
                                              : pt2.at(cx, cy));
            // Per-pixel LOD from the analytic derivatives of s,t: for
            // S = Sn/W with Sn, W screen-affine, dS/dx = (Sn'x·W − Sn·W'x)/W².
            if (P.tex[0].on) {
                const double sw = static_cast<double>(P.tex[0].wTex);
                const double sh = static_cast<double>(P.tex[0].hTex);
                double dsx, dsy, dtx, dty;
                if (persp0 && w != 0.0) {
                    const double w2 = 1.0 / (w * w);
                    dsx = (ps1.a * w - ps1.at(cx, cy) * pw.a) * w2;
                    dsy = (ps1.b * w - ps1.at(cx, cy) * pw.b) * w2;
                    dtx = (pt1.a * w - pt1.at(cx, cy) * pw.a) * w2;
                    dty = (pt1.b * w - pt1.at(cx, cy) * pw.b) * w2;
                } else {
                    dsx = ps1.a; dsy = ps1.b; dtx = pt1.a; dty = pt1.b;
                }
                const double rx = dsx * sw * dsx * sw + dtx * sh * dtx * sh;
                const double ry = dsy * sw * dsy * sw + dty * sh * dty * sh;
                const double rho2 = rx > ry ? rx : ry;
                fr.lod1 = rho2 > 0.0
                              ? static_cast<float>(0.5 * std::log2(rho2))
                              : 0.0f;
            }
            if (P.tex[1].on)
                fr.lod2 = fr.lod1; // second set: same footprint model
            shadeFragment(P, px, py, fr);
        }
    }
}

// ── Lines and points ──────────────────────────────────────────────────────
void rasterPoint(const Pipe& P, const Vtx& v)
{
    const int px = static_cast<int>(std::floor(v.x)) + P.winX;
    const int py = static_cast<int>(std::floor(v.y)) + P.winY;
    if (px < P.scL || px > P.scR || py < P.scT || py > P.scB || px < 0 ||
        py < 0)
        return;
    Frag fr;
    fr.z = v.z;
    fr.diffuse = {v.ca, v.cr, v.cg, v.cb};
    fr.spec = {255, v.sr, v.sg, v.sb};
    fr.fog = v.fog;
    fr.s1 = v.s1; fr.t1 = v.t1; fr.s2 = v.s2; fr.t2 = v.t2;
    ++eng().points3d;
    shadeFragment(P, px, py, fr);
}

void rasterLine(const Pipe& P, const Vtx& a, const Vtx& b)
{
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
        v.s1 = a.s1 + (b.s1 - a.s1) * f;
        v.t1 = a.t1 + (b.t1 - a.t1) * f;
        v.s2 = a.s2 + (b.s2 - a.s2) * f;
        v.t2 = a.t2 + (b.t2 - a.t2) * f;
        const int px = static_cast<int>(std::floor(v.x)) + P.winX;
        const int py = static_cast<int>(std::floor(v.y)) + P.winY;
        if (px < P.scL || px > P.scR || py < P.scT || py > P.scB ||
            px < 0 || py < 0)
            continue;
        Frag fr;
        fr.z = v.z;
        fr.diffuse = {v.ca, v.cr, v.cg, v.cb};
        fr.spec = {255, v.sr, v.sg, v.sb};
        fr.fog = v.fog;
        fr.s1 = v.s1; fr.t1 = v.t1; fr.s2 = v.s2; fr.t2 = v.t2;
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
        return src.fetch(vi, v);
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

} // namespace

// ── Entry points from the parser and the register engine ──────────────────

void r128Cce3dReset()
{
    std::memset(gFog.t, 0xFF, sizeof gFog.t); // f=255: no fog
    std::memset(gPalette, 0, sizeof gPalette);
    gWalk = WalkLatch{};
    gGate3d = -1;
    gLogBudget = 24;
}

int r128Gate3dState() { return gGate3d; }

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
        // sat in Scaling — 4,392 of them in the first real Nanosaur session,
        // the only refusal in the whole capture. Dropped state does not
        // announce itself: the primitives still draw, using whatever the
        // registers held before.
        // ⭐⭐ AND IT IS STEERED BY BOTH REGISTERS TOGETHER, NOT BY WHICHEVER
        // WAS WRITTEN LAST. Measured from Nanosaur: the driver runs with
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
            return true; // dropped, the documented behaviour
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
        assemble(P, src, cntl & 0xFu, count, nullptr, false);
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
        assemble(P, src, cntl & 0xFu, count, idxWords, walk == 1u);
        return true;
    }
    return false;
}

} // namespace opm
