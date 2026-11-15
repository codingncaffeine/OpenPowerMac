#pragma once
#include <string>
#include "opm/types.hpp"

#include <atomic>
#include <map>
#include <utility>
#include <vector>

namespace opm {

struct SnapWriter;
struct SnapReader;
class R128Cell;
// The card is going away (or a machine is starting): the command engine's
// own thread, if it is working, forgets its queue. With a cell, only if that
// is the cell it is working on. See "The command engine on its own thread".
void r128AsyncAbandon(const R128Cell* c);

// ATI Rage 128 Pro AGP ('PF', 1002:5046) — the Sawtooth's video card.
// Bring-up model: the card's own FCode ROM (user-supplied dump, served
// through the expansion-ROM BAR) programs the chip and publishes the
// ATY,Rage128Ps node with the embedded OS 9 display driver; this cell
// starts as an honest logged register store with just enough live
// behavior for the FCode's probes, and grows register-by-register from
// what the driver actually reads. The register block is little-endian
// on PCI (guests use lwbrx/stwbrx), same edge-swap convention as the
// OHCI cell.
class R128Cell {
public:
    u32 read(u32 off, u32 len);
    void write(u32 off, u32 v, u32 len);

    // Framebuffer aperture (BAR0): raw memory, both endian apertures
    // land here (the FCode/driver picks its own swap discipline).
    std::vector<u8> vram;
    // Framebuffer write census — see Sawtooth::write.
    u64 fbWrites = 0;
    u32 fbBase = 0; // CONFIG_APER_0_BASE: where the OS should paint
    u32 regBase = 0; // REG_BASE (the 16 KB BAR): aperture 0; aperture 1 is +0x2000
    u32 fbLo = 0xFFFFFFFFu, fbHi = 0;

    R128Cell() : vram(32u << 20, 0) {}
    // The engine's thread must not be left drawing into a card that is gone.
    ~R128Cell() { r128AsyncAbandon(this); }

    struct Ev {
        u64 at;
        u32 off, val, pc;
        bool wr;
    };
    std::vector<Ev> log; // first-touch + write traffic
    u64 logFrom = 0;     // --ati-log-from N: ignore traffic before N
    // A COMPLETE census of which registers the guest ever touches. `log` is
    // capped and printed head-and-tail, so "offset X never appears in it" is
    // NOT evidence that the guest never wrote X — exactly the truncation trap
    // that has produced several wrong conclusions on this project. This map is
    // uncapped, and is the only honest answer to questions of the form "does
    // the driver ever use the hardware cursor".
    std::map<u32, u64> writeCount, readCount;
    u32 crtcShown_ = 0;  // mode-change reports emitted (see write())
    const u64* stamp = nullptr;
    const u32* pcRef = nullptr;

    // Monitor identification over DDC, bit-banged on the GPIO trio the
    // Rage 128's FCode uses: 0x00A8 carries SCL, 0x00A0 carries SDA, and
    // 0x00A4 reads the resulting bus. Each line register holds a direction
    // bit at 22 and a level bit at 23; the lines are open drain, so an
    // undriven line is high and either side can pull it down.
    //
    // Without a slave the bus is dead and Open Firmware's display bring-up
    // has nothing to identify, which is the state a black screen with a
    // completed modeset comes from.
    struct Ddc {
        u8 state = 0;    // 0 idle, 1 addr, 2 ack-addr, 3 data, 4 ack-data
        u8 shift = 0;    // byte being clocked
        u8 bits = 0;     // bits seen in this byte
        u8 addr = 0;     // device address latched by the last START
        u8 ptr = 0;      // byte offset within the EDID
        bool sdaOut = true; // what the slave drives (true = released)
        bool lastScl = true, lastSda = true;
    } ddc_;
    u32 gpioScl_ = 0, gpioSda_ = 0;
    // DDC census: starts seen, address matches, EDID bytes served. A
    // bus that is being clocked and a slave that is answering are
    // different claims.
    u32 ddcStarts = 0, ddcMatches = 0, ddcBytes = 0;
    u32 ddcLastAddr = 0x100;
    // The raw waveform: (0xA0, 0xA8) after every write to either. Four
    // guesses at which register is level and which bit is the clock all
    // failed; the pairs themselves say what the lines are doing.
    std::vector<std::pair<u32, u32>> ddcWave;
    bool ddcSda() const { return ddc_.sdaOut; }
    void ddcStep(bool scl, bool sda);
    u8 edidByte(u8 at) const;

    // Harness peeks for the screen dump: raw register cell and the DAC
    // palette (PALETTE_INDEX/DATA auto-increment writes captured).
    u32 peek(u32 off) const
    {
        auto it = regs_.find(off & ~3u);
        return it != regs_.end() ? it->second : 0;
    }
    u32 pal(u32 i) const { return pal_[i & 0xFFu]; }
    // Set a register in NATIVE (little-endian) form from outside the guest's
    // register path — for the bus-master engine, which writes registers as a
    // RESULT rather than because the guest stored to them. A method, not a
    // member, so sizeof(R128Cell) and every snapshot are untouched.
    void setReg(u32 off, u32 nativeVal) { regs_[off & ~3u] = nativeVal; }

    // --- Vertical blank as an INTERRUPT, not merely a status bit ----------
    //
    // The pointer is moved by the SLOT VBL path and by nothing else: the
    // display driver's interrupt handler calls the ROM's slot-VBL dispatcher
    // (68K ffc0bcc0 through $0D28, native ffd94798), which — only for the
    // queue $0D10 names as the PRIMARY display — calls JCrsrTask ($08EE), and
    // JCrsrTask is the ONLY code that drains the Cursor Device Manager's delta
    // accumulators at CursorDevice+112..124 into CursorData.whereX/whereY.
    // CursorDeviceMove piles the deltas up correctly and returns noErr, so a
    // poll-only vblank yields a machine where every USB report is delivered,
    // parsed and accumulated and the cursor still never moves one pixel.
    //
    // From the register semantics: GEN_INT_CNTL bit 0 is CRTC_VBLANK_INT,
    // GEN_INT_STATUS latches it and is WRITE-1-TO-CLEAR, and the pin is a
    // LEVEL asserted while an enabled source stands latched.
    void tick(u64 tb);
    // The earliest timebase at which tick() could matter, so the machine loop
    // can stop calling it once per instruction; and a way to keep the cell's
    // notion of "now" exact in between, because the driver ARMS the blank from
    // inside a register write, before any tick can run.
    u64 nextTickTb() const;
    void noteTb(u64 tb) { tbNow_ = tb; }
    bool irqLine() const;
    // Census. "A blank fired" and "a blank reached the CPU" are different
    // claims: the OS programs this source's vector and priority and can still
    // leave it MASKED in the OpenPIC, in which case every raise goes nowhere.
    u64 vblanks = 0;        // blanks generated
    u64 vblIrqs = 0;        // blanks that found CRTC_VBLANK_INT enabled
    u64 vblAcks = 0; // acknowledges the driver wrote
    // Model the vertical blank as an INTERRUPT. ON by default, because it is
    // the only model in which the mouse pointer moves at all; `--no-ati-vbl`
    // restores the poll-only cell bit for bit.
    //
    // ⚠⚠ THIS DEFAULT IS LOAD-BEARING FOR THE SHIPPING APP. The capi never
    // touches these fields, so the shell takes the CONSTRUCTOR defaults —
    // exactly how the ganged-power bug reached the app once already. Anything
    // changed here has to be traced into the shell, and the PERIOD matters as
    // much as the switch: with the nominal 60 Hz the guest is flooded and
    // livelocks (see kTbPerVblank).
    bool vblEnabled = true;
    // --ati-vbl-tb N: timebase ticks between blanks, 0 = nominal 60 Hz at
    // 25 MHz. A STATIC, deliberately: it is a harness knob rather than machine
    // state, and keeping it out of the object leaves sizeof(R128Cell) — and so
    // every snapshot — untouched, the same reason the read watchpoint lives on
    // Cpu instead of CpuState.
    //
    // ⚠ THE NOMINAL RATE IS WRONG UNDER --fast-tb, AND NOT BY A LITTLE. That
    // switch advances the timebase far faster than instructions retire, so
    // guest time is COMPRESSED: over a boot to the desktop the machine logged
    // tb = 173,964,736,438 (6,958 s at 25 MHz) while the OS's own Ticks reached
    // only 9358 (156 s) — a factor of 44.6. A tb-paced 60 Hz vblank therefore
    // arrives ~2,550 times per second of the guest's OWN time. Polling devices
    // survive that (nothing gates on the OHCI frame rate); an INTERRUPT does
    // not — the machine livelocks in the 68K autovector dispatch and stops
    // retiring USB reports altogether.
    static void setVblTbPeriod(u64 n);
    static u64 vblTbPeriod();
    // --vbl-trace N: print the first N latch/acknowledge events. Aggregate
    // counts said "acks=2 out of 14 blanks" and could not say WHICH blank went
    // unanswered or what the driver wrote; only the sequence can. A static, so
    // no snapshot is invalidated.
    static void setVblTrace(int n);
    static u64 vblDropped();
    // The period actually in force. Reported rather than recomputed at the
    // print site: that duplicate kept its own stale fallback and announced
    // 416,666 while the cell was using 225,000,000.
    static u64 vblPeriodEffective();

    // Snapshot: register store, PLL file, DAC palette, and the 32 MB
    // framebuffer — the display work after the mount reads all of it.
    void snapSave(SnapWriter& w) const;
    void snapLoad(SnapReader& r);

private:
    static u32 swap32(u32 v)
    {
        return (v >> 24) | ((v >> 8) & 0xFF00u) | ((v << 8) & 0xFF0000u) |
               (v << 24);
    }
    u32 regRead(u32 idx);
    void note(u32 off, u32 val, bool wr);
    // --- The 2D engine ---------------------------------------------------
    //
    // ⭐ ALL ENGINE STATE LIVES IN `regs_`, WHICH IS ALREADY SNAPSHOTTED, AND
    // THAT IS THE WHOLE DESIGN. A member struct of engine registers would
    // change sizeof(R128Cell) and therefore sizeof(SawtoothBus), which the
    // layout digest hashes — every existing snapshot would die and
    // kSnapVersion would have to move. Instead the aliases and the composite
    // registers are NORMALIZED into their canonical offsets as they are
    // written, and a blit reads its parameters back out of the same map. The
    // engine adds no state of its own at all; it only adds behavior.
    u32 rd(u32 off) const
    {
        auto it = regs_.find(off);
        return it != regs_.end() ? it->second : 0;
    }
    // Execute the pending operation. Called from write() on the register the
    // guest wrote LAST — the Rage's trajectory registers "initiate draw
    // operations" (§2.1.5 of ATI's reference guide), so the trigger is a
    // write, not a command word.
    void engBlit();
    void engLine(u32 length);
    void engHostBegin(u32 rop, u32 bypp, bool byteAligned);
    void engHostWord(u32 v, bool last);
    // Absorb a write into the engine block: normalize aliases, apply the
    // side effects of a DP_GUI_MASTER_CNTL write, and initiate on a trigger.
    // Returns true when it has fully handled the write.
    bool engWrite(u32 off, u32 v);

    std::map<u32, u32> regs_; // sparse store, offsets word-aligned
    std::map<u32, u32> seen_; // first-touch dedup for the log
    bool gateOpened_ = false; // logFrom window has been entered
    u32 pllAddr_ = 0;         // CLOCK_CNTL_INDEX low byte
    std::map<u32, u32> pll_;  // PLL register file
    u32 palIdx_ = 0;
    u32 pal_[256] = {};
    // Timebase at which the next blank falls due. Zero means the clock is
    // parked: it starts when the driver arms CRTC_VBLANK_INT, not at reset,
    // so a machine whose driver never asks for the interrupt behaves exactly
    // as it did before.
    u64 vblNextTb_ = 0;
    u64 tbNow_ = 0; // last timebase this cell was ticked at
    u64 vblPeriod() const;
};

// Reads that fell through to the register model's default with nothing ever
// written at that offset — reads this card answered with zero and always will.
// See the census in r128.cpp for why this is separate from readCount.
const std::map<u32, u64>& r128RegReadsUnbacked();

// --- The scan-out: what the CRTC is displaying, decoded ONCE ---------------
//
// Two harnesses read the framebuffer — capi's opm_screen for the app and
// g4run's ppm dump for the gates — and each carried its own copy of the CRTC
// decode and the pixel conversion. They drifted once already (the olive
// desktop: 32 bpp read as little-endian in one of them), and they agreed on a
// second defect: both refused every pixel format except 8 and 32 bpp, so a
// game that switched the display to 15/16 bpp played to a harness that
// rendered nothing at all — while the machine underneath was alive and
// beeping. One decode, one conversion table, both callers.
//
// Free functions, deliberately: a member would change sizeof(R128Cell) and
// therefore sizeof(SawtoothBus), which the snapshot layout digest hashes.
//
// THE DAC'S TABLE, AND THE MONITOR BEHIND IT. Every pixel goes to the DAC
// through the 256-entry table (PALETTE_INDEX / PALETTE_DATA), in every mode:
// in 8 bpp the pixel is the index, and in the direct-color modes each
// channel indexes its own third of the entry. That is where Mac OS keeps its
// gamma ramp (entry 128 holds 158), and where a game fades the screen, by
// loading the ramp scaled down a step at a time.
//
// How a channel narrower than eight bits indexes the table is in none of the
// manuals; a driver's usage pins it. Linux's aty128fb, the driver these
// machines ran, writes exactly one entry for each value of a 5-bit channel,
// at value * 8, and one for each value of the 6-bit green, at value * 4. So
// the index is the channel's bits with zeros below them, and white in
// thousands of colors is entry 248, not 255.
//
// What leaves the DAC drives a monitor, and the ramp was computed for one:
// it is close to a 1/1.44 power, which on a tube of gamma 2.6 gives the Mac's
// 1.8. A host screen is not that tube, so the last stage is the monitor: the
// light a 2.6 tube makes of the value, expressed in the host's sRGB
// (r128MonitorCurve). Net: the Mac's own tone, mid-gray 128 shown as 146.
// The hardware cursor's two colors are "always 24 bpp, independent of
// graphics mode" (guide, CUR_CLR0): they reach the DAC without the table,
// and the monitor like everything else.
struct R128Scan {
    bool enabled = false; // CRTC_GEN_CNTL bit 25: CRTC_EN
    u32 fmt = 0;          // CRTC_GEN_CNTL[11:8]: CRTC_PIX_WIDTH
    u32 w = 0, h = 0;     // display size in pixels, from H/V_TOTAL_DISP
    u32 pitch8 = 0;       // CRTC_PITCH: row stride in 8-pixel units
    u32 offset = 0;       // CRTC_OFFSET: byte address of the top-left pixel
    u32 bypp = 0;         // bytes per pixel; 0 = a format this cannot scan
    u32 rowBytes = 0;     // pitch8 * 8 * bypp
    // What the screen shows for a table index, per channel (red, green,
    // blue): the entry's channel, then the monitor. Filled by the decode, so
    // a row conversion is three lookups a pixel.
    u8 lut[3][256] = {};
    // CRTC_OFFSET_CNTL bit 15 (CRTC_TILE_EN): the scan-out is reading a
    // TILED surface. This model walks VRAM linearly, so a tiled framebuffer
    // comes out sheared into diagonal bands with scrambled color — a
    // symptom that is otherwise indistinguishable from a stride bug.
    // Reported rather than silently mis-rendered.
    bool tiled = false;
};
R128Scan r128ScanDecode(const R128Cell& c);
// The monitor stage alone: 256 entries, DAC level to host level.
const u8* r128MonitorCurve();
// A short name for the format, for reports: "8bpp CLUT", "15bpp 1555", ...
const char* r128ScanFmtName(u32 fmt);
// One display row, VRAM to host BGRA (B,G,R,X=FF per pixel; `out` holds
// s.w * 4 bytes). Pixels past the end of VRAM render black.
void r128ScanRow(const R128Cell& c, const R128Scan& s, u32 y, u8* out);
// The hardware cursor, composited over a full s.w × s.h BGRA frame. Mac OS
// draws its pointer with the card's cursor engine, not into the framebuffer,
// so a scan-out that reads VRAM alone shows no pointer at all. No-op unless
// CRTC_GEN_CNTL bit 16 (cursor enable) is set.
void r128ScanCursor(const R128Cell& c, const R128Scan& s, u8* bgra);

// --- Bits the guest sets that nothing here reads -----------------------------
//
// A control bit this model does not decode is invisible until a picture is
// wrong, and then it looks like a drawing bug: a game's menu buttons drew as
// black rectangles because one bit of TEX_CNTL_C was not read. So every
// register write also folds its value into "bits ever written as one", and
// r128RegDecodedBits says, for the state registers, which bits the model
// reads or knowingly leaves alone. The difference is the list of features a
// guest is asking for and not getting; the diagnostics print it.
//
// ⚠ THE MASKS ARE A SECOND STATEMENT OF WHAT THE DECODERS DO, kept next to
// them (r128_3d.cpp for the 3D context, r128.cpp for the 2D engine) and to be
// changed with them. A register with no mask is reported as not audited.
u32 r128RegEverSet(u32 off);
void r128RegEverReset();
// The bits of register `off` this model decodes. False if it has no entry.
bool r128RegDecodedBits(u32 off, u32& mask);
// The list above as the diagnostics print it.
std::string r128UndecodedReport(const R128Cell& c);

// The bit census cannot see a setting that takes one of several VALUES, some
// of which the model draws and some not, and least of all one whose zero is
// a mode (the dither method, the vertex precision, how texture coordinates
// are given). So each such setting is noted with its value WHEN A DRAW USES
// IT: the 2D engine notes its own at every operation, the 3D pipeline at
// every packet of primitives, and only the settings that are in force (the
// depth test when depth is on, a unit's filter when the unit is on). The
// report names every value drawn with, and marks the ones this model draws
// some other way. The table is in r128_census.cpp.
enum class R128Setting : u8 {
    Dst2d, Brush2d, SrcFormat2d, SrcFrom2d, Tile2d, Gmc3dFn,
    Dst3d, ZWidth, ZTest, StencilTest, StencilFail, StencilZPass,
    StencilZFail, BlendSrc, BlendDst, BlendCombine, AlphaTest, DitherMethod,
    DitherInit, SubPixel, TexCoords, Shading, BackFace, FrontFace,
    TexAntiAlias, TexColorKey, EdgeAntiAlias, Supersample, LightColor,
    LightAlpha,
    // One run per texture unit, in this order; the second unit's begin at
    // Tex2MinFilter.
    Tex1MinFilter, Tex1MagFilter, Tex1ClampS, Tex1ClampT, Tex1Format,
    Tex1PalEntries, Tex1Combine, Tex1ColorFactor, Tex1ColorInput,
    Tex1CombineAlpha, Tex1AlphaFactor, Tex1AlphaInput,
    Tex2MinFilter, Tex2MagFilter, Tex2ClampS, Tex2ClampT, Tex2Format,
    Tex2PalEntries, Tex2Combine, Tex2ColorFactor, Tex2ColorInput,
    Tex2CombineAlpha, Tex2AlphaFactor, Tex2AlphaInput,
    Count
};
extern u32 gR128SettingSeen[static_cast<size_t>(R128Setting::Count)];
inline void r128SettingUsed(R128Setting s, u32 value)
{
    gR128SettingSeen[static_cast<size_t>(s)] |= 1u << (value & 31u);
}
void r128SettingsReset();
std::string r128SettingsReport();

// --- Who wrote this pixel ---------------------------------------------------
//
// A wrong pixel has three possible authors — the processor storing through
// the frame buffer aperture, the 2D engine, the 3D rasterizer — and from the
// picture they are indistinguishable: a sprite with a black box behind it
// looks the same whether the game's own blitter, a color-keyed engine blit
// or a textured quad drew the box. This names the author. Every store path
// into video memory asks r128PixelWatched() and, for the watched bytes, logs
// who it is and what it was doing: the storing pc, the blit's operation and
// color-compare setting, the triangle's state.
//
// Off, it is one load and one branch per store. The first `budget` writes
// are kept in order and the rest are counted per author. File-statics, like
// every instrument on this card: a member would move sizeof(R128Cell).
extern u32 gR128PwLo, gR128PwHi; // [lo, hi) in video-memory bytes; hi 0 = off
void r128SetPixelWatch(u32 vramOff, u32 bytes, u32 budget);
// Watch the pixel at (x, y) of the picture the card is scanning out now.
// False if the display is not in a mode this can address.
bool r128SetPixelWatchXY(const R128Cell& c, u32 x, u32 y, u32 budget);
inline bool r128PixelWatched(u64 at, u64 n)
{
    return gR128PwHi && at < gR128PwHi && at + n > gR128PwLo;
}
#if defined(__GNUC__)
__attribute__((format(printf, 2, 3)))
#endif
void r128PixelWatchNote(const char* who, const char* fmt, ...);
// What was logged, as text: the kept lines, then the totals per author.
std::string r128PixelWatchReport();

// --- The engine command stream, in order ---------------------------------
//
// The register census says WHICH engine registers the guest touches, and the
// unbacked-read census says which of those this model never answers. Neither
// can say what a BLIT LOOKS LIKE — and a blit is a dozen writes whose meaning
// is their order and their values, with the trigger being the last one. The
// existing `log` cannot serve: it is a 4096-entry ring shared with the card's
// own FCode bring-up, and it deduplicates reads.
//
// So capture the engine half of the card on its own (0x0700–0x1FFF: the
// CCE/PM4 block, video, bus mastering, the surface apertures and the whole
// GUI block), in order, with values — head plus a ring tail, and the number
// that fell between them REPORTED rather than dropped in silence. A truncated
// sample is not evidence of absence; this project has paid for that twice.
//
// File-statics behind free functions, deliberately: adding a member to
// R128Cell changes sizeof(SawtoothBus), which kills every snapshot.
struct R128EngEv {
    u64 at;
    u32 off, val, pc;
    bool wr;
};
void r128SetEngineLog(size_t maxEntries);
// The capture, already stitched: the head, then the ring tail oldest-first.
const std::vector<R128EngEv>& r128EngineLog();
u64 r128EngineLogDropped();
// A register's name from ATI's own reference guide where the OEM edition
// documents it, and from QEMU's map where that edition redacts the CCE block.
// nullptr where neither names it — an offset with no name is a finding, not a
// gap to be filled with a guess.
const char* r128RegName(u32 off);

// --- What the 2D engine did, and what it refused to do -------------------
//
// A blitter that quietly declines is worse than one that is absent: the
// screen is wrong and nothing says why. Every path that ends without drawing
// increments one of these, so an unimplemented raster op or an unreachable
// destination is a NUMBER IN THE REPORT rather than a rendering mystery.
struct R128EngStats {
    u64 blits = 0;       // operations executed
    u64 pixels = 0;      // destination pixels written
    u64 fills = 0, copies = 0;
    u64 ropUnimpl = 0;   // (all 256 ROP3s are evaluated; kept for the report)
    u64 brushUnimpl = 0; // an op that reads a pattern this engine has no data for
    u64 monoSrc = 0;     // a 1-bit source expanded through frgd/bkgd — not built
    u64 badBpp = 0;      // a destination datatype it cannot size
    u64 zeroPitch = 0;   // a pitch of zero — nothing addressable
    u64 offVram = 0;     // a rectangle that would run off the end of VRAM
    u64 agpTarget = 0;   // an offset in the upper 32 MB: AGP, not VRAM
    u64 clippedOut = 0;  // entirely outside the scissor
    u64 lines2d = 0;     // Bresenham lines drawn by the 2D engine
    u64 linePixels = 0;  // ...and the pixels they wrote
    u64 lineDeclined = 0; // a line asked for with a brush or ROP not drawn
    u64 hostData = 0;    // source is HOST_DATA (stage 3, not built yet)
    u64 hostShape = 0;   // color host data with a word count of no known layout
    u64 colorCompare = 0; // a color compare this engine did not apply
    u64 waitUntil = 0;   // WAIT_UNTIL events, all satisfiable at once here
    u64 cacheFlushes = 0; // PC_GUI_CTLSTAT flush/invalidate pulses answered
    u64 bmFetches = 0;    // bus-master fetches into the GUI scratch registers
    u64 bmBadAddr = 0;    // ...whose address was not in system memory
    // The CCE's PIO packet path (PM4_FIFO_DATA_EVEN/ODD). Measured
    // 2026-08-01: with a real AGP bridge the ARM configures BUFFER_CNTL
    // mode 7 and submits through this FIFO — the no-ring finding of
    // session 32 was the AGP-less machine's path, not the machine's.
    u64 cceWords = 0;     // DWORDs accepted into the PIO FIFO
    u64 ccePkt0 = 0;      // Type-0 packets executed (register bursts)
    u64 ccePkt1 = 0;      // Type-1 packets executed (two registers)
    u64 ccePkt2 = 0;      // Type-2 fillers skipped
    u64 ccePkt3 = 0;      // Type-3 packets SKIPPED — opcodes go in the map
    u64 cceBadWidth = 0;  // a FIFO write that was not a 32-bit store
    u64 cceIndFetch = 0;  // indirect-buffer dispatches
    u64 cceIndWords = 0;  // DWORDs fetched from indirect buffers
    u64 cceGartMiss = 0;  // an AGP address whose GART entry was invalid
    u64 cceIndNested = 0; // an indirect dispatch from inside an indirect body
    // The CCE ring (SDK §5.3): packets bus-mastered out of AGP space between
    // DL_RPTR and DL_WPTR, with the read pointer written back into system
    // memory at DL_RPTR_ADDR — the transport a 3D-era driver arms with
    // PM4_MICRO_CNTL FREERUN. The 2D queue measured on this machine is PIO
    // (mode 7); these stay zero until something turns the ring on.
    u64 cceRingKicks = 0; // WPTR doorbells that found fetchable work
    u64 cceRingWords = 0; // DWORDs fetched out of the ring
    u64 cceRingStall = 0; // a ring fetch that missed the GART — kick abandoned
    // The 3D pipeline (SDK ch 6 / App F.24-F.27).
    u64 tris = 0;         // triangles rasterized
    u64 lines3d = 0;      // 3D lines drawn
    u64 points3d = 0;     // 3D points drawn
    u64 triPixels = 0;    // pixels the 3D pipe wrote
    u64 triCulled = 0;    // triangles removed by the winding cull
    u64 triDegen = 0;     // zero-area or fully-clipped triangles
    u64 vtxFetched = 0;   // vertices decoded, all walk modes
    u64 vtxGartMiss = 0;  // vertex-walker fetches that missed the GART
    u64 prim3dDecline = 0; // a 3D primitive packet refused whole
    u64 gated3d = 0;      // 3D register writes dropped: SCALE_3D_FN not 2
    u64 texSamples = 0;   // texel fetches, both units
    u64 texUnimpl = 0;    // texture datatype this engine cannot decode
    u64 texGartMiss = 0;  // an AGP texture fetch that missed the GART
    u64 zTile = 0;        // Z_PITCH_C had Z_TILE set — layout not modeled
    // The S/T range the guest actually sends, accumulated at vertex decode.
    // ⭐ SETTLED (--tri-dump, 2026-08-02): stream S,T are u·rhw / v·rhw —
    // premultiplied numerators, RAVE's own uOverW contract — which is why
    // this census's historical max S (126.7) equals the stream's max rhw
    // to three decimals. RAW stream values on purpose: this census records
    // what the guest SENT, not what the sampler resolved.
    float stMinS = 1e30f, stMaxS = -1e30f;
    float stMinT = 1e30f, stMaxT = -1e30f;
    // ⭐ THE COORDINATE CONVENTION, SETTLED PER TRIANGLE.
    //
    // The extrema above CANNOT decide it: global maxima of ~127 fit
    // normalized-with-tiling and texel-space over a 128-map equally well,
    // and reading one story out of that ambiguity shipped a regression.
    // What discriminates is the span of ONE triangle against the size of
    // the texture bound to it — a triangle showing a whole texture spans
    // 1.0 normalized or `w` in texel space, and those differ by 128x.
    // Bucketed by log2 so no assumption about the answer is baked in.
    u64 triSpanBucket[12] = {}; // span < 2^-4, 2^-3 ... >= 2^6
    u64 triSpanTexW = 0;        // sum of bound widths over counted triangles
    u64 triSpanN = 0;
};


struct SnoopSink; // opm/bus.hpp — only a pointer is needed here

// 🚚 THE BUS-MASTER FETCH — the card reading system memory on its own.
//
// Writing a physical address to 0x0A50 makes the card bus-master EIGHT BYTES
// from there into GUI_SCRATCH_REG0/REG1 (little-endian: +0 into REG0, +4 into
// REG1). ⚠ This is DERIVED FROM OBSERVED BEHAVIOR, not from a datasheet:
// neither ATI register guide documents the 0x0A00 block beyond two names, and
// no open-source driver touches 0x0A50 at all. What pins it down is Mac OS's
// ATI Graphics Accelerator, which uses exactly this as its bus-master
// self-test — it plants an UpTime timestamp in memory, writes the address
// here, waits for 0x0A10 bit 27 to clear, and compares the scratch registers
// against what it planted. A card that cannot do this is recorded as having
// no working bus mastering, and the accelerator then never enables
// acceleration at all.
//
// Lives here rather than on the bus because the counters are file-statics in
// r128.cpp — a counter added to SawtoothBus changes its size and kills every
// snapshot.
void r128BusMasterFetch(R128Cell& c, const u8* ram, u32 ramSize,
                        SnoopSink* snoop);
// 📦 THE CCE's PIO PACKET PATH — a guest write to PM4_FIFO_DATA_EVEN/ODD
// (0x1000/0x1004) hands the command engine one DWORD of packet stream.
// SDK-G04000 App F gives the four packet types; Type-0 is a register-write
// burst, which is why the stage-1 register engine is the executor. A Type-0
// to PM4_IW_INDOFF/INDSIZE dispatches an INDIRECT BUFFER: the card fetches
// the body itself from AGP space through the Uni-N GART (whose table the
// .AGP driver builds in system RAM). `raw`/`len` are the guest store as the
// bus saw it; `gartBase` is the bridge's GART base config register
// (106b:0020 +0x8C) at the moment of the write.
//
// ⚠ The staged FIFO words live in a file-static — a snapshot taken between
// the words of one packet resumes with the tail lost. Snapshots are minted
// at quiescent points, and the stat line makes a truncation visible.
void r128CceFifoWord(R128Cell& c, u32 raw, u32 len, const u8* ram,
                     u32 ramSize, u32 gartBase, u32 aperBase,
                     SnoopSink* snoop);
// Direct dispatch for a guest that writes PM4_IW_INDSIZE itself rather than
// through a packet (SDK §5.3.3 names that flow too).
void r128CceIndirect(R128Cell& c, u32 sizeDwords, const u8* ram, u32 ramSize,
                     u32 gartBase, u32 aperBase, SnoopSink* snoop);
// Diagnostics: how many packet-stream words sit STAGED (accepted but not
// yet consumed by a complete packet), and the word at the parse head —
// a large backlog behind one absurd header is what a desynchronized
// stream looks like from outside.
size_t r128CceStaged(u32& headWord);
// Type-3 opcodes that arrived and were skipped, by opcode byte — the same
// contract as the ROP map: the next thing to implement is a number in the
// report, not a guess.
const std::map<u32, u64>& r128CceP3Skipped();
// ⚠ MUST be called when a machine is CREATED. The parser's staging FIFO is
// process-global (a file-static, to keep sizeof(R128Cell) and every
// snapshot untouched), and the shell creates a fresh machine on every
// Stop/Start of the SAME process. A previous machine that died mid-stream
// leaves a half-consumed packet staged; the new machine's first FIFO words
// are then parsed as that stale header's body and the whole stream is
// desynchronized — fences land only where the misparse happens to hit the
// scratch registers, which presents as "the desktop drew for a while and
// then froze mid-redraw". Found from a user diag capture after a
// Stop/Start.
void r128CceReset();

// --- The 3D engine (r128_3d.cpp) ------------------------------------------
//
// The memory routes CCE work needs beyond the packet stream itself: the
// vertex walker and AGP-resident textures both read system RAM through the
// Uni-N GART, and every card-initiated access is snooped. Stack-passed,
// never stored — the cell stays a leaf device and no snapshot moves.
struct CceMem {
    const u8* ram = nullptr;
    u32 ramSize = 0, gartBase = 0, aperBase = 0;
    SnoopSink* snoop = nullptr;
    // The engine's own thread is executing (see "The command engine on its
    // own thread" below): memory is read directly, as it stood when the
    // stream was handed over, and the register lock is held and may be let
    // go for the length of a draw.
    bool worker = false;
};
// One GART-translated LE32 read out of AGP space (the parser's own walk,
// exported for the 3D unit's vertex and texture fetches).
bool r128CceGartRead(const CceMem& m, u32 agp, u32& out);
// The 3D Type-3 packets: 0x25 3D_RNDR_GEN_PRIM (inline FTLVERTEX stream),
// 0x23 3D_RNDR_GEN_INDX_PRIM (vertex walker over an AGP-space buffer),
// 0x2E NEXT_VERTEX_BUNDLE (continuation of the last 0x23), 0x2C
// LOAD_PALETTE. Returns false — a counted decline — on a body it cannot
// honor; the stream is bounded by the Type-3 header either way.
bool r128Cce3dOp(R128Cell& c, u32 op, const u32* body, u32 n,
                 const CceMem& m);
// Register writes the 3D unit owns: the fog table pair at 0x1A14/0x1A18 and
// the SCALE_3D_FN write gate over the 3D context block (0x1C90-0x1D44).
// Returns true when the write is fully handled (stored or deliberately
// dropped); the caller must not also store it.
bool r128Eng3dWrite(R128Cell& c, u32 off, u32 v);
// Machine creation: forget the NEXT_VERTEX_BUNDLE continuation latch, the
// CCE palettes and the fog table — same contract and caller as
// r128CceReset, which invokes this itself.
void r128Cce3dReset();
// The SCALE_3D_FN write gate's live position: -1 no guest has steered it
// (writes pass), 0 CLOSED (context writes are being dropped), 1 open. A
// closed gate means the pipeline is drawing with STALE state, which looks
// like a rendering bug and is invisible without this.
int r128Gate3dState();
// The triangle dump (--tri-dump N): print the next N textured triangles as
// they reach the rasterizer — raw vertex floats (x y z rhw s t s2 t2) plus
// the live unit-0 state and the packet's VC_FORMAT — so coordinate-
// convention questions are settled from the stream itself, offline, rather
// than from a picture. File-static budget; snapshots unaffected.
void r128Set3dTriDump(int n);
// The CCE ring: a PM4_BUFFER_DL_WPTR or PM4_MICRO_CNTL write may hand the
// engine a span of ring DWORDs to fetch through the GART and execute
// (SDK §5.3). Runs only in the bus-mastered-packet modes with the
// microengine free-running; afterwards the read pointer is written back to
// system memory at PM4_BUFFER_DL_RPTR_ADDR, snooped. Routed from the bus
// like the FIFO path, because only the bus owns RAM and the GART base.
void r128CceRingKick(R128Cell& c, u8* ram, u32 ramSize, u32 gartBase,
                     u32 aperBase, SnoopSink* snoop);
const R128EngStats& r128EngStats();

// --- The command engine on its own thread ---------------------------------
//
// A packet stream executes where it is submitted: inside the guest's store to
// the FIFO. For a 3D title that is a frame's triangles rasterized in software
// while the emulated processor stands still — measured in one, 37% of the host
// time, in stretches of up to 10 ms during which no guest instruction runs and
// no interrupt is taken. The real card draws while the processor carries on.
//
// With this on, an indirect buffer's packets are executed by a second thread.
// What the guest can observe is unchanged, because every place it could look
// waits for the engine first:
//
//   - a read or write of the engine's own registers (the PM4 block, the bus
//     master block, the surface ranges, everything from 0x1000 up: the 2D
//     and 3D state) and any access to video memory DRAIN the queue: whatever
//     is still queued is executed there and then, by the calling thread,
//     before the access;
//   - the registers a driver ASKS the engine through are answered as the
//     engine stands: PM4_STAT and GUI_STAT say busy and how much room the
//     command FIFO has, and the scratch registers hold the last fence the
//     stream has reached. Those are the card's own answers to "are you
//     done" and "may I send more", and a driver that polls them is doing
//     what it does on the real card — where forcing the queue to finish at
//     every status read would stop the processor once per submission;
//   - the other registers (the CRTC, the DAC, the cursor, the interrupt pair
//     the vertical-blank handler uses) have nothing to do with the engine and
//     only take the register lock, which the engine holds for a packet's
//     register work and lets go while it rasterizes;
//   - FIFO words written while the engine is running queue up behind it.
//
// System memory is the one thing the second thread cannot simply read: a
// card-initiated read is snooped against the processor's caches, and those
// belong to the processor's thread. So at every hand-off the machine's thread
// copies the modified cache lines out to memory (SnoopSink::publish), and the
// engine's thread reads memory directly, as it stood then. That covers what a
// stream refers to — textures in AGP space, a vertex buffer, the GART — since
// the guest wrote those before it submitted the stream. What it does not
// cover is anything submitted LATER: FIFO words that arrive behind a running
// engine, and the indirect buffer they dispatch. The engine stops when it
// reaches those, and the machine's thread takes them at its next look at the
// devices (r128AsyncService), executes what is short, and hands the rest
// back with memory published again.
//
// Off by default, and left off wherever a run has to be reproducible: when
// the work happens is the one thing this changes, and under host pacing that
// is already not reproducible. All state is file-static in r128.cpp, like
// the rest of the engine: no snapshot moves.
//
// `strict` is the proof mode: the status words and the scratch registers
// finish the queue like every other engine register, so NOTHING the guest
// can read depends on how far the engine's thread has got. A strict run is
// then the synchronous machine instruction for instruction — same registers,
// same video memory, same fingerprint at any count — which is how the
// threading itself is checked (g4run --ati-thread-strict against
// --no-ati-thread).
void r128AsyncEnable(bool on, bool strict = false);
bool r128AsyncEnabled();
bool r128AsyncStrict();
extern std::atomic<bool> gR128AsyncBusy;     // packets queued or executing
extern std::atomic<bool> gR128AsyncNeedsEmu; // stopped at a packet that needs
                                             // the machine's thread
inline bool r128AsyncBusy()
{
    return gR128AsyncBusy.load(std::memory_order_acquire);
}
inline bool r128AsyncNeedsService()
{
    return gR128AsyncNeedsEmu.load(std::memory_order_relaxed);
}
// Execute everything still queued, on the calling thread. On return the
// engine is idle and the caller owns the card outright until it next submits.
void r128AsyncDrain(R128Cell& c, const CceMem& m);
// Keep the engine's thread from running what it is handed until this is
// lifted. For the tests, which need "a stream is queued and none of it has
// executed" to be a state they can stand in rather than a race they might
// win.
void r128AsyncHold(bool hold);
// FIFO words the guest has written that the engine has not taken yet: what a
// running engine's command FIFO holds.
u32 r128AsyncFifoQueued();
// Why the next drain is happening, for the census below: a register offset,
// or one of these.
constexpr u32 kR128DrainFbRead = 0xFFFF0001u, kR128DrainFbWrite = 0xFFFF0002u,
              kR128DrainOther = 0xFFFF0000u;
void r128AsyncDrainCause(u32 cause);
struct R128DrainCause {
    u64 n = 0, ns = 0, pkts = 0;
};
// What the guest looked at when it had to wait, by cause. A wait is the
// engine's thread not helping, so the busiest line here is the next thing to
// look at.
const std::map<u32, R128DrainCause>& r128AsyncDrainCauses();
// Take the stream on from where the engine stopped (words submitted after
// the hand-off), then hand the rest back.
void r128AsyncService(R128Cell& c, const CceMem& m);
// The register lock, for an access that does not need the queue finished.
// Takes nothing when the engine is idle; nests.
struct R128RegLock {
    R128RegLock();
    ~R128RegLock();
    R128RegLock(const R128RegLock&) = delete;
    R128RegLock& operator=(const R128RegLock&) = delete;

  private:
    bool held_ = false;
};
// For the 3D unit, on the engine's thread only: let the register lock go for
// the length of a draw, which touches video memory and nothing else.
void r128AsyncRegRelease();
void r128AsyncRegTake();
struct R128AsyncStats {
    u64 handOffs = 0;   // times a stream was handed to the engine's thread
    u64 workerPkts = 0; // packets that thread executed
    u64 inlinePkts = 0; // packets executed where they were submitted
    u64 drains = 0;     // times the guest looked and work was still queued
    u64 drainNs = 0;    // ...and how long finishing it took
    u64 drainPkts = 0;  // ...and how many packets the looking thread ran
    u64 services = 0;   // times the engine stopped for the machine's thread
    u64 inboxWords = 0; // FIFO words queued behind a running engine
};
const R128AsyncStats& r128AsyncStats();
// --no-ati-2d: put the card back the way it was before the engine existed,
// so a boot that never touches the GUI block can be shown to be unchanged.
void r128SetEngine2d(bool on);
// The raster ops that were refused, busiest first, so "which op do I have to
// implement next" is answered by the instrument and not by guesswork.
const std::map<u32, u64>& r128RopUnimplemented();

} // namespace opm
