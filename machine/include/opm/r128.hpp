#pragma once
#include "opm/types.hpp"

#include <map>
#include <utility>
#include <vector>

namespace opm {

struct SnapWriter;
struct SnapReader;

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
    u32 regBase = 0; // CONFIG_REG_1_BASE: where its own registers live
    u32 fbLo = 0xFFFFFFFFu, fbHi = 0;

    R128Cell() : vram(32u << 20, 0) {}

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
    // registers are NORMALISED into their canonical offsets as they are
    // written, and a blit reads its parameters back out of the same map. The
    // engine adds no state of its own at all; it only adds behaviour.
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
    // Absorb a write into the engine block: normalise aliases, apply the
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
    u64 hostData = 0;    // source is HOST_DATA (stage 3, not built yet)
    u64 colorCompare = 0; // a colour-key function this engine ignores
    u64 waitUntil = 0;   // WAIT_UNTIL events, all satisfiable at once here
    u64 cacheFlushes = 0; // PC_GUI_CTLSTAT flush/invalidate pulses answered
};
const R128EngStats& r128EngStats();
// --no-ati-2d: put the card back the way it was before the engine existed,
// so a boot that never touches the GUI block can be shown to be unchanged.
void r128SetEngine2d(bool on);
// The raster ops that were refused, busiest first, so "which op do I have to
// implement next" is answered by the instrument and not by guesswork.
const std::map<u32, u64>& r128RopUnimplemented();

} // namespace opm
