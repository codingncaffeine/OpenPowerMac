#pragma once
// Arc 2 M3: the onboard ATI 3D Rage Pro ("whisper" personality) at the
// mach64-GT register level — enough chip for the ROM's video driver to find
// a live device, program a mode, and draw first light. The rest of the chip
// (draw engine, overlay, accurate timing) lands in its own arc; until then
// unknown registers are honest storage, and every register touch is logged
// with its first-seen time so the boot tells us what it needs next.
//
// Empirical pins from the boot ROM (macrun stub log, 2026-07-26):
//   - the driver's I/O-space register map IS the memory-mapped Block 0
//     layout (identity): its pre-BAR writes decode as CRTC_INT_CNTL=0,
//     CRTC_GEN_CNTL=0x240 (8 bpp), CONFIG_STAT0=0x1D (SGRAM) — three
//     coherent decodes;
//   - registers are little-endian dwords driven with byte-reversed
//     loads/stores: memory lane k carries value bits 8k+7..8k;
//   - the driver probes aperture+0x7FF800 at the personality address
//     (0x8FFFF800): the mach64 8 MB aperture with the register file in its
//     top 2 KB. RECEIPT: Block 0 is served across BOTH 1 KB halves of that
//     window until traffic pins the split (ATI's own layout puts Block 0 in
//     the last 1 KB, Block 1 below it; the ROM probed the first dword of
//     the lower half).
//
// The beam is a deterministic counter: one scan line per kTicksPerLine
// machine ticks, VBLANK per the programmed CRTC geometry (525-line fallback
// frame until the CRTC is programmed). No wall clock anywhere.

#include "opm/types.hpp"

#include <map>
#include <vector>

namespace opm {

class AtiRage {
public:
    static constexpr u32 kAperture = 0x00800000u;      // 8 MB memory aperture
    static constexpr u32 kRegTop = kAperture - 0x800u; // register file start
    static constexpr u32 kTicksPerLine = 64;

    AtiRage();

    // Memory-aperture access: VRAM below the top 2 KB, registers inside it.
    u8 apRead8(u32 off);
    void apWrite8(u32 off, u8 v);
    // I/O-space window (256 bytes): Block 0 identity map.
    u8 ioRead8(u32 off) { return regRead8(off & 0xFFu, true); }
    void ioWrite8(u32 off, u8 v) { regWrite8(off & 0xFFu, v, true); }

    void tick(); // advance the beam one machine tick

    const std::vector<u8>& vram() const { return vram_; }

    // CRTC readout for the framebuffer dump.
    struct Mode {
        u32 width = 0, height = 0, bpp = 0;
        u32 pitchPixels = 0, offsetBytes = 0;
        bool enabled = false;
    };
    Mode mode() const;
    void palette(u32 i, u8& r, u8& g, u8& b) const;

    struct Touch {
        u64 reads = 0, writes = 0;
        u64 firstAt = 0;   // tick count at first touch
        u32 lastWrite = 0; // register dword picture after the last write
    };
    // Keyed by Block 0 dword offset; separate books per access path.
    const std::map<u32, Touch>& ioLog() const { return ioLog_; }
    const std::map<u32, Touch>& memLog() const { return memLog_; }

    // Chronological GP_IO conversation (monitor sense / DDC debugging):
    // every byte access, tagged with its lane; reads carry the composed
    // dword the CPU saw. First ops kept verbatim plus a ring of the last
    // ops so both the opening handshake and the steady state are visible.
    struct GpioOp {
        u64 at;
        u8 write, lane;
        u32 val;
        u32 pc, lr; // issuing instruction + its caller (via pcRef/lrRef)
    };
    const u32* pcRef = nullptr; // debug wiring: runner's live PC, optional
    const u32* lrRef = nullptr; // debug wiring: runner's live LR, optional
    static constexpr size_t kGpioKeep = 1024;
    const std::vector<GpioOp>& gpioHead() const { return gpioHead_; }
    std::vector<GpioOp> gpioTail() const; // ring, unrolled oldest-first
    u64 gpioOps() const { return gpioOps_; }

private:
    // Block 0 register file, one LE dword per slot (0x000..0x3FF).
    static constexpr u32 kRegs = 0x400u / 4u;

    u8 regRead8(u32 off, bool io);
    void regWrite8(u32 off, u8 v, bool io);
    u32 regValue(u32 regOff) const; // dword picture incl. live fields

    u32 vTotalLines() const; // full frame height in lines
    u32 vDispLines() const;  // displayed lines
    bool inVblank() const { return line_ >= vDispLines(); }

    void logTouch(std::map<u32, Touch>& log, u32 regOff, bool write);

    std::vector<u8> vram_;
    u32 regs_[kRegs] = {};

    // PLL register file behind CLOCK_CNTL's address/data window.
    u8 pll_[64] = {};
    u32 pllAddr_ = 0;
    bool pllWrEn_ = false;

    // RAMDAC palette behind DAC_REGS.
    u8 pal_[256][3] = {};
    u8 palWIndex_ = 0, palRIndex_ = 0, palPhase_ = 0;
    u8 dacMask_ = 0xFF;

    // Beam state.
    u32 line_ = 0, tickInLine_ = 0;
    bool frameOdd_ = false;
    u32 sticky_ = 0; // latched CRTC_INT_CNTL interrupt bits (0x4 | 0x10)

    void gpioRecord(bool write, u32 lane, u32 val);

    // DDC (I2C) slave on the monitor sense lines: SDA = GP_IO bit 13,
    // SCL = bit 12 (pinned empirically from the ROM FCode driver's
    // clocking pattern). Serves the EDID at I2C address 0x50.
    bool sclWire() const;
    bool sdaWire() const;
    void ddcUpdate();
    void ddcOnStart();
    void ddcOnStop();
    void ddcOnSclRise();
    void ddcOnSclFall();

    enum class Ddc : u8 { Idle, Bits, Ack, ReadBits, ReadAck };
    Ddc ddc_ = Ddc::Idle;
    u8 ddcShift_ = 0, ddcBits_ = 0, ddcPtr_ = 0;
    bool ddcRead_ = false, ddcAddrPhase_ = false;
    bool sdaPull_ = false; // slave holding SDA low
    bool prevScl_ = true, prevSda_ = true;
    u8 edid_[128] = {};

    u64 ticks_ = 0;
    std::map<u32, Touch> ioLog_, memLog_;
    std::vector<GpioOp> gpioHead_, gpioRing_;
    size_t gpioRingAt_ = 0;
    u64 gpioOps_ = 0;
};

} // namespace opm
