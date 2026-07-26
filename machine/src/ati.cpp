#include "opm/ati.hpp"

namespace opm {

// Block 0 register offsets (mach64 memory map; the I/O window is identical).
namespace ati {
inline constexpr u32 CRTC_H_TOTAL_DISP = 0x00;
inline constexpr u32 CRTC_V_TOTAL_DISP = 0x08;
inline constexpr u32 CRTC_VLINE_CRNT_VLINE = 0x10;
inline constexpr u32 CRTC_OFF_PITCH = 0x14;
inline constexpr u32 CRTC_INT_CNTL = 0x18;
inline constexpr u32 CRTC_GEN_CNTL = 0x1C;
inline constexpr u32 GP_IO = 0x78;
inline constexpr u32 CLOCK_CNTL = 0x90;
inline constexpr u32 MEM_CNTL = 0xB0;
inline constexpr u32 DAC_REGS = 0xC0;
inline constexpr u32 CONFIG_CHIP_ID = 0xE0;

// CRTC_INT_CNTL bit roles.
inline constexpr u32 kIntLiveVblank = 0x00000001; // read-only live status
inline constexpr u32 kIntVblankInt = 0x00000004;  // latched, write-1-to-ack
inline constexpr u32 kIntVlineInt = 0x00000010;   // latched, write-1-to-ack
inline constexpr u32 kIntLiveFrame = 0x00000040;  // odd/even frame flag
inline constexpr u32 kIntLiveMask = kIntLiveVblank | kIntVblankInt |
                                    kIntVlineInt | kIntLiveFrame;
} // namespace ati

AtiRage::AtiRage() : vram_(kAperture, 0)
{
    // RECEIPTS (deterministic cold-chip choices, logged for the wiki):
    //   MEM_CNTL[2:0] = 5 -> 8 MB of memory (mach64 size code), matching the
    //     8 MB aperture so the driver's view is self-consistent;
    //   CONFIG_CHIP_ID = 0x5C004750: 3D Rage Pro 'GP' with the PCI revision
    //     byte the config header advertises (served read-only);
    //   PLL file and palette power up as zeros (undefined on silicon).
    regs_[ati::MEM_CNTL / 4] = 0x00000005u;
    regs_[ati::CONFIG_CHIP_ID / 4] = 0x5C004750u;

    // The attached display's EDID (RECEIPT: authored here — a 640x480@67
    // analog monitor, manufacturer 'APP', preferred detailed timing at a
    // 30.24 MHz pixel clock; checksum computed below). Served over DDC.
    static const u8 kEdid[127] = {
        0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, // header
        0x06, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, // APP, product 1
        0x01, 0x07,                                     // week 1, 1997
        0x01, 0x01,                                     // EDID 1.1
        0x08,       // analog, separate syncs
        0x19, 0x13, // 25 cm x 19 cm
        0x78,       // gamma 2.2
        0x0A,       // RGB color, preferred timing
        0xEE, 0x91, 0xA3, 0x54, 0x4C, 0x99, 0x26, 0x0F, // chromaticity
        0x50, 0x54,
        0x30, 0x00, 0x00, // established: 640x480@60 + @67
        0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, // std timings unused
        0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
        // detailed timing: 640x480@67, pclk 30.24 MHz, blank 224/45
        0xD0, 0x0B, 0x80, 0xE0, 0x20, 0xE0, 0x2D, 0x10,
        0x40, 0x40, 0x33, 0x00, 0x04, 0xC3, 0x10, 0x00, 0x00, 0x00,
        // monitor range limits: V 59-61 Hz, H 30-40 kHz, pclk <= 100 MHz
        0x00, 0x00, 0x00, 0xFD, 0x00, 0x3B, 0x3D, 0x1E, 0x28, 0x0A,
        0x00, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
        // monitor name
        0x00, 0x00, 0x00, 0xFC, 0x00, 'O', 'P', 'M', ' ', 'D', 'i', 's',
        'p', 'l', 'a', 'y', 0x0A, 0x20,
        // dummy descriptor
        0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, // no extensions
    };
    static_assert(sizeof(kEdid) == 127, "EDID body must leave checksum room");
    u32 sum = 0;
    for (u32 i = 0; i < 127; ++i) {
        edid_[i] = kEdid[i];
        sum += kEdid[i];
    }
    edid_[127] = static_cast<u8>(0x100u - (sum & 0xFFu)); // block checksum
}

// ---- DDC: an I2C EDID slave bit-banged through GP_IO ----
//
// Wire levels: a line reads its host-driven value when its output enable is
// set, otherwise it floats to the pull-up unless the slave holds SDA low.

bool AtiRage::sclWire() const
{
    const u32 g = regs_[ati::GP_IO / 4];
    return (g & (1u << 28)) ? (g & (1u << 12)) != 0 : true;
}

bool AtiRage::sdaWire() const
{
    const u32 g = regs_[ati::GP_IO / 4];
    if (g & (1u << 29))
        return (g & (1u << 13)) != 0;
    return !sdaPull_;
}

void AtiRage::ddcOnStart()
{
    ddc_ = Ddc::Bits;
    ddcShift_ = 0;
    ddcBits_ = 0;
    ddcAddrPhase_ = true;
    sdaPull_ = false;
}

void AtiRage::ddcOnStop()
{
    ddc_ = Ddc::Idle;
    sdaPull_ = false;
}

void AtiRage::ddcOnSclRise()
{
    switch (ddc_) {
    case Ddc::Bits:
        ddcShift_ = static_cast<u8>((ddcShift_ << 1) | (sdaWire() ? 1 : 0));
        ++ddcBits_;
        break;
    case Ddc::ReadAck:
        if (sdaWire()) {
            ddc_ = Ddc::Idle; // master NACK: transfer over, await STOP
        } else {
            ddcShift_ = edid_[ddcPtr_++ & 127u]; // ACK: next byte follows
            ddcBits_ = 0;
            ddc_ = Ddc::ReadBits;
        }
        break;
    default:
        break;
    }
}

void AtiRage::ddcOnSclFall()
{
    switch (ddc_) {
    case Ddc::Bits:
        if (ddcBits_ == 8) {
            if (ddcAddrPhase_) {
                if ((ddcShift_ >> 1) == 0x50u) {
                    ddcRead_ = (ddcShift_ & 1u) != 0;
                    sdaPull_ = true; // address ACK
                    ddc_ = Ddc::Ack;
                } else {
                    ddc_ = Ddc::Idle; // not us: NACK by staying released
                }
            } else {
                ddcPtr_ = ddcShift_; // register pointer write
                sdaPull_ = true;
                ddc_ = Ddc::Ack;
            }
        }
        break;
    case Ddc::Ack:
        if (ddcRead_) {
            ddcShift_ = edid_[ddcPtr_++ & 127u];
            ddcBits_ = 0;
            ddc_ = Ddc::ReadBits;
            sdaPull_ = (ddcShift_ & 0x80u) == 0; // present bit 7
            ++ddcBits_;
        } else {
            ddcAddrPhase_ = false;
            ddcShift_ = 0;
            ddcBits_ = 0;
            ddc_ = Ddc::Bits;
            sdaPull_ = false;
        }
        break;
    case Ddc::ReadBits:
        if (ddcBits_ == 8) {
            sdaPull_ = false; // release for the master's ACK clock
            ddc_ = Ddc::ReadAck;
        } else {
            sdaPull_ = (ddcShift_ & (0x80u >> ddcBits_)) == 0;
            ++ddcBits_;
        }
        break;
    default:
        break;
    }
}

void AtiRage::ddcUpdate()
{
    const bool scl = sclWire();
    const bool sda = sdaWire();
    if (scl && prevScl_) {
        if (prevSda_ && !sda)
            ddcOnStart();
        else if (!prevSda_ && sda)
            ddcOnStop();
    } else if (scl && !prevScl_) {
        ddcOnSclRise();
    } else if (!scl && prevScl_) {
        ddcOnSclFall();
    }
    prevScl_ = scl;
    prevSda_ = sda;
}

u32 AtiRage::vTotalLines() const
{
    const u32 t = (regs_[ati::CRTC_V_TOTAL_DISP / 4] & 0x7FFu) + 1;
    return t > 1 ? t : 525; // fallback frame until the CRTC is programmed
}

u32 AtiRage::vDispLines() const
{
    const u32 d = ((regs_[ati::CRTC_V_TOTAL_DISP / 4] >> 16) & 0x7FFu) + 1;
    return d > 1 ? d : 480;
}

void AtiRage::tick()
{
    ++ticks_;
    if (++tickInLine_ < kTicksPerLine)
        return;
    tickInLine_ = 0;
    if (++line_ >= vTotalLines()) {
        line_ = 0;
        frameOdd_ = !frameOdd_;
    }
    if (line_ == vDispLines())
        sticky_ |= ati::kIntVblankInt; // vertical blank begins
    if (line_ == (regs_[ati::CRTC_VLINE_CRNT_VLINE / 4] & 0x7FFu))
        sticky_ |= ati::kIntVlineInt;
}

u32 AtiRage::regValue(u32 regOff) const
{
    const u32 stored = regs_[regOff / 4];
    switch (regOff) {
    case ati::CRTC_VLINE_CRNT_VLINE:
        return (stored & 0x000007FFu) | ((line_ & 0x7FFu) << 16);
    case ati::CRTC_INT_CNTL:
        return (stored & ~ati::kIntLiveMask) | sticky_ |
               (inVblank() ? ati::kIntLiveVblank : 0) |
               (frameOdd_ ? ati::kIntLiveFrame : 0);
    case ati::GP_IO: {
        // Apple monitor sense on the GPIO pins. Wiring per the mach64 Mac
        // convention: sense lines A/B/C on data bits 13/12/8, output
        // enables on bits 29/28/24. A pin reads its driven value when
        // enabled, the wire's level when not.
        //
        // RECEIPT: the attached display presents legacy sense code 6
        // (Apple Hi-Res 13" RGB: A/B float to the pull-ups, C grounded,
        // extended 0x2B) AND answers DDC on A=SDA / B=SCL with the EDID
        // authored in the constructor. The ROM FCode driver bit-bangs I2C
        // on exactly those two lines (pinned from its clocking trace).
        u32 v = stored;
        v = (v & ~(1u << 13)) | (sdaWire() ? (1u << 13) : 0);
        v = (v & ~(1u << 12)) | (sclWire() ? (1u << 12) : 0);
        if (!(stored & (1u << 24)))
            v &= ~(1u << 8); // C grounded by the monitor
        return v;
    }
    default:
        return stored;
    }
}

void AtiRage::logTouch(std::map<u32, Touch>& log, u32 regOff, bool write)
{
    Touch& t = log[regOff];
    if (t.reads + t.writes == 0)
        t.firstAt = ticks_;
    if (write) {
        ++t.writes;
        t.lastWrite = regs_[regOff / 4];
    } else {
        ++t.reads;
    }
}

u8 AtiRage::regRead8(u32 off, bool io)
{
    const u32 regOff = off & ~3u;
    const u32 lane = off & 3u;
    logTouch(io ? ioLog_ : memLog_, regOff, false);

    if (regOff == ati::GP_IO)
        gpioRecord(false, lane, regValue(regOff));

    // Byte-ported registers first: the DAC and the PLL data window.
    if (regOff == ati::DAC_REGS) {
        switch (lane) {
        case 0:
            return palWIndex_;
        case 1: {
            const u8 c = pal_[palRIndex_][palPhase_];
            if (++palPhase_ == 3) {
                palPhase_ = 0;
                ++palRIndex_;
            }
            return c;
        }
        case 2:
            return dacMask_;
        default:
            return palRIndex_;
        }
    }
    if (regOff == ati::CLOCK_CNTL && lane == 2)
        return pll_[pllAddr_]; // data window reads the addressed PLL register

    return static_cast<u8>(regValue(regOff) >> (8 * lane));
}

void AtiRage::regWrite8(u32 off, u8 v, bool io)
{
    const u32 regOff = off & ~3u;
    const u32 lane = off & 3u;

    if (regOff == ati::DAC_REGS) {
        switch (lane) {
        case 0:
            palWIndex_ = v;
            palPhase_ = 0;
            break;
        case 1:
            pal_[palWIndex_][palPhase_] = v;
            if (++palPhase_ == 3) {
                palPhase_ = 0;
                ++palWIndex_;
            }
            break;
        case 2:
            dacMask_ = v;
            break;
        default:
            palRIndex_ = v;
            palPhase_ = 0;
            break;
        }
        logTouch(io ? ioLog_ : memLog_, regOff, true);
        return;
    }

    if (regOff == ati::CLOCK_CNTL) {
        if (lane == 1) { // PLL address latch: bits 7:2 address, bit 1 wr-en
            pllAddr_ = (v >> 2) & 63u;
            pllWrEn_ = (v & 2u) != 0;
        } else if (lane == 2 && pllWrEn_) {
            pll_[pllAddr_] = v;
        }
        // fall through to lane storage: the latch bytes read back
    }

    if (regOff == ati::CONFIG_CHIP_ID) { // read-only identity
        logTouch(io ? ioLog_ : memLog_, regOff, true);
        return;
    }

    u32& reg = regs_[regOff / 4];
    const u32 shift = 8 * lane;
    reg = (reg & ~(0xFFu << shift)) | (u32(v) << shift);

    if (regOff == ati::CRTC_INT_CNTL && lane == 0)
        sticky_ &= ~(u32(v) & (ati::kIntVblankInt | ati::kIntVlineInt));

    if (regOff == ati::GP_IO) {
        gpioRecord(true, lane, reg);
        ddcUpdate(); // the write may have moved SDA/SCL
    }

    logTouch(io ? ioLog_ : memLog_, regOff, true);
}

void AtiRage::gpioRecord(bool write, u32 lane, u32 val)
{
    ++gpioOps_;
    const GpioOp op{ticks_, static_cast<u8>(write), static_cast<u8>(lane),
                    val, pcRef ? *pcRef : 0, lrRef ? *lrRef : 0};
    if (gpioHead_.size() < kGpioKeep) {
        gpioHead_.push_back(op);
        return;
    }
    if (gpioRing_.size() < kGpioKeep) {
        gpioRing_.push_back(op);
    } else {
        gpioRing_[gpioRingAt_] = op;
        gpioRingAt_ = (gpioRingAt_ + 1) % kGpioKeep;
    }
}

std::vector<AtiRage::GpioOp> AtiRage::gpioTail() const
{
    std::vector<GpioOp> out;
    out.reserve(gpioRing_.size());
    for (size_t i = 0; i < gpioRing_.size(); ++i)
        out.push_back(gpioRing_[(gpioRingAt_ + i) % gpioRing_.size()]);
    return out;
}

u8 AtiRage::apRead8(u32 off)
{
    if (off >= kRegTop)
        return regRead8((off - kRegTop) & 0x3FFu, false);
    return vram_[off];
}

void AtiRage::apWrite8(u32 off, u8 v)
{
    if (off >= kRegTop) {
        regWrite8((off - kRegTop) & 0x3FFu, v, false);
        return;
    }
    vram_[off] = v;
}

AtiRage::Mode AtiRage::mode() const
{
    Mode m;
    const u32 h = regs_[ati::CRTC_H_TOTAL_DISP / 4];
    const u32 v = regs_[ati::CRTC_V_TOTAL_DISP / 4];
    const u32 op = regs_[ati::CRTC_OFF_PITCH / 4];
    const u32 gen = regs_[ati::CRTC_GEN_CNTL / 4];
    m.width = (((h >> 16) & 0xFFu) + 1) * 8;
    m.height = ((v >> 16) & 0x7FFu) + 1;
    static const u32 bppOf[8] = {0, 4, 8, 15, 16, 24, 32, 0};
    m.bpp = bppOf[(gen >> 8) & 7u];
    m.pitchPixels = ((op >> 22) & 0x3FFu) * 8;
    m.offsetBytes = (op & 0xFFFFFu) * 8;
    m.enabled = (gen & 0x02000000u) != 0 && m.bpp != 0;
    return m;
}

void AtiRage::palette(u32 i, u8& r, u8& g, u8& b) const
{
    r = pal_[i & 0xFFu][0];
    g = pal_[i & 0xFFu][1];
    b = pal_[i & 0xFFu][2];
}

} // namespace opm
