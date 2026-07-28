#include "opm/r128.hpp"

namespace opm {

// Rage 128 register offsets the bring-up touches (register names per
// the chip's public lineage; values earn semantics as the FCode and
// the OS driver demand them).
static constexpr u32 kMmIndex = 0x0000;      // MM_INDEX / MM_DATA pair
static constexpr u32 kClockCntlIndex = 0x0008;
static constexpr u32 kClockCntlData = 0x000C;
static constexpr u32 kBiosScratch = 0x0010;  // BIOS_0_SCRATCH..
static constexpr u32 kGenReset = 0x00F0;     // GEN_RESET_CNTL
static constexpr u32 kConfigMemsize = 0x00F8;
static constexpr u32 kConfigAperSize = 0x0100;
static constexpr u32 kMemCntl = 0x0140;
static constexpr u32 kCrtcGenCntl = 0x0050;
static constexpr u32 kGpioMonid = 0x0068;    // DDC bit-bang
static constexpr u32 kGpioDvi = 0x006C;
// The I/O-aperture GPIO trio Open Firmware bit-bangs during monitor sense:
// it drives bits 22 and 23 through 0x00A0 and 0x00A8 and reads 0x00A4 back.
// Answering zero is a line held LOW, which on a two-wire bus is a stuck bus
// and not "no monitor" — undriven lines float high on their pull-ups.
static constexpr u32 kGpioA0 = 0x00A0;
static constexpr u32 kGpioA4 = 0x00A4;
static constexpr u32 kGpioA8 = 0x00A8;
// Which line is which, and which bit is the direction, are pinned by
// experiment: the address byte the slave latches is 0xFF until they are
// right, because a released SDA reads high for all eight bits.
static constexpr bool kLevelIsA0 = true; // else 0xA8 holds the levels
static constexpr u32 kSclBit = 0x00400000u;
static constexpr u32 kSdaBit = 0x00800000u;
static constexpr u32 kSenseIn = 0x00C00000u;
static constexpr u32 kPllTest = 0x0000;

u32 R128Cell::regRead(u32 idx)
{
    const u32 off = idx << 2;
    switch (off) {
    case kConfigMemsize:
    case kConfigAperSize: {
        // Power-on default 32 MB; if the init code programs the field,
        // read back what it wrote (verify loops depend on it).
        auto it = regs_.find(off);
        return it != regs_.end() ? it->second : (32u << 20);
    }
    case kClockCntlIndex:
        return pllAddr_;
    case kClockCntlData: {
        auto it = pll_.find(pllAddr_ & 0x3Fu);
        // PLL_TEST_CNTL-style status: report locked/ready bits set so
        // spin-until-lock loops fall through.
        return it != pll_.end() ? it->second : 0x00000000u;
    }
    case kGpioA0:
    case kGpioA4:
    case kGpioA8: {
        // Bit 22 is the direction, bit 23 the level, and the lines are open
        // drain: undriven reads high, and either side can pull down. 0x00A4
        // reads the bus back.
        const u32 lvl = kLevelIsA0 ? gpioSda_ : gpioScl_;
        const u32 en = kLevelIsA0 ? gpioScl_ : gpioSda_;
        const bool scl = !(en & kSclBit) || (lvl & kSclBit);
        const bool sda = (!(en & kSdaBit) || (lvl & kSdaBit)) && ddcSda();
        auto it = regs_.find(off);
        const u32 v = it != regs_.end() ? it->second : 0;
        return (v & 0xFF3FFFFFu) | (scl ? 0x00400000u : 0u) |
               (sda ? 0x00800000u : 0u);
    }
    case kGpioMonid:
    case kGpioDvi: {
        // DDC lines read back as driven; pulled-up (idle high) when
        // released. Bits 24-27 are the input reflections of 8-11.
        auto it = regs_.find(off);
        const u32 v = it != regs_.end() ? it->second : 0;
        u32 in = 0x0F000000u; // all lines high (no monitor yet)
        return (v & 0x00FFFFFFu) | in;
    }
    default: {
        auto it = regs_.find(off);
        return it != regs_.end() ? it->second : 0;
    }
    }
}

void R128Cell::note(u32 off, u32 val, bool wr)
{
    // The card's own FCode bring-up issues thousands of accesses around
    // 330M and fills the log, so by the time the OS driver touches a
    // register at 3.9G there is no room left and the accesses that matter
    // are invisible. logFrom opens the window late; the first-touch set is
    // cleared as it opens so the OS era gets its own first touches rather
    // than inheriting the firmware's.
    const u64 at = stamp ? *stamp : 0;
    if (at < logFrom)
        return;
    if (logFrom && !gateOpened_) {
        gateOpened_ = true;
        seen_.clear();
        log.clear();
    }
    if (!wr) {
        if (seen_.count(off))
            return;
        seen_[off] = 1;
    }
    if (log.size() < 4096)
        log.push_back({at, off, val, pcRef ? *pcRef : 0, wr});
}

u32 R128Cell::read(u32 off, u32 len)
{
    const u32 native = regRead(off >> 2);
    note(off & ~3u, native, false);
    if (len == 4)
        return swap32(native);
    u32 r = 0;
    for (u32 k = 0; k < len; ++k)
        r = (r << 8) | ((native >> (8 * ((off + k) & 3u))) & 0xFFu);
    return r;
}

void R128Cell::write(u32 off, u32 v, u32 len)
{
    u32 native;
    if (len == 4)
        native = swap32(v);
    else {
        native = regRead(off >> 2);
        for (u32 k = 0; k < len; ++k) {
            const u32 lane = (off + k) & 3u;
            native = (native & ~(0xFFu << (8 * lane))) |
                     (((v >> (8 * (len - 1 - k))) & 0xFFu) << (8 * lane));
        }
    }
    const u64 at = stamp ? *stamp : 0;
    note(off & ~3u, native, true);
    const u32 aligned = off & ~3u;
    if (aligned == kClockCntlIndex) {
        pllAddr_ = native & 0xFFu;
        // write-enable bit 7: data writes land in the PLL file
    } else if (aligned == kGpioA0 || aligned == kGpioA8) {
        // Every write to either line register can move the bus, so step the
        // DDC slave from both.
        if (aligned == kGpioA0)
            gpioSda_ = native; // holds levels when kLevelIsA0
        else
            gpioScl_ = native; // holds enables when kLevelIsA0
        regs_[aligned] = native;
        const u32 lvl2 = kLevelIsA0 ? gpioSda_ : gpioScl_;
        const u32 en2 = kLevelIsA0 ? gpioScl_ : gpioSda_;
        // Gated with the register log, and for the same reason: the FCode
        // fills 160 states long before the OS driver says anything.
        if (at >= logFrom && ddcWave.size() < 160)
            ddcWave.push_back({gpioSda_, gpioScl_});
        ddcStep(!(en2 & kSclBit) || (lvl2 & kSclBit),
                (!(en2 & kSdaBit) || (lvl2 & kSdaBit)) && ddcSda());
    } else if (aligned == kClockCntlData) {
        pll_[pllAddr_ & 0x3Fu] = native;
    } else if (aligned == 0x00B0u) { // PALETTE_INDEX
        palIdx_ = native & 0xFFu;
    } else if (aligned == 0x00B4u) { // PALETTE_DATA, auto-increment
        pal_[palIdx_ & 0xFFu] = native & 0x00FFFFFFu;
        palIdx_ = (palIdx_ + 1u) & 0xFFu;
    }
    regs_[aligned] = native;
    (void)kMmIndex;
    (void)kBiosScratch;
    (void)kGenReset;
    (void)kMemCntl;
    (void)kCrtcGenCntl;
    (void)kPllTest;
}


// A 128-byte EDID for a plain multiscan monitor that can do 640x480 and
// 800x600 at 60 Hz. Generated rather than stored: the fields a display
// driver reads are few, and a table of magic bytes teaches nobody which
// ones matter.
u8 R128Cell::edidByte(u8 at) const
{
    switch (at) {
    case 0: case 7: return 0x00;
    case 1: case 2: case 3: case 4: case 5: case 6: return 0xFF; // header
    case 8: return 0x06; case 9: return 0x10;   // manufacturer "APP"
    case 10: return 0x01; case 11: return 0x9D; // product
    case 16: return 0x0A; case 17: return 0x0B; // week 10, year 2001
    case 18: return 0x01; case 19: return 0x03; // EDID 1.3
    case 20: return 0x0E;                       // analogue input
    case 21: return 0x22; case 22: return 0x1B; // 34 cm x 27 cm
    case 23: return 0x78;                       // gamma 2.2
    case 24: return 0x0D;                       // RGB, preferred timing
    case 35: return 0x21;  // established: 640x480@60, 800x600@60
    case 54: return 0x31; case 55: return 0x40; // pixel clock 16.4 MHz
    case 56: return 0x80; case 57: return 0xE0; // 640 x 480 active
    case 58: return 0x00; case 59: return 0x18;
    case 60: return 0x10; case 61: return 0x20;
    default: break;
    }
    if (at == 127) {
        // Checksum: the whole 128 bytes must sum to zero mod 256.
        u32 s = 0;
        for (u32 k = 0; k < 127; ++k)
            s += edidByte(static_cast<u8>(k));
        return static_cast<u8>(0x100u - (s & 0xFFu));
    }
    return 0x00;
}

// One step of the DDC I2C bus, called whenever either line could have
// moved. START is SDA falling while SCL is high, STOP is SDA rising while
// SCL is high, and every other bit is sampled on SCL's rising edge.
void R128Cell::ddcStep(bool scl, bool sda)
{
    if (ddc_.lastScl && scl && sda != ddc_.lastSda) {
        if (!sda) { // START
            ++ddcStarts;
            ddc_.state = 1;
            ddc_.shift = 0;
            ddc_.bits = 0;
            ddc_.sdaOut = true;
        } else { // STOP
            ddc_.state = 0;
            ddc_.sdaOut = true;
        }
        ddc_.lastSda = sda;
        return;
    }
    if (scl && !ddc_.lastScl) { // rising edge: sample
        switch (ddc_.state) {
        case 1: // address byte
            ddc_.shift = static_cast<u8>((ddc_.shift << 1) | (sda ? 1 : 0));
            if (++ddc_.bits == 8) {
                ddc_.addr = ddc_.shift;
                ddcLastAddr = ddc_.addr;
                ddc_.bits = 0;
                // 0xA0/0xA1 is the EDID slave at 7-bit address 0x50.
                ddc_.state = (ddc_.addr & 0xFEu) == 0xA0u ? 2 : 0;
                if (ddc_.state == 2)
                    ++ddcMatches;
            }
            break;
        case 3: // data byte from the host (the EDID offset)
            ddc_.shift = static_cast<u8>((ddc_.shift << 1) | (sda ? 1 : 0));
            if (++ddc_.bits == 8) {
                ddc_.ptr = ddc_.shift;
                ddc_.bits = 0;
                ddc_.state = 4;
            }
            break;
        case 5: // data byte to the host
            if (++ddc_.bits == 8) {
                ddc_.bits = 0;
                ddc_.state = 6;
                ++ddcBytes;
                ++ddc_.ptr;
            }
            break;
        default: break;
        }
    } else if (!scl && ddc_.lastScl) { // falling edge: drive
        switch (ddc_.state) {
        case 2: // acknowledge the address
            ddc_.sdaOut = false;
            ddc_.state = (ddc_.addr & 1u) ? 5 : 3;
            ddc_.shift = (ddc_.addr & 1u) ? edidByte(ddc_.ptr) : 0;
            ddc_.bits = 0;
            break;
        case 4: // acknowledge the offset byte
            ddc_.sdaOut = false;
            ddc_.state = 3;
            break;
        case 5: // present the next bit, MSB first
            ddc_.sdaOut = (ddc_.shift & (0x80u >> ddc_.bits)) != 0;
            break;
        case 6: // release for the host's acknowledge
            ddc_.sdaOut = true;
            ddc_.state = 5;
            ddc_.shift = edidByte(ddc_.ptr);
            ddc_.bits = 0;
            break;
        default:
            ddc_.sdaOut = true;
            break;
        }
    }
    ddc_.lastScl = scl;
    ddc_.lastSda = sda;
}

} // namespace opm
