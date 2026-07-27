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
    if (!wr) {
        if (seen_.count(off))
            return;
        seen_[off] = 1;
    }
    if (log.size() < 4096)
        log.push_back({stamp ? *stamp : 0, off, val,
                       pcRef ? *pcRef : 0, wr});
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
    note(off & ~3u, native, true);
    const u32 aligned = off & ~3u;
    if (aligned == kClockCntlIndex) {
        pllAddr_ = native & 0xFFu;
        // write-enable bit 7: data writes land in the PLL file
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

} // namespace opm
