#pragma once
#include "opm/types.hpp"

#include <map>
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

    R128Cell() : vram(32u << 20, 0) {}

    struct Ev {
        u64 at;
        u32 off, val, pc;
        bool wr;
    };
    std::vector<Ev> log; // first-touch + write traffic
    const u64* stamp = nullptr;
    const u32* pcRef = nullptr;

    // Harness peeks for the screen dump: raw register cell and the DAC
    // palette (PALETTE_INDEX/DATA auto-increment writes captured).
    u32 peek(u32 off) const
    {
        auto it = regs_.find(off & ~3u);
        return it != regs_.end() ? it->second : 0;
    }
    u32 pal(u32 i) const { return pal_[i & 0xFFu]; }

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

    std::map<u32, u32> regs_; // sparse store, offsets word-aligned
    std::map<u32, u32> seen_; // first-touch dedup for the log
    u32 pllAddr_ = 0;         // CLOCK_CNTL_INDEX low byte
    std::map<u32, u32> pll_;  // PLL register file
    u32 palIdx_ = 0;
    u32 pal_[256] = {};
};

} // namespace opm
