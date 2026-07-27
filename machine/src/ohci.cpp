#include "opm/ohci.hpp"

namespace opm {

// One USB frame per millisecond of guest time; the timebase runs at
// bus/4 ≈ 24.94 MHz, so 25000 ticks is a frame within the tolerance any
// driver measures against.
static constexpr u64 kTbPerFrame = 25000;

u32 OhciCell::regRead(u32 idx)
{
    switch (idx) {
    case 0x00 >> 2: return 0x00000010u; // HcRevision: OHCI 1.0
    case 0x04 >> 2: return control_;
    case 0x08 >> 2: return cmdStatus_;
    case 0x0C >> 2: return intStatus_;
    case 0x10 >> 2:
    case 0x14 >> 2: return intEnable_;
    case 0x18 >> 2: return hcca_;
    case 0x1C >> 2: return periodCurrent_;
    case 0x20 >> 2: return ctrlHead_;
    case 0x24 >> 2: return ctrlCurrent_;
    case 0x28 >> 2: return bulkHead_;
    case 0x2C >> 2: return bulkCurrent_;
    case 0x30 >> 2: return doneHead_;
    case 0x34 >> 2: return fmInterval_;
    case 0x38 >> 2: return fmInterval_ & 0x3FFFu; // FmRemaining: coarse
    case 0x3C >> 2: return fmNumber_ & 0xFFFFu;
    case 0x40 >> 2: return periodicStart_;
    case 0x44 >> 2: return lsThreshold_;
    case 0x48 >> 2: return rhDescA_;
    case 0x4C >> 2: return rhDescB_;
    case 0x50 >> 2: return rhStatus_;
    case 0x54 >> 2: return rhPort_[0];
    case 0x58 >> 2: return rhPort_[1];
    default: return 0;
    }
}

void OhciCell::regWrite(u32 idx, u32 v)
{
    switch (idx) {
    case 0x04 >> 2:
        control_ = v;
        break;
    case 0x08 >> 2:
        // HCR (bit 0): software reset completes instantly — the spec
        // allows 10 µs; the bit reads back clear. Reset drops the
        // controller into UsbSuspend with interrupts disarmed.
        if (v & 1u) {
            control_ = (control_ & ~0xC0u) | 0xC0u; // UsbSuspend
            intStatus_ = 0;
            intEnable_ = 0;
            hcca_ = 0;
            fmNumber_ = 0;
        }
        cmdStatus_ = v & ~1u & 0x0000000Eu; // OCR/BLF/CLF latch, no lists
        break;
    case 0x0C >> 2:
        intStatus_ &= ~v; // write-1-to-clear
        break;
    case 0x10 >> 2:
        intEnable_ |= v; // write-1-to-set (MIE included)
        break;
    case 0x14 >> 2:
        intEnable_ &= ~v; // write-1-to-clear
        break;
    case 0x18 >> 2: hcca_ = v & 0xFFFFFF00u; break;
    case 0x20 >> 2: ctrlHead_ = v & ~0xFu; break;
    case 0x24 >> 2: ctrlCurrent_ = v & ~0xFu; break;
    case 0x28 >> 2: bulkHead_ = v & ~0xFu; break;
    case 0x2C >> 2: bulkCurrent_ = v & ~0xFu; break;
    case 0x34 >> 2: fmInterval_ = v; break;
    case 0x40 >> 2: periodicStart_ = v & 0x3FFFu; break;
    case 0x44 >> 2: lsThreshold_ = v & 0xFFFu; break;
    case 0x48 >> 2: // RhDescriptorA is mostly hardwired; NDP read-only
        rhDescA_ = (rhDescA_ & 0x000000FFu) | (v & 0xFFFFFF00u);
        break;
    case 0x4C >> 2: rhDescB_ = v; break;
    case 0x50 >> 2:
        // LPSC/LPS global power bits: accept and forget (ports report
        // powered through RhPortStatus PPS).
        break;
    case 0x54 >> 2:
    case 0x58 >> 2: {
        u32& p = rhPort_[idx - (0x54 >> 2)];
        // With nothing attached: SetPortEnable/SetPortReset on an empty
        // port set CSC per spec ("attempt on disconnected port"), the
        // status-change W1C bits clear, power bits track.
        if (v & 0x00000100u) // SetPortPower
            p |= 0x00000100u;
        if (v & 0x00000200u) // ClearPortPower
            p &= ~0x00000100u;
        if (v & 0x00000002u || v & 0x00000010u) // SPE / SPR on empty port
            p |= 0x00010000u;                   // CSC
        p &= ~((v & 0x001F0000u)); // W1C of change bits
        break;
    }
    default: break;
    }
}

u32 OhciCell::read(u32 off, u32 len)
{
    readCount[off & ~3u]++;
    const u32 native = regRead(off >> 2);
    if (len == 4)
        return swap32(native);
    // sub-word: serve the addressed LE byte lanes, BE-composed
    u32 r = 0;
    for (u32 k = 0; k < len; ++k)
        r = (r << 8) | ((native >> (8 * ((off + k) & 3u))) & 0xFFu);
    return r;
}

void OhciCell::write(u32 off, u32 v, u32 len)
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
    if (log.size() < 2048)
        log.push_back({stamp ? *stamp : 0, off, native,
                       pcRef ? *pcRef : 0});
    regWrite(off >> 2, native);
}

void OhciCell::tick(u64 tb)
{
    if (((control_ >> 6) & 3u) != 2u) { // not operational
        lastFrameTb_ = tb;
        return;
    }
    if (tb - lastFrameTb_ < kTbPerFrame)
        return;
    lastFrameTb_ = tb;
    fmNumber_ = (fmNumber_ + 1) & 0xFFFFu;
    if (hcca_ && ram && hcca_ + 0x84u <= ramSize) {
        // HccaFrameNumber: 16-bit little-endian at HCCA+0x80, pad zero.
        ram[hcca_ + 0x80] = static_cast<u8>(fmNumber_);
        ram[hcca_ + 0x81] = static_cast<u8>(fmNumber_ >> 8);
        ram[hcca_ + 0x82] = 0;
        ram[hcca_ + 0x83] = 0;
    }
    intStatus_ |= 0x00000004u; // SF
}

} // namespace opm
