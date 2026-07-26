#include "opm/pci.hpp"

namespace opm {

void PciConfig::put16(Dev& d, u32 off, u16 v)
{
    d.cfg[off] = static_cast<u8>(v);
    d.cfg[off + 1] = static_cast<u8>(v >> 8);
}
void PciConfig::put32(Dev& d, u32 off, u32 v)
{
    d.cfg[off] = static_cast<u8>(v);
    d.cfg[off + 1] = static_cast<u8>(v >> 8);
    d.cfg[off + 2] = static_cast<u8>(v >> 16);
    d.cfg[off + 3] = static_cast<u8>(v >> 24);
}

PciConfig::PciConfig()
{
    { // dev 0: MPC106 Grackle host bridge
        Dev& g = devs_[0x00];
        put16(g, 0x00, 0x1057);
        put16(g, 0x02, 0x0002);
        put16(g, 0x04, 0x0006); // command: memory + master
        put16(g, 0x06, 0x0080);
        g.cfg[0x08] = 0x40;     // revision
        put16(g, 0x0A, 0x0600); // host bridge
    }
    { // dev 0x10: Heathrow mac-io
        Dev& h = devs_[0x10];
        put16(h, 0x00, 0x106B);
        put16(h, 0x02, 0x0010);
        put16(h, 0x04, 0x0006);
        h.cfg[0x08] = 0x01;
        put16(h, 0x0A, 0xFF00); // Apple-specific class
        put32(h, 0x10, 0xF3000000u); // BAR0: the mac-io window
        h.barMask[0] = 0xFFF80000u;  // 512 KB
        h.cfg[0x3D] = 0x01;          // INTA
    }
    { // dev 0x12: ATI 3D Rage Pro
        Dev& a = devs_[0x12];
        put16(a, 0x00, 0x1002);
        put16(a, 0x02, 0x4750); // 'GP'
        put16(a, 0x04, 0x0006);
        put16(a, 0x06, 0x0280);
        a.cfg[0x08] = 0x5C;
        put16(a, 0x0A, 0x0300); // VGA-compatible display
        a.barMask[0] = ~(kAtiAperture - 1); // 16 MB memory aperture
        a.barMask[1] = 0xFFFFFF00u | 1u;    // 256 B I/O
        a.cfg[0x14] = 0x01;                 // io space bit
        a.barMask[2] = 0xFFFFF000u;         // 4 KB register aperture
        put16(a, 0x2C, 0x1002);
        put16(a, 0x2E, 0x4750);
        a.cfg[0x3D] = 0x01; // INTA
    }
}

PciConfig::Dev* PciConfig::find(u32 devNum)
{
    auto it = devs_.find(devNum);
    return it == devs_.end() ? nullptr : &it->second;
}

u8 PciConfig::readData(u32 lane)
{
    if (!(addr_ & 0x80000000u))
        return 0xFF;
    const u32 bus = (addr_ >> 16) & 0xFF;
    const u32 devNum = (addr_ >> 11) & 0x1F;
    const u32 reg = (addr_ & 0xFCu) | lane;
    ++probeLog_[(devNum << 8) | (addr_ & 0xFCu)].reads;
    if (bus != 0)
        return 0xFF;
    Dev* d = find(devNum);
    if (!d)
        return 0xFF; // master abort
    return d->cfg[reg];
}

void PciConfig::writeData(u32 lane, u8 v)
{
    if (!(addr_ & 0x80000000u))
        return;
    const u32 bus = (addr_ >> 16) & 0xFF;
    const u32 devNum = (addr_ >> 11) & 0x1F;
    const u32 reg = (addr_ & 0xFCu) | lane;
    ++probeLog_[(devNum << 8) | (addr_ & 0xFCu)].writes;
    if (bus != 0)
        return;
    Dev* d = find(devNum);
    if (!d)
        return;
    // BARs implement write-ones sizing against their masks.
    if (reg >= 0x10 && reg < 0x28) {
        const u32 barIdx = (reg - 0x10) / 4;
        const u32 byteIdx = reg & 3;
        const u32 mask = d->barMask[barIdx];
        if (mask == 0)
            return; // unimplemented BAR stays zero
        const u8 maskByte = static_cast<u8>(mask >> (8 * byteIdx));
        const u8 keep = static_cast<u8>(d->cfg[reg] & ~maskByte);
        d->cfg[reg] = static_cast<u8>((v & maskByte) | keep);
        return;
    }
    if (reg < 0x08)
        return; // IDs and command/status simplified: read-only
    d->cfg[reg] = v;
}

u32 PciConfig::atiBase() const
{
    auto it = devs_.find(0x12);
    if (it == devs_.end())
        return 0;
    const u8* c = it->second.cfg;
    const u32 bar = u32(c[0x10]) | (u32(c[0x11]) << 8) | (u32(c[0x12]) << 16) |
                    (u32(c[0x13]) << 24);
    return bar & ~(kAtiAperture - 1);
}

u32 PciConfig::atiIoBase() const
{
    auto it = devs_.find(0x12);
    if (it == devs_.end())
        return 0;
    const u8* c = it->second.cfg;
    const u32 bar = u32(c[0x14]) | (u32(c[0x15]) << 8) | (u32(c[0x16]) << 16) |
                    (u32(c[0x17]) << 24);
    return bar & 0xFFFFFF00u;
}

} // namespace opm
