#include "opm/gossamer.hpp"

namespace opm {

namespace {
inline constexpr size_t kStubLogCap = 512;
}

GossamerBus::GossamerBus(size_t ramBytes, std::vector<u8> rom)
    : ram_(ramBytes, 0), rom_(std::move(rom)),
      atiMem_(PciConfig::kAtiAperture, 0)
{
    rom_.resize(0x00400000u, 0xFF); // 4 MB window, unprogrammed bytes read FF
}

bool GossamerBus::atiWindow(u32 pa, u32& off) const
{
    const u32 base = pci_.atiBase();
    if (base == 0 || pa < base || pa - base >= PciConfig::kAtiAperture)
        return false;
    off = pa - base;
    return true;
}

// Grackle maps PCI I/O space at 0xFE000000; the ATI's I/O BAR lands inside.
bool GossamerBus::atiIoWindow(u32 pa, u32& off) const
{
    const u32 io = pci_.atiIoBase();
    if (io == 0 || pa < 0xFE000000u + io || pa - (0xFE000000u + io) >= 0x100u)
        return false;
    off = pa - (0xFE000000u + io);
    return true;
}

u8 GossamerBus::atiRead8(u32 off)
{
    // The mach64-family register file lives in the top of the aperture;
    // log that traffic so polled status registers announce themselves.
    if (off >= PciConfig::kAtiAperture - 0x1000 &&
        (atiRegLog_.size() < 256 || atiRegLog_.count(off)))
        ++atiRegLog_[off].reads;
    return atiMem_[off];
}

void GossamerBus::atiWrite8(u32 off, u8 v)
{
    if (off >= PciConfig::kAtiAperture - 0x1000 &&
        (atiRegLog_.size() < 256 || atiRegLog_.count(off))) {
        Touch& t = atiRegLog_[off];
        ++t.writes;
        t.lastWrite = v;
    }
    atiMem_[off] = v;
}

void GossamerBus::logStub(u32 pa, bool write, u32 v)
{
    if (stubLog_.size() >= kStubLogCap && stubLog_.find(pa) == stubLog_.end())
        return;
    Touch& t = stubLog_[pa];
    if (write) {
        ++t.writes;
        t.lastWrite = v;
    } else {
        ++t.reads;
    }
}

// Byte-granular routing built on the sized accessors below; RAM and ROM are
// the fast paths, everything else funnels through the word backend so the
// stub log sees device-register-shaped traffic.

u8 GossamerBus::read8(u32 pa)
{
    if (pa < ram_.size())
        return ram_[pa];
    if (pa >= kRomAlias)
        return rom_[pa & kRomMask];
    if (pa >= kMacIoBase && pa < kMacIoBase + kMacIoSize)
        return macio_.read8(pa - kMacIoBase);
    if ((pa & ~3u) == kConfigData)
        return pci_.readData(pa & 3u);
    u32 off;
    if (atiWindow(pa, off))
        return atiRead8(off);
    if (atiIoWindow(pa, off))
        return atiIo_[off];
    const u32 w = readWord(pa & ~3u);
    return static_cast<u8>(w >> (8 * (3 - (pa & 3u))));
}

u16 GossamerBus::read16(u32 pa)
{
    if (pa + 1 < ram_.size())
        return static_cast<u16>((ram_[pa] << 8) | ram_[pa + 1]);
    if (pa >= kRomAlias)
        return static_cast<u16>((rom_[pa & kRomMask] << 8) |
                                rom_[(pa + 1) & kRomMask]);
    if (pa >= kMacIoBase && pa < kMacIoBase + kMacIoSize)
        return static_cast<u16>((macio_.read8(pa - kMacIoBase) << 8) |
                                macio_.read8(pa - kMacIoBase + 1));
    const u32 w = readWord(pa & ~3u);
    return static_cast<u16>(w >> ((pa & 2u) ? 0 : 16));
}

u32 GossamerBus::read32(u32 pa)
{
    if (pa + 3 < ram_.size())
        return (u32(ram_[pa]) << 24) | (u32(ram_[pa + 1]) << 16) |
               (u32(ram_[pa + 2]) << 8) | u32(ram_[pa + 3]);
    if (pa >= kRomAlias) {
        const u32 off = pa & kRomMask;
        return (u32(rom_[off]) << 24) | (u32(rom_[off + 1]) << 16) |
               (u32(rom_[off + 2]) << 8) | u32(rom_[off + 3]);
    }
    if (pa >= kMacIoBase && pa < kMacIoBase + kMacIoSize) {
        const u32 off = pa - kMacIoBase;
        return (u32(macio_.read8(off)) << 24) |
               (u32(macio_.read8(off + 1)) << 16) |
               (u32(macio_.read8(off + 2)) << 8) | u32(macio_.read8(off + 3));
    }
    if ((pa & ~3u) == kConfigData)
        return (u32(pci_.readData(0)) << 24) | (u32(pci_.readData(1)) << 16) |
               (u32(pci_.readData(2)) << 8) | u32(pci_.readData(3));
    u32 off;
    if (atiWindow(pa, off))
        return (u32(atiRead8(off)) << 24) | (u32(atiRead8(off + 1)) << 16) |
               (u32(atiRead8(off + 2)) << 8) | u32(atiRead8(off + 3));
    if (atiIoWindow(pa, off))
        return (u32(atiIo_[off]) << 24) | (u32(atiIo_[off + 1]) << 16) |
               (u32(atiIo_[off + 2]) << 8) | u32(atiIo_[off + 3]);
    return readWord(pa);
}

u64 GossamerBus::read64(u32 pa)
{
    return (static_cast<u64>(read32(pa)) << 32) | read32(pa + 4);
}

void GossamerBus::write8(u32 pa, u8 v)
{
    if (pa < ram_.size()) {
        ram_[pa] = v;
        return;
    }
    if (pa >= kRomAlias) {
        ++romWrites_;
        return;
    }
    if (pa >= kMacIoBase && pa < kMacIoBase + kMacIoSize) {
        macio_.write8(pa - kMacIoBase, v);
        return;
    }
    if ((pa & ~3u) == kConfigData) {
        pci_.writeData(pa & 3u, v);
        return;
    }
    u32 off;
    if (atiWindow(pa, off)) {
        atiWrite8(off, v);
        return;
    }
    if (atiIoWindow(pa, off)) {
        atiIo_[off] = v;
        return;
    }
    writeWord(pa & ~3u, static_cast<u32>(v) << (8 * (3 - (pa & 3u))));
}

void GossamerBus::write16(u32 pa, u16 v)
{
    if (pa + 1 < ram_.size()) {
        ram_[pa] = static_cast<u8>(v >> 8);
        ram_[pa + 1] = static_cast<u8>(v);
        return;
    }
    if (pa >= kRomAlias) {
        ++romWrites_;
        return;
    }
    if (pa >= kMacIoBase && pa < kMacIoBase + kMacIoSize) {
        macio_.write8(pa - kMacIoBase, static_cast<u8>(v >> 8));
        macio_.write8(pa - kMacIoBase + 1, static_cast<u8>(v));
        return;
    }
    writeWord(pa & ~3u, static_cast<u32>(v) << ((pa & 2u) ? 0 : 16));
}

void GossamerBus::write32(u32 pa, u32 v)
{
    if (pa + 3 < ram_.size()) {
        ram_[pa] = static_cast<u8>(v >> 24);
        ram_[pa + 1] = static_cast<u8>(v >> 16);
        ram_[pa + 2] = static_cast<u8>(v >> 8);
        ram_[pa + 3] = static_cast<u8>(v);
        return;
    }
    if (pa >= kRomAlias) {
        ++romWrites_;
        return;
    }
    if (pa >= kMacIoBase && pa < kMacIoBase + kMacIoSize) {
        const u32 off = pa - kMacIoBase;
        macio_.write8(off, static_cast<u8>(v >> 24));
        macio_.write8(off + 1, static_cast<u8>(v >> 16));
        macio_.write8(off + 2, static_cast<u8>(v >> 8));
        macio_.write8(off + 3, static_cast<u8>(v));
        return;
    }
    if ((pa & ~3u) == kConfigData) {
        pci_.writeData(0, static_cast<u8>(v >> 24));
        pci_.writeData(1, static_cast<u8>(v >> 16));
        pci_.writeData(2, static_cast<u8>(v >> 8));
        pci_.writeData(3, static_cast<u8>(v));
        return;
    }
    u32 off;
    if (atiWindow(pa, off)) {
        atiWrite8(off, static_cast<u8>(v >> 24));
        atiWrite8(off + 1, static_cast<u8>(v >> 16));
        atiWrite8(off + 2, static_cast<u8>(v >> 8));
        atiWrite8(off + 3, static_cast<u8>(v));
        return;
    }
    if (atiIoWindow(pa, off)) {
        atiIo_[off] = static_cast<u8>(v >> 24);
        atiIo_[off + 1] = static_cast<u8>(v >> 16);
        atiIo_[off + 2] = static_cast<u8>(v >> 8);
        atiIo_[off + 3] = static_cast<u8>(v);
        return;
    }
    writeWord(pa, v);
}

void GossamerBus::write64(u32 pa, u64 v)
{
    write32(pa, static_cast<u32>(v >> 32));
    write32(pa + 4, static_cast<u32>(v));
}

u32 GossamerBus::readWord(u32 pa)
{
    if (pa == kConfigAddr) {
        logStub(pa, false, 0);
        return configAddr_;
    }
    if (pa == kConfigData) {
        logStub(pa, false, 0);
        return 0xFFFFFFFFu; // master abort: no PCI devices yet
    }
    logStub(pa, false, 0);
    return 0xFFFFFFFFu; // unmapped: all-ones
}

void GossamerBus::writeWord(u32 pa, u32 v)
{
    if (pa == kConfigAddr) {
        configAddr_ = v;
        // The CPU stores the config address with a byte-reversed store; the
        // bytes arrive here in memory order, so the LE meaning is the swap.
        pci_.setAddr(((v & 0xFFu) << 24) | ((v & 0xFF00u) << 8) |
                     ((v >> 8) & 0xFF00u) | (v >> 24));
        logStub(pa, true, v);
        return;
    }
    logStub(pa, true, v); // unmapped: dropped, logged
}

} // namespace opm
