#include "opm/gossamer.hpp"

namespace opm {

namespace {
inline constexpr size_t kStubLogCap = 512;
}

GossamerBus::GossamerBus(size_t ramBytes, std::vector<u8> rom)
    : ram_(ramBytes, 0), rom_(std::move(rom))
{
    rom_.resize(0x00400000u, 0xFF); // 4 MB window, unprogrammed bytes read FF
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
    if (pa >= kRomBase)
        return rom_[pa - kRomBase];
    const u32 w = readWord(pa & ~3u);
    return static_cast<u8>(w >> (8 * (3 - (pa & 3u))));
}

u16 GossamerBus::read16(u32 pa)
{
    if (pa + 1 < ram_.size())
        return static_cast<u16>((ram_[pa] << 8) | ram_[pa + 1]);
    if (pa >= kRomBase)
        return static_cast<u16>((rom_[pa - kRomBase] << 8) |
                                rom_[pa - kRomBase + 1]);
    const u32 w = readWord(pa & ~3u);
    return static_cast<u16>(w >> ((pa & 2u) ? 0 : 16));
}

u32 GossamerBus::read32(u32 pa)
{
    if (pa + 3 < ram_.size())
        return (u32(ram_[pa]) << 24) | (u32(ram_[pa + 1]) << 16) |
               (u32(ram_[pa + 2]) << 8) | u32(ram_[pa + 3]);
    if (pa >= kRomBase) {
        const u32 off = pa - kRomBase;
        return (u32(rom_[off]) << 24) | (u32(rom_[off + 1]) << 16) |
               (u32(rom_[off + 2]) << 8) | u32(rom_[off + 3]);
    }
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
    if (pa >= kRomBase) {
        ++romWrites_;
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
    if (pa >= kRomBase) {
        ++romWrites_;
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
    if (pa >= kRomBase) {
        ++romWrites_;
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
    if (pa >= kMacIoBase && pa < kMacIoBase + kMacIoSize) {
        logStub(pa, false, 0);
        return 0; // quiet device registers until M2
    }
    logStub(pa, false, 0);
    return 0xFFFFFFFFu; // unmapped: all-ones
}

void GossamerBus::writeWord(u32 pa, u32 v)
{
    if (pa == kConfigAddr) {
        configAddr_ = v;
        logStub(pa, true, v);
        return;
    }
    logStub(pa, true, v); // CONFIG_DATA / mac-io / unmapped: dropped, logged
}

} // namespace opm
