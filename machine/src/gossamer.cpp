#include "opm/gossamer.hpp"

namespace opm {

namespace {
inline constexpr size_t kStubLogCap = 512;
}

static_assert(PciConfig::kAtiAperture == AtiRage::kAperture,
              "config-space BAR sizing must match the device aperture");

GossamerBus::GossamerBus(size_t ramBytes, std::vector<u8> rom)
    : ram_(ramBytes, 0), rom_(std::move(rom))
{
    rom_.resize(0x00400000u, 0xFF); // 4 MB window, unprogrammed bytes read FF
    // The DIMM-wrap aliasing mask requires a power-of-two module.
    if (ramBytes == 0 || (ramBytes & (ramBytes - 1)) != 0)
        ram_.assign(0x04000000u, 0);
    macio_.dmaBus = this; // DBDMA descriptor fetches ride the system bus
}

// Grackle 60x-memory decode (MPC106 UM 3.2.8): a physical address is RAM
// only while MEMGO is set and a configured, enabled bank claims it. The
// claimed offset wraps modulo the backing DIMM — the address-line aliasing
// a real undersized DIMM exhibits, and exactly the signal the ROM's memory
// sizing algorithm measures. RECEIPT: wrap-modulo-DIMM approximates the
// row/column aliasing of the real array (MCCR1 bank-row fields not yet
// honored); refine per-geometry when the sizer demands it.
bool GossamerBus::ramClaim(u32 pa, u32& off)
{
    memCfgRefresh();
    if (forceBank0 && pa < ram_.size()) {
        off = pa;
        return true;
    }
    if (!memGo_)
        return false;
    for (const auto& b : banks_) {
        if (b.en && pa >= b.lo && pa <= b.hi) {
            off = (pa - b.lo) & (static_cast<u32>(ram_.size()) - 1u);
            return true;
        }
    }
    return false;
}

void GossamerBus::memCfgRefresh()
{
    if (bankGen_ == pci_.memGeneration())
        return;
    bankGen_ = pci_.memGeneration();
    pci_.ramBanks(banks_);
    memGo_ = pci_.memGo();
    // MPC106 internally-controlled L2: PICR1[CF_L2_MP] names an internal
    // write-through(01)/write-back(10) L2 with CF_EXTERNAL_L2 clear, and
    // PICR2[L2_EN] turns it on. Size per PICR2[CF_L2_SIZE].
    const u8* c = pci_.cfgBytes(0);
    if (!c) {
        ml2On_ = false;
        return;
    }
    const u32 picr1 = u32(c[0xA8]) | (u32(c[0xA9]) << 8) |
                      (u32(c[0xAA]) << 16) | (u32(c[0xAB]) << 24);
    const u32 picr2 = u32(c[0xAC]) | (u32(c[0xAD]) << 8) |
                      (u32(c[0xAE]) << 16) | (u32(c[0xAF]) << 24);
    const u32 mp = picr1 & 3u;
    const bool internal = (mp == 1u || mp == 2u) && !((picr1 >> 8) & 1u);
    ml2On_ = internal && (picr2 & 0x40000000u) != 0;
    if (ml2On_) {
        const u32 lines = (0x40000u << ((picr2 >> 4) & 3u)) / 32u;
        if (lines != ml2Lines_) {
            ml2Lines_ = lines;
            ml2_.assign(lines, Ml2Line{});
        }
    }
}

GossamerBus::Ml2Line* GossamerBus::ml2Lookup(u32 pa)
{
    memCfgRefresh();
    if (!ml2On_ || pa >= 0x40000000u)
        return nullptr;
    Ml2Line& e = ml2_[(pa >> 5) % ml2Lines_];
    return (e.v && e.tag == (pa >> 5)) ? &e : nullptr;
}

void GossamerBus::readLine32(u32 pa, u8* out)
{
    if (pa < 0x40000000u) {
        if (Ml2Line* e = ml2Route(pa)) {
            for (u32 k = 0; k < 32; ++k)
                out[k] = e->b[k];
            return;
        }
    }
    Bus::readLine32(pa, out);
}

void GossamerBus::writeLine32(u32 pa, const u8* b)
{
    if (pa < 0x40000000u) {
        if (Ml2Line* e = ml2Route(pa)) {
            for (u32 k = 0; k < 32; ++k)
                e->b[k] = b[k];
            e->d = true;
            return;
        }
    }
    Bus::writeLine32(pa, b);
}

// The inline L2 fronts the 60x memory space: hits never touch DRAM (which
// is how the boot's pre-DRAM world survives), burst misses cast out and
// fill from whatever the banks decode (all-ones where nothing does).
GossamerBus::Ml2Line* GossamerBus::ml2Route(u32 pa)
{
    memCfgRefresh();
    if (!ml2On_ || pa >= 0x40000000u)
        return nullptr;
    const u32 idx = (pa >> 5) % ml2Lines_;
    const u32 tag = pa >> 5;
    Ml2Line& e = ml2_[idx];
    if (e.v && e.tag == tag)
        return &e;
    if (e.v && e.d) {
        const u32 vbase = e.tag << 5;
        u32 off0;
        if (!ramClaim(vbase, off0)) {
            ++ml2LostCastouts;
            if (ml2LossLog.size() < 64)
                ml2LossLog.push_back({stamp ? *stamp : 0, vbase});
        }
        for (u32 k = 0; k < 32; ++k) {
            u32 off;
            if (ramClaim(vbase + k, off))
                ram_[off] = e.b[k];
        }
    }
    ++ml2Fills;
    const u32 base = pa & ~31u;
    for (u32 k = 0; k < 32; ++k) {
        u32 off;
        e.b[k] = ramClaim(base + k, off) ? ram_[off] : 0xFF;
    }
    e.tag = tag;
    e.v = true;
    e.d = false;
    return &e;
}

bool GossamerBus::atiWindow(u32 pa, u32& off) const
{
    const u32 base = pci_.atiBase();
    if (base != 0 && pa - base < AtiRage::kAperture) {
        off = pa - base;
        return true;
    }
    // Personality alias: the ROM's video driver addresses the onboard chip
    // at 0x8F800000 regardless of where enumeration parked BAR0 (observed
    // register probes at 0x8FFFF800 while BAR0 held 0x82000000). RECEIPT:
    // fixed decode kept until the real ROM's BAR-assignment story is pinned.
    if (pa - 0x8F800000u < AtiRage::kAperture) {
        off = pa - 0x8F800000u;
        return true;
    }
    return false;
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

void GossamerBus::logCfgWrite(u32 lane, u8 v)
{
    const u32 a = pci_.addr();
    if (!(a & 0x80000000u) || ((a >> 11) & 0x1Fu) != 0)
        return; // only Grackle itself
    if (cfgLog_.size() < 4096)
        cfgLog_.push_back({stamp ? *stamp : 0, (a & 0xFCu) | lane, v});
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
    if (pa < 0x40000000u) {
        if (Ml2Line* e = ml2Lookup(pa))
            return e->b[pa & 31u];
        u32 ro;
        if (ramClaim(pa, ro))
            return ram_[ro];
    }
    if (pa >= kRomAlias)
        return rom_[pa & kRomMask];
    if (pa >= kMacIoBase && pa < kMacIoBase + kMacIoSize)
        return macio_.read8(pa - kMacIoBase);
    if ((pa & ~3u) == kConfigData)
        return pci_.readData(pa & 3u);
    u32 off;
    if (atiWindow(pa, off))
        return ati_.apRead8(off);
    if (atiIoWindow(pa, off))
        return ati_.ioRead8(off);
    const u32 w = readWord(pa & ~3u);
    return static_cast<u8>(w >> (8 * (3 - (pa & 3u))));
}

u16 GossamerBus::read16(u32 pa)
{
    if (pa < 0x40000000u) {
        if ((pa & 31u) > 30u) // crosses a line: route per byte
            return static_cast<u16>((read8(pa) << 8) | read8(pa + 1));
        if (Ml2Line* e = ml2Lookup(pa))
            return static_cast<u16>((e->b[pa & 31u] << 8) |
                                    e->b[(pa & 31u) + 1]);
        u32 ro;
        if (ramClaim(pa, ro)) {
            const u32 m = static_cast<u32>(ram_.size()) - 1u;
            return static_cast<u16>((ram_[ro] << 8) | ram_[(ro + 1) & m]);
        }
    }
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
    if (pa < 0x40000000u) {
        if ((pa & 31u) > 28u)
            return (u32(read8(pa)) << 24) | (u32(read8(pa + 1)) << 16) |
                   (u32(read8(pa + 2)) << 8) | u32(read8(pa + 3));
        if (Ml2Line* e = ml2Lookup(pa)) {
            const u32 o = pa & 31u;
            return (u32(e->b[o]) << 24) | (u32(e->b[o + 1]) << 16) |
                   (u32(e->b[o + 2]) << 8) | u32(e->b[o + 3]);
        }
        u32 ro;
        if (ramClaim(pa, ro)) {
            const u32 m = static_cast<u32>(ram_.size()) - 1u;
            return (u32(ram_[ro]) << 24) | (u32(ram_[(ro + 1) & m]) << 16) |
                   (u32(ram_[(ro + 2) & m]) << 8) | u32(ram_[(ro + 3) & m]);
        }
    }
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
        return (u32(ati_.apRead8(off)) << 24) |
               (u32(ati_.apRead8(off + 1)) << 16) |
               (u32(ati_.apRead8(off + 2)) << 8) | u32(ati_.apRead8(off + 3));
    if (atiIoWindow(pa, off))
        return (u32(ati_.ioRead8(off)) << 24) |
               (u32(ati_.ioRead8(off + 1)) << 16) |
               (u32(ati_.ioRead8(off + 2)) << 8) | u32(ati_.ioRead8(off + 3));
    return readWord(pa);
}

u64 GossamerBus::read64(u32 pa)
{
    return (static_cast<u64>(read32(pa)) << 32) | read32(pa + 4);
}

void GossamerBus::write8(u32 pa, u8 v)
{
    if (pa < 0x40000000u) {
        if (Ml2Line* e = ml2Lookup(pa)) {
            e->b[pa & 31u] = v;
            e->d = true;
            return;
        }
        u32 ro;
        if (ramClaim(pa, ro)) {
            if (htabWatchSize && ro - htabWatchBase < htabWatchSize &&
                ram_[ro] != v && htabLog_.size() < 65536)
                htabLog_.push_back({stamp ? *stamp : 0, ro | 0x80000000u,
                                    ram_[ro], v});
            ram_[ro] = v;
            return;
        }
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
        logCfgWrite(pa & 3u, v);
        pci_.writeData(pa & 3u, v);
        return;
    }
    u32 off;
    if (atiWindow(pa, off)) {
        ati_.apWrite8(off, v);
        return;
    }
    if (atiIoWindow(pa, off)) {
        ati_.ioWrite8(off, v);
        return;
    }
    writeWord(pa & ~3u, static_cast<u32>(v) << (8 * (3 - (pa & 3u))));
}

void GossamerBus::write16(u32 pa, u16 v)
{
    if (pa < 0x40000000u) {
        if ((pa & 31u) > 30u) {
            write8(pa, static_cast<u8>(v >> 8));
            write8(pa + 1, static_cast<u8>(v));
            return;
        }
        if (Ml2Line* e = ml2Lookup(pa)) {
            e->b[pa & 31u] = static_cast<u8>(v >> 8);
            e->b[(pa & 31u) + 1] = static_cast<u8>(v);
            e->d = true;
            return;
        }
        u32 ro;
        if (ramClaim(pa, ro)) {
            const u32 m = static_cast<u32>(ram_.size()) - 1u;
            if (htabWatchSize && ro - htabWatchBase < htabWatchSize &&
                htabLog_.size() < 65536) {
                const u32 old = (u32(ram_[ro]) << 8) | ram_[(ro + 1) & m];
                if (old != v)
                    htabLog_.push_back({stamp ? *stamp : 0,
                                        ro | 0x40000000u, old, v});
            }
            ram_[ro] = static_cast<u8>(v >> 8);
            ram_[(ro + 1) & m] = static_cast<u8>(v);
            return;
        }
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
    if (pa < 0x40000000u) {
        if ((pa & 31u) > 28u) {
            write8(pa, static_cast<u8>(v >> 24));
            write8(pa + 1, static_cast<u8>(v >> 16));
            write8(pa + 2, static_cast<u8>(v >> 8));
            write8(pa + 3, static_cast<u8>(v));
            return;
        }
        if (Ml2Line* e = ml2Lookup(pa)) {
            const u32 o = pa & 31u;
            e->b[o] = static_cast<u8>(v >> 24);
            e->b[o + 1] = static_cast<u8>(v >> 16);
            e->b[o + 2] = static_cast<u8>(v >> 8);
            e->b[o + 3] = static_cast<u8>(v);
            e->d = true;
            return;
        }
        u32 ro;
        if (ramClaim(pa, ro)) {
            const u32 m = static_cast<u32>(ram_.size()) - 1u;
            if (htabWatchSize && ro - htabWatchBase < htabWatchSize &&
                htabLog_.size() < 65536) {
                const u32 old = (u32(ram_[ro]) << 24) |
                                (u32(ram_[(ro + 1) & m]) << 16) |
                                (u32(ram_[(ro + 2) & m]) << 8) |
                                u32(ram_[(ro + 3) & m]);
                if (old != v)
                    htabLog_.push_back({stamp ? *stamp : 0, ro, old, v});
            }
            ram_[ro] = static_cast<u8>(v >> 24);
            ram_[(ro + 1) & m] = static_cast<u8>(v >> 16);
            ram_[(ro + 2) & m] = static_cast<u8>(v >> 8);
            ram_[(ro + 3) & m] = static_cast<u8>(v);
            return;
        }
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
        logCfgWrite(0, static_cast<u8>(v >> 24));
        logCfgWrite(1, static_cast<u8>(v >> 16));
        logCfgWrite(2, static_cast<u8>(v >> 8));
        logCfgWrite(3, static_cast<u8>(v));
        pci_.writeData(0, static_cast<u8>(v >> 24));
        pci_.writeData(1, static_cast<u8>(v >> 16));
        pci_.writeData(2, static_cast<u8>(v >> 8));
        pci_.writeData(3, static_cast<u8>(v));
        return;
    }
    u32 off;
    if (atiWindow(pa, off)) {
        ati_.apWrite8(off, static_cast<u8>(v >> 24));
        ati_.apWrite8(off + 1, static_cast<u8>(v >> 16));
        ati_.apWrite8(off + 2, static_cast<u8>(v >> 8));
        ati_.apWrite8(off + 3, static_cast<u8>(v));
        return;
    }
    if (atiIoWindow(pa, off)) {
        ati_.ioWrite8(off, static_cast<u8>(v >> 24));
        ati_.ioWrite8(off + 1, static_cast<u8>(v >> 16));
        ati_.ioWrite8(off + 2, static_cast<u8>(v >> 8));
        ati_.ioWrite8(off + 3, static_cast<u8>(v));
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
