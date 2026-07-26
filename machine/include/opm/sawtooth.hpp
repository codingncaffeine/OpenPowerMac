#pragma once
#include "opm/bus.hpp"
#include "opm/types.hpp"

#include <map>
#include <vector>

namespace opm {

// Power Mac G4 AGP "Sawtooth" — the project's primary machine.
//
// M-SAW-0 founding skeleton: RAM at 0, the 1 MB New World boot ROM (flash)
// at 0xFFF00000, and NOTHING else claimed. Every unclaimed access
// master-aborts honestly (reads all-ones, writes dropped) and lands in a
// deduplicated log with the pc that issued it — the boot ROM itself maps
// out what Uni-North / KeyLargo / AGP need to look like, exactly the
// recipe that founded the Gossamer machine.
//
// The boot flash is writable on real hardware (OF's NVRAM partition lives
// inside it); writes are logged, not applied, until the flash command
// protocol is modeled.
class SawtoothBus : public Bus {
public:
    static constexpr u32 kRomBase = 0xFFF00000u;
    static constexpr u32 kRomSize = 0x00100000u;

    SawtoothBus(size_t ramBytes, std::vector<u8> rom)
        : ram_(ramBytes, 0), rom_(std::move(rom))
    {
        // GPIO-region byte +0x61: the ROM polls bit 1 (TB-bounded) right
        // after arming the VIA/PMU cell and before continuing hardware
        // init — a ready/level input, high at power-on. (Semantic name
        // still to be pinned from the KeyLargo GPIO map.)
        kl_[0x61] = 0x02;
    }

    u8 read8(u32 pa) override { return static_cast<u8>(read(pa, 1)); }
    u16 read16(u32 pa) override { return static_cast<u16>(read(pa, 2)); }
    u32 read32(u32 pa) override { return read(pa, 4); }
    u64 read64(u32 pa) override
    {
        return (static_cast<u64>(read(pa, 4)) << 32) | read(pa + 4, 4);
    }

    void write8(u32 pa, u8 v) override { write(pa, v, 1); }
    void write16(u32 pa, u16 v) override { write(pa, v, 2); }
    void write32(u32 pa, u32 v) override { write(pa, v, 4); }
    void write64(u32 pa, u64 v) override
    {
        write(pa, static_cast<u32>(v >> 32), 4);
        write(pa + 4, static_cast<u32>(v), 4);
    }

    // Uni-North host-bridge register block at 0xF8000000: a plain
    // word-register store, all-zero at power-on. Zero in HWINIT_STATE
    // (+0x70) is what tells the ROM this is a cold boot rather than a
    // sleep-wake (bit0) or a double boot (bit1) — served all-ones, the
    // ROM put the machine back to sleep. Individual registers earn real
    // semantics as the boot demands them.
    static constexpr u32 kUniNBase = 0xF8000000u;
    static constexpr u32 kUniNSize = 0x1000u;

    struct RegWr {
        u64 at;
        u32 pa, val, pc;
    };
    const std::vector<RegWr>& uninLog() const { return uninLog_; }

    struct Acc {
        u64 firstAt = 0;
        u32 firstPc = 0;
        u32 lastWr = 0;
        u64 reads = 0, writes = 0;
    };

    // KeyLargo mac-io at 0xF3000000 (standard 512 KB BAR, hard-decoded by
    // the boot ROM before PCI enumeration): a plain register store for
    // now — FCRs, SCC, VIA/PMU cell, sound — with a first-touch log so
    // each new register the ROM consults stays visible. Blocks earn real
    // device models as the boot demands behavior a store can't fake.
    static constexpr u32 kMacIoBase = 0xF3000000u;
    static constexpr u32 kMacIoSize = 0x80000u;
    const std::map<u32, Acc>& macioLog() const { return klLog_; }
    std::vector<u32> macioOrder;

    // Unclaimed + ROM-write traffic, keyed by physical address.
    const std::map<u32, Acc>& accessLog() const { return log_; }
    std::vector<u32> logOrder; // first-touch order

    const u32* pcRef = nullptr;
    const u64* stamp = nullptr;

    size_t ramBytes() const { return ram_.size(); }

private:
    u32 read(u32 pa, u32 len)
    {
        if (pa < ram_.size() && pa + len <= ram_.size())
            return get(ram_.data() + pa, len);
        if (pa >= kRomBase && pa - kRomBase + len <= rom_.size())
            return get(rom_.data() + (pa - kRomBase), len);
        if (pa >= kUniNBase && pa + len <= kUniNBase + kUniNSize)
            return get(unin_ + (pa - kUniNBase), len);
        if (pa >= kMacIoBase && pa + len <= kMacIoBase + kMacIoSize) {
            const u32 v = get(kl_.data() + (pa - kMacIoBase), len);
            klNote(pa, 0, false);
            return v;
        }
        note(pa, 0, false);
        return len == 1 ? 0xFFu : len == 2 ? 0xFFFFu : 0xFFFFFFFFu;
    }

    void write(u32 pa, u32 v, u32 len)
    {
        if (pa < ram_.size() && pa + len <= ram_.size()) {
            put(ram_.data() + pa, v, len);
            return;
        }
        if (pa >= kUniNBase && pa + len <= kUniNBase + kUniNSize) {
            put(unin_ + (pa - kUniNBase), v, len);
            if (uninLog_.size() < 4096)
                uninLog_.push_back({stamp ? *stamp : 0, pa, v,
                                    pcRef ? *pcRef : 0});
            return;
        }
        if (pa >= kMacIoBase && pa + len <= kMacIoBase + kMacIoSize) {
            put(kl_.data() + (pa - kMacIoBase), v, len);
            klNote(pa, v, true);
            return;
        }
        note(pa, v, true); // ROM/flash writes land here too, unapplied
    }

    void klNote(u32 pa, u32 v, bool wr)
    {
        auto [it, fresh] = klLog_.try_emplace(pa);
        Acc& a = it->second;
        if (fresh) {
            a.firstAt = stamp ? *stamp : 0;
            a.firstPc = pcRef ? *pcRef : 0;
            macioOrder.push_back(pa);
        }
        if (wr) {
            ++a.writes;
            a.lastWr = v;
        } else {
            ++a.reads;
        }
    }

    static u32 get(const u8* p, u32 len)
    {
        u32 v = 0;
        for (u32 k = 0; k < len; ++k)
            v = (v << 8) | p[k];
        return v;
    }
    static void put(u8* p, u32 v, u32 len)
    {
        for (u32 k = 0; k < len; ++k)
            p[k] = static_cast<u8>(v >> (8 * (len - 1 - k)));
    }

    void note(u32 pa, u32 v, bool wr)
    {
        auto [it, fresh] = log_.try_emplace(pa);
        Acc& a = it->second;
        if (fresh) {
            a.firstAt = stamp ? *stamp : 0;
            a.firstPc = pcRef ? *pcRef : 0;
            logOrder.push_back(pa);
        }
        if (wr) {
            ++a.writes;
            a.lastWr = v;
        } else {
            ++a.reads;
        }
    }

    std::vector<u8> ram_, rom_;
    u8 unin_[kUniNSize] = {};
    std::vector<u8> kl_ = std::vector<u8>(kMacIoSize, 0);
    std::vector<RegWr> uninLog_;
    std::map<u32, Acc> log_, klLog_;
};

} // namespace opm
