#pragma once
#include "opm/bus.hpp"
#include "opm/pmu.hpp"
#include "opm/types.hpp"

#include <map>
#include <string>
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
        // Sound codec status (+0x14020, accessed via lwbrx — the block is
        // byte-swapped on the 60x side): native value carries the same
        // Screamer-lineage fields the Gossamer receipts pinned — READY =
        // bit 22, revision in bits 15:12 (3 = Screamer class). Stored
        // here as the swapped image so the ROM's lwbrx sees 0x00403100.
        kl_[0x14020] = 0x00;
        kl_[0x14021] = 0x31;
        kl_[0x14022] = 0x40;
        kl_[0x14023] = 0x00;
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

    // The VIA cell (+0x16000..+0x17FFF) routes to the PMU99 model.
    PmuVia& pmu() { return pmu_; }

    // SCC (+0x13000): MacRISC layout — ctrl B/A at +0x00/+0x20, data B/A
    // at +0x10/+0x30. Enough Z8530 to drain transmit (RR0 TX-empty) and
    // capture the ROM's serial console log verbatim.
    const std::string& console() const { return console_; }

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
            const u32 off = pa - kMacIoBase;
            if (off >= 0x16000u && off < 0x18000u) {
                u32 v = 0;
                for (u32 k = 0; k < len; ++k)
                    v = (v << 8) |
                        pmu_.read(off - 0x16000u + k, stamp ? *stamp : 0);
                return v;
            }
            if (off >= 0x13000u && off < 0x14000u)
                return sccRead(off - 0x13000u);
            const u32 v = get(kl_.data() + off, len);
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
            const u32 off = pa - kMacIoBase;
            if (off >= 0x16000u && off < 0x18000u) {
                for (u32 k = 0; k < len; ++k)
                    pmu_.write(off - 0x16000u + k,
                               static_cast<u8>(v >> (8 * (len - 1 - k))),
                               stamp ? *stamp : 0);
                return;
            }
            if (off >= 0x13000u && off < 0x14000u) {
                sccWrite(off - 0x13000u, static_cast<u8>(v));
                return;
            }
            put(kl_.data() + off, v, len);
            klNote(pa, v, true);
            return;
        }
        note(pa, v, true); // ROM/flash writes land here too, unapplied
    }

    // Z8530, just enough for a polled console: pointer-register protocol
    // per channel, RR0 reports TX-empty always, data writes append to the
    // captured console text.
    u32 sccRead(u32 off)
    {
        const u32 ch = (off >> 5) & 1u; // 0 = B (+0x00), 1 = A (+0x20)
        if (off & 0x10u)
            return 0; // data read: nothing buffered
        const u32 r = sccPtr_[ch];
        sccPtr_[ch] = 0;
        if (r == 0)
            return 0x04u; // RR0: TX buffer empty
        return sccRr_[ch][r & 15u];
    }
    void sccWrite(u32 off, u8 v)
    {
        const u32 ch = (off >> 5) & 1u;
        if (off & 0x10u) {
            console_ += static_cast<char>(v);
            return;
        }
        if (sccPtr_[ch] == 0) {
            const u32 lo = v & 7u;
            if (lo == 0 && (v & 0x38u) == 0x08u)
                sccPtr_[ch] = 8; // point-high alone selects reg 8+
            else if (lo != 0)
                sccPtr_[ch] = lo | (((v & 0x38u) == 0x08u) ? 8u : 0u);
            // command/reset bits in WR0 are accepted and ignored
            return;
        }
        sccWr_[ch][sccPtr_[ch] & 15u] = v;
        if (sccPtr_[ch] == 8)
            console_ += static_cast<char>(v); // WR8 = data register
        sccPtr_[ch] = 0;
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
    PmuVia pmu_;
    u32 sccPtr_[2] = {0, 0};
    u8 sccWr_[2][16] = {};
    u8 sccRr_[2][16] = {};
    std::string console_;
};

} // namespace opm
