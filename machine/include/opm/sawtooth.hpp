#pragma once
#include "opm/ata.hpp"
#include "opm/dbdma.hpp"
#include "opm/bus.hpp"
#include "opm/ohci.hpp"
#include "opm/r128.hpp"
#include "opm/openpic.hpp"
#include "opm/pmu.hpp"
#include "opm/types.hpp"

#include <cstdio>
#include <map>
#include <string>
#include <vector>

namespace opm {

struct SnapWriter;
struct SnapReader;

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
        // Power-on DRAM is not zeroed on real hardware, and boot code
        // relies on that: the nanokernel reads a spinlock-timeout word
        // through a not-yet-set config pointer (landing in page zero) -
        // junk there gives a patient wait, zero gives an instant-timeout
        // complaint spiral. A deterministic non-zero fill models the
        // physical truth while keeping runs reproducible.
        for (size_t i = 0; i < ram_.size(); ++i)
            ram_[i] = static_cast<u8>(0x5Au ^ (i * 0x21u) ^ (i >> 11));
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
        // Uni-North VERSION (+0x00): Sawtooth silicon reports the 1.0.x
        // family; 0x07 = "1.0.10", the post-fix revision (Linux-validated
        // constant table).
        unin_[3] = 0x07;
        // PCI config seeds — mac-io KeyLargo at one-hot slot 23 on the
        // f2 bus (the ROM's own identity probe at latch 0x00800000
        // reg 0, observed @913). Identity per the Apple ID space:
        // vendor 0x106B, device 0x0022, class ff (other), rev 3.
        // Remaining registers of the device start as zero (writable
        // store) rather than master-abort all-ones.
        cfgSeed(1, 0x00800000u, 0x0022106Bu);
        cfgSeed(1, 0x00800008u, 0xFF000003u);
        for (u32 r = 0x04; r <= 0x3C; r += 4)
            if (r != 0x08)
                cfgSeed(1, 0x00800000u | r, 0);
        // KeyLargo's two OHCI USB functions: usb@18 / usb@19 on the f2
        // bus (one-hot bits 24/25). Apple id 0x0019, class 0x0C0310
        // (serial bus / USB / OHCI), INT pin A; BAR0 is a 4 KB memory
        // window with real sizing-mask behavior (cfgAccess below). The
        // Boot ROM carries ohci+usb-hid FCode; the USB Expert seeds the
        // boot-keyboard shim chain per controller it registers — no
        // controllers is the path Apple never booted.
        for (u32 f = 0; f < 2; ++f) {
            const u32 hot = 0x01000000u << f;
            cfgSeed(1, hot | 0x00u, 0x0019106Bu);
            cfgSeed(1, hot | 0x08u, 0x0C031001u);
            cfgSeed(1, hot | 0x3Cu, 0x00000100u);
            for (u32 r = 0x04; r <= 0x38; r += 4)
                if (r != 0x08)
                    cfgSeed(1, hot | r, 0);
        }
        for (u32 f = 0; f < 2; ++f) {
            ohci_[f].ram = ram_.data();
            ohci_[f].ramSize = static_cast<u32>(ram_.size());
        }
        // Uni-North's own host-bridge PCI functions at device 11 of
        // each bus ("11,UNI-N" in the ROM's slot names; the AGP-slot
        // probe consults its bridge): AGP = 106b:0020, internal 66MHz =
        // 106b:001E, 33MHz PCI = 106b:001F, all class 0x060000.
        {
            const u32 dev[3] = {0x0020106Bu, 0x001E106Bu, 0x001F106Bu};
            for (u32 b = 0; b < 3; ++b) {
                cfgSeed(b, 0x00000800u, dev[b]);
                cfgSeed(b, 0x00000808u, 0x06000000u);
                for (u32 r = 0x04; r <= 0x3C; r += 4)
                    if (r != 0x08)
                        cfgSeed(b, 0x00000800u | r, 0);
            }
        }
        // ATI Rage 128 Pro AGP at f0 device 16 (the AGP slot, "SLOT-A"
        // per the real card's dump): 1002:5046 'PF', class display.
        // BAR0 = 32 MB framebuffer aperture, BAR1 = 256 B I/O, BAR2 =
        // 16 KB register block, expansion ROM = 128 KB (the card's own
        // FCode image, attached from a file — never committed).
        cfgSeed(0, 0x00010000u, 0x50461002u);
        cfgSeed(0, 0x00010008u, 0x03000000u);
        cfgSeed(0, 0x0001003Cu, 0x00000100u);
        for (u32 r = 0x04; r <= 0x38; r += 4)
            if (r != 0x08)
                cfgSeed(0, 0x00010000u | r, 0);
        ataDma_.dmaBus = this;
        ataDma_.ata = &cd_;
    }

    bool attachAtiRom(const char* path)
    {
        FILE* f = fopen(path, "rb");
        if (!f)
            return false;
        fseek(f, 0, SEEK_END);
        const long n = ftell(f);
        fseek(f, 0, SEEK_SET);
        atiRom_.assign(static_cast<size_t>(n > 0 ? n : 0), 0);
        const bool ok =
            !atiRom_.empty() &&
            fread(atiRom_.data(), 1, atiRom_.size(), f) == atiRom_.size();
        fclose(f);
        if (!ok)
            atiRom_.clear();
        return ok;
    }
    R128Cell& ati() { return ati_; }
    bool atiPresent() const { return !atiRom_.empty(); }
    // Harness sequencing: hide the AGP function from config space until
    // this instruction count. OF picks its console (~228M) before the
    // injected probe (~245M) — with the card invisible at choice time
    // the console stays serial, and the FCode still runs at probe time.
    u64 atiVisibleAt = 0;
    // Memory write-watch bounds (inclusive); see write().
    u32 watchPa = 0;
    u32 watchPaEnd = 0;
    u32 watchHits = 0;

    void cfgSeed(u32 b, u32 latch, u32 nativeLeWord)
    {
        const u32 key = (b << 28) | (latch & 0x00FFFF00u) |
                        ((latch & 0xFFu) & 0xFCu);
        cfgSpace_[key] = nativeLeWord;
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

    struct RegWr {
        u64 at;
        u32 pa, val, pc;
    };
    const std::vector<RegWr>& sizeLog() const { return szLog_; }
    const std::vector<RegWr>& i2cLog() const { return i2cLog_; }

    // Uni-North host-bridge register block at 0xF8000000: a plain
    // word-register store, all-zero at power-on. Zero in HWINIT_STATE
    // (+0x70) is what tells the ROM this is a cold boot rather than a
    // sleep-wake (bit0) or a double boot (bit1) — served all-ones, the
    // ROM put the machine back to sleep. Individual registers earn real
    // semantics as the boot demands them.
    static constexpr u32 kUniNBase = 0xF8000000u;
    static constexpr u32 kUniNSize = 0x3000u;

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
    static constexpr u32 kMacIoBar = 0x80000000u; // OF's PCI BAR assignment
    static constexpr u32 kMacIoSize = 0x80000u;
    const std::map<u32, Acc>& macioLog() const { return klLog_; }
    std::vector<u32> macioOrder;

    // The VIA cell (+0x16000..+0x17FFF) routes to the PMU99 model.
    PmuVia& pmu() { return pmu_; }

    // OpenPIC at +0x40000 (the tree's interrupt-controller@40000).
    // Device level lines: ata-3@20000 = source 20, its DBDMA = 12;
    // the OHCI functions ride KeyLargo sources 27/28 (usb@18/usb@19).
    OpenPic& pic() { return pic_; }
    void syncIrqs()
    {
        pic_.setLine(19, hd_.irqLine()); // ata-4@1f000, interrupts 0x13
        pic_.setLine(20, cd_.irqLine());
        pic_.setLine(27, ohci_[0].irqLine());
        pic_.setLine(28, ohci_[1].irqLine());
    }

    OhciCell& ohci(u32 i) { return ohci_[i & 1u]; }
    void ohciTick(u64 tb)
    {
        ohci_[0].tick(tb);
        ohci_[1].tick(tb);
    }

    // DBDMA channel for ata-3@20000 at mac-io +0x8B00 — the OS ATA
    // driver's data path (task file stays PIO; INPUT descriptors pull
    // the CD's data phases straight into RAM).
    DbdmaChannel& ataDma() { return ataDma_; }

    // ATA cells (OF's tree: ata-4@1f000, ata-3@20000, ata-3@21000, each
    // with a /disk node). The CD lives on ata-3@20000 device 0 when an
    // ISO is attached; the other buses stay empty. Non-data register
    // traffic is logged bus-tagged.
    const std::vector<RegWr>& ataLog() const { return ataLog_; }
    bool attachCd(const char* path) { return cd_.attachIso(path); }
    bool attachHd(const char* path) { return hd_.attachDisk(path); }
    AtaCell& hd() { return hd_; }
    AtaCell& cd() { return cd_; }

    // SCC (+0x13000): MacRISC layout — ctrl B/A at +0x00/+0x20, data B/A
    // at +0x10/+0x30. Enough Z8530 to drain transmit (RR0 TX-empty) and
    // capture the ROM's serial console log verbatim. Bytes queued with
    // injectSerial() appear as channel-A receive data — a CR during the
    // firmware's 5-second escape window selects the serial console.
    const std::string& console() const { return console_; }
    void injectSerial(const std::string& s) { rxQueue_ += s; }

    // Uni-North "Keywest" I2C at 0xF8001000 — the DIMM SPD bus. Byte
    // registers at +3 of each 0x10-strided word, protocol as the ROM's
    // own polled driver at fff86a00 spells it out:
    //   +0x00 MODE   (sel<<4 | mode; 0xC = combined read, 0x8 = sub write)
    //   +0x10 CONTROL bit1 = launch address phase, bit0 = AAK
    //   +0x20 STATUS  bit1 = slave acked
    //   +0x30 ISR     bit1 = addr done, bit0 = data ready, bit2 = stop
    //                 done; write-1-to-clear, each clear advances the
    //                 transaction chain
    //   +0x50 ADDR (dev|1 = read), +0x60 SUBADDR, +0x70 DATA
    static constexpr u32 kI2cBase = 0xF8001000u;

    // Uni-North PCI config mechanism: three host bridges, each with an
    // address latch at fX800000 and a data window at fXc00000..+7
    // (X = 0 AGP, 2 mac-io/66MHz, 4 PCI slots). The latch value picks
    // {type-0 one-hot slot | type-1 bus/devfn} + register; every data
    // access is logged with the live latch so the ROM/OF teach us the
    // exact addressing they use. Config registers come from a sparse
    // store seeded with the devices the machine carries.
    const std::vector<RegWr>& cfgLog() const { return cfgLog_; }

    // Unclaimed + ROM-write traffic, keyed by physical address.
    const std::map<u32, Acc>& accessLog() const { return log_; }
    std::vector<u32> logOrder; // first-touch order

    const u32* pcRef = nullptr;
    const u64* stamp = nullptr;

    size_t ramBytes() const { return ram_.size(); }
    const std::vector<u8>& ram() const { return ram_; }

    // Snapshot of the whole machine minus the CPU: RAM, the boot flash and
    // the card's FCode image (verified rather than trusted on resume), the
    // Uni-North / KeyLargo register stores, both Keywest cells, the SCC
    // including the paced serial-injection cursor, PCI config space with
    // every latch and derived BAR, and each device cell in turn.
    void snapSave(SnapWriter& w) const;
    void snapLoad(SnapReader& r);

private:
    // During memory sizing the RAM controller exposes each DIMM slot in a
    // temporary wide-open decode (0x78000000..0x98000000 all aliasing the
    // DIMM under test modulo its size; the ROM probes with its rotating
    // "Mary" pattern at row-bit offsets and finds the size from where the
    // wrap aliasing begins). Slots whose SPD probe answered nothing are
    // never probed at all — absence is decided at the SPD stage.
    static constexpr u32 kSizeWin = 0x78000000u;
    static constexpr u32 kDimmBytes = 64u << 20;
    std::vector<RegWr> szLog_; // sizing-window probe traffic (val=data)
    std::vector<RegWr> i2cLog_; // one entry per launched transaction

    // mac-io answers at both its early hard decode and the OF-assigned
    // PCI BAR; the BAR wins over the overlapping sizing window.
    u32 macioOff(u32 pa) const
    {
        if (pa - kMacIoBase < kMacIoSize)
            return pa - kMacIoBase;
        if (pa - kMacIoBar < kMacIoSize)
            return pa - kMacIoBar;
        return 0xFFFFFFFFu;
    }

    u32 read(u32 pa, u32 len)
    {
        const u32 off = macioOff(pa);
        if (off != 0xFFFFFFFFu) {
            if (off >= 0x16000u && off < 0x18000u) {
                u32 v = 0;
                for (u32 k = 0; k < len; ++k)
                    v = (v << 8) |
                        pmu_.read(off - 0x16000u + k, stamp ? *stamp : 0);
                return v;
            }
            if (off >= 0x13000u && off < 0x14000u)
                return sccRead(off - 0x13000u);
            if (off >= 0x18000u && off < 0x18100u)
                return i2cRead(1, off - 0x18000u);
            if (off - 0x40000u < 0x40000u)
                return pic_.read(off - 0x40000u, len);
            if (off - 0x8B00u < 0x100u)
                return ataDma_.read(off - 0x8B00u, len);
            if (off - 0x1F000u < 0x3000u) {
                const bool isCd =
                    off - 0x20000u < 0x1000u && cd_.present();
                // Unpopulated channel: the ATA data lines float high on the
                // pull-ups, EXCEPT DD7 which the host pulls DOWN. DD7 is
                // the status register.s BSY bit, so an empty channel reads
                // 0x7F: BSY already clear, which is how a driver concludes
                // "no device" at once. Answering 0xFF leaves BSY stuck set
                // and every probe of an empty slot burns its full timeout
                // before the bus scan can move on.
                const bool isHd = off < 0x20000u && hd_.present();
                u32 v = isCd   ? cd_.read(off - 0x20000u, len)
                        : isHd ? hd_.read(off - 0x1F000u, len)
                               : ((~0u >> (32 - 8 * len)) & ~0x80u);
                if ((off & 0xFF0u) != 0) {
                    if (ataLog_.size() >= 6000)
                        ataLog_.erase(ataLog_.begin(),
                                      ataLog_.begin() + 3000);
                    ataLog_.push_back({stamp ? *stamp : 0, off | 1u, v,
                                       pcRef ? *pcRef : 0});
                }
                return v;
            }
            const u32 v = get(kl_.data() + off, len);
            klNote(kMacIoBase + off, 0, false);
            return v;
        }
        for (u32 f = 0; f < 2; ++f)
            if (ohciBar_[f] && pa - ohciBar_[f] < 0x1000u)
                return ohci_[f].read(pa - ohciBar_[f], len);
        if (atiRomBar_ > 1u && pa - (atiRomBar_ & ~1u) < 0x20000u &&
            !atiRom_.empty()) {
            const u32 ro = pa - (atiRomBar_ & ~1u);
            u32 v = 0;
            for (u32 k = 0; k < len; ++k)
                v = (v << 8) |
                    (ro + k < atiRom_.size() ? atiRom_[ro + k] : 0xFFu);
            return v;
        }
        if (atiRegBar_ && pa - atiRegBar_ < 0x4000u)
            return ati_.read(pa - atiRegBar_, len);
        if (atiFbBar_ && pa - atiFbBar_ < (32u << 20))
            return get(ati_.vram.data() + (pa - atiFbBar_), len);
        if (pa - kSizeWin < 0x20000000u) {
            const u32 v =
                get(ram_.data() + ((pa - kSizeWin) & (kDimmBytes - 1)), len);
            if (szLog_.size() < 4000)
                szLog_.push_back({stamp ? *stamp : 0, pa | 1u, v,
                                  pcRef ? *pcRef : 0});
            return v;
        }
        if (pa < ram_.size() && pa + len <= ram_.size())
            return get(ram_.data() + pa, len);
        if (pa >= kRomBase && pa - kRomBase + len <= rom_.size())
            return get(rom_.data() + (pa - kRomBase), len);
        if (pa >= kI2cBase && pa + len <= kI2cBase + 0x100u)
            return i2cRead(0, pa - kI2cBase);
        if (pa >= kUniNBase && pa + len <= kUniNBase + kUniNSize)
            return get(unin_ + (pa - kUniNBase), len);
        if (const int b = cfgBus(pa); b >= 0)
            return cfgAccess(static_cast<u32>(b), pa, 0, len, false);
        note(pa, 0, false);
        return len == 1 ? 0xFFu : len == 2 ? 0xFFFFu : 0xFFFFFFFFu;
    }

    // General memory write-watch: report the pc that writes a physical
    // address, rather than sampling for a change and losing the writer.
    // "Who wrote this?" is a whole class of question — which code clears
    // a field, which agent fills a table, whether anything ever touches
    // the drive queue — and answering it by hand-rolling a one-off watch
    // per address has been done three times on this machine already.
    void write(u32 pa, u32 v, u32 len)
    {
        if (watchPa) {
            const u32 hi = watchPaEnd ? watchPaEnd : watchPa;
            // A write of len bytes at pa covers [pa, pa+len)
            if (pa <= hi && pa + len > watchPa && watchHits < 200) {
                ++watchHits;
                printf("MEMW pa=%08x len=%u val=%08x pc=%08x @%llu\n", pa,
                       len, v, pcRef ? *pcRef : 0,
                       static_cast<unsigned long long>(stamp ? *stamp : 0));
                fflush(stdout);
            }
        }
        if (const u32 moff = macioOff(pa); moff != 0xFFFFFFFFu) {
            const u32 off = moff;
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
            if (off >= 0x18000u && off < 0x18100u) {
                i2cWrite(1, off - 0x18000u, static_cast<u8>(v));
                return;
            }
            if (off - 0x40000u < 0x40000u) {
                pic_.write(off - 0x40000u, v, len);
                return;
            }
            if (off - 0x8B00u < 0x100u) {
                ataDma_.write(off - 0x8B00u, v, len);
                return;
            }
            if (off - 0x1F000u < 0x3000u) {
                if ((off & 0xFF0u) != 0) {
                    if (ataLog_.size() >= 6000)
                        ataLog_.erase(ataLog_.begin(),
                                      ataLog_.begin() + 3000);
                    ataLog_.push_back({stamp ? *stamp : 0, off, v,
                                       pcRef ? *pcRef : 0});
                }
                if (off - 0x20000u < 0x1000u && cd_.present()) {
                    cd_.write(off - 0x20000u, v, len);
                    // a task-file write can open a fresh data phase:
                    // resume any standing DBDMA list
                    ataDma_.wake();
                } else if (off - 0x1F000u < 0x1000u && hd_.present()) {
                    hd_.write(off - 0x1F000u, v, len);
                }
                return;
            }
            put(kl_.data() + off, v, len);
            klNote(kMacIoBase + off, v, true);
            return;
        }
        for (u32 f = 0; f < 2; ++f)
            if (ohciBar_[f] && pa - ohciBar_[f] < 0x1000u) {
                ohci_[f].write(pa - ohciBar_[f], v, len);
                return;
            }
        if (atiRegBar_ && pa - atiRegBar_ < 0x4000u) {
            ati_.write(pa - atiRegBar_, v, len);
            return;
        }
        if (atiFbBar_ && pa - atiFbBar_ < (32u << 20)) {
            put(ati_.vram.data() + (pa - atiFbBar_), v, len);
            return;
        }
        if (pa - kSizeWin < 0x20000000u) {
            put(ram_.data() + ((pa - kSizeWin) & (kDimmBytes - 1)), v, len);
            if (szLog_.size() < 4000)
                szLog_.push_back({stamp ? *stamp : 0, pa, v,
                                  pcRef ? *pcRef : 0});
            return;
        }
        if (pa < ram_.size() && pa + len <= ram_.size()) {
            put(ram_.data() + pa, v, len);
            return;
        }
        if (pa >= kI2cBase && pa + len <= kI2cBase + 0x100u) {
            i2cWrite(0, pa - kI2cBase, static_cast<u8>(v));
            return;
        }
        if (pa >= kUniNBase && pa + len <= kUniNBase + kUniNSize) {
            put(unin_ + (pa - kUniNBase), v, len);
            if (uninLog_.size() < 4096)
                uninLog_.push_back({stamp ? *stamp : 0, pa, v,
                                    pcRef ? *pcRef : 0});
            return;
        }
        if (const int b = cfgBus(pa); b >= 0) {
            cfgAccess(static_cast<u32>(b), pa, v, len, true);
            return;
        }
        note(pa, v, true); // ROM/flash writes land here too, unapplied
    }

    // -1 if pa is not a config latch/window; else the bridge index 0/1/2
    // for f0/f2/f4.
    static int cfgBus(u32 pa)
    {
        const u32 top = pa >> 24;
        if (top != 0xF0u && top != 0xF2u && top != 0xF4u)
            return -1;
        const u32 sub = pa & 0x00FFFFFFu;
        if (sub - 0x800000u < 4u || sub - 0xC00000u < 8u)
            return static_cast<int>((top - 0xF0u) >> 1);
        return -1;
    }

    u32 cfgAccess(u32 b, u32 pa, u32 v, u32 len, bool wr)
    {
        const bool isData = (pa & 0x00FFFFFFu) >= 0xC00000u;
        if (!isData) { // address latch (LE device: stwbrx image arrives)
            if (wr) {
                u32 nat = 0;
                for (u32 k = 0; k < len; ++k)
                    nat |= ((v >> (8 * (len - 1 - k))) & 0xFFu)
                           << (8 * (((pa + k) & 3u)));
                cfgAddr_[b] = (cfgAddr_[b] & ~maskAt(pa, len)) |
                              (nat & maskAt(pa, len));
                return 0;
            }
            return swapLanes(cfgAddr_[b], pa, len);
        }
        const u32 reg = (cfgAddr_[b] & 0xFCu) | (pa & 7u);
        if (b == 0u && (cfgAddr_[b] & 0x0FFFFF00u) == 0x00010000u &&
            atiVisibleAt && stamp && *stamp < atiVisibleAt) {
            if (!wr)
                return len == 1 ? 0xFFu : len == 2 ? 0xFFFFu : 0xFFFFFFFFu;
            return 0; // absent card: master-abort both ways
        }
        const u32 key = (b << 28) | (cfgAddr_[b] & 0x00FFFF00u) |
                        ((cfgAddr_[b] & 0xFFu) & 0xFCu) | (pa & 7u);
        u32 out = 0xFFFFFFFFu;
        auto it = cfgSpace_.find(key & ~3u);
        if (it != cfgSpace_.end()) {
            // stored native-LE word; serve the requested lanes
            out = it->second;
        }
        if (wr) {
            u32 word = it != cfgSpace_.end() ? it->second : 0u;
            for (u32 k = 0; k < len; ++k) {
                const u32 lane = (pa + k) & 3u;
                word = (word & ~(0xFFu << (8 * lane))) |
                       (((v >> (8 * (len - 1 - k))) & 0xFFu) << (8 * lane));
            }
            // OHCI BAR0 (usb@18/19 reg 0x10): a real 4 KB memory BAR —
            // all-ones sizing writes read back the size mask, address
            // writes relocate the register cell. BARs 1-5 and the
            // expansion-ROM BAR are hardwired zero (a single-BAR
            // function; a writable store here grows phantom BARs).
            for (u32 f = 0; f < 2; ++f) {
                if (b == 1u && (cfgAddr_[b] & 0x0FFFFF00u) ==
                                   (0x01000000u << f)) {
                    if (reg == 0x10u) {
                        word &= 0xFFFFF000u;
                        ohciBar_[f] =
                            (word != 0xFFFFF000u) ? word : ohciBar_[f];
                    } else if ((reg >= 0x14u && reg <= 0x2Cu) ||
                               reg == 0x30u) {
                        word = 0;
                    }
                }
            }
            // ATI Rage 128 (f0 device 16): FB aperture 32 MB, I/O BAR
            // 256 B, register BAR 16 KB, expansion ROM 128 KB when an
            // FCode image is attached (absent card ROM reads zero).
            if (b == 0u && (cfgAddr_[b] & 0x0FFFFF00u) == 0x00010000u) {
                switch (reg) {
                case 0x10u:
                    word &= 0xFE000000u;
                    atiFbBar_ = (word != 0xFE000000u) ? word : atiFbBar_;
                    break;
                case 0x14u:
                    word = (word & 0xFFFFFF00u) | 1u;
                    break;
                case 0x18u:
                    word &= 0xFFFFC000u;
                    atiRegBar_ =
                        (word != 0xFFFFC000u) ? word : atiRegBar_;
                    break;
                case 0x30u:
                    if (atiRom_.empty())
                        word = 0;
                    else {
                        const u32 en = word & 1u;
                        word = (word & 0xFFFE0000u) | en;
                        atiRomBar_ = (word & 0xFFFE0000u) != 0xFFFE0000u
                                         ? word
                                         : atiRomBar_;
                    }
                    break;
                default:
                    if (reg >= 0x1Cu && reg <= 0x2Cu)
                        word = 0;
                    break;
                }
            }
            cfgSpace_[key & ~3u] = word;
        }
        if (cfgLog_.size() < 2048)
            cfgLog_.push_back({stamp ? *stamp : 0,
                               (b << 28) | (cfgAddr_[b] & 0x00FFFFFFu),
                               wr ? v : out, (pcRef ? *pcRef : 0) |
                                                 (wr ? 1u : 0u)});
        (void)reg;
        if (wr)
            return 0;
        // reads assemble from the stored LE word's lanes, BE-composed
        u32 r = 0;
        for (u32 k = 0; k < len; ++k) {
            const u32 lane = (pa + k) & 3u;
            r = (r << 8) | ((out >> (8 * lane)) & 0xFFu);
        }
        return r;
    }

    static u32 maskAt(u32 pa, u32 len)
    {
        u32 m = 0;
        for (u32 k = 0; k < len; ++k)
            m |= 0xFFu << (8 * ((pa + k) & 3u));
        return m;
    }
    static u32 swapLanes(u32 word, u32 pa, u32 len)
    {
        u32 r = 0;
        for (u32 k = 0; k < len; ++k)
            r = (r << 8) | ((word >> (8 * ((pa + k) & 3u))) & 0xFFu);
        return r;
    }

    // Keywest engine (two cells: Uni-North's SPD bus, and KeyLargo's own
    // at mac-io +0x18000 — the codec/sensor bus, no slaves modeled).
    // A slave that isn't populated simply never acks (STATUS bit1 stays
    // clear, ISR auto-advances to stop), which is the not-present path
    // both the ROM and OF handle.
    struct Keywest {
        u8 mode = 0, ctrl = 0, isr = 0, addr = 0, sub = 0;
        bool acked = false;
    };
    u32 i2cRead(u32 n, u32 off)
    {
        Keywest& kw = i2c_[n];
        if ((off & 0xFu) != (n == 0 ? 3u : 0u))
            return 0; // uni-n cell wires its bytes at lane 3, mac-io at 0

        switch (off >> 4) {
        case 0: return kw.mode;
        case 1: return kw.ctrl;
        case 2: return kw.acked ? 0x02u : 0x00u;
        case 3: return kw.isr;
        case 5: return kw.addr;
        case 6: return kw.sub;
        case 7: return kw.acked ? slaveByte(n) : 0xFFu;
        default: return 0;
        }
    }
    void i2cWrite(u32 n, u32 off, u8 v)
    {
        Keywest& kw = i2c_[n];
        if (n == 1 && i2cLog_.size() < 512)
            i2cLog_.push_back({stamp ? *stamp : 0,
                               0x01000000u | (off << 8) | v, 0,
                               pcRef ? *pcRef : 0});
        if ((off & 0xFu) != (n == 0 ? 3u : 0u))
            return;
        switch (off >> 4) {
        case 0: kw.mode = v; break;
        case 1:
            kw.ctrl = v;
            if ((v & 0x02u) && !(v & 0x01u)) { // launch address phase
                kw.acked = slavePresent(n);    // (|1 = AAK continuation)
                kw.isr |= 0x02u;
                if (i2cLog_.size() < 512)
                    i2cLog_.push_back({stamp ? *stamp : 0,
                                       (n << 16) | (u32(kw.addr) << 8) |
                                           kw.sub,
                                       kw.acked ? slaveByte(n)
                                                : 0xFFFFFFFFu,
                                       pcRef ? *pcRef : 0});
            }
            break;
        case 3: // W1C; each clear advances the polled chain
            if ((v & 0x02u) && (kw.isr & 0x02u))
                kw.isr |= kw.acked ? 0x01u  // acked: data byte ready
                                   : 0x04u; // nacked: auto-stop done
            if (v & 0x01u)
                kw.isr |= 0x04u; // data consumed -> stop completes
            kw.isr &= static_cast<u8>(~v);
            break;
        case 5: kw.addr = v; break;
        case 6: kw.sub = v; break;
        default: break;
        }
    }
    bool slavePresent(u32 n) const
    {
        if (n != 0)
            return false; // mac-io cell: nothing on the bus yet
        return (i2c_[0].addr & 0xFEu) == 0xA0u; // one DIMM, slot 1
    }
    u8 slaveByte(u32 n) const { return n == 0 ? spdByte() : 0xFFu; }
    u8 spdByte() const;

    // Z8530, just enough for a polled console: pointer-register protocol
    // per channel, RR0 reports TX-empty always, data writes append to the
    // captured console text.
    u32 sccRead(u32 off)
    {
        const u32 ch = (off >> 5) & 1u; // 0 = B (+0x00), 1 = A (+0x20)
        // Injected input is paced like a real serial line: one byte
        // becomes visible every few million instructions, matching the
        // console editor's per-keystroke processing. Un-paced delivery
        // overflows the firmware's small input ring and drops the tail.
        const bool rxReady = ch == 1 && !rxQueue_.empty() && stamp &&
                             *stamp >= rxNextAt_;
        if (off & 0x10u) { // data: serve queued RX on channel A
            if (rxReady) {
                const u8 b = static_cast<u8>(rxQueue_.front());
                rxQueue_.erase(rxQueue_.begin());
                rxNextAt_ = *stamp + 3000000ull;
                return b;
            }
            return 0;
        }
        const u32 r = sccPtr_[ch];
        sccPtr_[ch] = 0;
        if (r == 0) // RR0: TX empty, RX-avail as pacing allows
            return 0x04u | (rxReady ? 0x01u : 0x00u);
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
    std::string console_, rxQueue_;
    u64 rxNextAt_ = 0;
    Keywest i2c_[2]; // 0 = Uni-North SPD bus, 1 = mac-io cell
    u32 cfgAddr_[3] = {0, 0, 0};
    std::map<u32, u32> cfgSpace_; // (bus<<28|latch&~3) -> native-LE word
    std::vector<RegWr> cfgLog_;
    std::vector<RegWr> ataLog_;
    AtaCell cd_;
    AtaCell hd_; // ata-4@1f000: the internal drive a Sawtooth boots from
    OpenPic pic_;
    OhciCell ohci_[2];
    u32 ohciBar_[2] = {0, 0}; // OF/OS-assigned BAR0 per function
    R128Cell ati_;
    DbdmaChannel ataDma_;
    std::vector<u8> atiRom_;
    u32 atiFbBar_ = 0, atiRegBar_ = 0, atiRomBar_ = 0;
};

} // namespace opm
