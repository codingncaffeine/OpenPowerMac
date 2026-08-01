#pragma once
#include "opm/ata.hpp"
#include "opm/awacs.hpp"
#include "opm/dbdma.hpp"
#include "opm/bus.hpp"
#include "opm/ohci.hpp"
#include "opm/r128.hpp"
#include "opm/openpic.hpp"
#include "opm/pmu.hpp"
#include "opm/types.hpp"
#include <cstring>

#include <cstdio>
#include <map>
#include <string>
#include <vector>

namespace opm {

struct SnapWriter;
struct SnapReader;

// --no-agp-caps: build the machine with the PRE-2026-08-01 config space —
// no AGP capability chains on the Uni-N AGP host function or the card.
// A diagnostic A/B control in the --no-ati-2d mould, kept because the caps
// are now load-bearing (they are what makes `.AGP` install, the ARM
// initialise, and the accelerator draw) and "with/without the machine
// change" should stay a one-flag question. ⚠ It exists because of a
// GOOSE CHASE worth remembering: a question-mark boot was blamed on these
// seeds through a whole bisect — with the flag, without the flag, the
// stall never moved — and the actual cause was the boot CD missing from
// ~/Downloads ("cd attach FAILED" had been in the log from the first run;
// hd.img alone was never bootable). A null result without a positive
// control is not a finding. Set before construction.
inline bool gSkipAgpCaps = false;

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
        // Digest of the boot flash AS LOADED, before Open Firmware writes
        // its NVRAM partition into it. A snapshot has to be able to say
        // "this is a different ROM" without saying it about its own NVRAM
        // edits, and the live image cannot answer that question.
        romBase_ = 0xcbf29ce484222325ull;
        for (u8 b : rom_) {
            romBase_ ^= b;
            romBase_ *= 0x100000001b3ull;
        }
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
        cfgSeed(1, kMacIoSel | 0x00u, 0x0022106Bu);
        cfgSeed(1, kMacIoSel | 0x08u, 0xFF000003u);
        for (u32 r = 0x04; r <= 0x3C; r += 4)
            if (r != 0x08)
                cfgSeed(1, kMacIoSel | r, 0);
        // KeyLargo's two OHCI USB functions. They live on the SECONDARY bus
        // of the PCI-to-PCI bridge above, as usb@8 / usb@9 — not on the f2
        // bus itself. Open Firmware reaches them with type-1 configuration
        // cycles, latch (bus << 16) | (devfn << 8) | reg | 1, and it probes
        // exactly devfn 0x40 and 0x48 there because the ROM's own
        // built-in-names list for this bus reads
        // "7,MAC-IO,8,USB0,9,USB1,10,FireWire,11,Ethernet".
        // Apple id 0x0019, class 0x0C0310 (serial bus / USB / OHCI), INT
        // pin A; BAR0 is a 4 KB memory window with real sizing-mask
        // behaviour (cfgAccess below). The Boot ROM carries ohci+usb-hid
        // FCode; the USB Expert seeds the boot-keyboard shim chain per
        // controller it registers — no controllers is the path Apple never
        // booted, and is what put the sad Mac on the screen.
        for (u32 f = 0; f < 2; ++f) {
            const u32 sel = kOhciSel[f];
            cfgSeed(1, sel | 0x00u, 0x0019106Bu);
            cfgSeed(1, sel | 0x08u, 0x0C031001u);
            cfgSeed(1, sel | 0x3Cu, 0x00000100u);
            for (u32 r = 0x04; r <= 0x38; r += 4)
                if (r != 0x08)
                    cfgSeed(1, sel | r, 0);
        }
        for (u32 f = 0; f < 2; ++f) {
            ohci_[f].ram = ram_.data();
            ohci_[f].ramSize = static_cast<u32>(ram_.size());
        }
        // usb@8 carries the boot keyboard, usb@9 the boot mouse. Both cells
        // reported a keyboard before, which is two keyboards and no pointer:
        // a machine that cannot dismiss a dialog.
        ohci_[0].setHid(OhciCell::Hid::Keyboard);
        ohci_[1].setHid(OhciCell::Hid::Mouse);
        // The PCI-to-PCI bridge at f2 device 13 (one-hot bit 13). A real
        // Sawtooth carries its built-in devices on this bridge's SECONDARY
        // bus -- pci-bridge@d/{mac-io@7, usb@8, usb@9, firewire@a,
        // ethernet@b} -- and the boot ROM tests for it directly: the payload
        // at 0x00200920 reads config latch 0x6808 (device 13, register 8),
        // compares the class to 0x00060400, and stores the answer to
        // [startvec+0x74], which Open Firmware reads back as `mlb-bridge?`.
        // With nothing there the read is all-ones, the flag is false, and
        // bridge 1's probe-slots skips the one call that would create that
        // secondary bus -- which is why no OHCI controller is ever
        // enumerated and the USB shim later calls through a null pointer.
        // Header type 1 (register 0x0C byte 2) marks the bridge layout.
        cfgSeed(1, 0x00002000u, 0x00261011u); // DEC 21154, 1011:0026
        cfgSeed(1, 0x00002008u, 0x06040002u); // class 060400, rev 2
        cfgSeed(1, 0x0000200Cu, 0x00010000u); // header type 1
        for (u32 r = 0x04; r <= 0x3C; r += 4)
            if (r != 0x08 && r != 0x0C)
                cfgSeed(1, 0x00002000u | r, 0);
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
            // The AGP function is not just a host bridge with a different
            // device id: Mac OS's driver loader reads its CAPABILITIES
            // POINTER (+0x34) exactly once, at ~6.6 G, and decides whether
            // this machine has AGP at all. Measured 2026-08-01: five reads
            // — +34 (got 0), +00 x2, +10 x2 — then the device is never
            // touched again, the .AGP ndrv is never attached to the
            // uni-north-agp node Open Firmware built, and the ATI Resource
            // Manager's OpenDriver(".AGP") returns fnfErr 1.3 G later,
            // which is the whole desktop hang. So: a real AGP 2.0
            // capability chain (status bit 4 -> cap at +0x80, rates
            // 1x/2x/4x, SBA, RQ depth 31), and the Uni-N GART block
            // (+0x8C..+0x98, base/AGP-base/control/status) seeded ZERO so
            // the driver's reads see a bridge at reset rather than the
            // 0xFFFFFFFF master-abort an unseeded register answers.
            if (!gSkipAgpCaps) {
                cfgSeed(0, 0x00000804u, 0x00100000u); // status: cap list
                cfgSeed(0, 0x00000834u, 0x00000080u); // cap ptr -> +0x80
                cfgSeed(0, 0x00000880u, 0x00200002u); // AGP cap, rev 2.0
                cfgSeed(0, 0x00000884u, 0x1F000207u); // AGP status: RQ 31,
                                                      // SBA, 4x/2x/1x
                cfgSeed(0, 0x00000888u, 0);           // AGP command
                cfgSeed(0, 0x0000088Cu, 0);           // GART base
                cfgSeed(0, 0x00000890u, 0);           // AGP aperture base
                cfgSeed(0, 0x00000894u, 0);           // GART control
                cfgSeed(0, 0x00000898u, 0);           // internal status
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
        // The card's own AGP capability, at 0x50 exactly as on the real
        // 'PF': the ARM reads the MMIO mirror of PCI config 0x58
        // (AGP_COMMAND) at 0x0F58 — measured session 31 — which pins the
        // capability base at 0x50. The .AGP driver walks the MASTER's
        // capability chain too (dotagp.pef 0x8c4, second FindAGPCapability
        // call), and its walk does not zero-check the pointer: a bare
        // pointer of 0 lands on the vendor byte, 0x02, which reads as a
        // false AGP capability at offset 0 and feeds the driver the
        // command register as AGP status. PM capability chained at 0x5C.
        if (!gSkipAgpCaps) {
            cfgSeed(0, 0x00010004u, 0x00100000u); // status: cap list
            cfgSeed(0, 0x00010034u, 0x00000050u); // cap ptr -> +0x50
            cfgSeed(0, 0x00010050u, 0x00205C02u); // AGP cap, rev 2.0,
                                                  // -> PM at 0x5C
            cfgSeed(0, 0x00010054u, 0x1F000207u); // AGP status: RQ 31,
                                                  // SBA, 4x/2x/1x
            cfgSeed(0, 0x00010058u, 0);           // AGP command
            cfgSeed(0, 0x0001005Cu, 0x00010001u); // PM cap v1.0, end of
                                                  // chain — as on the
                                                  // real 'PF'
            cfgSeed(0, 0x00010060u, 0);           // PMCSR (D0)
        }
        ataDma_.dmaBus = this;
        ataDma_.dev = &cd_;
        hdDma_.dmaBus = this;
        hdDma_.dev = &hd_;
        // The two audio channels share the engine the ATA channels proved,
        // and both ends of the codec are the same cell: +0x8800 hands it
        // samples, +0x8900 asks it for them.
        sndOut_.dmaBus = sndIn_.dmaBus = this;
        sndOut_.dev = sndIn_.dev = &snd_;
        // ⭐ THE CHANNELS THIS MACHINE HAS BUT DRIVES NOTHING WITH — 0x8000 to
        // 0x87FF, which the CHRP map assigns to SCSI0, floppy, ethernet tx/rx
        // and the two SCC ports' tx/rx pairs. They carry no device here, but
        // they are REAL CHANNELS and their control registers have to behave.
        //
        // ⛔ Leaving them to the generic mac-io register store is what hung the
        // Mac OS 9 boot during extension loading. A driver shuts a channel
        // down by writing FLUSH (0x2000) to ChannelControl and then spinning
        // on ChannelStatus until the hardware clears it — measured, at
        // 006772bc, polling +0x8700 (channel 7, SCC receive B) forever. A dumb
        // register store never clears anything, so the wait never ends. The
        // engine already clears FLUSH the instant it is asked; it simply was
        // never reached. Same code for every channel, so there is no
        // second-class one to get subtly wrong.
        for (DbdmaChannel& ch : dmaGen_)
            ch.dmaBus = this;
        // The machine's clock, handed to the cells that model a DURATION.
        // Wired here rather than by the consumer for the reason setStamp
        // exists: a per-consumer wiring list is how the app ended up with a
        // hard disk that never completed a command.
        //
        // ⚠ It is the MACHINE's clock and not the processor's. The two differ
        // by whatever a batched run loop is holding, and only this one is
        // refreshed on the path a device access takes — see noteNow(). The PMU
        // used to be handed &cpu.st.tb by each front end and would have read a
        // batch stale.
        cd_.tbRef = hd_.tbRef = &nowTb_;
        pmu_.tbRef = &nowTb_;
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
    // …and hide it again for a window. The FCode runs during Open
    // Firmware's PCI probe and builds the display node with the
    // properties Mac OS needs; the console choice happens later, and a
    // display present at that moment takes the console with it into a
    // framebuffer terminal that never paints. Hiding the card across the
    // choice keeps the node and leaves the console on serial.
    u64 atiHideFrom = 0, atiHideTo = 0;
    // Device-service gate; see serviceDevices(). Not machine state — a
    // cache of "when could this answer change" — but it is snapshotted all
    // the same, because a resume that starts with a stale deadline would
    // withhold an interrupt until the next guest register access.
    u64 devGen_ = 0, devGenSeen_ = ~0ull;
    u64 devDueTb_ = 0, devServices_ = 0;
    bool cpuIrq_ = false;
    // KeyLargo timer: the timebase at the last reload and the value reloaded.
    u64 klTimerTb_ = 0, klTimerVal_ = 0;
    // ⏱ THE MACHINE'S CURRENT TIMEBASE, and the one clock every device reads.
    // It lives here because a device register access has no clock of its own:
    // it happens inside a guest load or store, with the service gate shut, and
    // whatever it needs to know about "now" it has to find already written
    // down. The KeyLargo timer is read exactly that way and it is what the
    // guest calibrates its entire clock against, so this store is
    // unconditional and stays that way (session 26).
    u64 nowTb_ = 0;
    // DIAGNOSTIC (--no-dev-gate): service every device on every instruction,
    // the way the machine did before the gate existed. A gate is a claim —
    // "nothing can have changed" — and a claim about timing needs a control
    // run to be settled rather than argued. Anything that behaves differently
    // with this set is the gate's fault, and nothing else's.
    bool devGateOff = false;

    // Memory write-watch bounds (inclusive); see write().
    u32 watchPa = 0;
    u32 watchPaEnd = 0;
    u32 watchHits = 0;
    // --wmap FROM TO: guest writes bucketed by megabyte of physical
    // address, so "where is the OS painting" has an answer.
    u64 wmapFrom = 0, wmapTo = 0;
    std::map<u32, u64> wmap;
    // --wmap-pc BUCKET: which instruction writes into that megabyte. Turns
    // "the desktop is at 0x02100000" into "this pc put it there".
    u32 wmapPcBucket = 0;
    bool wmapPcSet = false;
    std::map<u32, u64> wmapPcs;

    void cfgSeed(u32 b, u32 latch, u32 nativeLeWord)
    {
        const u32 key = (b << 28) | (latch & 0x00FFFF00u) |
                        ((latch & 0xFFu) & 0xFCu);
        cfgSpace_[key] = nativeLeWord;
    }

    // The Uni-N GART base as the .AGP driver programmed it: the AGP host
    // function's config register +0x8C (bus 0, dev-11 idsel latch 0x800).
    // Zero until the driver runs — the card-side GART walk declines on it.
    u32 agpGartBase() const
    {
        auto it = cfgSpace_.find(0x88Cu);
        return it == cfgSpace_.end() ? 0u : it->second;
    }
    // The AGP APERTURE base (+0x90): the PCI address where AGP space
    // starts. The GART indexes aperture-RELATIVE pages, so the card-side
    // walk must subtract this before indexing. On a 64 MB machine the
    // driver programmed 0 and the subtraction was invisible; on a 1.5 GB
    // machine the aperture sits above RAM and an unsubtracted address
    // indexed garbage far past the table.
    u32 agpAperBase() const
    {
        auto it = cfgSpace_.find(0x890u);
        return it == cfgSpace_.end() ? 0u : it->second;
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
        // Access WIDTH. The ATA traffic log printed `val & 0xFF` and no
        // width at all, so a 32-bit write of 0x00000180 and a byte write of
        // 0x80 to the same cell register were the same line — and the
        // question at +0x200 is exactly which of those it is.
        u32 len = 1;
    };
    const std::vector<RegWr>& sizeLog() const { return szLog_; }
    const std::vector<RegWr>& i2cLog() const { return i2cLog_; }

    // The processor module's I2C cache descriptor at slave 0xAC. ON, because
    // a Power Mac G4 AGP has that EEPROM and 1 MB of backside L2, and a
    // machine that answers it is the honest one. `--no-cpu-cache-rom`
    // restores the silent module.
    //
    // With it the boot ROM detects the cache, builds L2CR by hand
    // (0x38000000 = L2SIZ 1 MB, L2CLK ÷2), runs the um7400 §3.7.7 test and
    // sets L2E (0xb8000000) — and, at fff815ac, writes the size into the
    // configuration block the firmware hands the OS: **[0x00003010] =
    // 0x00100000**. That word is what closes the cache dialog. Measured, one
    // hop at a time: HWInit reads it (pc 00a57094/00a57408/00a57a50) and puts
    // it in NKProcessorInfo+0x34 at PA 0x03ffffb4; the NanoKernel copies the
    // record to KDP+0xF60 (pc 00f106bc), so ProcessorL2DSize reads
    // **0x00100000** instead of 0 and `Cache.s`'s `beq cacheFailAbsentL2CR`
    // is not taken.
    //
    // ⛔ Open Firmware's `l2-cache` word is a DIFFERENT chain and NOT this
    // one: it reads startvec+0xE8 from [0x00100004] and CASEs 3/5/6/7/8 —
    // legacy cache-CARD codes. Nothing writes that word even with the L2
    // fully running, so no `l2-cache` node is built, and the dialog goes away
    // regardless. Two sessions were spent treating it as the missing link.
    bool cpuModuleRom = true;

    // Uni-North host-bridge register block at 0xF8000000: a plain
    // word-register store, all-zero at power-on. Zero in HWINIT_STATE
    // (+0x70) is what tells the ROM this is a cold boot rather than a
    // sleep-wake (bit0) or a double boot (bit1) — served all-ones, the
    // ROM put the machine back to sleep. Individual registers earn real
    // semantics as the boot demands them.
    static constexpr u32 kUniNBase = 0xF8000000u;
    static constexpr u32 kUniNSize = 0x3000u;

    const std::vector<RegWr>& uninLog() const { return uninLog_; }
    const std::vector<RegWr>& flashLog() const { return flashLog_; }
    const std::vector<RegWr>& atiIoLog() const { return atiIoLog_; }
    // SCC access census. sccRead/sccWrite bypass the first-touch log, so
    // "did Open Firmware open the serial console at all" had no answer:
    // zero captured output is equally consistent with a console bound to
    // the screen and with a machine that never got that far.
    u64 sccReads = 0, sccWrites = 0;
    // Which SCC register the firmware polls, by offset. "It is reading the
    // SCC" is not the same claim as "it is reading channel A's receive
    // register", and only the second one means an injected byte can land.
    std::map<u32, u64> sccOffHist;
    const std::vector<u8>& flash() const { return rom_; }

    // A system reset, as the PMU's 0xD0 command performs it. The ASICs
    // come back at power-on values; RAM, the attached media and the flash
    // do not, which is what makes a warm restart a restart rather than a
    // new machine. The boot ROM writes 2 to Uni-North +0x70 on its way up
    // and, on the next pass, reads it back and parks forever at fff03600
    // if bit 1 is still set — so a reset that leaves this register alone
    // produces a machine that resets once and then never boots again.
    void systemReset()
    {
        memset(unin_, 0, kUniNSize);
        // The processor restart takes the timebase back to zero, so the
        // timer's reload point has to come with it or the counter would sit
        // frozen (clamped) until the guest happened to write it.
        klTimerTb_ = klTimerVal_ = nowTb_ = 0;
    }

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
    // KeyLargo's free-running timer, mac-io +0x15000 — and THE MACHINE'S
    // CLOCK IS CALIBRATED AGAINST IT. Open Firmware publishes the node
    // (`timer`, compatible "keylargo-timer", reg 0x15000/0x1000,
    // clock-frequency 18,432,000) whether or not anything answers there, and
    // the Mac OS ROM's HWInit times the processor against it: at 0x0020f55c
    // it zeroes the 64-bit counter at +0x38/+0x3c with `stwbrx` (the register
    // file is little-endian), reloads the decrementer with 0x000FFFFF, spins
    // until it passes zero, and reads the counter back with `lwbrx`.
    //
    // With nothing there the counter read back ZERO, and the ROM's 64-bit
    // divide helper at 0x0020f5c0 returns -1 for a zero divisor — so
    // 0xFFFFFFFF landed in NKProcessorInfo.BusClockRateHz (+0x08) and its /4,
    // 0x3FFFFFFF, in DecClockRateHz (+0x0c). DriverServicesLib then converted
    // EVERY Duration at 2^30 Hz instead of 25 MHz: the 60.15 Hz tick chain
    // asked for 16,626 µs and was handed 16,626 × 0x3FFFFFFF / 10^6 =
    // 17,852,031 timebase ticks — 43x too long, which is the whole reason the
    // guest's clock ran at 1.4 Hz. One unanswered register, every timed thing
    // in the OS.
    //
    // The count is derived from the TIMEBASE, like the VIA's and the USB
    // frame clock, so --fast-tb scales it with everything else and the guest
    // measures the ratio this machine actually runs at. 18,432,000 counts per
    // 25,000,000 timebase ticks is 2304/3125 exactly.
    static constexpr u32 kKlTimerLo = 0x15038u; // +0x3c holds the high word
    static constexpr u64 kKlTimerNum = 2304ull, kKlTimerDen = 3125ull;
    // ⚠⚠ ON BY DEFAULT, AND THE DEFAULT IS LOAD-BEARING — IT IS PAIRED WITH
    // THE PACING. This is the CONSTRUCTOR default, which is what the capi and
    // therefore the app get, so g4run's switch has to agree with it.
    //
    // Answering the timer is correct and it fixes the guest's clock. It was
    // shipped OFF for one session because a 12.95 B cold boot with it on
    // stopped dead at 2.5 G instructions — one distinct scanline of 480, no
    // welcome screen, 287 disk commands against a baseline 292 — and the
    // cause was read as a device timeout expiring. It is not.
    //
    // ⭐ THE CAUSE IS AN INTERRUPT LIVELOCK, AND IT IS ARITHMETIC. Mac OS
    // spends about 32,000 EMULATED INSTRUCTIONS on each 60 Hz tick: the 68K
    // VBL chain, the Time Manager, CrsrTask. --fast-tb n hands it
    // 416,666 x 4/(1+n) instructions per tick to do that in. At n = 60 that
    // is 27,300 — LESS THAN THE WORK COSTS — so the machine services ticks
    // back to back at 2.2% behind nominal, and the boot never advances again.
    // Measured, one variable, cold boots scored on distinct scanlines:
    // n = 60 stops at 2.5 G with 1 scanline; n = 30, 15, 8, 4 and 1 all reach
    // the desktop with 462 of 480, 292 disk commands and ati paint 1,261,505
    // — the recorded baseline, item for item. At n = 4 the timebase runs at
    // 26.57 MHz, 1.06x real, and the guest's clock reads 62.7 Ticks per HOST
    // second against a real 60.
    //
    // So the timer goes on together with a --fast-tb that leaves room for the
    // guest's own periodic work — or, better, with opm_set_realtime, which
    // sizes the interval from the host clock and does not have to guess.
    bool klTimerOn = true;
    // KeyLargo GPIO block: 0x30 bytes at mac-io +0x50 (QEMU hw/misc/macio).
    static constexpr u32 kGpioBase = 0x50u;
    static constexpr u32 kGpioSize = 0x30u;
    static constexpr u8 kGpioOutData = 1u;
    static constexpr u8 kGpioInData = 2u;
    static constexpr u8 kGpioOutEnable = 4u;
    static constexpr u32 kMacIoBar = 0x80000000u; // OF's PCI BAR assignment
    static constexpr u32 kMacIoSize = 0x80000u;
    // Type-1 configuration selectors for the devices on the PCI-to-PCI
    // bridge's secondary bus: (bus 1 << 16) | (devfn << 8). usb@8 is
    // devfn 0x40 and usb@9 is devfn 0x48, which is what Open Firmware
    // actually puts on the wire once `mlb-bridge?` is true. The low type
    // bit is dropped by the config key, and no one-hot type-0 latch can
    // collide with these (they have three device bits set, not one).
    static constexpr u32 kOhciSel[2] = {0x00014000u, 0x00014800u};
    // mac-io is devfn 0x38 = device 7 on that same secondary bus. Leaving it
    // at one-hot 23 on the primary side while USB moved was not a halfway
    // house, it was a broken machine: the Mac OS ROM's FCode looks a unit's
    // interrupt up in the bridge's interrupt-map, the ROM builds that map for
    // devices 7..11, and a mac-io that is not at 7 makes the lookup fail with
    // "MacOS: unit-interrupt-specifier not found in map" and stops the boot.
    static constexpr u32 kMacIoSel = 0x00013800u;
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
        pic_.setLine(11, hdDma_.irqLine()); // ata-4's DBDMA (reg 8a00)
        pic_.setLine(19, hd_.irqLine()); // ata-4@1f000, interrupts 0x13
        pic_.setLine(20, cd_.irqLine());
        // The audio DBDMA channels. A mac-io channel at +0x8000 + N*0x100
        // raises source N+1 — the two ATA channels are the recorded pairs
        // (0x8A00 = channel 10 = source 11, 0x8B00 = channel 11 = source
        // 12), so audio out is channel 8 = SOURCE 9 and audio in is channel
        // 9 = SOURCE 10. Session 25's census of sources the OS enables and
        // this machine never asserts named 9, 10, 12, 22, 24, 25 and 29:
        // 9 and 10 are these two, and 12 is the CD's, which reaches the
        // driver through the cell's +0x300 register instead.
        //
        // ⭐⭐ IT IS A PULSE, NOT A LEVEL, AND THE DIFFERENCE WEDGED THE
        // MACHINE. Mac OS's sound driver arms a self-sustaining ring —
        // INPUT_MORE, STORE_QUAD (interrupt always, writing its own
        // ready-flag word), INPUT_MORE, STORE_QUAD, NOP branching back — and
        // it never touches the channel again. There is nothing for a handler
        // to clear, and the OS reads the flag word the STORE_QUAD wrote, not
        // the channel. Held as a level, the first interrupt was taken and
        // never EOI'd: the run ended `inService=10 cpuLine=0`, every device
        // interrupt in the machine blocked behind it, the OS had defensively
        // MASKED source 9, and the guest's 60 Hz chain was running at 7.3 Hz
        // (Ticks 4,890 against the control's 39,731). Both OpenPIC sources
        // are edge-sensitive here — `vp=00040007`, sense bit 22 clear — so
        // raising and lowering in the same breath is the honest shape.
        if (soundOn) {
            pulse(9, sndOut_);
            pulse(10, sndIn_);
        }
        pic_.setLine(27, ohci_[0].irqLine());
        pic_.setLine(28, ohci_[1].irqLine());
        // The AGP display's vertical blank — what runs the slot-VBL
        // dispatcher, which is what moves the mouse pointer.
        //
        // ⚠ THE SOURCE NUMBER IS `AAPL,interrupt-vectors`, NOT `AAPL,interrupts`.
        // Both live on the card's Name Registry node and they are different
        // numbers; reading the wrong one costs a cold boot. The devices already
        // wired above settle it: usb carries vectors 0x1b/0x1c and interrupts
        // 3/4, and it is 27/28 that work; ata-4 carries vectors 0x13/0x0b and
        // interrupts 11/12, and it is 19/11 that work. The vp VECTOR field is
        // where `AAPL,interrupts` shows up, which is what makes the two easy to
        // swap. ATY,Rage128Pd carries vectors 0x30 and interrupts 0x20, so the
        // display is source 48 — and the OS has 48 unmasked at priority 2,
        // matching the node's AAPL,interrupt-priorities. Source 32 is the
        // mac-io TIMER, which nothing uses and which the OS leaves masked.
        pic_.setLine(48, ati_.irqLine());
    }
    // One edge, then the line goes back down and the channel's condition is
    // consumed. ⚠ The ATA channels deliberately do NOT go through here: their
    // interrupt reaches the driver through the cell's +0x300 latch, source 11
    // has never raised once in a whole boot, and their behaviour is the one
    // every recorded baseline was measured against.
    void pulse(u32 src, DbdmaChannel& ch)
    {
        if (!ch.irqLine())
            return;
        pic_.setLine(src, true);
        pic_.setLine(src, false);
        ch.clearIrq();
    }

    // Every bus master in the machine answers to the same snoop responder.
    // The DBDMA channels reach it through Bus::snoop; the OHCI cells hold
    // a raw pointer into guest RAM and need it handed to them.
    void attachSnoop(SnoopSink* s)
    {
        snoop = s;
        ohci_[0].snoop = s;
        ohci_[1].snoop = s;
    }

    // ⚠ THE MACHINE LOOP CALLED ohciTick + syncIrqs + cpuLine ONCE PER
    // EMULATED INSTRUCTION, and the profiler charged 26% of the entire
    // emulator to it — more than three times the instruction handlers. Almost
    // every one of those calls could not possibly find anything: a USB frame
    // is due about every 1,600 instructions, a vertical blank far less often,
    // and nothing else in the machine moves at all unless the GUEST touches a
    // device register.
    //
    // So say when the answer can change, and answer from the cache until then:
    //  - devGen_ counts guest accesses that were not plain RAM (read() and
    //    write() bump it), which covers every register write, every EOI, every
    //    IACK and every DMA arm;
    //  - devDueTb_ is the earliest timebase at which a timed device could do
    //    something — the nearer USB frame, the display's blank or its expiry,
    //    and either ATA cell's deferred command.
    // Miss any of those and an interrupt arrives late, so the deadline is a
    // conservative LOWER bound and the generation is bumped generously.
    //
    // ⏱ ONE AXIS, NOT TWO. There used to be a second deadline in INSTRUCTIONS
    // for the ATA cells, because their command delay was denominated that way.
    // A machine has one clock; a run loop that has to clamp on two of them
    // cannot batch, and under real-time pacing the instruction axis is not a
    // clock at all. See AtaCell::tick.
    bool serviceDevices(u64 tb)
    {
        // The display's notion of "now" is used when the driver ARMS the
        // blank, which happens inside a register write and therefore before
        // the gate below can open. One store keeps it exact — see nowTb_.
        nowTb_ = tb;
        ati_.noteTb(tb);
        snd_.noteTb(tb);
        if (!devGateOff && devGen_ == devGenSeen_ && tb < devDueTb_)
            return cpuIrq_;
        ++devServices_;
        devGenSeen_ = devGen_;
        ohciTick(tb);
        syncIrqs();
        cpuIrq_ = pic_.cpuLine();
        devDueTb_ = ohci_[0].nextTickTb();
        const u64 due[5] = {ohci_[1].nextTickTb(), ati_.nextTickTb(),
                            cd_.pendingTb(), hd_.pendingTb(), soundDueTb()};
        for (const u64 d : due)
            if (d < devDueTb_)
                devDueTb_ = d;
        return cpuIrq_;
    }
    // ⏱ SET THE MACHINE'S CLOCK FROM THE PROCESSOR'S, ON THE ONE PATH THAT
    // CANNOT WAIT FOR THE RUN LOOP.
    //
    // Called from read() and write() the moment an access turns out not to be
    // plain RAM. A batched loop is holding cycles it has not applied, so the
    // timebase it last wrote here is behind; every device that models a
    // duration reads this. It is also what ENDS the batch, so the interrupt
    // line this access may raise reaches the processor on the next
    // instruction rather than at the end of a batch sized for an idle machine.
    //
    // The display gets the same value in the same breath: its driver ARMS a
    // vertical blank from inside a register write, and "now" at that moment is
    // what the blank is scheduled against.
    void noteNow()
    {
        const u64 t = deviceNow(nowTb_);
        nowTb_ = t;
        ati_.noteTb(t);
        // The codec has the same problem the display has, from the other
        // side: the guest ARMS a DBDMA list from inside a register write,
        // and the engine asks the codec for room in that same breath.
        snd_.noteTb(t);
    }
    // A machine whose devices were poked from outside the run loop (the shell
    // attaching media, a test rig) must not be answered from the cache.
    void deviceStateChanged() { ++devGen_; }

    // The deadline the gate above is built on, published so a caller can skip
    // forward to it instead of asking once per instruction. A conservative
    // LOWER bound on when a device could next do something, so a caller that
    // stops there cannot miss an event.
    u64 deviceDueTb() const { return devDueTb_; }

    // 📏 HOW OFTEN THE MACHINE ACTUALLY HAS TO LOOK AT ITS DEVICES, which is
    // the question a batched run loop lives or dies on. A loop that charges
    // several instructions at once has to stop at anything that could change
    // an interrupt line, and these are those events: devGen counts the guest's
    // own non-RAM accesses, devServices counts the times the gate opened at
    // all (an access OR a timed device falling due). Instructions divided by
    // devServices is the longest batch this machine would allow — measure it
    // before building a batch, because if the answer is five there is nothing
    // to win.
    u64 devGen() const { return devGen_; }
    u64 deviceServices() const { return devServices_; }

    OhciCell& ohci(u32 i) { return ohci_[i & 1u]; }
    void ohciTick(u64 tb)
    {
        ohci_[0].tick(tb);
        ohci_[1].tick(tb);
        // The display retraces on the same clock as everything else.
        ati_.tick(tb);
        // DMA on this hardware is selected by the channel, not the opcode:
        // tell each cell whether its DBDMA list is armed before any command
        // can present a data phase.
        cd_.setDmaArmed(ataDma_.running());
        hd_.setDmaArmed(hdDma_.running());
        // The cell's +0x300 interrupt register carries a latched image of
        // its DBDMA channel's interrupt, not just the drive's INTRQ.
        cd_.setDmaIrq(ataDma_.irqLine());
        hd_.setDmaIrq(hdDma_.irqLine());
        // Deferred ATA commands (see AtaCell::write case 0x070). When one
        // fires, its data phase has just opened, so resume any DBDMA list
        // parked on that channel — a DMA read arms the list before the
        // command register is written, and register writes are the only
        // other thing that wakes it.
        if (cd_.tick())
            ataDma_.wake();
        if (hd_.tick())
            hdDma_.wake();
        // The codec is the one device that parks a channel on ITSELF: a
        // stream drains at its sample rate, so an output list is left
        // mid-descriptor until the FIFO has room again. Nothing the guest
        // does wakes it — the passage of time does.
        if (soundOn) {
            snd_.noteTb(tb);
            if (sndOut_.parked() && tb >= snd_.outDueTb())
                sndOut_.wake();
            if (sndIn_.parked() && tb >= snd_.inDueTb())
                sndIn_.wake();
        }
    }
    // The codec's contribution to the device gate's deadline. It is asked of
    // the CHANNELS, not of the codec: only a channel knows it is parked
    // mid-descriptor, and a descriptor larger than the FIFO — which every
    // audio buffer is — parks on a call where the codec accepted bytes and
    // had no reason to think anything was waiting.
    u64 soundDueTb() const
    {
        if (!soundOn)
            return ~0ull;
        u64 d = ~0ull;
        if (sndOut_.parked())
            d = snd_.outDueTb();
        if (sndIn_.parked()) {
            const u64 i = snd_.inDueTb();
            if (i < d)
                d = i;
        }
        return d;
    }

    // DBDMA channels, one per ATA cell, at the mac-io offsets the ROM's
    // own device tree names in each node's second `reg` pair:
    //   ata-3@20000 -> +0x8B00   (the CD)
    //   ata-4@1f000 -> +0x8A00   (the internal disk)
    // The second one was missing, and its absence is not quiet: ata-4 is
    // the Ultra ATA channel, the ROM publishes `cable-type 80-conductor`
    // on it, and Open Firmware's driver therefore drives it with DMA. With
    // no engine behind the window the driver configured the drive and then
    // never issued a read, so `dir hd:,\` returned an empty listing and
    // `boot hd:` answered "can't OPEN" — a missing device presenting as a
    // missing volume.
    DbdmaChannel& ataDma() { return ataDma_; }
    DbdmaChannel& hdDma() { return hdDma_; }
    // Channels 0-7, the deviceless ones. Exposed so a report can say what a
    // driver polling one of them is actually looking at.
    DbdmaChannel& genDma(u32 i) { return dmaGen_[i & 7u]; }

    // The sound codec and its two DBDMA channels, at the KeyLargo offsets
    // the same map gives: 0x8800 audio out, 0x8900 audio in.
    //
    // ⚠⚠ CONSTRUCTOR DEFAULT, WHICH IS WHAT THE CAPI AND THEREFORE THE APP
    // GET. Turning this on is a MACHINE-BEHAVIOUR change, not an addition:
    // the boot ROM arms the output channel for the startup chime and then
    // SPINS ON ITS ACTIVE BIT (fff868ec..fff868fc). With nothing behind the
    // window the status register read zero, the spin fell straight through
    // and the chime cost nothing; with a channel there the ROM waits for the
    // chime to finish, which is what the hardware does and what the sound is
    // for. `--no-sound` is the control, and it restores the old machine
    // exactly — the register window falls back to KeyLargo storage.
    bool soundOn = true;
    AwacsCell& sound() { return snd_; }
    const AwacsCell& sound() const { return snd_; }
    DbdmaChannel& sndOut() { return sndOut_; }
    DbdmaChannel& sndIn() { return sndIn_; }

    // ATA cells (OF's tree: ata-4@1f000, ata-3@20000, ata-3@21000, each
    // with a /disk node). The CD lives on ata-3@20000 device 0 when an
    // ISO is attached; the other buses stay empty. Non-data register
    // traffic is logged bus-tagged.
    const std::vector<RegWr>& ataLog() const { return ataLog_; }
    // --ata-log-from N: the ring trims as it grows, so a command at 3.7 G
    // is long gone by the end of a 5 G run. Gate it and the window lands
    // where the question is.
    u64 ataLogFrom = 0;
    // Where Open Firmware actually put the display. An NDRV computes its
    // register base from the device tree, so an unrouted BAR shows up as a
    // DSI on a plausible-looking address rather than as anything about
    // video — the boot died on lwzx from 0x92000104 with these at 0.
    u32 atiRegBar() const { return atiRegBar_; }
    u32 atiIoBar() const { return atiIoBar_; }
    u32 atiFbBar() const { return atiFbBar_; }
    u32 atiRomBar() const { return atiRomBar_; }
    u32 ohciBar(u32 i) const { return ohciBar_[i & 1u]; }
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
    // Instructions between delivered receive bytes. Un-paced delivery
    // overflows the firmware.s input ring and drops the tail, but 3M was
    // picked for a short script: a 300-character setup takes 900M
    // instructions to type, which is most of a boot.
    u64 rxPaceInsns = 3000000ull;

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
    // Open Firmware reaches every device register through a handful of
    // shared access primitives (ff80b5f8 is its 32-bit load), so the pc of
    // a device access names the primitive and never the driver. The LINK
    // register names the word that called it, which is the only useful
    // attribution here — and the ATA traffic log was quoting the pc.
    const u32* lrRef = nullptr;
    const u64* stamp = nullptr;

    // ⚠⚠ ONE CALL, EVERY CELL. The instruction counter used to be wired cell
    // by cell by each consumer, and the capi's list was missing hd() and
    // hdDma(): the app's hard disk raised BSY on its first command and held it
    // for the rest of the machine's life, while g4run — whose list was
    // complete — booted from the same image. A per-consumer wiring list is a
    // defect waiting for its next reader; call these instead of assigning the
    // fields. (The cells' CLOCK is wired in the constructor, because it is the
    // machine's own — see nowTb_.)
    void setStamp(const u64* s)
    {
        stamp = s;
        cd_.stamp = hd_.stamp = s;
        pic_.stamp = s;
        ohci_[0].stamp = ohci_[1].stamp = s;
        ati_.stamp = s;
        ataDma_.stamp = hdDma_.stamp = s;
        sndOut_.stamp = sndIn_.stamp = s;
        for (DbdmaChannel& ch : dmaGen_)
            ch.stamp = s;
    }
    void setPcRef(const u32* p)
    {
        pcRef = p;
        cd_.pcRef = hd_.pcRef = p;
        ohci_[0].pcRef = ohci_[1].pcRef = p;
        ati_.pcRef = p;
        ataDma_.pcRef = hdDma_.pcRef = p;
        sndOut_.pcRef = sndIn_.pcRef = p;
        for (DbdmaChannel& ch : dmaGen_)
            ch.pcRef = p;
    }

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
    // The window aliases modulo the MODULE size, which dimms() derives from the
    // allocation. A fixed constant here could disagree with what the SPD
    // advertises, and a ROM that does probe would then find a different size
    // from the one it was told.
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
        // Behind the PCI-to-PCI bridge the firmware allocates out of the
        // bridge's memory window, so the assignment is no longer the fixed
        // 0x80000000 this decode used to assume. Follow the BAR the guest
        // actually wrote; the two constants above stay as the early hard
        // decode and the historical assignment.
        if (macioBar_ && pa - macioBar_ < kMacIoSize)
            return pa - macioBar_;
        return 0xFFFFFFFFu;
    }

    // Plain RAM, answered before the device decode is walked at all. It is
    // the overwhelmingly common access, it is provably not a device (every
    // window below lives at 0x80000000 or above, and the machine tops out at
    // 1536 MB), and reaching it used to mean falling through a dozen range
    // tests. The bump on the other path is the machine's DEVICE GENERATION:
    // see serviceDevices().
    static constexpr u32 kDeviceWindowBase = 0x80000000u;
    bool ramFast(u32 pa, u32 len) const
    {
        return pa < kDeviceWindowBase &&
               static_cast<size_t>(pa) + len <= ram_.size();
    }

    // Plain storage: RAM, or the boot ROM while it answers as an array rather
    // than as a flash state machine. Everything else here is a device or an
    // unmapped hole, and both dislike being read speculatively — see
    // Bus::memoryAt.
    bool memoryAt(u32 pa, u32 len) const override
    {
        if (ramFast(pa, len))
            return true;
        return flashMode_ == kFlashArray && pa >= kRomBase &&
               static_cast<size_t>(pa - kRomBase) + len <= rom_.size();
    }

    u32 read(u32 pa, u32 len)
    {
        if (ramFast(pa, len))
            return get(ram_.data() + pa, len);
        ++devGen_;
        noteNow();
        const u32 off = macioOff(pa);
        if (off != 0xFFFFFFFFu) {
            if (off >= 0x16000u && off < 0x18000u) {
                u32 v = 0;
                for (u32 k = 0; k < len; ++k)
                    v = (v << 8) |
                        pmu_.read(off - 0x16000u + k, stamp ? *stamp : 0);
                return v;
            }

            if (off >= 0x13000u && off < 0x14000u) {
                ++sccReads;
                ++sccOffHist[off - 0x13000u];
                return sccRead(off - 0x13000u);
            }
            if (off >= 0x18000u && off < 0x18100u)
                return i2cRead(1, off - 0x18000u);
            if (off - 0x40000u < 0x40000u)
                return pic_.read(off - 0x40000u, len);
            if (off - 0x8B00u < 0x100u)
                return ataDma_.read(off - 0x8B00u, len);
            if (off - 0x8A00u < 0x100u)
                return hdDma_.read(off - 0x8A00u, len);
            // Channels 0-7: real engines with nothing attached. See the note
            // in the constructor — a register store here hangs the OS boot.
            if (off - 0x8000u < 0x800u)
                return dmaGen_[(off >> 8) & 7u].read(off & 0xFFu, len);
            if (soundOn) {
                if (off - 0x8800u < 0x100u)
                    return sndOut_.read(off - 0x8800u, len);
                if (off - 0x8900u < 0x100u)
                    return sndIn_.read(off - 0x8900u, len);
                if (off - AwacsCell::kRegBase < AwacsCell::kRegSize)
                    return snd_.read(off - AwacsCell::kRegBase, len);
            }
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
                if ((off & 0xFF0u) != 0 &&
                    !(stamp && *stamp < ataLogFrom)) {
                    if (ataLog_.size() >= 6000)
                        ataLog_.erase(ataLog_.begin(),
                                      ataLog_.begin() + 3000);
                    // Bit 31 marks a read, not bit 0: the cell registers at
                    // +0x200 are byte-addressable, so an odd offset is a
                    // real lane and `off | 1` turned a byte read of +0x201
                    // into a read of +0x200.
                    ataLog_.push_back({stamp ? *stamp : 0,
                                       off | 0x80000000u, v,
                                       lrRef ? *lrRef : (pcRef ? *pcRef : 0),
                                       len});
                }
                return v;
            }
            if (klTimerOn && off - kKlTimerLo < 8u &&
                off - kKlTimerLo + len <= 8u) {
                u8 img[8];
                klTimerImage(klTimerCount(), img);
                klNote(kMacIoBase + off, 0, false);
                return get(img + (off - kKlTimerLo), len);
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
        // The card's PCI I/O aperture: MM_INDEX at +0x00 latches a register
        // offset, MM_DATA at +0x04 reads or writes it.
        //
        // PCI I/O space is LITTLE-ENDIAN and the processor is not, so every
        // value through this window arrives byte-reversed. Without the swap
        // the latched indices read as 0xa0000000, 0xa4000000, 0xa8000000 —
        // absurd offsets into a 16 KB register file — and the writes landed
        // on register 0 while the screen stayed black. Reversed they are
        // 0xa0, 0xa4, 0xa8, and a byte written at +0x06 addresses lane
        // 3-(o-4) of the target dword, not lane o-4.
        if (atiIoBar_ && atiIoBar_ < 0x00100000u &&
            pa - (0xF0000000u + atiIoBar_) < 0x100u) {
            const u32 o = pa - (0xF0000000u + atiIoBar_);
            if (o < 4u)
                return ioSwap(atiMmIndex_, len);
            if (o < 8u) {
                const u32 idx = atiMmIndex_ & 0x3FFCu;
                // R128Cell stores registers NATIVE (little-endian) and swaps on
                // the way in, so a byte at I/O offset o-4 belongs in native
                // lane o-4 - not 3-(o-4), which put every byte in the wrong
                // half of the register.
                const u32 lane = len == 4u ? 0u : (o - 4u);
                return ioSwap(ati_.read(idx + lane, len), len);
            }
            return 0;
        }
        if (atiRegBar_ && pa - atiRegBar_ < 0x4000u)
            return ati_.read(pa - atiRegBar_, len);
        if (atiFbBar_ && pa - atiFbBar_ < (64u << 20))
            // Both halves address the same VRAM; the upper is the
            // big-endian alias.
            return get(ati_.vram.data() + ((pa - atiFbBar_) & 0x01FFFFFFu),
                       len);
        if (pa - kSizeWin < 0x20000000u) {
            const u32 v =
                get(ram_.data() + ((pa - kSizeWin) & ((dimms().mb << 20) - 1)), len);
            if (szLog_.size() < 4000)
                szLog_.push_back({stamp ? *stamp : 0, pa | 1u, v,
                                  pcRef ? *pcRef : 0});
            return v;
        }
        if (pa < ram_.size() && pa + len <= ram_.size())
            return get(ram_.data() + pa, len);
        if (pa >= kRomBase && pa - kRomBase + len <= rom_.size()) {
            if (flashMode_ != kFlashArray) {
                u32 st = 0; // WSM ready, no erase or program error
                for (u32 k = 0; k < len && k < 4; ++k)
                    st = (st << 8) | 0x80u;
                return st;
            }
            return get(rom_.data() + (pa - kRomBase), len);
        }
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
        // Write heatmap. "The OS set a video mode and then painted
        // nothing" is only ever half an observation — it painted
        // somewhere, and no instrument could say where. Bucket every
        // guest write by megabyte over a window and the destination names
        // itself, whether that is the framebuffer aperture, an alias of
        // it, or ordinary RAM because a base register was misread.
        if (wmapTo && stamp && *stamp >= wmapFrom && *stamp < wmapTo) {
            ++wmap[pa >> 20];
            if (wmapPcSet && (pa >> 20) == wmapPcBucket && pcRef)
                ++wmapPcs[*pcRef];
        }
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
        if (ramFast(pa, len)) {
            put(ram_.data() + pa, v, len);
            return;
        }
        ++devGen_;
        noteNow();
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
                ++sccWrites;
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
            if (off - 0x8A00u < 0x100u) {
                hdDma_.write(off - 0x8A00u, v, len);
                return;
            }
            // Channels 0-7 — see the read path and the constructor. FLUSH has
            // to clear, and only the engine does that.
            if (off - 0x8000u < 0x800u) {
                dmaGen_[(off >> 8) & 7u].write(off & 0xFFu, v, len);
                return;
            }
            if (soundOn) {
                if (off - 0x8800u < 0x100u) {
                    sndOut_.write(off - 0x8800u, v, len);
                    return;
                }
                if (off - 0x8900u < 0x100u) {
                    sndIn_.write(off - 0x8900u, v, len);
                    return;
                }
                if (off - AwacsCell::kRegBase < AwacsCell::kRegSize) {
                    snd_.write(off - AwacsCell::kRegBase, v, len);
                    return;
                }
            }
            if (off - 0x1F000u < 0x3000u) {
                if ((off & 0xFF0u) != 0 &&
                    !(stamp && *stamp < ataLogFrom)) {
                    if (ataLog_.size() >= 6000)
                        ataLog_.erase(ataLog_.begin(),
                                      ataLog_.begin() + 3000);
                    ataLog_.push_back({stamp ? *stamp : 0, off, v,
                                       lrRef ? *lrRef : (pcRef ? *pcRef : 0),
                                       len});
                }
                if (off - 0x20000u < 0x1000u && cd_.present()) {
                    cd_.write(off - 0x20000u, v, len);
                    // a task-file write can open a fresh data phase:
                    // resume any standing DBDMA list
                    ataDma_.wake();
                } else if (off - 0x1F000u < 0x1000u && hd_.present()) {
                    hd_.write(off - 0x1F000u, v, len);
                    hdDma_.wake(); // a command can open a fresh data phase
                }
                return;
            }
            if (klTimerOn && off - kKlTimerLo < 8u &&
                off - kKlTimerLo + len <= 8u) {
                klTimerWrite(off, v, len);
                klNote(kMacIoBase + off, v, true);
                return;
            }
            u8 gpioOld[8] = {};
            for (u32 k = 0; k < len && k < 8; ++k)
                gpioOld[k] = kl_[off + k]; // before the store
            put(kl_.data() + off, v, len);
            // KeyLargo's GPIO block is 0x30 bytes at mac-io +0x50, and it
            // is NOT storage. The first eight are level registers and are
            // read-only; in the rest, bit 1 is IN_DATA — an input — and a
            // store cannot drive it. It reflects OUT_DATA only while
            // OUT_ENABLE is set, and otherwise holds its own value.
            //
            // This was found one register at a time: +0x61 is index 17 in
            // the block (GPIO9), and Open Firmware reads its bit 1 as a
            // "reset the configuration" strap — with it low, startup runs
            // gen-defaults + gen-default-vars and overwrites every
            // /options property that (gen-configs) just built out of
            // NVRAM, which is why auto-boot?, use-nvramrc?, input-device
            // and output-device were never under our control. A wide store
            // across the block had been clearing it. The whole block obeys
            // the same rule, so model the rule rather than the symptom.
            for (u32 k = 0; k < len && k < 8; ++k) {
                const u32 g = off + k;
                if (g < kGpioBase || g >= kGpioBase + kGpioSize)
                    continue;
                if (g < kGpioBase + 8u) { // level registers: read-only
                    kl_[g] = gpioOld[k];
                    continue;
                }
                // The stored byte is now the incoming value; IN_DATA is an
                // input, so take it from the pre-store byte unless
                // OUT_ENABLE says the pin is driving.
                u8 nv = static_cast<u8>(kl_[g] & ~kGpioInData);
                nv = static_cast<u8>(
                    nv | ((nv & kGpioOutEnable)
                              ? static_cast<u8>((nv & kGpioOutData) << 1)
                              : (gpioOld[k] & kGpioInData)));
                kl_[g] = nv;
            }
            klNote(kMacIoBase + off, v, true);
            return;
        }
        for (u32 f = 0; f < 2; ++f)
            if (ohciBar_[f] && pa - ohciBar_[f] < 0x1000u) {
                ohci_[f].write(pa - ohciBar_[f], v, len);
                return;
            }
        if (atiIoBar_ && atiIoBar_ < 0x00100000u &&
            pa - (0xF0000000u + atiIoBar_) < 0x100u) {
            const u32 o = pa - (0xF0000000u + atiIoBar_);
            const u32 sv = ioSwap(v, len);
            if (atiIoLog_.size() < 400)
                atiIoLog_.push_back(
                    {stamp ? *stamp : 0, o | (len << 8), sv,
                     pcRef ? *pcRef : 0});
            if (o < 4u) {
                atiMmIndex_ = sv;
                return;
            }
            if (o < 8u) {
                const u32 idx = atiMmIndex_ & 0x3FFCu;
                // R128Cell stores registers NATIVE (little-endian) and swaps on
                // the way in, so a byte at I/O offset o-4 belongs in native
                // lane o-4 - not 3-(o-4), which put every byte in the wrong
                // half of the register.
                const u32 lane = len == 4u ? 0u : (o - 4u);
                ati_.write(idx + lane, sv, len);
                return;
            }
            return;
        }
        if (atiRegBar_ && pa - atiRegBar_ < 0x4000u) {
            const u32 off = pa - atiRegBar_;
            ati_.write(off, v, len);
            // The card is a BUS MASTER, and this is the one place it acts as
            // one: writing the transfer address starts a fetch out of system
            // memory. The walk lives here rather than in the cell because the
            // BUS owns RAM and the snoop responder — the cell is a leaf device
            // with no route to either, and giving it one would add members and
            // kill every snapshot.
            if ((off & ~3u) == 0x0A50u)
                r128BusMasterFetch(ati_, ram_.data(),
                                   static_cast<u32>(ram_.size()), snoop);
            // The CCE's PIO packet FIFO: each store hands the command
            // engine one DWORD of packet stream, and executing the stream
            // is what lands the ARM's GUI_SCRATCH fence. Routed from here
            // for the same reason as the bus-master fetch above — packet
            // execution can dispatch an INDIRECT buffer, which the card
            // fetches out of system RAM through the Uni-N GART, and only
            // the bus owns RAM, the snoop responder and the bridge's
            // config space where the GART base lives.
            else if ((off & ~3u) == 0x1000u || (off & ~3u) == 0x1004u)
                r128CceFifoWord(ati_, v, len, ram_.data(),
                                static_cast<u32>(ram_.size()), agpGartBase(),
                                agpAperBase(), snoop);
            // A guest that writes PM4_IW_INDSIZE directly (SDK §5.3.3's
            // other flow) dispatches the same way a packet-written one does.
            else if ((off & ~3u) == 0x073Cu)
                r128CceIndirect(ati_, ati_.peek(0x073Cu), ram_.data(),
                                static_cast<u32>(ram_.size()), agpGartBase(),
                                agpAperBase(), snoop);
            return;
        }
        if (atiFbBar_ && pa - atiFbBar_ < (64u << 20)) {
            // Framebuffer traffic, counted. "Is the OS drawing anything?"
            // is otherwise answerable only through the CRTC gate, which
            // stays shut until the display driver programs it — so a guest
            // that paints into VRAM without ever setting the mode looks
            // identical to one that never ran.
            ++ati_.fbWrites;
            const u32 fo = pa - atiFbBar_;
            if (fo < ati_.fbLo) ati_.fbLo = fo;
            if (fo > ati_.fbHi) ati_.fbHi = fo;
            put(ati_.vram.data() + ((pa - atiFbBar_) & 0x01FFFFFFu), v, len);
            return;
        }
        if (pa - kSizeWin < 0x20000000u) {
            put(ram_.data() + ((pa - kSizeWin) & ((dimms().mb << 20) - 1)), v, len);
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
        if (pa >= kRomBase && pa - kRomBase + len <= rom_.size()) {
            // The boot flash, in sequence. Open Firmware commits NVRAM here
            // on `reset-all` and announces "erasing fff06000" first, so the
            // command protocol is the last link between `setenv` and a
            // setting that survives a reboot. Per-address totals cannot show
            // a protocol; the order of the bytes is the protocol.
            if (flashLog_.size() < 256)
                flashLog_.push_back({stamp ? *stamp : 0, pa, v,
                                     pcRef ? *pcRef : 0});
            const u32 off = pa - kRomBase;
            const u8 cmd = static_cast<u8>(v);
            if (flashMode_ == kFlashProgram) {
                // Flash programming can only clear bits; the erase is what
                // sets them. Modelling it as a plain store would let a
                // driver "write" ones into an unerased block and hide its
                // own bug.
                for (u32 k = 0; k < len && off + k < rom_.size(); ++k)
                    rom_[off + k] &= static_cast<u8>(v >> (8 * (len - 1 - k)));
                flashMode_ = kFlashStatus;
                return;
            }
            switch (cmd) {
            case 0xFF: // read array
            case 0x50: // clear status register
                flashMode_ = kFlashArray;
                return;
            case 0x20: // block erase setup — the confirm carries the block
                flashErase_ = off;
                flashMode_ = kFlashEraseSetup;
                return;
            case 0xD0: // erase confirm
                if (flashMode_ == kFlashEraseSetup) {
                    // 8 KB blocks: Open Firmware erases at fff06000 to
                    // rewrite the NVRAM partition that begins at fff06200.
                    const u32 base = flashErase_ & ~0x1FFFu;
                    for (u32 k = 0; k < 0x2000u && base + k < rom_.size();
                         ++k)
                        rom_[base + k] = 0xFF;
                }
                flashMode_ = kFlashStatus;
                return;
            case 0x40: // word/byte program setup
            case 0x10:
                flashMode_ = kFlashProgram;
                return;
            case 0x70: // read status register
                flashMode_ = kFlashStatus;
                return;
            default:
                flashMode_ = kFlashStatus;
                return;
            }
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
            stamp &&
            ((atiVisibleAt && *stamp < atiVisibleAt) ||
             (atiHideTo > atiHideFrom && *stamp >= atiHideFrom &&
              *stamp < atiHideTo))) {
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
            // mac-io BAR0: a 512 KB memory window, sized the same way.
            if (b == 1u && (cfgAddr_[b] & 0x0FFFFF00u) == kMacIoSel &&
                reg == 0x10u) {
                word &= 0xFFF80000u;
                if (word != 0xFFF80000u && word) macioBar_ = word;
            }
            for (u32 f = 0; f < 2; ++f) {
                if (b == 1u &&
                    (cfgAddr_[b] & 0x0FFFFF00u) == kOhciSel[f]) {
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
                    // BAR0 is 64 MiB, not 32. The card's own FCode `reg`
                    // property declares 0x04000000, and it paints into the
                    // UPPER half — its `address` property is
                    // mapped-BAR0 + 0x8000 + 0x02000000 — because that half
                    // is the big-endian alias of the same 32 MiB of VRAM.
                    // Sized at 32 MiB, the aperture did not even cover the
                    // address the firmware publishes, so "0 framebuffer
                    // writes" was a window too small to see them.
                    word &= 0xFC000000u;
                    atiFbBar_ = (word != 0xFC000000u) ? word : atiFbBar_;
                    ati_.fbBase = atiFbBar_;
                    break;
                case 0x14u:
                    // The I/O BAR is not decoration: Open Firmware drives
                    // this card through it. With the FCode running, 415
                    // writes and 190 reads landed at 0xf0000400 — the AGP
                    // bus I/O window plus this BAR — and vanished, because
                    // we accepted the BAR in config space and routed
                    // nothing to it.
                    word = (word & 0xFFFFFF00u) | 1u;
                    // Sizing writes all-ones and reads back; a real
                    // assignment is a low I/O address. Latching the probe
                    // value put the window on top of the config latch and
                    // took every BAR in the machine down with it.
                    if ((word & 0xFFFFFF00u) != 0xFFFFFF00u &&
                        (word & 0xFFFFFF00u) < 0x00100000u)
                        atiIoBar_ = word & 0xFFFFFF00u;
                    break;
                case 0x18u:
                    word &= 0xFFFFC000u;
                    atiRegBar_ =
                        (word != 0xFFFFC000u) ? word : atiRegBar_;
                    // The card reports its own apertures back through
                    // CONFIG_REG_1_BASE; a driver that asks where its
                    // registers are must not be told zero.
                    ati_.regBase = atiRegBar_;
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
        // 65536, not 2048: the old cap filled during Open Firmware's probe,
        // so every Mac-OS-era access (a driver reading a device's config)
        // fell off the end — and a head-capped log answers "did the OS
        // driver ever probe this device" with a silence that reads as no.
        if (cfgLog_.size() < 65536)
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
        const u8 a = i2c_[0].addr & 0xFEu;
        // Slots 0..2 answer at 0xA0/0xA2/0xA4, as many as the installed memory
        // needs (see dimms()). Slot 3 is never populated: a fourth module would
        // take the machine to 2 GB, and RAM would then run into the 0x78000000
        // sizing window and the device region above it.
        if (a == 0xACu)
            return cpuModuleRom;                // the cache descriptor
        const u32 slot = (a - 0xA0u) >> 1;
        return a >= 0xA0u && a <= 0xA6u && slot < dimms().count;
    }
    u8 slaveByte(u32 n) const
    {
        if (n != 0)
            return 0xFFu;
        return (i2c_[0].addr & 0xFEu) == 0xACu ? cacheRomByte() : spdByte();
    }
    // The installed modules, derived from the allocation — see dimms() in
    // sawtooth.cpp for why this is not a constant.
    struct Dimms {
        u32 count, mb;
        u8 rows, cols, ranks, density;
    };
    Dimms dimms() const;
    u8 spdByte() const;
    u8 cacheRomByte() const;

    // Z8530, just enough for a polled console: pointer-register protocol
    // per channel, RR0 reports TX-empty always, data writes append to the
    // captured console text.
    // SCC access census. sccRead/sccWrite bypass the first-touch log, so
    // "did Open Firmware open the serial console at all" had no answer:
    // zero captured output is equally consistent with a console bound to
    // the screen and with a machine that never got that far.
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
                rxNextAt_ = *stamp + rxPaceInsns;
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

    // --- KeyLargo timer (see kKlTimerLo) ---------------------------------
    // 64 bits counting up at 18.432 MHz of GUEST time, presented as two
    // little-endian 32-bit registers so that the guest's lwbrx reads it
    // right way round.
    u64 klTimerCount() const
    {
        // The timebase RESTARTS at zero on a processor reset (Open Firmware's
        // `reset-all` does one mid-boot) and is restored wholesale by a
        // snapshot, so "now" can legitimately be earlier than the reload
        // point. An unsigned subtract there would hand the guest a count near
        // 2^64 and a calibrated clock built on it — clamp instead.
        const u64 dt = nowTb_ > klTimerTb_ ? nowTb_ - klTimerTb_ : 0ull;
        return klTimerVal_ + dt * kKlTimerNum / kKlTimerDen;
    }
    static void klTimerImage(u64 c, u8 img[8])
    {
        for (u32 k = 0; k < 8; ++k)
            img[k] = static_cast<u8>(c >> (8 * k));
    }
    static u64 klTimerFromImage(const u8 img[8])
    {
        u64 c = 0;
        for (u32 k = 0; k < 8; ++k)
            c |= static_cast<u64>(img[k]) << (8 * k);
        return c;
    }
    // Any store in the window reloads: the ROM zeroes both halves, and the
    // rebase has to happen on each of them or the second write would carry
    // the elapsed time of the first.
    void klTimerWrite(u32 off, u32 v, u32 len)
    {
        u8 img[8];
        klTimerImage(klTimerCount(), img);
        put(img + (off - kKlTimerLo), v, len);
        klTimerVal_ = klTimerFromImage(img);
        klTimerTb_ = nowTb_;
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
    u8 klGpioPrev_[0x30] = {};
    std::vector<RegWr> uninLog_;
    std::vector<RegWr> flashLog_;
    std::vector<RegWr> atiIoLog_; // the card's I/O aperture, in order
    // Intel/CFI command state for the boot flash. Reads answer the status
    // register while a command is in flight; 0x80 is WSM-ready, and Open
    // Firmware polls it for 8.2 million instructions before giving up.
    enum { kFlashArray, kFlashStatus, kFlashEraseSetup, kFlashProgram };
    u32 flashMode_ = kFlashArray;
    u32 flashErase_ = 0;
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
    u32 macioBar_ = 0;        // OF-assigned mac-io BAR0 (bridge window)
    u32 ohciBar_[2] = {0, 0}; // OF/OS-assigned BAR0 per function
    R128Cell ati_;
    DbdmaChannel ataDma_, hdDma_;
    // Channels 0-7 (+0x8000..+0x87FF): present, deviceless. See the
    // constructor for why they cannot be left to the register store.
    DbdmaChannel dmaGen_[8];
    AwacsCell snd_;
    DbdmaChannel sndOut_, sndIn_;
    std::vector<u8> atiRom_;
    u64 romBase_ = 0; // FNV-1a of the boot flash as loaded (see the ctor)
    u32 atiFbBar_ = 0, atiRegBar_ = 0, atiRomBar_ = 0;
    // PCI I/O space is little-endian and this processor is not.
    static u32 ioSwap(u32 v, u32 len)
    {
        if (len == 2u)
            return ((v & 0xFFu) << 8) | ((v >> 8) & 0xFFu);
        if (len == 4u)
            return ((v & 0xFFu) << 24) | ((v & 0xFF00u) << 8) |
                   ((v >> 8) & 0xFF00u) | ((v >> 24) & 0xFFu);
        return v;
    }
    u32 atiIoBar_ = 0; // PCI I/O aperture, 256 B: MM_INDEX / MM_DATA
    u32 atiMmIndex_ = 0;
};

} // namespace opm
