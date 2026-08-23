#pragma once
#include "opm/cdimage.hpp"
#include "opm/dbdma.hpp"
#include "opm/types.hpp"

#include <cstdio>
#include <string>
#include <vector>

namespace opm {

struct SnapWriter;
struct SnapReader;

// One mac-io ATA cell (Heathrow-lineage register file, stride 0x10)
// carrying a single ATAPI CD/DVD device as device 0, backed by a disc
// image streamed from the host file — plain 2048 (.iso/.cdr/.toast),
// raw 2352 (.bin, with or without a .cue sheet), or UDIF .dmg; the
// format mapping lives in CdImage. Built against the OF driver's own
// observed device-detect sequence:
//   +0x00 data (16-bit PIO) · +0x10 error/features · +0x20 int-reason
//   (nsect) · +0x30 lba0 · +0x40 byte-count lo (lba1) · +0x50 byte-count
//   hi (lba2) · +0x60 device · +0x70 status/command · +0x160
//   alt-status/devctl · +0x200 timing (inert store)
// ATAPI signature after reset/diagnostic: 01 01 14 EB.
//
// It is a DmaDevice: the DBDMA engine's device end, alongside the sound
// codec. A drive is the NON-streaming kind — a short transfer means its data
// phase ended, not that it will want the rest later.
class AtaCell : public DmaDevice {
public:
    // The image is held open for the life of the cell; nothing else owns it.
    ~AtaCell()
    {
        if (iso_)
            fclose(iso_);
    }

    bool attachIso(const char* path);
    // The same cell serving an ordinary ATA (non-packet) hard disk, which
    // is what a Sawtooth actually boots from — the CD is the exception,
    // not the rule. Read/write, LBA28, and the plain ATA power-on
    // signature (01 01 00 00) instead of the ATAPI 01 01 14 EB.
    bool attachDisk(const char* path);
    // A DRIVE is on the channel. ATA has no hot-plug: the drive is here from
    // power-on or never, and it is what the firmware's bus scan and the OS's
    // ATA Manager enumerate. The MEDIUM is the part that comes and goes.
    bool present() const
    {
        return seated_ || iso_ != nullptr || media_.ok();
    }
    // Why the last attachIso refused, for whoever owns the console.
    const char* mediaError() const { return media_.why().c_str(); }
    const CdImage& media() const { return media_; }

    // --- the medium, as distinct from the drive ---------------------------
    // The OS learns about media the way it learns from a real drive: TEST
    // UNIT READY (and every other media-access command) answers NOT READY /
    // MEDIUM NOT PRESENT while the tray is empty, and the first command
    // after a disc arrives gets UNIT ATTENTION / MEDIUM MAY HAVE CHANGED,
    // after which the driver reads the TOC and mounts what it finds. Nothing
    // is injected into the guest: this is the protocol a swapped CD speaks.
    //   seat()        an empty drive on the channel (tray open, nothing in it)
    //   hostEject()   pull the disc from outside — the pinhole, not the OS's
    //                 own eject, so PREVENT does not apply
    //   hostInsert()  put a disc in: present, with the unit attention armed
    //   hostSwap()    eject, let the guest SEE the empty tray (one media
    //                 command refused, and at least kSwapDwellTb), then
    //                 insert `path`; tick() carries it out. A guest that
    //                 never looks gets the new disc after kSwapFallbackTb.
    void seat() { seated_ = true; }
    bool mediaPresent() const
    {
        return disk_ ? iso_ != nullptr : (media_.ok() && !ejected_);
    }
    void hostEject();
    bool hostInsert(const char* path); // false: refused, mediaError() says why
    bool hostSwap(const char* path);
    // hostStage(): the SAFE republish, the only one the app may use. An
    // empty tray takes the disc now; a disc the guest has never read is
    // swapped (hostSwap); a disc the guest has READ stays in until the
    // guest itself puts the volume away (START STOP LoEj), and the staged
    // disc goes in after that. MEASURED (s45 E3): Mac OS swallows the
    // UNIT ATTENTION on a mounted volume and re-reads the new disc through
    // the OLD catalog — a file opened after such a swap shows garbage — and
    // (E4) never polls a mounted drive, locked or not. awaitingGuestEject()
    // is the state the app shows the user.
    bool hostStage(const char* path);
    bool awaitingGuestEject() const { return !stagedPath_.empty() && mediaPresent(); }
    bool swapPending() const { return !stagedPath_.empty(); }
    bool unitAttentionPending() const { return unitAttention_; }
    bool lockedByGuest() const { return locked_; }
    // Experiment: refuse PREVENT ALLOW MEDIUM REMOVAL (a drive with no door
    // lock) to learn whether the guest keeps polling a mounted disc it cannot lock.
    void setNoDoorLock(bool v) { noDoorLock_ = v; }
    u32 insertions() const { return insertions_; }
    u32 guestEjects() const { return guestEjects_; }
    const std::string& mediaPath() const { return mediaPath_; }
    u64 readsSinceInsert() const { return readsSinceInsert_; }
    static constexpr u64 kSwapDwellTb = 25000000ull;     // 1 s at 25 MHz
    static constexpr u64 kSwapFallbackTb = 125000000ull; // 5 s
    // The media-change state travels in its own optional snapshot section
    // (see SawtoothBus::snapSave) so snapshots written before it existed
    // still load.
    void snapSaveMedia(SnapWriter& w) const;
    void snapLoadMedia(SnapReader& r);
    bool irqLine() const { return irq_; }
    void setDmaArmed(bool a) { dmaArmed_ = a; }
    // The channel's DBDMA engine raised its interrupt: latch it into the
    // cell's +0x300 register, which is where the driver looks.
    void setDmaIrq(bool a)
    {
        if (a)
            dmaIrqLatch_ = true;
    }
    bool latchTrace = false; // --ata-latch: print the task file per command
    u8 devSel() const { return dev_; } // diagnostic: drive-select bits

    u32 read(u32 off, u32 len);
    void write(u32 off, u32 v, u32 len);

    // Everything a stalled data phase depends on, in one line. "The host
    // stopped reading" and "the drive stopped offering" look identical from
    // the register trace; only the cell's own counters tell them apart.
    void dumpState(const char* who) const;
    // The same report as a string, so it can reach somewhere other than this
    // process's stdout. A GUI has no stdout, and a machine that wedges while
    // the user is driving it interactively is exactly the machine whose state
    // is worth having — see opm_diag.
    std::string describe(const char* who) const;

    // Run the deferred command, if its BSY window has elapsed. Called from
    // the machine's peripheral tick.
    //
    // ⚠ THE DELAY IS A DURATION, AND A DURATION IS TIMEBASE. It used to be
    // 4000 INSTRUCTIONS, which is not a duration at all: it means 2.4 ms of
    // guest time at --fast-tb 60 and 200 µs at --fast-tb 4, and under
    // real-time pacing — where the clock comes from the host and the guest has
    // no instruction RATE — it means nothing whatsoever. It also cannot
    // survive a run loop that charges several instructions at once, because an
    // instruction-denominated deadline is then overshot by a whole batch.
    // 200 µs is what the old constant bought at the shipping --fast-tb 4, and
    // it is what every configuration gets now.
    //
    // What the window is FOR is unchanged: it must be wider than the driver's
    // arming sequence (Open Firmware's ATA driver writes the command, then
    // stores to the cell's +0x200 control word some hundreds of instructions
    // later, then waits). 200 µs is ~4000 instructions at --fast-tb 4 and
    // ~11,000 through the app's real-time loop, against an arming sequence of
    // about 700 — and a real drive takes milliseconds, so erring wide is
    // erring towards the hardware.
    bool tick(); // true when a deferred command ran: wake the DMA list
    // The timebase at which tick() could next do something, so the machine
    // loop need not ask on every instruction. ~0 when nothing is deferred.
    u64 pendingTb() const
    {
        u64 t = pending_ ? pendAtTb_ : ~0ull;
        // A staged swap is also a timed event the machine loop must not
        // sleep through.
        if (!stagedPath_.empty() && ejected_) {
            const u64 due = ejectedAtTb_ +
                            (absentSeen_ ? kSwapDwellTb : kSwapFallbackTb);
            if (due < t)
                t = due;
        }
        return t;
    }
    static constexpr u64 kTbPerUs = 25; // 25 MHz timebase = bus clock / 4
    u64 cmdDelayTb_ = 200 * kTbPerUs;
    // Set by the bus from the channel.s RUN bit. On this hardware DMA is
    // selected by the mac-io cell and its DBDMA channel, NOT by the ATA
    // opcode: Open Firmware arms the channel and then issues an ordinary
    // READ MULTIPLE. Keying off the opcode made every such transfer assert
    // DRQ and wait for data-register reads that never came.
    bool dmaArmed_ = false;

    // DBDMA drain of the current data phase: same completion semantics
    // as PIO reads (chunked READs refill through finishPio).
    u32 dmaAvail() const
    {
        return static_cast<u32>(data_.size() - dataAt_);
    }
    u32 dmaTake(u8* dst, u32 n) override;
    // The other direction: memory -> drive, for WRITE DMA. Returns how many
    // bytes the drive accepted, which falls short only when the command's
    // data phase ends before the descriptor does.
    //
    // Its absence was invisible for as long as the only DMA device was the
    // CD: an ATAPI packet and its payload go through the task file here, so
    // OUTPUT descriptors were absorbed and marked complete and nothing was
    // lost. A hard disk's OUTPUT descriptors ARE the write, and absorbing one
    // loses a sector the guest believes it wrote — the drive is left holding
    // a data phase for bytes that never arrive, and the driver waits on a
    // transfer that can never finish. Initialising a disk stops there.
    u32 dmaGive(const u8* src, u32 n) override;
    // Whether this cell's OUTPUT descriptors carry real data. See dmaGive.
    bool dmaWriteSink() const override { return disk_; }

    struct Ev {
        u64 at;
        char kind; // c ata cmd, p packet op, e error path
        u8 val;
        u32 a = 0, b = 0; // READ ops: LBA + sector count
        u32 pc = 0;    // the instruction that issued it
        u32 xfer = 0;  // bytes the host actually pulled for this command
        // The whole CDB for packet ops. Logging only cdb[0] meant every
        // MMC failure read as a bare opcode: "READ TOC was refused" could
        // not be told from "READ TOC in MSF form with a 4-byte buffer was
        // refused", and those call for different fixes.
        u8 cdb[12] = {};
    };
    std::vector<Ev> log;
    // ⚠ TWO CLOCKS, AND THEY ANSWER DIFFERENT QUESTIONS. `stamp` is the
    // emulator's instruction counter and it is a LABEL: every log entry
    // carries it so a command can be correlated with the rest of a run's
    // instrumentation, all of which counts instructions. `tbRef` is the
    // machine's timebase and it is the CLOCK: every duration this cell models
    // is measured against it. Timing anything against the instruction counter
    // is what this cell used to do, and it made "how long does a command take"
    // a question with a different answer for every harness setting.
    const u64* stamp = nullptr;
    const u64* tbRef = nullptr;
    // Which code issued a command. The command log had timestamps but no
    // issuer, so "who asked for this sector" meant correlating two logs by
    // eye across a trimmed window.
    const u32* pcRef = nullptr;

    // Snapshot. The backing FILE* is deliberately NOT state: every access
    // seeks explicitly before reading, so re-attaching the same image on
    // resume restores the device completely. snapLoad keeps the live handle
    // and the stamp pointer, and reports a present/absent mismatch.
    void snapSave(SnapWriter& w) const;
    // strictMedia: the run must attach the very image the snapshot had (the
    // boot disk, the CD). A drive whose medium is a host-built volume that
    // no resume can reproduce loads with strictMedia=false: whatever is in
    // it now stays, and a mismatch leaves the tray EJECTED so the guest
    // finds out the honest way, on its next poll.
    void snapLoad(SnapReader& r, bool strictMedia = true);

private:
    void checkCondition(u8 key, u8 asc, u8 ascq);
    static bool needsMedium(u8 op);
    void ataCommand(u8 cmd);
    void packet(const u8* cdb);
    void finishPio(bool moreData); // present data_ via DRQ or complete
    u32 lba32(const u8* p) const
    {
        return (u32(p[2]) << 24) | (u32(p[3]) << 16) | (u32(p[4]) << 8) |
               p[5];
    }

    // Cell interrupt register at +0x300 (KeyLargo and later only).
    u32 intRegRead(u32 lane, u32 len) const;
    void diskCommand(u8 cmd);  // ATA (non-packet) command set
    void diskStartRead();      // present the next sector run through DRQ
    // Commit the sector now filling data_ and advance. Shared by the data
    // register and the DBDMA write path: "the host delivered a sector" means
    // the same thing whichever of the two carried the bytes.
    bool commitWriteSector();
    // The transfer address out of the task file. Bit 6 of the device
    // register picks the mode, and BOTH are live on a real drive: Open
    // Firmware's driver sets it (dev = 0xE0) and addresses by LBA, while
    // Mac OS's ATA disk driver clears it (dev = 0xA0), programs the
    // translation with INITIALIZE DEVICE PARAMETERS and addresses by
    // cylinder/head/sector. Reading the task file as LBA unconditionally
    // is not an approximation, it is an OFF-BY-ONE at the only sector that
    // matters: CHS 0/0/1 is LBA 0, the Driver Descriptor Record, and it was
    // being served block 1 instead — so the OS found no 'ER' signature,
    // concluded the disk had no driver, and never touched it again. OF read
    // the same disk perfectly throughout, which is exactly why this looked
    // like an OS-side or device-tree problem.
    u32 diskLba() const
    {
        if (dev_ & 0x40u) // LBA28
            return (u32(dev_ & 0x0Fu) << 24) | (u32(bcHi_) << 16) |
                   (u32(bcLo_) << 8) | lba0_;
        // CHS: cylinder in the two byte-count registers, head in the low
        // nibble of the device register, and the sector number is 1-BASED.
        const u32 cyl = (u32(bcHi_) << 8) | bcLo_;
        const u32 h = curHeads_ ? curHeads_ : 1u;
        const u32 s = curSectors_ ? curSectors_ : 1u;
        return (cyl * h + (dev_ & 0x0Fu)) * s +
               (lba0_ ? u32(lba0_) - 1u : 0u);
    }

    // The hard-disk path holds its file here; a CD lives in media_, which
    // owns the format mapping (cue/raw/UDIF as well as plain). isoBytes_
    // is the 2048-view size either way — it is what the snapshot stream
    // records, so it must stay derivable from the attached image alone.
    FILE* iso_ = nullptr;
    u64 isoBytes_ = 0;
    CdImage media_;
    bool disk_ = false;    // ATA hard disk rather than ATAPI CD
    u64 diskSectors_ = 0;  // 512-byte sectors
    u32 wrLeft_ = 0;       // sectors still to be written this command
    u64 wrLba_ = 0;

    // task file — the ATAPI signature (01 01 14 EB) is presented from
    // power-on, not only after an explicit reset
    u8 features_ = 0, nsect_ = 0x01, lba0_ = 0x01, bcLo_ = 0x14,
       bcHi_ = 0xEB, dev_ = 0, status_ = 0x40, error_ = 0x01, devctl_ = 0;
    // PIO engine
    std::vector<u8> data_;
    size_t dataAt_ = 0;
    bool cdbPhase_ = false;
    u8 cdb_[12] = {};
    u32 cdbAt_ = 0;
    // multi-sector reads stream in chunks the byte-count limit allows
    u64 readLba_ = 0;
    u32 readLeft_ = 0;
    u8 sense_ = 0; // last sense key
    u8 asc_ = 0, ascq_ = 0; // ...and its additional sense code, for REQUEST SENSE
    // Media-change state (see the public block above). Existing refusal
    // paths keep reporting ASC 0, exactly as they always have; only the
    // media conditions carry a real ASC, because the driver acts on those.
    bool seated_ = false;        // a drive, whether or not a disc is in it
    bool ejected_ = false;       // media_ is open but the disc is out
    bool locked_ = false;        // PREVENT ALLOW MEDIUM REMOVAL, as the guest left it
    bool noDoorLock_ = false;    // PREVENT is refused with ILLEGAL REQUEST (experiment)
    bool unitAttention_ = false; // a disc arrived: the next command hears about it
    bool absentSeen_ = false;    // the guest was told "no medium" since the eject
    std::string stagedPath_;     // hostSwap: the disc to insert once the tray was seen empty
    std::string mediaPath_;      // the image in the tray (or ejected from it)
    u64 ejectedAtTb_ = 0;
    u32 insertions_ = 0;
    u32 guestEjects_ = 0;
    u64 readsSinceInsert_ = 0;   // 2048-byte blocks served from the current disc
    u32 pulled_ = 0; // data-register bytes served since the last command
    // Deferred command: the write lands, BSY goes up, and the command runs
    // cmdDelayTb_ timebase ticks later. See write() case 0x070.
    bool pending_ = false;
    u8 pendCmd_ = 0;
    u64 pendAtTb_ = 0;
    // Task file as it stood when the command byte landed.
    u32 latchShown_ = 0;
    u8 pendNsect_ = 0, pendLba0_ = 0, pendBcLo_ = 0, pendBcHi_ = 0,
       pendDev_ = 0;
    // The pc that WROTE the command byte. The command log sampled the live
    // pc inside ataCommand(), which runs from tick() a whole command delay
    // later — so its "issuer" column was whatever code happened to be
    // executing thousands of instructions afterwards, and it was quoted as
    // the driver's address.
    u32 pendPc_ = 0;
    // Sectors per DRQ block, from SET MULTIPLE MODE. Advertised as 16 in
    // IDENTIFY word 59 and then never recorded, so the drive could not
    // report what it had agreed to.
    u8 multiple_ = 0;
    // The CURRENT translation, which INITIALIZE DEVICE PARAMETERS sets and
    // every CHS transfer is measured in. Power-on values are the geometry
    // IDENTIFY advertises in words 3 and 6, so a driver that programs what
    // it was told changes nothing.
    u8 curHeads_ = 16, curSectors_ = 63;
    bool irq_ = false;
    // Latched DMA-complete bit of the +0x300 interrupt register. Set when
    // this cell's DBDMA channel raises its interrupt, cleared only by the
    // host writing a 1 back.
    bool dmaIrqLatch_ = false;
    // Cell (not drive) registers at +0x200 and up: PIO/DMA timing. They sit
    // in mac-io, so drive select and an absent slave are irrelevant to them
    // and they always read back what was written.
    u32 ctl_[16] = {};
};

} // namespace opm
