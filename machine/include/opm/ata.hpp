#pragma once
#include "opm/dbdma.hpp"
#include "opm/types.hpp"

#include <cstdio>
#include <string>
#include <vector>

namespace opm {

struct SnapWriter;
struct SnapReader;

// One mac-io ATA cell (Heathrow-lineage register file, stride 0x10)
// carrying a single ATAPI CD/DVD device as device 0, backed by an ISO
// image streamed from the host file. Built against the OF driver's own
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
    bool present() const { return iso_ != nullptr; }
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
    u64 pendingTb() const { return pending_ ? pendAtTb_ : ~0ull; }
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
    void snapLoad(SnapReader& r);

private:
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

    FILE* iso_ = nullptr;
    u64 isoBytes_ = 0;
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
