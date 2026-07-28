#pragma once
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
class AtaCell {
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
    u8 devSel() const { return dev_; } // diagnostic: drive-select bits

    u32 read(u32 off, u32 len);
    void write(u32 off, u32 v, u32 len);

    // Run the deferred command, if its BSY window has elapsed. Called once
    // per instruction from the machine's peripheral tick. The delay is in
    // INSTRUCTIONS rather than timebase ticks on purpose: it exists to be
    // wider than a driver's arming sequence, which is a fixed number of
    // instructions, and it must not change meaning when the harness
    // compresses guest time.
    bool tick(); // true when a deferred command ran: wake the DMA list
    u64 cmdDelay_ = 4000;

    // DBDMA drain of the current data phase: same completion semantics
    // as PIO reads (chunked READs refill through finishPio).
    u32 dmaAvail() const
    {
        return static_cast<u32>(data_.size() - dataAt_);
    }
    u32 dmaTake(u8* dst, u32 n);

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
    const u64* stamp = nullptr;
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

    void diskCommand(u8 cmd);  // ATA (non-packet) command set
    void diskStartRead();      // present the next sector run through DRQ
    u32 diskLba() const        // LBA28 out of the task file
    {
        return (u32(dev_ & 0x0Fu) << 24) | (u32(bcHi_) << 16) |
               (u32(bcLo_) << 8) | lba0_;
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
    // cmdDelay_ instructions later. See write() case 0x070.
    bool pending_ = false;
    u8 pendCmd_ = 0;
    u64 pendAt_ = 0;
    bool irq_ = false;
    // Cell (not drive) registers at +0x200 and up: PIO/DMA timing. They sit
    // in mac-io, so drive select and an absent slave are irrelevant to them
    // and they always read back what was written.
    u32 ctl_[16] = {};
};

} // namespace opm
