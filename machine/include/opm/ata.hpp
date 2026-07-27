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

    // DBDMA drain of the current data phase: same completion semantics
    // as PIO reads (chunked READs refill through finishPio).
    u32 dmaAvail() const
    {
        return static_cast<u32>(data_.size() - dataAt_);
    }
    u32 dmaTake(u8* dst, u32 n);

    struct Ev {
        u64 at;
        char kind; // 'c' ata cmd, 'p' packet op, 'e' error path
        u8 val;
        u32 a = 0, b = 0; // READ ops: LBA + sector count
    };
    std::vector<Ev> log;
    const u64* stamp = nullptr;

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
    bool irq_ = false;
};

} // namespace opm
