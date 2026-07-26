#pragma once
#include "opm/types.hpp"

#include <cstdio>
#include <string>
#include <vector>

namespace opm {

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
    bool attachIso(const char* path);
    bool present() const { return iso_ != nullptr; }
    bool irqLine() const { return irq_; }

    u32 read(u32 off, u32 len);
    void write(u32 off, u32 v, u32 len);

    struct Ev {
        u64 at;
        char kind; // 'c' ata cmd, 'p' packet op, 'e' error path
        u8 val;
    };
    std::vector<Ev> log;
    const u64* stamp = nullptr;

private:
    void ataCommand(u8 cmd);
    void packet(const u8* cdb);
    void finishPio(bool moreData); // present data_ via DRQ or complete
    u32 lba32(const u8* p) const
    {
        return (u32(p[2]) << 24) | (u32(p[3]) << 16) | (u32(p[4]) << 8) |
               p[5];
    }

    FILE* iso_ = nullptr;
    u64 isoBytes_ = 0;

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
