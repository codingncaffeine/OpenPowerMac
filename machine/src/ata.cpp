#include "opm/ata.hpp"

#include <cstring>

namespace opm {

// status bits
static constexpr u8 kBsy = 0x80, kDrdy = 0x40, kDrq = 0x08, kErr = 0x01;
// interrupt-reason (nsect) bits during ATAPI phases
static constexpr u8 kCoD = 0x01, kIo = 0x02;

bool AtaCell::attachIso(const char* path)
{
    iso_ = fopen(path, "rb");
    if (!iso_)
        return false;
#ifdef _MSC_VER
    _fseeki64(iso_, 0, SEEK_END);
    isoBytes_ = static_cast<u64>(_ftelli64(iso_));
#else
    fseeko(iso_, 0, SEEK_END);
    isoBytes_ = static_cast<u64>(ftello(iso_));
#endif
    return true;
}

u32 AtaCell::dmaTake(u8* dst, u32 n)
{
    u32 moved = 0;
    while (moved < n && dataAt_ < data_.size())
        dst[moved++] = data_[dataAt_++];
    if (!data_.empty() && dataAt_ >= data_.size())
        finishPio(true); // chunk drained: stream more or complete
    return moved;
}

u32 AtaCell::read(u32 off, u32 len)
{
    if (!present() || (dev_ & 0x10u))
        return 0; // empty bus / absent slave: zeros, no BSY
    switch (off & 0xFF0u) {
    case 0x000: { // data: PIO out of data_
        u32 v = 0;
        for (u32 k = 0; k < len && dataAt_ < data_.size(); ++k)
            v = (v << 8) | data_[dataAt_++];
        if (dataAt_ >= data_.size())
            finishPio(true); // chunk drained: continue or complete
        return v;
    }
    case 0x010: return error_;
    case 0x020: return nsect_;
    case 0x030: return lba0_;
    case 0x040: return bcLo_;
    case 0x050: return bcHi_;
    case 0x060: return dev_;
    case 0x070:
        irq_ = false; // status read acknowledges INTRQ
        return status_;
    case 0x160: return status_;
    default: return 0;
    }
}

void AtaCell::write(u32 off, u32 v, u32 len)
{
    if (!present())
        return;
    const u8 b = static_cast<u8>(v);
    switch (off & 0xFF0u) {
    case 0x000: // data: CDB bytes arrive as 16-bit writes
        if (cdbPhase_) {
            for (u32 k = 0; k < len && cdbAt_ < 12; ++k)
                cdb_[cdbAt_++] = static_cast<u8>(v >> (8 * (len - 1 - k)));
            if (cdbAt_ >= 12) {
                cdbPhase_ = false;
                packet(cdb_);
            }
        }
        break;
    case 0x010: features_ = b; break;
    case 0x020: nsect_ = b; break;
    case 0x030: lba0_ = b; break;
    case 0x040: bcLo_ = b; break;
    case 0x050: bcHi_ = b; break;
    case 0x060: dev_ = b; break;
    case 0x070:
        irq_ = false;
        if (!(dev_ & 0x10u))
            ataCommand(b);
        break;
    case 0x160:
        devctl_ = b;
        if (b & 0x04u) { // SRST: ATAPI signature
            nsect_ = 0x01;
            lba0_ = 0x01;
            bcLo_ = 0x14;
            bcHi_ = 0xEB;
            status_ = 0x40;
            error_ = 0x01;
        }
        break;
    default: break;
    }
}

void AtaCell::ataCommand(u8 cmd)
{
    if (log.size() >= 4096)
        log.erase(log.begin(), log.begin() + 2048);
    if (true)
        log.push_back({stamp ? *stamp : 0, 'c', cmd});
    error_ = 0;
    switch (cmd) {
    case 0x08: // DEVICE RESET -> ATAPI signature
    case 0x90: // EXECUTE DEVICE DIAGNOSTIC
        nsect_ = 0x01;
        lba0_ = 0x01;
        bcLo_ = 0x14;
        bcHi_ = 0xEB;
        error_ = 0x01;
        status_ = 0x40;
        break;
    case 0xA1: { // IDENTIFY PACKET DEVICE
        data_.assign(512, 0);
        auto w16 = [&](u32 word, u16 val) {
            data_[2 * word] = static_cast<u8>(val >> 8); // served BE per
            data_[2 * word + 1] = static_cast<u8>(val);  // PIO byte order
        };
        w16(0, 0x8580); // ATAPI, CD-ROM, removable, 12-byte CDB
        auto text = [&](u32 word, u32 words, const char* s) {
            for (u32 k = 0; k < words * 2; ++k) {
                const char c = *s ? *s++ : ' ';
                data_[2 * word + (k ^ 1)] = static_cast<u8>(c);
            }
        };
        // The Mac OS CD driver whitelists mechanisms by model string
        // (the on-disc Apple_Driver_ATAPI carries "CD-ROM CR-175",
        // "CD-ROM FFREDDIE", "CD-211E", …) — present the drive the
        // Sawtooth shipped with or the boot volume never mounts.
        text(10, 10, "OPM00001");                 // serial
        text(23, 4, "7T02");                      // firmware
        text(27, 20, "MATSHITA CD-ROM CR-175 "); // model
        w16(49, 0x0200);                     // LBA supported
        irq_ = true;
        dataAt_ = 0;
        nsect_ = kIo | kCoD;
        bcLo_ = 512 & 0xFF;
        bcHi_ = 512 >> 8;
        status_ = kDrq | kDrdy;
        break;
    }
    case 0xA0: // PACKET: await 12 CDB bytes through the data register
        cdbPhase_ = true;
        cdbAt_ = 0;
        nsect_ = kCoD; // command phase
        status_ = kDrq;
        break;
    case 0xEF: // SET FEATURES
        status_ = kDrdy;
        irq_ = true;
        break;
    default:
        if (log.size() >= 4096)
        log.erase(log.begin(), log.begin() + 2048);
    if (true)
            log.push_back({stamp ? *stamp : 0, 'e', cmd});
        error_ = 0x04; // ABRT
        status_ = kDrdy | kErr;
        break;
    }
}

void AtaCell::packet(const u8* cdb)
{
    irq_ = true; // every packet resolution interrupts
    if (log.size() >= 4096)
        log.erase(log.begin(), log.begin() + 2048);
    if (true)
        log.push_back({stamp ? *stamp : 0, 'p', cdb[0]});
    error_ = 0;
    data_.clear();
    dataAt_ = 0;
    readLeft_ = 0;
    auto complete = [&] {
        nsect_ = kIo | kCoD;
        status_ = kDrdy;
    };
    switch (cdb[0]) {
    case 0x00: // TEST UNIT READY
        complete();
        break;
    case 0x03: { // REQUEST SENSE
        data_.assign(18, 0);
        data_[0] = 0x70;
        data_[2] = sense_;
        data_[7] = 10;
        sense_ = 0;
        break;
    }
    case 0x12: { // INQUIRY: same mechanism identity as IDENTIFY
        data_.assign(36, 0);
        data_[0] = 0x05; // CD-ROM
        data_[1] = 0x80; // removable
        data_[3] = 0x02;
        data_[4] = 31;
        memcpy(&data_[8], "MATSHITA", 8);
        memcpy(&data_[16], "CD-ROM CR-175   ", 16);
        memcpy(&data_[32], "7T02", 4);
        break;
    }
    case 0x1B: // START STOP UNIT
    case 0x1E: // PREVENT ALLOW MEDIUM REMOVAL
        complete();
        break;
    case 0x25: { // READ CAPACITY(10)
        const u64 blocks = isoBytes_ / 2048;
        const u32 last = blocks ? static_cast<u32>(blocks - 1) : 0;
        data_.assign(8, 0);
        data_[0] = static_cast<u8>(last >> 24);
        data_[1] = static_cast<u8>(last >> 16);
        data_[2] = static_cast<u8>(last >> 8);
        data_[3] = static_cast<u8>(last);
        data_[6] = 0x08; // 2048-byte blocks
        break;
    }
    case 0x28:   // READ(10)
    case 0xA8: { // READ(12)
        readLba_ = lba32(cdb);
        readLeft_ = cdb[0] == 0x28
                        ? ((u32(cdb[7]) << 8) | cdb[8])
                        : ((u32(cdb[6]) << 24) | (u32(cdb[7]) << 16) |
                           (u32(cdb[8]) << 8) | cdb[9]);
        log.back().a = static_cast<u32>(readLba_);
        log.back().b = readLeft_;
        finishPio(false);
        return;
    }
    case 0x2B: // SEEK
        complete();
        break;
    case 0x43: { // READ TOC: one data track at LBA 0
        data_.assign(20, 0);
        data_[1] = 18;
        data_[2] = 1;
        data_[3] = 1;
        data_[5] = 0x14;
        data_[6] = 1;
        data_[13] = 0x14;
        data_[14] = 0xAA; // lead-out
        const u64 blocks = isoBytes_ / 2048;
        data_[16] = static_cast<u8>(blocks >> 24);
        data_[17] = static_cast<u8>(blocks >> 16);
        data_[18] = static_cast<u8>(blocks >> 8);
        data_[19] = static_cast<u8>(blocks);
        break;
    }
    case 0x5A: { // MODE SENSE(10)
        const u8 page = cdb[2] & 0x3Fu;
        if (page == 0x2A || page == 0x3F) {
            // CD capabilities / mechanical status: a 4x-8x tray-loading
            // audio-capable reader, no write paths.
            data_.assign(8 + 20, 0);
            data_[1] = 26; // mode data length
            u8* p = &data_[8];
            p[0] = 0x2A;
            p[1] = 18;   // page bytes following
            p[2] = 0x01; // reads CD-R
            p[4] = 0x71; // multisession, mode2 form1/2, audio play
            p[5] = 0x03; // CD-DA commands, stream accurate
            p[6] = 0x29; // tray loader, eject, lock
            p[7] = 0x03; // separate volume + mute
            p[8] = 0x2B; // max speed 11024 KB/s (8x)
            p[9] = 0x10;
            p[10] = 0x01; // 256 volume levels
            p[11] = 0x00;
            p[12] = 0x00; // 128 KB buffer
            p[13] = 0x80;
            p[14] = 0x2B; // current speed = max
            p[15] = 0x10;
        } else {
            data_.assign(8, 0);
            data_[1] = 6;
        }
        break;
    }
    default:
        if (log.size() >= 4096)
        log.erase(log.begin(), log.begin() + 2048);
    if (true)
            log.push_back({stamp ? *stamp : 0, 'e', cdb[0]});
        sense_ = 0x05; // illegal request
        error_ = 0x04 | (0x05 << 4);
        nsect_ = kIo | kCoD;
        status_ = kDrdy | kErr;
        return;
    }
    if (!data_.empty()) {
        nsect_ = kIo;
        bcLo_ = static_cast<u8>(data_.size());
        bcHi_ = static_cast<u8>(data_.size() >> 8);
        status_ = kDrq | kDrdy;
    } else {
        complete();
    }
}

// Data phase for READ: serve up to the host's byte-count limit per DRQ
// chunk; called again as the host drains each chunk.
void AtaCell::finishPio(bool chunkDrained)
{
    if (chunkDrained && readLeft_ == 0) { // non-read transfers end here
        nsect_ = kIo | kCoD;
        status_ = kDrdy;
        irq_ = true;
        return;
    }
    if (readLeft_ == 0)
        return;
    u32 limit = (u32(bcHi_) << 8) | bcLo_;
    if (limit == 0 || limit == 0xFFFFu)
        limit = 0xFE00;
    u32 sectors = limit / 2048;
    if (sectors == 0)
        sectors = 1;
    if (sectors > readLeft_)
        sectors = readLeft_;
    data_.assign(size_t(sectors) * 2048, 0);
#ifdef _MSC_VER
    _fseeki64(iso_, static_cast<long long>(readLba_ * 2048), SEEK_SET);
#else
    fseeko(iso_, static_cast<off_t>(readLba_ * 2048), SEEK_SET);
#endif
    const size_t got = fread(data_.data(), 1, data_.size(), iso_);
    (void)got;
    dataAt_ = 0;
    readLba_ += sectors;
    readLeft_ -= sectors;
    const u32 bytes = sectors * 2048;
    bcLo_ = static_cast<u8>(bytes);
    bcHi_ = static_cast<u8>(bytes >> 8);
    nsect_ = kIo;
    status_ = kDrq | kDrdy;
    irq_ = true;
}

} // namespace opm
