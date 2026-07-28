#include "opm/ata.hpp"

#include <cstring>

namespace opm {

// status bits
static constexpr u8 kBsy = 0x80, kDrdy = 0x40, kDrq = 0x08, kErr = 0x01;
// Seek-complete: ATA (non-packet) devices must assert it alongside DRDY;
// drivers wait on it during device detection. ATAPI devices do not.
static constexpr u8 kDsc = 0x10;
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

bool AtaCell::attachDisk(const char* path)
{
    iso_ = fopen(path, "r+b");
    if (!iso_)
        return false;
#ifdef _MSC_VER
    _fseeki64(iso_, 0, SEEK_END);
    isoBytes_ = static_cast<u64>(_ftelli64(iso_));
#else
    fseeko(iso_, 0, SEEK_END);
    isoBytes_ = static_cast<u64>(ftello(iso_));
#endif
    disk_ = true;
    diskSectors_ = isoBytes_ / 512u;
    // Plain ATA signature: no packet feature set, so the byte-count pair
    // stays zero where an ATAPI device answers 14 EB.
    nsect_ = 0x01;
    lba0_ = 0x01;
    bcLo_ = 0x00;
    bcHi_ = 0x00;
    status_ = kDrdy | kDsc;
    error_ = 0x01;
    return true;
}

void AtaCell::diskStartRead()
{
    if (readLeft_ == 0) {
        nsect_ = 0;
        status_ = (kDrdy | kDsc);
        irq_ = true;
        return;
    }
    // One sector per DRQ burst: the simplest presentation the taskfile
    // allows, and what the ROM driver's PIO loop expects.
    data_.assign(512, 0);
#ifdef _MSC_VER
    _fseeki64(iso_, static_cast<long long>(readLba_ * 512), SEEK_SET);
#else
    fseeko(iso_, static_cast<off_t>(readLba_ * 512), SEEK_SET);
#endif
    // A short read means the request ran past the end of the image. The
    // buffer is already zeroed, which is what a drive hands back for a
    // sector it cannot supply, so the transfer completes with zeros rather
    // than with whatever was in the buffer before.
    const size_t got = fread(data_.data(), 1, data_.size(), iso_);
    (void)got;
    dataAt_ = 0;
    ++readLba_;
    --readLeft_;
    status_ = kDrq | (kDrdy | kDsc);
    irq_ = true;
}

void AtaCell::diskCommand(u8 cmd)
{
    error_ = 0;
    switch (cmd) {
    case 0xEC: { // IDENTIFY DEVICE
        data_.assign(512, 0);
        auto w16 = [&](u32 word, u16 val) {
            // IDENTIFY words are stored LITTLE-ENDIAN in the buffer: the
            // mac-io data register is byte-reversed on the 60x side, so the
            // driver reads it with a byte-swapping load. Storing big-endian
            // put every value in the guest.s memory backwards - measured:
            // geometry 16383/16/63 arrived as ff3f/1000/3f00 and the serial
            // "OPM00000001" as "PO0M000000 1".
            data_[2 * word] = static_cast<u8>(val);
            data_[2 * word + 1] = static_cast<u8>(val >> 8);
        };
        auto text = [&](u32 word, u32 words, const char* s) {
            for (u32 k = 0; k < words * 2; ++k) {
                const char c = *s ? *s++ : ' ';
                // No swap: this buffer holds each word BIG-endian (see
                // w16 above, which writes the high byte first), and ATA
                // puts the first character in bits 15:8. Swapping here put
                // every character pair backwards, so the guest read the
                // model as "AMSTIHATC -DOR MRC1-57" — and the on-disc Apple
                // CD driver whitelists mechanisms BY MODEL STRING.
                // LE buffer: char0 goes in the HIGH byte of the word,
                // which is data_[2n+1] once the word is stored little-
                // endian. The transport swaps it back on the way out.
                data_[2 * word + (k ^ 1)] = static_cast<u8>(c);
            }
        };
        const u32 secs = static_cast<u32>(diskSectors_ > 0x0FFFFFFFull
                                              ? 0x0FFFFFFFull
                                              : diskSectors_);
        w16(0, 0x0040);  // non-removable ATA device
        w16(1, 16383);   // legacy CHS geometry, superseded by LBA
        w16(3, 16);
        w16(6, 63);
        text(10, 10, "OPM00000001");            // serial
        text(23, 4, "1.0 ");                    // firmware
        text(27, 20, "OpenPowerMac Hard Disk"); // model
        w16(47, 0x8010);                        // max sectors per interrupt
        // LBA and multiword DMA, which the channel's DBDMA engine at
        // mac-io +0x8A00 actually serves. Ultra DMA stays unadvertised: the
        // engine moves the bytes either way, and claiming a transfer mode
        // whose timing the cell does not model is the kind of capability
        // claim a driver acts on and we cannot honour.
        w16(49, 0x0300);                        // LBA + DMA supported
        w16(53, 0x0003);                        // words 54-58 and 64-70 valid
        w16(54, 16383);
        w16(55, 16);
        w16(56, 63);
        w16(57, static_cast<u16>(secs));
        w16(58, static_cast<u16>(secs >> 16));
        w16(59, 0x0100 | 16);       // multiple sectors currently set
        w16(60, static_cast<u16>(secs));
        w16(61, static_cast<u16>(secs >> 16));
        w16(63, 0x0007);            // multiword DMA 0-2 supported
        w16(51, 0x0200);            // PIO cycle timing mode
        w16(64, 0x0003);            // PIO modes 3 and 4
        w16(65, 120);               // min multiword DMA cycle
        w16(67, 120);               // min PIO cycle without flow control
        w16(68, 120);
        w16(80, 0x001E);            // ATA-1..ATA-4
        w16(82, 0x0000);
        w16(88, 0x0000);            // no ultra DMA
        dataAt_ = 0;
        nsect_ = 0;
        status_ = kDrq | (kDrdy | kDsc);
        irq_ = true;
        break;
    }
    case 0x20:
    case 0x21: // READ SECTOR(S)
    case 0xC4: // READ MULTIPLE
    case 0xC8: // READ DMA
    case 0xC9: // READ DMA (no retry)
        readLba_ = diskLba();
        readLeft_ = nsect_ ? nsect_ : 256u;
        diskStartRead();
        break;
    case 0x30:
    case 0x31: // WRITE SECTOR(S)
    case 0xC5: // WRITE MULTIPLE
    case 0xCA: // WRITE DMA
    case 0xCB: // WRITE DMA (no retry)
        wrLba_ = diskLba();
        wrLeft_ = nsect_ ? nsect_ : 256u;
        data_.assign(512, 0);
        dataAt_ = 0;
        status_ = kDrq | (kDrdy | kDsc); // host may deliver the first sector
        break;
    case 0x40:
    case 0x41: // READ VERIFY SECTOR(S)
    case 0x91: // INITIALIZE DEVICE PARAMETERS
    case 0xC6: // SET MULTIPLE MODE
    case 0xE7: // FLUSH CACHE
    case 0xEA: // FLUSH CACHE EXT
    case 0xEF: // SET FEATURES
    case 0x10: // RECALIBRATE
        status_ = (kDrdy | kDsc);
        irq_ = true;
        break;
    case 0x08:
    case 0x90: // DEVICE RESET / EXECUTE DEVICE DIAGNOSTIC
        nsect_ = 0x01;
        lba0_ = 0x01;
        bcLo_ = 0x00;
        bcHi_ = 0x00;
        error_ = 0x01;
        status_ = (kDrdy | kDsc);
        break;
    default:
        if (log.size() >= 4096)
            log.erase(log.begin(), log.begin() + 2048);
        log.push_back({stamp ? *stamp : 0, 'e', cmd, 0, 0, pcRef ? *pcRef : 0});
        error_ = 0x04; // ABRT
        status_ = (kDrdy | kDsc) | kErr;
        // An ABORTED command still completes, and completion is an
        // interrupt-generating event: the host waits on INTRQ to learn the
        // outcome, error or not. Every other arm raises it and this one did
        // not, so an unsupported command left the driver blocked forever.
        // Measured: the OS probes a non-packet drive with IDENTIFY PACKET
        // DEVICE (0xA1), which a hard disk must abort, and then never
        // touched the drive again - no status read, nothing.
        // ATA-4: a device that aborts an IDENTIFY meant for the other
        // device class must PLACE ITS SIGNATURE in the task file, which is
        // how the host classifies it. Without that the OS read a stale
        // signature, concluded this hard disk was a packet device, and sat
        // in a PACKET retry loop - 785 aborted commands in one boot.
        nsect_ = 0x01;
        lba0_ = 0x01;
        bcLo_ = 0x00;
        bcHi_ = 0x00;
        irq_ = true;
        break;
    }
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
    if (!present())
        return 0;
    // The timing/configuration registers at +0x200 and up live in the mac-io
    // ATA CELL, not on the drive. They are not on the data bus at all, so
    // neither drive select nor an absent slave has any bearing on them: they
    // answer whatever was last written, always. Treating them as drive
    // registers made them read back 0 (or 0x7F with device 1 selected)
    // instead of the value the firmware had just written, and Open
    // Firmware's ATA driver reads-modifies-writes them while setting up a
    // channel — so it wrote its timing word into a hole and read a
    // different one back. Same family as the DD7 truth, opposite direction:
    // that one is about what an ABSENT DRIVE drives; this is about a
    // register no drive drives at all.
    if (off >= 0x200u)
        return ctl_[(off - 0x200u) >> 4 & 15u];
    // No slave on this channel. Nothing drives the bus when device 1 is
    // selected, so the pull-ups win and every register reads all-ones —
    // EXCEPT on DD7, which the host pulls DOWN. DD7 carries the status
    // register.s BSY bit, so an absent device reads 0x7F rather than 0xFF:
    // BSY already clear, which is how a driver concludes "absent" at once
    // instead of waiting out a timeout on a bit that never falls.
    if (dev_ & 0x10u)
        return (~0u >> (32 - 8 * len)) & ~0x80u;
    switch (off & 0xFF0u) {
    case 0x000: { // data: PIO out of data_
        u32 v = 0;
        pulled_ += len;
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
    if (off >= 0x200u) { // cell timing registers — see read()
        ctl_[(off - 0x200u) >> 4 & 15u] = v;
        return;
    }
    const u8 b = static_cast<u8>(v);
    switch (off & 0xFF0u) {
    case 0x000: // data: CDB bytes, or hard-disk write data
        if (cdbPhase_) {
            for (u32 k = 0; k < len && cdbAt_ < 12; ++k)
                cdb_[cdbAt_++] = static_cast<u8>(v >> (8 * (len - 1 - k)));
            if (cdbAt_ >= 12) {
                cdbPhase_ = false;
                packet(cdb_);
            }
        } else if (disk_ && wrLeft_ && !data_.empty()) {
            for (u32 k = 0; k < len && dataAt_ < data_.size(); ++k)
                data_[dataAt_++] =
                    static_cast<u8>(v >> (8 * (len - 1 - k)));
            if (dataAt_ >= data_.size()) { // sector complete: commit it
#ifdef _MSC_VER
                _fseeki64(iso_, static_cast<long long>(wrLba_ * 512),
                          SEEK_SET);
#else
                fseeko(iso_, static_cast<off_t>(wrLba_ * 512), SEEK_SET);
#endif
                // A host write that does not land is a lost disk write, and
                // silently dropping one is how a guest filesystem corrupts
                // itself hours later. Report it the way a drive would.
                const bool wrote =
                    fwrite(data_.data(), 1, data_.size(), iso_) ==
                    data_.size();
                fflush(iso_);
                if (!wrote) {
                    error_ = 0x04; // ABRT
                    status_ = kDrdy | kDsc | kErr;
                    wrLeft_ = 0;
                    irq_ = true;
                    dataAt_ = 0;
                    return;
                }
                ++wrLba_;
                --wrLeft_;
                dataAt_ = 0;
                status_ = wrLeft_ ? (kDrq | kDrdy | kDsc) : (kDrdy | kDsc);
                irq_ = true;
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
        if (!(dev_ & 0x10u)) {
            // A real drive does not finish a command in zero time: it raises
            // BSY on the command write and only later clears BSY, sets DRQ
            // and asserts INTRQ. Completing instantly is not merely optimistic
            // — it inverts the ORDER a driver depends on. Open Firmware's ATA
            // driver writes the command, then arms its interrupt path (a
            // store to the cell's +0x200 control word ~700 instructions
            // later), then waits. With an instant device the whole transfer
            // was already done and the interrupt already consumed by the
            // status read before arming, so the wait never ended: measured on
            // the boot HD, eight 16 KiB READ MULTIPLEs succeeded and the
            // ninth hung with DRQ asserted and the driver polling forever.
            // Hold the command for a window wider than that arming sequence.
            if (!stamp) { // no clock to defer against: run it now
                ataCommand(b);
                break;
            }
            pendCmd_ = b;
            pendAt_ = (stamp ? *stamp : 0) + cmdDelay_;
            pending_ = true;
            status_ = kBsy;
        }
        break;
    case 0x160:
        devctl_ = b;
        if (b & 0x04u) {
            pending_ = false; // a reset cancels the command in flight
            // Software reset: the device places ITS OWN signature, and that
            // signature is how the host decides which command set to use.
            // This arm handed out the ATAPI 01 01 14 EB unconditionally, so
            // a hard disk announced itself as a packet device - after which
            // the OS correctly stopped trying IDENTIFY DEVICE and sat in a
            // PACKET retry loop instead. A non-packet device answers
            // 01 01 00 00.
            nsect_ = 0x01;
            lba0_ = 0x01;
            bcLo_ = disk_ ? 0x00 : 0x14;
            bcHi_ = disk_ ? 0x00 : 0xEB;
            status_ = disk_ ? static_cast<u8>(kDrdy | kDsc) : 0x40;
            error_ = 0x01;
        }
        break;
    default: break;
    }
}


void AtaCell::tick()
{
    if (!pending_ || !stamp || *stamp < pendAt_)
        return;
    pending_ = false;
    ataCommand(pendCmd_);
}
void AtaCell::ataCommand(u8 cmd)
{
    // Attribute the bytes the host pulled to the command that produced
    // them. The data register is deliberately absent from the register
    // trace, so "did the guest actually READ our IDENTIFY" had no answer
    // at all — and a driver that issues a command and never drains it
    // looks identical to one that read the data and disliked it.
    if (!log.empty())
        log.back().xfer = pulled_;
    pulled_ = 0;
    if (log.size() >= 4096)
        log.erase(log.begin(), log.begin() + 2048);
    if (true)
        log.push_back({stamp ? *stamp : 0, 'c', cmd, 0, 0, pcRef ? *pcRef : 0});
    error_ = 0;
    if (disk_) { // ATA hard disk: a different command set entirely
        diskCommand(cmd);
        return;
    }
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
            data_[2 * word] = static_cast<u8>(val);      // stored LE: the
            data_[2 * word + 1] = static_cast<u8>(val >> 8); // bus swaps
        };
        w16(0, 0x8580); // ATAPI, CD-ROM, removable, 12-byte CDB
        auto text = [&](u32 word, u32 words, const char* s) {
            for (u32 k = 0; k < words * 2; ++k) {
                const char c = *s ? *s++ : ' ';
                // No swap: this buffer holds each word BIG-endian (see
                // w16 above, which writes the high byte first), and ATA
                // puts the first character in bits 15:8. Swapping here put
                // every character pair backwards, so the guest read the
                // model as "AMSTIHATC -DOR MRC1-57" — and the on-disc Apple
                // CD driver whitelists mechanisms BY MODEL STRING.
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
            log.push_back({stamp ? *stamp : 0, 'e', cmd, 0, 0, pcRef ? *pcRef : 0});
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
    log.push_back({stamp ? *stamp : 0, 'p', cdb[0], 0, 0, pcRef ? *pcRef : 0});
    memcpy(log.back().cdb, cdb, 12);
    error_ = 0;
    data_.clear();
    dataAt_ = 0;
    readLeft_ = 0;
    auto complete = [&] {
        nsect_ = kIo | kCoD;
        status_ = kDrdy;
        // Command complete is an INTERRUPT, not just a status change. Without
        // it a driver that waits on INTRQ never learns the command finished:
        // the boot hung here after the CD driver's START STOP UNIT, spinning
        // on a parameter block whose ioResult stayed 1 (in progress) at
        // ffdd5764 for 2.5 billion instructions. Same defect shape as the
        // zero-latency command — the device did the work and never said so.
        irq_ = true;
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
    case 0x35: // SYNCHRONIZE CACHE — nothing is buffered here
    case 0xBB: // SET CD SPEED: a rate request, not a data transfer. Apple's
               // CD driver issues it during setup and treats the ABORT we
               // used to return as a hard failure — ATA Manager ExecIO
               // answered -9393 and the drive was rejected before use.
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
    case 0x43: { // READ TOC/PMA/ATIP
        // One data track plus the lead-out. Two fields were ignored here
        // and both are ones a real driver acts on: the MSF bit (cdb[1] bit
        // 1) selects min/sec/frame addresses instead of LBA, and the
        // allocation length (cdb[7..8]) caps the reply — a driver that asks
        // for four bytes and is handed twenty has been lied to about the
        // size of its own buffer.
        const bool msf = (cdb[1] & 0x02u) != 0;
        const u64 blocks = isoBytes_ / 2048;
        auto addr = [&](size_t at, u64 lba) {
            if (!msf) {
                data_[at] = static_cast<u8>(lba >> 24);
                data_[at + 1] = static_cast<u8>(lba >> 16);
                data_[at + 2] = static_cast<u8>(lba >> 8);
                data_[at + 3] = static_cast<u8>(lba);
                return;
            }
            const u64 f = lba + 150; // MSF counts from 00:02:00
            data_[at] = 0;
            data_[at + 1] = static_cast<u8>(f / (60 * 75));
            data_[at + 2] = static_cast<u8>((f / 75) % 60);
            data_[at + 3] = static_cast<u8>(f % 75);
        };
        data_.assign(20, 0);
        data_[1] = 18; // TOC data length, not counting these two bytes
        data_[2] = 1;  // first track
        data_[3] = 1;  // last track
        data_[5] = 0x14; // ADR 1, control 4: a data track
        data_[6] = 1;
        addr(8, 0);
        data_[13] = 0x14;
        data_[14] = 0xAA; // lead-out
        addr(16, blocks);
        const u32 alloc = (u32(cdb[7]) << 8) | cdb[8];
        if (alloc && alloc < data_.size())
            data_.resize(alloc);
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
        log.push_back(
            {stamp ? *stamp : 0, 'e', cdb[0], 0, 0, pcRef ? *pcRef : 0});
        memcpy(log.back().cdb, cdb, 12);
        sense_ = 0x05; // illegal request
        error_ = 0x04 | (0x05 << 4);
        nsect_ = kIo | kCoD;
        status_ = kDrdy | kErr;
        irq_ = true; // an aborted command still completes - see diskCommand
        return;
    }
    if (!data_.empty()) {
        nsect_ = kIo;
        bcLo_ = static_cast<u8>(data_.size());
        bcHi_ = static_cast<u8>(data_.size() >> 8);
        status_ = kDrq | kDrdy;
        irq_ = true; // data-ready is an interrupt too, per ATA/ATAPI-4 9.5
    } else {
        complete();
    }
}

// Data phase for READ: serve up to the host's byte-count limit per DRQ
// chunk; called again as the host drains each chunk.
void AtaCell::finishPio(bool chunkDrained)
{
    if (disk_) { // hard disk: one sector per DRQ burst, or completion
        diskStartRead();
        return;
    }
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
