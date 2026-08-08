#pragma once
#include "opm/types.hpp"

#include <cstdio>
#include <string>
#include <vector>

namespace opm {

// One CD (or DVD) image in any of the shapes discs actually ship in. The
// ATA cell reads 2048-byte USER-DATA blocks by absolute LBA; this class
// owns the mapping from that LBA to bytes in one or more host files, which
// is the whole difference between the formats:
//
//   plain    .iso/.cdr/.toast/.img — 2048 bytes per sector, block N at
//            byte N*2048. Disk Utility's .cdr and most Toast images are
//            exactly this.
//   raw      .bin/.img/.mdf and raw Toast — 2352 bytes per sector as the
//            laser sees them: 12-byte sync mark, 4-byte header, then the
//            2048 user bytes (at +16 for mode 1, +24 for mode 2 form 1)
//            and error correction. Recognised by the sync mark itself, not
//            the file name, so a renamed image still opens.
//   cue      a text sheet naming one or more BINARY files and the track
//            table: per-track mode and sector size, pregaps, and where
//            audio sits. This is how mixed-mode discs (data + audio) are
//            distributed.
//   udif     Disk Utility's own .dmg: a 512-byte-sector device image cut
//            into chunks (raw, zero-filled, or zlib-compressed) behind an
//            XML chunk table and a 'koly' trailer. bzip2/ADC/LZFSE chunks
//            exist and are refused by name.
//
// Detection order: 'koly' trailer, then .cue extension, then the sync
// mark, then plain. Content outranks the extension everywhere but .cue,
// which has no magic.
struct CdTrack {
    u32 number = 1;
    bool audio = false;
    u32 startLba = 0;     // INDEX 01, absolute disc LBA
    u32 sectorBytes = 2048;
    u32 dataOff = 0;      // user data offset inside one stored sector
    u64 fileByte = 0;     // byte offset of startLba in its file
    size_t file = 0;      // which files_ entry carries it
    // Sectors of this track that exist as bytes in the file. LBAs past the
    // content but before the next track are cue PREGAP runout and read as
    // zeros, like the pregap of a pressed disc.
    u64 contentSectors = 0;
};

class CdImage {
public:
    ~CdImage() { close(); }
    CdImage() = default;
    CdImage(const CdImage&) = delete;
    CdImage& operator=(const CdImage&) = delete;

    bool open(const char* path); // false -> why() says what refused
    void close();
    bool ok() const { return !tracks_.empty(); }
    u64 blocks() const { return blocks_; } // total 2048-byte user LBAs
    // The identity a snapshot records for "the same image is attached".
    // Plain images report their exact byte size — snapshots predating the
    // other formats stored that, and the virtual partition dress must not
    // change an image's identity. Shaped images report the 2048 view.
    u64 snapBytes() const
    {
        return plain_ && !fileBytes_.empty() ? fileBytes_[0]
                                             : blocks_ * 2048;
    }
    const std::vector<CdTrack>& tracks() const { return tracks_; }
    const std::string& why() const { return err_; }
    const char* shape() const { return shape_; } // "2048"|"raw"|"cue"|"udif"

    // One 2048-byte user-data block. Returns false ONLY for an LBA inside
    // an audio track — a real drive answers READ(10) there with ILLEGAL
    // MODE FOR THIS TRACK, and serving silence instead would tell a
    // copy-check the disc is a different disc. Beyond-lead-out and pregap
    // LBAs read as zeros with true, which is what the plain path always
    // did (a short fread left the buffer zero-filled).
    bool readBlock(u64 lba, u8* dst);
    // The track whose region [startLba, next track) holds this LBA.
    // Never null once ok(): LBAs before track 1 answer track 1.
    const CdTrack* trackAt(u64 lba) const;
    // Whether READ(10) at this LBA must be refused as an audio sector.
    bool audioAt(u64 lba) const
    {
        const CdTrack* t = trackAt(lba);
        return t && t->audio && lba < blocks_;
    }

private:
    bool openPlainOrRaw(const char* path);
    bool openCue(const char* path);
    bool openUdif(FILE* f, u64 fileBytes);
    bool refuse(const std::string& msg); // set err_, close, return false
    // A bare HFS-family volume with no partition map gets a virtual
    // Driver Descriptor Map + Apple Partition Map in front — Mac OS 9's
    // CD probe was MEASURED refusing bare volumes with the "unreadable…
    // ProDOS 0K" dialog (see hfsBuildImage, which writes the same dress
    // for real). 16 synthesized LBAs; the file itself is untouched.
    void maybeWrapBareVolume();

    std::vector<FILE*> files_;
    std::vector<u64> fileBytes_;
    std::vector<CdTrack> tracks_;
    u64 blocks_ = 0;
    std::string err_;
    const char* shape_ = "2048";
    bool plain_ = false; // a plain 2048 file: snapBytes is its exact size
    // The synthesized partition dress, when a bare volume needed one.
    u32 apmLbas_ = 0;
    std::vector<u8> apm_;

    // UDIF: the chunk runs of every blkx table, sorted by start sector.
    struct UdifRun {
        u32 type = 0;       // 0 zero, 1 raw, 2 unallocated, 0x80000005 zlib
        u64 sector = 0;     // absolute 512-byte device sector
        u64 sectors = 0;
        u64 off = 0;        // byte offset of the stored bytes in the file
        u64 len = 0;        // stored byte length
    };
    std::vector<UdifRun> runs_;
    bool udif_ = false;
    // One materialised chunk: zlib runs decompress whole, so consecutive
    // reads out of the same chunk cost one inflate, not one per block.
    size_t cachedRun_ = ~size_t(0);
    std::vector<u8> cache_;
    bool udifRead512(u64 sector, u8* dst);
};

} // namespace opm
