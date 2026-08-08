#include "opm/cdimage.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cstring>

namespace opm {

namespace {

// ---------------------------------------------------------------- files --

void seek64(FILE* f, u64 off)
{
#ifdef _MSC_VER
    _fseeki64(f, static_cast<long long>(off), SEEK_SET);
#else
    fseeko(f, static_cast<off_t>(off), SEEK_SET);
#endif
}

u64 size64(FILE* f)
{
#ifdef _MSC_VER
    _fseeki64(f, 0, SEEK_END);
    return static_cast<u64>(_ftelli64(f));
#else
    fseeko(f, 0, SEEK_END);
    return static_cast<u64>(ftello(f));
#endif
}

// ------------------------------------------------------------ raw marks --

// The 12-byte sync mark that opens every raw 2352-byte sector. A plain
// 2048-byte image cannot begin with it (an HFS or ISO volume starts with
// boot blocks or zeros), so the mark plus a 2352-divisible size is a safe
// identification whatever the file is called.
const u8 kSync[12] = {0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                      0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};

u32 be32(const u8* p)
{
    return (u32(p[0]) << 24) | (u32(p[1]) << 16) | (u32(p[2]) << 8) | p[3];
}

u64 be64(const u8* p) { return (u64(be32(p)) << 32) | be32(p + 4); }

std::string lowerExt(const std::string& path)
{
    const size_t dot = path.find_last_of('.');
    const size_t sep = path.find_last_of("/\\");
    if (dot == std::string::npos ||
        (sep != std::string::npos && dot < sep))
        return "";
    std::string e = path.substr(dot + 1);
    for (char& c : e)
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return e;
}

// -------------------------------------------------------------- inflate --
//
// DEFLATE per RFC 1951 and the zlib wrapper per RFC 1950, written from the
// specifications. Disk Utility compresses UDIF chunks with zlib and there
// is no third-party code in this machine to hand the job to.

struct BitIn {
    const u8* p;
    size_t n, at = 0;
    u32 bit = 0;
    bool bad = false;
    u32 take(u32 count)
    {
        u32 v = 0;
        for (u32 k = 0; k < count; ++k) {
            if (at >= n) {
                bad = true;
                return 0;
            }
            v |= u32((p[at] >> bit) & 1u) << k;
            if (++bit == 8) {
                bit = 0;
                ++at;
            }
        }
        return v;
    }
    void align()
    {
        if (bit) {
            bit = 0;
            ++at;
        }
    }
};

// Canonical Huffman decode (RFC 1951 §3.2.2): count the codes of each bit
// length, then walk the incoming code one bit at a time against the count
// table. Slow and simple; a chunk is at most a megabyte.
struct Huff {
    u16 count[16] = {};
    u16 sym[288] = {};
    bool build(const u8* lens, u32 n)
    {
        for (u16& c : count)
            c = 0;
        for (u32 k = 0; k < n; ++k)
            ++count[lens[k]];
        count[0] = 0;
        // A over-subscribed code cannot decode; reject it here rather than
        // reading garbage.
        int left = 1;
        for (u32 l = 1; l < 16; ++l) {
            left <<= 1;
            left -= count[l];
            if (left < 0)
                return false;
        }
        u16 offs[16] = {};
        for (u32 l = 1; l < 15; ++l)
            offs[l + 1] = static_cast<u16>(offs[l] + count[l]);
        for (u32 k = 0; k < n; ++k)
            if (lens[k])
                sym[offs[lens[k]]++] = static_cast<u16>(k);
        return true;
    }
    int decode(BitIn& in) const
    {
        u32 code = 0, first = 0, index = 0;
        for (u32 len = 1; len < 16; ++len) {
            code |= in.take(1);
            const u32 c = count[len];
            if (code < first + c)
                return sym[index + (code - first)];
            index += c;
            first = (first + c) << 1;
            code <<= 1;
        }
        return -1;
    }
};

bool inflateRaw(const u8* src, size_t n, std::vector<u8>& out, size_t cap)
{
    static const u16 kLenBase[29] = {3,  4,  5,  6,  7,  8,  9,  10,
                                     11, 13, 15, 17, 19, 23, 27, 31,
                                     35, 43, 51, 59, 67, 83, 99, 115,
                                     131, 163, 195, 227, 258};
    static const u8 kLenExtra[29] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
                                     1, 1, 2, 2, 2, 2, 3, 3, 3, 3,
                                     4, 4, 4, 4, 5, 5, 5, 5, 0};
    static const u16 kDistBase[30] = {1,    2,    3,    4,    5,    7,
                                      9,    13,   17,   25,   33,   49,
                                      65,   97,   129,  193,  257,  385,
                                      513,  769,  1025, 1537, 2049, 3073,
                                      4097, 6145, 8193, 12289, 16385, 24577};
    static const u8 kDistExtra[30] = {0, 0, 0,  0,  1,  1,  2,  2,  3,  3,
                                      4, 4, 5,  5,  6,  6,  7,  7,  8,  8,
                                      9, 9, 10, 10, 11, 11, 12, 12, 13, 13};
    static const u8 kClOrder[19] = {16, 17, 18, 0, 8,  7, 9,  6, 10, 5,
                                    11, 4,  12, 3, 13, 2, 14, 1, 15};
    BitIn in{src, n};
    for (;;) {
        const u32 fin = in.take(1);
        const u32 type = in.take(2);
        if (in.bad)
            return false;
        if (type == 0) { // stored
            in.align();
            if (in.at + 4 > in.n)
                return false;
            const u32 len = u32(in.p[in.at]) | (u32(in.p[in.at + 1]) << 8);
            const u32 nlen =
                u32(in.p[in.at + 2]) | (u32(in.p[in.at + 3]) << 8);
            in.at += 4;
            if ((len ^ 0xFFFFu) != nlen || in.at + len > in.n ||
                out.size() + len > cap)
                return false;
            out.insert(out.end(), in.p + in.at, in.p + in.at + len);
            in.at += len;
        } else if (type == 1 || type == 2) {
            Huff lit, dist;
            if (type == 1) { // fixed tables, RFC 1951 §3.2.6
                u8 lens[288];
                for (u32 k = 0; k < 144; ++k)
                    lens[k] = 8;
                for (u32 k = 144; k < 256; ++k)
                    lens[k] = 9;
                for (u32 k = 256; k < 280; ++k)
                    lens[k] = 7;
                for (u32 k = 280; k < 288; ++k)
                    lens[k] = 8;
                if (!lit.build(lens, 288))
                    return false;
                u8 dlens[30];
                for (u32 k = 0; k < 30; ++k)
                    dlens[k] = 5;
                if (!dist.build(dlens, 30))
                    return false;
            } else { // dynamic tables, §3.2.7
                const u32 hlit = in.take(5) + 257;
                const u32 hdist = in.take(5) + 1;
                const u32 hclen = in.take(4) + 4;
                if (hlit > 286 || hdist > 30)
                    return false;
                u8 cl[19] = {};
                for (u32 k = 0; k < hclen; ++k)
                    cl[kClOrder[k]] = static_cast<u8>(in.take(3));
                Huff clh;
                if (in.bad || !clh.build(cl, 19))
                    return false;
                u8 lens[288 + 30] = {};
                u32 got = 0;
                while (got < hlit + hdist) {
                    const int s = clh.decode(in);
                    if (s < 0 || in.bad)
                        return false;
                    if (s < 16) {
                        lens[got++] = static_cast<u8>(s);
                    } else if (s == 16) {
                        if (got == 0)
                            return false;
                        const u8 prev = lens[got - 1];
                        u32 rep = 3 + in.take(2);
                        while (rep-- && got < hlit + hdist)
                            lens[got++] = prev;
                    } else {
                        u32 rep = s == 17 ? 3 + in.take(3) : 11 + in.take(7);
                        while (rep-- && got < hlit + hdist)
                            lens[got++] = 0;
                    }
                }
                if (!lit.build(lens, hlit) || !dist.build(lens + hlit, hdist))
                    return false;
            }
            for (;;) {
                const int s = lit.decode(in);
                if (s < 0 || in.bad)
                    return false;
                if (s < 256) {
                    if (out.size() >= cap)
                        return false;
                    out.push_back(static_cast<u8>(s));
                } else if (s == 256) {
                    break;
                } else {
                    if (s - 257 >= 29)
                        return false;
                    const u32 len =
                        kLenBase[s - 257] + in.take(kLenExtra[s - 257]);
                    const int d = dist.decode(in);
                    if (d < 0 || d >= 30 || in.bad)
                        return false;
                    const u32 back = kDistBase[d] + in.take(kDistExtra[d]);
                    if (back > out.size() || out.size() + len > cap)
                        return false;
                    // Byte-at-a-time on purpose: a distance shorter than
                    // the length repeats the just-written bytes.
                    for (u32 k = 0; k < len; ++k)
                        out.push_back(out[out.size() - back]);
                }
            }
        } else {
            return false;
        }
        if (fin)
            return !in.bad;
    }
}

u32 adler32(const u8* p, size_t n)
{
    u32 a = 1, b = 0;
    for (size_t k = 0; k < n; ++k) {
        a = (a + p[k]) % 65521u;
        b = (b + a) % 65521u;
    }
    return (b << 16) | a;
}

// zlib wrapper (RFC 1950): 2-byte header, deflate stream, big-endian
// adler32 of the plain text. The checksum is verified — a decompressor
// with a defect that is TRUSTED produces silently wrong disc sectors.
bool zlibInflate(const u8* src, size_t n, std::vector<u8>& out, size_t cap)
{
    if (n < 6 || (src[0] & 0x0F) != 8 || (src[1] & 0x20) != 0 ||
        ((u32(src[0]) << 8) | src[1]) % 31 != 0)
        return false;
    out.clear();
    out.reserve(cap);
    if (!inflateRaw(src + 2, n - 2, out, cap))
        return false;
    return adler32(out.data(), out.size()) == be32(src + n - 4);
}

// -------------------------------------------------------------- base64 --

bool base64Decode(const char* s, const char* end, std::vector<u8>& out)
{
    int have = 0;
    u32 acc = 0;
    for (; s != end; ++s) {
        const char c = *s;
        int v;
        if (c >= 'A' && c <= 'Z')
            v = c - 'A';
        else if (c >= 'a' && c <= 'z')
            v = c - 'a' + 26;
        else if (c >= '0' && c <= '9')
            v = c - '0' + 52;
        else if (c == '+')
            v = 62;
        else if (c == '/')
            v = 63;
        else if (c == '=' || c == '\n' || c == '\r' || c == ' ' ||
                 c == '\t')
            continue;
        else
            return false;
        acc = (acc << 6) | u32(v);
        if (++have == 4) {
            out.push_back(static_cast<u8>(acc >> 16));
            out.push_back(static_cast<u8>(acc >> 8));
            out.push_back(static_cast<u8>(acc));
            have = 0;
            acc = 0;
        }
    }
    if (have == 2)
        out.push_back(static_cast<u8>(acc >> 4));
    else if (have == 3) {
        out.push_back(static_cast<u8>(acc >> 10));
        out.push_back(static_cast<u8>(acc >> 2));
    } else if (have == 1)
        return false;
    return true;
}

// ------------------------------------------------------------ cue sheet --

// One whitespace-tokenised cue line, honouring "quoted names".
std::vector<std::string> cueTokens(const std::string& line)
{
    std::vector<std::string> t;
    size_t k = 0;
    while (k < line.size()) {
        while (k < line.size() &&
               std::isspace(static_cast<unsigned char>(line[k])))
            ++k;
        if (k >= line.size())
            break;
        if (line[k] == '"') {
            const size_t close = line.find('"', k + 1);
            if (close == std::string::npos) {
                t.push_back(line.substr(k + 1));
                break;
            }
            t.push_back(line.substr(k + 1, close - k - 1));
            k = close + 1;
        } else {
            size_t end = k;
            while (end < line.size() &&
                   !std::isspace(static_cast<unsigned char>(line[end])))
                ++end;
            t.push_back(line.substr(k, end - k));
            k = end;
        }
    }
    return t;
}

std::string upper(std::string s)
{
    for (char& c : s)
        c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

bool parseMsf(const std::string& s, u64& sectors)
{
    unsigned m, sec, f;
    if (sscanf(s.c_str(), "%u:%u:%u", &m, &sec, &f) != 3 || sec >= 60 ||
        f >= 75)
        return false;
    sectors = (u64(m) * 60 + sec) * 75 + f;
    return true;
}

} // namespace

// ----------------------------------------------------------------- open --

bool CdImage::refuse(const std::string& msg)
{
    close();
    err_ = msg;
    return false;
}

void CdImage::close()
{
    for (FILE* f : files_)
        if (f)
            fclose(f);
    files_.clear();
    fileBytes_.clear();
    tracks_.clear();
    runs_.clear();
    cache_.clear();
    cachedRun_ = ~size_t(0);
    blocks_ = 0;
    udif_ = false;
    shape_ = "2048";
    plain_ = false;
    apmLbas_ = 0;
    apm_.clear();
}

bool CdImage::open(const char* path)
{
    close();
    err_.clear();
    const bool opened = lowerExt(path) == "cue" ? openCue(path)
                                                : openPlainOrRaw(path);
    if (opened)
        maybeWrapBareVolume();
    return opened;
}

bool CdImage::openPlainOrRaw(const char* path)
{
    FILE* f = fopen(path, "rb");
    if (!f)
        return refuse("cannot open the image file");
    const u64 bytes = size64(f);
    // Encrypted UDIF announces itself at the FRONT of the file; everything
    // else here is identified from the back or the first sector.
    u8 head[16] = {};
    if (bytes >= 16) {
        seek64(f, 0);
        if (fread(head, 1, 16, f) != 16) {
            fclose(f);
            return refuse("cannot read the image file");
        }
        if (!memcmp(head, "encrcdsa", 8) || !memcmp(head, "cdsaencr", 8)) {
            fclose(f);
            return refuse("the image is an ENCRYPTED disk image — "
                          "decrypt it with Disk Utility first");
        }
    }
    if (bytes >= 512) {
        u8 koly[4];
        seek64(f, bytes - 512);
        if (fread(koly, 1, 4, f) == 4 && !memcmp(koly, "koly", 4)) {
            files_.push_back(f);
            fileBytes_.push_back(bytes);
            return openUdif(f, bytes);
        }
    }
    if (lowerExt(path) == "dmg") {
        fclose(f);
        return refuse("the .dmg has no 'koly' trailer — Disk Copy/NDIF "
                      "images are not supported; convert to ISO/UDIF "
                      "with Disk Utility");
    }
    CdTrack t;
    if (bytes >= 2352 && bytes % 2352 == 0 && !memcmp(head, kSync, 12)) {
        // Raw sectors under any name. The mode byte of the first header
        // picks where the 2048 user bytes sit.
        t.sectorBytes = 2352;
        t.dataOff = head[15] == 2 ? 24u : 16u;
        blocks_ = bytes / 2352;
        shape_ = "raw";
    } else {
        blocks_ = bytes / 2048;
        shape_ = "2048";
        plain_ = true;
    }
    t.contentSectors = blocks_;
    files_.push_back(f);
    fileBytes_.push_back(bytes);
    tracks_.push_back(t);
    return true;
}

bool CdImage::openCue(const char* cuePath)
{
    FILE* cf = fopen(cuePath, "rb");
    if (!cf)
        return refuse("cannot open the cue sheet");
    std::string text;
    {
        const u64 n = size64(cf);
        if (n > 1u << 20) {
            fclose(cf);
            return refuse("the cue sheet is not a cue sheet (over 1 MB "
                          "of text)");
        }
        text.resize(static_cast<size_t>(n));
        seek64(cf, 0);
        if (n && fread(&text[0], 1, text.size(), cf) != text.size()) {
            fclose(cf);
            return refuse("cannot read the cue sheet");
        }
        fclose(cf);
    }
    std::string dir(cuePath);
    {
        const size_t sep = dir.find_last_of("/\\");
        dir = sep == std::string::npos ? "" : dir.substr(0, sep + 1);
    }

    // The sheet, collected before any layout is computed.
    struct CueTrack {
        u32 number = 0;
        bool audio = false;
        u32 sectorBytes = 0, dataOff = 0;
        u64 idxFirst = ~0ull, idx01 = ~0ull; // file-relative sectors
        u64 pregap = 0;                      // virtual sectors before idx01
        size_t file = ~size_t(0);
    };
    std::vector<std::string> fileNames;
    std::vector<CueTrack> ts;

    size_t at = 0;
    int lineNo = 0;
    while (at < text.size()) {
        size_t nl = text.find('\n', at);
        if (nl == std::string::npos)
            nl = text.size();
        std::string line = text.substr(at, nl - at);
        at = nl + 1;
        ++lineNo;
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        const std::vector<std::string> tok = cueTokens(line);
        if (tok.empty())
            continue;
        const std::string kw = upper(tok[0]);
        auto fail = [&](const char* what) {
            char b[160];
            snprintf(b, sizeof b, "cue line %d: %s", lineNo, what);
            return refuse(b);
        };
        if (kw == "FILE") {
            if (tok.size() < 3)
                return fail("FILE needs a name and a type");
            const std::string type = upper(tok[2]);
            if (type != "BINARY" && type != "MOTOROLA")
                return fail("only BINARY files are supported (audio "
                            "WAVE/MP3 sheets are not)");
            fileNames.push_back(tok[1]);
        } else if (kw == "TRACK") {
            if (tok.size() < 3)
                return fail("TRACK needs a number and a mode");
            if (fileNames.empty())
                return fail("TRACK before any FILE");
            CueTrack t;
            t.number = static_cast<u32>(atoi(tok[1].c_str()));
            t.file = fileNames.size() - 1;
            const std::string mode = upper(tok[2]);
            if (mode == "AUDIO") {
                t.audio = true;
                t.sectorBytes = 2352;
            } else if (mode == "MODE1/2048") {
                t.sectorBytes = 2048;
            } else if (mode == "MODE1/2352") {
                t.sectorBytes = 2352;
                t.dataOff = 16;
            } else if (mode == "MODE2/2336") {
                t.sectorBytes = 2336;
                t.dataOff = 8;
            } else if (mode == "MODE2/2352") {
                t.sectorBytes = 2352;
                t.dataOff = 24;
            } else {
                return fail("unsupported track mode");
            }
            ts.push_back(t);
        } else if (kw == "INDEX") {
            if (tok.size() < 3 || ts.empty())
                return fail("INDEX outside a TRACK");
            u64 sec;
            if (!parseMsf(tok[2], sec))
                return fail("INDEX time is not mm:ss:ff");
            const int idx = atoi(tok[1].c_str());
            CueTrack& t = ts.back();
            if (t.idxFirst == ~0ull || sec < t.idxFirst)
                t.idxFirst = sec;
            if (idx == 1)
                t.idx01 = sec;
        } else if (kw == "PREGAP") {
            if (tok.size() < 2 || ts.empty())
                return fail("PREGAP outside a TRACK");
            u64 sec;
            if (!parseMsf(tok[1], sec))
                return fail("PREGAP time is not mm:ss:ff");
            ts.back().pregap = sec;
        }
        // REM, TITLE, PERFORMER, FLAGS, ISRC, CATALOG, POSTGAP: no effect
        // on where bytes live.
    }
    if (ts.empty())
        return refuse("the cue sheet names no tracks");
    for (const CueTrack& t : ts)
        if (t.idx01 == ~0ull) {
            char b[96];
            snprintf(b, sizeof b, "cue track %u has no INDEX 01",
                     t.number);
            return refuse(b);
        }

    // Open every named file, resolving relative to the sheet.
    for (const std::string& name : fileNames) {
        const bool abs =
            name.size() > 1 &&
            (name[0] == '/' || name[0] == '\\' || name[1] == ':');
        const std::string p = abs ? name : dir + name;
        FILE* f = fopen(p.c_str(), "rb");
        if (!f) {
            char b[512];
            snprintf(b, sizeof b, "the cue names a file that is not "
                                  "beside it: %s",
                     name.c_str());
            return refuse(b);
        }
        files_.push_back(f);
        fileBytes_.push_back(size64(f));
    }

    // Layout, one file at a time. INDEX times are file-relative sector
    // counts; each track's REGION runs from its first INDEX (00 if it has
    // a stored pregap, else 01) to the next track's first INDEX, and the
    // bytes of a region are that track's own sector size. A track's DATA
    // begins at INDEX 01: the stored pregap before it reads as zeros, and
    // a PREGAP directive inserts sectors the disc has but the file does
    // not.
    shape_ = "cue";
    for (size_t fi = 0; fi < files_.size(); ++fi) {
        std::vector<size_t> mine;
        for (size_t k = 0; k < ts.size(); ++k)
            if (ts[k].file == fi)
                mine.push_back(k);
        if (mine.empty())
            continue;
        const u64 discBase = blocks_;
        u64 virt = 0, byteCursor = 0, sectorCursor = 0;
        std::vector<u64> regionByte(mine.size(), 0);
        for (size_t m = 0; m < mine.size(); ++m) {
            const CueTrack& t = ts[mine[m]];
            const u64 regionStart =
                m == 0 ? 0 : std::min(t.idxFirst, t.idx01);
            if (regionStart < sectorCursor)
                return refuse("cue INDEX times run backwards");
            // Bytes between the previous region start and this one belong
            // to the PREVIOUS track and use its sector size.
            if (m > 0)
                byteCursor += (regionStart - sectorCursor) *
                              ts[mine[m - 1]].sectorBytes;
            sectorCursor = regionStart;
            regionByte[m] = byteCursor;
            if (t.idx01 < regionStart)
                return refuse("cue INDEX 01 precedes INDEX 00");
            virt += t.pregap;
            CdTrack out;
            out.number = t.number;
            out.audio = t.audio;
            out.sectorBytes = t.sectorBytes;
            out.dataOff = t.dataOff;
            out.file = fi;
            out.fileByte =
                byteCursor + (t.idx01 - regionStart) * t.sectorBytes;
            if (out.fileByte > fileBytes_[fi])
                return refuse("cue INDEX lies beyond the end of its file");
            out.startLba = static_cast<u32>(discBase + virt + t.idx01);
            tracks_.push_back(out);
        }
        // Content length: INDEX 01 to the end of the track's region — the
        // next track's region start, or the file end for the last track.
        // Sectors of the NEXT track's stored pregap are past this length
        // and read as zeros rather than as this track's data.
        const u64 fileEndByte = fileBytes_[fi];
        for (size_t m = 0; m < mine.size(); ++m) {
            CdTrack& out = tracks_[tracks_.size() - mine.size() + m];
            const u64 endByte =
                m + 1 < mine.size() ? regionByte[m + 1] : fileEndByte;
            if (endByte < out.fileByte)
                return refuse("cue tracks overlap");
            out.contentSectors = (endByte - out.fileByte) / out.sectorBytes;
        }
        const CdTrack& last = tracks_.back();
        blocks_ = last.startLba + last.contentSectors;
    }
    if (tracks_.empty())
        return refuse("the cue sheet names no usable tracks");
    return true;
}

// ------------------------------------------------------------------ UDIF --

bool CdImage::openUdif(FILE* f, u64 fileBytes)
{
    u8 koly[512];
    seek64(f, fileBytes - 512);
    if (fread(koly, 1, 512, f) != 512)
        return refuse("cannot read the UDIF trailer");
    const u32 version = be32(koly + 4);
    if (version != 4) {
        char b[96];
        snprintf(b, sizeof b, "UDIF version %u is not supported", version);
        return refuse(b);
    }
    const u64 dataFork = be64(koly + 24);
    const u64 xmlOff = be64(koly + 216);
    const u64 xmlLen = be64(koly + 224);
    const u64 kolySectors = be64(koly + 492);
    if (!xmlLen || xmlOff + xmlLen > fileBytes)
        return refuse("the UDIF has no XML chunk table (resource-fork "
                      "images are not supported)");
    std::string xml(static_cast<size_t>(xmlLen), 0);
    seek64(f, xmlOff);
    if (fread(&xml[0], 1, xml.size(), f) != xml.size())
        return refuse("cannot read the UDIF chunk table");

    // Every <data> block that follows a <key>blkx</key> is one partition's
    // 'mish' run table, base64-encoded.
    const size_t blkx = xml.find("<key>blkx</key>");
    if (blkx == std::string::npos)
        return refuse("the UDIF XML carries no blkx table");
    const size_t arrEnd = xml.find("</array>", blkx);
    size_t at = blkx;
    u64 maxSector = 0;
    while (true) {
        const size_t d0 = xml.find("<data>", at);
        if (d0 == std::string::npos ||
            (arrEnd != std::string::npos && d0 > arrEnd))
            break;
        const size_t d1 = xml.find("</data>", d0);
        if (d1 == std::string::npos)
            break;
        at = d1 + 7;
        std::vector<u8> mish;
        if (!base64Decode(xml.data() + d0 + 6, xml.data() + d1, mish))
            return refuse("the UDIF blkx data does not decode");
        if (mish.size() < 208 || be32(mish.data()) != 0x6D697368u)
            continue; // not a mish block (other keys carry <data> too)
        const u64 partStart = be64(mish.data() + 8);
        const u64 tableDataOff = be64(mish.data() + 24);
        const u32 chunks = be32(mish.data() + 200);
        if (mish.size() < 204 + size_t(chunks) * 40)
            return refuse("the UDIF chunk table is truncated");
        for (u32 k = 0; k < chunks; ++k) {
            const u8* c = mish.data() + 204 + size_t(k) * 40;
            UdifRun r;
            r.type = be32(c);
            r.sector = partStart + be64(c + 8);
            r.sectors = be64(c + 16);
            r.off = dataFork + tableDataOff + be64(c + 24);
            r.len = be64(c + 32);
            if (r.type == 0xFFFFFFFFu || r.type == 0x7FFFFFFEu)
                continue; // terminator / comment
            if (r.type != 0 && r.type != 1 && r.type != 2 &&
                r.type != 0x80000005u) {
                const char* what =
                    r.type == 0x80000004u   ? "ADC"
                    : r.type == 0x80000006u ? "bzip2"
                    : r.type == 0x80000007u ? "LZFSE"
                                            : "an unknown codec";
                char b[128];
                snprintf(b, sizeof b, "the UDIF uses %s-compressed chunks "
                                      "(type %08x) — convert with Disk "
                                      "Utility",
                         what, r.type);
                return refuse(b);
            }
            if (!r.sectors)
                continue;
            runs_.push_back(r);
            if (r.sector + r.sectors > maxSector)
                maxSector = r.sector + r.sectors;
        }
    }
    if (runs_.empty())
        return refuse("the UDIF chunk table is empty");
    std::sort(runs_.begin(), runs_.end(),
              [](const UdifRun& a, const UdifRun& b) {
                  return a.sector < b.sector;
              });
    const u64 devSectors = std::max(kolySectors, maxSector);
    blocks_ = devSectors / 4; // 512-byte device sectors -> 2048 view
    udif_ = true;
    shape_ = "udif";
    CdTrack t;
    t.contentSectors = blocks_;
    tracks_.push_back(t);
    return true;
}

bool CdImage::udifRead512(u64 sector, u8* dst)
{
    memset(dst, 0, 512);
    // Last run with start <= sector.
    size_t lo = 0, hi = runs_.size();
    while (lo < hi) {
        const size_t mid = (lo + hi) / 2;
        if (runs_[mid].sector <= sector)
            lo = mid + 1;
        else
            hi = mid;
    }
    if (lo == 0)
        return true; // before the first run: zeros
    const UdifRun& r = runs_[lo - 1];
    if (sector >= r.sector + r.sectors)
        return true; // a hole between runs: zeros
    const u64 rel = sector - r.sector;
    switch (r.type) {
    case 0: // zero-fill
    case 2: // unallocated
        return true;
    case 1: { // raw
        seek64(files_[0], r.off + rel * 512);
        const size_t got = fread(dst, 1, 512, files_[0]);
        (void)got; // short read past EOF stays zeros, like the plain path
        return true;
    }
    case 0x80000005u: { // zlib
        const size_t idx = lo - 1;
        if (cachedRun_ != idx) {
            std::vector<u8> comp(static_cast<size_t>(r.len));
            seek64(files_[0], r.off);
            if (fread(comp.data(), 1, comp.size(), files_[0]) !=
                comp.size())
                return false;
            std::vector<u8> plain;
            if (!zlibInflate(comp.data(), comp.size(), plain,
                             static_cast<size_t>(r.sectors * 512)))
                return false;
            plain.resize(static_cast<size_t>(r.sectors * 512), 0);
            cache_ = std::move(plain);
            cachedRun_ = idx;
        }
        memcpy(dst, cache_.data() + rel * 512, 512);
        return true;
    }
    }
    return true;
}

// ---------------------------------------------------- the partition dress --

// Mac OS 9's CD probe was MEASURED refusing a bare HFS+ and a bare
// classic-HFS volume alike with the "unreadable … ProDOS 0K" initialise
// dialog; what every pressed Mac disc carries — and the same probe then
// accepts — is a Driver Descriptor Map at block 0 and an Apple Partition
// Map naming an Apple_HFS partition (hfsBuildImage writes this dress for
// real, same constants). Disk Utility's own "Master CD" images are bare
// HFS+ (the Halo disc that motivated this), so a faithful block layer
// alone still ends at that dialog. The dress here is VIRTUAL: 16
// synthesized LBAs in front, the file untouched, the volume presented at
// the customary 512-block 64.
void CdImage::maybeWrapBareVolume()
{
    if (!ok() || blocks_ < 2 || tracks_.size() != 1 || tracks_[0].audio)
        return;
    // Only containers that hold a bare volume; cue/raw rips are mastered
    // discs and already carry whatever the mastering put there.
    if (strcmp(shape_, "2048") != 0 && strcmp(shape_, "udif") != 0)
        return;
    u8 b0[2048];
    if (!readBlock(0, b0))
        return;
    if (b0[0] == 'E' && b0[1] == 'R')
        return; // already dressed
    const u32 sig = (u32(b0[1024]) << 8) | b0[1025];
    // 'H+' HFS Plus, 'HX' HFSX, 'BD' classic HFS — a volume header where
    // a bare volume keeps it.
    if (sig != 0x482Bu && sig != 0x4858u && sig != 0x4244u)
        return;
    if (blocks_ > 16) {
        u8 b16[2048];
        if (readBlock(16, b16) && !memcmp(b16 + 1, "CD001", 5))
            return; // an ISO/hybrid master mounts by its own structures
    }
    apm_.assign(16u * 2048u, 0);
    auto p16 = [&](size_t at, u16 v) {
        apm_[at] = static_cast<u8>(v >> 8);
        apm_[at + 1] = static_cast<u8>(v);
    };
    auto p32 = [&](size_t at, u32 v) {
        apm_[at] = static_cast<u8>(v >> 24);
        apm_[at + 1] = static_cast<u8>(v >> 16);
        apm_[at + 2] = static_cast<u8>(v >> 8);
        apm_[at + 3] = static_cast<u8>(v);
    };
    const u64 vol512 = blocks_ * 4u; // the volume, in DDM's 512-blocks
    // Driver Descriptor Map: signature, 512-byte blocks, device total.
    p16(0, 0x4552); // 'ER'
    p16(2, 512);
    p32(4, static_cast<u32>(64u + vol512));
    // Two map entries, the customary shape: the map itself at 1..63 and
    // the volume at 64. Same fields hfsBuildImage writes.
    auto entry = [&](u32 slot, const char* name, const char* type,
                     u32 start, u32 count, u32 status) {
        const size_t at = slot * 512u;
        p16(at, 0x504D); // 'PM'
        p32(at + 4, 2);  // entries in the map
        p32(at + 8, start);
        p32(at + 12, count);
        snprintf(reinterpret_cast<char*>(&apm_[at + 16]), 32, "%s", name);
        snprintf(reinterpret_cast<char*>(&apm_[at + 48]), 32, "%s", type);
        p32(at + 84, count); // data count
        p32(at + 88, status);
    };
    entry(1, "Apple", "Apple_partition_map", 1, 63, 0x3u);
    entry(2, "Disc", "Apple_HFS", 64, static_cast<u32>(vol512),
          0x40000033u);
    apmLbas_ = 16;
    blocks_ += 16;
    tracks_[0].contentSectors = blocks_;
    shape_ = udif_ ? "udif+apm" : "2048+apm";
}

// ----------------------------------------------------------------- read --

const CdTrack* CdImage::trackAt(u64 lba) const
{
    if (tracks_.empty())
        return nullptr;
    const CdTrack* t = &tracks_[0];
    for (const CdTrack& k : tracks_)
        if (k.startLba <= lba)
            t = &k;
        else
            break;
    return t;
}

bool CdImage::readBlock(u64 lba, u8* dst)
{
    memset(dst, 0, 2048);
    if (lba >= blocks_)
        return true; // runout reads as zeros, as the plain path always did
    if (apmLbas_) { // the virtual partition dress sits in front
        if (lba < apmLbas_) {
            memcpy(dst, &apm_[static_cast<size_t>(lba) * 2048u], 2048);
            return true;
        }
        lba -= apmLbas_;
    }
    const CdTrack* t = trackAt(lba);
    if (!t)
        return true;
    if (t->audio)
        return false; // the caller answers ILLEGAL MODE FOR THIS TRACK
    if (udif_) {
        for (u32 k = 0; k < 4; ++k)
            if (!udifRead512(lba * 4 + k, dst + k * 512))
                return true; // a bad chunk reads as zeros, loudly nowhere —
                             // the open-time checks are the real gate
        return true;
    }
    if (lba < t->startLba)
        return true; // lead-in / pregap of the first track
    const u64 rel = lba - t->startLba;
    if (rel >= t->contentSectors)
        return true; // virtual pregap between tracks
    FILE* f = files_[t->file];
    seek64(f, t->fileByte + rel * t->sectorBytes + t->dataOff);
    const size_t got = fread(dst, 1, 2048, f);
    (void)got; // short read keeps zeros
    return true;
}

} // namespace opm
