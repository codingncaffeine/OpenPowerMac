// framereplay — draw a recorded frame again, on the card model alone.
//
// A frame recording (opm/r128rec.hpp; `g4run --frame-record`, or `record` in
// tools/remote.py) holds the graphics card as it stood and everything that
// reached it over a few frames. This replays one into a card with no machine
// around it and writes the picture at every frame edge:
//
//   framereplay FILE [--out DIR] [--strict]
//       DIR/frame-K.png for each frame, DIR/end.png for the last state.
//       Says, per frame, whether video memory came out as the recording
//       says it stood. --strict: exit 1 unless every one did.
//   framereplay --diff A.png B.png [MAP.png]
//       Count the pixels that differ between two pictures this tool wrote,
//       and paint them red over a dimmed A in MAP.png.
//
// The use: record once, replay with the build before a change to the drawing
// code and with the build after it, and --diff the two. The input is the same
// to the byte, so every differing pixel is the change's.

#include "opm/r128rec.hpp"

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

using namespace opm;

namespace {

u32 crcTable[256];
void crcInit()
{
    for (u32 n = 0; n < 256; ++n) {
        u32 c = n;
        for (int k = 0; k < 8; ++k)
            c = (c & 1u) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
        crcTable[n] = c;
    }
}
u32 crc32(const u8* p, size_t n, u32 crc = 0xFFFFFFFFu)
{
    for (size_t i = 0; i < n; ++i)
        crc = crcTable[(crc ^ p[i]) & 0xFFu] ^ (crc >> 8);
    return crc;
}
void be32(std::vector<u8>& o, u32 v)
{
    o.push_back(static_cast<u8>(v >> 24));
    o.push_back(static_cast<u8>(v >> 16));
    o.push_back(static_cast<u8>(v >> 8));
    o.push_back(static_cast<u8>(v));
}
void chunk(std::vector<u8>& o, const char* type, const std::vector<u8>& body)
{
    be32(o, static_cast<u32>(body.size()));
    const size_t at = o.size();
    o.insert(o.end(), type, type + 4);
    o.insert(o.end(), body.begin(), body.end());
    be32(o, crc32(o.data() + at, o.size() - at) ^ 0xFFFFFFFFu);
}

struct Picture {
    u32 w = 0, h = 0;
    std::vector<u8> rgb; // w*h*3
};

// A PNG with its pixels in stored (uncompressed) deflate blocks: any viewer
// reads it, and readPng below reads it back without an inflater.
bool writePng(const std::string& path, const Picture& pic)
{
    std::vector<u8> raw;
    raw.reserve(static_cast<size_t>(pic.h) * (pic.w * 3u + 1u));
    for (u32 y = 0; y < pic.h; ++y) {
        raw.push_back(0); // filter: none
        const u8* row = &pic.rgb[static_cast<size_t>(y) * pic.w * 3u];
        raw.insert(raw.end(), row, row + pic.w * 3u);
    }
    std::vector<u8> z{0x78, 0x01};
    u32 a = 1, b = 0; // adler32
    for (size_t at = 0; at < raw.size() || at == 0;) {
        const size_t n = raw.size() - at < 65535u ? raw.size() - at : 65535u;
        z.push_back(at + n >= raw.size() ? 1 : 0);
        z.push_back(static_cast<u8>(n));
        z.push_back(static_cast<u8>(n >> 8));
        z.push_back(static_cast<u8>(~n));
        z.push_back(static_cast<u8>(~n >> 8));
        z.insert(z.end(), raw.begin() + static_cast<long>(at),
                 raw.begin() + static_cast<long>(at + n));
        for (size_t i = at; i < at + n; ++i) {
            a = (a + raw[i]) % 65521u;
            b = (b + a) % 65521u;
        }
        at += n;
        if (!n)
            break;
    }
    be32(z, (b << 16) | a);
    std::vector<u8> o{0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
    std::vector<u8> ihdr;
    be32(ihdr, pic.w);
    be32(ihdr, pic.h);
    ihdr.insert(ihdr.end(), {8, 2, 0, 0, 0}); // 8 bits, RGB
    chunk(o, "IHDR", ihdr);
    chunk(o, "IDAT", z);
    chunk(o, "IEND", {});
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f)
        return false;
    const bool ok = std::fwrite(o.data(), 1, o.size(), f) == o.size();
    std::fclose(f);
    return ok;
}

bool readPng(const std::string& path, Picture& pic, std::string& err)
{
    std::vector<u8> d;
    {
        FILE* f = std::fopen(path.c_str(), "rb");
        if (!f) {
            err = "cannot read " + path;
            return false;
        }
        u8 buf[65536];
        size_t n;
        while ((n = std::fread(buf, 1, sizeof buf, f)) > 0)
            d.insert(d.end(), buf, buf + n);
        std::fclose(f);
    }
    auto g32 = [&](size_t at) {
        return (u32(d[at]) << 24) | (u32(d[at + 1]) << 16) |
               (u32(d[at + 2]) << 8) | d[at + 3];
    };
    std::vector<u8> z;
    for (size_t at = 8; at + 12 <= d.size();) {
        const u32 n = g32(at);
        if (at + 12 + n > d.size())
            break;
        if (!std::memcmp(&d[at + 4], "IHDR", 4) && n >= 13) {
            pic.w = g32(at + 8);
            pic.h = g32(at + 12);
            if (d[at + 16] != 8 || d[at + 17] != 2) {
                err = path + " is not an 8-bit RGB picture";
                return false;
            }
        } else if (!std::memcmp(&d[at + 4], "IDAT", 4)) {
            z.insert(z.end(), d.begin() + static_cast<long>(at + 8),
                     d.begin() + static_cast<long>(at + 8 + n));
        }
        at += 12 + n;
    }
    std::vector<u8> raw;
    for (size_t at = 2; at + 5 <= z.size();) {
        if (z[at] & 6u) {
            err = path + " was not written by this tool (compressed pixels)";
            return false;
        }
        const bool last = z[at] & 1u;
        const size_t n = z[at + 1] | (static_cast<size_t>(z[at + 2]) << 8);
        if (at + 5 + n > z.size())
            break;
        raw.insert(raw.end(), z.begin() + static_cast<long>(at + 5),
                   z.begin() + static_cast<long>(at + 5 + n));
        at += 5 + n;
        if (last)
            break;
    }
    const size_t stride = static_cast<size_t>(pic.w) * 3u + 1u;
    if (!pic.w || !pic.h || raw.size() != stride * pic.h) {
        err = path + " is not a whole picture";
        return false;
    }
    pic.rgb.resize(static_cast<size_t>(pic.w) * pic.h * 3u);
    for (u32 y = 0; y < pic.h; ++y)
        std::memcpy(&pic.rgb[static_cast<size_t>(y) * pic.w * 3u],
                    &raw[y * stride + 1], pic.w * 3u);
    return true;
}

// What the display shows of this card's video memory, without the pointer.
bool scanOut(const R128Cell& c, Picture& pic)
{
    const R128Scan s = r128ScanDecode(c);
    if (!s.enabled || !s.bypp || !s.w || !s.h)
        return false;
    pic.w = s.w;
    pic.h = s.h;
    pic.rgb.resize(static_cast<size_t>(s.w) * s.h * 3u);
    std::vector<u8> row(static_cast<size_t>(s.w) * 4u);
    for (u32 y = 0; y < s.h; ++y) {
        r128ScanRow(c, s, y, row.data());
        u8* o = &pic.rgb[static_cast<size_t>(y) * s.w * 3u];
        for (u32 x = 0; x < s.w; ++x) { // BGRA in, RGB out
            o[x * 3u] = row[x * 4u + 2u];
            o[x * 3u + 1u] = row[x * 4u + 1u];
            o[x * 3u + 2u] = row[x * 4u];
        }
    }
    return true;
}

int diff(const std::string& pa, const std::string& pb, const char* mapPath)
{
    Picture a, b;
    std::string err;
    if (!readPng(pa, a, err) || !readPng(pb, b, err)) {
        std::fprintf(stderr, "framereplay: %s\n", err.c_str());
        return 2;
    }
    if (a.w != b.w || a.h != b.h) {
        std::printf("the pictures are not the same size: %ux%u and %ux%u\n",
                    a.w, a.h, b.w, b.h);
        return 1;
    }
    u64 differ = 0;
    u32 worst = 0, x0 = a.w, y0 = a.h, x1 = 0, y1 = 0;
    Picture map = a;
    for (u32 y = 0; y < a.h; ++y)
        for (u32 x = 0; x < a.w; ++x) {
            const size_t at = (static_cast<size_t>(y) * a.w + x) * 3u;
            u32 step = 0;
            for (u32 k = 0; k < 3; ++k) {
                const u32 d = a.rgb[at + k] > b.rgb[at + k]
                                  ? u32(a.rgb[at + k]) - b.rgb[at + k]
                                  : u32(b.rgb[at + k]) - a.rgb[at + k];
                step = d > step ? d : step;
                map.rgb[at + k] = static_cast<u8>(a.rgb[at + k] / 3u);
            }
            if (!step)
                continue;
            ++differ;
            worst = step > worst ? step : worst;
            x0 = x < x0 ? x : x0;
            y0 = y < y0 ? y : y0;
            x1 = x > x1 ? x : x1;
            y1 = y > y1 ? y : y1;
            map.rgb[at] = 255;
            map.rgb[at + 1] = static_cast<u8>(255u - (step > 255u ? 255u : step));
            map.rgb[at + 2] = 0;
        }
    const u64 total = static_cast<u64>(a.w) * a.h;
    if (!differ) {
        std::printf("identical: %llu pixels\n",
                    static_cast<unsigned long long>(total));
        return 0;
    }
    std::printf("%llu of %llu pixels differ (%.2f%%), largest step %u of 255, "
                "inside (%u,%u)-(%u,%u)\n",
                static_cast<unsigned long long>(differ),
                static_cast<unsigned long long>(total),
                100.0 * static_cast<double>(differ) / static_cast<double>(total),
                worst, x0, y0, x1, y1);
    if (mapPath && !writePng(mapPath, map))
        std::fprintf(stderr, "framereplay: cannot write %s\n", mapPath);
    return 1;
}

const char* edgeName(char kind)
{
    switch (kind) {
    case 'P': return "presented";
    case 'C': return "before it is wiped";
    case 'F': return "display switched";
    default: return "?";
    }
}

} // namespace

int main(int argc, char** argv)
{
    crcInit();
    std::vector<std::string> a(argv + 1, argv + argc);
    if (a.size() >= 3 && a[0] == "--diff")
        return diff(a[1], a[2], a.size() > 3 ? a[3].c_str() : nullptr);
    std::string file, out = ".";
    bool strict = false;
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i] == "--out" && i + 1 < a.size())
            out = a[++i];
        else if (a[i] == "--strict")
            strict = true;
        else if (file.empty() && a[i].rfind("--", 0) != 0)
            file = a[i];
        else
            file.clear(), i = a.size();
    }
    if (file.empty()) {
        std::fprintf(stderr,
                     "usage: framereplay FILE [--out DIR] [--strict]\n"
                     "       framereplay --diff A.png B.png [MAP.png]\n");
        return 2;
    }
    std::error_code ec;
    std::filesystem::create_directories(out, ec);
    auto cell = std::make_unique<R128Cell>();
    std::vector<std::string> wrote;
    const R128EdgeFn onEdge = [&](u32 index, char, const R128Cell& c) {
        Picture pic;
        const std::string path =
            out + "/frame-" + std::to_string(index + 1) + ".png";
        wrote.push_back(scanOut(c, pic) && writePng(path, pic)
                            ? path
                            : std::string("(no picture: the display is off)"));
    };
    R128ReplayReport rep;
    std::string err;
    if (!r128Replay(file, *cell, onEdge, rep, err)) {
        std::fprintf(stderr, "framereplay: %s\n", err.c_str());
        return 2;
    }
    std::printf("%s: %llu stores and reads, %llu pages of system memory\n",
                file.c_str(), static_cast<unsigned long long>(rep.events),
                static_cast<unsigned long long>(rep.pages));
    bool allEqual = rep.edges == rep.edgesWanted;
    for (u32 k = 0; k < rep.edges; ++k) {
        const bool known = k < rep.wanted.size();
        const bool equal = known && rep.wanted[k] == rep.heard[k];
        allEqual = allEqual && equal;
        std::printf("  frame %u (%s): %s  %s\n", k + 1,
                    edgeName(rep.heard[k].first), wrote[k].c_str(),
                    !known ? "NOT in the recording"
                    : equal ? "video memory equals the recording"
                            : "video memory DIFFERS from the recording");
    }
    if (rep.edges != rep.edgesWanted)
        std::printf("  the recording has %u frame edges, the replay heard %u\n",
                    rep.edgesWanted, rep.edges);
    {
        Picture pic;
        const std::string path = out + "/end.png";
        const bool drew = scanOut(*cell, pic) && writePng(path, pic);
        std::printf("  end: %s  %s\n",
                    drew ? path.c_str() : "(no picture: the display is off)",
                    !rep.endRecorded ? "the recording has no end mark (cut short)"
                    : rep.endEqual ? "video memory equals the recording"
                                   : "video memory DIFFERS from the recording");
        allEqual = allEqual && rep.endRecorded && rep.endEqual;
    }
    const R128EngStats& es = r128EngStats();
    std::printf("  drawn: %llu triangles (%llu pixels, %llu texture samples), "
                "%llu 2D operations (%llu pixels); %llu ring and %llu "
                "indirect words, %llu memory-map misses\n",
                static_cast<unsigned long long>(es.tris),
                static_cast<unsigned long long>(es.triPixels),
                static_cast<unsigned long long>(es.texSamples),
                static_cast<unsigned long long>(es.blits),
                static_cast<unsigned long long>(es.pixels),
                static_cast<unsigned long long>(es.cceRingWords),
                static_cast<unsigned long long>(es.cceIndWords),
                static_cast<unsigned long long>(es.cceGartMiss +
                                                es.vtxGartMiss +
                                                es.texGartMiss));
    return strict && !allEqual ? 1 : 0;
}
