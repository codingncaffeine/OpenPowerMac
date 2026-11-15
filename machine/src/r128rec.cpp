// The frame recorder and its replay: see opm/r128rec.hpp.

#include "opm/r128rec.hpp"

#include "opm/snapshot.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <utility>

namespace opm {

bool gR128RecOn = false;
void (*gR128FrameEdge)(R128Cell& c, char kind) = nullptr;

namespace {

constexpr char kMagic[8] = {'O', 'P', 'M', 'F', 'R', 'E', 'C', '1'};
constexpr u32 kPage = 4096;
// A recording that never meets a frame edge (a program that draws straight
// into the displayed surface without clearing it) ends here instead.
constexpr u64 kEventCap = 40'000'000;

struct Rec {
    FILE* f = nullptr;
    std::vector<u8> out; // bytes not yet in the file
    R128RecStatus st;
    // The store being carried out. Its record is written when it is done, so
    // the pages it made the engine read are ahead of it in the file.
    bool pending = false;
    char pKind = 'W';
    u32 pOff = 0, pVal = 0, pLen = 0;
    std::vector<std::pair<char, u64>> pEdges;
    // System memory as the file last described it: per page, the store it was
    // last compared at, and where its copy is kept.
    std::vector<u32> pageSeen, pageSlot;
    std::vector<u8> copies;
    u32 store = 1;
    u32 publishedAt = 0; // the store under which memory was last made current
    u32 gartBase = 0, aperBase = 0;
    bool wasAsync = false, wasStrict = false;
} gRec;

void put8(u8 v) { gRec.out.push_back(v); }
void put32(u32 v)
{
    const u8* p = reinterpret_cast<const u8*>(&v);
    gRec.out.insert(gRec.out.end(), p, p + 4);
}
void put64(u64 v)
{
    const u8* p = reinterpret_cast<const u8*>(&v);
    gRec.out.insert(gRec.out.end(), p, p + 8);
}
void flush(bool all)
{
    if (!gRec.f || (!all && gRec.out.size() < (1u << 20)))
        return;
    if (std::fwrite(gRec.out.data(), 1, gRec.out.size(), gRec.f) !=
        gRec.out.size())
        gRec.st.note = "the file could not be written";
    gRec.st.bytes += gRec.out.size();
    gRec.out.clear();
}

void recEdge(R128Cell& c, char kind)
{
    gRec.pEdges.emplace_back(kind, r128VramHash(c));
}

void finish(R128Cell& c, const char* why)
{
    put8('Z');
    put64(r128VramHash(c));
    flush(true);
    if (gRec.f)
        std::fclose(gRec.f);
    gRec.f = nullptr;
    gR128RecOn = false;
    gR128FrameEdge = nullptr;
    gRec.pending = false;
    gRec.pEdges.clear();
    gRec.pageSeen = {};
    gRec.pageSlot = {};
    gRec.copies = {};
    gRec.st.on = false;
    gRec.st.finished = true;
    if (gRec.st.note.empty())
        gRec.st.note = why;
    r128AsyncEnable(gRec.wasAsync, gRec.wasStrict);
}

} // namespace

u64 r128VramHash(const R128Cell& c)
{
    u64 h = 0x9E3779B97F4A7C15ull;
    const u8* p = c.vram.data();
    const size_t n = c.vram.size() & ~static_cast<size_t>(7);
    for (size_t i = 0; i < n; i += 8) {
        u64 w;
        std::memcpy(&w, p + i, 8);
        h = (h ^ w) * 0x100000001B3ull;
        h ^= h >> 29;
    }
    return h;
}

bool r128RecStart(R128Cell& c, const CceMem& m, u32 frames,
                  const std::string& path, std::string& err)
{
    if (gR128RecOn) {
        err = "a recording is already running (" + gRec.st.path + ")";
        return false;
    }
    if (!frames) {
        err = "no frames asked for";
        return false;
    }
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) {
        err = "cannot write " + path;
        return false;
    }
    gRec = Rec{};
    gRec.f = f;
    gRec.st.on = true;
    gRec.st.framesWanted = frames;
    gRec.st.path = path;
    gRec.gartBase = m.gartBase;
    gRec.aperBase = m.aperBase;
    gRec.pageSeen.assign((static_cast<size_t>(m.ramSize) + kPage - 1) / kPage,
                         0);
    gRec.pageSlot.assign(gRec.pageSeen.size(), 0);
    gRec.wasAsync = r128AsyncEnabled();
    gRec.wasStrict = r128AsyncStrict();
    r128AsyncEnable(false);

    SnapWriter cell, eng;
    c.snapSave(cell);
    r128EngineStateSave(eng);
    gRec.out.insert(gRec.out.end(), kMagic, kMagic + 8);
    put32(m.ramSize);
    put32(m.gartBase);
    put32(m.aperBase);
    put64(cell.buf.size());
    gRec.out.insert(gRec.out.end(), cell.buf.begin(), cell.buf.end());
    put64(eng.buf.size());
    gRec.out.insert(gRec.out.end(), eng.buf.begin(), eng.buf.end());
    flush(true);
    gR128FrameEdge = recEdge;
    gR128RecOn = true;
    return true;
}

void r128RecStop(R128Cell& c, const char* why)
{
    if (gR128RecOn)
        finish(c, why);
}

R128RecStatus r128RecStatus() { return gRec.st; }

void r128RecRegWrite(R128Cell&, char kind, u32 off, u32 v, u32 len,
                     const CceMem* m)
{
    if (m && (m->gartBase != gRec.gartBase || m->aperBase != gRec.aperBase)) {
        gRec.gartBase = m->gartBase;
        gRec.aperBase = m->aperBase;
        put8('G');
        put32(m->gartBase);
        put32(m->aperBase);
    }
    gRec.pending = true;
    gRec.pKind = kind;
    gRec.pOff = off;
    gRec.pVal = v;
    gRec.pLen = len;
    ++gRec.store; // the processor may have changed any page since the last one
}

void r128RecRegWriteDone(R128Cell& c)
{
    if (!gRec.pending)
        return;
    gRec.pending = false;
    put8(static_cast<u8>(gRec.pKind));
    put32(gRec.pOff);
    put32(gRec.pVal);
    put8(static_cast<u8>(gRec.pLen));
    ++gRec.st.events;
    for (const auto& [kind, hash] : gRec.pEdges) {
        put8('E');
        put8(static_cast<u8>(kind));
        put64(hash);
        ++gRec.st.frames;
    }
    gRec.pEdges.clear();
    flush(false);
    if (gRec.st.frames >= gRec.st.framesWanted)
        finish(c, "complete");
    else if (gRec.st.events >= kEventCap)
        finish(c, "stopped at the event cap: the program shows no frame edge");
}

void r128RecRegRead(u32 off, u32 len)
{
    put8('R');
    put32(off);
    put8(static_cast<u8>(len));
    ++gRec.st.events;
}

void r128RecFbWrite(u32 off, u32 v, u32 len)
{
    put8('F');
    put32(off);
    put32(v);
    put8(static_cast<u8>(len));
    ++gRec.st.events;
    flush(false);
}

void r128RecTouch(const u8* ram, u32 ramSize, u32 pa, u32 n, SnoopSink* snoop)
{
    if (!n)
        return;
    if (gRec.publishedAt != gRec.store) {
        gRec.publishedAt = gRec.store;
        if (snoop)
            snoop->publish(const_cast<u8*>(ram), ramSize);
    }
    const u32 last = (pa + n - 1) / kPage;
    for (u32 pg = pa / kPage; pg <= last; ++pg) {
        if (pg >= gRec.pageSeen.size() || gRec.pageSeen[pg] == gRec.store)
            continue;
        gRec.pageSeen[pg] = gRec.store;
        const size_t at = static_cast<size_t>(pg) * kPage;
        if (at + kPage > ramSize)
            continue;
        u32& slot = gRec.pageSlot[pg];
        if (slot &&
            !std::memcmp(&gRec.copies[static_cast<size_t>(slot - 1) * kPage],
                         ram + at, kPage))
            continue;
        if (!slot) {
            gRec.copies.resize(gRec.copies.size() + kPage);
            slot = static_cast<u32>(gRec.copies.size() / kPage);
        }
        std::memcpy(&gRec.copies[static_cast<size_t>(slot - 1) * kPage],
                    ram + at, kPage);
        put8('P');
        put32(pg);
        gRec.out.insert(gRec.out.end(), ram + at, ram + at + kPage);
        ++gRec.st.pages;
    }
}

void r128GuestRegWrite(R128Cell& c, u32 off, u32 v, u32 len, const CceMem& m)
{
    const bool rec = gR128RecOn;
    if (rec)
        r128RecRegWrite(c, 'W', off, v, len, &m);
    const u32 reg = off & ~3u;
    const u32 shownBefore = gR128FrameEdge && reg == 0x0224u ? c.peek(reg) : 0u;
    {
        R128RegLock lock;
        c.write(off, v, len);
    }
    // The card is a BUS MASTER, and these are the stores that make it act as
    // one. The walks are given the bus's memory from here rather than from
    // the cell, which is a leaf device with no route to it.
    //
    // 0A50: writing the transfer address starts a fetch out of system memory.
    if (reg == 0x0A50u)
        r128BusMasterFetch(c, m.ram, m.ramSize, m.snoop);
    // The CCE's PIO packet FIFO: each store hands the command engine one
    // DWORD of packet stream; executing it can dispatch an indirect buffer,
    // which is fetched out of system memory through the GART.
    else if (reg == 0x1000u || reg == 0x1004u)
        r128CceFifoWord(c, v, len, m.ram, m.ramSize, m.gartBase, m.aperBase,
                        m.snoop);
    // A guest that writes PM4_IW_INDSIZE directly (SDK 5.3.3's other flow)
    // dispatches the same way a packet-written one does.
    else if (reg == 0x073Cu)
        r128CceIndirect(c, c.peek(0x073Cu), m.ram, m.ramSize, m.gartBase,
                        m.aperBase, m.snoop);
    // The ring doorbell: advancing PM4_BUFFER_DL_WPTR, or starting the
    // microengine with work already queued, hands the card a span of ring
    // DWORDs to fetch and execute (SDK 5.3); the read pointer is written
    // back into system memory.
    else if (reg == 0x0714u || reg == 0x07FCu)
        r128CceRingKick(c, const_cast<u8*>(m.ram), m.ramSize, m.gartBase,
                        m.aperBase, m.snoop);
    // The display pointed at another surface: a page flip.
    if (gR128FrameEdge && reg == 0x0224u && c.peek(reg) != shownBefore)
        gR128FrameEdge(c, 'F');
    if (rec)
        r128RecRegWriteDone(c);
}

// ── Replay ────────────────────────────────────────────────────────────────

namespace {

struct ReplayCtx {
    const R128EdgeFn* onEdge = nullptr;
    std::vector<std::pair<char, u64>> heard;
};
ReplayCtx* gReplay = nullptr;

void replayEdge(R128Cell& c, char kind)
{
    if (!gReplay)
        return;
    const u32 index = static_cast<u32>(gReplay->heard.size());
    gReplay->heard.emplace_back(kind, r128VramHash(c));
    if (*gReplay->onEdge)
        (*gReplay->onEdge)(index, kind, c);
}

struct FreeDeleter {
    void operator()(void* p) const { std::free(p); }
};

} // namespace

bool r128Replay(const std::string& path, R128Cell& c, const R128EdgeFn& onEdge,
                R128ReplayReport& rep, std::string& err)
{
    rep = R128ReplayReport{};
    if (gR128RecOn) {
        err = "a recording is running";
        return false;
    }
    std::vector<u8> file;
    {
        FILE* f = std::fopen(path.c_str(), "rb");
        if (!f) {
            err = "cannot read " + path;
            return false;
        }
        std::fseek(f, 0, SEEK_END);
        const long size = std::ftell(f);
        std::fseek(f, 0, SEEK_SET);
        file.resize(size > 0 ? static_cast<size_t>(size) : 0);
        const size_t got = std::fread(file.data(), 1, file.size(), f);
        std::fclose(f);
        if (got != file.size()) {
            err = "short read of " + path;
            return false;
        }
    }
    SnapReader r(file.data(), file.size());
    char magic[8] = {};
    r.raw(magic, 8);
    if (!r.ok || std::memcmp(magic, kMagic, 8) != 0) {
        err = "not a frame recording: " + path;
        return false;
    }
    const u32 ramSize = r.u32v();
    CceMem m;
    m.gartBase = r.u32v();
    m.aperBase = r.u32v();
    auto blob = [&](SnapReader& into) {
        const u64 n = r.u64v();
        if (!r.ok || static_cast<u64>(r.end - r.p) < n) {
            r.fail("truncated");
            return;
        }
        into = SnapReader(r.p, static_cast<size_t>(n));
        r.p += n;
    };
    SnapReader cell, eng;
    blob(cell);
    blob(eng);
    if (!r.ok) {
        err = "the recording is cut short in its header";
        return false;
    }
    // The engine as the recording found it, on this thread.
    const bool wasAsync = r128AsyncEnabled(), wasStrict = r128AsyncStrict();
    r128AsyncEnable(false);
    r128CceReset();
    c.snapLoad(cell);
    r128EngineStateLoad(eng);
    if (!cell.ok || !eng.ok) {
        err = "the recording's starting state does not load: " +
              (cell.ok ? eng.err : cell.err);
        r128AsyncEnable(wasAsync, wasStrict);
        return false;
    }
    // System memory: zeroed, untouched pages cost nothing.
    std::unique_ptr<u8, FreeDeleter> ram(
        static_cast<u8*>(std::calloc(ramSize ? ramSize : 1u, 1)));
    if (!ram) {
        err = "no memory for the recorded machine's RAM";
        r128AsyncEnable(wasAsync, wasStrict);
        return false;
    }
    m.ram = ram.get();
    m.ramSize = ramSize;

    ReplayCtx ctx;
    ctx.onEdge = &onEdge;
    gReplay = &ctx;
    gR128FrameEdge = replayEdge;
    std::vector<std::pair<char, u64>> wanted;
    bool bad = false;
    const auto playFrom = std::chrono::steady_clock::now();
    while (r.ok && r.p < r.end && !bad) {
        const u8 tag = r.u8v();
        switch (tag) {
        case 'W': case 'w': {
            const u32 off = r.u32v(), v = r.u32v(), len = r.u8v();
            if (!r.ok)
                break;
            if (tag == 'W') {
                r128GuestRegWrite(c, off, v, len, m);
            } else {
                R128RegLock lock;
                c.write(off, v, len);
            }
            ++rep.events;
            break;
        }
        case 'R': {
            const u32 off = r.u32v(), len = r.u8v();
            if (r.ok)
                (void)c.read(off, len);
            ++rep.events;
            break;
        }
        case 'F': {
            const u32 off = r.u32v(), v = r.u32v(), len = r.u8v();
            if (r.ok && static_cast<u64>(off) + len <= c.vram.size())
                for (u32 k = 0; k < len; ++k) // big-endian, as the bus stores
                    c.vram[off + k] =
                        static_cast<u8>(v >> (8u * (len - 1u - k)));
            ++rep.events;
            break;
        }
        case 'P': {
            const u32 pg = r.u32v();
            const u64 at = static_cast<u64>(pg) * kPage;
            if (r.ok && at + kPage <= ramSize)
                r.raw(ram.get() + at, kPage);
            else
                bad = true;
            ++rep.pages;
            break;
        }
        case 'G':
            m.gartBase = r.u32v();
            m.aperBase = r.u32v();
            break;
        case 'E': {
            const char kind = static_cast<char>(r.u8v());
            wanted.emplace_back(kind, r.u64v());
            break;
        }
        case 'Z':
            rep.endRecorded = true;
            rep.endEqual = r.u64v() == r128VramHash(c);
            break;
        default:
            bad = true;
            break;
        }
    }
    rep.playNs = static_cast<u64>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - playFrom)
            .count());
    gR128FrameEdge = nullptr;
    gReplay = nullptr;
    r128AsyncEnable(wasAsync, wasStrict);
    if (bad || !r.ok) {
        err = "the recording is damaged at byte " +
              std::to_string(static_cast<size_t>(r.p - file.data()));
        return false;
    }
    rep.edges = static_cast<u32>(ctx.heard.size());
    rep.edgesWanted = static_cast<u32>(wanted.size());
    for (size_t k = 0; k < wanted.size() && k < ctx.heard.size(); ++k)
        if (wanted[k] == ctx.heard[k])
            ++rep.edgesEqual;
    rep.endHash = r128VramHash(c);
    rep.heard = std::move(ctx.heard);
    rep.wanted = std::move(wanted);
    return true;
}

} // namespace opm
