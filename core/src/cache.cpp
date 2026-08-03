// L1 data cache (UM ch.3): 32 KB, 8-way set-associative, 128 sets, 32-byte
// blocks, write-back with write-allocate; loads fill on miss, stores fill
// then merge. Write-through pages update the bus on every store and never
// allocate on a store miss; cache-inhibited pages bypass entirely, as does
// everything while HID0[DCE] is clear.
//
// FALSIFICATION RECEIPT (2026-07-26): the draft core had no data cache —
// "flat bus is coherent". The Gossamer boot ROM disproved it: HWInit runs
// dcbz-scratch (exception tables, memory-sizing rings, pointer blocks) in
// the cache BEFORE the Grackle's DRAM interface is configured, exactly as
// real silicon allows. Cache-as-RAM is an architectural contract here.
//
// The cache indexes and tags physical addresses (the 7400 L1 is PIPT for
// our purposes: lookups happen after translation).

#include "opm/cpu.hpp"
#include "opm/prof.hpp"

#include <bit>
#include <cstring>

#if defined(_M_X64) || defined(__x86_64__)
#include <immintrin.h>
#endif

namespace opm {

namespace {
inline u32 setOf(u32 pa) { return (pa >> 5) & 127u; }
inline u32 tagOf(u32 pa) { return pa >> 12; }
inline constexpr u32 kWimgW = 8u;
inline constexpr u32 kWimgI = 4u;

// ⭐ THE LINE'S BYTES, AS ONE ACCESS INSTEAD OF `len` OF THEM. A cache line
// holds the guest's memory in guest (big-endian) order, so assembling a value
// out of it was written the obvious way — `v = (v << 8) | b[o + i]` — and
// that loop is a runtime loop, because `len` reaches memRead as a parameter.
// Every 32-bit load in the machine therefore cost four dependent
// load-shift-or steps, and every store four shift-and-store steps, on the
// HOTTEST path this emulator has (mem ops are 15.9% of the instructions the
// game executes and a far larger share of its time).
//
// These do the identical thing in one unaligned host access plus a byte swap.
// Identical, not equivalent-in-practice: the caller has already proved the
// access lies inside the 32-byte block ((pa & 31) + len <= 32), the bytes are
// the same bytes in the same order, and a byte swap is exactly what
// "big-endian assembly on a little-endian host" means. The default arm keeps
// the general loop, so a length nobody passes today still behaves.
inline u64 lineLoadBE(const u8* p, u32 len)
{
    switch (len) {
    case 1:
        return p[0];
    case 2: {
        u16 v;
        std::memcpy(&v, p, 2);
        return u16((v >> 8) | (v << 8));
    }
    case 4: {
        u32 v;
        std::memcpy(&v, p, 4);
        return ((v & 0xFFu) << 24) | ((v & 0xFF00u) << 8) |
               ((v >> 8) & 0xFF00u) | (v >> 24);
    }
    case 8: {
        u64 v;
        std::memcpy(&v, p, 8);
        v = ((v & 0x00FF00FF00FF00FFull) << 8) |
            ((v >> 8) & 0x00FF00FF00FF00FFull);
        v = ((v & 0x0000FFFF0000FFFFull) << 16) |
            ((v >> 16) & 0x0000FFFF0000FFFFull);
        return (v << 32) | (v >> 32);
    }
    default: {
        u64 v = 0;
        for (u32 i = 0; i < len; ++i)
            v = (v << 8) | p[i];
        return v;
    }
    }
}
inline void lineStoreBE(u8* p, u32 len, u64 v)
{
    switch (len) {
    case 1:
        p[0] = u8(v);
        return;
    case 2: {
        const u16 t = u16((u16(v) >> 8) | (u16(v) << 8));
        std::memcpy(p, &t, 2);
        return;
    }
    case 4: {
        const u32 s = u32(v);
        const u32 t = ((s & 0xFFu) << 24) | ((s & 0xFF00u) << 8) |
                      ((s >> 8) & 0xFF00u) | (s >> 24);
        std::memcpy(p, &t, 4);
        return;
    }
    case 8: {
        u64 t = ((v & 0x00FF00FF00FF00FFull) << 8) |
                ((v >> 8) & 0x00FF00FF00FF00FFull);
        t = ((t & 0x0000FFFF0000FFFFull) << 16) |
            ((t >> 16) & 0x0000FFFF0000FFFFull);
        t = (t << 32) | (t >> 32);
        std::memcpy(p, &t, 8);
        return;
    }
    default:
        for (u32 i = 0; i < len; ++i)
            p[i] = static_cast<u8>(v >> (8 * (len - 1 - i)));
        return;
    }
}
} // namespace

static void busWriteLine(Cpu& c, u32 base, const u8* b)
{
    // The block the fetch buffer may be holding is about to be rewritten in
    // memory; see Cpu::fetchBase.
    c.fetchDropAt(base);
    c.bus->writeLine32(base, b); // burst write: chipset caches may allocate
}

// ---- the L1 index ----------------------------------------------------------
//
// One entry per way: (tag << 1) | valid, so "is this the block and is it
// live" is a single compare, and all eight sit in one host cache line with
// their ages. See the DIdx comment in cpu.hpp for why.
inline u32 tvOf(u32 pa) { return (tagOf(pa) << 1) | 1u; }

// The set's ways, searched. Returns the way or -1. Two forms because the
// distinction was already load-bearing and used to be spelled out three
// times: an instruction-fetch peek must NOT touch the age, or the instrument
// reorders the replacement policy it is there to observe.
static int lineWayQuiet(const Cpu& c, u32 pa)
{
    const Cpu::DIdx& x = c.l1x[setOf(pa)];
    const u32 want = tvOf(pa);
#if defined(_M_X64) || defined(__x86_64__)
    const __m128i k = _mm_set1_epi32(static_cast<int>(want));
    const u32 m =
        u32(_mm_movemask_ps(_mm_castsi128_ps(_mm_cmpeq_epi32(
            _mm_load_si128(reinterpret_cast<const __m128i*>(&x.tv[0])), k)))) |
        (u32(_mm_movemask_ps(_mm_castsi128_ps(_mm_cmpeq_epi32(
             _mm_load_si128(reinterpret_cast<const __m128i*>(&x.tv[4])), k))))
         << 4);
    // A block is installed into one way only, so at most one bit is set; the
    // lowest is the way the old first-match loop would have returned anyway.
    return m ? static_cast<int>(std::countr_zero(m)) : -1;
#else
    for (u32 w = 0; w < 8; ++w)
        if (x.tv[w] == want)
            return static_cast<int>(w);
    return -1;
#endif
}

static int lineWay(Cpu& c, u32 pa)
{
    const int w = lineWayQuiet(c, pa);
    if (w >= 0)
        c.l1x[setOf(pa)].age[w] = ++c.l1dClock;
    return w;
}

// L1 castout on replacement: allocates into an enabled L2, else memory.
static void lineCastout(Cpu& c, Cpu::DLine& e, u32 set, u32 tag)
{
    const u32 base = (tag << 12) | (set << 5);
    if (c.l2On())
        c.l2Install(base, e.b, true);
    else
        busWriteLine(c, base, e.b);
    e.d = false;
}

// Explicit dcbf/dcbst push. With L2CR[L2TS] the block is written only into
// the L2 and marked valid (UM: the dcbz/dcbf L2-as-RAM idiom); otherwise it
// goes to memory and any L2 copy is invalidated.
static void linePush(Cpu& c, Cpu::DLine& e, u32 set, u32 tag)
{
    const u32 base = (tag << 12) | (set << 5);
    if (c.l2On() && (c.st.l2cr & 0x00040000u)) {
        c.l2Install(base, e.b, true);
    } else {
        busWriteLine(c, base, e.b);
        c.l2Invalidate(base);
    }
    e.d = false;
}

static void lineFill(Cpu& c, u32 set, u32 way, u32 pa)
{
    Cpu::DLine& e = c.l1d[set][way];
    const u32 base = pa & ~31u;
    if (!c.l2ReadLine(base, e.b)) {
        c.bus->readLine32(base, e.b); // burst read
        if (c.l2On())
            c.l2Install(base, e.b, false); // reloads allocate clean
    }
    c.l1x[set].tv[way] = tvOf(pa);
    e.d = false;
}

// The way a fill should land in: the first invalid one, else the least
// recently used, cast out if it is dirty. Same order and same choice the
// eight-way walk made.
static u32 lineVictim(Cpu& c, u32 pa)
{
    const u32 set = setOf(pa);
    Cpu::DIdx& x = c.l1x[set];
    u32 best = 0;
    for (u32 w = 0; w < 8; ++w) {
        if (!(x.tv[w] & 1u))
            return w;
        if (x.age[w] < x.age[best])
            best = w;
    }
    if (c.l1d[set][best].d)
        lineCastout(c, c.l1d[set][best], set, x.tv[best] >> 1);
    x.tv[best] = 0;
    return best;
}

// ---- backside L2 ----------------------------------------------------------

void Cpu::l2Resize()
{
    static const u32 kBytes[4] = {0x200000u, 0x40000u, 0x80000u, 0x100000u};
    const u32 sets = kBytes[(st.l2cr >> 28) & 3u] / 32u / 2u;
    if (sets != l2Sets) {
        // Re-sizing throws the array away, and with it any MODIFIED line that
        // memory has never seen. Cast those out first — dropping them loses
        // the only copy of that data.
        l2FlushAll(true);
        l2Sets = sets;
        l2.assign(size_t(sets) * 2u, L2Line{});
    }
}

Cpu::L2Line* Cpu::l2Find(u32 pa)
{
    if (!l2On() || l2Sets == 0 || l2Inert)
        return nullptr;
    const u32 set = l2SetOf(pa);
    const u32 tag = pa >> 5;
    for (u32 w = 0; w < 2; ++w) {
        L2Line& e = l2[size_t(set) * 2 + w];
        if (e.v && e.tag == tag) {
            e.age = ++l2Clock;
            return &e;
        }
    }
    return nullptr;
}

void Cpu::l2Install(u32 pa, const u8* bytes, bool dirty)
{
    if (!l2On())
        return;
    // Only a MODIFIED install can change what this block reads as; a clean
    // reload is a copy of what memory already says, so the fetch buffer keeps
    // its block — and keeping it matters, because a clean install happens on
    // every L1 data miss.
    if (dirty)
        fetchDropAt(pa);
    if (l2Inert) { // diagnostic: the array holds nothing, so a modified block
        if (dirty)  // has nowhere to live but memory
            busWriteLine(*this, pa, bytes);
        return;
    }
    if (l2Sets == 0)
        l2Resize();
    const u32 set = l2SetOf(pa);
    const u32 tag = pa >> 5;
    L2Line* dst = nullptr;
    for (u32 w = 0; w < 2; ++w) {
        L2Line& e = l2[size_t(set) * 2 + w];
        if (e.v && e.tag == tag) {
            dst = &e;
            break;
        }
    }
    bool hit = dst != nullptr;
    if (!dst) {
        L2Line& a = l2[size_t(set) * 2];
        L2Line& b = l2[size_t(set) * 2 + 1];
        dst = !a.v ? &a : (!b.v ? &b : (a.age <= b.age ? &a : &b));
        if (dst->v && dst->d)
            busWriteLine(*this, dst->tag << 5, dst->b); // L2 castout
    }
    for (u32 k = 0; k < 32; ++k)
        dst->b[k] = bytes[k];
    dst->tag = tag;
    dst->v = true;
    dst->d = dirty || (hit && dst->d);
    dst->age = ++l2Clock;
}

bool Cpu::l2ReadLine(u32 pa, u8* out)
{
    L2Line* e = l2Find(pa);
    if (!e)
        return false;
    for (u32 k = 0; k < 32; ++k)
        out[k] = e->b[k];
    return true;
}

void Cpu::l2Invalidate(u32 pa)
{
    fetchDropAt(pa);
    if (l2Sets == 0)
        return;
    const u32 set = l2SetOf(pa);
    const u32 tag = pa >> 5;
    for (u32 w = 0; w < 2; ++w) {
        L2Line& e = l2[size_t(set) * 2 + w];
        if (e.v && e.tag == tag)
            e.v = false;
    }
}

void Cpu::l2WipeAll()
{
    OPM_COUNT(fetchDropFlush);
    fetchDrop();
    for (auto& e : l2)
        e = L2Line{};
}

bool Cpu::l2Peek32(u32 pa, u32& w)
{
    L2Line* e = l2Find(pa);
    if (!e)
        return false;
    const u32 o = pa & 31u & ~3u;
    w = (u32(e->b[o]) << 24) | (u32(e->b[o + 1]) << 16) |
        (u32(e->b[o + 2]) << 8) | u32(e->b[o + 3]);
    return true;
}

void Cpu::l2FlushAll(bool writeback)
{
    OPM_COUNT(fetchDropFlush);
    fetchDrop();
    for (auto& e : l2) {
        if (e.v && e.d && writeback)
            busWriteLine(*this, e.tag << 5, e.b);
        e.v = false;
        e.d = false;
    }
}

// An access that goes round the caches while the L2 still holds the block is
// a place where the two can disagree. Recorded, not repaired, so that "the
// machine misbehaves once the L2 is on" can be attributed rather than guessed.
void Cpu::noteL2Skew(u32 pa, bool write)
{
    // NOT l2Find: that bumps the line's age, and a census that reorders the
    // replacement policy it is watching is the same class of mistake as a
    // flush that rewrites memory.
    if (!l2On() || l2Sets == 0 || l2Inert)
        return;
    const u32 base = pa & ~31u;
    const u32 set = l2SetOf(base), tag = base >> 5;
    bool held = false;
    for (u32 w = 0; w < 2 && !held; ++w) {
        const L2Line& e = l2[size_t(set) * 2 + w];
        held = e.v && e.tag == tag;
    }
    if (!held)
        return;
    if (write)
        ++l2SkewW;
    else
        ++l2SkewR;
    const u32 at = st.pc - 4u;
    auto it = l2SkewByPc.find(at);
    if (it != l2SkewByPc.end())
        ++it->second;
    else if (l2SkewByPc.size() < 512)
        l2SkewByPc[at] = 1;
}

u64 Cpu::memRead(u32 pa, u32 len, u32 wimg)
{
    OPM_MARK(Read);
    // READ watchpoint, as a census keyed by the READING instruction. Every
    // other watch in this project catches stores, which cannot answer "does
    // anything CONSUME this?" -- and that is the whole question for a device
    // buffer the machine fills and the guest is supposed to read. A log would
    // be useless (reads run to millions), so this counts hits per pc: bounded
    // by the number of distinct code sites rather than by traffic.
    if (rpEnd && pa <= rpEnd && pa + len > rpPa) {
        ++rpHits;
        const u32 at = st.pc - 4u;
        auto it = rpByPc.find(at);
        if (it != rpByPc.end())
            ++it->second;
        else if (rpByPc.size() < 512) // never let a runaway exhaust memory
            rpByPc[at] = 1;
        const u32 caller = st.lr;
        auto lit = rpByLr.find(caller);
        if (lit != rpByLr.end())
            ++lit->second;
        else if (rpByLr.size() < 512)
            rpByLr[caller] = 1;
    }
    if (!dceOn() || (wimg & kWimgI)) {
        noteL2Skew(pa, false);
        switch (len) {
        case 1: return bus->read8(pa);
        case 2: return bus->read16(pa);
        case 4: return bus->read32(pa);
        default: return bus->read64(pa);
        }
    }
    if ((pa & 31u) + len > 32u) { // crosses a block boundary: split
        u64 v = 0;
        for (u32 i = 0; i < len; ++i)
            v = (v << 8) | memRead(pa + i, 1, wimg);
        return v;
    }
    const u32 set = setOf(pa);
    int w = lineWay(*this, pa);
    if (w < 0) {
        w = static_cast<int>(lineVictim(*this, pa));
        lineFill(*this, set, static_cast<u32>(w), pa);
        l1x[set].age[w] = ++l1dClock;
    }
    return lineLoadBE(&l1d[set][w].b[pa & 31u], len);
}

bool Cpu::wpNote(u32 pa, u32 len, u64 v)
{
    if (!wpEnd || pa > wpEnd || pa + len <= wpPa)
        return false;
    if (wpFrom && wpStamp && *wpStamp < wpFrom)
        return false;
    ++wpHits;
    WpSpan& s = wpByPa[pa];
    ++s.n;
    if (len > s.len) s.len = len;
    ++wpByPc[st.pc - 4];
    if (wpLog.size() < wpMax)
        wpLog.push_back({st.pc - 4, pa, static_cast<u32>(v), len, st.lr, st.tb,
                         wpStamp ? *wpStamp : 0, st.gpr[24], decIrqs,
                         extIrqs});
    return true;
}

void Cpu::memWrite(u32 pa, u32 len, u64 v, u32 wimg)
{
    OPM_MARK(Write);
    // Self-modifying code, a ROM shadow being expanded, a page being paged
    // in: if this store lands in the block the fetch buffer holds, that block
    // is no longer what the buffer says. Checked here rather than in the
    // callers so no store path can miss it.
    fetchDropRange(pa, len);
    if (wpEnd && wpNote(pa, len, v) && wpForceSet && len == 4 && pa == wpPa)
        v = wpForce; // diagnostic
    if (!dceOn() || (wimg & kWimgI)) {
        noteL2Skew(pa, true);
        switch (len) {
        case 1: bus->write8(pa, static_cast<u8>(v)); return;
        case 2: bus->write16(pa, static_cast<u16>(v)); return;
        case 4: bus->write32(pa, static_cast<u32>(v)); return;
        default: bus->write64(pa, v); return;
        }
    }
    if ((pa & 31u) + len > 32u) {
        for (u32 i = 0; i < len; ++i)
            memWrite(pa + i, 1, (v >> (8 * (len - 1 - i))) & 0xFFu, wimg);
        return;
    }
    const u32 set = setOf(pa);
    int w = lineWay(*this, pa);
    if (wimg & kWimgW) { // write-through: no allocation on a store miss
        noteL2Skew(pa, true);
        if (w >= 0)
            lineStoreBE(&l1d[set][w].b[pa & 31u], len, v);
        switch (len) {
        case 1: bus->write8(pa, static_cast<u8>(v)); return;
        case 2: bus->write16(pa, static_cast<u16>(v)); return;
        case 4: bus->write32(pa, static_cast<u32>(v)); return;
        default: bus->write64(pa, v); return;
        }
    }
    if (w < 0) { // write-allocate
        w = static_cast<int>(lineVictim(*this, pa));
        lineFill(*this, set, static_cast<u32>(w), pa);
        l1x[set].age[w] = ++l1dClock;
    }
    lineStoreBE(&l1d[set][w].b[pa & 31u], len, v);
    l1d[set][w].d = true;
}

// Whole-block variant of l1dPeek32, for the instruction fetch buffer. Same
// precedence and the same dceOn() gate; it deliberately does NOT touch the
// line's age, exactly as the single-word peek does not — an instruction fetch
// that reordered the data cache's replacement policy would be an instrument
// changing the thing it measures.
bool Cpu::l1dReadLine(u32 pa, u8* out)
{
    if (!dceOn())
        return false;
    const int w = lineWayQuiet(*this, pa);
    if (w < 0)
        return false;
    const DLine& e = l1d[setOf(pa)][w];
    for (u32 i = 0; i < 32; ++i)
        out[i] = e.b[i];
    return true;
}

bool Cpu::l1dPeek32(u32 pa, u32& w)
{
    if (!dceOn())
        return false;
    const int k = lineWayQuiet(*this, pa);
    if (k < 0)
        return false;
    const DLine& e = l1d[setOf(pa)][k];
    const u32 o = pa & 31u & ~3u;
    w = (u32(e.b[o]) << 24) | (u32(e.b[o + 1]) << 16) |
        (u32(e.b[o + 2]) << 8) | u32(e.b[o + 3]);
    return true;
}

void Cpu::dcbzLine(u32 pa)
{
    fetchDropAt(pa);
    // A dcbz is a store of 32 zero bytes and it reaches memWrite by no path,
    // so a watchpoint on a field that a dcbz clears reported that field as
    // never written. That is this instrument's answer in its exact wrong
    // direction, and "why is this field zero" is the question being asked.
    if (wpEnd) wpNote(pa, 32u, 0);
    const u32 set = setOf(pa);
    int w = lineWay(*this, pa);
    if (w < 0) { // conjured, not filled: no bus read, the zeros ARE the block
        w = static_cast<int>(lineVictim(*this, pa));
        l1x[set].tv[w] = tvOf(pa);
        l1x[set].age[w] = ++l1dClock;
    }
    DLine& e = l1d[set][w];
    for (u8& x : e.b)
        x = 0;
    e.d = true; // zeros exist only in the cache until written back
}

void Cpu::dcbClean(u32 pa, bool invalidate)
{
    fetchDropAt(pa);
    const u32 set = setOf(pa);
    const int w = lineWay(*this, pa);
    if (w < 0)
        return;
    if (l1d[set][w].d)
        linePush(*this, l1d[set][w], set, l1x[set].tv[w] >> 1);
    if (invalidate)
        l1x[set].tv[w] = 0;
}

void Cpu::dcbKill(u32 pa)
{
    fetchDropAt(pa);
    const int w = lineWay(*this, pa);
    if (w >= 0)
        l1x[setOf(pa)].tv[w] = 0; // discarded, never written back
    l2Invalidate(pa & ~31u);
}

// Snoop response for a bus master. Unlike dcbf/dcbst this must NOT honour
// L2CR[L2TS]: that idiom parks a pushed block in the L2 instead of memory,
// and a DMA engine cannot see the L2 any more than it can see the L1. The
// data has to reach RAM.
//
// A master READ takes the push and leaves the line valid. A master WRITE
// takes the push and then kills the line, because the memory it describes
// is about to change underneath the processor.
void Cpu::snoopPush(u32 pa, u32 len, bool invalidate)
{
    // ⚠⚠ THIS USED TO THROW AWAY THE WHOLE FETCH BLOCK CACHE ON EVERY SNOOP,
    // READ OR WRITE. A master READ cannot change memory, so it cannot make a
    // fetched block stale; a master WRITE can, but only the block it lands
    // on. Measured at the Finder desktop, 300 M instructions: 2,043,750 full
    // drops — one wipe of all 64 lines every 147 instructions, almost all of
    // them the USB controller's per-frame descriptor walk — which held the
    // block cache at 84.5% hit with 16.6 M fills coming straight off the bus,
    // against 97.2% in the firmware era. Dropping per line inside the loop
    // below is exact, and it is the same claim the old code made, only
    // narrower.
    if (!len)
        return;
    if (invalidate)
        OPM_COUNT(fetchDropSnoop);
    const u32 first = pa & ~31u;
    const u32 last = (pa + len - 1u) & ~31u;
    for (u32 a = first;; a += 32u) {
        if (invalidate)
            fetchDropAt(a);
        if (dceOn()) {
            // Quiet: a snoop is a bus master's business and must not reorder
            // the processor's own replacement policy.
            const int w = lineWayQuiet(*this, a);
            if (w >= 0) {
                DLine& e = l1d[setOf(a)][w];
                if (e.d) {
                    busWriteLine(*this, a, e.b);
                    e.d = false;
                    // The L1 held the newest copy; any L2 copy is stale.
                    l2Invalidate(a);
                }
                if (invalidate)
                    l1x[setOf(a)].tv[w] = 0;
            }
        }
        if (L2Line* q = l2Find(a)) {
            if (q->d) {
                busWriteLine(*this, a, q->b);
                q->d = false;
            }
            if (invalidate)
                q->v = false;
        }
        if (a == last || a + 32u < a) // range done, or address wrap
            break;
    }
}

void CpuSnoop::snoopRead(u32 pa, u32 len)
{
    if (cpu)
        cpu->snoopPush(pa, len, false);
}

void CpuSnoop::snoopWrite(u32 pa, u32 len)
{
    if (cpu)
        cpu->snoopPush(pa, len, true);
}

// Harness/instrument coherence: dirty lines go straight to the bus so the
// observed memory image is the architectural one.
//
// ⚠ THE L1 LINE IS THE NEWER ONE, SO ITS L2 COPY MUST DIE WITH IT. A block
// can be dirty in BOTH caches at once — cast out of the L1 into the L2, then
// re-fetched and stored to again — and the L1 holds the later value. Writing
// the L1 back to memory and then letting l2FlushAll write the L2's older copy
// over it silently reverts the block. That made an arming flush a corrupting
// event rather than a neutral one, which is the worst thing an instrument can
// be: --trace-of at 150 M turned a run that died one way into a run that died
// another, and neither told the truth about the machine.
void Cpu::l1dFlushAll(bool writeback)
{
    OPM_COUNT(fetchDropFlush);
    fetchDrop();
    for (u32 s = 0; s < 128; ++s)
        for (u32 w = 0; w < 8; ++w) {
            DLine& e = l1d[s][w];
            const u32 tv = l1x[s].tv[w];
            if ((tv & 1u) && e.d) {
                const u32 base = ((tv >> 1) << 12) | (s << 5);
                if (writeback)
                    busWriteLine(*this, base, e.b);
                l2Invalidate(base);
            }
            l1x[s].tv[w] = 0;
            e.d = false;
        }
    l2FlushAll(writeback);
}

} // namespace opm
