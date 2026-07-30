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

namespace opm {

namespace {
inline u32 setOf(u32 pa) { return (pa >> 5) & 127u; }
inline u32 tagOf(u32 pa) { return pa >> 12; }
inline constexpr u32 kWimgW = 8u;
inline constexpr u32 kWimgI = 4u;
} // namespace

static void busWriteLine(Cpu& c, u32 base, const u8* b)
{
    c.bus->writeLine32(base, b); // burst write: chipset caches may allocate
}

// L1 castout on replacement: allocates into an enabled L2, else memory.
static void lineCastout(Cpu& c, Cpu::DLine& e, u32 set)
{
    const u32 base = (e.tag << 12) | (set << 5);
    if (c.l2On())
        c.l2Install(base, e.b, true);
    else
        busWriteLine(c, base, e.b);
    e.d = false;
}

// Explicit dcbf/dcbst push. With L2CR[L2TS] the block is written only into
// the L2 and marked valid (UM: the dcbz/dcbf L2-as-RAM idiom); otherwise it
// goes to memory and any L2 copy is invalidated.
static void linePush(Cpu& c, Cpu::DLine& e, u32 set)
{
    const u32 base = (e.tag << 12) | (set << 5);
    if (c.l2On() && (c.st.l2cr & 0x00040000u)) {
        c.l2Install(base, e.b, true);
    } else {
        busWriteLine(c, base, e.b);
        c.l2Invalidate(base);
    }
    e.d = false;
}

static void lineFill(Cpu& c, Cpu::DLine& e, u32 pa)
{
    const u32 base = pa & ~31u;
    if (!c.l2ReadLine(base, e.b)) {
        c.bus->readLine32(base, e.b); // burst read
        if (c.l2On())
            c.l2Install(base, e.b, false); // reloads allocate clean
    }
    e.tag = tagOf(pa);
    e.v = true;
    e.d = false;
}

static Cpu::DLine* lineFind(Cpu& c, u32 pa)
{
    const u32 set = setOf(pa), tag = tagOf(pa);
    for (u32 w = 0; w < 8; ++w) {
        Cpu::DLine& e = c.l1d[set][w];
        if (e.v && e.tag == tag) {
            e.age = ++c.l1dClock;
            return &e;
        }
    }
    return nullptr;
}

static Cpu::DLine& lineVictim(Cpu& c, u32 pa)
{
    const u32 set = setOf(pa);
    Cpu::DLine* best = &c.l1d[set][0];
    for (u32 w = 0; w < 8; ++w) {
        Cpu::DLine& e = c.l1d[set][w];
        if (!e.v)
            return e;
        if (e.age < best->age)
            best = &e;
    }
    if (best->d)
        lineCastout(c, *best, set);
    best->v = false;
    return *best;
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
    if (!l2On() || l2Sets == 0)
        return nullptr;
    const u32 set = (pa >> 5) % l2Sets;
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
    if (l2Sets == 0)
        l2Resize();
    const u32 set = (pa >> 5) % l2Sets;
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
    if (l2Sets == 0)
        return;
    const u32 set = (pa >> 5) % l2Sets;
    const u32 tag = pa >> 5;
    for (u32 w = 0; w < 2; ++w) {
        L2Line& e = l2[size_t(set) * 2 + w];
        if (e.v && e.tag == tag)
            e.v = false;
    }
}

void Cpu::l2WipeAll()
{
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
    for (auto& e : l2) {
        if (e.v && e.d && writeback)
            busWriteLine(*this, e.tag << 5, e.b);
        e.v = false;
        e.d = false;
    }
}

u64 Cpu::memRead(u32 pa, u32 len, u32 wimg)
{
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
    DLine* e = lineFind(*this, pa);
    if (!e) {
        DLine& n = lineVictim(*this, pa);
        lineFill(*this, n, pa);
        n.age = ++l1dClock;
        e = &n;
    }
    u64 v = 0;
    for (u32 i = 0; i < len; ++i)
        v = (v << 8) | e->b[(pa & 31u) + i];
    return v;
}

void Cpu::memWrite(u32 pa, u32 len, u64 v, u32 wimg)
{
    if (wpEnd && pa <= wpEnd && pa + len > wpPa &&
        (!wpFrom || !wpStamp || *wpStamp >= wpFrom)) {
        if (wpLog.size() < wpMax)
            wpLog.push_back(
                {st.pc - 4, pa, static_cast<u32>(v), len, st.lr, st.tb});
        if (wpForceSet && len == 4 && pa == wpPa) v = wpForce; // diagnostic
    }
    if (!dceOn() || (wimg & kWimgI)) {
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
    DLine* e = lineFind(*this, pa);
    if (wimg & kWimgW) { // write-through: no allocation on a store miss
        if (e)
            for (u32 i = 0; i < len; ++i)
                e->b[(pa & 31u) + i] =
                    static_cast<u8>(v >> (8 * (len - 1 - i)));
        switch (len) {
        case 1: bus->write8(pa, static_cast<u8>(v)); return;
        case 2: bus->write16(pa, static_cast<u16>(v)); return;
        case 4: bus->write32(pa, static_cast<u32>(v)); return;
        default: bus->write64(pa, v); return;
        }
    }
    if (!e) { // write-allocate
        DLine& n = lineVictim(*this, pa);
        lineFill(*this, n, pa);
        n.age = ++l1dClock;
        e = &n;
    }
    for (u32 i = 0; i < len; ++i)
        e->b[(pa & 31u) + i] = static_cast<u8>(v >> (8 * (len - 1 - i)));
    e->d = true;
}

bool Cpu::l1dPeek32(u32 pa, u32& w)
{
    if (!dceOn())
        return false;
    const u32 set = setOf(pa), tag = tagOf(pa);
    for (u32 k = 0; k < 8; ++k) {
        const DLine& e = l1d[set][k];
        if (e.v && e.tag == tag) {
            const u32 o = pa & 31u & ~3u;
            w = (u32(e.b[o]) << 24) | (u32(e.b[o + 1]) << 16) |
                (u32(e.b[o + 2]) << 8) | u32(e.b[o + 3]);
            return true;
        }
    }
    return false;
}

void Cpu::dcbzLine(u32 pa)
{
    DLine* e = lineFind(*this, pa);
    if (!e) {
        DLine& n = lineVictim(*this, pa);
        n.tag = tagOf(pa);
        n.v = true;
        n.age = ++l1dClock;
        e = &n;
    }
    for (u8& x : e->b)
        x = 0;
    e->d = true; // zeros exist only in the cache until written back
}

void Cpu::dcbClean(u32 pa, bool invalidate)
{
    DLine* e = lineFind(*this, pa);
    if (!e)
        return;
    if (e->d)
        linePush(*this, *e, setOf(pa));
    if (invalidate)
        e->v = false;
}

void Cpu::dcbKill(u32 pa)
{
    DLine* e = lineFind(*this, pa);
    if (e)
        e->v = false; // discarded, never written back
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
    if (!len)
        return;
    const u32 first = pa & ~31u;
    const u32 last = (pa + len - 1u) & ~31u;
    for (u32 a = first;; a += 32u) {
        if (dceOn()) {
            const u32 set = setOf(a), tag = tagOf(a);
            for (u32 w = 0; w < 8; ++w) {
                DLine& e = l1d[set][w];
                if (!e.v || e.tag != tag)
                    continue;
                if (e.d) {
                    busWriteLine(*this, a, e.b);
                    e.d = false;
                    // The L1 held the newest copy; any L2 copy is stale.
                    l2Invalidate(a);
                }
                if (invalidate)
                    e.v = false;
                break;
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
void Cpu::l1dFlushAll(bool writeback)
{
    for (u32 s = 0; s < 128; ++s)
        for (u32 w = 0; w < 8; ++w) {
            DLine& e = l1d[s][w];
            if (e.v && e.d && writeback)
                busWriteLine(*this, (e.tag << 12) | (s << 5), e.b);
            e.v = false;
            e.d = false;
        }
    l2FlushAll(writeback);
}

} // namespace opm
