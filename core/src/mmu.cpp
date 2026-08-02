// MMU: real mode, block address translation (4 IBAT + 4 DBAT pairs, blocks
// to 256 MB), and the segment/hashed-page-table path through modeled TLBs
// (128-entry, 2-way, 64-set I and D sides, LRU per set) with hardware
// table search and R/C writeback on reload. Per UM ch.5 / PEM ch.7.
//
// The TLBs are load-bearing, not an optimization. The original draft
// walked the table on every access ("strictly more coherent than real
// TLBs — never stale") — FALSIFIED 2026-07-26 by the Gossamer boot ROM:
// its regime teardown byte-wipes the whole hash table while the wiping
// thread keeps executing through translated code, I/O, and pager mappings,
// relying on warm TLB entries until it issues tlbie. Software may depend
// on translation staleness; the TLB models it. tlbie invalidates the
// addressed congruence class in both TLBs; tlbsync stays a no-op (1 CPU).
//
// Fault status (verified against UM Tables 5-3/5-4 and PEM Tables 6-12/6-13):
//   DSI DSISR: miss=bit1 (0x40000000), protection=bit4 (0x08000000),
//   direct-store=bit5 (0x04000000), write=bit6 (0x02000000).
//   ISI SRR1: miss=bit1 (0x40000000); direct-store fetch, no-execute
//   segment, and guarded-memory fetch all share bit3 (0x10000000);
//   protection=bit4 (0x08000000).
// R/C policy (PEM Table 7-17; UM 5.2 "the MMU prevents the changed bit ...
// from being updated erroneously"): R is written back as soon as a matching
// PTE is found, even if protection then faults — the 7400's TLB reload does
// exactly this. C is written back only for a permitted store. A no-execute
// or guarded fetch fault sets neither (Table 7-17 row 1).

#include "opm/cpu.hpp"
#include "opm/bits.hpp"
#include "opm/prof.hpp"

namespace opm {

namespace {

// WIMG nibble as held in BATL / PTE word 1 bits 25-28: W=8, I=4, M=2, G=1.
inline constexpr u32 kWimgW = 8u;
inline constexpr u32 kWimgI = 4u;
inline constexpr u32 kWimgG = 1u;
// Real mode (translation off) is treated as WIMG = 0b0011 (PEM 7.2).
inline constexpr u32 kWimgReal = 3u;

struct BatHit {
    bool hit = false;
    u32 pa = 0;
    u32 pp = 0;
    u32 wimg = 0;
};

BatHit batLookup(const u32* batu, const u32* batl, u32 ea, bool user)
{
    for (int i = 0; i < 4; ++i) {
        const u32 u = batu[i];
        const bool valid = user ? (u & 1u) : (u & 2u); // Vp / Vs
        if (!valid)
            continue;
        const u32 blMask = (u >> 2) & 0x7FFu;      // BL: block length mask
        const u32 eaHi = ea >> 17;                 // EA[0-14]
        const u32 bepi = u >> 17;
        if (((eaHi ^ bepi) & (0x7FFFu & ~blMask)) != 0)
            continue;
        const u32 brpn = batl[i] >> 17;
        const u32 paHi = (brpn & ~blMask) | (eaHi & blMask);
        BatHit h;
        h.hit = true;
        h.pa = (paHi << 17) | (ea & 0x1FFFFu);
        h.pp = batl[i] & 3u;
        h.wimg = (batl[i] >> 3) & 0xFu;
        return h;
    }
    return {};
}

// PP+key protection per PEM Table 7-15. Returns true if the access is legal.
bool ppAllows(u32 pp, bool key, bool write)
{
    if (!key)
        return pp == 3u ? !write : true; // key=0: PP 0/1/2 RW, 3 RO
    switch (pp) {                        // key=1
    case 0: return false;
    case 1: return !write;
    case 2: return true;
    default: return !write;
    }
}

} // namespace

bool Cpu::translate(u32 ea, bool write, bool fetch, u32& pa, u32* wimg)
{
    OPM_MARK(Xlate);
    const bool on = (st.msr & (fetch ? msr::IR : msr::DR)) != 0;
    if (!on) {
        pa = ea;
        if (wimg)
            *wimg = (realModeInhibitBase && ea >= realModeInhibitBase)
                        ? (kWimgReal | kWimgI)
                        : kWimgReal;
        return true;
    }
    const bool user = userMode();
    const u32 cia = fetch ? ea : st.pc - 4;

    auto dsi = [&](u32 bits) {
        st.dar = ea;
        st.dsisr = bits | (write ? 0x02000000u : 0);
        raiseExc(Exc::Dsi, cia, 0);
    };

    // 1) BATs (a matching BAT takes priority over page translation).
    const BatHit b = fetch ? batLookup(st.ibatu, st.ibatl, ea, user)
                           : batLookup(st.dbatu, st.dbatl, ea, user);
    if (b.hit) {
        if (fetch && (b.wimg & kWimgG)) { // guarded-memory fetch
            raiseExc(Exc::Isi, cia, 0x10000000u);
            return false;
        }
        // BAT PP: 00 no access, x1 read-only, 10 read/write.
        if (b.pp == 0u || (write && (b.pp & 1u))) {
            if (fetch)
                raiseExc(Exc::Isi, cia, 0x08000000u);
            else
                dsi(0x08000000u);
            return false;
        }
        pa = b.pa;
        if (wimg)
            *wimg = b.wimg;
        return true;
    }

    // 2) Segment.
    const u32 sr = st.sr[ea >> 28];
    if (sr & 0x80000000u) { // T: direct-store — unsupported on the 7400
        if (fetch)
            raiseExc(Exc::Isi, cia, 0x10000000u);
        else
            dsi(0x04000000u);
        return false;
    }
    if (fetch && (sr & 0x10000000u)) { // N: no-execute
        raiseExc(Exc::Isi, cia, 0x10000000u);
        return false;
    }
    const bool key = user ? (sr & 0x20000000u) != 0   // Kp
                          : (sr & 0x40000000u) != 0;  // Ks
    const u32 vsid = sr & 0x00FFFFFFu;

    const u32 pageIndex = (ea >> 12) & 0xFFFFu;

    // 3) TLB lookup (EA[14-19] selects the set). A hit answers from the
    // cached PTE image — including after the memory PTE was modified or
    // wiped; that staleness is the architectural contract until tlbie.
    TlbEntry(&tlbSet)[64][2] = fetch ? itlb : dtlb;
    u8* tlbLru = fetch ? itlbLru : dtlbLru;
    const u32 set = pageIndex & 63u;
    if (!mmuProbe) {
        for (u32 way = 0; way < 2; ++way) {
            TlbEntry& e = tlbSet[set][way];
            if (!e.v || e.vsid != vsid || e.pi != pageIndex)
                continue;
            if (fetch && (e.wimg & kWimgG)) {
                raiseExc(Exc::Isi, cia, 0x10000000u);
                return false;
            }
            if (!ppAllows(e.pp, key, write)) {
                if (fetch)
                    raiseExc(Exc::Isi, cia, 0x08000000u);
                else
                    dsi(0x08000000u);
                return false;
            }
            if (write && !e.c) {
                // First store through the entry: hardware runs a table
                // search to set C. Modeled as invalidate + reload so the
                // walk below re-reads the PTE (and faults if it is gone).
                e.v = false;
                break;
            }
            pa = e.rpn | (ea & 0xFFFu);
            if (wimg)
                *wimg = e.wimg;
            tlbLru[set] = static_cast<u8>(way ^ 1u); // other way is LRU
            return true;
        }
    }

    // 4) Hashed page table search (primary, then secondary).
    const u32 api = pageIndex >> 10;
    const u32 htaborg = st.sdr1 & 0xFFFF0000u;
    const u32 htabmask = st.sdr1 & 0x1FFu;

    for (int pass = 0; pass < 2; ++pass) {
        const u32 hash = pass == 0
                             ? ((vsid & 0x7FFFFu) ^ pageIndex)
                             : (~((vsid & 0x7FFFFu) ^ pageIndex) & 0x7FFFFu);
        const u32 ptegAddr = htaborg |
                             ((((hash >> 10) & 0x1FFu) & htabmask) << 16) |
                             ((hash & 0x3FFu) << 6);
        for (u32 slot = 0; slot < 8; ++slot) {
            const u32 pteAddr = ptegAddr + slot * 8u;
            // Hardware table searches are cacheable (UM 5.x: implied WIM =
            // 0b001) and go through the data cache — a table living in
            // dirty cache lines is fully architectural, and the pre-DRAM
            // boot depends on it. Probes stay off the cache.
            const u32 w0 =
                mmuProbe ? bus->read32(pteAddr)
                         : static_cast<u32>(memRead(pteAddr, 4, 2u));
            if (!(w0 & 0x80000000u))
                continue;
            if (((w0 >> 7) & 0xFFFFFFu) != vsid)
                continue;
            if (((w0 >> 6) & 1u) != static_cast<u32>(pass))
                continue;
            if ((w0 & 0x3Fu) != api)
                continue;

            const u32 w1 =
                mmuProbe ? bus->read32(pteAddr + 4)
                         : static_cast<u32>(memRead(pteAddr + 4, 4, 2u));
            if (fetch && (w1 & (kWimgG << 3))) { // guarded-memory fetch: no R
                raiseExc(Exc::Isi, cia, 0x10000000u);
                return false;
            }
            u32 nw1 = w1 | 0x00000100u; // R: set on PTE match (TLB reload)
            const u32 pp = w1 & 3u;
            if (!ppAllows(pp, key, write)) {
                if (nw1 != w1 && !mmuProbe)
                    memWrite(pteAddr + 4, 4, nw1, 2u);
                if (fetch)
                    raiseExc(Exc::Isi, cia, 0x08000000u);
                else
                    dsi(0x08000000u);
                return false;
            }
            if (write)
                nw1 |= 0x00000080u; // C: only for a permitted store
            if (nw1 != w1 && !mmuProbe)
                memWrite(pteAddr + 4, 4, nw1, 2u);
            pa = (w1 & 0xFFFFF000u) | (ea & 0xFFFu);
            if (wimg)
                *wimg = (w1 >> 3) & 0xFu;
            if (!mmuProbe) { // TLB reload into the set's LRU way
                const u32 way = tlbLru[set];
                tlbSet[set][way] = {true, (nw1 & 0x80u) != 0, vsid,
                                    pageIndex, w1 & 0xFFFFF000u,
                                    (w1 >> 3) & 0xFu, pp};
                tlbLru[set] = static_cast<u8>(way ^ 1u);
            }
            return true;
        }
    }

    // Miss.
    if (fetch)
        raiseExc(Exc::Isi, cia, 0x40000000u);
    else
        dsi(0x40000000u);
    return false;
}

// tlbie: invalidate the congruence class EA[14-19] selects — both ways,
// both TLBs (UM 5.4.4: one class of each side per tlbie).
void Cpu::tlbInvalidateClass(u32 ea)
{
    ++mmuGen; // retires the one-page instruction translation cache
    batchBreak = true; // …and the block the line executor is running from
    const u32 set = (ea >> 12) & 63u;
    for (u32 way = 0; way < 2; ++way) {
        itlb[set][way].v = false;
        dtlb[set][way].v = false;
    }
}

void Cpu::tlbFlushAll()
{
    ++mmuGen;
    batchBreak = true;
    for (u32 s = 0; s < 64; ++s) {
        for (u32 w = 0; w < 2; ++w) {
            itlb[s][w] = TlbEntry{};
            dtlb[s][w] = TlbEntry{};
        }
        itlbLru[s] = dtlbLru[s] = 0;
    }
}

// ---- fault-aware virtual accessors ----------------------------------------
// Accesses that cross a 4 KB page translate each page separately.
//
// Little-endian mode (MSR[LE]): the EA's low bits are modified per access
// size (PEM Table 3-2: byte ^0b111, half ^0b110, word ^0b100), doublewords
// split into two munged word accesses (word swap), and any misaligned
// access raises an alignment exception (RECEIPT: PEM allows but does not
// require this; the classic implementations fault, and munging is undefined
// for misaligned EAs). DABR watchpoints compare the pre-munge EA at
// double-word granularity under the BAMR mask, honoring DABR[29]=BT
// against MSR[DR]; a hit is a DSI with DSISR[9].

namespace {

// UM: DABR_CMP[0-28] = DABR[0-28] AND BAMR[0-28], compared against the EA
// masked the same way — BAMR zero bits are don't-cares. DR=bit31, DW=bit30,
// BT=bit29 (matches MSR[DR]).
inline bool dabrHit(const CpuState& s, u32 ea, bool write)
{
    const u32 en = write ? 2u : 1u; // DW / DR
    if (!(s.dabr & en))
        return false;
    if ((((s.dabr >> 2) & 1u) != 0) != ((s.msr & msr::DR) != 0)) // BT
        return false;
    return ((((ea ^ s.dabr) >> 3) & (s.bamr >> 3)) & 0x1FFFFFFFu) == 0;
}

} // namespace

bool Cpu::leAlignCheck(u32 ea, u32 len)
{
    if (!(st.msr & msr::LE) || (ea & (len - 1)) == 0)
        return true;
    st.dar = ea;
    st.dsisr = alignDsisr(curInsn);
    raiseExc(Exc::Alignment, st.pc - 4, 0);
    return false;
}

// The data path's translation front: one page per direction, validated the
// way the fetch cache in fetchDecoded is (MSR data bits + segment register
// compared per access, everything else carried by mmuGen). The direction
// split is load-bearing — see the field comment in cpu.hpp: only a store's
// own walk sets the page table's C bit, so a store must never ride a
// translation a load filled. The marker matters for the same reason the
// fetch path's does: on a hit translate() never runs, and unmarked compare
// work would bill itself to the caller's bucket.
bool Cpu::xlateData(u32 ea, bool write, u32& pa, u32& wimg)
{
    OPM_MARK(Xlate);
    if (dxlCacheOff)
        return translate(ea, write, false, pa, &wimg);
    const u32 page = ea >> 12;
    const u32 msrKey = st.msr & (msr::DR | msr::PR | msr::LE);
    const u32 srKey = st.sr[ea >> 28];
    const int d = write ? 1 : 0;
    if (dxlGen[d] == mmuGen && dxlPage[d] == page && dxlMsr[d] == msrKey &&
        dxlSr[d] == srKey) {
        pa = dxlPa[d] | (ea & 0xFFFu);
        wimg = dxlWimg[d];
        return true;
    }
    if (!translate(ea, write, false, pa, &wimg))
        return false;
    dxlGen[d] = mmuGen;
    dxlPage[d] = page;
    dxlMsr[d] = msrKey;
    dxlSr[d] = srKey;
    dxlPa[d] = pa & ~0xFFFu;
    dxlWimg[d] = wimg;
    return true;
}

bool Cpu::readV(u32 ea, u32 len, u64& out)
{
    out = 0;
    u32 pa;
    if (!leAlignCheck(ea, len))
        return false;
    if (dabrHit(st, ea, false)) {
        st.dar = ea;
        st.dsisr = 0x00400000u;
        raiseExc(Exc::Dsi, st.pc - 4, 0);
        return false;
    }
    u32 wimg = 0;
    if (st.msr & msr::LE) {
        if (len == 8) { // two munged word accesses; words swap
            u64 hi, lo;
            if (!translate(ea ^ 4u, false, false, pa, &wimg))
                return false;
            hi = memRead(pa, 4, wimg);
            if (!translate((ea + 4u) ^ 4u, false, false, pa, &wimg))
                return false;
            lo = memRead(pa, 4, wimg);
            out = (hi << 32) | lo;
            return true;
        }
        ea ^= len == 1 ? 7u : (len == 2 ? 6u : 4u);
    }
    if (((ea ^ (ea + len - 1)) & ~0xFFFu) == 0) {
        if (!xlateData(ea, false, pa, wimg))
            return false;
        out = memRead(pa, len, wimg);
        return true;
    }
    for (u32 i = 0; i < len; ++i) {
        if (!translate(ea + i, false, false, pa, &wimg))
            return false;
        out = (out << 8) | memRead(pa, 1, wimg);
    }
    return true;
}

bool Cpu::writeV(u32 ea, u32 len, u64 v)
{
    u32 pa;
    if (!leAlignCheck(ea, len))
        return false;
    if (dabrHit(st, ea, true)) {
        st.dar = ea;
        st.dsisr = 0x00400000u | 0x02000000u;
        raiseExc(Exc::Dsi, st.pc - 4, 0);
        return false;
    }
    u32 wimg = 0;
    if (st.msr & msr::LE) {
        if (len == 8) {
            if (!translate(ea ^ 4u, true, false, pa, &wimg))
                return false;
            memWrite(pa, 4, static_cast<u32>(v >> 32), wimg);
            if (!translate((ea + 4u) ^ 4u, true, false, pa, &wimg))
                return false;
            memWrite(pa, 4, static_cast<u32>(v), wimg);
            return true;
        }
        ea ^= len == 1 ? 7u : (len == 2 ? 6u : 4u);
    }
    if (((ea ^ (ea + len - 1)) & ~0xFFFu) == 0) {
        if (!xlateData(ea, true, pa, wimg))
            return false;
        memWrite(pa, len, v, wimg);
        return true;
    }
    for (u32 i = 0; i < len; ++i) {
        if (!translate(ea + i, true, false, pa, &wimg))
            return false;
        memWrite(pa, 1, (v >> (8 * (len - 1 - i))) & 0xFFu, wimg);
    }
    return true;
}

bool Cpu::fetch32(u32 ea, u32& insn)
{
    u32 row;
    return fetchDecoded(ea, insn, row);
}

bool Cpu::fetchDecoded(u32 ea, u32& insn, u32& row)
{
    u32 pa;
    if (st.msr & msr::LE)
        ea ^= 4u; // instruction fetches munge like word data (PEM 3.1.4.4)
    // DIAGNOSTIC (--no-icache): the fetch path exactly as it was before any of
    // the caches below existed — translate every fetch, peek one word at the
    // L1 then the L2 then the bus, decode it. Three caches now stand between
    // the guest and its own instructions, and each is a claim that something
    // cannot have changed; a claim like that is settled by a control run, not
    // by reading the code. Anything that behaves differently with this set is
    // the caches' fault and nothing else's.
    if (fetchCacheOff) {
        if (!translate(ea, false, true, pa))
            return false;
        if (!l1dPeek32(pa, insn) && !l2Peek32(pa, insn))
            insn = bus->read32(pa);
        const InsnDesc* d = decode(insn);
        row = d ? static_cast<u32>(d - kIsa) : kNoRow;
        return true;
    }
    // Translation, from the one-page cache when its inputs are unchanged.
    // MSR and the segment register are compared directly; the BATs, SDR1 and
    // the page table are covered by mmuGen. See Cpu::xlPage.
    // ⚠ The marker covers the CACHE CHECK as well as the walk. Cpu::translate
    // marks itself, but on a hit it is never called, so the comparison work
    // was being billed to the fetch bucket and made "fetch" look like 42.7% of
    // the machine. Same failure as a clock advance inheriting its caller's
    // phase: a profiler that bills work to whoever it happened to run inside
    // sends the next reader to the wrong file.
    {
        OPM_MARK(Xlate);
        const u32 page = ea >> 12;
        const u32 msrKey = st.msr & (msr::IR | msr::PR | msr::LE);
        const u32 srKey = st.sr[ea >> 28];
        if (xlGen == mmuGen && xlPage == page && xlMsr == msrKey &&
            xlSr == srKey) {
            pa = xlPa | (ea & 0xFFFu);
        } else {
            if (!translate(ea, false, true, pa))
                return false;
            xlGen = mmuGen;
            xlPage = page;
            xlMsr = msrKey;
            xlSr = srKey;
            xlPa = pa & ~0xFFFu;
        }
    }
    // The block buffer, if it still holds this block. See Cpu::fetchBase for
    // the invalidation contract — every writer of memory or of a cache drops
    // it, so a hit here is the same word the long path below would produce.
    const u32 base = pa & ~31u;
    const u32 word = (pa >> 2) & 7u;
    FetchLine& fl = fetchLine[fetchSlot(pa)];
    if (fl.base == base) {
        OPM_COUNT(fetchHits);
        insn = fl.w[word];
        row = fl.row[word];
        return true;
    }
    // Fetch coherence: serve from a hitting D-cache or L2 line first.
    // Stronger than real hardware's split caches (which demand dcbst+icbi
    // sweeps), never weaker — deterministic superset, RECEIPT.
    u8 raw[32];
    bool filled = false;
    if (l1dReadLine(base, raw)) {
        OPM_COUNT(fetchFillsL1);
        filled = true;
    } else if (l2ReadLine(base, raw)) {
        OPM_COUNT(fetchFillsL2);
        filled = true;
    } else if (bus->memoryAt(base, 32)) {
        OPM_COUNT(fetchFillsMem);
        bus->readLine32(base, raw);
        filled = true;
    }
    if (filled) {
        fl.base = base;
        // Assemble and decode the whole block while it is here. Eighteen
        // instructions are executed per fill on average, so the seven words
        // that were not asked for are paid off many times over — and decode
        // has no side effects, so decoding a word that is never executed
        // costs nothing but this.
        for (u32 k = 0; k < 8; ++k) {
            const u32 v = (u32(raw[k * 4]) << 24) | (u32(raw[k * 4 + 1]) << 16) |
                          (u32(raw[k * 4 + 2]) << 8) | u32(raw[k * 4 + 3]);
            fl.w[k] = v;
            const InsnDesc* d = decode(v);
            fl.row[k] = d ? static_cast<u16>(d - kIsa) : kNoRow;
        }
        insn = fl.w[word];
        row = fl.row[word];
        return true;
    }
    fl.base = 1; // a partial fill must not be left looking valid
    // Not memory: a device window, or nothing at all. Read exactly the word
    // that was asked for and cache nothing — see Bus::memoryAt.
    OPM_COUNT(fetchUncached);
    insn = bus->read32(pa);
    const InsnDesc* d = decode(insn);
    row = d ? static_cast<u32>(d - kIsa) : kNoRow;
    return true;
}

} // namespace opm
