// MMU: real mode, block address translation (4 IBAT + 4 DBAT pairs, blocks
// to 256 MB), and the segment/hashed-page-table path with hardware table
// search and R/C writeback. Per UM ch.5 / PEM ch.7.
//
// No TLB contents are modeled: every access walks BATs/PTEGs, which is
// strictly more coherent than real TLBs (never stale), so tlbie/tlbsync
// remain architectural no-ops. A software TLB cache is a P8 performance
// item, not a correctness one.
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
    const bool on = (st.msr & (fetch ? msr::IR : msr::DR)) != 0;
    if (!on) {
        pa = ea;
        if (wimg)
            *wimg = kWimgReal;
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

    // 3) Hashed page table search (primary, then secondary).
    const u32 pageIndex = (ea >> 12) & 0xFFFFu;
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
            const u32 w0 = bus->read32(pteAddr);
            if (!(w0 & 0x80000000u))
                continue;
            if (((w0 >> 7) & 0xFFFFFFu) != vsid)
                continue;
            if (((w0 >> 6) & 1u) != static_cast<u32>(pass))
                continue;
            if ((w0 & 0x3Fu) != api)
                continue;

            const u32 w1 = bus->read32(pteAddr + 4);
            if (fetch && (w1 & (kWimgG << 3))) { // guarded-memory fetch: no R
                raiseExc(Exc::Isi, cia, 0x10000000u);
                return false;
            }
            u32 nw1 = w1 | 0x00000100u; // R: set on PTE match (TLB reload)
            const u32 pp = w1 & 3u;
            if (!ppAllows(pp, key, write)) {
                if (nw1 != w1)
                    bus->write32(pteAddr + 4, nw1);
                if (fetch)
                    raiseExc(Exc::Isi, cia, 0x08000000u);
                else
                    dsi(0x08000000u);
                return false;
            }
            if (write)
                nw1 |= 0x00000080u; // C: only for a permitted store
            if (nw1 != w1)
                bus->write32(pteAddr + 4, nw1);
            pa = (w1 & 0xFFFFF000u) | (ea & 0xFFFu);
            if (wimg)
                *wimg = (w1 >> 3) & 0xFu;
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

// ---- fault-aware virtual accessors ----------------------------------------
// Accesses that cross a 4 KB page translate each page separately.

bool Cpu::readV(u32 ea, u32 len, u64& out)
{
    out = 0;
    u32 pa;
    if (((ea ^ (ea + len - 1)) & ~0xFFFu) == 0) {
        if (!translate(ea, false, false, pa))
            return false;
        switch (len) {
        case 1: out = bus->read8(pa); return true;
        case 2: out = bus->read16(pa); return true;
        case 4: out = bus->read32(pa); return true;
        default: out = bus->read64(pa); return true;
        }
    }
    for (u32 i = 0; i < len; ++i) {
        if (!translate(ea + i, false, false, pa))
            return false;
        out = (out << 8) | bus->read8(pa);
    }
    return true;
}

bool Cpu::writeV(u32 ea, u32 len, u64 v)
{
    u32 pa;
    if (((ea ^ (ea + len - 1)) & ~0xFFFu) == 0) {
        if (!translate(ea, true, false, pa))
            return false;
        switch (len) {
        case 1: bus->write8(pa, static_cast<u8>(v)); return true;
        case 2: bus->write16(pa, static_cast<u16>(v)); return true;
        case 4: bus->write32(pa, static_cast<u32>(v)); return true;
        default: bus->write64(pa, v); return true;
        }
    }
    for (u32 i = 0; i < len; ++i) {
        if (!translate(ea + i, true, false, pa))
            return false;
        bus->write8(pa, static_cast<u8>(v >> (8 * (len - 1 - i))));
    }
    return true;
}

bool Cpu::fetch32(u32 ea, u32& insn)
{
    u32 pa;
    if (!translate(ea, false, true, pa))
        return false;
    insn = bus->read32(pa);
    return true;
}

} // namespace opm
