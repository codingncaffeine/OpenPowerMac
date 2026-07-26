// Page-protection matrix sweep: PP(0-3) x Ks x Kp x PR x read/write = 64
// combinations, driven through the real translate() path with a PTE built
// per combination. The expected-allow table is written out longhand from
// PEM Table 7-15 (not shared with mmu.cpp's logic) so the two must agree
// independently. Also pins the R/C writeback policy: R set on any PTE
// match, C only on a permitted store.

#include "doctest.h"
#include "opm/cpu.hpp"

#include <map>

using namespace opm;

namespace {

class MapBus final : public Bus {
public:
    std::map<u32, u8> m;

    u8 read8(u32 pa) override
    {
        auto it = m.find(pa);
        return it == m.end() ? 0 : it->second;
    }
    u16 read16(u32 pa) override { return static_cast<u16>((read8(pa) << 8) | read8(pa + 1)); }
    u32 read32(u32 pa) override
    {
        return (u32(read8(pa)) << 24) | (u32(read8(pa + 1)) << 16) |
               (u32(read8(pa + 2)) << 8) | u32(read8(pa + 3));
    }
    u64 read64(u32 pa) override { return (u64(read32(pa)) << 32) | read32(pa + 4); }
    void write8(u32 pa, u8 v) override { m[pa] = v; }
    void write16(u32 pa, u16 v) override
    {
        write8(pa, static_cast<u8>(v >> 8));
        write8(pa + 1, static_cast<u8>(v));
    }
    void write32(u32 pa, u32 v) override
    {
        write8(pa, static_cast<u8>(v >> 24));
        write8(pa + 1, static_cast<u8>(v >> 16));
        write8(pa + 2, static_cast<u8>(v >> 8));
        write8(pa + 3, static_cast<u8>(v));
    }
    void write64(u32 pa, u64 v) override
    {
        write32(pa, static_cast<u32>(v >> 32));
        write32(pa + 4, static_cast<u32>(v));
    }
};

// PEM Table 7-15, transcribed row by row.
bool expectAllowed(u32 pp, bool key, bool write)
{
    if (!key) {
        if (pp == 0 || pp == 1 || pp == 2)
            return true;       // read/write
        return !write;         // pp 3: read-only
    }
    if (pp == 0)
        return false;          // no access
    if (pp == 1)
        return !write;         // read-only
    if (pp == 2)
        return true;           // read/write
    return !write;             // pp 3: read-only
}

} // namespace

TEST_CASE("mmu page-protection matrix (PEM Table 7-15), 64 combinations")
{
    // vsid 5, EA 0x3000 -> pageIndex 3, hash 6, PTEG 0x90180 with SDR1
    // htaborg 0x0009, htabmask 0.
    constexpr u32 kEa = 0x00003000u;
    constexpr u32 kPteAddr = 0x00090180u;
    constexpr u32 kRpn = 0x00005000u;

    for (u32 pp = 0; pp < 4; ++pp)
        for (u32 ks = 0; ks < 2; ++ks)
            for (u32 kp = 0; kp < 2; ++kp)
                for (u32 pr = 0; pr < 2; ++pr)
                    for (u32 wr = 0; wr < 2; ++wr) {
                        MapBus bus;
                        Cpu c;
                        c.attach(bus);
                        c.reset();
                        c.st.msr = msr::DR | (pr ? msr::PR : 0u);
                        c.st.pc = 0x1004; // as if mid-step (handlers see NIA)
                        c.st.sdr1 = 0x00090000u;
                        c.st.sr[0] = (ks << 30) | (kp << 29) | 5u;
                        bus.write32(kPteAddr, 0x80000000u | (5u << 7));
                        bus.write32(kPteAddr + 4, kRpn | pp);

                        const bool key = pr ? kp != 0 : ks != 0;
                        const bool want = expectAllowed(pp, key, wr != 0);

                        u32 pa = 0;
                        const bool got = c.translate(kEa, wr != 0, false, pa);

                        CAPTURE(pp);
                        CAPTURE(ks);
                        CAPTURE(kp);
                        CAPTURE(pr);
                        CAPTURE(wr);
                        CHECK(got == want);

                        const u32 w1 = bus.read32(kPteAddr + 4);
                        CHECK((w1 & 0x100u) != 0);                    // R always set on match
                        CHECK(((w1 & 0x80u) != 0) == (want && wr));   // C only on permitted store
                        if (got) {
                            CHECK(pa == (kRpn | (kEa & 0xFFFu)));
                        } else {
                            CHECK(c.st.dar == kEa);
                            CHECK(c.st.dsisr ==
                                  (0x08000000u | (wr ? 0x02000000u : 0u)));
                        }
                    }
}

TEST_CASE("mmu BAT protection: PP=00 blocks, x1 read-only, 10 read/write")
{
    struct Row {
        u32 pp;
        bool write;
        bool allowed;
    };
    const Row rows[] = {
        {0, false, false}, {0, true, false},
        {1, false, true},  {1, true, false},
        {2, false, true},  {2, true, true},
        {3, false, true},  {3, true, false},
    };
    for (const Row& r : rows) {
        MapBus bus;
        Cpu c;
        c.attach(bus);
        c.reset();
        c.st.msr = msr::DR;
        c.st.pc = 0x1004;
        c.st.dbatu[0] = 0x10000002u;          // BEPI 0x10000000, 128KB, Vs
        c.st.dbatl[0] = 0x00060000u | r.pp;
        u32 pa = 0;
        const bool got = c.translate(0x10000010u, r.write, false, pa);
        CAPTURE(r.pp);
        CAPTURE(r.write);
        CHECK(got == r.allowed);
        if (got)
            CHECK(pa == 0x00060010u);
    }
}
