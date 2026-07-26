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

TEST_CASE("mmu TLB: staleness contract, tlbie class, C-update, LRU")
{
    // Same geometry as the matrix test: vsid 5, SDR1 htaborg 0x0009.
    auto setup = [](Cpu& c, MapBus& bus) {
        c.attach(bus);
        c.reset();
        c.st.msr = msr::DR;
        c.st.pc = 0x1004;
        c.st.sdr1 = 0x00090000u;
        for (u32 s = 0; s < 16; ++s)
            c.st.sr[s] = 5u;
    };
    // PTE for (vsid 5, pageIndex pi) at its primary PTEG, slot 0.
    auto pteAddr = [](u32 pi) {
        return 0x00090000u | (((5u ^ pi) & 0x3FFu) << 6);
    };
    auto insert = [&](MapBus& bus, u32 pi, u32 rpn, u32 pp) {
        bus.write32(pteAddr(pi), 0x80000000u | (5u << 7) | (pi >> 10));
        bus.write32(pteAddr(pi) + 4, rpn | pp);
    };

    SUBCASE("stale entry survives a PTE wipe until tlbie")
    {
        MapBus bus;
        Cpu c;
        setup(c, bus);
        insert(bus, 3, 0x00005000u, 2);
        u32 pa = 0;
        REQUIRE(c.translate(0x3000u, false, false, pa));
        CHECK(pa == 0x5000u);

        bus.write32(pteAddr(3), 0); // regime teardown wipes the PTE
        pa = 0;
        CHECK(c.translate(0x3000u, false, false, pa)); // warm TLB answers
        CHECK(pa == 0x5000u);

        c.tlbInvalidateClass(0x3000u);
        CHECK_FALSE(c.translate(0x3000u, false, false, pa)); // now a miss
        CHECK(c.st.dsisr == 0x40000000u);
    }

    SUBCASE("tlbie hits only its congruence class")
    {
        MapBus bus;
        Cpu c;
        setup(c, bus);
        insert(bus, 3, 0x00005000u, 2);   // set 3
        insert(bus, 4, 0x00006000u, 2);   // set 4
        u32 pa = 0;
        REQUIRE(c.translate(0x3000u, false, false, pa));
        REQUIRE(c.translate(0x4000u, false, false, pa));
        bus.write32(pteAddr(3), 0);
        bus.write32(pteAddr(4), 0);
        c.tlbInvalidateClass(0x4000u); // only set 4
        CHECK(c.translate(0x3000u, false, false, pa)); // stale, alive
        CHECK_FALSE(c.translate(0x4000u, false, false, pa));
    }

    SUBCASE("first store through a clean entry re-walks to set C")
    {
        MapBus bus;
        Cpu c;
        setup(c, bus);
        insert(bus, 3, 0x00005000u, 2);
        u32 pa = 0;
        REQUIRE(c.translate(0x3000u, false, false, pa)); // load: C stays 0
        CHECK((bus.read32(pteAddr(3) + 4) & 0x80u) == 0);
        REQUIRE(c.translate(0x3000u, true, false, pa)); // store: C-update
        CHECK((bus.read32(pteAddr(3) + 4) & 0x80u) != 0);

        // Wiped PTE + clean entry: the C-update walk must fault the store
        // while loads keep working off the stale entry.
        insert(bus, 4, 0x00006000u, 2);
        REQUIRE(c.translate(0x4000u, false, false, pa));
        bus.write32(pteAddr(4), 0);
        CHECK(c.translate(0x4000u, false, false, pa));
        CHECK_FALSE(c.translate(0x4000u, true, false, pa));
        CHECK(c.st.dsisr == (0x40000000u | 0x02000000u));
    }

    SUBCASE("two ways per set, LRU eviction")
    {
        MapBus bus;
        Cpu c;
        setup(c, bus);
        // Three pages in congruence class 3: pageIndex 3, 67, 131.
        insert(bus, 3, 0x00005000u, 2);
        insert(bus, 67, 0x00006000u, 2);
        insert(bus, 131, 0x00007000u, 2);
        u32 pa = 0;
        REQUIRE(c.translate(0x00003000u, false, false, pa)); // way A
        REQUIRE(c.translate(0x00043000u, false, false, pa)); // way B
        REQUIRE(c.translate(0x00083000u, false, false, pa)); // evicts A
        bus.write32(pteAddr(3), 0);
        bus.write32(pteAddr(67), 0);
        bus.write32(pteAddr(131), 0);
        CHECK_FALSE(c.translate(0x00003000u, false, false, pa)); // evicted
        c.st.dsisr = 0;
        CHECK(c.translate(0x00043000u, false, false, pa)); // still cached
        CHECK(c.translate(0x00083000u, false, false, pa)); // still cached
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
