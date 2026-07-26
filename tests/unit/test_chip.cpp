// P6: reset-state conformance (UM Table 2-18) and the SPR access-rights
// matrix — user/supervisor × read/write across the implemented SPR file,
// plus the undefined-SPR privilege rule (PEM 6.4.7).

#include "doctest.h"
#include "opm/cpu.hpp"

#include <map>

using namespace opm;

namespace {

class NullBus final : public Bus {
public:
    std::map<u32, u8> m;
    u8 read8(u32 a) override { auto i = m.find(a); return i == m.end() ? 0 : i->second; }
    u16 read16(u32 a) override { return static_cast<u16>((read8(a) << 8) | read8(a + 1)); }
    u32 read32(u32 a) override
    {
        return (u32(read8(a)) << 24) | (u32(read8(a + 1)) << 16) |
               (u32(read8(a + 2)) << 8) | u32(read8(a + 3));
    }
    u64 read64(u32 a) override { return (u64(read32(a)) << 32) | read32(a + 4); }
    void write8(u32 a, u8 v) override { m[a] = v; }
    void write16(u32 a, u16 v) override { write8(a, u8(v >> 8)); write8(a + 1, u8(v)); }
    void write32(u32 a, u32 v) override
    {
        write8(a, u8(v >> 24)); write8(a + 1, u8(v >> 16));
        write8(a + 2, u8(v >> 8)); write8(a + 3, u8(v));
    }
    void write64(u32 a, u64 v) override { write32(a, u32(v >> 32)); write32(a + 4, u32(v)); }
};

u32 encMfspr(u32 rd, u32 spr)
{
    return (31u << 26) | (rd << 21) | ((spr & 31u) << 16) |
           (((spr >> 5) & 31u) << 11) | (339u << 1);
}
u32 encMtspr(u32 rs, u32 spr)
{
    return (31u << 26) | (rs << 21) | ((spr & 31u) << 16) |
           (((spr >> 5) & 31u) << 11) | (467u << 1);
}

// Runs one instruction from a fresh CPU; returns the vector taken (0 = none).
u32 runOne(u32 insn, bool user)
{
    NullBus bus;
    Cpu c;
    c.attach(bus);
    c.reset();
    c.st.pc = 0x1000;
    c.st.msr = user ? msr::PR : 0u;
    bus.write32(0x1000, insn);
    c.step();
    if (!c.raisedThisStep)
        return 0;
    return c.st.pc & 0xFFFFFu;
}

} // namespace

TEST_CASE("reset state matches UM Table 2-18")
{
    Cpu c;
    c.reset();
    CHECK(c.st.pc == 0xFFF00100u);
    CHECK(c.st.msr == 0x00000040u); // only IP
    CHECK(c.st.pvr == 0x000C0209u);
    CHECK(c.st.dec == 0xFFFFFFFFu);
    CHECK(c.st.msscr0 == 0x00400000u);
    CHECK(c.st.tb == 0u);
    CHECK(c.st.hid0 == 0u);
    CHECK(c.st.fpscr == 0u);
    CHECK(c.st.vscr == 0u);
    CHECK(c.st.sdr1 == 0u);
    CHECK(c.st.l2cr == 0u);
    for (int i = 0; i < 4; ++i) {
        CHECK(c.st.ibatu[i] == 0u);
        CHECK(c.st.dbatu[i] == 0u);
    }
    CHECK(!c.napping);
}

TEST_CASE("SPR access-rights matrix")
{
    struct Row {
        u32 spr;
        bool userRead;   // no exception on user mfspr
        bool userWrite;  // no exception on user mtspr
        bool writable;   // supervisor mtspr is a defined operation
    };
    const Row rows[] = {
        {1, true, true, true},      {8, true, true, true},
        {9, true, true, true},      {256, true, true, true},
        {268, true, false, false},  {269, true, false, false}, // TB: mfspr-only
        {935, true, false, true},   {936, true, false, true},
        {943, true, false, true},
        {18, false, false, true},   {19, false, false, true},
        {22, false, false, true},   {25, false, false, true},
        {26, false, false, true},   {27, false, false, true},
        {272, false, false, true},  {287, false, false, true},
        {528, false, false, true},  {951, false, false, true},
        {1008, false, false, true}, {1013, false, false, true},
        {1017, false, false, true}, {1023, false, false, true},
    };
    for (const Row& r : rows) {
        CAPTURE(r.spr);
        CHECK(runOne(encMfspr(3, r.spr), true) == (r.userRead ? 0u : 0x700u));
        CHECK(runOne(encMtspr(3, r.spr), true) == (r.userWrite ? 0u : 0x700u));
        CHECK(runOne(encMfspr(3, r.spr), false) == 0u);
        CHECK(runOne(encMtspr(3, r.spr), false) == (r.writable ? 0u : 0x700u));
    }
}

TEST_CASE("undefined SPR: illegal vs privileged by SPR bit 4")
{
    NullBus bus;
    // user, spr 3 (bit4 clear): illegal
    {
        Cpu c;
        c.attach(bus);
        c.reset();
        c.st.pc = 0x1000;
        c.st.msr = msr::PR;
        bus.write32(0x1000, encMfspr(3, 3));
        c.step();
        CHECK(c.st.pc == 0x700u);
        CHECK((c.st.srr1 & 0x00080000u) != 0); // illegal
    }
    // user, spr 17 (bit4 set): privileged
    {
        Cpu c;
        c.attach(bus);
        c.reset();
        c.st.pc = 0x1000;
        c.st.msr = msr::PR;
        bus.write32(0x1000, encMfspr(3, 17));
        c.step();
        CHECK(c.st.pc == 0x700u);
        CHECK((c.st.srr1 & 0x00040000u) != 0); // privileged
    }
    // supervisor, spr 3: illegal
    {
        Cpu c;
        c.attach(bus);
        c.reset();
        c.st.pc = 0x1000;
        c.st.msr = 0;
        bus.write32(0x1000, encMfspr(3, 3));
        c.step();
        CHECK(c.st.pc == 0x700u);
        CHECK((c.st.srr1 & 0x00080000u) != 0);
    }
}
