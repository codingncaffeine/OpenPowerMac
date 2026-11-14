// The data memo against the path it stands in for.
//
// Cpu::readV / writeV answer a repeated access to the same 32 bytes from a
// memo (Cpu::DmLine) that replays what the full path — translate(), then the
// L1 model — would have done to the machine. Its claim is "the same machine,
// bit for bit", and a claim like that is settled the way the line executor's
// and the JIT's were: twin machines, the same guest program, one with the
// memo and one walking every access (dxlCacheOff, which is what --no-dxlate
// sets), compared state item for state item after every batch.
//
// The comparison covers what the memo touches and what it could get wrong:
// the registers, both TLBs and their LRU bits, every L1 line with its tag,
// age and dirty bit, the L1's clock, and all of memory.
//
// The guest program is built to move every input a memo entry depends on
// while entries for it are live: page-table and BAT mappings, real mode, two
// address spaces switched with mtsr, a BAT rewritten in place, tlbie, the
// cache-control instructions, more pages than a TLB set holds and more lines
// than an L1 set holds, accesses that cross a line, and a store that rewrites
// an instruction the loop is about to execute.

#include "doctest.h"
#include "opm/cpu.hpp"

#include <cstring>
#include <memory>
#include <string>
#include <vector>

using namespace opm;

namespace {

class RamBus final : public Bus {
public:
    std::vector<u8> m;
    explicit RamBus(size_t n) : m(n, 0) {}

    u8 read8(u32 pa) override { return pa < m.size() ? m[pa] : 0; }
    u16 read16(u32 pa) override
    {
        return static_cast<u16>((read8(pa) << 8) | read8(pa + 1));
    }
    u32 read32(u32 pa) override
    {
        return (u32(read8(pa)) << 24) | (u32(read8(pa + 1)) << 16) |
               (u32(read8(pa + 2)) << 8) | u32(read8(pa + 3));
    }
    u64 read64(u32 pa) override
    {
        return (u64(read32(pa)) << 32) | read32(pa + 4);
    }
    void write8(u32 pa, u8 v) override
    {
        if (pa < m.size())
            m[pa] = v;
    }
    void write16(u32 pa, u16 v) override
    {
        write8(pa, u8(v >> 8));
        write8(pa + 1, u8(v));
    }
    void write32(u32 pa, u32 v) override
    {
        write8(pa, u8(v >> 24));
        write8(pa + 1, u8(v >> 16));
        write8(pa + 2, u8(v >> 8));
        write8(pa + 3, u8(v));
    }
    void write64(u32 pa, u64 v) override
    {
        write32(pa, u32(v >> 32));
        write32(pa + 4, u32(v));
    }
    bool memoryAt(u32 pa, u32 len) const override
    {
        return size_t(pa) + len <= m.size();
    }
};

// ---- the machine's layout ----------------------------------------------------
//
//   0x00001000  code, fetched in real mode
//   0x00080000  16 data pages (physical)
//   0x000C0000  what DBAT0 maps, and 0x000E0000 what it is rewritten to map
//   0x00100000  the hashed page table, 64 KB
//   0x00400000  the data pages' VIRTUAL addresses, one every 64 pages so that
//               all sixteen share one TLB set; in real mode the same
//               addresses are plain memory
//   0x00C00000  a virtual alias of the code page, for the self-modifying store
//   0x20000000  the BAT-mapped window
constexpr u32 kCode = 0x1000;
constexpr u32 kPhys = 0x00080000;
constexpr u32 kHtab = 0x00100000;
constexpr u32 kVirt = 0x00400000;
constexpr u32 kAlias = 0x00C00000;
constexpr u32 kBatEa = 0x20000000;
constexpr u32 kVsidA = 0x111, kVsidB = 0x222;
constexpr u32 kRam = 16u << 20;

u32 dform(u32 op, u32 rt, u32 ra, u32 d)
{
    return op << 26 | rt << 21 | ra << 16 | (d & 0xFFFFu);
}
u32 x31(u32 rt, u32 ra, u32 rb, u32 xo)
{
    return 31u << 26 | rt << 21 | ra << 16 | rb << 11 | xo << 1;
}
u32 rlwinm(u32 rs, u32 ra, u32 sh, u32 mb, u32 me)
{
    return 21u << 26 | rs << 21 | ra << 16 | sh << 11 | mb << 6 | me << 1;
}
u32 sprSplit(u32 n) { return (n & 31u) << 16 | (n >> 5) << 11; }
u32 mtspr(u32 spr, u32 rs) { return 31u << 26 | rs << 21 | sprSplit(spr) | 467u << 1; }

struct Rng {
    u64 s;
    u32 next()
    {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        return u32(s >> 32);
    }
};

struct Machine {
    RamBus bus{kRam};
    std::unique_ptr<Cpu> cpuUp = std::make_unique<Cpu>();
    Cpu& cpu = *cpuUp;
    u64 stamp = 0;

    explicit Machine(bool memo)
    {
        cpu.attach(bus);
        cpu.jitOn = false; // one variable at a time: the JIT has its own twins
        cpu.dxlCacheOff = !memo;
    }

    // One page-table entry in the primary group, as the guest's OS would
    // have left it.
    bool map(u32 vsid, u32 ea, u32 pa)
    {
        const u32 pi = (ea >> 12) & 0xFFFFu;
        const u32 hash = (vsid & 0x7FFFFu) ^ pi;
        const u32 pteg = kHtab | ((hash & 0x3FFu) << 6);
        for (u32 slot = 0; slot < 8; ++slot) {
            const u32 at = pteg + slot * 8u;
            if (bus.read32(at) & 0x80000000u)
                continue;
            bus.write32(at, 0x80000000u | (vsid << 7) | (pi >> 10));
            bus.write32(at + 4, (pa & 0xFFFFF000u) | 2u); // PP = read/write
            return true;
        }
        return false;
    }

    bool setUp(const std::vector<u32>& prog)
    {
        bool ok = true;
        for (u32 k = 0; k < 16; ++k) {
            ok = map(kVsidA, kVirt + (k << 18), kPhys + (k << 12)) && ok;
            ok = map(kVsidB, kVirt + (k << 18), kPhys + ((15u - k) << 12)) && ok;
        }
        // The alias covers every page the program occupies.
        for (u32 p = 0; p * 4096u < prog.size() * 4u + 4096u; ++p) {
            ok = map(kVsidA, kAlias + (p << 12), kCode + (p << 12)) && ok;
            ok = map(kVsidB, kAlias + (p << 12), kCode + (p << 12)) && ok;
        }
        for (size_t k = 0; k < prog.size(); ++k)
            bus.write32(kCode + u32(k) * 4u, prog[k]);
        for (u32 k = 0; k < 0x10000u; ++k)
            bus.write8(kPhys + k, u8(k * 7u + (k >> 8)));
        for (u32 k = 0; k < 0x40000u; ++k)
            bus.write8(0x000C0000u + k, u8(k * 13u + 5u));

        CpuState& s = cpu.st;
        s.pc = kCode;
        s.msr = msr::DR | msr::FP; // data translated, code in real mode
        s.hid0 = 0x0000C000u;      // ICE | DCE
        s.sdr1 = kHtab;            // 64 KB table
        s.sr[0] = kVsidA;
        s.sr[2] = 0x333u;
        s.dbatu[0] = kBatEa | 2u; // 128 KB, supervisor
        s.dbatl[0] = 0x000C0000u | 2u;
        s.gpr[3] = 0x12345678u;                 // the address generator
        s.gpr[9] = 4;                           // an index for the X forms
        s.gpr[20] = kVirt;
        s.gpr[21] = kBatEa;
        s.gpr[22] = kAlias;
        s.gpr[24] = kVsidA;
        s.gpr[25] = kVsidB;
        s.gpr[26] = 0x000C0000u | 2u;
        s.gpr[27] = 0x000E0000u | 2u;
        s.gpr[28] = msr::DR | msr::FP;
        s.gpr[29] = msr::FP;
        s.ctr = 0x7FFFFFFFu;
        return ok;
    }

    void run(u64 n) { cpu.runSteps(n, stamp); }
};

// The loop body: straight-line ops chosen at random, then a branch back. One
// word in it is an `addi r11, r11, IMM` that the body itself rewrites through
// the alias mapping; `smcAt` says where so the stores can be aimed at it.
std::vector<u32> buildProgram(Rng& r, u32 ops)
{
    static const u32 disp[] = {0, 1, 3, 13, 28, 29, 30, 56};
    std::vector<u32> p;
    const u32 smcAt = 0; // word 0 of the body
    p.push_back(dform(14, 11, 11, 1)); // addi r11, r11, 1 (rewritten later)
    auto newAddress = [&] {
        p.push_back(dform(7, 3, 3, 0x4E6D));      // mulli r3, r3, 20077
        p.push_back(dform(14, 3, 3, 12345));      // addi  r3, r3, 12345
        p.push_back(rlwinm(3, 4, 7, 10, 13));     // r4 = page << 18
        p.push_back(rlwinm(3, 6, 0, 20, 25));     // r6 = offset, a multiple of 64
        p.push_back(x31(5, 20, 4, 266));          // add r5, r20, r4
        p.push_back(x31(5, 5, 6, 266));           // add r5, r5, r6
    };
    newAddress();
    for (u32 k = 0; k < ops; ++k) {
        const u32 d = disp[r.next() & 7u];
        switch (r.next() % 40u) {
        case 0: case 1: case 2: case 3: case 4:
            newAddress();
            break;
        case 5: case 6: case 7:
            p.push_back(dform(32, 7, 5, d)); // lwz r7, d(r5)
            p.push_back(x31(8, 8, 7, 266));  // add r8, r8, r7
            break;
        case 8: case 9:
            p.push_back(dform(34, 7, 5, d)); // lbz
            p.push_back(x31(8, 8, 7, 266));
            break;
        case 10:
            p.push_back(dform(40, 7, 5, d)); // lhz
            p.push_back(x31(8, 8, 7, 266));
            break;
        case 11:
            p.push_back(dform(42, 7, 5, d)); // lha
            p.push_back(x31(8, 8, 7, 266));
            break;
        case 12: case 13: case 14:
            p.push_back(dform(36, 8, 5, d)); // stw r8, d(r5)
            break;
        case 15: case 16:
            p.push_back(dform(38, 8, 5, d)); // stb
            break;
        case 17:
            p.push_back(dform(44, 8, 5, d)); // sth
            break;
        case 18:
            p.push_back(x31(7, 5, 9, 23));   // lwzx r7, r5, r9
            p.push_back(x31(8, 8, 7, 266));
            break;
        case 19:
            p.push_back(x31(8, 5, 9, 151));  // stwx r8, r5, r9
            break;
        // The floating-point forms take an alignment exception off a word
        // boundary (UM 4.6.6), so theirs is rounded down: 28 still crosses
        // into the next line.
        case 20:
            p.push_back(dform(50, 1, 5, d & ~3u)); // lfd f1, d(r5)
            break;
        case 21:
            p.push_back(dform(54, 1, 5, d & ~3u)); // stfd f1, d(r5)
            break;
        case 22: case 23:
            p.push_back(x31(10, 21, 6, 266)); // add r10, r21, r6 (BAT window)
            p.push_back(dform(32, 7, 10, d)); // lwz
            p.push_back(x31(8, 8, 7, 266));
            break;
        case 24:
            p.push_back(x31(10, 21, 6, 266));
            p.push_back(dform(36, 8, 10, d)); // stw into the BAT window
            break;
        case 25:
            p.push_back(x31(0, 0, 5, 1014)); // dcbz 0, r5
            break;
        case 26:
            p.push_back(x31(0, 0, 5, 86));   // dcbf
            break;
        case 27:
            p.push_back(x31(0, 0, 5, 54));   // dcbst
            break;
        case 28:
            p.push_back(x31(0, 0, 5, 470));  // dcbi
            break;
        case 29: case 30:
            p.push_back(x31(0, 0, 5, 306));  // tlbie r5
            break;
        case 31:
            p.push_back(31u << 26 | 24u << 21 | 210u << 1); // mtsr 0, r24
            break;
        case 32:
            p.push_back(31u << 26 | 25u << 21 | 210u << 1); // mtsr 0, r25
            break;
        case 33:
            p.push_back(mtspr(537, 26 + (r.next() & 1u))); // DBAT0L
            break;
        case 34:
            p.push_back(31u << 26 | 29u << 21 | 146u << 1); // mtmsr r29: DR off
            break;
        case 35: case 36:
            p.push_back(31u << 26 | 28u << 21 | 146u << 1); // mtmsr r28: DR on
            break;
        default:
            // Rewrite the addi at the top of the body with a new immediate.
            p.push_back(rlwinm(3, 12, 0, 17, 31));     // r12 = r3 & 0x7FFF
            p.push_back(dform(25, 12, 12, 0x396B));    // oris: addi r11,r11,…
            p.push_back(dform(36, 12, 22, smcAt * 4)); // stw r12, 0(r22)
            break;
        }
    }
    p.push_back(31u << 26 | 28u << 21 | 146u << 1); // DR back on for the top
    const i32 back = -static_cast<i32>(p.size()) * 4;
    p.push_back(18u << 26 | (u32(back) & 0x03FFFFFCu)); // b body
    return p;
}

bool sameTlb(const Cpu::TlbEntry (&a)[64][2], const Cpu::TlbEntry (&b)[64][2])
{
    for (u32 s = 0; s < 64; ++s)
        for (u32 w = 0; w < 2; ++w) {
            const Cpu::TlbEntry& x = a[s][w];
            const Cpu::TlbEntry& y = b[s][w];
            if (x.v != y.v || x.c != y.c || x.vsid != y.vsid || x.pi != y.pi ||
                x.rpn != y.rpn || x.wimg != y.wimg || x.pp != y.pp)
                return false;
        }
    return true;
}

// Names the first thing that differs, or nullptr.
const char* firstDifference(Machine& a, Machine& b)
{
    const CpuState& x = a.cpu.st;
    const CpuState& y = b.cpu.st;
    if (std::memcmp(x.gpr, y.gpr, sizeof x.gpr) != 0) return "a GPR";
    if (std::memcmp(x.fpr, y.fpr, sizeof x.fpr) != 0) return "an FPR";
    if (x.pc != y.pc) return "pc";
    if (x.msr != y.msr) return "MSR";
    if (x.cr != y.cr || x.xer != y.xer || x.ctr != y.ctr) return "CR/XER/CTR";
    if (x.srr0 != y.srr0 || x.srr1 != y.srr1 || x.dar != y.dar ||
        x.dsisr != y.dsisr)
        return "exception state";
    if (x.tb != y.tb || x.dec != y.dec) return "the clock";
    if (a.stamp != b.stamp) return "the instruction count";
    if (a.cpu.halted != b.cpu.halted) return "halted";
    if (!sameTlb(a.cpu.dtlb, b.cpu.dtlb)) return "the data TLB";
    if (!sameTlb(a.cpu.itlb, b.cpu.itlb)) return "the instruction TLB";
    if (std::memcmp(a.cpu.dtlbLru, b.cpu.dtlbLru, 64) != 0) return "the data TLB's LRU bits";
    if (std::memcmp(a.cpu.itlbLru, b.cpu.itlbLru, 64) != 0) return "the instruction TLB's LRU bits";
    if (a.cpu.l1dClock != b.cpu.l1dClock) return "the L1's clock";
    for (u32 s = 0; s < 128; ++s) {
        if (std::memcmp(a.cpu.l1x[s].tv, b.cpu.l1x[s].tv, sizeof a.cpu.l1x[s].tv) != 0)
            return "an L1 tag";
        if (std::memcmp(a.cpu.l1x[s].age, b.cpu.l1x[s].age, sizeof a.cpu.l1x[s].age) != 0)
            return "an L1 age";
        for (u32 w = 0; w < 8; ++w) {
            if (a.cpu.l1d[s][w].d != b.cpu.l1d[s][w].d) return "an L1 dirty bit";
            if (std::memcmp(a.cpu.l1d[s][w].b, b.cpu.l1d[s][w].b, 32) != 0)
                return "an L1 line's bytes";
        }
    }
    if (std::memcmp(a.bus.m.data(), b.bus.m.data(), a.bus.m.size()) != 0)
        return "memory";
    return nullptr;
}

u32 liveEntries(const Cpu& c)
{
    u32 n = 0;
    for (u32 k = 0; k < Cpu::kDmLines; ++k)
        if (c.dmLine[k].eaLine != ~0u)
            ++n;
    return n;
}

} // namespace

TEST_CASE("data memo: the same machine as walking every access")
{
    for (u32 iter = 0; iter < 12; ++iter) {
        Rng r{0xD4A7A000ull + iter * 0x9E3779B9ull};
        const std::vector<u32> prog = buildProgram(r, 500);
        Machine memo(true), walk(false);
        REQUIRE(memo.setUp(prog));
        REQUIRE(walk.setUp(prog));
        const char* diff = nullptr;
        u64 done = 0;
        for (u32 batch = 0; batch < 220 && !diff; ++batch) {
            const u64 n = 1u + r.next() % 3000u;
            memo.run(n);
            walk.run(n);
            done += n;
            diff = firstDifference(memo, walk);
            // Between batches, what a harness is allowed to do behind the
            // processor's back: a segment register poked directly, the TLB
            // flushed, the cache flushed to memory.
            switch (r.next() % 24u) {
            case 0:
                memo.cpu.st.sr[0] = walk.cpu.st.sr[0] =
                    (r.next() & 1u) ? kVsidA : kVsidB;
                break;
            case 1:
                memo.cpu.tlbFlushAll();
                walk.cpu.tlbFlushAll();
                break;
            case 2:
                memo.cpu.l1dFlushAll(true);
                walk.cpu.l1dFlushAll(true);
                break;
            default:
                break;
            }
        }
        CAPTURE(iter);
        CAPTURE(done);
        const std::string what = diff ? diff : "nothing";
        CAPTURE(what); // the first thing that differed
        CHECK(diff == nullptr);
        CHECK(!memo.cpu.halted);
        // …and only if the program was still the program: a stray fault would
        // leave both machines agreeing about an exception loop.
        CHECK(memo.cpu.st.pc >= kCode);
        CHECK(memo.cpu.st.pc < kCode + prog.size() * 4u);
        CHECK(memo.cpu.excRingAt == 0u);
        // The comparison above means something only if one machine used the
        // memo and the other did not.
        CHECK(liveEntries(memo.cpu) > 16u);
        CHECK(liveEntries(walk.cpu) == 0u);
    }
}

TEST_CASE("data memo: a repeated access is answered from the entry")
{
    // The control for the differential above: if a hit were never taken, the
    // twins would agree whatever the memo held. Two lines are loaded, the
    // first one's entry is then pointed at the second one's block, and the
    // next load of the first address has to come back with the second's
    // bytes — on the machine with the memo, and only there.
    const std::vector<u32> prog = {
        dform(32, 7, 5, 0),   // lwz r7, 0(r5)
        dform(32, 6, 10, 0),  // lwz r6, 0(r10)
        18u << 26 | (u32(-8) & 0x03FFFFFCu), // b top
    };
    Machine memo(true), walk(false);
    REQUIRE(memo.setUp(prog));
    REQUIRE(walk.setUp(prog));
    for (Machine* m : {&memo, &walk}) {
        m->cpu.st.gpr[5] = kVirt;             // page 0
        // Page 1, and a different offset: the table is direct-mapped on
        // the line number, and lines 128 KB apart would share an entry.
        m->cpu.st.gpr[10] = kVirt + 0x40040u;
        m->run(30);
    }
    const u32 first = memo.cpu.st.gpr[7], second = memo.cpu.st.gpr[6];
    REQUIRE(first != second);
    REQUIRE(walk.cpu.st.gpr[7] == first);

    Cpu::DmLine& a = memo.cpu.dmLine[(kVirt >> 5) & (Cpu::kDmLines - 1u)];
    const Cpu::DmLine& b =
        memo.cpu.dmLine[((kVirt + 0x40040u) >> 5) & (Cpu::kDmLines - 1u)];
    REQUIRE(a.eaLine == kVirt >> 5);
    REQUIRE(b.eaLine == (kVirt + 0x40040u) >> 5);
    a.paLine = b.paLine;
    a.tv = b.tv;
    a.set = b.set;
    a.way = b.way;

    memo.run(30);
    walk.run(30);
    CHECK(memo.cpu.st.gpr[7] == second); // served from the redirected entry
    CHECK(walk.cpu.st.gpr[7] == first);  // the full path never saw it
}

TEST_CASE("data memo: a store through an entry still drops the fetched copy")
{
    // Self-modifying code, with the store arriving through an entry rather
    // than through the full path. The loop rewrites its own first instruction
    // on every trip, through a second mapping of the code page; from the
    // second trip on that store is a hit, and if a hit did not drop the
    // fetch buffer's copy of the line the loop would go on executing the
    // instruction it had the trip before.
    const std::vector<u32> prog = {
        dform(14, 11, 11, 1),                 // top: addi r11, r11, IMM
        dform(14, 12, 12, 3),                 // addi r12, r12, 3
        rlwinm(12, 13, 0, 17, 31),            // r13 = r12 & 0x7FFF
        dform(25, 13, 13, 0x396B),            // oris: "addi r11, r11, r13"
        dform(36, 13, 22, 0),                 // stw r13, 0(r22): word 0
        18u << 26 | (u32(-20) & 0x03FFFFFCu), // b top
    };
    Machine memo(true), walk(false);
    REQUIRE(memo.setUp(prog));
    REQUIRE(walk.setUp(prog));
    memo.run(600);
    walk.run(600);
    // 100 trips: r11 is 1 + the sum of the immediates stored on trips 1-99.
    u32 want = 1;
    for (u32 trip = 1; trip < 100; ++trip)
        want += (trip * 3u) & 0x7FFFu;
    CHECK(walk.cpu.st.gpr[11] == want);
    CHECK(memo.cpu.st.gpr[11] == want);
    CHECK(firstDifference(memo, walk) == nullptr);
    const Cpu::DmLine& e = memo.cpu.dmLine[(kAlias >> 5) & (Cpu::kDmLines - 1u)];
    CHECK(e.eaLine == kAlias >> 5); // the stores did go through an entry
    CHECK(e.wkey != Cpu::kDmNever);
}

TEST_CASE("data memo: off outside a batch and while a watch is armed")
{
    const std::vector<u32> prog = {
        dform(32, 7, 5, 0),                  // lwz r7, 0(r5)
        dform(36, 7, 5, 4),                  // stw r7, 4(r5)
        18u << 26 | (u32(-8) & 0x03FFFFFCu), // b top
    };
    {
        // step() is the path every test rig and instrumented loop uses.
        Machine m(true);
        REQUIRE(m.setUp(prog));
        m.cpu.st.gpr[5] = kVirt;
        for (u32 k = 0; k < 30; ++k)
            m.cpu.step();
        CHECK(liveEntries(m.cpu) == 0u);
        CHECK(m.cpu.dmLive == Cpu::kDmOff);
    }
    {
        // A store watchpoint lives on the full path: with one armed, every
        // store has to reach it.
        Machine m(true);
        REQUIRE(m.setUp(prog));
        m.cpu.st.gpr[5] = kVirt;
        m.run(30); // entries are made
        REQUIRE(liveEntries(m.cpu) > 0u);
        m.cpu.wpPa = kPhys + 4u;
        m.cpu.wpEnd = kPhys + 7u;
        const u64 before = m.cpu.wpHits;
        m.run(30); // ten trips round the loop, one store each
        CHECK(m.cpu.wpHits - before == 10u);
    }
}
