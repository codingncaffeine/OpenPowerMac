// Twin machines: everything that stands in for the full path, against the
// full path.
//
// Three things in the processor answer a question from what they remember
// instead of working it out again, and each claims to leave "the same
// machine, bit for bit":
//
//   the data memo        Cpu::readV / writeV answer a repeated access to the
//   (Cpu::DmLine)        same 32 bytes by replaying what translate() and the
//                        L1 model would have done;
//   the page memo        the same for the page an instruction is fetched
//   (Cpu::ImPage)        from;
//   the line executor    Cpu::runSteps executes straight-line code out of a
//                        resident fetch block without the per-instruction
//                        fetch path.
//
// A claim like that is settled the way the JIT's was: twin machines, the
// same guest program, one with the shortcut and one without (dxlCacheOff,
// which is what --no-dxlate sets, walks every access; lineExecOff, which is
// --no-line-exec, takes one instruction per trip through step()), compared
// state item for state item after every batch.
//
// The comparison covers what the shortcuts touch and what they could get
// wrong: the registers, both TLBs and their LRU bits, every L1 line with its
// tag, age and dirty bit, the L1's clock, and all of memory.
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
    // Bus::directRam: the first `directBytes` of memory are offered to the
    // processor to read and write in place. Zero, the default, keeps the
    // full cache model, which is what every test above the last section
    // runs on.
    u32 directBytes = 0;
    bool directRam(u8*& base, u32& size) override
    {
        if (!directBytes)
            return false;
        base = m.data();
        size = directBytes;
        return true;
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
constexpr u32 kBatInRam = 0x00A00000; // the same window, inside memory
constexpr u32 kVsidA = 0x111, kVsidB = 0x222;
constexpr u32 kRam = 13u << 20;

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
    // Where the BAT-mapped window is. The program also reaches for it in
    // real mode, where the default address has nothing behind it at all: a
    // store there lives in the caches for as long as they happen to keep it.
    // That is the one thing the two cache modes are allowed to disagree
    // about, so their twins put the window where real mode finds memory
    // (set before setUp).
    u32 batEa = kBatEa;

    // The JIT is off unless asked for: one variable at a time, and it has
    // its own twins in test_jit.cpp. The last test here turns everything on.
    explicit Machine(bool memo, bool lineExec = true, bool jit = false)
    {
        cpu.attach(bus);
        cpu.jitOn = jit;
        cpu.dxlCacheOff = !memo;
        cpu.lineExecOff = !lineExec;
    }

    // The caches as an operating system runs them: the L2 enabled, no test
    // mode. Call after setUp. With `direct`, the bus offers its first
    // `bytes` and the processor reads memory there in place from its next
    // batch on; without, the same registers and the full model
    // (Cpu::exactCaches, the control).
    void normalCaches(bool direct, u32 bytes = kRam)
    {
        cpu.st.l2cr = 0x80000000u;
        cpu.l2Resize();
        bus.directBytes = bytes;
        cpu.exactCaches = !direct;
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
        s.dbatu[0] = batEa | 2u; // 128 KB, supervisor
        s.dbatl[0] = 0x000C0000u | 2u;
        s.gpr[3] = 0x12345678u;                 // the address generator
        s.gpr[9] = 4;                           // an index for the X forms
        s.gpr[20] = kVirt;
        s.gpr[21] = batEa;
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
std::vector<u32> buildProgram(Rng& r, u32 ops, bool dcbi = true)
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
            // dcbi throws a modified block away, which a machine reading
            // memory directly cannot do: its twins get a dcbf instead.
            p.push_back(x31(0, 0, 5, dcbi ? 470 : 86));
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

// Run both machines to `target` instructions and name the first thing that
// differs, or nullptr.
//
// ⚠ IN INSTRUCTIONS, NOT IN CALLS. runSteps returns at the first instruction
// that ends a batch, and this program has one every dozen or so (mtsr, tlbie,
// mtmsr, a BAT write), so a fixed number of calls is a few thousand
// instructions — three trips round the loop, which is not a test of anything
// that has to warm up. The registers are compared after every call and the
// whole machine every 251st, and at the end.
//
// `pokes` adds, between batches, what a harness is allowed to do behind the
// processor's back: a segment register written directly, the TLB flushed,
// the cache flushed to memory.
const char* runTwins(Machine& a, Machine& b, u64 target, Rng& r, bool pokes,
                     const char* (*cmp)(Machine&, Machine&) = firstDifference)
{
    u64 calls = 0;
    while (a.stamp < target) {
        const u64 n = 1u + r.next() % 3000u;
        a.run(n);
        b.run(n);
        ++calls;
        if (a.stamp != b.stamp)
            return "the instruction count";
        if (std::memcmp(a.cpu.st.gpr, b.cpu.st.gpr, sizeof a.cpu.st.gpr) != 0)
            return "a GPR";
        if (a.cpu.st.pc != b.cpu.st.pc || a.cpu.st.msr != b.cpu.st.msr)
            return "pc or MSR";
        if (calls % 251u == 0u)
            if (const char* d = cmp(a, b))
                return d;
        if (!pokes)
            continue;
        switch (r.next() % 400u) {
        case 0:
            a.cpu.st.sr[0] = b.cpu.st.sr[0] = (r.next() & 1u) ? kVsidA : kVsidB;
            break;
        case 1:
            a.cpu.tlbFlushAll();
            b.cpu.tlbFlushAll();
            break;
        case 2:
            a.cpu.l1dFlushAll(true);
            b.cpu.l1dFlushAll(true);
            break;
        default:
            break;
        }
    }
    return cmp(a, b);
}

// ---- memory read directly -------------------------------------------------------

// Memory as a load would return it: the bus's bytes with every modified
// block written over them.
std::vector<u8> memoryAsRead(const Machine& m)
{
    std::vector<u8> v = m.bus.m;
    m.cpu.publishModified(v.data(), static_cast<u32>(v.size()));
    return v;
}

// What a guest can know of the two machines: everything firstDifference
// compares except where the caches hold a block, and memory as it reads.
const char* firstGuestDifference(Machine& a, Machine& b)
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
    if (x.hid0 != y.hid0 || x.l2cr != y.l2cr) return "a cache control register";
    if (x.tb != y.tb || x.dec != y.dec) return "the clock";
    if (a.stamp != b.stamp) return "the instruction count";
    if (a.cpu.halted != b.cpu.halted) return "halted";
    if (!sameTlb(a.cpu.dtlb, b.cpu.dtlb)) return "the data TLB";
    if (!sameTlb(a.cpu.itlb, b.cpu.itlb)) return "the instruction TLB";
    if (std::memcmp(a.cpu.dtlbLru, b.cpu.dtlbLru, 64) != 0) return "the data TLB's LRU bits";
    if (std::memcmp(a.cpu.itlbLru, b.cpu.itlbLru, 64) != 0) return "the instruction TLB's LRU bits";
    if (memoryAsRead(a) != memoryAsRead(b)) return "memory as it reads";
    return nullptr;
}

u32 livePages(const Cpu& c)
{
    u32 n = 0;
    for (u32 k = 0; k < Cpu::kDpPages; ++k)
        if (c.dpPage[k].page != ~0u)
            ++n;
    return n;
}

// Blocks of main memory in either cache. None, while memory is read directly.
u32 cachedBlocks(const Cpu& c, u32 below)
{
    u32 n = 0;
    for (u32 s = 0; s < 128; ++s)
        for (u32 w = 0; w < 8; ++w) {
            const u32 tv = c.l1x[s].tv[w];
            if ((tv & 1u) && (((tv >> 1) << 12) | (s << 5)) < below)
                ++n;
        }
    for (const Cpu::L2Line& e : c.l2)
        if (e.v && (static_cast<u64>(e.tag) << 5) < below)
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
        const char* diff = runTwins(memo, walk, 150000, r, true);
        CAPTURE(iter);
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

// ---- the instruction page memo -------------------------------------------------

TEST_CASE("page memo: the ITLB's replacement order follows every change of page")
{
    // Code fetched through the page table, on three pages that share one
    // ITLB set of two ways. The loop goes A, B, A, B and every fourth time
    // round A, C: which of A and B the reload for C evicts depends on which
    // was used last, and that is the LRU write a change of page makes — the
    // one thing a memo hit has to replay. Get it wrong and the walking twin
    // ends up holding a different pair of pages.
    constexpr u32 kA = 0x01000000u, kB = 0x01040000u, kC = 0x01080000u;
    constexpr u32 kPa = 0x00300000u;
    Machine memo(true), walk(false);
    for (Machine* m : {&memo, &walk}) {
        REQUIRE(m->setUp({}));
        REQUIRE(m->map(kVsidA, kA, kPa));
        REQUIRE(m->map(kVsidA, kB, kPa + 0x1000u));
        REQUIRE(m->map(kVsidA, kC, kPa + 0x2000u));
        const u32 a[] = {
            dform(14, 3, 3, 1),                           // addi r3, r3, 1
            dform(28, 3, 4, 3),                           // andi. r4, r3, 3
            16u << 26 | 12u << 21 | 2u << 16 | 8u,        // beq +8
            18u << 26 | ((kB - (kA + 12u)) & 0x03FFFFFCu), // b B
            18u << 26 | ((kC - (kA + 16u)) & 0x03FFFFFCu), // b C
        };
        for (u32 k = 0; k < 5; ++k)
            m->bus.write32(kPa + k * 4u, a[k]);
        m->bus.write32(kPa + 0x1000u, dform(14, 5, 5, 1)); // B: addi r5, r5, 1
        m->bus.write32(kPa + 0x1004u,
                       18u << 26 | ((kA - (kB + 4u)) & 0x03FFFFFCu)); // b A
        m->bus.write32(kPa + 0x2000u, dform(14, 6, 6, 1)); // C: addi r6, r6, 1
        m->bus.write32(kPa + 0x2004u,
                       18u << 26 | ((kA - (kC + 4u)) & 0x03FFFFFCu)); // b A
        m->cpu.st.msr = msr::IR;
        m->cpu.st.pc = kA;
        m->cpu.st.gpr[3] = m->cpu.st.gpr[5] = m->cpu.st.gpr[6] = 0;
    }
    Rng r{0x1A6E5ull};
    const char* diff = runTwins(memo, walk, 6000, r, false);
    const std::string what = diff ? diff : "nothing";
    CAPTURE(what);
    CHECK(diff == nullptr);
    CHECK(memo.cpu.excRingAt == 0u);
    // A was entered about a thousand times, a quarter of them on the way to C.
    CHECK(memo.cpu.st.gpr[3] > 900u);
    CHECK(memo.cpu.st.gpr[6] * 4u >= memo.cpu.st.gpr[3] - 4u);
    CHECK(memo.cpu.st.gpr[5] > memo.cpu.st.gpr[6]);
    // The memo twin did hold the pages, and the walking twin held nothing.
    u32 held = 0, heldWalk = 0;
    for (u32 k = 0; k < Cpu::kImPages; ++k) {
        held += memo.cpu.imPage[k].page != ~0u;
        heldWalk += walk.cpu.imPage[k].page != ~0u;
    }
    CHECK(held >= 3u);
    CHECK(heldWalk == 0u);
}

// ---- the line executor --------------------------------------------------------

TEST_CASE("line executor: the same machine as one instruction at a time")
{
    for (u32 iter = 0; iter < 6; ++iter) {
        Rng r{0x11E3EC00ull + iter * 0x9E3779B9ull};
        const std::vector<u32> prog = buildProgram(r, 500);
        Machine line(true, true), single(true, false);
        REQUIRE(line.setUp(prog));
        REQUIRE(single.setUp(prog));
        const char* diff = runTwins(line, single, 150000, r, false);
        const std::string what = diff ? diff : "nothing";
        CAPTURE(iter);
        CAPTURE(what);
        CHECK(diff == nullptr);
        CHECK(line.cpu.excRingAt == 0u);
    }
}

TEST_CASE("line executor: an interrupt enabled by mtmsr is taken before the next instruction")
{
    // An external interrupt is waiting with MSR[EE] clear. The mtmsr that
    // sets EE is in the middle of a block; the interrupt belongs between it
    // and the instruction after it, wherever in the block that falls.
    const std::vector<u32> prog = {
        dform(14, 3, 3, 1),               // 1000: addi r3, r3, 1
        dform(14, 9, 0, 0),               // 1004: addi r9, 0, 0
        dform(24, 9, 9, 0x8000),          // 1008: ori  r9, r9, EE
        31u << 26 | 9u << 21 | 146u << 1, // 100c: mtmsr r9
        dform(14, 3, 3, 1),               // 1010: addi r3, r3, 1
        dform(14, 3, 3, 1),               // 1014
        dform(14, 3, 3, 1),               // 1018
        18u << 26,                        // 101c: b .
    };
    for (const bool lineExec : {true, false}) {
        Machine m(true, lineExec);
        REQUIRE(m.setUp(prog));
        m.cpu.st.msr = 0;
        m.cpu.st.gpr[3] = m.cpu.st.gpr[5] = 0;
        m.bus.write32(0x500, dform(14, 5, 5, 1)); // the handler: addi r5, r5, 1
        m.bus.write32(0x504, 18u << 26);          // b .
        m.cpu.setExternalIrq(true);
        // Several batches: the mtmsr ends the one it is in, and the handler
        // spins once it has counted itself.
        for (u32 k = 0; k < 4; ++k)
            m.run(8);
        CAPTURE(lineExec);
        CHECK(m.cpu.st.srr0 == 0x1010u);
        CHECK(m.cpu.st.gpr[3] == 1u);
        CHECK(m.cpu.st.gpr[5] == 1u);
    }
}

TEST_CASE("line executor: instructions after an mtmsr are fetched under the new translation")
{
    // The mtmsr turns instruction translation on, and with it on the next
    // address maps somewhere else. What runs next is what is THERE, not the
    // rest of the block the mtmsr was fetched from.
    const std::vector<u32> prog = {
        dform(14, 9, 0, 0x20),            // 1000: addi r9, 0, IR
        31u << 26 | 9u << 21 | 146u << 1, // 1004: mtmsr r9
        dform(14, 3, 3, 1),               // 1008: addi r3, r3, 1   (real)
        dform(14, 3, 3, 1),               // 100c
        18u << 26,                        // 1010: b .
    };
    for (const bool lineExec : {true, false}) {
        Machine m(true, lineExec);
        REQUIRE(m.setUp(prog));
        m.cpu.st.msr = 0;
        m.cpu.st.gpr[3] = m.cpu.st.gpr[4] = 0;
        // IBAT0: the first 128 KB of effective addresses at 0x00200000.
        m.cpu.st.ibatu[0] = 0x00000002u;
        m.cpu.st.ibatl[0] = 0x00200000u | 2u;
        m.bus.write32(0x00201008u, dform(14, 4, 4, 7)); // addi r4, r4, 7
        m.bus.write32(0x0020100Cu, dform(14, 4, 4, 7));
        m.bus.write32(0x00201010u, 18u << 26);          // b .
        for (u32 k = 0; k < 4; ++k) // the mtmsr ends the batch it is in
            m.run(8);
        CAPTURE(lineExec);
        CHECK(m.cpu.st.gpr[3] == 0u);
        CHECK(m.cpu.st.gpr[4] == 14u);
    }
}

// ---- everything at once -------------------------------------------------------

TEST_CASE("jit with the memos: the same machine as one instruction at a time, walking")
{
    // What ships on a host with the JIT: compiled blocks whose loads and
    // stores go through the data memo and whose page changes go through the
    // page memo — against the machine with none of the three.
    for (u32 iter = 0; iter < 6; ++iter) {
        Rng r{0x717A11ull + iter * 0x9E3779B9ull};
        const std::vector<u32> prog = buildProgram(r, 500);
        Machine all(true, true, true), none(false, false, false);
        REQUIRE(all.setUp(prog));
        REQUIRE(none.setUp(prog));
        const char* diff = runTwins(all, none, 150000, r, false);
        const std::string what = diff ? diff : "nothing";
        CAPTURE(iter);
        CAPTURE(what);
        CHECK(diff == nullptr);
        CHECK(all.cpu.excRingAt == 0u);
        // Compiled blocks did run, and their loads and stores did go through
        // the shims the memo sits behind. Only where the JIT exists: other
        // hosts interpret and never create the cache.
#if defined(__x86_64__) || defined(_M_X64)
        REQUIRE(all.cpu.jit != nullptr);
#endif
        if (all.cpu.jit) {
            CHECK(all.cpu.jit->insns > 100000u);
            CHECK(all.cpu.jit->memOps > 20000u);
            CHECK(liveEntries(all.cpu) > 16u);
        }
    }
}

// ---- memory read directly -------------------------------------------------------
//
// Cpu::dmDirect: with the caches in their normal mode, main memory is read
// and written in place and the L1 and L2 hold none of it. The claim is that
// a guest cannot tell, so the twins here are compared on what a guest can
// know (firstGuestDifference): one reads memory directly, the other has the
// same registers and the full model.

TEST_CASE("direct memory: the same guest-visible machine as the full model")
{
    for (u32 iter = 0; iter < 12; ++iter) {
        Rng r{0xD12EC700ull + iter * 0x9E3779B9ull};
        const std::vector<u32> prog = buildProgram(r, 500, false);
        Machine direct(true), model(true);
        direct.batEa = model.batEa = kBatInRam;
        REQUIRE(direct.setUp(prog));
        REQUIRE(model.setUp(prog));
        direct.normalCaches(true);
        model.normalCaches(false);
        const char* diff =
            runTwins(direct, model, 150000, r, true, firstGuestDifference);
        CAPTURE(iter);
        const std::string what = diff ? diff : "nothing";
        CAPTURE(what);
        CHECK(diff == nullptr);
        CHECK(!direct.cpu.halted);
        CHECK(direct.cpu.st.pc >= kCode);
        CHECK(direct.cpu.st.pc < kCode + prog.size() * 4u);
        CHECK(direct.cpu.excRingAt == 0u);
        // The comparison means something only if one machine read memory in
        // place, through page entries, and the other kept it in the caches.
        CHECK(direct.cpu.dmDirect);
        CHECK(direct.cpu.directEnters == 1u);
        CHECK(livePages(direct.cpu) > 8u);
        CHECK(cachedBlocks(direct.cpu, kRam) == 0u);
        CHECK(!model.cpu.dmDirect);
        CHECK(model.cpu.directEnters == 0u);
        CHECK(livePages(model.cpu) == 0u);
    }
}

TEST_CASE("direct memory: every executor, and the mode switched under the program")
{
    // One twin keeps the full model throughout. The other is switched
    // between the two modes from outside every few batches, so the mode is
    // entered with modified blocks in both caches and left again, over and
    // over, under a running program. The JIT and the single-step executor
    // take turns on the switched twin.
    for (u32 iter = 0; iter < 6; ++iter) {
        Rng r{0x5A17C400ull + iter * 0x9E3779B9ull};
        const std::vector<u32> prog = buildProgram(r, 500, false);
        Machine sw(true, iter % 3u != 1u, iter % 3u == 2u), model(true);
        sw.batEa = model.batEa = kBatInRam;
        REQUIRE(sw.setUp(prog));
        REQUIRE(model.setUp(prog));
        sw.normalCaches(false);
        model.normalCaches(false);
        const char* diff = nullptr;
        u32 calls = 0;
        while (sw.stamp < 120000 && !diff) {
            const u64 n = 1u + r.next() % 2000u;
            sw.run(n);
            model.run(n);
            if (++calls % 7u == 0u) {
                diff = firstGuestDifference(sw, model);
                sw.cpu.exactCaches = !sw.cpu.exactCaches;
            }
        }
        if (!diff)
            diff = firstGuestDifference(sw, model);
        CAPTURE(iter);
        const std::string what = diff ? diff : "nothing";
        CAPTURE(what);
        CHECK(diff == nullptr);
        CHECK(sw.cpu.excRingAt == 0u);
        CHECK(sw.cpu.directEnters > 3u);
        CHECK(sw.cpu.directLeaves > 2u);
        CHECK(sw.cpu.directPushed > 0u); // it did go in with modified blocks
    }
}

TEST_CASE("direct memory: a repeated access is answered from the page entry")
{
    // The control for the differential above, as for the line memo: point
    // one page's entry at another page's bytes, and the next load of the
    // first address must come back with the second's.
    const std::vector<u32> prog = {
        dform(32, 7, 5, 0),   // lwz r7, 0(r5)
        dform(32, 6, 10, 0),  // lwz r6, 0(r10)
        18u << 26 | (u32(-8) & 0x03FFFFFCu), // b top
    };
    Machine m(true);
    REQUIRE(m.setUp(prog));
    m.normalCaches(true);
    m.cpu.st.gpr[5] = kVirt;
    m.cpu.st.gpr[10] = kVirt + 0x40040u; // the next page, another offset
    m.run(30);
    REQUIRE(m.cpu.dmDirect);
    const u32 first = m.cpu.st.gpr[7];
    Cpu::DpPage& a = m.cpu.dpPage[(kVirt >> 12) & (Cpu::kDpPages - 1u)];
    const Cpu::DpPage& b =
        m.cpu.dpPage[((kVirt + 0x40040u) >> 12) & (Cpu::kDpPages - 1u)];
    REQUIRE(a.page == kVirt >> 12);
    REQUIRE(b.page == (kVirt + 0x40040u) >> 12);
    a.host = b.host;
    m.run(30);
    const u32 redirected = m.cpu.st.gpr[7];
    CHECK(redirected != first);
    CHECK(redirected == m.bus.read32(kPhys + 0x1000u)); // page 1, offset 0
}

TEST_CASE("direct memory: a store through a page entry still drops the fetched copy")
{
    // The self-modifying loop of the line memo's test, on memory read
    // directly: from the second trip on the store is a hit on a page entry.
    const std::vector<u32> prog = {
        dform(14, 11, 11, 1),                 // top: addi r11, r11, IMM
        dform(14, 12, 12, 3),                 // addi r12, r12, 3
        rlwinm(12, 13, 0, 17, 31),            // r13 = r12 & 0x7FFF
        dform(25, 13, 13, 0x396B),            // oris: "addi r11, r11, r13"
        dform(36, 13, 22, 0),                 // stw r13, 0(r22): word 0
        18u << 26 | (u32(-20) & 0x03FFFFFCu), // b top
    };
    Machine m(true);
    REQUIRE(m.setUp(prog));
    m.normalCaches(true);
    m.run(600);
    u32 want = 1;
    for (u32 trip = 1; trip < 100; ++trip)
        want += (trip * 3u) & 0x7FFFu;
    CHECK(m.cpu.st.gpr[11] == want);
    const Cpu::DpPage& e = m.cpu.dpPage[(kAlias >> 12) & (Cpu::kDpPages - 1u)];
    CHECK(e.page == kAlias >> 12); // the stores did go through an entry
    CHECK(e.wkey != Cpu::kDmNever);
    // …and they are in memory already, with no cache to write back.
    CHECK(m.bus.read32(kCode) == (0x396B0000u | ((100u * 3u) & 0x7FFFu)));
}

TEST_CASE("direct memory: the cache instructions, and what is not main memory")
{
    // The bus offers its first 8 MB. A block below that is memory itself; a
    // block above it is something else mapped cacheable (a device, as far as
    // the processor knows) and keeps to the model: conjured by dcbz, filled
    // by stores, and on the bus only when dcbf pushes it.
    constexpr u32 kDirect = 8u << 20;
    constexpr u32 kLow = 0x00090000u, kHigh = 0x00900000u;
    const std::vector<u32> prog = {
        dform(36, 8, 5, 0),       // stw r8, 0(r5)      low: memory
        x31(0, 0, 6, 1014),       // dcbz 0, r6         high: conjured
        dform(36, 8, 6, 4),       // stw r8, 4(r6)
        dform(36, 8, 6, 28),      // stw r8, 28(r6)
        x31(0, 0, 0, 0) | 24u << 26, // ori r0, r0, 0 (nop): the check point
        x31(0, 0, 6, 86),         // dcbf 0, r6
        x31(0, 0, 5, 1014),       // dcbz 0, r5         low: zeros at once
        x31(0, 0, 5, 470),        // dcbi 0, r5
        18u << 26,                // b .
    };
    Machine m(true);
    REQUIRE(m.setUp(prog));
    m.normalCaches(true, kDirect);
    m.cpu.st.msr = msr::FP; // real mode: every address is cacheable
    m.cpu.st.gpr[5] = kLow;
    m.cpu.st.gpr[6] = kHigh;
    m.cpu.st.gpr[8] = 0xCAFEF00Du;
    for (u32 k = 0; k < 64; ++k) {
        m.bus.write8(kLow + k, 0x11);
        m.bus.write8(kHigh + k, 0x22);
    }
    m.run(1); // a batch: the mode is decided
    REQUIRE(m.cpu.dmDirect);
    CHECK(m.bus.read32(kLow) == 0xCAFEF00Du); // in memory with the store
    for (u32 k = 0; k < 4; ++k)
        m.cpu.step(); // the instrumented path reads memory in place too
    // The high block is in the cache, modified, and the bus has seen none
    // of it.
    CHECK(m.bus.read32(kHigh) == 0x22222222u);
    CHECK(m.bus.read32(kHigh + 4) == 0x22222222u);
    CHECK(cachedBlocks(m.cpu, kDirect) == 0u);
    m.run(1); // dcbf
    CHECK(m.bus.read32(kHigh) == 0u);
    CHECK(m.bus.read32(kHigh + 4) == 0xCAFEF00Du);
    CHECK(m.bus.read32(kHigh + 28) == 0xCAFEF00Du);
    CHECK(m.bus.read32(kHigh + 8) == 0u);
    m.run(1); // dcbz on memory
    CHECK(m.bus.read32(kLow) == 0u);
    CHECK(m.bus.read32(kLow + 28) == 0u);
    CHECK(m.bus.read8(kLow + 32) == 0x11); // the next block is untouched
    CHECK(m.cpu.directDcbi == 0u);
    m.run(1); // dcbi on memory: counted, and nothing is lost
    CHECK(m.cpu.directDcbi == 1u);
    CHECK(m.cpu.excRingAt == 0u);
}

TEST_CASE("direct memory: the guest's cache controls decide the mode")
{
    // mtspr L2CR and HID0 are where the mode changes hands: a test mode or a
    // disabled data cache gets the full model on that instruction, and
    // normal operation gets memory read directly again.
    const std::vector<u32> prog = {
        dform(36, 8, 5, 0),  // stw r8, 0(r5)
        mtspr(1017, 14),     // L2CR <- enabled + test support
        dform(36, 9, 5, 4),  // stw r9, 4(r5)
        mtspr(1017, 15),     // L2CR <- enabled
        dform(36, 8, 5, 8),  // stw r8, 8(r5)
        mtspr(1008, 16),     // HID0 <- data cache off
        dform(36, 9, 5, 12), // stw r9, 12(r5)
        mtspr(1008, 17),     // HID0 <- data cache on
        dform(32, 7, 5, 4),  // lwz r7, 4(r5)
        18u << 26,           // b .
    };
    Machine m(true);
    REQUIRE(m.setUp(prog));
    m.normalCaches(true);
    m.cpu.st.gpr[5] = kVirt;
    m.cpu.st.gpr[8] = 0x11112222u;
    m.cpu.st.gpr[9] = 0x33334444u;
    m.cpu.st.gpr[14] = 0x80040000u;
    m.cpu.st.gpr[15] = 0x80000000u;
    m.cpu.st.gpr[16] = 0x00008000u;
    m.cpu.st.gpr[17] = 0x0000C000u;
    const auto stepTo = [&](u64 n) {
        while (m.stamp < n)
            m.run(n - m.stamp);
    };
    stepTo(1);
    CHECK(m.cpu.dmDirect);
    CHECK(m.bus.read32(kPhys) == 0x11112222u);
    stepTo(2); // the test mode
    CHECK(!m.cpu.dmDirect);
    stepTo(3); // this store sits in the L1, as the model has it
    CHECK(m.bus.read32(kPhys + 4) != 0x33334444u);
    CHECK(cachedBlocks(m.cpu, kRam) > 0u);
    stepTo(4); // normal again: written back on the way in
    CHECK(m.cpu.dmDirect);
    CHECK(m.bus.read32(kPhys + 4) == 0x33334444u);
    CHECK(cachedBlocks(m.cpu, kRam) == 0u);
    stepTo(6); // data cache off
    CHECK(!m.cpu.dmDirect);
    stepTo(8);
    CHECK(m.cpu.dmDirect);
    stepTo(9);
    CHECK(m.cpu.st.gpr[7] == 0x33334444u);
    CHECK(m.cpu.directEnters == 3u);
    CHECK(m.cpu.directLeaves == 2u);
    CHECK(m.cpu.directPushed >= 1u);
    CHECK(m.cpu.excRingAt == 0u);
    CHECK(m.cpu.directReport().find("memory read directly") != std::string::npos);
}

// ---- the fetch buffer -----------------------------------------------------------

TEST_CASE("fetch buffer: icbi drops the blocks at its address's offset, and no others")
{
    // icbi names an effective address and the buffer keeps blocks by their
    // physical one; what the two share is the offset in the page. So icbi
    // drops every block at that offset, whatever page it is in, and leaves
    // the rest of the buffer alone (it used to empty all of it).
    std::vector<u32> prog(8, 0x60000000u);   // 0x1000: a block of nops
    prog.push_back(x31(0, 0, 5, 982));       // 0x1020: icbi 0, r5
    prog.push_back(18u << 26);               // b .
    Machine m(true);
    REQUIRE(m.setUp(prog));
    m.cpu.st.gpr[5] = 0x00777000u + 8u; // another page, the first block's offset
    const u32 first = Cpu::fetchSlot(kCode), second = Cpu::fetchSlot(kCode + 32u);
    m.run(8);
    REQUIRE(m.cpu.fetchBase[first] == kCode);
    while (m.stamp < 9)
        m.run(9 - m.stamp);
    CHECK(m.cpu.fetchBase[first] == 1u);            // dropped: same offset
    CHECK(m.cpu.fetchBase[second] == kCode + 32u);  // kept: another offset
    CHECK(m.cpu.excRingAt == 0u);
    // ...and the dropped block fills again and runs as before.
    m.cpu.st.pc = kCode;
    m.run(8);
    CHECK(m.cpu.fetchBase[first] == kCode);
}
