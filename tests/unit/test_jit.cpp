// JIT differential tests: the compiled blocks against the interpreter, on
// identical twin machines. The JIT's whole correctness claim is "a native
// transcription of the line executor" (opm/jit.hpp), and a claim like that
// is settled the way the line executor's own was: run the same instructions
// both ways and demand the same machine, state item for state item.
//
// The twins share nothing. Each has its own flat-RAM bus, so stores diverge
// the moment an emitted sequence is wrong, and the comparison covers every
// GPR, CR, XER, LR, CTR, PC, TB, DEC and the RAM images.

#include "doctest.h"
#include "opm/cpu.hpp"

#include <cstring>
#include <vector>

using namespace opm;

namespace {

// Flat big-endian RAM, memory everywhere, so the fetch path may fill whole
// lines and the JIT engages exactly as it does on the real machine.
class FlatBus final : public Bus {
public:
    std::vector<u8> m;
    explicit FlatBus(size_t n) : m(n, 0) {}

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

// ---- tiny assembler --------------------------------------------------------

u32 dform(u32 op, u32 rt, u32 ra, u32 d)
{
    return op << 26 | rt << 21 | ra << 16 | (d & 0xFFFFu);
}
u32 x31(u32 rt, u32 ra, u32 rb, u32 xo, bool rc = false)
{
    return 31u << 26 | rt << 21 | ra << 16 | rb << 11 | xo << 1 | (rc ? 1 : 0);
}
u32 xo31(u32 rt, u32 ra, u32 rb, u32 xo9, bool oe, bool rc)
{
    return 31u << 26 | rt << 21 | ra << 16 | rb << 11 | (oe ? 1u << 10 : 0) |
           xo9 << 1 | (rc ? 1 : 0);
}
u32 rlwinm(u32 rs, u32 ra, u32 sh, u32 mb, u32 me, bool rc = false)
{
    return 21u << 26 | rs << 21 | ra << 16 | sh << 11 | mb << 6 | me << 1 |
           (rc ? 1 : 0);
}
u32 rlwimi(u32 rs, u32 ra, u32 sh, u32 mb, u32 me, bool rc = false)
{
    return 20u << 26 | rs << 21 | ra << 16 | sh << 11 | mb << 6 | me << 1 |
           (rc ? 1 : 0);
}
u32 cmpi(u32 crf, u32 ra, u32 si) { return dform(11, crf << 2, ra, si); }
u32 cmpli(u32 crf, u32 ra, u32 ui) { return dform(10, crf << 2, ra, ui); }
u32 b(i32 off, bool lk = false, bool aa = false)
{
    return 18u << 26 | (u32(off) & 0x03FFFFFCu) | (aa ? 2 : 0) | (lk ? 1 : 0);
}
u32 bc(u32 bo, u32 bi, i32 off, bool lk = false)
{
    return 16u << 26 | bo << 21 | bi << 16 | (u32(off) & 0xFFFCu) |
           (lk ? 1 : 0);
}
u32 bclr(u32 bo, u32 bi, bool lk = false)
{
    return 19u << 26 | bo << 21 | bi << 16 | 16u << 1 | (lk ? 1 : 0);
}
u32 bcctr(u32 bo, u32 bi, bool lk = false)
{
    return 19u << 26 | bo << 21 | bi << 16 | 528u << 1 | (lk ? 1 : 0);
}
u32 sprSplit(u32 n) { return (n & 31u) << 16 | (n >> 5) << 11; }
u32 mtspr(u32 spr, u32 rs) { return 31u << 26 | rs << 21 | sprSplit(spr) | 467u << 1; }
u32 mfspr(u32 spr, u32 rt) { return 31u << 26 | rt << 21 | sprSplit(spr) | 339u << 1; }

constexpr u32 kBase = 0x1000; // program origin
constexpr u32 kData = 0x4000; // data region — positive as a 16-bit simm,
                              // so `addi rN, 0, kData` builds the base

struct Twin {
    FlatBus bus{1u << 20};
    // Heap, not stack: a Cpu is a few hundred KB of cache arrays, and one
    // TEST_CASE function frame can hold several Twin pairs' scopes at once —
    // MSVC does not overlap large locals from sibling scopes, and two pairs
    // blew the default 1 MB thread stack.
    std::unique_ptr<Cpu> cpuUp = std::make_unique<Cpu>();
    Cpu& cpu = *cpuUp;
    u64 stamp = 0;

    explicit Twin(bool jitOn)
    {
        cpu.attach(bus);
        cpu.jitOn = jitOn;
        cpu.st.pc = kBase;
    }
    void poke(const std::vector<u32>& words)
    {
        for (size_t k = 0; k < words.size(); ++k)
            bus.write32(kBase + u32(k) * 4, words[k]);
    }
    void run(u64 n) { cpu.runSteps(n, stamp); }
};

struct Rng { // deterministic; no wall clock anywhere near a test
    u64 s = 0x9E3779B97F4A7C15ull;
    u32 next()
    {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        return u32(s >> 32);
    }
};

void seedRegs(Cpu& c, Rng& r)
{
    static const u32 edges[] = {0u, 1u, 0xFFFFFFFFu, 0x7FFFFFFFu,
                                0x80000000u, 0x5555AAAAu, 0x0000FFFFu,
                                0xFFFF0000u};
    for (u32 k = 0; k < 32; ++k)
        c.st.gpr[k] = (r.next() & 1u) ? edges[r.next() & 7u] : r.next();
    c.st.cr = r.next();
    c.st.xer = (r.next() & 0xE0000000u); // SO/OV/CA in their real positions
    c.st.lr = r.next();
    c.st.ctr = r.next();
}

#define CHECK_TWINS(a, b)                                                     \
    do {                                                                      \
        for (u32 k_ = 0; k_ < 32; ++k_)                                       \
            CHECK((a).cpu.st.gpr[k_] == (b).cpu.st.gpr[k_]);                  \
        CHECK((a).cpu.st.cr == (b).cpu.st.cr);                                \
        CHECK((a).cpu.st.xer == (b).cpu.st.xer);                              \
        CHECK((a).cpu.st.lr == (b).cpu.st.lr);                                \
        CHECK((a).cpu.st.ctr == (b).cpu.st.ctr);                              \
        CHECK((a).cpu.st.pc == (b).cpu.st.pc);                                \
        CHECK((a).cpu.st.tb == (b).cpu.st.tb);                                \
        CHECK((a).cpu.st.dec == (b).cpu.st.dec);                              \
        CHECK((a).stamp == (b).stamp);                                        \
        CHECK((a).cpu.halted == (b).cpu.halted);                              \
    } while (0)

// Run one program on both machines with one register seed; RAM compared too.
void differential(const std::vector<u32>& prog, u64 steps, Rng seed,
                  bool checkRam = true)
{
    Twin ji(true), in(false);
    Rng r1 = seed, r2 = seed;
    seedRegs(ji.cpu, r1);
    seedRegs(in.cpu, r2);
    ji.poke(prog);
    in.poke(prog);
    // A data pattern both sides share, for the load/store programs.
    for (u32 k = 0; k < 0x400; ++k) {
        ji.bus.write8(kData + k, u8(k * 7u + 3u));
        in.bus.write8(kData + k, u8(k * 7u + 3u));
    }
    ji.run(steps);
    in.run(steps);
    CHECK_TWINS(ji, in);
    if (checkRam)
        CHECK(std::memcmp(ji.bus.m.data(), in.bus.m.data(),
                          ji.bus.m.size()) == 0);
}

} // namespace

TEST_CASE("jit: randomized ALU differential (record, carry, edges)")
{
    // Encodable shapes; operands randomized per program. Every op here is in
    // the lowering set, and the trailing spin keeps the budget deterministic.
    for (u32 iter = 0; iter < 400; ++iter) {
        Rng r{0xC0FFEE00ull + iter};
        std::vector<u32> prog;
        for (u32 k = 0; k < 6; ++k) {
            const u32 rt = r.next() % 8u + 3u; // r3..r10: away from bases
            const u32 ra = r.next() % 8u + 3u;
            const u32 rb = r.next() % 8u + 3u;
            const bool rc = (r.next() & 1u) != 0;
            switch (r.next() % 24u) {
            case 0: prog.push_back(dform(14, rt, (r.next() & 1) ? ra : 0, r.next())); break; // addi
            case 1: prog.push_back(dform(15, rt, (r.next() & 1) ? ra : 0, r.next())); break; // addis
            case 2: prog.push_back(dform(12, rt, ra, r.next())); break;  // addic
            case 3: prog.push_back(dform(13, rt, ra, r.next())); break;  // addic.
            case 4: prog.push_back(dform(8, rt, ra, r.next())); break;   // subfic
            case 5: prog.push_back(dform(7, rt, ra, r.next())); break;   // mulli
            case 6: prog.push_back(dform(24 + (r.next() % 4u), ra, rt, r.next())); break; // ori/oris/xori/xoris
            case 7: prog.push_back(dform(28 + (r.next() & 1u), ra, rt, r.next())); break; // andi./andis.
            case 8: prog.push_back(xo31(rt, ra, rb, 266, false, rc)); break; // add
            case 9: prog.push_back(xo31(rt, ra, rb, 40, false, rc)); break;  // subf
            case 10: prog.push_back(xo31(rt, ra, rb, 10, false, rc)); break; // addc
            case 11: prog.push_back(xo31(rt, ra, rb, 138, false, rc)); break; // adde
            case 12: prog.push_back(xo31(rt, ra, 0, 202, false, rc)); break; // addze
            case 13: prog.push_back(xo31(rt, ra, rb, 8, false, rc)); break;  // subfc
            case 14: prog.push_back(xo31(rt, ra, rb, 136, false, rc)); break; // subfe
            case 15: prog.push_back(xo31(rt, ra, 0, 104, false, rc)); break; // neg
            case 16: prog.push_back(x31(rt, ra, rb, 28, rc)); break;  // and
            case 17: prog.push_back(x31(rt, ra, rb, 444, rc)); break; // or
            case 18: prog.push_back(x31(rt, ra, rb, 316, rc)); break; // xor
            case 19: prog.push_back(rlwinm(rt, ra, r.next() & 31u,
                                           r.next() & 31u, r.next() & 31u,
                                           rc)); break;
            case 20: prog.push_back(x31(rt, ra, rb, 24, rc)); break;  // slw
            case 21: prog.push_back(x31(rt, ra, r.next() & 31u, 824, rc)); break; // srawi
            case 22: prog.push_back(xo31(rt, ra, rb, 235, false, rc)); break; // mullw
            case 23: prog.push_back(cmpi(r.next() & 7u, ra, r.next())); break;
            }
        }
        prog.push_back(b(0)); // spin: b .
        differential(prog, 24, Rng{0xABCD0000ull + iter}, false);
    }
}

TEST_CASE("jit: more ALU shapes (logic complements, shifts, extends, mulh)")
{
    for (u32 iter = 0; iter < 200; ++iter) {
        Rng r{0xFACE0000ull + iter};
        std::vector<u32> prog;
        for (u32 k = 0; k < 6; ++k) {
            const u32 rt = r.next() % 8u + 3u;
            const u32 ra = r.next() % 8u + 3u;
            const u32 rb = r.next() % 8u + 3u;
            const bool rc = (r.next() & 1u) != 0;
            switch (r.next() % 14u) {
            case 0: prog.push_back(x31(rt, ra, rb, 60, rc)); break;  // andc
            case 1: prog.push_back(x31(rt, ra, rb, 412, rc)); break; // orc
            case 2: prog.push_back(x31(rt, ra, rb, 284, rc)); break; // eqv
            case 3: prog.push_back(x31(rt, ra, rb, 124, rc)); break; // nor
            case 4: prog.push_back(x31(rt, ra, rb, 476, rc)); break; // nand
            case 5: prog.push_back(x31(rt, ra, rb, 536, rc)); break; // srw
            case 6: prog.push_back(x31(rt, ra, 0, 26, rc)); break;   // cntlzw
            case 7: prog.push_back(x31(rt, ra, 0, 954, rc)); break;  // extsb
            case 8: prog.push_back(x31(rt, ra, 0, 922, rc)); break;  // extsh
            case 9: prog.push_back(xo31(rt, ra, rb, 75, false, rc)); break;  // mulhw
            case 10: prog.push_back(xo31(rt, ra, rb, 11, false, rc)); break; // mulhwu
            case 11: prog.push_back(xo31(rt, ra, 0, 234, false, rc)); break; // addme
            case 12: prog.push_back(xo31(rt, ra, 0, 232, false, rc)); break; // subfze
            case 13: prog.push_back(cmpli(r.next() & 7u, ra, r.next())); break;
            }
        }
        prog.push_back(b(0));
        differential(prog, 24, Rng{0xBEEF0000ull + iter}, false);
    }
}

TEST_CASE("jit: rlwimi/rlwnm and cmp/cmpl register forms")
{
    for (u32 iter = 0; iter < 100; ++iter) {
        Rng r{0xD00D0000ull + iter};
        std::vector<u32> prog;
        for (u32 k = 0; k < 6; ++k) {
            const u32 rt = r.next() % 8u + 3u;
            const u32 ra = r.next() % 8u + 3u;
            const u32 rb = r.next() % 8u + 3u;
            const bool rc = (r.next() & 1u) != 0;
            switch (r.next() % 4u) {
            case 0: prog.push_back(rlwimi(rt, ra, r.next() & 31u,
                                          r.next() & 31u, r.next() & 31u,
                                          rc)); break;
            case 1: prog.push_back(23u << 26 | rt << 21 | ra << 16 |
                                   rb << 11 | (r.next() & 31u) << 6 |
                                   (r.next() & 31u) << 1 | (rc ? 1 : 0));
                    break; // rlwnm
            case 2: prog.push_back(x31(r.next() & 7u ? (r.next() & 7u) << 2 : 0,
                                       ra, rb, 0)); break; // cmp
            case 3: prog.push_back(x31((r.next() & 7u) << 2, ra, rb, 32));
                    break; // cmpl
            }
        }
        prog.push_back(b(0));
        differential(prog, 24, Rng{0xF00D0000ull + iter}, false);
    }
}

TEST_CASE("jit: branch matrix — bc forms, bclr, bcctr, link bits")
{
    // bdnz loop: r3 counts 7 iterations, then falls through to the spin.
    {
        std::vector<u32> prog = {
            dform(14, 3, 0, 0),      // addi r3, 0, 0
            dform(14, 4, 0, 7),      // addi r4, 0, 7
            mtspr(9, 4),             // mtctr r4
            dform(14, 3, 3, 1),      // L: addi r3, r3, 1
            bc(16, 0, -4),           // bdnz L
            b(0),                    // spin
        };
        Twin ji(true), in(false);
        ji.poke(prog);
        in.poke(prog);
        ji.run(64);
        in.run(64);
        CHECK_TWINS(ji, in);
        CHECK(ji.cpu.st.gpr[3] == 7u);
    }
    // Condition forms across taken/not-taken, both bi ends, with LK.
    for (u32 iter = 0; iter < 120; ++iter) {
        Rng r{0xBADC0000ull + iter};
        static const u32 bos[] = {0u, 2u, 4u, 8u, 10u, 12u, 16u, 18u, 20u};
        const u32 bo = bos[r.next() % 9u];
        const u32 bi = r.next() % 32u;
        const bool lk = (r.next() & 1u) != 0;
        std::vector<u32> prog = {
            dform(14, 3, 0, 1),           // addi r3, 0, 1
            bc(bo, bi, 8, lk),            // maybe skip the next insn
            dform(14, 3, 3, 100),         // addi r3, r3, 100
            dform(14, 5, 0, 9),           // addi r5, 0, 9
            b(0),
        };
        differential(prog, 24, Rng{0x11110000ull + iter}, false);
    }
    // bclr / bcctr with a controlled target: lr/ctr point at the spin.
    for (u32 iter = 0; iter < 60; ++iter) {
        Rng r{0x22220000ull + iter};
        static const u32 bos[] = {0u, 4u, 8u, 12u, 16u, 20u};
        const u32 bo = bos[r.next() % 6u];
        const u32 bi = r.next() % 32u;
        const bool lk = (r.next() & 1u) != 0;
        const bool viaCtr = (r.next() & 1u) != 0;
        std::vector<u32> prog = {
            dform(14, 4, 0, i32(kBase + 24)), // addi r4, 0, spin-address
            viaCtr ? mtspr(9, 4) : mtspr(8, 4),
            viaCtr ? bcctr(bo & 20u ? bo : (bo | 16u), bi, lk) // no ctr-dec forms
                   : bclr(bo, bi, lk),
            dform(14, 3, 0, 55),          // not-taken path
            dform(14, 6, 0, 66),
            dform(14, 7, 0, 77),
            b(0),                          // spin @ kBase+24
        };
        differential(prog, 24, Rng{0x33330000ull + iter}, false);
    }
    // bl: the link register and the return path (getpc idiom included).
    {
        std::vector<u32> prog = {
            b(16, true),             // bl +16 -> the leaf
            dform(14, 3, 0, 2),      // return lands here
            b(12),                   // jump to spin
            dform(14, 9, 0, 9),      // (skipped)
            dform(14, 4, 0, 4),      // leaf: addi r4, 0, 4
            bclr(20, 0),             // blr
            b(0),                    // spin
        };
        differential(prog, 32, Rng{0x4444ull}, false);
    }
}

TEST_CASE("jit: loads and stores — forms, updates, byte-reverse, aliasing")
{
    for (u32 iter = 0; iter < 200; ++iter) {
        Rng r{0x51500000ull + iter};
        std::vector<u32> prog;
        // r20 = data base; r21 = small index; the ops stay inside the
        // pattern region so both RAM images see identical traffic.
        prog.push_back(dform(14, 20, 0, i32(kData)));
        prog.push_back(dform(14, 21, 0, i32(r.next() & 0xFCu)));
        for (u32 k = 0; k < 5; ++k) {
            const u32 rt = r.next() % 8u + 3u;
            const u32 d = r.next() % 0x300u;
            switch (r.next() % 16u) {
            case 0: prog.push_back(dform(32, rt, 20, d)); break;      // lwz
            case 1: prog.push_back(dform(34, rt, 20, d)); break;      // lbz
            case 2: prog.push_back(dform(40, rt, 20, d)); break;      // lhz
            case 3: prog.push_back(dform(42, rt, 20, d)); break;      // lha
            case 4: prog.push_back(dform(36, rt, 20, d)); break;      // stw
            case 5: prog.push_back(dform(38, rt, 20, d)); break;      // stb
            case 6: prog.push_back(dform(44, rt, 20, d)); break;      // sth
            case 7: prog.push_back(x31(rt, 20, 21, 23)); break;       // lwzx
            case 8: prog.push_back(x31(rt, 20, 21, 151)); break;      // stwx
            case 9: prog.push_back(x31(rt, 20, 21, 534)); break;      // lwbrx
            case 10: prog.push_back(x31(rt, 20, 21, 662)); break;     // stwbrx
            case 11: prog.push_back(x31(rt, 20, 21, 790)); break;     // lhbrx
            case 12: prog.push_back(x31(rt, 20, 21, 918)); break;     // sthbrx
            case 13: prog.push_back(dform(33, rt, 22, 4)); break;     // lwzu
            case 14: prog.push_back(dform(37, rt, 23, 4)); break;     // stwu
            case 15: prog.push_back(x31(rt, 20, 21, 375)); break;     // lhaux? no: 375=lhaux
            }
        }
        // Update-form bases point into the pattern too.
        std::vector<u32> pre = {dform(14, 22, 0, i32(kData + 0x40)),
                                dform(14, 23, 0, i32(kData + 0x80))};
        prog.insert(prog.begin() + 2, pre.begin(), pre.end());
        prog.push_back(b(0));
        differential(prog, 32, Rng{0x77770000ull + iter});
    }
}

TEST_CASE("jit: store into the executing line — the SMC contract")
{
    // The line at kBase holds 8 words; word 4 starts as `addi r6, r6, 1`.
    // Word 2 stores a different instruction over word 4 BEFORE it executes.
    // Both machines must run the NEW word: the interpreter's line executor
    // drops the block on the store; the JIT's per-store residency check must
    // exit the block the same way.
    const u32 newInsn = dform(14, 6, 6, 2); // addi r6, r6, 2
    std::vector<u32> prog = {
        dform(14, 10, 0, i32(kBase + 16)),  // r10 = &word4
        dform(15, 11, 0, i32(newInsn >> 16)),          // lis r11, hi
        dform(24, 11, 11, i32(newInsn & 0xFFFFu)),     // ori r11, r11, lo
        dform(36, 11, 10, 0),               // stw r11, 0(r10)
        dform(14, 6, 6, 1),                 // word 4: the doomed instruction
        dform(14, 7, 7, 1),
        dform(14, 8, 8, 1),
        b(0),
    };
    Twin ji(true), in(false);
    ji.poke(prog);
    in.poke(prog);
    ji.cpu.st.gpr[6] = in.cpu.st.gpr[6] = 0;
    ji.cpu.st.gpr[7] = in.cpu.st.gpr[7] = 0;
    ji.run(16);
    in.run(16);
    CHECK_TWINS(ji, in);
    CHECK(ji.cpu.st.gpr[6] == 2u); // the stored instruction ran, not the old
}

TEST_CASE("jit: fallback interleave mid-line keeps the tick identical")
{
    // divwu and mfspr XER are not in the lowering set; they must run through
    // execRow inside the block with the same charge/count as everything else.
    for (u32 iter = 0; iter < 60; ++iter) {
        Rng r{0x99990000ull + iter};
        std::vector<u32> prog = {
            dform(14, 3, 0, i32(r.next() & 0x7FFF)),
            xo31(4, 3, 5, 459, false, (r.next() & 1) != 0), // divwu
            dform(14, 6, 3, 12),
            mfspr(1, 7),                                    // mfspr r7, XER
            xo31(8, 6, 3, 266, false, true),                // add.
            mtspr(8, 8),                                    // mtlr r8
            mfspr(8, 9),                                    // mflr r9
            b(0),
        };
        differential(prog, 24, Rng{0xAAAA0000ull + iter}, false);
    }
}

TEST_CASE("jit: budget exactness — every sub-line stop lands on the count")
{
    std::vector<u32> prog;
    for (u32 k = 0; k < 16; ++k)
        prog.push_back(dform(14, 3, 3, 1)); // addi r3, r3, 1  x16
    prog.push_back(b(0));
    for (u64 n = 1; n <= 17; ++n) {
        Twin ji(true), in(false);
        ji.poke(prog);
        in.poke(prog);
        ji.cpu.st.gpr[3] = in.cpu.st.gpr[3] = 0;
        ji.run(n);
        in.run(n);
        CHECK_TWINS(ji, in);
        CHECK(ji.stamp == n);
    }
}

TEST_CASE("jit: instrument-style flush keeps compiled lines (refill parity)")
{
    std::vector<u32> prog = {
        dform(14, 3, 3, 1), dform(14, 4, 4, 2), dform(14, 5, 5, 3),
        dform(14, 6, 6, 4), dform(14, 7, 7, 5), dform(14, 8, 8, 6),
        dform(14, 9, 9, 7), b(-28),
    };
    Twin ji(true), in(false);
    ji.poke(prog);
    in.poke(prog);
    ji.run(64);
    in.run(64);
    CHECK_TWINS(ji, in);
    // The flush an instrument performs: caches written back, fetch lines
    // dropped. The refill must KEEP the identical compiled line...
    ji.cpu.l1dFlushAll(true);
    in.cpu.l1dFlushAll(true);
    ji.cpu.fetchDrop();
    in.cpu.fetchDrop();
    ji.run(64);
    in.run(64);
    CHECK_TWINS(ji, in);
    // The cache and its counters only exist where the JIT does (the emitter
    // targets the Win64 ABI, so other hosts interpret and never create it).
    // The behavioural half above — flush, refill, identical machines — holds
    // on every platform; the keep-counter claim is host-specific.
    if (ji.cpu.jit) {
        CHECK(ji.cpu.jit->refillKeeps > 0);
        CHECK(ji.cpu.jit->insns > 0);
    }
}

// ---- Stage B: block-to-block chaining (JIT_PLAN §7) ------------------------

TEST_CASE("jit chain: cross-line branches and fallthrough match the interpreter")
{
    // Line 0 counts to 20 via a taken forward bc into line 1, line 1 loops
    // back with a backward b — both cross-line, same-page, so both exits
    // chain. The spin at the end is an intra-line self-loop. Chains link on
    // the second traversal and carry every iteration after; the twins must
    // not be able to tell.
    std::vector<u32> prog = {
        dform(14, 3, 3, 1),      // w0: addi r3, r3, 1
        cmpi(0, 3, 20),          // w1: cmp r3, 20
        bc(12, 0, 24),           // w2: blt -> line 1 word 0 (kBase+32)
        dform(14, 4, 4, 1),      // w3: r3 == 20: fall out of the loop
        dform(14, 5, 5, 1),      // w4
        b(0),                    // w5: spin (intra-line self-chain)
        dform(14, 6, 6, 1),      // w6: (dead)
        dform(14, 6, 6, 1),      // w7: (dead)
        dform(14, 7, 7, 1),      // line 1 w0 (kBase+32)
        b(-36),                  // line 1 w1: back to line 0 word 0
    };
    Twin ji(true), in(false);
    ji.poke(prog);
    in.poke(prog);
    ji.run(160);
    in.run(160);
    CHECK_TWINS(ji, in);
    CHECK(ji.cpu.st.gpr[3] == 20u);
    CHECK(ji.cpu.st.gpr[7] == 19u);
    if (ji.cpu.jit) { // host-specific half: the chains actually engaged
        CHECK(ji.cpu.jitChainHops > 0);
        CHECK(ji.cpu.jit->chainLinks > 0);
    }
}

TEST_CASE("jit chain: intra-line backward loop keeps the budget exact")
{
    // bdnz to its own line: the cheap chain flavor, and the one that could
    // overrun a batch if a hop ever skipped the budget check. Every stop
    // count from 1 to 100 must land exactly, twins identical.
    std::vector<u32> prog = {
        dform(14, 9, 0, 40),     // w0: r9 = 40
        mtspr(9, 9),             // w1: mtctr r9
        dform(14, 3, 3, 1),      // w2: addi r3, r3, 1
        bc(16, 0, -4),           // w3: bdnz w2 (backward, same line)
        b(0),                    // w4: spin
        dform(14, 6, 6, 1),      // w5..w7: (dead)
        dform(14, 6, 6, 1),
        dform(14, 6, 6, 1),
    };
    for (u64 n : {1ull, 7ull, 8ull, 9ull, 15ull, 16ull, 17ull, 33ull, 50ull,
                  81ull, 82ull, 83ull, 100ull}) {
        Twin ji(true), in(false);
        ji.poke(prog);
        in.poke(prog);
        ji.run(n);
        in.run(n);
        CHECK_TWINS(ji, in);
        CHECK(ji.stamp == n);
    }
}

TEST_CASE("jit chain: SMC in the successor severs and relinks the chain")
{
    // Line 0 stores a DIFFERENT instruction into line 1 word 0 on every
    // iteration (r11 increments first), so every refill of line 1 mismatches:
    // the block drops, any chain into it severs, the next traversal
    // re-resolves against the recompiled block. The interpreter must see the
    // same stream of freshly-stored instructions — addi r7, r7, k on
    // iteration k — and the accumulators prove which stream actually ran.
    const u32 insn0 = dform(14, 7, 7, 0); // addi r7, r7, 0 (stored as +1, +2…)
    std::vector<u32> prog = {
        dform(14, 10, 0, i32(kBase + 32)),         // w0: r10 = &line1
        dform(15, 11, 0, i32(insn0 >> 16)),        // w1: lis r11, hi
        dform(24, 11, 11, i32(insn0 & 0xFFFFu)),   // w2: ori r11, r11, lo
        dform(14, 11, 11, 1),                      // w3: r11 += 1 (each pass)
        dform(36, 11, 10, 0),                      // w4: stw r11, 0(r10)
        dform(14, 3, 3, 1),                        // w5
        dform(14, 4, 4, 1),                        // w6
        dform(14, 5, 5, 1),                        // w7 -> falls into line 1
        dform(14, 7, 7, 1),                        // line1 w0: overwritten
        b(-36),                                    // line1 w1: back to w0
    };
    // Loop back to w3, not w0: w1/w2 would rebuild r11 from scratch and the
    // stored word would stop changing after the first pass — one sever
    // instead of a stream of them.
    prog[9] = b(-24); // line1 w1: back to w3 (skip the r10/r11 seeding)
    Twin ji(true), in(false);
    ji.poke(prog);
    in.poke(prog);
    ji.run(140); // ~19 iterations of the 7-insn loop after the 3-insn seed
    in.run(140);
    CHECK_TWINS(ji, in);
    CHECK(std::memcmp(ji.bus.m.data(), in.bus.m.data(), ji.bus.m.size()) == 0);
    if (ji.cpu.jit) {
        // No hop ever completes here, BY CONSTRUCTION: line 1 recompiles
        // every iteration (its bytes changed), so each incarnation's b(-24)
        // site links on its first-and-only traversal and the block dies
        // before the link is taken; the fallthrough site never even resolves
        // (its target's line is always freshly dropped at the check). What
        // the storm proves is the sever/relink lifecycle under maximum churn
        // — links keep forming and dying with the twins bit-identical.
        CHECK(ji.cpu.jit->refillDrops > 0); // the stored word really changed
        CHECK(ji.cpu.jit->chainLinks > 0);  // each incarnation re-linked
        CHECK(ji.cpu.jit->chainResolves > 0);
    }
}

TEST_CASE("jit chain: severing a LIVE link — modify a successor after it linked")
{
    // Nine quiet iterations first, so the fallthrough into line 1 and the
    // backward b out of it both LINK. On iteration 10 the guest overwrites
    // line 1's first word: the refill must sever the live link into line 1,
    // the old block must never be entered again, and the site must re-resolve
    // against the recompiled block. r7 counts +1 for nine passes and +2 from
    // then on — only the correct stream of blocks produces it.
    const u32 newInsn = dform(14, 7, 7, 2); // addi r7, r7, 2
    std::vector<u32> prog = {
        dform(14, 3, 3, 1),      // w0: addi r3, r3, 1 (iteration counter)
        cmpi(0, 3, 10),          // w1
        bc(4, 2, 8),             // w2: bne +8 -> w4 (intra-line forward chain)
        dform(36, 11, 10, 0),    // w3: stw r11, 0(r10) — iteration 10 only
        dform(14, 4, 4, 1),      // w4
        dform(14, 5, 5, 1),      // w5
        dform(14, 5, 5, 1),      // w6
        dform(14, 5, 5, 1),      // w7 -> falls into line 1
        dform(14, 7, 7, 1),      // line1 w0: becomes addi r7, r7, 2
        b(-36),                  // line1 w1: back to w0
    };
    Twin ji(true), in(false);
    ji.poke(prog);
    in.poke(prog);
    ji.cpu.st.gpr[10] = in.cpu.st.gpr[10] = kBase + 32; // &line1
    ji.cpu.st.gpr[11] = in.cpu.st.gpr[11] = newInsn;
    ji.run(200); // ~20 iterations of the 9/10-insn loop
    in.run(200);
    CHECK_TWINS(ji, in);
    CHECK(std::memcmp(ji.bus.m.data(), in.bus.m.data(), ji.bus.m.size()) == 0);
    if (ji.cpu.jit) {
        CHECK(ji.cpu.jitChainHops > 0);
        CHECK(ji.cpu.jit->chainLinks >= 2);  // linked, severed, relinked
        CHECK(ji.cpu.jit->chainSevers >= 1); // the live link was cut
        CHECK(ji.cpu.jit->refillDrops >= 1);
    }
}

TEST_CASE("jit chain: msr changes stop a chained run where the dispatcher would")
{
    // mtmsr does NOT break the batch (only its nap path does), so the
    // dispatcher only notices a changed MSR at its next fetchBlockFast — and
    // a chained hop must notice at the same boundary, through the same
    // comparison (msr fetch bits vs xlMsr; EE against the pending lines).
    // Case A: EE goes live with an external line already asserted — the hop
    // must hand the line back so the interrupt delivers on the dispatcher's
    // schedule, not a line late.
    {
        std::vector<u32> prog = {
            dform(14, 3, 3, 1),         // w0
            dform(14, 9, 0, 0),         // w1: r9 = 0
            dform(24, 9, 9, 0x8000),    // w2: ori r9, r9, EE
            31u << 26 | 9u << 21 | 146u << 1, // w3: mtmsr r9
            dform(14, 3, 3, 1),         // w4: still in the line after mtmsr
            dform(14, 3, 3, 1),         // w5
            dform(14, 3, 3, 1),         // w6
            dform(14, 3, 3, 1),         // w7 -> falls into line 1
            dform(14, 4, 4, 1),         // line1 w0
            b(-36),                     // line1 w1: loop
        };
        Twin ji(true), in(false);
        ji.poke(prog);
        in.poke(prog);
        ji.cpu.setExternalIrq(true);
        in.cpu.setExternalIrq(true);
        ji.run(48);
        in.run(48);
        CHECK_TWINS(ji, in);
    }
    // Case B: PR flips (user mode). Real-mode fetch is unchanged by PR, but
    // xlMsr includes it, so the interpreter's next fetchBlockFast misses and
    // refills — and the chained hop must fall back to the dispatcher through
    // the same compare, after which the now-privileged mtmsr faults
    // identically on both machines.
    {
        std::vector<u32> prog = {
            dform(14, 3, 3, 1),         // w0
            dform(14, 9, 0, 0),         // w1
            dform(24, 9, 9, 0x4000),    // w2: ori r9, r9, PR
            31u << 26 | 9u << 21 | 146u << 1, // w3: mtmsr r9
            dform(14, 3, 3, 1),         // w4
            dform(14, 3, 3, 1),         // w5
            dform(14, 3, 3, 1),         // w6
            dform(14, 3, 3, 1),         // w7 -> falls into line 1
            dform(14, 4, 4, 1),         // line1 w0
            b(-36),                     // line1 w1: loop
        };
        Twin ji(true), in(false);
        ji.poke(prog);
        in.poke(prog);
        ji.run(48);
        in.run(48);
        CHECK_TWINS(ji, in);
    }
}

// ---- the direct-call fallback ---------------------------------------------

TEST_CASE("jit direct: FP arithmetic through the direct call matches execRow")
{
    // fmadds/fmuls/fadds/fsubs and their double siblings are the rows the
    // --jit-tsc split named as 45.9% of the in-game window, and they are the
    // rows the direct call was built for. Every one of them writes an FPR and
    // the FPSCR, so the twins are compared on those too — the register
    // comparison in CHECK_TWINS covers the GPRs, and RAM covers the rest via
    // the stores below.
    auto fp = [](u32 op, u32 frt, u32 fra, u32 frb, u32 frc, u32 xo,
                 bool rc = false) {
        return op << 26 | frt << 21 | fra << 16 | frb << 11 | frc << 6 |
               xo << 1 | (rc ? 1u : 0u);
    };
    for (u32 iter = 0; iter < 40; ++iter) {
        Rng r{0x5150F000ull + iter};
        std::vector<u32> prog;
        // Seed three FPRs from memory so both machines start FP-identical.
        prog.push_back(dform(14, 10, 0, i32(kData)));   // r10 = &data
        prog.push_back(dform(50, 1, 10, 0));            // lfd f1, 0(r10)
        prog.push_back(dform(50, 2, 10, 8));            // lfd f2, 8(r10)
        prog.push_back(dform(50, 3, 10, 16));           // lfd f3, 16(r10)
        for (u32 k = 0; k < 3; ++k) {
            const bool sgl = (r.next() & 1u) != 0;
            switch (r.next() % 6u) {
            case 0: prog.push_back(fp(sgl ? 59 : 63, 4, 1, 2, 0, 21)); break; // fadd(s)
            case 1: prog.push_back(fp(sgl ? 59 : 63, 4, 1, 2, 0, 20)); break; // fsub(s)
            case 2: prog.push_back(fp(sgl ? 59 : 63, 4, 1, 0, 3, 25)); break; // fmul(s)
            case 3: prog.push_back(fp(sgl ? 59 : 63, 4, 1, 2, 3, 29)); break; // fmadd(s)
            case 4: prog.push_back(fp(sgl ? 59 : 63, 4, 1, 2, 3, 28)); break; // fmsub(s)
            case 5: prog.push_back(fp(63, 4, 0, 2, 0, 12)); break;            // frsp
            }
            prog.push_back(dform(54, 4, 10, i32(64 + k * 8))); // stfd f4, N(r10)
        }
        prog.push_back(b(0));
        Twin ji(true), in(false);
        ji.poke(prog);
        in.poke(prog);
        // Real double bit patterns, not the byte ramp: the ramp is a
        // denormal/NaN soup and would exercise only the model's cold paths.
        static const double vals[] = {1.5, -2.25, 3.0e7, 1.0e-8,
                                      0.1, 12345.678, -7.5e-3, 2.0};
        for (u32 k = 0; k < 8; ++k) {
            u64 bits;
            const double d = vals[(iter + k) % 8];
            std::memcpy(&bits, &d, 8);
            for (u32 j = 0; j < 8; ++j) {
                const u8 by = u8(bits >> (56 - 8 * j));
                ji.bus.write8(kData + k * 8 + j, by);
                in.bus.write8(kData + k * 8 + j, by);
            }
        }
        ji.cpu.st.msr |= msr::FP;
        in.cpu.st.msr |= msr::FP;
        ji.run(40);
        in.run(40);
        CHECK_TWINS(ji, in);
        for (u32 k = 0; k < 32; ++k)
            CHECK(ji.cpu.st.fpr[k] == in.cpu.st.fpr[k]);
        CHECK(ji.cpu.st.fpscr == in.cpu.st.fpscr);
        CHECK(std::memcmp(ji.bus.m.data(), in.bus.m.data(),
                          ji.bus.m.size()) == 0);
    }
}

TEST_CASE("jit direct: MSR[FP] clear takes the cold arm and raises")
{
    // The direct path's gate is execRow's kPreFp arm. With MSR[FP] clear the
    // block must reach the generic fallback, which raises FpUnavailable —
    // and the vector, SRR0/SRR1 and the untouched FPR must match a machine
    // that never compiled anything.
    const u32 fadds = 59u << 26 | 4u << 21 | 1u << 16 | 2u << 11 | 21u << 1;
    std::vector<u32> prog = {
        dform(14, 3, 3, 1), fadds, dform(14, 4, 4, 1), b(0),
        dform(14, 5, 5, 1), dform(14, 5, 5, 1),
        dform(14, 5, 5, 1), dform(14, 5, 5, 1),
    };
    Twin ji(true), in(false);
    ji.poke(prog);
    in.poke(prog);
    ji.run(12); // MSR[FP] is clear out of reset
    in.run(12);
    CHECK_TWINS(ji, in);
    CHECK(ji.cpu.st.srr0 == in.cpu.st.srr0);
    CHECK(ji.cpu.st.srr1 == in.cpu.st.srr1);
    CHECK(ji.cpu.st.fpr[4] == in.cpu.st.fpr[4]);
}

TEST_CASE("jit direct: non-FP fallback rows (divide, traps, mtspr) still match")
{
    // The direct call admits every ungated row with a bound handler, not
    // just the FP ones, and those handlers DO touch the clock and memory —
    // so they run with the counters settled (keepBatch false). divwu covers
    // arithmetic, mtspr/mfspr the SPR file, and lmw/stmw a multi-word memory
    // handler whose RAM effects the twins compare byte for byte.
    for (u32 iter = 0; iter < 40; ++iter) {
        Rng r{0xD1BE0000ull + iter};
        std::vector<u32> prog = {
            dform(14, 10, 0, i32(kData)),
            xo31(4, 3, 5, 459, false, (r.next() & 1) != 0), // divwu
            xo31(6, 3, 5, 491, false, false),               // divw
            mtspr(9, 7),                                    // mtctr r7
            mfspr(9, 8),                                    // mfctr r8
            dform(46, 28, 10, 0),                           // lmw r28, 0(r10)
            dform(47, 28, 10, 32),                          // stmw r28, 32(r10)
            b(0),
        };
        differential(prog, 24, Rng{0xD1BE8000ull + iter}, true);
    }
}

TEST_CASE("jit direct: --no-jit-direct is the shim path (control, twins match)")
{
    const u32 fadds = 59u << 26 | 4u << 21 | 1u << 16 | 2u << 11 | 21u << 1;
    std::vector<u32> prog = {
        dform(14, 10, 0, i32(kData)), dform(50, 1, 10, 0),
        dform(50, 2, 10, 8),          fadds,
        dform(54, 4, 10, 64),         dform(14, 3, 3, 1),
        dform(14, 4, 4, 1),           b(0),
    };
    Twin ji(true), in(false);
    ji.cpu.jitDirectOff = true; // compiled, but fallbacks go through execRow
    ji.poke(prog);
    in.poke(prog);
    ji.cpu.st.msr |= msr::FP;
    in.cpu.st.msr |= msr::FP;
    ji.run(24);
    in.run(24);
    CHECK_TWINS(ji, in);
    CHECK(ji.cpu.st.fpr[4] == in.cpu.st.fpr[4]);
    CHECK(ji.cpu.st.fpscr == in.cpu.st.fpscr);
}

TEST_CASE("jit chain: --no-jit-chain is v1 (control run, twins still match)")
{
    std::vector<u32> prog = {
        dform(14, 3, 3, 1),  cmpi(0, 3, 12),
        bc(12, 0, 24),       dform(14, 4, 4, 1),
        dform(14, 5, 5, 1),  b(0),
        dform(14, 6, 6, 1),  dform(14, 6, 6, 1),
        dform(14, 7, 7, 1),  b(-36),
    };
    Twin ji(true), in(false);
    ji.cpu.jitChainOff = true; // compiled, but every exit takes the dispatcher
    ji.poke(prog);
    in.poke(prog);
    ji.run(120);
    in.run(120);
    CHECK_TWINS(ji, in);
    if (ji.cpu.jit)
        CHECK(ji.cpu.jitChainHops == 0);
}
