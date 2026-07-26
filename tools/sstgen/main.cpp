// sstgen — generates the SST-PPC single-instruction test suite (§6.4 of the
// project plan): deterministic constrained-random vectors, executed by the
// core to produce final states, emitted as JSON chapters (one per mnemonic).
//
// Provenance: reference-implementation derived. Chapters are generated only
// for families that are KAT- and oracle-validated; the suite's job is
// regression and cross-emulator reuse, not primary proof.
//
// Determinism: fixed master seed + per-mnemonic hash; no time, no host
// randomness. Same sstgen build -> byte-identical suite.

#include "opm/cpu.hpp"
#include "opm/bits.hpp"

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

using namespace opm;
namespace fs = std::filesystem;

namespace {

// ---- deterministic PRNG (splitmix64) ---------------------------------------

struct Rng {
    u64 s;
    u64 next()
    {
        s += 0x9E3779B97F4A7C15ull;
        u64 z = s;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }
    u32 u(u32 lo, u32 hi) { return lo + static_cast<u32>(next() % (hi - lo + 1ull)); }
    u32 word() { return static_cast<u32>(next()); }
    bool chance(u32 pct) { return u(1, 100) <= pct; }
};

// ---- recording bus ---------------------------------------------------------

class RecBus final : public Bus {
public:
    Rng* rng = nullptr;
    std::map<u32, u8> initial; // bytes the test depends on (seeded or preloaded)
    std::map<u32, u8> mem;     // live memory image

    void preload(u32 addr, u8 v)
    {
        initial[addr] = v;
        mem[addr] = v;
    }

    u8 fetch(u32 pa)
    {
        auto it = mem.find(pa);
        if (it != mem.end())
            return it->second;
        const u8 v = static_cast<u8>(rng->word());
        initial[pa] = v;
        mem[pa] = v;
        return v;
    }

    u8 read8(u32 pa) override { return fetch(pa); }
    u16 read16(u32 pa) override { return static_cast<u16>((fetch(pa) << 8) | fetch(pa + 1)); }
    u32 read32(u32 pa) override
    {
        return (u32(fetch(pa)) << 24) | (u32(fetch(pa + 1)) << 16) |
               (u32(fetch(pa + 2)) << 8) | u32(fetch(pa + 3));
    }
    u64 read64(u32 pa) override { return (u64(read32(pa)) << 32) | read32(pa + 4); }
    void write8(u32 pa, u8 v) override { mem[pa] = v; }
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

// ---- instruction word synthesis --------------------------------------------

const InsnDesc* findDesc(const char* mnem)
{
    for (size_t i = 0; i < kIsaCount; ++i)
        if (std::strcmp(kIsa[i].mnem, mnem) == 0)
            return &kIsa[i];
    return nullptr;
}

u32 baseWord(const InsnDesc& d)
{
    u32 w = u32(d.primary) << 26;
    switch (d.kind) {
    case Xk::PRI: break;
    case Xk::X19:
    case Xk::X31:
    case Xk::X63: w |= u32(d.xo) << 1; break;
    case Xk::A59:
    case Xk::A63: w |= u32(d.xo) << 1; break;
    case Xk::VA4: w |= d.xo; break;
    case Xk::VX4: w |= d.xo; break;
    case Xk::VXR4: w |= d.xo; break;
    }
    return w;
}

struct GenCtx {
    Rng rng;
    Cpu cpu;
    RecBus bus;
    u32 memBase = 0;
};

constexpr u32 kPc = 0x00001000u;
constexpr u32 kMemLo = 0x00002000u;
constexpr u32 kMemHi = 0x0000BF00u;

bool isMemOp(Pat p)
{
    switch (p) {
    case Pat::RT_D_RA: case Pat::RS_D_RA: case Pat::LSWI:
    case Pat::RA_RB:
        return true;
    default:
        return false;
    }
}
bool isIndexedMem(const InsnDesc& d)
{
    // X-form loads/stores in our v0 set use RT_RA_RB / RS_RA_RB patterns but
    // touch memory; detect by mnemonic prefix.
    const char* m = d.mnem;
    return (m[0] == 'l' || (m[0] == 's' && m[1] == 't')) && d.kind == Xk::X31 &&
           (d.pat == Pat::RT_RA_RB || d.pat == Pat::RS_RA_RB);
}

// Builds one instruction + machine state; returns the word.
u32 synth(GenCtx& g, const InsnDesc& d)
{
    Rng& r = g.rng;
    u32 w = baseWord(d);
    u32 rt = r.u(0, 31), ra = r.u(0, 31), rb = r.u(0, 31);

    const bool mem = isMemOp(d.pat) || isIndexedMem(d);
    const bool update = std::strchr(d.mnem, 'u') != nullptr &&
                        (d.pat == Pat::RT_D_RA || d.pat == Pat::RS_D_RA ||
                         isIndexedMem(d)) &&
                        d.mnem[0] == 'l';

    if (mem) {
        if (ra == 0)
            ra = r.u(1, 31);
        if (update && rt == ra)
            rt = (ra + 1) & 31u;
        g.memBase = r.u(kMemLo >> 4, kMemHi >> 4) << 4;
        g.cpu.st.gpr[ra] = g.memBase;
    }

    // Alignment constraints until the P2 exception model records faults.
    const bool needAlign4 = !std::strcmp(d.mnem, "lmw") || !std::strcmp(d.mnem, "stmw") ||
                            !std::strcmp(d.mnem, "lwarx") || !std::strcmp(d.mnem, "stwcx.");

    switch (d.pat) {
    case Pat::RT_RA_SI:
    case Pat::RA_RS_UI:
        w |= (rt << 21) | (ra << 16) | (r.word() & 0xFFFFu);
        break;
    case Pat::RT_D_RA:
    case Pat::RS_D_RA: {
        i32 disp = static_cast<i32>(r.u(0, 0x1FF)) - 0x100;
        if (needAlign4)
            disp &= ~3;
        if (!std::strcmp(d.mnem, "lmw") || !std::strcmp(d.mnem, "stmw")) {
            rt = r.u(24, 31); // keep the register run short
            disp &= 0xFF;    // and forward
        }
        w |= (rt << 21) | (ra << 16) | (static_cast<u32>(disp) & 0xFFFFu);
        break;
    }
    case Pat::RT_RA_RB:
    case Pat::RS_RA_RB:
        if (isIndexedMem(d)) {
            u32 disp = r.u(0, 0x1FF);
            if (needAlign4)
                disp &= ~3u;
            if (rb == ra)
                rb = (ra + 3) & 31u;
            if (rb == 0)
                rb = 5;
            g.cpu.st.gpr[rb] = disp;
            if (update) {
                if (ra == rt)
                    rt = (ra + 1) & 31u;
            }
        }
        w |= (rt << 21) | (ra << 16) | (rb << 11);
        if (d.flags & FL_RC)
            w |= r.chance(40) ? 1u : 0u;
        if (d.flags & FL_OE)
            w |= r.chance(30) ? 0x400u : 0u;
        break;
    case Pat::RT_RA:
        w |= (rt << 21) | (ra << 16);
        if (d.flags & FL_RC)
            w |= r.chance(40) ? 1u : 0u;
        if (d.flags & FL_OE)
            w |= r.chance(30) ? 0x400u : 0u;
        break;
    case Pat::RA_RS_RB:
        w |= (rt << 21) | (ra << 16) | (rb << 11);
        if (d.flags & FL_RC)
            w |= r.chance(40) ? 1u : 0u;
        break;
    case Pat::RA_RS:
        w |= (rt << 21) | (ra << 16);
        if (d.flags & FL_RC)
            w |= r.chance(40) ? 1u : 0u;
        break;
    case Pat::RA_RS_SH:
        w |= (rt << 21) | (ra << 16) | (r.u(0, 31) << 11);
        if (d.flags & FL_RC)
            w |= r.chance(40) ? 1u : 0u;
        break;
    case Pat::CMP_SI:
    case Pat::CMP_UI:
        w |= (r.u(0, 7) << 23) | (ra << 16) | (r.word() & 0xFFFFu);
        break;
    case Pat::CMP_RB:
        w |= (r.u(0, 7) << 23) | (ra << 16) | (rb << 11);
        break;
    case Pat::RLWINM:
        w |= (rt << 21) | (ra << 16) | (r.u(0, 31) << 11) | (r.u(0, 31) << 6) |
             (r.u(0, 31) << 1) | (r.chance(40) ? 1u : 0u);
        break;
    case Pat::RLWNM:
        w |= (rt << 21) | (ra << 16) | (rb << 11) | (r.u(0, 31) << 6) |
             (r.u(0, 31) << 1) | (r.chance(40) ? 1u : 0u);
        break;
    case Pat::B: {
        const i32 disp = (static_cast<i32>(r.u(0, 0x3FF)) - 0x200) * 4;
        w |= (static_cast<u32>(disp) & 0x03FFFFFCu) | (r.chance(50) ? 1u : 0u); // LK
        break;
    }
    case Pat::BC: {
        static const u32 bos[] = {0, 2, 4, 8, 10, 12, 16, 18, 20, 25, 13, 5};
        const i32 disp = (static_cast<i32>(r.u(0, 0xFF)) - 0x80) * 4;
        w |= (bos[r.u(0, 11)] << 21) | (r.u(0, 31) << 16) |
             (static_cast<u32>(disp) & 0xFFFCu) | (r.chance(40) ? 1u : 0u);
        break;
    }
    case Pat::BCLR:
    case Pat::BCCTR: {
        static const u32 bos[] = {0, 4, 8, 12, 16, 20};
        u32 bo = bos[r.u(0, 5)];
        if (d.pat == Pat::BCCTR)
            bo |= 4u; // bcctr with CTR decrement is an invalid form — avoid
        w |= (bo << 21) | (r.u(0, 31) << 16) | (r.chance(40) ? 1u : 0u);
        g.cpu.st.lr = (kPc + r.u(0x10, 0x200) * 4) & ~3u;
        g.cpu.st.ctr = (kPc + r.u(0x10, 0x200) * 4) & ~3u;
        break;
    }
    case Pat::CRB3:
        w |= (r.u(0, 31) << 21) | (r.u(0, 31) << 16) | (r.u(0, 31) << 11);
        break;
    case Pat::MCRF:
    case Pat::MCRXR:
        w |= (r.u(0, 7) << 23) | (r.u(0, 7) << 18);
        break;
    case Pat::MTCRF:
        w |= (rt << 21) | (r.u(0, 255) << 12);
        break;
    case Pat::MFCR:
        w |= rt << 21;
        break;
    case Pat::LSWI:
        w |= (rt << 21) | (ra << 16) | (r.u(1, 16) << 11);
        break;
    case Pat::SC:
        w |= 2u; // canonical sc form (bit 30 set)
        break;
    case Pat::NONE:
        break;
    case Pat::RA_RB: // dcbz
        if (ra == 0)
            ra = 7;
        if (rb == ra)
            rb = (ra + 3) & 31u;
        if (rb == 0)
            rb = 5;
        g.cpu.st.gpr[rb] = r.u(0, 0x1FF);
        w |= (ra << 16) | (rb << 11);
        break;
    default:
        w |= (rt << 21) | (ra << 16) | (rb << 11);
        break;
    }

    // String-indexed count lives in XER[25-31].
    if (!std::strcmp(d.mnem, "lswx") || !std::strcmp(d.mnem, "stswx"))
        g.cpu.st.xer = (g.cpu.st.xer & ~0x7Fu) | g.rng.u(0, 12);

    return w;
}

// ---- JSON emit -------------------------------------------------------------

void emitState(FILE* f, const CpuState& s, const std::map<u32, u8>& ram)
{
    fprintf(f, "{\"pc\":%u,\"gprs\":[", s.pc);
    for (int i = 0; i < 32; ++i)
        fprintf(f, "%u%s", s.gpr[i], i == 31 ? "" : ",");
    fprintf(f,
            "],\"cr\":%u,\"xer\":%u,\"lr\":%u,\"ctr\":%u,\"msr\":%u,"
            "\"srr0\":%u,\"srr1\":%u,\"dec\":%u,\"tb\":%llu,"
            "\"resv\":[%u,%u],\"ram\":[",
            s.cr, s.xer, s.lr, s.ctr, s.msr, s.srr0, s.srr1, s.dec,
            static_cast<unsigned long long>(s.tb), s.resvValid ? 1u : 0u,
            s.resvAddr);
    bool first = true;
    for (const auto& [a, v] : ram) {
        fprintf(f, "%s[%u,%u]", first ? "" : ",", a, v);
        first = false;
    }
    fprintf(f, "]}");
}

int generate(const char* mnem, const fs::path& outDir, u32 count)
{
    const InsnDesc* d = findDesc(mnem);
    if (!d) {
        fprintf(stderr, "sstgen: no such instruction %s\n", mnem);
        return 1;
    }

    u64 seed = 0x4F70656E504D6163ull; // fixed master seed
    for (const char* p = mnem; *p; ++p)
        seed = seed * 131 + static_cast<u8>(*p);

    fs::create_directories(outDir);
    std::string fname = std::string(mnem);
    for (char& ch : fname)
        if (ch == '.')
            ch = '_';
    FILE* f = fopen((outDir / (fname + ".json")).string().c_str(), "wb");
    if (!f)
        return 1;
    fprintf(f, "{\"cpu\":\"mpc7400\",\"chapter\":\"%s\",\"provenance\":"
               "\"reference-implementation\",\"seed\":%llu,\"tests\":[\n",
            mnem, static_cast<unsigned long long>(seed));

    u32 made = 0, attempts = 0;
    Rng master{seed};
    while (made < count && attempts < count * 4) {
        ++attempts;
        GenCtx g{Rng{master.next()}, Cpu{}, RecBus{}, 0};
        g.bus.rng = &g.rng;
        g.cpu.attach(g.bus);
        g.cpu.reset();
        // Supervisor-ish random MSR: PR/FP/ME/FE0/FE1/IR/DR/RI/PM mix;
        // EE/SE/BE/IP kept 0 so single-step vectors stay self-contained.
        g.cpu.st.msr = g.rng.word() & 0x00007936u;
        g.cpu.st.srr0 = g.rng.word() & ~3u;
        g.cpu.st.srr1 = g.rng.word() & 0x0000FF73u;
        g.cpu.st.dec = g.rng.word();
        g.cpu.st.tb = g.rng.u(0, 0xFFFF);
        for (int i = 0; i < 32; ++i)
            g.cpu.st.gpr[i] = g.rng.word();
        g.cpu.st.cr = g.rng.word();
        g.cpu.st.xer = (g.rng.word() & 0xE0000000u) | g.rng.u(0, 16);
        g.cpu.st.lr = g.rng.word() & ~3u;
        g.cpu.st.ctr = g.rng.word();
        g.cpu.st.pc = kPc;
        if (g.rng.chance(25)) {
            g.cpu.st.resvValid = true;
            g.cpu.st.resvAddr = (kMemLo + g.rng.u(0, 0x400) * 32) & ~31u;
        }

        const u32 w = synth(g, *d);
        g.bus.preload(kPc + 0, static_cast<u8>(w >> 24));
        g.bus.preload(kPc + 1, static_cast<u8>(w >> 16));
        g.bus.preload(kPc + 2, static_cast<u8>(w >> 8));
        g.bus.preload(kPc + 3, static_cast<u8>(w));

        const CpuState before = g.cpu.st;
        const std::map<u32, u8> ramBeforeRun = g.bus.initial;
        g.cpu.step();
        if (g.cpu.halted)
            continue; // trap taken / not-yet-modeled path: skip in v0

        if (made)
            fprintf(f, ",\n");
        fprintf(f, "{\"name\":\"%s %u\",\"insn\":%u,\"initial\":", mnem, made, w);
        // initial.ram = everything the run turned out to depend on
        CpuState init = before;
        emitState(f, init, g.bus.initial);
        fprintf(f, ",\"final\":");
        // final ram: value of every initial address + all written addresses
        std::map<u32, u8> finalRam;
        for (const auto& [a, v] : g.bus.initial)
            finalRam[a] = g.bus.mem[a];
        for (const auto& [a, v] : g.bus.mem)
            if (g.bus.initial.count(a) == 0)
                finalRam[a] = v; // written fresh
        emitState(f, g.cpu.st, finalRam);
        fprintf(f, "}");
        ++made;
    }
    fprintf(f, "\n]}\n");
    fclose(f);
    printf("%s: %u tests\n", mnem, made);
    return made == count ? 0 : 1;
}

const char* kV0[] = {
    "addi", "addis", "addic", "addic.", "subfic", "mulli",
    "ori", "oris", "xori", "xoris", "andi.", "andis.",
    "cmpi", "cmpli", "cmp", "cmpl",
    "rlwinm", "rlwnm", "rlwimi",
    "add", "addc", "adde", "addme", "addze",
    "subf", "subfc", "subfe", "subfme", "subfze", "neg",
    "mullw", "mulhw", "mulhwu", "divw", "divwu",
    "and", "andc", "or", "orc", "xor", "eqv", "nor", "nand",
    "slw", "srw", "sraw", "srawi", "cntlzw", "extsb", "extsh",
    "mcrf", "crand", "cror", "crxor", "crnand", "crnor", "creqv", "crandc",
    "crorc", "mfcr", "mtcrf", "mcrxr",
    "b", "bc", "bclr", "bcctr",
    "lwz", "lbz", "lhz", "lha", "lwzx", "lbzx", "lhzx", "lhax",
    "lwzu", "lbzu", "lhzu", "lhau", "lwzux", "lbzux", "lhzux", "lhaux",
    "stw", "stb", "sth", "stwx", "stbx", "sthx",
    "stwu", "stbu", "sthu", "stwux", "stbux", "sthux",
    "lwbrx", "lhbrx", "stwbrx", "sthbrx",
    "lmw", "stmw", "lswi", "stswi", "lswx", "stswx",
    "lwarx", "stwcx.", "dcbz",
    // supervisor / exception chapters (P2): vectored outcomes recorded
    "sc", "tw", "twi", "rfi", "mfmsr", "mtmsr",
    "mfspr", "mtspr", "mftb",
    "mtsr", "mfsr", "mtsrin", "mfsrin",
    "tlbia", "fsqrt", "fsqrts",
};

} // namespace

int main(int argc, char** argv)
{
    fs::path outDir = "sst";
    u32 count = 200;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--out") && i + 1 < argc)
            outDir = argv[++i];
        else if (!std::strcmp(argv[i], "--count") && i + 1 < argc)
            count = static_cast<u32>(std::strtoul(argv[++i], nullptr, 0));
    }

    int rc = 0;
    for (const char* m : kV0)
        rc |= generate(m, outDir, count);
    return rc;
}
