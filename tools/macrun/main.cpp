// macrun — Arc 2 machine runner. Loads a Gossamer boot ROM at 0xFFC00000
// and executes from the hardware reset vector with authentic reset state
// (no test-rig conveniences: caches/FPU/vector gates exactly as HRESET
// leaves them; the ROM is expected to configure its own world).
//
// The M0 deliverable is the instrumentation: how far the ROM's init gets,
// which device addresses it touches (deduped stub log), the exception
// entries it takes, and the last instructions before the stop point.

#include "opm/cpu.hpp"
#include "opm/bits.hpp"
#include "opm/gossamer.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <vector>

using namespace opm;

namespace {

std::vector<u8> readFile(const char* path)
{
    FILE* f = fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "macrun: cannot open %s\n", path);
        exit(2);
    }
    fseek(f, 0, SEEK_END);
    const long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    std::vector<u8> v(static_cast<size_t>(n));
    if (n > 0 && fread(v.data(), 1, v.size(), f) != v.size()) {
        fprintf(stderr, "macrun: short read on %s\n", path);
        exit(2);
    }
    fclose(f);
    return v;
}

struct Ring {
    struct Ent {
        u32 pc = 0, insn = 0;
    };
    Ent e[32];
    u32 n = 0;
    void push(u32 pc, u32 insn) { e[n++ & 31u] = {pc, insn}; }
};

} // namespace

int main(int argc, char** argv)
{
    const char* romPath = nullptr;
    const char* fbPath = nullptr;
    u64 maxInsns = 50000000ull;
    size_t ramMb = 64;
    bool trace = false;
    int excShow = 16;

    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        auto next = [&]() -> const char* {
            if (i + 1 >= argc) {
                fprintf(stderr, "macrun: missing value for %s\n", a);
                exit(2);
            }
            return argv[++i];
        };
        if (!strcmp(a, "--rom")) romPath = next();
        else if (!strcmp(a, "--ram")) ramMb = strtoul(next(), nullptr, 0);
        else if (!strcmp(a, "--max")) maxInsns = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--trace")) trace = true;
        else if (!strcmp(a, "--fb")) fbPath = next();
        else if (!strcmp(a, "--exc")) excShow = atoi(next());
        else {
            fprintf(stderr,
                    "usage: macrun --rom FILE [--ram MB] [--max N] [--trace] [--exc N]\n");
            return 2;
        }
    }
    if (!romPath) {
        fprintf(stderr, "macrun: --rom is required\n");
        return 2;
    }

    std::vector<u8> rom = readFile(romPath);
    printf("-- rom: %zu bytes, ram: %zu MiB\n", rom.size(), ramMb);
    GossamerBus bus(ramMb * 1024 * 1024, std::move(rom));

    Cpu cpu;
    cpu.attach(bus);
    cpu.reset(); // pc = 0xFFF00100, MSR[IP] set: vectors in ROM — authentic

    Ring ring;
    char text[128];
    u64 executed = 0;
    int excLogged = 0;
    std::map<u32, u64> pcHist; // sampled every 64 steps
    // Coverage timeline: first execution in each 1 KB region, timestamped —
    // the boot's macro story, ending at its last new territory.
    std::map<u32, u64> seen;
    std::vector<std::pair<u64, u32>> firsts;
    while (executed < maxInsns && !cpu.halted) {
        const u32 pc = cpu.st.pc;
        cpu.step();
        const u32 region = pc >> 10;
        if (seen.emplace(region, executed).second)
            firsts.push_back({executed, pc});
        if ((executed & 63u) == 0)
            ++pcHist[pc];
        ring.push(pc, cpu.curInsn); // the word actually fetched (translated)
        if (trace) {
            disassemble(cpu.curInsn, pc, text, sizeof text, Style::Gnu);
            fprintf(stderr, "%08x: %s\n", pc, text);
        }
        bus.macio().tick();
        cpu.setExternalIrq(bus.macio().irqAsserted());
        ++executed;
        if (cpu.raisedThisStep && excLogged < excShow) {
            ++excLogged;
            printf("-- exc -> %08x from srr0=%08x srr1=%08x dsisr=%08x dar=%08x\n",
                   cpu.st.pc, cpu.st.srr0, cpu.st.srr1, cpu.st.dsisr,
                   cpu.st.dar);
        }
    }

    printf("-- executed %llu instructions; stop pc=%08x%s\n",
           static_cast<unsigned long long>(executed), cpu.st.pc,
           cpu.halted ? " (halted)" : "");
    {
        char st[160];
        bus.macio().debugState(st, sizeof st);
        printf("-- %s\n", st);
        printf("-- msr=%08x dec=%08x extLine=%d\n", cpu.st.msr, cpu.st.dec,
               cpu.extIrqLine ? 1 : 0);
    }
    {
        std::vector<std::pair<u64, u32>> top;
        for (const auto& [pc, n] : pcHist)
            top.push_back({n, pc});
        std::sort(top.rbegin(), top.rend());
        printf("-- hottest sampled pcs:\n");
        for (size_t k = 0; k < top.size() && k < 16; ++k)
            printf("   %08x  samples=%llu\n", top[k].second,
                   static_cast<unsigned long long>(top[k].first));
    }
    {
        printf("-- coverage timeline (%zu regions; last 40 first-entries):\n",
               firsts.size());
        const size_t start = firsts.size() > 40 ? firsts.size() - 40 : 0;
        for (size_t k = start; k < firsts.size(); ++k)
            printf("   @%-12llu %08x\n",
                   static_cast<unsigned long long>(firsts[k].first),
                   firsts[k].second);
    }
    if (cpu.halted)
        printf("-- halt: %s\n", cpu.haltReason.c_str());

    printf("-- last instructions before stop:\n");
    const u32 count = ring.n < 32 ? ring.n : 32;
    for (u32 k = 0; k < count; ++k) {
        const auto& e = ring.e[(ring.n - count + k) & 31u];
        disassemble(e.insn, e.pc, text, sizeof text, Style::Gnu);
        printf("   %08x: %08x  %s\n", e.pc, e.insn, text);
    }

    if (bus.romWrites())
        printf("-- rom writes dropped: %llu\n",
               static_cast<unsigned long long>(bus.romWrites()));
    if (!bus.stubLog().empty()) {
        printf("-- stub/unmapped accesses (%zu unique):\n",
               bus.stubLog().size());
        for (const auto& [pa, t] : bus.stubLog())
            printf("   %08x  reads=%-8llu writes=%-8llu lastWrite=%08x\n", pa,
                   static_cast<unsigned long long>(t.reads),
                   static_cast<unsigned long long>(t.writes), t.lastWrite);
    }
    if (!bus.macio().viaTrace().empty()) {
        static const char* rn[16] = {"ORB", "ORA", "DDRB", "DDRA", "T1CL",
                                     "T1CH", "T1LL", "T1LH", "T2CL", "T2CH",
                                     "SR",  "ACR", "PCR",  "IFR",  "IER",
                                     "ORAnh"};
        printf("-- via trace (%zu ops, ORB reads elided):\n",
               bus.macio().viaTrace().size());
        for (const auto& op : bus.macio().viaTrace())
            printf("   %s %-5s %02x\n", op.write ? "wr" : "rd", rn[op.reg & 15],
                   op.val);
    }
    if (!bus.pci().probeLog().empty()) {
        printf("-- pci config probes (dev:reg -> reads/writes):\n");
        for (const auto& [key, p] : bus.pci().probeLog())
            printf("   %02x:%02x  r=%llu w=%llu\n", key >> 8, key & 0xFFu,
                   static_cast<unsigned long long>(p.reads),
                   static_cast<unsigned long long>(p.writes));
        printf("-- ati aperture base: %08x\n", bus.pci().atiBase());
    }
    if (!bus.atiRegLog().empty()) {
        printf("-- ati register-block traffic:\n");
        for (const auto& [off, t] : bus.atiRegLog())
            printf("   +%06x  reads=%-8llu writes=%-8llu lastWrite=%02x\n", off,
                   static_cast<unsigned long long>(t.reads),
                   static_cast<unsigned long long>(t.writes), t.lastWrite);
    }
    {
        u64 nonzero = 0;
        for (u8 b : bus.atiMem())
            if (b)
                ++nonzero;
        printf("-- ati aperture: %llu nonzero bytes\n",
               static_cast<unsigned long long>(nonzero));
        if (fbPath && nonzero) {
            FILE* f = fopen(fbPath, "wb");
            if (f) { // 640x480 8bpp assumed until the CRTC says otherwise
                fprintf(f, "P5\n640 480\n255\n");
                fwrite(bus.atiMem().data(), 1, 640 * 480, f);
                fclose(f);
                printf("-- framebuffer written: %s\n", fbPath);
            }
        }
    }
    if (!bus.macio().cuda().commandLog().empty()) {
        printf("-- cuda packets (type:cmd -> count):\n");
        for (const auto& [key, cnt] : bus.macio().cuda().commandLog())
            printf("   %02x:%02x x%llu\n", key >> 8, key & 0xFFu,
                   static_cast<unsigned long long>(cnt));
    }
    if (!bus.macio().xferLog().empty()) {
        printf("-- cuda transport (>[to cuda] <[to host] s[SR write] "
               "b[portB edge]):\n   ");
        for (const auto& b : bus.macio().xferLog()) {
            const char tag[] = {'<', '>', 's', 'b'};
            printf("%c%02x ", tag[b.toCuda & 3], b.val);
        }
        printf("\n");
    }
    if (!bus.macio().unmodeledLog().empty()) {
        printf("-- mac-io unmodeled register traffic (%zu unique):\n",
               bus.macio().unmodeledLog().size());
        size_t shown = 0;
        for (const auto& [off, t] : bus.macio().unmodeledLog()) {
            if (++shown > 48) {
                printf("   ... (%zu more)\n",
                       bus.macio().unmodeledLog().size() - 48);
                break;
            }
            printf("   +%06x  reads=%-8llu writes=%-8llu lastWrite=%02x\n", off,
                   static_cast<unsigned long long>(t.reads),
                   static_cast<unsigned long long>(t.writes), t.lastWrite);
        }
    }
    if (!cpu.unimplemented.empty()) {
        printf("-- unimplemented hits:\n");
        for (const auto& [mn, cnt] : cpu.unimplemented)
            printf("   %-12s x%llu\n", mn.c_str(),
                   static_cast<unsigned long long>(cnt));
    }
    if (!cpu.unknownWords.empty()) {
        printf("-- unknown words:\n");
        for (const auto& [w, cnt] : cpu.unknownWords)
            printf("   0x%08x x%llu\n", w,
                   static_cast<unsigned long long>(cnt));
    }
    return 0;
}
