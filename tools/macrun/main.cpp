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

#include <cstdio>
#include <cstdlib>
#include <cstring>
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
    while (executed < maxInsns && !cpu.halted) {
        const u32 pc = cpu.st.pc;
        ring.push(pc, bus.read32(pc));
        if (trace) {
            disassemble(ring.e[(ring.n - 1) & 31u].insn, pc, text, sizeof text,
                        Style::Gnu);
            fprintf(stderr, "%08x: %s\n", pc, text);
        }
        cpu.step();
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
