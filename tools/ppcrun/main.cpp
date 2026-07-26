// ppcrun — CLI harness for the OpenPowerMac MPC7400 core.
//
// Loads a flat binary or big-endian ELF32 image into flat RAM, then either
// executes it (with a tiny MMIO console) or disassembles its executable
// sections (the decode oracle used to diff against llvm-objdump).
//
// MMIO (test rig only, not part of any real machine):
//   0xF0000000  write u8/u32: putchar
//   0xF0000004  write u32: exit with that code

#include "opm/cpu.hpp"
#include "opm/bits.hpp"
#include "kat.hpp"
#include "sst.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using namespace opm;

namespace {

constexpr u32 kMmioPutc = 0xF0000000u;
constexpr u32 kMmioExit = 0xF0000004u;

class FlatBus final : public Bus {
public:
    explicit FlatBus(size_t ramBytes) : ram_(ramBytes, 0) {}

    bool exited = false;
    int exitCode = 0;

    u8* data() { return ram_.data(); }
    size_t size() const { return ram_.size(); }

    u8 read8(u32 pa) override { return inRam(pa, 1) ? ram_[pa] : 0; }
    u16 read16(u32 pa) override
    {
        if (!inRam(pa, 2)) return 0;
        return static_cast<u16>((ram_[pa] << 8) | ram_[pa + 1]);
    }
    u32 read32(u32 pa) override
    {
        if (!inRam(pa, 4)) return 0;
        return (u32(ram_[pa]) << 24) | (u32(ram_[pa + 1]) << 16) |
               (u32(ram_[pa + 2]) << 8) | u32(ram_[pa + 3]);
    }
    u64 read64(u32 pa) override
    {
        return (u64(read32(pa)) << 32) | read32(pa + 4);
    }

    void write8(u32 pa, u8 v) override
    {
        if (pa == kMmioPutc) { fputc(v, stdout); return; }
        if (inRam(pa, 1)) ram_[pa] = v;
    }
    void write16(u32 pa, u16 v) override
    {
        if (!inRam(pa, 2)) return;
        ram_[pa] = static_cast<u8>(v >> 8);
        ram_[pa + 1] = static_cast<u8>(v);
    }
    void write32(u32 pa, u32 v) override
    {
        if (pa == kMmioPutc) { fputc(static_cast<int>(v & 0xFF), stdout); return; }
        if (pa == kMmioExit) { exited = true; exitCode = static_cast<int>(v); return; }
        if (!inRam(pa, 4)) return;
        ram_[pa] = static_cast<u8>(v >> 24);
        ram_[pa + 1] = static_cast<u8>(v >> 16);
        ram_[pa + 2] = static_cast<u8>(v >> 8);
        ram_[pa + 3] = static_cast<u8>(v);
    }
    void write64(u32 pa, u64 v) override
    {
        write32(pa, static_cast<u32>(v >> 32));
        write32(pa + 4, static_cast<u32>(v));
    }

private:
    std::vector<u8> ram_;
    bool inRam(u32 pa, u32 len) const { return u64(pa) + len <= ram_.size(); }
};

std::vector<u8> readFile(const char* path)
{
    FILE* f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "ppcrun: cannot open %s\n", path); exit(2); }
    fseek(f, 0, SEEK_END);
    const long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    std::vector<u8> v(static_cast<size_t>(n));
    if (n > 0 && fread(v.data(), 1, v.size(), f) != v.size()) {
        fprintf(stderr, "ppcrun: short read on %s\n", path);
        exit(2);
    }
    fclose(f);
    return v;
}

u16 be16(const u8* p) { return static_cast<u16>((p[0] << 8) | p[1]); }
u32 be32(const u8* p)
{
    return (u32(p[0]) << 24) | (u32(p[1]) << 16) | (u32(p[2]) << 8) | u32(p[3]);
}

struct ExecRange { u32 addr; u32 size; };

struct Image {
    bool isElf = false;
    u32 entry = 0;
    std::vector<ExecRange> exec; // executable sections (ELF) or whole image (flat)
};

Image loadImage(const std::vector<u8>& file, FlatBus& bus, u32 flatBase)
{
    Image img;
    if (file.size() >= 52 && file[0] == 0x7F && file[1] == 'E' && file[2] == 'L' &&
        file[3] == 'F') {
        if (file[4] != 1 || file[5] != 2) {
            fprintf(stderr, "ppcrun: ELF must be 32-bit big-endian\n");
            exit(2);
        }
        img.isElf = true;
        img.entry = be32(&file[24]);
        const u32 phoff = be32(&file[28]);
        const u32 shoff = be32(&file[32]);
        const u16 phentsize = be16(&file[42]);
        const u16 phnum = be16(&file[44]);
        const u16 shentsize = be16(&file[46]);
        const u16 shnum = be16(&file[48]);

        for (u16 i = 0; i < phnum; ++i) {
            const u8* ph = file.data() + phoff + u32(i) * phentsize;
            const u32 type = be32(ph + 0);
            if (type != 1) // PT_LOAD
                continue;
            const u32 offset = be32(ph + 4);
            const u32 paddr = be32(ph + 8); // use vaddr
            const u32 filesz = be32(ph + 16);
            if (u64(paddr) + filesz <= bus.size() && u64(offset) + filesz <= file.size())
                memcpy(bus.data() + paddr, file.data() + offset, filesz);
        }
        for (u16 i = 0; i < shnum; ++i) {
            const u8* sh = file.data() + shoff + u32(i) * shentsize;
            const u32 flags = be32(sh + 8);
            const u32 addr = be32(sh + 12);
            const u32 size = be32(sh + 20);
            if ((flags & 0x4u) && size) // SHF_EXECINSTR
                img.exec.push_back({addr, size});
        }
    } else {
        if (u64(flatBase) + file.size() > bus.size()) {
            fprintf(stderr, "ppcrun: image does not fit in RAM\n");
            exit(2);
        }
        memcpy(bus.data() + flatBase, file.data(), file.size());
        img.entry = flatBase;
        img.exec.push_back({flatBase, static_cast<u32>(file.size() & ~3u)});
    }
    return img;
}

void usage()
{
    fprintf(stderr,
            "usage: ppcrun [options] <image.bin|image.elf>\n"
            "  --base ADDR    flat-image load address (default 0x100000)\n"
            "  --entry ADDR   override entry point\n"
            "  --ram MB       RAM size in MiB (default 64)\n"
            "  --max N        stop after N instructions (default 50000000)\n"
            "  --trace        disassemble each instruction as it executes\n"
            "  --disasm       disassemble executable sections and exit\n"
            "  --style S      disassembly style: gnu | llvm (default gnu)\n"
            "  --ledger       dump the ISA ledger (implemented/decoded) and exit\n"
            "  --kat PATH     run known-answer tests (.kat file or directory)\n"
            "  --sst PATH     run SST-PPC JSON chapters (file or directory)\n");
    exit(2);
}

} // namespace

int main(int argc, char** argv)
{
    const char* path = nullptr;
    u32 base = 0x100000u;
    u32 entry = 0;
    bool haveEntry = false;
    u64 maxInsns = 50000000ull;
    size_t ramMb = 64;
    bool trace = false, disasmOnly = false, ledger = false;
    Style style = Style::Gnu;

    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        auto next = [&]() -> const char* {
            if (i + 1 >= argc) usage();
            return argv[++i];
        };
        if (!strcmp(a, "--base")) base = static_cast<u32>(strtoul(next(), nullptr, 0));
        else if (!strcmp(a, "--entry")) { entry = static_cast<u32>(strtoul(next(), nullptr, 0)); haveEntry = true; }
        else if (!strcmp(a, "--ram")) ramMb = strtoul(next(), nullptr, 0);
        else if (!strcmp(a, "--max")) maxInsns = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--trace")) trace = true;
        else if (!strcmp(a, "--disasm")) disasmOnly = true;
        else if (!strcmp(a, "--ledger")) ledger = true;
        else if (!strcmp(a, "--style")) style = strcmp(next(), "llvm") ? Style::Gnu : Style::Llvm;
        else if (!strcmp(a, "--kat")) return runKats(next()) ? 1 : 0;
        else if (!strcmp(a, "--sst")) return runSst(next()) ? 1 : 0;
        else if (a[0] == '-') usage();
        else path = a;
    }

    if (ledger) {
        bindHandlers(); // handlers live in the dispatch slots, not the table
        size_t total = 0, bound = 0, ill = 0;
        for (size_t n = 0; n < kIsaCount; ++n) {
            const InsnDesc& d = kIsa[n];
            ++total;
            const bool impl = handlerFor(&d) != nullptr;
            if (impl) ++bound;
            if (d.flags & FL_ILL7400) ++ill;
            printf("%-12s op=%2u xo=%4u %s%s\n", d.mnem, d.primary, d.xo,
                   impl ? "IMPL" : "decode-only",
                   (d.flags & FL_ILL7400) ? " [illegal-on-7400]" : "");
        }
        printf("-- %zu instructions, %zu implemented, %zu illegal-on-7400\n",
               total, bound, ill);
        return 0;
    }

    if (!path)
        usage();

    FlatBus bus(ramMb * 1024 * 1024);
    const std::vector<u8> file = readFile(path);
    const Image img = loadImage(file, bus, base);

    if (disasmOnly) {
        char text[128];
        for (const ExecRange& r : img.exec) {
            for (u32 a = r.addr; a + 4 <= r.addr + r.size; a += 4) {
                const u32 w = bus.read32(a);
                disassemble(w, a, text, sizeof text, style);
                printf("%8x: %08x  %s\n", a, w, text);
            }
        }
        return 0;
    }

    Cpu cpu;
    cpu.attach(bus);
    cpu.reset();
    cpu.st.pc = haveEntry ? entry : img.entry;
    cpu.st.msr |= msr::FP | msr::VEC; // the rig hands programs a usable FPU
                                      // and vector unit, as firmware would
                                      // (KAT/SST paths set MSR themselves)
    cpu.st.hid0 |= 0x0000C000u;       // ICE|DCE: caches on, so dcbz works
    cpu.realModeInhibitBase = 0xF0000000u; // rig MMIO stays uncached

    char text[128];
    u64 executed = 0;
    int excLogged = 0;
    while (executed < maxInsns && !bus.exited) {
        if (trace) {
            const u32 w = bus.read32(cpu.st.pc);
            disassemble(w, cpu.st.pc, text, sizeof text, style);
            fprintf(stderr, "%08x: %08x  %s\n", cpu.st.pc, w, text);
        }
        cpu.step();
        ++executed;
        // First few exception entries, for diagnosing vector run-offs.
        if (cpu.raisedThisStep && excLogged < 8) {
            ++excLogged;
            fprintf(stderr,
                    "-- exc -> %08x srr0=%08x srr1=%08x dsisr=%08x dar=%08x\n",
                    cpu.st.pc, cpu.st.srr0, cpu.st.srr1, cpu.st.dsisr,
                    cpu.st.dar);
        }
    }

    fflush(stdout);
    fprintf(stderr, "-- executed %llu instructions\n",
            static_cast<unsigned long long>(executed));
    if (!cpu.unimplemented.empty()) {
        fprintf(stderr, "-- unimplemented hits:\n");
        for (const auto& [mn, cnt] : cpu.unimplemented)
            fprintf(stderr, "   %-12s x%llu\n", mn.c_str(),
                    static_cast<unsigned long long>(cnt));
    }
    if (!cpu.unknownWords.empty()) {
        fprintf(stderr, "-- unknown words:\n");
        for (const auto& [w, cnt] : cpu.unknownWords)
            fprintf(stderr, "   0x%08x x%llu\n", w,
                    static_cast<unsigned long long>(cnt));
    }
    return bus.exited ? bus.exitCode : 1;
}
