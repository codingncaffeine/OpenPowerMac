// g4run — Sawtooth (Power Mac G4 AGP) machine runner. Loads the 1 MB New
// World boot ROM at 0xFFF00000 and executes from the hardware reset vector
// with authentic reset state. M-SAW-0 deliverable is the instrumentation:
// how far Open Firmware 3.x gets, which physical addresses it touches
// (the deduplicated unclaimed-access log IS the Uni-North/KeyLargo map),
// the exceptions it takes, and the last instructions before the stop.

#include "opm/cpu.hpp"
#include "opm/insn.hpp"
#include "opm/sawtooth.hpp"

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
        fprintf(stderr, "g4run: cannot open %s\n", path);
        exit(2);
    }
    fseek(f, 0, SEEK_END);
    const long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    std::vector<u8> v(static_cast<size_t>(n));
    if (n > 0 && fread(v.data(), 1, v.size(), f) != v.size()) {
        fprintf(stderr, "g4run: short read on %s\n", path);
        exit(2);
    }
    fclose(f);
    return v;
}

struct Ring {
    struct Ent {
        u32 pc = 0, insn = 0;
    };
    Ent e[128];
    u32 n = 0;
    void push(u32 pc, u32 insn) { e[n++ & 127u] = {pc, insn}; }
};

} // namespace

int main(int argc, char** argv)
{
    const char* romPath = nullptr;
    u64 maxInsns = 50000000ull;
    size_t ramMb = 256;
    bool trace = false;
    int excShow = 16;
    u32 disStart = 0, disEnd = 0;
    u32 fastTb = 0; // extra TB cycles per instruction: compresses the
                    // ROM's wall-clock waits (harness lever, not machine
                    // truth — timings scale, ordering is preserved)
    u64 fastTbUntil = ~0ull; // compression cutoff: the OS era runs its
                             // scheduler off the DEC and livelocks if
                             // the timebase runs tens of times fast
    const char* ramDumpPath = nullptr;
    bool serialCr = false;
    const char* cdPath = nullptr;
    const char* serialInput = nullptr; // ';' separates lines
    u64 serialAt = 240000000ull;       // inject once the prompt is up

    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        auto next = [&]() -> const char* {
            if (i + 1 >= argc) {
                fprintf(stderr, "g4run: missing value for %s\n", a);
                exit(2);
            }
            return argv[++i];
        };
        if (!strcmp(a, "--rom")) romPath = next();
        else if (!strcmp(a, "--ram")) ramMb = strtoul(next(), nullptr, 0);
        else if (!strcmp(a, "--max")) maxInsns = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--trace")) trace = true;
        else if (!strcmp(a, "--exc")) excShow = atoi(next());
        else if (!strcmp(a, "--dis")) {
            disStart = static_cast<u32>(strtoul(next(), nullptr, 0));
            disEnd = static_cast<u32>(strtoul(next(), nullptr, 0));
        }
        else if (!strcmp(a, "--fast-tb"))
            fastTb = static_cast<u32>(strtoul(next(), nullptr, 0));
        else if (!strcmp(a, "--fast-tb-until"))
            fastTbUntil = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--dump-ram")) ramDumpPath = next();
        else if (!strcmp(a, "--serial-cr")) serialCr = true;
        else if (!strcmp(a, "--serial-input")) serialInput = next();
        else if (!strcmp(a, "--serial-at"))
            serialAt = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--cd")) cdPath = next();
        else {
            fprintf(stderr,
                    "usage: g4run --rom FILE [--ram MB] [--max N] [--trace] "
                    "[--exc N] [--dis A B]\n");
            return 2;
        }
    }
    if (!romPath) {
        fprintf(stderr, "g4run: --rom is required\n");
        return 2;
    }

    std::vector<u8> rom = readFile(romPath);
    if (rom.size() != SawtoothBus::kRomSize)
        printf("-- note: rom is %zu bytes (expected 1 MiB)\n", rom.size());
    printf("-- rom: %zu bytes, ram: %zu MiB\n", rom.size(), ramMb);
    SawtoothBus bus(ramMb * 1024 * 1024, std::move(rom));

    char text[128];
    if (disStart && disEnd > disStart) {
        for (u32 a = disStart & ~3u; a < disEnd; a += 4) {
            const u32 w = bus.read32(a);
            disassemble(w, a, text, sizeof text, Style::Gnu);
            printf("   %08x: %08x  %s\n", a, w, text);
        }
        return 0;
    }

    if (cdPath) {
        if (bus.attachCd(cdPath))
            printf("-- cd attached: %s\n", cdPath);
        else
            printf("-- cd attach FAILED: %s\n", cdPath);
    }

    if (serialCr)
        bus.injectSerial("\r"); // CR in the escape window -> serial console

    Cpu cpu;
    cpu.attach(bus);
    cpu.reset(); // pc = 0xFFF00100, MSR[IP]: vectors in ROM — authentic
    u64 executed = 0;
    bus.pcRef = &cpu.st.pc;
    bus.stamp = &executed;
    bus.cd().stamp = &executed;
    bus.pic().stamp = &executed;

    Ring ring;
    int excLogged = 0;
    struct ExcEnt {
        u64 at;
        u32 vec, srr0, srr1, dsisr, dar;
    };
    ExcEnt excRing[16] = {};
    u32 excRingAt = 0;
    std::map<u32, u64> pcHist; // sampled every 64 steps
    std::map<u32, u64> seen;   // first execution per 1 KB region
    std::vector<std::pair<u64, u32>> firsts;

    while (executed < maxInsns && !cpu.halted) {
        const u32 pc = cpu.st.pc;
        cpu.step();
        const u32 region = pc >> 10;
        if (seen.emplace(region, executed).second)
            firsts.push_back({executed, pc});
        if ((executed & 63u) == 0)
            ++pcHist[pc];
        ring.push(pc, cpu.curInsn);
        if (trace) {
            disassemble(cpu.curInsn, pc, text, sizeof text, Style::Gnu);
            fprintf(stderr, "%08x: %s\n", pc, text);
        }
        if (fastTb && executed < fastTbUntil)
            cpu.tick(fastTb);
        bus.syncIrqs();
        cpu.setExternalIrq(bus.pic().cpuLine());
        if (serialInput && executed == serialAt) {
            std::string s(serialInput);
            for (char& c : s)
                if (c == ';')
                    c = '\r';
            bus.injectSerial(s + "\r");
            printf("-- serial input injected @%llu: %s\n",
                   static_cast<unsigned long long>(executed), serialInput);
        }
        static int nsectTrail = -1;
        if (nsectTrail < 0 && !bus.ataLog().empty() &&
            bus.ataLog().back().pa == 0x20020u &&
            !(bus.ataLog().back().pa & 1u) && bus.ataLog().size() > 40)
            nsectTrail = 3000; // trace the driver's continuation
        if (nsectTrail > 0) {
            --nsectTrail;
            if (cpu.st.pc == 0xFF80B710u)
                printf("-- post-nsect C! a=%08x v=%02x\n", cpu.st.gpr[20],
                       cpu.st.gpr[21] & 0xFFu);
            else if (cpu.st.pc == 0xFF80B1C0u)
                printf("-- post-nsect C@ a=%08x\n", cpu.st.gpr[20]);
        }
        static u32 slotPrev[2] = {0xEE00EE00u, 0xEE00EE00u};
        if (executed > 1030000000ull && (executed & 0x7FFFFu) == 0) {
            u32 v0 = 0, v1 = 0;
            if (cpu.l1dPeek32(0x03EFD500u, v0) == false)
                v0 = bus.read32(0x03EFD500u);
            if (cpu.l1dPeek32(0x03EFD540u, v1) == false)
                v1 = bus.read32(0x03EFD540u);
            if (v0 != slotPrev[0] || v1 != slotPrev[1]) {
                printf("-- slots @%llu [d500]=%08x [d540]=%08x pc=%08x\n",
                       static_cast<unsigned long long>(executed), v0, v1,
                       cpu.st.pc);
                slotPrev[0] = v0;
                slotPrev[1] = v1;
            }
        }
        // ExpandMem+0x294 watch (PA 000116C4): the Mixed Mode call-68K
        // primitive jumps through this cell and it is 0 at the fatal
        // call — the sad-mac's null procPtr. This names every writer:
        // the builder's clear, and (if it ever runs) the real install.
        static u32 instShown = 0;
        if ((pc == 0xFFE2325Cu || pc == 0xFFE23380u) && instShown < 12) {
            ++instShown;
            printf("-- installer %s @%llu lr=%08x r3=%08x r4=%08x r5=%08x "
                   "r29=%08x\n",
                   pc == 0xFFE2325Cu ? "ENTRY ffe2325c" : "uninstall 23380",
                   static_cast<unsigned long long>(executed), cpu.st.lr,
                   cpu.st.gpr[3], cpu.st.gpr[4], cpu.st.gpr[5],
                   cpu.st.gpr[29]);
            // DIAGNOSTIC (not machine truth): USBShim chain-calls the
            // prior boot-keyboard proc from [ExpandMem+0x294] with no
            // null check; the real seed comes from the USB Expert's
            // per-controller shim reference — absent while the machine
            // has no USB. Seed a bare ROM RTS so the boot can proceed
            // and reveal the next frontier. Real fix = OHCI on PCI.
            static bool poked = false;
            if (pc == 0xFFE2325Cu && !poked) {
                poked = true;
                cpu.l1dFlushAll(true);
                bus.write32(0x000116C4u, 0xFFC339A2u);
                printf("-- DIAGNOSTIC poke: [EM+294] := ffc339a2 (RTS)\n");
            }
        }
        static u32 emPrev = 0xEEEEEEEEu;
        static u32 emShown = 0;
        if (executed > 1000000000ull) {
            u32 cv = 0;
            if (!cpu.l1dPeek32(0x000116C4u, cv))
                cv = bus.read32(0x000116C4u);
            if (emPrev == 0xEEEEEEEEu) {
                emPrev = cv;
                printf("-- em+294 baseline %08x @%llu\n", cv,
                       static_cast<unsigned long long>(executed));
            }
            if (cv != emPrev && emShown < 24) {
                ++emShown;
                printf("-- em+294 %08x -> %08x @%llu pc=%08x lr=%08x "
                       "r24=%08x\n",
                       emPrev, cv,
                       static_cast<unsigned long long>(executed), cpu.st.pc,
                       cpu.st.lr, cpu.st.gpr[24]);
                printf("   ppc ring (last 16):\n");
                const u32 zc = ring.n < 16 ? ring.n : 16;
                for (u32 k = 0; k < zc; ++k) {
                    const auto& e = ring.e[(ring.n - zc + k) & 127u];
                    disassemble(e.insn, e.pc, text, sizeof text,
                                Style::Gnu);
                    printf("   %08x: %08x  %s\n", e.pc, e.insn, text);
                }
            }
            if (cv != emPrev)
                emPrev = cv;
        }
        // 68K-pc ring + sad-mac death-handler trigger (the dig's main
        // instrument). r24 = 68K pc while the emulator runs; Gossamer
        // conventions hold (D0-D7=r8-r15, A0-A7=r16-r23, r27=opcode).
        // The death handler at 68K ffc04a6e loads D6:=word[$0AF0] and
        // D7:=long[$02BA], prints them, and halts at ffc0477e. 68K
        // lowmem is per-address-space under the nanokernel, so those
        // cells must translate under the context LIVE at handler entry
        // (the Blue task) — a sampled context sees the idle task's
        // lowmem. The ring names the code that detected the failure and
        // jumped here; the 68K stack carries the vector-stub bsr return
        // address plus the exception frame with the faulting 68K pc.
        static u32 prev68k = 0, ring68At = 0;
        struct Ent68 {
            u32 pc68, op, ppc;
        };
        static Ent68 ring68[128] = {};
        static bool deathShown = false;
        if (cpu.st.gpr[24] != prev68k) {
            prev68k = cpu.st.gpr[24];
            ring68[ring68At++ & 127u] = {prev68k, cpu.st.gpr[27], pc};
            // Two triggers, first one wins: the fatal transfer itself
            // (r24 lands on 68K VA 0 — the null jump that becomes the
            // Line-F sad-mac) or, as backup, death-handler entry. The
            // former fires BEFORE the death cascade clobbers the
            // stack-hosted code that made the jump.
            if (!deathShown && executed > 1000000000ull &&
                (prev68k == 0 ||
                 (prev68k >= 0xFFC04A6Eu && prev68k <= 0xFFC04A90u)) &&
                (pc & 0xFFC00000u) == 0x68000000u) {
                deathShown = true;
                printf("-- 68K %s @%llu pc68=%08x ppcpc=%08x "
                       "lr=%08x\n",
                       prev68k == 0 ? "NULL-JUMP" : "DEATH HANDLER",
                       static_cast<unsigned long long>(executed), prev68k,
                       cpu.st.pc, cpu.st.lr);
                printf("   D0-D7: ");
                for (u32 k = 8; k < 16; ++k)
                    printf("%08x ", cpu.st.gpr[k]);
                printf("\n   A0-A7: ");
                for (u32 k = 16; k < 24; ++k)
                    printf("%08x ", cpu.st.gpr[k]);
                printf("\n   r24-r31: ");
                for (u32 k = 24; k < 32; ++k)
                    printf("%08x ", cpu.st.gpr[k]);
                printf("\n   r0-r7: ");
                for (u32 k = 0; k < 8; ++k)
                    printf("%08x ", cpu.st.gpr[k]);
                printf("\n   ctx: sdr1=%08x sr0=%08x sr1=%08x sr6=%08x "
                       "msr=%08x\n",
                       cpu.st.sdr1, cpu.st.sr[0], cpu.st.sr[1],
                       cpu.st.sr[6], cpu.st.msr);
                printf("   ppc ring (last 96):\n");
                const u32 pcnt = ring.n < 96 ? ring.n : 96;
                for (u32 k = 0; k < pcnt; ++k) {
                    const auto& e = ring.e[(ring.n - pcnt + k) & 127u];
                    disassemble(e.insn, e.pc, text, sizeof text, Style::Gnu);
                    printf("   %08x: %08x  %s\n", e.pc, e.insn, text);
                }
                cpu.l1dFlushAll(true); // bus peeks must see cached truth
                cpu.l2FlushAll(true);
                cpu.mmuProbe = true;
                const CpuState saved = cpu.st;
                const bool savedRaised = cpu.raisedThisStep;
                cpu.st.msr |= 0x30u; // translation on: live SR/BAT/PTEG
                const CpuState armed = cpu.st;
                auto xlat = [&](u32 ea, u32& pa) {
                    cpu.st = armed; // translate raises on fail; re-arm
                    const bool ok = cpu.translate(ea, false, false, pa);
                    cpu.st = armed;
                    return ok;
                };
                auto peek68 = [&](u32 a68, u32& v, u32& paOut) -> char {
                    u32 pa = 0;
                    char how = '-';
                    if (xlat(a68, pa))
                        how = 'v'; // live page tables / BATs at the EA
                    else if (a68 < 0x00400000u &&
                             xlat(0x68000000u + a68, pa))
                        how = 'b'; // the emulator's lowmem BAT window
                    paOut = pa;
                    v = 0;
                    if (how != '-' && (pa & ~3u) + 4 <= bus.ram().size())
                        v = bus.read32(pa & ~3u);
                    else if (how != '-')
                        how = 'm'; // translated to non-RAM: not read
                    return how;
                };
                u32 v = 0, pa2 = 0;
                char how = peek68(0x00000AF0u, v, pa2);
                printf("   [$0AF0] %c pa=%08x -> %08x  (major = hi word)\n",
                       how, pa2, v);
                how = peek68(0x000002B8u, v, pa2);
                printf("   [$02B8] %c pa=%08x -> %08x\n", how, pa2, v);
                how = peek68(0x000002BCu, v, pa2);
                printf("   [$02BC] %c pa=%08x -> %08x  (minor long @2BA "
                       "= 2B8.lo:2BC.hi)\n",
                       how, pa2, v);
                printf("   static pa [00F00AF0]=%08x [00004AF0]=%08x\n",
                       bus.read32(0x00F00AF0u), bus.read32(0x00004AF0u));
                auto rows = [&](const char* tag, u32 base, u32 n) {
                    printf("   %s @%08x:\n", tag, base);
                    for (u32 row = 0; row < n; row += 4) {
                        printf("   ");
                        for (u32 col = 0; col < 4; ++col) {
                            const u32 a = base + (row + col) * 4;
                            u32 vv = 0, ppa = 0;
                            const char h = peek68(a, vv, ppa);
                            printf(" [%08x]%c %08x", a, h, vv);
                        }
                        printf("\n");
                    }
                };
                rows("68K stack A7", cpu.st.gpr[23] & ~3u, 32);
                rows("A6 frame", cpu.st.gpr[22] & ~3u, 8);
                // The boot's stack-hosted code + fault stack, captured
                // before the death cascade rewrites it; and the lowmem
                // death cells ($BFF guard, $C6C/C70/C74 saves, $AF0).
                rows("stack region", 0x01DF7400u, 256);
                rows("lowmem BC0-CFF", 0x00000BC0u, 80);
                cpu.st = saved;
                cpu.raisedThisStep = savedRaised;
                cpu.mmuProbe = false;
                printf("   68k pc ring, oldest first (pc68/op; '*' = ppc "
                       "pc outside the emulator window):\n");
                for (u32 k = 0; k < 128; k += 4) {
                    printf("   ");
                    for (u32 j = 0; j < 4; ++j) {
                        const Ent68& e = ring68[(ring68At + k + j) & 127u];
                        printf(" %08x/%04x%c", e.pc68, e.op & 0xFFFFu,
                               (e.ppc & 0xFFC00000u) == 0x68000000u
                                   ? ' '
                                   : '*');
                    }
                    printf("\n");
                }
                // Capture the handler's print + park, then stop: the
                // remaining budget would only spin at the 60fe halt.
                if (maxInsns > executed + 20000000ull)
                    maxInsns = executed + 20000000ull;
            }
        }
        static int lockTrace = -1;
        static u64 spin68Last = 0;
        if (cpu.st.pc == 0x68067ECCu && executed - spin68Last > 300000000ull) {
            spin68Last = executed;
            printf("-- 68k spin @%llu r8-r15: ",
                   static_cast<unsigned long long>(executed));
            for (u32 k = 8; k < 16; ++k)
                printf("%08x ", cpu.st.gpr[k]);
            printf("\n   r16-r23: ");
            for (u32 k = 16; k < 24; ++k)
                printf("%08x ", cpu.st.gpr[k]);
            printf("\n   r24-r31: ");
            for (u32 k = 24; k < 32; ++k)
                printf("%08x ", cpu.st.gpr[k]);
            printf("\n");
        }
        if (lockTrace < 0 && cpu.st.pc == 0x00F25350u)
            lockTrace = 300;
        if (lockTrace > 0) {
            --lockTrace;
            disassemble(cpu.curInsn, pc, text, sizeof text, Style::Gnu);
            printf("-- lk %08x: %s  [r8=%08x r9=%08x r28=%08x r29=%08x]\n",
                   pc, text, cpu.st.gpr[8], cpu.st.gpr[9],
                   cpu.st.gpr[28], cpu.st.gpr[29]);
        }
        static u32 romPollShown = 0;
        if (cpu.st.pc == 0x00F12700u && romPollShown < 3 &&
            executed > 1000000000ull) {
            ++romPollShown;
            cpu.l1dFlushAll(true);
            const u32 r22 = cpu.st.gpr[22], r31 = cpu.st.gpr[31];
            printf("-- rom poll #%u @%llu r31=%08x [r31]=%08x r22=%08x "
                   "[r22-832]=%08x lr=%08x r8=%08x r9=%08x\n",
                   romPollShown, static_cast<unsigned long long>(executed),
                   r31, bus.read32(r31), r22, bus.read32(r22 - 832u),
                   cpu.st.lr, cpu.st.gpr[8], cpu.st.gpr[9]);
        }
        static u64 lastFetchSample = 0;
        if (cpu.st.pc == 0xFF80B640u &&
            executed - lastFetchSample > 20000000ull) {
            lastFetchSample = executed;
            printf("-- C@ sample @%llu addr=%08x lr=%08x\n",
                   static_cast<unsigned long long>(executed),
                   cpu.st.gpr[20], cpu.st.lr);
        }
        static u32 blinkShown = 0;
        if ((cpu.st.pc == 0xFFF82960u || cpu.st.pc == 0xFFF829D0u) &&
            blinkShown < 2) {
            ++blinkShown;
            printf("-- blink-%c @%llu code=r1&15=%u (r1=%08x) lr=%08x\n",
                   cpu.st.pc == 0xFFF82960u ? 'A' : 'B',
                   static_cast<unsigned long long>(executed),
                   cpu.st.gpr[1] & 15u, cpu.st.gpr[1], cpu.st.lr);
        }
        static bool code3Shown = false;
        if (cpu.raisedThisStep && cpu.st.pc == 0x00000700u && !code3Shown) {
            code3Shown = true;
            printf("-- first 0x700 @%llu srr0=%08x srr1=%08x lr=%08x "
                   "ctr=%08x\n",
                   static_cast<unsigned long long>(executed), cpu.st.srr0,
                   cpu.st.srr1, cpu.st.lr, cpu.st.ctr);
            printf("   r0-r7: ");
            for (u32 k = 0; k < 8; ++k)
                printf("%08x ", cpu.st.gpr[k]);
            printf("\n   r24-r31: ");
            for (u32 k = 24; k < 32; ++k)
                printf("%08x ", cpu.st.gpr[k]);
            cpu.l1dFlushAll(true);
            const u32 fr = cpu.st.gpr[2];
            printf("\n   frame[r2=%08x] +80..+9c:", fr);
            for (u32 k = 0x80; k <= 0x9C; k += 4)
                printf(" %08x", bus.read32(fr + k));
            printf("\n   ring:\n");
            const u32 cnt = ring.n < 96 ? ring.n : 96;
            for (u32 k = 0; k < cnt; ++k) {
                const auto& e = ring.e[(ring.n - cnt + k) & 127u];
                disassemble(e.insn, e.pc, text, sizeof text, Style::Gnu);
                printf("   %08x: %08x  %s\n", e.pc, e.insn, text);
            }
        }
        ++executed;
        if (cpu.raisedThisStep) {
            if (excLogged < excShow)
                printf("-- exc @%llu -> %08x from srr0=%08x srr1=%08x "
                       "dsisr=%08x dar=%08x\n",
                       static_cast<unsigned long long>(executed), cpu.st.pc,
                       cpu.st.srr0, cpu.st.srr1, cpu.st.dsisr, cpu.st.dar);
            ++excLogged;
            excRing[excRingAt % 16] = {executed, cpu.st.pc, cpu.st.srr0,
                                       cpu.st.srr1, cpu.st.dsisr, cpu.st.dar};
            ++excRingAt;
        }
    }

    if (excRingAt > static_cast<u32>(excShow)) {
        printf("-- last exceptions (of %u):\n", excRingAt);
        const u32 n = excRingAt < 16 ? excRingAt : 16;
        for (u32 k = 0; k < n; ++k) {
            const auto& e = excRing[(excRingAt - n + k) % 16];
            printf("   @%-11llu -> %08x srr0=%08x srr1=%08x dsisr=%08x "
                   "dar=%08x\n",
                   static_cast<unsigned long long>(e.at), e.vec, e.srr0,
                   e.srr1, e.dsisr, e.dar);
        }
    }

    cpu.l1dFlushAll(true);
    if (ramDumpPath) {
        FILE* f = fopen(ramDumpPath, "wb");
        if (f) {
            const size_t n = bus.ram().size() < (64u << 20)
                                 ? bus.ram().size()
                                 : size_t(64u << 20);
            fwrite(bus.ram().data(), 1, n, f);
            fclose(f);
            printf("-- ram dumped (%zu bytes): %s\n", n, ramDumpPath);
        }
    }
    printf("-- executed %llu instructions; stop pc=%08x%s\n",
           static_cast<unsigned long long>(executed), cpu.st.pc,
           cpu.halted ? " (halted)" : "");
    printf("-- msr=%08x dec=%08x hid0=%08x\n", cpu.st.msr, cpu.st.dec,
           cpu.st.hid0);
    {
        std::vector<std::pair<u64, u32>> top;
        for (const auto& [pc, n] : pcHist)
            top.push_back({n, pc});
        std::sort(top.rbegin(), top.rend());
        printf("-- hottest sampled pcs:\n");
        for (size_t k = 0; k < top.size() && k < 12; ++k)
            printf("   %08x  samples=%llu\n", top[k].second,
                   static_cast<unsigned long long>(top[k].first));
    }
    {
        printf("-- coverage timeline (%zu regions; last 32 first-entries):\n",
               firsts.size());
        const size_t start = firsts.size() > 32 ? firsts.size() - 32 : 0;
        for (size_t k = start; k < firsts.size(); ++k)
            printf("   @%-11llu %08x\n",
                   static_cast<unsigned long long>(firsts[k].first),
                   firsts[k].second);
    }
    {
        const auto& ul = bus.uninLog();
        printf("-- uni-north writes (%zu; first 40):\n", ul.size());
        for (size_t k = 0; k < ul.size() && k < 40; ++k)
            printf("   @%-11llu %08x <- %08x pc=%08x\n",
                   static_cast<unsigned long long>(ul[k].at), ul[k].pa,
                   ul[k].val, ul[k].pc);
    }
    {
        const auto& cl = bus.cd().log;
        printf("-- cd command log (%zu; c=ata p=packet e=err):\n   ",
               cl.size());
        for (size_t k = 0; k < cl.size() && k < 200; ++k)
            printf("%c%02x@%llu ", cl[k].kind, cl[k].val,
                   static_cast<unsigned long long>(cl[k].at));
        printf("\n");
    }
    {
        const auto& al = bus.ataLog();
        printf("-- ata traffic (%zu; off r/w val pc):\n", al.size());
        for (size_t k = 0; k < al.size() && k < 120; ++k)
            printf("   +%05x %c %02x pc=%08x @%llu\n", al[k].pa & ~1u,
                   (al[k].pa & 1u) ? 'r' : 'w', al[k].val & 0xFFu,
                   al[k].pc, static_cast<unsigned long long>(al[k].at));
    }
    {
        const auto& cl = bus.cfgLog();
        printf("-- pci config accesses (%zu; bus latch val pc r/w):\n",
               cl.size());
        for (size_t k = 0; k < cl.size() && k < 60; ++k)
            printf("   f%u %08x %08x pc=%08x %c @%llu\n",
                   ((cl[k].pa >> 28) & 7u) * 2u, cl[k].pa & 0x00FFFFFFu,
                   cl[k].val, cl[k].pc & ~1u, (cl[k].pc & 1u) ? 'w' : 'r',
                   static_cast<unsigned long long>(cl[k].at));
    }
    {
        const auto& il = bus.i2cLog();
        printf("-- i2c transactions (%zu; addr|sub -> byte):\n", il.size());
        for (size_t k = 0; k < il.size() && k < 140; ++k) {
            if (il[k].pa & 0x01000000u) {
                printf("   kw2 wr [%02x] <- %02x pc=%08x @%llu\n",
                       (il[k].pa >> 8) & 0xFFu, il[k].pa & 0xFFu,
                       il[k].pc, static_cast<unsigned long long>(il[k].at));
                continue;
            }
            printf("   %02x|%02x -> %s%02x pc=%08x @%llu\n",
                   (il[k].pa >> 8) & 0xFF, il[k].pa & 0xFF,
                   il[k].val == 0xFFFFFFFFu ? "nack " : "",
                   il[k].val & 0xFF, il[k].pc,
                   static_cast<unsigned long long>(il[k].at));
        }
    }
    {
        const auto& sz = bus.sizeLog();
        printf("-- sizing-window probes (%zu; r/w pa val pc):\n", sz.size());
        for (size_t k = 0; k < sz.size() && k < 120; ++k)
            printf("   %c %08x %08x pc=%08x @%llu\n",
                   (sz[k].pa & 1u) ? 'r' : 'w', sz[k].pa & ~1u, sz[k].val,
                   sz[k].pc, static_cast<unsigned long long>(sz[k].at));
    }
    {
        const std::string& con = bus.console();
        printf("-- serial console (%zu bytes):\n", con.size());
        const size_t lim = con.size() > 8192 ? 8192 : con.size();
        for (size_t k = 0; k < lim; ++k) {
            const char c = con[k];
            if (c == '\n' || c == '\r' || (c >= 0x20 && c < 0x7F))
                putchar(c == '\r' ? '\n' : c);
            else
                putchar('.');
        }
        if (!con.empty())
            putchar('\n');
        if (con.size() > 8192)
            printf("   ... (%zu more bytes)\n", con.size() - 8192);
    }
    {
        const auto& pl = bus.pmu().log;
        u64 cmdCount[256] = {};
        for (const auto& ev : pl)
            if (ev.kind == 'c')
                ++cmdCount[ev.val];
        printf("-- pmu commands seen:");
        for (u32 c = 0; c < 256; ++c)
            if (cmdCount[c])
                printf(" %02x:%llu", c,
                       static_cast<unsigned long long>(cmdCount[c]));
        printf("\n-- pmu conversation tail (of %zu events):\n   ",
               pl.size());
        const size_t start = pl.size() > 120 ? pl.size() - 120 : 0;
        for (size_t k = start; k < pl.size(); ++k)
            printf("%c%02x ", pl[k].kind, pl[k].val);
        printf("\n");
    }
    {
        const auto& kl = bus.macioLog();
        printf("-- keylargo first-touch log (%zu offsets):\n", kl.size());
        size_t shown = 0;
        for (u32 pa : bus.macioOrder) {
            if (++shown > 600) {
                printf("   ... %zu more\n", kl.size() - 100);
                break;
            }
            const auto& a = kl.at(pa);
            printf("   %08x  @%-11llu pc=%08x reads=%llu writes=%llu%s",
                   pa, static_cast<unsigned long long>(a.firstAt), a.firstPc,
                   static_cast<unsigned long long>(a.reads),
                   static_cast<unsigned long long>(a.writes),
                   a.writes ? " lastWr=" : "\n");
            if (a.writes)
                printf("%08x\n", a.lastWr);
        }
    }
    {
        const auto& log = bus.accessLog();
        printf("-- unclaimed/rom-write access log (%zu addresses, "
               "first-touch order):\n",
               log.size());
        size_t shown = 0;
        for (u32 pa : bus.logOrder) {
            if (++shown > 120) {
                printf("   ... %zu more\n", log.size() - 120);
                break;
            }
            const auto& a = log.at(pa);
            printf("   %08x  @%-11llu pc=%08x reads=%llu writes=%llu%s",
                   pa, static_cast<unsigned long long>(a.firstAt), a.firstPc,
                   static_cast<unsigned long long>(a.reads),
                   static_cast<unsigned long long>(a.writes),
                   a.writes ? " lastWr=" : "\n");
            if (a.writes)
                printf("%08x\n", a.lastWr);
        }
    }
    printf("-- last instructions:\n");
    const u32 cnt = ring.n < 32 ? ring.n : 32;
    for (u32 k = 0; k < cnt; ++k) {
        const auto& e = ring.e[(ring.n - cnt + k) & 127u];
        disassemble(e.insn, e.pc, text, sizeof text, Style::Gnu);
        printf("   %08x: %08x  %s\n", e.pc, e.insn, text);
    }
    return 0;
}
