// g4run — Sawtooth (Power Mac G4 AGP) machine runner. Loads the 1 MB New
// World boot ROM at 0xFFF00000 and executes from the hardware reset vector
// with authentic reset state. M-SAW-0 deliverable is the instrumentation:
// how far Open Firmware 3.x gets, which physical addresses it touches
// (the deduplicated unclaimed-access log IS the Uni-North/KeyLargo map),
// the exceptions it takes, and the last instructions before the stop.

#include "opm/cpu.hpp"
#include "opm/insn.hpp"
#include "opm/sawtooth.hpp"
#include "opm/snapshot.hpp"

#include <algorithm>
#include <chrono>
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

// Toolbox trap names, for the ones this dig actually meets. A trace line
// reading "$A03D" costs a lookup every time it is read; one reading
// "_DrvrInstall" does not — and _DrvrInstall and _AddDrive are precisely the
// two calls whose absence is the whole mount story.
const char* trapName(u32 t)
{
    switch (t & 0x0FFFu) {
    case 0x00E: return "_LoadSeg";
    case 0x01F: return "_DisposePtr";
    case 0x022: return "_NewHandle";
    case 0x023: return "_DisposeHandle";
    case 0x029: return "_HLock";
    case 0x02E: return "_BlockMove";
    case 0x03D: return "_DrvrInstall";     // never seen: see the mount dig
    case 0x04E: return "_AddDrive";        // never seen: see the mount dig
    case 0x055: return "_StripAddress";
    case 0x098: return "_HWPriv";
    case 0x11E: return "_NewPtr";
    case 0x122: return "_NewHandleClear";
    case 0x128: return "_RecoverHandle";
    case 0x146: return "_GetTrapAddress";
    case 0x147: return "_SetTrapAddress";
    case 0x14D: return "_SetToolTrapAddress";
    case 0x1AD: return "_Gestalt";
    case 0x71E: return "_NewPtrSysClear";
    case 0x746: return "_GetToolTrapAddress";
    case 0x976: return "_MixedModeDispatch";
    case 0xAF1: return "_ATAMgr";
    default: return "";
    }
}

// A machine-readable event stream beside the human one. Most analysis this
// session was ad-hoc awk over a hundred thousand lines of prose, which is
// slow to write and easy to mis-scope — "every manager completion between
// the driver call and its answer" took three attempts. One JSON object per
// event makes that a filter rather than a parser.
struct Events {
    FILE* f = nullptr;
    void open(const char* path)
    {
        if (path)
            f = fopen(path, "wb");
    }
    void close()
    {
        if (f)
            fclose(f);
        f = nullptr;
    }
    // kind plus a pre-formatted body of "key":value pairs.
    void emit(u64 at, const char* kind, const char* body)
    {
        if (!f)
            return;
        fprintf(f, "{\"at\":%llu,\"kind\":\"%s\"%s%s}\n",
                static_cast<unsigned long long>(at), kind,
                body && *body ? "," : "", body ? body : "");
    }
};

// Classic Mac code is self-describing: a MacsBug symbol — a length byte with
// the high bit set, then the name — sits just past each function's RTS. So
// the Mac OS ROM, once loaded into RAM, carries its own symbol table.
// tools/symbolize.sh has been rewriting logs with it after the fact; doing it
// in-process means every trace line names a routine as it is printed, which
// is the difference between reading a dig and decoding one.
//
// Built lazily: the ROM is not in RAM until Trampoline has run, so an early
// attempt finds nothing and is retried later rather than cached as empty.
struct SymTab {
    std::vector<std::pair<u32, std::string>> syms;
    bool built = false;
    u64 lastTry = 0;

    void build(const std::vector<u8>& ram, u32 paBase, u32 vaBase, u32 len)
    {
        syms.clear();
        if (static_cast<size_t>(paBase) + len > ram.size())
            return;
        auto idChar = [](u8 c, bool head) {
            return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                   c == '_' ||
                   (!head && ((c >= '0' && c <= '9') || c == '.'));
        };
        for (u32 k = 1; k < len; ++k) {
            // A MacsBug name is preceded by a byte with the high bit set.
            // The low bits carry the length in the common form, but Apple
            // also emits 0x80/0x81 with the length in the FOLLOWING byte, so
            // matching on the length field alone silently finds nothing.
            // Take the marker as the signal and measure the name itself.
            if (ram[paBase + k - 1] < 0x80u || !idChar(ram[paBase + k], true))
                continue;
            u32 n = 1;
            while (n < 32 && k + n < len && idChar(ram[paBase + k + n], false))
                ++n;
            if (n < 4)
                continue;
            syms.emplace_back(vaBase + k,
                              std::string(reinterpret_cast<const char*>(
                                              ram.data() + paBase + k),
                                          n));
            k += n;
        }
        std::sort(syms.begin(), syms.end());
        built = syms.size() > 200; // a handful of hits is noise, not a table
    }

    // Rotating buffers: a printf commonly symbolizes more than one address.
    const char* at(u32 a)
    {
        static char buf[6][64];
        static u32 turn = 0;
        char* out = buf[turn++ % 6u];
        if (!built || syms.empty() || a < syms.front().first) {
            out[0] = 0;
            return out;
        }
        size_t lo = 0, hi = syms.size() - 1, best = 0;
        while (lo <= hi) {
            const size_t mid = (lo + hi) / 2;
            if (syms[mid].first <= a) {
                best = mid;
                lo = mid + 1;
            } else {
                if (mid == 0)
                    break;
                hi = mid - 1;
            }
        }
        const u32 off = a - syms[best].first;
        if (off > 0x4000u) { // too far to be inside that routine
            out[0] = 0;
            return out;
        }
        snprintf(out, 64, "<%s+0x%x>", syms[best].second.c_str(), off);
        return out;
    }
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
    const char* hdPath = nullptr;
    const char* serialInput = nullptr; // ';' separates lines
    u64 serialAt = 240000000ull;       // inject once the prompt is up
    u32 viaA = 0x00;                   // VIA port A strap levels
    const char* atiRomPath = nullptr;  // Rage 128 FCode expansion ROM
    u64 atiAt = 0;                     // hide the card until this insn
    u32 watchMemPa = 0, watchMemEnd = 0;  // --watch-mem [--watch-mem-end]
    u32 stackAt = 0;                   // --stack-at ADDR: 68K backchain dump
    u32 watchReg = 99;                 // --watch-reg N: value-origin watch
    u32 watchVal = 0;                  // --watch-val V
    bool armOnValue = false;           // --arm-on-value: arm when the
                                       // watched register hits its value
    u32 armAtPark = 0;                 // --arm-at-park N: condition gate
    u32 parkSeen = 0;
    bool parkArmed = false;
    u32 tracePass = 0;                 // --trace-pass N: only this kick pass
    u64 ataPokeAt = 0;                 // hand-enqueue an .ATALoad request
    u32 ataPokeDev = 0;                // its deviceID field (entry+4)
    u32 ataPokeKind = 1;               // its kind field (entry+8: 1 or 3)
    u64 snapAt = 0;                    // --snapshot-at N
    const char* snapOut = nullptr;     // --snapshot-out FILE
    const char* resumeFrom = nullptr;  // --resume-from FILE
    u64 verifyAt = 0, verifySteps = 0; // --verify-snapshot N M
    bool fastTbSet = false;            // CLI wins over a resumed value
    bool realtime = false;             // --realtime: TB from the host clock
    u32 callLo = 0, callHi = 0;        // --call-trace LO HI
    u64 callFrom = 0;                  // --call-trace-at N: gate
    u32 t68Lo = 0, t68Hi = 0;          // --trace-68k LO HI
    u32 t68Cap = 4000;                 // --trace-68k-lines N
    u64 watchFrom = 0;                 // --watch-from N: value-watch gate
    u32 armOnPc = 0;                   // --arm-on-pc ADDR: arm on arrival
    u64 dumpStructsAt = 0;             // --dump-structs-at N
    bool dumpStructsEnd = false;       // --dump-structs
    const char* eventsPath = nullptr;  // --events FILE (JSONL)
    u32 watchVa = 0;                   // --watch-va ADDR
    u64 traceFrom = 0;                 // --trace-from N: full trace window
    u64 traceLines = 2000;             // --trace-lines M

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
        else if (!strcmp(a, "--fast-tb")) {
            fastTb = static_cast<u32>(strtoul(next(), nullptr, 0));
            fastTbSet = true;
        }
        else if (!strcmp(a, "--fast-tb-until"))
            fastTbUntil = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--dump-ram")) ramDumpPath = next();
        else if (!strcmp(a, "--serial-cr")) serialCr = true;
        else if (!strcmp(a, "--serial-input")) serialInput = next();
        else if (!strcmp(a, "--serial-at"))
            serialAt = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--cd")) cdPath = next();
        else if (!strcmp(a, "--hd")) hdPath = next();
        else if (!strcmp(a, "--via-a")) viaA = strtoul(next(), nullptr, 0);
        else if (!strcmp(a, "--ati-rom")) atiRomPath = next();
        else if (!strcmp(a, "--ati-at"))
            atiAt = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--watch-mem"))
            watchMemPa = static_cast<u32>(strtoul(next(), nullptr, 0));
        else if (!strcmp(a, "--watch-mem-end"))
            watchMemEnd = static_cast<u32>(strtoul(next(), nullptr, 0));
        else if (!strcmp(a, "--stack-at"))
            stackAt = static_cast<u32>(strtoul(next(), nullptr, 0));
        else if (!strcmp(a, "--watch-reg"))
            watchReg = static_cast<u32>(strtoul(next(), nullptr, 0));
        else if (!strcmp(a, "--watch-val"))
            watchVal = static_cast<u32>(strtoul(next(), nullptr, 0));
        else if (!strcmp(a, "--arm-on-value")) armOnValue = true;
        else if (!strcmp(a, "--arm-at-park"))
            armAtPark = static_cast<u32>(strtoul(next(), nullptr, 0));
        else if (!strcmp(a, "--trace-pass"))
            tracePass = static_cast<u32>(strtoul(next(), nullptr, 0));
        else if (!strcmp(a, "--ata-poke"))
            ataPokeAt = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--ata-poke-dev"))
            ataPokeDev = static_cast<u32>(strtoul(next(), nullptr, 0));
        else if (!strcmp(a, "--ata-poke-kind"))
            ataPokeKind = static_cast<u32>(strtoul(next(), nullptr, 0));
        else if (!strcmp(a, "--snapshot-at"))
            snapAt = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--snapshot-out")) snapOut = next();
        else if (!strcmp(a, "--resume-from")) resumeFrom = next();
        else if (!strcmp(a, "--verify-snapshot")) {
            verifyAt = strtoull(next(), nullptr, 0);
            verifySteps = strtoull(next(), nullptr, 0);
        }
        else if (!strcmp(a, "--realtime")) realtime = true;
        else if (!strcmp(a, "--call-trace")) {
            callLo = static_cast<u32>(strtoul(next(), nullptr, 0));
            callHi = static_cast<u32>(strtoul(next(), nullptr, 0));
        }
        else if (!strcmp(a, "--call-trace-at"))
            callFrom = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--trace-68k")) {
            t68Lo = static_cast<u32>(strtoul(next(), nullptr, 0));
            t68Hi = static_cast<u32>(strtoul(next(), nullptr, 0));
        }
        else if (!strcmp(a, "--trace-68k-lines"))
            t68Cap = static_cast<u32>(strtoul(next(), nullptr, 0));
        else if (!strcmp(a, "--watch-from"))
            watchFrom = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--arm-on-pc"))
            armOnPc = static_cast<u32>(strtoul(next(), nullptr, 0));
        else if (!strcmp(a, "--dump-structs")) dumpStructsEnd = true;
        else if (!strcmp(a, "--dump-structs-at"))
            dumpStructsAt = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--events")) eventsPath = next();
        else if (!strcmp(a, "--trace-from"))
            traceFrom = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--trace-lines"))
            traceLines = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--watch-va"))
            watchVa = static_cast<u32>(strtoul(next(), nullptr, 0));
        else {
            fprintf(stderr,
                    "usage: g4run --rom FILE [--ram MB] [--max N] [--trace] "
                    "[--exc N] [--dis A B]\n"
                    "       snapshots: --snapshot-at N --snapshot-out FILE | "
                    "--resume-from FILE\n"
                    "       validation: --verify-snapshot N M  (run to N, "
                    "snapshot, run M, restore,\n"
                    "                   run M again, compare "
                    "instruction-for-instruction)\n");
            return 2;
        }
    }
    if (!romPath) {
        fprintf(stderr, "g4run: --rom is required\n");
        return 2;
    }

    // Run manifest. A log found on disk a day later has to be able to say
    // what produced it: which flags, which images, which limits. More than
    // one conclusion this session had to be re-derived because a log could
    // not be matched to its command line.
    printf("-- g4run manifest:");
    for (int i = 1; i < argc; ++i)
        printf(" %s", argv[i]);
    printf("\n");

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

    if (atiRomPath) {
        if (bus.attachAtiRom(atiRomPath))
            printf("-- ati fcode rom attached: %s\n", atiRomPath);
        else
            printf("-- ati rom attach FAILED: %s\n", atiRomPath);
    }
    bus.watchPa = watchMemPa;
    bus.watchPaEnd = watchMemEnd ? watchMemEnd : watchMemPa;
    if (hdPath) {
        if (bus.attachHd(hdPath))
            printf("-- hd attached: %s (present=%d)\n", hdPath,
                   bus.hd().present() ? 1 : 0);
        else
            printf("-- hd attach FAILED: %s\n", hdPath);
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
    // Instrument census. A watch that emits nothing is ambiguous between
    // "never fired", "fired with nothing to say", and "its gate never
    // opened" — this session lost several runs to exactly that, because a
    // hardware fix moved the boot 830M instructions earlier and every
    // instrument gated on a hard-coded instruction count silently armed
    // after the window it meant to watch. Counting hits and reporting them
    // at the end makes a silent instrument visibly silent.
    struct Cen {
        const char* name;
        u64 hits;
    };
    Cen cen[] = {{"ADDBUS", 0},  {"ATAFN", 0}, {"ATARES", 0},
                 {"LK", 0},      {"SCMP", 0},  {"QSEL", 0},
                 {"CTL", 0},     {"DRV", 0},   {"ATAPOKE", 0},
                 {"68K-gate", 0}};
    enum {
        kCenAddbus, kCenAtafn, kCenAtares, kCenLk, kCenScmp,
        kCenQsel, kCenCtl, kCenDrv, kCenPoke, kCen68kGate
    };
    bus.pcRef = &cpu.st.pc;
    bus.stamp = &executed;
    bus.cd().stamp = &executed;
    bus.hd().stamp = &executed;
    bus.pic().stamp = &executed;
    bus.pmu().tbRef = &cpu.st.tb; // VIA time = TB/32 (real clock ratio)
    bus.pmu().portAIn = static_cast<u8>(viaA);
    for (u32 f = 0; f < 2; ++f) {
        bus.ohci(f).stamp = &executed;
        bus.ohci(f).pcRef = &cpu.st.pc;
    }
    bus.ati().stamp = &executed;
    bus.ati().pcRef = &cpu.st.pc;
    bus.atiVisibleAt = atiAt;
    bus.ataDma().stamp = &executed;
    bus.ataDma().pcRef = &cpu.st.pc;

    // One-shot diagnostic pokes. These are the only instruments that write
    // guest memory, so unlike every other counter here they are snapshot
    // state: a resume that re-fired them would poke a machine that has
    // already moved past the poke.
    bool ataPoked = false, emPoked = false;

    if (resumeFrom) {
        std::vector<u8> blob;
        if (!readSnapshotFile(resumeFrom, blob))
            return 2;
        HarnessState h;
        SnapReader r(blob.data(), blob.size());
        if (!loadSnapshot(cpu, bus, h, r)) {
            fprintf(stderr, "g4run: %s is not usable: %s\n", resumeFrom,
                    r.err.c_str());
            return 2;
        }
        executed = h.executed;
        parkSeen = h.parkSeen;
        parkArmed = h.parkArmed;
        ataPoked = h.ataPoked;
        emPoked = h.emPoked;
        // Timebase compression is a harness lever, not machine state: an
        // explicit flag on the resume command line wins, silence adopts
        // what the snapshot was running with.
        if (!fastTbSet) {
            fastTb = h.fastTb;
            fastTbUntil = h.fastTbUntil;
        } else if (fastTb != h.fastTb) {
            printf("-- note: snapshot ran with --fast-tb %u, this run uses "
                   "%u\n",
                   h.fastTb, fastTb);
        }
        // Instruments are re-armed from the command line, never resumed.
        bus.watchPa = watchMemPa;
        bus.watchPaEnd = watchMemEnd ? watchMemEnd : watchMemPa;
        bus.watchHits = 0;
        printf("-- resumed from %s @%llu insns: pc=%08x msr=%08x tb=%llu "
               "park=%s fingerprint=%016llx\n",
               resumeFrom, static_cast<unsigned long long>(executed),
               cpu.st.pc, cpu.st.msr,
               static_cast<unsigned long long>(cpu.st.tb),
               parkArmed ? "armed" : "not armed",
               static_cast<unsigned long long>(
                   snapshotFingerprint(cpu, bus, h)));
        if (hdPath)
            printf("-- note: --hd is a WRITABLE image and lives outside the "
                   "snapshot; a longer earlier run may already have written "
                   "past this point\n");
        if (maxInsns <= executed)
            printf("-- note: --max %llu is already behind the resumed "
                   "position; nothing will run\n",
                   static_cast<unsigned long long>(maxInsns));
    }
    if (verifyAt && realtime) {
        fprintf(stderr, "g4run: --verify-snapshot needs a reproducible run; "
                        "--realtime is nondeterministic by construction\n");
        return 2;
    }
    if (verifyAt) {
        // Validation drives the machine itself, so it owns the clock.
        maxInsns = verifyAt;
        printf("-- verify-snapshot: run to %llu, snapshot, run %llu, "
               "restore, run %llu again\n",
               static_cast<unsigned long long>(verifyAt),
               static_cast<unsigned long long>(verifySteps),
               static_cast<unsigned long long>(verifySteps));
    }

    // The per-step machine advance, shared by the instrumented run loop and
    // the bare loops the validator uses. Sharing it is the point: a
    // validator that advanced the machine even slightly differently would
    // prove nothing about the run it is meant to certify.
    // --realtime: the timebase follows the HOST CLOCK instead of the
    // instruction count. Cpu::step ticks once per instruction against
    // cyclesPerTbTick = 4, so TB advances one tick per four instructions —
    // roughly 3M/s at our speed against a real Sawtooth's 25 MHz, which is
    // why guest time runs about eight times slow and why --fast-tb, which
    // multiplies the same instruction-derived rate, overshoots into a
    // decrementer storm instead of fixing it.
    //
    // Instruction pacing stays the DEFAULT and is untouched: the snapshot
    // proof, every A/B run comparison and the whole debugging method depend
    // on runs being reproducible, and a wall clock is nondeterministic by
    // construction. Real time is for using the machine; instruction time is
    // for reasoning about it.
    const auto hostStart = std::chrono::steady_clock::now();
    auto rtBase = hostStart;
    u64 rtTbBase = 0, rtSlips = 0;
    constexpr u64 kRtNsPerTick = 40;   // 25 MHz = bus/4
    constexpr u64 kRtCatchup = 25000;  // at most 1 ms of debt per sample
    auto tickPeripherals = [&]() {
        if (realtime) {
            if ((executed & 0x3FFu) == 0) {
                const auto now = std::chrono::steady_clock::now();
                const u64 ns =
                    static_cast<u64>(std::chrono::duration_cast<
                                         std::chrono::nanoseconds>(
                                         now - rtBase)
                                         .count());
                const u64 want = rtTbBase + ns / kRtNsPerTick;
                if (want > cpu.st.tb) {
                    u64 delta = want - cpu.st.tb;
                    if (delta > kRtCatchup) {
                        // A host stall, or simply a stretch we could not
                        // keep up with. Take the cap and forgive the rest:
                        // injecting the whole debt would fire a burst of
                        // decrementer interrupts that models nothing.
                        delta = kRtCatchup;
                        rtBase = now;
                        rtTbBase = cpu.st.tb + delta;
                        ++rtSlips;
                    }
                    cpu.tick(static_cast<u32>(delta * cpu.cyclesPerTbTick));
                }
            }
        } else if (fastTb && executed < fastTbUntil) {
            cpu.tick(fastTb);
        }
        bus.ohciTick(cpu.st.tb);
        bus.syncIrqs();
        cpu.setExternalIrq(bus.pic().cpuLine());
    };

    // Symbolize on demand. The table cannot exist until the Mac OS ROM is in
    // RAM, so building it is retried rather than cached empty.
    SymTab symtab;
    auto sym = [&](u32 a) -> const char* {
        if (!symtab.built && executed > 100000000ull &&
            executed - symtab.lastTry > 100000000ull) {
            symtab.lastTry = executed;
            cpu.l1dFlushAll(true);
            cpu.l2FlushAll(true);
            symtab.build(bus.ram(), 0x00C00000u, 0xFFC00000u, 0x00200000u);
            if (symtab.built)
                printf("-- symbols: %zu MacsBug names from the ROM in RAM "
                       "@%llu\n",
                       symtab.syms.size(),
                       static_cast<unsigned long long>(executed));
        }
        return symtab.at(a);
    };

    // ONE guest reader. Six ad-hoc copies of this dance had accumulated —
    // flush the caches so dirty lines are visible, set mmuProbe so the walk
    // has no side effects, save and restore the CPU state around a
    // translation that raises on failure. Six copies means six chances to
    // forget one of the four steps, and forgetting the flush reads stale
    // memory that looks like real evidence.
    auto guest = [&](u32 ea, u32 len) -> long long {
        cpu.l1dFlushAll(true);
        cpu.l2FlushAll(true);
        cpu.mmuProbe = true;
        const CpuState saved = cpu.st;
        const bool raised = cpu.raisedThisStep;
        cpu.st.msr |= 0x30u;
        const CpuState armed = cpu.st;
        long long out = -1;
        u32 v = 0;
        bool ok = true;
        for (u32 k = 0; k < len && ok; ++k) {
            u32 pa = 0;
            cpu.st = armed;
            ok = cpu.translate(ea + k, false, false, pa);
            cpu.st = armed;
            if (!ok || pa >= bus.ram().size()) {
                ok = false;
                break;
            }
            v = (v << 8) | static_cast<u8>(bus.read32(pa & ~3u) >>
                                           (8 * (3 - (pa & 3u))));
        }
        if (ok)
            out = static_cast<long long>(v);
        cpu.st = saved;
        cpu.raisedThisStep = raised;
        cpu.mmuProbe = false;
        return out; // -1 when the address does not translate to RAM
    };

    // Structure dumpers. "Is the drive queue empty?" and "which drivers are
    // installed?" have been answered all session by reading raw hex out of
    // a RAM dump and counting offsets by hand. A queue is a shape; print
    // the shape.
    auto dumpStructs = [&](const char* why) {
        printf("== guest structures (%s) @%llu\n", why,
               static_cast<unsigned long long>(executed));
        // The Blue task's 68K lowmem lives at PHYSICAL 0x4000: lowmem 0 is
        // PA 0x4000, so $308 is PA 0x4308 — which is exactly what the drive
        // queue detector has been reading all along. Going through the MMU
        // instead reads whichever address space happens to be current, and
        // that is how this dumper first printed a queue head of 43a67c28 in
        // a 64 MB machine.
        cpu.l1dFlushAll(true);
        cpu.l2FlushAll(true);
        auto pa32 = [&](u32 pa) -> long long {
            return pa + 4 <= bus.ram().size()
                       ? static_cast<long long>(bus.read32(pa))
                       : -1;
        };
        auto pa16 = [&](u32 pa) -> long long {
            return pa + 2 <= bus.ram().size()
                       ? static_cast<long long>(bus.read32(pa & ~3u) >>
                                                (16 - 8 * (pa & 2u)) &
                                                0xFFFFu)
                       : -1;
        };
        // Every 68K address needs the same rebase, not just the lowmem
        // globals: a queue element or a unit-table entry is a 68K pointer
        // too, and reading one as a raw physical address lands in the RAM
        // junk fill and prints 64 confident garbage handles.
        auto b32 = [&](u32 a68) { return pa32(0x4000u + a68); };
        auto b16 = [&](u32 a68) { return pa16(0x4000u + a68); };
        auto lm32 = b32;
        auto lm16 = b16;

        // 68K lowmem is PER ADDRESS SPACE under the nanokernel, so these
        // cells only mean anything while the Blue task's context is live.
        // Read at an arbitrary moment they translate to somebody else's
        // memory and print confident nonsense — a queue head of 43a67c28
        // in a 64 MB machine, a unit table at 409e2ee4. Check the shape
        // before believing the contents, and say so rather than printing
        // numbers that will be quoted later as measurements.
        const long long utProbe = lm32(0x011C), unProbe = lm16(0x01D2);
        const bool sane = utProbe > 0x400 && utProbe < 0x04000000 &&
                          unProbe >= 16 && unProbe <= 256;
        if (!sane) {
            printf("   NOT IN THE 68K WORLD: UTableBase=%08llx count=%lld "
                   "are not plausible, so lowmem is another address space "
                   "right now.\n"
                   "   Re-run with --dump-structs-at N for an N inside the "
                   "Blue task (the park qualifies).\n",
                   utProbe & 0xFFFFFFFF, unProbe & 0xFFFF);
            fflush(stdout);
            return;
        }

        // Drive queue, lowmem $308: a QHdr {flags, qHead, qTail}. Each
        // element is the thing the Start Manager is waiting for.
        const long long qf = lm16(0x0308), qh = lm32(0x030A),
                        qt = lm32(0x030E);
        printf("   DrvQHdr $308: flags=%04llx head=%08llx tail=%08llx\n",
               qf & 0xFFFF, qh & 0xFFFFFFFF, qt & 0xFFFFFFFF);
        u32 el = static_cast<u32>(qh);
        for (int n = 0; n < 8 && el && qh > 0; ++n) {
            // A drive queue element is addressed at its qLink; the four
            // bytes before it are dQDrvSz/flags.
            const long long link = b32(el);
            const long long drv = b16(el + 6);
            const long long ref = b16(el + 8);
            const long long fsid = b16(el + 10);
            printf("     drive[%d] @%08x dQDrive=%lld dQRefNum=%lld "
                   "dQFSID=%04llx next=%08llx\n",
                   n, el, drv, static_cast<long long>(static_cast<i16>(ref)),
                   fsid & 0xFFFF, link & 0xFFFFFFFF);
            if (link <= 0)
                break;
            el = static_cast<u32>(link);
        }

        // Unit table: UTableBase $11C, UnitNtryCnt $1D2. Which DRVRs exist,
        // and at which refNum — .ATALoad is unit 50 / refNum -51.
        const long long ut = lm32(0x011C), un = lm16(0x01D2);
        printf("   UTableBase=%08llx entries=%lld (non-null:", ut & 0xFFFFFFFF,
               un & 0xFFFF);
        if (ut > 0)
            for (u32 k = 0; k < (un > 0 && un < 128 ? u32(un) : 64u); ++k) {
                const long long dce = b32(static_cast<u32>(ut) + k * 4);
                if (dce > 0)
                    printf(" %u=%08llx", k, dce & 0xFFFFFFFF);
            }
        printf(")\n");

        // The ATA Manager's own lists. r4/r5 at every manager call is the
        // globals pointer; +0x38 is the device list, +0x44 the registered
        // drivers that fn 0x98 enumerates.
        // The manager's globals pointer is a NATIVE address (r4/r5 at every
        // manager call), so it takes neither the 68K rebase nor a raw
        // physical read — it needs the native context that owns it. Print
        // the two list heads only when they look like list heads; a
        // "driver[0] refNum=-1 proc=ffffffff" assembled out of junk is
        // worse than no line at all, because it looks like a finding.
        const u32 g = 0x00067BF0u;
        const long long dev = pa32(g + 0x38), drv = pa32(g + 0x44);
        const bool ataSane = dev >= 0 && drv >= 0 &&
                             dev < static_cast<long long>(bus.ram().size()) &&
                             drv < static_cast<long long>(bus.ram().size());
        if (!ataSane) {
            printf("   ATA globals %08x: not readable from here "
                   "(+38=%08llx +44=%08llx are not plausible list heads; "
                   "this pointer lives in the native context)\n",
                   g, dev & 0xFFFFFFFF, drv & 0xFFFFFFFF);
            fflush(stdout);
            return;
        }
        printf("   ATA globals %08x: devices@+38=%08llx drivers@+44=%08llx\n",
               g, dev & 0xFFFFFFFF, drv & 0xFFFFFFFF);
        u32 node = static_cast<u32>(dev > 0 ? dev : 0);
        for (int n = 0; n < 8 && node; ++n) {
            const long long nx = pa32(node);
            const long long id = pa16(node + 0x0C);
            printf("     device[%d] @%08x id=%04llx next=%08llx\n", n, node,
                   id & 0xFFFF, nx & 0xFFFFFFFF);
            if (nx <= 0)
                break;
            node = static_cast<u32>(nx);
        }
        node = static_cast<u32>(drv > 0 ? drv : 0);
        for (int n = 0; n < 8 && node; ++n) {
            const long long nx = pa32(node);
            const long long ref = pa16(node + 4);
            const long long proc = pa32(node + 6);
            printf("     driver[%d] @%08x refNum=%lld proc=%08llx "
                   "next=%08llx\n",
                   n, node, static_cast<long long>(static_cast<i16>(ref)),
                   proc & 0xFFFFFFFF, nx & 0xFFFFFFFF);
            if (nx <= 0)
                break;
            node = static_cast<u32>(nx);
        }
        fflush(stdout);
    };

    Events evlog;
    evlog.open(eventsPath);
    if (eventsPath)
        printf("-- events: %s (one JSON object per line)\n", eventsPath);
    char evb[512];

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
    std::map<u32, u64> delayCallers; // DelayFor entry: lr census — what
                                     // is the parked boot waiting on?
    struct DfTrace {
        u64 at;
        u32 durHi, durLo;
        u32 caller[6];
    };
    DfTrace dfRing[12] = {};
    u32 dfRingAt = 0; // ring: the LAST traces always cover the park
    std::map<u32, u64> pc68Hist; // 68K-pc (r24) census inside the
                                 // emulator: names the 68K busy loop

    while (executed < maxInsns && !cpu.halted) {
        const u32 pc = cpu.st.pc;
        // Snapshot at an instruction boundary, BEFORE the step: `executed`
        // counts completed instructions and pc is the next one, so a resume
        // re-enters this loop in exactly the state the write saw.
        if (snapAt && executed == snapAt) {
            HarnessState h{executed, fastTb,   fastTbUntil, parkSeen,
                           parkArmed, ataPoked, emPoked};
            SnapWriter w;
            saveSnapshot(cpu, bus, h, w);
            if (snapOut && writeSnapshotFile(snapOut, w.buf))
                printf("-- snapshot written @%llu: %s (%zu bytes, "
                       "fingerprint=%016llx)\n",
                       static_cast<unsigned long long>(executed), snapOut,
                       w.buf.size(),
                       static_cast<unsigned long long>(
                           snapshotFingerprint(cpu, bus, h)));
            else if (!snapOut)
                printf("-- --snapshot-at given without --snapshot-out; "
                       "nothing written\n");
            fflush(stdout);
        }
        cpu.step();
        // kATAMgrAddATABus (fn 0x93) fails with -56, and its worker
        // ffdd18d0 is the ONLY thing that populates globals+0x44 — the
        // list whose emptiness makes fn 0x98 answer -56 for the rest of
        // the boot. The failure is one of two checks at its top:
        //   dd18e8 lwz r3,48(r29)  the bus/AIM parameter out of the PB
        //   dd18f0 bl  0xdd3b18    check 1 -> non-zero takes the error path
        //   dd190c bl  0xdd1834    check 2 -> non-zero IS the returned error
        if (pc == 0xFFDD18E8u || pc == 0xFFDD18F8u || pc == 0xFFDD1910u) {
            ++cen[kCenAddbus].hits;
            static int ab = 0;
            if (ab < 40) {
                ++ab;
                if (pc == 0xFFDD18E8u) {
                    printf("ADDBUS enter pb=%08x @%llu\n", cpu.st.gpr[29],
                           static_cast<unsigned long long>(executed));
                    // Who registers the bus? The parameter block this
                    // worker is handed carries a NULL at +0x3E, so the
                    // bus's registered-driver list is created empty and
                    // every later lookup answers -56. That makes the
                    // CALLER the whole remaining question, and a param
                    // block on its own does not name one. Walk the native
                    // backchain (PowerOpen linkage: saved LR at +8 of the
                    // caller's frame) and report the 68K pc too, since the
                    // manager is reachable from both worlds.
                    printf("   lr=%08x r24=%08x sp=%08x chain:", cpu.st.lr,
                           cpu.st.gpr[24], cpu.st.gpr[1]);
                    cpu.l1dFlushAll(true);
                    cpu.l2FlushAll(true);
                    cpu.mmuProbe = true;
                    const CpuState abSaved = cpu.st;
                    const bool abRaised = cpu.raisedThisStep;
                    const CpuState abArmed = cpu.st;
                    auto abRead = [&](u32 ea, u32& v) {
                        u32 pa = 0;
                        cpu.st = abArmed;
                        const bool ok = cpu.translate(ea, false, false, pa);
                        cpu.st = abArmed;
                        if (!ok || (pa & ~3u) + 4 > bus.ram().size())
                            return false;
                        v = bus.read32(pa & ~3u);
                        return true;
                    };
                    u32 sp = cpu.st.gpr[1];
                    for (u32 f = 0; f < 8 && sp; ++f) {
                        u32 bc = 0, slr = 0;
                        if (!abRead(sp, bc) || !bc || bc <= sp)
                            break;
                        if (abRead(bc + 8, slr))
                            printf(" %08x%s", slr, sym(slr));
                        sp = bc;
                    }
                    printf("\n");
                    cpu.st = abSaved;
                    cpu.raisedThisStep = abRaised;
                    cpu.mmuProbe = false;
                    fflush(stdout);
                }
                else
                    printf("ADDBUS %s -> %d\n",
                           pc == 0xFFDD18F8u ? "check1(dd3b18)"
                                             : "check2(dd1834)",
                           static_cast<int>(cpu.st.gpr[3]));
                fflush(stdout);
            }
        }
        // Generic "who calls this?" — an entry edge into an address range.
        // Every time this question came up it was answered by hand-rolling
        // a backchain walk at one specific site, which costs a build and a
        // run each time and only ever answers once. Detecting the transfer
        // itself is both simpler and more general: if the new pc is inside
        // the window and the old one was not, control just entered it, and
        // the OLD pc is the call site whatever instruction form got there —
        // bl, bctrl, a jump table, or a 68K trap thunk. LR and the first
        // three argument registers come along, since "who called" is nearly
        // always followed by "with what".
        if (callLo && cpu.st.pc >= callLo && cpu.st.pc < callHi &&
            !(pc >= callLo && pc < callHi) && executed >= callFrom) {
            static int ct = 0;
            if (ct < 200) {
                ++ct;
                printf("CALL %08x%s -> %08x%s lr=%08x r3=%08x r4=%08x "
                       "r5=%08x @%llu\n",
                       pc, sym(pc), cpu.st.pc, sym(cpu.st.pc), cpu.st.lr,
                       cpu.st.gpr[3], cpu.st.gpr[4], cpu.st.gpr[5],
                       static_cast<unsigned long long>(executed));
                snprintf(evb, sizeof evb,
                         "\"from\":%u,\"to\":%u,\"lr\":%u,\"r3\":%u,"
                         "\"r4\":%u,\"r5\":%u",
                         pc, cpu.st.pc, cpu.st.lr, cpu.st.gpr[3],
                         cpu.st.gpr[4], cpu.st.gpr[5]);
                evlog.emit(executed, "call", evb);
                fflush(stdout);
            }
        }
        // --watch-va: aim the memory watch at a GUEST VIRTUAL address by
        // translating it through the live MMU once the mapping exists. A
        // watch pointed at a hand-computed physical address fired zero
        // times this session and read exactly like "nothing ever writes
        // this", which is the most expensive kind of wrong answer.
        if (watchVa && !bus.watchPa && (executed & 0xFFFFFu) == 0) {
            const long long probe = guest(watchVa, 1);
            if (probe >= 0) {
                cpu.mmuProbe = true;
                const CpuState vSaved = cpu.st;
                cpu.st.msr |= 0x30u;
                u32 pa = 0;
                if (cpu.translate(watchVa, false, false, pa)) {
                    cpu.st = vSaved;
                    bus.watchPa = pa;
                    bus.watchPaEnd = pa + 3;
                    printf("-- watch-va %08x resolved to PA %08x @%llu\n",
                           watchVa, pa,
                           static_cast<unsigned long long>(executed));
                    fflush(stdout);
                }
                cpu.st = vSaved;
                cpu.mmuProbe = false;
            }
        }
        // A single-step view, windowed in TIME. --trace starts at
        // instruction zero, which is useless three billion in; the range
        // tracers only see code whose address you already know. What was
        // missing is "show me every instruction around HERE" — which,
        // resumed from a snapshot, costs seconds and is what a debugger's
        // step button actually provides.
        if (traceFrom && executed >= traceFrom &&
            executed < traceFrom + traceLines) {
            char tt[128];
            disassemble(cpu.curInsn, pc, tt, sizeof tt, Style::Gnu);
            printf("STEP @%llu %08x%s %08x  %-28s",
                   static_cast<unsigned long long>(executed), pc, sym(pc),
                   cpu.curInsn, tt);
            if ((pc & 0xFF000000u) == 0x68000000u)
                printf(" | 68K %08x%s op=%04x D0=%08x A0=%08x\n",
                       cpu.st.gpr[24], sym(cpu.st.gpr[24]),
                       cpu.st.gpr[27] & 0xFFFFu, cpu.st.gpr[8],
                       cpu.st.gpr[16]);
            else
                printf(" | r3=%08x r4=%08x lr=%08x\n", cpu.st.gpr[3],
                       cpu.st.gpr[4], cpu.st.lr);
            if (executed + 1 == traceFrom + traceLines)
                printf("-- trace window done (%llu lines)\n",
                       static_cast<unsigned long long>(traceLines));
        }
        if (dumpStructsAt && executed == dumpStructsAt)
            dumpStructs("--dump-structs-at");
        // Drive-queue change detector. We keep inferring "nothing ever
        // adds a drive"; this turns that into a positive statement.
        // DrvQHdr is 68K lowmem $308 and blue lowmem maps VA+0x4000, so
        // PA 0x4308. Sampled rather than hooked: a dirty cache line can
        // delay the observation, but the queue going non-empty is a
        // one-way event, so a late report is still a true one.
        if ((executed & 0x3FFFFFu) == 0) {
            static u32 lastDq = 0xFFFFFFFFu;
            const u32 dq = bus.read32(0x4308u);
            if (dq != lastDq) {
                printf("DRVQ $308 = %08x @%llu\n", dq,
                       static_cast<unsigned long long>(executed));
                fflush(stdout);
                lastDq = dq;
            }
        }
        // Value-origin watch: "who put this value here?" was the dominant
        // question this session, and answering it by bracketing across
        // separate runs cost hours. --watch-reg N --watch-val V reports
        // every instruction that makes gpr[N] become V, with the pc that
        // did it and the 68K pc in effect, which answers it in one run.
        if (watchReg < 32) {
            static u32 prevWatch = 0;
            static int wn = 0;
            const u32 now = cpu.st.gpr[watchReg];
            // --arm-at-park gates the REPORT, never the tracking: a register
            // hitting a common value over three billion instructions
            // exhausts the report cap long before the window of interest,
            // which lies the same way a stale instruction-count gate does —
            // by reporting the wrong window rather than none. --arm-on-value
            // is exempt, since it is the watch itself that opens that gate.
            // Two gates, because neither alone is enough: --arm-at-park is
            // durable against timeline shifts but can only open once the
            // machine has parked, which is too LATE for anything that
            // happens on the way there; --watch-from opens at an explicit
            // instruction count, which is exact but goes stale the moment a
            // fix moves the boot. Ungated, a 60-entry cap fills with
            // unrelated hits in the first few million instructions and the
            // watch reports the wrong window rather than none — the same
            // failure in a new costume.
            const bool report = (armOnValue || !armAtPark || parkArmed) &&
                                executed >= watchFrom;
            if (now == watchVal && prevWatch != watchVal && report &&
                wn < 60) {
                ++wn;
                if (armOnValue && !parkArmed) {
                    parkArmed = true;
                    printf("-- ARMED on value r%u == %08x\n", watchReg,
                           watchVal);
                }
                // r8-r15 are D0-D7 ONLY while the 68K emulator's dispatch
                // loop is running. Inside its helper routines they are
                // ordinary scratch, so a hit reported from a helper is not
                // a 68K register assignment at all — measured: an
                // `addic. r8,r0,-56` at 680b8640 looked exactly like
                // "D0 := -56" and is not. Say which world the hit is in.
                printf("REGSET r%u := %08x  [%s] pc=%08x r24=%08x lr=%08x "
                       "@%llu\n",
                       watchReg, watchVal,
                       (pc & 0xFF000000u) != 0x68000000u ? "native"
                       : (pc >= 0x68066000u && pc < 0x68068000u)
                           ? "68K dispatch"
                           : "68K emulator helper - regs are SCRATCH",
                       pc, cpu.st.gpr[24], cpu.st.lr,
                       static_cast<unsigned long long>(executed));
                snprintf(evb, sizeof evb,
                         "\"reg\":%u,\"val\":%u,\"pc\":%u,\"pc68\":%u,"
                         "\"lr\":%u",
                         watchReg, watchVal, pc, cpu.st.gpr[24], cpu.st.lr);
                evlog.emit(executed, "regset", evb);
                fflush(stdout);
            }
            prevWatch = now;
        }
        // Event-based arming. Hard-coded instruction gates go stale the
        // moment any fix shifts the timeline — one did this session, by
        // 830M instructions, and silently disarmed every instrument.
        // Arming on a CONDITION instead is durable: the park is the 68K
        // Start Manager's tick spin at ffc03666, so the Nth time we see it
        // the machine is definitively parked, whatever the clock says.
        // Arm on ARRIVAL at an address, native or 68K. The park gate is
        // durable but can only open once the machine has parked, so it is
        // useless for anything on the way there; an instruction count is
        // exact but goes stale on the next timing change. Arming on the
        // very code under study is both durable and early enough — three
        // separate watches this session reported the wrong window because
        // neither existing gate could express "when this routine runs".
        if (!parkArmed && armOnPc &&
            (pc == armOnPc || cpu.st.gpr[24] == armOnPc)) {
            parkArmed = true;
            printf("-- ARMED on pc %08x @%llu\n", armOnPc,
                   static_cast<unsigned long long>(executed));
            fflush(stdout);
        }
        if (!parkArmed && armAtPark && (pc & 0xFF000000u) == 0x68000000u &&
            cpu.st.gpr[24] == 0xFFC03666u && ++parkSeen >= armAtPark) {
            parkArmed = true;
            printf("-- ARMED on park (ffc03666 x%u) @%llu\n", armAtPark,
                   static_cast<unsigned long long>(executed));
            fflush(stdout);
        }
        const u32 region = pc >> 10;
        if (seen.emplace(region, executed).second)
            firsts.push_back({executed, pc});
        if ((executed & 63u) == 0) {
            ++pcHist[pc];
            if ((pc & 0xFF000000u) == 0x68000000u)
                ++pc68Hist[cpu.st.gpr[24] & ~1u];
        }
        // Every ATA Manager call, by function code. ffdd2bac starts with
        // lbz r31,18(r3) — the PB's function byte — and dispatches on it.
        // Reporting each code the first time it appears says which
        // operations the boot actually performs, and in particular whether
        // any device-registration call is ever made at all.
        if (pc == 0xFFDD2BC0u) {
            ++cen[kCenAtafn].hits;
            static u32 atafn[256] = {0};
            const u32 fn = cpu.st.gpr[31] & 0xFFu;
            if (atafn[fn]++ == 0) {
                printf("ATAFN %02x first @%llu pb=%08x\n", fn,
                       static_cast<unsigned long long>(executed),
                       cpu.st.gpr[3]);
                fflush(stdout); // rare: keep readable if a run is cut short
            }
            // The human log keeps its first-use census; the event stream
            // records EVERY call, because "which call, in what order, with
            // which block" is a question a census cannot answer.
            snprintf(evb, sizeof evb, "\"fn\":%u,\"pb\":%u", fn,
                     cpu.st.gpr[3]);
            evlog.emit(executed, "ata_call", evb);
            // The registration and lookup calls, byte for byte. The manager
            // reads a pointer at PB+0x3E and copies 256 bytes through it,
            // and its whole registered-driver list stays empty when that
            // pointer is null — but which field of .ATALoad's block lands
            // at +0x3E is a question about the 68K-to-native glue, not one
            // to answer by counting offsets in a disassembly. Dump the
            // block the native side actually sees.
            static int pbDumps = 0;
            if ((fn == 0x85u || fn == 0x86u || fn == 0x93u || fn == 0x98u) &&
                pbDumps < 10) {
                ++pbDumps;
                cpu.l1dFlushAll(true);
                cpu.l2FlushAll(true);
                cpu.mmuProbe = true;
                const CpuState pbSaved = cpu.st;
                const bool pbRaised = cpu.raisedThisStep;
                const CpuState pbArmed = cpu.st;
                const u32 pb = cpu.st.gpr[3];
                // Report the PHYSICAL address too. A parameter block is a
                // virtual pointer, --watch-mem takes a physical one, and a
                // watch aimed at a guessed translation fires zero times and
                // reads as "never written" — which is the same silence as a
                // watch that was simply pointed at the wrong page.
                u32 pbPa = 0;
                cpu.st = pbArmed;
                const bool pbOk = cpu.translate(pb, false, false, pbPa);
                cpu.st = pbArmed;
                printf("ATAPB fn=%02x pb=%08x pa=%s%08x @%llu\n", fn, pb,
                       pbOk ? "" : "?", pbPa,
                       static_cast<unsigned long long>(executed));
                for (u32 row = 0; row < 0x50u; row += 16) {
                    printf("   +%02x:", row);
                    for (u32 k = 0; k < 16; ++k) {
                        u32 pa = 0;
                        cpu.st = pbArmed;
                        const bool ok =
                            cpu.translate(pb + row + k, false, false, pa);
                        cpu.st = pbArmed;
                        if (ok && pa < bus.ram().size())
                            printf(" %02x", static_cast<u8>(
                                                bus.read32(pa & ~3u) >>
                                                (8 * (3 - (pa & 3u)))));
                        else
                            printf(" ??");
                    }
                    printf("\n");
                }
                // AddATABus is handed a pointer at PB+0x30 to a 16-byte bus
                // descriptor, which the handler copies into the new bus
                // record. Three calls arrive with byte-identical parameter
                // blocks, so whatever distinguishes one channel from the
                // next is inside that structure — and which channel each
                // registration is for is exactly the question when two of
                // the three come back -56.
                if (fn == 0x93u) {
                    u32 desc = 0, pa = 0;
                    cpu.st = pbArmed;
                    if (cpu.translate(pb + 0x30u, false, false, pa) &&
                        pa + 4 <= bus.ram().size())
                        desc = bus.read32(pa & ~3u);
                    cpu.st = pbArmed;
                    printf("   bus descriptor at %08x:", desc);
                    for (u32 k = 0; k < 16; ++k) {
                        u32 dpa = 0;
                        cpu.st = pbArmed;
                        const bool ok =
                            cpu.translate(desc + k, false, false, dpa);
                        cpu.st = pbArmed;
                        if (ok && dpa < bus.ram().size())
                            printf(" %02x", static_cast<u8>(
                                                bus.read32(dpa & ~3u) >>
                                                (8 * (3 - (dpa & 3u)))));
                        else
                            printf(" ??");
                    }
                    printf("\n");
                }
                cpu.st = pbSaved;
                cpu.raisedThisStep = pbRaised;
                cpu.mmuProbe = false;
                fflush(stdout);
            }
        }
        // Every ATA Manager call completes through ffdd4204(PB, result):
        // each arm ends with `mr r4,<result>; bl 0xdd4204`. The family has
        // 25 separate sites that can produce -56, so rather than watch
        // them individually, watch the one place they all funnel through
        // and correlate the PB pointer with the function code the tally
        // above already reports for that same PB.
        if (pc == 0xFFDD4204u) {
            ++cen[kCenAtares].hits;
            static int cp = 0;
            const int res = static_cast<int>(cpu.st.gpr[4]);
            if (cp < 200 && res != 0) {
                ++cp;
                // The parameter block of the call that FAILED, not of the
                // first call with each function code. Over a whole boot
                // there are only a handful of non-zero completions, and
                // which call each one belongs to — and with what arguments
                // — is the entire question. A first-use census cannot say,
                // because the failing call is rarely the first of its kind.
                cpu.l1dFlushAll(true);
                cpu.l2FlushAll(true);
                cpu.mmuProbe = true;
                const CpuState rSaved = cpu.st;
                const bool rRaised = cpu.raisedThisStep;
                const CpuState rArmed = cpu.st;
                const u32 rpb = cpu.st.gpr[3];
                auto rb = [&](u32 off) -> int {
                    u32 pa = 0;
                    cpu.st = rArmed;
                    const bool ok =
                        cpu.translate(rpb + off, false, false, pa);
                    cpu.st = rArmed;
                    if (!ok || pa >= bus.ram().size())
                        return -1;
                    return static_cast<int>(
                        static_cast<u8>(bus.read32(pa & ~3u) >>
                                        (8 * (3 - (pa & 3u)))));
                };
                printf("ATARES pb=%08x result=%d fn=%02x @%llu\n", rpb, res,
                       rb(0x12) & 0xFF,
                       static_cast<unsigned long long>(executed));
                snprintf(evb, sizeof evb,
                         "\"pb\":%u,\"result\":%d,\"fn\":%d", rpb, res,
                         rb(0x12) & 0xFF);
                evlog.emit(executed, "ata_result", evb);
                for (u32 row = 0x10; row < 0x50u; row += 16) {
                    printf("   +%02x:", row);
                    for (u32 k = 0; k < 16; ++k) {
                        const int v = rb(row + k);
                        if (v < 0)
                            printf(" ??");
                        else
                            printf(" %02x", v);
                    }
                    printf("\n");
                }
                cpu.st = rSaved;
                cpu.raisedThisStep = rRaised;
                cpu.mmuProbe = false;
                fflush(stdout);
            }
        }
        // The ATA Manager's device lookup, ffdd3a80: walk the singly-linked
        // list whose head is at globals+0x38 (then +2), comparing each
        // node's 16-bit id at +0x0C against the wanted one. Manager
        // function 0x86 preloads -56 (nsDrvErr) and returns it whenever
        // this yields NULL — which is the exact error the boot dies on.
        // Log the head, the wanted id, and every id the list actually
        // holds: that says whether any ATA device is registered at all,
        // and under which id.
        if (pc == 0xFFDD3A88u || pc == 0xFFDD3A98u || pc == 0xFFDD3AB0u) {
            ++cen[kCenLk].hits;
            static int lk = 0;
            if (lk < 120) {
                ++lk;
                if (pc == 0xFFDD3A88u)
                    printf("LK head=%08x want=%08x @%llu\n", cpu.st.gpr[12],
                           cpu.st.gpr[3],
                           static_cast<unsigned long long>(executed));
                else
                    printf("LK   node=%08x id=%08x want=%08x\n",
                           cpu.st.gpr[12],
                           cpu.st.gpr[pc == 0xFFDD3A98u ? 5 : 4],
                           cpu.st.gpr[11]);
                fflush(stdout);
            }
        }
        // Proof-first seed. .ATALoad's request queue never receives the
        // selector-1 "an ATA device arrived" notification (ffd9b504's jump
        // table, arm 1 at ffd9b54c), so all ten of its slots keep the free
        // tag '!act' and no disk driver is ever installed — which is why
        // the on-disc driver honestly answers nsDrvErr. Hand-enqueue one
        // request, the same way the [EM+294] seed proved the USB chain
        // before real OHCI existed. Entry layout is from the processor at
        // ffd9b23e: +0x00 tag 'load', +0x04 deviceID (handed to ATA
        // Manager function 0x86), +0x08 kind (tested against 1 and 3).
        // The queue is found by its own header rather than a fixed address:
        // 'LOAD' followed by free slots at +0x10/+0x20.
        if (!ataPoked && ataPokeAt && executed > ataPokeAt) {
            ataPoked = true;
            cpu.l1dFlushAll(true);
            u32 q = 0;
            for (u32 pa = 0x1000; pa + 0x120 < bus.ram().size(); pa += 4)
                if (bus.read32(pa) == 0x4C4F4144u &&
                    bus.read32(pa + 0x10) == 0x21616374u &&
                    bus.read32(pa + 0x20) == 0x21616374u) {
                    q = pa;
                    break;
                }
            if (q) {
                bus.write32(q + 0x14, ataPokeDev);
                bus.write32(q + 0x18, ataPokeKind);
                bus.write32(q + 0x10, 0x6C6F6164u); // 'load' published last
                printf("-- ATA POKE: queue PA %08x hdr=%08x/%08x count=%u; "
                       "slot0 := 'load' dev=%08x kind=%u @%llu\n",
                       q, bus.read32(q + 4), bus.read32(q + 8),
                       bus.read32(q + 0x0C) & 0xFFFFu, ataPokeDev,
                       ataPokeKind,
                       static_cast<unsigned long long>(executed));
            } else {
                printf("-- ATA POKE: .ATALoad request queue not found\n");
            }
        }
        // Decline forensics: every 68K transition inside the .ATALoad DRVR
        // body and its _Control caller, MINUS the strcmp character loop
        // (that loop is 80% of the flow's traffic and is already decoded —
        // it walks the APM types and matches Apple_Driver_ATAPI). Dropping
        // it, plus the registry walk's own code, keeps the budget on the
        // part that has never been captured: what the flow does after the
        // walk returns, where the install is refused.
        if ((pc & 0xFF000000u) == 0x68000000u) {
            static u32 prev68 = 0;
            const u32 cur68 = cpu.st.gpr[24];
            // A 68K tracer aimed by flag rather than by recompile, plus the
            // question that came up more than any other in this dig: WHAT
            // DID THAT ROUTINE ANSWER. The emulator holds the current 68K
            // opcode in r27, so an RTS (0x4E75) inside the window is a
            // return, and D0 at that moment is the answer. Bracketing a
            // return value by hand across a twelve-thousand-line trace was
            // costing a run each time; this prints it.
            if (t68Lo && cur68 != prev68 && cur68 >= t68Lo && cur68 < t68Hi &&
                executed >= watchFrom) {
                static u32 t68n = 0;
                if (t68n < t68Cap) {
                    ++t68n;
                    const bool isRts = (cpu.st.gpr[27] & 0xFFFFu) == 0x4E75u;
                    printf("%s %08x%s D0=%08x D1=%08x A0=%08x A1=%08x "
                           "A6=%08x\n",
                           isRts ? "RET" : "T68", cur68, sym(cur68),
                           cpu.st.gpr[8], cpu.st.gpr[9], cpu.st.gpr[16],
                           cpu.st.gpr[17], cpu.st.gpr[22]);
                    if (t68n == t68Cap)
                        printf("-- trace-68k cap %u reached @%llu\n", t68Cap,
                               static_cast<unsigned long long>(executed));
                }
            }
            // Ungated: the ROM only reaches this call site when it has
            // already matched Apple_HFS, so it is rare by construction and
            // worth catching from the first boot-time attempt onward.
            if (cur68 != prev68 &&
                (cur68 == 0xFFD9BC60u || cur68 == 0xFFD9BC64u)) {
                static int calls = 0;
                if (calls < 80) {
                    ++calls;
                    printf("DRV%s D0=%08x D5=%08x D6=%08x A0=%08x A6=%08x "
                           "@%llu\n",
                           cur68 == 0xFFD9BC60u ? ">" : "<", cpu.st.gpr[8],
                           cpu.st.gpr[13], cpu.st.gpr[14], cpu.st.gpr[16],
                           cpu.st.gpr[22],
                           static_cast<unsigned long long>(executed));
                }
            }
            if (cur68 != prev68 &&
                (armAtPark ? parkArmed : executed > 2800000000ull)) {
                ++cen[kCen68kGate].hits;
                const bool body = (cur68 >= 0xFFD9A000u &&
                                   cur68 < 0xFFD9D000u) ||
                                  (cur68 >= 0xFFC5DA00u &&
                                   cur68 < 0xFFC5DE00u);
                const bool strcmpLoop = cur68 >= 0xFFD9BC80u &&
                                        cur68 <= 0xFFD9BCA4u;
                // Apple's driver checksum (decoded: sum+=byte, sum<<=1,
                // bit16 rotates into bit0, low word returned). Verified
                // offline to reproduce the disc's stored pmBootCksum, so
                // the iterations carry no news — only the result does,
                // which lands in D0 at the first line past the loop.
                const bool cksumLoop = cur68 >= 0xFFD9BACCu &&
                                       cur68 <= 0xFFD9BAF2u;
                // The on-disc driver runs from the 0x2800 buffer, whose
                // address is only known at run time: the _NewPtrSysClear
                // return crosses ffd9c618 with D0 = noErr and A0 = the
                // pointer. Latch it, then trace the driver's own code in
                // ITS OWN offsets (they line up with the ISO image at
                // LBA 0x3c), minus its internal ADD.W/ROL.W checksum.
                static u32 drvBase = 0;
                if (cur68 == 0xFFD9C618u && cpu.st.gpr[8] == 0 &&
                    cpu.st.gpr[16] > 0x10000u) {
                    if (drvBase != cpu.st.gpr[16]) {
                        drvBase = cpu.st.gpr[16];
                        printf("-- driver base %08x @%llu\n", drvBase,
                               static_cast<unsigned long long>(executed));
                    }
                }
                const u32 drvOff = drvBase ? cur68 - drvBase : 0xFFFFFFFFu;
                static int dlines = 0;
                if (drvOff < 0x2800u &&
                    !(drvOff >= 0x6C0u && drvOff <= 0x6CCu) &&
                    dlines <= 8000) {
                    if (++dlines > 8000)
                        printf("-- driver trace done (8000)\n");
                    else
                        printf("V +%04x D0=%08x D1=%08x A0=%08x A1=%08x\n",
                               drvOff, cpu.st.gpr[8], cpu.st.gpr[9],
                               cpu.st.gpr[16], cpu.st.gpr[17]);
                }
                // The ROM hands the loaded driver one device at a time via
                // JSR 8(A0) at ffd9bc60 ('BOOT' in D0, the device selector
                // in D5, which the driver folds into 0x01000000|(dev<<16)).
                // Log the selector going in and the OSErr coming back at
                // ffd9bc64 — that says WHICH device it is asking about and
                // whether any of them is our CD on bus 0.
                // THE GATE: .ATALoad issues ATA_MgrInquiry (fn 0x90), then
                // requires the byte the manager returns at PB+0x30 to be
                // >= 4 (CMPI.B #4 / BCC at ffd9b698); below that it just
                // DisposePtrs and returns -23 openErr without attempting
                // any install. Watch a small RANGE, not an exact pc — r24
                // is a fetch pointer and not every instruction address is
                // ever equal to it.
                if (cur68 >= 0xFFD9B6DCu && cur68 <= 0xFFD9B6F0u) {
                    static int mi = 0;
                    if (mi < 60) {
                        ++mi;
                        printf("MGRINQ pc68=%08x D0=%08x D7=%08x @%llu\n",
                               cur68, cpu.st.gpr[8], cpu.st.gpr[15],
                               static_cast<unsigned long long>(executed));
                        fflush(stdout);
                    }
                }
                // Every Toolbox A-trap .ATALoad executes. The emulator keeps
                // the current 68K opcode in r27, so an opcode whose high
                // nibble is 0xA inside the DRVR body is a trap call. This
                // says what the driver DOES rather than what its control
                // flow looks like — in particular whether it ever reaches
                // _DrvrInstall ($A03D) or _AddDrive ($A04E), the calls
                // that would actually put a drive in the queue.
                if (cur68 >= 0xFFD9A000u && cur68 < 0xFFD9D000u &&
                    (cpu.st.gpr[27] & 0xF000u) == 0xA000u) {
                    static u32 traps[0x1000] = {0};
                    static int atFull = 0;
                    const u32 t = cpu.st.gpr[27] & 0x0FFFu;
                    // Arguments as well as identity: a trap census says
                    // WHICH calls happen, but the interesting value is
                    // almost always what was passed and what came back.
                    // A0 is the param-block pointer for the Device
                    // Manager and ATA Manager calls; D0 carries selectors
                    // and sizes. The result lands in D0 (and D7 for the
                    // manager) a few instructions later, so the NEXT
                    // trap line's registers bracket the previous call.
                    if (atFull < 200) {
                        ++atFull;
                        printf("ATRAP $A%03x %-18s pc68=%08x%s D0=%08x "
                               "D7=%08x A0=%08x A3=%08x @%llu\n",
                               t, trapName(t), cur68, sym(cur68),
                               cpu.st.gpr[8], cpu.st.gpr[15],
                               cpu.st.gpr[16], cpu.st.gpr[19],
                               static_cast<unsigned long long>(executed));
                        fflush(stdout);
                    }
                    snprintf(evb, sizeof evb,
                             "\"trap\":%u,\"name\":\"%s\",\"pc68\":%u,"
                             "\"d0\":%u,\"d7\":%u,\"a0\":%u",
                             t, trapName(t), cur68, cpu.st.gpr[8],
                             cpu.st.gpr[15], cpu.st.gpr[16]);
                    evlog.emit(executed, "atrap", evb);
                    ++traps[t];
                }
                // Branch-outcome truth. r24 is a FETCH pointer, so the
                // word after a short branch is always crossed before the
                // branch is applied — three watches this session fired on
                // addresses that are never instruction boundaries, and a
                // taken BEQ was misread as a fall-through. Flag every
                // discontinuity instead: a backward step, or a forward
                // step of more than 6 bytes, means control actually
                // transferred rather than fell through.
                // Pass counter: the give-up loop re-enters the request
                // processor once per kick, and only the FIRST pass gets
                // furthest, so a capped trace otherwise fills with pass
                // one.s prefix. --trace-pass N records just pass N.
                static u32 passNo = 0;
                if (cur68 == 0xFFD9B240u)
                    ++passNo;
                const bool passWanted = !tracePass || passNo == tracePass;
                if (cur68 >= 0xFFD9B6F0u && cur68 <= 0xFFD9C660u) {
                    static u32 prevSeq = 0;
                    static int bt = 0;
                    if (prevSeq && bt < 400 && passWanted) {
                        const int d = static_cast<int>(cur68 - prevSeq);
                        if (d < 0 || d > 6) {
                            ++bt;
                            printf("BR %08x -> %08x (%+d) D0=%08x D7=%08x\n",
                                   prevSeq, cur68, d, cpu.st.gpr[8],
                                   cpu.st.gpr[15]);
                            fflush(stdout);
                        }
                    }
                    prevSeq = cur68;
                }
                // General 68K call-stack capture at any address. "Who
                // called this?" came up repeatedly and was answered each
                // time by hand-walking a backchain at one specific site.
                // A LINK A6 frame stores the caller's A6 at (A6) and the
                // return address at 4(A6), so the chain walks itself.
                if (stackAt && cur68 == stackAt) {
                    static int sk = 0;
                    if (sk < 8) {
                        ++sk;
                        cpu.l1dFlushAll(true);
                        cpu.l2FlushAll(true);
                        cpu.mmuProbe = true;
                        const CpuState kSaved = cpu.st;
                        const bool kRaised = cpu.raisedThisStep;
                        cpu.st.msr |= 0x30u;
                        const CpuState kArmed = cpu.st;
                        auto rd32 = [&](u32 ea, u32& v) {
                            u32 pa = 0;
                            cpu.st = kArmed;
                            const bool ok =
                                cpu.translate(ea, false, false, pa);
                            cpu.st = kArmed;
                            if (!ok || pa + 4 > bus.ram().size())
                                return false;
                            v = bus.read32(pa);
                            return true;
                        };
                        printf("STACK at %08x @%llu:", cur68,
                               static_cast<unsigned long long>(executed));
                        u32 frame = cpu.st.gpr[22];
                        for (int k = 0; k < 8 && frame; ++k) {
                            u32 nxt = 0, ret = 0;
                            if (!rd32(frame, nxt) || !rd32(frame + 4, ret))
                                break;
                            printf(" %08x", ret);
                            if (nxt <= frame)
                                break;
                            frame = nxt;
                        }
                        printf("\n");
                        cpu.st = kSaved;
                        cpu.raisedThisStep = kRaised;
                        cpu.mmuProbe = false;
                        fflush(stdout);
                    }
                }
                // The generic strcmp at ffd9bc7c, read after both operand
                // loads (A1 = 8(A6), A3 = 12(A6)). The install path walks
                // the DDR, finds ddType 0x0701, and then this compare
                // returns -1 and the failure propagates out. Print both
                // strings so the failing comparison names itself.
                if (cur68 == 0xFFD9BC90u) {
                    ++cen[kCenScmp].hits;
                    static int sc = 0;
                    if (sc < 300) {
                        ++sc;
                        cpu.l1dFlushAll(true);
                        cpu.l2FlushAll(true);
                        cpu.mmuProbe = true;
                        const CpuState sSaved = cpu.st;
                        const bool sRaised = cpu.raisedThisStep;
                        cpu.st.msr |= 0x30u;
                        const CpuState sArmed = cpu.st;
                        auto str = [&](u32 ea, char* out) {
                            for (int k = 0; k < 24; ++k) {
                                u32 pa = 0;
                                cpu.st = sArmed;
                                const bool ok =
                                    cpu.translate(ea + k, false, false, pa);
                                cpu.st = sArmed;
                                if (!ok || pa >= bus.ram().size()) {
                                    out[k] = 0;
                                    return;
                                }
                                const u8 c = static_cast<u8>(
                                    bus.read32(pa & ~3u) >>
                                    (8 * (3 - (pa & 3u))));
                                out[k] = (c >= 32 && c < 127)
                                             ? static_cast<char>(c)
                                             : 0;
                                if (!c)
                                    return;
                            }
                            out[24] = 0;
                        };
                        char a[26] = {}, b[26] = {};
                        str(cpu.st.gpr[17], a);
                        str(cpu.st.gpr[19], b);
                        cpu.st = sSaved;
                        cpu.raisedThisStep = sRaised;
                        cpu.mmuProbe = false;
                        printf("SCMP |%s| vs |%s| @%llu\n", a, b,
                               static_cast<unsigned long long>(executed));
                        fflush(stdout);
                    }
                }
                // The queue's own entry point, ffd9b504: the ATA Manager
                // holds this proc plus refNum ffcd (-51) and a context
                // pointer in a registration record, and calls it to
                // announce events. It switches on the word at (A3);
                // selector 1 is the "device arrived" enqueue. We only ever
                // observe selector 8. Log every selector that actually
                // arrives, so "8 and nothing else" is a measured fact.
                if (cur68 == 0xFFD9B504u) {
                    static int qs = 0;
                    if (qs < 40) {
                        ++qs;
                        cpu.l1dFlushAll(true);
                        cpu.l2FlushAll(true);
                        cpu.mmuProbe = true;
                        const CpuState qSaved = cpu.st;
                        const bool qRaised = cpu.raisedThisStep;
                        cpu.st.msr |= 0x30u;
                        u32 sel = 0xFFFFFFFFu, pa = 0;
                        const u32 a3 = cpu.st.gpr[19];
                        if (cpu.translate(a3, false, false, pa) &&
                            pa + 2 < bus.ram().size())
                            sel = (bus.read32(pa & ~3u) >>
                                   (16 - 8 * (pa & 2u))) &
                                  0xFFFFu;
                        cpu.st = qSaved;
                        cpu.raisedThisStep = qRaised;
                        cpu.mmuProbe = false;
                        printf("QSEL sel=%04x a3=%08x @%llu\n", sel, a3,
                               static_cast<unsigned long long>(executed));
                        fflush(stdout);
                    }
                }
                // .ATALoad's Control entry. Its DRVR header at ffd9aff0
                // gives drvrControl = 0x3C, so every _Control the driver
                // receives arrives here with the csCode in the param block
                // at A0+0x1A. The 250 ms kick is csCode $41 and only reads
                // the request queue; the selector-1 "device arrived"
                // enqueue must come in as some other csCode — or never.
                if (cur68 == 0xFFD9B02Cu) {
                    static int cc = 0;
                    if (cc < 60) {
                        ++cc;
                        cpu.l1dFlushAll(true);
                        cpu.l2FlushAll(true);
                        cpu.mmuProbe = true;
                        const CpuState cSaved = cpu.st;
                        const bool cRaised = cpu.raisedThisStep;
                        cpu.st.msr |= 0x30u;
                        u32 cs = 0xFFFFFFFFu, pa = 0;
                        const u32 pb = cpu.st.gpr[16];
                        if (cpu.translate(pb + 0x1Au, false, false, pa) &&
                            pa + 2 < bus.ram().size())
                            cs = (bus.read32(pa & ~3u) >>
                                  (16 - 8 * (pa & 2u))) &
                                 0xFFFFu;
                        cpu.st = cSaved;
                        cpu.raisedThisStep = cRaised;
                        cpu.mmuProbe = false;
                        printf("CTL csCode=%04x pb=%08x @%llu\n", cs, pb,
                               static_cast<unsigned long long>(executed));
                        fflush(stdout);
                    }
                }
                // The 68K ATA Manager itself: GetTrapAddress($AAF1) returns
                // 000a3c10 at run time (RAM — the manager is installed and
                // patched in), and the loaded driver calls it a dozen times
                // per pass. Dump the parameter block going in and the OSErr
                // coming back: that is what the driver actually believes
                // about the bus and its devices.
                static bool inMgr = false;
                static int mgrLines = 0;
                if (cur68 == 0x000A3C10u && mgrLines < 400) {
                    ++mgrLines;
                    inMgr = true;
                    cpu.l1dFlushAll(true);
                    cpu.l2FlushAll(true);
                    cpu.mmuProbe = true;
                    const CpuState mSaved = cpu.st;
                    const bool mRaised = cpu.raisedThisStep;
                    cpu.st.msr |= 0x30u;
                    const CpuState mArmed = cpu.st;
                    const u32 pb = cpu.st.gpr[16];
                    printf("MGR> pb=%08x D0=%08x A1=%08x:", pb,
                           cpu.st.gpr[8], cpu.st.gpr[17]);
                    for (u32 k = 0; k < 0x30; ++k) {
                        u32 pa = 0;
                        cpu.st = mArmed;
                        bool ok = cpu.translate(pb + k, false, false, pa);
                        cpu.st = mArmed;
                        if (ok && pa < bus.ram().size())
                            printf("%s%02x", (k & 15u) ? "" : " ",
                                   static_cast<u8>(bus.read32(pa & ~3u) >>
                                                   (8 * (3 - (pa & 3u)))));
                        else
                            printf("%s??", (k & 15u) ? "" : " ");
                    }
                    printf("\n");
                    cpu.st = mSaved;
                    cpu.raisedThisStep = mRaised;
                    cpu.mmuProbe = false;
                } else if (inMgr && drvOff < 0x2800u) {
                    inMgr = false;
                    if (mgrLines < 400)
                        printf("MGR< D0=%08x D1=%08x A0=%08x +%04x\n",
                               cpu.st.gpr[8], cpu.st.gpr[9], cpu.st.gpr[16],
                               drvOff);
                }
                static int lines = 0;
                if (body && !strcmpLoop && !cksumLoop && lines <= 12000) {
                    if (++lines > 12000)
                        printf("-- ataload trace done (12000) @%llu\n",
                               static_cast<unsigned long long>(executed));
                    else
                        printf("L %08x D0=%08x D1=%08x A0=%08x A1=%08x "
                               "A3=%08x\n",
                               cur68, cpu.st.gpr[8], cpu.st.gpr[9],
                               cpu.st.gpr[16], cpu.st.gpr[17],
                               cpu.st.gpr[19]);
                }
            }
            prev68 = cur68;
        }
        ring.push(pc, cpu.curInsn);
        if (pc == 0xFFD8736Cu) {
            delayCallers[cpu.st.lr]++;
            // Ring-capture every entry's backchain (saved LR at 8(frame)
            // per CFM); the LAST traces always cover the parked steady
            // state no matter how the boot timeline shifts.
            DfTrace& t = dfRing[dfRingAt++ % 12u];
            t.at = executed;
            t.durHi = cpu.st.gpr[3];
            t.durLo = cpu.st.gpr[4];
            for (u32 f = 0; f < 6; ++f)
                t.caller[f] = 0;
            cpu.l1dFlushAll(true);
            cpu.l2FlushAll(true);
            cpu.mmuProbe = true;
            const CpuState dfSaved = cpu.st;
            const bool dfRaised = cpu.raisedThisStep;
            cpu.st.msr |= 0x30u;
            const CpuState dfArmed = cpu.st;
            auto dfRead = [&](u32 ea, u32& v) {
                u32 pa = 0;
                cpu.st = dfArmed;
                const bool ok = cpu.translate(ea, false, false, pa);
                cpu.st = dfArmed;
                if (!ok || (pa & ~3u) + 4 > bus.ram().size())
                    return false;
                v = bus.read32(pa & ~3u);
                return true;
            };
            u32 sp = cpu.st.gpr[1];
            for (u32 f = 0; f < 6 && sp; ++f) {
                u32 bc = 0, slr = 0;
                if (!dfRead(sp, bc) || !bc)
                    break;
                if (dfRead(bc + 8, slr))
                    t.caller[f] = slr;
                sp = bc;
            }
            cpu.st = dfSaved;
            cpu.raisedThisStep = dfRaised;
            cpu.mmuProbe = false;
        }
        if (trace) {
            disassemble(cpu.curInsn, pc, text, sizeof text, Style::Gnu);
            fprintf(stderr, "%08x: %s\n", pc, text);
        }
        tickPeripherals();
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
        // Who issues the PMU polls: print pc/lr at each new command byte
        // (the wire log has no pc; the issuers' code is the decode target).
        static size_t pmuLogSeen = 0;
        static u32 pmuCmdShown = 0;
        {
            const auto& pl2 = bus.pmu().log;
            while (pmuLogSeen < pl2.size()) {
                const auto& ev = pl2[pmuLogSeen++];
                if (ev.kind == 'c' && executed > 1200000000ull &&
                    pmuCmdShown < 40 &&
                    (ev.val == 0xDCu || ev.val == 0x78u || ev.val == 0x8Fu ||
                     ev.val == 0x39u)) {
                    ++pmuCmdShown;
                    printf("-- pmu cmd %02x @%llu pc=%08x lr=%08x r24=%08x\n",
                           ev.val,
                           static_cast<unsigned long long>(executed),
                           cpu.st.pc, cpu.st.lr, cpu.st.gpr[24]);
                }
            }
        }
        // Who computes product-code: the OF dictionary compiles it as
        // lis/ori/blr with the value baked into the ori at PA 03C3AF74-
        // region; the writer pc is the machine-identity computation.
        static u32 pcodePrev = 0xEEEEEEEEu;
        static u32 pcodeShown = 0;
        if (executed > 30000000ull && executed < 262000000ull) {
            u32 cv = 0;
            if (!cpu.l1dPeek32(0x03D08BD0u, cv))
                cv = bus.read32(0x03D08BD0u);
            if (cv != pcodePrev && pcodeShown < 8) {
                ++pcodeShown;
                printf("-- pcode cell %08x -> %08x @%llu pc=%08x "
                       "lr=%08x\n",
                       pcodePrev, cv,
                       static_cast<unsigned long long>(executed),
                       cpu.st.pc, cpu.st.lr);
                printf("   rstack r30=%08x:", cpu.st.gpr[30]);
                {
                    cpu.l1dFlushAll(true);
                    cpu.mmuProbe = true;
                    const CpuState saved2 = cpu.st;
                    const bool savedR2 = cpu.raisedThisStep;
                    for (u32 k = 0; k < 16; ++k) {
                        const u32 ea = (cpu.st.gpr[30] + k * 4) & ~3u;
                        u32 pa2 = ea;
                        cpu.st = saved2;
                        if ((cpu.st.msr & 0x10u) &&
                            !cpu.translate(ea, false, false, pa2))
                            pa2 = 0xFFFFFFFFu;
                        printf(" %08x", pa2 == 0xFFFFFFFFu
                                            ? 0xEEEEEEEEu
                                            : bus.read32(pa2));
                    }
                    cpu.st = saved2;
                    cpu.raisedThisStep = savedR2;
                    cpu.mmuProbe = false;
                }
                printf("\n");
                if ((cv & 0xFFFFu) != 0 && cv != 0xEEEEEEEEu) {
                    printf("   ppc ring (last 20):\n");
                    const u32 zc = ring.n < 20 ? ring.n : 20;
                    for (u32 k = 0; k < zc; ++k) {
                        const auto& e = ring.e[(ring.n - zc + k) & 127u];
                        disassemble(e.insn, e.pc, text, sizeof text,
                                    Style::Gnu);
                        printf("   %08x: %08x  %s\n", e.pc, e.insn, text);
                    }
                }
            }
            if (cv != pcodePrev)
                pcodePrev = cv;
        }
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
            if (pc == 0xFFE2325Cu && !emPoked) {
                emPoked = true;
                cpu.l1dFlushAll(true);
                const u32 cur = bus.read32(0x000116C4u);
                if (cur == 0) {
                    bus.write32(0x000116C4u, 0xFFC339A2u);
                    printf("-- DIAGNOSTIC poke: [EM+294] := ffc339a2 "
                           "(RTS)\n");
                } else {
                    printf("-- [EM+294] already seeded: %08x (no poke)\n",
                           cur);
                }
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
            // 60Hz tick delivery tracer: the 68K tick handler increments
            // lowmem Ticks at ffc0bc00 (ADDQ.L #1,($016A).W); entries
            // into the handler region name the delivery mechanism.
            static u32 tickShown = 0;
            if (prev68k >= 0xFFC0BBE8u && prev68k <= 0xFFC0BBF0u &&
                tickShown < 3 && executed > 1200000000ull &&
                (pc & 0xFFC00000u) == 0x68000000u) {
                ++tickShown;
                printf("-- tick delivery #%u @%llu pc68=%08x ppcpc=%08x "
                       "lr=%08x\n   ring:",
                       tickShown, static_cast<unsigned long long>(executed),
                       prev68k, cpu.st.pc, cpu.st.lr);
                for (u32 k = 0; k < 24; ++k) {
                    const Ent68& e =
                        ring68[(ring68At + 128u - 24u + k) & 127u];
                    printf(" %08x%s", e.pc68,
                           (e.ppc & 0xFFC00000u) == 0x68000000u ? "" : "*");
                }
                printf("\n");
            }
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

    // Snapshot validation. A snapshot that is believed rather than proven is
    // worse than no snapshot: it diverges silently and manufactures evidence
    // that looks exactly like the real thing. So the feature ships with the
    // proof attached, and the proof is deliberately harsh — the restore
    // happens ON TOP OF a machine that has already run past the snapshot
    // point, so any field that is not actually captured is still holding its
    // later value when leg B starts, and leg B diverges.
    if (verifyAt) {
        if (executed != verifyAt) {
            printf("-- VERIFY ABORTED: run stopped at %llu (halted=%d), "
                   "never reached %llu\n",
                   static_cast<unsigned long long>(executed), cpu.halted ? 1 : 0,
                   static_cast<unsigned long long>(verifyAt));
            return 3;
        }
        struct Step {
            u32 pc, insn;
        };
        constexpr size_t kTraceCap = 4000000; // 32 MB of first-divergence detail
        std::vector<Step> traceA;
        auto harness = [&]() {
            return HarnessState{executed, fastTb,   fastTbUntil, parkSeen,
                                parkArmed, ataPoked, emPoked};
        };
        // Advance exactly as the instrumented loop does: step, tick the
        // peripherals off the pre-increment clock, then count.
        auto leg = [&](u64 n, std::vector<Step>* rec,
                       const std::vector<Step>* expect, u64& digest,
                       u64& ran) {
            digest = 1469598103934665603ull;
            ran = 0;
            long long firstBad = -1;
            for (u64 k = 0; k < n && !cpu.halted; ++k) {
                const u32 pcNow = cpu.st.pc;
                cpu.step();
                const u32 insn = cpu.curInsn;
                const u32 pair[2] = {pcNow, insn};
                const u8* pb = reinterpret_cast<const u8*>(pair);
                for (size_t q = 0; q < sizeof pair; ++q) {
                    digest ^= pb[q];
                    digest *= 1099511628211ull;
                }
                if (rec && rec->size() < kTraceCap)
                    rec->push_back({pcNow, insn});
                if (expect && firstBad < 0 && k < expect->size() &&
                    ((*expect)[static_cast<size_t>(k)].pc != pcNow ||
                     (*expect)[static_cast<size_t>(k)].insn != insn)) {
                    firstBad = static_cast<long long>(k);
                    printf("-- DIVERGENCE at step %lld of the window "
                           "(insn %llu): straight run had pc=%08x insn=%08x, "
                           "the resumed run has pc=%08x insn=%08x\n",
                           firstBad,
                           static_cast<unsigned long long>(executed),
                           (*expect)[static_cast<size_t>(k)].pc,
                           (*expect)[static_cast<size_t>(k)].insn, pcNow,
                           insn);
                }
                tickPeripherals();
                ++executed;
                ++ran;
            }
        };

        const HarnessState h0 = harness();
        SnapWriter w0;
        saveSnapshot(cpu, bus, h0, w0);
        const u64 fp0 = snapshotFingerprint(cpu, bus, h0);
        printf("-- verify: snapshot at %llu is %zu bytes, "
               "fingerprint=%016llx\n",
               static_cast<unsigned long long>(executed), w0.buf.size(),
               static_cast<unsigned long long>(fp0));
        fflush(stdout);

        u64 digA = 0, ranA = 0;
        leg(verifySteps, &traceA, nullptr, digA, ranA);
        const u64 fpA = snapshotFingerprint(cpu, bus, harness());
        printf("-- verify: straight leg ran %llu insns, trace digest "
               "%016llx, end fingerprint %016llx\n",
               static_cast<unsigned long long>(ranA),
               static_cast<unsigned long long>(digA),
               static_cast<unsigned long long>(fpA));
        fflush(stdout);

        HarnessState hR;
        {
            SnapReader r(w0.buf.data(), w0.buf.size());
            if (!loadSnapshot(cpu, bus, hR, r)) {
                printf("-- VERIFY FAILED: the snapshot could not be read "
                       "back: %s\n",
                       r.err.c_str());
                return 3;
            }
        }
        executed = hR.executed;
        fastTb = hR.fastTb;
        fastTbUntil = hR.fastTbUntil;
        parkSeen = hR.parkSeen;
        parkArmed = hR.parkArmed;
        ataPoked = hR.ataPoked;
        emPoked = hR.emPoked;

        // Round-trip identity: serializing the restored machine must
        // reproduce the same bytes. This catches a field that is saved but
        // restored wrongly, independently of whether the guest ever reads
        // it.
        bool roundTrip = snapshotFingerprint(cpu, bus, hR) == fp0;
        if (!roundTrip) {
            SnapWriter w1;
            saveSnapshot(cpu, bus, hR, w1);
            size_t at = 0;
            while (at < w1.buf.size() && at < w0.buf.size() &&
                   w1.buf[at] == w0.buf[at])
                ++at;
            printf("-- VERIFY FAILED: save/load is not lossless — first "
                   "difference at byte %zu of %zu (restored size %zu)\n",
                   at, w0.buf.size(), w1.buf.size());
        }

        u64 digB = 0, ranB = 0;
        leg(verifySteps, nullptr, &traceA, digB, ranB);
        const u64 fpB = snapshotFingerprint(cpu, bus, harness());
        printf("-- verify: resumed leg ran %llu insns, trace digest "
               "%016llx, end fingerprint %016llx\n",
               static_cast<unsigned long long>(ranB),
               static_cast<unsigned long long>(digB),
               static_cast<unsigned long long>(fpB));

        const bool same = roundTrip && ranA == ranB && digA == digB &&
                          fpA == fpB;
        printf("-- VERIFY %s: round-trip %s, %llu instructions "
               "%s, end state %s\n",
               same ? "PASSED" : "FAILED", roundTrip ? "lossless" : "LOSSY",
               static_cast<unsigned long long>(ranA),
               (ranA == ranB && digA == digB) ? "identical" : "DIVERGED",
               fpA == fpB ? "identical" : "DIFFERENT");
        if (!same)
            printf("   A snapshot that fails this must not be used for "
                   "evidence: it will diverge silently.\n");
        return same ? 0 : 3;
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
    // Gate state first, because "hits=0" has two completely different
    // meanings and only this line distinguishes them: the watch looked and
    // saw nothing, or the watch never started looking. Three instruments
    // this session reported silence of the second kind and it was read as
    // silence of the first.
    printf("-- gates: arm-at-park=%u (park seen %u times, %s) "
           "arm-on-pc=%08x watch-from=%llu (%s) events=%s\n",
           armAtPark, parkSeen, parkArmed ? "ARMED" : "never armed", armOnPc,
           static_cast<unsigned long long>(watchFrom),
           executed >= watchFrom ? "reached" : "NEVER REACHED",
           eventsPath ? eventsPath : "off");
    printf("-- instrument census (hits=0 means the watch never fired —\n"
           "   check the address is a crossed fetch position and that any\n"
           "   gate actually opened before the run ended):\n");
    for (const auto& c : cen)
        printf("   %-10s hits=%llu\n", c.name,
               static_cast<unsigned long long>(c.hits));
    if (dumpStructsEnd)
        dumpStructs("end of run");
    evlog.emit(executed, "end", nullptr);
    evlog.close();
    printf("-- msr=%08x dec=%08x hid0=%08x\n", cpu.st.msr, cpu.st.dec,
           cpu.st.hid0);
    {
        // Does guest time track host time? The 60 Hz chain increments the
        // 68K lowmem Ticks long at $016A (blue lowmem maps VA+0x4000, so
        // PA 0x416A). A desktop that draws correctly but runs at a twelfth
        // of real speed is not a desktop, so this is the number the pacing
        // work has to move — stated rather than guessed at.
        const double host =
            std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                          hostStart)
                .count();
        const u32 ticks = bus.read32(0x0000416Au);
        // Before the 68K world starts, that cell holds power-on RAM junk.
        // Printing a rate derived from junk is how a measurement becomes a
        // wrong belief, so say so instead: a plausible count is bounded by
        // 60/s against a run that has never been longer than minutes.
        char tickText[96];
        if (ticks < 100000000u)
            snprintf(tickText, sizeof tickText,
                     "guest Ticks=%u (%.1f/host-s; real is 60/s)", ticks,
                     host > 0 ? ticks / host : 0.0);
        else
            snprintf(tickText, sizeof tickText,
                     "guest Ticks n/a (68K lowmem not initialised yet)");
        printf("-- timing: %.1f s host, %.1f MIPS, tb=%llu (%.2f MHz), %s%s\n",
               host, host > 0 ? executed / host / 1e6 : 0.0,
               static_cast<unsigned long long>(cpu.st.tb),
               host > 0 ? cpu.st.tb / host / 1e6 : 0.0, tickText,
               realtime ? (rtSlips ? " [realtime, slipped]" : " [realtime]")
                        : "");
    }
    {
        std::vector<std::pair<u64, u32>> top;
        for (const auto& [pc, n] : pcHist)
            top.push_back({n, pc});
        std::sort(top.rbegin(), top.rend());
        printf("-- DelayFor last traces (of %u):\n", dfRingAt);
        for (u32 k = 0; k < 12u && k < dfRingAt; ++k) {
            const DfTrace& t =
                dfRing[(dfRingAt - (dfRingAt < 12u ? dfRingAt : 12u) + k) %
                       12u];
            printf("   @%llu dur=%08x:%08x  %08x %08x %08x %08x %08x "
                   "%08x\n",
                   static_cast<unsigned long long>(t.at), t.durHi, t.durLo,
                   t.caller[0], t.caller[1], t.caller[2], t.caller[3],
                   t.caller[4], t.caller[5]);
        }
        printf("-- 68k pc histogram (top 16):\n");
        {
            std::vector<std::pair<u64, u32>> h;
            for (const auto& [a, n] : pc68Hist)
                h.push_back({n, a});
            std::sort(h.rbegin(), h.rend());
            for (size_t k = 0; k < h.size() && k < 16; ++k)
                printf("   %08x  samples=%llu\n", h[k].second,
                       static_cast<unsigned long long>(h[k].first));
        }
        printf("-- DelayFor callers (%zu):\n", delayCallers.size());
        {
            std::vector<std::pair<u64, u32>> dc;
            for (const auto& [lr, n] : delayCallers)
                dc.push_back({n, lr});
            std::sort(dc.rbegin(), dc.rend());
            for (size_t k = 0; k < dc.size() && k < 24; ++k)
                printf("   lr=%08x x%llu\n", dc[k].second,
                       static_cast<unsigned long long>(dc[k].first));
        }
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
        const size_t cs = cl.size() > 200 ? cl.size() - 200 : 0;
        for (size_t k = cs; k < cl.size(); ++k) {
            if (cl[k].a || cl[k].b)
                printf("%c%02x:%x+%x@%llu ", cl[k].kind, cl[k].val,
                       cl[k].a, cl[k].b,
                       static_cast<unsigned long long>(cl[k].at));
            else
                printf("%c%02x@%llu ", cl[k].kind, cl[k].val,
                       static_cast<unsigned long long>(cl[k].at));
        }
        printf("\n");
    }
    {
        // The disk's own command log, the same way the CD's is reported.
        // Register traffic says what the driver poked; the command log says
        // what it ASKED FOR, which is the question when a boot device is
        // probed but never read.
        const auto& hl = bus.hd().log;
        printf("-- hd command log (%zu; c=ata p=packet e=err):\n   ",
               hl.size());
        const size_t hs = hl.size() > 200 ? hl.size() - 200 : 0;
        for (size_t k = hs; k < hl.size(); ++k) {
            if (hl[k].a || hl[k].b)
                printf("%c%02x:%x+%x@%llu ", hl[k].kind, hl[k].val, hl[k].a,
                       hl[k].b, static_cast<unsigned long long>(hl[k].at));
            else
                printf("%c%02x@%llu ", hl[k].kind, hl[k].val,
                       static_cast<unsigned long long>(hl[k].at));
        }
        printf("\n");
    }
    {
        // The TAIL, not the head: the log is trimmed as it grows, so the
        // first 120 entries are an arbitrary window that has nothing to do
        // with what the run was asked to investigate.
        const auto& al = bus.ataLog();
        printf("-- ata traffic (%zu; last 160; off r/w val pc):\n",
               al.size());
        const size_t as = al.size() > 160 ? al.size() - 160 : 0;
        for (size_t k = as; k < al.size(); ++k)
            printf("   +%05x %c %02x pc=%08x @%llu\n", al[k].pa & ~1u,
                   (al[k].pa & 1u) ? 'r' : 'w', al[k].val & 0xFFu,
                   al[k].pc, static_cast<unsigned long long>(al[k].at));
    }
    {
        const auto& cl = bus.cfgLog();
        printf("-- pci config accesses (%zu; bus latch val pc r/w):\n",
               cl.size());
        const size_t cfs = cl.size() > 100 ? cl.size() - 100 : 0;
        for (size_t k = cfs; k < cl.size(); ++k)
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
        printf("\n-- pmu tail with stamps (last 40):\n");
        const size_t s2 = pl.size() > 40 ? pl.size() - 40 : 0;
        for (size_t k = s2; k < pl.size(); ++k)
            printf("   %c %02x @%llu\n", pl[k].kind, pl[k].val,
                   static_cast<unsigned long long>(pl[k].at));
    }
    for (u32 f = 0; f < 2; ++f) {
        const auto& ol = bus.ohci(f).log;
        printf("-- ohci%u writes (%zu):\n", f, ol.size());
        for (size_t k = 0; k < ol.size() && k < 60; ++k)
            printf("   +%03x <- %08x pc=%08x @%llu\n", ol[k].off,
                   ol[k].val, ol[k].pc,
                   static_cast<unsigned long long>(ol[k].at));
        printf("-- ohci%u read census:\n", f);
        for (const auto& [off, n] : bus.ohci(f).readCount)
            printf("   +%03x x%llu\n", off,
                   static_cast<unsigned long long>(n));
    }
    {
        // Head AND tail: the head is the card's own FCode bring-up, the
        // tail is whatever the OS driver did most recently — and when the
        // display work starts it is the tail that matters. Reporting only
        // the head is the same trimming trap the ATA log had.
        const auto& al = bus.ati().log;
        printf("-- ati reg traffic (%zu; w/r; first 60 then last 60):\n",
               al.size());
        for (size_t k = 0; k < al.size() && k < 60; ++k)
            printf("   %c +%04x %08x pc=%08x @%llu\n",
                   al[k].wr ? 'w' : 'r', al[k].off, al[k].val, al[k].pc,
                   static_cast<unsigned long long>(al[k].at));
        for (size_t k = al.size() > 60 ? al.size() - 60 : 60; k < al.size();
             ++k)
            printf("   %c +%04x %08x pc=%08x @%llu\n",
                   al[k].wr ? 'w' : 'r', al[k].off, al[k].val, al[k].pc,
                   static_cast<unsigned long long>(al[k].at));
    }
    {
        const auto& dl = bus.ataDma().log;
        printf("-- ata dbdma events (%zu; 0=ctl 1=desc 2=input 3=stop "
               "4=dead):\n",
               dl.size());
        for (size_t k = 0; k < dl.size() && k < 80; ++k)
            printf("   %u %08x %08x @%llu\n", dl[k].kind, dl[k].a,
                   dl[k].b, static_cast<unsigned long long>(dl[k].at));
    }
    if (bus.atiPresent()) {
        // CRTC-aware screen dump: geometry straight from the live CRTC
        // registers, palette from the DAC; the PPM is the machine's
        // first light (user-verified — never self-judged).
        const u32 gen = bus.ati().peek(0x0050);
        const u32 ht = bus.ati().peek(0x0200);
        const u32 vt = bus.ati().peek(0x0208);
        const u32 pitch8 = bus.ati().peek(0x022C) & 0xFFFFu;
        const u32 offset = bus.ati().peek(0x0224);
        const u32 w = (((ht >> 16) & 0x3FFu) + 1u) * 8u;
        const u32 h = ((vt >> 16) & 0xFFFu) + 1u;
        const u32 fmt = (gen >> 8) & 0xFu;
        printf("-- ati crtc: gen=%08x %ux%u fmt=%u pitch8=%u "
               "offset=%08x\n",
               gen, w, h, fmt, pitch8, offset);
        if ((gen & 0x02000000u) && w >= 64 && w <= 2048 && h >= 64 &&
            h <= 1536 && (fmt == 2u || fmt == 6u)) {
            const u32 bypp = fmt == 2u ? 1u : 4u;
            const u32 rowBytes = pitch8 * 8u * bypp;
            FILE* pf = fopen("ati_screen.ppm", "wb");
            if (pf) {
                fprintf(pf, "P6\n%u %u\n255\n", w, h);
                const auto& vr = bus.ati().vram;
                for (u32 y = 0; y < h; ++y)
                    for (u32 x = 0; x < w; ++x) {
                        const size_t o = offset + size_t(y) * rowBytes +
                                         size_t(x) * bypp;
                        u8 rgb[3] = {0, 0, 0};
                        if (o + bypp <= vr.size()) {
                            if (fmt == 2u) {
                                const u32 c = bus.ati().pal(vr[o]);
                                rgb[0] = static_cast<u8>(c >> 16);
                                rgb[1] = static_cast<u8>(c >> 8);
                                rgb[2] = static_cast<u8>(c);
                            } else {
                                rgb[0] = vr[o + 2];
                                rgb[1] = vr[o + 1];
                                rgb[2] = vr[o + 0];
                            }
                        }
                        fwrite(rgb, 1, 3, pf);
                    }
                fclose(pf);
                printf("-- ati screen dumped: ati_screen.ppm\n");
            }
        }
    }
    {
        const auto& il = bus.pic().log;
        printf("-- openpic events (%zu; v=src<<24|vp a=iack e=eoi):\n",
               il.size());
        for (size_t k = 0; k < il.size(); ++k)
            printf("   %c %08x @%llu\n", il[k].kind, il[k].val,
                   static_cast<unsigned long long>(il[k].at));
        printf("-- openpic raises per source:\n");
        for (u32 s = 0; s < 64; ++s)
            if (bus.pic().raiseCount[s])
                printf("   src %u x%llu\n", s,
                       static_cast<unsigned long long>(
                           bus.pic().raiseCount[s]));
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
