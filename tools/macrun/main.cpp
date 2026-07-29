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

const char* atiRegName(u32 off)
{
    switch (off) {
    case 0x00: return "CRTC_H_TOTAL_DISP";
    case 0x04: return "CRTC_H_SYNC_STRT_WID";
    case 0x08: return "CRTC_V_TOTAL_DISP";
    case 0x0C: return "CRTC_V_SYNC_STRT_WID";
    case 0x10: return "CRTC_VLINE_CRNT_VLINE";
    case 0x14: return "CRTC_OFF_PITCH";
    case 0x18: return "CRTC_INT_CNTL";
    case 0x1C: return "CRTC_GEN_CNTL";
    case 0x20: return "DSP_CONFIG";
    case 0x24: return "DSP_ON_OFF";
    case 0x28: return "TIMER_CONFIG";
    case 0x2C: return "MEM_BUF_CNTL";
    case 0x40: return "OVR_CLR";
    case 0x44: return "OVR_WID_LEFT_RIGHT";
    case 0x48: return "OVR_WID_TOP_BOTTOM";
    case 0x60: return "CUR_CLR0";
    case 0x64: return "CUR_CLR1";
    case 0x68: return "CUR_OFFSET";
    case 0x6C: return "CUR_HORZ_VERT_POSN";
    case 0x70: return "CUR_HORZ_VERT_OFF";
    case 0x78: return "GP_IO";
    case 0x7C: return "HW_DEBUG";
    case 0x80: return "SCRATCH_REG0";
    case 0x84: return "SCRATCH_REG1";
    case 0x90: return "CLOCK_CNTL";
    case 0xA0: return "BUS_CNTL";
    case 0xB0: return "MEM_CNTL";
    case 0xC0: return "DAC_REGS";
    case 0xC4: return "DAC_CNTL";
    case 0xD0: return "GEN_TEST_CNTL";
    case 0xDC: return "CONFIG_CNTL";
    case 0xE0: return "CONFIG_CHIP_ID";
    case 0xE4: return "CONFIG_STAT0";
    default: return "";
    }
}

// Dump the display as the CRTC + DAC describe it (PPM); before the CRTC is
// up, fall back to a raw 640x480 window on VRAM (PGM).
void dumpFramebuffer(const char* path, AtiRage& ati)
{
    const AtiRage::Mode m = ati.mode();
    const std::vector<u8>& vr = ati.vram();
    FILE* f = fopen(path, "wb");
    if (!f) {
        fprintf(stderr, "macrun: cannot write %s\n", path);
        return;
    }
    if (!m.enabled || m.width == 0 || m.height == 0 || m.bpp == 0) {
        fprintf(f, "P5\n640 480\n255\n");
        fwrite(vr.data(), 1, 640 * 480, f);
        fclose(f);
        printf("-- framebuffer written (crtc down; raw 640x480 window): %s\n",
               path);
        return;
    }
    const u32 bytesPP = m.bpp <= 8 ? 1 : (m.bpp <= 16 ? 2 : (m.bpp == 24 ? 3 : 4));
    const u32 pitchPx = m.pitchPixels ? m.pitchPixels : m.width;
    fprintf(f, "P6\n%u %u\n255\n", m.width, m.height);
    for (u32 y = 0; y < m.height; ++y) {
        for (u32 x = 0; x < m.width; ++x) {
            const size_t p = m.offsetBytes +
                             size_t(y) * pitchPx * bytesPP +
                             size_t(m.bpp == 4 ? x / 2 : x) * bytesPP;
            u8 rgb[3] = {0, 0, 0};
            if (p + bytesPP <= vr.size() && p + bytesPP <= AtiRage::kRegTop) {
                switch (m.bpp) {
                case 4: {
                    const u8 idx = (x & 1) ? (vr[p] & 0xFu) : (vr[p] >> 4);
                    ati.palette(idx, rgb[0], rgb[1], rgb[2]);
                    break;
                }
                case 8:
                    ati.palette(vr[p], rgb[0], rgb[1], rgb[2]);
                    break;
                case 15: {
                    const u32 v = vr[p] | (u32(vr[p + 1]) << 8);
                    rgb[0] = static_cast<u8>(((v >> 10) & 31) << 3);
                    rgb[1] = static_cast<u8>(((v >> 5) & 31) << 3);
                    rgb[2] = static_cast<u8>((v & 31) << 3);
                    break;
                }
                case 16: {
                    const u32 v = vr[p] | (u32(vr[p + 1]) << 8);
                    rgb[0] = static_cast<u8>(((v >> 11) & 31) << 3);
                    rgb[1] = static_cast<u8>(((v >> 5) & 63) << 2);
                    rgb[2] = static_cast<u8>((v & 31) << 3);
                    break;
                }
                case 24: // little-endian BGR bytes
                    rgb[0] = vr[p + 2];
                    rgb[1] = vr[p + 1];
                    rgb[2] = vr[p];
                    break;
                default: // 32: little-endian xRGB dword
                    rgb[0] = vr[p + 2];
                    rgb[1] = vr[p + 1];
                    rgb[2] = vr[p];
                    break;
                }
            }
            fwrite(rgb, 1, 3, f);
        }
    }
    fclose(f);
    printf("-- framebuffer written (%ux%u@%ubpp): %s\n", m.width, m.height,
           m.bpp, path);
}

} // namespace

int main(int argc, char** argv)
{
    const char* romPath = nullptr;
    const char* fbPath = nullptr;
    u64 maxInsns = 50000000ull;
    size_t ramMb = 64;
    bool trace = false;
    int excShow = 16;
    u32 disStart = 0, disEnd = 0;
    u32 vdisStart = 0, vdisEnd = 0;
    u32 pvrOverride = 0;
    bool forceBank0 = false;

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
        else if (!strcmp(a, "--dis")) {
            disStart = static_cast<u32>(strtoul(next(), nullptr, 0));
            disEnd = static_cast<u32>(strtoul(next(), nullptr, 0));
        }
        else if (!strcmp(a, "--vdis")) {
            vdisStart = static_cast<u32>(strtoul(next(), nullptr, 0));
            vdisEnd = static_cast<u32>(strtoul(next(), nullptr, 0));
        }
        else if (!strcmp(a, "--pvr")) pvrOverride = strtoul(next(), nullptr, 0);
        else if (!strcmp(a, "--force-bank0")) forceBank0 = true;
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

    if (disStart && disEnd > disStart) { // static window straight off the bus
        char text[128];
        for (u32 a = disStart & ~3u; a < disEnd; a += 4) {
            const u32 w = bus.read32(a);
            disassemble(w, a, text, sizeof text, Style::Gnu);
            printf("   %08x: %08x  %s\n", a, w, text);
        }
        return 0;
    }

    Cpu cpu;
    cpu.attach(bus);
    // 60x bus snooping: the mac-io DBDMA engine is a bus master and the
    // descriptor lists it walks are written by ordinary cached stores.
    CpuSnoop snoop;
    snoop.cpu = &cpu;
    bus.snoop = &snoop;
    cpu.reset(); // pc = 0xFFF00100, MSR[IP] set: vectors in ROM — authentic
    bus.ati().pcRef = &cpu.st.pc;
    bus.ati().lrRef = &cpu.st.lr;
    bus.macio().pcRef = &cpu.st.pc;
    if (pvrOverride) { // experiment: present as a different CPU to HWInit
        cpu.st.pvr = pvrOverride;
        printf("-- pvr override: %08x\n", pvrOverride);
    }
    if (forceBank0) {
        bus.forceBank0 = true;
        printf("-- experiment: bank 0 forced live\n");
    }
    bus.htabWatchBase = 0x00009000u; // the 68K-side handler page
    bus.htabWatchSize = 0x1000u;

    Ring ring;
    char text[128];
    u64 executed = 0;
    int excLogged = 0;
    bus.stamp = &executed;
    bus.macio().stamp = &executed;
    // Virtual-window disassembler: safe to run mid-step, snapshots and
    // restores the whole CPU state around the translate probes.
    auto vdump = [&](const char* when) {
        printf("-- virtual disassembly %08x..%08x (%s):\n", vdisStart,
               vdisEnd, when);
        cpu.mmuProbe = true; // probes must not disturb TLBs or R/C bits
        const CpuState saved = cpu.st;
        for (u32 va = vdisStart & ~3u; va < vdisEnd; va += 4) {
            cpu.st.msr |= 0x30u; // IR|DR on: the driver-time view
            u32 pa = 0;
            if (cpu.translate(va, false, true, pa)) {
                const u32 w = bus.read32(pa);
                disassemble(w, va, text, sizeof text, Style::Gnu);
                printf("   %08x -> %08x: %08x  %s\n", va, pa, w, text);
            } else {
                printf("   %08x -> unmapped\n", va);
                cpu.st = saved; // the failed probe raised; undo its edits
            }
        }
        cpu.st = saved;
        cpu.mmuProbe = false;
    };
    bool vdumped = false, wildCaught = false, mqProbed = false;
    u32 late300Shown = 0, ext500Shown = 0;
    bool ctx500Seen = false, chimeEntryShown = false;
    u32 prev68kPc = 0, ring68k[128] = {}, ring68kAt = 0, idScanShown = 0;
    struct InsRec {
        u64 at;
        u32 r8, r9, r10, r12, r13;
    };
    std::vector<InsRec> inserterLog;
    u64 lastGpioOps = 0;
    struct Dispatch {
        u64 at;
        u32 pc, ctr;
    };
    std::vector<Dispatch> dispatches; // threaded-code jumps in the engine
    dispatches.reserve(1 << 16);
    struct ExcEnt {
        u64 at;
        u32 vec, srr0, srr1, dsisr, dar;
    };
    ExcEnt excRing[16] = {};
    u32 excRingAt = 0;
    bool probed = false;
    // One-shot deep dump when the fatal-cascade DSI first appears: the
    // instruction ring, the translation machinery, and targeted probes.
    auto probeDump = [&]() {
        cpu.mmuProbe = true;
        printf("-- probe: last instructions into the fault:\n");
        const u32 cnt = ring.n < 32 ? ring.n : 32;
        for (u32 k = 0; k < cnt; ++k) {
            const auto& e = ring.e[(ring.n - cnt + k) & 31u];
            disassemble(e.insn, e.pc, text, sizeof text, Style::Gnu);
            printf("   %08x: %08x  %s\n", e.pc, e.insn, text);
        }
        printf("-- probe: srr0=%08x srr1=%08x dar=%08x dsisr=%08x msr=%08x\n",
               cpu.st.srr0, cpu.st.srr1, cpu.st.dar, cpu.st.dsisr,
               cpu.st.msr);
        printf("-- probe: sdr1=%08x\n", cpu.st.sdr1);
        for (int b = 0; b < 4; ++b)
            printf("-- probe: ibat%d %08x/%08x dbat%d %08x/%08x\n", b,
                   cpu.st.ibatu[b], cpu.st.ibatl[b], b, cpu.st.dbatu[b],
                   cpu.st.dbatl[b]);
        for (int s = 0; s < 16; s += 4)
            printf("-- probe: sr%02d %08x %08x %08x %08x\n", s,
                   cpu.st.sr[s], cpu.st.sr[s + 1], cpu.st.sr[s + 2],
                   cpu.st.sr[s + 3]);
        static const u32 probes[] = {0xF3013020u, 0xFF80C328u, 0xFF80C314u,
                                     0xFF808120u};
        const CpuState saved = cpu.st;
        for (u32 va : probes) {
            for (int asFetch = 0; asFetch < 2; ++asFetch) {
                cpu.st.msr |= 0x30u;
                u32 pa = 0;
                const bool ok =
                    cpu.translate(va, false, asFetch != 0, pa);
                printf("-- probe: %08x %s -> %s %08x\n", va,
                       asFetch ? "fetch" : "data ",
                       ok ? "ok" : "MISS", ok ? pa : 0);
                cpu.st = saved;
            }
        }
        printf("-- probe: r4=%08x r8=%08x r20=%08x r23=%08x r30=%08x "
               "ctr=%08x lr=%08x\n",
               cpu.st.gpr[4], cpu.st.gpr[8], cpu.st.gpr[20], cpu.st.gpr[23],
               cpu.st.gpr[30], cpu.st.ctr, cpu.st.lr);
        printf("-- probe: sprg0=%08x sprg1=%08x sprg2=%08x sprg3=%08x\n",
               cpu.st.sprg[0], cpu.st.sprg[1], cpu.st.sprg[2],
               cpu.st.sprg[3]);
        printf("-- probe: [sprg3..+0x40] bus vs cache:\n");
        for (u32 k = 0; k < 16; ++k) {
            const u32 a = cpu.st.sprg[3] + 4 * k;
            u32 cw = 0;
            const bool hit = cpu.l1dPeek32(a, cw);
            printf("   +%02x bus=%08x cache=%s\n", 4 * k, bus.read32(a),
                   hit ? (sprintf(text, "%08x", cw), text) : "-");
        }
        // Forth backtrace: r30 is the threaded-code return stack; every
        // compiled word opens with "lhz r5, TOKEN(r23); stwu r19,-4(r30)".
        // For each saved return address, scan backwards for the prologue
        // and report the owning word's token number.
        auto vread32 = [&](u32 va, u32& out) {
            cpu.st.msr |= 0x30u;
            u32 pa = 0;
            const bool ok = cpu.translate(va, false, false, pa);
            cpu.st = saved;
            if (ok)
                out = bus.read32(pa);
            return ok;
        };
        auto nameWord = [&](u32 ra) {
            for (u32 back = 0; back < 0x400; back += 4) {
                u32 w = 0;
                if (!vread32(ra - back, w))
                    return;
                if (w == 0x967EFFFCu) { // stwu r19, -4(r30)
                    u32 h = 0;
                    if (vread32(ra - back - 4, h) &&
                        (h & 0xFFFF0000u) == 0xA0B70000u)
                        printf(" word@%08x token=%03x", ra - back - 4,
                               h & 0xFFFFu);
                    else
                        printf(" word@%08x token=?", ra - back);
                    return;
                }
            }
        };
        printf("-- probe: forth return stack:\n");
        for (u32 d = 0; d < 20; ++d) {
            u32 ra = 0;
            if (!vread32(cpu.st.gpr[30] + 4 * d, ra))
                break;
            if (ra < 0xFF800000u || (ra & 3u))
                continue;
            printf("   [%2u] %08x", d, ra);
            nameWord(ra);
            printf("\n");
        }
        // Hash table sweep: every valid PTE whose EA reconstruction lands
        // in mac-io or the OF window. SDR1 gives the table base and size.
        const u32 htab = cpu.st.sdr1 & 0xFFFF0000u;
        const u32 htMask = ((cpu.st.sdr1 & 0x1FFu) << 16) | 0xFFFFu;
        u32 valid = 0, shown = 0;
        for (u32 off = 0; off <= htMask; off += 8) {
            const u32 w0 = bus.read32(htab + off);
            if (!(w0 & 0x80000000u))
                continue;
            ++valid;
            const u32 w1 = bus.read32(htab + off + 4);
            const u32 vsid = (w0 >> 7) & 0xFFFFFFu;
            const u32 api = w0 & 0x3Fu;
            const u32 h = (w0 >> 6) & 1u;
            // Reconstruct the EA page index from the hash (group index =
            // off / 64) for the primary-hash case.
            const u32 group = (off / 64) & (htMask >> 6);
            u32 pageIdx = (h ? ~group : group) ^ (vsid & 0x7FFu);
            pageIdx = (pageIdx & 0x3FFu) | (api << 10);
            const u32 ea = ((vsid & 0xFu) << 28) | (pageIdx << 12);
            const bool interesting =
                (ea >> 24) == 0xF3u || (ea >> 20) == 0xFF8u;
            if (interesting && shown < 24) {
                ++shown;
                printf("-- probe: pte@%05x v=%06x api=%02x h=%u ea~%08x "
                       "rpn=%08x wimg=%x pp=%x\n",
                       off, vsid, api, h, ea, w1 & 0xFFFFF000u,
                       (w1 >> 3) & 0xFu, w1 & 3u);
            }
        }
        printf("-- probe: hash table @%08x mask=%08x valid=%u\n", htab,
               htMask, valid);
        // Lifecycle of the load-bearing PTEs: every watched write whose
        // old or new word0 names one of the pages in the fatal chain.
        static const u32 hotPages[] = {0xF3013u, 0xFF808u, 0xF3008u,
                                       0xFF813u, 0xFF80Cu, 0xF3012u};
        printf("-- probe: hash-table writes touching the hot pages:\n");
        u32 printed = 0;
        for (const auto& wr : bus.htabLog()) {
            const u32 slot = (wr.pa - 0x004E0000u) & ~7u;
            const u32 group = slot / 64;
            bool hot = false;
            for (u32 cand = 0; cand < 2 && !hot; ++cand) {
                const u32 w0 = cand ? wr.newW : wr.oldW;
                if (!(w0 & 0x80000000u) || (wr.pa & 4u))
                    continue;
                const u32 vsid = (w0 >> 7) & 0xFFFFFFu;
                const u32 api = w0 & 0x3Fu;
                const u32 h = (w0 >> 6) & 1u;
                u32 pi = ((h ? ~group : group) ^ vsid) & 0x3FFu;
                pi |= api << 10;
                const u32 page = ((vsid & 0xFu) << 16) | pi;
                for (u32 hp : hotPages)
                    if (page == hp)
                        hot = true;
            }
            if (hot && printed < 40) {
                ++printed;
                printf("   @%-11llu pa=%08x %08x -> %08x\n",
                       static_cast<unsigned long long>(wr.at), wr.pa,
                       wr.oldW, wr.newW);
            }
        }
        printf("-- probe: htab writes total logged: %zu\n",
               bus.htabLog().size());
        printf("-- probe: raw PTEGs now: grp03:");
        for (u32 w = 0; w < 16; ++w)
            printf(" %08x", bus.read32(0x004E00C0u + 4 * w));
        printf("\n-- probe:            grp07:");
        for (u32 w = 0; w < 16; ++w)
            printf(" %08x", bus.read32(0x004E01C0u + 4 * w));
        printf("\n-- probe:            grp1C:");
        for (u32 w = 0; w < 16; ++w)
            printf(" %08x", bus.read32(0x004E0700u + 4 * w));
        printf("\n-- probe: htab writes in the fatal window:\n");
        u32 nshown = 0;
        for (const auto& wr : bus.htabLog()) {
            if (wr.at < 186300000ull || wr.at > 186520000ull)
                continue;
            if (++nshown > 200)
                break;
            const char* kind = (wr.pa & 0x80000000u)
                                   ? "b"
                                   : ((wr.pa & 0x40000000u) ? "h" : "w");
            printf("   @%-11llu %s pa=%08x %08x -> %08x\n",
                   static_cast<unsigned long long>(wr.at), kind,
                   wr.pa & 0x00FFFFFFu, wr.oldW, wr.newW);
        }
        cpu.mmuProbe = false;
    };
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
        if (cpu.curInsn == 0x4E800420u && (pc >> 12) == 0xFF809u &&
            executed >= 186300000ull && dispatches.size() < (1u << 16))
            dispatches.push_back({executed, pc, cpu.st.ctr});
        if (trace) {
            disassemble(cpu.curInsn, pc, text, sizeof text, Style::Gnu);
            fprintf(stderr, "%08x: %s\n", pc, text);
        }
        bus.macio().tick();
        bus.ati().tick();
        cpu.setExternalIrq(bus.macio().irqAsserted());
        ++executed;
        if (cpu.st.gpr[24] != prev68kPc) {
            prev68kPc = cpu.st.gpr[24];
            ring68k[ring68kAt++ & 127u] = prev68kPc;
            // D7 is BoxFlags during the 68K boot era; log every value
            // change with the 68K pc that produced it.
            static u32 lastD7 = 0xEE00EE00u;
            if (idScanShown < 80 && cpu.st.gpr[15] != lastD7) {
                lastD7 = cpu.st.gpr[15];
                ++idScanShown;
                printf("-- D7 %08x at pc68=%08x (prev %08x)\n", lastD7,
                       prev68kPc,
                       ring68k[(ring68kAt + 126u) & 127u]);
            }
            if (!chimeEntryShown && (prev68kPc & 0xFFFFFF00u) == 0xFFCCE100u) {
                chimeEntryShown = true;
                printf("-- 68k trail into the chime (last 128 pcs):\n   ");
                for (u32 k = 0; k < 128; ++k)
                    printf("%08x ", ring68k[(ring68kAt + k) & 127u]);
                printf("\n-- at entry: A0=%08x A1=%08x A2=%08x A6=%08x "
                       "D0=%08x @%llu\n",
                       cpu.st.gpr[16], cpu.st.gpr[17], cpu.st.gpr[18],
                       cpu.st.gpr[22], cpu.st.gpr[8],
                       static_cast<unsigned long long>(executed));
            }
        }
        if ((executed % 10000000) == 0 && executed > 150000000) {
            printf("-- 68k probe @%llu: pc68=%08x",
                   static_cast<unsigned long long>(executed),
                   cpu.st.gpr[24]);
            for (u32 r = 8; r < 24; ++r)
                printf(" %s%u=%08x", r < 16 ? "D" : "A", (r - 8) & 7,
                       cpu.st.gpr[r]);
            printf("\n");
        }
        if (!ctx500Seen && (executed % 100000) == 0 && executed > 400000 &&
            cpu.st.sprg[3]) {
            u32 w = 0;
            const u32 a = cpu.st.sprg[3] + 0x500u;
            if (!cpu.l1dPeek32(a, w) && !cpu.l2Peek32(a, w))
                w = bus.read32(a);
            if (w != 0 && w != 0xFFFFFFFFu && w != 0xDEADBEEFu) {
                ctx500Seen = true;
                printf("-- [sprg3+0x500] registered: %08x by @%llu\n", w,
                       static_cast<unsigned long long>(executed));
            }
        }
        if (vdisEnd && !vdumped && bus.ati().gpioOps() != lastGpioOps) {
            lastGpioOps = bus.ati().gpioOps();
            vdumped = true;
            vdump("at first GP_IO touch");
        }
        if (pc == 0xFFF22464u && inserterLog.size() < 256)
            inserterLog.push_back({executed, cpu.st.gpr[8], cpu.st.gpr[9],
                                   cpu.st.gpr[10], cpu.st.gpr[12],
                                   cpu.st.gpr[13]});
        if (!wildCaught && (cpu.st.pc < 0x100u ||
                            (cpu.st.pc >= 0x60000000u &&
                             cpu.st.pc < 0x70000000u))) {
            wildCaught = true;
            printf("-- wild jump: pc=%08x lr=%08x ctr=%08x @%llu; ring:\n",
                   cpu.st.pc, cpu.st.lr, cpu.st.ctr,
                   static_cast<unsigned long long>(executed));
            const u32 cnt = ring.n < 32 ? ring.n : 32;
            for (u32 k = 0; k < cnt; ++k) {
                const auto& e = ring.e[(ring.n - cnt + k) & 31u];
                disassemble(e.insn, e.pc, text, sizeof text, Style::Gnu);
                printf("   %08x: %08x  %s\n", e.pc, e.insn, text);
            }
            printf("-- wild: r3=%08x r4=%08x r5=%08x r12=%08x\n",
                   cpu.st.gpr[3], cpu.st.gpr[4], cpu.st.gpr[5],
                   cpu.st.gpr[12]);
            for (u32 r = 24; r < 32; ++r)
                printf("-- wild: r%u=%08x\n", r, cpu.st.gpr[r]);
            cpu.mmuProbe = true;
            const CpuState saved2 = cpu.st;
            const u32 blk = cpu.st.gpr[27] - 32; // the block just consumed
            for (u32 k = 0; k < 12; ++k) {
                cpu.st.msr |= 0x30u;
                u32 pa2 = 0;
                const bool ok = cpu.translate(blk + 4 * k, false, false, pa2);
                cpu.st = saved2;
                if (ok)
                    printf("-- wild: ctx[%2u] va=%08x pa=%08x val=%08x\n", k,
                           blk + 4 * k, pa2, bus.read32(pa2));
                else
                    printf("-- wild: ctx[%2u] va=%08x unmapped\n", k,
                           blk + 4 * k);
            }
            cpu.mmuProbe = false;
            printf("-- wild: watched writes to PA 004017c0-0040183f:\n");
            u32 ww = 0;
            for (const auto& wr : bus.htabLog()) {
                const u32 rawPa = wr.pa & 0x00FFFFFFu;
                if (rawPa < 0x004017C0u || rawPa > 0x0040183Fu)
                    continue;
                if (++ww > 60)
                    break;
                printf("   @%-11llu %s pa=%08x %08x -> %08x\n",
                       static_cast<unsigned long long>(wr.at),
                       (wr.pa & 0x80000000u) ? "b"
                       : (wr.pa & 0x40000000u) ? "h" : "w",
                       rawPa, wr.oldW, wr.newW);
            }
            printf("-- wild: total watched writes: %zu\n",
                   bus.htabLog().size());
        }
        if (cpu.raisedThisStep) {
            if (excLogged < excShow ||
                (executed >= 486000ull && executed <= 488100ull))
                printf("-- exc @%llu -> %08x from srr0=%08x srr1=%08x "
                       "dsisr=%08x dar=%08x\n",
                       static_cast<unsigned long long>(executed), cpu.st.pc,
                       cpu.st.srr0, cpu.st.srr1, cpu.st.dsisr, cpu.st.dar);
            ++excLogged;
            excRing[excRingAt % 16] = {executed, cpu.st.pc, cpu.st.srr0,
                                       cpu.st.srr1, cpu.st.dsisr, cpu.st.dar};
            ++excRingAt;
            if (!probed && cpu.st.dar == 0xFF808180u) {
                probed = true;
                probeDump();
            }
            if ((cpu.st.pc & 0xFFFu) == 0x500u && ext500Shown < 8) {
                ++ext500Shown;
                u32 slot = 0;
                const u32 sa = cpu.st.sprg[3] + 0x14u;
                if (!cpu.l1dPeek32(sa, slot) && !cpu.l2Peek32(sa, slot))
                    slot = bus.read32(sa);
                printf("-- ext irq #%u @%llu srr0=%08x srr1=%08x "
                       "[sprg3+14]=%08x\n",
                       ext500Shown,
                       static_cast<unsigned long long>(executed),
                       cpu.st.srr0, cpu.st.srr1, slot);
                if (ext500Shown == 1) {
                    printf("-- 0x500 handler body (phys, cache-aware):\n");
                    for (u32 k = 0; k < 40; ++k) {
                        const u32 a = (slot & ~3u) + 4 * k;
                        u32 w = 0;
                        if (!cpu.l1dPeek32(a, w) && !cpu.l2Peek32(a, w))
                            w = bus.read32(a);
                        disassemble(w, a, text, sizeof text, Style::Gnu);
                        printf("   %08x: %08x  %s\n", a, w, text);
                    }
                }
            }
            if (executed > 500000ull && cpu.st.pc == 0xFFF00300u &&
                late300Shown < 6) {
                ++late300Shown;
                u32 slot = 0;
                const u32 sa = cpu.st.sprg[3] + 12u;
                if (!cpu.l1dPeek32(sa, slot) && !cpu.l2Peek32(sa, slot))
                    slot = bus.read32(sa);
                printf("-- late dsi #%u @%llu srr0=%08x dar=%08x "
                       "sprg3=%08x [sprg3+12]=%08x\n",
                       late300Shown,
                       static_cast<unsigned long long>(executed),
                       cpu.st.srr0, cpu.st.dar, cpu.st.sprg[3], slot);
            }
            if (!mqProbed && executed > 500000ull &&
                cpu.st.pc == 0xFFF00800u) {
                mqProbed = true;
                printf("-- first late 0x800 @%llu: srr0=%08x srr1=%08x "
                       "lr=%08x r1=%08x\n",
                       static_cast<unsigned long long>(executed),
                       cpu.st.srr0, cpu.st.srr1, cpu.st.lr, cpu.st.gpr[1]);
                printf("-- sprg: %08x %08x %08x %08x\n", cpu.st.sprg[0],
                       cpu.st.sprg[1], cpu.st.sprg[2], cpu.st.sprg[3]);
                printf("-- [sprg3+0..0x3c]:");
                for (u32 k = 0; k < 16; ++k) {
                    const u32 a = cpu.st.sprg[3] + 4 * k;
                    u32 w = 0;
                    if (!cpu.l1dPeek32(a, w) && !cpu.l2Peek32(a, w))
                        w = bus.read32(a);
                    printf(" %08x", w);
                }
                printf("\n-- ring into it:\n");
                const u32 cnt = ring.n < 16 ? ring.n : 16;
                for (u32 k = 0; k < cnt; ++k) {
                    const auto& e = ring.e[(ring.n - cnt + k) & 31u];
                    disassemble(e.insn, e.pc, text, sizeof text, Style::Gnu);
                    printf("   %08x: %08x  %s\n", e.pc, e.insn, text);
                }
            }
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

    cpu.l1dFlushAll(true); // post-run dumps observe memory through the cache
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

    if (!inserterLog.empty()) {
        printf("-- pte ring-inserter consumes (%zu):\n", inserterLog.size());
        for (const auto& r : inserterLog)
            printf("   @%-9llu r8=%08x r9=%08x r10=%08x r12=%08x r13=%08x\n",
                   static_cast<unsigned long long>(r.at), r.r8, r.r9, r.r10,
                   r.r12, r.r13);
    }

    if (vdisStart && vdisEnd > vdisStart)
        vdump("at stop");

    if (!bus.htabLog().empty()) {
        printf("-- watched writes (%zu; first 40):\n", bus.htabLog().size());
        u32 shown = 0;
        for (const auto& wr : bus.htabLog()) {
            if (++shown > 40)
                break;
            printf("   @%-11llu %s pa=%08x %08x -> %08x\n",
                   static_cast<unsigned long long>(wr.at),
                   (wr.pa & 0x80000000u) ? "b"
                   : (wr.pa & 0x40000000u) ? "h" : "w",
                   wr.pa & 0x00FFFFFFu, wr.oldW, wr.newW);
        }
    }
    printf("-- ram at 9000:");
    for (u32 k = 0; k < 8; ++k)
        printf(" %08x", bus.read32(0x9000 + 4 * k));
    printf("\n");
    if (!bus.cfgLog().empty()) {
        printf("-- grackle config writes, memory-interface era (reg>=0x80):\n");
        u32 shown = 0;
        for (const auto& c : bus.cfgLog()) {
            if (c.reg < 0x80)
                continue;
            if (++shown > 120)
                break;
            printf("   @%-9llu %02x <= %02x\n",
                   static_cast<unsigned long long>(c.at), c.reg, c.val);
        }
    }
    if (bus.ml2Fills) {
        printf("-- inline L2: fills=%llu lost dirty castouts=%llu\n",
               static_cast<unsigned long long>(bus.ml2Fills),
               static_cast<unsigned long long>(bus.ml2LostCastouts));
        for (const auto& [at, pa2] : bus.ml2LossLog)
            printf("   @%-9llu lost line %08x\n",
                   static_cast<unsigned long long>(at), pa2);
    }
    if (const u8* g = bus.pci().cfgBytes(0)) {
        printf("-- grackle mem banks (cfg 0x80..0xA3):");
        for (u32 k = 0x80; k < 0xA4; ++k) {
            if ((k & 15u) == 0)
                printf("\n   %02x:", k);
            printf(" %02x", g[k]);
        }
        printf("\n");
    }

    if (!dispatches.empty()) {
        // Annotate each dispatch target with its word's token number by
        // scanning back for the "lhz r5, TOKEN(r23)" header (the OF heap
        // is RAM at VA-0xFF400000).
        auto tokenOf = [&](u32 va) -> int {
            if (va < 0xFF800000u)
                return -1;
            const u32 pa = va - 0xFF400000u;
            for (u32 back = 0; back <= 0x600; back += 4) {
                const u32 w = bus.read32(pa - back);
                if ((w & 0xFFFF0000u) == 0xA0B70000u ||
                    (w & 0xFFFF0000u) == 0xA0B80000u)
                    return static_cast<int>(w & 0xFFFFu);
                if (w == 0 && back > 0x40)
                    break;
            }
            return -1;
        };
        std::vector<size_t> pre; // dispatches before the wild jump
        for (size_t k = 0; k < dispatches.size(); ++k)
            if (dispatches[k].at < 187530054ull)
                pre.push_back(k);
        printf("-- threaded dispatches before the crash (last %zu of %zu):\n",
               pre.size() < 160 ? pre.size() : size_t(160), pre.size());
        const size_t start = pre.size() > 160 ? pre.size() - 160 : 0;
        for (size_t k = start; k < pre.size(); ++k) {
            const auto& d = dispatches[pre[k]];
            const int tok = tokenOf(d.ctr);
            printf("   @%-11llu %08x -> %08x tok=%03x\n",
                   static_cast<unsigned long long>(d.at), d.pc, d.ctr,
                   tok < 0 ? 0xFFFu : static_cast<u32>(tok));
        }
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
    {
        struct Ev {
            u64 at;
            bool io;
            u32 off;
            const AtiRage::Touch* t;
        };
        std::vector<Ev> evs;
        for (const auto& [off, t] : bus.ati().ioLog())
            evs.push_back({t.firstAt, true, off, &t});
        for (const auto& [off, t] : bus.ati().memLog())
            evs.push_back({t.firstAt, false, off, &t});
        std::sort(evs.begin(), evs.end(),
                  [](const Ev& a, const Ev& b) { return a.at < b.at; });
        if (!evs.empty()) {
            printf("-- ati register conversation (first-touch order):\n");
            for (const auto& e : evs)
                printf("   @%-11llu %s +%03x %-22s r=%-6llu w=%-6llu last=%08x\n",
                       static_cast<unsigned long long>(e.at),
                       e.io ? "io " : "mem", e.off, atiRegName(e.off),
                       static_cast<unsigned long long>(e.t->reads),
                       static_cast<unsigned long long>(e.t->writes),
                       e.t->lastWrite);
        }
    }
    if (!bus.ati().gpioHead().empty()) {
        auto show = [](const AtiRage::GpioOp& op) {
            printf("   @%-11llu %s.%u %08x pc=%08x lr=%08x\n",
                   static_cast<unsigned long long>(op.at),
                   op.write ? "wr" : "rd", op.lane, op.val, op.pc, op.lr);
        };
        printf("-- ati GP_IO conversation (%llu ops; head):\n",
               static_cast<unsigned long long>(bus.ati().gpioOps()));
        for (const auto& op : bus.ati().gpioHead())
            show(op);
        const auto tail = bus.ati().gpioTail();
        if (!tail.empty()) {
            printf("-- ati GP_IO tail (last %zu):\n", tail.size());
            for (const auto& op : tail)
                show(op);
        }
    }
    {
        const AtiRage::Mode m = bus.ati().mode();
        printf("-- ati crtc: %ux%u@%ubpp pitch=%upx offset=%u enabled=%d\n",
               m.width, m.height, m.bpp, m.pitchPixels, m.offsetBytes,
               m.enabled ? 1 : 0);
        u64 nonzero = 0;
        for (u8 b : bus.ati().vram())
            if (b)
                ++nonzero;
        printf("-- ati vram: %llu nonzero bytes\n",
               static_cast<unsigned long long>(nonzero));
        if (fbPath)
            dumpFramebuffer(fbPath, bus.ati());
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
    if (!bus.macio().sndTrace().empty()) {
        printf("-- sound-region conversation (%zu):\n",
               bus.macio().sndTrace().size());
        for (const auto& s : bus.macio().sndTrace())
            printf("   @%-11llu %s +%05x %02x pc=%08x\n",
                   static_cast<unsigned long long>(s.at),
                   s.write ? "wr" : "rd", s.off, s.val, s.pc);
    }
    if (!bus.macio().picTrace().empty()) {
        printf("-- pic writes:\n");
        for (const auto& p : bus.macio().picTrace())
            printf("   @%-11llu +%02x <= %02x\n",
                   static_cast<unsigned long long>(p.at), p.off, p.val);
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
