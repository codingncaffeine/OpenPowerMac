#include "opm/capi.h"

#include "opm/cpu.hpp"
#include "opm/hostclock.hpp"
#include "opm/pace.hpp"
#include "opm/sawtooth.hpp"

#include <chrono>
#include <thread>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace opm;

struct OpmMachine {
    SawtoothBus* bus = nullptr;
    Cpu cpu;
    // 60x bus snooping for every DMA engine in the machine. Wired before
    // the first instruction: a DBDMA channel with no snoop responder reads
    // its descriptor list out of RAM while the firmware's stores are still
    // sitting dirty in the L1, and transfers into memory the processor is
    // still caching.
    CpuSnoop snoop;
    uint64_t executed = 0;
    uint32_t fastTb = 0;
    size_t consoleAt = 0; // drained-up-to mark
    // Real-time pacing: the timebase follows the HOST CLOCK instead of the
    // instruction count. This is what makes the guest's clock true — with it
    // on, the machine's own 60 Hz tick chain emits 60 Ticks per HOST second
    // whatever this host's throughput turns out to be, and the emulator gets
    // (instructions per second)/60 instructions to spend on each of them.
    // Instruction pacing cannot promise either: it fixes a tb-per-instruction
    // ratio, so a faster host runs guest time faster, and past some speed the
    // 60 Hz work no longer fits in the interval it is given. See the note on
    // SawtoothBus::klTimerOn, and opm/hostclock.hpp for who owns the clock.
    bool realtime = false;
    HostPacer pace;
    // The other half of owning the clock: what this thread does when the guest
    // has nothing for it. It carries a timer handle and the learned cost of
    // asking for a wait, so it belongs to the machine rather than to one call.
    HostWait idle;

    ~OpmMachine() { delete bus; }
};

static std::vector<u8> slurp(const char* path)
{
    std::vector<u8> v;
    FILE* f = fopen(path, "rb");
    if (!f)
        return v;
    fseek(f, 0, SEEK_END);
    const long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    v.assign(static_cast<size_t>(n > 0 ? n : 0), 0);
    if (!v.empty() && fread(v.data(), 1, v.size(), f) != v.size())
        v.clear();
    fclose(f);
    return v;
}

OPM_API OpmMachine* opm_create(const char* romPath, const char* cdPath,
                               const char* hdPath, const char* atiRomPath,
                               uint32_t ramMb, uint32_t fastTb)
{
    std::vector<u8> rom = slurp(romPath);
    if (rom.size() != SawtoothBus::kRomSize)
        return nullptr;
    OpmMachine* m = new OpmMachine();
    m->bus = new SawtoothBus(size_t(ramMb ? ramMb : 256) * 1024 * 1024,
                             std::move(rom));
    if (cdPath && *cdPath)
        m->bus->attachCd(cdPath);
    if (hdPath && *hdPath)
        m->bus->attachHd(hdPath);
    if (atiRomPath && *atiRomPath)
        m->bus->attachAtiRom(atiRomPath);
    m->fastTb = fastTb;
    m->cpu.attach(*m->bus);
    m->snoop.cpu = &m->cpu;
    m->bus->attachSnoop(&m->snoop);
    m->cpu.reset();
    // ⚠ EVERY CELL, THROUGH THE BUS. This was a hand-written list and it was
    // missing hd() and hdDma() — see SawtoothBus::setStamp. The app had no
    // working hard disk at all: the drive raised BSY on its first command and
    // never lowered it, which is exactly what Mac OS reports as a disk it
    // cannot read and offers to initialise.
    m->bus->setPcRef(&m->cpu.st.pc);
    m->bus->setStamp(&m->executed);
    return m;
}

OPM_API void opm_destroy(OpmMachine* m) { delete m; }

OPM_API void opm_ati_at(OpmMachine* m, uint64_t insn)
{
    m->bus->atiVisibleAt = insn;
}

OPM_API uint64_t opm_run(OpmMachine* m, uint64_t insns)
{
    const uint64_t until = m->executed + insns;
    Cpu& cpu = m->cpu;
    SawtoothBus& bus = *m->bus;
    // The harness's clock advance rides along with the architectural one
    // inside step() instead of being a second call per instruction — see
    // Cpu::extraCycles. Set here rather than at create so that whatever
    // changes fastTb is honoured on the next entry.
    cpu.extraCycles = m->realtime ? 0u : m->fastTb;
    // ⏱⏱ AND THE ARCHITECTURAL CYCLE TOO. Real-time pacing means the HOST
    // CLOCK owns the timebase; leaving the per-instruction cycle in place left
    // the emulator's own throughput as a FLOOR under the guest's clock — see
    // Cpu::insnCycles for the measurement that named it — so the clock the
    // guest reads is now a function of nothing but elapsed host time.
    cpu.insnCycles = m->realtime ? 0u : 1u;
    if (m->realtime) {
        // How long this call may spend WAITING before it hands control back to
        // its caller, whether or not the guest woke up.
        //
        // ⚠ It is not a pacing constant and nothing about the clock depends on
        // it. It bounds LATENCY: the caller is a UI thread's partner and needs
        // this back promptly to pump the keyboard, the mouse, the screen and
        // the audio drain, and a guest can idle for a long time. It also stops
        // a core that naps with interrupts disabled — a machine that would be
        // hung on real hardware too — from hanging the app's run thread with
        // it, which would look like the app crashing rather than like the
        // machine stopping.
        //
        // ⭐ AND THEN IT RETURNS, RATHER THAN RUNNING OUT THE REST OF THE
        // BUDGET. Draining the remaining chunk through an asleep core is a
        // spin at full speed — it was measured at 79% of a host core with the
        // guest idle at the Finder — and it buys nothing: those steps execute
        // no instruction and, now that the host clock owns the timebase, do
        // not advance it either. They are pure heat. Returning early means a
        // call can legitimately end with the instruction count UNMOVED, which
        // is why opm_halted exists: the shell used to infer "halted" from
        // exactly that and would have stopped an idle machine.
        constexpr uint64_t kMaxIdleNsPerCall = 15000000; // 15 ms
        uint64_t idleNs = 0;
        while (m->executed < until && !cpu.halted) {
            // 1. THE CLOCK, FROM OUTSIDE. Nothing else moves it now.
            m->pace.sync(cpu);
            // 2. The devices, in the same breath, because the clock just
            //    crossed whatever deadlines it crossed and an interrupt that
            //    is due should reach the processor before more instructions
            //    run, not after them.
            cpu.setExternalIrq(bus.serviceDevices(cpu.st.tb));
            // 3. ⏳ NOTHING TO DO — so give the host processor back until the
            //    first moment something could happen, instead of spinning
            //    through millions of asleep steps to discover the same thing.
            //    This is the third face of the pacing bug and the one that
            //    reads as a shell problem: the app burned a whole core while
            //    the guest sat idle at the Finder. Going round again rather
            //    than running: the clock has moved by the length of the wait,
            //    so the devices have to be looked at before anything else.
            if (const uint64_t waited = hostIdleWait(cpu, bus, m->idle)) {
                idleNs += waited;
                if (idleNs >= kMaxIdleNsPerCall)
                    break; // idle, and the caller has been kept long enough
                continue;
            }
            // 4. A run of instructions. Nothing in the machine can happen
            //    inside it — executing does not advance the clock any more, so
            //    neither the decrementer nor a timed device can come due — and
            //    the things that end it early end it from the inside: a device
            //    access, a write to DEC or the timebase, the core going to
            //    sleep. See machine/include/opm/pace.hpp.
            //
            //    ⚠ SO THE CAP IS ABOUT THE CLOCK'S GRAIN, NOT ABOUT
            //    CORRECTNESS — see HostPacer::kBatchInsns, which g4run's
            //    --realtime loop uses too.
            const uint64_t left = until - m->executed;
            runBatch(cpu, bus, m->executed,
                     left < HostPacer::kBatchInsns ? left
                                                   : HostPacer::kBatchInsns);
        }
        return m->executed;
    }
    while (m->executed < until && !cpu.halted) {
        // An asleep core executes nothing, and nothing that could wake it can
        // happen before the deadlines this asks about — so charge the whole
        // run of sleeping steps at once. At the Finder desktop that is most
        // of them. See machine/include/opm/pace.hpp.
        if (const u64 n = napSkip(cpu, bus, until - m->executed)) {
            m->executed += n;
            cpu.setExternalIrq(bus.serviceDevices(cpu.st.tb));
            continue;
        }
        // A run of instructions with the clock charged once and the devices
        // looked at once, bounded by the first moment either could matter.
        runBatch(cpu, bus, m->executed, until - m->executed);
        // One call, and it does nothing at all unless a device could have
        // moved — see SawtoothBus::serviceDevices. Ticking every device and
        // recomputing seven interrupt lines on every emulated instruction was
        // a quarter of the whole emulator's host time.
        cpu.setExternalIrq(bus.serviceDevices(cpu.st.tb));
    }
    return m->executed;
}

OPM_API void opm_set_realtime(OpmMachine* m, int32_t on)
{
    m->realtime = on != 0;
    m->pace.anchor(m->cpu);
}

OPM_API uint64_t opm_rt_slips(const OpmMachine* m) { return m->pace.slips; }

// ⛔ Asked directly, because it can no longer be inferred. opm_run returns the
// same instruction count it started with whenever the guest spent the call
// idle, which under real-time pacing is most calls at the Finder desktop.
OPM_API int32_t opm_halted(const OpmMachine* m) { return m->cpu.halted ? 1 : 0; }

// ⏳ Time this machine spent OFF the host processor. HostWait counts it at the
// one place it can be counted honestly — around the system call itself — so
// this is measured, not modelled.
OPM_API uint64_t opm_idle_ns(const OpmMachine* m) { return m->idle.waitedNs; }

// The guest's own clock. Divided by 25,000,000 it is the machine's uptime in
// its own seconds, and divided by host seconds it says whether this machine
// is running at real time — the only way to check pacing from outside.
OPM_API uint64_t opm_tb(const OpmMachine* m) { return m->cpu.st.tb; }

// ⚠ Every entry point that pokes a device from OUTSIDE the run loop has to
// say so, or the loop's device-service gate will keep answering from its
// cache. Input arrives this way — a keystroke the machine never notices is
// exactly the class of bug this note exists to prevent.
OPM_API void opm_serial(OpmMachine* m, const char* text)
{
    if (text && *text) {
        m->bus->injectSerial(text);
        m->bus->deviceStateChanged();
    }
}

OPM_API void opm_key(OpmMachine* m, const char* text)
{
    if (text && *text) {
        m->bus->ohci(0).typeAscii(text);
        m->bus->deviceStateChanged();
    }
}

OPM_API void opm_key_event(OpmMachine* m, uint32_t usage, uint32_t down)
{
    m->bus->ohci(0).keyEvent(static_cast<uint8_t>(usage), down != 0u);
    m->bus->deviceStateChanged();
}

OPM_API void opm_mouse(OpmMachine* m, int32_t dx, int32_t dy,
                       uint32_t buttons)
{
    m->bus->ohci(1).moveMouse(dx, dy, static_cast<uint8_t>(buttons));
    m->bus->deviceStateChanged();
}

OPM_API uint32_t opm_diag(OpmMachine* m, char* buf, uint32_t cap)
{
    if (!buf || !cap)
        return 0;
    const Cpu& cpu = m->cpu;
    char b[512];
    std::string s = "\n===== machine diagnostics =====\n";
    snprintf(b, sizeof b,
             "-- cpu: pc=%08x msr=%08x executed=%llu tb=%llu dec=%08x\n"
             "--   napping=%d halted=%d%s%s\n",
             cpu.st.pc, cpu.st.msr,
             static_cast<unsigned long long>(m->executed),
             static_cast<unsigned long long>(cpu.st.tb), cpu.st.dec,
             cpu.napping ? 1 : 0, cpu.halted ? 1 : 0,
             cpu.halted ? " reason=" : "",
             cpu.halted ? cpu.haltReason.c_str() : "");
    s += b;
    snprintf(b, sizeof b, "--   irq lines: ext=%d dec=%d smi=%d pm=%d\n",
             cpu.extIrqLine ? 1 : 0, cpu.decPending ? 1 : 0,
             cpu.smiPending ? 1 : 0, cpu.pmPending ? 1 : 0);
    s += b;
    // ⏳ Whether this process is waiting or spinning, which says whether the
    // guest is asleep-and-idle or awake-and-stuck. They look the same from a
    // progress bar that stopped moving.
    snprintf(b, sizeof b,
             "-- pacing: realtime=%d slips=%llu idleWaits=%llu idleMs=%llu "
             "skipped=%llu askCostUs=%llu\n",
             m->realtime ? 1 : 0,
             static_cast<unsigned long long>(m->pace.slips),
             static_cast<unsigned long long>(m->idle.waits),
             static_cast<unsigned long long>(m->idle.waitedNs / 1000000ull),
             static_cast<unsigned long long>(m->idle.skipped),
             static_cast<unsigned long long>(m->idle.overshootNs / 1000ull));
    s += b;
    s += m->bus->pic().describe();
    s += m->bus->hd().describe("hd");
    s += m->bus->cd().describe("cd");
    s += "===== end =====\n";

    uint32_t n = 0;
    while (n + 1 < cap && n < s.size()) {
        buf[n] = s[n];
        ++n;
    }
    buf[n] = 0;
    return n;
}

OPM_API uint32_t opm_console(OpmMachine* m, char* buf, uint32_t cap)
{
    const std::string& con = m->bus->console();
    if (m->consoleAt > con.size())
        m->consoleAt = con.size(); // defensive
    uint32_t n = 0;
    while (m->consoleAt < con.size() && n + 1 < cap)
        buf[n++] = con[m->consoleAt++];
    if (cap)
        buf[n] = 0;
    return n;
}

OPM_API uint32_t opm_audio(OpmMachine* m, uint8_t* out, uint32_t cap)
{
    if (!m || !out || cap < 4u)
        return 0;
    // Whole frames only: a caller handed three bytes would have the two
    // channels out of step for the rest of the stream.
    return static_cast<uint32_t>(m->bus->sound().drain(out, cap & ~3u));
}

OPM_API uint32_t opm_audio_rate(const OpmMachine* m)
{
    return m ? m->bus->sound().rateHz() : 0u;
}

OPM_API uint64_t opm_audio_played(const OpmMachine* m)
{
    return m ? m->bus->sound().bytesPlayed() : 0ull;
}

OPM_API int32_t opm_screen(OpmMachine* m, uint8_t* bgra, uint32_t cap,
                           uint32_t* w, uint32_t* h)
{
    R128Cell& ati = m->bus->ati();
    const u32 gen = ati.peek(0x0050);
    if (!(gen & 0x02000000u))
        return -1;
    const u32 ht = ati.peek(0x0200);
    const u32 vt = ati.peek(0x0208);
    const u32 pitch8 = ati.peek(0x022C) & 0xFFFFu;
    const u32 offset = ati.peek(0x0224);
    const u32 fmt = (gen >> 8) & 0xFu;
    const u32 sw = (((ht >> 16) & 0x3FFu) + 1u) * 8u;
    const u32 sh = ((vt >> 16) & 0xFFFu) + 1u;
    if (sw < 64 || sw > 2048 || sh < 64 || sh > 1536 ||
        (fmt != 2u && fmt != 6u))
        return -1;
    if (w)
        *w = sw;
    if (h)
        *h = sh;
    if (!bgra || cap < sw * sh * 4u)
        return 0;
    const u32 bypp = fmt == 2u ? 1u : 4u;
    const u32 rowBytes = pitch8 * 8u * bypp;
    const auto& vr = ati.vram;
    for (u32 y = 0; y < sh; ++y) {
        uint8_t* out = bgra + size_t(y) * sw * 4u;
        const size_t row = offset + size_t(y) * rowBytes;
        for (u32 x = 0; x < sw; ++x) {
            uint8_t b = 0, g = 0, r = 0;
            const size_t o = row + size_t(x) * bypp;
            if (o + bypp <= vr.size()) {
                if (fmt == 2u) {
                    const u32 c = ati.pal(vr[o]);
                    r = static_cast<uint8_t>(c >> 16);
                    g = static_cast<uint8_t>(c >> 8);
                    b = static_cast<uint8_t>(c);
                } else {
                    // A Mac 32-bpp pixel is big-endian xRGB: byte 0 is the
                    // unused/alpha lane and R, G, B follow. Reading bytes
                    // 0,1,2 as B,G,R put the unused lane in blue and shifted
                    // the other two, so a 50% grey desktop (00 80 80 80) came
                    // out as r=80 g=80 b=00 -- olive. The whole screen was
                    // yellow, which is what named the bug.
                    r = vr[o + 1];
                    g = vr[o + 2];
                    b = vr[o + 3];
                }
            }
            out[x * 4 + 0] = b;
            out[x * 4 + 1] = g;
            out[x * 4 + 2] = r;
            out[x * 4 + 3] = 0xFF;
        }
    }

    // The hardware cursor. Mac OS draws its pointer with the card's cursor
    // engine, not into the framebuffer, so a scanout that reads VRAM alone
    // shows no pointer at all however well the USB side works -- which is
    // exactly what it looked like from the outside.
    //
    // 64x64 at two bits per pixel, 16 bytes per row: eight bytes of AND bits
    // then eight of XOR bits, each big-endian with the most significant bit
    // leftmost. AND=0 selects colour 0 or 1 from XOR; AND=1 with XOR=0 is
    // transparent, and with XOR=1 complements what is underneath -- which is
    // how the classic arrow keeps its black outline over any background.
    // CUR_HORZ_VERT_OFF is how many cursor rows/columns to skip, which is how
    // the driver clips the pointer against the top and left edges.
    if (ati.peek(0x0050) & 0x00010000u) { // CRTC_GEN_CNTL: cursor enable
        const u32 curOff = ati.peek(0x0260) & 0x07FFFFFFu;
        const u32 posn = ati.peek(0x0264);
        const u32 hvoff = ati.peek(0x0268);
        const u32 clr[2] = {ati.peek(0x026C), ati.peek(0x0270)};
        const u32 cx = (posn >> 16) & 0xFFFFu, cy = posn & 0xFFFFu;
        const u32 ox = (hvoff >> 16) & 0x3Fu, oy = hvoff & 0x3Fu;
        for (u32 row = oy; row < 64u; ++row) {
            const u32 sy = cy + (row - oy);
            if (sy >= sh)
                break;
            const size_t so = size_t(curOff) + size_t(row) * 16u;
            if (so + 16u > vr.size())
                break;
            uint64_t abits = 0, xbits = 0;
            for (u32 k = 0; k < 8; ++k) {
                abits = (abits << 8) | vr[so + k];
                xbits = (xbits << 8) | vr[so + 8u + k];
            }
            for (u32 col = ox; col < 64u; ++col) {
                const u32 sx = cx + (col - ox);
                if (sx >= sw)
                    break;
                const uint64_t bit = uint64_t(1) << (63u - col);
                uint8_t* px = bgra + (size_t(sy) * sw + sx) * 4u;
                if (abits & bit) {
                    if (!(xbits & bit))
                        continue; // transparent
                    px[0] = static_cast<uint8_t>(~px[0]);
                    px[1] = static_cast<uint8_t>(~px[1]);
                    px[2] = static_cast<uint8_t>(~px[2]);
                } else {
                    const u32 c = clr[(xbits & bit) ? 1 : 0];
                    px[0] = static_cast<uint8_t>(c);
                    px[1] = static_cast<uint8_t>(c >> 8);
                    px[2] = static_cast<uint8_t>(c >> 16);
                }
                px[3] = 0xFF;
            }
        }
    }
    return 1;
}

OPM_API uint64_t opm_executed(const OpmMachine* m) { return m->executed; }
OPM_API uint32_t opm_pc(const OpmMachine* m) { return m->cpu.st.pc; }
