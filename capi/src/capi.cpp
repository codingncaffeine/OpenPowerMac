#include "opm/capi.h"

#include "opm/cpu.hpp"
#include "opm/hostclock.hpp"
#include "opm/pace.hpp"
#include "opm/sawtooth.hpp"

#include <algorithm>
#include <chrono>
#include <thread>
#include <utility>
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
    // A fresh machine gets a fresh command-stream parser: the shell calls
    // this on every Start of the same process, and a previous machine that
    // stopped mid-packet would otherwise poison the new stream (see
    // r128CceReset in opm/r128.hpp).
    r128CceReset();
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

// The exception vectors by name. A report that prints 900 and lets the reader
// remember what that is will eventually be misread — it was, once, as a DSI.
static const char* excName(u32 v)
{
    switch (v) {
    case 0x100: return "RESET";
    case 0x200: return "MACHINE-CHECK";
    case 0x300: return "DSI";
    case 0x400: return "ISI";
    case 0x500: return "EXTERNAL";
    case 0x600: return "ALIGNMENT";
    case 0x700: return "PROGRAM";
    case 0x800: return "FP-UNAVAIL";
    case 0x900: return "DECREMENTER";
    case 0xC00: return "SYSCALL";
    case 0xD00: return "TRACE";
    case 0xF00: return "PERFMON";
    case 0xF20: return "ALTIVEC";
    default: return "?";
    }
}

// ⚠ ONLY THESE WRITE DAR/DSISR. Every other vector leaves whatever the last
// data fault put there, so printing those fields beside a DECREMENTER shows a
// stale address that looks exactly like a live one.
static bool excSetsDar(u32 v) { return v == 0x300 || v == 0x600; }

// 🔎 A READ-ONLY address translation, for dumping code the guest is running.
//
// ⛔ Cpu::translate CANNOT be used here even with mmuProbe: its failure paths
// call raiseExc, so a diagnostic that probed an unmapped address would vector
// the machine it was trying to observe. An instrument that mutates its subject
// invents bugs — so this walks BATs and the page table by hand, reading RAM
// only. ⚠ Same caveat as the PTE dump: the real walk reads through the data
// cache, so a mapping written and not yet flushed is invisible here.
static bool diagXlate(const Cpu& cpu, SawtoothBus& bus, u32 ea, bool fetch,
                      u32& pa)
{
    const u32* batu = fetch ? cpu.st.ibatu : cpu.st.dbatu;
    const u32* batl = fetch ? cpu.st.ibatl : cpu.st.dbatl;
    for (u32 i = 0; i < 4; ++i) {
        const u32 u = batu[i], l = batl[i];
        if (!(u & 3u)) // neither Vs nor Vp
            continue;
        const u32 bl = (u >> 2) & 0x7FFu;
        const u32 mask = ~((bl << 17) | 0x1FFFFu);
        if ((ea & mask) != (u & mask & 0xFFFE0000u))
            continue;
        pa = (l & 0xFFFE0000u) | (ea & ~mask);
        return true;
    }
    const u32 sr = cpu.st.sr[ea >> 28];
    if (sr & 0x80000000u)
        return false; // direct-store
    const u32 vsid = sr & 0x00FFFFFFu;
    const u32 pi = (ea >> 12) & 0xFFFFu;
    const u32 api = pi >> 10;
    const u32 htaborg = cpu.st.sdr1 & 0xFFFF0000u;
    const u32 htabmask = cpu.st.sdr1 & 0x1FFu;
    for (int pass = 0; pass < 2; ++pass) {
        const u32 hash = pass == 0 ? ((vsid & 0x7FFFFu) ^ pi)
                                   : (~((vsid & 0x7FFFFu) ^ pi) & 0x7FFFFu);
        const u32 pteg = htaborg |
                         ((((hash >> 10) & 0x1FFu) & htabmask) << 16) |
                         ((hash & 0x3FFu) << 6);
        for (u32 slot = 0; slot < 8; ++slot) {
            const u32 w0 = bus.read32(pteg + slot * 8u);
            if (!(w0 & 0x80000000u))
                continue;
            if (((w0 >> 7) & 0xFFFFFFu) != vsid)
                continue;
            if (((w0 >> 6) & 1u) != static_cast<u32>(pass))
                continue;
            if ((w0 & 0x3Fu) != api)
                continue;
            pa = (bus.read32(pteg + slot * 8u + 4) & 0xFFFFF000u) |
                 (ea & 0xFFFu);
            return true;
        }
    }
    return false;
}

// 🧭 THE PAGE-TABLE WALK FOR ONE ADDRESS, SPELLED OUT.
//
// Factored out because it was only ever pointed at DAR, and the address that
// most needed it was the pc: "code at pc 3e894d70: NOT MAPPED" is where a
// capture stopped being useful, when the machine was demonstrably EXECUTING
// there. An address the CPU is running from that the page table does not
// describe is either a stale translation or a walk that disagrees with the
// hardware's, and both are findings — but only if the walk shows its working.
static void dumpMmu(std::string& s, const Cpu& cpu, SawtoothBus& bus, u32 ea,
                    const char* what)
{
    char b[512];
    const u32 sr = cpu.st.sr[ea >> 28];
    const u32 vsid = sr & 0x00FFFFFFu;
    const u32 pi = (ea >> 12) & 0xFFFFu;
    const u32 api = pi >> 10;
    const u32 htaborg = cpu.st.sdr1 & 0xFFFF0000u;
    const u32 htabmask = cpu.st.sdr1 & 0x1FFu;
    snprintf(b, sizeof b,
             "-- mmu for %s=%08x: sr[%u]=%08x vsid=%06x pi=%04x api=%02x "
             "sdr1=%08x (org=%08x mask=%03x)\n",
             what, ea, ea >> 28, sr, vsid, pi, api, cpu.st.sdr1, htaborg,
             htabmask);
    s += b;
    // A BAT covering this address would have answered before the table was
    // consulted, so their absence is part of the evidence. Both sets: an
    // INSTRUCTION address is answered by the IBATs, and printing only the
    // DBATs beside a pc invites the wrong conclusion.
    for (u32 i = 0; i < 4; ++i) {
        snprintf(b, sizeof b, "--   bat%u: i u=%08x l=%08x   d u=%08x l=%08x\n",
                 i, cpu.st.ibatu[i], cpu.st.ibatl[i], cpu.st.dbatu[i],
                 cpu.st.dbatl[i]);
        s += b;
    }
    for (int pass = 0; pass < 2; ++pass) {
        const u32 hash = pass == 0 ? ((vsid & 0x7FFFFu) ^ pi)
                                   : (~((vsid & 0x7FFFFu) ^ pi) & 0x7FFFFu);
        const u32 pteg = htaborg | ((((hash >> 10) & 0x1FFu) & htabmask) << 16) |
                         ((hash & 0x3FFu) << 6);
        snprintf(b, sizeof b, "--   %s pteg @%08x (hash=%05x):\n",
                 pass ? "secondary" : "primary  ", pteg, hash);
        s += b;
        for (u32 slot = 0; slot < 8; ++slot) {
            const u32 a = pteg + slot * 8u;
            const u32 w0 = bus.read32(a);
            const u32 w1 = bus.read32(a + 4);
            const bool v = (w0 & 0x80000000u) != 0;
            const u32 eVsid = (w0 >> 7) & 0xFFFFFFu;
            const u32 eH = (w0 >> 6) & 1u;
            const u32 eApi = w0 & 0x3Fu;
            const bool match =
                v && eVsid == vsid && eH == static_cast<u32>(pass) && eApi == api;
            snprintf(b, sizeof b,
                     "--     %u %08x %08x  v=%d vsid=%06x h=%u api=%02x "
                     "rpn=%05x pp=%u%s\n",
                     slot, w0, w1, v ? 1 : 0, eVsid, eH, eApi, w1 >> 12,
                     w1 & 3u, match ? "   <== MATCHES" : "");
            s += b;
        }
    }
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
    // ⭐⭐ WHAT THE MACHINE FAULTED ON. A guest that stops with EE and the two
    // translation bits clear has not hung waiting for a device — it is sitting
    // in an exception handler, in exactly the state the hardware enters one:
    // EE=0, PR=0, IR=0, DR=0, RI=0, ME preserved. That reads from outside as a
    // frozen machine (no interrupts means no cursor and no ticks) and it is a
    // CRASH, which is a completely different investigation.
    //
    // ⚠ RI=0 is what makes these registers trustworthy: it says the handler
    // has not re-armed for a nested exception, so srr0 is still the faulting
    // instruction and srr1 still the MSR it faulted under.
    snprintf(b, sizeof b,
             "-- exception context: srr0=%08x srr1=%08x dar=%08x dsisr=%08x\n"
             "--   lr=%08x ctr=%08x sprg=%08x %08x %08x %08x\n",
             cpu.st.srr0, cpu.st.srr1, cpu.st.dar, cpu.st.dsisr, cpu.st.lr,
             cpu.st.ctr, cpu.st.sprg[0], cpu.st.sprg[1], cpu.st.sprg[2],
             cpu.st.sprg[3]);
    s += b;
    {
        const u32 msrv = cpu.st.msr, r = cpu.st.srr1;
        snprintf(b, sizeof b,
                 "--   msr: EE=%d PR=%d IR=%d DR=%d RI=%d ME=%d FP=%d%s\n",
                 (msrv & 0x8000u) ? 1 : 0, (msrv & 0x4000u) ? 1 : 0,
                 (msrv & 0x0020u) ? 1 : 0, (msrv & 0x0010u) ? 1 : 0,
                 (msrv & 0x0002u) ? 1 : 0, (msrv & 0x1000u) ? 1 : 0,
                 (msrv & 0x2000u) ? 1 : 0,
                 (!(msrv & 0x8000u) && !(msrv & 0x0020u) && !(msrv & 0x0010u))
                     ? "   <-- EXCEPTION CONTEXT, not a device wait"
                     : "");
        s += b;
        // Only the Program-exception bits are self-describing; a DSI is named
        // by DSISR and the address it faulted on, which is why DAR is above.
        snprintf(b, sizeof b, "--   srr1 flags:%s%s%s%s%s\n",
                 (r & kSrr1ProgIllegal) ? " ILLEGAL" : "",
                 (r & kSrr1ProgPrivileged) ? " PRIVILEGED" : "",
                 (r & kSrr1ProgTrap) ? " TRAP" : "",
                 (r & kSrr1ProgFpEnabled) ? " FP-ENABLED" : "",
                 (r & (kSrr1ProgIllegal | kSrr1ProgPrivileged |
                       kSrr1ProgTrap | kSrr1ProgFpEnabled))
                     ? ""
                     : " (none set: not a Program exception)");
        s += b;
    }
    // ⏳ Whether this process is waiting or spinning, which says whether the
    // guest is asleep-and-idle or awake-and-stuck. They look the same from a
    // progress bar that stopped moving.
    snprintf(b, sizeof b,
             "-- pacing: realtime=%d slips=%llu rebases=%llu idleWaits=%llu "
             "idleMs=%llu skipped=%llu askCostUs=%llu\n",
             m->realtime ? 1 : 0,
             static_cast<unsigned long long>(m->pace.slips),
             static_cast<unsigned long long>(m->pace.rebases),
             static_cast<unsigned long long>(m->idle.waits),
             static_cast<unsigned long long>(m->idle.waitedNs / 1000000ull),
             static_cast<unsigned long long>(m->idle.skipped),
             static_cast<unsigned long long>(m->idle.overshootNs / 1000ull));
    s += b;
    // 📓 WHAT IT HAS BEEN DOING, not just where it stopped. A handler entered
    // once and a fault re-taken forever look identical in a single sample.
    {
        const u64 n = cpu.excRingAt < Cpu::kExcRing ? cpu.excRingAt
                                                    : Cpu::kExcRing;
        snprintf(b, sizeof b,
                 "-- last %llu of %llu exceptions (oldest first; vec srr0 srr1 "
                 "dar dsisr @tb):\n",
                 static_cast<unsigned long long>(n),
                 static_cast<unsigned long long>(cpu.excRingAt));
        s += b;
        // The histogram first: which vector dominates, and at what rate. One
        // exception per few thousand instructions is a working machine; one
        // per few hundred is something drowning.
        s += "--   by vector:";
        for (u32 vi = 0; vi < 16; ++vi) {
            if (!cpu.excByVec[vi])
                continue;
            snprintf(b, sizeof b, " %s=%llu", excName(vi << 8),
                     static_cast<unsigned long long>(cpu.excByVec[vi]));
            s += b;
        }
        snprintf(b, sizeof b, "\n--   rate: 1 per %llu instructions\n",
                 static_cast<unsigned long long>(
                     cpu.excRingAt ? m->executed / cpu.excRingAt : 0));
        s += b;
        for (u64 k = 0; k < n; ++k) {
            const Cpu::ExcRec& r =
                cpu.excRing[(cpu.excRingAt - n + k) & (Cpu::kExcRing - 1u)];
            // ⚠ DAR/DSISR only for the vectors that actually write them —
            // see excSetsDar. Printing them beside a DECREMENTER shows the
            // last data fault's address looking exactly like a live one, and
            // that misread a whole capture once.
            if (excSetsDar(r.vec))
                snprintf(b, sizeof b,
                         "--   %03x %-12s srr0=%08x srr1=%08x dar=%08x "
                         "dsisr=%08x lr=%08x @%llu\n",
                         r.vec, excName(r.vec), r.srr0, r.srr1, r.dar, r.dsisr,
                         r.lr, static_cast<unsigned long long>(r.tb));
            else if (r.vec == 0xC00)
                // 📞 A SYSCALL IS A QUESTION, SO PRINT THE QUESTION. r0 is the
                // NanoKernel selector and lr is who asked; srr0 is only ever
                // the instruction after `sc`, which is the least informative
                // field of the three and was the only one printed.
                snprintf(b, sizeof b,
                         "--   %03x %-12s srr0=%08x sel=r0:%-10d lr=%08x @%llu\n",
                         r.vec, excName(r.vec), r.srr0,
                         static_cast<int>(r.r0), r.lr,
                         static_cast<unsigned long long>(r.tb));
            else
                snprintf(b, sizeof b,
                         "--   %03x %-12s srr0=%08x srr1=%08x %-16s lr=%08x @%llu\n",
                         r.vec, excName(r.vec), r.srr0, r.srr1,
                         "(no dar/dsisr)", r.lr,
                         static_cast<unsigned long long>(r.tb));
            s += b;
        }
    }
    // 🧠 THE NANOKERNEL BLOCK THE `sc -1` FAST PATH READS.
    //
    // Mac OS's syscall vector at physical 0xC00 saves r1/lr into SPRG1/SPRG2
    // and jumps through a table of handlers at [SPRG3 + vector>>8*4]; entry 12
    // (0xC00) is the dispatcher, and it fast-paths exactly three selectors
    // before the general save-everything path:
    //
    //     cmpwi r0,-1 ; mfspr r1,272 (SPRG0)
    //     lwz    r0,-16(r1)          <- a per-CPU flags word below SPRG0
    //     rlwinm. r0,r0,0,10,10      <- keeps ONE bit, 0x00200000, sets CR0
    //     rfi
    //
    // and the ROM stub that calls it is `li r0,-1 ; sc ; li r3,1 ; beqlr ;
    // li r3,0` — so it returns TRUE exactly when that bit is CLEAR. Measured on
    // a healthy desktop the word is 0x00280006, bit SET, stub answers 0, and
    // the ROM predicate that consults it takes its good path. Print the word
    // and the answer, because "the guest keeps asking the same question" is
    // only half a finding until the machine's answer is beside it.
    //
    // ⚠ SPRG0 is a PHYSICAL address: the dispatcher reads it with DR clear, so
    // this reads the bus directly rather than translating — one less thing to
    // get wrong, and it works when the page tables do not.
    {
        const u32 sprg0 = cpu.st.sprg[0];
        s += "-- nanokernel per-cpu block (physical; sc -1 reads [SPRG0-16]):\n";
        if (sprg0 >= 32 && u64(sprg0) + 4 <= m->bus->ramBytes()) {
            snprintf(b, sizeof b, "--   SPRG0=%08x ", sprg0);
            s += b;
            for (int off = -32; off <= -4; off += 4) {
                snprintf(b, sizeof b, "[%d]=%08x ", off,
                         m->bus->read32(static_cast<u32>(sprg0 + off)));
                s += b;
            }
            s += "\n";
            const u32 flags = m->bus->read32(sprg0 - 16u);
            const bool bit10 = (flags & 0x00200000u) != 0;
            snprintf(b, sizeof b,
                     "--   flags=[SPRG0-16]=%08x  bit10(0x00200000)=%s  =>  "
                     "`sc -1` answers %d %s\n",
                     flags, bit10 ? "SET" : "CLEAR", bit10 ? 0 : 1,
                     bit10 ? "(healthy: the caller's good path)"
                           : "<== NOT the healthy value (healthy = SET)");
            s += b;
        } else {
            snprintf(b, sizeof b,
                     "--   SPRG0=%08x is not a readable RAM address\n", sprg0);
            s += b;
        }
    }
    // 📇 …AND THE OTHER INPUTS TO THE SAME PREDICATE. The ROM routine that
    // calls the stub also consults ExpandMem (low-memory long at 0x2B6): it
    // reads emSize at +2, and only if that exceeds 0x31C does it test the field
    // at +0x31C. Four branches decide one answer, so print all four inputs at
    // once — chasing them one capture at a time is how a ten-minute
    // reproduction turns into a session.
    {
        s += "-- lowmem ExpandMem (the same predicate's other inputs):\n";
        u32 pa = 0;
        if (diagXlate(cpu, *m->bus, 0x2B6u, false, pa)) {
            const u32 em = m->bus->read32(pa);
            u32 empa = 0;
            if (em && diagXlate(cpu, *m->bus, em, false, empa)) {
                // emSize is a longword at +2 — deliberately unaligned, which is
                // why it is read a halfword at a time here.
                const u32 emSize = (u32(m->bus->read16(empa + 2)) << 16) |
                                   m->bus->read16(empa + 4);
                snprintf(b, sizeof b, "--   ExpandMem=%08x (pa %08x) emSize=%08x\n",
                         em, empa, emSize);
                s += b;
                u32 fpa = 0;
                if (emSize > 0x31Cu && diagXlate(cpu, *m->bus, em + 0x31Cu, false, fpa)) {
                    snprintf(b, sizeof b,
                             "--   [ExpandMem+0x31C]=%08x %s\n", m->bus->read32(fpa),
                             m->bus->read32(fpa) ? "<== NON-ZERO: predicate returns 6"
                                             : "(zero: predicate's good path)");
                    s += b;
                } else {
                    snprintf(b, sizeof b,
                             "--   emSize <= 0x31C, so +0x31C is not consulted\n");
                    s += b;
                }
            } else {
                // ⚠ Zero and unmapped are DIFFERENT facts and must not print
                // the same way: before Mac OS is up this pointer is simply not
                // set yet, which is normal, while a non-zero pointer that will
                // not translate is a broken one.
                snprintf(b, sizeof b, "--   ExpandMem=%08x %s\n", em,
                         em ? "(non-zero but NOT translatable)"
                            : "(not set up yet: normal before Mac OS boots)");
                s += b;
            }
        } else {
            s += "--   low memory 0x2B6 is not mapped in this context\n";
        }
    }
    // 🔬 THE HANDLER'S OWN INSTRUCTIONS. With IR clear the program counter IS
    // a physical address, so the code it is sitting in can be read straight
    // out of RAM — no translation to get wrong. Sixteen words around pc is
    // enough to see whether it is a loop and what it is testing.
    // ⚠ TRANSLATED WHEN IT HAS TO BE. The first version of this only printed
    // when IR was clear, on the reasoning that the pc is then physical and
    // there is nothing to get wrong — which meant the case that matters most,
    // a guest spinning in ORDINARY user-mode code, printed nothing at all.
    // pc, srr0 AND lr. The link register is the one that names the DRIVER: OF
    // and the OS reach device registers through a handful of shared accessor
    // primitives, so a pc inside one names the primitive and never the caller.
    // The bus keeps LR for device attribution for exactly this reason.
    for (int which = 0; which < 3; ++which) {
        const u32 ea = which == 0   ? cpu.st.pc
                       : which == 1 ? cpu.st.srr0
                                    : cpu.st.lr;
        if (which && (ea == 0 || ea == cpu.st.pc))
            continue;
        u32 base = ea;
        bool ok = true;
        const bool ir = (cpu.st.msr & 0x0020u) != 0;
        if (ir)
            ok = diagXlate(cpu, *m->bus, ea, true, base);
        if (!ok) {
            const char* nm =
                which == 0 ? "pc  " : which == 1 ? "srr0" : "lr  ";
            snprintf(b, sizeof b,
                     "-- code at %s %08x: no PTE and no BAT covers it\n", nm,
                     ea);
            s += b;
            // ⭐ ASK THE CPU'S OWN FETCH TRANSLATION TOO. This walk reads the
            // page table out of RAM; the machine fetches through a cached
            // translation guarded by a generation counter. If the pc will not
            // translate here but the CPU has a live cached mapping for that
            // very page, the two readers of one mapping disagree — and this
            // project has been here before: that disagreement IS the finding,
            // not a detail. If they agree that nothing maps it, then the guest
            // is genuinely about to take an ISI and the walk below says why.
            if (which == 0) {
                const bool live =
                    cpu.xlGen == cpu.mmuGen && cpu.xlPage == (ea >> 12) &&
                    cpu.xlMsr == (cpu.st.msr & (msr::IR | msr::PR | msr::LE)) &&
                    cpu.xlSr == cpu.st.sr[ea >> 28];
                if (live) {
                    const u32 pa = cpu.xlPa | (ea & 0xFFFu);
                    snprintf(b, sizeof b,
                             "--   !! but the CPU's OWN fetch translation maps "
                             "this page: pa=%08x (xlPage=%05x xlPa=%08x) - the "
                             "walk and the fetch path DISAGREE\n",
                             pa, cpu.xlPage, cpu.xlPa);
                    s += b;
                    for (u32 row = 0; row < 8; ++row) {
                        snprintf(b, sizeof b, "--   %08x:", ea - 16u + row * 16u);
                        s += b;
                        for (u32 w = 0; w < 4; ++w) {
                            snprintf(b, sizeof b, " %08x",
                                     m->bus->read32(pa - 16u + row * 16u + w * 4u));
                            s += b;
                        }
                        s += "\n";
                    }
                } else {
                    s += "--   the CPU has no live cached fetch translation for "
                         "this page either (they agree)\n";
                }
            }
            dumpMmu(s, cpu, *m->bus, ea, nm);
            continue;
        }
        // Walk back four instructions so the top of a short loop is visible.
        const u32 from = base - 16u;
        snprintf(b, sizeof b, "-- code at %s %08x (%s %08x):\n",
                 which == 0 ? "pc  " : which == 1 ? "srr0" : "lr  ", ea,
                 ir ? "pa" : "physical", from);
        s += b;
        // Twelve rows rather than six: a polling routine and the loop that
        // calls it do not fit in six, and the second half of the answer being
        // one row past the end costs another ten-minute boot to find out.
        for (u32 row = 0; row < 12; ++row) {
            snprintf(b, sizeof b, "--   %08x:", ea - 16u + row * 16u);
            s += b;
            for (u32 w = 0; w < 4; ++w) {
                snprintf(b, sizeof b, " %08x",
                         m->bus->read32(from + row * 16u + w * 4u));
                s += b;
            }
            s += (row == 1) ? "   <== the faulting/current row\n" : "\n";
        }
    }
    // 🔎 THE PAGE TABLE, FOR THE ADDRESS THAT FAULTED. A DSI with DSISR bit 1
    // says "no translation found", and there are two very different reasons
    // for that: the guest genuinely has not mapped the page (its own business,
    // and its handler's job to fix), or a PTE IS there and our search failed to
    // match it (ours). Printing our conclusion cannot tell those apart — only
    // the raw table can, so this dumps both PTEGs and lets the reader judge.
    //
    // ⚠ READ STRAIGHT FROM RAM, and that is a real caveat rather than a
    // detail: the walk in mmu.cpp reads through the data cache, because a page
    // table living in dirty cache lines is architectural. A PTE the guest has
    // just written and not yet flushed would be visible to the walk and NOT
    // here. So "no match below" means "no match in RAM", and a disagreement
    // between this and the fault is itself the finding.
    if (cpu.st.dsisr || cpu.st.dar)
        dumpMmu(s, cpu, *m->bus, cpu.st.dar, "dar");
    // 📋 THE REGISTERS. A guest polling a device holds the base address it is
    // polling in a register — this capture showed a loop calling a little-
    // endian accessor (lwbrx/stwbrx + eieio at DBDMA offsets 0x04/0x0C) with
    // the base in r3, and no way to see what that base WAS. Thirty-two words
    // is nothing next to another ten-minute run.
    for (u32 row = 0; row < 8; ++row) {
        snprintf(b, sizeof b, "--   r%-2u %08x  r%-2u %08x  r%-2u %08x  r%-2u %08x\n",
                 row * 4, cpu.st.gpr[row * 4], row * 4 + 1,
                 cpu.st.gpr[row * 4 + 1], row * 4 + 2, cpu.st.gpr[row * 4 + 2],
                 row * 4 + 3, cpu.st.gpr[row * 4 + 3]);
        s += (row == 0) ? std::string("-- gprs:\n") + b : std::string(b);
    }
    // 🔎 WHAT THE POINTER REGISTERS POINT AT, decoded as a DBDMA descriptor as
    // well as raw. A stalled driver polls a WORD, and knowing which register
    // holds the address is only half of it — this hang came down to a loop
    // reading offset +0x0C of something in r28 with lwbrx and testing 0x0400,
    // which is a descriptor's completion flag, at an address that belongs to
    // no channel this machine knows about. Only the memory says whose it is.
    // ⚠ Header printed unconditionally. It used to ride on r0's line, so when
    // r0 held zero — which is most of the time — the whole section silently
    // disappeared and read as "nothing to report" rather than "nothing
    // qualified".
    s += "-- memory at pointer registers (RAM-looking, word-aligned):\n";
    for (u32 gi = 0; gi < 32; ++gi) {
        const u32 ea = cpu.st.gpr[gi];
        // Anything that could be a pointer into RAM. Device space and small
        // integers are not worth the lines.
        if (ea < 0x1000u || ea >= 0x40000000u || (ea & 3u))
            continue;
        u32 pa = ea;
        if ((cpu.st.msr & 0x0010u) && !diagXlate(cpu, *m->bus, ea, false, pa))
            continue;
        const u32 w0 = m->bus->read32(pa), w1 = m->bus->read32(pa + 4);
        const u32 w2 = m->bus->read32(pa + 8), w3 = m->bus->read32(pa + 12);
        auto sw = [](u32 v) {
            return (v >> 24) | ((v >> 8) & 0xFF00u) | ((v << 8) & 0xFF0000u) |
                   (v << 24);
        };
        snprintf(b, sizeof b,
                 "--   r%-2u -> %08x: %08x %08x %08x %08x | as descriptor: "
                 "cmd=%u req=%u addr=%08x xferStatus=%04x res=%u\n",
                 gi, ea, w0, w1, w2, w3, (sw(w0) >> 28) & 7u, sw(w0) & 0xFFFFu,
                 sw(w1), sw(w3) >> 16, sw(w3) & 0xFFFFu);
        s += b;
    }
    // ⭐ WHAT THE MACHINE ASKED FOR AND NOBODY ANSWERED. Every access that no
    // device claims master-aborts (reads all-ones, writes dropped) and lands
    // here, keyed by address. A driver polling a register block this emulator
    // does not model reads 0xFFFFFFFF forever and waits forever — which is
    // exactly the shape of a boot that stops during extension loading.
    //
    // Sorted by traffic, because the one being polled in a loop is the one
    // with the enormous count. ⚠ firstPc names Open Firmware's shared access
    // primitive rather than the driver, which is why the bus also tracks LR.
    {
        const auto& lg = m->bus->accessLog();
        std::vector<std::pair<u64, const std::pair<const u32, SawtoothBus::Acc>*>>
            hot;
        for (const auto& kv : lg)
            hot.push_back({kv.second.reads + kv.second.writes, &kv});
        std::sort(hot.begin(), hot.end(),
                  [](const auto& a, const auto& c) { return a.first > c.first; });
        snprintf(b, sizeof b,
                 "-- unclaimed bus accesses: %zu distinct addresses, busiest "
                 "first:\n",
                 lg.size());
        s += b;
        for (size_t k = 0; k < hot.size() && k < 12; ++k) {
            const auto& kv = *hot[k].second;
            snprintf(b, sizeof b,
                     "--   %08x  reads=%llu writes=%llu lastWr=%08x "
                     "firstPc=%08x @%llu\n",
                     kv.first,
                     static_cast<unsigned long long>(kv.second.reads),
                     static_cast<unsigned long long>(kv.second.writes),
                     kv.second.lastWr, kv.second.firstPc,
                     static_cast<unsigned long long>(kv.second.firstAt));
            s += b;
        }
    }
    s += m->bus->pic().describe();
    // ⚠ PARKED is the one to read first. A channel parked mid-descriptor is
    // waiting for a device to take or supply more bytes, and it is cleared by
    // a periodic wake — so a channel that is parked and STAYS parked is a
    // transfer that will never finish, which from outside is a frozen machine.
    // RUN set with the interrupt line high and nobody lowering it is the other
    // shape, and it is how the session-29 wedge presented.
    // ⭐ EVERY CHANNEL, WITH THE BITS SPELLED OUT. A driver stalled on a DBDMA
    // channel is waiting for one specific bit, and "run/parked/irq" answers
    // three questions that are usually not the one being asked. The CHRP names
    // are here too, because +0x8700 meaning "SCC receive B" is the step that
    // turned an address into a diagnosis.
    {
        static const char* kChanName[8] = {
            "scsi0", "floppy", "ether-tx", "ether-rx",
            "scc-a-tx", "scc-a-rx", "scc-b-tx", "scc-b-rx"};
        s += "-- dbdma channels (status bits: RUN 8000 PAUSE 4000 FLUSH 2000 "
             "WAKE 1000 DEAD 800 ACTIVE 400 BRANCH 100):\n";
        auto one = [&](const char* nm, u32 off, DbdmaChannel& ch) {
            const u32 st = ch.status();
            snprintf(b, sizeof b,
                     "--   +%04x %-9s status=%08x cmdPtr=%08x irq=%d %s%s%s%s%s%s\n",
                     off, nm, st, ch.cmdPtr(), ch.irqLine() ? 1 : 0,
                     (st & 0x8000u) ? "RUN " : "", (st & 0x4000u) ? "PAUSE " : "",
                     (st & 0x2000u) ? "FLUSH " : "", (st & 0x1000u) ? "WAKE " : "",
                     (st & 0x0800u) ? "DEAD " : "",
                     (st & 0x0400u) ? "ACTIVE" : "");
            s += b;
        };
        for (u32 i = 0; i < 8; ++i)
            one(kChanName[i], 0x8000u + i * 0x100u, m->bus->genDma(i));
        one("audio-out", 0x8800, m->bus->sndOut());
        one("audio-in", 0x8900, m->bus->sndIn());
        one("ata-hd", 0x8A00, m->bus->hdDma());
        one("ata-cd", 0x8B00, m->bus->ataDma());

        // ⭐ WHAT THE CHANNEL WAS ASKED TO DO, and what it has been doing.
        // A status word says a channel is stuck; only the descriptor list says
        // what it is stuck ON, and only the event log says how it got there.
        // Both are dumped for any channel that has been programmed at all, so
        // whichever way the status reads, the answer is in the same capture.
        //
        // ⚠ Descriptors are LITTLE-ENDIAN in memory — that is why the guest
        // reaches these registers with lwbrx/stwbrx — so every word is swapped
        // on the way out here.
        auto swap32 = [](u32 v) {
            return (v >> 24) | ((v >> 8) & 0xFF00u) | ((v << 8) & 0xFF0000u) |
                   (v << 24);
        };
        static const char* kCmd[8] = {"OUTPUT_MORE", "OUTPUT_LAST",
                                      "INPUT_MORE",  "INPUT_LAST",
                                      "STORE_QUAD",  "LOAD_QUAD",
                                      "NOP",         "STOP"};
        static const char* kEvKind[8] = {"ctl",  "desc", "data",     "stop",
                                         "dead", "sq",   "ptr-live", "branch"};
        auto detail = [&](const char* nm, u32 off, DbdmaChannel& ch) {
            if (!ch.cmdPtr() && ch.log.empty())
                return; // never programmed: nothing to say
            // ⚠ START BEFORE cmdPtr, not at it. cmdPtr is where the engine has
            // got TO, so a dump that begins there shows the descriptors not
            // yet run and, past the end of a short list, whatever RAM follows
            // it — which reads as garbage and hides the ones that matter. Two
            // back puts the recently executed entries in view.
            const u32 first =
                ch.cmdPtr() > 32u ? ch.cmdPtr() - 32u : ch.cmdPtr();
            snprintf(b, sizeof b,
                     "--   +%04x %s: descriptors from %08x (cmdPtr=%08x)\n",
                     off, nm, first, ch.cmdPtr());
            s += b;
            for (u32 d = 0; d < 6 && ch.cmdPtr(); ++d) {
                const u32 a = first + d * 16u;
                const u32 w0 = swap32(m->bus->read32(a));
                const u32 w1 = swap32(m->bus->read32(a + 4));
                const u32 w2 = swap32(m->bus->read32(a + 8));
                const u32 w3 = swap32(m->bus->read32(a + 12));
                snprintf(b, sizeof b,
                         "--     %08x %-11s req=%-5u addr=%08x dep=%08x "
                         "xferStatus=%04x res=%u%s\n",
                         a, kCmd[(w0 >> 28) & 7u], w0 & 0xFFFFu, w1, w2,
                         w3 >> 16, w3 & 0xFFFFu,
                         a == ch.cmdPtr() ? "   <== cmdPtr" : "");
                s += b;
            }
            const size_t n = ch.log.size() > 16 ? 16 : ch.log.size();
            snprintf(b, sizeof b, "--     last %zu of %zu channel events:\n", n,
                     ch.log.size());
            s += b;
            for (size_t k = ch.log.size() - n; k < ch.log.size(); ++k) {
                const DbdmaChannel::Ev& e = ch.log[k];
                snprintf(b, sizeof b, "--       %-8s a=%08x b=%08x @%llu\n",
                         kEvKind[e.kind & 7u], e.a, e.b,
                         static_cast<unsigned long long>(e.at));
                s += b;
            }
        };
        for (u32 i = 0; i < 8; ++i)
            detail(kChanName[i], 0x8000u + i * 0x100u, m->bus->genDma(i));
        detail("audio-out", 0x8800, m->bus->sndOut());
        detail("ata-hd", 0x8A00, m->bus->hdDma());
        detail("ata-cd", 0x8B00, m->bus->ataDma());
    }
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
