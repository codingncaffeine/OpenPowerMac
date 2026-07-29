#pragma once
#include "types.hpp"
#include "bus.hpp"
#include "insn.hpp"
#include <map>
#include <string>
#include <vector>

namespace opm {

// Full architected state of the MPC7400. Reset values per UM Table 2-18
// ("Settings Caused by Hard Reset"); fields the table calls undefined are
// zero-initialized here for determinism.
struct CpuState {
    u32 gpr[32]{};
    u64 fpr[32]{};      // raw IEEE-754 double bit patterns
    V128 vr[32]{};

    u32 pc = 0xFFF00100u;
    u32 cr = 0, xer = 0, lr = 0, ctr = 0;
    u32 msr = 0x00000040u;      // only IP set at HRESET
    u32 fpscr = 0;
    u32 vscr = 0, vrsave = 0;

    u32 srr0 = 0, srr1 = 0;
    u32 sprg[4]{};
    u32 dar = 0, dsisr = 0, sdr1 = 0, ear = 0, pir = 0;
    u32 sr[16]{};
    u32 ibatu[4]{}, ibatl[4]{};
    u32 dbatu[4]{}, dbatl[4]{};

    u32 hid0 = 0, hid1 = 0;
    u32 msscr0 = 0x00400000u, msscr1 = 0;
    u32 l2cr = 0, ictc = 0;
    u32 thrm[3]{};
    u32 iabr = 0, dabr = 0, bamr = 0;
    u32 mmcr0 = 0, mmcr1 = 0, pmc[4]{}, siar = 0, sdar = 0;

    u64 tb = 0;
    u32 dec = 0xFFFFFFFFu;
    u32 pvr = 0x000C0209u;      // MPC7400 silicon rev 2.9 (errata Table 2)

    bool resvValid = false;
    u32 resvAddr = 0;           // 32-byte-granule base
};

// MSR bit masks (32-bit, PPC numbering bit 0 = MSB). Verified: VEC = bit 6
// (UM Table 2-1); layout per MPCPRG; PM position pinned pending P6 sweep.
namespace msr {
inline constexpr u32 VEC = 0x02000000u;
inline constexpr u32 POW = 0x00040000u;
inline constexpr u32 ILE = 0x00010000u;
inline constexpr u32 EE  = 0x00008000u;
inline constexpr u32 PR  = 0x00004000u;
inline constexpr u32 FP  = 0x00002000u;
inline constexpr u32 ME  = 0x00001000u;
inline constexpr u32 FE0 = 0x00000800u;
inline constexpr u32 SE  = 0x00000400u;
inline constexpr u32 BE  = 0x00000200u;
inline constexpr u32 FE1 = 0x00000100u;
inline constexpr u32 IP  = 0x00000040u;
inline constexpr u32 IR  = 0x00000020u;
inline constexpr u32 DR  = 0x00000010u;
inline constexpr u32 PM  = 0x00000004u;
inline constexpr u32 RI  = 0x00000002u;
inline constexpr u32 LE  = 0x00000001u;
// Bits the 7400 implements (mtmsr/rfi mask).
inline constexpr u32 VALID = VEC | POW | ILE | EE | PR | FP | ME | FE0 | SE |
                             BE | FE1 | IP | IR | DR | PM | RI | LE;
} // namespace msr

// Exception vectors (offsets; base 0xFFF00000 when MSR[IP], else 0).
enum class Exc : u32 {
    SystemReset = 0x00100,
    MachineCheck = 0x00200,
    Dsi = 0x00300,
    Isi = 0x00400,
    External = 0x00500,
    Alignment = 0x00600,
    Program = 0x00700,
    FpUnavailable = 0x00800,
    Decrementer = 0x00900,
    SystemCall = 0x00C00,
    Trace = 0x00D00,
    PerfMon = 0x00F00,
    VecUnavailable = 0x00F20,
    Iabr = 0x01300,
    Smi = 0x01400,
    VecAssist = 0x01600,
    Thermal = 0x01700,
};

// Program-exception SRR1 cause bits.
inline constexpr u32 kSrr1ProgFpEnabled = 0x00100000u;
inline constexpr u32 kSrr1ProgIllegal = 0x00080000u;
inline constexpr u32 kSrr1ProgPrivileged = 0x00040000u;
inline constexpr u32 kSrr1ProgTrap = 0x00020000u;

struct Cpu {
    CpuState st;
    Bus* bus = nullptr;

    // Census of decode gaps hit at runtime: mnemonic (or raw word) -> count.
    std::map<std::string, u64> unimplemented;
    std::map<u32, u64> unknownWords;

    // Async lines / pending state.
    bool extIrqLine = false;   // level-sensitive external interrupt
    bool smiPending = false;
    bool decPending = false;
    bool pmPending = false;    // performance monitor (PMC MSB with PMXE)
    bool raisedThisStep = false;
    bool napping = false;      // MSR[POW] + HID0 nap/doze/sleep: TB ticks,
                               // no instructions until an enabled interrupt
    u32 curInsn = 0;           // instruction being executed (LE align image)
    u32 cycleAccum = 0;
    u32 cyclesPerTbTick = 4;   // TB = bus clock / 4; provisional 1 cycle/insn

    // Timebase/decrementer accounting. Whether the guest's 60 Hz tick is
    // driven off the decrementer is a question the whole boot hangs on —
    // Ticks ($016A) advanced once in 1.45 billion instructions — and it is
    // answerable only by counting what the guest actually does with DEC:
    // how often it reloads it, with what period, and how many 0x900s it
    // takes. Guessing from the interrupt total alone cannot separate "never
    // programmed" from "programmed far too long".
    u64 decWrites = 0;         // mtspr DEC by the guest
    u64 decIrqs = 0;           // 0x900 exceptions actually delivered
    u32 decLastWrite = 0;      // last value written
    u64 decLastWriteTb = 0;    // TB at that write
    u64 decMinPeriod = ~0ull;  // smallest reload seen (TB ticks)

    void tick(u32 cycles)
    {
        cycleAccum += cycles;
        while (cycleAccum >= cyclesPerTbTick) {
            cycleAccum -= cyclesPerTbTick;
            st.tb += 1;
            const u32 old = st.dec;
            st.dec -= 1;
            if (!(old & 0x80000000u) && (st.dec & 0x80000000u))
                decPending = true;
        }
    }

    // Set when execution cannot continue (pre-P2 stand-in for the exception
    // model: traps, sc, illegal ops halt with a reason instead of vectoring).
    bool halted = false;
    std::string haltReason;

    Cpu();
    void attach(Bus& b) { bus = &b; }
    void reset()
    {
        st = CpuState{};
        halted = false;
        haltReason.clear();
        extIrqLine = smiPending = decPending = pmPending = false;
        napping = false;
        cycleAccum = 0;
        tlbFlushAll();
    }
    void halt(std::string reason)
    {
        halted = true;
        haltReason = std::move(reason);
    }

    // Execute up to n instructions; returns the number actually executed.
    u64 run(u64 n);
    void step();

    // Vector to an exception per the UM ch.4 model: SRR0/SRR1 composition,
    // MSR transition (clears VEC/POW/EE/PR/FP/FE0/SE/BE/FE1/PM/IR/DR/RI,
    // LE<-ILE, keeps ME/IP/ILE), PC to base|vector.
    // srr0 = address to save; extra = exception-specific SRR1 bits (1-4,10-15).
    void raiseExc(Exc v, u32 srr0, u32 extra);

    void setExternalIrq(bool level) { extIrqLine = level; }
    void raiseSmi() { smiPending = true; }

    bool userMode() const { return (st.msr & msr::PR) != 0; }

    // TLBs (UM ch.5: separate 128-entry, 2-way set-associative I and D
    // TLBs, 64 sets indexed by EA[14-19], LRU replacement per set). This is
    // REQUIRED micro-architectural state, not an optimization: the Old
    // World ROM's regime teardown wipes the hash table while relying on
    // warm TLB entries to keep its own code, I/O, and pager mappings alive
    // until it issues tlbie — walk-per-access dies exactly where real
    // silicon survives (pinned empirically, macrun boot trace 2026-07-26).
    // Lives outside CpuState: visible only through the staleness contract;
    // SST state deliberately excludes it.
    struct TlbEntry {
        bool v = false, c = false;
        u32 vsid = 0, pi = 0, rpn = 0, wimg = 0, pp = 0;
    };
    TlbEntry itlb[64][2], dtlb[64][2];
    u8 itlbLru[64] = {}, dtlbLru[64] = {};
    bool mmuProbe = false; // instrumentation: bypass TLB and R/C writeback
    // Harness convention (ppcrun flat rig only — never the machine): treat
    // real-mode accesses at/above this base as cache-inhibited so MMIO
    // (putc/exit) keeps device semantics with the caches enabled. The
    // architecture makes real mode cacheable (WIMG 0b0011); real software
    // must map I/O with I=1, and the machine path models exactly that.
    u32 realModeInhibitBase = 0;
    void tlbInvalidateClass(u32 ea); // tlbie: both ways, both TLBs
    void tlbFlushAll();

    // L1 data cache (UM ch.3: 32 KB, 8-way, 128 sets, 32-byte blocks,
    // write-back with write-allocate; LRU stands in for the PLRU tree —
    // architecturally invisible). Load-bearing the same way the TLBs are:
    // the Gossamer ROM runs its early world out of dcbz-conjured cache
    // lines (cache-as-RAM) before the memory controller is enabled, so
    // those lines must live and hit without any bus fill.
    struct DLine {
        bool v = false, d = false;
        u32 tag = 0; // pa >> 12
        u32 age = 0;
        u8 b[32] = {};
    };
    DLine l1d[128][8];
    u32 l1dClock = 0;
    bool dceOn() const { return (st.hid0 & 0x00004000u) != 0; }
    u64 memRead(u32 pa, u32 len, u32 wimg);
    void memWrite(u32 pa, u32 len, u64 v, u32 wimg);

    // Physical data watchpoint. Bus::watchPa only sees traffic that reaches
    // the bus, so an ordinary cached store to a low-memory global — Ticks,
    // a DCE field, a queue header — is invisible to it: the write lands in
    // an L1 line and the watch never fires, which reads exactly like "the
    // guest never wrote it". This sits on the store path itself, so it sees
    // the write whether or not the line is ever cast out.
    u32 wpPa = 0, wpEnd = 0; // inclusive byte range; wpEnd 0 disables
    struct WpHit {
        u32 pc, pa, val, len;
        u64 tb;
    };
    std::vector<WpHit> wpLog;
    u32 wpMax = 64;
    bool l1dPeek32(u32 pa, u32& w); // fetch path coherence peek
    void dcbzLine(u32 pa);          // allocate + zero, no fill
    void dcbClean(u32 pa, bool invalidate); // dcbst / dcbf
    void dcbKill(u32 pa);                   // dcbi
    void l1dFlushAll(bool writeback);
    // Snoop response for a bus master (see SnoopSink in bus.hpp). Pushes
    // every modified line covering [pa, pa+len) all the way to MEMORY —
    // not into the L2, which a DMA engine cannot see either — and
    // invalidates the processor's copy as well when the master is writing.
    void snoopPush(u32 pa, u32 len, bool invalidate);

    // Backside L2 (UM ch.3.7: L2CR-governed, 256K-2MB, 2-way here). The
    // boot ROM runs its pre-DRAM world in it via L2CR[L2TS]: dcbf/dcbst
    // pushes land in the L2 marked valid instead of dying on the bus (UM
    // 2-17/L2TS verbatim) — hash tables and sizing scratch live there
    // until real memory exists. Castouts allocate; fills probe L2 first.
    struct L2Line {
        bool v = false, d = false;
        u32 tag = 0; // pa >> 5 (line number)
        u32 age = 0;
        u8 b[32] = {};
    };
    std::vector<L2Line> l2;
    u32 l2Sets = 0, l2Clock = 0;
    bool l2On() const { return (st.l2cr & 0x80000000u) != 0; }
    void l2Resize();                 // per L2CR[L2SIZ]
    L2Line* l2Find(u32 pa);
    void l2Install(u32 pa, const u8* bytes, bool dirty);
    bool l2ReadLine(u32 pa, u8* out);
    void l2Invalidate(u32 pa);
    void l2WipeAll();                // L2I global invalidate
    bool l2Peek32(u32 pa, u32& w);
    void l2FlushAll(bool writeback); // instrumentation/harness coherence

    // MMU (mmu.cpp). translate() raises ISI/DSI itself on failure. wimg, if
    // requested, receives the access's WIMG nibble (W=8,I=4,M=2,G=1; real
    // mode reads as 0b0011 per PEM 7.2).
    bool translate(u32 ea, bool write, bool fetch, u32& pa, u32* wimg = nullptr);
    bool readV(u32 ea, u32 len, u64& out);
    bool writeV(u32 ea, u32 len, u64 v);
    bool fetch32(u32 ea, u32& insn);
    bool leAlignCheck(u32 ea, u32 len); // LE mode: misaligned -> alignment exc

    bool readV8(u32 ea, u32& v)  { u64 t; if (!readV(ea, 1, t)) return false; v = static_cast<u32>(t); return true; }
    bool readV16(u32 ea, u32& v) { u64 t; if (!readV(ea, 2, t)) return false; v = static_cast<u32>(t); return true; }
    bool readV32(u32 ea, u32& v) { u64 t; if (!readV(ea, 4, t)) return false; v = static_cast<u32>(t); return true; }
    bool writeV8(u32 ea, u32 v)  { return writeV(ea, 1, v & 0xFFu); }
    bool writeV16(u32 ea, u32 v) { return writeV(ea, 2, v & 0xFFFFu); }
    bool writeV32(u32 ea, u32 v) { return writeV(ea, 4, v); }

    // --- state helpers shared by executors ---
    void setCr0(u32 val)
    {
        const i32 s = static_cast<i32>(val);
        u32 f = (s < 0) ? 8u : (s > 0) ? 4u : 2u;
        f |= (st.xer >> 31) & 1u; // SO
        st.cr = (st.cr & 0x0FFFFFFFu) | (f << 28);
    }
    void setCrField(u32 field, u32 nibble)
    {
        const u32 sh = (7u - field) * 4u;
        st.cr = (st.cr & ~(0xFu << sh)) | ((nibble & 0xFu) << sh);
    }
    u32 crField(u32 field) const { return (st.cr >> ((7u - field) * 4u)) & 0xFu; }
    void setCa(bool ca)
    {
        st.xer = ca ? (st.xer | 0x20000000u) : (st.xer & ~0x20000000u);
    }
    bool ca() const { return (st.xer & 0x20000000u) != 0; }
    void setOv(bool ov)
    {
        if (ov)
            st.xer |= 0xC0000000u; // OV + sticky SO
        else
            st.xer &= ~0x40000000u;
    }
};

// Hands a Bus its snoop responder. Kept as a separate object rather than
// making Cpu itself a SnoopSink: a vtable would change sizeof(Cpu), and the
// snapshot format's layout digest is computed from exactly that.
struct CpuSnoop final : SnoopSink {
    Cpu* cpu = nullptr;
    void snoopRead(u32 pa, u32 len) override;
    void snoopWrite(u32 pa, u32 len) override;
};

// Binds every implemented handler into the dispatch used by Cpu::step.
// Idempotent; called from the Cpu constructor.
void bindHandlers();
void bindFpuHandlers(); // fpu.cpp's registrations, called by bindHandlers()
void bindVecHandlers(); // altivec.cpp's registrations, called by bindHandlers()

// Handler slot for an ISA row (parallel to kIsa; see decode()).
Handler handlerFor(const InsnDesc* d);
void setHandler(const char* mnem, Handler fn);

} // namespace opm
