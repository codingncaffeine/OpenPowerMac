#pragma once
// A sampling profiler for the interpreter, and the guest-pc census that goes
// with it.
//
// WHY THIS EXISTS. The speed work in DESKTOP_ROADMAP §9 opens with "profile
// first — never guess", and there was no profiler in the tree. The candidate
// answers are not distinguishable by reading: an instruction costs a fetch, a
// translation, a decode, a handler, one or two cache lookups and a round of
// device polling, and every one of those is plausible as the dominant term.
//
// WHY SAMPLING. A counter says how OFTEN something runs; the question is where
// the HOST TIME goes. Timing each region with rdtsc costs more than the regions
// being timed and inflates whichever phase transitions most, which is exactly
// the wrong bias. A sampler pays per SAMPLE, not per event.
//
// WHY IN-TREE and not an external profiler. The shipping build is MSVC, the
// portability build is MinGW, the run under test is a 16-minute boot, and the
// answer has to be reproducible, diffable and quotable in a handoff. An
// in-tree instrument is also the only kind that can name a GUEST pc.
//
// COST IN THE SHIPPING BUILD: none. The markers expand to nothing unless
// OPM_PROFILE is defined, and the profiling executable (g4prof) is a second
// target built from the same sources. A permanently resident marker would be a
// real tax — eight stores per instruction against a couple of hundred cycles —
// and taxing the machine to measure the machine is a bad trade.

#include "types.hpp"
#include <atomic>
#include <cstddef>

namespace opm {
namespace prof {

// The regions time is attributed to. Nesting is handled by Mark, which
// restores the enclosing region, so Exec means "the handler MINUS the memory
// it touched" and Fetch means "instruction fetch MINUS the translation".
enum class Ph : unsigned char {
    Other = 0, // unmarked: loop overhead, reporting, anything uncategorised
    Fetch,     // fetch32 after translation: L1/L2 peek, bus read
    Xlate,     // Cpu::translate — BATs, TLBs, hardware page-table search
    Decode,    // decode() + facility/privilege guards + handlerFor()
    Exec,      // the instruction handler itself
    Read,      // Cpu::memRead  — L1 lookup/fill, L2, bus
    Write,     // Cpu::memWrite — L1 lookup/fill/castout, L2, bus
    Tick,      // Cpu::tick + performance monitor + trace check
    DevTick,   // the machine's per-instruction device advance
    Irq,       // interrupt-line recomputation and delivery to the CPU
    Instr,     // the harness's own per-step instrumentation
    N
};

const char* name(Ph p);

// The marker itself. A plain relaxed store to one hot byte; the sampler thread
// reads it. Relaxed is right: this is a statistical instrument and a fence per
// instruction would cost more than the thing being measured.
extern std::atomic<unsigned char> gPhase;

inline void set(Ph p)
{
    gPhase.store(static_cast<unsigned char>(p), std::memory_order_relaxed);
}

// Scoped marker for a region that can be entered from more than one enclosing
// region (translate is reached from both fetch and data access; memRead is
// reached from a handler and from a page-table walk).
struct Mark {
    unsigned char prev;
    explicit Mark(Ph p) : prev(gPhase.load(std::memory_order_relaxed))
    {
        gPhase.store(static_cast<unsigned char>(p), std::memory_order_relaxed);
    }
    ~Mark() { gPhase.store(prev, std::memory_order_relaxed); }
    Mark(const Mark&) = delete;
    Mark& operator=(const Mark&) = delete;
};

struct Result {
    u64 samples = 0;
    u64 phase[static_cast<size_t>(Ph::N)] = {};
    double seconds = 0;
    u64 missed = 0; // sampler could not keep its period: periods skipped
};

// pcRef may be null; when given, every sample also records the guest pc, which
// is what turns "the interpreter is busy" into "the GUEST is spinning here".
bool start(unsigned hz, const u32* pcRef);
void stop();
bool running();
const Result& result();

// The n hottest sampled guest pcs, most first. Returns how many were written.
size_t topPcs(u32* pc, u64* hits, size_t cap);

} // namespace prof
} // namespace opm

#if defined(OPM_PROFILE) && OPM_PROFILE
// A plain counter that exists only in the profiling build. Hit rates are the
// other half of the answer: a sampler says where the time went, a counter says
// whether a cache is actually catching anything.
#define OPM_COUNT(f) (++(f))
#define OPM_PH(p) ::opm::prof::set(::opm::prof::Ph::p)
#define OPM_MARK_2(p, n) ::opm::prof::Mark opmProfMark##n(::opm::prof::Ph::p)
#define OPM_MARK_1(p, n) OPM_MARK_2(p, n)
#define OPM_MARK(p) OPM_MARK_1(p, __LINE__)
#define OPM_PROFILING 1
#else
#define OPM_COUNT(f) ((void)0)
#define OPM_PH(p) ((void)0)
#define OPM_MARK(p) ((void)0)
#define OPM_PROFILING 0
#endif
