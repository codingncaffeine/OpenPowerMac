#pragma once
// The JIT: a native x86-64 transcription of Cpu::runSteps' line executor.
// Design and correctness argument: _plans/JIT_PLAN.md. The short form:
//
//   - One compiled block per 32-byte FetchLine, eight entry points (one per
//     word), segments laid out adjacently so fallthrough is literal adjacency.
//   - Hot execFast-eligible instructions are inlined; everything else calls
//     execRow — the SAME dispatch tail the interpreter uses — and then runs
//     the line executor's own exit tests (batchBreak/halted, pc fell through,
//     line still resident). Nothing is re-derived, so nothing can disagree.
//   - Memory goes through shims around readV*/writeV*: every model side
//     effect (L1/L2 state, castouts, snoops, device clocks, batch breaks)
//     happens exactly as interpreted execution would make it happen, which is
//     what keeps whole-machine fingerprints comparable between --jit and
//     --no-jit.
//   - Validity rides the machine's one invalidation quantum: the FetchLine.
//     A block is entered only when the fetch line it was compiled from is
//     resident with the same base, for the same VA line, and the compiled
//     words are re-checked against the line on every refill (jitNoteRefill).
//     Stores, dcbz, castouts, snoops, icbi and flushes already drop the
//     line; the block re-checks `fl.base` after every instruction that can
//     write memory, so mid-line self-modifying code exits exactly where the
//     interpreter would have. That contract is the SMC story the ROM's own
//     68K emulator needs.
//
// ⚠ Pure cache. Never snapshotted, dropped on reset and (via the refill
// hook) after snapshot restore; CpuState is untouched and kSnapVersion does
// not move.

#include "types.hpp"

#include <memory>

namespace opm {

struct Cpu;

struct JitLine {
    u32 base = 1;    // PA line base the block was compiled from, or 1 (like
                     // FetchLine::base: no 32-byte base can be 1)
    u32 va = 0;      // VA of word 0 — branch targets and next-pc values are
                     // baked immediates, so a block is specific to one
                     // PA<->VA binding and re-keys when the mapping moves
    u32 srcW[8]{};   // the eight words compiled, so a refill with identical
                     // content KEEPS the block (instrument-era flushes would
                     // otherwise recompile the world)
    u32 off[8]{};    // arena offset of each word's segment
    // Chain backrefs: arena offsets of the rel32 words in OTHER blocks that
    // jump into this one (JIT_PLAN §7 Stage B). Severed — rewritten so the
    // site re-resolves — the moment this JitLine stops describing its line
    // (refill with different content, way eviction, dropAll), which is the
    // whole of what makes a chained entry equivalent to the dispatcher's
    // ways probe. Fixed capacity: a site that finds the list full parks
    // itself on the dispatcher exit permanently (counted, chainGiveups).
    static constexpr u32 kBrefs = 8;
    u32 bref[kBrefs]{};
    u8 nBref = 0;
};

class JitCache {
  public:
    JitCache();
    ~JitCache();
    JitCache(const JitCache&) = delete;
    JitCache& operator=(const JitCache&) = delete;

    static constexpr u32 kLines = 4096; // == Cpu::kFetchLines, asserted there
    // ⚠ WAYS, BECAUSE THE FETCH SLOTS CONFLICT AND A CONFLICT IS NOT AN
    // INVALIDATION. In-game the code working set collides constantly in the
    // 4096 direct-mapped fetch slots, and a one-way JIT cache that dies with
    // its slot recompiled 293k times over a 200M-instruction window — nearly
    // all of them for lines whose BYTES never changed. Four ways keyed by
    // (base, va) let the conflicting lines' blocks coexist: an eviction
    // leaves the block; only a refill whose CONTENT differs kills it (and
    // that comparison is also what catches code modified while its line was
    // not resident — the refill always precedes the next execution).
    static constexpr u32 kWays = 4;
    // 32 MB of code, bump-allocated; full -> drop everything and start over.
    // At the ~300 B/segment this emitter produces, a wholesale reset is a
    // rare event and a recompile is microseconds, so simplicity wins over an
    // eviction policy.
    static constexpr size_t kArenaBytes = 32u << 20;

    bool ready() const { return arena_ != nullptr && live_; }

    JitLine line[kLines * kWays]; // slot-major: ways of slot s at [s*kWays..]
    u8 rr[kLines] = {};           // round-robin victim way per slot

    // Counters, always on: they are how the report says what the JIT did.
    u64 compiles = 0, enters = 0, insns = 0, resets = 0, refillKeeps = 0,
        refillDrops = 0, bails = 0;
    // Chain lifecycle (Stage B). Hops themselves are counted by emitted code
    // into Cpu::jitChainHops (r13-addressable); everything here is C++-side.
    u64 chainLinks = 0, chainSevers = 0, chainResolves = 0, chainMisses = 0,
        chainGiveups = 0;
    // The mix census. What the blocks actually executed is the aim point
    // for the next lowering round: memOps counts shim round trips, fallbacks
    // counts execRow calls from inside blocks, and fbByRow names the rows —
    // it is how the s40 session found that 43% of the in-game window was
    // floating point and which ops they were. tscProbe/tscNative are wired
    // for the rdtsc split when a session needs it (writers currently absent;
    // the report prints them only when populated).
    u64 tscProbe = 0, tscNative = 0;
    u64 fallbacks = 0;
    u64 memOps = 0;
    u64 fbByRow[1024] = {};
    // 🔬 --jit-tsc: THE HOST-CYCLE SPLIT. Counting calls says which rows are
    // frequent; it cannot say where the time is, and this JIT's remaining
    // cost is concentrated in a few call-shaped things (the memory shims, the
    // execRow fallbacks) whose per-call price is the whole question for what
    // to lower next. Guarded by a bool so the flag is always available
    // without a second build: off, it is one perfectly-predicted branch per
    // shim; on, it costs an rdtsc pair per call and inflates the run, so the
    // numbers it produces are SHARES and per-call costs, never MIPS.
    bool tscOn = false;
    u64 tscMem = 0, tscFb = 0, tscByRow[1024] = {};
    // FP loads/stores are memory ops too, and they were invisible: `memOps`
    // counts only the integer shims, so the s40 "mem ops 15.9%" line omitted
    // the lfs/stfs/lfd/stfd traffic entirely — and the census had already
    // named lfs the single hottest row in the game. Counted apart so the
    // historical number stays comparable to the sessions that quoted it.
    u64 fpMemOps = 0;

    // r13-relative displacements into Cpu, measured off the live object at
    // bind() rather than via offsetof: Cpu holds std::map members, so it is
    // not standard-layout, and measuring the real object is both portable
    // and self-evidently right.
    struct Offs {
        i32 gpr0 = 0, pc = 0, cr = 0, xer = 0, lr = 0, ctr = 0;
        i32 pend = 0, curInsn = 0, halted = 0, batchBreak = 0;
        i32 msr = 0, fpr0 = 0;
        i32 fetchLine0 = 0; // fetchLine[0].base; slot k at +k*sizeof(FetchLine)
        i32 flStride = 0;
        // Chain-gate fields (Stage B): the dispatcher's own `slow` inputs,
        // the fetch-translation MSR word, the budget bound and the hop
        // counter. pend4 covers the four adjacent pending bools with one
        // dword compare when bind() confirms adjacency (pend4Ok).
        i32 lineExecOff = 0, napping = 0, mmcr0 = 0, iabr = 0, xlMsr = 0;
        i32 raisedThis = 0; // execRow clears it before every handler
        i32 pend4 = 0, extIrq = 0, smi = 0, decP = 0, pmP = 0;
        i32 until = 0, chainHops = 0;
        bool pend4Ok = false;
    } offs;

    using EnterFn = void (*)(const void* entry, Cpu*, u64* stamp, u64 cyc);
    EnterFn enterFn = nullptr;

    // Wire the cache to one Cpu: measure offsets, emit trampoline/epilogue.
    void bind(Cpu& c);
    void dropAll();
    // Unlink every chain site that jumps into this block: each recorded
    // rel32 goes back to 0, which re-targets the site at its own resolver
    // thunk. Must run before the JitLine stops describing its (base, va) —
    // see the field comment on JitLine::bref.
    void sever(JitLine& jl);
    bool compileLine(Cpu& c, u32 slot, u32 paBase, const u32* w,
                     const u16* rows, u32 vaBase);

    u8* arena_ = nullptr;
    size_t cap_ = 0, at_ = 0;
    size_t firstCode_ = 0; // end of trampoline+epilogue; resets rewind here
    u32 epilogue_ = 0;     // arena offset every exit jumps to
    bool live_ = false;
};

// runSteps' door. Compiles the slot's line on demand and executes native code
// from entry word `word`; returns false when the JIT cannot serve (wrong
// arch, arena exhausted mid-compile) and the caller falls back to the
// interpreter for this line. `stamp` is the caller's instruction counter —
// advanced per instruction by the emitted code, exactly as the line executor
// advances it. `until` is the batch's budget bound (runSteps' own local):
// chained hops re-check `until - stamp >= 8` exactly where the dispatcher
// would have, so a chained run can never overrun the batch.
bool jitRunLine(Cpu& c, u32 slot, u32 word, u64& stamp, u64 until);

// fetchDecoded's refill hook: a line was just (re)filled. Identical content
// keeps the compiled block; anything else invalidates the slot.
void jitNoteRefill(Cpu& c, u32 slot, u32 base, const u32* w);

} // namespace opm
