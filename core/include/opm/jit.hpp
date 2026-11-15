#pragma once
// The JIT: a native x86-64 transcription of Cpu::runSteps' line executor.
// Design and correctness argument: _plans/JIT_PLAN.md. The short form:
//
//   - Compiled blocks belong to a 32-byte FetchLine: one per word the line
//     is entered at, each running from its word to the end of the line.
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
#include <utility>
#include <vector>

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
    u32 off[8]{};    // arena offset of the block compiled for an entry at
                     // each word, or 0: none yet (compiled when first wanted)
    // Chain backrefs: the rel32 words in OTHER lines' blocks that jump into
    // this line's (JIT_PLAN §7 Stage B), as a list in JitCache::brefPool.
    // Severed — rewritten so the site re-resolves — the moment this JitLine
    // stops describing its line (refill with different content, way
    // eviction, dropAll), which is the whole of what makes a chained entry
    // equivalent to the dispatcher's ways probe.
    u32 brefHead = 0; // index of the first record, or 0
};

class JitCache {
  public:
    JitCache();
    ~JitCache();
    JitCache(const JitCache&) = delete;
    JitCache& operator=(const JitCache&) = delete;

    static constexpr u32 kLines = 65536; // == Cpu::kFetchLines, asserted there
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
    // 128 MB of code, bump-allocated; full -> drop everything and start
    // over. A block is one to two kilobytes and a game's flyby compiles
    // some forty thousand of them, so a wholesale reset is a rare event and
    // a recompile is microseconds: simplicity wins over an eviction policy.
    // (At 32 MB that flyby reset the arena every two seconds.)
    static constexpr size_t kArenaBytes = 128u << 20;

    bool ready() const { return arena_ != nullptr && live_; }

    JitLine line[kLines * kWays]; // slot-major: ways of slot s at [s*kWays..]
    u8 rr[kLines] = {};           // round-robin victim way per slot

    // Counters, always on: they are how the report says what the JIT did.
    u64 compiles = 0, enters = 0, insns = 0, resets = 0, refillKeeps = 0,
        refillDrops = 0, bails = 0;

    // Chain lifecycle (Stage B). Hops themselves are counted by emitted code
    // into Cpu::jitChainHops (r13-addressable); everything here is C++-side.
    u64 chainLinks = 0, chainSevers = 0, chainResolves = 0, chainMisses = 0;
    // The records of who jumps into a line: {the site's rel32, the next
    // record}. Index 0 is "none"; brefFree heads the records handed back.
    struct Bref {
        u32 site, next;
    };
    std::vector<Bref> brefPool = std::vector<Bref>(1);
    u32 brefFree = 0;
    // The mix census. What the blocks actually executed is the aim point
    // for the next lowering round: memOps counts shim round trips, fallbacks
    // counts execRow calls from inside blocks, and fbByRow names the rows —
    // it is how the s40 session found that 43% of the in-game window was
    // floating point and which ops they were. tscProbe/tscNative are wired
    // for the rdtsc split when a session needs it (writers currently absent;
    // the report prints them only when populated).
    u64 tscProbe = 0, tscNative = 0;
    u64 fallbacks = 0;
    // Every instruction a block called out for, by either path: the rows'
    // own counts added up (the direct call keeps only those).
    u64 fallbackTotal() const
    {
        u64 n = 0;
        for (u64 v : fbByRow)
            n += v;
        return n;
    }
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

    // 🔬 WHAT THE EMITTED BYTES ARE FOR (g4run --profile-host). A host sample
    // inside the arena says nothing by its address, and "translated code" is
    // half of the processor's thread: this is the list that turns an arena
    // offset back into the guest instruction it was compiled for and the
    // part of that instruction's code it lies in. Appended to in emission
    // order, so it is sorted by offset; kept only when asked for, emptied
    // with the arena.
    enum : u32 {
        kPartBody,     // the instruction's own work, in line
        kPartProbe,    // a load or store: the address and the page memo's probe
        kPartShim,     // a load or store through the call
        kPartCall,     // an instruction called out for, and its tests after
        kPartHop,      // leaving for the next block: counts, gate, the jump
        kPartCr,       // a condition register field built from a result
        kPartFp,       // floating-point arithmetic in line
        kPartN
    };
    static constexpr const char* kPartName[kPartN] = {
        "in line", "address + page probe", "memory call", "handler call",
        "hop to the next block", "condition field",
        "floating point in line"};
    static constexpr u32 kMapNoRow = 1023u; // not one instruction's (a line's end)
    bool mapOn = false;
    std::vector<std::pair<u32, u32>> map; // {arena offset, row | part << 10}
    // The tag of the code at an arena offset, or ~0u.
    u32 mapAt(u32 off) const;

    // r13-relative displacements into Cpu, measured off the live object at
    // bind() rather than via offsetof: Cpu holds std::map members, so it is
    // not standard-layout, and measuring the real object is both portable
    // and self-evidently right.
    struct Offs {
        i32 gpr0 = 0, pc = 0, cr = 0, xer = 0, lr = 0, ctr = 0;
        i32 pend = 0, curInsn = 0, halted = 0, batchBreak = 0;
        i32 msr = 0, fpr0 = 0, fpscr = 0;
        i32 fetchLine0 = 0; // fetchBase[0]; slot k at +k*flStride
        i32 flStride = 0;
        // Chain-gate fields (Stage B): the dispatcher's own `slow` inputs,
        // the fetch-translation MSR word, the budget bound and the hop
        // counter. pend4 covers the four adjacent pending bools with one
        // dword compare when bind() confirms adjacency (pend4Ok).
        i32 lineExecOff = 0, napping = 0, mmcr0 = 0, iabr = 0, xlMsr = 0;
        i32 raisedThis = 0; // execRow clears it before every handler
        i32 pend4 = 0, extIrq = 0, smi = 0, decP = 0, pmP = 0;
        i32 until = 0, chainHops = 0;
        i32 stampPtr = 0, cyc = 0; // Cpu::jitStampPtr, Cpu::jitCyc
        bool pend4Ok = false;
        // Memory read directly (Cpu::DpPage), for the loads and stores a
        // block makes without a call: the memo's key as the dispatcher
        // left it, the LRU bytes a hit writes, and where the memo's table
        // is (it is the processor's for life, so its address is a constant
        // of the compiled code).
        i32 jitMsr = 0; // Cpu::jitMsr, for the hop gate
        i32 dmLive = 0, dpKey = 0, dtlbLru0 = 0;
        u64 dpTable = 0;
    } offs;

    using EnterFn = void (*)(const void* entry, Cpu*);
    EnterFn enterFn = nullptr;

    // Wire the cache to one Cpu: measure offsets, emit trampoline/epilogue.
    void bind(Cpu& c);
    void dropAll();
    // Unlink every chain site that jumps into this block: each recorded
    // rel32 goes back to 0, which re-targets the site at its own resolver
    // thunk. Must run before the JitLine stops describing its (base, va) —
    // see the field comment on JitLine::bref.
    void sever(JitLine& jl);
    JitLine* lineFor(u32 slot, u32 paBase, u32 vaBase, const u32* w);
    bool compileEntry(Cpu& c, JitLine& jl, u32 slot, u32 word, const u32* w,
                      const u16* rows);

    bool movbe = false; // the host loads and stores byte-reversed in one step
    u8* arena_ = nullptr;
    size_t cap_ = 0, at_ = 0;
    size_t firstCode_ = 0; // end of trampoline+epilogue; resets rewind here
    u32 epilogue_ = 0;     // arena offset every exit ends at
    bool live_ = false;
};

// runSteps' door. Compiles the slot's line on demand and executes native code
// from entry word `word`; returns false when the JIT cannot serve (wrong
// arch, arena exhausted mid-compile) and the caller falls back to the
// interpreter for this line. `stamp` is the caller's instruction counter —
// advanced by the emitted code wherever something can read it and on the
// way out, to exactly what the line executor would have made it. `until` is
// the batch's budget bound (runSteps' own local): chained hops re-check
// `until - stamp >= 8` exactly where the dispatcher would have, so a chained
// run can never overrun the batch.
bool jitRunLine(Cpu& c, u32 slot, u32 word, u64& stamp, u64 until);

// fetchDecoded's refill hook: a line was just (re)filled. Identical content
// keeps the compiled block; anything else invalidates the slot.
void jitNoteRefill(Cpu& c, u32 slot, u32 base, const u32* w);

} // namespace opm
