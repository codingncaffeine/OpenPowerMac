// g4run — Sawtooth (Power Mac G4 AGP) machine runner. Loads the 1 MB New
// World boot ROM at 0xFFF00000 and executes from the hardware reset vector
// with authentic reset state. M-SAW-0 deliverable is the instrumentation:
// how far Open Firmware 3.x gets, which physical addresses it touches
// (the deduplicated unclaimed-access log IS the Uni-North/KeyLargo map),
// the exceptions it takes, and the last instructions before the stop.

#include "opm/cpu.hpp"
#include "opm/hostclock.hpp"
#include "opm/insn.hpp"
#include "opm/prof.hpp"
#include "opm/pace.hpp"
#include "opm/sawtooth.hpp"
#include "opm/snapshot.hpp"
#include "../../core/src/softfp.hpp" // fast-path counters in the report

#include <algorithm>
#include <chrono>
#include <thread>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <set>
#include <string>
#include <vector>

using namespace opm;

namespace {

// Cpu::excByVec is indexed by (vector >> 8) & 15, so the index alone reads as
// a magic number. Name it where it is printed.
const char* excVecName(u32 vi)
{
    switch (vi) {
    case 1: return "RESET";
    case 2: return "MACHINE-CHECK";
    case 3: return "DSI";
    case 4: return "ISI";
    case 5: return "EXTERNAL";
    case 6: return "ALIGNMENT";
    case 7: return "PROGRAM";
    case 8: return "FP-UNAVAIL";
    case 9: return "DECREMENTER";
    case 12: return "SYSCALL";
    case 13: return "TRACE";
    case 15: return "PERFMON";
    default: return "?";
    }
}

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

// ATA Manager function codes, from Apple's ATA Device Software Guide and
// TN1192/TN1098. 0x96 and 0x98 appear in no Apple document — 0x98 was
// decoded here as a driver-list ENUMERATOR (PB+0x32 selects the list,
// PB+0x18 is an index), so it is named from measurement, not a manual.
const char* ataFnName(u32 fn)
{
    switch (fn) {
    case 0x00: return "NOP";
    case 0x01: return "ExecIO";
    case 0x03: return "BusInquiry";
    case 0x04: return "QRelease";
    case 0x10: return "Abort";
    case 0x11: return "ResetBus";
    case 0x12: return "RegAccess";
    case 0x13: return "Identify";
    case 0x85: return "DrvrRegister";
    case 0x86: return "FindRefNum";
    case 0x87: return "DrvrDeregister";
    case 0x88: return "ModifyEventMask";
    case 0x89: return "EjectDrive";
    case 0x8A: return "GetDevConfig";
    case 0x8B: return "SetDevConfig";
    case 0x90: return "MgrInquiry";
    case 0x93: return "AddATABus";
    case 0x94: return "RemoveATABus";
    case 0x96: return "undocumented-96";
    case 0x98: return "EnumDrivers(decoded)";
    default: return "?";
    }
}

// Only codes whose ORIGIN has been located in this ROM get a name. Inventing
// plausible names for the rest is how a number quietly becomes a belief;
// everything else prints as bare decimal and hex so it stays a measurement.
// Use --find-code to locate a new one and then add it here.
const char* ataErrName(int e)
{
    switch (e) {
    case -56: return " nsDrvErr/no-such-drive";
    case -9325: return " ATALoad give-up (ffd9c74a)";
    case -9356: return " 'nope' sentinel at rec+0x74 (ffdc9a10)";
    default: return "";
    }
}

// OHCI operational-register names. A census line reading "+054 x3" says
// nothing; one reading "+054 RhPortStatus1 r=3" says the host looked at the
// port exactly three times and then stopped, which is the whole question.
const char* ohciRegName(u32 off)
{
    switch (off) {
    case 0x00: return "Revision       ";
    case 0x04: return "Control        ";
    case 0x08: return "CommandStatus  ";
    case 0x0C: return "InterruptStatus";
    case 0x10: return "InterruptEnable";
    case 0x14: return "InterruptDisabl";
    case 0x18: return "HCCA           ";
    case 0x1C: return "PeriodCurrentED";
    case 0x20: return "ControlHeadED  ";
    case 0x24: return "ControlCurrent ";
    case 0x28: return "BulkHeadED     ";
    case 0x2C: return "BulkCurrentED  ";
    case 0x30: return "DoneHead       ";
    case 0x34: return "FmInterval     ";
    case 0x38: return "FmRemaining    ";
    case 0x3C: return "FmNumber       ";
    case 0x40: return "PeriodicStart  ";
    case 0x44: return "LSThreshold    ";
    case 0x48: return "RhDescriptorA  ";
    case 0x4C: return "RhDescriptorB  ";
    case 0x50: return "RhStatus       ";
    case 0x54: return "RhPortStatus1  ";
    case 0x58: return "RhPortStatus2  ";
    default: return "?              ";
    }
}

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

// --mouse-beat BEAT:X,Y[:click|:dbl] — the struct and its parser live OUT
// HERE because the option ladder in main sits at MSVC's block-nesting
// ceiling (C1061): one more braced arm with its own ifs inside does not
// compile. The option is matched ahead of the ladder with the other
// hoisted flags, and it is called --mouse-beat because --mouse-at was
// already taken by the motion-cadence diagnostic.
struct MouseBeat {
    u64 at = 0;
    int x = 0, y = 0;
    int clicks = 0; // 0 = move only, 1 = click, 2 = double click
};
void parseMouseAt(const char* v, std::vector<MouseBeat>& out)
{
    MouseBeat mb;
    char* p = nullptr;
    mb.at = strtoull(v, &p, 0);
    bool ok = p && *p == ':';
    if (ok) {
        mb.x = static_cast<int>(strtol(p + 1, &p, 0));
        ok = *p == ',';
    }
    if (ok)
        mb.y = static_cast<int>(strtol(p + 1, &p, 0));
    if (ok && *p == ':') {
        if (!strcmp(p + 1, "click")) mb.clicks = 1;
        else if (!strcmp(p + 1, "dbl")) mb.clicks = 2;
        else ok = false;
    } else if (ok && *p)
        ok = false;
    if (ok)
        out.push_back(mb);
    else
        printf("-- --mouse-beat %s malformed (want BEAT:X,Y[:click|:dbl]); "
               "IGNORED\n",
               v);
}

// --type-beat AT:text / --cmd-beat AT:text (repeatable) — the single-shot
// --type/--cmd can open ONE thing, and launching a game from a CD takes
// two: select the volume icon and open its window, then select the title
// inside and open that. Repeatable beats make the whole launch
// keyboard-driven — the Finder's type-select needs no coordinates at all.
struct KeyBeat {
    u64 at = 0;
    std::string text;
};
void parseKeyBeat(const char* v, std::vector<KeyBeat>& out)
{
    KeyBeat kb;
    char* p = nullptr;
    kb.at = strtoull(v, &p, 0);
    if (!p || *p != ':' || !p[1]) {
        printf("-- --type-beat/--cmd-beat %s malformed (want AT:text); "
               "IGNORED\n",
               v);
        return;
    }
    // \xNN and \t, because the characters that matter here cannot be typed
    // on a command line: the Finder needs a literal MacRoman ™ (\xaa) to
    // type-select "Nanosaur™", and Tab to step to the next item.
    auto hex = [](char h) -> int {
        if (h >= '0' && h <= '9') return h - '0';
        if (h >= 'a' && h <= 'f') return h - 'a' + 10;
        if (h >= 'A' && h <= 'F') return h - 'A' + 10;
        return -1;
    };
    for (const char* q = p + 1; *q; ++q) {
        if (*q == '\\' && q[1] == 'x' && hex(q[2]) >= 0 && hex(q[3]) >= 0) {
            kb.text.push_back(static_cast<char>(hex(q[2]) * 16 + hex(q[3])));
            q += 3;
        } else if (*q == '\\' && q[1] == 't') {
            kb.text.push_back('\t');
            ++q;
        } else
            kb.text.push_back(*q);
    }
    out.push_back(kb);
}

} // namespace

int main(int argc, char** argv)
{
    const char* romPath = nullptr;
    const char* romDumpPath = nullptr; // --dump-rom FILE
    bool coverageAll = false; // --coverage-all
    const char* typeText = nullptr; // --type STR --type-at N: USB keystrokes
    u64 typeAt = 0;
    // --cmd STR --cmd-at N: the same keystrokes with Command held. The Finder
    // selects an icon by typed name and opens it with Command-O, so this drives
    // the machine while the pointer is still dead -- including launching the
    // installer sitting on the desktop.
    const char* cmdText = nullptr;
    u64 cmdAt = 0;
    u64 mouseAt = 0;   // --mouse-at N: inject USB mouse motion from N
    u64 clickAt = 0;   // --mouse-click-at N: one button down/up at N
    // --crsr-drag N: DIAGNOSTIC, not machine truth (like --em294-rts and
    // --wp-force). Drives the cursor the way a legacy 68K mouse driver does --
    // write MTemp/RawMouse/Mouse and then copy CrsrCouple into CrsrNew -- which
    // Apple's Technote HW01 says still works through a compatibility mode even
    // with the Cursor Device Manager installed. It answers one question: can
    // this OS draw a moved cursor at all? If it can, everything downstream of
    // the mouse module is fine and the whole defect is that nothing supplies
    // motion; if it cannot, the drawing path is broken too.
    u64 crsrDrag = 0;
    // --poke PA --poke-val V --poke-at N [--poke-every M]: DIAGNOSTIC, not
    // machine truth. Force one cell and see whether a decoded-but-unfixed
    // chain completes, exactly as --wp-force and --em294-rts do. Repeats,
    // because a gate the guest re-reads must stay forced.
    u32 pokePa = 0, pokeVal = 0;
    u64 pokeAt = 0, pokeEvery = 0;
    u32 pokeStride = 0, pokeCount = 1; // force a STRIDED SET: an array of
                                       // records each gate one field
    bool pokeSet = false;
    u64 clickHoldFor = 200000ull; // --mouse-hold N: instructions to hold it
    u64 mouseEvery = 2000000ull; // --mouse-every N: injection cadence
    // --mouse-dx/--mouse-dy: the injected delta. Making it DISTINCTIVE lets the
    // report be traced through the OS by searching a RAM dump for its bytes;
    // 8 and 4 are far too common to find.
    int mouseDx = 8, mouseDy = 4;
    // --mouse-vary: make every consecutive report DIFFER. A HID driver that keeps
    // a previous report to detect change would suppress an endless stream of
    // byte-identical ones -- which is what a fixed delta produces and what a
    // real mouse never sends.
    bool mouseVary = false;
    u64 mouseSent = 0;
    // The button state a real mouse carries in EVERY report. moveMouse()
    // assigns buttons_ outright, so injecting motion with a literal 0 cleared
    // whatever a click had just pressed -- which silently invalidated a
    // "motion with the button held" experiment: every one of those reports went
    // out with byte 0 = 0.
    u8 heldButtons = 0;
    u64 atiHideFrom = 0, atiHideTo = 0; // --ati-hide FROM TO
    std::vector<std::string> nvramSet; // --nvram-set NAME=VALUE
    u64 maxInsns = 50000000ull;
    size_t ramMb = 64; // the size validated to the desktop; see dimms()
    bool trace = false;
    u64 faultFrom = 0;                 // --fault-from N: skip firmware faults
    int excShow = 16;
    u32 disStart = 0, disEnd = 0;
    u32 ofSymAt[8] = {}, ofSymN = 0; // --of-word VA: name it
    u32 fastTb = 0; // extra TB cycles per instruction: compresses the
                    // ROM's wall-clock waits (harness lever, not machine
                    // truth — timings scale, ordering is preserved)
    u32 fastTbAfter = 0; // --fast-tb-after N: rate past the cutoff
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
    u32 rpPa = 0, rpEnd = 0;  // --rp PA [--rp-end PA]: read census
    u64 wpFrom = 0;                       // --wp-from N: ignore stores < N
    u32 wpMaxArg = 0;                     // --wp-max N: raise the 64 cap
    u32 wpPa = 0, wpEnd = 0;              // --wp PA [--wp-end PA]: cached
                                          // store watchpoint (CPU side)
    // --wp-va VA [--wp-va-end VA]: the same watchpoint aimed at a GUEST
    // VIRTUAL address. Guest data lives at addresses the guest knows and we
    // do not: a field inside a driver's context record is named by a
    // disassembly as a virtual address, and hand-computing its physical one
    // is a guess. The existing --watch-va translates for the BUS watch, which
    // an ordinary cached store never reaches.
    u32 wpVa = 0, wpVaEnd = 0;
    // --rp-va: the same for the READ census. Stores answer "who changed this";
    // only reads answer "who still consumes it", which is the question a stale
    // handle poses — and every address worth asking it about here is virtual.
    u32 rpVa = 0, rpVaEnd = 0;
    u32 wpForce = 0;                      // --wp-force V: DIAGNOSTIC, see
    bool wpForceSet = false;              // Cpu::wpForce — not machine truth
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
    // --mouse-beat BEAT:X,Y[:click|:dbl] (repeatable): scripted pointer
    // work, beats in instructions like --snapshot-at, coordinates absolute
    // from the top-left. The P7 loop needs a run to double-click a CD icon
    // and then a game without the user's hands; see the state machine ahead
    // of the run loop for how absolute positions are reached with a
    // delta-only pointer. Struct + parser are file-scope (C1061).
    std::vector<MouseBeat> mouseBeats;
    // --type-beat/--cmd-beat AT:text (repeatable): scripted keystrokes,
    // plain or with Command held. Two pairs launch a CD title with no
    // coordinates at all: type the volume name (desktop type-select),
    // Cmd-O, type the title, Cmd-O.
    std::vector<KeyBeat> typeBeats, cmdBeats;
    // --bench: run the loop THE APP RUNS. g4run's own loop carries the whole
    // instrument set — including a std::map probe per instruction for the
    // first-visit census — so its MIPS figure is the harness's speed, not the
    // machine's, and every recorded throughput number in this project was
    // measured through it. A speed claim has to be made about the shipping
    // path, which is capi's opm_run: step, tick, sync, deliver.
    bool bench = false;
    // --no-dev-gate: service every device on every instruction (the control
    // for the gate; see SawtoothBus::devGateOff).
    bool devGate = true;
    // --no-icache: bypass the fetch block cache, the cached decode and the
    // instruction translation cache (the control for all three).
    bool icache = true;
    bool dxlate = true;
    // --no-nap-skip: step an asleep processor one instruction at a time (the
    // control for machine/include/opm/pace.hpp).
    bool napSkipOn = true;
    // --no-batch: apply the clock and service the devices on every single
    // instruction, the way the machine did before batching existed. The
    // control for pace.hpp's batchSteps and for Cpu::runSteps.
    bool batchOn = true;
    // --no-line-exec: never run a whole resident block in one go. The control
    // for Cpu::runSteps' inner loop.
    bool lineExecOff = false;
    // --no-jit: interpret every line. The control for the compiled blocks.
    bool jitOff = false;
    // --no-jit-chain: compile blocks without chain sites, so every exit
    // returns to the dispatcher as v1 did. The control for Stage B.
    bool jitChainOff = false;
    // --jit-tsc: rdtsc the shims and the block entries (opm/jit.hpp).
    bool jitTsc = false;
    // --no-jit-direct: fallbacks go through the execRow shim, as before the
    // direct-call lowering. Its control.
    bool jitDirectOff = false;
    // 🔬 --fingerprint-every N: THE STATE DIFFERENTIAL.
    //
    // Every claim in this tree that some new machinery is transparent has been
    // settled by running with it off and comparing a handful of report lines —
    // stop pc, timebase, disk commands, painted bytes. That is a good gate and
    // it is a COARSE one: it compares four numbers out of a 256 MB machine,
    // and it can only say "the same" about a whole run.
    //
    // This compares everything. The fingerprint is FNV-1a over the complete
    // serialized machine — RAM, every register, every device cell — taken at
    // exact instruction counts, so two runs of the same window print two
    // streams of numbers and the FIRST line that differs names the window the
    // divergence happened in. The batch is clamped to land on each mark, so
    // the counts are the same in both runs whatever the batching does.
    //
    // tools/diffstate.sh runs the pair.
    u64 fpEvery = 0, fpNext = ~0ull;
    unsigned profHz = 0;               // --profile HZ (g4prof only)
    u64 verifyAt = 0, verifySteps = 0; // --verify-snapshot N M
    bool fastTbSet = false;            // CLI wins over a resumed value
    bool realtime = false;             // --realtime: TB from the host clock
    u32 callLo = 0, callHi = 0;        // --call-trace LO HI
    u64 callFrom = 0;                  // --call-trace-at N: gate
    // --trap-log N: the first N A-traps after --bp-from, each with the FILE it
    // names. The existing census says WHICH traps happen and how often, and
    // that is what narrowed this to file I/O — but "which file was not found"
    // is not answerable from a count. A File Manager parameter block carries
    // ioNamePtr at PB+18, pointing at a Pascal string.
    u32 trapLogN = 0, trapLogged = 0;
    u32 t68Lo = 0, t68Hi = 0;          // --trace-68k LO HI
    u32 t68Cap = 4000;                 // --trace-68k-lines N
    u64 watchFrom = 0;                 // --watch-from N: value-watch gate
    // --drv-from N: when the .ATALoad / on-disc-driver 68K trace opens.
    // The ATA Manager's device pass is at 2.4886 B and the old hard-coded
    // 2.8 B opened AFTER it, so the trace never covered the decision it
    // exists to explain.
    u64 drvFrom = 2800000000ull;
    u32 armOnPc = 0;                   // --arm-on-pc ADDR: arm on arrival
    u64 traceOfAt = 0;      // --trace-of AT N: OF word trace
    u32 traceOfLeft = 0;
    std::map<u32, std::string> ofNames;
    // The raw word trace is 90% stack primitives (swap/tuck/exit/(field)),
    // so 900 lines cover five thousand instructions and hide the shape of
    // the window. These three narrow it: --of-find dumps the dictionary,
    // --of-only keeps just the words worth reading, --of-hist counts every
    // entry over a window instead of listing them.
    std::vector<u32> ofSee;            // --of-see VA (repeatable)
    std::vector<u32> ofRefs;           // --of-refs VA (repeatable)
    std::vector<u32> ofCallers;        // --of-callers VA (repeatable)
    u32 ofSeeN = 160;                  // --of-see-n N
    std::vector<std::string> ofFind;   // --of-find SUBSTR (repeatable)
    std::vector<std::string> ofOnly;   // --of-only SUBSTR (repeatable)
    u64 ofHistFrom = 0, ofHistTo = 0;  // --of-hist FROM TO
    std::map<std::string, u64> ofHist;
    std::vector<std::pair<u64, std::string>> ofHistFirst;
    u32 bpPc[8] = {}, bpHit[8] = {}, bpN = 0; // --bp VA: register breakpoints
    u32 bp68Pc[8] = {}, bp68Hit[8] = {}, bp68N = 0; // --bp68 VA
    u32 bpMax = 4, bpDeref = 96;       // --bp-max N, --bp-deref BYTES
    u64 bpFrom = 0;                    // --bp-from N: ignore hits before N
    u64 dumpStructsAt = 0;             // --dump-structs-at N
    u64 dumpRamAt = 0;                 // --dump-ram-at N
    u32 peekAddr = 0, peekLen = 64;    // --peek VA [LEN] (guest virtual)
    u64 peekAt = 0;                    // --peek-at N
    u64 heartbeat = 0;                 // --heartbeat N: periodic digest
    u32 findVal = 0;                   // --find VALUE: scan RAM for a word
    bool em294Rts = false;             // --em294-rts: reseed the USB shim cell
    // The display's vertical blank as an INTERRUPT: what moves the mouse
    // pointer, and so on by default. --no-ati-vbl backs out to the poll-only
    // cell. See the note on R128Cell::vblEnabled — the capi takes the
    // constructor defaults, so this switch and the cell's must agree.
    bool atiVbl = true;
    // --kl-timer: answer the KeyLargo timer HWInit calibrates the timebase
    // against. It makes the guest's clock correct, and it is only survivable
    // with a --fast-tb that leaves room for the guest's own 60 Hz work — see
    // the note on SawtoothBus::klTimerOn, whose default this MUST match,
    // because the capi takes constructor defaults. --no-kl-timer restores the
    // 43x-slow clock every pre-session-27 baseline was recorded under.
    bool klTimer = true;
    // --no-sound: take the AWACS codec and the two audio DBDMA channels back
    // out, so the mac-io window falls through to KeyLargo storage the way it
    // did before 2026-07-31. It is the control for a machine-behaviour
    // change, not a preference: with the channels there the boot ROM waits
    // for its own startup chime to finish, because it spins on the output
    // channel's ACTIVE bit (fff868ec). See SawtoothBus::soundOn.
    bool soundOn = true;
    // --wav-out FILE: every byte the codec accepts, written out as a WAV at
    // the rate Sound Control selects. The boot ROM's chime arrives at about
    // 8.3 M instructions, so the whole output path is graded by a cold boot
    // rather than by a desktop that has to be persuaded to play something.
    const char* wavOut = nullptr;
    // --ati-vbl-tb N: timebase ticks between vertical blanks (0 = nominal
    // 60 Hz at 25 MHz). See the note in r128.hpp — under --fast-tb the guest's
    // own clock runs ~45x slower than the nominal timebase, so the nominal
    // period floods it with interrupts.
    u64 atiVblTb = 0;
    // --vbl-trace N: the first N latch/ack/iack/eoi events, in sequence. The
    // aggregate counters could say "acks=2 out of 14 blanks" and could not say
    // which blank went unanswered, or what the driver wrote when it did.
    int vblTrace = 0;
    const char* findStr = nullptr;     // --find-str TEXT: search RAM
    int findCode = 0;                  // --find-code N: where is N generated
    // The processor module's I2C cache descriptor at slave 0xAC: the boot ROM
    // finds, sizes, tests and ENABLES the 1 MB backside L2 (the whole um7400
    // §3.7.7 sequence) and records its size for the OS. Now the default —
    // `--no-cpu-cache-rom` restores a module that never answers.
    // `--cpu-cache-rom` is kept so old command lines still parse.
    bool cpuCacheRom = true;
    // --l2-inert: DIAGNOSTIC. L2CR is honoured in full, the array holds
    // nothing. Attributes a change to "the L2 is on" vs "our L2 serves the
    // wrong bytes".
    bool l2Inert = false;
    // --ohci-ndp N: force the root hub's downstream port count (DIAGNOSTIC).
    // Mac OS publishes NumPorts 0 for both root hubs while HcRhDescriptorA
    // reports NDP 2; driving NDP to a distinctive ODD value says whether the
    // OS reads this field at all, and settles the byte order at the same time
    // (the default 0x02000002 is byte-palindromic, so a swapped read of it is
    // indistinguishable from a correct one).
    u32 ohciNdp = 0;
    bool ohciNdpSet = false;
    // --ohci-port-power: model root-hub port power honestly (unpowered ports,
    // a real connect a few frames after power-up) instead of a port that
    // always reports a device. Opt-in: it takes the OS further into USB
    // enumeration but stalls the boot before the welcome screen. See
    // OhciCell::livePortPower. DEFAULT SINCE SESSION 18 -- it is the only model
    // that reaches a usable machine (USB input works, so the startup cache
    // dialog can be dismissed and Mac OS 9.1 boots to the Finder).
    // `--no-ohci-port-power` restores the pre-session-16 model.
    bool ohciPortPower = true;
    u64 disAt = 0;                     // --dis-at N: disassemble live VAs
    u64 findAt = 0;                    // --find-at N
    bool findSet = false;
    bool dumpStructsEnd = false;       // --dump-structs
    const char* eventsPath = nullptr;  // --events FILE (JSONL)
    const char* serialLogPath = nullptr; // --serial-log FILE
    u64 serialRate = 0;                // --serial-rate N
    u64 atiLogFrom = 0;                // --ati-log-from N
    u64 atiEngLog = 0;                 // --ati-engine-log N
    u64 ataLogFrom = 0;                // --ata-log-from N
    bool ataLatch = false;             // --ata-latch
    u64 wmapFrom = 0, wmapTo = 0;      // --wmap FROM TO
    u32 wmapPcBucket = 0;              // --wmap-pc BUCKET
    bool wmapPcSet = false;
    u32 watchVa = 0;                   // --watch-va ADDR
    u64 traceFrom = 0;                 // --trace-from N: full trace window
    // --callog PC: EVERY arrival at one address, as a table. "Who calls this,
    // how often, and with what" is the question that costs the most to answer
    // by hand here: the caller is only in LR, `ppcdis --callers` cannot see a
    // call made through a TVector (and returns 0, which reads as "no callers"),
    // and --trace-at-pc shows exactly ONE arrival. A count, an LR histogram and
    // the first N argument sets answer it in a single run.
    u32 callogPc = 0;
    u32 callogMax = 32;
    u64 callogHits = 0;
    std::map<u32, u64> callogByLr;
    struct CallogArgs {
        u64 at;
        u32 lr, r3, r4, r5, r6, r7, r8;
    };
    std::vector<CallogArgs> callogLog;
    u32 traceAtPc = 0;                 // --trace-at-pc ADDR: arm on arrival
    bool traceArmed = false;
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
        // The speed flags are matched HERE, ahead of the chain below,
        // and not inside it. MSVC caps how deeply blocks may nest and
        // an else-if ladder counts every rung: three more options put
        // this function over the limit (C1061). A `continue` costs no
        // depth at all.
        if (!strcmp(a, "--bench")) { bench = true; continue; }
        if (!strcmp(a, "--no-dev-gate")) { devGate = false; continue; }
        if (!strcmp(a, "--no-icache")) { icache = false; continue; }
        if (!strcmp(a, "--no-dxlate")) { dxlate = false; continue; }
        // The control for the nap skip. Every other cache and gate in here
        // ships with the switch that turns it off, because that is the only
        // way a claim about one gets settled.
        if (!strcmp(a, "--no-nap-skip")) { napSkipOn = false; continue; }
        if (!strcmp(a, "--no-batch")) { batchOn = false; continue; }
        if (!strcmp(a, "--no-line-exec")) { lineExecOff = true; continue; }
        // The control for the JIT (opm/jit.hpp), same rule as every cache
        // and gate here: the claim ships with the switch that turns it off.
        if (!strcmp(a, "--no-jit")) { jitOff = true; continue; }
        if (!strcmp(a, "--no-jit-chain")) { jitChainOff = true; continue; }
        // 🔬 The host-cycle split inside compiled blocks. Perturbs the run
        // (an rdtsc pair per shim call), so read it for SHARES and per-call
        // costs — never quote its MIPS.
        if (!strcmp(a, "--jit-tsc")) { jitTsc = true; continue; }
        if (!strcmp(a, "--no-jit-direct")) { jitDirectOff = true; continue; }
        // The softfp host fast paths, separately switchable so a speed series
        // can A/B either one inside ONE binary (s41's harness lesson: two
        // builds compared across a disk copy is not a measurement).
        if (!strcmp(a, "--no-fp-fast")) {
            sf::gHostFastOff |= sf::kFastOffSgl | sf::kFastOffDbl;
            continue;
        }
        if (!strcmp(a, "--no-fp-fast-sgl")) { sf::gHostFastOff |= sf::kFastOffSgl; continue; }
        if (!strcmp(a, "--no-fp-fast-dbl")) { sf::gHostFastOff |= sf::kFastOffDbl; continue; }
        // Up here for the same reason as the speed flags: the else-if ladder
        // below is already at MSVC's nesting limit (C1061).
        if (!strcmp(a, "--sound")) { soundOn = true; continue; }
        if (!strcmp(a, "--no-sound")) { soundOn = false; continue; }
        // --mouse-beat, NOT --mouse-at: that name was already taken by the
        // motion-cadence diagnostic below, and this ladder cannot grow.
        if (!strcmp(a, "--mouse-beat")) { parseMouseAt(next(), mouseBeats); continue; }
        // --tri-dump N: print the next N textured triangles' raw vertices
        // (see r128Set3dTriDump). Hoisted: the else-if ladder is at C1061.
        if (!strcmp(a, "--tri-dump")) {
            opm::r128Set3dTriDump(
                static_cast<int>(strtol(next(), nullptr, 0)));
            continue;
        }
        if (!strcmp(a, "--type-beat")) { parseKeyBeat(next(), typeBeats); continue; }
        if (!strcmp(a, "--cmd-beat")) { parseKeyBeat(next(), cmdBeats); continue; }
        if (!strcmp(a, "--wav-out") && i + 1 < argc) {
            wavOut = argv[++i];
            continue;
        }
        if (!strcmp(a, "--fingerprint-every") && i + 1 < argc) {
            fpEvery = strtoull(argv[++i], nullptr, 0);
            continue;
        }
        if (!strcmp(a, "--profile")) {
            profHz = static_cast<unsigned>(strtoul(next(), nullptr, 0));
            continue;
        }
        if (!strcmp(a, "--rom")) romPath = next();
        else if (!strcmp(a, "--dump-rom")) romDumpPath = next();
        else if (!strcmp(a, "--trace-of")) {
            traceOfAt = strtoull(next(), nullptr, 0);
            traceOfLeft = static_cast<u32>(strtoul(next(), nullptr, 0));
        }
        else if (!strcmp(a, "--of-callers"))
            ofCallers.push_back(static_cast<u32>(strtoul(next(), nullptr, 0)));
        else if (!strcmp(a, "--of-refs"))
            ofRefs.push_back(static_cast<u32>(strtoul(next(), nullptr, 0)));
        else if (!strcmp(a, "--of-see"))
            ofSee.push_back(static_cast<u32>(strtoul(next(), nullptr, 0)));
        else if (!strcmp(a, "--of-see-n"))
            ofSeeN = static_cast<u32>(strtoul(next(), nullptr, 0));
        else if (!strcmp(a, "--of-find")) ofFind.push_back(next());
        else if (!strcmp(a, "--of-only")) ofOnly.push_back(next());
        else if (!strcmp(a, "--of-hist")) {
            ofHistFrom = strtoull(next(), nullptr, 0);
            ofHistTo = strtoull(next(), nullptr, 0);
        }
        else if (!strcmp(a, "--cmd")) cmdText = next();
        else if (!strcmp(a, "--cmd-at")) cmdAt = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--type")) typeText = next();
        else if (!strcmp(a, "--type-at")) typeAt = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--mouse-at"))
            mouseAt = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--mouse-click-at"))
            clickAt = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--poke")) {
            pokePa = static_cast<u32>(strtoul(next(), nullptr, 0));
            pokeSet = true;
        }
        else if (!strcmp(a, "--poke-val"))
            pokeVal = static_cast<u32>(strtoul(next(), nullptr, 0));
        else if (!strcmp(a, "--poke-at"))
            pokeAt = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--poke-stride"))
            pokeStride = static_cast<u32>(strtoul(next(), nullptr, 0));
        else if (!strcmp(a, "--poke-count"))
            pokeCount = static_cast<u32>(strtoul(next(), nullptr, 0));
        else if (!strcmp(a, "--poke-every"))
            pokeEvery = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--crsr-drag"))
            crsrDrag = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--mouse-vary")) mouseVary = true;
        else if (!strcmp(a, "--mouse-dx"))
            mouseDx = static_cast<int>(strtol(next(), nullptr, 0));
        else if (!strcmp(a, "--mouse-dy"))
            mouseDy = static_cast<int>(strtol(next(), nullptr, 0));
        else if (!strcmp(a, "--mouse-hold"))
            clickHoldFor = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--mouse-every")) {
            mouseEvery = strtoull(next(), nullptr, 0);
            if (!mouseEvery) mouseEvery = 1ull; // never modulo by zero
        }
        else if (!strcmp(a, "--coverage-all")) coverageAll = true;
        else if (!strcmp(a, "--of-word")) {
            if (ofSymN < 8)
                ofSymAt[ofSymN++] =
                    static_cast<u32>(strtoul(next(), nullptr, 0));
        }
        else if (!strcmp(a, "--ati-hide")) {
            atiHideFrom = strtoull(next(), nullptr, 0);
            atiHideTo = strtoull(next(), nullptr, 0);
        }
        else if (!strcmp(a, "--nvram-set")) nvramSet.push_back(next());
        else if (!strcmp(a, "--ram")) ramMb = strtoul(next(), nullptr, 0);
        else if (!strcmp(a, "--max")) maxInsns = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--trace")) trace = true;
        else if (!strcmp(a, "--exc")) excShow = atoi(next());
        else if (!strcmp(a, "--fault-from"))
            faultFrom = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--dis")) {
            disStart = static_cast<u32>(strtoul(next(), nullptr, 0));
            disEnd = static_cast<u32>(strtoul(next(), nullptr, 0));
        }
        else if (!strcmp(a, "--fast-tb")) {
            fastTb = static_cast<u32>(strtoul(next(), nullptr, 0));
            fastTbSet = true;
        }
        else if (!strcmp(a, "--fast-tb-after"))
            fastTbAfter = static_cast<u32>(strtoul(next(), nullptr, 0));
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
        // ⚠ MSVC's else-if chain here is AT its nesting ceiling (C1061), so
        // the virtual forms JOIN these branches rather than adding new ones.
        else if (!strcmp(a, "--wp") || !strcmp(a, "--wp-va")) {
            const u32 at = static_cast<u32>(strtoul(next(), nullptr, 0));
            if (!strcmp(a, "--wp-va")) {
                wpVa = at;
                if (!wpVaEnd) wpVaEnd = at + 3u;
            } else {
                wpPa = at;
                if (!wpEnd) wpEnd = wpPa + 3u;
            }
        }
        else if (!strcmp(a, "--rp") || !strcmp(a, "--rp-va")) {
            const u32 at = static_cast<u32>(strtoul(next(), nullptr, 0));
            if (!strcmp(a, "--rp-va")) {
                rpVa = at;
                if (!rpVaEnd) rpVaEnd = at + 3u;
            } else {
                rpPa = at;
                if (!rpEnd) rpEnd = rpPa + 3u;
            }
        }
        else if (!strcmp(a, "--rp-end") || !strcmp(a, "--rp-va-end")) {
            const u32 at = static_cast<u32>(strtoul(next(), nullptr, 0));
            if (!strcmp(a, "--rp-va-end")) rpVaEnd = at;
            else rpEnd = at;
        }
        else if (!strcmp(a, "--wp-from"))
            wpFrom = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--wp-max"))
            wpMaxArg = static_cast<u32>(strtoul(next(), nullptr, 0));
        else if (!strcmp(a, "--wp-end") || !strcmp(a, "--wp-va-end")) {
            const u32 at = static_cast<u32>(strtoul(next(), nullptr, 0));
            if (!strcmp(a, "--wp-va-end")) wpVaEnd = at;
            else wpEnd = at;
        }
        else if (!strcmp(a, "--wp-force")) {
            wpForce = static_cast<u32>(strtoul(next(), nullptr, 0));
            wpForceSet = true;
        }
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
        else if (!strcmp(a, "--drv-from"))
            drvFrom = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--bp")) {
            if (bpN < 8)
                bpPc[bpN++] = static_cast<u32>(strtoul(next(), nullptr, 0));
        }
        else if (!strcmp(a, "--bp68")) {
            if (bp68N < 8)
                bp68Pc[bp68N++] =
                    static_cast<u32>(strtoul(next(), nullptr, 0)) & ~1u;
        }
        else if (!strcmp(a, "--bp-max"))
            bpMax = static_cast<u32>(strtoul(next(), nullptr, 0));
        else if (!strcmp(a, "--bp-deref"))
            bpDeref = static_cast<u32>(strtoul(next(), nullptr, 0)) & ~3u;
        else if (!strcmp(a, "--bp-from"))
            bpFrom = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--arm-on-pc"))
            armOnPc = static_cast<u32>(strtoul(next(), nullptr, 0));
        else if (!strcmp(a, "--dump-structs")) dumpStructsEnd = true;
        else if (!strcmp(a, "--peek")) {
            peekAddr = static_cast<u32>(strtoul(next(), nullptr, 0));
            peekLen = static_cast<u32>(strtoul(next(), nullptr, 0));
        }
        else if (!strcmp(a, "--find")) {
            findVal = static_cast<u32>(strtoul(next(), nullptr, 0));
            findSet = true;
        }
        else if (!strcmp(a, "--dis-at"))
            disAt = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--em294-rts")) em294Rts = true;
        else if (!strcmp(a, "--cpu-cache-rom")) cpuCacheRom = true; // now the default; kept so old command lines still work
        else if (!strcmp(a, "--no-cpu-cache-rom")) cpuCacheRom = false;
        else if (!strcmp(a, "--l2-inert")) l2Inert = true;
        else if (!strcmp(a, "--ohci-port-power")) ohciPortPower = true; // now the default; kept so old command lines still work
        else if (!strcmp(a, "--no-ohci-port-power")) ohciPortPower = false;
        else if (!strcmp(a, "--ohci-ndp")) {
            ohciNdp = static_cast<u32>(strtoul(next(), nullptr, 0));
            ohciNdpSet = true;
        }
        else if (!strcmp(a, "--find-str")) findStr = next();
        else if (!strcmp(a, "--find-code"))
            findCode = static_cast<int>(strtol(next(), nullptr, 0));
        else if (!strcmp(a, "--find-at"))
            findAt = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--heartbeat"))
            heartbeat = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--peek-at"))
            peekAt = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--dump-ram-at"))
            dumpRamAt = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--dump-structs-at"))
            dumpStructsAt = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--events")) eventsPath = next();
        else if (!strcmp(a, "--serial-log")) serialLogPath = next();
        else if (!strcmp(a, "--serial-rate"))
            serialRate = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--wmap-pc")) {
            wmapPcBucket = static_cast<u32>(strtoul(next(), nullptr, 0));
            wmapPcSet = true;
        }
        else if (!strcmp(a, "--wmap")) {
            wmapFrom = strtoull(next(), nullptr, 0);
            wmapTo = strtoull(next(), nullptr, 0);
        }
        else if (!strcmp(a, "--ata-latch")) ataLatch = true;
        else if (!strcmp(a, "--ata-log-from"))
            ataLogFrom = strtoull(next(), nullptr, 0);
        // ⚠ TWO FLAGS, ONE BRANCH, ON PURPOSE. This else-if chain is at MSVC's
        // block-nesting ceiling: adding one more link is C1061 "blocks nested
        // too deeply" and the file stops compiling. Any new option has to join
        // an existing branch until the chain is restructured into a table.
        else if (!strcmp(a, "--ati-log-from") ||
                 !strcmp(a, "--ati-engine-log")) {
            const bool eng = strstr(a, "engine") != nullptr;
            const u64 n = strtoull(next(), nullptr, 0);
            (eng ? atiEngLog : atiLogFrom) = n;
        }
        // Three card switches in one branch — see the note on --ati-log-from:
        // this chain is at MSVC's block-nesting ceiling, so collapsing links
        // is the only way to add an option. Folding these three together
        // frees two.
        else if (!strcmp(a, "--ati-vbl") || !strcmp(a, "--no-ati-vbl") ||
                 !strcmp(a, "--no-ati-2d") || !strcmp(a, "--no-agp-caps")) {
            if (!strcmp(a, "--no-ati-2d"))
                r128SetEngine2d(false);
            else if (!strcmp(a, "--no-agp-caps"))
                // Diagnostic A/B: build the machine WITHOUT the AGP
                // capability chains — the pre-2026-08-01 config space.
                // See the note on opm::gSkipAgpCaps.
                opm::gSkipAgpCaps = true;
            else
                atiVbl = strncmp(a, "--no", 4) != 0;
        }
        else if (!strcmp(a, "--kl-timer"))
            klTimer = true;
        else if (!strcmp(a, "--no-kl-timer"))
            klTimer = false;
        else if (!strcmp(a, "--vbl-trace")) vblTrace = atoi(next());
        else if (!strcmp(a, "--ati-vbl-tb"))
            atiVblTb = strtoull(next(), nullptr, 0);
        else if (!strcmp(a, "--trace-from"))
            traceFrom = strtoull(next(), nullptr, 0);
        // ⚠ C1061: --callog/--callog-max join these branches, they do not add
        // their own. Same reason --wp-va joins --wp.
        else if (!strcmp(a, "--trace-at-pc") || !strcmp(a, "--callog")) {
            const u32 v = static_cast<u32>(strtoul(next(), nullptr, 0));
            if (!strcmp(a, "--callog")) callogPc = v;
            else traceAtPc = v;
        }
        else if (!strcmp(a, "--trace-lines") || !strcmp(a, "--callog-max") ||
                 !strcmp(a, "--trap-log")) {
            const u64 v = strtoull(next(), nullptr, 0);
            if (!strcmp(a, "--callog-max")) callogMax = static_cast<u32>(v);
            else if (!strcmp(a, "--trap-log")) trapLogN = static_cast<u32>(v);
            else traceLines = v;
        }
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
                    "instruction-for-instruction)\n"
                    "       speed: --bench (the loop the app runs, no "
                    "instrumentation)\n"
                    "              --profile HZ (g4prof only: sample where "
                    "the host time goes)\n"
                    "       mouse: --mouse-beat N:X,Y[:click|:dbl] "
                    "(repeatable; beat in instructions,\n"
                    "              absolute px from top-left; dumps "
                    "ati_mouse_bK.ppm at each aim)\n"
                    "       keys:  --type-beat N:text --cmd-beat N:text "
                    "(repeatable; type-select and\n"
                    "              Command-chords at beats; no trailing "
                    "Return)\n");
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
    // Open Firmware configuration variables, set before the machine sees
    // the flash.
    //
    // The ROM carries the default "common" NVRAM partition, and its
    // `boot-device` is a FireWire path — `fwx/node@30e009e0138e08/sbp-2@…`
    // — left behind by whoever dumped this ROM. With `auto-boot?=true` and
    // a boot device that cannot exist, Open Firmware tries, fails, and
    // drops to a prompt. That is survivable only while OF is talking to the
    // serial port: once the display card's FCode runs, `output-device=screen`
    // wins, nothing reaches the serial console, and the injected script is
    // never read — so the machine that has a display is exactly the machine
    // that cannot be told to boot.
    //
    // The variables are a packed list of NAME=VALUE\0 inside the partition,
    // and the CHRP header checksum covers only the header, so an entry can
    // be rewritten and the remainder shifted up without touching it.
    // ⚠ MEASURED 2026-07-28: on the 4.2.8f1 Sawtooth ROM, **every** edit
    // to this partition trips Open Firmware's own validator, which answers
    //
    //   NVRAM-status: 2  at: 0
    //   NVRAM corrupted (init-nvram), cleaning it up...
    //
    // and reinitialises it, so the edit does NOT stick. Established with a
    // SAME-LENGTH replacement that shifted nothing and padded nothing
    // (`output-device=scca  ` for `output-device=screen`), which rules out
    // both the shift path and the sum-restoring bias — the earlier belief
    // that same-length edits were safe, and that shifting was what broke
    // them, are both wrong. Something beyond the byte sums is checked.
    //
    // Keep the tool: it is how that was established, and `--dump-rom` after
    // letting OF write its own NVRAM (setenv + reset-all) remains the way
    // to produce an image with valid checksums. But do not trust an edit to
    // survive a boot until this validator is understood.
    if (!nvramSet.empty())
        printf("-- nvram-set: NOTE — measured on this ROM, any edit to the "
               "config partition is rejected by OF's validator and the "
               "partition is reinitialised. See the comment above.\n");
    for (const std::string& kv : nvramSet) {
        const size_t eq = kv.find('=');
        if (eq == std::string::npos || eq == 0) {
            printf("-- nvram-set: ignoring %s (want NAME=VALUE)\n",
                   kv.c_str());
            continue;
        }
        // EVERY "common" partition, not the first. This ROM carries two —
        // the factory defaults and the live NVRAM image — and patching only
        // the defaults changes nothing, because Open Firmware reads the live
        // copy for as long as its checksum holds.
        std::vector<size_t> parts;
        for (size_t p = 0; p + 16 <= rom.size(); p += 4)
            if (rom[p] == 0x70 && memcmp(&rom[p + 4], "common", 6) == 0)
                parts.push_back(p);
        if (parts.empty()) {
            printf("-- nvram-set: no \"common\" partition in this rom\n");
            break;
        }
        for (const size_t part : parts) {
            const size_t dataAt = part + 16;
            const size_t dataEnd =
                part + size_t((rom[part + 2] << 8) | rom[part + 3]) * 16u;
            if (dataEnd <= dataAt || dataEnd > rom.size()) {
                printf("-- nvram-set: partition at %04zx has an "
                       "implausible length\n",
                       part);
                continue;
            }
            const std::string key = kv.substr(0, eq + 1); // "name="
            size_t at = std::string::npos;
            for (size_t p = dataAt; p + key.size() < dataEnd; ++p) {
                if ((p == dataAt || rom[p - 1] == 0) &&
                    memcmp(&rom[p], key.data(), key.size()) == 0) {
                    at = p;
                    break;
                }
            }
            if (at == std::string::npos) {
                printf("-- nvram-set: %s not in the partition at %04zx\n",
                       key.c_str(), part);
                continue;
            }
            size_t oldEnd = at;
            while (oldEnd < dataEnd && rom[oldEnd] != 0)
                ++oldEnd;
            ++oldEnd; // include the terminator
            const size_t oldLen = oldEnd - at;
            const size_t newLen = kv.size() + 1;
            if (newLen > oldLen) {
                printf("-- nvram-set: %s is longer than the entry it "
                       "replaces (%zu > %zu)\n",
                       kv.c_str(), newLen, oldLen);
                continue;
            }
            // Pad in place rather than shifting the rest of the list up.
            // Shifting looked harmless and was not: Open Firmware answered
            // "NVRAM corrupted (init-nvram), cleaning it up..." on the very
            // next boot, so something in this partition is position-
            // sensitive. The slack becomes a second entry, `oem-banner=`
            // padding, which `oem-banner?=false` makes inert.
            const size_t slack = oldLen - newLen;
            static const char kPad[] = "oem-banner=";
            const size_t padMin = sizeof kPad; // name + '=' + terminator
            // Slack smaller than a filler entry: SHIFT the rest of the
            // list up and let the freed bytes fall off the end of the
            // partition data, where they are already NUL padding.
            //
            // The in-place padding path exists because an earlier attempt
            // to shift produced "NVRAM corrupted (init-nvram), cleaning it
            // up..." — but the comment below records that the SAME message
            // followed an edit that moved nothing at all, and that the real
            // cure was preserving the partition's byte sums. So "position
            // sensitive" was never established; only "sum sensitive" was.
            // The sums are restored either way, and the boot-flash write
            // log is the positive control: OF rewriting the partition is
            // exactly what "cleaning it up" looks like from outside.
            //
            // Without this, shortening output-device=screen to scca is
            // refused over 2 bytes, and there is no way to move Open
            // Firmware's console onto the serial port — which is the only
            // way to talk to it once a display exists.
            if (slack != 0 && slack < padMin) {
                u32 pre[2] = {0, 0};
                for (size_t p = part; p < dataEnd; ++p)
                    pre[(p - part) & 1u] += rom[p];
                memmove(&rom[at + newLen], &rom[oldEnd], dataEnd - oldEnd);
                memcpy(&rom[at], kv.data(), kv.size());
                rom[at + kv.size()] = 0;
                std::fill(rom.begin() + static_cast<long>(dataEnd - slack),
                          rom.begin() + static_cast<long>(dataEnd), u8(0));
                // Put the sum-restoring bias at the very END of the
                // partition, not immediately after the shifted list. The
                // bytes right after the last entry's terminator are where
                // Open Firmware looks for the NEXT entry name, so a
                // non-zero byte there is a nameless variable — which is
                // exactly what "NVRAM corrupted (init-nvram), cleaning it
                // up..." was reporting. The tail of the partition is empty
                // padding and nobody parses it.
                for (u32 par = 0; par < 2; ++par) {
                    u32 after = 0;
                    for (size_t p = part; p < dataEnd; ++p)
                        if (((p - part) & 1u) == par)
                            after += rom[p];
                    for (size_t b = dataEnd; b-- > part + 16;)
                        if (((b - part) & 1u) == par && rom[b] == 0) {
                            rom[b] = static_cast<u8>(pre[par] - after);
                            break;
                        }
                }
                printf("-- nvram-set: %s at %04zx (shifted %zu bytes up)\n",
                       kv.c_str(), part, slack);
                continue;
            }
            // Preserve the partition's byte sum. Open Firmware answered
            // "NVRAM corrupted (init-nvram), cleaning it up..." to an edit
            // that kept every entry and moved nothing, so something beyond
            // the CHRP header checksum is being verified; the cheapest
            // invariant to keep is the sum itself, restored by biasing one
            // padding byte.
            u32 before[2] = {0, 0};
            for (size_t p = part; p < dataEnd; ++p)
                before[(p - part) & 1u] += rom[p];
            memcpy(&rom[at], kv.data(), kv.size());
            rom[at + kv.size()] = 0;
            if (slack) {
                memcpy(&rom[at + newLen], kPad, sizeof kPad - 1);
                std::fill(rom.begin() +
                              static_cast<long>(at + newLen + sizeof kPad - 1),
                          rom.begin() + static_cast<long>(at + oldLen - 1),
                          u8('x'));
                rom[at + oldLen - 1] = 0;
                // Bias one padding byte at each parity, so a 16-bit word
                // sum is preserved as well as a byte sum.
                for (u32 par = 0; par < 2; ++par) {
                    u32 after = 0;
                    for (size_t p = part; p < dataEnd; ++p)
                        if (((p - part) & 1u) == par)
                            after += rom[p];
                    size_t bias = at + oldLen - 2;
                    while (bias > at + newLen + sizeof kPad - 1 &&
                           ((bias - part) & 1u) != par)
                        --bias;
                    if (((bias - part) & 1u) != par)
                        continue;
                    rom[bias] =
                        static_cast<u8>(rom[bias] + (before[par] - after));
                }
            }
            printf("-- nvram-set: %s at %04zx (%zu bytes, %zu padded)\n",
                   kv.c_str(), part, oldLen - 1, slack);
        }
    }
    printf("-- rom: %zu bytes, ram: %zu MiB\n", rom.size(), ramMb);
    SawtoothBus bus(ramMb * 1024 * 1024, std::move(rom));

    char text[128];
    if (!disAt && disStart && disEnd > disStart) {
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
    bus.ati().vblEnabled = atiVbl;
    bus.klTimerOn = klTimer;
    bus.soundOn = soundOn;
    // A second tap on the codec, independent of the ring the application
    // drains, so a capture cannot be stolen by a consumer that is not there.
    std::vector<u8> wavPcm;
    if (wavOut)
        bus.sound().capture = &wavPcm;
    R128Cell::setVblTbPeriod(atiVblTb);
    R128Cell::setVblTrace(vblTrace);
    OpenPic::setTrace(vblTrace * 4); // iack+eoi pairs outnumber latch+ack
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

    if (serialRate)
        bus.rxPaceInsns = serialRate;
    if (atiLogFrom)
        bus.ati().logFrom = atiLogFrom;
    if (atiEngLog)
        r128SetEngineLog(static_cast<size_t>(atiEngLog));
    if (ataLogFrom) {
        bus.ataLogFrom = ataLogFrom;
        // One window, every ATA instrument. The DBDMA logs were unwindowed
        // and capped, so an --ata-log-from that put the register traffic on
        // the stall still reported DMA descriptors from the first second of
        // the boot — two views of "the same moment" that were 4 G apart.
        bus.ataDma().logFrom = ataLogFrom;
        bus.hdDma().logFrom = ataLogFrom;
    }
    if (ataLatch) {
        bus.hd().latchTrace = true;
        bus.cd().latchTrace = true;
    }
    // Audio interrupt accounting: set vs swallowed vs pulsed, per channel.
    dbdmaWatchIrq(&bus.sndOut(), &bus.sndIn());
    if (wmapTo) {
        bus.wmapFrom = wmapFrom;
        bus.wmapTo = wmapTo;
        bus.wmapPcBucket = wmapPcBucket;
        bus.wmapPcSet = wmapPcSet;
    }
    if (serialCr)
        bus.injectSerial("\r"); // CR in the escape window -> serial console

    Cpu cpu;
    cpu.attach(bus);
    // Bus-master coherency. Every DMA engine in this machine now announces
    // its transfers so the processor's L1/L2 answer them, exactly as the
    // 60x snoop does on real hardware.
    CpuSnoop snoop;
    snoop.cpu = &cpu;
    bus.attachSnoop(&snoop);
    cpu.reset(); // pc = 0xFFF00100, MSR[IP]: vectors in ROM — authentic
    cpu.wpPa = wpPa;   // physical store watchpoint, set before any stepping
    cpu.wpEnd = wpEnd;
    cpu.wpFrom = wpFrom;

    if (wpMaxArg) cpu.wpMax = wpMaxArg;
    cpu.rpPa = rpPa;   // physical READ watchpoint (census by reading pc)
    cpu.rpEnd = rpEnd;
    cpu.wpForce = wpForce;
    cpu.wpForceSet = wpForceSet;
    // Always go through the setter on a cold start: it sets the reset port
    // state that matches the model, and the two disagree. Calling it only for
    // the live model was safe while that was the opt-in case; now that it is
    // the default, --no-ohci-port-power must be able to put the old state back.
    bus.devGateOff = !devGate;
    cpu.fetchCacheOff = !icache;
    cpu.dxlCacheOff = !dxlate;
    cpu.lineExecOff = lineExecOff;
    cpu.jitOn = !jitOff;
    cpu.jitChainOff = jitChainOff;
    cpu.jitTscOn = jitTsc; // JitCache is created lazily; carried on the Cpu
    cpu.jitDirectOff = jitDirectOff;
    bus.ohci(0).setLivePortPower(ohciPortPower);
    bus.ohci(1).setLivePortPower(ohciPortPower);
    if (ohciNdpSet) {
        bus.ohci(0).setNdp(ohciNdp);
        bus.ohci(1).setNdp(ohciNdp);
    }
    u64 executed = 0;
    cpu.wpStamp = &executed; // the gate needs the live instruction counter
    // --wp-va resolution state. The retry is a COUNTDOWN and not an alignment
    // test on `executed`, because a batch or a nap-skip advances that counter
    // in jumps and an alignment test can be stepped straight over — an
    // instrument that never arms is indistinguishable from one that armed and
    // saw nothing.
    u64 wpVaNext = 0;
    u32 wpVaPa = 0;
    u64 wpVaAt = 0; // instruction count at which it resolved
    u64 rpVaNext = 0;
    u32 rpVaPa = 0;
    u32 wpVaMoves = 0, rpVaMoves = 0; // times the guest remapped the range
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
    // One call each, so this list and the capi's cannot drift apart again —
    // the capi's was missing the hard disk and its DMA channel, and a cell
    // with no stamp never runs a deferred command. See SawtoothBus::setStamp.
    bus.setPcRef(&cpu.st.pc);
    bus.setStamp(&executed);
    bus.lrRef = &cpu.st.lr;
    bus.pmu().portAIn = static_cast<u8>(viaA);
    cpu.l2Inert = l2Inert;
    if (l2Inert)
        printf("-- DIAGNOSTIC --l2-inert: L2CR is honoured but the array holds "
               "nothing, so every access reaches memory\n");
    bus.cpuModuleRom = cpuCacheRom;
    if (cpuCacheRom)
        printf("-- cpu module cache descriptor answered at i2c 0xac: the ROM "
               "finds, sizes, tests and ENABLES the 1 MB backside L2, and "
               "records it at [0x00003010]\n");
    else
        printf("-- --no-cpu-cache-rom: the processor module is silent, so the "
               "machine reports NO L2 and Mac OS puts up the cache alert\n");
    bus.atiVisibleAt = atiAt;
    bus.atiHideFrom = atiHideFrom;
    bus.atiHideTo = atiHideTo;

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
        // The port MODEL is a configuration choice, so it is re-applied from
        // the command line -- but by assignment, not through
        // setLivePortPower(), which resets live port state and would corrupt a
        // resumed machine. Pass the same flag you snapshotted with.
        bus.ohci(0).livePortPower = ohciPortPower;
        bus.ohci(1).livePortPower = ohciPortPower;
        // --ohci-ndp is a diagnostic override, not machine state, so it must
        // be re-applied on top of whatever the snapshot restored.
        if (ohciNdpSet) {
            bus.ohci(0).setNdp(ohciNdp);
            bus.ohci(1).setNdp(ohciNdp);
        }
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
    // Who owns the guest's clock, and the anchor it is paced against — shared
    // with the capi so the app and this tool cannot drift apart about it. See
    // machine/include/opm/hostclock.hpp.
    HostPacer pacer;
    pacer.anchor(cpu);
    // …and what this thread does when the guest has nothing for it. ⚠ IT IS
    // HERE BECAUSE THE APP HAS IT. --bench claims to run exactly the loop the
    // app runs, and a claim like that decays the moment one of them grows a
    // step the other has not: the last two times run-loop wiring existed in
    // two copies, one was missing the hard disk for weeks and the other
    // diagnosed a real-time-only audio defect from an instruction-paced
    // capture. See machine/include/opm/pace.hpp.
    HostWait idleWait;
    // The livelock guard the bench loop's wait is bounded by, and the
    // instruction count it watches to decide the machine is making progress.
    constexpr u64 kMaxIdleNs = 1000000000ull; // 1 s of waiting with the
                                              // instruction count unmoved
    u64 idleNs = 0, idleAt = 0;
    // Where the run STARTS. A resumed machine carries the whole history of
    // the boot that made the snapshot, so every rate divided by THIS run's
    // host seconds is a different question from the one being asked: a
    // real-time resume reported 8,983 ticks/host-s and 320 MIPS when its real
    // figures were 26 and 18.6.
    //
    // ⚠ Ticks has to be read through a FLUSH. bus.read32 reads RAM, and the
    // guest's copy of a low-memory global it writes 60 times a second is
    // normally sitting dirty in the L1 — the unflushed read came back 19,298
    // on a machine whose Ticks was 404,342.
    const u64 executedAtStart = executed, tbAtStart = cpu.st.tb;
    // Marks are counted from where THIS run starts, so a resumed pair lines up
    // whatever instruction count the snapshot carries.
    if (fpEvery)
        fpNext = executed + fpEvery;
    cpu.l1dFlushAll(true);
    cpu.l2FlushAll(true);
    const u32 ticksAtStart = bus.read32(0x0000416Au);
    // ⏱⏱ UNDER --realtime THE HOST CLOCK OWNS THE TIMEBASE OUTRIGHT: executing
    // an instruction stops advancing it, so the emulator's own throughput is
    // no longer a floor under the guest's clock. See Cpu::insnCycles for the
    // measurement that made this necessary — 2.04x real at the desktop, and
    // the top-up that was supposed to prevent it firing zero times.
    if (realtime)
        cpu.insnCycles = 0;
    // One rate for the whole run and no wall clock: then the extra cycles are
    // a constant per instruction and step() can carry them, which is one
    // tick() call per instruction instead of two (Cpu::extraCycles). The
    // two-rate and real-time modes vary it, so they keep the explicit call.
    const bool mergedTick =
        !realtime && !fastTbAfter && fastTbUntil == ~0ull;
    if (mergedTick)
        cpu.extraCycles = fastTb;
    auto tickPeripherals = [&]() {
        // Mark the clock advance as clock work. It used to inherit whatever
        // phase the caller was in, so --fast-tb's tick — a real cost, sixty
        // cycles' worth of accumulator per instruction — was billed to
        // "instrumentation" in one loop and to "tick" in the other, and the
        // same 300 M instructions appeared to spend twice as long ticking in
        // one run as the other. A profiler that moves cost between buckets
        // depending on who called it is the same failure as a watchpoint that
        // changes the run.
        OPM_PH(Tick);
        if (mergedTick) {
            // step() already advanced the clock by 1 + extraCycles.
        } else if (realtime) {
            // The whole clock, and nothing paces the other direction because
            // there is no other direction: instructions no longer advance the
            // timebase, so the guest can only be behind the host, never ahead
            // of it. What used to be a correction is now the source.
            pacer.sync(cpu);
        } else if (fastTb && executed < fastTbUntil) {
            cpu.tick(fastTb);
        } else if (fastTbAfter) {
            // Two rates, not one. Open Firmware's display bring-up contains
            // a wait long enough that at the OS-era rate it never ends —
            // 2.5 billion instructions parked in the Forth interpreter,
            // which reads as a hang and is not one; at 900 it clears and OF
            // reaches its boot code. The OS era cannot run at that rate,
            // because the NanoKernel reloads the decrementer about every
            // 6800 instructions and fifteen times that is a livelock. So
            // compress firmware time hard and hand the OS a sane clock.
            cpu.tick(fastTbAfter);
        }
        OPM_PH(DevTick);
        cpu.setExternalIrq(bus.serviceDevices(cpu.st.tb));
        OPM_PH(Other);
    };

    // How many instructions the run loop may hand to one batch, before
    // pace.hpp's own clamps. Three callers can only ever be handed one:
    //
    //  - --no-batch, the control;
    //  - the two-rate modes (--fast-tb-until / --fast-tb-after), where the
    //    harness's clock advance is a SECOND tick() from this loop rather
    //    than a constant carried by step(), so a batch of k instructions
    //    would be charged one instruction's worth of it;
    //  - and under --realtime the batch stops at the next clock sample, so
    //    the host-paced advance still lands every kBatchInsns (4096)
    //    instructions — see HostPacer::kBatchInsns, which capi shares.
    // Stop exactly on the next fingerprint mark, or the two runs being
    // compared are photographed at different instructions and every line
    // differs. Applies to the nap skip as well as to the batch — both charge
    // many instructions at once.
    auto fpClamp = [&](u64 done, u64 want) -> u64 {
        if (fpEvery && fpNext > done && fpNext - done < want)
            return fpNext - done;
        return want;
    };

    // The whole machine, as one number, at an exact instruction count. See
    // fpEvery. Costs a full serialization — about a fifth of a second for a
    // 256 MB machine — so it is a per-100-M-instruction instrument, not a
    // per-instruction one.
    auto fingerprint = [&](u64 done) {
        if (!fpEvery || done < fpNext)
            return;
        const HarnessState h{done,      fastTb,   fastTbUntil, parkSeen,
                             parkArmed, ataPoked, emPoked};
        printf("FP @%llu %016llx\n", static_cast<unsigned long long>(done),
               static_cast<unsigned long long>(
                   snapshotFingerprint(cpu, bus, h)));
        fflush(stdout);
        // With --snapshot-out, the FIRST mark is also written out in full.
        // A fingerprint says two machines differ; two snapshots say WHERE,
        // which is the question that follows immediately.
        if (snapOut) {
            SnapWriter w;
            saveSnapshot(cpu, bus, h, w);
            writeSnapshotFile(snapOut, w.buf);
            snapOut = nullptr;
        }
        while (fpNext <= done)
            fpNext += fpEvery;
    };

    auto batchBudget = [&](u64 done) -> u64 {
        if (!batchOn || (!mergedTick && !realtime))
            return 1;
        u64 want = maxInsns - done;
        if (realtime && want > HostPacer::kBatchInsns)
            want = HostPacer::kBatchInsns; // the clock's grain — see there
        return fpClamp(done, want);
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

    // Translate a guest VIRTUAL range for an instrument, with no side effect
    // on the machine being observed. Deliberately does NOT flush the caches
    // the way guest() must: a translation needs no data, and an observer that
    // writes invents bugs. Returns false until the mapping exists; sets `flat`
    // false when the range crosses into a different physical page, which a
    // single [pa, paEnd] watch cannot express.
    auto xlateRange = [&](u32 va, u32 vaEnd, u32& pa0, u32& pa1, bool& flat) {
        cpu.mmuProbe = true;
        const CpuState saved = cpu.st;
        const bool raised = cpu.raisedThisStep;
        cpu.st.msr |= 0x30u;
        const CpuState armed = cpu.st;
        const bool ok0 = cpu.translate(va, false, false, pa0);
        cpu.st = armed;
        const bool ok1 = cpu.translate(vaEnd, false, false, pa1);
        cpu.st = saved;
        cpu.raisedThisStep = raised;
        cpu.mmuProbe = false;
        flat = ok0 && ok1 && pa1 - pa0 == vaEnd - va;
        return ok0 && ok1;
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

    std::map<u32, u64> trapAll; // global A-trap census (see below)
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

    // --- The scripted pointer (--mouse-beat) -------------------------------
    //
    // The guest's cursor is positioned by DELTAS: USB reports feed the
    // Cursor Device Manager's accumulators and JCrsrTask drains them into
    // the pointer once per vblank. An absolute coordinate is therefore
    // reached in two moves — pin the pointer into the top-left corner with
    // a delta no screen can absorb, let the drain clamp it at (0,0), then
    // walk out to the target. Every stage advances on the machine's OWN
    // signals (OHCI reports the injection delivered, then a couple of
    // vblanks for the guest-side drain), with an instruction timeout so a
    // stalled guest cannot stall the script; fixed waits are how a script
    // works at one --fast-tb setting and silently misses at another.
    //
    // Progress is stated as NUMBERS (the cursor-position register at each
    // stage) — the per-beat ppm dumps are calibration artifacts for the
    // user's eyes, never this script's input.
    std::sort(mouseBeats.begin(), mouseBeats.end(),
              [](const MouseBeat& a, const MouseBeat& b) {
                  return a.at < b.at;
              });
    size_t mbIdx = 0;
    for (; mbIdx < mouseBeats.size() && mouseBeats[mbIdx].at < executed;
         ++mbIdx)
        printf("-- mouse beat @%llu SKIPPED: before this run's first "
               "instruction (%llu)\n",
               static_cast<unsigned long long>(mouseBeats[mbIdx].at),
               static_cast<unsigned long long>(executed));
    // The scripted keystrokes ride the same discipline: sorted, fired at
    // exact instruction beats, stale ones named rather than fired blind.
    std::sort(typeBeats.begin(), typeBeats.end(),
              [](const KeyBeat& a, const KeyBeat& b) { return a.at < b.at; });
    std::sort(cmdBeats.begin(), cmdBeats.end(),
              [](const KeyBeat& a, const KeyBeat& b) { return a.at < b.at; });
    size_t tbIdx = 0, cbIdx = 0;
    for (; tbIdx < typeBeats.size() && typeBeats[tbIdx].at < executed; ++tbIdx)
        printf("-- type-beat @%llu SKIPPED: before this run's first "
               "instruction (%llu)\n",
               static_cast<unsigned long long>(typeBeats[tbIdx].at),
               static_cast<unsigned long long>(executed));
    for (; cbIdx < cmdBeats.size() && cmdBeats[cbIdx].at < executed; ++cbIdx)
        printf("-- cmd-beat @%llu SKIPPED: before this run's first "
               "instruction (%llu)\n",
               static_cast<unsigned long long>(cmdBeats[cbIdx].at),
               static_cast<unsigned long long>(executed));
    int mbStage = 0;
    int mbHome = 0; // closed-loop correction rounds used this beat
    u64 mbInsn0 = 0, mbIdleVbl = 0;
    bool mbIdleSeen = false;
    constexpr u64 kMbStageTimeout = 60000000ull; // insns; a safety valve
    // A screen dump for the user at any beat (the final report keeps its
    // own, with the scanline census). Cursor composited: these exist to
    // calibrate pointer coordinates, and the pointer lives in the cursor
    // engine, not in VRAM.
    auto dumpScreen = [&](const char* path) {
        const R128Scan sc = r128ScanDecode(bus.ati());
        if (!sc.enabled || !sc.bypp || sc.w < 64 || sc.w > 2048 ||
            sc.h < 64 || sc.h > 1536) {
            printf("-- screen dump %s SKIPPED: fmt %u (%s) not renderable\n",
                   path, sc.fmt, r128ScanFmtName(sc.fmt));
            return;
        }
        std::vector<u8> frame(size_t(sc.w) * sc.h * 4u);
        for (u32 y = 0; y < sc.h; ++y)
            r128ScanRow(bus.ati(), sc, y,
                        frame.data() + size_t(y) * sc.w * 4u);
        r128ScanCursor(bus.ati(), sc, frame.data());
        FILE* pf = fopen(path, "wb");
        if (!pf)
            return;
        fprintf(pf, "P6\n%u %u\n255\n", sc.w, sc.h);
        for (size_t k = 0; k < size_t(sc.w) * sc.h; ++k) {
            const u8 rgb[3] = {frame[k * 4 + 2], frame[k * 4 + 1],
                               frame[k * 4 + 0]};
            fwrite(rgb, 1, 3, pf);
        }
        fclose(pf);
        printf("-- screen dumped: %s (%ux%u fmt %u %s)\n", path, sc.w,
               sc.h, sc.fmt, r128ScanFmtName(sc.fmt));
    };
    auto mouseMark = [&]() {
        mbInsn0 = executed;
        mbIdleSeen = false;
        mbIdleVbl = 0;
    };
    // Delivered-and-drained: the OHCI accumulators are empty (every injected
    // report is on the wire), and settleVbls vblanks have passed since —
    // JCrsrTask has had its turns at the guest-side accumulators.
    auto mouseSettled = [&](u64 settleVbls) {
        if (executed - mbInsn0 >= kMbStageTimeout) {
            printf("-- mouse beat %zu stage %d TIMED OUT after %llu insns "
                   "(vbl %llu, mouseIdle=%d); advancing anyway\n",
                   mbIdx + 1, mbStage,
                   static_cast<unsigned long long>(kMbStageTimeout),
                   static_cast<unsigned long long>(bus.ati().vblanks),
                   bus.ohci(1).mouseIdle() ? 1 : 0);
            return true;
        }
        if (!mbIdleSeen) {
            if (!bus.ohci(1).mouseIdle())
                return false;
            mbIdleSeen = true;
            mbIdleVbl = bus.ati().vblanks;
        }
        return bus.ati().vblanks >= mbIdleVbl + settleVbls;
    };
    auto mouseInject = [&](int dx, int dy, u8 btn, const char* what) {
        bus.ohci(1).moveMouse(dx, dy, btn);
        bus.deviceStateChanged();
        const u32 posn = bus.ati().peek(0x0264);
        printf("-- mouse beat %zu: %s @%llu (cursor reg %u,%u  vbl %llu)\n",
               mbIdx + 1, what, static_cast<unsigned long long>(executed),
               (posn >> 16) & 0xFFFFu, posn & 0xFFFFu,
               static_cast<unsigned long long>(bus.ati().vblanks));
        fflush(stdout);
        mouseMark();
    };
    auto mouseDone = [&]() {
        const u32 posn = bus.ati().peek(0x0264);
        printf("-- mouse beat %zu/%zu COMPLETE @%llu (cursor reg %u,%u)\n",
               mbIdx + 1, mouseBeats.size(),
               static_cast<unsigned long long>(executed),
               (posn >> 16) & 0xFFFFu, posn & 0xFFFFu);
        fflush(stdout);
        ++mbIdx;
        mbStage = 0;
        mbHome = 0;
    };
    // Beats run strictly in order; a beat whose time arrives while an
    // earlier sequence is mid-flight starts when that sequence completes.
    auto mouseStep = [&]() {
        if (mbIdx >= mouseBeats.size())
            return;
        const auto& mb = mouseBeats[mbIdx];
        switch (mbStage) {
        case 0: // waiting for the beat
            if (executed < mb.at)
                return;
            mouseInject(-4096, -4096, 0, "pin to top-left");
            mbStage = 1;
            break;
        case 1: // pinned and drained: walk out to the target
            if (!mouseSettled(3))
                return;
            mouseInject(mb.x, mb.y, 0, "move to target");
            mbStage = 2;
            break;
        case 2: { // walking onto the target: CLOSED LOOP on the cursor reg
            if (!mouseSettled(2))
                return;
            // The guest applies an acceleration curve to pointer deltas, so
            // one big move OVERSHOOTS — measured on the first scripted run:
            // commanded +590,+128 from the pinned corner, landed (638,156),
            // and the double-click missed the icon. Home in instead: inject
            // target-minus-actual, let it settle, read the cursor register
            // again. Every correction is smaller than the last, the curve
            // flattens toward 1:1, and a couple of rounds land it — steered
            // entirely by the register, which is a number in the log.
            const u32 posn = bus.ati().peek(0x0264);
            const int cx = int((posn >> 16) & 0xFFFFu);
            const int cy = int(posn & 0xFFFFu);
            const int ex = mb.x - cx, ey = mb.y - cy;
            if ((ex > 2 || ex < -2 || ey > 2 || ey < -2) && mbHome < 6) {
                ++mbHome;
                mouseInject(ex, ey, 0, "home toward target");
                return; // stay in stage 2; the clocks were re-marked
            }
            if (mbHome >= 6)
                printf("-- mouse beat %zu: homing gave up at (%d,%d), "
                       "wanted (%d,%d); clicking anyway\n",
                       mbIdx + 1, cx, cy, mb.x, mb.y);
            mbHome = 0;
            {
                char nm[64];
                snprintf(nm, sizeof nm, "ati_mouse_b%zu.ppm", mbIdx + 1);
                dumpScreen(nm);
            }
            if (mb.clicks == 0) {
                mouseDone();
                return;
            }
            mouseInject(0, 0, 1, "button down");
            mbStage = 3;
            break;
        }
        case 3:
            if (!mouseSettled(1))
                return;
            mouseInject(0, 0, 0, "button up");
            mbStage = 4;
            break;
        case 4:
            if (!mouseSettled(1))
                return;
            if (mb.clicks == 2) {
                // Two downs inside the guest's DoubleTime (default 12
                // ticks): one vblank of separation keeps the pair well
                // under it while still giving each transition its poll.
                mouseInject(0, 0, 1, "button down (2nd)");
                mbStage = 5;
            } else
                mouseDone();
            break;
        case 5:
            if (!mouseSettled(1))
                return;
            mouseInject(0, 0, 0, "button up (2nd)");
            mbStage = 6;
            break;
        case 6:
            if (!mouseSettled(1))
                return;
            mouseDone();
            break;
        }
    };

    if (profHz) {
#if OPM_PROFILING
        if (prof::start(profHz, &cpu.st.pc))
            printf("-- profiling at %u Hz%s\n", profHz,
                   bench ? " (bench loop)" : "");
#else
        printf("-- --profile ignored: this binary carries no markers. Build "
               "and run g4prof instead.\n");
#endif
    }
    // THE APP'S LOOP, and nothing else. Identical to capi's opm_run: step the
    // processor, advance the machine's clock, recompute the interrupt lines,
    // deliver. Everything g4run does around this is measurement, and a speed
    // number that includes the measurement is a number about g4run.
    if (bench)
        while (executed < maxInsns && !cpu.halted) {
            // The scripted pointer advances between batches: a batch is
            // bounded by the next device deadline (the vblank at the
            // longest), so stage latency stays under a frame.
            if (mbIdx < mouseBeats.size())
                mouseStep();
            // ⏳ NOTHING TO DO, AND THE CLOCK COMES FROM OUTSIDE — so wait for
            // the guest's next deadline instead of spinning through asleep
            // steps to discover the same thing. The previous trip round the
            // loop ended in tickPeripherals(), so the clock and the interrupt
            // lines are already as fresh as they get; going round again rather
            // than running, because the wait moved the clock and the devices
            // have to be looked at before any instruction is.
            //
            // ⚠ THE BOUND IS A LIVELOCK GUARD, NOT A PACING CONSTANT. Every
            // wait ends at a real deadline — deviceDueTb is the USB frame
            // clock, a millisecond out at most — so the guest normally wakes
            // and this never binds. What it catches is a core that naps with
            // interrupts disabled, which is a machine that would be hung on
            // real hardware too: without it an unattended measuring run would
            // hang instead of reaching --max and reporting the stop.
            if (realtime && idleNs < kMaxIdleNs) {
                if (executed != idleAt) { // it moved: this is not a livelock
                    idleAt = executed;
                    idleNs = 0;
                }
                if (const u64 w = hostIdleWait(cpu, bus, idleWait)) {
                    idleNs += w;
                    tickPeripherals();
                    continue;
                }
            }
            // Same call, same header, same order as capi's opm_run — see
            // machine/include/opm/pace.hpp. If this loop and the app's stop
            // agreeing, the number this tool reports is about this tool.
            //
            // ⚠ NOT UNDER --realtime, where the timebase comes from the host
            // clock: charging a run of asleep steps to it would advance the
            // clock twice. capi's real-time path is excluded for the same
            // reason, and for the same reason it is only a stopgap there.
            if (const u64 n =
                    (realtime || !napSkipOn)
                        ? 0
                        : napSkip(cpu, bus,
                                  fpClamp(executed, maxInsns - executed))) {
                executed += n;
                tickPeripherals();
                fingerprint(executed);
                continue;
            }
            runBatch(cpu, bus, executed, batchBudget(executed));
            tickPeripherals();
            fingerprint(executed);
        }

    while (!bench && executed < maxInsns && !cpu.halted) {
        const u32 pc = cpu.st.pc;
        // A machine that cannot restart. Open Firmware's `reset-all` sends
        // PMU_RESET and then spins until the world comes back; without this
        // it spun forever, which blocked every route that needs firmware
        // configuration to survive into a second pass. Devices keep their
        // state — this is the CPU restart the firmware is waiting for, not a
        // power cycle, and pretending otherwise would throw away the
        // attached media and the BAR assignments the next pass re-reads.
        if (bus.pmu().resetRequest) {
            bus.pmu().resetRequest = false;
            printf("-- PMU reset @%llu: restarting the processor "
                   "(devices keep their state)\n",
                   static_cast<unsigned long long>(executed));
            fflush(stdout);
            cpu.reset();
            bus.systemReset();
            cpu.wpPa = wpPa;
            cpu.wpEnd = wpEnd;
            cpu.wpFrom = wpFrom;
            cpu.wpStamp = &executed;
            if (wpMaxArg) cpu.wpMax = wpMaxArg;
            cpu.rpPa = rpPa;
            cpu.rpEnd = rpEnd;
            cpu.wpForce = wpForce;
            cpu.wpForceSet = wpForceSet;
            continue;
        }
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
        if (mbIdx < mouseBeats.size())
            mouseStep();
        fingerprint(executed);
        cpu.step();
        OPM_PH(Instr);
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
        // --wp-va: the CPU-side store watchpoint aimed at a guest VIRTUAL
        // address. Same translation as --watch-va above, a different watch.
        // That one lives in Bus::write and never sees an ordinary cached
        // store, which is how a field PROVED to change reported zero hits and
        // read as "nothing ever writes this".
        // ⛔ RE-AIM, do not resolve once. Open Firmware maps low memory one to
        // one and Mac OS does not: the same VA that answers PA 0078b000 in the
        // firmware era answers 0078f03c under the OS. A watch resolved on the
        // first mapping that appears spends the run on bytes that stopped
        // being the field, reports zero stores, and that reads exactly like
        // "nothing ever writes it" — which is the answer this instrument was
        // built to stop getting wrong. Every move is printed, and the report
        // says how many there were.
        if (wpVa && executed >= wpVaNext) {
            wpVaNext = executed + 0x40000ull;
            u32 pa0 = 0, pa1 = 0;
            bool flat = false;
            if (xlateRange(wpVa, wpVaEnd, pa0, pa1, flat) && pa0 != wpVaPa) {
                if (wpVaPa) ++wpVaMoves;
                cpu.wpPa = pa0;
                cpu.wpEnd = flat ? pa1 : (pa0 | 0xFFFu);
                wpVaPa = pa0;
                wpVaAt = executed;
                printf("-- wp-va %08x..%08x -> PA %08x..%08x%s @%llu%s\n", wpVa,
                       wpVaEnd, cpu.wpPa, cpu.wpEnd,
                       flat ? "" : "  WARNING: crosses a page, clamped",
                       static_cast<unsigned long long>(executed),
                       wpVaMoves ? "  (RE-AIMED: the guest remapped it)" : "");
                fflush(stdout);
            }
        }
        if (rpVa && executed >= rpVaNext) {
            rpVaNext = executed + 0x40000ull;
            u32 pa0 = 0, pa1 = 0;
            bool flat = false;
            if (xlateRange(rpVa, rpVaEnd, pa0, pa1, flat) && pa0 != rpVaPa) {
                if (rpVaPa) ++rpVaMoves;
                cpu.rpPa = pa0;
                cpu.rpEnd = flat ? pa1 : (pa0 | 0xFFFu);
                rpVaPa = pa0;
                printf("-- rp-va %08x..%08x -> PA %08x..%08x%s @%llu%s\n", rpVa,
                       rpVaEnd, cpu.rpPa, cpu.rpEnd,
                       flat ? "" : "  WARNING: crosses a page, clamped",
                       static_cast<unsigned long long>(executed),
                       rpVaMoves ? "  (RE-AIMED: the guest remapped it)" : "");
                fflush(stdout);
            }
        }
        // A single-step view, windowed in TIME. --trace starts at
        // instruction zero, which is useless three billion in; the range
        // tracers only see code whose address you already know. What was
        // missing is "show me every instruction around HERE" — which,
        // resumed from a snapshot, costs seconds and is what a debugger's
        // step button actually provides.
        // --trace-at-pc ADDR: start the trace window when the pc first ARRIVES
        // at ADDR, not at an instruction count. A count cannot be aimed at a
        // routine -- a breakpoint reports the count of one entry, and by the
        // next run the same count lands somewhere unrelated, which is exactly
        // how a 44-line window aimed at a shim landed in the idle loop. Code
        // addresses are stable across resumes of one snapshot; counts are not.
        // ⚠ --bp-from GATES THIS TOO. Arming on the FIRST arrival is wrong
        // whenever the interesting visit is not the first one: a routine that
        // runs healthily at boot and pathologically at the desktop arms during
        // the boot and shows the healthy pass. --bp-from already means "ignore
        // hits before N" for breakpoints; it means the same here.
        // --callog: every arrival, not the first. `pc` is the instruction
        // ABOUT to run, so the arguments and LR are exactly as the caller left
        // them. ⚠ If the address is the top of a loop rather than an entry
        // point, each iteration counts as an arrival — which is the honest
        // reading of "arrived here", and the LR histogram makes the two cases
        // tell themselves apart: one caller repeated is a loop, many callers
        // is an entry point.
        if (callogPc && pc == callogPc && executed >= bpFrom) {
            ++callogHits;
            ++callogByLr[cpu.st.lr];
            if (callogLog.size() < callogMax)
                callogLog.push_back({executed, cpu.st.lr, cpu.st.gpr[3],
                                     cpu.st.gpr[4], cpu.st.gpr[5],
                                     cpu.st.gpr[6], cpu.st.gpr[7],
                                     cpu.st.gpr[8]});
        }
        if (traceAtPc && !traceArmed && pc == traceAtPc && executed >= bpFrom) {
            traceArmed = true;
            traceFrom = executed;
            printf("-- trace armed at pc=%08x @%llu\n", traceAtPc,
                   static_cast<unsigned long long>(executed));
            fflush(stdout);
        }
        if (traceFrom && executed >= traceFrom &&
            executed < traceFrom + traceLines) {
            char tt[128];
            disassemble(cpu.curInsn, pc, tt, sizeof tt, Style::Gnu);
            printf("STEP @%llu %08x%s %08x  %-28s",
                   static_cast<unsigned long long>(executed), pc, sym(pc),
                   cpu.curInsn, tt);
            if ((pc & 0xFF000000u) == 0x68000000u)
                printf(" | 68K %08x%s op=%04x D0=%08x A0=%08x r3=%08x "
                       "r4=%08x\n",
                       cpu.st.gpr[24], sym(cpu.st.gpr[24]),
                       cpu.st.gpr[27] & 0xFFFFu, cpu.st.gpr[8],
                       cpu.st.gpr[16], cpu.st.gpr[3], cpu.st.gpr[4]);
            else
                // r0 and CR0 are here because of `sc`: the selector a system
                // call asks for lives in r0, and the answer a NanoKernel stub
                // reads back is CR0[EQ] -- `li r0,-1 ; sc ; li r3,1 ; beqlr`.
                // A trace through a syscall that prints neither shows the
                // dispatcher running and cannot say what was asked or what
                // came back.
                //
                // r28/r29/r31 are here for the ATI Resource Manager's wait:
                // r31 is its device record (the card base is [r31+4] and the
                // completion fence is the POINTER at [r31+0xC4]), and r28:r29
                // is the 64-bit deadline the engine timestamp is compared
                // against. Printing the pointer and the value being compared
                // is what turns "it polls something in memory" into an
                // address -- and an address says whether the fence lives in
                // VRAM or in system RAM, which decides what has to write it.
                printf(" | r0=%08x r3=%08x r4=%08x lr=%08x cr0=%x"
                       " r28=%08x r29=%08x r31=%08x\n",
                       cpu.st.gpr[0], cpu.st.gpr[3], cpu.st.gpr[4], cpu.st.lr,
                       (cpu.st.cr >> 28) & 15u, cpu.st.gpr[28], cpu.st.gpr[29],
                       cpu.st.gpr[31]);
            if (executed + 1 == traceFrom + traceLines)
                printf("-- trace window done (%llu lines)\n",
                       static_cast<unsigned long long>(traceLines));
        }
        // A dump taken at END of run does not describe memory at an
        // earlier moment: this driver module is reloaded every kick, so an
        // end-of-run image disassembles to code at different boundaries
        // than the ones that actually executed. Dump AT the instant.
        if (dumpRamAt && executed == dumpRamAt && ramDumpPath) {
            cpu.l1dFlushAll(true);
            cpu.l2FlushAll(true);
            if (FILE* rf = fopen(ramDumpPath, "wb")) {
                fwrite(bus.ram().data(), 1, bus.ram().size(), rf);
                fclose(rf);
                printf("-- ram dumped at %llu: %s\n",
                       static_cast<unsigned long long>(executed),
                       ramDumpPath);
            }
            ramDumpPath = nullptr; // once
        }
        // Read guest memory at a VIRTUAL address, through the live MMU, at a
        // chosen instant. Structures the guest builds live in whatever
        // address space is current, so a physical dump at the same number
        // reads the RAM junk fill instead — which is how a driver's identify
        // buffer looked empty when the host had in fact pulled all 512 bytes.
        if (peekAddr && executed == peekAt) {
            printf("== peek %08x len %u @%llu\n", peekAddr, peekLen,
                   static_cast<unsigned long long>(executed));
            for (u32 row = 0; row < peekLen; row += 16) {
                printf("   +%03x:", row);
                for (u32 k = 0; k < 16 && row + k < peekLen; ++k) {
                    const long long v = guest(peekAddr + row + k, 1);
                    if (v < 0)
                        printf(" ??");
                    else
                        printf(" %02llx", v & 0xFF);
                }
                printf("\n");
            }
            fflush(stdout);
        }
        // A heartbeat, because a long run is otherwise silent until it ends
        // and "still working" is indistinguishable from "wedged". Report
        // whether the machine found any NEW code and whether it did device
        // I/O since the last beat: real progress shows one or both, a spin
        // shows neither.
        if (heartbeat && executed && (executed % heartbeat) == 0) {
            static size_t lastRegions = 0, lastCd = 0, lastHd = 0;
            // ⚠ These logs are CAPPED and can shrink, so an unsigned
            // subtraction against the previous beat printed
            // `cd 2069 (+18446744073709549937)` — a delta that reads as
            // enormous device activity at exactly the moment there was none.
            // An instrument that lies gets believed; clamp it.
            auto grew = [](size_t now, size_t before) {
                return now > before ? now - before : size_t(0);
            };
            printf("-- beat @%llu: regions %zu (+%zu) cd %zu (+%zu) "
                   "hd %zu (+%zu) pc=%08x%s\n",
                   static_cast<unsigned long long>(executed), seen.size(),
                   grew(seen.size(), lastRegions), bus.cd().log.size(),
                   grew(bus.cd().log.size(), lastCd), bus.hd().log.size(),
                   grew(bus.hd().log.size(), lastHd), cpu.st.pc, sym(cpu.st.pc));
            // 📊 EXCEPTIONS PER BEAT, BY VECTOR. A running total cannot say
            // whether a storm is happening NOW or happened once during boot,
            // and those read identically in a final report. The DELTA over a
            // fixed window is a rate, and a rate is what distinguishes "this
            // machine has always done this" from "this machine started doing
            // this". Printed as instructions-per-exception, because that is
            // the unit the diagnostic's own histogram reports in.
            static u64 lastVec[16] = {};
            printf("--   exc/beat:");
            for (u32 vi = 0; vi < 16; ++vi) {
                const u64 d = cpu.excByVec[vi] - lastVec[vi];
                lastVec[vi] = cpu.excByVec[vi];
                if (!d)
                    continue;
                printf(" %s=%llu(1/%llu)", excVecName(vi),
                       static_cast<unsigned long long>(d),
                       static_cast<unsigned long long>(heartbeat / d));
            }
            printf("\n");
            fflush(stdout);
            lastRegions = seen.size();
            lastCd = bus.cd().log.size();
            lastHd = bus.hd().log.size();
        }
        // Scan physical RAM for a 32-bit value at a chosen instant. Live
        // structures move between runs, so hunting a sentinel in a saved
        // dump finds an address that is stale by the time it is watched —
        // which is exactly how a watch on a hand-found 'nope' marker landed
        // on an unrelated word. Search the machine that is running.
        // Disassemble a live virtual range. The startup --dis reads the boot
        // ROM through the bus before the Mac OS ROM is even loaded, so every
        // look at driver or ROM-in-RAM code so far has meant dumping bytes
        // and decoding PowerPC by hand. Translate through the guest's own
        // MMU, so a VA printed by any other instrument can be pasted here
        // verbatim.
        if (disAt && executed == disAt && disEnd > disStart) {
            cpu.l1dFlushAll(true);
            cpu.l2FlushAll(true);
            const CpuState savedDis = cpu.st;
            cpu.st.msr |= 0x30u;
            const CpuState armedDis = cpu.st;
            char dtext[128];
            printf("== dis %08x..%08x @%llu\n", disStart, disEnd,
                   static_cast<unsigned long long>(executed));
            for (u32 va = disStart & ~3u; va < disEnd; va += 4) {
                // Ask the guest's own MMU FIRST, always. Shortcutting
                // FFxxxxxx to the low ROM copy disassembled a live Open
                // Firmware address as a data table and made a running
                // machine look like one that had jumped into a table —
                // the guest maps that range through the page tables once
                // MSR[IR] is on, and only the MMU knows where it points.
                u32 pa = 0;
                cpu.st = armedDis;
                const bool ok = cpu.translate(va, false, false, pa);
                cpu.st = armedDis;
                if (!ok) {
                    // Not mapped in this context. ROM-in-RAM lives in the
                    // low copy (the convention dis68k.sh and --find-code
                    // use); firmware in real mode is its own address.
                    if (va >= 0xFF000000u) {
                        pa = va & 0x00FFFFFFu;
                    } else if (va < bus.ramBytes()) {
                        pa = va;
                    } else {
                        printf("   %08x: <untranslatable>\n", va);
                        continue;
                    }
                }
                const u32 w = bus.read32(pa);
                disassemble(w, va, dtext, sizeof dtext, Style::Gnu);
                printf("   %08x: %08x  %s\n", va, w, dtext);
            }
            cpu.st = savedDis;
            cpu.raisedThisStep = false;
            fflush(stdout);
        }
        // What does the DEVICE TREE actually say? Open Firmware's tree and
        // the Name Registry copy the OS builds from it both store property
        // names as plain strings in RAM, so the question "does the display
        // node have an AAPL,address" is answerable by searching for the
        // name and reading what follows it. Every previous attempt to
        // answer it went through an interactive Forth prompt — which does
        // not exist once the display is up, because the console moves to
        // the screen and injected serial input is never consumed (proved:
        // a run with an injected script was byte-identical to one without,
        // to the instruction).
        if (findStr && executed == findAt) {
            cpu.l1dFlushAll(true);
            cpu.l2FlushAll(true);
            const std::vector<u8>& r = bus.ram();
            const size_t n = strlen(findStr);
            u32 hits = 0;
            printf("== find-str \"%s\" @%llu\n", findStr,
                   static_cast<unsigned long long>(executed));
            for (size_t p = 0; p + n <= r.size() && hits < 24; ++p) {
                if (memcmp(r.data() + p, findStr, n) != 0)
                    continue;
                ++hits;
                printf("   PA %08zx:", p);
                // 48 bytes of context after the name: for a Name Registry
                // property the value follows, and for an OF property the
                // neighbouring names say which node this is.
                for (size_t k = 0; k < 48 && p + n + k < r.size(); ++k) {
                    const u8 b = r[p + n + k];
                    printf(" %02x", b);
                }
                printf("\n     text:");
                for (size_t k = 0; k < 48 && p + n + k < r.size(); ++k) {
                    const u8 b = r[p + n + k];
                    printf("%c", (b >= 0x20 && b < 0x7F) ? char(b) : '.');
                }
                printf("\n");
            }
            if (!hits)
                printf("   <not present anywhere in RAM>\n");
            fflush(stdout);
        }
        // Where is an error code GENERATED? Locating -9356 meant hand-
        // searching a dump for `li r3,-9356` and computing the VA by hand,
        // twice. Scan for `addi rD,0,N` (li) across the loaded ROM and print
        // the virtual addresses, so a new completion code names its own
        // origin instead of staying a bare number.
        if (findCode && executed == findAt) {
            cpu.l1dFlushAll(true);
            cpu.l2FlushAll(true);
            const std::vector<u8>& r = bus.ram();
            const u32 want = 0x38000000u |
                             (static_cast<u32>(findCode) & 0xFFFFu);
            u32 hits = 0;
            printf("== find-code %d (li rD,%d) @%llu\n", findCode, findCode,
                   static_cast<unsigned long long>(executed));
            for (size_t p = 0x00C00000; p + 4 <= r.size() &&
                                        p < 0x00E00000 && hits < 30;
                 p += 4) {
                const u32 w = (u32(r[p]) << 24) | (u32(r[p + 1]) << 16) |
                              (u32(r[p + 2]) << 8) | r[p + 3];
                if ((w & 0xFC1FFFFFu) == want) {
                    ++hits;
                    printf("   VA %08llx  (li r%u,%d)\n",
                           0xFF000000ull + p, (w >> 21) & 31u, findCode);
                }
            }
            printf("   (%u site%s in the loaded ROM)\n", hits,
                   hits == 1 ? "" : "s");
            fflush(stdout);
        }
        if (findSet && executed == findAt) {
            cpu.l1dFlushAll(true);
            cpu.l2FlushAll(true);
            const std::vector<u8>& r = bus.ram();
            u32 hits = 0;
            printf("== find %08x @%llu\n", findVal,
                   static_cast<unsigned long long>(executed));
            for (size_t p = 0; p + 4 <= r.size() && hits < 40; p += 2) {
                const u32 w = (u32(r[p]) << 24) | (u32(r[p + 1]) << 16) |
                              (u32(r[p + 2]) << 8) | r[p + 3];
                if (w == findVal) {
                    ++hits;
                    printf("   PA %08zx\n", p);
                }
            }
            printf("   (%u hit%s)\n", hits, hits == 1 ? "" : "s");
            fflush(stdout);
        }
        if (dumpStructsAt && executed == dumpStructsAt)
            dumpStructs("--dump-structs-at");
        // Drive-queue change detector. We keep inferring "nothing ever
        // adds a drive"; this turns that into a positive statement.
        // DrvQHdr is 68K lowmem $308 and blue lowmem maps VA+0x4000, so
        // PA 0x4308. Sampled rather than hooked: a dirty cache line can
        // delay the observation, but the queue going non-empty is a
        // one-way event, so a late report is still a true one.
        // $308 is the QHdr's qFlags WORD; the head pointer is the long at
        // $30A and the tail at $30E. Reading a long at $308 samples
        // qFlags in the top half and only the HIGH half of qHead in the
        // bottom - so a queue whose element sits below 64 KB reads as
        // exactly zero, i.e. "still empty", forever. Watch qHead.
        if ((executed & 0x3FFFFFu) == 0) {
            static u32 lastDq = 0xFFFFFFFFu;
            const u32 dq = bus.read32(0x430Au);
            if (dq != lastDq) {
                printf("DRVQ $30A qHead = %08x (qFlags=%04x tail=%08x) "
                       "@%llu\n",
                       dq, bus.read16(0x4308u), bus.read32(0x430Eu),
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
                // r8-r15 hold D0-D7 throughout the 68K emulator,
                // including inside its opcode handlers: a handler tail
                // such as `addic. r8,r0,-56; b <dispatch>` IS setting D0.
                // Outside 0x68xxxxxx the registers belong to native code
                // and say nothing about the 68K world. (An earlier label
                // here called handler registers scratch; that was wrong.)
                printf("REGSET r%u := %08x  [%s] pc=%08x r24=%08x lr=%08x "
                       "@%llu\n",
                       watchReg, watchVal,
                       (pc & 0xFF000000u) == 0x68000000u
                           ? "68K world - r8-r15 are D0-D7"
                           : "native - not 68K registers",
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
        // A real breakpoint. Every dig so far has reached "which instruction
        // produced this?" and then stopped, because the only ways to see
        // register state were a park dump (too late) or an instruction count
        // (goes stale the moment timing shifts). Naming a PC is durable, and
        // the state that matters at an error site is almost always behind a
        // pointer, so dereference the argument registers too.
        // The same breakpoint, on the 68K side. The emulator's fetch pointer
        // is r24, so a PowerPC breakpoint cannot express "stop when the 68K
        // reaches this address" — and the interesting faults are 68K
        // exceptions, whose vector number and fault address live in a frame
        // on the 68K stack (A7 = r23), not in any PowerPC register.
        if (bp68N && executed >= bpFrom &&
            (pc & 0xFF000000u) == 0x68000000u) {
            const u32 p68 = cpu.st.gpr[24] & ~1u;
            for (u32 bi = 0; bi < bp68N; ++bi) {
                if (p68 != bp68Pc[bi] || bp68Hit[bi] >= bpMax)
                    continue;
                ++bp68Hit[bi];
                printf("== BP68%u pc68=%08x hit#%u @%llu\n", bi, p68,
                       bp68Hit[bi],
                       static_cast<unsigned long long>(executed));
                printf("   D0-D7:");
                for (u32 k = 8; k < 16; ++k)
                    printf(" %08x", cpu.st.gpr[k]);
                printf("\n   A0-A7:");
                for (u32 k = 16; k < 24; ++k)
                    printf(" %08x", cpu.st.gpr[k]);
                printf("\n");
                cpu.l1dFlushAll(true);
                cpu.l2FlushAll(true);
                const CpuState saved68 = cpu.st;
                cpu.st.msr |= 0x30u;
                const CpuState armed68 = cpu.st;
                auto peek = [&](u32 a68, u32& out) {
                    u32 pa = 0;
                    cpu.st = armed68;
                    const bool ok = cpu.translate(a68, false, false, pa);
                    cpu.st = armed68;
                    if (!ok) {
                        if (a68 >= 0x00400000u)
                            return false;
                        pa = a68 + 0x4000u; // 68K low memory
                    }
                    if (pa + 4 > bus.ram().size())
                        return false;
                    out = bus.read32(pa);
                    return true;
                };
                const u32 a7 = cpu.st.gpr[23];
                printf("   (A7)@%08x:", a7);
                for (u32 o = 0; o < 32; o += 4) {
                    u32 v = 0;
                    if (peek(a7 + o, v))
                        printf(" %08x", v);
                    else
                        printf(" --------");
                }
                printf("\n");
                cpu.st = saved68;
                cpu.raisedThisStep = false;
                fflush(stdout);
            }
        }
        // Open Firmware word trace. The coverage timeline only ever shows a
        // word the FIRST time it is entered, so a loop — which is exactly
        // what the display console does — collapses to four lines and hides
        // its own shape. Build the name table once at the gate, then log
        // every entry into a known code field in order.
        if (traceOfAt && executed == traceOfAt) {
            cpu.l1dFlushAll(true);
            cpu.l2FlushAll(true);
            const CpuState tSaved = cpu.st;
            cpu.st.msr |= 0x30u;
            const CpuState tArmed = cpu.st;
            auto rd = [&](u32 va, u8& out) -> bool {
                u32 pa = 0;
                cpu.st = tArmed;
                const bool ok = cpu.translate(va, false, false, pa);
                cpu.st = tArmed;
                if (!ok || pa >= bus.ramBytes())
                    return false;
                out = static_cast<u8>(bus.read32(pa & ~3u) >>
                                      (8 * (3 - (pa & 3u))));
                return true;
            };
            for (u32 va = 0xFF800000u; va < 0xFF930000u; va += 4) {
                for (u32 len = 1; len <= 31; ++len) {
                    const u32 hdr = va - ((1u + len + 3u) & ~3u);
                    u8 b = 0;
                    if (!rd(hdr, b) || b != len)
                        continue;
                    std::string s;
                    bool ok = true;
                    for (u32 k = 0; k < len && ok; ++k) {
                        if (!rd(hdr + 1u + k, b) || b < 0x21 || b > 0x7E)
                            ok = false;
                        else
                            s.push_back(static_cast<char>(b));
                    }
                    for (u32 k = 1u + len;
                         ok && k < ((1u + len + 3u) & ~3u); ++k)
                        if (!rd(hdr + k, b) || b != 0)
                            ok = false;
                    if (ok) {
                        ofNames[va] = s;
                        // Some headers carry a zero cell between the padded
                        // name and the code field (probe-slots is one), so
                        // the word is entered at va+4 and a trace keyed on
                        // va alone never sees it run.
                        u8 z0 = 0, z1 = 0, z2 = 0, z3 = 0;
                        if (rd(va, z0) && rd(va + 1, z1) && rd(va + 2, z2) &&
                            rd(va + 3, z3) && !z0 && !z1 && !z2 && !z3)
                            ofNames[va + 4] = s;
                        break;
                    }
                }
            }
            cpu.st = tSaved;
            cpu.raisedThisStep = false;
            printf("-- of trace armed @%llu: %zu named words\n",
                   static_cast<unsigned long long>(executed), ofNames.size());
            for (const std::string& needle : ofFind) {
                u32 hits = 0;
                for (const auto& nv : ofNames)
                    if (nv.second.find(needle) != std::string::npos) {
                        printf("-- of-find \"%s\": %08x %s\n", needle.c_str(),
                               nv.first, nv.second.c_str());
                        ++hits;
                    }
                if (!hits)
                    printf("-- of-find \"%s\": no match\n", needle.c_str());
            }
            // "Which word writes this value?" is the question a Forth image
            // answers worst. Scan the loaded dictionary for the literal and
            // name the definition each hit falls inside.
            for (u32 ref : ofRefs) {
                printf("-- of-refs %08x %s:\n", ref,
                       ofNames.count(ref) ? ofNames[ref].c_str() : "?");
                u32 hits = 0;
                std::string owner = "<before first word>";
                for (u32 va = 0xFF800000u; va < 0xFF930000u; va += 4) {
                    const auto nit = ofNames.find(va);
                    if (nit != ofNames.end())
                        owner = nit->second;
                    u8 b0 = 0, b1 = 0, b2 = 0, b3 = 0;
                    if (!rd(va, b0) || !rd(va + 1, b1) || !rd(va + 2, b2) ||
                        !rd(va + 3, b3))
                        continue;
                    const u32 w = (u32(b0) << 24) | (u32(b1) << 16) |
                                  (u32(b2) << 8) | b3;
                    if (w != ref)
                        continue;
                    printf("   %08x  in %s\n", va, owner.c_str());
                    if (++hits >= 64)
                        break;
                }
                if (!hits)
                    printf("   <no literal reference>\n");
            }
            // The complement of of-refs: subroutine threading means "who
            // calls this word" is a scan for `bl <va>`, and that is the only
            // way to walk the startup sequence upward from a leaf.
            for (u32 tgt : ofCallers) {
                printf("-- of-callers %08x %s:\n", tgt,
                       ofNames.count(tgt) ? ofNames[tgt].c_str() : "?");
                u32 hits = 0;
                std::string owner = "<before first word>";
                for (u32 va = 0xFF800000u; va < 0xFF930000u; va += 4) {
                    const auto nit = ofNames.find(va);
                    if (nit != ofNames.end())
                        owner = nit->second;
                    u8 b0 = 0, b1 = 0, b2 = 0, b3 = 0;
                    if (!rd(va, b0) || !rd(va + 1, b1) || !rd(va + 2, b2) ||
                        !rd(va + 3, b3))
                        continue;
                    const u32 w = (u32(b0) << 24) | (u32(b1) << 16) |
                                  (u32(b2) << 8) | b3;
                    if ((w >> 26) != 18u)
                        continue;
                    i32 li = static_cast<i32>(w & 0x03FFFFFCu);
                    if (li & 0x02000000)
                        li |= static_cast<i32>(0xFC000000u);
                    const u32 dst = (w & 2u) ? static_cast<u32>(li)
                                             : va + static_cast<u32>(li);
                    if (dst != tgt)
                        continue;
                    printf("   %08x  %s  in %s\n", va,
                           (w & 1u) ? "bl" : "b ", owner.c_str());
                    if (++hits >= 64)
                        break;
                }
                if (!hits)
                    printf("   <no caller>\n");
            }
            // Apple's Open Firmware is subroutine-threaded: a colon
            // definition compiles to a run of `bl <word>`. With the name
            // table in hand that is decompilable, so a word can be read
            // instead of inferred from which of its callees happened to
            // show up in a trace.
            for (u32 seeVa : ofSee) {
                printf("-- of-see %08x %s:\n", seeVa,
                       ofNames.count(seeVa) ? ofNames[seeVa].c_str() : "?");
                char dtext[128];
                for (u32 k = 0; k < ofSeeN; ++k) {
                    const u32 va = seeVa + 4u * k;
                    u32 pa = 0;
                    cpu.st = tArmed;
                    const bool ok = cpu.translate(va, false, false, pa);
                    cpu.st = tArmed;
                    if (!ok || pa >= bus.ramBytes()) {
                        printf("   %08x: <untranslatable>\n", va);
                        break;
                    }
                    const u32 w = bus.read32(pa);
                    disassemble(w, va, dtext, sizeof dtext, Style::Gnu);
                    std::string ann;
                    if ((w >> 26) == 18u) { // b / bl / ba / bla
                        i32 li = static_cast<i32>(w & 0x03FFFFFCu);
                        if (li & 0x02000000)
                            li |= static_cast<i32>(0xFC000000u);
                        const u32 tgt = (w & 2u)
                                            ? static_cast<u32>(li)
                                            : va + static_cast<u32>(li);
                        const auto nit = ofNames.find(tgt);
                        if (nit != ofNames.end())
                            ann = "   <" + nit->second + ">";
                    }
                    printf("   %08x: %08x  %s%s\n", va, w, dtext,
                           ann.c_str());
                    if (w == 0x4E800020u) // blr ends the definition
                        break;
                }
            }
            fflush(stdout);
        }
        // The name table is built over ff800000..ff930000, but this gate used
        // to mask to ff8xxxxx and so dropped every word above ff900000 —
        // which is where the whole PCI-node method set lives (probe-slots,
        // ?probe-slot, pci-probe-mask). Match the table, not a prefix.
        if (!ofNames.empty() && executed >= traceOfAt &&
            pc >= 0xFF800000u && pc < 0xFF930000u) {
            const auto it = ofNames.find(pc);
            if (it != ofNames.end()) {
                const bool keep =
                    ofOnly.empty() ||
                    [&] {
                        for (const std::string& s : ofOnly)
                            if (it->second.find(s) != std::string::npos)
                                return true;
                        return false;
                    }();
                if (traceOfLeft && keep) {
                    --traceOfLeft;
                    printf("OF %08x %s @%llu\n", pc, it->second.c_str(),
                           static_cast<unsigned long long>(executed));
                }
                if (ofHistTo && executed >= ofHistFrom &&
                    executed < ofHistTo) {
                    // Key on the address: this ROM carries three copies of
                    // probe-slots and pci-probe-mask, one per PCI bus node,
                    // and collapsing them by name hides which bus ran.
                    char key[64];
                    snprintf(key, sizeof key, "%08x %s", pc,
                             it->second.c_str());
                    if (ofHist[key]++ == 0)
                        ofHistFirst.push_back({executed, key});
                }
            }
        }
        if (bpN && executed >= bpFrom) {
            for (u32 bi = 0; bi < bpN; ++bi) {
                if (pc != bpPc[bi] || bpHit[bi] >= bpMax)
                    continue;
                ++bpHit[bi];
                printf("== BP%u %08x hit#%u @%llu lr=%08x ctr=%08x cr=%08x\n",
                       bi, pc, bpHit[bi],
                       static_cast<unsigned long long>(executed),
                       cpu.st.lr, cpu.st.ctr, cpu.st.cr);
                for (u32 g = 0; g < 32; g += 8) {
                    printf("   r%-2u:", g);
                    for (u32 k = 0; k < 8; ++k)
                        printf(" %08x", cpu.st.gpr[g + k]);
                    printf("\n");
                }
                // Flush first. Dereferencing straight off the bus reads
                // AROUND the caches, so a pointer the guest has just
                // written still shows its old value — and a stale TVector
                // reads exactly like a corrupt one.
                cpu.l1dFlushAll(true);
                cpu.l2FlushAll(true);
                const CpuState savedBp = cpu.st;
                cpu.st.msr |= 0x30u;
                const CpuState armedBp = cpu.st;
                // r3-r12: the PowerOpen argument registers plus r11/r12, which
                // carry the TVector on every cross-fragment call — the
                // callee address lives at *r12, so a breakpoint that cannot
                // see r12 cannot name what is about to be called.
                // r3-r31: the argument registers, plus Open Firmware's Forth
                // stacks — r30 is the return stack and the data stack sits
                // in the high registers, so a breakpoint that stops at r12
                // cannot read the arguments of any OF word.
                for (u32 g = 3; g <= 31; ++g) {
                    const u32 ea = cpu.st.gpr[g];
                    // Open Firmware lives above 0xF0000000, so an upper bound
                    // there silently skipped every register worth reading.
                    // Let translation decide what is dereferenceable.
                    if (ea < 0x1000u)
                        continue;
                    u32 pa = 0;
                    cpu.st = armedBp;
                    const bool ok = cpu.translate(ea, false, false, pa);
                    cpu.st = armedBp;
                    if (!ok)
                        continue;
                    printf("   *r%u %08x (pa %08x):", g, ea, pa);
                    for (u32 o = 0; o < bpDeref; o += 4) {
                        if ((o & 31u) == 0 && o)
                            printf("\n            +%04x:", o);
                        printf(" %08x", bus.read32(pa + o));
                    }
                    printf("\n");
                }
                cpu.st = savedBp;
                cpu.raisedThisStep = false;
                fflush(stdout);
            }
        }
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
                {
                    const int fnb = rb(0x12) & 0xFF;
                    const auto& cl2 = bus.cd().log;
                    const auto& hl2 = bus.hd().log;
                    // Tie the manager's complaint to the WIRE: which ATA
                    // command each cell last saw and how many bytes moved.
                    // A manager error and a device that never transferred
                    // are the same event seen from two ends.
                    printf("ATARES pb=%08x result=%d (0x%04x)%s fn=%02x %s "
                           "lastCD=%c%02x[%uB] lastHD=%c%02x[%uB] @%llu\n",
                           rpb, res, res & 0xFFFF, ataErrName(res), fnb,
                           ataFnName(static_cast<u32>(fnb)),
                           cl2.empty() ? '-' : cl2.back().kind,
                           cl2.empty() ? 0 : cl2.back().val,
                           cl2.empty() ? 0u : cl2.back().xfer,
                           hl2.empty() ? '-' : hl2.back().kind,
                           hl2.empty() ? 0 : hl2.back().val,
                           hl2.empty() ? 0u : hl2.back().xfer,
                           static_cast<unsigned long long>(executed));
                }
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
            // Sampling on "r24 changed" has now produced three wrong
            // readings, because r24 is a FETCH pointer running ahead of the
            // instruction being executed and r27 lags it by a prefetch. The
            // one moment the pair is consistent is the emulator's own
            // opcode fetch: `lhau r27, 2(r24)` (0xaf780002) pre-increments
            // r24 and then loads the opcode AT it, so immediately after
            // that instruction r24 IS the address of the opcode in r27.
            // Trigger there and the trace is authoritative by construction.
            const bool atFetch = cpu.curInsn == 0xAF780002u;
            // A GLOBAL A-trap census, keyed to the same authoritative fetch.
            // The existing trap watch only covers .ATALoad's own body, so
            // "_DrvrInstall is never called" was only ever a statement about
            // that one driver. Whether ANY code in the entire boot installs
            // a unit-table entry is a different question, and the one that
            // matters now that an empty unit 61 is the known cause.
            if (atFetch && (cpu.st.gpr[27] & 0xF000u) == 0xA000u) {
                const u32 t = cpu.st.gpr[27] & 0x0FFFu;
                ++trapAll[t];
                // --trap-log: name the FILE, not just the trap. A File
                // Manager parameter block is in A0 (r16) and ioNamePtr is at
                // PB+18; a plausible Pascal string there is the answer to
                // "which file". ⚠ guest() flushes the caches write-back, so
                // this is an instrument that PERTURBS — hence hard-gated by
                // --bp-from and capped. Data is preserved (write-back, not
                // invalidate), timing and cache state are not.
                if (trapLogN && trapLogged < trapLogN && executed >= bpFrom) {
                    ++trapLogged;
                    const u32 a0 = cpu.st.gpr[16];
                    char nm[40] = {};
                    const long long np = guest(a0 + 18u, 4);
                    if (np > 0) {
                        const long long ln = guest(static_cast<u32>(np), 1);
                        if (ln > 0 && ln < 32) {
                            for (long long k = 0; k < ln; ++k) {
                                const long long c =
                                    guest(static_cast<u32>(np) + 1u +
                                              static_cast<u32>(k), 1);
                                nm[k] = (c >= 32 && c < 127)
                                            ? static_cast<char>(c)
                                            : '.';
                            }
                        }
                    }
                    printf("TRAP $A%03x %-18s pc68=%08x D0=%08x A0=%08x%s%s%s "
                           "@%llu\n",
                           t, trapName(t), cur68, cpu.st.gpr[8], a0,
                           nm[0] ? " name=\"" : "", nm, nm[0] ? "\"" : "",
                           static_cast<unsigned long long>(executed));
                    fflush(stdout);
                }
                if (t == 0x03Du || t == 0x04Eu) { // _DrvrInstall / _AddDrive
                    static int di = 0;
                    if (di < 40) {
                        ++di;
                        printf("INSTALL $A%03x %s pc68=%08x%s D0=%08x "
                               "A0=%08x @%llu\n",
                               t, trapName(t), cur68, sym(cur68),
                               cpu.st.gpr[8], cpu.st.gpr[16],
                               static_cast<unsigned long long>(executed));
                        fflush(stdout);
                    }
                }
            }
            if (t68Lo && atFetch && cur68 >= t68Lo && cur68 < t68Hi &&
                executed >= watchFrom) {
                static u32 t68n = 0;
                if (t68n < t68Cap) {
                    ++t68n;
                    const bool isRts = (cpu.st.gpr[27] & 0xFFFFu) == 0x4E75u;
                    printf("%s %08x%s op=%04x D0=%08x D6=%08x D7=%08x "
                           "A0=%08x A3=%08x A4=%08x\n",
                           isRts ? "RET" : "T68", cur68, sym(cur68),
                           cpu.st.gpr[27] & 0xFFFFu, cpu.st.gpr[8],
                           cpu.st.gpr[14], cpu.st.gpr[15], cpu.st.gpr[16],
                           cpu.st.gpr[19], cpu.st.gpr[20]);
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
            // The .ATALoad trace's own time gate. It was the literal
            // 2,800,000,000 below, and the ATA Manager's whole device pass
            // now runs at 2.4886 B — so the trace covered only the repeating
            // steady state AFTER the decision, and two runs that served
            // DIFFERENT SECTORS produced byte-identical traces. That reads
            // exactly like "the data made no difference", which is the most
            // expensive kind of wrong answer. Same failure the file warns
            // about three times over; the constant is now a knob.
            if (cur68 != prev68 &&
                (armAtPark ? parkArmed : executed > drvFrom)) {
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
        if (typeText && executed == typeAt) {
            // Keystrokes over USB, which is where Open Firmware actually
            // listens once a display exists.
            std::string t(typeText);
            for (char& c : t)
                if (c == ';')
                    c = '\r';
            bus.ohci(0).typeAscii(t + "\r");
            bus.deviceStateChanged(); // poked from outside: reopen the gate
            printf("-- typed on usb @%llu: %s\n",
                   static_cast<unsigned long long>(executed), typeText);
            fflush(stdout);
        }
        if (pokeSet && executed >= pokeAt &&
            (executed == pokeAt ||
             (pokeEvery && (executed - pokeAt) % pokeEvery == 0))) {
            // Through the DMA-write snoop so the processor cannot serve a stale
            // cached copy of the cell we are forcing.
            for (u32 i = 0; i < pokeCount; ++i) {
                const u32 pa = pokePa + i * pokeStride;
                bus.snoopBeforeDmaWrite(pa, 4);
                bus.write32(pa, pokeVal);
            }
            static u32 pokeN = 0;
            if (++pokeN <= 3)
                printf("-- poke: [%08x] := %08x @%llu\n", pokePa, pokeVal,
                       static_cast<unsigned long long>(executed));
            fflush(stdout);
        }
        if (crsrDrag && executed >= crsrDrag &&
            (executed - crsrDrag) % 1000000ull == 0) {
            // A Mac Point is {v,h} -- VERTICAL first -- so the word is
            // (y << 16) | x. Low memory lives at PA = logical + 0x4000.
            static u32 dragStep = 0;
            ++dragStep;
            const u32 x = 15 + dragStep * 12, y = 15 + dragStep * 6;
            const u32 pt = ((y & 0xFFFFu) << 16) | (x & 0xFFFFu);
            // Through the DMA-write snoop, so the processor's own cached copy
            // of low memory is invalidated rather than left stale.
            for (u32 pa : {0x4828u, 0x482Cu, 0x4830u}) {
                bus.snoopBeforeDmaWrite(pa, 4);
                bus.write32(pa, pt);
            }
            bus.snoopBeforeDmaWrite(0x48CEu, 1);
            bus.write8(0x48CEu, bus.read8(0x48CFu)); // CrsrNew = CrsrCouple
            if (dragStep <= 4 || (dragStep % 25) == 0)
                printf("-- crsr-drag: Mouse := (%u,%u) step %u @%llu\n", x, y,
                       dragStep, static_cast<unsigned long long>(executed));
            fflush(stdout);
        }
        if (cmdText && executed == cmdAt) {
            // Command held for each keystroke -- no trailing Return, because
            // Command-O IS the action.
            bus.ohci(0).typeChord(0x08, std::string(cmdText));
            bus.deviceStateChanged(); // poked from outside: reopen the gate
            printf("-- typed with COMMAND on usb @%llu: %s\n",
                   static_cast<unsigned long long>(executed), cmdText);
            fflush(stdout);
        }
        while (tbIdx < typeBeats.size() && executed == typeBeats[tbIdx].at) {
            // NO trailing Return, deliberately (unlike --type, whose \r is
            // for the Open Firmware console): in the Finder a Return on a
            // selected icon starts a RENAME, not an open. Type-select is
            // the text alone; opening is Cmd-O's job (--cmd-beat).
            bus.ohci(0).typeAscii(typeBeats[tbIdx].text);
            bus.deviceStateChanged(); // poked from outside: reopen the gate
            printf("-- type-beat @%llu: %s\n",
                   static_cast<unsigned long long>(executed),
                   typeBeats[tbIdx].text.c_str());
            fflush(stdout);
            ++tbIdx;
        }
        while (cbIdx < cmdBeats.size() && executed == cmdBeats[cbIdx].at) {
            bus.ohci(0).typeChord(0x08, cmdBeats[cbIdx].text);
            bus.deviceStateChanged(); // poked from outside: reopen the gate
            printf("-- cmd-beat @%llu: %s\n",
                   static_cast<unsigned long long>(executed),
                   cmdBeats[cbIdx].text.c_str());
            fflush(stdout);
            ++cbIdx;
        }
        // Mouse motion, headless. "The pointer does not move" has two very
        // different causes -- the shell not delivering events, and the device
        // never being polled -- and only injecting motion with no GUI in the
        // way can tell them apart. Repeats so the guest sees sustained travel
        // rather than one report it may legitimately coalesce away.
        // --mouse-every controls the cadence. One report every 2 M
        // instructions is ~150 polls apart, so the guest sees isolated 8-pixel
        // jumps; a driver that coalesces motion, or requires travel across
        // consecutive polls, would discard every one of them. Tightening this
        // to a few frames tells "motion is dropped" from "motion is not
        // sustained enough to be believed".
        if (mouseAt && executed >= mouseAt &&
            (executed - mouseAt) % mouseEvery == 0) {
            int dx = mouseDx, dy = mouseDy;
            if (mouseVary) { // never repeat a report byte-for-byte
                dx += static_cast<int>(mouseSent % 5u);
                dy += static_cast<int>(mouseSent % 3u);
            }
            bus.ohci(1).moveMouse(dx, dy, heldButtons);
            bus.deviceStateChanged(); // poked from outside: reopen the gate
            ++mouseSent;
        }
        // A CLICK, which reaches the OS by a different route than motion:
        // the Cursor Device Manager takes button transitions through
        // CrsrDevButtons and movement through CrsrDevMoveDelta. "Motion is
        // dropped but a click lands" and "nothing lands at all" are separate
        // bugs, and only injecting a button can separate them. Down, then up
        // 200k instructions later, so the OS sees a complete transition.
        if (clickAt && executed == clickAt) {
            heldButtons = 1; // stays pressed across any motion in between
            bus.ohci(1).moveMouse(0, 0, heldButtons);
            bus.deviceStateChanged(); // poked from outside: reopen the gate
            printf("-- mouse button DOWN @%llu\n",
                   static_cast<unsigned long long>(executed));
            fflush(stdout);
        }
        if (clickAt && executed == clickAt + clickHoldFor) {
            heldButtons = 0;
            bus.ohci(1).moveMouse(0, 0, heldButtons);
            bus.deviceStateChanged(); // poked from outside: reopen the gate
            printf("-- mouse button UP @%llu\n",
                   static_cast<unsigned long long>(executed));
            fflush(stdout);
        }
        if (serialInput && executed == serialAt) {
            std::string s(serialInput);
            for (char& c : s)
                if (c == ';')
                    c = '\r';
            bus.injectSerial(s + "\r");
            bus.deviceStateChanged(); // poked from outside: reopen the gate
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
            // DIAGNOSTIC (opt-in, not machine truth). USBShim chain-calls
            // the previous boot-keyboard proc from [ExpandMem+0x294] with
            // no null check, and the seed it expects comes from the USB
            // Expert's per-controller shim reference - absent while this
            // machine's OHCI controllers are never probed or assigned a
            // BAR. Calling through the zero lands the 68K at address 0,
            // which raises a Line-F: `[$0AF0] = 0x0A = dsLineFErr`, sad
            // Mac, `bra.s *`. That is the SAME wall session 5 found and it
            // is back because the recipe no longer runs a Forth script.
            //
            // The old poke keyed on the shim's entry pc (ffe2325c), which
            // belongs to the CD's Mac OS ROM and never executes on this
            // one. Key on the CELL instead: if it is ever cleared to zero
            // in the OS era, put a bare ROM RTS back. Real fix = USB.
            if (em294Rts && cv == 0 && emPrev != 0) {
                // ffc002d6 is `moveq #0,d0 / rts` in THIS Mac OS ROM,
                // located by scanning the loaded image for 70 00 4e 75 on
                // a word boundary. The old poke used ffc339a2, which is an
                // RTS in the CD's ROM and something else in this one - and
                // that announced itself honestly: the sad-Mac code moved
                // from 0A (Line-F, calling through zero) to 03 (illegal
                // instruction, calling into data).
                bus.write32(0x000116C4u, 0xFFC002D6u);
                cpu.dcbKill(0x000116C4u & ~31u);
                printf("-- DIAGNOSTIC poke: [EM+294] was cleared @%llu, "
                       "reseeded with a return-zero stub (--em294-rts)\n",
                       static_cast<unsigned long long>(executed));
                cv = 0xFFC002D6u;
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
            // 0x60fe is BRA.S *-2: a 68K branch to itself, i.e. a
            // deliberate halt. The system reaching one is the single most
            // informative event in an OS-era boot and there was no
            // instrument for it - the 300M-interval spin sampler reported
            // the halt as much as a third of a billion instructions late,
            // by which time the fetch ring held only the halt itself.
            // Report the FIRST few, each with the ring that led into it.
            // NOT simply "op == 60fe": r24 is a FETCH pointer, so the word
            // after any short branch is crossed before the branch applies.
            // A `bra.s +$24` two bytes earlier makes the 60fe at the next
            // address look executed when it never was — that false positive
            // fired six times at ffc0ab16 before this test existed. A real
            // self-branch shows up as the fetch pointer ALTERNATING between
            // X and X+2 and never leaving, so require the pattern twice.
            static u32 haltShown = 0;
            bool selfLoop = false;
            if ((cpu.st.gpr[27] & 0xFFFFu) == 0x60FEu && ring68At >= 5) {
                const u32 a1 = ring68[(ring68At - 1u) & 127u].pc68;
                const u32 a2 = ring68[(ring68At - 2u) & 127u].pc68;
                const u32 a3 = ring68[(ring68At - 3u) & 127u].pc68;
                const u32 a4 = ring68[(ring68At - 4u) & 127u].pc68;
                selfLoop = a1 == a3 && a2 == a4 && a2 == a1 + 2u;
            }
            if (selfLoop && haltShown < 6 &&
                (pc & 0xFFC00000u) == 0x68000000u) {
                ++haltShown;
                const char* hs = sym(prev68k);
                printf("-- 68K HALT (bra.s *) #%u @%llu pc68=%08x%s%s%s\n",
                       haltShown,
                       static_cast<unsigned long long>(executed), prev68k,
                       hs ? " <" : "", hs ? hs : "", hs ? ">" : "");
                // The sad-Mac handler reads the system-error code out of
                // 68K lowmem $0AF0 into D6 on its way here, so the 68K
                // data registers ARE the diagnosis: D6 = 1 is dsBusError,
                // 3 dsIllInstErr, 10 dsLineFErr, and so on. Printing only
                // the fetch ring meant reading the code required a second
                // run with a register sampler that fired every 300 M.
                printf("   D0-D7:");
                for (u32 k = 8; k < 16; ++k)
                    printf(" %08x", cpu.st.gpr[k]);
                printf("\n   A0-A7:");
                for (u32 k = 16; k < 24; ++k)
                    printf(" %08x", cpu.st.gpr[k]);
                printf("\n   how it got here (last 24 fetches):\n");
                for (u32 k = 0; k < 24; k += 4) {
                    printf("   ");
                    for (u32 j = 0; j < 4; ++j) {
                        const Ent68& e =
                            ring68[(ring68At + 128u - 25u + k + j) & 127u];
                        const char* s = sym(e.pc68);
                        printf(" %08x/%04x%s%s%s", e.pc68, e.op & 0xFFFFu,
                               s ? "<" : "", s ? s : "", s ? ">" : "");
                    }
                    printf("\n");
                }
                fflush(stdout);
            }
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
            // The registers alone never said WHERE. r24 is the emulator's
            // 68K fetch pointer and the ring carries the last 128 fetch
            // positions, so the loop can be named instead of guessed at —
            // and with the MacsBug table loaded, named literally.
            printf("   68k fetch ring (oldest first, pc/op):\n");
            for (u32 k = 0; k < 32; k += 4) {
                printf("   ");
                for (u32 j = 0; j < 4; ++j) {
                    const Ent68& e =
                        ring68[(ring68At + 128u - 32u + k + j) & 127u];
                    const char* s = sym(e.pc68);
                    printf(" %08x/%04x%s%s", e.pc68, e.op & 0xFFFFu,
                           s ? "<" : "", s ? s : "");
                    if (s)
                        printf(">");
                }
                printf("\n");
            }
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
            // The decrementer fires ~170,000 times in an OS-era boot, so a
            // 16-deep ring holds nothing but timer ticks and the FAULT that
            // ended the boot is never in it. Report the first few of every
            // OTHER kind, once each: a DSI on a poisoned pointer is the
            // event worth seeing, and it was being flushed out by clock
            // interrupts every time.
            static u32 faultShown = 0;
            const u32 v = cpu.st.pc & 0x0000FF00u;
            // Open Firmware probes the PCI buses by faulting on purpose,
            // so the first dozen non-timer exceptions of any boot are its
            // master aborts and the OS-era fault never gets a slot.
            // --fault-from moves the window past the firmware.
            // Also skip the 68K emulator's own `twi` ladder and the OS's
            // system calls: program exceptions with SRR1[TRAP] set are the
            // nanokernel's SERVICE-CALL mechanism, not faults, and sixteen
            // of them filled this report every time and hid the real one.
            const bool kernelTrap =
                v == 0x0700u && (cpu.st.srr1 & 0x00020000u);
            if (v != 0x0900u && v != 0x0500u && v != 0x0C00u &&
                !kernelTrap && faultShown < 16 && executed >= faultFrom) {
                ++faultShown;
                printf("-- FAULT #%u @%llu -> %08x srr0=%08x srr1=%08x "
                       "dsisr=%08x dar=%08x lr=%08x r24=%08x\n",
                       faultShown,
                       static_cast<unsigned long long>(executed), cpu.st.pc,
                       cpu.st.srr0, cpu.st.srr1, cpu.st.dsisr, cpu.st.dar,
                       cpu.st.lr, cpu.st.gpr[24]);
                fflush(stdout);
            }
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
    // Same rule as --wp-va: a watch that was asked for but never armed must say
    // so, or its silence gets read as a measurement.
    if (rpVa && !rpVaPa)
        printf("-- rp-va %08x..%08x NEVER RESOLVED: the address did not "
               "translate at any point in this run. THIS IS NOT A NULL "
               "RESULT — the watch was never armed.\n",
               rpVa, rpVaEnd);
    if (cpu.rpEnd) {
        // Who READS this range. "0 readers" and "read by one site a million
        // times" are opposite diagnoses for a buffer the machine fills, and no
        // store watch can tell them apart.
        printf("-- read watch %08x..%08x: %llu reads by %zu distinct pc%s\n",
               cpu.rpPa, cpu.rpEnd,
               static_cast<unsigned long long>(cpu.rpHits), cpu.rpByPc.size(),
               cpu.rpByPc.size() == 1 ? "" : "s");
        for (const auto& [at, n] : cpu.rpByPc)
            printf("   pc=%08x  x%llu\n", at,
                   static_cast<unsigned long long>(n));
        // And by CALLER. A shared copy primitive's pc names nothing -- the same
        // reason --wp had to start recording the LR.
        printf("   -- by caller (LR):\n");
        for (const auto& [at, n] : cpu.rpByLr)
            printf("   lr=%08x  x%llu\n", at,
                   static_cast<unsigned long long>(n));
    }
    // Accesses that went round the caches while the L2 still held the block.
    // Printed always, including the zero, because "the L2 is enabled and
    // nothing reads round it" is the reading that clears the whole class.
    printf("-- l2 skew (bypassing access over a live L2 line): %llu reads, "
           "%llu writes by %zu pc%s\n",
           static_cast<unsigned long long>(cpu.l2SkewR),
           static_cast<unsigned long long>(cpu.l2SkewW), cpu.l2SkewByPc.size(),
           cpu.l2SkewByPc.size() == 1 ? "" : "s");
    for (const auto& [at, n] : cpu.l2SkewByPc)
        printf("   pc=%08x  x%llu\n", at, static_cast<unsigned long long>(n));
    // ⚠ Printed when a watch was ASKED FOR, not only when one was armed. A
    // --wp-va whose address never translated leaves cpu.wpEnd at zero, and
    // reporting nothing in that case is the instrument saying "no stores"
    // when what happened is "never looked".
    if (callogPc) {
        // Printed even at zero, because "nothing ever calls it" and "the
        // address was wrong" are different findings and silence is neither.
        printf("-- callog %08x%s: %llu arrival%s from %zu caller%s\n", callogPc,
               sym(callogPc), static_cast<unsigned long long>(callogHits),
               callogHits == 1 ? "" : "s", callogByLr.size(),
               callogByLr.size() == 1 ? "" : "s");
        for (const auto& [at, n] : callogByLr)
            printf("   lr=%08x%s  x%llu\n", at, sym(at),
                   static_cast<unsigned long long>(n));
        for (const CallogArgs& c : callogLog)
            printf("   @%-12llu lr=%08x r3=%08x r4=%08x r5=%08x r6=%08x "
                   "r7=%08x r8=%08x\n",
                   static_cast<unsigned long long>(c.at), c.lr, c.r3, c.r4,
                   c.r5, c.r6, c.r7, c.r8);
        if (callogLog.size() >= callogMax)
            printf("   (argument list capped at %u — raise with --callog-max)\n",
                   callogMax);
    }
    if (wpVa && !wpVaPa)
        printf("-- wp-va %08x..%08x NEVER RESOLVED: the address did not "
               "translate at any point in this run. THIS IS NOT A NULL "
               "RESULT — the watch was never armed.\n",
               wpVa, wpVaEnd);
    if (cpu.wpEnd) {
        printf("-- watchpoint %08x..%08x: %llu store%s (%zu logged%s)\n",
               cpu.wpPa, cpu.wpEnd,
               static_cast<unsigned long long>(cpu.wpHits),
               cpu.wpHits == 1 ? "" : "s", cpu.wpLog.size(),
               cpu.wpLog.size() >= cpu.wpMax ? ", CAPPED" : "");
        if (wpVaPa)
            printf("   VA %08x, last aimed @%llu; the guest remapped it %u "
                   "time%s%s\n",
                   wpVa, static_cast<unsigned long long>(wpVaAt), wpVaMoves,
                   wpVaMoves == 1 ? "" : "s",
                   wpVaMoves ? " — PAs below span more than one mapping"
                             : "");
        // The census, which is what makes a range watch answer a question the
        // capped log cannot: WHICH bytes of the range were ever written. A
        // field under test and a neighbour known to change go in the same
        // watch, and the neighbour's row is the positive control for the
        // field's absence — in the same run, on the same armed instrument.
        if (!cpu.wpByPa.empty()) {
            printf("   written, by address (%zu distinct):\n",
                   cpu.wpByPa.size());
            for (const auto& [at, s] : cpu.wpByPa)
                if (wpVaPa && !wpVaMoves)
                    printf("     pa %08x +%-2u  (va %08x, +%-4d)  x%llu\n", at,
                           s.len, wpVa + (at - wpVaPa),
                           static_cast<int>(at - cpu.wpPa),
                           static_cast<unsigned long long>(s.n));
                else
                    printf("     pa %08x +%-2u  (+%-4d)  x%llu\n", at, s.len,
                           static_cast<int>(at - cpu.wpPa),
                           static_cast<unsigned long long>(s.n));
            // The gaps ARE the finding. Naming the bytes nothing wrote is the
            // difference between "the watch saw nothing" and "the watch saw
            // this record change and never this field".
            std::string gaps;
            u32 run0 = 0;
            bool inGap = false;
            for (u32 p = cpu.wpPa; p <= cpu.wpEnd; ++p) {
                bool hit = false;
                for (const auto& [at, s] : cpu.wpByPa)
                    if (p >= at && p < at + s.len) { hit = true; break; }
                if (!hit && !inGap) { run0 = p; inGap = true; }
                if ((hit || p == cpu.wpEnd) && inGap) {
                    const u32 last = hit ? p - 1 : p;
                    gaps += " +" + std::to_string(run0 - cpu.wpPa) + ".." +
                            std::to_string(last - cpu.wpPa);
                    inGap = false;
                }
            }
            printf("   NEVER written, by offset from %08x:%s\n", cpu.wpPa,
                   gaps.empty() ? " (none — every byte was written)"
                                : gaps.c_str());
        }
        if (!cpu.wpByPc.empty()) {
            printf("   by storing pc (%zu distinct):\n", cpu.wpByPc.size());
            for (const auto& [at, n] : cpu.wpByPc)
                printf("     pc=%08x%s  x%llu\n", at, sym(at),
                       static_cast<unsigned long long>(n));
        }
        // Name the CALLER, not the primitive. With --trace-of the dictionary
        // is loaded, so the LR resolves to the word the store was compiled
        // into; without it the bare address still beats naming nothing.
        auto ofOwner = [&](u32 va) -> std::string {
            if (ofNames.empty()) return std::string();
            auto it = ofNames.upper_bound(va);
            if (it == ofNames.begin()) return std::string();
            --it;
            return " <" + it->second + "+" + std::to_string(va - it->first) +
                   ">";
        };
        // dtb is the gap since the previous store to this range. "Who wrote
        // it" is only half the question for a periodic global; the other half
        // is HOW OFTEN, and reading a rate off a column of absolute timebases
        // by hand is how a 44x error in the guest's tick chain stayed a vague
        // impression for three sessions. r68 is r24, the 68K pc — see
        // Cpu::WpHit for why a pc of 0x68xxxxxx names nothing on its own.
        u64 prevTb = 0, prevDec = 0, prevExt = 0;
        for (const Cpu::WpHit& h : cpu.wpLog) {
            printf("   pa %08x <- %0*x  by pc=%08x lr=%08x%s r68=%08x "
                   "@%llu tb=%llu dtb=%llu ddec=%lld dext=%lld\n",
                   h.pa, static_cast<int>(h.len * 2), h.val, h.pc, h.lr,
                   ofOwner(h.lr).c_str(), h.r24,
                   static_cast<unsigned long long>(h.at),
                   static_cast<unsigned long long>(h.tb),
                   static_cast<unsigned long long>(
                       prevTb && h.tb > prevTb ? h.tb - prevTb : 0),
                   static_cast<long long>(prevTb ? h.dec - prevDec : 0),
                   static_cast<long long>(prevTb ? h.ext - prevExt : 0));
            prevTb = h.tb;
            prevDec = h.dec;
            prevExt = h.ext;
        }
    }
    // The MMU as the guest left it. A DSI names an address but not WHY it
    // failed, and "the BAR is routed on the bus" and "the guest can reach
    // that address" are different claims: the boot died on a load from
    // 0x92000104, which IS the ATI register BAR and IS routed, because
    // nothing in the guest's own translation covered it.
    {
        printf("-- mmu: sdr1=%08x msr=%08x\n", cpu.st.sdr1, cpu.st.msr);
        for (u32 i = 0; i < 4; ++i) {
            const u32 u = cpu.st.dbatu[i], l = cpu.st.dbatl[i];
            if (!u && !l)
                continue;
            printf("   dbat%u %08x/%08x  ea %08x len %uKB -> pa %08x "
                   "%s%s wimg=%x pp=%u\n",
                   i, u, l, u & 0xFFFE0000u,
                   (((u >> 2) & 0x7FFu) + 1u) * 128u, l & 0xFFFE0000u,
                   (u & 2u) ? "Vs" : "--", (u & 1u) ? "Vp" : "--",
                   (l >> 3) & 0xFu, l & 3u);
        }
        for (u32 i = 0; i < 4; ++i) {
            const u32 u = cpu.st.ibatu[i], l = cpu.st.ibatl[i];
            if (!u && !l)
                continue;
            printf("   ibat%u %08x/%08x  ea %08x len %uKB -> pa %08x "
                   "%s%s\n",
                   i, u, l, u & 0xFFFE0000u,
                   (((u >> 2) & 0x7FFu) + 1u) * 128u, l & 0xFFFE0000u,
                   (u & 2u) ? "Vs" : "--", (u & 1u) ? "Vp" : "--");
        }
        printf("   sr:");
        for (u32 i = 0; i < 16; ++i)
            printf(" %08x", cpu.st.sr[i]);
        printf("\n");
    }
    if (romDumpPath) {
        // The flash as the guest left it. Open Firmware computes its own
        // NVRAM checksums, so a run that does `setenv` and `reset-all` and
        // then dumps here produces a correctly-configured image no amount
        // of guessing at the partition format would have produced.
        FILE* f = fopen(romDumpPath, "wb");
        if (f) {
            fwrite(bus.flash().data(), 1, bus.flash().size(), f);
            fclose(f);
            printf("-- boot flash dumped (%zu bytes): %s\n",
                   bus.flash().size(), romDumpPath);
        } else {
            printf("-- boot flash dump FAILED: %s\n", romDumpPath);
        }
    }
    {
        const auto& il = bus.atiIoLog();
        printf("-- ati i/o aperture (%zu; off/len <- val pc @insn):\n",
               il.size());
        const size_t is = il.size() > 60 ? il.size() - 60 : 0;
        for (size_t k = is; k < il.size(); ++k)
            printf("   +%02x/%u <- %08x pc=%08x @%llu\n", il[k].pa & 0xFFu,
                   (il[k].pa >> 8) & 7u, il[k].val, il[k].pc,
                   static_cast<unsigned long long>(il[k].at));
    }
    {
        const auto& fl = bus.flashLog();
        printf("-- boot flash writes (%zu; addr <- val pc @insn):\n",
               fl.size());
        for (const auto& w : fl)
            printf("   %08x <- %08x pc=%08x @%llu\n", w.pa, w.val, w.pc,
                   static_cast<unsigned long long>(w.at));
    }
    // Guest time, measured rather than assumed. The 68K park at ffc03664 is
    // `moveq #15,d0; add.l Ticks,d0; cmp.l Ticks,d0; bcc *-4` — a quarter-
    // second delay off the 60 Hz tick — so how fast Ticks runs IS whether
    // the boot proceeds. Print the guest's own decrementer programming next
    // to what it got, because "the tick is slow" and "the tick was never
    // programmed" call for opposite fixes.
    {
        const u32 ticksPa = 0x4000u + 0x016Au; // 68K lowmem Ticks
        const u32 tk = bus.read32(ticksPa);
        printf("-- guest time: Ticks=%u tb=%llu dec=%08x "
               "dec-writes=%llu dec-irqs=%llu last-reload=%u min-reload=%s\n",
               tk, static_cast<unsigned long long>(cpu.st.tb), cpu.st.dec,
               static_cast<unsigned long long>(cpu.decWrites),
               static_cast<unsigned long long>(cpu.decIrqs),
               cpu.decLastWrite,
               cpu.decMinPeriod == ~0ull
                   ? "none"
                   : std::to_string(cpu.decMinPeriod).c_str());
        if (cpu.decMinPeriod != ~0ull && cpu.decMinPeriod)
            printf("--   implied tick rate: one 0x900 per %llu TB ticks "
                   "= %.2f Hz at 25 MHz\n",
                   static_cast<unsigned long long>(cpu.decMinPeriod),
                   25000000.0 / static_cast<double>(cpu.decMinPeriod));
        // WHO programs the decrementer, and with WHAT. The guest's 60 Hz
        // chain is timebase-paced and runs ~43x slow, so a routine somewhere
        // is computing a period in timebase units and getting it wrong. A
        // total says that happened and names nobody; this names the sites,
        // busiest first, with the last value each one wrote.
        if (!cpu.decByPc.empty()) {
            std::vector<std::pair<u64, u32>> ds;
            for (const auto& [pc, s] : cpu.decByPc)
                ds.push_back({s.hits, pc});
            std::sort(ds.begin(), ds.end(),
                      [](const std::pair<u64, u32>& a,
                         const std::pair<u64, u32>& b) {
                          return a.first > b.first;
                      });
            printf("--   who writes DEC (%zu sites; value in TB ticks, "
                   "416666 = 60 Hz at 25 MHz):\n",
                   cpu.decByPc.size());
            for (size_t k = 0; k < ds.size() && k < 12; ++k) {
                const Cpu::DecSite& s = cpu.decByPc[ds[k].second];
                const char* nm = sym(ds[k].second);
                printf("     %08x x%-9llu last=%-10u (%.2f Hz) lr=%08x "
                       "r68=%08x%s%s\n",
                       ds[k].second, static_cast<unsigned long long>(s.hits),
                       s.lastVal,
                       s.lastVal ? 25000000.0 / static_cast<double>(s.lastVal)
                                 : 0.0,
                       s.lastLr, s.lastR24, nm ? "  " : "", nm ? nm : "");
            }
        }
    }
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
    printf("-- A-trap census, WHOLE BOOT (not just the DRVR body):\n");
    for (const auto& [t, n] : trapAll)
        printf("   $A%03x %-18s x%llu\n", t, trapName(t),
               static_cast<unsigned long long>(n));
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
        cpu.l1dFlushAll(true); // see ticksAtStart: RAM is not the guest's copy
        cpu.l2FlushAll(true);
        const u32 ticks = bus.read32(0x0000416Au);
        // Before the 68K world starts, that cell holds power-on RAM junk.
        // Printing a rate derived from junk is how a measurement becomes a
        // wrong belief, so say so instead: a plausible count is bounded by
        // 60/s against a run that has never been longer than minutes.
        char tickText[96];
        const bool haveBase =
            ticksAtStart < 100000000u && ticks >= ticksAtStart;
        const u32 moved = haveBase ? ticks - ticksAtStart : ticks;
        if (ticks < 100000000u)
            snprintf(tickText, sizeof tickText,
                     "guest Ticks=%u (+%u this run, %.1f/host-s; real is "
                     "60/s)",
                     ticks, moved, host > 0 ? moved / host : 0.0);
        else
            snprintf(tickText, sizeof tickText,
                     "guest Ticks n/a (68K lowmem not initialised yet)");
        // Every rate is over what THIS run did, not over the whole history a
        // snapshot carries. 25.00 MHz is real time; the ratio to it is the
        // number the pacing work moves.
        const u64 ranInsns = executed - executedAtStart;
        const u64 ranTb = cpu.st.tb - tbAtStart;
        // ⏳ SAY WHETHER THE IDLE MACHINE WAITED OR SPUN. A wait that never
        // happens and a wait that never ends look identical from outside the
        // loop — both are just a host second going by — and the difference is
        // a whole processor core. `idle` is the share of this run's host time
        // spent off the processor; on an idle Finder desktop it should be most
        // of it, and a run that reports 0 waits is spinning however good its
        // clock looks. `skipped` is the deadlines that were too near to be
        // worth a system call, which is a healthy number, not a fault.
        char rtText[160] = "";
        if (realtime)
            snprintf(rtText, sizeof rtText,
                     " [realtime, %llu slips, %llu waits (%.0f%% idle, %llu "
                     "skipped), ask cost %.2f ms]",
                     static_cast<unsigned long long>(pacer.slips),
                     static_cast<unsigned long long>(idleWait.waits),
                     host > 0 ? 100.0 * (idleWait.waitedNs / 1e9) / host : 0.0,
                     static_cast<unsigned long long>(idleWait.skipped),
                     idleWait.overshootNs / 1e6);
        printf("-- timing: %.1f s host, %.1f MIPS, tb +%llu (%.2f MHz = "
               "%.2fx real), %s%s\n",
               host, host > 0 ? ranInsns / host / 1e6 : 0.0,
               static_cast<unsigned long long>(ranTb),
               host > 0 ? ranTb / host / 1e6 : 0.0,
               host > 0 ? ranTb / host / 25.0e6 : 0.0, tickText, rtText);
        if (bench)
            printf("--   ^ THE APP'S LOOP (--bench): step, tick, sync, "
                   "deliver, and nothing else.\n");
        // ⚠ MIPS ALONE CANNOT TELL A FAST MACHINE FROM A SLEEPING ONE. Steps
        // charged without executing anything are still steps; say how many,
        // or an idle desktop reports a throughput it never achieved.
        if (cpu.napSkipped)
            printf("--   of which %llu (%.1f%%) were SKIPPED asleep, not "
                   "executed: %.1f MIPS of real work\n",
                   static_cast<unsigned long long>(cpu.napSkipped),
                   100.0 * static_cast<double>(cpu.napSkipped) /
                       static_cast<double>(ranInsns ? ranInsns : 1),
                   host > 0
                       ? (static_cast<double>(ranInsns) -
                          static_cast<double>(cpu.napSkipped)) /
                             host / 1e6
                       : 0.0);
        // 📏 HOW LONG A BATCH COULD BE. A run loop that charges several
        // instructions at once has to stop at anything that could move an
        // interrupt line: a guest access to a device, a timed device falling
        // due, and the decrementer — its own deadline and every reload of it.
        // The mean gap between those is the ceiling on batching, and it is
        // cheaper to print it than to build a batch and find out.
        const u64 breaks = bus.deviceServices() + cpu.decWrites + cpu.decIrqs;
        printf("--   batch ceiling: %llu device services (%llu guest device "
               "accesses) + %llu dec writes + %llu dec irqs = one break per "
               "%.0f instructions\n",
               static_cast<unsigned long long>(bus.deviceServices()),
               static_cast<unsigned long long>(bus.devGen()),
               static_cast<unsigned long long>(cpu.decWrites),
               static_cast<unsigned long long>(cpu.decIrqs),
               breaks ? static_cast<double>(ranInsns) /
                            static_cast<double>(breaks)
                      : 0.0);
        // What the compiled blocks actually carried. `insns` against the
        // run's total is the JIT's coverage; `bails` are lines it declined
        // (arena pressure); resets are wholesale arena recycles. refill
        // keep/drop says whether invalidation is churning on instrument
        // flushes (keeps) or on real self-modifying code (drops).
        if (cpu.jit)
            printf("--   jit: %.1f%% of instructions native (%llu of %llu), "
                   "%llu compiles, %llu enters, %llu refill-keeps, "
                   "%llu refill-drops, %llu bails, %llu arena resets\n",
                   ranInsns ? 100.0 * static_cast<double>(cpu.jit->insns) /
                                  static_cast<double>(ranInsns)
                            : 0.0,
                   static_cast<unsigned long long>(cpu.jit->insns),
                   static_cast<unsigned long long>(ranInsns),
                   static_cast<unsigned long long>(cpu.jit->compiles),
                   static_cast<unsigned long long>(cpu.jit->enters),
                   static_cast<unsigned long long>(cpu.jit->refillKeeps),
                   static_cast<unsigned long long>(cpu.jit->refillDrops),
                   static_cast<unsigned long long>(cpu.jit->bails),
                   static_cast<unsigned long long>(cpu.jit->resets));
        // Stage B: hops are dispatcher round trips the chained exits saved.
        // insns per (enter + hop) is the true scaffold amortization — the
        // number that was 4.3 when every block exit paid the full round trip.
        if (cpu.jit && (cpu.jitChainHops || cpu.jit->chainResolves))
            printf("--   jit chain: %llu hops, %llu links, %llu severs, "
                   "%llu resolves (%llu misses, %llu giveups); "
                   "%.1f insns/dispatch-enter, %.1f insns/native-entry\n",
                   static_cast<unsigned long long>(cpu.jitChainHops),
                   static_cast<unsigned long long>(cpu.jit->chainLinks),
                   static_cast<unsigned long long>(cpu.jit->chainSevers),
                   static_cast<unsigned long long>(cpu.jit->chainResolves),
                   static_cast<unsigned long long>(cpu.jit->chainMisses),
                   static_cast<unsigned long long>(cpu.jit->chainGiveups),
                   cpu.jit->enters
                       ? static_cast<double>(cpu.jit->insns) /
                             static_cast<double>(cpu.jit->enters)
                       : 0.0,
                   cpu.jit->enters + cpu.jitChainHops
                       ? static_cast<double>(cpu.jit->insns) /
                             static_cast<double>(cpu.jit->enters +
                                                 cpu.jitChainHops)
                       : 0.0);
        if (cpu.jit && (cpu.jit->fallbacks || cpu.jit->memOps))
            printf("--   jit mix: %llu mem ops (%.1f%% of native insns) "
                   "+ %llu fp mem ops (%.1f%%), %llu fallback insns (%.1f%%)\n",
                   static_cast<unsigned long long>(cpu.jit->memOps),
                   cpu.jit->insns ? 100.0 * static_cast<double>(cpu.jit->memOps) /
                                        static_cast<double>(cpu.jit->insns)
                                  : 0.0,
                   static_cast<unsigned long long>(cpu.jit->fpMemOps),
                   cpu.jit->insns
                       ? 100.0 * static_cast<double>(cpu.jit->fpMemOps) /
                             static_cast<double>(cpu.jit->insns)
                       : 0.0,
                   static_cast<unsigned long long>(cpu.jit->fallbacks),
                   cpu.jit->insns
                       ? 100.0 * static_cast<double>(cpu.jit->fallbacks) /
                             static_cast<double>(cpu.jit->insns)
                       : 0.0);
        if (cpu.jit && cpu.jit->fallbacks) {
            std::vector<std::pair<u64, u32>> fb;
            for (u32 k = 0; k < 1024; ++k)
                if (cpu.jit->fbByRow[k])
                    fb.push_back({cpu.jit->fbByRow[k], k});
            std::sort(fb.rbegin(), fb.rend());
            // With --jit-tsc the census sorts by HOST CYCLES, not by calls:
            // the frequent row and the expensive row are different questions
            // and lowering work should follow the second one.
            const bool byTsc = cpu.jit->tscFb != 0;
            if (byTsc) {
                fb.clear();
                for (u32 k = 0; k < 1024; ++k)
                    if (cpu.jit->tscByRow[k])
                        fb.push_back({cpu.jit->tscByRow[k], k});
                std::sort(fb.rbegin(), fb.rend());
            }
            printf("--   jit fallback census (top 20 of %zu rows, by %s):\n",
                   fb.size(), byTsc ? "host cycles" : "calls");
            for (size_t k = 0; k < fb.size() && k < 20; ++k) {
                const u32 row = fb[k].second;
                const char* mn = row < kIsaCount ? kIsa[row].mnem : "?";
                if (byTsc)
                    printf("       %-10s %llu cycles (%.1f%% of fallback, "
                           "%llu calls, %.0f/call)\n",
                           mn,
                           static_cast<unsigned long long>(fb[k].first),
                           100.0 * static_cast<double>(fb[k].first) /
                               static_cast<double>(cpu.jit->tscFb),
                           static_cast<unsigned long long>(
                               cpu.jit->fbByRow[row]),
                           cpu.jit->fbByRow[row]
                               ? static_cast<double>(fb[k].first) /
                                     static_cast<double>(cpu.jit->fbByRow[row])
                               : 0.0);
                else
                    printf("       %-10s %llu\n", mn,
                           static_cast<unsigned long long>(fb[k].first));
            }
        }
        if (sf::gFastHits || sf::gFastMiss)
            printf("--   softfp fast path (single): %llu taken, %llu bailed "
                   "(%.1f%%)\n",
                   static_cast<unsigned long long>(sf::gFastHits),
                   static_cast<unsigned long long>(sf::gFastMiss),
                   100.0 * static_cast<double>(sf::gFastHits) /
                       static_cast<double>(sf::gFastHits + sf::gFastMiss));
        if (sf::gFastHitsD || sf::gFastMissD)
            printf("--   softfp fast path (double): %llu taken, %llu bailed "
                   "(%.1f%%)\n",
                   static_cast<unsigned long long>(sf::gFastHitsD),
                   static_cast<unsigned long long>(sf::gFastMissD),
                   100.0 * static_cast<double>(sf::gFastHitsD) /
                       static_cast<double>(sf::gFastHitsD + sf::gFastMissD));
        if (cpu.jit && cpu.jit->tscProbe) { // populated only by --jit-tsc
            const JitCache& J = *cpu.jit;
            printf("--   jit tsc: probe %.1f/enter, native %.1f/enter "
                   "(%.1f per native insn)\n",
                   static_cast<double>(J.tscProbe) /
                       static_cast<double>(J.enters),
                   static_cast<double>(J.tscNative) /
                       static_cast<double>(J.enters),
                   J.insns ? static_cast<double>(J.tscNative) /
                                 static_cast<double>(J.insns)
                           : 0.0);
            // ⭐ WHERE THE BLOCKS' TIME GOES — the aim point for the next
            // lowering round. Shares are of tscNative (the emitted code
            // itself, dispatcher overhead excluded); "emitted" is what is
            // left after the two call-shaped costs are taken out, i.e. the
            // inlined instructions plus the chain gates.
            const u64 mem = J.tscMem, fbk = J.tscFb;
            const u64 calls = J.memOps + J.fpMemOps;
            const double nat = static_cast<double>(J.tscNative);
            printf("--   jit tsc split: mem %llu (%.1f%%, %.0f/op over %llu "
                   "ops), fallback %llu (%.1f%%, %.0f/op over %llu ops), "
                   "emitted+gates %.1f%%\n",
                   static_cast<unsigned long long>(mem),
                   nat ? 100.0 * static_cast<double>(mem) / nat : 0.0,
                   calls ? static_cast<double>(mem) /
                               static_cast<double>(calls)
                         : 0.0,
                   static_cast<unsigned long long>(calls),
                   static_cast<unsigned long long>(fbk),
                   nat ? 100.0 * static_cast<double>(fbk) / nat : 0.0,
                   J.fallbacks ? static_cast<double>(fbk) /
                                     static_cast<double>(J.fallbacks)
                               : 0.0,
                   static_cast<unsigned long long>(J.fallbacks),
                   nat ? 100.0 * (nat - static_cast<double>(mem + fbk)) / nat
                       : 0.0);
        }
#if OPM_PROFILING
        // ⭐ HOW MUCH STRAIGHT-LINE CODE THIS MACHINE ACTUALLY RUNS. The line
        // executor's whole saving is the fetch work it does once per LINE
        // instead of once per instruction, so instructions-per-entry IS the
        // multiplier — an average of 1 would mean it bought nothing at all.
        if (cpu.lineEntries) {
            // ⚠ Against instructions actually EXECUTED, not against the step
            // count. At the desktop most steps are skipped asleep and never
            // reach an executor at all, so the step count as a denominator
            // makes an executor that covers nine tenths of the real work read
            // as 40%.
            const u64 ran =
                ranInsns > cpu.napSkipped ? ranInsns - cpu.napSkipped : 1;
            printf("--   line executor: %llu instructions over %llu entries = "
                   "%.2f per entry (%.1f%% of instructions executed)\n",
                   static_cast<unsigned long long>(cpu.lineInsns),
                   static_cast<unsigned long long>(cpu.lineEntries),
                   static_cast<double>(cpu.lineInsns) /
                       static_cast<double>(cpu.lineEntries),
                   100.0 * static_cast<double>(cpu.lineInsns) /
                       static_cast<double>(ran));
        }
#endif
    }
    // Where the host time went. Sampled, so it is a distribution and not a
    // ledger: percentages under about 1% are noise at any realistic rate.
    if (prof::running()) {
        prof::stop();
        const prof::Result& r = prof::result();
        printf("-- profile: %llu samples over %.1f host-s"
               "%s%s\n",
               static_cast<unsigned long long>(r.samples), r.seconds,
               bench ? " [bench loop]" : " [instrumented loop]",
               r.missed ? " (sampler fell behind; see 'missed')" : "");
        if (r.missed)
            printf("--   missed=%llu periods — the rate was optimistic, the "
                   "SHARES are still valid\n",
                   static_cast<unsigned long long>(r.missed));
        std::vector<std::pair<u64, u32>> byPh;
        for (u32 k = 0; k < static_cast<u32>(prof::Ph::N); ++k)
            if (r.phase[k])
                byPh.push_back({r.phase[k], k});
        std::sort(byPh.begin(), byPh.end(),
                  [](const std::pair<u64, u32>& a,
                     const std::pair<u64, u32>& b) { return a.first > b.first; });
        for (const auto& [n, k] : byPh)
            printf("   %-16s %5.1f%%  %llu\n",
                   prof::name(static_cast<prof::Ph>(k)),
                   r.samples ? 100.0 * static_cast<double>(n) /
                                   static_cast<double>(r.samples)
                             : 0.0,
                   static_cast<unsigned long long>(n));
        // The other half of the answer. A sampler says where the time went; a
        // hit rate says whether a cache is catching anything. Sequential
        // execution implies 7 in 8, so anything much below that means
        // something is dropping the block.
        const u64 ftot = cpu.fetchHits + cpu.fetchFillsL1 + cpu.fetchFillsL2 +
                         cpu.fetchFillsMem + cpu.fetchUncached;
        if (ftot)
            printf("--   fetch buffer: %.1f%% hit (%llu of %llu); fills "
                   "L1=%llu L2=%llu mem=%llu, uncacheable=%llu\n",
                   100.0 * static_cast<double>(cpu.fetchHits) /
                       static_cast<double>(ftot),
                   static_cast<unsigned long long>(cpu.fetchHits),
                   static_cast<unsigned long long>(ftot),
                   static_cast<unsigned long long>(cpu.fetchFillsL1),
                   static_cast<unsigned long long>(cpu.fetchFillsL2),
                   static_cast<unsigned long long>(cpu.fetchFillsMem),
                   static_cast<unsigned long long>(cpu.fetchUncached));
        // WHY it missed, not just how often. A capacity miss wants more lines;
        // a whole-cache drop wants a narrower invalidation, and enlarging the
        // cache would only make each drop cost more.
        if (cpu.napSteps)
            printf("--   nap steps: %llu stepped one at a time — the core was "
                   "asleep and fetched nothing\n",
                   static_cast<unsigned long long>(cpu.napSteps));
        if (ftot)
            printf("--   fetch drops: icbi=%llu snoop=%llu flush=%llu "
                   "(each throws away all %u lines)\n",
                   static_cast<unsigned long long>(cpu.fetchDropIcbi),
                   static_cast<unsigned long long>(cpu.fetchDropSnoop),
                   static_cast<unsigned long long>(cpu.fetchDropFlush),
                   Cpu::kFetchLines);
        u32 hotPc[24];
        u64 hotN[24];
        const size_t np = prof::topPcs(hotPc, hotN, 24);
        if (np) {
            printf("--   hottest GUEST pcs (where the machine is spending "
                   "its own time):\n");
            for (size_t k = 0; k < np; ++k) {
                const char* s = sym(hotPc[k]);
                printf("     %08x %5.2f%%%s%s\n", hotPc[k],
                       r.samples ? 100.0 * static_cast<double>(hotN[k]) /
                                       static_cast<double>(r.samples)
                                 : 0.0,
                       s ? "  " : "", s ? s : "");
            }
        }
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
        // The whole list with --coverage-all: two deterministic runs that
        // end differently diverge at a first-entry, and the last 32 cannot
        // show where.
    // Open Firmware word names, read out of its own dictionary.
    //
    // Every colon definition is laid out [len][name][pad to 4][code], which
    // is how `eol>` and `do-esc#` were identified by hand from a hex dump.
    // Doing that by hand for a 456-entry coverage timeline is why three
    // separate OF questions this session were answered with "some address
    // in ff82xxxx".
    //
    // Through the MMU, never through the ROM-in-RAM shortcut: that
    // shortcut already disassembled live Forth as a data table once today.
    cpu.l1dFlushAll(true);
    cpu.l2FlushAll(true);
    const CpuState ofSaved = cpu.st;
    cpu.st.msr |= 0x30u;
    const CpuState ofArmed = cpu.st;
    auto ofByte = [&](u32 va, u8& out) -> bool {
        u32 pa = 0;
        cpu.st = ofArmed;
        const bool ok = cpu.translate(va, false, false, pa);
        cpu.st = ofArmed;
        if (!ok)
            return false;
        if (pa >= bus.ramBytes())
            return false;
        out = static_cast<u8>(bus.read32(pa & ~3u) >> (8 * (3 - (pa & 3u))));
        return true;
    };
    auto ofWordName = [&](u32 va) -> std::string {
        if (va < 0xFF800000u || va >= 0xFFA00000u || (va & 3u))
            return std::string();
        for (u32 len = 1; len <= 31; ++len) {
            const u32 hdr = va - ((1u + len + 3u) & ~3u);
            u8 b = 0;
            if (!ofByte(hdr, b) || b != len)
                continue;
            std::string s;
            bool ok = true;
            for (u32 k = 0; k < len && ok; ++k) {
                if (!ofByte(hdr + 1u + k, b) || b < 0x21 || b > 0x7E)
                    ok = false;
                else
                    s.push_back(static_cast<char>(b));
            }
            for (u32 k = 1u + len; ok && k < ((1u + len + 3u) & ~3u); ++k)
                if (!ofByte(hdr + k, b) || b != 0)
                    ok = false;
            if (ok)
                return s;
        }
        return std::string();
    };
    if (!ofHist.empty()) {
        std::vector<std::pair<u64, std::string>> byCount;
        for (const auto& hv : ofHist)
            byCount.push_back({hv.second, hv.first});
        std::sort(byCount.begin(), byCount.end(),
                  [](const std::pair<u64, std::string>& a,
                     const std::pair<u64, std::string>& b) {
                      return a.first > b.first;
                  });
        printf("-- of word histogram over [%llu,%llu): %zu distinct\n",
               static_cast<unsigned long long>(ofHistFrom),
               static_cast<unsigned long long>(ofHistTo), byCount.size());
        for (size_t k = 0; k < byCount.size() && k < 60; ++k)
            printf("   %8llu  %s\n",
                   static_cast<unsigned long long>(byCount[k].first),
                   byCount[k].second.c_str());
        printf("-- of words in first-entry order (%zu):\n",
               ofHistFirst.size());
        for (const auto& fv : ofHistFirst)
            printf("   @%-12llu %s\n",
                   static_cast<unsigned long long>(fv.first),
                   fv.second.c_str());
    }
        printf("-- coverage timeline (%zu regions; %s):\n", firsts.size(),
               coverageAll ? "all first-entries" : "last 32 first-entries");
        const size_t start =
            (!coverageAll && firsts.size() > 32) ? firsts.size() - 32 : 0;
        for (u32 k = 0; k < ofSymN; ++k) {
            const std::string nm = ofWordName(ofSymAt[k]);
            printf("-- of word %08x: %s\n", ofSymAt[k],
                   nm.empty() ? "<no header here>" : nm.c_str());
        }
        cpu.st = ofSaved;
        cpu.raisedThisStep = false;
        for (size_t k = start; k < firsts.size(); ++k) {
            const std::string nm = ofWordName(firsts[k].second);
            printf("   @%-11llu %08x%s%s\n",
                   static_cast<unsigned long long>(firsts[k].first),
                   firsts[k].second, nm.empty() ? "" : "  ", nm.c_str());
        }
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
        printf("-- cd command log (%zu; c=ata p=packet e=err; packet ops\n"
               "   carry their full CDB, which is what a refusal is about):\n",
               cl.size());
        const size_t cs = cl.size() > 200 ? cl.size() - 200 : 0;
        for (size_t k = cs; k < cl.size(); ++k) {
            printf("   %c%02x @%-12llu pc=%08x xfer=%-6u", cl[k].kind,
                   cl[k].val, static_cast<unsigned long long>(cl[k].at),
                   cl[k].pc, cl[k].xfer);
            if (cl[k].a || cl[k].b)
                printf(" lba=%x+%x", cl[k].a, cl[k].b);
            if (cl[k].kind != 'c') {
                printf(" cdb=");
                for (u32 c = 0; c < 12; ++c)
                    printf("%02x", cl[k].cdb[c]);
            }
            printf("\n");
        }
    }
    {
        // The disk's own command log, the same way the CD's is reported.
        // Register traffic says what the driver poked; the command log says
        // what it ASKED FOR, which is the question when a boot device is
        // probed but never read.
        const auto& hl = bus.hd().log;
        // The cell's own counters at the moment the run stopped. "The host
        // stopped reading" and "the drive stopped offering" produce the
        // same register trace; only these tell them apart.
        bus.hd().dumpState("hd");
        printf("-- hd command log (%zu; c=ata D=ata/dma p=packet e=err):\n   ",
               hl.size());
        // Head AND tail: the head is how a probe STARTED, which is what
        // decides everything after it, and a tail-only view hid it.
        for (size_t k = 0; k < hl.size() && k < 40; ++k) {
            if (hl[k].a || hl[k].b)
                printf("%c%02x:%x+%x@%llu/%08x ", hl[k].kind, hl[k].val,
                       hl[k].a, hl[k].b,
                       static_cast<unsigned long long>(hl[k].at), hl[k].pc);
            else
                printf("%c%02x@%llu/%08x", hl[k].kind, hl[k].val,
                       static_cast<unsigned long long>(hl[k].at), hl[k].pc);
            if (hl[k].xfer) printf("[%uB]", hl[k].xfer);
            printf(" ");
        }
        printf("\n   ... tail ...\n   ");
        const size_t hs = hl.size() > 200 ? hl.size() - 200 : 0;
        for (size_t k = hs; k < hl.size(); ++k) {
            if (hl[k].a || hl[k].b)
                printf("%c%02x:%x+%x@%llu/%08x ", hl[k].kind, hl[k].val, hl[k].a,
                       hl[k].b, static_cast<unsigned long long>(hl[k].at), hl[k].pc);
            else
                printf("%c%02x@%llu/%08x", hl[k].kind, hl[k].val,
                       static_cast<unsigned long long>(hl[k].at), hl[k].pc);
            if (hl[k].xfer) printf("[%uB]", hl[k].xfer);
            printf(" ");
        }
        printf("\n");
    }
    {
        // The TAIL, not the head: the log is trimmed as it grows, so the
        // first 120 entries are an arbitrary window that has nothing to do
        // with what the run was asked to investigate.
        const auto& al = bus.ataLog();
        printf("-- ata traffic (%zu; last 1200; off r/w.width val pc):\n",
               al.size());
        const size_t as = al.size() > 1200 ? al.size() - 1200 : 0;
        for (size_t k = as; k < al.size(); ++k)
            printf("   +%05x %c%u %08x pc=%08x @%llu\n",
                   al[k].pa & 0x7FFFFFFFu,
                   (al[k].pa & 0x80000000u) ? 'r' : 'w', al[k].len,
                   al[k].val, al[k].pc,
                   static_cast<unsigned long long>(al[k].at));
    }
    {
        const auto& cl = bus.cfgLog();
        printf("-- pci config accesses (%zu; bus latch val pc r/w):\n",
               cl.size());
        // All of them with --coverage-all: which BUSES the firmware probes
        // is the question, and the last 100 accesses cannot answer it.
        const size_t cfs =
            (!coverageAll && cl.size() > 100) ? cl.size() - 100 : 0;
        for (size_t k = cfs; k < cl.size(); ++k)
            printf("   f%u %08x %08x pc=%08x %c @%llu\n",
                   ((cl[k].pa >> 28) & 7u) * 2u, cl[k].pa & 0x00FFFFFFu,
                   cl[k].val, cl[k].pc & ~1u, (cl[k].pc & 1u) ? 'w' : 'r',
                   static_cast<unsigned long long>(cl[k].at));
        // The AGP host function (f0 dev 11, 106b:0020) gets its own COMPLETE
        // section: whether Mac OS's .AGP driver ever probes it — and what it
        // read when it did — is the question the 100-entry tail cannot
        // answer. Bus-0 dev-11 idsel latch is 0x800 | reg, so
        // (latch & 0xFFFF00) == 0x000800 selects exactly this function.
        size_t agpN = 0;
        for (const auto& e : cl)
            if (((e.pa >> 28) & 7u) == 0u &&
                (e.pa & 0x00FFFF00u) == 0x00000800u)
                ++agpN;
        printf("-- agp host fn (f0 dev 11, 106b:0020) config accesses: %zu\n",
               agpN);
        for (const auto& e : cl)
            if (((e.pa >> 28) & 7u) == 0u &&
                (e.pa & 0x00FFFF00u) == 0x00000800u)
                printf("   reg +%02x %08x pc=%08x %c @%llu\n", e.pa & 0xFFu,
                       e.val, e.pc & ~1u, (e.pc & 1u) ? 'w' : 'r',
                       static_cast<unsigned long long>(e.at));
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
    if (!bus.wmap.empty()) {
        std::vector<std::pair<u64, u32>> hot;
        for (const auto& b : bus.wmap)
            hot.push_back({b.second, b.first});
        std::sort(hot.begin(), hot.end(),
                  [](const std::pair<u64, u32>& a,
                     const std::pair<u64, u32>& b) {
                      return a.first > b.first;
                  });
        printf("-- write heatmap over [%llu,%llu): %zu MiB buckets\n",
               (unsigned long long)wmapFrom, (unsigned long long)wmapTo,
               hot.size());
        for (size_t k = 0; k < hot.size() && k < 24; ++k)
            printf("   %04x00000  %12llu writes\n", hot[k].second,
                   (unsigned long long)hot[k].first);
        if (!bus.wmapPcs.empty()) {
            std::vector<std::pair<u64, u32>> pcs;
            for (const auto& p : bus.wmapPcs)
                pcs.push_back({p.second, p.first});
            std::sort(pcs.begin(), pcs.end(),
                      [](const std::pair<u64, u32>& a,
                         const std::pair<u64, u32>& b) {
                          return a.first > b.first;
                      });
            printf("--   writers into %04x00000 (%zu distinct pcs):\n",
                   bus.wmapPcBucket, pcs.size());
            for (size_t k = 0; k < pcs.size() && k < 20; ++k)
                printf("     pc=%08x  %12llu writes\n", pcs[k].second,
                       (unsigned long long)pcs[k].first);
        }
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
        // Open Firmware's own words are the best instrument this machine
        // has — printenv, dev, .properties, words — but their answers are
        // long and the digest above truncates. Write the stream verbatim so
        // a script can ask a real question and read the whole reply.
        if (serialLogPath) {
            FILE* sf = fopen(serialLogPath, "wb");
            if (sf) {
                fwrite(con.data(), 1, con.size(), sf);
                fclose(sf);
                printf("-- serial console written to %s (%zu bytes)\n",
                       serialLogPath, con.size());
            } else {
                printf("-- serial console: cannot write %s\n",
                       serialLogPath);
            }
        }
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
        // `log` above is capped AND snapshotted, so after a resume it still
        // holds Open Firmware's traffic and shows none of the OS's. These
        // two censuses are uncapped and unsnapshotted: on a resumed run they
        // describe the OS era and nothing else.
        printf("-- ohci%u register census (read / write per offset):\n", f);
        {
            std::map<u32, std::pair<u64, u64>> both;
            for (const auto& [off, n] : bus.ohci(f).readCount)
                both[off].first = n;
            for (const auto& [off, n] : bus.ohci(f).writeCount)
                both[off].second = n;
            for (const auto& [off, rw] : both)
                printf("   +%03x %s r=%llu w=%llu\n", off, ohciRegName(off),
                       static_cast<unsigned long long>(rw.first),
                       static_cast<unsigned long long>(rw.second));
        }
        // Enumeration lives or dies in the root-hub registers, and they are
        // touched tens of times, not thousands — so log them in full rather
        // than counting them.
        // WHICH interrupt the guest is servicing. SF every frame is ordinary;
        // an RHSC it acks thousands of times is a port event it keeps being
        // told about and never acts on. Those need opposite fixes and the
        // aggregate InterruptStatus counts cannot tell them apart.
        {
            static const char* kIntBit[8] = {
                "SO  scheduling-overrun", "WDH writeback-done",
                "SF  start-of-frame",     "RD  resume-detected",
                "UE  unrecoverable-err",  "FNO frame-no-overflow",
                "RHSC root-hub-change",   "OC  ownership-change"};
            const auto& oc = bus.ohci(f);
            printf("-- ohci%u interrupts (raised / acked), %llu frames "
                   "from @%llu to @%llu:\n",
                   f, static_cast<unsigned long long>(oc.frames),
                   static_cast<unsigned long long>(oc.firstFrameAt),
                   static_cast<unsigned long long>(oc.lastFrameAt));
            // An assertion the guest never armed reaches nobody: SF raised a
            // million times is only a storm if SF is set in intEnable.
            printf("   final: control=%08x intEnable=%08x intStatus=%08x "
                   "irq=%d rhDescA=%08x port1=%08x port2=%08x\n",
                   oc.controlView(), oc.intEnableView(), oc.intStatusView(),
                   oc.irqLine() ? 1 : 0, oc.rhDescAView(), oc.portView(0),
                   oc.portView(1));
            for (u32 b = 0; b < 8; ++b)
                if (oc.intRaised[b] || oc.intCleared[b])
                    printf("   %-24s raised=%llu acked=%llu\n", kIntBit[b],
                           static_cast<unsigned long long>(oc.intRaised[b]),
                           static_cast<unsigned long long>(oc.intCleared[b]));
            for (const auto& [v, n] : oc.intEnWrites)
                printf("   arm   %08x x%llu%s\n", v,
                       static_cast<unsigned long long>(n),
                       (v & 0x40u) ? "   <-- includes RHSC" : "");
            for (const auto& [v, n] : oc.intDisWrites)
                printf("   disarm %08x x%llu%s\n", v,
                       static_cast<unsigned long long>(n),
                       (v & 0x40u) ? "   <-- includes RHSC" : "");
        }
        const auto& rh = bus.ohci(f).rhLog;
        printf("-- ohci%u root hub (%zu accesses):\n", f, rh.size());
        for (size_t k = 0; k < rh.size() && k < 600; ++k)
            printf("   %s +%03x %s %08x pc=%08x @%llu\n",
                   rh[k].wr ? "W" : "R", rh[k].off, ohciRegName(rh[k].off),
                   rh[k].val, rh[k].pc,
                   static_cast<unsigned long long>(rh[k].at));
        // What the list walker read, as it read it. A host stuck polling
        // HcInterruptStatus is waiting for a transfer to retire, and only
        // the descriptors the controller actually fetched can tell "the
        // host queued nothing" from "we walked them wrong".
        // The HID census answers what the register log cannot: is the guest
        // actually POLLING this device? Zero interrupt-IN TDs on a controller
        // that enumerated means the driver bound and then never asked for a
        // report, which is a different bug from "the reports are wrong".
        printf("-- ohci%u hid: %llu setups, %llu interrupt-IN TDs, %llu "
               "reports delivered, %llu injected\n",
               f, static_cast<unsigned long long>(bus.ohci(f).setupsSeen),
               static_cast<unsigned long long>(bus.ohci(f).inTds),
               static_cast<unsigned long long>(bus.ohci(f).reportsSent),
               static_cast<unsigned long long>(f == 1 ? mouseSent : 0));
        printf("   branches: ep0=%llu nak-empty=%llu no-buffer=%llu\n",
               static_cast<unsigned long long>(bus.ohci(f).inEp0),
               static_cast<unsigned long long>(bus.ohci(f).nakEmpty),
               static_cast<unsigned long long>(bus.ohci(f).noBuffer));
        printf("   polled from @%llu to @%llu\n",
               static_cast<unsigned long long>(bus.ohci(f).firstInTd),
               static_cast<unsigned long long>(bus.ohci(f).lastInTd));
        const auto& wl = bus.ohci(f).walkLog;
        printf("-- ohci%u list walk (%zu%s):\n", f, wl.size(),
               wl.size() >= bus.ohci(f).walkMax ? ", capped" : "");
        for (const auto& w : wl) {
            if (w.kind == 0)
                printf("   ED %08x flags=%08x headP=%08x tailP=%08x "
                       "next=%08x\n",
                       w.a, w.b, w.c, w.d, w.e);
            else if (w.kind == 2)
                printf("   SETUP %08x  %02x %02x %02x %02x %02x %02x %02x "
                       "%02x  reply=%u\n",
                       w.a, w.b & 0xFF, (w.b >> 8) & 0xFF,
                       (w.b >> 16) & 0xFF, (w.b >> 24) & 0xFF, w.c & 0xFF,
                       (w.c >> 8) & 0xFF, (w.c >> 16) & 0xFF,
                       (w.c >> 24) & 0xFF, w.d);
            else if (w.kind == 3)
                // The HID report as bytes, signed, next to the deltas it was
                // built from -- "relative motion was queued" and "relative
                // motion reached the wire" are different claims.
                printf("   HIDREP %08x  %02x %02x %02x %02x  (buttons=%u dx=%d "
                       "dy=%d)  wanted dx=%d dy=%d  moved=%u\n",
                       w.a, w.b & 0xFF, (w.b >> 8) & 0xFF, (w.b >> 16) & 0xFF,
                       (w.b >> 24) & 0xFF, w.b & 0xFF,
                       static_cast<int>(static_cast<int8_t>((w.b >> 8) & 0xFF)),
                       static_cast<int>(static_cast<int8_t>((w.b >> 16) & 0xFF)),
                       static_cast<int>(static_cast<int32_t>(w.c)),
                       static_cast<int>(static_cast<int32_t>(w.d)), w.e);
            else
                printf("   TD %08x ctl=%08x cbp=%08x be=%08x moved=%u\n",
                       w.a, w.b, w.c, w.d, w.e);
        }
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
               "4=dead 5=storequad):\n",
               dl.size());
        for (size_t k = 0; k < dl.size() && k < 40; ++k)
            printf("   %u %08x %08x @%llu\n", dl[k].kind, dl[k].a,
                   dl[k].b, static_cast<unsigned long long>(dl[k].at));
        if (dl.size() > 40) {
            printf("   ... tail ...\n");
            for (size_t k = dl.size() > 120 ? dl.size() - 120 : 40;
                 k < dl.size(); ++k)
                printf("   %u %08x %08x @%llu\n", dl[k].kind, dl[k].a,
                       dl[k].b, static_cast<unsigned long long>(dl[k].at));
        }
    }
    {
        // The DISK's channel. Never stamped and never printed, so every
        // READ DMA the boot issued went unobserved while the CD's idle
        // channel was being read as though it were the disk's.
        const auto& dl = bus.hdDma().log;
        printf("-- hd dbdma events (%zu; 0=ctl 1=desc 2=input 3=stop "
               "4=dead 5=storequad):\n",
               dl.size());
        for (size_t k = 0; k < dl.size() && k < 40; ++k)
            printf("   %u %08x %08x @%llu\n", dl[k].kind, dl[k].a,
                   dl[k].b, static_cast<unsigned long long>(dl[k].at));
        if (dl.size() > 40) {
            printf("   ... tail ...\n");
            for (size_t k = dl.size() > 120 ? dl.size() - 120 : 40;
                 k < dl.size(); ++k)
                printf("   %u %08x %08x @%llu\n", dl[k].kind, dl[k].a,
                       dl[k].b, static_cast<unsigned long long>(dl[k].at));
        }
    }
    {
        // The codec and its two channels. `played` is the only number that
        // says the output path works end to end: the guest built a
        // descriptor list, the engine walked it, and the codec accepted
        // bytes at its own sample rate.
        const AwacsCell& sc = bus.sound();
        printf("-- sound: %s  ctl=%08x (rate %u Hz, in sf %u, out sf %u)  "
               "codec=%08x  byteswap=%08x  frames=%llu\n",
               bus.soundOn ? "on" : "OFF (--no-sound)", sc.soundCtl(),
               sc.rateHz(), sc.soundCtl() & 0xFu, (sc.soundCtl() >> 4) & 0xFu,
               sc.codecCtl(), sc.byteSwap(),
               static_cast<unsigned long long>(sc.frameCount()));
        printf("--   played %llu bytes, captured %llu, underruns %llu, "
               "queued %zu\n",
               static_cast<unsigned long long>(sc.bytesPlayed()),
               static_cast<unsigned long long>(sc.bytesCaptured()),
               static_cast<unsigned long long>(sc.underruns()), sc.queued());
        for (u32 c = 0; c < 2; ++c) {
            const auto& dl = c ? bus.sndIn().log : bus.sndOut().log;
            printf("--   %s dbdma events (%zu; 0=ctl 1=desc 2=data 3=stop "
                   "4=dead 5=storequad 7=branch 8=loadquad):\n",
                   c ? "audio-in" : "audio-out", dl.size());
            for (size_t k = 0; k < dl.size() && k < 24; ++k)
                printf("     %u %08x %08x @%llu\n", dl[k].kind, dl[k].a,
                       dl[k].b, static_cast<unsigned long long>(dl[k].at));
            if (dl.size() > 24) {
                printf("     ... tail ...\n");
                for (size_t k = dl.size() > 124 ? dl.size() - 100 : 24;
                     k < dl.size(); ++k)
                    printf("     %u %08x %08x @%llu\n", dl[k].kind, dl[k].a,
                           dl[k].b, static_cast<unsigned long long>(dl[k].at));
            }
            const auto& st = dbdmaIrqStats(c);
            printf("--   %s irq: set %llu, swallowed-by-ctl-write %llu, "
                   "pulsed %llu\n",
                   c ? "audio-in" : "audio-out",
                   static_cast<unsigned long long>(st.set),
                   static_cast<unsigned long long>(st.dropCtl),
                   static_cast<unsigned long long>(st.pulsed));
        }
        printf("--   codec regs:");
        for (u32 i = 0; i < 8; ++i)
            printf(" %u=%03x", i, sc.codecReg(i));
        printf("\n");
        // The guest's own traffic on the codec's register window. The Frame
        // Count reads are the ones that matter: the value the driver saw is
        // what its next mix size was computed from.
        const auto& rl = awacsRegLog();
        static const char* kRegName[] = {"SoundCtl", "CodecCtl", "CodecStat",
                                         "ClipCnt",  "ByteSwap", "FrameCnt"};
        u64 perReg[6][2] = {};
        for (const auto& e : rl)
            if (e.off < 0x60)
                ++perReg[e.off >> 4][e.wr ? 1 : 0];
        printf("--   codec reg traffic (%zu):", rl.size());
        for (u32 i = 0; i < 6; ++i)
            if (perReg[i][0] || perReg[i][1])
                printf("  %s r=%llu w=%llu", kRegName[i],
                       static_cast<unsigned long long>(perReg[i][0]),
                       static_cast<unsigned long long>(perReg[i][1]));
        printf("\n");
        for (size_t k = rl.size() > 80 ? rl.size() - 80 : 0; k < rl.size(); ++k) {
            const auto& e = rl[k];
            printf("--     %c %-8s len=%u %08x pc=%08x tb=%llu @%llu\n",
                   e.wr ? 'w' : 'r',
                   e.off < 0x60 ? kRegName[e.off >> 4] : "?", e.len, e.val,
                   e.pc, static_cast<unsigned long long>(e.tb),
                   static_cast<unsigned long long>(e.insn));
        }
    }
    if (wavOut) {
        // ⭐ THE BYTE ORDER IS DECIDED BY MEASUREMENT, NOT BY THE REGISTER.
        // Byte Swapping at +0x40 is the one sound register the ROM writes
        // with a plain store rather than a byte-reversed one, so reading a
        // sample order out of it would be a guess. Audio is smooth and noise
        // is not: the mean absolute difference between neighbouring samples
        // is tiny for the right interpretation and near full scale for the
        // wrong one, and the two numbers are printed so the claim is
        // checkable rather than asserted.
        auto rough = [&](bool be) {
            if (wavPcm.size() < 8)
                return 0.0;
            double sum = 0;
            size_t n = 0;
            int prev = 0;
            for (size_t k = 0; k + 1 < wavPcm.size(); k += 2) {
                const int v = static_cast<i16>(
                    be ? (wavPcm[k] << 8) | wavPcm[k + 1]
                       : (wavPcm[k + 1] << 8) | wavPcm[k]);
                if (n)
                    sum += std::abs(v - prev);
                prev = v;
                ++n;
            }
            return n > 1 ? sum / double(n - 1) : 0.0;
        };
        const double rBe = rough(true), rLe = rough(false);
        const bool srcBe = rBe <= rLe;
        const u32 rate = bus.sound().rateHz();
        const u32 frames = static_cast<u32>(wavPcm.size() / 4);
        int peak = 0;
        for (size_t k = 0; k + 1 < wavPcm.size(); k += 2) {
            const int v = static_cast<i16>(srcBe
                                               ? (wavPcm[k] << 8) | wavPcm[k + 1]
                                               : (wavPcm[k + 1] << 8) | wavPcm[k]);
            if (std::abs(v) > peak)
                peak = std::abs(v);
        }
        printf("-- wav: %zu bytes, roughness BE %.0f / LE %.0f -> samples "
               "read as %s-endian, peak %d\n",
               wavPcm.size(), rBe, rLe, srcBe ? "BIG" : "little", peak);
        // The mix-starvation meter: 512-frame (2048-byte) slices with any
        // sound in them, against all slices. A healthy continuous stream is
        // ~all content; the session-39 fault read 15 of 861.
        {
            size_t content = 0, total = 0, gap = 0, maxGap = 0;
            for (size_t at = 0; at + 2048 <= wavPcm.size(); at += 2048) {
                ++total;
                bool nz = false;
                for (size_t k = 0; k < 2048; ++k)
                    if (wavPcm[at + k]) { nz = true; break; }
                if (nz) {
                    ++content;
                    gap = 0;
                } else if (++gap > maxGap)
                    maxGap = gap;
            }
            printf("-- wav: content slices %zu of %zu, longest silent gap "
                   "%zu slices\n",
                   content, total, maxGap);
        }
        FILE* f = fopen(wavOut, "wb");
        if (!f) {
            printf("-- wav: cannot write %s\n", wavOut);
        } else {
            const u32 dataBytes = frames * 4u;
            auto w32 = [&](u32 v) { fwrite(&v, 4, 1, f); };
            auto w16 = [&](u16 v) { fwrite(&v, 2, 1, f); };
            fwrite("RIFF", 1, 4, f);
            w32(36u + dataBytes);
            fwrite("WAVEfmt ", 1, 8, f);
            w32(16);
            w16(1);              // PCM
            w16(2);              // stereo
            w32(rate);
            w32(rate * 4u);      // byte rate
            w16(4);              // block align
            w16(16);             // bits
            fwrite("data", 1, 4, f);
            w32(dataBytes);
            // A WAV holds little-endian samples whichever way the guest
            // wrote them.
            for (u32 k = 0; k < dataBytes; k += 2) {
                const u8 a = wavPcm[k], b = wavPcm[k + 1];
                const u8 out[2] = {srcBe ? b : a, srcBe ? a : b};
                fwrite(out, 1, 2, f);
            }
            fclose(f);
            printf("-- wav written: %s (%u frames, %.2f s at %u Hz)\n", wavOut,
                   frames, rate ? double(frames) / double(rate) : 0.0, rate);
        }
    }
    // The serial console census sat INSIDE the ATI report, so hiding the
    // card to force the console onto serial also hid the only evidence of
    // whether the firmware read a byte of it. That turns every injected
    // script into a null result with no positive control: "nothing
    // happened" and "nothing was typed" looked identical.
    printf("-- scc: %llu reads, %llu writes\n",
           static_cast<unsigned long long>(bus.sccReads),
           static_cast<unsigned long long>(bus.sccWrites));
    for (const auto& e : bus.sccOffHist)
        printf("--   scc +%03x read %llu times\n", e.first,
               static_cast<unsigned long long>(e.second));
    if (bus.atiPresent()) {
        // CRTC-aware screen dump: geometry straight from the live CRTC
        // registers, palette from the DAC; the PPM is the machine's
        // first light (user-verified — never self-judged). Decode and
        // pixel conversion come from the machine lib (r128ScanDecode /
        // r128ScanRow) — ONE truth table with capi's opm_screen, because
        // the two copies drifted twice (olive desktop; refused 15/16 bpp).
        const R128Scan scan = r128ScanDecode(bus.ati());
        const u32 gen = bus.ati().peek(0x0050);
        const u32 pitch8 = scan.pitch8;
        const u32 offset = scan.offset;
        const u32 w = scan.w;
        const u32 h = scan.h;
        const u32 fmt = scan.fmt;
        printf("-- ati crtc: gen=%08x %ux%u fmt=%u (%s) pitch8=%u "
               "rowBytes=%u offset=%08x offset_cntl=%08x%s\n",
               gen, w, h, fmt, r128ScanFmtName(fmt), pitch8, scan.rowBytes,
               offset, bus.ati().peek(0x0228),
               scan.tiled ? "  <== TILED: scanned linearly, expect diagonal "
                            "banding + scrambled colour"
                          : "");
        // The hardware cursor as the guest left it. Mac OS draws its pointer
        // with the cursor engine rather than into the framebuffer, so this is
        // the only place a missing pointer shows up at all.
        {
            const auto& ac = bus.ati();
            const u32 cgen = ac.peek(0x0050), posn = ac.peek(0x0264);
            const u32 coff = ac.peek(0x0260) & 0x07FFFFFFu;
            u32 nonBlank = 0;
            for (u32 k = 0; k < 1024u && coff + k < ac.vram.size(); ++k)
                if (ac.vram[coff + k])
                    ++nonBlank;
            printf("-- ati cursor: %s at (%u,%u) off=%08x hv-off=%08x "
                   "clr0=%06x clr1=%06x, %u/1024 non-zero bitmap bytes\n",
                   (cgen & 0x00010000u) ? "ENABLED" : "disabled",
                   (posn >> 16) & 0xFFFFu, posn & 0xFFFFu, coff,
                   ac.peek(0x0268), ac.peek(0x026C) & 0xFFFFFFu,
                   ac.peek(0x0270) & 0xFFFFFFu, nonBlank);
        }
        // Complete and uncapped: every register the guest ever touched. The
        // traffic log is head-and-tail truncated, so it cannot answer "was
        // this register never written" — only this can.
        {
            const auto& ac = bus.ati();
            printf("-- ati register census (off:w/r):\n  ");
            for (const auto& [o, n] : ac.writeCount) {
                const auto rit = ac.readCount.find(o);
                printf(" %03x:w%llu/r%llu", o,
                       static_cast<unsigned long long>(n),
                       static_cast<unsigned long long>(
                           rit == ac.readCount.end() ? 0 : rit->second));
            }
            for (const auto& [o, n] : ac.readCount)
                if (!ac.writeCount.count(o))
                    printf(" %03x:w0/r%llu", o,
                           static_cast<unsigned long long>(n));
            printf("\n");
            // "Is the OS driving the card, or painting with the processor?"
            // The Rage 128's 2D engine is 0x1400-0x17ff (DST_OFFSET 0x1404,
            // SRC_X 0x1414, WAIT_UNTIL 0x1720). Every blit is several writes
            // into that block, so its total against the framebuffer write
            // count settles the question in one line instead of by reading a
            // sixty-entry sample of a truncated ring and hoping.
            u64 engine = 0;
            for (const auto& [o, n] : ac.writeCount)
                if (o >= 0x1400u && o < 0x1800u)
                    engine += n;
            for (const auto& [o, n] : ac.readCount)
                if (o >= 0x1400u && o < 0x1800u)
                    engine += n;
            printf("-- ati 2D engine (0x1400-0x17ff): %llu accesses against "
                   "%llu framebuffer writes\n",
                   static_cast<unsigned long long>(engine),
                   static_cast<unsigned long long>(ac.fbWrites));
            // ⭐ THE REGISTERS THE GUEST READS THAT WE NEVER ANSWER. The card
            // claims its whole aperture, so these can never appear in the
            // unclaimed-access log — they read back zero for ever. A big count
            // here is a driver spinning on hardware this model does not have,
            // which is precisely how the session-31 desktop hang looked from
            // the inside: `ARMQSTimeStampElapsed` in the ATI Graphics
            // Accelerator polling a 64-bit engine timestamp at +0x15e0/+0x15e4.
            // Printed unconditionally, header included, so "nothing to report"
            // and "this section was filtered away" cannot be confused.
            {
                std::vector<std::pair<u64, u32>> ub;
                for (const auto& [o, n] : r128RegReadsUnbacked())
                    ub.push_back({n, o});
                std::sort(ub.rbegin(), ub.rend());
                printf("-- ati registers READ but never answered (always 0), "
                       "busiest first: %zu offset(s)\n",
                       ub.size());
                for (size_t i = 0; i < ub.size() && i < 12; ++i)
                    printf("--   %04x  %llu reads\n", ub[i].second,
                           static_cast<unsigned long long>(ub[i].first));
            }
        }
        // The CCE and bus-master blocks, by VALUE. The census says how often
        // an offset was touched; when a driver's bring-up leaves a state
        // machine half-configured, what matters is what it LEFT THERE — and
        // those writes happen once, early, so they are long gone from any
        // ring by the time the machine misbehaves. Values survive in the
        // register store, and therefore across a snapshot resume, which is
        // the only way to read them at a hang that starts a billion
        // instructions after the write that caused it.
        {
            static const u32 kWatch[] = {0x0704, 0x0708, 0x070C, 0x0710,
                                         0x0714, 0x07B8, 0x07FC, 0x0A10,
                                         0x0A14, 0x0A18, 0x0A50, 0x0A88,
                                         0x0040, 0x0044, 0x15E0, 0x15E4,
                                         0x1720, 0x1740, 0x1748};
            printf("-- ati cce/bus-master state:");
            for (u32 o : kWatch) {
                const char* nm = r128RegName(o);
                printf(" %s(%03x)=%08x", nm ? nm : "?", o,
                       bus.ati().peek(o));
            }
            printf("\n");
        }
        // ⭐ WHAT THE 2D ENGINE ACTUALLY DID. "The driver wrote engine
        // registers" and "the engine drew something" are different claims,
        // and the gap between them is every way a blit can be declined. Each
        // refusal has its own counter so an unimplemented raster op or an
        // unreachable destination is a number here rather than a screen that
        // is wrong for no stated reason.
        {
            const auto& e = r128EngStats();
            printf("-- ati 2D ENGINE: %llu blits (%llu copy, %llu fill), "
                   "%llu pixels; wait_until %llu\n",
                   static_cast<unsigned long long>(e.blits),
                   static_cast<unsigned long long>(e.copies),
                   static_cast<unsigned long long>(e.fills),
                   static_cast<unsigned long long>(e.pixels),
                   static_cast<unsigned long long>(e.waitUntil));
            printf("--   declined: rop %llu, bpp %llu, pitch0 %llu, "
                   "off-vram %llu, agp %llu, clipped %llu, hostdata %llu; "
                   "colour-key ignored %llu; bus-master fetches %llu (%llu bad "
                   "addr), cache flushes %llu\n",
                   static_cast<unsigned long long>(e.ropUnimpl),
                   static_cast<unsigned long long>(e.badBpp),
                   static_cast<unsigned long long>(e.zeroPitch),
                   static_cast<unsigned long long>(e.offVram),
                   static_cast<unsigned long long>(e.agpTarget),
                   static_cast<unsigned long long>(e.clippedOut),
                   static_cast<unsigned long long>(e.hostData),
                   static_cast<unsigned long long>(e.colorCompare),
                   static_cast<unsigned long long>(e.bmFetches),
                   static_cast<unsigned long long>(e.bmBadAddr),
                   static_cast<unsigned long long>(e.cacheFlushes));
            printf("--   cce pio: %llu fifo words, pkt0 %llu, pkt1 %llu, "
                   "pkt2 %llu, pkt3 %llu (unimplemented ops in the map "
                   "below), bad width %llu; indirect "
                   "%llu (%llu words, gart miss %llu, nested %llu)\n",
                   static_cast<unsigned long long>(e.cceWords),
                   static_cast<unsigned long long>(e.ccePkt0),
                   static_cast<unsigned long long>(e.ccePkt1),
                   static_cast<unsigned long long>(e.ccePkt2),
                   static_cast<unsigned long long>(e.ccePkt3),
                   static_cast<unsigned long long>(e.cceBadWidth),
                   static_cast<unsigned long long>(e.cceIndFetch),
                   static_cast<unsigned long long>(e.cceIndWords),
                   static_cast<unsigned long long>(e.cceGartMiss),
                   static_cast<unsigned long long>(e.cceIndNested));
            for (const auto& [op, n] : r128CceP3Skipped())
                printf("--     pkt3 opcode 0x%02x  x%llu\n", op,
                       static_cast<unsigned long long>(n));
            // The ring stays all-zero until a 3D-era driver arms it with
            // PM4_MICRO_CNTL FREERUN; a stall here is a GART hole under a
            // live ring and the first thing to read at a 3D hang.
            if (e.cceRingKicks || e.cceRingStall)
                printf("--   cce ring: %llu kicks, %llu words fetched, "
                       "%llu stalls\n",
                       static_cast<unsigned long long>(e.cceRingKicks),
                       static_cast<unsigned long long>(e.cceRingWords),
                       static_cast<unsigned long long>(e.cceRingStall));
            // ⭐ THE 3D PIPELINE. Printed whenever it was touched — a 3D app
            // that launched and drew nothing is exactly the situation the
            // decline counters exist to explain.
            if (e.tris || e.lines3d || e.points3d || e.prim3dDecline ||
                e.gated3d || e.vtxFetched)
                printf("-- ati 3D: %llu tris (%llu culled, %llu degen), "
                       "%llu lines, %llu points, %llu pixels; vtx %llu "
                       "(%llu gart miss); tex samples %llu (%llu unimpl, "
                       "%llu gart miss); declined prims %llu, gated writes "
                       "%llu, tiled-z %llu\n",
                       static_cast<unsigned long long>(e.tris),
                       static_cast<unsigned long long>(e.triCulled),
                       static_cast<unsigned long long>(e.triDegen),
                       static_cast<unsigned long long>(e.lines3d),
                       static_cast<unsigned long long>(e.points3d),
                       static_cast<unsigned long long>(e.triPixels),
                       static_cast<unsigned long long>(e.vtxFetched),
                       static_cast<unsigned long long>(e.vtxGartMiss),
                       static_cast<unsigned long long>(e.texSamples),
                       static_cast<unsigned long long>(e.texUnimpl),
                       static_cast<unsigned long long>(e.texGartMiss),
                       static_cast<unsigned long long>(e.prim3dDecline),
                       static_cast<unsigned long long>(e.gated3d),
                       static_cast<unsigned long long>(e.zTile));
            // 🧪 THE BOUND TEXTURE'S BYTES, for offline layout analysis.
            //
            // Every texture offset this guest programs carries tiling bits
            // 31:30 = 3 ("stored in a tiled surface") while all four
            // SURFACE range registers read zero. Whether that means the
            // bytes are swizzled is not answerable from any document on
            // the shelf — the RRG redacts the table — but it IS answerable
            // from the DATA: a correctly-interpreted texture is spatially
            // coherent, and a wrongly-interpreted one is not. Dump it and
            // score the candidate layouts outside the emulator.
            {
                const u32 sp = bus.ati().peek(0x1CB8);
                const u32 szl2 = (sp >> 4) & 0xFu;
                const u32 slot = szl2 > 10u ? 10u : szl2;
                const u32 raw = bus.ati().peek(0x1CBC + 4u * slot);
                const u32 tbase = raw & 0x03FFFFF0u;
                const u32 tw = 1u << szl2, th = 1u << ((sp >> 8) & 0xFu);
                const size_t want = size_t(tw) * th * 2u; // 1555 = 2 B
                if (tbase && want && tbase + want <= bus.ati().vram.size()) {
                    FILE* tf = fopen("ati_tex0.bin", "wb");
                    if (tf) {
                        fwrite(&bus.ati().vram[tbase], 1, want, tf);
                        fclose(tf);
                        printf("-- ati tex0 dumped: ati_tex0.bin  base=%08x "
                               "%ux%u fmt=%u (%zu bytes, tiling bits=%u)\n",
                               tbase, tw, th,
                               (bus.ati().peek(0x1CB0) >> 16) & 0xFu, want,
                               (raw >> 30) & 3u);
                    }
                }
            }
            // ⭐ The coordinate convention, per triangle. A triangle that
            // shows one whole texture spans 1.0 in normalised coordinates
            // and `w` in texel ones — 128x apart, so the histogram's mass
            // names the convention outright.
            if (e.triSpanN) {
                printf("-- ati st span (per triangle, max of S/T extent; "
                       "mean bound texture width %llu):\n",
                       (unsigned long long)(e.triSpanTexW / e.triSpanN));
                static const char* kLbl[12] = {
                    "<2^-5", "2^-5", "2^-4", "2^-3", "2^-2", "2^-1",
                    "1..2",  "2..4", "4..8", "8..16", "16..64", ">=64"};
                for (u32 b = 0; b < 12u; ++b)
                    if (e.triSpanBucket[b])
                        printf("--   %-6s %8llu (%.1f%%)\n", kLbl[b],
                               (unsigned long long)e.triSpanBucket[b],
                               100.0 * double(e.triSpanBucket[b]) /
                                   double(e.triSpanN));
            }
            const auto& ru = r128RopUnimplemented();
            if (!ru.empty()) {
                printf("--   raster ops not implemented (rop3:count):");
                for (const auto& [r, n] : ru)
                    printf(" %02x:%llu", r,
                           static_cast<unsigned long long>(n));
                printf("\n");
            }
        }
        // ⛳ THE ENGINE COMMAND STREAM ITSELF (--ati-engine-log N). The census
        // above says WHICH engine registers the guest touched and how often;
        // only this says what it ASKED FOR. A blit is a run of register writes
        // ending in a trigger, so the order and the values are the entire
        // content of the request — and the register written LAST is the one
        // that starts the operation.
        {
            const auto& el = r128EngineLog();
            if (!el.empty()) {
                printf("-- ati ENGINE stream (0x0700-0x1fff): %zu captured, "
                       "%llu dropped between head and tail\n",
                       el.size(),
                       static_cast<unsigned long long>(r128EngineLogDropped()));
                for (const auto& e : el)
                    printf("   %c +%04x %08x %-24s pc=%08x @%llu\n",
                           e.wr ? 'w' : 'r', e.off, e.val,
                           r128RegName(e.off) ? r128RegName(e.off)
                                              : "(unnamed)",
                           e.pc, static_cast<unsigned long long>(e.at));
            }
        }
        // The vertical blank, end to end in one line. The register traffic log
        // is a 4096-entry ring printed head-and-tail, so the nineteen
        // GEN_INT_CNTL writes fall in the trimmed middle every time and
        // "does the driver arm CRTC_VBLANK_INT" was not answerable from it.
        // Report the two registers' live values next to the counts, because
        // blanks generated, blanks the driver had enabled, and acknowledges it
        // wrote are three separate claims — and a raise on a source the
        // OpenPIC has MASKED reaches nobody at all (see the openpic state).
        printf("-- ati vblank: %s period=%llutb GEN_INT_CNTL=%08x "
               "GEN_INT_STATUS=%08x "
               "blanks=%llu enabled=%llu acks=%llu expired=%llu line=%d\n",
               bus.ati().vblEnabled ? "modelled" : "OFF (--no-ati-vbl)",
               static_cast<unsigned long long>(R128Cell::vblPeriodEffective()),
               bus.ati().peek(0x0040), bus.ati().peek(0x0044),
               static_cast<unsigned long long>(bus.ati().vblanks),
               static_cast<unsigned long long>(bus.ati().vblIrqs),
               static_cast<unsigned long long>(bus.ati().vblAcks),
               static_cast<unsigned long long>(R128Cell::vblDropped()),
               bus.ati().irqLine() ? 1 : 0);
        printf("-- ddc: %u starts, %u address matches, %u EDID bytes\n",
               bus.ati().ddcStarts, bus.ati().ddcMatches,
               bus.ati().ddcBytes);
        printf("--   last DDC address byte: 0x%03x\n",
               bus.ati().ddcLastAddr);
        // The waveform was captured all along and never printed, which is
        // why the line assignment kept being guessed instead of read. Each
        // state is the pair of raw register values at the moment either
        // line register was written; decode it the way the slave does, so
        // "617 starts and no address byte" can be seen rather than
        // inferred. A start is SDA falling while SCL is high, so a run of
        // them means the decode is calling ordinary data bits starts.
        {
            const auto& wave = bus.ati().ddcWave;
            printf("-- ddc waveform (%zu states; levels enables -> scl sda"
                   " | edge):\n", wave.size());
            bool pScl = true, pSda = true;
            for (size_t k = 0; k < wave.size(); ++k) {
                const u32 lvl = wave[k].first, en = wave[k].second;
                const bool scl = !(en & 0x00400000u) || (lvl & 0x00400000u);
                const bool sda = !(en & 0x00800000u) || (lvl & 0x00800000u);
                const char* edge = "";
                if (pScl && scl && sda != pSda)
                    edge = sda ? "STOP" : "START";
                else if (scl && !pScl)
                    edge = "sample";
                printf("   %3zu %08x %08x -> scl=%d sda=%d %s\n", k, lvl, en,
                       scl ? 1 : 0, sda ? 1 : 0, edge);
                pScl = scl;
                pSda = sda;
            }
        }
        printf("-- ati bars: reg=%08x fb=%08x rom=%08x io=%08x  "
               "ohci0=%08x ohci1=%08x\n",
               bus.atiRegBar(), bus.atiFbBar(), bus.atiRomBar(),
               bus.atiIoBar(), bus.ohciBar(0), bus.ohciBar(1));
        printf("-- ati framebuffer: %llu writes, span %08x..%08x\n",
               static_cast<unsigned long long>(bus.ati().fbWrites),
               bus.ati().fbWrites ? bus.ati().fbLo : 0u, bus.ati().fbHi);
        // What the guest ACTUALLY painted, measured rather than decoded.
        // A framebuffer's rows are strongly correlated with their
        // neighbours whatever the picture is, so the true row stride is
        // the one that minimises the mean absolute difference between
        // row n and row n+1. Two candidates matter here — 8/16/32 bpp at
        // this width — and reading the CRTC's pixel-width field wrong is
        // indistinguishable by any other means from the guest painting
        // garbage. Also reports each byte lane's distinct-value count:
        // a 32-bpp ARGB surface has a constant alpha lane, a 16-bpp one
        // has no such periodicity.
        if (bus.ati().fbWrites && bus.ati().fbHi > bus.ati().fbLo) {
            const auto& vr = bus.ati().vram;
            // fbLo/fbHi are APERTURE offsets and the aperture is twice the
            // size of VRAM (the second half is the byte-swapped view), so
            // they must be folded before they can index vram — unfolded,
            // every probe below silently skipped its whole loop and printed
            // nothing, which reads exactly like "there is nothing there".
            const size_t lo = bus.ati().fbLo & 0x01FFFFFFu;
            const size_t hi = bus.ati().fbHi & 0x01FFFFFFu;
            const size_t span = hi - lo + 1;
            printf("-- ati paint: %zu bytes touched; w*h*1=%u w*h*2=%u "
                   "w*h*4=%u\n",
                   span, w * h, w * h * 2u, w * h * 4u);
            printf("--   row-stride probe (lower = more likely):\n");
            for (u32 stride : {w, w * 2u, w * 3u, w * 4u, pitch8 * 8u,
                               pitch8 * 16u, pitch8 * 24u, pitch8 * 32u}) {
                if (!stride || lo + 2 * size_t(stride) > vr.size())
                    continue;
                u64 sum = 0, n = 0;
                for (size_t a = lo; a + stride <= hi && a + stride < vr.size();
                     a += 7) { // prime step: no alignment bias
                    const int d = int(vr[a]) - int(vr[a + stride]);
                    sum += u64(d < 0 ? -d : d);
                    ++n;
                }
                printf("--     stride %5u (%u bpp if w=%u): mean|diff| "
                       "%.2f over %llu\n",
                       stride, w ? stride / w : 0, w,
                       n ? double(sum) / double(n) : 0.0,
                       static_cast<unsigned long long>(n));
            }
            u32 laneN[4] = {0, 0, 0, 0};
            for (u32 lane = 0; lane < 4; ++lane) {
                bool laneSeen[256] = {};
                for (size_t a = lo + lane; a <= hi && a < vr.size(); a += 4)
                    laneSeen[vr[a]] = true;
                for (bool s : laneSeen)
                    laneN[lane] += s ? 1u : 0u;
            }
            printf("--   distinct values per byte lane: %u %u %u %u\n",
                   laneN[0], laneN[1], laneN[2], laneN[3]);
        }
        if (scan.enabled && scan.bypp && w >= 64 && w <= 2048 && h >= 64 &&
            h <= 1536) {
            const u32 bypp = scan.bypp;
            const u32 rowBytes = scan.rowBytes;
            FILE* pf = fopen("ati_screen.ppm", "wb");
            if (pf) {
                fprintf(pf, "P6\n%u %u\n255\n", w, h);
                const auto& vr = bus.ati().vram;
                std::vector<u8> rowBgra(size_t(w) * 4u);
                for (u32 y = 0; y < h; ++y) {
                    r128ScanRow(bus.ati(), scan, y, rowBgra.data());
                    for (u32 x = 0; x < w; ++x) {
                        const u8 rgb[3] = {rowBgra[x * 4 + 2],
                                           rowBgra[x * 4 + 1],
                                           rowBgra[x * 4 + 0]};
                        fwrite(rgb, 1, 3, pf);
                    }
                }
                fclose(pf);
                // ⚠⚠ WHETHER THE SCREEN HAS ANYTHING ON IT, stated as a
                // number. `ati paint` counts bytes ever written and SATURATES
                // once one screenful has been painted: it reads 1,261,501 on
                // a blank grey screen and 1,261,505 on the Finder desktop —
                // four bytes apart — and a whole session was called healthy on
                // the strength of it while the user was looking at nothing.
                //
                // Distinct scanlines separate them completely: the recorded
                // desktop has 462 of 480, a uniform screen has 1. Cheap, and
                // it cannot be confused by a counter that only goes up.
                std::set<std::string> rows;
                for (u32 y = 0; y < h; ++y) {
                    std::string r;
                    r.reserve(size_t(w) * bypp);
                    for (u32 x = 0; x < w; ++x) {
                        const size_t o = offset + size_t(y) * rowBytes +
                                         size_t(x) * bypp;
                        for (u32 b = 0; b < bypp; ++b)
                            r.push_back(static_cast<char>(
                                o + b < vr.size() ? vr[o + b] : 0));
                    }
                    rows.insert(std::move(r));
                }
                printf("-- ati screen dumped: ati_screen.ppm — %zu distinct "
                       "scanlines of %u (%s)\n",
                       rows.size(), h,
                       rows.size() <= 2
                           ? "UNIFORM: nothing is being displayed"
                           : "structured");
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
        bus.pic().dumpState();
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
