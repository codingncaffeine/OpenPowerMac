#pragma once
#include "opm/types.hpp"

#include <cstring>
#include <map>
#include <string>
#include <vector>

namespace opm {

struct Cpu;
class SawtoothBus;

// Machine snapshot / restore.
//
// The rule this file exists to enforce: a PARTIAL snapshot is worse than no
// snapshot at all. Restore the CPU and RAM but leave a device's internal
// state stale and the guest reads different bytes than it would have; the
// run then diverges SILENTLY and manufactures evidence that looks real.
// So every mutable field of every modelled device is written here by hand,
// field by field, in a fixed order — never a memcpy of a struct, whose
// padding and layout are a compiler's business rather than ours.
//
// Two things make an incomplete save loud instead of quiet:
//   - each section carries a tag and a byte length, and the reader asserts
//     it consumed exactly that many bytes;
//   - the header carries a layout digest built from the sizeof() of every
//     structure involved, so a snapshot taken by a differently-shaped build
//     is refused rather than misread.
//
// NOT captured, by design: the instruments in g4run (their counters are
// diagnostic and never feed back into the guest), and the CPU's decode-gap
// census. A resumed run's instrument output therefore covers only the
// post-resume window — which is what you want when you are resuming to run
// one experiment.

// Byte sink. Little-endian on the wire; the file is a same-machine artifact
// and the layout digest refuses anything else.
struct SnapWriter {
    std::vector<u8> buf;

    void raw(const void* p, size_t n)
    {
        const u8* b = static_cast<const u8*>(p);
        buf.insert(buf.end(), b, b + n);
    }
    void u8v(u8 v) { buf.push_back(v); }
    void b(bool v) { buf.push_back(v ? 1u : 0u); }
    void u16v(u16 v) { raw(&v, 2); }
    void u32v(u32 v) { raw(&v, 4); }
    void u64v(u64 v) { raw(&v, 8); }
    void i32v(i32 v) { raw(&v, 4); }
    void str(const std::string& s)
    {
        u32v(static_cast<u32>(s.size()));
        raw(s.data(), s.size());
    }
    void bytes(const std::vector<u8>& v)
    {
        u64v(v.size());
        raw(v.data(), v.size());
    }
    template <typename T> void arr32(const T* p, size_t n)
    {
        for (size_t k = 0; k < n; ++k)
            u32v(static_cast<u32>(p[k]));
    }

    // Section framing: begin() remembers where the length goes, end()
    // back-patches it. A reader that walks off the end of a section, or
    // stops short inside one, is reported rather than tolerated.
    size_t begin(const char* tag4)
    {
        raw(tag4, 4);
        const size_t at = buf.size();
        u64v(0);
        return at;
    }
    void end(size_t at)
    {
        const u64 n = static_cast<u64>(buf.size() - at - 8);
        std::memcpy(buf.data() + at, &n, 8);
    }
};

// Byte source. Every read is bounds-checked; the first failure latches and
// every later read returns zero, so a truncated file fails at the check
// rather than by reading garbage.
struct SnapReader {
    const u8* p = nullptr;
    const u8* end = nullptr;
    bool ok = true;
    std::string err;

    SnapReader() = default;
    SnapReader(const u8* data, size_t n) : p(data), end(data + n) {}

    void fail(const std::string& why)
    {
        if (ok) {
            ok = false;
            err = why;
        }
    }
    void raw(void* out, size_t n)
    {
        if (!ok)
            return;
        if (static_cast<size_t>(end - p) < n) {
            fail("truncated");
            return;
        }
        std::memcpy(out, p, n);
        p += n;
    }
    u8 u8v() { u8 v = 0; raw(&v, 1); return v; }
    bool b() { return u8v() != 0; }
    u16 u16v() { u16 v = 0; raw(&v, 2); return v; }
    u32 u32v() { u32 v = 0; raw(&v, 4); return v; }
    u64 u64v() { u64 v = 0; raw(&v, 8); return v; }
    i32 i32v() { i32 v = 0; raw(&v, 4); return v; }
    std::string str()
    {
        const u32 n = u32v();
        if (!ok || static_cast<size_t>(end - p) < n) {
            fail("truncated string");
            return {};
        }
        std::string s(reinterpret_cast<const char*>(p), n);
        p += n;
        return s;
    }
    void bytes(std::vector<u8>& v)
    {
        const u64 n = u64v();
        if (!ok || static_cast<u64>(end - p) < n) {
            fail("truncated blob");
            return;
        }
        v.assign(p, p + n);
        p += static_cast<size_t>(n);
    }
    template <typename T> void arr32(T* out, size_t n)
    {
        for (size_t k = 0; k < n; ++k)
            out[k] = static_cast<T>(u32v());
    }

    // Is the next section this one? Nothing is consumed. This is how a
    // section added after a format was already in the wild stays OPTIONAL:
    // a stream without it reads exactly as before, a stream with it reads
    // it, and the version number is left for changes to what already exists.
    bool nextSectionIs(const char* tag4) const
    {
        return ok && end - p >= 4 && std::memcmp(p, tag4, 4) == 0;
    }
    // Section framing, mirroring SnapWriter. beginSection checks the tag
    // and returns the payload end; endSection asserts the reader landed on
    // it exactly.
    const u8* beginSection(const char* tag4)
    {
        char got[4] = {};
        raw(got, 4);
        if (ok && std::memcmp(got, tag4, 4) != 0) {
            fail(std::string("expected section ") + std::string(tag4, 4) +
                 " but found " + std::string(got, 4));
            return p;
        }
        const u64 n = u64v();
        if (!ok || static_cast<u64>(end - p) < n) {
            fail(std::string("section ") + std::string(tag4, 4) +
                 " runs past end of file");
            return p;
        }
        return p + n;
    }
    void endSection(const char* tag4, const u8* want)
    {
        if (!ok)
            return;
        if (p != want) {
            const long long d = static_cast<long long>(p - want);
            fail(std::string("section ") + std::string(tag4, 4) +
                 " length mismatch: reader finished " + std::to_string(d < 0 ? -d : d) +
                 (d < 0 ? " bytes short of" : " bytes past") + " its declared end");
        }
    }
};

// Harness-level state g4run must carry across a resume. Instrument counters
// are deliberately absent (see the note above); what IS here is everything
// that changes what the GUEST sees: the instruction clock that stamps every
// log and gates every timed injection, the timebase-compression levers, and
// the one-shot diagnostic pokes, which must not re-fire on resume.
struct HarnessState {
    u64 executed = 0;
    u32 fastTb = 0;
    u64 fastTbUntil = ~0ull;
    u32 parkSeen = 0;
    bool parkArmed = false;
    bool ataPoked = false;
    bool emPoked = false;
};

// Whole-machine save/load. saveSnapshot appends to w; loadSnapshot restores
// into an already-constructed machine (same RAM size, same ROM) and returns
// false with a reason in r.err if anything does not line up.
void saveSnapshot(const Cpu& cpu, const SawtoothBus& bus, const HarnessState& h,
                  SnapWriter& w);
bool loadSnapshot(Cpu& cpu, SawtoothBus& bus, HarnessState& h, SnapReader& r);

// File helpers. Both report their own errors to stderr and return false.
bool writeSnapshotFile(const char* path, const std::vector<u8>& blob);
bool readSnapshotFile(const char* path, std::vector<u8>& blob);

// Full-state fingerprint: FNV-1a over a fresh serialization. Two machines
// with the same fingerprint hold the same architected AND device state.
u64 snapshotFingerprint(const Cpu& cpu, const SawtoothBus& bus,
                        const HarnessState& h);

} // namespace opm
