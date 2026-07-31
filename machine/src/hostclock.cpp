// The one piece of opm/hostclock.hpp that has to know what operating system
// this is: asking for a short wait and finding out what the asking cost.
//
// It is out of line rather than in the header because the header is included
// by the C API, by g4run and by pace.hpp — which is to say by most of the
// emulator — and <windows.h> is not a thing to pull into that many translation
// units. The measurement of the wait is portable and lives here too, next to
// the thing it measures.

// ⚠ BEFORE ANY SYSTEM HEADER. CreateWaitableTimerExW is Vista and later, and
// MinGW's headers hide it behind _WIN32_WINNT — where the default varies by
// toolchain version, so a build that works here fails on the CI box with the
// function simply undeclared. MSVC's SDK defaults to its own maximum and does
// not care either way.
#if defined(_WIN32) && !defined(_WIN32_WINNT)
#define _WIN32_WINNT 0x0601
#endif

#include "opm/hostclock.hpp"

#if defined(_WIN32)
// ⚠ GUARDED, BOTH OF THEM. MinGW's libstdc++ defines NOMINMAX itself, as `1`
// rather than as nothing, and a redefinition with a different token sequence
// is a warning — which is an error on that compiler. MSVC's SDK does not, so
// this is exactly the class of thing the second compiler exists to catch.
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
// Windows 10 1803 added the flag and older SDKs do not define it. Defining it
// by hand costs nothing: a kernel that does not know it fails the call, which
// is the same path an older SDK would have taken anyway, so there is one
// fallback below rather than two.
#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002
#endif
#else
#include <ctime>
#endif

#include <chrono>

namespace opm {

HostWait::~HostWait()
{
#if defined(_WIN32)
    if (h_)
        CloseHandle(static_cast<HANDLE>(h_));
#endif
    h_ = nullptr;
}

u64 HostWait::wait(u64 ns)
{
    // ⭐ ASK FOR LESS THAN IS WANTED, BY WHAT ASKING COSTS. Every mechanism
    // here comes back late — about 0.45 ms on this host even with the
    // high-resolution timer — and a pacer that ignored that would hand the
    // guest a deadline it had already missed, every time. The correction is
    // learned below rather than written here as a constant, because a constant
    // measured on one machine is wrong on the next one.
    //
    // This is also the whole of the degradation story. If the wait mechanism
    // turns out to be coarse — a kernel with no high-resolution timer, a
    // loaded machine — the learned overshoot grows until it swallows every
    // ask, this returns 0 without waiting, and the loop goes back to spinning.
    // ⭐ That is the RIGHT failure: spinning costs a host core, while wedging
    // the machine on a 15 ms scheduler quantum would run its USB frames at
    // 64 Hz instead of 1000 and read as a sluggish pointer, not as a bad sleep.
    if (ns <= overshootNs + kMinAskNs) {
        ++skipped;
        // ⛔⛔ DECAY HERE TOO, OR THE ESTIMATE IS A ONE-WAY RATCHET — and it
        // was, and it cost this session a measurement that read like the fix
        // not working. The learned cost is only ever revised at the bottom of
        // this function, and this path returns before it: once one scheduling
        // hitch pushes the estimate past the deadlines this machine actually
        // has, EVERY call skips, no wait ever happens, and there is nothing
        // left to revise it back down. The wait switches itself off for the
        // rest of the run.
        //
        // ⭐ It is not hypothetical and the numbers are worth keeping: this
        // machine's nearest deadline is the USB frame at almost exactly 1 ms,
        // the estimate settled at 1.00 ms, and a 96 s run at the Finder read
        // **5,661,470 skips and 15% idle** where the same code over a short
        // window read 52%. A degradation path has to be able to come back.
        overshootNs -= overshootNs / 64;
        return 0;
    }
    const u64 ask = ns - overshootNs;

    const auto t0 = std::chrono::steady_clock::now();
#if defined(_WIN32)
    if (!tried_) {
        tried_ = true;
        h_ = CreateWaitableTimerExW(nullptr, nullptr,
                                    CREATE_WAITABLE_TIMER_HIGH_RESOLUTION,
                                    TIMER_ALL_ACCESS);
        // Pre-1803, or a kernel that declines the flag. An ordinary waitable
        // timer still works; it is merely coarse, and the learned overshoot
        // above is what notices and stands down.
        if (!h_)
            h_ = CreateWaitableTimerExW(nullptr, nullptr, 0, TIMER_ALL_ACCESS);
    }
    if (h_) {
        LARGE_INTEGER due;
        // Negative is relative, and the unit is 100 ns. `ask` is at least
        // kMinAskNs, so this cannot round down to the zero that would mean
        // "signal immediately".
        due.QuadPart = -static_cast<LONGLONG>(ask / 100u);
        if (SetWaitableTimer(static_cast<HANDLE>(h_), &due, 0, nullptr, nullptr,
                             FALSE))
            WaitForSingleObject(static_cast<HANDLE>(h_), INFINITE);
    }
#else
    struct timespec req;
    req.tv_sec = static_cast<time_t>(ask / 1000000000ull);
    req.tv_nsec = static_cast<long>(ask % 1000000000ull);
    // A signal that cuts this short is not worth resuming for: waking early is
    // free here, because the caller is a loop and simply asks again.
    nanosleep(&req, nullptr);
#endif
    const u64 actual = static_cast<u64>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - t0)
            .count());

    // ⚠ RISE FASTER THAN IT FALLS, BUT DO NOT LATCH THE WORST CASE. Erring
    // high is the safe direction — asking for too little wakes early, which is
    // free, while asking for too much overshoots the guest's deadline and
    // delays a device — so the rise is a quarter and the fall a sixteenth.
    //
    // ⛔ It used to take the maximum outright, and that was wrong twice over:
    // this is a HOST-CONTENTION number as much as a timer cost, so one moment
    // of the machine being busy elsewhere latched a value that had nothing to
    // do with the timer, and — with the skip path not decaying — it could
    // never come back. The measured intrinsic cost here is 0.25–0.53 ms
    // (ask 250/500/1000 us -> 0.50/1.03/1.44 ms), so anything near a
    // millisecond is contention being mistaken for cost.
    const u64 over = actual > ask ? actual - ask : 0;
    if (over > overshootNs)
        overshootNs += (over - overshootNs) / 4;
    else
        overshootNs -= (overshootNs - over) / 16;

    ++waits;
    waitedNs += actual;
    return actual;
}

} // namespace opm
