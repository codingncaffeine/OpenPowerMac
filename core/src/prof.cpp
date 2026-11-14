// The sampler thread behind opm/prof.hpp.
//
// It wakes on a fixed period, reads the phase byte and the guest pc, and
// counts. Nothing here touches emulator state, and the emulator never waits on
// it: an instrument that can change the run it is watching is worse than no
// instrument at all, which this project has paid for twice (the arming flush
// that rewrote guest memory, session 24; the census that reordered the L2's
// replacement policy it was watching, same session).
//
// The pc read races with the emulator thread by construction. That is the
// point of sampling — the read is a single aligned word through a volatile
// pointer, so it yields some value the emulator held, which is all a histogram
// needs. It is never fed back into the machine.

#include "opm/prof.hpp"

#include <algorithm>
#include <chrono>
#include <map>
#include <string>
#include <thread>
#include <vector>

namespace opm {
namespace prof {

std::atomic<unsigned char> gPhase{0};

namespace {

std::thread gThread;
std::atomic<bool> gRun{false};
Result gRes;
std::map<u32, u64> gPcHist;
const u32* gPc = nullptr;
constexpr size_t kPcCap = 400000; // bound the census; a boot touches far less

void sampler(unsigned hz)
{
    using clock = std::chrono::steady_clock;
    const auto period = std::chrono::nanoseconds(1000000000ull / hz);
    const auto t0 = clock::now();
    auto next = t0 + period;
    while (gRun.load(std::memory_order_relaxed)) {
        const auto now = clock::now();
        if (now < next) {
            // Sleep only when the wait is long enough that the OS scheduler
            // can honour it; Windows' sleep granularity is milliseconds, so a
            // 250 us period has to be spun. One core out of many is a fair
            // price for not perturbing the thread under measurement.
            if (next - now > std::chrono::milliseconds(2))
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            else
                std::this_thread::yield();
            continue;
        }
        next += period;
        if (next < now) { // fell behind: resync and say so rather than catch up
            const auto behind = now - next;
            gRes.missed +=
                static_cast<u64>(behind / period) + 1;
            next = now + period;
        }
        const unsigned char p = gPhase.load(std::memory_order_relaxed);
        ++gRes.samples;
        if (p < static_cast<unsigned char>(Ph::N))
            ++gRes.phase[p];
        if (gPc) {
            const volatile u32* q = gPc;
            const u32 pc = *q;
            auto it = gPcHist.find(pc);
            if (it != gPcHist.end())
                ++it->second;
            else if (gPcHist.size() < kPcCap)
                gPcHist.emplace(pc, 1);
        }
    }
    gRes.seconds =
        std::chrono::duration<double>(clock::now() - t0).count();
}

} // namespace

const char* name(Ph p)
{
    switch (p) {
    case Ph::Other: return "other";
    case Ph::Fetch: return "fetch";
    case Ph::Xlate: return "translate";
    case Ph::Decode: return "decode";
    case Ph::Exec: return "execute";
    case Ph::Read: return "mem read";
    case Ph::Write: return "mem write";
    case Ph::Tick: return "tick+perfmon";
    case Ph::DevTick: return "device tick";
    case Ph::Irq: return "irq sync";
    case Ph::Instr: return "instrumentation";
    case Ph::Exc: return "exception";
    case Ph::Loop: return "run loop";
    case Ph::Cce: return "cce engine";
    default: return "?";
    }
}

bool start(unsigned hz, const u32* pcRef)
{
    if (gRun.load(std::memory_order_relaxed) || !hz)
        return false;
    gRes = Result{};
    gPcHist.clear();
    gPc = pcRef;
    gRun.store(true, std::memory_order_relaxed);
    gThread = std::thread(sampler, hz);
    return true;
}

void stop()
{
    if (!gRun.load(std::memory_order_relaxed))
        return;
    gRun.store(false, std::memory_order_relaxed);
    if (gThread.joinable())
        gThread.join();
}

bool running() { return gRun.load(std::memory_order_relaxed); }

const Result& result() { return gRes; }

size_t topPcs(u32* pc, u64* hits, size_t cap)
{
    if (!cap)
        return 0;
    std::vector<std::pair<u64, u32>> v;
    v.reserve(gPcHist.size());
    for (const auto& kv : gPcHist)
        v.push_back({kv.second, kv.first});
    const size_t n = std::min(cap, v.size());
    std::partial_sort(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(n),
                      v.end(),
                      [](const std::pair<u64, u32>& a,
                         const std::pair<u64, u32>& b) {
                          return a.first > b.first;
                      });
    for (size_t k = 0; k < n; ++k) {
        pc[k] = v[k].second;
        hits[k] = v[k].first;
    }
    return n;
}

// ---- host-address sampler (see the header) --------------------------------

#if defined(__linux__) && defined(__x86_64__)
} // namespace prof
} // namespace opm

#include <csignal>
#include <cstdio>
#include <ctime>
#include <link.h>
#include <sys/syscall.h>
#include <ucontext.h>
#include <unistd.h>

namespace opm {
namespace prof {
namespace {

// Written by the signal handler, on the one thread the timer is bound to, so
// no two writers ever race; read only after the timer is gone.
constexpr size_t kHostCap = 1u << 21;
std::vector<u64> gHostBuf;
volatile size_t gHostN = 0;
volatile size_t gHostLost = 0;
timer_t gHostTimer;
bool gHostOn = false;

void onHostSample(int, siginfo_t*, void* ctx)
{
    const ucontext_t* uc = static_cast<const ucontext_t*>(ctx);
    const size_t n = gHostN;
    if (n < kHostCap) {
        gHostBuf[n] = static_cast<u64>(uc->uc_mcontext.gregs[REG_RIP]);
        gHostN = n + 1;
    } else {
        gHostLost = gHostLost + 1;
    }
}

struct Module {
    u64 lo, hi, bias;
    std::string name;
};

int collectModules(struct dl_phdr_info* info, size_t, void* out)
{
    auto& v = *static_cast<std::vector<Module>*>(out);
    for (int i = 0; i < info->dlpi_phnum; ++i) {
        const auto& ph = info->dlpi_phdr[i];
        if (ph.p_type != PT_LOAD || !(ph.p_flags & PF_X))
            continue;
        const u64 lo = info->dlpi_addr + ph.p_vaddr;
        v.push_back({lo, lo + ph.p_memsz, info->dlpi_addr,
                     info->dlpi_name ? info->dlpi_name : ""});
    }
    return 0;
}

} // namespace

bool hostStart(unsigned hz)
{
    if (gHostOn || !hz)
        return false;
    gHostBuf.assign(kHostCap, 0);
    gHostN = 0;
    gHostLost = 0;
    struct sigaction sa {};
    sa.sa_sigaction = onHostSample;
    sa.sa_flags = SA_SIGINFO | SA_RESTART;
    sigemptyset(&sa.sa_mask);
    if (sigaction(SIGPROF, &sa, nullptr) != 0)
        return false;
    struct sigevent sev {};
    sev.sigev_notify = SIGEV_THREAD_ID;
    sev.sigev_signo = SIGPROF;
    // glibc names the field through a macro only in its newer releases;
    // musl only through the macro.
#if defined(sigev_notify_thread_id)
    sev.sigev_notify_thread_id = static_cast<pid_t>(syscall(SYS_gettid));
#else
    sev._sigev_un._tid = static_cast<pid_t>(syscall(SYS_gettid));
#endif
    if (timer_create(CLOCK_THREAD_CPUTIME_ID, &sev, &gHostTimer) != 0)
        return false;
    struct itimerspec its {};
    its.it_interval.tv_nsec = static_cast<long>(1000000000ull / hz);
    if (hz == 1) {
        its.it_interval.tv_sec = 1;
        its.it_interval.tv_nsec = 0;
    }
    its.it_value = its.it_interval;
    if (timer_settime(gHostTimer, 0, &its, nullptr) != 0) {
        timer_delete(gHostTimer);
        return false;
    }
    gHostOn = true;
    return true;
}

void hostStop()
{
    if (!gHostOn)
        return;
    timer_delete(gHostTimer);
    signal(SIGPROF, SIG_IGN);
    gHostOn = false;
}

size_t hostDump(const char* path)
{
    const size_t n = gHostN;
    if (!n)
        return 0;
    std::vector<Module> mods;
    dl_iterate_phdr(collectModules, &mods);
    // (module, offset) -> hits. The offset is what addr2line wants: the
    // address as the file on disk numbers it, whatever the loader chose.
    std::map<std::pair<std::string, u64>, u64> hist;
    for (size_t k = 0; k < n; ++k) {
        const u64 a = gHostBuf[k];
        std::string name = "?";
        u64 off = a;
        for (const Module& m : mods)
            if (a >= m.lo && a < m.hi) {
                name = m.name.empty() ? "self" : m.name;
                off = a - m.bias;
                break;
            }
        ++hist[{name, off}];
    }
    FILE* f = std::fopen(path, "w");
    if (!f)
        return 0;
    std::fprintf(f, "# samples %zu lost %zu\n", n,
                 static_cast<size_t>(gHostLost));
    for (const auto& [key, hits] : hist)
        std::fprintf(f, "%s %llx %llu\n", key.first.c_str(),
                     static_cast<unsigned long long>(key.second),
                     static_cast<unsigned long long>(hits));
    std::fclose(f);
    return n;
}

} // namespace prof
} // namespace opm
#else
bool hostStart(unsigned) { return false; }
void hostStop() {}
size_t hostDump(const char*) { return 0; }

} // namespace prof
} // namespace opm
#endif
