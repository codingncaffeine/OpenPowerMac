#include "opm/openpic.hpp"

#include <cstdio>
#include <cstring>

namespace opm {

// The window is little-endian; the bus layer hands us big-endian-composed
// values from byte storage, so full-word accesses arrive byte-reversed
// relative to the guest's stwbrx/lwbrx view. Normalize here: registers
// are kept in guest-native form and swapped at the edge for the BE
// composition path.
static u32 swap32(u32 v)
{
    return (v >> 24) | ((v >> 8) & 0xFF00u) | ((v << 8) & 0xFF0000u) |
           (v << 24);
}

// Harness instrumentation, not controller state — see the note in the header.
static int gPicTrace = 0;

void OpenPic::setTrace(int n) { gPicTrace = n; }

u32 OpenPic::read(u32 off, u32 len)
{
    u32 native = 0;
    if (off - 0x10000u < kSources * 0x20u) {
        const u32 src = (off - 0x10000u) >> 5;
        native = (off & 0x10u) ? dest_[src] : vp_[src];
        if (pending_[src] && !(off & 0x10u))
            native |= 0x40000000u; // ACTIVITY
    } else if (off - 0x1000u < 0x1000u) {
        native = global_[(off - 0x1000u) >> 2];
    } else if (off == 0x200A0u) {
        native = iack();
    } else if (off == 0x20080u) {
        native = taskPri_;
    }
    return len == 4 ? swap32(native) : native;
}

void OpenPic::write(u32 off, u32 v, u32 len)
{
    const u32 native = len == 4 ? swap32(v) : v;
    if (off - 0x10000u < kSources * 0x20u) {
        const u32 src = (off - 0x10000u) >> 5;
        if (off & 0x10u)
            dest_[src] = native;
        else {
            vp_[src] = native & ~0x40000000u;
            if (log.size() < 512)
                log.push_back({stamp ? *stamp : 0, 'v',
                               (src << 24) | (native & 0x80FFFFFFu)});
            // unmasking a held level line re-pends it
            if (!(native & 0x80000000u) && line_[src])
                pending_[src] = true;
        }
        return;
    }
    if (off - 0x1000u < 0x1000u) {
        global_[(off - 0x1000u) >> 2] = native;
        if (off == 0x10E0u)
            spurious_ = native & 0xFFu;
        return;
    }
    if (off == 0x20080u) {
        taskPri_ = native & 0xFu;
        return;
    }
    if (off == 0x200B0u) { // EOI
        if (log.size() < 512)
            log.push_back({stamp ? *stamp : 0, 'e', 0});
        if (gPicTrace > 0) {
            --gPicTrace;
            printf("PIC eoi    inService=%d @%llu\n", inService_,
                   static_cast<unsigned long long>(stamp ? *stamp : 0));
            fflush(stdout);
        }
        inService_ = -1;
        return;
    }
    // Anything that reaches here was written into the controller's window
    // and decoded by nothing. With every source stuck MASKED, the write
    // that should clear bit 31 is either landing in one of these holes or
    // is not being issued at all, and a raise count cannot tell the two
    // apart.
    if (unclaimed.size() < 256)
        unclaimed.push_back({stamp ? *stamp : 0, off, native});
}

void OpenPic::setLine(u32 src, bool level)
{
    if (src >= kSources)
        return;
    if (level && !line_[src]) {
        pending_[src] = true;
        raiseCount[src < 64 ? src : 63]++;
    }
    line_[src] = level;
    if (!level && (vp_[src] & 0x00400000u))
        pending_[src] = false; // level-sensitive follows the line down
}

// ⚠ THE MACHINE LOOP ASKS THIS QUESTION ONCE PER EMULATED INSTRUCTION, through
// cpuLine(). Written as a plain 64-source sweep it was the single largest item
// in the profile — "irq sync" 26.2% of the whole emulator, more than three
// times the instruction handlers — and the answer is "nothing is pending"
// almost every time it is asked.
//
// So look at the pending flags eight at a time. They are 64 contiguous bools,
// an all-zero octet answers for eight sources at once, and the result is
// identical to the sweep. memcpy, not a cast: reading a bool array through a
// u64* is an aliasing violation, and every compiler here turns this into the
// single load it looks like.
int OpenPic::highestPending() const
{
    static_assert(sizeof(bool) == 1, "the octet scan assumes 1-byte bools");
    static_assert(kSources % 8 == 0, "the octet scan assumes a multiple of 8");
    int best = -1;
    u32 bestPri = taskPri_;
    for (u32 base = 0; base < kSources; base += 8) {
        u64 oct;
        std::memcpy(&oct, pending_ + base, sizeof oct);
        if (!oct)
            continue;
        for (u32 s = base; s < base + 8; ++s) {
            if (!pending_[s] || (vp_[s] & 0x80000000u))
                continue;
            const u32 pri = (vp_[s] >> 16) & 0xFu;
            if (pri > bestPri) {
                bestPri = pri;
                best = static_cast<int>(s);
            }
        }
    }
    return best;
}

bool OpenPic::cpuLine() const
{
    return inService_ < 0 && highestPending() >= 0;
}

u32 OpenPic::iack()
{
    const int s = highestPending();
    if (s < 0)
        return spurious_;
    if (!(vp_[s] & 0x00400000u) || !line_[s])
        pending_[s] = false; // edge consumed; level re-pends while high
    inService_ = s;
    if (log.size() < 512)
        log.push_back({stamp ? *stamp : 0, 'a', static_cast<u32>(s)});
    if (gPicTrace > 0) {
        --gPicTrace;
        printf("PIC iack   src=%d vec=%02x line=%d pending=%d @%llu\n", s,
               vp_[s] & 0xFFu, line_[s] ? 1 : 0, pending_[s] ? 1 : 0,
               static_cast<unsigned long long>(stamp ? *stamp : 0));
        fflush(stdout);
    }
    return vp_[s] & 0xFFu;
}


// Diagnostic: the delivery decision, spelled out. highestPending() skips a
// source when it is masked (vp bit 31) or when its priority does not exceed
// taskPri_, and cpuLine() additionally requires that nothing is still in
// service. Any of those three silently turns a raised line into no
// interrupt at all, and none of them are visible from a raise count.
void OpenPic::dumpState() const
{
    printf("-- openpic state: taskPri=%u inService=%d cpuLine=%d\n",
           taskPri_, inService_, cpuLine() ? 1 : 0);
    for (u32 s = 0; s < kSources; ++s) {
        if (!line_[s] && !pending_[s] && !vp_[s])
            continue;
        printf("   src %2u vp=%08x pri=%u %s line=%d pending=%d dest=%08x\n",
               s, vp_[s], (vp_[s] >> 16) & 0xFu,
               (vp_[s] & 0x80000000u) ? "MASKED" : "enabled",
               line_[s] ? 1 : 0, pending_[s] ? 1 : 0, dest_[s]);
    }
    printf("--   unclaimed writes into the controller window (%zu):\n",
           unclaimed.size());
    for (size_t k = 0; k < unclaimed.size() && k < 24; ++k)
        printf("     +%05x <- %08x @%llu\n", unclaimed[k].off,
               unclaimed[k].val,
               static_cast<unsigned long long>(unclaimed[k].at));
}

} // namespace opm
