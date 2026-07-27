#include "opm/openpic.hpp"

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
        inService_ = -1;
        return;
    }
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

int OpenPic::highestPending() const
{
    int best = -1;
    u32 bestPri = taskPri_;
    for (u32 s = 0; s < kSources; ++s) {
        if (!pending_[s] || (vp_[s] & 0x80000000u))
            continue;
        const u32 pri = (vp_[s] >> 16) & 0xFu;
        if (pri > bestPri) {
            bestPri = pri;
            best = static_cast<int>(s);
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
    return vp_[s] & 0xFFu;
}

} // namespace opm
