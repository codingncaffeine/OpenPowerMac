#include "opm/awacs.hpp"

#include <cstring>

namespace opm {

// AWACS/Screamer sample rates, in Sound Control's rate field order: 44100
// divided by 1, 1.5, 2, 2.5, 3, 4, 5 and 6. The guest selects 0 (44.1 kHz).
static constexpr u32 kRates[8] = {44100, 29400, 22050, 17640,
                                  14700, 11025, 8820,  7350};

u32 AwacsCell::rateHz() const { return kRates[(soundCtl_ >> 8) & 7u]; }

u64 AwacsCell::frameCount() const
{
    // The timebase restarts at zero on a processor reset (Open Firmware runs
    // one mid-boot) and is replaced wholesale by a snapshot restore, so "now"
    // can legitimately be earlier than the epoch. An unsigned subtract there
    // would hand the guest a count near 2^64 — clamp, as klTimerCount does.
    const u64 dt = nowTb_ > frameTb_ ? nowTb_ - frameTb_ : 0ull;
    return frameVal_ + dt * rateHz() / kTbHz;
}

u32 AwacsCell::read(u32 off, u32 len) const
{
    u32 native = 0;
    switch (off & ~3u) {
    case kSoundCtl: native = soundCtl_; break;
    case kCodecCtl:
        // Bit 24 is the codec's "command pending" lock: the driver sets it
        // with the command and waits for the codec to clear it. This codec
        // takes a command in the cycle it is written, so it is never set on
        // the way out. Answering the written value back would park any
        // driver that polls it.
        native = codecCtl_ & ~0x01000000u;
        break;
    case kCodecStat:
        // The value the ROM's revision probe has always been answered with:
        // READY (bit 22) and revision 3 in bits 15:12, which is the Screamer
        // class. It was seeded into the KeyLargo store byte by byte before
        // this cell existed; it is the same number.
        native = 0x00403100u;
        break;
    case kClipCount: native = clipCount_; break;
    case kByteSwap: native = byteSwap_; break;
    case kFrameCount: native = static_cast<u32>(frameCount()); break;
    default: break;
    }
    // Little-endian register file behind a big-endian bus: the guest reads
    // these with lwbrx, so the bus has to carry the reversed image.
    if (len == 4)
        return swap32(native);
    u32 r = 0;
    for (u32 k = 0; k < len; ++k)
        r = (r << 8) | ((native >> (8 * ((off + k) & 3u))) & 0xFFu);
    return r;
}

void AwacsCell::write(u32 off, u32 v, u32 len)
{
    u32 native;
    if (len == 4)
        native = swap32(v);
    else {
        // Lane-merge against the current image, so a byte store touches one
        // byte of the little-endian register rather than the whole word.
        native = swap32(read(off & ~3u, 4));
        for (u32 k = 0; k < len; ++k) {
            const u32 lane = (off + k) & 3u;
            native = (native & ~(0xFFu << (8 * lane))) |
                     (((v >> (8 * (len - 1 - k))) & 0xFFu) << (8 * lane));
        }
    }
    switch (off & ~3u) {
    case kSoundCtl: {
        const u32 old = soundCtl_;
        soundCtl_ = native;
        // A rate change re-denominates both cursors and the frame counter,
        // all three of which are expressed in bytes-per-timebase at the OLD
        // rate. Re-anchor rather than let a stale conversion run on.
        if (((old ^ native) & 0x0700u) != 0) {
            frameVal_ = frameCount();
            frameTb_ = nowTb_;
            playTb_ = recTb_ = nowTb_;
        }
        break;
    }
    case kCodecCtl:
        codecCtl_ = native;
        // Address in bits 15:12, twelve bits of data below it.
        codecRegs_[(native >> 12) & 7u] = native & 0xFFFu;
        break;
    case kClipCount: clipCount_ = native; break;
    case kByteSwap: byteSwap_ = native; break;
    default: break; // Codec Status and Frame Count are read-only
    }
}

// How many whole frames may be handed to the codec right now: everything the
// hardware FIFO has room for ahead of the cursor. A cursor behind "now" means
// the stream ran dry, so it restarts at now — the alternative is a codec that
// silently owes the guest the whole silent gap and then accepts a burst.
u32 AwacsCell::credit(u64& cursorTb)
{
    if (cursorTb < nowTb_)
        cursorTb = nowTb_;
    const u64 fifoTb = tbForBytes(kFifoBytes);
    const u64 ahead = cursorTb - nowTb_;
    if (ahead >= fifoTb)
        return 0;
    u64 can = bytesForTb(fifoTb - ahead);
    if (can > kFifoBytes)
        can = kFifoBytes;
    return static_cast<u32>(can) & ~(kFrameBytes - 1u);
}

u32 AwacsCell::dmaGive(const u8* src, u32 n)
{
    if (playTb_ < nowTb_ && played_)
        ++underruns_; // the guest did not refill in time; a real codec clicks
    u32 can = credit(playTb_);
    if (can > n)
        can = n;
    if (can == 0)
        return 0;
    // The ring is bounded and the application drains it; a run nobody is
    // listening to must not grow without limit. Drop oldest, which is what a
    // late consumer would hear anyway.
    if (ring_.size() + can > kRingCap) {
        const size_t drop = ring_.size() + can - kRingCap;
        ring_.erase(ring_.begin(),
                    ring_.begin() + static_cast<ptrdiff_t>(
                                        drop < ring_.size() ? drop : ring_.size()));
    }
    ring_.insert(ring_.end(), src, src + can);
    if (capture && capture->size() + can <= captureCap)
        capture->insert(capture->end(), src, src + can);
    playTb_ += tbForBytes(can);
    played_ += can;
    return can;
}

u32 AwacsCell::dmaTake(u8* dst, u32 n)
{
    // There is no microphone on this machine, and a capture stream that
    // never advances is worse than one that returns silence: the driver
    // parks on a descriptor that is never completed. Supply zeros, at the
    // rate the codec would actually sample them.
    u32 can = credit(recTb_);
    if (can > n)
        can = n;
    if (can == 0)
        return 0;
    std::memset(dst, 0, can);
    recTb_ += tbForBytes(can);
    recorded_ += can;
    return can;
}

size_t AwacsCell::drain(u8* out, size_t cap)
{
    const size_t n = ring_.size() < cap ? ring_.size() : cap;
    if (n) {
        std::memcpy(out, ring_.data(), n);
        ring_.erase(ring_.begin(), ring_.begin() + static_cast<ptrdiff_t>(n));
    }
    return n;
}

} // namespace opm
