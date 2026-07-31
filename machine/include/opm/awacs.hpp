#pragma once
#include "opm/dbdma.hpp"
#include "opm/types.hpp"

#include <vector>

namespace opm {

struct SnapWriter;
struct SnapReader;

// AWACS / Screamer sound codec at mac-io +0x14000, and the sink both audio
// DBDMA channels drain into.
//
// This was four bytes of storage until 2026-07-31 — the codec status word,
// seeded so the ROM's revision probe found a Screamer. A first-touch census
// of the mac-io window over a full boot said the rest of the block was being
// driven anyway, by two different programs:
//
//   f3014000  @2,907,062      pc=fff859c8   Sound Control   w=9
//   f3014010  @2,907,100      pc=fff85a4c   Codec Control   w=242
//   f3014020  @2,907,080      pc=fff85a80   Codec Status    r=50
//   f3014040  @2,907,072      pc=fff859f0   Byte Swapping   w=1
//   f3014050  @3,428,000,490  pc=0046a1e4   Frame Count     r=4
//
// `fff8xxxx` is the boot ROM; `0046xxxx` is guest RAM, i.e. Mac OS's own
// sound driver, polling a frame counter that never moved.
//
// REGISTERS ARE LITTLE-ENDIAN ON THE BUS, like DBDMA's. That is not an
// inference: the ROM reads Sound Control with `lwbrx r2,0,r1` (fff859c4) and
// writes it with `stwbrx r2,0,r1` (fff859d4), and the `rlwimi r2,r0,8,21,23`
// between them puts the rate field at bits 10:8 of the byte-reversed word,
// exactly where the PCI-Mac register map says MASK_RATE lives.
//
// ⚠ Byte Swapping at +0x40 is the one register the ROM stores with a PLAIN
// `stwx` (fff859ec, value 0x80000001). Its little-endian image is therefore
// 0x01000080. Nothing in this machine reads it: the sample byte order is
// decided by what the captured PCM actually sounds like, and the report
// prints both interpretations' roughness so the question is settled by
// measurement rather than by this register.
class AwacsCell : public DmaDevice {
public:
    // The six documented PCI-Mac registers. The rest of the 0x1000 window is
    // left to the KeyLargo store — nothing in a full boot touches it, and
    // claiming address space nobody uses only hides the next surprise.
    static constexpr u32 kRegBase = 0x14000u, kRegSize = 0x60u;
    static constexpr u32 kSoundCtl = 0x00, kCodecCtl = 0x10, kCodecStat = 0x20,
                         kClipCount = 0x30, kByteSwap = 0x40,
                         kFrameCount = 0x50;

    // 16-bit stereo: four bytes to the frame, at every rate the codec has.
    static constexpr u32 kFrameBytes = 4;
    // The timebase is the bus clock divided by four, and the bus is 100 MHz.
    static constexpr u64 kTbHz = 25000000ull;
    // "The DMA hardware buffer size is set to be 0x2000 bytes" (PCI-Mac
    // AWACS notes). It is how far ahead of the play cursor the channel may
    // run, and it is what stops an output RING — which is what a sound
    // driver's descriptor list is — from being swallowed whole inside a
    // single guest instruction.
    static constexpr u32 kFifoBytes = 0x2000;

    u32 read(u32 off, u32 len) const;
    void write(u32 off, u32 v, u32 len);

    // The machine's clock. Every duration here is denominated in it, so
    // --fast-tb scales the codec with everything else and the guest measures
    // the ratio this machine actually runs at.
    void noteTb(u64 tb) { nowTb_ = tb; }
    // ⏱ WHEN A PARKED CHANNEL SHOULD BE LOOKED AT AGAIN, one per direction.
    //
    // ⚠ It is the CHANNEL that knows it is parked, not the codec: a channel
    // stalls the moment a descriptor is bigger than the FIFO, which is the
    // normal case and happens on a call where the codec accepted bytes and
    // therefore did NOT think it was waiting for anything. Asking the codec
    // "did you return zero" left the ROM's chime spinning on a list that
    // nothing ever resumed. See SawtoothBus::soundDueTb.
    //
    // The deadline is a HALF-EMPTY FIFO rather than a byte of room, because
    // a wake per byte is a wake per service call: refilling from half to
    // full moves 4 KB, which is 23 ms of guest audio.
    u64 outDueTb() const { return dueTb(playTb_); }
    u64 inDueTb() const { return dueTb(recTb_); }

    // --- DmaDevice: the device end of the two audio channels -------------
    u32 dmaTake(u8* dst, u32 n) override;       // +0x8900, capture
    u32 dmaGive(const u8* src, u32 n) override; // +0x8800, playback
    bool dmaWriteSink() const override { return true; }
    // A codec is a STREAM. A short transfer means "the FIFO is full for
    // now", not "the data phase ended" — see DmaDevice::dmaStreams.
    bool dmaStreams() const override { return true; }

    // --- host side -------------------------------------------------------
    // Sample rate the Sound Control register currently selects.
    u32 rateHz() const;
    // PCM handed to the codec, 16-bit stereo at rateHz(), in the byte order
    // the guest wrote it. Drained by the application through opm_audio; what
    // is not drained is dropped oldest-first rather than allowed to grow.
    size_t drain(u8* out, size_t cap);
    size_t queued() const { return ring_.size(); }
    // A second, independent tap for a capture tool: every byte the codec
    // accepts is appended here as well, so draining for playback cannot
    // steal a recording. Off unless a pointer is set.
    std::vector<u8>* capture = nullptr;
    size_t captureCap = 64u << 20;

    // Census, for the report.
    u64 bytesPlayed() const { return played_; }
    u64 bytesCaptured() const { return recorded_; }
    u64 underruns() const { return underruns_; }
    u32 soundCtl() const { return soundCtl_; }
    u32 codecCtl() const { return codecCtl_; }
    u32 byteSwap() const { return byteSwap_; }
    u64 frameCount() const;
    u32 codecReg(u32 i) const { return codecRegs_[i & 7u]; }

    void snapSave(SnapWriter& w) const;
    void snapLoad(SnapReader& r);

private:
    static u32 swap32(u32 v)
    {
        return (v >> 24) | ((v >> 8) & 0xFF00u) | ((v << 8) & 0xFF0000u) |
               (v << 24);
    }
    u64 tbForBytes(u64 b) const { return b * kTbHz / (rateHz() * kFrameBytes); }
    u64 bytesForTb(u64 t) const { return t * (rateHz() * kFrameBytes) / kTbHz; }
    // Common half of both directions: how many whole frames the FIFO has
    // room for, given a cursor that says when everything queued so far will
    // have been consumed.
    u32 credit(u64& cursorTb);
    u64 dueTb(u64 cursorTb) const
    {
        const u64 halfTb = tbForBytes(kFifoBytes / 2);
        return cursorTb > halfTb ? cursorTb - halfTb : 0ull;
    }

    u32 soundCtl_ = 0, codecCtl_ = 0, clipCount_ = 0, byteSwap_ = 0;
    // The codec's own register file, addressed through Codec Control's
    // 15:12 address field. Held so a read-back of a written register is not
    // a lie, and so the report can say what the guest asked for.
    u32 codecRegs_[8] = {};
    // The play cursor: the timebase at which everything handed to the codec
    // so far will have finished. The record cursor is the same idea for the
    // input channel.
    u64 playTb_ = 0, recTb_ = 0;
    // Frame Count is DERIVED FROM THE TIMEBASE, like the VIA clock, the USB
    // frame clock and the KeyLargo timer. A counter that counts service
    // calls is the session-26 bug again: it measures the emulator instead of
    // the machine. The epoch is carried so the count survives Open
    // Firmware's mid-boot PMU reset, which restarts the timebase at zero.
    u64 frameTb_ = 0, frameVal_ = 0;
    u64 nowTb_ = 0;
    u64 played_ = 0, recorded_ = 0, underruns_ = 0;
    std::vector<u8> ring_;
    static constexpr size_t kRingCap = 1u << 20; // ~6 s at 44.1 kHz stereo
};

} // namespace opm
