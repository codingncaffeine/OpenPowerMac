#pragma once
#include "opm/types.hpp"

#include <vector>

namespace opm {

class Bus;
struct SnapWriter;
struct SnapReader;

// The device end of a DBDMA channel. The engine moves bytes; what they MEAN
// is the device's business, and until 2026-07-31 the channel could only ever
// talk to an ATA cell, which is why the two audio channels could not exist.
class DmaDevice {
public:
    virtual ~DmaDevice() = default;
    // device -> memory (INPUT_MORE / INPUT_LAST). Returns bytes supplied.
    virtual u32 dmaTake(u8* dst, u32 n) = 0;
    // memory -> device (OUTPUT_MORE / OUTPUT_LAST). Returns bytes accepted.
    virtual u32 dmaGive(const u8* src, u32 n) = 0;
    // Whether this device's OUTPUT descriptors carry data it wants.
    virtual bool dmaWriteSink() const = 0;
    // ⚠ WHAT A SHORT TRANSFER MEANS — AND THE TWO KINDS OF DEVICE DISAGREE.
    // A drive's data phase ENDS: it took what it could, the descriptor is
    // finished, residual and all, and the channel moves on. A codec is a
    // STREAM draining at its sample rate: it took what the FIFO had room
    // for, and the rest of that same descriptor is still owed. False is the
    // behaviour of every ATA transfer this machine has ever done.
    virtual bool dmaStreams() const { return false; }
};

// Apple DBDMA channel (KeyLargo lineage), one 0x100 register window:
//   +0x00 channelControl (write: mask in 31:16, values in 15:0)
//   +0x04 channelStatus  (RUN 0x8000, PAUSE 0x4000, FLUSH 0x2000,
//                         WAKE 0x1000, DEAD 0x0800, ACTIVE 0x0400,
//                         BT 0x0100, s7..s0 low byte)
//   +0x0C commandPtrLo · +0x10 interruptSelect · +0x14 branchSelect
//   +0x18 waitSelect
// Registers are little-endian on the bus; the BE-composed access is
// swapped at the edge. The engine walks the 16-byte little-endian
// descriptors at commandPtr: op in word0 bits 31:28 (OUTPUT_MORE=0,
// OUTPUT_LAST=1, INPUT_MORE=2, INPUT_LAST=3, STORE_QUAD=4, LOAD_QUAD=5,
// NOP=6, STOP=7), reqCount in 15:0, word1 = address, word2 = cmdDep (the
// branch target, or STORE_QUAD's literal), word3 receives {xferStatus,
// resCount}. INPUT ops pull real bytes from the attached device's current
// data phase into RAM — the CD's DMA read path.
class DbdmaChannel {
public:
    u32 read(u32 off, u32 len);
    void write(u32 off, u32 v, u32 len);

    void wake(); // device has fresh data or fresh room: resume a standing list

    bool irqLine() const { return irq_; }
    // The interrupt is a LEVEL the OpenPIC samples, and a level nothing ever
    // lowers is an interrupt storm. A driver lowers it by writing the
    // channel's control register, which every DBDMA driver does inside its
    // handler; this is the explicit path for a consumer that latches it.
    void clearIrq() { irq_ = false; }
    // RUN set: the channel is armed, so a read command on this cell is a
    // DMA transfer whatever its opcode says.
    bool running() const { return (status_ & 0x8000u) != 0; }
    // Parked mid-descriptor waiting on a streaming device (see
    // DmaDevice::dmaStreams) — the state a periodic wake exists to clear.
    bool parked() const { return (status_ & 0x8000u) && (status_ & 0x0400u); }
    // The raw registers, for a report. run/parked/irq answer three questions,
    // and a stalled driver always turns on a fourth: WHICH bit it is waiting
    // for. Accessors only — sizeof is unchanged, so snapshots are untouched.
    u32 status() const { return status_; }
    u32 cmdPtr() const { return cmdPtr_; }

    Bus* dmaBus = nullptr;
    DmaDevice* dev = nullptr;
    const u64* stamp = nullptr;
    const u32* pcRef = nullptr;

    struct Ev {
        u64 at;
        u32 kind, a, b; // 0=ctl 1=desc 2=data 3=stop 4=dead 5=storequad
                        // 6=cmdPtr write while ACTIVE 7=branch
    };
    std::vector<Ev> log;
    // The log used to stop recording once it held 2048 events, so it was a
    // record of the FIRST DMA the machine ever did and nothing else — a
    // question about a stall at 4.8 G was being answered with traffic from
    // 0.9 G. Ring it, and gate it the way the ATA traffic log is gated.
    u64 logFrom = 0;

    // Snapshot; the bus and ATA-cell pointers are wired by the machine's
    // constructor and stay valid across a load.
    void snapSave(SnapWriter& w) const;
    void snapLoad(SnapReader& r);

private:
    static u32 swap32(u32 v)
    {
        return (v >> 24) | ((v >> 8) & 0xFF00u) | ((v << 8) & 0xFF0000u) |
               (v << 24);
    }
    void run();
    void note(u32 kind, u32 a, u32 b);

    u32 status_ = 0, cmdPtr_ = 0, intSel_ = 0, brSel_ = 0, waitSel_ = 0;
    // Bytes of the CURRENT descriptor already moved. Zero for every ATA
    // transfer — a drive completes or ends a descriptor, never pauses inside
    // one — and the whole point of the field for a codec.
    u32 xferDone_ = 0;
    bool irq_ = false;
};

// Interrupt bookkeeping for the two audio channels, kept OUTSIDE the class:
// sizeof(DbdmaChannel) is in the snapshot layout digest, so a member would
// orphan every snapshot. Slot 0 = audio out, slot 1 = audio in.
struct DbdmaIrqStats {
    u64 set = 0;     // completions whose i-bits latched irq_
    u64 dropCtl = 0; // a ChannelControl write cleared irq_ while it was
                     // PENDING — a completion the PIC never saw
    u64 pulsed = 0;  // times the machine's edge actually reached the PIC
};
void dbdmaWatchIrq(const DbdmaChannel* out, const DbdmaChannel* in);
const DbdmaIrqStats& dbdmaIrqStats(u32 slot);
void dbdmaNotePulse(const DbdmaChannel* ch);

} // namespace opm
