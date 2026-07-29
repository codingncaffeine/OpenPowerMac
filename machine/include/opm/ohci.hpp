#pragma once
#include "opm/bus.hpp"
#include "opm/types.hpp"

#include <map>
#include <string>
#include <vector>

namespace opm {

struct SnapWriter;
struct SnapReader;

// OHCI host controller (KeyLargo carries two as PCI functions — the
// Sawtooth's usb@18/usb@19). Operational registers per the OpenHCI 1.0
// spec, little-endian on the PCI side: the bus layer hands us the
// BE-composed image of the guest's access, so full words are swapped at
// the edge (guests use lwbrx/stwbrx and see LE values, exactly as on
// real hardware).
//
// Scope: an honest empty-bus controller. Reset, interrupt mask/status
// with write-1 semantics, list pointers as plain state, a frame counter
// that advances in real (TB-derived) milliseconds while operational,
// HCCA frame-number writeback, and a two-port root hub with nothing
// attached. That is what the boot needs: the USB Expert registers the
// controller, seeds the boot-keyboard shim chain, finds zero devices,
// and moves on — the classic no-controller path is the one Apple never
// tested (it sad-macs through a null procPtr in USBShim).
class OhciCell {
public:
    // Register file offsets (dword index = off >> 2).
    u32 read(u32 off, u32 len);
    void write(u32 off, u32 v, u32 len);

    // Frame clock: called with the timebase; one frame per ms of TB
    // (24.94 MHz ≈ 25000 ticks). Advances HcFmNumber, mirrors it into
    // the HCCA, raises SF while in the operational state.
    void tick(u64 tb);

    bool irqLine() const
    {
        return (intEnable_ & 0x80000000u) &&
               (intStatus_ & intEnable_ & 0x7FFFFFFFu);
    }

    // DMA writeback target (guest RAM) for the HCCA mirror.
    u8* ram = nullptr;
    u32 ramSize = 0;
    // This controller is a bus master, and the 60x bus is snooped: the ED
    // and TD lists the host builds are ordinary cached stores, and the
    // buffers it reads back go through the cache. Reaching straight into
    // the RAM array models a machine with no coherency at all — the same
    // defect that made DBDMA read power-on junk for descriptors. See
    // SnoopSink in bus.hpp.
    SnoopSink* snoop = nullptr;
    void snoopRd(u32 pa, u32 len) const
    {
        if (snoop)
            snoop->snoopRead(pa, len);
    }
    void snoopWr(u32 pa, u32 len) const
    {
        if (snoop)
            snoop->snoopWrite(pa, len);
    }

    struct Ev {
        u64 at;
        u32 off, val, pc;
    };
    std::vector<Ev> log;          // write traffic
    std::map<u32, u64> readCount; // steady-state poll census per offset
    const u64* stamp = nullptr;
    const u32* pcRef = nullptr;

    // Snapshot; the DMA writeback pointer into guest RAM is re-wired by
    // the bus after a load rather than stored.
    void snapSave(SnapWriter& w) const;
    void snapLoad(SnapReader& r);

    // A USB HID boot keyboard on root-hub port 1.
    //
    // Open Firmware's console input is the keyboard package, not the serial
    // port: with a display present it blocks in $call-method "read" on
    // stdin, and injecting into the SCC changes nothing. The ROM carries
    // usb-kbd FCode, so a keyboard that enumerates is what unblocks the
    // prompt — and the desktop needs one regardless.
    //
    // The port reported nothing attached and no list was ever walked, so
    // the controller had nothing to find and nothing to do.
    void typeAscii(const std::string& s); // queue keystrokes
    bool keyboardIdle() const { return pending_.empty() && !reportDue_; }
    u64 setupsSeen = 0, inTds = 0, reportsSent = 0; // census

    // Two controllers, one device each: usb@8 carries the boot keyboard and
    // usb@9 the boot mouse. Both devices on one cell would need per-device
    // control-transfer state (each has its own address and its own EP0); a
    // device per controller is how the machine is wired anyway.
    enum class Hid { Keyboard, Mouse };
    void setHid(Hid k)
    {
        hid_ = k;
        reportLen_ = (k == Hid::Mouse) ? 3u : 8u;
    }
    Hid hid() const { return hid_; }
    // Boot-protocol mouse report: buttons, then signed X and Y deltas.
    void moveMouse(int dx, int dy, u8 buttons);

private:
    // Control-transfer state for the single attached device.
    u8 setup_[8] = {};
    std::vector<u8> reply_;  // data still owed to an IN transfer
    u8 address_ = 0;         // address assigned by SET_ADDRESS
    u8 pendingAddress_ = 0;
    std::vector<u8> pending_; // queued HID reports, reportLen_ bytes each
    bool reportDue_ = false;  // a key is down and its release still owed
    Hid hid_ = Hid::Keyboard;
    u32 reportLen_ = 8;

    u32 ldLe(u32 pa) const;
    void stLe(u32 pa, u32 v);
    void buildDescriptor();
    u32 runList(u32 head, bool control);

public:
    // The list walk is the one part of this cell nothing outside can see:
    // "the host queued nothing", "the descriptors read back as junk" and "we
    // walked them wrong" all produce the same silence on the register bus,
    // which is a host polling HcInterruptStatus forever. Record what the
    // walker actually reads.
    struct Walk {
        u32 kind; // 0 = ED, 1 = TD
        u32 a, b, c, d, e;
    };
    std::vector<Walk> walkLog;
    u32 walkMax = 400;

private:
    // Returns false when the endpoint NAKed and the descriptor must stay
    // where it is; true when the TD was retired, with `next` set to the rest
    // of the chain -- which is legitimately 0 at the end of a transfer.
    // Returning the next pointer alone cannot express that difference, and
    // conflating them leaves a retired TD sitting at HeadP to be run again.
    bool doTd(u32 ed0, u32 td, u32& next);
    void retire(u32 td, u32 cc);

    static u32 swap32(u32 v)
    {
        return (v >> 24) | ((v >> 8) & 0xFF00u) | ((v << 8) & 0xFF0000u) |
               (v << 24);
    }

    u32 regRead(u32 idx);
    void regWrite(u32 idx, u32 v);

    // Operational state (values in device-native LE-logical form).
    u32 control_ = 0;            // HcControl: HCFS in bits 7:6
    u32 cmdStatus_ = 0;          // HcCommandStatus (HCR self-clears)
    u32 intStatus_ = 0;          // HcInterruptStatus (W1C)
    u32 intEnable_ = 0;          // W1S via 0x10, W1C via 0x14
    u32 hcca_ = 0;
    u32 periodCurrent_ = 0, ctrlHead_ = 0, ctrlCurrent_ = 0;
    u32 bulkHead_ = 0, bulkCurrent_ = 0, doneHead_ = 0;
    u32 fmInterval_ = 0x00002EDFu; // FI reset value
    u32 fmNumber_ = 0;             // 16-bit counter
    u32 periodicStart_ = 0;
    u32 lsThreshold_ = 0x0628u;
    u32 rhDescA_ = 0x02000002u; // POTPGT=2, NDP=2, ports powered on
    u32 rhDescB_ = 0;
    u32 rhStatus_ = 0;
    // Port 1 reports a low-speed device attached from power-on: CCS,
    // LSDA and the connect-status change. Port 2 is empty.
    u32 rhPort_[2] = {0x00010201u, 0};
    // Port reset is not instantaneous, and it must not be here either: the
    // driver writes SetPortReset and only THEN arms what it waits on. A reset
    // that finished inside the store would have its completion wiped by the
    // very next "clear the status I am about to wait on" — the same ordering
    // inversion that cost this project two days on the ATA cell. USB gives
    // reset 10 ms, which is 10 frames.
    static constexpr u32 kResetFrames = 10;
    u32 portReset_[2] = {0, 0}; // frames remaining; 0 = idle
    // OHCI 1.0 §6.5: setting ANY root-hub change bit raises RHSC in
    // HcInterruptStatus. Without it a host that polls HcInterruptStatus after
    // a port reset — which is exactly what Open Firmware does, 43,530 times —
    // waits forever for a bit the controller never sets.
    void rhSignal(u32 before, u32 after)
    {
        if ((after & ~before) & 0x001F0000u) intStatus_ |= 0x40u; // RHSC
    }
    u64 lastFrameTb_ = 0;
};

} // namespace opm
