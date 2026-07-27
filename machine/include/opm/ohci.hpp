#pragma once
#include "opm/types.hpp"

#include <map>
#include <vector>

namespace opm {

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

    struct Ev {
        u64 at;
        u32 off, val, pc;
    };
    std::vector<Ev> log;          // write traffic
    std::map<u32, u64> readCount; // steady-state poll census per offset
    const u64* stamp = nullptr;
    const u32* pcRef = nullptr;

private:
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
    u32 rhPort_[2] = {0, 0}; // no CCS ever: nothing attached
    u64 lastFrameTb_ = 0;
};

} // namespace opm
