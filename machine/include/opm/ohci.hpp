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
    // Read-only views for the end-of-run report. An interrupt only reaches the
    // CPU when the guest has ARMED it, so "SF raised 1.17M times" is a storm
    // only if the driver actually enabled SF -- otherwise those assertions go
    // nowhere and the half-million InterruptStatus round-trips are something
    // else entirely. Member functions, so sizeof (and every snapshot) is
    // unaffected.
    u32 intEnableView() const { return intEnable_; }
    u32 intStatusView() const { return intStatus_; }
    u32 controlView() const { return control_; }
    u32 rhDescAView() const { return rhDescA_; }
    u32 portView(u32 n) const { return rhPort_[n & 1u]; }

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
    // `log` is capped at 2048 AND snapshotted, so it fills with Open
    // Firmware's traffic and silently drops everything the OS does after a
    // resume — which is how "the OS never writes an OHCI register" was
    // concluded when in fact it writes plenty. These two are uncapped and
    // deliberately NOT snapshotted, so a resumed run measures the OS era
    // alone.
    std::map<u32, u64> writeCount;
    // The root-hub registers (0x48-0x58) are where enumeration lives or
    // dies, and they are touched tens of times, not thousands: log every
    // access to them in full.
    struct RhEv {
        u64 at;
        u32 off, val, pc;
        bool wr;
    };
    std::vector<RhEv> rhLog;
    // Uncapped in practice -- these registers are touched tens of times, not
    // thousands -- but a guest that polls a port forever must not be able to
    // exhaust memory. Deliberately NOT snapshotted.
    size_t rhMax = 50000;
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
    // Keystrokes with a modifier held: bit 0 LeftCtrl, 1 LeftShift, 2 LeftAlt,
    // 3 LeftGUI (Command). Command-O opens the Finder selection, which is how
    // the machine is drivable while the pointer is still dead.
    void typeChord(u8 mod, const std::string& s);
    // A REAL key event, which is what a keyboard is. typeAscii above turns
    // text into keystrokes and is right for a script; it is wrong for a
    // person, because a keyboard does not send text. It sends a usage code
    // going down and the same code coming up, and the GUEST decides what
    // character that is — which is why Backspace, Tab, the arrows, Delete,
    // the function keys and every modifier are unreachable through the text
    // path no matter how the host spells them.
    //
    // HID usages 0xE0-0xE7 are the modifiers and live in byte 0 of the boot
    // report as a bitmap (LeftCtrl, LeftShift, LeftAlt, LeftGUI, then the
    // right-hand four); everything else occupies one of the six key slots in
    // bytes 2-7. The cell holds the CURRENT state and queues the whole
    // report on every change, so chords and held modifiers behave the way
    // the guest expects rather than as isolated characters.
    void keyEvent(u8 usage, bool down);
    bool keyboardIdle() const { return pending_.empty() && !reportDue_; }
    // What is queued for the interrupt IN endpoint, one reportLen_ chunk per
    // report. Exposed because a wrong modifier byte or a wrong key slot is
    // still a well-formed report — the guest accepts it and the only symptom
    // is that typing does nothing — so the bytes need asserting somewhere
    // cheaper than a whole boot.
    const std::vector<u8>& queuedReports() const { return pending_; }
    u64 setupsSeen = 0, inTds = 0, reportsSent = 0; // census
    // WHEN the guest polled, not just how often. "18,548 polls, 0 reports"
    // reads as a broken delivery path, but it is the same number you get when
    // every poll happened before anything was injected — and those are
    // opposite bugs.
    u64 firstInTd = 0, lastInTd = 0;
    // Which branch an interrupt IN took. "Polled but nothing delivered" has
    // three causes that look identical from outside: the ED names endpoint
    // zero so it fell into the control path, the report queue was empty, or
    // the TD offered no buffer to write into.
    u64 inEp0 = 0, nakEmpty = 0, noBuffer = 0;

    // WHICH HcInterruptStatus bit the guest is servicing. An OS-era census of
    // half a million InterruptStatus round-trips says the driver is stuck in
    // an interrupt it cannot get rid of, but not which one -- and SF (raised
    // every frame, ordinary) and RHSC (a port event the driver failed to
    // consume) call for opposite fixes. Counts every assertion, not just
    // 0->1 edges: a bit the driver never clears would otherwise show up once.
    // Not snapshotted, so a resumed run measures the OS era alone.
    u64 intRaised[32] = {};
    u64 intCleared[32] = {};
    // WHAT the guest arms and disarms, by value. The OS ends a boot with
    // intEnable = 8000003b -- RHSC (bit 6) CLEAR -- so the root-hub connect
    // interrupt reaches nobody and the hub driver never resets the port it
    // just powered. Counts alone cannot say whether RHSC was never armed or
    // armed and then disarmed, and those are different bugs.
    std::map<u32, u64> intEnWrites;  // via 0x10 (write-1-to-SET)
    std::map<u32, u64> intDisWrites; // via 0x14 (write-1-to-CLEAR)
    // Frames, and when. `--fast-tb` accelerates the timebase this clock is
    // derived from, so a frame can land every few thousand instructions
    // instead of every ~900,000 -- which shrinks every frame-counted device
    // delay inside the window the driver expected to be long. That is the
    // ordering inversion that has already cost this project the ATA cell and
    // the port reset, so measure the rate rather than assuming it.
    u64 frames = 0, firstFrameAt = 0, lastFrameAt = 0;

    // DIAGNOSTIC (--ohci-ndp), not machine truth: force the number of
    // downstream ports the root hub reports. Mac OS publishes NumPorts 0 for
    // both root hubs while we report NDP 2; driving NDP to a distinctive
    // value says whether the OS is reading this field at all. 0x02000002 is
    // byte-palindromic, so it cannot distinguish a correct read from a
    // swapped one -- an odd count can.
    void setNdp(u32 n)
    {
        rhDescA_ = (rhDescA_ & ~0x000000FFu) | (n & 0xFFu);
    }

    // Two controllers, one device each: usb@8 carries the boot keyboard and
    // usb@9 the boot mouse. Both devices on one cell would need per-device
    // control-transfer state (each has its own address and its own EP0); a
    // device per controller is how the machine is wired anyway.
    // Root-hub port-power model. OFF BY DEFAULT, and the default is the one
    // that boots.
    //
    //  false -- the pre-session-16 model: port 1 reports a device from
    //    power-on, SetPortPower/ClearPortPower are a bit with no side effects,
    //    and enable/reset act on a STATIC "is this port populated" predicate.
    //    Mac OS's USB Family finds nothing it has to wait for and moves on, so
    //    the boot reaches the desktop and the cache dialog.
    //
    //  true (--ohci-port-power) -- the honest model, measured from a real
    //    boot: an unpowered port reports CCS 0, power-up schedules a real
    //    connect kConnectFrames later, and enable/reset act on the live CCS
    //    bit. The OS gets MEASURABLY FURTHER with this -- it powers both
    //    ports and reads the hub descriptor (pc=ffdfacb0/ffe01e50/ffe02220) --
    //    but it never arms RHSC (intEnable settles at 8000003b, bit 6 clear),
    //    so the connect interrupt reaches nobody, it never issues the
    //    SetPortReset that Open Firmware does issue, and the boot stalls
    //    before the welcome screen.
    //
    // The stall above was session 17's POTPGT/per-port-power bug and is fixed.
    // DEFAULT SINCE SESSION 18, because it is now the only model that reaches
    // a usable machine: the OS enumerates both HID devices, keyboard keys and
    // mouse buttons work, and a bare CR dismisses the startup cache dialog so
    // Mac OS 9.1 boots through to the Finder desktop. With this off there is no
    // USB input at all, so the dialog can never be dismissed and the desktop is
    // unreachable -- the "default must be a machine that boots" rule now points
    // the other way. `--no-ohci-port-power` restores the pre-session-16 model.
    bool livePortPower = true;
    void setLivePortPower(bool on)
    {
        livePortPower = on;
        rhPort_[0] = on ? 0u : 0x00010201u;
        rhPort_[1] = 0u;
        portPower_[0] = portPower_[1] = 0;
        portReset_[0] = portReset_[1] = 0;
        if (on) {
            // PSM (bit 8): power is switched PER PORT, as on real hardware.
            rhDescA_ = 0x02000102u;
            // PortPowerControlMask for ports 1 and 2 (bits 17,18): both have
            // their own switch, so a global power command does not attach them.
            rhDescB_ = 0x00060000u;
        } else {
            // Ganged switching, the pre-session-16 descriptors. Symmetric on
            // purpose: this setter is what --no-ohci-port-power uses to put
            // the old machine back, so it has to undo every field it sets.
            rhDescA_ = 0x02000002u;
            rhDescB_ = 0;
        }
    }

    enum class Hid { Keyboard, Mouse };
    void setHid(Hid k)
    {
        hid_ = k;
        reportLen_ = (k == Hid::Mouse) ? 4u : 8u; // mouse: buttons,X,Y,wheel
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
    // A mouse ACCUMULATES motion between polls and reports the sum, rather
    // than queueing one packet per movement: the host polls at its own
    // cadence, and if it is busy for 50 ms the next poll should return one
    // larger delta, not a backlog of tiny ones. Travel beyond a signed byte
    // stays here and goes out on the following poll, so a fast flick becomes
    // several packets instead of one saturated lie.
    int accDx_ = 0, accDy_ = 0;
    u8 buttons_ = 0, sentButtons_ = 0;
    // The boot keyboard's live state, as keyEvent() maintains it: the
    // modifier bitmap and the six key slots a boot report carries.
    u8 keyMod_ = 0;
    u8 keySlots_[6] = {};

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
    // These must agree with livePortPower's default, which is now TRUE: the
    // live model REQUIRES per-port switching, and with PSM clear a global
    // power command attaches the device ~22 M instructions before the hub
    // driver exists, which is the bug session 17 spent its day on. The capi
    // never calls setLivePortPower(), so the shell gets exactly these values
    // -- a mismatch here ships a machine that silently reverts to ganged.
    u32 rhDescA_ = 0x02000102u; // POTPGT=2, PSM (per-port switching), NDP=2
    u32 rhDescB_ = 0x00060000u; // PPCM: ports 1 and 2 have their own switch
    u32 rhStatus_ = 0;
    // A port starts UNPOWERED, and an unpowered port sees nothing: with no
    // VBUS there is no pull-up to sense, so CCS reads 0 no matter what is
    // plugged in. Reporting a permanent connection instead is what cost the
    // mouse. Open Firmware powers the ports, enumerates, and then — measured,
    // at 1,686,375,933 — writes ClearPortPower to both and clears every
    // change bit before handing the machine over. Mac OS's OHCIUIM then
    // powers the ports back up and waits to be told a device arrived. A port
    // that had simply kept CCS asserted throughout looked like a device that
    // was always there and never changed, so the hub driver had nothing to
    // act on and never enumerated.
    // Both ports start UNPOWERED, which is the live model's reset state: with
    // no VBUS there is nothing to sense, so CCS reads 0 until the host powers
    // the port and the power-good delay elapses. `setLivePortPower(false)`
    // puts back the pre-session-16 value (0x00010201: CCS|PES|PPS|LSDA plus
    // the connect change on port 1), which must be applied through that
    // setter -- assigning livePortPower alone would leave this reset state.
    u32 rhPort_[2] = {0, 0};
    // Port 1 carries this controller's HID device; port 2 is genuinely empty.
    bool portPopulated(u32 n) const { return n == 0; }
    // Power on, power off. Powering down drops the connection along with
    // everything that depends on it (enable, suspend, reset) and reports the
    // disconnect as a change; powering up starts the power-on-to-power-good
    // delay, after which the device is detected.
    void portPowerSet(u32 n, bool on)
    {
        const u32 before = rhPort_[n];
        if (on) {
            if (before & 0x00000100u) return; // already powered
            rhPort_[n] |= 0x00000100u;
            // POWER-ON TO POWER-GOOD IS POTPGT, AND THE DRIVER CHOSE IT.
            //
            // HcRhDescriptorA bits 31:24 are POTPGT in 2 ms units, and Mac OS
            // programs 0x0a = 20 ms. A hardcoded 3 frames delivered the
            // connect ~5,000 instructions after SetPortPower -- while the hub
            // driver was still walking its ports powering them up, before it
            // had queued the status-change transfer that the event is supposed
            // to complete. So it saw the port, and had nowhere to put it.
            // Honouring POTPGT lands the connect after the driver is ready,
            // which is the whole reason the field exists.
            const u32 potpgt = ((rhDescA_ >> 24) & 0xFFu) * 2u;
            portPower_[n] =
                portPopulated(n) ? (potpgt ? potpgt : kConnectFrames) : 0;
        } else {
            rhPort_[n] &= ~0x00000317u; // PPS|LSDA|PRS|PSS|PES|CCS
            portPower_[n] = 0;
            portReset_[n] = 0;
            if (before & 1u) rhPort_[n] |= 0x00010000u; // CSC: it went away
        }
        rhSignal(before, rhPort_[n]);
    }
    // Neither the connect nor the reset may complete inside the store that
    // starts it: the driver writes the command and only THEN arms what it
    // waits on, so a completion delivered synchronously is wiped by the very
    // next "clear the status I am about to wait on". That ordering inversion
    // has now cost this project the ATA cell, the port reset, and the mouse.
    // USB gives reset 10 ms; power-on-to-power-good is POTPGT, and the driver
    // programs 0x0a = 20 ms, so landing the connect a few frames in is well
    // inside its wait.
    static constexpr u32 kResetFrames = 10;
    static constexpr u32 kConnectFrames = 3;
    u32 portReset_[2] = {0, 0}; // frames remaining; 0 = idle
    u32 portPower_[2] = {0, 0}; // frames to power-good; 0 = idle
    // OHCI 1.0 §6.5: setting ANY root-hub change bit raises RHSC in
    // HcInterruptStatus. Without it a host that polls HcInterruptStatus after
    // a port reset — which is exactly what Open Firmware does, 43,530 times —
    // waits forever for a bit the controller never sets.
    void rhSignal(u32 before, u32 after)
    {
        if ((after & ~before) & 0x001F0000u) raise(0x40u); // RHSC
    }
    // Every assertion of HcInterruptStatus goes through here so the per-bit
    // census cannot drift out of step with the register.
    void raise(u32 bits)
    {
        intStatus_ |= bits;
        for (u32 b = 0; b < 32; ++b)
            if (bits & (1u << b)) intRaised[b]++;
    }
    u64 lastFrameTb_ = 0;
};

} // namespace opm
