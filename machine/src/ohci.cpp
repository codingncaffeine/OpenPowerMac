#include "opm/ohci.hpp"
#include <cstring>

namespace opm {

// One USB frame per millisecond of guest time; the timebase runs at
// bus/4 ≈ 24.94 MHz, so 25000 ticks is a frame within the tolerance any
// driver measures against.
static constexpr u64 kTbPerFrame = 25000;

u32 OhciCell::regRead(u32 idx)
{
    switch (idx) {
    case 0x00 >> 2: return 0x00000010u; // HcRevision: OHCI 1.0
    case 0x04 >> 2: return control_;
    case 0x08 >> 2: return cmdStatus_;
    case 0x0C >> 2: return intStatus_;
    case 0x10 >> 2:
    case 0x14 >> 2: return intEnable_;
    case 0x18 >> 2: return hcca_;
    case 0x1C >> 2: return periodCurrent_;
    case 0x20 >> 2: return ctrlHead_;
    case 0x24 >> 2: return ctrlCurrent_;
    case 0x28 >> 2: return bulkHead_;
    case 0x2C >> 2: return bulkCurrent_;
    case 0x30 >> 2: return doneHead_;
    case 0x34 >> 2: return fmInterval_;
    case 0x38 >> 2: return fmInterval_ & 0x3FFFu; // FmRemaining: coarse
    case 0x3C >> 2: return fmNumber_ & 0xFFFFu;
    case 0x40 >> 2: return periodicStart_;
    case 0x44 >> 2: return lsThreshold_;
    case 0x48 >> 2: return rhDescA_;
    case 0x4C >> 2: return rhDescB_;
    case 0x50 >> 2: return rhStatus_;
    case 0x54 >> 2: return rhPort_[0];
    case 0x58 >> 2: return rhPort_[1];
    default: return 0;
    }
}

void OhciCell::regWrite(u32 idx, u32 v)
{
    switch (idx) {
    case 0x04 >> 2:
        control_ = v;
        break;
    case 0x08 >> 2:
        // HCR (bit 0): software reset completes instantly — the spec
        // allows 10 µs; the bit reads back clear. Reset drops the
        // controller into UsbSuspend with interrupts disarmed.
        if (v & 1u) {
            control_ = (control_ & ~0xC0u) | 0xC0u; // UsbSuspend
            intStatus_ = 0;
            intEnable_ = 0;
            hcca_ = 0;
            fmNumber_ = 0;
            // A change bit that survived the reset still owes the host an
            // RHSC: port 1 reports its connect-status change from power-on,
            // and a host that resets the controller and then waits on
            // HcInterruptStatus must still be told the port has news.
            if ((rhPort_[0] | rhPort_[1]) & 0x001F0000u) raise(0x40u);
        }
        cmdStatus_ = v & ~1u & 0x0000000Eu; // OCR/BLF/CLF latch, no lists
        break;
    case 0x0C >> 2:
        // Write-1-to-clear. Census the bits the driver is actually acking:
        // half a million round-trips here says it is servicing something it
        // cannot shake off, and only the per-bit split says which.
        for (u32 b = 0; b < 32; ++b)
            if ((v & intStatus_) & (1u << b)) intCleared[b]++;
        intStatus_ &= ~v;
        break;
    case 0x10 >> 2:
        intEnWrites[v]++;
        intEnable_ |= v; // write-1-to-set (MIE included)
        break;
    case 0x14 >> 2:
        intDisWrites[v]++;
        intEnable_ &= ~v; // write-1-to-clear
        break;
    case 0x18 >> 2: hcca_ = v & 0xFFFFFF00u; break;
    case 0x20 >> 2: ctrlHead_ = v & ~0xFu; break;
    case 0x24 >> 2: ctrlCurrent_ = v & ~0xFu; break;
    case 0x28 >> 2: bulkHead_ = v & ~0xFu; break;
    case 0x2C >> 2: bulkCurrent_ = v & ~0xFu; break;
    case 0x34 >> 2: fmInterval_ = v; break;
    case 0x40 >> 2: periodicStart_ = v & 0x3FFFu; break;
    case 0x44 >> 2: lsThreshold_ = v & 0xFFFu; break;
    case 0x48 >> 2:
        // RhDescriptorA is mostly hardwired; NDP read-only.
        //
        // With the live power model, the power-switching CAPABILITY bits
        // (PSM/NPS/DT/OCPM/NOCP, 12:8) are read-only too, because they
        // describe the BOARD and not a driver preference -- QEMU makes the
        // whole register read-only (OHCI_RHA_RW_MASK is 0). Mac OS's UIM
        // writes 0a001002, which would clear PSM and silently turn a per-port
        // hub back into a ganged one; the global-power write that follows
        // would then fire the port connect ~22M instructions before the hub
        // driver is ready to enumerate it, which is exactly the bug.
        if (livePortPower)
            rhDescA_ = (rhDescA_ & 0x00001FFFu) | (v & 0xFF000000u);
        else
            rhDescA_ = (rhDescA_ & 0x000000FFu) | (v & 0xFFFFFF00u);
        break;
    case 0x4C >> 2:
        // HcRhDescriptorB is BOARD WIRING -- DeviceRemovable and
        // PortPowerControlMask say how the ports are physically attached and
        // switched -- so with the live power model it is read-only. Open
        // Firmware writes 0 here during its own init (pc=ff80b6b4), which
        // wiped our per-port mask and handed the OS a ganged hub again; the
        // global power write in the UIM's init then attached the device ~22M
        // instructions before the hub driver could enumerate it.
        if (!livePortPower)
            rhDescB_ = v;
        break;
    case 0x50 >> 2:
        // HcRhStatus. Bit 16 written is SetGlobalPower and bit 0 is
        // ClearGlobalPower; with HcRhDescriptorB's PortPowerControlMask
        // clear the hub is ganged, so both reach every port. Mac OS's hub
        // driver uses this path as well as the per-port bits, and treating
        // global power as decoration meant the ports never came back up.
        // (Only when the live power model is on -- otherwise global power is
        // accepted and forgotten, and ports report powered through PPS.)
        // A global power command only reaches ports the board wires to the
        // global switch: HcRhDescriptorB's PortPowerControlMask bit set means
        // "this port has its own switch" (OHCI 1.0 §7.4.2), and with PSM=1
        // that is how the hub driver's per-port SetPortPower becomes the thing
        // that actually attaches the device -- at the moment it is ready.
        if (livePortPower) {
            for (u32 n = 0; n < 2; ++n) {
                if (rhDescB_ & (0x00020000u << n)) continue; // per-port switch
                if (v & 0x00010000u) portPowerSet(n, true);
                if (v & 0x00000001u) portPowerSet(n, false);
            }
        }
        // DRWE/OCIC: accepted, nothing downstream reads them.
        break;
    case 0x54 >> 2:
    case 0x58 >> 2: {
        const u32 n = idx - (0x54 >> 2);
        // Power first: everything below is conditional on the port being
        // connected, and connection is conditional on power.
        if (livePortPower) {
            if (v & 0x00000200u) portPowerSet(n, false); // ClearPortPower
            if (v & 0x00000100u) portPowerSet(n, true);  // SetPortPower
        }
        u32& p = rhPort_[n];
        const u32 before = p;
        if (!livePortPower) {
            // Power is a plain bit with no side effects.
            if (v & 0x00000100u) p |= 0x00000100u;
            if (v & 0x00000200u) p &= ~0x00000100u;
        }
        // "Set if connected": with nothing attached these commands do
        // nothing but report a change, which is how a driver learns the
        // port is empty rather than waiting out a timeout. With the live
        // model that is the CCS bit; without it, a static predicate.
        const bool connected =
            livePortPower ? ((p & 1u) != 0) : portPopulated(n);
        if (v & 0x00000002u) { // SetPortEnable
            if (connected)
                p |= 0x00000002u;
            else
                p |= 0x00010000u; // CSC: enable attempt on a dead port
        }
        if (v & 0x00000001u) // ClearPortEnable
            p &= ~0x00000002u;
        if (v & 0x00000010u) { // SetPortReset
            if (connected) {
                // Reset is IN PROGRESS: report PRS and finish it on a later
                // frame (see kResetFrames).
                p |= 0x00000010u;
                portReset_[n] = kResetFrames;
            } else {
                p |= 0x00010000u;
            }
        }
        p &= ~(v & 0x001F0000u); // W1C of the change bits
        rhSignal(before, p);
        break;
    }
    default: break;
    }
}

u32 OhciCell::read(u32 off, u32 len)
{
    readCount[off & ~3u]++;
    const u32 native = regRead(off >> 2);
    if ((off & ~3u) >= 0x48u && (off & ~3u) <= 0x58u && rhLog.size() < rhMax)
        rhLog.push_back({stamp ? *stamp : 0, off & ~3u, native,
                         pcRef ? *pcRef : 0, false});
    if (len == 4)
        return swap32(native);
    // sub-word: serve the addressed LE byte lanes, BE-composed
    u32 r = 0;
    for (u32 k = 0; k < len; ++k)
        r = (r << 8) | ((native >> (8 * ((off + k) & 3u))) & 0xFFu);
    return r;
}

void OhciCell::write(u32 off, u32 v, u32 len)
{
    u32 native;
    if (len == 4)
        native = swap32(v);
    else {
        native = regRead(off >> 2);
        for (u32 k = 0; k < len; ++k) {
            const u32 lane = (off + k) & 3u;
            native = (native & ~(0xFFu << (8 * lane))) |
                     (((v >> (8 * (len - 1 - k))) & 0xFFu) << (8 * lane));
        }
    }
    writeCount[off & ~3u]++;
    if (log.size() < 2048)
        log.push_back({stamp ? *stamp : 0, off, native,
                       pcRef ? *pcRef : 0});
    if ((off & ~3u) >= 0x48u && (off & ~3u) <= 0x58u && rhLog.size() < rhMax)
        rhLog.push_back({stamp ? *stamp : 0, off & ~3u, native,
                         pcRef ? *pcRef : 0, true});
    regWrite(off >> 2, native);
}

void OhciCell::tick(u64 tb)
{
    if (tb - lastFrameTb_ < kTbPerFrame)
        return;
    lastFrameTb_ = tb;
    const u64 now = stamp ? *stamp : 0;
    if (!frames++) firstFrameAt = now;
    lastFrameAt = now;

    // THE ROOT HUB IS LIVE WHENEVER THE CONTROLLER HAS POWER, and these two
    // timers must run OUTSIDE the operational gate below.
    //
    // A host powers a port and polls its status long before it sets HCFS to
    // UsbOperational -- Open Firmware does exactly that to find the boot
    // keyboard. Counting power-on-to-power-good only while operational meant
    // the countdown never advanced, the connect event never arrived, and the
    // port read empty forever: OF found no keyboard, blocked in its console
    // read on stdin, and the machine never reached the desktop at all.
    // Frame GENERATION is what the operational state gates (HcFmNumber, SF,
    // the ED lists); root-hub electrical behaviour is not.
    for (u32 n = 0; n < 2; ++n) {
        if (!portPower_[n] || --portPower_[n]) continue;
        const u32 before = rhPort_[n];
        rhPort_[n] |= 0x00000001u   // CCS
                    | 0x00000200u   // LSDA: the HID devices are low speed
                    | 0x00010000u;  // CSC
        rhSignal(before, rhPort_[n]);
    }
    // Finish any port reset that is running. The port comes out enabled, with
    // PRSC set, and RHSC raised so a host polling HcInterruptStatus is told to
    // go and look at the port.
    for (u32 n = 0; n < 2; ++n) {
        if (!portReset_[n] || --portReset_[n]) continue;
        const u32 before = rhPort_[n];
        rhPort_[n] &= ~0x00000010u;              // PRS clear: reset done
        rhPort_[n] |= 0x00000002u | 0x00100000u; // PES + PRSC
        rhSignal(before, rhPort_[n]);
    }

    if (((control_ >> 6) & 3u) != 2u)
        return; // not operational: no frames, no list service
    fmNumber_ = (fmNumber_ + 1) & 0xFFFFu;
    if (hcca_ && ram && hcca_ + 0x84u <= ramSize) {
        // HccaFrameNumber: 16-bit little-endian at HCCA+0x80, pad zero.
        snoopWr(hcca_ + 0x80u, 4);
        ram[hcca_ + 0x80] = static_cast<u8>(fmNumber_);
        ram[hcca_ + 0x81] = static_cast<u8>(fmNumber_ >> 8);
        ram[hcca_ + 0x82] = 0;
        ram[hcca_ + 0x83] = 0;
    }
    raise(0x00000004u); // SF

    // Service the lists once per frame. Nothing walked them before, so a
    // host could enumerate forever and never get a descriptor back.
    if (control_ & 0x10u) // CLE: control list enable
        runList(ctrlHead_, true);
    if (control_ & 0x20u) // BLE: bulk list enable
        runList(bulkHead_, false);
    if ((control_ & 0x04u) && hcca_ && ram) {
        // PLE: the periodic list hangs off the HCCA interrupt table; a
        // boot keyboard lives on exactly one of its 32 entries.
        const u32 slot = (fmNumber_ & 31u) * 4u;
        if (hcca_ + slot + 4 <= ramSize)
            runList(ldLe(hcca_ + slot) & ~0xFu, false);
    }
    // HccaDoneHead lives at HCCA +0x84. This wrote it to +0x80, which is
    // HccaFrameNumber -- so the done queue landed in the frame counter, the
    // driver never saw a list of finished descriptors, and it could not
    // reclaim them. One report would be delivered and the endpoint then went
    // quiet.
    //
    // The handoff is also once per acknowledgement, not once per frame: while
    // WritebackDoneHead is still set the driver has not read the list yet, so
    // the controller accumulates internally rather than overwriting it.
    if (doneHead_ && !(intStatus_ & 0x02u) && hcca_ && ram &&
        hcca_ + 0x88u <= ramSize) {
        snoopWr(hcca_ + 0x84u, 4);
        stLe(hcca_ + 0x84u, doneHead_);
        doneHead_ = 0;
        raise(0x02u); // WritebackDoneHead
    }
}


// ---------------------------------------------------------------------
// USB: root-hub port 1 carries a low-speed HID boot keyboard.
//
// Open Firmware's console input is the keyboard package, not the serial
// port: with a display present it blocks in $call-method "read" on stdin,
// and injecting into the SCC changes nothing. The port reported nothing
// attached and no list was ever walked, so the controller had nothing to
// find and nothing to do.
// ---------------------------------------------------------------------

u32 OhciCell::ldLe(u32 pa) const
{
    if (!ram || pa + 4 > ramSize)
        return 0;
    snoopRd(pa, 4);
    return u32(ram[pa]) | (u32(ram[pa + 1]) << 8) |
           (u32(ram[pa + 2]) << 16) | (u32(ram[pa + 3]) << 24);
}

void OhciCell::stLe(u32 pa, u32 v)
{
    if (!ram || pa + 4 > ramSize)
        return;
    snoopWr(pa, 4);
    ram[pa] = u8(v);
    ram[pa + 1] = u8(v >> 8);
    ram[pa + 2] = u8(v >> 16);
    ram[pa + 3] = u8(v >> 24);
}

// The reply owed to the SETUP packet now in setup_. Descriptors are built
// here rather than kept as byte tables so the fields a host actually reads
// stay legible.
void OhciCell::buildDescriptor()
{
    reply_.clear();
    const u8 type = setup_[0];
    const u8 req = setup_[1];
    const u16 val = static_cast<u16>(setup_[2] | (setup_[3] << 8));
    const u16 len = static_cast<u16>(setup_[6] | (setup_[7] << 8));
    auto push = [&](std::initializer_list<u8> b) {
        for (u8 x : b)
            reply_.push_back(x);
    };
    if (type == 0x80 && req == 0x06) { // GET_DESCRIPTOR
        switch (val >> 8) {
        case 1: // DEVICE
            push({18, 1, 0x10, 0x01, 0, 0, 0, 8, 0xAC, 0x05,
                  static_cast<u8>(hid_ == Hid::Mouse ? 0x01 : 0x01),
                  static_cast<u8>(hid_ == Hid::Mouse ? 0x03 : 0x02), 0x00,
                  0x01, 1, 2, 0, 1});
            break;
        case 2: { // CONFIGURATION + INTERFACE + HID + ENDPOINT
            // bInterfaceProtocol 1 = keyboard, 2 = mouse; the boot protocol
            // is what a firmware and the Mac OS USB Expert bind to, and the
            // protocol byte is the only thing that tells them which is which.
            const u8 proto = hid_ == Hid::Mouse ? 2 : 1;
            const u8 rlen = hid_ == Hid::Mouse ? 52 : 63;
            push({9, 2, 34, 0, 1, 1, 0, 0xA0, 25});
            push({9, 4, 0, 0, 1, 3, 1, proto, 0}); // HID, boot subclass
            push({9, 0x21, 0x11, 0x01, 0, 1, 0x22, rlen, 0});
            // wMaxPacketSize is the ENDPOINT's capacity, not the report
            // length. Declaring it equal to reportLen_ made every report an
            // exact buffer fill, and OHCI 1.0 4.3.1.2 says a
            // CurrentBufferPointer of 0 means "a zero-length data packet OR
            // all bytes transferred" -- ambiguous, and a UIM that reads it as
            // the former computes an actual count of zero and drops the
            // report. A real low-speed Apple HID endpoint declares the
            // low-speed maximum of 8 and short-packets every report, which
            // leaves CBP non-zero and the count unambiguous.
            push({7, 5, 0x81, 3, 8, 0, 10});
            break;
        }
        case 3: // STRING
            if ((val & 0xFF) == 0)
                push({4, 3, 0x09, 0x04});
            else {
                // Both HID cells shared one product string, so the Name
                // Registry described the mouse as a "Keyboard" -- cosmetic,
                // but it is a confounder every time these nodes are read.
                const char* s = (val & 0xFF) == 1
                                    ? "OpenPowerMac"
                                    : (hid_ == Hid::Mouse ? "Mouse"
                                                          : "Keyboard");
                reply_.push_back(static_cast<u8>(2 + 2 * strlen(s)));
                reply_.push_back(3);
                for (const char* p = s; *p; ++p) {
                    reply_.push_back(static_cast<u8>(*p));
                    reply_.push_back(0);
                }
            }
            break;
        default: break;
        }
    } else if (type == 0x81 && req == 0x06 && (val >> 8) == 0x22 &&
               hid_ == Hid::Mouse) {
        // HID REPORT descriptor for a standard wheel mouse: FIVE button bits,
        // three bits of padding, then signed 8-bit X, Y and WHEEL as RELATIVE
        // values -- a four-byte report.
        //
        // This was a three-button, three-byte descriptor with no wheel. The
        // bytes reaching the wire were measured correct (`00 08 04` = buttons
        // 0, dx +8, dy +4, relative) and Mac OS still ignored every one of
        // them, while acting on the button byte of the same reports. The
        // driver's own GET_REPORT asks for wLength=4, which is the standard
        // four-byte layout, not our three-byte one. This is the canonical
        // mouse descriptor that ships in countless devices and that Mac OS 9
        // is known to drive; the layout is a fact, not borrowed code.
        push({0x05, 0x01, 0x09, 0x02, 0xA1, 0x01, 0x09, 0x01, 0xA1, 0x00,
              0x05, 0x09, 0x19, 0x01, 0x29, 0x05, 0x15, 0x00, 0x25, 0x01,
              0x95, 0x05, 0x75, 0x01, 0x81, 0x02, 0x95, 0x01, 0x75, 0x03,
              0x81, 0x01, 0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x38,
              0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x03, 0x81, 0x06,
              0xC0, 0xC0});
    } else if (type == 0x81 && req == 0x06 && (val >> 8) == 0x22) {
        // HID REPORT descriptor: the standard boot-keyboard layout - eight
        // modifier bits, one reserved byte, six key slots.
        push({0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x05, 0x07, 0x19, 0xE0,
              0x29, 0xE7, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x08,
              0x81, 0x02, 0x95, 0x01, 0x75, 0x08, 0x81, 0x03, 0x95, 0x05,
              0x75, 0x01, 0x05, 0x08, 0x19, 0x01, 0x29, 0x05, 0x91, 0x02,
              0x95, 0x01, 0x75, 0x03, 0x91, 0x03, 0x95, 0x06, 0x75, 0x08,
              0x15, 0x00, 0x25, 0x65, 0x05, 0x07, 0x19, 0x00, 0x29, 0x65,
              0x81, 0x00, 0xC0});
    } else if (type == 0xA1 && req == 0x01) {
        // HID GET_REPORT (HID 1.11 7.2.1) is MANDATORY for every HID device,
        // and it was the ONE request in the whole enumeration this device
        // answered with nothing -- measured: `a1 01 01 01 00 00 04 00
        // reply=0`. The mouse module issues it while initializing, so the
        // step failed, the module never reached its kCreateCursor state and
        // never registered a cursor device with the Cursor Device Manager.
        // The interrupt pipe was already open by then, so reports kept
        // arriving and the button byte still reached the global button state
        // while every X/Y was discarded -- exactly the asymmetry measured
        // (MBState toggles on an injected click, the pointer never moves).
        // Relative axes legitimately read as zero: there is no movement to
        // report until the next poll.
        if (hid_ == Hid::Mouse)
            push({buttons_, 0, 0, 0}); // relative axes read as no movement
        else
            reply_.assign(reportLen_, static_cast<u8>(0));
    } else if (type == 0x00 && req == 0x05) { // SET_ADDRESS
        pendingAddress_ = static_cast<u8>(val & 0x7F);
    }
    if (reply_.size() > len)
        reply_.resize(len);
}

// One transfer descriptor. Returns the next TD pointer, or 0 to leave the
// TD in place (a NAK).
bool OhciCell::doTd(u32 ed0, u32 td, u32& next)
{
    const u32 t0 = ldLe(td);
    const u32 cbp = ldLe(td + 4);
    next = ldLe(td + 8) & ~0xFu;
    const u32 be = ldLe(td + 12);
    const u32 dp = (t0 >> 19) & 3u;
    const u32 epn = (ed0 >> 7) & 0xFu;
    const u32 avail = (cbp && be >= cbp) ? (be - cbp + 1u) : 0u;
    u32 moved = 0;
    if (dp == 0) { // SETUP
        snoopRd(cbp, 8);
        for (u32 k = 0; k < 8 && cbp + k < ramSize; ++k)
            setup_[k] = ram[cbp + k];
        ++setupsSeen;
        buildDescriptor();
        moved = 8;
        // The eight bytes that decide everything downstream. An unrecognised
        // request leaves reply_ empty, and the IN that follows then moves
        // zero bytes -- which on the register bus looks exactly like a
        // controller that never ran.
        if (walkLog.size() < walkMax)
            walkLog.push_back(
                {2u, td,
                 u32(setup_[0]) | (u32(setup_[1]) << 8) |
                     (u32(setup_[2]) << 16) | (u32(setup_[3]) << 24),
                 u32(setup_[4]) | (u32(setup_[5]) << 8) |
                     (u32(setup_[6]) << 16) | (u32(setup_[7]) << 24),
                 static_cast<u32>(reply_.size()), 0u});
    } else if (dp == 2) { // IN
        ++inTds;
        if (epn != 0) {
            const u64 now = stamp ? *stamp : 0;
            if (!firstInTd) firstInTd = now;
            lastInTd = now;
        }
        if (epn == 0) ++inEp0;
        if (epn != 0 && !avail) ++noBuffer;
        if (epn != 0 && (hid_ == Hid::Mouse
                          ? (!accDx_ && !accDy_ && buttons_ == sentButtons_)
                          : pending_.size() < reportLen_))
            ++nakEmpty;
        if (epn == 0) {
            moved = avail < reply_.size() ? avail
                                          : static_cast<u32>(reply_.size());
            snoopWr(cbp, moved);
            for (u32 k = 0; k < moved && cbp + k < ramSize; ++k)
                ram[cbp + k] = reply_[k];
            reply_.erase(reply_.begin(), reply_.begin() + moved);
            // The status stage of a control transfer with NO data stage is
            // an IN of zero length, not an OUT -- and SET_ADDRESS is exactly
            // that shape. Committing the new address only on the OUT path
            // meant the very first request of enumeration was acknowledged
            // and then ignored, so every transfer after it went to an
            // address the device had never adopted.
            if (reply_.empty() && pendingAddress_) {
                address_ = pendingAddress_;
                pendingAddress_ = 0;
            }
        } else if (hid_ == Hid::Mouse) {
            // Report only on change: no motion and no button transition is a
            // NAK, which is what an idle mouse does on the wire.
            if (!accDx_ && !accDy_ && buttons_ == sentButtons_)
                return false;
            auto take = [](int& acc) -> u8 {
                int v = acc > 127 ? 127 : (acc < -127 ? -127 : acc);
                acc -= v; // the rest rides on the next poll
                return static_cast<u8>(static_cast<int8_t>(v));
            };
            const int wantDx = accDx_, wantDy = accDy_; // before take() drains
            // buttons, X, Y, WHEEL -- the four-byte standard mouse report the
            // descriptor above declares. The wheel is always still.
            const u8 rep[4] = {buttons_, take(accDx_), take(accDy_), 0};
            sentButtons_ = buttons_;
            moved = avail < reportLen_ ? avail : reportLen_;
            snoopWr(cbp, moved);
            for (u32 k = 0; k < moved && cbp + k < ramSize; ++k)
                ram[cbp + k] = rep[k];
            ++reportsSent;
            // The bytes actually placed on the wire, and the deltas they were
            // built from. Every previous claim about this report's contents was
            // read off the source rather than measured, which is the one kind
            // of evidence this project has repeatedly found to be wrong.
            if (walkLog.size() < walkMax)
                walkLog.push_back({3u, td,
                                   u32(rep[0]) | (u32(rep[1]) << 8) |
                                       (u32(rep[2]) << 16) |
                                       (u32(rep[3]) << 24),
                                   static_cast<u32>(wantDx),
                                   static_cast<u32>(wantDy), moved});
        } else if (pending_.size() >= reportLen_) {
            moved = avail < reportLen_ ? avail : reportLen_;
            snoopWr(cbp, moved);
            for (u32 k = 0; k < moved && cbp + k < ramSize; ++k)
                ram[cbp + k] = pending_[k];
            pending_.erase(pending_.begin(),
                           pending_.begin() + static_cast<int>(reportLen_));
            ++reportsSent;
        } else {
            // No key down. A keyboard must NAK rather than hand back a
            // stale frame, so leave the descriptor where it is.
            return false;
        }
    } else { // OUT or STATUS
        moved = avail;
        if (pendingAddress_) {
            address_ = pendingAddress_;
            pendingAddress_ = 0;
        }
    }
    if (walkLog.size() < walkMax)
        walkLog.push_back({1u, td, t0, cbp, be, moved});
    stLe(td + 4, (moved && cbp + moved <= be) ? cbp + moved : 0);
    retire(td, 0);
    return true;
}

// Move a finished descriptor onto the done queue with a completion code.
void OhciCell::retire(u32 td, u32 cc)
{
    u32 t0 = ldLe(td);
    t0 = (t0 & 0x0FFFFFFFu) | (cc << 28);
    stLe(td, t0);
    stLe(td + 8, doneHead_); // NextTD doubles as the done-queue link
    doneHead_ = td;
    // WritebackDoneHead is raised when the list is actually handed over in
    // tick(), not per descriptor: the bit tells the driver a queue is waiting
    // at HccaDoneHead, and setting it before anything was written there is a
    // promise the controller has not kept.
}

// Walk one endpoint-descriptor list.
u32 OhciCell::runList(u32 head, bool control)
{
    (void)control;
    u32 done = 0;
    for (u32 ed = head; ed && done < 64;) {
        const u32 e0 = ldLe(ed);
        const u32 tail = ldLe(ed + 4) & ~0xFu;
        u32 hp = ldLe(ed + 8);
        const u32 nextEd = ldLe(ed + 12) & ~0xFu;
        const bool skip = (e0 & 0x4000u) != 0;
        const bool halted = (hp & 1u) != 0;
        const u32 doneBefore = done;
        u32 h = hp & ~0xFu;
        while (!skip && !halted && h && h != tail && done < 64) {
            u32 nx = 0;
            if (!doTd(e0, h, nx))
                break; // NAKed: the descriptor stays where it is
            h = nx;    // retired: advance, and 0 legitimately empties the queue
            ++done;
        }
        // Log the ED only once it has actually RETIRED a descriptor. A
        // skipped ED is interrupt-tree scaffolding, an ED whose HeadP has
        // caught up with TailP is empty, and a NAKing one carries no data:
        // logging those refilled the capped log on every frame and crowded
        // out the TD records entirely -- which is how this log came to hold
        // 400 endpoint descriptors and not one of the reports the driver was
        // being handed.
        if (done != doneBefore && walkLog.size() < walkMax)
            walkLog.push_back({0u, ed, e0, hp, tail, nextEd});
        stLe(ed + 8, (hp & 0xFu) | h);
        ed = nextEd;
    }
    return done;
}

// Queue an ASCII string as HID boot-keyboard reports.
void OhciCell::moveMouse(int dx, int dy, u8 buttons)
{
    // Accumulate; the report is built when the host actually polls.
    accDx_ += dx;
    accDy_ += dy;
    buttons_ = buttons & 0x07u;
}

void OhciCell::typeAscii(const std::string& s)
{
    for (char c : s) {
        u8 usage = 0, mod = 0;
        if (c >= 'a' && c <= 'z')
            usage = static_cast<u8>(4 + (c - 'a'));
        else if (c >= 'A' && c <= 'Z') {
            usage = static_cast<u8>(4 + (c - 'A'));
            mod = 0x02; // left shift
        } else if (c >= '1' && c <= '9')
            usage = static_cast<u8>(30 + (c - '1'));
        else if (c == '0')
            usage = 39;
        else if (c == '\r' || c == '\n')
            usage = 40;
        else if (c == ' ')
            usage = 44;
        else if (c == '-')
            usage = 45;
        else if (c == '.')
            usage = 55;
        else if (c == '/')
            usage = 56;
        else if (c == '\\')
            usage = 49;
        else if (c == ',')
            usage = 54;
        else if (c == ';')
            usage = 51;
        else if (c == ':') {
            usage = 51;
            mod = 0x02;
        } else
            continue;
        // Press then release: a host that only ever sees the key down
        // repeats it forever.
        const u8 down[8] = {mod, 0, usage, 0, 0, 0, 0, 0};
        for (u8 b : down)
            pending_.push_back(b);
        for (u32 k = 0; k < 8; ++k)
            pending_.push_back(0);
    }
}
} // namespace opm
