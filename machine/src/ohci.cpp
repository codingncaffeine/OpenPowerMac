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
            if ((rhPort_[0] | rhPort_[1]) & 0x001F0000u) intStatus_ |= 0x40u;
        }
        cmdStatus_ = v & ~1u & 0x0000000Eu; // OCR/BLF/CLF latch, no lists
        break;
    case 0x0C >> 2:
        intStatus_ &= ~v; // write-1-to-clear
        break;
    case 0x10 >> 2:
        intEnable_ |= v; // write-1-to-set (MIE included)
        break;
    case 0x14 >> 2:
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
    case 0x48 >> 2: // RhDescriptorA is mostly hardwired; NDP read-only
        rhDescA_ = (rhDescA_ & 0x000000FFu) | (v & 0xFFFFFF00u);
        break;
    case 0x4C >> 2: rhDescB_ = v; break;
    case 0x50 >> 2:
        // LPSC/LPS global power bits: accept and forget (ports report
        // powered through RhPortStatus PPS).
        break;
    case 0x54 >> 2:
    case 0x58 >> 2: {
        const u32 n = idx - (0x54 >> 2);
        u32& p = rhPort_[n];
        const u32 before = p;
        // Port 1 carries the keyboard; port 2 is genuinely empty.
        const bool populated = (n == 0);
        if (v & 0x00000100u) // SetPortPower
            p |= 0x00000100u;
        if (v & 0x00000200u) // ClearPortPower
            p &= ~0x00000100u;
        if (v & 0x00000002u) { // SetPortEnable
            if (populated)
                p |= 0x00000002u;
            else
                p |= 0x00010000u; // CSC: enable attempt on a dead port
        }
        if (v & 0x00000001u) // ClearPortEnable
            p &= ~0x00000002u;
        if (v & 0x00000010u) { // SetPortReset
            if (populated) {
                // Reset is IN PROGRESS: report PRS and finish it on a later
                // frame (see kResetFrames). The driver arms what it waits on
                // after this store, so completing inside it is invisible.
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
    if (log.size() < 2048)
        log.push_back({stamp ? *stamp : 0, off, native,
                       pcRef ? *pcRef : 0});
    regWrite(off >> 2, native);
}

void OhciCell::tick(u64 tb)
{
    if (((control_ >> 6) & 3u) != 2u) { // not operational
        lastFrameTb_ = tb;
        return;
    }
    if (tb - lastFrameTb_ < kTbPerFrame)
        return;
    lastFrameTb_ = tb;
    fmNumber_ = (fmNumber_ + 1) & 0xFFFFu;
    if (hcca_ && ram && hcca_ + 0x84u <= ramSize) {
        // HccaFrameNumber: 16-bit little-endian at HCCA+0x80, pad zero.
        snoopWr(hcca_ + 0x80u, 4);
        ram[hcca_ + 0x80] = static_cast<u8>(fmNumber_);
        ram[hcca_ + 0x81] = static_cast<u8>(fmNumber_ >> 8);
        ram[hcca_ + 0x82] = 0;
        ram[hcca_ + 0x83] = 0;
    }
    intStatus_ |= 0x00000004u; // SF

    // Finish any port reset that is running. The port comes out enabled,
    // with PRSC set, and RHSC raised so a host polling HcInterruptStatus
    // is told to go and look at the port.
    for (u32 n = 0; n < 2; ++n) {
        if (!portReset_[n] || --portReset_[n]) continue;
        const u32 before = rhPort_[n];
        rhPort_[n] &= ~0x00000010u;             // PRS clear: reset done
        rhPort_[n] |= 0x00000002u | 0x00100000u; // PES + PRSC
        rhSignal(before, rhPort_[n]);
    }

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
        intStatus_ |= 0x02u; // WritebackDoneHead
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
            const u8 rlen = hid_ == Hid::Mouse ? 50 : 63;
            push({9, 2, 34, 0, 1, 1, 0, 0xA0, 25});
            push({9, 4, 0, 0, 1, 3, 1, proto, 0}); // HID, boot subclass
            push({9, 0x21, 0x11, 0x01, 0, 1, 0x22, rlen, 0});
            push({7, 5, 0x81, 3, static_cast<u8>(reportLen_), 0, 10});
            break;
        }
        case 3: // STRING
            if ((val & 0xFF) == 0)
                push({4, 3, 0x09, 0x04});
            else {
                const char* s =
                    (val & 0xFF) == 1 ? "OpenPowerMac" : "Keyboard";
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
        // HID REPORT descriptor, boot mouse: three button bits, five bits of
        // padding, then signed 8-bit X and Y as RELATIVE values.
        push({0x05, 0x01, 0x09, 0x02, 0xA1, 0x01, 0x09, 0x01, 0xA1, 0x00,
              0x05, 0x09, 0x19, 0x01, 0x29, 0x03, 0x15, 0x00, 0x25, 0x01,
              0x95, 0x03, 0x75, 0x01, 0x81, 0x02, 0x95, 0x01, 0x75, 0x05,
              0x81, 0x03, 0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x15, 0x81,
              0x25, 0x7F, 0x75, 0x08, 0x95, 0x02, 0x81, 0x06, 0xC0, 0xC0});
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
            const u8 rep[3] = {buttons_, take(accDx_), take(accDy_)};
            sentButtons_ = buttons_;
            moved = avail < 3u ? avail : 3u;
            snoopWr(cbp, moved);
            for (u32 k = 0; k < moved && cbp + k < ramSize; ++k)
                ram[cbp + k] = rep[k];
            ++reportsSent;
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
        if (walkLog.size() < walkMax)
            walkLog.push_back({0u, ed, e0, hp, tail, nextEd});
        const bool skip = (e0 & 0x4000u) != 0;
        const bool halted = (hp & 1u) != 0;
        u32 h = hp & ~0xFu;
        while (!skip && !halted && h && h != tail && done < 64) {
            u32 nx = 0;
            if (!doTd(e0, h, nx))
                break; // NAKed: the descriptor stays where it is
            h = nx;    // retired: advance, and 0 legitimately empties the queue
            ++done;
        }
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
