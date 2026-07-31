// Machine snapshot / restore — every mutable field of the CPU and of every
// modelled device, written by hand in one place.
//
// It is deliberately ONE file. The property that makes a snapshot usable as
// evidence is completeness, and completeness is auditable only if the whole
// state list sits under one pair of eyes: add a field to a device and the
// question "is it saved?" is answered by searching this file, not by
// remembering which of eight translation units owns it.

#include "opm/snapshot.hpp"

#include "opm/ata.hpp"
#include "opm/cpu.hpp"
#include "opm/dbdma.hpp"
#include "opm/ohci.hpp"
#include "opm/openpic.hpp"
#include "opm/pmu.hpp"
#include "opm/r128.hpp"
#include "opm/sawtooth.hpp"

#include <cstdio>

namespace opm {

namespace {

constexpr u32 kSnapMagic = 0x314D504Fu; // 'OPM1'
// Bump when the SHAPE of the stream changes (a field added, an order
// changed). The layout digest below catches struct-size changes on its own;
// this catches everything else.
// 10 carries no new fields: the machine grew a device-service gate, which
// changes sizeof(SawtoothBus) and so is refused by the layout digest anyway.
// The number is bumped so the refusal reads as a deliberate change rather
// than as a mystery.
// 11 adds the KeyLargo timer's reload point and value. It also changes
// sizeof(SawtoothBus), so old files are refused by the digest either way.
constexpr u32 kSnapVersion = 11; // 11: the KeyLargo timer

u64 fnv1a(const void* p, size_t n, u64 h = 1469598103934665603ull)
{
    const u8* b = static_cast<const u8*>(p);
    for (size_t k = 0; k < n; ++k) {
        h ^= b[k];
        h *= 1099511628211ull;
    }
    return h;
}

// A digest of every structure whose layout the stream depends on. A build
// that shapes any of them differently reads a different machine, so it must
// be refused up front rather than allowed to misparse the bytes.
u64 layoutDigest()
{
    const u64 sizes[] = {
        sizeof(CpuState),        sizeof(Cpu::TlbEntry), sizeof(Cpu::DLine),
        sizeof(Cpu::L2Line),     sizeof(AtaCell),       sizeof(OpenPic),
        sizeof(OhciCell),        sizeof(DbdmaChannel),  sizeof(PmuVia),
        sizeof(R128Cell),        sizeof(SawtoothBus),   sizeof(V128),
        sizeof(HarnessState),    kSnapVersion,
    };
    return fnv1a(sizes, sizeof sizes);
}

// Fixed-size blob restore: the vector keeps its allocation, so pointers
// taken into it before the load (the OHCI cells' view of guest RAM) stay
// valid. A size mismatch is a different machine and is refused.
void bytesInPlace(SnapReader& r, std::vector<u8>& v, const char* what)
{
    const u64 n = r.u64v();
    if (!r.ok)
        return;
    if (n != v.size()) {
        char msg[128];
        snprintf(msg, sizeof msg, "%s is %llu bytes in the snapshot, %llu here",
                 what, static_cast<unsigned long long>(n),
                 static_cast<unsigned long long>(v.size()));
        r.fail(msg);
        return;
    }
    r.raw(v.data(), static_cast<size_t>(n));
}

void saveMapU32(SnapWriter& w, const std::map<u32, u32>& m)
{
    w.u64v(m.size());
    for (const auto& [k, v] : m) {
        w.u32v(k);
        w.u32v(v);
    }
}
void loadMapU32(SnapReader& r, std::map<u32, u32>& m)
{
    m.clear();
    const u64 n = r.u64v();
    for (u64 k = 0; k < n && r.ok; ++k) {
        const u32 key = r.u32v();
        m[key] = r.u32v();
    }
}
void saveVecU32(SnapWriter& w, const std::vector<u32>& v)
{
    w.u64v(v.size());
    for (u32 x : v)
        w.u32v(x);
}
void loadVecU32(SnapReader& r, std::vector<u32>& v)
{
    const u64 n = r.u64v();
    v.clear();
    for (u64 k = 0; k < n && r.ok; ++k)
        v.push_back(r.u32v());
}

// --- CPU ------------------------------------------------------------------

void saveCpuState(SnapWriter& w, const CpuState& s)
{
    w.arr32(s.gpr, 32);
    for (u32 k = 0; k < 32; ++k)
        w.u64v(s.fpr[k]);
    for (u32 k = 0; k < 32; ++k)
        w.raw(s.vr[k].b, 16);
    w.u32v(s.pc);
    w.u32v(s.cr);
    w.u32v(s.xer);
    w.u32v(s.lr);
    w.u32v(s.ctr);
    w.u32v(s.msr);
    w.u32v(s.fpscr);
    w.u32v(s.vscr);
    w.u32v(s.vrsave);
    w.u32v(s.srr0);
    w.u32v(s.srr1);
    w.arr32(s.sprg, 4);
    w.u32v(s.dar);
    w.u32v(s.dsisr);
    w.u32v(s.sdr1);
    w.u32v(s.ear);
    w.u32v(s.pir);
    w.arr32(s.sr, 16);
    w.arr32(s.ibatu, 4);
    w.arr32(s.ibatl, 4);
    w.arr32(s.dbatu, 4);
    w.arr32(s.dbatl, 4);
    w.u32v(s.hid0);
    w.u32v(s.hid1);
    w.u32v(s.msscr0);
    w.u32v(s.msscr1);
    w.u32v(s.l2cr);
    w.u32v(s.ictc);
    w.arr32(s.thrm, 3);
    w.u32v(s.iabr);
    w.u32v(s.dabr);
    w.u32v(s.bamr);
    w.u32v(s.mmcr0);
    w.u32v(s.mmcr1);
    w.arr32(s.pmc, 4);
    w.u32v(s.siar);
    w.u32v(s.sdar);
    w.u64v(s.tb);
    w.u32v(s.dec);
    w.u32v(s.pvr);
    w.b(s.resvValid);
    w.u32v(s.resvAddr);
}

void loadCpuState(SnapReader& r, CpuState& s)
{
    r.arr32(s.gpr, 32);
    for (u32 k = 0; k < 32; ++k)
        s.fpr[k] = r.u64v();
    for (u32 k = 0; k < 32; ++k)
        r.raw(s.vr[k].b, 16);
    s.pc = r.u32v();
    s.cr = r.u32v();
    s.xer = r.u32v();
    s.lr = r.u32v();
    s.ctr = r.u32v();
    s.msr = r.u32v();
    s.fpscr = r.u32v();
    s.vscr = r.u32v();
    s.vrsave = r.u32v();
    s.srr0 = r.u32v();
    s.srr1 = r.u32v();
    r.arr32(s.sprg, 4);
    s.dar = r.u32v();
    s.dsisr = r.u32v();
    s.sdr1 = r.u32v();
    s.ear = r.u32v();
    s.pir = r.u32v();
    r.arr32(s.sr, 16);
    r.arr32(s.ibatu, 4);
    r.arr32(s.ibatl, 4);
    r.arr32(s.dbatu, 4);
    r.arr32(s.dbatl, 4);
    s.hid0 = r.u32v();
    s.hid1 = r.u32v();
    s.msscr0 = r.u32v();
    s.msscr1 = r.u32v();
    s.l2cr = r.u32v();
    s.ictc = r.u32v();
    r.arr32(s.thrm, 3);
    s.iabr = r.u32v();
    s.dabr = r.u32v();
    s.bamr = r.u32v();
    s.mmcr0 = r.u32v();
    s.mmcr1 = r.u32v();
    r.arr32(s.pmc, 4);
    s.siar = r.u32v();
    s.sdar = r.u32v();
    s.tb = r.u64v();
    s.dec = r.u32v();
    s.pvr = r.u32v();
    s.resvValid = r.b();
    s.resvAddr = r.u32v();
}

void saveTlb(SnapWriter& w, const Cpu::TlbEntry (&t)[64][2])
{
    for (u32 set = 0; set < 64; ++set)
        for (u32 way = 0; way < 2; ++way) {
            const Cpu::TlbEntry& e = t[set][way];
            w.b(e.v);
            w.b(e.c);
            w.u32v(e.vsid);
            w.u32v(e.pi);
            w.u32v(e.rpn);
            w.u32v(e.wimg);
            w.u32v(e.pp);
        }
}
void loadTlb(SnapReader& r, Cpu::TlbEntry (&t)[64][2])
{
    for (u32 set = 0; set < 64; ++set)
        for (u32 way = 0; way < 2; ++way) {
            Cpu::TlbEntry& e = t[set][way];
            e.v = r.b();
            e.c = r.b();
            e.vsid = r.u32v();
            e.pi = r.u32v();
            e.rpn = r.u32v();
            e.wimg = r.u32v();
            e.pp = r.u32v();
        }
}

void saveCpu(SnapWriter& w, const Cpu& c)
{
    const size_t sec = w.begin("CPU ");
    saveCpuState(w, c.st);
    w.b(c.extIrqLine);
    w.b(c.smiPending);
    w.b(c.decPending);
    w.b(c.pmPending);
    w.b(c.raisedThisStep);
    w.b(c.napping);
    w.u32v(c.curInsn);
    w.u32v(c.cycleAccum);
    w.u32v(c.cyclesPerTbTick);
    w.b(c.halted);
    w.str(c.haltReason);
    w.b(c.mmuProbe);
    w.u32v(c.realModeInhibitBase);
    w.end(sec);

    // The TLBs are architected-invisible but load-bearing: the ROM's regime
    // teardown wipes the hash table and keeps running out of warm entries.
    const size_t mmu = w.begin("MMU ");
    saveTlb(w, c.itlb);
    saveTlb(w, c.dtlb);
    w.raw(c.itlbLru, 64);
    w.raw(c.dtlbLru, 64);
    w.end(mmu);

    // Caches are state, not an optimization. The nanokernel's spinlock bug
    // was a cache-coherency bug: dirty L1D lines held values that RAM did
    // not, and a snapshot that dropped them would resume a different
    // machine while looking identical.
    const size_t l1 = w.begin("L1D ");
    w.u32v(c.l1dClock);
    for (u32 set = 0; set < 128; ++set)
        for (u32 way = 0; way < 8; ++way) {
            const Cpu::DLine& l = c.l1d[set][way];
            w.b(l.v);
            w.b(l.d);
            w.u32v(l.tag);
            w.u32v(l.age);
            w.raw(l.b, 32);
        }
    w.end(l1);

    const size_t l2 = w.begin("L2  ");
    w.u32v(c.l2Sets);
    w.u32v(c.l2Clock);
    w.u64v(c.l2.size());
    for (const Cpu::L2Line& l : c.l2) {
        w.b(l.v);
        w.b(l.d);
        w.u32v(l.tag);
        w.u32v(l.age);
        w.raw(l.b, 32);
    }
    w.end(l2);
}

void loadCpu(SnapReader& r, Cpu& c)
{
    // Caches on Cpu are outside the stream by design (they cost no snapshot
    // compatibility), so they hold the PREVIOUS machine's data and must be
    // dropped rather than carried across. The fetch buffer in particular
    // would serve instructions from a machine that no longer exists.
    c.fetchDrop();
    c.xlDrop();
    const u8* e = r.beginSection("CPU ");
    loadCpuState(r, c.st);
    c.extIrqLine = r.b();
    c.smiPending = r.b();
    c.decPending = r.b();
    c.pmPending = r.b();
    c.raisedThisStep = r.b();
    c.napping = r.b();
    c.curInsn = r.u32v();
    c.cycleAccum = r.u32v();
    c.cyclesPerTbTick = r.u32v();
    c.halted = r.b();
    c.haltReason = r.str();
    c.mmuProbe = r.b();
    c.realModeInhibitBase = r.u32v();
    r.endSection("CPU ", e);

    e = r.beginSection("MMU ");
    loadTlb(r, c.itlb);
    loadTlb(r, c.dtlb);
    r.raw(c.itlbLru, 64);
    r.raw(c.dtlbLru, 64);
    r.endSection("MMU ", e);

    e = r.beginSection("L1D ");
    c.l1dClock = r.u32v();
    for (u32 set = 0; set < 128; ++set)
        for (u32 way = 0; way < 8; ++way) {
            Cpu::DLine& l = c.l1d[set][way];
            l.v = r.b();
            l.d = r.b();
            l.tag = r.u32v();
            l.age = r.u32v();
            r.raw(l.b, 32);
        }
    r.endSection("L1D ", e);

    e = r.beginSection("L2  ");
    c.l2Sets = r.u32v();
    c.l2Clock = r.u32v();
    const u64 n = r.u64v();
    c.l2.assign(static_cast<size_t>(r.ok ? n : 0), Cpu::L2Line{});
    for (u64 k = 0; k < n && r.ok; ++k) {
        Cpu::L2Line& l = c.l2[static_cast<size_t>(k)];
        l.v = r.b();
        l.d = r.b();
        l.tag = r.u32v();
        l.age = r.u32v();
        r.raw(l.b, 32);
    }
    r.endSection("L2  ", e);
}

} // namespace

// --- devices --------------------------------------------------------------

void AtaCell::snapSave(SnapWriter& w) const
{
    w.b(iso_ != nullptr);
    w.u64v(isoBytes_);
    w.b(disk_);
    w.u64v(diskSectors_);
    w.u32v(wrLeft_);
    w.u64v(wrLba_);
    w.u8v(features_);
    w.u8v(nsect_);
    w.u8v(lba0_);
    w.u8v(bcLo_);
    w.u8v(bcHi_);
    w.u8v(dev_);
    w.u8v(status_);
    w.u8v(error_);
    w.u8v(devctl_);
    w.bytes(data_);
    w.u64v(dataAt_);
    w.b(cdbPhase_);
    w.raw(cdb_, 12);
    w.u32v(cdbAt_);
    w.u64v(readLba_);
    w.u32v(readLeft_);
    w.u8v(sense_);
    // Deferred command: a snapshot taken inside the BSY window must resume
    // with the command still owed, or the drive silently drops it.
    w.b(pending_);
    w.u8v(pendCmd_);
    w.u64v(pendAt_);
    w.u64v(cmdDelay_);
    // The latched task file travels WITH the pending command. Saving the
    // command without the registers it sampled resumes the drive running a
    // different transfer than the host asked for.
    w.u8v(pendNsect_);
    w.u8v(pendLba0_);
    w.u8v(pendBcLo_);
    w.u8v(pendBcHi_);
    w.u8v(pendDev_);
    w.u32v(pendPc_);
    w.u8v(multiple_);
    // The CHS translation INITIALIZE DEVICE PARAMETERS set. A resume that
    // reverts it addresses different sectors than the guest asked for, and
    // the guest has no way to notice: it programmed the geometry once, at
    // boot, and never asks again.
    w.u8v(curHeads_);
    w.u8v(curSectors_);
    w.b(irq_);
    w.b(dmaIrqLatch_);
    w.arr32(ctl_, 16);
    w.u64v(log.size());
    for (const Ev& ev : log) {
        w.u64v(ev.at);
        w.u8v(static_cast<u8>(ev.kind));
        w.u8v(ev.val);
        w.u32v(ev.a);
        w.u32v(ev.b);
        w.u32v(ev.pc);
        w.raw(ev.cdb, 12);
        w.u32v(ev.xfer);
    }
}

void AtaCell::snapLoad(SnapReader& r)
{
    const bool wasPresent = r.b();
    const u64 bytes = r.u64v();
    if (r.ok && wasPresent != (iso_ != nullptr)) {
        r.fail(wasPresent ? "snapshot has media attached to an ATA cell that "
                            "is empty in this run"
                          : "this run attaches media to an ATA cell that was "
                            "empty in the snapshot");
        return;
    }
    if (r.ok && wasPresent && bytes != isoBytes_) {
        r.fail("a different image is attached than the one in the snapshot");
        return;
    }
    isoBytes_ = bytes;
    disk_ = r.b();
    diskSectors_ = r.u64v();
    wrLeft_ = r.u32v();
    wrLba_ = r.u64v();
    features_ = r.u8v();
    nsect_ = r.u8v();
    lba0_ = r.u8v();
    bcLo_ = r.u8v();
    bcHi_ = r.u8v();
    dev_ = r.u8v();
    status_ = r.u8v();
    error_ = r.u8v();
    devctl_ = r.u8v();
    r.bytes(data_);
    dataAt_ = static_cast<size_t>(r.u64v());
    cdbPhase_ = r.b();
    r.raw(cdb_, 12);
    cdbAt_ = r.u32v();
    readLba_ = r.u64v();
    readLeft_ = r.u32v();
    sense_ = r.u8v();
    pending_ = r.b();
    pendCmd_ = r.u8v();
    pendAt_ = r.u64v();
    cmdDelay_ = r.u64v();
    pendNsect_ = r.u8v();
    pendLba0_ = r.u8v();
    pendBcLo_ = r.u8v();
    pendBcHi_ = r.u8v();
    pendDev_ = r.u8v();
    pendPc_ = r.u32v();
    multiple_ = r.u8v();
    curHeads_ = r.u8v();
    curSectors_ = r.u8v();
    irq_ = r.b();
    dmaIrqLatch_ = r.b();
    r.arr32(ctl_, 16);
    const u64 n = r.u64v();
    log.clear();
    for (u64 k = 0; k < n && r.ok; ++k) {
        Ev ev{};
        ev.at = r.u64v();
        ev.kind = static_cast<char>(r.u8v());
        ev.val = r.u8v();
        ev.a = r.u32v();
        ev.b = r.u32v();
        ev.pc = r.u32v();
        r.raw(ev.cdb, 12);
        ev.xfer = r.u32v();
        log.push_back(ev);
    }
}

void OpenPic::snapSave(SnapWriter& w) const
{
    for (u32 s = 0; s < kSources; ++s) {
        w.u32v(vp_[s]);
        w.u32v(dest_[s]);
        w.b(line_[s]);
        w.b(pending_[s]);
        w.u64v(raiseCount[s]);
    }
    w.u32v(taskPri_);
    w.u32v(spurious_);
    w.i32v(inService_);
    w.arr32(global_, 0x400);
    w.u64v(log.size());
    for (const Ev& ev : log) {
        w.u64v(ev.at);
        w.u8v(static_cast<u8>(ev.kind));
        w.u32v(ev.val);
    }
}

void OpenPic::snapLoad(SnapReader& r)
{
    for (u32 s = 0; s < kSources; ++s) {
        vp_[s] = r.u32v();
        dest_[s] = r.u32v();
        line_[s] = r.b();
        pending_[s] = r.b();
        raiseCount[s] = r.u64v();
    }
    taskPri_ = r.u32v();
    spurious_ = r.u32v();
    inService_ = r.i32v();
    r.arr32(global_, 0x400);
    const u64 n = r.u64v();
    log.clear();
    for (u64 k = 0; k < n && r.ok; ++k) {
        Ev ev{};
        ev.at = r.u64v();
        ev.kind = static_cast<char>(r.u8v());
        ev.val = r.u32v();
        log.push_back(ev);
    }
}

void OhciCell::snapSave(SnapWriter& w) const
{
    w.u32v(control_);
    w.u32v(cmdStatus_);
    w.u32v(intStatus_);
    w.u32v(intEnable_);
    w.u32v(hcca_);
    w.u32v(periodCurrent_);
    w.u32v(ctrlHead_);
    w.u32v(ctrlCurrent_);
    w.u32v(bulkHead_);
    w.u32v(bulkCurrent_);
    w.u32v(doneHead_);
    w.u32v(fmInterval_);
    w.u32v(fmNumber_);
    w.u32v(periodicStart_);
    w.u32v(lsThreshold_);
    w.u32v(rhDescA_);
    w.u32v(rhDescB_);
    w.u32v(rhStatus_);
    w.u32v(rhPort_[0]);
    w.u32v(rhPort_[1]);
    w.u32v(portReset_[0]);
    w.u32v(portReset_[1]);
    w.u32v(portPower_[0]);
    w.u32v(portPower_[1]);
    w.u64v(lastFrameTb_);
    // What the keyboard is currently holding down. A resume that dropped it
    // would release keys the guest believes are still pressed, and a stuck
    // modifier is invisible until it changes the meaning of every later key.
    w.u8v(keyMod_);
    w.raw(keySlots_, 6);
    w.u64v(log.size());
    for (const Ev& ev : log) {
        w.u64v(ev.at);
        w.u32v(ev.off);
        w.u32v(ev.val);
        w.u32v(ev.pc);
    }
    // readCount/writeCount/rhLog and the interrupt census are DELIBERATELY not
    // saved. They exist to measure the OS era, and a resumed run must not
    // inherit Open Firmware's counts: `readCount` used to be saved while
    // `writeCount` was not, so the same report mixed a whole-boot read census
    // with an OS-era write census and "+048 RhDescriptorA r=18" looked like
    // the OS reading a register it had never touched.
}

void OhciCell::snapLoad(SnapReader& r)
{
    control_ = r.u32v();
    cmdStatus_ = r.u32v();
    intStatus_ = r.u32v();
    intEnable_ = r.u32v();
    hcca_ = r.u32v();
    periodCurrent_ = r.u32v();
    ctrlHead_ = r.u32v();
    ctrlCurrent_ = r.u32v();
    bulkHead_ = r.u32v();
    bulkCurrent_ = r.u32v();
    doneHead_ = r.u32v();
    fmInterval_ = r.u32v();
    fmNumber_ = r.u32v();
    periodicStart_ = r.u32v();
    lsThreshold_ = r.u32v();
    rhDescA_ = r.u32v();
    rhDescB_ = r.u32v();
    rhStatus_ = r.u32v();
    rhPort_[0] = r.u32v();
    rhPort_[1] = r.u32v();
    portReset_[0] = r.u32v();
    portReset_[1] = r.u32v();
    portPower_[0] = r.u32v();
    portPower_[1] = r.u32v();
    lastFrameTb_ = r.u64v();
    keyMod_ = r.u8v();
    r.raw(keySlots_, 6);
    const u64 n = r.u64v();
    log.clear();
    for (u64 k = 0; k < n && r.ok; ++k) {
        Ev ev{};
        ev.at = r.u64v();
        ev.off = r.u32v();
        ev.val = r.u32v();
        ev.pc = r.u32v();
        log.push_back(ev);
    }
    // (readCount is no longer part of the stream — see snapSave.)
}

void DbdmaChannel::snapSave(SnapWriter& w) const
{
    w.u32v(status_);
    w.u32v(cmdPtr_);
    w.u32v(intSel_);
    w.u32v(brSel_);
    w.u32v(waitSel_);
    w.b(irq_);
    w.u64v(log.size());
    for (const Ev& ev : log) {
        w.u64v(ev.at);
        w.u32v(ev.kind);
        w.u32v(ev.a);
        w.u32v(ev.b);
    }
}

void DbdmaChannel::snapLoad(SnapReader& r)
{
    status_ = r.u32v();
    cmdPtr_ = r.u32v();
    intSel_ = r.u32v();
    brSel_ = r.u32v();
    waitSel_ = r.u32v();
    irq_ = r.b();
    const u64 n = r.u64v();
    log.clear();
    for (u64 k = 0; k < n && r.ok; ++k) {
        Ev ev{};
        ev.at = r.u64v();
        ev.kind = r.u32v();
        ev.a = r.u32v();
        ev.b = r.u32v();
        log.push_back(ev);
    }
}

void PmuVia::snapSave(SnapWriter& w) const
{
    w.u8v(portAIn);
    w.u8v(orb_);
    w.u8v(ora_);
    w.u8v(ddrb_);
    w.u8v(ddra_);
    w.u8v(t1ll_);
    w.u8v(t1lh_);
    w.u8v(t2cl_);
    w.u16v(t1Load_);
    w.u16v(t2Load_);
    w.u64v(t1At_);
    w.u64v(t2At_);
    w.u8v(sr_);
    w.u8v(acr_);
    w.u8v(pcr_);
    w.u8v(ifr_);
    w.u8v(ier_);
    w.b(ack_);
    w.b(lastReq_);
    w.b(lastDirIn_);
    w.bytes(frame_);
    w.bytes(reply_);
    w.u32v(replyAt_);
    w.u64v(log.size());
    for (const Ev& ev : log) {
        w.u64v(ev.at);
        w.u8v(static_cast<u8>(ev.kind));
        w.u8v(ev.val);
    }
}

void PmuVia::snapLoad(SnapReader& r)
{
    portAIn = r.u8v();
    orb_ = r.u8v();
    ora_ = r.u8v();
    ddrb_ = r.u8v();
    ddra_ = r.u8v();
    t1ll_ = r.u8v();
    t1lh_ = r.u8v();
    t2cl_ = r.u8v();
    t1Load_ = r.u16v();
    t2Load_ = r.u16v();
    t1At_ = r.u64v();
    t2At_ = r.u64v();
    sr_ = r.u8v();
    acr_ = r.u8v();
    pcr_ = r.u8v();
    ifr_ = r.u8v();
    ier_ = r.u8v();
    ack_ = r.b();
    lastReq_ = r.b();
    lastDirIn_ = r.b();
    r.bytes(frame_);
    r.bytes(reply_);
    replyAt_ = r.u32v();
    const u64 n = r.u64v();
    log.clear();
    for (u64 k = 0; k < n && r.ok; ++k) {
        Ev ev{};
        ev.at = r.u64v();
        ev.kind = static_cast<char>(r.u8v());
        ev.val = r.u8v();
        log.push_back(ev);
    }
}

void R128Cell::snapSave(SnapWriter& w) const
{
    saveMapU32(w, regs_);
    saveMapU32(w, seen_);
    w.u32v(pllAddr_);
    saveMapU32(w, pll_);
    w.u32v(palIdx_);
    w.arr32(pal_, 256);
    // The vertical-blank clock. A resume that lands inside an armed blank
    // period has to keep its phase, or the first tick after the load re-bases
    // and the driver sees one stretched frame. The vblank/irq/ack COUNTERS are
    // deliberately not saved: a snapshotted census mixed with a live one is how
    // an OS-era write count came to be read against a whole-boot read count.
    w.u64v(vblNextTb_);
    w.u64v(tbNow_);
    w.u64v(log.size());
    for (const Ev& ev : log) {
        w.u64v(ev.at);
        w.u32v(ev.off);
        w.u32v(ev.val);
        w.u32v(ev.pc);
        w.b(ev.wr);
    }
    w.u64v(vram.size());
    w.raw(vram.data(), vram.size());
}

void R128Cell::snapLoad(SnapReader& r)
{
    loadMapU32(r, regs_);
    loadMapU32(r, seen_);
    pllAddr_ = r.u32v();
    loadMapU32(r, pll_);
    palIdx_ = r.u32v();
    r.arr32(pal_, 256);
    vblNextTb_ = r.u64v();
    tbNow_ = r.u64v();
    const u64 n = r.u64v();
    log.clear();
    for (u64 k = 0; k < n && r.ok; ++k) {
        Ev ev{};
        ev.at = r.u64v();
        ev.off = r.u32v();
        ev.val = r.u32v();
        ev.pc = r.u32v();
        ev.wr = r.b();
        log.push_back(ev);
    }
    bytesInPlace(r, vram, "framebuffer");
}

// --- the bus --------------------------------------------------------------

namespace {

void saveRegWr(SnapWriter& w, const std::vector<SawtoothBus::RegWr>& v)
{
    w.u64v(v.size());
    for (const auto& e : v) {
        w.u64v(e.at);
        w.u32v(e.pa);
        w.u32v(e.val);
        w.u32v(e.pc);
    }
}
void loadRegWr(SnapReader& r, std::vector<SawtoothBus::RegWr>& v)
{
    const u64 n = r.u64v();
    v.clear();
    for (u64 k = 0; k < n && r.ok; ++k) {
        SawtoothBus::RegWr e{};
        e.at = r.u64v();
        e.pa = r.u32v();
        e.val = r.u32v();
        e.pc = r.u32v();
        v.push_back(e);
    }
}
void saveAccMap(SnapWriter& w, const std::map<u32, SawtoothBus::Acc>& m)
{
    w.u64v(m.size());
    for (const auto& [pa, a] : m) {
        w.u32v(pa);
        w.u64v(a.firstAt);
        w.u32v(a.firstPc);
        w.u32v(a.lastWr);
        w.u64v(a.reads);
        w.u64v(a.writes);
    }
}
void loadAccMap(SnapReader& r, std::map<u32, SawtoothBus::Acc>& m)
{
    m.clear();
    const u64 n = r.u64v();
    for (u64 k = 0; k < n && r.ok; ++k) {
        const u32 pa = r.u32v();
        SawtoothBus::Acc a{};
        a.firstAt = r.u64v();
        a.firstPc = r.u32v();
        a.lastWr = r.u32v();
        a.reads = r.u64v();
        a.writes = r.u64v();
        m[pa] = a;
    }
}

} // namespace

void SawtoothBus::snapSave(SnapWriter& w) const
{
    // Memory and the two images the machine is built from. The images are
    // written in full so a resume can verify it is the same machine rather
    // than trusting the command line to be repeated correctly.
    size_t sec = w.begin("MEM ");
    w.u64v(ram_.size());
    w.raw(ram_.data(), ram_.size());
    w.bytes(rom_);
    w.u64v(romBase_); // the ROM this run STARTED from - see snapLoad
    w.bytes(atiRom_);
    w.end(sec);

    sec = w.begin("BUS ");
    w.raw(unin_, kUniNSize);
    w.u64v(kl_.size());
    w.raw(kl_.data(), kl_.size());
    // The KeyLargo timer is not in kl_ — it is derived from the timebase, so
    // what has to travel is where it was last reloaded and to what. klTb_ is
    // restored from the CPU's timebase by the machine's own load, not stored.
    w.u64v(klTimerTb_);
    w.u64v(klTimerVal_);
    w.u32v(sccPtr_[0]);
    w.u32v(sccPtr_[1]);
    w.raw(sccWr_, sizeof sccWr_);
    w.raw(sccRr_, sizeof sccRr_);
    w.str(console_);
    w.str(rxQueue_);
    w.u64v(rxNextAt_);
    for (u32 n = 0; n < 2; ++n) {
        w.u8v(i2c_[n].mode);
        w.u8v(i2c_[n].ctrl);
        w.u8v(i2c_[n].isr);
        w.u8v(i2c_[n].addr);
        w.u8v(i2c_[n].sub);
        w.b(i2c_[n].acked);
    }
    w.arr32(cfgAddr_, 3);
    saveMapU32(w, cfgSpace_);
    w.u32v(ohciBar_[0]);
    w.u32v(ohciBar_[1]);
    w.u32v(macioBar_);
    w.u32v(atiFbBar_);
    w.u32v(atiRegBar_);
    w.u32v(atiRomBar_);
    w.u64v(atiVisibleAt);
    w.u32v(watchPa);
    w.u32v(watchPaEnd);
    w.u32v(watchHits);
    w.end(sec);

    sec = w.begin("LOGS");
    saveRegWr(w, szLog_);
    saveRegWr(w, i2cLog_);
    saveRegWr(w, uninLog_);
    saveRegWr(w, cfgLog_);
    saveRegWr(w, ataLog_);
    saveAccMap(w, log_);
    saveAccMap(w, klLog_);
    saveVecU32(w, logOrder);
    saveVecU32(w, macioOrder);
    w.end(sec);

    sec = w.begin("ATA0");
    cd_.snapSave(w);
    w.end(sec);
    sec = w.begin("ATA1");
    hd_.snapSave(w);
    w.end(sec);
    sec = w.begin("PIC ");
    pic_.snapSave(w);
    w.end(sec);
    sec = w.begin("OHC0");
    ohci_[0].snapSave(w);
    w.end(sec);
    sec = w.begin("OHC1");
    ohci_[1].snapSave(w);
    w.end(sec);
    sec = w.begin("DBDM");
    ataDma_.snapSave(w);
    w.end(sec);
    sec = w.begin("DBD2");
    hdDma_.snapSave(w);
    w.end(sec);
    sec = w.begin("PMU ");
    pmu_.snapSave(w);
    w.end(sec);
    sec = w.begin("R128");
    ati_.snapSave(w);
    w.end(sec);
}

void SawtoothBus::snapLoad(SnapReader& r)
{
    const u8* e = r.beginSection("MEM ");
    bytesInPlace(r, ram_, "RAM");
    {
        std::vector<u8> img;
        r.bytes(img);
        // The boot flash is WRITABLE - Open Firmware keeps its NVRAM
        // partition inside it and rewrites the checksum on the way up - so
        // by the time a snapshot is taken the image no longer matches the
        // file it was loaded from, and comparing the LIVE images rejected
        // every snapshot this machine has ever produced. The identity that
        // matters is the ROM the run STARTED from, which is what romBase_
        // digests; the live image is machine state and gets restored.
        const u64 base = r.u64v();
        if (r.ok && base != romBase_)
            r.fail("the boot ROM in this run is not the one the snapshot "
                   "was taken with");
        else if (r.ok && img.size() != rom_.size())
            r.fail("the boot ROM in this run is a different size than the "
                   "snapshot's");
        else if (r.ok)
            rom_ = std::move(img);
        r.bytes(img);
        if (r.ok) {
            if (!atiRom_.empty() && atiRom_ != img)
                r.fail("the ATI FCode image differs from the snapshot's");
            else
                atiRom_ = img; // resume without repeating --ati-rom
        }
    }
    r.endSection("MEM ", e);

    e = r.beginSection("BUS ");
    r.raw(unin_, kUniNSize);
    bytesInPlace(r, kl_, "mac-io register store");
    klTimerTb_ = r.u64v();
    klTimerVal_ = r.u64v();
    sccPtr_[0] = r.u32v();
    sccPtr_[1] = r.u32v();
    r.raw(sccWr_, sizeof sccWr_);
    r.raw(sccRr_, sizeof sccRr_);
    console_ = r.str();
    rxQueue_ = r.str();
    rxNextAt_ = r.u64v();
    for (u32 n = 0; n < 2; ++n) {
        i2c_[n].mode = r.u8v();
        i2c_[n].ctrl = r.u8v();
        i2c_[n].isr = r.u8v();
        i2c_[n].addr = r.u8v();
        i2c_[n].sub = r.u8v();
        i2c_[n].acked = r.b();
    }
    r.arr32(cfgAddr_, 3);
    loadMapU32(r, cfgSpace_);
    ohciBar_[0] = r.u32v();
    ohciBar_[1] = r.u32v();
    macioBar_ = r.u32v();
    atiFbBar_ = r.u32v();
    atiRegBar_ = r.u32v();
    atiRomBar_ = r.u32v();
    atiVisibleAt = r.u64v();
    watchPa = r.u32v();
    watchPaEnd = r.u32v();
    watchHits = r.u32v();
    r.endSection("BUS ", e);

    e = r.beginSection("LOGS");
    loadRegWr(r, szLog_);
    loadRegWr(r, i2cLog_);
    loadRegWr(r, uninLog_);
    loadRegWr(r, cfgLog_);
    loadRegWr(r, ataLog_);
    loadAccMap(r, log_);
    loadAccMap(r, klLog_);
    loadVecU32(r, logOrder);
    loadVecU32(r, macioOrder);
    r.endSection("LOGS", e);

    e = r.beginSection("ATA0");
    cd_.snapLoad(r);
    r.endSection("ATA0", e);
    e = r.beginSection("ATA1");
    hd_.snapLoad(r);
    r.endSection("ATA1", e);
    e = r.beginSection("PIC ");
    pic_.snapLoad(r);
    r.endSection("PIC ", e);
    e = r.beginSection("OHC0");
    ohci_[0].snapLoad(r);
    r.endSection("OHC0", e);
    e = r.beginSection("OHC1");
    ohci_[1].snapLoad(r);
    r.endSection("OHC1", e);
    e = r.beginSection("DBDM");
    ataDma_.snapLoad(r);
    r.endSection("DBDM", e);
    e = r.beginSection("DBD2");
    hdDma_.snapLoad(r);
    r.endSection("DBD2", e);
    e = r.beginSection("PMU ");
    pmu_.snapLoad(r);
    r.endSection("PMU ", e);
    e = r.beginSection("R128");
    ati_.snapLoad(r);
    r.endSection("R128", e);

    // Pointers the constructor wired stay valid because RAM was restored in
    // place; re-assert the one that is derived from a container's storage.
    for (u32 f = 0; f < 2; ++f) {
        ohci_[f].ram = ram_.data();
        ohci_[f].ramSize = static_cast<u32>(ram_.size());
    }
    ataDma_.dmaBus = this;
    ataDma_.ata = &cd_;
    hdDma_.dmaBus = this;
    hdDma_.ata = &hd_;
    // The device-service gate is a CACHE of "when could an interrupt line next
    // change", and every device it summarises has just been replaced. Say
    // "unknown" rather than carry a deadline computed for a different machine:
    // a stale one would withhold an interrupt until the guest happened to
    // touch a register. Nothing here needs to be in the stream — this restores
    // the invariant instead of the value.
    devGenSeen_ = ~0ull;
    devDueTb_ = 0;
    devDueStamp_ = 0;
    // Likewise the KeyLargo timer's notion of "now", which the run loop
    // refreshes on the first serviceDevices call. Starting it at the reload
    // point makes the first read report the reloaded value rather than a
    // count derived from a timebase this machine has not reached yet.
    klTb_ = klTimerTb_;
}

// --- whole machine --------------------------------------------------------

void saveSnapshot(const Cpu& cpu, const SawtoothBus& bus, const HarnessState& h,
                  SnapWriter& w)
{
    w.u32v(kSnapMagic);
    w.u32v(kSnapVersion);
    w.u64v(layoutDigest());

    saveCpu(w, cpu);
    bus.snapSave(w);

    const size_t sec = w.begin("HARN");
    w.u64v(h.executed);
    w.u32v(h.fastTb);
    w.u64v(h.fastTbUntil);
    w.u32v(h.parkSeen);
    w.b(h.parkArmed);
    w.b(h.ataPoked);
    w.b(h.emPoked);
    w.end(sec);

    const size_t fin = w.begin("END ");
    w.end(fin);
}

bool loadSnapshot(Cpu& cpu, SawtoothBus& bus, HarnessState& h, SnapReader& r)
{
    if (r.u32v() != kSnapMagic) {
        r.fail("not an OpenPowerMac snapshot");
        return false;
    }
    const u32 ver = r.u32v();
    if (ver != kSnapVersion) {
        r.fail("snapshot version " + std::to_string(ver) + ", this build "
               "writes " + std::to_string(kSnapVersion));
        return false;
    }
    if (r.u64v() != layoutDigest()) {
        r.fail("snapshot was written by a build with a different state "
               "layout — refusing rather than misreading it");
        return false;
    }

    loadCpu(r, cpu);
    bus.snapLoad(r);

    const u8* e = r.beginSection("HARN");
    h.executed = r.u64v();
    h.fastTb = r.u32v();
    h.fastTbUntil = r.u64v();
    h.parkSeen = r.u32v();
    h.parkArmed = r.b();
    h.ataPoked = r.b();
    h.emPoked = r.b();
    r.endSection("HARN", e);

    e = r.beginSection("END ");
    r.endSection("END ", e);
    if (r.ok && r.p != r.end)
        r.fail("trailing bytes after the end marker");
    return r.ok;
}

u64 snapshotFingerprint(const Cpu& cpu, const SawtoothBus& bus,
                        const HarnessState& h)
{
    SnapWriter w;
    saveSnapshot(cpu, bus, h, w);
    return fnv1a(w.buf.data(), w.buf.size());
}

bool writeSnapshotFile(const char* path, const std::vector<u8>& blob)
{
    FILE* f = fopen(path, "wb");
    if (!f) {
        fprintf(stderr, "snapshot: cannot create %s\n", path);
        return false;
    }
    const bool ok = fwrite(blob.data(), 1, blob.size(), f) == blob.size();
    fclose(f);
    if (!ok)
        fprintf(stderr, "snapshot: short write on %s\n", path);
    return ok;
}

bool readSnapshotFile(const char* path, std::vector<u8>& blob)
{
    FILE* f = fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "snapshot: cannot open %s\n", path);
        return false;
    }
    fseek(f, 0, SEEK_END);
    const long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    blob.assign(static_cast<size_t>(n > 0 ? n : 0), 0);
    const bool ok =
        !blob.empty() && fread(blob.data(), 1, blob.size(), f) == blob.size();
    fclose(f);
    if (!ok)
        fprintf(stderr, "snapshot: short read on %s\n", path);
    return ok;
}

} // namespace opm
