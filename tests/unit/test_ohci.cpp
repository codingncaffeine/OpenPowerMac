// The USB HID boot keyboard's report state.
//
// This deserves a test rather than a boot because the failure it guards
// against is silent: a report whose modifier byte or key slots are wrong is
// still a well-formed 8-byte report, the guest accepts it, and the only
// symptom is that typing does nothing or that a modifier appears stuck. The
// shell reaches this through opm_key_event, so the report bytes are the
// contract between a host keypress and the guest's Key Map.

#include "doctest.h"
#include "opm/ohci.hpp"

#include <vector>

using namespace opm;

namespace {

// Drain what the cell has queued for its interrupt IN endpoint. The reports
// are 8 bytes each and go out one per poll, which is why this reads them back
// as whole reports rather than as a byte stream.
std::vector<std::vector<u8>> reports(const OhciCell& kb)
{
    std::vector<std::vector<u8>> out;
    const std::vector<u8>& raw = kb.queuedReports();
    for (size_t k = 0; k + 8 <= raw.size(); k += 8)
        out.push_back(std::vector<u8>(raw.begin() + k, raw.begin() + k + 8));
    return out;
}

} // namespace

TEST_CASE("a key going down and coming up reports what is held")
{
    OhciCell kb;
    kb.setHid(OhciCell::Hid::Keyboard);

    kb.keyEvent(4, true);  // 'a'
    kb.keyEvent(4, false);
    const auto r = reports(kb);
    REQUIRE(r.size() == 2);
    CHECK(r[0][0] == 0x00); // no modifier
    CHECK(r[0][2] == 4);    // first key slot holds the usage
    CHECK(r[1][2] == 0);    // and the release empties it
}

TEST_CASE("a modifier is a bitmap, not a key slot")
{
    OhciCell kb;
    kb.setHid(OhciCell::Hid::Keyboard);

    kb.keyEvent(0xE1, true); // left shift
    kb.keyEvent(4, true);    // shift-A
    kb.keyEvent(4, false);
    kb.keyEvent(0xE1, false);
    const auto r = reports(kb);
    REQUIRE(r.size() == 4);
    CHECK(r[0][0] == 0x02); // shift alone: bit 1 set, no key held
    CHECK(r[0][2] == 0);
    CHECK(r[1][0] == 0x02); // the letter arrives WITH the modifier held
    CHECK(r[1][2] == 4);
    CHECK(r[2][0] == 0x02); // the letter goes, the modifier stays
    CHECK(r[2][2] == 0);
    CHECK(r[3][0] == 0x00);
}

TEST_CASE("keys held together occupy separate slots and release independently")
{
    OhciCell kb;
    kb.setHid(OhciCell::Hid::Keyboard);

    kb.keyEvent(4, true); // 'a'
    kb.keyEvent(5, true); // 'b'
    kb.keyEvent(4, false);
    const auto r = reports(kb);
    REQUIRE(r.size() == 3);
    CHECK(r[1][2] == 4);
    CHECK(r[1][3] == 5); // both held at once
    CHECK(r[2][2] == 0); // 'a' released, 'b' still down in its own slot
    CHECK(r[2][3] == 5);
}

TEST_CASE("a held key repeats no faster than the guest asks")
{
    OhciCell kb;
    kb.setHid(OhciCell::Hid::Keyboard);

    // A real keyboard reports a press ONCE and lets the host repeat it. The
    // shell filters the host's auto-repeat, and the cell must not invent one
    // of its own: pressing an already-held key adds no second slot.
    kb.keyEvent(4, true);
    kb.keyEvent(4, true);
    const auto r = reports(kb);
    REQUIRE(r.size() == 2);
    CHECK(r[1][2] == 4);
    CHECK(r[1][3] == 0);
}

TEST_CASE("the mouse cell ignores key events")
{
    OhciCell mouse;
    mouse.setHid(OhciCell::Hid::Mouse);
    mouse.keyEvent(4, true);
    CHECK(reports(mouse).empty());
}
