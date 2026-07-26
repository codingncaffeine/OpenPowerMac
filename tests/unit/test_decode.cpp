#include "doctest.h"
#include "opm/insn.hpp"
#include <cstring>
#include <string>

using namespace opm;

namespace {
std::string dis(u32 insn, u32 pc = 0x1000)
{
    char buf[128];
    disassemble(insn, pc, buf, sizeof buf, Style::Gnu);
    return buf;
}
} // namespace

TEST_CASE("decode + disassemble known encodings")
{
    CHECK(dis(0x7C221A14u) == "add r1, r2, r3");
    CHECK(dis(0x38600001u) == "li r3, 1");
    CHECK(dis(0x4E800020u) == "blr");
    CHECK(dis(0x7C0004ACu) == "sync");
    CHECK(dis(0x7C85312Du) == "stwcx. r4, r5, r6");
    CHECK(dis(0x10221800u) == "vaddubm v1, v2, v3");
    CHECK(dis(0x100110EBu) == "vperm v0, v1, v2, v3");
    CHECK(dis(0xEC2220FAu) == "fmadds f1, f2, f3, f4");
    CHECK(dis(0x7C0802A6u) == "mflr r0");
    CHECK(dis(0x54832834u) == "slwi r3, r4, 5");
    CHECK(dis(0x2C030005u) == "cmpwi r3, 5");
    CHECK(dis(0x7C4FF120u) == "mtcr r2");
    CHECK(dis(0x60000000u) == "nop");
    CHECK(dis(0x00000000u) == ".long 0x00000000");
}

TEST_CASE("branch target arithmetic")
{
    // bne cr1, pc+8 : bc BO=4, BI=6, BD=2
    CHECK(dis(0x40860008u, 0x2000) == "bne cr1, 0x2008");
    // b pc-4
    CHECK(dis(0x4BFFFFFCu, 0x2000) == "b 0x1ffc");
}

TEST_CASE("7400 unimplemented opcodes still decode for the disassembler")
{
    const InsnDesc* d = decode(0xFC20102Cu); // fsqrt f1, f2
    REQUIRE(d != nullptr);
    CHECK(std::strcmp(d->mnem, "fsqrt") == 0);
    CHECK((d->flags & FL_ILL7400) != 0);
    CHECK(dis(0xFC20102Cu) == "fsqrt f1, f2");

    const InsnDesc* t = decode(0x7C0002E4u); // tlbia
    REQUIRE(t != nullptr);
    CHECK((t->flags & FL_ILL7400) != 0);
}

TEST_CASE("OE folding registers both XO slots")
{
    // addo r1,r2,r3 = add with OE (xo 266+512)
    const InsnDesc* d = decode(0x7C221E14u);
    REQUIRE(d != nullptr);
    CHECK(std::strcmp(d->mnem, "add") == 0);
    CHECK(dis(0x7C221E14u) == "addo r1, r2, r3");
}

TEST_CASE("VXR record bit folds and prints dot")
{
    // vcmpequb. v0, v1, v2 : op4 xo=6, Rc(bit21)=1
    // (4<<26)|(0<<21)|(1<<16)|(2<<11)|(1<<10)|6
    CHECK(dis(0x10011406u) == "vcmpequb. v0, v1, v2");
    CHECK(dis(0x10011006u) == "vcmpequb v0, v1, v2");
}
