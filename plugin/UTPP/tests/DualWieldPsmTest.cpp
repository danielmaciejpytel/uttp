// Dual-wield D0 offline test: the psmBlock dual flag (src/mm/GameIntegration.hpp kPsmDualFlag, bit 31) and its consumers.
// Usage: utpp_dualwield_psm_test   Exit code 0 = all checks passed.
//
// What it proves:
//  A  encode / decode round trip for every UpperBody value (-1, 0..8 incl. 7 = DualWield), flag set / unset, over a set of
//     other-field patterns (vehicle, detail, melee bits, melee state, weapon bits): the raw decoder returns the true UpperBody,
//     every other field decodes unchanged.
//  B  behaviour identity: the legacy consumers (PsmUpperBodyOf, PsmBlockBits, ComputeGate, the aim-offset guard) see exactly the
//     values they saw before the flag existed - UpperBody 7 stays -1, the flag never reaches the gate bits.
//  C  the status suffix is parseable by tools/gametest/gt_log.py FIELD_RE ('(?:^|\s)([A-Za-z|]+)=(\S+)') and carries the values.
//
// PackPsm mirrors the UpperBody / dual lines of scripts/utpp.reds UTPPPsmBlockMask (the redscript itself cannot run here; it is
// covered by the offline redscript compile and the in-game [utpp_dual] log).
// Mutations (CMake targets utpp_dualwield_psm_test_mut1 / _mut2, UTPP_DUALWIELD_MUTATION): 1 = the flag is not stripped from the
// gate bits, 2 = PsmUpperBodyOf reports 7 for a dual-wield mask. Both must make this test fail.
#include <cstdint>
#include <cstdio>
#include <regex>
#include <string>
#include <vector>

#include "../src/mm/AimOffset.hpp"
#include "../src/mm/GameIntegration.hpp"

using namespace utpp::mm;

static int g_fail = 0;
static int g_pass = 0;

static void Expect(const char* aName, bool aOk, const std::string& aDetail = {})
{
    if (aOk)
        ++g_pass;
    else
    {
        ++g_fail;
        std::printf("FAIL %s%s%s\n", aName, aDetail.empty() ? "" : " -- ", aDetail.c_str());
    }
}

// Mirror of utpp.reds UTPPPsmBlockMask: `if upper >= 0 && upper < 7 { mask |= (upper + 1) * 8192 }` and
// `if dualWield { mask |= sign bit }` (dualWield = UpperBody == 7 || PSM.IsDualWielding).
static uint32_t PackUpper(uint32_t aOther, int aUpper, bool aDual)
{
    uint32_t m = aOther;
    if (aUpper >= 0 && aUpper < 7)
        m |= uint32_t(aUpper + 1) * 8192u;
    if (aDual)
        m |= 0x80000000u;
    return m;
}

// The decode that existed before this change (independent copy, not the production function).
static int OldUpperBodyOf(uint32_t aMask) { return int((aMask & (0x7u << 13)) >> 13) - 1; }
static uint32_t OldBlockBits(uint32_t aMask)
{
    const uint32_t diag = (0xFFu << 16) | (0x7u << 13) | (0x1Fu << 24) | (1u << 29) | (1u << 30);
    return aMask & ~(diag | (1u << 11) | (1u << 12));
}

int main()
{
    static_assert(integ::kPsmDualFlag == 0x80000000u, "bit 31 is the only free bit of the packed mask");
    // The occupied fields do not overlap bit 31 (13..15, 16..23, 24..28, 29, 30, gate bits 0..12).
    static_assert((integ::kPsmUpperFieldMask | integ::kPsmDetailFieldMask | integ::kPsmMeleeStateFieldMask | integ::kPsmWeaponKnown |
                   integ::kPsmWeaponDrawn | 0x1FFFu) == 0x7FFFFFFFu, "bits 0..30 are taken");

    // other-field patterns: none, vehicle, a detail field, melee bits, melee state, weapon bits, a mix
    const std::vector<uint32_t> others = {
        0u,
        integ::kPsmVehicle,
        integ::kPsmDetailed | (uint32_t(14 + 1) << integ::kPsmDetailShift),
        integ::kPsmMeleeAttack,
        integ::kPsmMeleePose | (uint32_t(9 + 1) << integ::kPsmMeleeStateShift),
        integ::kPsmWeaponKnown | integ::kPsmWeaponDrawn,
        integ::kPsmVehicle | integ::kPsmDetailed | (uint32_t(3 + 1) << integ::kPsmDetailShift) | integ::kPsmMeleeAttack |
            (uint32_t(12 + 1) << integ::kPsmMeleeStateShift) | integ::kPsmWeaponKnown,
    };

    for (const uint32_t other : others)
    {
        for (int upper = -1; upper <= 8; ++upper)
        {
            for (int dualFlag = 0; dualFlag <= 1; ++dualFlag)
            {
                // reds: dualWield is true whenever UpperBody == 7 (so 7 without the flag cannot happen); the flag may also come
                // from PSM.IsDualWielding with any UpperBody.
                const bool dual = dualFlag != 0 || upper == 7;
                const uint32_t m = PackUpper(other, upper, dual);
                char tag[96];
                std::snprintf(tag, sizeof tag, "other=0x%X upper=%d flag=%d", other, upper, dualFlag);
                const std::string t = tag;

                // ---- A: round trip
                const bool inField = upper >= 0 && upper < 7;
                Expect(("A raw upper " + t).c_str(),
                       integ::PsmUpperBodyRawOf(m) == (inField ? upper : (upper == 7 ? 7 : -1)) ||
                           // UpperBody outside 0..7 with the flag from IsDualWielding is documented as inferred 7 (never occurs in game)
                           (!inField && upper != 7 && dual && integ::PsmUpperBodyRawOf(m) == 7),
                       std::to_string(integ::PsmUpperBodyRawOf(m)));
                Expect(("A dual flag " + t).c_str(), integ::PsmDualOf(m) == dual);
                Expect(("A other fields " + t).c_str(),
                       integ::PsmDetailOf(m) == integ::PsmDetailOf(other) &&
                           integ::PsmMeleeStateOf(m) == integ::PsmMeleeStateOf(other) &&
                           integ::PsmMeleeBits(m) == integ::PsmMeleeBits(other) &&
                           integ::PsmWeaponKnown(m) == integ::PsmWeaponKnown(other) &&
                           integ::PsmWeaponDrawn(m) == integ::PsmWeaponDrawn(other));

                // ---- B: identity for the legacy consumers
                Expect(("B PsmUpperBodyOf == old decode " + t).c_str(), integ::PsmUpperBodyOf(m) == OldUpperBodyOf(m),
                       std::to_string(integ::PsmUpperBodyOf(m)));
                Expect(("B PsmUpperBodyOf of 7 is -1 " + t).c_str(), upper != 7 || integ::PsmUpperBodyOf(m) == -1);
                Expect(("B PsmUpperBodyOf of 0..6 unchanged " + t).c_str(), !inField || integ::PsmUpperBodyOf(m) == upper);
                Expect(("B gate bits == old " + t).c_str(), integ::PsmBlockBits(m) == OldBlockBits(m & 0x7FFFFFFFu), // old reds never set bit 31
                       std::to_string(integ::PsmBlockBits(m)));
                Expect(("B gate bits independent of the flag " + t).c_str(),
                       integ::PsmBlockBits(m) == integ::PsmBlockBits(PackUpper(other, upper, false)));
                integ::GateInput gi;
                gi.mmEnabled = gi.dbLoaded = gi.mapped = gi.tppActive = gi.matcherReady = true;
                gi.locoState = integ::kLocoStand;
                gi.psmBlock = integ::PsmBlockBits(m);
                integ::GateInput gj = gi;
                gj.psmBlock = integ::PsmBlockBits(PackUpper(other, upper, false));
                Expect(("B ComputeGate unchanged by the flag " + t).c_str(), integ::ComputeGate(gi) == integ::ComputeGate(gj),
                       std::to_string(integ::ComputeGate(gi)) + " vs " + std::to_string(integ::ComputeGate(gj)));

                // aim-offset guard (consumer of the raw mask): identical with and without the flag
                aim::GuardInput g;
                g.targetEnabled = g.targetValid = g.tpp = g.rigOk = true;
                g.targetAge = g.feedAge = 0.016f;
                g.maskStates = 7;
                g.upperScale = 0.0f;
                g.mmWeight = 1.0f;
                g.psm = m;
                aim::GuardInput g0 = g;
                g0.psm = PackUpper(other, upper, false);
                Expect(("B aim guard unchanged by the flag " + t).c_str(), aim::ComputeGuard(g) == aim::ComputeGuard(g0),
                       std::to_string(aim::ComputeGuard(g)) + " vs " + std::to_string(aim::ComputeGuard(g0)));
            }
        }
    }

    // explicit headline cases
    {
        const uint32_t seven = PackUpper(0, 7, true);
        Expect("UpperBody 7: legacy -1", integ::PsmUpperBodyOf(seven) == -1);
        Expect("UpperBody 7: raw 7", integ::PsmUpperBodyRawOf(seven) == 7);
        Expect("UpperBody 7: gate bits 0", integ::PsmBlockBits(seven) == 0u);
        Expect("UpperBody 6 (Aim): legacy 6, flag clear", integ::PsmUpperBodyOf(PackUpper(0, 6, false)) == 6 &&
                                                              !integ::PsmDualOf(PackUpper(0, 6, false)));
        Expect("not reported: raw -1", integ::PsmUpperBodyRawOf(0u) == -1 && !integ::PsmDualOf(0u));
        Expect("IsDualWielding with UpperBody 0..6: raw is the real value, flag set",
               integ::PsmUpperBodyRawOf(PackUpper(0, 2, true)) == 2 && integ::PsmDualOf(PackUpper(0, 2, true)));
        // Int32 round trip as the native receives it (reds Int32 -> static_cast<uint32_t>)
        const int32_t viaInt32 = static_cast<int32_t>(seven);
        Expect("sign bit survives the Int32 round trip", viaInt32 < 0 && static_cast<uint32_t>(viaInt32) == seven);
    }

    // ---- C: status suffix
    {
        char buf[96];
        const std::regex fieldRe(R"((?:^|\s)([A-Za-z|]+)=(\S+))");
        struct Case { uint32_t mask; const char* dual; const char* raw; };
        const Case cases[] = {{PackUpper(0, 7, true), "1", "7"}, {PackUpper(0, 6, false), "0", "6"},
                              {PackUpper(0, 3, true), "1", "3"}, {0u, "0", "-1"}};
        for (const Case& c : cases)
        {
            integ::PsmDualStatusSuffix(buf, sizeof buf, c.mask);
            const std::string s = std::string("UTPP MOTION enabled=1 aim=tgt00") + buf;
            std::string dual, raw;
            for (auto it = std::sregex_iterator(s.begin(), s.end(), fieldRe); it != std::sregex_iterator(); ++it)
            {
                if ((*it)[1] == "dualWield")
                    dual = (*it)[2];
                else if ((*it)[1] == "upperBodyRaw")
                    raw = (*it)[2];
            }
            Expect("C status suffix parses", dual == c.dual && raw == c.raw, s);
        }
    }

    std::printf("dualwield psm test: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
