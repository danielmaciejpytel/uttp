// Admission of the reload contact correction per pose hook owner (reload_contacts_runtime::Admit / ObservesRifle). No data files.
// Default build: the FK owner keeps the game's reload arms for every overlay; the Mapping owner keeps its rule (every reload overlay,
// authored clips included). Build with UTPP_RELOAD_FK_OWNER_RIFLE=1: the FK owner also corrects game-clip Rifle reloads, never an
// authored reload and never a pistol.
#include "../src/mm/NativeReloadContactsRuntime.hpp"
#include <cstdio>
using namespace utpp::mm;
namespace rt = reload_contacts_runtime;

int main()
{
    int passed = 0, failed = 0;
    const auto check = [&](bool ok, const char* what) { (ok ? passed : failed)++; std::printf("%s %s\n", ok ? "PASS" : "FAIL", what); };
    const auto snap = [](int32_t overlay, bool eligible, bool authored) {
        integ::PoseSnapshot s{};
        s.traceOverlay = overlay; s.reloadContactsEligible = eligible; s.reloadAuthoredClip = authored && eligible;
        s.reloadContactsWeight = eligible ? 1.0f : 0.0f; s.traceAction = 2;
        return s;
    };
    const int32_t rifle = int32_t(ov::PoseOverlay::Rifle), p1 = int32_t(ov::PoseOverlay::Pistol1H), p2 = int32_t(ov::PoseOverlay::Pistol2H);
    const int32_t others[] = {int32_t(ov::PoseOverlay::Default), int32_t(ov::PoseOverlay::Injured), int32_t(ov::PoseOverlay::Game)};
    const auto fk = rt::HookOwner::FK, mapping = rt::HookOwner::Mapping;

    // Mapping owner: unchanged rule.
    check(rt::Admit(snap(rifle, true, false), mapping) == rt::kReloadContactsRifle, "mapping: rifle reload admitted");
    check(rt::Admit(snap(p1, true, false), mapping) && rt::Admit(snap(p2, true, false), mapping), "mapping: pistol reloads admitted");
    check(rt::Admit(snap(p2, true, true), mapping), "mapping: authored pistol reload admitted (unchanged)");
    check(rt::ObservesRifle(rifle, mapping) == rt::kReloadContactsRifle, "mapping: rifle published pose observed");
    // Never without eligibility or on a non-reload overlay.
    for (const auto owner : {fk, mapping})
    {
        check(!rt::Admit(snap(rifle, false, false), owner) && !rt::Admit(snap(p2, false, false), owner), "not eligible: never admitted");
        bool none = true;
        for (const int32_t o : others) none = none && !rt::Admit(snap(o, true, false), owner) && !rt::ObservesRifle(o, owner);
        check(none, "other overlays: never admitted or observed");
    }
    // FK owner: pistols (the solve fails the anatomy target on the recorded revolver) and authored reloads stay as the game shows them.
    check(!rt::Admit(snap(p1, true, false), fk) && !rt::Admit(snap(p2, true, false), fk), "fk: game-clip pistol reloads not corrected");
    check(!rt::Admit(snap(p2, true, true), fk) && !rt::Admit(snap(rifle, true, true), fk), "fk: authored reloads never corrected");
#if defined(UTPP_RELOAD_FK_OWNER_RIFLE) && UTPP_RELOAD_FK_OWNER_RIFLE == 1
    check(rt::Admit(snap(rifle, true, false), fk), "fk (rifle switch): game-clip rifle reload corrected");
    check(rt::ObservesRifle(rifle, fk), "fk (rifle switch): rifle published pose observed");
#else
    check(!rt::Admit(snap(rifle, true, false), fk), "fk (default): game-clip rifle reload keeps the game arms");
    check(!rt::ObservesRifle(rifle, fk), "fk (default): no rifle observation");
#endif
    std::printf("passed=%d failed=%d\n", passed, failed);
    return failed ? 1 : 0;
}
