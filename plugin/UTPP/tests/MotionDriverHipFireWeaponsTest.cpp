// P9 (recording 2026-10-21 12:15): hipfire with a weapon that has no recoil route (rifle, SMG, shotgun, every pistol but the Nue preset
// outside the native FK stance) must keep the authored Aim arms through the shot and the 0.75 s hold. Before the fix the native
// first-burst fallback (RecordGripInfoLocked -> HipFireNativeActionOwner) published upperScale 0 (FPP arms, ~100 degrees off the MM
// shoulders) on every act=8 frame and snapped back to 1 in one frame (log: 41/41 rifle episodes, |d upper| = 1.00 at both edges).
// Runs the hipfire-* cases of the integration test (MaskHandoverChildMain "hipfire-" block: ready, a 0.217 s shot, the hold, a burst of
// four 0.1 s shots, the tail and a first shot without hold) in child processes of this exe. Every frame of the weapons without a route
// needs upperScale > 0.99 with a step <= 0.01, the hipfire pose during the shot and the hold, shootRoute=0 and no applied transfer.
// Controls: hipfire-nue2h* (the authored recoil route owns the arms, route stays open). hipfire-pistol2h-other-still is the revolver
// regression (recording 2026-10-25, Overture): Pistol2H standing still WITHOUT a recoil profile no longer has a native FK route, so the
// arms stay authored (UTPP_MOTION_DRIVER_HIPFIRE_ROUTE_MUTATION=3 = the old route must fail it and the hipfire-route-* cases).
// Male V cases (suffix -man, W5 2026-11-06) run the same assertions on the man bundle; the man bundle needs no other expectation.
// Usage: utpp_motion_driver_hipfire_weapons_test <data dir> [ignored]
// UTPP_MOTION_DRIVER_HIPFIRE_ROUTE_MUTATION=1 (the route gate is always open, the pre-P9 behaviour) must print a FAIL line.
#define main UnusedIntegrationMain
#include "MotionDriverIntegrationTest.cpp"
#undef main

int main(int argc, char** argv)
{
    if (argc >= 2 && std::string(argv[1]) == "--weapon-upper-child")
        return UnusedIntegrationMain(argc, argv);
    if (argc < 2)
    {
        std::puts("usage: utpp_motion_driver_hipfire_weapons_test <data dir>");
        return 2;
    }
    const std::string data = argv[1];
    const std::string v1 = data + "\\utpp_base.utppmm";
    struct Run { const char* variant; int hz; };
    std::vector<Run> runs;
    for (const char* variant : {"hipfire-rifle", "hipfire-rifle-still", "hipfire-rifle-crouch", "hipfire-rifle-air", "hipfire-rifle-slide",
                                "hipfire-rifle-man", "hipfire-pistol2h-other", "hipfire-pistol2h-other-crouch",
                                "hipfire-pistol2h-other-slide", "hipfire-pistol1h-other", "hipfire-pistol1h-other-crouch",
                                "hipfire-pistol2h-other-still", "hipfire-nue2h", "hipfire-nue2h-still",
                                // Male V (W5 2026-11-06): the same weapons on the man database (MM man bundle + overlays_man; the slide
                                // variants on utpp_motion_man_knee_slide), incl. the crouched Pistol2H / Pistol1H hipfire
                                "hipfire-rifle-crouch-man", "hipfire-rifle-slide-man", "hipfire-pistol2h-other-man",
                                "hipfire-pistol2h-other-crouch-man", "hipfire-pistol2h-other-slide-man", "hipfire-pistol1h-other-crouch-man",
                                "hipfire-pistol2h-other-still-man", "hipfire-nue2h-man"})
        runs.push_back({variant, 60});
    for (int hz : {30, 144})
        for (const char* variant : {"hipfire-rifle", "hipfire-nue2h-still", "hipfire-pistol2h-other-crouch-man"})
            runs.push_back({variant, hz});
    // Fix batch 3, authored recoil profiles (step B of P9): a weapon WITH a profile (rifle: Ajax, pistols: Liberty) keeps the authored
    // arms (upperScale 1) and owns them with its own recoil episode, in every stance; the kick is applied (shootApplied grows).
    for (const char* variant : {"hipfire-rifle-profile", "hipfire-rifle-profile-still", "hipfire-rifle-profile-crouch", "hipfire-rifle-profile-air",
                                "hipfire-rifle-profile-slide", "hipfire-rifle-profile-man", "hipfire-pistol2h-profile",
                                "hipfire-pistol2h-profile-still", "hipfire-pistol2h-profile-crouch", "hipfire-pistol2h-profile-slide",
                                "hipfire-pistol1h-profile", "hipfire-pistol1h-profile-crouch",
                                "nue-recoil-hipfire-rifle", "nue-recoil-ads-rifle", "nue-recoil-hipfire-2h-liberty", "nue-recoil-ads-2h-liberty",
                                // the review's worst case, in full-auto (1 s at 12 rounds/s) through the real driver: the burst rifle Masamune
                                // (19.8 cm raw, 8 cm after the budget), and a revolver (Quasar): no clamp, bounded steps, the IK follows the kick
                                "hipfire-rifle-masamune-profile", "hipfire-rifle-masamune-profile-crouch", "hipfire-pistol1h-quasar-profile",
                                // an id outside the table, and a retired one (the Overture hammer-cock clip), behave like a weapon without a
                                // profile: authored aim arms, no route
                                "hipfire-pistol1h-retired", "hipfire-pistol2h-retired-crouch",
                                "hipfire-rifle-badprofile", "hipfire-pistol2h-badprofile-crouch",
                                // Male V: the same profile cases on the man database
                                "hipfire-pistol2h-profile-man", "hipfire-pistol2h-profile-crouch-man", "hipfire-pistol1h-profile-crouch-man",
                                "nue-recoil-hipfire-rifle-man", "nue-recoil-ads-2h-liberty-man", "nue-recoil-hipfire-2h-man",
                                "hipfire-rifle-masamune-profile-man", "hipfire-pistol1h-quasar-profile-man", "hipfire-pistol2h-badprofile-crouch-man"})
        runs.push_back({variant, 60});
    for (int hz : {30, 144})
        runs.push_back({"hipfire-rifle-profile", hz});
    // the route is decided once per shot: a micro-step in the middle of a standing Pistol2H shot, a stop in the middle of a moving one
    for (int hz : {30, 60, 144})
        for (const char* variant : {"hipfire-route-pulse", "hipfire-route-stop"})
            runs.push_back({variant, hz});
    for (const char* variant : {"hipfire-route-pulse-man", "hipfire-route-stop-man"})
        runs.push_back({variant, 60});
    // All variants are submitted to the child pool at once (they run in parallel, bounded by the pool), then evaluated in the order above.
    const bool baseExists = Exists(v1);
    std::vector<ChildRef> kids;
    for (const Run& run : runs) // the child strips the v1-/v2- feed prefix
        kids.push_back(SubmitModeChild("hipfireweapons", L"--weapon-upper-child", data, v1,
                                       L" " + std::to_wstring(run.hz) + L" v2-" + Wide(run.variant), baseExists));
    for (size_t runIndex = 0; runIndex < runs.size(); ++runIndex)
    {
        const Run& run = runs[runIndex];
        const std::string& out = kids[runIndex].out;
        const DWORD code = WaitChild(kids[runIndex]);
        std::string head, phases, line;
        {
            std::ifstream in(out);
            std::getline(in, head);
            while (std::getline(in, line))
                if (line.rfind("phase=", 0) == 0)
                    phases += line.substr(0, 170) + " | ";
                else if (line.rfind("trace=", 0) == 0)
                    phases += line.substr(0, 700);
        }
        DeleteFileA(out.c_str());
        if (head.size() > 700)
            head.resize(700);
        const bool routeCase = std::string(run.variant).find("hipfire-route-") == 0;
        const bool control = std::string(run.variant).find("nue2h") != std::string::npos;
        const bool profile = std::string(run.variant).find("-profile") != std::string::npos ||
                             std::string(run.variant).find("nue-recoil-") == 0;
        Check(code == 0, std::string(run.variant) + " at " + std::to_string(run.hz) +
              (routeCase ? "Hz: the route is decided once per shot, no extra arm switches from a micro-step / a stop (exit code " :
               profile ? "Hz: a weapon with a recoil profile owns the arms with its authored recoil (upperScale 1, recoil owner, applied transfers) (exit code " :
               control ? "Hz: control, the route stays open and the Nue recoil / native FK behaviour is unchanged (exit code " :
                         "Hz: a weapon without a recoil route keeps upperScale 1 and the hipfire pose through the shot and the hold (exit code ") +
              std::to_string(code) + ")", head + " | " + phases);
    }
    std::printf("hipfire weapons: %d PASS %d FAIL\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
