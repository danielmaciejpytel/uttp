// fix2/hip P8 (recording 2026-10-21 12:15): hipfire in the v2 slide (chooser MovementMode 2) must keep the authored weapon recoil and
// the MM arms. Before the fix authoredDomain admitted only MovementMode 0 / 1, so every slide shot reset the authored recoil and the
// native action fallback published upperScale 0 (FPP arms) for the whole shot (log: shootApplied 0/41, shootEvaluations stuck).
// Runs the v2-nue-recoil-hipfire-{2h,1h}-slide cases of the integration test (MaskHandoverChildMain nue-recoil block: a ground shot
// carried into the slide, a new shot in the slide, the frames between shots, a slide shot carried into the crouch) in child
// processes of this exe on the slide bundle (utpp_motion_woman_knee_slide). Every phase needs upperScale > 0.99, the authored
// recoil owning the arms (nueRecoilOwns=1), the hipfire pose, chooser mode 2 and a growing shootEvaluations / slideFrames.
// The -man variants (Male V, W5 2026-11-06) run on the man slide bundle utpp_motion_man_knee_slide.
// Usage: utpp_motion_driver_slide_recoil_test <data dir> [ignored]
// UTPP_MOTION_DRIVER_FIX_MUTATION=21 (slide outside the authored recoil domain, the pre-fix gate) must print a FAIL line.
#define main UnusedIntegrationMain
#include "MotionDriverIntegrationTest.cpp"
#undef main

int main(int argc, char** argv)
{
    if (argc >= 2 && std::string(argv[1]) == "--weapon-upper-child")
        return UnusedIntegrationMain(argc, argv);
    if (argc < 2)
    {
        std::puts("usage: utpp_motion_driver_slide_recoil_test <data dir>");
        return 2;
    }
    const std::string data = argv[1];
    const std::string v1 = data + "\\utpp_base.utppmm";
    const bool baseExists = Exists(v1);
    std::vector<ChildRef> kids; // both variants run in parallel through the child pool, evaluated in order
    for (const char* variant : {"v2-nue-recoil-hipfire-2h-slide", "v2-nue-recoil-hipfire-1h-slide", "v2-nue-recoil-hipfire-2h-slide-man",
                                 "v2-nue-recoil-hipfire-1h-slide-man"})
        kids.push_back(SubmitModeChild("sliderecoil", L"--weapon-upper-child", data, v1, L" 60 " + Wide(variant), baseExists));
    size_t kidIndex = 0;
    for (const char* variant : {"v2-nue-recoil-hipfire-2h-slide", "v2-nue-recoil-hipfire-1h-slide", "v2-nue-recoil-hipfire-2h-slide-man",
                                 "v2-nue-recoil-hipfire-1h-slide-man"})
    {
        const std::string& out = kids[kidIndex].out;
        const DWORD code = WaitChild(kids[kidIndex++]);
        std::string head, phases, line;
        {
            std::ifstream in(out);
            std::getline(in, head);
            while (std::getline(in, line))
                if (line.rfind("phase=", 0) == 0 || line.rfind("groundShot=", 0) == 0)
                    phases += line.substr(0, 170) + " | ";
        }
        DeleteFileA(out.c_str());
        const size_t cut = head.find(" status=");
        if (cut != std::string::npos)
            head.resize(cut);
        Check(code == 0, std::string(variant) + ": slide shots keep the authored recoil and upperScale 1 (exit code " + std::to_string(code) + ")",
              head + " | " + phases);
    }
    std::printf("slide recoil: %d PASS %d FAIL\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
