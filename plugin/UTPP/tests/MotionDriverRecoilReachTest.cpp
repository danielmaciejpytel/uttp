// Fix batch 3, recoil profiles (review round): every profile of RecoilProfiles.hpp through the real driver and the reach solver.
// Hip (shoulder clip): hipfire-<rifle|pistol1h>-pid<N>-profile = ready, a shot, the hold, a burst, a first shot without hold and one
// second of full-auto at 12 rounds/s. ADS (ironsight clip): nue-recoil-ads-<rifle|1h>-pid<N> = the whole recoil block (retrigger, clip
// switch, hitch, reload fade, guards). Both need upperScale 1 and the recoil owner on every shot frame, no clamp, at most 3 cm / 5 degrees
// between two frames, and the IK following at least 70% of the kick on every applied frame (shootScale >= 0.7). That last bound is the
// reason the Nova / Overture / Burya clips (a revolver hammer-cock, 3-30% reach) are retired from the table.
// The children run in parallel through the child pool of the integration test (the recoil blocks run on the simulated recoil clock, so the load
// does not matter).
// Seven representative profiles also run with the -man suffix (Male V, W5 2026-11-06).
// Usage: utpp_motion_driver_recoil_reach_test <data dir> [ignored]
#define main UnusedIntegrationMain
#include "MotionDriverIntegrationTest.cpp"
#undef main
#include "../src/mm/RecoilProfiles.hpp"

int main(int argc, char** argv)
{
    if (argc >= 2 && std::string(argv[1]) == "--weapon-upper-child")
        return UnusedIntegrationMain(argc, argv);
    if (argc < 2)
    {
        std::puts("usage: utpp_motion_driver_recoil_reach_test <data dir>");
        return 2;
    }
    const std::string data = argv[1];
    const std::string v1 = data + "\\utpp_base.utppmm";
    namespace rp = utpp::mm::recoil_profile;
    struct Job { std::string variant; std::string out; size_t id = 0; DWORD code = 1; };
    std::vector<Job> jobs;
    for (int32_t id = 2; id < rp::kSlots; ++id)
    {
        if (!rp::Valid(id)) continue;
        const bool longGun = rp::kTable[static_cast<size_t>(id)].longGun;
        jobs.push_back({"hipfire-" + std::string(longGun ? "rifle" : "pistol1h") + "-pid" + std::to_string(id) + "-profile", {}, 0, 1});
        jobs.push_back({"nue-recoil-ads-" + std::string(longGun ? "rifle" : "1h") + "-pid" + std::to_string(id), {}, 0, 1});
        // Male V (W5 2026-11-06): representative profiles through the man bundle (rig body 1, overlays_man): the largest pistol ADS kick (chao),
        // a standard pistol (liberty), a revolver (quasar), the Nue (the only authored reload), a rifle (ajax), the budget-scaled burst rifle (masamune)
        // and an SMG (pulsar). The profiles are weapon-local and shared; the reach solver runs on the male arms, so the 70% bound is checked on the
        // male skeleton too. (Every family would double the run time: a child takes ~20 s with the 15.6 ms timer; the curve parity of all 16
        // families is covered by utpp_authored_recoil_male_parity_test.)
        const std::string family = rp::kTable[static_cast<size_t>(id)].family;
        if (family == "nue" || family == "liberty" || family == "chao" || family == "quasar" || family == "ajax" || family == "masamune" ||
            family == "pulsar")
        {
            jobs.push_back({"hipfire-" + std::string(longGun ? "rifle" : "pistol1h") + "-pid" + std::to_string(id) + "-profile-man", {}, 0, 1});
            jobs.push_back({"nue-recoil-ads-" + std::string(longGun ? "rifle" : "1h") + "-pid" + std::to_string(id) + "-man", {}, 0, 1});
        }
    }
    // every job runs through the child pool (parallel, bounded); the results are evaluated in the order of the jobs
    const bool baseExists = Exists(v1);
    for (size_t i = 0; i < jobs.size(); ++i)
    {
        const ChildRef kid = SubmitModeChild(("recoilreach" + std::to_string(i)).c_str(), L"--weapon-upper-child", data, v1,
                                             L" 60 v2-" + Wide(jobs[i].variant), baseExists, 600000);
        jobs[i].id = kid.id;
        jobs[i].out = kid.out;
    }
    for (Job& job : jobs)
    {
        job.code = WaitChild(ChildRef{job.id, job.out});
        std::string head;
        {
            std::ifstream in(job.out);
            std::getline(in, head);
        }
        DeleteFileA(job.out.c_str());
        if (head.size() > 600)
            head.resize(600);
        Check(job.code == 0, job.variant + ": the IK follows the kick (scale >= 0.7), no clamp, bounded steps, owned arms (exit code " +
              std::to_string(job.code) + ")", head);
    }
    std::printf("recoil reach: %d PASS %d FAIL\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
