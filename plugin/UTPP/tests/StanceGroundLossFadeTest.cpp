// StanceGuard on a ground loss reported only by the physics probe (one feed with onGround = false while walking, as on a slope or a
// step). The real host + motion driver walk straight ahead; four runs: guard on / off, each with and without the one-feed flicker.
//  - guard correction at the flicker feed: published legs, guard on vs guard off (no flicker) - the fixture needs a real correction;
//  - other effects of the flicker: published legs, flicker vs no flicker, guard off (foot locks, driver air state);
//  - guard part of the flicker: published legs, flicker vs no flicker, guard on, minus the other effects.
// The guard part must stay within the fade's share of the correction on the flicker feed (dt / fade time) and must not leave a tail
// on the following feeds (the guard's filters keep running through the flicker).
#define main UnusedIntegrationMain
#include "MotionDriverIntegrationTest.cpp"
#undef main
namespace
{
constexpr int kFeeds = 260, kStart = 30, kFlick = 180, kAfter = 4; // the old cut regrows over ~4 feeds (80 ms swivel filter)
constexpr float kDt = 1.0f / 60.0f, kFadeSec = 0.1f, kWalk = 1.5f, kMinHalfWidthCm = 3.0f; // default StanceGuardConfig
const char* const kLegBones[] = {"LeftUpLeg", "LeftLeg", "LeftFoot", "RightUpLeg", "RightLeg", "RightFoot"};

struct RunOut
{
    bool ok = false;
    int owned = 0;
    std::vector<mm::integ::PoseSnapshot> pose; // published pose per feed (kFlick - 2 .. kFlick + kAfter)
    std::vector<bool> published;
};

RunOut Run(const Rig& aRig, bool aGuard, bool aFlicker)
{
    RunOut out;
    if (!ResetBase(aRig))
        return out;
    motionhost::SetOffsetRoot(true);
    motionhost::SetHoldPose(true);
    motionhost::SetStanceGuard(aGuard, kMinHalfWidthCm);
    motiondriver::SetEnabled(true);
    motionhost::FeedInput f;
    f.tppActive = true;
    f.locoState = mm::integ::kLocoStand;
    f.haveAir = true;
    f.psmBlock = mm::integ::kPsmWeaponKnown;
    float sp = 0;
    for (int i = 0; i < kFeeds; ++i)
    {
        const bool moving = i >= kStart;
        if (moving)
            sp = std::fmin(kWalk, sp + 3.0f * kDt);
        f.dt = kDt;
        f.velWorld[0] = 0;
        f.velWorld[1] = sp;
        const bool flick = aFlicker && i == kFlick;
        f.velWorld[2] = flick ? -1.0f : 0.0f; // as logged in game on such feeds: falling slightly, no ground probe
        f.distToGround = flick ? -2.0f : 0.0f;
        f.move[0] = 0;
        f.move[1] = moving ? 1.0f : 0.0f;
        f.cameraYawDeg = 0;
        f.playerYawDeg = 0;
        f.haveTarget = moving;
        f.targetYawDeg = 0;
        f.onGround = !flick;
        motiondriver::FeedAirFromInput(f);
        g_log = {};
        motionhost::Feed(f);
        out.owned += g_log.stepResult ? 1 : 0;
        if (i >= kFlick - 2 && i <= kFlick + kAfter)
        {
            out.published.push_back(ReadPublished());
            out.pose.push_back(g_read);
        }
    }
    motionhost::SetStanceGuard(true, kMinHalfWidthCm);
    out.ok = true;
    return out;
}

std::vector<uint32_t> g_legIdx;
float LegAngleDeg(const mm::integ::PoseSnapshot& a, const mm::integ::PoseSnapshot& b)
{
    float m = 0;
    for (uint32_t i : g_legIdx)
    {
        if (i >= a.numBones || i >= b.numBones)
            continue;
        const float* p = &a.pose[size_t(i) * 7 + 3];
        const float* q = &b.pose[size_t(i) * 7 + 3];
        const float d = std::fabs(p[0] * q[0] + p[1] * q[1] + p[2] * q[2] + p[3] * q[3]);
        m = std::max(m, 2.0f * std::acos(std::min(1.0f, d)) * 57.2957795f);
    }
    return m;
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::puts("usage StanceGroundLossFadeTest <driver-data> <basedb>");
        return 2;
    }
    Rig rig;
    std::unique_ptr<mm::motion::MotionDatabase> db;
    if (!RigFromDb(DataPaths(argv[1], "woman").files.db, rig, &db))
        return 3;
    for (const char* name : kLegBones)
        for (uint32_t i = 0; i < db->NumBones(); ++i)
            if (std::string(db->BoneName(i)) == name)
                g_legIdx.push_back(i);
    Check(g_legIdx.size() == 6, "fixture: six leg bones found");
    g_baseDb = std::wstring(argv[2], argv[2] + std::strlen(argv[2]));
    motiondriver::SetPathsOverride(0, DataPaths(argv[1], "woman"));
    motiondriver::SetRigBody(0);
    motiondriver::SetOverlayInputs(0, 0, false, false);
    const motionhost::MotionHooks hooks{&WStep, &WApply, &motiondriver::OnArmed, &motiondriver::OnDisarmed, &WFrame, &motiondriver::OwnsTraversal};
    motionhost::SetMotionHooks(&hooks);
    Check(ResetBase(rig), "real host initialized");
    motiondriver::SetEnabled(true);
    motiondriver::Step({}, 0, kDt);
    motiondriver::WaitLoader();

    const RunOut onCtl = Run(rig, true, false), onFl = Run(rig, true, true);
    const RunOut offCtl = Run(rig, false, false), offFl = Run(rig, false, true);
    const bool runs = onCtl.ok && onFl.ok && offCtl.ok && offFl.ok && onCtl.pose.size() == size_t(kAfter + 3);
    bool allPublished = runs;
    for (const RunOut* r : {&onCtl, &onFl, &offCtl, &offFl})
        for (bool p : r->published)
            allPublished = allPublished && p;
    const size_t at = 2; // index of the flicker feed in the recorded window
    const float corr = runs ? LegAngleDeg(onCtl.pose[at], offCtl.pose[at]) : 0.0f;
    char b[256];
    std::snprintf(b, sizeof b, "owned %d/%d/%d/%d, guard correction at the flicker feed %.2f deg", onCtl.owned, onFl.owned, offCtl.owned,
                  offFl.owned, static_cast<double>(corr));
    std::printf("%s\n", b);
    Check(runs && allPublished && onCtl.owned >= kFeeds - kStart - 10 && onFl.owned >= kFeeds - kStart - 10,
          "fixture: motion owns the walk and publishes every recorded feed", b);
    Check(corr >= 5.0f, "fixture: the guard corrects the legs by at least 5 deg at the flicker feed", b);

    // guard part of the flicker per feed (flicker feed and the feeds after it)
    float worstFlick = 0, worstTail = 0;
    for (size_t k = at; runs && k < onCtl.pose.size(); ++k)
    {
        const float on = LegAngleDeg(onFl.pose[k], onCtl.pose[k]);
        const float other = LegAngleDeg(offFl.pose[k], offCtl.pose[k]);
        const float guardPart = std::fmax(0.0f, on - other);
        std::printf("  feed %+d: flicker vs control guard on %.3f deg, guard off %.3f deg, guard part %.3f deg\n", int(k - at),
                    static_cast<double>(on), static_cast<double>(other), static_cast<double>(guardPart));
        if (k == at)
            worstFlick = guardPart;
        else
            worstTail = std::fmax(worstTail, guardPart);
    }
    const float allowFlick = corr * (kDt / kFadeSec) + 1.0f;
    const float allowTail = std::fmax(1.0f, 0.1f * corr);
    std::snprintf(b, sizeof b, "flicker feed %.2f deg (allowed %.2f), later feeds %.2f deg (allowed %.2f)", static_cast<double>(worstFlick),
                  static_cast<double>(allowFlick), static_cast<double>(worstTail), static_cast<double>(allowTail));
    Check(runs && worstFlick <= allowFlick, "ground loss: the guard fades out instead of dropping its correction in one feed", b);
    Check(runs && worstTail <= allowTail, "ground loss: no regrowth tail after the flicker", b);

    motionhost::SetMotionHooks(nullptr);
    motionhost::OnDisarmed();
    std::printf("StanceGroundLossFade: %d PASS %d FAIL\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
