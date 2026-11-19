// Near-reversal tie (real host + motion driver): V runs or sprints straight ahead and the stick flips by 180 deg while the body yaw sits a
// fraction of a degree off the old heading (+0.3 / -0.3 deg). The shorter way of the body turn then depends on that residual alone, while the
// motion search picks the 180 deg turn clip by its pose cost. The body yaw commands are issued in the game order (command for the current yaw,
// applied after the feed). Feed rates 60, 30 and 200 Hz. Checks per body, gait, residual sign and rate:
//  - fixture: a moving turn / pivot clip with an authored root turn of at least 90 deg plays after the flick;
//  - the body turns the same way as that clip (sign of the body yaw change over the turn and over its first 0.1 s = sign of the clip's
//    authored root turn, summed from the database);
//  - the body reaches the reversed heading within 1 s;
//  - 60 Hz: the planted toe path over the turn stays bounded: largest world path of one planted toe (drawn root: capsule + root offset)
//    during 1 s after the flick <= 200 cm running / 400 cm sprinting. This fixture reverses the capsule at 60 m/s^2, so even a turn that agrees
//    with its clip leaves a long path (64-340 cm here); a body turning against the clip measured 232-267 cm running and 500-529 cm sprinting.
// Mutant UTPP_REVERSAL_TIE_MUTATION=1 (the body turn takes the shorter way at the tie) MUST print "FAIL ... body turns with the clip".
// Usage: utpp_bodyturn_reversal_tie_host_test <driver-data> <basedb>
#define main UnusedIntegrationMain
#include "MotionDriverIntegrationTest.cpp"
#undef main
#include <limits>

#include "../src/mm/Steering.hpp"
namespace
{
constexpr float kDegT = mm::integ::kDegToRad;
float WrapT(float a) { return std::remainder(a, 360.0f); }

struct TieFrame
{
    bool have = false, root = false;
    uint8_t clipClass = 0;
    float off[3] = {0, 0, 0}; // root offset of the drawn root (x, y in m, capsule axes; yaw in rad)
};
TieFrame g_tf;
bool TieFrameHook(const int32_t* aBaseGameIndex, uint32_t aBaseBones, motionhost::MotionSetFrame& aOut)
{
    const bool ok = WFrame(aBaseGameIndex, aBaseBones, aOut);
    g_tf.have = ok;
    g_tf.clipClass = ok ? aOut.clipClass : 0;
    g_tf.root = ok && aOut.providedRoot;
    g_tf.off[0] = g_tf.root ? aOut.rootOffsetX : 0.0f;
    g_tf.off[1] = g_tf.root ? aOut.rootOffsetY : 0.0f;
    g_tf.off[2] = g_tf.root ? aOut.rootOffsetYaw : 0.0f;
    return ok;
}

// authored ROOT turn of a clip over all its frames (deg, + = left)
float ClipTurnDeg(const mm::motion::MotionDatabase& aDb, int32_t aClip)
{
    if (aClip < 0 || static_cast<uint32_t>(aClip) >= aDb.NumClips())
        return 0.0f;
    const auto& cl = aDb.Clip(static_cast<uint32_t>(aClip));
    double t = 0;
    for (uint32_t f = 0; f < cl.numFrames; ++f)
        t += aDb.Root(cl.firstFrame + f).dyaw;
    return static_cast<float>(t) / kDegT;
}

struct TieRun
{
    bool ok = false;
    std::string clips;      // moving turn / pivot clips shown during the 1 s after the flick
    float clipTurnDeg = 0;  // authored root turn of the first of them (deg, + = left)
    float bodyTurnDeg = 0;  // unwrapped body yaw change from the flick until the error is below 20 deg (deg, + = left)
    float firstStepDeg = 0; // sum of the body yaw steps of the first 0.1 s after the flick (deg)
    float lockPathCm = 0;   // largest world path of one planted toe (contact >= 0.5) during the 1 s after the flick, drawn root
    float yawAt1s = 0;      // body yaw 1 s after the flick
    int pickFeed = -1;      // feeds from the flick to the first moving turn / pivot frame
};

TieRun RunTie(const Rig& aRig, const mm::motion::MotionDatabase& aDb, const int32_t aToe[2], float aHz, bool aSprint, float aResidualDeg)
{
    TieRun out;
    if (!ResetBase(aRig))
        return out;
    motionhost::SetOffsetRoot(true);
    motionhost::SetHoldPose(true);
    motiondriver::SetEnabled(true);
    const float dt = 1.0f / aHz;
    const float speed = aSprint ? 8.2f : 3.85f;
    const int stand = int(aHz * 0.5f), flick = int(aHz * 4.0f), total = int(aHz * 5.5f);
    motionhost::FeedInput f;
    f.tppActive = true;
    f.haveAir = true;
    f.onGround = true;
    f.distToGround = 0;
    f.psmBlock = mm::integ::kPsmWeaponKnown;
    float yaw = aResidualDeg, vx = 0, vy = 0, px = 0, py = 0;
    bool turnDone = false, haveClipTurn = false;
    float lockPath[2] = {0, 0}, prevToe[2][2] = {{0, 0}, {0, 0}};
    bool prevC[2] = {false, false};
    int32_t lastClip = -1;
    for (int i = 0; i < total; ++i)
    {
        const bool moving = i >= stand;
        const bool after = i >= flick;
        const float target = after ? 180.0f : aResidualDeg; // camera yaw 0: the stick flips from ahead (body at the residual) to behind
        const float accel = after ? 60.0f : 8.0f;
        const float dvx = (moving ? -std::sin(target * kDegT) * speed : 0.0f) - vx, dvy = (moving ? std::cos(target * kDegT) * speed : 0.0f) - vy;
        const float dv = std::hypot(dvx, dvy), lim = accel * dt;
        if (dv > lim)
        {
            vx += dvx * lim / dv;
            vy += dvy * lim / dv;
        }
        else
        {
            vx += dvx;
            vy += dvy;
        }
        f.dt = dt;
        f.velWorld[0] = vx;
        f.velWorld[1] = vy;
        f.move[0] = 0;
        f.move[1] = moving ? 1.0f : 0.0f;
        f.cameraYawDeg = 0;
        f.haveTarget = moving;
        f.targetYawDeg = target;
        f.locoState = aSprint && moving ? mm::integ::kLocoSprint : mm::integ::kLocoStand;
        f.yawAheadDeg = std::numeric_limits<float>::quiet_NaN();
        float cmdStep = 0;
        if (motionhost::BodyTurnActive())
            cmdStep = WrapT(motionhost::BodyYawCommand(yaw, yaw) - yaw);
        f.playerYawDeg = yaw;
        motiondriver::FeedAirFromInput(f);
        g_log = {};
        g_tf = {};
        motionhost::Feed(f);
        px += vx * dt;
        py += vy * dt;
        int32_t clip = -1;
        float clipTime = 0;
        {
            const std::string pk = StatusField(motiondriver::Status(), "pick=clip:");
            const size_t at = pk.find('@'), sl = pk.find("/db");
            if (at != std::string::npos && sl != std::string::npos && sl > at)
            {
                clip = std::atoi(pk.substr(0, at).c_str());
                clipTime = static_cast<float>(std::atof(pk.substr(at + 1, sl - at - 1).c_str()));
            }
        }
        if (after && i < flick + int(aHz * 1.0f))
        {
            if (i < flick + int(aHz * 0.1f))
                out.firstStepDeg += cmdStep;
            if (g_tf.have && (mm::steer::IsPivot(g_tf.clipClass) || mm::steer::IsMoveTurn(g_tf.clipClass)) && clip != lastClip && clip >= 0 &&
                static_cast<uint32_t>(clip) < aDb.NumClips())
            {
                if (out.pickFeed < 0)
                    out.pickFeed = i - flick;
                out.clips += (out.clips.empty() ? "" : ",") + std::string(aDb.ClipName(static_cast<uint32_t>(clip)));
                lastClip = clip;
                const float turn = ClipTurnDeg(aDb, clip);
                if (!haveClipTurn && std::fabs(turn) >= 90.0f)
                {
                    out.clipTurnDeg = turn;
                    haveClipTurn = true;
                }
            }
            // planted toe world path: drawn root = capsule (integrated feed velocity, fed actor yaw) + root offset of the frame
            if (ReadPublished())
            {
                const float c0 = std::cos(yaw * kDegT), s0 = std::sin(yaw * kDegT);
                const float rx = px + c0 * g_tf.off[0] - s0 * g_tf.off[1], ry = py + s0 * g_tf.off[0] + c0 * g_tf.off[1];
                const float ry2 = yaw * kDegT + g_tf.off[2], c = std::cos(ry2), sn = std::sin(ry2);
                for (int side = 0; side < 2; ++side)
                {
                    float m[3];
                    const bool con = clip >= 0 && ContactAt(aDb, clip, clipTime, side) >= 0.5f && aToe[side] >= 0 &&
                                     ModelPos(g_read, aRig, aToe[side], m);
                    if (con)
                    {
                        const float w[2] = {rx + c * m[0] - sn * m[1], ry + sn * m[0] + c * m[1]};
                        if (prevC[side])
                            lockPath[side] += std::hypot(w[0] - prevToe[side][0], w[1] - prevToe[side][1]) * 100.0f;
                        prevToe[side][0] = w[0];
                        prevToe[side][1] = w[1];
                    }
                    else if (prevC[side])
                    {
                        out.lockPathCm = std::fmax(out.lockPathCm, lockPath[side]);
                        lockPath[side] = 0;
                    }
                    prevC[side] = con;
                }
            }
        }
        if (after && !turnDone)
        {
            out.bodyTurnDeg += cmdStep;
            turnDone = std::fabs(WrapT(180.0f - (yaw + cmdStep))) < 20.0f;
        }
        yaw = WrapT(yaw + cmdStep); // game order: the command is applied after the feed
        if (i == flick + int(aHz * 1.0f))
            out.yawAt1s = yaw;
    }
    out.lockPathCm = std::fmax(out.lockPathCm, std::fmax(lockPath[0], lockPath[1]));
    out.ok = true;
    return out;
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::puts("usage ReversalTieHostTest <driver-data> <basedb>");
        return 2;
    }
    g_baseDb = std::wstring(argv[2], argv[2] + std::strlen(argv[2]));
    motiondriver::SetOverlayInputs(0, 0, false, false);
    const motionhost::MotionHooks hooks{&WStep, &WApply, &motiondriver::OnArmed, &motiondriver::OnDisarmed, &TieFrameHook, &motiondriver::OwnsTraversal};
    motionhost::SetMotionHooks(&hooks);
    const struct
    {
        const char* body;
        int32_t index;
    } bodies[] = {{"woman", 0}, {"man", 1}};
    for (const auto& bd : bodies)
    {
        Rig rig;
        std::unique_ptr<mm::motion::MotionDatabase> db;
        Check(RigFromDb(DataPaths(argv[1], bd.body).files.db, rig, &db), std::string(bd.body) + ": rig from the motion database");
        if (!db)
            continue;
        int32_t toe[2] = {-1, -1};
        for (uint32_t b = 0; b < db->NumBones(); ++b)
        {
            if (db->BoneName(b) == "LeftToeBase")
                toe[0] = static_cast<int32_t>(b);
            if (db->BoneName(b) == "RightToeBase")
                toe[1] = static_cast<int32_t>(b);
        }
        motiondriver::SetEnabled(false);
        motiondriver::SetPathsOverride(bd.index, DataPaths(argv[1], bd.body));
        motiondriver::SetRigBody(bd.index);
        Check(ResetBase(rig), std::string(bd.body) + ": real host initialized");
        motiondriver::SetEnabled(true);
        motiondriver::Step({}, 0, 1.f / 60);
        motiondriver::WaitLoader();
        for (float hz : {60.0f, 30.0f, 200.0f})
        for (bool sprint : {false, true})
            for (float res : {0.3f, -0.3f})
            {
                const TieRun r = RunTie(rig, *db, toe, hz, sprint, res);
                char d[512];
                std::snprintf(d, sizeof d,
                              "%.0f Hz %s residual %+.1f deg: turn clips [%s] authored root turn %.1f deg, body turn %.1f deg (first 0.1 s %.1f deg), yaw 1 s "
                              "after the flick %.1f deg, planted toe lock path max %.1f cm, turn clip from feed %d",
                              double(hz), sprint ? "sprint" : "run", double(res), r.clips.c_str(), double(r.clipTurnDeg), double(r.bodyTurnDeg),
                              double(r.firstStepDeg), double(r.yawAt1s), double(r.lockPathCm), r.pickFeed);
                std::printf("INFO %s %s\n", bd.body, d);
                const std::string tag = std::string(bd.body) + " " + std::to_string(int(hz)) + " Hz" + (sprint ? " sprint" : " run") +
                                        (res > 0 ? " +0.3" : " -0.3");
                Check(r.ok && !r.clips.empty() && std::fabs(r.clipTurnDeg) >= 90.0f,
                      tag + ": fixture - a moving turn / pivot clip with an authored root turn plays after the flick", d);
                Check(r.clipTurnDeg * r.bodyTurnDeg > 0.0f && r.clipTurnDeg * r.firstStepDeg > 0.0f,
                      tag + ": body turns with the clip (same direction as the authored root turn)", d);
                Check(std::fabs(WrapT(r.yawAt1s - 180.0f)) <= 20.0f, tag + ": the body reaches the reversed heading within 1 s", d);
                if (hz == 60.0f)
                    Check(r.lockPathCm <= (sprint ? 400.0f : 200.0f), tag + ": planted toe lock path over the turn bounded (200 / 400 cm)", d);
            }
    }
    motiondriver::SetEnabled(false);
    motiondriver::Shutdown();
    std::printf("%s: %d passed, %d failed\n", g_fail ? "FAILED" : "PASSED", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
