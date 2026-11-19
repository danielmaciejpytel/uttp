// Turn foot phase (real host + motion driver, MatcherConfig::turnFootPhase): V runs or sprints straight ahead and flicks the stick by
// 180 deg through the dead zone, at several points of the gait cycle (flick time offsets over one cycle), so the flick lands on each foot
// phase. The flick follows recorded game flicks (see kFlickGapS): a stop clip is picked in the dead zone and the turn jump replaces it.
// Feed rates 60, 30 and 200 Hz, both bodies. The same flick is run with the re-rank on and off (motiondriver::SetTurnFootPhase). Checks:
//  - fixture: every flick picks a moving turn / pivot clip with an authored root turn of at least 90 deg;
//  - the reversal search receives the shown pose with the re-rank on, never with it off (TestTurnFootPhaseSearches);
//  - the turn direction is kept: the authored root turn of the picked clip has the sign of the clip picked with the re-rank off, and the
//    body turns with the clip and reaches the reversed heading within 1 s;
//  - foot phase (per body, rate and gait): the turn clip enters on the shown foot phase. Measure: largest world path of one planted toe
//    (contact >= 0.5, drawn root: capsule + root offset) during 1 s after the flick, summed over the phases. Running, the re-rank must make
//    it smaller than the trajectory-only pick; sprinting, not larger (the sprint stop often leaves the trajectory winner in place: a later
//    entry in its clip or a turn outside the steering look-ahead is refused). Single phases may differ either way (run: up to +12 cm seen).
// Mutant UTPP_TURNFOOT_MUTATION=1 (no re-rank) MUST print "FAIL ... turn clip enters on the shown foot phase".
// Usage: utpp_mm_loco_turn_foot_phase_test <driver-data> <basedb>
#define main UnusedIntegrationMain
#include "MotionDriverIntegrationTest.cpp"
#undef main
#include <limits>

#include "../src/mm/Steering.hpp"
namespace
{
constexpr float kDegT = mm::integ::kDegToRad;
float WrapT(float a) { return std::remainder(a, 360.0f); }

// Recorded game flicks (run and sprint, Female V): the stick sits in the dead zone for about 0.05 s while the capsule slows (about
// 25 m/s^2 running, 50 m/s^2 sprinting); the stick then points behind with the sprint state gone (the turn is a run turn) while the
// capsule still moves ahead for about 0.03 s, then its velocity reverses at once.
constexpr float kFlickGapS = 0.05f, kFlipLagS = 0.03f, kRunSpeed = 3.85f, kSprintSpeed = 8.2f;

struct PhaseFrame
{
    bool have = false, root = false;
    uint8_t clipClass = 0;
    float off[3] = {0, 0, 0}; // root offset of the drawn root (x, y in m, capsule axes; yaw in rad)
};
PhaseFrame g_pf;
bool PhaseFrameHook(const int32_t* aBaseGameIndex, uint32_t aBaseBones, motionhost::MotionSetFrame& aOut)
{
    const bool ok = WFrame(aBaseGameIndex, aBaseBones, aOut);
    g_pf.have = ok;
    g_pf.clipClass = ok ? aOut.clipClass : 0;
    g_pf.root = ok && aOut.providedRoot;
    g_pf.off[0] = g_pf.root ? aOut.rootOffsetX : 0.0f;
    g_pf.off[1] = g_pf.root ? aOut.rootOffsetY : 0.0f;
    g_pf.off[2] = g_pf.root ? aOut.rootOffsetYaw : 0.0f;
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

bool ParsePick(int32_t& aClip, float& aTime)
{
    const std::string pk = StatusField(motiondriver::Status(), "pick=clip:");
    const size_t at = pk.find('@'), sl = pk.find("/db");
    if (at == std::string::npos || sl == std::string::npos || sl <= at)
        return false;
    aClip = std::atoi(pk.substr(0, at).c_str());
    aTime = static_cast<float>(std::atof(pk.substr(at + 1, sl - at - 1).c_str()));
    return true;
}

struct FlickRun
{
    bool ok = false;
    std::string prevName, pickName; // clip shown before the turn jump, picked turn clip
    float prevTime = 0, pickTime = 0;
    float clipTurnDeg = 0; // authored root turn of the picked turn clip (deg, + = left)
    float bodyTurnDeg = 0; // body yaw change from the flick until the error is below 20 deg (deg, + = left)
    float yawAt1s = 0;     // body yaw 1 s after the flick
    float lockPathCm = 0;  // largest world path of one planted toe during 1 s after the flick, drawn root
    uint64_t searches = 0; // reversal searches that received the shown pose during the run
    std::string seq;       // clips shown from 0.2 s before the flick to 1 s after it (fixture diagnostics)
};

FlickRun RunFlick(const Rig& aRig, const mm::motion::MotionDatabase& aDb, const int32_t aToe[2], float aHz, bool aSprint, float aFlickS, bool aOn)
{
    FlickRun out;
    motiondriver::SetTurnFootPhase(aOn);
    if (!ResetBase(aRig))
        return out;
    motionhost::SetOffsetRoot(true);
    motionhost::SetHoldPose(true);
    motiondriver::SetEnabled(true);
    const uint64_t searches0 = motiondriver::TestTurnFootPhaseSearches();
    const float dt = 1.0f / aHz;
    const float speed = aSprint ? kSprintSpeed : kRunSpeed, decel = aSprint ? 50.0f : 25.0f;
    const int stand = int(aHz * 0.5f), flick = int(aHz * aFlickS), total = flick + int(aHz * 1.5f);
    motionhost::FeedInput f;
    f.tppActive = true;
    f.haveAir = true;
    f.onGround = true;
    f.distToGround = 0;
    f.psmBlock = mm::integ::kPsmWeaponKnown;
    float yaw = 0, vx = 0, vy = 0, px = 0, py = 0;
    bool turnDone = false, picked = false;
    int32_t lastClip = -1;
    float lastTime = 0;
    float lockPath[2] = {0, 0}, prevToe[2][2] = {{0, 0}, {0, 0}};
    bool prevC[2] = {false, false};
    for (int i = 0; i < total; ++i)
    {
        const bool after = i >= flick;
        const float ta = float(i - flick) * dt, slowEnd = kFlickGapS + kFlipLagS; // time since the flick; end of the slow-down
        const bool moving = i >= stand && !(after && ta < kFlickGapS - 1e-4f); // the stick passes the dead zone (released) first
        const bool slowing = after && ta < slowEnd - 1e-4f;
        const float target = after ? 180.0f : 0.0f;
        if (slowing) // the capsule slows ahead at the recorded rate, the same speeds at every feed rate
        {
            const float v = std::fmax(0.0f, speed - decel * std::fmin(ta + dt, slowEnd));
            vx = 0;
            vy = v;
        }
        else if (after && ta - dt < slowEnd - 1e-4f) // first feed after the slow-down: the velocity reverses at once
        {
            vx = 0;
            vy = -std::fmax(0.0f, speed - decel * slowEnd);
        }
        const float sp = after ? kRunSpeed : speed; // run speed after the flick (sprint released)
        const float dvx = (moving && !slowing ? -std::sin(target * kDegT) * sp : 0.0f) - vx,
                    dvy = (moving && !slowing ? std::cos(target * kDegT) * sp : 0.0f) - vy;
        const float dv = std::hypot(dvx, dvy), lim = slowing ? 0.0f : 8.0f * dt;
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
        f.locoState = aSprint && moving && !after ? mm::integ::kLocoSprint : mm::integ::kLocoStand;
        f.yawAheadDeg = std::numeric_limits<float>::quiet_NaN();
        float cmdStep = 0;
        if (motionhost::BodyTurnActive())
            cmdStep = WrapT(motionhost::BodyYawCommand(yaw, yaw) - yaw);
        f.playerYawDeg = yaw;
        motiondriver::FeedAirFromInput(f);
        g_log = {};
        g_pf = {};
        motionhost::Feed(f);
        px += vx * dt;
        py += vy * dt;
        int32_t clip = -1;
        float clipTime = 0;
        const bool havePick = ParsePick(clip, clipTime) && clip >= 0 && static_cast<uint32_t>(clip) < aDb.NumClips();
        const bool window = after && i < flick + int(aHz * 1.0f);
        if (window && !picked && havePick && g_pf.have && clip != lastClip && lastClip >= 0 &&
            (mm::steer::IsPivot(g_pf.clipClass) || mm::steer::IsMoveTurn(g_pf.clipClass)) && std::fabs(ClipTurnDeg(aDb, clip)) >= 90.0f)
        {
            picked = true;
            out.prevName = aDb.ClipName(static_cast<uint32_t>(lastClip));
            out.pickName = aDb.ClipName(static_cast<uint32_t>(clip));
            out.prevTime = lastTime;
            out.pickTime = clipTime;
            out.clipTurnDeg = ClipTurnDeg(aDb, clip);
        }
        if (havePick && clip != lastClip && i >= flick - int(aHz * 0.2f) && i < flick + int(aHz * 1.0f))
        {
            char at[32];
            std::snprintf(at, sizeof at, "%+.2f ", double(float(i - flick) * dt));
            out.seq += std::string(out.seq.empty() ? "" : ", ") + at + std::string(aDb.ClipName(static_cast<uint32_t>(clip)));
        }
        // planted toe world path: drawn root = capsule (integrated feed velocity, fed actor yaw) + root offset of the frame
        if (window && ReadPublished())
        {
            const float c0 = std::cos(yaw * kDegT), s0 = std::sin(yaw * kDegT);
            const float rx = px + c0 * g_pf.off[0] - s0 * g_pf.off[1], ry = py + s0 * g_pf.off[0] + c0 * g_pf.off[1];
            const float ry2 = yaw * kDegT + g_pf.off[2], c = std::cos(ry2), sn = std::sin(ry2);
            for (int side = 0; side < 2; ++side)
            {
                float m[3];
                const bool con = havePick && ContactAt(aDb, clip, clipTime, side) >= 0.5f && aToe[side] >= 0 && ModelPos(g_read, aRig, aToe[side], m);
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
        if (havePick)
        {
            lastClip = clip;
            lastTime = clipTime;
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
    out.searches = motiondriver::TestTurnFootPhaseSearches() - searches0;
    out.ok = picked;
    motiondriver::SetTurnFootPhase(false); // the mod default
    return out;
}

std::string Describe(const FlickRun& aR)
{
    char d[512];
    std::snprintf(d, sizeof d, "%s@%.2f -> %s@%.2f, clip turn %.1f, body turn %.1f, yaw 1 s %.1f, planted toe lock path max %.1f cm, searches %llu",
                  aR.prevName.c_str(), double(aR.prevTime), aR.pickName.c_str(), double(aR.pickTime), double(aR.clipTurnDeg), double(aR.bodyTurnDeg),
                  double(aR.yawAt1s), double(aR.lockPathCm), static_cast<unsigned long long>(aR.searches));
    return aR.ok ? std::string(d) : std::string(d) + ", shown clips: " + aR.seq;
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::puts("usage TurnFootPhaseTest <driver-data> <basedb>");
        return 2;
    }
    g_baseDb = std::wstring(argv[2], argv[2] + std::strlen(argv[2]));
    motiondriver::SetOverlayInputs(0, 0, false, false);
    motionhost::MotionHooks hooks{&WStep, &WApply, &motiondriver::OnArmed, &motiondriver::OnDisarmed, &PhaseFrameHook, &motiondriver::OwnsTraversal};
    hooks.published = &motiondriver::RecordPublishedPose; // the shown pose ring, as the mod's natives install it
    motionhost::SetMotionHooks(&hooks);
    const struct
    {
        const char* body;
        int32_t index;
    } bodies[] = {{"woman", 0}, {"man", 1}};
    constexpr int kPhases = 6;
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
            {
                const std::string tag = std::string(bd.body) + " " + std::to_string(int(hz)) + " Hz" + (sprint ? " sprint" : " run");
                const float cycle = sprint ? 0.62f : 0.72f; // one gait cycle of the loop (s); the offsets cover it
                float pathOn = 0, pathOff = 0;
                for (int k = 0; k < kPhases; ++k)
                {
                    const float flickS = 3.5f + cycle * float(k) / float(kPhases);
                    const FlickRun on = RunFlick(rig, *db, toe, hz, sprint, flickS, true);
                    const FlickRun off = RunFlick(rig, *db, toe, hz, sprint, flickS, false);
                    std::printf("INFO %s phase %d on : %s\n", tag.c_str(), k, Describe(on).c_str());
                    std::printf("INFO %s phase %d off: %s\n", tag.c_str(), k, Describe(off).c_str());
                    const std::string pt = tag + " phase " + std::to_string(k);
                    Check(on.ok && off.ok, pt + ": fixture - a moving turn / pivot clip with an authored root turn plays after the flick",
                          Describe(on) + " | " + Describe(off));
                    if (!on.ok || !off.ok)
                        continue;
                    Check(on.searches > 0 && off.searches == 0, pt + ": the reversal search receives the shown pose (re-rank on only)", Describe(on));
                    Check(on.clipTurnDeg * off.clipTurnDeg > 0.0f, pt + ": turn direction kept (same sign as the trajectory pick)",
                          Describe(on) + " | " + Describe(off));
                    Check(on.clipTurnDeg * on.bodyTurnDeg > 0.0f && std::fabs(WrapT(on.yawAt1s - 180.0f)) <= 20.0f,
                          pt + ": body turns with the clip and reaches the reversed heading within 1 s", Describe(on));
                    pathOn += on.lockPathCm;
                    pathOff += off.lockPathCm;
                }
                char d[256];
                std::snprintf(d, sizeof d, "planted toe lock path over %d phases: re-rank %.1f cm, trajectory only %.1f cm", kPhases, double(pathOn),
                              double(pathOff));
                std::printf("INFO %s %s\n", tag.c_str(), d);
                Check(sprint ? pathOn <= pathOff : pathOn < pathOff,
                      tag + ": turn clip enters on the shown foot phase (planted toe path vs the trajectory-only pick)", d);
            }
    }
    motiondriver::SetEnabled(false);
    motiondriver::Shutdown();
    std::printf("%s: %d passed, %d failed\n", g_fail ? "FAILED" : "PASSED", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
