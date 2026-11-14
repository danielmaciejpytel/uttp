// mm.slide through the REAL v1 host (src/MotionHost.cpp Feed + the v2 seam, no RED4ext): with the slide on and a slide bundle the
// v1 body-yaw command stays out of the slide (BodyYawCommand = pass-through of the base yaw on PSM detail 5 feeds, active again
// after the slide), the published weight never pops across slide entry / exit (v2 owns the body through the existing hold), and
// with the setting off the host is the pre-slide one (the command is active, as today). Same harness as CapsuleBrakingHostTest.
// Usage: utpp_motion_driver_slide_host_test <data dir> <v1 database>
// fix2/hip P6 K1 wiring: the weapon pose's leg layer is faded (Status airLegs=) only by the real air (chooser MovementMode 1), never
// by the slide (mode 2); the weight rises within 5 feeds of the take-off (capped at 0.22 per feed), holds 0.15 s after the landing and is gone 0.5 s after it
// (UTPP_MOTION_DRIVER_FIX_MUTATION=22 also fades it in the slide). Review fixes: a skip (frames handed to the game / v1) resets the weight
// (mutation 23 keeps it), and a jump into a climb hands the weight over with hold + fall instead of a reset on the first montage frame
// (mutation 24 resets it). Each mutation build must print a FAIL line.
#define main UnusedIntegrationMain
#include "MotionDriverIntegrationTest.cpp"
#undef main

namespace
{
struct Run
{
    int sprintFrames = 0, sprintLate = 0, slideFrames = 0, exitFrames = 0;
    int slideStepped = 0, slideCmdPassThrough = 0, slideCmdActive = 0, sprintCmdActive = 0, exitCmdActive = 0;
    float minWeight = 9.0f, minWeightSlide = 9.0f;
    uint64_t statusSlideFrames = 0;
    float maxAirLegs = 0.0f; // Status airLegs= over the sprint / slide / exit feeds (none of them is air)
    std::string lastSlideStatus;
};

float AirLegsOfStatus()
{
    const std::string v = StatusField(motiondriver::Status(), "airLegs=");
    return v.empty() ? -1.0f : std::strtof(v.c_str(), nullptr);
}

constexpr float kTarget = 30.0f; // world target yaw of the stick (deg): the body (yaw 0) is 30 deg off -> an active command moves
constexpr float kBaseYaw = 90.0f; // what the base turn would command: a value the controller never returns for this feed

Run Scenario(const Rig& aRig, bool aSlideOn)
{
    Run r;
    motiondriver::SetSlide(aSlideOn);
    motionhost::FeedInput f;
    f.dt = 1.0f / 60.0f;
    f.tppActive = true;
    f.haveAir = true;
    f.onGround = true;
    f.distToGround = 0.0f;
    f.move[1] = 1.0f;
    f.haveTarget = true;
    f.targetYawDeg = kTarget;
    f.cameraYawDeg = 0.0f;
    f.playerYawDeg = 0.0f;
    (void)aRig;
    auto feed = [&](float aSpeed, int aLoco, uint32_t aPsm) {
        f.velWorld[1] = aSpeed;
        f.locoState = aLoco;
        f.psmBlock = aPsm;
        motiondriver::FeedAirFromInput(f);
        g_log = HookLog{};
        motionhost::Feed(f);
        const float cmd = motionhost::BodyYawCommand(0.0f, kBaseYaw);
        mm::integ::PoseSnapshot snap;
        const float w = g_mmPose.Read(snap) ? snap.weight : -1.0f;
        return std::pair<float, float>{cmd, w};
    };
    for (int i = 0; i < 60; ++i)
    {
        const auto [cmd, w] = feed(8.2f, mm::integ::kLocoSprint, 0u);
        ++r.sprintFrames;
        if (i >= 20) // the host needs a few feeds after the toggle before its body-turn path is live
        {
            ++r.sprintLate;
            r.sprintCmdActive += cmd != kBaseYaw ? 1 : 0;
            r.minWeight = std::min(r.minWeight, w);
        }
    }
    float v = 10.8f;
    const uint32_t slidePsm = mm::integ::kPsmDetailed | (6u << mm::integ::kPsmDetailShift);
    for (int i = 0; i < 80; ++i, v = std::max(2.73f, v - 2.75f / 60.0f))
    {
        const auto [cmd, w] = feed(v, 0, slidePsm);
        ++r.slideFrames;
        r.slideStepped += g_log.stepResult ? 1 : 0;
        r.slideCmdPassThrough += cmd == kBaseYaw ? 1 : 0;
        r.slideCmdActive += cmd != kBaseYaw ? 1 : 0;
        r.minWeight = std::min(r.minWeight, w);
        r.minWeightSlide = std::min(r.minWeightSlide, w);
        r.maxAirLegs = std::max(r.maxAirLegs, AirLegsOfStatus());
    }
    r.lastSlideStatus = motiondriver::Status();
    r.statusSlideFrames = std::strtoull(StatusField(r.lastSlideStatus, "slideFrames=").c_str(), nullptr, 10);
    for (int i = 0; i < 40; ++i)
    {
        const auto [cmd, w] = feed(2.7f, mm::integ::kLocoCrouch, 0u);
        ++r.exitFrames;
        r.exitCmdActive += cmd != kBaseYaw ? 1 : 0;
        r.minWeight = std::min(r.minWeight, w);
        r.maxAirLegs = std::max(r.maxAirLegs, AirLegsOfStatus());
    }
    return r;
}

// A jump in place on the same host: 30 ground feeds, a 0.7 s flight (physical air feed), 90 ground feeds. The mode of the chooser
// comes from AirDiagnostics (the same value the trace prints as air=mode<n>).
struct Jump
{
    int firstAir = -1, lastAir = -1, frames = 0;
    float riseW = -1.0f, flightMin = 9.0f, holdMin = 9.0f, goneW = -1.0f, maxStep = 0.0f, maxGround = 0.0f;
    std::vector<float> w;
};

Jump JumpScenario()
{
    Jump j;
    motionhost::FeedInput f;
    f.dt = 1.0f / 60.0f;
    f.tppActive = true;
    f.haveAir = true;
    auto feed = [&](bool aAir, float aVz) {
        f.onGround = !aAir;
        f.distToGround = aAir ? 2.0f : -2.0f;
        f.velWorld[2] = aVz;
        f.locoState = 0;
        f.psmBlock = aAir ? (mm::integ::kPsmDetailed | (15u << mm::integ::kPsmDetailShift)) : 0u;
        motiondriver::FeedAirFromInput(f);
        motionhost::Feed(f);
        const int mode = motiondriver::AirDiagnostics().movementMode;
        const int i = j.frames++;
        if (mode == 1)
        {
            if (j.firstAir < 0) j.firstAir = i;
            j.lastAir = i;
        }
        const float w = AirLegsOfStatus();
        if (!j.w.empty()) j.maxStep = std::max(j.maxStep, std::fabs(w - j.w.back()));
        j.w.push_back(w);
    };
    for (int i = 0; i < 30; ++i) feed(false, 0.0f);
    for (int i = 0; i < 42; ++i) feed(true, i < 10 ? 3.0f : -4.0f);
    for (int i = 0; i < 90; ++i) feed(false, 0.0f);
    if (j.firstAir < 0) return j;
    const auto at = [&](int k) { return j.w[static_cast<size_t>(std::min<int>(k, static_cast<int>(j.w.size()) - 1))]; };
    j.riseW = at(j.firstAir + 5);
    for (int i = 0; i < j.firstAir; ++i) j.maxGround = std::max(j.maxGround, at(i));
    for (int i = j.firstAir + 6; i <= j.lastAir; ++i) j.flightMin = std::min(j.flightMin, at(i));
    for (int i = j.lastAir + 1; i <= j.lastAir + 8; ++i) j.holdMin = std::min(j.holdMin, at(i)); // 8 feeds = 0.133 s < 0.15 s
    j.goneW = at(j.lastAir + 36);
    return j;
}

// r9 jump-out arms wiring (Status joArms=): the arm / hand layers of the weapon pose are taken off in the flight of a vehicle jump-out
// (ride feeds vehicle + workspot, then the vehicle-only feed rising at 9.6 m/s = JumpOutLatch) and never in an ordinary jump of the same host.
struct JumpOutRun
{
    int firstAir = -1, lastAir = -1, frames = 0;
    float maxBefore = 0.0f, flightMax = 0.0f, goneW = -1.0f, maxStep = 0.0f;
    float wBeforeAction = -1.0f, wEndAction = -1.0f, wAfterAction = -1.0f; // flight feeds 19 / 39 / 55 (aAction runs on feeds 20..39)
    int modeBefore = -1, modeEnd = -1, modeAfter = -1; // chooser MovementMode at the same feeds
    std::vector<float> w;
};

float JoArmsOfStatus()
{
    const std::string v = StatusField(motiondriver::Status(), "joArms=");
    return v.empty() ? -1.0f : std::strtof(v.c_str(), nullptr);
}

JumpOutRun JumpOutScenario(bool aRide, bool aAim = false, int aAction = 0)
{
    JumpOutRun j;
    int lastMode = -1;
    motionhost::FeedInput f;
    f.dt = 1.0f / 60.0f;
    f.tppActive = true;
    f.haveAir = true;
    auto feed = [&](bool aAir, float aVz, uint32_t aPsm) {
        f.onGround = !aAir;
        f.distToGround = aAir ? 2.0f : -2.0f;
        f.velWorld[2] = aVz;
        f.locoState = 0;
        f.psmBlock = aPsm;
        motiondriver::FeedAirFromInput(f);
        motionhost::Feed(f);
        const int mode = motiondriver::AirDiagnostics().movementMode;
        lastMode = mode;
        const int i = j.frames++;
        if (mode == 1)
        {
            if (j.firstAir < 0) j.firstAir = i;
            j.lastAir = i;
        }
        const float w = JoArmsOfStatus();
        if (!j.w.empty()) j.maxStep = std::max(j.maxStep, std::fabs(w - j.w.back()));
        j.w.push_back(w);
    };
    const uint32_t fall = mm::integ::kPsmDetailed | (15u << mm::integ::kPsmDetailShift);
    motiondriver::SetOverlayInputs(3, aAim ? 2 : 0, false, false); // Pistol2H, Aiming / Relaxed
    for (int i = 0; i < 30; ++i) feed(false, 0.0f, 0u);
    if (aRide)
        for (int i = 0; i < 30; ++i) feed(false, 0.0f, 0x9u); // kPsmVehicle | kPsmWorkspot: the seated ride
    for (int i = 0; i < 72; ++i) // 1.2 s flight, launch 9.6 m/s, g 16
    {
        const float vz = 9.6f - 16.0f * (static_cast<float>(i) / 60.0f);
        // r11: a hip shot (8) / a reload (2) for 20 feeds in the middle of the flight
        if (aAction != 0 && (i == 20 || i == 40))
            motiondriver::SetWeaponAction(i == 20 ? aAction : 0);
        feed(true, vz, aRide ? 0x1u : fall); // the game keeps the vehicle bit through a jump-out flight (no other block bit)
        if (i == 19) { j.wBeforeAction = j.w.back(); j.modeBefore = lastMode; }
        if (i == 39) { j.wEndAction = j.w.back(); j.modeEnd = lastMode; }
        if (i == 55) { j.wAfterAction = j.w.back(); j.modeAfter = lastMode; }
    }
    motiondriver::SetWeaponAction(0);
    for (int i = 0; i < 90; ++i) feed(false, 0.0f, 0u);
    motiondriver::SetOverlayInputs(0, 0, false, false);
    if (j.firstAir < 0) return j;
    const auto at = [&](int k) { return j.w[static_cast<size_t>(std::min<int>(k, static_cast<int>(j.w.size()) - 1))]; };
    for (int i = 0; i < j.firstAir; ++i) j.maxBefore = std::max(j.maxBefore, at(i));
    for (int i = j.firstAir; i <= j.lastAir; ++i) j.flightMax = std::max(j.flightMax, at(i));
    j.goneW = at(j.lastAir + 36);
    return j;
}

// Review (B): frames handed to the game / v1 (here the overlay-release skip) while the character is airborne must not leave the air
// weight behind: the v2 re-entry may be on the ground. 30 ground feeds, 30 air feeds (weight 1), release on for 2 air feeds, release
// off and 30 ground feeds.
struct SkipRun
{
    float inAir = -1.0f, afterSkip = -1.0f, firstGround = -1.0f;
};

SkipRun SkipScenario()
{
    SkipRun r;
    motionhost::FeedInput f;
    f.dt = 1.0f / 60.0f;
    f.tppActive = true;
    f.haveAir = true;
    auto feed = [&](bool aAir) {
        f.onGround = !aAir;
        f.distToGround = aAir ? 2.0f : -2.0f;
        f.velWorld[2] = aAir ? -4.0f : 0.0f;
        f.locoState = 0;
        f.psmBlock = aAir ? (mm::integ::kPsmDetailed | (15u << mm::integ::kPsmDetailShift)) : 0u;
        motiondriver::FeedAirFromInput(f);
        motionhost::Feed(f);
        return AirLegsOfStatus();
    };
    motiondriver::SetOverlayInputs(0, 0, false, false);
    for (int i = 0; i < 30; ++i) feed(false);
    for (int i = 0; i < 30; ++i) r.inAir = feed(true);
    motiondriver::SetOverlayInputs(0, 0, true, false);
    for (int i = 0; i < 2; ++i) r.afterSkip = feed(true);
    motiondriver::SetOverlayInputs(0, 0, false, false);
    r.firstGround = feed(false);
    for (int i = 0; i < 30; ++i) feed(false);
    return r;
}

// Melee rest pose: a drawn melee weapon sends the Game overlay (5) without the release. A melee action whose clip the full-body
// layer does not cover (melee bits, full-body flag off) and the melee guard mask the upper body only: the driver keeps running
// (rel=0, meleeMask=1) until the melee gate's release hold and the host's upper-mask fade out end, and on the first feed whose
// PSM.MeleeWeapon state is no longer a melee weapon state (holster / removal) it owns the body again. Only the full-body layer
// (flag + setting on, standing, allowed state) releases the driver (rel=1).
struct MeleeRun
{
    std::string idle, strike;
    int holdFeeds = -1;    // feeds after the melee bits clear (state Idle) until meleeMask=0
    int holsterFeeds = -1; // feeds after the melee bits clear with the state NotReady until meleeMask=0
    std::string guardOff, guardOn, fullStrike;
    std::string hostAfterHold, hostFull; // host melee status (diagnostics)
    int fullBackFeeds = -1; // feeds after the full-body strike's bits clear until rel=0
};

MeleeRun MeleeScenario()
{
    MeleeRun r;
    motionhost::FeedInput f;
    f.dt = 1.0f / 60.0f;
    f.tppActive = true;
    f.haveAir = true;
    const auto state = [](int aState) { return static_cast<uint32_t>(aState + 1) << mm::integ::kPsmMeleeStateShift; };
    auto feed = [&](uint32_t aPsm) {
        f.onGround = true;
        f.distToGround = -2.0f;
        f.velWorld[2] = 0.0f;
        f.locoState = 1; // standing (Locomotion Stand): the melee full-body layer only takes a standing body
        f.psmBlock = aPsm;
        motiondriver::FeedAirFromInput(f);
        motionhost::Feed(f);
        return StatusField(motiondriver::Status(), "rel=");
    };
    const auto masked = [&](uint32_t aPsm) { feed(aPsm); return StatusField(motiondriver::Status(), "meleeMask="); };
    const uint32_t idle = state(2), attack = mm::integ::kPsmMeleeAttack | state(6), notReady = state(0);
    const uint32_t comboAttack = mm::integ::kPsmMeleeAttack | state(12); // an attack state in the default full-body state mask
    motiondriver::SetOverlayInputs(5, 0, false, false); // PoseOverlay::Game
    for (int i = 0; i < 30; ++i) feed(idle);
    r.idle = motiondriver::Status();
    for (int i = 0; i < 30; ++i) feed(attack);
    r.strike = motiondriver::Status();
    for (int i = 0; i < 120 && r.holdFeeds < 0; ++i)
        if (masked(idle) == "0") r.holdFeeds = i + 1;
    r.hostAfterHold = motionhost::MeleeFullBodyStatus();
    for (int i = 0; i < 30; ++i) feed(idle);
    for (int i = 0; i < 30; ++i) feed(attack);
    for (int i = 0; i < 120 && r.holsterFeeds < 0; ++i)
        if (masked(notReady) == "0") r.holsterFeeds = i + 1;
    for (int i = 0; i < 60; ++i) feed(notReady); // the melee gate's release hold of the last attack runs out
    // the guard flag with melee.guard off: the driver keeps the body between strikes, nothing masked
    motionhost::SetMeleeGuard(true);
    for (int i = 0; i < 30; ++i) feed(idle);
    r.guardOff = motiondriver::Status();
    // melee.guard on: the guard masks the upper body, the driver keeps running (no release)
    motionhost::SetMeleeGuardSetting(true);
    for (int i = 0; i < 30; ++i) feed(idle);
    r.guardOn = motiondriver::Status();
    motionhost::SetMeleeGuardSetting(false);
    motionhost::SetMeleeGuard(false);
    for (int i = 0; i < 60; ++i) feed(notReady);
    // a weapon covered by the full-body clips (flag + setting on), standing attack: the full-body layer owns the whole body -> release
    motionhost::SetMeleeFullBody(true);
    motionhost::SetMeleeFullBodySetting(true);
    for (int i = 0; i < 30; ++i) feed(idle);
    for (int i = 0; i < 30; ++i) feed(comboAttack);
    r.fullStrike = motiondriver::Status();
    r.hostFull = motionhost::MeleeFullBodyStatus();
    for (int i = 0; i < 180 && r.fullBackFeeds < 0; ++i)
        if (feed(idle) == "0") r.fullBackFeeds = i + 1;
    motionhost::SetMeleeFullBody(false);
    for (int i = 0; i < 10; ++i) feed(0u);
    motiondriver::SetOverlayInputs(0, 0, false, false);
    return r;
}

// Review (A): a jump into a climb. The leg layer of the weapon pose still composes over the traversal montage (overlayWeight 1), so
// the air weight must hand over like a landing (hold 0.15 s, fall 0.25 s) and not be reset on the first montage frame (a one-frame
// pop of the legs from the clip to the weapon stance). The first montage of the version 2 traversal file that is a climb (FrontLedge
// window, >= 0.9 s) is forced on an obstacle built from its own FrontLedge contact, as the integration test's check 8i does, but after
// 30 airborne feeds (detail 14, weight 1). 120 Hz feeds.
struct TravRun
{
    bool ran = false;
    int montageFeeds = 0;
    float beforeTrav = -1.0f, firstMontage = -1.0f, maxStep = 0.0f, atEnd = -1.0f;
    std::string note;
};

TravRun TravFromAirScenario(const std::string& aData, const Rig& aRig)
{
    namespace trv = utpp::mm::traversal;
    namespace motiontrav = utpp::mm::traversal::v2;
    TravRun r;
    trv::TraversalFile tf;
    std::string terr;
    if (!trv::LoadTraversalFile(DataPaths(aData, "woman").traversal, tf, terr))
    {
        r.note = "traversal file: " + terr;
        return r;
    }
    const trv::TravMontage* mon = nullptr;
    int fw = -1;
    for (const trv::TravMontage& m : tf.montages)
    {
        const bool climb = m.action == "Mantle" || m.action == "ClimbStart" || m.name.find("_Catch_Mantle_") != std::string::npos ||
                           m.name.find("_Catch_Cliff_") != std::string::npos;
        int w = -1;
        for (size_t k = 0; k < m.windows.size(); ++k)
            if (m.windows[k].target == "FrontLedge" && w < 0) w = static_cast<int>(k);
        if (climb && w >= 0 && m.frames / m.fps >= 0.9)
        {
            mon = &m;
            fw = w;
            break;
        }
    }
    if (!mon)
    {
        r.note = "no climb montage with a FrontLedge window >= 0.9 s";
        return r;
    }
    const trv::TravMontage& m = *mon;
    const float dtI = 1.0f / 120.0f;
    float cap[3] = {0, 0, 0};
    auto feedI = [&](int aDetail, bool aAir, const float* aVel) {
        motionhost::FeedInput in;
        for (int k = 0; k < 3; ++k)
        {
            in.velWorld[k] = aVel[k];
            cap[k] += aVel[k] * dtI;
        }
        const bool trav = aDetail == 8 || aDetail == 9;
        in.locoState = trav ? mm::integ::kLocoOther : mm::integ::kLocoStand;
        in.tppActive = true;
        in.dt = dtI;
        in.haveExtra = true;
        std::memcpy(in.capsulePos, cap, sizeof cap);
        in.haveAir = true;
        in.onGround = !(trav || aAir);
        in.distToGround = aAir ? 2.0f : -2.0f;
        if (aDetail >= 0)
            in.psmBlock = mm::integ::kPsmDetailed | (static_cast<uint32_t>(aDetail + 1) << mm::integ::kPsmDetailShift);
        motiondriver::FeedAirFromInput(in);
        motionhost::Feed(in);
        return AirLegsOfStatus();
    };
    const trv::RootKey& r0 = m.root.front();
    const trv::TravWindow& win = m.windows[static_cast<size_t>(fw)];
    const trv::RootKey& re = m.root[static_cast<size_t>(win.endFrame)];
    trv::V3 fc = trv::RotZ(re.pos + trv::RotZ(win.offset, re.yaw) - r0.pos, -r0.yaw);
    fc.z = std::max(fc.z, 0.2);
    motiondriver::TestForceTraversal(m.name, false);
    motiondriver::SetEnabled(false);
    ResetBase(aRig);
    motiondriver::SetEnabled(true);
    motiondriver::SetOverlayInputs(0, 0, false, false);
    const float zero[3] = {0, 0, 0};
    for (int i = 0; i < 12; ++i) feedI(-1, false, zero);
    for (int i = 0; i < 40; ++i) r.beforeTrav = feedI(14, true, zero);
    motiontrav::GameObstacle o;
    o.action = motiontrav::GameAction::Climb;
    o.topPoint = {fc.x, fc.y, fc.z};
    o.collisionNormal = {0.0, -1.0, 0.0};
    o.topExtent = 0.4;
    o.obstacleEnd = {fc.x, fc.y + 0.4, fc.z};
    o.behindPoint = {fc.x, fc.y + 0.7, 0.0};
    o.inputDirection = {0.0, 1.0, 0.0};
    o.hasHands = true;
    o.leftHand = o.topPoint;
    o.rightHand = o.topPoint;
    motiondriver::TraversalEvent e;
    e.action = static_cast<int32_t>(o.action);
    auto put = [](float* d, const trv::V3& v) {
        d[0] = static_cast<float>(v.x);
        d[1] = static_cast<float>(v.y);
        d[2] = static_cast<float>(v.z);
    };
    put(e.topPoint, o.topPoint);
    put(e.collisionNormal, o.collisionNormal);
    e.topExtent = static_cast<float>(o.topExtent);
    put(e.behindPoint, o.behindPoint);
    put(e.obstacleEnd, o.obstacleEnd);
    put(e.inputDirection, o.inputDirection);
    e.hasHands = o.hasHands;
    put(e.leftHand, o.leftHand);
    put(e.rightHand, o.rightHand);
    put(e.playerPos, trv::V3{cap[0], cap[1], cap[2]});
    e.ownerForward[1] = 1.0f;
    e.climbSpeedModifier = 1.0f;
    const motiondriver::TraversalProbe q0 = motiondriver::ProbeTraversal();
    const bool fed = motiondriver::FeedTraversal(e);
    const motiondriver::TraversalProbe q1 = motiondriver::ProbeTraversal();
    if (!(fed && q1.accepted == q0.accepted + 1 && q1.lastResult == 0))
    {
        r.note = "the forced traversal was not accepted from the air";
        motiondriver::TestForceTraversal("", true);
        return r;
    }
    motiondriver::TraversalProbe last = q1;
    float prev = r.beforeTrav;
    const int maxFeeds = static_cast<int>(m.frames / m.fps / dtI) + 40;
    for (int i = 0; i < maxFeeds; ++i)
    {
        float v[3] = {0, 0, 0};
        if (last.frames > q1.frames)
            for (int k = 0; k < 3; ++k) v[k] = (last.root[k] - cap[k]) / dtI;
        const float w = feedI(8, false, v);
        const motiondriver::TraversalProbe pc = motiondriver::ProbeTraversal();
        r.maxStep = std::max(r.maxStep, std::fabs(w - prev));
        prev = w;
        if (pc.frames > last.frames)
        {
            if (r.montageFeeds == 0) r.firstMontage = w;
            ++r.montageFeeds;
            r.atEnd = w;
        }
        last = pc;
        if (pc.handbacks > q1.handbacks) break;
    }
    r.ran = r.montageFeeds >= 5;
    motiondriver::TestForceTraversal("", true);
    return r;
}

std::string Show(const Run& r)
{
    char b[256];
    std::snprintf(b, sizeof b, "sprint cmd active %d/%d, slide: stepped %d/%d pass-through %d active %d, exit cmd active %d/%d, min weight %.3f (slide %.3f)",
                  r.sprintCmdActive, r.sprintLate, r.slideStepped, r.slideFrames, r.slideCmdPassThrough, r.slideCmdActive,
                  r.exitCmdActive, r.exitFrames, static_cast<double>(r.minWeight), static_cast<double>(r.minWeightSlide));
    return b;
}
} // namespace

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::puts("usage: utpp_motion_driver_slide_host_test <data dir> <base database>");
        return 2;
    }
    const std::string data = argv[1];
    motiondriver::Paths slide = DataPaths(data, "woman");
    slide.files.db = data + "\\utpp_motion_woman_knee_slide.utppmm";
    slide.files.chooser = data + "\\utpp_motion_woman_knee_slide.chooser.json";
    Rig rig;
    if (!RigFromDb(slide.files.db, rig))
    {
        std::puts("FAIL slide database / rig");
        return 3;
    }
    g_baseDb = std::wstring(argv[2], argv[2] + std::strlen(argv[2]));
    motiondriver::SetPathsOverride(0, slide);
    motiondriver::SetRigBody(0);
    motiondriver::SetOverlayInputs(0, 0, false, false);
    const motionhost::MotionHooks hooks{&WStep, &WApply, &motiondriver::OnArmed, &motiondriver::OnDisarmed, &motiondriver::Frame, &motiondriver::OwnsTraversal,
                               &motiondriver::OwnsSlide};
    motionhost::SetMotionHooks(&hooks);
    Check(ResetBase(rig), "real host initialized");
    motionhost::SetCapsuleSync(true);
    motionhost::SetHoldPose(true);
    motiondriver::SetEnabled(true);
    (void)motiondriver::Step({}, 0, 1.0f / 60.0f);
    motiondriver::WaitLoader();
    Check(motiondriver::SlideBundleDeclared(), "slide bundle declared");

    const Run off = Scenario(rig, false);
    std::printf("INFO slide off: %s\n", Show(off).c_str());
    Check(off.sprintCmdActive == off.sprintLate, "off: body-yaw command active while sprinting", Show(off));
    Check(off.slideStepped == off.slideFrames, "off: motion owns every slide frame (the existing hold)", Show(off));
    Check(off.slideCmdActive == off.slideFrames && off.slideCmdPassThrough == 0, "off: body-yaw command stays active in the slide (pre-slide behaviour)", Show(off));
    Check(off.statusSlideFrames == 0, "off: no slide frames counted");

    // fresh host state for the second run
    motiondriver::SetEnabled(false);
    Check(ResetBase(rig), "real host re-initialized");
    motionhost::SetCapsuleSync(true);
    motionhost::SetHoldPose(true);
    motiondriver::SetEnabled(true);
    (void)motiondriver::Step({}, 0, 1.0f / 60.0f);
    motiondriver::WaitLoader();
    const Run on = Scenario(rig, true);
    std::printf("INFO slide on : %s\n", Show(on).c_str());
    Check(on.sprintCmdActive == on.sprintLate, "on: body-yaw command active while sprinting", Show(on));
    Check(on.slideStepped == on.slideFrames, "on: motion owns every slide frame", Show(on));
    Check(on.slideCmdPassThrough == on.slideFrames && on.slideCmdActive == 0, "on: no body-yaw command during the slide (pass-through of the base yaw)", Show(on));
    Check(on.exitCmdActive >= on.exitFrames - 2, "on: body-yaw command active again after the slide", Show(on));
    Check(on.statusSlideFrames == static_cast<uint64_t>(on.slideFrames), "on: status slideFrames counts the slide", on.lastSlideStatus);
    Check(on.minWeight >= 0.99f, "on: published weight never drops across slide entry / exit (no pop)", Show(on));
    Check(off.minWeight >= 0.99f, "off: published weight (pre-slide behaviour) also stays 1", Show(off));
    Check(on.maxAirLegs == 0.0f, "on: the slide (chooser mode 2) never fades the weapon pose's legs (airLegs stays 0)",
          "max airLegs " + std::to_string(on.maxAirLegs));

    // fix2/hip P6 K1 wiring on the real air of the same host
    const Jump jump = JumpScenario();
    char jd[256];
    std::snprintf(jd, sizeof jd, "air feeds %d..%d of %d, airLegs 5 feeds after take-off %.3f, flight min %.3f, 8 feeds after landing min %.3f, "
                  "36 feeds after landing %.3f, max step %.3f, ground max %.3f", jump.firstAir, jump.lastAir, jump.frames,
                  static_cast<double>(jump.riseW), static_cast<double>(jump.flightMin), static_cast<double>(jump.holdMin),
                  static_cast<double>(jump.goneW), static_cast<double>(jump.maxStep), static_cast<double>(jump.maxGround));
    std::printf("INFO jump: %s\n", jd);
    Check(jump.firstAir >= 0 && jump.lastAir - jump.firstAir >= 30, "jump: the physical air feed reaches chooser mode 1 for the flight", jd);
    Check(jump.maxGround == 0.0f, "jump: airLegs is 0 before the take-off", jd);
    Check(jump.riseW >= 0.9f && jump.flightMin >= 0.999f, "jump: airLegs >= 0.9 within 5 feeds of the take-off and 1 for the rest of the flight", jd);
    Check(jump.holdMin >= 0.999f, "jump: airLegs holds 1 for 0.13 s after the landing (hold 0.15 s)", jd);
    Check(jump.goneW <= 0.001f, "jump: airLegs is 0 within 0.6 s of the landing (hold 0.15 s + fall 0.35 s)", jd);
    Check(jump.maxStep <= 0.23f, "jump: airLegs changes by at most 0.23 per feed", jd);

    // r9 jump-out arms: the vehicle jump-out flight takes the arm layers off the weapon pose, an ordinary jump never does
    {
        const JumpOutRun jo = JumpOutScenario(true);
        char od[256];
        std::snprintf(od, sizeof od, "air feeds %d..%d of %d, joArms before %.3f, flight max %.3f, 36 feeds after landing %.3f, max step %.3f", jo.firstAir, jo.lastAir,
                      jo.frames, static_cast<double>(jo.maxBefore), static_cast<double>(jo.flightMax), static_cast<double>(jo.goneW), static_cast<double>(jo.maxStep));
        std::printf("INFO jump-out: %s\n", od);
        Check(jo.firstAir >= 0 && jo.maxBefore == 0.0f, "jump-out: joArms is 0 on the ground and in the ride", od);
        Check(jo.flightMax >= 0.999f, "jump-out: joArms reaches 1 in the flight of a vehicle jump-out", od);
        Check(jo.goneW <= 0.001f && jo.maxStep <= 0.36f, "jump-out: joArms is 0 within 0.6 s of the landing and changes by at most 0.36 per feed", od);
        const JumpOutRun ctl = JumpOutScenario(false);
        std::snprintf(od, sizeof od, "air feeds %d..%d of %d, joArms flight max %.3f", ctl.firstAir, ctl.lastAir, ctl.frames, static_cast<double>(ctl.flightMax));
        std::printf("INFO ordinary jump: %s\n", od);
        Check(ctl.firstAir >= 0 && ctl.flightMax == 0.0f, "ordinary jump (same host, no ride): joArms stays exactly 0", od);
        const JumpOutRun aim = JumpOutScenario(true, true);
        std::snprintf(od, sizeof od, "air feeds %d..%d of %d, joArms flight max %.3f", aim.firstAir, aim.lastAir, aim.frames, static_cast<double>(aim.flightMax));
        std::printf("INFO aiming jump-out: %s\n", od);
        Check(aim.firstAir >= 0 && aim.flightMax == 0.0f, "jump-out while aiming (Pistol2H, Aiming): the weapon arms stay, joArms stays exactly 0", od);
        // r11 review: a hip shot / a reload in the flight keeps (brings back) the weapon arms: joArms falls to 0 within the request and rises again after it
        for (const int act : {8, 2})
        {
            const JumpOutRun r = JumpOutScenario(true, false, act);
            std::snprintf(od, sizeof od, "action %d on feeds 20..39: joArms before %.3f, at the end %.3f, 16 feeds after %.3f, flight max %.3f", act,
                          static_cast<double>(r.wBeforeAction), static_cast<double>(r.wEndAction), static_cast<double>(r.wAfterAction), static_cast<double>(r.flightMax));
            std::printf("INFO jump-out %s: %s, chooser mode at the samples %d / %d / %d\n", act == 8 ? "hip shot" : "reload", od, r.modeBefore, r.modeEnd, r.modeAfter);
            // the sample feeds must lie in the flight (chooser mode 1), else the rise-back check would fail for another reason
            Check(r.modeBefore == 1 && r.modeEnd == 1 && r.modeAfter == 1, "jump-out action cases: the sample feeds 19 / 39 / 55 of the flight are in chooser mode 1", od);
            Check(r.wBeforeAction >= 0.999f && r.wEndAction <= 0.001f && r.wAfterAction >= 0.999f,
                  act == 8 ? "jump-out hip shot mid-flight: the weapon arms return (joArms 1 -> 0 within the shot) and the flight fades them again after it"
                           : "jump-out reload mid-flight: the weapon arms return (joArms 1 -> 0 within the reload) and the flight fades them again after it", od);
        }
    }

    // review (B): a skip (frames handed to the game / v1) drops the air weight
    const SkipRun sk = SkipScenario();
    char sd[160];
    std::snprintf(sd, sizeof sd, "airLegs in the air %.3f, after 2 skipped feeds %.3f, first feed after the re-entry on the ground %.3f",
                  static_cast<double>(sk.inAir), static_cast<double>(sk.afterSkip), static_cast<double>(sk.firstGround));
    std::printf("INFO skip: %s\n", sd);
    Check(sk.inAir >= 0.999f, "skip: the air weight is 1 before the skip", sd);
    Check(sk.afterSkip == 0.0f && sk.firstGround == 0.0f, "skip: frames handed to the game / base reset the air weight (the re-entry on the ground starts at 0)", sd);

    // melee rest pose: the host's melee gate drives the Game overlay release
    {
        const MeleeRun m = MeleeScenario();
        char md[200];
        std::snprintf(md, sizeof md, "mask hold feeds %d, holster feeds %d, full-body back feeds %d", m.holdFeeds, m.holsterFeeds,
                      m.fullBackFeeds);
        std::printf("INFO melee: %s\n", md);
        std::printf("INFO melee host after hold: %s\nINFO melee host full strike: %s\n", m.hostAfterHold.c_str(), m.hostFull.c_str());
        const auto status = [](const std::string& aS) {
            return "rel=" + StatusField(aS, "rel=") + " meleeMask=" + StatusField(aS, "meleeMask=") + " skip=" + StatusField(aS, "skip=");
        };
        std::printf("INFO melee states: idle %s | strike %s | guardOff %s | guardOn %s | fullStrike %s\n", status(m.idle).c_str(),
                    status(m.strike).c_str(), status(m.guardOff).c_str(), status(m.guardOn).c_str(), status(m.fullStrike).c_str());
        Check(StatusField(m.idle, "rel=") == "0" && StatusField(m.idle, "skip=") == "none" && StatusField(m.idle, "meleeMask=") == "0",
              "melee: Game overlay, weapon state Idle, no melee bits -> motion keeps the body", m.idle.substr(0, 200));
        Check(StatusField(m.strike, "rel=") == "0" && StatusField(m.strike, "skip=") == "none" && StatusField(m.strike, "meleeMask=") == "1",
              "melee: melee bits, weapon not covered by the full-body clips -> masked upper body, the driver keeps running", status(m.strike));
        // release hold 0.55 s at 60 Hz = 33 feeds, then the host's eased upper-mask fade out (kMeleeReleaseFadeOutSeconds 0.35 s =
        // 21 feeds) keeps the masked upper body; the full-body layer is off here (no flag), so no fade on top
        Check(m.holdFeeds >= 30 + 21 && m.holdFeeds <= 40 + 21, "melee: the mask ends with the release hold and the upper-mask fade out", md);
        Check(m.holsterFeeds >= 1 && m.holsterFeeds <= 1, "melee: weapon state NotReady (holster / removal) -> the driver owns the body on the first feed", md);
        Check(StatusField(m.guardOff, "rel=") == "0" && StatusField(m.guardOff, "skip=") == "none" && StatusField(m.guardOff, "meleeMask=") == "0",
              "melee: guard flag on with melee.guard off -> motion keeps the body between strikes", m.guardOff.substr(0, 200));
        Check(StatusField(m.guardOn, "rel=") == "0" && StatusField(m.guardOn, "skip=") == "none" && StatusField(m.guardOn, "meleeMask=") == "1",
              "melee: guard on -> masked upper body, the driver keeps the lower body", status(m.guardOn));
        Check(StatusField(m.fullStrike, "rel=") == "1" && StatusField(m.fullStrike, "meleeMask=") == "0",
              "melee: covered weapon, standing attack -> the full-body layer owns the body (release)", status(m.fullStrike));
        Check(m.fullBackFeeds >= 1, "melee: the full-body release ends after the strike", md);
    }

    // review (A): a jump into a climb hands the weight over like a landing, without a reset on the first montage frame
    const TravRun tr = TravFromAirScenario(data, rig);
    char td[256];
    std::snprintf(td, sizeof td, "%s montage feeds %d, airLegs before %.3f, first montage feed %.3f, max step per feed %.3f, last montage feed %.3f",
                  tr.note.c_str(), tr.montageFeeds, static_cast<double>(tr.beforeTrav), static_cast<double>(tr.firstMontage),
                  static_cast<double>(tr.maxStep), static_cast<double>(tr.atEnd));
    std::printf("INFO trav: %s\n", td);
    Check(tr.ran && tr.beforeTrav >= 0.999f, "trav: a climb forced from the air plays and the weight was 1 before it", td);
    Check(tr.ran && tr.firstMontage >= 0.999f && tr.maxStep <= 0.23f,
          "trav: the air weight is held on the first montage frame and never changes by more than 0.23 per feed (no reset pop)", td);
    Check(tr.montageFeeds >= 50 && tr.atEnd >= 0.0f && tr.atEnd <= 0.15f, "trav: the weight handed over (hold 0.15 s, then falling: <= 0.15 at the end of a 0.48 s montage) and is not stuck", td);

    motiondriver::SetEnabled(false);
    motiondriver::Shutdown();
    std::printf("slide host: %d PASS %d FAIL\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
