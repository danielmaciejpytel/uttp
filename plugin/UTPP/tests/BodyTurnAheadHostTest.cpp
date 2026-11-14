// The coming tick's body turn derived by the host (MotionHost.cpp Feed: the last BodyYawCommand, issued for the yaw this feed reads).
// Runs the real host + motion driver through a standing start towards a target behind-left of V and drives BodyYawCommand / Feed in
// two orders:
//  - game order: the command is issued for the current yaw, the feed reads that yaw, the game applies the command after the feed;
//    the pose of the feed is drawn with the capsule after that turn. The derived turn must equal the command step, and the drawn
//    heading (fed yaw + turn + offset yaw) must advance by the clip's own turn on every feed, also around a long frame;
//  - command applied before the feed (offline order): the fed yaw already holds the turn, the derived turn must stay 0 on every
//    feed (also for steps below the match tolerance) and the run must equal one with the turn forced to 0.
// A command the game applies only partly may cost one frame of heading (the part not applied) but never more.
#define main UnusedIntegrationMain
#include "MotionDriverIntegrationTest.cpp"
#undef main
#include <limits>
namespace
{
float g_hostAhead = 0; // FeedInput::capsuleYawAhead as the host handed it to the motion driver on the last feed (rad)
bool AheadStep(const motionhost::FeedInput& aIn, uint32_t aGate, float aDt)
{
    g_hostAhead = aIn.capsuleYawAhead;
    return WStep(aIn, aGate, aDt);
}
constexpr float kDeg = mm::integ::kDegToRad;
float WrapDeg(float a) { return std::remainder(a, 360.0f); }

enum class Order { Game, AppliedFirst };
enum class Apply { Full, Half, Cap1 };
struct RunSpec
{
    Order order = Order::Game;
    Apply apply = Apply::Full;
    bool longFrame = true;  // one 60 ms frame while the body turns fast
    bool skipOne = true;    // one tick without a command (game order) a few feeds after the long frame
    bool forceZero = false; // FeedInput::yawAheadDeg = 0 (the turn is not derived)
    float targetDeg = 135.0f;
    float laterTurnDeg = 30.0f; // from feed kLater on, the target moves by this much (a followed turn: steps decay below 0.5 deg)
};
constexpr int kStand = 60, kLater = 200, kFeeds = 400;
struct Rec
{
    bool owned = false, prepared = false, commanded = false, longFrame = false, skipped = false;
    float dt = 0, fed = 0, cmdStep = 0, applied = 0, ahead = 0, anim = 0, off = 0;
};
struct RunOut
{
    bool ok = false;
    std::vector<Rec> r;
    int longAt = -1, skipAt = -1;
};

RunOut Run(const Rig& aRig, const RunSpec& s)
{
    RunOut out;
    if (!ResetBase(aRig))
        return out;
    motionhost::SetOffsetRoot(true);
    motionhost::SetHoldPose(true);
    motiondriver::SetEnabled(true);
    motionhost::FeedInput f;
    f.tppActive = true;
    f.locoState = mm::integ::kLocoStand;
    f.haveAir = true;
    f.onGround = true;
    f.distToGround = 0;
    f.psmBlock = mm::integ::kPsmWeaponKnown;
    float yaw = 0, vx = 0, vy = 0, prevCmdStep = 0;
    for (int i = 0; i < kFeeds; ++i)
    {
        const bool moving = i >= kStand;
        float dt = 1.0f / 60.0f;
        Rec rec;
        // the long frame lands on a feed whose turn is fast (the previous command stepped more than 1.5 deg)
        if (s.longFrame && moving && out.longAt < 0 && std::fabs(prevCmdStep) > 1.5f)
        {
            dt = 0.06f;
            out.longAt = i;
            rec.longFrame = true;
        }
        const float target = s.targetDeg + (i >= kLater ? s.laterTurnDeg : 0.0f);
        if (moving)
        {
            const float sp = std::fmin(3.5f, std::sqrt(vx * vx + vy * vy) + 8.0f * dt);
            vx = -std::sin(target * kDeg) * sp;
            vy = std::cos(target * kDeg) * sp;
        }
        f.dt = dt;
        f.velWorld[0] = vx;
        f.velWorld[1] = vy;
        f.move[0] = 0;
        f.move[1] = moving ? 1.0f : 0.0f;
        f.cameraYawDeg = 0;
        f.haveTarget = moving;
        f.targetYawDeg = target;
        f.yawAheadDeg = s.forceZero ? 0.0f : std::numeric_limits<float>::quiet_NaN();
        const bool skip = s.skipOne && s.order == Order::Game && out.longAt >= 0 && out.skipAt < 0 && i == out.longAt + 8;
        if (skip)
        {
            out.skipAt = i;
            rec.skipped = true;
        }
        auto applied = [&](float step) {
            switch (s.apply)
            {
            case Apply::Half: return 0.5f * step;
            case Apply::Cap1: return std::clamp(step, -1.0f, 1.0f);
            default: return step;
            }
        };
        float cmdStep = 0;
        if (!skip && motionhost::BodyTurnActive())
        {
            const float cmd = motionhost::BodyYawCommand(yaw, yaw);
            cmdStep = WrapDeg(cmd - yaw);
            rec.commanded = true;
        }
        if (s.order == Order::AppliedFirst)
        {
            rec.applied = applied(cmdStep);
            yaw = WrapDeg(yaw + rec.applied);
        }
        f.playerYawDeg = yaw;
        rec.fed = yaw;
        motiondriver::FeedAirFromInput(f);
        g_log = {};
        g_hostAhead = std::numeric_limits<float>::quiet_NaN();
        motionhost::Feed(f);
        const motiondriver::RootProbe p = motiondriver::TestRootProbe();
        rec.owned = g_log.stepResult;
        rec.prepared = p.prepared;
        rec.dt = dt;
        rec.cmdStep = cmdStep;
        rec.ahead = g_hostAhead / kDeg;
        rec.anim = p.animYawDelta / kDeg;
        rec.off = p.after[2] / kDeg;
        if (s.order == Order::Game)
        {
            rec.applied = applied(cmdStep);
            yaw = WrapDeg(yaw + rec.applied);
        }
        prevCmdStep = cmdStep;
        out.r.push_back(rec);
    }
    out.ok = true;
    return out;
}

// Heading of the pose of feed i as the game draws it (capsule after this tick's applied turn + offset yaw) and as the frame intends it
// (fed yaw + derived turn + offset yaw).
float DrawnGameOrder(const Rec& r, Order o) { return o == Order::Game ? r.fed + r.applied + r.off : r.fed + r.off; }
float Intended(const Rec& r) { return r.fed + r.ahead + r.off; }

struct Stats
{
    int owned = 0, prepared = 0, commanded = 0, nonZeroAhead = 0, aheadMismatch = 0, nanAhead = 0;
    float maxAheadErr = 0, maxIntendedDev = 0, maxDrawnDev = 0, maxDrawnDevLong = 0, maxShortfallExcess = 0, maxEndGap = 0, maxAbsAhead = 0;
    int devAt = -1, longDevAt = -1;
};
Stats Measure(const RunOut& o, Order order)
{
    Stats st;
    const auto& r = o.r;
    for (size_t i = 0; i < r.size(); ++i)
    {
        const Rec& x = r[i];
        st.owned += x.owned ? 1 : 0;
        st.prepared += x.prepared ? 1 : 0;
        st.commanded += x.commanded ? 1 : 0;
        if (!std::isfinite(x.ahead))
        {
            ++st.nanAhead;
            continue;
        }
        st.nonZeroAhead += x.ahead != 0.0f ? 1 : 0;
        st.maxAbsAhead = std::fmax(st.maxAbsAhead, std::fabs(x.ahead));
        // game order: the derived turn is the step of the command issued for this feed's yaw
        const float expect = order == Order::Game ? x.cmdStep : 0.0f;
        const float e = std::fabs(WrapDeg(x.ahead - expect));
        if (e > 1e-3f)
            ++st.aheadMismatch;
        st.maxAheadErr = std::fmax(st.maxAheadErr, e);
        // the frame never sees more than this feed's shortfall: drawn - intended = applied - derived turn
        if (order == Order::Game)
        {
            const float gap = WrapDeg(DrawnGameOrder(x, order) - Intended(x));
            st.maxShortfallExcess = std::fmax(st.maxShortfallExcess, std::fabs(gap - WrapDeg(x.applied - x.ahead)));
        }
        if (i == 0 || !x.prepared || !r[i - 1].prepared || !x.owned || !r[i - 1].owned)
            continue;
        const float dI = WrapDeg(Intended(x) - Intended(r[i - 1])) - x.anim;
        const float dD = WrapDeg(DrawnGameOrder(x, order) - DrawnGameOrder(r[i - 1], order)) - x.anim;
        if (std::fabs(dI) > st.maxIntendedDev)
            st.maxIntendedDev = std::fabs(dI), st.devAt = int(i);
        st.maxDrawnDev = std::fmax(st.maxDrawnDev, std::fabs(dD));
        if (o.longAt >= 0 && int(i) >= o.longAt - 1 && int(i) <= o.longAt + 3 && std::fabs(dD) > st.maxDrawnDevLong)
            st.maxDrawnDevLong = std::fabs(dD), st.longDevAt = int(i);
    }
    // last 40 feeds: no turn left, drawn = intended
    for (size_t i = r.size() - 40; i < r.size(); ++i)
        st.maxEndGap = std::fmax(st.maxEndGap, std::fabs(WrapDeg(DrawnGameOrder(r[i], order) - Intended(r[i]))));
    return st;
}
std::string Describe(const char* aName, const RunOut& o, const Stats& s)
{
    char b[640];
    std::snprintf(b, sizeof b,
                  "%s: owned %d prepared %d commanded %d nonZeroAhead %d (max %.3f deg) aheadErr %.5f deg (%d feeds) intendedDev %.3f deg "
                  "(feed %d) drawnDev %.3f deg, at the long frame %.3f deg (feed %d, long frame %d, skip %d) shortfallExcess %.5f endGap %.5f",
                  aName, s.owned, s.prepared, s.commanded, s.nonZeroAhead, static_cast<double>(s.maxAbsAhead), static_cast<double>(s.maxAheadErr),
                  s.aheadMismatch, static_cast<double>(s.maxIntendedDev), s.devAt, static_cast<double>(s.maxDrawnDev),
                  static_cast<double>(s.maxDrawnDevLong), s.longDevAt, o.longAt, o.skipAt, static_cast<double>(s.maxShortfallExcess),
                  static_cast<double>(s.maxEndGap));
    return b;
}
void Dump(const char* aName, const RunOut& o, int aFrom, int aTo)
{
    for (int i = std::max(aFrom, 0); i < std::min<int>(aTo, int(o.r.size())); ++i)
    {
        const Rec& x = o.r[size_t(i)];
        std::printf("  %s %3d dt %.4f own %d prep %d cmd %d step %8.3f appl %8.3f ahead %8.3f anim %8.3f off %8.3f fed %8.3f%s%s\n", aName, i,
                    static_cast<double>(x.dt), x.owned ? 1 : 0, x.prepared ? 1 : 0, x.commanded ? 1 : 0, static_cast<double>(x.cmdStep),
                    static_cast<double>(x.applied), static_cast<double>(x.ahead), static_cast<double>(x.anim), static_cast<double>(x.off),
                    static_cast<double>(x.fed), x.longFrame ? " LONG" : "", x.skipped ? " SKIP" : "");
    }
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::puts("usage BodyTurnAheadHostTest <driver-data> <basedb> [--dump]");
        return 2;
    }
    const bool dump = argc > 3 && std::string(argv[3]) == "--dump";
    Rig rig;
    if (!RigFromDb(DataPaths(argv[1], "woman").files.db, rig))
        return 3;
    g_baseDb = std::wstring(argv[2], argv[2] + std::strlen(argv[2]));
    motiondriver::SetPathsOverride(0, DataPaths(argv[1], "woman"));
    motiondriver::SetRigBody(0);
    motiondriver::SetOverlayInputs(0, 0, false, false);
    const motionhost::MotionHooks hooks{&AheadStep, &WApply, &motiondriver::OnArmed, &motiondriver::OnDisarmed, &WFrame, &motiondriver::OwnsTraversal};
    motionhost::SetMotionHooks(&hooks);
    Check(ResetBase(rig), "real host initialized");
    motiondriver::SetEnabled(true);
    motiondriver::Step({}, 0, 1.f / 60);
    motiondriver::WaitLoader();

    // ---- game order, command applied in full
    RunSpec game;
    const RunOut g = Run(rig, game);
    const Stats gs = Measure(g, Order::Game);
    std::printf("%s\n", Describe("game order", g, gs).c_str());
    if (dump)
    {
        Dump("game", g, g.longAt - 6, g.longAt + 14);
        Dump("game", g, kLater - 2, kLater + 60);
    }
    const bool fixture = g.ok && gs.owned >= 200 && gs.prepared >= 200 && gs.commanded >= 20 && g.longAt >= 0;
    Check(fixture, "fixture: motion owns the start, the body turn commands the capsule, the long frame lands on a fast turn",
          Describe("game order", g, gs));
    Check(gs.nanAhead == 0 && gs.nonZeroAhead >= 10, "game order: the derived turn is non-zero during the body turn");
    Check(gs.aheadMismatch == 0, "game order: the derived turn equals the step of the command issued for the fed yaw");
    Check(g.skipAt >= 0 && g.r[size_t(g.skipAt)].ahead == 0.0f, "game order: a tick without a command derives no turn");
    Check(gs.maxIntendedDev <= 0.05f, "game order: the intended heading advances by the clip's turn on every feed");
    Check(gs.maxDrawnDevLong <= 0.05f, "game order: no heading dip at the long frame (drawn heading follows the clip)");
    Check(gs.maxEndGap <= 1e-3f, "game order: drawn = intended once the turn is done");

    // ---- control: the same run without the derived turn has the dip (the check above is not vacuous)
    RunSpec zero = game;
    zero.forceZero = true;
    const RunOut z = Run(rig, zero);
    const Stats zs = Measure(z, Order::Game);
    std::printf("%s\n", Describe("game order, turn forced to 0", z, zs).c_str());
    if (dump)
        Dump("zero", z, z.longAt - 6, z.longAt + 14);
    Check(z.ok && z.longAt >= 0 && zs.maxDrawnDevLong >= 2.0f, "control: without the derived turn the drawn heading dips at the long frame");

    // ---- command applied before the feed: nothing derived, identical to the forced-zero run
    RunSpec first;
    first.order = Order::AppliedFirst;
    first.skipOne = false;
    const RunOut a = Run(rig, first);
    const Stats as = Measure(a, Order::AppliedFirst);
    std::printf("%s\n", Describe("applied first", a, as).c_str());
    int smallSteps = 0;
    for (const Rec& x : a.r)
        smallSteps += x.commanded && x.cmdStep != 0.0f && std::fabs(x.cmdStep) <= 0.5f ? 1 : 0;
    Check(a.ok && as.commanded >= 20 && smallSteps >= 3, "fixture: applied-first run turns, with steps below the 0.5 deg match tolerance",
          "small steps " + std::to_string(smallSteps));
    Check(as.nanAhead == 0 && as.nonZeroAhead == 0, "applied first: the derived turn stays 0 on every feed",
          "non-zero on " + std::to_string(as.nonZeroAhead) + " feeds, max " + std::to_string(as.maxAbsAhead) + " deg");
    RunSpec firstZero = first;
    firstZero.forceZero = true;
    const RunOut az = Run(rig, firstZero);
    float maxDiff = 0;
    bool sameLen = az.r.size() == a.r.size();
    for (size_t i = 0; sameLen && i < a.r.size(); ++i)
        maxDiff = std::fmax(maxDiff, std::fmax(std::fabs(WrapDeg(a.r[i].off - az.r[i].off)), std::fabs(WrapDeg(a.r[i].fed - az.r[i].fed))));
    Check(sameLen && maxDiff <= 1e-4f, "applied first: the run equals the one with the turn forced to 0", "max diff " + std::to_string(maxDiff) + " deg");

    // ---- the game applies only part of the command: one frame of shortfall, nothing lasting
    for (Apply ap : {Apply::Half, Apply::Cap1})
    {
        RunSpec part = game;
        part.apply = ap;
        const char* name = ap == Apply::Half ? "half applied" : "capped at 1 deg";
        const RunOut c = Run(rig, part);
        const Stats cs = Measure(c, Order::Game);
        std::printf("%s\n", Describe(name, c, cs).c_str());
        float maxShort = 0;
        for (const Rec& x : c.r)
            maxShort = std::fmax(maxShort, std::fabs(x.cmdStep - x.applied));
        Check(c.ok && cs.commanded >= 20 && maxShort >= 0.5f, std::string("fixture: ") + name + " leaves a shortfall", std::to_string(maxShort));
        Check(cs.aheadMismatch == 0, std::string(name) + ": the derived turn is the command step");
        Check(cs.maxShortfallExcess <= 1e-3f, std::string(name) + ": the drawn heading misses only this feed's shortfall");
        Check(cs.maxIntendedDev <= 0.05f, std::string(name) + ": the intended heading still advances by the clip's turn");
        Check(cs.maxEndGap <= 1e-3f, std::string(name) + ": no lasting heading error once the turn is done");
    }
    motionhost::SetMotionHooks(nullptr);
    motionhost::OnDisarmed();
    std::printf("BodyTurnAheadHost: %d PASS %d FAIL\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
