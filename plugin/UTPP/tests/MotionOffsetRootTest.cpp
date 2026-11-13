#include "../src/mm/MotionOffsetRoot.hpp"
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
using namespace utpp::mm;
static int pass = 0, fail = 0;
static void C(const char *n, bool v)
{
    std::printf("%s %s\n", v ? "PASS" : "FAIL", n);
    v ? ++pass : ++fail;
}
static bool Near(float a, float b, float e = 2e-5f)
{
    return std::isfinite(a) && std::fabs(a - b) < e;
}
struct XY
{
    double x, y;
};
static XY R(double a, XY v)
{
    return {std::cos(a) * v.x - std::sin(a) * v.y, std::sin(a) * v.x + std::cos(a) * v.y};
}
int main()
{
    using motionRoot::OffsetRoot;
    using motionRoot::Step;
    OffsetRoot root;
    float actor = 0, visual = 0;
    for (int i = 0; i < 30; ++i)
    {
        Step s;
        s.dt = 1.f / 60;
        s.movingGround = true;
        s.capsuleYawDelta = math::kPi / 10;
        actor += s.capsuleYawDelta;
        C("18deg capsule turn accepted", root.Update(s));
        const float actual = actor + root.Output().yaw;
        C("zero authored turn keeps world visual yaw", Near(std::sin(actual), 0) && Near(std::cos(actual), 1));
    }
    root.Reset();
    actor = 0;
    visual = 0;
    for (int i = 0; i < 40; ++i)
    {
        Step s;
        s.dt = .017f;
        s.movingGround = true;
        s.capsuleYawDelta = (i % 2 ? .31f : -.22f);
        s.animationYawDelta = .025f;
        actor += s.capsuleYawDelta;
        visual += s.animationYawDelta;
        root.Update(s);
        C("world visual turn contains only authored turn",
          Near(std::sin(actor + root.Output().yaw), std::sin(visual), 5e-5f));
    }
    const float beforeYaw = std::fabs(root.Output().yaw);
    Step release;
    release.dt = .02f;
    release.montage = true;
    release.capsuleYawDelta = .4f;
    release.animationYawDelta = 2;
    root.Update(release);
    C("montage release ignores authored root rotation", std::fabs(root.Output().yaw) < beforeYaw);
    for (int i = 0; i < 150; ++i)
        root.Update(release);
    C("release returns yaw offset to zero", std::fabs(root.Output().yaw) < 1e-6f);
    root.Reset();
    Step s;
    s.dt = .02f;
    s.movingGround = true;
    s.capsuleDeltaCurrent = {.12f, 0, 0};
    root.Update(s);
    C("zero anim moving correction cap is zero", root.Output().correction == 0 && Near(root.Output().x, -.12f));
    s.capsuleDeltaCurrent = {.02f, 0, 0};
    s.animationDeltaVisual = {.001f, 0, 0};
    root.Update(s);
    C("moving correction limited to half extracted animation delta",
      root.Output().correction <= .0005001f && Near(root.Output().maxCorrection, .0005f));
    for (int i = 0; i < 30; ++i)
    {
        s.capsuleDeltaCurrent = {.1f, .05f, 0};
        s.animationDeltaVisual = {};
        root.Update(s);
    }
    C("translation error radius bounded at 30cm", Near(std::hypot(root.Output().x, root.Output().y), .3f));
    // Idle Release follows component, consumes no new animated translation, then recovers.
    auto state = root.Output();
    s.movingGround = false;
    s.capsuleDeltaCurrent = {10, -3, 0};
    s.animationDeltaVisual = {99, 17, 0};
    s.capsuleYawDelta = 0;
    root.Update(s);
    const float decay = 1 - OffsetRoot::DampingAlpha(.1f, s.dt);
    C("stop discards new animation and component mismatch",
      Near(root.Output().x, state.x * decay) && Near(root.Output().y, state.y * decay));
    state = root.Output();
    Step zero = s;
    zero.dt = 0;
    zero.capsuleYawDelta = 3;
    root.Update(zero);
    C("dt0 is exact no-op", std::memcmp(&state, &root.Output(), sizeof state) == 0);
    s.dt = std::numeric_limits<float>::quiet_NaN();
    C("nonfinite dt rejected", !root.Update(s));
    s.dt = .02f;
    s.animationYawDelta = std::numeric_limits<float>::infinity();
    C("nonfinite animation rejected", !root.Update(s));
    // Independent world-space oracle: sum world anim delta, component displacement,
    // first-order analytic exponential correction, radial bound. No local recurrence.
    root.Reset();
    XY component{0, 0}, simulated{0, 0};
    double capYaw = .7, simYaw = .7;
    // Start with a global heading using a zero-motion initial frame (rotation offset0).
    for (int i = 0; i < 70; ++i)
    {
        double turn = i % 3 == 0 ? .18 : -.06;
        XY capD{.024 * std::cos(i * .13), .018 * std::sin(i * .11)}, animD{.004, .019};
        double oldCapYaw = capYaw;
        capYaw += turn;
        XY localCap = R(-capYaw, capD);
        Step f;
        f.dt = .016f;
        f.movingGround = true;
        f.capsuleYawDelta = float(turn);
        f.capsuleDeltaCurrent = {float(localCap.x), float(localCap.y), 0};
        f.animationDeltaVisual = {float(animD.x), float(animD.y), 0};
        f.animationYawDelta = .012f;
        XY aw = R(simYaw, animD);
        simulated.x += aw.x;
        simulated.y += aw.y;
        component.x += capD.x;
        component.y += capD.y;
        simYaw += .012;
        XY correction{(component.x - simulated.x) * (1 - std::exp(-std::log(2.) * .016 / .3)),
                      (component.y - simulated.y) * (1 - std::exp(-std::log(2.) * .016 / .3))};
        double length = std::hypot(correction.x, correction.y), maximum = .5 * std::hypot(animD.x, animD.y);
        if (length > maximum)
        {
            correction.x *= maximum / length;
            correction.y *= maximum / length;
        }
        simulated.x += correction.x;
        simulated.y += correction.y;
        XY off{simulated.x - component.x, simulated.y - component.y};
        length = std::hypot(off.x, off.y);
        if (length > .3)
        {
            simulated.x = component.x + off.x * .3 / length;
            simulated.y = component.y + off.y * .3 / length;
        }
        root.Update(f);
        XY got = R(capYaw, {root.Output().x, root.Output().y});
        C("independent world-position oracle including turns",
          std::hypot(component.x + got.x - simulated.x, component.y + got.y - simulated.y) < .001);
        (void)oldCapYaw;
    }
    // The same physical path under arbitrary global frame rotation must agree.
    OffsetRoot a, b;
    for (int i = 0; i < 40; ++i)
    {
        Step f;
        f.dt = .015f;
        f.movingGround = true;
        f.capsuleYawDelta = .11f;
        f.capsuleDeltaCurrent = {.03f, -.01f, 0};
        f.animationDeltaVisual = {.02f, .005f, 0};
        f.animationYawDelta = .04f;
        a.Update(f);
        b.Update(f);
        XY wa = R(.11 * (i + 1), {a.Output().x, a.Output().y});
        XY wb = R(1.234 + .11 * (i + 1), {b.Output().x, b.Output().y});
        XY expected = R(1.234, wa);
        C("global-frame covariance", std::hypot(wb.x - expected.x, wb.y - expected.y) < 1e-6);
    }
    OffsetRoot seeded;
    C("finite seed accepted", seeded.Seed(.12f, -.08f, .4f));
    C("invalid seed rejected", !seeded.Seed(std::numeric_limits<float>::infinity(), 0, 0));
    const float releaseDecay = 1 - OffsetRoot::DampingAlpha(.1f, .02f);
    XY released = R(-.3, {.12, -.08});
    C("scalar API accepts release", seeded.Update(.02f, 3, -8, .3f, 99, 17, 2, false, true));
    C("Release transports old offset without consuming capsule or anim delta",
      Near(seeded.X(), float(released.x) * releaseDecay) && Near(seeded.Y(), float(released.y) * releaseDecay));
    C("Release rotation follows capsule then damps same relative yaw", Near(seeded.Yaw(), .4f * releaseDecay));
    // Aim-facing recovers the recorded large offset without fighting actor
    // turns. This is separate from montage Release and leaves translation alone.
    for (int hz : {30, 60, 144})
    {
        for (float seedDegrees : {-180.f, -157.f, 157.f, 180.f})
        {
            OffsetRoot aimed;
            const float sign = seedDegrees < 0 ? -1.f : 1.f;
            aimed.Seed(.12f, -.08f, seedDegrees * math::kPi / 180.f);
            const float initialYaw = aimed.Yaw(); // -pi and +pi share the canonical wrap boundary.
            Step aim;
            aim.dt = 1.f / hz;
            aim.aimFacing = true;
            aim.movingGround = true;
            aim.capsuleYawDelta = sign * .07f;
            aim.animationYawDelta = -sign * .35f;
            float previous = initialYaw, maximumStep = 0;
            bool monotonic = true, valid = true;
            for (int i = 0; i < hz; ++i)
            {
                valid = aimed.Update(aim) && valid;
                const float yaw = aimed.Yaw();
                maximumStep = std::fmax(maximumStep, std::fabs(yaw - previous));
                monotonic = monotonic && std::fabs(yaw) <= std::fabs(previous) + 1e-6f && yaw * initialYaw >= 0;
                previous = yaw;
            }
            C("aim-facing large offset converges within 2deg in one second", valid && std::fabs(aimed.Yaw()) < 2 * math::kPi / 180);
            C("aim-facing is monotonic and bounded at 540deg/s", monotonic && maximumStep <= 3 * math::kPi / hz + 1e-5f);
            std::printf("aim-facing pure hz=%d seed=%.1f final=%.3f maxStep=%.3f deg\n", hz,
                        seedDegrees, aimed.Yaw() * 180 / math::kPi, maximumStep * 180 / math::kPi);
            Step leave = aim;
            leave.aimFacing = false;
            leave.animationYawDelta = .04f;
            leave.capsuleYawDelta = .20f;
            const float expected = math::WrapAngle(aimed.Yaw() + .04f - .20f);
            aimed.Update(leave);
            C("aim exit resumes authored/capsule accumulation without reseed", Near(aimed.Yaw(), expected));
        }
    }
    // A stationary aiming character must wait for authored footwork rather
    // than inherit the body's gameplay/capsule turn around planted feet.
    for (int hz : {30, 60, 144}) for (float sign : {-1.f, 1.f}) for (bool montage : {false, true})
    {
        OffsetRoot stationary;
        Step aim;
        aim.dt = 1.f / hz;
        aim.aimFacing = true;
        aim.montage = montage;
        aim.capsuleYawDelta = sign * math::kPi / hz;
        float actorYaw = 0;
        bool planted = true, authored = true;
        for (int i = 0; i < hz / 2; ++i)
        {
            actorYaw += aim.capsuleYawDelta;
            stationary.Update(aim);
            planted &= std::fabs(math::WrapAngle(actorYaw + stationary.Yaw())) < 1e-5f;
        }
        C("stationary aim cancels capsule spin until authored turn begins, including native upper montage", planted);
        aim.capsuleYawDelta = 0;
        aim.animationYawDelta = sign * math::kPi / (2 * hz);
        for (int i = 0; i < hz; ++i)
        {
            const float before = math::WrapAngle(actorYaw + stationary.Yaw());
            stationary.Update(aim);
            const float after = math::WrapAngle(actorYaw + stationary.Yaw());
            authored &= Near(math::WrapAngle(after - before), aim.animationYawDelta);
        }
        C("stationary aim visible turn follows authored yaw in both directions", authored);
        C("stationary turn reaches capsule heading through animation, without yaw release",
          std::fabs(stationary.Yaw()) < 1e-4f && !stationary.Output().rotationRelease);
    }
    for (bool moving : {false, true})
    {
        OffsetRoot aimed, ordinary;
        aimed.Seed(.12f, -.08f, -2.74f);
        ordinary.Seed(.12f, -.08f, -2.74f);
        Step same;
        same.dt = .016f;
        same.movingGround = moving;
        same.capsuleDeltaCurrent = {.04f, -.01f, 0};
        same.animationDeltaVisual = {.02f, .005f, 0};
        same.capsuleYawDelta = .10f;
        same.animationYawDelta = -.04f;
        ordinary.Update(same);
        same.aimFacing = true;
        aimed.Update(same);
        C("aim-facing changes rotation only: translation remains bit-identical",
          aimed.X() == ordinary.X() && aimed.Y() == ordinary.Y() &&
          aimed.Output().correction == ordinary.Output().correction &&
          aimed.Output().maxCorrection == ordinary.Output().maxCorrection &&
          aimed.Output().translationRelease == ordinary.Output().translationRelease);
        const auto snapshot = aimed.Output();
        same.dt = 0;
        same.capsuleYawDelta = 3;
        aimed.Update(same);
        C("aim-facing dt0 remains exact no-op", std::memcmp(&snapshot, &aimed.Output(), sizeof snapshot) == 0);
    }
    // UTPP policy 1: the montage rotation Release is bounded (kReleaseMaxRate = 300 deg/s, the p95 of authored standing turns in the
    // recording). Below ~43 deg it is the motion half-life decay bit for bit; above, a hidden offset no longer spins V (was ~800 deg/s).
    // -DUTPP_ROOT_RELEASE_MUTATION=1 (unbounded) MUST fail the bound checks.
    for (int hz : {30, 60, 144})
    {
        const float dt = 1.f / float(hz), bound = motionRoot::kReleaseMaxRate * dt;
        for (float degrees : {5.f, 20.f, 43.f})
            for (float sign : {-1.f, 1.f})
            {
                OffsetRoot r;
                r.Seed(0, 0, sign * degrees * motionRoot::kDegToRad);
                const float expected = r.Yaw() * (1 - OffsetRoot::DampingAlpha(.1f, dt));
                Step rel;
                rel.dt = dt;
                rel.montage = true;
                rel.capsuleYawDelta = .3f;
                rel.animationYawDelta = 1.f; // ignored by the Release
                r.Update(rel);
                C("bounded release: below the bound it is the Neutral half-life decay, bit-exact", r.Yaw() == expected);
            }
        for (float degrees : {60.f, 90.f, 117.f, 176.f, -176.f})
        {
            OffsetRoot r, referenceRoot;
            r.Seed(0, 0, degrees * motionRoot::kDegToRad);
            referenceRoot.Seed(0, 0, degrees * motionRoot::kDegToRad);
            Step rel;
            rel.dt = dt;
            rel.montage = true;
            Step contract = rel;
            contract.releaseMaxRate = 0; // the unbounded motion contract
            float previous = r.Yaw(), maxStep = 0, time = 0, tReach = 99, motionMax = 0, motionPrev = referenceRoot.Yaw();
            bool monotonic = true, reached = false;
            for (int i = 0; i < 4 * hz; ++i)
            {
                r.Update(rel);
                referenceRoot.Update(contract);
                maxStep = std::fmax(maxStep, std::fabs(r.Yaw() - previous));
                motionMax = std::fmax(motionMax, std::fabs(referenceRoot.Yaw() - motionPrev));
                monotonic = monotonic && std::fabs(r.Yaw()) <= std::fabs(previous) + 1e-7f;
                previous = r.Yaw();
                motionPrev = referenceRoot.Yaw();
                time += dt;
                if (!reached && std::fabs(r.Yaw()) < 2 * motionRoot::kDegToRad)
                    reached = true, tReach = time, std::printf("release %.0f deg %d Hz: bounded peak %.0f deg/s, <2 deg after %.2f s | contract peak %.0f deg/s\n",
                                                degrees, hz, maxStep / dt / motionRoot::kDegToRad, time, motionMax / dt / motionRoot::kDegToRad);
            }
            C("bounded release: never faster than 300 deg/s, monotone, and it does reach the bound for a large offset",
              maxStep <= bound * 1.0001f && maxStep >= bound * 0.98f && monotonic);
            C("bounded release: a large hidden offset is folded in below 1.2 s", reached && tReach < 1.2f);
            C("releaseMaxRate = 0 keeps the unbounded Neutral release (peak above the bound)", motionMax > bound * 1.15f);
        }
        OffsetRoot z;
        z.Seed(0, 0, 2.f);
        Step noTime;
        noTime.dt = 0;
        noTime.montage = true;
        const auto before = z.Output();
        z.Update(noTime);
        C("bounded release: dt0 stays an exact no-op", std::memcmp(&before, &z.Output(), sizeof before) == 0);
    }
    // UTPP policy 2: rest recentering (Step::restRecenter, opt-in) - the FALLBACK of the rest turn-in-place. While V stands, grounded,
    // with nothing owning the facing, an offset beyond the 40 deg knee that no authored turn took for 0.5 s is folded back at <= 120
    // deg/s (soft knee 40..90 deg, 0.25 s ramp up, 0.12 s ramp down). -DUTPP_ROOT_REST_MUTATION=1 (never moves) MUST fail the checks
    // that need it to move.
    {
        const float knee = motionRoot::kRestKneeLow, rate = motionRoot::kRestRate;
        for (int hz : {30, 60, 144})
        {
            const float dt = 1.f / float(hz);
            // off by default: the contract is untouched (absorption only)
            {
                OffsetRoot r;
                r.Seed(0, 0, 2.f);
                Step st;
                st.dt = dt;
                st.capsuleYawDelta = .1f;
                st.animationYawDelta = .02f;
                r.Update(st);
                C("rest recentering is opt-in: default Step only absorbs (yaw + anim - capsule)", Near(r.Yaw(), math::WrapAngle(2.f + .02f - .1f), 1e-6f) &&
                                                                                                     r.Output().restStep == 0);
            }
            for (float degrees : {176.f, -176.f, 117.f, 75.f})
            {
                OffsetRoot r;
                r.Seed(0, 0, degrees * motionRoot::kDegToRad);
                Step st;
                st.dt = dt;
                st.restRecenter = true;
                float previous = r.Yaw(), peak = 0, at05 = 0, at08 = 0, time = 0, lowest = 99, firstMove = -1;
                bool monotone = true, sameSign = true;
                for (int i = 0; i < 6 * hz; ++i)
                {
                    r.Update(st);
                    time += dt;
                    peak = std::fmax(peak, std::fabs(r.Yaw() - previous) / dt);
                    monotone = monotone && std::fabs(r.Yaw()) <= std::fabs(previous) + 1e-7f;
                    sameSign = sameSign && r.Yaw() * degrees > 0;
                    if (firstMove < 0 && r.Output().restStep != 0)
                        firstMove = time;
                    if (time < 0.5f - 0.5f * dt)
                        at05 = std::fabs(r.Yaw() - degrees * motionRoot::kDegToRad);
                    if (time <= 0.8f + 1e-4f)
                        at08 = std::fabs(r.Yaw() - degrees * motionRoot::kDegToRad);
                    lowest = std::fmin(lowest, std::fabs(r.Yaw()));
                    previous = r.Yaw();
                }
                const float finalDeg = std::fabs(r.Yaw()) / motionRoot::kDegToRad;
                std::printf("rest recentering %.0f deg %d Hz: first move %.2f s, peak %.0f deg/s, %.1f deg in 0.8 s, after 6 s %.1f deg\n", degrees, hz,
                            firstMove, peak / motionRoot::kDegToRad, at08 / motionRoot::kDegToRad, finalDeg);
                C("rest recentering: never faster than 120 deg/s, monotone, never crosses zero", peak <= rate * 1.0001f && monotone && sameSign);
                C("rest recentering is only the fallback: nothing moves for the first 0.5 s (the matcher's turn-in-place gets that time)",
                  at05 == 0 && firstMove >= 0.5f - 1e-4f && firstMove <= 0.5f + 2 * dt);
                C("rest recentering: the first 0.3 s after the fallback starts are gentle (<= 28 deg, ramp)", at08 / motionRoot::kDegToRad <= 28.f);
                C("rest recentering: settles at the knee (40..44 deg after 6 s) and never goes below it", finalDeg >= 39.9f && finalDeg <= 44.f && lowest >= knee - 1e-5f);
                if (std::fabs(degrees) > 90.f)
                {
                    OffsetRoot q;
                    q.Seed(0, 0, degrees * motionRoot::kDegToRad);
                    for (int i = 0; i < int(std::lround(3.f * hz)); ++i)
                        q.Update(st);
                    C("rest recentering: a hidden offset of 117..176 deg is below 60 deg after 3 s of waiting for a turn", std::fabs(q.Yaw()) <= 60.f * motionRoot::kDegToRad);
                }
            }
            // nothing below the knee moves
            for (float degrees : {35.f, 40.f, -39.f})
            {
                OffsetRoot r;
                r.Seed(0, 0, degrees * motionRoot::kDegToRad);
                const float before = r.Yaw();
                Step st;
                st.dt = dt;
                st.restRecenter = true;
                for (int i = 0; i < 3 * hz; ++i)
                    r.Update(st);
                C("rest recentering: an offset at or below the 40 deg knee is left alone", r.Yaw() == before);
            }
            // eligibility: any owner of the facing or the motion switches it off
            {
                struct Case { const char *name; bool moving, aim, montage; };
                for (const Case &c : {Case{"moving", true, false, false}, Case{"aim facing", false, true, false}, Case{"montage", false, false, true},
                                     Case{"aim facing in motion", true, true, false}})
                {
                    OffsetRoot r;
                    r.Seed(0, 0, 2.f);
                    Step st;
                    st.dt = dt;
                    st.restRecenter = true;
                    st.movingGround = c.moving;
                    st.aimFacing = c.aim;
                    st.montage = c.montage;
                    float moved = 0;
                    for (int i = 0; i < 3 * hz; ++i)
                    {
                        r.Update(st);
                        moved += std::fabs(r.Output().restStep);
                    }
                    C((std::string("rest recentering: not while ") + c.name).c_str(), moved == 0);
                }
                Step st;
                st.dt = dt;
                st.restRecenter = true;
                OffsetRoot p;
                p.Seed(0, 0, 2.f);
                Step noPermission = st;
                noPermission.restRecenter = false;
                for (int i = 0; i < 3 * hz; ++i)
                    p.Update(noPermission);
                C("rest recentering: the caller's permission (grounded / traversal) gates it", p.Yaw() == math::WrapAngle(2.f) && p.Output().restStep == 0);
                // the wait for the matcher restarts after any ineligible frame
                OffsetRoot r;
                r.Seed(0, 0, 2.f);
                for (int i = 0; i < int(std::lround(0.4f * hz)); ++i)
                    r.Update(st);
                Step walking = st;
                walking.movingGround = true;
                r.Update(walking);
                float t = 0, firstMove = -1;
                for (int i = 0; i < 2 * hz && firstMove < 0; ++i)
                {
                    r.Update(st);
                    t += dt;
                    if (r.Output().restStep != 0)
                        firstMove = t;
                }
                C("rest recentering: an ineligible frame restarts the 0.5 s wait", firstMove >= 0.5f - 1.5f * dt && firstMove <= 0.5f + 2 * dt);
            }
            // an authored turn owns the yaw: nothing moves while it plays, then the 0.25 s hold and the 0.5 s wait
            {
                OffsetRoot r;
                r.Seed(0, 0, 2.f);
                Step st;
                st.dt = dt;
                st.restRecenter = true;
                st.animationYawDelta = 40.f * motionRoot::kDegToRad * dt; // a TIP clip at 40 deg/s
                float moved = 0;
                for (int i = 0; i < 2 * hz; ++i)
                {
                    r.Update(st);
                    moved += std::fabs(r.Output().restStep);
                }
                st.animationYawDelta = 0;
                float firstMove = -1, t = 0;
                for (int i = 0; i < 3 * hz && firstMove < 0; ++i)
                {
                    r.Update(st);
                    t += dt;
                    if (r.Output().restStep != 0)
                        firstMove = t;
                }
                const float expected = motionRoot::kRestAuthoredHold + motionRoot::kRestFallbackDelay;
                C("rest recentering: waits while an authored turn plays, then 0.25 s hold + 0.5 s wait", moved == 0 && firstMove >= expected - 2.f * dt && firstMove <= expected + 3.f * dt);
            }
            // a turn-in-place database the matcher plays owns the yaw (also in a clip's still phases) for 1.6 s without authored progress;
            // then the hold of 0.25 s and the 0.5 s wait. A "turn" that never turns (aim state, stale camera) does not block the fallback.
            for (float playSeconds : {1.4f, 100.f})
            {
                OffsetRoot r;
                r.Seed(0, 0, 2.f);
                Step st;
                st.dt = dt;
                st.restRecenter = true;
                st.turnPlaying = true;
                float moved = 0, firstMove = -1, t = 0;
                const int limit = int(std::lround(std::fmin(playSeconds + 4.f, 8.f) * float(hz)));
                for (int i = 0; i < limit && firstMove < 0; ++i)
                {
                    if (t >= playSeconds)
                        st.turnPlaying = false;
                    r.Update(st);
                    t += dt;
                    if (r.Output().restStep != 0)
                        firstMove = t;
                }
                const float stopsAt = std::fmin(playSeconds, motionRoot::kRestTurnGrace);
                const float expected = stopsAt + motionRoot::kRestAuthoredHold + motionRoot::kRestFallbackDelay;
                (void)moved;
                C(playSeconds < 2.f ? "rest recentering: stays out of the way while a turn database plays (1.4 s), then 0.25 s hold + 0.5 s wait"
                                    : "rest recentering: a turn database that never turns blocks it for 1.6 s only, then 0.25 s hold + 0.5 s wait",
                  firstMove >= expected - 3.f * dt && firstMove <= expected + 3.f * dt);
            }
            // losing the permission slews the speed down, never a one-frame stop (aim / stick in the accumulate branch)
            for (int how = 0; how < 2; ++how)
            {
                OffsetRoot r;
                r.Seed(0, 0, 176.f * motionRoot::kDegToRad);
                Step st;
                st.dt = dt;
                st.restRecenter = true;
                for (int i = 0; i < int(std::lround(0.95f * hz)); ++i) // 0.5 s wait + 0.45 s of turning: full speed, offset still > 90 deg
                    r.Update(st);
                const float fullSpeed = r.Output().restSpeed;
                Step lost = st;
                lost.aimFacing = how == 0;
                lost.movingGround = how == 1;
                float previousSpeed = fullSpeed, worstDrop = 0, still = 0;
                int frames = 0;
                for (int i = 0; i < hz / 2; ++i)
                {
                    r.Update(lost);
                    worstDrop = std::fmax(worstDrop, previousSpeed - r.Output().restSpeed);
                    previousSpeed = r.Output().restSpeed;
                    if (r.Output().restSpeed > 0)
                        ++frames;
                    still += std::fabs(r.Output().restStep);
                }
                const float perFrame = rate / motionRoot::kRestExit * dt;
                std::printf("rest recentering exit (%s) %d Hz: speed %.0f deg/s -> 0 over %d frames (%.2f s), worst drop %.1f deg/s per frame\n",
                            how == 0 ? "aim" : "stick", hz, fullSpeed / motionRoot::kDegToRad, frames, float(frames) * dt, worstDrop / motionRoot::kDegToRad);
                C("rest recentering: losing the permission (aim / stick) slews the speed down, no one-frame stop",
                  fullSpeed > .9f * rate && worstDrop <= perFrame * 1.001f && frames >= int(motionRoot::kRestExit / dt) - 1 && r.Output().restSpeed == 0 && still > 0);
            }
            // the capsule turning does not change what V shows: the visible yaw moves by the recentering step only
            {
                OffsetRoot r;
                r.Seed(0, 0, 100.f * motionRoot::kDegToRad);
                Step st;
                st.dt = dt;
                st.restRecenter = true;
                st.capsuleYawDelta = 90.f * motionRoot::kDegToRad * dt;
                bool exact = true;
                for (int i = 0; i < 2 * hz; ++i)
                {
                    const float before = r.Yaw();
                    r.Update(st);
                    const float visibleDelta = math::WrapAngle(st.capsuleYawDelta + (r.Yaw() - before));
                    exact = exact && Near(visibleDelta, -r.Output().restStep, 2e-5f);
                }
                C("rest recentering: capsule rotation is absorbed, the visible V turns by the recentering step only", exact);
            }
            // dt0 no-op and Seed clears the rest state
            {
                OffsetRoot r;
                r.Seed(0, 0, 2.f);
                Step st;
                st.dt = dt;
                st.restRecenter = true;
                for (int i = 0; i < 2 * hz; ++i)
                    r.Update(st);
                const auto before = r.Output();
                Step zeroStep = st;
                zeroStep.dt = 0;
                r.Update(zeroStep);
                C("rest recentering: dt0 is an exact no-op", std::memcmp(&before, &r.Output(), sizeof before) == 0);
                r.Seed(0, 0, 2.f);
                C("rest recentering: Seed clears the speed, the wait and the authored hold", r.Output().restSpeed == 0 && r.Output().restIdle == 0 && r.Output().restHold == 0);
            }
        }
    }
    std::printf("%d PASS %d FAIL\n", pass, fail);
    return fail ? 1 : 0;
}
