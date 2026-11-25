// Steps fitted to stair treads (mm/StairSteps.hpp, option mm.stairSteps): the tread detector on synthetic flights (17 cm / 28 cm
// and 12 cm / 32 cm, up and down, 30 / 60 / 144 Hz), no estimate on flat ground or a smooth slope, the stride fit (whole treads per
// step, cadence, no fit below the shortest step, hysteresis), the tread phase error, the runtime switch (off = no output) and the
// lowered stride floor of the stride warp (StrideWarp.hpp Floor, the default floor unchanged) and the swing over a flight
// (stairsteps::swing: smooth maximum, swing plan, glide, follow, landing from the clip, nose / tread clearance, pelvis drop).
#include <cmath>
#include <cstdio>
#include <string>

#include "../src/mm/StairSteps.hpp"
#include "../src/mm/StrideWarp.hpp"

using namespace utpp::mm;
using namespace utpp::mm::stairsteps;
static int passed = 0, failed = 0;
static void Check(bool ok, const std::string& name)
{
    std::printf("%s %s\n", ok ? "PASS" : "FAIL", name.c_str());
    ok ? ++passed : ++failed;
}

enum class Ground
{
    Flat,
    Slope,
    Up,
    Down
};

// Height of the ground at distance s (m) along the travel.
static float Height(Ground g, float s, float rise, float tread)
{
    switch (g)
    {
    case Ground::Flat: return 0.0f;
    case Ground::Slope: return s * std::tan(15.0f * 3.14159265f / 180.0f);
    case Ground::Up: return rise * std::floor((s + 0.1f) / tread);
    case Ground::Down: return -rise * std::floor((s + 0.1f) / tread);
    }
    return 0.0f;
}

// Walks a capsule over the ground at speed (m/s) for seconds along the direction (dx, dy), the four traces of a walking gait (the
// ball of each foot ahead / behind the capsule, swapping every half second, the heel 0.2 m behind its ball); returns the last
// estimate and the last feed output.
struct Walk
{
    Estimate est;
    FeedOut out;
    int validFits = 0;
};
static Walk Run(Ground g, float rise, float tread, float hz, float speed, float seconds, float dx = 0.6f, float dy = 0.8f)
{
    Tracker tr;
    Walk w;
    const float dt = 1.0f / hz;
    const int n = int(seconds * hz);
    for (int i = 0; i < n; ++i)
    {
        const float t = float(i) * dt, s = speed * t;
        const float phase = std::fmod(t, 1.0f) < 0.5f ? 1.0f : -1.0f;
        bool hit[2] = {true, true}, heelHit[2] = {true, true};
        float pos[2][3], heel[2][3];
        for (int k = 0; k < 2; ++k)
        {
            const float sb = s + (k == 0 ? 0.3f : -0.1f) * phase, sh = sb - 0.2f;
            pos[k][0] = sb * dx + 5.0f;
            pos[k][1] = sb * dy - 3.0f;
            pos[k][2] = Height(g, sb, rise, tread) + 10.0f;
            heel[k][0] = sh * dx + 5.0f;
            heel[k][1] = sh * dy - 3.0f;
            heel[k][2] = Height(g, sh, rise, tread) + 10.0f;
        }
        tr.AddTraces(hit, pos, heelHit, heel);
        w.out = tr.Update(dt, true, s * dx + 5.0f, s * dy - 3.0f, speed * dx, speed * dy, true);
    }
    w.est = tr.Current();
    w.validFits = int(tr.ValidFits());
    return w;
}

static void Detection()
{
    for (const float hz : {30.0f, 60.0f, 144.0f})
    {
        for (const Ground g : {Ground::Up, Ground::Down})
        {
            for (const auto& st : {std::pair<float, float>{0.17f, 0.28f}, std::pair<float, float>{0.12f, 0.32f}})
            {
                for (const float speed : {1.0f, 3.8f})
                {
                    const Walk w = Run(g, st.first, st.second, hz, speed, 3.0f);
                    char name[256];
                    std::snprintf(name, sizeof name, "detect %s rise %.2f tread %.2f at %.0f Hz %.1f m/s: tread %.3f rise %.3f weight %.2f",
                                  g == Ground::Up ? "up" : "down", double(st.first), double(st.second), double(hz), double(speed),
                                  double(w.est.tread), double(w.est.rise), double(w.out.weight));
                    const float riseSigned = g == Ground::Up ? st.first : -st.first;
                    Check(w.est.valid && std::fabs(w.est.tread - st.second) < 0.01f && std::fabs(w.est.rise - riseSigned) < 0.01f &&
                              w.out.weight == 1.0f && std::fabs(w.out.treadM - st.second) < 0.01f,
                          name);
                    // the nose line of the estimate lies on a real nose (distance from the nearest nose along the travel)
                    const float s0 = (w.est.noseX - 5.0f) * 0.6f + (w.est.noseY + 3.0f) * 0.8f + 0.1f;
                    const float off = s0 - st.second * std::round(s0 / st.second);
                    // within 2 cm; 5 cm when one frame moves the traces more than 5 cm (a run at 30 / 60 Hz: the staircase fit has no
                    // point near some noses)
                    const float tol = speed / hz > 0.05f ? 0.05f : 0.02f;
                    Check(std::fabs(off) < tol, std::string(name) + ": nose line on a nose (" + std::to_string(off) + " m)");
                }
            }
        }
    }
    for (const Ground g : {Ground::Flat, Ground::Slope})
    {
        const Walk w = Run(g, 0.17f, 0.28f, 60.0f, 2.0f, 3.0f);
        Check(!w.est.valid && w.validFits == 0 && w.out.weight == 0.0f && w.out.treadM == 0.0f,
              g == Ground::Flat ? "flat ground: no stairs, weight 0" : "15 deg slope: no stairs, weight 0");
    }
    // leaving the flight onto level ground: the weight falls to 0
    {
        Tracker tr;
        const float dt = 1.0f / 60.0f;
        FeedOut o;
        for (int i = 0; i < 360; ++i)
        {
            const float t = float(i) * dt, s = 1.5f * t;
            bool hit[2] = {true, true}, heelHit[2] = {false, false};
            float pos[2][3] = {{0.0f, s + 0.3f, 0.0f}, {0.0f, s - 0.1f, 0.0f}}, heel[2][3] = {};
            // the traces carry their own heights (the trailing foot may still be on the flight)
            for (int k = 0; k < 2; ++k)
            {
                const float sk = pos[k][1];
                pos[k][2] = sk < 2.8f ? 0.17f * std::floor((sk + 0.1f) / 0.28f) : 0.17f * 10.0f;
            }
            tr.AddTraces(hit, pos, heelHit, heel);
            o = tr.Update(dt, true, 0.0f, s, 0.0f, 1.5f, true);
        }
        Check(o.weight == 0.0f && o.treadM == 0.0f, "after the flight on level ground for 3 s: weight 0, no stairs");
    }
}

static void Fit()
{
    const StrideFit a = FitStride(1.1f, 0.28f, 0.5f);
    Check(a.treads == 2 && std::fabs(a.cadence - 1.1f / 0.56f) < 1e-5f, "walk step 1.1 m on 28 cm treads: 2 treads, cadence 1.96");
    const StrideFit b = FitStride(0.4f, 0.28f, 0.5f);
    Check(b.treads == 1 && std::fabs(b.cadence - 0.4f / 0.28f) < 1e-5f, "step 0.4 m on 28 cm treads: 1 tread, cadence 1.43");
    const StrideFit c = FitStride(0.2f, 0.28f, 0.5f);
    Check(c.treads == 0 && c.cadence == 1.0f, "step shorter than one tread: no fit (a step is never lengthened)");
    const StrideFit d = FitStride(0.56f, 0.28f, 0.5f);
    Check(d.treads == 1 && std::fabs(d.cadence - 2.0f) < 1e-5f, "step 0.56 m, shortest half: 1 tread, cadence 2");
    const StrideFit e = FitStride(1.73f, 0.32f, 0.75f);
    Check(e.treads == 5 && std::fabs(e.cadence - 1.73f / 1.6f) < 1e-5f, "run step 1.73 m, 32 cm treads, shortest 0.75: 5 treads");
    const StrideFit f = FitStride(0.574f, 0.28f, 0.5f, 2); // 0.5 x 0.574 = 0.287 > 0.28: nMin 2 = nMax 2
    Check(f.treads == 2, "one count fits: it is taken");
    const StrideFit g = FitStride(1.15f, 0.28f, 0.5f, 3); // nMin 3 (0.84 >= 0.575), 0.84 < 1.03 x 0.575? no -> 3 stays nMin
    Check(g.treads == 3, "hysteresis: the count is the smallest fitting one when the previous one is not nMin + 1");
    const StrideFit h = FitStride(1.10f, 0.28f, 0.5f, 3); // nMin 2: 0.56 < 1.03 x 0.55 = 0.5665 -> keep 3
    Check(h.treads == 3, "hysteresis: the previous count (nMin + 1) is kept while nMin only just fits");
    Check(FitStride(NAN, 0.28f, 0.5f).treads == 0 && FitStride(1.0f, 0.0f, 0.5f).treads == 0 && FitStride(1.0f, 0.28f, 1.5f).treads == 0,
          "invalid input: no fit");
}

static void Phase()
{
    Estimate e;
    e.valid = true;
    e.tread = 0.3f;
    e.dirX = 1.0f;
    e.dirY = 0.0f;
    e.noseX = 2.0f;
    e.noseY = 7.0f;
    Check(std::fabs(TreadPhaseError(e, 2.15f, 7.0f)) < 1e-5f, "phase error 0 on the tread centre");
    Check(std::fabs(TreadPhaseError(e, 2.15f + 0.9f, 3.0f)) < 1e-5f, "phase error 0 three treads further (across the travel ignored)");
    Check(std::fabs(TreadPhaseError(e, 2.20f, 7.0f) - 0.05f) < 1e-5f, "5 cm past the centre: +0.05");
    Check(std::fabs(TreadPhaseError(e, 2.08f, 7.0f) + 0.07f) < 1e-5f, "7 cm before the centre: -0.07");
    Check(std::fabs(std::fabs(TreadPhaseError(e, 2.0f, 7.0f)) - 0.15f) < 1e-5f, "on a nose: half a tread");
    Estimate none;
    Check(TreadPhaseError(none, 1.0f, 1.0f) == 0.0f, "no estimate: 0");
}

static void Runtime()
{
    Check(!rt::Enabled(), "default: off at startup (the menu default mm.stairSteps)");
    bool hit[2] = {true, true};
    float pos[2][3] = {{0, 0, 0}, {0, 0.1f, 0.17f}};
    for (int i = 0; i < 300; ++i)
        rt::AddTraces(hit, pos, hit, pos);
    const FeedOut o = rt::Update(1.0f / 60.0f, true, 0, 0, 0, 2.0f, true);
    Check(o.treadM == 0.0f && o.weight == 0.0f && rt::g_tracker.Fits() == 0, "off: no point stored, no fit, no stairs");
    rt::SetEnabled(true);
    Check(rt::Enabled(), "switched on");
    rt::SetEnabled(false);
}

static void Floor()
{
    stridewarp::Inputs in;
    Check(stridewarp::Floor(in) == stridewarp::kScaleFloor, "stride floor: default = kScaleFloor (unchanged off the stairs)");
    in.floor = 0.5f;
    Check(stridewarp::Floor(in) == 0.5f, "stride floor: a stair floor below it is taken");
    in.floor = 0.1f;
    Check(stridewarp::Floor(in) == 0.3f, "stride floor: never below 0.3");
    in.floor = 0.9f;
    Check(stridewarp::Floor(in) == stridewarp::kScaleFloor, "stride floor: never above kScaleFloor");
    in.floor = NAN;
    Check(stridewarp::Floor(in) == stridewarp::kScaleFloor, "stride floor: NaN = kScaleFloor");
    // a stair floor lets the clamp reach the shortened stride: capsule 0.8 m/s, root 2.0 m/s -> 0.4
    stridewarp::Inputs s;
    s.allowed = true;
    s.capsuleSpeed = 0.8f;
    s.rootSpeed = 2.0f;
    Check(stridewarp::TargetScale(s) == stridewarp::kScaleFloor, "target scale clamped at the default floor");
    s.floor = 0.35f;
    Check(std::fabs(stridewarp::TargetScale(s) - 0.4f) < 1e-6f, "target scale 0.4 with a stair floor of 0.35");
}

static Flight MakeFlight(float rise, float tread, int treads)
{
    Flight f;
    f.valid = 1;
    f.weight = 1.0f;
    f.tread = tread;
    f.rise = rise;
    f.dirX = 1.0f;
    f.dirY = 0.0f;
    f.treads = treads;
    return f;
}

static void Swing()
{
    namespace sw = swing;
    const sw::Config c{};
    // smooth maximum: the larger one apart, no kink at equality, never below either
    Check(std::fabs(sw::SmoothMax(1.0f, 0.0f, 0.015f) - 1.0f) < 1e-3f, "smooth max: the larger height when far apart");
    Check(std::fabs(sw::SmoothMax(0.3f, 0.3f, 0.015f) - 0.3075f) < 1e-5f, "smooth max: equal heights raised by half the width");
    Check(sw::SmoothMax(0.30f, 0.29f, 0.015f) >= 0.30f, "smooth max: never below the larger");
    // plan with a standing support: the landing is the fit's treads past the support, on the tread above / below it
    const Flight up = MakeFlight(0.17f, 0.28f, 2);
    sw::Plan p;
    Check(sw::PlanSwing(c, up, -0.14f, 1.0f, 0.14f, 1.17f, true, p), "plan: support foot standing");
    Check(std::fabs(p.sLand - (0.14f + 2.0f * 0.28f)) < 1e-5f && std::fabs(p.h1 - (1.17f + 2.0f * 0.17f)) < 1e-5f && p.anchored,
          "plan: landing two treads past the support, two rises higher, anchored");
    // no support (the flight phase of a run): two steps past the tread the foot left, not anchored
    Check(sw::PlanSwing(c, up, 0.14f, 1.0f, NAN, NAN, false, p), "plan: flight phase");
    Check(std::fabs(p.sLand - (0.14f + 4.0f * 0.28f)) < 1e-5f && std::fabs(p.h1 - (1.0f + 4.0f * 0.17f)) < 1e-5f && !p.anchored,
          "plan: flight phase lands two steps on");
    Check(!sw::PlanSwing(c, MakeFlight(0.17f, 0.28f, 0), 0.14f, 1.0f, NAN, NAN, false, p), "plan: no fit in the flight phase = no plan");
    // no fit with a support: as far ahead of the support as the foot is behind it
    Check(sw::PlanSwing(c, MakeFlight(-0.17f, 0.28f, 0), -0.14f, 1.17f, 0.42f, 1.0f, true, p) && std::fabs(p.sLand - 0.98f) < 1e-5f &&
              !p.anchored,
          "plan: no fit mirrors the step about the support");
    Check(!sw::PlanSwing(c, MakeFlight(0.17f, 0.28f, 0), 0.10f, 1.0f, 0.12f, 1.0f, true, p), "plan: a swing shorter than minSwing = none");
    // a measured step: one step past the support, on the tread that many whole treads on
    Check(sw::PlanSwing(c, up, -0.14f, 1.0f, 0.14f, 1.17f, true, p, NAN, 0.84f) && std::fabs(p.sLand - 0.98f) < 1e-5f &&
              p.kLand == 3.0f && std::fabs(p.h1 - (1.17f + 3.0f * 0.17f)) < 1e-5f,
          "plan: a measured step of three treads lands three treads past the support");
    // the flight phase with the foot's own last landing: two steps past that landing, not past the lift-off
    Check(sw::PlanSwing(c, up, 0.10f, 1.0f, NAN, NAN, false, p, NAN, 0.56f, 0.0f) && std::fabs(p.sLand - 1.12f) < 1e-5f &&
              p.kLand == 4.0f && std::fabs(p.h1 - (1.0f + 4.0f * 0.17f)) < 1e-5f && !p.anchored,
          "plan: flight phase lands two steps past the foot's own last landing");
    // anchor: the other foot landed -> one step past it, anchored; not once half the swing is gone
    sw::Plan a;
    Check(sw::PlanSwing(c, up, 0.14f, 1.0f, NAN, NAN, false, a) &&
              sw::Anchor(c, up, a, 0.42f, 1.17f, 0.56f) && std::fabs(a.sLand - 0.98f) < 1e-5f && a.kLand == 3.0f &&
              std::fabs(a.h1 - (1.17f + 2.0f * 0.17f)) < 1e-5f && a.anchored,
          "anchor: one step past the landed foot, on its tread plus the step's treads");
    sw::Plan a2;
    sw::PlanSwing(c, up, 0.14f, 1.0f, NAN, NAN, false, a2);
    a2.u = 0.6f;
    Check(!sw::Anchor(c, up, a2, 0.42f, 1.17f, 0.56f) && !a2.anchored, "anchor: none past half the swing");
    // converge: late in the swing the own trace over the landing spot becomes the landing height
    sw::Plan cv;
    sw::PlanSwing(c, up, -0.14f, 1.0f, 0.14f, 1.17f, true, cv); // lands at 0.70, 1.51
    cv.u = 0.5f;
    Check(!sw::Converge(c, up, cv, 0.65f, 1.40f), "converge: not before convergeFrom");
    cv.u = 0.7f;
    Check(!sw::Converge(c, up, cv, 0.40f, 1.40f), "converge: not a trace well short of the landing");
    Check(sw::Converge(c, up, cv, 0.65f, 1.40f) && cv.h1 == 1.40f, "converge: the trace over the landing spot is the landing height");
    // glide: reaches the landing tread at the landing, linear going up, front-loaded going down, no jump when the plan moves
    sw::Plan g;
    g.h0 = 0.0f;
    g.h1 = g.h1s = -0.34f;
    g.s0 = 0.0f;
    g.sLand = 1.0f;
    float path = 0.0f, prev = 0.0f;
    bool mono = true;
    float half = 0.0f;
    for (int i = 1; i <= 100; ++i)
    {
        const float s = 0.01f * float(i);
        const float n = sw::PathStep(g, path, prev, s, true);
        mono = mono && n <= path + 1e-6f;
        path = n;
        prev = s;
        if (i == 50)
            half = path;
    }
    Check(mono && std::fabs(path - g.h1) < 1e-4f, "glide down: monotone, on the landing tread at the landing");
    Check(std::fabs(half - g.h1 * 0.75f) < 2e-3f, "glide down: front-loaded (3/4 of the drop at half the way)");
    path = 0.0f;
    prev = 0.0f;
    g.h1 = g.h1s = 0.34f;
    for (int i = 1; i <= 50; ++i)
    {
        const float s = 0.01f * float(i);
        path = sw::PathStep(g, path, prev, s, false);
        prev = s;
    }
    Check(std::fabs(path - 0.17f) < 1e-3f, "glide up: linear (half the rise at half the way)");
    g.sLand = 1.28f; // the landing moved one tread on: the glide goes on from where it is
    const float before = path;
    path = sw::PathStep(g, path, prev, prev + 0.01f, false);
    Check(path >= before && path - before < 0.02f, "glide: a moved landing does not jump the height");
    // landing from the clip: a sinking ball moves a free plan's landing, an anchored plan stays
    sw::Plan r;
    Check(sw::PlanSwing(c, up, 0.14f, 1.0f, NAN, NAN, false, r), "retarget: plan");
    // mid-sole at 0.40 (progress 0.23), 4 m/s, 0.1 s to the ground: lands near 0.80, 4 cm short of tread 3 -> tread 3 (the planned
    // side of the boundary, within the hysteresis), 3 rises above the tread left
    Check(sw::Retarget(c, up, r, 0.40f, 4.0f, 0.10f, -1.0f) && std::fabs(r.sLand - 0.80f) < 1e-5f && r.kLand == 3.0f &&
              std::fabs(r.h1 - (1.0f + 0.17f * 3.0f)) < 1e-5f,
          "retarget: landing 0.1 s ahead, the tread on the planned side within the hysteresis");
    Check(sw::Retarget(c, up, r, 0.30f, 4.0f, 0.10f, -1.0f) && r.kLand == 2.0f && std::fabs(r.h1 - (1.0f + 0.17f * 2.0f)) < 1e-5f,
          "retarget: well inside a tread takes it");
    Check(!sw::Retarget(c, up, r, 0.40f, 4.0f, 0.10f, 0.5f), "retarget: a rising ball = no prediction");
    sw::Config early = c;
    early.retargetSinking = false; // only early in the swing of an unanchored plan
    sw::Plan late;
    Check(sw::PlanSwing(c, up, 0.14f, 1.0f, NAN, NAN, false, late), "retarget late: plan");
    late.u = 0.6f;
    Check(!sw::Retarget(early, up, late, 0.90f, 4.0f, 0.10f, -1.0f) && late.kLand == 4.0f, "retarget (early only): none past half the swing");
    Check(sw::PlanSwing(c, up, -0.14f, 1.0f, 0.14f, 1.17f, true, r) && !sw::Retarget(early, up, r, 0.30f, 4.0f, 0.1f, -1.0f),
          "retarget (early only): an anchored plan stays");
    // sinking (retargetSinking): the landing follows the clip at any progress, anchored too
    sw::Config sink = c;
    sink.retargetSinking = true;
    Check(sw::Retarget(sink, up, late, 0.90f, 4.0f, 0.10f, -1.0f) && late.kLand == 4.0f && std::fabs(late.sLand - 1.30f) < 1e-5f,
          "retarget: a sinking ball late in the swing moves the landing (the tread kept within the hysteresis)");
    Check(sw::PlanSwing(c, up, -0.14f, 1.0f, 0.14f, 1.17f, true, r) && sw::Retarget(sink, up, r, 0.30f, 4.0f, 0.1f, -1.0f) &&
              r.kLand == 2.0f,
          "retarget: a sinking ball moves an anchored plan, the tread on the planned side within the hysteresis");
    Check(!sw::Retarget(sink, up, r, 0.30f, 4.0f, 0.1f, 0.2f), "retarget: a ball not sinking leaves an anchored plan");
    // follow: a higher target at once, a lower one smoothly and never below it, stable for a long step
    float h = 0.0f, v = 0.0f;
    sw::Follow(h, v, 0.1f, c.follow, 1.0f / 60.0f);
    Check(h == 0.1f, "follow: a higher ground at once");
    h = 0.17f;
    v = 0.0f;
    sw::Follow(h, v, 0.0f, c.follow, 1.0f / 60.0f);
    Check(h > 0.0f && h < 0.17f, "follow: a lower ground smoothly");
    bool above = true;
    for (int i = 0; i < 200; ++i)
    {
        sw::Follow(h, v, 0.0f, c.follow, 1.0f / 30.0f);
        above = above && h >= 0.0f;
    }
    Check(above && h < 1e-3f, "follow: settles on the lower ground, never below it");
    h = 0.17f;
    v = 0.0f;
    sw::Follow(h, v, 0.0f, c.follow, 0.25f);
    Check(std::isfinite(h) && h >= 0.0f && h < 0.17f, "follow: stable for a long step");
    // clearance: the toe over the nose line going up, the heel over the tread it leaves going down
    sw::Plan q;
    Check(sw::PlanSwing(c, up, 0.14f, 0.0f, NAN, NAN, false, q), "clearance: plan");
    const float sToe = 0.27f; // toe just short of the first nose (0.28), clip toe on the floor, half way
    const float gUp = sw::SwingGround(c, up, q, 0.0f, 0.5f, sToe, 0.0f, 0.0f, 0.0f, 0.10f, 0.0f);
    Check(gUp + 1e-4f >= sw::NoseLineUp(up, sToe, q.hs, q.ks), "clearance up: toe not below the nose line");
    const Flight down = MakeFlight(-0.17f, 0.28f, 2);
    sw::Plan d;
    Check(sw::PlanSwing(c, down, 0.14f, 0.0f, NAN, NAN, false, d), "clearance down: plan");
    const float gDown = sw::SwingGround(c, down, d, -0.30f, 0.4f, 0.40f, 0.05f, 0.20f, 0.0f, 0.33f, 0.05f);
    Check(gDown >= -1e-3f, "clearance down: the heel over the tread it leaves holds the foot");
    const float gPast = sw::SwingGround(c, down, d, -0.30f, 0.4f, 0.75f, 0.05f, 0.55f, 0.0f, 0.68f, 0.05f);
    Check(gPast < -0.15f, "clearance down: past the nose the foot glides down");
    // pelvis drop: only going down, more with longer steps, weighted and capped
    Check(sw::DescentDrop(up, 0.3f, 0.15f, 0.25f) == 0.0f, "descent drop: none going up");
    const float d1 = sw::DescentDrop(MakeFlight(-0.17f, 0.28f, 1), 0.3f, 0.15f, 0.25f);
    const float d4 = sw::DescentDrop(MakeFlight(-0.17f, 0.28f, 4), 0.3f, 0.15f, 0.25f);
    Check(d1 < 0.0f && d4 < d1 && std::fabs(d4 + 0.17f * 0.9f) < 1e-5f, "descent drop: more with more treads per step");
    Flight half2 = MakeFlight(-0.17f, 0.28f, 4);
    half2.weight = 0.5f;
    Check(std::fabs(sw::DescentDrop(half2, 0.3f, 0.15f, 0.25f) - 0.5f * d4) < 1e-5f, "descent drop: weighted");
    Check(sw::DescentDrop(MakeFlight(-0.40f, 0.28f, 4), 0.3f, 0.15f, 0.25f) == -0.25f, "descent drop: capped");
}

int main()
{
    Swing();
    Runtime();
    Detection();
    Fit();
    Phase();
    Floor();
    std::printf("stair steps: %d passed, %d failed\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
