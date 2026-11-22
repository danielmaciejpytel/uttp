// Body height smoothing (mm/BodyHeightFilter.hpp, mm/BodyHeightRuntime.hpp): the tracker (zero lag on a constant slope, a ground snap
// absorbed, offset bounds, hitch guard, restart on a large step, gate fade, the same response at 30 / 60 / 144 Hz) and the pose
// application on the real rig topology (pelvis moved along model z, both ankles and foot rotations kept, zero offset / another topology
// untouched), plus the animation-thread stepper (one update per frame, restart on a serial bump, melee lower-body frames untouched).
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "../src/BaseDatabase.hpp"
#include "../src/mm/BodyHeightFilter.hpp"
#include "../src/mm/BodyHeightRuntime.hpp"

using namespace utpp::mm;
using namespace utpp::mm::bodyheight;
static int passed = 0, failed = 0;
static void Check(bool ok, const char* name, int hz = 0)
{
    std::printf("%s %s%s%s\n", ok ? "PASS" : "FAIL", name, hz ? " @Hz=" : "", hz ? std::to_string(hz).c_str() : "");
    ok ? ++passed : ++failed;
}

// drawn height series: ground z(t) plus the filter offset; returns the largest per-frame second difference of the drawn height
// over [aFrom, aTo) seconds, and the raw ground's in aRaw
template <class Ground>
static float Run(int hz, Ground&& ground, float aFrom, float aTo, float& aRaw, Filter* aOut = nullptr, float aSeconds = 2.0f)
{
    Filter f;
    const float dt = 1.0f / float(hz);
    const int n = int(aSeconds * float(hz));
    std::vector<float> drawn(static_cast<size_t>(n), 0.0f), raw(static_cast<size_t>(n), 0.0f);
    for (int i = 0; i < n; ++i)
    {
        const float t = float(i) * dt, z = ground(t);
        const float o = f.Step(i ? dt : 0.0f, z, true);
        drawn[size_t(i)] = z + o;
        raw[size_t(i)] = z;
    }
    float worst = 0.0f;
    aRaw = 0.0f;
    for (int i = 1; i + 1 < n; ++i)
    {
        const float t = float(i) * dt;
        if (t < aFrom || t >= aTo)
            continue;
        worst = std::fmax(worst, std::fabs(drawn[size_t(i + 1)] - 2.0f * drawn[size_t(i)] + drawn[size_t(i - 1)]));
        aRaw = std::fmax(aRaw, std::fabs(raw[size_t(i + 1)] - 2.0f * raw[size_t(i)] + raw[size_t(i - 1)]));
    }
    if (aOut)
        *aOut = f;
    return worst;
}

static void Tracker(int hz)
{
    float raw = 0.0f;
    Filter f;
    // constant slope (0.12 m/s up a ramp): zero lag once the velocity estimate settled
    Run(hz, [](float t) { return 0.12f * t; }, 0.0f, 2.0f, raw, &f);
    Check(std::fabs(f.Offset()) < 1e-4f, "slope: constant ramp passes with zero lag", hz);
    // ground snap: the capsule rises 2.5 cm in one frame at t = 1 s and stays there; the drawn height takes it over several frames
    const float snap = Run(hz, [](float t) { return t < 1.0f ? 0.0f : 0.025f; }, 0.9f, 1.5f, raw);
    std::printf("  snap 2.5 cm @%d Hz: largest second difference drawn %.2f cm, raw %.2f cm\n", hz, snap * 100.0f, raw * 100.0f);
    // the time constants are in seconds: at 30 Hz a frame is 0.67 posTau, so less of a one-frame snap can be spread
    Check(snap < (hz < 60 ? 0.75f : 0.5f) * raw, "snap: second difference of the drawn height well below the raw one", hz);
    Filter g;
    Run(hz, [](float t) { return t < 1.0f ? 0.0f : -0.025f; }, 0.0f, 2.0f, raw, &g, 1.0f + 1.0f / float(hz) + 1e-4f);
    Check(g.Offset() > 0.0f && g.Offset() <= 0.005f + 1e-6f, "snap down: the body stays above the capsule by at most +0.5 cm", hz);
    // a snap up: the body is drawn below the capsule, bounded at -4 cm
    Filter u;
    Run(hz, [](float t) { return t < 1.0f ? 0.0f : 0.20f; }, 0.0f, 2.0f, raw, &u, 1.0f + 1.0f / float(hz) + 1e-4f);
    Check(u.Offset() < -0.03f && u.Offset() >= -0.04f - 1e-6f && u.Diagnostics().clamps > 0, "snap up 20 cm: offset clamped at -4 cm", hz);
    // after the snap the offset returns to zero
    Filter v;
    Run(hz, [](float t) { return t < 1.0f ? 0.0f : 0.20f; }, 0.0f, 2.0f, raw, &v, 2.0f);
    Check(std::fabs(v.Offset()) < 1e-3f, "snap up: offset back to zero within 1 s", hz);
    // a step above 0.5 m restarts the tracker (offset 0 at once)
    Filter r;
    Run(hz, [](float t) { return t < 1.0f ? 0.0f : 0.8f; }, 0.0f, 2.0f, raw, &r, 1.0f + 1.0f / float(hz) + 1e-4f);
    Check(r.Offset() == 0.0f && r.Diagnostics().resets == 1, "teleport step 0.8 m: restart, offset 0", hz);
}

// a lift: V stands while the floor moves at 2 / 4 m/s up or down for 5 s (a smooth 0.5 s start and stop): the offset stays a short
// transient at the start and the stop (under 1 s at a bound), it is not held at a bound for the ride, and it is back to zero 2 s after the stop
static void Lift(int hz)
{
    for (float speed : {2.0f, -2.0f, 4.0f, -4.0f})
    {
        Filter f;
        const float dt = 1.0f / float(hz);
        const float ramp = 0.5f, ride = 5.0f, total = 9.0f; // 2 s after the stop
        auto vel = [&](float t) {
            if (t < 1.0f) return 0.0f;
            const float u = t - 1.0f;
            if (u < ramp) return speed * u / ramp;
            if (u < ramp + ride) return speed;
            if (u < 2.0f * ramp + ride) return speed * (2.0f * ramp + ride - u) / ramp;
            return 0.0f;
        };
        float z = 0.0f, worstRide = 0.0f, worstAll = 0.0f, boundTime = 0.0f, boundRun = 0.0f;
        for (int i = 0; i < int(total * float(hz)); ++i)
        {
            const float t = float(i) * dt;
            z += vel(t) * dt;
            const float o = f.Step(i ? dt : 0.0f, z, true);
            worstAll = std::fmax(worstAll, std::fabs(o));
            if (t > 1.0f + ramp + 0.3f && t < 1.0f + ramp + ride)
                worstRide = std::fmax(worstRide, std::fabs(o));
            const bool atBound = o <= -0.04f + 1e-5f || o >= 0.005f - 1e-6f;
            boundRun = atBound ? boundRun + dt : 0.0f;
            boundTime = std::fmax(boundTime, boundRun);
        }
        std::printf("  lift %.0f m/s @%d Hz: offset max %.2f cm, during the ride %.3f cm, longest at a bound %.3f s, end %.4f cm\n", speed, hz,
                    worstAll * 100.0f, worstRide * 100.0f, boundTime, f.Offset() * 100.0f);
        Check(worstRide < 0.002f && boundTime < 1.0f && std::fabs(f.Offset()) < 1e-3f,
              speed > 0 ? "lift up: offset only a short start / stop transient, back to zero" : "lift down: offset only a short start / stop transient, back to zero", hz);
    }
}

static void RateIndependence()
{
    // the drawn height 0.1 s after a 2 cm snap up does not depend on the frame rate (closed-form exponentials)
    float at[3] = {};
    const int rates[3] = {30, 60, 144};
    for (int k = 0; k < 3; ++k)
    {
        Filter f;
        const float dt = 1.0f / float(rates[k]);
        float o = 0.0f;
        for (int i = 0;; ++i)
        {
            const float t = float(i) * dt;
            if (t > 1.1f + 1e-4f)
                break;
            o = f.Step(i ? dt : 0.0f, t < 1.0f - 1e-4f ? 0.0f : 0.02f, true);
        }
        at[k] = o;
    }
    Check(std::fabs(at[0] - at[2]) < 0.003f && std::fabs(at[1] - at[2]) < 0.002f, "rates: 30 / 60 / 144 Hz give the same offset within 3 mm");
}

static void HitchGuardAndGate()
{
    Filter f;
    float z = 0.0f;
    for (int i = 0; i < 120; ++i)
        f.Step(i ? 1.0f / 60.0f : 0.0f, z, true);
    const uint64_t before = f.Diagnostics().hitches;
    // a 50 ms frame (3 x the median) carries a catch-up step of the capsule: no velocity update from it
    z = 0.01f;
    f.Step(0.05f, z, true);
    Check(f.Diagnostics().hitches == before + 1, "hitch: a 3x frame skips the velocity update");
    for (int i = 0; i < 120; ++i)
        f.Step(1.0f / 60.0f, z, true);
    Check(std::fabs(f.Offset()) < 1e-4f, "hitch: offset settles afterwards");
    // gate: a snap while not allowed gives no offset after the fade; allowed again fades the stage back in
    Filter g;
    for (int i = 0; i < 60; ++i)
        g.Step(i ? 1.0f / 60.0f : 0.0f, 0.0f, true);
    float o = 0.0f;
    for (int i = 0; i < 60; ++i)
        o = g.Step(1.0f / 60.0f, i < 30 ? 0.0f : 0.03f, false);
    Check(o == 0.0f && g.Fade() == 0.0f, "gate closed: offset faded to zero");
    o = g.Step(1.0f / 60.0f, 0.06f, true);
    Check(o < 0.0f && o > -0.03f, "gate opened: the offset fades in (no full jump in the first frame)");
    // non-positive dt: no state change
    const float keep = g.Offset();
    Check(g.Step(0.0f, 0.5f, true) == keep, "dt 0: last offset, no update");
    Check(g.Step(1.0f / 60.0f, std::nanf(""), true) == 0.0f && !g.Started(), "non-finite height: restart, offset 0");
}

// mock topology for the pose application without a database file
struct MockDb
{
    std::vector<std::string> names{"Root", "Hips", "LeftUpLeg", "LeftLeg", "LeftFoot", "LeftToeBase", "RightUpLeg", "RightLeg",
                                   "RightFoot", "RightToeBase", "Spine"};
    std::vector<int32_t> parents{-1, 0, 1, 2, 3, 4, 1, 6, 7, 8, 1};
    bool Loaded() const { return true; }
    uint32_t NumBones() const { return uint32_t(names.size()); }
    int32_t BoneParent(uint32_t b) const { return parents[b]; }
    std::string_view BoneName(uint32_t b) const { return names[b]; }
};

static BoneTransform Model(const LegRig& rig, const float* pose, const std::vector<int32_t>& parents, int32_t b)
{
    std::vector<int32_t> chain;
    for (int32_t k = b; k >= 0; k = parents[size_t(k)])
        chain.push_back(k);
    BoneTransform g{};
    for (auto it = chain.rbegin(); it != chain.rend(); ++it)
        g = bodyheight::detail::Mul(g, bodyheight::detail::Load(pose, *it));
    (void)rig;
    return g;
}

// angle between two rotations (rad), in double from normalised quaternions (a float acos near 1 reads 0.05 deg of rounding noise)
static float QuatDist(math::Quat a, math::Quat b)
{
    const double na = std::sqrt(double(a.x) * a.x + double(a.y) * a.y + double(a.z) * a.z + double(a.w) * a.w);
    const double nb = std::sqrt(double(b.x) * b.x + double(b.y) * b.y + double(b.z) * b.z + double(b.w) * b.w);
    const double d = std::fabs(double(a.x) * b.x + double(a.y) * b.y + double(a.z) * b.z + double(a.w) * b.w) / (na * nb);
    return float(2.0 * std::acos(std::fmin(1.0, d)));
}

template <class Db>
static void PoseCheck(const Db& db, const char* label, const std::vector<std::vector<float>>& poses)
{
    LegRig rig;
    const bool ok = rig.Init(db);
    Check(ok && rig.pelvis >= 0, (std::string(label) + ": leg rig found").c_str());
    if (!ok)
        return;
    std::vector<int32_t> parents(db.NumBones());
    for (uint32_t b = 0; b < db.NumBones(); ++b)
        parents[b] = db.BoneParent(b);
    float worstAnkle = 0.0f, worstAnkleUp = 0.0f, worstRot = 0.0f, worstPelvis = 0.0f, worstSpine = 0.0f;
    int solved = 0, clamped = 0, raiseLimited = 0;
    bool raiseBelow = false, lowerCut = false;
    bool zeroUntouched = true, otherUntouched = true;
    for (const auto& src : poses)
    {
        for (float offset : {-0.04f, -0.02f, 0.005f})
        {
            std::vector<float> p = src;
            const ApplyResult r = ApplyOffset(rig, p.data(), db.NumBones(), offset);
            solved += r.solved;
            clamped += r.clamped;
            for (int s = 0; s < 2; ++s)
            {
                const BoneTransform a0 = Model(rig, src.data(), parents, rig.chain[s][2]);
                const BoneTransform a1 = Model(rig, p.data(), parents, rig.chain[s][2]);
                // lowering: the legs bend more; raising: limited to the reach slack of the straighter leg; the ankles stay either way
                (offset < 0.0f ? worstAnkle : worstAnkleUp) = std::fmax(offset < 0.0f ? worstAnkle : worstAnkleUp, legik::Len(a1.p - a0.p));
                worstRot = std::fmax(worstRot, QuatDist(a0.q, a1.q));
            }
            const BoneTransform p0 = Model(rig, src.data(), parents, rig.pelvis), p1 = Model(rig, p.data(), parents, rig.pelvis);
            worstPelvis = std::fmax(worstPelvis, legik::Len((p1.p - p0.p) - math::Vec3{0, 0, r.offset}));
            raiseLimited += r.raiseLimited ? 1 : 0;
            if (offset > 0.0f && r.offset < offset)
                raiseBelow = true;
            if (offset < 0.0f && r.offset != offset)
                lowerCut = true;
            // a bone under the pelvis outside the legs moves with the pelvis
            for (uint32_t b = 0; b < db.NumBones(); ++b)
                if (parents[b] == rig.pelvis && int32_t(b) != rig.chain[0][0] && int32_t(b) != rig.chain[1][0])
                {
                    const BoneTransform c0 = Model(rig, src.data(), parents, int32_t(b)), c1 = Model(rig, p.data(), parents, int32_t(b));
                    worstSpine = std::fmax(worstSpine, legik::Len((c1.p - c0.p) - math::Vec3{0, 0, r.offset}));
                }
        }
        std::vector<float> z = src;
        ApplyOffset(rig, z.data(), db.NumBones(), 0.0f);
        zeroUntouched = zeroUntouched && std::memcmp(z.data(), src.data(), z.size() * sizeof(float)) == 0;
        std::vector<float> o = src;
        ApplyOffset(rig, o.data(), db.NumBones() - 1, -0.02f);
        otherUntouched = otherUntouched && std::memcmp(o.data(), src.data(), o.size() * sizeof(float)) == 0;
    }
    std::printf("  %s: poses %zu solved %d clamped %d ankle down %.3f mm up %.3f mm foot rot %.4f deg pelvis %.3f mm spine %.3f mm\n", label,
                poses.size(), solved, clamped, worstAnkle * 1000.0f, worstAnkleUp * 1000.0f, worstRot * 57.29578f, worstPelvis * 1000.0f,
                worstSpine * 1000.0f);
    Check(worstAnkle <= 0.001f, (std::string(label) + ": lowered pelvis: ankles stay within 1 mm").c_str());
    std::printf("  %s: raises cut to the reach slack %d\n", label, raiseLimited);
    Check(worstAnkleUp <= 0.001f, (std::string(label) + ": raised pelvis (reach-bounded): ankles stay within 1 mm").c_str());
    Check(!lowerCut, (std::string(label) + ": a lowering is applied in full").c_str());
    Check(worstRot <= 0.001f, (std::string(label) + ": foot model rotation kept").c_str());
    Check(worstPelvis <= 1e-4f && worstSpine <= 1e-4f, (std::string(label) + ": pelvis and spine move by the offset along model z").c_str());
    Check(solved > 0, (std::string(label) + ": legs solved").c_str());
    Check(zeroUntouched, (std::string(label) + ": zero offset leaves the pose bit for bit").c_str());
    Check(otherUntouched, (std::string(label) + ": another bone count leaves the pose bit for bit").c_str());
}

static math::Quat X(float degrees) { return legik::AxisAngle({1, 0, 0}, degrees * math::kPi / 180.f); }

static void MockPose()
{
    MockDb db;
    std::vector<std::vector<float>> poses;
    for (float bend : {8.0f, 20.0f, 35.0f})
    {
        std::vector<float> p(db.NumBones() * 7, 0.0f);
        auto set = [&](int b, math::Vec3 t, math::Quat q) {
            float* v = &p[size_t(b) * 7];
            v[0] = t.x, v[1] = t.y, v[2] = t.z, v[3] = q.x, v[4] = q.y, v[5] = q.z, v[6] = q.w;
        };
        set(0, {0, 0, 0}, legik::AxisAngle({0, 0, 1}, 0.4f)); // root turned about z: model z is still up
        set(1, {0.02f, 0.01f, 0.95f}, X(5));
        for (int s = 0; s < 2; ++s)
        {
            const int u = s == 0 ? 2 : 6;
            set(u, {s == 0 ? .1f : -.1f, 0, -.05f}, X(bend));
            set(u + 1, {0, 0, -.44f}, X(-2 * bend));
            set(u + 2, {0, 0, -.43f}, X(bend));
            set(u + 3, {0, .13f, -.05f}, X(0));
        }
        set(10, {0, 0, .1f}, X(0));
        poses.push_back(p);
    }
    PoseCheck(db, "mock rig", poses);
}

// Moving platform (elevator ride, Filter::Step aPlatform): V steps onto the cabin over a 10 cm sill one frame before the ride (the offset sits near its bound
// at the entry), the cabin accelerates at 6 m/s^2 to 5 m/s up, cruises, brakes and stands. Variants: the flag one frame late at the
// start, the flag dropping two frames before the cabin stands. While the flag is on the offset fades to 0 and stays exactly 0; the
// offset never changes by more than 0.5 cm in one frame from the first platform frame on. Without the flag the same ride moves the offset by more than 0.5 cm.
static void ElevatorRide(int hz)
{
    const float dt = 1.0f / float(hz);
    const float rideStart = 1.0f, accel = 6.0f, vmax = 5.0f, cruise = 1.0f;
    const float tAcc = vmax / accel, rideEnd = rideStart + 2.0f * tAcc + cruise;
    auto cabin = [&](float t) {
        if (t <= rideStart)
            return 0.0f;
        const float u = t - rideStart;
        if (u < tAcc)
            return 0.5f * accel * u * u;
        const float zAcc = 0.5f * accel * tAcc * tAcc;
        if (u < tAcc + cruise)
            return zAcc + vmax * (u - tAcc);
        const float w = std::fmin(u - tAcc - cruise, tAcc);
        return zAcc + vmax * cruise + vmax * w - 0.5f * accel * w * w;
    };
    auto ground = [&](float t) { return (t >= rideStart - 1.5f * dt ? 0.10f : 0.0f) + cabin(t); }; // the sill one frame before the ride
    struct Variant
    {
        const char* name;
        int lateFrames;  // the flag turns on this many frames after the cabin starts
        int earlyFrames; // the flag drops this many frames before the cabin stands
        bool useFlag;
    };
    const Variant variants[] = {{"flag on", 0, 0, true}, {"flag one frame late", 1, 0, true}, {"flag drops 2 frames early", 0, 2, true},
                                {"without the flag", 0, 0, false}};
    const int n = int(4.5f * float(hz));
    const int startFrame = int(std::ceil(rideStart * float(hz)));
    const int endFrame = int(std::ceil(rideEnd * float(hz)));
    for (const Variant& v : variants)
    {
        Filter f;
        float prev = 0.0f, maxStep = 0.0f, maxRideAbs = 0.0f, entryAbs = 0.0f;
        bool zeroOnRide = true;
        for (int i = 0; i < n; ++i)
        {
            const float t = float(i) * dt;
            const bool flag = v.useFlag && i >= startFrame + v.lateFrames && i < endFrame - v.earlyFrames;
            const float o = f.Step(i ? dt : 0.0f, ground(t), true, flag);
            if (i == startFrame - 1)
                entryAbs = std::fabs(o);
            if (i >= startFrame + v.lateFrames) // from the first platform frame on (before it: the normal smoothing of the sill)
                maxStep = std::fmax(maxStep, std::fabs(o - prev));
            prev = o;
            const bool settledRide = flag && t >= rideStart + 0.4f; // the linear fade from the -4 cm bound takes 0.33 s
            if (settledRide && o != 0.0f)
                zeroOnRide = false;
            if (i >= startFrame && i < endFrame)
                maxRideAbs = std::fmax(maxRideAbs, std::fabs(o));
        }
        char name[200];
        if (v.useFlag)
        {
            std::snprintf(name, sizeof name,
                          "elevator ride (%s): offset 0 on the ride after 0.4 s, offset step max %.2f cm per frame (<= 0.5), entry offset %.2f cm",
                          v.name, double(maxStep) * 100.0, double(entryAbs) * 100.0);
            Check(zeroOnRide && maxStep <= 0.005f && entryAbs > 0.01f, name, hz);
        }
        else
        {
            std::snprintf(name, sizeof name, "elevator ride without the platform flag: the offset reaches %.2f cm on the ride (> 0.5)",
                          double(maxRideAbs) * 100.0);
            Check(maxRideAbs > 0.005f, name, hz);
        }
    }
}

// Platform flag flicker mid-fade (Filter::Step aPlatform; the flag is held platformHold s, the offset of the entry is carried): V steps up
// a 10 cm sill (the offset near its -4 cm bound, fading), then
//   on k:    the flag turns on for k frames on the ground and never again,
//   drop k:  the cabin starts at the sill (6 m/s^2 up to 5 m/s), the flag drops for k frames 3 frames into the ride and comes back,
//   ends k:  a k-frame ride (the flag on for k frames, the cabin stands after it).
// k = 1 .. 5. The offset never changes by more than 0.5 cm in one frame from the sill on (before the fix the tracker restart dropped it by
// 3 - 3.8 cm in one frame), and on the ride it is 0 once the carried offset faded out.
static void PlatformFlicker(int hz)
{
    const float dt = 1.0f / float(hz);
    const int sill = hz; // the sill frame (1 s of flat ground before)
    const int n = 3 * hz;
    for (int kind = 0; kind < 3; ++kind)
    {
        float worst = 0.0f, entryMin = 1.0f;
        bool zeroOnRide = true;
        for (int k = 1; k <= 5; ++k)
        {
            Filter f;
            float prev = 0.0f, maxStep = 0.0f, z = 0.0f, vz = 0.0f;
            for (int i = 0; i < n; ++i)
            {
                bool flag = false;
                bool moving = false;
                if (kind == 0)
                    flag = i >= sill + 1 && i < sill + 1 + k;
                else if (kind == 1)
                {
                    moving = i > sill;
                    flag = i > sill && !(i >= sill + 3 && i < sill + 3 + k);
                }
                else
                {
                    moving = i > sill && i <= sill + k;
                    flag = moving;
                }
                if (moving)
                {
                    vz = std::fmin(5.0f, vz + 6.0f * dt);
                    z += vz * dt;
                }
                else
                    vz = 0.0f;
                const float o = f.Step(i ? dt : 0.0f, (i >= sill ? 0.10f : 0.0f) + z, true, flag);
                if (i == sill)
                    entryMin = std::fmin(entryMin, std::fabs(o));
                if (i > sill)
                    maxStep = std::fmax(maxStep, std::fabs(o - prev));
                if (kind == 1 && float(i - sill) * dt >= 0.4f && o != 0.0f)
                    zeroOnRide = false;
                prev = o;
            }
            worst = std::fmax(worst, maxStep);
        }
        static const char* const kinds[] = {"flag on for 1 - 5 frames on the ground", "flag drops for 1 - 5 frames on a 5 m/s lift",
                                            "a 1 - 5 frame ride (the flag ends mid-fade)"};
        char name[240];
        std::snprintf(name, sizeof name, "platform flicker mid-fade (%s): offset step max %.2f cm per frame (<= 0.5), offset at the sill >= %.2f cm%s",
                      kinds[kind], double(worst) * 100.0, double(entryMin) * 100.0, kind == 1 ? ", 0 on the ride after 0.4 s" : "");
        Check(worst <= 0.005f && entryMin > 0.01f && zeroOnRide, name, hz);
    }
}

static void Stepper()
{
    namespace r = rt;
    r::SetEnabled(true);
    r::PublishGate(true, true);
    const int64_t tps = 1000000;
    int64_t now = 1000;
    for (int i = 0; i < 60; ++i, now += tps / 60)
        r::Step(now, tps, true, 0.0f);
    const uint64_t steps = r::g_steps.load();
    const float a = r::Step(now, tps, true, 0.20f);  // snap up 20 cm: offset < 0
    const float a2 = r::Step(now + 100, tps, true, 0.40f); // a second call of the same frame (0.1 ms later): no update
    Check(a < -0.01f && a2 == a && r::g_steps.load() == steps + 1, "stepper: one update per frame, repeated calls reuse the offset");
    r::PublishGate(true, true); // reset request (teleport / hand-back)
    const float c = r::Step(now + tps / 60, tps, true, 0.20f);
    Check(c == 0.0f && r::g_serialResets.load() >= 2, "stepper: a reset request restarts the tracker");
    const float d = r::Step(now + 2 * tps / 60, tps, false, 0.0f);
    Check(d == 0.0f, "stepper: no height source -> offset 0");
    // gate bit 1 (a moving platform carries V): a 20 cm rise per frame (a lift) leaves the offset at 0
    r::PublishGate(true, true, true);
    float e = 0.0f;
    for (int i = 0; i < 30; ++i)
        e = std::fmax(e, std::fabs(r::Step(now + (3 + i) * tps / 60, tps, true, 0.2f * float(i))));
    r::PublishGate(true, true, false);
    Check(e == 0.0f, "stepper: the platform gate bit passes the height through (offset 0 on a lift)");
    // snapshot application: melee lower body on the game pose (lowerScale < 1) and weight 0 stay untouched
    MockDb db;
    LegRig rig;
    rig.Init(db);
    r::PublishRig(rig);
    integ::PoseSnapshot snap{};
    snap.numBones = db.NumBones();
    snap.weight = 1.0f;
    snap.lowerScale = 1.0f;
    for (uint32_t b = 0; b < db.NumBones(); ++b)
    {
        float* v = &snap.pose[size_t(b) * 7];
        v[6] = 1.0f;
    }
    snap.pose[1 * 7 + 2] = 0.95f;
    for (int s = 0; s < 2; ++s)
    {
        const int u = s == 0 ? 2 : 6;
        snap.pose[size_t(u) * 7 + 0] = s == 0 ? .1f : -.1f;
        const math::Quat q = X(15);
        for (int k : {u, u + 2})
        {
            float* v = &snap.pose[size_t(k) * 7];
            v[3] = q.x, v[4] = q.y, v[5] = q.z, v[6] = q.w;
        }
        const math::Quat q2 = X(-30);
        float* v = &snap.pose[size_t(u + 1) * 7];
        v[2] = -.44f, v[3] = q2.x, v[4] = q2.y, v[5] = q2.z, v[6] = q2.w;
        snap.pose[size_t(u + 2) * 7 + 2] = -.43f;
    }
    integ::PoseSnapshot melee = snap;
    melee.lowerScale = 0.5f;
    Check(!r::ApplyToSnapshot(melee, -0.02f) && std::memcmp(melee.pose, snap.pose, sizeof(snap.pose)) == 0,
          "snapshot: melee lower body on the game pose -> untouched");
    integ::PoseSnapshot zeroW = snap;
    zeroW.weight = 0.0f;
    Check(!r::ApplyToSnapshot(zeroW, -0.02f), "snapshot: weight 0 -> untouched");
    integ::PoseSnapshot on = snap;
    Check(r::ApplyToSnapshot(on, -0.02f) && std::fabs(on.pose[1 * 7 + 2] - (0.95f - 0.02f)) < 1e-5f, "snapshot: pelvis moved by the offset");
    r::SetEnabled(false);
    Check(!r::Enabled(), "stepper: off");
}

int main(int argc, char** argv)
{
    Check(rt::Enabled(), "default: the stage is on at startup (the menu default mm.bodyHeightSmooth)");
    for (int hz : {30, 60, 144})
        Lift(hz);
    for (int hz : {30, 60, 144})
        Tracker(hz);
    RateIndependence();
    for (int hz : {30, 60, 144})
        ElevatorRide(hz);
    for (int hz : {30, 60})
        PlatformFlicker(hz);
    HitchGuardAndGate();
    MockPose();
    Stepper();
    if (argc >= 2)
    {
        Database db;
        std::string error;
        if (!db.LoadFromFile(argv[1], 0x72CB314EB01F562Dull, error))
        {
            std::printf("database: %s\n", error.c_str());
            return 2;
        }
        std::vector<std::vector<float>> poses;
        const uint32_t step = std::max<uint32_t>(1, db.NumFrames() / 400);
        for (uint32_t f = 0; f < db.NumFrames(); f += step)
        {
            const auto p = db.Pose(f);
            poses.emplace_back(p.begin(), p.end());
        }
        PoseCheck(db, "database rig", poses);
    }
    std::printf("%d passed, %d failed\n", passed, failed);
    return failed ? 1 : 0;
}
