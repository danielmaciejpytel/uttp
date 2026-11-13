// Offline test of the ALS layering runtime (src/mm/Additive.hpp, Overlay.hpp, OverlayFile.hpp; port 10.6).
//
//   utpp_mm_overlay_test [woman.utppov] [man.utppov]
//
// Synthetic: identity (zero weights / Default overlay = L), single-bone known deltas (D_MS / A_MS, alpha), mesh-space
// vs local-space additive on a 3-bone chain, curve-weight interpolation + LBB branch ramp, switch blend continuity
// (per-bone rotation change per frame bounded at 30 / 60 / 144 fps), state machine (state table), lean, spine yaw,
// weapon map. Data (when the .utppov files are given): fail-closed load, stored mesh rotations = FK of the local
// pose, every baked delta reproduces its full target pose from base + delta (A_MS and A_LS) within 0.1 deg,
// identity on real poses.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "../src/mm/AimSpineYawFilter.hpp"
#include "../src/mm/OverlayFile.hpp"
#include "../src/mm/HipFirePose.hpp"

using namespace utpp::mm;
using namespace utpp::mm::ov;

static int g_pass = 0, g_fail = 0;
static void Check(bool aOk, const std::string& aWhat)
{
    if (aOk)
        ++g_pass;
    else
    {
        ++g_fail;
        std::printf("FAIL: %s\n", aWhat.c_str());
    }
}
static Quat AxisAngle(Vec3 aAxis, float aDeg)
{
    const float h = 0.5f * aDeg * math::kPi / 180.0f;
    const float l = std::sqrt(math::Dot(aAxis, aAxis));
    return Quat{aAxis.x / l * std::sin(h), aAxis.y / l * std::sin(h), aAxis.z / l * std::sin(h), std::cos(h)};
}
static float MaxPoseDiffDeg(const Skeleton& sk, const Pose& a, const Pose& b, float* aMaxT = nullptr)
{
    Quat qa[kMaxBones], qb[kMaxBones];
    MeshRotations(sk, a, qa);
    MeshRotations(sk, b, qb);
    float m = 0.0f, mt = 0.0f;
    for (int i = 0; i < sk.n; ++i)
    {
        m = std::max(m, AngleDeg(qa[i], qb[i]));
        const Vec3 d = a.b[i].t - b.b[i].t;
        mt = std::max(mt, std::sqrt(math::Dot(d, d)));
    }
    if (aMaxT)
        *aMaxT = mt;
    return m;
}

// Synthetic 12-bone rig: 0 Root, 1 Hips, 2 LeftUpLeg, 3 RightUpLeg, 4 Spine, 5 Spine1, 6 Spine2, 7 Spine3, 8 Neck,
// 9 LeftShoulder, 10 RightShoulder, 11 LeftInHandThumb (child of 9 for the test).
static const char* kNames[] = {"Root",   "Hips", "LeftUpLeg",    "RightUpLeg",    "Spine",          "Spine1",
                               "Spine2", "Spine3", "Neck", "LeftShoulder", "RightShoulder", "LeftInHandThumb"};
static Skeleton MakeSkel()
{
    Skeleton sk;
    sk.n = 12;
    const int16_t p[] = {-1, 0, 1, 1, 1, 4, 5, 6, 7, 7, 7, 9};
    for (int i = 0; i < 12; ++i)
        sk.parent[i] = p[i];
    return sk;
}
static void RandomPose(std::mt19937& rng, int n, Pose& p, float aDeg = 40.0f)
{
    std::uniform_real_distribution<float> u(-1.0f, 1.0f);
    p.n = n;
    for (int i = 0; i < n; ++i)
        p.b[i] = Xf{AxisAngle(Vec3{u(rng), u(rng), u(rng) + 0.01f}, aDeg * u(rng)), Vec3{u(rng), u(rng), u(rng)} * 0.2f,
                    Vec3{1.0f, 1.0f, 1.0f}};
}

static void TestSynthetic()
{
    const Skeleton sk = MakeSkel();
    Check(sk.Valid(), "synthetic skeleton valid");
    Regions R;
    BuildRegions(sk, [](int i) { return std::string_view(kNames[i]); }, R);
    Check(std::fabs(R.spine.w[4] - 0.25f) < 1e-6f && std::fabs(R.spine.w[7] - 1.0f) < 1e-6f && R.spine.w[8] == 1.0f &&
              R.spine.w[1] == 0.0f,
          "spine ramp 0.25..1.0, children 1, hips 0");
    Check(R.pelvis.w[1] == 1.0f && R.pelvis.w[2] == 0.0f && R.pelvis.w[3] == 0.0f && R.pelvis.w[4] == 1.0f,
          "pelvis mask: Hips subtree minus thighs");
    Check(R.handL.w[11] == 1.0f && R.handL.w[9] == 0.0f && R.spineYawCount == 4, "hand mask + spine yaw bones");

    std::mt19937 rng(7);
    auto L = std::make_unique<Pose>(), B = std::make_unique<Pose>(), OP = std::make_unique<Pose>(),
         out = std::make_unique<Pose>(), BP = std::make_unique<Pose>(), OB = std::make_unique<Pose>();
    auto S = std::make_unique<LayerScratch>();
    RandomPose(rng, sk.n, *L);
    RandomPose(rng, sk.n, *B);
    RandomPose(rng, sk.n, *OP);
    RandomPose(rng, sk.n, *BP);
    RandomPose(rng, sk.n, *OB);
    auto dP = std::make_unique<Additive>();
    MakeAdditive(sk, *BP, *OB, true, *dP);

    // 1. identity: zero curve weights -> L; Default overlay (no OP) -> L
    {
        LayerInputs in;
        in.L = L.get();
        in.B = B.get();
        in.OP = OP.get();
        in.dP = dP.get();
        ComposeLayers(sk, R, in, *S, *out);
        float mt = 0.0f;
        const float d = MaxPoseDiffDeg(sk, *L, *out, &mt);
        std::printf("identity zero weights: max %.6f deg, %.7f m\n", d, mt);
        Check(d < 1e-3f && mt < 1e-6f, "identity: zero weights = L");
        in.OP = nullptr;
        ComposeLayers(sk, R, in, *S, *out);
        Check(MaxPoseDiffDeg(sk, *L, *out) < 1e-4f, "identity: Default overlay = L (copy)");
        // all weights 1, L == B, dP null, LS 0 -> regions take OP mesh rotations
        in.OP = OP.get();
        in.L = B.get();
        in.dP = nullptr;
        for (int c : {kLegs, kPelvis, kSpine, kSpineAdd, kHead, kHeadAdd, kArmL, kArmLAdd, kArmR, kArmRAdd})
            in.curves.Set(c, 1.0f);
        ComposeLayers(sk, R, in, *S, *out);
        Quat qo[kMaxBones], qp[kMaxBones];
        MeshRotations(sk, *out, qo);
        MeshRotations(sk, *OP, qp);
        float m = 0.0f;
        for (int i = 1; i < sk.n; ++i)
            m = std::max(m, AngleDeg(qo[i], qp[i]));
        std::printf("full weights, L = B: max mesh diff vs OP %.6f deg\n", m);
        Check(m < 1e-2f, "full weights with L = B reproduce OP mesh rotations");
    }

    // 2. single-bone known delta (D_MS / A_MS, alpha 1 and 0.5)
    {
        Pose& b0 = *BP;
        RandomPose(rng, sk.n, b0);
        Pose t0 = b0;
        t0.b[5].r = math::QuatNormalize(math::QuatMul(t0.b[5].r, AxisAngle(Vec3{1, 0, 0}, 30.0f)));
        t0.b[5].t = t0.b[5].t + Vec3{0.1f, 0.0f, 0.0f};
        Additive d;
        MakeAdditive(sk, b0, t0, true, d);
        float other = 0.0f;
        for (int i = 0; i < sk.n; ++i)
            if (i != 5 && sk.DepthBelow(5, i) < 0)
                other = std::max(other, AngleDeg(d.b[i].r, Quat{}));
        Check(std::fabs(AngleDeg(d.b[5].r, Quat{}) - 30.0f) < 1e-3f && other < 1e-3f, "D_MS: one bone 30 deg, others 0");
        Pose y = *L;
        Quat q0[kMaxBones], q1[kMaxBones];
        MeshRotations(sk, y, q0);
        ApplyAdditive(sk, y, d, 1.0f);
        MeshRotations(sk, y, q1);
        Check(std::fabs(AngleDeg(q0[5], q1[5]) - 30.0f) < 1e-2f && std::fabs(y.b[5].t.x - L->b[5].t.x - 0.1f) < 1e-6f,
              "A_MS alpha 1: +30 deg mesh, +0.1 m");
        y = *L;
        ApplyAdditive(sk, y, d, 0.5f);
        MeshRotations(sk, y, q1);
        Check(std::fabs(AngleDeg(q0[5], q1[5]) - 15.0f) < 1e-2f, "A_MS alpha 0.5: +15 deg");
        Check(AngleDeg(q0[2], q1[2]) < 1e-3f && AngleDeg(q0[3], q1[3]) < 1e-3f, "A_MS: bones outside the subtree unchanged");
    }

    // 3. mesh-space vs local-space on a 3-bone chain
    {
        Skeleton c3;
        c3.n = 3;
        c3.parent[0] = -1;
        c3.parent[1] = 0;
        c3.parent[2] = 1;
        Pose base, tgt, y;
        base.n = tgt.n = y.n = 3;
        for (int i = 0; i < 3; ++i)
            base.b[i] = tgt.b[i] = y.b[i] = Xf{Quat{}, Vec3{0, 0, 1}, Vec3{1, 1, 1}};
        const Quat dz = AxisAngle(Vec3{0, 0, 1}, 90.0f);
        tgt.b[1].r = dz; // bend the middle bone
        const Quat rx = AxisAngle(Vec3{1, 0, 0}, 90.0f);
        y.b[0].r = rx; // different parent orientation in the target pose
        Additive ms, ls;
        MakeAdditive(c3, base, tgt, true, ms);
        MakeAdditive(c3, base, tgt, false, ls);
        Check(AngleDeg(ms.b[2].r, dz) < 1e-3f && AngleDeg(ls.b[2].r, Quat{}) < 1e-3f,
              "chain: child delta = parent bend in mesh space, identity in local space");
        Pose yms = y, yls = y;
        ApplyAdditive(c3, yms, ms, 1.0f);
        ApplyAdditive(c3, yls, ls, 1.0f);
        Quat gms[3], gls[3];
        MeshRotations(c3, yms, gms);
        MeshRotations(c3, yls, gls);
        const Quat expMs = math::QuatMul(dz, rx);  // G1 = d * Q0
        const Quat expLs = math::QuatMul(rx, dz);  // G1 = Q0 * d
        std::printf("chain: MS bone1 err %.6f, LS bone1 err %.6f, MS vs LS %.3f deg\n", AngleDeg(gms[1], expMs),
                    AngleDeg(gls[1], expLs), AngleDeg(gms[1], gls[1]));
        Check(AngleDeg(gms[1], expMs) < 1e-3f && AngleDeg(gms[2], expMs) < 1e-3f, "chain: A_MS G1 = G2 = d * Q0");
        Check(AngleDeg(gls[1], expLs) < 1e-3f && AngleDeg(gls[2], expLs) < 1e-3f, "chain: A_LS G1 = G2 = Q0 * d");
        Check(AngleDeg(gms[1], gls[1]) > 60.0f, "chain: MS and LS differ when the parent is rotated");
        // LBB_MS weight 0 on bone 2 keeps the base MESH rotation; LBB_LS keeps the local one
        BoneWeights w;
        w.Clear(3);
        w.w[1] = 1.0f;
        Pose o;
        LayeredBlend(c3, y, yms, w, 1.0f, true, o);
        Quat go[3];
        MeshRotations(c3, o, go);
        Check(AngleDeg(go[1], gms[1]) < 1e-3f && AngleDeg(go[2], rx) < 1e-3f, "LBB_MS: weight-0 child keeps base mesh rotation");
        LayeredBlend(c3, y, yms, w, 1.0f, false, o);
        MeshRotations(c3, o, go);
        Check(AngleDeg(go[2], gms[1]) < 1e-3f, "LBB_LS: weight-0 child keeps base local rotation (follows parent)");
    }

    // 4. curve-weight interpolation (region blend monotonic, endpoints exact) on Spine2 (ramp 0.75 inside LBB)
    {
        LayerInputs in;
        in.L = L.get();
        in.B = B.get();
        in.OP = OP.get();
        float prev = -1.0f, full = 0.0f;
        bool mono = true;
        float ang[5]{};
        for (int k = 0; k <= 4; ++k)
        {
            const float w = 0.25f * k;
            in.curves = Curves{};
            in.curves.Set(kSpine, w);
            in.curves.Set(kSpineAdd, 1.0f);
            ComposeLayers(sk, R, in, *S, *out);
            Quat a[kMaxBones], b[kMaxBones];
            MeshRotations(sk, *L, a);
            MeshRotations(sk, *out, b);
            ang[k] = AngleDeg(a[7], b[7]); // Spine3: spine ramp weight 1
            mono = mono && ang[k] >= prev - 1e-3f;
            prev = ang[k];
        }
        full = ang[4];
        std::printf("spine curve 0/.25/.5/.75/1 -> Spine3 offset %.3f %.3f %.3f %.3f %.3f deg\n", ang[0], ang[1], ang[2],
                    ang[3], ang[4]);
        Check(ang[0] < 1e-3f && mono && full > 1.0f, "curve weight: 0 = L, monotonic");
        Check(std::fabs(ang[2] - 0.5f * full) < 0.05f * full + 0.5f, "curve weight 0.5 ~ half angle (nlerp)");
    }

    // 5. switch blend continuity at 30 / 60 / 144 fps (pose 0.2 s, style 0.3 s, aim CubicInOut 0.75 s)
    {
        Pose poseA = *OP, poseB = *OP;
        poseB.b[9].r = math::QuatNormalize(math::QuatMul(poseB.b[9].r, AxisAngle(Vec3{0, 1, 0}, 120.0f)));
        poseB.b[5].r = math::QuatNormalize(math::QuatMul(poseB.b[5].r, AxisAngle(Vec3{1, 0, 0}, 60.0f)));
        Curves cA, cB;
        for (int c : {kSpine, kSpineAdd, kHead, kArmL, kArmR, kArmLAdd})
        {
            cA.Set(c, 1.0f);
            cB.Set(c, c == kArmL ? 0.2f : 1.0f);
        }
        cB.Set(kArmLLS, 1.0f);
        for (float fps : {30.0f, 60.0f, 144.0f})
        {
            const float dt = 1.0f / fps;
            OverlayMachine m;
            OverlayInputs oi;
            oi.overlay = PoseOverlay::Rifle;
            oi.style = Style::Neutral;
            m.Update(oi, dt);
            Pose prevOut = *L;
            bool havePrev = false;
            float maxStep = 0.0f, popStep = 0.0f;
            auto prevNoBlend = std::make_unique<Pose>();
            for (int f = 0; f < static_cast<int>(1.5f * fps); ++f)
            {
                if (f == static_cast<int>(0.3f * fps))
                    oi.overlay = PoseOverlay::Pistol2H; // pose switch
                if (f == static_cast<int>(0.6f * fps))
                    oi.style = Style::Feminine;         // style switch
                m.Update(oi, dt);
                // keys -> synthetic poses: Rifle = A, Pistol2H = B
                const Pose* ps[6];
                float ws[6];
                Curves cur;
                for (int i = 0; i < m.poses.count; ++i)
                {
                    PoseOverlay o;
                    AimState a;
                    bool cr, mv;
                    DecodePoseKey(m.poses.key[i], o, a, cr, mv);
                    ps[i] = o == PoseOverlay::Pistol2H ? &poseB : &poseA;
                    ws[i] = m.poses.w[i];
                    cur.Accumulate(o == PoseOverlay::Pistol2H ? cB : cA, ws[i]);
                }
                Pose opb;
                BlendPosesWeighted(ps, ws, m.poses.count, opb);
                Additive id, dPb;
                id.SetIdentity(sk.n, true);
                const Additive* ds[2] = {&id, dP.get()};
                const float dw[2] = {m.styles.WeightOf(0), m.styles.WeightOf(1)};
                BlendAdditives(ds, dw, 2, dPb);
                dPb.meshSpace = true;
                LayerInputs in;
                in.L = L.get();
                in.B = B.get();
                in.OP = &opb;
                in.dP = &dPb;
                in.curves = cur;
                ComposeLayers(sk, R, in, *S, *out);
                // the same frame without blending (motion pop)
                LayerInputs pin = in;
                pin.OP = oi.overlay == PoseOverlay::Pistol2H ? &poseB : &poseA;
                pin.curves = oi.overlay == PoseOverlay::Pistol2H ? cB : cA;
                pin.dP = oi.style == Style::Feminine ? dP.get() : &id;
                Pose popOut;
                ComposeLayers(sk, R, pin, *S, popOut);
                if (havePrev)
                {
                    maxStep = std::max(maxStep, MaxPoseDiffDeg(sk, prevOut, *out));
                    popStep = std::max(popStep, MaxPoseDiffDeg(sk, *prevNoBlend, popOut));
                }
                prevOut = *out;
                *prevNoBlend = popOut;
                havePrev = true;
            }
            // bound: the largest pose difference (<= ~180 deg) spread over 0.2 s, slope 1 (linear) -> 180 * dt / 0.2
            const float bound = 180.0f * dt / 0.2f;
            std::printf("switch %.0f fps: max per-frame bone change %.3f deg (bound %.3f), without blend %.3f deg\n", fps,
                        maxStep, bound, popStep);
            Check(maxStep <= bound, "switch blend: per-frame rotation change bounded at " + std::to_string(int(fps)) + " fps");
            Check(popStep > 2.0f * maxStep, "switch blend: one-frame pop would be larger");
        }
    }

    // 6. state machine (state table)
    {
        const float dt = 1.0f / 60.0f;
        OverlayMachine m;
        OverlayInputs oi;
        oi.overlay = PoseOverlay::Rifle;
        m.Update(oi, dt);
        oi.wantsToAim = true;
        int frames = 0;
        while (m.aim != AimState::Aiming && frames < 200)
        {
            m.Update(oi, dt);
            ++frames;
        }
        std::printf("state: Relaxed -> Aiming in %d frames (%.3f s)\n", frames, frames * dt);
        Check(m.aim == AimState::Aiming && frames * dt >= 0.04f && frames * dt < 0.15f, "Relaxed -> Ready -> Aiming (Ready weight >= 0.25)");
        oi.wantsToAim = false;
        for (int i = 0; i < 30; ++i)
            m.Update(oi, dt);
        Check(m.aim == AimState::Aiming, "Aiming held for 0.75 s minimum");
        for (int i = 0; i < 20; ++i)
            m.Update(oi, dt);
        Check(m.aim == AimState::Ready, "Aiming -> Ready after 0.75 s");
        for (int i = 0; i < 170; ++i)
            m.Update(oi, dt);
        Check(m.aim == AimState::Ready, "Ready held < 3 s idle");
        for (int i = 0; i < 20; ++i)
            m.Update(oi, dt);
        Check(m.aim == AimState::Relaxed, "Ready -> Relaxed after 3 s idle");
        oi.wantsToAim = true;
        for (int i = 0; i < 30; ++i)
            m.Update(oi, dt);
        oi.overlay = PoseOverlay::Default;
        m.Update(oi, dt);
        Check(m.aim == AimState::Relaxed, "non-aim overlay forces Relaxed");
        float wsum = 0.0f;
        for (int i = 0; i < m.poses.count; ++i)
            wsum += m.poses.w[i];
        Check(std::fabs(wsum - 1.0f) < 1e-4f, "pose crossfade weights sum to 1");
    }

    // Combat Ready must be distinct from a non-aiming Relaxed input.
    for (int hz : {30, 60, 144})
    {
        OverlayMachine m;
        OverlayInputs oi;
        oi.overlay = PoseOverlay::Pistol1H;
        oi.wantsReady = true;
        const float dt = 1.0f / float(hz);
        for (int i = 0; i < 5 * hz; ++i)
            m.Update(oi, dt);
        Check(m.aim == AimState::Ready,
              "explicit combat Ready remains Ready without ADS at " + std::to_string(hz) + " Hz");
        Check(m.AimingWeight() == 0.0f, "combat Ready does not enable aim offset");
        oi.overlay = PoseOverlay::Pistol2H;
        oi.wantsToAim = true;
        m.Update(oi, dt);
        Check(m.poses.duration == m.times.pistol1HTo2HAim && m.aimW.duration == m.times.pistol1HTo2HAim,
              "1H Ready to 2H Aim fades pose and aim offset together at " + std::to_string(hz) + " Hz");
        for (int i = 1; i < hz; ++i)
            m.Update(oi, dt);
        Check(m.aim == AimState::Aiming && m.AimingWeight() > 0.999f,
              "Ready 1H to Aim 2H reaches ADS");
        oi.overlay = PoseOverlay::Pistol1H;
        oi.wantsToAim = false;
        for (int i = 0; i < 5 * hz; ++i)
            m.Update(oi, dt);
        Check(m.aim == AimState::Ready && m.AimingWeight() == 0.0f,
              "Aim exit returns to explicit Ready without idle expiry");
        oi.sprinting = true;
        for (int i = 0; i < hz; ++i)
            m.Update(oi, dt);
        Check(m.aim == AimState::Relaxed, "sprint suppresses explicit combat Ready");
        oi.sprinting = false;
        for (int i = 0; i < hz; ++i)
            m.Update(oi, dt);
        Check(m.aim == AimState::Ready, "combat Ready resumes after sprint");
        oi.wantsReady = false;
        for (int i = 0; i < 5 * hz; ++i)
            m.Update(oi, dt);
        Check(m.aim == AimState::Relaxed, "cleared combat Ready uses normal idle timeout");
        oi.wantsReady = true;
        oi.overlay = PoseOverlay::Default;
        for (int i = 0; i < hz; ++i)
            m.Update(oi, dt);
        Check(m.aim == AimState::Relaxed, "non-aim overlay ignores combat Ready");

        for (PoseOverlay steady : {PoseOverlay::Rifle, PoseOverlay::Pistol2H})
        {
            OverlayMachine other;
            OverlayInputs input;
            input.overlay = steady;
            input.wantsReady = true;
            for (int i = 0; i < hz; ++i)
                other.Update(input, dt);
            input.wantsToAim = true;
            other.Update(input, dt);
            Check(other.poses.duration == other.times.readyToAiming &&
                  other.aimW.duration == other.times.readyToAiming,
                  "steady rifle/2H Ready to Aim keeps original timing at " + std::to_string(hz) + " Hz");
        }

        // summed weight of the Aiming-state pose keys: the hands / arms crossfade (AimingWeight() is the AO alpha)
        const auto aimPoseW = [](const OverlayMachine& aM) {
            float w = 0.0f;
            for (int i = 0; i < aM.poses.count; ++i)
            {
                PoseOverlay ko;
                AimState ka;
                bool kc, km;
                DecodePoseKey(aM.poses.key[i], ko, ka, kc, km);
                if (ka == AimState::Aiming)
                    w += aM.poses.w[i];
            }
            return w;
        };
        // P9 C: a hip shot out of Ready (hipFireRaise) raises the HANDS in hipFireReadyToAiming; real ADS keeps readyToAiming.
        // P9 B (review 3): the AO alpha takes the raise with the hands (the second shot of a burst, ~0.1 s, has the full sweep pitch and yaw);
        // its yaw step stays under AoJumpDeg(dt) of the AO filter (camera-like tracking passes through), 480 deg/s is only its catch-up rate.
        for (PoseOverlay steady : {PoseOverlay::Rifle, PoseOverlay::Pistol1H, PoseOverlay::Pistol2H})
        {
            OverlayMachine hip, ads;
            OverlayInputs input;
            input.overlay = steady;
            input.wantsReady = true;
            for (int i = 0; i < hz; ++i)
            {
                hip.Update(input, dt);
                ads.Update(input, dt);
            }
            input.wantsToAim = true;
            OverlayInputs shot = input;
            shot.hipFireRaise = true;
            int hipFrames = 0, adsFrames = 0;
            float aoDiff = 0.0f, aoStepDeg = 0.0f, aoSync = 0.0f, prevAo = hip.AimingWeight();
            for (int f = 0; f < 3 * hz; ++f)
            {
                hip.Update(shot, dt);
                ads.Update(input, dt);
                if (hipFrames == 0 && aimPoseW(hip) >= 0.8f)
                    hipFrames = f + 1;
                if (adsFrames == 0 && aimPoseW(ads) >= 0.8f)
                    adsFrames = f + 1;
                aoDiff = std::max(aoDiff, std::fabs(hip.AimingWeight() - ads.AimingWeight()));
                aoStepDeg = std::max(aoStepDeg, (hip.AimingWeight() - prevAo) * 60.0f); // a yaw at the +-60 deg clamp
                aoSync = std::max(aoSync, std::fabs(hip.AimingWeight() - aimPoseW(hip)));
                prevAo = hip.AimingWeight();
            }
            const std::string tag = " (" + std::to_string(int(steady)) + ") at " + std::to_string(hz) + " Hz";
            Check(hipFrames > 0 && hipFrames * dt <= 0.1f + 1.5f * dt && hipFrames * dt >= 0.05f,
                  "hipfire P9 C: the hip shot raises the hands to 80% within 0.1 s, frames=" + std::to_string(hipFrames) + tag);
            Check(adsFrames > 0 && adsFrames * dt >= 0.8f * 0.2f - 1.5f * dt && adsFrames * dt <= 0.2f + 1.5f * dt && adsFrames > hipFrames,
                  "hipfire P9 C: real ADS keeps the 0.2 s Ready -> Aiming raise, frames=" + std::to_string(adsFrames) + tag);
            (void)aoDiff;
            Check(aoSync <= 0.01f, "hipfire P9 B: the AO alpha follows the hands pose during the hip raise (+-0.01), max diff=" + std::to_string(aoSync) + tag);
            Check(aoStepDeg <= motionpose::AoJumpDeg(dt),
                  "hipfire P9 B: the AO yaw step per frame (60 deg clamp) stays under AoJumpDeg(dt), deg/frame=" + std::to_string(aoStepDeg) + tag);
        }
        {
            // the larger authored 1H Ready -> 2H Aim arc is never shortened by the hip raise
            OverlayMachine mixed;
            OverlayInputs input;
            input.overlay = PoseOverlay::Pistol1H;
            input.wantsReady = true;
            for (int i = 0; i < hz; ++i)
                mixed.Update(input, dt);
            input.overlay = PoseOverlay::Pistol2H;
            input.wantsToAim = true;
            input.hipFireRaise = true;
            mixed.Update(input, dt);
            Check(mixed.poses.duration == mixed.times.pistol1HTo2HAim && mixed.aimW.duration == mixed.times.pistol1HTo2HAim,
                  "hipfire P9 C: 1H Ready to 2H Aim keeps the 0.45 s arc under the hip raise at " + std::to_string(hz) + " Hz");
        }

        // P9 A: a crouch / move / slide key change 0 - 4 frames after the hip shot must not restart the hands fade at poseSwitch (T2 guard of
        // P1.md, 0.2 s): the hands reach the Aiming pose when they would have without the change and the AO alpha never runs ahead of them.
        {
            // aKind: 0 none, 1 crouch flips, 2 moving starts, 3 crouch flips then moving starts one frame later (a slide start)
            const auto run = [&](PoseOverlay aOv, int aKind, int aFrame, float* aT90, float* aGap) {
                OverlayMachine m;
                OverlayInputs in;
                in.overlay = aOv;
                in.wantsReady = true;
                for (int i = 0; i < hz; ++i)
                    m.Update(in, dt);
                in.wantsToAim = true;
                in.hipFireRaise = true;
                *aT90 = -1.0f;
                *aGap = -1.0f;
                for (int f = 0; f < 2 * hz; ++f)
                {
                    if (aKind == 1 && f == aFrame) in.crouch = true;
                    if (aKind == 2 && f == aFrame) in.moving = in.hasVelocity = true;
                    if (aKind == 3 && f == aFrame) in.crouch = true;
                    if (aKind == 3 && f == aFrame + 1) in.moving = in.hasVelocity = true;
                    m.Update(in, dt);
                    *aGap = std::max(*aGap, m.AimingWeight() - aimPoseW(m));
                    if (*aT90 < 0.0f && aimPoseW(m) >= 0.9f)
                        *aT90 = float(f + 1) * dt;
                }
            };
            for (PoseOverlay steady : {PoseOverlay::Rifle, PoseOverlay::Pistol2H})
            {
                float refT90 = 0.0f, refGap = 0.0f;
                run(steady, 0, 0, &refT90, &refGap);
                Check(refT90 > 0.0f && refT90 <= 0.1f + 1.5f * dt, "hipfire P9 A: the reference raise without a key change reaches 0.9 within 0.1 s at " + std::to_string(hz) + " Hz, t90=" + std::to_string(refT90));
                for (int kind = 1; kind <= 3; ++kind)
                    for (int frame = 0; frame <= 4; ++frame)
                    {
                        float t90 = 0.0f, gap = 0.0f;
                        run(steady, kind, frame, &t90, &gap);
                        const std::string tag = " (overlay " + std::to_string(int(steady)) + ", change kind " + std::to_string(kind) + " at frame " +
                                                std::to_string(frame) + ", " + std::to_string(hz) + " Hz) t90=" + std::to_string(t90) +
                                                " gap=" + std::to_string(gap);
                        // T2 (P1.md): within the key-change-free time + 0.1 s - and the raise itself must stay 0.1 s: ref + 1.5 frames
                        Check(t90 > 0.0f && t90 <= refT90 + 1.5f * dt && t90 <= 0.1f + 1.5f * dt,
                              "hipfire P9 A: a stance / move key change inside the hip raise does not restart the hands fade" + tag);
                        // the floor (half of the raise) can let the hands finish up to 0.11 (two changes in the last frames: 0.15) after the AO alpha
                        Check(gap <= 0.16f, "hipfire P9 A: the AO alpha does not run ahead of the hands pose" + tag);
                    }
            }
        }

        // P9 A2 (review 3): hipFireRaise stays set for the whole 0.75 s hold of the hip pose, but only the fade of the RAISE may be handed on to
        // the next key change. Move / crouch / stop changes after it, a second one inside the 0.2 s fade of the first, move every pose key by
        // at most dt / poseSwitch per frame (as before the guard: 0.167 / 0.083 / 0.035); one in the last frames of the raise by at most
        // dt / (half of the raise), not by a whole Stand_Aim -> Crouch_Aim (13 - 19 deg in arms and head) in one frame.
        {
            const auto keyStep = [&](PoseOverlay aOv, const std::vector<std::pair<float, int>>& aEvents, float aFrom, float aTo) {
                OverlayMachine m;
                OverlayInputs in;
                in.overlay = aOv;
                in.wantsReady = true;
                for (int i = 0; i < hz; ++i)
                    m.Update(in, dt);
                in.wantsToAim = true;
                in.hipFireRaise = true;
                float prev[72] = {}, now[72] = {}, maxStep = 0.0f;
                for (int i = 0; i < m.poses.count; ++i)
                    prev[m.poses.key[i]] = m.poses.w[i];
                for (int f = 0; f < 2 * hz; ++f)
                {
                    const float t = float(f) * dt;
                    for (const auto& e : aEvents)
                        if (std::fabs(t - e.first) < 0.5f * dt)
                        {
                            if (e.second == 0) in.moving = in.hasVelocity = !in.moving;
                            if (e.second == 1) in.crouch = !in.crouch;
                        }
                    m.Update(in, dt);
                    std::fill(std::begin(now), std::end(now), 0.0f);
                    for (int i = 0; i < m.poses.count; ++i)
                        now[m.poses.key[i]] = m.poses.w[i];
                    if (t >= aFrom && t <= aTo)
                        for (int k = 0; k < 72; ++k)
                            maxStep = std::max(maxStep, std::fabs(now[k] - prev[k]));
                    std::copy(std::begin(now), std::end(now), std::begin(prev));
                }
                return maxStep;
            };
            for (PoseOverlay steady : {PoseOverlay::Rifle, PoseOverlay::Pistol2H})
            {
                // after the raise: move, then crouch 0.05 - 0.18 s later (the reported 0.15 - 0.18 s: run -> crouch), crouch then move, crouch then stand
                float worst = 0.0f;
                for (const float gap : {0.05f, 0.10f, 0.15f, 0.18f})
                {
                    const float g = std::round(gap / dt) * dt;
                    worst = std::max(worst, keyStep(steady, {{0.30f, 0}, {0.30f + g, 1}}, 0.25f, 2.0f));
                    worst = std::max(worst, keyStep(steady, {{0.30f, 1}, {0.30f + g, 0}}, 0.25f, 2.0f));
                    worst = std::max(worst, keyStep(steady, {{0.30f, 1}, {0.30f + g, 1}}, 0.25f, 2.0f));
                    worst = std::max(worst, keyStep(steady, {{0.30f, 0}, {0.30f + g, 0}, {0.30f + 2.0f * g, 1}}, 0.25f, 2.0f));
                }
                std::printf("hipfire P9 A2: %d Hz, overlay %d: key changes after the raise move a pose key by at most %.3f per frame (linear 0.2 s: %.3f)\n", hz,
                            int(steady), double(worst), double(dt / 0.2f));
                Check(worst <= dt / 0.2f + 0.005f, "hipfire P9 A2: key changes after the raise move a pose key by at most dt / 0.2 s per frame (" +
                                                       std::to_string(worst) + " vs " + std::to_string(dt / 0.2f) + ") at " + std::to_string(hz) + " Hz");
                // inside the raise, the last frames included: the floor (half of the raise) bounds the step
                float inRaise = 0.0f;
                for (int frame = 0; frame <= 7; ++frame)
                    inRaise = std::max(inRaise, keyStep(steady, {{float(frame) * dt, 1}}, 0.0f, 2.0f));
                for (int frame = 0; frame <= 7; ++frame)
                    inRaise = std::max(inRaise, keyStep(steady, {{float(frame) * dt, 0}}, 0.0f, 2.0f));
                const float limit = std::max(dt / (0.5f * 0.1f), dt / 0.1f) + 0.01f; // the raise itself moves a key by dt / 0.1 per frame
                std::printf("hipfire P9 A2: %d Hz, overlay %d: key changes inside the raise move a pose key by at most %.3f per frame (limit %.3f)\n", hz,
                            int(steady), double(inRaise), double(limit));
                Check(inRaise <= limit, "hipfire P9 A2: a key change inside the hip raise (last frames included) moves a pose key by at most dt / 0.05 s (" +
                                            std::to_string(inRaise) + " vs " + std::to_string(limit) + ") at " + std::to_string(hz) + " Hz");
            }
        }

        OverlayMachine interrupted;
        OverlayInputs input;
        input.overlay = PoseOverlay::Pistol1H;
        input.wantsReady = true;
        for (int i = 0; i < hz; ++i)
            interrupted.Update(input, dt);
        input.overlay = PoseOverlay::Pistol2H;
        input.wantsToAim = true;
        for (int i = 0; i < hz / 10 + 1; ++i)
            interrupted.Update(input, dt);
        input.overlay = PoseOverlay::Pistol1H;
        input.wantsToAim = false;
        // Aiming has a 0.75 s minimum hold, followed by a 0.5 s exit fade.
        for (int i = 0; i < 2 * hz; ++i)
            interrupted.Update(input, dt);
        float totalWeight = 0.0f, oneHandWeight = 0.0f, twoHandWeight = 0.0f;
        for (int i = 0; i < interrupted.poses.count; ++i)
        {
            totalWeight += interrupted.poses.w[i];
            if (interrupted.poses.key[i] / 12 == static_cast<int>(PoseOverlay::Pistol1H))
                oneHandWeight += interrupted.poses.w[i];
            if (interrupted.poses.key[i] / 12 == static_cast<int>(PoseOverlay::Pistol2H))
                twoHandWeight += interrupted.poses.w[i];
        }
        Check(interrupted.aim == AimState::Ready && interrupted.AimingWeight() == 0.0f &&
              std::fabs(totalWeight - 1.0f) < 1e-4f && oneHandWeight > 0.999f && twoHandWeight < 0.001f,
              "interrupted 1H to 2H Aim returns entirely to 1H Ready at " + std::to_string(hz) +
              " Hz: aim=" + std::to_string(static_cast<int>(interrupted.aim)) +
              " aimW=" + std::to_string(interrupted.AimingWeight()) +
              " weights=" + std::to_string(totalWeight) + "/" + std::to_string(oneHandWeight) +
              "/" + std::to_string(twoHandWeight));
    }

    // 7. lean, spine yaw, aim grid
    {
        Additive left, right;
        Pose lp = *L, rp = *L;
        rp.b[4].r = math::QuatNormalize(math::QuatMul(AxisAngle(Vec3{1, 0, 0}, 10.0f), rp.b[4].r));
        lp.b[4].r = math::QuatNormalize(math::QuatMul(AxisAngle(Vec3{1, 0, 0}, -10.0f), lp.b[4].r));
        MakeAdditive(sk, *L, lp, true, left);
        MakeAdditive(sk, *L, rp, true, right);
        LeanState ls;
        Pose y = *L;
        ls.Apply(sk, y, left, right, 0.0f, true, 1.0f / 60.0f);
        Check(MaxPoseDiffDeg(sk, *L, y) < 1e-3f, "lean 0 = identity");
        for (int i = 0; i < 120; ++i)
        {
            y = *L;
            ls.Apply(sk, y, left, right, 1.0f, true, 1.0f / 60.0f);
        }
        Check(std::fabs(MaxPoseDiffDeg(sk, rp, y)) < 1e-2f, "lean +1 after 2 s = Right sample");
        Check(std::fabs(LeanAmount(1.0f, 1.0f) - 0.5f) < 1e-6f && std::fabs(LeanAmount(1.0f, 6.0f) - 1.0f) < 1e-6f &&
                  std::fabs(LeanAmount(0.5f, 3.5f) - 0.375f) < 1e-6f,
              "lean amount MapRange(2..5 m/s -> 0.5..1)");
        y = *L;
        ApplySpineYaw(sk, y, R.spineYawBones, R.spineYawCount, 60.0f * math::kPi / 180.0f);
        Quat a[kMaxBones], b[kMaxBones];
        MeshRotations(sk, *L, a);
        MeshRotations(sk, y, b);
        Check(std::fabs(AngleDeg(a[7], b[7]) - 60.0f) < 1e-2f && std::fabs(AngleDeg(a[4], b[4]) - 15.0f) < 1e-2f,
              "spine yaw: psi/4 per bone, Spine3 total psi");
        AimOffsetGrid g;
        float w[kAOMaxSamples];
        g.TargetWeights(22.5f, 45.0f, w);
        Check(std::fabs(w[1 * 5 + 2] - 0.25f) < 1e-6f && std::fabs(w[2 * 5 + 3] - 0.25f) < 1e-6f, "AO grid bilinear weights");
        Check(std::fabs(SweepTimeFromPitch(0.0f) * 30.0f - 15.0f) < 1e-4f, "sweep: pitch 0 = reference frame 15 of 31");
    }

    // 8. weapon map
    {
        WeaponOverlayMap wm;
        std::string err;
        Check(wm.Parse(kDefaultWeaponOverlayMap, &err), "default weapon map parses");
        Check(wm.Lookup("Wea_Handgun") == PoseOverlay::Pistol2H && wm.Lookup("Wea_Handgun", true) == PoseOverlay::Pistol1H &&
                  wm.Lookup("Wea_SniperRifle") == PoseOverlay::Rifle && wm.Lookup("Wea_HeavyMachineGun") == PoseOverlay::Rifle &&
                  wm.Lookup("GrenadeAim") == PoseOverlay::Pistol1H && wm.Lookup("Wea_Katana") == PoseOverlay::Game,
              "weapon map lookups (user decision table, unknown = Game)");
        Check(SelectOverlay(wm, "", false, true, false) == PoseOverlay::Injured &&
                  SelectOverlay(wm, "Wea_Rifle", true, true, false) == PoseOverlay::Rifle &&
                  SelectOverlay(wm, "Wea_Rifle", true, false, true) == PoseOverlay::Game,
              "overlay selection: injured / weapon / body carry");
        WeaponOverlayMap bad;
        Check(!bad.Parse("Wea_X = Bow\n", &err) && bad.count == 0, "weapon map rejects unknown overlay");
    }
}

static void TestData(const char* aPath, uint64_t aHash, const char* aTag)
{
    auto f = std::make_unique<OverlayFile>();
    std::string err;
    const bool ok = f->LoadFromFile(aPath, aHash, err);
    Check(ok, std::string(aTag) + ": load " + err);
    if (!ok)
        return;
    std::printf("%s: bones %d curves %d poses %d frames %d deltas %d overlays %d\n", aTag, f->NumBones(), f->NumCurves(),
                f->NumPoses(), f->NumFrames(), f->NumDeltas(), f->NumOverlays());
    std::string e2;
    auto g = std::make_unique<OverlayFile>();
    Check(!g->LoadFromFile(aPath, aHash ^ 1ull, e2), std::string(aTag) + ": wrong rig hash rejected");
    int mapped = 0;
    for (int k = 0; k < kCurveCount; ++k)
        mapped += f->CurveIndexOf(k) >= 0;
    std::printf("%s: layering curves mapped %d/%d\n", aTag, mapped, kCurveCount);
    Check(mapped == kCurveCount - 1 || mapped == kCurveCount, std::string(aTag) + ": layering curves present");
    const Skeleton& sk = f->Skel();
    // stored mesh rotations vs FK of the local pose
    float fkErr = 0.0f;
    auto p = std::make_unique<Pose>(), q = std::make_unique<Pose>();
    Quat ms[kMaxBones];
    for (int fr = 0; fr < f->NumFrames(); ++fr)
    {
        f->FramePose(fr, *p);
        MeshRotations(sk, *p, ms);
        for (int i = 0; i < sk.n; ++i)
            fkErr = std::max(fkErr, AngleDeg(ms[i], f->FrameMeshRot(fr, i)));
    }
    std::printf("%s: stored mesh rotation vs FK max %.5f deg over %d frames\n", aTag, fkErr, f->NumFrames());
    Check(fkErr < 0.01f, std::string(aTag) + ": FRAM mesh rotations = FK(local)");
    // base + delta -> full target pose
    int targets = 0;
    float worstMs = 0.0f, worstLs = 0.0f;
    auto add = std::make_unique<Additive>();
    for (int d = 0; d < f->NumDeltas(); ++d)
    {
        const auto& dr = f->DeltaAt(d);
        const auto& base = f->PoseAt(dr.basePose);
        const auto& tgt = f->PoseAt(dr.targetPose);
        float mMs = 0.0f, mLs = 0.0f;
        for (uint32_t k = 0; k < dr.numFrames; ++k)
        {
            f->FramePose(base.firstFrame + dr.baseFrame, *p);
            *q = *p;
            f->DeltaFrame(d, static_cast<int>(k), true, *add);
            ApplyAdditive(sk, *p, *add, 1.0f);
            f->DeltaFrame(d, static_cast<int>(k), false, *add);
            ApplyAdditive(sk, *q, *add, 1.0f);
            auto t = std::make_unique<Pose>();
            f->FramePose(tgt.firstFrame + dr.targetFirstFrame + k, *t);
            mMs = std::max(mMs, MaxPoseDiffDeg(sk, *p, *t));
            mLs = std::max(mLs, MaxPoseDiffDeg(sk, *q, *t));
        }
        std::printf("%s: delta %-28.*s frames %3u  A_MS err %.5f deg  A_LS err %.5f deg\n", aTag,
                    static_cast<int>(f->DeltaName(d).size()), f->DeltaName(d).data(), dr.numFrames, mMs, mLs);
        worstMs = std::max(worstMs, mMs);
        worstLs = std::max(worstLs, mLs);
        ++targets;
    }
    Check(targets >= 5, std::string(aTag) + ": >= 5 delta targets");
    Check(worstMs < 0.1f && worstLs < 0.1f, std::string(aTag) + ": base + delta reproduces full poses within 0.1 deg");
    // identity on real data: Default overlay and zero weights -> L
    const int pb = f->FindPose("Pose_Neutral_Stand_Idle"), pl = f->FindPose("Pose_Neutral_Stand_Move");
    const int op = f->FindOverlayPose(PoseOverlay::Rifle, AimState::Aiming, false, false);
    Check(pb >= 0 && pl >= 0 && op >= 0, std::string(aTag) + ": base / loco / Rifle aim poses found");
    if (pb >= 0 && pl >= 0 && op >= 0)
    {
        auto B = std::make_unique<Pose>(), L = std::make_unique<Pose>(), OP = std::make_unique<Pose>(),
             out = std::make_unique<Pose>();
        auto S = std::make_unique<LayerScratch>();
        f->FramePose(f->PoseAt(pb).firstFrame, *B);
        f->FramePose(f->PoseAt(pl).firstFrame, *L);
        f->FramePose(f->PoseAt(op).firstFrame, *OP);
        LayerInputs in;
        in.L = L.get();
        in.B = B.get();
        in.OP = OP.get();
        ComposeLayers(sk, f->Masks(), in, *S, *out);
        const float d0 = MaxPoseDiffDeg(sk, *L, *out);
        f->FrameCurves(op, f->PoseAt(op).firstFrame, in.curves);
        ComposeLayers(sk, f->Masks(), in, *S, *out);
        const float d1 = MaxPoseDiffDeg(sk, *L, *out);
        std::printf("%s: real data zero weights diff %.6f deg; Rifle aim curves (arm_r %.2f, spine %.2f) diff %.2f deg\n",
                    aTag, d0, in.curves[kArmR], in.curves[kSpine], d1);
        Check(d0 < 1e-3f, std::string(aTag) + ": identity on real poses");
        Check(d1 > 1.0f, std::string(aTag) + ": Rifle aim layer changes the pose");
        // L = B and pose curves: arm bones with arm mask 1 take OP mesh rotations (LS 0)
        in.L = B.get();
        ComposeLayers(sk, f->Masks(), in, *S, *out);
        Quat qo[kMaxBones], qp[kMaxBones];
        MeshRotations(sk, *out, qo);
        MeshRotations(sk, *OP, qp);
        float m = 0.0f;
        int n = 0;
        if (in.curves[kArmR] >= 0.999f && in.curves[kSpine] >= 0.999f)
            for (int i = 0; i < sk.n; ++i)
                if (f->Masks().armR.w[i] >= 0.999f && f->Masks().handR.w[i] <= 0.0f)
                {
                    m = std::max(m, AngleDeg(qo[i], qp[i]));
                    ++n;
                }
        std::printf("%s: L = B, right arm (%d bones) vs OP max %.5f deg\n", aTag, n, m);
        Check(n > 0 && m < 0.1f, std::string(aTag) + ": L = B reproduces the overlay arm");
    }
}

// ---- per-clip style weight by source family (fix-style 2026-10-13) --------------------------
static void TestStyleWeightsSynthetic()
{
    // 1. weight lookup: defaults, sidecar parse, clip -> family, fail-closed
    {
        const StyleWeightTable d;
        bool allOne = true;
        for (int st = 0; st < int(Style::Count); ++st)
            for (int fa = 0; fa < int(SourceFamily::Count); ++fa)
                allOne = allOne && d.Get(static_cast<Style>(st), static_cast<SourceFamily>(fa)) == 1.0f;
        Check(allOne, "style weights: every default (style x family) is 1");
        const char* text = "# test\nutpp-style-weights 1\n[weights]\nFeminine/other = 0.25\nMasculine/other = 0\n"
                           "[clips]\n* = primary\n! = other\nX_Walk = primary\r\nX_Run = other\n";
        StyleFamilyData sf;
        std::string err;
        Check(sf.Parse(text, &err) && sf.clips.size() == 2, "style weights: sidecar parses (" + err + ")");
        Check(sf.weights.Get(Style::Feminine, SourceFamily::Other) == 0.25f &&
                  sf.weights.Get(Style::Masculine, SourceFamily::Other) == 0.0f &&
                  sf.weights.Get(Style::Feminine, SourceFamily::Primary) == 1.0f,
              "style weights: overrides applied, other entries keep the defaults");
        Check(sf.Lookup("X_Walk", true) == SourceFamily::Primary && sf.Lookup("X_Run", false) == SourceFamily::Other &&
                  sf.Lookup("M_Neutral_Walk_Loop_F", false) == SourceFamily::Primary && sf.Lookup("X_Unknown", true) == SourceFamily::Other,
              "style weights: clip lookup (listed, unlisted Neutral, unlisted non-Neutral)");
        const char* names[] = {"A_SourcePack", "X_Run", "X_Walk", "Z_Added"};
        const bool ng[] = {false, true, true, true};
        std::vector<SourceFamily> fam;
        sf.BuildClipFamilies(4, [&](int c) { return std::string_view(names[c]); }, [&](int c) { return ng[c]; }, fam);
        Check(fam.size() == 4 && fam[0] == SourceFamily::Primary && fam[1] == SourceFamily::Other &&
                  fam[2] == SourceFamily::Primary && fam[3] == SourceFamily::Other,
              "style weights: per-clip family array for a database");
        const char* bad[] = {"[weights]\nFeminine/primary = 1\n",                                   // no header
                             "utpp-style-weights 1\n[weights]\nFeminine/male = 1\n",             // family
                             "utpp-style-weights 1\n[weights]\nFem/primary = 1\n",                // style
                             "utpp-style-weights 1\n[weights]\nFeminine/primary = 1.5\n",           // range
                             "utpp-style-weights 1\n[weights]\nFeminine/primary = 0.5x\n",          // number
                             "utpp-style-weights 1\n[clips]\nA = primary\nA = other\n",             // duplicate
                             "utpp-style-weights 1\nA = primary\n",                                 // no section
                             "utpp-style-weights 2\n"};                                          // version
        int failedClosed = 0;
        for (const char* b : bad)
        {
            StyleFamilyData x;
            x.clips.push_back({"junk", SourceFamily::Other});
            if (!x.Parse(b) && x.clips.empty() && x.weights.Get(Style::Feminine, SourceFamily::Primary) == 1.0f &&
                x.weights.Get(Style::Feminine, SourceFamily::Other) == 1.0f)
                ++failedClosed;
        }
        std::printf("style weights: %d / %d malformed sidecars refused (fail-closed to the defaults)\n", failedClosed,
                    int(sizeof(bad) / sizeof(bad[0])));
        Check(failedClosed == int(sizeof(bad) / sizeof(bad[0])), "style weights: malformed sidecars fail closed");
    }

    const Skeleton sk = MakeSkel();
    Regions R;
    BuildRegions(sk, [](int i) { return std::string_view(kNames[i]); }, R);
    std::mt19937 rng(11);
    auto L = std::make_unique<Pose>(), B = std::make_unique<Pose>(), OP = std::make_unique<Pose>(),
         BP = std::make_unique<Pose>(), OB = std::make_unique<Pose>(), out = std::make_unique<Pose>(),
         ref = std::make_unique<Pose>();
    auto S = std::make_unique<LayerScratch>();
    RandomPose(rng, sk.n, *L);
    RandomPose(rng, sk.n, *B);
    RandomPose(rng, sk.n, *OP);
    RandomPose(rng, sk.n, *BP);
    RandomPose(rng, sk.n, *OB);
    auto dP = std::make_unique<Additive>(), id = std::make_unique<Additive>(), dPb = std::make_unique<Additive>();
    MakeAdditive(sk, *BP, *OB, true, *dP);
    id->SetIdentity(sk.n, true);
    LayerInputs base;
    base.L = L.get();
    base.B = B.get();
    base.OP = OP.get();
    for (int c : {kLegs, kPelvis, kSpine, kSpineAdd, kHead, kHeadAdd, kArmL, kArmLAdd, kArmR, kArmRAdd})
        base.curves.Set(c, 1.0f);
    StyleWeightTable table; // the family mechanism is exercised with the synthetic weight 0 of Feminine on 'other'
    table.Set(Style::Feminine, SourceFamily::Other, 0.0f);

    // Separate right-arm style attenuation must keep locomotion additive and torso/left style.
    {
        LayerInputs full = base;
        full.dP = dP.get();
        full.curves.Set(kArmRLS, 1.0f);
        ComposeLayers(sk, R, full, *S, *ref);
        LayerInputs narrowed = full;
        narrowed.styleArmR = 0.0f;
        ComposeLayers(sk, R, narrowed, *S, *out);
        Pose noStyle;
        LayerInputs noStyleInput = narrowed;
        noStyleInput.dP = nullptr;
        ComposeLayers(sk, R, noStyleInput, *S, noStyle);
        Check(std::memcmp(&out->b[10], &noStyle.b[10], sizeof(Xf)) == 0,
              "idle pistol style: right arm keeps dLls with dP suppressed");
        Check(std::memcmp(&out->b[7], &ref->b[7], sizeof(Xf)) == 0 &&
              std::memcmp(&out->b[9], &ref->b[9], sizeof(Xf)) == 0,
              "idle pistol style: torso and left style unchanged");
        Check(std::memcmp(&out->b[10], &OP->b[10], sizeof(Xf)) != 0,
              "idle pistol style: locomotion additive not removed with style");
        LayerInputs defaults = full;
        defaults.styleArmL = defaults.styleArmR = 1.0f;
        ComposeLayers(sk, R, defaults, *S, *out);
        Check(std::memcmp(out->b, ref->b, sizeof(Xf) * sk.n) == 0,
              "idle pistol style: default weights preserve original output exactly");
        OverlayMachine machine;
        OverlayInputs input;
        input.style = Style::Feminine;
        input.overlay = PoseOverlay::Pistol1H;
        machine.Update(input, 0.0f);
        Check(IdlePistolRightArmStyleWeight(machine, true) == 0.0f &&
              IdlePistolRightArmStyleWeight(machine, false) == 1.0f,
              "idle pistol style: eligible female Relax only, ineligible default");
        machine.style = Style::Masculine;
        Check(IdlePistolRightArmStyleWeight(machine, true) == 1.0f,
              "idle pistol style: male unchanged");
        machine.style = Style::Feminine;
        for (int key : {PoseKey(PoseOverlay::Pistol1H, AimState::Ready, false, false),
                        PoseKey(PoseOverlay::Pistol1H, AimState::Aiming, false, false),
                        PoseKey(PoseOverlay::Pistol1H, AimState::Relaxed, false, true),
                        PoseKey(PoseOverlay::Pistol1H, AimState::Relaxed, true, false)})
        {
            machine.poses.Reset(key);
            Check(IdlePistolRightArmStyleWeight(machine, true) == 1.0f,
                  "idle pistol style: Ready/Aim/moving/crouch unchanged");
        }
        // male V policy (Style::Masculine, explicit policy style): same scope as the female policy, never mixed between styles
        {
            OverlayMachine male;
            OverlayInputs mi;
            mi.style = Style::Masculine;
            mi.overlay = PoseOverlay::Pistol1H;
            male.Update(mi, 0.0f);
            Check(IdlePistolRightArmStyleWeight(male, true, Style::Masculine) == 0.0f &&
                  IdlePistolRightArmStyleWeight(male, false, Style::Masculine) == 1.0f,
                  "idle pistol style (male policy): eligible male Relax only, ineligible default");
            Check(IdlePistolRightArmStyleWeight(male, true) == 1.0f,
                  "idle pistol style (male policy): the default (female) policy leaves a masculine machine unchanged");
            Check(IdlePistolRightArmStyleWeight(machine, true, Style::Masculine) == 1.0f,
                  "idle pistol style (male policy): does not apply to a feminine machine");
            for (int key : {PoseKey(PoseOverlay::Pistol1H, AimState::Ready, false, false),
                            PoseKey(PoseOverlay::Pistol1H, AimState::Aiming, false, false),
                            PoseKey(PoseOverlay::Pistol1H, AimState::Relaxed, false, true),
                            PoseKey(PoseOverlay::Pistol1H, AimState::Relaxed, true, false),
                            PoseKey(PoseOverlay::Pistol2H, AimState::Relaxed, false, false),
                            PoseKey(PoseOverlay::Rifle, AimState::Relaxed, false, false)})
            {
                male.poses.Reset(key);
                Check(IdlePistolRightArmStyleWeight(male, true, Style::Masculine) == 1.0f,
                      "idle pistol style (male policy): Ready/Aim/moving/crouch/2H/rifle unchanged");
                Curves c;
                for (int i = 0; i < kCurveCount; ++i)
                    c.Set(i, 1.0f);
                PreserveIdlePistolLowerBody(male, true, c, male.poses.WeightOf(PoseKey(PoseOverlay::Pistol1H, AimState::Relaxed, false, false)), Style::Masculine);
                Check(c[kLegs] == 1.0f && c[kPelvis] == 1.0f,
                      "idle pistol lower body (male policy): other pose contexts keep legs / pelvis");
            }
            male.poses.Reset(PoseKey(PoseOverlay::Pistol1H, AimState::Relaxed, false, false));
            Curves c;
            for (int i = 0; i < kCurveCount; ++i)
                c.Set(i, 1.0f);
            PreserveIdlePistolLowerBody(male, true, c, male.poses.WeightOf(PoseKey(PoseOverlay::Pistol1H, AimState::Relaxed, false, false)), Style::Masculine);
            Check(c[kLegs] == 0.0f && c[kPelvis] == 0.0f && c[kArmL] == 1.0f && c[kArmR] == 1.0f && c[kSpine] == 1.0f,
                  "idle pistol lower body (male policy): Relaxed 1H idle removes only legs / pelvis layering");
            Curves ineligible;
            for (int i = 0; i < kCurveCount; ++i)
                ineligible.Set(i, 1.0f);
            PreserveIdlePistolLowerBody(male, false, ineligible, male.poses.WeightOf(PoseKey(PoseOverlay::Pistol1H, AimState::Relaxed, false, false)), Style::Masculine);
            Check(ineligible[kLegs] == 1.0f && ineligible[kPelvis] == 1.0f, "idle pistol lower body (male policy): ineligible unchanged");
            Curves wrongStyle;
            for (int i = 0; i < kCurveCount; ++i)
                wrongStyle.Set(i, 1.0f);
            PreserveIdlePistolLowerBody(male, true, wrongStyle, 1.0f); // default policy style = Feminine on a masculine machine
            Check(wrongStyle[kLegs] == 1.0f && wrongStyle[kPelvis] == 1.0f, "idle pistol lower body: female policy leaves a masculine machine unchanged");
            male.poses.SetTarget(PoseKey(PoseOverlay::Pistol1H, AimState::Relaxed, false, true), 0.2f);
            male.poses.Update(0.1f);
            const float malePartial = IdlePistolRightArmStyleWeight(male, true, Style::Masculine);
            Check(malePartial > 0.0f && malePartial < 1.0f,
                  "idle pistol style (male policy): movement transition uses the existing smooth pose weight");
        }
        machine.poses.Reset(PoseKey(PoseOverlay::Pistol1H, AimState::Relaxed, false, false));
        machine.poses.SetTarget(PoseKey(PoseOverlay::Pistol1H, AimState::Relaxed, false, true), 0.2f);
        machine.poses.Update(0.1f);
        const float partial = IdlePistolRightArmStyleWeight(machine, true);
        Check(partial > 0.0f && partial < 1.0f,
              "idle pistol style: movement transition uses existing smooth pose weight");
    }

    // 2. identity when the weight is 0: dPWeight 0 == no style delta; Feminine on a zero-weight family == motion style
    {
        LayerInputs in = base;
        ComposeLayers(sk, R, in, *S, *ref); // dP null
        in.dP = dP.get();
        in.dPWeight = 0.0f;
        ComposeLayers(sk, R, in, *S, *out);
        float mt = 0.0f;
        const float d0 = MaxPoseDiffDeg(sk, *ref, *out, &mt);
        const bool exact0 = std::memcmp(ref->b, out->b, sizeof(Xf) * sk.n) == 0;
        in.dPWeight = 1.0f;
        Pose full;
        ComposeLayers(sk, R, in, *S, full);
        const float d1 = MaxPoseDiffDeg(sk, *ref, full);
        in.dPWeight = 0.5f;
        ComposeLayers(sk, R, in, *S, *out);
        const float dh = MaxPoseDiffDeg(sk, *ref, *out), dhf = MaxPoseDiffDeg(sk, full, *out);
        std::printf("style weight: dPWeight 0 vs no dP %.6f deg %.7f m; weight 1 vs none %.2f deg; 0.5 vs none %.2f, vs full %.2f\n",
                    d0, mt, d1, dh, dhf);
        Check(exact0 && d0 < 1e-3f,
              "style weight: dPWeight 0 = identity (no style delta), bit-exact");
        Check(d1 > 1.0f && dh > 0.1f && dhf > 0.1f, "style weight: weight 1 / 0.5 change the pose");
        OverlayMachine m;
        OverlayInputs oi;
        oi.style = Style::Feminine;
        oi.family = SourceFamily::Other;
        m.Update(oi, 0.0f);
        m.Update(oi, 1.0f / 60.0f);
        float w[3];
        m.StyleBlendWeights(table, w);
        const Additive* ds[2] = {id.get(), dP.get()};
        BlendAdditives(ds, w, 2, *dPb);
        dPb->meshSpace = true;
        LayerInputs bin = base;
        bin.dP = dPb.get();
        ComposeLayers(sk, R, bin, *S, *out);
        const float db = MaxPoseDiffDeg(sk, *ref, *out);
        std::printf("style weight: Feminine on a zero-weight family blend weights Neutral %.3f Feminine %.3f; vs Neutral style %.6f deg\n",
                    w[0], w[1], db);
        Check(w[1] == 0.0f && w[0] == 1.0f && m.StyleWeight(table) == 0.0f, "style weight: Feminine on a zero-weight family = 0");
        Check(db < 1e-3f, "style weight: Feminine on a zero-weight family composes the Neutral style (identity)");
        oi.family = SourceFamily::Primary;
        OverlayMachine g;
        g.Update(oi, 0.0f);
        g.StyleBlendWeights(table, w);
        Check(w[1] == 1.0f && w[0] == 0.0f && g.StyleWeight(table) == 1.0f, "style weight: Feminine on a Neutral clip = 1");
    }

    // 3. blended transition between families (motion -> other (weight 0) -> motion): weight continuous, bounded per frame
    for (const float fps : {30.0f, 60.0f, 144.0f})
    {
        const float dt = 1.0f / fps;
        OverlayMachine m;
        OverlayInputs oi;
        oi.style = Style::Feminine;
        oi.family = SourceFamily::Primary;
        float prevW = -1.0f, maxDw = 0.0f, maxStep = 0.0f, popStep = 0.0f, wAtHalf = -1.0f;
        bool monotonicDown = true, havePrev = false;
        Pose prevOut, prevPop;
        const int frames = int(2.0f * fps);
        for (int f = 0; f <= frames; ++f)
        {
            const float t = f * dt;
            oi.family = t >= 1.2f ? SourceFamily::Primary : t >= 0.5f ? SourceFamily::Other : SourceFamily::Primary;
            m.Update(oi, f == 0 ? 0.0f : dt);
            const float w = m.StyleWeight(table);
            if (t >= 0.5f && t < 1.2f && prevW >= 0.0f && w > prevW + 1e-6f)
                monotonicDown = false;
            if (wAtHalf < 0.0f && t >= 0.5f + 0.15f - 1e-4f)
                wAtHalf = w;
            if (prevW >= 0.0f)
                maxDw = std::max(maxDw, std::fabs(w - prevW));
            prevW = w;
            LayerInputs in = base;
            in.dP = dP.get();
            in.dPWeight = w;
            ComposeLayers(sk, R, in, *S, *out);
            LayerInputs pin = in; // the same frame with the family weight popped
            pin.dPWeight = table.Get(Style::Feminine, oi.family);
            Pose popOut;
            ComposeLayers(sk, R, pin, *S, popOut);
            if (havePrev)
            {
                maxStep = std::max(maxStep, MaxPoseDiffDeg(sk, prevOut, *out));
                popStep = std::max(popStep, MaxPoseDiffDeg(sk, prevPop, popOut));
            }
            prevOut = *out;
            prevPop = popOut;
            havePrev = true;
        }
        const float wBound = dt / m.times.familySwitch + 1e-4f, bound = 180.0f * dt / m.times.familySwitch;
        std::printf("family switch %.0f fps: max weight step %.4f (bound %.4f), weight 0.15 s after the switch %.3f, "
                    "max bone step %.3f deg (bound %.3f), popped %.3f deg, final weight %.3f\n",
                    fps, maxDw, wBound, wAtHalf, maxStep, bound, popStep, prevW);
        const std::string sf = " at " + std::to_string(int(fps)) + " fps";
        Check(maxDw <= wBound && monotonicDown, "family switch: weight ramps without a jump" + sf);
        Check(std::fabs(wAtHalf - 0.5f) <= 1.5f * dt / m.times.familySwitch + 0.01f, "family switch: half weight mid-blend" + sf);
        Check(maxStep <= bound && popStep > 2.0f * maxStep, "family switch: pose change bounded, pop larger" + sf);
        Check(std::fabs(prevW - 1.0f) < 1e-6f, "family switch: back on a Neutral clip = full style" + sf);
    }
    // blend time from the caller (e.g. the MM blend time) + reversal mid-blend stays continuous
    {
        OverlayMachine m;
        OverlayInputs oi;
        oi.style = Style::Feminine;
        m.Update(oi, 0.0f);
        oi.family = SourceFamily::Other;
        oi.familyBlendTime = 0.5f;
        const float dt = 1.0f / 60.0f;
        for (int f = 0; f < 15; ++f)
            m.Update(oi, dt);
        const float w1 = m.StyleWeight(table);
        oi.family = SourceFamily::Primary; // reversal at 0.25 s
        m.Update(oi, 0.0f);
        const float w2 = m.StyleWeight(table);
        std::printf("family switch: caller blend time 0.5 s -> weight %.3f at 0.25 s, %.3f right after the reversal\n", w1, w2);
        Check(std::fabs(w1 - 0.5f) < 0.02f, "family switch: caller blend time (familyBlendTime) used");
        Check(std::fabs(w2 - w1) < 1e-5f, "family switch: reversal mid-blend has no jump");
    }
}

static void TestStyleWeightsData(const char* aOvPath, const char* aTag)
{
    std::string path(aOvPath);
    const size_t sl = path.find_last_of("/\\");
    path = (sl == std::string::npos ? std::string() : path.substr(0, sl + 1)) + "utpp_style_weights_" + aTag + ".txt";
    std::string text;
    FILE* fp = nullptr;
#ifdef _MSC_VER
    if (fopen_s(&fp, path.c_str(), "rb") != 0)
        fp = nullptr;
#else
    fp = std::fopen(path.c_str(), "rb");
#endif
    if (fp)
    {
        char buf[4096];
        size_t n;
        while ((n = std::fread(buf, 1, sizeof(buf), fp)) > 0)
            text.append(buf, n);
        std::fclose(fp);
    }
    Check(!text.empty(), std::string(aTag) + ": style weight sidecar present (" + path + ")");
    if (text.empty())
        return;
    StyleFamilyData sf;
    std::string err;
    Check(sf.Parse(text, &err), std::string(aTag) + ": style weight sidecar parses (" + err + ")");
    bool allOne = true, noMotionListed = true;
    for (int st = 0; st < int(Style::Count); ++st)
        for (int fa = 0; fa < int(SourceFamily::Count); ++fa)
            allOne = allOne && sf.weights.Get(static_cast<Style>(st), static_cast<SourceFamily>(fa)) == 1.0f;
    for (const auto& e : sf.clips)
        noMotionListed = noMotionListed && e.family != SourceFamily::Primary; // the exporter lists the non-motion clips only
    std::printf("%s: style weight sidecar %zu listed clips, every weight 1: %d\n", aTag, sf.clips.size(), allOne ? 1 : 0);
    Check(allOne && noMotionListed && sf.Lookup("M_Neutral_Walk_Loop_F", false) == SourceFamily::Primary &&
              sf.Lookup("X_Unlisted_Added", true) == SourceFamily::Other,
          std::string(aTag) + ": every style weight 1 on every family, only non-motion set clips listed, unlisted Neutral = primary, unlisted non-Neutral = other");
}

static void TestHipFirePosePolicy()
{
    // HipFireNativeActionOwner(weaponUpper, recoilTransfer, transferRoute, hipFire, action, transferred, aimWeight, inFlight)
    Check(HipFireNativeActionOwner(false, true, true, true, 8, false, 1.0f),
          "hipfire: enabled recoil keeps first-burst fallback despite Neutral idle native-upper opt-out");
    Check(!HipFireNativeActionOwner(false, false, true, true, 8, false, 1.0f) &&
          !HipFireNativeActionOwner(false, true, true, true, 0, false, 1.0f),
          "hipfire: both options off respected and fallback limited to action8");
    // P9: a weapon without a transfer route (rifle, SMG, shotgun, pistol outside the Nue domain) has nothing to wait for
    Check(!HipFireNativeActionOwner(false, true, false, true, 8, false, 1.0f) &&
          !HipFireNativeActionOwner(false, true, false, true, 8, false, 0.5f) &&
          !HipFireNativeActionOwner(false, true, false, true, 8, false, std::numeric_limits<float>::quiet_NaN()),
          "hipfire P9: no transfer route keeps the authored Aim arms through the shot (no native FPP fallback)");
    Check(HipFireNativeActionOwner(true, true, false, true, 8, false, 1.0f) &&
          HipFireNativeActionOwner(true, false, false, true, 8, false, 1.0f) &&
          !HipFireNativeActionOwner(true, false, false, true, 0, false, 1.0f),
          "hipfire P9: the weaponUpper option keeps the native shot with or without a route (user choice)");
    Check(!HipFireNativeActionOwner(false, true, false, true, 8, false, 1.0f, true) &&
          !HipFireNativeActionOwner(false, true, true, true, 8, false, 1.0f, true),
          "hipfire P9: an episode in flight never yields to the native shot");
    Check(HipFireNeedsNativeAction(true, 8, false, 1.0f),
          "hipfire: first burst without transferred recoil keeps native action");
    Check(!HipFireNeedsNativeAction(true, 8, true, 0.5f),
          "hipfire: successful recoil transfer retains authored arms during partial Aim transition");
    Check(!HipFireNeedsNativeAction(true, 8, true, 1.0f) && !HipFireNeedsNativeAction(true, 0, false, 1.0f),
          "hipfire: proven complete Aim transfer or postshot hold uses authored visual pose");
    Check(CaptureNativeReadyReference(0, false, true),
          "hipfire: idle native pre-shot capture independent of authored Aim preparation");
    Check(CaptureNativeReadyReference(0, true, false) &&
          !CaptureNativeReadyReference(8, true, false) && !CaptureNativeReadyReference(8, true, true) &&
          !CaptureNativeReadyReference(2, true, true),
          "hipfire: post-shot authored warmup and Reload cannot contaminate native pre-shot base");
    HipFirePoseHold hold;
    Check(!hold.Active(), "hipfire: no stale firing pose on init");
    hold.Step(8, 0.016f, true);
    Check(hold.Active() && UseHipFirePose(false, 8, hold.Active(), false, true, true,
              PoseOverlay::Pistol1H, PoseOverlay::Pistol2H), "hipfire: Overture Ready1H uses resolved Aim2H without ADS");
    Check(UseHipFirePose(false, 8, true, false, true, true, PoseOverlay::Pistol1H, PoseOverlay::Pistol1H),
          "hipfire: manual or Slaughtomatic Aim1H preserved");
    Check(UseHipFirePose(false, 8, true, false, true, true, PoseOverlay::Rifle, PoseOverlay::Rifle),
          "hipfire: rifle remains rifle");
    Check(!UseHipFirePose(true, 8, true, false, true, true, PoseOverlay::Pistol1H, PoseOverlay::Pistol2H) &&
          !UseHipFirePose(false, 8, true, true, true, true, PoseOverlay::Pistol1H, PoseOverlay::Pistol2H) &&
          !UseHipFirePose(false, 8, true, false, false, true, PoseOverlay::Pistol1H, PoseOverlay::Pistol2H),
          "hipfire: real ADS, release and no-weapon do not synthesize visual override");
    hold.Step(0, kIsolatedHoldS * 0.5f, true);
    Check(hold.Active(), "hipfire: uses the isolated-shot hold after short shot pulse");
    hold.Step(0, kIsolatedHoldS, true);
    Check(!hold.Active(), "hipfire: firing pose expires");
    hold.Step(8, 0.016f, true); hold.Step(2, 0.016f, true);
    Check(!hold.Active() && !UseHipFirePose(false, 2, true, false, true, true, PoseOverlay::Pistol1H, PoseOverlay::Pistol2H),
          "hipfire: reload cancels visual firing pose");
    hold.Step(8, 0.016f, true); hold.Step(0, 0.016f, false);
    Check(!hold.Active(), "hipfire: unequip clears hold");
}

int main(int argc, char** argv)
{
    TestSynthetic();
    TestHipFirePosePolicy();
    if (argc > 1)
        TestData(argv[1], kRigIdentityWomanOv, "woman");
    if (argc > 2)
        TestData(argv[2], kRigIdentityManOv, "man");
    TestStyleWeightsSynthetic();
    if (argc > 1)
        TestStyleWeightsData(argv[1], "woman");
    if (argc > 2)
        TestStyleWeightsData(argv[2], "man");
    std::printf("overlay test: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
