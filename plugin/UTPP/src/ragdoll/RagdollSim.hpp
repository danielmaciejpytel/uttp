#pragma once
// UTPP ragdoll S0 prototype (design stage, NOT built into the shipped DLL, no game dependency, header only).
//
// What this is: a reduced-skeleton PBD / Verlet body (17 particles, 16 bone-length links, torso / pelvis rigidity links, soft
// joint-angle limits as min/max distance links) with a ground plane, friction, fixed-step integration and an ALS-style
// "angular drive" substitute (pose-target spring on the bone vectors). It answers: cost per step, stability, settle height.
// It is the cheap baseline of the design (2026-11-02); the production candidate is
// the rigid-body XPBD variant (15 bodies with orientation + joint anchors), whose per-step cost is estimated at 3-5x this one.
//
// Coordinates: world metres, +Z up (same as the game). Particle index order = kP below.
// Not here (see report): world queries (script supplies ground z + normal, later vehicle OBB), pose-out (local Hips / bone records
// for PoseSnapshot), get-up selection, gate / ownership.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace utpp::ragdoll
{
struct V3
{
    float x = 0, y = 0, z = 0;
};
inline V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline V3 operator*(V3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline float Dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline float Len(V3 a) { return std::sqrt(Dot(a, a)); }

// 17 joint particles of the reduced V skeleton (names = game bones at the particle).
enum P : int
{
    kHips, kChest /*Spine3*/, kNeck, kHead,
    kLSh, kRSh /*arm root (LeftArm / RightArm)*/, kLEl, kREl /*ForeArm*/, kLWr, kRWr /*Hand*/,
    kLHip, kRHip /*UpLeg*/, kLKn, kRKn /*Leg*/, kLAn, kRAn /*Foot*/, kLToe, kRToe /*ToeBase*/,
    kCount
};

struct Link
{
    int a, b;
    float rest;
    float stiffness; // 0..1 per iteration
    int kind;        // 0 = bone (equality), 1 = min distance, 2 = max distance
};

struct Params
{
    float dt = 1.0f / 120.0f;
    int iterations = 8;
    float gravity = -9.81f;
    float airDamping = 0.15f;      // 1/s, linear
    float groundZ = 0.0f;          // ground plane height (script probe under the pelvis in the real thing)
    V3 groundN{0, 0, 1};           // ground normal (TraceResult.normal)
    float friction = 0.55f;        // tangential velocity kept per contact step (Coulomb-ish, position based)
    float restitution = 0.0f;
    float limbRadius = 0.07f;      // capsule radius against the plane (limbs, hands, feet)
    float torsoRadius = 0.12f;     // pelvis / chest / neck particles (a lying pelvis sits ~0.13 m high in the get-up clips)
    float driveStiffness = 0.0f;   // 0 = passive; >0 = ALS-style pose-target spring (active ragdoll), see Drive()
};

class Sim
{
public:
    // Local rest dimensions of the woman rig measured from M_ragdoll_getup_stand_B frame 0 (bone local lengths, m).
    // spine chain Hips->Spine3 0.327, Spine3->Neck 0.174, Neck->Head 0.136, thigh 0.431, shin 0.469, foot 0.172,
    // upper arm 0.290, forearm 0.275, clavicle+shoulder 0.168 + 0.174.
    void Build(float aScale = 1.0f)
    {
        const float s = aScale;
        const float hipW = 0.09f * s, shW = 0.20f * s;
        m_n = kCount;
        m_rest[kHips] = {0, 0, 0};
        m_rest[kChest] = {0, 0, 0.327f * s};
        m_rest[kNeck] = {0, 0, 0.501f * s};
        m_rest[kHead] = {0, 0, 0.637f * s};
        m_rest[kLSh] = {0, shW, 0.50f * s};
        m_rest[kRSh] = {0, -shW, 0.50f * s};
        m_rest[kLEl] = {0, shW + 0.0f, 0.50f * s - 0.29f * s};
        m_rest[kREl] = {0, -shW - 0.0f, 0.50f * s - 0.29f * s};
        m_rest[kLWr] = {0, shW, 0.50f * s - 0.565f * s};
        m_rest[kRWr] = {0, -shW, 0.50f * s - 0.565f * s};
        m_rest[kLHip] = {0, hipW, -0.02f * s};
        m_rest[kRHip] = {0, -hipW, -0.02f * s};
        m_rest[kLKn] = {0, hipW, -0.02f * s - 0.431f * s};
        m_rest[kRKn] = {0, -hipW, -0.02f * s - 0.431f * s};
        m_rest[kLAn] = {0, hipW, -0.02f * s - 0.900f * s};
        m_rest[kRAn] = {0, -hipW, -0.02f * s - 0.900f * s};
        m_rest[kLToe] = {0.172f * s, hipW, -0.02f * s - 0.900f * s};
        m_rest[kRToe] = {0.172f * s, -hipW, -0.02f * s - 0.900f * s};
        for (int i = 0; i < m_n; ++i)
            m_invMass[i] = 1.0f;
        m_invMass[kHips] = 1.0f / 3.0f; // heavier core: pelvis / chest
        m_invMass[kChest] = 1.0f / 3.0f;
        m_invMass[kHead] = 1.0f / 1.5f;
        m_links = {};
        m_nl = 0;
        auto bone = [&](int a, int b) { AddLink(a, b, Len(m_rest[a] - m_rest[b]), 1.0f, 0); };
        bone(kHips, kChest); bone(kChest, kNeck); bone(kNeck, kHead);
        bone(kChest, kLSh); bone(kChest, kRSh); bone(kLSh, kLEl); bone(kRSh, kREl); bone(kLEl, kLWr); bone(kREl, kRWr);
        bone(kHips, kLHip); bone(kHips, kRHip); bone(kLHip, kLKn); bone(kRHip, kRKn); bone(kLKn, kLAn); bone(kRKn, kRAn);
        bone(kLAn, kLToe); bone(kRAn, kRToe);
        // rigid pelvis triangle / chest triangle / torso cross links (no twist of the trunk, no folding of the shoulders)
        auto rigid = [&](int a, int b, float k = 1.0f) { AddLink(a, b, Len(m_rest[a] - m_rest[b]), k, 0); };
        rigid(kLHip, kRHip); rigid(kLSh, kRSh); rigid(kHips, kNeck, 0.9f); rigid(kHips, kLSh, 0.8f); rigid(kHips, kRSh, 0.8f);
        rigid(kChest, kLHip, 0.8f); rigid(kChest, kRHip, 0.8f); rigid(kNeck, kLSh); rigid(kNeck, kRSh);
        rigid(kLAn, kLKn, 1.0f);
        // joint limits as distance bounds (knee: no hyperextension / full fold, elbow likewise, neck / spine bend)
        const float thigh = 0.431f * s, shin = 0.469f * s, ua = 0.29f * s, fa = 0.275f * s;
        AddLink(kLHip, kLAn, thigh + shin, 0.9f, 2); AddLink(kRHip, kRAn, thigh + shin, 0.9f, 2);
        AddLink(kLHip, kLAn, 0.16f * s, 0.7f, 1);   AddLink(kRHip, kRAn, 0.16f * s, 0.7f, 1);
        AddLink(kLSh, kLWr, ua + fa, 0.9f, 2);      AddLink(kRSh, kRWr, ua + fa, 0.9f, 2);
        AddLink(kLSh, kLWr, 0.14f * s, 0.7f, 1);    AddLink(kRSh, kRWr, 0.14f * s, 0.7f, 1);
        AddLink(kHips, kHead, 0.30f * s, 0.7f, 1);  // spine + neck cannot fold the head into the pelvis
        AddLink(kLHip, kRKn, 0.10f * s, 0.6f, 1);   AddLink(kRHip, kLKn, 0.10f * s, 0.6f, 1); // thighs do not cross through each other
    }

    // Initial state: world positions of the particles from the animated pose (world = pelvisPos + R_yaw * rest-pose offsets in the
    // real thing: here the caller supplies m_pos directly via SetPose) plus a uniform velocity (vehicle impulse) and optional spin.
    void SetPose(const V3* aWorld) { for (int i = 0; i < m_n; ++i) { m_pos[i] = aWorld[i]; m_prev[i] = aWorld[i]; } }
    void SetUniformVelocity(V3 v)
    {
        for (int i = 0; i < m_n; ++i)
            m_prev[i] = m_pos[i] - v * m_p.dt;
    }
    void SetVelocityAt(int i, V3 v) { m_prev[i] = m_pos[i] - v * m_p.dt; }
    void SetTarget(const V3* aWorld) { for (int i = 0; i < m_n; ++i) m_target[i] = aWorld[i]; } // for Drive()
    void SetParams(const Params& p) { m_p = p; }
    const Params& GetParams() const { return m_p; }

    void Step()
    {
        const float dt = m_p.dt;
        const float damp = std::max(0.0f, 1.0f - m_p.airDamping * dt);
        for (int i = 0; i < m_n; ++i)
        {
            V3 v = (m_pos[i] - m_prev[i]) * damp;
            m_prev[i] = m_pos[i];
            m_pos[i] = m_pos[i] + v; // v = displacement of the last step (Verlet); gravity as g*dt^2 below
            m_pos[i].z += m_p.gravity * dt * dt;
        }
        for (int it = 0; it < m_p.iterations; ++it)
        {
            for (int l = 0; l < m_nl; ++l)
                SolveLink(m_links[l]);
            if (m_p.driveStiffness > 0.0f)
                Drive();
            Collide();
        }
        ++m_steps;
    }

    // ALS-style angular drive substitute: pull each particle towards the pose-target (animation) with a spring whose strength grows
    // with speed (motion: MapRange(|v| 0..1000 -> 0..25000)); here a flat stiffness per iteration. Bone lengths stay intact.
    void Drive()
    {
        const float k = std::min(1.0f, m_p.driveStiffness);
        for (int i = 0; i < m_n; ++i)
            m_pos[i] = m_pos[i] + (m_target[i] - m_pos[i]) * (k * m_invMass[i] / 3.0f);
    }

    const V3& Pos(int i) const { return m_pos[i]; }
    V3 Velocity(int i) const { return (m_pos[i] - m_prev[i]) * (1.0f / m_p.dt); }
    uint64_t Steps() const { return m_steps; }
    int Count() const { return m_n; }
    const V3& Rest(int i) const { return m_rest[i]; }
    float MaxBoneError() const
    {
        float e = 0;
        for (int l = 0; l < m_nl; ++l)
            if (m_links[l].kind == 0 && m_links[l].stiffness >= 1.0f)
                e = std::max(e, std::fabs(Len(m_pos[m_links[l].a] - m_pos[m_links[l].b]) - m_links[l].rest));
        return e;
    }
    // Deepest penetration seen BEFORE the contact projection (any iteration) since the last ResetPenetration(); this is the real
    // stability measure. (A post-step probe is zero by construction because Collide() projects every particle out.)
    float MaxPrePenetration() const { return m_prePen; }
    void ResetPenetration() { m_prePen = 0.0f; }
    float Radius(int i) const { return (i == kHips || i == kChest || i == kNeck) ? m_p.torsoRadius : (i == kHead ? 0.10f : m_p.limbRadius); }
    float KineticSpeedMax() const
    {
        float m = 0;
        for (int i = 0; i < m_n; ++i)
            m = std::max(m, Len(Velocity(i)));
        return m;
    }

private:
    void AddLink(int a, int b, float rest, float k, int kind)
    {
        if (m_nl < static_cast<int>(m_links.size()))
            m_links[m_nl++] = {a, b, rest, k, kind};
    }
    void SolveLink(const Link& L)
    {
        V3 d = m_pos[L.b] - m_pos[L.a];
        const float len = Len(d);
        if (len < 1e-6f)
            return;
        if ((L.kind == 1 && len >= L.rest) || (L.kind == 2 && len <= L.rest))
            return;
        const float wA = m_invMass[L.a], wB = m_invMass[L.b];
        const float w = wA + wB;
        if (w <= 0.0f)
            return;
        const float c = (len - L.rest) / (len * w) * L.stiffness;
        m_pos[L.a] = m_pos[L.a] + d * (c * wA);
        m_pos[L.b] = m_pos[L.b] - d * (c * wB);
    }
    void Collide()
    {
        for (int i = 0; i < m_n; ++i)
        {
            const float h = Dot(m_pos[i] - V3{0, 0, m_p.groundZ}, m_p.groundN);
            const float pen = Radius(i) - h;
            if (pen <= 0.0f)
                continue;
            m_prePen = std::max(m_prePen, pen);
            m_pos[i] = m_pos[i] + m_p.groundN * pen;
            // position based friction: scale the tangential displacement of this step
            V3 disp = m_pos[i] - m_prev[i];
            const float nd = Dot(disp, m_p.groundN);
            V3 tang = disp - m_p.groundN * nd;
            m_prev[i] = m_prev[i] + tang * (1.0f - m_p.friction);
            // restitution 0: the normal velocity is dropped by the position projection itself (prev stays put)
        }
    }

    Params m_p{};
    int m_n = 0, m_nl = 0;
    std::array<V3, kCount> m_pos{}, m_prev{}, m_rest{}, m_target{};
    std::array<float, kCount> m_invMass{};
    std::array<Link, 48> m_links{};
    uint64_t m_steps = 0;
    float m_prePen = 0.0f;
};
} // namespace utpp::ragdoll
