// UTPP motion - dual wield left arm: the left arm takes the mirror image of the right arm across the body's mid plane.
// Header-only, no game types.
//
// With two weapons drawn the one-handed weapon pose holds the right weapon; the left arm of that pose is a free arm. This module
// replaces the left clavicle, upper arm, forearm, hand and the left weapon bone (WeaponLeft) with the reflection of their right
// counterparts in the chest frame (Spine3). The fingers and the other children of the left hand keep their own locals.
//
// Reflection: S = I - 2 n n^T with n the lateral axis of the chest, the plane passes through the midpoint of the two clavicle heads;
// both are measured once on the rest pose. A reflected frame S * R is not a rotation, so each bone pair carries the constant
// N = Rr_rest^T * S * Rl_rest (chest frame): the left target is S * Rr * N, the position mid + S * (pr - mid). For a rig whose rest
// pose is mirror-symmetric N is a signed permutation with determinant -1; anything else (no such bone, other parents, an asymmetric
// rest pose) leaves the mirror off (Valid() false, Apply does nothing).
// Apply works on game-local transforms in database bone order and edits only the left bones; weight 0 returns at once (bit-identical).
#pragma once

#include <cmath>
#include <cstdint>
#include <string_view>

#include "Inertializer.hpp"
#include "MMMath.hpp"

namespace utpp::mm::dualmirror
{
struct M3
{
    double m[3][3]{};
};
inline M3 Identity3()
{
    M3 r;
    r.m[0][0] = r.m[1][1] = r.m[2][2] = 1.0;
    return r;
}
inline M3 Mul(const M3& a, const M3& b)
{
    M3 r;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            r.m[i][j] = a.m[i][0] * b.m[0][j] + a.m[i][1] * b.m[1][j] + a.m[i][2] * b.m[2][j];
    return r;
}
inline M3 Transpose(const M3& a)
{
    M3 r;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            r.m[i][j] = a.m[j][i];
    return r;
}
struct V3d
{
    double x = 0.0, y = 0.0, z = 0.0;
};
inline V3d Apply3(const M3& a, V3d v)
{
    return {a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z, a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z,
            a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z};
}
inline V3d Sub(V3d a, V3d b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline V3d Add(V3d a, V3d b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline V3d ToD(math::Vec3 v) { return {v.x, v.y, v.z}; }
inline math::Vec3 ToF(V3d v) { return {static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)}; }

inline M3 FromQuat(const math::Quat& aQ)
{
    const double x = aQ.x, y = aQ.y, z = aQ.z, w = aQ.w;
    const double n = x * x + y * y + z * z + w * w;
    const double s = n > 0.0 ? 2.0 / n : 0.0;
    M3 r;
    r.m[0][0] = 1.0 - s * (y * y + z * z);
    r.m[0][1] = s * (x * y - z * w);
    r.m[0][2] = s * (x * z + y * w);
    r.m[1][0] = s * (x * y + z * w);
    r.m[1][1] = 1.0 - s * (x * x + z * z);
    r.m[1][2] = s * (y * z - x * w);
    r.m[2][0] = s * (x * z - y * w);
    r.m[2][1] = s * (y * z + x * w);
    r.m[2][2] = 1.0 - s * (x * x + y * y);
    return r;
}
inline math::Quat ToQuat(const M3& a)
{
    const auto& m = a.m;
    const double tr = m[0][0] + m[1][1] + m[2][2];
    double x, y, z, w;
    if (tr > 0.0)
    {
        const double s = std::sqrt(tr + 1.0) * 2.0;
        w = 0.25 * s, x = (m[2][1] - m[1][2]) / s, y = (m[0][2] - m[2][0]) / s, z = (m[1][0] - m[0][1]) / s;
    }
    else if (m[0][0] > m[1][1] && m[0][0] > m[2][2])
    {
        const double s = std::sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0;
        w = (m[2][1] - m[1][2]) / s, x = 0.25 * s, y = (m[0][1] + m[1][0]) / s, z = (m[0][2] + m[2][0]) / s;
    }
    else if (m[1][1] > m[2][2])
    {
        const double s = std::sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0;
        w = (m[0][2] - m[2][0]) / s, x = (m[0][1] + m[1][0]) / s, y = 0.25 * s, z = (m[1][2] + m[2][1]) / s;
    }
    else
    {
        const double s = std::sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0;
        w = (m[1][0] - m[0][1]) / s, x = (m[0][2] + m[2][0]) / s, y = (m[1][2] + m[2][1]) / s, z = 0.25 * s;
    }
    return math::QuatNormalize(math::Quat{static_cast<float>(x), static_cast<float>(y), static_cast<float>(z), static_cast<float>(w)});
}

// Rigid transform in double precision (chest frame).
struct Xd
{
    M3 r = Identity3();
    V3d t;
};
inline Xd Compose(const Xd& aParent, const Xd& aLocal) { return {Mul(aParent.r, aLocal.r), Add(Apply3(aParent.r, aLocal.t), aParent.t)}; }
inline Xd Inverse(const Xd& a)
{
    const M3 rt = Transpose(a.r);
    const V3d t = Apply3(rt, a.t);
    return {rt, {-t.x, -t.y, -t.z}};
}
inline Xd FromBone(const BoneTransform& b) { return {FromQuat(b.q), ToD(b.p)}; }

// Mirrored pairs, parent first. Index 0 is the clavicle (child of the chest); WeaponLeft / WeaponRight hang on the hands.
inline constexpr int kPairs = 5;
inline constexpr std::string_view kLeftNames[kPairs] = {"LeftShoulder", "LeftArm", "LeftForeArm", "LeftHand", "WeaponLeft"};
inline constexpr std::string_view kRightNames[kPairs] = {"RightShoulder", "RightArm", "RightForeArm", "RightHand", "WeaponRight"};
inline constexpr std::string_view kChestName = "Spine3";
// Largest |N - round(N)| accepted as a mirror-symmetric rest pose (both shipped rigs: 6e-4).
inline constexpr double kMaxRestAsymmetry = 0.01;

struct Mirror
{
    bool ok = false;
    int32_t chest = -1;
    int32_t left[kPairs] = {-1, -1, -1, -1, -1};
    int32_t right[kPairs] = {-1, -1, -1, -1, -1};
    M3 s = Identity3();   // reflection across the mid plane (chest frame)
    V3d mid;              // a point of the mid plane (chest frame)
    V3d lateral;          // plane normal (chest frame), for diagnostics
    M3 n[kPairs];         // per pair: Rr_rest^T S Rl_rest
    double restAsymmetry = 0.0; // largest |N - round(N)| over the pairs (diagnostics, tests)
    V3d up;               // chest up axis inside the mid plane (chest frame): the turn axis of the twin stance
    V3d fwd;              // chest forward inside the mid plane (chest frame), up x fwd ~ lateral
    bool Valid() const { return ok; }
};

// aRest: game-local rest transforms (database RIGR) in bone order; aParent(i) -> parent index (-1 root).
template<class NameFn, class ParentFn>
inline Mirror Build(uint32_t aCount, NameFn aName, ParentFn aParent, const BoneTransform* aRest)
{
    Mirror m;
    if (aRest == nullptr)
        return m;
    auto find = [&](std::string_view s) -> int32_t {
        for (uint32_t i = 0; i < aCount; ++i)
            if (std::string_view(aName(i)) == s)
                return static_cast<int32_t>(i);
        return -1;
    };
    m.chest = find(kChestName);
    if (m.chest < 0)
        return m;
    for (int k = 0; k < kPairs; ++k)
    {
        m.left[k] = find(kLeftNames[k]);
        m.right[k] = find(kRightNames[k]);
        if (m.left[k] < 0 || m.right[k] < 0)
            return m;
        const int32_t wantL = k == 0 ? m.chest : m.left[k - 1];
        const int32_t wantR = k == 0 ? m.chest : m.right[k - 1];
        if (static_cast<int32_t>(aParent(static_cast<uint32_t>(m.left[k]))) != wantL ||
            static_cast<int32_t>(aParent(static_cast<uint32_t>(m.right[k]))) != wantR)
            return m;
    }
    // rest chains in the chest frame
    Xd restL[kPairs], restR[kPairs];
    for (int k = 0; k < kPairs; ++k)
    {
        const Xd pl = k == 0 ? Xd{} : restL[k - 1];
        const Xd pr = k == 0 ? Xd{} : restR[k - 1];
        restL[k] = Compose(pl, FromBone(aRest[m.left[k]]));
        restR[k] = Compose(pr, FromBone(aRest[m.right[k]]));
    }
    const V3d d = Sub(restL[0].t, restR[0].t);
    const double len = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    if (!(len > 1e-4))
        return m;
    m.lateral = {d.x / len, d.y / len, d.z / len};
    m.mid = {0.5 * (restL[0].t.x + restR[0].t.x), 0.5 * (restL[0].t.y + restR[0].t.y), 0.5 * (restL[0].t.z + restR[0].t.z)};
    const double nv[3] = {m.lateral.x, m.lateral.y, m.lateral.z};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            m.s.m[i][j] = (i == j ? 1.0 : 0.0) - 2.0 * nv[i] * nv[j];
    double worst = 0.0;
    for (int k = 0; k < kPairs; ++k)
    {
        m.n[k] = Mul(Mul(Transpose(restR[k].r), m.s), restL[k].r);
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                worst = std::fmax(worst, std::fabs(m.n[k].m[i][j] - std::round(m.n[k].m[i][j])));
    }
    m.restAsymmetry = worst;
    // chest up = the chest bone's local x axis without its lateral part; forward = the mid plane direction that points away from
    // the back (local -y). A chest frame without such axes leaves the mirror off.
    const double ux = 1.0 - m.lateral.x * m.lateral.x, uy = -m.lateral.x * m.lateral.y, uz = -m.lateral.x * m.lateral.z;
    const double ul = std::sqrt(ux * ux + uy * uy + uz * uz);
    if (!(ul > 0.9))
        return m;
    m.up = {ux / ul, uy / ul, uz / ul};
    V3d f = {m.up.y * m.lateral.z - m.up.z * m.lateral.y, m.up.z * m.lateral.x - m.up.x * m.lateral.z,
             m.up.x * m.lateral.y - m.up.y * m.lateral.x};
    if (f.y > 0.0)
        f = {-f.x, -f.y, -f.z};
    m.fwd = f;
    m.ok = worst <= kMaxRestAsymmetry;
    return m;
}

// Mirrored left locals (targets) of the pose, without blending. aOut[k] = local transform for m.left[k].
// aOwnClavicle (0..1): share of the pose's own left clavicle (local) instead of the mirrored one. Below the clavicle the bones keep
// the mirrored rotations in the chest frame with their own bone offsets, so nothing stretches. The aim
// offset of the one-handed weapon pulls the right clavicle forward when aiming down; its mirror would pull the left one forward
// too and narrow the shoulders, so the twin stance keeps the left clavicle of the pose (Mirror::Apply callers pass 0: the plain
// mirror, bit for bit as before).
inline bool MirroredLeftLocals(const Mirror& aM, const BoneTransform* aPose, BoneTransform (&aOut)[kPairs], float aOwnClavicle = 0.0f)
{
    if (!aM.ok || aPose == nullptr)
        return false;
    const float own = std::isfinite(aOwnClavicle) ? (aOwnClavicle < 0.0f ? 0.0f : (aOwnClavicle > 1.0f ? 1.0f : aOwnClavicle)) : 0.0f;
    Xd right = {}, leftParent = {};
    for (int k = 0; k < kPairs; ++k)
    {
        right = Compose(right, FromBone(aPose[aM.right[k]])); // chest frame (k = 0: parent = chest)
        Xd target;
        target.r = Mul(Mul(aM.s, right.r), aM.n[k]);
        target.t = Add(aM.mid, Apply3(aM.s, Sub(right.t, aM.mid)));
        Xd local = Compose(Inverse(leftParent), target);
        aOut[k].q = ToQuat(local.r);
        aOut[k].p = ToF(local.t);
        if (own > 0.0f)
        {
            const BoneTransform& ownL = aPose[aM.left[k]];
            if (k == 0)
            {
                aOut[k].q = math::QuatNlerpShortest(aOut[k].q, ownL.q, own);
                aOut[k].p = math::Lerp(aOut[k].p, ownL.p, own);
            }
            else
                aOut[k].p = ownL.p; // bone offset in the parent frame (the rig's own; the mirrored one differs by < 1 mm)
            local = FromBone(aOut[k]);
            if (k > 0) // the mirrored rotation in the chest frame under the actual parent
            {
                local.r = Mul(Transpose(leftParent.r), target.r);
                aOut[k].q = ToQuat(local.r);
            }
        }
        if (!math::IsFinite(aOut[k].q) || !math::IsFinite(aOut[k].p))
            return false;
        leftParent = own > 0.0f ? Compose(leftParent, FromBone(aOut[k])) : target;
    }
    return true;
}

// Blend the left chain towards the mirror of the right chain with weight aW (0..1). aW <= 0: the pose is not touched.
// Returns true when the pose was edited.
inline bool Apply(const Mirror& aM, BoneTransform* aPose, float aW)
{
    if (!(aW > 0.0f) || !aM.ok || aPose == nullptr)
        return false;
    BoneTransform target[kPairs];
    if (!MirroredLeftLocals(aM, aPose, target))
        return false;
    const float w = aW < 1.0f ? aW : 1.0f;
    for (int k = 0; k < kPairs; ++k)
    {
        BoneTransform& b = aPose[aM.left[k]];
        if (w >= 1.0f)
            b = target[k];
        else
        {
            b.q = math::QuatNlerpShortest(b.q, target[k].q, w);
            b.p = math::Lerp(b.p, target[k].p, w);
        }
    }
    return true;
}

// Blend time of the mirror weight (see Ramp). 0.65 s: the left hand travels up to ~0.6 m between the free arm of the weapon pose
// and the mirror (ready pose), so the smoothstep peak stays near 1.5 m/s (<= 2.5 cm per frame at 60 Hz).
inline constexpr float kBlendSeconds = 0.65f;

// Twin stance. The aim / fire pose of the one-handed weapon is a side stance: the right weapon points along the arm, far to the
// side of the chest, and its mirror image would point the left weapon to the other side. With two weapons drawn the right upper
// arm is turned about the shoulder joint, around the chest up axis, until the right barrel lies in the chest mid plane direction
// (no sideways component); its elevation is kept, so the aim offset pitch, the elevation of the recoil and everything below the
// shoulder stay as they are. The mirror of that arm then points the left weapon the same way: the two barrels are parallel.
// That turn alone ends with the right hand ~5 cm from the mid plane (the weapons ~8 cm apart). The arm is then turned out about
// the same axis until the hand is kTwinHandHalfWidth from the mid plane, and the hand is turned back by the same angle at the
// wrist, so the barrel keeps its direction (spread <= kTwinMaxSpreadDeg).
// A barrel near the up axis (aiming steeply up) has no reliable heading: the turn fades out with the barrel's forward component
// (kTwinHeadingFade), so the arm never swings round when the heading flips.
// The turn is scaled by the caller's weight (aiming weight x mirror weight), so the ready pose is untouched.
inline constexpr int kBarrelAxis = 1;               // the barrel is the weapon bone's local +y
inline constexpr double kTwinMaxTurnDeg = 80.0;     // larger turns are clamped (the stance needs ~45 deg)
inline constexpr double kTwinHandHalfWidth = 0.15;  // m: hands ~30 cm apart, a little narrower than the shoulders
inline constexpr double kTwinMaxSpreadDeg = 25.0;
inline constexpr double kTwinHeadingFade = 0.3;     // forward component of the barrel at which the turn has its full weight

// Turn of the right upper arm (radians, about Mirror::up) that brings the right barrel into the mid plane direction; 0 when the
// barrel is (nearly) along the up axis. aPose: game-local transforms.
inline double TwinTurn(const Mirror& aM, const BoneTransform* aPose)
{
    if (!aM.ok || aPose == nullptr)
        return 0.0;
    Xd x = {};
    for (int k = 0; k < kPairs; ++k)
        x = Compose(x, FromBone(aPose[aM.right[k]]));
    const V3d b = {x.r.m[0][kBarrelAxis], x.r.m[1][kBarrelAxis], x.r.m[2][kBarrelAxis]};
    const double bu = b.x * aM.up.x + b.y * aM.up.y + b.z * aM.up.z;
    const V3d h = {b.x - bu * aM.up.x, b.y - bu * aM.up.y, b.z - bu * aM.up.z};
    const double hl = std::sqrt(h.x * h.x + h.y * h.y + h.z * h.z);
    if (!(hl > 0.05))
        return 0.0;
    // signed angle from h to fwd about up
    const V3d c = {h.y * aM.fwd.z - h.z * aM.fwd.y, h.z * aM.fwd.x - h.x * aM.fwd.z, h.x * aM.fwd.y - h.y * aM.fwd.x};
    const double sn = (c.x * aM.up.x + c.y * aM.up.y + c.z * aM.up.z) / hl;
    const double cs = (h.x * aM.fwd.x + h.y * aM.fwd.y + h.z * aM.fwd.z) / hl;
    const double lim = kTwinMaxTurnDeg * 3.14159265358979323846 / 180.0;
    const double ang = std::atan2(sn, cs);
    const double fade = std::fmin(1.0, std::fmax(0.0, (b.x * aM.fwd.x + b.y * aM.fwd.y + b.z * aM.fwd.z) / kTwinHeadingFade));
    return (ang > lim ? lim : (ang < -lim ? -lim : ang)) * fade * fade * (3.0 - 2.0 * fade);
}

// Rotation matrix about a unit axis.
inline M3 AxisRotation(V3d aAxis, double aRad)
{
    const double c = std::cos(aRad), s = std::sin(aRad), t = 1.0 - c;
    const double x = aAxis.x, y = aAxis.y, z = aAxis.z;
    M3 r;
    r.m[0][0] = t * x * x + c, r.m[0][1] = t * x * y - s * z, r.m[0][2] = t * x * z + s * y;
    r.m[1][0] = t * x * y + s * z, r.m[1][1] = t * y * y + c, r.m[1][2] = t * y * z - s * x;
    r.m[2][0] = t * x * z - s * y, r.m[2][1] = t * y * z + s * x, r.m[2][2] = t * z * z + c;
    return r;
}

// Turns the right upper arm by aWeight x TwinTurn about the chest up axis through the shoulder joint, then spreads it out to the
// hand half width with the wrist turned back (see kTwinHandHalfWidth). Only the RightArm and RightHand locals change; the
// clavicle is not touched. aWeight <= 0: the pose is not touched. Returns the barrel turn in degrees (aSpreadDeg: the spread).
inline double ApplyTwinRight(const Mirror& aM, BoneTransform* aPose, float aWeight, double* aSpreadDeg = nullptr)
{
    if (aSpreadDeg)
        *aSpreadDeg = 0.0;
    if (!(aWeight > 0.0f) || !aM.ok || aPose == nullptr)
        return 0.0;
    const double w = aWeight < 1.0f ? aWeight : 1.0;
    const double ang = w * TwinTurn(aM, aPose);
    const Xd clav = FromBone(aPose[aM.right[0]]);
    Xd arm = Compose(clav, FromBone(aPose[aM.right[1]]));
    const Xd foreL = FromBone(aPose[aM.right[2]]);
    const Xd handL = FromBone(aPose[aM.right[3]]);
    arm.r = Mul(AxisRotation(aM.up, ang), arm.r); // pivot = the joint itself: its position does not move
    // the hand (chest frame) after the turn: its rotation is kept through the spread
    const Xd hand = Compose(Compose(arm, foreL), handL);
    const V3d v = Sub(hand.t, arm.t);
    const double vu = v.x * aM.up.x + v.y * aM.up.y + v.z * aM.up.z;
    const V3d vh = {v.x - vu * aM.up.x, v.y - vu * aM.up.y, v.z - vu * aM.up.z};
    const V3d uxv = {aM.up.y * vh.z - aM.up.z * vh.y, aM.up.z * vh.x - aM.up.x * vh.z, aM.up.x * vh.y - aM.up.y * vh.x};
    const auto lat = [&](V3d p) { return p.x * aM.lateral.x + p.y * aM.lateral.y + p.z * aM.lateral.z; };
    // lateral of the hand after a turn s: lat(joint - mid) + A cos s + B sin s; the right side is negative
    const double A = lat(vh), B = lat(uxv), C = -kTwinHandHalfWidth - lat(Sub(arm.t, aM.mid));
    const double R = std::sqrt(A * A + B * B);
    double spread = 0.0;
    if (R > 1e-4)
    {
        const double phi = std::atan2(B, A), c = std::fmax(-1.0, std::fmin(1.0, C / R));
        const double s1 = phi + std::acos(c), s2 = phi - std::acos(c);
        auto wrap = [](double x) { return std::atan2(std::sin(x), std::cos(x)); };
        spread = std::fabs(wrap(s1)) < std::fabs(wrap(s2)) ? wrap(s1) : wrap(s2);
        const double lim = kTwinMaxSpreadDeg * 3.14159265358979323846 / 180.0;
        spread = w * (spread > lim ? lim : (spread < -lim ? -lim : spread));
    }
    if (ang == 0.0 && spread == 0.0)
        return 0.0;
    arm.r = Mul(AxisRotation(aM.up, spread), arm.r);
    const Xd armLocal = Compose(Inverse(clav), arm);
    const Xd fore = Compose(arm, foreL);
    Xd handLocal = Compose(Inverse(fore), Xd{hand.r, Compose(fore, handL).t});
    const math::Quat qa = ToQuat(armLocal.r), qh = ToQuat(handLocal.r);
    if (!math::IsFinite(qa) || !math::IsFinite(qh))
        return 0.0;
    aPose[aM.right[1]].q = qa;
    aPose[aM.right[3]].q = qh;
    if (aSpreadDeg)
        *aSpreadDeg = spread * 180.0 / 3.14159265358979323846;
    return ang * 180.0 / 3.14159265358979323846;
}

// Left arm follower. The left hand (chest frame) moves at most kFollowSpeed x dt per frame (2.4 cm at 60 Hz), whatever drives it:
//  - the mirror target is followed with that speed limit, so a fast raise of the right arm (hip fire raises in 0.1 s) or a quick
//    stance on / off does not throw the left hand; a target that moves slower is followed exactly;
//  - the shown weight (ramp x right arm layer x reload envelope) is followed with the rest of the same budget: it changes by at
//    most (budget left) / (distance between the left hand of the pose and of the mirror) per frame, so a reload envelope that
//    takes the arm back to the pose of the frame (and gives it again) moves the hand within the limit too.
// The pose's own left arm under weight 0 is never limited. Shown weight 0: the pose is not touched and the follower is reset.
inline constexpr float kFollowSpeed = 1.44f; // m/s = 2.4 cm per frame at 60 Hz
struct Follow
{
    bool valid = false;
    float w = 0.0f; // shown weight (limited)
    BoneTransform local[kPairs];
    void Reset() { valid = false; w = 0.0f; }
};
inline V3d LeftHandInChest(const BoneTransform (&aL)[kPairs])
{
    Xd x = {};
    for (int k = 0; k < 4; ++k)
        x = Compose(x, FromBone(aL[k]));
    return x.t;
}
inline double Norm(V3d v) { return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }

// Like Apply, with the follower. aW: the wanted shown weight; aOwnClavicle: see MirroredLeftLocals (the twin stance passes its
// weight). aMirrorSource: the pose the mirror target is built from (nullptr = aPose). Recoil per arm: while the right weapon's kick
// is on the right arm the caller passes a copy of the pose with the right arm as it was before that kick, so the left arm does not
// copy it. The shown left chain is always blended over aPose. Returns true when the pose was edited.
// aTarget: a ready target of the left chain (kPairs locals, e.g. TwinTarget) instead of the mirror; nullptr = the mirror.
inline bool ApplyFollow(const Mirror& aM, Follow& aF, BoneTransform* aPose, float aW, float aDt, float aOwnClavicle = 0.0f,
                        const BoneTransform* aMirrorSource = nullptr, const BoneTransform* aTarget = nullptr)
{
    const float want = std::isfinite(aW) ? (aW < 0.0f ? 0.0f : (aW > 1.0f ? 1.0f : aW)) : 0.0f;
    if (!aM.ok || aPose == nullptr || (!(want > 0.0f) && !(aF.w > 0.0f)))
    {
        aF.Reset();
        return false;
    }
    BoneTransform target[kPairs];
    if (aTarget != nullptr)
    {
        for (int k = 0; k < kPairs; ++k)
            target[k] = aTarget[k];
    }
    else if (!MirroredLeftLocals(aM, aMirrorSource ? aMirrorSource : aPose, target, aOwnClavicle))
    {
        aF.Reset();
        return false;
    }
    const float dt = std::isfinite(aDt) && aDt > 0.0f ? std::fmin(aDt, 0.1f) : 0.0f;
    const double cap = static_cast<double>(kFollowSpeed) * dt;
    double step = 0.0; // hand step of the follower this frame
    if (!aF.valid)
    {
        for (int k = 0; k < kPairs; ++k)
            aF.local[k] = target[k];
        aF.valid = true;
        aF.w = 0.0f; // the weight starts from the pose (no jump on the first frame)
    }
    else
    {
        const double dist = Norm(Sub(LeftHandInChest(target), LeftHandInChest(aF.local)));
        float f = dist > cap ? static_cast<float>(dist > 0.0 ? cap / dist : 0.0) : 1.0f;
        for (int pass = 0; pass < 6; ++pass)
        {
            // a rotation blend does not move the hand on a straight line: halve the fraction until the hand step is in the cap
            BoneTransform next[kPairs];
            for (int k = 0; k < kPairs; ++k)
            {
                next[k].q = f >= 1.0f ? target[k].q : math::QuatNlerpShortest(aF.local[k].q, target[k].q, f);
                next[k].p = f >= 1.0f ? target[k].p : math::Lerp(aF.local[k].p, target[k].p, f);
            }
            const double e = Norm(Sub(LeftHandInChest(next), LeftHandInChest(aF.local)));
            if (f >= 1.0f || e <= cap * 1.0001)
            {
                for (int k = 0; k < kPairs; ++k)
                    aF.local[k] = next[k];
                step = e;
                break;
            }
            f *= 0.5f;
        }
    }
    // the shown weight with the rest of the budget
    BoneTransform own[kPairs];
    for (int k = 0; k < kPairs; ++k)
        own[k] = aPose[aM.left[k]];
    const auto shownHand = [&](float aWeight) {
        BoneTransform l[kPairs];
        for (int k = 0; k < kPairs; ++k)
        {
            l[k].q = math::QuatNlerpShortest(own[k].q, aF.local[k].q, aWeight);
            l[k].p = math::Lerp(own[k].p, aF.local[k].p, aWeight);
        }
        return LeftHandInChest(l);
    };
    const double gapHand = Norm(Sub(LeftHandInChest(aF.local), LeftHandInChest(own)));
    const double budget = std::fmax(0.0, cap - static_cast<double>(aF.w) * step);
    float dw = gapHand > 1e-6 ? static_cast<float>(budget / gapHand) : 1.0f;
    float nw = want > aF.w ? std::fmin(want, aF.w + dw) : std::fmax(want, aF.w - dw);
    if (nw != aF.w && gapHand > 1e-6)
    {
        // the rotation blend is not linear in the weight: halve the change until the hand step is in the budget
        const V3d from = shownHand(aF.w);
        for (int pass = 0; pass < 6 && Norm(Sub(shownHand(nw), from)) > budget * 1.0001; ++pass)
        {
            dw *= 0.5f;
            nw = want > aF.w ? std::fmin(want, aF.w + dw) : std::fmax(want, aF.w - dw);
        }
    }
    aF.w = nw;
    const float w = aF.w;
    if (!(w > 0.0f))
    {
        aF.Reset();
        return false;
    }
    for (int k = 0; k < kPairs; ++k)
    {
        BoneTransform& b = aPose[aM.left[k]];
        if (w >= 1.0f)
            b = aF.local[k];
        else
        {
            b.q = math::QuatNlerpShortest(b.q, aF.local[k].q, w);
            b.p = math::Lerp(b.p, aF.local[k].p, w);
        }
    }
    return true;
}

// Twin stance in model space. The one-handed aim / fire stance is bladed (the chest turned ~45 deg to the left), so a mirror across the
// chest mid plane points the left weapon far to the left of the right one. In the stance the right arm keeps its own pose and the left
// chain gets its own target:
//  - orientation: the right weapon reflected across the VERTICAL plane that contains the right barrel (model up = +z), then the pair
//    constant N (S_v R_R N): the left barrel is parallel to the right barrel;
//  - position: kTwinGap to the left of the right weapon (horizontal, perpendicular to the barrel), at its height; when the left hand
//    would be out of reach of the left shoulder it slides back along the barrel until it is kTwinReach of the arm length from the
//    shoulder joint (the bladed chest puts the left shoulder ~26 cm behind the right one), so the left weapon stays on the line
//    parallel to the right barrel (level aim: same height; aiming up / down it sits lower / higher on that line); when even that line
//    never comes into reach the hand is pulled toward the shoulder (TwinInfo::pulled);
//  - the left arm is solved for that hand with a two-bone IK, elbow pole down and out; clavicle, bone offsets and WeaponLeft local are
//    the ones of the mirror target passed in (the twin passes the pose's own left clavicle).
// A barrel near vertical has no reliable heading: the result carries a fade (full at a horizontal barrel component >= kTwinHeadingFade).
inline constexpr double kTwinGap = 0.25;   // m between the weapons, sideways
inline constexpr double kTwinReach = 0.90; // fraction of upper arm + forearm (the arm IK's soft reach starts at 0.95)
struct TwinInfo
{
    double fade = 0.0;        // 0..1 heading fade
    double slideBack = 0.0;   // m the left weapon slid back along the barrel to be in reach
    double barrelDeg = 0.0;   // angle between the left target barrel and the right barrel
    bool pulled = false;      // the barrel line never came into reach: the hand was pulled toward the shoulder (height / gap not kept)
};
inline V3d Cross(V3d a, V3d b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline double Dot(V3d a, V3d b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3d Scale(V3d a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline bool Unit(V3d& a)
{
    const double l = std::sqrt(Dot(a, a));
    if (!(l > 1e-9) || !std::isfinite(l))
        return false;
    a = Scale(a, 1.0 / l);
    return true;
}
// Rotation that takes the frame (aFrom, side aFromSide) to (aTo, aToSide): x = bone direction, y = side without its x part.
inline bool FrameRotation(V3d aFrom, V3d aFromSide, V3d aTo, V3d aToSide, M3& aOut)
{
    V3d x0 = aFrom, x1 = aTo;
    if (!Unit(x0) || !Unit(x1))
        return false;
    V3d y0 = Sub(aFromSide, Scale(x0, Dot(aFromSide, x0))), y1 = Sub(aToSide, Scale(x1, Dot(aToSide, x1)));
    if (!Unit(y0) || !Unit(y1))
        return false;
    const V3d z0 = Cross(x0, y0), z1 = Cross(x1, y1);
    M3 f0, f1;
    const V3d c0[3] = {x0, y0, z0}, c1[3] = {x1, y1, z1};
    for (int j = 0; j < 3; ++j)
    {
        f0.m[0][j] = c0[j].x, f0.m[1][j] = c0[j].y, f0.m[2][j] = c0[j].z;
        f1.m[0][j] = c1[j].x, f1.m[1][j] = c1[j].y, f1.m[2][j] = c1[j].z;
    }
    aOut = Mul(f1, Transpose(f0));
    return true;
}
// aSrc: the pose the right arm is read from (game-local, bone order); aChestModel: model rotation of the chest bone; aMirror: the
// mirror target of the left chain (MirroredLeftLocals). aOut: the twin left chain (locals). false: no twin (keep the mirror).
inline bool TwinLeftChain(const Mirror& aM, const BoneTransform* aSrc, const M3& aChestModel, const BoneTransform (&aMirror)[kPairs],
                          BoneTransform (&aOut)[kPairs], TwinInfo* aInfo = nullptr)
{
    if (aInfo)
        *aInfo = TwinInfo{};
    if (!aM.ok || aSrc == nullptr)
        return false;
    Xd wr = {};
    for (int k = 0; k < kPairs; ++k)
        wr = Compose(wr, FromBone(aSrc[aM.right[k]]));
    V3d up = Apply3(Transpose(aChestModel), V3d{0.0, 0.0, 1.0}); // model up in the chest frame
    if (!Unit(up))
        return false;
    const V3d d = {wr.r.m[0][kBarrelAxis], wr.r.m[1][kBarrelAxis], wr.r.m[2][kBarrelAxis]};
    V3d u = Sub(d, Scale(up, Dot(d, up))); // horizontal barrel direction
    const double hl = std::sqrt(Dot(u, u));
    if (!(hl > 0.05) || !Unit(u))
        return false;
    V3d left = Cross(up, u);
    if (!Unit(left))
        return false;
    if (Dot(left, aM.lateral) < 0.0)
        left = Scale(left, -1.0);
    // reflection across the vertical plane through the barrel: its normal is the horizontal sideways axis
    M3 sv;
    const double nv[3] = {left.x, left.y, left.z};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            sv.m[i][j] = (i == j ? 1.0 : 0.0) - 2.0 * nv[i] * nv[j];
    const M3 rwl = Mul(Mul(sv, wr.r), aM.n[kPairs - 1]);
    const V3d dl = {rwl.m[0][kBarrelAxis], rwl.m[1][kBarrelAxis], rwl.m[2][kBarrelAxis]};
    if (!(Dot(dl, d) > 0.99)) // the pair constant must keep the barrel axis (else the left gun would point back)
        return false;
    // left chain of the mirror target in the chest frame
    const Xd clav = FromBone(aMirror[0]);
    const Xd up0 = Compose(clav, FromBone(aMirror[1]));
    const Xd fo0 = Compose(up0, FromBone(aMirror[2]));
    const Xd ha0 = Compose(fo0, FromBone(aMirror[3]));
    const Xd wl = FromBone(aMirror[4]); // weapon in the hand
    const V3d a = up0.t;
    const double la = Norm(ToD(aMirror[2].p)), lb = Norm(ToD(aMirror[3].p));
    if (!(la > 1e-3) || !(lb > 1e-3))
        return false;
    // hand target
    const M3 rh = Mul(rwl, Transpose(wl.r));
    V3d pw = Add(wr.t, Scale(left, kTwinGap));
    V3d hand = Sub(pw, Apply3(rh, wl.t));
    const double reach = kTwinReach * (la + lb);
    const V3d v = Sub(hand, a);
    double slide = 0.0;
    bool pulled = false;
    V3d along = d; // slide along the barrel itself: the left weapon stays on the line parallel to the right barrel
    if (!Unit(along))
        return false;
    if (Norm(v) > reach)
    {
        const double vu = Dot(v, along), disc = vu * vu - Dot(v, v) + reach * reach;
        slide = disc >= 0.0 ? vu - std::sqrt(disc) : vu;
        V3d moved = Sub(v, Scale(along, slide));
        if (Norm(moved) > reach * (1.0 + 1e-6)) // the barrel line never comes into reach: pull the hand in toward the shoulder
        {
            V3d dir = moved;
            if (!Unit(dir))
                return false;
            moved = Scale(dir, reach);
            pulled = true;
        }
        const V3d shift = Sub(Add(a, moved), hand);
        hand = Add(hand, shift);
        pw = Add(pw, shift);
    }
    // two-bone IK, pole down and out
    const V3d toT = Sub(hand, a);
    const double dist = Norm(toT);
    const double dmin = std::fabs(la - lb) + 1e-3;
    const double dd = std::fmin(std::fmax(dist, dmin), la + lb - 1e-4);
    V3d dir = toT;
    if (!Unit(dir))
        return false;
    V3d pole = Add(Scale(up, -1.0), Scale(left, 0.6));
    pole = Sub(pole, Scale(dir, Dot(pole, dir)));
    if (!Unit(pole))
        return false;
    const double x = (la * la - lb * lb + dd * dd) / (2.0 * dd);
    const double h = std::sqrt(std::fmax(la * la - x * x, 0.0));
    const V3d e = Add(a, Add(Scale(dir, x), Scale(pole, h)));
    const V3d hNew = Add(a, Scale(dir, dd));
    M3 qUp, qLo;
    if (!FrameRotation(Sub(fo0.t, a), Sub(ha0.t, a), Sub(e, a), Sub(hNew, a), qUp) ||
        !FrameRotation(Sub(ha0.t, fo0.t), Sub(a, fo0.t), Sub(hNew, e), Sub(a, e), qLo))
        return false;
    const M3 upN = Mul(qUp, up0.r), foN = Mul(qLo, fo0.r);
    for (int k = 0; k < kPairs; ++k)
        aOut[k] = aMirror[k];
    aOut[1].q = ToQuat(Mul(Transpose(clav.r), upN));
    aOut[2].q = ToQuat(Mul(Transpose(upN), foN));
    aOut[3].q = ToQuat(Mul(Transpose(foN), rh));
    for (int k = 1; k <= 3; ++k)
        if (!math::IsFinite(aOut[k].q))
            return false;
    if (aInfo)
    {
        const double f = std::fmin(1.0, hl / kTwinHeadingFade);
        aInfo->fade = f * f * (3.0 - 2.0 * f);
        aInfo->slideBack = slide;
        aInfo->pulled = pulled;
        aInfo->barrelDeg = std::acos(std::fmax(-1.0, std::fmin(1.0, Dot(dl, d)))) * 180.0 / 3.14159265358979323846;
    }
    return true;
}
// Model rotation of the chest: walks the parents of the chest bone (aParent(i) -> parent index, -1 root).
template<class ParentFn>
inline M3 ChestModelRotation(const Mirror& aM, const BoneTransform* aPose, ParentFn aParent)
{
    M3 r = Identity3();
    for (int32_t i = aM.chest, guard = 0; i >= 0 && guard < 256; i = static_cast<int32_t>(aParent(static_cast<uint32_t>(i))), ++guard)
        r = Mul(FromQuat(aPose[i].q), r);
    return r;
}
// Twin target of the left chain: the mirror target (own clavicle by aWeight) blended toward TwinLeftChain by aWeight x heading fade.
// aWeight <= 0: the plain mirror target. Returns false when no target exists (no mirror).
inline bool TwinTarget(const Mirror& aM, const BoneTransform* aSrc, const M3& aChestModel, float aWeight, BoneTransform (&aOut)[kPairs],
                       TwinInfo* aInfo = nullptr)
{
    if (aInfo)
        *aInfo = TwinInfo{};
    const float w = std::isfinite(aWeight) ? (aWeight < 0.0f ? 0.0f : (aWeight > 1.0f ? 1.0f : aWeight)) : 0.0f;
    if (!MirroredLeftLocals(aM, aSrc, aOut, w))
        return false;
    if (!(w > 0.0f))
        return true;
    BoneTransform twin[kPairs];
    TwinInfo info;
    if (!TwinLeftChain(aM, aSrc, aChestModel, aOut, twin, &info))
        return true;
    const float tw = static_cast<float>(w * info.fade);
    for (int k = 1; k <= 3; ++k)
        aOut[k].q = math::QuatNlerpShortest(aOut[k].q, twin[k].q, tw);
    if (aInfo)
        *aInfo = info;
    return true;
}

// Recoil per arm. A recoil clip is a motion of the weapon in its own frame (rotation aR, translation aT). The left weapon of the
// mirror is L = S R N (N = the weapon pair's constant), so the mirror image of the right weapon moved by D is
// S (R D) N = L (N^T D N): the same kick, mirrored, in the left weapon's own frame. Position: N^T t.
inline bool MirrorWeaponMotion(const Mirror& aM, const math::Quat& aR, math::Vec3 aT, math::Quat& aOutR, math::Vec3& aOutT)
{
    if (!aM.ok)
        return false;
    const M3& n = aM.n[kPairs - 1];
    const M3 nt = Transpose(n);
    aOutR = ToQuat(Mul(Mul(nt, FromQuat(aR)), n));
    aOutT = ToF(Apply3(nt, ToD(aT)));
    return math::IsFinite(aOutR) && math::IsFinite(aOutT);
}

// Weight of the mirror, stepped once per frame: rises / falls over kBlendSeconds (smoothstep shaped) while the condition holds /
// is gone. The shown weight multiplies the shaped ramp by the caller's other factors.
// aGap: the ramp was not stepped for a while (skipped frames, game pause). Still wanted: it keeps its value (no pop of the arm when
// the frames resume); no longer wanted: it is 0 at once (two weapons were put away during the gap, nothing to fade from).
struct Ramp
{
    float u = 0.0f;
    float Step(bool aOn, float aDt, bool aGap = false)
    {
        if (aGap && !aOn)
            u = 0.0f;
        const float step = (std::isfinite(aDt) && aDt > 0.0f ? aDt : 0.0f) / kBlendSeconds;
        u = aOn ? std::fmin(1.0f, u + step) : std::fmax(0.0f, u - step);
        return Shaped();
    }
    float Shaped() const { return u * u * (3.0f - 2.0f * u); }
    void Reset() { u = 0.0f; }
};

// Shown weight: ramp x the right arm layer weight of the weapon pose (a right arm that follows locomotion is not copied) x the
// reload envelope complement (the reload keeps its own behaviour).
inline float ShownWeight(float aRamp, float aRightArmLayer, float aReloadUpper)
{
    auto c01 = [](float v) { return std::isfinite(v) ? (v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v)) : 0.0f; };
    return c01(aRamp) * c01(aRightArmLayer) * (1.0f - c01(aReloadUpper));
}
} // namespace utpp::mm::dualmirror
