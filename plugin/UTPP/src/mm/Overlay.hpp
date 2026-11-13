// UTPP motion matching runtime - ALS overlay layering (port stage 10.6). Header-only, no game types.
//
// 1. ComposeLayers: ABP_LayerBlending on the CP rig:
//      dLms = D_MS(B -> L), dLls = D_LS(B -> L), dP = D_MS(BP -> OB) (style layer, baked per stance in .utppov)
//      Legs   = Blend(L, A_MS(A_MS(OP, dLms, 1), dP, 1), c[Layering_Legs])
//      Pelvis = Blend(L, A_MS(A_MS(OP, dLms, 1), dP, 1), c[Layering_Pelvis])
//      Spine  = Blend(L, A_MS(A_MS(OP, dLms, aS), dP, aS), c[Layering_Spine]),  aS = c[Layering_Spine_Add]
//      Head   = Blend(L, A_MS(A_MS(OP, dLms, aH), dP, aH), c[Layering_Head]),   aH = c[Layering_Head_Add]
//      Arm_X  = Blend(L, A_MS(A_LS(OP, dLls, a), dP, a), c[Layering_Arm_X]),    a  = c[Layering_Arm_X_Add]
//      P1 = LBB_MS(Legs, Pelvis); P2 = LBB_MS(P1, Spine); P3 = LBB_MS(P2, Head);
//      P4 = LBB_MS(P3, ArmL) * (1 - floor(c[Arm_L_LS])), P5 = LBB_LS(P4, ArmL) * c[Arm_L_LS]; same right (P6, P7);
//      P8 = LBB_LS(P7, OP) on the hand masks with c[Layering_Hand_L/R].
//    Curves out = Override merge L < OB < OP (absent curves keep the lower value); Default overlay = no OP = L.
// 2. OverlayMachine: style + pose overlay + Relaxed/Ready/Aiming (transition table) + stand/crouch + idle/move
//    with blended switches (motion pops the overlay in one frame; UTPP crossfades: pose ~0.2 s, style ~0.3 s).
// 3. WeaponOverlayMap: weapon class -> overlay as data, text "Class = Overlay[, Alt]".
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>

#include "Additive.hpp"

namespace utpp::mm::ov
{
// ---- layering curves ------------------------------------------------------------------------------------------
enum Curve : int
{
    kArmL,
    kArmLAdd,
    kArmLLS,
    kArmR,
    kArmRAdd,
    kArmRLS,
    kHandL,
    kHandR,
    kHead,
    kHeadAdd,
    kLegs,
    kPelvis,
    kSpine,
    kSpineAdd,
    kHandIKL,
    kHandIKR,
    kSpineRotation,
    kCurveCount // 17
};

inline const char* CurveName(int aCurve) // lower case, as in utppclip / .utppov CURV
{
    static const char* kNames[kCurveCount] = {
        "layering_arm_l", "layering_arm_l_add", "layering_arm_l_ls", "layering_arm_r", "layering_arm_r_add",
        "layering_arm_r_ls", "layering_hand_l", "layering_hand_r", "layering_head", "layering_head_add",
        "layering_legs", "layering_pelvis", "layering_spine", "layering_spine_add", "enable_handik_l",
        "enable_handik_r", "enable_spinerotation"};
    return aCurve >= 0 && aCurve < kCurveCount ? kNames[aCurve] : "";
}

struct Curves
{
    float v[kCurveCount]{};
    uint32_t present = 0;

    float operator[](int i) const { return v[i]; }
    void Set(int i, float a)
    {
        v[i] = a;
        present |= 1u << i;
    }
    // LayeredBoneBlend Override merge: every curve present in aSrc replaces the value.
    void Override(const Curves& aSrc)
    {
        for (int i = 0; i < kCurveCount; ++i)
            if (aSrc.present & (1u << i))
                v[i] = aSrc.v[i];
        present |= aSrc.present;
    }
    // Weighted accumulation for crossfades (absent = 0, presence = union).
    void Accumulate(const Curves& aSrc, float aW)
    {
        for (int i = 0; i < kCurveCount; ++i)
            if (aSrc.present & (1u << i))
                v[i] += aSrc.v[i] * aW;
        present |= aSrc.present;
    }
};

// ---- region masks (MASK section of .utppov, or built from bone names) -----------------------------------------
struct Regions
{
    BoneWeights pelvis, spine, head, armL, armR, handL, handR;
    int spineYawBones[8]{};
    int spineYawCount = 0;
};

struct RegionNames // CP rig defaults; [W] spine ramp 0.25..1.0 over Spine..Spine3
{
    const char* pelvisRoot = "Hips";
    const char* pelvisExclude[2] = {"LeftUpLeg", "RightUpLeg"};
    const char* spineRoot = "Spine";
    int spineBlendDepth = 4;
    const char* headRoot = "Neck";
    const char* armL = "LeftShoulder";
    const char* armR = "RightShoulder";
    const char* handL[6] = {"LeftInHandThumb", "LeftHandIndex1", "LeftHandMiddle1", "LeftHandRing1", "LeftHandPinky1",
                            "WeaponLeft"}; // UE hand masks include ik_hand_l -> CP WeaponLeft (as the baked .utppov masks)
    const char* handR[6] = {"RightInHandThumb", "RightHandIndex1", "RightHandMiddle1", "RightHandRing1",
                            "RightHandPinky1", "WeaponRight"}; // ik_hand_r -> CP WeaponRight
    const char* spineYaw[4] = {"Spine", "Spine1", "Spine2", "Spine3"};
};

template <class NameFn> // NameFn(int bone) -> std::string_view
inline void BuildRegions(const Skeleton& aSk, NameFn aName, Regions& aR, const RegionNames& aN = {})
{
    auto find = [&](const char* s) -> int {
        for (int i = 0; i < aSk.n; ++i)
            if (aName(i) == std::string_view(s))
                return i;
        return -1;
    };
    BoneWeights* all[] = {&aR.pelvis, &aR.spine, &aR.head, &aR.armL, &aR.armR, &aR.handL, &aR.handR};
    for (BoneWeights* w : all)
        w->Clear(aSk.n);
    AddBranch(aSk, find(aN.pelvisRoot), 0, aR.pelvis);
    for (const char* e : aN.pelvisExclude)
        ExcludeBranch(aSk, find(e), aR.pelvis);
    AddBranch(aSk, find(aN.spineRoot), aN.spineBlendDepth, aR.spine);
    AddBranch(aSk, find(aN.headRoot), 0, aR.head);
    AddBranch(aSk, find(aN.armL), 0, aR.armL);
    AddBranch(aSk, find(aN.armR), 0, aR.armR);
    for (const char* h : aN.handL)
        AddBranch(aSk, find(h), 0, aR.handL);
    for (const char* h : aN.handR)
        AddBranch(aSk, find(h), 0, aR.handR);
    aR.spineYawCount = 0;
    for (const char* s : aN.spineYaw)
    {
        const int b = find(s);
        if (b >= 0 && aR.spineYawCount < 8)
            aR.spineYawBones[aR.spineYawCount++] = b;
    }
}

// ---- composition ----------------------------------------------------------------------------------------
struct LayerInputs
{
    const Pose* L = nullptr;          // locomotion (MM output)
    const Pose* B = nullptr;          // neutral base pose (blended Pose_Neutral_{Stand|Crouch}_{Idle|Move})
    const Pose* OP = nullptr;         // overlay pose (null = Default overlay: result is L)
    // P1 B2 (fix batch 2): overlay pose of the legs / pelvis regions, blended with the stance alpha of the base poses B (OverlayMachine::
    // LowerBodyKeys) instead of the pose crossfade; null = OP. Those two regions are OP + (L - B), so OP and B must change stance together.
    const Pose* OPLower = nullptr;
    const Additive* dP = nullptr;     // style delta D_MS(BP -> OB); null = motion style (identity)
    float dPWeight = 1.0f;            // per-clip style weight (OverlayMachine::StyleWeight); <= 0 = identity
    float styleArmL = 1.0f, styleArmR = 1.0f; // style only; locomotion additive remains unchanged
    Curves curves;                    // c[] (caller decides the one-frame delay of UE)
    // CR-4 (loco/stance, recording 2026-10-31 22:55): the spine / head regions during a stance fade of B, composed once per stance side
    // (OverlayMachine::StanceGroupsOf) and mixed by the crouch share of B - see ComposeLayers. torso[0 / 1] = stand / crouch side: overlay
    // pose of the stance-free groups, base pose of that side, its layer curves. Any pointer null = off (spine / head take OP and curves).
    struct TorsoSide
    {
        const Pose* OP = nullptr;
        const Pose* B = nullptr;
        float spine = 0.0f, spineAdd = 0.0f, head = 0.0f, headAdd = 0.0f;
        float legs = 0.0f, pelvis = 0.0f; // OVLEGS: the legs / pelvis regions take the same sides (layer weights of that side)
    };
    TorsoSide torso[2];
    float torsoCrouch = 0.0f; // crouch share s of B (0 < s < 1 while the sides are set)
};

struct LayerScratch // ~100 KB; keep one per character, not on the stack of a hook
{
    Additive dLms, dLls, dSide;
    Pose inner, regionTmp, legs, pelvis, spine, head, armL, armR, acc;
    Pose spineSide[2], headSide[2], legsSide[2], pelvisSide[2];
};

inline void ComposeLayers(const Skeleton& aSk, const Regions& aR, const LayerInputs& aIn, LayerScratch& aS, Pose& aOut)
{
    const Pose& L = *aIn.L;
    aOut.n = aSk.n;
    if (!aIn.OP || !aIn.B)
    {
        std::memcpy(aOut.b, L.b, sizeof(Xf) * aSk.n);
        return;
    }
    const Curves& c = aIn.curves;
    MakeAdditive(aSk, *aIn.B, L, true, aS.dLms);
    MakeAdditive(aSk, *aIn.B, L, false, aS.dLls);
    auto region = [&](float aAdd, bool aLocalLoco, float aWeight, Pose& aDst, float aStyle = 1.0f, const Pose* aOp = nullptr) {
        aS.inner = aOp ? *aOp : *aIn.OP;
        ApplyAdditive(aSk, aS.inner, aLocalLoco ? aS.dLls : aS.dLms, aAdd);
        if (aIn.dP && aIn.dPWeight > 0.0f)
            ApplyAdditive(aSk, aS.inner, *aIn.dP, aAdd * aIn.dPWeight * Clamp01(aStyle));
        Blend(L, aS.inner, aWeight, aDst);
    };
    // CR-4 (loco/stance, recording 2026-10-31 22:55, crouch -> stand with a drawn Pistol2H): the spine / head regions are OP + (L - B) as well.
    // On the pose crossfade their overlay pose AND their layer weights (Pistol2H Relaxed: Layering_Spine 1 standing, 0 crouched) reached the
    // stand pose in poseSwitch (0.2 s) while B left the crouch base over stanceSwitch (1.0 s) and L rose over the 0.5 s base blend: the
    // composed torso stood upright 5.6 - 7.8 deg ahead of the base pose (head up to 5 cm higher) while the hips were still low, and it cancelled
    // the authored forward lean of Transition_Crouch_to_Stand. One blended inner on the stance alpha of B (OPLower + B-synced curves, the P1 B2
    // form of the legs) is not enough here: it leaves the cross term s (1 - s) c_crouch (OP_crouch - B_crouch), and the crouch overlay spine
    // (never shown in a settled crouch: Layering_Spine 0) is far from the crouch base - running into a crouch the torso stayed 7 - 9 deg too
    // upright and the head 5 - 8 cm too high for ~0.5 s (mutation 2). During a stance fade of B each stance side is therefore composed with
    // its own overlay pose, base pose and curves, Blend(L, OP_side + c_add D(B_side -> L), c_w), and the two are mixed by the crouch share s:
    // L + (1 - s) c_stand (OP_stand - B_stand) + s c_crouch (OP_crouch - B_crouch) to first order, no cross term. Outside a stance fade (s 0
    // or 1) the sides are not set and nothing changes. Legs / pelvis (P1 B2), arms and hands are unchanged. tests/LocoStanceTest.cpp;
    // mutation 1 (UTPP_LOCOSTANCE_MUTATION) = spine / head on the pose crossfade (pre-fix), 2 = one inner on OPLower + B-synced curves.
#if defined(UTPP_LOCOSTANCE_MUTATION) && UTPP_LOCOSTANCE_MUTATION == 2
    const Pose* opTorso = aIn.OPLower;
#else
    const Pose* opTorso = nullptr;
#endif
#if defined(UTPP_LOCOSTANCE_MUTATION) && (UTPP_LOCOSTANCE_MUTATION == 1 || UTPP_LOCOSTANCE_MUTATION == 2)
    const bool sides = false;
#else
    const bool sides = aIn.torso[0].OP && aIn.torso[0].B && aIn.torso[1].OP && aIn.torso[1].B;
#endif
    // OVLEGS (loco/ovlegs, recording 2026-10-31 22:55, crouch -> stand with a drawn Pistol2H): the legs / pelvis regions are OP + (L - B) with
    // the P1 B2 form above - OPLower and the curves on the stance alpha s of B - and that form has the same cross term as the spine / head
    // (CR-4): c = (1 - s) c_stand + s c_crouch multiplies OPLower - B = (1 - s) (OP_stand - B_stand) + s (OP_crouch - B_crouch), which leaves
    // s (1 - s) c_stand (OP_crouch - B_crouch) in the legs. The crouch overlay legs (a Pistol2H crouch pose with its own knee and hip bend, never
    // shown settled: the crouch Layering_Legs is 0) are not the crouch base; with the stand-side nonlinearity below, the composed left ankle
    // sank 3.0 cm (right 1.0 cm) under the ankle height of the base pose, which keeps the feet on the layers (BlendStack::KeepFootHeight),
    // for the 1 s of the stance fade of B - about 0.5 cm of it this cross term. During a stance fade the legs / pelvis are therefore composed
    // per stance side as well and mixed by s - L + (1 - s) c_stand (OP_stand - B_stand) + s c_crouch (OP_crouch - B_crouch) to first order -
    // and at s 0 or 1 that is the single-inner form, bit for bit (the sides are not set outside a fade). The rest is not a weighting problem:
    // each side is OP_side + D_MS(B_side -> L) and L is far from B_side while the stance fades (PoseEvaluator::KeepLowerFootHeight lifts the
    // composed ankles for that). tests/LocoStanceTest.cpp OVLEGS; mutation 1 (UTPP_OVLEGS_MUTATION) = legs / pelvis on OPLower and no lift
    // (pre-fix), mutation 2 = per-side legs / pelvis without the lift (MotionPose.hpp).
#if defined(UTPP_OVLEGS_MUTATION) && UTPP_OVLEGS_MUTATION == 1
    const bool lowerSides = false;
#else
    const bool lowerSides = sides;
#endif
    if (!lowerSides)
    {
        region(1.0f, false, c[kLegs], aS.legs, 1.0f, aIn.OPLower);
        region(1.0f, false, c[kPelvis], aS.pelvis, 1.0f, aIn.OPLower);
    }
    if (sides)
    {
        for (int k = 0; k < 2; ++k)
        {
            const LayerInputs::TorsoSide& sd = aIn.torso[k];
            MakeAdditive(aSk, *sd.B, L, true, aS.dSide);
            auto side = [&](float aAdd, float aWeight, Pose& aDst) {
                aS.inner = *sd.OP;
                ApplyAdditive(aSk, aS.inner, aS.dSide, aAdd);
                if (aIn.dP && aIn.dPWeight > 0.0f)
                    ApplyAdditive(aSk, aS.inner, *aIn.dP, aAdd * aIn.dPWeight);
                Blend(L, aS.inner, aWeight, aDst);
            };
            side(sd.spineAdd, sd.spine, aS.spineSide[k]);
            side(sd.headAdd, sd.head, aS.headSide[k]);
            if (lowerSides)
            {
                side(1.0f, sd.legs, aS.legsSide[k]);
                side(1.0f, sd.pelvis, aS.pelvisSide[k]);
            }
        }
        const float s = Clamp01(aIn.torsoCrouch);
        Blend(aS.spineSide[0], aS.spineSide[1], s, aS.spine);
        Blend(aS.headSide[0], aS.headSide[1], s, aS.head);
        if (lowerSides)
        {
            Blend(aS.legsSide[0], aS.legsSide[1], s, aS.legs);
            Blend(aS.pelvisSide[0], aS.pelvisSide[1], s, aS.pelvis);
        }
    }
    else
    {
        region(c[kSpineAdd], false, c[kSpine], aS.spine, 1.0f, opTorso);
        region(c[kHeadAdd], false, c[kHead], aS.head, 1.0f, opTorso);
    }
    region(c[kArmLAdd], true, c[kArmL], aS.armL, aIn.styleArmL);
    region(c[kArmRAdd], true, c[kArmR], aS.armR, aIn.styleArmR);

    Pose& P = aS.acc;
    LayeredBlend(aSk, aS.legs, aS.pelvis, aR.pelvis, 1.0f, true, P);
    LayeredBlend(aSk, P, aS.spine, aR.spine, 1.0f, true, P);
    LayeredBlend(aSk, P, aS.head, aR.head, 1.0f, true, P);
    const float lsL = Clamp01(c[kArmLLS]), lsR = Clamp01(c[kArmRLS]);
    LayeredBlend(aSk, P, aS.armL, aR.armL, 1.0f - std::floor(lsL), true, P);
    LayeredBlend(aSk, P, aS.armL, aR.armL, lsL, false, P);
    LayeredBlend(aSk, P, aS.armR, aR.armR, 1.0f - std::floor(lsR), true, P);
    LayeredBlend(aSk, P, aS.armR, aR.armR, lsR, false, P);
    LayeredBlend(aSk, P, *aIn.OP, aR.handL, c[kHandL], false, P);
    LayeredBlend(aSk, P, *aIn.OP, aR.handR, c[kHandR], false, P);
    std::memcpy(aOut.b, P.b, sizeof(Xf) * aSk.n);
}

// Hand fix after the aim offset: LBB_LS restores clavicles (+ ik_hand_*) from the pre-AO pose.
inline void RestoreHandsAfterAO(const Skeleton& aSk, const Pose& aPreAO, const BoneWeights& aClavL,
                                const BoneWeights& aClavR, const Curves& aC, Pose& aIo)
{
    LayeredBlend(aSk, aIo, aPreAO, aClavL, aC[kHandL], false, aIo);
    LayeredBlend(aSk, aIo, aPreAO, aClavR, aC[kHandR], false, aIo);
}

// ---- crossfade helper -----------------------------------------------------------------------------------------
enum class BlendCurve : uint8_t
{
    Linear,
    CubicInOut,
    SineOut // sin(u * pi / 2): full speed at the start (pi / 2 x the linear rate), settles at the end (the hip raise out of Relaxed)
};

inline float EvalBlendCurve(BlendCurve aC, float u)
{
    u = Clamp01(u);
    if (aC == BlendCurve::SineOut)
        return u >= 1.0f ? 1.0f : std::sin(u * 1.57079633f);
    return aC == BlendCurve::CubicInOut ? (u < 0.5f ? 4.0f * u * u * u : 1.0f - std::pow(-2.0f * u + 2.0f, 3.0f) * 0.5f)
                                        : u;
}

// Weighted set of keys; a new target fades in over `time` while the frozen previous weights fade out.
template <int N>
struct Crossfade
{
    int key[N]{};
    float from[N]{}; // weights frozen at the start of the current fade
    float w[N]{};
    int count = 0;
    int target = -1;
    float elapsed = 0.0f, duration = 0.0f;
    BlendCurve curve = BlendCurve::Linear;

    void Reset(int aKey)
    {
        count = 1;
        key[0] = aKey;
        w[0] = from[0] = 1.0f;
        target = aKey;
        elapsed = duration = 0.0f;
    }
    void SetTarget(int aKey, float aTime, BlendCurve aCurve = BlendCurve::Linear)
    {
        if (count == 0 || aTime <= 0.0f)
        {
            Reset(aKey);
            return;
        }
        if (aKey == target)
            return;
        // freeze current weights; ensure aKey has a slot
        int slot = -1;
        for (int i = 0; i < count; ++i)
        {
            from[i] = w[i];
            if (key[i] == aKey)
                slot = i;
        }
        if (slot < 0)
        {
            if (count == N) // drop the smallest weight
            {
                int m = 0;
                for (int i = 1; i < count; ++i)
                    if (w[i] < w[m])
                        m = i;
                key[m] = key[count - 1];
                from[m] = from[count - 1];
                w[m] = w[count - 1];
                --count;
                float s = 0.0f;
                for (int i = 0; i < count; ++i)
                    s += from[i];
                for (int i = 0; i < count; ++i)
                    from[i] = s > 0.0f ? from[i] / s : 1.0f / count;
            }
            slot = count++;
            key[slot] = aKey;
            from[slot] = w[slot] = 0.0f;
        }
        target = aKey;
        elapsed = 0.0f;
        duration = aTime;
        curve = aCurve;
    }
    void Update(float aDt)
    {
        if (count == 0)
            return;
        elapsed += std::max(0.0f, aDt);
        const float a = duration > 0.0f ? EvalBlendCurve(curve, elapsed / duration) : 1.0f;
        for (int i = 0; i < count; ++i)
            w[i] = from[i] * (1.0f - a) + (key[i] == target ? a : 0.0f);
        if (a >= 1.0f) // compact
        {
            Reset(target);
        }
    }
    float WeightOf(int aKey) const
    {
        for (int i = 0; i < count; ++i)
            if (key[i] == aKey)
                return w[i];
        return 0.0f;
    }
};

// ---- overlay state machine ------------------------------------------------------------------------------------
enum class PoseOverlay : uint8_t // UTPP subset of motion Enum_OverlayPose (0 Default, 1 Rifle, 2 Pistol1H, 3 Pistol2H, 9 Injured)
{
    Default,
    Rifle,
    Pistol1H,
    Pistol2H,
    Injured,
    Game, // melee / body carry: no overlay, game animations own the upper body
    Count
};
enum class Style : uint8_t // Enum_OverlayBase subset: 0 motion, 2 Masculine, 3 Feminine
{
    Neutral,
    Feminine,
    Masculine,
    Count
};
enum class AimState : uint8_t
{
    Relaxed,
    Ready,
    Aiming
};

inline bool CanAim(PoseOverlay o) { return o == PoseOverlay::Rifle || o == PoseOverlay::Pistol1H || o == PoseOverlay::Pistol2H; }

// ---- per-clip style weight by source family ----------------------------------------------
// The style delta can be scaled per source family of the active clip: the dP weight is weight(style, family of the
// active MM clip), crossfaded on a family change (OverlayMachine::families). The v2 database only flags ADDED clips, so the family comes from a
// sidecar (tools/mmbake/export_style_weights.py, text "utpp-style-weights 1"):
//   [weights]  Style/family = w (0..1)        [clips]  name = family,  * = family of unlisted motion clips,
//                                                       ! = family of unlisted ADDED clips
enum class SourceFamily : uint8_t
{
    Primary,
    Other,
    Count
};

inline const char* SourceFamilyName(SourceFamily f)
{
    static const char* kN[] = {"primary", "other"};
    return f < SourceFamily::Count ? kN[static_cast<int>(f)] : "";
}
inline bool ParseSourceFamily(std::string_view s, SourceFamily& f)
{
    for (int i = 0; i < static_cast<int>(SourceFamily::Count); ++i)
        if (s == SourceFamilyName(static_cast<SourceFamily>(i)))
        {
            f = static_cast<SourceFamily>(i);
            return true;
        }
    return false;
}
inline bool ParseStyleName(std::string_view s, Style& st)
{
    static const char* kN[] = {"Neutral", "Feminine", "Masculine"};
    for (int i = 0; i < static_cast<int>(Style::Count); ++i)
        if (s == kN[i])
        {
            st = static_cast<Style>(i);
            return true;
        }
    return false;
}

struct StyleWeightTable // weight of the style delta per source family of the active clip
{
    static constexpr int kStyles = static_cast<int>(Style::Count), kFamilies = static_cast<int>(SourceFamily::Count);
    float w[kStyles][kFamilies]{};

    StyleWeightTable() { SetDefaults(); }
    void SetDefaults() // every style on every family: 1
    {
        for (auto& row : w)
            for (float& x : row)
                x = 1.0f;
    }
    float Get(Style s, SourceFamily f) const
    {
        return s < Style::Count && f < SourceFamily::Count ? w[static_cast<int>(s)][static_cast<int>(f)] : 1.0f;
    }
    void Set(Style s, SourceFamily f, float a) { w[static_cast<int>(s)][static_cast<int>(f)] = Clamp01(a); }
};

struct StyleFamilyData // parsed sidecar: weights + clip name -> family
{
    struct Entry
    {
        std::string clip;
        SourceFamily family = SourceFamily::Primary;
    };
    StyleWeightTable weights;
    std::vector<Entry> clips; // sorted by name
    SourceFamily unlistedSourcePack = SourceFamily::Primary, unlistedAdded = SourceFamily::Other;

    // Fail-closed: a bad line, an unknown style / family, a weight outside 0..1, a duplicated clip or a missing
    // "utpp-style-weights 1" first line returns false and resets to the defaults.
    bool Parse(std::string_view aText, std::string* aErr = nullptr)
    {
        Reset();
        auto trim = [](std::string_view s) {
            while (!s.empty() && (s.front() == ' ' || s.front() == '\t' || s.front() == '\r'))
                s.remove_prefix(1);
            while (!s.empty() && (s.back() == ' ' || s.back() == '\t' || s.back() == '\r'))
                s.remove_suffix(1);
            return s;
        };
        int lineNo = 0, section = 0; // 1 weights, 2 clips
        bool header = false;
        while (!aText.empty())
        {
            const size_t nl = aText.find('\n');
            std::string_view line = trim(aText.substr(0, nl));
            aText = nl == std::string_view::npos ? std::string_view{} : aText.substr(nl + 1);
            ++lineNo;
            if (line.empty() || line.front() == '#')
                continue;
            if (!header)
            {
                if (line != "utpp-style-weights 1")
                    return Fail(aErr, lineNo);
                header = true;
                continue;
            }
            if (line == "[weights]" || line == "[clips]")
            {
                section = line == "[weights]" ? 1 : 2;
                continue;
            }
            const size_t eq = line.find('=');
            if (eq == std::string_view::npos || section == 0)
                return Fail(aErr, lineNo);
            const std::string_view lhs = trim(line.substr(0, eq)), rhs = trim(line.substr(eq + 1));
            if (section == 1)
            {
                const size_t sl = lhs.find('/');
                Style st{};
                SourceFamily fa{};
                if (sl == std::string_view::npos || !ParseStyleName(trim(lhs.substr(0, sl)), st) ||
                    !ParseSourceFamily(trim(lhs.substr(sl + 1)), fa))
                    return Fail(aErr, lineNo);
                char* end = nullptr;
                const std::string num(rhs);
                const float v = std::strtof(num.c_str(), &end);
                if (num.empty() || end != num.c_str() + num.size() || !(v >= 0.0f && v <= 1.0f))
                    return Fail(aErr, lineNo);
                weights.Set(st, fa, v);
                continue;
            }
            SourceFamily fa{};
            if (lhs.empty() || !ParseSourceFamily(rhs, fa))
                return Fail(aErr, lineNo);
            if (lhs == "*")
                unlistedSourcePack = fa;
            else if (lhs == "!")
                unlistedAdded = fa;
            else
                clips.push_back(Entry{std::string(lhs), fa});
        }
        if (!header)
            return Fail(aErr, lineNo);
        std::sort(clips.begin(), clips.end(), [](const Entry& a, const Entry& b) { return a.clip < b.clip; });
        for (size_t i = 1; i < clips.size(); ++i)
            if (clips[i].clip == clips[i - 1].clip)
            {
                Reset();
                if (aErr)
                    *aErr = "style weights: duplicated clip " + clips[i].clip;
                return false;
            }
        return true;
    }
    SourceFamily Lookup(std::string_view aClip, bool aAdded) const
    {
        const auto it = std::lower_bound(clips.begin(), clips.end(), aClip,
                                         [](const Entry& e, std::string_view n) { return std::string_view(e.clip) < n; });
        if (it != clips.end() && std::string_view(it->clip) == aClip)
            return it->family;
        return aAdded ? unlistedAdded : unlistedSourcePack;
    }
    // Per-clip family array for a database: NameFn(int clip) -> string_view, AddedFn(int clip) -> bool.
    template <class NameFn, class AddedFn>
    void BuildClipFamilies(int aNumClips, NameFn aName, AddedFn aAdded, std::vector<SourceFamily>& aOut) const
    {
        aOut.resize(static_cast<size_t>(std::max(0, aNumClips)));
        for (int c = 0; c < aNumClips; ++c)
            aOut[static_cast<size_t>(c)] = Lookup(aName(c), aAdded(c));
    }

private:
    void Reset()
    {
        weights.SetDefaults();
        clips.clear();
        unlistedSourcePack = SourceFamily::Primary;
        unlistedAdded = SourceFamily::Other;
    }
    bool Fail(std::string* aErr, int aLine)
    {
        Reset();
        if (aErr)
            *aErr = "style weights: bad line " + std::to_string(aLine);
        return false;
    }
};

struct OverlayTimes
{
#if defined(UTPP_POSEV2_OVL_MUTATION) && UTPP_POSEV2_OVL_MUTATION == 1
    float poseSwitch = 0.0f;        // mutation 1: one-frame pop (motion)
#else
    float poseSwitch = 0.2f;        // overlay change (motion: one-frame pop)
#endif
    float styleSwitch = 0.3f;
    float familySwitch = 0.3f;      // style weight crossfade when the active MM clip changes source family
    float relaxedToReady = 0.2f;    // table
    float readyToAiming = 0.2f;
    // P9 C: the raise of a weapon that is shot from the hip out of Ready (OverlayInputs::hipFireRaise): the hands AND the AO alpha (aimW:
    // sweep pitch + spine yaw up to 60 deg) take it. The first shot without the 0.75 s hold left the muzzle 20-29 cm low for the 12 frames
    // of readyToAiming (recording 2026-10-21: 17 of 41 rifle shots, flash into the ground); an AO alpha that trails the hands (0.2 s) leaves
    // the second shot of a burst (~0.1 s) with half of the sweep pitch and yaw. The 0.1 s ramp spins the yaw at 600 deg/s (10 deg per frame
    // at 60 fps, 20 at 30 fps): below AoJumpDeg(dt) of AimSpineYawFilter.hpp (clamp(1500 * dt, 45, 90) deg per frame), which lets
    // camera-like tracking through untouched. Real ADS keeps readyToAiming. Mutation 1 (UTPP_OVL_HIPFIRE_MUTATION) = no faster raise;
    // mutation 3 = the AO alpha keeps readyToAiming while the hands take the raise time.
#if defined(UTPP_OVL_HIPFIRE_MUTATION) && UTPP_OVL_HIPFIRE_MUTATION == 1
    float hipFireReadyToAiming = 0.2f;
#else
    float hipFireReadyToAiming = 0.1f;
#endif
    // Hip shot out of RELAXED (recording 2026-10-31 22:57:26.88, first crouched rifle shot, HIP-1): Relaxed -> Ready (0.2 s) chained into
    // the 0.1 s P9 C raise once Ready reached readyWeightToAim (~50 ms), so the hands and the AO alpha crossed the whole Relaxed crouch ->
    // Aiming crouch delta (torso yaw 30 - 40 deg, head pitch, rifle hip -> chest; AO pitch -25.6 deg and the barrel loop on the same
    // alpha) in ~0.13 s: aiming weight 10 / s, 0.33 of the delta in one 30 fps frame (video: one dominant step 7.167 -> 7.200 s). Now the
    // hip shot goes Relaxed -> Aiming directly in one fade of hipFireRelaxedToAiming with BlendCurve::SineOut on the hands AND the AO alpha
    // (the barrel loop takes the AO alpha, MotionPose): peak rate (pi / 2) / 0.25 = 6.3 / s (0.21 of the delta per 30 fps frame, ~8 deg of a
    // 40 deg torso turn; 0.10 at 60 fps), and it starts on the shot frame, so the raise is not slower where it matters for the burst:
    // 0.59 at 0.1 s (the chain: ~0.5), 0.81 at 0.15 s, 0.95 at 0.2 s, 1 at 0.25 s. A smoothstep of the same peak (0.25 s) would leave the
    // second round of a burst (~0.1 s) with 35 % of the sweep pitch (the P9 C flash into the ground); 0.2 s SineOut peaks at 7.9 / s
    // (0.26 per 30 fps frame). The P9 C raise out of Ready (hipFireReadyToAiming) and real ADS are unchanged. Mutation 1
    // (UTPP_LOCOHIP_MUTATION) = the old chain. tests/LocoHipTest.cpp
    float hipFireRelaxedToAiming = 0.25f;
    float pistol1HTo2HAim = 0.45f; // larger authored arm arc when one-handed Ready enters two-handed Aim
    float aimingToReady = 0.5f;
    float readyToRelaxed = 0.75f;   // CubicInOut
    float readyToRelaxedSprint = 0.2f;
    float aimingMinTime = 0.75f;
    float readyIdleTimeout = 3.0f;
    float readyMovingTimeout = 2.0f;
    float readyWeightToAim = 0.25f;
    float stanceSwitch = 1.0f;      // SM_BasePoses: 1.0 s on stance change, 0.1 s on movement change
    float moveSwitch = 0.1f;
    // CR-5 (recording 2026-10-31 22:56:34.98, crouch walk -> stop with Pistol2H): the arm regions are OP + D_LS(B -> L). At the crouch stop
    // B switched Crouch_Move -> Crouch_Idle in moveSwitch (0.1 s) while the locomotion pose L inertializes over 0.5 s (the torso came upright
    // in ~0.25 s), so the arm delta - and the pistol - snapped in ~3 frames (Pistol2H has one crouch pose: OP did not move). A move <-> idle
    // change of B in a SETTLED crouch takes crouchMoveSwitch (the visible body settle); stand keeps moveSwitch (tuned there), a stance fade
    // and a move change inside it keep their times. The legs / pelvis crouch share (LowerBodyKeys) does not change: both keys are crouch.
    // Mutation 3 (UTPP_LOCOHIP_MUTATION) = moveSwitch in the crouch too.
    float crouchMoveSwitch = 0.3f;
};

struct OverlayInputs
{
    PoseOverlay overlay = PoseOverlay::Default;
    Style style = Style::Neutral;
    bool crouch = false;
    bool moving = false;      // movement state Moving (pose variant Idle/Move, base pose B)
    bool hasVelocity = false;
    bool wantsToAim = false;
    bool wantsReady = false; // explicit combat readiness; does not request ADS
    bool hipFireRaise = false; // the Aiming request is a hip shot (not ADS): Ready -> Aiming takes times.hipFireReadyToAiming
    bool wantsToSprint = false;
    bool sprinting = false;   // Gait = Sprint
    SourceFamily family = SourceFamily::Primary; // source family of the active MM clip (StyleFamilyData)
    float familyBlendTime = -1.0f;            // < 0: times.familySwitch (e.g. pass the MM blend time instead)
    float airW = 0.0f; // 0..1 weight of the real jump / fall (driver AirLegsFade); fades only the leg layer, see FadeLowerBodyInAir
    float jumpOutArmW = 0.0f; // 0..1 weight of a vehicle jump-out flight (driver JumpOutArmsFade); fades only the arm / hand layers, see FadeArmsInJumpOutFlight
};

// Pose key = overlay * 12 + aim * 4 + crouch * 2 + moving. The caller maps keys to file poses (OverlayFile).
inline int PoseKey(PoseOverlay o, AimState a, bool crouch, bool moving)
{
    return static_cast<int>(o) * 12 + static_cast<int>(a) * 4 + (crouch ? 2 : 0) + (moving ? 1 : 0);
}
inline void DecodePoseKey(int k, PoseOverlay& o, AimState& a, bool& crouch, bool& moving)
{
    o = static_cast<PoseOverlay>(k / 12);
    a = static_cast<AimState>((k % 12) / 4);
    crouch = (k & 2) != 0;
    moving = (k & 1) != 0;
}
inline int BaseKey(bool crouch, bool moving) { return (crouch ? 2 : 0) + (moving ? 1 : 0); }

// Offline mutation switch of the P1 stance changes (shared with MotionAdapter.hpp; tests build mutated copies with -DUTPP_P1_MUTATION=k):
// 2 = the legs / pelvis overlay pose follows the pose crossfade (the pre-P1 behaviour: no separate stance alpha),
// 5 = the retired B1 on top of B2 (the whole pose crossfade takes stanceSwitch on a stance change),
// 6 = PreserveIdlePistolLowerBody reads the idle weight from the pose crossfade, not from the keys the legs / pelvis are built from.
#ifndef UTPP_P1_MUTATION
#define UTPP_P1_MUTATION 0
#endif

struct OverlayMachine
{
    OverlayTimes times;
    AimState aim = AimState::Relaxed;
    float timeInAim = 0.0f;
    PoseOverlay overlay = PoseOverlay::Default;
    Style style = Style::Neutral;
    bool crouch = false, moving = false;
    bool started = false;
    Crossfade<6> poses;  // keys PoseKey
    bool hipRaiseFade = false; // the running `poses` fade is the Ready -> Aiming raise of a hip shot (or its continuation after a key change)
    Crossfade<3> styles; // keys Style (dP per stance chosen by the caller)
    Crossfade<4> bases;  // keys BaseKey (B), BP = crouch weight
    Crossfade<3> aimW;   // keys AimState: weight of each aim state (Aiming weight = AO sweep alpha)
    Crossfade<4> families; // keys SourceFamily of the active MM clip (per-clip style weight)
    SourceFamily family = SourceFamily::Primary;

    void Update(const OverlayInputs& aIn, float aDt)
    {
        if (!started)
        {
            started = true;
            overlay = aIn.overlay;
            style = aIn.style;
            crouch = aIn.crouch;
            moving = aIn.moving;
            aim = AimState::Relaxed;
            timeInAim = 0.0f;
            poses.Reset(PoseKey(overlay, aim, crouch, moving));
            hipRaiseFade = false;
            styles.Reset(static_cast<int>(style));
            bases.Reset(BaseKey(crouch, moving));
            aimW.Reset(static_cast<int>(aim));
            family = aIn.family;
            families.Reset(static_cast<int>(family));
            return;
        }
        float poseTime = times.poseSwitch;
        BlendCurve poseCurve = BlendCurve::Linear;
        // Quick ADS can pass through Ready while the previous 1H pose is still fading out.
        bool fadingPistol1HTo2H = overlay == PoseOverlay::Pistol1H;
        for (int i = 0; i < poses.count; ++i)
            fadingPistol1HTo2H = fadingPistol1HTo2H ||
                (poses.w[i] > 0.005f && poses.key[i] / 12 == static_cast<int>(PoseOverlay::Pistol1H));
        fadingPistol1HTo2H = fadingPistol1HTo2H && aIn.overlay == PoseOverlay::Pistol2H && aIn.wantsToAim;
        // P9 A: the hands fade of a hip RAISE that is still running (hipRaiseFade: started by the Ready -> Aiming change of a hip shot, or the
        // continuation of that fade after a key change inside it): a crouch / move / slide key change inside it must not restart the hands at
        // poseSwitch (0.2 s) - they would reach Aim 0.18 - 0.22 s after the shot, not 0.1 s, and the AO alpha (aimW) would run ahead of them.
        // It takes the rest of the raise, but at least handsFloor (half of it): a change in the last frames would otherwise turn Stand_Aim into
        // Crouch_Aim (13 - 19 deg in arms and head) in one frame. ADS, every other fade and every key change after the raise (the 0.75 s hold
        // of the hip pose keeps hipFireRaise set) keep poseSwitch - a second change inside THAT fade must not be shortened to its remainder.
        // Mutation 2 = no guard; 4 = the guard also applies to the fades after the raise; 5 = no floor.
        float handsLeft = 0.0f;
        [[maybe_unused]] const float handsFloor = 0.5f * times.hipFireReadyToAiming; // unused in mutation 5
#if !(defined(UTPP_OVL_HIPFIRE_MUTATION) && UTPP_OVL_HIPFIRE_MUTATION == 2)
#if defined(UTPP_OVL_HIPFIRE_MUTATION) && UTPP_OVL_HIPFIRE_MUTATION == 4
        if (aIn.hipFireRaise && poses.count > 0 && poses.elapsed < poses.duration)
#else
        if (aIn.hipFireRaise && hipRaiseFade && poses.count > 0 && poses.elapsed < poses.duration)
#endif
        {
            PoseOverlay ko;
            AimState ka;
            bool kc, km;
            DecodePoseKey(poses.target, ko, ka, kc, km);
            if (ka == AimState::Aiming)
                handsLeft = poses.duration - poses.elapsed;
        }
#endif
        bool hipRaiseStart = false, handsGuard = false;
        // aim state machine, evaluated before the overlay change so its blend time wins
        const bool canAim = CanAim(aIn.overlay);
        timeInAim += aDt;
        AimState next = aim;
        float t = 0.0f;
        BlendCurve tc = BlendCurve::Linear;
        if (!canAim)
        {
            next = AimState::Relaxed;
            t = times.poseSwitch;
        }
        else if (aim == AimState::Relaxed)
        {
            if ((aIn.wantsToAim || aIn.wantsReady) && !aIn.wantsToSprint && !aIn.sprinting)
            {
#if !(defined(UTPP_LOCOHIP_MUTATION) && UTPP_LOCOHIP_MUTATION == 1) // mutation 1: the hip shot chains Relaxed -> Ready -> Aiming (0.2 + 0.1 s)
                if (aIn.hipFireRaise && aIn.wantsToAim) // HIP-1: one eased Relaxed -> Aiming raise (hands and AO alpha)
                {
                    next = AimState::Aiming;
                    t = times.hipFireRelaxedToAiming;
                    tc = BlendCurve::SineOut;
                }
                else
#endif
                {
                    next = AimState::Ready;
                    t = times.relaxedToReady;
                }
            }
        }
        else if (aim == AimState::Ready)
        {
            // HIP-2 (recording 2026-10-31 22:57:37.71-37.98): the gate keeps Relaxed -> Ready -> Aiming in order, but a hip shot inside an
            // Aiming -> Ready fade (no Relaxed weight left: the weapon is still mostly up) re-raises at once. The gate held the arms on their
            // way down for ~85 ms after the Shoot frame (aimW 0.87 -> 0.78 -> 1.00 in 0.22 s). ADS keeps the gate. Mutation 2 = no bypass.
#if defined(UTPP_LOCOHIP_MUTATION) && UTPP_LOCOHIP_MUTATION == 2
            const bool reRaise = false;
#else
            const bool reRaise = aIn.hipFireRaise && aimW.WeightOf(static_cast<int>(AimState::Relaxed)) <= 0.001f;
#endif
            if (aIn.wantsToAim && (reRaise || aimW.WeightOf(static_cast<int>(AimState::Ready)) >= times.readyWeightToAim) &&
                !aIn.wantsToSprint)
            {
                next = AimState::Aiming;
                t = aIn.hipFireRaise ? times.hipFireReadyToAiming : times.readyToAiming;
            }
            else if (!aIn.wantsToAim && !aIn.wantsReady && (timeInAim > times.readyIdleTimeout ||
                                         (timeInAim > times.readyMovingTimeout && aIn.hasVelocity)))
            {
                next = AimState::Relaxed;
                t = times.readyToRelaxed;
                tc = BlendCurve::CubicInOut;
            }
            else if (aIn.sprinting)
            {
                next = AimState::Relaxed;
                t = times.readyToRelaxedSprint;
            }
        }
        else // Aiming
        {
            if ((!aIn.wantsToAim || aIn.wantsToSprint) && timeInAim > times.aimingMinTime)
            {
                next = AimState::Ready;
                t = times.aimingToReady;
            }
        }
        const bool aimChanged = next != aim;
        if (aimChanged)
        {
            if ((aim == AimState::Ready || aim == AimState::Relaxed) && next == AimState::Aiming)
            {
                hipRaiseStart = aIn.hipFireRaise;
                if (fadingPistol1HTo2H)
                    t = std::max(t, times.pistol1HTo2HAim);
                // HIP-1: an overlay change on the raise frame (1H Relaxed -> resolved 2H Aim) fades the hands linearly over
                // max(t, poseSwitch) below; the AO alpha must take the same curve, not run ahead of them on the SineOut
                if (aim == AimState::Relaxed && aIn.overlay != overlay)
                {
                    tc = BlendCurve::Linear;
                    t = std::max(t, times.poseSwitch);
                }
            }
            float aoT = t;
#if defined(UTPP_OVL_HIPFIRE_MUTATION) && UTPP_OVL_HIPFIRE_MUTATION == 3
            if (hipRaiseStart)
                aoT = std::max(t, times.readyToAiming); // mutation 3: the AO alpha trails the hands (0.2 s)
#endif
            aim = next;
            timeInAim = 0.0f;
            aimW.SetTarget(static_cast<int>(aim), aoT, tc);
            poseTime = t;
            poseCurve = tc;
        }
        const bool overlayChanged = aIn.overlay != overlay;
        if (overlayChanged)
        {
            if (fadingPistol1HTo2H)
                poseTime = std::max(poseTime, times.pistol1HTo2HAim);
            overlay = aIn.overlay;
            // A simultaneous Aim -> Ready and 2H -> 1H change must not shorten the 0.5 s aim exit.
            poseTime = std::max(poseTime, times.poseSwitch);
            poseCurve = BlendCurve::Linear;
        }
        if (aIn.crouch != crouch || aIn.moving != moving)
        {
            float bt = aIn.crouch != crouch ? times.stanceSwitch : times.moveSwitch;
#if !(defined(UTPP_LOCOHIP_MUTATION) && UTPP_LOCOHIP_MUTATION == 3)
            // CR-5: only `moving` changes in a settled crouch (every B weight on a crouch key: no stance fade running)
            if (aIn.crouch == crouch && crouch)
            {
                float s = 0.0f;
                for (int i = 0; i < bases.count; ++i)
                    if (bases.key[i] & 2)
                        s += bases.w[i];
                if (s >= 0.999f)
                    bt = times.crouchMoveSwitch;
            }
#endif
#if UTPP_P1_MUTATION == 5
            // mutation 5: the retired B1 (the whole pose crossfade takes stanceSwitch on a stance change): hands / weapon reach the Aiming pose
            // 0.8 - 0.97 s late when the stance changes during a hipfire / ADS start, and the next key change breaks the OP / B sync
            if (aIn.crouch != crouch && bt > poseTime)
            {
                poseTime = bt;
                poseCurve = BlendCurve::Linear;
            }
#endif
            crouch = aIn.crouch;
            moving = aIn.moving;
            bases.SetTarget(BaseKey(crouch, moving), bt);
            // P9 A: the new key takes the rest of the running hip raise (the hands finish when they would have without the change);
            // a simultaneous overlay change keeps its max(poseSwitch), a simultaneous aim change already set poseTime = t.
            if (handsLeft > 0.0f && !overlayChanged && !aimChanged)
            {
#if defined(UTPP_OVL_HIPFIRE_MUTATION) && UTPP_OVL_HIPFIRE_MUTATION == 5
                poseTime = std::min(poseTime, handsLeft); // mutation 5: no floor (a change in the last raise frame snaps in one frame)
#else
                poseTime = std::min(poseTime, std::max(handsLeft, handsFloor));
#endif
                handsGuard = true;
            }
        }
        if (aIn.style != style)
        {
            style = aIn.style;
            styles.SetTarget(static_cast<int>(style), times.styleSwitch);
        }
        const int poseKey = PoseKey(overlay, aim, crouch, moving);
        if (poseKey != poses.target)
            hipRaiseFade = hipRaiseStart || handsGuard; // any other new target is a plain fade (poseSwitch, or the aim time of ADS)
        poses.SetTarget(poseKey, poseTime, poseCurve);
        if (aIn.family != family)
        {
            family = aIn.family;
            families.SetTarget(static_cast<int>(family),
                               aIn.familyBlendTime >= 0.0f ? aIn.familyBlendTime : times.familySwitch);
        }
        poses.Update(aDt);
        if (poses.duration <= 0.0f)
            hipRaiseFade = false; // the fade ended (compacted)
        styles.Update(aDt);
        bases.Update(aDt);
        aimW.Update(aDt);
        families.Update(aDt);
    }
    // P1 B2 (fix batch 2). ComposeLayers builds the legs / pelvis regions as OP + (L - B), weighted by c[Layering_Legs / Pelvis]. B moves
    // between the stand and crouch base poses over stanceSwitch (1.0 s, `bases`); the overlay poses `poses` fade over poseSwitch (0.2 s) or the
    // aim time, and EVERY key change (stance, move, aim) restarts that fade from frozen weights. For 0.2 s (OP - B) was then the whole
    // stand / crouch pose difference (~0.5 m of hips): armed hips dipped 12 cm at +0.08 s and rose 9.5 cm (12:15 recording, 10 of 10
    // armed crouches; leaving the crouch 36 - 41 cm), and a move / aim change inside the 1 s of the base fade brought the pop back (4 - 23 cm).
    // The legs / pelvis regions therefore take their overlay pose and layer weights from these keys: the weight of each stance-free group
    // (overlay, aim, moving) of `poses`, split between its stand and crouch pose by the crouch share s of `bases` - the stance alpha of
    // B itself, whatever else changes. Hands, arms and their curves keep `poses` (0.2 s / aim times); the spine / head regions take the same
    // groups per stance side during a stance fade (CR-4, StanceGroupsOf + LayerInputs::torso).
    struct StanceGroups
    {
        int key[6]{};   // stance-free group: PoseKey without the crouch bit (stand side = key, crouch side = key | 2)
        float w[6]{};   // summed `poses` weight of the group
        int count = 0;
        float crouch = 0.0f; // crouch share s of `bases`
    };
    void StanceGroupsOf(StanceGroups& aOut) const
    {
        float s = 0.0f;
        for (int i = 0; i < bases.count; ++i)
            if (bases.key[i] & 2)
                s += bases.w[i];
        aOut.crouch = Clamp01(s);
        aOut.count = 0;
        for (int i = 0; i < poses.count && aOut.count < 6; ++i)
        {
            const int g = poses.key[i] & ~2; // PoseKey: bit 1 = crouch
            int j = 0;
            while (j < aOut.count && aOut.key[j] != g)
                ++j;
            if (j == aOut.count)
            {
                aOut.key[j] = g;
                aOut.w[aOut.count++] = 0.0f;
            }
            aOut.w[j] += poses.w[i];
        }
    }
    struct LowerKeys
    {
        int key[12]{};
        float w[12]{};
        int count = 0;
    };
    void LowerBodyKeys(LowerKeys& aOut) const
    {
        aOut.count = 0;
#if UTPP_P1_MUTATION == 2
        for (int i = 0; i < poses.count && aOut.count < 12; ++i) // mutation 2: the pose crossfade as it was (no separate stance alpha)
        {
            aOut.key[aOut.count] = poses.key[i];
            aOut.w[aOut.count++] = poses.w[i];
        }
#else
        StanceGroups g;
        StanceGroupsOf(g);
        const float s = g.crouch;
        for (int j = 0; j < g.count; ++j)
        {
            if ((1.0f - s) * g.w[j] > 0.0f)
            {
                aOut.key[aOut.count] = g.key[j];
                aOut.w[aOut.count++] = (1.0f - s) * g.w[j];
            }
            if (s * g.w[j] > 0.0f)
            {
                aOut.key[aOut.count] = g.key[j] | 2;
                aOut.w[aOut.count++] = s * g.w[j];
            }
        }
#endif
    }
    // P1 B2 + the idle pistol exception: weight of the Pistol1H / Relaxed / stand / idle pose in the keys the legs / pelvis regions are built from
    // (LowerBodyKeys), what PreserveIdlePistolLowerBody must remove from their weights. `poses` alone is not enough: its stand key is gone 0.2 s
    // after a stance change while the legs / pelvis pose keeps (1 - s) of it for 1.0 s (0.2 - 0.8 s of the weapon-idle legs in a female V
    // crouching with a Relaxed 1H pistol, on an overlay file without the E7E5 curve patch).
    float LowerIdlePistolWeight() const
    {
        const int idle = PoseKey(PoseOverlay::Pistol1H, AimState::Relaxed, false, false);
#if UTPP_P1_MUTATION == 6
        return poses.WeightOf(idle); // mutation 6: the exception reads the pose crossfade (B2 bypasses it)
#else
        LowerKeys lk;
        LowerBodyKeys(lk);
        float w = 0.0f;
        for (int i = 0; i < lk.count; ++i)
            if (lk.key[i] == idle)
                w += lk.w[i];
        return w;
#endif
    }
    float AimingWeight() const { return aimW.WeightOf(static_cast<int>(AimState::Aiming)); }
    // Per-clip weight of style aS: table weights of the (crossfading) source families of the active clip.
    float FamilyWeight(Style aS, const StyleWeightTable& aT) const
    {
        if (families.count == 0)
            return aT.Get(aS, family);
        float w = 0.0f;
        for (int i = 0; i < families.count; ++i)
            w += families.w[i] * aT.Get(aS, static_cast<SourceFamily>(families.key[i]));
        return w;
    }
    // Weight of each style delta for BlendAdditives: style crossfade x FamilyWeight; motion (identity) takes the
    // remainder, so a family weight 0 gives the identity (no style) and the weights sum to 1.
    void StyleBlendWeights(const StyleWeightTable& aT, float aOut[static_cast<int>(Style::Count)]) const
    {
        float rest = 1.0f;
        for (int s = 1; s < static_cast<int>(Style::Count); ++s)
        {
            aOut[s] = styles.WeightOf(s) * FamilyWeight(static_cast<Style>(s), aT);
            rest -= aOut[s];
        }
        aOut[0] = std::max(0.0f, rest);
    }
    // LayerInputs::dPWeight when a single style delta is passed (dP of the active style).
    float StyleWeight(const StyleWeightTable& aT) const { return FamilyWeight(style, aT); }
};

// Preserve the authored lowered pistol arm while retaining locomotion/breathing additive.
// Existing pose crossfades supply movement/stance/aim continuity; no velocity threshold or clip ID.
// aPolicyStyle: the style whose idle policy the caller asks for (Feminine = the female V policy, Masculine = the male V policy);
// a machine in another style is left untouched.
inline float IdlePistolRightArmStyleWeight(const OverlayMachine& aMachine, bool aEligible, Style aPolicyStyle = Style::Feminine)
{
    if (!aEligible || aMachine.style != aPolicyStyle) return 1.0f;
    const float idleRelax = aMachine.poses.WeightOf(PoseKey(PoseOverlay::Pistol1H, AimState::Relaxed, false, false));
    return 1.0f - Clamp01(idleRelax);
}

// The relaxed 1H arm pose must not replace the unarmed locomotion stance.
// Fade only lower-body layering with the existing pose weights; Aim, crouch,
// movement and other weapons retain their authored lower-body curves.
// aIdleWeight = the weight of the idle pose in the keys the legs / pelvis weights come from (OverlayMachine::LowerIdlePistolWeight, P1 B2).
inline void PreserveIdlePistolLowerBody(const OverlayMachine& aMachine, bool aEligible, Curves& aCurves, float aIdleWeight,
                                        Style aPolicyStyle = Style::Feminine)
{
    if (!aEligible || aMachine.style != aPolicyStyle) return;
    const float keep = 1.0f - Clamp01(aIdleWeight);
    aCurves.v[kLegs] *= keep;
    aCurves.v[kPelvis] *= keep;
}
// Pose crossfade weight (the pre-B2 source of the idle weight; curves built from the pose crossfade only).
inline void PreserveIdlePistolLowerBody(const OverlayMachine& aMachine, bool aEligible, Curves& aCurves)
{
    PreserveIdlePistolLowerBody(aMachine, aEligible, aCurves,
        aMachine.poses.WeightOf(PoseKey(PoseOverlay::Pistol1H, AimState::Relaxed, false, false)));
}

// fix2/hip P6 K1 (recording 2026-10-21 12:15, jumps in place with a weapon). The weapon poses carry layering_legs = 1, so the leg
// layer is OP + D_MS(B -> L): the knee flexion of the jump clip (86-123 deg, measured against the unarmed idle B) is added to a
// thigh that the weapon pose has already turned by 58-89 deg in mesh space, which bends the knee around the wrong axis (knee
// off-hinge 31 deg median / 47 deg max in the air, 1.3 / 11 deg for jumps in motion where the leg curve is already 0).
// In the air the leg layer therefore follows the locomotion clip; pelvis and the upper body keep the weapon pose. Multiplies, so
// it composes with PreserveIdlePistolLowerBody; aAirW 0 leaves the curves untouched.
inline void FadeLowerBodyInAir(float aAirW, Curves& aCurves)
{
#if defined(UTPP_AIRLEGS_MUTATION) && UTPP_AIRLEGS_MUTATION == 1
    (void)aAirW; (void)aCurves; // mutation 1: the leg layer keeps the weapon pose in the air (pre-fix)
#else
    aCurves.v[kLegs] *= 1.0f - Clamp01(aAirW);
#endif
}

// Smoothed air weight for FadeLowerBodyInAir, stepped once per v2 frame with "the character is really airborne" (chooser
// MovementMode InAir; the slide and its air grace are not). Rises in 0.06 s (the knee starts to twist 0.03-0.07 s after take-off),
// holds 0.15 s after the landing (the landing clip bends the knees to 88-94 deg) and falls in 0.35 s; a new take-off during the
// fall rises again from the current value. The rise moves the thigh by up to ~70 deg (mesh space) from the weapon stance to the jump
// clip, so it is also capped per frame: at 30 Hz the 0.06 s rise alone is a 0.55 step and a 17 deg median thigh step per frame at the
// take-off (criterion 13 deg median / 24 max, measured on the real jump clips); 0.22 per frame gives 12 deg at 30 Hz and leaves 60 Hz
// and above nearly unchanged (0.076 s at 60 Hz). The fall needs no per-frame cap (the landing / ground window steps equal the pre-fix
// ones) but it moves the feet on the ground: the standing weapon stance is 6-23 cm (feet) / 14-34 deg (thighs) away from the clip
// stance, so 0.25 s was up to 0.92 m/s of foot speed on a planted foot; 0.35 s keeps it at the speed of the unarmed landing clip's own
// foot motion (~0.63 m/s) and leaves the FootLocker (settle 0.12 m/s, replant 0.07 / 0.10 m) less to fight.
inline constexpr float kAirLegsRiseSeconds = 0.06f;
inline constexpr float kAirLegsMaxRiseStep = 0.22f;
inline constexpr float kAirLegsHoldSeconds = 0.15f;
#if defined(UTPP_AIRLEGS_MUTATION) && UTPP_AIRLEGS_MUTATION == 5
inline constexpr float kAirLegsFallSeconds = 0.25f; // mutation 5: the pre-review fall (0.92 m/s foot speed on the ground)
#else
inline constexpr float kAirLegsFallSeconds = 0.35f;
#endif
struct AirLegsFade
{
    float w = 0.0f;
    float hold = 0.0f; // seconds of the post-landing hold left
    void Reset() { w = hold = 0.0f; }
    float Step(bool aAir, float aDt)
    {
        const float dt = std::isfinite(aDt) && aDt > 0.0f ? aDt : 0.0f;
        if (aAir)
        {
            hold = kAirLegsHoldSeconds;
#if defined(UTPP_AIRLEGS_MUTATION) && UTPP_AIRLEGS_MUTATION == 4
            w = Clamp01(w + dt / kAirLegsRiseSeconds); // mutation 4: no per-frame cap on the rise (0.06 s at any frame rate)
#else
            w = Clamp01(w + std::fmin(dt / kAirLegsRiseSeconds, kAirLegsMaxRiseStep));
#endif
        }
#if defined(UTPP_AIRLEGS_MUTATION) && UTPP_AIRLEGS_MUTATION == 2
        else // mutation 2: no hold after the landing
            w = Clamp01(w - dt / kAirLegsFallSeconds);
#else
        else if (hold > 0.0f)
            hold = std::fmax(0.0f, hold - dt);
        else
            w = Clamp01(w - dt / kAirLegsFallSeconds);
#endif
        return w;
    }
};

// r9 jump-out arms (game test r5a, move case: the pistol arm straight and horizontal in the flight over the tucked legs; the arm layers of the
// Pistol2H pose ramp 0.14 (+0.08 s) -> 1.00 (+0.59 s) with aimW 0 and compose the weapon arms over the jump clip: RightArm +68 / LeftForeArm +123 deg
// from the clip at full weight, against +2 deg at 0.14). The flight of a vehicle jump-out belongs to the jump clips, like the leg layer of
// FadeLowerBodyInAir: arm and hand layers and the hand IK weights * (1 - w). Only a jump-out flight raises w (JumpOutArmsFade); a normal jump, the ground, aiming and
// every other weapon state keep w 0, which multiplies by exactly 1 (bit-identical).
inline void FadeArmsInJumpOutFlight(float aW, Curves& aCurves)
{
#if defined(UTPP_JOARMS_MUTATION) && UTPP_JOARMS_MUTATION == 1
    (void)aW; (void)aCurves; // mutation 1: the arm layers keep the weapon pose in the jump-out flight (pre-fix)
#else
    const float keep = 1.0f - Clamp01(aW);
    aCurves.v[kArmL] *= keep;
    aCurves.v[kArmR] *= keep;
    aCurves.v[kHandL] *= keep;
    aCurves.v[kHandR] *= keep;
#if !(defined(UTPP_JOARMS_MUTATION) && UTPP_JOARMS_MUTATION == 6) // mutation 6: the game-grip IK keeps pulling the hands over the clip arms
    // 2026-11-04 r11: the grip IK (Enable_HandIK_L / R) is faded with the arms, else it pulls the hands to the grip targets over the clip arms
    aCurves.v[kHandIKL] *= keep;
    aCurves.v[kHandIKR] *= keep;
#endif
#endif
}

// Smoothed jump-out flight weight for FadeArmsInJumpOutFlight, stepped once per v2 frame with "the jump-out window is open and the character
// is really airborne". Rises in 0.10 s (the overlay arm layers themselves are still ramping up from 0 over the same time after the launch, so the
// weapon arms are never shown), capped per frame (a long frame cannot snap the arms), falls in 0.40 s after the touchdown (the arms return to the
// weapon pose while the landing clip plays). A flight that is not a jump-out never raises it.
// 2026-11-04 r11: the fall is capped per frame like the rise (a 0.2 s hitch at the touchdown dropped the weight by 0.5 in one frame), and a weapon
// request (aim / shot / reload) in the flight returns the weapon arms in 0.15 s instead of 0.40 s and blocks the rise.
inline constexpr float kJumpOutArmsRiseSeconds = 0.10f;
inline constexpr float kJumpOutArmsMaxRiseStep = 0.35f;
inline constexpr float kJumpOutArmsMaxFallStep = 0.15f;
inline constexpr float kJumpOutArmsRequestFallSeconds = 0.15f;
#if defined(UTPP_JOARMS_MUTATION) && UTPP_JOARMS_MUTATION == 2
inline constexpr float kJumpOutArmsFallSeconds = 0.02f; // mutation 2: the arms snap back to the weapon pose at the touchdown
#else
inline constexpr float kJumpOutArmsFallSeconds = 0.40f;
#endif
struct JumpOutArmsFade
{
    float w = 0.0f;
    void Reset() { w = 0.0f; }
    float Step(bool aJumpOutAir, float aDt, bool aWeaponRequest = false)
    {
        const float dt = std::isfinite(aDt) && aDt > 0.0f ? aDt : 0.0f;
        if (aJumpOutAir && !aWeaponRequest)
#if defined(UTPP_JOARMS_MUTATION) && UTPP_JOARMS_MUTATION == 3
            w = Clamp01(w + dt / kJumpOutArmsRiseSeconds); // mutation 3: no per-frame cap on the rise (0.10 s at any frame rate)
#else
            w = Clamp01(w + std::fmin(dt / kJumpOutArmsRiseSeconds, kJumpOutArmsMaxRiseStep));
#endif
        else
        {
            const float fallStep = dt / (aWeaponRequest ? kJumpOutArmsRequestFallSeconds : kJumpOutArmsFallSeconds);
#if defined(UTPP_JOARMS_MUTATION) && UTPP_JOARMS_MUTATION == 5
            w = Clamp01(w - fallStep); // mutation 5: no per-frame cap on the fall
#else
            w = Clamp01(w - std::fmin(fallStep, kJumpOutArmsMaxFallStep));
#endif
        }
        return w;
    }
};

// ---- weapon class -> overlay (data) ---------------------------------------------------------------------------
inline const char* kDefaultWeaponOverlayMap =
    "# class = overlay [, alternative]\n"
    "Wea_Handgun = Pistol2H, Pistol1H\n"
    "Wea_Revolver = Pistol2H, Pistol1H\n"
    "Wea_SubmachineGun = Rifle\n"
    "Wea_AssaultRifle = Rifle\n"
    "Wea_Rifle = Rifle\n"
    "Wea_Shotgun = Rifle\n"
    "Wea_ShotgunDual = Rifle\n"
    "Wea_PrecisionRifle = Rifle\n"
    "Wea_SniperRifle = Rifle\n"
    "Wea_LightMachineGun = Rifle\n"
    "Wea_HeavyMachineGun = Rifle\n"
    "ProjectileLauncher = Pistol1H\n"
    "GrenadeAim = Pistol1H\n"
    "ThrowingKnifeAim = Pistol1H\n"
    "Melee = Game\n"
    "BodyCarry = Game\n"
    "Unarmed = Default\n"
    "* = Game\n";

inline bool ParseOverlayName(std::string_view s, PoseOverlay& o)
{
    static const char* kN[] = {"Default", "Rifle", "Pistol1H", "Pistol2H", "Injured", "Game"};
    for (int i = 0; i < static_cast<int>(PoseOverlay::Count); ++i)
        if (s == kN[i])
        {
            o = static_cast<PoseOverlay>(i);
            return true;
        }
    return false;
}

struct WeaponOverlayMap
{
    static constexpr int kMax = 64;
    struct Entry
    {
        char cls[40]{};
        PoseOverlay primary = PoseOverlay::Game, alt = PoseOverlay::Game;
    };
    Entry e[kMax];
    int count = 0;
    PoseOverlay fallback = PoseOverlay::Game;

    // Returns false (and keeps no partial table) on a malformed line.
    bool Parse(std::string_view aText, std::string* aErr = nullptr)
    {
        count = 0;
        fallback = PoseOverlay::Game;
        auto trim = [](std::string_view s) {
            while (!s.empty() && (s.front() == ' ' || s.front() == '\t' || s.front() == '\r'))
                s.remove_prefix(1);
            while (!s.empty() && (s.back() == ' ' || s.back() == '\t' || s.back() == '\r'))
                s.remove_suffix(1);
            return s;
        };
        int lineNo = 0;
        while (!aText.empty())
        {
            const size_t nl = aText.find('\n');
            std::string_view line = trim(aText.substr(0, nl));
            aText = nl == std::string_view::npos ? std::string_view{} : aText.substr(nl + 1);
            ++lineNo;
            if (line.empty() || line.front() == '#')
                continue;
            const size_t eq = line.find('=');
            if (eq == std::string_view::npos)
                return Fail(aErr, lineNo);
            const std::string_view cls = trim(line.substr(0, eq));
            std::string_view rhs = trim(line.substr(eq + 1));
            std::string_view alt;
            if (const size_t comma = rhs.find(','); comma != std::string_view::npos)
            {
                alt = trim(rhs.substr(comma + 1));
                rhs = trim(rhs.substr(0, comma));
            }
            PoseOverlay p{}, a{};
            if (cls.empty() || cls.size() >= sizeof(Entry::cls) || !ParseOverlayName(rhs, p))
                return Fail(aErr, lineNo);
            a = p;
            if (!alt.empty() && !ParseOverlayName(alt, a))
                return Fail(aErr, lineNo);
            if (cls == "*")
            {
                fallback = p;
                continue;
            }
            if (count >= kMax)
                return Fail(aErr, lineNo);
            Entry& en = e[count++];
            std::memcpy(en.cls, cls.data(), cls.size());
            en.cls[cls.size()] = 0;
            en.primary = p;
            en.alt = a;
        }
        return true;
    }
    PoseOverlay Lookup(std::string_view aClass, bool aUseAlt = false) const
    {
        for (int i = 0; i < count; ++i)
            if (aClass == e[i].cls)
                return aUseAlt ? e[i].alt : e[i].primary;
        return fallback;
    }

private:
    bool Fail(std::string* aErr, int aLine)
    {
        count = 0;
        if (aErr)
            *aErr = "weapon overlay map: bad line " + std::to_string(aLine);
        return false;
    }
};

// Overlay selection: weapon drawn -> mapped overlay; otherwise Injured (low HP / bleeding) or Default.
inline PoseOverlay SelectOverlay(const WeaponOverlayMap& aMap, std::string_view aWeaponClass, bool aWeaponDrawn,
                                 bool aInjured, bool aCarryingBody, bool aPreferPistol1H = false)
{
    if (aCarryingBody)
        return PoseOverlay::Game;
    if (aWeaponDrawn && !aWeaponClass.empty())
        return aMap.Lookup(aWeaponClass, aPreferPistol1H);
    return aInjured ? PoseOverlay::Injured : PoseOverlay::Default;
}

} // namespace utpp::mm::ov
