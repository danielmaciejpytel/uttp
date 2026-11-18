// Per-bone blend profiles of the motion matching cross-fade and of the hand-back from an external pose, as a data table.
// Semantics of the UE blend stack (UBlendProfile::FillSkeletonBoneDurationsArray): a TimeFactor entry gives a bone the duration
// T x scale (0 = the bone switches at once); a WeightFactor entry gives it T / scale (scale > 0). Bones without an entry keep T.
// An entry applies to its own bone only, not to its children. The per-bone weight is the blend curve (HermiteCubic = smoothstep)
// of min(t / duration_bone, 1). The table names the reference skeleton's bones and maps them onto the rig; reference bones the rig
// does not have (twist and foot IK bones) resolve to nothing, rig bones the reference does not list keep factor 1.
// Header-only, no game hooks.
#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

namespace utpp::mm::blendprofile
{
enum class Mode
{
    TimeFactor,
    WeightFactor
};
struct Entry
{
    std::string_view bone; // reference skeleton bone
    float scale;
};
struct Profile
{
    Mode mode;
    std::span<const Entry> entries;
};

// Locomotion cross-fade of the motion matching node: root 10, legs and feet 1.5, WeightFactor.
inline constexpr Entry kLocomotionEntries[] = {
    {"root", 10.0f},           {"thigh_l", 1.5f},         {"thigh_twist_01_l", 1.5f}, {"calf_l", 1.5f},
    {"calf_twist_01_l", 1.5f}, {"calf_twist_02_l", 1.5f}, {"foot_l", 1.5f},           {"ball_l", 1.5f},
    {"thigh_r", 1.5f},         {"thigh_twist_01_r", 1.5f}, {"calf_r", 1.5f},          {"calf_twist_01_r", 1.5f},
    {"calf_twist_02_r", 1.5f}, {"foot_r", 1.5f},          {"ball_r", 1.5f},           {"ik_foot_root", 1.5f},
    {"ik_foot_l", 1.5f},       {"ik_foot_r", 1.5f}};
inline constexpr Profile kLocomotion{Mode::WeightFactor, kLocomotionEntries};

// Hand-back from an external pose (montage blend-out): root instant, legs and feet 0.75, TimeFactor.
inline constexpr Entry kHandBackEntries[] = {
    {"root", 0.0f},             {"thigh_l", 0.75f},         {"thigh_twist_01_l", 0.75f}, {"calf_l", 0.75f},
    {"calf_twist_01_l", 0.75f}, {"calf_twist_02_l", 0.75f}, {"foot_l", 0.75f},           {"ball_l", 0.75f},
    {"thigh_r", 0.75f},         {"thigh_twist_01_r", 0.75f}, {"calf_r", 0.75f},          {"calf_twist_01_r", 0.75f},
    {"calf_twist_02_r", 0.75f}, {"foot_r", 0.75f},          {"ball_r", 0.75f},           {"ik_foot_root", 0.75f},
    {"ik_foot_l", 0.75f},       {"ik_foot_r", 0.75f}};
inline constexpr Profile kHandBack{Mode::TimeFactor, kHandBackEntries};

// Blend times (s) and interruption rules of the reference motion matching node.
inline constexpr float kBlendGround = 0.5f;         // ground locomotion
inline constexpr float kBlendLandingOrSlide = 0.2f; // after a landing or a slide
inline constexpr float kBlendAirRising = 0.15f;     // rising in the air
inline constexpr float kBlendHandBack = 0.3f;       // montage blend-out (most notifies)
inline constexpr uint32_t kMaxActiveBlends = 4;
inline constexpr float kMaxBlendInTimeToOverride = 0.03f; // a pick replaces a head younger than this instead of stacking

// Reference bone -> rig bones (empty: the rig has no counterpart).
struct RigName
{
    std::string_view reference;
    std::array<std::string_view, 2> rig;
};
inline constexpr RigName kRigNames[] = {
    {"root", {"Root", "Trajectory"}}, {"pelvis", {"Hips", {}}},
    {"thigh_l", {"LeftUpLeg", {}}},   {"calf_l", {"LeftLeg", {}}},
    {"foot_l", {"LeftFoot", {}}},     {"ball_l", {"LeftToeBase", {}}},
    {"thigh_r", {"RightUpLeg", {}}},  {"calf_r", {"RightLeg", {}}},
    {"foot_r", {"RightFoot", {}}},    {"ball_r", {"RightToeBase", {}}},
};

// Duration factor of one entry (duration_bone = T x factor). WeightFactor scale <= 0 or non-finite keeps the full time.
inline float DurationFactor(Mode mode, float scale)
{
    if (!std::isfinite(scale))
        return 1.0f;
    if (mode == Mode::TimeFactor)
        return scale > 0.0f ? scale : 0.0f;
    return scale > 0.0f ? 1.0f / scale : 1.0f;
}

// Per-bone duration factors of a rig (size numBones, rig bone order); boneName(i) returns the rig name of bone i.
template <class NameFn> std::vector<float> Resolve(const Profile &profile, uint32_t numBones, NameFn &&boneName)
{
    std::vector<float> factors(numBones, 1.0f);
    for (const auto &e : profile.entries)
        for (const auto &m : kRigNames)
            if (m.reference == e.bone)
                for (const auto rig : m.rig)
                    if (!rig.empty())
                        for (uint32_t b = 0; b < numBones; ++b)
                            if (std::string_view(boneName(b)) == rig)
                                factors[b] = DurationFactor(profile.mode, e.scale);
    return factors;
}
} // namespace utpp::mm::blendprofile
