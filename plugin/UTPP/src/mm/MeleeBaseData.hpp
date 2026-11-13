#pragma once
// Fists clip table for the melee chest base correction (mm/MeleeBase.hpp). The data is generated from the clips that ship in the
// melee archive (tools/melee/fist_base_table.py -> MeleeBaseData.cpp), one table per body.
//
// Row layout (kRowInts int16, normalised quaternions xyzw, w >= 0, scaled by 32767):
//   [0, kKeyBones)      local rotation of each key bone (kKeyNames order) as the clip plays it
//   [kKeyBones]         the clip's Spine3 orientation in model space (the chest as authored on the clip's own hips and spine)
// Frames of one clip are consecutive; clipOfFrame[i] is the index into clipNames.

#include <cstdint>

namespace utpp::mm::meleebase
{
inline constexpr int kKeyBones = 13;
inline constexpr int kRowInts = (kKeyBones + 1) * 4;
inline constexpr const char* kKeyNames[kKeyBones] = {"Neck",         "Head",     "RightShoulder", "RightArm", "RightForeArm", "RightHand",
                                                    "LeftShoulder", "LeftArm",  "LeftForeArm",   "LeftHand", "Spine1",       "Spine2",        "Spine3"};
// weight of each key bone in the match: the arms / neck / head are the clip's own motion (1); the spine locals separate rows whose arms
// are equal but whose trunk (and so chest) differs, but the game adds a small constant offset to them (FPP additive: Female V Spine3
// ~7 deg), so they count less
inline constexpr float kKeyWeight[kKeyBones] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.5f, 0.5f, 0.2f};
inline constexpr float kKeyWeightSum = 11.2f;
inline constexpr int kMotionKeys = 10; // keys [0, kMotionKeys) are the clip's own motion (neck, head, clavicles, arms); the rest are spine locals
// the order the matcher compares the key bones in: the arms first (they differ most between frames), so most rows are rejected early
inline constexpr int kKeyOrder[kKeyBones] = {3, 7, 4, 8, 5, 9, 10, 11, 12, 2, 6, 0, 1};

struct TableData
{
    const int16_t* frames;
    uint32_t frameCount;
    const uint8_t* clipOfFrame;
    uint32_t clipCount;
    const char* const* clipNames;
};

const TableData& FemaleTable();
const TableData& MaleTable();
} // namespace utpp::mm::meleebase
