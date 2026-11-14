#pragma once
// Melee full pose. Header-only, no RED4ext / Windows types; unit-tested offline (tests/MeleePoseTest.cpp). Runtime glue:
// mm/MeleePose.cpp (FK detour, right after the MM blend, before the chest correction of mm/MeleeBase.hpp).
//
// While V stands and strikes, the game's graph supplies Hips, Spine and the legs from its own base pose, not from the melee clip it
// plays on the upper body. This layer recognises the playing clip frame from the key bones of the game pose (the same matcher as the
// chest correction, on a per-weapon section of a table loaded from a data file) and writes the clip's own lower body (Hips rotation and
// translation, Spine, both legs) time-synchronised with that clip:
//   * a clip clock: once locked on a clip it advances with the elapsed time at the table rate and is pulled softly toward the matched
//     time (the per-frame match time jitters by a fraction of a row; the hips must not), it re-locks (a short cross-fade) when the clip
//     changes or the match is more than kRelockRows away;
//   * a seam guard: when no row matches (the engine's one-frame reference pose at clip seams, a one-frame mismatch) the last written
//     lower pose is held for at most kHoldCalls calls, then the layer fades out;
//   * the weight is the full-body fade of the snapshot (1 - lowerScale: the existing standing gate and its 0.15 s fades, so starting
//     to move hands the legs back to locomotion on the same timeline as before) x the match gate (eases in / out) x the user switch.
// The chest correction keeps running as the fallback, scaled by (1 - the share of the playing clip this layer really writes: its weight
// x the part of a relock cross-fade already on the new time), so the two never stack and never leave a gap.
//
// Data file (little endian; tools/melee/melee_pose_table.py):
//   header 64 bytes: 'UTPPMPOS', u32 format (1), body, keyBones (13), lowerBones (8), weaponCount, clipCount, rowCount, fps, 0
//   weapons  weaponCount x { u64 anim variable CName hash, u32 firstClip, clipCount, firstRow, rowCount }
//   clips    clipCount   x { u32 firstRow, rowCount, char name[40] }
//   keys     rowCount x kRowInts int16   (meleebase row layout: 13 key locals + Spine3 model orientation)
//   lower    rowCount x kLowerInts int16 (8 local quaternions, kLowerNames order, then Hips local translation in mm, pad)
//   clipOf   rowCount x u8 (clip index inside its weapon section), padded to 4

#include <cmath>
#include <cstdint>
#include <cstring>
#include <span>
#include <string_view>
#include <vector>

#include "GameIntegration.hpp"
#include "LegIk.hpp"
#include "MeleeBase.hpp"

namespace utpp::mm::meleepose
{
using aim::Xf;
using math::Quat;
using math::Vec3;
using meleebase::TableData;

inline constexpr int kLowerBones = 8;
inline constexpr int kLowerInts = kLowerBones * 4 + 4;
inline constexpr const char* kLowerNames[kLowerBones] = {"Hips", "Spine", "LeftUpLeg", "LeftLeg", "LeftFoot", "RightUpLeg", "RightLeg", "RightFoot"};
inline constexpr uint32_t kMaxWeapons = 32;
inline constexpr uint32_t kNameBytes = 40;
inline constexpr uint32_t kFormat = 1;

inline constexpr float kRowsPerSecondDefault = 30.0f;
inline constexpr float kPullHz = 12.0f;         // the clip clock follows the matched time with this rate (time constant 83 ms) ...
inline constexpr float kRateGain = 0.5f;        // ... and its rate with this share of the squared position gain
inline constexpr float kRelockBlendSeconds = 0.1f; // a relock (another clip / a jump in time) cross-fades the lower body over this time
inline constexpr float kClockToleranceDeg = 1.5f; // the locked clip at the clock's time is kept while it is at most this much worse than the best row
inline constexpr float kRelockRows = 3.0f;      // a matched time further than this from the clock re-locks it (a restart / jump inside a clip)
inline constexpr int kHoldCalls = 2;            // a lost match holds the last lower pose for at most this many calls
inline constexpr float kGateRiseSeconds = 0.12f; // match gate ease in ...
inline constexpr float kGateFallSeconds = 0.15f; // ... and out (after the hold)
inline constexpr float kMaxDt = 0.1f;
inline constexpr float kVariantToleranceDeg = 1.5f; // a fresh lock on a _hit / _deflect clip takes its base clip while that explains the pose within this
inline constexpr float kStillDegPerSec = 25.0f;     // held states: the upper body counts as still below this key speed (rms, smoothed) ...
inline constexpr float kStillSmoothSeconds = 0.08f; // ... smoothed with this time constant; while still, the clip clock stands

// ------------------------------------------------------------------------------------------------ data file

struct Weapon
{
    uint64_t hash = 0;
    uint32_t firstClip = 0, clipCount = 0, firstRow = 0, rowCount = 0;
    TableData table{};
};

// Parsed view of one body's data file. Owns the bytes; the TableData of every weapon points into them.
struct PoseTables
{
    bool ok = false;
    uint32_t body = 0;
    float rowsPerSecond = kRowsPerSecondDefault;
    uint32_t rowCount = 0;
    std::vector<uint8_t> bytes;
    std::vector<Weapon> weapons;
    std::vector<const char*> clipNames; // global clip index -> name
    std::vector<uint32_t> clipFirstRow, clipRowCount;
    std::vector<int32_t> baseOf; // global clip index -> its base clip ("x_hit" / "x_deflect" -> "x" of the same weapon), -1 = none
    const int16_t* keys = nullptr;
    const int16_t* lower = nullptr;
    const uint8_t* clipOf = nullptr;

    const Weapon* Find(uint64_t aHash) const
    {
        for (const Weapon& w : weapons)
            if (w.hash == aHash)
                return &w;
        return nullptr;
    }
};

template <class T> inline T ReadLe(const uint8_t* aP)
{
    T v;
    std::memcpy(&v, aP, sizeof(T));
    return v;
}

// Parses (and validates) a data file. Fail closed: ok = false for anything inconsistent.
inline bool ParseTables(std::vector<uint8_t> aBytes, PoseTables& aOut)
{
    aOut = PoseTables{};
    aOut.bytes = std::move(aBytes);
    const std::vector<uint8_t>& b = aOut.bytes;
    if (b.size() < 64 || std::memcmp(b.data(), "UTPPMPOS", 8) != 0)
        return false;
    const uint32_t format = ReadLe<uint32_t>(&b[8]), body = ReadLe<uint32_t>(&b[12]), nKeys = ReadLe<uint32_t>(&b[16]),
                   nLower = ReadLe<uint32_t>(&b[20]), nW = ReadLe<uint32_t>(&b[24]), nC = ReadLe<uint32_t>(&b[28]),
                   nR = ReadLe<uint32_t>(&b[32]), fps = ReadLe<uint32_t>(&b[36]);
    if (format != kFormat || body > 1 || nKeys != uint32_t(meleebase::kKeyBones) || nLower != uint32_t(kLowerBones) || nW == 0 ||
        nW > kMaxWeapons || nC == 0 || nC > 65535 || nR == 0 || nR > (1u << 22) || fps < 10 || fps > 120)
        return false;
    const size_t offW = 64, offC = offW + size_t(nW) * 24, offK = offC + size_t(nC) * (8 + kNameBytes);
    const size_t offL = offK + size_t(nR) * meleebase::kRowInts * 2, offO = offL + size_t(nR) * kLowerInts * 2, end = offO + nR;
    if (b.size() < end || (offK % 2) != 0)
        return false;
    aOut.body = body;
    aOut.rowsPerSecond = float(fps);
    aOut.rowCount = nR;
    aOut.keys = reinterpret_cast<const int16_t*>(b.data() + offK);
    aOut.lower = reinterpret_cast<const int16_t*>(b.data() + offL);
    aOut.clipOf = b.data() + offO;
    aOut.clipNames.resize(nC);
    aOut.clipFirstRow.resize(nC);
    aOut.clipRowCount.resize(nC);
    for (uint32_t c = 0; c < nC; ++c)
    {
        const uint8_t* p = b.data() + offC + size_t(c) * (8 + kNameBytes);
        aOut.clipFirstRow[c] = ReadLe<uint32_t>(p);
        aOut.clipRowCount[c] = ReadLe<uint32_t>(p + 4);
        if (p[8 + kNameBytes - 1] != 0 || aOut.clipRowCount[c] == 0 || uint64_t(aOut.clipFirstRow[c]) + aOut.clipRowCount[c] > nR)
            return false;
        aOut.clipNames[c] = reinterpret_cast<const char*>(p + 8);
    }
    uint32_t rowsSeen = 0, clipsSeen = 0;
    for (uint32_t i = 0; i < nW; ++i)
    {
        const uint8_t* p = b.data() + offW + size_t(i) * 24;
        Weapon w;
        w.hash = ReadLe<uint64_t>(p);
        w.firstClip = ReadLe<uint32_t>(p + 8);
        w.clipCount = ReadLe<uint32_t>(p + 12);
        w.firstRow = ReadLe<uint32_t>(p + 16);
        w.rowCount = ReadLe<uint32_t>(p + 20);
        // sections are consecutive, non-empty, at most 255 clips (u8 clip index), and every clip's rows lie inside its section
        if (w.hash == 0 || w.firstClip != clipsSeen || w.firstRow != rowsSeen || w.clipCount == 0 || w.clipCount > 255 || w.rowCount == 0)
            return false;
        uint32_t r = w.firstRow;
        for (uint32_t c = 0; c < w.clipCount; ++c)
        {
            const uint32_t gc = w.firstClip + c;
            if (gc >= nC || aOut.clipFirstRow[gc] != r)
                return false;
            for (uint32_t k = 0; k < aOut.clipRowCount[gc]; ++k)
                if (aOut.clipOf[r + k] != c)
                    return false;
            r += aOut.clipRowCount[gc];
        }
        if (r != w.firstRow + w.rowCount)
            return false;
        rowsSeen = r;
        clipsSeen += w.clipCount;
        aOut.weapons.push_back(w);
    }
    if (rowsSeen != nR || clipsSeen != nC)
        return false;
    aOut.baseOf.assign(nC, -1);
    for (const Weapon& w : aOut.weapons)
        for (uint32_t c = w.firstClip; c < w.firstClip + w.clipCount; ++c)
        {
            const std::string_view n(aOut.clipNames[c]);
            for (const std::string_view suffix : {std::string_view("_hit"), std::string_view("_deflect")})
            {
                if (n.size() <= suffix.size() || n.substr(n.size() - suffix.size()) != suffix)
                    continue;
                const std::string_view base = n.substr(0, n.size() - suffix.size());
                for (uint32_t bc = w.firstClip; bc < w.firstClip + w.clipCount; ++bc)
                    if (std::string_view(aOut.clipNames[bc]) == base)
                        aOut.baseOf[c] = int32_t(bc);
            }
        }
    for (Weapon& w : aOut.weapons)
        w.table = TableData{aOut.keys + size_t(w.firstRow) * meleebase::kRowInts, w.rowCount, aOut.clipOf + w.firstRow, w.clipCount,
                            aOut.clipNames.data() + w.firstClip};
    for (uint32_t r = 0; r < nR * uint32_t(kLowerInts); ++r)
        if (r % kLowerInts == kLowerBones * 4 + 3 && aOut.lower[r] != 0)
            return false; // pad
    aOut.ok = true;
    return true;
}

// ------------------------------------------------------------------------------------------------ rig

inline constexpr int kMaxHipsAncestors = 4;

struct LowerRig
{
    int32_t bone[kLowerBones];
    int32_t anc[kMaxHipsAncestors]; // ancestors of Hips, root first (their locals give the frame the Hips translation is in)
    int32_t ancCount = 0;
    uint32_t ok = 0;
    LowerRig()
    {
        for (int32_t& b : bone)
            b = -1;
        for (int32_t& a : anc)
            a = -1;
    }
};

// Resolves the lower-body bones by name (game CName hash). Fail closed: a missing name or a non-tree parent order -> ok = 0.
inline LowerRig ResolveLowerRig(std::span<const uint64_t> aHashes, std::span<const int16_t> aParents)
{
    LowerRig r;
    if (aHashes.empty() || aParents.size() != aHashes.size())
        return r;
    for (int k = 0; k < kLowerBones; ++k)
    {
        const uint64_t h = integ::CNameHash(kLowerNames[k]);
        for (size_t i = 0; i < aHashes.size(); ++i)
            if (aHashes[i] == h)
            {
                r.bone[k] = int32_t(i);
                break;
            }
        if (r.bone[k] < 0)
            return LowerRig{};
    }
    // the same parents as the clip rig: Spine, UpLegs under Hips, Leg under UpLeg, Foot under Leg
    const int32_t* b = r.bone;
    if (aParents[size_t(b[1])] != b[0] || aParents[size_t(b[2])] != b[0] || aParents[size_t(b[5])] != b[0] || aParents[size_t(b[3])] != b[2] ||
        aParents[size_t(b[4])] != b[3] || aParents[size_t(b[6])] != b[5] || aParents[size_t(b[7])] != b[6])
        return LowerRig{};
    // ancestors of Hips (at most kMaxHipsAncestors; a longer chain or an index out of range fails closed)
    int32_t chain[kMaxHipsAncestors];
    int32_t n = 0;
    for (int32_t p = aParents[size_t(b[0])]; p >= 0; p = aParents[size_t(p)])
    {
        if (n >= kMaxHipsAncestors || size_t(p) >= aParents.size())
            return LowerRig{};
        chain[n++] = p;
    }
    for (int32_t i = 0; i < n; ++i)
        r.anc[i] = chain[n - 1 - i];
    r.ancCount = n;
    r.ok = 1;
    return r;
}

// ------------------------------------------------------------------------------------------------ lower pose sample

struct LowerPose
{
    Quat q[kLowerBones];
    Vec3 hips{};
};

inline LowerPose RowPose(const PoseTables& aT, uint32_t aRow)
{
    LowerPose p;
    const int16_t* r = aT.lower + size_t(aRow) * kLowerInts;
    for (int k = 0; k < kLowerBones; ++k)
        p.q[k] = math::QuatNormalize(meleebase::DecodeQuat(r + k * 4));
    p.hips = Vec3{float(r[kLowerBones * 4]) * 0.001f, float(r[kLowerBones * 4 + 1]) * 0.001f, float(r[kLowerBones * 4 + 2]) * 0.001f};
    return p;
}

inline LowerPose LerpPose(const LowerPose& aA, const LowerPose& aB, float aT)
{
    LowerPose p;
    for (int k = 0; k < kLowerBones; ++k)
        p.q[k] = math::QuatNlerpShortest(aA.q[k], aB.q[k], aT);
    p.hips = aA.hips + (aB.hips - aA.hips) * aT;
    return p;
}

// Relock cross-fade as a decaying offset: the clip as it plays (aClip) plus aK (1 -> 0) of the difference the relock found between the pose
// written before it (aFrom) and the clip at the relock (aBase). Equal to aFrom at the relock; the clip's own motion passes through at once
// (blending toward a frozen aFrom would add the clip's motion since the relock to the catch-up).
inline LowerPose CarryOffset(const LowerPose& aFrom, const LowerPose& aBase, const LowerPose& aClip, float aK)
{
    LowerPose p;
    const Quat id{0.0f, 0.0f, 0.0f, 1.0f};
    for (int k = 0; k < kLowerBones; ++k)
    {
        const Quat d = math::QuatNormalize(math::QuatMul(aFrom.q[k], math::QuatInv(math::QuatNormalize(aBase.q[k]))));
        p.q[k] = math::QuatNormalize(math::QuatMul(math::QuatNlerpShortest(id, d, aK), aClip.q[k]));
    }
    p.hips = aClip.hips + (aFrom.hips - aBase.hips) * aK;
    return p;
}

// Pose of the clip at a fractional row (absolute row index of the file), clamped to the clip's rows.
inline LowerPose SamplePose(const PoseTables& aT, uint32_t aClip, float aRow)
{
    const float first = float(aT.clipFirstRow[aClip]), last = first + float(aT.clipRowCount[aClip] - 1);
    const float p = aRow < first ? first : (aRow > last ? last : aRow);
    const uint32_t i0 = uint32_t(p);
    const uint32_t i1 = float(i0) + 1.0f <= last ? i0 + 1 : i0;
    return LerpPose(RowPose(aT, i0), RowPose(aT, i1), p - float(i0));
}

// rms angle (deg) of the motion keys of the live pose against the clip interpolated at the matched time (between the nearest row
// and its better neighbour): the engine plays 60 fps between the 30 fps rows, so the distance to the nearest row alone reads 6..9 deg
// in a fast strike although the pose is exactly on the clip
inline float PolylineRmsDeg(const TableData& aT, const meleebase::Match& aM, const Quat* aKey)
{
    const int16_t* r0 = aT.frames + size_t(aM.frame) * meleebase::kRowInts;
    const int16_t* r1 = aM.neighbour >= 0 ? aT.frames + size_t(aM.neighbour) * meleebase::kRowInts : r0;
    float sum = 0.0f;
    for (int k = 0; k < meleebase::kMotionKeys; ++k)
    {
        const Quat q = math::QuatNlerpShortest(math::QuatNormalize(meleebase::DecodeQuat(r0 + k * 4)),
                                               math::QuatNormalize(meleebase::DecodeQuat(r1 + k * 4)), aM.t);
        const float d = std::fabs(math::QuatDot(q, math::QuatNormalize(aKey[k])));
        sum += 1.0f - (d > 1.0f ? 1.0f : d);
    }
    return std::sqrt(8.0f * sum / float(meleebase::kMotionKeys)) * meleebase::kRadToDeg;
}

// rms angle (deg) of the motion keys of the live pose against the table at a fractional row (rows of one clip, row-local index of aT)
inline float RowRmsDeg(const TableData& aT, float aRow, const Quat* aKey)
{
    const uint32_t i0 = aRow <= 0.0f ? 0u : uint32_t(aRow);
    const uint32_t i1 = (i0 + 1 < aT.frameCount && aT.clipOfFrame[i0 + 1] == aT.clipOfFrame[i0]) ? i0 + 1 : i0;
    if (i0 >= aT.frameCount)
        return 1e9f;
    meleebase::Match m;
    m.frame = int32_t(i0);
    m.neighbour = int32_t(i1);
    m.t = aim::Clampf(aRow - float(i0), 0.0f, 1.0f);
    return PolylineRmsDeg(aT, m, aKey);
}

// ------------------------------------------------------------------------------------------------ state

struct State
{
    int32_t clip = -1;     // global clip index the clock is locked on (-1 = none)
    float row = 0.0f;      // clock: absolute fractional row
    float rate = 0.0f;     // clock rate (rows per second)
    int32_t hint = -1;     // sticky row hint of the matcher (row inside the weapon section)
    uint64_t weapon = 0;   // hash of the weapon section the state belongs to
    float gate = 0.0f;
    int lost = 0;          // calls since the last match
    int weak = 0;          // consecutive calls whose match was weaker than the gate
    bool havePose = false;
    LowerPose out{};       // last pose returned (held across a lost match)
    LowerPose from{};      // pose the relock cross-fade starts from
    LowerPose fromClip{};  // the clip at the relock (the cross-fade carries from - fromClip and lets it decay)
    float xfade = 1.0f;    // relock cross-fade progress (1 = done)
    float heldTime = 0.0f; // seconds of lost calls since the last match (the clock did not advance meanwhile)
    Quat prevKey[meleebase::kMotionKeys]{}; // motion keys of the previous call (upper body speed)
    bool havePrevKey = false;
    float keySpeed = 0.0f; // smoothed rms angular speed of the motion keys (deg/s)
};

struct StepInfo
{
    bool matched = false, held = false, relocked = false;
    bool toBase = false; // a fresh lock took the base clip instead of its _hit / _deflect variant
    bool frozen = false; // held state with a still upper body: the clip clock stood
    int32_t clip = -1;
    float matchRow = 0.0f, clockRow = 0.0f, rmsDeg = 0.0f, conf = 0.0f, gate = 0.0f, weight = 0.0f;
    // share of the locked clip's own pose in the returned pose (1 - the part still crossing from the previous pose after a relock): the
    // chest correction covers the rest (weight x clipShare is what the layer really writes of the clip the upper body plays)
    float clipShare = 1.0f;
};

// Advances the state by one FK call. aKey = key bone locals of the game pose (kKeyNames order); aWeapon = the drawn weapon's section
// (null: nothing matches, the layer fades out). Returns the share (0..1) with which the clip's lower body is to be written; aPose
// receives that pose. aShare = full-body fade x user switch (0 = the layer is off: the state resets). aHoldState = the melee weapon is
// in a held state (hold, charged hold, block, targeting, deflect): while its upper body is still the clip clock stands (the legs stop with it).
// aPreferBase = a fresh lock on a _hit / _deflect clip takes its base clip while that explains the pose within kVariantToleranceDeg
// (false: the matched clip as it is).
inline float Step(State& aS, const PoseTables& aT, const Weapon* aWeapon, const Quat* aKey, float aShare, float aDt, LowerPose& aPose,
                  StepInfo* aInfo = nullptr, bool aHoldState = false, bool aPreferBase = true)
{
    StepInfo info;
    const float dt = (std::isfinite(aDt) && aDt > 0.0f) ? (aDt > kMaxDt ? kMaxDt : aDt) : 0.0f;
    const float share = std::isfinite(aShare) ? aim::Clampf(aShare, 0.0f, 1.0f) : 0.0f;
    if (!(share > 0.0f) || !aT.ok)
    {
        aS = State{};
        if (aInfo)
            *aInfo = info;
        return 0.0f;
    }
    if (!aWeapon || aWeapon->hash != aS.weapon)
    {
        // another weapon: a new section, the clock and the hint start over (the gate keeps easing)
        aS.clip = -1;
        aS.hint = -1;
        aS.weapon = aWeapon ? aWeapon->hash : 0;
    }
    // upper body speed: rms angle of the motion keys against the previous call, smoothed
    if (aKey)
    {
        if (aS.havePrevKey && dt > 0.0f)
        {
            float sum = 0.0f;
            for (int k = 0; k < meleebase::kMotionKeys; ++k)
            {
                const float d = std::fabs(math::QuatDot(math::QuatNormalize(aKey[k]), aS.prevKey[k]));
                sum += 1.0f - (d > 1.0f ? 1.0f : d);
            }
            const float v = std::sqrt(8.0f * sum / float(meleebase::kMotionKeys)) * meleebase::kRadToDeg / dt;
            aS.keySpeed += (v - aS.keySpeed) * (1.0f - std::exp(-dt / kStillSmoothSeconds));
        }
        else if (!aS.havePrevKey)
            aS.keySpeed = 1e3f; // unknown speed: not still
        for (int k = 0; k < meleebase::kMotionKeys; ++k)
            aS.prevKey[k] = math::QuatNormalize(aKey[k]);
        aS.havePrevKey = true;
    }
    else
        aS.havePrevKey = false;
    const bool frozen = aHoldState && aS.havePrevKey && aS.keySpeed < kStillDegPerSec && aS.clip >= 0;
    const float rateEff = frozen ? 0.0f : aS.rate;
    info.frozen = frozen;
    meleebase::Match m;
    float conf = 0.0f;
    float predicted = -1.0f; // >= 0: the locked clip's predicted row was kept
    int32_t hint = -1;
    if (aWeapon && aKey && meleebase::FindMatch(aWeapon->table, aKey, aS.hint, m) && m.ok)
    {
        hint = m.frame;
        info.rmsDeg = PolylineRmsDeg(aWeapon->table, m, aKey);
        // the locked clip at the clock's predicted time explains the pose as well (within kClockToleranceDeg): keep it. The upper body
        // of a strike is often still while the legs move on (recovery, held guard), and sibling clips share their first frames; the
        // nearest row alone would then jump the lower body to another time or another clip
        if (aS.clip >= 0 && dt > 0.0f && uint32_t(aS.clip) >= aWeapon->firstClip && uint32_t(aS.clip) < aWeapon->firstClip + aWeapon->clipCount)
        {
            const float first = float(aT.clipFirstRow[uint32_t(aS.clip)]), last = first + float(aT.clipRowCount[uint32_t(aS.clip)] - 1);
            const float pred = aim::Clampf(aS.row + rateEff * (dt + aS.heldTime), first, last);
            const float rmsPred = RowRmsDeg(aWeapon->table, pred - float(aWeapon->firstRow), aKey);
            if (rmsPred <= info.rmsDeg + kClockToleranceDeg && rmsPred <= meleebase::kConfFullDeg)
            {
                m.clip = uint32_t(aS.clip) - aWeapon->firstClip;
                m.frame = int32_t(pred - float(aWeapon->firstRow) + 0.5f);
                m.neighbour = -1;
                m.t = 0.0f;
                predicted = pred;
                info.rmsDeg = rmsPred;
                hint = m.frame;
            }
        }
        conf = meleebase::Confidence(info.rmsDeg);
    }
    if (conf > 0.0f)
    {
        int32_t gclip = int32_t(aWeapon->firstClip + m.clip);
        float matchRow = predicted >= 0.0f ? predicted
                                           : float(aWeapon->firstRow) + float(m.frame) + (m.neighbour >= 0 ? m.t * float(m.neighbour - m.frame) : 0.0f);
        // a _hit / _deflect variant shares the start of its base clip; it is taken only while it explains the pose clearly better (the
        // game plays the variant after a hit or a deflect, the upper body then leaves the base clip)
        const int32_t base = (aPreferBase && predicted < 0.0f && size_t(gclip) < aT.baseOf.size()) ? aT.baseOf[size_t(gclip)] : -1;
        if (base >= 0 && aS.clip != gclip)
        {
            const float bFirst = float(aT.clipFirstRow[size_t(base)]);
            const float bRow = bFirst + aim::Clampf(matchRow - float(aT.clipFirstRow[size_t(gclip)]), 0.0f, float(aT.clipRowCount[size_t(base)] - 1));
            const float rmsBase = RowRmsDeg(aWeapon->table, bRow - float(aWeapon->firstRow), aKey);
            if (rmsBase <= info.rmsDeg + kVariantToleranceDeg && rmsBase <= meleebase::kConfFullDeg)
            {
                gclip = base;
                matchRow = bRow;
                info.rmsDeg = rmsBase;
                info.toBase = true;
                conf = meleebase::Confidence(rmsBase);
                aS.hint = int32_t(bRow - float(aWeapon->firstRow) + 0.5f);
                hint = aS.hint;
            }
        }
        info.matchRow = matchRow;
        aS.hint = hint; // only a confident match steers the sticky search (a reference-pose frame must not move it)
        if (aS.clip != gclip || std::fabs(matchRow - aS.row) > kRelockRows)
        {
            // another clip or a jump in time: the difference between what was written and the new clip is carried on the playing clip
            // and decays linearly over kRelockBlendSeconds (sibling clips differ by up to 80 deg in the lower body at a switch);
            // meanwhile StepInfo::clipShare hands the not yet written part of the new clip to the chest correction
            if (aS.havePose)
            {
                aS.from = aS.out;
                aS.xfade = 0.0f;
            }
            aS.clip = gclip;
            aS.row = matchRow;
            aS.rate = aT.rowsPerSecond;
            aS.fromClip = SamplePose(aT, uint32_t(gclip), matchRow);
            info.relocked = true;
        }
        else if (dt > 0.0f)
        {
            // alpha-beta tracker of the matched time: the clock moves at the observed playback rate (the engine may play a clip slower,
            // hold its last frame or blend out), the residual pulls position and rate; no steady-state lag on a held clip end
            const float a = 1.0f - std::exp(-kPullHz * dt);
            const float pred = aS.row + rateEff * (dt + aS.heldTime);
            const float r = matchRow - pred;
            aS.row = pred + a * r;
            if (!frozen) // a standing clock keeps its rate for when the upper body moves on
                aS.rate = aim::Clampf(aS.rate + kRateGain * a * a * r / dt, 0.0f, 2.0f * aT.rowsPerSecond);
        }
        aS.xfade = aS.xfade < 1.0f ? std::fmin(1.0f, aS.xfade + (info.relocked ? 0.0f : dt / kRelockBlendSeconds)) : 1.0f;
        const LowerPose clip = SamplePose(aT, uint32_t(aS.clip), aS.row);
        const float x = aS.xfade; // linear: the relock offset decays at a constant rate (no peak speed mid-way)
        aS.out = aS.xfade < 1.0f ? CarryOffset(aS.from, aS.fromClip, clip, 1.0f - x) : clip;
        aS.havePose = true;
        aS.lost = 0;
        aS.heldTime = 0.0f;
        if (!(dt > 0.0f))
            aS.gate = conf; // first call of an episode: the share (full-body fade) eases it in
        else if (frozen && conf < aS.gate)
        {
            // held state, upper body still: the gate keeps its value (a weaker match of the held guard does not hand the legs back)
        }
        else if (conf >= aS.gate)
        {
            aS.weak = 0;
            aS.gate = std::fmin(conf, aS.gate + dt / kGateRiseSeconds);
        }
        else if (++aS.weak > kHoldCalls)
            aS.gate = std::fmax(conf, aS.gate - dt / kGateFallSeconds);
        // else: a weaker match for at most kHoldCalls calls (a frame between rows, a seam) keeps the gate, like a lost one: the lower body
        // does not dip toward the base pose for a frame (in a lunge that is a hips jump of several cm)
        info.matched = true;
    }
    else
    {
        ++aS.lost;
        if (frozen && aS.havePose)
        {
            // held state, upper body still: the pose the upper body holds may lie between the clip rows (the game layers the guard over
            // it); the legs stay where the clip clock stands instead of fading back to the engine's base pose
            info.held = true;
        }
        else if (aS.lost > kHoldCalls)
        {
            aS.gate = aS.gate - dt / kGateFallSeconds;
            aS.heldTime += dt; // the clock stands from here on: a regained match predicts across the gap
        }
        else if (aS.havePose && aS.clip >= 0 && uint32_t(aS.clip) < aT.clipFirstRow.size())
        {
            // a lost match for at most kHoldCalls calls (the engine's one-frame reference pose at clip seams): the clip keeps playing on
            // the clock instead of freezing, so the next matched frame does not catch up two frames of motion at once
            info.held = true;
            const float first = float(aT.clipFirstRow[uint32_t(aS.clip)]), last = first + float(aT.clipRowCount[uint32_t(aS.clip)] - 1);
            aS.row = aim::Clampf(aS.row + rateEff * dt, first, last);
            aS.xfade = aS.xfade < 1.0f ? std::fmin(1.0f, aS.xfade + dt / kRelockBlendSeconds) : 1.0f;
            const LowerPose clip = SamplePose(aT, uint32_t(aS.clip), aS.row);
            const float x = aS.xfade;
            aS.out = aS.xfade < 1.0f ? CarryOffset(aS.from, aS.fromClip, clip, 1.0f - x) : clip;
        }
    }
    aS.gate = aim::Clampf(aS.gate, 0.0f, 1.0f);
    info.clip = aS.clip;
    info.clockRow = aS.row;
    info.conf = conf;
    info.gate = aS.gate;
    info.clipShare = aS.xfade < 1.0f ? aS.xfade : 1.0f;
    const float w = aS.havePose ? share * aS.gate : 0.0f;
    info.weight = w;
    aPose = aS.out;
    if (aInfo)
        *aInfo = info;
    return w;
}

// Writes aPose into the local records with weight aW (rotations nlerp, Hips translation lerp). False (records untouched) for a bad
// rig / record.
inline bool WriteLower(uint8_t* aBones, uint32_t aCount, const LowerRig& aRig, const LowerPose& aPose, float aW)
{
    if (!aRig.ok || !aBones || !(aW > 0.0f))
        return false;
    for (int k = 0; k < kLowerBones; ++k)
        if (aRig.bone[k] < 0 || uint32_t(aRig.bone[k]) >= aCount)
            return false;
    const float w = aW > 1.0f ? 1.0f : aW;
    for (int k = 0; k < kLowerBones; ++k)
    {
        Xf l = aim::ReadLocal(aBones, uint32_t(aRig.bone[k]));
        if (!math::IsFinite(l.q) || !math::IsFinite(l.p))
            return false;
        aim::WriteLocalRot(aBones, uint32_t(aRig.bone[k]), math::QuatNlerpShortest(math::QuatNormalize(l.q), aPose.q[k], w));
        if (k == 0)
        {
            const Vec3 p = l.p + (aPose.hips - l.p) * w;
            float* r = reinterpret_cast<float*>(aBones + size_t(aRig.bone[k]) * integ::kGameBoneStride);
            r[0] = p.x;
            r[1] = p.y;
            r[2] = p.z;
        }
    }
    return true;
}

// ------------------------------------------------------------------------------------------------ borders: feet and Hips height
//
// Runs on the written lower body (after WriteLower), from the first call the layer writes until shortly after it stopped writing.
// Frame: the parent of Hips (model space of the character, z up; the capsule stands while the layer runs).
//   * Hips height: while the layer's weight is below 1 and after it stopped, the Hips height moves toward the written height at most
//     kHipsSpeed; at full weight a remaining offset decays at the same speed (the clip's own motion passes unchanged).
//   * Feet: each ankle is planted where it stood when the layer started (the pose before the first write) and held there by a two-bone
//     leg IK while the written ankle stays within kLiftHeight of the plant height. A written ankle that rises above that height is a
//     real step of the clip: the foot follows it and is planted again where it lands. A written ankle that moves more than
//     kStepDistance along the ground (a stance change without lift) gets a short step with lift toward it. When the layer stops
//     writing, a planted foot more than kSlideTolerance from the incoming pose steps there; a closer one is released over
//     kReleaseSeconds. The plants live in the character frame: while the capsule moves (an attack that carries V forward) no foot is
//     held, the written pose is shown as it is (a release fades over kReleaseSeconds) and the feet are planted again when it stands.

inline constexpr float kHipsSpeed = 0.35f;       // m/s, vertical speed of the Hips height offset in the fades
inline constexpr float kLiftHeight = 0.03f;      // m, written ankle above its plant height = the foot is lifted
inline constexpr float kStepDistance = 0.10f;    // m, horizontal distance of the written ankle from the plant that starts a step
inline constexpr float kSlideTolerance = 0.03f;  // m, at the end a planted foot this close to the incoming pose is released
inline constexpr float kStepSpeed = 0.8f;        // m/s, horizontal speed of a step (no faster than the clips' own feet)
inline constexpr float kStepLiftShare = 0.25f;   // step height = this share of the step length ...
inline constexpr float kStepLiftMin = 0.045f, kStepLiftMax = 0.07f; // ... clamped (m; above kLiftHeight: a step never moves on the ground)
inline constexpr float kStepUpSeconds = 0.06f;   // a step rises to its height in place and sets down in place in this time
inline constexpr float kReleaseSeconds = 0.1f;   // a released foot reaches the written ankle in this time
inline constexpr float kTailSeconds = 0.6f;      // after the last write: the borders end at the latest after this time
inline constexpr float kCapsuleMoveSpeed = 0.3f; // m/s, planar capsule speed above which no foot is held (the plants are in the character frame)
inline constexpr float kLegMaxExtension = 0.995f, kLegSoftExtension = 0.97f;

enum class FootMode : uint8_t
{
    Free,     // follows the written ankle (a release fades in over kReleaseSeconds)
    Planted,  // held at lock
    Stepping  // rises in place, moves toward the written ankle at kStepSpeed, sets down in place
};

struct FootState
{
    FootMode mode = FootMode::Free;
    Vec3 lock{}, from{};
    float lift = 0.0f, h = 0.0f; // Stepping: step height, current height above 'from'
    uint8_t phase = 0;           // Stepping: 0 rise, 1 move, 2 set down
    float release = 1.0f;  // Free: progress of the release from 'from' (1 = done)
    Vec3 prevIn{};         // written ankle of the previous call
    bool havePrev = false;
};

struct BorderState
{
    bool active = false;
    bool writing = false;   // the layer wrote on the previous call
    float tail = 0.0f;      // seconds since the layer stopped writing
    float hipsOut = 0.0f;   // Hips height (local z) shown on the previous call
    float hipsOffset = 0.0f; // shown - written
    float floorZ = 0.0f;    // lowest plant height (the ground in the parent frame of Hips)
    FootState foot[2];
};

struct BorderInfo
{
    bool active = false, hipsLimited = false, stepStarted = false;
    int planted = 0; // feet held at their plant this call
    float hipsOffset = 0.0f;
};

// Model transforms of the lower chain from the local records: parent of Hips (root first through the ancestors), Hips, and per leg
// UpLeg, Leg, Foot. False for a bad rig / record.
struct LowerModel
{
    mm::BoneTransform parent, hips, leg[2][3];
    float hipsLocalZ = 0.0f; // Hips translation z in its parent frame (the record)
};

inline bool ComputeLowerModel(const uint8_t* aBones, uint32_t aCount, const LowerRig& aRig, LowerModel& aOut)
{
    if (!aRig.ok || !aBones)
        return false;
    for (int k = 0; k < kLowerBones; ++k)
        if (aRig.bone[k] < 0 || uint32_t(aRig.bone[k]) >= aCount)
            return false;
    mm::BoneTransform m{Vec3{}, Quat{0.0f, 0.0f, 0.0f, 1.0f}};
    for (int32_t i = 0; i < aRig.ancCount; ++i)
    {
        if (aRig.anc[i] < 0 || uint32_t(aRig.anc[i]) >= aCount)
            return false;
        const Xf l = aim::ReadLocal(aBones, uint32_t(aRig.anc[i]));
        m.p = m.p + math::QuatRotate(m.q, l.p);
        m.q = math::QuatNormalize(math::QuatMul(m.q, l.q));
    }
    aOut.parent = m;
    auto child = [&](const mm::BoneTransform& aP, int32_t aBone) {
        const Xf l = aim::ReadLocal(aBones, uint32_t(aBone));
        return mm::BoneTransform{aP.p + math::QuatRotate(aP.q, l.p), math::QuatNormalize(math::QuatMul(aP.q, l.q))};
    };
    aOut.hips = child(m, aRig.bone[0]);
    aOut.hipsLocalZ = aim::ReadLocal(aBones, uint32_t(aRig.bone[0])).p.z;
    for (int s = 0; s < 2; ++s)
    {
        aOut.leg[s][0] = child(aOut.hips, aRig.bone[2 + 3 * s]);
        aOut.leg[s][1] = child(aOut.leg[s][0], aRig.bone[3 + 3 * s]);
        aOut.leg[s][2] = child(aOut.leg[s][1], aRig.bone[4 + 3 * s]);
    }
    return math::IsFinite(aOut.hips.p) && math::IsFinite(aOut.leg[0][2].p) && math::IsFinite(aOut.leg[1][2].p);
}

inline float Smooth01(float aX)
{
    const float x = aX < 0.0f ? 0.0f : (aX > 1.0f ? 1.0f : aX);
    return x * x * (3.0f - 2.0f * x);
}

inline float MoveToward(float aFrom, float aTo, float aMaxStep)
{
    const float d = aTo - aFrom;
    return d > aMaxStep ? aFrom + aMaxStep : (d < -aMaxStep ? aFrom - aMaxStep : aTo);
}

inline float Horizontal(Vec3 aA, Vec3 aB) { return std::sqrt((aA.x - aB.x) * (aA.x - aB.x) + (aA.y - aB.y) * (aA.y - aB.y)); }

inline void StartStep(FootState& aF, Vec3 aFrom, Vec3 aTo)
{
    const float d = Horizontal(aFrom, aTo);
    aF.mode = FootMode::Stepping;
    aF.from = aFrom;
    aF.lift = aim::Clampf(d * kStepLiftShare, kStepLiftMin, kStepLiftMax);
    aF.h = 0.0f;
    aF.phase = 0;
}

// One FK call. aWriting = the layer wrote this call (weight aWeight > 0); aPre = the lower chain before the layer's write of this call
// (used when the borders start). Returns true while the borders are active (records possibly changed). The records keep their bone
// translations except the Hips height; only UpLeg / Leg / Foot rotations change for the feet.
inline bool BorderStep(BorderState& aS, uint8_t* aBones, uint32_t aCount, const LowerRig& aRig, const LowerModel* aPre, bool aWriting,
                       float aWeight, float aDt, BorderInfo* aInfo = nullptr, bool aCapsuleMoving = false)
{
    BorderInfo info;
    const float dt = (std::isfinite(aDt) && aDt > 0.0f) ? (aDt > kMaxDt ? kMaxDt : aDt) : 0.0f;
    LowerModel in;
    if ((!aWriting && !aS.active) || !ComputeLowerModel(aBones, aCount, aRig, in))
    {
        aS = BorderState{};
        if (aInfo)
            *aInfo = info;
        return false;
    }
    const float* hipsRec = reinterpret_cast<const float*>(aBones + size_t(aRig.bone[0]) * integ::kGameBoneStride);
    const float writtenZ = hipsRec[2];
    Vec3 inAnkle[2] = {in.leg[0][2].p, in.leg[1][2].p};
    if (!aS.active)
    {
        // start: the feet and the Hips height where the pose before the first write had them
        aS = BorderState{};
        aS.active = true;
        const LowerModel& p = aPre ? *aPre : in;
        for (int s = 0; s < 2; ++s)
        {
            aS.foot[s].mode = FootMode::Planted;
            aS.foot[s].lock = p.leg[s][2].p;
        }
        aS.floorZ = std::fmin(aS.foot[0].lock.z, aS.foot[1].lock.z);
        aS.hipsOut = p.hipsLocalZ;
        aS.hipsOffset = aS.hipsOut - writtenZ;
    }
    const bool ending = !aWriting && aS.writing;
    if (!aWriting)
        aS.tail += dt;
    else
        aS.tail = 0.0f;
    // Hips height
    const bool fading = !aWriting || !(aWeight >= 0.999f);
    float outZ;
    if (fading)
        outZ = MoveToward(aS.hipsOut, writtenZ, kHipsSpeed * dt);
    else
        outZ = writtenZ + MoveToward(aS.hipsOffset, 0.0f, kHipsSpeed * dt);
    info.hipsLimited = std::fabs(outZ - writtenZ) > 1e-4f;
    aS.hipsOut = outZ;
    aS.hipsOffset = outZ - writtenZ;
    info.hipsOffset = aS.hipsOffset;
    {
        float* r = reinterpret_cast<float*>(aBones + size_t(aRig.bone[0]) * integ::kGameBoneStride);
        r[2] = outZ;
    }
    LowerModel cur;
    if (!ComputeLowerModel(aBones, aCount, aRig, cur))
    {
        aS = BorderState{};
        if (aInfo)
            *aInfo = info;
        return false;
    }
    bool anyCorrection = info.hipsLimited;
    for (int s = 0; s < 2; ++s)
    {
        FootState& f = aS.foot[s];
        const Vec3 w = inAnkle[s];
        f.prevIn = w;
        f.havePrev = true;
        if (ending && f.mode == FootMode::Planted)
        {
            if (Horizontal(w, f.lock) > kSlideTolerance || std::fabs(w.z - f.lock.z) > kSlideTolerance)
            {
                StartStep(f, f.lock, w);
                info.stepStarted = true;
            }
            else
            {
                f.mode = FootMode::Free;
                f.from = f.lock;
                f.release = 0.0f;
            }
        }
        if (aCapsuleMoving && f.mode == FootMode::Planted)
        {
            f.mode = FootMode::Free;
            f.from = f.lock;
            f.release = 0.0f;
        }
        Vec3 target = w;
        switch (f.mode)
        {
        case FootMode::Planted:
            if (w.z > f.lock.z + kLiftHeight)
            {
                // the clip lifts the foot: a real step. A foot far from its plant joins it with a step of its own.
                if (Horizontal(w, f.lock) > kSlideTolerance)
                {
                    StartStep(f, f.lock, w);
                    info.stepStarted = true;
                }
                else
                {
                    f.mode = FootMode::Free;
                    f.from = f.lock;
                    f.release = 0.0f;
                }
            }
            else if (Horizontal(w, f.lock) > kStepDistance)
            {
                StartStep(f, f.lock, w);
                info.stepStarted = true;
            }
            else
            {
                target = f.lock;
                ++info.planted;
            }
            break;
        case FootMode::Free:
            if (aWriting && !aCapsuleMoving && f.release >= 1.0f && w.z <= aS.floorZ + kLiftHeight)
            {
                f.mode = FootMode::Planted;
                f.lock = w;
                aS.floorZ = std::fmin(aS.floorZ, w.z);
                target = w;
                ++info.planted;
            }
            break;
        case FootMode::Stepping:
            break;
        }
        if (f.mode == FootMode::Stepping)
        {
            // rise in place, move the ground point toward the written ankle at step speed (it may move on meanwhile), set down in place.
            // The foot moves no faster than kStepSpeed in 3D: the rise, the move along the ground and the height change share that speed
            const float maxStep = kStepSpeed * dt, maxRise = std::fmin(f.lift / kStepUpSeconds, kStepSpeed) * dt;
            float used = 0.0f;
            if (f.phase == 0)
            {
                const float h0 = f.h;
                f.h = std::fmin(f.lift, f.h + maxRise);
                used = f.h - h0;
                if (f.h >= f.lift)
                    f.phase = 1;
            }
            else if (f.phase == 1)
            {
                const float rem0 = Horizontal(w, f.from), dz = w.z - f.from.z;
                const float total = std::sqrt(rem0 * rem0 + dz * dz);
                const float k = total > maxStep ? maxStep / total : 1.0f;
                f.from.x += (w.x - f.from.x) * k;
                f.from.y += (w.y - f.from.y) * k;
                f.from.z += dz * k;
                used = maxStep;
                if (Horizontal(w, f.from) < 1e-3f)
                    f.phase = 2;
            }
            else
            {
                const float h0 = f.h;
                f.h = std::fmax(0.0f, f.h - maxRise);
                used = h0 - f.h;
            }
            if (f.phase != 1)
            {
                const float rest = maxStep * maxStep - used * used;
                f.from.z = MoveToward(f.from.z, w.z, rest > 0.0f ? std::sqrt(rest) : 0.0f);
            }
            target = f.from;
            target.z += f.h;
            if (f.phase == 2 && f.h <= 0.0f)
            {
                // set down: planted where it landed (while the layer writes), else released toward the incoming pose
                if (aWriting)
                {
                    f.mode = FootMode::Planted;
                    f.lock = f.from;
                }
                else
                {
                    f.mode = FootMode::Free;
                    f.release = 0.0f;
                }
                target = f.from;
            }
        }
        else if (f.mode == FootMode::Free && f.release < 1.0f)
        {
            f.release = std::fmin(1.0f, f.release + (dt > 0.0f ? dt / kReleaseSeconds : 1.0f));
            target = f.from + (w - f.from) * Smooth01(f.release);
        }
        // IK whenever the shown ankle is not on the target (also a free foot under a Hips height offset)
        const Vec3 c = cur.leg[s][2].p;
        const Vec3 d = target - c;
        if (math::Dot(d, d) > 1e-8f)
        {
            const legik::TwoBoneSolution sol = legik::SolveTwoBone(cur.hips, cur.leg[s][0], cur.leg[s][1], cur.leg[s][2], target,
                                                                    cur.leg[s][2].p + math::QuatRotate(cur.leg[s][2].q, Vec3{0.0f, 0.1f, 0.0f}),
                                                                    kLegMaxExtension, kLegSoftExtension, true, true);
            if (sol.status == legik::TwoBoneStatus::kOk)
            {
                for (int j = 0; j < 3; ++j)
                    aim::WriteLocalRot(aBones, uint32_t(aRig.bone[2 + j + 3 * s]), sol.local[j]);
                anyCorrection = true;
            }
        }
        if (f.mode != FootMode::Free || f.release < 1.0f)
            anyCorrection = true;
    }
    aS.writing = aWriting;
    info.active = true;
    if (!aWriting && (!anyCorrection || aS.tail >= kTailSeconds))
        aS = BorderState{}; // done: the next call shows the incoming pose unchanged
    if (aInfo)
        *aInfo = info;
    return true;
}
} // namespace utpp::mm::meleepose
