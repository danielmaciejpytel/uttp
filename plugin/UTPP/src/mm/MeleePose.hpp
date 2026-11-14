#pragma once
// Melee full pose. Header-only, no RED4ext / Windows types; unit-tested offline (tests/MeleePoseTest.cpp). Runtime glue:
// mm/MeleePose.cpp (FK detour, right after the MM blend, before the chest correction of mm/MeleeBase.hpp).
//
// While V stands and strikes, the game's graph supplies Hips, Spine and the legs from its own base pose, not from the melee clip it
// plays on the upper body. This layer recognises the playing clip frame from the key bones of the game pose (the same matcher as the
// chest correction, on a per-weapon section of a table loaded from a data file) and writes the clip's own lower body (Hips rotation and
// translation, Spine, both legs) time-synchronised with that clip:
//   * a clip clock: once locked on a clip it advances with the elapsed time at the table rate and is pulled softly toward the matched
//     time (the per-frame match time jitters by a fraction of a row; the hips must not), it re-locks when the clip changes or the match
//     is more than kRelockRows away;
//   * a seam guard: when no row matches (the engine's one-frame reference pose at clip seams, a one-frame mismatch) the last written
//     lower pose is held for at most kHoldCalls calls, then the layer fades out;
//   * the weight is the full-body fade of the snapshot (1 - lowerScale: the existing standing gate and its 0.15 s fades, so starting
//     to move hands the legs back to locomotion on the same timeline as before) x the match gate (eases in / out) x the user switch.
// The chest correction keeps running as the fallback, scaled by (1 - the weight of this layer), so the two never stack.
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

struct LowerRig
{
    int32_t bone[kLowerBones];
    uint32_t ok = 0;
    LowerRig()
    {
        for (int32_t& b : bone)
            b = -1;
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
    bool havePose = false;
    LowerPose out{};       // last pose returned (held across a lost match)
    LowerPose from{};      // pose the relock cross-fade starts from
    float xfade = 1.0f;    // relock cross-fade progress (1 = done)
    float heldTime = 0.0f; // seconds of lost calls since the last match (the clock did not advance meanwhile)
};

struct StepInfo
{
    bool matched = false, held = false, relocked = false;
    int32_t clip = -1;
    float matchRow = 0.0f, clockRow = 0.0f, rmsDeg = 0.0f, conf = 0.0f, gate = 0.0f, weight = 0.0f;
};

// Advances the state by one FK call. aKey = key bone locals of the game pose (kKeyNames order); aWeapon = the drawn weapon's section
// (null: nothing matches, the layer fades out). Returns the share (0..1) with which the clip's lower body is to be written; aPose
// receives that pose. aShare = full-body fade x user switch (0 = the layer is off: the state resets).
inline float Step(State& aS, const PoseTables& aT, const Weapon* aWeapon, const Quat* aKey, float aShare, float aDt, LowerPose& aPose,
                  StepInfo* aInfo = nullptr)
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
            const float pred = aim::Clampf(aS.row + aS.rate * (dt + aS.heldTime), first, last);
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
        const int32_t gclip = int32_t(aWeapon->firstClip + m.clip);
        const float matchRow = predicted >= 0.0f ? predicted
                                                 : float(aWeapon->firstRow) + float(m.frame) + (m.neighbour >= 0 ? m.t * float(m.neighbour - m.frame) : 0.0f);
        info.matchRow = matchRow;
        aS.hint = hint; // only a confident match steers the sticky search (a reference-pose frame must not move it)
        if (aS.clip != gclip || std::fabs(matchRow - aS.row) > kRelockRows)
        {
            // another clip or a jump in time: the lower body crosses from what was written over kRelockBlendSeconds
            if (aS.havePose)
            {
                aS.from = aS.out;
                aS.xfade = 0.0f;
            }
            aS.clip = gclip;
            aS.row = matchRow;
            aS.rate = aT.rowsPerSecond;
            info.relocked = true;
        }
        else if (dt > 0.0f)
        {
            // alpha-beta tracker of the matched time: the clock moves at the observed playback rate (the engine may play a clip slower,
            // hold its last frame or blend out), the residual pulls position and rate; no steady-state lag on a held clip end
            const float a = 1.0f - std::exp(-kPullHz * dt);
            const float pred = aS.row + aS.rate * (dt + aS.heldTime);
            const float r = matchRow - pred;
            aS.row = pred + a * r;
            aS.rate = aim::Clampf(aS.rate + kRateGain * a * a * r / dt, 0.0f, 2.0f * aT.rowsPerSecond);
        }
        aS.xfade = aS.xfade < 1.0f ? std::fmin(1.0f, aS.xfade + (info.relocked ? 0.0f : dt / kRelockBlendSeconds)) : 1.0f;
        const LowerPose clip = SamplePose(aT, uint32_t(aS.clip), aS.row);
        const float x = aS.xfade * aS.xfade * (3.0f - 2.0f * aS.xfade);
        aS.out = aS.xfade < 1.0f ? LerpPose(aS.from, clip, x) : clip;
        aS.havePose = true;
        aS.lost = 0;
        aS.heldTime = 0.0f;
        if (!(dt > 0.0f))
            aS.gate = conf; // first call of an episode: the share (full-body fade) eases it in
        else if (aS.gate < conf)
            aS.gate = std::fmin(conf, aS.gate + dt / kGateRiseSeconds);
        else
            aS.gate = std::fmax(conf, aS.gate - dt / kGateFallSeconds);
        info.matched = true;
    }
    else
    {
        ++aS.lost;
        if (aS.lost > kHoldCalls)
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
            aS.row = aim::Clampf(aS.row + aS.rate * dt, first, last);
            aS.xfade = aS.xfade < 1.0f ? std::fmin(1.0f, aS.xfade + dt / kRelockBlendSeconds) : 1.0f;
            const LowerPose clip = SamplePose(aT, uint32_t(aS.clip), aS.row);
            const float x = aS.xfade * aS.xfade * (3.0f - 2.0f * aS.xfade);
            aS.out = aS.xfade < 1.0f ? LerpPose(aS.from, clip, x) : clip;
        }
    }
    aS.gate = aim::Clampf(aS.gate, 0.0f, 1.0f);
    info.clip = aS.clip;
    info.clockRow = aS.row;
    info.conf = conf;
    info.gate = aS.gate;
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
} // namespace utpp::mm::meleepose
