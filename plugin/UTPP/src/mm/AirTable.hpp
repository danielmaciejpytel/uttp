// UTPP air (jump / fall / land) clip table. Header-only, no globals, no per-frame allocation (all buffers are sized in
// Parse / FromDatabase; Pick, RootZAt and LegPoseCost do not allocate).
//
// 1. Parse(bytes, db, dbSha256, reason): reads the .airz root_z sidecar written by Source tools make_airz.py
//    (little-endian: char[8] "UTPPAIRZ", u32 version 1, u8[32] SHA-256 of the .utppmm bytes, u32 clipCount, per clip in
//    ascending clipIndex: u32 clipIndex, u16 nameLen, UTF-8 name, u32 numFrames, f32 root_z[numFrames]).
//    File-level failures (magic, version, SHA, truncation, clip index / order / name / frame count, non-finite value,
//    trailing bytes) reject the whole table with an exact reason string. Per clip, (Hips z - root_z) must stay in
//    [kHipsMin, kHipsMax] m on every frame, otherwise only that clip is rejected (root_z stays 0 = today's behaviour).
//    A clip whose POSE is already RTZ-clean (raw Hips z in range on the whole clip and subtracting root_z would ADD
//    vertical variance instead of removing it) is detected and its root_z is NOT applied (no double subtraction);
//    only the metadata is kept.
// 2. RootZ(): per global frame root_z (size = db.NumFrames(), 0 for frames without data), ready for a future
//    BaseMatcher::SetRootZ.
// 3. Per clip metadata for every DB clip whose name parses as a motion jump clip (kind Start / FallLoop / Land / Off,
//    name direction F/B/L/R, travel direction from the summed ROOT deltas, band, weight, foot, event frames).
// 4. Pick(kind, dir, band, weight, foot): nearest available clip with an EXPLICIT, deterministic fallback order (see
//    Pick). Documented v6 gaps it resolves:
//      - only F starts have band variants; B / L / R starts have foot variants only (band ignored),
//      - B / L / R lands have no Sprint (-> Run, BandRelaxed),
//      - Walk / Run lands of B / L / R have no foot variant (foot ignored, not flagged),
//      - F_Land_Walk_Heavy exists only as Lfoot (Rfoot request -> Lfoot, FootRelaxed),
//      - Off exists only as F Walk / Run (B / L / R -> F with DirRelaxed, Stand -> Walk, Sprint -> Run).
// 5. LegPoseCost(poseA, poseB, chain): sum of local rotation angles (rad) between two poses over a bone chain
//    (UpLeg / Leg / Foot of both legs), used to choose a land / start clip matching the current leg pose.
// 6. Jump_Loop_Fall loop check: frame N-1 vs 0, every bone rotation within 1 deg and every non-root bone position
//    within 1 cm after root_z correction of Hips -> LoopOk().
// 7. FromDatabase(db): stub for the future .utppmm v2 RTZ_ section (returns false with a reason today).
//
// The Db type is BaseDatabase.hpp utpp::mm::Database or any type with the same accessors: NumFrames, NumBones,
// NumClips, Clips()[c].firstFrame/.numFrames, ClipName, BoneName, Pose (px,py,pz,qx,qy,qz,qw per bone), Root (dx,dy
// per frame, may return nullptr), Contacts (array<float,2>).
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace utpp::mm::airtab
{
inline constexpr char kAirzMagic[8] = {'U', 'T', 'P', 'P', 'A', 'I', 'R', 'Z'};
inline constexpr uint32_t kAirzVersion = 1;
inline constexpr float kHipsMin = 0.30f;
inline constexpr float kHipsMax = 1.40f;
inline constexpr float kLoopMaxDeg = 1.0f;
inline constexpr float kLoopMaxM = 0.01f;
inline constexpr float kAirborneEps = 0.02f; // m of root_z above take-off level counted as airborne
inline constexpr float kDropEps = 0.05f;     // m below take-off level counted as a drop (F_Off)
inline constexpr float kContactOn = 0.5f;

enum class Kind : uint8_t { None, Start, FallLoop, Land, Off };
enum class Dir : uint8_t { None, F, B, L, R };
enum class Band : uint8_t { None, Stand, Walk, Run, Sprint };
enum class Weight : uint8_t { None, Light, Heavy };
enum class Foot : uint8_t { None, L, R };

enum class ClipStatus : uint8_t
{
    NoData,       // no .airz entry: root_z 0, metadata only
    Applied,      // root_z applied
    RtzClean,     // POSE already RTZ-clean: root_z not applied, metadata only
    RejectedRange // (Hips z - root_z) out of [kHipsMin, kHipsMax]: root_z not applied
};

struct ClipMeta
{
    int32_t clip = -1;
    uint32_t firstFrame = 0;
    uint32_t numFrames = 0;
    Kind kind = Kind::None;
    Dir nameDir = Dir::None;
    Dir travelDir = Dir::None; // from summed ROOT deltas (+y forward, +x right)
    Band band = Band::None;
    Weight weight = Weight::None;
    Foot foot = Foot::None;
    ClipStatus status = ClipStatus::NoData;
    float travelX = 0.0f;
    float travelY = 0.0f;
    float maxRootZ = 0.0f;
    float minHips = 0.0f; // min / max of the corrected Hips z (raw when no data / clean)
    float maxHips = 0.0f;
    int32_t liftoff = -1;   // clip-local frame of contact loss (Start / Off)
    int32_t apex = -1;      // clip-local frame of max root_z height (Start / FallLoop / Land), -1 if never airborne
    int32_t zeroCross = -1; // first frame after apex where height returns to take-off level
    int32_t touch = -1;     // first frame of foot contact after an airborne phase (Land)
    int32_t drop = -1;      // first frame below take-off level by kDropEps (Off)
};

struct PickResult
{
    enum : uint32_t { kExact = 0, kFootRelaxed = 1, kBandRelaxed = 2, kWeightRelaxed = 4, kDirRelaxed = 8 };
    int32_t clip = -1;
    int32_t meta = -1;
    uint32_t relaxed = kExact;
};

// ---------------------------------------------------------------- SHA-256 (for the .utppmm hash)

class Sha256
{
public:
    void Update(std::span<const uint8_t> aData)
    {
        for (const uint8_t b : aData)
        {
            m_buf[m_len++] = b;
            if (m_len == 64)
            {
                Block(m_buf.data());
                m_len = 0;
            }
            m_bits += 8;
        }
    }

    std::array<uint8_t, 32> Final()
    {
        const uint64_t bits = m_bits;
        const uint8_t pad80 = 0x80, zero = 0;
        Update(std::span<const uint8_t>(&pad80, 1));
        while (m_len != 56)
            Update(std::span<const uint8_t>(&zero, 1));
        std::array<uint8_t, 8> len{};
        for (int i = 0; i < 8; ++i)
            len[size_t(i)] = uint8_t(bits >> (56 - 8 * i));
        Update(len);
        std::array<uint8_t, 32> out{};
        for (int i = 0; i < 8; ++i)
            for (int j = 0; j < 4; ++j)
                out[size_t(i * 4 + j)] = uint8_t(m_h[size_t(i)] >> (24 - 8 * j));
        return out;
    }

    static std::array<uint8_t, 32> Of(std::span<const uint8_t> aData)
    {
        Sha256 s;
        s.Update(aData);
        return s.Final();
    }

private:
    static uint32_t Rotr(uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }

    void Block(const uint8_t* p)
    {
        static constexpr uint32_t k[64] = {
            0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5, 0xd807aa98,
            0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786,
            0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8,
            0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
            0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819,
            0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a,
            0x5b9cca4f, 0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7,
            0xc67178f2};
        uint32_t w[64];
        for (int i = 0; i < 16; ++i)
            w[i] = uint32_t(p[i * 4]) << 24 | uint32_t(p[i * 4 + 1]) << 16 | uint32_t(p[i * 4 + 2]) << 8 | uint32_t(p[i * 4 + 3]);
        for (int i = 16; i < 64; ++i)
        {
            const uint32_t s0 = Rotr(w[i - 15], 7) ^ Rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
            const uint32_t s1 = Rotr(w[i - 2], 17) ^ Rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16] + s0 + w[i - 7] + s1;
        }
        uint32_t a = m_h[0], b = m_h[1], c = m_h[2], d = m_h[3], e = m_h[4], f = m_h[5], g = m_h[6], h = m_h[7];
        for (int i = 0; i < 64; ++i)
        {
            const uint32_t t1 = h + (Rotr(e, 6) ^ Rotr(e, 11) ^ Rotr(e, 25)) + ((e & f) ^ (~e & g)) + k[i] + w[i];
            const uint32_t t2 = (Rotr(a, 2) ^ Rotr(a, 13) ^ Rotr(a, 22)) + ((a & b) ^ (a & c) ^ (b & c));
            h = g;
            g = f;
            f = e;
            e = d + t1;
            d = c;
            c = b;
            b = a;
            a = t1 + t2;
        }
        m_h[0] += a;
        m_h[1] += b;
        m_h[2] += c;
        m_h[3] += d;
        m_h[4] += e;
        m_h[5] += f;
        m_h[6] += g;
        m_h[7] += h;
    }

    std::array<uint32_t, 8> m_h = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
    std::array<uint8_t, 64> m_buf{};
    size_t m_len = 0;
    uint64_t m_bits = 0;
};

// ---------------------------------------------------------------- name parsing

// Parses motion jump clip names (M_Neutral_Jump_F_Start_Run_Lfoot, M_Neutral_Jump_Loop_Fall,
// M_Neutral_Jump_B_Land_Walk_Heavy, M_Neutral_Jump_F_Off_Walk_Lfoot). Returns false if the name has no Jump token or
// no kind token.
inline bool ParseClipName(std::string_view aName, ClipMeta& aOut)
{
    bool jump = false, loop = false, fall = false;
    Kind kind = Kind::None;
    size_t p = 0;
    while (p <= aName.size())
    {
        size_t q = aName.find('_', p);
        if (q == std::string_view::npos)
            q = aName.size();
        const std::string_view t = aName.substr(p, q - p);
        if (t == "Jump")
            jump = true;
        else if (jump)
        {
            if (t == "Start")
                kind = Kind::Start;
            else if (t == "Land")
                kind = Kind::Land;
            else if (t == "Off")
                kind = Kind::Off;
            else if (t == "Loop")
                loop = true;
            else if (t == "Fall")
                fall = true;
            else if (t == "F")
                aOut.nameDir = Dir::F;
            else if (t == "B")
                aOut.nameDir = Dir::B;
            else if (t == "LL" || t == "L")
                aOut.nameDir = Dir::L;
            else if (t == "RL" || t == "R")
                aOut.nameDir = Dir::R;
            else if (t == "Stand")
                aOut.band = Band::Stand;
            else if (t == "Walk")
                aOut.band = Band::Walk;
            else if (t == "Run")
                aOut.band = Band::Run;
            else if (t == "Sprint")
                aOut.band = Band::Sprint;
            else if (t == "Light")
                aOut.weight = Weight::Light;
            else if (t == "Heavy")
                aOut.weight = Weight::Heavy;
            else if (t == "Lfoot")
                aOut.foot = Foot::L;
            else if (t == "Rfoot")
                aOut.foot = Foot::R;
        }
        p = q + 1;
    }
    if (kind == Kind::None && loop && fall)
        kind = Kind::FallLoop;
    aOut.kind = jump ? kind : Kind::None;
    return aOut.kind != Kind::None;
}

// ---------------------------------------------------------------- pose helpers

// Angle (rad) between two quaternions (x,y,z,w) in [0, pi]; NaN-safe (returns pi for non-finite input).
// Uses 2*atan2(|v|, |w|) of the relative rotation conj(a)*b, which is exact near identity (acos of the dot product
// has ~5e-4 rad float noise there) and independent of quaternion scale.
inline float QuatAngle(const float* a, const float* b)
{
    const float w = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    const float vx = a[3] * b[0] - b[3] * a[0] - (a[1] * b[2] - a[2] * b[1]);
    const float vy = a[3] * b[1] - b[3] * a[1] - (a[2] * b[0] - a[0] * b[2]);
    const float vz = a[3] * b[2] - b[3] * a[2] - (a[0] * b[1] - a[1] * b[0]);
    const float v = std::sqrt(vx * vx + vy * vy + vz * vz);
    if (!std::isfinite(w) || !std::isfinite(v) || (v == 0.0f && w == 0.0f))
        return 3.14159265f;
    return 2.0f * std::atan2(v, std::fabs(w));
}

// Sum of local rotation angles (rad) over aChain between two poses (7 floats per bone). Bones outside either pose are
// skipped. Non-finite rotations count as pi.
inline float LegPoseCost(std::span<const float> aPoseA, std::span<const float> aPoseB, std::span<const uint32_t> aChain)
{
    float cost = 0.0f;
    for (const uint32_t b : aChain)
    {
        const size_t o = size_t(b) * 7 + 3;
        if (o + 4 > aPoseA.size() || o + 4 > aPoseB.size())
            continue;
        cost += QuatAngle(aPoseA.data() + o, aPoseB.data() + o);
    }
    return cost;
}

// Leg chain bone ids (LeftUpLeg, RightUpLeg, LeftLeg, RightLeg, LeftFoot, RightFoot) resolved by name; missing bones
// are left out. Returns the number of valid ids written.
template <class Db>
inline uint32_t FindLegChain(const Db& aDb, std::array<uint32_t, 6>& aOut)
{
    static constexpr std::string_view kNames[6] = {"LeftUpLeg", "RightUpLeg", "LeftLeg", "RightLeg", "LeftFoot", "RightFoot"};
    uint32_t n = 0;
    for (const std::string_view name : kNames)
        for (uint32_t b = 0; b < aDb.NumBones(); ++b)
            if (aDb.BoneName(b) == name)
            {
                aOut[n++] = b;
                break;
            }
    return n;
}

// ---------------------------------------------------------------- table

class AirTable
{
public:
    template <class Db>
    bool Parse(std::span<const uint8_t> aBytes, const Db& aDb, std::span<const uint8_t> aDbSha256, std::string& aReason)
    {
        Reset();
        if (!BuildMeta(aDb, aReason))
            return Fail();
        size_t p = 0;
        auto need = [&](size_t n) { return p + n <= aBytes.size(); };
        auto u32 = [&]() {
            uint32_t v;
            std::memcpy(&v, aBytes.data() + p, 4);
            p += 4;
            return v;
        };
        if (!need(8) || std::memcmp(aBytes.data(), kAirzMagic, 8) != 0)
            return Fail(aReason, "bad magic");
        p = 8;
        if (!need(4 + 32 + 4))
            return Fail(aReason, "truncated header");
        if (u32() != kAirzVersion)
            return Fail(aReason, "unsupported version");
        if (aDbSha256.size() != 32 || std::memcmp(aBytes.data() + p, aDbSha256.data(), 32) != 0)
            return Fail(aReason, "db sha256 mismatch");
        p += 32;
        const uint32_t count = u32();
        if (count > aDb.NumClips())
            return Fail(aReason, "clip count exceeds db");
        int64_t prev = -1;
        for (uint32_t i = 0; i < count; ++i)
        {
            if (!need(6))
                return Fail(aReason, "truncated clip header");
            const uint32_t ci = u32();
            uint16_t nameLen;
            std::memcpy(&nameLen, aBytes.data() + p, 2);
            p += 2;
            if (ci >= aDb.NumClips())
                return Fail(aReason, "clip index out of range");
            if (int64_t(ci) <= prev)
                return Fail(aReason, "clip order");
            prev = ci;
            if (!need(size_t(nameLen) + 4))
                return Fail(aReason, "truncated name");
            const std::string_view name(reinterpret_cast<const char*>(aBytes.data() + p), nameLen);
            p += nameLen;
            if (name != aDb.ClipName(ci))
                return Fail(aReason, "clip name mismatch");
            const uint32_t nf = u32();
            const auto& rec = aDb.Clips()[ci];
            if (nf != rec.numFrames)
                return Fail(aReason, "frame count mismatch");
            if (!need(size_t(nf) * 4))
                return Fail(aReason, "truncated root_z");
            float* dst = m_rootZ.data() + rec.firstFrame;
            std::memcpy(dst, aBytes.data() + p, size_t(nf) * 4);
            p += size_t(nf) * 4;
            for (uint32_t f = 0; f < nf; ++f)
                if (!std::isfinite(dst[f]))
                    return Fail(aReason, "non-finite root_z");
            ClassifyClip(aDb, ci);
        }
        if (p != aBytes.size())
            return Fail(aReason, "trailing bytes");
        FinishMeta(aDb);
        m_loaded = true;
        aReason.clear();
        return true;
    }

    // Stub for the future .utppmm v2 RTZ_ section (root_z inside the DB). Builds the metadata only.
    template <class Db>
    bool FromDatabase(const Db& aDb, std::string& aReason)
    {
        Reset();
        if (!BuildMeta(aDb, aReason))
            return Fail();
        FinishMeta(aDb);
        aReason = "RTZ_ section not supported yet (metadata only)";
        return false;
    }

    void Reset() { *this = AirTable{}; }

    bool Loaded() const { return m_loaded; }
    std::span<const float> RootZ() const { return m_rootZ; }
    float RootZAt(uint32_t aFrame) const { return aFrame < m_rootZ.size() ? m_rootZ[aFrame] : 0.0f; }
    std::span<const ClipMeta> Meta() const { return m_meta; }
    const ClipMeta* MetaOfClip(int32_t aClip) const
    {
        for (const ClipMeta& m : m_meta)
            if (m.clip == aClip)
                return &m;
        return nullptr;
    }
    uint32_t CountStatus(ClipStatus aS) const
    {
        uint32_t n = 0;
        for (const ClipMeta& m : m_meta)
            n += m.status == aS ? 1u : 0u;
        return n;
    }
    uint32_t NumAirzClips() const { return m_airzClips; }
    uint32_t NumAirzRejected() const { return m_airzRejected; }
    uint32_t NumAirzClean() const { return m_airzClean; }
    int32_t FirstRejectedClip() const { return m_firstRejected; } // -1 if none
    int32_t HipsBone() const { return m_hips; }                   // -1 before a successful BuildMeta
    int32_t FallLoopClip() const { return m_fallLoop; }
    bool LoopOk() const { return m_loopOk; }
    float LoopMaxDeg() const { return m_loopDeg; }
    float LoopMaxM() const { return m_loopM; }

    // Nearest clip of aKind. Fallback order (lexicographic cost, ties -> lowest clip index):
    //   1. direction mismatch (F preferred: cost 100000, other dirs 200000; sets kDirRelaxed),
    //   2. band distance x 1000 (Stand < Walk < Run < Sprint; sets kBandRelaxed),
    //   3. weight mismatch 100 (kWeightRelaxed),
    //   4. foot mismatch 10 (kFootRelaxed).
    // Wildcards never set relaxed flags but carry a small deterministic preference (total < 10, below every real
    // mismatch) so None does not fall to the lowest clip index:
    //   - request Dir::None   -> F preferred (non-F +100000, same weight as a relaxed F fallback, no flag),
    //   - request Band::None  -> Run preferred (+2 per band step from Run), Weight::None -> Light (Heavy +2),
    //     Foot::None -> Lfoot (Rfoot +1),
    //   - clip without band / weight / foot under a specific request -> +1 each (a specific match wins a tie).
    // FallLoop ignores every parameter.
    PickResult Pick(Kind aKind, Dir aDir, Band aBand, Weight aWeight, Foot aFoot) const
    {
        PickResult best;
        int bestCost = 1 << 30;
        for (size_t i = 0; i < m_meta.size(); ++i)
        {
            const ClipMeta& m = m_meta[i];
            if (m.kind != aKind)
                continue;
            int cost = 0;
            uint32_t rel = PickResult::kExact;
            if (aKind != Kind::FallLoop)
            {
                if (aDir == Dir::None)
                    cost += m.nameDir == Dir::F ? 0 : 100000;
                else if (m.nameDir != aDir)
                {
                    cost += m.nameDir == Dir::F ? 100000 : 200000;
                    rel |= PickResult::kDirRelaxed;
                }
                if (aBand == Band::None)
                    cost += m.band == Band::None ? 0 : 2 * std::abs(int(m.band) - int(Band::Run));
                else if (m.band == Band::None)
                    cost += 1;
                else if (m.band != aBand)
                {
                    cost += 1000 * std::abs(int(m.band) - int(aBand));
                    rel |= PickResult::kBandRelaxed;
                }
                if (aWeight == Weight::None)
                    cost += m.weight == Weight::Heavy ? 2 : 0;
                else if (m.weight == Weight::None)
                    cost += 1;
                else if (m.weight != aWeight)
                {
                    cost += 100;
                    rel |= PickResult::kWeightRelaxed;
                }
                if (aFoot == Foot::None)
                    cost += m.foot == Foot::R ? 1 : 0;
                else if (m.foot == Foot::None)
                    cost += 1;
                else if (m.foot != aFoot)
                {
                    cost += 10;
                    rel |= PickResult::kFootRelaxed;
                }
            }
            if (cost < bestCost)
            {
                bestCost = cost;
                best.clip = m.clip;
                best.meta = int32_t(i);
                best.relaxed = rel;
            }
        }
        return best;
    }

private:
    bool Fail() { m_meta.clear(); m_rootZ.clear(); return false; }
    bool Fail(std::string& aReason, const char* aMsg)
    {
        aReason = aMsg;
        Reset();
        return false;
    }

    template <class Db>
    bool BuildMeta(const Db& aDb, std::string& aReason)
    {
        m_hips = -1;
        for (uint32_t b = 0; b < aDb.NumBones(); ++b)
            if (aDb.BoneName(b) == "Hips")
                m_hips = int32_t(b);
        if (m_hips < 0)
        {
            aReason = "db has no Hips bone";
            return false;
        }
        m_rootZ.assign(aDb.NumFrames(), 0.0f);
        for (uint32_t c = 0; c < aDb.NumClips(); ++c)
        {
            ClipMeta m;
            // motion clips only: clips of non-package sources are never air picks
            if (!ParseClipName(aDb.ClipName(c), m))
                continue;
            m.clip = int32_t(c);
            m.firstFrame = aDb.Clips()[c].firstFrame;
            m.numFrames = aDb.Clips()[c].numFrames;
            m_meta.push_back(m);
        }
        return true;
    }

    template <class Db>
    float HipsZ(const Db& aDb, uint32_t aFrame) const
    {
        const auto pose = aDb.Pose(aFrame);
        const size_t o = size_t(m_hips) * 7 + 2;
        return o < pose.size() ? pose[o] : 0.0f;
    }

    // Applied / RtzClean / RejectedRange decision for one .airz clip (root_z already copied into m_rootZ).
    template <class Db>
    void ClassifyClip(const Db& aDb, uint32_t aClip)
    {
        ++m_airzClips;
        const auto& rec = aDb.Clips()[aClip];
        float* rz = m_rootZ.data() + rec.firstFrame;
        bool rawIn = true, subIn = true;
        double sr = 0, sr2 = 0, ss = 0, ss2 = 0;
        for (uint32_t f = 0; f < rec.numFrames; ++f)
        {
            const float raw = HipsZ(aDb, rec.firstFrame + f);
            const float sub = raw - rz[f];
            rawIn = rawIn && std::isfinite(raw) && raw >= kHipsMin && raw <= kHipsMax;
            subIn = subIn && std::isfinite(sub) && sub >= kHipsMin && sub <= kHipsMax;
            sr += raw;
            sr2 += double(raw) * raw;
            ss += sub;
            ss2 += double(sub) * sub;
        }
        const double n = rec.numFrames > 0 ? double(rec.numFrames) : 1.0;
        const double varRaw = sr2 / n - (sr / n) * (sr / n);
        const double varSub = ss2 / n - (ss / n) * (ss / n);
        ClipStatus st;
        if (rawIn && varSub > varRaw + 1e-6)
            st = ClipStatus::RtzClean;
        else if (!subIn)
            st = ClipStatus::RejectedRange;
        else
            st = ClipStatus::Applied;
        if (st == ClipStatus::RtzClean)
            ++m_airzClean;
        if (st == ClipStatus::RejectedRange && m_airzRejected++ == 0)
            m_firstRejected = int32_t(aClip);
        if (st != ClipStatus::Applied)
            std::fill(rz, rz + rec.numFrames, 0.0f);
        for (ClipMeta& m : m_meta)
            if (m.clip == int32_t(aClip))
                m.status = st;
    }

    template <class Db>
    void FinishMeta(const Db& aDb)
    {
        for (ClipMeta& m : m_meta)
            Measure(aDb, m);
        m_fallLoop = -1;
        m_loopOk = false;
        for (const ClipMeta& m : m_meta)
            if (m.kind == Kind::FallLoop && m.numFrames >= 2)
            {
                m_fallLoop = m.clip;
                const auto a = aDb.Pose(m.firstFrame);
                const auto b = aDb.Pose(m.firstFrame + m.numFrames - 1);
                float maxDeg = 0.0f, maxM = 0.0f;
                for (uint32_t bone = 0; bone < aDb.NumBones() && size_t(bone) * 7 + 7 <= a.size() && size_t(bone) * 7 + 7 <= b.size(); ++bone)
                {
                    const size_t o = size_t(bone) * 7;
                    maxDeg = std::max(maxDeg, QuatAngle(a.data() + o + 3, b.data() + o + 3) * 57.2957795f);
                    if (bone == 0)
                        continue;
                    float dz = a[o + 2] - b[o + 2];
                    if (int32_t(bone) == m_hips)
                        dz = (a[o + 2] - RootZAt(m.firstFrame)) - (b[o + 2] - RootZAt(m.firstFrame + m.numFrames - 1));
                    const float dx = a[o] - b[o], dy = a[o + 1] - b[o + 1];
                    const float d = std::sqrt(dx * dx + dy * dy + dz * dz);
                    maxM = std::max(maxM, std::isfinite(d) ? d : 1e9f);
                }
                m_loopDeg = maxDeg;
                m_loopM = maxM;
                m_loopOk = maxDeg <= kLoopMaxDeg && maxM <= kLoopMaxM;
                break;
            }
    }

    template <class Db>
    void Measure(const Db& aDb, ClipMeta& m) const
    {
        const bool useRz = m.status == ClipStatus::Applied;
        float tx = 0.0f, ty = 0.0f;
        m.minHips = 1e9f;
        m.maxHips = -1e9f;
        m.maxRootZ = 0.0f;
        for (uint32_t f = 0; f < m.numFrames; ++f)
        {
            const uint32_t g = m.firstFrame + f;
            if (const auto* r = aDb.Root(g); r && f + 1 < m.numFrames && std::isfinite(r->dx) && std::isfinite(r->dy))
            {
                tx += r->dx;
                ty += r->dy;
            }
            const float h = HipsZ(aDb, g) - (useRz ? m_rootZ[g] : 0.0f);
            if (std::isfinite(h))
            {
                m.minHips = std::min(m.minHips, h);
                m.maxHips = std::max(m.maxHips, h);
            }
        }
        m.travelX = tx;
        m.travelY = ty;
        if (std::fabs(tx) + std::fabs(ty) < 0.05f)
            m.travelDir = Dir::None;
        else if (std::fabs(ty) >= std::fabs(tx))
            m.travelDir = ty > 0.0f ? Dir::F : Dir::B;
        else
            m.travelDir = tx > 0.0f ? Dir::R : Dir::L;

        // Height signal: root_z when applied, else raw Hips z relative to frame 0 (Start / Off / FallLoop) or the last
        // frame (Land: the clip ends standing at take-off level).
        const uint32_t ref = m.kind == Kind::Land && m.numFrames > 0 ? m.firstFrame + m.numFrames - 1 : m.firstFrame;
        const float base = useRz ? 0.0f : HipsZ(aDb, ref);
        auto height = [&](uint32_t f) {
            const uint32_t g = m.firstFrame + f;
            const float v = useRz ? m_rootZ[g] : HipsZ(aDb, g) - base;
            return std::isfinite(v) ? v : 0.0f;
        };
        auto anyContact = [&](uint32_t f) {
            const auto c = aDb.Contacts(m.firstFrame + f);
            return c[0] >= kContactOn || c[1] >= kContactOn;
        };
        float best = -1e9f;
        for (uint32_t f = 0; f < m.numFrames; ++f)
            if (const float h = height(f); h > best)
            {
                best = h;
                m.apex = int32_t(f);
            }
        m.maxRootZ = best;
        if (best < kAirborneEps)
            m.apex = -1;
        if (m.apex >= 0)
            for (uint32_t f = uint32_t(m.apex) + 1; f < m.numFrames; ++f)
                if (height(f) <= 0.0f)
                {
                    m.zeroCross = int32_t(f);
                    break;
                }
        bool seenAir = false;
        for (uint32_t f = 0; f < m.numFrames; ++f)
        {
            const bool c = anyContact(f);
            if (m.liftoff < 0 && f > 0 && !c && anyContact(f - 1))
                m.liftoff = int32_t(f);
            if (!c)
                seenAir = true;
            else if (seenAir && m.touch < 0)
                m.touch = int32_t(f);
        }
        if (m.kind == Kind::Off)
        {
            const int32_t from = std::max(m.liftoff, 0);
            for (uint32_t f = uint32_t(from); f < m.numFrames; ++f)
                if (height(f) < -kDropEps)
                {
                    m.drop = int32_t(f);
                    break;
                }
        }
    }

    std::vector<float> m_rootZ;
    std::vector<ClipMeta> m_meta;
    int32_t m_hips = -1;
    int32_t m_fallLoop = -1;
    int32_t m_firstRejected = -1;
    uint32_t m_airzClips = 0;
    uint32_t m_airzRejected = 0;
    uint32_t m_airzClean = 0;
    float m_loopDeg = 0.0f;
    float m_loopM = 0.0f;
    bool m_loopOk = false;
    bool m_loaded = false;
};
} // namespace utpp::mm::airtab
