// UTPP motion matching runtime - motion lean / head-turn pose pack "utpp.lean/1" reader + evaluator.
// Header-only. Fail-closed: magic / version / header size / file size / header + payload CRC-32 / rig hash / section
// table (tags, bounds, record sizes) / cross-section ranges / triangulation topology / non-finite floats reject the
// whole file. Loading allocates (once); the per-frame API (Evaluate, LayerState::Update, BlendAdditive, curve sampling,
// the gate helpers) does not allocate.
//
// Data: Data\utpp_lean_<woman|man>.bin written by Source\tools\mmbake\leanbake.py (the binary layout is documented in
// the file header of that script and repeated here in short; Data\utpp_lean_<rig>.json is the human readable header).
//
//   Header 64 B: "UTPPLEAN", u32 version 1, u32 header_size 64, u64 rig_hash, u32 num_bones 73, u32 num_sections,
//                u32 file_size, u32 payload_crc32 ([64,size)), u32 header_crc32 (header with this field 0), 20 B zero.
//   Section table (16 B each: tag[4], u32 offset, u32 size, u32 count), sections 16-byte aligned:
//     BONE 36 B name[32] + i32 parent | PARM 60 x f32 | LAYR 240 B | SAMP 80 B | TRIS 72 B (18 x i32)
//     DELT S x (73 bones x 10 f32: dq xyzw, dt xyz, ds xyz) | CLIP 80 B | CVAL f32 curve values
//   A delta is the AAT_ROTATION_OFFSET_MESH_SPACE additive of a sample against the layer's reference pose (same
//   representation as overlaybake.py / Additive.hpp: dq = G_X * inv(G_A), dt = t_X - t_A, ds = s_X / s_A - 1).
//
// Evaluation of a layer = UE 5.8 UBlendSpace / UBlendSpace1D sample weights:
//   * input clamped to the axis range (GetClampedAndWrappedBlendInput, no wrap) and normalised to 0..1
//     (GetNormalizedBlendInput); 1D: BlendSpace.cpp FBlendSpaceData::GetSamples1D (segment walk, end clamp);
//   * 2D: FBlendSpaceData::GetSamples2D on the UE triangulation that leanbake.py computed with a 1:1 port of
//     FDelaunayTriangleGenerator (the stored neighbour / perimeter links are UE's FBlendSpaceTriangle edge info):
//     incremental search through the edge normals from the cached triangle, perimeter walk with projection when the
//     point is outside the (convex, possibly non-square) sample hull, FMath::GetBaryCentric2D inside;
//   * weights <= ZERO_ANIMWEIGHT_THRESH (1e-5) dropped, the rest normalised to sum 1 (GetBlendSamples);
//   * LayerState::Update smooths the per-sample weights like UBlendSpace::InterpolateWeightOfSampleData (critically
//     damped, smoothing time 1/(e*speed), bTargetWeightInterpolationEaseInOut default true), reusing BlendSpace.hpp.
// The 2D support of BlendSpace.hpp is limited to full rectilinear grids; the lean sets (5-sample cross, 13-sample head
// grid with holes) are not, hence the stored UE triangulation. Smoothing.hpp supplies the smoothing primitives.
//
// ABP logic (sourcepack-lean-data-2026-10.json abp_logic; SandboxCharacter_Mover_ABP AdditiveLeans) for the runtime:
//   lat = clamp( a_lat / MapRangeClamped(speed 200..320 -> 500..800), -1, 1 )  (cm/s, cm/s^2), a_lat = acceleration
//         component to the RIGHT of the velocity heading (LateralAccelAmount below, metres in / out);
//   (x, y) = lat * dirScale[MovementDirection]  (F (1,0) B (-1,0) LL/LR (0,1) RL/RR (0,-1); enum F0 B1 LR2 LL3 RL4 RR5)
//   body additive = walk layer (x,y) and run layer (x,y), crossfade alpha = MapRangeClamped(speed 200..320 -> 0..1)
//        interpolated at 10/s (WalkRunAlpha); head additive on top, dropped while EnableAO (0.3 s blend);
//   active only while MovementState == Moving and MovementMode != Sliding (0.3 s blends); final lean weight is
//   1 - Disable_AdditiveLeans (curve of the 16 Relaxed Turn_090 / Spin_F_360 clips, interp 5/s; CLIP records);
//   CMC variant (Neutral BS1D layer "cmc_run"): x = rel_accel_y * MapRangeClamped(speed 165..375 -> 0.5..1), alpha
//   blend 0.2 s by IsMoving, weight interp 1.5/s (CmcLeanX).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>

#include "Additive.hpp"
#include "Smoothing.hpp"

namespace utpp::mm::lean
{
using ov::Additive;
using ov::Quat;
using ov::Skeleton;
using ov::Vec3;
using ov::Xf;

inline constexpr int kNumBones = 73;
inline constexpr int kMaxSamples = 16;
inline constexpr int kMaxLayers = 8;
inline constexpr int kParmCount = 60;
inline constexpr int kDeltaFloats = 10; // per bone: dq(4) dt(3) ds(3)
inline constexpr float kZeroWeight = 1e-5f;   // ZERO_ANIMWEIGHT_THRESH
inline constexpr float kKindaSmall = 1e-4f;   // UE_KINDA_SMALL_NUMBER
inline constexpr long kMaxFileBytes = 8L * 1024L * 1024L; // real packs are ~100 KB: anything bigger is not a lean pack
inline constexpr uint64_t kRigIdentityWoman = 0xF62810A88A8BFDA9ull;
inline constexpr uint64_t kRigIdentityMan = 0xAFE2CFEDCF97CDF7ull;

enum class Role : uint32_t { Body = 0, Head = 1, Cmc = 2 };
enum class Style : uint32_t { Relaxed = 0, Neutral = 1 };
// E_MovementDirection values (Mover ABP).
enum Direction : int { kDirF = 0, kDirB = 1, kDirLR = 2, kDirLL = 3, kDirRL = 4, kDirRR = 5, kNumDirections = 6 };

inline uint32_t Crc32(const uint8_t* aData, size_t aSize)
{
    uint32_t c = ~0u;
    for (size_t i = 0; i < aSize; ++i)
    {
        c ^= aData[i];
        for (int k = 0; k < 8; ++k)
            c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
    }
    return ~c;
}

struct Params // UE units (cm, cm/s, cm/s^2, seconds)
{
    float latSpeedLo = 200, latSpeedHi = 320, latAccelLo = 500, latAccelHi = 800, latClampLo = -1, latClampHi = 1;
    float xfadeSpeedLo = 200, xfadeSpeedHi = 320, xfadeInterpPerS = 10;
    float boolBlendS = 0.3f, stateBlendS = 0.3f, disableCurveInterpPerS = 5;
    float cmcSpeedLo = 165, cmcSpeedHi = 375, cmcScaleLo = 0.5f, cmcScaleHi = 1.0f, cmcAlphaBlendS = 0.2f;
    float cmcWeightInterpPerS = 1.5f, headWeightInterpPerS = 1.75f, bodyWeightInterpPerS = 1.2f;
    float dirScale[kNumDirections][2]{};
};

struct Layer
{
    char id[17]{};
    char blendspace[49]{};
    char baseClip[65]{};
    char axis[2][17]{};
    int dims = 0;
    Style style = Style::Relaxed;
    int baseFrame = 0;
    Role role = Role::Body;
    int firstSample = 0, numSamples = 0, firstTri = 0, numTris = 0;
    float min[2]{}, max[2]{};
    float interpPerS = 0;
};

struct SampleRec
{
    float x = 0, y = 0;
    int frame = 0;
    char clip[65]{};
};

struct ClipRec
{
    char name[65]{};
    int first = 0, count = 0;
    int kind = 0; // 0 = Relaxed source clip carrying the curve, 1 = proposed Neutral equivalent
    float fps = 30;
};

struct Weights // sample weights of one evaluation (layer-local sample indices), sum 1 after normalisation
{
    int n = 0;
    int sample[3]{-1, -1, -1};
    float w[3]{};
};

class LeanPack
{
public:
    // UE FBlendSpaceTriangle (normalised vertices, outward edge normals derived at load).
    struct Tri
    {
        int s[3];
        float vx[3], vy[3], nx[3], ny[3];
        int nbr[3];
        int pt[3][2], pv[3][2];
    };

    bool LoadFromFile(const char* aPath, uint64_t aExpectedRigHash, std::string& aErr)
    {
        std::vector<uint8_t> bytes;
        FILE* f = nullptr;
#ifdef _MSC_VER
        if (fopen_s(&f, aPath, "rb") != 0)
            f = nullptr;
#else
        f = std::fopen(aPath, "rb");
#endif
        if (!f)
            return Fail(aErr, "cannot open file");
        std::fseek(f, 0, SEEK_END);
        const long sz = std::ftell(f);
        std::fseek(f, 0, SEEK_SET);
        if (sz > kMaxFileBytes)
        {
            std::fclose(f);
            return Fail(aErr, "file too large");
        }
        if (sz > 0)
        {
            bytes.resize(static_cast<size_t>(sz));
            if (std::fread(bytes.data(), 1, bytes.size(), f) != bytes.size())
                bytes.clear();
        }
        std::fclose(f);
        return LoadFromMemory(std::move(bytes), aExpectedRigHash, aErr);
    }

    // aExpectedRigHash == 0 skips the rig identity check (tests); the game passes kRigIdentityWoman / Man.
    bool LoadFromMemory(std::vector<uint8_t> aBytes, uint64_t aExpectedRigHash, std::string& aErr)
    {
        m_ok = false;
        if (aBytes.size() > static_cast<size_t>(kMaxFileBytes))
            return Fail(aErr, "file too large");
        m_b = std::move(aBytes);
        m_layers.clear();
        m_samples.clear();
        m_tris.clear();
        m_clips.clear();
        m_cval.clear();
        if (!Parse(aExpectedRigHash, aErr))
        {
            m_b.clear();
            m_layers.clear();
            m_samples.clear();
            m_tris.clear();
            m_clips.clear();
            m_cval.clear();
            return false;
        }
        m_ok = true;
        return true;
    }

    bool Ok() const { return m_ok; }
    uint64_t RigHash() const { return m_rigHash; }
    uint32_t PayloadCrc() const { return m_payloadCrc; }
    const Params& P() const { return m_p; }
    const Skeleton& Skel() const { return m_sk; }
    // Bone i of the BONE table (73 bones, parent order). The runtime checks names + parents against the overlay skeleton.
    std::string_view BoneName(int i) const { return std::string_view(m_boneName[i]); }
    int NumLayers() const { return static_cast<int>(m_layers.size()); }
    const Layer& LayerAt(int i) const { return m_layers[static_cast<size_t>(i)]; }
    int FindLayer(std::string_view aId) const
    {
        for (int i = 0; i < NumLayers(); ++i)
            if (aId == m_layers[static_cast<size_t>(i)].id)
                return i;
        return -1;
    }
    int NumSamplesTotal() const { return static_cast<int>(m_samples.size()); }
    const SampleRec& Sample(int aLayer, int aLocal) const
    {
        return m_samples[static_cast<size_t>(m_layers[static_cast<size_t>(aLayer)].firstSample + aLocal)];
    }
    int NumTris(int aLayer) const { return m_layers[static_cast<size_t>(aLayer)].numTris; }
    const Tri& TriAt(int aLayer, int aTri) const
    {
        return m_tris[static_cast<size_t>(m_layers[static_cast<size_t>(aLayer)].firstTri + aTri)];
    }

    // Raw stored delta of a sample: kNumBones x kDeltaFloats floats (dq xyzw, dt xyz, ds xyz).
    const float* SampleDelta(int aLayer, int aLocal) const
    {
        const size_t g = static_cast<size_t>(m_layers[static_cast<size_t>(aLayer)].firstSample + aLocal);
        return reinterpret_cast<const float*>(m_b.data() + m_secDelt + g * kNumBones * kDeltaFloats * 4);
    }
    // Copies the sample delta into an Additive (mesh-space rotation offset).
    void SampleAdditive(int aLayer, int aLocal, Additive& aOut) const
    {
        const float* p = SampleDelta(aLayer, aLocal);
        aOut.n = kNumBones;
        aOut.meshSpace = true;
        for (int i = 0; i < kNumBones; ++i, p += kDeltaFloats)
            aOut.b[i] = Xf{Quat{p[0], p[1], p[2], p[3]}, Vec3{p[4], p[5], p[6]}, Vec3{p[7], p[8], p[9]}};
    }
    // Weighted blend of the layer's sample deltas (same math as ov::BlendAdditives: shortest-path weighted sum of the
    // rotations, normalised; weights <= kZeroWeight skipped). aW has numSamples entries. No allocation.
    void BlendAdditive(int aLayer, const float* aW, Additive& aOut) const
    {
        const Layer& L = m_layers[static_cast<size_t>(aLayer)];
        aOut.n = kNumBones;
        aOut.meshSpace = true;
        const float* base[kMaxSamples];
        for (int k = 0; k < L.numSamples; ++k)
            base[k] = SampleDelta(aLayer, k);
        for (int i = 0; i < kNumBones; ++i)
        {
            Quat acc{0, 0, 0, 0}, ref{};
            Vec3 t{}, s{};
            bool first = true;
            for (int k = 0; k < L.numSamples; ++k)
            {
                const float w = aW[k];
                if (!(w > kZeroWeight))
                    continue;
                const float* d = base[k] + i * kDeltaFloats;
                const Quat q{d[0], d[1], d[2], d[3]};
                if (first)
                {
                    ref = q;
                    first = false;
                }
                const float sg = math::QuatDot(ref, q) >= 0.0f ? w : -w;
                acc = Quat{acc.x + q.x * sg, acc.y + q.y * sg, acc.z + q.z * sg, acc.w + q.w * sg};
                t = t + Vec3{d[4], d[5], d[6]} * w;
                s = s + Vec3{d[7], d[8], d[9]} * w;
            }
            aOut.b[i] = Xf{first ? Quat{} : math::QuatNormalize(acc), t, s};
        }
    }

    // ---- Disable_AdditiveLeans curves --------------------------------------------------------------------------
    int NumClips() const { return static_cast<int>(m_clips.size()); }
    const ClipRec& ClipAt(int i) const { return m_clips[static_cast<size_t>(i)]; }
    int FindClip(std::string_view aName) const
    {
        for (int i = 0; i < NumClips(); ++i)
            if (aName == m_clips[static_cast<size_t>(i)].name)
                return i;
        return -1;
    }
    // Curve value 0..1 (1 = lean muted) at time aSeconds into the clip: linear between 30 fps frames, clamped to the
    // first / last value outside the clip.
    float DisableCurveAt(int aClip, float aSeconds) const
    {
        const ClipRec& c = m_clips[static_cast<size_t>(aClip)];
        const float f = std::clamp(aSeconds * c.fps, 0.0f, static_cast<float>(c.count - 1));
        const int i0 = std::min(c.count - 1, static_cast<int>(f));
        const int i1 = std::min(c.count - 1, i0 + 1);
        const float t = f - static_cast<float>(i0);
        const float a = m_cval[static_cast<size_t>(c.first + i0)], b = m_cval[static_cast<size_t>(c.first + i1)];
        return a + (b - a) * t;
    }

    // ---- blend space evaluation ----------------------------------------------------------------------------------
    // aCache = cached triangle / segment index (start -1); updated like UE's InOutTriangleIndex. Returns false when
    // the layer is invalid or the search fails (aOut empty).
    bool Evaluate(int aLayer, float aX, float aY, int& aCache, Weights& aOut) const
    {
        aOut = Weights{};
        if (!m_ok || aLayer < 0 || aLayer >= NumLayers())
            return false;
        const Layer& L = m_layers[static_cast<size_t>(aLayer)];
        const float px = Norm(aX, L.min[0], L.max[0]);
        Raw r;
        const bool ok = L.dims == 1 ? Eval1D(L, px, aCache, r) : Eval2D(L, px, Norm(aY, L.min[1], L.max[1]), aCache, r);
        if (!ok)
            return false;
        // GetBlendSamples: drop <= ZERO_ANIMWEIGHT_THRESH, normalise.
        float total = 0.0f;
        for (int i = 0; i < r.n; ++i)
            if (r.w[i] > kZeroWeight)
            {
                aOut.sample[aOut.n] = r.s[i];
                aOut.w[aOut.n++] = r.w[i];
                total += r.w[i];
            }
        if (aOut.n == 0 || !(total > 0.0f))
        {
            aOut = Weights{};
            return false;
        }
        for (int i = 0; i < aOut.n; ++i)
            aOut.w[i] /= total;
        return true;
    }

private:
    struct Raw
    {
        int n = 0;
        int s[3]{};
        float w[3]{};
        void Push(int aS, float aW)
        {
            s[n] = aS;
            w[n++] = aW;
        }
    };

    static float Norm(float v, float lo, float hi)
    {
        if (!std::isfinite(v))
            v = lo;
        return (std::clamp(v, lo, hi) - lo) / (hi - lo);
    }

    bool Eval1D(const Layer& L, float p, int& aCache, Raw& r) const
    {
        const Tri* seg = &m_tris[static_cast<size_t>(L.firstTri)];
        const int n = L.numTris;
        if (n <= 0)
            return false;
        auto vx = [&](int sample) { return Norm(m_samples[static_cast<size_t>(L.firstSample + sample)].x, L.min[0], L.max[0]); };
        if (aCache < 0 || aCache >= n)
            aCache = n / 2;
        for (int attempt = 0; attempt != n; ++attempt)
        {
            const Tri& s = seg[aCache];
            const float p0 = vx(s.s[0]), p1 = vx(s.s[1]);
            if (p < p0)
            {
                if (aCache > 0)
                    --aCache;
                else
                {
                    r.Push(s.s[0], 1.0f);
                    return true;
                }
            }
            else if (p > p1)
            {
                if (aCache < n - 1)
                    ++aCache;
                else
                {
                    r.Push(s.s[1], 1.0f);
                    return true;
                }
            }
            else
            {
                const float frac = std::clamp((p1 > p0) ? (p - p0) / (p1 - p0) : 0.0f, 0.0f, 1.0f);
                r.Push(s.s[0], 1.0f - frac);
                r.Push(s.s[1], frac);
                return true;
            }
        }
        return false;
    }

    bool Eval2D(const Layer& L, float px, float py, int& aCache, Raw& r) const
    {
        const Tri* T = &m_tris[static_cast<size_t>(L.firstTri)];
        const int nt = L.numTris;
        if (nt <= 0)
            return false;
        if (aCache < 0 || aCache >= nt)
            aCache = nt / 2;
        for (int attempt = 0; attempt != nt; ++attempt)
        {
            const Tri* tr = &T[aCache];
            float largest = kKindaSmall;
            int le = -1;
            for (int v = 0; v < 3; ++v)
            {
                const float d = (px - tr->vx[v]) * tr->nx[v] + (py - tr->vy[v]) * tr->ny[v];
                if (d > largest)
                {
                    largest = d;
                    le = v;
                }
            }
            if (le < 0)
            {
                // FMath::GetBaryCentric2D
                const float ax = tr->vx[0], ay = tr->vy[0], bx = tr->vx[1], by = tr->vy[1], cx = tr->vx[2], cy = tr->vy[2];
                const float den = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy);
                const float a = ((by - cy) * (px - cx) + (cx - bx) * (py - cy)) / den;
                const float b = ((cy - ay) * (px - cx) + (ax - cx) * (py - cy)) / den;
                r.Push(tr->s[0], a);
                r.Push(tr->s[1], b);
                r.Push(tr->s[2], 1.0f - a - b);
                return true;
            }
            if (tr->nbr[le] < 0)
            {
                // Outside the perimeter: walk around it (BlendSpace.cpp GetSamples2D).
                int origDir = -1, start = le, end = (le + 1) % 3, prev = aCache;
                for (int pa = 0; pa != nt; ++pa)
                {
                    const float sx = tr->vx[start], sy = tr->vy[start], ex = tr->vx[end], ey = tr->vy[end];
                    const float dx = ex - sx, dy = ey - sy;
                    const float t = (dx * (px - sx) + dy * (py - sy)) / (dx * dx + dy * dy);
                    if (t >= 0.0f && t <= 1.0f)
                    {
                        r.Push(tr->s[start], 1.0f - t);
                        r.Push(tr->s[end], t);
                        return true;
                    }
                    if (origDir == -1)
                        origDir = t > 1.0f ? 1 : 0;
                    const int dir = t > 1.0f ? 1 : 0;
                    if (origDir != dir)
                    {
                        if (dir == 0)
                            r.Push(tr->s[start], 1.0f - t);
                        else
                            r.Push(tr->s[end], t);
                        aCache = prev;
                        return true;
                    }
                    prev = aCache;
                    const int nextTri = tr->pt[start][dir];
                    const int nextVert = tr->pv[start][dir];
                    aCache = nextTri;
                    if (nextTri < 0)
                    {
                        if (dir == 0)
                            r.Push(tr->s[start], 1.0f - t);
                        else
                            r.Push(tr->s[end], t);
                        aCache = prev;
                        return true;
                    }
                    tr = &T[nextTri];
                    start = nextVert;
                    end = (start + 1) % 3;
                }
            }
            else
                aCache = tr->nbr[le];
        }
        return false;
    }

    static bool Fail(std::string& aErr, const char* aWhy)
    {
        aErr = aWhy;
        return false;
    }
    static bool Fail(std::string& aErr, const char* aWhy, const std::string& aDetail)
    {
        aErr = std::string(aWhy) + ": " + aDetail;
        return false;
    }
    template <typename T> T Rd(size_t aOff) const
    {
        T v;
        std::memcpy(&v, m_b.data() + aOff, sizeof(T));
        return v;
    }
    static bool StrOk(const uint8_t* p, size_t n, char* out, size_t outCap) // NUL-terminated within n, ASCII printable
    {
        size_t len = 0;
        while (len < n && p[len] != 0)
            ++len;
        if (len >= n || len >= outCap)
            return false;
        for (size_t i = 0; i < len; ++i)
            if (p[i] < 0x20 || p[i] > 0x7E)
                return false;
        for (size_t i = len; i < n; ++i)
            if (p[i] != 0)
                return false;
        std::memcpy(out, p, len);
        out[len] = 0;
        return true;
    }

    bool Parse(uint64_t aExpectedRigHash, std::string& aErr)
    {
        const size_t size = m_b.size();
        if (size < 64 + 16)
            return Fail(aErr, "file too small");
        if (std::memcmp(m_b.data(), "UTPPLEAN", 8) != 0)
            return Fail(aErr, "bad magic");
        if (Rd<uint32_t>(8) != 1)
            return Fail(aErr, "unsupported version");
        if (Rd<uint32_t>(12) != 64)
            return Fail(aErr, "bad header size");
        m_rigHash = Rd<uint64_t>(16);
        if (aExpectedRigHash != 0 && m_rigHash != aExpectedRigHash)
            return Fail(aErr, "rig identity mismatch");
        if (Rd<uint32_t>(24) != static_cast<uint32_t>(kNumBones))
            return Fail(aErr, "bone count mismatch");
        const uint32_t nsec = Rd<uint32_t>(28);
        if (Rd<uint32_t>(32) != size)
            return Fail(aErr, "file size mismatch");
        m_payloadCrc = Rd<uint32_t>(36);
        if (Crc32(m_b.data() + 64, size - 64) != m_payloadCrc)
            return Fail(aErr, "payload crc mismatch");
        {
            uint8_t h[64];
            std::memcpy(h, m_b.data(), 64);
            const uint32_t want = Rd<uint32_t>(40);
            std::memset(h + 40, 0, 4);
            if (Crc32(h, 64) != want)
                return Fail(aErr, "header crc mismatch");
        }
        for (size_t i = 44; i < 64; ++i)
            if (m_b[i] != 0)
                return Fail(aErr, "header reserved bytes not zero");
        if (nsec != 8 || 64 + 16ull * nsec > size)
            return Fail(aErr, "bad section count");

        struct Sec
        {
            size_t off = 0, size = 0, count = 0;
            bool seen = false;
        };
        static const char* const kTags[8] = {"BONE", "PARM", "LAYR", "SAMP", "TRIS", "DELT", "CLIP", "CVAL"};
        static const size_t kRec[8] = {36, 4, 240, 80, 72, kNumBones * kDeltaFloats * 4, 80, 4};
        Sec sec[8];
        for (uint32_t i = 0; i < nsec; ++i)
        {
            const size_t e = 64 + 16ull * i;
            int t = -1;
            for (int k = 0; k < 8; ++k)
                if (std::memcmp(m_b.data() + e, kTags[k], 4) == 0)
                    t = k;
            if (t < 0 || sec[t].seen)
                return Fail(aErr, "unknown or duplicate section tag");
            const uint64_t off = Rd<uint32_t>(e + 4), sz = Rd<uint32_t>(e + 8), cnt = Rd<uint32_t>(e + 12);
            if (off % 16 != 0 || off < 64 + 16ull * nsec || off + sz > size)
                return Fail(aErr, "section out of bounds", kTags[t]);
            if (sz != cnt * kRec[t])
                return Fail(aErr, "section size != count * record size", kTags[t]);
            sec[t] = Sec{static_cast<size_t>(off), static_cast<size_t>(sz), static_cast<size_t>(cnt), true};
        }
        for (int k = 0; k < 8; ++k)
            if (!sec[k].seen)
                return Fail(aErr, "missing section", kTags[k]);
        if (sec[0].count != kNumBones || sec[1].count != kParmCount)
            return Fail(aErr, "BONE / PARM count");
        auto finiteRange = [&](size_t off, size_t floats) {
            for (size_t i = 0; i < floats; ++i)
                if (!std::isfinite(Rd<float>(off + i * 4)))
                    return false;
            return true;
        };

        // BONE
        m_sk = Skeleton{};
        m_sk.n = kNumBones;
        for (int i = 0; i < kNumBones; ++i)
        {
            char nm[33];
            if (!StrOk(m_b.data() + sec[0].off + i * 36, 32, nm, sizeof(nm)))
                return Fail(aErr, "bad bone name");
            std::memcpy(m_boneName[i], nm, sizeof(nm));
            m_sk.parent[i] = static_cast<int16_t>(Rd<int32_t>(sec[0].off + i * 36 + 32));
        }
        if (!m_sk.Valid())
            return Fail(aErr, "bad bone parents");

        // PARM
        if (!finiteRange(sec[1].off, kParmCount))
            return Fail(aErr, "non-finite PARM");
        {
            float f[kParmCount];
            std::memcpy(f, m_b.data() + sec[1].off, sizeof(f));
            m_p = Params{};
            m_p.latSpeedLo = f[0];
            m_p.latSpeedHi = f[1];
            m_p.latAccelLo = f[2];
            m_p.latAccelHi = f[3];
            m_p.latClampLo = f[4];
            m_p.latClampHi = f[5];
            m_p.xfadeSpeedLo = f[6];
            m_p.xfadeSpeedHi = f[7];
            m_p.xfadeInterpPerS = f[8];
            m_p.boolBlendS = f[9];
            m_p.stateBlendS = f[10];
            m_p.disableCurveInterpPerS = f[11];
            m_p.cmcSpeedLo = f[12];
            m_p.cmcSpeedHi = f[13];
            m_p.cmcScaleLo = f[14];
            m_p.cmcScaleHi = f[15];
            m_p.cmcAlphaBlendS = f[16];
            m_p.cmcWeightInterpPerS = f[17];
            m_p.headWeightInterpPerS = f[18];
            m_p.bodyWeightInterpPerS = f[19];
            for (int d = 0; d < kNumDirections; ++d)
            {
                m_p.dirScale[d][0] = f[20 + 2 * d];
                m_p.dirScale[d][1] = f[21 + 2 * d];
            }
            for (int i = 32; i < kParmCount; ++i)
                if (f[i] != 0.0f)
                    return Fail(aErr, "PARM reserved not zero");
            if (!(m_p.latSpeedHi > m_p.latSpeedLo && m_p.latAccelLo > 0 && m_p.latAccelHi >= m_p.latAccelLo &&
                  m_p.latClampLo < m_p.latClampHi && m_p.xfadeSpeedHi > m_p.xfadeSpeedLo && m_p.xfadeInterpPerS > 0 &&
                  m_p.boolBlendS >= 0 && m_p.stateBlendS >= 0 && m_p.disableCurveInterpPerS > 0 &&
                  m_p.cmcSpeedHi > m_p.cmcSpeedLo && m_p.cmcAlphaBlendS >= 0 && m_p.cmcWeightInterpPerS > 0 &&
                  m_p.headWeightInterpPerS > 0 && m_p.bodyWeightInterpPerS > 0))
                return Fail(aErr, "PARM out of range");
        }

        // LAYR
        const size_t nLayers = sec[2].count, nSamples = sec[3].count, nTris = sec[4].count;
        if (nLayers == 0 || nLayers > static_cast<size_t>(kMaxLayers))
            return Fail(aErr, "layer count");
        if (sec[5].count != nSamples)
            return Fail(aErr, "DELT count != SAMP count");
        m_layers.resize(nLayers);
        size_t sCur = 0, tCur = 0;
        for (size_t li = 0; li < nLayers; ++li)
        {
            const size_t o = sec[2].off + li * 240;
            const uint8_t* b = m_b.data() + o;
            Layer& L = m_layers[li];
            if (!StrOk(b, 16, L.id, sizeof(L.id)) || !StrOk(b + 16, 48, L.blendspace, sizeof(L.blendspace)) ||
                !StrOk(b + 64, 64, L.baseClip, sizeof(L.baseClip)) || !StrOk(b + 128, 16, L.axis[0], sizeof(L.axis[0])) ||
                !StrOk(b + 144, 16, L.axis[1], sizeof(L.axis[1])) || L.id[0] == 0 || L.blendspace[0] == 0 ||
                L.baseClip[0] == 0)
                return Fail(aErr, "bad layer strings");
            for (size_t k = 0; k < li; ++k)
                if (std::strcmp(m_layers[k].id, L.id) == 0)
                    return Fail(aErr, "duplicate layer id");
            const uint32_t dims = Rd<uint32_t>(o + 160), style = Rd<uint32_t>(o + 164), role = Rd<uint32_t>(o + 172);
            L.baseFrame = static_cast<int>(Rd<uint32_t>(o + 168));
            const uint32_t fs = Rd<uint32_t>(o + 176), ns = Rd<uint32_t>(o + 180), ft = Rd<uint32_t>(o + 184),
                           nt = Rd<uint32_t>(o + 188);
            if ((dims != 1 && dims != 2) || style > 1 || role > 2)
                return Fail(aErr, "bad layer dims / style / role");
            L.dims = static_cast<int>(dims);
            L.style = static_cast<Style>(style);
            L.role = static_cast<Role>(role);
            L.firstSample = static_cast<int>(fs);
            L.numSamples = static_cast<int>(ns);
            L.firstTri = static_cast<int>(ft);
            L.numTris = static_cast<int>(nt);
            if (fs != sCur || ns < (dims == 1 ? 2u : 3u) || ns > static_cast<uint32_t>(kMaxSamples) || sCur + ns > nSamples)
                return Fail(aErr, "layer sample range");
            if (ft != tCur || nt == 0 || tCur + nt > nTris)
                return Fail(aErr, "layer triangle range");
            sCur += ns;
            tCur += nt;
            if (!finiteRange(o + 192, 5))
                return Fail(aErr, "non-finite layer floats");
            L.min[0] = Rd<float>(o + 192);
            L.max[0] = Rd<float>(o + 196);
            L.min[1] = Rd<float>(o + 200);
            L.max[1] = Rd<float>(o + 204);
            L.interpPerS = Rd<float>(o + 208);
            if (!(L.max[0] > L.min[0]) || (dims == 2 && !(L.max[1] > L.min[1])) || !(L.interpPerS > 0.0f))
                return Fail(aErr, "layer axis / interp");
            for (size_t k = 212; k < 240; ++k)
                if (b[k - 0] != 0 && k >= 212)
                    return Fail(aErr, "layer reserved not zero");
        }
        if (sCur != nSamples || tCur != nTris)
            return Fail(aErr, "layers do not cover SAMP / TRIS");

        // SAMP
        m_samples.resize(nSamples);
        for (size_t li = 0; li < nLayers; ++li)
        {
            const Layer& L = m_layers[li];
            for (int k = 0; k < L.numSamples; ++k)
            {
                const size_t g = static_cast<size_t>(L.firstSample + k);
                const size_t o = sec[3].off + g * 80;
                SampleRec& s = m_samples[g];
                if (!finiteRange(o, 2) || !StrOk(m_b.data() + o + 16, 64, s.clip, sizeof(s.clip)) || s.clip[0] == 0 ||
                    Rd<uint32_t>(o + 12) != 0)
                    return Fail(aErr, "bad sample record");
                s.x = Rd<float>(o);
                s.y = Rd<float>(o + 4);
                s.frame = static_cast<int>(Rd<uint32_t>(o + 8));
                if (s.x < L.min[0] || s.x > L.max[0] || (L.dims == 2 && (s.y < L.min[1] || s.y > L.max[1])))
                    return Fail(aErr, "sample outside axis range");
                if (L.dims == 1 && s.y != 0.0f)
                    return Fail(aErr, "1D sample with y");
                for (int j = 0; j < k; ++j)
                {
                    const SampleRec& q = m_samples[static_cast<size_t>(L.firstSample + j)];
                    if (q.x == s.x && (L.dims == 1 || q.y == s.y))
                        return Fail(aErr, "duplicate sample position");
                }
            }
        }

        // TRIS
        m_tris.resize(nTris);
        for (size_t li = 0; li < nLayers; ++li)
        {
            const Layer& L = m_layers[li];
            for (int k = 0; k < L.numTris; ++k)
            {
                const size_t o = sec[4].off + static_cast<size_t>(L.firstTri + k) * 72;
                int32_t r[18];
                std::memcpy(r, m_b.data() + o, 72);
                Tri& T = m_tris[static_cast<size_t>(L.firstTri + k)];
                for (int v = 0; v < 3; ++v)
                    if (r[v] < 0 || r[v] >= L.numSamples)
                        return Fail(aErr, "triangle sample index");
                if (L.dims == 1)
                {
                    if (r[1] != r[2])
                        return Fail(aErr, "1D segment form");
                    for (int e = 3; e < 18; ++e)
                        if (r[e] != -1)
                            return Fail(aErr, "1D segment edge info");
                    const float x0 = m_samples[static_cast<size_t>(L.firstSample + r[0])].x;
                    const float x1 = m_samples[static_cast<size_t>(L.firstSample + r[1])].x;
                    if (!(x1 > x0))
                        return Fail(aErr, "1D segments not ascending");
                    if (k > 0 && m_tris[static_cast<size_t>(L.firstTri + k - 1)].s[1] != r[0])
                        return Fail(aErr, "1D segments not contiguous");
                    T = Tri{};
                    T.s[0] = r[0];
                    T.s[1] = r[1];
                    T.s[2] = r[2];
                    continue;
                }
                if (r[0] == r[1] || r[1] == r[2] || r[0] == r[2])
                    return Fail(aErr, "degenerate triangle");
                T.s[0] = r[0];
                T.s[1] = r[1];
                T.s[2] = r[2];
                for (int v = 0; v < 3; ++v)
                {
                    const SampleRec& sr = m_samples[static_cast<size_t>(L.firstSample + r[v])];
                    T.vx[v] = (sr.x - L.min[0]) / (L.max[0] - L.min[0]);
                    T.vy[v] = (sr.y - L.min[1]) / (L.max[1] - L.min[1]);
                }
                const float area2 = (T.vx[1] - T.vx[0]) * (T.vy[2] - T.vy[0]) - (T.vy[1] - T.vy[0]) * (T.vx[2] - T.vx[0]);
                if (!(area2 > 1e-9f))
                    return Fail(aErr, "triangle not counter-clockwise / degenerate");
                for (int v = 0; v < 3; ++v)
                {
                    const int vn = (v + 1) % 3;
                    const float dx = T.vx[vn] - T.vx[v], dy = T.vy[vn] - T.vy[v];
                    const float nx = dy, ny = -dx;          // outward normal of a CCW triangle
                    const float l2 = nx * nx + ny * ny;
                    const float inv = l2 > 1e-8f ? 1.0f / std::sqrt(l2) : 0.0f; // FVector2D::GetSafeNormal
                    T.nx[v] = nx * inv;
                    T.ny[v] = ny * inv;
                    const int32_t* e = r + 3 + 5 * v;
                    T.nbr[v] = e[0];
                    T.pt[v][0] = e[1];
                    T.pt[v][1] = e[2];
                    T.pv[v][0] = e[3];
                    T.pv[v][1] = e[4];
                    for (int q = 0; q < 3; ++q)
                        if (e[q] < -1 || e[q] >= L.numTris)
                            return Fail(aErr, "triangle link index");
                    for (int q = 3; q < 5; ++q)
                        if (e[q] < -1 || e[q] > 2)
                            return Fail(aErr, "perimeter vertex index");
                }
            }
            if (L.dims == 2)
            {
                // topology: neighbours reciprocal over the shared (reversed) edge; perimeter links only on free edges
                for (int k = 0; k < L.numTris; ++k)
                {
                    const Tri& T = m_tris[static_cast<size_t>(L.firstTri + k)];
                    for (int v = 0; v < 3; ++v)
                    {
                        const int vn = (v + 1) % 3;
                        if (T.nbr[v] >= 0)
                        {
                            if (T.nbr[v] == k)
                                return Fail(aErr, "triangle is its own neighbour");
                            const Tri& N = m_tris[static_cast<size_t>(L.firstTri + T.nbr[v])];
                            bool found = false;
                            for (int w = 0; w < 3; ++w)
                                if (N.s[w] == T.s[vn] && N.s[(w + 1) % 3] == T.s[v] && N.nbr[w] == k)
                                    found = true;
                            if (!found)
                                return Fail(aErr, "neighbour link not reciprocal");
                            if (T.pt[v][0] != -1 || T.pt[v][1] != -1)
                                return Fail(aErr, "perimeter links on an interior edge");
                        }
                        else
                            for (int d = 0; d < 2; ++d)
                            {
                                const int pt = T.pt[v][d], pv = T.pv[v][d];
                                if ((pt < 0) != (pv < 0))
                                    return Fail(aErr, "perimeter link pair");
                                if (pt >= 0 && m_tris[static_cast<size_t>(L.firstTri + pt)].nbr[pv] != -1)
                                    return Fail(aErr, "perimeter link not on a free edge");
                            }
                    }
                }
            }
        }

        // DELT
        if (!finiteRange(sec[5].off, sec[5].size / 4))
            return Fail(aErr, "non-finite delta");
        m_secDelt = sec[5].off;
        for (size_t g = 0; g < nSamples; ++g)
            for (int i = 0; i < kNumBones; ++i)
            {
                const float* d = reinterpret_cast<const float*>(m_b.data() + m_secDelt + (g * kNumBones + i) * kDeltaFloats * 4);
                const float l = d[0] * d[0] + d[1] * d[1] + d[2] * d[2] + d[3] * d[3];
                if (!(std::fabs(l - 1.0f) < 2e-3f))
                    return Fail(aErr, "delta quaternion not unit");
            }

        // CLIP / CVAL
        const size_t nClips = sec[6].count, nVals = sec[7].count;
        if (!finiteRange(sec[7].off, nVals))
            return Fail(aErr, "non-finite curve value");
        m_cval.resize(nVals);
        for (size_t i = 0; i < nVals; ++i)
        {
            m_cval[i] = Rd<float>(sec[7].off + i * 4);
            if (m_cval[i] < 0.0f || m_cval[i] > 1.0f)
                return Fail(aErr, "curve value outside 0..1");
        }
        m_clips.resize(nClips);
        for (size_t i = 0; i < nClips; ++i)
        {
            const size_t o = sec[6].off + i * 80;
            ClipRec& c = m_clips[i];
            if (!StrOk(m_b.data() + o, 64, c.name, sizeof(c.name)) || c.name[0] == 0)
                return Fail(aErr, "bad clip name");
            const uint32_t first = Rd<uint32_t>(o + 64), count = Rd<uint32_t>(o + 68), kind = Rd<uint32_t>(o + 72);
            c.fps = Rd<float>(o + 76);
            if (count == 0 || kind > 1 || !(c.fps > 0.0f) || !std::isfinite(c.fps) || static_cast<size_t>(first) + count > nVals)
                return Fail(aErr, "clip curve range");
            c.first = static_cast<int>(first);
            c.count = static_cast<int>(count);
            c.kind = static_cast<int>(kind);
            for (size_t k = 0; k < i; ++k)
                if (std::strcmp(m_clips[k].name, c.name) == 0)
                    return Fail(aErr, "duplicate clip curve name");
        }
        return true;
    }

    bool m_ok = false;
    std::vector<uint8_t> m_b;
    uint64_t m_rigHash = 0;
    uint32_t m_payloadCrc = 0;
    size_t m_secDelt = 0;
    Params m_p;
    Skeleton m_sk;
    char m_boneName[kNumBones][33]{};
    std::vector<Layer> m_layers;
    std::vector<SampleRec> m_samples;
    std::vector<Tri> m_tris;
    std::vector<ClipRec> m_clips;
    std::vector<float> m_cval;
};

// Per-layer smoothing of the sample weights + triangle cache. Fixed storage, no allocation per frame.
struct LayerState
{
    float w[kMaxSamples]{}, rate[kMaxSamples]{}, out[kMaxSamples]{};
    int n = 0;
    bool primed = false;
    int tri = -1;
    bool easeInOut = true; // bTargetWeightInterpolationEaseInOut default

    void Reset()
    {
        *this = LayerState{};
    }

    // Evaluates aLayer at (aX, aY) and advances the smoothed weights by aDt; returns the normalised smoothed weights
    // (numSamples entries, layer-local sample order) or nullptr when the evaluation fails. aSpeed < 0 uses the layer's
    // weight_interp_per_s. The first call (or aSpeed == 0) snaps to the target.
    const float* Update(const LeanPack& aPack, int aLayer, float aX, float aY, float aDt, float aSpeed = -1.0f)
    {
        Weights tw;
        if (!aPack.Evaluate(aLayer, aX, aY, tri, tw))
            return nullptr;
        const Layer& L = aPack.LayerAt(aLayer);
        const float speed = aSpeed < 0.0f ? L.interpPerS : aSpeed;
        n = L.numSamples;
        float t[kMaxSamples]{};
        for (int i = 0; i < tw.n; ++i)
            t[tw.sample[i]] += tw.w[i];
        if (!primed || speed <= 0.0f)
        {
            for (int i = 0; i < n; ++i)
            {
                w[i] = t[i];
                rate[i] = 0.0f;
            }
            primed = true;
        }
        else
        {
            const float st = 1.0f / (2.71828182845904523536f * speed);
            for (int i = 0; i < n; ++i)
            {
                if (easeInOut)
                    bs::CriticallyDampedSmoothing(w[i], rate[i], t[i], aDt, st);
                else
                    w[i] = bs::FInterpConstantTo(w[i], t[i], aDt, speed);
                if (w[i] <= 1e-5f)
                {
                    w[i] = 0.0f;
                    rate[i] = 0.0f;
                }
            }
        }
        float sum = 0.0f;
        for (int i = 0; i < n; ++i)
            sum += w[i];
        for (int i = 0; i < n; ++i)
            out[i] = sum > 1e-6f ? w[i] / sum : 0.0f;
        return out;
    }
};

// ---- ABP gates / inputs (metres in, UE formulas on cm) -------------------------------------------------------------
// Lateral acceleration amount of the Mover ABP: lat = clamp(a_right / MapRangeClamped(speed 200..320 -> 500..800), -1, 1)
// with a_right = component of the acceleration to the RIGHT of the velocity heading (CP world: right of (hx, hy) is
// (hy, -hx)). Velocity in m/s, acceleration in m/s^2 (planar x, y). Returns 0 when the planar speed is ~0.
inline float LateralAccelAmount(const Params& aP, float aVelX, float aVelY, float aAccX, float aAccY)
{
    const float sp = std::sqrt(aVelX * aVelX + aVelY * aVelY);
    if (!(sp > 1e-4f) || !std::isfinite(aAccX) || !std::isfinite(aAccY))
        return 0.0f;
    const float hx = aVelX / sp, hy = aVelY / sp;
    const float aRightCm = (aAccX * hy - aAccY * hx) * 100.0f;
    const float denom = ov::MapRangeClamped(sp * 100.0f, aP.latSpeedLo, aP.latSpeedHi, aP.latAccelLo, aP.latAccelHi);
    return std::clamp(aRightCm / denom, aP.latClampLo, aP.latClampHi);
}

// (x, y) blend space input from lat and the movement direction (enum Direction): lat * dirScale.
inline void LeanInput(const Params& aP, int aDirection, float aLat, float& aX, float& aY)
{
    if (aDirection < 0 || aDirection >= kNumDirections)
    {
        aX = aY = 0.0f;
        return;
    }
    aX = aLat * aP.dirScale[aDirection][0];
    aY = aLat * aP.dirScale[aDirection][1];
}

// Walk -> run crossfade target (TwoWayBlend alpha): MapRangeClamped(speed 200..320 cm/s -> 0..1); speed in m/s.
inline float WalkRunAlpha(const Params& aP, float aSpeedMs)
{
    return ov::MapRangeClamped(aSpeedMs * 100.0f, aP.xfadeSpeedLo, aP.xfadeSpeedHi, 0.0f, 1.0f);
}

// CMC variant (Neutral BS1D "cmc_run"): x = rel_accel_lateral * MapRangeClamped(speed 165..375 -> 0.5..1), clamped.
inline float CmcLeanX(const Params& aP, float aRelAccelLateral, float aSpeedMs)
{
    const float s = ov::MapRangeClamped(aSpeedMs * 100.0f, aP.cmcSpeedLo, aP.cmcSpeedHi, aP.cmcScaleLo, aP.cmcScaleHi);
    return std::clamp(aRelAccelLateral * s, -1.0f, 1.0f);
}

} // namespace utpp::mm::lean
