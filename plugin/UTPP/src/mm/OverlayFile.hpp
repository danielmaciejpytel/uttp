// UTPP motion matching runtime - .utppov overlay data reader (Source\tools\mmbake\FORMAT_OVERLAY.txt, version 1).
// Header-only. Fail-closed: magic / version / header size / file size / header + payload CRC-32 / rig hash /
// section table (tags, bounds, record sizes) / cross-section ranges / non-finite floats reject the whole file.
#pragma once

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>

#include "Overlay.hpp"

namespace utpp::mm::ov
{
inline uint32_t OvCrc32(const uint8_t* aData, size_t aSize, uint32_t aCrc = 0)
{
    aCrc = ~aCrc;
    for (size_t i = 0; i < aSize; ++i)
    {
        aCrc ^= aData[i];
        for (int k = 0; k < 8; ++k)
            aCrc = (aCrc >> 1) ^ (0xEDB88320u & (0u - (aCrc & 1u)));
    }
    return ~aCrc;
}

inline constexpr uint64_t kRigIdentityWomanOv = 0xF62810A88A8BFDA9ull; // = MotionDatabase kRigIdentityWoman
inline constexpr uint64_t kRigIdentityManOv = 0xAFE2CFEDCF97CDF7ull;   // = MotionDatabase kRigIdentityMan

class OverlayFile
{
public:
    static constexpr uint32_t kVersion = 1;
    static constexpr uint32_t kHeaderSize = 64;

    struct PoseRec
    {
        char name[64];
        uint32_t group, firstFrame, numFrames, additiveType;
        int32_t basePose;
        uint32_t baseFrame, baseKind, reserved;
        uint64_t curveMask;
    };
    struct DeltaRec
    {
        char name[48];
        uint32_t kind;
        int32_t targetPose;
        uint32_t targetFirstFrame, numFrames;
        int32_t basePose;
        uint32_t baseFrame, space, first;
    };
    struct OverlayRec
    {
        char name[32];
        uint32_t kind, flags, firstPose, numPoses;
    };
    static_assert(sizeof(PoseRec) == 104 && sizeof(DeltaRec) == 80 && sizeof(OverlayRec) == 48);

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
        if (sz > 0)
        {
            bytes.resize(static_cast<size_t>(sz));
            if (std::fread(bytes.data(), 1, bytes.size(), f) != bytes.size())
                bytes.clear();
        }
        std::fclose(f);
        return LoadFromMemory(std::move(bytes), aExpectedRigHash, aErr);
    }

    bool LoadFromMemory(std::vector<uint8_t> aBytes, uint64_t aExpectedRigHash, std::string& aErr)
    {
        m_ok = false;
        m_b = std::move(aBytes);
        if (!Parse(aExpectedRigHash, aErr))
        {
            m_b.clear();
            return false;
        }
        m_ok = true;
        return true;
    }

    bool Ok() const { return m_ok; }
    uint64_t RigHash() const { return m_rigHash; }
    int NumBones() const { return m_nb; }
    int NumCurves() const { return m_nc; }
    int NumPoses() const { return m_np; }
    int NumFrames() const { return m_nf; }
    int NumDeltas() const { return m_nd; }
    int NumOverlays() const { return m_no; }
    const Skeleton& Skel() const { return m_sk; }
    const Regions& Masks() const { return m_regions; }
    std::string_view BoneName(int i) const { return Str(m_b.data() + m_secBone + i * 36, 32); }
    std::string_view CurveFileName(int i) const { return Str(m_b.data() + m_secCurv + i * 48, 48); }
    int CurveIndexOf(int aLayerCurve) const { return m_curveMap[aLayerCurve]; }
    const PoseRec& PoseAt(int i) const { return Rec<PoseRec>(m_secPose, i); }
    const DeltaRec& DeltaAt(int i) const { return Rec<DeltaRec>(m_secDelt, i); }
    const OverlayRec& OverlayAt(int i) const { return Rec<OverlayRec>(m_secOvly, i); }
    std::string_view PoseName(int i) const { return Str(PoseAt(i).name, 64); }
    std::string_view DeltaName(int i) const { return Str(DeltaAt(i).name, 48); }
    std::string_view OverlayName(int i) const { return Str(OverlayAt(i).name, 32); }

    int FindPose(std::string_view n) const { return Find(n, m_np, [&](int i) { return PoseName(i); }); }
    int FindDelta(std::string_view n) const { return Find(n, m_nd, [&](int i) { return DeltaName(i); }); }

    // Frame f (global FRAM index): local pose.
    void FramePose(int aFrame, Pose& aOut) const
    {
        const float* p = FramePtr(aFrame);
        aOut.n = m_nb;
        for (int i = 0; i < m_nb; ++i, p += 17)
            aOut.b[i] = Xf{Quat{p[3], p[4], p[5], p[6]}, Vec3{p[0], p[1], p[2]}, Vec3{p[7], p[8], p[9]}};
    }
    Quat FrameMeshRot(int aFrame, int aBone) const
    {
        const float* p = FramePtr(aFrame) + aBone * 17;
        return Quat{p[10], p[11], p[12], p[13]};
    }
    // Layering curves of a frame; presence from the pose curve mask.
    void FrameCurves(int aPose, int aFrame, Curves& aOut) const
    {
        aOut = Curves{};
        const float* c = FramePtr(aFrame) + m_nb * 17;
        const uint64_t mask = PoseAt(aPose).curveMask;
        for (int k = 0; k < kCurveCount; ++k)
        {
            const int ci = m_curveMap[k];
            if (ci >= 0 && (mask >> ci) & 1ull)
                aOut.Set(k, c[ci]);
        }
    }
    // Delta d, frame k (0..numFrames-1). aMeshSpace selects dq_ms (A_MS) or dq_ls (A_LS).
    void DeltaFrame(int aDelta, int aFrame, bool aMeshSpace, Additive& aOut) const
    {
        const DeltaRec& d = DeltaAt(aDelta);
        const float* p = reinterpret_cast<const float*>(m_b.data() + m_secDfrm) +
                         static_cast<size_t>(d.first + aFrame) * m_nb * 14;
        aOut.n = m_nb;
        aOut.meshSpace = aMeshSpace;
        for (int i = 0; i < m_nb; ++i, p += 14)
        {
            const float* q = aMeshSpace ? p : p + 4;
            aOut.b[i] = Xf{Quat{q[0], q[1], q[2], q[3]}, Vec3{p[8], p[9], p[10]}, Vec3{p[11], p[12], p[13]}};
        }
    }

    // Overlay pose for the state machine: "Pose_<Overlay>_<Stand|Crouch>_<Relax|Ready|Aim>[_Idle|_Move|_Run]",
    // crouch poses without the gait suffix, Injured "Pose_Injured_<Stand_Idle|Stand_Move|Crouch>". -1 = none.
    int FindOverlayPose(PoseOverlay aOv, AimState aAim, bool aCrouch, bool aMoving) const
    {
        static const char* kOv[] = {"Default", "Rifle", "Pistol1H", "Pistol2H", "Injured", "Game"};
        static const char* kAim[] = {"Relax", "Ready", "Aim"};
        if (aOv == PoseOverlay::Default || aOv == PoseOverlay::Game)
            return -1;
        std::string base = std::string("Pose_") + kOv[static_cast<int>(aOv)] + (aCrouch ? "_Crouch" : "_Stand");
        if (aOv != PoseOverlay::Injured)
            base += std::string("_") + kAim[static_cast<int>(aAim)];
        const char* sfx[] = {aMoving ? "_Move" : "_Idle", aMoving ? "_Run" : "", aMoving ? "_Idle" : "_Move", ""};
        for (const char* s : sfx)
            if (const int i = FindPose(base + s); i >= 0)
                return i;
        if (aAim != AimState::Relaxed) // fall back to a lower aim state of the same overlay
            return FindOverlayPose(aOv, static_cast<AimState>(static_cast<int>(aAim) - 1), aCrouch, aMoving);
        return -1;
    }

private:
    template <class T>
    const T& Rec(size_t aSec, int i) const
    {
        return *reinterpret_cast<const T*>(m_b.data() + aSec + static_cast<size_t>(i) * sizeof(T));
    }
    const float* FramePtr(int aFrame) const
    {
        return reinterpret_cast<const float*>(m_b.data() + m_secFram) + static_cast<size_t>(aFrame) * (m_nb * 17 + m_nc);
    }
    static std::string_view Str(const void* p, size_t n)
    {
        const char* c = static_cast<const char*>(p);
        size_t l = 0;
        while (l < n && c[l])
            ++l;
        return std::string_view(c, l);
    }
    template <class F>
    static int Find(std::string_view n, int aCount, F aName)
    {
        for (int i = 0; i < aCount; ++i)
            if (aName(i) == n)
                return i;
        return -1;
    }
    bool Fail(std::string& aErr, const char* aWhy)
    {
        aErr = std::string(".utppov: ") + aWhy;
        return false;
    }
    template <class T>
    T Rd(size_t o) const
    {
        T v;
        std::memcpy(&v, m_b.data() + o, sizeof(T));
        return v;
    }

    bool Parse(uint64_t aExpectedRigHash, std::string& aErr)
    {
        const size_t size = m_b.size();
        if (size < kHeaderSize || std::memcmp(m_b.data(), "UTPPOV\0\0", 8) != 0)
            return Fail(aErr, "bad magic");
        if (Rd<uint32_t>(8) != kVersion || Rd<uint32_t>(12) != kHeaderSize)
            return Fail(aErr, "unsupported version / header size");
        if (Rd<uint32_t>(32) != size)
            return Fail(aErr, "file size mismatch");
        uint8_t hdr[kHeaderSize];
        std::memcpy(hdr, m_b.data(), kHeaderSize);
        std::memset(hdr + 40, 0, 4);
        if (OvCrc32(hdr, kHeaderSize) != Rd<uint32_t>(40))
            return Fail(aErr, "header CRC mismatch");
        if (OvCrc32(m_b.data() + kHeaderSize, size - kHeaderSize) != Rd<uint32_t>(36))
            return Fail(aErr, "payload CRC mismatch");
        m_rigHash = Rd<uint64_t>(16);
        if (m_rigHash != aExpectedRigHash)
            return Fail(aErr, "rig hash mismatch");
        m_nb = static_cast<int>(Rd<uint32_t>(24));
        if (m_nb <= 0 || m_nb > kMaxBones)
            return Fail(aErr, "bad bone count");
        const uint32_t ns = Rd<uint32_t>(28);
        if (ns > 64 || kHeaderSize + static_cast<size_t>(ns) * 16 > size)
            return Fail(aErr, "bad section table");
        struct Sec
        {
            const char* tag;
            size_t* off;
            int* count;
        } secs[] = {{"BONE", &m_secBone, nullptr}, {"CURV", &m_secCurv, &m_nc}, {"POSE", &m_secPose, &m_np},
                    {"FRAM", &m_secFram, &m_nf},   {"DELT", &m_secDelt, &m_nd}, {"DFRM", &m_secDfrm, &m_ndf},
                    {"OVLY", &m_secOvly, &m_no},   {"MASK", &m_secMask, &m_nm}};
        uint32_t sizes[8]{};
        for (int s = 0; s < 8; ++s)
        {
            bool found = false;
            for (uint32_t k = 0; k < ns; ++k)
            {
                const size_t e = kHeaderSize + k * 16;
                if (std::memcmp(m_b.data() + e, secs[s].tag, 4) != 0)
                    continue;
                const uint32_t off = Rd<uint32_t>(e + 4), sz = Rd<uint32_t>(e + 8), cnt = Rd<uint32_t>(e + 12);
                if (found || off % 16 != 0 || off < kHeaderSize + ns * 16 || static_cast<size_t>(off) + sz > size)
                    return Fail(aErr, "bad section entry");
                *secs[s].off = off;
                sizes[s] = sz;
                if (secs[s].count)
                    *secs[s].count = static_cast<int>(cnt);
                else if (cnt != static_cast<uint32_t>(m_nb))
                    return Fail(aErr, "BONE count != num_bones");
                found = true;
            }
            if (!found)
                return Fail(aErr, "missing section");
        }
        const size_t nb = static_cast<size_t>(m_nb);
        if (m_nc < 0 || m_nc > 64 || sizes[0] != nb * 36 || sizes[1] != static_cast<size_t>(m_nc) * 48 ||
            sizes[2] != static_cast<size_t>(m_np) * 104 || sizes[3] != static_cast<size_t>(m_nf) * (nb * 17 + m_nc) * 4 ||
            sizes[4] != static_cast<size_t>(m_nd) * 80 || sizes[5] != static_cast<size_t>(m_ndf) * nb * 14 * 4 ||
            sizes[6] != static_cast<size_t>(m_no) * 48 || m_nm != 7 || sizes[7] != 7 * (16 + nb * 4))
            return Fail(aErr, "section size / count mismatch");
        // skeleton
        m_sk.n = m_nb;
        for (int i = 0; i < m_nb; ++i)
        {
            const int32_t p = Rd<int32_t>(m_secBone + i * 36 + 32);
            if (p >= i || p < -1)
                return Fail(aErr, "bone parents not topological");
            m_sk.parent[i] = static_cast<int16_t>(p);
        }
        // curves
        for (int k = 0; k < kCurveCount; ++k)
        {
            m_curveMap[k] = -1;
            for (int c = 0; c < m_nc; ++c)
                if (CurveFileName(c) == CurveName(k))
                    m_curveMap[k] = c;
        }
        // finite floats
        auto finite = [&](size_t off, size_t bytes) {
            for (size_t o = off; o + 4 <= off + bytes; o += 4)
                if (!std::isfinite(Rd<float>(o)))
                    return false;
            return true;
        };
        if (!finite(m_secFram, sizes[3]) || !finite(m_secDfrm, sizes[5]) || !finite(m_secMask, 0))
            return Fail(aErr, "non-finite value");
        // ranges
        for (int i = 0; i < m_np; ++i)
        {
            const PoseRec& p = PoseAt(i);
            if (static_cast<uint64_t>(p.firstFrame) + p.numFrames > static_cast<uint64_t>(m_nf) || p.numFrames == 0 ||
                p.basePose < -1 || p.basePose >= m_np || p.group > 4 || p.additiveType > 2)
                return Fail(aErr, "POSE range");
        }
        for (int i = 0; i < m_nd; ++i)
        {
            const DeltaRec& d = DeltaAt(i);
            if (static_cast<uint64_t>(d.first) + d.numFrames > static_cast<uint64_t>(m_ndf) || d.targetPose < 0 ||
                d.targetPose >= m_np || d.basePose < 0 || d.basePose >= m_np || (d.space != 1 && d.space != 2))
                return Fail(aErr, "DELT range");
        }
        for (int i = 0; i < m_no; ++i)
        {
            const OverlayRec& o = OverlayAt(i);
            if (static_cast<uint64_t>(o.firstPose) + o.numPoses > static_cast<uint64_t>(m_np) || o.kind > 3)
                return Fail(aErr, "OVLY range");
        }
        // masks -> regions
        static const char* kMasks[7] = {"pelvis", "spine", "head", "arm_l", "arm_r", "hand_l", "hand_r"};
        BoneWeights* dst[7] = {&m_regions.pelvis, &m_regions.spine, &m_regions.head, &m_regions.armL,
                               &m_regions.armR, &m_regions.handL, &m_regions.handR};
        for (int m = 0; m < 7; ++m)
        {
            int found = -1;
            for (int k = 0; k < 7; ++k)
                if (Str(m_b.data() + m_secMask + k * (16 + nb * 4), 16) == kMasks[m])
                    found = k;
            if (found < 0)
                return Fail(aErr, "MASK missing");
            const size_t o = m_secMask + found * (16 + nb * 4) + 16;
            dst[m]->Clear(m_nb);
            for (int i = 0; i < m_nb; ++i)
            {
                const float w = Rd<float>(o + i * 4);
                if (!std::isfinite(w) || w < 0.0f || w > 1.0f)
                    return Fail(aErr, "MASK weight");
                dst[m]->w[i] = w;
            }
        }
        m_regions.spineYawCount = 0;
        for (const char* s : {"Spine", "Spine1", "Spine2", "Spine3"})
            for (int i = 0; i < m_nb; ++i)
                if (BoneName(i) == s && m_regions.spineYawCount < 8)
                    m_regions.spineYawBones[m_regions.spineYawCount++] = i;
        return true;
    }

    std::vector<uint8_t> m_b;
    bool m_ok = false;
    uint64_t m_rigHash = 0;
    int m_nb = 0, m_nc = 0, m_np = 0, m_nf = 0, m_nd = 0, m_ndf = 0, m_no = 0, m_nm = 0;
    size_t m_secBone = 0, m_secCurv = 0, m_secPose = 0, m_secFram = 0, m_secDelt = 0, m_secDfrm = 0, m_secOvly = 0,
           m_secMask = 0;
    int m_curveMap[kCurveCount]{};
    Skeleton m_sk;
    Regions m_regions;
};

} // namespace utpp::mm::ov
