// UTPP Motion Matching database (.utppmm v1) reader + search. Header-only, C++20, no external dependencies.
//
// Format spec: Source/tools/mmbake/FORMAT.txt (baker: Source/tools/mmbake/mmbake_base.py).
// Fails closed: any bad magic / version / size / CRC / rig hash / bounds / non-finite value leaves the database
// empty (Loaded() == false) and returns false with a reason string.
//
// Feature normalization and the small/large AABB search follow Daniel Holden's Motion-Matching reference
// implementation (orangeduck, MIT License, Copyright (c) 2021 Daniel Holden; database.h motion_matching_search).
// All section data is copied into typed vectors (memcpy), so no unaligned / aliasing access to the file bytes.
#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iterator>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include "Fnv1a64.hpp"

namespace utpp::mm
{
static_assert(std::endian::native == std::endian::little, "utppmm reader assumes a little-endian host");

inline constexpr char kMagic[8] = {'U', 'T', 'P', 'P', 'M', 'M', 'D', 'B'};
inline constexpr uint32_t kVersion = 1;
inline constexpr uint32_t kHeaderSize = 128;
inline constexpr uint32_t kSectionEntrySize = 32;
inline constexpr uint32_t kMaxSections = 64;
inline constexpr uint32_t kMaxBones = 1024;
inline constexpr uint32_t kMaxFeatures = 256;
inline constexpr uint32_t kMaxHorizons = 4;

enum HeaderFlags : uint32_t
{
    kHeaderHasScale = 1u << 0,
};

enum FrameFlags : uint32_t
{
    kFrameSearchable = 1u << 0,
    kFrameExcludedNotify = 1u << 1,
    kFrameHorizonPastEnd = 1u << 2,
    kFrameLoopDuplicate = 1u << 3,
    kFrameBlockTransition = 1u << 4,
    kFrameBranchIn = 1u << 5,
    kFrameClipStart = 1u << 6,
    kFrameClipEnd = 1u << 7,
};

enum ClipFlags : uint32_t
{
    kClipLoop = 1u << 0,
    kClipEndsPoseMatch = 1u << 1,
    kClipNameLoop = 1u << 2,
};

enum class GroupKind : uint32_t
{
    BonePosition = 1,
    BoneVelocity = 2,
    TrajectoryPosition = 3,
    TrajectoryDirection = 4,
};

constexpr uint32_t FourCC(const char (&aS)[5])
{
    return static_cast<uint32_t>(static_cast<uint8_t>(aS[0])) | (static_cast<uint32_t>(static_cast<uint8_t>(aS[1])) << 8) |
           (static_cast<uint32_t>(static_cast<uint8_t>(aS[2])) << 16) |
           (static_cast<uint32_t>(static_cast<uint8_t>(aS[3])) << 24);
}

// ---------------------------------------------------------------- hashing / CRC

namespace detail
{
constexpr std::array<uint32_t, 256> MakeCrcTable()
{
    std::array<uint32_t, 256> t{};
    for (uint32_t i = 0; i < 256; ++i)
    {
        uint32_t c = i;
        for (int k = 0; k < 8; ++k)
            c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        t[i] = c;
    }
    return t;
}
inline constexpr std::array<uint32_t, 256> kCrcTable = MakeCrcTable();
} // namespace detail

// CRC-32 (IEEE 802.3, reflected, init/xorout 0xFFFFFFFF) == zlib.crc32.
inline uint32_t Crc32(const uint8_t* aData, size_t aSize, uint32_t aCrc = 0)
{
    uint32_t c = ~aCrc;
    for (size_t i = 0; i < aSize; ++i)
        c = detail::kCrcTable[(c ^ aData[i]) & 0xFFu] ^ (c >> 8);
    return ~c;
}

using util::Fnv1a64; // FNV-1a 64, continuable through the third argument (Fnv1a64.hpp)

// Rig hash: FNV-1a 64 over, for each bone in order: UTF-8 name bytes, one 0x00 byte, int32 LE parent index.
inline uint64_t ComputeRigHash(std::span<const std::string_view> aNames, std::span<const int32_t> aParents)
{
    uint64_t h = 0xCBF29CE484222325ull;
    const size_t n = std::min(aNames.size(), aParents.size());
    for (size_t i = 0; i < n; ++i)
    {
        h = Fnv1a64(reinterpret_cast<const uint8_t*>(aNames[i].data()), aNames[i].size(), h);
        const uint8_t zero = 0;
        h = Fnv1a64(&zero, 1, h);
        uint8_t p[4];
        std::memcpy(p, &aParents[i], 4);
        h = Fnv1a64(p, 4, h);
    }
    return h;
}

// ---------------------------------------------------------------- on-disk records (little-endian, packed by layout)

struct Header
{
    char magic[8];
    uint32_t version;
    uint32_t headerSize;
    uint64_t fileSize;
    uint64_t rigHash;
    uint32_t flags;
    uint32_t numFrames;
    uint32_t numBones;
    uint32_t numFeatures;
    uint32_t numClips;
    uint32_t numRanges;
    uint32_t numGroups;
    uint32_t numTags;
    uint32_t boundSmSize;
    uint32_t boundLrSize;
    uint32_t numBoundSm;
    uint32_t numBoundLr;
    float fps;
    uint32_t numSections;
    uint32_t sectionTableOffset;
    uint32_t payloadCrc32;
    uint32_t headerCrc32;
    uint8_t reserved[28];
};
static_assert(sizeof(Header) == kHeaderSize);
static_assert(offsetof(Header, headerCrc32) == 96);

struct SectionEntry
{
    uint32_t id;
    uint32_t count;
    uint32_t elemSize;
    uint32_t reserved;
    uint64_t offset;
    uint64_t size;
};
static_assert(sizeof(SectionEntry) == kSectionEntrySize);

struct BoneRecord
{
    uint32_t nameOffset;
    int32_t parent;
    uint32_t nameLength;
    uint32_t reserved;
};
static_assert(sizeof(BoneRecord) == 16);

struct ClipRecord
{
    uint32_t nameOffset;
    uint32_t firstFrame;
    uint32_t numFrames;
    uint32_t flags;
    uint32_t loopPeriod;
    uint32_t numSearchable;
    uint64_t tagMask;
};
static_assert(sizeof(ClipRecord) == 32);

struct GroupRecord
{
    uint32_t nameOffset;
    uint32_t kind;
    uint32_t firstFeature;
    uint32_t numFeatures;
    int32_t bone;
    float weight;
    float std;
    uint32_t numHorizons;
    int32_t horizonFrames[4];
};
static_assert(sizeof(GroupRecord) == 48);

struct Range
{
    uint32_t start; // first global frame (inclusive)
    uint32_t stop;  // one past last global frame
};
static_assert(sizeof(Range) == 8);

// Per-frame root motion: planar step (metres, in the Trajectory frame of this frame) and yaw step (radians, CCW
// about +Z) to go from this frame to the next played frame.
struct RootDelta
{
    float dx;
    float dy;
    float dyaw;
    float reserved;
};
static_assert(sizeof(RootDelta) == 16);

struct SearchResult
{
    int32_t index = -1;
    float cost = std::numeric_limits<float>::max();
};

// ---------------------------------------------------------------- database

class Database
{
public:
    // Loads from an in-memory copy of the whole file. aExpectedRigHash must equal the stored rig hash.
    bool LoadFromMemory(std::span<const uint8_t> aBytes, uint64_t aExpectedRigHash, std::string& aError)
    {
        Reset();
        const bool ok = Parse(aBytes, aExpectedRigHash, aError);
        if (!ok)
            Reset();
        return ok;
    }

    bool LoadFromFile(const char* aPath, uint64_t aExpectedRigHash, std::string& aError)
    {
        Reset();
        std::vector<uint8_t> bytes;
        if (!ReadFile(aPath, bytes, aError))
            return false;
        return LoadFromMemory(bytes, aExpectedRigHash, aError);
    }

    void Reset() { *this = Database{}; }

    bool Loaded() const { return m_loaded; }
    const Header& GetHeader() const { return m_header; }
    uint64_t RigHash() const { return m_header.rigHash; }
    uint32_t NumFrames() const { return m_header.numFrames; }
    uint32_t NumBones() const { return m_header.numBones; }
    uint32_t NumFeatures() const { return m_header.numFeatures; }
    uint32_t NumClips() const { return m_header.numClips; }
    float Fps() const { return m_header.fps; }
    bool HasScale() const { return (m_header.flags & kHeaderHasScale) != 0; }

    std::string_view BoneName(uint32_t aBone) const { return aBone < m_bones.size() ? Str(m_bones[aBone].nameOffset) : std::string_view{}; }
    int32_t BoneParent(uint32_t aBone) const { return aBone < m_bones.size() ? m_bones[aBone].parent : -1; }
    std::span<const ClipRecord> Clips() const { return m_clips; }
    std::string_view ClipName(uint32_t aClip) const { return aClip < m_clips.size() ? Str(m_clips[aClip].nameOffset) : std::string_view{}; }
    std::string_view TagName(uint32_t aTag) const { return aTag < m_tags.size() ? Str(m_tags[aTag]) : std::string_view{}; }
    uint32_t NumTags() const { return static_cast<uint32_t>(m_tags.size()); }
    std::span<const GroupRecord> Groups() const { return m_groups; }
    std::string_view GroupName(uint32_t aGroup) const { return aGroup < m_groups.size() ? Str(m_groups[aGroup].nameOffset) : std::string_view{}; }
    std::span<const Range> SearchRanges() const { return m_ranges; }
    std::span<const float> FeatureOffsets() const { return m_offset; }
    std::span<const float> FeatureScales() const { return m_scale; }

    // Returns the clip index containing a global frame, or -1.
    int32_t ClipOfFrame(uint32_t aFrame) const
    {
        if (aFrame >= m_header.numFrames)
            return -1;
        auto it = std::upper_bound(m_clips.begin(), m_clips.end(), aFrame,
                                   [](uint32_t f, const ClipRecord& c) { return f < c.firstFrame; });
        return static_cast<int32_t>(std::distance(m_clips.begin(), it)) - 1;
    }

    uint32_t FrameFlagsOf(uint32_t aFrame) const { return aFrame < m_flags.size() ? m_flags[aFrame] : 0u; }

    // Normalized + weighted features of a frame (NumFeatures() floats) or empty span if out of range.
    std::span<const float> Features(uint32_t aFrame) const
    {
        if (aFrame >= m_header.numFrames)
            return {};
        return std::span<const float>(m_features).subspan(size_t(aFrame) * m_header.numFeatures, m_header.numFeatures);
    }

    // Game-local transforms of bones 0..NumBones()-1: per bone px,py,pz,qx,qy,qz,qw.
    std::span<const float> Pose(uint32_t aFrame) const
    {
        if (aFrame >= m_header.numFrames)
            return {};
        const size_t n = size_t(m_header.numBones) * 7;
        return std::span<const float>(m_pose).subspan(size_t(aFrame) * n, n);
    }

    // Per bone sx,sy,sz; empty when the file stores no scale (all scales are 1).
    std::span<const float> Scale(uint32_t aFrame) const
    {
        if (!HasScale() || aFrame >= m_header.numFrames)
            return {};
        const size_t n = size_t(m_header.numBones) * 3;
        return std::span<const float>(m_scaleData).subspan(size_t(aFrame) * n, n);
    }

    const RootDelta* Root(uint32_t aFrame) const { return aFrame < m_root.size() ? &m_root[aFrame] : nullptr; }

    // contact_l, contact_r
    std::array<float, 2> Contacts(uint32_t aFrame) const
    {
        if (aFrame >= m_header.numFrames)
            return {0.0f, 0.0f};
        return {m_contacts[size_t(aFrame) * 2], m_contacts[size_t(aFrame) * 2 + 1]};
    }

    // Raw (character-frame) query -> normalized query: (raw - offset) / scale. Sizes must be NumFeatures().
    bool NormalizeQuery(std::span<const float> aRaw, std::span<float> aOut) const
    {
        const uint32_t nf = m_header.numFeatures;
        if (!m_loaded || aRaw.size() != nf || aOut.size() != nf)
            return false;
        for (uint32_t j = 0; j < nf; ++j)
            aOut[j] = (aRaw[j] - m_offset[j]) / m_scale[j];
        return true;
    }

    void DenormalizeFeatures(std::span<const float> aNorm, std::span<float> aOut) const
    {
        const uint32_t nf = m_header.numFeatures;
        if (!m_loaded || aNorm.size() != nf || aOut.size() != nf)
            return;
        for (uint32_t j = 0; j < nf; ++j)
            aOut[j] = aNorm[j] * m_scale[j] + m_offset[j];
    }

    // AABB-accelerated search over searchable frames (orangeduck motion_matching_search semantics):
    //  - aCurrentIndex >= 0: best cost starts as the (transition-cost-free) cost of the current frame and frames
    //    with |i - aCurrentIndex| < aIgnoreSurrounding inside the current clip are skipped;
    //  - every candidate pays aTransitionCost.
    // Returns index -1 when the query has the wrong size, the db is not loaded, or nothing beats the start cost.
    SearchResult Search(std::span<const float> aQuery, int32_t aCurrentIndex = -1, int32_t aIgnoreSurrounding = 0,
                        float aTransitionCost = 0.0f) const
    {
        return SearchInRanges(aQuery, m_ranges, aCurrentIndex, aIgnoreSurrounding, aTransitionCost);
    }

    // Same as Search() but only visits aRanges (e.g. a tag-filtered subset of SearchRanges()). Precondition: every
    // range is a sub-range of one SearchRanges() entry (only searchable frames), otherwise the AABB pruning is not
    // exact for the extra frames. Ranges with start >= stop or stop > NumFrames() are skipped.
    SearchResult SearchInRanges(std::span<const float> aQuery, std::span<const Range> aRanges, int32_t aCurrentIndex = -1,
                                int32_t aIgnoreSurrounding = 0, float aTransitionCost = 0.0f) const
    {
        SearchResult best;
        const uint32_t nf = m_header.numFeatures;
        if (!m_loaded || aQuery.size() != nf)
            return best;
        const float* q = aQuery.data();
        const int32_t curr = InitBest(q, aCurrentIndex, best);
        uint32_t clipStart = 0, clipEnd = 0;
        CurrentClipBounds(curr, clipStart, clipEnd);
        const uint32_t sm = m_header.boundSmSize;
        const uint32_t lr = m_header.boundLrSize;

        for (const Range& r : aRanges)
        {
            if (r.start >= r.stop || r.stop > m_header.numFrames)
                continue;
            uint32_t i = r.start;
            const uint32_t end = r.stop;
            while (i < end)
            {
                const uint32_t iLr = i / lr;
                const uint32_t iLrNext = (iLr + 1) * lr;
                if (BoxCost(q, &m_lrMin[size_t(iLr) * nf], &m_lrMax[size_t(iLr) * nf], aTransitionCost, best.cost) >= best.cost)
                {
                    i = iLrNext;
                    continue;
                }
                while (i < iLrNext && i < end)
                {
                    const uint32_t iSm = i / sm;
                    const uint32_t iSmNext = (iSm + 1) * sm;
                    if (BoxCost(q, &m_smMin[size_t(iSm) * nf], &m_smMax[size_t(iSm) * nf], aTransitionCost, best.cost) >=
                        best.cost)
                    {
                        i = iSmNext;
                        continue;
                    }
                    while (i < iSmNext && i < end)
                    {
                        if (!(curr != -1 && Skip(i, curr, aIgnoreSurrounding, clipStart, clipEnd)))
                            TestFrame(q, i, aTransitionCost, best);
                        ++i;
                    }
                }
            }
        }
        return best;
    }

    // Reference linear scan with identical semantics and identical floating-point accumulation order.
    SearchResult SearchBruteForce(std::span<const float> aQuery, int32_t aCurrentIndex = -1, int32_t aIgnoreSurrounding = 0,
                                  float aTransitionCost = 0.0f) const
    {
        return SearchBruteForceInRanges(aQuery, m_ranges, aCurrentIndex, aIgnoreSurrounding, aTransitionCost);
    }

    SearchResult SearchBruteForceInRanges(std::span<const float> aQuery, std::span<const Range> aRanges,
                                          int32_t aCurrentIndex = -1, int32_t aIgnoreSurrounding = 0,
                                          float aTransitionCost = 0.0f) const
    {
        SearchResult best;
        if (!m_loaded || aQuery.size() != m_header.numFeatures)
            return best;
        const float* q = aQuery.data();
        const int32_t curr = InitBest(q, aCurrentIndex, best);
        uint32_t clipStart = 0, clipEnd = 0;
        CurrentClipBounds(curr, clipStart, clipEnd);
        for (const Range& r : aRanges)
            for (uint32_t i = r.start; i < r.stop && i < m_header.numFrames; ++i)
                if (!(curr != -1 && Skip(i, curr, aIgnoreSurrounding, clipStart, clipEnd)))
                    TestFrame(q, i, aTransitionCost, best);
        return best;
    }

    static bool ReadFile(const char* aPath, std::vector<uint8_t>& aOut, std::string& aError)
    {
        aOut.clear();
        std::FILE* f = nullptr;
#if defined(_MSC_VER)
        if (fopen_s(&f, aPath, "rb") != 0)
            f = nullptr;
#else
        f = std::fopen(aPath, "rb");
#endif
        if (!f)
        {
            aError = "cannot open file";
            return false;
        }
        bool ok = std::fseek(f, 0, SEEK_END) == 0;
        const long size = ok ? std::ftell(f) : -1;
        ok = ok && size >= 0 && size <= (1L << 30) && std::fseek(f, 0, SEEK_SET) == 0;
        if (ok)
        {
            aOut.resize(static_cast<size_t>(size));
            ok = aOut.empty() || std::fread(aOut.data(), 1, aOut.size(), f) == aOut.size();
        }
        std::fclose(f);
        if (!ok)
        {
            aOut.clear();
            aError = "read error";
        }
        return ok;
    }

private:
    static bool Fail(std::string& aError, const char* aMsg)
    {
        aError = aMsg;
        return false;
    }

    std::string_view Str(uint32_t aOffset) const
    {
        if (aOffset >= m_strings.size())
            return {};
        const char* s = m_strings.data() + aOffset;
        return std::string_view(s, std::strlen(s)); // NUL termination validated at load
    }

    // Frames near the current one are skipped only inside the current clip (aClipStart..aClipEnd): neighbouring
    // indices of another clip are unrelated motion and must stay eligible.
    static bool Skip(uint32_t aI, int32_t aCurr, int32_t aIgnore, uint32_t aClipStart, uint32_t aClipEnd)
    {
        if (aI < aClipStart || aI >= aClipEnd)
            return false;
        const int64_t d = static_cast<int64_t>(aI) - aCurr;
        return (d < 0 ? -d : d) < aIgnore;
    }

    void CurrentClipBounds(int32_t aCurr, uint32_t& aStart, uint32_t& aEnd) const
    {
        aStart = aEnd = 0;
        if (aCurr < 0)
            return;
        const int32_t c = ClipOfFrame(static_cast<uint32_t>(aCurr));
        if (c < 0)
            return;
        aStart = m_clips[size_t(c)].firstFrame;
        aEnd = aStart + m_clips[size_t(c)].numFrames;
    }

    int32_t InitBest(const float* aQ, int32_t aCurrentIndex, SearchResult& aBest) const
    {
        if (aCurrentIndex < 0 || static_cast<uint32_t>(aCurrentIndex) >= m_header.numFrames)
            return -1;
        const float* f = &m_features[size_t(aCurrentIndex) * m_header.numFeatures];
        float c = 0.0f;
        for (uint32_t j = 0; j < m_header.numFeatures; ++j)
        {
            const float d = aQ[j] - f[j];
            c += d * d;
        }
        aBest.index = aCurrentIndex;
        aBest.cost = c;
        return aCurrentIndex;
    }

    float BoxCost(const float* aQ, const float* aMin, const float* aMax, float aStart, float aBest) const
    {
        float c = aStart;
        for (uint32_t j = 0; j < m_header.numFeatures; ++j)
        {
            const float v = std::min(std::max(aQ[j], aMin[j]), aMax[j]);
            const float d = aQ[j] - v;
            c += d * d;
            if (c >= aBest)
                break;
        }
        return c;
    }

    void TestFrame(const float* aQ, uint32_t aI, float aStart, SearchResult& aBest) const
    {
        const float* f = &m_features[size_t(aI) * m_header.numFeatures];
        float c = aStart;
        for (uint32_t j = 0; j < m_header.numFeatures; ++j)
        {
            const float d = aQ[j] - f[j];
            c += d * d;
            if (c >= aBest.cost)
                return;
        }
        aBest.index = static_cast<int32_t>(aI);
        aBest.cost = c;
    }

    template <class T>
    static bool CopySection(std::span<const uint8_t> aBytes, const SectionEntry& aE, std::vector<T>& aOut)
    {
        static_assert(std::is_trivially_copyable_v<T>);
        if (aE.size % sizeof(T) != 0)
            return false;
        aOut.resize(static_cast<size_t>(aE.size / sizeof(T)));
        if (aE.size)
            std::memcpy(aOut.data(), aBytes.data() + aE.offset, static_cast<size_t>(aE.size));
        return true;
    }

    static bool AllFinite(const std::vector<float>& aV)
    {
        return std::all_of(aV.begin(), aV.end(), [](float x) { return std::isfinite(x); });
    }

    bool Parse(std::span<const uint8_t> aBytes, uint64_t aExpectedRigHash, std::string& aError)
    {
        if (aBytes.size() < kHeaderSize)
            return Fail(aError, "file too small");
        Header h{};
        std::memcpy(&h, aBytes.data(), kHeaderSize);
        if (std::memcmp(h.magic, kMagic, 8) != 0)
            return Fail(aError, "bad magic");
        if (h.version != kVersion)
            return Fail(aError, "unsupported version");
        if (h.headerSize != kHeaderSize)
            return Fail(aError, "bad header size");
        {
            Header z = h;
            z.headerCrc32 = 0;
            if (Crc32(reinterpret_cast<const uint8_t*>(&z), kHeaderSize) != h.headerCrc32)
                return Fail(aError, "header CRC mismatch");
        }
        if (h.fileSize != aBytes.size())
            return Fail(aError, "file size mismatch (truncated?)");
        if (Crc32(aBytes.data() + kHeaderSize, aBytes.size() - kHeaderSize) != h.payloadCrc32)
            return Fail(aError, "payload CRC mismatch");
        if (h.rigHash != aExpectedRigHash)
            return Fail(aError, "rig hash mismatch");
        if (h.numFrames == 0 || h.numBones == 0 || h.numBones > kMaxBones || h.numFeatures == 0 ||
            h.numFeatures > kMaxFeatures || h.numClips == 0 || h.numRanges == 0 || h.numGroups == 0 || h.numTags > 64)
            return Fail(aError, "bad counts");
        if (h.boundSmSize == 0 || h.boundLrSize == 0 || h.boundLrSize % h.boundSmSize != 0 ||
            h.numBoundSm != (h.numFrames + h.boundSmSize - 1) / h.boundSmSize ||
            h.numBoundLr != (h.numFrames + h.boundLrSize - 1) / h.boundLrSize)
            return Fail(aError, "bad AABB layout");
        if (!(std::isfinite(h.fps) && h.fps > 0.0f))
            return Fail(aError, "bad fps");
        if (h.numSections == 0 || h.numSections > kMaxSections || h.sectionTableOffset != kHeaderSize ||
            uint64_t(h.sectionTableOffset) + uint64_t(h.numSections) * kSectionEntrySize > aBytes.size())
            return Fail(aError, "bad section table");

        // ---- section table
        std::vector<SectionEntry> entries(h.numSections);
        std::memcpy(entries.data(), aBytes.data() + h.sectionTableOffset, size_t(h.numSections) * kSectionEntrySize);
        const uint64_t dataStart = uint64_t(h.sectionTableOffset) + uint64_t(h.numSections) * kSectionEntrySize;
        for (const SectionEntry& e : entries)
        {
            if (e.offset % 16 != 0 || e.offset < dataStart || e.offset > aBytes.size() || e.size > aBytes.size() - e.offset ||
                e.elemSize == 0 || uint64_t(e.count) * e.elemSize != e.size)
                return Fail(aError, "section out of bounds / inconsistent");
        }
        {
            std::vector<SectionEntry> sorted = entries;
            std::sort(sorted.begin(), sorted.end(), [](const SectionEntry& a, const SectionEntry& b) { return a.offset < b.offset; });
            for (size_t i = 1; i < sorted.size(); ++i)
                if (sorted[i - 1].offset + sorted[i - 1].size > sorted[i].offset)
                    return Fail(aError, "overlapping sections");
            for (size_t i = 0; i < entries.size(); ++i)
                for (size_t k = i + 1; k < entries.size(); ++k)
                    if (entries[i].id == entries[k].id)
                        return Fail(aError, "duplicate section id");
        }
        auto find = [&](const char (&aId)[5]) -> const SectionEntry* {
            for (const SectionEntry& e : entries)
                if (e.id == FourCC(aId))
                    return &e;
            return nullptr;
        };
        const uint32_t nf = h.numFeatures;
        const uint32_t nb = h.numBones;
        auto need = [&](const char (&aId)[5], uint64_t aCount, uint64_t aElem) -> const SectionEntry* {
            const SectionEntry* e = find(aId);
            if (!e || e->count != aCount || e->elemSize != aElem)
                return nullptr;
            return e;
        };
        const SectionEntry* sStr = find("STRS");
        const SectionEntry* sBone = need("BONE", nb, 16);
        const SectionEntry* sTags = need("TAGS", h.numTags, 4);
        const SectionEntry* sClip = need("CLIP", h.numClips, 32);
        const SectionEntry* sGrp = need("GRUP", h.numGroups, 48);
        const SectionEntry* sOff = need("FOFF", nf, 4);
        const SectionEntry* sScl = need("FSCL", nf, 4);
        const SectionEntry* sFeat = need("FEAT", h.numFrames, uint64_t(nf) * 4);
        const SectionEntry* sFlg = need("FFLG", h.numFrames, 4);
        const SectionEntry* sRng = need("RNGS", h.numRanges, 8);
        const SectionEntry* sSmN = need("BSMN", h.numBoundSm, uint64_t(nf) * 4);
        const SectionEntry* sSmX = need("BSMX", h.numBoundSm, uint64_t(nf) * 4);
        const SectionEntry* sLrN = need("BLRN", h.numBoundLr, uint64_t(nf) * 4);
        const SectionEntry* sLrX = need("BLRX", h.numBoundLr, uint64_t(nf) * 4);
        const SectionEntry* sPose = need("POSE", h.numFrames, uint64_t(nb) * 28);
        const SectionEntry* sRoot = need("ROOT", h.numFrames, 16);
        const SectionEntry* sCont = need("CONT", h.numFrames, 8);
        const bool hasScale = (h.flags & kHeaderHasScale) != 0;
        const SectionEntry* sScal = hasScale ? need("SCAL", h.numFrames, uint64_t(nb) * 12) : nullptr;
        if (!sStr || sStr->elemSize != 1 || sStr->size == 0 || !sBone || !sTags || !sClip || !sGrp || !sOff || !sScl ||
            !sFeat || !sFlg || !sRng || !sSmN || !sSmX || !sLrN || !sLrX || !sPose || !sRoot || !sCont ||
            (hasScale && !sScal) || (h.flags & ~uint32_t(kHeaderHasScale)) != 0)
            return Fail(aError, "missing or malformed section");

        Database d;
        d.m_header = h;
        if (!CopySection(aBytes, *sStr, d.m_strings) || !CopySection(aBytes, *sBone, d.m_bones) ||
            !CopySection(aBytes, *sTags, d.m_tags) || !CopySection(aBytes, *sClip, d.m_clips) ||
            !CopySection(aBytes, *sGrp, d.m_groups) || !CopySection(aBytes, *sOff, d.m_offset) ||
            !CopySection(aBytes, *sScl, d.m_scale) || !CopySection(aBytes, *sFeat, d.m_features) ||
            !CopySection(aBytes, *sFlg, d.m_flags) || !CopySection(aBytes, *sRng, d.m_ranges) ||
            !CopySection(aBytes, *sSmN, d.m_smMin) || !CopySection(aBytes, *sSmX, d.m_smMax) ||
            !CopySection(aBytes, *sLrN, d.m_lrMin) || !CopySection(aBytes, *sLrX, d.m_lrMax) ||
            !CopySection(aBytes, *sPose, d.m_pose) || !CopySection(aBytes, *sRoot, d.m_root) ||
            !CopySection(aBytes, *sCont, d.m_contacts) || (hasScale && !CopySection(aBytes, *sScal, d.m_scaleData)))
            return Fail(aError, "section copy failed");

        // ---- strings: last byte must be NUL so every offset yields a terminated string
        if (d.m_strings.back() != '\0')
            return Fail(aError, "string table not terminated");
        auto validStr = [&](uint32_t aOff) { return aOff < d.m_strings.size(); };

        // ---- skeleton + rig hash
        std::vector<std::string_view> names(nb);
        std::vector<int32_t> parents(nb);
        for (uint32_t b = 0; b < nb; ++b)
        {
            const BoneRecord& r = d.m_bones[b];
            if (!validStr(r.nameOffset) || d.Str(r.nameOffset).size() != r.nameLength)
                return Fail(aError, "bad bone name");
            if ((b == 0) != (r.parent < 0) || r.parent >= static_cast<int32_t>(b))
                return Fail(aError, "bad bone parent");
            names[b] = d.Str(r.nameOffset);
            parents[b] = r.parent;
        }
        if (ComputeRigHash(names, parents) != h.rigHash)
            return Fail(aError, "stored skeleton does not match rig hash");

        for (uint32_t t : d.m_tags)
            if (!validStr(t))
                return Fail(aError, "bad tag name");

        // ---- clips: contiguous cover of [0, numFrames)
        uint64_t next = 0;
        for (const ClipRecord& c : d.m_clips)
        {
            if (!validStr(c.nameOffset) || c.firstFrame != next || c.numFrames == 0 || c.numSearchable > c.numFrames ||
                (h.numTags < 64 && (c.tagMask >> h.numTags) != 0) || c.loopPeriod > c.numFrames ||
                ((c.flags & kClipLoop) != 0) != (c.loopPeriod != 0))
                return Fail(aError, "bad clip record");
            next += c.numFrames;
        }
        if (next != h.numFrames)
            return Fail(aError, "clips do not cover all frames");

        // ---- groups: contiguous cover of [0, numFeatures)
        uint32_t fnext = 0;
        for (const GroupRecord& g : d.m_groups)
        {
            const bool traj = g.kind == uint32_t(GroupKind::TrajectoryPosition) || g.kind == uint32_t(GroupKind::TrajectoryDirection);
            const bool bone = g.kind == uint32_t(GroupKind::BonePosition) || g.kind == uint32_t(GroupKind::BoneVelocity);
            if (!validStr(g.nameOffset) || (!traj && !bone) || g.firstFeature != fnext || g.numFeatures == 0 ||
                g.numFeatures > nf - fnext || (bone && (g.bone < 0 || g.bone >= static_cast<int32_t>(nb))) ||
                (traj && (g.numHorizons == 0 || g.numHorizons > kMaxHorizons || g.numFeatures != 2 * g.numHorizons)) ||
                (!traj && g.numHorizons != 0) || !(std::isfinite(g.weight) && g.weight > 0.0f) ||
                !(std::isfinite(g.std) && g.std > 0.0f))
                return Fail(aError, "bad feature group");
            fnext += g.numFeatures;
        }
        if (fnext != nf)
            return Fail(aError, "groups do not cover all features");

        // ---- numbers
        if (!AllFinite(d.m_offset) || !AllFinite(d.m_scale) || !AllFinite(d.m_features) || !AllFinite(d.m_smMin) ||
            !AllFinite(d.m_smMax) || !AllFinite(d.m_lrMin) || !AllFinite(d.m_lrMax) || !AllFinite(d.m_pose) ||
            !AllFinite(d.m_contacts) || !AllFinite(d.m_scaleData))
            return Fail(aError, "non-finite value");
        for (float s : d.m_scale)
            if (!(s > 0.0f))
                return Fail(aError, "non-positive feature scale");
        for (const RootDelta& r : d.m_root)
            if (!std::isfinite(r.dx) || !std::isfinite(r.dy) || !std::isfinite(r.dyaw))
                return Fail(aError, "non-finite root delta");
        for (size_t i = 0; i < d.m_pose.size(); i += 7)
        {
            const float* qq = &d.m_pose[i + 3];
            const float len2 = qq[0] * qq[0] + qq[1] * qq[1] + qq[2] * qq[2] + qq[3] * qq[3];
            if (std::fabs(len2 - 1.0f) > 1e-3f)
                return Fail(aError, "non-unit quaternion");
        }

        // ---- search ranges: sorted, disjoint, inside one clip each, exactly the searchable frames
        uint64_t prevStop = 0;
        uint64_t inRanges = 0;
        for (const Range& r : d.m_ranges)
        {
            if (r.start >= r.stop || r.stop > h.numFrames || r.start < prevStop)
                return Fail(aError, "bad search range");
            const int32_t c = d.ClipOfFrame(r.start);
            if (c < 0 || r.stop > d.m_clips[size_t(c)].firstFrame + d.m_clips[size_t(c)].numFrames)
                return Fail(aError, "search range crosses clip boundary");
            for (uint32_t i = r.start; i < r.stop; ++i)
                if ((d.m_flags[i] & kFrameSearchable) == 0)
                    return Fail(aError, "unsearchable frame inside search range");
            inRanges += r.stop - r.start;
            prevStop = r.stop;
        }
        uint64_t searchable = 0;
        for (uint32_t f : d.m_flags)
            searchable += (f & kFrameSearchable) ? 1u : 0u;
        if (searchable != inRanges)
            return Fail(aError, "searchable flags disagree with search ranges");

        // ---- AABBs must contain every searchable frame of their span (guarantees exact search)
        auto checkBoxes = [&](const std::vector<float>& aMin, const std::vector<float>& aMax, uint32_t aSize) {
            for (uint32_t i = 0; i < h.numFrames; ++i)
            {
                if ((d.m_flags[i] & kFrameSearchable) == 0)
                    continue;
                const size_t b = size_t(i / aSize) * nf;
                const float* f = &d.m_features[size_t(i) * nf];
                for (uint32_t j = 0; j < nf; ++j)
                    if (f[j] < aMin[b + j] || f[j] > aMax[b + j])
                        return false;
            }
            return true;
        };
        if (!checkBoxes(d.m_smMin, d.m_smMax, h.boundSmSize) || !checkBoxes(d.m_lrMin, d.m_lrMax, h.boundLrSize))
            return Fail(aError, "AABB does not bound its frames");

        d.m_loaded = true;
        *this = std::move(d);
        aError.clear();
        return true;
    }

    Header m_header{};
    bool m_loaded = false;
    std::vector<char> m_strings;
    std::vector<BoneRecord> m_bones;
    std::vector<uint32_t> m_tags;
    std::vector<ClipRecord> m_clips;
    std::vector<GroupRecord> m_groups;
    std::vector<float> m_offset;
    std::vector<float> m_scale;
    std::vector<float> m_features;
    std::vector<uint32_t> m_flags;
    std::vector<Range> m_ranges;
    std::vector<float> m_smMin, m_smMax, m_lrMin, m_lrMax;
    std::vector<float> m_pose;
    std::vector<float> m_scaleData;
    std::vector<RootDelta> m_root;
    std::vector<float> m_contacts;
};

} // namespace utpp::mm
