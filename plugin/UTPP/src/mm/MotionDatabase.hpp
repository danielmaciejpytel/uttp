// UTPP Motion Matching database (.utppmm VERSION 2) reader + reference search. Header-only, C++20.
//
// Format spec: Source/tools/mmbake/FORMAT.txt, part "VERSION 2" (producer mmbake_motion.py). Fails closed: any bad
// magic / version / size / CRC / rig identity / section layout / cross-section range / non-finite value leaves the
// database empty (Loaded() == false) and returns false with a reason string. Databases whose schema reads the root
// vertical (SCHC flag Z_FROM_RTZ) but contain a clip without a root_z track (CLIP flag RTZ_UNKNOWN) are refused
// individually (DatabaseUsable() == false), as FORMAT.txt requires.
// All section data is copied into typed vectors (memcpy): no unaligned / aliasing access to the file bytes.
// Shares CRC-32 / FNV-1a / v1 topology hash with the v1 reader (BaseDatabase.hpp). Not used by the runtime yet.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "../BaseDatabase.hpp"

namespace utpp::mm::motion
{
inline constexpr uint32_t kVersion = 2;
inline constexpr uint32_t kMaxDatabases = 64;
inline constexpr uint32_t kMaxCurves = 32;
inline constexpr uint32_t kMaxDims = 1024;

// Rig identities of the two player rigs, bones 0..72 (FORMAT.txt v2 "Rig identity"). The v1 topology hash
// (0x72CB314EB01F562D) is the same for both rigs.
inline constexpr uint64_t kRigIdentityWoman = 0xF62810A88A8BFDA9ull;
inline constexpr uint64_t kRigIdentityMan = 0xAFE2CFEDCF97CDF7ull;

enum MotionClipFlags : uint32_t
{
    kClipLoop = 1u << 0,
    kClipEndsPoseMatch = 1u << 1,
    kClipNameLoop = 1u << 2,
    kClipRtzUnknown = 1u << 3,
    kClipAdded = 1u << 4,
    kClipRtzPresent = 1u << 5,
    kClipRtzNone = 1u << 6,
};

enum MotionFrameFlags : uint32_t
{
    kFrameSelectable = 1u << 0,
    kFrameExcludedNotify = 1u << 1,
    kFrameLoopDuplicate = 1u << 3,
    kFrameBlockTransition = 1u << 4,
    kFrameBranchIn = 1u << 5,
    kFrameClipStart = 1u << 6,
    kFrameClipEnd = 1u << 7,
    kFrameExcludedTail = 1u << 8,
    kFrameIndexed = 1u << 9,
};

enum ChannelFlags : uint32_t
{
    kChanTrajectory = 1u << 0,
    kChanNormalizeOutput = 1u << 1,
    kChanClampUnitUe = 1u << 2,
    kChanZFromRtz = 1u << 3,
    kChanUnitless = 1u << 4,
};

enum DatabaseFlags : uint32_t
{
    kDbRtzPending = 1u << 0,
    kDbInNormalizationSet = 1u << 1,
};

enum EntryFlags : uint32_t
{
    kEntryEnabled = 1u << 0,
    kEntryDisableReselection = 1u << 1,
    kEntryLooping = 1u << 2,
    kEntryAdded = 1u << 3,
    kEntryNotifyMotion58 = 1u << 4,
};

enum RowFlags : uint32_t
{
    kRowBlock = 1u << 0,
    kRowBranchIn = 1u << 1,
    kRowLooping = 1u << 2,
    kRowRangeStart = 1u << 3,
    kRowRangeEnd = 1u << 4,
};

enum class RtzSource : uint32_t
{
    None = 0,
    Unknown = 1,
    RootZ = 2,
};

#pragma pack(push, 4)
struct MotionHeader
{
    char magic[8];
    uint32_t version;
    uint32_t headerSize;
    uint64_t fileSize;
    uint64_t rigHash;
    uint32_t flags;
    uint32_t numFrames;
    uint32_t numBones;
    uint32_t numClips;
    uint32_t numSchemas;
    uint32_t numChannels;
    uint32_t numNorms;
    uint32_t numDatabases;
    uint32_t numEntries;
    uint32_t numRows;
    uint32_t numCurves;
    uint32_t numEvents;
    float fps;
    uint32_t numSections;
    uint32_t sectionTableOffset;
    uint32_t payloadCrc32;
    uint32_t headerCrc32;
    uint32_t boundSmSize;
    uint32_t boundLrSize;
    uint32_t maxDims;
    uint64_t dfeaFloats;
    uint64_t dboxFloats;
};
#pragma pack(pop)
static_assert(sizeof(MotionHeader) == 128);
static_assert(offsetof(MotionHeader, rigHash) == 24 && offsetof(MotionHeader, numFrames) == 36 && offsetof(MotionHeader, numEvents) == 76);
static_assert(offsetof(MotionHeader, fps) == 80 && offsetof(MotionHeader, headerCrc32) == 96 && offsetof(MotionHeader, maxDims) == 108);
static_assert(offsetof(MotionHeader, dfeaFloats) == 112 && offsetof(MotionHeader, dboxFloats) == 120);

struct RigRestRecord
{
    float px, py, pz, qx, qy, qz, qw;
};
static_assert(sizeof(RigRestRecord) == 28);

struct MotionClipRecord
{
    uint32_t nameOff;
    uint32_t firstFrame;
    uint32_t numFrames;
    uint32_t flags;
    uint32_t loopPeriod;
    uint32_t numSelectable;
    uint64_t dbMask;
};
static_assert(sizeof(MotionClipRecord) == 32);

struct Clip2Record
{
    uint32_t firstEvent;
    uint32_t numEvents;
    uint32_t rtzSource;
    uint32_t curveMask;
    float hipRatio;
    float lengthS;
    uint32_t addedAlt;
    uint32_t recomputedCurveMask;
};
static_assert(sizeof(Clip2Record) == 32);

struct SchemaRecord
{
    uint32_t nameOff;
    uint32_t dims;
    uint32_t firstChannel;
    uint32_t numChannels;
    float sampleRate;
    uint32_t preprocessor;
    float weightSum;
    uint32_t zero;
};
static_assert(sizeof(SchemaRecord) == 32);

struct ChannelRecord
{
    uint32_t kind;       // 1 Position, 2 Velocity, 3 VelocityDirection, 4 Heading
    uint32_t components; // 7 xyz, 3 xy, 4 z
    int32_t bone;
    int32_t originBone;
    float tSample;
    float tOrigin;
    uint32_t velocitySpace; // 0 none, 1 character, 2 world
    uint32_t inputQuery;    // 0 continuing pose, 1 character
    float weight;
    float weightNorm;
    uint32_t dataOffset;
    uint32_t cardinality;
    uint32_t normGroupOff;
    uint32_t labelOff;
    float axis[3];
    uint32_t flags;
    uint32_t zero;
};
static_assert(sizeof(ChannelRecord) == 76);

struct NormRecord
{
    uint32_t schema;
    uint32_t dims;
    uint32_t nrmvOffset;
    uint32_t numMembers;
    uint64_t memberMask;
    uint32_t rows;
    uint32_t zero;
};
static_assert(sizeof(NormRecord) == 32);

struct DatabaseRecord
{
    uint32_t nameOff;
    uint32_t schema;
    uint32_t norm;
    uint32_t family;
    uint32_t tagBits;
    uint32_t flags;
    float baseCostBias;
    float continuingPoseCostBias;
    float loopingCostBias;
    float excludeMin;
    float excludeMax;
    uint32_t firstEntry;
    uint32_t numEntries;
    uint32_t firstRow;
    uint32_t numRows;
    uint32_t dims;
    uint64_t featOffset;
    uint64_t boxOffset;
    uint32_t numSmallBoxes;
    uint32_t numLargeBoxes;
    float minAddend;
    uint32_t numSelectable;
};
static_assert(sizeof(DatabaseRecord) == 96 && offsetof(DatabaseRecord, firstEntry) == 44 &&
              offsetof(DatabaseRecord, featOffset) == 64 && offsetof(DatabaseRecord, numSelectable) == 92);

struct EntryRecord
{
    uint32_t database;
    uint32_t clip;
    uint32_t flags;
    float rangeMin;
    float rangeMax;
    uint32_t firstRow;
    uint32_t numRows;
    uint32_t zero;
};
static_assert(sizeof(EntryRecord) == 32);

struct RowRecord
{
    uint32_t frame;
    float costAddend;
    float continuingAddend;
    uint32_t entry;
    uint32_t flags;
};
static_assert(sizeof(RowRecord) == 20);

struct CurveNameRecord
{
    uint32_t nameOff;
    uint32_t unit; // 0 alpha, 1 m/s, 2 m, 3 phase
};
static_assert(sizeof(CurveNameRecord) == 8);

struct EventRecord
{
    uint32_t clip;
    float timeS;
    float durationS;
    uint32_t tagOff;
    uint32_t classOff;
    uint32_t flags;
};
static_assert(sizeof(EventRecord) == 24);

struct PoseRecord
{
    float px, py, pz, qx, qy, qz, qw; // game-local transform (relative to the game parent)
};
static_assert(sizeof(PoseRecord) == 28);

struct RtzRecord
{
    float rootZ;
    float vz;
};
static_assert(sizeof(RtzRecord) == 8);

struct ContactRecord
{
    float l;
    float r;
};
static_assert(sizeof(ContactRecord) == 8);

struct MotionSearchResult
{
    int32_t localRow = -1; // -1 = none
    float cost = std::numeric_limits<float>::max();
    uint32_t rowsEvaluated = 0;
};

// Rig identity hash (FORMAT.txt v2): FNV-1a 64 over, per bone: name bytes, 0x00, int32 parent, then int32
// floor(double(t) * 1000 + 0.5) of the float32 rest local translation x, y, z (metres).
inline uint64_t ComputeRigIdentity(std::span<const std::string_view> aNames, std::span<const int32_t> aParents,
                                   std::span<const float> aRestXyz)
{
    uint64_t h = 0xCBF29CE484222325ull;
    const size_t n = std::min({aNames.size(), aParents.size(), aRestXyz.size() / 3});
    for (size_t i = 0; i < n; ++i)
    {
        h = Fnv1a64(reinterpret_cast<const uint8_t*>(aNames[i].data()), aNames[i].size(), h);
        const uint8_t zero = 0;
        h = Fnv1a64(&zero, 1, h);
        uint8_t p[4];
        std::memcpy(p, &aParents[i], 4);
        h = Fnv1a64(p, 4, h);
        for (int k = 0; k < 3; ++k)
        {
            const int32_t q = static_cast<int32_t>(std::floor(static_cast<double>(aRestXyz[i * 3 + k]) * 1000.0 + 0.5));
            std::memcpy(p, &q, 4);
            h = Fnv1a64(p, 4, h);
        }
    }
    return h;
}

class MotionDatabase
{
public:
    // aExpectedRigIdentity: identity hash of the live rig (or kRigIdentityWoman / kRigIdentityMan).
    bool LoadFromMemory(std::span<const uint8_t> aBytes, uint64_t aExpectedRigIdentity, std::string& aError)
    {
        Clear();
        const bool ok = Parse(aBytes, aExpectedRigIdentity, aError);
        if (!ok)
            Clear();
        m_loaded = ok;
        return ok;
    }

    bool LoadFromFile(const char* aPath, uint64_t aExpectedRigIdentity, std::string& aError)
    {
        std::vector<uint8_t> bytes;
        if (!ReadFile(aPath, bytes, aError))
            return false;
        return LoadFromMemory(bytes, aExpectedRigIdentity, aError);
    }

    void Clear()
    {
        *this = MotionDatabase{};
    }

    bool Loaded() const { return m_loaded; }
    const MotionHeader& Header() const { return m_h; }
    uint64_t RigIdentity() const { return m_h.rigHash; }
    uint64_t TopologyHash() const { return m_topologyHash; }
    uint32_t NumFrames() const { return m_h.numFrames; }
    uint32_t NumBones() const { return m_h.numBones; }
    uint32_t NumClips() const { return m_h.numClips; }
    uint32_t NumDatabases() const { return m_h.numDatabases; }
    uint32_t NumRows() const { return m_h.numRows; }
    bool HasScale() const { return (m_h.flags & 1u) != 0; }

    std::string_view String(uint32_t aOff) const
    {
        if (aOff == 0xFFFFFFFFu || aOff >= m_strs.size())
            return {};
        return std::string_view(m_strs.data() + aOff);
    }

    std::string_view BoneName(uint32_t i) const { return String(m_bones[i].nameOffset); }
    int32_t BoneParent(uint32_t i) const { return m_bones[i].parent; }
    const RigRestRecord& RigRest(uint32_t i) const { return m_rigr[i]; }
    const MotionClipRecord& Clip(uint32_t i) const { return m_clips[i]; }
    std::string_view ClipName(uint32_t i) const { return String(m_clips[i].nameOff); }
    uint32_t FrameFlags(uint32_t f) const { return m_fflg[f]; }
    const SchemaRecord& Schema(uint32_t i) const { return m_schemas[i]; }
    const ChannelRecord& Channel(uint32_t i) const { return m_channels[i]; }
    const NormRecord& Norm(uint32_t i) const { return m_norms[i]; }
    const float* NormMean(uint32_t i) const { return m_nrmv.data() + m_norms[i].nrmvOffset; }
    const float* NormScale(uint32_t i) const { return m_nrmv.data() + m_norms[i].nrmvOffset + m_norms[i].dims; }
    const float* NormDeviation(uint32_t i) const { return m_nrmv.data() + m_norms[i].nrmvOffset + 2u * m_norms[i].dims; }
    const DatabaseRecord& Db(uint32_t i) const { return m_dbs[i]; }
    std::string_view DbName(uint32_t i) const { return String(m_dbs[i].nameOff); }
    bool DatabaseUsable(uint32_t i) const { return i < m_dbUsable.size() && m_dbUsable[i] != 0; }
    uint32_t NumRefusedDatabases() const { return m_numRefused; }
    const EntryRecord& Entry(uint32_t i) const { return m_entries[i]; }
    const RowRecord& Row(uint32_t i) const { return m_rows[i]; }
    // Normalized feature row (dims floats) of local row r of database d.
    const float* Features(uint32_t d, uint32_t aLocalRow) const
    {
        return m_dfea.data() + m_dbs[d].featOffset + static_cast<uint64_t>(aLocalRow) * m_dbs[d].dims;
    }
    uint32_t NumCurves() const { return m_h.numCurves; }
    std::string_view CurveName(uint32_t k) const { return String(m_crvn[k].nameOff); }
    uint32_t CurvePresence(uint32_t f) const { return m_curv[static_cast<size_t>(f) * (m_h.numCurves + 1)]; }
    float CurveValue(uint32_t f, uint32_t k) const
    {
        float v;
        std::memcpy(&v, &m_curv[static_cast<size_t>(f) * (m_h.numCurves + 1) + 1 + k], 4);
        return v;
    }
    const EventRecord& Event(uint32_t i) const { return m_events[i]; }
    const RtzRecord& Rtz(uint32_t f) const { return m_rtz[f]; }
    const PoseRecord* Pose(uint32_t f) const { return m_pose.data() + static_cast<size_t>(f) * m_h.numBones; }
    const RootDelta& Root(uint32_t f) const { return m_root[f]; }
    const ContactRecord& Contact(uint32_t f) const { return m_cont[f]; }

    // ---- reference search (FORMAT.txt v2 "Search"): aQuery = normalized query (dims floats) of database d.
    // aExcluded (optional, num_rows of d bytes): non-zero = local row excluded by the runtime (reselection window,
    // DISABLE_RESELECTION of the current entry). BLOCK rows are never candidates. Ties -> lowest local row.
    MotionSearchResult SearchLinear(uint32_t d, const float* aQuery, const uint8_t* aExcluded = nullptr) const
    {
        MotionSearchResult r;
        const DatabaseRecord& db = m_dbs[d];
        for (uint32_t row = 0; row < db.numRows; ++row)
        {
            const RowRecord& rr = m_rows[db.firstRow + row];
            if ((rr.flags & kRowBlock) || (aExcluded && aExcluded[row]))
                continue;
            ++r.rowsEvaluated;
            const float c = RowCost(d, row, aQuery) + rr.costAddend;
            if (c < r.cost)
            {
                r.cost = c;
                r.localRow = static_cast<int32_t>(row);
            }
        }
        return r;
    }

    MotionSearchResult SearchAabb(uint32_t d, const float* aQuery, const uint8_t* aExcluded = nullptr) const
    {
        MotionSearchResult r;
        const DatabaseRecord& db = m_dbs[d];
        const uint32_t dims = db.dims;
        const uint32_t sm = m_h.boundSmSize, lr = m_h.boundLrSize;
        const uint32_t rec = 2 * dims + 1;
        const float* small = m_dbox.data() + db.boxOffset;
        const float* large = small + static_cast<size_t>(db.numSmallBoxes) * rec;
        for (uint32_t bl = 0; bl < db.numLargeBoxes; ++bl)
        {
            const float* B = large + static_cast<size_t>(bl) * rec;
            if (BoxBound(B, dims, aQuery) + B[2 * dims] >= r.cost)
                continue;
            const uint32_t s0 = bl * (lr / sm);
            const uint32_t s1 = std::min(s0 + lr / sm, db.numSmallBoxes);
            for (uint32_t bs = s0; bs < s1; ++bs)
            {
                const float* S = small + static_cast<size_t>(bs) * rec;
                if (BoxBound(S, dims, aQuery) + S[2 * dims] >= r.cost)
                    continue;
                const uint32_t r0 = bs * sm;
                const uint32_t r1 = std::min(r0 + sm, db.numRows);
                for (uint32_t row = r0; row < r1; ++row)
                {
                    const RowRecord& rr = m_rows[db.firstRow + row];
                    if ((rr.flags & kRowBlock) || (aExcluded && aExcluded[row]))
                        continue;
                    ++r.rowsEvaluated;
                    const float c = RowCost(d, row, aQuery) + rr.costAddend;
                    if (c < r.cost)
                    {
                        r.cost = c;
                        r.localRow = static_cast<int32_t>(row);
                    }
                }
            }
        }
        return r;
    }

    // Feature distance of one row (sum over dimensions in ascending order, float32).
    float RowCost(uint32_t d, uint32_t aLocalRow, const float* aQuery) const
    {
        const uint32_t dims = m_dbs[d].dims;
        const float* f = Features(d, aLocalRow);
        float s = 0.0f;
        for (uint32_t k = 0; k < dims; ++k)
        {
            const float t = aQuery[k] - f[k];
            s += t * t;
        }
        return s;
    }

private:
    static float BoxBound(const float* aBox, uint32_t aDims, const float* aQ)
    {
        float s = 0.0f;
        for (uint32_t k = 0; k < aDims; ++k)
        {
            const float lo = aBox[k] - aQ[k];
            const float hi = aQ[k] - aBox[aDims + k];
            const float t = std::max(0.0f, std::max(lo, hi));
            s += t * t;
        }
        return s;
    }

    static bool ReadFile(const char* aPath, std::vector<uint8_t>& aOut, std::string& aError)
    {
        FILE* f = nullptr;
        if (fopen_s(&f, aPath, "rb") != 0 || !f)
        {
            aError = "cannot open file";
            return false;
        }
        _fseeki64(f, 0, SEEK_END);
        const long long size = _ftelli64(f);
        _fseeki64(f, 0, SEEK_SET);
        if (size < 0)
        {
            fclose(f);
            aError = "cannot size file";
            return false;
        }
        aOut.resize(static_cast<size_t>(size));
        const size_t got = aOut.empty() ? 0 : fread(aOut.data(), 1, aOut.size(), f);
        fclose(f);
        if (got != aOut.size())
        {
            aError = "read error";
            return false;
        }
        return true;
    }

    static bool Fail(std::string& aError, const std::string& aMsg)
    {
        aError = aMsg;
        return false;
    }

    struct Sec
    {
        uint32_t id = 0, count = 0, elem = 0, reserved = 0;
        uint64_t offset = 0, size = 0;
    };

    const Sec* Find(uint32_t aId) const
    {
        for (const Sec& s : m_secs)
            if (s.id == aId)
                return &s;
        return nullptr;
    }

    template <class T>
    static void Copy(std::span<const uint8_t> aBytes, const Sec& s, std::vector<T>& aOut, size_t aCount)
    {
        aOut.resize(aCount);
        if (aCount)
            std::memcpy(aOut.data(), aBytes.data() + s.offset, aCount * sizeof(T));
    }

    static bool Finite(const float* p, size_t n)
    {
        for (size_t i = 0; i < n; ++i)
            if (!std::isfinite(p[i]))
                return false;
        return true;
    }

    static std::string Id(uint32_t id)
    {
        std::string s(4, ' ');
        for (int k = 0; k < 4; ++k)
            s[k] = static_cast<char>((id >> (8 * k)) & 0xFF);
        return s;
    }

    bool Parse(std::span<const uint8_t> aBytes, uint64_t aExpected, std::string& aError)
    {
        if (aBytes.size() < kHeaderSize)
            return Fail(aError, "file too small");
        std::memcpy(&m_h, aBytes.data(), sizeof(MotionHeader));
        const MotionHeader& h = m_h;
        if (std::memcmp(h.magic, kMagic, 8) != 0)
            return Fail(aError, "bad magic");
        if (h.version != kVersion)
            return Fail(aError, "unsupported version " + std::to_string(h.version));
        if (h.headerSize != kHeaderSize)
            return Fail(aError, "bad header size");
        {
            MotionHeader z = h;
            z.headerCrc32 = 0;
            if (Crc32(reinterpret_cast<const uint8_t*>(&z), kHeaderSize) != h.headerCrc32)
                return Fail(aError, "header CRC mismatch");
        }
        if (h.fileSize != aBytes.size())
            return Fail(aError, "file size mismatch (truncated?)");
        if (Crc32(aBytes.data() + kHeaderSize, aBytes.size() - kHeaderSize) != h.payloadCrc32)
            return Fail(aError, "payload CRC mismatch");
        if (h.rigHash != aExpected)
            return Fail(aError, "rig identity mismatch (database for another rig)");
        if (h.flags & ~1u)
            return Fail(aError, "unknown header flags");
        if (h.numBones == 0 || h.numBones > kMaxBones || h.numFrames == 0 || h.numClips == 0 || h.numSchemas == 0 ||
            h.numDatabases == 0 || h.numDatabases > kMaxDatabases || h.numCurves > kMaxCurves || h.numSections == 0 ||
            h.numSections > kMaxSections || h.sectionTableOffset != kHeaderSize || !(h.fps > 0.0f) ||
            h.boundSmSize == 0 || h.boundLrSize == 0 || h.boundLrSize % h.boundSmSize != 0 || h.maxDims == 0 ||
            h.maxDims > kMaxDims)
            return Fail(aError, "bad counts / limits in the header");

        // ---- section table
        const uint64_t tableEnd = kHeaderSize + static_cast<uint64_t>(h.numSections) * kSectionEntrySize;
        if (tableEnd > aBytes.size())
            return Fail(aError, "section table out of bounds");
        m_secs.resize(h.numSections);
        for (uint32_t i = 0; i < h.numSections; ++i)
        {
            Sec& s = m_secs[i];
            std::memcpy(&s, aBytes.data() + kHeaderSize + i * kSectionEntrySize, sizeof(Sec));
            if (s.reserved != 0 || s.offset % 16 != 0 || s.offset < tableEnd || s.size != static_cast<uint64_t>(s.count) * s.elem ||
                s.offset + s.size > aBytes.size() || s.offset + s.size < s.offset)
                return Fail(aError, "bad section entry " + Id(s.id));
            for (uint32_t j = 0; j < i; ++j)
            {
                const Sec& o = m_secs[j];
                if (o.id == s.id)
                    return Fail(aError, "duplicate section " + Id(s.id));
                if (s.size && o.size && s.offset < o.offset + o.size && o.offset < s.offset + s.size)
                    return Fail(aError, "overlapping sections " + Id(o.id) + "/" + Id(s.id));
            }
        }
        const uint32_t C = h.numCurves;
        struct Req
        {
            const char* id;
            uint64_t count; // 0 = any (checked below)
            uint32_t elem;
        };
        const Req reqs[] = {
            {"STRS", 0, 1},
            {"BONE", h.numBones, 16},
            {"RIGR", h.numBones, 28},
            {"CLIP", h.numClips, 32},
            {"CLP2", h.numClips, 32},
            {"FFLG", h.numFrames, 4},
            {"SCHM", h.numSchemas, 32},
            {"SCHC", h.numChannels, 76},
            {"NORM", h.numNorms, 32},
            {"NRMV", 0, 4},
            {"DBAS", h.numDatabases, 96},
            {"DENT", h.numEntries, 32},
            {"DROW", h.numRows, 20},
            {"DFEA", h.dfeaFloats, 4},
            {"DBOX", h.dboxFloats, 4},
            {"CRVN", C, 8},
            {"CURV", h.numFrames, 4 * (C + 1)},
            {"EVNT", h.numEvents, 24},
            {"RTZ_", h.numFrames, 8},
            {"POSE", h.numFrames, h.numBones * 28},
            {"ROOT", h.numFrames, 16},
            {"CONT", h.numFrames, 8},
        };
        auto fourcc = [](const char* s) {
            return static_cast<uint32_t>(static_cast<uint8_t>(s[0])) | (static_cast<uint32_t>(static_cast<uint8_t>(s[1])) << 8) |
                   (static_cast<uint32_t>(static_cast<uint8_t>(s[2])) << 16) |
                   (static_cast<uint32_t>(static_cast<uint8_t>(s[3])) << 24);
        };
        const Sec* S[sizeof(reqs) / sizeof(reqs[0])] = {};
        for (size_t i = 0; i < sizeof(reqs) / sizeof(reqs[0]); ++i)
        {
            S[i] = Find(fourcc(reqs[i].id));
            const bool optionalEmpty = (reqs[i].elem != 0 && reqs[i].count == 0 &&
                                        (std::strcmp(reqs[i].id, "CRVN") == 0 || std::strcmp(reqs[i].id, "EVNT") == 0 ||
                                         std::strcmp(reqs[i].id, "NORM") == 0));
            if (!S[i])
            {
                if (optionalEmpty)
                    continue;
                return Fail(aError, std::string("missing section ") + reqs[i].id);
            }
            if (S[i]->elem != reqs[i].elem)
                return Fail(aError, std::string("bad element size of ") + reqs[i].id);
            if (reqs[i].count && S[i]->count != reqs[i].count)
                return Fail(aError, std::string("bad count of ") + reqs[i].id);
            if (!reqs[i].count && std::strcmp(reqs[i].id, "STRS") != 0 && std::strcmp(reqs[i].id, "NRMV") != 0 && S[i]->count)
                return Fail(aError, std::string("non-empty section with zero header count: ") + reqs[i].id);
        }
        const Sec* scal = Find(fourcc("SCAL"));
        if (HasScale() != (scal != nullptr))
            return Fail(aError, "SCAL presence differs from HAS_SCALE");
        if (scal && (scal->elem != h.numBones * 12 || scal->count != h.numFrames))
            return Fail(aError, "bad SCAL section");

        // ---- copy
        const Sec& sStrs = *S[0];
        m_strs.assign(reinterpret_cast<const char*>(aBytes.data() + sStrs.offset),
                      reinterpret_cast<const char*>(aBytes.data() + sStrs.offset + sStrs.size));
        if (m_strs.empty() || m_strs[0] != '\0' || m_strs.back() != '\0' || m_strs.size() % 4 != 0)
            return Fail(aError, "bad string blob");
        Copy(aBytes, *S[1], m_bones, h.numBones);
        Copy(aBytes, *S[2], m_rigr, h.numBones);
        Copy(aBytes, *S[3], m_clips, h.numClips);
        Copy(aBytes, *S[4], m_clip2, h.numClips);
        Copy(aBytes, *S[5], m_fflg, h.numFrames);
        Copy(aBytes, *S[6], m_schemas, h.numSchemas);
        Copy(aBytes, *S[7], m_channels, h.numChannels);
        if (S[8])
            Copy(aBytes, *S[8], m_norms, h.numNorms);
        Copy(aBytes, *S[9], m_nrmv, S[9]->count);
        Copy(aBytes, *S[10], m_dbs, h.numDatabases);
        Copy(aBytes, *S[11], m_entries, h.numEntries);
        Copy(aBytes, *S[12], m_rows, h.numRows);
        Copy(aBytes, *S[13], m_dfea, static_cast<size_t>(h.dfeaFloats));
        Copy(aBytes, *S[14], m_dbox, static_cast<size_t>(h.dboxFloats));
        if (S[15])
            Copy(aBytes, *S[15], m_crvn, C);
        Copy(aBytes, *S[16], m_curv, static_cast<size_t>(h.numFrames) * (C + 1));
        if (S[17])
            Copy(aBytes, *S[17], m_events, h.numEvents);
        Copy(aBytes, *S[18], m_rtz, h.numFrames);
        Copy(aBytes, *S[19], m_pose, static_cast<size_t>(h.numFrames) * h.numBones);
        Copy(aBytes, *S[20], m_root, h.numFrames);
        Copy(aBytes, *S[21], m_cont, h.numFrames);
        if (scal)
            Copy(aBytes, *scal, m_scal, static_cast<size_t>(h.numFrames) * h.numBones * 3);

        auto strOk = [&](uint32_t off) { return off == 0xFFFFFFFFu || (off < m_strs.size() && (off == 0 || m_strs[off - 1] == '\0')); };

        // ---- skeleton + rig identity
        std::vector<std::string_view> names(h.numBones);
        std::vector<int32_t> parents(h.numBones);
        std::vector<float> rest(static_cast<size_t>(h.numBones) * 3);
        for (uint32_t i = 0; i < h.numBones; ++i)
        {
            const auto& b = m_bones[i];
            if (!strOk(b.nameOffset) || b.nameOffset == 0xFFFFFFFFu || String(b.nameOffset).size() != b.nameLength ||
                b.reserved != 0)
                return Fail(aError, "bad bone name");
            if (i == 0 ? b.parent != -1 : (b.parent < 0 || b.parent >= static_cast<int32_t>(i)))
                return Fail(aError, "bad bone parent");
            names[i] = String(b.nameOffset);
            parents[i] = b.parent;
            const RigRestRecord& r = m_rigr[i];
            if (!Finite(&r.px, 7))
                return Fail(aError, "non-finite RIGR");
            rest[i * 3 + 0] = r.px;
            rest[i * 3 + 1] = r.py;
            rest[i * 3 + 2] = r.pz;
        }
        if (ComputeRigIdentity(names, parents, rest) != h.rigHash)
            return Fail(aError, "rig identity differs from BONE + RIGR");
        m_topologyHash = ComputeRigHash(names, parents);

        // ---- clips
        uint64_t frameCursor = 0;
        for (uint32_t c = 0; c < h.numClips; ++c)
        {
            const MotionClipRecord& cl = m_clips[c];
            const Clip2Record& c2 = m_clip2[c];
            if (!strOk(cl.nameOff) || cl.firstFrame != frameCursor || cl.numFrames == 0)
                return Fail(aError, "bad clip range / name");
            frameCursor += cl.numFrames;
            const uint32_t rtzBits = cl.flags & (kClipRtzUnknown | kClipRtzPresent | kClipRtzNone);
            if (rtzBits != kClipRtzUnknown && rtzBits != kClipRtzPresent && rtzBits != kClipRtzNone)
                return Fail(aError, "clip without exactly one RTZ flag");
            const uint32_t expectSrc = rtzBits == kClipRtzNone ? 0u : (rtzBits == kClipRtzUnknown ? 1u : 2u);
            if (c2.rtzSource != expectSrc)
                return Fail(aError, "CLP2 rtz_source differs from the CLIP RTZ flag");
            if (cl.flags & ~0x7Fu)
                return Fail(aError, "unknown clip flags");
            if (static_cast<uint64_t>(c2.firstEvent) + c2.numEvents > h.numEvents)
                return Fail(aError, "clip events out of range");
            if (!std::isfinite(c2.hipRatio) || !std::isfinite(c2.lengthS))
                return Fail(aError, "non-finite CLP2");
            if (h.numDatabases < 64 && (cl.dbMask >> h.numDatabases) != 0)
                return Fail(aError, "clip db_mask references a missing database");
        }
        if (frameCursor != h.numFrames)
            return Fail(aError, "clips do not cover the frames");

        // ---- schemas / channels
        for (uint32_t s = 0; s < h.numSchemas; ++s)
        {
            const SchemaRecord& sc = m_schemas[s];
            if (!strOk(sc.nameOff) || sc.dims == 0 || sc.dims > h.maxDims ||
                static_cast<uint64_t>(sc.firstChannel) + sc.numChannels > h.numChannels || !std::isfinite(sc.weightSum) ||
                sc.preprocessor > 2)
                return Fail(aError, "bad schema");
            for (uint32_t k = sc.firstChannel; k < sc.firstChannel + sc.numChannels; ++k)
            {
                const ChannelRecord& ch = m_channels[k];
                if (ch.cardinality < 1 || ch.cardinality > 3 || static_cast<uint64_t>(ch.dataOffset) + ch.cardinality > sc.dims ||
                    ch.bone < -1 || ch.bone >= static_cast<int32_t>(h.numBones) || ch.originBone < -1 ||
                    ch.originBone >= static_cast<int32_t>(h.numBones) || !Finite(&ch.tSample, 2) || !Finite(&ch.weight, 2) ||
                    !Finite(ch.axis, 3) || ch.kind < 1 || ch.kind > 4 || !strOk(ch.normGroupOff) || !strOk(ch.labelOff))
                    return Fail(aError, "bad schema channel");
            }
        }

        // ---- norms
        for (uint32_t n = 0; n < h.numNorms; ++n)
        {
            const NormRecord& nr = m_norms[n];
            if (nr.schema >= h.numSchemas || nr.dims != m_schemas[nr.schema].dims ||
                static_cast<uint64_t>(nr.nrmvOffset) + 3ull * nr.dims > m_nrmv.size())
                return Fail(aError, "bad norm");
            if (!Finite(NormMean(n), nr.dims) || !Finite(NormDeviation(n), nr.dims))
                return Fail(aError, "non-finite NRMV");
            const float* sc = NormScale(n);
            for (uint32_t k = 0; k < nr.dims; ++k)
                if (!(sc[k] > 0.0f) || !std::isfinite(sc[k]))
                    return Fail(aError, "non-positive NRMV scale");
        }

        // ---- databases / entries / rows / features / boxes
        uint64_t entryCursor = 0, rowCursor = 0;
        m_dbUsable.assign(h.numDatabases, 1);
        for (uint32_t d = 0; d < h.numDatabases; ++d)
        {
            const DatabaseRecord& db = m_dbs[d];
            if (!strOk(db.nameOff) || db.schema >= h.numSchemas || db.norm >= h.numNorms ||
                db.dims != m_schemas[db.schema].dims || m_norms[db.norm].dims != db.dims ||
                db.firstEntry != entryCursor || db.firstRow != rowCursor ||
                static_cast<uint64_t>(db.firstEntry) + db.numEntries > h.numEntries ||
                static_cast<uint64_t>(db.firstRow) + db.numRows > h.numRows)
                return Fail(aError, "bad database " + std::to_string(d) + " ranges");
            entryCursor += db.numEntries;
            rowCursor += db.numRows;
            const uint64_t nsm = (db.numRows + h.boundSmSize - 1) / h.boundSmSize;
            const uint64_t nlr = (db.numRows + h.boundLrSize - 1) / h.boundLrSize;
            if (db.numSmallBoxes != nsm || db.numLargeBoxes != nlr ||
                db.featOffset + static_cast<uint64_t>(db.numRows) * db.dims > h.dfeaFloats ||
                db.boxOffset + (nsm + nlr) * (2ull * db.dims + 1) > h.dboxFloats)
                return Fail(aError, "bad database " + std::to_string(d) + " features / boxes");
            if (!Finite(&db.baseCostBias, 5) || !std::isfinite(db.minAddend))
                return Fail(aError, "non-finite database biases");
            uint32_t selectable = 0;
            float minAdd = std::numeric_limits<float>::max();
            for (uint32_t e = db.firstEntry; e < db.firstEntry + db.numEntries; ++e)
            {
                const EntryRecord& en = m_entries[e];
                if (en.database != d || en.clip >= h.numClips || en.firstRow < db.firstRow ||
                    static_cast<uint64_t>(en.firstRow) + en.numRows > static_cast<uint64_t>(db.firstRow) + db.numRows ||
                    !std::isfinite(en.rangeMin) || !std::isfinite(en.rangeMax))
                    return Fail(aError, "bad entry " + std::to_string(e));
                if (((m_clips[en.clip].dbMask >> d) & 1u) == 0)
                    return Fail(aError, "entry clip without the database bit in db_mask");
            }
            for (uint32_t r = db.firstRow; r < db.firstRow + db.numRows; ++r)
            {
                const RowRecord& rr = m_rows[r];
                if (rr.frame >= h.numFrames || rr.entry < db.firstEntry || rr.entry >= db.firstEntry + db.numEntries ||
                    r < m_entries[rr.entry].firstRow || r >= m_entries[rr.entry].firstRow + m_entries[rr.entry].numRows ||
                    !std::isfinite(rr.costAddend) || !std::isfinite(rr.continuingAddend))
                    return Fail(aError, "bad row " + std::to_string(r));
                const MotionClipRecord& cl = m_clips[m_entries[rr.entry].clip];
                if (rr.frame < cl.firstFrame || rr.frame >= cl.firstFrame + cl.numFrames)
                    return Fail(aError, "row frame outside its entry clip");
                if (!(rr.flags & kRowBlock))
                {
                    ++selectable;
                    minAdd = std::min(minAdd, rr.costAddend);
                }
            }
            if (selectable != db.numSelectable || minAdd != db.minAddend)
                return Fail(aError, "database " + std::to_string(d) + " selectable count / min addend mismatch");
            if (!Finite(m_dfea.data() + db.featOffset, static_cast<size_t>(db.numRows) * db.dims))
                return Fail(aError, "non-finite DFEA");
            if (!Finite(m_dbox.data() + db.boxOffset, static_cast<size_t>((nsm + nlr) * (2ull * db.dims + 1))))
                return Fail(aError, "non-finite DBOX");
            // RTZ: a schema reading the root vertical cannot use clips without a root_z track
            bool zFromRtz = false;
            const SchemaRecord& sc = m_schemas[db.schema];
            for (uint32_t k = sc.firstChannel; k < sc.firstChannel + sc.numChannels; ++k)
                zFromRtz = zFromRtz || (m_channels[k].flags & kChanZFromRtz) != 0;
            bool unknown = false;
            for (uint32_t e = db.firstEntry; e < db.firstEntry + db.numEntries; ++e)
                unknown = unknown || (m_clips[m_entries[e].clip].flags & kClipRtzUnknown) != 0;
            if (zFromRtz && unknown)
            {
                m_dbUsable[d] = 0;
                ++m_numRefused;
            }
            if (((db.flags & kDbRtzPending) != 0) != (zFromRtz && unknown))
                return Fail(aError, "DBAS RTZ_PENDING differs from the clips / schema");
        }
        if (entryCursor != h.numEntries || rowCursor != h.numRows)
            return Fail(aError, "databases do not cover entries / rows");

        // ---- per-frame data
        for (uint32_t f = 0; f < h.numFrames; ++f)
        {
            const PoseRecord* p = Pose(f);
            for (uint32_t b = 0; b < h.numBones; ++b)
            {
                if (!Finite(&p[b].px, 7))
                    return Fail(aError, "non-finite POSE");
                const float n2 = p[b].qx * p[b].qx + p[b].qy * p[b].qy + p[b].qz * p[b].qz + p[b].qw * p[b].qw;
                if (std::fabs(n2 - 1.0f) > 1e-3f)
                    return Fail(aError, "non-unit POSE quaternion");
            }
            if (!Finite(&m_rtz[f].rootZ, 2) || !std::isfinite(m_root[f].dx) || !std::isfinite(m_root[f].dy) ||
                !std::isfinite(m_root[f].dyaw) || !Finite(&m_cont[f].l, 2))
                return Fail(aError, "non-finite RTZ_ / ROOT / CONT");
            const uint32_t* cv = &m_curv[static_cast<size_t>(f) * (C + 1)];
            if (C < 32 && (cv[0] >> C) != 0)
                return Fail(aError, "CURV presence bit beyond num_curves");
            for (uint32_t k = 0; k < C; ++k)
            {
                float v;
                std::memcpy(&v, &cv[1 + k], 4);
                if (!std::isfinite(v))
                    return Fail(aError, "non-finite CURV");
            }
        }
        for (uint32_t c = 0; c < h.numClips; ++c)
            if (!(m_clips[c].flags & kClipRtzPresent))
                for (uint32_t f = m_clips[c].firstFrame; f < m_clips[c].firstFrame + m_clips[c].numFrames; ++f)
                    if (m_rtz[f].rootZ != 0.0f || m_rtz[f].vz != 0.0f)
                        return Fail(aError, "RTZ_ row of a clip without RTZ_PRESENT is not 0");
        for (uint32_t k = 0; k < C; ++k)
            if (!strOk(m_crvn[k].nameOff) || m_crvn[k].unit > 3)
                return Fail(aError, "bad curve name");
        for (uint32_t e = 0; e < h.numEvents; ++e)
        {
            const EventRecord& ev = m_events[e];
            if (ev.clip >= h.numClips || !std::isfinite(ev.timeS) || !std::isfinite(ev.durationS) || !strOk(ev.tagOff) ||
                !strOk(ev.classOff))
                return Fail(aError, "bad event");
        }
        if (!m_scal.empty())
            for (float s : m_scal)
                if (!(s > 0.0f) || !std::isfinite(s))
                    return Fail(aError, "bad SCAL value");
        return true;
    }

    MotionHeader m_h{};
    bool m_loaded = false;
    uint64_t m_topologyHash = 0;
    uint32_t m_numRefused = 0;
    std::vector<Sec> m_secs;
    std::vector<char> m_strs;
    std::vector<BoneRecord> m_bones;
    std::vector<RigRestRecord> m_rigr;
    std::vector<MotionClipRecord> m_clips;
    std::vector<Clip2Record> m_clip2;
    std::vector<uint32_t> m_fflg;
    std::vector<SchemaRecord> m_schemas;
    std::vector<ChannelRecord> m_channels;
    std::vector<NormRecord> m_norms;
    std::vector<float> m_nrmv;
    std::vector<DatabaseRecord> m_dbs;
    std::vector<uint8_t> m_dbUsable;
    std::vector<EntryRecord> m_entries;
    std::vector<RowRecord> m_rows;
    std::vector<float> m_dfea;
    std::vector<float> m_dbox;
    std::vector<CurveNameRecord> m_crvn;
    std::vector<uint32_t> m_curv;
    std::vector<EventRecord> m_events;
    std::vector<RtzRecord> m_rtz;
    std::vector<PoseRecord> m_pose;
    std::vector<RootDelta> m_root;
    std::vector<ContactRecord> m_cont;
    std::vector<float> m_scal;
};

} // namespace utpp::mm::motion
