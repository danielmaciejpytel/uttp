// Offline self-test for the .utppmm reader and motion matching search (not shipped).
// Usage: utpp_mmdb_selftest <database.utppmm>      Exit code 0 = all checks passed.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <random>
#include <string>
#include <string_view>
#include <vector>

#include "../src/BaseDatabase.hpp"

using namespace utpp::mm;

static int g_fail = 0;
static int g_pass = 0;

static void Expect(const char* aName, bool aOk, const std::string& aDetail = {})
{
    std::printf("[%s] %s%s%s\n", aOk ? "PASS" : "FAIL", aName, aDetail.empty() ? "" : " -- ", aDetail.c_str());
    if (aOk)
        ++g_pass;
    else
        ++g_fail;
}

// player_woman_skeleton bones 0..72 (Extracted/game/json/vanilla/player_woman_skeleton.rig.json boneNames/boneParentIndexes).
static constexpr std::string_view kBoneNames[73] = {
    "Root", "Trajectory", "Hips", "reference_joint", "Spine", "LeftUpLeg", "RightUpLeg", "Spine1", "LeftLeg",
    "RightLeg", "Spine2", "LeftFoot", "RightFoot", "Spine3", "LeftShoulder", "RightShoulder", "Neck", "LeftArm",
    "RightArm", "Neck1", "LeftForeArm", "RightForeArm", "Head", "LeftHand", "RightHand", "LeftHandEnd",
    "WeaponLeft", "RightHandEnd", "WeaponRight", "LeftHeel", "RightHeel", "LeftEye", "RightEye",
    "LeftInHandThumb", "LeftInHandIndex", "LeftInHandMiddle", "LeftInHandRing", "LeftInHandPinky",
    "RightInHandThumb", "RightInHandIndex", "RightInHandMiddle", "RightInHandRing", "RightInHandPinky",
    "LeftToeBase", "RightToeBase", "LeftHandThumb1", "LeftHandIndex1", "LeftHandMiddle1", "LeftHandRing1",
    "LeftHandPinky1", "RightHandThumb1", "RightHandIndex1", "RightHandMiddle1", "RightHandRing1",
    "RightHandPinky1", "LeftHandThumb2", "LeftHandIndex2", "LeftHandMiddle2", "LeftHandRing2", "LeftHandPinky2",
    "RightHandThumb2", "RightHandIndex2", "RightHandMiddle2", "RightHandRing2", "RightHandPinky2",
    "LeftHandIndex3", "LeftHandMiddle3", "LeftHandRing3", "LeftHandPinky3", "RightHandIndex3",
    "RightHandMiddle3", "RightHandRing3", "RightHandPinky3"};
static constexpr int32_t kBoneParents[73] = {
    -1, 0, 0, 0, 2, 2, 2, 4, 5, 6, 7, 8, 9, 10, 13, 13, 13, 14, 15, 16, 17, 18, 19, 20, 21, 23, 23, 24, 24, 11, 12, 22, 22,
    23, 23, 23, 23, 23, 24, 24, 24, 24, 24, 29, 30, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 45, 46, 47, 48, 49, 50, 51,
    52, 53, 54, 56, 57, 58, 59, 61, 62, 63, 64};

static void PutU32(std::vector<uint8_t>& aB, size_t aOff, uint32_t aV) { std::memcpy(aB.data() + aOff, &aV, 4); }
static void PutU64(std::vector<uint8_t>& aB, size_t aOff, uint64_t aV) { std::memcpy(aB.data() + aOff, &aV, 8); }

static void FixCrcs(std::vector<uint8_t>& aB)
{
    PutU32(aB, offsetof(Header, payloadCrc32), Crc32(aB.data() + kHeaderSize, aB.size() - kHeaderSize));
    PutU32(aB, offsetof(Header, headerCrc32), 0);
    PutU32(aB, offsetof(Header, headerCrc32), Crc32(aB.data(), kHeaderSize));
}

static bool FindSection(const std::vector<uint8_t>& aB, const char (&aId)[5], SectionEntry& aOut, size_t& aEntryOff)
{
    Header h{};
    std::memcpy(&h, aB.data(), sizeof h);
    for (uint32_t i = 0; i < h.numSections; ++i)
    {
        const size_t off = h.sectionTableOffset + size_t(i) * kSectionEntrySize;
        std::memcpy(&aOut, aB.data() + off, sizeof aOut);
        if (aOut.id == FourCC(aId))
        {
            aEntryOff = off;
            return true;
        }
    }
    return false;
}

static void ExpectReject(const char* aName, const std::vector<uint8_t>& aBytes, uint64_t aRigHash, const char* aReason)
{
    Database db;
    std::string err;
    const bool ok = db.LoadFromMemory(aBytes, aRigHash, err);
    const bool closed = !ok && !db.Loaded() && db.NumFrames() == 0 && db.Features(0).empty() && db.Pose(0).empty();
    Expect(aName, closed && err == aReason, "error='" + err + "' expected='" + aReason + "'");
}

static double Percentile(std::vector<double> aV, double aP)
{
    if (aV.empty())
        return 0.0;
    std::sort(aV.begin(), aV.end());
    const size_t i = std::min(aV.size() - 1, static_cast<size_t>(aP * double(aV.size() - 1) + 0.5));
    return aV[i];
}

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::printf("usage: utpp_mmdb_selftest <database.utppmm>\n");
        return 2;
    }
    const uint64_t rigHash = ComputeRigHash(kBoneNames, kBoneParents);
    std::printf("expected rig hash (embedded table) = 0x%016llx\n", static_cast<unsigned long long>(rigHash));

    std::vector<uint8_t> bytes;
    std::string err;
    if (!Database::ReadFile(argv[1], bytes, err))
    {
        std::printf("[FAIL] read %s: %s\n", argv[1], err.c_str());
        return 1;
    }

    // ---------------------------------------------------------------- load + header
    Database db;
    const bool loaded = db.LoadFromFile(argv[1], rigHash, err);
    Expect("load (magic/version/header CRC/payload CRC/rig hash/bounds)", loaded, err);
    if (!loaded)
        return 1;
    const Header& h = db.GetHeader();
    std::printf("file %zu bytes, frames=%u bones=%u features=%u clips=%u ranges=%u groups=%u tags=%u sm=%u/%u lr=%u/%u fps=%.1f scale=%d\n",
                bytes.size(), h.numFrames, h.numBones, h.numFeatures, h.numClips, h.numRanges, h.numGroups, h.numTags,
                h.boundSmSize, h.numBoundSm, h.boundLrSize, h.numBoundLr, double(h.fps), int(db.HasScale()));
    Expect("rig hash == FNV-1a64(embedded names+parents)", db.RigHash() == rigHash);
    Expect("73 bones", db.NumBones() == 73);
    bool namesOk = db.NumBones() == 73;
    for (uint32_t b = 0; namesOk && b < 73; ++b)
        namesOk = db.BoneName(b) == kBoneNames[b] && db.BoneParent(b) == kBoneParents[b];
    Expect("bone names/parents match embedded table", namesOk);
    Expect("Crc32 known vector ('123456789' -> 0xCBF43926)",
           Crc32(reinterpret_cast<const uint8_t*>("123456789"), 9) == 0xCBF43926u);

    uint64_t clipFrames = 0, clipSearchable = 0, flagSearchable = 0, rangeFrames = 0;
    uint32_t loops = 0;
    for (const ClipRecord& c : db.Clips())
    {
        clipFrames += c.numFrames;
        clipSearchable += c.numSearchable;
        loops += (c.flags & kClipLoop) ? 1u : 0u;
    }
    for (uint32_t i = 0; i < db.NumFrames(); ++i)
        flagSearchable += (db.FrameFlagsOf(i) & kFrameSearchable) ? 1u : 0u;
    for (const Range& r : db.SearchRanges())
        rangeFrames += r.stop - r.start;
    Expect("sum(clip frames) == numFrames", clipFrames == db.NumFrames());
    Expect("clip searchable == flag searchable == range frames",
           clipSearchable == flagSearchable && flagSearchable == rangeFrames,
           std::to_string(flagSearchable) + " searchable, " + std::to_string(db.NumFrames() - flagSearchable) + " unsearchable, loops=" +
               std::to_string(loops));
    for (uint32_t g = 0; g < db.Groups().size(); ++g)
    {
        const GroupRecord& gr = db.Groups()[g];
        std::printf("  group %-22.*s kind=%u dims=%u..%u bone=%d weight=%.3f std=%.5f horizons=%u\n",
                    int(db.GroupName(g).size()), db.GroupName(g).data(), gr.kind, gr.firstFeature,
                    gr.firstFeature + gr.numFeatures - 1, gr.bone, double(gr.weight), double(gr.std), gr.numHorizons);
    }

    // ---------------------------------------------------------------- fail closed
    {
        std::vector<uint8_t> b = bytes;
        b[0] = 'X';
        ExpectReject("reject bad magic", b, rigHash, "bad magic");
    }
    {
        std::vector<uint8_t> b = bytes;
        PutU32(b, offsetof(Header, version), kVersion + 1);
        FixCrcs(b);
        ExpectReject("reject unsupported version (CRCs valid)", b, rigHash, "unsupported version");
    }
    {
        std::vector<uint8_t> b = bytes;
        PutU32(b, offsetof(Header, numFrames), h.numFrames + 1);
        ExpectReject("reject header tamper (header CRC)", b, rigHash, "header CRC mismatch");
    }
    {
        std::vector<uint8_t> b = bytes;
        b[b.size() / 2] ^= 0x01;
        ExpectReject("reject payload bit flip (payload CRC)", b, rigHash, "payload CRC mismatch");
    }
    {
        std::vector<uint8_t> b(bytes.begin(), bytes.end() - 16);
        ExpectReject("reject truncated file", b, rigHash, "file size mismatch (truncated?)");
        std::vector<uint8_t> tiny(bytes.begin(), bytes.begin() + 64);
        ExpectReject("reject file shorter than header", tiny, rigHash, "file too small");
    }
    ExpectReject("reject wrong expected rig hash", bytes, rigHash ^ 1ull, "rig hash mismatch");
    {
        std::vector<uint8_t> b = bytes;
        SectionEntry e{};
        size_t eo = 0;
        if (FindSection(b, "FEAT", e, eo))
        {
            PutU64(b, eo + offsetof(SectionEntry, offset), uint64_t(b.size()) + 16);
            FixCrcs(b);
        }
        ExpectReject("reject section offset out of file (CRCs valid)", b, rigHash, "section out of bounds / inconsistent");
    }
    {
        std::vector<uint8_t> b = bytes;
        SectionEntry e{};
        size_t eo = 0;
        if (FindSection(b, "FEAT", e, eo))
        {
            const float nan = std::numeric_limits<float>::quiet_NaN();
            std::memcpy(b.data() + e.offset + 4 * 5, &nan, 4);
            FixCrcs(b);
        }
        ExpectReject("reject NaN feature (CRCs valid)", b, rigHash, "non-finite value");
    }
    {
        std::vector<uint8_t> b = bytes;
        SectionEntry e{};
        size_t eo = 0;
        if (FindSection(b, "BONE", e, eo))
        {
            int32_t p = 5; // bone 11 (LeftFoot) parent 8 -> 5
            std::memcpy(b.data() + e.offset + 11 * 16 + 4, &p, 4);
            FixCrcs(b);
        }
        ExpectReject("reject skeleton not matching stored rig hash (CRCs valid)", b, rigHash,
                     "stored skeleton does not match rig hash");
    }
    {
        // A loaded database that is reloaded from bad data must end up empty.
        Database d2;
        std::string e2;
        const bool first = d2.LoadFromMemory(bytes, rigHash, e2);
        std::vector<uint8_t> b = bytes;
        b[0] = 0;
        const bool second = d2.LoadFromMemory(b, rigHash, e2);
        Expect("reload with bad data clears a previously loaded db", first && !second && !d2.Loaded() && d2.NumFrames() == 0);
    }

    // ---------------------------------------------------------------- normalization round trip
    const uint32_t nf = db.NumFeatures();
    {
        std::vector<float> raw(nf), norm(nf);
        float maxErr = 0.0f;
        for (uint32_t i = 0; i < db.NumFrames(); i += 7)
        {
            const auto f = db.Features(i);
            db.DenormalizeFeatures(f, raw);
            db.NormalizeQuery(raw, norm);
            for (uint32_t j = 0; j < nf; ++j)
                maxErr = std::max(maxErr, std::fabs(norm[j] - f[j]));
        }
        Expect("NormalizeQuery(Denormalize(f)) == f", maxErr < 1e-4f, "max err " + std::to_string(maxErr));
    }

    // ---------------------------------------------------------------- AABB search == brute force
    std::mt19937 rng(0x5EED1234u);
    std::normal_distribution<float> gauss(0.0f, 1.0f);
    std::vector<uint32_t> searchableFrames;
    for (uint32_t i = 0; i < db.NumFrames(); ++i)
        if (db.FrameFlagsOf(i) & kFrameSearchable)
            searchableFrames.push_back(i);
    std::uniform_int_distribution<size_t> pickS(0, searchableFrames.size() - 1);
    std::uniform_int_distribution<uint32_t> pickAny(0, db.NumFrames() - 1);

    struct Q
    {
        std::vector<float> q;
        int32_t curr;
        int32_t ignore;
        float tcost;
    };
    std::vector<Q> queries;
    // (a) every searchable frame's own features (exact real queries)
    for (uint32_t i : searchableFrames)
        queries.push_back({std::vector<float>(db.Features(i).begin(), db.Features(i).end()), -1, 0, 0.0f});
    // (b) real frames with trajectory dims perturbed (simulated controller input), with current-frame semantics
    for (int k = 0; k < 4000; ++k)
    {
        const uint32_t i = searchableFrames[pickS(rng)];
        std::vector<float> q(db.Features(i).begin(), db.Features(i).end());
        for (uint32_t j = 15; j < nf; ++j)
            q[j] += 0.5f * gauss(rng);
        const bool withCurr = (k % 2) == 0;
        queries.push_back({std::move(q), withCurr ? static_cast<int32_t>(pickAny(rng)) : -1, withCurr ? 10 : 0,
                           (k % 3 == 0) ? 0.25f : 0.0f});
    }
    // (c) fully random queries in normalized space
    for (int k = 0; k < 4000; ++k)
    {
        std::vector<float> q(nf);
        for (float& v : q)
            v = 1.5f * gauss(rng);
        queries.push_back({std::move(q), -1, 0, 0.0f});
    }

    uint32_t mismatches = 0, noResult = 0;
    std::vector<double> tAabb, tBrute;
    tAabb.reserve(queries.size());
    tBrute.reserve(queries.size());
    using Clock = std::chrono::steady_clock;
    // warm-up
    for (int k = 0; k < 50; ++k)
        (void)db.Search(queries[size_t(k)].q);
    for (const Q& q : queries)
    {
        const auto t0 = Clock::now();
        const SearchResult a = db.Search(q.q, q.curr, q.ignore, q.tcost);
        const auto t1 = Clock::now();
        const SearchResult b = db.SearchBruteForce(q.q, q.curr, q.ignore, q.tcost);
        const auto t2 = Clock::now();
        tAabb.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
        tBrute.push_back(std::chrono::duration<double, std::micro>(t2 - t1).count());
        if (a.index != b.index || std::memcmp(&a.cost, &b.cost, sizeof(float)) != 0)
        {
            if (mismatches < 5)
                std::printf("  mismatch: aabb=(%d, %.9g) brute=(%d, %.9g)\n", a.index, double(a.cost), b.index, double(b.cost));
            ++mismatches;
        }
        if (a.index < 0)
            ++noResult;
        else if (q.curr < 0 && !(db.FrameFlagsOf(static_cast<uint32_t>(a.index)) & kFrameSearchable))
            ++mismatches; // without a current frame only searchable frames may be returned
    }
    Expect("AABB search == brute force (index and bitwise cost)", mismatches == 0,
           std::to_string(queries.size()) + " queries (" + std::to_string(searchableFrames.size()) +
               " exact real, 4000 perturbed real incl. current-frame/transition-cost variants, 4000 random), " +
               std::to_string(mismatches) + " mismatches, " + std::to_string(noResult) + " without result");
    {
        uint32_t exactHits = 0;
        for (size_t k = 0; k < searchableFrames.size(); ++k)
        {
            const SearchResult r = db.Search(queries[k].q);
            exactHits += (r.index >= 0 && r.cost == 0.0f) ? 1u : 0u;
        }
        Expect("exact real queries return a zero-cost frame", exactHits == searchableFrames.size(),
               std::to_string(exactHits) + "/" + std::to_string(searchableFrames.size()));
    }

    const double aMed = Percentile(tAabb, 0.5), aP99 = Percentile(tAabb, 0.99), aMax = Percentile(tAabb, 1.0);
    const double bMed = Percentile(tBrute, 0.5), bP99 = Percentile(tBrute, 0.99), bMax = Percentile(tBrute, 1.0);
    std::printf("timing per query (us): AABB median=%.2f p99=%.2f max=%.2f | brute median=%.2f p99=%.2f max=%.2f | n=%zu\n",
                aMed, aP99, aMax, bMed, bP99, bMax, tAabb.size());
    Expect("AABB median search time < 300 us budget", aMed < 300.0);

    std::printf("SUMMARY: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
