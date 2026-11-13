// Offline test for src/mm/AirTable.hpp: synthetic .airz tables (exact reasons, range reject, RTZ-clean detection) and
// the real v6 DB + .airz (counts, metadata, Pick coverage incl. the explicit fallback table, loop check).
// Usage: MMAirTableTest [db.utppmm] [table.airz]   (real part skipped when the files are missing)
#include <cstddef>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "BaseDatabase.hpp"
#include "mm/AirTable.hpp"

using namespace utpp::mm;
using namespace utpp::mm::airtab;

static int g_fail = 0;
static void Expect(const char* aName, bool aOk)
{
    std::printf("[%s] %s\n", aOk ? "PASS" : "FAIL", aName);
    if (!aOk)
        ++g_fail;
}

// ---------------------------------------------------------------- synthetic DB

struct FakeDb
{
    std::vector<std::string> bones = {"Root", "Hips", "LeftUpLeg"};
    std::vector<std::string> names;
    std::vector<ClipRecord> clips;
    std::vector<float> pose; // 3 bones * 7 per frame
    std::vector<RootDelta> root;
    std::vector<std::array<float, 2>> contacts;
    uint32_t frames = 0;

    // aContact(f, n): true when a foot is planted on clip-local frame f (nullptr = always planted).
    void AddClip(const char* aName, uint32_t aN, float (*aHips)(uint32_t, uint32_t), bool (*aContact)(uint32_t, uint32_t) = nullptr)
    {
        ClipRecord r{};
        r.firstFrame = frames;
        r.numFrames = aN;
        clips.push_back(r);
        names.emplace_back(aName);
        for (uint32_t f = 0; f < aN; ++f)
        {
            for (uint32_t b = 0; b < 3; ++b)
            {
                const float p[7] = {0.0f, 0.0f, b == 1 ? aHips(f, aN) : 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
                pose.insert(pose.end(), p, p + 7);
            }
            root.push_back(RootDelta{0.0f, 0.1f, 0.0f, 0.0f});
            const bool c = aContact ? aContact(f, aN) : true;
            contacts.push_back({c ? 1.0f : 0.0f, 0.0f});
        }
        frames += aN;
    }
    uint32_t NumFrames() const { return frames; }
    uint32_t NumBones() const { return 3; }
    uint32_t NumClips() const { return uint32_t(clips.size()); }
    std::span<const ClipRecord> Clips() const { return clips; }
    std::string_view ClipName(uint32_t c) const { return c < names.size() ? std::string_view(names[c]) : std::string_view{}; }
    std::string_view BoneName(uint32_t b) const { return b < bones.size() ? std::string_view(bones[b]) : std::string_view{}; }
    std::span<const float> Pose(uint32_t f) const { return f < frames ? std::span<const float>(pose).subspan(size_t(f) * 21, 21) : std::span<const float>{}; }
    const RootDelta* Root(uint32_t f) const { return f < frames ? &root[f] : nullptr; }
    std::array<float, 2> Contacts(uint32_t f) const { return f < frames ? contacts[f] : std::array<float, 2>{0.0f, 0.0f}; }
};

static float Arc(uint32_t f, uint32_t n) { return 1.0f + 0.4f * std::sin(3.14159265f * float(f) / float(n - 1)); }
static float Flat(uint32_t, uint32_t) { return 1.0f; }
static float RzArc(uint32_t f, uint32_t n) { return 0.4f * std::sin(3.14159265f * float(f) / float(n - 1)); }
static float OffHips(uint32_t f, uint32_t) { return 1.0f - 0.02f * float(f); }
static float RzOff(uint32_t f, uint32_t) { return -0.02f * float(f); }
// Start: planted 0-2, airborne 3-7, planted 8-10 -> liftoff 3, touch 8. Off: planted 0-1 then airborne -> liftoff 2.
static bool StartContact(uint32_t f, uint32_t) { return f < 3 || f >= 8; }
static bool OffContact(uint32_t f, uint32_t) { return f < 2; }

struct Entry
{
    uint32_t clip;
    std::string name;
    std::vector<float> rz;
};

static void Put(std::vector<uint8_t>& o, const void* p, size_t n)
{
    const auto* b = static_cast<const uint8_t*>(p);
    o.insert(o.end(), b, b + n);
}

static std::vector<uint8_t> MakeAirz(const std::array<uint8_t, 32>& aSha, const std::vector<Entry>& aE, uint32_t aVersion = 1)
{
    std::vector<uint8_t> o;
    Put(o, "UTPPAIRZ", 8);
    Put(o, &aVersion, 4);
    Put(o, aSha.data(), 32);
    const uint32_t n = uint32_t(aE.size());
    Put(o, &n, 4);
    for (const Entry& e : aE)
    {
        const uint16_t nl = uint16_t(e.name.size());
        const uint32_t nf = uint32_t(e.rz.size());
        Put(o, &e.clip, 4);
        Put(o, &nl, 2);
        Put(o, e.name.data(), nl);
        Put(o, &nf, 4);
        Put(o, e.rz.data(), size_t(nf) * 4);
    }
    return o;
}

static void Synthetic()
{
    const uint8_t abc[3] = {'a', 'b', 'c'};
    const auto shaAbc = Sha256::Of(abc);
    Expect("sha256('abc') known vector", shaAbc[0] == 0xba && shaAbc[1] == 0x78 && shaAbc[31] == 0xad);

    FakeDb db;
    db.AddClip("Idle", 5, Flat);
    db.AddClip("M_Neutral_Jump_F_Start_Run_Lfoot", 11, Arc, StartContact); // raw Hips carries the arc -> Applied
    db.AddClip("M_Neutral_Jump_F_Land_Run_Light_Lfoot", 11, Flat); // already RTZ-clean -> RtzClean
    db.AddClip("M_Neutral_Jump_Loop_Fall", 4, Flat);
    db.AddClip("M_Neutral_Jump_F_Off_Walk_Lfoot", 8, OffHips, OffContact); // frames 31..38, root_z goes negative
    std::array<uint8_t, 32> sha{};
    sha[0] = 7;
    auto rz = [](uint32_t n, float (*fn)(uint32_t, uint32_t)) {
        std::vector<float> v(n);
        for (uint32_t f = 0; f < n; ++f)
            v[f] = fn(f, n);
        return v;
    };
    const std::vector<Entry> good = {{1, "M_Neutral_Jump_F_Start_Run_Lfoot", rz(11, RzArc)},
                                     {2, "M_Neutral_Jump_F_Land_Run_Light_Lfoot", rz(11, RzArc)},
                                     {3, "M_Neutral_Jump_Loop_Fall", std::vector<float>(4, 0.0f)},
                                     {4, "M_Neutral_Jump_F_Off_Walk_Lfoot", rz(8, RzOff)}};
    AirTable t;
    std::string why;
    Expect("good table parses", t.Parse(MakeAirz(sha, good), db, sha, why) && why.empty());
    Expect("rootZ size == db frames", t.RootZ().size() == db.NumFrames());
    Expect("Applied clip keeps root_z", std::fabs(t.RootZAt(5 + 5) - 0.4f) < 1e-5f);
    Expect("RtzClean clip: root_z not applied", t.RootZAt(16 + 5) == 0.0f && t.MetaOfClip(2)->status == ClipStatus::RtzClean);
    Expect("frames without data are 0", t.RootZAt(0) == 0.0f && t.RootZAt(4) == 0.0f);
    Expect("Applied status", t.MetaOfClip(1)->status == ClipStatus::Applied && t.NumAirzClean() == 1);
    Expect("Idle is not a jump clip -> 4 metas", t.Meta().size() == 4);
    const ClipMeta* s = t.MetaOfClip(1);
    Expect("start meta kind/dir/band/foot", s->kind == Kind::Start && s->nameDir == Dir::F && s->band == Band::Run && s->foot == Foot::L);
    Expect("start apex at mid clip, travel F", s->apex == 5 && s->travelDir == Dir::F);
    Expect("start liftoff 3 / touch 8 from contacts", s->liftoff == 3 && s->touch == 8);
    Expect("start zeroCross after apex (frame 10)", s->zeroCross == 10);
    const ClipMeta* off = t.MetaOfClip(4);
    Expect("off meta: liftoff 2, drop 3 (first frame below -5 cm after liftoff), no apex",
           off && off->kind == Kind::Off && off->liftoff == 2 && off->drop == 3 && off->apex == -1 && off->touch == -1);
    // Pick wildcards: F, Run, Light, Lfoot preferred; never flagged.
    FakeDb pdb;
    pdb.AddClip("M_Neutral_Jump_B_Start_Lfoot", 3, Flat);
    pdb.AddClip("M_Neutral_Jump_F_Land_Run_Heavy_Lfoot", 3, Flat);
    pdb.AddClip("M_Neutral_Jump_F_Land_Run_Light_Rfoot", 3, Flat);
    pdb.AddClip("M_Neutral_Jump_F_Land_Run_Light_Lfoot", 3, Flat);
    pdb.AddClip("M_Neutral_Jump_F_Start_Walk_Lfoot", 3, Flat);
    pdb.AddClip("M_Neutral_Jump_F_Start_Run_Lfoot", 3, Flat);
    AirTable pt;
    Expect("pick table parses", pt.Parse(MakeAirz(sha, {}), pdb, sha, why));
    const PickResult w1 = pt.Pick(Kind::Land, Dir::None, Band::None, Weight::None, Foot::None);
    Expect("Pick all-None land -> F Run Light Lfoot, not flagged", w1.clip == 3 && w1.relaxed == PickResult::kExact);
    const PickResult w2 = pt.Pick(Kind::Start, Dir::None, Band::Run, Weight::None, Foot::None);
    Expect("Pick dir None start -> F_Start_Run (not B, not Walk)", w2.clip == 5 && w2.relaxed == PickResult::kExact);
    const PickResult w3 = pt.Pick(Kind::Start, Dir::B, Band::Run, Weight::None, Foot::L);
    Expect("Pick B start keeps bandless B", w3.clip == 0 && w3.relaxed == PickResult::kExact);
    Expect("loop ok on identical frames", t.LoopOk() && t.FallLoopClip() == 3);

    auto reason = [&](const std::vector<uint8_t>& aBytes, std::span<const uint8_t> aSha) {
        AirTable x;
        std::string r;
        const bool ok = x.Parse(aBytes, db, aSha, r);
        return ok ? std::string("OK") : r;
    };
    auto bad = MakeAirz(sha, good);
    bad[0] = 'X';
    Expect("bad magic", reason(bad, sha) == "bad magic");
    Expect("bad version", reason(MakeAirz(sha, good, 2), sha) == "unsupported version");
    std::array<uint8_t, 32> other = sha;
    other[5] = 1;
    Expect("sha mismatch", reason(MakeAirz(sha, good), other) == "db sha256 mismatch");
    auto e = good;
    e[0].name = "M_Neutral_Jump_F_Start_Run_Rfoot";
    Expect("name mismatch", reason(MakeAirz(sha, e), sha) == "clip name mismatch");
    e = good;
    e[0].rz.pop_back();
    Expect("frame count mismatch", reason(MakeAirz(sha, e), sha) == "frame count mismatch");
    e = good;
    e[0].clip = 9;
    Expect("clip index out of range", reason(MakeAirz(sha, e), sha) == "clip index out of range");
    e = good;
    std::swap(e[0], e[1]);
    Expect("clip order", reason(MakeAirz(sha, e), sha) == "clip order");
    e = good;
    e[0].rz[3] = std::nanf("");
    Expect("non-finite root_z", reason(MakeAirz(sha, e), sha) == "non-finite root_z");
    auto tr = MakeAirz(sha, good);
    tr.push_back(0);
    Expect("trailing bytes", reason(tr, sha) == "trailing bytes");
    tr = MakeAirz(sha, good);
    tr.resize(tr.size() - 3);
    Expect("truncated root_z", reason(tr, sha) == "truncated root_z");
    Expect("truncated header", reason(std::vector<uint8_t>(tr.begin(), tr.begin() + 20), sha) == "truncated header");

    // Range reject: Hips 1.0 - root_z 0.9 = 0.1 < 0.30 -> only that clip rejected, table still loads.
    e = good;
    std::fill(e[0].rz.begin(), e[0].rz.end(), 0.9f);
    AirTable r;
    Expect("range reject keeps table", r.Parse(MakeAirz(sha, e), db, sha, why));
    Expect("range reject status + zeroed", r.MetaOfClip(1)->status == ClipStatus::RejectedRange && r.RootZAt(6) == 0.0f && r.NumAirzRejected() == 1);

    // Leg pose cost.
    const float a[7] = {0, 0, 0, 0, 0, 0, 1};
    const float b[7] = {0, 0, 0, 0, 0, 0.7071068f, 0.7071068f};
    const uint32_t chain[1] = {0};
    Expect("LegPoseCost 90 deg", std::fabs(LegPoseCost(a, b, chain) - 1.5707963f) < 1e-3f);
    const float n[7] = {0, 0, 0, std::nanf(""), 0, 0, 1};
    Expect("LegPoseCost NaN-safe", std::isfinite(LegPoseCost(a, n, chain)));
}

// ---------------------------------------------------------------- real v6

static bool ReadAll(const char* aPath, std::vector<uint8_t>& aOut)
{
    std::ifstream f(aPath, std::ios::binary);
    if (!f)
        return false;
    aOut.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
    return true;
}

static const char* KindName(Kind k) { static const char* n[] = {"None", "Start", "FallLoop", "Land", "Off"}; return n[int(k)]; }
static const char* DirName(Dir d) { static const char* n[] = {"-", "F", "B", "L", "R"}; return n[int(d)]; }
static const char* BandName(Band b) { static const char* n[] = {"-", "Stand", "Walk", "Run", "Sprint"}; return n[int(b)]; }
static const char* StatusName(ClipStatus s) { static const char* n[] = {"NoData", "Applied", "RtzClean", "Rejected"}; return n[int(s)]; }

static void Real(const char* aDb, const char* aAirz)
{
    std::vector<uint8_t> dbBytes, airz;
    if (!ReadAll(aDb, dbBytes) || !ReadAll(aAirz, airz))
    {
        std::printf("[SKIP] real v6: files missing\n");
        return;
    }
    uint64_t rig = 0;
    std::memcpy(&rig, dbBytes.data() + offsetof(Header, rigHash), 8);
    Database db;
    std::string err;
    Expect("v6 loads", db.LoadFromMemory(dbBytes, rig, err));
    const auto sha = Sha256::Of(dbBytes);
    dbBytes.clear();
    dbBytes.shrink_to_fit();
    AirTable t;
    const bool ok = t.Parse(airz, db, sha, err);
    std::printf("v6 parse: %s reason='%s' airzClips=%u applied=%u clean=%u rejected=%u metas=%zu\n", ok ? "ok" : "FAIL", err.c_str(),
                t.NumAirzClips(), t.CountStatus(ClipStatus::Applied), t.NumAirzClean(), t.NumAirzRejected(), t.Meta().size());
    Expect("v6 .airz parses", ok);
    if (t.FirstRejectedClip() >= 0)
        std::printf("first range-rejected .airz clip: %d %s\n", t.FirstRejectedClip(), std::string(db.ClipName(uint32_t(t.FirstRejectedClip()))).c_str());
    if (!ok)
        return;
    Expect("v6 rootZ size", t.RootZ().size() == db.NumFrames());
    Expect("v6 58 jump clips", t.Meta().size() == 58);
    uint32_t nk[5] = {}, dirAgree = 0, dirKnown = 0;
    bool sane = true;
    for (const ClipMeta& m : t.Meta())
    {
        ++nk[int(m.kind)];
        if (m.travelDir != Dir::None && m.kind != Kind::FallLoop)
        {
            ++dirKnown;
            dirAgree += m.travelDir == m.nameDir ? 1u : 0u;
        }
        sane = sane && m.numFrames > 0 && m.minHips >= kHipsMin && m.maxHips <= kHipsMax && m.nameDir != (m.kind == Kind::FallLoop ? Dir::F : Dir::None);
        std::printf("  clip %3d %-40s %-8s nd=%s td=%s(%+.2f,%+.2f) %-6s w=%d f=%d %-8s hips[%.2f,%.2f] maxRz=%.2f lift=%d apex=%d zc=%d touch=%d drop=%d\n",
                    m.clip, std::string(db.ClipName(uint32_t(m.clip))).c_str(), KindName(m.kind), DirName(m.nameDir), DirName(m.travelDir),
                    m.travelX, m.travelY, BandName(m.band), int(m.weight), int(m.foot), StatusName(m.status), m.minHips, m.maxHips, m.maxRootZ,
                    m.liftoff, m.apex, m.zeroCross, m.touch, m.drop);
    }
    std::printf("kinds: Start=%u FallLoop=%u Land=%u Off=%u; travel dir agrees with name %u/%u\n", nk[1], nk[2], nk[3], nk[4], dirAgree, dirKnown);
    Expect("v6 kind counts 14/1/39/4", nk[1] == 14 && nk[2] == 1 && nk[3] == 39 && nk[4] == 4);
    Expect("v6 metadata sane (hips in range, dir parsed)", sane);
    std::printf("loop: clip=%d loopOk=%d maxDeg=%.3f maxM=%.4f\n", t.FallLoopClip(), int(t.LoopOk()), t.LoopMaxDeg(), t.LoopMaxM());
    Expect("v6 fall loop clip found", t.FallLoopClip() >= 0 && db.ClipName(uint32_t(t.FallLoopClip())) == "M_Neutral_Jump_Loop_Fall");
    Expect("v6 fall loop loopOk (1 deg / 1 cm)", t.LoopOk() && t.LoopMaxDeg() < 0.2f && t.LoopMaxM() < 1e-3f);

    // Event frames (guards the measured ranges): Land touch Light 14-18 / Heavy 29-33, Off liftoff >= 0 and
    // drop 20-45, Start apex 5-35 with zeroCross after it.
    bool landOk = true, offOk = true, startOk = true;
    for (const ClipMeta& m : t.Meta())
    {
        if (m.kind == Kind::Land)
        {
            const bool heavy = m.weight == Weight::Heavy;
            landOk = landOk && m.touch >= (heavy ? 29 : 14) && m.touch <= (heavy ? 33 : 18) && m.zeroCross >= 0;
        }
        else if (m.kind == Kind::Off)
            offOk = offOk && m.liftoff >= 0 && m.drop >= 20 && m.drop <= 45 && m.drop > m.liftoff;
        else if (m.kind == Kind::Start)
            startOk = startOk && m.apex >= 5 && m.apex <= 35 && m.zeroCross > m.apex;
    }
    Expect("v6 Land touch frames (Light 14-18, Heavy 29-33)", landOk);
    Expect("v6 Off liftoff / drop frames", offOk);
    Expect("v6 Start apex / zeroCross frames", startOk);

    // Pick coverage: every combination returns a clip of the right kind; direction relaxed only for Off.
    const Kind kinds[4] = {Kind::Start, Kind::FallLoop, Kind::Land, Kind::Off};
    uint32_t combos = 0, exact = 0;
    bool allValid = true, dirOk = true;
    for (Kind k : kinds)
        for (int d = 1; d <= 4; ++d)
            for (int b = 1; b <= 4; ++b)
                for (int w = 1; w <= 2; ++w)
                    for (int f = 1; f <= 2; ++f)
                    {
                        const PickResult p = t.Pick(k, Dir(d), Band(b), Weight(w), Foot(f));
                        ++combos;
                        exact += p.relaxed == PickResult::kExact ? 1u : 0u;
                        const ClipMeta* m = t.MetaOfClip(p.clip);
                        allValid = allValid && m && m->kind == k;
                        if ((p.relaxed & PickResult::kDirRelaxed) && !(k == Kind::Off && d != 1))
                            dirOk = false;
                    }
    std::printf("Pick: %u combos, %u exact\n", combos, exact);
    Expect("Pick: every combination valid", allValid);
    Expect("Pick: direction relaxed only for B/L/R Off", dirOk);

    // Explicit fallback table (critique MUST): requested -> expected clip name + flags.
    struct Row
    {
        Kind k;
        Dir d;
        Band b;
        Weight w;
        Foot f;
        const char* name;
        uint32_t relaxed;
    };
    const Row rows[] = {
        {Kind::Start, Dir::F, Band::Sprint, Weight::Light, Foot::R, "M_Neutral_Jump_F_Start_Sprint_Rfoot", PickResult::kExact},
        {Kind::Start, Dir::B, Band::Sprint, Weight::Light, Foot::R, "M_Neutral_Jump_B_Start_Rfoot", PickResult::kExact},
        {Kind::Start, Dir::L, Band::Walk, Weight::Heavy, Foot::L, "M_Neutral_Jump_LL_Start_Lfoot", PickResult::kExact},
        {Kind::Land, Dir::B, Band::Sprint, Weight::Heavy, Foot::L, "M_Neutral_Jump_B_Land_Run_Heavy", PickResult::kBandRelaxed},
        {Kind::Land, Dir::R, Band::Sprint, Weight::Light, Foot::R, "M_Neutral_Jump_RL_Land_Run_Light", PickResult::kBandRelaxed},
        {Kind::Land, Dir::L, Band::Walk, Weight::Light, Foot::R, "M_Neutral_Jump_LL_Land_Walk_Light", PickResult::kExact},
        {Kind::Land, Dir::F, Band::Walk, Weight::Heavy, Foot::R, "M_Neutral_Jump_F_Land_Walk_Heavy_Lfoot", PickResult::kFootRelaxed},
        {Kind::Land, Dir::B, Band::Stand, Weight::Heavy, Foot::R, "M_Neutral_Jump_B_Land_Stand_Heavy_Rfoot", PickResult::kExact},
        {Kind::Off, Dir::B, Band::Walk, Weight::Light, Foot::L, "M_Neutral_Jump_F_Off_Walk_Lfoot", PickResult::kDirRelaxed},
        {Kind::Off, Dir::F, Band::Stand, Weight::Light, Foot::R, "M_Neutral_Jump_F_Off_Walk_Rfoot", PickResult::kBandRelaxed},
        {Kind::Off, Dir::F, Band::Sprint, Weight::Light, Foot::R, "M_Neutral_Jump_F_Off_Run_Rfoot", PickResult::kBandRelaxed},
        {Kind::FallLoop, Dir::R, Band::Sprint, Weight::Heavy, Foot::R, "M_Neutral_Jump_Loop_Fall", PickResult::kExact},
        // Wildcards (None): F, Run, Light, Lfoot preferred, never flagged.
        {Kind::Start, Dir::None, Band::Run, Weight::None, Foot::L, "M_Neutral_Jump_F_Start_Run_Lfoot", PickResult::kExact},
        {Kind::Start, Dir::None, Band::None, Weight::None, Foot::None, "M_Neutral_Jump_F_Start_Run_Lfoot", PickResult::kExact},
        {Kind::Land, Dir::None, Band::Run, Weight::Light, Foot::None, "M_Neutral_Jump_F_Land_Run_Light_Lfoot", PickResult::kExact},
        {Kind::Land, Dir::F, Band::Run, Weight::None, Foot::R, "M_Neutral_Jump_F_Land_Run_Light_Rfoot", PickResult::kExact},
        {Kind::Off, Dir::None, Band::None, Weight::None, Foot::None, "M_Neutral_Jump_F_Off_Run_Lfoot", PickResult::kExact},
    };
    for (const Row& r : rows)
    {
        const PickResult p = t.Pick(r.k, r.d, r.b, r.w, r.f);
        const std::string got(db.ClipName(uint32_t(std::max(p.clip, 0))));
        const bool okRow = p.clip >= 0 && got == r.name && p.relaxed == r.relaxed;
        std::printf("  fallback %s %s %s -> %s relaxed=%u\n", KindName(r.k), DirName(r.d), BandName(r.b), got.c_str(), p.relaxed);
        Expect(r.name, okRow);
    }

    std::array<uint32_t, 6> chain{};
    const uint32_t nc = FindLegChain(db, chain);
    Expect("leg chain 6 bones", nc == 6);
    const auto* land = t.MetaOfClip(300);
    if (land)
    {
        const float self = LegPoseCost(db.Pose(land->firstFrame), db.Pose(land->firstFrame), std::span<const uint32_t>(chain.data(), nc));
        const float other = LegPoseCost(db.Pose(land->firstFrame), db.Pose(land->firstFrame + 20), std::span<const uint32_t>(chain.data(), nc));
        std::printf("LegPoseCost self=%.4f vs +20 frames=%.4f\n", self, other);
        Expect("LegPoseCost self == 0 (atan2 angle), other > 0.5", self < 1e-5f && other > 0.5f);
    }
}

int main(int argc, char** argv)
{
    Synthetic();
    Real(argc > 1 ? argv[1] : "", argc > 2 ? argv[2] : "");
    std::printf("%s (%d failures)\n", g_fail ? "FAILED" : "ALL PASSED", g_fail);
    return g_fail ? 1 : 0;
}
