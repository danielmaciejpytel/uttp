// utpp_mm_chooser_matcher_test: ChooserMatcher (motion pose selection on .utppmm v2 + chooser) on synthetic scenarios.
// Usage: utpp_mm_chooser_matcher_test [--clip-map <sourcepack-clip-map.json>] [--query-ref <ref.txt>] <db.utppmm> [...]   (chooser = <db>.chooser.json next to the file;
// rig identity chosen from the file name: "_man_" -> man, else woman).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <string>
#include <tuple>
#include <vector>

#include "../src/mm/ChooserMatcher.hpp"

using namespace utpp::mm;
using namespace utpp::mm::motion;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...)                                                                                               \
    do                                                                                                                 \
    {                                                                                                                  \
        if (cond)                                                                                                      \
            ++g_pass;                                                                                                  \
        else                                                                                                           \
        {                                                                                                              \
            ++g_fail;                                                                                                  \
            std::printf("FAIL %s:%d: %s | ", __FILE__, __LINE__, #cond);                                              \
            std::printf(__VA_ARGS__);                                                                                  \
            std::printf("\n");                                                                                         \
        }                                                                                                              \
    } while (0)

namespace
{
constexpr float kDt = 1.0f / 30.0f;

// Character motion: planar velocity (m/s) and facing as functions of absolute time; z for jumps.
struct Motion
{
    virtual ~Motion() = default;
    virtual void At(double t, float& vx, float& vy, float& yaw, float& z) const = 0;
};

// Integrates a Motion into absolute positions on a fine grid, then builds trajectory samples around "now".
struct Sim
{
    const Motion* m = nullptr;
    double t0 = -3.0;
    std::vector<float> px, py, pyaw, pz; // grid every 1/120 s starting at t0
    void Build(double tEnd)
    {
        const double h = 1.0 / 120.0;
        const size_t n = static_cast<size_t>((tEnd - t0) / h) + 2;
        px.assign(n, 0.0f);
        py.assign(n, 0.0f);
        pyaw.assign(n, 0.0f);
        pz.assign(n, 0.0f);
        float x = 0, y = 0;
        for (size_t i = 0; i < n; ++i)
        {
            float vx, vy, yaw, z;
            m->At(t0 + i * h, vx, vy, yaw, z);
            px[i] = x;
            py[i] = y;
            pyaw[i] = yaw;
            pz[i] = z;
            x += vx * static_cast<float>(h);
            y += vy * static_cast<float>(h);
        }
    }
    TrajSample At(double t) const
    {
        const double h = 1.0 / 120.0;
        double fi = (t - t0) / h;
        fi = std::clamp(fi, 0.0, static_cast<double>(px.size() - 2));
        const size_t i = static_cast<size_t>(fi);
        const float u = static_cast<float>(fi - i);
        TrajSample s;
        s.x = px[i] + (px[i + 1] - px[i]) * u;
        s.y = py[i] + (py[i + 1] - py[i]) * u;
        s.z = pz[i] + (pz[i + 1] - pz[i]) * u;
        s.yaw = pyaw[i] + math::WrapAngle(pyaw[i + 1] - pyaw[i]) * u;
        return s;
    }
    // Samples every 1/60 s over [-1.0, +1.6] s relative to now (covers every schema sample time).
    void Traj(double now, std::vector<TrajSample>& out) const
    {
        out.clear();
        for (int k = -60; k <= 96; ++k)
        {
            TrajSample s = At(now + k / 60.0);
            s.t = k / 60.0f;
            out.push_back(s);
        }
    }
};

// Facing +Y at yaw 0; speed(t) along +Y unless reversed.
struct Straight : Motion
{
    float (*speed)(double) = nullptr;
    void At(double t, float& vx, float& vy, float& yaw, float& z) const override
    {
        vx = 0.0f;
        vy = speed(t);
        yaw = 0.0f;
        z = 0.0f;
    }
};

struct Pivot : Motion // walk +Y 1.6 m/s, at t >= tp walk -Y (velocity reversal over 0.25 s), facing follows
{
    double tp = 2.0;
    void At(double t, float& vx, float& vy, float& yaw, float& z) const override
    {
        vx = 0.0f;
        z = 0.0f;
        if (t < tp)
        {
            vy = 1.6f;
            yaw = 0.0f;
        }
        else
        {
            const float u = static_cast<float>(std::min(1.0, (t - tp) / 0.25));
            vy = 1.6f * (1.0f - 2.0f * u);
            yaw = u * math::kPi;
        }
    }
};

struct Jump : Motion // run 3.5 m/s along +Y, jump at tj with vz0 4 m/s (g 9.81), lands when z returns to 0
{
    double tj = 2.0;
    void At(double t, float& vx, float& vy, float& yaw, float& z) const override
    {
        vx = 0.0f;
        vy = 3.5f;
        yaw = 0.0f;
        const double a = t - tj;
        z = (a > 0.0) ? static_cast<float>(std::max(0.0, 4.0 * a - 0.5 * 9.81 * a * a)) : 0.0f;
    }
};

std::string MaskNames(const MotionDatabase& db, uint64_t m)
{
    std::string s;
    for (uint32_t d = 0; d < db.NumDatabases(); ++d)
        if ((m >> d) & 1u)
        {
            if (!s.empty())
                s += ",";
            s += std::string(db.DbName(d));
        }
    return s;
}

// Exact: some database of mask m is named one of aNames.
bool AnyExact(const MotionDatabase& db, uint64_t m, std::initializer_list<const char*> aNames)
{
    for (uint32_t d = 0; d < db.NumDatabases(); ++d)
        if ((m >> d) & 1u)
            for (const char* n : aNames)
                if (db.DbName(d) == n)
                    return true;
    return false;
}

// Exact DENT / DROW membership of a playing pose: the row lies in database d, its entry belongs to d and to the
// clip, and the row's frame is the reported frame inside the clip.
bool RowMember(const MotionDatabase& db, const MatchResult& r)
{
    if (r.database < 0 || r.database >= static_cast<int32_t>(db.NumDatabases()) || r.localRow < 0)
        return false;
    const DatabaseRecord& D = db.Db(static_cast<uint32_t>(r.database));
    if (static_cast<uint32_t>(r.localRow) >= D.numRows)
        return false;
    const RowRecord& row = db.Row(D.firstRow + static_cast<uint32_t>(r.localRow));
    if (row.entry != r.entry || row.frame != r.frame || r.entry < D.firstEntry || r.entry >= D.firstEntry + D.numEntries)
        return false;
    const EntryRecord& en = db.Entry(r.entry);
    if (en.database != static_cast<uint32_t>(r.database) || static_cast<int32_t>(en.clip) != r.clip)
        return false;
    const MotionClipRecord& cl = db.Clip(en.clip);
    return r.frame >= cl.firstFrame && r.frame < cl.firstFrame + cl.numFrames && ((cl.dbMask >> r.database) & 1u);
}

// Chooser databases of the update that selected the playing pose (kept across phases of one scenario).
uint64_t g_selMask = 0;

struct PhaseStats
{
    uint64_t maskUnion = 0;
    uint32_t updates = 0, jumps = 0, badJumpDb = 0, nonFinite = 0, historyHits = 0, noPose = 0;
    uint32_t notMember = 0, notChosen = 0;
    std::vector<std::pair<int32_t, int32_t>> picks; // (database, clip) of every jump
    std::vector<std::tuple<int32_t, float, bool>> clipClock; // clip, selected/continuing time, jump flag
    std::vector<double> us;
};

// Runs aSteps updates; aState(i) gives the chooser state per step. Checks per-step invariants.
template <class StateFn>
PhaseStats RunPhase(ChooserMatcher& mm, const MotionDatabase& db, const Sim& sim, double& now, int aSteps, StateFn aState)
{
    PhaseStats st;
    std::vector<TrajSample> traj;
    for (int i = 0; i < aSteps; ++i)
    {
        now += kDt;
        sim.Traj(now, traj);
        const ChooserState cs = aState(i, now);
        // reselection-history check is done against the state BEFORE the update (rows played within 0.3 s)
        std::vector<std::pair<uint32_t, uint32_t>> dummy;
        const auto t0 = std::chrono::high_resolution_clock::now();
        const MatchResult& r = mm.Update(kDt, traj.data(), static_cast<uint32_t>(traj.size()), cs);
        const auto t1 = std::chrono::high_resolution_clock::now();
        st.us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
        ++st.updates;
        st.maskUnion |= r.dbMask;
        if (r.database < 0)
        {
            ++st.noPose;
            continue;
        }
        if (!std::isfinite(r.cost))
            ++st.nonFinite;
        if (!RowMember(db, r))
            ++st.notMember;
        st.clipClock.emplace_back(r.clip, r.clipTime, r.jumped);
        if (r.jumped)
            g_selMask = r.dbMask; // the first pose after Reset() is always a jump
        if (!((g_selMask >> r.database) & 1u))
            ++st.notChosen;
        if (r.jumped)
        {
            st.picks.emplace_back(r.database, r.clip);
            ++st.jumps;
            if (!((r.dbMask >> r.database) & 1u))
                ++st.badJumpDb;
            const MotionClipRecord& cl = db.Clip(static_cast<uint32_t>(r.clip));
            if (!((cl.dbMask >> r.database) & 1u))
                ++st.badJumpDb;
        }
    }
    return st;
}

double Median(std::vector<double> v)
{
    if (v.empty())
        return 0.0;
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

float SpeedIdle(double) { return 0.0f; }
float SpeedWalkStart(double t) { return t < 2.0 ? 0.0f : static_cast<float>(std::min(1.6, (t - 2.0) * 4.0)); }
float SpeedWalkStop(double t) { return t < 2.0 ? 1.6f : static_cast<float>(std::max(0.0, 1.6 - (t - 2.0) * 5.0)); }
float SpeedRun(double) { return 3.5f; }

ChooserState Ground(int moving, int gait, float speed)
{
    ChooserState s;
    s.movementMode = 0;
    s.stance = 0;
    s.movementState = moving;
    s.gait = gait;
    s.speed2dCmS = speed * 100.0f;
    s.timeToLand = 1.5f;
    return s;
}

// Clip map (sourcepack-clip-map.json) text: true when the record named aClip lists aDb in
// databases.dense ("clips") or in its families list ("female_clips", assigned female clips).
bool ClipMapDense(const std::string& aMap, const std::string& aClip, const std::string& aDb)
{
    size_t p = aMap.find("\"clips\": [");
    const std::string key = "\"name\": \"" + aClip + "\"";
    while (p != std::string::npos)
    {
        p = aMap.find(key, p);
        if (p == std::string::npos)
            return false;
        const size_t next = aMap.find("\"name\": \"", p + key.size());
        const size_t lst = aMap.find("\"families\": [", p); // female_clips: assigned motion databases
        if (lst != std::string::npos && (next == std::string::npos || lst < next))
        {
            const size_t le = aMap.find(']', lst);
            const size_t hit = aMap.find("\"" + aDb + "\"", lst);
            if (hit != std::string::npos && hit < le)
                return true;
        }
        const size_t dbs = aMap.find("\"databases\": {", p);
        if (dbs != std::string::npos && (next == std::string::npos || dbs < next))
        {
            const size_t dn = aMap.find("\"dense\": [", dbs);
            const size_t de = dn == std::string::npos ? std::string::npos : aMap.find(']', dn);
            const size_t hit = de == std::string::npos ? std::string::npos : aMap.find("\"" + aDb + "\"", dn);
            if (hit != std::string::npos && hit < de)
                return true;
        }
        p += key.size();
    }
    return false;
}

std::vector<std::string> Tokens(std::istream& f)
{
    std::vector<std::string> v;
    std::string t;
    while (f >> t)
        v.push_back(t);
    return v;
}

// Query parity against the baker (tools/mmbake/dump_query_reference.py): rebuild each stored DFEA row from the
// raw trajectory / pose-history inputs with ChooserMatcher::BuildQuery and require |query - stored| <= 1e-4 on
// every trajectory dimension and every pose dimension the history covers (t_sample, t_origin <= 0).
void QueryParity(ChooserMatcher& mm, const MotionDatabase& db, const char* aRef)
{
    std::ifstream f(aRef, std::ios::binary);
    CHECK(f.good(), "query ref %s missing", aRef);
    if (!f.good())
        return;
    const std::vector<std::string> T = Tokens(f);
    size_t i = 0;
    auto tok = [&]() -> const std::string& {
        static const std::string empty;
        return i < T.size() ? T[i++] : empty;
    };
    auto num = [&]() { return std::strtod(tok().c_str(), nullptr); };
    CHECK(tok() == "utpp_query_ref" && num() == 1.0, "query ref header");
    tok();
    const int cases = static_cast<int>(num());
    CHECK(cases >= 10, "query ref: %d cases", cases);
    float worst = 0.0f, worstFlip = 0.0f;
    uint32_t dimsCompared = 0, turnCases = 0, flipCaught = 0;
    for (int c = 0; c < cases; ++c)
    {
        tok();
        const uint32_t d = static_cast<uint32_t>(num());
        const uint32_t row = static_cast<uint32_t>(num());
        num();
        num();
        const std::string clipName = tok();
        tok();
        std::vector<TrajSample> traj(static_cast<size_t>(num()));
        for (auto& s : traj)
        {
            s.t = static_cast<float>(num());
            s.x = static_cast<float>(num());
            s.y = static_cast<float>(num());
            s.z = static_cast<float>(num());
            s.yaw = static_cast<float>(num());
        }
        tok();
        static PoseHistory H;
        H = PoseHistory{};
        H.numSamples = static_cast<uint32_t>(num());
        H.numBones = static_cast<uint32_t>(num());
        CHECK(H.numSamples <= kHistMaxSamples && H.numBones <= kHistMaxBones, "query ref: history too large");
        if (H.numSamples > kHistMaxSamples || H.numBones > kHistMaxBones)
            return;
        for (uint32_t k = 0; k < H.numBones; ++k)
            H.bone[k] = static_cast<int32_t>(num());
        for (uint32_t s = 0; s < H.numSamples; ++s)
        {
            TrajSample& r = H.root[s];
            r.t = static_cast<float>(num());
            r.x = static_cast<float>(num());
            r.y = static_cast<float>(num());
            r.z = static_cast<float>(num());
            r.yaw = static_cast<float>(num());
            for (uint32_t k = 0; k < H.numBones; ++k)
            {
                H.gp[s][k] = math::Vec3{static_cast<float>(num()), static_cast<float>(num()), static_cast<float>(num())};
                math::Quat q;
                q.x = static_cast<float>(num());
                q.y = static_cast<float>(num());
                q.z = static_cast<float>(num());
                q.w = static_cast<float>(num());
                H.gq[s][k] = q;
            }
        }
        tok();
        std::vector<float> feat(static_cast<size_t>(num()));
        for (auto& v : feat)
            v = static_cast<float>(num());
        if (d >= db.NumDatabases() || row >= db.Db(d).numRows || feat.size() != db.Db(d).dims)
        {
            CHECK(false, "query ref case %d: database / row / dims mismatch", c);
            continue;
        }
        const float* stored = db.Features(d, row);
        float fileDiff = 0.0f;
        for (size_t k = 0; k < feat.size(); ++k)
            fileDiff = std::max(fileDiff, std::fabs(stored[k] - feat[k]));
        CHECK(fileDiff <= 1e-6f, "query ref case %d: dumped row differs from the loaded DFEA by %g", c, fileDiff);

        const SchemaRecord& S = db.Schema(db.Db(d).schema);
        auto compare = [&](float& aWorst, std::string& aWhere) {
            mm.Reset();
            mm.SetQueryInputs(traj.data(), static_cast<uint32_t>(traj.size()), &H);
            mm.BuildQuery(d, false);
            const float* q = mm.NormalizedQuery();
            uint32_t n = 0;
            aWorst = 0.0f;
            for (uint32_t ci = S.firstChannel; ci < S.firstChannel + S.numChannels; ++ci)
            {
                const ChannelRecord& ch = db.Channel(ci);
                const bool isTraj = (ch.flags & kChanTrajectory) != 0;
                if (!isTraj && (ch.tSample > 1e-6f || ch.tOrigin > 1e-6f))
                    continue;
                for (uint32_t k = 0; k < ch.cardinality; ++k)
                {
                    const uint32_t o = ch.dataOffset + k;
                    const float e = std::fabs(q[o] - stored[o]);
                    if (e > aWorst)
                    {
                        aWorst = e;
                        aWhere = std::string(db.String(ch.labelOff)) + "[" + std::to_string(k) + "]";
                    }
                    ++n;
                }
            }
            return n;
        };
        std::string where;
        float err = 0.0f;
        const uint32_t n = compare(err, where);
        dimsCompared += n;
        worst = std::max(worst, err);
        std::printf("  query parity %-36s row %6u %-44s dims %2u/%2u max |q - DFEA| %.2e (%s)\n",
                    std::string(db.DbName(d)).c_str(), row, clipName.c_str(), n, S.dims, err, where.c_str());
        CHECK(err <= 1e-4f, "query parity %s row %u: max error %g at %s", clipName.c_str(), row, err, where.c_str());
        // yaw-sign negative control on turning rows: mirrored yaw must break parity
        float ymin = traj[0].yaw, ymax = traj[0].yaw;
        for (const auto& s : traj)
        {
            ymin = std::min(ymin, s.yaw);
            ymax = std::max(ymax, s.yaw);
        }
        if (ymax - ymin > 0.35f)
        {
            ++turnCases;
            for (auto& s : traj)
                s.yaw = -s.yaw;
            for (uint32_t s = 0; s < H.numSamples; ++s)
                H.root[s].yaw = -H.root[s].yaw;
            float e2 = 0.0f;
            std::string w2;
            compare(e2, w2);
            worstFlip = std::max(worstFlip, e2);
            if (e2 > 0.05f)
                ++flipCaught;
            std::printf("    mirrored yaw: max |q - DFEA| %.3f (%s)\n", e2, w2.c_str());
        }
    }
    std::printf("  query parity: %d rows, %u dims compared, max error %.2e; mirrored-yaw rows %u, detected %u\n", cases,
                dimsCompared, worst, turnCases, flipCaught);
    CHECK(turnCases >= 1 && flipCaught == turnCases, "mirrored yaw not detected on %u of %u turning rows",
          turnCases - flipCaught, turnCases);
    mm.Reset();
}

// ---- continuing-pose rules of UE 5.8 UPoseSearchLibrary::MotionMatch (ChooserMatcher, commit c3cff14).
// UE 5.8 sources (Engine/Plugins/Animation/PoseSearch/Source/Runtime/Private):
// PSL = PoseSearchLibrary.cpp, PSD = PoseSearchDatabase.cpp. motion = the sandbox character animation blueprint export.
constexpr float kNoCost = std::numeric_limits<float>::max();
const Interrupt kModes[5] = {Interrupt::DoNotInterrupt, Interrupt::InterruptOnDatabaseChange,
                             Interrupt::InterruptOnDatabaseChangeAndInvalidateContinuingPose, Interrupt::ForceInterrupt,
                             Interrupt::ForceInterruptAndInvalidateContinuingPose};
const char* const kModeNames[5] = {"DoNotInterrupt", "InterruptOnDatabaseChange", "InterruptOnDatabaseChangeAndInvalidate",
                                   "ForceInterrupt", "ForceInterruptAndInvalidate"};
enum
{
    kDNI,
    kIOD,
    kIODInv,
    kFI,
    kFIInv
};

bool Near(float a, float b) { return std::fabs(a - b) <= 1e-4f * std::max(1.0f, std::fabs(b)); }
float SpeedWalk(double) { return 1.6f; }

// Local row of global frame aFrame of clip aClip in database d: first entry of the clip holding a row at that frame
// (linear scan, independent of ChooserMatcher::FindRow); -1 = not indexed.
int32_t RowOf(const MotionDatabase& db, uint32_t d, int32_t aClip, uint32_t aFrame, uint32_t* aEntry = nullptr)
{
    const DatabaseRecord& D = db.Db(d);
    for (uint32_t e = D.firstEntry; e < D.firstEntry + D.numEntries; ++e)
    {
        const EntryRecord& en = db.Entry(e);
        if (static_cast<int32_t>(en.clip) != aClip)
            continue;
        for (uint32_t r = en.firstRow; r < en.firstRow + en.numRows; ++r)
            if (db.Row(r).frame == aFrame)
            {
                if (aEntry)
                    *aEntry = e;
                return static_cast<int32_t>(r - D.firstRow);
            }
    }
    return -1;
}

struct Cand
{
    uint32_t d = 0;
    int32_t row = -1;
    uint32_t entry = 0;
    bool searched = false;
};

// Continuing pose searches UE builds for the playing pose P: PSL 276-300 PopulateContinuingPoseSearches (the playing
// database, then every database to search indexing the same clip at P's frame); a search is dropped when invalidated
// (PSL 346-365 IsInvalidatingContinuingPose: *AndInvalidate && !Contains, ForceInterruptAndInvalidate always) and its
// cost is evaluated unless force-interrupted (PSL 326-344 IsForceInterrupt: DoNotInterrupt never,
// InterruptOnDatabaseChange* when !Contains, ForceInterrupt* always; PSL 1340-1373).
std::vector<Cand> ExpectedCands(const MotionDatabase& db, const MatchResult& P, uint64_t aMask, Interrupt m)
{
    std::vector<Cand> v;
    const auto add = [&](uint32_t d) {
        for (const Cand& c : v)
            if (c.d == d)
                return;
        const bool in = (aMask >> d) & 1u;
        const bool invalidated = (m == Interrupt::InterruptOnDatabaseChangeAndInvalidateContinuingPose && !in) ||
                                 m == Interrupt::ForceInterruptAndInvalidateContinuingPose;
        if (invalidated)
            return;
        Cand c;
        c.d = d;
        c.row = RowOf(db, d, P.clip, P.frame, &c.entry);
        if (c.row < 0)
            return;
        const bool forced = m == Interrupt::ForceInterrupt || m == Interrupt::ForceInterruptAndInvalidateContinuingPose ||
                            ((m == Interrupt::InterruptOnDatabaseChange ||
                              m == Interrupt::InterruptOnDatabaseChangeAndInvalidateContinuingPose) && !in);
        c.searched = !forced;
        v.push_back(c);
    };
    if (P.database >= 0)
        add(static_cast<uint32_t>(P.database));
    for (uint32_t d = 0; d < db.NumDatabases(); ++d)
        if ((aMask >> d) & 1u)
            add(d);
    return v;
}

// Expected normalized query of database d: trajectory channels from traj; pose channels (input query 0) from the first
// cached continuing pose with d's schema, else from the pose history (none here -> mean, 0 after normalization).
std::vector<float> ExpectedQuery(const ChooserMatcher& aMm, const MotionDatabase& db, uint32_t d,
                                 const std::vector<TrajSample>& traj, const std::vector<Cand>& cands)
{
    ChooserMatcher x = aMm;
    x.Reset();
    x.SetQueryInputs(traj.data(), static_cast<uint32_t>(traj.size()), nullptr);
    x.BuildQuery(d, false);
    const DatabaseRecord& D = db.Db(d);
    std::vector<float> q(x.NormalizedQuery(), x.NormalizedQuery() + D.dims);
    for (const Cand& c : cands)
    {
        const DatabaseRecord& C = db.Db(c.d);
        if (C.schema != D.schema)
            continue;
        const float* f = db.Features(c.d, static_cast<uint32_t>(c.row));
        const float* cm = db.NormMean(C.norm);
        const float* cs = db.NormScale(C.norm);
        const float* m = db.NormMean(D.norm);
        const float* s = db.NormScale(D.norm);
        const SchemaRecord& S = db.Schema(D.schema);
        for (uint32_t ci = S.firstChannel; ci < S.firstChannel + S.numChannels; ++ci)
        {
            const ChannelRecord& ch = db.Channel(ci);
            if ((ch.flags & kChanTrajectory) || ch.inputQuery != 0)
                continue;
            for (uint32_t k = 0; k < ch.cardinality; ++k)
            {
                const uint32_t o = ch.dataOffset + k;
                q[o] = ((f[o] / cs[o] + cm[o]) - m[o]) * s[o];
            }
        }
        break;
    }
    return q;
}

// Non-selectable poses of database d: rows in the pose reselection history (PSL 952-956 PoseIndicesHistory.Update with
// PoseReselectHistory, PSD 1630-1641) and, when d holds a cached continuing pose of a DISABLE_RESELECTION entry, every
// pose of every index asset of the same source asset in d (PSD 1557-1580).
std::vector<uint8_t> ExpectedExcluded(const ChooserMatcher& pre, const MotionDatabase& db, uint32_t d,
                                      const std::vector<Cand>& cands, int32_t aClip)
{
    const DatabaseRecord& D = db.Db(d);
    std::vector<uint8_t> ex(D.numRows, 0);
    for (uint32_t r = 0; r < D.numRows; ++r)
        ex[r] = pre.InHistory(d, r) ? 1 : 0;
    // Stance-entry admission (f973894, 2026-10-19, ChooserMatcher::UpdateStanceAdmission / MarkExcluded): the stand<->crouch entry
    // clips of an idle database are legal NEW candidates only inside the 0.6 s window after a real stance edge seen with dt > 0
    // ("a zero-time observation may record an edge but cannot select/consume a new player"). Every query of this oracle comes from
    // Probe(), which updates a copy with dt = 0: the window is never open, so those rows are non-selectable. Before f973894 the
    // oracle expected the transition start itself (e.g. rows 301/511). Already playing transitions are not affected (continuing
    // pose path); the positive path (a real edge plays the transition exactly once) is pinned by the transitionJumps checks below.
    // Mutation UTPP_MOTION_SNAP_MUTATION=5 (entry clips always admitted) therefore fails the crouch/idle probes.
    for (uint32_t e = D.firstEntry; e < D.firstEntry + D.numEntries; ++e)
    {
        const EntryRecord& en = db.Entry(e);
        if (steer::ClassifyClip(db.ClipName(en.clip)) != steer::kClassStanceTransition)
            continue;
        for (uint32_t r = en.firstRow; r < en.firstRow + en.numRows; ++r)
            ex[r - D.firstRow] = 1;
    }
    for (const Cand& c : cands)
        if (c.d == d && (db.Entry(c.entry).flags & kEntryDisableReselection))
            for (uint32_t e = D.firstEntry; e < D.firstEntry + D.numEntries; ++e)
            {
                const EntryRecord& en = db.Entry(e);
                if (static_cast<int32_t>(en.clip) != aClip)
                    continue;
                for (uint32_t r = en.firstRow; r < en.firstRow + en.numRows; ++r)
                    ex[r - D.firstRow] = 1;
            }
    return ex;
}

struct Pick
{
    int32_t d = -1;
    int32_t row = -1;
    float cost = kNoCost;
};

// Exact (linear) search of every database to search with the expected queries and non-selectable poses.
Pick ExpectedSearch(const ChooserMatcher& pre, const MotionDatabase& db, uint64_t aMask, const std::vector<TrajSample>& traj,
                    const std::vector<Cand>& cands, int32_t aClip)
{
    Pick best;
    for (uint32_t d = 0; d < db.NumDatabases(); ++d)
    {
        if (!((aMask >> d) & 1u))
            continue;
        const std::vector<uint8_t> ex = ExpectedExcluded(pre, db, d, cands, aClip);
        const std::vector<float> q = ExpectedQuery(pre, db, d, traj, cands);
        const MotionSearchResult r = db.SearchLinear(d, q.data(), ex.data());
        if (r.localRow >= 0 && r.cost < best.cost)
            best = Pick{static_cast<int32_t>(d), r.localRow, r.cost};
    }
    return best;
}

// One update of a copy of mm with aDt = 0 (the continuing pose stays on P's frame, the history clock does not move).
MatchResult Probe(const ChooserMatcher& mm, const std::vector<TrajSample>& traj, const ChooserState& s, int aMode)
{
    ChooserMatcher x = mm;
    return x.Update(0.0f, traj.data(), static_cast<uint32_t>(traj.size()), s, nullptr, &kModes[aMode]);
}

// Checks one probe result r of the pre-update matcher pre (playing pose P) against the UE rules recomputed
// independently: continuing cost (cost + cost addend + continuing addend of every evaluated continuing search, query
// built from the cached continuing pose), exact search over the databases to search, continuing pose first (a search
// result must be strictly cheaper), winner not a non-selectable pose.
struct Verdict
{
    std::vector<Cand> cands;
    float expCont = kNoCost;
    int32_t contD = -1, contRow = -1;
    Pick search;
    bool expContinuing = false;
};
Verdict VerifyProbe(const ChooserMatcher& pre, const MotionDatabase& db, const std::vector<TrajSample>& traj,
                    const MatchResult& P, const MatchResult& r, int aMode, const char* aLabel)
{
    Verdict v;
    v.cands = ExpectedCands(db, P, r.dbMask, kModes[aMode]);
    for (const Cand& c : v.cands)
    {
        if (!c.searched)
            continue;
        const std::vector<float> q = ExpectedQuery(pre, db, c.d, traj, v.cands);
        const RowRecord& rr = db.Row(db.Db(c.d).firstRow + static_cast<uint32_t>(c.row));
        const float cost = db.RowCost(c.d, static_cast<uint32_t>(c.row), q.data()) + rr.costAddend + rr.continuingAddend;
        if (cost < v.expCont)
        {
            v.expCont = cost;
            v.contD = static_cast<int32_t>(c.d);
            v.contRow = c.row;
        }
    }
    v.search = ExpectedSearch(pre, db, r.dbMask, traj, v.cands, P.clip);
    v.expContinuing = v.contD >= 0 && !(v.search.cost < v.expCont);
    const char* mn = kModeNames[aMode];
    CHECK((r.continuingCost == kNoCost) == (v.expCont == kNoCost) && (v.expCont == kNoCost || Near(r.continuingCost, v.expCont)),
          "%s %s: continuing cost %g, expected %g", aLabel, mn, r.continuingCost, v.expCont);
    if (v.expContinuing)
        CHECK(!r.jumped && r.database == v.contD && r.localRow == v.contRow &&
                  r.continuingOutsideSet == !((r.dbMask >> v.contD) & 1u) && Near(r.cost, v.expCont),
              "%s %s: expected the continuing pose db %d row %d, got jumped %d db %d row %d outside %d", aLabel, mn, v.contD,
              v.contRow, static_cast<int>(r.jumped), r.database, r.localRow, static_cast<int>(r.continuingOutsideSet));
    else if (v.search.d >= 0)
    {
        CHECK(r.jumped && r.database >= 0 && ((r.dbMask >> r.database) & 1u) && !r.continuingOutsideSet &&
                  Near(r.cost, v.search.cost),
              "%s %s: expected a jump of cost %g (db %d row %d), got jumped %d db %d row %d cost %g", aLabel, mn, v.search.cost,
              v.search.d, v.search.row, static_cast<int>(r.jumped), r.database, r.localRow, r.cost);
        if (r.database >= 0 && r.localRow >= 0)
        {
            const std::vector<uint8_t> ex = ExpectedExcluded(pre, db, static_cast<uint32_t>(r.database), v.cands, P.clip);
            CHECK(!ex[static_cast<size_t>(r.localRow)], "%s %s: jump onto a non-selectable pose (db %d row %d)", aLabel, mn,
                  r.database, r.localRow);
        }
    }
    else
        CHECK(r.database < 0, "%s %s: no result expected, got db %d", aLabel, mn, r.database);
    return v;
}

// Cost of row (d, row) for query q including its cost addend.
float RowTotal(const MotionDatabase& db, uint32_t d, int32_t aRow, const std::vector<float>& q)
{
    const RowRecord& rr = db.Row(db.Db(d).firstRow + static_cast<uint32_t>(aRow));
    return db.RowCost(d, static_cast<uint32_t>(aRow), q.data()) + rr.costAddend;
}

std::vector<MatchResult> Prime(ChooserMatcher& mm, const Sim& sim, double& now, int aSteps, const ChooserState& s,
                               std::vector<TrajSample>& traj)
{
    std::vector<MatchResult> v;
    mm.Reset();
    for (int i = 0; i < aSteps; ++i)
    {
        now += kDt;
        sim.Traj(now, traj);
        v.push_back(mm.Update(kDt, traj.data(), static_cast<uint32_t>(traj.size()), s));
    }
    return v;
}

void ContinuingRules(ChooserMatcher& mm, const MotionDatabase& db)
{
    std::vector<TrajSample> traj;
    double now = 0.0;
    Straight idleM;
    idleM.speed = SpeedIdle;
    Sim idleSim;
    idleSim.m = &idleM;
    idleSim.Build(10.0);
    const ChooserState idle = Ground(0, 0, 0.0f), walk = Ground(1, 0, 1.6f);
    const std::vector<MatchResult> played = Prime(mm, idleSim, now, 40, idle, traj);
    const MatchResult P = mm.Last();
    CHECK(P.database >= 0 && P.localRow >= 0, "rules: idle prime without a pose");
    if (P.database < 0)
        return;
    const uint32_t C = static_cast<uint32_t>(P.database);
    const bool flaggedC = (db.Entry(P.entry).flags & kEntryDisableReselection) != 0;
    // crouch idle: the Stand Idles database leaves the set, Crouch Idles (same schema) enters it -> the query source of
    // the jump target is observable (a query is built from a continuing pose of the same schema only)
    ChooserState crouch = idle;
    crouch.stance = 1;
    MatchResult rw[5], ri[5], rc[5];
    Verdict vi[5]; // oracle verdict of the idle-set probes (used by rule (d))
    for (int m = 0; m < 5; ++m)
    {
        rw[m] = Probe(mm, traj, walk, m);
        ri[m] = Probe(mm, traj, idle, m);
        rc[m] = Probe(mm, traj, crouch, m);
        VerifyProbe(mm, db, traj, P, rw[m], m, "idle pose, walk set");
        vi[m] = VerifyProbe(mm, db, traj, P, ri[m], m, "idle pose, idle set");
        VerifyProbe(mm, db, traj, P, rc[m], m, "idle pose, crouch set");
    }
    {
        // Non-vacuity of the stance-entry rule in the oracle: the idle/crouch sets really hold stand<->crouch entry rows.
        uint32_t stanceRows = 0;
        for (uint32_t d = 0; d < db.NumDatabases(); ++d)
            if (((rc[kDNI].dbMask | ri[kDNI].dbMask) >> d) & 1u)
                for (uint32_t e = db.Db(d).firstEntry; e < db.Db(d).firstEntry + db.Db(d).numEntries; ++e)
                    if (steer::ClassifyClip(db.ClipName(db.Entry(e).clip)) == steer::kClassStanceTransition)
                        stanceRows += db.Entry(e).numRows;
        CHECK(stanceRows > 0, "precondition: the idle/crouch sets hold no stand<->crouch entry rows (stance-entry oracle rule untested)");
    }
    const uint64_t maskW = rw[kDNI].dbMask, maskI = ri[kDNI].dbMask, maskC = rc[kDNI].dbMask;
    std::printf("  rules: crouch set %s; IOD pick %s\n", MaskNames(db, maskC).c_str(),
                rc[kIOD].database >= 0 ? std::string(db.DbName(static_cast<uint32_t>(rc[kIOD].database))).c_str() : "-");
    CHECK(!((maskC >> C) & 1u) && (db.Clip(static_cast<uint32_t>(P.clip)).dbMask & maskC) == 0,
          "precondition: the crouch set must not index the idle clip (set %s)", MaskNames(db, maskC).c_str());
    std::printf("  rules: continuing %s frame %u in %s (DISABLE_RESELECTION %d, schema %u); walk set %s\n",
                std::string(db.ClipName(static_cast<uint32_t>(P.clip))).c_str(), P.frame, std::string(db.DbName(C)).c_str(),
                static_cast<int>(flaggedC), db.Db(C).schema, MaskNames(db, maskW).c_str());
    for (int m = 0; m < 5; ++m)
        std::printf("    %-40s walk set: jumped %d db %-34s cont %.4g cost %.4g | idle set: jumped %d cont %.4g cost %.4g\n",
                    kModeNames[m], static_cast<int>(rw[m].jumped),
                    rw[m].database >= 0 ? std::string(db.DbName(static_cast<uint32_t>(rw[m].database))).c_str() : "-",
                    rw[m].continuingCost == kNoCost ? -1.0 : rw[m].continuingCost, rw[m].cost, static_cast<int>(ri[m].jumped),
                    ri[m].continuingCost == kNoCost ? -1.0 : ri[m].continuingCost, ri[m].cost);
    CHECK(!((maskW >> C) & 1u) && (db.Clip(static_cast<uint32_t>(P.clip)).dbMask & maskW) == 0,
          "precondition: the walk set must not index the idle clip (set %s)", MaskNames(db, maskW).c_str());
    CHECK((maskI >> C) & 1u, "precondition: the idle set holds the playing idle database");

    // (a) PSL 330-331 IsForceInterrupt(DoNotInterrupt) = false + PSL 1340-1373: the continuing pose of a database that
    // left the set is still evaluated and, cheaper than every search result, stays selected (outside the set).
    CHECK(rw[kDNI].continuingCost < kNoCost && !rw[kDNI].jumped && rw[kDNI].database == static_cast<int32_t>(C) &&
              rw[kDNI].localRow == P.localRow && rw[kDNI].continuingOutsideSet,
          "(a) DoNotInterrupt: continuing pose outside the set not kept (cont %g jumped %d db %d outside %d)",
          rw[kDNI].continuingCost, static_cast<int>(rw[kDNI].jumped), rw[kDNI].database,
          static_cast<int>(rw[kDNI].continuingOutsideSet));

    // query of the winning database built from the continuing pose vs from the pose history (no history -> mean)
    const auto costs = [&](const MatchResult& r, float& aCont, float& aHist) {
        aCont = aHist = kNoCost;
        if (r.database < 0)
            return;
        const uint32_t d = static_cast<uint32_t>(r.database);
        const std::vector<Cand> cc = {Cand{C, P.localRow, P.entry, false}};
        aCont = RowTotal(db, d, r.localRow, ExpectedQuery(mm, db, d, traj, cc));
        aHist = RowTotal(db, d, r.localRow, ExpectedQuery(mm, db, d, traj, {}));
    };
    // (b) PSL 333-335 IsForceInterrupt: InterruptOnDatabaseChange && !Contains -> force interrupt (not evaluated, not selectable);
    // PSL 346-365: not invalidated -> the cached continuing pose still builds the query (PSL 1340-1373).
    {
        for (const MatchResult* pr : {&rw[kIOD], &rc[kIOD]})
        {
            const MatchResult& r = *pr;
            const uint64_t mask = pr == &rw[kIOD] ? maskW : maskC;
            CHECK(r.continuingCost == kNoCost && r.jumped && r.database >= 0 && ((mask >> r.database) & 1u) &&
                      !r.continuingOutsideSet,
                  "(b) InterruptOnDatabaseChange: continuing pose outside the set evaluated or kept (cont %g jumped %d)",
                  r.continuingCost, static_cast<int>(r.jumped));
        }
        const MatchResult& r = rc[kIOD];
        float qc, qh;
        costs(r, qc, qh);
        const bool sameSchema = r.database >= 0 && db.Db(static_cast<uint32_t>(r.database)).schema == db.Db(C).schema;
        CHECK(sameSchema && Near(r.cost, qc) && !Near(r.cost, qh),
              "(b) InterruptOnDatabaseChange: cost %g, from the continuing-pose query %g, from the history query %g (same schema %d)",
              r.cost, qc, qh, static_cast<int>(sameSchema));
        CHECK(r.database == rc[kFI].database && r.localRow == rc[kFI].localRow && Near(r.cost, rc[kFI].cost),
              "(b) outside the set InterruptOnDatabaseChange == ForceInterrupt");
        CHECK(rw[kDNI].continuingCost <= rw[kIOD].cost, "(b) the kept continuing pose (%g) must be cheaper than the jump (%g)",
              rw[kDNI].continuingCost, rw[kIOD].cost);
    }
    // (c) PSL 346-365: *AndInvalidateContinuingPose -> no continuing search, query from the pose history; IODAndInvalidate
    // invalidates only when the database left the set (in the set it is evaluated like DoNotInterrupt), FIAndInvalidate always.
    {
        const MatchResult& r = rc[kIODInv];
        float qc, qh;
        costs(r, qc, qh);
        CHECK(r.continuingCost == kNoCost && r.jumped && Near(r.cost, qh) && !Near(r.cost, qc),
              "(c) IODAndInvalidate outside the set: cont %g jumped %d cost %g, history query %g, continuing query %g",
              r.continuingCost, static_cast<int>(r.jumped), r.cost, qh, qc);
        CHECK(ri[kIODInv].continuingCost < kNoCost && Near(ri[kIODInv].continuingCost, ri[kDNI].continuingCost),
              "(c) IODAndInvalidate in the set: continuing cost %g, DoNotInterrupt %g", ri[kIODInv].continuingCost,
              ri[kDNI].continuingCost);
        const MatchResult& f = ri[kFIInv];
        costs(f, qc, qh);
        CHECK(f.continuingCost == kNoCost && f.jumped && Near(f.cost, qh) && !Near(f.cost, qc),
              "(c) FIAndInvalidate in the set: cont %g jumped %d cost %g, history query %g, continuing query %g",
              f.continuingCost, static_cast<int>(f.jumped), f.cost, qh, qc);
        CHECK(rc[kFIInv].database == rc[kIODInv].database && rc[kFIInv].localRow == rc[kIODInv].localRow &&
                  rw[kFIInv].database == rw[kIODInv].database && rw[kFIInv].localRow == rw[kIODInv].localRow,
              "(c) outside the set FIAndInvalidate == IODAndInvalidate");
    }
    // (d) PSL 337-339 IsForceInterrupt(ForceInterrupt*) = true: never evaluated, in or outside the set, every update;
    // the cached continuing pose still builds the query.
    {
        float qc, qh;
        costs(ri[kFI], qc, qh);
        CHECK(!ri[kDNI].jumped && ri[kDNI].continuingCost < kNoCost, "(d) control: DoNotInterrupt keeps the idle pose");
        // ForceInterrupt never evaluates the continuing pose and, in the set or outside it, always takes the search result. Since
        // f973894 the stand<->crouch entry clips are not selectable at dt 0, so an idle set can hold no selectable row at all
        // (man: only the DISABLE_RESELECTION stand idle remains): the search result is then empty (database -1). The exact
        // search of the oracle (vi) decides which of the two is expected; the continuing pose is never kept either way.
        const bool expIdleJump = vi[kFI].search.d >= 0;
        CHECK(ri[kFI].continuingCost == kNoCost && ri[kFI].jumped == expIdleJump && (expIdleJump || ri[kFI].database < 0) &&
                  rw[kFI].continuingCost == kNoCost && rw[kFI].jumped,
              "(d) ForceInterrupt evaluated the continuing pose (in set %g, outside %g; jumped %d, oracle expects a jump %d)",
              ri[kFI].continuingCost, rw[kFI].continuingCost, static_cast<int>(ri[kFI].jumped), static_cast<int>(expIdleJump));
        CHECK(!expIdleJump || Near(ri[kFI].cost, qc), "(d) ForceInterrupt query from the continuing pose: cost %g, expected %g",
              ri[kFI].cost, qc);
        ChooserMatcher x = mm;
        std::vector<TrajSample> t2;
        double n2 = now;
        uint32_t evaluated = 0, kept = 0, empty = 0;
        for (int i = 0; i < 15; ++i)
        {
            n2 += kDt;
            idleSim.Traj(n2, t2);
            const MatchResult& r = x.Update(kDt, t2.data(), static_cast<uint32_t>(t2.size()), idle, nullptr, &kModes[kFI]);
            evaluated += r.continuingCost != kNoCost ? 1u : 0u;
            // "kept" = the continuing pose stays selected (a non-jump result that still carries a pose).
            kept += (!r.jumped && r.database >= 0) ? 1u : 0u;
            // "empty" = no result at all (database -1). Legal only where the oracle has nothing to pick in this set (expIdleJump false:
            // the man's idle set, see above); where it has candidates (expIdleJump: the woman's idle set: many rows, of which the
            // 0.3 s pose history excludes only a few) every ForceInterrupt update at dt > 0 must jump onto a pose.
            empty += r.database < 0 ? 1u : 0u;
        }
        CHECK(evaluated == 0 && kept == 0 && (!expIdleJump || empty == 0),
              "(d) 15 ForceInterrupt updates: %u evaluated, %u kept, %u without a result (oracle expects a jump on every update: %d)",
              evaluated, kept, empty, static_cast<int>(expIdleJump));
    }
    // (e) PoseReselectHistory 0.3 s (motion node CDO; PSL 952-956, PSD 1630-1641): rows of the last 0.3 s excluded,
    // older ones selectable again.
    uint32_t eChecked = 0;
    {
        const size_t n = played.size();
        uint32_t bad = 0, old = 0;
        for (size_t i = 0; i < n; ++i)
        {
            const MatchResult& h = played[i];
            const size_t age = n - 1 - i; // updates ago (1/30 s)
            const bool in = mm.InHistory(static_cast<uint32_t>(h.database), static_cast<uint32_t>(h.localRow));
            if (age <= 8)
                bad += in ? 0u : 1u;
            else if (age >= 11)
            {
                bool replayed = false;
                for (size_t j = n - 10; j < n; ++j)
                    replayed |= played[j].database == h.database && played[j].localRow == h.localRow;
                if (!replayed)
                {
                    ++old;
                    bad += in ? 1u : 0u;
                }
            }
        }
        CHECK(bad == 0 && old > 0, "(e) reselection history: %u wrong, %u rows older than 0.3 s", bad, old);
    }
    // (e) DISABLE_RESELECTION (PSD 1557-1580): in the database holding the continuing pose every pose of its clip is
    // non-selectable. Control: without that rule the best row of the idle database is a pose of the playing clip.
    const auto disableCheck = [&](const ChooserMatcher& pre, const std::vector<TrajSample>& t, const MatchResult& Q,
                                  const MatchResult& r, uint32_t d, const char* aLabel) {
        const std::vector<Cand> cands = ExpectedCands(db, Q, r.dbMask, Interrupt::ForceInterrupt);
        std::vector<uint8_t> exH(db.Db(d).numRows, 0);
        for (uint32_t k = 0; k < db.Db(d).numRows; ++k)
            exH[k] = pre.InHistory(d, k) ? 1 : 0;
        const std::vector<float> q = ExpectedQuery(pre, db, d, t, cands);
        const MotionSearchResult ctl = db.SearchLinear(d, q.data(), exH.data());
        const bool ctlSame = ctl.localRow >= 0 &&
                             static_cast<int32_t>(db.Entry(db.Row(db.Db(d).firstRow + static_cast<uint32_t>(ctl.localRow)).entry).clip) == Q.clip;
        uint32_t entries = 0;
        for (uint32_t e = db.Db(d).firstEntry; e < db.Db(d).firstEntry + db.Db(d).numEntries; ++e)
            entries += static_cast<int32_t>(db.Entry(e).clip) == Q.clip ? 1u : 0u;
        std::printf("    %s: DISABLE_RESELECTION clip %s in %s (%u entries): history-only best row of the clip %d, "
                    "ForceInterrupt pick %s\n",
                    aLabel, std::string(db.ClipName(static_cast<uint32_t>(Q.clip))).c_str(), std::string(db.DbName(d)).c_str(),
                    entries, static_cast<int>(ctlSame),
                    r.clip >= 0 ? std::string(db.ClipName(static_cast<uint32_t>(r.clip))).c_str() : "-");
        CHECK(!(r.database == static_cast<int32_t>(d) && r.clip == Q.clip), "(e) %s: ForceInterrupt jumped into the "
              "DISABLE_RESELECTION clip in %s", aLabel, std::string(db.DbName(d)).c_str());
        if (ctlSame)
            ++eChecked;
    };
    if (flaggedC)
        disableCheck(mm, traj, P, ri[kFI], C, "idle");

    // (f) PSL 276-300: the continuing clip is looked up in every database to search indexing it. A clip indexed in two
    // databases of the v2 file (TurnInPlace + Walk_Stops, Walk_Starts + Walk_SpinTransition, Run_Starts +
    // Run_SpinTransition, Walk_Starts + Walk_Stops) is played from one of them (turn in place, walk start), then a
    // chooser state whose set lacks the playing database but holds the other one is probed.
    {
        struct TurnLeft : Motion
        {
            void At(double t, float& vx, float& vy, float& yaw, float& z) const override
            {
                vx = vy = z = 0.0f;
                yaw = static_cast<float>(std::clamp((t - 2.0) / 0.5, 0.0, 1.0)) * 0.5f * math::kPi;
            }
        } turnM;
        Straight startM;
        startM.speed = SpeedWalkStart;
        ChooserState tip = idle, starting = Ground(1, 0, 0.8f), spin = walk, stop = Ground(0, 0, 1.0f),
                     stopSlow = Ground(0, 0, 0.4f);
        tip.shouldTurnInPlace = true;
        starting.isStarting = true;
        spin.shouldSpinTransition = true;
        const ChooserState probes[4] = {stop, stopSlow, spin, starting};
        const Motion* scnMotion[2] = {&turnM, &startM};
        const ChooserState scnState[2] = {tip, starting};
        bool found = false;
        for (int sc = 0; sc < 2 && !found; ++sc)
        {
            Sim ws;
            ws.m = scnMotion[sc];
            ws.Build(10.0);
            double n = 0.0;
            std::vector<TrajSample> t;
            Prime(mm, ws, n, 59, idle, t);
            for (int i = 0; i < 40 && !found; ++i)
            {
                n += kDt;
                ws.Traj(n, t);
                mm.Update(kDt, t.data(), static_cast<uint32_t>(t.size()), scnState[sc]);
                const MatchResult Q = mm.Last();
                for (int pi = 0; pi < 4 && !found; ++pi)
                {
                    const ChooserState& w1 = probes[pi];
                    const uint64_t mask1 = Probe(mm, t, w1, kDNI).dbMask;
                    int32_t Y = -1;
                    uint32_t yEntry = 0;
                    if (Q.database >= 0 && !((mask1 >> Q.database) & 1u))
                        for (uint32_t d = 0; d < db.NumDatabases() && Y < 0; ++d)
                            if (((mask1 >> d) & 1u) && RowOf(db, d, Q.clip, Q.frame, &yEntry) >= 0)
                                Y = static_cast<int32_t>(d);
                    if (Y < 0)
                        continue;
                    found = true;
                    MatchResult rf[5];
                    for (int m = 0; m < 5; ++m)
                    {
                        rf[m] = Probe(mm, t, w1, m);
                        VerifyProbe(mm, db, t, Q, rf[m], m, "two-database clip");
                    }
                    const int32_t yRow = RowOf(db, static_cast<uint32_t>(Y), Q.clip, Q.frame);
                    std::printf("  rules: %s frame %u playing in %s, also indexed in %s (row %d), probe state %d set %s; IOD: "
                                "jumped %d db %s cont %.4g\n",
                                std::string(db.ClipName(static_cast<uint32_t>(Q.clip))).c_str(), Q.frame,
                                std::string(db.DbName(static_cast<uint32_t>(Q.database))).c_str(),
                                std::string(db.DbName(static_cast<uint32_t>(Y))).c_str(), yRow, pi, MaskNames(db, mask1).c_str(),
                                static_cast<int>(rf[kIOD].jumped),
                                rf[kIOD].database >= 0 ? std::string(db.DbName(static_cast<uint32_t>(rf[kIOD].database))).c_str() : "-",
                                rf[kIOD].continuingCost == kNoCost ? -1.0 : rf[kIOD].continuingCost);
                    // the playing database left the set (force-interrupted there) but Y indexes the clip and is in the set
                    CHECK(rf[kIOD].continuingCost < kNoCost && rf[kIODInv].continuingCost < kNoCost,
                          "(f) continuing clip not looked up in the chooser database indexing it (IOD %g, IODInv %g)",
                          rf[kIOD].continuingCost, rf[kIODInv].continuingCost);
                    CHECK(rf[kIOD].jumped || (rf[kIOD].database == Y && rf[kIOD].localRow == yRow && !rf[kIOD].continuingOutsideSet),
                          "(f) kept continuing pose must move to %s", std::string(db.DbName(static_cast<uint32_t>(Y))).c_str());
                    CHECK(rf[kFI].continuingCost == kNoCost && rf[kFIInv].continuingCost == kNoCost,
                          "(f) ForceInterrupt evaluated a continuing search");
                    if (db.Entry(yEntry).flags & kEntryDisableReselection)
                        disableCheck(mm, t, Q, rf[kFI], static_cast<uint32_t>(Y), "second database");
                    else
                        std::printf("    entry in %s not DISABLE_RESELECTION\n", std::string(db.DbName(static_cast<uint32_t>(Y))).c_str());
                }
            }
        }
        CHECK(found, "(f) no frame of a clip indexed in two databases reached with only the other database in the set");
    }
    CHECK(eChecked >= 1, "(e) no DISABLE_RESELECTION setup where the rule decides the pick (%u)", eChecked);

    // (g) PSL 888 GetBestResult + 959 InitFrom: an invalid search result resets the motion matching state (no
    // continuing pose afterwards: the next pick is a jump without a blend source).
    {
        ChooserMatcher x = mm;
        ChooserState bad = idle;
        bad.movementMode = bad.movementState = bad.gait = bad.stance = 9;
        const MatchResult r0 = x.Update(0.0f, traj.data(), static_cast<uint32_t>(traj.size()), bad, nullptr, &kModes[kFI]);
        CHECK(r0.numDatabases == 0, "(g) precondition: out-of-range enums still choose %u databases", r0.numDatabases);
        CHECK(r0.database < 0 && r0.clip < 0, "(g) invalid search result kept a pose (db %d)", r0.database);
        const MatchResult r1 = x.Update(kDt, traj.data(), static_cast<uint32_t>(traj.size()), idle);
        CHECK(r1.database >= 0 && r1.jumped && !r1.blend.valid && r1.blend.fromClip == -1 && r1.continuingCost == kNoCost,
              "(g) after the reset: jumped %d blend %d from %d cont %g", static_cast<int>(r1.jumped),
              static_cast<int>(r1.blend.valid), r1.blend.fromClip, r1.continuingCost);
        // control: a valid ForceInterrupt result keeps the blend source
        ChooserMatcher y = mm;
        const MatchResult c = y.Update(kDt, traj.data(), static_cast<uint32_t>(traj.size()), idle, nullptr, &kModes[kFI]);
        CHECK(c.jumped && c.blend.valid && c.blend.fromClip >= 0, "(g) control: ForceInterrupt jump without a blend source");
    }
    // (h) motion Get_MMInterruptMode (pseudo l.1924-1927): InterruptOnDatabaseChange if MovementMode changed or (MovementMode
    // OnGround and MovementState / Gait / Stance changed), else DoNotInterrupt. All 576 combinations; other inputs vary.
    {
        uint32_t n = 0, bad = 0, iod = 0;
        for (int mo0 = 0; mo0 < 2; ++mo0)
            for (int mo1 = 0; mo1 < 2; ++mo1)
                for (int st0 = 0; st0 < 2; ++st0)
                    for (int st1 = 0; st1 < 2; ++st1)
                        for (int g0 = 0; g0 < 3; ++g0)
                            for (int g1 = 0; g1 < 3; ++g1)
                                for (int sn0 = 0; sn0 < 2; ++sn0)
                                    for (int sn1 = 0; sn1 < 2; ++sn1)
                                    {
                                        ChooserState cur, last;
                                        cur.movementMode = mo0;
                                        last.movementMode = mo1;
                                        cur.movementState = st0;
                                        last.movementState = st1;
                                        cur.gait = g0;
                                        last.gait = g1;
                                        cur.stance = sn0;
                                        last.stance = sn1;
                                        cur.speed2dCmS = 100.0f * static_cast<float>(n % 5);
                                        cur.isPivoting = (n & 1u) != 0;
                                        last.isStarting = (n & 2u) != 0;
                                        last.databaseLod = static_cast<int>(n % 2);
                                        cur.justLandedLight = (n & 4u) != 0;
                                        const bool want = mo0 != mo1 || ((st0 != st1 || g0 != g1 || sn0 != sn1) && mo0 == 0);
                                        const Interrupt e = want ? Interrupt::InterruptOnDatabaseChange : Interrupt::DoNotInterrupt;
                                        const Interrupt got = mm.InterruptFor(cur, last);
                                        if (got != e)
                                        {
                                            ++bad;
                                            if (bad <= 5)
                                                std::printf("    InterruptFor mode %d/%d state %d/%d gait %d/%d stance %d/%d -> %d, want %d\n",
                                                            mo0, mo1, st0, st1, g0, g1, sn0, sn1, static_cast<int>(got),
                                                            static_cast<int>(e));
                                        }
                                        iod += want ? 1u : 0u;
                                        ++n;
                                    }
        std::printf("  rules: Get_MMInterruptMode table %u combinations, %u InterruptOnDatabaseChange, %u wrong\n", n, iod, bad);
        CHECK(n == 576 && bad == 0, "(h) InterruptFor: %u of %u combinations wrong", bad, n);
    }
    mm.Reset();
}

void RunDb(const char* aPath, const char* aRef, const std::string& aClipMap)
{
    std::string path(aPath);
    const bool man = path.find("_man") != std::string::npos;
    std::string err;
    MotionDatabase db;
    const auto l0 = std::chrono::high_resolution_clock::now();
    const bool ok = db.LoadFromFile(aPath, man ? kRigIdentityMan : kRigIdentityWoman, err);
    const auto l1 = std::chrono::high_resolution_clock::now();
    CHECK(ok, "load %s: %s", aPath, err.c_str());
    if (!ok)
        return;
    std::printf("[%s] load %.0f ms, %u databases, %u rows\n", aPath,
                std::chrono::duration<double, std::milli>(l1 - l0).count(), db.NumDatabases(), db.NumRows());
    std::string chPath = path.substr(0, path.size() - 7) + ".chooser.json";
    chooser::Table table;
    const bool tok = table.LoadFile(chPath.c_str(), err);
    CHECK(tok, "chooser %s: %s", chPath.c_str(), err.c_str());
    if (!tok)
        return;
    ChooserMatcher mm;
    MatcherConfig cfg;
    CHECK(mm.Init(&db, &table, cfg, err), "init: %s", err.c_str());
    if (aRef)
        QueryParity(mm, db, aRef);
    // wrong rig is refused
    {
        MotionDatabase other;
        std::string e2;
        CHECK(!other.LoadFromFile(aPath, man ? kRigIdentityWoman : kRigIdentityMan, e2), "other rig must be refused");
    }

    std::vector<double> allUs;
    std::vector<std::pair<int32_t, int32_t>> allPicks;
    auto report = [&](const char* name, const PhaseStats& s) {
        std::printf("  %-22s updates %3u jumps %3u median %.1f us max %.1f us  dbs: %s\n", name, s.updates, s.jumps,
                    Median(s.us), s.us.empty() ? 0.0 : *std::max_element(s.us.begin(), s.us.end()),
                    MaskNames(db, s.maskUnion).c_str());
        CHECK(s.badJumpDb == 0, "%s: %u jumps outside the chooser databases", name, s.badJumpDb);
        CHECK(s.nonFinite == 0, "%s: %u non-finite costs", name, s.nonFinite);
        CHECK(s.noPose == 0, "%s: %u updates without a pose", name, s.noPose);
        CHECK(s.notMember == 0, "%s: %u playing poses not an exact DENT/DROW member of their database / clip", name, s.notMember);
        CHECK(s.notChosen == 0, "%s: %u playing poses outside the chooser databases of their selection", name, s.notChosen);
        allPicks.insert(allPicks.end(), s.picks.begin(), s.picks.end());
        allUs.insert(allUs.end(), s.us.begin(), s.us.end());
    };

    // ---- 1. idle 2 s
    {
        Straight m;
        m.speed = SpeedIdle;
        Sim sim;
        sim.m = &m;
        sim.Build(10.0);
        mm.Reset();
        double now = 0.0;
        const PhaseStats s = RunPhase(mm, db, sim, now, 60, [](int, double) { return Ground(0, 0, 0.0f); });
        report("idle 2 s", s);
        CHECK(AnyExact(db, s.maskUnion, {"PSD_Dense_Stand_Idles"}), "idle: chooser set without an Idle database: %s", MaskNames(db, s.maskUnion).c_str());
        CHECK(s.jumps <= 6, "idle: %u jumps in 2 s (expected a stable idle)", s.jumps);
        const MatchResult& r = mm.Last();
        CHECK(r.database >= 0 && db.DbName(static_cast<uint32_t>(r.database)) == "PSD_Dense_Stand_Idles",
              "idle: playing database %s", r.database >= 0 ? std::string(db.DbName(r.database)).c_str() : "-");
    }
    // ---- crouch transition must be one-shot: a completed stand->crouch clip must not be selected again
    // during a stable crouch idle. Check the clip clock as well as clip-ID changes: retail video 2026-10-15 showed
    // the same non-looping clip restarting, which a jump-count-only assertion cannot detect.
    {
        Straight m;
        m.speed = SpeedIdle;
        Sim sim;
        sim.m = &m;
        sim.Build(16.0);
        int32_t transitionClip = -1;
        for (uint32_t c = 0; c < db.NumClips(); ++c)
            if (db.ClipName(c) == "M_Neutral_Transition_Stand_to_Crouch")
                transitionClip = static_cast<int32_t>(c);
        uint32_t transitionEntryFlags = 0;
        for (uint32_t d = 0; d < db.NumDatabases(); ++d)
            if (db.DbName(d) == "PSD_Dense_Crouch_Idles")
                for (uint32_t e = db.Db(d).firstEntry; e < db.Db(d).firstEntry + db.Db(d).numEntries; ++e)
                    if (db.Entry(e).clip == static_cast<uint32_t>(std::max(transitionClip, 0)))
                        transitionEntryFlags = db.Entry(e).flags;
        CHECK(transitionClip >= 0 && (transitionEntryFlags & kEntryDisableReselection),
              "crouch transition test: dense entry lacks DISABLE_RESELECTION (clip=%d flags=0x%x)", transitionClip,
              transitionEntryFlags);
        mm.Reset();
        g_selMask = 0;
        double now = 0.0;
        const PhaseStats stand = RunPhase(mm, db, sim, now, 30, [](int, double) { return Ground(0, 0, 0.0f); });
        ChooserState crouch = Ground(0, 0, 0.0f);
        crouch.stance = 1;
        const PhaseStats crouchIdle = RunPhase(mm, db, sim, now, 300, [crouch](int, double) { return crouch; });
        uint32_t transitionJumps = 0;
        for (const auto& pick : stand.picks)
            transitionJumps += db.ClipName(static_cast<uint32_t>(pick.second)) == "M_Neutral_Transition_Stand_to_Crouch" ? 1u : 0u;
        for (const auto& pick : crouchIdle.picks)
            transitionJumps += db.ClipName(static_cast<uint32_t>(pick.second)) == "M_Neutral_Transition_Stand_to_Crouch" ? 1u : 0u;
        CHECK(AnyExact(db, crouchIdle.maskUnion, {"PSD_Dense_Crouch_Idles"}), "crouch transition test: chooser lacks crouch idle database");
        CHECK(transitionJumps == 1, "crouch transition selections=%u during one stand->crouch change; expected exactly one", transitionJumps);
        float previousTransitionTime = -1.0f;
        uint32_t transitionClockResets = 0;
        for (const auto& sample : crouchIdle.clipClock)
        {
            const int32_t clip = std::get<0>(sample);
            const float clipTime = std::get<1>(sample);
            if (clip != transitionClip)
                continue;
            if (previousTransitionTime >= 1.0f && clipTime + 0.10f < previousTransitionTime)
                ++transitionClockResets;
            previousTransitionTime = clipTime;
        }
        CHECK(transitionClockResets == 0,
              "crouch transition clip clock reset %u times during stable crouch; expected no restart", transitionClockResets);
        // Simulate a one-frame gate/fallback re-entry while the player remains crouched. Reset the playing pose and
        // history as the runtime does, but keep the completed transition suppressed until the stance actually changes.
        mm.Reset(true);
        g_selMask = 0;
        const PhaseStats reentry = RunPhase(mm, db, sim, now, 300, [crouch](int, double) { return crouch; });
        uint32_t transitionReentries = 0;
        for (const auto& pick : reentry.picks)
            transitionReentries += pick.second == transitionClip ? 1u : 0u;
        CHECK(transitionReentries == 0,
              "crouch transition reselected %u times after matcher re-entry with unchanged stance; expected none",
              transitionReentries);
        const PhaseStats standAgain = RunPhase(mm, db, sim, now, 30, [](int, double) { return Ground(0, 0, 0.0f); });
        (void)standAgain;
        const PhaseStats crouchAgain = RunPhase(mm, db, sim, now, 30, [crouch](int, double) { return crouch; });
        uint32_t freshTransitionPicks = 0;
        for (const auto& pick : crouchAgain.picks)
            freshTransitionPicks += pick.second == transitionClip ? 1u : 0u;
        CHECK(freshTransitionPicks == 1,
              "fresh stand->crouch transition selections=%u after stance changed; expected exactly one", freshTransitionPicks);
    }
    // ---- 2. idle -> walk start (start at t = 2 s)
    {
        Straight m;
        m.speed = SpeedWalkStart;
        Sim sim;
        sim.m = &m;
        sim.Build(10.0);
        mm.Reset();
        double now = 0.0;
        PhaseStats a = RunPhase(mm, db, sim, now, 45, [](int, double) { return Ground(0, 0, 0.0f); });
        report("walk start: idle", a);
        // starting: moving, IsStarting while the future speed exceeds the current one
        PhaseStats b = RunPhase(mm, db, sim, now, 15, [](int, double t) {
            ChooserState s = Ground(1, 0, SpeedWalkStart(t));
            s.isStarting = true;
            return s;
        });
        report("walk start: start", b);
        CHECK(AnyExact(db, b.maskUnion, {"PSD_Dense_Stand_Walk_Starts"}), "start: no Starts database: %s", MaskNames(db, b.maskUnion).c_str());
        CHECK(b.jumps >= 1, "start: no reselection after the interrupt");
        const MatchResult& r = mm.Last();
        CHECK(r.database >= 0 && ((b.maskUnion >> r.database) & 1u), "start: playing database not chosen");
        PhaseStats c = RunPhase(mm, db, sim, now, 45, [](int, double t) { return Ground(1, 0, SpeedWalkStart(t)); });
        report("walk start: walk", c);
        CHECK(AnyExact(db, c.maskUnion, {"PSD_Dense_Stand_Walk_Loops"}), "walk: no Walk database");
    }
    // ---- 3. walk -> stop (decelerate at t = 2 s)
    {
        Straight m;
        m.speed = SpeedWalkStop;
        Sim sim;
        sim.m = &m;
        sim.Build(10.0);
        mm.Reset();
        double now = 0.0;
        PhaseStats a = RunPhase(mm, db, sim, now, 50, [](int, double) { return Ground(1, 0, 1.6f); });
        report("walk stop: walk", a);
        PhaseStats b = RunPhase(mm, db, sim, now, 40, [](int, double t) {
            const float v = SpeedWalkStop(t);
            return Ground(0, 0, v);
        });
        report("walk stop: stop/idle", b);
        CHECK(AnyExact(db, b.maskUnion, {"PSD_Dense_Stand_Walk_Stops", "PSD_Dense_Stand_Idles"}), "stop: no Stop/Idle database: %s",
              MaskNames(db, b.maskUnion).c_str());
        CHECK(b.jumps >= 1, "stop: no reselection after Moving -> Idle");
    }
    // ---- 4. walk 180 pivot
    {
        Pivot m;
        Sim sim;
        sim.m = &m;
        sim.Build(10.0);
        mm.Reset();
        double now = 0.0;
        PhaseStats a = RunPhase(mm, db, sim, now, 45, [](int, double) { return Ground(1, 0, 1.6f); });
        report("pivot: walk", a);
        PhaseStats b = RunPhase(mm, db, sim, now, 20, [](int, double) {
            ChooserState s = Ground(1, 0, 1.6f);
            s.isPivoting = true;
            return s;
        });
        report("pivot: pivoting", b);
        CHECK(AnyExact(db, b.maskUnion, {"PSD_Dense_Stand_Walk_Pivots"}), "pivot: no Pivots database: %s", MaskNames(db, b.maskUnion).c_str());
    }
    // ---- 5. run loop 3 s
    {
        Straight m;
        m.speed = SpeedRun;
        Sim sim;
        sim.m = &m;
        sim.Build(10.0);
        mm.Reset();
        double now = 0.0;
        PhaseStats a = RunPhase(mm, db, sim, now, 90, [](int, double) { return Ground(1, 1, 3.5f); });
        report("run loop", a);
        CHECK(AnyExact(db, a.maskUnion, {"PSD_Dense_Stand_Run_Loops"}), "run: no Run database");
        CHECK(a.jumps <= 30, "run: %u jumps in 3 s", a.jumps);
    }
    // ---- 6. jump: run, take-off at 2 s, in air with TTL decreasing, landing
    {
        Jump m;
        Sim sim;
        sim.m = &m;
        sim.Build(10.0);
        mm.Reset();
        double now = 0.0;
        PhaseStats a = RunPhase(mm, db, sim, now, 60, [](int, double) { return Ground(1, 1, 3.5f); });
        report("jump: run", a);
        const double flight = 2.0 * 4.0 / 9.81; // 0.815 s
        float lastTtl = 10.0f;
        uint32_t ttlNonDecreasing = 0;
        PhaseStats b = RunPhase(mm, db, sim, now, static_cast<int>(flight / kDt), [&](int, double t) {
            ChooserState s = Ground(1, 1, 3.5f);
            s.movementMode = 1;
            s.timeToLand = static_cast<float>(std::max(0.0, 2.0 + flight - t));
            s.verticalVelocityCmS = static_cast<float>((4.0 - 9.81 * (t - 2.0)) * 100.0);
            if (s.timeToLand >= lastTtl)
                ++ttlNonDecreasing;
            lastTtl = s.timeToLand;
            return s;
        });
        report("jump: in air", b);
        CHECK(ttlNonDecreasing == 0, "TTL not decreasing");
        CHECK(b.jumps >= 1, "air: no reselection after OnGround -> InAir");
        CHECK(!AnyExact(db, b.maskUnion, {"PSD_Dense_Stand_Idles", "PSD_Dense_Crouch_Idles"}), "air: Idle database chosen in air");
        PhaseStats c = RunPhase(mm, db, sim, now, 30, [](int i, double) {
            ChooserState s = Ground(1, 1, 3.5f);
            s.justLandedLight = i < 3;
            return s;
        });
        report("jump: landing", c);
        CHECK(c.jumps >= 1, "landing: no reselection after InAir -> OnGround");
    }
    // ---- blend times and interrupt modes (report 3.8)
    {
        const ChooserState g = Ground(1, 0, 1.6f);
        ChooserState air = g;
        air.movementMode = 1;
        air.verticalVelocityCmS = 300.0f;
        ChooserState fall = air;
        fall.verticalVelocityCmS = -300.0f;
        CHECK(std::fabs(mm.BlendTimeFor(g, g) - 0.5f) < 1e-6f, "ground->ground");
        CHECK(std::fabs(mm.BlendTimeFor(g, air) - 0.2f) < 1e-6f, "landing");
        CHECK(std::fabs(mm.BlendTimeFor(air, g) - 0.15f) < 1e-6f, "air rising");
        CHECK(std::fabs(mm.BlendTimeFor(fall, air) - 0.5f) < 1e-6f, "air other");
        ChooserState run = g;
        run.gait = 1;
        ChooserState idle = g;
        idle.movementState = 0;
        ChooserState crouch = g;
        crouch.stance = 1;
        CHECK(mm.InterruptFor(g, g) == Interrupt::DoNotInterrupt, "same state");
        CHECK(mm.InterruptFor(run, g) == Interrupt::InterruptOnDatabaseChange, "gait change");
        CHECK(mm.InterruptFor(idle, g) == Interrupt::InterruptOnDatabaseChange, "state change");
        CHECK(mm.InterruptFor(crouch, g) == Interrupt::InterruptOnDatabaseChange, "stance change");
        CHECK(mm.InterruptFor(air, g) == Interrupt::InterruptOnDatabaseChange, "mode change");
        ChooserState air2 = air;
        air2.gait = 2;
        CHECK(mm.InterruptFor(air2, air) == Interrupt::DoNotInterrupt, "air gait change does not interrupt");
    }
    // ---- reselection history: a jump never lands on a row played within 0.3 s; blend request carries the time
    {
        Straight m;
        m.speed = SpeedRun;
        Sim sim;
        sim.m = &m;
        sim.Build(10.0);
        mm.Reset();
        double now = 0.0;
        std::vector<TrajSample> traj;
        uint32_t histViolations = 0, jumps = 0, badBlend = 0;
        for (int i = 0; i < 120; ++i)
        {
            now += kDt;
            sim.Traj(now, traj);
            const ChooserState cs = Ground(1, i < 60 ? 0 : 1, i < 60 ? 1.6f : 3.5f);
            // candidate check needs the pre-update history: predict by re-checking after the update with the
            // selected row (the matcher pushes the selected row itself, so test against rows played before)
            ChooserMatcher snapshot = mm;
            const MatchResult& r = mm.Update(kDt, traj.data(), static_cast<uint32_t>(traj.size()), cs);
            if (r.jumped && r.database >= 0)
            {
                ++jumps;
                // the snapshot's history is the pre-update one; its clock is one step behind
                if (snapshot.InHistory(static_cast<uint32_t>(r.database), static_cast<uint32_t>(r.localRow)))
                    ++histViolations;
                if (i > 0 && (!r.blend.valid || !(r.blend.blendTime > 0.0f)))
                    ++badBlend;
            }
        }
        std::printf("  reselection: %u jumps, %u into the 0.3 s history, %u bad blend requests\n", jumps, histViolations, badBlend);
        CHECK(histViolations == 0, "jump into the reselection history");
        CHECK(badBlend == 0, "jump without a blend request");
    }
    // ---- mirrored turn-in-place negative control: standing, the trajectory facing turns 90 deg left (CCW, yaw +)
    // or right (yaw -) over 0.5 s; the first pose selected with ShouldTurnInPlace must be a turn clip of that side
    for (int side = 0; side < 2; ++side)
    {
        struct Turn : Motion
        {
            float sign = 1.0f;
            void At(double t, float& vx, float& vy, float& yaw, float& z) const override
            {
                vx = vy = z = 0.0f;
                const float u = static_cast<float>(std::clamp((t - 2.0) / 0.5, 0.0, 1.0));
                yaw = sign * u * 0.5f * math::kPi;
            }
        } m;
        m.sign = side == 0 ? 1.0f : -1.0f;
        Sim sim;
        sim.m = &m;
        sim.Build(10.0);
        mm.Reset();
        double now = 0.0;
        PhaseStats a = RunPhase(mm, db, sim, now, 50, [](int, double) { return Ground(0, 0, 0.0f); });
        PhaseStats b = RunPhase(mm, db, sim, now, 10, [](int, double) {
            ChooserState s = Ground(0, 0, 0.0f);
            s.shouldTurnInPlace = true;
            return s;
        });
        const char* want = side == 0 ? "_L" : "_R";
        const char* other = side == 0 ? "_R" : "_L";
        int32_t clip = -1;
        for (const auto& pk : b.picks)
            if (db.DbName(static_cast<uint32_t>(pk.first)) == "PSD_Dense_Stand_TurnInPlace")
            {
                clip = pk.second;
                break;
            }
        const std::string name = clip >= 0 ? std::string(db.ClipName(static_cast<uint32_t>(clip))) : std::string("-");
        std::printf("  turn in place %s: %u jumps, first TurnInPlace clip %s\n", side == 0 ? "left " : "right", b.jumps,
                    name.c_str());
        CHECK(a.notMember == 0 && b.notMember == 0 && b.notChosen == 0, "turn %d: membership", side);
        CHECK(AnyExact(db, b.maskUnion, {"PSD_Dense_Stand_TurnInPlace"}), "turn %d: no TurnInPlace database: %s", side,
              MaskNames(db, b.maskUnion).c_str());
        const bool isTurn = name.rfind("M_Neutral_Stand_Turn_", 0) == 0;
        const bool wantSide = name.size() >= 2 && name.compare(name.size() - 2, 2, want) == 0;
        const bool otherSide = name.size() >= 2 && name.compare(name.size() - 2, 2, other) == 0;
        CHECK(isTurn && wantSide && !otherSide, "turn %s: selected %s (expected a M_Neutral_Stand_Turn_*%s clip)",
              side == 0 ? "left" : "right", name.c_str(), want);
        allPicks.insert(allPicks.end(), b.picks.begin(), b.picks.end());
    }
    // ---- cross-check 5 distinct picks against the clip map's dense database membership
    if (!aClipMap.empty())
    {
        std::vector<std::pair<int32_t, int32_t>> uniq;
        for (const auto& pk : allPicks)
            if (std::find(uniq.begin(), uniq.end(), pk) == uniq.end())
                uniq.push_back(pk);
        uint32_t checked = 0;
        for (size_t i = 0; i < uniq.size() && checked < 5; ++i)
        {
            // spread over the pick list
            const auto& pk = uniq[i * uniq.size() / std::min<size_t>(uniq.size(), 5) % uniq.size()];
            const std::string clipName(db.ClipName(static_cast<uint32_t>(pk.second)));
            const std::string dbName(db.DbName(static_cast<uint32_t>(pk.first)));
            const bool inMap = ClipMapDense(aClipMap, clipName, dbName);
            std::printf("  clip map: %s in %s -> %s\n", clipName.c_str(), dbName.c_str(), inMap ? "listed" : "NOT LISTED");
            CHECK(inMap, "clip map: %s is not a dense member of %s", clipName.c_str(), dbName.c_str());
            ++checked;
        }
        CHECK(checked == 5, "clip map: only %u distinct picks", checked);
        CHECK(!ClipMapDense(aClipMap, "M_Neutral_Stand_Turn_090_L", "PSD_Dense_Stand_Run_Loops"),
              "clip map negative control: a turn clip listed in Run_Loops");
    }
    // ---- UE 5.8 continuing-pose rules and Get_MMInterruptMode (commit c3cff14)
    ContinuingRules(mm, db);
    const double med = Median(allUs);
    std::printf("  all scenario updates: %zu, median %.1f us, p95 %.1f us\n", allUs.size(), med,
                [&] { auto v = allUs; std::sort(v.begin(), v.end()); return v.empty() ? 0.0 : v[v.size() * 95 / 100]; }());
    CHECK(med < 300.0, "median update %.1f us >= 300 us", med);
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::printf("usage: utpp_mm_chooser_matcher_test [--clip-map <json>] [--query-ref <ref.txt>] <db.utppmm> [...]\n");
        return 2;
    }
    // options: --clip-map <sourcepack-clip-map.json> (all databases), --query-ref <ref.txt> (the next database only)
    std::string clipMap;
    const char* ref = nullptr;
    for (int i = 1; i < argc; ++i)
    {
        const std::string a(argv[i]);
        if (a == "--clip-map" && i + 1 < argc)
        {
            std::ifstream f(argv[++i], std::ios::binary);
            CHECK(f.good(), "clip map %s missing", argv[i]);
            clipMap.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
        }
        else if (a == "--query-ref" && i + 1 < argc)
            ref = argv[++i];
        else
        {
            RunDb(argv[i], ref, clipMap);
            ref = nullptr;
        }
    }
    std::printf("utpp_mm_chooser_matcher_test: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
