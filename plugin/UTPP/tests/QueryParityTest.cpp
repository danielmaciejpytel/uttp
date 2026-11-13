// MM query-vs-bake parity on non-straight rows (finding 8, strong version of the MotionAdapterCompileTest parity block).
// Usage: utpp_mm_query_parity_test --db <plain utpp_motion_woman_noslide.utppmm> [--db <plain man>] [--load-only <shipped db>]
// (chooser table next to each database: *.utppmm -> *.chooser.json). Without --db the test FAILS (no silent pass).
//
// Per body, four real rows chosen by a root-path metric (not by hand): an accelerating start (*_Run_Starts), a turning
// pivot (*_Run_Pivots, largest root-yaw change over the history window), a strafe / backward loop (*_Run_Loops, largest
// |sin(facing - velocity direction)| at t = 0) and a decelerating stop (*_Run_Stops). For each row:
//  T(h)  the clip root path on the half-frame grid of FORMAT.txt "Feature values" (l. 370-388): integer frames from the ROOT
//        steps (loops wrap, non-loops extrapolate with the first / last step), odd half frames averaged; double precision.
//  feed  60 Hz MotionFeed frames h = h0 .. H: velWorld = (T(h) - T(h-1)) x 60, playerYaw = yaw(h) (actor yaw = clip root yaw),
//        stick along the clip's 1 s displacement (stop: released), Stand / Run; MapToChooser each frame, then
//        BuildQueryTrajectory -> query; ChooserMatcher::BuildQuery -> raw trajectory features.
//  Time offset: MotionTrajectory (UE UpdateHistory_TransformHistoryInternal, PoseSearchTrajectoryLibrary.cpp l. 297-317, mirrored in
//        MotionTrajectory.hpp l. 398-413) stores last frame's current sample as the record at t = 0 and the current capsule at
//        +DeltaTime, so the query origin t = 0 is the capsule BEFORE the last feed frame: the feed matches row F when it ends
//        at H = 1 (one 60 Hz frame = half a clip frame after F).
// Checks:
//  (M) ChooserMatcher features of the clip's own root path at the row = DFEA raw, every trajectory dim (eps below);
//  (H) history-side dims (every read time <= 0: Position / Heading with t_sample, t_origin <= 0, velocities incl. the
//      t - 1/60 reads) of the query = DFEA raw (eps); they include t = 0 velocity and facing;
//  (S) spaces: query samples at history times = world-space clip path relative to the current sample (metres) and
//      yaw = game yaw (rad), under a rotated + translated feed too;
//  (T) times: BuildQueryTrajectory returns exactly the BuildQueryTimes times, strictly ascending;
//  (R) character-space invariance: the feed rotated by a world yaw R (velocities, actor and camera yaw) after a translating
//      lead-in gives the same raw features in EVERY trajectory dim (history and future) within eps;
//  (D) time shift: feeds ending at H = 1 +- 1 and +- 2 frames must NOT match the history dims (some dim beyond its eps);
//      this proves the row is discriminative, so (H) passing means the offset agrees;
//  future-side dims (t_sample > 0) depend on the CMC prediction vs the authored clip: reported (|query - DFEA| per dim and
//      the maximum per kind), not asserted.
// eps (no free numbers): float32 rounding bounds from the actual magnitudes of each row, u = 2^-24:
//  feed integration error E = n (6u Dmax + u Lmax) (n frames, Dmax the largest per-frame step, Lmax the largest |capsule
//  position| reached, both in m; 6 roundings per step: velocity, x100, x dt, dt itself, sum, x0.01), matcher composition
//  6u Rmax per relative vector (subtract, cos / sin, products, sum; Rmax = 2 x largest |T(h) - T(F)| of the window), yaw
//  conversion e_yaw = 6u (|yaw| + 540 deg) in rad (deg -> +90 -> normalize -> -90 -> rad), DFEA reconstruction
//  3u (|stored / scale| + |mean|) + u |raw|. Position eps = 2 (E + 2u Lmax) + 2 x 6u Rmax + Rmax e_yaw + dfea; velocity =
//  60 x 2 x (position eps without dfea) + dfea; velocity direction = 2 x 100 x velocity eps / max(1, 100 |v|) + dfea;
//  heading = 2 e_yaw + 4u + dfea. For (M) the feed term E is replaced by the bake's own float32 integration bound over the
//  integrated window, since the baker's precision is not stored in the file: nb (6u Dmax + u Lmax) with the clip steps
//  (nb = window frames) PLUS a yaw accumulation term nb^2 u Dmax (ymax + 1): the baked root yaw is accumulated over the
//  nb steps with a rounding of <= u (ymax + 1) rad per step, and the k-th yaw error rotates every later step (length
//  <= Dmax), so the position error is <= sum_k k u (ymax + 1) Dmax <= nb^2 u Dmax (ymax + 1) (ymax = largest |root yaw|,
//  rad, of the window).
// These eps are worst-case rounding bounds, not tolerances fitted to the data: observed (H) / (M) errors are 0.000 - 0.003
//  of eps (about 10^3 below), so (H) / (M) alone would not detect an error smaller than eps. Discrimination is shown only
//  for the injected error classes (sensitivity checks (X) below: (D) time shift, (V) velocity space, (F1) / (F2) facing);
//  (V) is asserted on the moving rows only - on the stop row (0.07 m/s at t = 0) the swap is 1.3 x eps and only reported.
//  Production mutations are caught by (M) / (H) anyway (verifier scratch mutation: matcher velocity-space swap -> (M) and
//  (H) fail on all 8 rows).
#include <cmath>
#include <cstdio>
#include <string>
#include <string_view>
#include <vector>

#include "../src/mm/MotionAdapter.hpp"

namespace
{
int g_pass = 0, g_fail = 0;
void Check(bool aOk, const std::string& aWhat)
{
    if (aOk)
        ++g_pass;
    else
    {
        ++g_fail;
        std::printf("FAIL %s\n", aWhat.c_str());
    }
}

namespace motionns = utpp::mm::motion;
using namespace utpp::mm::adapter;
constexpr double kU = 5.9604644775390625e-8; // 2^-24, float32 unit roundoff
constexpr double kPiD = 3.14159265358979323846;
constexpr double kR2D = 180.0 / kPiD;

// Clip root path around row frame F on integer frames k = -kB .. kA (k = 0 identity), double precision.
struct Path
{
    static constexpr int kB = 40, kA = 48;
    double x[kB + kA + 1] = {}, y[kB + kA + 1] = {}, yaw[kB + kA + 1] = {};
    double maxStep = 0.0; // largest per-frame step (m)
    bool Build(const motionns::MotionDatabase& db, uint32_t aRow)
    {
        const motionns::RowRecord& R = db.Row(aRow);
        const motionns::EntryRecord& E = db.Entry(R.entry);
        const motionns::MotionClipRecord& C = db.Clip(E.clip);
        const bool loop = (C.flags & motionns::kClipLoop) && C.loopPeriod > 0;
        if (C.numFrames < 2 || R.frame < C.firstFrame || R.frame >= C.firstFrame + C.numFrames)
            return false;
        const int j0 = static_cast<int>(R.frame - C.firstFrame);
        auto step = [&](int k) -> const auto& {
            int mi = j0 + k;
            if (loop)
                mi = (mi % static_cast<int>(C.loopPeriod) + static_cast<int>(C.loopPeriod)) % static_cast<int>(C.loopPeriod);
            else
                mi = mi < 0 ? 0 : (mi > static_cast<int>(C.numFrames) - 2 ? static_cast<int>(C.numFrames) - 2 : mi);
            return db.Root(C.firstFrame + static_cast<uint32_t>(mi));
        };
        for (int k = 0; k < kA; ++k) // T(k+1) = T(k) o step(k)
        {
            const auto& sd = step(k);
            const double cy = std::cos(yaw[kB + k]), sy = std::sin(yaw[kB + k]);
            x[kB + k + 1] = x[kB + k] + cy * sd.dx - sy * sd.dy;
            y[kB + k + 1] = y[kB + k] + sy * sd.dx + cy * sd.dy;
            yaw[kB + k + 1] = yaw[kB + k] + sd.dyaw;
            maxStep = std::fmax(maxStep, std::hypot(static_cast<double>(sd.dx), static_cast<double>(sd.dy)));
        }
        for (int k = 0; k > -kB; --k) // T(k-1) = T(k) o step(k-1)^-1
        {
            const auto& sd = step(k - 1);
            const double yw = yaw[kB + k] - sd.dyaw;
            yaw[kB + k - 1] = yw;
            x[kB + k - 1] = x[kB + k] - (std::cos(yw) * sd.dx - std::sin(yw) * sd.dy);
            y[kB + k - 1] = y[kB + k] - (std::sin(yw) * sd.dx + std::cos(yw) * sd.dy);
            maxStep = std::fmax(maxStep, std::hypot(static_cast<double>(sd.dx), static_cast<double>(sd.dy)));
        }
        return true;
    }
    bool HalfOk(int h) const { return h >= -2 * kB && h <= 2 * kA; }
    // half frame h (t = h / 60 s): integer frames exact, odd half frames averaged (FORMAT.txt l. 373)
    void At(int h, double& ox, double& oy, double& oyaw) const
    {
        const int a = (h >= 0 ? h : h - 1) / 2; // floor(h / 2)
        const int b = (h % 2 == 0) ? a : a + 1;
        ox = 0.5 * (x[kB + a] + x[kB + b]);
        oy = 0.5 * (y[kB + a] + y[kB + b]);
        oyaw = 0.5 * (yaw[kB + a] + yaw[kB + b]);
    }
    double SpeedFrame(int k) const { return std::hypot(x[kB + k] - x[kB + k - 1], y[kB + k] - y[kB + k - 1]) * 30.0; }
    double VelHeading(int k) const { return std::atan2(-(x[kB + k] - x[kB + k - 1]), y[kB + k] - y[kB + k - 1]); }
};

double Wrap(double a)
{
    while (a > kPiD)
        a -= 2.0 * kPiD;
    while (a < -kPiD)
        a += 2.0 * kPiD;
    return a;
}

enum class Cat { Start, Pivot, Strafe, Stop };
const char* CatName(Cat c) { return c == Cat::Start ? "start" : c == Cat::Pivot ? "pivot" : c == Cat::Strafe ? "strafe" : "stop"; }

// Row metric on the root path (larger = more discriminative for the category); history window = frames -6 .. 0 (0.2 s,
// the deepest history read of the motion schemas is t = -0.2 s - 1/60). Rows whose path leaves [-kB, kA] are never used.
double Metric(Cat c, const Path& p)
{
    switch (c)
    {
    case Cat::Start: return p.SpeedFrame(0) - p.SpeedFrame(-6);  // acceleration over the history window (m/s)
    case Cat::Stop: return p.SpeedFrame(-6) - p.SpeedFrame(0);   // deceleration over the history window
    case Cat::Pivot: return p.SpeedFrame(0) > 0.5 ? std::fabs(p.yaw[Path::kB] - p.yaw[Path::kB - 6]) : -1.0; // facing turn
    case Cat::Strafe:
    {
        // facing != velocity direction by >= 60 deg [P] (half-way between diagonal and lateral) at t = 0, then the largest
        // velocity-vector change over the history window (a constant-velocity strafe loop cannot detect a time offset)
        if (!(p.SpeedFrame(0) > 1.0) || std::fabs(std::sin(p.VelHeading(0) - p.yaw[Path::kB])) < std::sin(60.0 / kR2D))
            return -1.0;
        const double v0x = p.x[Path::kB] - p.x[Path::kB - 1], v0y = p.y[Path::kB] - p.y[Path::kB - 1];
        const double v6x = p.x[Path::kB - 6] - p.x[Path::kB - 7], v6y = p.y[Path::kB - 6] - p.y[Path::kB - 7];
        return std::hypot(v0x - v6x, v0y - v6y) * 30.0;
    }
    }
    return -1.0;
}

struct RowPick
{
    int32_t db = -1;
    uint32_t row = 0;
    double metric = -1.0;
};

RowPick PickRow(const motionns::MotionDatabase& db, std::string_view aDbSuffix, Cat c)
{
    RowPick best;
    for (uint32_t d = 0; d < db.NumDatabases(); ++d)
    {
        const std::string_view n = db.DbName(d);
        bool want = false; // aDbSuffix: "|"-separated list of PSD_Dense_Stand_<suffix> databases
        for (size_t a = 0, b; a <= aDbSuffix.size(); a = b + 1)
        {
            b = aDbSuffix.find('|', a);
            b = b == std::string_view::npos ? aDbSuffix.size() : b;
            want = want || n == std::string("PSD_Dense_Stand_") + std::string(aDbSuffix.substr(a, b - a));
        }
        if (!want)
            continue;
        const motionns::DatabaseRecord& D = db.Db(d);
        for (uint32_t r = D.firstRow; r < D.firstRow + D.numRows; ++r)
        {
            Path p;
            if (!p.Build(db, r))
                continue;
            const double m = Metric(c, p);
            if (m > best.metric)
                best = RowPick{static_cast<int32_t>(d), r, m};
        }
    }
    return best;
}

struct FeedSpec
{
    int hEnd = 1;              // last fed half frame (H)
    double rotDeg = 0.0;       // world yaw rotation of the whole feed
    int leadFrames = 0;        // translating lead-in frames before the clip part
    double leadVel[2] = {0, 0}; // lead-in velocity (m/s, world, before rotation)
};
constexpr int kClipFeedFrames = 40; // >= 30 history samples + 1 current (MotionTrajectory kHistoryCount) + margin

struct FeedResult
{
    uint32_t n = 0;
    motionns::TrajSample q[kQueryTrajMax];
    double lmax = 0.0; // largest |capsule position| reached (m)
    int frames = 0;
};

FeedResult RunFeed(const Path& p, Cat c, const QueryTimes& qt, const FeedSpec& fs)
{
    MapConfig pc;
    pc.speedScale = 1.0f;          // game m/s x 100 = motion cm/s (no piecewise map): the clip speed space
    pc.gameAnalogModifier = false; // CMC AnalogInputModifier = |stick| (UE PSTL l. 73)
    pc.stickMode = StickMode::VariableSpeedWalkRun;
    MapState ms;
    const MapContext x0;
    const double R = fs.rotDeg / kR2D, cr = std::cos(R), sr = std::sin(R);
    // stick world direction (before rotation): the clip's displacement over the next 1 s from F; stop: released
    double wx = p.x[Path::kB + 30] - p.x[Path::kB], wy = p.y[Path::kB + 30] - p.y[Path::kB];
    const double wl = std::hypot(wx, wy);
    const bool stick = c != Cat::Stop && wl > 1e-6;
    wx = stick ? wx / wl : 0.0;
    wy = stick ? wy / wl : 0.0;
    FeedResult res;
    double px = 0.0, py = 0.0; // capsule position (m) for Lmax
    auto frame = [&](double vx, double vy, double actorYaw, double camYaw) {
        MotionFeed g;
        g.dt = 1.0f / 60.0f;
        g.psmDetailed = kPsmStand;
        g.aiming = c == Cat::Strafe;
        const double rvx = cr * vx - sr * vy, rvy = sr * vx + cr * vy;
        g.velWorld[0] = static_cast<float>(rvx);
        g.velWorld[1] = static_cast<float>(rvy);
        g.playerYawDeg = static_cast<float>((actorYaw + R) * kR2D);
        const double cam = camYaw + R;
        g.cameraYawDeg = static_cast<float>(cam * kR2D);
        // camera-space stick for the rotated world direction: inverse of StickToWorld (MotionAdapter.hpp l. 308-314)
        const double ax = cr * wx - sr * wy, ay = sr * wx + cr * wy;
        g.move[0] = static_cast<float>(std::cos(cam) * ax + std::sin(cam) * ay);
        g.move[1] = static_cast<float>(-std::sin(cam) * ax + std::cos(cam) * ay);
        if (res.frames > 0)
        {
            px += rvx / 60.0;
            py += rvy / 60.0;
        }
        res.lmax = std::fmax(res.lmax, std::hypot(px, py));
        ++res.frames;
        MapToChooser(g, pc, ms, x0);
    };
    double x0d, y0d, yaw0;
    const int h0 = fs.hEnd - kClipFeedFrames;
    p.At(h0, x0d, y0d, yaw0);
    for (int i = 0; i < fs.leadFrames; ++i)
        frame(fs.leadVel[0], fs.leadVel[1], yaw0, c == Cat::Strafe ? yaw0 : 0.0);
    for (int h = h0; h <= fs.hEnd; ++h)
    {
        double ax, ay, ayaw, bx, by, byaw;
        p.At(h, ax, ay, ayaw);
        p.At(h - 1, bx, by, byaw);
        frame((ax - bx) * 60.0, (ay - by) * 60.0, ayaw, c == Cat::Strafe ? ayaw : 0.0);
    }
    res.n = BuildQueryTrajectory(ms, qt, res.q, kQueryTrajMax);
    return res;
}

struct Dim
{
    uint32_t o;        // offset in the schema
    uint32_t kind;
    bool history;      // every read time <= 0
    std::string label;
    uint32_t comp;
    const motionns::ChannelRecord* ch = nullptr;
};

std::vector<Dim> TrajDims(const motionns::MotionDatabase& db, uint32_t d)
{
    std::vector<Dim> out;
    const motionns::SchemaRecord& S = db.Schema(db.Db(d).schema);
    for (uint32_t ci = S.firstChannel; ci < S.firstChannel + S.numChannels; ++ci)
    {
        const motionns::ChannelRecord& ch = db.Channel(ci);
        if (!(ch.flags & motionns::kChanTrajectory))
            continue;
        const bool hist = ch.tSample <= 0.0f && ch.tOrigin <= 0.0f; // velocity reads t - 1/60 are earlier still
        for (uint32_t k = 0; k < ch.cardinality; ++k)
            out.push_back(Dim{ch.dataOffset + k, ch.kind, hist, std::string(db.String(ch.labelOff)), k, &ch});
    }
    return out;
}

void Features(motionns::ChooserMatcher& mm, uint32_t d, const motionns::TrajSample* s, uint32_t n, std::vector<float>& out, uint32_t dims)
{
    mm.SetQueryInputs(s, n, nullptr);
    mm.BuildQuery(d, false);
    out.assign(mm.RawQuery(), mm.RawQuery() + dims);
    mm.SetQueryInputs(nullptr, 0, nullptr);
}

struct EpsIn
{
    double feedE = 0.0;  // accumulated position error of the trajectory being compared (m)
    double rmax = 0.0;   // largest relative vector (m)
    double eyaw = 0.0;   // yaw error (rad)
};

// Sensitivity helper: a Velocity channel evaluated like ChooserMatcher::TrajectoryFeature (l. 690-757: linear SampleTraj,
// rel = R(-yaw_o)(p_s - p_o)) but with the velocity space SWAPPED (character <-> world origin of the t - 1/60 read), i.e. the
// error class "world vs character velocity". Returns the component of dim aComp (components bitmask 1 x, 2 y, 4 z).
double SwappedVelocity(const motionns::TrajSample* s, uint32_t n, const motionns::ChannelRecord& ch, uint32_t aComp)
{
    auto at = [&](float t) {
        if (t <= s[0].t)
            return s[0];
        if (t >= s[n - 1].t)
            return s[n - 1];
        uint32_t i = 1;
        while (i < n && s[i].t < t)
            ++i;
        const motionns::TrajSample& a = s[i - 1];
        const motionns::TrajSample& b = s[i];
        const float u = (b.t > a.t) ? (t - a.t) / (b.t - a.t) : 0.0f;
        motionns::TrajSample r;
        r.t = t;
        r.x = a.x + (b.x - a.x) * u;
        r.y = a.y + (b.y - a.y) * u;
        r.z = a.z + (b.z - a.z) * u;
        r.yaw = a.yaw + utpp::mm::math::WrapAngle(b.yaw - a.yaw) * u;
        return r;
    };
    auto rel = [&](float ts, float to, double o[3]) {
        const motionns::TrajSample a = at(ts), b = at(to);
        const double c = std::cos(b.yaw), sn = std::sin(b.yaw), dx = a.x - b.x, dy = a.y - b.y;
        o[0] = c * dx + sn * dy;
        o[1] = -sn * dx + c * dy;
        o[2] = a.z - b.z;
    };
    constexpr float h = 1.0f / 60.0f;
    double a[3], b[3], v[3];
    rel(ch.tSample, ch.tOrigin, a);
    rel(ch.tSample - h, ch.velocitySpace == 1 ? ch.tOrigin : ch.tOrigin - h, b); // swapped on purpose
    for (int k = 0; k < 3; ++k)
        v[k] = (a[k] - b[k]) * 60.0;
    uint32_t seen = 0;
    for (int k = 0; k < 3; ++k)
        if (ch.components & (1u << k))
        {
            if (seen == aComp)
                return v[k];
            ++seen;
        }
    return 0.0;
}

double DimEps(const Dim& dm, const EpsIn& e, double aDfea, double aRef)
{
    const double pos = 2.0 * e.feedE + 2.0 * 6.0 * kU * e.rmax + e.rmax * e.eyaw;
    switch (dm.kind)
    {
    case 1: return pos + aDfea;
    case 2: return 60.0 * 2.0 * pos + aDfea;
    case 3: return 2.0 * 100.0 * (60.0 * 2.0 * pos) / std::fmax(1.0, 100.0 * std::fabs(aRef)) + aDfea;
    case 4: return 2.0 * e.eyaw + 4.0 * kU + aDfea;
    default: return aDfea;
    }
}

// aLoadOnly (--load-only): the shipped database only has to load, build its matcher and report its deepest query read. The parity checks
// below feed a 30-sample history; a database with the slide-exit schema reads deeper (-0.75 s), so its parity runs on the plain
// fixture (Build/test-data/motion/*_noslide) and its own run here is limited to what is valid for it.
int RunBody(const std::string& aDbPath, bool aLoadOnly = false)
{
    const bool man = aDbPath.find("_man") != std::string::npos;
    const char* body = man ? "man" : "woman";
    motionns::MotionDatabase db;
    std::string err;
    const bool loaded = db.LoadFromFile(aDbPath.c_str(), man ? motionns::kRigIdentityMan : motionns::kRigIdentityWoman, err);
    Check(loaded, std::string(body) + ": database loads");
    if (!loaded)
    {
        std::printf("%s: %s\n", aDbPath.c_str(), err.c_str());
        return 1;
    }
    const std::string chooserPath = aDbPath.substr(0, aDbPath.size() - 7) + ".chooser.json";
    utpp::mm::chooser::Table table;
    const bool tOk = aDbPath.size() > 7 && aDbPath.compare(aDbPath.size() - 7, 7, ".utppmm") == 0 && table.LoadFile(chooserPath.c_str(), err);
    Check(tOk, std::string(body) + ": chooser table next to the database loads");
    motionns::ChooserMatcher mm;
    const bool mmOk = tOk && mm.Init(&db, &table, motionns::MatcherConfig{}, err);
    Check(mmOk, std::string(body) + ": matcher init");
    if (!mmOk)
        return 1;
    const QueryTimes qt = BuildQueryTimes(db);
    Check(qt.n > 0 && !qt.overflow, std::string(body) + ": query times");
    if (aLoadOnly)
    {
        std::printf("INFO %s (load only): deepest history read %.1f frames, the parity feed covers %d\n", body,
                    static_cast<double>(qt.t[0] * 60.0f), kClipFeedFrames - 8);
        return g_fail == 0 ? 0 : 1;
    }
    // the feed must cover every query read: deepest history read inside the 30-sample history, future inside the path
    Check(qt.n > 0 && qt.t[0] * 60.0f >= -(kClipFeedFrames - 8) && qt.t[qt.n - 1] * 60.0f <= 2 * Path::kA - 8,
          std::string(body) + ": query times inside the feed history and the integrated path");

    const struct { Cat c; const char* suffix; } cats[] = {
        {Cat::Start, "Run_Starts"}, {Cat::Pivot, "Run_Pivots"}, {Cat::Strafe, "Run_Loops|Run_Pivots|Run_Stops"}, {Cat::Stop, "Run_Stops"}};
    for (const auto& cs : cats)
    {
        const std::string tag = std::string(body) + " " + CatName(cs.c);
        const RowPick pk = PickRow(db, cs.suffix, cs.c);
        Check(pk.db >= 0 && pk.metric > 0.0, tag + ": row found in PSD_Dense_Stand_" + cs.suffix);
        if (pk.db < 0)
            continue;
        const uint32_t d = static_cast<uint32_t>(pk.db);
        const motionns::DatabaseRecord& D = db.Db(d);
        const motionns::RowRecord& R = db.Row(pk.row);
        const motionns::EntryRecord& E = db.Entry(R.entry);
        const motionns::MotionClipRecord& C = db.Clip(E.clip);
        Path p;
        p.Build(db, pk.row);
        std::printf("\n== %s: db %s, clip %s, frame %u (local %u of %u%s), metric %.4f; speed(-0.2 s) %.3f, speed(0) %.3f m/s, "
                    "yaw(-0.2 s) %.2f, yaw(0) %.2f deg, velocity heading(0) %.2f deg\n",
                    tag.c_str(), std::string(db.DbName(d)).c_str(), std::string(db.ClipName(E.clip)).c_str(), R.frame,
                    R.frame - C.firstFrame, C.numFrames, (C.flags & motionns::kClipLoop) ? ", loop" : "", pk.metric, p.SpeedFrame(-6),
                    p.SpeedFrame(0), p.yaw[Path::kB - 6] * kR2D, p.yaw[Path::kB] * kR2D, p.VelHeading(0) * kR2D);

        // magnitudes for eps
        double tmax = 0.0, ymax = 0.0;
        for (int h = -2 * Path::kB; h <= 2 * Path::kA; ++h)
        {
            double a, b, w;
            p.At(h, a, b, w);
            tmax = std::fmax(tmax, std::hypot(a, b));
            ymax = std::fmax(ymax, std::fabs(w));
        }
        const double rmax = 2.0 * tmax;
        const double eyaw = 6.0 * kU * (ymax * kR2D + 540.0 + 360.0) / kR2D; // + rotated feed (|R| <= 360)
        const double nb = Path::kB + Path::kA;
        const double bakeE = nb * (6.0 * kU * p.maxStep + kU * tmax) + nb * nb * kU * p.maxStep * (ymax + 1.0);

        const std::vector<Dim> dims = TrajDims(db, d);
        Check(!dims.empty(), tag + ": schema has trajectory dims");
        const uint32_t ndims = db.Schema(D.schema).dims;
        const float* stored = db.Features(d, pk.row - D.firstRow);
        const float* mean = db.NormMean(D.norm);
        const float* scale = db.NormScale(D.norm);
        std::vector<double> fd(ndims), dfeaEps(ndims);
        for (const Dim& dm : dims)
        {
            fd[dm.o] = static_cast<double>(stored[dm.o]) / scale[dm.o] + mean[dm.o];
            dfeaEps[dm.o] = 3.0 * kU * (std::fabs(stored[dm.o] / scale[dm.o]) + std::fabs(mean[dm.o])) + kU * std::fabs(fd[dm.o]);
        }

        // (M) ChooserMatcher on the clip root path
        motionns::TrajSample cl[kQueryTrajMax];
        for (uint32_t i = 0; i < qt.n; ++i)
        {
            const int h = static_cast<int>(std::lround(qt.t[i] * 60.0f));
            double a, b, w;
            p.At(h, a, b, w);
            cl[i] = motionns::TrajSample{qt.t[i], static_cast<float>(a), static_cast<float>(b), 0.0f, static_cast<float>(w)};
        }
        std::vector<float> fc;
        Features(mm, d, cl, qt.n, fc, ndims);
        bool mOk = true;
        double mWorst = 0.0;
        for (const Dim& dm : dims)
        {
            const double e = DimEps(dm, EpsIn{bakeE, rmax, eyaw}, dfeaEps[dm.o], fd[dm.o]);
            const double dv = std::fabs(fc[dm.o] - fd[dm.o]);
            mWorst = std::fmax(mWorst, dv / e);
            if (dv > e)
            {
                mOk = false;
                std::printf("  (M) %s [%u]: clip %+.6f dfea %+.6f |d| %.2e eps %.2e\n", dm.label.c_str(), dm.comp, fc[dm.o], fd[dm.o], dv, e);
            }
        }
        Check(mOk, tag + " (M): ChooserMatcher features of the clip root path = DFEA raw, every trajectory dim");
        std::printf("  (M) worst |clip - dfea| / eps %.3f\n", mWorst);

        // base feed (H = 1)
        const FeedResult base = RunFeed(p, cs.c, qt, FeedSpec{});
        bool tOkT = base.n == qt.n;
        for (uint32_t i = 0; tOkT && i < qt.n; ++i)
            tOkT = base.q[i].t == qt.t[i] && (i == 0 || base.q[i].t > base.q[i - 1].t);
        Check(tOkT, tag + " (T): query sample times = BuildQueryTimes, strictly ascending");
        if (base.n != qt.n)
            continue;
        const double feedE = base.frames * (6.0 * kU * p.maxStep + kU * base.lmax) + 2.0 * kU * base.lmax;
        std::vector<float> fq;
        Features(mm, d, base.q, qt.n, fq, ndims);
        bool hOk = true;
        double hWorst = 0.0;
        double futWorst[5] = {};
        for (const Dim& dm : dims)
        {
            const double e = DimEps(dm, EpsIn{feedE + bakeE, rmax, eyaw}, dfeaEps[dm.o], fd[dm.o]);
            const double dv = std::fabs(fq[dm.o] - fd[dm.o]);
            if (dm.history)
            {
                hWorst = std::fmax(hWorst, dv / e);
                if (dv > e)
                    hOk = false;
            }
            else
                futWorst[dm.kind < 5 ? dm.kind : 0] = std::fmax(futWorst[dm.kind < 5 ? dm.kind : 0], dv);
            std::printf("  %-26s [%u] %s: query %+.5f dfea %+.5f clip %+.5f |q-d| %.2e eps %.2e%s\n", dm.label.c_str(), dm.comp,
                        dm.history ? "hist" : "fut ", fq[dm.o], fd[dm.o], fc[dm.o], dv, e,
                        dm.history ? (dv > e ? " <- FAIL" : "") : " (reported)");
        }
        Check(hOk, tag + " (H): history-side query dims (t <= 0, incl. t = 0 velocity / facing) = DFEA raw");
        std::printf("  (H) worst |query - dfea| / eps %.3f; future-side max |query - dfea|: position %.4f m, velocity %.4f m/s, "
                    "velocity direction %.4f, heading %.4f (prediction model vs clip, reported only)\n",
                    hWorst, futWorst[1], futWorst[2], futWorst[3], futWorst[4]);

        // (S) spaces at history times (world, metres, relative to the current sample = T(H); yaw = game yaw in rad)
        // + (R) rotated / translated feed
        const FeedSpec rot{1, 137.0, 60, {41.0, -23.0}}; // [P] arbitrary yaw (not a multiple of 90) and a 2.8 km/min lead-in
        const FeedResult rf = RunFeed(p, cs.c, qt, rot);
        const double rfE = rf.frames * (6.0 * kU * p.maxStep + kU * rf.lmax) + 2.0 * kU * rf.lmax;
        bool sOk = rf.n == qt.n && base.n == qt.n;
        double sWorst = 0.0;
        for (const FeedResult* fr : {&base, &rf})
        {
            const double Rr = (fr == &rf ? rot.rotDeg : 0.0) / kR2D;
            const double e = 2.0 * (fr == &rf ? rfE : feedE) + 2.0 * 6.0 * kU * rmax + rmax * eyaw;
            double hx, hy, hw;
            p.At(1, hx, hy, hw);
            for (uint32_t i = 0; sOk && i < qt.n; ++i)
            {
                if (qt.t[i] > 0.0f)
                    continue;
                const int h = static_cast<int>(std::lround(qt.t[i] * 60.0f));
                double a, b, w;
                p.At(h, a, b, w);
                const double dx = a - hx, dy = b - hy;
                const double ex = std::cos(Rr) * dx - std::sin(Rr) * dy, ey = std::sin(Rr) * dx + std::cos(Rr) * dy;
                const double dp = std::hypot(fr->q[i].x - ex, fr->q[i].y - ey);
                const double dw = std::fabs(Wrap(fr->q[i].yaw - (w + Rr)));
                sWorst = std::fmax(sWorst, dp / e);
                sOk = dp <= e && dw <= eyaw;
                if (!sOk)
                    std::printf("  (S) t %+.4f: query (%+.5f, %+.5f, yaw %+.5f) expected (%+.5f, %+.5f, yaw %+.5f)\n", qt.t[i],
                                fr->q[i].x, fr->q[i].y, fr->q[i].yaw, ex, ey, Wrap(w + Rr));
            }
        }
        Check(sOk, tag + " (S): history query samples = world-space clip path relative to the current sample, yaw = game yaw "
                         "(base and rotated + translated feed)");
        std::vector<float> fr;
        Features(mm, d, rf.q, qt.n, fr, ndims);
        bool rOk = rf.n == qt.n;
        double rWorst = 0.0;
        for (const Dim& dm : dims)
        {
            const double e = DimEps(dm, EpsIn{feedE + rfE, rmax, eyaw}, 0.0, fq[dm.o]);
            const double dv = std::fabs(fr[dm.o] - fq[dm.o]);
            rWorst = std::fmax(rWorst, dv / e);
            if (dv > e)
            {
                rOk = false;
                std::printf("  (R) %s [%u]: base %+.6f rotated %+.6f |d| %.2e eps %.2e\n", dm.label.c_str(), dm.comp, fq[dm.o], fr[dm.o], dv, e);
            }
        }
        Check(rOk, tag + " (R): rotated (137 deg) + translated feed -> identical raw features, every trajectory dim");
        std::printf("  (S) worst / eps %.3f, (R) worst / eps %.3f, rotated feed max |capsule| %.1f m\n", sWorst, rWorst, rf.lmax);

        // (D) time shift: H = 1 + k, k = -2, -1, +1, +2 must break the history parity
        for (const int k : {-2, -1, 1, 2})
        {
            FeedSpec sh;
            sh.hEnd = 1 + k;
            const FeedResult sf = RunFeed(p, cs.c, qt, sh);
            std::vector<float> fs;
            Features(mm, d, sf.q, qt.n, fs, ndims);
            const double sE = sf.frames * (6.0 * kU * p.maxStep + kU * sf.lmax) + 2.0 * kU * sf.lmax;
            double worst = 0.0;
            std::string at;
            for (const Dim& dm : dims)
            {
                if (!dm.history)
                    continue;
                const double e = DimEps(dm, EpsIn{sE + bakeE, rmax, eyaw}, dfeaEps[dm.o], fd[dm.o]);
                const double r = std::fabs(fs[dm.o] - fd[dm.o]) / e;
                if (r > worst)
                {
                    worst = r;
                    at = dm.label + " [" + std::to_string(dm.comp) + "]";
                }
            }
            std::printf("  (D) feed shifted %+d frame(s): worst history |query - dfea| / eps %.1f at %s\n", k, worst, at.c_str());
            Check(sf.n == qt.n && worst > 1.0, tag + " (D): feed shifted by " + std::to_string(k) + " frame(s) does not match the history dims");
        }

        // (X) sensitivity of the history parity to the other two error classes, on the real query of this row:
        //  (V) velocity space swapped (world <-> character origin of the t - 1/60 read) in every history velocity dim;
        //  (F1) facing taken from the velocity direction instead of the actor: every query yaw + (velocity heading(0) -
        //       facing(0)) [a rotation of the origin frame only, positions not rotated];
        //  (F2) mirrored yaw convention (yaw -> -yaw) on the ROTATED feed (world yaw 137): the history dims of the motion schemas
        //       read the facing only at the origin t = 0 (positions / velocities rotate by it; headings at t_sample = t_origin =
        //       0), and the unrotated feed has yaw(0) = 0, where a mirror is the identity.
        // Asserted where the row was chosen for it: (V) on the moving rows (start, pivot, strafe: speed(0) >= 1.6 m/s by
        // selection; the stop row ends at 0.07 m/s and is reported only), (F1) on start (velocity 135 deg off the facing) and
        // strafe (>= 60 deg by selection), (F2) on every row (each moves in the history window). Others are reported.
        {
            const double eBase = feedE + bakeE;
            auto worstHist = [&](const std::vector<double>& f, std::string& at, double eF) {
                double w = 0.0;
                for (const Dim& dm : dims)
                {
                    if (!dm.history || std::isnan(f[dm.o]))
                        continue;
                    const double r = std::fabs(f[dm.o] - fd[dm.o]) / DimEps(dm, EpsIn{eF, rmax, eyaw}, dfeaEps[dm.o], fd[dm.o]);
                    if (r > w)
                    {
                        w = r;
                        at = dm.label + " [" + std::to_string(dm.comp) + "]";
                    }
                }
                return w;
            };
            std::vector<double> fv(ndims, std::nan(""));
            bool anyVel = false;
            for (const Dim& dm : dims)
                if (dm.history && dm.kind == 2)
                {
                    fv[dm.o] = SwappedVelocity(base.q, qt.n, *dm.ch, dm.comp);
                    anyVel = true;
                }
            std::string atV, atF1, atF2;
            const double wV = worstHist(fv, atV, eBase);
            motionns::TrajSample m1[kQueryTrajMax], m2[kQueryTrajMax];
            const double delta = p.VelHeading(0) - p.yaw[Path::kB];
            for (uint32_t i = 0; i < qt.n; ++i)
            {
                m1[i] = base.q[i];
                m1[i].yaw = static_cast<float>(Wrap(base.q[i].yaw + delta));
                m2[i] = rf.q[i];
                m2[i].yaw = -rf.q[i].yaw;
            }
            std::vector<float> g1, g2;
            Features(mm, d, m1, qt.n, g1, ndims);
            Features(mm, d, m2, qt.n, g2, ndims);
            const double wF1 = worstHist(std::vector<double>(g1.begin(), g1.end()), atF1, eBase);
            const double wF2 = worstHist(std::vector<double>(g2.begin(), g2.end()), atF2, rfE + bakeE); // the rotated feed's own (larger) bound
            std::printf("  (X) worst history |mutated - dfea| / eps: velocity space swapped %.1f at %s; facing from velocity "
                        "(%+.1f deg) %.1f at %s; mirrored yaw %.1f at %s\n",
                        anyVel ? wV : 0.0, atV.c_str(), delta * kR2D, wF1, atF1.c_str(), wF2, atF2.c_str());
            if (cs.c != Cat::Stop)
                Check(anyVel && wV > 1.0, tag + " (V): swapped velocity space is rejected by the history parity");
            if (cs.c == Cat::Start || cs.c == Cat::Strafe)
                Check(wF1 > 1.0, tag + " (F1): facing from the velocity direction is rejected by the history parity");
            if (rf.n == qt.n)
                Check(wF2 > 1.0, tag + " (F2): mirrored yaw (rotated feed) is rejected by the history parity");
        }
    }
    return 0;
}
} // namespace

int main(int argc, char** argv)
{
    std::vector<std::string> dbs, loadOnly;
    for (int i = 1; i < argc; ++i)
        if (std::string_view(argv[i]) == "--db" && i + 1 < argc)
            dbs.push_back(argv[++i]);
        else if (std::string_view(argv[i]) == "--load-only" && i + 1 < argc)
            loadOnly.push_back(argv[++i]);
    Check(!dbs.empty(), "at least one --db <utpp_motion_*.utppmm> given");
    for (const std::string& p : dbs)
        RunBody(p);
    for (const std::string& p : loadOnly)
        RunBody(p, true);
    std::printf("query parity test: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
