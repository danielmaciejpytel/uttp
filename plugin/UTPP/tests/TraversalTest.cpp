// UTPP motion stage 10.8 - offline test of src/mm/BaseTraversal.hpp (no game hooks).
// Usage: utpp_mm_traversal_test <traversal_table.json> <traversal_vectors.txt> <traversal_oracle.json>
//  1) decision table vs the Python reference vectors (traversal_ref.py vectors) - 100% agreement required;
//  2) decision table + CBP gate vs the independent oracle (traversal_oracle.json) - 100%;
//  3) synthetic smoothstep mantle: warp point error at FrontLedge window end <= 5 cm for a grid of ledge offsets,
//     no per-frame root step > 0.30 m, exit continuity (root velocity at clip end vs walk velocity);
//  4) CP obstacle classification (strict '>' like ClimbEvents) and height scale mapping.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "../src/mm/BaseTraversal.hpp"

using namespace utpp::mm::traversal;

static int g_pass = 0, g_fail = 0;
static void Check(bool aOk, const std::string& aWhat)
{
    if (aOk)
        ++g_pass;
    else
    {
        ++g_fail;
        std::printf("FAIL: %s\n", aWhat.c_str());
    }
}

static bool ReadAll(const char* aPath, std::string& aOut)
{
    std::ifstream f(aPath, std::ios::binary);
    if (!f)
        return false;
    std::ostringstream ss;
    ss << f.rdbuf();
    aOut = ss.str();
    return true;
}

static const char* StatusName(Status s)
{
    switch (s)
    {
    case Status::Ok: return "ok";
    case Status::CheckFailed: return "check_failed";
    default: return "montage_failed";
    }
}

static int TestVectors(const DecisionTable& t, const char* path)
{
    std::ifstream f(path);
    Check(static_cast<bool>(f), std::string("open vectors ") + path);
    if (!f)
        return 0;
    std::string line;
    int n = 0, bad = 0;
    while (std::getline(f, line))
    {
        if (line.empty() || line[0] == '#')
            continue;
        std::istringstream ls(line);
        int fr, bk, fl, mm;
        Inputs in;
        std::string bar, status, action, mons;
        ls >> fr >> bk >> fl >> in.obstacleDepth >> in.backLedgeHeight >> in.obstacleHeight >> in.speed >> mm >> bar >>
            status >> action >> mons;
        in.hasFrontLedge = fr != 0;
        in.hasBackLedge = bk != 0;
        in.hasBackFloor = fl != 0;
        in.movementMode = mm;
        const Decision d = t.Evaluate(in);
        std::string got;
        for (size_t i = 0; i < d.montages.size(); ++i)
            got += (i ? "," : "") + d.montages[i];
        if (got.empty())
            got = "-";
        ++n;
        if (status != StatusName(d.status) || action != d.actionName || mons != got)
        {
            if (++bad <= 10)
                std::printf("vector mismatch: %s -> %s %s %s\n", line.c_str(), StatusName(d.status),
                            d.actionName.c_str(), got.c_str());
        }
    }
    std::printf("python vectors: %d/%d agree\n", n - bad, n);
    Check(n > 0 && bad == 0, "decision table == traversal_ref.py vectors (100%)");
    return n;
}

static void TestOracle(const DecisionTable& t, const char* path)
{
    std::string text, err;
    Check(ReadAll(path, text), std::string("read oracle ") + path);
    utpp::json::Value o;
    if (!utpp::json::Parser::Parse(text, o, err))
    {
        Check(false, "parse oracle: " + err);
        return;
    }
    const utpp::json::Value* vecs = o.Find("vectors");
    Check(vecs && vecs->IsArray() && !vecs->arr.empty(), "oracle has vectors");
    if (!vecs)
        return;
    int n = 0, bad = 0;
    for (const auto& v : vecs->arr)
    {
        const auto* in = v.Find("inputs");
        const auto* ex = v.Find("expected");
        const auto* id = v.Find("id");
        if (!in || !ex)
        {
            ++bad;
            continue;
        }
        auto B = [&](const char* k) {
            const auto* x = in->Find(k);
            return x && x->b;
        };
        auto N = [&](const char* k) {
            const auto* x = in->Find(k);
            return x ? x->num : 0.0;
        };
        Inputs ip;
        ip.hasFrontLedge = B("has_front_ledge") && !B("room_check_blocked"); // BP sets HasFrontLedge=false
        ip.hasBackLedge = B("has_back_ledge");
        ip.hasBackFloor = B("has_back_floor");
        ip.obstacleDepth = N("obstacle_depth");
        ip.obstacleHeight = N("obstacle_height");
        ip.backLedgeHeight = N("back_ledge_height");
        ip.speed = N("speed");
        ip.movementMode = static_cast<int>(N("movement_mode_value"));
        std::string outcome;
        Decision d;
        if (B("doing_traversal_action"))
        {
            outcome = "skip"; // IA_Jump only calls TryTraversalAction when !DoingTraversalAction
            d.actionName = t.ActionName(0);
        }
        else
        {
            d = t.Evaluate(ip);
            outcome = d.status == Status::Ok ? "traverse"
                      : d.status == Status::CheckFailed ? "jump:TraversalCheckFailed"
                                                        : "jump:MontageSelectionFailed";
        }
        std::set<std::string> got(d.montages.begin(), d.montages.end()), exp;
        if (const auto* m = ex->Find("montages"))
            for (const auto& x : m->arr)
                exp.insert(x.str);
        const auto* ea = ex->Find("action_type");
        const auto* eo = ex->Find("outcome");
        ++n;
        if (!ea || ea->str != d.actionName || !eo || eo->str != outcome || got != exp)
        {
            ++bad;
            std::printf("oracle mismatch %s: got %s %s (%zu montages)\n", id ? id->str.c_str() : "?",
                        d.actionName.c_str(), outcome.c_str(), got.size());
        }
    }
    std::printf("oracle: %d/%d agree\n", n - bad, n);
    Check(n > 0 && bad == 0, "decision table + gate == oracle (100%)");
}

static double Smooth(double x)
{
    x = std::clamp(x, 0.0, 1.0);
    return x * x * (3 - 2 * x);
}

// Synthetic mantle clip at 30 fps: approach at walk speed, smoothstep rise and forward move, walk-out.
static void TestWarp()
{
    const double fps = 30.0, walk = 1.5; // m/s
    const int frames = 61;               // 2 s
    std::vector<RootKey> clip(frames);
    const double animLedgeH = 1.0, animLedgeX = 0.9; // ledge in clip space (synthetic)
    for (int f = 0; f < frames; ++f)
    {
        const double t = f / fps;
        const double rise = Smooth((t - 0.3) / 0.7);
        const double x = walk * 0.3 * Smooth(t / 0.3 * 0.5) * 2.0 * 0 + walk * t * 0.5 + 0.45 * rise;
        clip[f].pos = {x, 0.0, animLedgeH * rise};
        clip[f].yaw = 0.0;
    }
    // FrontLedge window: frames 6..21 (0.2 s .. 0.7 s); warp point = hand/attach offset from root at window end
    const int ws = 6, we = 21;
    const V3 attachLocal = {animLedgeX - clip[we].pos.x, 0.0, animLedgeH - clip[we].pos.z + 0.0};
    double worstErr = 0, worstStep = 0, worstExitDv = 0;
    int cases = 0;
    for (double dx = -0.30; dx <= 0.301; dx += 0.10)
        for (double dz = -0.40; dz <= 0.401; dz += 0.20)
            for (double yawDeg = -20; yawDeg <= 20.1; yawDeg += 10)
            {
                const double yaw = yawDeg * 3.141592653589793 / 180.0;
                RootKey start{{2.0, -1.0, 0.0}, 0.0};
                WarpWindow w;
                w.startFrame = ws;
                w.endFrame = we;
                w.targetYaw = yaw;
                // ledge in world: nominal ledge rotated by target yaw around start + offsets
                w.target = start.pos + RotZ({animLedgeX + dx, 0, animLedgeH + dz}, 0.0);
                w.warpPointOffset = attachLocal;
                const auto out = SolveWarp(clip, start, {w});
                const V3 hand = out[we].pos + RotZ(attachLocal, out[we].yaw);
                worstErr = std::max(worstErr, Len(hand - w.target));
                for (int f = 1; f < frames; ++f)
                    worstStep = std::max(worstStep, Len(out[f].pos - out[f - 1].pos));
                // exit continuity: last-frame velocity vs animated walk velocity (magnitude)
                const double vOut = Len(out[frames - 1].pos - out[frames - 2].pos) * fps;
                const double vAnim = Len(clip[frames - 1].pos - clip[frames - 2].pos) * fps;
                worstExitDv = std::max(worstExitDv, std::fabs(vOut - vAnim));
                ++cases;
            }
    std::printf("warp: %d cases, worst FrontLedge end error %.4f m, worst root step %.4f m/frame, worst exit |dv| %.4f m/s\n",
                cases, worstErr, worstStep, worstExitDv);
    Check(worstErr <= 0.05, "warp point within 5 cm of the ledge at FrontLedge window end");
    Check(worstStep <= 0.30, "no root teleport > 0.30 m per frame");
    Check(worstExitDv <= 0.05, "exit velocity continuity (|dv| <= 0.05 m/s)");
}

static void TestCp()
{
    // thresholds are game data (inputs); strict '>' semantics only
    Check(ClassifyCpObstacle(1.21, 1.2, 0.8) == CpClimbClass::High, "cp high");
    Check(ClassifyCpObstacle(1.2, 1.2, 0.8) == CpClimbClass::Mid, "cp boundary high -> mid");
    Check(ClassifyCpObstacle(0.8, 1.2, 0.8) == CpClimbClass::Low, "cp boundary mid -> low");
    const HeightMapping m = MapObstacleHeight(1.1, 1.0, 0.8, 1.25);
    Check(m.ok && std::fabs(m.scale - 1.1) < 1e-12, "height scale 1.1");
    Check(!MapObstacleHeight(2.0, 1.0, 0.8, 1.25).ok, "height scale out of range rejected");
}

int main(int argc, char** argv)
{
    if (argc < 4)
    {
        std::printf("usage: %s <traversal_table.json> <traversal_vectors.txt> <traversal_oracle.json>\n", argv[0]);
        return 2;
    }
    std::string text, err;
    DecisionTable t;
    Check(ReadAll(argv[1], text), std::string("read table ") + argv[1]);
    const bool loaded = t.Load(text, err);
    Check(loaded, "load table: " + err);
    if (loaded)
    {
        TestVectors(t, argv[2]);
        TestOracle(t, argv[3]);
    }
    TestWarp();
    TestCp();
    std::printf("traversal test: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
