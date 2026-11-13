// utpp_mm_montage_test: report 4.2 #32-#34 offline checks (BlendSpace.hpp; DeadBlending/Montage pending).
// Usage: utpp_mm_montage_test <Data/anim/blendspaces.txt>
// The REF rows are UE reference weights computed by tools/mmbake/export_blendspaces.py (independent brute-force
// Delaunay + Tangential flip, see its header for the UE source lines).
#include "../src/mm/BlendSpace.hpp"
#include "../src/mm/DeadBlending.hpp"
#include "../src/mm/Montage.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace utpp::mm;

static int g_pass = 0, g_fail = 0;
#define CHECK(c, ...) do { if (c) ++g_pass; else { ++g_fail; std::printf("FAIL %s:%d ", __FILE__, __LINE__); std::printf(__VA_ARGS__); std::printf("\n"); } } while (0)

struct RefRow { float x, y; std::map<int, float> w; };
struct BsFile { bs::BlendSpaceDef def; std::vector<RefRow> refs; };

static bool LoadBs(const char* path, std::vector<BsFile>& out) {
    std::ifstream f(path);
    if (!f) return false;
    std::string line;
    if (!std::getline(f, line) || line.rfind("utpp-blendspaces 1", 0) != 0) return false;
    BsFile cur; bool in = false;
    while (std::getline(f, line)) {
        std::istringstream ss(line); std::string tag; ss >> tag;
        if (tag == "BS") {
            int ease = 1, n = 0;
            ss >> cur.def.name >> cur.def.dims >> cur.def.min[0] >> cur.def.max[0] >> cur.def.min[1] >> cur.def.max[1]
               >> cur.def.speedPerSec >> ease >> n;
            cur.def.easeInOut = ease != 0; cur.def.samples.clear(); cur.refs.clear(); in = true;
        } else if (tag == "S" && in) {
            bs::Sample s; ss >> s.x >> s.y >> s.anim; cur.def.samples.push_back(s);
        } else if (tag == "REF" && in) {
            RefRow r; int n = 0; ss >> r.x >> r.y >> n;
            for (int i = 0; i < n; ++i) { int k; float w; ss >> k >> w; r.w[k] = w; }
            cur.refs.push_back(r);
        } else if (tag == "END" && in) { out.push_back(cur); in = false; }
    }
    return !out.empty();
}

static void TestBlendSpaces(const char* path) {
    std::vector<BsFile> files;
    const bool loaded = LoadBs(path, files);
    CHECK(loaded, "load %s", path);
    if (!loaded) return;
    CHECK(files.size() == 2, "2 blend spaces (got %zu)", files.size());
    for (const auto& bf : files) {
        bs::BlendSpaceEval ev;
        CHECK(ev.Build(bf.def), "build %s", bf.def.name.c_str());
        CHECK(bf.refs.size() == 20, "%s: 20 reference points (got %zu)", bf.def.name.c_str(), bf.refs.size());
        float maxErr = 0.f;
        for (const auto& r : bf.refs) {
            const auto got = ev.Evaluate(r.x, r.y);
            std::map<int, float> g; float sum = 0.f;
            for (const auto& w : got) { g[w.sample] += w.weight; sum += w.weight; }
            bool same = g.size() == r.w.size();
            for (const auto& [k, w] : r.w) {
                const float e = std::fabs((g.count(k) ? g[k] : 0.f) - w);
                maxErr = std::max(maxErr, e);
                same = same && e < 1e-4f;
            }
            CHECK(same, "%s at (%.2f, %.2f): weights differ from UE reference", bf.def.name.c_str(), r.x, r.y);
            CHECK(std::fabs(sum - 1.f) < 1e-4f, "%s at (%.2f, %.2f): weight sum %.6f", bf.def.name.c_str(), r.x, r.y, sum);
        }
        std::printf("BS %s dims=%d samples=%zu refs=%zu maxErr=%.2e\n", bf.def.name.c_str(), bf.def.dims,
                    bf.def.samples.size(), bf.refs.size(), maxErr);
        // smoothing: ease-in-out converges to the target, never overshoots past 1, and is frame-rate stable
        for (float fps : {30.f, 60.f, 144.f}) {
            bs::WeightSmoother sm; sm.Reset(bf.def.samples.size());
            sm.Update(ev.Evaluate(bf.def.min[0], bf.def.min[1]), 1.f / fps, bf.def.speedPerSec, bf.def.easeInOut);
            const auto tgt = ev.Evaluate(bf.def.max[0], bf.def.max[1]);
            std::vector<float> w;
            const int frames = int(fps * 3.f);
            for (int i = 0; i < frames; ++i) w = sm.Update(tgt, 1.f / fps, bf.def.speedPerSec, bf.def.easeInOut);
            const int ti = tgt.front().sample;
            CHECK(w[ti] > 0.99f, "%s smoothing %g fps: weight after 3 s %.4f", bf.def.name.c_str(), fps, w[ti]);
        }
    }
    // negative control: a non-grid 2D sample set is rejected (fail closed)
    bs::BlendSpaceDef bad; bad.dims = 2; bad.min[0] = bad.min[1] = -1; bad.max[0] = bad.max[1] = 1;
    bad.samples = {{0, 0, "a"}, {1, 0, "b"}, {0.3f, 1, "c"}};
    bs::BlendSpaceEval bev;
    CHECK(!bev.Build(bad), "non-grid 2D blend space must be rejected");
}

// DeadBlending continuity: source bone moves at constant velocity (translation 300 cm/s on X, yaw 180 deg/s), at the
// transition the destination is a fixed pose. Checks per fps: no pop at the switch, bounded per-frame change, exact
// destination after the blend, frame-rate independence of the closed form at shared times, velocity clamp.
static void TestDeadBlending() {
    using namespace deadblend;
    const float kPi = 3.14159265358979f;
    auto srcAt = [&](float t) { BoneTR b; b.t = {300.f * t, 0, 100}; b.r = FromRotVec({0, 0, kPi * t}); return b; };
    BoneTR dst; dst.t = {80.f, 20.f, 90.f}; dst.r = FromRotVec({0, 0, -0.6f});
    std::map<int, BoneTR> at60;   // pose at t = k/30 s, used to compare fps
    for (float fps : {30.f, 60.f, 144.f}) {
        const float dt = 1.f / fps, t0 = 1.0f;
        Settings s; s.blend = BlendOption::Cubic;
        DeadBlender db;
        std::vector<BoneTR> curr{srcAt(t0)}, prev{srcAt(t0 - dt)}, d{dst};
        db.Start(curr, prev, dt, d, s);
        std::vector<BoneTR> pose{dst};
        db.Apply(pose, 0.f);
        const float pop = Len(pose[0].t - curr[0].t) + Len(ToRotVec(Mul(pose[0].r, Inv(curr[0].r))));
        CHECK(pop < 1e-4f, "deadblend %g fps: pop at the switch %.6f", fps, pop);
        BoneTR last = pose[0];
        float maxStepT = 0.f, maxStepR = 0.f;
        const int frames = int(std::ceil(0.4f * fps));
        for (int f = 1; f <= frames; ++f) {
            std::vector<BoneTR> p{dst};
            db.Apply(p, dt);
            maxStepT = std::max(maxStepT, Len(p[0].t - last.t));
            maxStepR = std::max(maxStepR, Len(ToRotVec(Mul(p[0].r, Inv(last.r)))));
            last = p[0];
            const float tt = f * dt;
            const int k = int(std::lround(tt * 30.f));
            if (std::fabs(tt * 30.f - k) < 1e-3f && k <= 6) {
                if (fps == 30.f) at60[k] = p[0];
                else {
                    const float e = Len(p[0].t - at60[k].t) + Len(ToRotVec(Mul(p[0].r, Inv(at60[k].r))));
                    CHECK(e < 1e-3f, "deadblend %g fps vs 30 fps at t=%d/30 s: diff %.6f", fps, k, e);
                }
            }
        }
        // per-frame bound: dst-src gap (~240 cm, ~3.7 rad) over 0.25 s with cubic peak slope 1.5 + source velocity
        const float boundT = (1.5f * 260.f / 0.25f + 300.f) * dt * 1.05f;
        const float boundR = (1.5f * 3.8f / 0.25f + kPi) * dt * 1.05f;
        CHECK(maxStepT < boundT, "deadblend %g fps: max translation step %.3f >= %.3f", fps, maxStepT, boundT);
        CHECK(maxStepR < boundR, "deadblend %g fps: max rotation step %.4f >= %.4f", fps, maxStepR, boundR);
        const float endErr = Len(last.t - dst.t) + Len(ToRotVec(Mul(last.r, Inv(dst.r))));
        CHECK(endErr < 1e-4f && !db.Active(), "deadblend %g fps: end error %.6f active %d", fps, endErr, db.Active());
        std::printf("DeadBlend %g fps: frames %d maxStep %.3f cm / %.4f rad (bounds %.3f / %.4f) end %.2e\n", fps, frames,
                    maxStepT, maxStepR, boundT, boundR, endErr);
    }
    // velocity clamp: a 50 m/s source is extrapolated as 500 cm/s at most
    {
        Settings s; s.halfLife = 1.f; s.halfLifeMin = 1.f; s.halfLifeMax = 1.f;   // fixed half-life
        DeadBlender db;
        BoneTR a; a.t = {0, 0, 0}; BoneTR b; b.t = {5000.f / 60.f, 0, 0};
        std::vector<BoneTR> curr{b}, prev{a}, d{b};
        db.Start(curr, prev, 1.f / 60.f, d, s);
        std::vector<BoneTR> p{b};
        s.blendDuration = 0.25f;
        db.Apply(p, 0.1f);   // alpha = 1 - 0.4 = 0.6, extrapolated = b + 500/c (1 - InvExpApprox(c 0.1))
        const float c = kLn2 / 1.f, ext = 500.f / c * (1.f - InvExpApprox(c * 0.1f));
        const float want = b.t.x + 0.6f * ext;
        CHECK(std::fabs(p[0].t.x - want) < 1e-2f, "deadblend velocity clamp: x %.4f want %.4f", p[0].t.x, want);
    }
    // blend options (AlphaBlend.cpp 244-247)
    CHECK(std::fabs(AlphaToBlendOption(0.25f, BlendOption::Cubic) - 0.15625f) < 1e-6f, "cubic alpha");
    CHECK(std::fabs(AlphaToBlendOption(0.25f, BlendOption::Linear) - 0.25f) < 1e-6f, "linear alpha");
}

// Montage player on 2 real exported traversal montages (Data/anim/montages.txt from export_blendspaces.py).
static void TestMontages(const char* path) {
    using namespace montage;
    std::vector<MontageDef> defs; std::string err;
    const bool ok = LoadMontages(path, defs, &err);
    CHECK(ok && defs.size() == 2, "load %s: %s (%zu montages)", path, err.c_str(), defs.size());
    if (!ok) return;
    for (const auto& d : defs) {
        CHECK(d.slot == "DefaultSlot" && d.sections.size() == 1 && d.segments.size() == 1, "%s: slot/sections", d.name.c_str());
        const float rootLen = float(d.root.size() - 1) / d.rootFps;
        CHECK(std::fabs(rootLen - d.length) < 0.05f, "%s: root track %.3f s vs length %.3f s", d.name.c_str(), rootLen, d.length);
        const Xf total = Compose(Inverse(SampleRoot(d, 0.f)), SampleRoot(d, d.length));
        const float dist = std::sqrt(total.t[0] * total.t[0] + total.t[1] * total.t[1] + total.t[2] * total.t[2]);
        CHECK(dist > 0.5f, "%s: root displacement %.3f m", d.name.c_str(), dist);
        for (float fps : {30.f, 60.f, 144.f}) {
            MontagePlayer mp;
            struct Ev { Event e; std::string n; float pos; };
            std::vector<Ev> evs;
            mp.SetListener([&](Event e, const Notify* n) { evs.push_back({e, n ? n->name : "", mp.Position()}); });
            mp.Play(&d, 1.f, 0.f);
            Xf acc; float prevW = mp.Weight(), maxDW = 0.f, wAtIn = -1.f, blendOutPos = -1.f;
            int frames = 0;
            const float dt = 1.f / fps;
            while (mp.Update(dt) && frames < 100000) {
                ++frames;
                acc = Compose(acc, mp.RootDelta());
                maxDW = std::max(maxDW, std::fabs(mp.Weight() - prevW)); prevW = mp.Weight();
                if (wAtIn < 0.f && frames * dt >= d.blendInTime - 1e-4f) wAtIn = mp.Weight();
                if (blendOutPos < 0.f && mp.BlendingOut()) blendOutPos = mp.Position();
            }
            acc = Compose(acc, mp.RootDelta());
            float e = 0.f; for (int i = 0; i < 3; ++i) e = std::max(e, std::fabs(acc.t[i] - total.t[i]));
            CHECK(e < 1e-3f, "%s %g fps: accumulated root motion off by %.5f m", d.name.c_str(), fps, e);
            CHECK(std::fabs(wAtIn - 1.f) < 1e-4f, "%s %g fps: weight after blend-in %.4f", d.name.c_str(), fps, wAtIn);
            CHECK(maxDW <= 1.5f * dt / 0.25f + 1e-4f, "%s %g fps: weight step %.4f (HermiteCubic 0.25 s bound %.4f)",
                  d.name.c_str(), fps, maxDW, 1.5f * dt / 0.25f);
            CHECK(std::fabs(blendOutPos - (d.length - d.blendOutTime)) <= dt + 1e-4f, "%s %g fps: auto blend-out at %.4f (want %.4f)",
                  d.name.c_str(), fps, blendOutPos, d.length - d.blendOutTime);
            // each notify state: exactly one Begin and one End, inside one frame of its window
            int states = 0, good = 0;
            for (const auto& n : d.notifies) {
                if (!n.state) continue;
                ++states;
                int b = 0, en = 0; float bp = -1, ep = -1;
                for (const auto& v : evs) {
                    if (v.n != n.name) continue;
                    if (v.e == Event::NotifyBegin && std::fabs(v.pos - n.start) <= dt + 1e-4f) { ++b; bp = v.pos; }
                    if (v.e == Event::NotifyEnd) { ++en; ep = v.pos; }
                }
                // same-named states (2 MotionWarping windows) are counted per window by position
                (void)bp; (void)ep;
                if (b == 1 && en >= 1) ++good;
            }
            CHECK(good == states, "%s %g fps: notify windows begin/end %d of %d", d.name.c_str(), fps, good, states);
            int nBegin = 0, nEnd = 0, nBlendOut = 0, nDone = 0, nInt = 0;
            for (const auto& v : evs) {
                nBegin += v.e == Event::NotifyBegin; nEnd += v.e == Event::NotifyEnd; nBlendOut += v.e == Event::BlendOut;
                nDone += v.e == Event::Completed; nInt += v.e == Event::Interrupted;
            }
            CHECK(nBegin == states && nEnd == states && nBlendOut == 1 && nDone == 1 && nInt == 0,
                  "%s %g fps: events begin %d end %d blendOut %d completed %d interrupted %d", d.name.c_str(), fps,
                  nBegin, nEnd, nBlendOut, nDone, nInt);
            if (fps == 60.f)
                std::printf("Montage %s: %d frames @60, root %.3f m, notify states %d, blend-out at %.3f s\n", d.name.c_str(),
                            frames, dist, states, blendOutPos);
        }
        // Montage_StopWithBlendSettings 0.2 s HermiteCubic in the middle of the first MotionWarping window,
        // start time 0.1 s (MotionMatch start time) and rate 1.2
        {
            MontagePlayer mp; int nInt = 0, nDone = 0, nOut = 0, open = 0;
            mp.SetListener([&](Event e, const Notify*) {
                nInt += e == Event::Interrupted; nDone += e == Event::Completed; nOut += e == Event::BlendOut;
                open += e == Event::NotifyBegin ? 1 : e == Event::NotifyEnd ? -1 : 0; });
            mp.Play(&d, 1.2f, 0.1f);
            // first notify state whose middle is reached after the blend-in and before the auto blend-out
            const Notify* win = nullptr;
            for (const auto& n : d.notifies) {
                const float mid = n.start + 0.5f * n.duration;
                if (n.state && (mid - 0.1f) / 1.2f >= d.blendInTime + 0.02f && mid < d.length - d.blendOutTime) { win = &n; break; }
            }
            CHECK(win != nullptr, "%s stop: no notify window to stop in", d.name.c_str());
            if (!win) continue;
            const float dt = 1.f / 60.f;
            while (mp.Position() < win->start + 0.5f * win->duration && mp.Update(dt)) {}
            const float w0 = mp.Weight();
            const int openBefore = open;
            mp.Stop(0.2f, Blend::HermiteCubic);
            int after = 1;   // counts every Update until the one that ends the montage
            while (mp.Update(dt)) ++after;
            CHECK(w0 > 0.99f && openBefore >= 1 && nInt == 1 && nDone == 0 && nOut == 1 && open == 0 && std::abs(after - 12) <= 1,
                  "%s stop in %s at %.3f s: w0 %.3f open before %d, interrupted %d completed %d blendOut %d open after %d, "
                  "updates to 0 = %d (want 12 = 0.2 s @60)", d.name.c_str(), win->name.c_str(), mp.Position(), w0, openBefore,
                  nInt, nDone, nOut, open, after);
            std::printf("Montage stop %s in %s: w0 %.3f, updates to weight 0 = %d\n", d.name.c_str(), win->name.c_str(), w0, after);
        }
    }
}

int main(int argc, char** argv) {
    TestBlendSpaces(argc > 1 ? argv[1] : "blendspaces.txt-NOT-SET");
    TestDeadBlending();
    TestMontages(argc > 2 ? argv[2] : "montages.txt-NOT-SET");
    std::printf("utpp_mm_montage_test: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
