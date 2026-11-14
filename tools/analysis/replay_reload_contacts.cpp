// Offline replay of the reload contact stage of the pose hook on recorded text pose recordings (utpp-pose-<ticks>.csv).
// Per frame: stage 0 records (the game's pose) -> Capture, stage 1 records (after the motion blend) -> Finish / HoldOnly, with
// the production continuity state and the correction weight of the driver's reload envelope and correction fade
// (ReloadContactFade.hpp), stepped with the recorded frame times. The admission is reload_contacts_runtime::Admit with the
// owner given on the command line, i.e. the same predicate the hook evaluates.
//
// usage: replay_reload_contacts <pose.csv> <overlays.utppov> <windows.txt> <out.csv> <frames.csv> [owner: fk|mapping|off]
//   windows.txt: one window per line "<first tick> <last tick> <label>" (QPC ticks of the recording)
//   out.csv:     the recording's own rows of every replayed frame, the s2 columns replaced by the replayed output records
//                (same schema as the input, so the recording loaders read it unchanged)
//   frames.csv:  one line per frame: label,t,action,overlay,envelope,weight,admitted,captured,written,reject,
//                lh_rel_cm,rh_rel_cm,lh_rel_base_cm,cost_us
//                lh_rel_cm = distance of the left hand in the WeaponRight frame between the output and the game's pose (stage 0):
//                0 = the hand sits on the weapon where the game's reload puts it (magazine well, magazine); *_base = stage 1.
// Environment: REPLAY_PISTOL_MOVING=1 treats every pistol frame as moving (no correction; the stationary rule).
#include "../../plugin/UTPP/src/mm/NativeReloadContactsRuntime.hpp"
#include "../../plugin/UTPP/src/mm/OverlayFile.hpp"
#include "../../plugin/UTPP/src/mm/ReloadContactFade.hpp"
#include <windows.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
using namespace utpp::mm;
namespace rt = reload_contacts_runtime;

namespace
{
struct Row
{
    std::string text;
    long long ticks = 0, seq = 0;
    int action = 0, overlay = 0, aiming = 0, bone = 0, gameIndex = -1, masked = 0;
    float weight = 0, upper = 0;
    float s[3][12];
    float mm[7];
};

bool ParseRow(const std::string& line, Row& r)
{
    const char* p = line.c_str();
    char* end = nullptr;
    double f[58];
    int n = 0;
    while (*p && n < 58)
    {
        f[n] = std::strtod(p, &end);
        if (end == p) { f[n] = std::nan(""); while (*p && *p != ',') ++p; }
        else p = end;
        ++n;
        if (*p == ',') ++p; else break;
    }
    if (n < 58) return false;
    r.text = line;
    r.ticks = (long long)f[0]; r.seq = (long long)f[4]; r.action = (int)f[6]; r.overlay = (int)f[7]; r.aiming = (int)f[8];
    r.weight = (float)f[10]; r.upper = (float)f[11]; r.bone = (int)f[12]; r.gameIndex = (int)f[13]; r.masked = (int)f[14];
    for (int s = 0; s < 3; ++s) for (int k = 0; k < 12; ++k) r.s[s][k] = (float)f[15 + s * 12 + k];
    for (int k = 0; k < 7; ++k) r.mm[k] = (float)f[51 + k];
    return true;
}

struct Window { long long a = 0, b = 0; std::string label; };

// Left / right hand position in the WeaponRight frame (model space of the pose).
math::Vec3 HandInWeapon(const rt::Prepared& p, const ov::Pose& pose, int hand)
{
    const auto w = native_reload::Model(p.skeleton, pose, p.arms.weaponRight);
    const auto h = native_reload::Model(p.skeleton, pose, hand);
    return math::QuatRotate(math::QuatInv(w.r), h.t - w.t);
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 6)
    {
        std::fprintf(stderr, "usage: pose.csv overlays.utppov windows.txt out.csv frames.csv [fk|mapping|off]\n");
        return 2;
    }
    const std::string ownerArg = argc > 6 ? argv[6] : "fk";
    const bool ownerMapping = ownerArg == "mapping";
    const bool solveOff = ownerArg == "off";
    const bool pistolMoving = std::getenv("REPLAY_PISTOL_MOVING") != nullptr;
    ov::OverlayFile file;
    std::string error;
    if (!file.LoadFromFile(argv[2], ov::kRigIdentityWomanOv, error) &&
        !file.LoadFromFile(argv[2], ov::kRigIdentityManOv, error))
    {
        std::cerr << error;
        return 2;
    }
    const auto& sk = file.Skel();
    std::vector<int32_t> coldMap(size_t(sk.n));
    for (int i = 0; i < sk.n; ++i) coldMap[size_t(i)] = i;
    rt::Prepared prepared;
    if (!rt::FillPrepared(prepared, 0x1234, uint32_t(sk.n), coldMap.data(), [&](uint32_t i) { return int(sk.parent[i]); },
                          [&](uint32_t i) { return file.BoneName(int(i)); }))
    {
        std::cerr << "rig invalid\n";
        return 2;
    }
    std::vector<Window> windows;
    {
        std::ifstream w(argv[3]);
        Window x;
        while (w >> x.a >> x.b >> x.label) windows.push_back(x);
    }
    if (windows.empty()) { std::cerr << "no windows\n"; return 2; }
    std::sort(windows.begin(), windows.end(), [](const Window& l, const Window& r) { return l.a < r.a; });
    std::ifstream in(argv[1]);
    std::ofstream out(argv[4], std::ios::binary);
    FILE* frames = std::fopen(argv[5], "w");
    if (!in || !out || !frames) { std::cerr << "cannot open files\n"; return 2; }
    std::string line;
    for (int k = 0; k < 2 && std::getline(in, line); ++k) out << line << "\n"; // comment + header, unchanged
    std::fprintf(frames, "label,t,action,overlay,envelope,weight,admitted,captured,written,reject,lh_rel_cm,rh_rel_cm,lh_rel_base_cm,cost_us\n");
    LARGE_INTEGER qpf;
    QueryPerformanceFrequency(&qpf);

    size_t wi = 0;
    std::vector<Row> frame;
    long long curTicks = -1;
    // Driver-side state (MotionDriver FrameInputsLocked) and hook-side state (FkHook g_continuity), reset per window.
    rt::UpperEnvelope envelope;
    rt::CorrectionFade fade;
    rt::Continuity continuity;
    long long prevTicks = 0;
    int prevAiming = 0;
    size_t activeWindow = size_t(-1);
    int replayed = 0, written = 0;
    std::vector<uint8_t> rec;
    static const float kIdentity[12] = {0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1};
    const auto put = [&](int stage) {
        for (const Row& r : frame)
            if (r.gameIndex >= 0) std::memcpy(rec.data() + size_t(r.gameIndex) * 48, r.s[stage], 48);
    };

    const auto flush = [&]() {
        if (frame.empty()) return;
        const Row& h = frame[0];
        while (wi < windows.size() && h.ticks > windows[wi].b) ++wi;
        if (wi >= windows.size() || h.ticks < windows[wi].a) { frame.clear(); return; }
        if (activeWindow != wi)
        {
            activeWindow = wi; envelope.Reset(); fade.Reset(); continuity.Reset(); prevTicks = 0; prevAiming = h.aiming;
        }
        const float dt = prevTicks ? float(double(h.ticks - prevTicks) / 1e7) : 1.0f / 60.0f;
        prevTicks = h.ticks;
        const bool pistol = h.overlay == int(ov::PoseOverlay::Pistol1H) || h.overlay == int(ov::PoseOverlay::Pistol2H);
        const bool rifle = h.overlay == int(ov::PoseOverlay::Rifle);
        // Driver: native reload envelope, then the correction fade (rifle: every ground stance; pistols: stationary only).
        const float env = envelope.Step(dt, h.action == 2, prevAiming == 2, pistol);
        prevAiming = h.aiming;
        const bool domain = (h.action == 2 || h.action == 0) && rt::ReloadContactsOverlay(h.overlay);
        const bool correctionPose = rifle || (pistol && !pistolMoving);
        fade.Step(dt, env, correctionPose, domain, true);
        const bool eligible = domain && fade.Active();

        int maxGame = -1;
        for (const Row& r : frame) maxGame = std::max(maxGame, r.gameIndex);
        const uint32_t count = uint32_t(maxGame + 1);
        rec.assign(size_t(count) * 48, 0);
        std::memcpy(rec.data(), kIdentity, 48);
        if (count > 1) std::memcpy(rec.data() + 48, kIdentity, 48);
        integ::PoseSnapshot snap{};
        snap.rig = prepared.rig; snap.numBones = uint32_t(sk.n); snap.firstBone = 2; snap.weight = h.weight; snap.upperScale = h.upper;
        snap.traceAction = h.action; snap.traceOverlay = h.overlay; snap.sequence = uint64_t(h.seq);
        snap.reloadContactsEligible = eligible; snap.reloadAuthoredClip = false;
        snap.reloadContactsWeight = eligible ? fade.weight : 0.0f;
        for (int i = 0; i < sk.n; ++i) snap.gameIndex[i] = i >= 2 ? prepared.gameIndex[i] : -1;
        for (const Row& r : frame)
        {
            if (r.bone >= sk.n || r.bone < 0) continue;
            float* d = snap.pose + size_t(r.bone) * 7;
            for (int k = 0; k < 7; ++k) d[k] = r.mm[k];
            if (r.masked) snap.upperBits[r.bone >> 5] |= 1u << (r.bone & 31);
        }
        const bool admitted = !solveOff && rt::Admit(snap, ownerMapping ? rt::HookOwner::Mapping : rt::HookOwner::FK);
        LARGE_INTEGER c0, c1;
        QueryPerformanceCounter(&c0);
        put(0);
        rt::Cycle cycle;
        const bool captured = admitted && rt::Capture(prepared, snap, rec.data(), count, true, cycle);
        put(1);
        native_reload_contacts::Result result;
        bool wrote = false;
        if (captured)
            wrote = rt::Finish(prepared, snap, rec.data(), count, true, cycle, result, snap.reloadContactsWeight,
                               rt::kAuthoredContactMinReach, &continuity, h.ticks, 10000000);
        else if (admitted)
            wrote = rt::HoldOnly(prepared, snap, rec.data(), count, true, continuity, h.ticks, 10000000);
        QueryPerformanceCounter(&c1);
        const double costUs = double(c1.QuadPart - c0.QuadPart) * 1e6 / double(qpf.QuadPart);

        // contact measures
        float lhRel = -1, rhRel = -1, lhRelBase = -1;
        {
            std::vector<uint8_t> nat(size_t(count) * 48, 0), base(size_t(count) * 48, 0);
            std::memcpy(nat.data(), kIdentity, 48); std::memcpy(base.data(), kIdentity, 48);
            if (count > 1) { std::memcpy(nat.data() + 48, kIdentity, 48); std::memcpy(base.data() + 48, kIdentity, 48); }
            for (const Row& r : frame)
                if (r.gameIndex >= 0)
                {
                    std::memcpy(nat.data() + size_t(r.gameIndex) * 48, r.s[0], 48);
                    std::memcpy(base.data() + size_t(r.gameIndex) * 48, r.s[1], 48);
                }
            ov::Pose pn, pb, po;
            if (rt::Read(prepared, nat.data(), pn) && rt::Read(prepared, base.data(), pb) && rt::Read(prepared, rec.data(), po))
            {
                const auto ln = HandInWeapon(prepared, pn, prepared.arms.left.hand), rn = HandInWeapon(prepared, pn, prepared.arms.right.hand);
                lhRel = gik::Len(HandInWeapon(prepared, po, prepared.arms.left.hand) - ln) * 100;
                rhRel = gik::Len(HandInWeapon(prepared, po, prepared.arms.right.hand) - rn) * 100;
                lhRelBase = gik::Len(HandInWeapon(prepared, pb, prepared.arms.left.hand) - ln) * 100;
            }
        }
        std::fprintf(frames, "%s,%.4f,%d,%d,%.4f,%.4f,%d,%d,%d,%d,%.2f,%.2f,%.2f,%.2f\n", windows[wi].label.c_str(),
                     double(h.ticks - windows[wi].a) / 1e7, h.action, h.overlay, env, snap.reloadContactsWeight, admitted ? 1 : 0,
                     captured ? 1 : 0, wrote ? 1 : 0, captured ? int(result.reject) : -1, lhRel, rhRel, lhRelBase, costUs);
        // the recording's rows with s2 = the replayed output
        char buf[64];
        for (const Row& r : frame)
        {
            // keep the first 15 + 24 fields (ids, s0, s1), replace s2, keep mm
            size_t pos = 0;
            int commas = 0;
            while (pos < r.text.size() && commas < 39) { if (r.text[pos] == ',') ++commas; ++pos; }
            size_t endS2 = pos;
            int c2 = 0;
            while (endS2 < r.text.size() && c2 < 12) { if (r.text[endS2] == ',') ++c2; ++endS2; }
            out.write(r.text.data(), std::streamsize(pos));
            for (int k = 0; k < 12; ++k)
            {
                float v = r.s[2][k];
                if (r.gameIndex >= 0) std::memcpy(&v, rec.data() + size_t(r.gameIndex) * 48 + size_t(k) * 4, 4);
                const int len = std::snprintf(buf, sizeof buf, "%.9g,", double(v));
                out.write(buf, len);
            }
            out.write(r.text.data() + endS2, std::streamsize(r.text.size() - endS2));
            out << "\n";
        }
        ++replayed; written += wrote ? 1 : 0;
        frame.clear();
    };
    while (std::getline(in, line))
    {
        if (line.empty() || line[0] == '#') continue;
        const long long ticks = std::atoll(line.c_str());
        if (ticks > windows.back().b + 10000000) break;
        // cheap window filter on the first field
        bool inWindow = false;
        for (size_t k = wi; k < windows.size() && windows[k].a <= ticks; ++k)
            if (ticks <= windows[k].b) { inWindow = true; break; }
        if (!inWindow) { if (!frame.empty()) { flush(); curTicks = -1; } continue; }
        Row r;
        if (!ParseRow(line, r)) continue;
        if (r.ticks != curTicks) { flush(); curTicks = r.ticks; }
        frame.push_back(std::move(r));
    }
    flush();
    std::fprintf(stderr, "frames %d written %d\n", replayed, written);
    std::fclose(frames);
    return 0;
}
