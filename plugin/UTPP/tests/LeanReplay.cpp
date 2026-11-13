// Offline replay of mm/LeanRuntime.hpp (motion lean layer) on a 3-stage pose capture reduced by a scratch parser to a compact binary of
// the stage-1 (blend) LOCAL poses of the 73 game-rig bones (records of {float t; int32 clip, action, overlay, aim; float weight, upper;
// float pose[73][7]} = 2072 B, p.xyz q.xyzw; t = feed time) plus the rig list ("index name parent" per line) and the feed dump of the
// same session (utpp-feed-*.txt). Mirrors the driver order: lean Step per feed row, Apply per pose frame on the v2 pose, then (optional)
// the orientation warp Measure + Apply on the leaned pose (the shipped order: lean BEFORE OrientWarp). Writes one CSV row per frame
// with the model-space positions of the bones the scratch analysis needs, BEFORE (captured pose) and AFTER (leaned [+ warped] pose).
// The Disable_AdditiveLeans curve is not replayed here (the capture carries a clip index but no clip time); the driver level replay does.
// Usage: utpp_lean_replay <pose.bin> <rig.txt> <feed.txt> <lean.bin> <overlay.utppov> <rig hash 0xWOMAN|0xMAN> <out.csv> [key=value ...]
//   keys: on=0|1 (0 = layer off: AFTER must equal BEFORE bit for bit), head=0|1, scale, headScale, tc (accel time constant),
//         ow=0|1 (orientation warp on top), shoulderInto (orientation warp target)
#define _CRT_SECURE_NO_WARNINGS 1
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "../src/mm/LeanRuntime.hpp"
#include "../src/mm/OrientWarp.hpp"

namespace ow = utpp::mm::orientwarp;
namespace hl = utpp::mm::headlead;
namespace ln = utpp::mm::lean;
namespace lr = utpp::mm::leanrt;
namespace ov = utpp::mm::ov;

namespace
{
struct FeedRow
{
    double t = 0, dt = 0;
    float vx = 0, vy = 0, yaw = 0;
    bool onGround = true;
    int loco = 1;
};
const char* kOut[] = {"Hips", "Spine1", "Spine3", "Neck", "Head", "LeftShoulder", "RightShoulder", "LeftUpLeg", "RightUpLeg",
                      "LeftEye", "RightEye", "LeftFoot", "RightFoot", "LeftToeBase", "RightToeBase", "Spine"};
constexpr int kNumOut = sizeof(kOut) / sizeof(kOut[0]);
constexpr int kNb = 73;

void Fk(const std::vector<int>& par, const float* pose, float* pos /*kNb*3*/, float* rot /*kNb*4*/)
{
    for (int b = 0; b < kNb; ++b)
    {
        const float* l = pose + b * 7;
        const int p = par[b];
        if (p < 0)
        {
            pos[b * 3] = l[0]; pos[b * 3 + 1] = l[1]; pos[b * 3 + 2] = l[2];
            std::memcpy(rot + b * 4, l + 3, 16);
            continue;
        }
        const hl::Quat q{rot[p * 4], rot[p * 4 + 1], rot[p * 4 + 2], rot[p * 4 + 3]};
        const float cx = q.y * l[2] - q.z * l[1], cy = q.z * l[0] - q.x * l[2], cz = q.x * l[1] - q.y * l[0];
        pos[b * 3] = pos[p * 3] + l[0] + 2.0f * (q.w * cx + q.y * cz - q.z * cy);
        pos[b * 3 + 1] = pos[p * 3 + 1] + l[1] + 2.0f * (q.w * cy + q.z * cx - q.x * cz);
        pos[b * 3 + 2] = pos[p * 3 + 2] + l[2] + 2.0f * (q.w * cz + q.x * cy - q.y * cx);
        const hl::Quat r = hl::Mul(q, hl::Quat{l[3], l[4], l[5], l[6]});
        rot[b * 4] = r.x; rot[b * 4 + 1] = r.y; rot[b * 4 + 2] = r.z; rot[b * 4 + 3] = r.w;
    }
}

void RootYaw(const std::vector<int>& par, int hips, float* pose, float deg)
{
    if (deg == 0.0f)
        return;
    const float h = 0.5f * deg * ow::kRad;
    const hl::Quat qz{0.0f, 0.0f, std::sin(h), std::cos(h)};
    const int root = par[hips];
    hl::Quat pr{};
    if (root >= 0)
        pr = hl::Quat{pose[root * 7 + 3], pose[root * 7 + 4], pose[root * 7 + 5], pose[root * 7 + 6]};
    const hl::Quat k = hl::Mul(hl::Mul(hl::Conj(pr), qz), pr);
    float* l = pose + hips * 7;
    const hl::Quat q = hl::Mul(k, hl::Quat{l[3], l[4], l[5], l[6]});
    l[3] = q.x; l[4] = q.y; l[5] = q.z; l[6] = q.w;
    const float v[3] = {l[0], l[1], l[2]};
    const float cx = k.y * v[2] - k.z * v[1], cy = k.z * v[0] - k.x * v[2], cz = k.x * v[1] - k.y * v[0];
    l[0] = v[0] + 2.0f * (k.w * cx + k.y * cz - k.z * cy);
    l[1] = v[1] + 2.0f * (k.w * cy + k.z * cx - k.x * cz);
    l[2] = v[2] + 2.0f * (k.w * cz + k.x * cy - k.y * cx);
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 8)
        return 2;
    std::vector<std::string> names;
    std::vector<int> par;
    {
        std::ifstream f(argv[2]);
        int i, p;
        std::string n;
        while (f >> i >> n >> p)
        {
            names.push_back(n);
            par.push_back(p);
        }
    }
    if (static_cast<int>(names.size()) != kNb)
        return 3;
    std::vector<FeedRow> feed;
    {
        std::ifstream f(argv[3]);
        std::string line;
        while (std::getline(f, line))
        {
            if (line.empty() || line[0] == '#')
                continue;
            std::istringstream ss(line);
            std::vector<std::string> z;
            std::string tok;
            while (ss >> tok)
                z.push_back(tok);
            if (z.size() < 28)
                continue;
            FeedRow r;
            r.t = std::atof(z[0].c_str());
            r.dt = std::atof(z[1].c_str());
            r.vx = static_cast<float>(std::atof(z[2].c_str()));
            r.vy = static_cast<float>(std::atof(z[3].c_str()));
            r.yaw = static_cast<float>(std::atof(z[7].c_str()));
            r.loco = std::atoi(z[9].c_str());
            r.onGround = z[27] != "0";
            feed.push_back(r);
        }
    }
    const uint64_t rigHash = std::strtoull(argv[6], nullptr, 0);
    ln::LeanPack pack;
    ov::OverlayFile ovf;
    std::string err;
    if (!pack.LoadFromFile(argv[4], rigHash, err))
    {
        std::printf("pack: %s\n", err.c_str());
        return 4;
    }
    if (!ovf.LoadFromFile(argv[5], rigHash, err) || !lr::PackMatchesOverlay(pack, ovf, err))
    {
        std::printf("overlay: %s\n", err.c_str());
        return 4;
    }
    // capture bone -> pack bone by name
    int toPack[kNb];
    for (int b = 0; b < kNb; ++b)
    {
        toPack[b] = -1;
        for (int k = 0; k < ln::kNumBones; ++k)
            if (pack.BoneName(k) == names[b])
                toPack[b] = k;
        if (toPack[b] < 0)
        {
            std::printf("bone %s not in the pack\n", names[b].c_str());
            return 4;
        }
    }
    bool on = true, head = false, useOw = false;
    lr::Params lp;
    ow::Params op;
    for (int a = 8; a < argc; ++a)
    {
        char key[64];
        float v = 0.0f;
        if (std::sscanf(argv[a], "%63[^=]=%f", key, &v) != 2)
            continue;
        const std::string k = key;
        if (k == "on") on = v != 0.0f;
        else if (k == "head") head = v != 0.0f;
        else if (k == "scale") lp.scale = v;
        else if (k == "headScale") lp.headScale = v;
        else if (k == "tc") lp.accelTc = v;
        else if (k == "ow") useOw = v != 0.0f;
        else if (k == "shoulderInto") op.shoulderIntoDeg = v;
    }
    const auto nameFn = [&](uint32_t i) { return std::string_view(names[i]); };
    const auto parFn = [&](uint32_t i) { return par[i]; };
    const ow::Skeleton sk = ow::Resolve(kNb, nameFn, parFn);
    if (!sk.ok)
        return 4;
    int outIdx[kNumOut];
    for (int k = 0; k < kNumOut; ++k)
    {
        outIdx[k] = -1;
        for (int b = 0; b < kNb; ++b)
            if (names[b] == kOut[k])
                outIdx[k] = b;
        if (outIdx[k] < 0)
            return 5;
    }
    std::ifstream bin(argv[1], std::ios::binary);
    std::vector<char> data((std::istreambuf_iterator<char>(bin)), std::istreambuf_iterator<char>());
    const size_t kRec = 28 + kNb * 7 * 4;
    const size_t frames = data.size() / kRec;
    lr::LeanRuntime lean;
    lean.Bind(pack);
    ow::OrientWarp warp;
    ov::Pose base;
    ovf.FramePose(ovf.PoseAt(0).firstFrame, base);
    std::FILE* out = std::fopen(argv[7], "w");
    if (!out)
        return 6;
    std::fprintf(out, "t,clip,w,lat,aLat,x,walkRun,leanW,headW,dp,tau,identical");
    for (const char* tag : {"b", "a"})
        for (int k = 0; k < kNumOut; ++k)
            std::fprintf(out, ",%s_%s_x,%s_%s_y,%s_%s_z", tag, kOut[k], tag, kOut[k], tag, kOut[k]);
    std::fprintf(out, "\n");
    size_t fi = 0;
    double prevT = -1.0;
    size_t identical = 0;
    for (size_t i = 0; i < frames; ++i)
    {
        const char* rec = data.data() + i * kRec;
        float t;
        int32_t clip, action, overlay, aim;
        float weight, upper;
        std::memcpy(&t, rec, 4);
        std::memcpy(&clip, rec + 4, 4);
        std::memcpy(&action, rec + 8, 4);
        std::memcpy(&overlay, rec + 12, 4);
        std::memcpy(&aim, rec + 16, 4);
        std::memcpy(&weight, rec + 20, 4);
        std::memcpy(&upper, rec + 24, 4);
        float pose[kNb * 7], orig[kNb * 7];
        std::memcpy(pose, rec + 28, sizeof pose);
        std::memcpy(orig, pose, sizeof pose);
        float stepDt = 1.0f / 60.0f;
        if (prevT >= 0.0)
            stepDt = static_cast<float>(t - prevT);
        prevT = t;
        while (fi + 1 < feed.size() && feed[fi].t < t)
        {
            const FeedRow& r = feed[fi];
            lr::Signals s;
            s.vx = r.vx;
            s.vy = r.vy;
            s.velValid = true;
            s.aiming = aim != 0;
            s.weaponBusy = action == 2 || action == 8;
            s.air = !r.onGround;
            s.crouch = r.loco == 2;
            s.moving = std::hypot(r.vx, r.vy) > 0.3f && r.onGround;
            s.blocked = !on;
            s.head = head;
            lean.Step(pack, lp, s, static_cast<float>(r.dt));
            ow::Signals w;
            w.vx = r.vx; w.vy = r.vy; w.yawDeg = r.yaw; w.yawValid = true;
            w.aiming = s.aiming; w.weaponBusy = s.weaponBusy; w.air = s.air; w.crouch = s.crouch; w.blocked = !useOw;
            warp.Step(op, w, static_cast<float>(r.dt));
            ++fi;
        }
        if (on && !lean.Idle())
        {
            ov::Pose p = base;
            for (int b = 0; b < kNb; ++b)
            {
                ov::Xf& x = p.b[toPack[b]];
                const float* l = pose + b * 7;
                x.t = ov::Vec3{l[0], l[1], l[2]};
                x.r = ov::Quat{l[3], l[4], l[5], l[6]};
            }
            if (lean.Apply(pack, lp, p))
                for (int b = 0; b < kNb; ++b)
                {
                    const ov::Xf& x = p.b[toPack[b]];
                    float* l = pose + b * 7;
                    l[0] = x.t.x; l[1] = x.t.y; l[2] = x.t.z;
                    l[3] = x.r.x; l[4] = x.r.y; l[5] = x.r.z; l[6] = x.r.w;
                }
        }
        float dp = 0.0f;
        if (useOw)
        {
            dp = warp.Measure(op, sk, pose, kNb, 0.0f, stepDt);
            warp.Apply(op, sk, pose, kNb);
            RootYaw(par, sk.hips, pose, dp);
        }
        const bool same = std::memcmp(pose, orig, sizeof pose) == 0;
        identical += same ? 1 : 0;
        const lr::Telemetry& tl = lean.Tel();
        std::fprintf(out, "%.5f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d", static_cast<double>(t), clip, static_cast<double>(weight),
                     static_cast<double>(tl.lat), static_cast<double>(tl.aLat), static_cast<double>(tl.x), static_cast<double>(tl.walkRun),
                     static_cast<double>(lean.Weight()), static_cast<double>(lean.HeadWeight()), static_cast<double>(dp),
                     static_cast<double>(warp.TorsoDeg()), same ? 1 : 0);
        for (const float* ps : {orig, pose})
        {
            float pos[kNb * 3], rot[kNb * 4];
            Fk(par, ps, pos, rot);
            for (int k = 0; k < kNumOut; ++k)
                std::fprintf(out, ",%.6f,%.6f,%.6f", static_cast<double>(pos[outIdx[k] * 3]), static_cast<double>(pos[outIdx[k] * 3 + 1]),
                             static_cast<double>(pos[outIdx[k] * 3 + 2]));
        }
        std::fprintf(out, "\n");
    }
    std::fclose(out);
    std::printf("frames=%zu feed=%zu identical=%zu\n", frames, feed.size(), identical);
    return 0;
}
