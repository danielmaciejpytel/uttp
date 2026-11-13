// Offline replay of mm/HeadLead.hpp on the film 12-43-58 (3-stage pose capture reduced to the root-to-Head chain by a scratch
// parser: records of {float t; int32 clip, action, overlay, aim; float weight, upper; float q[3 stages][9 bones][4]} (460 B)
// and the feed dump of the same session). Applies the correction to stage 1 (pose after Blend) exactly like the driver does
// (Step per feed row, Apply per pose frame) and writes one CSV row per frame for the scratch analysis.
// Usage: utpp_headlead_replay <chain.bin> <feed.txt> <out.csv> [leadDegAtRef maxLead [pathBlend [twistBiasDeg]]]
// pathBlend (default 1) 0 = the lead against Spine3 (first version), 1 = against the travel direction (feed yawDeg = actor yaw).
// twistBiasDeg (default Params::twistBiasDeg) centre of the twist window; 23.39 (= restBiasDeg) reproduces the previous window.
// The last CSV column tclamp is 1 when the twist window changed the demanded correction.
#define _CRT_SECURE_NO_WARNINGS 1
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "../src/mm/HeadLead.hpp"

using namespace utpp::mm::headlead;

namespace
{
struct FeedRow
{
    double t = 0, dt = 0;
    float vx = 0, vy = 0, yaw = 0;
    bool onGround = true;
    int loco = 1;
};
const char* kNames[9] = {"Root", "Hips", "Spine", "Spine1", "Spine2", "Spine3", "Neck", "Neck1", "Head"};
} // namespace

int main(int argc, char** argv)
{
    if (argc < 4)
        return 2;
    std::ifstream bin(argv[1], std::ios::binary);
    std::vector<char> data((std::istreambuf_iterator<char>(bin)), std::istreambuf_iterator<char>());
    const size_t kRec = 460;
    const size_t frames = data.size() / kRec;
    std::vector<FeedRow> feed;
    {
        std::ifstream f(argv[2]);
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
    Params p;
    if (argc >= 6)
    {
        p.leadDegAtRef = static_cast<float>(std::atof(argv[4]));
        p.maxLeadDeg = static_cast<float>(std::atof(argv[5]));
    }
    if (argc >= 7)
        p.pathBlend = static_cast<float>(std::atof(argv[6]));
    if (argc >= 8)
        p.twistBiasDeg = static_cast<float>(std::atof(argv[7]));
    const Skeleton sk = Resolve(9, [](uint32_t i) { return std::string_view(kNames[i]); },
                                [](uint32_t i) { return i == 0 ? -1 : static_cast<int>(i) - 1; });
    if (!sk.ok)
        return 3;
    HeadLead hl;
    std::FILE* out = std::fopen(argv[3], "w");
    if (!out)
        return 4;
    std::fprintf(out, "t,clip,lead,w,corr,measured,rate,applied,path,hvp");
    for (int b = 0; b < 9; ++b)
        std::fprintf(out, ",q%dx,q%dy,q%dz,q%dw", b, b, b, b);
    std::fprintf(out, ",tclamp\n");
    size_t fi = 0;
    double prevT = -1.0;
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
        float q[3][9][4];
        std::memcpy(q, rec + 28, sizeof q);
        // advance the feed rows up to this pose frame
        Signals s;
        float stepDt = 1.0f / 60.0f;
        while (fi + 1 < feed.size() && feed[fi].t < t)
        {
            const FeedRow& r = feed[fi];
            s.vx = r.vx; s.vy = r.vy;
            s.yawDeg = r.yaw; s.yawValid = true;
            s.aiming = aim != 0;
            s.weaponBusy = action == 2 || action == 8;
            s.air = !r.onGround;
            s.crouch = r.loco == 2;
            hl.Step(p, s, static_cast<float>(r.dt));
            ++fi;
        }
        if (prevT >= 0.0)
            stepDt = static_cast<float>(t - prevT);
        prevT = t;
        // stage 1 -> database-ordered pose (positions 0)
        float pose[9 * 7] = {};
        for (int b = 0; b < 9; ++b)
            for (int k = 0; k < 4; ++k)
                pose[b * 7 + 3 + k] = q[1][b][k];
        const float c = hl.Apply(p, sk, pose, 9, stepDt);
        const Telemetry& tl = hl.Tel();
        std::fprintf(out, "%.5f,%d,%.4f,%.4f,%.4f,%.4f,%.3f,%d,%.3f,%.3f", static_cast<double>(t), clip, static_cast<double>(tl.lead),
                     static_cast<double>(tl.weight), static_cast<double>(c), static_cast<double>(tl.measured),
                     static_cast<double>(tl.rate), tl.applied ? 1 : 0, static_cast<double>(tl.path), static_cast<double>(tl.headVsPath));
        for (int b = 0; b < 9; ++b)
            std::fprintf(out, ",%.9g,%.9g,%.9g,%.9g", static_cast<double>(pose[b * 7 + 3]), static_cast<double>(pose[b * 7 + 4]),
                         static_cast<double>(pose[b * 7 + 5]), static_cast<double>(pose[b * 7 + 6]));
        std::fprintf(out, ",%d\n", tl.twistClamped ? 1 : 0);
    }
    std::fclose(out);
    std::printf("frames=%zu feed=%zu\n", frames, feed.size());
    return 0;
}
