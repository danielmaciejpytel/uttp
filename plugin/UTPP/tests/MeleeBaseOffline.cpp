// Offline replay of recorded fists punches through the PRODUCTION glue of the melee chest base correction (tools/melee/fist_base_offline.py,
// tools/melee/fist_base_fixture.py), not shipped, not a test. See tests/MeleeBaseReplay.hpp for the file format.
// Usage: utpp_melee_base_offline <in.bin> <out.bin>
// out.bin: "MBOO" int32 nb | int32 nframes | nframes x { float Q[nb*4] after the layer (FILE bone order), StepOut2 }
#define _CRT_SECURE_NO_WARNINGS
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <string>
#include <vector>

#include "MeleeBaseReplay.hpp"

using namespace utpp;
using namespace utpp::mm;

struct StepOut2
{
    int32_t matched, body, clip, frame, neighbour;
    float t, rmsDeg, gate, weight;
    float applied[3], raw[3], lim[3]; // applied: rotation vector (rad); raw / lim: yaw pitch roll (deg)
    uint32_t flags;
    float keep;
};

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::fprintf(stderr, "usage: utpp_melee_base_offline <in.bin> <out.bin>\n");
        return 2;
    }
    replay::Replay rp;
    if (!rp.Load(argv[1]))
    {
        std::fprintf(stderr, "cannot read %s\n", argv[1]);
        return 2;
    }
    rp.Arm();
    FILE* out = std::fopen(argv[2], "wb");
    if (!out)
        return 2;
    const int32_t nb = rp.nb, nf = int32_t(rp.frames.size());
    std::fwrite("MBOO", 1, 4, out);
    std::fwrite(&nb, 4, 1, out);
    std::fwrite(&nf, 4, 1, out);
    std::vector<double> micros;
    for (size_t i = 0; i < rp.frames.size(); ++i)
    {
        const auto t0 = std::chrono::steady_clock::now();
        rp.Step(i);
        micros.push_back(std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count());
        std::vector<float> Qo(size_t(nb) * 4);
        for (int k = 0; k < nb; ++k)
        {
            const float* r = reinterpret_cast<const float*>(rp.records.data() + size_t(k) * integ::kGameBoneStride);
            const int32_t o = rp.origIndex[size_t(k)];
            Qo[size_t(o) * 4] = r[4], Qo[size_t(o) * 4 + 1] = r[5], Qo[size_t(o) * 4 + 2] = r[6], Qo[size_t(o) * 4 + 3] = r[7];
        }
        const utpp::meleebase::LastStep ls = utpp::meleebase::GetLastStep();
        StepOut2 so{ls.matched ? 1 : 0, ls.body, ls.clip, ls.frame, ls.neighbour, ls.t, ls.rmsDeg, ls.gate, ls.weight,
                    {ls.applied[0], ls.applied[1], ls.applied[2]}, {ls.raw[0], ls.raw[1], ls.raw[2]}, {ls.lim[0], ls.lim[1], ls.lim[2]}, ls.flags,
                    rp.lastKeep};
        std::fwrite(Qo.data(), 4, Qo.size(), out);
        std::fwrite(&so, sizeof(so), 1, out);
    }
    std::fclose(out);
    if (!micros.empty())
    {
        std::sort(micros.begin(), micros.end());
        std::fprintf(stderr, "Step (guard + blend + Apply) us: n=%zu p50=%.1f p95=%.1f p99=%.1f max=%.1f\n", micros.size(), micros[micros.size() / 2],
                     micros[micros.size() * 95 / 100], micros[micros.size() * 99 / 100], micros.back());
    }
    std::string names;
    for (int body = 0; body < 2; ++body)
    {
        const utpp::mm::meleebase::TableData& t = body == 0 ? utpp::mm::meleebase::FemaleTable() : utpp::mm::meleebase::MaleTable();
        for (uint32_t c = 0; c < t.clipCount; ++c)
        {
            uint32_t first = 0, count = 0;
            for (uint32_t i = 0; i < t.frameCount; ++i)
                if (t.clipOfFrame[i] == c)
                {
                    if (count == 0)
                        first = i;
                    ++count;
                }
            names += std::string(body == 0 ? "F:" : "M:") + t.clipNames[c] + ":" + std::to_string(first) + ":" + std::to_string(count) + "\n";
        }
    }
    const std::string np = std::string(argv[2]) + ".clips.txt";
    if (FILE* f = std::fopen(np.c_str(), "wb"))
    {
        std::fwrite(names.data(), 1, names.size(), f);
        std::fclose(f);
    }
    return 0;
}
