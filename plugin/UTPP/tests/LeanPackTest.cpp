// Offline test of the lean pose pack reader / evaluator (src/mm/LeanPack.hpp, pack "utpp.lean/1" from
// Source\tools\mmbake\leanbake.py; motion lean + head-turn blend spaces).
//
//   utpp_mm_lean_pack_test <utpp_lean_woman.bin> <utpp_lean_woman.json> [utpp_lean_man.bin]
//
// Data driven on the produced pack (motion derived, outside git; a missing file makes the test FAIL):
//   * round trip: pack vs the JSON header (layers, axes, sample coordinates + clip names, file size, payload crc,
//     rig hash), parameters vs the quoted ABP constants, section / layer / sample / curve counts;
//   * fail closed: ~30 mutations (magic, version, sizes, CRCs, rig hash, section table, with CRCs re-fixed: triangle
//     topology, delta quaternion, curve range, layer ranges, non-finite floats ...) must all be rejected;
//   * blend space: every sample coordinate reproduces exactly that sample (weight 1) and the stored delta; weights
//     sum to 1 on 60000 random inputs per layer; inside the hull the weights reconstruct the input point; outside
//     (the 5-sample cross has a diamond hull) the result is the closest perimeter point; hand computed golden
//     vectors; an independent brute-force triangle search agrees with the walking search from every start triangle;
//     continuity along a path; the 1D layer equals BlendSpace.hpp's BlendSpaceEval;
//   * BlendAdditive == ov::BlendAdditives; LayerState smoothing == bs::WeightSmoother on the same targets;
//   * Disable_AdditiveLeans curves (16 Relaxed clips + 8 proposed Neutral equivalents), ABP gate helpers.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <random>
#include <string>
#include <vector>

#include "../src/mm/LeanPack.hpp"
#include "../src/mm/BlendSpace.hpp"
#include "../src/mm/MiniJson.hpp"

using namespace utpp::mm;
using namespace utpp::mm::lean;
namespace json = utpp::json;
using Tri = LeanPack::Tri;

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
static bool Near(float a, float b, float tol) { return std::fabs(a - b) <= tol; }

static std::vector<uint8_t> ReadAll(const std::string& aPath)
{
    std::ifstream f(aPath, std::ios::binary);
    return std::vector<uint8_t>((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}

static void FixCrc(std::vector<uint8_t>& b)
{
    const uint32_t pc = Crc32(b.data() + 64, b.size() - 64);
    std::memcpy(b.data() + 36, &pc, 4);
    uint8_t h[64];
    std::memcpy(h, b.data(), 64);
    std::memset(h + 40, 0, 4);
    const uint32_t hc = Crc32(h, 64);
    std::memcpy(b.data() + 40, &hc, 4);
}

static uint32_t U32(const std::vector<uint8_t>& b, size_t o)
{
    uint32_t v;
    std::memcpy(&v, b.data() + o, 4);
    return v;
}
static size_t SecOff(const std::vector<uint8_t>& b, const char* tag)
{
    const uint32_t n = U32(b, 28);
    for (uint32_t i = 0; i < n; ++i)
        if (std::memcmp(b.data() + 64 + 16 * i, tag, 4) == 0)
            return U32(b, 64 + 16 * i + 4);
    return 0;
}
static void PutF(std::vector<uint8_t>& b, size_t o, float v) { std::memcpy(b.data() + o, &v, 4); }
static void PutI(std::vector<uint8_t>& b, size_t o, int32_t v) { std::memcpy(b.data() + o, &v, 4); }

static float QuatDelta(const Quat& a, const Quat& b) // 1 - |dot| (sign independent)
{
    return 1.0f - std::fabs(math::QuatDot(a, b));
}

static void TestRoundTrip(const LeanPack& P, const std::string& aJsonPath, const std::vector<uint8_t>& aBytes, uint64_t aRig)
{
    Check(P.Ok(), "pack loads");
    Check(P.RigHash() == aRig, "rig hash");
    Check(P.Skel().n == kNumBones && P.NumLayers() == 4, "73 bones, 4 layers");
    const char* ids[4] = {"run", "walk", "head", "cmc_run"};
    const int ns[4] = {5, 5, 13, 3}, nt[4] = {4, 4, 12, 2}, dims[4] = {2, 2, 2, 1};
    const float interp[4] = {1.2f, 1.2f, 1.75f, 1.5f};
    for (int i = 0; i < 4; ++i)
    {
        const int li = P.FindLayer(ids[i]);
        Check(li == i, std::string("layer order ") + ids[i]);
        if (li < 0)
            continue;
        const Layer& L = P.LayerAt(li);
        Check(L.numSamples == ns[i] && L.numTris == nt[i] && L.dims == dims[i], std::string("layer shape ") + ids[i]);
        Check(L.interpPerS == interp[i], std::string("layer interp ") + ids[i]);
        Check(L.min[0] == -1.0f && L.max[0] == 1.0f && (dims[i] == 1 || (L.min[1] == -1.0f && L.max[1] == 1.0f)),
              std::string("axis range ") + ids[i]);
    }
    Check(std::string(P.LayerAt(0).blendspace) == "BS_Relaxed_Run_Leans" && std::string(P.LayerAt(0).baseClip) == "M_Relaxed_Run_Pose_Lean_Base",
          "run blend space / reference pose names");
    Check(std::string(P.LayerAt(2).blendspace) == "BS_Relaxed_Lean_Head" && std::string(P.LayerAt(2).baseClip) == "M_Relaxed_Stand_Pose_Head_Turn_Base",
          "head blend space / reference pose names");
    Check(P.LayerAt(0).role == Role::Body && P.LayerAt(2).role == Role::Head && P.LayerAt(3).role == Role::Cmc &&
              P.LayerAt(3).style == Style::Neutral && P.LayerAt(0).style == Style::Relaxed,
          "roles / styles");
    const Params& p = P.P();
    Check(p.latSpeedLo == 200 && p.latSpeedHi == 320 && p.latAccelLo == 500 && p.latAccelHi == 800 && p.latClampLo == -1 && p.latClampHi == 1,
          "lat accel constants");
    Check(p.xfadeSpeedLo == 200 && p.xfadeSpeedHi == 320 && p.xfadeInterpPerS == 10, "walk/run crossfade constants");
    Check(Near(p.boolBlendS, 0.3f, 1e-6f) && Near(p.stateBlendS, 0.3f, 1e-6f) && p.disableCurveInterpPerS == 5, "blend / disable constants");
    Check(p.cmcSpeedLo == 165 && p.cmcSpeedHi == 375 && p.cmcScaleLo == 0.5f && p.cmcScaleHi == 1.0f &&
              Near(p.cmcAlphaBlendS, 0.2f, 1e-6f) && p.cmcWeightInterpPerS == 1.5f,
          "cmc constants");
    Check(p.headWeightInterpPerS == 1.75f && p.bodyWeightInterpPerS == 1.2f, "weight interp constants");
    const float dm[6][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, 1}, {0, -1}, {0, -1}}; // F B LR LL RL RR
    bool dmOk = true;
    for (int d = 0; d < 6; ++d)
        dmOk = dmOk && p.dirScale[d][0] == dm[d][0] && p.dirScale[d][1] == dm[d][1];
    Check(dmOk, "direction map (F B LR LL RL RR)");
    Check(P.NumClips() == 24, "24 curve clips");
    int kinds[2] = {0, 0};
    for (int i = 0; i < P.NumClips(); ++i)
        ++kinds[P.ClipAt(i).kind];
    Check(kinds[0] == 16 && kinds[1] == 8, "16 Relaxed source + 8 Neutral proposed curves");

    // JSON header cross-check
    const std::vector<uint8_t> jb = ReadAll(aJsonPath);
    Check(!jb.empty(), "json header readable: " + aJsonPath);
    json::Value root;
    std::string err;
    if (!jb.empty() && json::Parse(std::string_view(reinterpret_cast<const char*>(jb.data()), jb.size()), root, err))
    {
        Check(root.Find("schema") && root.Find("schema")->str == "utpp.lean/1", "json schema");
        const json::Value* bin = root.Find("binary");
        Check(bin && bin->Find("size_bytes") && static_cast<size_t>(bin->Find("size_bytes")->num) == aBytes.size(), "json size_bytes == file size");
        char crc[16];
        std::snprintf(crc, sizeof(crc), "0x%08x", P.PayloadCrc());
        Check(bin && bin->Find("payload_crc32") && bin->Find("payload_crc32")->str == crc, "json payload crc == pack payload crc");
        char rh[32];
        std::snprintf(rh, sizeof(rh), "0x%016llx", static_cast<unsigned long long>(P.RigHash()));
        Check(root.Find("rig_hash") && root.Find("rig_hash")->str == rh, "json rig_hash");
        const json::Value* layers = root.Find("layers");
        Check(layers && layers->arr.size() == 4, "json 4 layers");
        for (size_t i = 0; layers && i < layers->arr.size() && i < 4; ++i)
        {
            const json::Value& jl = layers->arr[i];
            const Layer& L = P.LayerAt(static_cast<int>(i));
            Check(jl.Find("id")->str == L.id && jl.Find("blendspace")->str == L.blendspace && jl.Find("base_clip")->str == L.baseClip,
                  std::string("json layer names ") + L.id);
            Check(static_cast<float>(jl.Find("weight_interp_per_s")->num) == L.interpPerS, std::string("json interp ") + L.id);
            const json::Value* js = jl.Find("samples");
            Check(js && static_cast<int>(js->arr.size()) == L.numSamples, std::string("json sample count ") + L.id);
            for (int k = 0; js && k < L.numSamples && k < static_cast<int>(js->arr.size()); ++k)
            {
                const SampleRec& s = P.Sample(static_cast<int>(i), k);
                const json::Value& e = js->arr[static_cast<size_t>(k)];
                if (!(e.Find("clip")->str == s.clip && e.Find("x")->num == s.x && e.Find("y")->num == s.y && e.Find("frame")->num == s.frame))
                    Check(false, std::string("json sample ") + L.id + " #" + std::to_string(k));
            }
            const json::Value* jt = jl.Find("triangles");
            Check(jt && static_cast<int>(jt->arr.size()) == L.numTris, std::string("json triangle count ") + L.id);
            for (int k = 0; jt && k < L.numTris && k < static_cast<int>(jt->arr.size()); ++k)
            {
                const Tri& t = P.TriAt(static_cast<int>(i), k);
                const json::Value& e = jt->arr[static_cast<size_t>(k)];
                if (!(e.arr.size() == 3 && e.arr[0].num == t.s[0] && e.arr[1].num == t.s[1] && e.arr[2].num == t.s[2]))
                    Check(false, std::string("json triangle ") + L.id + " #" + std::to_string(k));
            }
        }
    }
    else
        Check(false, "json parses: " + err);
}

static void TestFailClosed(const std::vector<uint8_t>& aGood, uint64_t aRig)
{
    auto rejects = [&](const char* what, std::vector<uint8_t> b, bool fix, uint64_t rig) {
        if (fix)
            FixCrc(b);
        LeanPack q;
        std::string err;
        const bool ok = q.LoadFromMemory(std::move(b), rig, err);
        Check(!ok && !q.Ok(), std::string("rejected: ") + what);
        if (ok)
            std::printf("  (accepted: %s)\n", what);
    };
    {
        LeanPack q;
        std::string err;
        Check(q.LoadFromMemory(aGood, aRig, err) && q.Ok(), "unmodified copy loads: " + err);
        LeanPack r;
        Check(r.LoadFromMemory(aGood, 0, err), "rig hash 0 skips the identity check");
    }
    auto m = [&](auto fn) {
        std::vector<uint8_t> b = aGood;
        fn(b);
        return b;
    };
    rejects("empty", {}, false, aRig);
    rejects("truncated", m([](auto& b) { b.resize(b.size() - 16); }), false, aRig);
    rejects("extended", m([](auto& b) { b.push_back(0); }), false, aRig);
    rejects("magic", m([](auto& b) { b[0] = 'X'; }), true, aRig);
    rejects("version 2", m([](auto& b) { b[8] = 2; }), true, aRig);
    rejects("header size", m([](auto& b) { b[12] = 80; }), true, aRig);
    rejects("rig hash differs", aGood, false, aRig ^ 1ull);
    rejects("file rig hash flipped", m([](auto& b) { b[16] ^= 1; }), true, aRig);
    rejects("num_bones 74", m([](auto& b) { b[24] = 74; }), true, aRig);
    rejects("payload byte flipped (stale crc)", m([](auto& b) { b[b.size() - 40] ^= 0x10; }), false, aRig);
    rejects("header byte flipped (stale crc, rig check off so only the header crc can catch it)", m([](auto& b) { b[20] ^= 0x01; }), false, 0);
    rejects("header reserved byte", m([](auto& b) { b[50] = 1; }), true, aRig);
    rejects("section count 7", m([](auto& b) { b[28] = 7; }), true, aRig);
    rejects("section tag unknown", m([](auto& b) { b[64] = 'Z'; }), true, aRig);
    rejects("section size != count*rec", m([](auto& b) { b[64 + 16 * 3 + 12] += 1; }), true, aRig); // SAMP count + 1
    rejects("section offset past end", m([](auto& b) { b[64 + 16 * 2 + 6] = 0x7F; }), true, aRig);
    rejects("parent >= index", m([](auto& b) { PutI(b, SecOff(b, "BONE") + 36 * 5 + 32, 9); }), true, aRig);
    rejects("PARM NaN", m([](auto& b) { PutF(b, SecOff(b, "PARM") + 4 * 3, std::nanf("")); }), true, aRig);
    rejects("PARM speed range inverted", m([](auto& b) { PutF(b, SecOff(b, "PARM") + 4 * 1, 100.0f); }), true, aRig);
    rejects("layer dims 3", m([](auto& b) { PutI(b, SecOff(b, "LAYR") + 160, 3); }), true, aRig);
    rejects("layer sample range gap", m([](auto& b) { PutI(b, SecOff(b, "LAYR") + 240 + 176, 6); }), true, aRig);
    rejects("layer too many samples", m([](auto& b) { PutI(b, SecOff(b, "LAYR") + 480 + 180, 17); }), true, aRig);
    rejects("layer axis inverted", m([](auto& b) { PutF(b, SecOff(b, "LAYR") + 192, 5.0f); }), true, aRig);
    rejects("layer interp 0", m([](auto& b) { PutF(b, SecOff(b, "LAYR") + 208, 0.0f); }), true, aRig);
    rejects("layer id empty", m([](auto& b) { b[SecOff(b, "LAYR")] = 0; }), true, aRig);
    rejects("sample outside axis", m([](auto& b) { PutF(b, SecOff(b, "SAMP") + 80 * 1, 3.0f); }), true, aRig);
    rejects("duplicate sample position", m([](auto& b) { const size_t o = SecOff(b, "SAMP"); PutF(b, o + 80, 0.0f); PutF(b, o + 84, 0.0f); }), true, aRig);
    rejects("triangle sample index out of range", m([](auto& b) { PutI(b, SecOff(b, "TRIS"), 99); }), true, aRig);
    rejects("triangle degenerate (repeated sample)", m([](auto& b) { PutI(b, SecOff(b, "TRIS") + 4, 2); }), true, aRig);
    rejects("triangle clockwise", m([](auto& b) { const size_t o = SecOff(b, "TRIS"); PutI(b, o + 4, 3); PutI(b, o + 8, 0); }), true, aRig);
    rejects("neighbour not reciprocal", m([](auto& b) { PutI(b, SecOff(b, "TRIS") + 12, 3); }), true, aRig);
    rejects("neighbour = self", m([](auto& b) { PutI(b, SecOff(b, "TRIS") + 12, 0); }), true, aRig);
    rejects("perimeter link on interior edge", m([](auto& b) {
                const size_t o = SecOff(b, "TRIS");
                for (int t = 0; t < 4; ++t)
                    for (int e = 0; e < 3; ++e)
                        if (static_cast<int32_t>(U32(b, o + 72 * t + 12 + 20 * e)) >= 0)
                        {
                            PutI(b, o + 72 * t + 16 + 20 * e, 1);
                            PutI(b, o + 72 * t + 24 + 20 * e, 0);
                            return;
                        }
            }),
            true, aRig);
    rejects("delta quaternion not unit", m([](auto& b) { PutF(b, SecOff(b, "DELT") + 12, 2.0f); }), true, aRig);
    rejects("delta NaN", m([](auto& b) { PutF(b, SecOff(b, "DELT") + 4 * 20 + 16, std::nanf("")); }), true, aRig);
    rejects("curve value > 1", m([](auto& b) { PutF(b, SecOff(b, "CVAL") + 8, 2.0f); }), true, aRig);
    rejects("curve value < 0", m([](auto& b) { PutF(b, SecOff(b, "CVAL") + 8, -0.5f); }), true, aRig);
    rejects("clip range past CVAL", m([](auto& b) { PutI(b, SecOff(b, "CLIP") + 64, 99999); }), true, aRig);
    rejects("clip kind 2", m([](auto& b) { PutI(b, SecOff(b, "CLIP") + 72, 2); }), true, aRig);
    rejects("clip fps 0", m([](auto& b) { PutF(b, SecOff(b, "CLIP") + 76, 0.0f); }), true, aRig);
}

static bool Contains(const Tri& t, float px, float py)
{
    const float ax = t.vx[0], ay = t.vy[0], bx = t.vx[1], by = t.vy[1], cx = t.vx[2], cy = t.vy[2];
    const float den = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy);
    const float a = ((by - cy) * (px - cx) + (cx - bx) * (py - cy)) / den;
    const float b = ((cy - ay) * (px - cx) + (ax - cx) * (py - cy)) / den;
    const float c = 1.0f - a - b;
    return a >= -1e-6f && b >= -1e-6f && c >= -1e-6f;
}

static void TestBlendSpaces(const LeanPack& P)
{
    std::mt19937 rng(12345);
    std::uniform_real_distribution<float> U(-1.2f, 1.2f);
    for (int li = 0; li < P.NumLayers(); ++li)
    {
        const Layer& L = P.LayerAt(li);
        const std::string tag = std::string(L.id) + ": ";
        // corner samples: exact reproduction of the sample and of its stored delta
        for (int k = 0; k < L.numSamples; ++k)
        {
            const SampleRec& s = P.Sample(li, k);
            for (int start = -1; start < L.numTris; ++start)
            {
                int cache = start;
                Weights w;
                const bool ok = P.Evaluate(li, s.x, s.y, cache, w);
                Check(ok && w.n == 1 && w.sample[0] == k && Near(w.w[0], 1.0f, 1e-5f),
                      tag + "sample " + std::to_string(k) + " weight 1 (start " + std::to_string(start) + ")");
            }
            float oneHot[kMaxSamples]{};
            oneHot[k] = 1.0f;
            Additive a;
            P.BlendAdditive(li, oneHot, a);
            const float* d = P.SampleDelta(li, k);
            bool same = true;
            for (int i = 0; i < kNumBones && same; ++i)
            {
                const float* q = d + i * kDeltaFloats;
                same = QuatDelta(a.b[i].r, Quat{q[0], q[1], q[2], q[3]}) < 1e-6f && Near(a.b[i].t.x, q[4], 1e-6f) && Near(a.b[i].t.y, q[5], 1e-6f) &&
                       Near(a.b[i].t.z, q[6], 1e-6f) && Near(a.b[i].s.x, q[7], 1e-6f);
            }
            Check(same, tag + "one-hot blend reproduces the stored delta of sample " + std::to_string(k));
        }
        // random inputs: sum 1, finite
        int cache = -1;
        int badSum = 0, fails = 0;
        double maxRecon = 0.0;
        for (int it = 0; it < 60000; ++it)
        {
            const float x = U(rng), y = L.dims == 2 ? U(rng) : 0.0f;
            Weights w;
            if (!P.Evaluate(li, x, y, cache, w))
            {
                ++fails;
                continue;
            }
            float sum = 0;
            for (int i = 0; i < w.n; ++i)
                sum += w.w[i];
            if (!Near(sum, 1.0f, 1e-5f))
                ++badSum;
            if (L.dims == 2 && std::string(L.id) == "head")
            {
                // hull = full square: weights reconstruct the clamped input
                float rx = 0, ry = 0;
                for (int i = 0; i < w.n; ++i)
                {
                    rx += w.w[i] * P.Sample(li, w.sample[i]).x;
                    ry += w.w[i] * P.Sample(li, w.sample[i]).y;
                }
                const float cx = std::clamp(x, -1.0f, 1.0f), cy = std::clamp(y, -1.0f, 1.0f);
                maxRecon = std::max(maxRecon, static_cast<double>(std::max(std::fabs(rx - cx), std::fabs(ry - cy))));
            }
        }
        {
            // out-of-range inputs equal the clamped input; the cache stays a valid index
            int bad = 0, badCache = 0;
            for (int it = 0; it < 3000; ++it)
            {
                const float x = std::uniform_real_distribution<float>(-5, 5)(rng), y = L.dims == 2 ? std::uniform_real_distribution<float>(-5, 5)(rng) : 0.0f;
                int c1 = -1, c2 = -1;
                Weights a, b;
                P.Evaluate(li, x, y, c1, a);
                P.Evaluate(li, std::clamp(x, -1.0f, 1.0f), std::clamp(y, -1.0f, 1.0f), c2, b);
                float ga[kMaxSamples]{}, gb[kMaxSamples]{};
                for (int k = 0; k < a.n; ++k)
                    ga[a.sample[k]] += a.w[k];
                for (int k = 0; k < b.n; ++k)
                    gb[b.sample[k]] += b.w[k];
                for (int k = 0; k < kMaxSamples; ++k)
                    if (!Near(ga[k], gb[k], 1e-6f))
                    {
                        ++bad;
                        break;
                    }
                if (c1 < 0 || c1 >= L.numTris)
                    ++badCache;
                else
                    for (int k = 0; k < a.n; ++k) // the cached triangle carries every sample of the result
                    {
                        const Tri& ct = P.TriAt(li, c1);
                        if (a.sample[k] != ct.s[0] && a.sample[k] != ct.s[1] && a.sample[k] != ct.s[2])
                            ++badCache;
                    }
            }
            Check(bad == 0, tag + "input beyond the axis range == clamped input (" + std::to_string(bad) + " mismatches)");
            Check(badCache == 0, tag + "cached triangle / segment is a valid index and holds every result sample after every evaluation");
        }
        Check(fails == 0, tag + "evaluation never fails (60000 random inputs, incl. out of range)");
        Check(badSum == 0, tag + "weights sum to 1 (60000 random inputs)");
        if (std::string(L.id) == "head")
            Check(maxRecon < 5e-4, tag + "weights reconstruct the input inside the (square) hull, max err " + std::to_string(maxRecon));
        if (L.dims != 2)
            continue;
        // independent brute-force containing triangle (inside the sample hull) vs the walking search from every start
        int mism = 0, tested = 0;
        for (int it = 0; it < 4000; ++it)
        {
            const float x = std::uniform_real_distribution<float>(-1, 1)(rng), y = std::uniform_real_distribution<float>(-1, 1)(rng);
            const float nx = (x + 1) * 0.5f, ny = (y + 1) * 0.5f;
            int found = -1;
            for (int t = 0; t < L.numTris && found < 0; ++t)
                if (Contains(P.TriAt(li, t), nx, ny))
                    found = t;
            if (found < 0)
                continue; // outside the hull
            const Tri& T = P.TriAt(li, found);
            const float ax = T.vx[0], ay = T.vy[0], bx = T.vx[1], by = T.vy[1], cx = T.vx[2], cy = T.vy[2];
            const float den = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy);
            const float a = ((by - cy) * (nx - cx) + (cx - bx) * (ny - cy)) / den;
            const float b = ((cy - ay) * (nx - cx) + (ax - cx) * (ny - cy)) / den;
            const float ref[3] = {a, b, 1 - a - b};
            if (std::min(ref[0], std::min(ref[1], ref[2])) < 1e-3f)
                continue; // within 1e-3 of a triangle edge UE's kinda-small-number slack may pick the neighbour
            ++tested;
            for (int start = -1; start < L.numTris; ++start)
            {
                int c2 = start;
                Weights w;
                P.Evaluate(li, x, y, c2, w);
                float got[kMaxSamples]{};
                for (int i = 0; i < w.n; ++i)
                    got[w.sample[i]] += w.w[i];
                float exp[kMaxSamples]{};
                for (int i = 0; i < 3; ++i)
                    if (ref[i] > kZeroWeight)
                        exp[T.s[i]] += ref[i];
                float tot = 0;
                for (float v : exp)
                    tot += v;
                for (float& v : exp)
                    v /= tot;
                for (int i = 0; i < kMaxSamples; ++i)
                    if (!Near(got[i], exp[i], 2e-4f))
                    {
                        ++mism;
                        break;
                    }
            }
        }
        Check(tested > 500 && mism == 0, tag + "walking search == brute force, all start triangles (" + std::to_string(tested) + " points, " + std::to_string(mism) + " mismatches)");
        // triangles tile the hull: area = hull area (cross: 2 / 4 = 0.5 of the unit square, head: 1)
        double area = 0;
        for (int t = 0; t < L.numTris; ++t)
        {
            const Tri& T = P.TriAt(li, t);
            area += 0.5 * ((T.vx[1] - T.vx[0]) * (T.vy[2] - T.vy[0]) - (T.vy[1] - T.vy[0]) * (T.vx[2] - T.vx[0]));
        }
        const double want = std::string(L.id) == "head" ? 1.0 : 0.5;
        Check(std::fabs(area - want) < 1e-6, tag + "triangle areas tile the hull, area " + std::to_string(area));
        // continuity along a path
        {
            int c3 = -1;
            Weights prev;
            P.Evaluate(li, -1.2f, 0.0f, c3, prev);
            float prevW[kMaxSamples]{};
            for (int i = 0; i < prev.n; ++i)
                prevW[prev.sample[i]] = prev.w[i];
            double maxJump = 0;
            for (int s = 1; s <= 24000; ++s)
            {
                const float u = static_cast<float>(s) / 24000.0f;
                const float x = -1.2f + 2.4f * u, y = 1.2f * std::sin(6.0f * u * 3.14159f);
                Weights w;
                P.Evaluate(li, x, y, c3, w);
                float cur[kMaxSamples]{};
                for (int i = 0; i < w.n; ++i)
                    cur[w.sample[i]] = w.w[i];
                for (int i = 0; i < kMaxSamples; ++i)
                    maxJump = std::max(maxJump, static_cast<double>(std::fabs(cur[i] - prevW[i])));
                std::memcpy(prevW, cur, sizeof(cur));
            }
            Check(maxJump < 0.01, tag + "weights continuous along a path (max step " + std::to_string(maxJump) + ")");
        }
    }

    // golden vectors, 5-sample cross (run / walk): samples 0 Base(0,0) 1 R(1,0) 2 B(0,-1) 3 L(-1,0) 4 F(0,1)
    for (const char* id : {"run", "walk"})
    {
        const int li = P.FindLayer(id);
        auto eval = [&](float x, float y, float* out5) {
            int c = -1;
            Weights w;
            std::memset(out5, 0, 5 * sizeof(float));
            const bool ok = P.Evaluate(li, x, y, c, w);
            for (int i = 0; i < w.n; ++i)
                out5[w.sample[i]] += w.w[i];
            return ok;
        };
        float o[5];
        Check(eval(0.5f, 0.25f, o) && Near(o[1], 0.5f, 1e-5f) && Near(o[4], 0.25f, 1e-5f) && Near(o[0], 0.25f, 1e-5f) && o[2] == 0 && o[3] == 0,
              std::string(id) + ": golden (0.5, 0.25) = R .5 F .25 Base .25");
        Check(eval(-0.3f, -0.6f, o) && Near(o[3], 0.3f, 1e-5f) && Near(o[2], 0.6f, 1e-5f) && Near(o[0], 0.1f, 1e-5f) && o[1] == 0 && o[4] == 0,
              std::string(id) + ": golden (-0.3, -0.6) = L .3 B .6 Base .1");
        Check(eval(0.0f, 0.0f, o) && Near(o[0], 1.0f, 1e-6f), std::string(id) + ": origin = Base");
        Check(eval(0.0f, 0.5f, o) && Near(o[4], 0.5f, 1e-5f) && Near(o[0], 0.5f, 1e-5f), std::string(id) + ": axis point (0, 0.5)");
        // outside the diamond hull: closest perimeter point (orthogonal projection onto x + y = 1 etc.)
        Check(eval(0.8f, 0.8f, o) && Near(o[1], 0.5f, 1e-4f) && Near(o[4], 0.5f, 1e-4f) && o[0] == 0, std::string(id) + ": outside (0.8, 0.8) -> edge midpoint of R-F");
        Check(eval(1.0f, 1.0f, o) && Near(o[1], 0.5f, 1e-4f) && Near(o[4], 0.5f, 1e-4f), std::string(id) + ": corner (1, 1) -> R-F midpoint");
        Check(eval(-0.9f, -0.7f, o) && Near(o[3], 0.6f, 1e-4f) && Near(o[2], 0.4f, 1e-4f), std::string(id) + ": outside (-0.9, -0.7) -> L .6 B .4");
        Check(eval(1.0f, 0.1f, o) && Near(o[1], 0.95f, 1e-4f) && Near(o[4], 0.05f, 1e-4f), std::string(id) + ": outside (1, 0.1) -> projection (0.95, 0.05) on the R-F edge");
        // inputs beyond the range clamp to the axis range
        float a[5], b[5];
        eval(5.0f, 0.0f, a);
        eval(1.0f, 0.0f, b);
        Check(std::memcmp(a, b, sizeof(a)) == 0, std::string(id) + ": input clamped to the axis range");
        // non-finite input does not crash and gives a valid weight set
        int c = -1;
        Weights w;
        Check(P.Evaluate(li, std::nanf(""), std::nanf(""), c, w) && w.n >= 1, std::string(id) + ": NaN input handled");
    }

    // 1D layer == BlendSpace.hpp BlendSpaceEval
    {
        const int li = P.FindLayer("cmc_run");
        const Layer& L = P.LayerAt(li);
        bs::BlendSpaceDef def;
        def.name = "BS1D_Additive_Lean_Run";
        def.dims = 1;
        def.min[0] = L.min[0];
        def.max[0] = L.max[0];
        def.speedPerSec = L.interpPerS;
        for (int k = 0; k < L.numSamples; ++k)
            def.samples.push_back({P.Sample(li, k).x, 0.f, P.Sample(li, k).clip});
        bs::BlendSpaceEval ev;
        Check(ev.Build(def), "BlendSpace.hpp builds the 1D lean layer");
        int bad = 0;
        int cache = -1;
        for (int i = -130; i <= 130; ++i)
        {
            const float x = static_cast<float>(i) * 0.01f;
            Weights w;
            P.Evaluate(li, x, 0.f, cache, w);
            const auto ref = ev.Evaluate(x);
            float g[kMaxSamples]{}, r[kMaxSamples]{};
            for (int k = 0; k < w.n; ++k)
                g[w.sample[k]] += w.w[k];
            for (const auto& e : ref)
                r[e.sample] += e.weight;
            for (int k = 0; k < kMaxSamples; ++k)
                if (!Near(g[k], r[k], 1e-5f))
                    ++bad;
        }
        Check(bad == 0, "1D layer weights == BlendSpaceEval over -1.3..1.3 (" + std::to_string(bad) + " mismatches)");
        // CMC samples: Left -1, Right +1, Base 0 -> x = -0.5 is half Left half Base
        Weights w;
        int c = -1;
        P.Evaluate(li, -0.5f, 0.f, c, w);
        float g[3]{};
        for (int k = 0; k < w.n; ++k)
            g[w.sample[k]] += w.w[k];
        Check(Near(g[0], 0.5f, 1e-6f) && Near(g[2], 0.5f, 1e-6f) && g[1] == 0, "1D golden x = -0.5 = Left .5 + Base .5");
    }
}

static void TestBlendAndSmoothing(const LeanPack& P, const std::vector<uint8_t>& aBytes)
{
    // shortest-path blend: negating a sample's stored quaternion (same rotation) must not change the blend
    {
        std::vector<uint8_t> nb = aBytes;
        const size_t o = SecOff(nb, "DELT");
        const int li = P.FindLayer("run");
        const int k = 3; // Lean_L
        const size_t g = static_cast<size_t>(P.LayerAt(li).firstSample + k);
        for (int i = 0; i < kNumBones; ++i)
            for (int c = 0; c < 4; ++c)
            {
                float v;
                const size_t at = o + (g * kNumBones + i) * 40 + c * 4;
                std::memcpy(&v, nb.data() + at, 4);
                PutF(nb, at, -v);
            }
        FixCrc(nb);
        LeanPack neg;
        std::string err;
        Check(neg.LoadFromMemory(nb, 0, err), "pack with negated sample quaternions loads");
        if (neg.Ok())
        {
            const float w[kMaxSamples] = {0.4f, 0.0f, 0.0f, 0.6f, 0.0f};
            Additive a, b;
            P.BlendAdditive(li, w, a);
            neg.BlendAdditive(li, w, b);
            bool same = true;
            for (int i = 0; i < kNumBones && same; ++i)
                same = QuatDelta(a.b[i].r, b.b[i].r) < 1e-6f;
            Check(same, "BlendAdditive is invariant under q -> -q of a sample (shortest path)");
        }
    }
    // BlendAdditive == ov::BlendAdditives on random weights
    std::mt19937 rng(777);
    for (int li = 0; li < P.NumLayers(); ++li)
    {
        const Layer& L = P.LayerAt(li);
        static Additive samples[kMaxSamples];
        const Additive* ptr[kMaxSamples];
        for (int k = 0; k < L.numSamples; ++k)
        {
            P.SampleAdditive(li, k, samples[k]);
            ptr[k] = &samples[k];
        }
        bool same = true;
        for (int it = 0; it < 20 && same; ++it)
        {
            float w[kMaxSamples]{}, sum = 0;
            for (int k = 0; k < L.numSamples; ++k)
            {
                w[k] = std::uniform_real_distribution<float>(0, 1)(rng) * (it % 3 == 0 ? 0.0f : 1.0f) + (k == it % L.numSamples ? 0.2f : 0.0f);
                sum += w[k];
            }
            for (int k = 0; k < L.numSamples; ++k)
                w[k] /= sum;
            Additive a, b;
            P.BlendAdditive(li, w, a);
            ov::BlendAdditives(ptr, w, L.numSamples, b);
            for (int i = 0; i < kNumBones && same; ++i)
                same = QuatDelta(a.b[i].r, b.b[i].r) < 1e-6f && Near(a.b[i].t.x, b.b[i].t.x, 1e-6f) && Near(a.b[i].s.z, b.b[i].s.z, 1e-6f);
        }
        Check(same, std::string(L.id) + ": BlendAdditive == ov::BlendAdditives");
    }
    // real data: the lean directions are non-trivial, Base is identity
    {
        const int li = P.FindLayer("run");
        float maxDeg[5];
        for (int k = 0; k < 5; ++k)
        {
            float m = 0;
            const float* d = P.SampleDelta(li, k);
            for (int i = 0; i < kNumBones; ++i)
                m = std::max(m, ov::AngleDeg(Quat{}, Quat{d[i * 10], d[i * 10 + 1], d[i * 10 + 2], d[i * 10 + 3]}));
            maxDeg[k] = m;
        }
        Check(maxDeg[0] < 0.01f, "run Base delta is identity (max " + std::to_string(maxDeg[0]) + " deg)");
        {
            struct B { const char* id; int k; };
            const B bases[] = {{"run", 0}, {"walk", 0}, {"head", 0}, {"head", 1}, {"head", 2}, {"cmc_run", 2}};
            for (const B& bb : bases)
            {
                float m = 0;
                const int l2 = P.FindLayer(bb.id);
                const float* d = P.SampleDelta(l2, bb.k);
                for (int i = 0; i < kNumBones; ++i)
                    m = std::max(m, ov::AngleDeg(Quat{}, Quat{d[i * 10], d[i * 10 + 1], d[i * 10 + 2], d[i * 10 + 3]}));
                Check(m < 0.01f, std::string(bb.id) + " sample " + std::to_string(bb.k) + " (Base clip) delta is identity, " + std::to_string(m) + " deg");
            }
            // layers are not aliases of each other
            auto differs = [&](const char* a, int ka, const char* b2, int kb) {
                const float* x = P.SampleDelta(P.FindLayer(a), ka);
                const float* y = P.SampleDelta(P.FindLayer(b2), kb);
                float m = 0;
                for (int i = 0; i < kNumBones * 10; ++i)
                    m = std::max(m, std::fabs(x[i] - y[i]));
                return m > 1e-4f;
            };
            Check(differs("walk", 1, "run", 1) && differs("walk", 3, "run", 3) && differs("cmc_run", 0, "run", 3) && differs("head", 3, "run", 3),
                  "sample deltas of different layers are distinct data");
        }
        bool leans = true;
        for (int k = 1; k < 5; ++k)
            leans = leans && maxDeg[k] > 1.0f && maxDeg[k] < 40.0f;
        Check(leans, "run lean samples rotate some bone by 1..40 deg");
        const int hl = P.FindLayer("head");
        float hm = 0;
        const float* d = P.SampleDelta(hl, 3);
        for (int i = 0; i < kNumBones; ++i)
            hm = std::max(hm, ov::AngleDeg(Quat{}, Quat{d[i * 10], d[i * 10 + 1], d[i * 10 + 2], d[i * 10 + 3]}));
        Check(hm > 5.0f && hm < 90.0f, "head Turn_L rotates some bone by 5..90 deg (" + std::to_string(hm) + ")");
    }
    // LayerState == bs::WeightSmoother on the same targets; step response sane
    {
        const int li = P.FindLayer("run");
        LayerState st;
        bs::WeightSmoother ref;
        ref.Reset(5);
        const float dt = 1.0f / 60.0f;
        int cache = -1;
        float maxDiff = 0;
        bool monotone = true, sums = true;
        float prevR = 0;
        for (int f = 0; f < 600; ++f)
        {
            const float x = f < 60 ? 0.0f : 1.0f, y = f < 60 ? 0.0f : 0.25f;
            const float* o = st.Update(P, li, x, y, dt);
            Weights tw;
            P.Evaluate(li, x, y, cache, tw);
            std::vector<bs::Weighted> tgt;
            for (int i = 0; i < tw.n; ++i)
                tgt.push_back({tw.sample[i], tw.w[i]});
            const auto& rw = ref.Update(tgt, dt, 1.2f, true);
            if (!o)
            {
                sums = false;
                break;
            }
            float s = 0;
            for (int i = 0; i < 5; ++i)
            {
                maxDiff = std::max(maxDiff, std::fabs(o[i] - rw[static_cast<size_t>(i)]));
                s += o[i];
            }
            sums = sums && Near(s, 1.0f, 1e-5f);
            if (f >= 60)
            {
                monotone = monotone && o[1] >= prevR - 1e-6f;
                prevR = o[1];
            }
        }
        Check(sums, "LayerState output sums to 1");
        Check(maxDiff < 1e-6f, "LayerState == bs::WeightSmoother (max diff " + std::to_string(maxDiff) + ")");
        Check(monotone && prevR > 0.74f, "step response rises monotonically to the target (R -> " + std::to_string(prevR) + ")");
        LayerState st2;
        const float* first = st2.Update(P, li, 1.0f, 0.0f, dt);
        Check(first && Near(first[1], 1.0f, 1e-6f), "first Update snaps to the target");
    }
}

static void TestCurvesAndGates(const LeanPack& P)
{
    const char* relaxed[16] = {"M_Relaxed_Run_Spin_F_360_L_Lfoot", "M_Relaxed_Run_Spin_F_360_L_Rfoot", "M_Relaxed_Run_Spin_F_360_R_Lfoot", "M_Relaxed_Run_Spin_F_360_R_Rfoot",
                               "M_Relaxed_Run_Turn_L_090_Lfoot", "M_Relaxed_Run_Turn_L_090_Rfoot", "M_Relaxed_Run_Turn_R_090_Lfoot", "M_Relaxed_Run_Turn_R_090_Rfoot",
                               "M_Relaxed_Walk_Spin_F_360_L_Lfoot", "M_Relaxed_Walk_Spin_F_360_L_Rfoot", "M_Relaxed_Walk_Spin_F_360_R_Lfoot", "M_Relaxed_Walk_Spin_F_360_R_Rfoot",
                               "M_Relaxed_Walk_Turn_090_L_Lfoot", "M_Relaxed_Walk_Turn_090_L_Rfoot", "M_Relaxed_Walk_Turn_090_R_Lfoot", "M_Relaxed_Walk_Turn_090_R_Rfoot"};
    const int frames[16] = {119, 113, 118, 117, 119, 117, 117, 122, 175, 172, 171, 185, 157, 153, 154, 153};
    // frames_with_value_gt0 last index from the lean data JSON (+1 = first zero frame lower bound)
    const int lastGt0[16] = {70, 67, 69, 68, 48, 53, 51, 50, 101, 108, 100, 118, 62, 62, 63, 63};
    for (int i = 0; i < 16; ++i)
    {
        const int c = P.FindClip(relaxed[i]);
        Check(c >= 0, std::string("curve clip present: ") + relaxed[i]);
        if (c < 0)
            continue;
        const ClipRec& r = P.ClipAt(c);
        Check(r.kind == 0 && r.count == frames[i] && r.fps == 30.0f, std::string("curve clip shape: ") + relaxed[i]);
        Check(Near(P.DisableCurveAt(c, 0.0f), 1.0f, 1e-6f), std::string("curve starts at 1: ") + relaxed[i]);
        Check(P.DisableCurveAt(c, static_cast<float>(lastGt0[i] + 2) / 30.0f) < 0.02f, std::string("curve has faded out after the lean-data frame range: ") + relaxed[i]);
        Check(Near(P.DisableCurveAt(c, 100.0f), P.DisableCurveAt(c, static_cast<float>(frames[i] - 1) / 30.0f), 1e-6f) && P.DisableCurveAt(c, -1.0f) == P.DisableCurveAt(c, 0.0f),
              std::string("curve clamps outside the clip: ") + relaxed[i]);
        float prev = 2.0f;
        bool nonInc = true;
        for (int f = 0; f < r.count; ++f)
        {
            const float v = P.DisableCurveAt(c, static_cast<float>(f) / 30.0f);
            nonInc = nonInc && v <= prev + 1e-5f;
            prev = v;
        }
        Check(nonInc, std::string("curve is non-increasing: ") + relaxed[i]);
    }
    // interpolation between frames is linear
    {
        const int c = P.FindClip("M_Relaxed_Run_Turn_L_090_Lfoot");
        bool linear = true, sloped = false;
        for (int f = 0; f < P.ClipAt(c).count - 1; ++f)
        {
            const float a = P.DisableCurveAt(c, static_cast<float>(f) / 30.0f), b = P.DisableCurveAt(c, static_cast<float>(f + 1) / 30.0f);
            const float m = P.DisableCurveAt(c, (static_cast<float>(f) + 0.5f) / 30.0f);
            linear = linear && Near(m, 0.5f * (a + b), 1e-5f);
            sloped = sloped || std::fabs(a - b) > 0.01f;
        }
        Check(linear && sloped, "curve linear between frames (and has sloped segments)");
    }
    const char* neutral[8] = {"M_Neutral_Run_Turn_L_090_Lfoot", "M_Neutral_Run_Turn_L_090_Rfoot", "M_Neutral_Run_Turn_R_090_Lfoot", "M_Neutral_Run_Turn_R_090_Rfoot",
                              "M_Neutral_Walk_Turn_L_090_Lfoot", "M_Neutral_Walk_Turn_L_090_Rfoot", "M_Neutral_Walk_Turn_R_090_Lfoot", "M_Neutral_Walk_Turn_R_090_Rfoot"};
    for (const char* n : neutral)
    {
        const int c = P.FindClip(n);
        Check(c >= 0 && P.ClipAt(c).kind == 1 && Near(P.DisableCurveAt(c, 0.0f), 1.0f, 1e-6f) && P.DisableCurveAt(c, 5.0f) == 0.0f,
              std::string("proposed Neutral curve: ") + n);
    }
    Check(P.FindClip("M_Neutral_Run_Turn_L_180_Lfoot") < 0 && P.FindClip("nonexistent") < 0, "no curve for Neutral 180 clips / unknown names");

    // ABP gate helpers
    const Params& p = P.P();
    // velocity +x 4 m/s (400 cm/s >= 320 -> denominator 800), acceleration to the right of +x = -y: 3 m/s^2 -> 300 / 800
    Check(Near(LateralAccelAmount(p, 4, 0, 0, -3), 0.375f, 1e-6f), "lat accel: right turn at 4 m/s, 3 m/s^2 -> +0.375");
    Check(Near(LateralAccelAmount(p, 4, 0, 0, 3), -0.375f, 1e-6f), "lat accel: left -> -0.375");
    Check(Near(LateralAccelAmount(p, 2, 0, 0, -5), 1.0f, 1e-6f) && Near(LateralAccelAmount(p, 2, 0, 0, -50), 1.0f, 1e-6f), "lat accel: 2 m/s, 5 m/s^2 -> 1 (denominator 500), clamped");
    Check(Near(LateralAccelAmount(p, 2.6f, 0, 0, -3.25f), 3.25f * 100 / 650, 1e-5f), "lat accel: speed 260 cm/s -> denominator 650");
    Check(LateralAccelAmount(p, 4, 0, 6, 0) == 0.0f, "lat accel: along the heading = 0");
    Check(Near(LateralAccelAmount(p, 0, 4, 3, 0), 0.375f, 1e-6f), "lat accel: heading +y, right = +x");
    Check(LateralAccelAmount(p, 0, 0, 1, 1) == 0.0f, "lat accel: zero velocity = 0");
    float x, y;
    LeanInput(p, kDirF, 0.5f, x, y);
    Check(x == 0.5f && y == 0.0f, "lean input F = (lat, 0)");
    LeanInput(p, kDirB, 0.5f, x, y);
    Check(x == -0.5f && y == 0.0f, "lean input B = (-lat, 0)");
    LeanInput(p, kDirLL, 0.5f, x, y);
    Check(x == 0.0f && y == 0.5f, "lean input LL = (0, lat)");
    LeanInput(p, kDirLR, 0.5f, x, y);
    Check(x == 0.0f && y == 0.5f, "lean input LR = (0, lat)");
    LeanInput(p, kDirRL, 0.5f, x, y);
    Check(x == 0.0f && y == -0.5f, "lean input RL = (0, -lat)");
    LeanInput(p, kDirRR, 0.5f, x, y);
    Check(x == 0.0f && y == -0.5f, "lean input RR = (0, -lat)");
    LeanInput(p, 9, 0.5f, x, y);
    Check(x == 0.0f && y == 0.0f, "lean input bad direction = 0");
    Check(WalkRunAlpha(p, 2.0f) == 0.0f && WalkRunAlpha(p, 3.2f) == 1.0f && Near(WalkRunAlpha(p, 2.6f), 0.5f, 1e-6f) && WalkRunAlpha(p, 9.0f) == 1.0f,
          "walk/run alpha 200..320 cm/s");
    Check(Near(CmcLeanX(p, 1.0f, 1.65f), 0.5f, 1e-6f) && Near(CmcLeanX(p, 1.0f, 3.75f), 1.0f, 1e-6f) && Near(CmcLeanX(p, 0.5f, 2.7f), 0.5f * 0.75f, 1e-6f) &&
              CmcLeanX(p, 5.0f, 3.75f) == 1.0f,
          "cmc lean x = rel_accel * MapRangeClamped(165..375 -> 0.5..1), clamped");
}

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::printf("usage: utpp_mm_lean_pack_test <utpp_lean_woman.bin> <utpp_lean_woman.json> [utpp_lean_man.bin]\n");
        return 2;
    }
    const std::vector<uint8_t> bytes = ReadAll(argv[1]);
    Check(!bytes.empty(), std::string("woman pack readable: ") + argv[1]);
    if (bytes.empty())
    {
        std::printf("pass %d fail %d\n", g_pass, g_fail);
        return 1;
    }
    LeanPack woman;
    std::string err;
    Check(woman.LoadFromMemory(bytes, kRigIdentityWoman, err), "woman pack loads: " + err);
    if (woman.Ok())
    {
        TestRoundTrip(woman, argv[2], bytes, kRigIdentityWoman);
        TestBlendSpaces(woman);
        TestBlendAndSmoothing(woman, bytes);
        TestCurvesAndGates(woman);
    }
    TestFailClosed(bytes, kRigIdentityWoman);
    {
        LeanPack wrong;
        Check(!wrong.LoadFromMemory(bytes, kRigIdentityMan, err), "woman pack rejected for the man rig identity");
    }
    if (argc > 3)
    {
        LeanPack man;
        const std::vector<uint8_t> mb = ReadAll(argv[3]);
        Check(man.LoadFromMemory(mb, kRigIdentityMan, err), "man pack loads: " + err);
        if (man.Ok())
        {
            Check(man.NumLayers() == 4 && man.NumClips() == 24 && man.NumSamplesTotal() == woman.NumSamplesTotal(), "man pack has the same structure");
            bool sameTris = true, sameSamples = true;
            for (int li = 0; li < 4; ++li)
            {
                for (int t = 0; t < man.NumTris(li); ++t)
                    sameTris = sameTris && std::memcmp(man.TriAt(li, t).s, woman.TriAt(li, t).s, sizeof(int) * 3) == 0;
                for (int k = 0; k < man.LayerAt(li).numSamples; ++k)
                    sameSamples = sameSamples && man.Sample(li, k).x == woman.Sample(li, k).x && man.Sample(li, k).y == woman.Sample(li, k).y &&
                                  std::string(man.Sample(li, k).clip) == woman.Sample(li, k).clip;
            }
            Check(sameTris && sameSamples, "man triangulation / samples equal the woman pack (same blend space definitions)");
            Check(man.RigHash() != woman.RigHash(), "man and woman rig identities differ");
            TestBlendSpaces(man);
        }
    }
    std::printf("utpp_mm_lean_pack_test: pass %d fail %d\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
