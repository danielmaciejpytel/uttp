#pragma once
// Replay of a recorded fists sequence through the PRODUCTION glue of the melee chest base correction (mm/MeleeBase.cpp: OnArmed, SetArmedBody,
// LowerBodyKeepWeight / LowerBodyReset, BlendIntoRecords, Apply - the calls and their order of FkHook.cpp), shared by the ctest
// (MeleeBaseGlueTest.cpp) and the offline replay tool (MeleeBaseOffline.cpp, tools/melee/fist_base_offline.py), not shipped.
//
// File ("MBO2" or "MBO3", little endian), written by tools/melee/fist_base_offline.py / fist_base_blend_fixture.py from a pose recording:
//   "MBO2" | int32 nb | nb x { int16 parent, char name[32] } | int32 body (0 Female, 1 Male) | int32 nLower | nLower x int32 bone |
//   int32 nframes | nframes x { float P[nb*3] (the engine records: local positions), float Q[nb*4] (local rotations xyzw),
//     float mmP[nLower*3], float mmQ[nLower*4] (the MM pose the snapshot carries for the lower bones), float meleeWeight (snapshot
//     meleeAimWeight), float lowerScale, float upperScale, uint32 meleeBase, float dt (s since the previous frame; < 0 = a new arm:
//     OnArmed again, hard reset), float refElev (shoulder->wrist elevation of the clip at the matched time, NaN = none), int32 side }.
// "MBO3" additionally carries the MM pose of the UPPER bones the snapshot blends (the upper-body mask, upperScale): after the nLower list
//   int32 nUpper | nUpper x int32 bone, and per frame after mmQ: float mmUP[nUpper*3], float mmUQ[nUpper*4]. The records are then the pure
//   engine pose and the replay does the blend itself (BlendIntoRecords with upperBits), as the game does while the mask fades in.
//   In an "MBO2" file the upper bones are not blended (upperScale only enters the layer's weight).
// The recording's bone order is renumbered topologically (the game rig has parents[i] < i); origIndex[k] is the file index of bone k.

#define _CRT_SECURE_NO_WARNINGS
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "../src/mm/MeleeBase.hpp"
#include "../src/mm/MeleeBaseRuntime.hpp"

namespace utpp::replay
{
using namespace utpp::mm;

struct Frame
{
    std::vector<float> P, Q, mmP, mmQ, mmUP, mmUQ; // renumbered bone order
    float meleeWeight = 0, lowerScale = 1, upperScale = 0, dt = 0, refElev = NAN;
    uint32_t base = 1;
    int32_t side = 0;
};

class Replay
{
public:
    int nb = 0, body = 0;
    std::vector<std::string> names;
    std::vector<int16_t> parents;
    std::vector<int32_t> origIndex, lower, upper; // lower / upper = renumbered indices
    std::vector<Frame> frames;
    std::vector<uint8_t> records;          // the live local records (48 bytes per bone)

    bool Load(const std::string& aPath)
    {
        std::vector<uint8_t> in;
        FILE* f = std::fopen(aPath.c_str(), "rb");
        if (!f)
            return false;
        std::fseek(f, 0, SEEK_END);
        const long n = std::ftell(f);
        std::fseek(f, 0, SEEK_SET);
        in.resize(size_t(n));
        const bool got = n > 0 && std::fread(in.data(), 1, size_t(n), f) == size_t(n);
        std::fclose(f);
        if (!got || in.size() < 8 || (std::memcmp(in.data(), "MBO2", 4) != 0 && std::memcmp(in.data(), "MBO3", 4) != 0))
            return false;
        const bool v3 = std::memcmp(in.data(), "MBO3", 4) == 0;
        const uint8_t* p = in.data() + 4;
        auto rd = [&](void* dst, size_t k) {
            std::memcpy(dst, p, k);
            p += k;
        };
        int32_t nbF;
        rd(&nbF, 4);
        std::vector<std::string> nm(static_cast<size_t>(nbF));
        std::vector<int16_t> par(static_cast<size_t>(nbF));
        for (int32_t i = 0; i < nbF; ++i)
        {
            rd(&par[size_t(i)], 2);
            char s[33] = {};
            rd(s, 32);
            nm[size_t(i)] = s;
        }
        int32_t bodyF, nLow, nf;
        rd(&bodyF, 4);
        rd(&nLow, 4);
        std::vector<int32_t> lowF(static_cast<size_t>(nLow));
        for (auto& v : lowF)
            rd(&v, 4);
        int32_t nUp = 0;
        std::vector<int32_t> upF;
        if (v3)
        {
            rd(&nUp, 4);
            upF.resize(static_cast<size_t>(nUp));
            for (auto& v : upF)
                rd(&v, 4);
        }
        rd(&nf, 4);
        // topological renumbering
        std::vector<int32_t> order, newIdx(static_cast<size_t>(nbF), -1);
        {
            std::vector<char> done(static_cast<size_t>(nbF), 0);
            for (int pass = 0; pass < nbF && int32_t(order.size()) < nbF; ++pass)
                for (int32_t i = 0; i < nbF; ++i)
                    if (!done[size_t(i)] && (par[size_t(i)] < 0 || done[size_t(par[size_t(i)])]))
                    {
                        newIdx[size_t(i)] = int32_t(order.size());
                        order.push_back(i);
                        done[size_t(i)] = 1;
                    }
        }
        nb = nbF;
        body = bodyF;
        origIndex = order;
        names.resize(size_t(nb));
        parents.resize(size_t(nb));
        for (int k = 0; k < nb; ++k)
        {
            const int32_t i = order[size_t(k)];
            names[size_t(k)] = nm[size_t(i)];
            parents[size_t(k)] = par[size_t(i)] < 0 ? int16_t(-1) : int16_t(newIdx[size_t(par[size_t(i)])]);
        }
        lower.clear();
        for (int32_t v : lowF)
            lower.push_back(newIdx[size_t(v)]);
        upper.clear();
        for (int32_t v : upF)
            upper.push_back(newIdx[size_t(v)]);
        frames.resize(size_t(nf));
        std::vector<float> P(static_cast<size_t>(nb) * 3), Q(static_cast<size_t>(nb) * 4);
        for (auto& fr : frames)
        {
            rd(P.data(), P.size() * 4);
            rd(Q.data(), Q.size() * 4);
            fr.P.resize(size_t(nb) * 3);
            fr.Q.resize(size_t(nb) * 4);
            for (int k = 0; k < nb; ++k)
            {
                const int32_t i = order[size_t(k)];
                for (int c = 0; c < 3; ++c)
                    fr.P[size_t(k) * 3 + c] = P[size_t(i) * 3 + c];
                for (int c = 0; c < 4; ++c)
                    fr.Q[size_t(k) * 4 + c] = Q[size_t(i) * 4 + c];
            }
            fr.mmP.resize(size_t(nLow) * 3);
            fr.mmQ.resize(size_t(nLow) * 4);
            rd(fr.mmP.data(), fr.mmP.size() * 4);
            rd(fr.mmQ.data(), fr.mmQ.size() * 4);
            fr.mmUP.resize(size_t(nUp) * 3);
            fr.mmUQ.resize(size_t(nUp) * 4);
            rd(fr.mmUP.data(), fr.mmUP.size() * 4);
            rd(fr.mmUQ.data(), fr.mmUQ.size() * 4);
            rd(&fr.meleeWeight, 4);
            rd(&fr.lowerScale, 4);
            rd(&fr.upperScale, 4);
            rd(&fr.base, 4);
            rd(&fr.dt, 4);
            rd(&fr.refElev, 4);
            rd(&fr.side, 4);
        }
        records.assign(static_cast<size_t>(nb) * integ::kGameBoneStride, 0);
        snap = std::make_unique<integ::PoseSnapshot>();
        return true;
    }

    // Arm: the rig and the body as the game thread does at arm time. A frame with dt < 0 re-arms the RIG only (hard reset of the layer).
    void Arm()
    {
        utpp::mm::meleebase::SetArmedBody(body);
        ArmRig();
    }
    void ArmRig()
    {
        std::vector<uint64_t> hashes(static_cast<size_t>(nb));
        for (int i = 0; i < nb; ++i)
            hashes[size_t(i)] = integ::CNameHash(names[size_t(i)]);
        utpp::meleebase::OnArmed(&rigToken, hashes, parents);
        ticks = 1000 * utpp::meleebase::TicksPerSecond();
    }

    // One frame, in the order of FkHook.cpp. aBase / aLowerHipsZ are test hooks (-1 / NaN = as recorded).
    void Step(size_t aIndex, int32_t aBaseOverride = -1, float aMMHipsZOverride = NAN, float aEngineHipsZOverride = NAN)
    {
        const Frame& fr = frames[aIndex];
        if (fr.dt < 0.0f)
            ArmRig();
        else
            ticks += int64_t(double(fr.dt) * double(utpp::meleebase::TicksPerSecond()));
        for (int k = 0; k < nb; ++k)
        {
            float* r = reinterpret_cast<float*>(records.data() + size_t(k) * integ::kGameBoneStride);
            r[0] = fr.P[size_t(k) * 3], r[1] = fr.P[size_t(k) * 3 + 1], r[2] = fr.P[size_t(k) * 3 + 2], r[3] = 0.0f;
            r[4] = fr.Q[size_t(k) * 4], r[5] = fr.Q[size_t(k) * 4 + 1], r[6] = fr.Q[size_t(k) * 4 + 2], r[7] = fr.Q[size_t(k) * 4 + 3];
            r[8] = r[9] = r[10] = r[11] = 1.0f;
        }
        if (!std::isnan(aEngineHipsZOverride))
            reinterpret_cast<float*>(records.data() + size_t(Find("Hips")) * integ::kGameBoneStride)[2] = aEngineHipsZOverride;
        integ::PoseSnapshot& s = *snap;
        s = integ::PoseSnapshot{};
        s.weight = 1.0f;
        // the snapshot carries the MM pose of the lower bones only (the recording has 185 bones, a snapshot at most 80; the upper bones are
        // on the game pose in these windows - upperScale 0 - and a blend of the records with themselves changes nothing)
        s.numBones = uint32_t(lower.size() + upper.size());
        s.firstBone = 0;
        s.rig = reinterpret_cast<uint64_t>(&rigToken);
        s.upperScale = fr.upperScale;
        s.lowerScale = fr.lowerScale;
        s.meleeAimWeight = fr.meleeWeight;
        s.meleeBase = aBaseOverride >= 0 ? uint32_t(aBaseOverride) : fr.base;
        for (size_t j = 0; j < lower.size(); ++j)
        {
            const size_t k = size_t(lower[j]);
            s.gameIndex[j] = int32_t(k);
            for (int c = 0; c < 3; ++c)
                s.pose[j * 7 + size_t(c)] = fr.mmP[j * 3 + size_t(c)];
            for (int c = 0; c < 4; ++c)
                s.pose[j * 7 + 3 + size_t(c)] = fr.mmQ[j * 4 + size_t(c)];
            if (!std::isnan(aMMHipsZOverride) && names[k] == "Hips")
                s.pose[j * 7 + 2] = aMMHipsZOverride;
        }
        for (size_t u = 0; u < upper.size(); ++u)
        {
            const size_t j = lower.size() + u, k = size_t(upper[u]);
            s.gameIndex[j] = int32_t(k);
            s.upperBits[j >> 5] |= 1u << (j & 31u);
            for (int c = 0; c < 3; ++c)
                s.pose[j * 7 + size_t(c)] = fr.mmUP[u * 3 + size_t(c)];
            for (int c = 0; c < 4; ++c)
                s.pose[j * 7 + 3 + size_t(c)] = fr.mmUQ[u * 4 + size_t(c)];
        }
        uint8_t* bones = records.data();
        void* rig = &rigToken;
        const float keep = s.lowerScale < 1.0f ? utpp::meleebase::LowerBodyKeepWeight(bones, uint32_t(nb), rig, s, ticks) : utpp::meleebase::LowerBodyReset();
        lastKeep = keep;
        utpp::meleebase::CaptureEngineKeys(bones, uint32_t(nb), rig, s.meleeAimWeight); // before the blend, as FkHook.cpp
        integ::BlendIntoRecords(bones, uint32_t(nb), s, s.weight, keep);
        utpp::meleebase::Apply(bones, uint32_t(nb), rig, s.weight, s.upperScale, s.meleeAimWeight, s.meleeBase, ticks);
    }

    // model positions / rotations of the current records (renumbered order)
    std::vector<aim::Xf> Model() const
    {
        std::vector<aim::Xf> m(static_cast<size_t>(nb));
        for (int k = 0; k < nb; ++k)
        {
            const aim::Xf l = aim::ReadLocal(records.data(), uint32_t(k));
            m[size_t(k)] = parents[size_t(k)] < 0 ? l : aim::Mul(m[size_t(parents[size_t(k)])], l);
        }
        return m;
    }
    int Find(const char* aName) const
    {
        for (int k = 0; k < nb; ++k)
            if (names[size_t(k)] == aName)
                return k;
        return -1;
    }
    float lastKeep = 0.0f;
    int64_t ticks = 0;

private:
    std::unique_ptr<integ::PoseSnapshot> snap;
    int rigToken = 0;
};
} // namespace utpp::replay
