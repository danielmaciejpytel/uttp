// Offline replay of the head look layer (src/mm/HeadLook.hpp) on recorded in-game poses, not shipped.
// Usage: utpp_headlook_replay <fixture.txt> [out.csv]   Exit code 0 = all checks passed.
//
// The fixture (tools/analysis/head_look_replay.py export; tests/data/headlook_replay_frames.txt) holds, per frame, the local records
// of the bones of the head chain as the FK detour saw them AFTER the motion blend, the aim offset and the melee tilt (the stage the
// layer runs in; recorded with the head look off in the DLL), the head look decision of utpp_core (weight, mode) and the target the
// camera defines (direction from the head of the recorded pose: yaw left > 0, pitch up > 0, distance). Every frame is run through the real layer exactly like the FK detour does:
// open loop (each call starts from the recorded pose), dt = the time between frames.
//
// Fixture format (whitespace separated tokens):
//   rig <N>                                  bones 0..N-1 of the player rig
//   bone <index> <name> <parent> <recorded>  N lines; recorded 1 = the frames carry its local record (p.xyz, q.xyzw), else identity
//   frames <M>
//   frame <t seconds> <weight> <mode> <chestAllowed> <yawDeg> <pitchDeg> <dist m> then 7 floats per recorded bone (index order)   M times
//
// Checks: no non-finite output; every bone outside the chain (Spine3, Neck, Neck1, Head, LeftEye, RightEye) bit-identical; bone
// lengths kept; frames the layer did not write are bit-identical; the head + eyes reach the gaze once the gaze has been steady for
// 0.8 s (within 4 deg, outside the chain limit); the frame-to-frame change of the applied turn stays below 600 deg/s; chest share
// and eye limits; the layer is at rest again after the last release. Gate pass: the same frames stepped three times each (about
// 60 Hz, uneven) with a layer gate that switches in single steps; no step may move the applied head / chest turn by more than 30 %
// of the turn held before the edge.
#define _CRT_SECURE_NO_WARNINGS
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "../src/mm/HeadLook.hpp"

using namespace utpp::mm;
using namespace utpp::mm::headlook;
using math::Quat;
using math::Vec3;

static constexpr float kR2D = 57.2957795f;
static int g_fail = 0, g_pass = 0;

static void Expect(const char* aName, bool aOk, const std::string& aDetail = {})
{
    std::printf("[%s] %s%s%s\n", aOk ? "PASS" : "FAIL", aName, aDetail.empty() ? "" : " -- ", aDetail.c_str());
    (aOk ? g_pass : g_fail)++;
}

static std::string Fmt(const char* aFmt, double a = 0, double b = 0, double c = 0, double d = 0)
{
    char buf[256];
    std::snprintf(buf, sizeof(buf), aFmt, a, b, c, d);
    return buf;
}

struct Frame
{
    double t = 0;
    float weight = 0;
    int mode = 0;
    int chestAllowed = 0;
    float yawDeg = 0, pitchDeg = 0, dist = 5.0f; // the target direction seen from the head of the recorded pose
    std::vector<float> rec; // 7 per recorded bone
};

struct Fixture
{
    std::vector<std::string> names;
    std::vector<int16_t> parents;
    std::vector<int> recorded; // bone indices with records
    std::vector<Frame> frames;
};

static bool Load(const char* aPath, Fixture& aOut)
{
    std::ifstream in(aPath);
    if (!in)
        return false;
    std::string tok;
    size_t n = 0;
    while (in >> tok)
    {
        if (tok == "rig")
        {
            in >> n;
            aOut.names.assign(n, "");
            aOut.parents.assign(n, -1);
            std::vector<int> rec(n, 0);
            for (size_t i = 0; i < n; ++i)
            {
                std::string b, name;
                size_t idx;
                int parent, recorded;
                in >> b >> idx >> name >> parent >> recorded;
                if (b != "bone" || idx >= n)
                    return false;
                aOut.names[idx] = name;
                aOut.parents[idx] = static_cast<int16_t>(parent);
                rec[idx] = recorded;
            }
            for (size_t i = 0; i < n; ++i)
                if (rec[i])
                    aOut.recorded.push_back(static_cast<int>(i));
        }
        else if (tok == "frames")
        {
            size_t m;
            in >> m;
            aOut.frames.resize(m);
            for (size_t f = 0; f < m; ++f)
            {
                std::string w;
                Frame& fr = aOut.frames[f];
                in >> w >> fr.t >> fr.weight >> fr.mode >> fr.chestAllowed >> fr.yawDeg >> fr.pitchDeg >> fr.dist;
                if (w != "frame")
                    return false;
                fr.rec.resize(aOut.recorded.size() * 7);
                for (float& v : fr.rec)
                    in >> v;
            }
            return static_cast<bool>(in) || in.eof();
        }
    }
    return false;
}

static float YawOfBone(const aim::Xf* aModel, int aBone, float* aPitch = nullptr)
{
    float y = 0, p = 0;
    aim::YawPitchOf(math::QuatRotate(aModel[aBone].q, kLocalForward), y, p);
    if (aPitch)
        *aPitch = p;
    return y;
}

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::printf("usage: utpp_headlook_replay <fixture.txt> [out.csv]\n");
        return 2;
    }
    Fixture fx;
    if (!Load(argv[1], fx) || fx.names.empty() || fx.frames.empty())
    {
        std::printf("[FAIL] fixture %s unreadable\n", argv[1]);
        return 1;
    }
    const uint32_t n = static_cast<uint32_t>(fx.names.size());
    std::vector<uint64_t> hashes;
    for (const std::string& s : fx.names)
        hashes.push_back(integ::CNameHash(s));
    const Rig rig = ResolveRig(hashes, fx.parents);
    Expect("fixture: rig resolves (Spine3, Neck, Neck1, Head, eyes)", rig.ok == 1 && rig.eye[0] >= 0 && rig.eye[1] >= 0);
    if (!rig.ok)
        return 1;
    std::FILE* csv = argc > 2 ? std::fopen(argv[2], "w") : nullptr;
    if (csv)
        std::fprintf(csv, "t,weight,mode,gaze_yaw,gaze_pitch,head_yaw_in,head_pitch_in,head_yaw_out,head_pitch_out,chest_yaw_in,chest_yaw_out,"
                          "eye_yaw,eye_pitch,wrote,flags\n");

    const std::vector<int> chain = {rig.spine3, rig.neck, rig.neck1, rig.head, rig.eye[0], rig.eye[1]};
    auto isChain = [&](uint32_t b) {
        for (int c : chain)
            if (static_cast<int>(b) == c)
                return true;
        return false;
    };
    std::vector<uint8_t> base(size_t(n) * integ::kGameBoneStride), work;
    Layer layer;
    Output out;
    bool finite = true, untouched = true, lengths = true, identicalWhenIdle = true, chestOk = true, eyesOk = true;
    int writtenFrames = 0, idleFrames = 0, settledFrames = 0, settledBad = 0, jumpFrames = 0;
    double worstTrack = 0, worstRate = 0;
    float prevApplied = 0.0f;
    double prevT = fx.frames.front().t;
    // steady-gaze tracker: the last time the gaze (clamped) moved by more than 1 deg
    float refYaw = 0, refPitch = 0;
    double refT = fx.frames.front().t;
    bool lastWeightOn = false;
    for (size_t f = 0; f < fx.frames.size(); ++f)
    {
        const Frame& fr = fx.frames[f];
        // pose of this frame
        for (uint32_t b = 0; b < n; ++b)
        {
            float* r = reinterpret_cast<float*>(base.data() + size_t(b) * integ::kGameBoneStride);
            r[0] = r[1] = r[2] = r[3] = 0.0f;
            r[4] = r[5] = r[6] = 0.0f;
            r[7] = 1.0f;
            r[8] = r[9] = r[10] = r[11] = 1.0f;
        }
        for (size_t k = 0; k < fx.recorded.size(); ++k)
        {
            float* r = reinterpret_cast<float*>(base.data() + size_t(fx.recorded[k]) * integ::kGameBoneStride);
            const float* v = &fr.rec[k * 7];
            r[0] = v[0], r[1] = v[1], r[2] = v[2];
            r[4] = v[3], r[5] = v[4], r[6] = v[5], r[7] = v[6];
        }
        work = base;
        aim::Xf mIn[aim::kMaxFkBones], mOut[aim::kMaxFkBones];
        aim::ComputeModel(base.data(), n, rig.parents, n, mIn);
        Input in;
        in.targetValid = fr.weight > 0.0f;
        {
            const float y = fr.yawDeg / kR2D, p = fr.pitchDeg / kR2D;
            in.target = mIn[rig.head].p + Vec3{-std::sin(y) * std::cos(p) * fr.dist, std::cos(y) * std::cos(p) * fr.dist, std::sin(p) * fr.dist};
        }
        in.weight = fr.weight;
        in.mode = fr.mode;
        in.chest = 0.5f;
        in.maxYawDeg = kDefaultMaxYawDeg;
        in.gate = 1.0f;
        in.chestAllowed = fr.chestAllowed != 0;
        const double dtRaw = f == 0 ? 0.0 : fr.t - prevT;
        in.dt = static_cast<float>(dtRaw);
        prevT = fr.t;
        const bool ok = layer.Run(work.data(), n, rig, in, out);
        if (!ok)
            finite = false;
        aim::ComputeModel(work.data(), n, rig.parents, n, mOut);
        float pIn = 0, pOut = 0, eyeP = 0;
        const float yIn = YawOfBone(mIn, rig.head, &pIn), yOut = YawOfBone(mOut, rig.head, &pOut);
        const float cIn = YawOfBone(mIn, rig.spine3), cOut = YawOfBone(mOut, rig.spine3);
        const float eyeY = YawOfBone(mOut, rig.eye[0], &eyeP);
        for (uint32_t b = 0; b < n; ++b)
        {
            const bool same = std::memcmp(work.data() + size_t(b) * integ::kGameBoneStride, base.data() + size_t(b) * integ::kGameBoneStride,
                                          integ::kGameBoneStride) == 0;
            if (!same && !isChain(b))
                untouched = false;
            const float* r = reinterpret_cast<const float*>(work.data() + size_t(b) * integ::kGameBoneStride);
            for (int k = 0; k < 8; ++k)
                finite = finite && std::isfinite(r[k]);
        }
        if (!out.wrote)
        {
            ++idleFrames;
            if (work != base)
                identicalWhenIdle = false;
        }
        else
            ++writtenFrames;
        // bone lengths of the chain (parent -> child distance)
        for (int c : chain)
            if (rig.parents[c] >= 0 && std::fabs(aim::Length(mOut[c].p - mOut[rig.parents[c]].p) - aim::Length(mIn[c].p - mIn[rig.parents[c]].p)) > 1e-5f)
                lengths = false;
        // chest share and eye limits
        if (std::fabs(cOut - cIn) > kChestMaxDeg * 0.01745329f + 1e-3f || (!in.chestAllowed && std::fabs(WrapPi(cOut - cIn)) > 1e-4f))
            chestOk = false;
        const float eyeRelYaw = WrapPi(eyeY - yOut) * kR2D, eyeRelPitch = (eyeP - pOut) * kR2D;
        if (std::fabs(eyeRelYaw) > kEyeYawDeg + 0.5f || eyeRelPitch > kEyeUpDeg + 0.5f || eyeRelPitch < -kEyeDownDeg - 0.5f)
            eyesOk = false;
        // rate of the applied turn (head yaw out - head yaw in), deg/s
        const float applied = WrapPi(yOut - yIn) * kR2D;
        if (f > 0 && dtRaw > 1e-3)
        {
            const double rate = std::fabs(applied - prevApplied) / dtRaw;
            worstRate = std::fmax(worstRate, rate);
            if (rate > 600.0)
                ++jumpFrames;
        }
        prevApplied = applied;
        // tracking: the head + eyes reach the (steady) gaze
        const bool on = fr.weight > 0.0f;
        if (!on || !lastWeightOn || std::fabs(WrapPi(out.gazeYaw - refYaw)) * kR2D > 1.0f || std::fabs(out.gazePitch - refPitch) * kR2D > 1.0f)
        {
            refYaw = out.gazeYaw;
            refPitch = out.gazePitch;
            refT = fr.t;
        }
        lastWeightOn = on;
        if (on && fr.t - refT >= 0.8 && !(out.flags & (kFlagChain | kFlagEyes | kFlagNoTarget)))
        {
            ++settledFrames;
            const double err = std::fabs(WrapPi(yOut + WrapPi(eyeY - yOut) - out.gazeYaw)) * kR2D;
            worstTrack = std::fmax(worstTrack, err);
            if (err > 4.0)
                ++settledBad;
        }
        if (csv)
            std::fprintf(csv, "%.4f,%g,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%u\n", fr.t, static_cast<double>(fr.weight), fr.mode,
                         static_cast<double>(out.gazeYaw * kR2D), static_cast<double>(out.gazePitch * kR2D), static_cast<double>(yIn * kR2D),
                         static_cast<double>(pIn * kR2D), static_cast<double>(yOut * kR2D), static_cast<double>(pOut * kR2D),
                         static_cast<double>(cIn * kR2D), static_cast<double>(cOut * kR2D), static_cast<double>(eyeRelYaw),
                         static_cast<double>(eyeRelPitch), out.wrote ? 1 : 0, static_cast<unsigned>(out.flags));
    }
    if (csv)
        std::fclose(csv);

    // Gate pass: the same frames, the layer stepped three times per recorded frame (about 60 Hz with the uneven intervals of the
    // recording), the layer gate switched in single steps: closed for 0.5 s, a one-step dropout, held at 0.4 for 0.6 s, closed for
    // 0.3 s. Measured at every gate edge (the 20 following steps): the largest single-step change of the applied head / chest turn
    // relative to the turn held before the edge, and the peak angular speed of the applied head turn.
    struct GateEvent
    {
        double t0, t1;
        float gate;
    };
    const double tStart = fx.frames.front().t;
    const GateEvent gates[] = {{3.0, 3.5, 0.0f}, {6.6, 6.6, 0.0f}, {8.0, 8.6, 0.4f}, {11.6, 11.9, 0.0f}};
    auto gateAt = [&](double aT, double aDt) {
        const double rel = aT - tStart;
        for (const GateEvent& g : gates)
            if ((g.t1 > g.t0 && rel >= g.t0 && rel < g.t1) || (g.t1 == g.t0 && rel >= g.t0 && rel < g.t0 + aDt))
                return g.gate;
        return 1.0f;
    };
    Layer gl;
    float gPrevGate = 1.0f, gPrevHead = 0.0f, gPrevChest = 0.0f, gHeld = 0.0f, gHeldChest = 0.0f;
    double gWorstRel = 0.0, gWorstRelChest = 0.0, gWorstSpeed = 0.0, gWorstStep = 0.0;
    int gEdges = 0, gWindow = 0;
    bool gFinite = true;
    for (size_t f = 0; f < fx.frames.size(); ++f)
    {
        const Frame& fr = fx.frames[f];
        for (uint32_t b = 0; b < n; ++b)
        {
            float* r = reinterpret_cast<float*>(base.data() + size_t(b) * integ::kGameBoneStride);
            r[0] = r[1] = r[2] = r[3] = 0.0f;
            r[4] = r[5] = r[6] = 0.0f;
            r[7] = 1.0f;
            r[8] = r[9] = r[10] = r[11] = 1.0f;
        }
        for (size_t k = 0; k < fx.recorded.size(); ++k)
        {
            float* r = reinterpret_cast<float*>(base.data() + size_t(fx.recorded[k]) * integ::kGameBoneStride);
            const float* v = &fr.rec[k * 7];
            r[0] = v[0], r[1] = v[1], r[2] = v[2];
            r[4] = v[3], r[5] = v[4], r[6] = v[5], r[7] = v[6];
        }
        aim::Xf mIn[aim::kMaxFkBones], mOut[aim::kMaxFkBones];
        aim::ComputeModel(base.data(), n, rig.parents, n, mIn);
        const float yIn = YawOfBone(mIn, rig.head), cIn = YawOfBone(mIn, rig.spine3);
        Input in;
        in.targetValid = fr.weight > 0.0f;
        {
            const float y = fr.yawDeg / kR2D, p = fr.pitchDeg / kR2D;
            in.target = mIn[rig.head].p + Vec3{-std::sin(y) * std::cos(p) * fr.dist, std::cos(y) * std::cos(p) * fr.dist, std::sin(p) * fr.dist};
        }
        in.weight = fr.weight;
        in.mode = fr.mode;
        in.chest = 0.5f;
        in.maxYawDeg = kDefaultMaxYawDeg;
        in.chestAllowed = fr.chestAllowed != 0;
        const double frameDt = f == 0 ? 0.0 : fr.t - fx.frames[f - 1].t;
        for (int s = 0; s < 3; ++s)
        {
            const double sdt = frameDt / 3.0;
            const double ts = (f == 0 ? fr.t : fx.frames[f - 1].t) + sdt * (s + 1);
            in.dt = static_cast<float>(sdt);
            in.gate = gateAt(ts, sdt);
            work = base;
            if (!gl.Run(work.data(), n, rig, in, out))
                gFinite = false;
            aim::ComputeModel(work.data(), n, rig.parents, n, mOut);
            const float head = WrapPi(YawOfBone(mOut, rig.head) - yIn) * kR2D;
            const float chest = WrapPi(YawOfBone(mOut, rig.spine3) - cIn) * kR2D;
            gFinite = gFinite && std::isfinite(head) && std::isfinite(chest);
            if (in.gate != gPrevGate)
            {
                ++gEdges;
                gWindow = 20;
                gHeld = std::fabs(gPrevHead);
                gHeldChest = std::fabs(gPrevChest);
            }
            else if (gWindow > 0 && sdt > 1e-4)
            {
                const double d = std::fabs(head - gPrevHead), dc = std::fabs(chest - gPrevChest);
                gWorstStep = std::fmax(gWorstStep, d);
                gWorstSpeed = std::fmax(gWorstSpeed, d / sdt);
                if (gHeld > 2.0f)
                    gWorstRel = std::fmax(gWorstRel, d / gHeld);
                if (gHeldChest > 1.0f)
                    gWorstRelChest = std::fmax(gWorstRelChest, dc / gHeldChest);
                --gWindow;
            }
            if (in.gate != gPrevGate && sdt > 1e-4)
            {
                // the edge step itself
                const double d = std::fabs(head - gPrevHead), dc = std::fabs(chest - gPrevChest);
                gWorstStep = std::fmax(gWorstStep, d);
                gWorstSpeed = std::fmax(gWorstSpeed, d / sdt);
                if (gHeld > 2.0f)
                    gWorstRel = std::fmax(gWorstRel, d / gHeld);
                if (gHeldChest > 1.0f)
                    gWorstRelChest = std::fmax(gWorstRelChest, dc / gHeldChest);
            }
            gPrevGate = in.gate;
            gPrevHead = head;
            gPrevChest = chest;
        }
    }
    std::printf("gate pass: %d edges, largest step %.2f deg, peak %.0f deg/s, largest step / held turn: head %.2f chest %.2f\n", gEdges,
                gWorstStep, gWorstSpeed, gWorstRel, gWorstRelChest);

    Expect("replay: every output record finite, the layer never failed", finite);
    Expect("replay: bones outside the chain are bit-identical in every frame", untouched);
    Expect("replay: bone lengths (parent -> child) of the chain unchanged", lengths);
    Expect("replay: a frame the layer did not write is bit-identical to the recorded pose", identicalWhenIdle,
           Fmt("written %g idle %g", writtenFrames, idleFrames));
    Expect("replay: the layer wrote on some frames and was idle on others (the fixture covers on and off)", writtenFrames > 20 && idleFrames > 5);
    Expect("replay: chest turn <= 25 deg and zero without the chest permission; eyes within +-30 / +20 / -20 deg of the head", chestOk && eyesOk);
    Expect("replay: head + eyes reach the gaze (4 deg) once the gaze was steady for 0.8 s", settledFrames > 10 && settledBad == 0,
           Fmt("settled frames %g, worst error %.2f deg, bad %g", settledFrames, worstTrack, settledBad));
    Expect("replay: the applied turn never changes faster than 600 deg/s between frames", jumpFrames == 0, Fmt("worst %.0f deg/s, frames above %g", worstRate, jumpFrames));
    Expect("replay: the layer is at rest again at the end (the last frames are released)", layer.Idle() || fx.frames.back().weight > 0.0f);
    Expect("replay, gate pass: finite output; the gate edges were reached", gFinite && gEdges >= 6, Fmt("edges %g", gEdges));
    Expect("replay, gate pass: a gate stepping in one frame never moves the applied head / chest turn by more than 30 % of the turn held "
           "before the edge in one step",
           gWorstRel <= 0.30 && gWorstRelChest <= 0.30,
           Fmt("head %.2f chest %.2f, peak %.0f deg/s, largest step %.2f deg", gWorstRel, gWorstRelChest, gWorstSpeed, gWorstStep));
    std::printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
