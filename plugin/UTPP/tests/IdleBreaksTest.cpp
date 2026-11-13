// Offline test of the motion idle-break player (src/mm/IdleBreaks.hpp, report 2.5.1 / 3.18, stage 10.9).
//
//   utpp_mm_idlebreaks_test --rules <choosers_rules.txt> --clips <retarget out dir>... [--db <v2 .utppmm>]...
//
// 1. Rule fidelity: the 10 in-code rows equal the IdleBreak rows of CHT_AnimationsForStateMachine in the extracted
//    chooser rules (clip, Get_AO_Yaw range, StartTime, BlendTime, UseMM=False, weight); Stand_Idle_Break_v05 has none.
// 2. Clip presence by name: all 11 break clips in every --clips dir (fail if missing; durations, Disable_AO set =
//    Stand v03/v04/v05 + Crouch v01/v04/v05); names searched in every --db (listed, informational: breaks are not
//    meant to be in the MM databases).
// 3. Selection: stand / crouch sets, inclusive Get_AO_Yaw bounds, OrientToMovement ignores the yaw, uniform
//    distribution, deterministic sequence per seed.
// 4. Controller at 30 / 60 / 144 fps on the real clip data: break after > 3 s idle, StartTime / blend 0.5, end at
//    <= 0.75 s remaining with blend 0.75, timer restart, no break while moving / input / aiming / air / turn /
//    transition, interruption on the same update (<= 0.2 s), stance change, Disable_AO weight, NoValidAnim.

#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "../src/mm/IdleBreaks.hpp"
#include "../src/mm/MiniJson.hpp"

using namespace utpp::mm::idlebreak;

static int g_pass = 0;
static int g_fail = 0;

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

static bool ReadFile(const std::string& aPath, std::string& aOut)
{
    std::ifstream f(aPath, std::ios::binary);
    if (!f)
        return false;
    std::ostringstream ss;
    ss << f.rdbuf();
    aOut = ss.str();
    return true;
}

static float ParseAfter(const std::string& aLine, const std::string& aKey, bool& aFound)
{
    const size_t p = aLine.find(aKey);
    aFound = p != std::string::npos;
    return aFound ? std::stof(aLine.substr(p + aKey.size())) : 0.0f;
}

// ---- 1. rule fidelity
static void TestRules(const std::string& aPath)
{
    std::string text;
    Check(ReadFile(aPath, text), "read rules " + aPath);
    std::istringstream in(text);
    std::string line;
    int n = 0;
    bool v05Row = false;
    while (std::getline(in, line))
    {
        if (line.find("StateMachineState in {IdleBreak}") == std::string::npos || line.find("Idle_Break") == std::string::npos)
            continue;
        if (line.find(kUnlistedClip) != std::string::npos)
            v05Row = true;
        if (n >= kRowCount)
        {
            ++n;
            continue;
        }
        const Row& r = kRows[n];
        const size_t arrow = line.find("=> ");
        const std::string clip = arrow == std::string::npos ? "" : line.substr(arrow + 3, line.find(' ', arrow + 3) - arrow - 3);
        Check(clip == r.clip, "rule row " + std::to_string(n) + " clip " + clip + " == " + std::string(r.clip));
        const size_t ao = line.find(" <= Get_AO_Yaw <= ");
        Check((ao != std::string::npos) == r.hasAoRange, "rule row " + std::to_string(n) + " AO range presence");
        if (ao != std::string::npos)
        {
            const size_t amp = line.rfind("&& ", ao);
            const float lo = std::stof(line.substr(amp + 3, ao - amp - 3));
            const float hi = std::stof(line.substr(ao + std::string(" <= Get_AO_Yaw <= ").size()));
            Check(lo == r.aoYawMin && hi == r.aoYawMax, "rule row " + std::to_string(n) + " AO range values");
        }
        bool f1 = false, f2 = false, f3 = false;
        const float st = ParseAfter(line, "StartTime=", f1);
        const float bt = ParseAfter(line, "BlendTime=", f2);
        const float w = ParseAfter(line, "weight=", f3);
        Check(f1 && std::fabs(st - r.startTime) < 1e-6f, "rule row " + std::to_string(n) + " StartTime");
        Check(f2 && std::fabs(bt - r.blendTime) < 1e-6f, "rule row " + std::to_string(n) + " BlendTime");
        Check(f3 && std::fabs(w - r.weight) < 1e-6f, "rule row " + std::to_string(n) + " weight");
        Check(line.find("UseMM=False") != std::string::npos, "rule row " + std::to_string(n) + " UseMM=False");
        Check((line.find("Stand_") != std::string::npos) == (r.stance == Stance::Stand), "rule row stance");
        ++n;
    }
    Check(n == kRowCount, "IdleBreak rule rows " + std::to_string(n) + " == 10");
    Check(!v05Row, "Stand_Idle_Break_v05 has no chooser row");
    std::printf("rules: %d IdleBreak rows compared\n", n);
}

// ---- 2. clips
static std::vector<std::string> AllBreakClips()
{
    std::vector<std::string> v;
    for (const Row& r : kRows)
        v.emplace_back(r.clip);
    v.emplace_back(kUnlistedClip);
    return v;
}

static bool LoadClip(const std::string& aPath, ClipData& aOut)
{
    std::string text, err;
    if (!ReadFile(aPath, text))
        return false;
    utpp::json::Value v;
    if (!utpp::json::Parse(text, v, err))
    {
        std::printf("parse error %s: %s\n", aPath.c_str(), err.c_str());
        return false;
    }
    const auto* fps = v.Find("fps");
    const auto* nf = v.Find("num_frames");
    if (!fps || !nf || fps->num <= 0.0 || nf->num < 2.0)
        return false;
    aOut.fps = static_cast<float>(fps->num);
    aOut.duration = static_cast<float>((nf->num - 1.0) / fps->num);
    aOut.disableAO.clear();
    if (const auto* curves = v.Find("curves"))
        if (const auto* dao = curves->Find("disable_ao"); dao && dao->IsArray())
            for (const auto& s : dao->arr)
                aOut.disableAO.push_back(static_cast<float>(s.num));
    return true;
}

static std::map<std::string, ClipData> TestClips(const std::vector<std::string>& aDirs)
{
    std::map<std::string, ClipData> first;
    const char* withCurve[] = {"M_Neutral_Stand_Idle_Break_v03", "M_Neutral_Stand_Idle_Break_v04",
                               "M_Neutral_Stand_Idle_Break_v05", "M_Neutral_Crouch_Idle_Break_v01",
                               "M_Neutral_Crouch_Idle_Break_v04", "M_Neutral_Crouch_Idle_Break_v05"};
    for (size_t d = 0; d < aDirs.size(); ++d)
    {
        int found = 0, curves = 0;
        std::string missing;
        for (const std::string& name : AllBreakClips())
        {
            ClipData c;
            if (!LoadClip(aDirs[d] + "/" + name + ".utppclip.json", c))
            {
                missing += " " + name;
                continue;
            }
            ++found;
            bool expectCurve = false;
            for (const char* w : withCurve)
                expectCurve |= name == w;
            float mx = 0.0f;
            for (float s : c.disableAO)
                mx = std::max(mx, s);
            Check(expectCurve == (mx > 0.5f), aDirs[d] + " " + name + " Disable_AO presence");
            curves += mx > 0.5f ? 1 : 0;
            Check(c.duration > 0.75f + 0.6f, name + " duration > StartTime + 0.75");
            if (d == 0)
                first[name] = c;
        }
        std::printf("clips %s: %d/11 found, %d with Disable_AO, missing:%s\n", aDirs[d].c_str(), found, curves,
                    missing.empty() ? " none" : missing.c_str());
        Check(found == 11, "all 11 idle-break clips in " + aDirs[d]);
        Check(curves == 6, "6 clips with Disable_AO in " + aDirs[d]);
    }
    return first;
}

static void ListDb(const std::string& aPath)
{
    std::string bytes;
    if (!ReadFile(aPath, bytes))
    {
        Check(false, "read db " + aPath);
        return;
    }
    std::string missing;
    int present = 0;
    for (const std::string& name : AllBreakClips())
    {
        if (bytes.find(name) != std::string::npos)
            ++present;
        else
            missing += " " + name;
    }
    // sanity of the name search: the stand idle loop is in every v2 database
    Check(bytes.find(kIdleLoopClip[0]) != std::string::npos, "name search finds M_Neutral_Stand_Idle_Loop in " + aPath);
    std::printf("db %s: %d/11 idle-break clips by name; missing (expected, breaks are not MM clips):%s\n", aPath.c_str(),
                present, missing.empty() ? " none" : missing.c_str());
}

// ---- 3. selection
static void TestSelection()
{
    auto setOf = [](Stance s, float yaw) {
        std::vector<int> v;
        for (int i = 0; i < kRowCount; ++i)
            if (RowPasses(kRows[i], s, yaw))
                v.push_back(i);
        return v;
    };
    Check(setOf(Stance::Stand, 0.0f) == std::vector<int>{0, 1, 2, 3, 4}, "stand yaw 0: all 5 stand rows");
    Check(setOf(Stance::Stand, 90.0f) == std::vector<int>{0, 2, 3, 4}, "stand yaw 90: no v04");
    Check(setOf(Stance::Stand, -90.0f) == std::vector<int>{0, 1, 3, 4}, "stand yaw -90: no v03");
    Check(setOf(Stance::Stand, 10.0f) == std::vector<int>{0, 1, 2, 3, 4}, "stand yaw 10: inclusive bound");
    Check(setOf(Stance::Stand, 10.01f) == std::vector<int>{0, 2, 3, 4}, "stand yaw 10.01: v04 out");
    Check(setOf(Stance::Stand, -10.01f) == std::vector<int>{0, 1, 3, 4}, "stand yaw -10.01: v03 out");
    Check(setOf(Stance::Crouch, 0.0f) == std::vector<int>{5, 6, 7, 8, 9}, "crouch yaw 0: all 5 crouch rows");
    Check(setOf(Stance::Crouch, 90.0f) == std::vector<int>{5, 6, 7}, "crouch yaw 90: v01 v02 v03");
    Check(setOf(Stance::Crouch, -90.0f) == std::vector<int>{6, 7, 8, 9}, "crouch yaw -90: v02-v05");
    Check(ChooserAoYaw(false, 120.0f, 0.0f) == 0.0f, "OrientToMovement: Get_AO_Yaw = 0");
    Check(ChooserAoYaw(true, 120.0f, 0.5f) == 60.0f, "Strafe: Get_AO_Yaw lerped by Disable_AO");

    for (int st = 0; st < 2; ++st)
    {
        const Stance s = static_cast<Stance>(st);
        Rng rng(12345);
        int counts[kRowCount] = {};
        const int n = 100000;
        for (int i = 0; i < n; ++i)
            ++counts[SelectRow(s, 0.0f, rng)];
        int wrong = 0;
        for (int i = 0; i < kRowCount; ++i)
        {
            if (kRows[i].stance != s)
                wrong += counts[i];
            else
                Check(std::fabs(counts[i] / double(n) - 0.2) < 0.01,
                      std::string(kRows[i].clip) + " share " + std::to_string(counts[i] / double(n)));
        }
        Check(wrong == 0, std::string(st ? "crouch" : "stand") + ": no row of the other stance");
    }
    bool none[kRowCount] = {};
    Rng r0(1);
    Check(SelectRow(Stance::Stand, 0.0f, r0, none) == -1, "no available clip -> NoValidAnim (-1)");

    auto seq = [](uint64_t seed) {
        Rng rng(seed);
        std::vector<int> v;
        for (int i = 0; i < 64; ++i)
            v.push_back(SelectRow(i % 3 ? Stance::Stand : Stance::Crouch, (i % 5 - 2) * 60.0f, rng));
        return v;
    };
    Check(seq(7) == seq(7), "same seed -> same sequence");
    Check(seq(7) != seq(8), "different seed -> different sequence");
}

// ---- 4. controller
struct Sim
{
    Controller c;
    float t = 0.0f;
    float dt;
    Sim(const std::map<std::string, ClipData>& aClips, uint64_t aSeed, float aFps) : c(aSeed), dt(1.0f / aFps)
    {
        for (const auto& kv : aClips)
            c.SetClip(kv.first, kv.second);
    }
    Output Step(Inputs in)
    {
        in.dt = dt;
        t += dt;
        return c.Update(in);
    }
};

static void TestController(const std::map<std::string, ClipData>& aClips)
{
    for (float fps : {30.0f, 60.0f, 144.0f})
    {
        const std::string tag = " @" + std::to_string(int(fps)) + "fps";
        const float dt = 1.0f / fps;
        // timing of the first break, its end and the restart
        {
            Sim s(aClips, 42, fps);
            Inputs in;
            float tStart = -1.0f, tEnd = -1.0f, tNext = -1.0f;
            Output startOut, endOut;
            int firstRow = -1;
            float maxDao = 0.0f, daoAtStart = -1.0f;
            int badEvents = 0;
            while (s.t < 30.0f && tNext < 0.0f)
            {
                const Output o = s.Step(in);
                const float tBefore = s.t - dt; // time before this update = idle time at the rule check
                if (o.event == Event::StartBreak && tStart < 0.0f)
                {
                    tStart = tBefore;
                    startOut = o;
                    firstRow = o.row;
                    daoAtStart = o.disableAO;
                }
                else if (o.event == Event::EndBreak && tEnd < 0.0f)
                {
                    tEnd = tBefore;
                    endOut = o;
                }
                else if (o.event == Event::StartBreak && tEnd >= 0.0f)
                    tNext = tBefore;
                if (tStart >= 0.0f && tEnd < 0.0f)
                    maxDao = std::max(maxDao, o.disableAO);
                badEvents += (o.event == Event::Interrupt || o.event == Event::NoValidAnim) ? 1 : 0;
            }
            Check(badEvents == 0, "no Interrupt / NoValidAnim without input" + tag);
            Check(tStart > 3.0f - 1e-4f && tStart <= 3.0f + 2.0f * dt, "first break after > 3 s idle: " + std::to_string(tStart) + tag);
            Check(firstRow >= 0 && kRows[firstRow].stance == Stance::Stand, "stand break row" + tag);
            if (firstRow >= 0)
            {
                const Row& r = kRows[firstRow];
                Check(startOut.clip == r.clip && startOut.startTime == r.startTime && startOut.blendTime == 0.5f,
                      "StartBreak clip / StartTime / blend 0.5" + tag);
                const float dur = aClips.at(std::string(r.clip)).duration;
                const float expectEnd = tStart + (dur - 0.75f - r.startTime);
                Check(tEnd >= expectEnd - 1e-3f && tEnd <= expectEnd + 2.0f * dt + 1e-3f,
                      "EndBreak at <= 0.75 s remaining: " + std::to_string(tEnd) + " vs " + std::to_string(expectEnd) + tag);
                Check(endOut.clip == kIdleLoopClip[0] && endOut.blendTime == 0.75f, "EndBreak -> stand idle loop, blend 0.75" + tag);
                Check(tNext > tEnd + 3.0f - 1e-3f && tNext <= tEnd + 3.0f + 2.0f * dt + 1e-3f,
                      "next break 3 s after the end: " + std::to_string(tNext - tEnd) + tag);
                const bool hasCurve = !aClips.at(std::string(r.clip)).disableAO.empty();
                Check(daoAtStart <= 0.05f, "Disable_AO starts near 0 (blend-in weight)" + tag);
                Check(hasCurve ? maxDao > 0.9f : maxDao == 0.0f, "Disable_AO max " + std::to_string(maxDao) + tag);
            }
        }
        // no break while blocked (each flag alone, 12 s)
        const char* names[] = {"moving", "inputPressed", "aiming", "inAir", "turnInPlace", "transition"};
        for (int f = 0; f < 6; ++f)
        {
            Sim s(aClips, 3, fps);
            Inputs in;
            in.moving = f == 0;
            in.inputPressed = f == 1;
            in.aiming = f == 2;
            in.inAir = f == 3;
            in.turnInPlace = f == 4;
            in.idleLoopActive = f != 5;
            int starts = 0;
            while (s.t < 12.0f)
                starts += s.Step(in).event == Event::StartBreak ? 1 : 0;
            Check(starts == 0, std::string("no break while ") + names[f] + tag);
        }
        // input pulses every 2.9 s keep resetting the timer
        {
            Sim s(aClips, 3, fps);
            int starts = 0;
            float nextPulse = 2.9f;
            while (s.t < 20.0f)
            {
                Inputs in;
                if (s.t >= nextPulse)
                {
                    in.inputPressed = true;
                    nextPulse += 2.9f;
                }
                starts += s.Step(in).event == Event::StartBreak ? 1 : 0;
            }
            Check(starts == 0, "input every 2.9 s: no break" + tag);
        }
        // interruption on the update that sees the input (latency 0 <= 0.2 s), each blocker
        for (int f = 0; f < 4; ++f)
        {
            Sim s(aClips, 9, fps);
            Inputs idle;
            while (s.c.GetPhase() != Phase::Break && s.t < 10.0f)
                s.Step(idle);
            for (int i = 0; i < int(0.5f * fps); ++i)
                s.Step(idle);
            Inputs in;
            in.moving = f == 0;
            in.inputPressed = f == 1;
            in.aiming = f == 2;
            in.stance = f == 3 ? Stance::Crouch : Stance::Stand;
            const Output o = s.Step(in);
            Check(o.event == Event::Interrupt && o.phase == Phase::IdleLoop && o.blendTime == kDefaultInterruptBlendTime,
                  std::string("interrupt by ") + (f == 3 ? "stance change" : names[f]) + " on the same update" + tag);
            Check(o.breakWeight > 0.0f && o.breakWeight <= 1.0f, "break blends out after interrupt" + tag);
            Output last = o;
            for (int i = 0; i < int(fps); ++i)
                last = s.Step(in);
            Check(last.breakWeight == 0.0f && last.disableAO == 0.0f, "break weight 0 after the blend-out" + tag);
        }
        // crouch: only crouch rows, crouch idle loop on end
        {
            Sim s(aClips, 77, fps);
            Inputs in;
            in.stance = Stance::Crouch;
            int starts = 0, wrong = 0, ends = 0, endWrong = 0;
            while (s.t < 60.0f)
            {
                const Output o = s.Step(in);
                if (o.event == Event::StartBreak)
                {
                    ++starts;
                    wrong += kRows[o.row].stance != Stance::Crouch;
                }
                if (o.event == Event::EndBreak)
                {
                    ++ends;
                    endWrong += o.clip != kIdleLoopClip[1];
                }
            }
            Check(starts >= 6 && wrong == 0 && ends >= 5 && endWrong == 0,
                  "crouch: " + std::to_string(starts) + " breaks, only crouch rows, crouch idle loop" + tag);
        }
    }
    // deterministic controller sequence for a seed
    auto rows = [&](uint64_t seed) {
        Sim s(aClips, seed, 60.0f);
        std::vector<int> v;
        Inputs in;
        while (s.t < 120.0f)
            if (const Output o = s.Step(in); o.event == Event::StartBreak)
                v.push_back(o.row);
        return v;
    };
    const auto a = rows(2026), b = rows(2026), c = rows(2027);
    Check(a.size() >= 10 && a == b, "controller: same seed -> same break sequence (" + std::to_string(a.size()) + ")");
    Check(a != c, "controller: different seed -> different sequence");
    // NoValidAnim without loaded clips: retried every 3 s
    {
        Controller ctl(1);
        Inputs in;
        in.dt = 1.0f / 60.0f;
        int novalid = 0, starts = 0;
        for (int i = 0; i < 60 * 10; ++i)
        {
            const Output o = ctl.Update(in);
            novalid += o.event == Event::NoValidAnim;
            starts += o.event == Event::StartBreak;
        }
        Check(novalid == 3 && starts == 0, "no clips: NoValidAnim every 3 s (" + std::to_string(novalid) + ")");
    }
}

int main(int argc, char** argv)
{
    std::string rules;
    std::vector<std::string> clipDirs, dbs;
    for (int i = 1; i + 1 < argc; i += 2)
    {
        const std::string k = argv[i];
        if (k == "--rules")
            rules = argv[i + 1];
        else if (k == "--clips")
            clipDirs.emplace_back(argv[i + 1]);
        else if (k == "--db")
            dbs.emplace_back(argv[i + 1]);
    }
    Check(!rules.empty(), "--rules given");
    Check(!clipDirs.empty(), "--clips given");
    if (!rules.empty())
        TestRules(rules);
    const auto clips = TestClips(clipDirs);
    for (const auto& d : dbs)
        ListDb(d);
    TestSelection();
    if (clips.size() == 11)
        TestController(clips);
    else
        Check(false, "controller tests need the 11 clips of the first --clips dir");
    std::printf("utpp_mm_idlebreaks_test: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
