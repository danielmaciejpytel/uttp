// UTPP 0.5.2 - per-database speed calibration (game m/s -> database m/s points of the speed map).
// Header-only (no Windows / game dependency), shared with the offline tests.
//
// Until 0.5.1 the database side of the speed map was hard-coded for motion (walk 2.19, run 5.46, sprint 7.65, crouch 2.46
// m/s = forward loop root speeds). Each database now gets its own values on load:
//   1. measured: per class (Walk / Run / Sprint in the Stand_ clip set, Crouch in the Crouch_ set) the median of the mean
//      planar root speeds of LOOP clips named "..._<Class>_Loop_F" (end or '_' after F: motion M_Neutral_Run_Loop_F,
//      M_Neutral_Walk_Loop_F_L_20, female a non-package clip, a non-package clip); without forward loops any
//      "_<Class>_Loop_" loop (no Arc) of the class;
//   2. fallback: missing walk / run from the other one with the motion ratio; missing sprint = fastest forward stand loop
//      (the query never asks for a speed the database cannot play); missing crouch = default; nothing = motion defaults;
//   3. optional sidecar <database>.json (same folder) overrides: {"walk":1.75,"run":3.7,"sprint":5.2,"crouch":1.5,
//      "stand":[[1.6,1.75],[3.85,3.7],[8.2,5.2]],"crouchPoints":[[2.7,1.5]]} (every key optional; points = game, db).
// Game-side points stay fixed (2.31 measurements): stand/sprint 1.6 / 3.85 / 8.2, crouch 2.7. The walk point is also
// the slow-walk floor reference (SpeedPoints.db[0], see PlaybackRate.hpp).
#pragma once

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <string_view>
#include <vector>

#include "../BaseDatabase.hpp"
#include "PlaybackRate.hpp"

namespace utpp::mm
{
enum SpeedClass : int
{
    kSpeedWalk = 0,
    kSpeedRun = 1,
    kSpeedSprint = 2,
    kSpeedCrouch = 3,
    kSpeedClassCount = 4
};
inline constexpr const char* kSpeedClassName[kSpeedClassCount] = {"Walk", "Run", "Sprint", "Crouch"};
inline constexpr float kMotionLoopSpeed[kSpeedClassCount] = {2.19f, 5.46f, 7.65f, 2.46f}; // 0.5.1 hard-coded db side
inline constexpr float kGameStandPoints[3] = {1.6f, 3.85f, 8.2f};
inline constexpr float kGameCrouchPoint = 2.7f;

struct SpeedClassResult
{
    float speed = 0.0f;  // database m/s used
    int clips = 0;       // loop clips measured (0 = not measured)
    float minSpeed = 0.0f, maxSpeed = 0.0f;
    char source = 'd';   // 'm' measured forward loops, 'a' measured any-direction loops, 'f' fallback, 'o' sidecar, 'd' default
};

struct SpeedCalibration
{
    SpeedClassResult cls[kSpeedClassCount];
    SpeedPoints stand{3, {kGameStandPoints[0], kGameStandPoints[1], kGameStandPoints[2]},
                      {kMotionLoopSpeed[0], kMotionLoopSpeed[1], kMotionLoopSpeed[2]}};
    SpeedPoints crouch{1, {kGameCrouchPoint, 0, 0}, {kMotionLoopSpeed[3], 0, 0}};
    bool sidecar = false;     // sidecar applied
    bool standPointsOverride = false, crouchPointsOverride = false;
    std::string sidecarError; // non-empty: sidecar present but rejected (calibration from the database kept)

    std::string Summary() const
    {
        char buf[512];
        int n = std::snprintf(buf, sizeof buf, "walk %.3f(%c%d) run %.3f(%c%d) sprint %.3f(%c%d) crouch %.3f(%c%d) | stand",
                              double(cls[0].speed), cls[0].source, cls[0].clips, double(cls[1].speed), cls[1].source,
                              cls[1].clips, double(cls[2].speed), cls[2].source, cls[2].clips, double(cls[3].speed),
                              cls[3].source, cls[3].clips);
        for (int i = 0; i < stand.count && n > 0 && n < int(sizeof buf); ++i)
            n += std::snprintf(buf + n, sizeof buf - size_t(n), " %.2f>%.3f", double(stand.game[i]), double(stand.db[i]));
        if (n > 0 && n < int(sizeof buf))
            n += std::snprintf(buf + n, sizeof buf - size_t(n), " crouch");
        for (int i = 0; i < crouch.count && n > 0 && n < int(sizeof buf); ++i)
            n += std::snprintf(buf + n, sizeof buf - size_t(n), " %.2f>%.3f", double(crouch.game[i]), double(crouch.db[i]));
        std::string s = buf;
        if (sidecar)
            s += " | sidecar";
        if (!sidecarError.empty())
            s += " | sidecar REJECTED: " + sidecarError;
        return s;
    }
};

// Mean planar root speed (database m/s) of a clip; loop-duplicate frames skipped. 0 if no root data.
inline float ClipMeanRootSpeed(const Database& aDb, uint32_t aClip)
{
    const auto clips = aDb.Clips();
    if (aClip >= clips.size() || aDb.Fps() <= 0.0f)
        return 0.0f;
    const ClipRecord& c = clips[aClip];
    double sum = 0.0;
    uint32_t n = 0;
    for (uint32_t f = c.firstFrame; f < c.firstFrame + c.numFrames; ++f)
    {
        if (aDb.FrameFlagsOf(f) & kFrameLoopDuplicate)
            continue;
        const RootDelta* r = aDb.Root(f);
        if (!r)
            continue;
        sum += std::sqrt(double(r->dx) * r->dx + double(r->dy) * r->dy);
        ++n;
    }
    return n ? float(sum / n * aDb.Fps()) : 0.0f;
}

// "_<cls>_Loop_" in the name; aForwardOnly: followed by 'F' and then end or '_' (excludes FL / FR / Arc).
inline bool IsClassLoopName(std::string_view aName, std::string_view aCls, bool aForwardOnly)
{
    const std::string key = "_" + std::string(aCls) + "_Loop_";
    size_t p = aName.find(key);
    while (p != std::string_view::npos)
    {
        const size_t q = p + key.size();
        if (!aForwardOnly)
            return true;
        if (q < aName.size() && aName[q] == 'F' && (q + 1 == aName.size() || aName[q + 1] == '_'))
            return true;
        p = aName.find(key, p + 1);
    }
    return false;
}

inline float Median(std::vector<float> v)
{
    if (v.empty())
        return 0.0f;
    std::sort(v.begin(), v.end());
    const size_t m = v.size() / 2;
    return (v.size() & 1) ? v[m] : 0.5f * (v[m - 1] + v[m]);
}

// Rebuilds the maps from cls[] (keeps sidecar point overrides) with strictly increasing database speeds.
inline void BuildCalibrationMaps(SpeedCalibration& c)
{
    if (!c.standPointsOverride)
    {
        c.stand.count = 3;
        for (int i = 0; i < 3; ++i)
        {
            c.stand.game[i] = kGameStandPoints[i];
            c.stand.db[i] = c.cls[i].speed;
        }
        for (int i = 1; i < 3; ++i)
            c.stand.db[i] = std::max(c.stand.db[i], c.stand.db[i - 1] + 0.05f);
    }
    if (!c.crouchPointsOverride)
    {
        c.crouch.count = 1;
        c.crouch.game[0] = kGameCrouchPoint;
        c.crouch.db[0] = c.cls[kSpeedCrouch].speed;
    }
}

// aStandMask / aCrouchMask: clip tag masks of the stand and crouch sets (driver state masks).
inline SpeedCalibration DeriveSpeedCalibration(const Database& aDb, uint64_t aStandMask, uint64_t aCrouchMask)
{
    SpeedCalibration c;
    const auto clips = aDb.Clips();
    for (int k = 0; k < kSpeedClassCount; ++k)
    {
        const uint64_t mask = k == kSpeedCrouch ? aCrouchMask : aStandMask;
        for (int pass = 0; pass < 2 && c.cls[k].clips == 0; ++pass)
        {
            std::vector<float> speeds;
            for (uint32_t i = 0; i < clips.size(); ++i)
            {
                const ClipRecord& r = clips[i];
                const std::string_view name = aDb.ClipName(i);
                if (!(r.flags & kClipLoop) || !(r.tagMask & mask) || name.find("_Arc_") != std::string_view::npos ||
                    !IsClassLoopName(name, kSpeedClassName[k], pass == 0))
                    continue;
                const float s = ClipMeanRootSpeed(aDb, i);
                if (s > 0.05f)
                    speeds.push_back(s);
            }
            if (!speeds.empty())
            {
                c.cls[k].speed = Median(speeds);
                c.cls[k].clips = int(speeds.size());
                c.cls[k].minSpeed = *std::min_element(speeds.begin(), speeds.end());
                c.cls[k].maxSpeed = *std::max_element(speeds.begin(), speeds.end());
                c.cls[k].source = pass == 0 ? 'm' : 'a';
            }
        }
    }
    SpeedClassResult& w = c.cls[kSpeedWalk];
    SpeedClassResult& r = c.cls[kSpeedRun];
    SpeedClassResult& s = c.cls[kSpeedSprint];
    SpeedClassResult& cr = c.cls[kSpeedCrouch];
    if (w.clips == 0 && r.clips > 0)
    {
        w.speed = r.speed * kMotionLoopSpeed[0] / kMotionLoopSpeed[1];
        w.source = 'f';
    }
    if (r.clips == 0 && w.clips > 0)
    {
        r.speed = w.speed * kMotionLoopSpeed[1] / kMotionLoopSpeed[0];
        r.source = 'f';
    }
    if (w.clips == 0 && r.clips == 0)
    {
        w.speed = kMotionLoopSpeed[0];
        r.speed = kMotionLoopSpeed[1];
    }
    if (s.clips == 0)
    {
        if (w.clips > 0 || r.clips > 0)
        {
            s.speed = std::max(w.clips ? w.maxSpeed : 0.0f, r.clips ? r.maxSpeed : 0.0f);
            s.speed = std::max(s.speed, r.speed);
            s.source = 'f';
        }
        else
            s.speed = kMotionLoopSpeed[2];
    }
    if (cr.clips == 0)
        cr.speed = kMotionLoopSpeed[3];
    BuildCalibrationMaps(c);
    return c;
}

// ---- minimal JSON reader for the sidecar (objects, arrays, numbers, strings, true/false/null)
namespace json
{
struct Value
{
    enum Type { Null, Bool, Number, String, Array, Object } type = Null;
    double number = 0.0;
    bool boolean = false;
    std::string str;
    std::vector<Value> items;                            // Array
    std::vector<std::pair<std::string, Value>> members;  // Object
    const Value* Find(std::string_view aKey) const
    {
        for (const auto& m : members)
            if (m.first == aKey)
                return &m.second;
        return nullptr;
    }
};

class Parser
{
public:
    explicit Parser(std::string_view aText) : m_s(aText) {}
    bool Parse(Value& aOut, std::string& aErr)
    {
        if (!ParseValue(aOut, 0))
        {
            aErr = m_err.empty() ? "syntax error" : m_err;
            return false;
        }
        Skip();
        if (m_p != m_s.size())
        {
            aErr = "trailing characters";
            return false;
        }
        return true;
    }

private:
    void Skip()
    {
        while (m_p < m_s.size() && (m_s[m_p] == ' ' || m_s[m_p] == '\t' || m_s[m_p] == '\r' || m_s[m_p] == '\n'))
            ++m_p;
    }
    bool Lit(std::string_view aLit)
    {
        if (m_s.substr(m_p, aLit.size()) != aLit)
            return false;
        m_p += aLit.size();
        return true;
    }
    bool ParseString(std::string& aOut)
    {
        if (m_p >= m_s.size() || m_s[m_p] != '"')
            return false;
        ++m_p;
        while (m_p < m_s.size() && m_s[m_p] != '"')
        {
            char ch = m_s[m_p++];
            if (ch == '\\' && m_p < m_s.size())
            {
                const char e = m_s[m_p++];
                ch = e == 'n' ? '\n' : e == 't' ? '\t' : e;
            }
            aOut.push_back(ch);
        }
        if (m_p >= m_s.size())
            return false;
        ++m_p;
        return true;
    }
    bool ParseValue(Value& v, int aDepth)
    {
        if (aDepth > 16)
        {
            m_err = "nesting too deep";
            return false;
        }
        Skip();
        if (m_p >= m_s.size())
            return false;
        const char ch = m_s[m_p];
        if (ch == '{')
        {
            v.type = Value::Object;
            ++m_p;
            Skip();
            if (m_p < m_s.size() && m_s[m_p] == '}')
                return ++m_p, true;
            for (;;)
            {
                Skip();
                std::string key;
                if (!ParseString(key))
                    return false;
                Skip();
                if (m_p >= m_s.size() || m_s[m_p] != ':')
                    return false;
                ++m_p;
                Value item;
                if (!ParseValue(item, aDepth + 1))
                    return false;
                v.members.emplace_back(std::move(key), std::move(item));
                Skip();
                if (m_p < m_s.size() && m_s[m_p] == ',')
                {
                    ++m_p;
                    continue;
                }
                if (m_p < m_s.size() && m_s[m_p] == '}')
                    return ++m_p, true;
                return false;
            }
        }
        if (ch == '[')
        {
            v.type = Value::Array;
            ++m_p;
            Skip();
            if (m_p < m_s.size() && m_s[m_p] == ']')
                return ++m_p, true;
            for (;;)
            {
                Value item;
                if (!ParseValue(item, aDepth + 1))
                    return false;
                v.items.push_back(std::move(item));
                Skip();
                if (m_p < m_s.size() && m_s[m_p] == ',')
                {
                    ++m_p;
                    continue;
                }
                if (m_p < m_s.size() && m_s[m_p] == ']')
                    return ++m_p, true;
                return false;
            }
        }
        if (ch == '"')
        {
            v.type = Value::String;
            return ParseString(v.str);
        }
        if (Lit("true"))
            return v.type = Value::Bool, v.boolean = true, true;
        if (Lit("false"))
            return v.type = Value::Bool, v.boolean = false, true;
        if (Lit("null"))
            return v.type = Value::Null, true;
        const size_t start = m_p;
        while (m_p < m_s.size() && (std::isdigit(static_cast<unsigned char>(m_s[m_p])) || m_s[m_p] == '-' || m_s[m_p] == '+' ||
                                    m_s[m_p] == '.' || m_s[m_p] == 'e' || m_s[m_p] == 'E'))
            ++m_p;
        if (m_p == start)
            return false;
        const std::string num(m_s.substr(start, m_p - start));
        char* end = nullptr;
        v.type = Value::Number;
        v.number = std::strtod(num.c_str(), &end);
        return end && *end == '\0' && std::isfinite(v.number);
    }

    std::string_view m_s;
    size_t m_p = 0;
    std::string m_err;
};
} // namespace json

// Points array [[game, db], ...] (1..3, increasing, 0.1..20 m/s). False + error on invalid input.
inline bool ReadPoints(const json::Value& aV, int aMax, SpeedPoints& aOut, std::string& aErr)
{
    if (aV.type != json::Value::Array || aV.items.empty() || int(aV.items.size()) > aMax)
        return aErr = "points must be an array of 1.." + std::to_string(aMax) + " [game, db] pairs", false;
    SpeedPoints p{0, {0, 0, 0}, {0, 0, 0}};
    for (const json::Value& it : aV.items)
    {
        if (it.type != json::Value::Array || it.items.size() != 2 || it.items[0].type != json::Value::Number ||
            it.items[1].type != json::Value::Number)
            return aErr = "point must be [game, db]", false;
        const float g = float(it.items[0].number), d = float(it.items[1].number);
        if (!(g >= 0.1f && g <= 20.0f && d >= 0.1f && d <= 20.0f))
            return aErr = "point speed out of range 0.1..20", false;
        if (p.count > 0 && !(g > p.game[p.count - 1] && d > p.db[p.count - 1]))
            return aErr = "points must increase", false;
        p.game[p.count] = g;
        p.db[p.count] = d;
        ++p.count;
    }
    aOut = p;
    return true;
}

// Applies a sidecar text. On any error nothing is changed and aCal.sidecarError is set.
inline bool ApplySpeedSidecar(SpeedCalibration& aCal, std::string_view aText)
{
    json::Value root;
    std::string err;
    if (!json::Parser(aText).Parse(root, err) || root.type != json::Value::Object)
    {
        aCal.sidecarError = err.empty() ? "not a JSON object" : err;
        return false;
    }
    SpeedCalibration c = aCal;
    static constexpr const char* kKeys[kSpeedClassCount] = {"walk", "run", "sprint", "crouch"};
    for (int k = 0; k < kSpeedClassCount; ++k)
    {
        const json::Value* v = root.Find(kKeys[k]);
        if (!v)
            continue;
        if (v->type != json::Value::Number || !(v->number >= 0.1 && v->number <= 20.0))
        {
            aCal.sidecarError = std::string(kKeys[k]) + " must be a number 0.1..20";
            return false;
        }
        c.cls[k].speed = float(v->number);
        c.cls[k].source = 'o';
    }
    if (const json::Value* v = root.Find("stand"))
    {
        if (!ReadPoints(*v, 3, c.stand, err))
            return aCal.sidecarError = "stand: " + err, false;
        c.standPointsOverride = true;
    }
    if (const json::Value* v = root.Find("crouchPoints"))
    {
        if (!ReadPoints(*v, 3, c.crouch, err))
            return aCal.sidecarError = "crouchPoints: " + err, false;
        c.crouchPointsOverride = true;
    }
    BuildCalibrationMaps(c);
    c.sidecar = true;
    c.sidecarError.clear();
    aCal = c;
    return true;
}
// ---- 0.5.3: per-clip rate factors and the slow-walk reference ------------------------------------------------------
// Each clip gets a playback-rate factor = calibrated class speed / the forward-loop speed of its class; forward loops use their
// own measured speed (class speed / loop speed). Every package clip belongs to the one family "motion", so the factors are ~1
// everywhere (0.5.2 behaviour, checked by utpp_mm_speedcal_test). Factors are clamped to [0.5, 2].

inline std::string ClipFamily(std::string_view)
{
    return "Neutral";
}

// Speed class of a clip for the rate factor (-1 = none: idle, misc, jumps).
inline int SpeedClassOfClip(std::string_view aName)
{
    auto has = [&](std::string_view s) { return aName.find(s) != std::string_view::npos; };
    if (has("Idle") || has("Jump") || has("Land") || has("Misc"))
        return -1;
    if (has("Crouch_"))
        return kSpeedCrouch;
    if (has("Sprint_"))
        return kSpeedSprint;
    if (has("Run_"))
        return kSpeedRun;
    if (has("Walk_"))
        return kSpeedWalk;
    return -1;
}

struct FamilyRates
{
    std::vector<float> clipFactor;  // per clip
    std::vector<std::string> names; // families
    std::vector<std::array<float, kSpeedClassCount>> ref; // forward loop speed per family / class (0 = none)
    int families = 0;
    std::string Summary() const
    {
        std::string out;
        for (size_t f = 0; f < names.size(); ++f)
        {
            char buf[192];
            std::snprintf(buf, sizeof buf, "%s%s w%.2f r%.2f s%.2f c%.2f", out.empty() ? "" : " ", names[f].c_str(), double(ref[f][0]),
                          double(ref[f][1]), double(ref[f][2]), double(ref[f][3]));
            out += buf;
        }
        return out;
    }
};

inline FamilyRates ComputeFamilyRates(const Database& aDb, uint64_t aStandMask, uint64_t aCrouchMask, const SpeedCalibration& aCal)
{
    FamilyRates fr;
    const auto clips = aDb.Clips();
    fr.clipFactor.assign(clips.size(), 1.0f);
    std::vector<int> famOf(clips.size(), 0);
    for (uint32_t i = 0; i < clips.size(); ++i)
    {
        const std::string fam = ClipFamily(aDb.ClipName(i));
        auto it = std::find(fr.names.begin(), fr.names.end(), fam);
        if (it == fr.names.end())
        {
            fr.names.push_back(fam);
            it = fr.names.end() - 1;
        }
        famOf[i] = int(it - fr.names.begin());
    }
    fr.families = int(fr.names.size());
    // forward loop speeds per family and class
    std::vector<std::array<std::vector<float>, kSpeedClassCount>> speeds(fr.names.size());
    std::vector<float> ownSpeed(clips.size(), 0.0f); // forward loops: own speed (per-clip factor)
    std::vector<int> ownClass(clips.size(), -1);
    for (uint32_t i = 0; i < clips.size(); ++i)
    {
        const ClipRecord& r = clips[i];
        const std::string_view name = aDb.ClipName(i);
        if (name.find("_Arc_") != std::string_view::npos)
            continue;
        if (!(r.flags & kClipLoop))
            continue;
        for (int k = 0; k < kSpeedClassCount; ++k)
        {
            const uint64_t mask = k == kSpeedCrouch ? aCrouchMask : aStandMask;
            if (!(r.tagMask & mask) || !IsClassLoopName(name, kSpeedClassName[k], true))
                continue;
            const float sp = ClipMeanRootSpeed(aDb, i);
            if (sp <= 0.05f)
                continue;
            speeds[size_t(famOf[i])][size_t(k)].push_back(sp);
            ownSpeed[i] = sp;
            ownClass[i] = k;
        }
    }
    fr.ref.assign(fr.names.size(), {0.0f, 0.0f, 0.0f, 0.0f});
    for (size_t f = 0; f < fr.names.size(); ++f)
        for (int k = 0; k < kSpeedClassCount; ++k)
        {
            fr.ref[f][size_t(k)] = Median(speeds[f][size_t(k)]);
        }
    // motion alone: every forward loop of a class has (almost) the class speed and the family median is the class median, so
    // all factors are 1 within 0.1 % (0.5.2 behaviour; checked by utpp_mm_speedcal_test).
    for (uint32_t i = 0; i < clips.size(); ++i)
    {
        if (ownClass[i] >= 0) // forward loop: its own speed (plays at the capsule speed)
        {
            const float cls = aCal.cls[ownClass[i]].speed;
            if (cls > 0.0f && ownSpeed[i] > 0.3f)
                fr.clipFactor[i] = std::clamp(cls / ownSpeed[i], 0.5f, 2.0f);
            continue;
        }
        const int k = SpeedClassOfClip(aDb.ClipName(i));
        if (k < 0)
            continue;
        const size_t f = size_t(famOf[i]);
        const float ref = fr.ref[f][size_t(k)];
        if (ref > 0.3f && aCal.cls[k].speed > 0.0f)
            fr.clipFactor[i] = std::clamp(aCal.cls[k].speed / ref, 0.5f, 2.0f);
    }
    return fr;
}

// Slowest forward loop of a class (walk / crouch): the slow-walk gate asks for it, so a database with slow walk loops
// plays them near their natural speed (a former test database: 1.29 m/s FemaleWalk instead of the 1.97 median).
// 0 = no measurement (driver keeps the map point).
inline float SlowestForwardLoop(const SpeedCalibration& aCal, int aClass)
{
    const SpeedClassResult& c = aCal.cls[aClass];
    return (c.source == 'm' && c.clips > 0) ? c.minSpeed : 0.0f;
}
} // namespace utpp::mm
