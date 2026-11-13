// UTPP - traversal runtime core (motion report 3.14, stage 10.8). Header-only, no game hooks.
//
// 1) DecisionTable: data-driven evaluation of the normalized traversal chooser table written by
//    tools/mmbake/export_traversal_table.py (Data/traversal/traversal_table.json, motion-derived).
//    Same semantics as tools/mmbake/traversal_ref.py (UE Chooser: rows in order, filters narrow, output columns
//    written for every surviving row before its result, last write wins; AllResults; FMultiEnumColumn bit test,
//    closed float ranges with float32 bounds compared in double) plus the CBP gate (no front ledge / ActionType
//    none -> CheckFailed, no candidate montage -> MontageFailed).
// 2) Warp solving in the style of UE MotionWarping (RootMotionModifier_SkewWarp.cpp / RootMotionModifier.cpp
//    rotation): inside a window the remaining animated root translation is skewed so that the warp point (root
//    plus an attach offset given in root space) lands on the world target at the window end; yaw is blended
//    toward the target yaw with the window time fraction. Frames outside windows keep the animated deltas.
// 3) CP obstacle classes (ClimbEvents: High if h > highThreshold, Mid if h > midThreshold, else Low;
//    locomotionTransitions.script ClimbEvents.GetClimbParameter) and the vertical scale that maps a montage's
//    animated ledge height onto the real obstacle height. Thresholds are inputs (game data), never constants here.
#pragma once

#include "MiniJson.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace utpp::mm::traversal
{
// ------------------------------------------------------------------------------------------------ decision table
enum class Status : uint8_t
{
    Ok,
    CheckFailed,
    MontageFailed
};

struct Inputs
{
    bool hasFrontLedge = false;
    bool hasBackLedge = false;
    bool hasBackFloor = false;
    double obstacleHeight = 0.0; // UE units (cm), as in the motion struct
    double obstacleDepth = 0.0;
    double backLedgeHeight = 0.0;
    double speed = 0.0;
    int movementMode = 0; // engine EMovementMode value
};

struct Decision
{
    Status status = Status::CheckFailed;
    int action = 0;
    std::string actionName;
    std::vector<std::string> montages; // row order (AllResults)
};

class DecisionTable
{
public:
    enum class Kind : uint8_t
    {
        Bool,
        Float,
        MultiEnum,
        OutputEnum
    };
    enum class Input : uint8_t
    {
        HasFrontLedge,
        HasBackLedge,
        HasBackFloor,
        ObstacleHeight,
        ObstacleDepth,
        BackLedgeHeight,
        Speed,
        MovementMode,
        Unknown
    };
    struct FloatCell
    {
        double mn = 0, mx = 0;
        bool noMin = false, noMax = false;
    };
    struct Column
    {
        Kind kind = Kind::Bool;
        Input input = Input::Unknown;
        std::vector<int8_t> boolCells; // -1 any, 0 false, 1 true
        std::vector<FloatCell> floatCells;
        std::vector<uint32_t> intCells; // multienum mask / output enum value
    };
    struct Result
    {
        int type = 0; // 0 none, 1 asset, 2 nested
        int table = -1;
        std::string asset;
    };
    struct Table
    {
        std::string name;
        std::vector<Column> columns;
        std::vector<Result> rows;
        bool hasFallback = false;
        Result fallback;
    };

    bool Load(std::string_view aJson, std::string& aError)
    {
        json::Value v;
        if (!json::Parser::Parse(aJson, v, aError))
            return false;
        const json::Value* fmt = v.Find("format");
        const json::Value* ver = v.Find("version");
        if (!fmt || fmt->str != "utpp_traversal_table" || !ver || ver->num != 1.0)
            return Fail(aError, "not a utpp_traversal_table base");
        const json::Value* root = v.Find("root");
        const json::Value* tables = v.Find("tables");
        const json::Value* actEnum = v.Find("action_enum");
        const json::Value* none = v.Find("action_none");
        if (!root || !tables || !tables->IsArray() || !actEnum || !actEnum->IsArray() || !none)
            return Fail(aError, "missing root/tables/action_enum/action_none");
        m_root = static_cast<int>(root->num);
        m_actionNone = static_cast<int>(none->num);
        for (const auto& e : actEnum->arr)
        {
            const json::Value* ev = e.Find("value");
            const json::Value* en = e.Find("name");
            if (!ev || !en)
                return Fail(aError, "bad action_enum entry");
            m_actionNames.emplace_back(static_cast<int>(ev->num), en->str);
        }
        for (const auto& t : tables->arr)
        {
            Table tb;
            if (const json::Value* n = t.Find("name"))
                tb.name = n->str;
            const json::Value* cols = t.Find("columns");
            const json::Value* rows = t.Find("rows");
            if (!cols || !rows)
                return Fail(aError, "table without columns/rows");
            for (const auto& c : cols->arr)
            {
                Column col;
                const json::Value* kind = c.Find("kind");
                const json::Value* cells = c.Find("cells");
                if (!kind || !cells)
                    return Fail(aError, "bad column");
                if (const json::Value* in = c.Find("input"))
                    col.input = InputOf(in->str);
                if (kind->str == "bool")
                {
                    col.kind = Kind::Bool;
                    for (const auto& x : cells->arr)
                        col.boolCells.push_back(x.str == "any" ? int8_t(-1) : (x.str == "true" ? int8_t(1) : int8_t(0)));
                }
                else if (kind->str == "float")
                {
                    col.kind = Kind::Float;
                    for (const auto& x : cells->arr)
                    {
                        if (!x.IsArray() || x.arr.size() != 4)
                            return Fail(aError, "bad float cell");
                        col.floatCells.push_back({x.arr[0].num, x.arr[1].num, x.arr[2].b, x.arr[3].b});
                    }
                }
                else if (kind->str == "multienum" || kind->str == "output_enum")
                {
                    col.kind = kind->str == "multienum" ? Kind::MultiEnum : Kind::OutputEnum;
                    for (const auto& x : cells->arr)
                        col.intCells.push_back(static_cast<uint32_t>(x.num));
                }
                else
                    return Fail(aError, "unknown column kind " + kind->str);
                if (col.kind != Kind::OutputEnum && col.input == Input::Unknown)
                    return Fail(aError, "unknown input in table " + tb.name);
                tb.columns.push_back(std::move(col));
            }
            for (const auto& r : rows->arr)
            {
                const json::Value* res = r.Find("result");
                if (!res)
                    return Fail(aError, "row without result");
                tb.rows.push_back(ResultOf(*res));
            }
            if (const json::Value* fb = t.Find("fallback"); fb && fb->IsObject())
            {
                tb.hasFallback = true;
                tb.fallback = ResultOf(*fb);
            }
            m_tables.push_back(std::move(tb));
        }
        if (m_root < 0 || m_root >= static_cast<int>(m_tables.size()))
            return Fail(aError, "bad root index");
        for (const auto& tb : m_tables)
            for (const auto& r : tb.rows)
                if (r.type == 2 && (r.table < 0 || r.table >= static_cast<int>(m_tables.size())))
                    return Fail(aError, "bad nested table index");
        return true;
    }

    Decision Evaluate(const Inputs& aIn) const
    {
        Decision d;
        d.action = m_actionNone;
        if (!aIn.hasFrontLedge)
        {
            d.status = Status::CheckFailed;
            d.actionName = ActionName(d.action);
            return d;
        }
        int action = m_actionNone;
        EvalTable(m_root, aIn, action, d.montages, 0);
        d.action = action;
        d.actionName = ActionName(action);
        if (action == m_actionNone)
        {
            d.status = Status::CheckFailed;
            d.montages.clear();
        }
        else
            d.status = d.montages.empty() ? Status::MontageFailed : Status::Ok;
        return d;
    }

    std::string ActionName(int aValue) const
    {
        for (const auto& [v, n] : m_actionNames)
            if (v == aValue)
                return n;
        return std::to_string(aValue);
    }

private:
    static bool Fail(std::string& aErr, std::string aMsg)
    {
        aErr = std::move(aMsg);
        return false;
    }
    static Input InputOf(const std::string& s)
    {
        static const char* kNames[] = {"HasFrontLedge", "HasBackLedge",    "HasBackFloor", "ObstacleHeight",
                                       "ObstacleDepth", "BackLedgeHeight", "Speed",        "MovementMode"};
        for (int i = 0; i < 8; ++i)
            if (s == kNames[i])
                return static_cast<Input>(i);
        return Input::Unknown;
    }
    static Result ResultOf(const json::Value& r)
    {
        Result out;
        const json::Value* type = r.Find("type");
        if (!type)
            return out;
        if (type->str == "asset")
        {
            out.type = 1;
            if (const json::Value* a = r.Find("asset"))
                out.asset = a->str;
        }
        else if (type->str == "nested")
        {
            out.type = 2;
            if (const json::Value* t = r.Find("table"))
                out.table = static_cast<int>(t->num);
        }
        return out;
    }
    static double NumberOf(const Inputs& in, Input k)
    {
        switch (k)
        {
        case Input::ObstacleHeight: return in.obstacleHeight;
        case Input::ObstacleDepth: return in.obstacleDepth;
        case Input::BackLedgeHeight: return in.backLedgeHeight;
        case Input::Speed: return in.speed;
        case Input::MovementMode: return in.movementMode;
        case Input::HasFrontLedge: return in.hasFrontLedge ? 1.0 : 0.0;
        case Input::HasBackLedge: return in.hasBackLedge ? 1.0 : 0.0;
        case Input::HasBackFloor: return in.hasBackFloor ? 1.0 : 0.0;
        default: return 0.0;
        }
    }
    static bool Pass(const Column& c, size_t i, const Inputs& in)
    {
        switch (c.kind)
        {
        case Kind::Bool:
        {
            if (i >= c.boolCells.size())
                return false;
            const int8_t cell = c.boolCells[i];
            return cell < 0 || (NumberOf(in, c.input) != 0.0) == (cell == 1);
        }
        case Kind::Float:
        {
            if (i >= c.floatCells.size())
                return false;
            const FloatCell& f = c.floatCells[i];
            const double x = NumberOf(in, c.input);
            return (f.noMin || x >= f.mn) && (f.noMax || x <= f.mx);
        }
        case Kind::MultiEnum:
        {
            if (i >= c.intCells.size())
                return false;
            const uint32_t bit = 1u << (static_cast<uint32_t>(in.movementMode) & 31u);
            return c.intCells[i] == 0 || (c.intCells[i] & bit) != 0;
        }
        default: return true;
        }
    }
    bool EvalResult(const Result& r, const Inputs& in, int& action, std::vector<std::string>& out, int depth) const
    {
        if (r.type == 1)
        {
            out.push_back(r.asset);
            return true;
        }
        if (r.type == 2)
            return EvalTable(r.table, in, action, out, depth + 1);
        return false;
    }
    bool EvalTable(int ti, const Inputs& in, int& action, std::vector<std::string>& out, int depth) const
    {
        if (depth > 16)
            return false;
        const Table& t = m_tables[static_cast<size_t>(ti)];
        bool anyOk = false;
        for (size_t i = 0; i < t.rows.size(); ++i)
        {
            bool pass = true;
            for (const Column& c : t.columns)
                if (c.kind != Kind::OutputEnum && !Pass(c, i, in))
                {
                    pass = false;
                    break;
                }
            if (!pass)
                continue;
            for (const Column& c : t.columns) // outputs before the row result (Chooser.cpp:716-737)
                if (c.kind == Kind::OutputEnum && i < c.intCells.size())
                    action = static_cast<int>(c.intCells[i]);
            if (EvalResult(t.rows[i], in, action, out, depth))
                anyOk = true;
        }
        if (!anyOk && t.hasFallback)
            anyOk = EvalResult(t.fallback, in, action, out, depth);
        return anyOk;
    }

    std::vector<Table> m_tables;
    std::vector<std::pair<int, std::string>> m_actionNames;
    int m_root = 0;
    int m_actionNone = 0;
};

// ------------------------------------------------------------------------------------------------ warping
struct V3
{
    double x = 0, y = 0, z = 0;
};
inline V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline V3 operator*(V3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline double Len(V3 a) { return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z); }
inline V3 RotZ(V3 v, double yaw) // yaw in radians, counter-clockwise around +Z
{
    const double c = std::cos(yaw), s = std::sin(yaw);
    return {c * v.x - s * v.y, s * v.x + c * v.y, v.z};
}
inline double WrapPi(double a)
{
    const double twoPi = 6.283185307179586;
    a = std::fmod(a + 3.141592653589793, twoPi);
    if (a < 0)
        a += twoPi;
    return a - 3.141592653589793;
}

// Root sample of the clip in clip space (start of clip = origin, yaw 0 facing +X).
struct RootKey
{
    V3 pos;
    double yaw = 0;
};

// One warp window (notify state): frames [startFrame, endFrame], target in world space. warpPointOffset is the
// warp bone (e.g. attach / hand) relative to the root at the window end frame, in root space (from the bake);
// zero = warp the root itself. warpTranslation / warpYaw mirror bWarpTranslation / bWarpRotation; ignoreZ mirrors
// bIgnoreZAxis.
struct WarpWindow
{
    int startFrame = 0;
    int endFrame = 0;
    V3 target;
    double targetYaw = 0;
    V3 warpPointOffset;
    bool warpTranslation = true;
    bool warpYaw = true;
    bool ignoreZ = false;
};

// Produces world root keys for every clip frame. aStart is the world root at frame 0. Windows must be sorted and
// non-overlapping. Outside windows the animated per-frame delta (rotated by the current warped yaw) is applied.
// Inside a window, for each frame the remaining animated translation to the window end is scaled so the warp point
// reaches the target at the end frame (SkewWarp idea: the correction is distributed in proportion to the animated
// path still left, falling back to the time fraction when the clip does not move), and yaw moves linearly in
// time toward the target yaw (SimpleWarp style).
inline std::vector<RootKey> SolveWarp(const std::vector<RootKey>& aClip, const RootKey& aStart,
                                      const std::vector<WarpWindow>& aWindows)
{
    std::vector<RootKey> out(aClip.size());
    if (aClip.empty())
        return out;
    out[0] = aStart;
    // animated delta of frame f-1 -> f in the root space of frame f-1
    auto localDelta = [&](size_t f) {
        return RotZ(aClip[f].pos - aClip[f - 1].pos, -aClip[f - 1].yaw);
    };
    size_t w = 0;
    for (size_t f = 1; f < aClip.size(); ++f)
    {
        while (w < aWindows.size() && static_cast<int>(f) > aWindows[w].endFrame)
            ++w;
        const RootKey& prev = out[f - 1];
        const double dYaw = WrapPi(aClip[f].yaw - aClip[f - 1].yaw);
        const V3 animStep = RotZ(localDelta(f), prev.yaw);
        if (w < aWindows.size() && static_cast<int>(f) > aWindows[w].startFrame &&
            static_cast<int>(f) <= aWindows[w].endFrame)
        {
            const WarpWindow& win = aWindows[w];
            const size_t e = static_cast<size_t>(win.endFrame);
            // yaw
            double yaw = prev.yaw + dYaw;
            if (win.warpYaw)
            {
                // remaining animated yaw to the end, then close the gap linearly over the remaining frames
                const double animRemain = WrapPi(aClip[e].yaw - aClip[f - 1].yaw);
                const double gap = WrapPi(win.targetYaw - (prev.yaw + animRemain));
                const double frac = 1.0 / static_cast<double>(e - (f - 1));
                yaw = prev.yaw + dYaw + gap * frac;
            }
            // remaining animated translation from f-1 to e in world space (current yaw frame of reference)
            V3 remain{};
            double remainLen = 0;
            for (size_t k = f; k <= e; ++k)
            {
                const V3 s = RotZ(localDelta(k), prev.yaw + WrapPi(aClip[k - 1].yaw - aClip[f - 1].yaw));
                remain = remain + s;
                remainLen += Len(s);
            }
            V3 step = animStep;
            if (win.warpTranslation)
            {
                const double yawEnd = win.warpYaw ? win.targetYaw : prev.yaw + WrapPi(aClip[e].yaw - aClip[f - 1].yaw);
                const V3 rootTarget = win.target - RotZ(win.warpPointOffset, yawEnd);
                V3 err = rootTarget - (prev.pos + remain);
                if (win.ignoreZ)
                    err.z = 0;
                const double share = remainLen > 1e-9 ? Len(animStep) / remainLen
                                                      : 1.0 / static_cast<double>(e - (f - 1));
                step = animStep + err * share;
            }
            out[f] = {prev.pos + step, yaw};
        }
        else
            out[f] = {prev.pos + animStep, prev.yaw + dYaw};
    }
    return out;
}

// ------------------------------------------------------------------------------------------------ CP mapping
enum class CpClimbClass : uint8_t
{
    High = 0, // ClimbParameters.SetClimbType(0)
    Mid = 1,
    Low = 2
};

// ClimbEvents.GetClimbParameter: obstacleHeight = topPoint.Z - playerPosition.Z (metres), strict '>' tests.
inline CpClimbClass ClassifyCpObstacle(double aHeightM, double aHighThresholdM, double aMidThresholdM)
{
    if (aHeightM > aHighThresholdM)
        return CpClimbClass::High;
    if (aHeightM > aMidThresholdM)
        return CpClimbClass::Mid;
    return CpClimbClass::Low;
}

// Vertical scale mapping a montage's animated ledge height (warp point Z at the FrontLedge window end, metres)
// onto the real obstacle height. The scale is clamped so extreme mismatches are rejected rather than stretched.
struct HeightMapping
{
    bool ok = false;
    double scale = 1.0;
};
inline HeightMapping MapObstacleHeight(double aObstacleM, double aClipLedgeM, double aMinScale, double aMaxScale)
{
    HeightMapping m;
    if (aClipLedgeM <= 1e-6 || aObstacleM <= 0)
        return m;
    m.scale = aObstacleM / aClipLedgeM;
    m.ok = m.scale >= aMinScale && m.scale <= aMaxScale;
    return m;
}

// Chooser inputs in motion units (cm, cm/s) from game metres; height/depth/speed scaled, booleans copied.
inline Inputs MakeInputsFromMetres(bool aFront, bool aBack, bool aFloor, double aHeightM, double aDepthM,
                                   double aBackLedgeHeightM, double aSpeedMs, int aMovementMode)
{
    Inputs in;
    in.hasFrontLedge = aFront;
    in.hasBackLedge = aBack;
    in.hasBackFloor = aFloor;
    in.obstacleHeight = aHeightM * 100.0;
    in.obstacleDepth = aDepthM * 100.0;
    in.backLedgeHeight = aBackLedgeHeightM * 100.0;
    in.speed = aSpeedMs * 100.0;
    in.movementMode = aMovementMode;
    return in;
}
} // namespace utpp::mm::traversal
