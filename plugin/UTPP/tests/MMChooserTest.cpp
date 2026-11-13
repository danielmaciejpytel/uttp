// Offline test of the data-driven chooser (src/mm/Chooser.hpp, stage 10.1 "utpp_mm_chooser_test").
//
//   utpp_mm_chooser_test <chooser table json> <oracle json> <vectors json>
//
// 1. Synthetic tables (generic names, no game data): closed float32 ranges, bNoMin / bNoMax, int / float / double
//    property conversion, wrap mode, enum MatchEqual / MatchNotEqual / MatchAny / unknown, bool cells, nested tables,
//    fallback only when no row succeeded (table still Failed), null asset / null table / invalid results, FirstResult
//    stop, unreadable-input passthrough, rows beyond a column's cells, depth limit, loader rejections, JSON details.
// 2. Agreement with the hand-written oracle (chooser_oracle.json): database list (order and set) for every vector.
// 3. Agreement with the generator vectors (chooser_vectors.json, motion_chooser_ref.py): AllResults database list in
//    order, status, fallback flag and per-result trace, FirstResult first database - for every vector.
// 4. MM node rules from the data file (blend time / interrupt mode): every branch of the stored rule.
// The data files are motion-derived (outside git); the test prints counts only.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <set>
#include <string>
#include <vector>

#include "../src/mm/Chooser.hpp"

using namespace utpp::mm;
using namespace utpp::mm::chooser;
namespace json = utpp::json;

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

static bool ReadFile(const char* aPath, std::string& aOut)
{
    FILE* f = nullptr;
    if (fopen_s(&f, aPath, "rb") != 0 || !f)
        return false;
    char buf[65536];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0)
        aOut.append(buf, n);
    fclose(f);
    return true;
}

// ---- trace collection (same text as motion_chooser_ref.py: "label[row] > label[fallback] > ...")

struct TraceCollector
{
    const Table* table = nullptr;
    std::vector<std::string> lines;
};

static void CollectTrace(void* aUser, int /*aDb*/, const PathStep* aPath, uint32_t aDepth)
{
    auto* tc = static_cast<TraceCollector*>(aUser);
    std::string s;
    for (uint32_t i = 0; i < aDepth; ++i)
    {
        if (i)
            s += " > ";
        s += tc->table->GetTable(aPath[i].table).label;
        s += aPath[i].row < 0 ? std::string("[fallback]") : "[" + std::to_string(aPath[i].row) + "]";
    }
    tc->lines.push_back(s);
}

static const char* StatusName(Status s)
{
    return s == Status::Continue ? "Continue" : (s == Status::Stop ? "Stop" : "Failed");
}

// Fills Inputs from a JSON object {key: value}; returns false (with message) on an unknown key / bad value.
static bool FillInputs(const Table& t, const json::Value& obj, Inputs& in, std::string& err)
{
    t.ResetInputs(in);
    for (const auto& kv : obj.obj)
    {
        const int idx = t.FindInput(kv.first);
        if (idx < 0)
        {
            err = "unknown input " + kv.first;
            return false;
        }
        const InputDef& def = t.GetInput(static_cast<uint32_t>(idx));
        const json::Value& v = kv.second;
        if (v.IsNull())
        {
            t.SetUnreadable(in, idx);
            continue;
        }
        bool ok = false;
        if (def.kind == InputKind::Float && v.IsNumber())
            ok = t.SetNumber(in, idx, v.num);
        else if (def.kind == InputKind::Bool && v.IsBool())
            ok = t.SetBool(in, idx, v.b);
        else if (def.kind == InputKind::Enum && v.IsString())
            ok = t.SetEnum(in, idx, t.FindEnumValue(idx, v.str));
        else if (def.kind == InputKind::Enum && v.IsNumber())
            ok = t.SetEnum(in, idx, static_cast<int>(v.num));
        if (!ok)
        {
            err = "bad value for input " + kv.first;
            return false;
        }
    }
    return true;
}

static std::vector<std::string> Names(const Table& t, const Output& o)
{
    std::vector<std::string> v;
    for (uint32_t i = 0; i < o.count; ++i)
        v.push_back(t.GetDatabase(static_cast<uint32_t>(o.db[i])).name);
    return v;
}

static std::string Join(const std::vector<std::string>& v)
{
    std::string s = "{";
    for (size_t i = 0; i < v.size(); ++i)
        s += (i ? ", " : "") + v[i];
    return s + "}";
}

// ------------------------------------------------------------------------------------------------ synthetic tables

static const char* kSynthetic = R"JSON({
 "format": "utpp-chooser-table", "version": 1,
 "inputs": [
  {"key": "Ctx.Speed", "kind": "float", "property_type": "double", "display": ["Speed Display"]},
  {"key": "Level", "kind": "float", "property_type": "int", "display": []},
  {"key": "Time", "kind": "float", "property_type": "float", "display": []},
  {"key": "Angle", "kind": "float", "property_type": "double", "display": []},
  {"key": "Flag", "kind": "bool", "property_type": "bool", "display": []},
  {"key": "Mode", "kind": "enum", "property_type": "enum", "display": [],
   "enum": {"name": "E_Mode", "values": [{"value": 0, "name": "NewEnumerator0", "display": "Ground"},
                                          {"value": 1, "name": "NewEnumerator1", "display": "Air"},
                                          {"value": 2, "name": "NewEnumerator2", "display": "Water"}]}},
  {"key": "State", "kind": "enum", "property_type": "enum", "display": [],
   "enum": {"name": "E_State", "values": [{"value": 0, "name": "A", "display": "Idle"},
                                           {"value": 1, "name": "B", "display": "Moving"}]}}
 ],
 "tables": [
  {"label": "Root", "columns": [
     {"input": 1, "key": "Level", "kind": "float", "disabled": false, "wrap": false, "min_value": -180, "max_value": 180,
      "cells": [[0.0, 0.0, false, false], [1.0, 1.0, false, false], [2.0, 2.0, false, false], [3.0, 3.0, false, false],
                [4.0, 4.0, false, false], [5.0, 5.0, false, false], [6.0, 6.0, false, false]]}],
   "rows": [{"disabled": false, "result": {"kind": "table", "table": 1}},
            {"disabled": false, "result": {"kind": "table", "table": 2}},
            {"disabled": false, "result": {"kind": "table", "table": 3}},
            {"disabled": false, "result": {"kind": "table", "table": 4}},
            {"disabled": false, "result": {"kind": "table", "table": 5}},
            {"disabled": false, "result": {"kind": "table", "table": -1}},
            {"disabled": false, "result": {"kind": "table", "table": 6}}],
   "fallback": {"kind": "database", "name": "DB_Fallback", "db": 7}},
  {"label": "Root/Ranges", "columns": [
     {"input": 0, "key": "Ctx.Speed", "kind": "float", "disabled": false, "wrap": false, "min_value": -180, "max_value": 180,
      "cells": [[0.0, 20.0, false, false], [20.0, 999.0, false, true], [0.0, 100.0, true, false],
                [0.0, 0.0, true, true], [0.20000000298023224, 0.5, false, false]]},
     {"input": 4, "key": "Flag", "kind": "bool", "disabled": false,
      "cells": ["MatchFalse", "MatchAny", "MatchAny", "MatchTrue", "MatchAny"]},
     {"input": 2, "key": "Time", "kind": "float", "disabled": true, "wrap": false, "min_value": -180, "max_value": 180,
      "cells": [[50.0, 60.0, false, false], [50.0, 60.0, false, false], [50.0, 60.0, false, false],
                [50.0, 60.0, false, false], [50.0, 60.0, false, false]]}],
   "rows": [{"disabled": false, "result": {"kind": "database", "name": "DB_Low", "db": 0}},
            {"disabled": false, "result": {"kind": "database", "name": "DB_High", "db": 1}},
            {"disabled": false, "result": {"kind": "database", "name": "DB_Below100", "db": 2}},
            {"disabled": false, "result": {"kind": "database", "name": "DB_Flag", "db": 3}},
            {"disabled": false, "result": {"kind": "database", "name": "DB_F32Bound", "db": 4}}],
   "fallback": null},
  {"label": "Root/Enums", "columns": [
     {"input": 5, "key": "Mode", "kind": "enum", "disabled": false,
      "cells": [["MatchEqual", 0], ["MatchNotEqual", 0], ["MatchAny", 2], ["Bogus", 1], ["MatchEqual", 2]]},
     {"input": 2, "key": "Time", "kind": "float", "disabled": false, "wrap": false, "min_value": -180, "max_value": 180,
      "cells": [[1.2000000476837158, 999.0, false, false], [0.0, 0.0, true, true], [0.0, 0.0, true, true],
                [0.0, 0.0, true, true], [0.0, 0.0, true, true]]}],
   "rows": [{"disabled": false, "result": {"kind": "database", "name": "DB_GroundLate", "db": 5}},
            {"disabled": false, "result": {"kind": "database", "name": "DB_NotGround", "db": 6}},
            {"disabled": false, "result": {"kind": "database", "name": "DB_Any", "db": 8}},
            {"disabled": false, "result": {"kind": "database", "name": "DB_Bogus", "db": 9}},
            {"disabled": true, "result": {"kind": "database", "name": "DB_DisabledRow", "db": 10}}],
   "fallback": null},
  {"label": "Root/Wrap", "columns": [
     {"input": 3, "key": "Angle", "kind": "float", "disabled": false, "wrap": true, "min_value": -180, "max_value": 180,
      "cells": [[150.0, -150.0, false, false], [-30.0, 30.0, false, false]]}],
   "rows": [{"disabled": false, "result": {"kind": "database", "name": "DB_Back", "db": 11}},
            {"disabled": false, "result": {"kind": "database", "name": "DB_Front", "db": 12}}],
   "fallback": {"kind": "database", "name": "DB_Side", "db": 13}},
  {"label": "Root/NullAsset", "columns": [],
   "rows": [{"disabled": false, "result": {"kind": "database", "name": null, "db": -1}},
            {"disabled": false, "result": {"kind": "invalid"}}],
   "fallback": {"kind": "database", "name": "DB_NullFallback", "db": 14}},
  {"label": "Root/Nested", "columns": [
     {"input": 6, "key": "State", "kind": "enum", "disabled": false, "cells": [["MatchEqual", 0], ["MatchAny", 0]]}],
   "rows": [{"disabled": false, "result": {"kind": "table", "table": 2}},
            {"disabled": false, "result": {"kind": "database", "name": "DB_Low", "db": 0}},
            {"disabled": false, "result": {"kind": "database", "name": "DB_Beyond", "db": 15}}],
   "fallback": null},
  {"label": "Root/Loop", "columns": [],
   "rows": [{"disabled": false, "result": {"kind": "table", "table": 6}}],
   "fallback": null}
 ],
 "mm_rules": {
  "blend_time": {"mode_input": "Mode", "ground_value": 0, "air_value": 1, "ground_to_ground_s": 0.4,
                 "landing_s": 0.25, "air_rising_s": 0.1, "air_other_s": 0.3, "rising_threshold_cm_s": 50.0},
  "interrupt": {"mode_input": "Mode", "ground_value": 0, "ground_state_inputs": ["State", "Flag"]}
 },
 "blend_profile": {"name": "P", "mode": "WeightFactor", "entries": [{"bone": "a", "scale": 2.0, "cp": "A"},
                                                                    {"bone": "b", "scale": 4.0, "cp": null}]}
})JSON";

struct Synth
{
    Table t;
    int speed = -1, level = -1, time = -1, angle = -1, flag = -1, mode = -1, state = -1;
};

static std::vector<std::string> Run(const Synth& s, const Inputs& in, Mode m = Mode::AllResults, Output* out = nullptr,
                                    std::vector<std::string>* trace = nullptr)
{
    Output o;
    TraceCollector tc;
    tc.table = &s.t;
    s.t.Evaluate(in, m, o, CollectTrace, &tc);
    if (out)
        *out = o;
    if (trace)
        *trace = tc.lines;
    return Names(s.t, o);
}

static void TestSynthetic()
{
    Synth s;
    std::string err;
    Check(s.t.Load(kSynthetic, err), "synthetic table loads: " + err);
    if (!s.t.Loaded())
        return;
    Check(s.t.NumInputs() == 7 && s.t.NumTables() == 7, "synthetic counts");
    s.speed = s.t.FindInput("Ctx.Speed");
    Check(s.speed == 0 && s.t.FindInput("speed") == 0 && s.t.FindInput("Speed Display") == 0 &&
              s.t.FindInput("speed_display") == 0,
          "FindInput: key, last segment, display, normalized");
    Check(s.t.FindInput("nope") == -1, "FindInput unknown = -1");
    s.level = s.t.FindInput("Level");
    s.time = s.t.FindInput("Time");
    s.angle = s.t.FindInput("Angle");
    s.flag = s.t.FindInput("Flag");
    s.mode = s.t.FindInput("Mode");
    s.state = s.t.FindInput("State");
    Check(s.t.FindEnumValue(s.mode, "Air") == 1 && s.t.FindEnumValue(s.mode, "NewEnumerator2") == 2 &&
              s.t.FindEnumValue(s.mode, "E_Mode::NewEnumerator1 (Air)") == 1 && s.t.FindEnumValue(s.mode, "2") == 2 &&
              s.t.FindEnumValue(s.mode, "Lava") == -1 && s.t.FindEnumValue(s.speed, "Air") == -1,
          "FindEnumValue forms");

    Inputs in;
    auto base = [&]() {
        s.t.ResetInputs(in);
        s.t.SetNumber(in, s.speed, 0.0);
        s.t.SetNumber(in, s.level, 0.0);
        s.t.SetNumber(in, s.time, 0.0);
        s.t.SetNumber(in, s.angle, 0.0);
        s.t.SetBool(in, s.flag, false);
        s.t.SetEnum(in, s.mode, 0);
        s.t.SetEnum(in, s.state, 0);
    };

    // ---- property conversion
    base();
    Check(!s.t.SetNumber(in, s.level, 0.5), "int property rejects 0.5");
    Check(s.t.SetNumber(in, s.level, -1.0), "int property accepts -1");
    Check(!s.t.SetBool(in, s.speed, true) && !s.t.SetNumber(in, s.flag, 1.0) && !s.t.SetEnum(in, s.mode, 256) &&
              !s.t.SetEnum(in, s.mode, -1),
          "Set* type / range checks");
    s.t.SetNumber(in, s.time, 1.2);
    Check(in.value[s.time] == static_cast<double>(1.2f), "float property stored as float32");
    s.t.SetNumber(in, s.speed, 1.2);
    Check(in.value[s.speed] == 1.2, "double property kept as double");

    // ---- ranges (Level 0 -> Root/Ranges)
    struct RangeCase
    {
        double speed;
        bool flag;
        const char* expect;
    };
    // Root/Ranges rows: 0 [0,20], 1 [20,+inf), 2 (-inf,100], 3 any, 4 [f32(0.2), 0.5]; Flag: 0 false, 3 true, others any.
    const RangeCase cases[] = {
        {0.0, false, "{DB_Low, DB_Below100}"},
        {-0.01, false, "{DB_Below100}"},
        {20.0, false, "{DB_Low, DB_High, DB_Below100}"},
        {std::nextafter(20.0, 30.0), false, "{DB_High, DB_Below100}"},
        {std::nextafter(20.0, 0.0), false, "{DB_Low, DB_Below100}"},
        {100.0, true, "{DB_High, DB_Below100, DB_Flag}"},
        {std::nextafter(100.0, 200.0), true, "{DB_High, DB_Flag}"},
        {1e9, true, "{DB_High, DB_Flag}"},
        {0.2, true, "{DB_Below100, DB_Flag}"},                             // double 0.2 < float32 bound 0.2f
        {0.20000000298023224, true, "{DB_Below100, DB_Flag, DB_F32Bound}"}, // exactly the float32 bound (closed)
        {0.5, false, "{DB_Low, DB_Below100, DB_F32Bound}"},
    };
    for (const RangeCase& c : cases)
    {
        base();
        s.t.SetNumber(in, s.speed, c.speed);
        s.t.SetBool(in, s.flag, c.flag);
        const std::string got = Join(Run(s, in));
        char what[200];
        std::snprintf(what, sizeof(what), "range speed=%.17g flag=%d -> %s (got %s)", c.speed, c.flag ? 1 : 0, c.expect,
                      got.c_str());
        Check(got == c.expect, what);
    }
    // closed float32 bound on a float property: Time 1.2 == f32(1.2) bound of Root/Enums row 0
    base();
    s.t.SetNumber(in, s.level, 1.0);
    s.t.SetNumber(in, s.time, 1.2);
    Check(Join(Run(s, in)) == "{DB_GroundLate, DB_Any}", "float property: 1.2 hits the closed float32 bound 1.2f");
    base();
    s.t.SetNumber(in, s.level, 1.0);
    s.t.SetNumber(in, s.time, std::nextafter(1.2f, 0.0f));
    Check(Join(Run(s, in)) == "{DB_Any}", "float property: just below 1.2f misses");
    // enum comparisons: Mode = Air -> NotEqual + Any (+ unknown comparison never matches)
    base();
    s.t.SetNumber(in, s.level, 1.0);
    s.t.SetEnum(in, s.mode, 1);
    Check(Join(Run(s, in)) == "{DB_NotGround, DB_Any}", "enum MatchNotEqual / MatchAny / unknown comparison / disabled row");
    // unreadable input -> passthrough of that column only
    base();
    s.t.SetNumber(in, s.level, 1.0);
    s.t.SetUnreadable(in, s.mode);
    Check(Join(Run(s, in)) == "{DB_NotGround, DB_Any, DB_Bogus}",
          "unreadable enum: the whole column passes (even the unknown comparison); Time column still filters");
    base();
    s.t.SetNumber(in, s.level, 1.0);
    s.t.SetUnreadable(in, s.mode);
    s.t.SetUnreadable(in, s.time);
    Check(Join(Run(s, in)) == "{DB_GroundLate, DB_NotGround, DB_Any, DB_Bogus}",
          "all unreadable: every enabled row passes; disabled row stays out");

    // wrap mode
    struct WrapCase
    {
        double angle;
        const char* expect;
    };
    // A nested table that produced results only through its fallback still reports Failed, so the root row fails and
    // the root fallback runs too (Chooser.cpp:824).
    const WrapCase wraps[] = {{0.0, "{DB_Front}"},   {170.0, "{DB_Back}"},  {-170.0, "{DB_Back}"},
                              {390.0, "{DB_Front}"}, {-330.0, "{DB_Front}"}, {90.0, "{DB_Side, DB_Fallback}"},
                              {540.0, "{DB_Back}"},  {150.0, "{DB_Back}"},  {30.0, "{DB_Front}"}};
    for (const WrapCase& w : wraps)
    {
        base();
        s.t.SetNumber(in, s.level, 2.0);
        s.t.SetNumber(in, s.angle, w.angle);
        Output o;
        const std::string got = Join(Run(s, in, Mode::AllResults, &o));
        Check(got == w.expect, "wrap angle " + std::to_string(w.angle) + " -> " + w.expect + " got " + got);
        if (w.angle == 90.0)
            Check(o.fallbackUsed && o.status == Status::Failed, "fallback-only nested table: root Failed, root fallback ran");
        else
            Check(!o.fallbackUsed && o.status == Status::Continue, "wrap row hit: Continue, no fallback");
    }
    // null asset / invalid result -> table fallback (nested Failed -> root fallback as well)
    base();
    s.t.SetNumber(in, s.level, 3.0);
    {
        Output o;
        std::vector<std::string> tr;
        Check(Join(Run(s, in, Mode::AllResults, &o, &tr)) == "{DB_NullFallback, DB_Fallback}",
              "null asset + invalid row -> nested fallback, then root fallback");
        Check(tr.size() == 2 && tr[0] == "Root[3] > Root/NullAsset[fallback]" && tr[1] == "Root[fallback]",
              "trace of nested + root fallback: " + (tr.empty() ? "" : tr[0]));
    }
    // null table row -> Failed -> root fallback (root returns Failed although the fallback produced a result)
    base();
    s.t.SetNumber(in, s.level, 5.0);
    {
        Output o;
        std::vector<std::string> tr;
        Check(Join(Run(s, in, Mode::AllResults, &o, &tr)) == "{DB_Fallback}" && o.status == Status::Failed && o.fallbackUsed,
              "null table -> root fallback, root status Failed");
        Check(tr.size() == 1 && tr[0] == "Root[fallback]", "trace root fallback");
    }
    // no root row -> fallback
    base();
    s.t.SetNumber(in, s.level, 9.0);
    Check(Join(Run(s, in)) == "{DB_Fallback}", "no matching root row -> fallback");
    // nested table reached twice (State Idle): Root/Nested row 0 -> Root/Enums, row 1 -> DB_Low
    base();
    s.t.SetNumber(in, s.level, 4.0);
    {
        Output o;
        std::vector<std::string> tr;
        const std::string got = Join(Run(s, in, Mode::AllResults, &o, &tr));
        Check(got == "{DB_Any, DB_Low}", "nested table chain order: " + got);
        Check(tr.size() == 2 && tr[0] == "Root[4] > Root/Nested[0] > Root/Enums[2]" && tr[1] == "Root[4] > Root/Nested[1]",
              "nested traces");
        Check(o.Mask() == ((uint64_t(1) << s.t.FindDatabase("DB_Any")) | (uint64_t(1) << s.t.FindDatabase("DB_Low"))),
              "Mask over database ids");
        Output f;
        const std::string first = Join(Run(s, in, Mode::FirstResult, &f));
        Check(first == "{DB_Any}" && f.status == Status::Stop, "FirstResult stops after the first database");
    }
    // self-recursive table -> depth limit, Failed, fallback of root
    base();
    s.t.SetNumber(in, s.level, 6.0);
    {
        Output o;
        Check(Join(Run(s, in, Mode::AllResults, &o)) == "{DB_Fallback}" && o.depthExceeded, "depth limit fails the loop");
    }
    // database ids keep first-appearance order; dbIndex from the file
    Check(s.t.FindDatabase("DB_Fallback") >= 0 && s.t.GetDatabase(static_cast<uint32_t>(s.t.FindDatabase("DB_Fallback"))).dbIndex == 7,
          "database dbIndex loaded");

    // ---- rules
    const BlendTimeRules& bt = s.t.GetBlendTimeRules();
    Check(bt.valid && bt.modeInput == s.mode, "blend time rules loaded");
    Check(GetBlendTime(bt, 0, 0, 0.0f, -1.0f) == 0.4f && GetBlendTime(bt, 0, 1, 0.0f, -1.0f) == 0.25f &&
              GetBlendTime(bt, 1, 0, 51.0f, -1.0f) == 0.1f && GetBlendTime(bt, 1, 1, 50.0f, -1.0f) == 0.3f &&
              GetBlendTime(bt, 0, 2, 0.0f, -1.0f) == -1.0f && GetBlendTime(bt, 2, 0, 0.0f, -1.0f) == -1.0f &&
              GetBlendTime(BlendTimeRules{}, 0, 0, 0.0f, -2.0f) == -2.0f,
          "GetBlendTime branches (threshold strict >)");
    const InterruptRules& ir = s.t.GetInterruptRules();
    Check(ir.valid && ir.numStateInputs == 2, "interrupt rules loaded");
    {
        Inputs a, b;
        base();
        a = in;
        b = in;
        Check(GetInterruptMode(ir, a, b) == InterruptMode::DoNotInterrupt, "interrupt: nothing changed");
        s.t.SetEnum(b, s.state, 1);
        Check(GetInterruptMode(ir, a, b) == InterruptMode::InterruptOnDatabaseChange, "interrupt: state change on ground");
        s.t.SetEnum(a, s.mode, 1);
        s.t.SetEnum(b, s.mode, 1);
        Check(GetInterruptMode(ir, a, b) == InterruptMode::DoNotInterrupt, "interrupt: state change in air ignored");
        s.t.SetEnum(b, s.mode, 0);
        Check(GetInterruptMode(ir, a, b) == InterruptMode::InterruptOnDatabaseChange, "interrupt: mode change");
    }
    const BlendProfile& bp = s.t.GetBlendProfile();
    Check(bp.valid && bp.weightFactor && bp.entries.size() == 2 && bp.entries[0].cpBone == "A" && bp.entries[1].cpBone.empty(),
          "blend profile loaded");

    // ---- loader rejections
    {
        Table bad;
        std::string e;
        Check(!bad.Load("{", e) && !e.empty(), "malformed JSON rejected: " + e);
        Check(!bad.Load(R"({"format": "x", "version": 1, "inputs": [], "tables": [{}]})", e), "wrong format rejected");
        std::string doc = kSynthetic;
        std::string d1 = doc;
        d1.replace(d1.find("\"table\": 6}"), 10, "\"table\": 60"); // first occurrence: root row 6
        Check(!bad.Load(d1, e) && !bad.Loaded(), "table index out of range rejected: " + e);
        std::string d2 = doc;
        d2.replace(d2.find("\"MatchFalse\", \"MatchAny\""), 12, "\"MatchMaybe\"");
        Check(!bad.Load(d2, e), "bad bool cell rejected: " + e);
        std::string d3 = doc;
        d3.replace(d3.find("{\"input\": 5, \"key\": \"Mode\", \"kind\": \"enum\""), 11, "{\"input\": 4");
        Check(!bad.Load(d3, e), "column kind differing from its input rejected: " + e);
        Inputs i0;
        Output o0;
        bad.ResetInputs(i0);
        Check(bad.Evaluate(i0, Mode::AllResults, o0) == Status::Failed && o0.count == 0, "empty table evaluates Failed");
    }

    // ---- MiniJson details
    {
        json::Value v;
        std::string e;
        Check(json::Parse(R"([5e-324, -0.0, 19.999999999999996, 1e308, "\u00e9\ud83d\ude00\n", true, null])", v, e) &&
                  v.arr.size() == 7,
              "MiniJson parses: " + e);
        if (v.arr.size() == 7)
        {
            Check(v.arr[0].num == std::nextafter(0.0, 1.0), "subnormal 5e-324 exact");
            Check(std::signbit(v.arr[1].num) && v.arr[1].num == 0.0, "-0.0");
            Check(v.arr[2].num == std::nextafter(20.0, 0.0), "19.999999999999996 = nextafter(20, 0)");
            Check(v.arr[4].str == "\xC3\xA9\xF0\x9F\x98\x80\n", "unicode escapes / surrogate pair");
            Check(v.arr[0].isInteger == false && json::Parse("[12]", v, e) && v.arr[0].isInteger, "isInteger");
        }
        Check(!json::Parse("[1,]", v, e) && !json::Parse("01", v, e) && !json::Parse("\"a", v, e) &&
                  !json::Parse("[1e999]", v, e) && !json::Parse("{\"a\":1} x", v, e) && !json::Parse("\"\\ud800\"", v, e),
              "MiniJson rejects malformed input");
        std::string deep(100, '[');
        deep += std::string(100, ']');
        Check(!json::Parse(deep, v, e), "MiniJson depth limit");
    }
}

// ------------------------------------------------------------------------------------------------ data files

static int TestOracle(const Table& t, const char* aPath)
{
    std::string text, err;
    if (!ReadFile(aPath, text))
    {
        Check(false, std::string("cannot read ") + aPath);
        return 0;
    }
    json::Value doc;
    if (!json::Parse(text, doc, err))
    {
        Check(false, "oracle JSON: " + err);
        return 0;
    }
    const json::Value* vecs = doc.Find("vectors");
    if (!vecs || !vecs->IsArray())
    {
        Check(false, "oracle: no vectors");
        return 0;
    }
    int agreeOrder = 0, agreeSet = 0, n = 0;
    for (const json::Value& v : vecs->arr)
    {
        ++n;
        const json::Value* id = v.Find("id");
        const std::string name = (id && id->IsString()) ? id->str : std::to_string(n);
        Inputs in;
        if (!FillInputs(t, *v.Find("inputs"), in, err))
        {
            Check(false, "oracle " + name + ": " + err);
            continue;
        }
        Output o;
        t.Evaluate(in, Mode::AllResults, o);
        const std::vector<std::string> got = Names(t, o);
        std::vector<std::string> exp;
        for (const json::Value& d : v.Find("expected_databases")->arr)
            exp.push_back(d.str);
        const bool order = got == exp;
        const bool set = std::set<std::string>(got.begin(), got.end()) == std::set<std::string>(exp.begin(), exp.end()) &&
                         got.size() == exp.size();
        agreeOrder += order ? 1 : 0;
        agreeSet += set ? 1 : 0;
        Check(set, "oracle " + name + ": expected " + Join(exp) + " got " + Join(got));
    }
    std::printf("oracle: %d vectors, set agreement %d/%d, order agreement %d/%d\n", n, agreeSet, n, agreeOrder, n);
    Check(n > 0 && agreeSet == n && agreeOrder == n, "oracle 100% agreement (set and UE emission order)");
    return n;
}

static int TestVectors(const Table& t, const char* aPath)
{
    std::string text, err;
    if (!ReadFile(aPath, text))
    {
        Check(false, std::string("cannot read ") + aPath);
        return 0;
    }
    json::Value doc;
    if (!json::Parse(text, doc, err))
    {
        Check(false, "vectors JSON: " + err);
        return 0;
    }
    const json::Value* vecs = doc.Find("vectors");
    const json::Value* count = doc.Find("count");
    const json::Value* inputs = doc.Find("inputs");
    if (!vecs || !vecs->IsArray())
    {
        Check(false, "vectors: no vectors");
        return 0;
    }
    Check(count && count->IsNumber() && static_cast<size_t>(count->num) == vecs->arr.size(), "vectors: header count");
    Check(inputs && inputs->IsObject() && inputs->obj.size() == t.NumInputs(), "vectors: same input set size as the table");
    if (inputs && inputs->IsObject())
        for (const auto& kv : inputs->obj)
        {
            const int idx = t.FindInput(kv.first);
            const json::Value* pt = kv.second.Find("property_type");
            bool same = idx >= 0 && pt && pt->IsString();
            if (same)
            {
                const PropType p = t.GetInput(static_cast<uint32_t>(idx)).prop;
                const std::string& s = pt->str;
                same = (s == "float" && p == PropType::Float) || (s == "double" && p == PropType::Double) ||
                       (s == "int" && p == PropType::Int) || (s == "bool" && p == PropType::Bool) ||
                       (s == "enum" && p == PropType::Enum);
            }
            Check(same, "vectors: input " + kv.first + " present with the same property type");
        }

    int n = 0, okDb = 0, okStatus = 0, okFallback = 0, okTrace = 0, okFirst = 0, empty = 0, fallbacks = 0;
    int reportedFails = 0;
    double evalSeconds = 0.0;
    for (const json::Value& v : vecs->arr)
    {
        ++n;
        Inputs in;
        if (!FillInputs(t, *v.Find("inputs"), in, err))
        {
            Check(false, "vector " + std::to_string(n) + ": " + err);
            continue;
        }
        Output o;
        TraceCollector tc;
        tc.table = &t;
        const auto t0 = std::chrono::steady_clock::now();
        t.Evaluate(in, Mode::AllResults, o, CollectTrace, &tc);
        const auto t1 = std::chrono::steady_clock::now();
        evalSeconds += std::chrono::duration<double>(t1 - t0).count();
        Output f;
        t.Evaluate(in, Mode::FirstResult, f);

        std::vector<std::string> exp, expTrace;
        for (const json::Value& d : v.Find("databases")->arr)
            exp.push_back(d.str);
        for (const json::Value& d : v.Find("trace")->arr)
            expTrace.push_back(d.str);
        const json::Value* st = v.Find("status");
        const json::Value* fb = v.Find("fallback_used");
        const json::Value* fr = v.Find("first_result");
        const std::vector<std::string> got = Names(t, o);
        const bool dbOk = got == exp;
        const bool statusOk = st && st->IsString() && st->str == StatusName(o.status);
        const bool fbOk = fb && fb->IsBool() && fb->b == o.fallbackUsed;
        const bool traceOk = tc.lines == expTrace;
        const std::string firstGot = f.count ? t.GetDatabase(static_cast<uint32_t>(f.db[0])).name : std::string();
        const bool firstOk = fr && ((fr->IsNull() && f.count == 0) || (fr->IsString() && fr->str == firstGot)) &&
                             !f.overflow && f.count <= 1;
        okDb += dbOk;
        okStatus += statusOk;
        okFallback += fbOk;
        okTrace += traceOk;
        okFirst += firstOk;
        empty += exp.empty() ? 1 : 0;
        fallbacks += (fb && fb->IsBool() && fb->b) ? 1 : 0;
        if (!(dbOk && statusOk && fbOk && traceOk && firstOk) && reportedFails < 20)
        {
            ++reportedFails;
            std::printf("FAIL vector %d: db %s exp %s status %s fb %d trace %d first %s\n", n, Join(got).c_str(),
                        Join(exp).c_str(), StatusName(o.status), o.fallbackUsed ? 1 : 0, traceOk ? 1 : 0, firstGot.c_str());
        }
    }
    std::printf("vectors: %d (empty %d, fallback %d); agreement databases %d/%d, status %d/%d, fallback %d/%d, "
                "trace %d/%d, first_result %d/%d; mean AllResults evaluation %.3f us\n",
                n, empty, fallbacks, okDb, n, okStatus, n, okFallback, n, okTrace, n, okFirst, n,
                n ? evalSeconds * 1e6 / n : 0.0);
    Check(n > 0 && okDb == n, "vectors: 100% database list agreement (order)");
    Check(okStatus == n, "vectors: 100% status agreement");
    Check(okFallback == n, "vectors: 100% fallback flag agreement");
    Check(okTrace == n, "vectors: 100% trace agreement");
    Check(okFirst == n, "vectors: 100% FirstResult agreement");
    return n;
}

static void TestDataRules(const Table& t)
{
    const BlendTimeRules& bt = t.GetBlendTimeRules();
    const InterruptRules& ir = t.GetInterruptRules();
    Check(bt.valid && ir.valid, "data file: mm_rules present");
    if (!bt.valid || !ir.valid)
        return;
    const int g = bt.groundValue, a = bt.airValue;
    Check(g != a && bt.groundToGround > 0.0f && bt.landing > 0.0f && bt.airRising > 0.0f && bt.airOther > 0.0f,
          "data file: blend time rule values");
    Check(GetBlendTime(bt, g, g, 0.0f, -1.0f) == bt.groundToGround && GetBlendTime(bt, g, a, 0.0f, -1.0f) == bt.landing &&
              GetBlendTime(bt, a, g, bt.risingThresholdCmS + 1.0f, -1.0f) == bt.airRising &&
              GetBlendTime(bt, a, a, bt.risingThresholdCmS, -1.0f) == bt.airOther &&
              GetBlendTime(bt, a, a, -500.0f, -1.0f) == bt.airOther,
          "data file: blend time branches");
    std::printf("data rules: blend time ground->ground %.3f s, landing %.3f s, rising %.3f s (vz > %.1f cm/s), other in "
                "air %.3f s; interrupt: %u ground state inputs\n",
                bt.groundToGround, bt.landing, bt.airRising, bt.risingThresholdCmS, bt.airOther, ir.numStateInputs);
    Check(ir.modeInput == bt.modeInput && ir.groundValue == bt.groundValue, "data file: interrupt and blend time agree on mode");
    // Every ground state input change interrupts on the ground and is ignored in the air.
    Inputs a0;
    t.ResetInputs(a0);
    for (uint32_t i = 0; i < ir.numStateInputs; ++i)
    {
        Inputs now = a0, last = a0;
        now.value[ir.modeInput] = last.value[ir.modeInput] = g;
        now.value[ir.stateInputs[i]] = 1.0;
        Check(GetInterruptMode(ir, now, last) == InterruptMode::InterruptOnDatabaseChange, "data file: ground state change interrupts");
        now.value[ir.modeInput] = last.value[ir.modeInput] = a;
        Check(GetInterruptMode(ir, now, last) == InterruptMode::DoNotInterrupt, "data file: air state change does not");
    }
    const BlendProfile& bp = t.GetBlendProfile();
    uint32_t mapped = 0;
    for (const auto& e : bp.entries)
        mapped += e.cpBone.empty() ? 0 : 1;
    std::printf("data blend profile: %s, %zu entries, %u mapped to the game rig\n", bp.weightFactor ? "WeightFactor" : "TimeFactor",
                bp.entries.size(), mapped);
    Check(bp.valid && !bp.entries.empty(), "data file: blend profile present");
}

int main(int argc, char** argv)
{
    TestSynthetic();
    std::printf("synthetic: %d passed, %d failed\n", g_pass, g_fail);
    if (argc >= 2)
    {
        Table t;
        std::string err;
        const bool ok = t.LoadFile(argv[1], err);
        Check(ok, "chooser data file loads: " + err);
        if (ok)
        {
            std::printf("chooser data: %u inputs, %u tables, %u databases\n", t.NumInputs(), t.NumTables(), t.NumDatabases());
            TestDataRules(t);
            if (argc >= 3)
                TestOracle(t, argv[2]);
            if (argc >= 4)
                TestVectors(t, argv[3]);
        }
    }
    else
        std::printf("NOTE: no chooser data file given - data agreement tests not run\n");
    std::printf("utpp_mm_chooser_test: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
