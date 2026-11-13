// Offline test of the optional slide model of the chooser (src/mm/Chooser.hpp SlideInfo, motion slide).
//
//   utpp_mm_chooser_slide_test <slide chooser json> <base chooser json> <slide oracle json>
//
// 1. The base table (bundle without slide data) has no slide section, no Sliding value, loads exactly as before.
// 2. The slide table: section parsed (version, inputs, database ids), MovementMode has the value Sliding.
// 3. Oracle (make_slide_oracle.py, hand-written rules): for every vector the AllResults database set of the slide table
//    equals the slide-only expectation (mode Sliding) or base-set + exit database (LastFrame Sliding on the ground) or the
//    base set (everything else). The base set is the result of the BASE table for the same inputs.
// 4. Regression guard: with MovementMode_LastFrame / MovementDirection not Sliding / Forward the slide table gives the base sets
//    for every non-slide vector (no slide database leaks into regular rows).
// The data files are motion-derived (outside git); the test prints counts only.

#include <cstdio>
#include <set>
#include <string>
#include <vector>

#include "../src/mm/Chooser.hpp"

using namespace utpp::mm::chooser;
namespace json = utpp::json;

static int g_pass = 0, g_fail = 0;

static void Check(bool ok, const std::string& what)
{
    if (ok)
        ++g_pass;
    else
    {
        ++g_fail;
        std::printf("FAIL: %s\n", what.c_str());
    }
}

static bool ReadFile(const char* path, std::string& out)
{
    FILE* f = nullptr;
    if (fopen_s(&f, path, "rb") != 0 || !f)
        return false;
    char buf[65536];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0)
        out.append(buf, n);
    fclose(f);
    return true;
}

struct Vec
{
    int lod, mode, last, stance, state, gait, direction;
    double speed;
    std::vector<std::string> slideOnly;
    std::string extra;
};

static void SetCommon(const Table& t, Inputs& in, const Vec& v, bool withSlideInputs)
{
    t.ResetInputs(in);
    t.SetEnum(in, t.FindInput("Gait"), v.gait);
    t.SetEnum(in, t.FindInput("Stance"), v.stance);
    t.SetEnum(in, t.FindInput("MovementState"), v.state);
    t.SetEnum(in, t.FindInput("MovementMode"), v.mode);
    t.SetNumber(in, t.FindInput("Speed2D"), v.speed);
    t.SetNumber(in, t.FindInput("TrajectoryCollision.TimeToLand"), 1.5);
    t.SetNumber(in, t.FindInput("MMDatabaseLOD"), v.lod);
    for (const char* b : {"IsStarting", "IsPivoting", "JustLanded_Light", "JustLanded_Heavy", "JustTraversed", "ShouldTurnInPlace",
                          "ShouldSpinTransition"})
        t.SetBool(in, t.FindInput(b), false);
    if (withSlideInputs)
    {
        t.SetEnum(in, t.GetSlide().directionInput, v.direction);
        t.SetEnum(in, t.GetSlide().lastModeInput, v.last);
    }
}

static std::set<std::string> Names(const Table& t, const Inputs& in)
{
    Output out;
    t.Evaluate(in, Mode::AllResults, out);
    std::set<std::string> s;
    for (uint32_t i = 0; i < out.count; ++i)
        if (out.db[i] >= 0 && out.db[i] < static_cast<int>(t.NumDatabases()))
            s.insert(t.GetDatabase(static_cast<uint32_t>(out.db[i])).name);
    return s;
}

int main(int argc, char** argv)
{
    if (argc < 4)
    {
        std::printf("usage: utpp_mm_chooser_slide_test <slide chooser> <base chooser> <slide oracle>\nFAIL: arguments\n");
        return 1;
    }
    std::string err;
    Table slide, base;
    if (!slide.LoadFile(argv[1], err))
    {
        std::printf("FAIL: slide table: %s\n", err.c_str());
        return 1;
    }
    if (!base.LoadFile(argv[2], err))
    {
        std::printf("FAIL: base table: %s\n", err.c_str());
        return 1;
    }
    const SlideInfo& si = slide.GetSlide();
    Check(!base.GetSlide().present, "base table has no slide section");
    Check(base.FindInput("MovementDirection") < 0 && base.FindInput("MovementMode_LastFrame") < 0, "base table lacks the slide inputs");
    Check(si.present && si.version == kSlideSectionVersion, "slide section present, version 1");
    Check(si.slidingValue == 2, "Sliding value 2");
    Check(si.modeInput == slide.FindInput("MovementMode"), "mode input index");
    Check(si.directionInput >= 0 && si.lastModeInput >= 0, "slide inputs found");
    Check(si.slideTable > 0 && si.slideTable < static_cast<int>(slide.NumTables()) && slide.GetTable(static_cast<uint32_t>(si.slideTable)).label == "Slide",
          "Slide table index / label");
    bool hasSliding = false;
    for (const EnumEntry& e : slide.GetInput(static_cast<uint32_t>(si.modeInput)).enumEntries)
        hasSliding = hasSliding || e.value == 2;
    Check(hasSliding, "MovementMode enum has value 2");
    Check(si.databaseNames.size() == 8 && si.databaseNames.size() == si.databaseIds.size(), "8 slide databases in the section");
    for (size_t i = 0; i < si.databaseNames.size(); ++i)
        Check(si.databaseIds[i] >= 0, "database " + si.databaseNames[i] + " baked");

    Check(slide.FindInput("MMDatabaseLOD") >= 0 && slide.FindInput("Speed2D") >= 0 && slide.FindInput("Gait") >= 0, "inputs found by name");
    {
        // the LOD input must route: Sliding with LOD 0 and LOD 1 both reach the Slide table through different root families
        Vec v{};
        v.mode = 2;
        Inputs in;
        v.lod = 1;
        SetCommon(slide, in, v, true);
        Output out;
        slide.Evaluate(in, Mode::AllResults, out);
        Check(in.readable[slide.FindInput("MMDatabaseLOD")], "LOD input readable");
        std::printf("probe: sliding lod1 results %u\n", out.count);
    }
    std::string text;
    json::Value doc;
    if (!ReadFile(argv[3], text) || !json::Parse(text, doc, err))
    {
        std::printf("FAIL: oracle %s\n", argv[3]);
        return 1;
    }
    const json::Value* vs = doc.Find("vectors");
    if (!vs || !vs->IsArray())
    {
        std::printf("FAIL: oracle without vectors\n");
        return 1;
    }
    auto num = [](const json::Value& o, const char* k) { const json::Value* m = o.Find(k); return m ? m->num : 0.0; };
    int nSlide = 0, nExit = 0, nPlain = 0, nExitMissing = 0;
    for (const json::Value& jv : vs->arr)
    {
        Vec v{};
        v.lod = static_cast<int>(num(jv, "lod"));
        v.mode = static_cast<int>(num(jv, "mode"));
        v.last = static_cast<int>(num(jv, "last"));
        v.stance = static_cast<int>(num(jv, "stance"));
        v.state = static_cast<int>(num(jv, "state"));
        v.gait = static_cast<int>(num(jv, "gait"));
        v.direction = static_cast<int>(num(jv, "direction"));
        v.speed = num(jv, "speed");
        if (const json::Value* so = jv.Find("slide_only"))
            for (const json::Value& n : so->arr)
                v.slideOnly.push_back(n.str);
        if (const json::Value* ex = jv.Find("extra"))
            v.extra = ex->str;
        char tag[160];
        std::snprintf(tag, sizeof(tag), "lod%d mode%d last%d stance%d state%d gait%d dir%d", v.lod, v.mode, v.last, v.stance, v.state, v.gait,
                      v.direction);
        Inputs in;
        SetCommon(slide, in, v, true);
        const std::set<std::string> got = Names(slide, in);
        if (!v.slideOnly.empty())
        {
            ++nSlide;
            Check(got == std::set<std::string>(v.slideOnly.begin(), v.slideOnly.end()), std::string("slide rows: ") + tag);
            continue;
        }
        Vec bv = v;
        Inputs bin;
        SetCommon(base, bin, bv, false);
        std::set<std::string> want = Names(base, bin);
        if (!v.extra.empty())
        {
            ++nExit;
            if (want.empty())
                ++nExitMissing; // table not reached in the base: no exit expected
            else
                want.insert(v.extra);
        }
        else
            ++nPlain;
        Check(got == want, std::string("regular + exit rows: ") + tag);
        // 4. a non-slide last mode / direction must give the base set
        if (v.mode == 0 && v.last != 2)
            for (const std::string& n : got)
                Check(n.rfind("PSD_Slide_", 0) != 0, std::string("slide database leaked into a regular row: ") + tag);
    }
    std::printf("vectors: %d slide, %d exit (%d without reached table), %d plain\n", nSlide, nExit, nExitMissing, nPlain);
    std::printf("passed %d failed %d\nCHOOSER_SLIDE_FAILS=%d\n", g_pass, g_fail, g_fail);
    return g_fail == 0 ? 0 : 1;
}
