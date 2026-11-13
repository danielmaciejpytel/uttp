// UTPP motion matching runtime - data-driven chooser table evaluation (UE 5.8 Chooser plugin semantics). Header-only.
//
// The code knows only the STRUCTURE of chooser tables; every table, threshold, database name and rule is loaded from a
// data file ("utpp-chooser-table" v1 JSON, written by Source/tools/mmbake/export_chooser_table.py or next to a baked
// database by mmbake_motion.py). motion tables are motion-derived data.
//
// Semantics (port of Source/tools/mmbake/motion_chooser_ref.py, which cites C = UE_5.8\Engine\Plugins\Chooser\Source\
// Chooser):
//   * UChooserTable::EvaluateChooser (C\Private\Chooser.cpp:592-825): candidate rows = non-disabled rows in row order;
//     every enabled filter column narrows the list in column order; each surviving row calls ChooseMulti on its result;
//     a row succeeds when the status is not Failed; Stop ends the whole evaluation; the fallback result is evaluated
//     only when no row succeeded, and the table then still returns Failed (824).
//   * Results: database (null asset = Failed), nested / evaluated table (null table = Failed). Invalid results are
//     skipped (they neither succeed nor fail the row loop).
//   * Modes: AllResults (callback returns Continue - union of every matching row, in UE callback order) and
//     FirstResult (callback returns Stop).
//   * FFloatRangeColumn (FloatRangeColumn.cpp:55-114): closed range (bNoMin || v >= Min) && (bNoMax || v <= Max), Min /
//     Max stored as float32 and compared in double; wrap mode FMath::Wrap + reversed-range OR test. The input is read
//     according to the bound property type: float (rounded to float32), double, int (whole numbers only).
//   * FBoolColumn: MatchAny or value == (cell == MatchTrue). FEnumColumn: MatchEqual / MatchNotEqual / MatchAny on the
//     uint8 value (unknown comparison = no match).
//   * A row beyond a column's cell array is filtered out; an unreadable input (not set) passes every row through.
//
// Allocates only in Load(); Evaluate() is allocation-free (fixed-size output and stacks).
#pragma once

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <string_view>
#include <vector>

#include "MiniJson.hpp"

namespace utpp::mm::chooser
{
inline constexpr uint32_t kMaxInputs = 32;
inline constexpr uint32_t kMaxResults = 64;  // databases per evaluation
inline constexpr uint32_t kMaxDepth = 16;    // nested table depth
inline constexpr uint32_t kMaxRows = 256;    // rows per table
inline constexpr uint32_t kMaxTables = 256;
inline constexpr uint32_t kMaxDatabases = 1024;

enum class InputKind : uint8_t
{
    Float,
    Bool,
    Enum
};
enum class PropType : uint8_t
{
    Float,
    Double,
    Int,
    Bool,
    Enum
};
enum class Status : uint8_t
{
    Continue,
    Stop,
    Failed
};
enum class Mode : uint8_t
{
    AllResults,
    FirstResult
};
enum class BoolCell : uint8_t
{
    MatchFalse,
    MatchTrue,
    MatchAny
};
enum class EnumCmp : uint8_t
{
    MatchEqual,
    MatchNotEqual,
    MatchAny,
    Unknown
};
enum class ResultKind : uint8_t
{
    Invalid,
    Database,
    Table
};
enum class InterruptMode : uint8_t
{
    DoNotInterrupt,
    InterruptOnDatabaseChange
};

struct EnumEntry
{
    int value = 0;
    std::string name;    // internal enumerator name
    std::string display; // display name
};

struct InputDef
{
    std::string key; // binding chain "A.B"
    std::vector<std::string> display;
    InputKind kind = InputKind::Float;
    PropType prop = PropType::Double;
    std::string enumName;
    std::vector<EnumEntry> enumEntries;
};

struct FloatCell
{
    float mn = 0.0f;
    float mx = 0.0f;
    bool noMin = false;
    bool noMax = false;
};

struct EnumCell
{
    EnumCmp cmp = EnumCmp::MatchEqual;
    int value = 0;
};

struct ColumnDef
{
    int input = -1;
    InputKind kind = InputKind::Float;
    bool disabled = false;
    bool wrap = false;
    double minValue = -180.0;
    double maxValue = 180.0;
    std::vector<FloatCell> f;
    std::vector<BoolCell> b;
    std::vector<EnumCell> e;
    size_t NumCells() const { return kind == InputKind::Float ? f.size() : (kind == InputKind::Bool ? b.size() : e.size()); }
};

struct ResultDef
{
    ResultKind kind = ResultKind::Invalid;
    int target = -1; // Database: database id (-1 = null asset); Table: table index (-1 = null table)
};

struct TableDef
{
    std::string label;
    std::vector<ColumnDef> columns;
    std::vector<uint8_t> rowDisabled;
    std::vector<ResultDef> rows;
    bool hasFallback = false;
    ResultDef fallback;
};

struct DatabaseDef
{
    std::string name;
    int dbIndex = -1; // DBAS index in the database file the table was baked for (-1 = none)
};

// Optional slide model of the data file (block "slide", FORMAT.txt chooser slide section v1; written by mmbake_motion.py
// --slide-spec): MovementMode gains the value Sliding, the inputs MovementDirection / MovementMode_LastFrame exist and the
// tables carry the Sliding root row and the exit rows. A file without the block loads exactly as before (present = false).
inline constexpr int kSlideSectionVersion = 1;
struct SlideInfo
{
    bool present = false;
    int version = 0;
    int slidingValue = -1;      // MovementMode value of Sliding (2)
    int modeInput = -1;         // input index of MovementMode
    int directionInput = -1;    // MovementDirection (enum: Forward 0, Other 1)
    int lastModeInput = -1;     // MovementMode_LastFrame (enum, same values as MovementMode)
    int directionForward = 0;
    int directionOther = 1;
    int slideTable = -1;
    std::vector<std::string> databaseNames; // PSD_Slide_FeetOut, ... in section order
    std::vector<int> databaseIds;           // DBAS index in the database file (same order)
};

// Blend-time and interrupt rules of the MM node (data file block mm_rules; optional).
struct BlendTimeRules
{
    bool valid = false;
    int modeInput = -1;
    int groundValue = 0;
    int airValue = 1;
    float groundToGround = 0.0f;
    float landing = 0.0f;
    float airRising = 0.0f;
    float airOther = 0.0f;
    float risingThresholdCmS = 0.0f;
};

struct InterruptRules
{
    bool valid = false;
    int modeInput = -1;
    int groundValue = 0;
    int stateInputs[8] = {};
    uint32_t numStateInputs = 0;
};

// Blend profile of the MM node (WeightFactor: bone duration = blend time / scale; TimeFactor: blend time * scale).
struct BlendProfileEntry
{
    std::string bone;   // source (UE) bone name
    std::string cpBone; // mapped Cyberpunk bone name ("" = not mapped)
    float scale = 1.0f;
};
struct BlendProfile
{
    bool valid = false;
    bool weightFactor = true;
    std::string name;
    std::vector<BlendProfileEntry> entries;
};

// Input vector. Unset inputs are "unreadable" (UE passthrough). Values are stored as read by the column type: float
// inputs after the property-type conversion, bool as 0/1, enum as the uint8 value.
struct Inputs
{
    uint32_t count = 0;
    bool readable[kMaxInputs] = {};
    double value[kMaxInputs] = {};
};

struct PathStep
{
    uint16_t table;
    int16_t row; // -1 = fallback
};

// Called for each emitted database (in emission order) with the table/row path that produced it.
using TraceFn = void (*)(void* aUser, int aDatabase, const PathStep* aPath, uint32_t aDepth);

struct Output
{
    uint32_t count = 0;
    int db[kMaxResults] = {}; // database ids in emission order
    Status status = Status::Failed;
    bool fallbackUsed = false; // some table's fallback was evaluated
    bool overflow = false;     // more than kMaxResults databases (extra ones dropped)
    bool depthExceeded = false;

    // Bit mask over database ids < 64.
    uint64_t Mask() const
    {
        uint64_t m = 0;
        for (uint32_t i = 0; i < count; ++i)
            if (db[i] >= 0 && db[i] < 64)
                m |= uint64_t(1) << db[i];
        return m;
    }
};

namespace detail
{
inline std::string NormKey(std::string_view s)
{
    std::string out;
    for (char c : s)
    {
        if (c == ' ' || c == '_' || c == '\t')
            continue;
        out.push_back((c >= 'A' && c <= 'Z') ? static_cast<char>(c - 'A' + 'a') : c);
    }
    return out;
}

// FMath::Wrap for floating point (Core\Public\Math\UnrealMathUtility.h:610-634), as sourcepack_chooser_ref.ue_wrap.
inline double UeWrap(double x, double mn, double mx)
{
    const double size = mx - mn;
    if (size == 0.0)
        return mx;
    double end = x;
    if (end < mn)
    {
        const double mod = std::fmod(mn - end, size);
        end = mod != 0.0 ? mx - mod : mn;
    }
    else if (end > mx)
    {
        const double mod = std::fmod(end - mx, size);
        end = mod != 0.0 ? mn + mod : mx;
    }
    return end;
}
} // namespace detail

class Table
{
public:
    // Loads a "utpp-chooser-table" v1 JSON document. On failure the table is left empty and aError is set.
    bool Load(std::string_view aJson, std::string& aError)
    {
        Clear();
        json::Value doc;
        if (!json::Parse(aJson, doc, aError))
            return false;
        if (!LoadDoc(doc, aError))
        {
            Clear();
            return false;
        }
        return true;
    }

    bool LoadFile(const char* aPath, std::string& aError)
    {
        std::string text;
        FILE* f = nullptr;
        if (fopen_s(&f, aPath, "rb") != 0 || f == nullptr)
        {
            aError = std::string("cannot open ") + aPath;
            return false;
        }
        char buf[65536];
        size_t n;
        while ((n = fread(buf, 1, sizeof(buf), f)) > 0)
            text.append(buf, n);
        fclose(f);
        return Load(text, aError);
    }

    void Clear()
    {
        m_inputs.clear();
        m_tables.clear();
        m_dbs.clear();
        m_blend = {};
        m_interrupt = {};
        m_profile = {};
        m_slide = {};
        m_loaded = false;
    }

    bool Loaded() const { return m_loaded; }
    uint32_t NumInputs() const { return static_cast<uint32_t>(m_inputs.size()); }
    uint32_t NumTables() const { return static_cast<uint32_t>(m_tables.size()); }
    uint32_t NumDatabases() const { return static_cast<uint32_t>(m_dbs.size()); }
    const InputDef& GetInput(uint32_t i) const { return m_inputs[i]; }
    const TableDef& GetTable(uint32_t i) const { return m_tables[i]; }
    const DatabaseDef& GetDatabase(uint32_t i) const { return m_dbs[i]; }
    const BlendTimeRules& GetBlendTimeRules() const { return m_blend; }
    const InterruptRules& GetInterruptRules() const { return m_interrupt; }
    const BlendProfile& GetBlendProfile() const { return m_profile; }
    const SlideInfo& GetSlide() const { return m_slide; }

    // Input by binding key, last binding segment or display name (case, space and underscore insensitive). -1 = none,
    // -2 = ambiguous.
    int FindInput(std::string_view aName) const
    {
        const std::string k = detail::NormKey(aName);
        int hit = -1;
        for (uint32_t i = 0; i < m_inputs.size(); ++i)
        {
            const InputDef& in = m_inputs[i];
            bool match = detail::NormKey(in.key) == k;
            const size_t dot = in.key.rfind('.');
            if (!match && dot != std::string::npos)
                match = detail::NormKey(std::string_view(in.key).substr(dot + 1)) == k;
            for (const auto& d : in.display)
                match = match || detail::NormKey(d) == k;
            if (match)
            {
                if (hit >= 0 && hit != static_cast<int>(i))
                    return -2;
                hit = static_cast<int>(i);
            }
        }
        return hit;
    }

    // Enum value of an enum input by display name, internal name, "Type::Internal (Display)" or a decimal number.
    // -1 = unknown.
    int FindEnumValue(int aInput, std::string_view aName) const
    {
        if (aInput < 0 || aInput >= static_cast<int>(m_inputs.size()) || m_inputs[aInput].kind != InputKind::Enum)
            return -1;
        const InputDef& in = m_inputs[aInput];
        std::string_view s = aName;
        if (const size_t p = s.find("::"); p != std::string_view::npos)
            s = s.substr(p + 2);
        std::string_view disp;
        if (const size_t p = s.find(" ("); p != std::string_view::npos && s.back() == ')')
        {
            disp = s.substr(p + 2, s.size() - p - 3);
            s = s.substr(0, p);
        }
        bool numeric = !s.empty();
        int num = 0;
        for (char c : s)
        {
            if (c < '0' || c > '9' || num > 255)
            {
                numeric = false;
                break;
            }
            num = num * 10 + (c - '0');
        }
        if (numeric)
            return num <= 255 ? num : -1;
        for (const auto& e : in.enumEntries)
            if (e.name == s)
                return e.value;
        const std::string ks = detail::NormKey(s);
        const std::string kd = detail::NormKey(disp);
        for (const auto& e : in.enumEntries)
            if (!e.display.empty() && (detail::NormKey(e.display) == ks || (!kd.empty() && detail::NormKey(e.display) == kd)))
                return e.value;
        return -1;
    }

    int FindDatabase(std::string_view aName) const
    {
        for (uint32_t i = 0; i < m_dbs.size(); ++i)
            if (m_dbs[i].name == aName)
                return static_cast<int>(i);
        return -1;
    }

    // ---- input vector helpers (return false when the value cannot be stored for that input)

    void ResetInputs(Inputs& aIn) const
    {
        aIn.count = static_cast<uint32_t>(m_inputs.size());
        for (uint32_t i = 0; i < kMaxInputs; ++i)
        {
            aIn.readable[i] = false;
            aIn.value[i] = 0.0;
        }
    }

    bool SetNumber(Inputs& aIn, int aInput, double aValue) const
    {
        if (aInput < 0 || aInput >= static_cast<int>(m_inputs.size()) || m_inputs[aInput].kind != InputKind::Float)
            return false;
        double v = aValue;
        switch (m_inputs[aInput].prop)
        {
        case PropType::Float:
            v = static_cast<double>(static_cast<float>(aValue));
            break;
        case PropType::Int:
            if (!(aValue == std::floor(aValue)) || aValue < -2147483648.0 || aValue >= 2147483648.0)
                return false;
            break;
        case PropType::Double:
            break;
        default:
            return false;
        }
        aIn.readable[aInput] = true;
        aIn.value[aInput] = v;
        return true;
    }

    bool SetBool(Inputs& aIn, int aInput, bool aValue) const
    {
        if (aInput < 0 || aInput >= static_cast<int>(m_inputs.size()) || m_inputs[aInput].kind != InputKind::Bool)
            return false;
        aIn.readable[aInput] = true;
        aIn.value[aInput] = aValue ? 1.0 : 0.0;
        return true;
    }

    bool SetEnum(Inputs& aIn, int aInput, int aValue) const
    {
        if (aInput < 0 || aInput >= static_cast<int>(m_inputs.size()) || m_inputs[aInput].kind != InputKind::Enum ||
            aValue < 0 || aValue > 255)
            return false;
        aIn.readable[aInput] = true;
        aIn.value[aInput] = static_cast<double>(aValue);
        return true;
    }

    void SetUnreadable(Inputs& aIn, int aInput) const
    {
        if (aInput >= 0 && aInput < static_cast<int>(kMaxInputs))
            aIn.readable[aInput] = false;
    }

    // ---- evaluation (port of UChooserTable::EvaluateChooser; root = tables[0])

    Status Evaluate(const Inputs& aIn, Mode aMode, Output& aOut, TraceFn aTrace = nullptr, void* aUser = nullptr) const
    {
        aOut = Output{};
        if (!m_loaded || m_tables.empty())
        {
            aOut.status = Status::Failed;
            return aOut.status;
        }
        Ctx ctx{aIn, aMode, aOut, aTrace, aUser, {}, 0};
        aOut.status = EvalTable(ctx, 0, 0);
        return aOut.status;
    }

private:
    struct Ctx
    {
        const Inputs& in;
        Mode mode;
        Output& out;
        TraceFn trace;
        void* user;
        PathStep path[kMaxDepth + 1];
        uint32_t depth;
    };

    bool ColumnPasses(const ColumnDef& c, const Inputs& aIn, uint32_t aRow) const
    {
        if (aRow >= c.NumCells())
            return false; // IsValidIndex
        const double v = aIn.value[c.input];
        switch (c.kind)
        {
        case InputKind::Float:
        {
            const FloatCell& cell = c.f[aRow];
            const double mn = static_cast<double>(cell.mn);
            const double mx = static_cast<double>(cell.mx);
            if (c.wrap)
            {
                const double w = detail::UeWrap(v, c.minValue, c.maxValue);
                if (mx < mn)
                    return (cell.noMin || w >= mn) || (cell.noMax || w <= mx);
                return (cell.noMin || w >= mn) && (cell.noMax || w <= mx);
            }
            return (cell.noMin || v >= mn) && (cell.noMax || v <= mx);
        }
        case InputKind::Bool:
        {
            const BoolCell cell = c.b[aRow];
            return cell == BoolCell::MatchAny || ((v != 0.0) == (cell == BoolCell::MatchTrue));
        }
        case InputKind::Enum:
        {
            const EnumCell& cell = c.e[aRow];
            const int iv = static_cast<int>(v);
            switch (cell.cmp)
            {
            case EnumCmp::MatchEqual: return iv == cell.value;
            case EnumCmp::MatchNotEqual: return iv != cell.value;
            case EnumCmp::MatchAny: return true;
            default: return false;
            }
        }
        }
        return false;
    }

    Status Emit(Ctx& c, int aDb) const
    {
        if (c.out.count < kMaxResults)
            c.out.db[c.out.count++] = aDb;
        else
            c.out.overflow = true;
        if (c.trace)
            c.trace(c.user, aDb, c.path, c.depth);
        return c.mode == Mode::AllResults ? Status::Continue : Status::Stop;
    }

    Status ChooseMulti(Ctx& c, const ResultDef& r, uint32_t aDepth) const
    {
        if (r.kind == ResultKind::Database)
            return r.target < 0 ? Status::Failed : Emit(c, r.target);
        if (r.kind == ResultKind::Table)
            return EvalTable(c, r.target, aDepth + 1);
        return Status::Failed;
    }

    Status EvalTable(Ctx& c, int aTable, uint32_t aDepth) const
    {
        if (aTable < 0 || aTable >= static_cast<int>(m_tables.size()))
            return Status::Failed; // Chooser.cpp:597-600
        if (aDepth >= kMaxDepth)
        {
            c.out.depthExceeded = true;
            return Status::Failed;
        }
        const TableDef& t = m_tables[aTable];
        uint16_t idx[kMaxRows];
        uint32_t n = 0;
        for (uint32_t i = 0; i < t.rows.size() && i < kMaxRows; ++i)
            if (!t.rowDisabled[i])
                idx[n++] = static_cast<uint16_t>(i);
        for (const ColumnDef& col : t.columns)
        {
            if (col.disabled)
                continue;
            if (!c.in.readable[col.input])
                continue; // passthrough fallback (unreadable input)
            uint32_t m = 0;
            for (uint32_t k = 0; k < n; ++k)
                if (ColumnPasses(col, c.in, idx[k]))
                    idx[m++] = idx[k];
            n = m;
        }
        bool anyOk = false;
        for (uint32_t k = 0; k < n; ++k)
        {
            const ResultDef& r = t.rows[idx[k]];
            if (r.kind == ResultKind::Invalid)
                continue;
            c.path[aDepth] = PathStep{static_cast<uint16_t>(aTable), static_cast<int16_t>(idx[k])};
            c.depth = aDepth + 1;
            const Status st = ChooseMulti(c, r, aDepth);
            if (st != Status::Failed)
                anyOk = true;
            if (st == Status::Stop)
                return Status::Stop;
        }
        if (!anyOk && t.hasFallback && t.fallback.kind != ResultKind::Invalid)
        {
            c.out.fallbackUsed = true;
            c.path[aDepth] = PathStep{static_cast<uint16_t>(aTable), static_cast<int16_t>(-1)};
            c.depth = aDepth + 1;
            if (ChooseMulti(c, t.fallback, aDepth) == Status::Stop)
                return Status::Stop;
        }
        return anyOk ? Status::Continue : Status::Failed; // Chooser.cpp:824
    }

    // ---- loading

    static bool Fail(std::string& aError, const std::string& aMsg)
    {
        aError = aMsg;
        return false;
    }

    static const json::Value* Member(const json::Value& o, const char* k, json::Type t)
    {
        const json::Value* v = o.Find(k);
        return (v && v->type == t) ? v : nullptr;
    }

    static bool GetBool(const json::Value& o, const char* k, bool aDefault)
    {
        const json::Value* v = o.Find(k);
        return (v && v->IsBool()) ? v->b : aDefault;
    }

    int AddDatabase(const std::string& aName, int aDbIndex)
    {
        for (uint32_t i = 0; i < m_dbs.size(); ++i)
            if (m_dbs[i].name == aName)
                return static_cast<int>(i);
        m_dbs.push_back(DatabaseDef{aName, aDbIndex});
        return static_cast<int>(m_dbs.size() - 1);
    }

    bool ParseResult(const json::Value& r, ResultDef& aOut, size_t aNumTables, std::string& aError)
    {
        aOut = ResultDef{};
        if (r.IsNull())
            return true;
        if (!r.IsObject())
            return Fail(aError, "result is not an object");
        const json::Value* kind = Member(r, "kind", json::Type::String);
        if (!kind)
            return Fail(aError, "result without kind");
        if (kind->str == "database")
        {
            aOut.kind = ResultKind::Database;
            const json::Value* name = r.Find("name");
            const json::Value* db = Member(r, "db", json::Type::Number);
            if (name && name->IsString() && !name->str.empty())
            {
                if (m_dbs.size() >= kMaxDatabases)
                    return Fail(aError, "too many databases");
                aOut.target = AddDatabase(name->str, db ? static_cast<int>(db->num) : -1);
            }
            else
                aOut.target = -1; // null asset
            return true;
        }
        if (kind->str == "table")
        {
            aOut.kind = ResultKind::Table;
            const json::Value* t = Member(r, "table", json::Type::Number);
            if (!t || !t->isInteger)
                return Fail(aError, "table result without table index");
            const int ti = static_cast<int>(t->num);
            if (ti >= static_cast<int>(aNumTables) || ti < -1)
                return Fail(aError, "table result index out of range");
            aOut.target = ti;
            return true;
        }
        if (kind->str == "invalid")
            return true;
        return Fail(aError, "unknown result kind " + kind->str);
    }

    bool LoadDoc(const json::Value& doc, std::string& aError)
    {
        if (!doc.IsObject())
            return Fail(aError, "document is not an object");
        const json::Value* fmt = Member(doc, "format", json::Type::String);
        const json::Value* ver = Member(doc, "version", json::Type::Number);
        if (!fmt || fmt->str != "utpp-chooser-table" || !ver || ver->num != 1.0)
            return Fail(aError, "not a utpp-chooser-table base file");
        const json::Value* inputs = Member(doc, "inputs", json::Type::Array);
        const json::Value* tables = Member(doc, "tables", json::Type::Array);
        if (!inputs || !tables || tables->arr.empty())
            return Fail(aError, "missing inputs / tables");
        if (inputs->arr.size() > kMaxInputs)
            return Fail(aError, "too many inputs");
        if (tables->arr.size() > kMaxTables)
            return Fail(aError, "too many tables");

        for (const json::Value& iv : inputs->arr)
        {
            InputDef in;
            const json::Value* key = Member(iv, "key", json::Type::String);
            const json::Value* kind = Member(iv, "kind", json::Type::String);
            const json::Value* prop = Member(iv, "property_type", json::Type::String);
            if (!key || !kind)
                return Fail(aError, "input without key / kind");
            in.key = key->str;
            if (kind->str == "float")
                in.kind = InputKind::Float;
            else if (kind->str == "bool")
                in.kind = InputKind::Bool;
            else if (kind->str == "enum")
                in.kind = InputKind::Enum;
            else
                return Fail(aError, "input " + in.key + ": unknown kind " + kind->str);
            const std::string p = prop ? prop->str : std::string();
            if (in.kind == InputKind::Float)
            {
                if (p == "float")
                    in.prop = PropType::Float;
                else if (p == "double")
                    in.prop = PropType::Double;
                else if (p == "int")
                    in.prop = PropType::Int;
                else
                    return Fail(aError, "input " + in.key + ": unknown property type " + p);
            }
            else
                in.prop = in.kind == InputKind::Bool ? PropType::Bool : PropType::Enum;
            if (const json::Value* disp = Member(iv, "display", json::Type::Array))
                for (const json::Value& d : disp->arr)
                    if (d.IsString())
                        in.display.push_back(d.str);
            if (in.kind == InputKind::Enum)
            {
                const json::Value* en = Member(iv, "enum", json::Type::Object);
                if (en)
                {
                    if (const json::Value* nm = Member(*en, "name", json::Type::String))
                        in.enumName = nm->str;
                    if (const json::Value* vals = Member(*en, "values", json::Type::Array))
                        for (const json::Value& ev : vals->arr)
                        {
                            EnumEntry e;
                            const json::Value* v = Member(ev, "value", json::Type::Number);
                            if (!v)
                                return Fail(aError, "input " + in.key + ": enum value without number");
                            e.value = static_cast<int>(v->num);
                            if (const json::Value* nm = Member(ev, "name", json::Type::String))
                                e.name = nm->str;
                            if (const json::Value* d = Member(ev, "display", json::Type::String))
                                e.display = d->str;
                            in.enumEntries.push_back(std::move(e));
                        }
                }
            }
            m_inputs.push_back(std::move(in));
        }

        const size_t numTables = tables->arr.size();
        for (size_t ti = 0; ti < numTables; ++ti)
        {
            const json::Value& tv = tables->arr[ti];
            TableDef t;
            if (const json::Value* lbl = Member(tv, "label", json::Type::String))
                t.label = lbl->str;
            const std::string where = "table " + std::to_string(ti) + " (" + t.label + ")";
            const json::Value* cols = Member(tv, "columns", json::Type::Array);
            const json::Value* rows = Member(tv, "rows", json::Type::Array);
            if (!cols || !rows)
                return Fail(aError, where + ": missing columns / rows");
            if (rows->arr.size() > kMaxRows)
                return Fail(aError, where + ": too many rows");
            for (const json::Value& cv : cols->arr)
            {
                ColumnDef c;
                const json::Value* input = Member(cv, "input", json::Type::Number);
                const json::Value* kind = Member(cv, "kind", json::Type::String);
                const json::Value* cells = Member(cv, "cells", json::Type::Array);
                if (!input || !kind || !cells)
                    return Fail(aError, where + ": column without input / kind / cells");
                c.input = static_cast<int>(input->num);
                c.disabled = GetBool(cv, "disabled", false);
                if (c.input < 0 || c.input >= static_cast<int>(m_inputs.size()))
                {
                    if (!c.disabled)
                        return Fail(aError, where + ": column input index out of range");
                    c.input = 0; // never read
                }
                if (kind->str == "float")
                    c.kind = InputKind::Float;
                else if (kind->str == "bool")
                    c.kind = InputKind::Bool;
                else if (kind->str == "enum")
                    c.kind = InputKind::Enum;
                else
                    return Fail(aError, where + ": unsupported column kind " + kind->str);
                if (!c.disabled && m_inputs[c.input].kind != c.kind)
                    return Fail(aError, where + ": column kind differs from its input " + m_inputs[c.input].key);
                if (c.kind == InputKind::Float)
                {
                    c.wrap = GetBool(cv, "wrap", false);
                    if (const json::Value* v = Member(cv, "min_value", json::Type::Number))
                        c.minValue = v->num;
                    if (const json::Value* v = Member(cv, "max_value", json::Type::Number))
                        c.maxValue = v->num;
                    for (const json::Value& cell : cells->arr)
                    {
                        if (!cell.IsArray() || cell.arr.size() != 4 || !cell.arr[0].IsNumber() || !cell.arr[1].IsNumber() ||
                            !cell.arr[2].IsBool() || !cell.arr[3].IsBool())
                            return Fail(aError, where + ": bad float cell");
                        // Bounds are float32 in UE (FloatRangeColumn.h:47-51); the file stores them already rounded.
                        c.f.push_back(FloatCell{static_cast<float>(cell.arr[0].num), static_cast<float>(cell.arr[1].num),
                                                cell.arr[2].b, cell.arr[3].b});
                    }
                }
                else if (c.kind == InputKind::Bool)
                {
                    for (const json::Value& cell : cells->arr)
                    {
                        if (!cell.IsString())
                            return Fail(aError, where + ": bad bool cell");
                        if (cell.str == "MatchFalse")
                            c.b.push_back(BoolCell::MatchFalse);
                        else if (cell.str == "MatchTrue")
                            c.b.push_back(BoolCell::MatchTrue);
                        else if (cell.str == "MatchAny")
                            c.b.push_back(BoolCell::MatchAny);
                        else
                            return Fail(aError, where + ": bad bool cell " + cell.str);
                    }
                }
                else
                {
                    for (const json::Value& cell : cells->arr)
                    {
                        if (!cell.IsArray() || cell.arr.size() != 2 || !cell.arr[0].IsString() || !cell.arr[1].IsNumber())
                            return Fail(aError, where + ": bad enum cell");
                        EnumCell e;
                        const std::string& cmp = cell.arr[0].str;
                        e.cmp = cmp == "MatchEqual"      ? EnumCmp::MatchEqual
                                : cmp == "MatchNotEqual" ? EnumCmp::MatchNotEqual
                                : cmp == "MatchAny"      ? EnumCmp::MatchAny
                                                         : EnumCmp::Unknown;
                        e.value = static_cast<int>(cell.arr[1].num);
                        c.e.push_back(e);
                    }
                }
                t.columns.push_back(std::move(c));
            }
            for (const json::Value& rv : rows->arr)
            {
                if (!rv.IsObject())
                    return Fail(aError, where + ": row is not an object");
                t.rowDisabled.push_back(GetBool(rv, "disabled", false) ? 1 : 0);
                ResultDef r;
                const json::Value* res = rv.Find("result");
                if (res && !ParseResult(*res, r, numTables, aError))
                    return Fail(aError, where + ": " + aError);
                t.rows.push_back(r);
            }
            const json::Value* fb = tv.Find("fallback");
            if (fb && !fb->IsNull())
            {
                t.hasFallback = true;
                if (!ParseResult(*fb, t.fallback, numTables, aError))
                    return Fail(aError, where + " fallback: " + aError);
            }
            m_tables.push_back(std::move(t));
        }

        if (!LoadRules(doc, aError))
            return false;
        if (!LoadSlide(doc, aError))
            return false;
        m_loaded = true;
        return true;
    }

    int InputIndexByKey(const json::Value& o, const char* aMember) const
    {
        const json::Value* k = Member(o, aMember, json::Type::String);
        if (!k)
            return -1;
        for (uint32_t i = 0; i < m_inputs.size(); ++i)
            if (m_inputs[i].key == k->str)
                return static_cast<int>(i);
        return -1;
    }

    static bool Num(const json::Value& o, const char* k, double& aOut)
    {
        const json::Value* v = Member(o, k, json::Type::Number);
        if (!v)
            return false;
        aOut = v->num;
        return true;
    }

    bool LoadSlide(const json::Value& doc, std::string& aError)
    {
        const json::Value* sl = Member(doc, "slide", json::Type::Object);
        if (!sl)
            return true;
        SlideInfo s;
        double v = 0;
        if (!Num(*sl, "version", v) || static_cast<int>(v) != kSlideSectionVersion)
            return Fail(aError, "slide section: unsupported version");
        s.version = static_cast<int>(v);
        s.modeInput = InputIndexByKey(*sl, "movement_mode_input");
        s.directionInput = InputIndexByKey(*sl, "direction_input");
        s.lastModeInput = InputIndexByKey(*sl, "last_mode_input");
        if (s.modeInput < 0 || s.directionInput < 0 || s.lastModeInput < 0 || !Num(*sl, "sliding_value", v))
            return Fail(aError, "slide section: inputs / sliding_value incomplete");
        s.slidingValue = static_cast<int>(v);
        if (const json::Value* dv = Member(*sl, "direction_values", json::Type::Object))
        {
            if (const json::Value* f = Member(*dv, "Forward", json::Type::Number))
                s.directionForward = static_cast<int>(f->num);
            if (const json::Value* o = Member(*dv, "Other", json::Type::Number))
                s.directionOther = static_cast<int>(o->num);
        }
        if (!Num(*sl, "slide_table", v) || v < 0 || static_cast<size_t>(v) >= m_tables.size())
            return Fail(aError, "slide section: slide_table out of range");
        s.slideTable = static_cast<int>(v);
        if (const json::Value* dbs = Member(*sl, "databases", json::Type::Object))
            for (const auto& kv : dbs->obj)
            {
                if (!kv.second.IsNumber())
                    return Fail(aError, "slide section: database id is not a number");
                s.databaseNames.push_back(kv.first);
                s.databaseIds.push_back(static_cast<int>(kv.second.num));
            }
        s.present = true;
        m_slide = std::move(s);
        return true;
    }

    bool LoadRules(const json::Value& doc, std::string& aError)
    {
        if (const json::Value* rules = Member(doc, "mm_rules", json::Type::Object))
        {
            if (const json::Value* bt = Member(*rules, "blend_time", json::Type::Object))
            {
                BlendTimeRules r;
                double g = 0, a = 0, gg = 0, l = 0, ar = 0, ao = 0, th = 0;
                r.modeInput = InputIndexByKey(*bt, "mode_input");
                if (r.modeInput < 0 || m_inputs[r.modeInput].kind != InputKind::Enum || !Num(*bt, "ground_value", g) ||
                    !Num(*bt, "air_value", a) || !Num(*bt, "ground_to_ground_s", gg) || !Num(*bt, "landing_s", l) ||
                    !Num(*bt, "air_rising_s", ar) || !Num(*bt, "air_other_s", ao) || !Num(*bt, "rising_threshold_cm_s", th))
                    return Fail(aError, "mm_rules.blend_time incomplete");
                r.groundValue = static_cast<int>(g);
                r.airValue = static_cast<int>(a);
                r.groundToGround = static_cast<float>(gg);
                r.landing = static_cast<float>(l);
                r.airRising = static_cast<float>(ar);
                r.airOther = static_cast<float>(ao);
                r.risingThresholdCmS = static_cast<float>(th);
                r.valid = true;
                m_blend = r;
            }
            if (const json::Value* ir = Member(*rules, "interrupt", json::Type::Object))
            {
                InterruptRules r;
                double g = 0;
                r.modeInput = InputIndexByKey(*ir, "mode_input");
                const json::Value* st = Member(*ir, "ground_state_inputs", json::Type::Array);
                if (r.modeInput < 0 || !Num(*ir, "ground_value", g) || !st || st->arr.size() > 8)
                    return Fail(aError, "mm_rules.interrupt incomplete");
                r.groundValue = static_cast<int>(g);
                for (const json::Value& s : st->arr)
                {
                    int idx = -1;
                    for (uint32_t i = 0; i < m_inputs.size(); ++i)
                        if (s.IsString() && m_inputs[i].key == s.str)
                            idx = static_cast<int>(i);
                    if (idx < 0)
                        return Fail(aError, "mm_rules.interrupt: unknown state input");
                    r.stateInputs[r.numStateInputs++] = idx;
                }
                r.valid = true;
                m_interrupt = r;
            }
        }
        if (const json::Value* bp = Member(doc, "blend_profile", json::Type::Object))
        {
            BlendProfile p;
            if (const json::Value* nm = Member(*bp, "name", json::Type::String))
                p.name = nm->str;
            const json::Value* mode = Member(*bp, "mode", json::Type::String);
            if (!mode || (mode->str != "WeightFactor" && mode->str != "TimeFactor"))
                return Fail(aError, "blend_profile: unknown mode");
            p.weightFactor = mode->str == "WeightFactor";
            if (const json::Value* es = Member(*bp, "entries", json::Type::Array))
                for (const json::Value& e : es->arr)
                {
                    BlendProfileEntry be;
                    const json::Value* b = Member(e, "bone", json::Type::String);
                    const json::Value* s = Member(e, "scale", json::Type::Number);
                    if (!b || !s)
                        return Fail(aError, "blend_profile: bad entry");
                    be.bone = b->str;
                    be.scale = static_cast<float>(s->num);
                    if (const json::Value* cp = Member(e, "cp", json::Type::String))
                        be.cpBone = cp->str;
                    p.entries.push_back(std::move(be));
                }
            p.valid = true;
            m_profile = std::move(p);
        }
        return true;
    }

    std::vector<InputDef> m_inputs;
    std::vector<TableDef> m_tables;
    std::vector<DatabaseDef> m_dbs;
    BlendTimeRules m_blend;
    InterruptRules m_interrupt;
    BlendProfile m_profile;
    SlideInfo m_slide;
    bool m_loaded = false;
};

// ---- MM node rules (data-driven; see export_chooser_table.py mm_rules)

// Blend time for the current frame from the movement-mode input of this and the last frame and the vertical velocity
// (cm/s, up positive). Returns aDefault when the rules are missing or the combination is not covered.
inline float GetBlendTime(const BlendTimeRules& r, int aMode, int aLastMode, float aVerticalVelocityCmS, float aDefault)
{
    if (!r.valid)
        return aDefault;
    if (aMode == r.groundValue)
    {
        if (aLastMode == r.groundValue)
            return r.groundToGround;
        if (aLastMode == r.airValue)
            return r.landing;
        return aDefault;
    }
    if (aMode == r.airValue)
        return aVerticalVelocityCmS > r.risingThresholdCmS ? r.airRising : r.airOther;
    return aDefault;
}

// Interrupt mode from the inputs of this and the last frame (enum values as stored in Inputs::value).
inline InterruptMode GetInterruptMode(const InterruptRules& r, const Inputs& aNow, const Inputs& aLast)
{
    if (!r.valid)
        return InterruptMode::DoNotInterrupt;
    const int mode = static_cast<int>(aNow.value[r.modeInput]);
    if (mode != static_cast<int>(aLast.value[r.modeInput]))
        return InterruptMode::InterruptOnDatabaseChange;
    if (mode != r.groundValue)
        return InterruptMode::DoNotInterrupt;
    for (uint32_t i = 0; i < r.numStateInputs; ++i)
        if (aNow.value[r.stateInputs[i]] != aLast.value[r.stateInputs[i]])
            return InterruptMode::InterruptOnDatabaseChange;
    return InterruptMode::DoNotInterrupt;
}

} // namespace utpp::mm::chooser
