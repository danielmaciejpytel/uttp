// Offline self-test of the .utppmm VERSION 2 reader (src/mm/MotionDatabase.hpp, stage 10.1 "utpp_mmdb_selftest v2").
//
//   utpp_motion_db_test <db.utppmm> [summary.json] [dump.txt] [woman rig.json] [man rig.json]
//
// 1. Load on the woman rig identity; refused on the man rig identity (fail closed on both rigs).
// 2. Rig identity: header == hash from BONE + RIGR (loader) == hash from the woman rig json; the man rig json gives
//    another identity; RIGR equals the rig json rest transforms.
// 3. Corrupted copies are refused: header CRC, payload CRC, truncation, version, RIGR edit (CRCs fixed), DROW frame
//    out of range (CRCs fixed), DBAS min_addend edit (CRCs fixed).
// 4. Counts equal the bake summary json (frames, clips, databases, entries, rows, events, size, rig hash, frame flag
//    counts, schemas, norms, RTZ clip counts, per clip / per database records when present).
// 5. Python dump (Source/tools/mmbake/dump_utppmm_motion.py, independent reader): header fields, independent CRC / rig
//    checks, flag counts, 20 random DFEA rows and the RTZ_ rows bit-exact.
// 6. RTZ_ values: vz = central difference of root_z x fps for RTZ_PRESENT clips; no RTZ_UNKNOWN clip, no refused or
//    RTZ_PENDING database.
// 7. Reference search: AABB result == linear scan (same row, bitwise-equal cost) on queries near real rows.
// The database is motion-derived (outside git); the test prints numbers only.

#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

#include "../src/mm/MotionDatabase.hpp"
#include "../src/mm/MiniJson.hpp"

using namespace utpp::mm;
using namespace utpp::mm::motion;
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

static bool ReadBytes(const char* aPath, std::vector<uint8_t>& aOut)
{
    FILE* f = nullptr;
    if (fopen_s(&f, aPath, "rb") != 0 || !f)
        return false;
    _fseeki64(f, 0, SEEK_END);
    const long long n = _ftelli64(f);
    _fseeki64(f, 0, SEEK_SET);
    aOut.resize(static_cast<size_t>(n));
    const size_t got = n > 0 ? fread(aOut.data(), 1, aOut.size(), f) : 0;
    fclose(f);
    return got == aOut.size();
}

static bool ReadText(const char* aPath, std::string& aOut)
{
    std::vector<uint8_t> b;
    if (!ReadBytes(aPath, b))
        return false;
    aOut.assign(b.begin(), b.end());
    return true;
}

static uint32_t Bits(float f)
{
    uint32_t u;
    std::memcpy(&u, &f, 4);
    return u;
}

// ---- in-memory corruption helpers

static void FixCrcs(std::vector<uint8_t>& b)
{
    MotionHeader h;
    std::memcpy(&h, b.data(), sizeof(h));
    h.fileSize = b.size();
    h.payloadCrc32 = Crc32(b.data() + kHeaderSize, b.size() - kHeaderSize);
    h.headerCrc32 = 0;
    h.headerCrc32 = Crc32(reinterpret_cast<const uint8_t*>(&h), kHeaderSize);
    std::memcpy(b.data(), &h, sizeof(h));
}

static bool SectionOffset(const std::vector<uint8_t>& b, const char* aId, uint64_t& aOff, uint32_t& aCount)
{
    MotionHeader h;
    std::memcpy(&h, b.data(), sizeof(h));
    for (uint32_t i = 0; i < h.numSections; ++i)
    {
        uint32_t id, count;
        uint64_t off;
        std::memcpy(&id, b.data() + kHeaderSize + 32 * i, 4);
        std::memcpy(&count, b.data() + kHeaderSize + 32 * i + 4, 4);
        std::memcpy(&off, b.data() + kHeaderSize + 32 * i + 16, 8);
        if (std::memcmp(&id, aId, 4) == 0)
        {
            aOff = off;
            aCount = count;
            return true;
        }
    }
    return false;
}

static std::string LoadError(const std::vector<uint8_t>& b, uint64_t aRig)
{
    MotionDatabase db;
    std::string err;
    if (db.LoadFromMemory(b, aRig, err))
        return "";
    return err.empty() ? "?" : err;
}

// ---- rig json (WolvenKit animRig export) -> identity

static bool RigIdentityFromJson(const char* aPath, uint32_t aNumBones, uint64_t& aOut, std::vector<float>& aRest)
{
    std::string text, err;
    if (!ReadText(aPath, text))
        return false;
    json::Value doc;
    if (!json::Parse(text, doc, err))
        return false;
    const json::Value* rc = doc.Find("Data");
    rc = rc ? rc->Find("RootChunk") : nullptr;
    const json::Value* names = rc ? rc->Find("boneNames") : nullptr;
    const json::Value* parents = rc ? rc->Find("boneParentIndexes") : nullptr;
    const json::Value* xf = rc ? rc->Find("boneTransforms") : nullptr;
    if (!names || !parents || !xf || names->arr.size() < aNumBones || parents->arr.size() < aNumBones || xf->arr.size() < aNumBones)
        return false;
    std::vector<std::string> nm(aNumBones);
    std::vector<std::string_view> nv(aNumBones);
    std::vector<int32_t> pa(aNumBones);
    aRest.assign(static_cast<size_t>(aNumBones) * 7, 0.0f);
    auto num = [](const json::Value* v) -> double {
        if (!v)
            return NAN;
        if (v->IsNumber())
            return v->num;
        const json::Value* x = v->Find("$value");
        return (x && x->IsNumber()) ? x->num : NAN;
    };
    for (uint32_t i = 0; i < aNumBones; ++i)
    {
        const json::Value* s = names->arr[i].Find("$value");
        nm[i] = s && s->IsString() ? s->str : std::string();
        nv[i] = nm[i];
        pa[i] = static_cast<int32_t>(num(&parents->arr[i]));
        const json::Value* t = xf->arr[i].Find("Translation");
        const json::Value* r = xf->arr[i].Find("Rotation");
        if (!t || !r)
            return false;
        const double vals[7] = {num(t->Find("X")), num(t->Find("Y")), num(t->Find("Z")), num(r->Find("i")),
                                num(r->Find("j")), num(r->Find("k")), num(r->Find("r"))};
        for (int k = 0; k < 7; ++k)
            aRest[i * 7 + k] = static_cast<float>(vals[k]);
    }
    std::vector<float> xyz(static_cast<size_t>(aNumBones) * 3);
    for (uint32_t i = 0; i < aNumBones; ++i)
        for (int k = 0; k < 3; ++k)
            xyz[i * 3 + k] = aRest[i * 7 + k];
    aOut = ComputeRigIdentity(nv, pa, xyz);
    return true;
}

// ------------------------------------------------------------------------------------------------ tests

static void TestRig(const MotionDatabase& db, const std::vector<uint8_t>& bytes, const char* aWomanRig, const char* aManRig)
{
    std::string err;
    MotionDatabase other;
    Check(!other.LoadFromMemory(bytes, kRigIdentityMan, err) && err.find("rig identity") != std::string::npos,
          "refused on the man rig identity: " + err);
    Check(db.RigIdentity() == kRigIdentityWoman, "header rig identity = player_woman_skeleton identity");
    Check(db.TopologyHash() == 0x72CB314EB01F562Dull, "base topology hash from BONE = 0x72CB314EB01F562D");
    if (aWomanRig)
    {
        uint64_t idW = 0;
        std::vector<float> restW;
        const bool ok = RigIdentityFromJson(aWomanRig, db.NumBones(), idW, restW);
        Check(ok, std::string("woman rig json readable: ") + aWomanRig);
        if (ok)
        {
            std::printf("rig identity: header 0x%016" PRIx64 ", woman rig json 0x%016" PRIx64 "\n", db.RigIdentity(), idW);
            Check(idW == db.RigIdentity(), "identity from the woman rig json == header");
            uint32_t same = 0;
            for (uint32_t i = 0; i < db.NumBones(); ++i)
            {
                const RigRestRecord& r = db.RigRest(i);
                const float v[7] = {r.px, r.py, r.pz, r.qx, r.qy, r.qz, r.qw};
                bool eq = true;
                for (int k = 0; k < 7; ++k)
                    eq = eq && Bits(v[k]) == Bits(restW[i * 7 + k]);
                same += eq ? 1 : 0;
            }
            std::printf("RIGR == woman rig json rest transforms (float32 bits): %u/%u bones\n", same, db.NumBones());
            Check(same == db.NumBones(), "RIGR equals the woman rig json rest transforms");
        }
    }
    if (aManRig)
    {
        uint64_t idM = 0;
        std::vector<float> restM;
        const bool ok = RigIdentityFromJson(aManRig, db.NumBones(), idM, restM);
        Check(ok, std::string("man rig json readable: ") + aManRig);
        if (ok)
        {
            std::printf("rig identity: man rig json 0x%016" PRIx64 "\n", idM);
            Check(idM == kRigIdentityMan && idM != db.RigIdentity(), "man rig json identity = kRigIdentityMan != header");
        }
    }
}

static void TestCorruption(const std::vector<uint8_t>& bytes)
{
    const uint64_t rig = kRigIdentityWoman;
    {
        auto b = bytes;
        b[40] ^= 0x01; // num_bones byte, header CRC not fixed
        const std::string e = LoadError(b, rig);
        Check(e.find("header CRC") != std::string::npos, "header byte flip -> " + e);
    }
    {
        auto b = bytes;
        b[b.size() / 2] ^= 0x10;
        const std::string e = LoadError(b, rig);
        Check(e.find("payload CRC") != std::string::npos, "payload byte flip -> " + e);
    }
    {
        auto b = bytes;
        b.resize(b.size() - 16);
        const std::string e = LoadError(b, rig);
        Check(e.find("size") != std::string::npos, "truncated file -> " + e);
    }
    {
        auto b = bytes;
        const uint32_t v1 = 1;
        std::memcpy(b.data() + 8, &v1, 4);
        FixCrcs(b);
        const std::string e = LoadError(b, rig);
        Check(e.find("version") != std::string::npos, "version 1 header -> " + e);
    }
    uint64_t off = 0;
    uint32_t count = 0;
    if (SectionOffset(bytes, "RIGR", off, count) && count > 5)
    {
        auto b = bytes;
        float x;
        std::memcpy(&x, b.data() + off + 28 * 5, 4); // bone 5 rest x
        x += 0.002f;
        std::memcpy(b.data() + off + 28 * 5, &x, 4);
        FixCrcs(b);
        const std::string e = LoadError(b, rig);
        Check(e.find("RIGR") != std::string::npos, "RIGR translation +2 mm (CRCs fixed) -> " + e);
    }
    if (SectionOffset(bytes, "DROW", off, count) && count > 0)
    {
        auto b = bytes;
        const uint32_t big = 0x7FFFFFFF;
        std::memcpy(b.data() + off + 20 * (count / 2), &big, 4);
        FixCrcs(b);
        const std::string e = LoadError(b, rig);
        Check(e.find("row") != std::string::npos, "DROW frame out of range (CRCs fixed) -> " + e);
    }
    if (SectionOffset(bytes, "DBAS", off, count) && count > 0)
    {
        auto b = bytes;
        float m;
        std::memcpy(&m, b.data() + off + 88, 4);
        m -= 1.0f;
        std::memcpy(b.data() + off + 88, &m, 4);
        FixCrcs(b);
        const std::string e = LoadError(b, rig);
        Check(e.find("min addend") != std::string::npos, "DBAS min_addend edit (CRCs fixed) -> " + e);
    }
    {
        auto b = bytes;
        FixCrcs(b);
        Check(LoadError(b, rig).empty(), "unchanged copy with recomputed CRCs still loads (negative control)");
    }
}

struct DerivedCounts
{
    uint64_t frames[10] = {};
    uint32_t rtzPresent = 0, rtzUnknown = 0, rtzNone = 0, pending = 0;
};

static DerivedCounts Derive(const MotionDatabase& db)
{
    DerivedCounts c;
    for (uint32_t f = 0; f < db.NumFrames(); ++f)
        for (int k = 0; k < 10; ++k)
            if (db.FrameFlags(f) & (1u << k))
                ++c.frames[k];
    for (uint32_t i = 0; i < db.NumClips(); ++i)
    {
        const uint32_t fl = db.Clip(i).flags;
        c.rtzPresent += (fl & kClipRtzPresent) ? 1 : 0;
        c.rtzUnknown += (fl & kClipRtzUnknown) ? 1 : 0;
        c.rtzNone += (fl & kClipRtzNone) ? 1 : 0;
    }
    for (uint32_t d = 0; d < db.NumDatabases(); ++d)
        c.pending += (db.Db(d).flags & kDbRtzPending) ? 1 : 0;
    return c;
}

static void TestSummary(const MotionDatabase& db, size_t aFileSize, const char* aPath)
{
    std::string text, err;
    if (!ReadText(aPath, text))
    {
        Check(false, std::string("cannot read summary ") + aPath);
        return;
    }
    json::Value s;
    if (!json::Parse(text, s, err))
    {
        Check(false, "summary JSON: " + err);
        return;
    }
    const MotionHeader& h = db.Header();
    const DerivedCounts c = Derive(db);
    int compared = 0, equal = 0;
    auto num = [&](const char* k, double v) {
        const json::Value* x = s.Find(k);
        ++compared;
        const bool ok = x && x->IsNumber() && x->num == v;
        equal += ok ? 1 : 0;
        if (!ok)
            std::printf("  summary %s = %.17g, database %.17g\n", k, x && x->IsNumber() ? x->num : NAN, v);
    };
    num("size_bytes", static_cast<double>(aFileSize));
    num("num_frames", h.numFrames);
    num("num_clips", h.numClips);
    num("num_databases", h.numDatabases);
    num("num_entries", h.numEntries);
    num("num_rows", h.numRows);
    num("num_events", h.numEvents);
    num("fps", h.fps);
    num("frames_selectable", static_cast<double>(c.frames[0]));
    num("frames_excluded_notify", static_cast<double>(c.frames[1]));
    num("frames_block_transition", static_cast<double>(c.frames[4]));
    num("frames_excluded_tail", static_cast<double>(c.frames[8]));
    num("frames_indexed", static_cast<double>(c.frames[9]));
    {
        const json::Value* x = s.Find("has_scale");
        ++compared;
        equal += (x && x->IsBool() && x->b == db.HasScale()) ? 1 : 0;
        const json::Value* rh = s.Find("rig_hash");
        ++compared;
        const bool ok = rh && rh->IsString() && std::strtoull(rh->str.c_str(), nullptr, 16) == h.rigHash;
        equal += ok ? 1 : 0;
        if (!ok)
            std::printf("  summary rig_hash %s, header 0x%016" PRIx64 "\n", rh && rh->IsString() ? rh->str.c_str() : "?", h.rigHash);
        const json::Value* sc = s.Find("schemas");
        const json::Value* nr = s.Find("norms");
        compared += 2;
        equal += (sc && sc->IsArray() && sc->arr.size() == h.numSchemas) ? 1 : 0;
        equal += (nr && nr->IsArray() && nr->arr.size() == h.numNorms) ? 1 : 0;
    }
    if (const json::Value* rtz = s.Find("rtz"))
    {
        const json::Value* p = rtz->Find("clips_root_z");
        const json::Value* u = rtz->Find("clips_unknown");
        const json::Value* n = rtz->Find("clips_none");
        compared += 3;
        equal += (p && p->IsNumber() && p->num == c.rtzPresent) ? 1 : 0;
        equal += (u && u->IsArray() && u->arr.size() == c.rtzUnknown) ? 1 : 0;
        equal += (n && n->IsNumber() && n->num == c.rtzNone) ? 1 : 0;
    }
    // per clip
    uint32_t clipsOk = 0, clipsN = 0;
    if (const json::Value* cl = s.Find("clips"); cl && cl->IsArray())
    {
        clipsN = static_cast<uint32_t>(cl->arr.size());
        for (uint32_t i = 0; i < clipsN && i < db.NumClips(); ++i)
        {
            const json::Value& e = cl->arr[i];
            const json::Value* nm = e.Find("name");
            const json::Value* ff = e.Find("first_frame");
            const json::Value* nf = e.Find("num_frames");
            const json::Value* sel = e.Find("selectable");
            const MotionClipRecord& r = db.Clip(i);
            const bool ok = nm && nm->IsString() && nm->str == db.ClipName(i) && ff && ff->num == r.firstFrame && nf &&
                            nf->num == r.numFrames && (!sel || sel->num == r.numSelectable);
            clipsOk += ok ? 1 : 0;
        }
    }
    compared += 1;
    equal += (clipsN == db.NumClips() && clipsOk == clipsN) ? 1 : 0;
    // per database (keys compared when present)
    uint32_t dbOk = 0, dbN = 0, dbFields = 0;
    if (const json::Value* dl = s.Find("databases"); dl && dl->IsArray())
    {
        dbN = static_cast<uint32_t>(dl->arr.size());
        for (uint32_t d = 0; d < dbN && d < db.NumDatabases(); ++d)
        {
            const json::Value& e = dl->arr[d];
            const DatabaseRecord& r = db.Db(d);
            bool ok = true;
            auto cmp = [&](const char* k, double v) {
                if (const json::Value* x = e.Find(k); x && x->IsNumber())
                {
                    ++dbFields;
                    ok = ok && x->num == v;
                }
            };
            if (const json::Value* x = e.Find("name"); x && x->IsString())
            {
                ++dbFields;
                ok = ok && x->str == db.DbName(d);
            }
            cmp("first_row", r.firstRow);
            cmp("num_rows", r.numRows);
            cmp("first_entry", r.firstEntry);
            cmp("num_entries", r.numEntries);
            cmp("dims", r.dims);
            cmp("schema", r.schema);
            cmp("norm", r.norm);
            cmp("num_selectable", r.numSelectable);
            dbOk += ok ? 1 : 0;
        }
    }
    compared += 1;
    equal += (dbN == db.NumDatabases() && dbOk == dbN) ? 1 : 0;
    std::printf("summary: %d/%d comparisons equal; clips %u/%u, databases %u/%u (%u fields)\n", equal, compared, clipsOk,
                clipsN, dbOk, dbN, dbFields);
    Check(equal == compared, "counts equal to the summary json");
}

static void TestDump(const MotionDatabase& db, const char* aPath)
{
    std::string text;
    if (!ReadText(aPath, text))
    {
        Check(false, std::string("cannot read dump ") + aPath);
        return;
    }
    const MotionHeader& h = db.Header();
    const DerivedCounts c = Derive(db);
    std::istringstream in(text);
    std::string line;
    int hdrN = 0, hdrOk = 0, chkN = 0, chkOk = 0, cntN = 0, cntOk = 0, featN = 0, featOk = 0, rtzN = 0, rtzOk = 0, rtzNonZero = 0;
    bool end = false;
    while (std::getline(in, line))
    {
        std::istringstream ls(line);
        std::string tag;
        ls >> tag;
        if (tag == "HDR")
        {
            std::string k, v;
            ls >> k >> v;
            ++hdrN;
            const unsigned long long u =
                std::strtoull(v.c_str(), nullptr, (k == "fps" || v.rfind("0x", 0) == 0) ? 16 : 10); // fps: float32 bits
            uint64_t mine = 0;
            bool known = true;
            if (k == "version") mine = h.version;
            else if (k == "header_size") mine = h.headerSize;
            else if (k == "file_size") mine = h.fileSize;
            else if (k == "rig_hash") mine = h.rigHash;
            else if (k == "flags") mine = h.flags;
            else if (k == "num_frames") mine = h.numFrames;
            else if (k == "num_bones") mine = h.numBones;
            else if (k == "num_clips") mine = h.numClips;
            else if (k == "num_schemas") mine = h.numSchemas;
            else if (k == "num_channels") mine = h.numChannels;
            else if (k == "num_norms") mine = h.numNorms;
            else if (k == "num_databases") mine = h.numDatabases;
            else if (k == "num_entries") mine = h.numEntries;
            else if (k == "num_rows") mine = h.numRows;
            else if (k == "num_curves") mine = h.numCurves;
            else if (k == "num_events") mine = h.numEvents;
            else if (k == "fps") mine = Bits(h.fps);
            else if (k == "num_sections") mine = h.numSections;
            else if (k == "section_table_offset") mine = h.sectionTableOffset;
            else if (k == "payload_crc32") mine = h.payloadCrc32;
            else if (k == "header_crc32") mine = h.headerCrc32;
            else if (k == "bound_sm_size") mine = h.boundSmSize;
            else if (k == "bound_lr_size") mine = h.boundLrSize;
            else if (k == "max_dims") mine = h.maxDims;
            else if (k == "dfea_floats") mine = h.dfeaFloats;
            else if (k == "dbox_floats") mine = h.dboxFloats;
            else known = false;
            const bool ok = known && mine == u;
            hdrOk += ok ? 1 : 0;
            if (!ok)
                std::printf("  dump HDR %s %s differs\n", k.c_str(), v.c_str());
        }
        else if (tag == "CHK")
        {
            std::string k, v;
            ls >> k >> v;
            ++chkN;
            bool ok = v == "1";
            if (k == "rig_identity_from_bone_rigr")
                ok = std::strtoull(v.c_str(), nullptr, 16) == h.rigHash;
            chkOk += ok ? 1 : 0;
            if (!ok)
                std::printf("  dump CHK %s %s\n", k.c_str(), v.c_str());
        }
        else if (tag == "CNT")
        {
            std::string k;
            uint64_t v = 0;
            ls >> k >> v;
            ++cntN;
            static const char* kFlagNames[10] = {"frames_selectable", "frames_excluded_notify", "", "frames_loop_duplicate",
                                                 "frames_block_transition", "frames_branch_in", "frames_clip_start",
                                                 "frames_clip_end", "frames_excluded_tail", "frames_indexed"};
            int64_t mine = -1;
            for (int i = 0; i < 10; ++i)
                if (k == kFlagNames[i])
                    mine = static_cast<int64_t>(c.frames[i]);
            if (k == "clips_rtz_present") mine = c.rtzPresent;
            if (k == "clips_rtz_unknown") mine = c.rtzUnknown;
            if (k == "clips_rtz_none") mine = c.rtzNone;
            if (k == "databases_refused_rtz") mine = db.NumRefusedDatabases();
            if (k == "databases_rtz_pending_flag") mine = c.pending;
            if (k == "string_bytes") mine = -2; // informational
            const bool ok = mine == -2 || mine == static_cast<int64_t>(v);
            cntOk += ok ? 1 : 0;
            if (!ok)
                std::printf("  dump CNT %s %" PRIu64 " vs %" PRId64 "\n", k.c_str(), v, mine);
        }
        else if (tag == "FEAT")
        {
            uint32_t d, row, grow, frame, dims;
            ls >> d >> row >> grow >> frame >> dims;
            ++featN;
            bool ok = d < db.NumDatabases() && row < db.Db(d).numRows && db.Db(d).firstRow + row == grow &&
                      db.Row(grow).frame == frame && db.Db(d).dims == dims;
            if (ok)
            {
                const float* f = db.Features(d, row);
                for (uint32_t k = 0; k < dims; ++k)
                {
                    std::string hx;
                    ls >> hx;
                    ok = ok && static_cast<uint32_t>(std::strtoul(hx.c_str(), nullptr, 16)) == Bits(f[k]);
                }
            }
            featOk += ok ? 1 : 0;
            if (!ok)
                std::printf("  dump FEAT db %u row %u differs\n", d, row);
        }
        else if (tag == "RTZ")
        {
            uint32_t frame, clip;
            std::string rz, vz;
            ls >> frame >> clip >> rz >> vz;
            ++rtzN;
            const bool ok = frame < db.NumFrames() && clip < db.NumClips() && frame >= db.Clip(clip).firstFrame &&
                            frame < db.Clip(clip).firstFrame + db.Clip(clip).numFrames &&
                            static_cast<uint32_t>(std::strtoul(rz.c_str(), nullptr, 16)) == Bits(db.Rtz(frame).rootZ) &&
                            static_cast<uint32_t>(std::strtoul(vz.c_str(), nullptr, 16)) == Bits(db.Rtz(frame).vz);
            rtzOk += ok ? 1 : 0;
            rtzNonZero += (ok && db.Rtz(frame).rootZ != 0.0f) ? 1 : 0;
            if (!ok)
                std::printf("  dump RTZ frame %u differs\n", frame);
        }
        else if (tag == "END")
            end = true;
    }
    std::printf("dump: HDR %d/%d, CHK %d/%d, CNT %d/%d, FEAT %d/%d (bit-exact), RTZ %d/%d (bit-exact, %d non-zero)\n", hdrOk,
                hdrN, chkOk, chkN, cntOk, cntN, featOk, featN, rtzOk, rtzN, rtzNonZero);
    Check(end, "dump complete (END)");
    Check(hdrN > 20 && hdrOk == hdrN, "dump: header fields equal");
    Check(chkN >= 4 && chkOk == chkN, "dump: independent CRC / rig identity checks");
    Check(cntN > 0 && cntOk == cntN, "dump: flag / RTZ counts equal");
    Check(featN >= 20 && featOk == featN, "dump: 20 random DFEA rows bit-exact");
    Check(rtzN > 0 && rtzOk == rtzN, "dump: RTZ_ rows bit-exact");
}

static void TestRtz(const MotionDatabase& db)
{
    const DerivedCounts c = Derive(db);
    const float fps = db.Header().fps;
    uint32_t clips = 0, frames = 0, bad = 0, zeroClips = 0;
    float maxAbsZ = 0.0f, maxErr = 0.0f;
    for (uint32_t i = 0; i < db.NumClips(); ++i)
    {
        const MotionClipRecord& cl = db.Clip(i);
        if (!(cl.flags & kClipRtzPresent))
            continue;
        ++clips;
        const uint32_t f0 = cl.firstFrame, n = cl.numFrames;
        bool allZero = true;
        for (uint32_t k = 0; k < n; ++k)
        {
            const float z = db.Rtz(f0 + k).rootZ;
            allZero = allZero && z == 0.0f;
            maxAbsZ = std::fmax(maxAbsZ, std::fabs(z));
            double expect;
            if (n < 2)
                expect = 0.0;
            else if (k == 0)
                expect = (static_cast<double>(db.Rtz(f0 + 1).rootZ) - db.Rtz(f0).rootZ) * fps;
            else if (k == n - 1)
                expect = (static_cast<double>(db.Rtz(f0 + n - 1).rootZ) - db.Rtz(f0 + n - 2).rootZ) * fps;
            else
                expect = (static_cast<double>(db.Rtz(f0 + k + 1).rootZ) - db.Rtz(f0 + k - 1).rootZ) * fps / 2.0;
            const double err = std::fabs(expect - db.Rtz(f0 + k).vz);
            maxErr = std::fmax(maxErr, static_cast<float>(err));
            if (err > 1e-3 + 1e-4 * std::fabs(expect))
                ++bad;
            ++frames;
        }
        zeroClips += allZero ? 1 : 0;
    }
    std::printf("RTZ_: %u clips with root_z (%u frames, max |root_z| %.3f m, %u all-zero), vz vs central difference max "
                "err %.2e m/s (%u frames over tolerance); clips RTZ_UNKNOWN %u, RTZ_NONE %u; databases refused %u, "
                "RTZ_PENDING %u\n",
                clips, frames, maxAbsZ, zeroClips, maxErr, bad, c.rtzUnknown, c.rtzNone, db.NumRefusedDatabases(), c.pending);
    Check(clips > 0 && maxAbsZ > 0.0f, "RTZ_: root_z rows present for RTZ clips");
    Check(bad == 0, "RTZ_: vz = central difference of root_z x fps (one-sided at the ends)");
    Check(c.rtzUnknown == 0, "RTZ_: no clip left RTZ_UNKNOWN");
    Check(db.NumRefusedDatabases() == 0 && c.pending == 0, "RTZ_: no RTZ_PENDING / refused database");
}

static void TestSearch(const MotionDatabase& db)
{
    uint64_t lin = 0, aabb = 0;
    uint32_t queries = 0, same = 0;
    uint32_t seed = 12345u;
    auto rnd = [&]() {
        seed = seed * 1664525u + 1013904223u;
        return static_cast<float>((seed >> 8) & 0xFFFF) / 65535.0f - 0.5f;
    };
    std::vector<float> q;
    for (uint32_t d = 0; d < db.NumDatabases(); ++d)
    {
        const DatabaseRecord& r = db.Db(d);
        if (!db.DatabaseUsable(d) || r.numRows == 0 || r.numSelectable == 0)
            continue;
        q.resize(r.dims);
        for (int i = 0; i < 10; ++i)
        {
            const uint32_t row = (seed = seed * 1664525u + 1013904223u) % r.numRows;
            const float* f = db.Features(d, row);
            const float amp = i < 5 ? 0.05f : 0.5f;
            for (uint32_t k = 0; k < r.dims; ++k)
                q[k] = f[k] + amp * rnd();
            const MotionSearchResult a = db.SearchLinear(d, q.data());
            const MotionSearchResult b = db.SearchAabb(d, q.data());
            ++queries;
            same += (a.localRow == b.localRow && Bits(a.cost) == Bits(b.cost) && a.localRow >= 0 &&
                     !(db.Row(r.firstRow + static_cast<uint32_t>(a.localRow)).flags & kRowBlock))
                        ? 1
                        : 0;
            lin += a.rowsEvaluated;
            aabb += b.rowsEvaluated;
        }
    }
    std::printf("search: %u queries, AABB == linear (row + bitwise cost, never a BLOCK row) %u/%u, rows evaluated "
                "linear %" PRIu64 " vs AABB %" PRIu64 "\n",
                queries, same, queries, lin, aabb);
    Check(queries > 0 && same == queries, "AABB search == linear scan");
}

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::printf("usage: utpp_motion_db_test <db.utppmm> [summary.json] [dump.txt] [woman rig.json] [man rig.json]\n");
        return 2;
    }
    std::vector<uint8_t> bytes;
    if (!ReadBytes(argv[1], bytes))
    {
        std::printf("NOTE: database %s not found - utpp_motion_db_test NOT RUN\n", argv[1]);
        return 2;
    }
    MotionDatabase db;
    std::string err;
    const auto t0 = std::chrono::steady_clock::now();
    const bool ok = db.LoadFromMemory(bytes, kRigIdentityWoman, err);
    const auto t1 = std::chrono::steady_clock::now();
    Check(ok, "motion database loads on the woman rig identity: " + err);
    if (!ok)
    {
        std::printf("utpp_motion_db_test: %d passed, %d failed\n", g_pass, g_fail);
        return 1;
    }
    const MotionHeader& h = db.Header();
    std::printf("loaded %zu bytes in %.1f ms: frames %u, bones %u, clips %u, schemas %u, channels %u, norms %u, databases %u "
                "(usable %u), entries %u, rows %u, curves %u, events %u, max dims %u\n",
                bytes.size(), std::chrono::duration<double, std::milli>(t1 - t0).count(), h.numFrames, h.numBones, h.numClips,
                h.numSchemas, h.numChannels, h.numNorms, h.numDatabases, h.numDatabases - db.NumRefusedDatabases(),
                h.numEntries, h.numRows, h.numCurves, h.numEvents, h.maxDims);
    TestRig(db, bytes, argc >= 5 ? argv[4] : nullptr, argc >= 6 ? argv[5] : nullptr);
    TestCorruption(bytes);
    if (argc >= 3)
        TestSummary(db, bytes.size(), argv[2]);
    if (argc >= 4)
        TestDump(db, argv[3]);
    TestRtz(db);
    TestSearch(db);
    std::printf("utpp_motion_db_test: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
