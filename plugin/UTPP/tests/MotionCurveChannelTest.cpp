// Offline test of the wide database set (chooser::DbSet, more than 64 databases) and of SCHC Curve channels (kind 5) in
// the .utppmm VERSION 2 reader (src/mm/MotionDatabase.hpp).
//   utpp_mmdb_curve_channel_test <db.utppmm> [fixture_out.utppmm]
// The Curve fixture is the given database with one pose channel rewritten in memory as a Curve channel (header flag
// CURVE_CHANNELS set, CRCs recomputed); it is written to fixture_out when given.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "../src/mm/ChooserMatcher.hpp"
#include "../src/mm/MotionDatabase.hpp"

using namespace utpp::mm;
using namespace utpp::mm::motion;

static int g_pass = 0, g_fail = 0;

static void Check(bool aOk, const std::string& aWhat)
{
    if (aOk)
        ++g_pass;
    else
    {
        ++g_fail;
        std::printf("FAIL %s\n", aWhat.c_str());
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
    const bool ok = n > 0 && std::fread(aOut.data(), 1, aOut.size(), f) == aOut.size();
    std::fclose(f);
    return ok;
}

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

static bool SectionOffset(const std::vector<uint8_t>& b, const char* aId, uint64_t& aOff)
{
    MotionHeader h;
    std::memcpy(&h, b.data(), sizeof(h));
    for (uint32_t i = 0; i < h.numSections; ++i)
    {
        uint32_t id;
        std::memcpy(&id, b.data() + kHeaderSize + 32 * i, 4);
        if (std::memcmp(&id, aId, 4) == 0)
        {
            std::memcpy(&aOff, b.data() + kHeaderSize + 32 * i + 16, 8);
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

static void TestDbSet()
{
    chooser::DbSet s;
    Check(!s.Any() && s.Low() == 0, "empty set");
    s.Set(5);
    s.Set(63);
    s.Set(64);
    s.Set(70);
    s.Set(127);
    s.Set(128); // ignored
    s.Set(-1);  // ignored
    Check(s.Test(5) && s.Test(63) && s.Test(64) && s.Test(70) && s.Test(127), "set ids up to 127");
    Check(!s.Test(6) && !s.Test(65) && !s.Test(128) && !s.Test(-1) && !s.Test(1000), "unset / out of range ids");
    Check(s.Low() == ((uint64_t(1) << 5) | (uint64_t(1) << 63)), "low word holds ids < 64 only");
    chooser::DbSet t = chooser::DbSet::FromLow(uint64_t(1) << 5);
    Check(t.Intersects(s) && !chooser::DbSet::FromLow(2).Intersects(s), "intersects");
    t |= s;
    Check(t == s && t != chooser::DbSet{}, "union / equality");

    chooser::Output o;
    o.count = 4;
    o.db[0] = 3;
    o.db[1] = 64;
    o.db[2] = 127;
    o.db[3] = -1;
    const chooser::DbSet os = o.Set();
    Check(os.Test(3) && os.Test(64) && os.Test(127) && !os.Test(0), "chooser output set holds ids >= 64");
    Check(o.Mask() == (uint64_t(1) << 3), "chooser output Mask keeps the low-word semantics");
    Check(chooser::kMaxResults >= 128 && kMaxDatabases >= 128, "limits widened to 128 databases");
}

static void TestOldBundle(const MotionDatabase& db)
{
    Check(!db.HasCurveChannels(), "existing bundle has no curve channels");
    bool maskOk = true, dentOk = true;
    for (uint32_t c = 0; c < db.NumClips(); ++c)
        maskOk = maskOk && db.ClipDatabases(c).Low() == db.Clip(c).dbMask && db.ClipDatabases(c).w[1] == 0;
    for (uint32_t d = 0; d < db.NumDatabases(); ++d)
        for (uint32_t e = db.Db(d).firstEntry; e < db.Db(d).firstEntry + db.Db(d).numEntries; ++e)
            dentOk = dentOk && db.ClipInDatabase(db.Entry(e).clip, d);
    Check(maskOk, "per-clip database set equals the CLIP db_mask word for <= 64 databases");
    Check(dentOk, "every DENT entry's clip is in its database's set");
}

static void TestCurveAt(const MotionDatabase& db, uint32_t k)
{
    const float fps = db.Header().fps;
    int loopClip = -1, plainClip = -1;
    for (uint32_t c = 0; c < db.NumClips() && (loopClip < 0 || plainClip < 0); ++c)
    {
        const MotionClipRecord& cl = db.Clip(c);
        if (cl.numFrames < 8 || !(db.CurvePresence(cl.firstFrame) & (1u << k)))
            continue;
        if ((cl.flags & motion::kClipLoop) && cl.loopPeriod > 0 && loopClip < 0)
            loopClip = static_cast<int>(c);
        else if (!(cl.flags & motion::kClipLoop) && plainClip < 0)
            plainClip = static_cast<int>(c);
    }
    Check(loopClip >= 0 && plainClip >= 0, "curve present on a loop and a non-loop clip");
    if (loopClip < 0 || plainClip < 0)
        return;
    for (int c : {loopClip, plainClip})
    {
        const MotionClipRecord& cl = db.Clip(static_cast<uint32_t>(c));
        bool exact = true, half = true;
        for (uint32_t f = 0; f + 1 < cl.numFrames; ++f)
        {
            const float a = db.CurveValue(cl.firstFrame + f, k), b = db.CurveValue(cl.firstFrame + f + 1, k);
            exact = exact && db.CurveAt(static_cast<uint32_t>(c), k, f / static_cast<double>(fps)) == a;
            half = half && std::fabs(db.CurveAt(static_cast<uint32_t>(c), k, (f + 0.5) / static_cast<double>(fps)) - (a + (b - a) * 0.5f)) < 1e-6f;
        }
        Check(exact, "CurveAt equals CURV at whole frames (clip " + std::to_string(c) + ")");
        Check(half, "CurveAt interpolates linearly at half frames (clip " + std::to_string(c) + ")");
    }
    {
        const MotionClipRecord& cl = db.Clip(static_cast<uint32_t>(plainClip));
        const uint32_t pc = static_cast<uint32_t>(plainClip);
        Check(db.CurveAt(pc, k, -0.5) == db.CurveValue(cl.firstFrame, k) &&
                  db.CurveAt(pc, k, cl.numFrames / static_cast<double>(fps) + 1.0) == db.CurveValue(cl.firstFrame + cl.numFrames - 1, k),
              "non-loop clamps before the start / after the end");
    }
    {
        const MotionClipRecord& cl = db.Clip(static_cast<uint32_t>(loopClip));
        const uint32_t lc = static_cast<uint32_t>(loopClip);
        const double period = cl.loopPeriod / static_cast<double>(fps);
        bool wrap = true;
        for (double t : {0.0, 1.0 / 30.0, 0.25, 0.5})
            wrap = wrap && std::fabs(db.CurveAt(lc, k, t + period) - db.CurveAt(lc, k, t)) < 1e-5f &&
                   std::fabs(db.CurveAt(lc, k, t - period) - db.CurveAt(lc, k, t)) < 1e-5f;
        Check(wrap, "loop wraps by loop_period in both directions");
        const double ts = -1.0 / 30.0;
        ChannelRecord ch{};
        ch.kind = kChanCurve;
        ch.curve = k;
        ch.tSample = static_cast<float>(ts);
        Check(db.CurveFeature(ch, lc, 0.0) == db.CurveAt(lc, k, static_cast<double>(static_cast<float>(ts))),
              "CurveFeature reads the curve at time + t_sample (loop start reads the previous loop end)");
    }
}

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::printf("usage: utpp_mmdb_curve_channel_test <db.utppmm> [fixture_out.utppmm]\n");
        return 2;
    }
    TestDbSet();
    std::vector<uint8_t> bytes;
    if (!ReadBytes(argv[1], bytes))
    {
        std::printf("FAIL cannot read %s\n", argv[1]);
        return 1;
    }
    MotionHeader h;
    std::memcpy(&h, bytes.data(), sizeof(h));
    const uint64_t rig = h.rigHash;
    MotionDatabase base;
    std::string err;
    Check(base.LoadFromMemory(bytes, rig, err), "existing bundle loads: " + err);
    if (!base.Loaded())
        return 1;
    TestOldBundle(base);

    // curve to read: a phase curve (unit 3) when present, else curve 0
    uint64_t crvn = 0, schc = 0;
    Check(SectionOffset(bytes, "CRVN", crvn) && SectionOffset(bytes, "SCHC", schc), "CRVN / SCHC sections");
    uint32_t curve = 0;
    for (uint32_t k = 0; k < h.numCurves; ++k)
    {
        uint32_t unit;
        std::memcpy(&unit, bytes.data() + crvn + 8ull * k + 4, 4);
        if (unit == 3)
        {
            curve = k;
            break;
        }
    }
    Check(h.numCurves > 0, "bundle has curves");
    if (h.numCurves == 0)
        return 1;
    TestCurveAt(base, curve);

    // fixture: first non-trajectory channel of database 0's schema becomes Curve(curve) at -1/30 s
    const SchemaRecord& sc = base.Schema(base.Db(0).schema);
    int chan = -1;
    for (uint32_t c = sc.firstChannel; c < sc.firstChannel + sc.numChannels && chan < 0; ++c)
        if (!(base.Channel(c).flags & kChanTrajectory))
            chan = static_cast<int>(c);
    Check(chan >= 0, "schema has a pose channel to rewrite");
    if (chan < 0)
        return 1;
    const uint64_t at = schc + 76ull * static_cast<uint32_t>(chan);
    ChannelRecord ch;
    std::memcpy(&ch, bytes.data() + at, sizeof(ch));
    ch.kind = kChanCurve;
    ch.components = 1;
    ch.cardinality = 1;
    ch.bone = -1;
    ch.originBone = -1;
    ch.tSample = -1.0f / 30.0f;
    ch.tOrigin = 0.0f;
    ch.velocitySpace = 0;
    ch.inputQuery = 0;
    ch.flags = kChanUnitless;
    ch.curve = curve;
    std::vector<uint8_t> fx = bytes;
    std::memcpy(fx.data() + at, &ch, sizeof(ch));
    {
        MotionHeader fh;
        std::memcpy(&fh, fx.data(), sizeof(fh));
        fh.flags |= kHeaderCurveChannels;
        std::memcpy(fx.data(), &fh, sizeof(fh));
    }
    FixCrcs(fx);
    MotionDatabase cdb;
    Check(cdb.LoadFromMemory(fx, rig, err), "curve fixture loads: " + err);
    Check(cdb.HasCurveChannels() && cdb.Channel(static_cast<uint32_t>(chan)).kind == kChanCurve &&
              cdb.Channel(static_cast<uint32_t>(chan)).curve == curve,
          "curve channel read back");
    if (argc >= 3)
    {
        FILE* f = nullptr;
        const bool ok = fopen_s(&f, argv[2], "wb") == 0 && f && std::fwrite(fx.data(), 1, fx.size(), f) == fx.size();
        if (f)
            std::fclose(f);
        Check(ok, "fixture written");
    }

    // matcher query: the Curve dimension comes from the playing clip when no continuing row of the schema is used
    {
        std::string path = argv[1];
        const std::string chPath = path.substr(0, path.size() - 7) + ".chooser.json";
        chooser::Table table;
        ChooserMatcher mm;
        MatcherConfig cfg;
        const bool ok = table.LoadFile(chPath.c_str(), err) && mm.Init(&cdb, &table, cfg, err);
        Check(ok, "matcher on the curve fixture: " + err);
        if (ok)
        {
            std::vector<TrajSample> traj;
            for (int i = -10; i <= 20; ++i)
            {
                TrajSample t;
                t.t = i * 0.1f;
                traj.push_back(t);
            }
            ChooserState cs;
            MatchResult r;
            for (int i = 0; i < 40; ++i)
                r = mm.Update(1.0f / 30.0f, traj.data(), static_cast<uint32_t>(traj.size()), cs);
            Check(r.clip >= 0, "matcher plays a clip");
            uint32_t checked = 0;
            bool same = true;
            for (uint32_t d = 0; d < cdb.NumDatabases() && r.clip >= 0; ++d)
            {
                if (cdb.Db(d).schema != cdb.Db(0).schema)
                    continue;
                mm.BuildQuery(d, false);
                const ChannelRecord& c = cdb.Channel(static_cast<uint32_t>(chan));
                const float want = cdb.CurveFeature(c, static_cast<uint32_t>(r.clip), r.clipTime);
                same = same && mm.RawQuery()[c.dataOffset] == want;
                ++checked;
            }
            Check(checked > 0 && same, "curve query = curve of the playing clip at its time + t_sample (" + std::to_string(checked) + " databases)");
            // the dimensions next to the curve keep the database mean (they belonged to the rewritten channel)
            mm.BuildQuery(0, false);
            const ChannelRecord& c = cdb.Channel(static_cast<uint32_t>(chan));
            Check(mm.RawQuery()[c.dataOffset + 1] == cdb.NormMean(cdb.Db(0).norm)[c.dataOffset + 1], "no stray write past the curve dimension");
        }
    }

    // refusals
    auto mutate = [&](auto&& aEdit, bool aFlag) {
        std::vector<uint8_t> b = bytes;
        ChannelRecord c2 = ch;
        aEdit(c2);
        std::memcpy(b.data() + at, &c2, sizeof(c2));
        MotionHeader fh;
        std::memcpy(&fh, b.data(), sizeof(fh));
        if (aFlag)
            fh.flags |= kHeaderCurveChannels;
        std::memcpy(b.data(), &fh, sizeof(fh));
        FixCrcs(b);
        return LoadError(b, rig);
    };
    Check(!mutate([](ChannelRecord&) {}, false).empty(), "curve channel without the header flag is refused");
    Check(!mutate([&](ChannelRecord& c) { c.curve = h.numCurves; }, true).empty(), "curve index out of range is refused");
    Check(!mutate([](ChannelRecord& c) { c.cardinality = 2; }, true).empty(), "curve cardinality != 1 is refused");
    Check(!mutate([](ChannelRecord& c) { c.components = 7; }, true).empty(), "curve components != 1 is refused");
    Check(!mutate([](ChannelRecord& c) { c.bone = 2; }, true).empty(), "curve channel with a bone is refused");
    Check(!mutate([](ChannelRecord& c) { c.inputQuery = 1; }, true).empty(), "curve channel with a character query is refused");
    Check(!mutate([](ChannelRecord& c) { c.tOrigin = 0.1f; }, true).empty(), "curve channel with an origin time is refused");
    Check(!mutate([](ChannelRecord& c) { c.flags |= kChanTrajectory; }, true).empty(), "curve channel flagged trajectory is refused");
    Check(!mutate([](ChannelRecord& c) { c.kind = 6; }, true).empty(), "unknown channel kind is refused");
    {
        std::vector<uint8_t> b = bytes;
        MotionHeader fh;
        std::memcpy(&fh, b.data(), sizeof(fh));
        fh.flags |= kHeaderCurveChannels;
        std::memcpy(b.data(), &fh, sizeof(fh));
        FixCrcs(b);
        Check(!LoadError(b, rig).empty(), "header flag without a curve channel is refused");
        fh.flags = (fh.flags & ~kHeaderCurveChannels) | (1u << 2);
        std::memcpy(b.data(), &fh, sizeof(fh));
        FixCrcs(b);
        Check(!LoadError(b, rig).empty(), "unknown header flag is still refused");
    }
    // trajectory offsets off the 1/60 s grid load unchanged (the reader does not quantise; the query interpolates)
    {
        std::vector<uint8_t> b = bytes;
        int tch = -1;
        for (uint32_t c = sc.firstChannel; c < sc.firstChannel + sc.numChannels && tch < 0; ++c)
            if (base.Channel(c).flags & kChanTrajectory)
                tch = static_cast<int>(c);
        if (tch >= 0)
        {
            ChannelRecord t;
            std::memcpy(&t, b.data() + schc + 76ull * static_cast<uint32_t>(tch), sizeof(t));
            t.tSample = 0.33f;
            std::memcpy(b.data() + schc + 76ull * static_cast<uint32_t>(tch), &t, sizeof(t));
            FixCrcs(b);
            Check(LoadError(b, rig).empty(), "trajectory offset 0.33 s (off the 1/60 s grid) loads");
        }
    }
    std::printf("%d PASS %d FAIL\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
