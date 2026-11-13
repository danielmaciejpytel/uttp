// Fix batch 3 / step B of P9: authored recoil profiles per weapon family.
// Usage: utpp_recoil_profiles_test <recoil data dir> <scripts\utpp_motion.reds> <tools\analysis\export_nue_native_recoil_curves.py>
// 1. mm/RecoilProfiles.hpp: ids are the index, families unique and lower case, file names follow the exporter's naming.
// 2. Every profile pair in the recoil data dir loads with Clip::Load (identity endpoints, bounded keys) and replays bounded through
//    Player at the sample rate of three frame rates; every curve really moves the weapon (a flat file is not a profile).
// 3. The family -> id lines of utpp_motion.reds (UTPPMotionRecoilProfileOfFamily, sent to UTPPSetMMRecoilWeapon) agree with the table, the
//    Nue stays bound to its exact preset id 1, and the exporter's PROFILE_SOURCES names exactly the table's families.
// 4. Budget and series: every profile but the Nue is exported within 8 cm / 0.50 rad per shot and opens without a jump (first
//    interval <= 25% of the peak); a burst of 10 shots at 8 and 12 rounds/s at 30, 60 and 144 fps - through the real Player with the
//    profile's Limits, the driver's admission ramp and tail weight - never hits the saturation clamp and never moves the weapon more
//    than 3 cm / 5 degrees between two frames (the review of the Masamune / Overture bursts: 25 cm clamp, 8 cm / 13 degrees per round).
// A tampered copy of the reds (see recoilprofiles.cmake) must fail check 3: the sync guard is itself guarded.
#include "../src/mm/AuthoredRecoil.hpp"
#include "../src/mm/RecoilProfiles.hpp"
#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <process.h>
#include <regex>
#include <set>
#include <sstream>
using namespace utpp::mm;
namespace rp = utpp::mm::recoil_profile;
namespace ar = utpp::mm::authored_recoil;
constexpr float kBudgetT = .08f, kBudgetR = .50f, kStepT = .03f, kStepR = 5.0f * 3.14159265f / 180.0f;

// The kick opens with a ramp: the first sample interval carries at most 25% of the peak. A curve that jumps to its first key (the
// Masamune shoulder: 15.6 cm after 7 ms, peak 19.8 cm) fails it.
static bool OpensSmoothly(const ar::Clip& aClip)
{
    ov::Xf first;
    if (!aClip.Sample(double(aClip.Duration()) / 96.0, first)) return false;
    return ar::Translation(first) <= .25f * aClip.PeakTranslation() + 1e-4f && ar::Rotation(first) <= .25f * aClip.PeakRotation() + 1e-3f;
}

struct Burst { float maxT = 0, maxR = 0, stepT = 0, stepR = 0; bool clamped = false; };
// aShots rounds aPeriod seconds apart on aFps frames per second; the arms get BlendXf(identity, out, admission * tail) as the driver
// hands it to the IK (admission: 0.001 until the first applied frame, then a 50 ms smoothstep; a running kick keeps its admission).
static Burst Fire(const ar::Clip& aClip, const ar::Limits& aLimits, double aFps, double aPeriod, int aShots)
{
    ar::Player player;
    const ar::Binding binding{1, 1, 1};
    Burst r;
    double next = 0, admitTime = 0;
    bool admitted = false;
    uint64_t serial = 0;
    int fired = 0;
    ov::Xf prev;
    const int frames = int(aFps * (aShots * aPeriod + double(aClip.Duration()) + .5));
    for (int f = 0; f < frames; ++f)
    {
        const double now = 1.0 + f / aFps;
        if (fired < aShots && now - 1.0 >= next - 1e-9)
        {
            if (!player.Active()) admitted = false;
            player.Trigger(aClip, binding, ++serial, now, aLimits);
            next += aPeriod;
            ++fired;
        }
        ov::Xf out;
        player.Sample(aClip, binding, now, out);
        float admission;
        if (!admitted) { admission = .001f; if (player.Active()) { admitted = true; admitTime = now; } }
        else { const float u = std::clamp(float((now - admitTime) / .05), 0.0f, 1.0f); admission = std::max(.001f, u * u * (3.0f - 2.0f * u)); }
        const ov::Xf effective = ov::BlendXf(ov::Xf{}, out, admission * player.TailWeight(now));
        r.clamped = r.clamped || ar::Translation(out) >= ar::kMaxTranslation - 1e-4f || ar::Rotation(out) >= ar::kMaxRotation - 1e-3f;
        r.maxT = std::max(r.maxT, ar::Translation(effective));
        r.maxR = std::max(r.maxR, ar::Rotation(effective));
        if (f > 0)
        {
            const math::Vec3 d = effective.t - prev.t;
            r.stepT = std::max(r.stepT, std::sqrt(math::Dot(d, d)));
            const math::Quat rel = math::QuatNormalize(math::QuatMul(math::QuatInv(prev.r), effective.r));
            r.stepR = std::max(r.stepR, 2 * std::atan2(std::sqrt(rel.x * rel.x + rel.y * rel.y + rel.z * rel.z), std::fabs(rel.w)));
        }
        prev = effective;
    }
    return r;
}
static int g_checks = 0, g_failed = 0;
static bool Check(bool aOk, const std::string& aName)
{
    ++g_checks;
    if (!aOk)
    {
        ++g_failed;
        std::cerr << "FAIL " << aName << '\n';
    }
    return aOk;
}
static std::string ReadAll(const std::string& aPath)
{
    std::ifstream in(aPath, std::ios::binary);
    std::ostringstream s;
    s << in.rdbuf();
    std::string text = s.str();
    text.erase(std::remove(text.begin(), text.end(), '\r'), text.end()); // CRLF checkouts / cmake-written copies
    return text;
}
static std::string Lower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}
int main(int argc, char** argv)
{
    if (argc < 4)
    {
        std::cerr << "usage: utpp_recoil_profiles_test <recoil dir> <utpp_motion.reds> <exporter.py>\n";
        return 2;
    }
    const std::filesystem::path dir = argv[1];

    // 1. table
    Check(rp::kTable[0].family == nullptr && !rp::Valid(0) && !rp::Valid(-1) && !rp::Valid(rp::kSlots) && rp::Valid(1) && rp::Valid(rp::kSlots - 1),
          "id 0 is none, ids 1..kSlots-1 are valid");
    Check(std::string(rp::kTable[rp::kNue].family) == "nue" && rp::kNue == 1, "the Nue keeps profile id 1");
    std::set<std::string> families;
    bool names = true;
    for (int32_t id = 1; id < rp::kSlots; ++id)
    {
        const rp::Entry& e = rp::kTable[static_cast<size_t>(id)];
        if (!rp::Valid(id)) { names = names && !e.family && !e.shoulder && !e.ironsight; continue; } // retired slot stays empty
        const std::string family = e.family ? e.family : "";
        names = names && !family.empty() && Lower(family) == family && families.insert(family).second;
        if (id == rp::kNue)
            names = names && std::string(e.shoulder) == "add_recoil_single_shot_shoulder.utpprecoil" &&
                    std::string(e.ironsight) == "add_recoil_single_shot_ironsight.utpprecoil";
        else
            names = names && std::string(e.shoulder) == family + "_single_shot_shoulder.utpprecoil" &&
                    std::string(e.ironsight) == family + "_single_shot_ironsight.utpprecoil";
    }
    Check(names, "families unique, lower case; file names follow <family>_single_shot_{shoulder,ironsight}.utpprecoil");

    // 2. data
    size_t samples = 0;
    // the three revolvers whose clips the hand IK cannot follow keep their slots but are not valid, not mapped and have no file
    Check(!rp::Valid(8) && !rp::Valid(9) && !rp::Valid(10) && rp::Valid(11) && rp::Valid(rp::kSlots - 1), "ids 8, 9, 10 (nova, overture, burya) are retired");
    for (int32_t id = 1; id < rp::kSlots; ++id)
    {
        const rp::Entry& e = rp::kTable[static_cast<size_t>(id)];
        if (!rp::Valid(id)) continue;
        for (int clipIndex = 0; clipIndex < 2; ++clipIndex)
        {
            const std::string file = (dir / (clipIndex ? e.ironsight : e.shoulder)).string();
            const std::string label = std::string(e.family) + (clipIndex ? " ironsight" : " shoulder");
            authored_recoil::Clip clip;
            std::string error;
            if (!Check(clip.Load(file.c_str(), error) && clip.Valid(), label + " loads (" + error + ")"))
                continue;
            Check(clip.Duration() >= .05f && clip.Duration() <= 1.5f && clip.Count() == 97, label + " duration 0.05..1.5 s, 97 samples");
            if (id != rp::kNue)
            {
                Check(clip.PeakTranslation() <= kBudgetT + 1e-4f && clip.PeakRotation() <= kBudgetR + 1e-3f,
                      label + " peak within the per-shot budget 8 cm / 0.50 rad (" + std::to_string(clip.PeakTranslation()) + " m, " +
                          std::to_string(clip.PeakRotation()) + " rad)");
                Check(OpensSmoothly(clip), label + " opens with a ramp, not with a jump to the first key");
                const ar::Limits limits = rp::LimitsOf(id);
                Check(limits.Headroom() && limits.Slew(), label + " runs in series-safe mode");
                Burst worst;
                for (const double fps : {30.0, 60.0, 144.0})
                    for (const double rate : {8.0, 12.0})
                    {
                        const Burst r = Fire(clip, limits, fps, 1.0 / rate, 10);
                        worst.stepT = std::max(worst.stepT, r.stepT); worst.stepR = std::max(worst.stepR, r.stepR);
                        worst.maxT = std::max(worst.maxT, r.maxT); worst.maxR = std::max(worst.maxR, r.maxR);
                        worst.clamped = worst.clamped || r.clamped;
                    }
                    for (const double fps : {30.0, 60.0, 144.0})
                    {
                        const Burst r = Fire(clip, limits, fps, 1.0, 1);
                        worst.stepT = std::max(worst.stepT, r.stepT); worst.stepR = std::max(worst.stepR, r.stepR);
                        worst.clamped = worst.clamped || r.clamped;
                    }
                Check(!worst.clamped, label + " burst: never reaches the saturation clamp");
                Check(worst.stepT <= kStepT && worst.stepR <= kStepR,
                      label + " burst: step per frame <= 3 cm / 5 deg (" + std::to_string(worst.stepT * 100) + " cm, " +
                          std::to_string(worst.stepR * 57.29578f) + " deg)");
                Check(worst.maxT <= limits.seriesTranslation * 1.05f && worst.maxR <= limits.seriesRotation * 1.05f,
                      label + " burst: stays under the series cap");
            }
            for (const double fps : {30.0, 60.0, 144.0})
            {
                authored_recoil::Player player;
                const authored_recoil::Binding binding{static_cast<uint64_t>(id), 8, 9};
                bool bounded = player.Trigger(clip, binding, 1, 100.0);
                ov::Xf out;
                float peakT = 0, peakR = 0;
                for (double t = 0; t <= double(clip.Duration()) + 1.0 / fps; t += 1.0 / fps)
                {
                    player.Sample(clip, binding, 100.0 + t, out);
                    bounded = bounded && authored_recoil::Unit(out) && authored_recoil::Translation(out) <= authored_recoil::kMaxTranslation + 1e-6f &&
                              authored_recoil::Rotation(out) <= authored_recoil::kMaxRotation + 1e-6f;
                    peakT = std::max(peakT, authored_recoil::Translation(out));
                    peakR = std::max(peakR, authored_recoil::Rotation(out));
                    ++samples;
                }
                Check(bounded && authored_recoil::Identity(out), label + " replays bounded and returns to identity at " + std::to_string(int(fps)) + " fps");
                Check(peakT > .0005f || peakR > .005f, label + " moves the weapon (peak " + std::to_string(peakT) + " m / " + std::to_string(peakR) + " rad)");
            }
        }
    }

    // negative control of the opening check: a curve that jumps to 15.6 cm in its first sample interval (the first Masamune export)
    {
        // the test and its mutation twins run concurrently under ctest -j: the fixture name is unique per process
        const auto jump = std::filesystem::temp_directory_path() / ("utpp-rp-negative-control-" + std::to_string(::_getpid()) + ".utpprecoil");
        {
            std::ofstream f(jump);
            f << "UTPP_RECOIL 1\n3 .6\n0 0 0 0 1 0 0 0\n.007 0 0 0 1 .156 0 0\n.6 0 0 0 1 0 0 0\n";
        }
        ar::Clip control;
        std::string error;
        const bool loaded = control.Load(jump.string().c_str(), error);
        std::error_code ignored;
        std::filesystem::remove(jump, ignored);
        Check(loaded, "negative control fixture loads");
        // the first interval (6.25 ms) of this fixture already carries 89% of the 15.6 cm peak
        Check(!OpensSmoothly(control), "negative control: a curve that jumps to its first key is rejected by the opening check");
    }

    // 3. reds + exporter
    const std::string reds = ReadAll(argv[2]);
    Check(!reds.empty(), "utpp_motion.reds readable");
    Check(reds.find("UTPPSetMMRecoilWeapon(weaponEpoch: Uint32, recoilProfile: Int32)") != std::string::npos,
          "native signature carries the profile id");
    const size_t fn = reds.find("func UTPPMotionRecoilProfileOfFamily(family: CName) -> Int32 {");
    Check(fn != std::string::npos, "UTPPMotionRecoilProfileOfFamily present");
    std::map<std::string, int32_t> mapped;
    if (fn != std::string::npos)
    {
        const size_t end = reds.find("\n}\n", fn);
        const std::string body = reds.substr(fn, end == std::string::npos ? std::string::npos : end - fn);
        const std::regex line(R"re(Equals\(family, n"(\w+)"\) \{ return (\d+); \})re");
        for (std::sregex_iterator it(body.begin(), body.end(), line), last; it != last; ++it)
            Check(mapped.emplace(Lower((*it)[1]), std::stoi((*it)[2])).second, "family " + (*it)[1].str() + " mapped once");
    }
    bool every = true, ids = true;
    for (int32_t id = 2; id < rp::kSlots; ++id)
    {
        if (!rp::Valid(id)) continue;
        const auto found = mapped.find(rp::kTable[static_cast<size_t>(id)].family);
        every = every && found != mapped.end() && found->second == id;
    }
    for (const auto& [name, id] : mapped)
        ids = ids && rp::Valid(id) && id != rp::kNue; // the Nue is bound by its exact preset id, never by a family tag
    Check(every, "every table family id >= 2 is returned by the reds for its own tag");
    Check(ids, "no reds family returns an invalid id, a retired id or the Nue id");
    const auto kyubi = mapped.find("kyubi"), ajax = mapped.find("ajax");
    Check(kyubi != mapped.end() && ajax != mapped.end() && kyubi->second == ajax->second, "Kyubi (Ajax animation set) maps to the Ajax profile");
    // aliases are the only reds tags that are not a table family
    size_t aliases = 0;
    for (const auto& [name, id] : mapped)
        aliases += families.count(name) ? 0 : 1;
    Check(aliases == 1, "kyubi is the only alias");
    Check(reds.find("Equals(recordID, t\"Items.Preset_Nue_Default\") {\n      return 1;") != std::string::npos,
          "the Nue stays bound to its exact preset id and returns profile 1");
    const std::string exporter = ReadAll(argv[3]);
    std::set<std::string> sources;
    const size_t table = exporter.find("PROFILE_SOURCES = {");
    if (table != std::string::npos)
    {
        const std::string body = exporter.substr(table, exporter.find("\n}\n", table) - table);
        const std::regex entry(R"re('(\w+)': 'pwa_w_)re");
        for (std::sregex_iterator it(body.begin(), body.end(), entry), last; it != last; ++it)
            sources.insert((*it)[1]);
    }
    std::set<std::string> expected = families;
    expected.erase("nue");
    Check(sources == expected, "the exporter's PROFILE_SOURCES names exactly the table families (Nue has its own entry point)");
    std::cout << "Recoil profiles: " << (rp::kSlots - 1) << " profiles, " << g_checks << " checks, " << samples << " replayed samples, "
              << (g_failed ? "FAIL" : "PASS") << '\n';
    return g_failed ? 1 : 0;
}
