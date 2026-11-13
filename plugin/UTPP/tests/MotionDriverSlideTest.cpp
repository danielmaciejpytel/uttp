// mm.slide through the real MotionDriver (src/MotionDriver.cpp without MotionHost.cpp / RED4ext): bundle detection of the chooser.json
// "slide" section, the setting, OwnsSlide (the lock-free answer MMDriver asks before the body-yaw command), slide frames without foot
// contacts, the chooser pick of the slide databases, and OFF = a bundle without slide data behaves as before.
// Usage: utpp_motion_driver_slide_test <data dir (flat motion-derived files as MotionDriverTest)>
// Uses utpp_motion_woman_knee_slide.* (the slide bundle) and utpp_motion_woman_noslide.* (the plain bundle: no slide section, a fixture in
// --plain-dir = Build/test-data/motion; man: utpp_motion_man_knee_slide / utpp_motion_man_noslide). Missing files FAIL.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include "../src/MotionHost.hpp"
#include "../src/MotionDriver.hpp"

namespace mm = utpp::mm;
namespace motionhost = utpp::motionhost;
namespace motiondriver = utpp::motiondriver;

namespace
{
int g_pass = 0, g_fail = 0;
int g_body = 0;                  // TR-dll male parity: --body man selects the man slide / plain bundles, overlay, style and traversal files
const char* g_bodyName = "woman";
void Check(bool aOk, const char* aWhat, const std::string& aDetail = {})
{
    (aOk ? g_pass : g_fail)++;
    std::printf("%s %s%s%s\n", aOk ? "PASS" : "FAIL", aWhat, aDetail.empty() ? "" : " | ", aDetail.c_str());
}

std::string StatusField(const std::string& aStatus, const char* aKey)
{
    const size_t p = aStatus.find(aKey);
    if (p == std::string::npos)
        return {};
    const size_t s = p + std::strlen(aKey);
    const size_t e = aStatus.find(' ', s);
    return aStatus.substr(s, e == std::string::npos ? std::string::npos : e - s);
}

struct Rig
{
    std::vector<uint64_t> hashes;
    std::vector<int16_t> parents;
    uint32_t bones = 0;
};
bool RigFromDb(const std::string& aPath, Rig& aRig)
{
    std::ifstream f(aPath, std::ios::binary);
    if (!f)
        return false;
    std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    if (bytes.size() < 32)
        return false;
    uint64_t rig = 0;
    std::memcpy(&rig, bytes.data() + 24, sizeof(rig));
    mm::motion::MotionDatabase db;
    std::string err;
    if (!db.LoadFromMemory(bytes, rig, err))
        return false;
    aRig.bones = db.NumBones();
    for (uint32_t i = 0; i < db.NumBones(); ++i)
    {
        aRig.hashes.push_back(mm::integ::CNameHash(db.BoneName(i)));
        aRig.parents.push_back(static_cast<int16_t>(db.BoneParent(i)));
    }
    return true;
}

motiondriver::Paths DataPaths(const std::string& aData, const std::string& aStem)
{
    motiondriver::Paths p;
    p.files.db = aData + "\\" + aStem + ".utppmm";
    p.files.chooser = aData + "\\" + aStem + ".chooser.json";
    p.files.overlay = aData + "\\motion\\utpp_overlays_" + g_bodyName + ".utppov";
    p.files.style = aData + "\\motion\\utpp_style_weights_" + g_bodyName + ".txt";
    p.traversal = aData + "\\traversal\\utpp_traversal_" + g_bodyName + ".utpptrav";
    return p;
}
bool Exists(const std::string& aPath) { return std::ifstream(aPath).good(); }

constexpr uint64_t kRigPtr = 0x1234567800ull;

// Straight sprint / slide along +Y, actor + camera yaw 0, stick forward all the time (the game keeps it forward in a slide).
motionhost::FeedInput SprintFeed(float aSpeed)
{
    motionhost::FeedInput f;
    f.velWorld[1] = aSpeed;
    f.move[1] = 1.0f;
    f.locoState = mm::integ::kLocoSprint;
    f.tppActive = true;
    f.dt = 1.0f / 60.0f;
    return f;
}
motionhost::FeedInput SlideFeed(float aSpeed)
{
    motionhost::FeedInput f = SprintFeed(aSpeed);
    f.locoState = 0; // the dump shows loco 0 during PSM LocomotionDetailed 5
    f.psmBlock = mm::integ::kPsmDetailed | (6u << mm::integ::kPsmDetailShift);
    return f;
}
motionhost::FeedInput CrouchFeed(float aSpeed)
{
    motionhost::FeedInput f = SprintFeed(aSpeed);
    f.locoState = mm::integ::kLocoCrouch;
    return f;
}

struct FrameOut
{
    bool stepped = false, framed = false;
    float contact[2] = {0, 0};
    std::string status;
};

// One Step + Frame (identity bone map as MotionDriverTest) with the status after it.
FrameOut Run(const motionhost::FeedInput& aFeed, const Rig& aRig)
{
    static std::array<mm::BoneTransform, mm::integ::kMaxMMBones> buf{};
    int32_t map[mm::integ::kMaxMMBones];
    for (uint32_t i = 0; i < mm::integ::kMaxMMBones; ++i)
        map[i] = (i >= motiondriver::kFirstBone && i < aRig.bones) ? static_cast<int32_t>(i) : -1;
    FrameOut o;
    o.stepped = motiondriver::Step(aFeed, 0, aFeed.dt);
    if (o.stepped)
    {
        motionhost::MotionSetFrame fr;
        fr.buf = buf.data();
        fr.bufCap = static_cast<uint32_t>(buf.size());
        o.framed = motiondriver::Frame(map, aRig.bones, fr);
        o.contact[0] = fr.contact[0];
        o.contact[1] = fr.contact[1];
    }
    o.status = motiondriver::Status();
    return o;
}

// sprint 0.5 s, slide 1.2 s (boost to 10.8 m/s then -2.75 m/s^2: the measured slide), crouch walk 0.5 s
struct Result
{
    int slideFrames = 0, slideStepped = 0, slideContactFrames = 0, preContactFrames = 0, pickSlideDb = 0, pickOtherDb = 0;
    int stepFail = 0;
    uint64_t statusSlideFrames = 0;
    std::string lastSlideStatus, afterStatus;
};
Result Scenario(const Rig& aRig, int aSlideDbLo, int aSlideDbHi)
{
    Result r;
    for (int i = 0; i < 30; ++i)
    {
        const FrameOut o = Run(SprintFeed(8.2f), aRig);
        r.stepFail += o.stepped ? 0 : 1;
        if (o.contact[0] > 0.05f || o.contact[1] > 0.05f)
            ++r.preContactFrames;
    }
    float v = 10.8f;
    for (int i = 0; i < 72; ++i, v = std::max(2.73f, v - 2.75f / 60.0f))
    {
        const FrameOut o = Run(SlideFeed(v), aRig);
        ++r.slideFrames;
        r.stepFail += o.stepped ? 0 : 1;
        r.slideStepped += o.stepped ? 1 : 0;
        if (o.contact[0] != 0.0f || o.contact[1] != 0.0f)
            ++r.slideContactFrames;
        const int db = std::atoi(StatusField(o.status, "/db").c_str());
        if (db >= aSlideDbLo && db <= aSlideDbHi)
            ++r.pickSlideDb;
        else
            ++r.pickOtherDb;
        r.lastSlideStatus = o.status;
    }
    for (int i = 0; i < 20; ++i)
        r.afterStatus = Run(CrouchFeed(2.7f), aRig).status;
    r.statusSlideFrames = std::strtoull(StatusField(r.lastSlideStatus, "slideFrames=").c_str(), nullptr, 10);
    return r;
}

// "PSD_Slide_*" database indices of a chooser.json (slide.databases)
bool SlideDbRange(const std::string& aChooser, int& aLo, int& aHi)
{
    std::ifstream f(aChooser, std::ios::binary);
    std::stringstream ss;
    ss << f.rdbuf();
    utpp::json::Value root;
    std::string err;
    if (!utpp::json::Parse(ss.str(), root, err))
        return false;
    const auto* sl = root.Find("slide");
    const auto* dbs = sl ? sl->Find("databases") : nullptr;
    if (!dbs)
        return false;
    const auto* a = dbs->Find("PSD_Slide_FeetOut");
    const auto* b = dbs->Find("PSD_Slide_KneesOut");
    if (!a || !b)
        return false;
    aLo = static_cast<int>(std::min(a->num, b->num));
    aHi = static_cast<int>(std::max(a->num, b->num));
    return true;
}

bool Load(const motiondriver::Paths& aP, const Rig& aRig)
{
    motiondriver::SetEnabled(false);
    motiondriver::OnDisarmed();
    motiondriver::SetPathsOverride(g_body, aP);
    motiondriver::SetRigBody(g_body);
    motiondriver::OnArmed(kRigPtr, aRig.hashes, aRig.parents);
    motiondriver::SetEnabled(true);
    motionhost::FeedInput f = SprintFeed(0.0f);
    f.locoState = mm::integ::kLocoStand;
    (void)motiondriver::Step(f, 0, f.dt); // starts the load
    motiondriver::WaitLoader();
    (void)motiondriver::Step(f, 0, f.dt);
    motiondriver::WaitLoader();
    return motiondriver::Reason() == mm::adapter::Fallback::None;
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::printf("usage: utpp_motion_driver_slide_test <data dir> [--body man|woman]\n");
        return 2;
    }
    std::string plainDir; // folder with the plain bundles utpp_motion_<body>_noslide.* (test fixtures, not shipped data)
    for (int i = 2; i + 1 < argc; ++i)
    {
        if (!std::strcmp(argv[i], "--body"))
        {
            g_body = !std::strcmp(argv[i + 1], "man") ? 1 : 0;
            g_bodyName = g_body == 1 ? "man" : "woman";
        }
        else if (!std::strcmp(argv[i], "--plain-dir"))
            plainDir = argv[i + 1];
    }
    const std::string data = argv[1];
    if (plainDir.empty())
    {
        std::printf("usage: utpp_motion_driver_slide_test <data dir> [--body man|woman] --plain-dir <fixture dir>\n");
        return 2;
    }
    // the woman slide bundle is the twistfix one the game installs; the man slide bundle is utpp_motion_man_knee_slide (integration test)
    const motiondriver::Paths slide = DataPaths(data, g_body == 1 ? "utpp_motion_man_knee_slide" : "utpp_motion_woman_knee_slide");
    // the plain bundle (no slide section): the shipped databases carry the slide section, so it is the fixture of that body; its overlay,
    // style weights and traversal are the shipped ones
    motiondriver::Paths plain = DataPaths(data, g_body == 1 ? "utpp_motion_man_noslide" : "utpp_motion_woman_noslide");
    plain.files.db = plainDir + "\\" + (g_body == 1 ? "utpp_motion_man_noslide" : "utpp_motion_woman_noslide") + ".utppmm";
    plain.files.chooser = plainDir + "\\" + (g_body == 1 ? "utpp_motion_man_noslide" : "utpp_motion_woman_noslide") + ".chooser.json";
    for (const motiondriver::Paths* p : {&slide, static_cast<const motiondriver::Paths*>(&plain)})
        for (const std::string* f : {&p->files.db, &p->files.chooser, &p->files.overlay, &p->files.style})
            Check(Exists(*f), "data file present", *f);
    Rig rig;
    Check(RigFromDb(slide.files.db, rig) && rig.bones > motiondriver::kFirstBone, "rig from the slide database", std::to_string(rig.bones));
    int dbLo = -1, dbHi = -1;
    const bool rangeOk = SlideDbRange(slide.files.chooser, dbLo, dbHi) && dbLo >= 0;
    Check(rangeOk, "slide.databases of the slide chooser", std::to_string(dbLo) + ".." + std::to_string(dbHi));

    // ---- defaults: off, nothing owned
    motiondriver::SetDataDir(data + "\\does-not-exist");
    Check(!motiondriver::SlideSetting() && !motiondriver::OwnsSlide(5) && !motiondriver::SlideBundleDeclared(), "default: slide off, not owned, no bundle");
    // UTPPSlideOwned (SlideHandlingActive): the lock-free flag behind the GetCameraYaw wrap; false = vanilla camera exit
    Check(!motiondriver::SlideHandlingActive(), "default: SlideHandlingActive false");
    Check(motiondriver::SetSlide(true) && motiondriver::SlideSetting() && !motiondriver::OwnsSlide(5), "setting on without a bundle / motion off: OwnsSlide false");
    Check(!motiondriver::SlideHandlingActive(), "setting on without a bundle / motion off: SlideHandlingActive false");
    Check(motiondriver::SetSlide(false) && !motiondriver::SlideSetting(), "setting off again");

    // ---- slide bundle
    const bool slideLoaded = Load(slide, rig);
    Check(slideLoaded, "slide bundle loaded, motion active", mm::adapter::FallbackName(motiondriver::Reason()));
    Check(motiondriver::SlideBundleDeclared(), "slide bundle declares the slide section");
    Check(StatusField(motiondriver::Status(), "slideBundle=") == "1" && StatusField(motiondriver::Status(), "slide=") == "0", "status: slideBundle=1 slide=0",
          motiondriver::Status());
    Check(!motiondriver::OwnsSlide(5), "setting off: OwnsSlide(5) false");
    Check(!motiondriver::SlideHandlingActive(), "slide bundle, setting off: SlideHandlingActive false");
    motiondriver::SetSlide(true);
    Check(motiondriver::SlideHandlingActive(), "slide bundle, setting on: SlideHandlingActive true (any detail)");
    Check(motiondriver::OwnsSlide(5) && !motiondriver::OwnsSlide(4) && !motiondriver::OwnsSlide(3) && !motiondriver::OwnsSlide(6) && !motiondriver::OwnsSlide(-1),
          "setting on + slide bundle: OwnsSlide only for detail 5");
    motiondriver::SetEnabled(false);
    Check(!motiondriver::OwnsSlide(5), "motion switched off: OwnsSlide(5) false");
    Check(!motiondriver::SlideHandlingActive(), "motion switched off: SlideHandlingActive false");
    motiondriver::SetEnabled(true);
    (void)Run(SprintFeed(8.2f), rig);
    Check(motiondriver::OwnsSlide(5), "motion back on: OwnsSlide(5) true again");
    Check(motiondriver::SlideHandlingActive(), "motion back on: SlideHandlingActive true again");
    // disarm (MM left the player): reset at once, without waiting for a Step; re-arm + Step brings it back
    motiondriver::OnDisarmed();
    Check(!motiondriver::SlideHandlingActive() && !motiondriver::OwnsSlide(5), "OnDisarmed: SlideHandlingActive / OwnsSlide reset at once");
    motiondriver::OnArmed(kRigPtr, rig.hashes, rig.parents);
    (void)Run(SprintFeed(8.2f), rig);
    Check(motiondriver::SlideHandlingActive(), "re-armed + Step: SlideHandlingActive true again");

    const Result on = Scenario(rig, dbLo, dbHi);
    Check(on.stepFail == 0, "slide bundle, slide on: every frame produced a motion pose", std::to_string(on.stepFail));
    Check(on.slideContactFrames == 0, "slide on: no foot contacts on any slide frame", std::to_string(on.slideContactFrames));
    Check(on.preContactFrames > 0, "(the sprint before the slide does carry contacts)", std::to_string(on.preContactFrames));
    Check(on.statusSlideFrames == static_cast<uint64_t>(on.slideFrames), "status slideFrames counts the slide frames", on.lastSlideStatus);
    Check(on.pickSlideDb >= on.slideFrames - 3, "slide on: the slide databases are picked on the slide frames",
          std::to_string(on.pickSlideDb) + " of " + std::to_string(on.slideFrames) + " in db " + std::to_string(dbLo) + ".." + std::to_string(dbHi));
    std::printf("INFO slide on : %s\n", on.lastSlideStatus.c_str());
    std::printf("INFO after    : %s\n", on.afterStatus.c_str());

    motiondriver::SetSlide(false);
    const Result off = Scenario(rig, dbLo, dbHi);
    Check(off.stepFail == 0, "slide bundle, slide off: every frame produced a motion pose", std::to_string(off.stepFail));
    Check(off.statusSlideFrames == on.statusSlideFrames, "slide off: no further slide frames counted");
    Check(off.pickSlideDb == 0, "slide off: the slide databases are never picked", std::to_string(off.pickSlideDb));
    Check(off.slideContactFrames > 0 || off.slideStepped > 0, "slide off: slide frames take the pre-slide path (contacts as the held pose)");
    std::printf("INFO slide off: %s\n", off.lastSlideStatus.c_str());

    // ---- bundle without the slide section: the setting is a no-op
    // same scenario from the same fresh state, setting on and off: identical (the matcher is deterministic)
    auto plainRun = [&](bool aSlide, Result& aOut) {
        const bool ok = Load(plain, rig);
        motiondriver::SetSlide(aSlide);
        aOut = Scenario(rig, dbLo, dbHi);
        return ok;
    };
    Result plainOn, plainOff;
    const bool plainLoaded = plainRun(true, plainOn);
    Check(plainLoaded, "plain bundle loaded, motion active", mm::adapter::FallbackName(motiondriver::Reason()));
    Check(!motiondriver::SlideBundleDeclared(), "plain bundle does not declare slide");
    Check(!motiondriver::OwnsSlide(5), "plain bundle, setting on: OwnsSlide(5) false");
    Check(!motiondriver::SlideHandlingActive(), "database switch to a plain bundle, setting on: SlideHandlingActive false");
    (void)plainRun(false, plainOff);
    Check(plainOn.stepFail == 0 && plainOff.stepFail == 0, "plain bundle: every frame produced a motion pose");
    // the slideFrames counter is process-wide: the plain bundle adds none to what the slide bundle counted before
    Check(plainOn.statusSlideFrames == on.statusSlideFrames && plainOff.statusSlideFrames == on.statusSlideFrames &&
              plainOn.slideContactFrames == plainOff.slideContactFrames &&
              plainOn.preContactFrames == plainOff.preContactFrames && plainOn.pickSlideDb == plainOff.pickSlideDb &&
              StatusField(plainOn.lastSlideStatus, "pick=clip:") == StatusField(plainOff.lastSlideStatus, "pick=clip:") &&
              StatusField(plainOn.afterStatus, "pick=clip:") == StatusField(plainOff.afterStatus, "pick=clip:"),
          "plain bundle: setting on == off (no slide frames, same contacts, same clip picks)",
          "contacts " + std::to_string(plainOn.slideContactFrames) + "/" + std::to_string(plainOff.slideContactFrames) + ", pre " +
              std::to_string(plainOn.preContactFrames) + "/" + std::to_string(plainOff.preContactFrames) + ", slide pick " +
              StatusField(plainOn.lastSlideStatus, "pick=clip:") + " vs " + StatusField(plainOff.lastSlideStatus, "pick=clip:") +
              ", after pick " + StatusField(plainOn.afterStatus, "pick=clip:") + " vs " + StatusField(plainOff.afterStatus, "pick=clip:"));

    // ---- reload on map slide frames (MovementMode Sliding 2 of the slide bundle; recording 2026-10-26 11:28, rifle slide reloads):
    //      a RIFLE reload keeps the native reload-contact correction on the slide frames (ReloadCorrectionPose + the de-roll chain guard
    //      of FrameInputsLocked); a pistol reload stays outside the correction (the stationary rule). Mutation (tests/reloadfix.cmake)
    //      UTPP_RELOAD_RIFLE_MUTATION=4 MUST fail "slide reload: rifle ... correction".
    if (Load(slide, rig))
    {
        struct ReloadRun
        {
            int frames = 0, eligible = 0;
            float deRollMax = 0.0f;
        };
        const auto reloadSlide = [&](int32_t aPose) {
            ReloadRun r;
            motiondriver::SetSlide(true);
            (void)motiondriver::SetOverlayInputs(aPose, 1, false, false);
            (void)motiondriver::SetWeaponAction(0);
            const uint32_t weapon = mm::integ::kPsmWeaponKnown | mm::integ::kPsmWeaponDrawn;
            for (int i = 0; i < 30; ++i)
            {
                motionhost::FeedInput f = SprintFeed(8.2f);
                f.psmBlock |= weapon;
                (void)Run(f, rig);
            }
            (void)motiondriver::SetWeaponAction(2);
            float v = 10.8f;
            for (int i = 0; i < 60; ++i, v = std::max(2.73f, v - 2.75f / 60.0f))
            {
                motionhost::FeedInput f = SlideFeed(v);
                f.psmBlock |= weapon;
                const FrameOut o = Run(f, rig);
                mm::integ::PoseSnapshot s{};
                s.weight = 1.0f;
                s.numBones = 3;
                s.firstBone = 2;
                s.rig = kRigPtr;
                (void)motiondriver::ApplyToSnapshot(s);
                ++r.frames;
                r.eligible += s.reloadContactsEligible ? 1 : 0;
                r.deRollMax = std::max(r.deRollMax, static_cast<float>(std::atof(StatusField(o.status, "slideDeRollW=").c_str())));
            }
            (void)motiondriver::SetWeaponAction(0);
            for (int i = 0; i < 30; ++i)
            {
                motionhost::FeedInput f = CrouchFeed(2.7f);
                f.psmBlock |= weapon;
                (void)Run(f, rig);
            }
            return r;
        };
        const ReloadRun rifle = reloadSlide(1), pistol = reloadSlide(2);
        const std::string rd = "rifle eligible " + std::to_string(rifle.eligible) + "/" + std::to_string(rifle.frames) + " de-roll max " +
                               std::to_string(rifle.deRollMax) + " | pistol eligible " + std::to_string(pistol.eligible) + "/" +
                               std::to_string(pistol.frames) + " de-roll max " + std::to_string(pistol.deRollMax);
        std::printf("INFO slide reload: %s\n", rd.c_str());
        Check(rifle.eligible >= rifle.frames - 2, "slide reload: rifle keeps the native reload-contact correction on the map slide frames", rd);
        Check(pistol.eligible == 0, "slide reload: pistol gets no contact correction on slide frames (stationary rule, unchanged)", rd);
        // The knee-slide bundle rolls the MM shoulder line by less than the 22 deg of the roll range (de-roll weight 0 for both overlays,
        // like the KneesOut rifle reload 11:31:10 of the recording): the uprighting itself is covered by ReloadSlideRollHoldTest section 4.
        (void)motiondriver::SetOverlayInputs(0, 0, false, false);

        // ---- rifle CARRY on slide frames (no reload; recording 2026-10-27 15:14): the torso uprighting gate opens for a drawn rifle whose
        //      arms MM owns, only on slide frames (and the rolled exit latch); pistols never; ADS with the game upper body (not MM-owned)
        //      never; ADS with MM arms opens the gate with aim scale 0 (no edit). The knee bundle keeps the shoulder line under the 22 deg
        //      roll range on its slide frames but its exit (first crouch frames) rolls past it: the exit latch uprights that (deRollDeg on the
        //      exit). The levelling geometry / rates are covered by ReloadSlideRollHoldTest section 5 (mutation 8 disables the carry path).
        struct CarryRun
        {
            uint64_t sprint = 0, slide = 0, after = 0, late = 0;
            float deRollDegMax = 0.0f, afterDeRollDegMax = 0.0f, afterDePitchDegMax = 0.0f, scaleLast = -1.0f;
        };
        const auto carryCount = [] { return std::strtoull(StatusField(motiondriver::Status(), " slideCarry=").c_str(), nullptr, 10); };
        const auto carrySlide = [&](int32_t aPose, int32_t aAim, bool aGameUpperAim) {
            CarryRun r;
            motiondriver::SetSlide(true);
            motiondriver::SetWeaponUpper(false); // MM owns the arms of a drawn weapon (CET default mm.weaponUpper off)
            (void)motiondriver::SetAimGameUpper(aGameUpperAim);
            (void)motiondriver::SetOverlayInputs(aPose, aAim, false, false);
            (void)motiondriver::SetWeaponAction(0);
            const uint32_t weapon = mm::integ::kPsmWeaponKnown | mm::integ::kPsmWeaponDrawn;
            uint64_t c = carryCount();
            for (int i = 0; i < 30; ++i)
            {
                motionhost::FeedInput f = SprintFeed(8.2f);
                f.psmBlock |= weapon;
                (void)Run(f, rig);
            }
            r.sprint = carryCount() - c;
            c = carryCount();
            float v = 10.8f;
            for (int i = 0; i < 60; ++i, v = std::max(2.73f, v - 2.75f / 60.0f))
            {
                motionhost::FeedInput f = SlideFeed(v);
                f.psmBlock |= weapon;
                const FrameOut o = Run(f, rig);
                r.deRollDegMax = std::max(r.deRollDegMax, std::fabs(static_cast<float>(std::atof(StatusField(o.status, "slideDeRollDeg=").c_str()))));
                r.scaleLast = static_cast<float>(std::atof(StatusField(o.status, "slideCarryScale=").c_str()));
            }
            r.slide = carryCount() - c;
            c = carryCount();
            uint64_t c40 = 0;
            for (int i = 0; i < 60; ++i)
            {
                if (i == 40)
                    c40 = carryCount();
                motionhost::FeedInput f = CrouchFeed(2.7f);
                f.psmBlock |= weapon;
                const FrameOut o = Run(f, rig);
                r.afterDeRollDegMax = std::max(r.afterDeRollDegMax, std::fabs(static_cast<float>(std::atof(StatusField(o.status, "slideDeRollDeg=").c_str()))));
                r.afterDePitchDegMax = std::max(r.afterDePitchDegMax, std::fabs(static_cast<float>(std::atof(StatusField(o.status, "slideDePitchDeg=").c_str()))));
            }
            r.after = carryCount() - c;
            r.late = carryCount() - c40;
            return r;
        };
        const CarryRun rifleReady = carrySlide(1, 1, true), pistol2 = carrySlide(3, 1, true), pistol1 = carrySlide(2, 1, true),
                       rifleAdsGame = carrySlide(1, 2, true), rifleAdsMM = carrySlide(1, 2, false);
        const auto cd = [](const char* aN, const CarryRun& r) {
            return std::string(aN) + " sprint/slide/after " + std::to_string(r.sprint) + "/" + std::to_string(r.slide) + "/" +
                   std::to_string(r.after) + " (last 20: " + std::to_string(r.late) + ") deRollDeg max slide " +
                   std::to_string(r.deRollDegMax) + " exit " + std::to_string(r.afterDeRollDegMax) + " (de-pitch " +
                   std::to_string(r.afterDePitchDegMax) + ") scale " + std::to_string(r.scaleLast);
        };
        const std::string cdAll = cd("rifle", rifleReady) + " | " + cd("p2h", pistol2) + " | " + cd("p1h", pistol1) + " | " +
                                  cd("rifle ADS game upper", rifleAdsGame) + " | " + cd("rifle ADS MM", rifleAdsMM);
        std::printf("INFO slide carry: %s\n", cdAll.c_str());
        // after the slide only the exit latch may keep it (the knee exit clip stays rolled >= 22 deg for a while), then it is released
        Check(rifleReady.sprint == 0 && rifleReady.slide >= 55 && rifleReady.slide <= 60 && rifleReady.after <= 30 && rifleReady.late == 0 &&
                  rifleReady.afterDeRollDegMax > 10.0f,
              "slide carry: rifle Ready without a reload opens the uprighting gate on the slide frames (+ the rolled exit latch) only", cdAll);
        Check(pistol2.sprint + pistol2.slide + pistol2.after == 0 && pistol1.sprint + pistol1.slide + pistol1.after == 0 &&
                  pistol2.deRollDegMax + pistol2.afterDeRollDegMax == 0.0f && pistol1.deRollDegMax + pistol1.afterDeRollDegMax == 0.0f,
              "slide carry: pistol slides never open it (no torso edit)", cdAll);
        Check(rifleAdsGame.sprint + rifleAdsGame.slide + rifleAdsGame.after == 0 && rifleAdsGame.deRollDegMax + rifleAdsGame.afterDeRollDegMax == 0.0f,
              "slide carry: rifle ADS with the game upper body (arms not MM-owned): closed", cdAll);
        Check(rifleAdsMM.slide >= 55 && rifleAdsMM.scaleLast == 0.0f && rifleAdsMM.deRollDegMax + rifleAdsMM.afterDeRollDegMax == 0.0f,
              "slide carry: rifle ADS with MM arms: gate open, aim scale 0, no torso edit", cdAll);
        (void)motiondriver::SetAimGameUpper(true);
        (void)motiondriver::SetOverlayInputs(0, 0, false, false);
    }
    else
        Check(false, "slide reload: slide bundle reloaded for the reload block", mm::adapter::FallbackName(motiondriver::Reason()));

    motiondriver::SetEnabled(false);
    motiondriver::Shutdown();
    std::printf("slide driver: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
