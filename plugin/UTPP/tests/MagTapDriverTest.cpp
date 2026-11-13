// Magazine top-up gesture (src/mm/MagTap.hpp) through the real MotionDriver (src/MotionDriver.cpp without MotionHost.cpp / RED4ext, built
// with UTPP_MOTION_DRIVER_TEST_FAULTS for the same-frame pre / post probe TestMagTapProbe).
// Usage: utpp_magtap_driver_test <data dir (flat motion-derived files as MotionDriverTest)>
// Uses utpp_motion_woman_knee_slide.* (slide bundle: the gesture on slide frames too). Missing files FAIL.
// Checks:
//  I) CET mm.shinobiBonusReload (SetMagTapEnabled), default OFF: refused / no frame edited until switched on; on -> plays; switched off
//     with an accepted request -> never plays; the flag survives SetEnabled(false) + bundle reload (all later checks run with it on);
//  A) the WeaponRight frame of the real overlays (INFO: left grip, model up / forward, right hand in WeaponRight space);
//  B) per two-handed overlay (Rifle, Pistol2H Ready, standing; Pistol1H Ready: left hand ~0.95 m away -> no gesture, kMaxTravel):
//     FeedMagTap -> the gesture plays (edited frames, peak weight 1), the left hand
//     reaches the magazine target within 1 cm at the peak, the gesture offset of the hand changes by a bounded amount per frame,
//     every bone outside the left arm (right hand, WeaponRight, spine, legs) is bit-identical to the pre-gesture pose of the same frame,
//     the pose is bit-identical again after the gesture (0.45 s);
//  C) no request: no frame edited (bit-identical);
//  D) refusals: reload action, release, Default overlay -> FeedMagTap false; TPP off / gate closed for 3 Steps, ADS hand-back
//     (aimGameUpper), the game low-ready upper body (weaponUpper) -> the request is dropped and never plays later; a stale request
//     (> 0.25 s) is dropped;
//  E) a second request in the fall of the gesture: no weight jump (bounded per-frame change), the gesture plays again;
//  F) rifle on slide frames (mm.slide, slide bundle): the start waits for the slide de-roll (magtap::SlideDefer; the knee bundle
//     stays under the roll range, so the full kSlideDelaySeconds), then the gesture plays and reaches the target; standing requests
//     (B) start on the first Step; pistol requests on slide frames (G) never wait.
// Mutations (tests/magtap.cmake): UTPP_MAGTAP_MUTATION=1 (trigger ignored) MUST fail B; =2 (naive restart) MUST fail E;
// =3 (driver only: the mm.shinobiBonusReload setting ignored in FeedMagTap) MUST fail "I off by default"; =4 (no slide wait) MUST fail
// "F ... the start waits".
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "../src/MotionHost.hpp"
#include "../src/MotionDriver.hpp"
#include "../src/mm/MagTap.hpp"
#include "../src/mm/MotionDatabase.hpp"
#include "TestFailFast.hpp"

namespace mm = utpp::mm;
namespace motionhost = utpp::motionhost;
namespace motiondriver = utpp::motiondriver;
namespace magtap = utpp::mm::magtap;
using mm::math::Quat;
using mm::math::Vec3;

namespace
{
int g_pass = 0, g_fail = 0;
void Check(bool aOk, const char* aWhat, const std::string& aDetail = {})
{
    (aOk ? g_pass : g_fail)++;
    std::printf("%s %s%s%s\n", aOk ? "PASS" : "FAIL", aWhat, aDetail.empty() ? "" : " | ", aDetail.c_str());
    if (!aOk)
        UtppFailFastNote(std::string("FAIL ") + aWhat + (aDetail.empty() ? "" : " | ") + aDetail);
}
std::string F(double v) { char b[32]; std::snprintf(b, sizeof b, "%.4f", v); return b; }
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
    std::vector<std::string> names;
    uint32_t bones = 0;
    int Find(const char* aName) const
    {
        for (uint32_t i = 0; i < bones; ++i)
            if (names[i] == aName)
                return static_cast<int>(i);
        return -1;
    }
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
        aRig.names.emplace_back(db.BoneName(i));
    }
    return true;
}

motiondriver::Paths DataPaths(const std::string& aData, const std::string& aStem, const char* aBody = "woman")
{
    const std::string b(aBody);
    motiondriver::Paths p;
    p.files.db = aData + "\\" + aStem + ".utppmm";
    p.files.chooser = aData + "\\" + aStem + ".chooser.json";
    p.files.overlay = aData + "\\motion\\utpp_overlays_" + b + ".utppov";
    p.files.style = aData + "\\motion\\utpp_style_weights_" + b + ".txt";
    p.traversal = aData + "\\traversal\\utpp_traversal_" + b + ".utpptrav";
    return p;
}
bool Exists(const std::string& aPath) { return std::ifstream(aPath).good(); }

constexpr uint64_t kRigPtr = 0x1234567800ull;
constexpr uint32_t kWeapon = mm::integ::kPsmWeaponKnown | mm::integ::kPsmWeaponDrawn;

motionhost::FeedInput StandFeed(float aDt = 1.0f / 60.0f)
{
    motionhost::FeedInput f;
    f.locoState = mm::integ::kLocoStand;
    f.tppActive = true;
    f.psmBlock = kWeapon;
    f.dt = aDt;
    return f;
}
motionhost::FeedInput SlideFeed(float aSpeed)
{
    motionhost::FeedInput f;
    f.velWorld[1] = aSpeed;
    f.move[1] = 1.0f;
    f.locoState = 0;
    f.tppActive = true;
    f.psmBlock = kWeapon | mm::integ::kPsmDetailed | (6u << mm::integ::kPsmDetailShift);
    f.dt = 1.0f / 60.0f;
    return f;
}
motionhost::FeedInput SprintFeed(float aSpeed)
{
    motionhost::FeedInput f = SlideFeed(aSpeed);
    f.locoState = mm::integ::kLocoSprint;
    f.psmBlock = kWeapon;
    return f;
}

bool Load(const motiondriver::Paths& aP, const Rig& aRig, int32_t aBody = 0)
{
    motiondriver::SetEnabled(false);
    motiondriver::OnDisarmed();
    motiondriver::SetPathsOverride(aBody, aP);
    motiondriver::SetRigBody(aBody);
    motiondriver::OnArmed(kRigPtr, aRig.hashes, aRig.parents);
    motiondriver::SetEnabled(true);
    motionhost::FeedInput f = StandFeed();
    (void)motiondriver::Step(f, 0, f.dt);
    motiondriver::WaitLoader();
    (void)motiondriver::Step(f, 0, f.dt);
    motiondriver::WaitLoader();
    return motiondriver::Reason() == mm::adapter::Fallback::None;
}

// model transforms of a probe pose (database order, 7 floats per bone; parents precede children)
struct Model
{
    std::vector<Quat> q;
    std::vector<Vec3> p;
};
Model ModelOf(const std::vector<float>& aPose, const Rig& aRig)
{
    Model m;
    m.q.assign(aRig.bones, Quat{});
    m.p.assign(aRig.bones, Vec3{});
    for (uint32_t b = 0; b < aRig.bones && (b + 1) * 7 <= aPose.size(); ++b)
    {
        const float* d = &aPose[size_t(b) * 7];
        const Quat q{d[3], d[4], d[5], d[6]};
        const Vec3 t{d[0], d[1], d[2]};
        const int pa = aRig.parents[b];
        if (pa < 0)
        {
            m.q[b] = q;
            m.p[b] = t;
        }
        else
        {
            m.q[b] = mm::math::QuatNormalize(mm::math::QuatMul(m.q[pa], q));
            m.p[b] = m.p[pa] + mm::math::QuatRotate(m.q[pa], t);
        }
    }
    return m;
}
float Len(Vec3 v) { return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }
Vec3 InFrame(const Model& m, int aFrame, Vec3 aWorld) { return mm::math::QuatRotate(mm::math::QuatInv(m.q[aFrame]), aWorld - m.p[aFrame]); }
Vec3 DirInFrame(const Model& m, int aFrame, Vec3 aDir) { return mm::math::QuatRotate(mm::math::QuatInv(m.q[aFrame]), aDir); }
std::string V(Vec3 v) { return "(" + F(v.x) + "," + F(v.y) + "," + F(v.z) + ")"; }

struct Bones
{
    int lArm = -1, lFore = -1, lHand = -1, rHand = -1, weaponR = -1;
    std::vector<bool> leftArm; // bones the gesture may write: LeftArm, LeftForeArm, the children of LeftForeArm
};

// every bone outside the left arm bit-identical between the two poses
bool OthersIdentical(const std::vector<float>& a, const std::vector<float>& b, const Bones& aB, std::string& aWhy)
{
    if (a.size() != b.size())
    {
        aWhy = "size";
        return false;
    }
    for (size_t i = 0; i < a.size() / 7; ++i)
        if (!aB.leftArm[i] && std::memcmp(&a[i * 7], &b[i * 7], 7 * sizeof(float)) != 0)
        {
            aWhy = "bone " + std::to_string(i);
            return false;
        }
    return true;
}
bool AllIdentical(const std::vector<float>& a, const std::vector<float>& b)
{
    return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(float)) == 0;
}

struct GestureRun
{
    int edited = 0, frames = 0, othersBad = 0, notIdenticalAfter = 0, firstEdited = -1;
    float peakW = 0.0f, maxStepW = 0.0f, maxStepOffset = 0.0f, peakErr = 1e9f, peakMoved = 0.0f, targetDist = 0.0f;
    bool clampedAtPeak = false, endIdle = false;
    std::string othersWhy;
};

// Runs aFrames feeds; aRetriggerAt >= 0 requests a second gesture before that frame. FeedMagTap before frame 0.
GestureRun Gesture(const Rig& aRig, const Bones& aB, magtap::Kind aKind, int aFrames, int aRetriggerAt,
                   motionhost::FeedInput (*aFeed)(int), bool aTrigger = true)
{
    GestureRun r;
    if (aTrigger)
        (void)motiondriver::FeedMagTap();
    float lastW = 0.0f;
    Vec3 lastOff{};
    bool haveLast = false;
    for (int i = 0; i < aFrames; ++i)
    {
        if (i == aRetriggerAt)
            (void)motiondriver::FeedMagTap();
        const motionhost::FeedInput f = aFeed(i);
        (void)motiondriver::Step(f, 0, f.dt);
        const motiondriver::MagTapProbe pr = motiondriver::TestMagTapProbe();
        ++r.frames;
        if (!pr.stepped)
            continue;
        r.maxStepW = std::max(r.maxStepW, std::fabs(pr.weight - lastW));
        lastW = pr.weight;
        const Model mb = ModelOf(pr.before, aRig), ma = ModelOf(pr.after, aRig);
        const Vec3 off = ma.p[aB.lHand] - mb.p[aB.lHand]; // gesture offset of the hand in this frame
        if (haveLast)
            r.maxStepOffset = std::max(r.maxStepOffset, Len(off - lastOff));
        lastOff = off;
        haveLast = true;
        std::string why;
        if (!OthersIdentical(pr.before, pr.after, aB, why))
        {
            ++r.othersBad;
            r.othersWhy = why;
        }
        if (pr.edited)
        {
            ++r.edited;
            if (r.firstEdited < 0)
                r.firstEdited = i;
        }
        else if (!AllIdentical(pr.before, pr.after))
            ++r.notIdenticalAfter;
        if (pr.weight > r.peakW - 1e-6f)
        {
            r.peakW = pr.weight;
            const Vec3 target = ma.p[aB.weaponR] + mm::math::QuatRotate(ma.q[aB.weaponR], magtap::TargetOffset(aKind));
            r.peakErr = Len(ma.p[aB.lHand] - target);
            r.peakMoved = Len(off);
            r.targetDist = pr.targetDist;
            r.clampedAtPeak = pr.clamped;
        }
        r.endIdle = pr.weight == 0.0f && !pr.edited && AllIdentical(pr.before, pr.after);
    }
    return r;
}
std::string Desc(const GestureRun& r)
{
    return "edited " + std::to_string(r.edited) + "/" + std::to_string(r.frames) + " from frame " + std::to_string(r.firstEdited) + " peakW " + F(r.peakW) + " peak err " + F(r.peakErr) +
           " m, hand moved " + F(r.peakMoved) + " m (target dist " + F(r.targetDist) + (r.clampedAtPeak ? ", clamped" : "") +
           "), max step w " + F(r.maxStepW) + " offset " + F(r.maxStepOffset) + " m, others changed " + std::to_string(r.othersBad) +
           (r.othersWhy.empty() ? "" : " (" + r.othersWhy + ")") + ", unedited-but-changed " + std::to_string(r.notIdenticalAfter) +
           (r.endIdle ? ", ends identical" : ", NOT identical at the end");
}
motionhost::FeedInput Stand60(int) { return StandFeed(); }
} // namespace

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::printf("usage: utpp_magtap_driver_test <data dir>\n");
        return 2;
    }
    const std::string data = argv[1];
    const motiondriver::Paths slide = DataPaths(data, "utpp_motion_woman_knee_slide");
    for (const std::string* f : {&slide.files.db, &slide.files.chooser, &slide.files.overlay, &slide.files.style})
        Check(Exists(*f), "data file present", *f);
    Rig rig;
    Check(RigFromDb(slide.files.db, rig) && rig.bones > motiondriver::kFirstBone, "rig from the slide database", std::to_string(rig.bones));
    const auto makeBones = [](const Rig& aRig) {
        Bones b;
        b.lArm = aRig.Find("LeftArm");
        b.lFore = aRig.Find("LeftForeArm");
        b.lHand = aRig.Find("LeftHand");
        b.rHand = aRig.Find("RightHand");
        b.weaponR = aRig.Find("WeaponRight");
        bool parentsFirst = true;
        for (uint32_t i = 0; i < aRig.bones; ++i)
            parentsFirst = parentsFirst && aRig.parents[i] < static_cast<int>(i);
        Check(b.lArm >= 0 && b.lFore >= 0 && b.lHand >= 0 && b.rHand >= 0 && b.weaponR >= 0 && parentsFirst,
              "arm / weapon bones in the database (parents precede children)");
        b.leftArm.assign(aRig.bones, false);
        if (b.lArm >= 0 && b.lFore >= 0)
        {
            b.leftArm[static_cast<size_t>(b.lArm)] = b.leftArm[static_cast<size_t>(b.lFore)] = true;
            for (uint32_t i = 0; i < aRig.bones; ++i)
                if (aRig.parents[i] == b.lFore)
                    b.leftArm[i] = true;
        }
        return b;
    };
    const Bones bn = makeBones(rig);
    if (!Load(slide, rig))
    {
        Check(false, "slide bundle loaded, motion active", mm::adapter::FallbackName(motiondriver::Reason()));
        std::printf("magtap driver: %d passed, %d failed\n", g_pass, g_fail);
        return 1;
    }
    (void)motiondriver::SetSlide(true);
    motiondriver::SetWeaponUpper(false); // game default (CET mm.weaponUpper off): MM owns the arms of a drawn weapon
    const auto settle = [&](int aN) {
        for (int i = 0; i < aN; ++i)
        {
            const motionhost::FeedInput f = StandFeed();
            (void)motiondriver::Step(f, 0, f.dt);
        }
    };

    // ---- I) CET mm.shinobiBonusReload "Reload animation during Shinobi bonus", default OFF: a fresh driver (no SetMagTapEnabled call)
    // refuses every request and never edits a frame; on -> the gesture plays (the rest of this test runs with it on); a request
    // accepted while on and the option switched off before the next Step never plays; the flag survives SetEnabled(false) / Load.
    {
        (void)motiondriver::SetOverlayInputs(1, 1, false, false);
        settle(60);
        const std::string st0 = motiondriver::Status();
        Check(StatusField(st0, " magTapOn=") == "0", "I status: magTapOn=0 by default", StatusField(st0, " magTapOn="));
        const bool fedOff = motiondriver::FeedMagTap();
        const GestureRun off = Gesture(rig, bn, magtap::Kind::Rifle, 40, 20, &Stand60, false);
        Check(!fedOff && off.edited == 0 && off.peakW == 0.0f && off.notIdenticalAfter == 0,
              "I off by default: requests refused, no frame edited (bit-identical)", Desc(off));
        motiondriver::SetMagTapEnabled(true);
        settle(10);
        const GestureRun on = Gesture(rig, bn, magtap::Kind::Rifle, 40, -1, &Stand60);
        Check(on.edited >= 20 && on.peakW >= 0.999f && on.endIdle, "I on: the gesture plays", Desc(on));
        Check(motiondriver::FeedMagTap(), "I on: request accepted");
        motiondriver::SetMagTapEnabled(false);
        const GestureRun offPending = Gesture(rig, bn, magtap::Kind::Rifle, 40, -1, &Stand60, false);
        Check(offPending.edited == 0 && offPending.peakW == 0.0f,
              "I switched off before the next Step: the accepted request never plays", Desc(offPending));
        motiondriver::SetMagTapEnabled(true);
        motiondriver::SetEnabled(false);
        const bool reloaded = Load(slide, rig);
        const std::string st1 = motiondriver::Status();
        Check(reloaded && StatusField(st1, " magTapOn=") == "1", "I the setting survives SetEnabled(false) and a reload of the bundle",
              StatusField(st1, " magTapOn="));
        (void)motiondriver::SetSlide(true);
        motiondriver::SetWeaponUpper(false);
    }

    // ---- A) WeaponRight frame of the overlays
    struct Ov
    {
        int32_t pose;
        const char* name;
        magtap::Kind kind;
    };
    const Ov ovs[3] = {{1, "Rifle", magtap::Kind::Rifle}, {3, "Pistol2H", magtap::Kind::Pistol}, {2, "Pistol1H", magtap::Kind::Pistol}};
    for (const Ov& o : ovs)
    {
        (void)motiondriver::SetOverlayInputs(o.pose, 1, false, false);
        settle(90);
        const motiondriver::MagTapProbe pr = motiondriver::TestMagTapProbe();
        const Model m = ModelOf(pr.before, rig);
        std::printf("INFO %s Ready: leftHand in WeaponRight %s (|%s|), rightHand %s, model up %s, model fwd %s, model left %s, target %s\n",
                    o.name, V(InFrame(m, bn.weaponR, m.p[bn.lHand])).c_str(), F(Len(InFrame(m, bn.weaponR, m.p[bn.lHand]))).c_str(),
                    V(InFrame(m, bn.weaponR, m.p[bn.rHand])).c_str(), V(DirInFrame(m, bn.weaponR, Vec3{0, 0, 1})).c_str(),
                    V(DirInFrame(m, bn.weaponR, Vec3{0, 1, 0})).c_str(), V(DirInFrame(m, bn.weaponR, Vec3{-1, 0, 0})).c_str(),
                    V(magtap::TargetOffset(o.kind)).c_str());
    }

    // ---- B) the gesture per two-handed overlay, standing, Ready; Pistol1H (left hand ~0.95 m away) gets none (kMaxTravel)
    for (const Ov& o : ovs)
    {
        (void)motiondriver::SetOverlayInputs(o.pose, 1, false, false);
        settle(60);
        const GestureRun r = Gesture(rig, bn, o.kind, 40, -1, &Stand60);
        std::printf("INFO B %s: %s\n", o.name, Desc(r).c_str());
        if (o.pose == 2)
        {
            Check(r.edited == 0 && r.peakW == 0.0f && r.notIdenticalAfter == 0,
                  "B Pistol1H: left hand farther than kMaxTravel from the magazine -> no gesture (bit-identical)", Desc(r));
            continue;
        }
        const std::string what = std::string("B ") + o.name + ": the gesture plays (edited frames, peak weight 1)";
        Check(r.edited >= 20 && r.peakW >= 0.999f, what.c_str(), Desc(r));
        const std::string now = std::string("B ") + o.name + ": standing (no slide frame): the gesture starts on the first Step (no wait)";
        Check(r.firstEdited == 0 && StatusField(motiondriver::Status(), " magTapWait=").rfind("0.000000", 0) == 0, now.c_str(),
              Desc(r) + ", wait " + StatusField(motiondriver::Status(), " magTapWait="));
        const std::string reach = std::string("B ") + o.name + ": left hand within 1 cm of the magazine target at the peak";
        Check(r.peakErr <= 0.01f, reach.c_str(), Desc(r));
        // smoothstep slope 1.5 / kOutSeconds; the hand path of the weighted solve may bow a little: 30 % margin
        const float bound = 1.3f * 1.5f / magtap::kOutSeconds * (1.0f / 60.0f) * r.targetDist + 0.002f;
        const std::string step = std::string("B ") + o.name + ": per-frame change of the gesture offset bounded (60 Hz) <= " + F(bound) + " m";
        Check(r.maxStepOffset <= bound, step.c_str(), Desc(r));
        const std::string others = std::string("B ") + o.name + ": right hand, WeaponRight and every bone outside the left arm bit-identical";
        Check(r.othersBad == 0 && r.notIdenticalAfter == 0, others.c_str(), Desc(r));
        const std::string end = std::string("B ") + o.name + ": after 0.45 s the published pose is bit-identical again";
        Check(r.endIdle, end.c_str(), Desc(r));
    }

    // ---- C) no request: nothing edited
    {
        (void)motiondriver::SetOverlayInputs(1, 1, false, false);
        settle(30);
        const GestureRun r = Gesture(rig, bn, magtap::Kind::Rifle, 60, -1, &Stand60, false);
        Check(r.edited == 0 && r.notIdenticalAfter == 0 && r.peakW == 0.0f, "C no request: no frame edited, pose bit-identical", Desc(r));
    }

    // ---- D) refusals
    {
        (void)motiondriver::SetOverlayInputs(1, 1, false, false);
        settle(30);
        const std::string st0 = motiondriver::Status();
        (void)motiondriver::SetWeaponAction(2);
        Check(!motiondriver::FeedMagTap(), "D reload action: request refused");
        (void)motiondriver::SetWeaponAction(0);
        settle(60);
        (void)motiondriver::SetOverlayInputs(1, 1, true, false);
        Check(!motiondriver::FeedMagTap(), "D release: request refused");
        (void)motiondriver::SetOverlayInputs(0, 0, false, false);
        Check(!motiondriver::FeedMagTap(), "D Default overlay (no weapon): request refused");
        (void)motiondriver::SetOverlayInputs(1, 1, false, false);
        settle(60);
        // 3 Steps without ownership, then normal frames: the request must never play
        const auto blocked = [&](int aMode) {
            Check(motiondriver::FeedMagTap(), "D request accepted (decided by the next Steps)");
            for (int i = 0; i < 3; ++i)
            {
                motionhost::FeedInput f = StandFeed();
                uint32_t gate = 0;
                if (aMode == 0)
                    f.tppActive = false;
                else if (aMode == 1)
                    gate = 1;
                (void)motiondriver::Step(f, gate, f.dt);
            }
            return Gesture(rig, bn, magtap::Kind::Rifle, 40, -1, &Stand60, false);
        };
        const GestureRun fpp = blocked(0), gate = blocked(1);
        Check(fpp.edited == 0 && fpp.peakW == 0.0f, "D TPP off for 3 Steps: the request is dropped, never played later", Desc(fpp));
        Check(gate.edited == 0 && gate.peakW == 0.0f, "D gate closed (vehicle / scene) for 3 Steps: dropped, never played later", Desc(gate));
        // ADS with the game-owned upper body (aimGameUpper default on): not owned -> dropped
        (void)motiondriver::SetAimGameUpper(true);
        (void)motiondriver::SetOverlayInputs(1, 2, false, false);
        settle(30);
        const GestureRun ads = Gesture(rig, bn, magtap::Kind::Rifle, 40, -1, &Stand60);
        Check(ads.edited == 0 && ads.peakW == 0.0f, "D ADS with the game upper body: no gesture", Desc(ads));
        (void)motiondriver::SetOverlayInputs(1, 1, false, false);
        settle(60);
        // the game's low-ready upper body (mm.weaponUpper on): the game owns the arms -> no gesture
        motiondriver::SetWeaponUpper(true);
        settle(30);
        const GestureRun lowReady = Gesture(rig, bn, magtap::Kind::Rifle, 40, -1, &Stand60);
        Check(lowReady.edited == 0 && lowReady.peakW == 0.0f, "D game low-ready upper body (weaponUpper on): no gesture", Desc(lowReady));
        motiondriver::SetWeaponUpper(false);
        settle(60);
        // stale request: 0.3 s of (simulated) wall time between the refill and the next Step
        motiondriver::TestSetRecoilClock(100.0);
        Check(motiondriver::FeedMagTap(), "D stale: request accepted");
        motiondriver::TestAdvanceRecoilClock(0.30);
        const GestureRun stale = Gesture(rig, bn, magtap::Kind::Rifle, 40, -1, &Stand60, false);
        motiondriver::TestSetRecoilClock(-1.0);
        Check(stale.edited == 0 && stale.peakW == 0.0f, "D a request older than 0.25 s is dropped", Desc(stale));
        const std::string st1 = motiondriver::Status();
        std::printf("INFO D status magTap= %s -> %s\n", StatusField(st0, " magTap=").c_str(), StatusField(st1, " magTap=").c_str());
    }

    // ---- E) second request in the fall of the gesture (frame 22 of 60 Hz = 0.37 s: weight ~0.5 falling)
    {
        (void)motiondriver::SetOverlayInputs(1, 1, false, false);
        settle(60);
        const GestureRun r = Gesture(rig, bn, magtap::Kind::Rifle, 70, 22, &Stand60);
        std::printf("INFO E: %s\n", Desc(r).c_str());
        const float wBound = 1.5f / magtap::kOutSeconds / 60.0f + 1e-3f;
        Check(r.maxStepW <= wBound, "E second request in the fall: no weight jump (per-frame change <= smoothstep slope)",
              "max step " + F(r.maxStepW) + " bound " + F(wBound));
        Check(r.edited >= 40 && r.endIdle && r.othersBad == 0, "E the gesture plays again and ends bit-identical", Desc(r));
    }

    // ---- F) rifle on slide frames
    {
        (void)motiondriver::SetOverlayInputs(1, 1, false, false);
        for (int i = 0; i < 30; ++i)
        {
            const motionhost::FeedInput f = SprintFeed(8.2f);
            (void)motiondriver::Step(f, 0, f.dt);
        }
        const uint64_t deferred0 = std::strtoull(StatusField(motiondriver::Status(), " magTapDeferred=").c_str(), nullptr, 10);
        const GestureRun r = Gesture(rig, bn, magtap::Kind::Rifle, 70, 0, [](int i) {
            return SlideFeed(std::max(2.73f, 10.8f - 2.75f * static_cast<float>(i) / 60.0f));
        }, false);
        const std::string st = motiondriver::Status();
        const float wait = std::strtof(StatusField(st, " magTapWait=").c_str(), nullptr);
        const uint64_t deferred1 = std::strtoull(StatusField(st, " magTapDeferred=").c_str(), nullptr, 10);
        std::printf("INFO F slide: wait %.4f s (de-roll weight at the end %s), %s\n", wait,
                    StatusField(st, " slideDeRollW=").c_str(), Desc(r).c_str());
        Check(r.edited >= 20 && r.peakW >= 0.999f && r.peakErr <= 0.01f && r.othersBad == 0 && r.endIdle,
              "F rifle on slide frames: the gesture plays and reaches the magazine within 1 cm, nothing else moves", Desc(r));
        // the knee-slide bundle keeps the MM shoulder line under the 22 deg roll range in its first second of slide frames (the carry
        // de-roll weight stays 0 there), so the request waits the full magtap::kSlideDelaySeconds (21 Steps at 60 Hz), then starts
        Check(r.firstEdited >= 19 && r.firstEdited <= 22 && wait >= magtap::kSlideDelaySeconds - 1.0f / 60.0f - 1e-4f &&
                  wait <= magtap::kSlideDelaySeconds + 1.0f / 60.0f + 1e-4f && deferred1 == deferred0 + 1,
              "F rifle on slide frames: the start waits for the slide de-roll (here kSlideDelaySeconds), then plays",
              Desc(r) + ", wait " + F(wait) + ", deferred " + std::to_string(deferred0) + " -> " + std::to_string(deferred1));
    }
    // ---- F2) the waiting request keeps the drop rules: (a) with the (test) wall clock advancing with the feed it outlives the 0.25 s
    //      window and starts (window + kSlideDelaySeconds); (b) a long gap without Steps (0.7 s) still drops it; (c) ownership lost
    //      during the wait (TPP off for 3 Steps) drops it, it never plays later.
    {
        (void)motiondriver::SetOverlayInputs(1, 1, false, false);
        const auto sprint = [&] {
            // 1 s: the knee slide exit stays rolled >= 22 deg for ~.47 s after a slide (exit latch), then the de-roll releases (.25 s)
            for (int i = 0; i < 60; ++i)
            {
                const motionhost::FeedInput f = SprintFeed(8.2f);
                (void)motiondriver::Step(f, 0, f.dt);
            }
        };
        const auto slideRun = [&](int aFrames, int aGapAt, int aFppFrom, int aFppTo) {
            GestureRun r;
            for (int i = 0; i < aFrames; ++i)
            {
                motiondriver::TestAdvanceRecoilClock(i == aGapAt ? 0.70 : 1.0 / 60.0);
                motionhost::FeedInput f = SlideFeed(std::max(2.73f, 10.8f - 2.75f * static_cast<float>(i) / 60.0f));
                if (i >= aFppFrom && i < aFppTo)
                    f.tppActive = false;
                (void)motiondriver::Step(f, 0, f.dt);
                const motiondriver::MagTapProbe pr = motiondriver::TestMagTapProbe();
                ++r.frames;
                if (pr.edited && r.firstEdited < 0)
                    r.firstEdited = i;
                r.edited += pr.edited ? 1 : 0;
                r.peakW = std::max(r.peakW, pr.weight);
            }
            return r;
        };
        motiondriver::TestSetRecoilClock(200.0);
        sprint();
        Check(StatusField(motiondriver::Status(), " slideDeRollW=").rfind("0.000000", 0) == 0, "F2 de-roll released before the slide",
              StatusField(motiondriver::Status(), " slideDeRollW="));
        Check(motiondriver::FeedMagTap(), "F2 request accepted");
        const GestureRun a = slideRun(60, -1, -1, -1);
        sprint();
        Check(motiondriver::FeedMagTap(), "F2 request accepted");
        const GestureRun b = slideRun(60, 5, -1, -1);
        sprint();
        Check(motiondriver::FeedMagTap(), "F2 request accepted");
        const GestureRun c = slideRun(60, -1, 5, 8);
        motiondriver::TestSetRecoilClock(-1.0);
        Check(a.firstEdited >= 19 && a.firstEdited <= 22 && a.peakW >= 0.999f,
              "F2 a waiting request outlives the 0.25 s window (wall clock with the feed) and starts after the wait", Desc(a));
        Check(b.edited == 0 && b.peakW == 0.0f, "F2 a 0.7 s gap without Steps during the wait drops the request", Desc(b));
        Check(c.edited == 0 && c.peakW == 0.0f, "F2 TPP off for 3 Steps during the wait drops the request, it never plays later", Desc(c));
    }

    // ---- G) Relaxed aim (out of combat: PSM.Combat 0, RightWeapon 5 -> aim Relaxed, as in the 11:28 refill lines) on slide frames
    for (const Ov& o : ovs)
    {
        (void)motiondriver::SetOverlayInputs(o.pose, 0, false, false);
        for (int i = 0; i < 45; ++i)
        {
            const motionhost::FeedInput f = SprintFeed(8.2f);
            (void)motiondriver::Step(f, 0, f.dt);
        }
        const uint64_t deferred0 = std::strtoull(StatusField(motiondriver::Status(), " magTapDeferred=").c_str(), nullptr, 10);
        const GestureRun r = Gesture(rig, bn, o.kind, 70, 0, [](int i) {
            return SlideFeed(std::max(2.73f, 10.8f - 2.75f * static_cast<float>(i) / 60.0f));
        }, false);
        const std::string st = motiondriver::Status();
        const uint64_t deferred1 = std::strtoull(StatusField(st, " magTapDeferred=").c_str(), nullptr, 10);
        if (o.kind == magtap::Kind::Pistol)
        {
            const std::string nw = std::string("G ") + o.name + " on slide frames: a pistol request never waits for the slide de-roll";
            Check(deferred1 == deferred0, nw.c_str(), std::to_string(deferred0) + " -> " + std::to_string(deferred1));
        }
        std::printf("INFO G %s Relaxed slide: travel %s, %s\n", o.name, StatusField(st, " magTapTravel=").c_str(), Desc(r).c_str());
        const float travel = std::strtof(StatusField(st, " magTapTravel=").c_str(), nullptr);
        const bool plays = r.edited >= 20 && r.peakW >= 0.999f && r.peakErr <= 0.01f && r.othersBad == 0;
        const bool none = r.edited == 0 && r.peakW == 0.0f && r.notIdenticalAfter == 0;
        const std::string what = std::string("G ") + o.name + " Relaxed on slide frames: plays within kMaxTravel, else no edit at all";
        Check(travel <= magtap::kMaxTravel ? plays : none, what.c_str(), Desc(r));
    }
    // ---- H) male V (man knee slide bundle, man overlays): the same offsets reach the magazine, nothing else moves
    {
        const motiondriver::Paths man = DataPaths(data, "utpp_motion_man_knee_slide", "man");
        Rig mrig;
        const bool okMan = Exists(man.files.db) && Exists(man.files.overlay) && RigFromDb(man.files.db, mrig) && Load(man, mrig, 1);
        Check(okMan, "H man bundle loaded, motion active", mm::adapter::FallbackName(motiondriver::Reason()));
        if (okMan)
        {
            const Bones mb = makeBones(mrig);
            (void)motiondriver::SetSlide(true);
            motiondriver::SetWeaponUpper(false);
            for (const Ov& o : ovs)
            {
                if (o.pose == 2)
                    continue;
                (void)motiondriver::SetOverlayInputs(o.pose, 1, false, false);
                settle(90);
                const GestureRun r = Gesture(mrig, mb, o.kind, 40, -1, &Stand60);
                const std::string st = motiondriver::Status();
                std::printf("INFO H man %s: travel %s, %s\n", o.name, StatusField(st, " magTapTravel=").c_str(), Desc(r).c_str());
                const std::string what = std::string("H man ") + o.name + ": the gesture plays, reaches the magazine within 1 cm, nothing else moves";
                Check(r.edited >= 20 && r.peakW >= 0.999f && r.peakErr <= 0.01f && r.othersBad == 0 && r.endIdle, what.c_str(), Desc(r));
            }
        }
    }
    std::printf("INFO status: %s\n", motiondriver::Status().c_str());
    motiondriver::SetEnabled(false);
    motiondriver::Shutdown();
    std::printf("magtap driver: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
