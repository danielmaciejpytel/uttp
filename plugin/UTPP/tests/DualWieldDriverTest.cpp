// Dual wield recoil per arm through the real driver (MotionDriver.cpp + MotionHost.cpp): two handguns drawn (psmBlock dual flag, the
// UpperBody field left unreported as the game does for DualWield 7), the weapon overlay up, an authored recoil profile on the right
// weapon (SetRecoilProfile) and a profile on the left weapon (FeedRecoilShotLeft).
// Every case runs in its own child process with the same feed; a run without any shot is the reference, so the kick of a hand is
// its difference to that run (chest frame, Spine3), frame by frame: the hand position (cm) and the turn of the weapon bone held by
// that hand (WeaponLeft / WeaponRight, deg; short handgun clips are mostly a turn of the weapon about the grip).
//  - left shot (FeedRecoilShotLeft): the left hand and the left weapon move by at least a third of the right hand's / right weapon's
//    kick of the SAME profile (a right-shot run with that profile on the right weapon), no frame step larger than that right kick's
//    largest step + 0.3 cm, and they return (< 0.1 cm / 0.2 deg over the last 0.15 s);
//  - right shot (FeedRecoilShot): the left hand moves by at most 0.5 cm;
//  - both: both hands kick;
//  - every frame of every two-weapon run: the arms stay on the driver's pose (upperScale > 0.99) and no shot event wait expires
//    (a left shot raises the weapon action 8 without a right pulse);
//  - holster during a left kick: the left hand difference has no step larger than the reference bound (the kick fades with the mirror);
//  - one weapon drawn: every published pose is bit-identical with and without the left feed, and the left feed is counted dropped.
// States: hip fire and aimed fire, standing and moving, both bodies; profile pairs: Liberty in both hands (2 / 2), the pair of the
// in-game dual wield runs (Liberty right, Lexington left: 2 / 6) and Lexington in both hands (6 / 6). The shot pulse is fed between
// two frames with the weapon action 8 raised on the frame before (the action edge, then the shot event).
// Usage: utpp_dualwield_driver_test <data dir> [ignored]
// UTPP_DUALWIELD_KICK_MUTATION=1 (the left kick is never applied to the left arm) must print a FAIL line.
#define main UnusedIntegrationMain
#include "MotionDriverIntegrationTest.cpp"
#undef main

namespace utpp::motiondriver
{
bool SetAimWorldSmoothing(bool aOn, float aHz);
} // namespace utpp::motiondriver

namespace
{
Xf ModelXf(const mm::integ::PoseSnapshot& s, const Rig& aR, int32_t aBone, bool& aOk)
{
    int32_t path[64];
    int n = 0;
    aOk = false;
    for (int32_t b = aBone; b >= 0 && n < 64; b = aR.parents[size_t(b)])
    {
        if (static_cast<uint32_t>(b) >= s.numBones)
            return {};
        path[n++] = b;
    }
    Xf g;
    for (int i = n - 1; i >= 0; --i)
        g = Mul(g, &s.pose[size_t(path[i]) * 7]);
    aOk = true;
    return g;
}
void QMulRaw(const float* a, const float* b, float* r)
{
    r[0] = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
    r[1] = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];
    r[2] = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];
    r[3] = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
}
// position of aBone in the frame of aChest (cm) and its rotation in that frame (quaternion)
bool InChest(const mm::integ::PoseSnapshot& s, const Rig& aR, int32_t aChest, int32_t aBone, float* aPos, float* aRot)
{
    bool okC = false, okB = false;
    const Xf c = ModelXf(s, aR, aChest, okC), b = ModelXf(s, aR, aBone, okB);
    if (!okC || !okB)
        return false;
    const float d[3] = {b.p[0] - c.p[0], b.p[1] - c.p[1], b.p[2] - c.p[2]};
    const float inv[4] = {-c.q[0], -c.q[1], -c.q[2], c.q[3]};
    QRot(inv, d, aPos);
    for (int k = 0; k < 3; ++k)
        aPos[k] *= 100.0f;
    QMulRaw(inv, b.q, aRot);
    return true;
}
bool Has(const std::string& aCase, const char* aToken) { return aCase.find(aToken) != std::string::npos; }
// "-pl<N>" / "-pr<N>": profile id of the left / right weapon (default 2)
int32_t ProfileToken(const std::string& aCase, const char* aToken)
{
    const size_t at = aCase.find(aToken);
    return at == std::string::npos ? 2 : std::atoi(aCase.c_str() + at + std::strlen(aToken));
}

constexpr uint32_t kRightEpoch = 702, kLeftEpoch = 9001;
constexpr double kSettleS = 2.0, kRecordS = 1.2, kShotS = 0.217, kHolsterAtS = 0.05;

// Child: <exe> --dual-child <data> <db> <out> <hz> <case>. Case tokens: single | dual, -aim, -move, -man, -pl<N> / -pr<N> (profiles),
// the shots -right / -left / -both / -none, -leftfeed (single weapon: the left feed is called too), -holster (two weapons put away
// 0.05 s after the shot).
int DualChildMain(const std::string& aData, const std::string& aDb, const std::string& aOut, int aHz, const std::string& aCase)
{
    const bool man = Has(aCase, "-man"), aim = Has(aCase, "-aim"), moving = Has(aCase, "-move");
    bool dual = Has(aCase, "dual");
    const bool shotRight = Has(aCase, "-right") || Has(aCase, "-both"), shotLeft = Has(aCase, "-left") || Has(aCase, "-both");
    const bool leftFeed = shotLeft || Has(aCase, "-leftfeed");
    const bool holster = Has(aCase, "-holster");
    const int32_t profileLeft = ProfileToken(aCase, "-pl"), profileRight = ProfileToken(aCase, "-pr");
    // "-early<N>": the shot pulses come N frames BEFORE the weapon action 8 (a shot event ahead of the action publication)
    const size_t earlyAt = aCase.find("-early");
    const int early = earlyAt == std::string::npos ? 0 : std::max(0, std::atoi(aCase.c_str() + earlyAt + 6));
    motionhost::SetAimUpperFade(true);
    motiondriver::SetAimUpperFade(true);
    motionhost::SetWeaponUpper(true);
    motiondriver::SetWeaponUpper(true);
    motiondriver::SetPathsOverride(0, DataPaths(aData, "woman"));
    motiondriver::SetPathsOverride(1, DataPaths(aData, "man"));
    motionhost::SetMotionHooks(&kHooks);
    motiondriver::SetRigBody(0);
    motiondriver::SetOverlayInputs(2, 0, false, false);
    motiondriver::SetEnabled(true);
    motiondriver::Step(motionhost::FeedInput{}, 0, 1.0f / 60.0f);
    motiondriver::WaitLoader();
    motiondriver::SetEnabled(false);
    mm::Database db;
    std::string err;
    if (!db.LoadFromFile(aDb.c_str(), motionhost::kExpectedRigHash, err) ||
        !motionhost::LoadDatabase(std::wstring(aDb.begin(), aDb.end()), "test"))
    {
        std::ofstream(aOut) << "database not loaded " << err << "\n";
        return 3;
    }
    std::vector<uint64_t> hashes;
    std::vector<int16_t> parents;
    for (uint32_t b = 0; b < db.NumBones(); ++b)
    {
        hashes.push_back(mm::integ::CNameHash(db.BoneName(b)));
        parents.push_back(static_cast<int16_t>(db.BoneParent(b)));
    }
    static int rig = 0;
    g_hot.armed.store(true, std::memory_order_release);
    motionhost::OnArmed(&rig, hashes, parents);
    motionhost::SetEnabled(true);
    motionhost::SetUpperMask(true);
    motionhost::SetUpperMaskStates(7);
    motionhost::SetHoldPose(true);
    const float dt = 1.0f / static_cast<float>(aHz);
    uint32_t weaponBits = 0;
    bool simClock = false;
    auto feed = [&]() {
        if (simClock)
            motiondriver::TestAdvanceRecoilClock(dt);
        motionhost::FeedInput in;
        in.locoState = mm::integ::kLocoStand;
        in.tppActive = true;
        in.dt = dt;
        in.haveAir = true;
        in.onGround = true;
        in.distToGround = -2.0f;
        if (moving)
        {
            in.velWorld[1] = 2.5f;
            in.move[1] = 1.0f;
        }
        // two weapons: PSM.UpperBody DualWield (7) is not reported in the 3-bit field (0), the dual flag is set; one weapon: Ready / Aim
        const uint32_t upper = dual ? 0u : ((aim ? 6u : 0u) + 1u) << mm::integ::kPsmUpperShift;
        in.psmBlock = upper | weaponBits | (dual ? mm::integ::kPsmDualFlag : 0u);
        motiondriver::FeedAirFromInput(in);
        motionhost::Feed(in);
    };
    for (int i = 0; i < 45; ++i)
        feed();
    Rig sourceRig;
    std::unique_ptr<mm::motion::MotionDatabase> sourceDb;
    if (!RigFromDb(DataPaths(aData, man ? "man" : "woman").files.db, sourceRig, &sourceDb))
        return 34;
    int32_t chest = -1, handL = -1, handR = -1, weaponL = -1, weaponR = -1;
    for (uint32_t i = 0; i < sourceDb->NumBones(); ++i)
    {
        const std::string_view n = sourceDb->BoneName(i);
        if (n == "Spine3") chest = static_cast<int32_t>(i);
        if (n == "LeftHand") handL = static_cast<int32_t>(i);
        if (n == "RightHand") handR = static_cast<int32_t>(i);
        if (n == "WeaponLeft") weaponL = static_cast<int32_t>(i);
        if (n == "WeaponRight") weaponR = static_cast<int32_t>(i);
    }
    if (chest < 0 || handL < 0 || handR < 0 || weaponL < 0 || weaponR < 0)
        return 35;
    motiondriver::SetRigBody(man ? 1 : 0);
    motiondriver::SetOverlayInputs(3, aim ? 2 : 1, false, false); // Pistol2H, turned into the one-handed pose by the dual flag
    motiondriver::SetAimGameUpper(false);
    motionhost::SetWeaponUpper(false);
    motiondriver::SetWeaponUpper(false);
    motiondriver::SetGripEnabled(true);
    motiondriver::SetShootTransferEnabled(true);
    motiondriver::SetSlide(false);
    motiondriver::SetEnabled(true);
    motiondriver::Step(motionhost::FeedInput{}, 0, dt);
    motiondriver::WaitLoader();
    motionhost::OnArmed(&rig, sourceRig.hashes, sourceRig.parents);
    weaponBits = mm::integ::kPsmWeaponKnown | mm::integ::kPsmWeaponDrawn;
    motiondriver::SetRecoilProfile(kRightEpoch, profileRight);
    simClock = true;
    motiondriver::TestSetRecoilClock(1000.0);
    const int settle = static_cast<int>(std::lround(kSettleS * aHz));
    for (int i = 0; i < settle; ++i)
        feed();
    std::ofstream out(aOut);
    const std::string settled = motiondriver::Status();
    out << "SETTLED mirror=" << StatusField(settled, "dualMirror=") << " profileReady=" << StatusField(settled, "recoilProfileReady=")
        << " wait=" << StatusField(settled, "shotWaitExpired=") << "\n";
    const int record = static_cast<int>(std::lround(kRecordS * aHz)), shotFrames = static_cast<int>(std::lround(kShotS * aHz));
    const int holsterAt = static_cast<int>(std::lround(kHolsterAtS * aHz));
    for (int i = 0; i < record; ++i)
    {
        if (i == 0)
        {
            // the action edge first (the weapon enters Shoot), then the shot event of each weapon before the next frame;
            // -early<N>: the shot events first, the action edge N frames later
            if (early == 0)
                motiondriver::SetWeaponAction(8);
            feed();
            if (shotRight)
                motiondriver::FeedRecoilShot(kRightEpoch, false);
            if (leftFeed)
                motiondriver::FeedRecoilShotLeft(kLeftEpoch, profileLeft, false);
        }
        else
        {
            if (early > 0 && i == early)
                motiondriver::SetWeaponAction(8);
            if (i == shotFrames + early)
                motiondriver::SetWeaponAction(0);
            if (holster && i == holsterAt)
                dual = false;
            feed();
        }
        float l[3] = {0, 0, 0}, r[3] = {0, 0, 0}, lw[3] = {0, 0, 0}, rw[3] = {0, 0, 0};
        float lq[4] = {0, 0, 0, 1}, rq[4] = {0, 0, 0, 1}, unused[4] = {0, 0, 0, 1};
        const bool pub = ReadPublished();
        const bool ok = pub && InChest(g_read, sourceRig, chest, handL, l, unused) && InChest(g_read, sourceRig, chest, handR, r, unused) &&
                        InChest(g_read, sourceRig, chest, weaponL, lw, lq) && InChest(g_read, sourceRig, chest, weaponR, rw, rq);
        const std::string st = motiondriver::Status();
        char line[768];
        std::snprintf(line, sizeof line,
                      "F %d %d %.5f %.5f %.5f %.5f %.5f %.5f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.7f %.4f %s %s %llu", i, ok ? 1 : 0,
                      l[0], l[1], l[2], r[0], r[1], r[2], lq[0], lq[1], lq[2], lq[3], rq[0], rq[1], rq[2], rq[3],
                      pub ? static_cast<double>(g_read.upperScale) : -1.0, StatusField(st, "dualLeftKick=").c_str(),
                      StatusField(st, "dualMirror=").c_str(), static_cast<unsigned long long>(pub ? SnapHash(g_read) : 0));
        out << line << "\n";
    }
    const std::string st = motiondriver::Status();
    out << "END dualLeftShot=" << StatusField(st, "dualLeftShot=") << " dualMirror=" << StatusField(st, "dualMirror=")
        << " dualLeftProfile=" << StatusField(st, "dualLeftProfile=") << " dualLeftKickFrames=" << StatusField(st, "dualLeftKickFrames=")
        << " dualLeftKickPeak=" << StatusField(st, "dualLeftKickPeak=") << " dualLeftSkip=" << StatusField(st, "dualLeftSkip=")
        << " dualPreMirrorFrames=" << StatusField(st, "dualPreMirrorFrames=") << " dualNativeFrames=" << StatusField(st, "dualNativeFrames=")
        << " dualWaitExpired=" << StatusField(st, "dualWaitExpired=") << " wait=" << StatusField(st, "shotWaitExpired=") << "\n";
    out.close();
    motiondriver::SetEnabled(false);
    motionhost::SetMotionHooks(nullptr);
    motiondriver::Shutdown();
    motionhost::Shutdown();
    return 0;
}

struct DualRun
{
    std::string head, end;
    std::vector<std::array<float, 14>> f; // left hand xyz, right hand xyz (cm), left weapon quat, right weapon quat (chest frame)
    std::vector<float> upper, kick, mirror;
    std::vector<uint64_t> hash;
    bool ok = false;
};
DualRun ReadRun(const ChildRef& aKid)
{
    DualRun r;
    const DWORD code = WaitChild(aKid);
    std::ifstream in(aKid.out);
    std::string line;
    bool allValid = true;
    while (std::getline(in, line))
    {
        if (line.rfind("SETTLED", 0) == 0)
            r.head = line;
        else if (line.rfind("END", 0) == 0)
            r.end = line;
        else if (line.rfind("F ", 0) == 0)
        {
            std::istringstream s(line.substr(2));
            int i = 0, ok = 0;
            std::array<float, 14> h{};
            float upper = -1.0f;
            std::string kick, mirror;
            unsigned long long hash = 0;
            s >> i >> ok;
            for (float& v : h)
                s >> v;
            s >> upper >> kick >> mirror >> hash;
            allValid = allValid && ok == 1;
            r.f.push_back(h);
            r.upper.push_back(upper);
            r.kick.push_back(kick.empty() ? -1.0f : std::stof(kick));
            r.mirror.push_back(mirror.empty() ? -1.0f : std::stof(mirror));
            r.hash.push_back(hash);
        }
        else if (r.head.empty())
            r.head = line;
    }
    in.close();
    DeleteFileA(aKid.out.c_str());
    r.ok = code == 0 && allValid && !r.f.empty();
    if (code != 0)
        r.head += " (exit code " + std::to_string(code) + ")";
    return r;
}
std::string FieldOf(const std::string& aLine, const char* aKey) { return StatusField(aLine, aKey); }
// per-frame displacement of one hand against the reference run (cm), aSide 0 left / 1 right
std::vector<std::array<float, 3>> HandDelta(const DualRun& a, const DualRun& aRef, int aSide)
{
    std::vector<std::array<float, 3>> d;
    const size_t n = std::min(a.f.size(), aRef.f.size());
    for (size_t i = 0; i < n; ++i)
    {
        const int o = aSide * 3;
        d.push_back({a.f[i][o] - aRef.f[i][o], a.f[i][o + 1] - aRef.f[i][o + 1], a.f[i][o + 2] - aRef.f[i][o + 2]});
    }
    return d;
}
std::vector<float> Lengths(const std::vector<std::array<float, 3>>& d)
{
    std::vector<float> v;
    for (const auto& x : d)
        v.push_back(std::sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]));
    return v;
}
std::vector<float> HandDiff(const DualRun& a, const DualRun& aRef, int aSide) { return Lengths(HandDelta(a, aRef, aSide)); }
// largest per-frame step of the displacement vector of one hand (cm)
float MaxStep(const DualRun& a, const DualRun& aRef, int aSide)
{
    const auto d = HandDelta(a, aRef, aSide);
    float m = 0.0f;
    for (size_t i = 1; i < d.size(); ++i)
    {
        const float x = d[i][0] - d[i - 1][0], y = d[i][1] - d[i - 1][1], z = d[i][2] - d[i - 1][2];
        m = std::max(m, std::sqrt(x * x + y * y + z * z));
    }
    return m;
}
// per-frame turn of one weapon bone against the reference run (deg)
std::vector<float> WeaponTurn(const DualRun& a, const DualRun& aRef, int aSide)
{
    std::vector<float> v;
    const size_t n = std::min(a.f.size(), aRef.f.size());
    for (size_t i = 0; i < n; ++i)
    {
        const int o = 6 + aSide * 4;
        double dot = 0.0;
        for (int k = 0; k < 4; ++k)
            dot += static_cast<double>(a.f[i][o + k]) * aRef.f[i][o + k];
        v.push_back(static_cast<float>(2.0 * std::acos(std::fmin(1.0, std::fabs(dot))) * 57.29577951308232));
    }
    return v;
}
float MaxOf(const std::vector<float>& v)
{
    float m = 0.0f;
    for (float x : v)
        m = std::max(m, x);
    return m;
}
float MinOf(const std::vector<float>& v)
{
    float m = 1e9f;
    for (float x : v)
        m = std::min(m, x);
    return m;
}
float TailMax(const std::vector<float>& v, size_t aFrom)
{
    float m = 0.0f;
    for (size_t i = aFrom; i < v.size(); ++i)
        m = std::max(m, v[i]);
    return m;
}
std::string Fmt(double v)
{
    char b[32];
    std::snprintf(b, sizeof b, "%.3f", v);
    return b;
}
} // namespace

int main(int argc, char** argv)
{
    motiondriver::SetAimWorldSmoothing(false, 0.0f); // the kicks are measured on the unsmoothed aim (the DLL default is on at 12 Hz)
    if (argc == 7 && std::string(argv[1]) == "--dual-child")
        return DualChildMain(argv[2], argv[3], argv[4], std::stoi(argv[5]), argv[6]);
    if (argc < 2)
    {
        std::puts("usage: utpp_dualwield_driver_test <data dir> [ignored]");
        return 2;
    }
    const std::string data = argv[1];
    const std::string v1 = data + "\\utpp_base.utppmm";
    const bool baseExists = Exists(v1);
    struct Group
    {
        std::string state; // "", -aim, -move, -aim-move
        std::string body;  // "" or -man
        int hz = 60;
        int32_t right = 2, left = 2;
    };
    std::vector<Group> groups;
    for (const char* body : {"", "-man"})
        for (const char* state : {"", "-aim", "-move", "-aim-move"})
            groups.push_back({state, body, 60, 2, 2});
    for (int hz : {30, 144})
        groups.push_back({"", "", hz, 2, 2});
    // the pair of the in-game runs (Liberty right, Lexington left) and Lexington in both hands
    for (const char* body : {"", "-man"})
        for (const char* state : {"", "-aim", "-move"})
            groups.push_back({state, body, 60, 2, 6});
    groups.push_back({"", "", 60, 6, 6});
    groups.push_back({"-aim", "-man", 60, 6, 6});
    // shots of a group; the last run is the same-profile reference: a right shot with the LEFT weapon's profile on the right weapon
    const char* shots[] = {"-none", "-right", "-left", "-both", "-left-holster", "-none-holster", "-right"};
    constexpr size_t kShots = 7;
    std::vector<ChildRef> kids;
    for (const Group& g : groups)
        for (size_t s = 0; s < kShots; ++s)
        {
            const int32_t right = s + 1 == kShots ? g.left : g.right;
            const std::string name = "dual" + g.state + shots[s] + "-pr" + std::to_string(right) + "-pl" + std::to_string(g.left) + g.body;
            kids.push_back(SubmitModeChild("dualwielddriver", L"--dual-child", data, v1, L" " + std::to_wstring(g.hz) + L" " + Wide(name),
                                           baseExists));
        }
    // one weapon: the same shot with and without the left feed (hip and aim, both bodies)
    std::vector<std::string> singles;
    for (const char* body : {"", "-man"})
        for (const char* state : {"", "-aim"})
        {
            singles.push_back(std::string("single") + state + "-right" + body);
            singles.push_back(std::string("single") + state + "-right-leftfeed" + body);
        }
    // the left pulse 1 or 6 frames before the action edge (in-game pair, hip fire, both bodies)
    std::vector<std::string> earlies;
    for (const char* body : {"", "-man"})
        for (const char* when : {"-early1", "-early6"})
            earlies.push_back(std::string("dual-left") + when + "-pr2-pl6" + body);
    std::vector<ChildRef> earlyKids;
    for (const std::string& e : earlies)
        earlyKids.push_back(SubmitModeChild("dualwielddriver", L"--dual-child", data, v1, L" 60 " + Wide(e), baseExists));
    std::vector<ChildRef> singleKids;
    for (const std::string& s : singles)
        singleKids.push_back(SubmitModeChild("dualwielddriver", L"--dual-child", data, v1, L" 60 " + Wide(s), baseExists));

    size_t k = 0;
    for (const Group& g : groups)
    {
        DualRun run[kShots];
        for (size_t s = 0; s < kShots; ++s)
            run[s] = ReadRun(kids[k++]);
        const std::string tag = "dual" + g.state + g.body + " right " + std::to_string(g.right) + " left " + std::to_string(g.left) + " at " +
                                std::to_string(g.hz) + " Hz";
        bool allOk = true;
        for (size_t s = 0; s < kShots; ++s)
        {
            Check(run[s].ok, tag + shots[s] + ": the child ran and published every frame", run[s].head);
            allOk = allOk && run[s].ok;
        }
        if (!allOk)
            continue;
        const DualRun &none = run[0], &right = run[1], &left = run[2], &both = run[3], &leftHolster = run[4], &noneHolster = run[5],
                      &same = run[6];
        Check(none.head.find("mirror=1.00") != std::string::npos && none.head.find("profileReady=1") != std::string::npos,
              tag + ": the mirror is shown (1.00) and the profile is loaded before the shot", none.head);
        // the feed of every run is the same up to the shot: the first frame (shot fed after it) is bit-identical
        Check(right.hash[0] == none.hash[0] && left.hash[0] == none.hash[0] && both.hash[0] == none.hash[0] && same.hash[0] == none.hash[0],
              tag + ": the runs are identical before the shot (deterministic reference)");
        // same-profile reference: the right kick of the left weapon's profile
        const float refKick = MaxOf(HandDiff(same, none, 1)), refTurn = MaxOf(WeaponTurn(same, none, 1)), refStep = MaxStep(same, none, 1);
        const float rightKick = MaxOf(HandDiff(right, none, 1));
        const auto leftOfLeft = HandDiff(left, none, 0);
        const auto turnOfLeft = WeaponTurn(left, none, 0);
        const float leftKick = MaxOf(leftOfLeft), leftTurn = MaxOf(turnOfLeft), leftStep = MaxStep(left, none, 0);
        const float leftOnRight = MaxOf(HandDiff(right, none, 0)), rightOnLeft = MaxOf(HandDiff(left, none, 1));
        const float bothLeft = MaxOf(HandDiff(both, none, 0)), bothRight = MaxOf(HandDiff(both, none, 1));
        const size_t tail = left.f.size() - static_cast<size_t>(std::lround(0.15 * g.hz));
        // upperScale and the shot event wait of the runs with a shot (the reference run raises the Shoot state without any shot event,
        // so its wait expires by design; it is reported, not checked)
        float minUpper = 1.0f;
        bool waitSteady = true;
        std::string perRun;
        for (size_t s = 0; s < kShots; ++s)
        {
            const DualRun* r = &run[s];
            int firstLow = -1;
            for (size_t i = 0; i < r->upper.size() && firstLow < 0; ++i)
                if (!(r->upper[i] > 0.99f))
                    firstLow = static_cast<int>(i);
            perRun += std::string(" ") + shots[s] + (s + 1 == kShots ? "(same)" : "") + ": upper min " + Fmt(MinOf(r->upper)) + " first low " +
                      std::to_string(firstLow) + " wait " + FieldOf(r->head, "wait=") + "->" + FieldOf(r->end, "wait=") + ";";
            if (s == 0 || s == 5)
                continue;
            minUpper = std::min(minUpper, MinOf(r->upper));
            waitSteady = waitSteady && FieldOf(r->head, "wait=") == FieldOf(r->end, "wait=");
        }
        const std::string detail = "same-profile right kick " + Fmt(refKick) + " cm / " + Fmt(refTurn) + " deg (step " + Fmt(refStep) +
            "), right kick " + Fmt(rightKick) + " cm, left kick " + Fmt(leftKick) + " cm / " + Fmt(leftTurn) + " deg (step " + Fmt(leftStep) +
            ", end " + Fmt(TailMax(leftOfLeft, tail)) + " cm / " + Fmt(TailMax(turnOfLeft, tail)) + " deg), left hand on the right shot " +
            Fmt(leftOnRight) + " cm, right hand on the left shot " + Fmt(rightOnLeft) + " cm, both: left " + Fmt(bothLeft) + " right " +
            Fmt(bothRight) + ", min upperScale " + Fmt(minUpper) + " | left run " + left.end;
        // the same-profile reference must itself be a kick, otherwise the relative checks below say nothing
        Check(refKick >= 0.05f || refTurn >= 0.5f, tag + ": the same-profile right shot kicks the right arm", detail);
        Check(leftOnRight <= 0.5f, tag + ": a right shot moves the left hand by at most 0.5 cm", detail);
        Check(leftKick >= refKick / 3.0f && leftTurn >= refTurn / 3.0f && (leftKick > 0.0f || leftTurn > 0.0f),
              tag + ": a left shot kicks the left hand and weapon by at least a third of the right kick of the same profile", detail);
        Check(rightOnLeft <= 0.5f, tag + ": a left shot moves the right hand by at most 0.5 cm", detail);
        Check(leftStep <= refStep + 0.3f, tag + ": the left kick starts and ends continuously (step <= same-profile right step + 0.3 cm)", detail);
        Check(TailMax(leftOfLeft, tail) < 0.1f && TailMax(turnOfLeft, tail) < 0.2f,
              tag + ": the left hand returns after the kick (< 0.1 cm / 0.2 deg over the last 0.15 s)", detail);
        Check(bothLeft >= leftKick / 3.0f && bothRight >= rightKick / 3.0f && bothLeft > 0.0f && bothRight > 0.0f,
              tag + ": both shots kick both hands", detail);
        Check(minUpper > 0.99f && waitSteady,
              tag + ": the arms stay on the driver's pose on every frame (upperScale > 0.99) and no shot event wait expires", perRun);
        Check(FieldOf(left.end, "dualLeftShot=") == "1/0" && FieldOf(left.end, "dualLeftProfile=") == std::to_string(g.left) &&
              FieldOf(left.end, "dualLeftKickFrames=") != "0" && FieldOf(left.end, "dualLeftSkip=") == "0/0/0/0",
              tag + ": the left shot is accepted and applied (status)", left.end);
        Check(FieldOf(right.end, "dualPreMirrorFrames=") != "0" && FieldOf(right.end, "dualLeftKickFrames=") == "0",
              tag + ": a right shot builds the mirror from the pre-kick right arm and starts no left kick (status)", right.end);
        // holster 0.05 s after a left shot: the kick fades with the mirror weight (no larger step than with the holster alone)
        const float holsterStep = MaxStep(leftHolster, noneHolster, 0);
        Check(holsterStep <= refStep + 0.3f, tag + ": a holster during the left kick fades it (step " + Fmt(holsterStep) + " cm)", detail);
    }
    for (size_t i = 0; i < earlies.size(); ++i)
    {
        const DualRun r = ReadRun(earlyKids[i]);
        Check(r.ok, earlies[i] + ": the child ran and published every frame", r.head);
        if (!r.ok)
            continue;
        Check(MinOf(r.upper) > 0.99f && FieldOf(r.head, "wait=") == FieldOf(r.end, "wait=") &&
                  FieldOf(r.end, "dualNativeFrames=") == "0" && FieldOf(r.end, "dualWaitExpired=") == "0",
              earlies[i] + ": a left shot ahead of the action edge still owns the Shoot state (upperScale > 0.99, no wait expiry, "
                           "no native frames)",
              "upper min " + Fmt(MinOf(r.upper)) + " | " + r.head + " | " + r.end);
        Check(FieldOf(r.end, "dualLeftKickFrames=") != "0" && FieldOf(r.end, "dualLeftShot=") == "1/0",
              earlies[i] + ": the early left shot is accepted and kicks", r.end);
    }
    for (size_t i = 0; i + 1 < singles.size(); i += 2)
    {
        const DualRun plain = ReadRun(singleKids[i]), fed = ReadRun(singleKids[i + 1]);
        Check(plain.ok && fed.ok, singles[i] + ": both single weapon children ran", plain.head + " | " + fed.head);
        if (!plain.ok || !fed.ok)
            continue;
        bool sameHash = plain.hash.size() == fed.hash.size();
        for (size_t f = 0; sameHash && f < plain.hash.size(); ++f)
            sameHash = plain.hash[f] == fed.hash[f];
        Check(sameHash, singles[i] + ": one weapon: every published pose is bit-identical with and without the left feed");
        Check(FieldOf(fed.end, "dualLeftShot=") == "0/1" && FieldOf(fed.end, "dualLeftKickFrames=") == "0",
              singles[i] + ": one weapon: the left feed is counted dropped and starts no kick", fed.end);
    }
    std::printf("dual wield driver: %d PASS %d FAIL\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
