// Offline test of src/input/NativeInputCore.hpp: the byte proof of the native locomotion input reader (synthetic image built
// from the pattern table, every mutation must fail closed; optionally the real 2.5 exe given as argv[1]), the arming TTL and
// the unit clamp.

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "../src/input/NativeInputCore.hpp"
#include "../src/input/ApproachCore.hpp"

using namespace utpp::freemove::nativein;

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

static const Pat* const kAll[] = {&kPatReader,  &kPatCaller, &kPatHandlerCall, &kPatLookup, &kPatMoveX, &kPatMoveY,
                                  &kPatStoreY,  &kPatGuardA, &kPatGuardB, &kPatBranch, &kPatAnimX,       &kPatAnimY,  &kPatAnimSet};

// Image with every pattern placed at its RVA and its rel32 fields resolved to the expected targets.
static std::vector<uint8_t> MakeImage()
{
    std::vector<uint8_t> img(0xC00000, 0xCC);
    for (const Pat* p : kAll)
    {
        std::memcpy(img.data() + p->rva, p->bytes, p->len);
        auto fix = [&](int aOff, uint64_t aTarget)
        {
            if (aOff < 0)
                return;
            const int32_t disp = static_cast<int32_t>(static_cast<int64_t>(aTarget) - static_cast<int64_t>(p->rva + aOff + 4));
            std::memcpy(img.data() + p->rva + aOff, &disp, sizeof(disp));
        };
        fix(p->rel0Off, p->rel0Target);
        fix(p->rel1Off, p->rel1Target);
    }
    return img;
}

static Verdict Run(const std::vector<uint8_t>& aImg, uint64_t aHandler = kRvaHandlerActionValue, uint64_t aHx = kHashMoveX,
                   uint64_t aHy = kHashMoveY)
{
    auto read = [&](uint64_t aRva, uint8_t* aDst, size_t aLen)
    {
        if (aRva + aLen > aImg.size())
            return false;
        std::memcpy(aDst, aImg.data() + aRva, aLen);
        return true;
    };
    return VerifyReader(aHandler, aHx, aHy, read);
}

static void TestSynthetic()
{
    const std::vector<uint8_t> img = MakeImage();
    const Verdict v = Run(img);
    Check(v.ok && v.animOk, std::string("clean image proven: ") + v.why);

    // wrong anchor / hashes
    Check(!Run(img, kRvaHandlerActionValue + 0x10).ok, "wrong handler RVA fails closed");
    Check(!Run(img, kRvaHandlerActionValue, kHashMoveX ^ 1).ok, "wrong MoveX hash fails closed");
    Check(!Run(img, kRvaHandlerActionValue, kHashMoveX, kHashMoveY + 1).ok, "wrong MoveY hash fails closed");

    // failed read (image too small) fails closed
    const std::vector<uint8_t> small(0x1000, 0);
    Check(!Run(small).ok, "unreadable image fails closed");

    // every required pattern: a flipped byte in a compared position -> no attach; a retargeted rel32 -> no attach
    const Pat* const required[] = {&kPatHandlerCall, &kPatLookup, &kPatReader, &kPatGuardA, &kPatGuardB, &kPatCaller, &kPatMoveX, &kPatMoveY,
                                &kPatStoreY};
    const Pat* const animOnly[] = {&kPatBranch, &kPatAnimX, &kPatAnimY, &kPatAnimSet};
    int mutations = 0;
    for (const Pat* p : kAll)
    {
        bool isRequired = false;
        for (const Pat* r : required)
            isRequired = isRequired || r == p;
        for (size_t i = 0; i < p->len; ++i)
        {
            const bool inRel0 = p->rel0Off >= 0 && i >= static_cast<size_t>(p->rel0Off) && i < static_cast<size_t>(p->rel0Off) + 4;
            const bool inRel1 = p->rel1Off >= 0 && i >= static_cast<size_t>(p->rel1Off) && i < static_cast<size_t>(p->rel1Off) + 4;
            std::vector<uint8_t> m = img;
            if (inRel0 || inRel1)
            {
                // retarget: the rel32 low byte +1 changes the call target
                if (i != static_cast<size_t>(inRel0 ? p->rel0Off : p->rel1Off))
                    continue;
            }
            m[p->rva + i] ^= 0x01;
            const Verdict mv = Run(m);
            ++mutations;
            if (isRequired)
                Check(!mv.ok && !mv.animOk, "required pattern mutation fails closed @rva " + std::to_string(p->rva) + "+" + std::to_string(i));
            else
                Check(mv.ok && !mv.animOk, "anim pattern mutation keeps the reader hook but drops the anim rewrite @rva " +
                                               std::to_string(p->rva) + "+" + std::to_string(i));
        }
    }
    Check(mutations > 150, "mutation sweep ran (" + std::to_string(mutations) + ")");
    (void)animOnly;
}

static void TestReal(const char* aPath)
{
    std::ifstream f(aPath, std::ios::binary);
    if (!f)
    {
        Check(false, std::string("cannot open ") + aPath);
        return;
    }
    const std::vector<uint8_t> file((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    auto u16 = [&](size_t o) { uint16_t v; std::memcpy(&v, file.data() + o, 2); return v; };
    auto u32 = [&](size_t o) { uint32_t v; std::memcpy(&v, file.data() + o, 4); return v; };
    const size_t pe = u32(0x3C);
    const size_t nsec = u16(pe + 6);
    const size_t opt = u16(pe + 20);
    const size_t so = pe + 24 + opt;
    auto read = [&](uint64_t aRva, uint8_t* aDst, size_t aLen)
    {
        for (size_t i = 0; i < nsec; ++i)
        {
            const size_t s = so + 40 * i;
            const uint32_t vs = u32(s + 8), va = u32(s + 12), rs = u32(s + 16), ro = u32(s + 20);
            if (aRva >= va && aRva + aLen <= va + (vs > rs ? vs : rs) && ro + (aRva - va) + aLen <= file.size())
            {
                std::memcpy(aDst, file.data() + ro + (aRva - va), aLen);
                return true;
            }
        }
        return false;
    };
    const Verdict v = VerifyReader(kRvaHandlerActionValue, kHashMoveX, kHashMoveY, read);
    Check(v.ok && v.animOk, std::string("real 2.5 exe proven: ") + v.why);
}

static void TestArming()
{
    Arming a;
    Check(!a.Armed(0.0) && !a.Armed(5.0), "fresh Arming is disarmed");
    Check(a.Arm(10.0, 0.2), "Arm accepted");
    Check(a.Armed(10.0) && a.Armed(10.19) && !a.Armed(10.2) && !a.Armed(11.0), "armed exactly for the TTL");
    Check(a.Arm(10.0, 99.0), "long TTL accepted");
    Check(a.Armed(10.49) && !a.Armed(10.51), "TTL clamped to kMaxArmTtlSec");
    a.Disarm();
    Check(!a.Armed(10.0), "Disarm drops the arm");
    a.Arm(10.0, 0.2);
    Check(!a.Arm(10.0, 0.0) && !a.Armed(10.05), "TTL 0 rejects and disarms");
    a.Arm(10.0, 0.2);
    Check(!a.Arm(10.0, -1.0) && !a.Armed(10.05), "negative TTL rejects and disarms");
    a.Arm(10.0, 0.2);
    Check(!a.Arm(NAN, 0.2) && !a.Armed(10.05), "non-finite time rejects and disarms");
    Check(!a.Arm(1.0, INFINITY) && !a.Armed(1.1), "non-finite TTL rejects");
    Check(a.Arm(0.0, 0.3) && a.Armed(0.1), "arming at time 0 works");

    Check(ShouldApply(true, true, false), "apply when armed + live + no fault");
    Check(!ShouldApply(false, true, false), "no apply when not armed");
    Check(!ShouldApply(true, false, false), "no apply without a live override");
    Check(!ShouldApply(true, true, true), "no apply after a fault");
}

static void TestGuardAndLatch()
{
    // reader early-out: ref block present + strong count non-zero + object present
    Check(ReaderWouldRun(true, 1, true), "reader runs with a live ref and object");
    Check(!ReaderWouldRun(false, 1, true), "reader bails without a ref block");
    Check(!ReaderWouldRun(true, 0, true), "reader bails on a zero strong count");
    Check(!ReaderWouldRun(true, 1, false), "reader bails without an object");

    int a = 0, b = 0, c = 0;
    InstanceLatch l;
    Check(l.Latched() == nullptr && l.Distinct() == 0, "fresh latch is empty");
    l.Track(&a);
    l.Track(&a);
    l.Track(&b);
    Check(l.Distinct() == 2, "distinct instances are counted once");
    Check(l.Admit(&a) && l.Latched() == &a && l.Latches() == 1, "first applying call latches its instance");
    Check(l.Admit(&a), "the latched instance keeps being admitted");
    Check(!l.Admit(&b) && !l.Admit(&c) && l.Refused() == 2, "other instances are refused and counted");
    Check(l.Latched() == &a, "a refused call does not move the latch");
    l.Release();
    Check(l.Latched() == nullptr, "Release ends the arm session");
    Check(l.Admit(&b) && l.Latched() == &b && l.Latches() == 2, "the next arm session latches afresh");
    Check(!l.Admit(&a), "previous instance is refused in the new session");
    static int pool[40];
    for (int i = 0; i < 40; ++i)
        l.Track(&pool[i]);
    Check(l.Distinct() == InstanceLatch::kMaxTracked, "distinct tracking is capped");
}

// Commands of the approach controller as the native reader hook sees them: the unit clamp must not change any of them (walk and turn
// sticks are far below 1), the arm TTL of one step covers the next script tick, and the detour applies them while armed.
static void TestApproachCommands()
{
    namespace ap = utpp::approach;
    ap::Scene sc;
    sc.box = ap::BoxPoly(0.0f, 0.0f, 0.95f, 2.4f, 0.35f);
    const ap::V2 spot{-2.11f, -0.91f};
    const ap::Params prm;
    Check(prm.ttlS > 0.0f && prm.ttlS <= static_cast<float>(kMaxArmTtlSec), "approach step TTL fits the arm TTL cap");
    ap::Approach a;
    Check(a.Begin(sc, prm, {-1.75f, -0.54f}, -5.0f, spot, -80.8f) == ap::BeginResult::Started, "ride 5 like begin");
    Arming arm;
    bool same = true, applies = true, small = true;
    ap::V2 pos{-1.75f, -0.54f};
    float yaw = -5.0f;
    double now = 1.0;
    int steps = 0;
    for (int i = 0; i < 900; ++i)
    {
        now += 1.0 / 60.0;
        ap::Command cmd;
        if (a.Step(pos, yaw, 0.0f, 1.0f / 60.0f, true, 0.0f, 0.0f, cmd) != ap::Result::Running)
            break;
        ++steps;
        float x = cmd.stickX, y = cmd.stickY;
        ClampUnit(x, y);
        same = same && x == cmd.stickX && y == cmd.stickY;
        small = small && std::sqrt(x * x + y * y) < 0.7f;
        arm.Arm(now, prm.ttlS);
        applies = applies && ShouldApply(arm.Armed(now + 1.0 / 60.0), true, false);
        // simple plant: walk along the stick, turn after it
        const float mag = std::sqrt(x * x + y * y);
        const float want = ap::Deg(std::atan2(-x, y));
        pos = pos + ap::Fwd(want) * (2.3f * mag / 60.0f);
        yaw = ap::WrapDeg(yaw + std::clamp(ap::WrapDeg(want - yaw), -4.5f, 4.5f));
    }
    Check(steps > 10, "ride 5 like: the controller ran");
    Check(same, "unit clamp leaves every approach command unchanged (bit identical)");
    Check(small, "approach commands stay below the Walk/Run split (0.7)");
    Check(applies, "every command is applied by the detour while armed");
}

static void TestClamp()
{
    float x = 0.6f, y = 0.8f;
    Check(ClampUnit(x, y) && std::fabs(x - 0.6f) < 1e-6f && std::fabs(y - 0.8f) < 1e-6f, "unit vector unchanged");
    x = 0.3f;
    y = 0.4f;
    Check(ClampUnit(x, y) && x == 0.3f && y == 0.4f, "short vector never scaled up");
    x = 3.0f;
    y = 4.0f;
    Check(ClampUnit(x, y) && std::fabs(x - 0.6f) < 1e-5f && std::fabs(y - 0.8f) < 1e-5f, "long vector scaled to unit length");
    x = -2.0f;
    y = 0.0f;
    Check(ClampUnit(x, y) && x == -1.0f && y == 0.0f, "axis vector clamped");
    x = NAN;
    y = 0.5f;
    Check(!ClampUnit(x, y) && x == 0.0f && y == 0.0f, "NaN -> (0,0) and false");
    x = 0.5f;
    y = INFINITY;
    Check(!ClampUnit(x, y) && x == 0.0f && y == 0.0f, "inf -> (0,0) and false");
}

int main(int argc, char** argv)
{
    TestSynthetic();
    TestArming();
    TestGuardAndLatch();
    TestClamp();
    TestApproachCommands();
    if (argc > 1)
        TestReal(argv[1]);
    std::printf("native input test: %d passed, %d failed%s\n", g_pass, g_fail, argc > 1 ? " (real exe checked)" : "");
    return g_fail == 0 ? 0 : 1;
}
