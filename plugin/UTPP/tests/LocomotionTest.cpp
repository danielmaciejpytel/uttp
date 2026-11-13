// Offline test of the UTPP Locomotion core (Phase 5N step 4a): angle remap, rewrite rules, gating, staleness,
// coexistence / install decisions, kill switch. No game, no RED4ext.

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <regex>
#include <string>

#include "../src/input/LocomotionCore.hpp"

using namespace utpp::freemove;

static int g_pass = 0;
static int g_fail = 0;

static void Check(bool aOk, const char* aWhat)
{
    if (aOk)
        ++g_pass;
    else
    {
        ++g_fail;
        std::printf("FAIL: %s\n", aWhat);
    }
}

static bool Near(float a, float b, float eps = 1e-4f)
{
    return std::fabs(a - b) <= eps;
}

// Core installed + enabled + fresh TPP heartbeat without blocks at time t.
static void MakeActive(Core& c, double t)
{
    c.SetInstalled(true);
    c.SetEnabled(true);
    c.Heartbeat(t, false, 1.5f, 0);
}

static void TestAngles()
{
    Check(Near(RawAngleDeg(0, 1), 0), "forward = 0");
    Check(Near(RawAngleDeg(-1, 0), 90), "left = +90");
    Check(Near(RawAngleDeg(1, 0), -90), "right = -90");
    Check(Near(RawAngleDeg(0, -1), 180), "back = 180 (wrap keeps +180)");
    Check(Near(RawAngleDeg(1, 1), -45), "forward-right = -45");
    Check(Near(RawAngleDeg(-1, -1), 135), "back-left = 135");
    Check(Near(RawAngleDeg(0, 0), 0), "zero stick = 0");
    Check(Near(RawAngleDeg(NAN, 1), 0), "NaN stick = 0");
    Check(Near(WrapDeg(540), 180) && Near(WrapDeg(-190), 170) && Near(WrapDeg(INFINITY), 0), "WrapDeg");
    // Consistency with the UTPP yaw convention: forward(yaw) = (-sin, cos); stick (x right, y fwd) -> world dir in camera
    // frame (-x... ) i.e. angle a gives (-sin a, cos a) == (x, y) mapped with x' = x.
    for (float a = -170.0f; a <= 180.0f; a += 10.0f)
    {
        const float r = a * kPi / 180.0f;
        const float x = std::sin(-r) * 0.8f; // stick right component for angle a (CCW positive = left => x negative)
        const float y = std::cos(r) * 0.8f;
        // Compare the wrapped difference: at the +-180 seam atan2 may return -179.99999 for +180.
        if (std::fabs(WrapDeg(RawAngleDeg(x, y) - a)) > 1e-3f)
        {
            Check(false, "angle round trip");
            return;
        }
    }
    Check(true, "angle round trip -170..180");
    Check(Near(StickMagnitude(3, 4), 5) && Near(StickMagnitude(NAN, 1), 0), "magnitude");
}

static void TestRewrite()
{
    Core c;
    MakeActive(c, 10.0);
    Check(c.Active(10.0), "active after heartbeat");
    // Pure right: X read first (Y unknown -> 0), X -> 0; then Y read completes magnitude 1.
    Check(Near(c.FilterAxis(Axis::X, 1.0f, 10.0), 0.0f), "MoveX right -> 0");
    Check(Near(c.FilterAxis(Axis::Y, 0.0f, 10.0), 1.0f), "MoveY right -> |v| = 1");
    Check(Near(c.FilterHeading(-90.0f, 10.0), 0.0f), "heading right -> 0");
    Check(Near(c.RawAngle(10.0), -90.0f), "raw angle right = -90");
    // Back-left diagonal from WASD: magnitude sqrt(2) preserved (vanilla GetMovementInputActionValue unchanged).
    c.FilterAxis(Axis::X, -1.0f, 10.01);
    const float y = c.FilterAxis(Axis::Y, -1.0f, 10.01);
    Check(Near(y, std::sqrt(2.0f)), "diagonal magnitude preserved");
    Check(Near(StickMagnitude(c.FilterAxis(Axis::X, -1.0f, 10.01), y), std::sqrt(2.0f)),
          "script magnitude == vanilla magnitude");
    Check(Near(c.RawAngle(10.01), 135.0f), "raw angle back-left = 135");
    // Analog walk: 0.4 back -> (0, 0.4).
    c.FilterAxis(Axis::X, 0.0f, 10.02);
    Check(Near(c.FilterAxis(Axis::Y, -0.4f, 10.02), 0.4f), "analog back 0.4 -> forward 0.4");
    // Below the deadzone: pass-through.
    c.FilterAxis(Axis::X, 0.05f, 10.03);
    Check(Near(c.FilterAxis(Axis::Y, -0.05f, 10.03), -0.05f), "deadzone pass-through (Y)");
    Check(Near(c.FilterAxis(Axis::X, 0.05f, 10.03), 0.05f), "deadzone pass-through (X)");
    Check(Near(c.FilterHeading(170.0f, 10.03), 170.0f), "deadzone heading pass-through");
    Check(!c.HasInput(10.03) && Near(c.RawAngle(10.03), 0.0f), "no input in deadzone");
    // NaN raw value passes through untouched.
    Check(std::isnan(c.FilterAxis(Axis::Y, NAN, 10.04)), "NaN pass-through");
    // Stale other axis is not used.
    c.FilterAxis(Axis::X, 1.0f, 10.05);
    Check(Near(c.FilterAxis(Axis::Y, 0.0f, 10.05 + 0.2), 0.0f), "stale X not used for Y magnitude (pass-through)");
    Check(c.AxisRewrites() > 0 && c.HeadingRewrites() == 1, "counters");
}

static void TestGating()
{
    Core c;
    // Not installed / not enabled by default.
    c.Heartbeat(1.0, true, 0.0f, 0);
    Check((c.Reasons(1.0) & kNotInstalled) && (c.Reasons(1.0) & kDisabled), "default: not installed, disabled");
    Check(Near(c.FilterAxis(Axis::X, 1.0f, 1.0), 1.0f), "inactive: raw passes");
    Check(c.HasInput(1.0) == false, "only X fresh -> no input");
    c.FilterAxis(Axis::Y, 0.0f, 1.0);
    Check(c.HasInput(1.0) && Near(c.RawAngle(1.0), -90.0f), "raw angle available even when inactive");

    MakeActive(c, 2.0);
    Check(c.Reasons(2.0) == 0, "active");
    // FPP: owner says no and camera close.
    c.Heartbeat(2.1, false, 0.2f, 0);
    Check(c.Reasons(2.1) == kNotTpp, "FPP blocks");
    c.Heartbeat(2.2, true, 0.1f, 0);
    Check(c.Reasons(2.2) == 0, "owner TPP flag wins over camera distance");
    // Each script bit blocks by default.
    const uint32_t bits[] = {kVehicle, kScene, kSwimming, kWorkspot, kTakedown, kCarrying, kVitals, kMenu,
                             kNoBlackboard, kReaction, kTraversal, kAiming, kWeapon, kHold};
    bool all = true;
    for (uint32_t b : bits)
    {
        c.Heartbeat(3.0, false, 2.0f, b);
        all = all && c.Reasons(3.0) == b && !c.Active(3.0);
    }
    Check(all, "every script block bit blocks");
    c.Heartbeat(3.0, false, 2.0f, kReserved64);
    Check(c.Reasons(3.0) == 0, "bit 64 (generic detailed-loco from utpp.reds) ignored");
    // Block mask: allow weapon in hand.
    c.SetBlockMask(kDefaultBlockMask & ~kWeapon);
    c.Heartbeat(3.1, false, 2.0f, kWeapon);
    Check(c.Reasons(3.1) == 0, "block mask can allow weapon");
    c.SetBlockMask(kDefaultBlockMask);
    // External hold.
    c.Heartbeat(3.2, false, 2.0f, 0);
    c.SetHold(true);
    Check(c.Reasons(3.2) == kHold, "external hold");
    c.SetHold(false);
    // Stale heartbeat (vehicle / menu: PSM locomotion stops ticking).
    Check(c.Active(3.2 + 0.24) && !c.Active(3.2 + 0.26), "stale after 0.25 s");
    Check(c.Reasons(3.2 + 1.0) == kStale, "stale reason");
    Check(!c.Active(3.0), "clock going backwards = stale");
    // Inactive => no rewrite.
    c.FilterAxis(Axis::X, 1.0f, 5.0);
    Check(Near(c.FilterAxis(Axis::Y, 0.5f, 5.0), 0.5f), "stale -> raw passes");
    // Disable at runtime.
    c.Heartbeat(6.0, false, 2.0f, 0);
    c.SetEnabled(false);
    Check(c.Reasons(6.0) == kDisabled, "disable");
    c.SetEnabled(true);
    // Kill switch latches.
    c.Kill();
    c.SetEnabled(true);
    c.Heartbeat(6.1, false, 2.0f, 0);
    Check((c.Reasons(6.1) & kKilled) && !c.Active(6.1), "kill latched");
    // Activation counter counts rising edges only.
    Core d;
    d.SetInstalled(true);
    d.SetEnabled(true);
    d.Heartbeat(1.0, true, 0, 0);
    d.Heartbeat(1.01, true, 0, 0);
    d.Heartbeat(1.02, true, 0, kMenu);
    d.Heartbeat(1.03, true, 0, 0);
    Check(d.Activations() == 2 && d.Beats() == 4, "activations count rising edges");
    // Camera distance threshold / NaN.
    Check(IsThirdPerson(false, 0.61f, 0.6f) && !IsThirdPerson(false, 0.6f, 0.6f) && !IsThirdPerson(false, NAN, 0.6f),
          "IsThirdPerson");
}

static void TestParams()
{
    Core c;
    Params p;
    p.staleSec = 0.5;
    p.deadzone = 0.3f;
    Check(c.SetParams(p) && Near(c.GetParams().deadzone, 0.3f), "valid params");
    Params bad = p;
    bad.deadzone = -1.0f;
    Check(!c.SetParams(bad) && Near(c.GetParams().deadzone, 0.3f), "invalid deadzone rejected, previous kept");
    bad = p;
    bad.staleSec = NAN;
    Check(!c.SetParams(bad), "NaN stale rejected");
    bad = p;
    bad.tppCameraDist = 50.0f;
    Check(!c.SetParams(bad), "huge camera distance rejected");
    MakeActive(c, 1.0);
    c.FilterAxis(Axis::X, 0.0f, 1.0);
    Check(Near(c.FilterAxis(Axis::Y, -0.25f, 1.0), -0.25f), "deadzone 0.3 passes 0.25");
    Check(c.Active(1.45) && !c.Active(1.55), "stale 0.5 s");
}

static void TestCoexistence()
{
    InstallFacts f;
    f.gameValidated = true;
    f.classFound = true;
    f.actionValue = {true, true, false, true, 1, true, true, true, true, true, true, true};
    f.inputHeading = {true, true, false, true, 0, false, true, true, true, true, false, true};
    f.handlersDistinct = true;
    f.tableProven = true;
    Check(DecideInstall(f) == InstallResult::Ok, "all proven -> install");

    InstallFacts g = f;
    g.gameValidated = false;
    g.killFile = true;
    Check(DecideInstall(g) == InstallResult::KillFile, "kill file wins");
    g = f;
    g.gameValidated = false;
    Check(DecideInstall(g) == InstallResult::GameNotValidated, "not validated -> off");
    g = f;
    g.classFound = false;
    Check(DecideInstall(g) == InstallResult::NoClass, "no class");
    g = f;
    g.actionValue.isStatic = true;
    Check(DecideInstall(g) == InstallResult::BadSignature, "static -> signature fail");
    g = f;
    g.actionValue.firstParamIsCName = false;
    Check(DecideInstall(g) == InstallResult::BadSignature, "param type");
    g = f;
    g.inputHeading.paramCount = 1;
    Check(DecideInstall(g) == InstallResult::BadSignature, "heading param count");
    g = f;
    g.inputHeading.returnsFloat = false;
    Check(DecideInstall(g) == InstallResult::BadSignature, "heading return type");
    g = f;
    g.actionValue.isNative = false;
    Check(DecideInstall(g) == InstallResult::BadSignature, "scripted function");
    g = f;
    g.actionValue.firstParamByValue = false;
    Check(DecideInstall(g) == InstallResult::BadSignature, "out/opt first param -> signature fail");
    g = f;
    g.inputHeading.hasReturnType = false;
    Check(DecideInstall(g) == InstallResult::BadSignature, "missing return property -> signature fail");

    // In-game 2026-10-07 (code -5 at PostRegisterTypes): natives exist with handlers but no
    // return type / params until the script runtime binds them. Must stay fail-closed, and be recognisable.
    g = f;
    g.actionValue.hasReturnType = g.actionValue.returnsFloat = false;
    g.actionValue.paramCount = 0;
    g.actionValue.firstParamIsCName = g.actionValue.firstParamByValue = false;
    g.inputHeading.hasReturnType = g.inputHeading.returnsFloat = false;
    Check(DecideInstall(g) == InstallResult::BadSignature, "unbound natives (pre-script) -> signature fail");
    Check(LooksUnbound(g.actionValue) && LooksUnbound(g.inputHeading), "unbound natives recognised");
    Check(!LooksUnbound(f.actionValue) && !LooksUnbound(f.inputHeading), "bound natives not flagged unbound");
    FunctionFacts missing;
    Check(!LooksUnbound(missing), "missing function not flagged unbound");

    // PostRegister: cheap decisions only, RTTI proof deferred to Running.
    Check(DecideEarly(f) == InstallResult::Pending, "early: all cheap checks pass -> pending");
    g = f;
    g.classFound = false;
    g.actionValue = FunctionFacts{};
    Check(DecideEarly(g) == InstallResult::Pending, "early: ignores RTTI facts");
    g.killFile = true;
    Check(DecideEarly(g) == InstallResult::KillFile, "early: kill file");
    g.killFile = false;
    g.gameValidated = false;
    Check(DecideEarly(g) == InstallResult::GameNotValidated, "early: not validated");
    // In-game 2026-10-07 run 2 (code -6, handlerTable=0): table not proven -> own code.
    g = f;
    g.tableProven = false;
    Check(DecideInstall(g) == InstallResult::BadTable, "table/dispatcher not proven -> off");
    g = f;
    g.inputHeading.regOk = false;
    Check(DecideInstall(g) == InstallResult::BadHandler, "implausible register index -> off");
    g = f;
    g.actionValue.handlerUnique = false;
    Check(DecideInstall(g) == InstallResult::BadHandler, "shared handler -> off");
    g = f;
    g.inputHeading.handlerInImage = false;
    Check(DecideInstall(g) == InstallResult::BadHandler, "handler outside image (patched slot) -> off");
    g = f;
    g.actionValue.handlerExecutable = false;
    Check(DecideInstall(g) == InstallResult::BadHandler, "non-executable handler");
    g = f;
    g.handlersDistinct = false;
    Check(DecideInstall(g) == InstallResult::BadHandler, "same handler for both");

    // A hook that is not installed keeps the core inactive even if everything else is fine.
    Core c;
    MakeActive(c, 1.0);
    c.SetInstalled(false);
    c.Heartbeat(1.0, true, 2.0f, 0);
    Check(c.Reasons(1.0) == kNotInstalled, "not installed -> inactive");
    c.FilterAxis(Axis::X, 1.0f, 1.0);
    Check(Near(c.FilterAxis(Axis::Y, 0.0f, 1.0), 0.0f) && Near(c.FilterAxis(Axis::X, 1.0f, 1.0), 1.0f),
          "not installed -> pass-through");
    for (int r = -9; r <= 0; ++r)
        if (r != -2) // -2 is unused
            Check(InstallResultName(static_cast<InstallResult>(r))[0] != '?', "result names");
}

// Handler table / dispatcher proof on the real 2.5 bytes of rtti::Function::InternalCallNative
// (Extracted\reverse\exe250\Cyberpunk2077.exe, RVA 0x278C10, 0x130 bytes; found 2026-10-23 as the unique match of the 2.31
// prologue with rel32/disp32 masked; same layout as 2.31, where it was RVA 0x14619C, tables 0x33A6E50 / 0x33E6E50).
// The last 4 bytes are the start of the next function (2.31 had int3 padding there).
static void TestDispatcherScan()
{
    static const uint8_t kCode[] = {
        0x48, 0x8b, 0xc4, 0x48, 0x89, 0x58, 0x08, 0x48, 0x89, 0x68, 0x10, 0x48, 0x89, 0x70, 0x18, 0x48, 0x89, 0x78,
        0x20, 0x41, 0x56, 0x48, 0x83, 0xec, 0x30, 0x8b, 0x81, 0xa8, 0x00, 0x00, 0x00, 0x49, 0x8b, 0xe8, 0x4c, 0x8b,
        0xf2, 0xa8, 0x02, 0x74, 0x6a, 0x48, 0x8b, 0x01, 0xff, 0x50, 0x18, 0x8b, 0xc8, 0x48, 0x8d, 0x05, 0xb8, 0x73,
        0xd8, 0xff, 0x48, 0x8b, 0x9c, 0xc8, 0x30, 0xbd, 0xb3, 0x03, 0x48, 0x85, 0xdb, 0x0f, 0x84, 0xdc, 0x00, 0x00,
        0x00, 0x49, 0x8b, 0x06, 0x49, 0x8b, 0xce, 0xff, 0x50, 0x10, 0x49, 0x8b, 0x16, 0x49, 0x8b, 0xce, 0x48, 0x8b,
        0xf8, 0xff, 0x52, 0x08, 0x49, 0x8b, 0x4e, 0x18, 0x4c, 0x8b, 0xc0, 0x48, 0x85, 0xc9, 0x0f, 0x84, 0x85, 0x00,
        0x00, 0x00, 0x4c, 0x8b, 0xcf, 0x48, 0x8b, 0xd5, 0xff, 0xd3, 0xb0, 0x01, 0x48, 0x8b, 0x5c, 0x24, 0x40, 0x48,
        0x8b, 0x6c, 0x24, 0x48, 0x48, 0x8b, 0x74, 0x24, 0x50, 0x48, 0x8b, 0x7c, 0x24, 0x58, 0x48, 0x83, 0xc4, 0x30,
        0x41, 0x5e, 0xc3, 0x48, 0x8b, 0x7a, 0x18, 0x48, 0x85, 0xff, 0x0f, 0x84, 0x29, 0xb2, 0x1c, 0x02, 0x48, 0x8b,
        0x01, 0xeb, 0x10, 0x49, 0x8b, 0x7e, 0x20, 0x48, 0x8b, 0x01, 0x48, 0x85, 0xff, 0x0f, 0x84, 0x61, 0xb3, 0x1c,
        0x02, 0xff, 0x50, 0x18, 0x8b, 0xc8, 0x48, 0x8d, 0x05, 0x2f, 0x73, 0xd8, 0xff, 0x48, 0x8b, 0xb4, 0xc8, 0x30,
        0xbd, 0xb7, 0x03, 0x48, 0x85, 0xf6, 0x74, 0x57, 0x49, 0x8b, 0x06, 0x49, 0x8b, 0xce, 0xff, 0x50, 0x10, 0x49,
        0x8b, 0x16, 0x49, 0x8b, 0xce, 0x48, 0x8b, 0xd8, 0xff, 0x52, 0x08, 0x4c, 0x8b, 0xcb, 0x48, 0x8b, 0xd5, 0x4c,
        0x8b, 0xc0, 0x48, 0x8b, 0xcf, 0xff, 0xd6, 0xeb, 0x83, 0x49, 0x8b, 0x4e, 0x28, 0x48, 0x85, 0xc9, 0x74, 0x22,
        0x48, 0x8b, 0x05, 0xd5, 0x2f, 0x8c, 0x03, 0x48, 0x3b, 0xc1, 0x75, 0x09, 0x49, 0x8b, 0x4e, 0x20, 0xe9, 0x5d,
        0xff, 0xff, 0xff, 0x48, 0x8b, 0x51, 0x10, 0x48, 0x85, 0xd2, 0x0f, 0x85, 0x6a, 0xb0, 0x1c, 0x02, 0x33, 0xc9,
        0xe9, 0x49, 0xff, 0xff, 0xff, 0x32, 0xc0, 0xe9, 0x4c, 0xff, 0xff, 0xff, 0x48, 0x8b, 0x44, 0x24};
    constexpr uint64_t kRva = 0x278C10;
    constexpr uint64_t kTable = 0x3B7BD30;
    Check(sizeof(kCode) == 0x130, "dispatcher fixture size");
    DispatchScan r = ScanDispatcher(kCode, sizeof(kCode), kRva, kTable);
    Check(r.reads == 2 && r.tableReads == 1 && r.firstOtherDisp == 0x3B3BD30,
          "2.5 dispatcher: member table read once, static table read once");
    r = ScanDispatcher(kCode, sizeof(kCode), kRva, kTable + 8);
    Check(r.reads == 2 && r.tableReads == 0, "wrong table address -> not proven");
    r = ScanDispatcher(kCode, sizeof(kCode), kRva + 0x10, kTable);
    Check(r.reads == 0, "lea not resolving to the image base -> no reads");
    uint8_t mod[sizeof(kCode)];
    for (size_t i = 0; i < sizeof(kCode); ++i)
        mod[i] = kCode[i];
    // Break the GetRegIndex call (vfunc 0x18 -> 0x20) of the member branch.
    for (size_t i = 0; i + 3 <= sizeof(mod); ++i)
        if (mod[i] == 0xFF && mod[i + 1] == 0x50 && mod[i + 2] == 0x18 && i > 0xB0)
            mod[i + 2] = 0x20;
    r = ScanDispatcher(mod, sizeof(mod), kRva, kTable);
    Check(r.tableReads == 0 && r.reads == 1, "dispatcher not indexing by GetRegIndex -> not proven");
    Check(ScanDispatcher(nullptr, 0, kRva, kTable).reads == 0, "no code");
    Check(ScanDispatcher(kCode, 10, kRva, kTable).reads == 0, "short code");

    // Register index plausibility: in-game reg 13305 / 13333 with a counter of N registrations (idx = N + 0xFF).
    Check(RegPlausible(13305, 13100) && RegPlausible(13333, 13100), "in-game reg indices plausible");
    Check(!RegPlausible(13333, 13000), "reg beyond the counter -> implausible");
    Check(!RegPlausible(0x10, 13100), "reg below the first member index -> implausible");
    Check(!RegPlausible(kHandlerTableEntries, 0xFFFFFFFFu), "reg outside the table -> implausible");
    Check(!RegPlausible(13305, 0), "counter zero -> implausible");
}

// Determinism / no state leak: same inputs, same outputs.
static void TestDeterminism()
{
    float out1[64], out2[64];
    for (int run = 0; run < 2; ++run)
    {
        Core c;
        MakeActive(c, 0.0);
        float* out = run == 0 ? out1 : out2;
        for (int i = 0; i < 32; ++i)
        {
            const double t = i / 60.0;
            if (i % 8 == 0)
                c.Heartbeat(t, false, 1.5f, i == 16 ? kMenu : 0);
            const float a = static_cast<float>(i) * 0.4f;
            out[2 * i] = c.FilterAxis(Axis::X, std::sin(a), t);
            out[2 * i + 1] = c.FilterAxis(Axis::Y, std::cos(a), t);
        }
    }
    bool same = true;
    for (int i = 0; i < 64; ++i)
        same = same && out1[i] == out2[i];
    Check(same, "deterministic");
    // During the kMenu heartbeat window (i 16..23) raw values pass through.
    Check(Near(out1[2 * 17], std::sin(17 * 0.4f)), "blocked window passes raw");
    Check(Near(out1[2 * 9], 0.0f), "active window rewrites X");
}

// Exercise the actual script bridge mask against the native consumer. The old ~64 mask let packed
// UpperBody diagnostics masquerade as kHold/kWeapon; a native-only gating test could not catch it.
static void TestScriptPsmBridge(const char* scriptPath)
{
    std::ifstream input(scriptPath);
    const std::string source((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    std::smatch match;
    const std::regex pattern(R"(UTPPPsmBlockMask\(scriptInterface\)\s*&\s*(~?)\s*([0-9]+)\s*;)");
    if (!std::regex_search(source, match, pattern))
    {
        Check(false, "read the production REDscript PSM bridge mask");
        return;
    }
    uint32_t mask = static_cast<uint32_t>(std::stoul(match[2].str()));
    if (match[1].str() == "~")
        mask = ~mask;
    Core c;
    MakeActive(c, 1.0);
    c.SetBlockMask(kDefaultBlockMask & ~kWeapon); // actual weapon-turn-enabled session profile
    c.Heartbeat(1.0, true, 1.5f, 0x2100C000u & mask); // recorded ForceEmptyHands, no hard blockers
    Check(c.Active(1.0), "recorded ForceEmptyHands must not disable Locomotion");
    c.FilterAxis(Axis::X, 0.0f, 1.0);
    Check(Near(c.FilterAxis(Axis::Y, -1.0f, 1.0), 1.0f), "forced-empty-hands backward input still remaps");
    Check(Near(c.RawAngle(1.0), 180.0f), "body turn retains the recorded backward direction");
    bool upperStatesClean = true;
    for (uint32_t upper = 0; upper < 7; ++upper)
    {
        const uint32_t packed = 0x21000000u | ((upper + 1u) << 13);
        c.Heartbeat(1.0, true, 1.5f, packed & mask);
        upperStatesClean = upperStatesClean && c.Active(1.0) && ((packed & mask) == 0);
    }
    Check(upperStatesClean, "all packed UpperBody states are diagnostics, not hold or weapon reasons");
    Check((0xFFFFE000u & mask) == 0, "all high packed diagnostics stay outside the Locomotion contract");
    bool guardsKept = true;
    for (uint32_t bit = 1; bit <= 4096; bit <<= 1)
    {
        c.Heartbeat(1.0, true, 1.5f, (0x2100C000u | bit) & mask);
        guardsKept = guardsKept && c.Reasons(1.0) == (bit == 64 ? 0 : bit);
    }
    Check(guardsKept, "real low PSM guards, including legacy melee guards, are preserved");
    c.Heartbeat(1.0, true, 1.5f, (0x2100E000u & mask) | kAiming);
    Check(c.Reasons(1.0) == kAiming, "explicit aim reason still blocks without a false hold");
    c.SetBlockMask(kDefaultBlockMask);
    c.Heartbeat(1.0, true, 1.5f, (0x61002000u & mask) | kWeapon);
    Check(c.Reasons(1.0) == kWeapon, "actual weapon reason still obeys configured blocking");
    c.Heartbeat(1.0, true, 1.5f, 0x2100C000u & mask);
    c.SetHold(true);
    Check(c.Reasons(1.0) == kHold, "real external hold remains effective");
}

int main(int argc, char** argv)
{
    TestAngles();
    TestRewrite();
    TestGating();
    TestParams();
    TestCoexistence();
    TestDispatcherScan();
    TestDeterminism();
    if (argc == 2)
        TestScriptPsmBridge(argv[1]);
    else
    {
        const std::string file = __FILE__;
        const std::string script = file.substr(0, file.find_last_of("/\\") + 1) + "../scripts/utpp_locomotion.reds";
        TestScriptPsmBridge(script.c_str());
    }
    std::printf("utpp_locomotion_test: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
