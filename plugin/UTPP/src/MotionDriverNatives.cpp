// port v2 integration: RED4ext natives + host glue of MotionDriver (FIXED NATIVE INTERFACE, see MotionDriver.hpp).
// Plugin only (the unit test links MotionDriver.cpp without this file). Registration follows Main.cpp PostRegisterTypes
// (CGlobalFunction::Create / AddParam / SetReturnType / RegisterFunction, flags isNative + isStatic).

#include <Windows.h>

#include <RED4ext/RED4ext.hpp>
#include <RED4ext/Scripting/Natives/Vector4.hpp> // UTPP-MOTION SEAM trav: UTPPFeedTraversal
#include <RED4ext/Scripting/Natives/Quaternion.hpp>     // ragdoll get-up: UTPPRagdollGetUpPlan

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include "CommonUtil.hpp"
#include "MotionHost.hpp"
#include "MotionDriver.hpp"
#include "ragdoll/RagdollSlotProbe.hpp" // ragdoll get-up: standing belly reference sign (PelvisTracker)

namespace utpp::motiondriver
{
namespace
{
const motionhost::MotionHooks kHooks{&Step, &ApplyToSnapshot, &OnArmed, &OnDisarmed, &Frame, &OwnsTraversal, &OwnsSlide,
                            &TravActive, &VehicleVaultWeapon}; // UTPP-MOTION SEAM ownsTraversal, ownsSlide, travActive (TRAV-1), vehicleVaultWeapon (vault torso limiter, keep last)

void Native_SetMotionEnabled(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool v = false;
    RED4ext::GetParameter(aFrame, &v);
    aFrame->code++; // ParamEnd
    SetEnabled(v);
}

// UTPP-MOTION SEAM motionfix item 2: mm.aimGameUpper (CET utpp_motion)
void Native_SetMotionAimGameUpper(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool v = false;
    RED4ext::GetParameter(aFrame, &v);
    aFrame->code++; // ParamEnd
    SetAimGameUpper(v);
}

void Native_SetMMRigBody(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    int32_t body = -1;
    RED4ext::GetParameter(aFrame, &body);
    aFrame->code++;
    SetRigBody(body); // an invalid value keeps the driver's body and clears the body reported on host poses
}

void Native_SetMMOverlayInputs(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    int32_t pose = 0;
    int32_t aim = 0;
    bool release = false;
    bool injured = false;
    int32_t resolvedAimPose = 0;
    RED4ext::GetParameter(aFrame, &pose);
    RED4ext::GetParameter(aFrame, &aim);
    RED4ext::GetParameter(aFrame, &release);
    RED4ext::GetParameter(aFrame, &injured);
    RED4ext::GetParameter(aFrame, &resolvedAimPose);
    aFrame->code++;
    SetOverlayInputs(pose, aim, release, injured, resolvedAimPose);
}

void Native_SetMMWeaponAction(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    int32_t action = 0;
    RED4ext::GetParameter(aFrame, &action);
    aFrame->code++;
    SetWeaponAction(action);
}

void Native_SetMMRecoilWeapon(RED4ext::IScriptable*, RED4ext::CStackFrame* frame, void*, int64_t)
{
    uint32_t epoch = 0; int32_t profile = 0;
    RED4ext::GetParameter(frame, &epoch); RED4ext::GetParameter(frame, &profile);
    frame->code++; SetRecoilProfile(epoch, profile);
}
void Native_SetMMAuthoredReload(RED4ext::IScriptable*, RED4ext::CStackFrame* frame, void*, int64_t)
{
    bool on = false; RED4ext::GetParameter(frame, &on);
    frame->code++; SetAuthoredReload(on);
}
void Native_FeedMMRecoilShot(RED4ext::IScriptable*, RED4ext::CStackFrame* frame, void*, int64_t)
{
    uint32_t epoch = 0; bool aiming = false;
    RED4ext::GetParameter(frame, &epoch); RED4ext::GetParameter(frame, &aiming);
    frame->code++; FeedRecoilShot(epoch, aiming);
}

void Native_GetMotionStatus(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::CString* aOut, int64_t)
{
    aFrame->code++;
    if (aOut)
    {
        try // natives run inside the game's script VM: no exception may leave them
        {
            const std::string s = Status();
            *aOut = RED4ext::CString(s.c_str());
        }
        catch (...)
        {
        }
    }
}

// UTPPFeedTraversalObstacleKind (scripts/utpp_motion.reds UTPPMotionFeedTraversalFrom, right before UTPPFeedTraversal): kind of the
// obstacle of the next traversal event (TraversalEvent::obstacleKind). Game thread only (both natives run in the script VM of
// the same state OnEnter); an atomic anyway, so a stray caller cannot tear it. Consumed and reset by Native_FeedTraversal.
std::atomic<int32_t> g_pendingObstacleKind{0};

void Native_FeedTraversalObstacleKind(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    int32_t kind = 0;
    RED4ext::GetParameter(aFrame, &kind);
    aFrame->code++; // ParamEnd
    g_pendingObstacleKind.store(kind >= kObstacleStatic && kind <= kObstacleOtherEntity ? kind : kObstacleStatic,
                                std::memory_order_relaxed);
}

// UTPP-MOTION SEAM trav: UTPPFeedTraversal (scripts/utpp_motion.reds ClimbEvents / VaultEvents OnEnter wraps). Parameter order =
// the redscript declaration; read in order like the other natives (GetParameter per parameter, then ParamEnd).
void Native_FeedTraversal(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    int32_t action = 0;
    RED4ext::Vector4 top{}, normal{}, behind{}, end{}, input{}, left{}, right{}, pos{}, fwd{};
    float topExtent = 0.0f, speed = 0.0f, climbSpeed = 1.0f;
    float highT = -1.0f, midT = -1.0f, vH = -1.0f, vM = -1.0f, vL = -1.0f, hH = -1.0f, hM = -1.0f, hL = -1.0f, fwdStep = -1.0f;
    bool hasHands = false, airborne = false;
    RED4ext::GetParameter(aFrame, &action);
    RED4ext::GetParameter(aFrame, &top);
    RED4ext::GetParameter(aFrame, &normal);
    RED4ext::GetParameter(aFrame, &topExtent);
    RED4ext::GetParameter(aFrame, &behind);
    RED4ext::GetParameter(aFrame, &end);
    RED4ext::GetParameter(aFrame, &input);
    RED4ext::GetParameter(aFrame, &hasHands);
    RED4ext::GetParameter(aFrame, &left);
    RED4ext::GetParameter(aFrame, &right);
    RED4ext::GetParameter(aFrame, &pos);
    RED4ext::GetParameter(aFrame, &fwd);
    RED4ext::GetParameter(aFrame, &speed);
    RED4ext::GetParameter(aFrame, &airborne);
    RED4ext::GetParameter(aFrame, &climbSpeed);
    RED4ext::GetParameter(aFrame, &highT); // review minor 5: live climb / vault tweaks (GetStaticFloatParameterDefault)
    RED4ext::GetParameter(aFrame, &midT);
    RED4ext::GetParameter(aFrame, &vH);
    RED4ext::GetParameter(aFrame, &vM);
    RED4ext::GetParameter(aFrame, &vL);
    RED4ext::GetParameter(aFrame, &hH);
    RED4ext::GetParameter(aFrame, &hM);
    RED4ext::GetParameter(aFrame, &hL);
    RED4ext::GetParameter(aFrame, &fwdStep);
    aFrame->code++; // ParamEnd
    TraversalEvent ev;
    auto put = [](float* aDst, const RED4ext::Vector4& aV) {
        aDst[0] = aV.X;
        aDst[1] = aV.Y;
        aDst[2] = aV.Z;
    };
    ev.action = action;
    put(ev.topPoint, top);
    put(ev.collisionNormal, normal);
    ev.topExtent = topExtent;
    put(ev.behindPoint, behind);
    put(ev.obstacleEnd, end);
    put(ev.inputDirection, input);
    ev.hasHands = hasHands;
    put(ev.leftHand, left);
    put(ev.rightHand, right);
    put(ev.playerPos, pos);
    put(ev.ownerForward, fwd);
    ev.speed = speed;
    ev.airborne = airborne;
    ev.climbSpeedModifier = climbSpeed;
    ev.haveTweaks = true;
    ev.highThreshold = highT;
    ev.midThreshold = midT;
    ev.verticalDuration[0] = vH;
    ev.verticalDuration[1] = vM;
    ev.verticalDuration[2] = vL;
    ev.horizontalDuration[0] = hH;
    ev.horizontalDuration[1] = hM;
    ev.horizontalDuration[2] = hL;
    ev.forwardStep = fwdStep;
    ev.obstacleKind = g_pendingObstacleKind.exchange(kObstacleStatic, std::memory_order_relaxed); // one event only
    bool ok = false;
    try // natives run inside the game's script VM: no exception may leave them
    {
        ok = FeedTraversal(ev);
    }
    catch (...)
    {
    }
    if (aOut)
        *aOut = ok;
}

// UTPPSetVehicleEntryHoldCap(seconds: Float) -> Void (scripts/utpp_vehicle_door.reds): per-mount limit of the standing-entry root hold
// (vehtrans HOLD CAP). Seconds after the mount cut by which the visible root must be back on the entity path; < 0 = none.
void Native_SetVehicleEntryHoldCap(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    float seconds = -1.0f;
    RED4ext::GetParameter(aFrame, &seconds);
    aFrame->code++; // ParamEnd
    try // natives run inside the game's script VM: no exception may leave them
    {
        motionhost::SetVehicleEntryHoldCap(seconds);
    }
    catch (...)
    {
    }
}

// ragdoll R0 probe (CET utpp_probe RagdollProbe): UTPPRagdollGetUpProbe(offsetM, durationS) -> String, ...Stop() -> String,
// ...Status() -> String. Debug only; nothing runs unless called (MotionDriver.hpp RagdollGetUpProbeStart).
void WriteString(RED4ext::CString* aOut, const std::string& aS)
{
    if (aOut)
        *aOut = RED4ext::CString(aS.c_str());
}

void Native_RagdollGetUpProbe(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::CString* aOut, int64_t)
{
    float offset = 0.0f, duration = 2.0f;
    RED4ext::GetParameter(aFrame, &offset);
    RED4ext::GetParameter(aFrame, &duration);
    aFrame->code++; // ParamEnd
    try // natives run inside the game's script VM: no exception may leave them
    {
        WriteString(aOut, RagdollGetUpProbeStart(offset, duration));
    }
    catch (...)
    {
    }
}

void Native_RagdollGetUpProbeStop(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::CString* aOut, int64_t)
{
    aFrame->code++;
    try
    {
        WriteString(aOut, RagdollGetUpProbeStop());
    }
    catch (...)
    {
    }
}

void Native_RagdollGetUpProbeStatus(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::CString* aOut, int64_t)
{
    aFrame->code++;
    try
    {
        WriteString(aOut, RagdollGetUpProbeStatus());
    }
    catch (...)
    {
    }
}

// ragdoll native get-up (scripts/utpp_ragdoll.reds UTPPRgRelease; spec section 3.6). Global functions, no object. The DLL keeps all state.
//   UTPPRagdollGetUpAvailable() -> Bool
//   UTPPRagdollGetUpPlan(entityPos: Vector4, entityRot: Quaternion) -> Int32   0..3 = B F L R, -1 unavailable, -2 no snapshot, -3 bones
//                                                                                missing, -4 pose-mismatch, -5 anchor-far, -6 busy
//   UTPPRagdollGetUpTarget() -> Vector4        anchor x, y, z, w = rms
//   UTPPRagdollGetUpForward() -> Vector4       world forward of the anchor (x, y, 0, 0)
//   UTPPRagdollGetUpBegin(groundZ: Float) -> Bool
//   UTPPRagdollGetUpCancel() -> Void
//   UTPPRagdollGetUpState() -> Int32           0 idle, 1 blend, 2 play, 3 handoff, -1 the last run aborted
//   UTPPRagdollGetUpStatus() -> String
//   UTPPRagdollSetGetUp(enabled: Bool) -> Void
int64_t QpcNow()
{
    LARGE_INTEGER n{};
    QueryPerformanceCounter(&n);
    return n.QuadPart;
}
int64_t QpcHz() { return util::QpcFrequency(); }

void Native_RagdollSetGetUp(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool enabled = false;
    RED4ext::GetParameter(aFrame, &enabled);
    aFrame->code++;
    RagdollSetGetUp(enabled);
}

void Native_RagdollGetUpAvailable(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    aFrame->code++;
    if (aOut)
        *aOut = RagdollGetUpAvailable(QpcNow(), QpcHz());
}

void Native_RagdollGetUpPlan(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, int32_t* aOut, int64_t)
{
    RED4ext::Vector4 pos{};
    RED4ext::Quaternion rot{};
    RED4ext::GetParameter(aFrame, &pos);
    RED4ext::GetParameter(aFrame, &rot);
    aFrame->code++;
    const float p[3] = {pos.X, pos.Y, pos.Z};
    const float q[4] = {rot.i, rot.j, rot.k, rot.r};
    const bool haveRef = utpp::ragdollslot::Pelvis().HasReference();
    const float sign = haveRef ? utpp::ragdollslot::Pelvis().ReferenceSign() : 1.0f;
    const int32_t code = RagdollGetUpPlan(p, q, QpcNow(), QpcHz(), haveRef, sign);
    if (aOut)
        *aOut = code;
}

void Native_RagdollGetUpTarget(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::Vector4* aOut, int64_t)
{
    aFrame->code++;
    float t[4];
    RagdollGetUpTarget(t);
    if (aOut)
    {
        aOut->X = t[0];
        aOut->Y = t[1];
        aOut->Z = t[2];
        aOut->W = t[3];
    }
}

void Native_RagdollGetUpForward(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::Vector4* aOut, int64_t)
{
    aFrame->code++;
    float f[2];
    RagdollGetUpForward(f);
    if (aOut)
    {
        aOut->X = f[0];
        aOut->Y = f[1];
        aOut->Z = 0.0f;
        aOut->W = 0.0f;
    }
}

void Native_RagdollGetUpBegin(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float groundZ = 0.0f;
    RED4ext::GetParameter(aFrame, &groundZ);
    aFrame->code++;
    const bool ok = RagdollGetUpBeginPublish(groundZ);
    if (aOut)
        *aOut = ok;
}

void Native_RagdollGetUpCancel(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    aFrame->code++;
    RagdollGetUpCancel();
}

void Native_RagdollGetUpState(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, int32_t* aOut, int64_t)
{
    aFrame->code++;
    if (aOut)
        *aOut = RagdollGetUpState();
}

void Native_RagdollGetUpStatus(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::CString* aOut, int64_t)
{
    aFrame->code++;
    try
    {
        WriteString(aOut, RagdollGetUpStatus());
    }
    catch (...)
    {
    }
}

// ragdoll R1d probe (utpp_ragdoll_probe.reds): UTPPRagdollCompDump(comp, hex) -> String. Read only, debug only: copies the first
// 0x200 bytes of an entRagdollComponent under SEH and decodes the activation fields of exe 2.5 (FUN_140287850 = activation
// job, FUN_14052a15c = activator): +0x8b enabled, +0x1b0 request mode (1/2 skip the pose pre-check and the 512 gate),
// +0x1d8 request pending, +0x1d9 active, +0x1db job took the request, +0x1df forced; pose double buffer: ptr +0xd8/+0xe8,
// count +0xe4/+0xf4, selector +0xf8 (the activator needs both counts > 0); +0x50 bound animated component.
constexpr size_t kRagdollCompBytes = 0x200;

bool SafeCopy(void* aDst, const void* aSrc, size_t aN)
{
    __try
    {
        memcpy(aDst, aSrc, aN);
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

std::string RagdollCompDump(const void* aComp, bool aHex)
{
    if (!aComp)
        return "rcomp none";
    uint8_t b[kRagdollCompBytes];
    if (!SafeCopy(b, aComp, sizeof(b)))
        return "rcomp unreadable";
    const auto u32 = [&b](size_t aO) {
        uint32_t v;
        memcpy(&v, b + aO, sizeof(v));
        return v;
    };
    const auto ptr = [&b](size_t aO) {
        uint64_t v;
        memcpy(&v, b + aO, sizeof(v));
        return v != 0 ? 1 : 0;
    };
    char line[256];
    snprintf(line, sizeof(line),
             "rcomp en=%u mode=%u req=%u act=%u 1da=%u job=%u forced=%u 1b9=%u 1ba=%u sel=%u cntA=%u cntB=%u ptrA=%d ptrB=%d anim=%d",
             b[0x8b], u32(0x1b0), b[0x1d8], b[0x1d9], b[0x1da], b[0x1db], b[0x1df], b[0x1b9], b[0x1ba], u32(0xf8), u32(0xe4),
             u32(0xf4), ptr(0xd8), ptr(0xe8), ptr(0x50));
    std::string s(line);
    if (aHex)
    {
        static const char kDigits[] = "0123456789abcdef";
        s += " hex40=";
        for (size_t i = 0x40; i < kRagdollCompBytes; ++i)
        {
            if (i > 0x40 && (i % 16) == 0)
                s += '.';
            s += kDigits[b[i] >> 4];
            s += kDigits[b[i] & 15];
        }
    }
    return s;
}

void Native_RagdollCompDump(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::CString* aOut, int64_t)
{
    RED4ext::Handle<RED4ext::IScriptable> comp;
    bool hex = false;
    RED4ext::GetParameter(aFrame, &comp);
    RED4ext::GetParameter(aFrame, &hex);
    aFrame->code++;
    try
    {
        WriteString(aOut, RagdollCompDump(comp ? comp.instance : nullptr, hex));
    }
    catch (...)
    {
    }
}

std::string Narrow(const std::wstring& aW)
{
    if (aW.empty())
        return {};
    const int n = WideCharToMultiByte(CP_ACP, 0, aW.c_str(), static_cast<int>(aW.size()), nullptr, 0, nullptr, nullptr);
    if (n <= 0)
        return {};
    std::string s(static_cast<size_t>(n), '\0');
    WideCharToMultiByte(CP_ACP, 0, aW.c_str(), static_cast<int>(aW.size()), s.data(), n, nullptr, nullptr);
    return s;
}
} // namespace

void RegisterNatives()
{
    auto* rtti = RED4ext::CRTTISystem::Get();
    const RED4ext::CBaseFunction::Flags flags = {.isNative = true, .isStatic = true};

    auto* f1 = RED4ext::CGlobalFunction::Create("UTPPSetMotionEnabled", "UTPPSetMotionEnabled", &Native_SetMotionEnabled);
    f1->flags = flags;
    f1->AddParam("Bool", "on");
    rtti->RegisterFunction(f1);

    auto* fc = RED4ext::CGlobalFunction::Create("UTPPSetVehicleEntryHoldCap", "UTPPSetVehicleEntryHoldCap", &Native_SetVehicleEntryHoldCap);
    fc->flags = flags;
    fc->AddParam("Float", "seconds");
    rtti->RegisterFunction(fc);

    auto* f2 = RED4ext::CGlobalFunction::Create("UTPPSetMMRigBody", "UTPPSetMMRigBody", &Native_SetMMRigBody);
    f2->flags = flags;
    f2->AddParam("Int32", "body");
    rtti->RegisterFunction(f2);

    auto* f3 = RED4ext::CGlobalFunction::Create("UTPPSetMMOverlayInputs", "UTPPSetMMOverlayInputs", &Native_SetMMOverlayInputs);
    f3->flags = flags;
    f3->AddParam("Int32", "pose");
    f3->AddParam("Int32", "aim");
    f3->AddParam("Bool", "release");
    f3->AddParam("Bool", "injured");
    f3->AddParam("Int32", "resolvedAimPose");
    rtti->RegisterFunction(f3);

    auto* fw = RED4ext::CGlobalFunction::Create("UTPPSetMMWeaponAction", "UTPPSetMMWeaponAction", &Native_SetMMWeaponAction);
    fw->flags = flags;
    fw->AddParam("Int32", "action");
    rtti->RegisterFunction(fw);
    auto* rw = RED4ext::CGlobalFunction::Create("UTPPSetMMRecoilWeapon", "UTPPSetMMRecoilWeapon", &Native_SetMMRecoilWeapon);
    rw->flags = flags; rw->AddParam("Uint32", "weaponEpoch"); rw->AddParam("Int32", "recoilProfile");
    rtti->RegisterFunction(rw);
    auto* rs = RED4ext::CGlobalFunction::Create("UTPPFeedMMRecoilShot", "UTPPFeedMMRecoilShot", &Native_FeedMMRecoilShot);
    rs->flags = flags; rs->AddParam("Uint32", "weaponEpoch"); rs->AddParam("Bool", "aiming");
    rtti->RegisterFunction(rs);
    auto* ra = RED4ext::CGlobalFunction::Create("UTPPSetMMAuthoredReload", "UTPPSetMMAuthoredReload", &Native_SetMMAuthoredReload);
    ra->flags = flags; ra->AddParam("Bool", "on"); rtti->RegisterFunction(ra);

    // UTPP-MOTION SEAM motionfix item 2
    auto* fa = RED4ext::CGlobalFunction::Create("UTPPSetMotionAimGameUpper", "UTPPSetMotionAimGameUpper", &Native_SetMotionAimGameUpper);
    fa->flags = flags;
    fa->AddParam("Bool", "on");
    rtti->RegisterFunction(fa);

    auto* f4 = RED4ext::CGlobalFunction::Create("UTPPGetMotionStatus", "UTPPGetMotionStatus", &Native_GetMotionStatus);
    f4->flags = flags;
    f4->SetReturnType("String");
    rtti->RegisterFunction(f4);

    // UTPP-MOTION SEAM trav
    auto* f5 = RED4ext::CGlobalFunction::Create("UTPPFeedTraversal", "UTPPFeedTraversal", &Native_FeedTraversal);
    f5->flags = flags;
    f5->AddParam("Int32", "action");
    f5->AddParam("Vector4", "topPoint");
    f5->AddParam("Vector4", "collisionNormal");
    f5->AddParam("Float", "topExtent");
    f5->AddParam("Vector4", "behindPoint");
    f5->AddParam("Vector4", "obstacleEnd");
    f5->AddParam("Vector4", "inputDirection");
    f5->AddParam("Bool", "hasHands");
    f5->AddParam("Vector4", "leftHand");
    f5->AddParam("Vector4", "rightHand");
    f5->AddParam("Vector4", "playerPos");
    f5->AddParam("Vector4", "ownerForward");
    f5->AddParam("Float", "speed");
    f5->AddParam("Bool", "airborne");
    f5->AddParam("Float", "climbSpeedModifier");
    f5->AddParam("Float", "highThreshold"); // review minor 5
    f5->AddParam("Float", "midThreshold");
    f5->AddParam("Float", "verticalDurationHigh");
    f5->AddParam("Float", "verticalDurationMid");
    f5->AddParam("Float", "verticalDurationLow");
    f5->AddParam("Float", "horizontalDurationHigh");
    f5->AddParam("Float", "horizontalDurationMid");
    f5->AddParam("Float", "horizontalDurationLow");
    f5->AddParam("Float", "forwardStep");
    f5->SetReturnType("Bool");
    rtti->RegisterFunction(f5);

    auto* f6 = RED4ext::CGlobalFunction::Create("UTPPFeedTraversalObstacleKind", "UTPPFeedTraversalObstacleKind",
                                                &Native_FeedTraversalObstacleKind);
    f6->flags = flags;
    f6->AddParam("Int32", "kind");
    rtti->RegisterFunction(f6);

    // ragdoll R0 probe
    auto* r1 = RED4ext::CGlobalFunction::Create("UTPPRagdollGetUpProbe", "UTPPRagdollGetUpProbe", &Native_RagdollGetUpProbe);
    r1->flags = flags;
    r1->AddParam("Float", "offsetM");
    r1->AddParam("Float", "durationS");
    r1->SetReturnType("String");
    rtti->RegisterFunction(r1);
    auto* r2 = RED4ext::CGlobalFunction::Create("UTPPRagdollGetUpProbeStop", "UTPPRagdollGetUpProbeStop", &Native_RagdollGetUpProbeStop);
    r2->flags = flags;
    r2->SetReturnType("String");
    rtti->RegisterFunction(r2);
    auto* r3 = RED4ext::CGlobalFunction::Create("UTPPRagdollGetUpProbeStatus", "UTPPRagdollGetUpProbeStatus", &Native_RagdollGetUpProbeStatus);
    r3->flags = flags;
    r3->SetReturnType("String");
    rtti->RegisterFunction(r3);
    // ragdoll native get-up (the exact list C's scripts/utpp_ragdoll.reds declares)
    auto* g0 = RED4ext::CGlobalFunction::Create("UTPPRagdollGetUpAvailable", "UTPPRagdollGetUpAvailable", &Native_RagdollGetUpAvailable);
    g0->flags = flags;
    g0->SetReturnType("Bool");
    rtti->RegisterFunction(g0);
    auto* g1 = RED4ext::CGlobalFunction::Create("UTPPRagdollGetUpPlan", "UTPPRagdollGetUpPlan", &Native_RagdollGetUpPlan);
    g1->flags = flags;
    g1->AddParam("Vector4", "entityPos");
    g1->AddParam("Quaternion", "entityRot");
    g1->SetReturnType("Int32");
    rtti->RegisterFunction(g1);
    auto* g2 = RED4ext::CGlobalFunction::Create("UTPPRagdollGetUpTarget", "UTPPRagdollGetUpTarget", &Native_RagdollGetUpTarget);
    g2->flags = flags;
    g2->SetReturnType("Vector4");
    rtti->RegisterFunction(g2);
    auto* g3 = RED4ext::CGlobalFunction::Create("UTPPRagdollGetUpForward", "UTPPRagdollGetUpForward", &Native_RagdollGetUpForward);
    g3->flags = flags;
    g3->SetReturnType("Vector4");
    rtti->RegisterFunction(g3);
    auto* g4 = RED4ext::CGlobalFunction::Create("UTPPRagdollGetUpBegin", "UTPPRagdollGetUpBegin", &Native_RagdollGetUpBegin);
    g4->flags = flags;
    g4->AddParam("Float", "groundZ");
    g4->SetReturnType("Bool");
    rtti->RegisterFunction(g4);
    auto* g5 = RED4ext::CGlobalFunction::Create("UTPPRagdollGetUpCancel", "UTPPRagdollGetUpCancel", &Native_RagdollGetUpCancel);
    g5->flags = flags;
    rtti->RegisterFunction(g5);
    auto* g6 = RED4ext::CGlobalFunction::Create("UTPPRagdollGetUpState", "UTPPRagdollGetUpState", &Native_RagdollGetUpState);
    g6->flags = flags;
    g6->SetReturnType("Int32");
    rtti->RegisterFunction(g6);
    auto* g7 = RED4ext::CGlobalFunction::Create("UTPPRagdollGetUpStatus", "UTPPRagdollGetUpStatus", &Native_RagdollGetUpStatus);
    g7->flags = flags;
    g7->SetReturnType("String");
    rtti->RegisterFunction(g7);
    auto* g8 = RED4ext::CGlobalFunction::Create("UTPPRagdollSetGetUp", "UTPPRagdollSetGetUp", &Native_RagdollSetGetUp);
    g8->flags = flags;
    g8->AddParam("Bool", "enabled");
    rtti->RegisterFunction(g8);
    // ragdoll R1d probe
    auto* r4 =RED4ext::CGlobalFunction::Create("UTPPRagdollCompDump", "UTPPRagdollCompDump", &Native_RagdollCompDump);
    r4->flags = flags;
    r4->AddParam("handle:IComponent", "comp");
    r4->AddParam("Bool", "hex");
    r4->SetReturnType("String");
    rtti->RegisterFunction(r4);
}

void Install(const std::wstring& aDllDir)
{
    try
    {
        SetDataDir(Narrow(aDllDir));
    }
    catch (...)
    {
        return; // no data dir: hooks stay uninstalled, v1 only
    }
    motionhost::SetMotionHooks(&kHooks);
    SetGetUpGapPublisher(&motionhost::PublishGetUpGap);
}

void Uninstall()
{
    SetGetUpGapPublisher(nullptr);
    motionhost::SetMotionHooks(nullptr); // takes the v1 g_mutex: no Step / Apply runs after this returns
    Shutdown();
}
} // namespace utpp::motiondriver
