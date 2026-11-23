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
#include "mm/GroundFeetRuntime.hpp" // feet on uneven ground: natives of the stage and the ground recorder
#include "PluginLog.hpp"
#include "ragdoll/RagdollSlotProbe.hpp" // ragdoll get-up: standing belly reference sign (PelvisTracker)

namespace utpp::motionhost
{
void SetPelvisPlacement(bool aOn); // MotionHost.cpp
void SetBodyHeightSmooth(bool aOn); // MotionHost.cpp
void SetGroundFeet(bool aOn); // MotionHost.cpp
void SetAirStepDownReach(bool aOn); // MotionHost.cpp
}
namespace utpp::motiondriver
{
// MotionDriver.cpp: optional behaviours (default off)
bool SetLeanCoverage(int32_t aCoverage);
bool SetAimWorldSmoothing(bool aOn, float aHz);
bool SetNeutralAimOffset(bool aOn);
void SetHandBackBlend(bool aOn);
void SetStepDownReach(bool aOn);
void SetCapsuleForecast(bool aOn);
void SetAimRotation(bool aOn);
void SetFarJumpGravityScale(bool aOn);
bool SetOverlayInertialSwitch(bool aOn);
}

namespace utpp::motiondriver
{
namespace
{
const motionhost::MotionHooks kHooks{&Step, &ApplyToSnapshot, &OnArmed, &OnDisarmed, &Frame, &OwnsTraversal, &OwnsSlide,
                            &TravActive, &VehicleVaultWeapon, &RecordPublishedPose}; // UTPP-MOTION SEAM ownsTraversal, ownsSlide, travActive (TRAV-1), vehicleVaultWeapon (vault torso limiter), published (live pose history, keep last)

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
void Native_FeedMMRecoilShotLeft(RED4ext::IScriptable*, RED4ext::CStackFrame* frame, void*, int64_t)
{
    uint32_t epoch = 0; int32_t profile = 0; bool aiming = false;
    RED4ext::GetParameter(frame, &epoch); RED4ext::GetParameter(frame, &profile); RED4ext::GetParameter(frame, &aiming);
    frame->code++; FeedRecoilShotLeft(epoch, profile, aiming);
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

// optional motion behaviours (default off; CET utpp_motion mm.pelvisPlacement, mm.handBackBlend, mm.stepDownReach,
// mm.farJumpGravity, mm.leanCoverage)
void Native_SetMMPelvisPlacement(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool v = false;
    RED4ext::GetParameter(aFrame, &v);
    aFrame->code++;
    motionhost::SetPelvisPlacement(v);
}
// body height smoothing on uneven ground (mm/BodyHeightRuntime.hpp; CET utpp_motion mm.bodyHeightSmooth, default on)
void Native_SetMMBodyHeightSmooth(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool v = false;
    RED4ext::GetParameter(aFrame, &v);
    aFrame->code++;
    motionhost::SetBodyHeightSmooth(v);
    pluginlog::LogInfo("UTPPSetMMBodyHeightSmooth(%d) -> 1", v ? 1 : 0);
}
void Native_SetMMHandBackBlend(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool v = false;
    RED4ext::GetParameter(aFrame, &v);
    aFrame->code++;
    SetHandBackBlend(v);
}
void Native_SetMMStepDownReach(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool v = false;
    RED4ext::GetParameter(aFrame, &v);
    aFrame->code++;
    SetStepDownReach(v); // chooser mapping
    motionhost::SetAirStepDownReach(v); // runtime air state: one switch for both
}
// capsule forecast for turning and partial-stick starts (MapConfig::capsuleForecast; CET utpp_motion mm.capsuleForecast, default off)
void Native_SetMMCapsuleForecast(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool v = false;
    RED4ext::GetParameter(aFrame, &v);
    aFrame->code++;
    SetCapsuleForecast(v);
    pluginlog::LogInfo("UTPPSetMMCapsuleForecast(%d) -> 1", v ? 1 : 0);
}
// aim rotation sub-mode while aiming (MapConfig::aimRotation; CET utpp_motion mm.aimRotation, default off)
void Native_SetMMAimRotation(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool v = false;
    RED4ext::GetParameter(aFrame, &v);
    aFrame->code++;
    SetAimRotation(v);
    pluginlog::LogInfo("UTPPSetMMAimRotation(%d) -> 1", v ? 1 : 0);
}
void Native_SetMMFarJumpGravity(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool v = false;
    RED4ext::GetParameter(aFrame, &v);
    aFrame->code++;
    SetFarJumpGravityScale(v);
}
void Native_SetMMAimWorldSmoothing(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    bool on = false;
    float hz = 0.0f;
    RED4ext::GetParameter(aFrame, &on);
    RED4ext::GetParameter(aFrame, &hz);
    aFrame->code++;
    const bool ok = SetAimWorldSmoothing(on, hz);
    if (aOut)
        *aOut = ok;
}
void Native_SetMMNeutralAimOffset(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    bool on = false;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    const bool ok = SetNeutralAimOffset(on);
    if (aOut)
        *aOut = ok;
}
void Native_SetMotionLeanCoverage(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    int32_t v = 0;
    RED4ext::GetParameter(aFrame, &v);
    aFrame->code++;
    const bool ok = SetLeanCoverage(v);
    if (aOut)
        *aOut = ok;
}
// stride warping (CET utpp_motion mm.strideWarp; the DLL default is off, CET sends its value). Logged in the setting-call form the
// offline replay applies ("UTPPSet<Name>(<args>) -> <result>").
void Native_SetMMStrideWarp(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    bool on = false;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    const bool ok = SetStrideWarp(on);
    pluginlog::LogInfo("UTPPSetMMStrideWarp(%d) -> %d", on ? 1 : 0, ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}
// inertialized overlay switch (CET utpp_motion mm.overlayInertialSwitch; default off). Logged in the setting-call form the offline
// replay applies.
void Native_SetMMOverlayInertialSwitch(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    bool on = false;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    const bool ok = SetOverlayInertialSwitch(on);
    pluginlog::LogInfo("UTPPSetMMOverlayInertialSwitch(%d) -> %d", on ? 1 : 0, ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}
// firm feet in turns and stops (CET utpp_motion mm.firmFeet; DLL default on). Logged in the setting-call form the offline replay applies.
void Native_SetMMFirmFeet(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    bool on = false;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    const bool ok = motionhost::SetFirmFeet(on);
    pluginlog::LogInfo("UTPPSetMMFirmFeet(%d) -> %d", on ? 1 : 0, ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}
// ---- feet on uneven ground (mm/GroundFeetRuntime.hpp; CET utpp_motion mm.groundFeet, default off) and the ground recorder
// (mm.trace: traces under the published feet, logged as UTPP GRT lines, no pose change). Script side: scripts/utpp.reds
// UTPPFeedGroundFromLocomotion, right before the locomotion feed. Yaw: the feed's player yaw (deg, forward = (-sin, cos)).
void Native_SetMMGroundFeet(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool v = false;
    RED4ext::GetParameter(aFrame, &v);
    aFrame->code++;
    motionhost::SetGroundFeet(v);
    pluginlog::LogInfo("UTPPSetMMGroundFeet(%d) -> 1", v ? 1 : 0);
}
void Native_SetMMGroundFeetBias(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    float v = 0.0f;
    RED4ext::GetParameter(aFrame, &v);
    aFrame->code++;
    mm::groundfeet::rt::SetBias(v);
    pluginlog::LogInfo("UTPPSetMMGroundFeetBias(%.4f)", double(v));
}
// True when the script should trace this frame: the stage is on with its gate open, or the recorder is on; and a published pose
// gives the points.
void Native_GroundWantsTrace(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    aFrame->code++;
    namespace gf = mm::groundfeet::rt;
    const bool want = gf::g_probe.valid && ((gf::Enabled() && gf::GateOpen()) || gf::Recording());
    if (aOut)
        *aOut = want;
}
// The world point to trace under for one foot (0 left, 1 right): x, y = the trace point (the stage leads a swinging foot by one
// step; the recorder alone traces under the published ball), z = the published ball height; w = 1 valid, 0 none.
void Native_GroundProbePoint(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::Vector4* aOut, int64_t)
{
    int32_t side = 0;
    RED4ext::Vector4 cap{};
    float yawDeg = 0.0f;
    RED4ext::GetParameter(aFrame, &side);
    RED4ext::GetParameter(aFrame, &cap);
    RED4ext::GetParameter(aFrame, &yawDeg);
    aFrame->code++;
    namespace gf = mm::groundfeet::rt;
    const float yaw = yawDeg * 0.0174532925f;
    float ball[3] = {0.0f, 0.0f, 0.0f};
    float x = 0.0f, y = 0.0f;
    bool ok = gf::BallWorld(side, cap.X, cap.Y, cap.Z, yaw, ball);
    if (ok && gf::Enabled())
        ok = gf::ProbePoint(side, cap.X, cap.Y, yaw, x, y);
    else
        x = ball[0], y = ball[1];
    if (aOut)
    {
        aOut->X = ok ? x : 0.0f;
        aOut->Y = ok ? y : 0.0f;
        aOut->Z = ok ? ball[2] : 0.0f;
        aOut->W = ok ? 1.0f : 0.0f;
    }
}
void Native_GroundTraceBegin(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    aFrame->code++;
    LARGE_INTEGER t{};
    QueryPerformanceCounter(&t);
    mm::groundfeet::rt::g_traceBegin = t.QuadPart;
}
// Microseconds since UTPPGroundTraceBegin (the cost of the script's trace block); kept as the last and the largest value.
void Native_GroundTraceEnd(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, float* aOut, int64_t)
{
    aFrame->code++;
    namespace gf = mm::groundfeet::rt;
    LARGE_INTEGER t{};
    QueryPerformanceCounter(&t);
    const float us = gf::g_traceBegin != 0
                         ? static_cast<float>(double(t.QuadPart - gf::g_traceBegin) * 1.0e6 / double(util::QpcFrequency()))
                         : 0.0f;
    gf::g_traceBegin = 0;
    gf::g_traceUs.store(us, std::memory_order_relaxed);
    gf::StoreMax(gf::g_traceMaxUs, us);
    if (aOut)
        *aOut = us;
}
// The traces of this frame (hit flags, world hit points and normals) with the capsule and yaw of the probe points. Hands them to the
// stage and, with the recorder on, logs one UTPP GRT line: per foot the published ball, the hit, hit - capsule z, ball - hit, the
// normal and the game's world foot IK joint (local translation before the UTPP write), then the trace cost and the stage state.
void Native_FeedGround(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool hit[2] = {false, false};
    RED4ext::Vector4 pos[2]{}, nrm[2]{}, cap{};
    float yawDeg = 0.0f;
    RED4ext::GetParameter(aFrame, &hit[0]);
    RED4ext::GetParameter(aFrame, &pos[0]);
    RED4ext::GetParameter(aFrame, &nrm[0]);
    RED4ext::GetParameter(aFrame, &hit[1]);
    RED4ext::GetParameter(aFrame, &pos[1]);
    RED4ext::GetParameter(aFrame, &nrm[1]);
    RED4ext::GetParameter(aFrame, &cap);
    RED4ext::GetParameter(aFrame, &yawDeg);
    aFrame->code++;
    namespace gf = mm::groundfeet::rt;
    const float yaw = yawDeg * 0.0174532925f;
    LARGE_INTEGER now{};
    QueryPerformanceCounter(&now);
    float p[2][3], n[2][3];
    for (int s = 0; s < 2; ++s)
    {
        p[s][0] = pos[s].X, p[s][1] = pos[s].Y, p[s][2] = pos[s].Z;
        n[s][0] = nrm[s].X, n[s][1] = nrm[s].Y, n[s][2] = nrm[s].Z;
    }
    if (gf::Enabled())
        gf::PublishSamples(hit, p, n, cap.X, cap.Y, cap.Z, yaw, now.QuadPart);
    if (!gf::Recording())
        return;
    char line[768];
    int len = std::snprintf(line, sizeof(line), "UTPP GRT t=%.4f cap=%.3f,%.3f,%.3f yaw=%.2f",
                            double(now.QuadPart) / double(util::QpcFrequency()), double(cap.X), double(cap.Y), double(cap.Z),
                            double(yawDeg));
    for (int s = 0; s < 2 && len > 0 && len < int(sizeof(line)); ++s)
    {
        float ball[3] = {0.0f, 0.0f, 0.0f};
        const bool haveBall = gf::BallWorld(s, cap.X, cap.Y, cap.Z, yaw, ball);
        len += std::snprintf(line + len, sizeof(line) - size_t(len),
                             " %s: hit=%d ball=%.3f,%.3f,%.3f g=%.3f,%.3f,%.3f gCap=%.3f ballG=%.3f n=%.2f,%.2f,%.2f ik=%.3f,%.3f,%.3f",
                             s == 0 ? "L" : "R", hit[s] ? 1 : 0, double(ball[0]), double(ball[1]), double(ball[2]), double(p[s][0]),
                             double(p[s][1]), double(p[s][2]), hit[s] ? double(p[s][2] - cap.Z) : -9.0,
                             hit[s] && haveBall ? double(ball[2] - p[s][2]) : -9.0, double(n[s][0]), double(n[s][1]), double(n[s][2]),
                             double(gf::g_ikPre[s][0].load(std::memory_order_relaxed)),
                             double(gf::g_ikPre[s][1].load(std::memory_order_relaxed)),
                             double(gf::g_ikPre[s][2].load(std::memory_order_relaxed)));
    }
    if (len > 0 && len < int(sizeof(line)))
        std::snprintf(line + len, sizeof(line) - size_t(len),
                      " traceUs=%.1f traceMaxUs=%.1f stage=%d gate=%d w=%.2f,%.2f planeH=%.3f,%.3f lift=%.3f,%.3f pelvis=%.3f stageUs=%.1f "
                      "stageMaxUs=%.1f applied=%llu identity=%llu noSamples=%llu knee=%llu",
                      double(gf::g_traceUs.load(std::memory_order_relaxed)), double(gf::g_traceMaxUs.load(std::memory_order_relaxed)),
                      gf::Enabled() ? 1 : 0, gf::GateOpen() ? 1 : 0, double(gf::g_weight[0].load(std::memory_order_relaxed)),
                      double(gf::g_weight[1].load(std::memory_order_relaxed)), double(gf::g_planeH[0].load(std::memory_order_relaxed)),
                      double(gf::g_planeH[1].load(std::memory_order_relaxed)), double(gf::g_lift[0].load(std::memory_order_relaxed)),
                      double(gf::g_lift[1].load(std::memory_order_relaxed)), double(gf::g_pelvis.load(std::memory_order_relaxed)),
                      double(gf::g_lastUs.load(std::memory_order_relaxed)), double(gf::g_maxUs.load(std::memory_order_relaxed)),
                      static_cast<unsigned long long>(gf::g_applied.load(std::memory_order_relaxed)),
                      static_cast<unsigned long long>(gf::g_identity.load(std::memory_order_relaxed)),
                      static_cast<unsigned long long>(gf::g_noSamples.load(std::memory_order_relaxed)),
                      static_cast<unsigned long long>(gf::g_kneeLimits.load(std::memory_order_relaxed)));
    pluginlog::LogInfo("%s", line);
}
void Native_GroundRecorderOn(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    aFrame->code++;
    if (aOut)
        *aOut = mm::groundfeet::rt::Recording();
}
// body yaw offset per body (CET utpp_motion mm.bodyYawOffsetFemale / mm.bodyYawOffsetMale): deg, + = left, -20..20 each
void Native_SetMotionBodyYawOffsets(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float female = 0.0f, male = 0.0f;
    RED4ext::GetParameter(aFrame, &female);
    RED4ext::GetParameter(aFrame, &male);
    aFrame->code++;
    const bool ok = SetBodyYawOffsets(female, male);
    pluginlog::LogInfo("UTPPSetMotionBodyYawOffsets(%.2f,%.2f) -> %d", static_cast<double>(female), static_cast<double>(male), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

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
    auto* rsl = RED4ext::CGlobalFunction::Create("UTPPFeedMMRecoilShotLeft", "UTPPFeedMMRecoilShotLeft", &Native_FeedMMRecoilShotLeft);
    rsl->flags = flags; rsl->AddParam("Uint32", "weaponEpoch"); rsl->AddParam("Int32", "recoilProfile"); rsl->AddParam("Bool", "aiming");
    rtti->RegisterFunction(rsl);
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
    // optional motion behaviours (default off)
    auto* o1 = RED4ext::CGlobalFunction::Create("UTPPSetMMPelvisPlacement", "UTPPSetMMPelvisPlacement", &Native_SetMMPelvisPlacement);
    o1->flags = flags; o1->AddParam("Bool", "on"); rtti->RegisterFunction(o1);
    auto* o1b = RED4ext::CGlobalFunction::Create("UTPPSetMMBodyHeightSmooth", "UTPPSetMMBodyHeightSmooth", &Native_SetMMBodyHeightSmooth);
    o1b->flags = flags; o1b->AddParam("Bool", "on"); rtti->RegisterFunction(o1b);
    auto* o2 = RED4ext::CGlobalFunction::Create("UTPPSetMMHandBackBlend", "UTPPSetMMHandBackBlend", &Native_SetMMHandBackBlend);
    o2->flags = flags; o2->AddParam("Bool", "on"); rtti->RegisterFunction(o2);
    auto* o3 = RED4ext::CGlobalFunction::Create("UTPPSetMMStepDownReach", "UTPPSetMMStepDownReach", &Native_SetMMStepDownReach);
    o3->flags = flags; o3->AddParam("Bool", "on"); rtti->RegisterFunction(o3);
    auto* o3b = RED4ext::CGlobalFunction::Create("UTPPSetMMCapsuleForecast", "UTPPSetMMCapsuleForecast", &Native_SetMMCapsuleForecast);
    o3b->flags = flags; o3b->AddParam("Bool", "on"); rtti->RegisterFunction(o3b);
    auto* o3c = RED4ext::CGlobalFunction::Create("UTPPSetMMAimRotation", "UTPPSetMMAimRotation", &Native_SetMMAimRotation);
    o3c->flags = flags; o3c->AddParam("Bool", "on"); rtti->RegisterFunction(o3c);
    auto* o4 = RED4ext::CGlobalFunction::Create("UTPPSetMMFarJumpGravity", "UTPPSetMMFarJumpGravity", &Native_SetMMFarJumpGravity);
    o4->flags = flags; o4->AddParam("Bool", "on"); rtti->RegisterFunction(o4);
    auto* o5 = RED4ext::CGlobalFunction::Create("UTPPSetMotionLeanCoverage", "UTPPSetMotionLeanCoverage", &Native_SetMotionLeanCoverage);
    o5->flags = flags; o5->AddParam("Int32", "coverage"); o5->SetReturnType("Bool"); rtti->RegisterFunction(o5);
    auto* o6 = RED4ext::CGlobalFunction::Create("UTPPSetMMAimWorldSmoothing", "UTPPSetMMAimWorldSmoothing", &Native_SetMMAimWorldSmoothing);
    o6->flags = flags; o6->AddParam("Bool", "on"); o6->AddParam("Float", "hz"); o6->SetReturnType("Bool"); rtti->RegisterFunction(o6);
    auto* o7 = RED4ext::CGlobalFunction::Create("UTPPSetMMNeutralAimOffset", "UTPPSetMMNeutralAimOffset", &Native_SetMMNeutralAimOffset);
    o7->flags = flags; o7->AddParam("Bool", "on"); o7->SetReturnType("Bool"); rtti->RegisterFunction(o7);
    auto* o8 = RED4ext::CGlobalFunction::Create("UTPPSetMMStrideWarp", "UTPPSetMMStrideWarp", &Native_SetMMStrideWarp);
    o8->flags = flags; o8->AddParam("Bool", "on"); o8->SetReturnType("Bool"); rtti->RegisterFunction(o8);
    auto* o9 = RED4ext::CGlobalFunction::Create("UTPPSetMotionBodyYawOffsets", "UTPPSetMotionBodyYawOffsets", &Native_SetMotionBodyYawOffsets);
    o9->flags = flags; o9->AddParam("Float", "female"); o9->AddParam("Float", "male"); o9->SetReturnType("Bool");
    rtti->RegisterFunction(o9);
    auto* o10 = RED4ext::CGlobalFunction::Create("UTPPSetMMOverlayInertialSwitch", "UTPPSetMMOverlayInertialSwitch",
                                                 &Native_SetMMOverlayInertialSwitch);
    o10->flags = flags; o10->AddParam("Bool", "on"); o10->SetReturnType("Bool"); rtti->RegisterFunction(o10);
    auto* o11 = RED4ext::CGlobalFunction::Create("UTPPSetMMFirmFeet", "UTPPSetMMFirmFeet", &Native_SetMMFirmFeet);
    o11->flags = flags; o11->AddParam("Bool", "on"); o11->SetReturnType("Bool"); rtti->RegisterFunction(o11);
    // feet on uneven ground + ground recorder
    auto* gf1 = RED4ext::CGlobalFunction::Create("UTPPSetMMGroundFeet", "UTPPSetMMGroundFeet", &Native_SetMMGroundFeet);
    gf1->flags = flags; gf1->AddParam("Bool", "on"); rtti->RegisterFunction(gf1);
    auto* gf2 = RED4ext::CGlobalFunction::Create("UTPPSetMMGroundFeetBias", "UTPPSetMMGroundFeetBias", &Native_SetMMGroundFeetBias);
    gf2->flags = flags; gf2->AddParam("Float", "bias"); rtti->RegisterFunction(gf2);
    auto* gf3 = RED4ext::CGlobalFunction::Create("UTPPGroundWantsTrace", "UTPPGroundWantsTrace", &Native_GroundWantsTrace);
    gf3->flags = flags; gf3->SetReturnType("Bool"); rtti->RegisterFunction(gf3);
    auto* gf4 = RED4ext::CGlobalFunction::Create("UTPPGroundProbePoint", "UTPPGroundProbePoint", &Native_GroundProbePoint);
    gf4->flags = flags; gf4->AddParam("Int32", "side"); gf4->AddParam("Vector4", "capsule"); gf4->AddParam("Float", "yawDeg");
    gf4->SetReturnType("Vector4"); rtti->RegisterFunction(gf4);
    auto* gf5 = RED4ext::CGlobalFunction::Create("UTPPGroundTraceBegin", "UTPPGroundTraceBegin", &Native_GroundTraceBegin);
    gf5->flags = flags; rtti->RegisterFunction(gf5);
    auto* gf6 = RED4ext::CGlobalFunction::Create("UTPPGroundTraceEnd", "UTPPGroundTraceEnd", &Native_GroundTraceEnd);
    gf6->flags = flags; gf6->SetReturnType("Float"); rtti->RegisterFunction(gf6);
    auto* gf7 = RED4ext::CGlobalFunction::Create("UTPPFeedGround", "UTPPFeedGround", &Native_FeedGround);
    gf7->flags = flags; gf7->AddParam("Bool", "hitL"); gf7->AddParam("Vector4", "posL"); gf7->AddParam("Vector4", "normalL");
    gf7->AddParam("Bool", "hitR"); gf7->AddParam("Vector4", "posR"); gf7->AddParam("Vector4", "normalR");
    gf7->AddParam("Vector4", "capsule"); gf7->AddParam("Float", "yawDeg"); rtti->RegisterFunction(gf7);
    auto* gf8 = RED4ext::CGlobalFunction::Create("UTPPGroundRecorderOn", "UTPPGroundRecorderOn", &Native_GroundRecorderOn);
    gf8->flags = flags; gf8->SetReturnType("Bool"); rtti->RegisterFunction(gf8);
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
