// UTPP Phase 5N step 5 glue: TPP gate (heartbeat), kill files, natives for LootRange + HeadGuard + InteractRange.
// Natives are declared in scripts/utpp_loothead.reds. Game thread only (script natives + RED4ext Running update).

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>

#include <RED4ext/RED4ext.hpp>
#include <RED4ext/Scripting/Stack.hpp>

#include <mutex>
#include <string>

#include "../CommonUtil.hpp"
#include "../PathScrub.hpp"
#include "../render/HeadGuard.hpp"
#include "InteractRange.hpp"
#include "LootHead.hpp"
#include "LootHeadLog.hpp"
#include "LootRange.hpp"

namespace utpp::loothead
{
namespace
{
const RED4ext::v1::Sdk* g_sdk = nullptr;
RED4ext::v1::PluginHandle g_handle = nullptr;
std::mutex g_mutex; // natives and Tick may come from different engine threads in theory; never held across engine calls
                    // that could re-enter a native
loot::TppGate g_gate;
bool g_initDone = false;
bool g_posted = false;

double Now() { return util::QpcSeconds(); }

using util::PluginDir;

bool FileExists(const std::wstring& aDir, const wchar_t* aName)
{
    if (aDir.empty())
        return false;
    const std::wstring p = aDir + aName;
    return GetFileAttributesW(p.c_str()) != INVALID_FILE_ATTRIBUTES;
}

// ---------------------------------------------------------------------------------------------
// Natives (all static global functions).

template<typename T>
RED4ext::CGlobalFunction* Make(const char* aName, RED4ext::ScriptingFunction_t<T> aFn)
{
    auto* f = RED4ext::CGlobalFunction::Create(aName, aName, aFn);
    f->flags = {.isNative = true, .isStatic = true};
    return f;
}

void N_Heartbeat(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool owner = false;
    float dist = 0.0f;
    RED4ext::GetParameter(aFrame, &owner);
    RED4ext::GetParameter(aFrame, &dist);
    aFrame->code++;
    std::lock_guard lock(g_mutex);
    g_gate.Heartbeat(Now(), owner, dist);
}

void N_SetGate(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float stale = 0.5f, dist = 0.6f;
    RED4ext::GetParameter(aFrame, &stale);
    RED4ext::GetParameter(aFrame, &dist);
    aFrame->code++;
    loot::GateParams p;
    p.staleSec = stale;
    p.tppCameraDist = dist;
    bool ok = false;
    {
        std::lock_guard lock(g_mutex);
        ok = g_gate.SetParams(p);
    }
    LogF(0, "UTPPLootHeadSetGate(%.3f, %.3f) -> %d", static_cast<double>(stale), static_cast<double>(dist), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

void N_IsTpp(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    aFrame->code++;
    std::lock_guard lock(g_mutex);
    if (aOut)
        *aOut = g_gate.Tpp(Now());
}

// ---- LootRange
void N_LrStatus(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::CString* aOut, int64_t)
{
    aFrame->code++;
    std::string s;
    {
        std::lock_guard lock(g_mutex);
        const double now = Now();
        s = lootrange::Status(now, g_gate.Tpp(now));
    }
    if (aOut)
        *aOut = RED4ext::CString(s.c_str());
}

void N_LrSetEnabled(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool on = false;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    std::lock_guard lock(g_mutex);
    lootrange::SetEnabled(on);
}

void N_LrKill(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    aFrame->code++;
    std::lock_guard lock(g_mutex);
    lootrange::Kill();
}

void N_LrSetScale(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float scale = 1.0f;
    RED4ext::GetParameter(aFrame, &scale);
    aFrame->code++;
    loot::WindowParams w;
    bool ok = loot::ScaledWindow(scale, w);
    {
        std::lock_guard lock(g_mutex);
        ok = ok && lootrange::SetWindow(w);
    }
    LogF(0, "UTPPLootRangeSetScale(%.3f) -> %d", static_cast<double>(scale), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

void N_LrSetWindow(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    loot::WindowParams w;
    RED4ext::GetParameter(aFrame, &w.width);
    RED4ext::GetParameter(aFrame, &w.height);
    RED4ext::GetParameter(aFrame, &w.curvature);
    RED4ext::GetParameter(aFrame, &w.maxPriorityBoundsFactor);
    aFrame->code++;
    bool ok = false;
    {
        std::lock_guard lock(g_mutex);
        ok = lootrange::SetWindow(w);
    }
    LogF(0, "UTPPLootRangeSetWindow(%.3f, %.3f, %.3f, %.3f) -> %d", static_cast<double>(w.width),
         static_cast<double>(w.height), static_cast<double>(w.curvature), static_cast<double>(w.maxPriorityBoundsFactor),
         ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

void N_LrAddDescriptor(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    RED4ext::CString path;
    RED4ext::GetParameter(aFrame, &path);
    aFrame->code++;
    bool ok = false;
    {
        std::lock_guard lock(g_mutex);
        ok = lootrange::AddDescriptor(path.c_str());
    }
    LogF(0, "UTPPLootRangeAddDescriptor(%s) -> %d", path.c_str(), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

void N_LrActive(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    aFrame->code++;
    std::lock_guard lock(g_mutex);
    if (aOut)
        *aOut = lootrange::Active();
}

void N_LrReasons(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, int32_t* aOut, int64_t)
{
    aFrame->code++;
    std::lock_guard lock(g_mutex);
    const double now = Now();
    if (aOut)
        *aOut = static_cast<int32_t>(lootrange::Reasons(now, g_gate.Tpp(now)));
}

// ---- InteractRange (vehicles, devices / doors, NPC talk; see world/InteractRangeCore.hpp)
void N_IrStatus(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::CString* aOut, int64_t)
{
    aFrame->code++;
    std::string s;
    {
        std::lock_guard lock(g_mutex);
        const double now = Now();
        s = interactrange::Status(now, g_gate.Tpp(now));
    }
    if (aOut)
        *aOut = RED4ext::CString(s.c_str());
}

void N_IrSetEnabled(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool on = false;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    std::lock_guard lock(g_mutex);
    interactrange::SetEnabled(on);
}

void N_IrKill(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    aFrame->code++;
    std::lock_guard lock(g_mutex);
    interactrange::Kill();
}

void N_IrSetScale(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float scale = 1.0f;
    RED4ext::GetParameter(aFrame, &scale);
    aFrame->code++;
    bool ok = false;
    {
        std::lock_guard lock(g_mutex);
        ok = interactrange::SetScale(scale);
    }
    LogF(0, "UTPPInteractRangeSetScale(%.3f) -> %d", static_cast<double>(scale), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

void N_IrSetVehicleMode(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    int32_t mode = 1;
    RED4ext::GetParameter(aFrame, &mode);
    aFrame->code++;
    bool ok = false;
    {
        std::lock_guard lock(g_mutex);
        ok = interactrange::SetVehicleMode(mode);
    }
    LogF(0, "UTPPInteractRangeSetVehicleMode(%d) -> %d", mode, ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

void N_IrSetVehiclePriority(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float p = 0.5f;
    RED4ext::GetParameter(aFrame, &p);
    aFrame->code++;
    bool ok = false;
    {
        std::lock_guard lock(g_mutex);
        ok = interactrange::SetVehiclePriority(p);
    }
    LogF(0, "UTPPInteractRangeSetVehiclePriority(%.3f) -> %d", static_cast<double>(p), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

void N_IrActive(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    aFrame->code++;
    std::lock_guard lock(g_mutex);
    if (aOut)
        *aOut = interactrange::Active();
}

void N_IrReasons(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, int32_t* aOut, int64_t)
{
    aFrame->code++;
    std::lock_guard lock(g_mutex);
    if (aOut)
        *aOut = static_cast<int32_t>(interactrange::Reasons(g_gate.Tpp(Now())));
}

void N_IrDescriptorOf(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::CString* aOut, int64_t)
{
    RED4ext::Handle<RED4ext::IScriptable> comp;
    RED4ext::GetParameter(aFrame, &comp);
    aFrame->code++;
    std::string s;
    {
        std::lock_guard lock(g_mutex);
        s = interactrange::DescribeComponent(comp ? comp.instance : nullptr);
    }
    if (aOut)
        *aOut = RED4ext::CString(s.c_str());
}

// ---- HeadGuard
void N_HgStatus(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::CString* aOut, int64_t)
{
    aFrame->code++;
    std::string s;
    {
        std::lock_guard lock(g_mutex);
        s = headguard::Status(g_gate.Tpp(Now()));
    }
    if (aOut)
        *aOut = RED4ext::CString(s.c_str());
}

void N_HgSetEnabled(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool on = false;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    std::lock_guard lock(g_mutex);
    headguard::SetEnabled(on);
}

void N_HgKill(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    aFrame->code++;
    std::lock_guard lock(g_mutex);
    headguard::Kill();
}

void N_HgSetSite(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    RED4ext::CString pattern;
    int32_t offset = 0;
    hg::Site site;
    RED4ext::GetParameter(aFrame, &pattern);
    RED4ext::GetParameter(aFrame, &offset);
    RED4ext::GetParameter(aFrame, &site.expected);
    RED4ext::GetParameter(aFrame, &site.value);
    aFrame->code++;
    site.pattern = pattern.c_str();
    site.insnOffset = offset;
    std::string why;
    bool ok = false;
    {
        std::lock_guard lock(g_mutex);
        ok = headguard::SetSite(site, why);
    }
    if (!ok)
        LogF(1, "UTPPHeadGuardSetSite rejected: %s", why.c_str());
    if (aOut)
        *aOut = ok;
}

void N_HgSetValue(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float v = 0.0f;
    RED4ext::GetParameter(aFrame, &v);
    aFrame->code++;
    bool ok = false;
    {
        std::lock_guard lock(g_mutex);
        ok = headguard::SetValue(v);
    }
    if (aOut)
        *aOut = ok;
}

void N_HgActive(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    aFrame->code++;
    std::lock_guard lock(g_mutex);
    if (aOut)
        *aOut = headguard::Active();
}

void N_HgReasons(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, int32_t* aOut, int64_t)
{
    aFrame->code++;
    std::lock_guard lock(g_mutex);
    if (aOut)
        *aOut = static_cast<int32_t>(headguard::Reasons(g_gate.Tpp(Now())));
}

void RegisterNatives()
{
    auto* rtti = RED4ext::CRTTISystem::Get();
    if (!rtti)
        return;
    auto* f = Make("UTPPLootHeadHeartbeat", &N_Heartbeat);
    f->AddParam("Bool", "ownerSaysTpp");
    f->AddParam("Float", "cameraDistance");
    rtti->RegisterFunction(f);

    f = Make("UTPPLootHeadSetGate", &N_SetGate);
    f->AddParam("Float", "staleSeconds");
    f->AddParam("Float", "tppCameraDistance");
    f->SetReturnType("Bool");
    rtti->RegisterFunction(f);

    f = Make("UTPPLootHeadIsTpp", &N_IsTpp);
    f->SetReturnType("Bool");
    rtti->RegisterFunction(f);

    f = Make("UTPPLootRangeStatus", &N_LrStatus);
    f->SetReturnType("String");
    rtti->RegisterFunction(f);

    f = Make("UTPPLootRangeSetEnabled", &N_LrSetEnabled);
    f->AddParam("Bool", "on");
    rtti->RegisterFunction(f);

    f = Make("UTPPLootRangeKill", &N_LrKill);
    rtti->RegisterFunction(f);

    f = Make("UTPPLootRangeSetScale", &N_LrSetScale);
    f->AddParam("Float", "scale");
    f->SetReturnType("Bool");
    rtti->RegisterFunction(f);

    f = Make("UTPPLootRangeSetWindow", &N_LrSetWindow);
    f->AddParam("Float", "width");
    f->AddParam("Float", "height");
    f->AddParam("Float", "curvature");
    f->AddParam("Float", "maxPriorityBoundsFactor");
    f->SetReturnType("Bool");
    rtti->RegisterFunction(f);

    f = Make("UTPPLootRangeAddDescriptor", &N_LrAddDescriptor);
    f->AddParam("String", "path");
    f->SetReturnType("Bool");
    rtti->RegisterFunction(f);

    f = Make("UTPPLootRangeActive", &N_LrActive);
    f->SetReturnType("Bool");
    rtti->RegisterFunction(f);

    f = Make("UTPPLootRangeReasons", &N_LrReasons);
    f->SetReturnType("Int32");
    rtti->RegisterFunction(f);

    f = Make("UTPPInteractRangeStatus", &N_IrStatus);
    f->SetReturnType("String");
    rtti->RegisterFunction(f);

    f = Make("UTPPInteractRangeSetEnabled", &N_IrSetEnabled);
    f->AddParam("Bool", "on");
    rtti->RegisterFunction(f);

    f = Make("UTPPInteractRangeKill", &N_IrKill);
    rtti->RegisterFunction(f);

    f = Make("UTPPInteractRangeSetScale", &N_IrSetScale);
    f->AddParam("Float", "scale");
    f->SetReturnType("Bool");
    rtti->RegisterFunction(f);

    f = Make("UTPPInteractRangeSetVehicleMode", &N_IrSetVehicleMode);
    f->AddParam("Int32", "mode");
    f->SetReturnType("Bool");
    rtti->RegisterFunction(f);

    f = Make("UTPPInteractRangeSetVehiclePriority", &N_IrSetVehiclePriority);
    f->AddParam("Float", "priority");
    f->SetReturnType("Bool");
    rtti->RegisterFunction(f);

    f = Make("UTPPInteractRangeActive", &N_IrActive);
    f->SetReturnType("Bool");
    rtti->RegisterFunction(f);

    f = Make("UTPPInteractRangeReasons", &N_IrReasons);
    f->SetReturnType("Int32");
    rtti->RegisterFunction(f);

    f = Make("UTPPInteractDescriptorOf", &N_IrDescriptorOf);
    f->AddParam("handle:entIComponent", "component");
    f->SetReturnType("String");
    rtti->RegisterFunction(f);

    f = Make("UTPPHeadGuardStatus", &N_HgStatus);
    f->SetReturnType("String");
    rtti->RegisterFunction(f);

    f = Make("UTPPHeadGuardSetEnabled", &N_HgSetEnabled);
    f->AddParam("Bool", "on");
    rtti->RegisterFunction(f);

    f = Make("UTPPHeadGuardKill", &N_HgKill);
    rtti->RegisterFunction(f);

    f = Make("UTPPHeadGuardSetSite", &N_HgSetSite);
    f->AddParam("String", "pattern");
    f->AddParam("Int32", "instructionOffset");
    f->AddParam("Float", "expectedValue");
    f->AddParam("Float", "tppValue");
    f->SetReturnType("Bool");
    rtti->RegisterFunction(f);

    f = Make("UTPPHeadGuardSetValue", &N_HgSetValue);
    f->AddParam("Float", "tppValue");
    f->SetReturnType("Bool");
    rtti->RegisterFunction(f);

    f = Make("UTPPHeadGuardActive", &N_HgActive);
    f->SetReturnType("Bool");
    rtti->RegisterFunction(f);

    f = Make("UTPPHeadGuardReasons", &N_HgReasons);
    f->SetReturnType("Int32");
    rtti->RegisterFunction(f);
}
} // namespace

void LogLine(int aLevel, const char* aText)
{
    if (!g_sdk || !g_sdk->logger)
        return;
    const std::string text = pathscrub::ScrubLine(aText ? aText : ""); // no absolute path in a log line (same scrub as PluginLog.hpp)
    if (aLevel >= 2)
        g_sdk->logger->Error(g_handle, text.c_str());
    else if (aLevel == 1)
        g_sdk->logger->Warn(g_handle, text.c_str());
    else
        g_sdk->logger->Info(g_handle, text.c_str());
}

void Init(const RED4ext::v1::Sdk* aSdk, RED4ext::v1::PluginHandle aHandle)
{
    g_sdk = aSdk;
    g_handle = aHandle;
    const std::wstring dir = PluginDir();
    const bool lrKill = FileExists(dir, L"utpp_lootrange.disable");
    const bool hgKill = FileExists(dir, L"utpp_headguard.disable");
    const bool irKill = FileExists(dir, L"utpp_interactrange.disable");
    std::lock_guard lock(g_mutex);
    lootrange::Init(lrKill);
    headguard::Init(hgKill);
    interactrange::Init(irKill);
    g_initDone = true;
    LogF(0, "UTPP loothead init: lootrange kill=%d, headguard kill=%d, interactrange kill=%d", lrKill ? 1 : 0,
         hgKill ? 1 : 0, irKill ? 1 : 0);
}

void OnPostRegister(bool aGameValidated)
{
    RegisterNatives();
    std::lock_guard lock(g_mutex);
    if (!g_initDone)
    {
        LogF(1, "UTPP loothead: Init() not called - LootRange / HeadGuard stay off");
        return;
    }
    lootrange::Resolve(aGameValidated);
    headguard::Resolve(aGameValidated);
    interactrange::Resolve(aGameValidated);
    g_posted = true;
}

void Tick()
{
    std::unique_lock lock(g_mutex, std::try_to_lock);
    if (!lock.owns_lock() || !g_posted)
        return;
    const double now = Now();
    const bool tpp = g_gate.Tpp(now);
    lootrange::Tick(now, tpp);
    headguard::Tick(tpp);
    interactrange::Tick(now, tpp);
}

void Shutdown()
{
    std::lock_guard lock(g_mutex);
    lootrange::Shutdown();
    headguard::Shutdown();
    interactrange::Shutdown();
}
} // namespace utpp::loothead
