// UTPP: RED4ext plugin entry, fail-closed validation, FK hook, natives.
// 0.2.0 (Phase 4b): motion matching driver (MMDriver), database <dll dir>/utpp.utppmm loaded at plugin load
// (fail closed: MM unavailable, everything else works), bone map built at arm time, new MM natives.

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>

#include <RED4ext/RED4ext.hpp>
#include <RED4ext/Scripting/Natives/Quaternion.hpp>
#include <RED4ext/Scripting/Natives/Vector2.hpp>
#include <RED4ext/Scripting/Natives/Vector4.hpp>
#include <algorithm>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "Arming.hpp"
#include "BodyHelperRuntime.hpp"
#include "EarlyPoseRuntime.hpp"
#include "FkHook.hpp"
#include "MotionHost.hpp"
#include "MotionDriver.hpp" // UTPP-MOTION SEAM
#include "PoseHook.hpp"
#include "BodyTraceRuntime.hpp"
#include "PoseCaptureRuntime.hpp"
#include "DiagReportRuntime.hpp"
#include "State.hpp"
#include "utpp_version.h" // generated from the CMake project version (utpp_version.h.in)
#include "CommonUtil.hpp"
#include "PluginLog.hpp"
#include "input/GameLook.hpp"
#include "input/Locomotion.hpp"
#include "mm/AimOffset.hpp"
#include "mm/AimOffsetRuntime.hpp"
#include "mm/MeleeAimRuntime.hpp" // melee aim tilt (2026-11-09)
#include "mm/MeleeBaseRuntime.hpp" // melee chest base correction (2026-11-13)
#include "mm/MeleePoseRuntime.hpp" // melee full pose
#include "mm/HeadLookRuntime.hpp" // head look layer
#include "mm/VaultTorso.hpp" // vault torso limiter (2026-11-10)
#include "mm/BodyDeformerTrace.hpp"
#include "mm/CoreWriteRecord.hpp"
#include "mm/RearmProbe.hpp"
#include "render/ItemPlane.hpp"
#include "world/LootHead.hpp"
#include "world/TppRepGuard.hpp"
#include "ragdoll/RagdollSlotProbe.hpp"

namespace
{
using namespace utpp;

RED4ext::v1::PluginHandle g_handle = nullptr;
const RED4ext::v1::Sdk* g_sdk = nullptr;

// Cold state (game thread; guarded by g_coldMutex).
std::mutex g_coldMutex;
bool g_inert = true;
std::string g_reason = "not initialized";
bool g_hooked = false;
void* g_fkTarget = nullptr;
void* g_mappingTarget = nullptr;
bool g_mappingHooked = false;
std::string g_boneName = "Spine3";
ArmInfo g_armInfo;
int32_t g_lastArmCode = 0;
bool g_lastArmCodeValid = false;
std::wstring g_bodyTraceFolder;
std::vector<uint64_t> g_earlyHelperHashes;
std::filesystem::path g_bodyTracePath;
FILE* g_bodyTraceFile = nullptr; // g_coldMutex; never accessed in FK
std::filesystem::path g_bodyTracePostHelperPath;
FILE* g_bodyTracePostHelperFile = nullptr; // g_coldMutex; never accessed in FK
uint64_t g_bodyTraceFrames = 0;
uint64_t g_bodyTracePostHelperFrames = 0;
bool g_bodyTraceFailed = false;
bool g_bodyTracePostHelperFailed = false;

constexpr uint32_t kExpectedFileVerMS = (3u << 16) | 0u;     // 3.0
constexpr uint32_t kExpectedFileVerLS = (88u << 16) | 46235u; // 88.46235 (2.50)

constexpr uint8_t kFKPrologue[] = {0x40, 0x53, 0x48, 0x83, 0xEC, 0x40, 0x4C, 0x8B, 0x52, 0x10, 0x4C, 0x8B,
                                   0xC9, 0x44, 0x8B, 0x41, 0x0C, 0x33, 0xD2, 0x0F, 0x29, 0x74, 0x24, 0x30};
constexpr uint8_t kMappingPrologue[] = {0x48, 0x89, 0x5C, 0x24, 0x08, 0x4C, 0x89, 0x44, 0x24, 0x18, 0x55,
                                        0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48,
                                        0x83, 0xEC, 0x60, 0x48, 0x8B, 0x42, 0x38, 0x48, 0x8B, 0xF2};
constexpr uint8_t kFKCall[] = {0xE8, 0xBF, 0xE8, 0x3C, 0x00};

// ---------------------------------------------------------------------------------------------
// Logging (game thread / load only; never from the FK detour).
using pluginlog::LogInfo;
using pluginlog::LogWarn;
using pluginlog::LogError;

// ---------------------------------------------------------------------------------------------
// Validation.

// SEH-only: copies module bytes at aRva if inside SizeOfImage. Returns false on fault/out of range.
bool SehReadImage(const uint8_t* aBase, uint32_t aRva, uint8_t* aOut, uint32_t aLen)
{
    __try
    {
        auto* dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(aBase);
        if (dos->e_magic != IMAGE_DOS_SIGNATURE)
            return false;
        auto* nt = reinterpret_cast<const IMAGE_NT_HEADERS64*>(aBase + dos->e_lfanew);
        if (nt->Signature != IMAGE_NT_SIGNATURE)
            return false;
        const uint64_t size = nt->OptionalHeader.SizeOfImage;
        if (static_cast<uint64_t>(aRva) + aLen > size)
            return false;
        std::memcpy(aOut, aBase + aRva, aLen);
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

using util::HexBytes;

bool CheckBytes(const uint8_t* aBase, uint32_t aRva, const uint8_t* aExpected, uint32_t aLen, const char* aWhat,
                std::string& aReason)
{
    uint8_t actual[64] = {};
    if (aLen > sizeof(actual) || !SehReadImage(aBase, aRva, actual, aLen))
    {
        aReason = std::string(aWhat) + ": unreadable";
        return false;
    }
    if (std::memcmp(actual, aExpected, aLen) != 0)
    {
        aReason = std::string(aWhat) + ": byte mismatch, found [" + HexBytes(actual, aLen) + "]";
        return false;
    }
    return true;
}

bool CheckExeFileVersion(std::string& aReason)
{
    wchar_t path[MAX_PATH * 2] = {};
    const DWORD len = GetModuleFileNameW(nullptr, path, static_cast<DWORD>(std::size(path)));
    if (len == 0 || len >= std::size(path))
    {
        aReason = "exe file version: GetModuleFileNameW failed";
        return false;
    }
    DWORD dummy = 0;
    const DWORD size = GetFileVersionInfoSizeW(path, &dummy);
    if (size == 0)
    {
        aReason = "exe file version: no version resource";
        return false;
    }
    std::vector<uint8_t> data(size);
    if (!GetFileVersionInfoW(path, 0, size, data.data()))
    {
        aReason = "exe file version: GetFileVersionInfoW failed";
        return false;
    }
    VS_FIXEDFILEINFO* ffi = nullptr;
    UINT ffiLen = 0;
    if (!VerQueryValueW(data.data(), L"\\", reinterpret_cast<void**>(&ffi), &ffiLen) || !ffi ||
        ffiLen < sizeof(VS_FIXEDFILEINFO))
    {
        aReason = "exe file version: VerQueryValueW failed";
        return false;
    }
    if (ffi->dwFileVersionMS != kExpectedFileVerMS || ffi->dwFileVersionLS != kExpectedFileVerLS)
    {
        char buf[160];
        std::snprintf(buf, sizeof(buf), "exe file version %u.%u.%u.%u != 3.0.88.46235 (2.50)",
                      HIWORD(ffi->dwFileVersionMS), LOWORD(ffi->dwFileVersionMS), HIWORD(ffi->dwFileVersionLS),
                      LOWORD(ffi->dwFileVersionLS));
        aReason = buf;
        return false;
    }
    return true;
}

bool CheckRuntimeSemVer(std::string& aReason)
{
    if (!g_sdk || !g_sdk->runtime)
    {
        aReason = "sdk->runtime unavailable";
        return false;
    }
    const auto& v = *g_sdk->runtime;
    // RED4ext maps CDPR product versions as "2.50" -> 2.5.0 (see Api/v1/Runtime.hpp comment);
    // 2.50.0 is accepted as the literal alternative encoding of the same product version.
    const bool ok = v.major == 2 && ((v.minor == 5 && v.patch == 0) || (v.minor == 50 && v.patch == 0));
    if (!ok)
    {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "runtime product version %u.%u.%u is not 2.50", static_cast<unsigned>(v.major),
                      static_cast<unsigned>(v.minor), static_cast<unsigned>(v.patch));
        aReason = buf;
        return false;
    }
    return true;
}

bool ValidateGame(std::string& aReason)
{
    if (!CheckRuntimeSemVer(aReason) || !CheckExeFileVersion(aReason))
        return false;

    const auto* base = reinterpret_cast<const uint8_t*>(GetModuleHandleW(nullptr));
    if (!base)
    {
        aReason = "GetModuleHandle(nullptr) failed";
        return false;
    }
    if (!CheckBytes(base, kRvaFK, kFKPrologue, sizeof(kFKPrologue), "FK prologue @0x582B3C", aReason))
        return false;
    if (!CheckBytes(base, kRvaMapping, kMappingPrologue, sizeof(kMappingPrologue), "mapping prologue @0x1B3C08",
                    aReason))
        return false;
    if (!CheckBytes(base, kRvaFKCall, kFKCall, sizeof(kFKCall), "FK call @0x1B4278", aReason))
        return false;

    // Redundant but explicit: the rel32 call target must resolve to FK.
    int32_t rel = 0;
    std::memcpy(&rel, kFKCall + 1, sizeof(rel));
    const int64_t target = static_cast<int64_t>(kRvaFKCall) + 5 + rel;
    if (target != static_cast<int64_t>(kRvaFK))
    {
        aReason = "FK call target does not resolve to 0x582B3C";
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------------------------
// Arming (game thread).

bool BodyTraceFlushLocked()
{
    if (!mm::bodytrace::g_capture.Cancel() || !mm::bodytrace::g_postHelperCapture.Cancel())
        return false; // producer busy; retry on control path
    if (!g_bodyTraceFile && !g_bodyTracePostHelperFile) return true;
    mm::bodytrace::Frame frame;
    while (g_bodyTraceFile && mm::bodytrace::g_capture.Pop(frame))
    {
        ++g_bodyTraceFrames;
        for (uint32_t n = 0; n < mm::bodytrace::kMaxBones; ++n)
        {
            if (std::fprintf(g_bodyTraceFile, "%lld,%llu,%u,%llu,%llu,%u,%s,%d", frame.ticks,
                static_cast<unsigned long long>(frame.desc), frame.thread,
                static_cast<unsigned long long>(frame.rig), static_cast<unsigned long long>(frame.generation),
                frame.rigCount, mm::bodytrace::kNames[n].data(), frame.index[n]) < 0) g_bodyTraceFailed = true;
            for (uint32_t stage = 0; stage < 2; ++stage)
                for (uint32_t value = 0; value < 12; ++value)
                    if (std::fprintf(g_bodyTraceFile, ",%.9g", double(frame.stage[stage][n][value])) < 0)
                        g_bodyTraceFailed = true;
            if (std::fputc('\n', g_bodyTraceFile) == EOF) g_bodyTraceFailed = true;
        }
    }
    if (g_bodyTraceFile)
    {
        if (std::fprintf(g_bodyTraceFile, "# frames=%llu dropped=%llu failed=%d\n",
            static_cast<unsigned long long>(g_bodyTraceFrames),
            static_cast<unsigned long long>(mm::bodytrace::g_capture.Dropped()), g_bodyTraceFailed ? 1 : 0) < 0)
            g_bodyTraceFailed = true;
        if (std::fclose(g_bodyTraceFile)) g_bodyTraceFailed = true;
        g_bodyTraceFile = nullptr;
    }
    mm::bodytrace::PostHelperFrame post;
    while (g_bodyTracePostHelperFile && mm::bodytrace::g_postHelperCapture.Pop(post))
    {
        ++g_bodyTracePostHelperFrames;
        for (uint32_t n = 0; n < mm::bodytrace::kMaxBones; ++n)
        {
            if (std::fprintf(g_bodyTracePostHelperFile, "%lld,%llu,%u,%llu,%llu,%u,%llu,%llu,%lld,%lld,%s,%d",
                post.ticks, static_cast<unsigned long long>(post.desc), post.thread,
                static_cast<unsigned long long>(post.rig), static_cast<unsigned long long>(post.generation),
                post.rigCount, static_cast<unsigned long long>(post.receiptGeneration),
                static_cast<unsigned long long>(post.receiptCycle), post.receiptStartedTicks,
                post.receiptLastTicks, mm::bodytrace::kNames[n].data(), post.index[n]) < 0)
                g_bodyTracePostHelperFailed = true;
            for (uint32_t value = 0; value < 12; ++value)
                if (std::fprintf(g_bodyTracePostHelperFile, ",%.9g", double(post.local[n][value])) < 0)
                    g_bodyTracePostHelperFailed = true;
            if (std::fputc('\n', g_bodyTracePostHelperFile) == EOF) g_bodyTracePostHelperFailed = true;
        }
    }
    if (g_bodyTracePostHelperFile)
    {
        if (std::fprintf(g_bodyTracePostHelperFile, "# frames=%llu dropped=%llu failed=%d\n",
            static_cast<unsigned long long>(g_bodyTracePostHelperFrames),
            static_cast<unsigned long long>(mm::bodytrace::g_postHelperCapture.Dropped()),
            g_bodyTracePostHelperFailed ? 1 : 0) < 0) g_bodyTracePostHelperFailed = true;
        if (std::fclose(g_bodyTracePostHelperFile)) g_bodyTracePostHelperFailed = true;
        g_bodyTracePostHelperFile = nullptr;
    }
    return !g_bodyTraceFailed && !g_bodyTracePostHelperFailed;
}

// Map of the helper catalogue onto the rig of aInfo (by name hash). Control path only. Duplicate names reject the mapping.
bool FillBodyTraceMap(const ArmInfo& aInfo, const std::vector<uint64_t>& aHashes, mm::bodytrace::Map& aMap)
{
    aMap = mm::bodytrace::Map{};
    aMap.rig = reinterpret_cast<uint64_t>(aInfo.metaRig);
    aMap.rigCount = aInfo.boneCount;
    for (uint32_t n = 0; n < mm::bodytrace::kMaxBones; ++n)
    {
        const uint64_t hash = RED4ext::CName(mm::bodytrace::kNames[n].data()).hash;
        for (uint32_t i = 0; i < aHashes.size(); ++i) if (aHashes[i] == hash)
        {
            if (aMap.index[n] >= 0) return false;
            aMap.index[n] = static_cast<int32_t>(i);
        }
    }
    return true;
}

void BodyTraceContinuousLog(const char* aLine) { LogInfo("%s", aLine); }

// The post-helper stream (records after the helper commit, a second file per capture) is developer-only: it exists only when the helper
// recompute writes (APPLY) AND the marker file <diagnostic folder>\post-helper-trace.txt exists, like pose-text-csv.txt for the text CSV.
// Player-facing diagnostic logs therefore stay the same size with APPLY on. Control path only (file system access).
std::atomic<bool> g_postHelperStreamRequested{false}; // UTPPSetPostHelperStream: the same request as the marker file, for one session
bool PostHelperStreamWanted()
{
    if (!bodyhelperruntime::ApplyEnabled() || g_bodyTraceFolder.empty()) return false;
    if (g_postHelperStreamRequested.load(std::memory_order_relaxed)) return true;
    std::error_code ec;
    return std::filesystem::exists(std::filesystem::path(g_bodyTraceFolder) / L"post-helper-trace.txt", ec);
}

// Paths in log lines and results shown to scripts are relative to the game folder (no Windows user name, no install folder).
std::string RelPath(const std::filesystem::path& aPath) { return util::RelativeToGame(aPath.wstring()); }

// UTPPPoseCaptureStart: the compact capture publishes through the same path as the text capture.
bool PoseCaptureRunning() { return posecaptureruntime::Enabled(); }

// UTPPBodyPoseTraceContinuous: publishes the map of the current player rig to the writer thread (it starts a new file pair when the
// rig differs from the one it records). Called from ArmLocked (every (re)arm, rig/tracks change) and from the native itself.
void PublishBodyTraceContinuousLocked(const ArmInfo& aInfo, const std::vector<uint64_t>& aHashes, const std::vector<int16_t>& aParents)
{
    const bool text = bodytraceruntime::Enabled(), compact = PoseCaptureRunning();
    if ((!text && !compact) || aHashes.empty() || aHashes.size() != aInfo.boneCount) return;
    mm::bodytrace::Map map;
    if (!FillBodyTraceMap(aInfo, aHashes, map))
    {
        bodytraceruntime::ClearMap();
        posecaptureruntime::ClearMap();
        return;
    }
    const bool post = PostHelperStreamWanted();
    if (text) bodytraceruntime::PublishMap(map, post, aHashes, aParents);
    else posecaptureruntime::PublishMap(map, post, aHashes, aParents);
}

bool BodyTraceArmLocked(int32_t budget)
{
    if (bodytraceruntime::Enabled() || PoseCaptureRunning()) return false; // a continuous mode owns the captures
    if (budget <= 0 || budget > int32_t(mm::bodytrace::kMaxFrames) || g_inert || !g_hooked ||
        !g_hot.armed.load(std::memory_order_acquire) || !g_armInfo.metaRig ||
        g_bodyTraceFolder.empty() || g_bodyTraceFile || g_bodyTracePostHelperFile ||
        mm::bodytrace::g_capture.Active() || mm::bodytrace::g_postHelperCapture.Active()) return false;
    ArmInfo current;
    if (ResolvePlayer(RED4ext::CName(g_boneName.c_str()).hash, current) < 0 ||
        current.metaRig != g_armInfo.metaRig || current.trackA != g_armInfo.trackA ||
        current.trackB != g_armInfo.trackB || current.playerDesc != g_armInfo.playerDesc) return false;
    std::vector<uint64_t> hashes;
    std::vector<int16_t> parents;
    if (!ReadRigNamesAndParents(current.metaRig, hashes, parents) || hashes.empty() ||
        hashes.size() != current.boneCount || parents.size() != hashes.size()) return false;
    mm::bodytrace::Map map;
    if (!FillBodyTraceMap(current, hashes, map)) return false;
    LARGE_INTEGER ticks{}, hz{};
    QueryPerformanceCounter(&ticks); QueryPerformanceFrequency(&hz);
    g_bodyTracePath = std::filesystem::path(g_bodyTraceFolder) /
        (L"utpp-body-pose-" + std::to_wstring(ticks.QuadPart) + L".csv");
    const bool capturePostHelperApply = PostHelperStreamWanted();
    g_bodyTracePostHelperPath = capturePostHelperApply ? (g_bodyTracePath.parent_path() /
        (g_bodyTracePath.stem().wstring() + L"-post-helper.csv")) : std::filesystem::path{};
    FILE* file = nullptr;
    if (_wfopen_s(&file, g_bodyTracePath.c_str(), L"wx") || !file) return false;
    FILE* postFile = nullptr;
    if (capturePostHelperApply && (_wfopen_s(&postFile, g_bodyTracePostHelperPath.c_str(), L"wx") || !postFile))
    { std::fclose(file); return false; }
    SYSTEMTIME utc{}; GetSystemTime(&utc);
    bool ok = std::fprintf(file,
        "# qpc_hz=%lld arm_ticks=%lld utc=%04u-%02u-%02uT%02u:%02u:%02u.%03uZ max_frames=%d stages=before_mm,after_mm_and_aim scope=main_rig_helpers\n",
        hz.QuadPart, ticks.QuadPart, utc.wYear, utc.wMonth, utc.wDay, utc.wHour, utc.wMinute, utc.wSecond,
        utc.wMilliseconds, budget) >= 0;
    // Owned rig topology is required for offline skinning. Mesh joint nodes may
    // be flat global bind transforms and cannot substitute for these parents.
    for (uint32_t i = 0; i < hashes.size(); ++i)
        ok = std::fprintf(file, "# rig_bone index=%u hash=%016llX parent=%d\n", i,
            static_cast<unsigned long long>(hashes[i]), int(parents[i])) >= 0 && ok;
    ok = std::fprintf(file, "ticks,desc,thread,rig,generation,rigCount,name,gameIndex") >= 0 && ok;
    for (int stage = 0; stage < 2; ++stage) for (int value = 0; value < 12; ++value)
        ok = std::fprintf(file, ",s%d_%d", stage, value) >= 0 && ok;
    ok = std::fputc('\n', file) != EOF && ok;
    ok = std::fflush(file) == 0 && ok;
    SYSTEMTIME postUtc{}; GetSystemTime(&postUtc);
    bool postOk = !capturePostHelperApply || std::fprintf(postFile,
        "# qpc_hz=%lld arm_ticks=%lld utc=%04u-%02u-%02uT%02u:%02u:%02u.%03uZ max_frames=%d stage=post_helper_apply_after_commit scope=main_rig_helpers; correlate by ticks,desc,rig,thread with legacy file\n",
        hz.QuadPart, ticks.QuadPart, postUtc.wYear, postUtc.wMonth, postUtc.wDay, postUtc.wHour,
        postUtc.wMinute, postUtc.wSecond, postUtc.wMilliseconds, budget) >= 0;
    for (uint32_t i = 0; capturePostHelperApply && i < hashes.size(); ++i)
        postOk = std::fprintf(postFile, "# rig_bone index=%u hash=%016llX parent=%d\n", i,
            static_cast<unsigned long long>(hashes[i]), int(parents[i])) >= 0 && postOk;
    if (capturePostHelperApply) postOk = std::fprintf(postFile,
        "ticks,desc,thread,rig,generation,rigCount,receiptGeneration,receiptCycle,receiptStartedTicks,receiptLastTicks,name,gameIndex") >= 0 && postOk;
    for (int value = 0; capturePostHelperApply && value < 12; ++value)
        postOk = std::fprintf(postFile, ",local_%d", value) >= 0 && postOk;
    if (capturePostHelperApply)
        postOk = std::fputc('\n', postFile) != EOF && postOk && std::fflush(postFile) == 0;
    g_bodyTraceFile = file; g_bodyTracePostHelperFile = postFile;
    g_bodyTraceFrames = 0; g_bodyTracePostHelperFrames = 0;
    g_bodyTraceFailed = false; g_bodyTracePostHelperFailed = false;
    if (!ok || !postOk)
    { BodyTraceFlushLocked(); return false; }
    const bool legacyArmed = mm::bodytrace::g_capture.Arm(map, static_cast<uint32_t>(budget));
    const bool postArmed = legacyArmed && (!capturePostHelperApply ||
        mm::bodytrace::g_postHelperCapture.Arm(map, static_cast<uint32_t>(budget)));
    if (!legacyArmed || !postArmed)
    { BodyTraceFlushLocked(); return false; }
    LogInfo("UTPP body pose trace armed: frames=%d rig=0x%llX file=%s", budget,
        static_cast<unsigned long long>(map.rig), RelPath(g_bodyTracePath).c_str());
    if (capturePostHelperApply)
        LogInfo("UTPP post-helper APPLY trace armed: maxFrames=%d rig=0x%llX stage=post_helper_apply_after_commit file=%s",
            budget, static_cast<unsigned long long>(map.rig), RelPath(g_bodyTracePostHelperPath).c_str());
    return true;
}

std::string BodyTraceStatusLocked()
{
    return "bodyPoseTrace active=" + std::to_string(mm::bodytrace::g_capture.Active()) +
        " pendingFlush=" + std::to_string(g_bodyTraceFile != nullptr) +
        " written=" + std::to_string(g_bodyTraceFrames) + " dropped=" +
        std::to_string(mm::bodytrace::g_capture.Dropped()) + " failed=" +
        std::to_string(g_bodyTraceFailed) + " file=" + RelPath(g_bodyTracePath) +
        " postHelperWritten=" + std::to_string(g_bodyTracePostHelperFrames) +
        " postHelperDropped=" + std::to_string(mm::bodytrace::g_postHelperCapture.Dropped()) +
        " postHelperFailed=" + std::to_string(g_bodyTracePostHelperFailed) +
        " postHelperFile=" + RelPath(g_bodyTracePostHelperPath);
}

void Native_BodyTraceArm(RED4ext::IScriptable*, RED4ext::CStackFrame* frame, bool* out, int64_t)
{
    int32_t frames = 200; RED4ext::GetParameter(frame, &frames); frame->code++;
    std::lock_guard lock(g_coldMutex);
    const bool ok = BodyTraceArmLocked(frames);
    if (out) *out = ok;
}
void Native_BodyTraceFlush(RED4ext::IScriptable*, RED4ext::CStackFrame* frame, bool* out, int64_t)
{
    frame->code++;
    std::lock_guard lock(g_coldMutex);
    const bool ok = BodyTraceFlushLocked();
    LogInfo("UTPP body pose trace flush: %s", BodyTraceStatusLocked().c_str());
    if (out) *out = ok;
}
// UTPPBodyPoseTraceContinuous(on): continuous capture of ALL helper bones (BodyTraceRuntime.cpp / mm/BodyTraceContinuous.hpp). Idempotent:
// the flag survives session changes; the same value again changes nothing. Refused (false) while an explicit Arm session is running.
void Native_BodyTraceContinuous(RED4ext::IScriptable*, RED4ext::CStackFrame* frame, bool* out, int64_t)
{
    bool on = false; RED4ext::GetParameter(frame, &on); frame->code++;
    std::lock_guard lock(g_coldMutex);
    bool ok = true;
    if (on)
    {
        ok = !g_inert && !g_bodyTraceFile && !g_bodyTracePostHelperFile && bodytraceruntime::Enable(true);
        if (ok && g_hot.armed.load(std::memory_order_acquire) && g_armInfo.metaRig)
        {
            ArmInfo current;
            std::vector<uint64_t> hashes;
            std::vector<int16_t> parents;
            if (ResolvePlayer(RED4ext::CName(g_boneName.c_str()).hash, current) >= 0 && current.metaRig == g_armInfo.metaRig &&
                ReadRigNamesAndParents(current.metaRig, hashes, parents))
                PublishBodyTraceContinuousLocked(current, hashes, parents);
        }
    }
    else
        ok = bodytraceruntime::Enable(false);
    LogInfo("UTPPBodyPoseTraceContinuous(%d)", on ? 1 : 0);
    if (out) *out = ok;
}
// UTPPSetPostHelperStream(on): asks the captures started afterwards for the post-helper stream (the helper bones after the commit),
// like the marker file but for this game session only. Needs the helper recompute in APPLY mode; returns whether the stream will be written.
void Native_SetPostHelperStream(RED4ext::IScriptable*, RED4ext::CStackFrame* frame, bool* out, int64_t)
{
    bool on = false; RED4ext::GetParameter(frame, &on); frame->code++;
    g_postHelperStreamRequested.store(on, std::memory_order_relaxed);
    const bool wanted = PostHelperStreamWanted();
    LogInfo("UTPPSetPostHelperStream(%d) -> %d", on ? 1 : 0, wanted ? 1 : 0);
    if (out) *out = wanted;
}
// UTPPPoseCaptureStart(keepMinutes): compact capture of all helper bones (PoseCaptureRuntime.cpp). keepMinutes 0 = the whole session in one
// file per rig; > 0 = one-minute segments, only the last keepMinutes minutes stay on disk. Refused (false) while the text capture or an
// explicit Arm session owns the rings.
void Native_PoseCaptureStart(RED4ext::IScriptable*, RED4ext::CStackFrame* frame, bool* out, int64_t)
{
    int32_t keepMinutes = 0; RED4ext::GetParameter(frame, &keepMinutes); frame->code++;
    std::lock_guard lock(g_coldMutex);
    const bool ok = !g_inert && !g_bodyTraceFile && !g_bodyTracePostHelperFile && posecaptureruntime::Start(keepMinutes);
    if (ok && g_hot.armed.load(std::memory_order_acquire) && g_armInfo.metaRig)
    {
        ArmInfo current;
        std::vector<uint64_t> hashes;
        std::vector<int16_t> parents;
        if (ResolvePlayer(RED4ext::CName(g_boneName.c_str()).hash, current) >= 0 && current.metaRig == g_armInfo.metaRig &&
            ReadRigNamesAndParents(current.metaRig, hashes, parents))
            PublishBodyTraceContinuousLocked(current, hashes, parents);
    }
    LogInfo("UTPPPoseCaptureStart(%d) -> %d", keepMinutes, ok ? 1 : 0);
    if (out) *out = ok;
}
// UTPPPoseCaptureStop(): closes the capture and returns the files of the session (paths relative to the game folder, joined with '|').
void Native_PoseCaptureStop(RED4ext::IScriptable*, RED4ext::CStackFrame* frame, RED4ext::CString* out, int64_t)
{
    frame->code++;
    std::lock_guard lock(g_coldMutex);
    const std::string files = posecaptureruntime::Stop();
    LogInfo("UTPPPoseCaptureStop: %s", files.empty() ? "no files" : files.c_str());
    if (out) *out = RED4ext::CString(files.c_str());
}
void Native_PoseCaptureStatus(RED4ext::IScriptable*, RED4ext::CStackFrame* frame, RED4ext::CString* out, int64_t)
{
    frame->code++;
    std::lock_guard lock(g_coldMutex);
    if (out) *out = RED4ext::CString(posecaptureruntime::Status().c_str());
}
// "Cyberpunk 2077 runtime 2.5.0 (exe 3.0.88.46235)" for the report file; the parts the DLL cannot read are left out.
std::string GameVersionText()
{
    std::string text = "Cyberpunk 2077";
    if (g_sdk && g_sdk->runtime)
        text += " runtime " + std::to_string(g_sdk->runtime->major) + "." + std::to_string(g_sdk->runtime->minor) + "." + std::to_string(g_sdk->runtime->patch);
    wchar_t path[MAX_PATH * 2] = {};
    const DWORD len = GetModuleFileNameW(nullptr, path, static_cast<DWORD>(std::size(path)));
    DWORD dummy = 0;
    const DWORD size = len > 0 && len < std::size(path) ? GetFileVersionInfoSizeW(path, &dummy) : 0;
    if (size > 0)
    {
        std::vector<uint8_t> data(size);
        VS_FIXEDFILEINFO* ffi = nullptr;
        UINT ffiLen = 0;
        if (GetFileVersionInfoW(path, 0, size, data.data()) && VerQueryValueW(data.data(), L"\\", reinterpret_cast<void**>(&ffi), &ffiLen) && ffi &&
            ffiLen >= sizeof(VS_FIXEDFILEINFO))
            text += " (exe " + std::to_string(HIWORD(ffi->dwFileVersionMS)) + "." + std::to_string(LOWORD(ffi->dwFileVersionMS)) + "." +
                    std::to_string(HIWORD(ffi->dwFileVersionLS)) + "." + std::to_string(LOWORD(ffi->dwFileVersionLS)) + ")";
    }
    return text;
}
// UTPPDiagWriteReport(body): writes red4ext\logs\utpp-report-<time>.txt (mod and game version, the text from the script, the mod's files of the last
// 30 minutes with the instruction to zip and send them). Returns the path relative to the game folder, "" when nothing could be written.
void Native_DiagWriteReport(RED4ext::IScriptable*, RED4ext::CStackFrame* frame, RED4ext::CString* out, int64_t)
{
    RED4ext::CString body;
    RED4ext::GetParameter(frame, &body);
    frame->code++;
    const std::string text = body.c_str() ? body.c_str() : "";
    std::wstring folder;
    {
        std::lock_guard lock(g_coldMutex);
        folder = g_bodyTraceFolder;
    }
    const std::string rel = diagreportruntime::WriteReport(folder, UTPP_VERSION_STRING, GameVersionText(), text);
    LogInfo("UTPPDiagWriteReport: %s", rel.empty() ? "failed" : rel.c_str());
    if (out) *out = RED4ext::CString(rel.c_str());
}
void Native_BodyTraceStatus(RED4ext::IScriptable*, RED4ext::CStackFrame* frame, RED4ext::CString* out, int64_t)
{
    frame->code++;
    std::lock_guard lock(g_coldMutex);
    if (out) *out = RED4ext::CString(BodyTraceStatusLocked().c_str());
}

void Native_PoseCensus(RED4ext::IScriptable*, RED4ext::CStackFrame* frame, bool* out, int64_t)
{
    bool enabled = false; RED4ext::GetParameter(frame, &enabled); frame->code++;
    std::lock_guard lock(g_coldMutex);
    const bool ok = !enabled || (g_mappingHooked && !g_inert && g_hot.armed.load(std::memory_order_acquire));
    posediag::SetEnabled(enabled && ok);
    if (out) *out = ok && (!enabled || posediag::ReadCounters().enabled);
}

void Native_SetPoseWriteTpp(RED4ext::IScriptable*, RED4ext::CStackFrame* frame, bool* out, int64_t)
{
    bool value = false;
    RED4ext::GetParameter(frame, &value);
    frame->code++;
    // A false camera signal revokes immediately, independently of the clock
    // and independently of the locomotion/MM fade or delayed feed.
    if (!value || !g_hot.armed.load(std::memory_order_acquire))
        g_hot.perspectiveWrite.Reset();
    else
    {
        LARGE_INTEGER now{}, hz{};
        const bool clockOk = QueryPerformanceCounter(&now) && QueryPerformanceFrequency(&hz) && hz.QuadPart > 1;
        g_hot.perspectiveWriteMaxAge.store(clockOk ? hz.QuadPart / 2 : 0, std::memory_order_release);
        g_hot.perspectiveWrite.Publish(clockOk && g_hot.armed.load(std::memory_order_acquire), now.QuadPart);
    }
    if (out) *out = true;
}

bool WriteSemanticCensus(const std::filesystem::path& aPath, const posediag::Counters& aStats,
                         uint32_t& aRecords)
{
    aRecords = 0;
    if (!posediag::SemanticCount(aRecords)) return false; // explicit failure instead of a partial dump
    FILE* file = nullptr;
    if (_wfopen_s(&file, aPath.c_str(), L"wx") != 0 || !file) return false;
    FILETIME anchor{};
    anchor.dwLowDateTime = static_cast<DWORD>(aStats.utcFileTime);
    anchor.dwHighDateTime = static_cast<DWORD>(aStats.utcFileTime >> 32u);
    SYSTEMTIME utc{};
    const bool utcOk = FileTimeToSystemTime(&anchor, &utc) != 0;
    bool ok = std::fprintf(file,
        "# owned_hash_and_pair_copies_only=1 qpc_hz=%lld start_ticks=%lld utc_filetime=%llu utc_known=%d utc=%04u-%02u-%02uT%02u:%02u:%02u.%03uZ generation=%llu records=%u capacity=%u max_bones=%u max_pairs=%u invalid=%llu busy=%llu full=%llu stale=%llu\n",
        aStats.qpcHz, aStats.startTicks, aStats.utcFileTime, utcOk ? 1 : 0,
        utc.wYear, utc.wMonth, utc.wDay, utc.wHour, utc.wMinute, utc.wSecond, utc.wMilliseconds,
        aStats.semanticGeneration, aRecords, mm::semanticpose::kMaxSignatures,
        mm::semanticpose::kMaxBones, mm::semanticpose::kMaxPairs,
        aStats.semanticInvalid, aStats.semanticBusy, aStats.semanticFull, aStats.semanticStale) >= 0;
    ok = std::fprintf(file,
        "record,kind,index,nameHash,sourceIndex,destinationIndex,generation,ticks,thread,lod,sourceRig,destinationRig,graph,desc,bones,tag,sourceCount,destinationCount,pairCount,sourceNamesFingerprint,destinationNamesFingerprint,pairsFingerprint\n") >= 0 && ok;
    auto record = std::make_unique<mm::semanticpose::Record>();
    for (uint32_t r = 0; r < aRecords && ok; ++r)
    {
        if (!posediag::CopySemantic(r, *record)) { ok = false; break; }
        const auto writeRow = [&](const char* kind, uint32_t index, uint64_t hash, int32_t src, int32_t dst)
        {
            const auto& id = record->identity;
            return std::fprintf(file,
                "%u,%s,%u,%016llX,%d,%d,%llu,%lld,%u,%u,%llu,%llu,%llu,%llu,%llu,%llu,%u,%u,%u,%016llX,%016llX,%016llX\n",
                r, kind, index, hash, src, dst, id.generation, id.ticks, id.thread, id.lod,
                id.sourceRig, id.destinationRig, id.graph, id.desc, id.bones, id.tag,
                record->sourceCount, record->destinationCount, record->pairCount,
                record->sourceNamesFingerprint, record->destinationNamesFingerprint, record->pairsFingerprint) >= 0;
        };
        ok = writeRow("identity", 0, 0, -1, -1);
        for (uint32_t i = 0; i < record->sourceCount && ok; ++i)
            ok = writeRow("sourceName", i, record->sourceNames[i], static_cast<int32_t>(i), -1);
        for (uint32_t i = 0; i < record->destinationCount && ok; ++i)
            ok = writeRow("destinationName", i, record->destinationNames[i], -1, static_cast<int32_t>(i));
        for (uint32_t i = 0; i < record->pairCount && ok; ++i)
            ok = writeRow("pair", i, 0, static_cast<int32_t>(record->pairs[i * 2]),
                          static_cast<int32_t>(record->pairs[i * 2 + 1]));
    }
    return std::fclose(file) == 0 && ok;
}

void Native_PoseCensusDump(RED4ext::IScriptable*, RED4ext::CStackFrame* frame, RED4ext::CString* out, int64_t)
{
    frame->code++;
    std::lock_guard lock(g_coldMutex);
    posediag::SetEnabled(false);
    posediag::Event events[posediag::kCensusCapacity];
    const size_t count = posediag::CopyCensus(events, posediag::kCensusCapacity);
    std::sort(events, events + count, [](const auto& a, const auto& b) { return a.sequence < b.sequence; });
    LARGE_INTEGER now{}, hz{}; QueryPerformanceCounter(&now); QueryPerformanceFrequency(&hz);
    const auto path = std::filesystem::path(g_bodyTraceFolder) /
        (L"utpp-pose-census-" + std::to_wstring(now.QuadPart) + L".csv");
    const auto semanticPath = std::filesystem::path(g_bodyTraceFolder) /
        (L"utpp-pose-semantics-" + std::to_wstring(now.QuadPart) + L".csv");
    FILE* file = nullptr;
    bool ok = !g_bodyTraceFolder.empty() && _wfopen_s(&file, path.c_str(), L"wx") == 0 && file;
    if (ok)
    {
        const auto stats = posediag::ReadCounters();
        ok = std::fprintf(file, "# qpc_hz=%lld mappingCalls=%llu playerPasses=%llu rootPasses=%llu fkPasses=%llu drops=%llu faults=%llu faultCode=%u native_receipt_before_mod=1 equal_pair_mask_word_order=low_first\n",
            hz.QuadPart, stats.mappingCalls, stats.playerPasses, stats.rootPasses, stats.fkPasses,
            stats.contentionDrops, stats.readFaults, stats.faultCode) >= 0;
        ok = std::fprintf(file, "sequence,ticks,kind,thread,desc,bones,tag,rig,graph,source,sourceBones,count,lod,rootRig,graphHashKnown,sourceCountKnown,mappedBonePairsCount,mappedBonePairsFingerprint,mappedBonePairsKnown,sourceCount,nativeBytesValid,nativeBuffersOverlap,nativeCompared,nativeEqual,nativeEqualPairMask,cycleConfigured,cycleGeneration,cycleSerial,cycleObservation,cycleStatus,cycleRequired,cycleCovered,cycleEqual,cycleComplete\n") >= 0 && ok;
        for (size_t i = 0; i < count; ++i)
        {
            const auto& e = events[i];
            char equalMask[257]{};
            for (size_t word=0; word<16; ++word)
                std::snprintf(equalMask + word*16, 17, "%016llX", e.nativeBytes.equalPairs[word]);
            ok = std::fprintf(file, "%llu,%lld,%u,%u,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%u,%u,%d,%d,%d,%u,%llX,%d,%u,%d,%d,%u,%u,%s,%d,%llu,%llu,%u,%u,%u,%u,%u,%d\n",
                e.sequence, e.ticks, static_cast<uint32_t>(e.kind), e.thread, e.desc, e.bones, e.tag, e.rig,
                e.graph, e.source, e.sourceBones, e.count, e.lod, e.rootRig ? 1 : 0,
                e.graphHashKnown ? 1 : 0, e.sourceCountKnown ? 1 : 0, e.mappedBonePairsCount,
                e.mappedBonePairsFingerprint, e.mappedBonePairsKnown ? 1 : 0, e.sourceCount,
                e.nativeBytes.valid ? 1 : 0, e.nativeBytes.overlap ? 1 : 0,
                e.nativeBytes.compared, e.nativeBytes.equal, equalMask, e.cycleConfigured ? 1 : 0,
                e.cycle.generation, e.cycle.cycle, e.cycleObservation, static_cast<uint32_t>(e.cycle.status),
                e.cycle.required, e.cycle.covered, e.cycle.equal, e.cycle.complete ? 1 : 0) >= 0 && ok;
        }
        ok = std::fclose(file) == 0 && ok;
    }
    uint32_t semanticRecords = 0;
    const bool semanticOk = !g_bodyTraceFolder.empty() &&
        WriteSemanticCensus(semanticPath, posediag::ReadCounters(), semanticRecords);
    ok = semanticOk && ok;
    const auto helper = bodyhelperruntime::ReadCounters();
    LogInfo("UTPP skin helpers totals: evaluated=%llu rejected=%llu buildFailed=%llu proposed=%llu proposedRecords=%llu applied=%llu writes=%llu commitRejected=%llu jumpRejected=%llu",
        helper.evaluated, helper.rejected, helper.buildFailed, helper.proposed, helper.proposedRecords,
        helper.applied, helper.appliedRecords, helper.commitRejected, helper.jumpRejected);
    const std::string result = ok ? RelPath(path) : "pose census/semantic dump failed";
    LogInfo("UTPP pose census: events=%u %s", static_cast<uint32_t>(count), result.c_str());
    LogInfo("UTPP pose semantic discovery: records=%u ok=%d path=%s", semanticRecords,
            semanticOk ? 1 : 0, RelPath(semanticPath).c_str());
    if (out) *out = RED4ext::CString(result.c_str());
}

// Explicit local experiment, selected once at DLL load; never a live owner toggle.
// An invalid opt-in keeps Mapping owner but denies writes rather than falling back.
void LoadEarlyPoseExperiment(const std::wstring& dir)
{
    const auto path = std::filesystem::path(dir) / L"early-pose.txt";
    std::error_code ec;
    if (!std::filesystem::exists(path, ec)) return;

    std::ifstream input(path);
    std::string line;
    const bool hasHeader = bool(std::getline(input, line));
    if (hasHeader && line == "UTPP_EARLY_POSE_OFF") return;
    earlyruntime::SelectOwner(earlyruntime::Owner::Mapping);
    bool valid = hasHeader && (line == "UTPP_EARLY_POSE_HELPERS" || line == "UTPP_EARLY_POSE_V1"); // the earlier header is still read
    std::vector<uint64_t> hashes;
    while (valid && std::getline(input, line))
    {
        if (line.empty()) continue;
        if (line.size() != 16 || line.find_first_not_of("0123456789abcdefABCDEF") != std::string::npos ||
            hashes.size() == earlyruntime::kMaxRigBones) { valid = false; break; }
        uint64_t hash = 0;
        std::istringstream parser(line); parser >> std::hex >> hash;
        if (!parser || !hash || std::find(hashes.begin(), hashes.end(), hash) != hashes.end())
            { valid = false; break; }
        hashes.push_back(hash);
    }
    valid = valid && !hashes.empty() && !input.bad();
    if (valid) g_earlyHelperHashes = std::move(hashes);
    LogInfo("UTPP early pose experiment: owner=Mapping catalogValid=%d helpers=%u restartRequired=1",
            valid ? 1 : 0, static_cast<uint32_t>(g_earlyHelperHashes.size()));
}

void DisarmLocked()
{
    g_hot.armed.store(false, std::memory_order_release);
    utpp::ragdollslot::OnDisarm(); // ragdoll episode aborted, keep store invalidated, slot cache dropped
    g_hot.perspectiveWrite.Reset();
    earlyruntime::Disarm();
    BodyTraceFlushLocked();
    bodytraceruntime::ClearMap(); // continuous capture: no rig to record until the next arm
    posecaptureruntime::ClearMap();
    posediag::SetEnabled(false);
    g_hot.trackA.store(0, std::memory_order_relaxed);
    g_hot.trackB.store(0, std::memory_order_relaxed);
    g_hot.playerDesc.store(0, std::memory_order_relaxed);
    g_hot.playerRig.store(0, std::memory_order_relaxed);
    g_hot.playerBones.store(0, std::memory_order_relaxed);
    g_armInfo = ArmInfo{};
    motionhost::OnDisarmed();
    aimofs::OnDisarmed(); // 0.6.3
    utpp::meleeaim::OnDisarmed(); // melee aim tilt (2026-11-09)
    utpp::meleebase::OnDisarmed(); // melee chest base correction (2026-11-13)
    utpp::meleepose::OnDisarmed(); // melee full pose
    utpp::headlook::OnDisarmed(); // head look layer
}

int32_t ArmLocked(bool aLogAlways)
{
    const uint64_t boneHash = RED4ext::CName(g_boneName.c_str()).hash;
    ArmInfo info;
    const int32_t code = ResolvePlayer(boneHash, info);
    if (code < 0)
    {
        DisarmLocked();
    }
    else
    {
        if (g_armInfo.metaRig != info.metaRig || g_armInfo.trackA != info.trackA || g_armInfo.trackB != info.trackB)
            BodyTraceFlushLocked();
        // Publish: disarm while switching tracks so a half-updated pair is never matched.
        g_hot.armed.store(false, std::memory_order_release);
        g_hot.perspectiveWrite.Reset();
        earlyruntime::Disarm();
        g_hot.boneIndex.store(info.boneIndex >= 0 ? info.boneIndex : -1, std::memory_order_relaxed);
        g_hot.trackA.store(info.trackA, std::memory_order_relaxed);
        g_hot.trackB.store(info.trackB, std::memory_order_relaxed);
        g_hot.playerDesc.store(reinterpret_cast<uint64_t>(info.playerDesc), std::memory_order_relaxed);
        g_hot.playerRig.store(reinterpret_cast<uint64_t>(info.metaRig), std::memory_order_relaxed);
        g_hot.playerBones.store(info.boneCount, std::memory_order_relaxed);
        g_armInfo = info;
        // Motion matching bone map (by name) against this rig. Allocation is fine here (game thread, arm time).
        std::vector<uint64_t> hashes;
        std::vector<int16_t> parents;
        if (!ReadRigNamesAndParents(info.metaRig, hashes, parents))
            hashes.clear();
        // Required-helper catalog of the mapping-cycle receipt: the early-pose experiment list when it exists, otherwise the
        // owned helpers of the approved helper bundle (shipped default; empty = no receipt, so no helper writes).
        const std::vector<uint64_t> helperCatalog = g_earlyHelperHashes.empty() ? bodyhelperruntime::RequiredCatalog() : g_earlyHelperHashes;
        const bool cycleReady = posediag::PrepareCycleCensus(reinterpret_cast<uint64_t>(info.metaRig), hashes, helperCatalog);
        {
            // Ragdoll R1m probe: merged-rig indices of Hips / Head / LeftFoot (-1 when absent).
            auto find = [&hashes](const char* aName) -> int32_t {
                const uint64_t h = RED4ext::CName(aName).hash;
                for (size_t i = 0; i < hashes.size(); ++i)
                    if (hashes[i] == h)
                        return static_cast<int32_t>(i);
                return -1;
            };
            const int32_t hips = find("Hips"), head = find("Head"), foot = find("LeftFoot");
            RdFkTraceSetBones(hips, head, foot);
            // Ragdoll pelvis telemetry (UTPPRagdollPelvis / PelvisSpeed / FaceUp): Hips, Spine, LeftUpLeg, RightUpLeg of the merged rig.
            const int32_t spine = find("Spine"), lup = find("LeftUpLeg"), rup = find("RightUpLeg");
            utpp::ragdollslot::SetPelvisBones(hips, spine, lup, rup);
            LogInfo("UTPP ragdoll fk trace bones: Hips=%d Head=%d LeftFoot=%d Spine=%d LeftUpLeg=%d RightUpLeg=%d (rig bones %zu)", hips, head,
                    foot, spine, lup, rup, hashes.size());
        }
        if (bodyhelperruntime::Enabled())
        {
            const bool armed = bodyhelperruntime::PrepareArm(reinterpret_cast<uint64_t>(info.metaRig), parents, hashes, helperCatalog);
            LogInfo("UTPP skin helpers arm: %d cycleCensus=%d catalog=%u rigBones=%u (%s)", armed ? 1 : 0, cycleReady ? 1 : 0,
                static_cast<uint32_t>(helperCatalog.size()), static_cast<uint32_t>(hashes.size()), bodyhelperruntime::Status().c_str());
        }
        if (earlyruntime::CurrentOwner() == earlyruntime::Owner::Mapping)
        {
            const bool ready = g_mappingHooked && earlyruntime::PrepareArm(
                reinterpret_cast<uint64_t>(info.metaRig), info.boneCount, info.trackA, info.trackB,
                hashes, g_earlyHelperHashes);
            LogInfo("UTPP early pose arm: ready=%d rig=%llu bones=%u", ready ? 1 : 0,
                    reinterpret_cast<uint64_t>(info.metaRig), info.boneCount);
        }
        motionhost::OnArmed(info.metaRig, hashes, parents);
        aimofs::OnArmed(info.metaRig, hashes, parents); // 0.6.3 aim offset bones (fail closed)
        utpp::meleeaim::OnArmed(info.metaRig, hashes, parents); // melee aim tilt: Spine1..Neck (fail closed)
        utpp::meleebase::OnArmed(info.metaRig, hashes, parents); // melee chest base correction: Hips..Spine3 + key bones (fail closed)
        utpp::meleepose::OnArmed(info.metaRig, hashes, parents); // melee full pose: key bones + Hips, Spine, legs (fail closed)
        utpp::headlook::OnArmed(info.metaRig, hashes, parents); // head look: Spine3, Neck, Neck1, Head, eyes (fail closed)
        utpp::mm::vaulttorso::rt::OnArmed(info.metaRig, hashes, parents); // vault torso limiter: Hips, Spine..Spine3 (fail closed)
        g_hot.armed.store(true, std::memory_order_release);
        PublishBodyTraceContinuousLocked(info, hashes, parents); // continuous capture follows the rig (no-op unless on)
    }

    if (aLogAlways || !g_lastArmCodeValid || code != g_lastArmCode)
    {
        if (code < 0)
            LogInfo("UTPP arm: code %d (not armed)", code);
        else
            LogInfo("UTPP arm: code %d trackA=0x%llX trackB=0x%llX desc=0x%llX rig=0x%llX bone '%s' index %d of %u",
                    code, static_cast<unsigned long long>(info.trackA), static_cast<unsigned long long>(info.trackB),
                    reinterpret_cast<unsigned long long>(info.playerDesc),
                    reinterpret_cast<unsigned long long>(info.metaRig), g_boneName.c_str(), info.boneIndex,
                    info.boneCount);
    }
    g_lastArmCode = code;
    g_lastArmCodeValid = true;
    return code;
}

// 2.5 port proof (T2 2026-10-23): FK hook counters in the log while the camera's TPP write permission is fresh (the gate
// the FK / mapping writers use), so a game run shows whether the FK path matches the player's calls and writes the MM
// pose. Diagnostics stay quiet by default: only the first 3 TPP samples of a session (10 s apart) are logged, afterwards
// only the warning. A TPP gap > 15 s re-baselines without warning. Atomics only; no lock, no writes to game memory.
void LogFkCountersIfDue()
{
    static ULONGLONG s_lastLog = 0;
    static uint64_t s_lastMatched = 0, s_lastMmApplied = 0, s_lastTotal = 0;
    static uint32_t s_tppLogs = 0;
    const ULONGLONG nowMs = GetTickCount64();
    if (s_lastLog != 0 && nowMs - s_lastLog < 10000)
        return;
    LARGE_INTEGER now{};
    if (!QueryPerformanceCounter(&now) ||
        !g_hot.perspectiveWrite.Allows(now.QuadPart, g_hot.perspectiveWriteMaxAge.load(std::memory_order_acquire)))
        return;
    const bool rebaseline = s_lastLog != 0 && nowMs - s_lastLog > 15000; // FPP / menu gap: the deltas would span it
    s_lastLog = nowMs;
    const uint64_t total = g_hot.totalCalls.value.load(std::memory_order_relaxed);
    const uint64_t matched = g_hot.matchedCalls.value.load(std::memory_order_relaxed);
    const uint64_t mmApplied = g_hot.mmAppliedCalls.value.load(std::memory_order_relaxed);
    if (rebaseline)
    {
        s_lastTotal = total;
        s_lastMatched = matched;
        s_lastMmApplied = mmApplied;
        return;
    }
    const auto early = earlyruntime::ReadCounters();
    if (++s_tppLogs <= 3)
        LogInfo("UTPP FK counters (tpp): totalCalls=%llu (+%llu) matchedCalls=%llu (+%llu) appliedCalls=%llu "
            "mmAppliedCalls=%llu (+%llu) badIndexCalls=%llu fault=%d faultCode=0x%08X armed=%d matchMode=%d "
            "matchTag=%llu matchRig=%llu sameCountCalls=%llu lastDescCount=%u rigBones=%u mmBonesWritten=%llu "
            "mmRigMismatch=%llu mmStaleSkips=%llu mmTornReads=%llu mmActive=%d earlyOwner=%u earlyApplied=%llu",
            static_cast<unsigned long long>(total), static_cast<unsigned long long>(total - s_lastTotal),
            static_cast<unsigned long long>(matched), static_cast<unsigned long long>(matched - s_lastMatched),
            static_cast<unsigned long long>(g_hot.appliedCalls.value.load(std::memory_order_relaxed)),
            static_cast<unsigned long long>(mmApplied), static_cast<unsigned long long>(mmApplied - s_lastMmApplied),
            static_cast<unsigned long long>(g_hot.badIndexCalls.value.load(std::memory_order_relaxed)),
            g_hot.fault.load(std::memory_order_acquire) ? 1 : 0, g_hot.faultCode.load(std::memory_order_relaxed),
            g_hot.armed.load(std::memory_order_acquire) ? 1 : 0, g_hot.matchMode.load(std::memory_order_relaxed),
            static_cast<unsigned long long>(g_hot.matchTag.value.load(std::memory_order_relaxed)),
            static_cast<unsigned long long>(g_hot.matchRig.value.load(std::memory_order_relaxed)),
            static_cast<unsigned long long>(g_hot.sameCountCalls.value.load(std::memory_order_relaxed)),
            g_hot.lastMatchedCount.load(std::memory_order_relaxed), g_hot.playerBones.load(std::memory_order_relaxed),
            static_cast<unsigned long long>(g_hot.mmBonesWritten.value.load(std::memory_order_relaxed)),
            static_cast<unsigned long long>(g_hot.mmRigMismatch.value.load(std::memory_order_relaxed)),
            static_cast<unsigned long long>(g_hot.mmStaleSkips.value.load(std::memory_order_relaxed)),
            static_cast<unsigned long long>(g_hot.mmTornReads.value.load(std::memory_order_relaxed)),
            g_hot.mmActive.load(std::memory_order_acquire) ? 1 : 0,
            static_cast<uint32_t>(earlyruntime::CurrentOwner()), static_cast<unsigned long long>(early.applied));
    // Helper recompute counters (first TPP samples only, like the line above): applied > 0 proves that helper records were written;
    // rej[...] counts the hot-gate rejections by reason: 1 ragdoll, 2 gate (disabled/fault/owner/armed/MM off), 3 identity,
    // 4 perspective write window, 5 mapping-cycle receipt, 6 body.
    if (s_tppLogs <= 3 && bodyhelperruntime::Enabled())
    {
        const auto hc = bodyhelperruntime::ReadCounters();
        LogInfo("UTPP skin helpers counters (tpp): evaluated=%llu rejected=%llu buildFailed=%llu proposed=%llu applied=%llu writes=%llu "
                "commitRejected=%llu jumpRejected=%llu rej[ragdoll=%llu gate=%llu identity=%llu perspective=%llu receipt=%llu body=%llu] apply=%d",
                hc.evaluated, hc.rejected, hc.buildFailed, hc.proposed, hc.applied, hc.appliedRecords, hc.commitRejected, hc.jumpRejected,
                hc.rejectReason[1], hc.rejectReason[2], hc.rejectReason[3], hc.rejectReason[4], hc.rejectReason[5],
                hc.rejectReason[6], bodyhelperruntime::ApplyEnabled() ? 1 : 0);
    }
    // Second TPP sample onward (>= 10 s of TPP): an armed FK hook that sees calls but never the player's is a layout /
    // identity failure. Nothing is written in that case (writes need a match), so this only makes the silence visible.
    // Only meaningful while the FK path owns the pose; with the Mapping owner the pose is written elsewhere.
    if (s_tppLogs >= 2 && earlyruntime::CurrentOwner() == earlyruntime::Owner::FK &&
        g_hot.armed.load(std::memory_order_acquire) && total > s_lastTotal && matched == s_lastMatched)
        LogWarn("UTPP FK counters: no player FK call matched since the previous TPP sample (armed, %llu FK calls) - "
                "the MM pose does not reach the skeleton on this build",
                static_cast<unsigned long long>(total - s_lastTotal));
    s_lastTotal = total;
    s_lastMatched = matched;
    s_lastMmApplied = mmApplied;
}

// Reload contact correction funnel: one line (<= 1 per 2 s) whenever an eligible reload frame was seen since the last line.
void LogReloadContactCountersIfDue()
{
    if (!g_hot.timing.load(std::memory_order_relaxed))
        return; // diagnostic line: only while mm.trace is on
    static ULONGLONG s_last = 0;
    static uint64_t s_prevElig = 0;
    const uint64_t elig = g_hot.rcSnapEligible.value.load(std::memory_order_relaxed);
    if (elig == s_prevElig)
        return;
    const ULONGLONG nowMs = GetTickCount64();
    if (s_last != 0 && nowMs - s_last < 2000)
        return;
    s_last = nowMs;
    s_prevElig = elig;
    auto v = [](const auto& c) { return static_cast<unsigned long long>(c.value.load(std::memory_order_relaxed)); };
    LogInfo("UTPP reload-contacts funnel: snapEligible=%llu overlayOk=%llu ownerOk=%llu haveConfig=%llu captured=%llu "
            "finishWritten=%llu holdOnly=%llu lockLost=%llu",
            v(g_hot.rcSnapEligible), v(g_hot.rcOverlayOk), v(g_hot.rcOwnerOk), v(g_hot.rcHaveConfig), v(g_hot.rcCaptured),
            v(g_hot.rcFinishWritten), v(g_hot.rcHoldOnly), v(g_hot.rcLockLost));
}

// Skin helper coverage by core writer (BodyHelperRuntime.hpp Coverage): one line about once per second while player FK calls are
// counted, only while mm.trace is on (held on by "Diagnostic logs"). Cumulative counts (a reader takes differences of consecutive
// lines): calls = player FK calls; w[...] = calls in which the writer wrote core bones (none = no writer); c[...] = of those, calls
// with a helper commit; o.<writer>[...] = outcomes of that writer's calls other than a commit (only the non-zero ones); state[...] =
// the last published pose (source, weight, body), the motion driver's fallback and last hand-over reason, PSM LocomotionDetailed.
// tools/analysis/helper_coverage_report.py reads it.
void LogHelperCoverageIfDue()
{
    if (!g_hot.timing.load(std::memory_order_relaxed) || !bodyhelperruntime::Enabled())
        return;
    static ULONGLONG s_last = 0;
    static uint64_t s_prevCalls = 0;
    const ULONGLONG nowMs = GetTickCount64();
    if (s_last != 0 && nowMs - s_last < 1000)
        return;
    const auto cov = bodyhelperruntime::ReadCoverage();
    if (cov.calls == s_prevCalls)
        return;
    s_last = nowMs;
    s_prevCalls = cov.calls;
    namespace cw = mm::corewrite;
    namespace br = bodyhelperruntime;
    const char* rowNames[br::kWriterRows];
    for (size_t w = 0; w < cw::kWriterCount; ++w)
        rowNames[w] = cw::kWriterNames[w];
    rowNames[br::kNoWriter] = "none";
    std::string line = "UTPP skin helpers coverage: calls=" + std::to_string(cov.calls);
    std::string written = " w[", committed = " c[", outcomes;
    for (size_t w = 0; w < br::kWriterRows; ++w)
    {
        const auto& row = cov.byWriter[w];
        uint64_t total = 0;
        for (const uint64_t n : row)
            total += n;
        written += std::string(w ? " " : "") + rowNames[w] + "=" + std::to_string(total);
        committed += std::string(w ? " " : "") + rowNames[w] + "=" + std::to_string(row[size_t(br::Outcome::Committed)]);
        std::string other;
        for (size_t o = 1; o < size_t(br::Outcome::Count); ++o)
            if (row[o])
                other += std::string(other.empty() ? "" : " ") + br::kOutcomeNames[o] + "=" + std::to_string(row[o]);
        if (!other.empty())
            outcomes += std::string(" o.") + rowNames[w] + "[" + other + "]";
    }
    line += written + "]" + committed + "]" + outcomes;
    mm::integ::PoseSnapshot snap;
    char state[256];
    const bool haveSnap = g_mmPose.Read(snap);
    std::snprintf(state, sizeof state, " state[src=%s w=%.2f body=%d fallback=%s skip=%s detail=%d]",
                  !haveSnap ? "-" : (snap.source == mm::integ::kPoseSourceDriver ? "driver" : "host"), haveSnap ? snap.weight : 0.0f,
                  haveSnap ? snap.bodyType : -1, mm::adapter::FallbackName(utpp::motiondriver::Reason()),
                  utpp::motiondriver::LastSkipReason(), motionhost::LastPsmDetail());
    line += state;
    LogInfo("%s", line.c_str());
}

// RED4ext Running-state update: at most once per second, re-arm if not armed or the player changed.
bool OnRunningEnter(RED4ext::CGameApplication*)
{
    utpp::locomotion::OnRunning(); // Locomotion proof + attach once scripts are bound (idempotent)
    return true;
}
bool OnRunningUpdate(RED4ext::CGameApplication*)
{
    utpp::locomotion::OnRunning(); // fallback if Enter was missed; no-op after the first call
    motionhost::Tick(); // every frame: fades the MM weight out when the script feed stops
    utpp::ragdollslot::OnFrame(); // ragdoll episode clock (cheap while idle)
    utpp::loothead::Tick(); // Phase 5N step 5: LootRange / HeadGuard (reverts when the TPP heartbeat stops)
    LogFkCountersIfDue(); // 2.5 port proof: FK counters, <= 1 line / 10 s, TPP only
    LogReloadContactCountersIfDue();
    LogHelperCoverageIfDue();
    static ULONGLONG s_last = 0;
    const ULONGLONG now = GetTickCount64();
    // 2026-10-28 bike jump-out (mm/RearmProbe.hpp): the hooks still run but the player's calls stopped - the game replaced the animated
    // object; probe for it now instead of at the next 1 s poll (a probe that finds the same object changes nothing)
    static utpp::mm::RearmStallProbe s_stallProbe;
    const bool stallProbe = s_stallProbe.Update(g_hot.armed.load(std::memory_order_acquire),
                                                g_hot.totalCalls.value.load(std::memory_order_relaxed),
                                                g_hot.matchedCalls.value.load(std::memory_order_relaxed), static_cast<int64_t>(now));
    if (now - s_last < 1000 && !stallProbe)
        return false;
    if (now - s_last >= 1000)
        s_last = now;

    std::unique_lock lock(g_coldMutex, std::try_to_lock);
    if (!lock.owns_lock())
        return false;
    void* player = GetPlayerEntityRaw();
    const bool armed = g_hot.armed.load(std::memory_order_acquire);
    if (!player)
    {
        if (armed)
        {
            DisarmLocked();
            LogInfo("UTPP: player gone, disarmed");
            g_lastArmCodeValid = false;
        }
        return false;
    }
    utpp::ragdollslot::PublishPlayerEntity(player); // the player's EntityID for the ragdoll slot lookup (idempotent)
    // 0.1.1 in-game test: the first arm can catch a loading-time root object (294-bone rig) whose tracks
    // never match; the player pointer stays the same, so also re-arm when the root animated object or
    // its track buffers change.
    if (!armed || player != g_armInfo.entity)
    {
        ArmLocked(false);
        return false;
    }
    ArmInfo probe;
    const int32_t code = ResolvePlayer(RED4ext::CName(g_boneName.c_str()).hash, probe);
    if (code >= 0 && (probe.animObj != g_armInfo.animObj || probe.trackA != g_armInfo.trackA ||
                      probe.trackB != g_armInfo.trackB || probe.playerDesc != g_armInfo.playerDesc))
    {
        LogInfo("UTPP: player animated object/tracks changed, re-arming");
        ArmLocked(true);
    }
    return false;
}
bool OnRunningExit(RED4ext::CGameApplication*)
{
    return true;
}

// ---------------------------------------------------------------------------------------------
// Natives.

double TicksToUs(uint64_t aTicks)
{
    static const double freq = static_cast<double>(util::QpcFrequency());
    return static_cast<double>(aTicks) * 1.0e6 / freq;
}

std::string BuildStatus()
{
    std::lock_guard lock(g_coldMutex);
    char buf[2048];
    std::snprintf(
        buf, sizeof(buf),
        "UTPP " UTPP_VERSION_STRING " state=%s reason=\"%s\" hooked=%d armed=%d trackA=0x%llX trackB=0x%llX bone=%s boneIndex=%d "
        "rigBones=%u lastDescCount=%u enabled=%d angle=%.3f axis=%d totalCalls=%llu matchedCalls=%llu "
        "appliedCalls=%llu badIndexCalls=%llu fault=%d faultCode=0x%08X lastArm=%d | matchMode=%d "
        "playerDesc=0x%llX playerRig=0x%llX matchTag=%llu matchDesc=%llu matchRig=%llu sameCountCalls=%llu "
        "sampleDesc=0x%llX sampleRig=0x%llX sampleTag=0x%llX | ownUsTotal=%.1f ownNsMax=%.0f playerFkUsTotal=%.1f timing=%d",
        g_inert ? "inert" : "ready", g_reason.c_str(), g_hooked ? 1 : 0, g_hot.armed.load() ? 1 : 0,
        static_cast<unsigned long long>(g_hot.trackA.load()), static_cast<unsigned long long>(g_hot.trackB.load()),
        g_boneName.c_str(), g_hot.boneIndex.load(), g_armInfo.boneCount, g_hot.lastMatchedCount.load(),
        g_hot.enabled.load() ? 1 : 0, static_cast<double>(g_hot.angleDeg.load()), g_hot.axis.load(),
        static_cast<unsigned long long>(g_hot.totalCalls.value.load()),
        static_cast<unsigned long long>(g_hot.matchedCalls.value.load()),
        static_cast<unsigned long long>(g_hot.appliedCalls.value.load()),
        static_cast<unsigned long long>(g_hot.badIndexCalls.value.load()), g_hot.fault.load() ? 1 : 0,
        g_hot.faultCode.load(), g_lastArmCodeValid ? g_lastArmCode : 0, g_hot.matchMode.load(),
        static_cast<unsigned long long>(g_hot.playerDesc.load()), static_cast<unsigned long long>(g_hot.playerRig.load()),
        static_cast<unsigned long long>(g_hot.matchTag.value.load()),
        static_cast<unsigned long long>(g_hot.matchDesc.value.load()),
        static_cast<unsigned long long>(g_hot.matchRig.value.load()),
        static_cast<unsigned long long>(g_hot.sameCountCalls.value.load()),
        static_cast<unsigned long long>(g_hot.sampleDesc.load()), static_cast<unsigned long long>(g_hot.sampleRig.load()),
        static_cast<unsigned long long>(g_hot.sampleTag.load()), TicksToUs(g_hot.ownTicks.value.load()),
        TicksToUs(g_hot.ownTicksMax.load()) * 1000.0, TicksToUs(g_hot.playerFkTicks.value.load()),
        g_hot.timing.load(std::memory_order_relaxed) ? 1 : 0);
    const auto early = earlyruntime::ReadCounters();
    return std::string(buf) + motionhost::Status() + " earlyOwner=" +
        std::to_string(static_cast<uint32_t>(earlyruntime::CurrentOwner())) +
        " earlyApplied=" + std::to_string(early.applied) +
        " earlyIdentityReject=" + std::to_string(early.identityRejected) +
        " earlySemanticReject=" + std::to_string(early.semanticRejected) +
        " earlyNotReady=" + std::to_string(early.notReady) +
        " earlyPoseReject=" + std::to_string(early.poseRejected) +
        " earlyPerspectiveReject=" + std::to_string(early.perspectiveRejected);
}

void Native_GetStatus(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::CString* aOut, int64_t)
{
    aFrame->code++; // ParamEnd
    if (aOut)
    {
        const std::string s = BuildStatus();
        *aOut = RED4ext::CString(s.c_str());
    }
}

void Native_SetEnabled(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool value = false;
    RED4ext::GetParameter(aFrame, &value);
    aFrame->code++;
    if (value && g_hot.fault.load())
    {
        LogWarn("UTPPSetEnabled(true) ignored: fault latched for this session");
        return;
    }
    g_hot.enabled.store(value, std::memory_order_relaxed);
    LogInfo("UTPPSetEnabled(%d)", value ? 1 : 0);
}

void Native_SetTestRotation(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    float deg = 0.0f;
    RED4ext::GetParameter(aFrame, &deg);
    aFrame->code++;
    if (!std::isfinite(deg))
    {
        LogWarn("UTPPSetTestRotation: non-finite value ignored");
        return;
    }
    if (deg > 180.0f)
        deg = 180.0f;
    if (deg < -180.0f)
        deg = -180.0f;
    g_hot.angleDeg.store(deg, std::memory_order_relaxed);
    LogInfo("UTPPSetTestRotation(%.3f)", static_cast<double>(deg));
}

void Native_SetTestAxis(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    int32_t axis = 1;
    RED4ext::GetParameter(aFrame, &axis);
    aFrame->code++;
    if (axis < 0 || axis > 2)
    {
        LogWarn("UTPPSetTestAxis(%d) ignored: expected 0, 1 or 2", axis);
        return;
    }
    g_hot.axis.store(axis, std::memory_order_relaxed);
    LogInfo("UTPPSetTestAxis(%d)", axis);
}

void Native_SetMMSpeedMap(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    float gw = 0, gr = 0, dw = 0, dr = 0, rmin = 0, rmax = 0;
    RED4ext::GetParameter(aFrame, &gw);
    RED4ext::GetParameter(aFrame, &gr);
    RED4ext::GetParameter(aFrame, &dw);
    RED4ext::GetParameter(aFrame, &dr);
    RED4ext::GetParameter(aFrame, &rmin);
    RED4ext::GetParameter(aFrame, &rmax);
    aFrame->code++;
    motionhost::SetSpeedMap(gw, gr, dw, dr, rmin, rmax);
    LogInfo("UTPPSetMMSpeedMap(gameWalk=%.2f gameRun=%.2f dbWalk=%.2f dbRun=%.2f rate=[%.2f,%.2f])", gw, gr, dw, dr, rmin, rmax);
}

void Native_SetMMSpeedPoint(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    int32_t loco = 0, index = 0;
    float game = 0, db = 0;
    RED4ext::GetParameter(aFrame, &loco);
    RED4ext::GetParameter(aFrame, &index);
    RED4ext::GetParameter(aFrame, &game);
    RED4ext::GetParameter(aFrame, &db);
    aFrame->code++;
    motionhost::SetSpeedPoint(loco, index, game, db);
    LogInfo("UTPPSetMMSpeedPoint(loco=%d index=%d game=%.2f db=%.2f)", loco, index, game, db);
}

void Native_SetMMInputDriven(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool value = true;
    RED4ext::GetParameter(aFrame, &value);
    aFrame->code++;
    motionhost::SetInputDriven(value);
    LogInfo("UTPPSetMMInputDriven(%d)", value ? 1 : 0);
}

void Native_SetMMTransitionCost(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    float value = 0.15f;
    RED4ext::GetParameter(aFrame, &value);
    aFrame->code++;
    motionhost::SetTransitionCost(value);
    LogInfo("UTPPSetMMTransitionCost(%.3f)", value);
}

void Native_SetMMInputCameraRelative(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool value = false;
    RED4ext::GetParameter(aFrame, &value);
    aFrame->code++;
    motionhost::SetInputCameraRelative(value);
    LogInfo("UTPPSetMMInputCameraRelative(%d)", value ? 1 : 0);
}

void Native_SetMMOffsetRoot(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool value = true;
    RED4ext::GetParameter(aFrame, &value);
    aFrame->code++;
    motionhost::SetOffsetRoot(value);
    LogInfo("UTPPSetMMOffsetRoot(%d)", value ? 1 : 0);
}

void Native_SetMMOffsetRootParams(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float th = 0.25f, rh = 0.2f, maxD = 0.25f, maxYaw = 15.0f;
    RED4ext::GetParameter(aFrame, &th);
    RED4ext::GetParameter(aFrame, &rh);
    RED4ext::GetParameter(aFrame, &maxD);
    RED4ext::GetParameter(aFrame, &maxYaw);
    aFrame->code++;
    const bool ok = motionhost::SetOffsetRootParams(th, rh, maxD, maxYaw);
    LogInfo("UTPPSetMMOffsetRootParams(%.3f, %.3f, %.3f, %.1f) -> %d", static_cast<double>(th), static_cast<double>(rh),
            static_cast<double>(maxD), static_cast<double>(maxYaw), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

void Native_SetMMRootRate(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool value = true;
    RED4ext::GetParameter(aFrame, &value);
    aFrame->code++;
    motionhost::SetRootRate(value);
    LogInfo("UTPPSetMMRootRate(%d)", value ? 1 : 0);
}

void Native_SetMMRootRateParams(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float rmin = 0.5f, rmax = 1.5f, eps = 0.5f, hl = 0.08f;
    RED4ext::GetParameter(aFrame, &rmin);
    RED4ext::GetParameter(aFrame, &rmax);
    RED4ext::GetParameter(aFrame, &eps);
    RED4ext::GetParameter(aFrame, &hl);
    aFrame->code++;
    const bool ok = motionhost::SetRootRateParams(rmin, rmax, eps, hl);
    LogInfo("UTPPSetMMRootRateParams(%.2f, %.2f, %.2f, %.3f) -> %d", static_cast<double>(rmin), static_cast<double>(rmax),
            static_cast<double>(eps), static_cast<double>(hl), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

void Native_SetMMSlowWalk(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    bool on = true;
    float rmin = 0.25f;
    RED4ext::GetParameter(aFrame, &on);
    RED4ext::GetParameter(aFrame, &rmin);
    aFrame->code++;
    const bool ok = motionhost::SetSlowWalk(on, rmin);
    LogInfo("UTPPSetMMSlowWalk(%d, %.2f) -> %d", on ? 1 : 0, static_cast<double>(rmin), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

void Native_SetMMOffsetSaturation(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float sec = 0.3f, hl = 0.1f;
    RED4ext::GetParameter(aFrame, &sec);
    RED4ext::GetParameter(aFrame, &hl);
    aFrame->code++;
    const bool ok = motionhost::SetOffsetSaturation(sec, hl);
    LogInfo("UTPPSetMMOffsetSaturation(%.2f, %.3f) -> %d", static_cast<double>(sec), static_cast<double>(hl), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

void Native_SetMMExcludeLands(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool on = true;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    motionhost::SetExcludeLands(on);
    LogInfo("UTPPSetMMExcludeLands(%d)", on ? 1 : 0);
}

void Native_SetMMStartReface(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool on = true;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    motionhost::SetStartReface(on);
    LogInfo("UTPPSetMMStartReface(%d)", on ? 1 : 0);
}

// ---- Phase 5.3c (0.3.5) steering stability
void Native_SetMMSteerFollow(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool on = true;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    motionhost::SetSteerFollow(on);
    LogInfo("UTPPSetMMSteerFollow(%d)", on ? 1 : 0);
}

void Native_SetMMSteerParams(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float enter = 100.0f, exit = 30.0f, ft = 0.15f, maxRate = 360.0f;
    RED4ext::GetParameter(aFrame, &enter);
    RED4ext::GetParameter(aFrame, &exit);
    RED4ext::GetParameter(aFrame, &ft);
    RED4ext::GetParameter(aFrame, &maxRate);
    aFrame->code++;
    const bool ok = motionhost::SetSteerParams(enter, exit, ft, maxRate);
    LogInfo("UTPPSetMMSteerParams(%.1f, %.1f, %.3f, %.1f) -> %d", static_cast<double>(enter), static_cast<double>(exit),
            static_cast<double>(ft), static_cast<double>(maxRate), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

void Native_SetMMDwell(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    bool on = true;
    float minDwell = 0.3f, ratio = 0.5f;
    RED4ext::GetParameter(aFrame, &on);
    RED4ext::GetParameter(aFrame, &minDwell);
    RED4ext::GetParameter(aFrame, &ratio);
    aFrame->code++;
    const bool ok = motionhost::SetDwell(on, minDwell, ratio);
    LogInfo("UTPPSetMMDwell(%d, %.2f, %.2f) -> %d", on ? 1 : 0, static_cast<double>(minDwell), static_cast<double>(ratio),
            ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

void Native_SetMMCuration(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool on = true;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    motionhost::SetCuration(on);
    LogInfo("UTPPSetMMCuration(%d)", on ? 1 : 0);
}

void Native_SetMMSlowWalkCapsule(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool on = true;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    motionhost::SetSlowWalkCapsule(on);
    LogInfo("UTPPSetMMSlowWalkCapsule(%d)", on ? 1 : 0);
}

// ---- Phase 5.3c-2 (0.3.6)
void Native_SetMMCurationMoving(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool on = true;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    motionhost::SetCurationMoving(on);
    LogInfo("UTPPSetMMCurationMoving(%d)", on ? 1 : 0);
}

// ---- Phase 5.3e (0.3.9)
void Native_SetMMSlowWalkLoop(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool on = true;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    motionhost::SetSlowWalkLoop(on);
    LogInfo("UTPPSetMMSlowWalkLoop(%d)", on ? 1 : 0);
}

void Native_SetMMStanceSmooth(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float t = 0.08f;
    RED4ext::GetParameter(aFrame, &t);
    aFrame->code++;
    const bool ok = motionhost::SetStanceSmooth(t);
    LogInfo("UTPPSetMMStanceSmooth(%.3f) -> %d", static_cast<double>(t), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

// ---- 0.5.2: stance-change transition window (s; < 0 = always allowed = 0.5.1)
void Native_SetMMStanceTransitionWindow(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float t = 0.6f;
    RED4ext::GetParameter(aFrame, &t);
    aFrame->code++;
    const bool ok = motionhost::SetStanceTransitionWindow(t);
    LogInfo("UTPPSetMMStanceTransitionWindow(%.3f) -> %d", static_cast<double>(t), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

// ---- 0.5.3 locomotion kill switches (default on)
void Native_SetMMStickFilter(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool v = true;
    RED4ext::GetParameter(aFrame, &v);
    aFrame->code++;
    motionhost::SetStickFilter(v);
    LogInfo("UTPPSetMMStickFilter(%d)", v ? 1 : 0);
}

void Native_SetMMFamilyRate(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool v = true;
    RED4ext::GetParameter(aFrame, &v);
    aFrame->code++;
    motionhost::SetFamilyRate(v);
    LogInfo("UTPPSetMMFamilyRate(%d)", v ? 1 : 0);
}

void Native_SetMMStartRules(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool v = true;
    RED4ext::GetParameter(aFrame, &v);
    aFrame->code++;
    motionhost::SetStartRules(v);
    LogInfo("UTPPSetMMStartRules(%d)", v ? 1 : 0);
}

// ---- 0.5.4 speed bands kill switch (default on; false = 0.5.3)
void Native_SetMMSpeedBands(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool v = true;
    RED4ext::GetParameter(aFrame, &v);
    aFrame->code++;
    motionhost::SetSpeedBands(v);
    LogInfo("UTPPSetMMSpeedBands(%d)", v ? 1 : 0);
}

// ---- 0.5.6 kill switches (default on; off = 0.5.5)
void Native_SetMMGateDebounce(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float t = 0.06f;
    RED4ext::GetParameter(aFrame, &t);
    aFrame->code++;
    const bool ok = motionhost::SetGateDebounce(t);
    LogInfo("UTPPSetMMGateDebounce(%.3f) -> %d", static_cast<double>(t), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

void Native_SetMMCrouchRules(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool v = true;
    RED4ext::GetParameter(aFrame, &v);
    aFrame->code++;
    motionhost::SetCrouchRules(v);
    LogInfo("UTPPSetMMCrouchRules(%d)", v ? 1 : 0);
}

// ---- 0.5.7 kill switches (default on; off = 0.5.6)
template <void (*Setter)(bool)>
void Native_SetBool057(RED4ext::CStackFrame* aFrame, const char* aName)
{
    bool v = true;
    RED4ext::GetParameter(aFrame, &v);
    aFrame->code++;
    Setter(v);
    LogInfo("%s(%d)", aName, v ? 1 : 0);
}
void Native_SetMMCrouchDirTag(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&motionhost::SetCrouchDirTag>(aFrame, "UTPPSetMMCrouchDirTag");
}
void Native_SetMMStanceOnce(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&motionhost::SetStanceOnce>(aFrame, "UTPPSetMMStanceOnce");
}
void Native_SetMMTurnCommit(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&motionhost::SetTurnCommit>(aFrame, "UTPPSetMMTurnCommit");
}
void Native_SetMMStandIdle(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&motionhost::SetStandIdle>(aFrame, "UTPPSetMMStandIdle");
}
void Native_SetMMStartTurn(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&motionhost::SetStartTurn>(aFrame, "UTPPSetMMStartTurn");
}

// ---- 0.5.8 kill switches (default on; off = 0.5.7)
template <bool (*Setter)(float)>
void Native_SetFloat058(RED4ext::CStackFrame* aFrame, bool* aOut, float aDefault, const char* aName)
{
    float v = aDefault;
    RED4ext::GetParameter(aFrame, &v);
    aFrame->code++;
    const bool ok = Setter(v);
    LogInfo("%s(%.3f) -> %d", aName, static_cast<double>(v), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}
void Native_SetMMSpeedCap(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    Native_SetFloat058<&motionhost::SetSpeedCap>(aFrame, aOut, 1.25f, "UTPPSetMMSpeedCap");
}
void Native_SetMMCostCap(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    Native_SetFloat058<&motionhost::SetCostCap>(aFrame, aOut, 100.0f, "UTPPSetMMCostCap");
}
void Native_SetMMStanceCap(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&motionhost::SetStanceCap>(aFrame, "UTPPSetMMStanceCap");
}
void Native_SetMMTurnSettle(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&motionhost::SetTurnSettle>(aFrame, "UTPPSetMMTurnSettle");
}
void Native_SetMMLoopBands(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&motionhost::SetLoopBands>(aFrame, "UTPPSetMMLoopBands");
}
void Native_SetMMStartBind(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&motionhost::SetStartBind>(aFrame, "UTPPSetMMStartBind");
}

// ---- 0.5.9 (Phase 5.3; default on; off = 0.5.8)
void Native_SetMMCapsuleSync(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&motionhost::SetCapsuleSync>(aFrame, "UTPPSetMMCapsuleSync");
}
// b5 testorder repair (default on; off = 0.7.0 database swap)
void Native_SetMMSwapReset(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&motionhost::SetSwapReset>(aFrame, "UTPPSetMMSwapReset");
}
void Native_SetMMOffsetCatchup(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&motionhost::SetOffsetCatchup>(aFrame, "UTPPSetMMOffsetCatchup");
}
void Native_SetMMTrajCal(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&motionhost::SetTrajCal>(aFrame, "UTPPSetMMTrajCal");
}
void Native_SetMMOffsetCatchupParams(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float maxDistance = 0.30f, catchupTime = 0.6f, softDistance = 0.15f, halflife = 0.06f;
    RED4ext::GetParameter(aFrame, &maxDistance);
    RED4ext::GetParameter(aFrame, &catchupTime);
    RED4ext::GetParameter(aFrame, &softDistance);
    RED4ext::GetParameter(aFrame, &halflife);
    aFrame->code++;
    const bool ok = motionhost::SetOffsetCatchupParams(maxDistance, catchupTime, softDistance, halflife);
    LogInfo("UTPPSetMMOffsetCatchupParams(%.3f, %.3f, %.3f, %.3f) -> %d", static_cast<double>(maxDistance),
            static_cast<double>(catchupTime), static_cast<double>(softDistance), static_cast<double>(halflife), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}
void Native_SetMMFootSlipParams(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float start = 0.05f, halflife = 0.02f;
    RED4ext::GetParameter(aFrame, &start);
    RED4ext::GetParameter(aFrame, &halflife);
    aFrame->code++;
    const bool ok = motionhost::SetFootSlipParams(start, halflife);
    LogInfo("UTPPSetMMFootSlipParams(%.3f, %.3f) -> %d", static_cast<double>(start), static_cast<double>(halflife), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}
void Native_MMDecelTarget(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, float* aOut, int64_t)
{
    float base = 0.0f, mul = 1.0f;
    RED4ext::GetParameter(aFrame, &base);
    RED4ext::GetParameter(aFrame, &mul);
    aFrame->code++;
    const float r = motionhost::DecelTarget(base, mul);
    if (aOut)
        *aOut = r;
}

// ---- 0.6.0 (default on; off = 0.5.9)
void Native_SetMMMeleeGate(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&motionhost::SetMeleeGate>(aFrame, "UTPPSetMMMeleeGate");
}
void Native_SetMMMountCut(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&motionhost::SetMountCut>(aFrame, "UTPPSetMMMountCut");
}
void Native_SetMMStanceClipMinSpeed(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    Native_SetFloat058<&motionhost::SetStanceClipMinSpeed>(aFrame, aOut, 4.0f, "UTPPSetMMStanceClipMinSpeed");
}
void Native_MMMountEvent(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    aFrame->code++;
    motionhost::MountEvent();
}
// vehicle kind variant (2026-11-01): aIsCar = the mounted vehicle is a car (the mount transition may blend a large root yaw)
void Native_MMMountEventKind(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool isCar = false;
    RED4ext::GetParameter(aFrame, &isCar);
    aFrame->code++;
    motionhost::MountEvent(isCar);
}

// ---- 0.6.2 (default on; off = 0.6.1)
void Native_SetMMUpperMask(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&motionhost::SetUpperMask>(aFrame, "UTPPSetMMUpperMask");
}
void SetAllAimUpperFade(bool aOn)
{
    motionhost::SetAimUpperFade(aOn);
    motiondriver::SetAimUpperFade(aOn);
}
void Native_SetMMAimUpperFade(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&SetAllAimUpperFade>(aFrame, "UTPPSetMMAimUpperFade");
}
void SetAllWeaponUpper(bool aOn)
{
    motionhost::SetWeaponUpper(aOn);
    motiondriver::SetWeaponUpper(aOn);
}
void Native_SetMotionGrip(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&motiondriver::SetGripEnabled>(aFrame, "UTPPSetMotionGrip");
}
void Native_SetMotionShootTransfer(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&motiondriver::SetShootTransferEnabled>(aFrame, "UTPPSetMotionShootTransfer");
}
// magazine top-up gesture setting (CET mm.shinobiBonusReload, default off; motiondriver::SetMagTapEnabled - the DLL default is off too)
void Native_SetMotionMagTap(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&motiondriver::SetMagTapEnabled>(aFrame, "UTPPSetMotionMagTap");
}
void Native_SetMotionAimGameUpper(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&motiondriver::SetAimGameUpper>(aFrame, "UTPPSetMotionAimGameUpper");
}
void Native_SetMMWeaponUpper(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&SetAllWeaponUpper>(aFrame, "UTPPSetMMWeaponUpper");
}
void Native_SetMMHoldPose(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&motionhost::SetHoldPose>(aFrame, "UTPPSetMMHoldPose");
}
void Native_SetMMUpperMaskStates(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    int32_t bits = 7;
    RED4ext::GetParameter(aFrame, &bits);
    aFrame->code++;
    const bool ok = motionhost::SetUpperMaskStates(bits);
    if (ok)
        aimofs::SetMaskStates(bits); // 0.6.3: the aim offset needs mask state bit 2 (aim)
    LogInfo("UTPPSetMMUpperMaskStates(0x%X) -> %d", static_cast<unsigned>(bits), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

// ---- 0.6.3 aim offset (mm/AimOffset.hpp, mm/AimOffsetRuntime.hpp)
// Player entity world transform through RTTI (entEntity GetWorldPosition / GetWorldOrientation). Game thread only.
bool ReadPlayerTransform(RED4ext::Vector4& aPos, RED4ext::Quaternion& aRot)
{
    auto* player = static_cast<RED4ext::IScriptable*>(GetPlayerEntityRaw());
    if (!player)
        return false;
    RED4ext::CClass* cls = player->GetType();
    if (!cls)
        return false;
    static RED4ext::CClass* s_cls = nullptr;
    static RED4ext::CClassFunction* s_pos = nullptr;
    static RED4ext::CClassFunction* s_rot = nullptr;
    if (cls != s_cls)
    {
        s_pos = cls->GetFunction("GetWorldPosition");
        s_rot = cls->GetFunction("GetWorldOrientation");
        s_cls = cls;
    }
    if (!s_pos || !s_rot)
        return false;
    return RED4ext::ExecuteFunction(player, s_pos, &aPos) && RED4ext::ExecuteFunction(player, s_rot, &aRot);
}

uint64_t g_aimTargetFails = 0; // game thread (logged on the first failure and every 600th)

// Called each frame by CET while the utpp_camera aim state is on (enabled = false once when it ends). The target point is
// Q = camPos + norm(camFwd) * 60 m (aimofs::TargetDistance), converted to the player's model space here (fail closed: no player /
// transform -> target invalid, no offset). 2026-10-30 aim point feed (utpp_camera CM.AIM_POINT, with mm.aimAtCrosshair): CET sends
// org = P - fwd * 60 for the crosshair hit point P, so Q = org + fwd * 60 = P; otherwise org = the camera / eye ray origin as before.
// The CET side hard-codes the 60 m (CM.AIM_POINT.targetDist): pinned by the static_assert below.
static_assert(utpp::mm::aim::Params{}.targetDistance == 60.0f,
              "utpp_camera CM.AIM_POINT.targetDist (org = P - fwd * 60) assumes aimofs targetDistance 60 m");
void Native_SetAimTarget(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    RED4ext::Vector4 camPos{};
    RED4ext::Vector4 camFwd{};
    bool enabled = false;
    RED4ext::GetParameter(aFrame, &camPos);
    RED4ext::GetParameter(aFrame, &camFwd);
    RED4ext::GetParameter(aFrame, &enabled);
    aFrame->code++;
    if (!enabled)
    {
        aimofs::SetTarget(false, false, nullptr);
        utpp::motiondriver::SetAimTarget(false, false, nullptr); // UTPP-MOTION SEAM motionfix item 2
        return;
    }
    using utpp::mm::math::Quat;
    using utpp::mm::math::Vec3;
    RED4ext::Vector4 pos{};
    RED4ext::Quaternion rot{};
    Vec3 fwd;
    bool ok = ReadPlayerTransform(pos, rot) && utpp::mm::aim::NormalizeTo(Vec3{camFwd.X, camFwd.Y, camFwd.Z}, fwd);
    float model[3] = {0, 0, 0};
    if (ok)
    {
        const Vec3 p = Vec3{camPos.X, camPos.Y, camPos.Z} + fwd * aimofs::TargetDistance();
        const Vec3 m = utpp::mm::aim::WorldToModel(Vec3{pos.X, pos.Y, pos.Z}, Quat{rot.i, rot.j, rot.k, rot.r}, p);
        model[0] = m.x;
        model[1] = m.y;
        model[2] = m.z;
        ok = utpp::mm::math::IsFinite(m);
    }
    if (!ok && (g_aimTargetFails++ % 600) == 0)
        LogWarn("UTPPSetAimTarget: player transform / camera forward unavailable (%llu failures); aim offset off",
                static_cast<unsigned long long>(g_aimTargetFails));
    aimofs::SetTarget(true, ok, model);
    utpp::motiondriver::SetAimTarget(true, ok, model); // UTPP-MOTION SEAM motionfix item 2: v2 aim offset yaw / pitch
}

// The owned MM overlay also needs a fresh ray during hipfire. Keep this cache
// separate from UTPPSetAimTarget so the native FPP aim-mask solver keeps its
// gates.
void Native_SetCameraTarget(RED4ext::IScriptable *, RED4ext::CStackFrame *aFrame, void *, int64_t)
{
    RED4ext::Vector4 camPos{}, camFwd{};
    bool enabled = false;
    RED4ext::GetParameter(aFrame, &camPos);
    RED4ext::GetParameter(aFrame, &camFwd);
    RED4ext::GetParameter(aFrame, &enabled);
    aFrame->code++;
    if (!enabled)
    {
        utpp::motiondriver::SetCameraTarget(false, false, nullptr);
        utpp::meleeaim::SetTarget(false, false, nullptr); // melee aim tilt: no target -> the tilt fades to level
        return;
    }
    using utpp::mm::math::Quat;
    using utpp::mm::math::Vec3;
    RED4ext::Vector4 pos{};
    RED4ext::Quaternion rot{};
    Vec3 fwd;
    bool valid = ReadPlayerTransform(pos, rot) && utpp::mm::aim::NormalizeTo(Vec3{camFwd.X, camFwd.Y, camFwd.Z}, fwd);
    float point[3] = {0, 0, 0};
    if (valid)
    {
        const Vec3 world = Vec3{camPos.X, camPos.Y, camPos.Z} + fwd * aimofs::TargetDistance();
        const Vec3 actor =
            utpp::mm::aim::WorldToModel(Vec3{pos.X, pos.Y, pos.Z}, Quat{rot.i, rot.j, rot.k, rot.r}, world);
        point[0] = actor.x;
        point[1] = actor.y;
        point[2] = actor.z;
        valid = utpp::mm::math::IsFinite(actor);
    }
    utpp::motiondriver::SetCameraTarget(true, valid, point);
    // melee aim tilt (2026-11-09): the same camera / crosshair point (fed every third-person camera frame, also in melee)
    utpp::meleeaim::SetTarget(true, valid, point);
}

// ---- melee full body + melee aim tilt (2026-11-09; mm/MeleeFullBody.hpp, mm/MeleeAim.hpp)
// UTPPSetMeleeFullBody: script flag (utpp_melee.reds: UTPP third person on foot + a melee weapon type covered by the UTPP
// clips). The script repeats it every ~2 s, so only changes are logged.
void Native_SetMeleeFullBody(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool on = false;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    motionhost::SetMeleeFullBody(on);
    static int s_logged = -1; // game thread
    if (s_logged != (on ? 1 : 0))
    {
        s_logged = on ? 1 : 0;
        LogInfo("UTPPSetMeleeFullBody(%d)", s_logged);
    }
}
void Native_SetMeleeFullBodySetting(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&motionhost::SetMeleeFullBodySetting>(aFrame, "UTPPSetMeleeFullBodySetting");
}
// UTPPSetMeleeFullPoseSetting(on): user setting melee.fullPose (default on). Off = the chest correction alone.
void Native_SetMeleeFullPoseSetting(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&utpp::meleepose::SetEnabled>(aFrame, "UTPPSetMeleeFullPoseSetting");
}
// UTPPSetMeleePoseWeapon(variable): the anim variable of the drawn melee weapon (n"" = none); picks the clip section of the melee
// full pose tables. Repeated by the script; only changes are logged.
void Native_SetMeleePoseWeapon(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    RED4ext::CName variable;
    RED4ext::GetParameter(aFrame, &variable);
    aFrame->code++;
    utpp::meleepose::SetWeapon(variable.hash);
    static uint64_t s_logged = ~0ull; // game thread
    if (s_logged != variable.hash)
    {
        s_logged = variable.hash;
        LogInfo("UTPPSetMeleePoseWeapon(%s)", variable.ToString());
    }
}
void Native_SetMeleeAimParams(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float maxUp = 0.0f, maxDown = 0.0f, smoothHz = 10.0f;
    RED4ext::GetParameter(aFrame, &maxUp);
    RED4ext::GetParameter(aFrame, &maxDown);
    RED4ext::GetParameter(aFrame, &smoothHz);
    aFrame->code++;
    const bool ok = utpp::meleeaim::SetParams(maxUp, maxDown, smoothHz);
    LogInfo("UTPPSetMeleeAimParams(%.1f, %.1f, %.1f) -> %d", static_cast<double>(maxUp), static_cast<double>(maxDown),
            static_cast<double>(smoothHz), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}
// ---- melee round 3 (2026-11-09): full-body allowed states, guard, melee aim target (mm/MeleeFullBody.hpp, mm/MeleeAim.hpp)
// UTPPSetMeleeFullBodyStates(mask: Int32) -> Bool: PSM.MeleeWeapon states (bit = state 0..21) that may take the full body; the
// script repeats it every ~2 s, only changes are logged. False = bits outside 0..21 (kept).
void Native_SetMeleeFullBodyStates(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    int32_t mask = 0;
    RED4ext::GetParameter(aFrame, &mask);
    aFrame->code++;
    const bool ok = motionhost::SetMeleeFullBodyStates(mask);
    static int64_t s_logged = -1; // game thread
    if (s_logged != (ok ? int64_t(uint32_t(mask)) : -2))
    {
        s_logged = ok ? int64_t(uint32_t(mask)) : -2;
        LogInfo("UTPPSetMeleeFullBodyStates(0x%X) -> %d", static_cast<unsigned>(mask), ok ? 1 : 0);
    }
    if (aOut)
        *aOut = ok;
}
// UTPPSetMeleeGuard: script flag (utpp_melee.reds: a guard-capable melee weapon drawn, UTPP third person on foot); repeated
// every ~2 s, only changes are logged.
void Native_SetMeleeGuard(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool on = false;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    motionhost::SetMeleeGuard(on);
    static int s_logged = -1; // game thread
    if (s_logged != (on ? 1 : 0))
    {
        s_logged = on ? 1 : 0;
        LogInfo("UTPPSetMeleeGuard(%d)", s_logged);
    }
}
void Native_SetMeleeGuardSetting(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&motionhost::SetMeleeGuardSetting>(aFrame, "UTPPSetMeleeGuardSetting");
}
void Native_SetMeleeGuardFullSetting(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&motionhost::SetMeleeGuardFullSetting>(aFrame, "UTPPSetMeleeGuardFullSetting");
}
void Native_SetMeleeGuardAimTiltSetting(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&motionhost::SetMeleeGuardTiltSetting>(aFrame, "UTPPSetMeleeGuardAimTiltSetting");
}
// UTPPSetMeleeAimTarget(feet: Vector4, valid: Bool): the game's melee target (script, <= 20 Hz while a melee weapon is drawn),
// its world position (feet) -> model space -> meleeaim::SetLockTarget. Not logged (heartbeat).
void Native_SetMeleeAimTarget(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    RED4ext::Vector4 feet{};
    bool valid = false;
    RED4ext::GetParameter(aFrame, &feet);
    RED4ext::GetParameter(aFrame, &valid);
    aFrame->code++;
    float model[3] = {0, 0, 0};
    if (valid)
    {
        using utpp::mm::math::Quat;
        using utpp::mm::math::Vec3;
        RED4ext::Vector4 pos{};
        RED4ext::Quaternion rot{};
        valid = ReadPlayerTransform(pos, rot);
        if (valid)
        {
            const Vec3 m = utpp::mm::aim::WorldToModel(Vec3{pos.X, pos.Y, pos.Z}, Quat{rot.i, rot.j, rot.k, rot.r},
                                                       Vec3{feet.X, feet.Y, feet.Z});
            model[0] = m.x;
            model[1] = m.y;
            model[2] = m.z;
            valid = utpp::mm::math::IsFinite(m);
        }
    }
    utpp::meleeaim::SetLockTarget(valid, model);
}

// ---- head look layer (mm/HeadLook.hpp, mm/HeadLookRuntime.hpp)
// UTPPSetHeadLook(target: Vector4, weight: Float, mode: Int32, chest: Float, maxYawDeg: Float) -> Bool: the head look decision of
// utpp_core, sent every tick while it looks (the tick is the heartbeat: a target older than 0.25 s is no target) and once with
// weight 0 to release. target = world point; mode 0 camera / 1 face / 2 loot; chest 0..1 (headLook.chest); maxYawDeg (headLook.maxYaw).
// Returns true when the layer can act now (rig resolved, motion matching active, the motion pose owns the head: no reload / shot / aim
// mask closed the layer's gate within 0.5 s): the caller then skips the game's own look-at request.
// Only edges (on / off / mode) are logged.
void Native_SetHeadLook(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    RED4ext::Vector4 target{};
    float weight = 0.0f, chest = 0.5f, maxYaw = 100.0f;
    int32_t mode = 0;
    RED4ext::GetParameter(aFrame, &target);
    RED4ext::GetParameter(aFrame, &weight);
    RED4ext::GetParameter(aFrame, &mode);
    RED4ext::GetParameter(aFrame, &chest);
    RED4ext::GetParameter(aFrame, &maxYaw);
    aFrame->code++;
    using utpp::mm::math::Quat;
    using utpp::mm::math::Vec3;
    const bool on = weight > 0.0f;
    float model[3] = {0, 0, 0};
    bool valid = false;
    if (on)
    {
        RED4ext::Vector4 pos{};
        RED4ext::Quaternion rot{};
        valid = ReadPlayerTransform(pos, rot);
        if (valid)
        {
            const Vec3 m = utpp::mm::aim::WorldToModel(Vec3{pos.X, pos.Y, pos.Z}, Quat{rot.i, rot.j, rot.k, rot.r},
                                                       Vec3{target.X, target.Y, target.Z});
            model[0] = m.x;
            model[1] = m.y;
            model[2] = m.z;
            valid = utpp::mm::math::IsFinite(m);
        }
    }
    const bool accepted = utpp::headlook::SetTarget(on, valid, model, weight, mode, chest, maxYaw);
    static int s_logged = -1; // game thread: -1 never, 0 off, 1 + mode on
    const int state = !accepted ? -2 : (on ? 1 + mode : 0);
    if (s_logged != state)
    {
        s_logged = state;
        LogInfo("UTPPSetHeadLook(%s mode=%d chest=%.2f maxYaw=%.0f valid=%d) -> %d", on ? "on" : "off", mode,
                static_cast<double>(chest), static_cast<double>(maxYaw), valid ? 1 : 0, accepted ? 1 : 0);
    }
    if (aOut)
        *aOut = accepted && utpp::headlook::CanAct() && g_hot.mmActive.load(std::memory_order_acquire);
}

void Native_HeadLookStatus(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::CString* aOut, int64_t)
{
    aFrame->code++;
    if (aOut)
        *aOut = RED4ext::CString(utpp::headlook::Status().c_str());
}

void Native_MeleeStatus(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::CString* aOut, int64_t)
{
    aFrame->code++;
    if (aOut)
        *aOut = RED4ext::CString((motionhost::MeleeFullBodyStatus() + " | " + utpp::meleeaim::Status() + " | " + utpp::meleebase::Status() + " | " + utpp::meleepose::Status()).c_str());
}

void Native_SetAimOffsetParams(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float maxYaw = 30.0f, maxPitch = 25.0f, smoothHz = 12.0f;
    RED4ext::GetParameter(aFrame, &maxYaw);
    RED4ext::GetParameter(aFrame, &maxPitch);
    RED4ext::GetParameter(aFrame, &smoothHz);
    aFrame->code++;
    const bool ok = aimofs::SetParams(maxYaw, maxPitch, smoothHz);
    LogInfo("UTPPSetAimOffsetParams(%.1f, %.1f, %.1f) -> %d", static_cast<double>(maxYaw), static_cast<double>(maxPitch),
            static_cast<double>(smoothHz), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

void Native_SetAimOffsetAxis(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    int32_t axis = utpp::mm::aim::kDefaultBarrelAxis;
    RED4ext::GetParameter(aFrame, &axis);
    aFrame->code++;
    const bool ok = aimofs::SetAxis(axis);
    if (ok)
        motiondriver::SetBarrelAxis(axis); // aim pitch plan 2026-10-30: the same WeaponRight barrel axis for the v2 barrel loop
    LogInfo("UTPPSetAimOffsetAxis(%d) -> %d", axis, ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

void Native_AimOffsetStatus(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::CString* aOut, int64_t)
{
    aFrame->code++;
    if (aOut)
        *aOut = RED4ext::CString(aimofs::Status().c_str());
}

// ---- 0.6.4 turn rate natives (motionhost::SetTurnRate / SetTurnRateParams / TurnRateStatus)
void Native_SetMMTurnRate(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&motionhost::SetTurnRate>(aFrame, "UTPPSetMMTurnRate");
}
void Native_SetMMTurnRateParams(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float mn = 0.9f, mx = 1.4f;
    RED4ext::GetParameter(aFrame, &mn);
    RED4ext::GetParameter(aFrame, &mx);
    aFrame->code++;
    const bool ok = motionhost::SetTurnRateParams(mn, mx);
    LogInfo("UTPPSetMMTurnRateParams(%.3f, %.3f) -> %d", static_cast<double>(mn), static_cast<double>(mx), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}
void Native_MMTurnRateStatus(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::CString* aOut, int64_t)
{
    aFrame->code++;
    const std::string s = motionhost::TurnRateStatus();
    LogInfo("UTPPMMTurnRateStatus: %s", s.c_str());
    if (aOut)
        *aOut = RED4ext::CString(s.c_str());
}

// ---- loco P0 derived class data recompute (motionhost::SetDerivedRecompute; default on, off = 0.6.5b order)
void Native_SetMMDerivedRecompute(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&motionhost::SetDerivedRecompute>(aFrame, "UTPPSetMMDerivedRecompute");
}

// ---- loco C0 feet diagnostics / C1a crouch idle curation (motionhost::SetFeetDiag / SetCrouchIdleExclude / SetCrouchExcludeTags;
// default on, off = the loco P0 build)
void Native_SetMMFeetDiag(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&motionhost::SetFeetDiag>(aFrame, "UTPPSetMMFeetDiag");
}
void Native_SetMMCrouchIdleExclude(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&motionhost::SetCrouchIdleExclude>(aFrame, "UTPPSetMMCrouchIdleExclude");
}
void Native_SetMMCrouchExcludeTags(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    RED4ext::CString tags;
    RED4ext::GetParameter(aFrame, &tags);
    aFrame->code++;
    const char* str = tags.c_str();
    const bool ok = str && std::strlen(str) <= 512 && motionhost::SetCrouchExcludeTags(str);
    LogInfo("UTPPSetMMCrouchExcludeTags('%.512s') -> %d", str ? str : "", ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

// ---- stage standstill idle (motionhost::SetIdleHold; default on, off = the batch 1 build)
void Native_SetMMIdleHold(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&motionhost::SetIdleHold>(aFrame, "UTPPSetMMIdleHold");
}

// ---- loco C1b plant / replant (motionhost::SetFootReplant / SetFootReplantParams; default crouch, 0 = the loco C1a build)
void Native_SetMMFootReplant(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    int32_t mask = 2;
    RED4ext::GetParameter(aFrame, &mask);
    aFrame->code++;
    const bool ok = motionhost::SetFootReplant(mask);
    LogInfo("UTPPSetMMFootReplant(%d) -> %d", static_cast<int>(mask), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}
void Native_SetMMFootReplantParams(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float radius = 0.07f, stopsRadius = 0.10f, ratio = 0.2f, stopsRatio = 0.5f, stepSpeed = 1.0f, stopsStepSpeed = 2.5f, lift = 0.03f;
    RED4ext::GetParameter(aFrame, &radius);
    RED4ext::GetParameter(aFrame, &stopsRadius);
    RED4ext::GetParameter(aFrame, &ratio);
    RED4ext::GetParameter(aFrame, &stopsRatio);
    RED4ext::GetParameter(aFrame, &stepSpeed);
    RED4ext::GetParameter(aFrame, &stopsStepSpeed);
    RED4ext::GetParameter(aFrame, &lift);
    aFrame->code++;
    const bool ok = motionhost::SetFootReplantParams(radius, stopsRadius, ratio, stopsRatio, stepSpeed, stopsStepSpeed, lift);
    LogInfo("UTPPSetMMFootReplantParams(%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f) -> %d", static_cast<double>(radius),
            static_cast<double>(stopsRadius), static_cast<double>(ratio), static_cast<double>(stopsRatio), static_cast<double>(stepSpeed),
            static_cast<double>(stopsStepSpeed), static_cast<double>(lift), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

// ---- batch 2 knee fix (motionhost::SetKneeFix; default 7 since batch 5 (bit 4 planted swivel hold), 0 = the batch 1b leg pipeline)
void Native_SetMMKneeFix(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    int32_t mask = 7;
    RED4ext::GetParameter(aFrame, &mask);
    aFrame->code++;
    const bool ok = motionhost::SetKneeFix(mask);
    LogInfo("UTPPSetMMKneeFix(%d) -> %d", static_cast<int>(mask), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

// ---- opt-in head lead (motiondriver::SetHeadLead / SetHeadLeadParams; CET mm.headLead, default off)
void Native_SetMMHeadLead(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    bool on = false;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    const bool ok = motiondriver::SetHeadLead(on);
    LogInfo("UTPPSetMMHeadLead(%d) -> %d", on ? 1 : 0, ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}
void Native_SetMMHeadLeadParams(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float leadDeg = 14.0f, maxDeg = 22.0f;
    RED4ext::GetParameter(aFrame, &leadDeg);
    RED4ext::GetParameter(aFrame, &maxDeg);
    aFrame->code++;
    const bool ok = motiondriver::SetHeadLeadParams(leadDeg, maxDeg);
    LogInfo("UTPPSetMMHeadLeadParams(%.2f, %.2f) -> %d", static_cast<double>(leadDeg), static_cast<double>(maxDeg), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

// ---- feminine shoulder narrowing (motiondriver::SetFeminineShoulderCm): cm per side, default 2.5. CET mm.shoulderWidth (2026-11-09,
// Mod Settings "Shoulder width (female V)", -5..5 cm) sends 2.5 - width / 2 = 0..5; the accepted range is the driver's
// (motionpose::kFeminineMaxAmount * 100, femshoulder::kMaxAmountCm = 5 is the setting's end)
void Native_SetMotionShoulderNarrow(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float cm = 0.0f;
    RED4ext::GetParameter(aFrame, &cm);
    aFrame->code++;
    const bool ok = motiondriver::SetFeminineShoulderCm(cm);
    LogInfo("UTPPSetMotionShoulderNarrow(%.2f) -> %d", static_cast<double>(cm), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

// ---- run pelvis yaw trim (motiondriver::SetPelvisYawTrim; CET mm.pelvisYawTrim, default 0): deg, + = left, -10..10
void Native_SetMotionPelvisYawTrim(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float deg = 0.0f;
    RED4ext::GetParameter(aFrame, &deg);
    aFrame->code++;
    const bool ok = motiondriver::SetPelvisYawTrim(deg);
    LogInfo("UTPPSetMotionPelvisYawTrim(%.2f) -> %d", static_cast<double>(deg), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

// ---- body yaw offset (motiondriver::SetBodyYawOffset; CET mm.bodyYawOffset, default 0): deg, + = left, -20..20
void Native_SetMotionBodyYawOffset(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float deg = 0.0f;
    RED4ext::GetParameter(aFrame, &deg);
    aFrame->code++;
    const bool ok = motiondriver::SetBodyYawOffset(deg);
    LogInfo("UTPPSetMotionBodyYawOffset(%.2f) -> %d", static_cast<double>(deg), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

// ---- v2 barrel pitch loop (motiondriver::SetBarrelLoop; CET mm.aimAtCrosshair, DLL default off) and muzzle length (0.05..1.0 m)
void Native_SetMotionBarrelLoop(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    bool on = false;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    const bool ok = motiondriver::SetBarrelLoop(on);
    LogInfo("UTPPSetMotionBarrelLoop(%d) -> %d", on ? 1 : 0, ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

void Native_SetMotionBarrelMuzzle(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float len = 0.25f;
    RED4ext::GetParameter(aFrame, &len);
    aFrame->code++;
    const bool ok = motiondriver::SetBarrelMuzzle(len);
    LogInfo("UTPPSetMotionBarrelMuzzle(%.3f) -> %d", static_cast<double>(len), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

// ---- v2 slide feed (motiondriver::SetSlide; CET mm.slide, default off; effective only with a slide bundle)
void Native_SetMotionSlide(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    bool on = false;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    const bool ok = motiondriver::SetSlide(on);
    LogInfo("UTPPSetMotionSlide(%d) -> %d", on ? 1 : 0, ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

// ---- magazine top-up gesture (motiondriver::FeedMagTap; scripts/utpp_motion.reds OnMagazineAmmo: a refill without a reload state)
void Native_FeedMotionMagTap(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    aFrame->code++; // ParamEnd
    bool ok = false;
    try // natives run inside the game's script VM: no exception may leave them
    {
        ok = motiondriver::FeedMagTap();
    }
    catch (...)
    {
    }
    if (aOut)
        *aOut = ok;
}

// ---- Shooter mode (motiondriver::SetShooterMode; CET bodyturn.shooterMode, default off) and the H2 safeguard kill switch (motiondriver::SetH2Guard, default on)
void Native_SetMotionShooterMode(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    bool on = false;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    const bool ok = motiondriver::SetShooterMode(on);
    LogInfo("UTPPSetMotionShooterMode(%d) -> %d", on ? 1 : 0, ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}
void Native_SetMotionH2Guard(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    bool on = false;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    const bool ok = motiondriver::SetH2Guard(on);
    LogInfo("UTPPSetMotionH2Guard(%d) -> %d", on ? 1 : 0, ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}
// CET hip-fire lease -> the DLL (H2 exclusion). Always true, idempotent, no state reset; logged only when the value changes.
void Native_SetMotionHipLease(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    static bool s_last = false;
    bool on = false;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    motiondriver::SetHipLease(on);
    if (on != s_last)
        LogInfo("UTPPSetMotionHipLease(%d) -> 1", on ? 1 : 0);
    s_last = on;
    if (aOut)
        *aOut = true;
}
// Reds calls it once per locomotion update (UTPPFeedFromLocomotion): lock-free, no log. Exactly 1.0 unless Shooter is live.
void Native_MotionShooterModeSpeedMul(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, float* aOut, int64_t)
{
    float moveX = 0.0f, moveY = 0.0f;
    int32_t detailed = 0;
    RED4ext::GetParameter(aFrame, &moveX);
    RED4ext::GetParameter(aFrame, &moveY);
    RED4ext::GetParameter(aFrame, &detailed);
    aFrame->code++;
    const float r = motiondriver::ShooterModeSpeedMul(moveX, moveY, detailed);
    if (aOut)
        *aOut = r;
}
// ---- walk toggle speed (motiondriver::SetWalkSpeed; CET mm.walkSpeed, DLL default 0 = off): game m/s at the full stick, 0 .. 3.5
void Native_SetMotionWalkSpeed(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float ms = 0.0f;
    RED4ext::GetParameter(aFrame, &ms);
    aFrame->code++;
    const bool ok = motiondriver::SetWalkSpeed(ms);
    LogInfo("UTPPSetMotionWalkSpeed(%.2f) -> %d", static_cast<double>(ms), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}
// Reds calls it once per locomotion update (UTPPUpdateWalkSpeed): lock-free, no log. Exactly 0.0 unless v2 drives third-person locomotion.
void Native_MotionWalkSpeedTarget(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, float* aOut, int64_t)
{
    aFrame->code++;
    const float r = motiondriver::WalkSpeedTarget();
    if (aOut)
        *aOut = r;
}
// ---- minimum analog walk speed (motiondriver::SetMinAnalogWalk; CET mm.minWalkSpeed, DLL default 0 = off): game m/s, 0 .. 1.6
void Native_SetMotionMinAnalogWalk(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float ms = 0.0f;
    RED4ext::GetParameter(aFrame, &ms);
    aFrame->code++;
    const bool ok = motiondriver::SetMinAnalogWalk(ms);
    LogInfo("UTPPSetMotionMinAnalogWalk(%.2f) -> %d", static_cast<double>(ms), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}
// Reds calls it once per locomotion update (UTPPUpdateAnalogFloor): lock-free, no log. Exactly 1.0 unless the floor is on and v2 drives
// third-person locomotion.
void Native_MotionAnalogFloorMul(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, float* aOut, int64_t)
{
    float moveX = 0.0f, moveY = 0.0f;
    int32_t detailed = 0;
    bool walk = false;
    RED4ext::GetParameter(aFrame, &moveX);
    RED4ext::GetParameter(aFrame, &moveY);
    RED4ext::GetParameter(aFrame, &detailed);
    RED4ext::GetParameter(aFrame, &walk);
    aFrame->code++;
    const float r = motiondriver::AnalogFloorMul(moveX, moveY, detailed, walk);
    if (aOut)
        *aOut = r;
}
// Reds sends the Tier2 Slow status right before every locomotion feed (UTPPFeedFromLocomotion): one atomic store, no log.
void Native_FeedMotionTier2Slow(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool slow = false;
    RED4ext::GetParameter(aFrame, &slow);
    aFrame->code++;
    motiondriver::FeedTier2Slow(slow);
}
// CET polls it (once per second): the DLL side of Shooter mode is really live (setting on, v2 enabled, no fallback, bundle loaded).
void Native_MotionShooterModeLive(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    aFrame->code++;
    if (aOut)
        *aOut = motiondriver::ShooterModeLive();
}

// ---- v2 slide camera: UTPPSlideOwned (read by the utpp_locomotion.reds GetCameraYaw wrap on the game thread; one atomic load)
void Native_SlideOwned(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    aFrame->code++; // ParamEnd
    if (aOut)
        *aOut = motiondriver::SlideHandlingActive();
}

// ---- v2 orientation warp (motiondriver::SetOrientWarp; CET mm.orientWarp, default off)
void Native_SetMotionOrientWarp(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    bool on = false;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    const bool ok = motiondriver::SetOrientWarp(on);
    LogInfo("UTPPSetMotionOrientWarp(%d) -> %d", on ? 1 : 0, ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

// ---- v2 selection churn guards as a group (motiondriver::SetSelectionGuards; CET mm.selectionGuards, developer option, default on)
void Native_SetMotionSelectionGuards(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    bool on = true;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    const bool ok = motiondriver::SetSelectionGuards(on);
    LogInfo("UTPPSetMotionSelectionGuards(%d) -> %d", on ? 1 : 0, ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

// ---- skin helper recompute on the frames of the weight-0 core writers (bodyhelperruntime::SetZeroWriterRecompute; CET
// setting mm.helperZeroWeightWriters, developer option, default on)
void Native_SetHelperZeroWeightWriters(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    bool on = true;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    bodyhelperruntime::SetZeroWriterRecompute(on);
    LogInfo("UTPPSetHelperZeroWeightWriters(%d)", on ? 1 : 0);
    if (aOut)
        *aOut = true;
}

// ---- melee full pose: switch blend of the arms and the spine at a clip switch (utpp::meleepose::SetSwitchBlend; CET setting
// melee.switchBlend, developer option, default off)
void Native_SetMeleeSwitchBlend(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    bool on = false;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    utpp::meleepose::SetSwitchBlend(on);
    LogInfo("UTPPSetMeleeSwitchBlend(%d)", on ? 1 : 0);
    if (aOut)
        *aOut = true;
}

// ---- v2 motion lean (motiondriver::SetLean / SetLeanHead / SetLeanParams; CET mm.lean, mm.leanHead, default off)
void Native_SetMotionLean(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    bool on = false;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    const bool ok = motiondriver::SetLean(on);
    LogInfo("UTPPSetMotionLean(%d) -> %d", on ? 1 : 0, ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

void Native_SetMotionLeanHead(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    bool on = false;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    const bool ok = motiondriver::SetLeanHead(on);
    LogInfo("UTPPSetMotionLeanHead(%d) -> %d", on ? 1 : 0, ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

void Native_SetMotionLeanParams(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float scale = 1.0f, headScale = 1.0f;
    RED4ext::GetParameter(aFrame, &scale);
    RED4ext::GetParameter(aFrame, &headScale);
    aFrame->code++;
    const bool ok = motiondriver::SetLeanParams(scale, headScale);
    LogInfo("UTPPSetMotionLeanParams(%.2f, %.2f) -> %d", static_cast<double>(scale), static_cast<double>(headScale), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

// ---- B2 strafe S1 (motionhost::SetStrafe / SetStrafeParams; default on, off = the 0.6.5 path)
void Native_SetMMStrafe(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&motionhost::SetStrafe>(aFrame, "UTPPSetMMStrafe");
}
void Native_SetMMStrafeParams(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    bool unarmed = false;
    float rootTurn = 10.0f, rootWarp = 30.0f, exitHold = 0.15f, jogF = 0.0f;
    RED4ext::GetParameter(aFrame, &unarmed);
    RED4ext::GetParameter(aFrame, &rootTurn);
    RED4ext::GetParameter(aFrame, &rootWarp);
    RED4ext::GetParameter(aFrame, &exitHold);
    RED4ext::GetParameter(aFrame, &jogF);
    aFrame->code++;
    const bool ok = motionhost::SetStrafeParams(unarmed, rootTurn, rootWarp, exitHold, jogF);
    LogInfo("UTPPSetMMStrafeParams(%d, %.2f, %.2f, %.3f, %.2f) -> %d", unarmed ? 1 : 0, static_cast<double>(rootTurn),
            static_cast<double>(rootWarp), static_cast<double>(exitHold), static_cast<double>(jogF), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

// ---- B2 strafe S4 jog while aiming (motionhost::AimJogTarget; reds UTPPUpdateAimSpeed): target m/s, 0 = vanilla aim speed
void Native_MMAimJogTarget(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, float* aOut, int64_t)
{
    float moveX = 0.0f, moveY = 0.0f;
    RED4ext::GetParameter(aFrame, &moveX);
    RED4ext::GetParameter(aFrame, &moveY);
    aFrame->code++;
    const float r = motionhost::AimJogTarget(moveX, moveY);
    // G9: no jog answer (jog off, strafe off / not Active, no run strafe gait) = the vanilla aim walk speed; reds queries every
    // locomotion update, so the v2 driver follows the speed the capsule really has
    motiondriver::SetAimJogOff(!(r > 0.0f));
    if (aOut)
        *aOut = r;
}

// ---- B2 strafe S2 chest align (motionhost::SetChestAlign / SetChestAlignParams; default on, off = the S1 pose path)
void Native_SetMMChestAlign(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&motionhost::SetChestAlign>(aFrame, "UTPPSetMMChestAlign");
}
void Native_SetMMChestAlignParams(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float swayKeep = 0.5f, chestMaxDeg = 60.0f;
    RED4ext::GetParameter(aFrame, &swayKeep);
    RED4ext::GetParameter(aFrame, &chestMaxDeg);
    aFrame->code++;
    const bool ok = motionhost::SetChestAlignParams(swayKeep, chestMaxDeg);
    LogInfo("UTPPSetMMChestAlignParams(%.2f, %.1f) -> %d", static_cast<double>(swayKeep), static_cast<double>(chestMaxDeg), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

// ---- 0.6.4 item render plane (render/ItemPlane.cpp)
uint64_t g_itemPlaneFailLogs = 0; // game thread
void Native_SetItemRenderPlane(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    RED4ext::Handle<RED4ext::IScriptable> item;
    int32_t plane = 0;
    RED4ext::GetParameter(aFrame, &item);
    RED4ext::GetParameter(aFrame, &plane);
    aFrame->code++;
    std::string why;
    const bool ok = utpp::itemplane::QueuePlaneEvent(item ? item.instance : nullptr, plane, why);
    utpp::itemplane::CountEvent(plane, ok);
    // Success is logged by the script ("[utpp_itemplane] plane -> ..."); failures here (first 20, then every 100th).
    if (!ok && (g_itemPlaneFailLogs++ < 20 || (g_itemPlaneFailLogs % 100) == 0))
        LogWarn("UTPPSetItemRenderPlane(plane %d) failed: %s", plane, why.c_str());
    if (aOut)
        *aOut = ok;
}
void Native_SetItemPlaneOverride(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    bool on = false;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    // Script natives run on the game thread: SetOverride may re-apply (calls the plane setter) on a change.
    const bool live = utpp::itemplane::SetOverride(on); // logs one line on a change
    if (aOut)
        *aOut = live;
}
void Native_ItemPlaneNativeStatus(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::CString* aOut, int64_t)
{
    aFrame->code++;
    if (aOut)
        *aOut = RED4ext::CString(utpp::itemplane::Status().c_str());
}
// 2026-11-01 registry re-apply / dump (render/ItemPlane.cpp). Game thread (script natives) only: both call into the components.
void Native_ItemPlaneReapply(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, int32_t* aOut, int64_t)
{
    aFrame->code++;
    const int32_t written = utpp::itemplane::Reapply();
    if (aOut)
        *aOut = written;
}
void Native_ItemPlaneDump(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::CString* aOut, int64_t)
{
    int32_t max = 0;
    RED4ext::GetParameter(aFrame, &max);
    aFrame->code++;
    const std::string dump = utpp::itemplane::Dump(max, GetPlayerEntityRaw()); // logs every line
    if (aOut)
        *aOut = RED4ext::CString(dump.c_str());
}
// Round 2 (render-plane audit A3): read-only dump of the player's, its items' and their child items' visual / effect components.
void Native_ItemPlaneDumpPlayer(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::CString* aOut, int64_t)
{
    int32_t max = 0;
    RED4ext::GetParameter(aFrame, &max);
    aFrame->code++;
    const std::string dump = utpp::itemplane::DumpPlayer(max); // logs every line
    if (aOut)
        *aOut = RED4ext::CString(dump.c_str());
}

// ---- 0.6.5 game look write (input/GameLook.cpp)
void Native_SetGameLook(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float yaw = 0.0f, pitch = 0.0f, ahead = 0.0f; // 0.6.5b: optional entityYawAheadDeg (omitted = 0)
    RED4ext::GetParameter(aFrame, &yaw);
    RED4ext::GetParameter(aFrame, &pitch);
    RED4ext::GetParameter(aFrame, &ahead);
    aFrame->code++;
    const bool ok = utpp::gamelook::SetTarget(yaw, pitch, ahead); // logs a failure reason once per change
    if (aOut)
        *aOut = ok;
}
void Native_StopGameLook(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    aFrame->code++;
    const bool was = utpp::gamelook::Stop();
    if (aOut)
        *aOut = was;
}
void Native_GameLookVehicleExit(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    bool utppTpp = false;
    int32_t rideTpp = -1;
    RED4ext::GetParameter(aFrame, &utppTpp);
    RED4ext::GetParameter(aFrame, &rideTpp);
    aFrame->code++;
    const bool ok = utpp::gamelook::VehicleExitStart(utppTpp, rideTpp); // logs its own line
    if (aOut)
        *aOut = ok;
}
void Native_GameLookStatus(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::CString* aOut, int64_t)
{
    aFrame->code++;
    if (aOut)
        *aOut = RED4ext::CString(utpp::gamelook::Status().c_str());
}

// ---- Phase 5.3d (0.3.8)
void Native_LogLine(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    RED4ext::CString line;
    RED4ext::GetParameter(aFrame, &line);
    aFrame->code++;
    const char* str = line.c_str();
    if (!str)
        return;
    // Always logged (independent of UTPPSetMMTrace): CET prints do not reach a file in game.
    char buf[480];
    size_t n = 0;
    for (; str[n] != 0 && n + 1 < sizeof(buf); ++n)
        buf[n] = (str[n] >= 32 && str[n] < 127) ? str[n] : '?';
    buf[n] = 0;
    LogInfo("UTPP CET %s", buf);
}

void Native_SetMMLargeTurn(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool on = true;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    motionhost::SetLargeTurn(on);
    LogInfo("UTPPSetMMLargeTurn(%d)", on ? 1 : 0);
}

void Native_SetMMPivotAccel(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float stand = 8.0f, crouch = 8.0f, sprint = 12.0f;
    RED4ext::GetParameter(aFrame, &stand);
    RED4ext::GetParameter(aFrame, &crouch);
    RED4ext::GetParameter(aFrame, &sprint);
    aFrame->code++;
    const bool ok = motionhost::SetPivotAccel(stand, crouch, sprint);
    LogInfo("UTPPSetMMPivotAccel(%.2f, %.2f, %.2f) -> %d", static_cast<double>(stand), static_cast<double>(crouch),
            static_cast<double>(sprint), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

void Native_SetMMStanceParams(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float half = 3.0f, sep = 6.0f;
    bool knee = true;
    RED4ext::GetParameter(aFrame, &half);
    RED4ext::GetParameter(aFrame, &sep);
    RED4ext::GetParameter(aFrame, &knee);
    aFrame->code++;
    const bool ok = motionhost::SetStanceParams(half, sep, knee);
    LogInfo("UTPPSetMMStanceParams(%.1f, %.1f, %d) -> %d", static_cast<double>(half), static_cast<double>(sep), knee ? 1 : 0,
            ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

// ---- Phase 5.3c-3 (0.3.7)
void Native_SetMMRootWarp(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    bool on = true;
    float maxDeg = 60.0f;
    RED4ext::GetParameter(aFrame, &on);
    RED4ext::GetParameter(aFrame, &maxDeg);
    aFrame->code++;
    const bool ok = motionhost::SetRootWarp(on, maxDeg);
    LogInfo("UTPPSetMMRootWarp(%d, %.1f) -> %d", on ? 1 : 0, static_cast<double>(maxDeg), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

void Native_SetMMHeadingLean(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool on = true;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    motionhost::SetHeadingLean(on);
    LogInfo("UTPPSetMMHeadingLean(%d)", on ? 1 : 0);
}

void Native_SetMMStanceSwing(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool on = true;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    motionhost::SetStanceSwing(on);
    LogInfo("UTPPSetMMStanceSwing(%d)", on ? 1 : 0);
}

void Native_SetMMStanceGuard(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    bool on = true;
    float halfCm = 5.0f;
    RED4ext::GetParameter(aFrame, &on);
    RED4ext::GetParameter(aFrame, &halfCm);
    aFrame->code++;
    const bool ok = motionhost::SetStanceGuard(on, halfCm);
    LogInfo("UTPPSetMMStanceGuard(%d, %.1f) -> %d", on ? 1 : 0, static_cast<double>(halfCm), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

void Native_SetMMSteerGap(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float gap = 8.0f;
    RED4ext::GetParameter(aFrame, &gap);
    aFrame->code++;
    const bool ok = motionhost::SetSteerGap(gap);
    LogInfo("UTPPSetMMSteerGap(%.1f) -> %d", static_cast<double>(gap), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

void Native_SetMMStopSync(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    bool on = true;
    float hl = 0.05f;
    RED4ext::GetParameter(aFrame, &on);
    RED4ext::GetParameter(aFrame, &hl);
    aFrame->code++;
    const bool ok = motionhost::SetStopSync(on, hl);
    LogInfo("UTPPSetMMStopSync(%d, %.3f) -> %d", on ? 1 : 0, static_cast<double>(hl), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

void Native_SetMMAccelMatch(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool value = true;
    RED4ext::GetParameter(aFrame, &value);
    aFrame->code++;
    motionhost::SetAccelMatch(value);
    LogInfo("UTPPSetMMAccelMatch(%d)", value ? 1 : 0);
}

void Native_SetMMAccelParams(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float stand = 0.0f, crouch = 0.0f, sprint = 0.0f;
    RED4ext::GetParameter(aFrame, &stand);
    RED4ext::GetParameter(aFrame, &crouch);
    RED4ext::GetParameter(aFrame, &sprint);
    aFrame->code++;
    const bool ok = motionhost::SetAccelParams(stand, crouch, sprint);
    LogInfo("UTPPSetMMAccelParams(%.2f, %.2f, %.2f) -> %d", static_cast<double>(stand), static_cast<double>(crouch),
            static_cast<double>(sprint), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

void Native_MMAccelTarget(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, float* aOut, int64_t)
{
    float base = 0.0f, mul = 1.0f;
    RED4ext::GetParameter(aFrame, &base);
    RED4ext::GetParameter(aFrame, &mul);
    aFrame->code++;
    const float r = motionhost::AccelTarget(base, mul);
    if (aOut)
        *aOut = r;
}

void Native_SetMMFootLock(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool value = true;
    RED4ext::GetParameter(aFrame, &value);
    aFrame->code++;
    motionhost::SetFootLock(value);
    LogInfo("UTPPSetMMFootLock(%d)", value ? 1 : 0);
}

void MMTraceLog(const char* aLine)
{
    LogInfo("%s", aLine);
}

void Native_SetMMTrace(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool value = false;
    RED4ext::GetParameter(aFrame, &value);
    aFrame->code++;
    motionhost::SetLogger(&MMTraceLog);
    motionhost::SetTrace(value);
    utpp::motiondriver::SetTrace(value); // UTPP-MOTION SEAM motionfix (item 4): MOTION trace line follows mm.trace (v2 frames only)
    g_hot.timing.store(value, std::memory_order_relaxed); // FK detour own-cost timing + reload-contacts funnel line only while tracing
    LogInfo("UTPPSetMMTrace(%d)", value ? 1 : 0);
}

void Native_SetMatchMode(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    int32_t mode = 3;
    RED4ext::GetParameter(aFrame, &mode);
    aFrame->code++;
    if (mode < 0 || mode > 3)
    {
        LogWarn("UTPPSetMatchMode(%d) ignored: expected 0..3", mode);
        return;
    }
    g_hot.matchMode.store(mode, std::memory_order_relaxed);
    LogInfo("UTPPSetMatchMode(%d)", mode);
}

// Returns true only if the name resolved against the currently armed rig. When not armed the name
// is stored and resolved at the next arm, and false is returned.
void Native_SetTestBone(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    RED4ext::CString name;
    RED4ext::GetParameter(aFrame, &name);
    aFrame->code++;
    bool result = false;
    const char* str = name.c_str();
    if (str && str[0] && std::strlen(str) < 128)
    {
        std::lock_guard lock(g_coldMutex);
        if (g_hot.armed.load() && g_armInfo.metaRig)
        {
            uint32_t count = 0;
            const int32_t idx = ResolveBoneIndex(g_armInfo.metaRig, RED4ext::CName(str).hash, count);
            if (idx >= 0)
            {
                g_boneName = str;
                g_armInfo.boneIndex = idx;
                g_armInfo.boneCount = count;
                g_hot.boneIndex.store(idx, std::memory_order_relaxed);
                result = true;
            }
            LogInfo("UTPPSetTestBone('%s') -> index %d (rig bones %u)%s", str, idx, count,
                    idx >= 0 ? "" : ", previous bone kept");
        }
        else
        {
            g_boneName = str;
            g_hot.boneIndex.store(-1, std::memory_order_relaxed);
            LogInfo("UTPPSetTestBone('%s') stored; not armed, will resolve on next arm", str);
        }
    }
    if (aOut)
        *aOut = result;
}

void Native_Arm(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, int32_t* aOut, int64_t)
{
    aFrame->code++;
    int32_t code = kArmBusy;
    {
        std::unique_lock lock(g_coldMutex, std::try_to_lock);
        if (lock.owns_lock())
            code = ArmLocked(true);
    }
    if (aOut)
        *aOut = code;
}

// ---- Motion matching natives (Phase 4b) ----

// batch 1b (2026-10-13): extra fields for the NEXT UTPPFeedLocomotion call on the same thread (scripts/utpp.reds sends them right
// before the feed): PSM WalkToggled and the capsule world position -> FeedInput.walkToggled / capsulePos (feed dump columns).
// Consumed by that feed; an older script never sends them (FeedInput.haveExtra false).
struct PendingFeedExtra
{
    bool valid = false;
    bool walkToggled = false;
    float capsule[3] = {0.0f, 0.0f, 0.0f};
};
thread_local PendingFeedExtra t_feedExtra;

void Native_FeedLocomotionExtra(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool walk = false;
    RED4ext::Vector4 pos{};
    RED4ext::GetParameter(aFrame, &walk);
    RED4ext::GetParameter(aFrame, &pos);
    aFrame->code++;
    t_feedExtra.valid = true;
    t_feedExtra.walkToggled = walk;
    t_feedExtra.capsule[0] = pos.X;
    t_feedExtra.capsule[1] = pos.Y;
    t_feedExtra.capsule[2] = pos.Z;
}

// AIR HOLD stage (2026-10-13): air fields for the NEXT UTPPFeedLocomotion call on the same thread (scripts/utpp.reds
// UTPPFeedFromLocomotion sends them right before the feed): onGround, downward raycast distance (-1 no hit, -2 not probed), PSM.Fall
// (previous frame). Consumed by that feed (FeedInput.haveAir); without it the air area infers onGround from the PSM detail.
struct PendingFeedAir
{
    bool valid = false;
    bool onGround = true;
    float dist = -2.0f;
    int32_t fall = 0;
};
thread_local PendingFeedAir t_feedAir;

void Native_FeedAir(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool onGround = true;
    float dist = -2.0f;
    int32_t fall = 0;
    RED4ext::GetParameter(aFrame, &onGround);
    RED4ext::GetParameter(aFrame, &dist);
    RED4ext::GetParameter(aFrame, &fall);
    aFrame->code++;
    t_feedAir.valid = true;
    t_feedAir.onGround = onGround;
    t_feedAir.dist = dist;
    t_feedAir.fall = fall;
}

void Native_MMAirWantsProbe(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    aFrame->code++;
    const bool r = motionhost::AirWantsProbe();
    if (aOut)
        *aOut = r;
}

void Native_SetMMAir(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    int32_t mode = 0;
    RED4ext::GetParameter(aFrame, &mode);
    aFrame->code++;
    const bool ok = motionhost::SetAir(mode);
    LogInfo("UTPPSetMMAir(%d) -> %d", static_cast<int>(mode), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

void Native_SetMMAirParams(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    int32_t lead = 2;
    float landLead = 0.2f, rateMin = 0.7f, rateMax = 1.8f, recLight = 0.3f, recHeavy = 0.5f, holdRate = 0.5f, heavyVz = 7.0f;
    RED4ext::GetParameter(aFrame, &lead);
    RED4ext::GetParameter(aFrame, &landLead);
    RED4ext::GetParameter(aFrame, &rateMin);
    RED4ext::GetParameter(aFrame, &rateMax);
    RED4ext::GetParameter(aFrame, &recLight);
    RED4ext::GetParameter(aFrame, &recHeavy);
    RED4ext::GetParameter(aFrame, &holdRate);
    RED4ext::GetParameter(aFrame, &heavyVz);
    aFrame->code++;
    const bool ok = motionhost::SetAirParams(lead, landLead, rateMin, rateMax, recLight, recHeavy, holdRate, heavyVz);
    LogInfo("UTPPSetMMAirParams(%d, %.3f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f) -> %d", static_cast<int>(lead), static_cast<double>(landLead),
            static_cast<double>(rateMin), static_cast<double>(rateMax), static_cast<double>(recLight), static_cast<double>(recHeavy),
            static_cast<double>(holdRate), static_cast<double>(heavyVz), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

void Native_SetMMAirProbe(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&motionhost::SetAirProbe>(aFrame, "UTPPSetMMAirProbe");
}

void Native_SetMMAirSteer(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&motionhost::SetAirSteer>(aFrame, "UTPPSetMMAirSteer");
}

// BATCH 6 AIR stage (CET mm.airRobust)
void Native_SetMMAirRobust(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    Native_SetBool057<&motionhost::SetAirRobust>(aFrame, "UTPPSetMMAirRobust");
}

// CLIMB RELEASE stage (CET mm.climbRelease)
void Native_SetMMClimbRelease(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    bool on = true;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    const bool ok = motionhost::SetClimbRelease(on);
    LogInfo("UTPPSetMMClimbRelease(%d) -> %d", on ? 1 : 0, ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

void Native_SetMMFeedDump(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    bool on = false;
    RED4ext::GetParameter(aFrame, &on);
    aFrame->code++;
    const bool ok = motionhost::SetFeedDump(on); // logs the file / summary itself
    LogInfo("UTPPSetMMFeedDump(%d) -> %d", on ? 1 : 0, ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

void Native_MMFeedDumpStatus(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::CString* aOut, int64_t)
{
    aFrame->code++;
    if (aOut)
        *aOut = RED4ext::CString(motionhost::FeedDumpStatus().c_str());
}

// batch 1b: folder of the feed dump files = RED4ext's log folder (<game>\red4ext\logs\, where RED4ext writes utpp-<time>.log):
// <RED4ext.dll folder>\logs, else <plugin folder>\..\..\logs (red4ext\plugins\UTPP\), else the plugin folder.
std::wstring FeedDumpFolder(const std::wstring& aPluginDir)
{
    auto isDir = [](const std::wstring& aPath) {
        const DWORD a = GetFileAttributesW(aPath.c_str());
        return a != INVALID_FILE_ATTRIBUTES && (a & FILE_ATTRIBUTE_DIRECTORY) != 0;
    };
    wchar_t buf[MAX_PATH * 2] = {};
    if (HMODULE r4 = GetModuleHandleW(L"RED4ext.dll"); r4 && GetModuleFileNameW(r4, buf, static_cast<DWORD>(std::size(buf))) > 0)
    {
        std::wstring p = buf;
        const size_t slash = p.find_last_of(L"\\/");
        if (slash != std::wstring::npos)
        {
            p = p.substr(0, slash + 1) + L"logs\\";
            if (isDir(p))
                return p;
        }
    }
    const std::wstring up = aPluginDir + L"..\\..\\logs\\";
    wchar_t full[MAX_PATH * 2] = {};
    const DWORD n = GetFullPathNameW(up.c_str(), static_cast<DWORD>(std::size(full)), full, nullptr);
    if (n > 0 && n < std::size(full) && isDir(full))
        return full;
    return aPluginDir;
}

void Native_FeedLocomotion(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    RED4ext::Vector4 vel{};
    RED4ext::Vector2 move{};
    float camYaw = 0.0f, playerYaw = 0.0f, dt = 0.0f;
    int32_t loco = 0;
    bool tpp = false;
    bool haveTarget = false;
    float targetYaw = 0.0f;
    int32_t psmBlock = 0;
    RED4ext::GetParameter(aFrame, &vel);
    RED4ext::GetParameter(aFrame, &move);
    RED4ext::GetParameter(aFrame, &camYaw);
    RED4ext::GetParameter(aFrame, &playerYaw);
    RED4ext::GetParameter(aFrame, &loco);
    RED4ext::GetParameter(aFrame, &tpp);
    RED4ext::GetParameter(aFrame, &dt);
    RED4ext::GetParameter(aFrame, &haveTarget);
    RED4ext::GetParameter(aFrame, &targetYaw);
    RED4ext::GetParameter(aFrame, &psmBlock);
    aFrame->code++;
    motionhost::FeedInput in;
    in.velWorld[0] = vel.X;
    in.velWorld[1] = vel.Y;
    in.velWorld[2] = vel.Z;
    in.move[0] = move.X;
    in.move[1] = move.Y;
    in.cameraYawDeg = camYaw;
    in.playerYawDeg = playerYaw;
    in.locoState = loco;
    in.tppActive = tpp;
    in.dt = dt;
    in.haveTarget = haveTarget;
    in.targetYawDeg = targetYaw;
    in.psmBlock = static_cast<uint32_t>(psmBlock);
    if (t_feedExtra.valid) // batch 1b: UTPPFeedLocomotionExtra right before this feed
    {
        in.haveExtra = true;
        in.walkToggled = t_feedExtra.walkToggled;
        for (int i = 0; i < 3; ++i)
            in.capsulePos[i] = t_feedExtra.capsule[i];
        t_feedExtra.valid = false;
    }
    if (t_feedAir.valid) // AIR HOLD stage: UTPPFeedAir right before this feed
    {
        in.haveAir = true;
        in.onGround = t_feedAir.onGround;
        in.distToGround = t_feedAir.dist;
        in.psmFall = t_feedAir.fall;
        t_feedAir.valid = false;
    }
    utpp::motiondriver::FeedAirFromInput(in); // UTPP-MOTION SEAM air: UTPPFeedAir fields -> one-shot v2 AirFeed of this feed's Step (v2 mutex only; v1 untouched)
    motionhost::Feed(in);
    aimofs::OnFeed(tpp, static_cast<uint32_t>(psmBlock)); // 0.6.3 aim offset guards (UpperBody, melee, vehicle, TPP)
    utpp::gamelook::OnFeed((static_cast<uint32_t>(psmBlock) & mm::integ::kPsmVehicle) != 0, tpp); // vehicle edge look zeroing
}

// ---- Phase 5.1 body yaw from the MM animation ----

void Native_SetMMBodyTurn(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool value = true;
    RED4ext::GetParameter(aFrame, &value);
    aFrame->code++;
    motionhost::SetBodyTurn(value);
    LogInfo("UTPPSetMMBodyTurn(%d)", value ? 1 : 0);
}

void Native_SetMMBodyTurnParams(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float t = 0.4f, minR = 20.0f, maxCorr = 180.0f, maxR = 900.0f;
    RED4ext::GetParameter(aFrame, &t);
    RED4ext::GetParameter(aFrame, &minR);
    RED4ext::GetParameter(aFrame, &maxCorr);
    RED4ext::GetParameter(aFrame, &maxR);
    aFrame->code++;
    const bool ok = motionhost::SetBodyTurnParams(t, minR, maxCorr, maxR);
    LogInfo("UTPPSetMMBodyTurnParams(%.3f, %.1f, %.1f, %.1f) -> %d", static_cast<double>(t), static_cast<double>(minR),
            static_cast<double>(maxCorr), static_cast<double>(maxR), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

void Native_MMBodyTurnActive(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    aFrame->code++;
    if (aOut)
        *aOut = motionhost::BodyTurnActive();
}

void Native_MMBodyYawCommand(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, float* aOut, int64_t)
{
    float cur = 0.0f, base = 0.0f;
    RED4ext::GetParameter(aFrame, &cur);
    RED4ext::GetParameter(aFrame, &base);
    aFrame->code++;
    const float r = motionhost::BodyYawCommand(cur, base);
    if (aOut)
        *aOut = r;
}

void Native_SetMMEnabled(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    bool value = false;
    RED4ext::GetParameter(aFrame, &value);
    aFrame->code++;
    motionhost::SetEnabled(value);
    LogInfo("UTPPSetMMEnabled(%d)", value ? 1 : 0);
}

void Native_SetMMTagFilter(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    RED4ext::CString filter;
    RED4ext::GetParameter(aFrame, &filter);
    aFrame->code++;
    const char* str = filter.c_str();
    bool ok = false;
    if (str && std::strlen(str) < 128)
        ok = motionhost::SetTagFilter(str);
    LogInfo("UTPPSetMMTagFilter('%s') -> %d", str ? str : "", ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

void Native_SetMMBlendTime(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    float seconds = motionhost::kDefaultBlendTime;
    RED4ext::GetParameter(aFrame, &seconds);
    aFrame->code++;
    motionhost::SetBlendTime(seconds);
    LogInfo("UTPPSetMMBlendTime(%.3f)", static_cast<double>(seconds));
}

void Native_SetMMGateFade(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    float hardOut = motionhost::kDefaultHardOutTime, hold = motionhost::kDefaultReopenHold;
    RED4ext::GetParameter(aFrame, &hardOut);
    RED4ext::GetParameter(aFrame, &hold);
    aFrame->code++;
    const bool ok = motionhost::SetGateFade(hardOut, hold);
    LogInfo("UTPPSetMMGateFade(%.3f, %.3f) -> %d", static_cast<double>(hardOut), static_cast<double>(hold), ok ? 1 : 0);
    if (aOut)
        *aOut = ok;
}

void Native_SetMMAllowedStates(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, void*, int64_t)
{
    int32_t mask = 2;
    RED4ext::GetParameter(aFrame, &mask);
    aFrame->code++;
    motionhost::SetAllowedStates(static_cast<uint32_t>(mask));
    LogInfo("UTPPSetMMAllowedStates(0x%X)", static_cast<unsigned>(mask));
}

void Native_MMWantsFeed(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    aFrame->code++;
    if (aOut)
        *aOut = motionhost::WantsFeed();
}

void Native_VehicleJumpOutEpoch(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, int32_t* aOut, int64_t)
{
    aFrame->code++;
    if (aOut)
        *aOut = motionhost::JumpOutLaunchEpoch();
}

// ---- 0.5.1 motion matching database switching ----
void Native_ListDatabases(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::CString* aOut, int64_t)
{
    aFrame->code++;
    if (aOut)
        *aOut = RED4ext::CString(motionhost::ListDatabases().c_str());
}

void Native_LoadDatabase(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, bool* aOut, int64_t)
{
    RED4ext::CString name;
    RED4ext::GetParameter(aFrame, &name);
    aFrame->code++;
    const std::string n = name.c_str() ? name.c_str() : "";
    const bool ok = motionhost::RequestDatabase(n);
    LogInfo("UTPPLoadDatabase('%s') -> %s (%s)", n.c_str(), ok ? "queued" : "rejected", motionhost::DatabaseSwitchStatus().c_str());
    if (aOut)
        *aOut = ok;
}

void Native_GetDatabaseName(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::CString* aOut, int64_t)
{
    aFrame->code++;
    if (aOut)
        *aOut = RED4ext::CString(motionhost::DatabaseName().c_str());
}

void Native_GetDatabaseSwitchStatus(RED4ext::IScriptable*, RED4ext::CStackFrame* aFrame, RED4ext::CString* aOut, int64_t)
{
    aFrame->code++;
    if (aOut)
        *aOut = RED4ext::CString(motionhost::DatabaseSwitchStatus().c_str());
}

void RegisterTypes()
{
}

void PostRegisterTypes()
{
    auto* rtti = RED4ext::CRTTISystem::Get();
    const RED4ext::CBaseFunction::Flags flags = {.isNative = true, .isStatic = true};

    auto* f1 = RED4ext::CGlobalFunction::Create("UTPPGetStatus", "UTPPGetStatus", &Native_GetStatus);
    f1->flags = flags;
    f1->SetReturnType("String");
    rtti->RegisterFunction(f1);
    auto* bodyArm = RED4ext::CGlobalFunction::Create("UTPPBodyPoseTraceArm", "UTPPBodyPoseTraceArm", &Native_BodyTraceArm);
    bodyArm->flags = flags; bodyArm->AddParam("Int32", "frames"); bodyArm->SetReturnType("Bool");
    rtti->RegisterFunction(bodyArm);
    auto* bodyFlush = RED4ext::CGlobalFunction::Create("UTPPBodyPoseTraceFlush", "UTPPBodyPoseTraceFlush", &Native_BodyTraceFlush);
    bodyFlush->flags = flags; bodyFlush->SetReturnType("Bool"); rtti->RegisterFunction(bodyFlush);
    auto* bodyContinuous = RED4ext::CGlobalFunction::Create("UTPPBodyPoseTraceContinuous", "UTPPBodyPoseTraceContinuous", &Native_BodyTraceContinuous);
    bodyContinuous->flags = flags; bodyContinuous->AddParam("Bool", "on"); bodyContinuous->SetReturnType("Bool"); rtti->RegisterFunction(bodyContinuous);
    auto* poseCaptureStart = RED4ext::CGlobalFunction::Create("UTPPPoseCaptureStart", "UTPPPoseCaptureStart", &Native_PoseCaptureStart);
    poseCaptureStart->flags = flags; poseCaptureStart->AddParam("Int32", "keepMinutes"); poseCaptureStart->SetReturnType("Bool"); rtti->RegisterFunction(poseCaptureStart);
    auto* postHelperStream = RED4ext::CGlobalFunction::Create("UTPPSetPostHelperStream", "UTPPSetPostHelperStream", &Native_SetPostHelperStream);
    postHelperStream->flags = flags; postHelperStream->AddParam("Bool", "on"); postHelperStream->SetReturnType("Bool"); rtti->RegisterFunction(postHelperStream);
    auto* poseCaptureStop = RED4ext::CGlobalFunction::Create("UTPPPoseCaptureStop", "UTPPPoseCaptureStop", &Native_PoseCaptureStop);
    poseCaptureStop->flags = flags; poseCaptureStop->SetReturnType("String"); rtti->RegisterFunction(poseCaptureStop);
    auto* poseCaptureStatus = RED4ext::CGlobalFunction::Create("UTPPPoseCaptureStatus", "UTPPPoseCaptureStatus", &Native_PoseCaptureStatus);
    poseCaptureStatus->flags = flags; poseCaptureStatus->SetReturnType("String"); rtti->RegisterFunction(poseCaptureStatus);
    auto* diagReport = RED4ext::CGlobalFunction::Create("UTPPDiagWriteReport", "UTPPDiagWriteReport", &Native_DiagWriteReport);
    diagReport->flags = flags; diagReport->AddParam("String", "body"); diagReport->SetReturnType("String"); rtti->RegisterFunction(diagReport);
    auto* bodyStatus = RED4ext::CGlobalFunction::Create("UTPPBodyPoseTraceStatus", "UTPPBodyPoseTraceStatus", &Native_BodyTraceStatus);
    bodyStatus->flags = flags; bodyStatus->SetReturnType("String"); rtti->RegisterFunction(bodyStatus);
    auto* census = RED4ext::CGlobalFunction::Create("UTPPPoseCensus", "UTPPPoseCensus", &Native_PoseCensus);
    census->flags = flags; census->AddParam("Bool", "enabled"); census->SetReturnType("Bool"); rtti->RegisterFunction(census);
    auto* censusDump = RED4ext::CGlobalFunction::Create("UTPPPoseCensusDump", "UTPPPoseCensusDump", &Native_PoseCensusDump);
    censusDump->flags = flags; censusDump->SetReturnType("String"); rtti->RegisterFunction(censusDump);
    auto* poseWriteTpp = RED4ext::CGlobalFunction::Create("UTPPSetPoseWriteTpp", "UTPPSetPoseWriteTpp", &Native_SetPoseWriteTpp);
    poseWriteTpp->flags = flags; poseWriteTpp->AddParam("Bool", "enabled");
    poseWriteTpp->SetReturnType("Bool"); rtti->RegisterFunction(poseWriteTpp);
    utpp::motiondriver::RegisterNatives(); // UTPP-MOTION SEAM: UTPPSetMotionEnabled / SetMMRigBody / SetMMOverlayInputs / GetMotionStatus

    auto* f2 = RED4ext::CGlobalFunction::Create("UTPPSetEnabled", "UTPPSetEnabled", &Native_SetEnabled);
    f2->flags = flags;
    f2->AddParam("Bool", "enabled");
    rtti->RegisterFunction(f2);

    auto* f3 = RED4ext::CGlobalFunction::Create("UTPPSetTestRotation", "UTPPSetTestRotation", &Native_SetTestRotation);
    f3->flags = flags;
    f3->AddParam("Float", "degrees");
    rtti->RegisterFunction(f3);

    auto* f4 = RED4ext::CGlobalFunction::Create("UTPPSetTestAxis", "UTPPSetTestAxis", &Native_SetTestAxis);
    f4->flags = flags;
    f4->AddParam("Int32", "axis");
    rtti->RegisterFunction(f4);

    auto* f5 = RED4ext::CGlobalFunction::Create("UTPPSetTestBone", "UTPPSetTestBone", &Native_SetTestBone);
    f5->flags = flags;
    f5->AddParam("String", "boneName");
    f5->SetReturnType("Bool");
    rtti->RegisterFunction(f5);

    auto* fSpeed = RED4ext::CGlobalFunction::Create("UTPPSetMMSpeedMap", "UTPPSetMMSpeedMap", &Native_SetMMSpeedMap);
    fSpeed->flags = flags;
    for (const char* p : {"gameWalk", "gameRun", "dbWalk", "dbRun", "rateMin", "rateMax"})
        fSpeed->AddParam("Float", p);
    rtti->RegisterFunction(fSpeed);

    auto* fSp = RED4ext::CGlobalFunction::Create("UTPPSetMMSpeedPoint", "UTPPSetMMSpeedPoint", &Native_SetMMSpeedPoint);
    fSp->flags = flags;
    fSp->AddParam("Int32", "loco");
    fSp->AddParam("Int32", "index");
    fSp->AddParam("Float", "gameSpeed");
    fSp->AddParam("Float", "dbSpeed");
    rtti->RegisterFunction(fSp);

    auto* fInp = RED4ext::CGlobalFunction::Create("UTPPSetMMInputDriven", "UTPPSetMMInputDriven", &Native_SetMMInputDriven);
    fInp->flags = flags;
    fInp->AddParam("Bool", "on");
    rtti->RegisterFunction(fInp);

    auto* fTc = RED4ext::CGlobalFunction::Create("UTPPSetMMTransitionCost", "UTPPSetMMTransitionCost", &Native_SetMMTransitionCost);
    fTc->flags = flags;
    fTc->AddParam("Float", "cost");
    rtti->RegisterFunction(fTc);

    auto* fCam = RED4ext::CGlobalFunction::Create("UTPPSetMMInputCameraRelative", "UTPPSetMMInputCameraRelative",
                                                  &Native_SetMMInputCameraRelative);
    fCam->flags = flags;
    fCam->AddParam("Bool", "on");
    rtti->RegisterFunction(fCam);

    auto* fFoot = RED4ext::CGlobalFunction::Create("UTPPSetMMFootLock", "UTPPSetMMFootLock", &Native_SetMMFootLock);
    fFoot->flags = flags;
    fFoot->AddParam("Bool", "on");
    rtti->RegisterFunction(fFoot);

    // Phase 5.3c-2 (0.3.6)
    auto* d1 = RED4ext::CGlobalFunction::Create("UTPPSetMMCurationMoving", "UTPPSetMMCurationMoving", &Native_SetMMCurationMoving);
    d1->flags = flags;
    d1->AddParam("Bool", "on");
    rtti->RegisterFunction(d1);

    auto* d2 = RED4ext::CGlobalFunction::Create("UTPPSetMMSteerGap", "UTPPSetMMSteerGap", &Native_SetMMSteerGap);
    d2->flags = flags;
    d2->AddParam("Float", "gapDeg");
    d2->SetReturnType("Bool");
    rtti->RegisterFunction(d2);

    auto* d3 = RED4ext::CGlobalFunction::Create("UTPPSetMMStanceGuard", "UTPPSetMMStanceGuard", &Native_SetMMStanceGuard);
    d3->flags = flags;
    d3->AddParam("Bool", "on");
    d3->AddParam("Float", "minHalfWidthCm");
    d3->SetReturnType("Bool");
    rtti->RegisterFunction(d3);

    // Phase 5.3e (0.3.9)
    auto* k1 = RED4ext::CGlobalFunction::Create("UTPPSetMMSlowWalkLoop", "UTPPSetMMSlowWalkLoop", &Native_SetMMSlowWalkLoop);
    k1->flags = flags;
    k1->AddParam("Bool", "on");
    rtti->RegisterFunction(k1);

    auto* k2 = RED4ext::CGlobalFunction::Create("UTPPSetMMStanceSmooth", "UTPPSetMMStanceSmooth", &Native_SetMMStanceSmooth);
    k2->flags = flags;
    k2->AddParam("Float", "smoothTime");
    k2->SetReturnType("Bool");
    rtti->RegisterFunction(k2);

    auto* k3 = RED4ext::CGlobalFunction::Create("UTPPSetMMStanceTransitionWindow", "UTPPSetMMStanceTransitionWindow",
                                                &Native_SetMMStanceTransitionWindow); // 0.5.2
    k3->flags = flags;
    k3->AddParam("Float", "seconds");
    k3->SetReturnType("Bool");
    rtti->RegisterFunction(k3);

    // 0.5.3 locomotion kill switches
    auto* l1 = RED4ext::CGlobalFunction::Create("UTPPSetMMStickFilter", "UTPPSetMMStickFilter", &Native_SetMMStickFilter);
    l1->flags = flags;
    l1->AddParam("Bool", "on");
    rtti->RegisterFunction(l1);
    auto* l2 = RED4ext::CGlobalFunction::Create("UTPPSetMMFamilyRate", "UTPPSetMMFamilyRate", &Native_SetMMFamilyRate);
    l2->flags = flags;
    l2->AddParam("Bool", "on");
    rtti->RegisterFunction(l2);
    auto* l3 = RED4ext::CGlobalFunction::Create("UTPPSetMMStartRules", "UTPPSetMMStartRules", &Native_SetMMStartRules);
    l3->flags = flags;
    l3->AddParam("Bool", "on");
    rtti->RegisterFunction(l3);
    // 0.5.4
    auto* l4 = RED4ext::CGlobalFunction::Create("UTPPSetMMSpeedBands", "UTPPSetMMSpeedBands", &Native_SetMMSpeedBands);
    l4->flags = flags;
    l4->AddParam("Bool", "on");
    rtti->RegisterFunction(l4);
    // 0.5.6
    auto* l5 = RED4ext::CGlobalFunction::Create("UTPPSetMMGateDebounce", "UTPPSetMMGateDebounce", &Native_SetMMGateDebounce);
    l5->flags = flags;
    l5->AddParam("Float", "seconds");
    l5->SetReturnType("Bool");
    rtti->RegisterFunction(l5);
    auto* l6 = RED4ext::CGlobalFunction::Create("UTPPSetMMCrouchRules", "UTPPSetMMCrouchRules", &Native_SetMMCrouchRules);
    l6->flags = flags;
    l6->AddParam("Bool", "on");
    rtti->RegisterFunction(l6);
    // 0.5.8 float switches
    auto* l7 = RED4ext::CGlobalFunction::Create("UTPPSetMMSpeedCap", "UTPPSetMMSpeedCap", &Native_SetMMSpeedCap);
    l7->flags = flags;
    l7->AddParam("Float", "factor");
    l7->SetReturnType("Bool");
    rtti->RegisterFunction(l7);
    auto* l8 = RED4ext::CGlobalFunction::Create("UTPPSetMMCostCap", "UTPPSetMMCostCap", &Native_SetMMCostCap);
    l8->flags = flags;
    l8->AddParam("Float", "cost");
    l8->SetReturnType("Bool");
    rtti->RegisterFunction(l8);
    // 0.5.7
    {
        struct Bool057
        {
            const char* name;
            RED4ext::ScriptingFunction_t<void*> fn;
        };
        const Bool057 fns[] = {{"UTPPSetMMCrouchDirTag", &Native_SetMMCrouchDirTag},
                               {"UTPPSetMMStanceOnce", &Native_SetMMStanceOnce},
                               {"UTPPSetMMTurnCommit", &Native_SetMMTurnCommit},
                               {"UTPPSetMMStandIdle", &Native_SetMMStandIdle},
                               {"UTPPSetMMStartTurn", &Native_SetMMStartTurn},
                               // 0.5.8
                               {"UTPPSetMMStanceCap", &Native_SetMMStanceCap},
                               {"UTPPSetMMTurnSettle", &Native_SetMMTurnSettle},
                               {"UTPPSetMMLoopBands", &Native_SetMMLoopBands},
                               {"UTPPSetMMStartBind", &Native_SetMMStartBind},
                               // 0.5.9
                               {"UTPPSetMMCapsuleSync", &Native_SetMMCapsuleSync},
                               {"UTPPSetMMSwapReset", &Native_SetMMSwapReset}, // b5 testorder repair
                               {"UTPPSetMMOffsetCatchup", &Native_SetMMOffsetCatchup},
                               {"UTPPSetMMTrajCal", &Native_SetMMTrajCal},
                               // 0.6.0
                               {"UTPPSetMMMeleeGate", &Native_SetMMMeleeGate},
                               {"UTPPSetMMMountCut", &Native_SetMMMountCut},
                               // 0.6.2
                               {"UTPPSetMMUpperMask", &Native_SetMMUpperMask},
                               {"UTPPSetMMAimUpperFade", &Native_SetMMAimUpperFade},
                               {"UTPPSetMMWeaponUpper", &Native_SetMMWeaponUpper},
                               {"UTPPSetMotionGrip", &Native_SetMotionGrip},
                                {"UTPPSetMotionShootTransfer", &Native_SetMotionShootTransfer},
                               {"UTPPSetMotionMagTap", &Native_SetMotionMagTap}, // declared in scripts/utpp.reds
                               {"UTPPSetMotionAimGameUpper", &Native_SetMotionAimGameUpper},
                               {"UTPPSetMMHoldPose", &Native_SetMMHoldPose}};
        for (const Bool057& b : fns)
        {
            auto* f = RED4ext::CGlobalFunction::Create(b.name, b.name, b.fn);
            f->flags = flags;
            f->AddParam("Bool", "on");
            rtti->RegisterFunction(f);
        }
    }

    // 0.5.9
    {
        auto* c1 = RED4ext::CGlobalFunction::Create("UTPPSetMMOffsetCatchupParams", "UTPPSetMMOffsetCatchupParams",
                                                    &Native_SetMMOffsetCatchupParams);
        c1->flags = flags;
        for (const char* p : {"maxDistance", "catchupTime", "softDistance", "catchupHalflife"})
            c1->AddParam("Float", p);
        c1->SetReturnType("Bool");
        rtti->RegisterFunction(c1);
        auto* c2 = RED4ext::CGlobalFunction::Create("UTPPSetMMFootSlipParams", "UTPPSetMMFootSlipParams", &Native_SetMMFootSlipParams);
        c2->flags = flags;
        c2->AddParam("Float", "slipStart");
        c2->AddParam("Float", "slipHalflife");
        c2->SetReturnType("Bool");
        rtti->RegisterFunction(c2);
        auto* c3 = RED4ext::CGlobalFunction::Create("UTPPMMDecelTarget", "UTPPMMDecelTarget", &Native_MMDecelTarget);
        c3->flags = flags;
        c3->AddParam("Float", "baseDecel");
        c3->AddParam("Float", "appliedMul");
        c3->SetReturnType("Float");
        rtti->RegisterFunction(c3);
    }

    // 0.6.0
    {
        auto* f060a = RED4ext::CGlobalFunction::Create("UTPPSetMMStanceClipMinSpeed", "UTPPSetMMStanceClipMinSpeed",
                                                    &Native_SetMMStanceClipMinSpeed);
        f060a->flags = flags;
        f060a->AddParam("Float", "mps");
        f060a->SetReturnType("Bool");
        rtti->RegisterFunction(f060a);
        auto* f060b = RED4ext::CGlobalFunction::Create("UTPPMMMountEvent", "UTPPMMMountEvent", &Native_MMMountEvent);
        f060b->flags = flags;
        rtti->RegisterFunction(f060b);
        auto* f060bk = RED4ext::CGlobalFunction::Create("UTPPMMMountEventKind", "UTPPMMMountEventKind", &Native_MMMountEventKind);
        f060bk->flags = flags;
        f060bk->AddParam("Bool", "isCar");
        rtti->RegisterFunction(f060bk);
    }

    // 0.6.2
    {
        auto* f062a = RED4ext::CGlobalFunction::Create("UTPPSetMMUpperMaskStates", "UTPPSetMMUpperMaskStates",
                                                    &Native_SetMMUpperMaskStates);
        f062a->flags = flags;
        f062a->AddParam("Int32", "bits");
        f062a->SetReturnType("Bool");
        rtti->RegisterFunction(f062a);
    }

    // 0.6.3 aim offset
    {
        auto* a063a = RED4ext::CGlobalFunction::Create("UTPPSetAimTarget", "UTPPSetAimTarget", &Native_SetAimTarget);
        a063a->flags = flags;
        a063a->AddParam("Vector4", "camPos");
        a063a->AddParam("Vector4", "camFwd");
        a063a->AddParam("Bool", "enabled");
        rtti->RegisterFunction(a063a);
        auto*cameraTarget =
            RED4ext::CGlobalFunction::Create("UTPPSetCameraTarget", "UTPPSetCameraTarget", &Native_SetCameraTarget);
        cameraTarget->flags = flags;
        cameraTarget->AddParam("Vector4", "camPos");
        cameraTarget->AddParam("Vector4", "camFwd");
        cameraTarget->AddParam("Bool", "enabled");
        rtti->RegisterFunction(cameraTarget);
        auto* a063b = RED4ext::CGlobalFunction::Create("UTPPSetAimOffsetParams", "UTPPSetAimOffsetParams",
                                                    &Native_SetAimOffsetParams);
        a063b->flags = flags;
        a063b->AddParam("Float", "maxYawDeg");
        a063b->AddParam("Float", "maxPitchDeg");
        a063b->AddParam("Float", "smoothHz");
        a063b->SetReturnType("Bool");
        rtti->RegisterFunction(a063b);
        auto* a063c = RED4ext::CGlobalFunction::Create("UTPPSetAimOffsetAxis", "UTPPSetAimOffsetAxis", &Native_SetAimOffsetAxis);
        a063c->flags = flags;
        a063c->AddParam("Int32", "axis");
        a063c->SetReturnType("Bool");
        rtti->RegisterFunction(a063c);
        auto* a063d = RED4ext::CGlobalFunction::Create("UTPPAimOffsetStatus", "UTPPAimOffsetStatus", &Native_AimOffsetStatus);
        a063d->flags = flags;
        a063d->SetReturnType("String");
        rtti->RegisterFunction(a063d);
    }

    // melee full body + melee aim tilt (2026-11-09)
    {
        auto* mfb = RED4ext::CGlobalFunction::Create("UTPPSetMeleeFullBody", "UTPPSetMeleeFullBody", &Native_SetMeleeFullBody);
        mfb->flags = flags;
        mfb->AddParam("Bool", "enabled");
        rtti->RegisterFunction(mfb);
        auto* mfs = RED4ext::CGlobalFunction::Create("UTPPSetMeleeFullBodySetting", "UTPPSetMeleeFullBodySetting",
                                                     &Native_SetMeleeFullBodySetting);
        mfs->flags = flags;
        mfs->AddParam("Bool", "on");
        rtti->RegisterFunction(mfs);
        auto* mfp = RED4ext::CGlobalFunction::Create("UTPPSetMeleeFullPoseSetting", "UTPPSetMeleeFullPoseSetting",
                                                     &Native_SetMeleeFullPoseSetting);
        mfp->flags = flags;
        mfp->AddParam("Bool", "on");
        rtti->RegisterFunction(mfp);
        auto* mpw = RED4ext::CGlobalFunction::Create("UTPPSetMeleePoseWeapon", "UTPPSetMeleePoseWeapon", &Native_SetMeleePoseWeapon);
        mpw->flags = flags;
        mpw->AddParam("CName", "variable");
        rtti->RegisterFunction(mpw);
        auto* map = RED4ext::CGlobalFunction::Create("UTPPSetMeleeAimParams", "UTPPSetMeleeAimParams", &Native_SetMeleeAimParams);
        map->flags = flags;
        map->AddParam("Float", "maxUpDeg");
        map->AddParam("Float", "maxDownDeg");
        map->AddParam("Float", "smoothHz");
        map->SetReturnType("Bool");
        rtti->RegisterFunction(map);
        auto* mst = RED4ext::CGlobalFunction::Create("UTPPMeleeStatus", "UTPPMeleeStatus", &Native_MeleeStatus);
        mst->flags = flags;
        mst->SetReturnType("String");
        rtti->RegisterFunction(mst);
        // round 3 (2026-11-09): full-body allowed states, guard, melee aim target
        auto* mfst = RED4ext::CGlobalFunction::Create("UTPPSetMeleeFullBodyStates", "UTPPSetMeleeFullBodyStates",
                                                      &Native_SetMeleeFullBodyStates);
        mfst->flags = flags;
        mfst->AddParam("Int32", "mask");
        mfst->SetReturnType("Bool");
        rtti->RegisterFunction(mfst);
        auto* mg = RED4ext::CGlobalFunction::Create("UTPPSetMeleeGuard", "UTPPSetMeleeGuard", &Native_SetMeleeGuard);
        mg->flags = flags;
        mg->AddParam("Bool", "enabled");
        rtti->RegisterFunction(mg);
        auto* mgs = RED4ext::CGlobalFunction::Create("UTPPSetMeleeGuardSetting", "UTPPSetMeleeGuardSetting",
                                                     &Native_SetMeleeGuardSetting);
        mgs->flags = flags;
        mgs->AddParam("Bool", "on");
        rtti->RegisterFunction(mgs);
        auto* mgf = RED4ext::CGlobalFunction::Create("UTPPSetMeleeGuardFullSetting", "UTPPSetMeleeGuardFullSetting",
                                                     &Native_SetMeleeGuardFullSetting);
        mgf->flags = flags;
        mgf->AddParam("Bool", "on");
        rtti->RegisterFunction(mgf);
        auto* mgt = RED4ext::CGlobalFunction::Create("UTPPSetMeleeGuardAimTiltSetting", "UTPPSetMeleeGuardAimTiltSetting",
                                                     &Native_SetMeleeGuardAimTiltSetting);
        mgt->flags = flags;
        mgt->AddParam("Bool", "on");
        rtti->RegisterFunction(mgt);
        auto* mat = RED4ext::CGlobalFunction::Create("UTPPSetMeleeAimTarget", "UTPPSetMeleeAimTarget", &Native_SetMeleeAimTarget);
        mat->flags = flags;
        mat->AddParam("Vector4", "feet");
        mat->AddParam("Bool", "valid");
        rtti->RegisterFunction(mat);
        // head look layer: the decision of utpp_core (target, weight, mode, chest, maxYaw) and its status line
        auto* hls = RED4ext::CGlobalFunction::Create("UTPPSetHeadLook", "UTPPSetHeadLook", &Native_SetHeadLook);
        hls->flags = flags;
        hls->AddParam("Vector4", "target");
        hls->AddParam("Float", "weight");
        hls->AddParam("Int32", "mode");
        hls->AddParam("Float", "chest");
        hls->AddParam("Float", "maxYawDeg");
        hls->SetReturnType("Bool");
        rtti->RegisterFunction(hls);
        auto* hlst = RED4ext::CGlobalFunction::Create("UTPPHeadLookStatus", "UTPPHeadLookStatus", &Native_HeadLookStatus);
        hlst->flags = flags;
        hlst->SetReturnType("String");
        rtti->RegisterFunction(hlst);
    }

    // 0.6.4 turn rate
    {
        auto* t064a = RED4ext::CGlobalFunction::Create("UTPPSetMMTurnRate", "UTPPSetMMTurnRate", &Native_SetMMTurnRate);
        t064a->flags = flags;
        t064a->AddParam("Bool", "on");
        rtti->RegisterFunction(t064a);
        auto* t064b = RED4ext::CGlobalFunction::Create("UTPPSetMMTurnRateParams", "UTPPSetMMTurnRateParams",
                                                       &Native_SetMMTurnRateParams);
        t064b->flags = flags;
        t064b->AddParam("Float", "min");
        t064b->AddParam("Float", "max");
        t064b->SetReturnType("Bool");
        rtti->RegisterFunction(t064b);
        auto* t064c = RED4ext::CGlobalFunction::Create("UTPPMMTurnRateStatus", "UTPPMMTurnRateStatus", &Native_MMTurnRateStatus);
        t064c->flags = flags;
        t064c->SetReturnType("String");
        rtti->RegisterFunction(t064c);
    }

    // 0.6.4 item render plane
    {
        auto* p064a = RED4ext::CGlobalFunction::Create("UTPPSetItemRenderPlane", "UTPPSetItemRenderPlane",
                                                       &Native_SetItemRenderPlane);
        p064a->flags = flags;
        p064a->AddParam("handle:gameItemObject", "item");
        p064a->AddParam("Int32", "plane");
        p064a->SetReturnType("Bool");
        rtti->RegisterFunction(p064a);
        auto* p064b = RED4ext::CGlobalFunction::Create("UTPPSetItemPlaneOverride", "UTPPSetItemPlaneOverride",
                                                       &Native_SetItemPlaneOverride);
        p064b->flags = flags;
        p064b->AddParam("Bool", "on");
        p064b->SetReturnType("Bool");
        rtti->RegisterFunction(p064b);
        auto* p064c = RED4ext::CGlobalFunction::Create("UTPPItemPlaneNativeStatus", "UTPPItemPlaneNativeStatus",
                                                       &Native_ItemPlaneNativeStatus);
        p064c->flags = flags;
        p064c->SetReturnType("String");
        rtti->RegisterFunction(p064c);
        auto* p064d = RED4ext::CGlobalFunction::Create("UTPPItemPlaneReapply", "UTPPItemPlaneReapply", &Native_ItemPlaneReapply);
        p064d->flags = flags;
        p064d->SetReturnType("Int32");
        rtti->RegisterFunction(p064d);
        auto* p064e = RED4ext::CGlobalFunction::Create("UTPPItemPlaneDump", "UTPPItemPlaneDump", &Native_ItemPlaneDump);
        p064e->flags = flags;
        p064e->AddParam("Int32", "max");
        p064e->SetReturnType("String");
        rtti->RegisterFunction(p064e);
        auto* p064f = RED4ext::CGlobalFunction::Create("UTPPItemPlaneDumpPlayer", "UTPPItemPlaneDumpPlayer",
                                                       &Native_ItemPlaneDumpPlayer);
        p064f->flags = flags;
        p064f->AddParam("Int32", "max");
        p064f->SetReturnType("String");
        rtti->RegisterFunction(p064f);
    }

    utpp::tpprepguard::RegisterNatives(); // vehicle exit guard of the TPP representation (world/TppRepGuard.cpp)
    utpp::ragdollslot::RegisterNatives(); // ragdoll R1f probe: read-only pose slot dump (ragdoll/RagdollSlotProbe.cpp)

    // 0.6.5 game look write
    {
        auto* g065a = RED4ext::CGlobalFunction::Create("UTPPSetGameLook", "UTPPSetGameLook", &Native_SetGameLook);
        g065a->flags = flags;
        g065a->AddParam("Float", "worldYawDeg");
        g065a->AddParam("Float", "pitchDeg");
        g065a->AddParam("Float", "entityYawAheadDeg", false, true); // 0.6.5b, optional (2-argument callers keep working)
        g065a->SetReturnType("Bool");
        rtti->RegisterFunction(g065a);
        auto* g065b = RED4ext::CGlobalFunction::Create("UTPPStopGameLook", "UTPPStopGameLook", &Native_StopGameLook);
        g065b->flags = flags;
        g065b->SetReturnType("Bool");
        rtti->RegisterFunction(g065b);
        auto* g065c = RED4ext::CGlobalFunction::Create("UTPPGameLookStatus", "UTPPGameLookStatus", &Native_GameLookStatus);
        g065c->flags = flags;
        g065c->SetReturnType("String");
        rtti->RegisterFunction(g065c);
        // 2026-10-28 vehicle exit look zeroing (scripts/utpp_gamelook_exit.reds)
        auto* g065d =
            RED4ext::CGlobalFunction::Create("UTPPGameLookVehicleExit", "UTPPGameLookVehicleExit", &Native_GameLookVehicleExit);
        g065d->flags = flags;
        g065d->AddParam("Bool", "utppTpp");
        g065d->AddParam("Int32", "rideTpp");
        g065d->SetReturnType("Bool");
        rtti->RegisterFunction(g065d);
    }

    // loco P0 derived class data recompute
    {
        auto* lp0 = RED4ext::CGlobalFunction::Create("UTPPSetMMDerivedRecompute", "UTPPSetMMDerivedRecompute",
                                                     &Native_SetMMDerivedRecompute);
        lp0->flags = flags;
        lp0->AddParam("Bool", "on");
        rtti->RegisterFunction(lp0);
    }

    // loco C0 feet diagnostics / C1a crouch idle curation
    {
        auto* lc0 = RED4ext::CGlobalFunction::Create("UTPPSetMMFeetDiag", "UTPPSetMMFeetDiag", &Native_SetMMFeetDiag);
        lc0->flags = flags;
        lc0->AddParam("Bool", "on");
        rtti->RegisterFunction(lc0);
        auto* lc1 = RED4ext::CGlobalFunction::Create("UTPPSetMMCrouchIdleExclude", "UTPPSetMMCrouchIdleExclude",
                                                     &Native_SetMMCrouchIdleExclude);
        lc1->flags = flags;
        lc1->AddParam("Bool", "on");
        rtti->RegisterFunction(lc1);
        auto* lc2 = RED4ext::CGlobalFunction::Create("UTPPSetMMCrouchExcludeTags", "UTPPSetMMCrouchExcludeTags",
                                                     &Native_SetMMCrouchExcludeTags);
        lc2->flags = flags;
        lc2->AddParam("String", "tags");
        lc2->SetReturnType("Bool");
        rtti->RegisterFunction(lc2);
    }

    // stage standstill idle
    {
        auto* ih = RED4ext::CGlobalFunction::Create("UTPPSetMMIdleHold", "UTPPSetMMIdleHold", &Native_SetMMIdleHold);
        ih->flags = flags;
        ih->AddParam("Bool", "on");
        rtti->RegisterFunction(ih);
    }

    // loco C1b plant / replant
    {
        auto* lr0 = RED4ext::CGlobalFunction::Create("UTPPSetMMFootReplant", "UTPPSetMMFootReplant", &Native_SetMMFootReplant);
        lr0->flags = flags;
        lr0->AddParam("Int32", "mask");
        lr0->SetReturnType("Bool");
        rtti->RegisterFunction(lr0);
        auto* lr1 = RED4ext::CGlobalFunction::Create("UTPPSetMMFootReplantParams", "UTPPSetMMFootReplantParams",
                                                     &Native_SetMMFootReplantParams);
        lr1->flags = flags;
        lr1->AddParam("Float", "radius");
        lr1->AddParam("Float", "stopsRadius");
        lr1->AddParam("Float", "ratio");
        lr1->AddParam("Float", "stopsRatio");
        lr1->AddParam("Float", "stepSpeed");
        lr1->AddParam("Float", "stopsStepSpeed");
        lr1->AddParam("Float", "lift");
        lr1->SetReturnType("Bool");
        rtti->RegisterFunction(lr1);
    }
    // batch 2 knee fix
    {
        auto* kf = RED4ext::CGlobalFunction::Create("UTPPSetMMKneeFix", "UTPPSetMMKneeFix", &Native_SetMMKneeFix);
        kf->flags = flags;
        kf->AddParam("Int32", "mask");
        kf->SetReturnType("Bool");
        rtti->RegisterFunction(kf);
    }

    // opt-in head lead
    {
        auto* hl = RED4ext::CGlobalFunction::Create("UTPPSetMMHeadLead", "UTPPSetMMHeadLead", &Native_SetMMHeadLead);
        hl->flags = flags;
        hl->AddParam("Bool", "on");
        hl->SetReturnType("Bool");
        rtti->RegisterFunction(hl);
        auto* hp = RED4ext::CGlobalFunction::Create("UTPPSetMMHeadLeadParams", "UTPPSetMMHeadLeadParams", &Native_SetMMHeadLeadParams);
        hp->flags = flags;
        hp->AddParam("Float", "leadDeg");
        hp->AddParam("Float", "maxDeg");
        hp->SetReturnType("Bool");
        rtti->RegisterFunction(hp);
    }

    // feminine shoulder narrowing
    {
        auto* sn = RED4ext::CGlobalFunction::Create("UTPPSetMotionShoulderNarrow", "UTPPSetMotionShoulderNarrow", &Native_SetMotionShoulderNarrow);
        sn->flags = flags;
        sn->AddParam("Float", "cm");
        sn->SetReturnType("Bool");
        rtti->RegisterFunction(sn);
    }

    // run pelvis yaw trim
    {
        auto* pt = RED4ext::CGlobalFunction::Create("UTPPSetMotionPelvisYawTrim", "UTPPSetMotionPelvisYawTrim", &Native_SetMotionPelvisYawTrim);
        pt->flags = flags;
        pt->AddParam("Float", "deg");
        pt->SetReturnType("Bool");
        rtti->RegisterFunction(pt);
    }

    // body yaw offset
    {
        auto* bo = RED4ext::CGlobalFunction::Create("UTPPSetMotionBodyYawOffset", "UTPPSetMotionBodyYawOffset", &Native_SetMotionBodyYawOffset);
        bo->flags = flags;
        bo->AddParam("Float", "deg");
        bo->SetReturnType("Bool");
        rtti->RegisterFunction(bo);
    }

    // v2 barrel pitch loop + muzzle length
    {
        auto* bl = RED4ext::CGlobalFunction::Create("UTPPSetMotionBarrelLoop", "UTPPSetMotionBarrelLoop", &Native_SetMotionBarrelLoop);
        bl->flags = flags;
        bl->AddParam("Bool", "on");
        bl->SetReturnType("Bool");
        rtti->RegisterFunction(bl);
        auto* bm = RED4ext::CGlobalFunction::Create("UTPPSetMotionBarrelMuzzle", "UTPPSetMotionBarrelMuzzle", &Native_SetMotionBarrelMuzzle);
        bm->flags = flags;
        bm->AddParam("Float", "len");
        bm->SetReturnType("Bool");
        rtti->RegisterFunction(bm);
    }

    // v2 slide feed
    {
        auto* sl = RED4ext::CGlobalFunction::Create("UTPPSetMotionSlide", "UTPPSetMotionSlide", &Native_SetMotionSlide);
        sl->flags = flags;
        sl->AddParam("Bool", "on");
        sl->SetReturnType("Bool");
        rtti->RegisterFunction(sl);
    }

    // magazine top-up gesture (declaration: scripts/utpp_motion.reds UTPPFeedMotionMagTap; keep them in sync - a declared native without a
    // registration breaks the scripts)
    {
        auto* mt = RED4ext::CGlobalFunction::Create("UTPPFeedMotionMagTap", "UTPPFeedMotionMagTap", &Native_FeedMotionMagTap);
        mt->flags = flags;
        mt->SetReturnType("Bool");
        rtti->RegisterFunction(mt);
    }

    // Shooter mode + H2 safeguard (declarations: scripts/utpp_motion.reds UTPPSetMotionShooterMode / UTPPSetMotionH2Guard / UTPPMotionShooterModeLive,
    // scripts/utpp.reds UTPPMotionShooterModeSpeedMul; keep them in sync - a declared native without a registration breaks the scripts)
    {
        auto* fn = RED4ext::CGlobalFunction::Create("UTPPSetMotionShooterMode", "UTPPSetMotionShooterMode", &Native_SetMotionShooterMode);
        fn->flags = flags;
        fn->AddParam("Bool", "on");
        fn->SetReturnType("Bool");
        rtti->RegisterFunction(fn);
        auto* hg = RED4ext::CGlobalFunction::Create("UTPPSetMotionH2Guard", "UTPPSetMotionH2Guard", &Native_SetMotionH2Guard);
        hg->flags = flags;
        hg->AddParam("Bool", "on");
        hg->SetReturnType("Bool");
        rtti->RegisterFunction(hg);
        auto* hl = RED4ext::CGlobalFunction::Create("UTPPSetMotionHipLease", "UTPPSetMotionHipLease", &Native_SetMotionHipLease);
        hl->flags = flags;
        hl->AddParam("Bool", "on");
        hl->SetReturnType("Bool");
        rtti->RegisterFunction(hl);
        auto* sm = RED4ext::CGlobalFunction::Create("UTPPMotionShooterModeSpeedMul", "UTPPMotionShooterModeSpeedMul", &Native_MotionShooterModeSpeedMul);
        sm->flags = flags;
        sm->AddParam("Float", "moveX");
        sm->AddParam("Float", "moveY");
        sm->AddParam("Int32", "detailed");
        sm->SetReturnType("Float");
        rtti->RegisterFunction(sm);
        auto* fl = RED4ext::CGlobalFunction::Create("UTPPMotionShooterModeLive", "UTPPMotionShooterModeLive", &Native_MotionShooterModeLive);
        fl->flags = flags;
        fl->SetReturnType("Bool");
        rtti->RegisterFunction(fl);
    }

    // Walk toggle speed + Tier2 Slow flag (declarations: scripts/utpp.reds UTPPSetMotionWalkSpeed / UTPPMotionWalkSpeedTarget / UTPPFeedMotionTier2Slow;
    // keep them in sync - a declared native without a registration breaks the scripts)
    {
        auto* ws = RED4ext::CGlobalFunction::Create("UTPPSetMotionWalkSpeed", "UTPPSetMotionWalkSpeed", &Native_SetMotionWalkSpeed);
        ws->flags = flags;
        ws->AddParam("Float", "metersPerSecond");
        ws->SetReturnType("Bool");
        rtti->RegisterFunction(ws);
        auto* wt = RED4ext::CGlobalFunction::Create("UTPPMotionWalkSpeedTarget", "UTPPMotionWalkSpeedTarget", &Native_MotionWalkSpeedTarget);
        wt->flags = flags;
        wt->SetReturnType("Float");
        rtti->RegisterFunction(wt);
        auto* t2 = RED4ext::CGlobalFunction::Create("UTPPFeedMotionTier2Slow", "UTPPFeedMotionTier2Slow", &Native_FeedMotionTier2Slow);
        t2->flags = flags;
        t2->AddParam("Bool", "slow");
        rtti->RegisterFunction(t2);
    }

    // Minimum analog walk speed (declarations: scripts/utpp.reds UTPPSetMotionMinAnalogWalk / UTPPMotionAnalogFloorMul; keep them in sync - a
    // declared native without a registration breaks the scripts)
    {
        auto* mw = RED4ext::CGlobalFunction::Create("UTPPSetMotionMinAnalogWalk", "UTPPSetMotionMinAnalogWalk", &Native_SetMotionMinAnalogWalk);
        mw->flags = flags;
        mw->AddParam("Float", "metersPerSecond");
        mw->SetReturnType("Bool");
        rtti->RegisterFunction(mw);
        auto* af = RED4ext::CGlobalFunction::Create("UTPPMotionAnalogFloorMul", "UTPPMotionAnalogFloorMul", &Native_MotionAnalogFloorMul);
        af->flags = flags;
        af->AddParam("Float", "moveX");
        af->AddParam("Float", "moveY");
        af->AddParam("Int32", "detailed");
        af->AddParam("Bool", "walk");
        af->SetReturnType("Float");
        rtti->RegisterFunction(af);
    }

    // v2 slide camera (read-only getter; keep the declaration in scripts/utpp_motion.reds in sync)
    {
        auto* so = RED4ext::CGlobalFunction::Create("UTPPSlideOwned", "UTPPSlideOwned", &Native_SlideOwned);
        so->flags = flags;
        so->SetReturnType("Bool");
        rtti->RegisterFunction(so);
    }

    // v2 orientation warp
    {
        auto* ow = RED4ext::CGlobalFunction::Create("UTPPSetMotionOrientWarp", "UTPPSetMotionOrientWarp", &Native_SetMotionOrientWarp);
        ow->flags = flags;
        ow->AddParam("Bool", "on");
        ow->SetReturnType("Bool");
        rtti->RegisterFunction(ow);
    }

    // v2 selection churn guards as a group
    {
        auto* sg = RED4ext::CGlobalFunction::Create("UTPPSetMotionSelectionGuards", "UTPPSetMotionSelectionGuards", &Native_SetMotionSelectionGuards);
        sg->flags = flags;
        sg->AddParam("Bool", "on");
        sg->SetReturnType("Bool");
        rtti->RegisterFunction(sg);
    }

    // skin helper recompute on the weight-0 writers' frames
    {
        auto* hz = RED4ext::CGlobalFunction::Create("UTPPSetHelperZeroWeightWriters", "UTPPSetHelperZeroWeightWriters", &Native_SetHelperZeroWeightWriters);
        hz->flags = flags;
        hz->AddParam("Bool", "on");
        hz->SetReturnType("Bool");
        rtti->RegisterFunction(hz);
    }

    // melee full pose: switch blend at a clip switch
    {
        auto* sb = RED4ext::CGlobalFunction::Create("UTPPSetMeleeSwitchBlend", "UTPPSetMeleeSwitchBlend", &Native_SetMeleeSwitchBlend);
        sb->flags = flags;
        sb->AddParam("Bool", "on");
        sb->SetReturnType("Bool");
        rtti->RegisterFunction(sb);
    }

    // v2 motion lean
    {
        auto* ln = RED4ext::CGlobalFunction::Create("UTPPSetMotionLean", "UTPPSetMotionLean", &Native_SetMotionLean);
        ln->flags = flags;
        ln->AddParam("Bool", "on");
        ln->SetReturnType("Bool");
        rtti->RegisterFunction(ln);
        auto* lh = RED4ext::CGlobalFunction::Create("UTPPSetMotionLeanHead", "UTPPSetMotionLeanHead", &Native_SetMotionLeanHead);
        lh->flags = flags;
        lh->AddParam("Bool", "on");
        lh->SetReturnType("Bool");
        rtti->RegisterFunction(lh);
        auto* lp = RED4ext::CGlobalFunction::Create("UTPPSetMotionLeanParams", "UTPPSetMotionLeanParams", &Native_SetMotionLeanParams);
        lp->flags = flags;
        lp->AddParam("Float", "scale");
        lp->AddParam("Float", "headScale");
        lp->SetReturnType("Bool");
        rtti->RegisterFunction(lp);
    }

    // B2 strafe S1
    {
        auto* sa = RED4ext::CGlobalFunction::Create("UTPPSetMMStrafe", "UTPPSetMMStrafe", &Native_SetMMStrafe);
        sa->flags = flags;
        sa->AddParam("Bool", "on");
        rtti->RegisterFunction(sa);
        auto* sb = RED4ext::CGlobalFunction::Create("UTPPSetMMStrafeParams", "UTPPSetMMStrafeParams", &Native_SetMMStrafeParams);
        sb->flags = flags;
        sb->AddParam("Bool", "unarmed");
        sb->AddParam("Float", "rootTurnMaxDeg");
        sb->AddParam("Float", "rootWarpMaxDeg");
        sb->AddParam("Float", "exitHold");
        sb->AddParam("Float", "jogF");
        sb->SetReturnType("Bool");
        rtti->RegisterFunction(sb);
    }

    // B2 strafe S4 jog while aiming
    {
        auto* aj = RED4ext::CGlobalFunction::Create("UTPPMMAimJogTarget", "UTPPMMAimJogTarget", &Native_MMAimJogTarget);
        aj->flags = flags;
        aj->AddParam("Float", "moveX");
        aj->AddParam("Float", "moveY");
        aj->SetReturnType("Float");
        rtti->RegisterFunction(aj);
    }

    // B2 strafe S2 chest align
    {
        auto* ca = RED4ext::CGlobalFunction::Create("UTPPSetMMChestAlign", "UTPPSetMMChestAlign", &Native_SetMMChestAlign);
        ca->flags = flags;
        ca->AddParam("Bool", "on");
        rtti->RegisterFunction(ca);
        auto* cb = RED4ext::CGlobalFunction::Create("UTPPSetMMChestAlignParams", "UTPPSetMMChestAlignParams", &Native_SetMMChestAlignParams);
        cb->flags = flags;
        cb->AddParam("Float", "swayKeep");
        cb->AddParam("Float", "chestMaxDeg");
        cb->SetReturnType("Bool");
        rtti->RegisterFunction(cb);
    }

    // AIR HOLD stage (2026-10-13)
    {
        auto* a0 = RED4ext::CGlobalFunction::Create("UTPPFeedAir", "UTPPFeedAir", &Native_FeedAir);
        a0->flags = flags;
        a0->AddParam("Bool", "onGround");
        a0->AddParam("Float", "distToGround");
        a0->AddParam("Int32", "fallState");
        rtti->RegisterFunction(a0);
        auto* a1 = RED4ext::CGlobalFunction::Create("UTPPMMAirWantsProbe", "UTPPMMAirWantsProbe", &Native_MMAirWantsProbe);
        a1->flags = flags;
        a1->SetReturnType("Bool");
        rtti->RegisterFunction(a1);
        auto* a2 = RED4ext::CGlobalFunction::Create("UTPPSetMMAir", "UTPPSetMMAir", &Native_SetMMAir);
        a2->flags = flags;
        a2->AddParam("Int32", "mode");
        a2->SetReturnType("Bool");
        rtti->RegisterFunction(a2);
        auto* a3 = RED4ext::CGlobalFunction::Create("UTPPSetMMAirParams", "UTPPSetMMAirParams", &Native_SetMMAirParams);
        a3->flags = flags;
        a3->AddParam("Int32", "takeoffLeadFrames");
        a3->AddParam("Float", "landLeadSeconds");
        a3->AddParam("Float", "rateMin");
        a3->AddParam("Float", "rateMax");
        a3->AddParam("Float", "recoverLightSeconds");
        a3->AddParam("Float", "recoverHeavySeconds");
        a3->AddParam("Float", "holdRate");
        a3->AddParam("Float", "heavyVz");
        a3->SetReturnType("Bool");
        rtti->RegisterFunction(a3);
        auto* a4 = RED4ext::CGlobalFunction::Create("UTPPSetMMAirProbe", "UTPPSetMMAirProbe", &Native_SetMMAirProbe);
        a4->flags = flags;
        a4->AddParam("Bool", "on");
        rtti->RegisterFunction(a4);
        auto* a5 = RED4ext::CGlobalFunction::Create("UTPPSetMMAirSteer", "UTPPSetMMAirSteer", &Native_SetMMAirSteer);
        a5->flags = flags;
        a5->AddParam("Bool", "on");
        rtti->RegisterFunction(a5);
        auto* a5r = RED4ext::CGlobalFunction::Create("UTPPSetMMAirRobust", "UTPPSetMMAirRobust", &Native_SetMMAirRobust);
        a5r->flags = flags;
        a5r->AddParam("Bool", "on");
        rtti->RegisterFunction(a5r);
        auto* a6 = RED4ext::CGlobalFunction::Create("UTPPSetMMClimbRelease", "UTPPSetMMClimbRelease", &Native_SetMMClimbRelease);
        a6->flags = flags;
        a6->AddParam("Bool", "on");
        a6->SetReturnType("Bool");
        rtti->RegisterFunction(a6);
    }

    // batch 1b feed dump + extra feed fields (walk toggle, capsule position)
    {
        auto* fd0 = RED4ext::CGlobalFunction::Create("UTPPFeedLocomotionExtra", "UTPPFeedLocomotionExtra", &Native_FeedLocomotionExtra);
        fd0->flags = flags;
        fd0->AddParam("Bool", "walkToggled");
        fd0->AddParam("Vector4", "capsulePos");
        rtti->RegisterFunction(fd0);
        auto* fd1 = RED4ext::CGlobalFunction::Create("UTPPSetMMFeedDump", "UTPPSetMMFeedDump", &Native_SetMMFeedDump);
        fd1->flags = flags;
        fd1->AddParam("Bool", "on");
        fd1->SetReturnType("Bool");
        rtti->RegisterFunction(fd1);
        auto* fd2 = RED4ext::CGlobalFunction::Create("UTPPMMFeedDumpStatus", "UTPPMMFeedDumpStatus", &Native_MMFeedDumpStatus);
        fd2->flags = flags;
        fd2->SetReturnType("String");
        rtti->RegisterFunction(fd2);
    }

    // Phase 5.3d (0.3.8)
    auto* h1 = RED4ext::CGlobalFunction::Create("UTPPLogLine", "UTPPLogLine", &Native_LogLine);
    h1->flags = flags;
    h1->AddParam("String", "line");
    rtti->RegisterFunction(h1);

    auto* h2 = RED4ext::CGlobalFunction::Create("UTPPSetMMLargeTurn", "UTPPSetMMLargeTurn", &Native_SetMMLargeTurn);
    h2->flags = flags;
    h2->AddParam("Bool", "on");
    rtti->RegisterFunction(h2);

    auto* h3 = RED4ext::CGlobalFunction::Create("UTPPSetMMPivotAccel", "UTPPSetMMPivotAccel", &Native_SetMMPivotAccel);
    h3->flags = flags;
    for (const char* p : {"standAccel", "crouchAccel", "sprintAccel"})
        h3->AddParam("Float", p);
    h3->SetReturnType("Bool");
    rtti->RegisterFunction(h3);

    auto* h4 = RED4ext::CGlobalFunction::Create("UTPPSetMMStanceParams", "UTPPSetMMStanceParams", &Native_SetMMStanceParams);
    h4->flags = flags;
    h4->AddParam("Float", "minHalfWidthCm");
    h4->AddParam("Float", "minSeparationCm");
    h4->AddParam("Bool", "kneeIk");
    h4->SetReturnType("Bool");
    rtti->RegisterFunction(h4);

    // Phase 5.3c-3 (0.3.7)
    auto* e1 = RED4ext::CGlobalFunction::Create("UTPPSetMMRootWarp", "UTPPSetMMRootWarp", &Native_SetMMRootWarp);
    e1->flags = flags;
    e1->AddParam("Bool", "on");
    e1->AddParam("Float", "maxDeg");
    e1->SetReturnType("Bool");
    rtti->RegisterFunction(e1);

    auto* e2 = RED4ext::CGlobalFunction::Create("UTPPSetMMHeadingLean", "UTPPSetMMHeadingLean", &Native_SetMMHeadingLean);
    e2->flags = flags;
    e2->AddParam("Bool", "on");
    rtti->RegisterFunction(e2);

    auto* e3 = RED4ext::CGlobalFunction::Create("UTPPSetMMStanceSwing", "UTPPSetMMStanceSwing", &Native_SetMMStanceSwing);
    e3->flags = flags;
    e3->AddParam("Bool", "on");
    rtti->RegisterFunction(e3);

    // Phase 5.3c (0.3.5)
    auto* c1 = RED4ext::CGlobalFunction::Create("UTPPSetMMSteerFollow", "UTPPSetMMSteerFollow", &Native_SetMMSteerFollow);
    c1->flags = flags;
    c1->AddParam("Bool", "on");
    rtti->RegisterFunction(c1);

    auto* c2 = RED4ext::CGlobalFunction::Create("UTPPSetMMSteerParams", "UTPPSetMMSteerParams", &Native_SetMMSteerParams);
    c2->flags = flags;
    for (const char* p : {"enterDeg", "exitDeg", "followTime", "maxFollowRateDeg"})
        c2->AddParam("Float", p);
    c2->SetReturnType("Bool");
    rtti->RegisterFunction(c2);

    auto* c3 = RED4ext::CGlobalFunction::Create("UTPPSetMMDwell", "UTPPSetMMDwell", &Native_SetMMDwell);
    c3->flags = flags;
    c3->AddParam("Bool", "on");
    c3->AddParam("Float", "minDwell");
    c3->AddParam("Float", "costRatio");
    c3->SetReturnType("Bool");
    rtti->RegisterFunction(c3);

    auto* c4 = RED4ext::CGlobalFunction::Create("UTPPSetMMCuration", "UTPPSetMMCuration", &Native_SetMMCuration);
    c4->flags = flags;
    c4->AddParam("Bool", "on");
    rtti->RegisterFunction(c4);

    auto* c5 = RED4ext::CGlobalFunction::Create("UTPPSetMMSlowWalkCapsule", "UTPPSetMMSlowWalkCapsule",
                                                &Native_SetMMSlowWalkCapsule);
    c5->flags = flags;
    c5->AddParam("Bool", "on");
    rtti->RegisterFunction(c5);

    // Phase 5.3b (0.3.4)
    auto* r1 = RED4ext::CGlobalFunction::Create("UTPPSetMMRootRate", "UTPPSetMMRootRate", &Native_SetMMRootRate);
    r1->flags = flags;
    r1->AddParam("Bool", "on");
    rtti->RegisterFunction(r1);

    auto* r2 = RED4ext::CGlobalFunction::Create("UTPPSetMMRootRateParams", "UTPPSetMMRootRateParams", &Native_SetMMRootRateParams);
    r2->flags = flags;
    for (const char* p : {"rateMin", "rateMax", "epsilon", "halflife"})
        r2->AddParam("Float", p);
    r2->SetReturnType("Bool");
    rtti->RegisterFunction(r2);

    auto* r3 = RED4ext::CGlobalFunction::Create("UTPPSetMMStopSync", "UTPPSetMMStopSync", &Native_SetMMStopSync);
    r3->flags = flags;
    r3->AddParam("Bool", "on");
    r3->AddParam("Float", "stopHalflife");
    r3->SetReturnType("Bool");
    rtti->RegisterFunction(r3);

    auto* r4 = RED4ext::CGlobalFunction::Create("UTPPSetMMSlowWalk", "UTPPSetMMSlowWalk", &Native_SetMMSlowWalk);
    r4->flags = flags;
    r4->AddParam("Bool", "on");
    r4->AddParam("Float", "walkRateMin");
    r4->SetReturnType("Bool");
    rtti->RegisterFunction(r4);

    auto* r5 = RED4ext::CGlobalFunction::Create("UTPPSetMMOffsetSaturation", "UTPPSetMMOffsetSaturation",
                                                &Native_SetMMOffsetSaturation);
    r5->flags = flags;
    r5->AddParam("Float", "seconds");
    r5->AddParam("Float", "halflife");
    r5->SetReturnType("Bool");
    rtti->RegisterFunction(r5);

    auto* r6 = RED4ext::CGlobalFunction::Create("UTPPSetMMStartReface", "UTPPSetMMStartReface", &Native_SetMMStartReface);
    r6->flags = flags;
    r6->AddParam("Bool", "on");
    rtti->RegisterFunction(r6);

    auto* r7 = RED4ext::CGlobalFunction::Create("UTPPSetMMExcludeLands", "UTPPSetMMExcludeLands", &Native_SetMMExcludeLands);
    r7->flags = flags;
    r7->AddParam("Bool", "on");
    rtti->RegisterFunction(r7);

    // Phase 5.3 (0.3.3)
    auto* o1 = RED4ext::CGlobalFunction::Create("UTPPSetMMOffsetRoot", "UTPPSetMMOffsetRoot", &Native_SetMMOffsetRoot);
    o1->flags = flags;
    o1->AddParam("Bool", "on");
    rtti->RegisterFunction(o1);

    auto* o2 = RED4ext::CGlobalFunction::Create("UTPPSetMMOffsetRootParams", "UTPPSetMMOffsetRootParams",
                                                &Native_SetMMOffsetRootParams);
    o2->flags = flags;
    for (const char* p : {"translationHalflife", "rotationHalflife", "maxDistance", "maxYawDeg"})
        o2->AddParam("Float", p);
    o2->SetReturnType("Bool");
    rtti->RegisterFunction(o2);

    auto* a1 = RED4ext::CGlobalFunction::Create("UTPPSetMMAccelMatch", "UTPPSetMMAccelMatch", &Native_SetMMAccelMatch);
    a1->flags = flags;
    a1->AddParam("Bool", "on");
    rtti->RegisterFunction(a1);

    auto* a2 = RED4ext::CGlobalFunction::Create("UTPPSetMMAccelParams", "UTPPSetMMAccelParams", &Native_SetMMAccelParams);
    a2->flags = flags;
    for (const char* p : {"standAccel", "crouchAccel", "sprintAccel"})
        a2->AddParam("Float", p);
    a2->SetReturnType("Bool");
    rtti->RegisterFunction(a2);

    auto* a3 = RED4ext::CGlobalFunction::Create("UTPPMMAccelTarget", "UTPPMMAccelTarget", &Native_MMAccelTarget);
    a3->flags = flags;
    a3->AddParam("Float", "baseAccel");
    a3->AddParam("Float", "appliedMul");
    a3->SetReturnType("Float");
    rtti->RegisterFunction(a3);

    auto* fTrace = RED4ext::CGlobalFunction::Create("UTPPSetMMTrace", "UTPPSetMMTrace", &Native_SetMMTrace);
    fTrace->flags = flags;
    fTrace->AddParam("Bool", "on");
    rtti->RegisterFunction(fTrace);

    auto* f7 = RED4ext::CGlobalFunction::Create("UTPPSetMatchMode", "UTPPSetMatchMode", &Native_SetMatchMode);
    f7->flags = flags;
    f7->AddParam("Int32", "mode");
    rtti->RegisterFunction(f7);

    auto* f6 = RED4ext::CGlobalFunction::Create("UTPPArm", "UTPPArm", &Native_Arm);
    f6->flags = flags;
    f6->SetReturnType("Int32");
    rtti->RegisterFunction(f6);

    auto* m1 = RED4ext::CGlobalFunction::Create("UTPPFeedLocomotion", "UTPPFeedLocomotion", &Native_FeedLocomotion);
    m1->flags = flags;
    m1->AddParam("Vector4", "velWorld");
    m1->AddParam("Vector2", "moveInput");
    m1->AddParam("Float", "cameraYawDeg");
    m1->AddParam("Float", "playerYawDeg");
    m1->AddParam("Int32", "locoState");
    m1->AddParam("Bool", "tppActive");
    m1->AddParam("Float", "dt");
    m1->AddParam("Bool", "haveTargetYaw");
    m1->AddParam("Float", "targetYawDeg");
    m1->AddParam("Int32", "psmBlockMask");
    rtti->RegisterFunction(m1);

    auto* g1 = RED4ext::CGlobalFunction::Create("UTPPSetMMGateFade", "UTPPSetMMGateFade", &Native_SetMMGateFade);
    g1->flags = flags;
    g1->AddParam("Float", "hardOutSeconds");
    g1->AddParam("Float", "reopenHoldSeconds");
    g1->SetReturnType("Bool");
    rtti->RegisterFunction(g1);

    auto* b1 = RED4ext::CGlobalFunction::Create("UTPPSetMMBodyTurn", "UTPPSetMMBodyTurn", &Native_SetMMBodyTurn);
    b1->flags = flags;
    b1->AddParam("Bool", "on");
    rtti->RegisterFunction(b1);

    auto* b2 = RED4ext::CGlobalFunction::Create("UTPPSetMMBodyTurnParams", "UTPPSetMMBodyTurnParams", &Native_SetMMBodyTurnParams);
    b2->flags = flags;
    for (const char* p : {"correctionTime", "minRateDeg", "maxCorrectionRateDeg", "maxRateDeg"})
        b2->AddParam("Float", p);
    b2->SetReturnType("Bool");
    rtti->RegisterFunction(b2);

    auto* b3 = RED4ext::CGlobalFunction::Create("UTPPMMBodyTurnActive", "UTPPMMBodyTurnActive", &Native_MMBodyTurnActive);
    b3->flags = flags;
    b3->SetReturnType("Bool");
    rtti->RegisterFunction(b3);

    auto* b4 = RED4ext::CGlobalFunction::Create("UTPPMMBodyYawCommand", "UTPPMMBodyYawCommand", &Native_MMBodyYawCommand);
    b4->flags = flags;
    b4->AddParam("Float", "currentYawDeg");
    b4->AddParam("Float", "baseYawDeg");
    b4->SetReturnType("Float");
    rtti->RegisterFunction(b4);

    auto* m2 = RED4ext::CGlobalFunction::Create("UTPPSetMMEnabled", "UTPPSetMMEnabled", &Native_SetMMEnabled);
    m2->flags = flags;
    m2->AddParam("Bool", "enabled");
    rtti->RegisterFunction(m2);

    auto* m3 = RED4ext::CGlobalFunction::Create("UTPPSetMMTagFilter", "UTPPSetMMTagFilter", &Native_SetMMTagFilter);
    m3->flags = flags;
    m3->AddParam("String", "filter");
    m3->SetReturnType("Bool");
    rtti->RegisterFunction(m3);

    auto* m4 = RED4ext::CGlobalFunction::Create("UTPPSetMMBlendTime", "UTPPSetMMBlendTime", &Native_SetMMBlendTime);
    m4->flags = flags;
    m4->AddParam("Float", "seconds");
    rtti->RegisterFunction(m4);

    auto* m5 = RED4ext::CGlobalFunction::Create("UTPPSetMMAllowedStates", "UTPPSetMMAllowedStates", &Native_SetMMAllowedStates);
    m5->flags = flags;
    m5->AddParam("Int32", "mask");
    rtti->RegisterFunction(m5);

    auto* m6 = RED4ext::CGlobalFunction::Create("UTPPMMWantsFeed", "UTPPMMWantsFeed", &Native_MMWantsFeed);
    m6->flags = flags;
    m6->SetReturnType("Bool");
    rtti->RegisterFunction(m6);

    auto* m7 = RED4ext::CGlobalFunction::Create("UTPPVehicleJumpOutEpoch", "UTPPVehicleJumpOutEpoch", &Native_VehicleJumpOutEpoch);
    m7->flags = flags;
    m7->SetReturnType("Int32");
    rtti->RegisterFunction(m7);

    // 0.5.1 database switching
    auto* db1 = RED4ext::CGlobalFunction::Create("UTPPListDatabases", "UTPPListDatabases", &Native_ListDatabases);
    db1->flags = flags;
    db1->SetReturnType("String");
    rtti->RegisterFunction(db1);
    auto* db2 = RED4ext::CGlobalFunction::Create("UTPPLoadDatabase", "UTPPLoadDatabase", &Native_LoadDatabase);
    db2->flags = flags;
    db2->AddParam("String", "name");
    db2->SetReturnType("Bool");
    rtti->RegisterFunction(db2);
    auto* db3 = RED4ext::CGlobalFunction::Create("UTPPGetDatabaseName", "UTPPGetDatabaseName", &Native_GetDatabaseName);
    db3->flags = flags;
    db3->SetReturnType("String");
    rtti->RegisterFunction(db3);
    auto* db4 = RED4ext::CGlobalFunction::Create("UTPPGetDatabaseSwitchStatus", "UTPPGetDatabaseSwitchStatus",
                                                 &Native_GetDatabaseSwitchStatus);
    db4->flags = flags;
    db4->SetReturnType("String");
    rtti->RegisterFunction(db4);

    // Phase 5N step 4a: Locomotion natives + input hook (src/input/Locomotion.cpp), only on a validated game.
    bool validated = false;
    {
        std::lock_guard lock(g_coldMutex);
        validated = !g_inert;
    }
    utpp::locomotion::OnPostRegister(validated);
    utpp::loothead::OnPostRegister(validated); // Phase 5N step 5: LootRange / HeadGuard natives + RTTI proof
}

RED4ext::v1::GameState g_runningState{&OnRunningEnter, &OnRunningUpdate, &OnRunningExit};

void OnLoad()
{
    pathscrub::SetGameRoot(util::WideToUtf8(util::GameRoot())); // log lines: the game folder shows as a relative path (PluginLog.hpp)
    utpp::locomotion::Init(g_sdk, g_handle); // Phase 5N step 4a: kill file
    utpp::loothead::Init(g_sdk, g_handle); // Phase 5N step 5: kill files
    auto* rtti = RED4ext::CRTTISystem::Get();
    rtti->AddRegisterCallback(&RegisterTypes);
    rtti->AddPostRegisterCallback(&PostRegisterTypes);

    // Motion matching database: <dll dir>/utpp.utppmm. Fail closed (MM unavailable), plugin otherwise unaffected.
    {
        std::wstring path;
        const std::wstring dir = util::PluginDir();
        if (!dir.empty())
        {
            motionhost::SetDatabaseDir(dir); // 0.5.1: <dll dir>\db\*.utppmm selectable at runtime (UTPPLoadDatabase)
            motionhost::SetFeedDumpDir(FeedDumpFolder(dir)); // batch 1b: next to the RED4ext UTPP logs
            g_bodyTraceFolder = FeedDumpFolder(dir);
            bodytraceruntime::Init(&BodyTraceContinuousLog, g_bodyTraceFolder);
            posecaptureruntime::Init(&BodyTraceContinuousLog, g_bodyTraceFolder);
            LoadEarlyPoseExperiment(dir);
            const bool helperLoaded = bodyhelperruntime::Load(dir);
            LogInfo("UTPP skin helpers load: %d %s", helperLoaded ? 1 : 0, bodyhelperruntime::Status().c_str());
            utpp::meleepose::Load(dir + L"data\\"); // melee full pose tables: <dll dir>\data\utpp_melee_pose_female.bin / _male.bin (fail closed)
            LogInfo("UTPP melee full pose: %s", utpp::meleepose::Status().c_str());
            utpp::motiondriver::Install(dir); // UTPP-MOTION SEAM: <dll dir>\data\ data, hooks (mm.motionDriver stays off until UTPPSetMotionEnabled)
            utpp::motiondriver::SetLocomotionReasonsProbe(&utpp::locomotion::CurrentReasons); // H2 safeguard: why the native body turn is off (read only)
            utpp::motiondriver::SetDodgeHeadingProbe(&utpp::locomotion::LastDodgeHeading); // dodge push diagnostics (read only)
            utpp::motiondriver::SetLogger(&MMTraceLog); // UTPP-MOTION SEAM: v2 fallback reasons / fault cause, once each
            path = dir + L"data\\utpp_base.utppmm";
        }
        motionhost::SetLogger(&MMTraceLog); // 0.5.2: speed calibration / database switch lines (trace stays off)
        motionhost::SetMeleeAimLogSource(&utpp::meleeaim::FormatLogLine); // 'UTPP MELEEAIM' (trace only, 4 Hz)
        motionhost::SetMeleePoseLogSource(&utpp::meleepose::FormatLogLine); // 'UTPP MELEEPOSE' (trace only, 1 Hz)
        motionhost::SetMeleePoseStateSink(&utpp::meleepose::SetMeleeState); // melee full pose: PSM melee weapon state
        const bool dbOk = !path.empty() && motionhost::LoadDatabase(path);
        const std::string st = motionhost::Status();
        if (dbOk)
            LogInfo("UTPP MM database loaded:%s", st.substr(0, 400).c_str());
        else
            LogWarn("UTPP MM unavailable (database not loaded):%s", st.substr(0, 400).c_str());
    }

    std::string reason;
    const bool valid = ValidateGame(reason);
    std::lock_guard lock(g_coldMutex);
    if (!valid)
    {
        g_inert = true;
        g_reason = reason;
        LogError("UTPP inert: %s. FK hook NOT installed; natives report inert.", reason.c_str());
        return;
    }

    // 0.6.4: TPP weapon render plane (render/ItemPlane.cpp); own prologue checks, fail-closed, independent of the FK hook.
    utpp::itemplane::Install(g_sdk, g_handle);
    // 0.6.5: game look write (input/GameLook.cpp); own code-site checks, fail-closed, independent of the FK hook.
    utpp::gamelook::Install(g_sdk, g_handle);
    // Vehicle exit guard of the TPP representation (world/TppRepGuard.cpp); own code-site checks, fail-closed.
    utpp::tpprepguard::Install(g_sdk, g_handle);
    // Ragdoll R1f probe (ragdoll/RagdollSlotProbe.cpp): read-only hook, own prologue check, fail-closed.
    utpp::ragdollslot::Install(g_sdk, g_handle);

    auto* base = reinterpret_cast<uint8_t*>(GetModuleHandleW(nullptr));
    g_fkTarget = base + kRvaFK;
    if (!g_sdk->hooking ||
        !g_sdk->hooking->Attach(g_handle, g_fkTarget, reinterpret_cast<void*>(&FkDetour),
                                reinterpret_cast<void**>(FkOriginalSlot())) ||
        !*FkOriginalSlot())
    {
        g_inert = true;
        g_reason = "hooking->Attach failed";
        g_fkTarget = nullptr;
        LogError("UTPP inert: hooking->Attach on FK @0x582B3C failed");
        return;
    }
    g_hooked = true;
    // Validate() already checked the mapping prologue. This optional hook only observes records.
    g_mappingTarget = base + kRvaMapping;
    if (g_sdk->hooking->Attach(g_handle, g_mappingTarget, reinterpret_cast<void*>(&MappingDetour),
        reinterpret_cast<void**>(MappingOriginalSlot())) && *MappingOriginalSlot())
    {
        g_mappingHooked = true;
        LogInfo("UTPP read-only pose census hook attached @%p (default OFF)", g_mappingTarget);
    }
    else
    {
        g_mappingTarget = nullptr;
        LogWarn("UTPP pose census hook unavailable; FK pose correction remains available");
    }
    g_inert = false;
    g_reason = "validated 2.50 (file 3.0.88.46235), prologues and call site match";
    LogInfo("UTPP ready: FK hook attached @%p (RVA 0x582B3C)", g_fkTarget);

    if (g_sdk->gameStates && g_sdk->gameStates->Add(g_handle, RED4ext::EGameStateType::Running, &g_runningState))
        LogInfo("UTPP: Running-state auto-arm registered (<= 1 Hz)");
    else
        LogWarn("UTPP: Running-state callback not registered; use UTPPArm()");
}

void OnUnload()
{
    bodytraceruntime::Shutdown(); // continuous helper capture: stop the writer thread, drain and close the files first
    posecaptureruntime::Shutdown(); // compact capture: the same, pending block and trailer included
    utpp::locomotion::Shutdown(); // Phase 5N step 4a
    utpp::loothead::Shutdown(); // Phase 5N step 5: restore predicates / threshold
    utpp::itemplane::Shutdown(); // 0.6.4
    utpp::gamelook::Shutdown();  // 0.6.5
    utpp::tpprepguard::Shutdown();
    utpp::ragdollslot::Shutdown();
    std::lock_guard lock(g_coldMutex);
    g_hot.enabled.store(false);
    utpp::motiondriver::Uninstall(); // UTPP-MOTION SEAM
    motionhost::Shutdown();
    DisarmLocked();
    if (g_mappingHooked && g_mappingTarget && g_sdk && g_sdk->hooking)
    {
        const bool ok = g_sdk->hooking->Detach(g_handle, g_mappingTarget);
        LogInfo("UTPP unload: mapping census detach %s", ok ? "ok" : "FAILED");
        g_mappingHooked = false;
    }
    if (g_hooked && g_fkTarget && g_sdk && g_sdk->hooking)
    {
        const bool ok = g_sdk->hooking->Detach(g_handle, g_fkTarget);
        LogInfo("UTPP unload: FK hook detach %s", ok ? "ok" : "FAILED");
    }
    g_hooked = false;
}
} // namespace

RED4EXT_C_EXPORT bool RED4EXT_CALL Main(RED4ext::v1::PluginHandle aHandle, RED4ext::v1::EMainReason aReason,
                                        const RED4ext::v1::Sdk* aSdk)
{
    switch (aReason)
    {
    case RED4ext::v1::EMainReason::Load:
        g_handle = aHandle;
        g_sdk = aSdk;
        pluginlog::Bind(aSdk, aHandle); // one log sink for every module (PluginLog.hpp)
        OnLoad();
        break;
    case RED4ext::v1::EMainReason::Unload:
        OnUnload();
        break;
    }
    return true;
}

RED4EXT_C_EXPORT void RED4EXT_CALL Query(RED4ext::v1::PluginInfo* aInfo)
{
    aInfo->name = L"UTPP";
    aInfo->author = L"Daniel Maciej Pytel";
    aInfo->version = RED4EXT_V1_SEMVER(UTPP_VERSION_MAJOR, UTPP_VERSION_MINOR, UTPP_VERSION_PATCH);
    aInfo->runtime = RED4EXT_V1_RUNTIME_VERSION_2_50;
    aInfo->sdk = RED4EXT_V1_SDK_VERSION_CURRENT;
}

RED4EXT_C_EXPORT uint32_t RED4EXT_CALL Supports()
{
    return RED4EXT_API_VERSION_1;
}
