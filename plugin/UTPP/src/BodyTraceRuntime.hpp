#pragma once
// Production side of the continuous helper-bone capture (UTPPBodyPoseTraceContinuous): CSV files, writer thread, logger. The state
// machine, row formatting and the hot-path rings are in mm/BodyTraceContinuous.hpp / mm/BodyDeformerTrace.hpp (offline tested).
// Control path (Main.cpp natives, under g_coldMutex): Init once, Enable(on), PublishMap on every (re)arm of the player rig, ClearMap on
// disarm. The writer thread never touches game memory: it only drains the rings and writes files.
#include <cstdint>
#include <string>
#include <vector>

#include "mm/BodyDeformerTrace.hpp"

namespace utpp::bodytraceruntime
{
using LogFn = void (*)(const char* aLine);

// Folder = <game>\red4ext\logs (g_bodyTraceFolder). Idempotent; the logger is called from the writer thread (RED4ext's logger is spdlog).
void Init(LogFn aLog, const std::wstring& aFolder);
// Idempotent. true = the requested state holds afterwards (false when on was requested without Init / folder).
bool Enable(bool aOn);
bool Enabled();
// The map of the CURRENT player rig (same arguments as the explicit arm). An identical map is ignored; a different one starts a new file pair.
void PublishMap(const mm::bodytrace::Map& aMap, bool aPost, std::vector<uint64_t> aHashes, std::vector<int16_t> aParents);
void ClearMap();
std::string Status();
// DLL unload: stops the thread, drains and closes the files.
void Shutdown();
} // namespace utpp::bodytraceruntime
