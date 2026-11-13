#pragma once
// Production side of the compact pose capture (UTPPPoseCaptureStart / Stop / Status): real files, the Windows Compression API and the writer
// thread around mm/PoseCaptureSession.hpp (which is offline tested). The format is documented in mm/PoseCaptureFormat.hpp.
// Control path (Main.cpp natives, under g_coldMutex): Init once, Start / Stop / Status from the natives, PublishMap on every (re)arm of the
// player rig, ClearMap on disarm. The writer thread never touches game memory: it only drains the capture rings and writes files.
// The text capture (BodyTraceRuntime.cpp) and this one share the rings: whichever starts second is refused (mm/CaptureOwner.hpp).
#include <cstdint>
#include <string>
#include <vector>

#include "mm/BodyDeformerTrace.hpp"

namespace utpp::posecaptureruntime
{
using LogFn = void (*)(const char* aLine);

// aFolder = <game>\red4ext\logs (files are written there); paths in log lines and results are shown relative to the game folder.
void Init(LogFn aLog, const std::wstring& aFolder);
// keepMinutes 0 = one file per rig for the whole session; > 0 = one-minute segments, older than the window deleted as they close.
// false = refused (not initialised, the text capture owns the rings, bad window, or already recording with ANOTHER window: Stop first);
// already recording with the same window = true, nothing changes.
bool Start(int32_t aKeepMinutes);
// Stops the capture, closes the file(s) and returns the files of this session that exist, relative to the game folder, joined with '|'
// (empty when nothing was written).
std::string Stop();
// "poseCapture state=recording|stopped|off keep=<n> frames=<n> dropped=<n> bytes=<n> files=<n> file=<relative path>"
std::string Status();
bool Enabled();
void PublishMap(const mm::bodytrace::Map& aMap, bool aPost, std::vector<uint64_t> aHashes, std::vector<int16_t> aParents);
void ClearMap();
// DLL unload: stops the thread, writes the pending block, closes the file.
void Shutdown();
} // namespace utpp::posecaptureruntime
