#pragma once
// Production side of the diagnostic report (UTPPDiagWriteReport): lists the mod's recent files in the log folder and writes
// <game>\red4ext\logs\utpp-report-<yyyy-mm-dd-hh-mm-ss>.txt (UTF-8). The text logic is in DiagReport.hpp.
#include <string>

namespace utpp::diagreportruntime
{
// aLogFolder = <game>\red4ext\logs. Returns the report's path relative to the game folder, or "" when it could not be written.
std::string WriteReport(const std::wstring& aLogFolder, const std::string& aModVersion, const std::string& aGameVersion, const std::string& aBody);
} // namespace utpp::diagreportruntime
