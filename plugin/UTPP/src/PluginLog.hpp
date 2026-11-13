// UTPP shared RED4ext log sink: one SDK / plugin handle (bound once by Main.cpp at load) and the LogInfo / LogWarn / LogError
// printf templates that every module used to carry its own copy of. Messages before Bind() are dropped, as before.
// Every line is formatted here and passed through pathscrub::ScrubLine before it reaches the logger: an absolute path (the Windows user name, the
// install folder) never ends up in a log file; the game folder shows as a relative path, any other drive path as "<path removed>".
#pragma once

#include <RED4ext/RED4ext.hpp>

#include "PathScrub.hpp"

namespace utpp::pluginlog
{
inline const RED4ext::v1::Sdk* g_sdk = nullptr;
inline RED4ext::v1::PluginHandle g_handle = nullptr;

inline void Bind(const RED4ext::v1::Sdk* aSdk, RED4ext::v1::PluginHandle aHandle)
{
    g_sdk = aSdk;
    g_handle = aHandle;
}

template<typename... Args>
void LogInfo(const char* aFmt, Args... aArgs)
{
    if (g_sdk && g_sdk->logger)
        g_sdk->logger->Info(g_handle, pathscrub::FormatLine(aFmt, aArgs...).c_str());
}
template<typename... Args>
void LogWarn(const char* aFmt, Args... aArgs)
{
    if (g_sdk && g_sdk->logger)
        g_sdk->logger->Warn(g_handle, pathscrub::FormatLine(aFmt, aArgs...).c_str());
}
template<typename... Args>
void LogError(const char* aFmt, Args... aArgs)
{
    if (g_sdk && g_sdk->logger)
        g_sdk->logger->Error(g_handle, pathscrub::FormatLine(aFmt, aArgs...).c_str());
}
} // namespace utpp::pluginlog
