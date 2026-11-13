// UTPP loothead module: logging into the RED4ext log (implemented in world/LootHead.cpp). Game thread / load only.
#pragma once

#include <cstdio>

namespace utpp::loothead
{
void LogLine(int aLevel, const char* aText); // 0 info, 1 warn, 2 error

template<typename... Args>
void LogF(int aLevel, const char* aFmt, Args... aArgs)
{
    char buf[768];
    std::snprintf(buf, sizeof(buf), aFmt, aArgs...);
    LogLine(aLevel, buf);
}
} // namespace utpp::loothead
