// FNV-1a 64 (standard offset basis 0xCBF29CE484222325, prime 0x100000001B3). Portable, header-only; shared by the plugin
// (CommonUtil.hpp) and the database header (BaseDatabase.hpp).
#pragma once

#include <cstddef>
#include <cstdint>

namespace utpp::util
{
// Continuable: pass the previous result as aHash.
inline uint64_t Fnv1a64(const uint8_t* aData, size_t aSize, uint64_t aHash = 0xCBF29CE484222325ull)
{
    for (size_t i = 0; i < aSize; ++i)
    {
        aHash ^= aData[i];
        aHash *= 0x100000001B3ull;
    }
    return aHash;
}
} // namespace utpp::util
