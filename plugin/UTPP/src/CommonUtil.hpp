// UTPP shared low-level helpers (Windows only, no RED4ext): QPC clock, SEH-guarded memory reads, SHA-256 / FNV-1a hashes, the plugin
// directory. One definition for what used to be copied into several translation units; each function does exactly what the
// copies did (same exception filter, same hex format, same hash constants).
#pragma once

#include <Windows.h>
#include <bcrypt.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "Fnv1a64.hpp"

namespace utpp::util
{
// ---- QueryPerformanceCounter ----
// The frequency is fixed for the life of the process: read once.
inline int64_t QpcFrequency()
{
    static const int64_t f = []
    {
        LARGE_INTEGER x{};
        QueryPerformanceFrequency(&x);
        return static_cast<int64_t>(x.QuadPart);
    }();
    return f;
}
inline int64_t QpcTicks()
{
    LARGE_INTEGER t{};
    QueryPerformanceCounter(&t);
    return static_cast<int64_t>(t.QuadPart);
}
// Now in seconds (ticks * (1 / frequency), the arithmetic of the former per-module Now()).
inline double QpcSeconds()
{
    static const double inv = 1.0 / static_cast<double>(QpcFrequency());
    return static_cast<double>(QpcTicks()) * inv;
}

// ---- SEH-guarded reads of game memory (a fault returns false / "unreadable", never propagates) ----
inline bool SehBytesEqual(const uint8_t* aAddr, const uint8_t* aExpected, size_t aLen)
{
    __try
    {
        return std::memcmp(aAddr, aExpected, aLen) == 0;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

inline bool SehCopy(const uint8_t* aAddr, uint8_t* aOut, size_t aLen)
{
    __try
    {
        std::memcpy(aOut, aAddr, aLen);
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

// "AA BB CC" (upper-case hex, single spaces).
inline std::string HexBytes(const uint8_t* aData, size_t aLen)
{
    std::string s;
    char buf[4];
    for (size_t i = 0; i < aLen; ++i)
    {
        std::snprintf(buf, sizeof(buf), "%02X", aData[i]);
        if (i)
            s += ' ';
        s += buf;
    }
    return s;
}

// HexBytes of up to 64 bytes read under SEH; "unreadable" on a fault or a longer request.
inline std::string HexAt(const uint8_t* aAddr, size_t aLen)
{
    uint8_t bytes[64] = {};
    if (aLen > sizeof(bytes) || !SehCopy(aAddr, bytes, aLen))
        return "unreadable";
    return HexBytes(bytes, aLen);
}

// ---- hashes (Fnv1a64: Fnv1a64.hpp) ----
// Lower-case hex SHA-256 (Windows CNG); "" on failure.
inline std::string Sha256Hex(const std::vector<uint8_t>& aBytes)
{
    BCRYPT_ALG_HANDLE alg = nullptr;
    BCRYPT_HASH_HANDLE hash = nullptr;
    uint8_t digest[32] = {};
    bool ok = BCryptOpenAlgorithmProvider(&alg, BCRYPT_SHA256_ALGORITHM, nullptr, 0) == 0;
    ok = ok && BCryptCreateHash(alg, &hash, nullptr, 0, nullptr, 0, 0) == 0;
    size_t off = 0;
    while (ok && off < aBytes.size())
    {
        const ULONG chunk = static_cast<ULONG>(std::min<size_t>(aBytes.size() - off, size_t(1) << 30));
        ok = BCryptHashData(hash, const_cast<PUCHAR>(aBytes.data() + off), chunk, 0) == 0;
        off += chunk;
    }
    ok = ok && BCryptFinishHash(hash, digest, sizeof digest, 0) == 0;
    if (hash)
        BCryptDestroyHash(hash);
    if (alg)
        BCryptCloseAlgorithmProvider(alg, 0);
    if (!ok)
        return {};
    static const char* hex = "0123456789abcdef";
    std::string s(64, '0');
    for (int i = 0; i < 32; ++i)
    {
        s[size_t(i) * 2] = hex[digest[i] >> 4];
        s[size_t(i) * 2 + 1] = hex[digest[i] & 15];
    }
    return s;
}

// ---- plugin directory ----
// Directory of this DLL with a trailing separator ("" when it cannot be resolved).
inline std::wstring PluginDir()
{
    HMODULE self = nullptr;
    wchar_t buf[MAX_PATH * 2] = {};
    if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                            reinterpret_cast<LPCWSTR>(&PluginDir), &self) ||
        GetModuleFileNameW(self, buf, static_cast<DWORD>(std::size(buf))) == 0)
        return {};
    const std::wstring path = buf;
    const size_t slash = path.find_last_of(L"\\/");
    return slash == std::wstring::npos ? std::wstring() : path.substr(0, slash + 1);
}

// ---- paths in logs and reports ----
// Nothing the DLL writes (log lines, capture headers, the report file) may carry an absolute path: it would contain the Windows user name
// or the game's install folder. Paths are shown relative to the game folder; a path outside it shows its file name only.
inline std::string WideToUtf8(const std::wstring& aWide)
{
    if (aWide.empty())
        return {};
    const int n = WideCharToMultiByte(CP_UTF8, 0, aWide.data(), static_cast<int>(aWide.size()), nullptr, 0, nullptr, nullptr);
    if (n <= 0)
        return {};
    std::string out(static_cast<size_t>(n), '\0');
    WideCharToMultiByte(CP_UTF8, 0, aWide.data(), static_cast<int>(aWide.size()), out.data(), n, nullptr, nullptr);
    return out;
}
namespace detail
{
inline std::wstring NormalizeSlashes(std::wstring aPath)
{
    for (wchar_t& c : aPath)
        if (c == L'/')
            c = L'\\';
    while (!aPath.empty() && aPath.back() == L'\\')
        aPath.pop_back();
    return aPath;
}
inline wchar_t LowerWide(wchar_t c) { return (c >= L'A' && c <= L'Z') ? static_cast<wchar_t>(c + (L'a' - L'A')) : c; }
} // namespace detail
namespace detail
{
inline std::wstring UpLevels(const std::wstring& aFolder, int aLevels)
{
    std::wstring p = NormalizeSlashes(aFolder);
    for (int i = 0; i < aLevels; ++i)
    {
        const size_t slash = p.find_last_of(L'\\');
        if (slash == std::wstring::npos)
            return {};
        p.resize(slash);
    }
    return p;
}
} // namespace detail
// "<game>\red4ext\logs\" -> "<game>" (the folder two levels up, no trailing separator); empty when there are fewer than two parents.
inline std::wstring GameRootOfLogFolder(const std::wstring& aLogFolder) { return detail::UpLevels(aLogFolder, 2); }
// "<game>\red4ext\plugins\UTPP\" -> "<game>" (three levels up).
inline std::wstring GameRootOfPluginDir(const std::wstring& aPluginDir) { return detail::UpLevels(aPluginDir, 3); }
// aPath relative to aRoot with backslashes ("red4ext\logs\x.csv") when it lies inside aRoot (case-insensitive, either slash); otherwise the
// file name alone. Never an absolute path, a drive letter or a user folder. UTF-8.
inline std::string RelativeForLog(const std::wstring& aPath, const std::wstring& aRoot)
{
    const std::wstring path = detail::NormalizeSlashes(aPath), root = detail::NormalizeSlashes(aRoot);
    if (!root.empty() && path.size() > root.size() + 1 && path[root.size()] == L'\\')
    {
        bool same = true;
        for (size_t i = 0; i < root.size() && same; ++i)
            same = detail::LowerWide(path[i]) == detail::LowerWide(root[i]);
        if (same)
            return WideToUtf8(path.substr(root.size() + 1));
    }
    const size_t slash = path.find_last_of(L'\\');
    return WideToUtf8(slash == std::wstring::npos ? path : path.substr(slash + 1));
}
// The game folder as seen from this DLL (<game>\red4ext\plugins\UTPP\ -> <game>); outside the game (unit tests) it is some other folder and
// every path then shows as a file name only.
inline const std::wstring& GameRoot()
{
    static const std::wstring root = GameRootOfPluginDir(PluginDir());
    return root;
}
inline std::string RelativeToGame(const std::wstring& aPath) { return RelativeForLog(aPath, GameRoot()); }
} // namespace utpp::util
