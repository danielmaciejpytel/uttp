#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

namespace utpp::mm::mappingreceipt
{
constexpr size_t kStride = 48;
constexpr size_t kMaxBones = 1024;
struct Result
{
    uint32_t compared = 0, equal = 0;
    bool valid = false, overlap = false;
    std::array<uint64_t,16> equalPairs{}; // bit i describes mapping pair i
};
// Read-only evidence AFTER native mapping, BEFORE any mod write. Equality is
// observational: native post-copy operators may legitimately change records.
// This is not a frame token or permission to write. Caller owns live-read SEH.
inline Result Compare(std::span<const std::byte> source, std::span<const std::byte> destination,
                      std::span<const uint32_t> pairs)
{
    Result result;
    if (source.empty() || destination.empty() || source.size()%kStride || destination.size()%kStride ||
        source.size()/kStride > kMaxBones || destination.size()/kStride > kMaxBones ||
        pairs.empty() || pairs.size()%2 || pairs.size()/2 > kMaxBones) return result;
    const auto a = reinterpret_cast<uintptr_t>(source.data());
    const auto b = reinterpret_cast<uintptr_t>(destination.data());
    // Difference form avoids wrapping end-address arithmetic.
    result.overlap = a <= b ? b-a < source.size() : a-b < destination.size();
    if (result.overlap) return result;
    // Validate the whole table before dereferencing ANY mapped record.
    for (size_t i=0;i<pairs.size();i+=2)
        if (pairs[i]>=source.size()/kStride || pairs[i+1]>=destination.size()/kStride) return result;
    for (size_t i=0;i<pairs.size();i+=2)
    {
        ++result.compared;
        if (std::memcmp(source.data()+pairs[i]*kStride,
                        destination.data()+pairs[i+1]*kStride,kStride)==0)
        { ++result.equal; result.equalPairs[(i/2)/64] |= uint64_t{1} << ((i/2)%64); }
    }
    result.valid = true;
    return result;
}
} // namespace utpp::mm::mappingreceipt
