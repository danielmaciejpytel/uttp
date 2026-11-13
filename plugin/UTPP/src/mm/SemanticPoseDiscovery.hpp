#pragma once

// Bounded READ-ONLY discovery for mapping callbacks. The caller must guard
// accesses to engine-owned spans with SEH. No pointer is dereferenced later:
// a successful observation owns all source/destination CName hashes and pairs.
// Source rig and merged destination rig are deliberately separate identities.
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>

namespace utpp::mm::semanticpose
{
// Covers the observed 619-bone merged player and all four observed sources.
// These are storage limits, not rig/pass recognition criteria. Larger rigs or
// tables fail closed; no truncated sample can become an approved write policy.
constexpr uint32_t kMaxBones = 1024;
constexpr uint32_t kMaxPairs = 1024;
constexpr uint32_t kMaxSignatures = 8;

struct Identity
{
    uint64_t generation = 0, sourceRig = 0, destinationRig = 0, graph = 0;
    uint64_t desc = 0, bones = 0, tag = 0;
    int64_t ticks = 0;
    uint32_t lod = 0, thread = 0;
};

struct Record
{
    Identity identity;
    uint64_t sourceNamesFingerprint = 0, destinationNamesFingerprint = 0, pairsFingerprint = 0;
    uint32_t sourceCount = 0, destinationCount = 0, pairCount = 0;
    std::array<uint64_t, kMaxBones> sourceNames{};
    std::array<uint64_t, kMaxBones> destinationNames{};
    std::array<uint32_t, kMaxPairs * 2> pairs{};
};
static_assert(std::is_trivially_copyable_v<Record>);
static_assert(sizeof(Record) <= 25 * 1024, "semantic discovery record exceeded callback scratch budget");

enum class Result : uint8_t { Stored, Duplicate, Disabled, Invalid, Busy, Full, StaleGeneration };

inline uint64_t Hash64(std::span<const uint64_t> aValues)
{
    uint64_t hash = 14695981039346656037ULL;
    for (uint64_t value : aValues)
        for (unsigned shift = 0; shift < 64; shift += 8)
        { hash ^= (value >> shift) & 0xFFu; hash *= 1099511628211ULL; }
    return hash;
}

inline uint64_t HashPairs(std::span<const uint32_t> aValues)
{
    uint64_t hash = 14695981039346656037ULL;
    for (uint32_t value : aValues)
        for (unsigned shift = 0; shift < 32; shift += 8)
        { hash ^= (value >> shift) & 0xFFu; hash *= 1099511628211ULL; }
    return hash;
}

class Capture
{
public:
    // Explicit census only. Call on control path. A concurrent callback can
    // cause a failed Start; it never causes a blocking animation worker.
    bool Start(uint64_t aGeneration)
    {
        m_enabled.store(false, std::memory_order_release);
        if (!aGeneration || m_busy.test_and_set(std::memory_order_acquire)) return false;
        m_count = 0;
        m_generation.store(aGeneration, std::memory_order_relaxed);
        m_busy.clear(std::memory_order_release);
        m_enabled.store(true, std::memory_order_release);
        return true;
    }
    void Stop() { m_enabled.store(false, std::memory_order_release); }
    bool Enabled() const { return m_enabled.load(std::memory_order_acquire); }

    // The only live-data reader. Validate/copy before taking m_busy: a guarded
    // engine read fault cannot strand a lock. SDK CName is an 8-byte hash;
    // the integration must assert that layout before constructing these spans.
    Result Observe(const Identity& aIdentity, std::span<const uint64_t> aSourceNames,
                   std::span<const uint64_t> aDestinationNames, std::span<const uint32_t> aPairs)
    {
        if (!Enabled()) return Result::Disabled;
        if (aIdentity.generation != m_generation.load(std::memory_order_acquire)) return Result::StaleGeneration;
        if (!aIdentity.sourceRig || !aIdentity.destinationRig || !aIdentity.graph ||
            !aIdentity.bones || !aIdentity.tag || aSourceNames.empty() || aDestinationNames.empty() ||
            aSourceNames.size() > kMaxBones || aDestinationNames.size() > kMaxBones ||
            aPairs.empty() || (aPairs.size() & 1u) || aPairs.size() > kMaxPairs * 2u)
            return Result::Invalid;
        Record copy;
        copy.identity = aIdentity;
        copy.sourceCount = static_cast<uint32_t>(aSourceNames.size());
        copy.destinationCount = static_cast<uint32_t>(aDestinationNames.size());
        copy.pairCount = static_cast<uint32_t>(aPairs.size() / 2);
        for (std::size_t i = 0; i < aSourceNames.size(); ++i) copy.sourceNames[i] = aSourceNames[i];
        for (std::size_t i = 0; i < aDestinationNames.size(); ++i) copy.destinationNames[i] = aDestinationNames[i];
        for (std::size_t i = 0; i < aPairs.size(); i += 2)
        {
            const uint32_t src = aPairs[i], dst = aPairs[i + 1];
            if (src >= copy.sourceCount || dst >= copy.destinationCount) return Result::Invalid;
            copy.pairs[i] = src; copy.pairs[i + 1] = dst;
        }
        copy.sourceNamesFingerprint = Hash64({copy.sourceNames.data(), copy.sourceCount});
        copy.destinationNamesFingerprint = Hash64({copy.destinationNames.data(), copy.destinationCount});
        copy.pairsFingerprint = HashPairs({copy.pairs.data(), copy.pairCount * 2u});
        if (m_busy.test_and_set(std::memory_order_acquire)) return Result::Busy;
        Result result = Result::Stored;
        if (!Enabled()) result = Result::Disabled;
        else if (aIdentity.generation != m_generation.load(std::memory_order_relaxed)) result = Result::StaleGeneration;
        else
        {
            for (uint32_t i = 0; i < m_count; ++i)
                if (SameSignature(copy, m_records[i])) { result = Result::Duplicate; break; }
            if (result == Result::Stored)
            {
                if (m_count == kMaxSignatures) result = Result::Full;
                else m_records[m_count++] = copy;
            }
        }
        m_busy.clear(std::memory_order_release);
        return result;
    }

    // Copies OWNED bytes only. Output pointer metadata is evidence, never a
    // licence to dereference the original engine objects on the control path.
    bool Copy(uint32_t aIndex, Record& aOut)
    {
        if (m_busy.test_and_set(std::memory_order_acquire)) return false;
        const bool ok = aIndex < m_count;
        if (ok) aOut = m_records[aIndex];
        m_busy.clear(std::memory_order_release);
        return ok;
    }
    bool Count(uint32_t& aOut)
    {
        if (m_busy.test_and_set(std::memory_order_acquire)) return false;
        aOut = m_count;
        m_busy.clear(std::memory_order_release);
        return true;
    }

private:
    static bool SameSignature(const Record& a, const Record& b)
    {
        if (a.identity.sourceRig != b.identity.sourceRig || a.identity.destinationRig != b.identity.destinationRig ||
            a.identity.graph != b.identity.graph || a.identity.lod != b.identity.lod ||
            a.sourceCount != b.sourceCount || a.destinationCount != b.destinationCount || a.pairCount != b.pairCount ||
            a.sourceNamesFingerprint != b.sourceNamesFingerprint ||
            a.destinationNamesFingerprint != b.destinationNamesFingerprint || a.pairsFingerprint != b.pairsFingerprint)
            return false;
        // Hash collisions do not merge signatures or authorize a policy.
        for (uint32_t i = 0; i < a.sourceCount; ++i) if (a.sourceNames[i] != b.sourceNames[i]) return false;
        for (uint32_t i = 0; i < a.destinationCount; ++i) if (a.destinationNames[i] != b.destinationNames[i]) return false;
        for (uint32_t i = 0; i < a.pairCount * 2u; ++i) if (a.pairs[i] != b.pairs[i]) return false;
        return true;
    }
    std::atomic<bool> m_enabled{false};
    std::atomic<uint64_t> m_generation{0};
    std::atomic_flag m_busy = ATOMIC_FLAG_INIT;
    uint32_t m_count = 0;
    std::array<Record, kMaxSignatures> m_records{}; // keep Capture static/off worker stack
};
} // namespace utpp::mm::semanticpose
