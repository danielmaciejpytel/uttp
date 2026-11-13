// UTPP - early re-arm probe after a stall of the player's animation calls. Header-only, no allocation, no game access.
//
// Recording 2026-10-28 11-49-37 (bike jump-out): the game replaced the player's animated object about 0.5 s after the exit start (the TPP
// representation is activated again: new animated object, new track buffers). The pose hooks match the player by the track tag, so from
// that frame no MM pose, no exit capture and no trace reached the visible body; the 1 s re-arm poll (OnRunningUpdate) found the new object
// only 0.66 s later, 0.28 s after the launch. The visible pose was the game's flight pose (straight legs) for 17 frames and then jumped
// to the MM jump clip in one frame (up to 108 deg per bone). The same happened on every exit of recording 05-03-09 (re-arm 0.01-0.58 s
// after the launch, the representation activated 0.5 s before it).
//
// RearmStallProbe sees the hook counters once per main-thread frame: the hook keeps running (total calls advance) but the player's
// matched calls stop for kStallFrames frames in a row -> a re-arm probe is due now (at most every kProbeIntervalMs while the stall
// lasts, every kSlowProbeIntervalMs once it lasted kFastStallMs - e.g. a body that is not animated for a while). A probe that finds the
// same object changes nothing; frames without a stall never probe (the 1 s poll is unchanged).
#pragma once

#include <cstdint>

namespace utpp::mm
{
class RearmStallProbe
{
public:
    static constexpr uint32_t kStallFrames = 2;      // one skipped player evaluation is normal (LOD / budget); two in a row is a stall
    static constexpr int64_t kProbeIntervalMs = 50;  // repeat while the stall lasts (the new object may need a few frames to get tracks)
    static constexpr int64_t kFastStallMs = 1000;    // after a stall this long the repeat slows down
    static constexpr int64_t kSlowProbeIntervalMs = 250;

    void Reset() { m_have = false; m_stall = 0; m_stallStartMs = m_lastProbeMs = -1; }
    // aArmed: the hooks have a player to match; aTotal / aMatched: hook call counters (monotonic); aNowMs: a millisecond clock.
    // Returns true when a re-arm probe should run on this frame.
    bool Update(bool aArmed, uint64_t aTotal, uint64_t aMatched, int64_t aNowMs)
    {
        if (!aArmed)
        {
            Reset();
            return false;
        }
        if (!m_have || aTotal < m_total || aMatched < m_matched)
        {
            m_have = true;
            m_total = aTotal;
            m_matched = aMatched;
            m_stall = 0;
            return false;
        }
        const bool matchedMoved = aMatched != m_matched;
        const bool totalMoved = aTotal != m_total;
        m_total = aTotal;
        m_matched = aMatched;
        if (matchedMoved)
        {
            m_stall = 0;
            m_stallStartMs = m_lastProbeMs = -1;
            return false;
        }
        if (!totalMoved) // nothing animated this frame (pause, menu, loading): neither a stall nor a recovery
            return false;
#if defined(UTPP_VEHEXIT_MUTATION) && UTPP_VEHEXIT_MUTATION == 16
        return false; // mutation 16: no stall probe (only the 1 s poll finds a new animated object)
#else
        if (++m_stall == 1)
            m_stallStartMs = aNowMs;
        if (m_stall < kStallFrames)
            return false;
        const int64_t interval = aNowMs - m_stallStartMs < kFastStallMs ? kProbeIntervalMs : kSlowProbeIntervalMs;
        if (m_lastProbeMs >= 0 && aNowMs - m_lastProbeMs < interval)
            return false;
        m_lastProbeMs = aNowMs;
        return true;
#endif
    }

private:
    bool m_have = false;
    uint64_t m_total = 0, m_matched = 0;
    uint32_t m_stall = 0;
    int64_t m_stallStartMs = -1, m_lastProbeMs = -1;
};
} // namespace utpp::mm
