// RearmStallProbe (src/mm/RearmProbe.hpp): early re-arm probe after the game replaced the player's animated object.
// Timeline of recording 2026-10-28 11-49-37 (bike jump-out): the last call on the old object 0.39 s before the launch (the representation
// was activated again 0.53 s before it), the 1 s poll re-armed 0.28 s after the launch. Built with -DUTPP_VEHEXIT_MUTATION=16 (no stall
// probe) this test MUST report FAIL lines.
#include "../src/mm/RearmProbe.hpp"
#include <cstdio>
#include <string>

using utpp::mm::RearmStallProbe;

namespace
{
int passed = 0, failed = 0;
void Check(const std::string& name, bool ok, const std::string& detail = {})
{
    std::printf("%s %s%s%s\n", ok ? "PASS" : "FAIL", name.c_str(), detail.empty() ? "" : " | ", detail.c_str());
    ok ? ++passed : ++failed;
}
// main-thread frames at 60 Hz (16.7 ms); the hook counters advance on the animation threads in between
struct Sim
{
    RearmStallProbe probe;
    uint64_t total = 1000, matched = 500;
    int64_t ms = 100000;
    int frame = 0;
    bool Step(bool aArmed, int aOther, int aPlayer)
    {
        total += uint64_t(aOther + aPlayer);
        matched += uint64_t(aPlayer);
        ms = 100000 + (int64_t(frame) * 50) / 3; // 16.67 ms per frame
        ++frame;
        return probe.Update(aArmed, total, matched, ms);
    }
};
} // namespace

int main()
{
    // 1. normal play: the player's calls advance ~58 per 60 frames (single skipped frames, as in the session's FK counters) - never a probe
    {
        Sim s;
        int probes = 0;
        for (int i = 0; i < 600; ++i)
            probes += s.Step(true, 40, (i % 25 == 7) ? 0 : 1) ? 1 : 0;
        Check("normal play with single skipped player frames: no probe in 10 s", probes == 0, "probes " + std::to_string(probes));
    }
    // 2. the recorded jump-out: object replaced at frame 100 (0.53 s = 32 frames before the launch at frame 132); the first probe must come
    //    within 3 frames of the swap and before the launch (the 1 s poll could come up to 60 frames later, 17 frames after the launch here)
    {
        Sim s;
        const int swap = 100, launch = 132;
        int first = -1, probes = 0;
        bool rearmed = false;
        for (int i = 0; i < 200; ++i)
        {
            const bool live = i < swap || rearmed; // after the swap the player's calls stop until the re-arm matches the new object
            const bool due = s.Step(true, 40, live ? 1 : 0);
            if (due)
            {
                ++probes;
                if (first < 0)
                    first = i;
                rearmed = true; // the probe finds the new object (tracks present) and re-arms: calls are matched from the next frame
            }
        }
        Check("recorded jump-out: object swap 32 frames before the launch - re-arm probe within 3 frames of the swap, before the launch",
              first >= swap && first <= swap + 3 && first < launch, "first " + std::to_string(first) + " swap " + std::to_string(swap));
        Check("recorded jump-out: one probe re-arms, then the matched calls resume - no further probes", probes == 1,
              "probes " + std::to_string(probes));
    }
    // 3. the new object has no tracks yet for 0.2 s: the probe repeats every >= 50 ms while the stall lasts, not every frame
    {
        Sim s;
        for (int i = 0; i < 30; ++i)
            s.Step(true, 40, 1);
        int probes = 0;
        int64_t lastMs = -1, minGap = 1 << 30;
        for (int i = 0; i < 60; ++i) // 1 s of stall
            if (s.Step(true, 40, 0))
            {
                ++probes;
                if (lastMs >= 0 && s.ms - lastMs < minGap)
                    minGap = s.ms - lastMs;
                lastMs = s.ms;
            }
        Check("stall of 1 s: probes repeat at >= 50 ms (14..20 per second), never per frame",
              probes >= 14 && probes <= 20 && minGap >= RearmStallProbe::kProbeIntervalMs,
              "probes " + std::to_string(probes) + " minGap " + std::to_string(minGap));
        int late = 0; // 2 s more of the same stall (a body not animated for a while): the repeat slows to 250 ms
        for (int i = 0; i < 120; ++i)
            late += s.Step(true, 40, 0) ? 1 : 0;
        Check("stall lasting 3 s: after the first second the probe repeats every 250 ms (6..9 in 2 s)", late >= 6 && late <= 9,
              "probes " + std::to_string(late));
    }
    // 4. negatives: paused game (nothing animated), not armed, counters reset by a re-arm - no probe
    {
        Sim s;
        int probes = 0;
        for (int i = 0; i < 30; ++i)
            s.Step(true, 40, 1);
        for (int i = 0; i < 120; ++i)
            probes += s.Step(true, 0, 0) ? 1 : 0; // menu / pause: no hook calls at all
        Check("paused game (no hook calls): no probe", probes == 0, "probes " + std::to_string(probes));
        for (int i = 0; i < 120; ++i)
            probes += s.Step(false, 40, 0) ? 1 : 0; // not armed: the 1 s poll arms
        Check("not armed: no probe", probes == 0, "probes " + std::to_string(probes));
        s.Step(true, 40, 1);
        s.total = 10;
        s.matched = 5; // counters restarted
        probes += s.Step(true, 40, 0) ? 1 : 0;
        Check("counters restarted: the first frame after is a new baseline, no probe", probes == 0, "probes " + std::to_string(probes));
    }
    std::printf("RearmProbeTest: %d passed, %d failed\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
