// Root frame relative to the drawn capsule (rootframe::Inputs::capsuleYawAhead).
// The game applies the body turn requested for a tick after the feed of that tick read the yaw, and draws the pose with the turned
// capsule. The feed therefore sees each capsule turn one feed late, while the clip's root turn of the same tick is already in its
// animation yaw delta. Without the coming turn the offset dipped for one frame whenever the frame time changed (a long frame sizes both
// turns) and the drawn heading stepped forward and back. With it the drawn heading (fed yaw + coming turn + offset) follows the clip.
#include "../src/mm/MotionRootFrame.hpp"
#include <cmath>
#include <cstdio>
#include <vector>
using namespace utpp::mm;
static int pass = 0, fail = 0;
static void C(const char *n, bool v)
{
    std::printf("%s %s\n", v ? "PASS" : "FAIL", n);
    v ? ++pass : ++fail;
}
namespace
{
constexpr float kDeg = 3.14159265358979f / 180.f;
float WrapDeg(float a)
{
    return std::remainder(a, 360.f);
}
// Recorded root inputs around a long frame: dt (s), clip root yaw delta, fed capsule yaw delta, the capsule yaw delta of the next feed
// (= the turn applied in this tick), the offset yaw the root had without the coming turn (deg).
struct Row
{
    float dt, anim, cap, next, offset;
};
// Female V, keyboard run, Reface_Start (180) at a 61 ms frame
const Row kRunRefaceRight[] = {
    {0.01471f, -4.7948f, -5.7685f, -4.7414f, 18.695f},  {0.01644f, -5.3490f, -4.7414f, -5.3260f, 18.087f},
    {0.01708f, -5.5261f, -5.3260f, -5.5440f, 17.887f},  {0.01771f, -5.6742f, -5.5440f, -5.7047f, 17.757f},
    {0.06138f, -22.2760f, -5.7047f, -19.7948f, 1.185f}, {0.01283f, -3.5027f, -19.7948f, -4.5804f, 17.478f},
    {0.01647f, -4.4578f, -4.5804f, -5.4738f, 17.600f},  {0.01806f, -4.7904f, -5.4738f, -5.6440f, 18.284f},
};
// Reface_Start (180, left) at a 51 ms frame
const Row kRunRefaceLeft[] = {
    {0.01635f, 1.9153f, 3.0742f, 2.9385f, -29.290f}, {0.01809f, 2.4360f, 2.9385f, 3.2366f, -29.792f},
    {0.01691f, 2.5554f, 3.2366f, 3.0675f, -30.474f}, {0.01668f, 2.8046f, 3.0675f, 2.9962f, -30.737f},
    {0.05083f, 11.7751f, 2.9962f, 9.1129f, -21.958f}, {0.01391f, 3.2504f, 9.1129f, 2.8485f, -27.820f},
    {0.01689f, 4.2663f, 2.8485f, 3.5508f, -26.402f}, {0.01640f, 4.4100f, 3.5508f, 3.6577f, -25.543f},
};
// Run loop backwards at a 38 ms frame
const Row kRunLoop[] = {
    {0.01634f, 5.6903f, 3.1495f, 3.8676f, 12.751f},  {0.01690f, 5.8375f, 3.8676f, 4.6241f, 14.721f},
    {0.02012f, 6.6491f, 4.6241f, 4.9352f, 16.746f},  {0.01351f, 4.7006f, 4.9352f, 5.1790f, 16.511f},
    {0.03778f, 10.3665f, 5.1790f, 11.8406f, 21.699f}, {0.02025f, 4.8612f, 11.8406f, 5.8271f, 14.719f},
    {0.01717f, 3.5074f, 5.8271f, 4.8190f, 12.400f},  {0.01444f, 2.5580f, 4.8190f, 3.7024f, 10.139f},
};

struct Run
{
    std::vector<float> offset, drawn, clip; // published offset, drawn heading (fed yaw + coming turn + offset), clip turn of the feed (deg)
};
// Feeds the rows (row 0 seeds the root); useAhead false = the coming turn is not passed (fed yaw paired with the drawn translation). The game draws the pose with
// the capsule after the tick's turn in both cases, so the drawn heading is fed yaw + coming turn + published offset either way.
template <size_t N> Run Replay(const Row (&rows)[N], bool useAhead)
{
    rootframe::Processor p;
    p.Reset(0.f);
    float actor = 0.f; // fed yaw (deg)
    Run r;
    for (size_t k = 0; k < N; ++k)
    {
        actor += k ? rows[k].cap : 0.f;
        rootframe::Inputs in;
        in.dt = rows[k].dt;
        in.actorYaw = actor * kDeg;
        in.capsuleYawDelta = rows[k].cap * kDeg;
        in.animationYawDelta = rows[k].anim * kDeg;
        in.capsuleYawAhead = useAhead ? rows[k].next * kDeg : 0.f;
        in.movingGround = true;
        rootframe::Proposal next;
        if (!p.Prepare(in, next) || !p.Commit(next))
            return {};
        if (k == 0)
            continue; // the seed feed (reseed: offset 0)
        r.offset.push_back(next.Frame().yaw / kDeg);
        r.drawn.push_back(actor + rows[k].next + next.Frame().yaw / kDeg);
        r.clip.push_back(rows[k].anim);
    }
    return r;
}
// largest single-frame dip of a series: |x_k - (x_{k-1} + x_{k+1}) / 2|
float Dip(const std::vector<float> &x)
{
    float m = 0;
    for (size_t k = 1; k + 1 < x.size(); ++k)
        m = std::fmax(m, std::fabs(WrapDeg(x[k] - 0.5f * (x[k - 1] + x[k + 1]))));
    return m;
}
// largest step of the drawn heading against the clip's own turn of the same feed
float DrawnError(const Run &r)
{
    float m = 0;
    for (size_t k = 1; k < r.drawn.size(); ++k)
        m = std::fmax(m, std::fabs(WrapDeg((r.drawn[k] - r.drawn[k - 1]) - r.clip[k])));
    return m;
}
template <size_t N> void Recorded(const char *name, const Row (&rows)[N])
{
    const Run now = Replay(rows, true), old = Replay(rows, false);
    std::vector<float> recorded;
    for (size_t k = 1; k < N; ++k)
        recorded.push_back(rows[k].offset);
    char buf[256];
    std::snprintf(buf, sizeof buf, "%s: the rows reproduce the recorded dip without the coming turn (%.2f vs %.2f deg)", name, Dip(old.offset),
                  Dip(recorded));
    C(buf, !old.offset.empty() && std::fabs(Dip(old.offset) - Dip(recorded)) < 0.5f && Dip(recorded) > 5.f);
    std::snprintf(buf, sizeof buf, "%s: drawn heading turns with the clip in every feed (max error %.3f deg; without the coming turn %.2f)",
                  name, DrawnError(now), DrawnError(old));
    C(buf, !now.drawn.empty() && DrawnError(now) < 0.01f);
    std::snprintf(buf, sizeof buf, "%s: no single-frame dip of the offset (%.2f deg; without the coming turn %.2f)", name, Dip(now.offset),
                  Dip(old.offset));
    C(buf, !now.offset.empty() && Dip(now.offset) < 3.f);
}

// Closed loop as in the game: the clip turns by aTurn deg (smoothstep over aSeconds), the capsule turn requested before tick N is the
// previous feed's clip rate x the tick's time (BodyTurn::CommandAnim, animation-led) and is applied in tick N; feed N reads the yaw
// before it. Drawn heading = fed yaw + the coming turn + the published offset. aLongFrame: one frame of 50 ms in the middle of the turn.
struct Turn
{
    std::vector<float> t, drawn, clip;
};
Turn ClosedLoop(float aHz, float aTurn, float aSeconds, bool aLongFrame, bool useAhead)
{
    const auto clipAt = [&](float t) {
        const float u = std::fmin(std::fmax(t / aSeconds, 0.f), 1.f);
        return aTurn * u * u * (3.f - 2.f * u);
    };
    rootframe::Processor p;
    p.Reset(0.f);
    rootframe::Inputs in;
    in.dt = 1.f / aHz;
    in.movingGround = true;
    rootframe::Proposal first;
    p.Prepare(in, first);
    p.Commit(first);
    Turn r;
    float t = 0.f, yaw = 0.f, rate = 0.f, applied = 0.f; // fed yaw (deg), clip rate of the last feed (deg/s), turn of the last tick
    const float tStart = 0.1f;
    bool longDone = false;
    while (t < tStart + aSeconds + 0.3f)
    {
        float dt = 1.f / aHz;
        if (aLongFrame && !longDone && t >= tStart + 0.4f * aSeconds)
            dt = 0.05f, longDone = true;
        const float ahead = rate * dt; // the request of this tick (CET frame before the tick: last feed's rate x this frame's time)
        yaw += applied;                // the feed reads the yaw with the previous tick's turn
        const float a = clipAt(t + dt - tStart) - clipAt(t - tStart);
        in.dt = dt;
        in.actorYaw = yaw * kDeg;
        in.capsuleYawDelta = applied * kDeg;
        in.animationYawDelta = a * kDeg;
        in.capsuleYawAhead = useAhead ? ahead * kDeg : 0.f;
        rootframe::Proposal next;
        if (!p.Prepare(in, next) || !p.Commit(next))
            return {};
        t += dt;
        rate = a / dt;
        applied = ahead;
        r.t.push_back(t);
        r.drawn.push_back(yaw + ahead + next.Frame().yaw / kDeg); // the capsule as drawn this tick
        r.clip.push_back(clipAt(t - tStart));
    }
    return r;
}
float Reach(const Turn &r, float aTurn, float aShare)
{
    for (size_t k = 0; k < r.t.size(); ++k)
        if (r.drawn[k] * (aTurn > 0 ? 1.f : -1.f) >= aShare * std::fabs(aTurn))
            return r.t[k];
    return 1e9f;
}
bool Monotone(const Turn &r, float aTurn)
{
    for (size_t k = 1; k < r.drawn.size(); ++k)
        if ((r.drawn[k] - r.drawn[k - 1]) * (aTurn > 0 ? 1.f : -1.f) < -1e-3f)
            return false;
    return true;
}
float MaxClipError(const Turn &r)
{
    float m = 0;
    for (size_t k = 0; k < r.t.size(); ++k)
        m = std::fmax(m, std::fabs(r.drawn[k] - r.clip[k]));
    return m;
}
} // namespace

int main()
{
    Recorded("run reface right, 61 ms frame", kRunRefaceRight);
    Recorded("run reface left, 51 ms frame", kRunRefaceLeft);
    Recorded("run loop, 38 ms frame", kRunLoop);
    for (const float hz : {30.f, 60.f, 120.f})
        for (const float turn : {90.f, -180.f})
        {
            const float seconds = std::fabs(turn) > 100.f ? 0.9f : 0.6f;
            const Turn now = ClosedLoop(hz, turn, seconds, false, true), old = ClosedLoop(hz, turn, seconds, false, false);
            const float frame = 1.f / hz;
            for (const float share : {0.5f, 0.9f})
            {
                const float tn = Reach(now, turn, share), to = Reach(old, turn, share);
                char buf[200];
                std::snprintf(buf, sizeof buf, "%.0f Hz, %.0f deg turn: %.0f %% reached at %.3f s, without the coming turn %.3f s (<= +1 frame)",
                              hz, turn, share * 100.f, tn, to);
                C(buf, tn <= to + frame + 1e-4f);
            }
            char buf[200];
            std::snprintf(buf, sizeof buf, "%.0f Hz, %.0f deg turn: drawn heading = clip heading (max %.3f deg, without the coming turn %.2f)", hz,
                          turn, MaxClipError(now), MaxClipError(old));
            C(buf, MaxClipError(now) < 0.05f);
            const Turn hitch = ClosedLoop(hz, turn, seconds, true, true), hitchOld = ClosedLoop(hz, turn, seconds, true, false);
            std::snprintf(buf, sizeof buf, "%.0f Hz, %.0f deg turn with a 50 ms frame: drawn heading never turns back (without the coming turn: %s)",
                          hz, turn, Monotone(hitchOld, turn) ? "monotone" : "turns back");
            C(buf, Monotone(hitch, turn) && MaxClipError(hitch) < 0.05f);
        }
    // a requested turn that did not happen (request dropped): one frame off, then the frame follows the measured yaw again
    {
        rootframe::Processor p;
        p.Reset(0.f);
        rootframe::Inputs in;
        in.dt = 1.f / 60.f;
        in.movingGround = true;
        rootframe::Proposal pr;
        p.Prepare(in, pr);
        p.Commit(pr);
        in.capsuleYawAhead = 5.f * kDeg; // requested, never applied
        p.Prepare(in, pr);
        p.Commit(pr);
        const float drawnRequested = (in.capsuleYawAhead + pr.Frame().yaw) / kDeg;
        in.capsuleYawAhead = 0.f;
        in.capsuleYawDelta = 0.f; // the yaw did not change
        p.Prepare(in, pr);
        p.Commit(pr);
        const float drawnAfter = pr.Frame().yaw / kDeg;
        C("dropped request: the drawn heading stays put and the offset returns at once", std::fabs(drawnRequested) < 1e-3f &&
                                                                                          std::fabs(drawnAfter) < 1e-3f);
    }
    std::printf("%d PASS %d FAIL\n", pass, fail);
    return fail ? 1 : 0;
}
