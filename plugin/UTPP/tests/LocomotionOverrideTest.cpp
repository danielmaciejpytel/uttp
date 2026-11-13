// Offline test of the Locomotion move override (vehicle approach walk): bit-identity of every output and counter without an
// override against a verbatim copy of the pre-override core (tests/ref/LocomotionCoreReference.hpp = git show of the commit before
// the override), override semantics (TTL, clamp, rejection, Locomotion active / inactive), and a two-thread seqlock check.

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>

// The old core goes into its own namespace (all standard headers it needs are included first; pragma once keeps them out).
#include <algorithm>
#include <cstddef>
namespace oldref
{
#include "ref/LocomotionCoreReference.hpp"
}
#include "../src/input/LocomotionCore.hpp"
#include "../src/input/ApproachCore.hpp"

static int g_pass = 0;
static int g_fail = 0;

static void Check(bool aOk, const char* aWhat)
{
    if (aOk)
        ++g_pass;
    else
    {
        ++g_fail;
        std::printf("FAIL: %s\n", aWhat);
    }
}

static bool SameBits(float a, float b)
{
    return std::memcmp(&a, &b, sizeof(float)) == 0;
}

// xorshift: deterministic without <random> distribution differences between standard libraries.
struct Rng
{
    uint64_t s;
    explicit Rng(uint64_t aSeed) : s(aSeed ? aSeed : 88172645463325252ull) {}
    uint64_t Next()
    {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        return s;
    }
    float Unit() { return static_cast<float>(Next() >> 40) / static_cast<float>(1u << 24); }
    float Range(float a, float b) { return a + (b - a) * Unit(); }
    uint32_t Below(uint32_t n) { return static_cast<uint32_t>(Next() % n); }
};

using NewCore = utpp::freemove::Core;
using OldCore = oldref::utpp::freemove::Core;

enum class Variant
{
    Plain,            // never touched by the override API
    SetClearBefore,   // override set and cleared before any read
    SetClearMidway,   // ... and again at random points (no read in between)
    ExpiredBefore,    // override set with a short TTL; every later read is after the expiry
    RejectedNotInst,  // override attempted while not installed (rejected)
    RejectedBadInput, // NaN / zero-ttl calls (rejected)
};

static float RandRaw(Rng& r)
{
    switch (r.Below(10))
    {
    case 0: return 0.0f;
    case 1: return NAN;
    case 2: return -r.Range(0.0f, 1.0f);
    case 3: return 1.0f;
    case 4: return r.Range(0.0f, 0.2f);
    default: return r.Range(-1.0f, 1.0f);
    }
}

static bool RunIdentity(Variant aV, uint64_t aSeed)
{
    Rng rng(aSeed);
    OldCore o;
    NewCore n;
    double t = 1.0;
    bool ok = true;
    // Variants that try the override while not installed must do so BEFORE SetInstalled below.
    if (aV == Variant::RejectedNotInst)
    {
        n.SetMoveOverride(0.5f, 0.5f, 10.0f, 0.4, t);
        n.SetMoveOverride(-1.0f, 0.2f, 10.0f, 0.4, t);
    }
    const bool install = aV != Variant::RejectedNotInst || rng.Below(2) == 0;
    if (install)
    {
        o.SetInstalled(true);
        n.SetInstalled(true);
    }
    if (aV == Variant::SetClearBefore || aV == Variant::SetClearMidway)
    {
        n.SetMoveOverride(0.7f, -0.2f, 33.0f, 0.4, t);
        n.ClearMoveOverride();
    }
    if (aV == Variant::ExpiredBefore)
    {
        n.SetMoveOverride(0.7f, -0.2f, 33.0f, 0.05, t);
        t += 100.0; // every later read happens after the expiry (the sequence also jumps the clock back by < 0.3 s)
    }
    if (aV == Variant::RejectedBadInput && install)
    {
        n.SetMoveOverride(NAN, 0.2f, 0.0f, 0.4, t);
        n.SetMoveOverride(0.2f, INFINITY, 0.0f, 0.4, t);
        n.SetMoveOverride(0.2f, 0.2f, NAN, 0.4, t);
        n.SetMoveOverride(0.2f, 0.2f, 0.0f, 0.0, t);
        n.SetMoveOverride(0.2f, 0.2f, 0.0f, -1.0, t);
        n.SetMoveOverride(0.2f, 0.2f, 0.0f, NAN, t);
    }
    for (int step = 0; step < 4000 && ok; ++step)
    {
        t += rng.Range(0.0f, 0.12f);
        if (rng.Below(40) == 0)
            t -= rng.Range(0.0f, 0.3f); // non-monotone clock must behave the same too
        switch (rng.Below(14))
        {
        case 0:
        {
            const bool on = rng.Below(2) != 0;
            o.SetEnabled(on);
            n.SetEnabled(on);
            break;
        }
        case 1:
        {
            const bool on = rng.Below(4) == 0;
            o.SetHold(on);
            n.SetHold(on);
            break;
        }
        case 2:
        {
            const uint32_t m = rng.Next() & 0x7FFFu;
            o.SetBlockMask(m);
            n.SetBlockMask(m);
            break;
        }
        case 3:
        case 4:
        {
            const bool tpp = rng.Below(4) != 0;
            const float cam = rng.Range(0.0f, 2.0f);
            const uint32_t bits = rng.Below(3) == 0 ? (rng.Next() & 0xFFFFFFu) : 0u;
            o.Heartbeat(t, tpp, cam, bits);
            n.Heartbeat(t, tpp, cam, bits);
            break;
        }
        case 5:
        {
            const float dz = rng.Range(0.0f, 0.5f);
            auto p = o.GetParams();
            p.deadzone = dz;
            p.axisFreshSec = rng.Range(0.05f, 0.4f);
            const bool a = o.SetParams(p);
            auto q = n.GetParams();
            q.deadzone = dz;
            q.axisFreshSec = p.axisFreshSec;
            const bool b = n.SetParams(q);
            ok = ok && a == b;
            break;
        }
        case 6:
        case 7:
        case 8:
        {
            const auto ax = rng.Below(2) ? oldref::utpp::freemove::Axis::X : oldref::utpp::freemove::Axis::Y;
            const auto bx = ax == oldref::utpp::freemove::Axis::X ? utpp::freemove::Axis::X : utpp::freemove::Axis::Y;
            const float raw = RandRaw(rng);
            ok = ok && SameBits(o.FilterAxis(ax, raw, t), n.FilterAxis(bx, raw, t));
            break;
        }
        case 9:
        case 10:
        {
            const float raw = RandRaw(rng);
            ok = ok && SameBits(o.FilterHeading(raw, t), n.FilterHeading(raw, t));
            break;
        }
        case 11:
            ok = ok && o.HasInput(t) == n.HasInput(t) && SameBits(o.RawAngle(t), n.RawAngle(t)) &&
                 o.Reasons(t) == n.Reasons(t) && o.Active(t) == n.Active(t);
            break;
        case 12:
            ok = ok && SameBits(o.RawAxis(oldref::utpp::freemove::Axis::X), n.RawAxis(utpp::freemove::Axis::X)) &&
                 SameBits(o.RawAxis(oldref::utpp::freemove::Axis::Y), n.RawAxis(utpp::freemove::Axis::Y));
            break;
        default:
            if (aV == Variant::SetClearMidway && rng.Below(3) == 0)
            {
                n.SetMoveOverride(rng.Range(-1.0f, 1.0f), rng.Range(-1.0f, 1.0f), rng.Range(-180.0f, 180.0f), 0.3, t);
                n.ClearMoveOverride();
            }
            break;
        }
        ok = ok && o.AxisReads() == n.AxisReads() && o.AxisRewrites() == n.AxisRewrites() &&
             o.HeadingReads() == n.HeadingReads() && o.HeadingRewrites() == n.HeadingRewrites() &&
             o.Beats() == n.Beats() && o.Activations() == n.Activations();
    }
    if (aV == Variant::Plain || aV == Variant::ExpiredBefore || aV == Variant::RejectedNotInst)
        ok = ok && n.OverrideAxisReads() == 0;
    return ok;
}

static void TestBitIdentity()
{
    const char* names[] = {"plain", "set+clear before", "set+clear midway", "expired before", "rejected (not installed)",
                           "rejected (bad input)"};
    const Variant vs[] = {Variant::Plain,           Variant::SetClearBefore,   Variant::SetClearMidway,
                          Variant::ExpiredBefore,   Variant::RejectedNotInst,  Variant::RejectedBadInput};
    for (int v = 0; v < 6; ++v)
    {
        bool all = true;
        for (uint64_t seed = 1; seed <= 40; ++seed)
            all = all && RunIdentity(vs[v], seed * 7919u + static_cast<uint64_t>(v));
        char msg[160];
        std::snprintf(msg, sizeof(msg), "bit-identical to the pre-override core, 40 random sequences: %s", names[v]);
        Check(all, msg);
    }
}

static void TestSemantics()
{
    using namespace utpp::freemove;
    // Locomotion off (disabled): the PSM sees the override stick and heading as they are.
    {
        Core c;
        c.SetInstalled(true);
        Check(c.SetMoveOverride(0.3f, 0.4f, 25.0f, 0.25, 10.0), "override accepted when installed");
        Check(c.OverrideActive(10.1) && c.OverrideActive(10.2) && !c.OverrideActive(10.25),
              "override lives until now + ttl (end exclusive)");
        Check(c.FilterAxis(Axis::X, 0.9f, 10.1) == 0.3f && c.FilterAxis(Axis::Y, -0.9f, 10.1) == 0.4f,
              "override replaces both axes (Locomotion off)");
        Check(c.FilterHeading(77.0f, 10.1) == 25.0f, "override heading returned (Locomotion off)");
        Check(c.RealMagnitude(10.1) > 1.27f && c.RealMagnitude(10.1) < 1.28f, "real stick keeps the hardware values (0.9, -0.9)");
        Check(c.FilterAxis(Axis::X, 0.9f, 10.25) == 0.9f && c.FilterHeading(77.0f, 10.25) == 77.0f,
              "after the TTL the engine values pass through again");
    }
    // Locomotion on: the usual rewrite applies to the override, RawAngle (body-turn target) follows it.
    {
        Core c;
        c.SetInstalled(true);
        c.SetEnabled(true);
        c.Heartbeat(5.0, true, 1.5f, 0);
        c.SetMoveOverride(-0.3f, 0.3f, 45.0f, 0.3, 5.0);
        const float x = c.FilterAxis(Axis::X, 0.0f, 5.01);
        const float y = c.FilterAxis(Axis::Y, 0.0f, 5.01);
        Check(x == 0.0f && std::fabs(y - std::sqrt(0.18f)) < 1e-5f, "Locomotion on: MoveX -> 0, MoveY -> |override|");
        Check(c.FilterHeading(0.0f, 5.01) == 0.0f, "Locomotion on: heading -> 0");
        Check(std::fabs(c.RawAngle(5.01) - 45.0f) < 1e-3f, "RawAngle follows the override (-0.3, 0.3 = 45 deg left)");
        Check(!c.RealHasInput(5.01) && c.RealMagnitude(5.01) == 0.0f, "no hardware input while only the override is set");
        c.FilterAxis(Axis::X, 0.8f, 5.02);
        c.FilterAxis(Axis::Y, 0.0f, 5.02);
        Check(std::fabs(c.RealMagnitude(5.02) - 0.8f) < 1e-6f && std::fabs(c.RealAngle(5.02) - (-90.0f)) < 1e-3f,
              "hardware stick (right) readable while the override is active");
        c.ClearMoveOverride();
        Check(std::fabs(c.RawAngle(5.02) - (-90.0f)) < 1e-3f, "Clear returns the effective stick to the hardware values at once");
        Check(!c.OverrideActive(5.02), "cleared");
    }
    // Gates: not installed / killed / invalid.
    {
        Core c;
        Check(!c.SetMoveOverride(0.1f, 0.1f, 0.0f, 0.2, 1.0), "rejected before the hook is installed");
        c.SetInstalled(true);
        Check(!c.SetMoveOverride(0.1f, 0.1f, 0.0f, 0.0, 1.0) && !c.SetMoveOverride(NAN, 0.1f, 0.0f, 0.2, 1.0) &&
                  !c.SetMoveOverride(0.1f, 0.1f, 0.0f, 0.2, NAN),
              "rejected: ttl 0 / NaN stick / NaN clock");
        Check(c.SetMoveOverride(3.0f, 4.0f, 0.0f, 99.0, 1.0), "oversized stick and ttl are clamped, not rejected");
        float x, y, h;
        Check(c.GetMoveOverride(1.0, x, y, h) && std::fabs(std::sqrt(x * x + y * y) - 1.0f) < 1e-5f,
              "stick clamped to the unit disc");
        Check(c.OverrideActive(1.49) && !c.OverrideActive(1.5), "ttl clamped to kMaxOverrideTtlSec (0.5 s)");
        c.Kill();
        Check(!c.OverrideActive(1.1) && c.FilterAxis(Axis::X, 0.25f, 1.1) == 0.25f, "kill switch drops a live override");
        Check(!c.SetMoveOverride(0.1f, 0.1f, 0.0f, 0.2, 1.2), "rejected after the kill switch");
    }
    // New call replaces the old one; heading wrapped.
    {
        Core c;
        c.SetInstalled(true);
        c.SetMoveOverride(0.1f, 0.2f, 190.0f, 0.2, 2.0);
        c.SetMoveOverride(0.5f, 0.6f, 190.0f, 0.2, 2.1);
        float x, y, h;
        Check(c.GetMoveOverride(2.15, x, y, h) && x == 0.5f && y == 0.6f && std::fabs(h - (-170.0f)) < 1e-4f,
              "replaced by the latest call; heading wrapped to (-180, 180]");
        Check(c.OverrideSets() == 2, "set counter");
    }
}

// The approach controller drives the override channel: every command of a ride (ride 5 like: facing away, 0.5 m; ride 4 like: waypoint, turn
// first, facing phase) must pass the Locomotion gates - a stick below the deadzone would be ignored (no walk, no body turn), the body-turn target
// (RawAngle) must follow the commanded angle, and the effective MoveY (|stick|) must stay in the walk range below the matcher's Walk/Run split.
static void TestApproachChannel()
{
    using namespace utpp::freemove;
    namespace ap = utpp::approach;
    struct Ride
    {
        ap::V2 start;
        float yaw;
    };
    const Ride rides[] = {{{-1.75f, -0.54f}, -5.0f}, {{-2.18f, 0.0f}, -87.0f}, {{-2.40f, -0.10f}, -21.0f}};
    ap::Scene sc;
    sc.box = ap::BoxPoly(0.0f, 0.0f, 0.95f, 2.4f, 0.35f);
    const ap::V2 spot{-2.11f, -0.91f};
    const ap::Params prm;
    bool started = true, angleOk = true, magOk = true, walkOk = true, anyFacing = false;
    const float refYaw = 117.0f;
    for (const Ride& r : rides)
    {
        ap::Approach a;
        if (a.Begin(sc, prm, r.start, r.yaw, spot, -80.8f) != ap::BeginResult::Started)
        {
            started = false;
            continue;
        }
        Core c;
        c.SetInstalled(true);
        c.SetEnabled(true);
        double now = 20.0;
        c.Heartbeat(now, true, 1.5f, 0);
        ap::V2 pos = r.start;
        float yaw = r.yaw;
        ap::V2 vel;
        float prevX = 0.0f, prevY = 0.0f;
        for (int i = 0; i < 600; ++i)
        {
            now += 1.0 / 60.0;
            c.Heartbeat(now, true, 1.5f, 0);
            ap::Command cmd;
            const ap::Result res = a.Step(pos, yaw, refYaw, 1.0f / 60.0f, true, 0.0f, 0.0f, cmd);
            if (res != ap::Result::Running)
                break;
            if (!c.SetMoveOverride(cmd.stickX, cmd.stickY, cmd.headingDeg, prm.ttlS, now))
                started = false;
            const float x = c.FilterAxis(Axis::X, 0.0f, now);
            const float y = c.FilterAxis(Axis::Y, 0.0f, now);
            const float mag = std::sqrt(cmd.stickX * cmd.stickX + cmd.stickY * cmd.stickY);
                        // MoveX is judged against the previous frame's MoveY (engine order X then Y): frames with a sudden change of the command are skipped
            const bool settled = i > 0 && std::fabs(cmd.stickX - prevX) < 0.03f && std::fabs(cmd.stickY - prevY) < 0.03f;
            const bool magThis = mag >= 0.15f + 0.02f && (!settled || (x == 0.0f && std::fabs(y - mag) < 1e-5f));
            prevX = cmd.stickX;
            prevY = cmd.stickY;
            if (!magThis && magOk)
                std::printf("  approach channel: first bad frame %d phase %d mag %.4f x %.4f y %.4f cmd (%.3f, %.3f)\n", i, a.Phase(), static_cast<double>(mag),
                            static_cast<double>(x), static_cast<double>(y), static_cast<double>(cmd.stickX), static_cast<double>(cmd.stickY));
            magOk = magOk && magThis;
            if (a.Phase() == 0)
                walkOk = walkOk && mag <= 0.7f - 0.02f;
            anyFacing = anyFacing || a.Phase() == 2;
            const float want = ap::WrapDeg(cmd.headingDeg);
            angleOk = angleOk && c.HasInput(now) && std::fabs(ap::WrapDeg(c.RawAngle(now) - want)) < 0.5f;
            // plant: velocity along the stick world direction, body turns after it
            const float spd = 2.3f * mag;
            const ap::V2 target = ap::Fwd(refYaw + c.RawAngle(now)) * spd;
            vel = vel + (target - vel) * (1.0f - std::exp(-(1.0f / 60.0f) / 0.2f));
            pos = pos + vel * (1.0f / 60.0f);
            yaw = ap::WrapDeg(yaw + std::clamp(ap::WrapDeg(refYaw + c.RawAngle(now) - yaw), -4.5f, 4.5f));
        }
        c.ClearMoveOverride();
    }
    Check(started, "approach rides: begin and every override set accepted");
    Check(magOk, "approach rides: every command is above the Locomotion deadzone and arrives as MoveX 0 / MoveY |stick|");
    Check(angleOk, "approach rides: RawAngle (body-turn target) follows every commanded angle, including the low turn / facing sticks");
    Check(walkOk, "approach rides: walking sticks stay below the Walk/Run split (0.7)");
    Check(anyFacing, "approach rides: a facing phase ran");
}

static void TestThreads()
{
    using namespace utpp::freemove;
    Core c;
    c.SetInstalled(true);
    std::atomic<bool> stop{false};
    std::atomic<int> torn{0};
    std::atomic<uint64_t> applied{0};
    std::atomic<int> readersRunning{0};
    constexpr int kReaders = 3;
    // Start order is part of the test: the writer begins only after every reader thread runs its loop, and it does not stop before the readers
    // took at least one live snapshot (a loaded machine can schedule the writer for its whole 200k updates before a reader thread gets a time slice,
    // which left the readers with nothing to read). The 200k updates stay the minimum; a run in which no reader ever sees a live override still
    // fails (hard wall limit).
    // Writer: x = y = k / 1000, heading = k / 10 (so |heading| = 100 x); time = k * 1e-4. Readers verify the invariant in one snapshot.
    std::thread writer(
        [&]
        {
            const auto begin = std::chrono::steady_clock::now();
            while (readersRunning.load() < kReaders && std::chrono::steady_clock::now() - begin < std::chrono::seconds(20))
                std::this_thread::yield();
            for (int k = 1; !stop.load(); ++k)
            {
                const float v = static_cast<float>((k % 700) + 1) / 1000.0f; // |(v, v)| < 1: no unit-disc clamp
                c.SetMoveOverride(v, v, v * 100.0f, 0.5, 1000.0);
                if (k % 7 == 0)
                    c.ClearMoveOverride();
                if (k >= 200000 && (applied.load() > 0 || std::chrono::steady_clock::now() - begin >= std::chrono::seconds(20)))
                    break;
            }
            stop.store(true);
        });
    std::vector<std::thread> readers;
    for (int r = 0; r < kReaders; ++r)
        readers.emplace_back(
            [&]
            {
                readersRunning.fetch_add(1);
                while (!stop.load())
                {
                    float x, y, h;
                    if (c.GetMoveOverride(1000.1, x, y, h))
                    {
                        applied.fetch_add(1);
                        if (x != y || std::fabs(h - x * 100.0f) > 1e-3f)
                            torn.fetch_add(1);
                    }
                    const float ax = c.FilterAxis(Axis::X, 0.0f, 1000.1);
                    const float hd = c.FilterHeading(0.0f, 1000.1);
                    (void)ax;
                    (void)hd;
                }
            });
    writer.join();
    for (auto& t : readers)
        t.join();
    Check(torn.load() == 0, "seqlock: no torn (x, y, heading) snapshot in 200k concurrent updates");
    Check(applied.load() > 0, "readers saw live overrides during the run");
}

int main()
{
    TestBitIdentity();
    TestSemantics();
    TestApproachChannel();
    TestThreads();
    std::printf("utpp_locomotion_override_test: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
