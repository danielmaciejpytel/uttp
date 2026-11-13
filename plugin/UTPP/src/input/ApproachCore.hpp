// UTPP vehicle approach walk (pure logic, no Windows / RED4ext dependency, offline-testable).
//
// When V chooses "enter" on a car, the engine slides the entity to the entry clip's start spot in a fixed time (~0.3 s), which
// at 1-3 m looks like skating. The approach walks V to that spot first (stick override, LocomotionCore.hpp), then the mount is
// issued with ~0 m left to slide. This file plans the walk and steers it; utpp_vehicle_approach.reds feeds it the poses once
// per PSM locomotion update and applies the result.
//
// Frame: the vehicle frame (origin = vehicle entity origin, x right, y forward, metres). Yaw: UTPP convention, degrees,
// counter-clockwise, 0 = +y (forward(yaw) = (-sin, cos)); a stick angle a (LocomotionCore RawAngleDeg) is a world yaw offset
// from the reference yaw (camera yaw with Locomotion active, body yaw otherwise): world yaw = refYaw + a.
//
// Planner: a straight line start -> walk's end (see "Plan choice" below), optionally through a "run-up" point 0.6 m behind the spot along
// the clip's start heading (so the last leg is aligned with the start pose), detoured around convex obstacles
// (the vehicle footprint box inflated by 0.35 m and the swept sector of the closed front door) with at most two extra
// waypoints. The last 0.7 m before the spot are exempt from the obstacle test (the spot lies beside the car by design).
// Controller: cruise at a walking speed, slow down in the last 0.4 m; the stick-to-speed gain is unknown and estimated from the
// measured speed. The stick magnitude stays below the Walk/Run split of the matcher (0.7; a larger stick selects run-start
// clips), rises and falls at a bounded rate (no step), is held low while the body is still more than faceFirstDeg away from
// the leg direction (turn first, then walk: no backpedal at walking speed), and the speed is capped before a sharp waypoint
// corner. The speed gain is only re-estimated while the stick is steady. A low-stick stall near the spot (the engine does not
// move at a very small stick) is lifted by an integral boost and, as a last resort, ends the position phase.
// Plan choice (PlanBest): the walk is chosen by an estimated cost (path time, turns at the start / corners / the final facing turn, a
// penalty for a pirouette of more than 200 deg in one direction, and the predicted distance left to the spot after the facing turn):
// direct to the spot, direct to a "settle point" behind the spot (against the clip heading, by the distance the facing turn is expected
// to carry V along that heading: the body follows the stick, so the small facing stick walks V forward while she turns), or the
// run-up point. The body follows the stick, so the stick never points further than maxLeadDeg from the body yaw (V arcs round a
// turn instead of walking against her facing: the matcher plays reface / backpedal clips for a larger mismatch) and the heading is
// latched within stableHeadM of a waypoint (no swing while V passes it). The arrival needs the residual to the spot (arriveTightM 0.03 m since r13c, arriveGateM 0.10 m after a creep of creepS)
// as well as the facing; an overshoot ends as NearSpot (mount, the engine slide is short) instead of 'arrived'.
// Position phase done = within 0.03 m of the walk's end (less speedLeadS x speed; the game snaps V's velocity to the next stick, there is no momentum
// that carries her the rest: r5a rides 2026-11-04 ended the old 0.12 m + 0.2 s radius ~0.25 m short and the facing carry then ran on a line parallel
// to the clip heading, 0.17-0.36 m from the spot), or the distance
// started growing again within 0.35 m ("passed the spot": by min(0.08 m, max(0.02 m, 0.25 x the closest approach)) since r11, so a lateral miss of
// 0.06-0.15 m ends at the closest approach), or a stall within 0.3 m. r11: the heading latch is not taken while the turn-first hold may run (a short leg
// latched the bearing of the start), and the facing aim keeps the turn direction near a half turn. Then, when the body yaw is controllable, a facing
// phase turns V (a small stick toward the spot yaw) until her yaw is within arriveYawDeg of the clip's start yaw (bounded by
// turnMaxS) and she stands within arriveTightM of the spot (aligned but short: she creeps the rest along the clip heading; past it or after creepS: arriveGateM); only then Arrived.
// r13c: the settle point behind the spot is placed by the carry model of the facing turn (faceDriftBaseM + faceDriftMPerDeg x turn, x settleFrac) and shifted sideways against the
// lateral drift of the aimed facing stick (faceLatPerDeg x turn): the turn then ends short of the spot, aligned and within 0.03 m (the jump between the last approach sample and the
// first clip sample, entry_feel E6). The model is fitted to the ride plant of the tests (r5a / r6a game logs); a start within skipNearM (0.03 m) of the spot mounts at once.
// r13d (r13c game rides 2026-11-04, 14 of 20 ends within 0.03 m): (1) an aligned V short of the spot walks the rest with the walking stick from arriveTightM on (creepWalks: the engine stops an
// aligned body at the 0.2 facing stick, the old creep never moved and creepS mounted it up to 0.10 m short); (2) the carry of small turns is limited (faceDriftSmallPerDeg: 22 deg carried V
// 0.07-0.10 m, not 0.25 m); (3) the walk's end lies settleBackM further behind the spot (the carry scatters +-0.04 m: short is closed by the aligned walk, past the spot there is no way back).
// r13e (offline, ride plant variants): (1) a close start behind the spot with a sideways offset (sport1 door 0.25 m from the spot) has no settle point on its side: the two-step
// update of the settle point runs away (24 -> 49.5 deg) and the direct walk's facing turn carried V 0.05-0.09 m past the spot; the self-consistent settle points (SettleRoots,
// the far side: a short step away and a larger facing turn back onto the clip line) are planned instead; (2) the walk's end moves back by the carry the measured walk-loop speed / turn gain adds (CarryAdaptShift); (3) the undershoot settleBackM also
// applies to a last leg parallel to the clip heading (a body that lags the leg after a corner still turns at the spot: 0.04 m past it).
// Aborts: timeout, player stick input against / beside the path, no motion at all (the engine does not follow the script
// stick; the clock counts walking time only, not the turn-first hold or the facing phase).

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "../MutationSelect.hpp" // tests only: a runtime-selected mutation (UTPP_RUNTIME_MUTATION_APPROACH); production keeps the constexpr constant

// Test-only mutations (tests/ApproachTest.cpp, CMake utpp_approach_test_mut*): 0 = production behaviour.
//   1 no facing phase (position arrival ends the walk)   2 stick up to 0.85 (Run gait range)   3 no stall handling / boost
//   4 no turn-first hold at the leg start / corners      5 stick steps (no rate limit)
//   6 facing-phase release keeps the stale minD / passed and the facing budget restarts per entry (unbounded facing loop)
//   7 the stick points straight at the waypoint (no lead limit against the body yaw, no heading latch near the waypoint)
//   8 arrival ignores the residual to the spot (the facing walk may end the approach far from it)
//   9 old planner (run-up point whenever the spot is far and off the clip heading, no facing-drift compensation, no cost choice)
//   11 the old carry model of the facing turn (0.08 m + 0.0022 m/deg, at most 0.35 m, settle point at the full carry)
//   13 no lead-limit reset on a facing release (the walk back inherits the lead grown on the first walk)
//   14 no own time budget for the walk back after a facing release
//   18 the old arrival gate (0.15 m from the spot after the facing)
//   19 the old plan cost of the predicted residual (4 s per metre: a close start that needs a 70+ deg turn takes the direct plan and mounts past the spot)
//   16 InputAbort compares the player's stick with the bearing to the settle point also in the facing phase (a push toward the car aborts the mount)
//   15 no stall boost while the facing turn is still running
//   21 the facing release measures the distance to the settle point too (a settle point 0.45+ m behind the spot releases V once the carry passes the spot by 0.15 m)
//   20 no aim past the clip yaw during the facing turn (the slow tail of the turn carries V past the spot)
//   12 no steering of the facing stick toward the spot (the facing carry keeps the lateral miss of the walk)
//   10 the old position-phase end (momentum lead 0.2 s x speed, radius 0.12 m): V stops walking ~0.25 m before the settle point
//   24 the fixed 'passed' growth (0.08 m past the closest approach, whatever the lateral miss)
//   23 the heading latch is taken before the turn-first hold (a short leg latches the bearing of the start)
//   22 no direction guard of the facing aim (near a half turn the aim past the clip yaw flips the turn direction every frame)
//   25 the pre-r13c arrival: gate 0.10 m, carry model 0.08 + 0.0028 m/deg, settle fraction 0.95, no lateral pre-compensation of the settle point (the mount ends 0.04-0.15 m from the spot)
//   26 the steering lean toward the spot does not fade out at the end of the facing budget (a V that does not translate keeps the body off the clip yaw)
//   27 a stick pushed along the clip heading within exemptM of the spot is judged against the (sideways offset) walk bearing only (aborts the walk of a start beside the car)
//   28 the pre-r13d creep: an aligned V short of the spot keeps the 0.2 facing stick inside arriveGateM (the engine stops an aligned body at that stick: the creep never moves, creepS mounts it up to 0.10 m short)
//   29 the pre-r13d carry model for small turns (no 0.0072 m/deg limit below ~40 deg: a 22 deg turn from the spot is expected to carry V 0.25 m, a close start is planned as a direct walk and mounts past the spot)
//   30 no undershoot of the walk's end (settleBackM): the facing carry then ends past the spot as often as short of it, and past the spot there is no way back
//   31 no self-consistent settle points (SettleRoots): a close start behind the spot walks straight at it and the facing turn carries V past it (sport1 door)
//   32 no online carry adaptation (CarryAdaptShift: the slowest turner with the fastest walk of the ride plant variants carries V past the spot)
//   33 no undershoot of a last leg parallel to the clip heading (a body that lags the leg after a corner turns at the spot and passes it)
#ifndef UTPP_APPROACH_MUTATION
#define UTPP_APPROACH_MUTATION 0
#endif

namespace utpp::approach
{
#ifdef UTPP_RUNTIME_MUTATION_APPROACH // tests only (tests/ApproachTest.cpp runtime mutants): the number comes from UTPP_MUTATION_APPROACH
inline constexpr mutsel::RuntimeValue<1> kMutation{"UTPP_MUTATION_APPROACH"};
#else
constexpr int kMutation = UTPP_APPROACH_MUTATION;
#endif

constexpr float kPi = 3.14159265358979323846f;

struct V2
{
    float x = 0.0f;
    float y = 0.0f;
};
inline V2 operator+(V2 a, V2 b) { return {a.x + b.x, a.y + b.y}; }
inline V2 operator-(V2 a, V2 b) { return {a.x - b.x, a.y - b.y}; }
inline V2 operator*(V2 a, float k) { return {a.x * k, a.y * k}; }
inline float Dot(V2 a, V2 b) { return a.x * b.x + a.y * b.y; }
inline float Cross(V2 a, V2 b) { return a.x * b.y - a.y * b.x; }
inline float Len(V2 a) { return std::sqrt(a.x * a.x + a.y * a.y); }
inline float Dist(V2 a, V2 b) { return Len(a - b); }
inline bool Finite(V2 a) { return std::isfinite(a.x) && std::isfinite(a.y); }

inline float WrapDeg(float a)
{
    if (!std::isfinite(a))
        return 0.0f;
    a = std::fmod(a, 360.0f);
    if (a > 180.0f)
        a -= 360.0f;
    else if (a <= -180.0f)
        a += 360.0f;
    return a;
}
inline float Rad(float d) { return d * (kPi / 180.0f); }
inline float Deg(float r) { return r * (180.0f / kPi); }
// UTPP yaw of a planar direction (0 = +y, CCW).
inline float YawOf(V2 d) { return Deg(std::atan2(-d.x, d.y)); }
inline V2 Fwd(float yawDeg) { return {-std::sin(Rad(yawDeg)), std::cos(Rad(yawDeg))}; }

// ---- convex obstacles ----
constexpr int kMaxPoly = 16;
struct Poly
{
    V2 v[kMaxPoly];
    int n = 0;
    float margin = 0.0f; // clearance the path keeps (m)
    V2 Centroid() const
    {
        V2 c;
        for (int i = 0; i < n; ++i)
            c = c + v[i];
        return n ? c * (1.0f / static_cast<float>(n)) : c;
    }
};

inline float PointSegDist(V2 p, V2 a, V2 b)
{
    const V2 ab = b - a;
    const float l2 = Dot(ab, ab);
    const float t = l2 > 1e-12f ? std::clamp(Dot(p - a, ab) / l2, 0.0f, 1.0f) : 0.0f;
    return Dist(p, a + ab * t);
}

// Distance from p to the polygon (0 inside). Works for either winding.
inline float PolyDist(const Poly& g, V2 p)
{
    if (g.n < 3)
        return 1e9f;
    bool pos = false, neg = false;
    float best = 1e9f;
    for (int i = 0; i < g.n; ++i)
    {
        const V2 a = g.v[i];
        const V2 b = g.v[(i + 1) % g.n];
        const float c = Cross(b - a, p - a);
        pos = pos || c > 1e-9f;
        neg = neg || c < -1e-9f;
        best = std::min(best, PointSegDist(p, a, b));
    }
    return (pos && neg) ? best : 0.0f;
}

inline Poly BoxPoly(float cx, float cy, float hx, float hy, float margin)
{
    Poly g;
    g.n = 4;
    g.v[0] = {cx - hx, cy - hy};
    g.v[1] = {cx + hx, cy - hy};
    g.v[2] = {cx + hx, cy + hy};
    g.v[3] = {cx - hx, cy + hy};
    g.margin = margin;
    return g;
}

// Swept sector of a hinged door (door_timing.json arc contract: closedDirDeg = atan2(dy, dx) from +x of the hinge -> rear edge
// direction with the door closed, openDeg = signed peak swing). radius = length + 0.20 (the cap's body radius), the sector is
// widened by `extraDeg` on both ends and the chord polygon is inflated so it contains the arc.
inline Poly SectorPoly(V2 hinge, float length, float closedDirDeg, float openDeg, float extraDeg)
{
    Poly g;
    const float r = (length + 0.20f) * 1.02f;
    const float sgn = openDeg < 0.0f ? -1.0f : 1.0f;
    const float a0 = closedDirDeg - sgn * extraDeg;
    const float span = std::fabs(openDeg) + 2.0f * extraDeg;
    g.v[0] = hinge;
    const int steps = 6;
    for (int i = 0; i <= steps; ++i)
    {
        const float a = Rad(a0 + sgn * span * static_cast<float>(i) / static_cast<float>(steps));
        g.v[1 + i] = hinge + V2{std::cos(a), std::sin(a)} * r;
    }
    g.n = 2 + steps;
    g.margin = 0.15f;
    return g;
}

struct Scene
{
    Poly box;     // vehicle footprint, margin = path clearance
    Poly sector;  // closed-door sweep (n = 0: none)
    bool haveSector = false;
};

struct Params
{
    float cruiseMps = 1.4f;
    float finalSlowM = 0.4f;
    float finalMps = 0.45f;
    float minSpeedMps = 0.28f;
    float minStick = 0.22f;
    float maxStick = kMutation == 2 ? 0.85f : 0.66f; // below the matcher's Walk/Run split (0.7): a larger stick selects run-start clips
    float gain0 = 2.3f;        // initial guess: m/s per unit stick (game 2.5 walk range: ~0.6 stick = 1.4 m/s)
    float arriveM = 0.03f;      // position phase done within this of the walk's end (less speedLeadS x speed). The game snaps V's velocity direction to the
                                // stick (r5a rides 2026-11-04: no momentum), so what is left to the walk's end at this moment is a lateral miss of the facing carry
    float arriveGateM = kMutation == 18 ? 0.15f : 0.10f;  // the mount needs V within this of the spot after the facing (else NearSpot / keep facing)
    float arriveTightM = kMutation == 25 ? 0.10f : 0.03f; // r13c target: the logged ends of the r6a game test were 0.04-0.15 m from the spot = the visible jump between the last
                                // approach sample and the first clip sample (entry_feel E6, limit 0.03 m). Aligned and short of the spot V creeps along the clip heading up to this;
                                // past the spot, or when the creep does not close the gap within creepS, the looser arriveGateM still mounts. The settle point sits behind the spot so that
                                // the facing turn ends short of it (settleFrac) and the lateral drift of the aimed facing stick is pre-compensated (faceLatPerDeg)
    float creepS = 0.3f;        // longest creep (aligned, within arriveGateM of the spot but not within arriveTightM, short of it) before the looser gate mounts
    float maxLeadDeg = 60.0f;   // the stick direction never leads the body yaw by more than this while walking / holding (body controllable)
    float leadGrowDegPerS = 90.0f; // the allowed lead grows by this per second the body yaw is nearly still (below leadStuckDegPerS): a body that
                                   // does not follow the stick still ends in a direct walk
    float leadStuckDegPerS = 45.0f;
    float speedLeadS = 0.02f;   // the position phase ends this long (x speed) before the walk's end: the command takes effect a tick later. The old 0.2 s
                                // assumed a momentum that carries V onto the walk's end; in the game V's velocity follows the next stick at once
    float stableHeadM = 0.35f;  // heading latched within this of a waypoint
    float faceDriftBaseM = kMutation == 25 ? 0.08f : 0.20f; // carry of any facing turn (the walk loops move V at ~0.8 m/s from the first tick of the facing stick)
    float faceDriftMPerDeg = kMutation == 25 ? 0.0028f : 0.0022f; // expected carry of the facing turn along the clip heading (r5a rides 2026-11-04: 50 deg -> 0.31 m, 76 deg -> 0.40 m, 150 deg -> 0.49 m;
                                // r13c refit on the ride plant with V arriving at walking speed: 120 deg -> 0.46 m, 179 deg -> 0.59 m)
    float faceDriftMaxM = kMutation == 25 ? 0.5f : 0.6f;
    float faceDriftSmallPerDeg = kMutation == 29 || kMutation == 25 ? 0.0f : 0.0072f; // r13d: small turns carry V by about this per degree beyond the facing tolerance (r13c rides: 22 deg -> 0.07..0.10 m, 41 deg -> 0.27 m);
                                // the linear model above (0.20 m base) over-predicts them by 0.1-0.15 m. 0 = off (the older model)
    bool creepWalks = kMutation != 28 && kMutation != 25;  // r13d: an aligned V short of the spot walks the rest with the walking stick (>= minStick) from arriveTightM on, not only beyond arriveGateM
    float faceLatPerDeg = kMutation == 25 ? 0.0f : 0.0016f; // lateral drift (m per deg of the facing turn, toward the side the body turns to) of the aimed facing stick (faceLeadDeg): the settle point is shifted the other way
    float settleFrac = kMutation == 25 ? 0.95f : 0.92f;     // the settle point sits this fraction of the expected carry behind the spot: short is closed by the aligned walk, past it is not
    float costTurnDegPerS = 300.0f; // plan cost model: body turn rate
    float costRefaceS = 0.15f;      // extra for a first turn of more than 100 deg (reface clip)
    float costCornerS = 0.3f;       // per 90 deg at a corner (slow-down)
    float costPivotS = 0.25f;       // per 90 deg of the facing turn (pivot clips look worse than an arc)
    float costResidualPerM = kMutation == 19 ? 4.0f : 6.0f;  // per metre the facing turn is expected to leave V from the spot
    float settleBackM = kMutation == 30 || kMutation == 25 ? 0.0f : 0.04f; // r13d: the walk's end (settle point) is moved this far further behind the spot (along the clip heading): the carry then ends short of the
                                // spot more often than past it (short: the aligned rest is walked, ~0.1 s per 0.08 m; past: no way back, the mount slides V up to arriveGateM)
    float carryAdaptGain = kMutation == 32 ? 0.0f : 0.5f; // r13e: online carry adaptation (CarryAdaptShift), 0 = off
    float carryAdaptRefMps = 0.82f;   // walk-loop speed at the facing / hold stick the carry model is fitted to (ride plant)
    float carryAdaptRefK = 4.2f;      // body turn gain (deg/s per deg of stick-to-body error) the carry model is fitted to
    float carryAdaptMaxM = 0.06f;     // largest shift of the walk's end
    bool settleBackParallel = true; // r13e: the undershoot also applies to a last leg parallel to the clip heading
    float settleRootMaxM = 0.6f;    // r13e: starts within this of the spot also get the self-consistent settle points (SettleRoots) as plan candidates; 0 = off
    float settleRootJumpDeg = 30.0f; // r13e: ... only when the two-step update of the settle point does not converge (its bearing still moves by more than this)
    float settleRootK = 0.6f;       // r13e: carry factor of those settle points (a short leg ends before the body has turned onto it)
    float costLoopDeg = 200.0f;     // more rotation than this in one direction is a pirouette
    float costLoopS = 1.0f;         // per 90 deg beyond costLoopDeg
    float arriveYawDeg = 8.0f; // facing tolerance before the mount (body yaw vs the clip's start yaw)
    float turnMaxS = 1.2f;     // longest TOTAL facing time at the spot (summed over re-entries after a drift)
    float turnStick = 0.2f;    // facing / turn-first stick (above the Locomotion deadzone 0.15)
    float stickRiseRate = 1.3f; // stick units per second (no step: the matcher would see a run start)
    float stickFallRate = 3.0f;
    float faceFirstDeg = 80.0f; // body further than this from the leg direction: low stick until it has turned (turn first)
    float alignMaxS = 0.7f;     // longest turn-first hold per leg
    float cornerSlowM = 0.8f;   // speed cap towards a waypoint with a sharp corner, from this distance
    float cornerSlowMps = 0.7f;
    float stallRadiusM = 0.3f;  // stall = (nearly) no movement within this distance of the spot
    float stallSpeedMps = 0.1f;
    float stallS = 0.35f;
    float boostRate = 1.0f;     // stick units per second added while the speed stays below boostSpeedFrac of the wanted speed
    float boostMax = 0.25f;
    float boostSpeedFrac = 0.6f;
    float boostRangeM = 0.7f;
    float faceBoostAfterS = 0.0f;   // facing phase: the stall boost starts this long after the facing began (the speed ramps up first)
    float faceLeadDeg = 12.0f;  // facing phase: the stick aims this far past the clip yaw (toward the turn) while the yaw error is above arriveYawDeg: the body turn
                                // slows in proportion to the remaining error (r5a rides: 46 deg -> 200 deg/s, 21 deg -> 104 deg/s), the tail of the turn is where V walks on
    float steerMaxDeg = 35.0f;  // facing phase: the stick leans at most this far from the clip heading toward the spot (closes the lateral miss)
    float steerBudgetFrac = 0.35f; // facing phase: the steering lean fades to 0 over the last this fraction of turnMaxS (0.42 s)
    float steerLookM = 0.12f;
    float steerFadeM = 0.2f;
    float releaseMaxS = 1.6f;   // longest walk back after a facing release (then: mount within nearMountM, else no mount)
    float faceReleaseM = 0.6f;  // the facing phase drifted this far from the spot: back to walking
    float passedTriggerM = 0.35f;
    float passedGrowM = 0.08f;       // 'passed': the distance to the walk's end grew by this much (at most) since the closest approach
    float passedGrowMissFrac = 0.25f; // r11: ... or by this fraction of the closest approach (the lateral miss), at least passedGrowMinM: a walk that misses the
    float passedGrowMinM = 0.02f;     // walk's end sideways by 0.06-0.15 m ends at the closest approach instead of 0.11-0.19 m of walking past it
    float timeoutS = 4.0f;
    float nearMountM = 0.6f;
    float graceS = 0.3f;
    float abortStick = 0.5f;
    float abortAngleDeg = 75.0f;
    float abortSpotBearingM = 0.15f; // facing phase: the abort test also accepts the bearing to the spot when V is farther than this from it
    float noMotionAfterS = 0.45f;
    float noMotionMinM = 0.08f;
    float noMotionPathM = 0.6f; // the no-motion guard only judges walks planned at least this long (a turn-only approach legitimately stays put)
    float waypointM = 0.25f;
    float skipNearM = kMutation == 25 ? 0.35f : 0.03f; // r13c: was 0.35 m (V that close mounted at once and the engine slid her up to 0.35 m = the jump the arrival gate removes)
    float skipNearYawDeg = 25.0f;
    float exemptM = 0.7f;      // obstacle test off within this of the spot
    float runUpM = 0.6f;
    float runUpMinDistM = 0.9f;
    float runUpAngleDeg = 35.0f;
    float ttlS = 0.25f;        // override lifetime per tick
    bool allowThroughClosedDoor = false; // false (default): a walk that cannot keep clear of the closed door's sweep sector is not started (vanilla slide)
    float maxPathM = 3.0f;     // longer walks are not started (vanilla slide); 3.0 m is ~2.1 s at cruise speed
};

constexpr int kMaxPath = 8;

enum class BeginResult : int
{
    AlreadyThere = 0, // close enough: mount at once
    Started = 1,
    Invalid = -1,     // non-finite input
    NoPath = -2,      // no detour found
    TooFar = -3,      // planned path longer than Params::maxPathM
};

enum class Result : int
{
    Running = 0,
    Arrived = 1,    // within the arrival radius (or passed the spot): mount
    MountNear = 2,  // timeout but within nearMountM: mount (the slide is short)
    Timeout = 3,    // timeout far from the spot: do not mount
    InputAbort = 4, // the player pushed the stick against / beside the path: do not mount
    NoMotion = 5,   // V did not move although the stick was set: the engine ignores the script stick
    NearSpot = 6,   // facing done (or its budget spent) but V stands more than arriveGateM from the spot (within nearMountM): mount, the slide is short
};

inline const char* ResultName(Result r)
{
    switch (r)
    {
    case Result::Running: return "running";
    case Result::Arrived: return "arrived";
    case Result::MountNear: return "timeout_near";
    case Result::Timeout: return "timeout";
    case Result::InputAbort: return "input";
    case Result::NoMotion: return "no_motion";
    case Result::NearSpot: return "near_spot";
    }
    return "?";
}

struct Command
{
    float stickX = 0.0f;
    float stickY = 0.0f;
    float headingDeg = 0.0f;
    float speedWantMps = 0.0f;
};

class Approach
{
public:
    // ---- planning ----
    static bool SegmentOk(const Scene& aS, const Params& aP, V2 aA, V2 aB, V2 aTarget)
    {
        const float len = Dist(aA, aB);
        const int n = std::max(2, static_cast<int>(std::ceil(len / 0.05f)));
        const Poly* polys[2] = {&aS.box, aS.haveSector ? &aS.sector : nullptr};
        for (const Poly* g : polys)
        {
            if (!g)
                continue;
            // allowed clearance = min(margin, best clearance reached so far on this segment): a start inside the margin zone may
            // move away, but never closer than where it already was
            float best = PolyDist(*g, aA);
            for (int i = 1; i <= n; ++i)
            {
                const V2 p = aA + (aB - aA) * (static_cast<float>(i) / static_cast<float>(n));
                const float d = PolyDist(*g, p);
                const bool exempt = Dist(p, aTarget) <= aP.exemptM;
                if (!exempt && d < std::min(g->margin, best) - 0.03f)
                    return false;
                best = std::max(best, d);
            }
        }
        return true;
    }

    static float PathLen(const V2* aPts, int aN)
    {
        float l = 0.0f;
        for (int i = 1; i < aN; ++i)
            l += Dist(aPts[i - 1], aPts[i]);
        return l;
    }

    // Plans the polyline of the old rule (mutation 9 and the planner tests): through a run-up point 0.6 m behind the spot along the clip
    // heading when the spot is farther than runUpMinDistM and more than runUpAngleDeg off that heading. out[0] = start, out[n-1] = spot.
    static int Plan(const Scene& aS, const Params& aP, V2 aStart, V2 aSpot, float aSpotYaw, V2* aOut)
    {
        V2 pts[3];
        int np = 0;
        pts[np++] = aStart;
        const float bearing = YawOf(aSpot - aStart);
        if (Dist(aStart, aSpot) > aP.runUpMinDistM && std::fabs(WrapDeg(bearing - aSpotYaw)) > aP.runUpAngleDeg)
        {
            const V2 pre = aSpot - Fwd(aSpotYaw) * aP.runUpM;
            if (PolyDist(aS.box, pre) > 0.05f)
                pts[np++] = pre;
        }
        pts[np++] = aSpot;
        return PlanThrough(aS, aP, pts, np, aSpot, aOut);
    }

    // Expected carry (m) of a facing turn of aTurnDeg along the clip heading.
    static float FaceDriftM(const Params& aP, float aTurnDeg)
    {
        if (aTurnDeg <= aP.arriveYawDeg)
            return 0.0f;
        if (kMutation == 11) // old carry model
            return std::min(0.35f, 0.08f + 0.0022f * aTurnDeg);
        float c = std::min(aP.faceDriftMaxM, aP.faceDriftBaseM + aP.faceDriftMPerDeg * aTurnDeg);
        if (aP.faceDriftSmallPerDeg > 0.0f)
            c = std::min(c, aP.faceDriftSmallPerDeg * (aTurnDeg - aP.arriveYawDeg));
        return c;
    }

    // Expected lateral drift (m) of a facing turn of aTurnDeg toward the side the body turns to (the aimed facing stick leads the clip yaw by faceLeadDeg).
    static float FaceLatM(const Params& aP, float aTurnDeg)
    {
        return aTurnDeg <= aP.arriveYawDeg ? 0.0f : aP.faceLatPerDeg * aTurnDeg;
    }

    // Settle point of a walk whose last leg has the bearing aLegYaw: behind the spot by the expected carry of the facing turn (aLegYaw -> clip yaw), sideways
    // against its lateral drift. aAlongK scales the carry along the clip heading.
    static V2 SettleFor(const Params& aP, V2 aSpot, float aSpotYaw, float aLegYaw, float aAlongK)
    {
        const V2 fwd = Fwd(aSpotYaw);
        const V2 left{-fwd.y, fwd.x};
        const float rs = WrapDeg(aSpotYaw - aLegYaw);
        const float rf = std::fabs(rs);
        if (rf <= aP.arriveYawDeg)
            return aSpot;
        const float c = FaceDriftM(aP, rf) * aAlongK;
        const float l = FaceLatM(aP, rf) * (rs > 0.0f ? 1.0f : -1.0f);
        return aSpot - fwd * (c * aP.settleFrac) - left * (l * aP.settleFrac);
    }

    // r13e: self-consistent settle points. The settle point depends on the bearing of the leg that reaches it, and that bearing on the settle point; the
    // two-step update of PlanBest diverges for a start close to the spot (sport1 door 0.25 m from the spot, 11 deg off: 24 -> 49.5 deg, the settle point
    // then lies behind the start and only the direct walk is left; its facing turn carries V 0.05-0.09 m past the spot). This scans the leg bearing for
    // the fixed points (bearing of the settle point seen from the start == the leg bearing), both turn directions, and returns them as walk ends. A
    // short leg ends before the body has turned onto it: the facing turn is smaller than the leg bearing says, the carry along the clip heading is
    // scaled by settleRootK.
    static constexpr int kMaxRoots = 3;
    static int SettleRoots(const Params& aP, V2 aStart, V2 aSpot, float aSpotYaw, V2* aOut)
    {
        const auto g = [&](float b) { return WrapDeg(YawOf(SettleFor(aP, aSpot, aSpotYaw, b, aP.settleRootK) - aStart) - b); };
        int n = 0;
        float b0 = -180.0f, g0 = g(b0);
        for (int i = 1; i <= 360 && n < kMaxRoots; ++i)
        {
            const float b1 = -180.0f + static_cast<float>(i);
            const float g1 = g(b1);
            if (g0 * g1 <= 0.0f && std::fabs(g0) < 45.0f && std::fabs(g1) < 45.0f)
            {
                float lo = b0, hi = b1, glo = g0;
                for (int k = 0; k < 24; ++k)
                {
                    const float m = 0.5f * (lo + hi);
                    const float gm = g(m);
                    if (gm * glo <= 0.0f)
                        hi = m;
                    else
                    {
                        lo = m;
                        glo = gm;
                    }
                }
                const float b = 0.5f * (lo + hi);
                if (std::fabs(WrapDeg(aSpotYaw - b)) > aP.arriveYawDeg)
                    aOut[n++] = SettleFor(aP, aSpot, aSpotYaw, b, aP.settleRootK);
            }
            b0 = b1;
            g0 = g1;
        }
        return n;
    }

    // Cost of a planned polyline (see the header): lower is better. aPts[aN-1] is the walk's end (the spot or the settle point).
    static float PlanCost(const Params& aP, const V2* aPts, int aN, float aBodyYaw, V2 aSpot, float aSpotYaw, float aAlongK = 1.0f)
    {
        float cost = PathLen(aPts, aN) / aP.cruiseMps;
        float head = aBodyYaw, cum = 0.0f, firstTurn = 0.0f;
        int leg = 0;
        for (int i = 1; i < aN; ++i)
        {
            const V2 d = aPts[i] - aPts[i - 1];
            if (Len(d) < 0.08f)
                continue;
            const float b = YawOf(d);
            const float r = WrapDeg(b - head);
            const float a = std::fabs(r);
            cost += a / aP.costTurnDegPerS + (leg == 0 ? (a > 100.0f ? aP.costRefaceS : 0.0f) : aP.costCornerS * a / 90.0f);
            cum += r;
            if (leg == 0)
                firstTurn = r;
            head = b;
            ++leg;
        }
        const float rf = WrapDeg(aSpotYaw - head);
        const float af = std::fabs(rf);
        V2 end = aPts[aN - 1];
        if (af > aP.arriveYawDeg)
        {
            cost += af / aP.costTurnDegPerS + aP.costPivotS * af / 90.0f;
            cum += rf;
            const V2 f = Fwd(aSpotYaw);
            end = end + f * (FaceDriftM(aP, af) * aAlongK) + V2{-f.y, f.x} * ((rf > 0.0f ? 1.0f : -1.0f) * FaceLatM(aP, af));
        }
        cost += aP.costResidualPerM * Dist(end, aSpot);
        float loop = std::fabs(cum);
        if (std::fabs(firstTurn) > 150.0f)
        {
            // a first turn near 180 deg may go either way (the engine picks by the sign of a tiny angle): judge the pirouette by the better sense
            const float other = firstTurn - (firstTurn > 0.0f ? 360.0f : -360.0f);
            loop = std::min(loop, std::fabs(cum - firstTurn + other));
        }
        if (loop > aP.costLoopDeg)
            cost += aP.costLoopS * (loop - aP.costLoopDeg) / 90.0f;
        return cost;
    }

    // Plans the walk by cost: direct to the spot, direct to a settle point behind the spot (the facing turn then carries V onto the
    // spot), or through the run-up point. aOut[n-1] is the walk's end (the settle point or the spot). Returns the point count (0 = none).
    static int PlanBest(const Scene& aS, const Params& aP, V2 aStart, float aStartYaw, V2 aSpot, float aSpotYaw, V2* aOut)
    {
        if (kMutation == 9)
            return Plan(aS, aP, aStart, aSpot, aSpotYaw, aOut);
        const V2 fwd = Fwd(aSpotYaw);
        constexpr int kMaxCand = 3 + kMaxRoots;
        V2 cand[kMaxCand][3];
        int candN[kMaxCand] = {};
        float candK[kMaxCand], candExtra[kMaxCand];
        for (int i = 0; i < kMaxCand; ++i)
        {
            candK[i] = 1.0f;
            candExtra[i] = 0.0f;
        }
        int nc = 0;
        // direct to the spot
        cand[nc][0] = aStart;
        cand[nc][1] = aSpot;
        candN[nc++] = 2;
        // direct to the settle point
        float settleJump = 0.0f; // how far the bearing of the settle point still moved in the second update (deg): large = the update does not converge
        {
            const float rs0 = WrapDeg(aSpotYaw - YawOf(aSpot - aStart));
            const float rf0 = std::fabs(rs0);
            const V2 left{-fwd.y, fwd.x};
            const float frac = kMutation == 11 ? 1.0f : aP.settleFrac;
            float c = rf0 > aP.arriveYawDeg ? FaceDriftM(aP, rf0) : 0.0f;
            float l = rf0 > aP.arriveYawDeg ? FaceLatM(aP, rf0) * (rs0 > 0.0f ? 1.0f : -1.0f) : 0.0f;
            V2 settle = aSpot - fwd * (c * frac) - left * (l * frac);
            if (c > 0.0f)
            {
                const float rs1 = WrapDeg(aSpotYaw - YawOf(settle - aStart));
                const float rf1 = std::fabs(rs1);
                c = rf1 > aP.arriveYawDeg ? FaceDriftM(aP, rf1) : 0.0f;
                l = rf1 > aP.arriveYawDeg ? FaceLatM(aP, rf1) * (rs1 > 0.0f ? 1.0f : -1.0f) : 0.0f;
                settle = aSpot - fwd * (c * frac) - left * (l * frac);
                settleJump = std::fabs(WrapDeg(WrapDeg(aSpotYaw - YawOf(settle - aStart)) - rs1));
            }
            c *= frac;
            if (c >= 0.03f && PolyDist(aS.box, settle) > 0.05f)
            {
                cand[nc][0] = aStart;
                cand[nc][1] = settle;
                candN[nc++] = 2;
            }
        }
        // r13e: self-consistent settle points of a close start (see SettleRoots)
        if (kMutation != 31 && aP.settleRootMaxM > 0.0f && Dist(aStart, aSpot) <= aP.settleRootMaxM && settleJump > aP.settleRootJumpDeg)
        {
            // The direct walk's facing turn carries V past the spot whatever the walk's end on that side (that is why the update runs away): the
            // self-consistent settle points (and the run-up point) are preferred; the direct and two-step candidates only plan when none of them does
            V2 roots[kMaxRoots];
            const int nr = SettleRoots(aP, aStart, aSpot, aSpotYaw, roots);
            const int ncBefore = nc;
            for (int i = 0; i < nr && nc < kMaxCand; ++i)
                if (PolyDist(aS.box, roots[i]) > 0.05f && Dist(aStart, roots[i]) > 0.05f)
                {
                    for (int k = 0; k < ncBefore; ++k)
                        candExtra[k] = 50.0f;
                    cand[nc][0] = aStart;
                    cand[nc][1] = roots[i];
                    candK[nc] = aP.settleRootK;
                    candN[nc++] = 2;
                }
        }
        // run-up point
        {
            const V2 pre = aSpot - fwd * aP.runUpM;
            if (PolyDist(aS.box, pre) > 0.05f && Dist(aStart, pre) > 0.15f)
            {
                cand[nc][0] = aStart;
                cand[nc][1] = pre;
                cand[nc][2] = aSpot;
                candN[nc++] = 3;
            }
        }
        float bestCost = 1e9f;
        int bestN = 0;
        for (int i = 0; i < nc; ++i)
        {
            V2 path[kMaxPath];
            const int n = PlanThrough(aS, aP, cand[i], candN[i], aSpot, path);
            if (n < 2)
                continue;
            float cost = PlanCost(aP, path, n, aStartYaw, aSpot, aSpotYaw, candK[i]) + candExtra[i];
            if (PathLen(path, n) > aP.maxPathM)
                cost += 100.0f; // only chosen when nothing shorter exists (Begin then reports TooFar)
            if (cost < bestCost)
            {
                bestCost = cost;
                bestN = n;
                for (int k = 0; k < n; ++k)
                    aOut[k] = path[k];
            }
        }
        return bestN;
    }

    // Detours a polyline through the given points (aVia[0] = start, aVia[aNVia-1] = end) around the obstacles with at most two extra
    // waypoints per segment. aSpot is the clip start spot (the obstacle test is exempt within Params::exemptM of it). Returns the point
    // count (0 = no path).
    static int PlanThrough(const Scene& aS, const Params& aP, const V2* aVia, int aNVia, V2 aSpot, V2* aOut)
    {
        const V2* pts = aVia;
        const int np = aNVia;

        // candidate detour points: obstacle vertices pushed away from the centroid
        V2 cand[2 * kMaxPoly];
        int nc = 0;
        const Poly* polys[2] = {&aS.box, aS.haveSector ? &aS.sector : nullptr};
        for (const Poly* g : polys)
        {
            if (!g)
                continue;
            const V2 c = g->Centroid();
            const float off = std::max(g->margin, 0.35f) + 0.15f;
            for (int i = 0; i < g->n; ++i)
            {
                V2 d = g->v[i] - c;
                const float l = Len(d);
                if (l < 1e-4f)
                    continue;
                const V2 w = g->v[i] + d * (off / l);
                if (nc < 2 * kMaxPoly && PolyDist(aS.box, w) >= aS.box.margin - 0.03f)
                    cand[nc++] = w;
            }
        }

        int n = 0;
        V2 out[kMaxPath];
        out[n++] = pts[0];
        for (int s = 1; s < np; ++s)
        {
            const V2 a = pts[s - 1];
            const V2 b = pts[s];
            if (SegmentOk(aS, aP, a, b, aSpot))
            {
                out[n++] = b;
                continue;
            }
            float best = 1e9f;
            V2 bw[2];
            int bn = 0;
            for (int i = 0; i < nc; ++i)
            {
                if (SegmentOk(aS, aP, a, cand[i], aSpot) && SegmentOk(aS, aP, cand[i], b, aSpot))
                {
                    const float l = Dist(a, cand[i]) + Dist(cand[i], b);
                    if (l < best)
                    {
                        best = l;
                        bw[0] = cand[i];
                        bn = 1;
                    }
                }
            }
            if (bn == 0)
            {
                for (int i = 0; i < nc; ++i)
                    for (int j = 0; j < nc; ++j)
                    {
                        if (i == j)
                            continue;
                        const float l = Dist(a, cand[i]) + Dist(cand[i], cand[j]) + Dist(cand[j], b);
                        if (l < best && SegmentOk(aS, aP, a, cand[i], aSpot) && SegmentOk(aS, aP, cand[i], cand[j], aSpot) &&
                            SegmentOk(aS, aP, cand[j], b, aSpot))
                        {
                            best = l;
                            bw[0] = cand[i];
                            bw[1] = cand[j];
                            bn = 2;
                        }
                    }
            }
            if (bn == 0 || n + bn + 1 > kMaxPath)
                return 0;
            for (int k = 0; k < bn; ++k)
                out[n++] = bw[k];
            out[n++] = b;
        }
        for (int i = 0; i < n; ++i)
            aOut[i] = out[i];
        return n;
    }

    // ---- control ----
    BeginResult Begin(const Scene& aS, const Params& aP, V2 aPos, float aYawDeg, V2 aSpot, float aSpotYawDeg)
    {
        m_active = false;
        if (!Finite(aPos) || !Finite(aSpot) || !std::isfinite(aYawDeg) || !std::isfinite(aSpotYawDeg))
            return BeginResult::Invalid;
        m_p = aP;
        m_scene = aS;
        m_spot = aSpot;
        m_spotYaw = WrapDeg(aSpotYawDeg);
        if (Dist(aPos, aSpot) <= aP.skipNearM && std::fabs(WrapDeg(m_spotYaw - aYawDeg)) <= aP.skipNearYawDeg)
            return BeginResult::AlreadyThere;
        // The walk keeps clear of the footprint box AND the closed door's sweep sector. When that plan does not exist or is too long the
        // walk is not started (NoPath / TooFar: the vanilla slide runs), unless Params::allowThroughClosedDoor (default off, tests only)
        // lets it fall back to a plan that only keeps the box clear.
        m_sectorSkipped = false;
        m_n = PlanBest(aS, aP, aPos, aYawDeg, aSpot, m_spotYaw, m_path);
        m_pathLen = m_n >= 2 ? PathLen(m_path, m_n) : 0.0f;
        if ((m_n < 2 || m_pathLen > aP.maxPathM) && aS.haveSector && aP.allowThroughClosedDoor)
        {
            Scene noSector = aS;
            noSector.haveSector = false;
            V2 alt[kMaxPath];
            const int altN = PlanBest(noSector, aP, aPos, aYawDeg, aSpot, m_spotYaw, alt);
            if (altN >= 2)
            {
                m_n = altN;
                for (int i = 0; i < altN; ++i)
                    m_path[i] = alt[i];
                m_pathLen = PathLen(m_path, m_n);
                m_sectorSkipped = true;
            }
        }
        if (m_n < 2)
            return BeginResult::NoPath;
        // r13d: undershoot. The facing carry scatters by +-0.04 m around the model; short of the spot the aligned rest is walked (cheap), past it there is no way back.
        if (aP.settleBackM > 0.0f && m_pathLen > 0.05f)
        {
            const V2 last = m_path[m_n - 1];
            const V2 prev = m_path[m_n - 2];
            const float legYaw = YawOf(last - prev);
            if (Len(last - prev) > 0.08f && (std::fabs(WrapDeg(m_spotYaw - legYaw)) > aP.arriveYawDeg || (aP.settleBackParallel && kMutation != 33)))
            {
                const V2 moved = last - Fwd(m_spotYaw) * aP.settleBackM;
                if (PolyDist(aS.box, moved) > 0.05f && Dist(prev, moved) > 0.05f)
                {
                    m_path[m_n - 1] = moved;
                    m_pathLen = PathLen(m_path, m_n);
                }
            }
        }
        if (m_pathLen > aP.maxPathM)
            return BeginResult::TooFar;
        m_idx = 1;
        m_target = m_path[m_n - 1];
        m_targetPlan = m_target;
        m_adaptShift = 0.0f;
        m_floorSum = 0.0f;
        m_floorN = 0;
        m_kSum = 0.0f;
        m_kN = 0;
        m_latched = false;
        m_legT = 0.0f;
        m_latchIdx = 0;
        m_latchYaw = 0.0f;
        m_elapsed = 0.0f;
        m_startPos = aPos;
        m_walkT = 0.0f;
        m_moved = false;
        m_minD = Dist(aPos, m_target);
        m_gain = aP.gain0;
        m_lastStick = 0.0f;
        m_havePrev = false;
        m_turnT = 0.0f;
        m_passed = false;
        m_lastD = Dist(aPos, aSpot);
        m_haveCmd = false;
        m_stickOut = std::min(aP.turnStick, aP.maxStick);
        m_prevStickOut = m_stickOut;
        m_steadyT = 0.0f;
        m_boost = 0.0f;
        m_stallT = 0.0f;
        m_posDone = false;
        m_released = false;
        m_releaseT = 0.0f;
        m_posT = -1.0f;
        m_posD = 0.0f;
        m_faceT = 0.0f;
        m_creepT = 0.0f;
        m_faceGaveUp = false;
        m_alignT = 0.0f;
        m_phase = 0;
        m_lastYawErr = std::fabs(WrapDeg(m_spotYaw - aYawDeg));
        m_lastRefYaw = 0.0f;
        m_lastBodyYaw = aYawDeg;
        m_velYaw = 0.0f;
        m_headErrSum = 0.0f;
        m_headErrN = 0;
        m_cruiseSum = 0.0f;
        m_cruiseN = 0;
        m_active = true;
        m_result = Result::Running;
        return BeginResult::Started;
    }

    bool Active() const { return m_active; }
    Result Last() const { return m_result; }
    float Elapsed() const { return m_elapsed; }
    float Gain() const { return m_gain; }
    bool Passed() const { return m_passed; }
    bool SectorSkipped() const { return m_sectorSkipped; }
    // mean angle (deg) between the commanded world direction and the measured velocity direction (n samples) and the mean measured speed at cruise
    float HeadingErrMeanDeg() const { return m_headErrN ? m_headErrSum / static_cast<float>(m_headErrN) : -1.0f; }
    int HeadingErrSamples() const { return m_headErrN; }
    float CruiseSpeedMean() const { return m_cruiseN ? m_cruiseSum / static_cast<float>(m_cruiseN) : -1.0f; }
    float Residual() const { return m_lastD; }
    // distance (m) from the settle point (the walk's end) to the spot: how far behind the spot the facing turn is expected to start
    float Settle() const { return Dist(m_target, m_spot); }
    // diagnostics: body yaw error vs the clip start yaw at the last step, time the position phase finished (-1 = not yet),
    // facing phase gave up (turnMaxS), phase 0 walk / 1 turn-first hold / 2 facing, last measured velocity yaw, last reference yaw
    float YawErr() const { return m_lastYawErr; }
    float PosDoneT() const { return m_posT; }
    float PosDoneResidual() const { return m_posD; }
    bool FaceGaveUp() const { return m_faceGaveUp; }
    int Phase() const { return m_phase; }
    float VelYaw() const { return m_velYaw; }
    float RefYaw() const { return m_lastRefYaw; }
    float BodyYaw() const { return m_lastBodyYaw; }
    float StickOut() const { return m_stickOut; }
    float Speed() const { return m_speed; }
    int PathCount() const { return m_n; }
    V2 PathPoint(int i) const { return m_path[i]; }

    void Cancel() { m_active = false; }

    // One control step. aRealMag / aRealAngleDeg: hardware stick (0 when none). Returns Running while the command is valid.
    Result Step(V2 aPos, float aYawDeg, float aRefYawDeg, float aDt, bool aYawControl, float aRealMag, float aRealAngleDeg,
                Command& aOut)
    {
        aOut = Command{};
        if (!m_active)
            return m_result;
        if (!Finite(aPos) || !std::isfinite(aYawDeg) || !std::isfinite(aRefYawDeg) || !std::isfinite(aDt) || aDt <= 0.0f ||
            aDt > 0.5f)
            return Finish(Result::Timeout, aPos);
        m_elapsed += aDt;
        // D: distance to the walk's end (the settle point behind the spot; the spot itself when the body yaw is not controllable, as no
        // facing turn follows), Ds: distance to the spot (residual, arrival gate)
        if (aYawControl && !m_posDone)
            CarryAdaptShift(aPos);
        const V2 tgt = aYawControl ? m_target : m_spot;
        const float D = Dist(aPos, tgt);
        const float Ds = Dist(aPos, m_spot);
        m_lastD = Ds;
        m_minD = std::min(m_minD, D);
        m_lastRefYaw = aRefYawDeg;
        m_lastBodyYaw = aYawDeg;
        if (Dist(aPos, m_startPos) >= m_p.noMotionMinM)
            m_moved = true;
        // measured speed and gain estimate
        const float dStick = std::fabs(m_stickOut - m_prevStickOut);
        m_steadyT = dStick < 0.004f ? m_steadyT + aDt : 0.0f;
        m_prevStickOut = m_stickOut;
        m_yawRate = m_havePrev ? std::fabs(WrapDeg(aYawDeg - m_prevYaw)) / aDt : 999.0f;
        // r13e: walk-loop speed at the hold stick and the body turn gain, measured on the way (CarryAdaptShift)
        if (m_havePrev && m_haveCmd && !m_posDone)
        {
            if (m_phase == 1 && m_alignT >= 0.25f && m_lastStick <= m_p.turnStick + 0.01f)
            {
                m_floorSum += m_speed;
                ++m_floorN;
            }
            const float ePrev = std::fabs(WrapDeg(m_cmdYaw - m_prevYaw));
            if (ePrev >= 20.0f && m_yawRate < 900.0f)
            {
                m_kSum += m_yawRate / ePrev;
                ++m_kN;
            }
        }
        m_prevYaw = aYawDeg;
        if (m_havePrev)
        {
            const float v = Dist(aPos, m_prevPos) / aDt;
            m_speed += 0.5f * (v - m_speed);
            if (v > 0.1f)
                m_velYaw = YawOf(aPos - m_prevPos);
            if (m_haveCmd && v > 0.3f)
            {
                m_headErrSum += std::fabs(WrapDeg(YawOf(aPos - m_prevPos) - m_cmdYaw));
                ++m_headErrN;
            }
            if (m_haveCmd && m_cmdWant >= m_p.cruiseMps - 1e-3f && v > 0.1f)
            {
                m_cruiseSum += v;
                ++m_cruiseN;
            }
            // only while the stick has been steady for a while: during the ramp the speed lags the stick and the ratio would read low
            if (m_lastStick >= 0.3f && m_speed > 0.15f && m_steadyT >= 0.35f)
                m_gain = std::clamp(m_gain + 0.15f * (m_speed / m_lastStick - m_gain), 1.2f, 6.0f);
        }
        else
            m_speed = 0.0f;
        m_prevPos = aPos;
        m_havePrev = true;

        const float yawErr = std::fabs(WrapDeg(m_spotYaw - aYawDeg));
        m_lastYawErr = yawErr;

        // waypoint
        const int idxBefore = m_idx;
        while (m_idx < m_n - 1 && Dist(aPos, m_path[m_idx]) <= m_p.waypointM)
            ++m_idx;
        if (m_idx != idxBefore)
        {
            m_alignT = 0.0f; // a new leg gets its own turn-first budget
            m_legT = 0.0f;
        }
        if (!m_posDone && m_yawRate < m_p.leadStuckDegPerS)
            m_legT += aDt; // only a body that does not turn loosens the lead limit
        const auto pathPt = [&](int i) { return i >= m_n - 1 ? tgt : m_path[i]; };
        const V2 wp = pathPt(m_idx);
        const V2 toWp = wp - aPos;
        const float toWpLen = Len(toWp);
        float pathYaw = toWpLen > 1e-4f ? YawOf(toWp) : m_spotYaw;
        // heading latch: close to the waypoint the bearing swings with every centimetre of lateral error; the stick keeps the direction
        // it had when V came within stableHeadM (no reversal of the heading while V passes beside the waypoint)
        // 2026-11-04 r11: not while the turn-first hold may run. A short leg (the walk's end closer than stableHeadM at the start) latched the bearing of
        // the START position; the hold then moves V ~0.2 m sideways (the game walks V along the stick at 0.8 m/s even at the hold stick) and the walk
        // ran on the latched line parallel to the leg, missing the walk's end by that much (fixture rides 5 / 10).
        const bool holdMayRun = kMutation != 4 && kMutation != 23 && aYawControl && !m_posDone &&
                                std::fabs(WrapDeg(pathYaw - aYawDeg)) > m_p.faceFirstDeg && m_alignT < m_p.alignMaxS;
        if (kMutation != 7)
        {
            if (toWpLen <= m_p.stableHeadM && !holdMayRun)
            {
                if (!m_latched || m_latchIdx != m_idx)
                {
                    m_latched = true;
                    m_latchIdx = m_idx;
                    m_latchYaw = pathYaw;
                }
                pathYaw = m_latchYaw;
            }
            else
                m_latched = false;
        }

        // Turn first: the body is still more than faceFirstDeg from the leg direction (start of the walk, after a sharp corner).
        // A full-size stick now would make V back-pedal at walking speed (the matcher then picks run-start clips). Low stick until
        // she has turned, bounded by alignMaxS per leg. Not with an uncontrollable body yaw.
        const float legErr = std::fabs(WrapDeg(pathYaw - aYawDeg));
        const bool holdNow = kMutation != 4 && aYawControl && !m_posDone && legErr > m_p.faceFirstDeg && m_alignT < m_p.alignMaxS;

        // Position phase done: at the spot, passed it, or a stall near it (the engine does not move V at a very small stick)
        if (!m_posDone)
        {
            bool done = false;
            if (D - (kMutation == 10 ? 0.2f : m_p.speedLeadS) * std::min(m_speed, 1.0f) <= (kMutation == 10 ? 0.12f : m_p.arriveM))
                done = true;
            else if (m_minD <= m_p.passedTriggerM &&
                     D > m_minD + (kMutation == 24 ? m_p.passedGrowM : std::min(m_p.passedGrowM, std::max(m_p.passedGrowMinM, m_p.passedGrowMissFrac * m_minD))) &&
                     D <= m_p.passedTriggerM + 0.2f)
            {
                m_passed = true;
                done = true;
            }
            if (kMutation != 3)
            {
                if (m_elapsed >= 0.5f && D <= m_p.stallRadiusM && m_speed < m_p.stallSpeedMps && !holdNow)
                    m_stallT += aDt;
                else
                    m_stallT = 0.0f;
                if (m_stallT >= m_p.stallS)
                    done = true;
            }
            if (done)
            {
                m_posDone = true;
                if (m_posT < 0.0f)
                {
                    m_posT = m_elapsed;
                    m_posD = D;
                }
                if (kMutation == 6)
                    m_faceT = 0.0f; // old: the facing budget restarted at every entry
            }
        }
        else if ((kMutation == 21 ? std::max(D, Ds) : Ds) > m_p.faceReleaseM)
        {
            m_posDone = false; // drifted away while turning: walk back first
            m_stallT = 0.0f;
            // the walk back is a new leg: the stick lead limit starts again at maxLeadDeg (a lead grown on the first walk would let the stick
            // lead V's facing by 75-90 deg, the matcher then plays reface clips) and the walk back has its own time budget
            if (kMutation != 13)
                m_legT = 0.0f;
            m_released = true;
            m_releaseT = 0.0f;
            if (kMutation != 6)
            {
                // the closest-approach / passed bookkeeping belongs to the walk that ended: a stale m_minD would make the walk back
                // 'pass' the spot again at passedTriggerM + 0.2 m and V would never get back to it
                m_minD = D;
                m_passed = false;
            }
        }
        if (m_posDone)
        {
            const bool yawOk = kMutation == 1 || !aYawControl || yawErr <= m_p.arriveYawDeg;
            // arrival needs the residual to the spot as well: the facing stick walks V along the clip heading (rides 2 / behind of
            // 2026-11-04 mounted 0.41 / 0.40 m from the spot)
            const float alongNow = Dot(m_spot - aPos, Fwd(m_spotYaw));
            if (yawOk && Ds > m_p.arriveTightM && Ds <= m_p.arriveGateM && alongNow > 0.0f)
                m_creepT += aDt;
            const bool resOk = kMutation == 8 || !aYawControl || Ds <= m_p.arriveTightM || (Ds <= m_p.arriveGateM && (alongNow <= 0.0f || m_creepT >= m_p.creepS));
            if (yawOk && resOk)
                return Finish(Result::Arrived, aPos);
            if (yawOk)
            {
                // aligned, but not at the spot: the facing stick (along the clip heading) closes a gap in front of V; past the spot (or
                // with the budget spent) the engine slide is short enough to mount
                const float along = Dot(m_spot - aPos, Fwd(m_spotYaw));
                if (along <= 0.0f || m_faceT >= m_p.turnMaxS)
                    return Finish(Result::NearSpot, aPos);
            }
            else if (m_faceT >= m_p.turnMaxS)
            {
                m_faceGaveUp = true;
                return Finish(Ds <= m_p.arriveGateM ? Result::Arrived : Result::NearSpot, aPos);
            }
        }
        else if (kMutation != 6 && m_faceT >= m_p.turnMaxS && Ds <= m_p.nearMountM)
        {
            // the TOTAL facing time (all entries) is spent and V is walking back within mount range: mount, no facing loop until the timeout
            m_faceGaveUp = true;
            return Finish(Ds <= m_p.arriveGateM ? Result::Arrived : Result::NearSpot, aPos);
        }
        if (m_released && !m_posDone)
        {
            m_releaseT += aDt;
            // the walk back after a release is bounded on its own: a body that does not follow the stick would circle until the timeout
            if (kMutation != 14 && m_releaseT >= m_p.releaseMaxS)
            {
                m_faceGaveUp = true;
                return Finish(Ds <= m_p.nearMountM ? Result::NearSpot : Result::Timeout, aPos);
            }
        }
        if (m_elapsed >= m_p.timeoutS)
            return Finish(Ds <= m_p.nearMountM ? Result::MountNear : Result::Timeout, aPos);
        // The no-motion clock runs on walking time only (phase 0): the turn-first hold and the facing phase use a very small stick, and a body
        // that turns without the capsule following is exactly the failure this guard exists for (the body turn follows the script stick).
        if (m_walkT >= m_p.noMotionAfterS && !m_moved && m_pathLen >= m_p.noMotionPathM)
            return Finish(Result::NoMotion, aPos);

        if (m_elapsed >= m_p.graceS && aRealMag > m_p.abortStick)
        {
            const float stickWorldYaw = aRefYawDeg + aRealAngleDeg;
            float off = std::fabs(WrapDeg(stickWorldYaw - pathYaw));
            if (m_posDone && kMutation != 16)
            {
                // facing phase: pathYaw points at the settle point, which lies BEHIND V once the facing carry has walked her past it. The player
                // pushing toward the car (the spot, or along the clip heading) is not against the path
                off = std::fabs(WrapDeg(stickWorldYaw - m_spotYaw));
                if (Ds > m_p.abortSpotBearingM)
                    off = std::min(off, std::fabs(WrapDeg(stickWorldYaw - YawOf(m_spot - aPos))));
            }
            else if (kMutation != 27 && kMutation != 16 && Ds <= m_p.exemptM)
            {
                // r13c: the settle point lies to the side of the clip line, so the walk of a start beside the car points 90-100 deg away from the clip heading. A player who
                // pushes the stick toward the car (the clip heading) within the last 0.7 m is walking the way she is being led, not against the path
                off = std::min(off, std::fabs(WrapDeg(stickWorldYaw - m_spotYaw)));
            }
            if (off > m_p.abortAngleDeg)
                return Finish(Result::InputAbort, aPos);
        }

        float wantYaw = pathYaw;
        float target;
        float want;
        if (m_posDone)
        {
            // facing phase: a small stick toward the clip's start yaw turns V in place (bounded)
            m_faceT += aDt;
            m_phase = 2;
            wantYaw = m_spotYaw;
            // The game snaps V's velocity to the facing stick, so what the walk left across the clip heading stays until the end: aim the
            // stick a little toward the spot (clamped, fading out over the last steerFadeM so the body ends within the facing tolerance)
            if (kMutation != 12 && aYawControl)
            {
                const V2 d = m_spot - aPos;
                const V2 f = Fwd(m_spotYaw);
                const float al = Dot(d, f);
                const float fade = std::clamp(al / m_p.steerFadeM, 0.0f, 1.0f);
                // r13c: the lean also fades out over the last steerBudgetFrac of the facing budget: a V that does not translate (blocked, stalled stick) keeps a lateral offset
                // and the lean would hold the body off the clip yaw until the budget ends
                const float budgetK = kMutation == 26 ? 1.0f : std::clamp((m_p.turnMaxS - m_faceT) / (m_p.steerBudgetFrac * m_p.turnMaxS), 0.0f, 1.0f);
                wantYaw = m_spotYaw + fade * budgetK * std::clamp(Deg(std::atan2(Cross(f, d), std::max(al, m_p.steerLookM))), -m_p.steerMaxDeg, m_p.steerMaxDeg);
            }
            if (kMutation != 20 && aYawControl)
            {
                // the body turn slows down in proportion to the remaining error: aim past the clip yaw while the error is large so the last degrees go faster
                const float es = WrapDeg(m_spotYaw - aYawDeg);
                if (std::fabs(es) > m_p.arriveYawDeg)
                    wantYaw += (es > 0.0f ? 1.0f : -1.0f) * m_p.faceLeadDeg;
                // 2026-11-04 r11: near a half turn the aim past the clip yaw (and the steering) can put the target on the other side of the body: the
                // shortest way to it then flips with the sign of the error and every frame turns the body the opposite way (fixture rides 30 / 32: the
                // body stood at 117 deg for the whole facing phase and V walked 0.5 m away). The turn keeps the direction of the error to the clip yaw.
                if (kMutation != 22 && std::fabs(es) > m_p.arriveYawDeg && WrapDeg(wantYaw - aYawDeg) * es < 0.0f)
                    wantYaw = m_spotYaw;
            }
            target = m_p.turnStick;
            want = m_p.turnStick * m_gain;
            // aligned but still short of the spot (the facing stick did not carry V far enough, e.g. a stalling low stick): walk the rest
            // along the clip heading at the final speed instead of waiting for the budget
            if (kMutation != 8 && aYawControl && yawErr <= m_p.arriveYawDeg && Ds > (m_p.creepWalks ? m_p.arriveTightM : m_p.arriveGateM) && Dot(m_spot - aPos, Fwd(m_spotYaw)) > 0.0f)
            {
                want = m_p.finalMps;
                if (kMutation != 3 && m_speed < m_p.boostSpeedFrac * want)
                    m_boost = std::min(m_p.boostMax, m_boost + m_p.boostRate * aDt); // a low stick that does not move V: lift it
                else
                    m_boost = std::max(0.0f, m_boost - m_p.boostRate * aDt);
                target = std::clamp(want / m_gain + m_boost, m_p.minStick, m_p.maxStick);
            }
            else if (kMutation != 15 && kMutation != 8 && aYawControl && Ds > m_p.arriveGateM && m_faceT > m_p.faceBoostAfterS &&
                     Dot(m_spot - aPos, Fwd(m_spotYaw)) > 0.0f && m_speed < m_p.boostSpeedFrac * m_p.finalMps)
            {
                // still turning, short of the spot and (nearly) standing still: the facing stick is below what moves V (stall). Lift it, the turn
                // then carries V along the clip heading as planned (without it the walk the rest starts only after the facing)
                m_boost = std::min(m_p.boostMax, m_boost + m_p.boostRate * aDt);
                target = std::clamp(m_p.turnStick + m_boost, m_p.turnStick, m_p.maxStick);
                want = m_p.finalMps;
            }
            else
                m_boost = std::max(0.0f, m_boost - m_p.boostRate * aDt);
        }
        else if (holdNow)
        {
            m_alignT += aDt;
            m_phase = 1;
            target = m_p.turnStick;
            want = m_p.turnStick * m_gain;
        }
        else
        {
            m_phase = 0;
            float rem = D;
            if (m_idx < m_n - 1)
            {
                rem = toWpLen;
                for (int i = m_idx; i < m_n - 1; ++i)
                    rem += Dist(pathPt(i), pathPt(i + 1));
            }
            want = rem >= m_p.finalSlowM ? m_p.cruiseMps
                                         : std::clamp(m_p.finalMps * rem / m_p.finalSlowM, m_p.minSpeedMps, m_p.cruiseMps);
            // sharp corner ahead: slow down toward it so the body can turn on the way (the turn-first hold then takes over)
            if (m_idx < m_n - 1 && toWpLen < m_p.cornerSlowM)
            {
                const float turn = std::fabs(WrapDeg(YawOf(pathPt(m_idx + 1) - pathPt(m_idx)) - pathYaw));
                const float f = std::clamp(turn / 90.0f, 0.0f, 1.0f);
                const float k = 1.0f - std::clamp((toWpLen - m_p.waypointM) / (m_p.cornerSlowM - m_p.waypointM), 0.0f, 1.0f);
                want = std::min(want, m_p.cruiseMps + (m_p.cornerSlowMps - m_p.cruiseMps) * f * k);
            }
            // integral boost against a low-stick stall: the speed stays far below the wanted one near the spot
            if (kMutation != 3 && m_elapsed > 0.4f && D <= m_p.boostRangeM && m_speed < m_p.boostSpeedFrac * want)
                m_boost = std::min(m_p.boostMax, m_boost + m_p.boostRate * aDt);
            else
                m_boost = std::max(0.0f, m_boost - m_p.boostRate * aDt);
            target = std::clamp(want / m_gain + m_boost, m_p.minStick, m_p.maxStick);
        }
        // The body follows the stick: while walking / holding the stick leads the body yaw by at most maxLeadDeg, so V arcs round a turn and
        // the matcher never sees a stick pointing against her facing (reface / backpedal clips). The facing phase turns in place on purpose.
        if (m_phase != 2 && aYawControl && kMutation != 7)
        {
            const float lead = m_p.maxLeadDeg + m_p.leadGrowDegPerS * m_legT;
            wantYaw = aYawDeg + std::clamp(WrapDeg(pathYaw - aYawDeg), -lead, lead);
        }
        // bounded stick rate: no step into the Run gait range, no sudden drop below the walk range
        if (kMutation == 5)
            m_stickOut = target;
        else
            m_stickOut = std::clamp(target, m_stickOut - m_p.stickFallRate * aDt, m_stickOut + m_p.stickRiseRate * aDt);
        if (m_phase == 0)
            m_walkT += aDt;
        const float stick = m_stickOut;
        m_lastStick = stick;
        m_cmdYaw = wantYaw;
        m_cmdWant = want;
        m_haveCmd = true;
        const float a = WrapDeg(wantYaw - aRefYawDeg);
        aOut.stickX = -std::sin(Rad(a)) * stick;
        aOut.stickY = std::cos(Rad(a)) * stick;
        aOut.headingDeg = a;
        aOut.speedWantMps = want;
        return Result::Running;
    }

    std::string Describe() const
    {
        char buf[512];
        int o = 0;
        o += std::snprintf(buf + o, sizeof(buf) - o, "path=");
        for (int i = 0; i < m_n && o < 440; ++i)
            o += std::snprintf(buf + o, sizeof(buf) - o, "%s(%.2f,%.2f)", i ? ">" : "", static_cast<double>(m_path[i].x),
                               static_cast<double>(m_path[i].y));
        std::snprintf(buf + o, sizeof(buf) - o, " len=%.2f spotYaw=%.1f yawErr=%.1f posT=%.2f settle=%.2f%s%s", static_cast<double>(m_pathLen),
                      static_cast<double>(m_spotYaw), static_cast<double>(m_lastYawErr), static_cast<double>(m_posT),
                      static_cast<double>(Settle()), m_faceGaveUp ? " faceGaveUp=1" : "", m_sectorSkipped ? " doorSector=ignored" : "");
        return buf;
    }

private:
    // r13e: online carry adaptation. The settle point is placed by the carry model of the ride plant (walk-loop speed ~0.82 m/s at the facing stick, body turn
    // gain ~4.2 deg/s per deg of stick-to-body error). A V that walks faster at the hold stick or turns slower than that is carried further: the walk's end on the
    // last leg moves back along the clip heading by carryAdaptGain x the planned carry x the relative excess (at most carryAdaptMaxM; never forward), frozen
    // within stableHeadM of it (no target jump while V arrives).
    void CarryAdaptShift(V2 aPos)
    {
        // only before the first facing phase: a walk back after a facing release keeps its end (a drifting body would push it further every time)
        if (kMutation == 32 || m_p.carryAdaptGain <= 0.0f || m_posT >= 0.0f || m_idx < m_n - 1 || (m_floorN < 6 && m_kN < 6))
            return;
        if (Dist(aPos, m_target) <= m_p.stableHeadM)
            return;
        const V2 f = Fwd(m_spotYaw);
        const float carry = Dot(m_spot - m_targetPlan, f);
        if (carry <= 0.05f)
            return;
        const float rv = m_floorN >= 6 ? std::clamp((m_floorSum / static_cast<float>(m_floorN)) / m_p.carryAdaptRefMps, 0.7f, 1.4f) : 1.0f;
        // a turn gain far outside the fitted law (a body that turns at a fixed slow rate, or not after the stick at all) says nothing about the carry: no shift from it
        const float kHat = m_kN >= 6 ? m_kSum / static_cast<float>(m_kN) : 0.0f;
        const bool kFits = m_p.carryAdaptRefK > 0.0f && kHat >= 0.6f * m_p.carryAdaptRefK && kHat <= 1.5f * m_p.carryAdaptRefK;
        const float rk = kFits ? m_p.carryAdaptRefK / kHat : 1.0f;
        // the walk's end stays inside the facing-release radius (a facing phase that starts farther than faceReleaseM from the spot walks back at once)
        const float room = std::max(0.0f, m_p.faceReleaseM - 0.05f - Dist(m_spot, m_targetPlan));
        const float shift = std::min(std::clamp(m_p.carryAdaptGain * carry * (rv * rk - 1.0f), 0.0f, m_p.carryAdaptMaxM), room);
        if (shift > m_adaptShift)
        {
            m_adaptShift = shift;
            m_target = m_targetPlan - f * shift;
            m_minD = Dist(aPos, m_target); // the closest-approach bookkeeping belongs to the new end
        }
    }

    Result Finish(Result r, V2 aPos)
    {
        m_lastD = Dist(aPos, m_spot);
        m_active = false;
        m_result = r;
        return r;
    }

    Params m_p;
    Scene m_scene;
    V2 m_spot;
    V2 m_target;          // the walk's end: the settle point behind the spot (or the spot)
    float m_yawRate = 999.0f; // measured body yaw rate (deg/s)
    float m_prevYaw = 0.0f;
    float m_legT = 0.0f;  // time on the current leg (grows the allowed stick lead)
    bool m_latched = false;
    int m_latchIdx = 0;
    float m_latchYaw = 0.0f;
    float m_spotYaw = 0.0f;
    V2 m_path[kMaxPath];
    int m_n = 0;
    float m_pathLen = 0.0f;
    bool m_sectorSkipped = false;
    int m_idx = 1;
    bool m_active = false;
    Result m_result = Result::Running;
    float m_elapsed = 0.0f;
    V2 m_startPos;
    bool m_moved = false;
    float m_minD = 0.0f;
    float m_lastD = 0.0f;
    float m_gain = 2.3f;
    float m_speed = 0.0f;
    float m_lastStick = 0.0f;
    V2 m_prevPos;
    bool m_havePrev = false;
    float m_turnT = 0.0f;
    bool m_passed = false;
    // diagnostics for the game test: does V follow the commanded direction, and how fast does she walk at cruise
    float m_walkT = 0.0f;
    float m_stickOut = 0.0f;
    float m_prevStickOut = 0.0f;
    float m_steadyT = 0.0f;
    float m_boost = 0.0f;
    float m_stallT = 0.0f;
    bool m_posDone = false;
    bool m_released = false;  // a facing release happened (the walk back is bounded by releaseMaxS)
    float m_releaseT = 0.0f;
    float m_posT = -1.0f;
    float m_posD = 0.0f;
    float m_faceT = 0.0f;
    V2 m_targetPlan;          // r13e: the planned walk's end (before CarryAdaptShift)
    float m_adaptShift = 0.0f;
    float m_floorSum = 0.0f;  // hold-stick speed samples
    int m_floorN = 0;
    float m_kSum = 0.0f;      // body turn gain samples
    int m_kN = 0;
    float m_creepT = 0.0f;   // time aligned and creeping toward the spot between arriveTightM and arriveGateM
    bool m_faceGaveUp = false;
    float m_alignT = 0.0f;
    int m_phase = 0;
    float m_lastYawErr = 0.0f;
    float m_lastRefYaw = 0.0f;
    float m_lastBodyYaw = 0.0f;
    float m_velYaw = 0.0f;
    float m_cmdYaw = 0.0f;
    float m_cmdWant = 0.0f;
    bool m_haveCmd = false;
    float m_headErrSum = 0.0f;
    int m_headErrN = 0;
    float m_cruiseSum = 0.0f;
    int m_cruiseN = 0;
};
} // namespace utpp::approach
