// UTPP motion matching runtime core (Phase 4a, offline). Header-only, C++20, deterministic, no allocations in
// Update() after Init().
//
// Pipeline per Update(dt, input) (structure after orangeduck controller.cpp, Motion-Matching, MIT License,
// Copyright (c) 2021 Daniel Holden):
//   1. trajectory simulation + prediction (BaseTrajectory.hpp) from the desired velocity / facing;
//   2. inertialization offsets decay by dt;
//   3. playback time advances at the database rate (30 fps) with variable dt; loops wrap (frame N-1 == frame 0, the
//      next played frame after N-1 is 1, matching the ROOT section); non-loops hold their last frame;
//   4. search when the timer expires (default 0.1 s), when the desired velocity / facing changed beyond a threshold
//      since the last search (rate-limited), or near the end of a non-loop clip (then without the current frame);
//      query = the CURRENT playback frame's stored features (denormalized) with the trajectory part replaced by the
//      prediction, normalized with the database offset/scale (orangeduck query_copy_denormalized_feature);
//      the search only visits ranges of allowed clips (tag allow-mask, optional frame-range allow-list);
//   5. on a transition the inertializer records src(+offset) - dst so the output is continuous; the output pose is
//      the sampled pose (linear / nlerp interpolation between the two neighbouring 30 fps frames) plus the offsets.
//
// Output: local transforms of all bones (game-local, relative to the game parent). Bones < firstBodyBone (Root,
// Trajectory) are identity; root motion is NOT applied to the pose - it is reported in Diagnostics (consumed
// animation root delta and simulated character step) because the game moves the capsule itself.
#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

#include "../BaseDatabase.hpp"
#include "Inertializer.hpp"
#include "MMMath.hpp"
#include "BaseTrajectory.hpp"

namespace utpp::mm
{
struct MatcherConfig
{
    float searchInterval = 0.1f;                // s, periodic search
    float forceSearchVelocityThreshold = 0.5f;  // m/s change of the desired velocity since the last search
    float forceSearchFacingThreshold = 0.35f;   // rad change of the desired facing since the last search (Direction mode)
    float forceSearchYawRateThreshold = 0.5f;   // rad/s change of the desired yaw rate since the last search (YawRate mode)
    float forceSearchCooldown = 0.1f;           // s, minimum time between forced searches
    float transitionCost = 0.0f;                // added to every candidate except the current frame
    int32_t ignoreSurroundingFrames = 10;       // frames around the current one that are skipped (0.33 s at 30 fps)
    int32_t endOfClipMarginFrames = 1;          // non-loops: forced search (without current frame) when <= this many frames remain
    uint64_t allowedTagMask = ~0ull;            // clip allowed if (clip.tagMask & mask) != 0; ~0 = every clip
    std::vector<Range> allowedFrameRanges;      // optional extra allow-list of global frame ranges [start, stop); empty = all
    int32_t initialFrame = -1;                  // -1: first searchable frame of the first allowed clip with a tag containing "Idle"
    float inertializationHalflife = 0.1f;       // s; <= 0 disables inertialization (hard cuts, test baseline)
    float velocityHalflife = 0.27f;             // s, trajectory velocity spring
    float rotationHalflife = 0.27f;             // s, trajectory yaw spring
    float stopHalflife = 0.0f;                  // s, trajectory velocity spring while the goal is zero (0 = off; 0.3.4)
    // Phase 5.3c (0.3.5) transition stability: for minDwell s (matcher time) after a transition a search may only move to
    // another frame if its cost < dwellCostRatio * cost of the continuing frame. Exempt: first / end-of-clip searches,
    // tag-set changes, MatcherInput::dwellExempt, and sharp input changes since the last search (goal velocity change >
    // dwellExemptVelocity m/s or goal facing change > dwellExemptFacing rad). minDwell 0 = off (0.3.4 bit for bit).
    float minDwell = 0.0f;
    float dwellCostRatio = 0.5f;
    float dwellExemptVelocity = 1.0f;
    float dwellExemptFacing = 1.0f;
    // 0.5.8 cost cap (UTPPSetMMCostCap): a search whose best candidate costs more than costCap keeps the continuing frame
    // (at most costCapHoldMax s matcher time in a row); without a continuing frame (end of clip, playing clip not allowed /
    // excluded) or after the hold it searches again with MatcherInput::capRetryExclude and takes the cheaper result; a
    // candidate of MatcherInput::capNeverClasses above the cap is never taken when the retry found anything. 0 = off (0.5.7).
    float costCap = 0.0f;
    float costCapHoldMax = 0.3f;
    uint32_t firstBodyBone = 2;                 // Hips; bones below it are output as identity
    float maxDt = 0.25f;                        // s, larger steps are clamped
};

struct MatcherInput
{
    float desiredVelocity[2] = {0.0f, 0.0f}; // m/s, character frame (x right, y forward)
    FacingMode facingMode = FacingMode::Direction;
    float desiredFacing[2] = {0.0f, 1.0f}; // unit xy, character frame
    float yawRate = 0.0f;                  // rad/s CCW, YawRate mode
    int32_t gait = 0;                      // placeholder (Phase 4b+), recorded only
    int32_t stance = 0;                    // placeholder (Phase 4b+), recorded only
    // Phase 5.1: character yaw driven externally (see TrajectoryInput::externalYaw). Rate in matcher time (rad/s).
    bool externalYaw = false;
    float externalYawStep = 0.0f;
    float externalYawRate = 0.0f;
    // Phase 5.3c (0.3.5): trajectory curvature (TrajectoryInput::turnRate, Direction mode, rad/s matcher time); clip
    // classes excluded from the candidates of this update's search (bit i = class i of SetClipClasses; the continuing
    // frame stays a candidate unless its class is also in excludeCurrentClasses); dwell exemption (latched until the
    // next search).
    float turnRate = 0.0f;
    uint32_t excludeClasses = 0;
    uint32_t excludeCurrentClasses = 0;
    bool dwellExempt = false;
    bool clearAngVel = false; // 0.3.8, see TrajectoryInput::clearAngVel
    // 0.5.8 cost cap (MatcherConfig::costCap): class mask of the retry search ("ordinary" candidate set) and the classes
    // that are never selected above the cap (stance transitions)
    bool capRetry = false;
    uint32_t capRetryExclude = 0;
    uint32_t capNeverClasses = 0;
    // AIR HOLD stage (2026-10-13): no search this update (input-change, timer and end-of-clip searches all skipped; a non-loop
    // end holds its last frame). false = bit-identical to before.
    bool suppressSearch = false;
    // AIR CLIPS stage: >= 0 = the next search (latched until it runs) takes its bone features from this global frame instead
    // of the playing one (a clean ground frame after air clips); trajectory features are unchanged. -1 = bit-identical.
    int32_t queryBoneFrame = -1;
};

struct Pose
{
    std::vector<BoneTransform> bones; // NumBones() entries, game-local
};

struct MatcherDiagnostics
{
    int32_t frame = -1;     // global database frame being played
    int32_t clip = -1;      // clip index
    int32_t clipFrame = -1; // frame inside the clip
    float alpha = 0.0f;     // interpolation weight towards the next played frame
    uint64_t updates = 0;
    uint64_t searches = 0;
    uint64_t forcedSearches = 0;   // input-change searches
    uint64_t endOfClipSearches = 0;
    uint64_t transitions = 0;
    uint64_t nonFiniteResets = 0;
    int32_t lastSearchIndex = -1;
    float lastSearchCost = 0.0f;   // best cost of the last search (includes transitionCost unless it kept the current frame)
    double lastSearchMicros = 0.0; // wall time of the last search (diagnostic only, never used for decisions)
    bool searchedThisUpdate = false;
    bool transitionedThisUpdate = false;
    float consumedRoot[3] = {0.0f, 0.0f, 0.0f}; // animation root motion of whole frames stepped this update: dx, dy (m), dyaw (rad)
    float simStep[3] = {0.0f, 0.0f, 0.0f};      // simulated character step this update (character frame at update start)
    float simVelocity[2] = {0.0f, 0.0f};        // simulated velocity, character frame
    float trajectoryPosition[2 * Trajectory::kMaxHorizons] = {};
    float trajectoryDirection[2 * Trajectory::kMaxHorizons] = {};
    float maxOffsetAngle = 0.0f; // rad, largest inertialization rotation offset after this update
    float maxOffsetDist = 0.0f;  // m
    uint64_t dwellHolds = 0;       // 5.3c: searches whose better candidate was rejected by the dwell rule
    uint64_t dwellExempts = 0;     // 5.3c: searches inside the dwell window that were exempt (sharp change)
    uint64_t filteredSearches = 0; // 5.3c: searches with a clip-class filter applied
    float sinceTransition = 0.0f;  // s matcher time since the last transition
    // 0.5.8 cost cap
    uint64_t costCapHits = 0;      // searches whose best candidate cost more than the cap
    uint64_t costCapHolds = 0;     // ... answered by keeping the continuing frame
    uint64_t costCapRetries = 0;   // ... answered by the retry search
    uint64_t costCapForced = 0;    // selections above the cap (nothing cheaper found)
    uint64_t costCapNever = 0;     // capNeverClasses candidates above the cap replaced by the retry result
    int32_t costCapLastIndex = -1; // rejected / retried best frame of the last cap hit
    float costCapLastCost = 0.0f;  // its cost
    bool costCapHeldThisUpdate = false;
    int32_t gait = 0;
    int32_t stance = 0;
};

class BaseMatcher
{
public:
    // OR of the masks of every tag whose name contains aSubstring.
    static uint64_t TagMaskContaining(const Database& aDb, std::string_view aSubstring)
    {
        uint64_t m = 0;
        for (uint32_t t = 0; t < aDb.NumTags() && t < 64; ++t)
            if (aDb.TagName(t).find(aSubstring) != std::string_view::npos)
                m |= 1ull << t;
        return m;
    }

    // aDb must outlive this object and stay unchanged. Returns false (object unusable) on invalid input.
    bool Init(const Database& aDb, const MatcherConfig& aCfg)
    {
        m_ok = false;
        m_db = nullptr;
        if (!aDb.Loaded() || !ValidConfig(aCfg) || aDb.NumBones() <= aCfg.firstBodyBone || !(aDb.Fps() > 0.0f))
            return false;
        // trajectory groups
        const GroupRecord* gp = nullptr;
        const GroupRecord* gd = nullptr;
        for (const GroupRecord& g : aDb.Groups())
        {
            if (g.kind == uint32_t(GroupKind::TrajectoryPosition) && !gp)
                gp = &g;
            else if (g.kind == uint32_t(GroupKind::TrajectoryDirection) && !gd)
                gd = &g;
        }
        if (!gp || !gd || gp->numHorizons != gd->numHorizons || gp->numHorizons > Trajectory::kMaxHorizons)
            return false;
        for (uint32_t k = 0; k < gp->numHorizons; ++k)
            if (gp->horizonFrames[k] != gd->horizonFrames[k])
                return false;
        m_cfg = aCfg;
        m_fps = aDb.Fps();
        m_frameDt = 1.0f / m_fps;
        m_trajPosFirst = gp->firstFeature;
        m_trajDirFirst = gd->firstFeature;
        m_numHorizons = gp->numHorizons;
        if (!m_traj.Init(gp->horizonFrames, gp->numHorizons, m_fps, aCfg.velocityHalflife, aCfg.rotationHalflife) ||
            !m_traj.SetStopHalflife(aCfg.stopHalflife))
            return false;

        // allowed clips / ranges
        const auto clips = aDb.Clips();
        m_clipAllowed.assign(clips.size(), 0);
        std::vector<Range> extra = aCfg.allowedFrameRanges;
        std::sort(extra.begin(), extra.end(), [](const Range& a, const Range& b) { return a.start < b.start; });
        for (size_t c = 0; c < clips.size(); ++c)
        {
            const bool tagOk = aCfg.allowedTagMask == ~0ull || (clips[c].tagMask & aCfg.allowedTagMask) != 0;
            bool rangeOk = extra.empty();
            for (const Range& r : extra)
                rangeOk = rangeOk || (r.start < clips[c].firstFrame + clips[c].numFrames && r.stop > clips[c].firstFrame);
            m_clipAllowed[c] = (tagOk && rangeOk) ? 1 : 0;
        }
        m_ranges.clear();
        m_ranges.reserve(aDb.SearchRanges().size() * (extra.size() + 1));
        m_searchRanges.clear();
        m_searchRanges.reserve(m_ranges.capacity());
        if (m_clipClass.size() != clips.size())
            m_clipClass.assign(clips.size(), 0);
        for (const Range& r : aDb.SearchRanges())
        {
            const int32_t c = aDb.ClipOfFrame(r.start);
            if (c < 0 || !m_clipAllowed[size_t(c)])
                continue;
            if (extra.empty())
            {
                m_ranges.push_back(r);
                continue;
            }
            for (const Range& e : extra)
            {
                const uint32_t s = std::max(r.start, e.start), t = std::min(r.stop, e.stop);
                if (s < t)
                    m_ranges.push_back({s, t});
            }
        }
        if (m_ranges.empty())
            return false;

        // initial frame
        int32_t init = aCfg.initialFrame;
        if (init >= 0)
        {
            if (static_cast<uint32_t>(init) >= aDb.NumFrames())
                return false;
        }
        else
        {
            for (const Range& r : m_ranges)
            {
                const uint64_t mask = clips[size_t(aDb.ClipOfFrame(r.start))].tagMask;
                bool idle = false;
                for (uint32_t t = 0; t < aDb.NumTags() && t < 64; ++t)
                    idle = idle || (((mask >> t) & 1ull) && aDb.TagName(t).find("Idle") != std::string_view::npos);
                if (idle)
                {
                    init = static_cast<int32_t>(r.start);
                    break;
                }
            }
            if (init < 0)
                init = static_cast<int32_t>(m_ranges.front().start);
        }

        // buffers (the only allocations)
        const uint32_t nb = aDb.NumBones();
        const uint32_t nf = aDb.NumFeatures();
        m_query.assign(nf, 0.0f);
        m_queryNorm.assign(nf, 0.0f);
        m_sampleA.assign(nb, BoneTransform{});
        m_sampleB.assign(nb, BoneTransform{});
        m_lvA.assign(nb, math::Vec3{});
        m_avA.assign(nb, math::Vec3{});
        m_lvB.assign(nb, math::Vec3{});
        m_avB.assign(nb, math::Vec3{});
        m_out.bones.assign(nb, BoneTransform{});
        if (!m_inert.Init(nb, aCfg.firstBodyBone))
            return false;

        m_db = &aDb;
        m_frame = init;
        m_clip = aDb.ClipOfFrame(static_cast<uint32_t>(init));
        m_acc = 0.0f;
        m_searchTimer = 0.0f;
        m_forceCooldown = 0.0f;
        m_first = true;
        m_lastGoalVx = m_lastGoalVy = m_lastGoalYaw = m_lastYawRate = 0.0f;
        m_sinceTransition = 1e6f;
        m_dwellExemptPending = false;
        // b5 testorder repair: the 0.5.8 cost-cap hold is NOT reset here (0.7.0: it carries over SetEnabled / transition cost /
        // tag filter re-inits); the driver clears it with ResetCapHold() on a session reset (LoadDatabase) and on a database swap
        // (UTPPSetMMSwapReset on)
        m_diag = MatcherDiagnostics{};
        m_rootZ.clear();        // AIR CLIPS stage: per-database data, re-applied by the caller after Init
        m_loopOverride.clear();
        m_transHalflife = -1.0f;
        m_queryBoneFrame = -1;
        m_ok = true;
        Sample(m_frame, 0.0f, m_sampleA.data(), nullptr, nullptr);
        m_inert.ResetHistory();
        m_inert.Apply(m_sampleA.data(), m_out.bones.data());
        FillDiagnostics();
        return true;
    }

    bool Ready() const { return m_ok; }

    void Update(float aDt, const MatcherInput& aIn)
    {
        if (!m_ok)
            return;
        float dt = (std::isfinite(aDt) && aDt > 0.0f) ? aDt : 0.0f;
        dt = std::min(dt, m_cfg.maxDt);
        m_diag.searchedThisUpdate = false;
        m_diag.transitionedThisUpdate = false;
        m_diag.costCapHeldThisUpdate = false;
        if (m_capHolding)
            m_capHoldTime += dt;
        m_diag.gait = aIn.gait;
        m_diag.stance = aIn.stance;
        ++m_diag.updates;

        // 1. trajectory
        TrajectoryInput ti;
        ti.desiredVelocity[0] = aIn.desiredVelocity[0];
        ti.desiredVelocity[1] = aIn.desiredVelocity[1];
        ti.facingMode = aIn.facingMode;
        ti.desiredFacing[0] = aIn.desiredFacing[0];
        ti.desiredFacing[1] = aIn.desiredFacing[1];
        ti.yawRate = aIn.yawRate;
        ti.externalYaw = aIn.externalYaw;
        ti.externalYawStep = aIn.externalYawStep;
        ti.externalYawRate = aIn.externalYawRate;
        ti.turnRate = aIn.turnRate;
        ti.clearAngVel = aIn.clearAngVel;
        m_traj.Update(dt, ti);
        m_sinceTransition = std::fmin(m_sinceTransition + dt, 1e6f);
        if (aIn.dwellExempt)
            m_dwellExemptPending = true;
        if (aIn.queryBoneFrame >= 0 && static_cast<uint32_t>(aIn.queryBoneFrame) < m_db->NumFrames())
            m_queryBoneFrame = aIn.queryBoneFrame; // AIR CLIPS stage: latched until the next search

        // 2. inertialization decay (AIR CLIPS stage: a ForcePlay half-life applies until the next transition)
        if (m_cfg.inertializationHalflife > 0.0f)
            m_inert.Decay(m_transHalflife > 0.0f ? m_transHalflife : m_cfg.inertializationHalflife, dt);
        else
            m_inert.Reset();

        // 3. playback
        Advance(dt);

        // 4. search
        m_searchTimer -= dt;
        m_forceCooldown -= dt;
        const ClipRecord& clip = m_db->Clips()[size_t(m_clip)];
        const uint32_t local = static_cast<uint32_t>(m_frame) - clip.firstFrame;
        const bool loop = IsLoopClip(m_clip);
        const bool endOfClip = !loop && static_cast<int64_t>(local) >= static_cast<int64_t>(clip.numFrames) - 1 - m_cfg.endOfClipMarginFrames;
        const bool force = m_forceCooldown <= 0.0f && InputChanged(aIn.facingMode, aIn.yawRate);
        if (!aIn.suppressSearch && (m_first || force || endOfClip || m_searchTimer <= 0.0f))
        {
            if (force && !m_first)
            {
                ++m_diag.forcedSearches;
                m_forceCooldown = m_cfg.forceSearchCooldown;
            }
            if (endOfClip)
                ++m_diag.endOfClipSearches;
            const bool exempt = m_first || endOfClip || m_dwellExemptPending || SharpChange(aIn.facingMode, aIn.yawRate);
            DoSearch(endOfClip || m_first, aIn.facingMode, aIn.yawRate, exempt, aIn.excludeClasses, aIn.excludeCurrentClasses,
                     aIn.capRetry ? &aIn : nullptr);
            m_dwellExemptPending = false;
            m_queryBoneFrame = -1;
            m_searchTimer = m_cfg.searchInterval;
            m_first = false;
        }

        // 5. output
        Sample(m_frame, m_acc * m_fps, m_sampleA.data(), nullptr, nullptr);
        if (!m_inert.Apply(m_sampleA.data(), m_out.bones.data()))
            ++m_diag.nonFiniteResets;
        FillDiagnostics();
    }

    // Switch the allowed tag set at runtime (e.g. stand <-> crouch) without resetting playback or inertialization.
    // Reuses the capacity reserved by Init (no allocation). The next Update searches immediately; if the playing clip
    // is no longer allowed it is not kept as the current candidate. Returns false (and keeps the old set) if no
    // searchable range would remain.
    bool SetAllowedTagMask(uint64_t aMask)
    {
        if (!m_db)
            return false;
        if (aMask == m_cfg.allowedTagMask)
            return true;
        const auto clips = m_db->Clips();
        const std::vector<Range>& extra = m_cfg.allowedFrameRanges;
        const auto allowedFor = [&](size_t c, uint64_t mask) {
            const bool tagOk = mask == ~0ull || (clips[c].tagMask & mask) != 0;
            bool rangeOk = extra.empty();
            for (const Range& r : extra)
                rangeOk = rangeOk || (r.start < clips[c].firstFrame + clips[c].numFrames && r.stop > clips[c].firstFrame);
            return tagOk && rangeOk;
        };
        bool any = false;
        for (const Range& r : m_db->SearchRanges())
        {
            const int32_t c = m_db->ClipOfFrame(r.start);
            if (c >= 0 && allowedFor(size_t(c), aMask))
            {
                any = true;
                break;
            }
        }
        if (!any)
            return false;
        for (size_t c = 0; c < clips.size(); ++c)
            m_clipAllowed[c] = allowedFor(c, aMask) ? 1 : 0;
        m_ranges.clear();
        for (const Range& r : m_db->SearchRanges())
        {
            const int32_t c = m_db->ClipOfFrame(r.start);
            if (c < 0 || !m_clipAllowed[size_t(c)])
                continue;
            if (extra.empty())
            {
                m_ranges.push_back(r);
                continue;
            }
            for (const Range& e : extra)
            {
                const uint32_t s = std::max(r.start, e.start), t = std::min(r.stop, e.stop);
                if (s < t)
                    m_ranges.push_back({s, t});
            }
        }
        m_cfg.allowedTagMask = aMask;
        m_dwellExemptPending = true; // state change (stand <-> crouch): no dwell
        m_searchTimer = 0.0f;
        m_forceCooldown = 0.0f;
        return true;
    }

    // Phase 5.3c: per-clip class index (0..31, 0 = unclassified) used by MatcherInput::excludeClasses. Allocates once
    // (call after Init, outside the per-frame path). Size must equal the clip count, otherwise false.
    bool SetClipClasses(std::span<const uint8_t> aClasses)
    {
        if (!m_db || aClasses.size() != m_db->Clips().size())
            return false;
        for (uint8_t c : aClasses)
            if (c >= 32)
                return false;
        m_clipClass.assign(aClasses.begin(), aClasses.end());
        return true;
    }
    // The per-clip class buffer in use (loco P0 test hook: the class set the matcher holds).
    std::span<const uint8_t> ClipClasses() const { return m_clipClass; }
    // Phase 5.3c: dwell parameters at runtime (no reset). False = invalid, previous kept.
    bool SetDwell(float aMinDwell, float aCostRatio)
    {
        if (!std::isfinite(aMinDwell) || aMinDwell < 0.0f || aMinDwell > 2.0f || !std::isfinite(aCostRatio) || aCostRatio <= 0.0f ||
            aCostRatio > 1.0f)
            return false;
        m_cfg.minDwell = aMinDwell;
        m_cfg.dwellCostRatio = aCostRatio;
        return true;
    }
    // UTPP-MOTION SEAM handback (MotionHost.cpp MotionHandbackLocked): the next Update searches as the first one after Init - without the
    // UTPP-MOTION SEAM current frame, dwell exempt, hard cut (no inertialization from a stale pose) - after a span in which Update did
    // UTPP-MOTION SEAM not run (mm.motionDriver owned the feeds; only the trajectory advanced). Playback position and trajectory are kept. Never
    // UTPP-MOTION SEAM called by v1 alone (v1 bit-identical).
    void ForceFreshSearch() // UTPP-MOTION SEAM
    { // UTPP-MOTION SEAM
        if (!m_ok) // UTPP-MOTION SEAM
            return; // UTPP-MOTION SEAM
        m_first = true; // UTPP-MOTION SEAM search without the current frame, exempt, TransitionTo resets the inertializer
        m_searchTimer = 0.0f; // UTPP-MOTION SEAM
        m_forceCooldown = 0.0f; // UTPP-MOTION SEAM
        m_dwellExemptPending = false; // UTPP-MOTION SEAM
        m_sinceTransition = 1e6f; // UTPP-MOTION SEAM no dwell latch from before the span
        m_capHolding = false; // UTPP-MOTION SEAM
        m_capHoldTime = 0.0f; // UTPP-MOTION SEAM
        m_inert.Reset(); // UTPP-MOTION SEAM no offset from before the span (decay did not run)
        // UTPP-MOTION SEAM x AIR (merge): per-transition air overrides from before the span end here (a ForcePlay half-life, a
        // UTPP-MOTION SEAM latched query bone frame); the per-database root_z / loop override tables stay (re-applied only after Init).
        m_transHalflife = -1.0f; // UTPP-MOTION SEAM
        m_queryBoneFrame = -1; // UTPP-MOTION SEAM the caller may latch a clean one through MatcherInput::queryBoneFrame
    } // UTPP-MOTION SEAM
    const MatcherConfig& Config() const { return m_cfg; }
    // 0.5.8: cost cap at runtime (no reset). aCap 0 = off; false = invalid, previous kept.
    bool SetCostCap(float aCap, float aHoldMax)
    {
        if (!std::isfinite(aCap) || aCap < 0.0f || !std::isfinite(aHoldMax) || aHoldMax < 0.0f || aHoldMax > 2.0f)
            return false;
        m_cfg.costCap = aCap;
        m_cfg.costCapHoldMax = aHoldMax;
        if (!(aCap > 0.0f))
        {
            m_capHolding = false;
            m_capHoldTime = 0.0f;
        }
        return true;
    }
    // b5 testorder repair: the cost-cap hold of the previous playback (Init keeps it); CapHoldCarried = a hold is pending
    void ResetCapHold()
    {
        m_capHolding = false;
        m_capHoldTime = 0.0f;
    }
    bool CapHoldCarried() const { return m_capHolding || m_capHoldTime > 0.0f; }
    // 0.5.8: the clip is in the allowed (tag / frame range) set.
    bool ClipAllowed(int32_t aClip) const
    {
        return m_ok && aClip >= 0 && size_t(aClip) < m_clipAllowed.size() && m_clipAllowed[size_t(aClip)] != 0;
    }

    // ---- AIR CLIPS stage (2026-10-13; runtime only, no database format change; defaults bit-identical) ----
    // Inertialized transition to aGlobalFrame, which may lie outside the allowed tag set / search ranges (air clips).
    // aHalflife > 0: inertialization half-life used until the next transition (else the configured one). Resets the
    // search timer; the playing frame is NOT treated as a fresh start (no first-update search). False = invalid frame.
    bool ForcePlay(int32_t aGlobalFrame, float aHalflife = -1.0f)
    {
        if (!m_ok || aGlobalFrame < 0 || static_cast<uint32_t>(aGlobalFrame) >= m_db->NumFrames() || m_db->ClipOfFrame(uint32_t(aGlobalFrame)) < 0)
            return false;
        TransitionTo(aGlobalFrame);
        m_transHalflife = (std::isfinite(aHalflife) && aHalflife > 0.0f) ? aHalflife : -1.0f;
        m_first = false;
        m_searchTimer = m_cfg.searchInterval;
        FillDiagnostics();
        return true;
    }
    // Per global frame root vertical (m) subtracted from the Hips (firstBodyBone) local p.z in every Sample (output,
    // inertialization sources and segment velocities). Empty span = off (bit-identical). Size must equal NumFrames().
    // Allocates once (call after Init, outside the per-frame path); Init clears it.
    bool SetRootZ(std::span<const float> aPerGlobalFrame)
    {
        if (!m_ok)
            return false;
        if (aPerGlobalFrame.empty())
        {
            m_rootZ.clear();
            return true;
        }
        if (aPerGlobalFrame.size() != m_db->NumFrames())
            return false;
        for (float v : aPerGlobalFrame)
            if (!std::isfinite(v))
                return false;
        m_rootZ.assign(aPerGlobalFrame.begin(), aPerGlobalFrame.end());
        return true;
    }
    bool HasRootZ() const { return !m_rootZ.empty(); }
    float RootZAt(int32_t aGlobalFrame) const
    {
        return (aGlobalFrame >= 0 && size_t(aGlobalFrame) < m_rootZ.size()) ? m_rootZ[size_t(aGlobalFrame)] : 0.0f;
    }
    // Treats a non-loop clip as a loop (frame N-1 == frame 0) in playback AND in the end-of-clip test. Init clears it.
    bool SetLoopOverride(int32_t aClip, bool aOn)
    {
        if (!m_ok || aClip < 0 || size_t(aClip) >= m_db->Clips().size())
            return false;
        if (m_loopOverride.size() != m_db->Clips().size())
            m_loopOverride.assign(m_db->Clips().size(), 0);
        m_loopOverride[size_t(aClip)] = aOn ? 1 : 0;
        return true;
    }
    // The single loop rule (database flag or override) used by NextPlayFrame and Update's end-of-clip test.
    bool IsLoopClip(int32_t aClip) const
    {
        if (aClip < 0 || !m_db || size_t(aClip) >= m_db->Clips().size())
            return false;
        const ClipRecord& clip = m_db->Clips()[size_t(aClip)];
        return (clip.flags & kClipLoop) != 0 || (size_t(aClip) < m_loopOverride.size() && m_loopOverride[size_t(aClip)] != 0);
    }
    float TransitionHalflife() const { return m_transHalflife > 0.0f ? m_transHalflife : m_cfg.inertializationHalflife; }
    int32_t PendingQueryBoneFrame() const { return m_queryBoneFrame; }

    // Root (Trajectory bone) yaw rate of the segment being played, rad/s in MATCHER time (multiply by the playback
    // rate for real time). 0 while a non-loop clip holds its last frame (the ROOT entry there is only extrapolation).
    float RootYawRate() const
    {
        if (!m_ok || NextPlayFrame(m_frame) == m_frame)
            return 0.0f;
        const float r = m_db->Root(static_cast<uint32_t>(m_frame))->dyaw * m_fps;
        return std::isfinite(r) ? r : 0.0f;
    }

    // Phase 5.3: planar root (Trajectory bone) velocity of the segment being played, m/s in MATCHER time, in the
    // character frame of the played frame (multiply by the playback rate for real time). 0 while a non-loop clip holds
    // its last frame. Continuous within a segment (unlike consumedRoot, which is quantized to whole frames).
    void RootVelocity(float& aVx, float& aVy) const
    {
        aVx = aVy = 0.0f;
        if (!m_ok || NextPlayFrame(m_frame) == m_frame)
            return;
        const RootDelta* rd = m_db->Root(static_cast<uint32_t>(m_frame));
        const float vx = rd->dx * m_fps, vy = rd->dy * m_fps;
        if (std::isfinite(vx) && std::isfinite(vy))
        {
            aVx = vx;
            aVy = vy;
        }
    }

    const Pose& Output() const { return m_out; }
    const MatcherDiagnostics& Diagnostics() const { return m_diag; }
    const Trajectory& GetTrajectory() const { return m_traj; }
    Trajectory& GetTrajectory() { return m_traj; }
    std::span<const Range> AllowedRanges() const { return m_ranges; }
    // Last query (raw = denormalized, character frame; normalized = what was searched).
    std::span<const float> LastRawQuery() const { return m_query; }
    std::span<const float> LastNormalizedQuery() const { return m_queryNorm; }

    // Builds the current query into LastRawQuery()/LastNormalizedQuery() without searching (tests / tools).
    void BuildQuery()
    {
        if (!m_ok)
            return;
        const auto f = m_db->Features(static_cast<uint32_t>(m_queryBoneFrame >= 0 ? m_queryBoneFrame : m_frame)); // AIR CLIPS: override
        m_db->DenormalizeFeatures(f, m_query);
        const float* tp = m_traj.PositionFeatures();
        const float* td = m_traj.DirectionFeatures();
        for (uint32_t k = 0; k < 2 * m_numHorizons; ++k)
        {
            m_query[m_trajPosFirst + k] = tp[k];
            m_query[m_trajDirFirst + k] = td[k];
        }
        m_db->NormalizeQuery(m_query, m_queryNorm);
    }

private:
    static bool ValidConfig(const MatcherConfig& c)
    {
        auto pos = [](float v) { return std::isfinite(v) && v > 0.0f; };
        auto nonneg = [](float v) { return std::isfinite(v) && v >= 0.0f; };
        return pos(c.searchInterval) && pos(c.forceSearchVelocityThreshold) && pos(c.forceSearchFacingThreshold) &&
               pos(c.forceSearchYawRateThreshold) && nonneg(c.forceSearchCooldown) && nonneg(c.transitionCost) &&
               c.ignoreSurroundingFrames >= 0 && c.endOfClipMarginFrames >= 0 && std::isfinite(c.inertializationHalflife) &&
               pos(c.velocityHalflife) && pos(c.rotationHalflife) && pos(c.maxDt) && nonneg(c.minDwell) &&
               pos(c.dwellCostRatio) && c.dwellCostRatio <= 1.0f && pos(c.dwellExemptVelocity) && pos(c.dwellExemptFacing);
    }

    int32_t NextPlayFrame(int32_t aFrame) const
    {
        const int32_t c = m_db->ClipOfFrame(static_cast<uint32_t>(aFrame));
        const ClipRecord& clip = m_db->Clips()[size_t(c)];
        const uint32_t local = static_cast<uint32_t>(aFrame) - clip.firstFrame;
        if (local + 1 < clip.numFrames)
            return aFrame + 1;
        if (IsLoopClip(c) && clip.numFrames >= 2)
            return static_cast<int32_t>(clip.firstFrame + 1); // frame N-1 == frame 0 -> continue with frame 1
        return aFrame;
    }

    static BoneTransform ReadBone(std::span<const float> aPose, uint32_t b)
    {
        const float* s = aPose.data() + size_t(b) * 7;
        BoneTransform t;
        t.p = {s[0], s[1], s[2]};
        t.q = {s[3], s[4], s[5], s[6]};
        return t;
    }

    // Samples the pose at (frame, alpha in [0,1)) and optionally the local velocities of that segment.
    void Sample(int32_t aFrame, float aAlpha, BoneTransform* aOut, math::Vec3* aLv, math::Vec3* aAv) const
    {
        int32_t f0 = aFrame;
        int32_t f1 = NextPlayFrame(aFrame);
        float alpha = std::min(std::max(aAlpha, 0.0f), 1.0f);
        int32_t v0 = f0;
        if (f1 == f0)
        {
            alpha = 0.0f;
            const int32_t c = m_db->ClipOfFrame(static_cast<uint32_t>(f0));
            if (static_cast<uint32_t>(f0) > m_db->Clips()[size_t(c)].firstFrame)
                v0 = f0 - 1; // velocity of the last segment (hold)
        }
        const auto p0 = m_db->Pose(static_cast<uint32_t>(f0));
        const auto p1 = m_db->Pose(static_cast<uint32_t>(f1));
        const uint32_t nb = m_db->NumBones();
        for (uint32_t b = 0; b < nb; ++b)
        {
            if (b < m_cfg.firstBodyBone)
            {
                aOut[b] = BoneTransform{};
                if (aLv)
                {
                    aLv[b] = {};
                    aAv[b] = {};
                }
                continue;
            }
            const BoneTransform a = ReadBone(p0, b);
            const BoneTransform c = ReadBone(p1, b);
            if (alpha == 0.0f)
                aOut[b] = a;
            else
                aOut[b] = BoneTransform{math::Lerp(a.p, c.p, alpha), math::QuatNlerpShortest(a.q, c.q, alpha)};
            if (aLv)
            {
                const BoneTransform va = (v0 == f0) ? a : ReadBone(m_db->Pose(static_cast<uint32_t>(v0)), b);
                const BoneTransform vb = (v0 == f0) ? c : a;
                aLv[b] = (vb.p - va.p) * m_fps;
                aAv[b] = math::QuatAngularVelocity(vb.q, va.q, m_fps);
            }
        }
        // AIR CLIPS stage: root vertical out of the Hips (empty table = untouched, bit-identical)
        if (!m_rootZ.empty() && m_cfg.firstBodyBone < nb)
        {
            const uint32_t h = m_cfg.firstBodyBone;
            const float z0 = m_rootZ[size_t(f0)], z1 = m_rootZ[size_t(f1)];
            aOut[h].p.z -= (alpha == 0.0f) ? z0 : z0 + (z1 - z0) * alpha;
            if (aLv)
            {
                const float za = (v0 == f0) ? z0 : m_rootZ[size_t(v0)];
                const float zb = (v0 == f0) ? z1 : z0;
                aLv[h].z -= (zb - za) * m_fps;
            }
        }
    }

    void Advance(float aDt)
    {
        m_diag.consumedRoot[0] = m_diag.consumedRoot[1] = m_diag.consumedRoot[2] = 0.0f;
        m_acc += aDt;
        for (int guard = 0; m_acc >= m_frameDt && guard < 64; ++guard)
        {
            const int32_t next = NextPlayFrame(m_frame);
            if (next == m_frame)
            {
                m_acc = 0.0f; // non-loop end: hold the last frame until the forced search moves away
                break;
            }
            const RootDelta* rd = m_db->Root(static_cast<uint32_t>(m_frame));
            float rx = 0.0f, ry = 0.0f;
            math::Rot2(m_diag.consumedRoot[2], rd->dx, rd->dy, rx, ry);
            m_diag.consumedRoot[0] += rx;
            m_diag.consumedRoot[1] += ry;
            m_diag.consumedRoot[2] += rd->dyaw;
            m_frame = next;
            m_acc -= m_frameDt;
        }
        if (!(m_acc >= 0.0f) || m_acc >= m_frameDt)
            m_acc = 0.0f;
        m_clip = m_db->ClipOfFrame(static_cast<uint32_t>(m_frame));
    }

    float GoalYawWorld() const { return m_traj.WorldYaw() + m_traj.DesiredYawRelative(); }

    bool InputChanged(FacingMode aMode, float aYawRate) const
    {
        const float dvx = m_traj.GoalVelocityX() - m_lastGoalVx;
        const float dvy = m_traj.GoalVelocityY() - m_lastGoalVy;
        if (std::sqrt(dvx * dvx + dvy * dvy) > m_cfg.forceSearchVelocityThreshold)
            return true;
        if (aMode == FacingMode::YawRate)
            return std::fabs((std::isfinite(aYawRate) ? aYawRate : 0.0f) - m_lastYawRate) > m_cfg.forceSearchYawRateThreshold;
        return std::fabs(math::WrapAngle(GoalYawWorld() - m_lastGoalYaw)) > m_cfg.forceSearchFacingThreshold;
    }

    // Sharp input change since the last search (dwell exemption): same quantities as InputChanged, larger thresholds.
    bool SharpChange(FacingMode aMode, float aYawRate) const
    {
        const float dvx = m_traj.GoalVelocityX() - m_lastGoalVx;
        const float dvy = m_traj.GoalVelocityY() - m_lastGoalVy;
        if (std::sqrt(dvx * dvx + dvy * dvy) > m_cfg.dwellExemptVelocity)
            return true;
        if (aMode == FacingMode::YawRate)
            return std::fabs((std::isfinite(aYawRate) ? aYawRate : 0.0f) - m_lastYawRate) > 2.0f * m_cfg.forceSearchYawRateThreshold;
        return std::fabs(math::WrapAngle(GoalYawWorld() - m_lastGoalYaw)) > m_cfg.dwellExemptFacing;
    }

    // Cost of one frame against the last normalized query (same accumulation as Database::InitBest).
    float FrameCost(int32_t aFrame) const
    {
        const auto f = m_db->Features(static_cast<uint32_t>(aFrame));
        float c = 0.0f;
        for (size_t j = 0; j < f.size() && j < m_queryNorm.size(); ++j)
        {
            const float d = m_queryNorm[j] - f[j];
            c += d * d;
        }
        return c;
    }

    void DoSearch(bool aWithoutCurrent, FacingMode aMode, float aYawRate, bool aExempt = true, uint32_t aExclude = 0,
                  uint32_t aExcludeCurrent = 0, const MatcherInput* aCap = nullptr)
    {
        bool capHeld = false;
        BuildQuery();
        const bool currAllowed =
            m_clipAllowed[size_t(m_clip)] != 0 && ((aExcludeCurrent >> m_clipClass[size_t(m_clip)]) & 1u) == 0;
        const int32_t curr = (aWithoutCurrent || !currAllowed) ? -1 : m_frame;
        // 5.3c clip-class filter (candidates only); an empty result falls back to the unfiltered set.
        std::span<const Range> ranges = m_ranges;
        if (aExclude != 0)
        {
            m_searchRanges.clear();
            for (const Range& r : m_ranges)
            {
                const int32_t c = m_db->ClipOfFrame(r.start);
                if (c >= 0 && ((aExclude >> m_clipClass[size_t(c)]) & 1u) == 0)
                    m_searchRanges.push_back(r);
            }
            if (!m_searchRanges.empty())
            {
                ranges = m_searchRanges;
                ++m_diag.filteredSearches;
            }
        }
        const auto t0 = std::chrono::steady_clock::now();
        SearchResult r = m_db->SearchInRanges(m_queryNorm, ranges, curr, m_cfg.ignoreSurroundingFrames, m_cfg.transitionCost);
        if (m_cfg.costCap > 0.0f && r.index >= 0 && r.index != m_frame && r.cost > m_cfg.costCap)
        {
            ++m_diag.costCapHits;
            m_diag.costCapLastIndex = r.index;
            m_diag.costCapLastCost = r.cost;
            if (curr >= 0 && m_capHoldTime < m_cfg.costCapHoldMax)
            {
                // keep the continuing frame (no transition): the playing clip continues
                ++m_diag.costCapHolds;
                capHeld = true;
                m_diag.costCapHeldThisUpdate = true;
            }
            else if (aCap)
            {
                const int32_t rc = m_db->ClipOfFrame(static_cast<uint32_t>(r.index));
                const bool never = rc >= 0 && ((aCap->capNeverClasses >> m_clipClass[size_t(rc)]) & 1u) != 0;
                m_searchRanges.clear();
                for (const Range& q : m_ranges)
                {
                    const int32_t c = m_db->ClipOfFrame(q.start);
                    if (c >= 0 && ((aCap->capRetryExclude >> m_clipClass[size_t(c)]) & 1u) == 0)
                        m_searchRanges.push_back(q);
                }
                if (!m_searchRanges.empty())
                {
                    ++m_diag.costCapRetries;
                    const SearchResult r2 =
                        m_db->SearchInRanges(m_queryNorm, m_searchRanges, curr, m_cfg.ignoreSurroundingFrames, m_cfg.transitionCost);
                    if (r2.index >= 0 && (never || r2.cost < r.cost))
                    {
                        if (never)
                            ++m_diag.costCapNever;
                        r = r2;
                    }
                }
            }
            if (!capHeld && r.index != m_frame && r.cost > m_cfg.costCap)
                ++m_diag.costCapForced;
        }
        // the hold timer runs only while consecutive searches keep the frame because of the cap
        m_capHolding = capHeld;
        if (!capHeld)
            m_capHoldTime = 0.0f;
        const auto t1 = std::chrono::steady_clock::now();
        m_diag.lastSearchMicros = std::chrono::duration<double, std::micro>(t1 - t0).count();
        ++m_diag.searches;
        m_diag.searchedThisUpdate = true;
        m_diag.lastSearchIndex = capHeld ? m_frame : r.index;
        m_diag.lastSearchCost = capHeld ? FrameCost(m_frame) : r.cost;
        m_lastGoalVx = m_traj.GoalVelocityX();
        m_lastGoalVy = m_traj.GoalVelocityY();
        m_lastGoalYaw = GoalYawWorld();
        m_lastYawRate = (aMode == FacingMode::YawRate && std::isfinite(aYawRate)) ? aYawRate : 0.0f;
        if (r.index >= 0 && r.index != m_frame && !capHeld)
        {
            if (m_cfg.minDwell > 0.0f && curr >= 0 && m_sinceTransition < m_cfg.minDwell)
            {
                if (aExempt)
                    ++m_diag.dwellExempts;
                else if (!(r.cost < m_cfg.dwellCostRatio * FrameCost(m_frame)))
                {
                    ++m_diag.dwellHolds; // keep the young segment (improvement too small)
                    return;
                }
            }
            TransitionTo(r.index);
        }
    }

    void TransitionTo(int32_t aDst)
    {
        const float alpha = m_acc * m_fps;
        if (m_cfg.inertializationHalflife > 0.0f && !m_first)
        {
            Sample(m_frame, alpha, m_sampleA.data(), m_lvA.data(), m_avA.data());
            Sample(aDst, alpha, m_sampleB.data(), m_lvB.data(), m_avB.data());
            m_inert.Transition(m_sampleA.data(), m_lvA.data(), m_avA.data(), m_sampleB.data(), m_lvB.data(), m_avB.data());
        }
        else
        {
            m_inert.Reset();
        }
        m_frame = aDst;
        m_clip = m_db->ClipOfFrame(static_cast<uint32_t>(aDst));
        m_transHalflife = -1.0f; // AIR CLIPS stage: ForcePlay sets its own after this
        ++m_diag.transitions;
        m_diag.transitionedThisUpdate = true;
        m_sinceTransition = 0.0f;
    }

    void FillDiagnostics()
    {
        m_diag.frame = m_frame;
        m_diag.clip = m_clip;
        m_diag.clipFrame = m_frame - static_cast<int32_t>(m_db->Clips()[size_t(m_clip)].firstFrame);
        m_diag.alpha = m_acc * m_fps;
        m_traj.LastStep(m_diag.simStep[0], m_diag.simStep[1], m_diag.simStep[2]);
        m_traj.LocalVelocity(m_diag.simVelocity[0], m_diag.simVelocity[1]);
        for (uint32_t k = 0; k < 2 * m_numHorizons; ++k)
        {
            m_diag.trajectoryPosition[k] = m_traj.PositionFeatures()[k];
            m_diag.trajectoryDirection[k] = m_traj.DirectionFeatures()[k];
        }
        m_inert.MaxOffsets(m_diag.maxOffsetAngle, m_diag.maxOffsetDist);
        m_diag.sinceTransition = m_sinceTransition;
    }

    bool m_ok = false;
    const Database* m_db = nullptr;
    MatcherConfig m_cfg{};
    float m_fps = 30.0f;
    float m_frameDt = 1.0f / 30.0f;
    uint32_t m_trajPosFirst = 0, m_trajDirFirst = 0, m_numHorizons = 0;
    Trajectory m_traj;
    PoseInertializer m_inert;
    std::vector<uint8_t> m_clipAllowed;
    std::vector<Range> m_ranges;
    std::vector<Range> m_searchRanges; // 5.3c class-filtered copy (capacity reserved in Init)
    std::vector<uint8_t> m_clipClass;  // 5.3c per-clip class (0 = unclassified)
    float m_sinceTransition = 1e6f;
    bool m_dwellExemptPending = false;
    bool m_capHolding = false;  // 0.5.8: the last search kept the frame because of the cost cap
    float m_capHoldTime = 0.0f; // s matcher time of consecutive cap holds
    std::vector<float> m_query, m_queryNorm;
    std::vector<BoneTransform> m_sampleA, m_sampleB;
    std::vector<math::Vec3> m_lvA, m_avA, m_lvB, m_avB;
    Pose m_out;

    int32_t m_frame = -1;
    int32_t m_clip = -1;
    float m_acc = 0.0f;
    float m_searchTimer = 0.0f;
    float m_forceCooldown = 0.0f;
    bool m_first = true;
    float m_lastGoalVx = 0.0f, m_lastGoalVy = 0.0f, m_lastGoalYaw = 0.0f, m_lastYawRate = 0.0f;
    MatcherDiagnostics m_diag{};
    // AIR CLIPS stage
    std::vector<float> m_rootZ;          // per global frame, empty = off
    std::vector<uint8_t> m_loopOverride; // per clip, empty = none
    float m_transHalflife = -1.0f;       // ForcePlay half-life until the next transition (-1 = configured)
    int32_t m_queryBoneFrame = -1;       // latched MatcherInput::queryBoneFrame
};

} // namespace utpp::mm
