// UTPP Locomotion core (Phase 5N step 4a): pure logic, no Windows / RED4ext dependency (offline-testable).
//
// Standalone behaviour: while Locomotion is active, the player state machine
// scripts see every stick / WASD direction as "forward" relative to the body:
//   GetActionValue(n"MoveX")  -> 0
//   GetActionValue(n"MoveY")  -> |(MoveX, MoveY)|  (vanilla magnitude kept: walk / sprint thresholds unchanged)
//   GetInputHeading()         -> 0
// The real direction stays available as the raw camera-relative stick angle (RawAngleDeg) for the body-turn controller.
//
// Yaw / angle convention (same as utpp.reds): degrees, counter-clockwise about +Z, 0 = stick forward (camera forward),
// +90 = stick left, -90 = stick right, 180 = back. RawAngleDeg(x, y) = atan2(-x, y).
//
// Thread model: Filter* run on whatever thread executes the PSM scripts (hot path: atomics only); Heartbeat and the
// setters run on the game thread (natives). Time is injected (seconds, monotonic) so tests are deterministic.
//
// Move override (vehicle approach walk): a bounded, time-limited replacement of the camera-relative stick the PSM scripts
// read. SetMoveOverride(x, y, headingDeg, ttl, now) holds (x, y) = raw stick (MoveX right+, MoveY forward+, camera frame)
// and headingDeg = GetInputHeading value for at most `ttl` seconds (clamped to kMaxOverrideTtlSec); it is replaced by every
// new call and dropped by ClearMoveOverride or by expiry, so a stale override can never stick. It needs the hook installed
// and not killed (Reasons bits kNotInstalled / kKilled), but NOT the user setting or the Locomotion gates: the override is
// fed into the same path as the real stick, so with Locomotion active the usual rewrite (MoveX -> 0, MoveY -> |stick|,
// heading -> 0) and the body-turn target (RawAngle) follow it, and with Locomotion inactive the PSM sees the override stick
// and heading as they are. The hardware stick stays readable through Real* (abort-on-input checks). With no override set
// every output and counter is bit-identical to the pre-override core (tests/LocomotionOverrideTest.cpp keeps a verbatim copy
// of the old logic as the reference). Note: this changes what the PSM SCRIPTS read; whether the engine's native movement
// also follows it is a runtime question the caller must guard (see utpp_vehicle_approach.reds, no-motion guard).

#pragma once

#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace utpp::freemove
{
constexpr float kPi = 3.14159265358979323846f;
constexpr double kMaxOverrideTtlSec = 0.5; // longest time one SetMoveOverride call can hold the stick

// Reasons Locomotion is not active (bit set = blocked). Bits 0..14 come from the script heartbeat (utpp_locomotion.reds,
// mirror there); bits 16+ are native state.
enum Reason : uint32_t
{
    kVehicle = 1u << 0,      // PSM Vehicle / mounted
    kScene = 1u << 1,        // high level scene tier 3..5 / unknown
    kSwimming = 1u << 2,
    kWorkspot = 1u << 3,
    kTakedown = 1u << 4,     // takedown / grapple
    kCarrying = 1u << 5,     // body carry
    kReserved64 = 1u << 6,   // unused (utpp.reds "detailed locomotion" bit is replaced by kTraversal below)
    kVitals = 1u << 7,       // dead / resurrecting
    kMenu = 1u << 8,         // menu / photo mode
    kNoBlackboard = 1u << 9,
    kReaction = 1u << 10,    // stagger
    kTraversal = 1u << 11,   // ladder, climb, vault, knockdown, felled
    kAiming = 1u << 12,      // UpperBody == Aim, AimWalk
    kWeapon = 1u << 13,      // item in AttachmentSlots.WeaponRight
    kHold = 1u << 14,        // external hold (free look, camera transition, other UTPP modules)
    kNotTpp = 1u << 16,      // no third-person view this heartbeat
    kStale = 1u << 17,       // no heartbeat within staleSec (PSM locomotion not ticking: vehicle, menus, loading)
    kDisabled = 1u << 18,    // user setting off (default)
    kKilled = 1u << 19,      // kill switch latched for this session (native or fault)
    kNotInstalled = 1u << 20, // hook not installed (validation failed, kill file, not yet resolved)
    // bit 21 unused
};

// Script bits that block by default (all of them). SetBlockMask can clear e.g. kWeapon.
constexpr uint32_t kScriptBits = 0x7FFFu;
constexpr uint32_t kDefaultBlockMask = kScriptBits & ~kReserved64;

enum class Axis : int
{
    X = 0,
    Y = 1,
};

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

// Camera-relative stick angle (deg, CCW, 0 = forward). 0 for a zero / non-finite stick.
inline float RawAngleDeg(float aX, float aY)
{
    if (!std::isfinite(aX) || !std::isfinite(aY) || (aX == 0.0f && aY == 0.0f))
        return 0.0f;
    return WrapDeg(std::atan2(-aX, aY) * (180.0f / kPi));
}

inline float StickMagnitude(float aX, float aY)
{
    if (!std::isfinite(aX) || !std::isfinite(aY))
        return 0.0f;
    return std::sqrt(aX * aX + aY * aY);
}

struct Params
{
    double staleSec = 0.25;    // heartbeat timeout
    double axisFreshSec = 0.15; // how long a cached axis value is used to complete the other axis
    float deadzone = 0.15f;    // stick magnitude below which nothing is rewritten
    float tppCameraDist = 0.6f; // horizontal camera-to-player distance (m) treated as third-person view
};

inline bool ValidParams(const Params& aP)
{
    return std::isfinite(aP.staleSec) && aP.staleSec > 0.0 && aP.staleSec <= 5.0 && std::isfinite(aP.axisFreshSec) &&
           aP.axisFreshSec > 0.0 && aP.axisFreshSec <= 1.0 && std::isfinite(aP.deadzone) && aP.deadzone >= 0.0f &&
           aP.deadzone <= 0.9f && std::isfinite(aP.tppCameraDist) && aP.tppCameraDist >= 0.0f &&
           aP.tppCameraDist <= 10.0f;
}

// TPP decision from the heartbeat inputs: an explicit owner flag (UTPP camera) or the active camera
// far enough from the player capsule horizontally (FPP camera sits within ~0.2 m of the capsule axis).
inline bool IsThirdPerson(bool aOwnerSaysTpp, float aCameraDist2D, float aThreshold)
{
    if (aOwnerSaysTpp)
        return true;
    return std::isfinite(aCameraDist2D) && aCameraDist2D > aThreshold;
}

class Core
{
public:
    // ---- cold setters (game thread) ----
    void SetInstalled(bool aOn) { m_installed.store(aOn, std::memory_order_release); }
    void SetEnabled(bool aOn) { m_enabled.store(aOn, std::memory_order_release); }
    void Kill() { m_killed.store(true, std::memory_order_release); }
    void SetHold(bool aOn) { m_hold.store(aOn, std::memory_order_release); }
    void SetBlockMask(uint32_t aMask) { m_blockMask.store(aMask & kScriptBits, std::memory_order_release); }
    bool SetParams(const Params& aP)
    {
        if (!ValidParams(aP))
            return false;
        m_staleSec.store(aP.staleSec, std::memory_order_relaxed);
        m_axisFreshSec.store(aP.axisFreshSec, std::memory_order_relaxed);
        m_deadzone.store(aP.deadzone, std::memory_order_relaxed);
        m_tppDist.store(aP.tppCameraDist, std::memory_order_relaxed);
        return true;
    }
    Params GetParams() const
    {
        Params p;
        p.staleSec = m_staleSec.load(std::memory_order_relaxed);
        p.axisFreshSec = m_axisFreshSec.load(std::memory_order_relaxed);
        p.deadzone = m_deadzone.load(std::memory_order_relaxed);
        p.tppCameraDist = m_tppDist.load(std::memory_order_relaxed);
        return p;
    }

    bool Installed() const { return m_installed.load(std::memory_order_acquire); }
    bool Enabled() const { return m_enabled.load(std::memory_order_acquire); }
    bool Killed() const { return m_killed.load(std::memory_order_acquire); }
    uint32_t BlockMask() const { return m_blockMask.load(std::memory_order_relaxed); }

    // Script heartbeat (every PSM locomotion update). aGameBits: Reason bits 0..14 seen this frame.
    void Heartbeat(double aNow, bool aOwnerSaysTpp, float aCameraDist2D, uint32_t aGameBits)
    {
        const bool tpp = IsThirdPerson(aOwnerSaysTpp, aCameraDist2D, m_tppDist.load(std::memory_order_relaxed));
        m_tpp.store(tpp, std::memory_order_relaxed);
        m_gameBits.store(aGameBits & kScriptBits, std::memory_order_relaxed);
        m_lastBeat.store(aNow, std::memory_order_release);
        m_beats.fetch_add(1, std::memory_order_relaxed);
        const bool active = Reasons(aNow) == 0;
        if (active && !m_wasActive)
            m_activations.fetch_add(1, std::memory_order_relaxed);
        m_wasActive = active;
    }

    uint32_t Reasons(double aNow) const
    {
        uint32_t r = 0;
        if (!m_installed.load(std::memory_order_acquire))
            r |= kNotInstalled;
        if (m_killed.load(std::memory_order_acquire))
            r |= kKilled;
        if (!m_enabled.load(std::memory_order_acquire))
            r |= kDisabled;
        const double beat = m_lastBeat.load(std::memory_order_acquire);
        if (beat < 0.0 || !(aNow - beat <= m_staleSec.load(std::memory_order_relaxed)) || aNow < beat)
            r |= kStale;
        if (!m_tpp.load(std::memory_order_relaxed))
            r |= kNotTpp;
        r |= m_gameBits.load(std::memory_order_relaxed) & m_blockMask.load(std::memory_order_relaxed);
        if (m_hold.load(std::memory_order_acquire))
            r |= kHold;
        return r;
    }

    bool Active(double aNow) const { return Reasons(aNow) == 0; }

    // ---- hot path (script thread) ----
    // Records the raw axis value and returns what the script should see.
    float FilterAxis(Axis aAxis, float aRaw, double aNow)
    {
        const int i = static_cast<int>(aAxis);
        const int o = 1 - i;
        // Hardware stick as the engine reported it (never replaced).
        m_real[i].store(std::isfinite(aRaw) ? aRaw : 0.0f, std::memory_order_relaxed);
        m_realAt[i].store(aNow, std::memory_order_release);
        OverrideSnap ov;
        if (ReadOverride(aNow, ov))
        {
            aRaw = aAxis == Axis::X ? ov.x : ov.y;
            m_overrideAxisReads.fetch_add(1, std::memory_order_relaxed);
        }
        m_raw[i].store(std::isfinite(aRaw) ? aRaw : 0.0f, std::memory_order_relaxed);
        m_rawAt[i].store(aNow, std::memory_order_release);
        m_axisReads.fetch_add(1, std::memory_order_relaxed);
        if (!std::isfinite(aRaw) || !Active(aNow))
            return aRaw;
        const float other = Fresh(o, aNow) ? m_raw[o].load(std::memory_order_relaxed) : 0.0f;
        const float x = aAxis == Axis::X ? aRaw : other;
        const float y = aAxis == Axis::Y ? aRaw : other;
        const float mag = StickMagnitude(x, y);
        if (!(mag >= m_deadzone.load(std::memory_order_relaxed)))
            return aRaw;
        m_axisRewrites.fetch_add(1, std::memory_order_relaxed);
        return aAxis == Axis::X ? 0.0f : mag;
    }

    float FilterHeading(float aRaw, double aNow)
    {
        m_headingReads.fetch_add(1, std::memory_order_relaxed);
        OverrideSnap ov;
        const bool haveOv = ReadOverride(aNow, ov);
        if (!Active(aNow) || !HasInput(aNow))
            return haveOv ? ov.headingDeg : aRaw;
        m_headingRewrites.fetch_add(1, std::memory_order_relaxed);
        return 0.0f;
    }

    // Raw stick from the last script reads (both axes fresh and above the deadzone).
    bool HasInput(double aNow) const
    {
        if (!Fresh(0, aNow) || !Fresh(1, aNow))
            return false;
        return StickMagnitude(m_raw[0].load(std::memory_order_relaxed), m_raw[1].load(std::memory_order_relaxed)) >=
               m_deadzone.load(std::memory_order_relaxed);
    }

    // Camera-relative raw stick angle (deg); 0 without input.
    float RawAngle(double aNow) const
    {
        if (!HasInput(aNow))
            return 0.0f;
        return RawAngleDeg(m_raw[0].load(std::memory_order_relaxed), m_raw[1].load(std::memory_order_relaxed));
    }

    float RawAxis(Axis aAxis) const { return m_raw[static_cast<int>(aAxis)].load(std::memory_order_relaxed); }

    // ---- move override (game thread setters; read on the script thread) ----
    // x / y: stick (clamped to the unit disc); headingDeg: GetInputHeading value (wrapped). False = rejected (non-finite input,
    // ttl <= 0, or hook not installed / killed); the previous override then stays untouched.
    bool SetMoveOverride(float aX, float aY, float aHeadingDeg, double aTtlSec, double aNow)
    {
        if (!std::isfinite(aX) || !std::isfinite(aY) || !std::isfinite(aHeadingDeg) || !std::isfinite(aTtlSec) ||
            !std::isfinite(aNow) || !(aTtlSec > 0.0) || !OverrideAllowed())
            return false;
        const float mag = StickMagnitude(aX, aY);
        if (mag > 1.0f)
        {
            aX /= mag;
            aY /= mag;
        }
        const double ttl = aTtlSec > kMaxOverrideTtlSec ? kMaxOverrideTtlSec : aTtlSec;
        const uint32_t g = m_ovGen.load(std::memory_order_relaxed);
        m_ovGen.store(g + 1u, std::memory_order_release); // odd: write in progress
        m_ovX.store(aX, std::memory_order_relaxed);
        m_ovY.store(aY, std::memory_order_relaxed);
        m_ovHeading.store(WrapDeg(aHeadingDeg), std::memory_order_relaxed);
        m_ovUntil.store(aNow + ttl, std::memory_order_relaxed);
        m_ovGen.store(g + 2u, std::memory_order_release);
        m_overrideSets.fetch_add(1, std::memory_order_relaxed);
        return true;
    }

    void ClearMoveOverride()
    {
        const uint32_t g = m_ovGen.load(std::memory_order_relaxed);
        m_ovGen.store(g + 1u, std::memory_order_release);
        m_ovUntil.store(-1.0, std::memory_order_relaxed);
        m_ovGen.store(g + 2u, std::memory_order_release);
        // The effective stick cache returns to the hardware values at once (not only at the next script read).
        for (int i = 0; i < 2; ++i)
            m_raw[i].store(m_real[i].load(std::memory_order_relaxed), std::memory_order_relaxed);
    }

    // True while an override would be applied at aNow (installed, not killed, set and not expired).
    bool OverrideActive(double aNow) const
    {
        OverrideSnap ov;
        return ReadOverride(aNow, ov);
    }
    // Consistent snapshot of the override that would be applied at aNow (false = none).
    bool GetMoveOverride(double aNow, float& aX, float& aY, float& aHeadingDeg) const
    {
        OverrideSnap ov;
        if (!ReadOverride(aNow, ov))
            return false;
        aX = ov.x;
        aY = ov.y;
        aHeadingDeg = ov.headingDeg;
        return true;
    }
    // The hook is proven and the kill switch is not latched: an override can reach the script thread at all.
    bool OverrideAllowed() const
    {
        return m_installed.load(std::memory_order_acquire) && !m_killed.load(std::memory_order_acquire);
    }

    // Hardware stick (the engine's value before any override or rewrite) from the last script reads.
    bool RealHasInput(double aNow) const
    {
        if (!RealFresh(0, aNow) || !RealFresh(1, aNow))
            return false;
        return StickMagnitude(m_real[0].load(std::memory_order_relaxed), m_real[1].load(std::memory_order_relaxed)) >=
               m_deadzone.load(std::memory_order_relaxed);
    }
    float RealMagnitude(double aNow) const
    {
        if (!RealFresh(0, aNow) || !RealFresh(1, aNow))
            return 0.0f;
        return StickMagnitude(m_real[0].load(std::memory_order_relaxed), m_real[1].load(std::memory_order_relaxed));
    }
    float RealAngle(double aNow) const
    {
        if (!RealHasInput(aNow))
            return 0.0f;
        return RawAngleDeg(m_real[0].load(std::memory_order_relaxed), m_real[1].load(std::memory_order_relaxed));
    }

    uint64_t OverrideSets() const { return m_overrideSets.load(std::memory_order_relaxed); }
    uint64_t OverrideAxisReads() const { return m_overrideAxisReads.load(std::memory_order_relaxed); }

    // Counters.
    uint64_t AxisReads() const { return m_axisReads.load(std::memory_order_relaxed); }
    uint64_t AxisRewrites() const { return m_axisRewrites.load(std::memory_order_relaxed); }
    uint64_t HeadingReads() const { return m_headingReads.load(std::memory_order_relaxed); }
    uint64_t HeadingRewrites() const { return m_headingRewrites.load(std::memory_order_relaxed); }
    uint64_t Beats() const { return m_beats.load(std::memory_order_relaxed); }
    uint64_t Activations() const { return m_activations.load(std::memory_order_relaxed); }

private:
    struct OverrideSnap
    {
        float x = 0.0f;
        float y = 0.0f;
        float headingDeg = 0.0f;
    };

    // Seqlock read: a torn read (a setter running at the same time) counts as "no override" for this call.
    bool ReadOverride(double aNow, OverrideSnap& aOut) const
    {
        if (!OverrideAllowed())
            return false;
        const uint32_t g1 = m_ovGen.load(std::memory_order_acquire);
        if (g1 & 1u)
            return false;
        const float x = m_ovX.load(std::memory_order_relaxed);
        const float y = m_ovY.load(std::memory_order_relaxed);
        const float h = m_ovHeading.load(std::memory_order_relaxed);
        const double until = m_ovUntil.load(std::memory_order_relaxed);
        if (m_ovGen.load(std::memory_order_acquire) != g1)
            return false;
        if (!(aNow < until))
            return false;
        aOut.x = x;
        aOut.y = y;
        aOut.headingDeg = h;
        return true;
    }

    bool RealFresh(int aIdx, double aNow) const
    {
        const double at = m_realAt[aIdx].load(std::memory_order_acquire);
        return at >= 0.0 && aNow >= at && (aNow - at) <= m_axisFreshSec.load(std::memory_order_relaxed);
    }

    bool Fresh(int aIdx, double aNow) const
    {
        const double at = m_rawAt[aIdx].load(std::memory_order_acquire);
        return at >= 0.0 && aNow >= at && (aNow - at) <= m_axisFreshSec.load(std::memory_order_relaxed);
    }

    std::atomic<bool> m_installed{false};
    std::atomic<bool> m_enabled{false};
    std::atomic<bool> m_killed{false};
    std::atomic<bool> m_hold{false};
    std::atomic<bool> m_tpp{false};
    std::atomic<uint32_t> m_gameBits{0};
    std::atomic<uint32_t> m_blockMask{kDefaultBlockMask};
    std::atomic<double> m_lastBeat{-1.0};
    std::atomic<double> m_staleSec{0.25};
    std::atomic<double> m_axisFreshSec{0.15};
    std::atomic<float> m_deadzone{0.15f};
    std::atomic<float> m_tppDist{0.6f};
    std::atomic<float> m_raw[2]{{0.0f}, {0.0f}};
    std::atomic<double> m_rawAt[2]{{-1.0}, {-1.0}};
    bool m_wasActive = false; // game thread only (Heartbeat)

    // hardware stick
    std::atomic<float> m_real[2]{{0.0f}, {0.0f}};
    std::atomic<double> m_realAt[2]{{-1.0}, {-1.0}};
    // move override (seqlock: odd generation = write in progress)
    std::atomic<uint32_t> m_ovGen{0};
    std::atomic<float> m_ovX{0.0f};
    std::atomic<float> m_ovY{0.0f};
    std::atomic<float> m_ovHeading{0.0f};
    std::atomic<double> m_ovUntil{-1.0};
    std::atomic<uint64_t> m_overrideSets{0};
    std::atomic<uint64_t> m_overrideAxisReads{0};

    std::atomic<uint64_t> m_axisReads{0};
    std::atomic<uint64_t> m_axisRewrites{0};
    std::atomic<uint64_t> m_headingReads{0};
    std::atomic<uint64_t> m_headingRewrites{0};
    std::atomic<uint64_t> m_beats{0};
    std::atomic<uint64_t> m_activations{0};
};

// ---- Hook-install decision (pure; inputs gathered by Locomotion.cpp) ----
enum class InstallResult : int
{
    Ok = 0,
    KillFile = -1,         // <dll dir>/utpp_locomotion.disable exists
    // -2 unused
    GameNotValidated = -3, // UTPP's 2.31 validation failed (plugin inert)
    NoClass = -4,          // gamestateMachineGameScriptInterface not in RTTI
    BadSignature = -5,     // function missing / not native / static / wrong return or params
    BadHandler = -6,       // handler null, outside the game image, not executable, or shared with another function
    AttachFailed = -7,     // handler-table slot swap failed (slot changed under us / not writable)
    Pending = -8,          // cheap checks passed at PostRegister; RTTI proof + attach deferred to the first Running
                           // frame (native class functions get their return/param types only when scripts load)
    BadTable = -9,         // member-native handler table / dispatcher layout not proven
};

struct FunctionFacts
{
    bool found = false;
    bool isNative = false;
    bool isStatic = false;
    bool returnsFloat = false;
    int paramCount = -1;
    bool firstParamIsCName = false;
    bool handlerInImage = false;
    bool handlerExecutable = false;
    bool handlerUnique = false; // no other function of the class shares the handler address
    bool hasReturnType = false; // returnType property present (null before the script runtime binds the native)
    bool firstParamByValue = false; // first parameter is neither 'out' nor 'opt'
    bool regOk = false; // CClassFunction::regIndex == GetRegIndex(), inside the table and below the registration count
};

struct InstallFacts
{
    bool killFile = false;
    bool gameValidated = false;
    bool classFound = false;
    FunctionFacts actionValue;   // GetActionValue(CName) -> Float
    FunctionFacts inputHeading;  // GetInputHeading() -> Float
    bool handlersDistinct = false;
    bool tableProven = false; // handler table resolved in the image, writable, and read by the engine's dispatcher
};

// ---- Member-native handler table (2.31 exe == retail exe; see Locomotion.cpp header) ----
// rtti::Function::InternalCallNative (address-library hash 0xA3961FF0) dispatches a non-static native as
//   call qword ptr [imageBase + regIndex*8 + tableRva]
// where tableRva is the address-library entry CBaseFunction_Handlers (0x5A7D28A9) itself - the ARRAY, not a pointer
// to it. rtti::NativeMemberFunction's constructor fills slot (count + 0xFF) and increments the uint32 counter that
// directly follows the 0x8000-entry array.
constexpr uint32_t kHandlerTableEntries = 0x8000;
constexpr uint32_t kFirstMemberReg = 0xFF;

// Plausible member-native register index: inside the array and already handed out by the registration counter.
inline bool RegPlausible(uint32_t aReg, uint32_t aCount)
{
    return aReg >= kFirstMemberReg && aReg < kHandlerTableEntries &&
           static_cast<uint64_t>(aReg) < static_cast<uint64_t>(aCount) + kFirstMemberReg;
}

struct DispatchScan
{
    int reads = 0;      // "call [rax+18h]; mov ecx,eax; lea rax,[imageBase]; mov r64,[rax+rcx*8+disp32]" sequences
    int tableReads = 0; // ... whose disp32 is the handler table
    uint32_t firstOtherDisp = 0;
};

// Pure scan of the dispatcher's code bytes. aCodeRva = RVA of aCode[0]; the lea must resolve to RVA 0 (image base).
inline DispatchScan ScanDispatcher(const uint8_t* aCode, size_t aLen, uint64_t aCodeRva, uint64_t aTableRva)
{
    DispatchScan r;
    if (!aCode)
        return r;
    // FF 50 18 | 8B C8 | 48 8D 05 rel32 | 48 8B modrm C8 disp32   (modrm: mod=10, rm=100 -> SIB; SIB C8 = rax+rcx*8)
    constexpr size_t kLen = 20;
    for (size_t i = 0; i + kLen <= aLen; ++i)
    {
        const uint8_t* c = aCode + i;
        if (c[0] != 0xFF || c[1] != 0x50 || c[2] != 0x18 || c[3] != 0x8B || c[4] != 0xC8 || c[5] != 0x48 ||
            c[6] != 0x8D || c[7] != 0x05 || c[12] != 0x48 || c[13] != 0x8B || (c[14] & 0xC7) != 0x84 || c[15] != 0xC8)
            continue;
        const int32_t rel = static_cast<int32_t>(static_cast<uint32_t>(c[8]) | (static_cast<uint32_t>(c[9]) << 8) |
                                                 (static_cast<uint32_t>(c[10]) << 16) |
                                                 (static_cast<uint32_t>(c[11]) << 24));
        const int64_t leaTarget = static_cast<int64_t>(aCodeRva + i + 12) + rel;
        if (leaTarget != 0)
            continue;
        const uint32_t disp = static_cast<uint32_t>(c[16]) | (static_cast<uint32_t>(c[17]) << 8) |
                              (static_cast<uint32_t>(c[18]) << 16) | (static_cast<uint32_t>(c[19]) << 24);
        ++r.reads;
        if (disp == aTableRva)
            ++r.tableReads;
        else if (!r.firstOtherDisp)
            r.firstOtherDisp = disp;
    }
    return r;
}

// Expected (vanilla 2.31 orphans.script, class StateGameScriptInterface):
//   public final native const func GetActionValue(actionName: CName) -> Float;
//   public final native const func GetInputHeading() -> Float;
inline bool SignatureOk(const FunctionFacts& aF, int aParams, bool aFirstCName)
{
    return aF.found && aF.isNative && !aF.isStatic && aF.hasReturnType && aF.returnsFloat && aF.paramCount == aParams &&
           (!aFirstCName || (aF.firstParamIsCName && aF.firstParamByValue));
}

// A native that exists with a handler but has no return type and no parameters yet: the script runtime has not bound
// its declaration (RTTI natives are registered by name + handler; types come from final.redscripts). Diagnostic only;
// it never relaxes SignatureOk.
inline bool LooksUnbound(const FunctionFacts& aF)
{
    return aF.found && aF.isNative && !aF.hasReturnType && aF.paramCount == 0;
}

inline bool HandlerOk(const FunctionFacts& aF)
{
    return aF.regOk && aF.handlerInImage && aF.handlerExecutable && aF.handlerUnique;
}

// PostRegister: only the decisions that need no script-bound RTTI. Pending = run DecideInstall on the first Running
// frame (after the script runtime is initialised).
inline InstallResult DecideEarly(const InstallFacts& aF)
{
    if (aF.killFile)
        return InstallResult::KillFile;
    if (!aF.gameValidated)
        return InstallResult::GameNotValidated;
    return InstallResult::Pending;
}

// Order matters: user settings first (no RTTI access needed), then RTTI proof.
inline InstallResult DecideInstall(const InstallFacts& aF)
{
    if (aF.killFile)
        return InstallResult::KillFile;
    if (!aF.gameValidated)
        return InstallResult::GameNotValidated;
    if (!aF.classFound)
        return InstallResult::NoClass;
    if (!SignatureOk(aF.actionValue, 1, true) || !SignatureOk(aF.inputHeading, 0, false))
        return InstallResult::BadSignature;
    if (!aF.tableProven)
        return InstallResult::BadTable;
    if (!HandlerOk(aF.actionValue) || !HandlerOk(aF.inputHeading) || !aF.handlersDistinct)
        return InstallResult::BadHandler;
    return InstallResult::Ok;
}

inline const char* InstallResultName(InstallResult aR)
{
    switch (aR)
    {
    case InstallResult::Ok: return "ok";
    case InstallResult::KillFile: return "kill file utpp_locomotion.disable present";
    case InstallResult::GameNotValidated: return "game not validated (UTPP inert)";
    case InstallResult::NoClass: return "gamestateMachineGameScriptInterface not found";
    case InstallResult::BadSignature: return "GetActionValue/GetInputHeading signature mismatch";
    case InstallResult::BadHandler: return "native handler not proven (reg/image/exec/unique)";
    case InstallResult::AttachFailed: return "handler table slot swap failed";
    case InstallResult::BadTable: return "member-native handler table / dispatcher not proven";
    case InstallResult::Pending: return "pending: RTTI proof deferred to the first Running frame (scripts bound)";
    }
    return "?";
}
} // namespace utpp::freemove
