#pragma once
// port (report 3.3 / 3.4): motion trajectory + state predicates from the game feed, UE units (cm, cm/s, deg).
// Sources (read 2026-10-13, UE 5.8 PoseSearchTrajectoryLibrary.cpp = "PSTL", motion pseudo = "ABP" / "CBP"):
// - ABP Update_Trajectory (l. 1740-1747): PoseSearchGenerateTransformTrajectory(Speed2D > 0 ? TrajectoryGenerationData_Moving :
//   _Idle, history interval -1 / 30 samples, prediction 0.1 s x 15), then HandleTransformTrajectoryWorldCollisions(bApplyGravity
//   true, FloorCollisionsOffset 0.01, MaxObstacleHeight 150) whose result is stored back into Trajectory (l. 1744); velocities
//   Trj_PastVelocity [-0.3,-0.2], Trj_CurrentVelocity [0,0.2], Trj_FutureVelocity [0.4,0.5] s, bExtrapolate false.
// - ABP l. 31-32 TrajectoryGenerationData_Idle / _Moving: MaxControllerYawRate 100 / 0, RotateTowardsMovementSpeed 0 / 0,
//   BendVelocityTowardsAcceleration 0, no speed / acceleration remapping curves.
// - PSTL UpdateData (l. 42-122), StepCharacterMovementGroundPrediction (l. 125-193), InitTrajectorySamples (l. 195-225),
//   UpdateHistory_TransformHistoryInternal (l. 263-338), UpdatePrediction_SimulateCharacterMovement* (l. 400-520),
//   PoseSearchGenerateTransformTrajectory (l. 666-706), HandleTransformTrajectoryWorldCollisions* (l. 851-974),
//   GetTransformTrajectoryVelocity (l. 982-996); TrajectoryTypes.cpp GetSampleAtTime (l. 40-64) / Sample Lerp (l. 12-22).
// - CBP CMC CDO (l. 755-777): GroundFriction 5, MaxAcceleration 800, MinAnalogWalkSpeed 150, BrakingFrictionFactor 1,
//   bUseSeparateBrakingFriction True, BrakingFriction NOT set -> UE CMC constructor (CharacterMovementComponent.cpp l. 653-788
//   assigns BrakingFrictionFactor l. 734 and bUseSeparateBrakingFriction l. 788 but never BrakingFriction; UObject memory is
//   zero-initialised) -> 0. CBP CalculateBrakingFriction (l. 1197-1199, 3 / 0) is defined but never called anywhere in the
//   pseudo (grep: only its definition); UpdateMovement_PreCMC (l. 1130-1140) sets MaxAcceleration, BrakingDecelerationWalking
//   (500 with input / 2000 without), GroundFriction (Sprint 5..3 over 0..500), MaxWalkSpeed (CalculateMaxSpeed) and
//   MaxWalkSpeedCrouched only. => braking friction without acceleration = 0 * 1 = 0 (PSTL l. 88-92).
// - CBP CalculateMaxSpeed (l. 1205-1209) / CalculateMaxCrouchSpeed (l. 1369-1370): StrafeSpeedMap = Curve_StrafeSpeedMap(|
//   CalculateDirection(Velocity, ActorRotation)|) when strafing (bUseControllerDesiredRotation / !bOrientRotationToMovement),
//   else 0; speed = MapRangeClamped(map, 0..1, X..Y) for map < 1, else MapRangeClamped(map, 1..2, Y..Z); vectors CBP l. 10-13
//   Walk 200/180/150, Run 500/350/300, Sprint 700/700/700, Crouch 225/200/180. Curve keys from
//   GASPALS__Blueprints__Data__Curve_StrafeSpeedMap.t3d: (0,0) (45,0) (80,1) (100,1) (135,2) (180,2),
//   linear (RichCurve.h l. 121 FRichCurveKey default RCIM_Linear; the export lists no InterpMode).
// - ABP predicates IsMoving / IsStarting / IsPivoting / Get_TrajectoryTurnAngle / ShouldTurnInPlace / ShouldSpinTransition /
//   JustLanded_Light / _Heavy (HeavyLandSpeedThreshold 700) / PlayLand / PlayMovingLand (pseudo l. 1777-1937, 2180, 2231, 2274).
// Game feed units: m/s (velWorld, z up). Horizontal velocity -> cm/s with 100 * speedScale (SpeedCalibration of the active
// database), vertical velocity -> cm/s with 100 only (gravity / land thresholds are physical, not database speeds).
// Yaw convention of this header: YawOf(v) = atan2(y, x) in degrees for every yaw (facing, actor, control); a game yaw psi with
// forward (-sin psi, cos psi) converts with GameYawToTrajYaw (+90). Facing is the actor (mesh) forward yaw: the constant mesh
// relative rotation (-90 in motion) of FTransformTrajectorySample::Facing is dropped [P] (a constant offset of every facing;
// the query builder compares facings relative to the current one).

#include <algorithm>
#include <cmath>
#include <cstdint>
#include "CapsuleBraking.hpp"
#include "CapsuleAcceleration.hpp"

namespace utpp::mm::motiontraj
{
inline constexpr float kPi = 3.14159265358979f;
inline constexpr float kRad2Deg = 180.0f / kPi;

// ---------------------------------------------------------------- motion / UE constants (cm, s)
inline constexpr int kHistoryCount = 30;          // InTrajectoryHistoryCount (interval -1 = every update)
inline constexpr int kPredictionCount = 15;       // InTrajectoryPredictionCount
inline constexpr float kPredictionInterval = 0.1f; // InPredictionSamplingInterval
inline constexpr float kMovingTolerance = 10.0f;  // IsMoving: Trj_FutureVelocity != 0 with tolerance 10 per component
inline constexpr float kStartingMargin = 100.0f;  // IsStarting: |Future|xy >= |Velocity|xy + 100
inline constexpr float kPivotOrient = 45.0f;      // IsPivoting OrientToMovement
inline constexpr float kPivotStrafe = 30.0f;      // IsPivoting Strafe
inline constexpr float kTurnInPlaceDeg = 50.0f;   // ShouldTurnInPlace
inline constexpr float kSpinDeg = 130.0f;         // ShouldSpinTransition
inline constexpr float kSpinMinSpeed = 150.0f;
inline constexpr float kHeavyLand = 700.0f;       // HeavyLandSpeedThreshold
inline constexpr float kJustLandedSeconds = 0.3f; // CBP OnLanded RetriggerableDelay
inline constexpr float kMovingLandMaxTurn = 120.0f;
inline constexpr float kHasVelocity = 5.0f;       // Update_EssentialValues HasVelocity
inline constexpr float kMinTickTime = 1.0e-6f;    // UCharacterMovementComponent::MIN_TICK_TIME
inline constexpr float kBrakeToStop = 10.0f;      // UCharacterMovementComponent::BRAKE_TO_STOP_VELOCITY
inline constexpr float kKindaSmall = 1.0e-4f;     // KINDA_SMALL_NUMBER
inline constexpr float kSmall = 1.0e-8f;          // UE_SMALL_NUMBER
inline constexpr float kBrakingSubStep = 1.0f / 33.0f; // UCharacterMovementComponent::BrakingSubStepTime default
inline constexpr float kFloorCollisionsOffset = 0.01f; // ABP l. 1742 HandleTransformTrajectoryWorldCollisions FloorCollisionsOffset
inline constexpr float kMaxObstacleHeight = 150.0f;    // ABP l. 1742 MaxObstacleHeight (trace from sample + 150 down to sample)
inline constexpr float kFiniteDelta = 1.0f / 60.0f;    // PoseSearchDefines.h l. 43 FiniteDelta (query velocity difference)

enum class RotationMode : uint8_t { OrientToMovement = 0, Strafe = 1 };

struct V3
{
    float x = 0, y = 0, z = 0;
};
inline V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline V3 operator*(V3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline float Dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline float Size(V3 a) { return std::sqrt(Dot(a, a)); }
inline float SizeXY(V3 a) { return std::sqrt(a.x * a.x + a.y * a.y); }
inline bool IsZero(V3 a) { return a.x == 0.0f && a.y == 0.0f && a.z == 0.0f; }
inline V3 SafeNormal(V3 a)
{
    const float s2 = Dot(a, a);
    if (s2 <= 1.0e-8f) // UE_SMALL_NUMBER
        return {};
    return a * (1.0f / std::sqrt(s2));
}
inline V3 ClampMaxSize(V3 a, float aMax)
{
    if (aMax < kKindaSmall)
        return {};
    const float s2 = Dot(a, a);
    if (s2 > aMax * aMax)
        return a * (aMax / std::sqrt(s2));
    return a;
}
// FVector::Equals(Zero, Tolerance): every component within tolerance (IsMoving "!= Vector() ~tol=10").
inline bool NearlyZeroPerComponent(V3 a, float aTol) { return std::fabs(a.x) <= aTol && std::fabs(a.y) <= aTol && std::fabs(a.z) <= aTol; }

// FRotator::NormalizeAxis: (-180, 180]
inline float NormalizeAxis(float d)
{
    d = std::fmod(d, 360.0f);
    if (d < 0.0f)
        d += 360.0f;
    if (d > 180.0f)
        d -= 360.0f;
    return d;
}
// Conv_VectorToRotator(v).Yaw = atan2(y, x) in degrees (0 for the zero vector).
inline float YawOf(V3 v) { return (v.x == 0.0f && v.y == 0.0f) ? 0.0f : std::atan2(v.y, v.x) * kRad2Deg; }
// Get_TrajectoryTurnAngle = NormalizedDeltaRotator(rot(Trj_FutureVelocity), rot(Velocity)).Yaw
inline float TurnAngle(V3 aFuture, V3 aVelocity) { return NormalizeAxis(YawOf(aFuture) - YawOf(aVelocity)); }
// Game yaw psi (forward = (-sin psi, cos psi), MotionAdapter convention) -> YawOf convention of this header.
inline float GameYawToTrajYaw(float aGameYawDeg) { return NormalizeAxis(aGameYawDeg + 90.0f); }
// FQuat::MakeFromEuler(0, 0, yaw) * v for a z rotation (x toward y for a positive yaw).
inline V3 RotateYaw(V3 v, float aYawDeg)
{
    const float r = aYawDeg / kRad2Deg, c = std::cos(r), s = std::sin(r);
    return {v.x * c - v.y * s, v.x * s + v.y * c, v.z};
}

inline float MapRangeClamped(float v, float a, float b, float oa, float ob)
{
    const float t = std::clamp((v - a) / (b - a), 0.0f, 1.0f);
    return oa + (ob - oa) * t;
}

// ---------------------------------------------------------------- unit conversion (game m/s -> UE cm/s)
inline V3 GameVelocityToCm(const float aVelWorld[3], float aSpeedScale)
{
    return {aVelWorld[0] * 100.0f * aSpeedScale, aVelWorld[1] * 100.0f * aSpeedScale, aVelWorld[2] * 100.0f};
}

// ---------------------------------------------------------------- CMC-derived prediction inputs (UpdateData)
struct Derived
{
    float maxSpeed = 500.0f;       // max(MaxSpeed * AnalogInputModifier, MinAnalogSpeed)
    float brakingDeceleration = 0; // 500 with input / 2000 without (motion CalculateBrakingDeceleration)
    float friction = 5.0f;         // GroundFriction with acceleration, BrakingFriction * factor without
    float brakingSubStep = kBrakingSubStep;
    bool orientToMovement = false;
    bool stepGround = true;        // !IsFalling && !IsFlying
    brake::Prediction capsuleStop;
    accel::Prediction capsuleStart;
};

// motion CBP gait parameters (cm/s). walkSpeed / runSpeed / sprintSpeed / crouchSpeed are the forward (X) columns of
// WalkSpeeds / RunSpeeds / SprintSpeeds / CrouchSpeeds (CBP l. 10-13); *Strafe = Y (map 1), *Back = Z (map 2).
struct MotionCmc
{
    float walkSpeed = 200.0f, runSpeed = 500.0f, sprintSpeed = 700.0f, crouchSpeed = 225.0f;
    float walkSpeedStrafe = 180.0f, runSpeedStrafe = 350.0f, sprintSpeedStrafe = 700.0f, crouchSpeedStrafe = 200.0f;
    float walkSpeedBack = 150.0f, runSpeedBack = 300.0f, sprintSpeedBack = 700.0f, crouchSpeedBack = 180.0f;
    float minAnalogSpeed = 150.0f;
    float groundFriction = 5.0f;
    float brakingFriction = 0.0f;          // CDO does not set it, UE constructor default 0 (see header comment)
    float brakingFrictionFactor = 1.0f;    // CDO l. 761
    bool useSeparateBrakingFriction = true; // CDO l. 771
    float brakingDecelInput = 500.0f, brakingDecelNoInput = 2000.0f;
    float maxAcceleration = 800.0f;
};

// gait 0 Walk / 1 Run / 2 Sprint; stance 1 Crouch. aStick = CMC AnalogInputModifier = |Acceleration| / MaxAcceleration
// (1 with input in the motion FixedSpeed stick modes: GetMovementInputScaleValue = Normal2D, CBP l. 1119).
inline float MotionMaxAcceleration(const MotionCmc& c, int aGait, float aSpeed2D)
{
    return aGait == 2 ? MapRangeClamped(aSpeed2D, 300.0f, 700.0f, 800.0f, 300.0f) : c.maxAcceleration;
}
inline float MotionGroundFriction(const MotionCmc& c, int aGait, float aSpeed2D)
{
    return aGait == 2 ? MapRangeClamped(aSpeed2D, 0.0f, 500.0f, 5.0f, 3.0f) : c.groundFriction;
}
// KismetAnimationLibrary.cpp l. 276-301 CalculateDirection(Velocity, BaseRotation) with a yaw-only base rotation (YawOf convention):
// signed angle of the 2D velocity from the actor forward, 0 for a nearly zero velocity.
inline float CalculateDirection(V3 aVel, float aActorYawDeg)
{
    if (NearlyZeroPerComponent(aVel, kKindaSmall)) // FVector::IsNearlyZero (KINDA_SMALL_NUMBER)
        return 0.0f;
    const float r = aActorYawDeg / kRad2Deg;
    const V3 fwd{std::cos(r), std::sin(r), 0.0f}, right{-std::sin(r), std::cos(r), 0.0f};
    const float s2 = aVel.x * aVel.x + aVel.y * aVel.y; // GetSafeNormal2D
    const V3 n = s2 < kSmall ? V3{} : V3{aVel.x / std::sqrt(s2), aVel.y / std::sqrt(s2), 0.0f};
    float deg = std::acos(std::clamp(Dot(fwd, n), -1.0f, 1.0f)) * kRad2Deg; // FMath::Acos clamps
    if (Dot(right, n) < 0.0f)
        deg = -deg;
    return deg;
}
// Curve_StrafeSpeedMap (t3d keys, linear, constant extrapolation).
inline float StrafeSpeedMap(float aAbsDirectionDeg)
{
    static constexpr float kT[6] = {0.0f, 45.0f, 80.0f, 100.0f, 135.0f, 180.0f};
    static constexpr float kV[6] = {0.0f, 0.0f, 1.0f, 1.0f, 2.0f, 2.0f};
    if (aAbsDirectionDeg <= kT[0])
        return kV[0];
    for (int i = 0; i + 1 < 6; ++i)
        if (aAbsDirectionDeg <= kT[i + 1])
            return kV[i] + (kV[i + 1] - kV[i]) * (aAbsDirectionDeg - kT[i]) / (kT[i + 1] - kT[i]);
    return kV[5];
}
// CBP CalculateMaxSpeed / CalculateMaxCrouchSpeed: aStrafe = bUseControllerDesiredRotation (= !bOrientRotationToMovement,
// UpdateRotation_PreCMC l. 1146-1151); aDirectionDeg = CalculateDirection(Velocity, ActorRotation).
inline float MotionMaxSpeed(const MotionCmc& c, int aGait, int aStance, bool aStrafe, float aDirectionDeg)
{
    const float map = aStrafe ? StrafeSpeedMap(std::fabs(aDirectionDeg)) : 0.0f;
    float x, y, z;
    if (aStance == 1)
        x = c.crouchSpeed, y = c.crouchSpeedStrafe, z = c.crouchSpeedBack;
    else if (aGait == 2)
        x = c.sprintSpeed, y = c.sprintSpeedStrafe, z = c.sprintSpeedBack;
    else if (aGait == 1)
        x = c.runSpeed, y = c.runSpeedStrafe, z = c.runSpeedBack;
    else
        x = c.walkSpeed, y = c.walkSpeedStrafe, z = c.walkSpeedBack;
    return map < 1.0f ? MapRangeClamped(map, 0.0f, 1.0f, x, y) : MapRangeClamped(map, 1.0f, 2.0f, y, z);
}
// aDirectionDeg (defaulted 0 = forward column) = CalculateDirection(Velocity, ActorRotation); used only when strafing (!aOrient).
inline Derived MakeDerived(const MotionCmc& c, int aGait, int aStance, float aStick, float aSpeed2D, V3 aAccel, bool aOrient,
                           bool aFalling, float aDirectionDeg = 0.0f)
{
    Derived d;
    const float maxSpeed = MotionMaxSpeed(c, aGait, aStance, !aOrient, aDirectionDeg);
    d.maxSpeed = std::max(maxSpeed * std::clamp(aStick, 0.0f, 1.0f), c.minAnalogSpeed);
    const bool input = aStick > 0.0f;
    d.brakingDeceleration = std::max(0.0f, input ? c.brakingDecelInput : c.brakingDecelNoInput);
    if (IsZero(aAccel)) // PSTL l. 88-92
    {
        const float f = c.useSeparateBrakingFriction ? c.brakingFriction : MotionGroundFriction(c, aGait, aSpeed2D);
        d.friction = std::max(0.0f, f * std::max(0.0f, c.brakingFrictionFactor));
    }
    else
        d.friction = MotionGroundFriction(c, aGait, aSpeed2D);
    d.orientToMovement = aOrient;
    d.stepGround = !aFalling;
    return d;
}

// UE StepCharacterMovementGroundPrediction (PoseSearchTrajectoryLibrary.cpp:125-193).
inline V3 StepGround(float aDt, V3 aVel, V3 aAcc, const Derived& d)
{
    V3 out = aVel;
    if (IsZero(aAcc))
    {
        if (IsZero(aVel))
            return {};
        const bool zeroFriction = d.friction == 0.0f;
        const bool zeroBraking = d.brakingDeceleration == 0.0f;
        if (zeroFriction && zeroBraking)
            return aVel;
        float remaining = aDt;
        const float maxStep = std::clamp(d.brakingSubStep, 1.0f / 75.0f, 1.0f / 20.0f);
        const V3 prev = out;
        const V3 rev = zeroBraking ? V3{} : SafeNormal(out) * (-d.brakingDeceleration);
        while (remaining >= kMinTickTime)
        {
            const float dt = (remaining > maxStep && !zeroFriction) ? std::min(maxStep, remaining * 0.5f) : remaining;
            remaining -= dt;
            out = out + (out * (-d.friction) + rev) * dt;
            if (Dot(out, prev) <= 0.0f)
                return {};
        }
        const float s2 = Dot(out, out);
        if (s2 <= kKindaSmall || (!zeroBraking && s2 <= kBrakeToStop * kBrakeToStop))
            out = {};
    }
    else
    {
        const V3 dir = SafeNormal(aAcc);
        const float vs = Size(out);
        out = out - (out - dir * vs) * std::min(aDt * d.friction, 1.0f);
        out = out + aAcc * aDt;
        out = ClampMaxSize(out, d.maxSpeed);
    }
    return out;
}

// ---------------------------------------------------------------- trajectory generation data (ABP l. 31-32)
struct GenerationData
{
    float maxControllerYawRate = 0.0f;       // deg/s, < 0 = unclamped (PSTL l. 104-107)
    float rotateTowardsMovementSpeed = 0.0f; // QInterpConstantTo speed; <= 0 jumps to the target (UnrealMath.cpp l. 2727-2731)
};
inline constexpr GenerationData kGenIdle{100.0f, 0.0f};
inline constexpr GenerationData kGenMoving{0.0f, 0.0f};
// ABP l. 1741: Speed2D > 0 ? _Moving : _Idle. Update_Trajectory runs before Update_EssentialValues (Update_Logic l. 2264-2266),
// so the caller passes the Speed2D of the previous frame.
inline GenerationData SelectGenerationData(float aSpeed2DLastFrame) { return aSpeed2DLastFrame > 0.0f ? kGenMoving : kGenIdle; }

// Facing inputs of one update (YawOf convention). valid false = legacy callers: no controller rotation, facing 0.
struct FacingInput
{
    bool valid = false;
    float actorYawDeg = 0.0f;   // mesh component rotation (GVarPoseSearchTrajectoryFacingFromMesh = true, PSTL l. 21, 113-116)
    float controlYawDeg = 0.0f; // Character->GetViewRotation().Yaw (PSTL l. 95)
    GenerationData gen = kGenMoving;
    bool aimTurnInPlace = false;
    bool shotTurn = false;
};

// QInterpConstantTo for yaw-only rotations (UnrealMath.cpp l. 2725-2743): speed <= 0 -> target; else Slerp by
// Alpha = clamp(clamp(dt * speed, 0, 1) / AngularDistance, 0, 1).
inline float YawInterpConstantTo(float aCur, float aTarget, float aDt, float aSpeed)
{
    if (aSpeed <= 0.0f)
        return aTarget;
    const float diff = NormalizeAxis(aTarget - aCur);
    if (std::fabs(diff) < 1.0e-4f)
        return aTarget;
    const float step = std::clamp(aDt * aSpeed, 0.0f, 1.0f);
    const float dist = std::max(kSmall, std::fabs(diff) / kRad2Deg);
    const float alpha = std::clamp(step / dist, 0.0f, 1.0f);
    return NormalizeAxis(aCur + diff * alpha);
}

// FTransformTrajectorySample::Lerp facing (TrajectoryTypes.cpp l. 19): FQuat::FastLerp (Quat.h l. 1371-1377) + GetNormalized for
// yaw-only quaternions (w = cos(yaw/2), z = sin(yaw/2)).
inline float FacingLerp(float aA, float aB, float aAlpha)
{
    const float ha = aA / kRad2Deg * 0.5f, hb = aB / kRad2Deg * 0.5f;
    const float aw = std::cos(ha), az = std::sin(ha), bw = std::cos(hb), bz = std::sin(hb);
    const float bias = (aw * bw + az * bz) >= 0.0f ? 1.0f : -1.0f;
    const float w = bw * aAlpha + aw * bias * (1.0f - aAlpha), z = bz * aAlpha + az * bias * (1.0f - aAlpha);
    return NormalizeAxis(2.0f * std::atan2(z, w) * kRad2Deg);
}

// ---------------------------------------------------------------- trajectory
struct Sample
{
    V3 pos;
    float t = 0.0f;
    float facing = 0.0f; // yaw (deg, YawOf convention)
};

class Trajectory
{
public:
    static constexpr int kNum = kHistoryCount + 1 + kPredictionCount;

    // ABP Trajectory / PreviousDesiredControllerYaw defaults (empty samples -> InitTrajectorySamples on the next update, yaw 0).
    void Reset()
    {
        m_init = false;
        m_pos = {};
        m_ctrlYawLast = 0.0f;
        m_ctrlYawRate = 0.0f;
        m_capsuleAcceleration = 0.0f;
    }

    // Legacy overload (no facing input): controller yaw rate 0, facing 0.
    void Update(float aDt, V3 aVel, V3 aAcc, const Derived& d, float aGravityCm = 0.0f, bool aHasFloor = false,
                float aFloorZ = 0.0f)
    {
        Update(aDt, aVel, aAcc, d, aGravityCm, aHasFloor, aFloorZ, FacingInput{});
    }

    // One ABP Update_Trajectory.
    // aVel = CMC Velocity (cm/s). On the ground (d.stepGround) its z is dropped like CMC walking does
    // (CharacterMovementComponent.cpp l. 5635-5650 MaintainHorizontalGroundVelocity, bMaintainHorizontalGroundVelocity default
    // true, called from PhysWalking l. 5708 ff.); UE's ground delta (PSTL l. 282-284) then folds slope climbing into every history
    // sample, which equals integrating the horizontal velocity here.
    // aGravityCm > 0 = HandleTransformTrajectoryWorldCollisionsWithGravity (bApplyGravity true, motion always); <= 0 = no world
    // collision pass [P] (unit tests only; UE world gravity is never 0 in motion).
    // Floor [P]: the game gives no per-sample traces; a flat floor at aFloorZ relative to the current position (UTPPFeedAir
    // -distanceToGround) when aHasFloor, else at the current position while on the ground, else none (no hit in the air).
    void Update(float aDt, V3 aVel, V3 aAcc, const Derived& d, float aGravityCm, bool aHasFloor, float aFloorZ,
                const FacingInput& aFacing)
    {
        if (!(aDt > 0.0f) || !std::isfinite(aDt))
            aDt = 0.0f;
        V3 vel = aVel;
        if (d.stepGround)
            vel.z = 0.0f;

        // UpdateData (PSTL l. 95-108): ControllerYawRate from the view yaw, clamped by MaxControllerYawRate.
        m_ctrlYawRate = 0.0f;
        if (aFacing.valid)
        {
            const float delta = aFacing.controlYawDeg - m_ctrlYawLast;
            m_ctrlYawLast = aFacing.controlYawDeg;
            if (aDt > kSmall)
            {
                m_ctrlYawRate = NormalizeAxis(delta) / aDt;
                if (aFacing.gen.maxControllerYawRate >= 0.0f)
                    m_ctrlYawRate = (m_ctrlYawRate > 0.0f ? 1.0f : (m_ctrlYawRate < 0.0f ? -1.0f : 0.0f)) *
                                    std::min(std::fabs(m_ctrlYawRate), aFacing.gen.maxControllerYawRate);
            }
        }
        const float facing0 = aFacing.valid ? NormalizeAxis(aFacing.actorYawDeg) : 0.0f;

        // Position (UpdateData l. 110): integrated capsule. InitTrajectorySamples (l. 195-225) on the first update: history at the
        // current position with time max(-1, 0) * (i - 31) = 0, current sample at DeltaTime.
        if (!m_init)
        {
            for (int i = 0; i < kHistoryCount; ++i)
                m_hist[i] = Sample{m_pos, 0.0f, facing0};
            m_pred[0] = Sample{m_pos, aDt, facing0};
            m_init = true;
        }
        else
            m_pos = m_pos + vel * aDt;

        // UpdateHistory_TransformHistoryInternal (l. 263-338), interval -1 -> record every update: TimeShift = last frame's
        // current sample time (its DeltaTime), older samples shift by it, last frame's current sample (after the collision pass,
        // ABP l. 1744 stores the collided trajectory) becomes the record at t = 0.
        // [P] ground delta not applied. UE (l. 281-284, 297-317) shifts the record and every older history sample by
        // InferredGroundTranslation = CurrentPosition - Samples[NumHistorySamples].Position - v dt; Samples[NumHistorySamples] is
        // last frame's COLLIDED current sample, so the delta is minus that frame's collision z offset (ground: -0.01 cm, the
        // FloorCollisionsOffset; air without a floor hit: +0.5 g FreeFall^2, ~0.14 cm at 60 Hz) and UE's record equals the
        // un-collided position. Here the collided sample is stored as is. Reason: z-only, < 1 cm, PSS features read PosXY;
        // it shifts GetSampleAtTime(0).z (the TimeToLand FallHeight origin) by that offset. Open: model it exactly.
        const float shift = m_pred[0].t;
        for (int i = 0; i + 1 < kHistoryCount; ++i)
        {
            m_hist[i] = m_hist[i + 1];
            m_hist[i].t = m_hist[i + 1].t - shift;
        }
        m_hist[kHistoryCount - 1] = Sample{m_pred[0].pos, 0.0f, m_pred[0].facing};

        // UpdatePrediction_SimulateCharacterMovement (l. 474-520) + Step (l. 440-472): AccumulatedSeconds starts at DeltaTime.
        V3 p = m_pos, v = vel, a = aAcc;
        const bool capsuleStop = d.stepGround && IsZero(aAcc) && d.capsuleStop.Active();
        const V3 stopDirection = SafeNormal(vel);
        float f = facing0, t = aDt;
        const float rotStep = m_ctrlYawRate * kPredictionInterval; // ControllerRotationPerStep (l. 451)
        // Scalar physical acceleration is known only for an aligned straight
        // start. A rotating prediction still uses the existing vector/friction model.
        const bool capsuleStart = d.stepGround && !IsZero(aAcc) && d.capsuleStart.Active() &&
            std::fabs(rotStep) <= kKindaSmall;
        m_capsuleAcceleration = capsuleStart ? d.capsuleStart.acceleration : 0.f;
        const V3 startDirection = SafeNormal(aAcc);
        for (int k = 0; k <= kPredictionCount; ++k)
        {
            m_pred[k] = Sample{p, t, f};
            if (k == kPredictionCount)
                break;
            p = p + (capsuleStop ? stopDirection * d.capsuleStop.DistanceBetween(
                float(k) * kPredictionInterval, float(k + 1) * kPredictionInterval) :
                capsuleStart ? startDirection * d.capsuleStart.DistanceBetween(
                    float(k) * kPredictionInterval, float(k + 1) * kPredictionInterval) : v * kPredictionInterval);
            t += kPredictionInterval;
            if (d.stepGround)
            {
                a = RotateYaw(a, rotStep);
                v = capsuleStop ? stopDirection * d.capsuleStop.QuerySpeedAt(float(k + 1) * kPredictionInterval) :
                    capsuleStart ? startDirection * d.capsuleStart.QuerySpeedAt(float(k + 1) * kPredictionInterval) :
                    StepGround(kPredictionInterval, v, a, d);
                f = NormalizeAxis(f + rotStep);
                if (d.orientToMovement && !NearlyZeroPerComponent(a, kKindaSmall))
                    f = YawInterpConstantTo(f, YawOf(a), kPredictionInterval, aFacing.gen.rotateTowardsMovementSpeed);
                if (aFacing.aimTurnInPlace)
                    // This is a held facing goal, not measured camera velocity.
                    // Starting every prediction from the visible root and then
                    // limiting it to 100deg/s asks for only a 50deg TIP at +.5s,
                    // even when the live view is 170deg behind the body.
                    f = aFacing.controlYawDeg;
            }
        }

        // HandleTransformTrajectoryWorldCollisionsWithGravity (l. 876-972), bApplyGravity true.
        m_ttl = m_pred[kPredictionCount].t; // l. 900: no hit -> time of the last sample
        m_landHit = false;
        m_v0 = vel.z; // StartingVelocity = Character velocity (l. 866); InitialVelocityZ = its gravity-axis component (l. 889-897)
        m_gravity = aGravityCm > 0.0f ? aGravityCm : 0.0f;
        if (aGravityCm > 0.0f)
        {
            const bool floor = aHasFloor || d.stepGround;
            const float floorZ = m_pos.z + (aHasFloor ? aFloorZ : 0.0f);
            bool lastImpact = false;
            float freeFall = 0.0f;
            float prevT = m_hist[kHistoryCount - 1].t, prevZ = m_hist[kHistoryCount - 1].pos.z;
            for (int k = 0; k <= kPredictionCount; ++k) // history samples have t <= 0 and are skipped (l. 914)
            {
                Sample& s = m_pred[k];
                if (s.t > 0.0f)
                {
                    freeFall += s.t - prevT;
                    if (lastImpact)
                        s.pos.z = prevZ; // FPlane::PointPlaneProject onto the plane through PrevSample, normal up (l. 920-925)
                    s.pos.z -= 0.5f * aGravityCm * freeFall * freeFall;
                    // LineTraceSingle(sample + 150 up, sample) against the flat floor (l. 932)
                    if (floor && s.pos.z <= floorZ && floorZ <= s.pos.z + kMaxObstacleHeight)
                    {
                        lastImpact = true;
                        s.pos.z = floorZ + kFloorCollisionsOffset; // l. 939
                        if (!m_landHit)
                        {
                            // l. 941-949: FallHeight = |sample z - GetSampleAtTime(0) z| (trajectory already partly collided)
                            m_landHit = true;
                            const float fallHeight = std::fabs(s.pos.z - PositionAt(0.0f).z);
                            m_ttl = m_v0 / aGravityCm + std::sqrt(m_v0 * m_v0 + 2.0f * aGravityCm * fallHeight) / aGravityCm;
                        }
                        freeFall = 0.0f; // l. 952
                    }
                }
                prevT = s.t;
                prevZ = s.pos.z;
            }
        }
    }

    // FTransformTrajectory::GetSampleAtTime(bExtrapolate = false) (TrajectoryTypes.cpp l. 40-64): lower bound, alpha clamped.
    Sample SampleAt(float aT) const
    {
        const Sample* all[kNum];
        int n = 0;
        for (int i = 0; i < kHistoryCount; ++i)
            all[n++] = &m_hist[i];
        for (int k = 0; k <= kPredictionCount; ++k)
            all[n++] = &m_pred[k];
        int lb = 0;
        while (lb < n && aT > all[lb]->t)
            ++lb;
        const int next = std::clamp(lb, 1, n - 1), prev = next - 1;
        const float den = all[next]->t - all[prev]->t;
        if (std::fabs(den) <= kSmall)
            return *all[prev];
        const float u = std::clamp((aT - all[prev]->t) / den, 0.0f, 1.0f);
        Sample r;
        r.pos = all[prev]->pos + (all[next]->pos - all[prev]->pos) * u;
        r.t = all[prev]->t + (all[next]->t - all[prev]->t) * u;
        r.facing = FacingLerp(all[prev]->facing, all[next]->facing, u);
        return r;
    }
    V3 PositionAt(float aT) const { return SampleAt(aT).pos; }
    float FacingYawAt(float aT) const { return SampleAt(aT).facing; }
    // GetTransformTrajectoryVelocity (l. 982-996)
    V3 VelocityBetween(float aT1, float aT2) const
    {
        if (std::fabs(aT2 - aT1) <= kSmall) // FMath::IsNearlyEqual default UE_SMALL_NUMBER
            return {};
        return (PositionAt(aT2) - PositionAt(aT1)) * (1.0f / (aT2 - aT1));
    }
    // PoseSearch query velocity at a schema sample time (PoseSearchContext.cpp l. 1015-1019): (Pos(t) - Pos(t - FiniteDelta)) /
    // FiniteDelta, FiniteDelta = 1/60 (PoseSearchDefines.h l. 43).
    V3 VelocityAt(float aT) const { return (PositionAt(aT) - PositionAt(aT - kFiniteDelta)) * (1.0f / kFiniteDelta); }
    V3 PastVelocity() const { return VelocityBetween(-0.3f, -0.2f); }
    V3 CurrentVelocity() const { return VelocityBetween(0.0f, 0.2f); }
    V3 FutureVelocity() const { return VelocityBetween(0.4f, 0.5f); }
    float TimeToLand() const { return m_ttl; }
    // CollisionResult.LandSpeed = InitialVelocityZ + GravityZ * TimeToLand (l. 973), GravityZ = -g
    float LandSpeed() const { return m_v0 - m_gravity * m_ttl; }
    bool LandPredicted() const { return m_landHit; }
    float ControllerYawRate() const { return m_ctrlYawRate; }
    float CapsuleAcceleration() const { return m_capsuleAcceleration; }
    int NumHistory() const { return m_init ? kHistoryCount : 0; }
    const Sample& History(int i) const { return m_hist[i]; }
    const Sample& Prediction(int k) const { return m_pred[k]; }

private:
    Sample m_hist[kHistoryCount];
    Sample m_pred[kPredictionCount + 1];
    bool m_init = false;
    V3 m_pos;
    float m_ctrlYawLast = 0.0f; // ABP PreviousDesiredControllerYaw
    float m_ctrlYawRate = 0.0f;
    float m_capsuleAcceleration = 0.0f;
    float m_ttl = kPredictionCount * kPredictionInterval;
    float m_v0 = 0.0f, m_gravity = 0.0f;
    bool m_landHit = false;
};

// ---------------------------------------------------------------- predicates (ABP functions, exact)
// IsMoving: Trj_FutureVelocity != 0 (tolerance 10 per component) && Acceleration != 0
inline bool IsMoving(V3 aFuture, V3 aAccel) { return !NearlyZeroPerComponent(aFuture, kMovingTolerance) && !IsZero(aAccel); }
// IsStarting: IsMoving && |Future|xy >= |Velocity|xy + 100 && current database tags without "Pivots"
// [P] slow targets (2026-10-31 loco diagnostic, findings CR-2 / HIP-3 / GW-2, tests/LocoStartTest.cpp): motion's walk targets are >= 150 cm/s
// (MinAnalogWalkSpeed 150), the game's slow ones are not - MinAnalogSpeed is 0 here (MotionGameCmc: a 150 cm/s floor predicted an acceleration
// the capsule never performs) and the speed map puts the crouch analog walk at ~48 cm/s, the minimum analog walk floor at ~80-100 cm/s. A
// future below kStartingMargin can never be 100 cm/s above any velocity, so such a start was never IsStarting: the first pick came from
// the Loops or (with the IsPivoting below) the Pivots databases, never from the *_Starts ones. Below kStartingMargin the margin is half the
// future speed (the capsule is still below half its target: a start from rest, not a steady slow walk where |Velocity| ~= |Future|).
// |Future|xy >= kStartingMargin is the motion predicate, bit for bit.
inline constexpr float kStartingSlowFraction = 0.5f;
inline bool IsStarting(bool aMoving, V3 aFuture, V3 aVelocity, bool aCurrentDbPivots)
{
    const float future = SizeXY(aFuture);
#if UTPP_LOCOSTART_MUTATION == 2
    const float margin = kStartingMargin; // mutation 2: the fixed motion margin for slow targets too
#else
    const float margin = future < kStartingMargin ? kStartingSlowFraction * future : kStartingMargin;
#endif
    return aMoving && future >= SizeXY(aVelocity) + margin && !aCurrentDbPivots;
}
// IsPivoting (MM, UseExperimentalStateMachine false): |TurnAngle| >= 45 (OrientToMovement) / 30 (Strafe) && IsMoving
// [P] only with a velocity heading (|Velocity|xy > kHasVelocity, motion's own HasVelocity bound): the game feed reports exactly zero capsule
// velocity on the first stick frame of every start from rest, YawOf(0) = 0, so TurnAngle was the world yaw of the future and a start
// counted as a pivot whenever that yaw was >= 45 / 30 deg from +X (2026-10-31: the crouch start played Crouch_Pivot_BR_FL at 2.33 s, the
// hipfire crouch start Crouch_Box_F_RL, the walk start Walk_Diamond_BR at 1.33 s). Above kHasVelocity the motion predicate, bit for bit.
inline bool IsPivoting(bool aMoving, V3 aFuture, V3 aVelocity, RotationMode aMode)
{
    const float thr = aMode == RotationMode::OrientToMovement ? kPivotOrient : kPivotStrafe;
#if UTPP_LOCOSTART_MUTATION == 1
    return std::fabs(TurnAngle(aFuture, aVelocity)) >= thr && aMoving; // mutation 1: no velocity floor
#else
    return SizeXY(aVelocity) > kHasVelocity && std::fabs(TurnAngle(aFuture, aVelocity)) >= thr && aMoving;
#endif
}
// |NormalizedDeltaRotator(actor, RootTransform).Yaw| (RootTransform yaw = OffsetRoot yaw + 90 in UE mesh space; the caller
// passes both yaws in the same world convention)
inline float RootDeltaYaw(float aActorYawDeg, float aRootYawDeg) { return std::fabs(NormalizeAxis(aActorYawDeg - aRootYawDeg)); }
inline bool ShouldTurnInPlace(float aRootDeltaAbs, bool aAiming, int aMovementState, int aMovementStateLast, bool aFirstPerson)
{
    return aRootDeltaAbs >= kTurnInPlaceDeg && (aAiming || (aMovementState == 0 && aMovementStateLast == 1) || aFirstPerson);
}
inline bool ShouldSpinTransition(float aRootDeltaAbs, float aSpeed2D, bool aCurrentDbPivots)
{
    return aRootDeltaAbs >= kSpinDeg && aSpeed2D >= kSpinMinSpeed && !aCurrentDbPivots;
}
inline bool JustLandedLight(bool aJustLanded, float aLandVz) { return aJustLanded && std::fabs(aLandVz) < kHeavyLand; }
inline bool JustLandedHeavy(bool aJustLanded, float aLandVz) { return aJustLanded && std::fabs(aLandVz) >= kHeavyLand; }
// HeavyLandSpeedThreshold 700 cm/s is a UE fall at 9.81 m/s^2 from 700^2 / (2 x 981) = 2.5 m. The game falls at about 16-19 m/s^2,
// so the same 2.5 m drop lands faster: the threshold scales with sqrt(g / 9.81) (g 16 -> 894 cm/s). The motion gravity, a
// non-finite g or one outside 8-30 m/s^2 gives exactly kHeavyLand.
inline constexpr float kMotionGravityMs2 = 9.81f;
inline float HeavyLandThresholdCmS(float aGravityMs2)
{
#if UTPP_TRAJV2_HEAVY_MUTATION == 1
    (void)aGravityMs2;
    return kHeavyLand; // mutation 1: the threshold stays 700 at every gravity
#else
    // g = 9.81 is exact: 9.81f / 9.81f == 1, sqrt(1) == 1 (IEEE), so the standalone default stays bit-identical at 700
    const float g = std::isfinite(aGravityMs2) && aGravityMs2 >= 8.0f && aGravityMs2 <= 30.0f ? aGravityMs2 : kMotionGravityMs2;
    return kHeavyLand * std::sqrt(g / kMotionGravityMs2);
#endif
}
// Vehicle jump-out (cool exit, double F): the launch is the game's exit jump (about +9.8 m/s up) from the seat, so the capsule comes down
// onto the car or the road at 10-12 m/s even on level ground - above the 2.5 m threshold (8.9 m/s at g 16), and the heavy land plays (a deep
// hunch that runs 3+ s). The threshold stays the ordinary one (the lighter land clip has a fast bone swing about 0.1 s after the touchdown);
// instead the recovery of a HEAVY land after a jump-out flight is capped: this many seconds after the touchdown the chooser reports
// ChooserState::jumpOutLandOver and the matcher leaves the land database for the ordinary ground one.
inline constexpr float kJumpOutLandRecoverS = 0.6f;
inline bool JustLandedLight(bool aJustLanded, float aLandVz, float aThresholdCmS) { return aJustLanded && std::fabs(aLandVz) < aThresholdCmS; }
inline bool JustLandedHeavy(bool aJustLanded, float aLandVz, float aThresholdCmS) { return aJustLanded && std::fabs(aLandVz) >= aThresholdCmS; }
inline bool PlayLand(int aMode, int aModeLast) { return aMode == 0 && aModeLast == 1; }
inline bool PlayMovingLand(int aMode, int aModeLast, float aTurnAngle)
{
    return aMode == 0 && aModeLast == 1 && std::fabs(aTurnAngle) <= kMovingLandMaxTurn;
}

// ---------------------------------------------------------------- per-frame state with _LastFrame copies (Update_States)
struct States
{
    int movementMode = 0, movementModeLast = 0;   // 0 OnGround, 1 InAir
    int movementState = 0, movementStateLast = 0; // 0 Idle, 1 Moving
    int gait = 1, gaitLast = 1;
    int stance = 0, stanceLast = 0;
    RotationMode rotationMode = RotationMode::OrientToMovement, rotationModeLast = RotationMode::OrientToMovement;
    V3 velocity, velocityLast, acceleration, accelerationLast; // cm/s, cm/s^2 (input acceleration)
    float speed2D = 0.0f;
    bool hasVelocity = false;
    // CBP OnLanded
    bool justLanded = false;
    float justLandedTimer = 0.0f;
    float landVz = 0.0f;
    // derived predicates
    V3 trjPast, trjCurrent, trjFuture;
    float turnAngle = 0.0f;
    bool isMoving = false, isStarting = false, isPivoting = false;
    bool shouldTurnInPlace = false, shouldSpin = false;
    bool justLandedLight = false, justLandedHeavy = false;
    bool playLand = false, playMovingLand = false;
    float timeToLand = kPredictionCount * kPredictionInterval;
    float stopHoldCmS = 0.0f; // motionfix item 7: chooser speed held after a speed collapse (0 = none; diagnostics)
};
} // namespace utpp::mm::motiontraj
