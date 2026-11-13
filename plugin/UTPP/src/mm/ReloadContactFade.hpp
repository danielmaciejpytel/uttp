#pragma once
// Game-thread policy only; never call Step
// from FK. Native reload envelope remains independent of this correction fade.
#include <algorithm>
#include <cmath>
namespace utpp::mm::reload_contacts_runtime
{
// Verified authored clips only: shape the output, never the raw controller clock.
// Pure remapping keeps retrigger C0 continuity when raw weight is continuous.
inline float AuthoredFadeWeight(float raw)
{
    if(!std::isfinite(raw))return 0;
    const float w=std::clamp(raw,0.0f,1.0f);
    return w*w*(3.0f-2.0f*w);
}
// Native-reload upper-body handback envelope (MotionDriver g_reloadUpperWeight), kept
// here so its timing policy is testable offline. Bit-identical to the former inline
// ramp (.12s in / .20s out) except for a reload that interrupts ADS: the game drops
// the aim flag and pops its arms to the reload pose within ~2 frames, and the old .12s
// ramp moved the weapon 20-40cm per .1s (recorded 03:37 k5/k8/k10/k11, 20.10 10:31).
inline constexpr float kUpperEntrySeconds=.12f;
#if defined(UTPP_RELOAD_ENVELOPE_MUTATION) && UTPP_RELOAD_ENVELOPE_MUTATION==1
inline constexpr float kUpperEntryFromAimSeconds=.12f; // mutation 1: old aim-interrupt ramp
#else
inline constexpr float kUpperEntryFromAimSeconds=.30f;
#endif
// Pistol reload whose edge is NOT native ADS (hip-fire / ready overlay at the edge, crouch turn, stand): the MM overlay arms
// and the FPP reload pose are 82-127 deg apart (recording 2026-10-25: 43 Nue entries, arc g-vs-mm 60-127 deg; peak final
// arm speed 15-28 deg/frame at 60 fps = 900-1700 deg/s with the .12 s smoothstep). Entry of the authored smoothstep over
// .24 s halves the handback share of that speed (weight-only peak 18.7 -> 11.0 deg/frame mean, offline replay of the arms2.jsonl
// stages). Rifles keep .12 s bit for bit.
#if defined(UTPP_RELOAD_ENVELOPE_MUTATION) && UTPP_RELOAD_ENVELOPE_MUTATION==3
inline constexpr float kUpperEntryPistolSeconds=.12f; // mutation 3: pistol entry as before
#else
inline constexpr float kUpperEntryPistolSeconds=.24f;
#endif
inline constexpr float kUpperExitSeconds=.20f;
struct UpperEnvelope
{
    float weight=0;
    bool active=false,fromAim=false,entryPistol=false;
    void Reset(){weight=0;active=false;fromAim=false;entryPistol=false;}
    // aimingBeforeEdge = the overlay aim state of the previous published frame; it is only
    // sampled on the non-reload -> reload edge and latched for that whole reload. pistol (default false = former
    // behaviour for every caller) is latched on the same edge; ADS (fromAim) keeps its .30 s ramp.
    float Step(float dt,bool reload,bool aimingBeforeEdge,bool pistol=false)
    {
        if(!std::isfinite(dt) || dt<=0)return weight;
        if(reload && !active){fromAim=aimingBeforeEdge;entryPistol=pistol;}
        if(!reload && !active){fromAim=false;entryPistol=false;}
        active=reload;
        const float seconds=reload ? (fromAim ? kUpperEntryFromAimSeconds : (entryPistol ? kUpperEntryPistolSeconds : kUpperEntrySeconds))
                                   : kUpperExitSeconds;
        const float step=std::min(dt,.25f)*(1.0f/seconds);
        weight=reload ? std::min(1.0f,weight+step) : std::max(0.0f,weight-step);
        return weight;
    }
    // While an aim-interrupting reload is still blending in, the MM overlay keeps the aim
    // pose it had before the edge (the game's own aim drop would otherwise move the MM
    // arms by ~9cm in one frame at ~full weight). Releases when the arms are fully native.
    bool AimHold()const{return active && fromAim && weight<1.0f;}
};

// Reload started inside the rolled phase of a slide (Slide_FootOut_Into_*): the MM torso is rolled
// ~35deg sideways and yawed ~55deg, the native arm locals (authored for an upright torso) follow it
// and the weapon flew 33cm in .05s to ABOVE the head (recorded 21.10 03:37 k10/k11). While the
// shoulder line (elevation = torso roll proxy, deg) is rolled, the MM overlay arms keep ownership
// and the native reload arms are blended in over kRollHoldReleaseSeconds once the torso rises.
// Gated by the driver's slide frame, so every non-slide reload keeps hold == 0 exactly.
inline constexpr float kRollHoldStartDeg=22.0f;
inline constexpr float kRollHoldFullDeg=32.0f;
inline constexpr float kRollHoldAttackSeconds=.15f;
#if defined(UTPP_RELOAD_ENVELOPE_MUTATION) && UTPP_RELOAD_ENVELOPE_MUTATION==2
inline constexpr float kRollHoldReleaseSeconds=1.0e-4f; // mutation 2: hold released at once
inline constexpr bool kRollHoldEnabled=false;           // mutation 2: policy off
#else
inline constexpr float kRollHoldReleaseSeconds=.5f;
inline constexpr bool kRollHoldEnabled=true;
#endif
// Elevation of the shoulder line, 0..90deg (0 = shoulders level).
inline float ShoulderLineRollDeg(float lx,float ly,float lz,float rx,float ry,float rz)
{
    const float dx=rx-lx,dy=ry-ly,dz=rz-lz;
    const float len=std::sqrt(dx*dx+dy*dy+dz*dz);
    if(!std::isfinite(len) || len<1.0e-4f)return 0.0f;
    return std::asin(std::clamp(std::fabs(dz)/len,0.0f,1.0f))*57.29578f;
}
struct SlideRollHold
{
    float hold=0;
    bool active=false;
    void Reset(){hold=0;active=false;}
    // reloadNow: the native reload owns (or is blending in) the arms. slideFrame: driver slide frame.
    // deRolled: SlideDeRoll is uprighting the MM torso this frame and rollDeg is the roll left after it; the reload edge then
    // takes the hold through the attack ramp instead of at once (the de-roll removes the roll within kDeRollAttackSeconds while
    // the native arms blend in; the hold covers only the frames whose roll left is still in the 22..32 deg range).
    // Default false = the former behaviour bit for bit.
    float Step(float dt,bool reloadNow,bool slideFrame,float rollDeg,bool deRolled=false)
    {
        if(!std::isfinite(dt) || dt<=0)return hold;
        float raw=0;
        if constexpr(kRollHoldEnabled)
        {
            if(reloadNow && slideFrame && std::isfinite(rollDeg))
            {
                const float x=std::clamp((rollDeg-kRollHoldStartDeg)/(kRollHoldFullDeg-kRollHoldStartDeg),0.0f,1.0f);
                raw=x*x*(3.0f-2.0f*x);
            }
        }
        const float d=std::min(dt,.25f);
        if(reloadNow && !active && !deRolled)hold=raw; // a reload that starts rolled owns the hold at once
        else if(raw>hold)hold=std::min(raw,hold+d/kRollHoldAttackSeconds);
        else hold=std::max(raw,hold-d/kRollHoldReleaseSeconds);
        active=reloadNow;
        return hold;
    }
};

// Pistol reload on a slide frame: weight of the MM torso uprighting (de-roll + de-pitch, geometry: SlideDeRoll.hpp). The slide
// torso is set upright so the reload arm locals (authored upright) can be handed back as in a standing reload; SlideRollHold, fed
// with the roll LEFT after the de-roll, stays as the safety. weight = smoothstep(ramp) * peak:
//   ramp  rises at 1/kDeRollAttackSeconds while the gate (pistol reload on a slide frame, valid rig) is open and the torso has
//         rolled into the roll range of this episode, falls at 1/kDeRollReleaseSeconds otherwise (hitches capped at .25 s);
//   peak  = max over the episode of smoothstep((roll - kRollHoldStartDeg) / (kRollHoldFullDeg - kRollHoldStartDeg)), the same
//         range as the hold; it never decreases while the episode lasts (the torso rising out of the slide must not let the
//         correction drop away and roll the shoulders back up), and is cleared once the ramp is back at 0 outside the gate.
// rollDeg is the shoulder-line roll of the MM pose BEFORE the de-roll (no feedback). Closed gate and ramp 0: weight 0, the
// caller never touches the pose (bit-identical).
// .15 s. Offline replay of the 2026-10-25 slide reloads (tools/analysis/replay_slide_deroll.cpp): .25 s lowers the entry weapon
// motion only to 13.1 / 15.3 cm per .05 s (from 15.4 / 16.4, matrix #4 / rec4 #27; the Spine3-frame entry motion is unchanged), but
// the roll stays above 22 deg longer, the hold takes up to .54 and the native arms are full .09-.12 s later than in a standing
// reload (tests/ReloadSlideRollHoldTest.cpp hand-back checks fail with .25 s).
inline constexpr float kDeRollAttackSeconds=.15f;
inline constexpr float kDeRollReleaseSeconds=.25f;
#if defined(UTPP_RELOAD_ENVELOPE_MUTATION) && UTPP_RELOAD_ENVELOPE_MUTATION==4
inline constexpr bool kDeRollEnabled=false; // mutation 4: no torso counter-roll (the hold alone, as before)
#else
inline constexpr bool kDeRollEnabled=true;
#endif
// Rifle reload on a slide frame (recording 2026-10-26 11:28: 8 rifle slide reloads, PSM 10/5, uncorrected - the native arm locals on
// the rolled slide torso put the left upper arm 2.3 cm from the neck and the shoulder line up to 10 cm above it) gets the same torso
// uprighting as the pistols, so the rifle reload-contact correction (ReloadCorrectionPose, chest-frame transport) carries the native
// hands in an upright chest frame. The roll hold (SlideRollHold) stays pistol-only: for a rifle the correction owns the arms, and
// the hold would change the rifle upper scale / AO (MotionDriver g_reloadNowStep stays pistol-only, the de-roll reads this gate).
// Mutation 6 (UTPP_RELOAD_ENVELOPE_MUTATION) keeps the de-roll pistol-only.
#if defined(UTPP_RELOAD_ENVELOPE_MUTATION) && UTPP_RELOAD_ENVELOPE_MUTATION==6
inline constexpr bool kSlideDeRollRifle=false; // mutation 6: rifle slide reloads keep the rolled slide torso
#else
inline constexpr bool kSlideDeRollRifle=true;
#endif
// Gate of the slide torso uprighting (SlideDeRoll, before the slide-frame test): a reload (PSM action 2 with a weapon overlay, not
// released) of a pistol (Pistol1H / Pistol2H) or of a rifle (overlay Rifle).
inline bool SlideTorsoReload(bool reload,bool pistol,bool rifle)
{
    return reload && (pistol || (kSlideDeRollRifle && rifle));
}
// Rifle slide reload that goes on after the slide frames end (MovementMode back to OnGround while the MM slide exit clip is still
// rolled 25-35 deg): with the slide-frame gate alone the de-roll released over kDeRollReleaseSeconds on the first ground frame and
// rolled the uprighted chest - and the native hands carried in its frame - back by up to 29 deg before the exit clip un-rolled it
// (replay of recording 2026-10-26 11:28 ep 17: roll left 3 -> 29 -> 0 deg within .5 s; ep 12 3 -> 27). The latch keeps the gate open
// after a slide frame of the same rifle reload while the MM roll (before the de-roll) is still in the roll range (>= kRollHoldStartDeg);
// once it drops below, or the reload ends, the latch is gone until the next slide frame (no re-latch on a roll that rises again).
// Rifle only: the pistol de-roll keeps the slide-frame gate bit for bit. Mutation 7 (UTPP_RELOAD_ENVELOPE_MUTATION) disables the latch.
#if defined(UTPP_RELOAD_ENVELOPE_MUTATION) && UTPP_RELOAD_ENVELOPE_MUTATION==7
inline constexpr bool kRifleSlideExitLatch=false; // mutation 7: the rifle de-roll releases on the first ground frame
#else
inline constexpr bool kRifleSlideExitLatch=true;
#endif
struct RifleSlideExitLatch
{
    bool latched=false;
    void Reset(){latched=false;}
    // rifleReload: SlideTorsoReload(...) of a rifle reload, or the rifle carry path (SlideCarryRifle; the driver passes either);
    // slideFrame: the de-roll's slide frame (g_slideFrame); rollDeg: MM roll before the de-roll. Returns true when a NON-slide frame keeps the de-roll gate open.
    bool Step([[maybe_unused]] bool rifleReload,[[maybe_unused]] bool slideFrame,[[maybe_unused]] float rollDeg)
    {
        if constexpr(kRifleSlideExitLatch)
        {
            if(!rifleReload){latched=false;return false;}
            if(slideFrame){latched=true;return false;}
            if(latched && std::isfinite(rollDeg) && rollDeg>=kRollHoldStartDeg)return true;
        }
        latched=false;
        return false;
    }
};
// Rifle CARRY on a slide frame (no reload; recording 2026-10-27 15:14): MM owns the arms
// (upper 1), the Rifle overlay arm locals ride the slide clip torso whose shoulder line is rolled ~35 deg, so the barrel pitched up to
// +33..35 deg (Ready -23.6) and snapped back at the slide exit (23-43 deg per .2 s). The same torso uprighting (SlideDeRoll + the exit
// latch) runs for a drawn rifle whose arms MM owns, outside a reload; the weight is scaled by (1 - overlay AimingWeight) (SlideCarryAim,
// the AO alpha that stays 1 through the aim / hip-fire hold) so aimed slides keep their pose. Its ramp starts on the first slide frame
// (earlyRamp, the weight rise limited to kCarryRiseSeconds) instead of waiting for the roll range: offline model sim2.py 'early' (entry
// barrel peak 11-12 deg instead of 17-20, pitch on the levelled slide -2.4 deg). Pistols never take this path (bit-identical). Mutation 8 (UTPP_RELOAD_ENVELOPE_MUTATION)
// disables it.
#if defined(UTPP_RELOAD_ENVELOPE_MUTATION) && UTPP_RELOAD_ENVELOPE_MUTATION==8
inline constexpr bool kSlideCarryDeRoll=false; // mutation 8: no torso uprighting outside a reload
#else
inline constexpr bool kSlideCarryDeRoll=true;
#endif
// rifle: overlay Rifle; mmOwnsArms: MM publishes the upper body at scale 1 (MotionDriver MMOwnsArmsLocked); reload: a reload of this Step
// (SlideTorsoReload) - the reload path owns the torso uprighting then.
inline bool SlideCarryRifle(bool rifle,bool mmOwnsArms,bool reload)
{
    return kSlideCarryDeRoll && rifle && mmOwnsArms && !reload;
}
inline constexpr float kCarryRiseSeconds=.06f; // fastest rise of the carry weight 0 -> 1 (SlideDeRoll::Step earlyRamp)
// Scale of the carry de-roll weight: 1 - AimingWeight while the carry path owns the episode (an aimed slide is not uprighted). When a
// reload takes the torso over from the carry path the scale rises back to 1 at 1/kDeRollAttackSeconds (an aimed carry hands over
// without a step), during the release after a carry it keeps following the aim. Without a carry frame it stays exactly 1 (reload /
// pistol: w * 1 == w, bit-identical); the caller resets it once the de-roll is idle.
struct SlideCarryAim
{
    bool carry=false;
    float scale=1.0f;
    void Reset(){carry=false;scale=1.0f;}
    float Step(float dt,bool carryFrame,bool reloadFrame,float aimWeight)
    {
        if(carryFrame)carry=true;
        else if(reloadFrame)carry=false;
        if(carry)scale=1.0f-(std::isfinite(aimWeight) ? std::clamp(aimWeight,0.0f,1.0f) : 1.0f); // non-finite aim: treated as aimed
        else if(scale<1.0f && std::isfinite(dt) && dt>0)scale=std::min(1.0f,scale+std::min(dt,.25f)/kDeRollAttackSeconds);
        return scale;
    }
};
struct SlideDeRoll
{
    float ramp=0,peak=0,weight=0;
    bool riseLimited=false; // the carry rise limit clipped the weight below its target; kept until it has caught up (any gate)
    void Reset(){ramp=0;peak=0;weight=0;riseLimited=false;}
    bool Idle()const{return ramp==0.0f && weight==0.0f;}
    // gate: pistol / rifle reload (SlideTorsoReload) on a slide frame (a rifle also while RifleSlideExitLatch holds) with a valid torso
    // chain, or the rifle carry path (SlideCarryRifle) on a slide frame / while the latch holds. earlyRamp (carry path only): the ramp
    // rises while the gate is open even before the roll has reached the roll range (the weight is still ramp x peak, so a slide that
    // never rolls into the range gets no edit), and the weight rises by at most d / kCarryRiseSeconds per Step: with the ramp already
    // up, peak follows the roll of the slide entry blend (up to 5 deg / frame, smoothstep 22..32 -> weight +.57 in one 60 Hz frame,
    // a 16 deg chest / head turn); replay of 15:34-15:35 (5 pure slides): entry barrel peak 10.7-13.5 deg (unlimited 10.7-12.2, roll-range
    // ramp 16.8-19.6), per-frame barrel pitch change within 0.5 deg of the raw clip. Reload calls pass false: the former policy bit for bit.
    float Step(float dt,bool gate,float rollDeg,bool earlyRamp=false)
    {
        if(!std::isfinite(dt) || dt<=0)return weight;
        const bool on=Engaged(gate);
        if(on && std::isfinite(rollDeg))
        {
            const float x=std::clamp((rollDeg-kRollHoldStartDeg)/(kRollHoldFullDeg-kRollHoldStartDeg),0.0f,1.0f);
            peak=std::max(peak,x*x*(3.0f-2.0f*x));
        }
        const float d=std::min(dt,.25f);
        ramp=(on && (peak>0.0f || earlyRamp)) ? std::min(1.0f,ramp+d/kDeRollAttackSeconds) : std::max(0.0f,ramp-d/kDeRollReleaseSeconds);
        if(ramp==0.0f && !on)peak=0.0f;
        const float target=ramp*ramp*(3.0f-2.0f*ramp)*peak;
        if(earlyRamp || riseLimited)
        {
            // a weight clipped on a carry frame keeps the limit until it has caught up, whichever path steps it next (a reload that
            // takes over in the entry, a carry gate that closes): it never jumps to the target, and never rises while the gate is closed
            const float lim=on ? weight+d/kCarryRiseSeconds : weight;
            riseLimited=target>lim;
            weight=riseLimited ? lim : target;
        }
        else
            weight=target;
        return weight;
    }
    // The de-roll owns the torso this frame (passed to SlideRollHold::Step as deRolled).
    static bool Engaged([[maybe_unused]] bool gate)
    {
        if constexpr(kDeRollEnabled)return gate;
        else return false;
    }
};

struct CorrectionFade
{
    float weight=0;
    void Reset(){weight=0;}
    // domainPermitted: same drawn pistol, action2 OR explicit action0 tail,
    // valid TPP owner. Movement/stance MUST NOT be part of this safety domain:
    // they only change stationaryEligible, allowing existing correction to fade.
    // Fresh FPP denial is immediate in caller Finish AND resets this controller.
    float Step(float dt,float nativeEnvelope,bool stationaryEligible,
               bool domainPermitted,bool perspectiveAllowed)
    {
        if(!domainPermitted || !perspectiveAllowed || !std::isfinite(dt) || dt<0 ||
           !std::isfinite(nativeEnvelope) || nativeEnvelope<0 || nativeEnvelope>1)
        {Reset();return 0;}
        const float target=stationaryEligible?nativeEnvelope:0;
        // Preserve the established .12s native entry ceiling; all correction
        // withdrawals (movement, crouch, action0 tail) last at most .2s from1.
        // A long game-thread hitch cannot make a one-frame correction jump.
        const float step=std::min(dt,.05f)/(target>weight?.12f:.2f);
        weight=target>weight?std::min(target,weight+step):std::max(target,weight-step);
        return weight;
    }
    bool Active()const{return weight>1e-5f;}
};

// Classifies one native reload episode once, before the stationary contact
// correction can take ownership. The PSM action edge starts an episode; pose
// motion changes inside that episode may only revoke eligibility, never grant
// it again. Keep this policy separate from CorrectionFade so existing fades
// and their continuity are unchanged.
// 2026-10-25 experiment (default OFF: policy tests expect stationary-only; in-game funnel showed 100 corrected frames for a moving pistol reload but no visible gain): reloads while moving / crouching / sliding also get the native contact correction (the divergence
// transport already carries the native hands when the MM body is far from the game body). 0 = former stationary-only policy.
#ifndef UTPP_RELOAD_CORRECTION_MOVING
#define UTPP_RELOAD_CORRECTION_MOVING 0
#endif
inline constexpr bool kReloadCorrectionMoving=UTPP_RELOAD_CORRECTION_MOVING!=0;
struct CorrectionEpisode
{
    static constexpr float kStationarySpeedMps=0.05f;
    static constexpr float kEnvelopeEndEpsilon=1.0e-5f;

    bool entryPending=false;
    bool observedActive=false;
    bool allowed=false;
    bool authoredSeen=false;

    void Reset(){entryPending=false;observedActive=false;allowed=false;authoredSeen=false;}
    // Call only on the actual non-reload -> reload action edge. Does not touch
    // any CorrectionFade weight owned by the caller.
    void Begin(){entryPending=true;observedActive=false;allowed=false;authoredSeen=false;}

    // reloadActive is PSM action==2; false is the action-0 envelope tail.
    // domainPermitted includes TPP/weapon/release/body safety. Any loss of
    // that domain or invalid feed clears the episode fail-closed.
    bool Observe(bool reloadActive,bool stationaryPose,float physicalSpeed,
                 bool domainPermitted,float nativeEnvelope,bool authoredClip=false)
    {
        if(!domainPermitted || !std::isfinite(physicalSpeed) || physicalSpeed<0.0f ||
           !std::isfinite(nativeEnvelope) || nativeEnvelope<0.0f || nativeEnvelope>1.0f)
        {Reset();return false;}

        if(reloadActive)
        {
            if(entryPending)
            {
                entryPending=false;
                observedActive=true;
                allowed=kReloadCorrectionMoving || (stationaryPose && physicalSpeed<=kStationarySpeedMps);
            }
            else if(!observedActive)
                return false; // no edge: never admit late on a stop/FPP return
            else if(!kReloadCorrectionMoving && allowed && (!stationaryPose || physicalSpeed>kStationarySpeedMps))
                allowed=false; // movement/stance change revokes until the next episode

            authoredSeen=authoredSeen || authoredClip;
            return allowed && nativeEnvelope>kEnvelopeEndEpsilon;
        }

        // Action 0 may be the ordinary native fade tail. It inherits only the
        // already classified episode and expires with that native envelope.
        if(!observedActive)
        {Reset();return false;}
        authoredSeen=authoredSeen || authoredClip;
        if(!kReloadCorrectionMoving && (!stationaryPose || physicalSpeed>kStationarySpeedMps))
            allowed=false; // tail movement also cannot acquire on a later stop
        const bool result=allowed && nativeEnvelope>kEnvelopeEndEpsilon;
        if(nativeEnvelope<=kEnvelopeEndEpsilon)Reset();
        return result;
    }
};

// Rifle reloads (overlay Rifle) in every ground stance and gait. The rifle hand targets are carried in the chest (Spine3) frame of the
// MM torso (NativeReloadContactsRuntime kRifleChestTransport), so they do not depend on the native body standing where the MM body
// stands; the stationary rule of the pistols (whose targets sit in model space / on the MM shoulders) is not needed for them. Offline
// replay of the recorded rifle reloads (recordings 3/4/5 of 2026-10-25):
// runs / sprints, crouch, stand <-> crouch and run <-> stop inside the reload keep the native contacts (move 826 / 830 frames accepted,
// crouch 202 / 203; the rejects are the clavicle-reach / Contact holds the standing reloads also have). Air (MovementMode
// InAir) keeps the correction out: the target falls to 0 and the CorrectionFade withdraws over its .2 s (never a reset), and a landing
// ramps it back in over .12 s. Slide frames are admitted since 2026-10-26 (kReloadContactsRifleSlide below). No authored clip exists for a rifle, so CorrectionEpisode never gates a rifle reload.
// Mutation 3 (UTPP_RELOAD_RIFLE_MUTATION, tests/reloadfix.cmake) restores the stationary-only rule for rifles (88910e9).
#if defined(UTPP_RELOAD_RIFLE_MUTATION) && UTPP_RELOAD_RIFLE_MUTATION==3
inline constexpr bool kReloadContactsRifleAnyStance=false; // mutation 3: rifles keep the stationary-only rule of the pistols
#else
inline constexpr bool kReloadContactsRifleAnyStance=true;
#endif
// Rifle reloads on a slide frame (MovementMode Sliding 2 with the slide bundle, or the PSM slide detail on the ground) keep the
// correction too: the de-roll above (SlideTorsoReload) uprights the MM torso and the chest-frame transport puts the native hands on it,
// with the MM shoulders restored (recording 2026-10-26 11:28: the uncorrected slide reloads were the only rifle reloads left with
// the native shoulders, LeftArm 2.3 cm from the neck). The air (MovementMode InAir 1) stays out. The driver adds a static guard:
// a map slide frame is admitted only with a valid de-roll chain (the native hands are never carried onto an un-uprightable rolled
// chest). Mutation 4 (UTPP_RELOAD_RIFLE_MUTATION) restores the former rule (slide frames fade the correction out).
#if defined(UTPP_RELOAD_RIFLE_MUTATION) && UTPP_RELOAD_RIFLE_MUTATION==4
inline constexpr bool kReloadContactsRifleSlide=false; // mutation 4: rifle slide frames excluded (e00bb09)
#else
inline constexpr bool kReloadContactsRifleSlide=true;
#endif
// Pose rule of the correction fade target for one frame (MotionDriver FrameInputsLocked; the offline replay calls the same function).
// stationary: the pistol rule (PSM stance Stand, MovementState Idle, MovementMode OnGround) - returned unchanged for every overlay but Rifle.
// Rifle: MovementMode OnGround (0) - stand / crouch, idle / walk / run / sprint and changes between them - and slide frames (MovementMode
// 0 or Sliding 2); never InAir (1). slideFrame = the driver's slide frame (map slide or PSM slide detail).
inline bool ReloadCorrectionPose(bool rifle,bool stationary,int movementMode,bool slideFrame)
{
    if constexpr(kReloadContactsRifleAnyStance)
    {
        if(rifle)
        {
            if(!slideFrame)return movementMode==0;
            return kReloadContactsRifleSlide && (movementMode==0 || movementMode==2);
        }
    }
    return stationary;
}
}
