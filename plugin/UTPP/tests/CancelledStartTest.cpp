#include "../src/mm/MotionBlendStack.hpp"
#include <cstdio>
#include <limits>

using namespace utpp::mm;
static int failures = 0, checks = 0;
// Clip indices of the three clips under test. They differ between bakes (fixture 710 clips: 414 / 479 / 692, Data/ bake 800 clips:
// 504 / 569 / 782, bake installed in the game 726 clips: 414 / 495 / 708), so main() looks them up by NAME.
static int gRunStart = -1, gIdle = -1, gStop = -1;
static int FindClip(const motion::MotionDatabase& db, std::string_view name)
{
    for (uint32_t i = 0; i < db.NumClips(); ++i)
        if (db.ClipName(i) == name)
            return static_cast<int>(i);
    return -1;
}
static void Check(bool ok, const char* name)
{ ++checks; if (!ok) { ++failures; std::printf("FAIL %s\n", name); } }
static motion::MatchResult Match(int clip, float time, bool jump = false)
{
    motion::MatchResult result;
    result.clip = clip; result.clipTime = time; result.playRate = 1; result.jumped = jump;
    result.blend.valid = jump; result.blend.blendTime = .5f;
    result.blend.fromClip = gRunStart;
    result.cancelledStartSupportAllowed = true;
    return result;
}
static std::array<math::Vec3, 2> Feet(const motion::MotionDatabase& db, const BoneTransform* pose)
{
    std::vector<BoneTransform> model(db.NumBones());
    std::array<math::Vec3, 2> result{};
    for (uint32_t b = 0; b < db.NumBones(); ++b)
    {
        const auto parent = db.BoneParent(b);
        model[b] = parent < 0 ? pose[b] : BoneTransform{
            model[parent].p + math::QuatRotate(model[parent].q, pose[b].p),
            math::QuatNormalize(math::QuatMul(model[parent].q, pose[b].q))};
        if (db.BoneName(b) == "LeftFoot") result[0] = model[b].p;
        if (db.BoneName(b) == "RightFoot") result[1] = model[b].p;
    }
    return result;
}
static float Distance(math::Vec3 a, math::Vec3 b)
{ const auto delta = a - b; return std::sqrt(math::Dot(delta, delta)); }
static void Prepare(motionstack::BlendStack& stack, const motion::MotionDatabase& db)
{
    Check(stack.Init(db), "stack init");
    Check(stack.Step(Match(gIdle, 1), 0), "idle sample");
    Check(stack.Step(Match(gRunStart, 0, true), 0), "start request");
    for (int step = 1; step <= 12; ++step)
        Check(stack.Step(Match(gRunStart, float(step) / 60), 1.f / 60), "start phase advance");
}
static void MappedLifecycle()
{
    adapter::MapConfig config;
    config.capsuleAcceleration = 3.91f;
    config.capsuleBrakingDeceleration = 24;
    adapter::MapState state;
    adapter::MapContext context;
    adapter::MotionFeed feed;
    feed.dt = 1.f / 60;
    feed.psmDetailed = adapter::kPsmStand;
    feed.haveTarget = true;
    for (int i=0;i<30;++i) adapter::MapToChooser(feed,config,state,context);
    motion::ChooserState running;
    feed.move[1]=1;
    // Measured session 002427: ~0.200753 s hold, 3.91 m/s^2 ramp,
    // 0.789 m/s at release. Use the actual production mapper and native model.
    for(int i=0;i<12;++i)
    {
        feed.velWorld[1]=3.91f*float(i)/60;
        running=adapter::MapToChooser(feed,config,state,context);
    }
    Check(running.gait==1 && running.movementState==1 && running.isStarting,
          "actual mapper marks pre-release run as starting under native prediction");
    Check(std::fabs(state.capsuleAccelModel-3.91f)<.001f,"actual mapper uses current native acceleration predictor");
    const auto candidate=[&](const motion::ChooserState& a,const motion::ChooserState& b) {
        return cancelstart::Candidate(motion::CancellationState(a),motion::CancellationState(b),true,
            steer::kClassStart,.2f,steer::kClassStop,false);
    };
    feed.velWorld[1]=.789f;feed.move[1]=0;
    const auto released=adapter::MapToChooser(feed,config,state,context);
    std::printf("INFO mapped run(gait=%d,start=%d,movement=%d) release(gait=%d,movement=%d,speed=%.3f)\n",
                running.gait,running.isStarting,running.movementState,released.gait,released.movementState,released.speed2dCmS);
    Check(released.gait==0 && released.movementState==0 && !released.explicitWalk,
          "released VariableSpeedWalkRun input is inferred Walk, not explicit gait switch");
    Check(candidate(running,released),"actual mapped Run to released Walk cancellation is admitted");
    Check(cancelstart::SupportAllowed(motion::CancellationState(released)),"mapped release permits support lifecycle");
    feed.velWorld[1]=0;
    const auto stopped=adapter::MapToChooser(feed,config,state,context);
    Check(cancelstart::SupportAllowed(motion::CancellationState(stopped)),"mapped idle keeps support window eligible");
    feed.move[1]=1;
    const auto resumed=adapter::MapToChooser(feed,config,state,context);
    Check(!cancelstart::SupportAllowed(motion::CancellationState(resumed)),"actual new input cancels support eligibility");
    feed.wantsToWalk=true;
    const auto walk=adapter::MapToChooser(feed,config,state,context);
    Check(walk.explicitWalk && !candidate(running,walk),"real held-input walk toggle is excluded");
    feed.move[1]=0;
    const auto walkReleased=adapter::MapToChooser(feed,config,state,context);
    Check(walkReleased.explicitWalk && !candidate(running,walkReleased) && !candidate(walk,released),
          "real explicit Walk on either side cannot become run cancellation");
    feed.wantsToWalk=false;feed.psmDetailed=adapter::kPsmJump;
    const auto air=adapter::MapToChooser(feed,config,state,context);
    Check(!cancelstart::SupportAllowed(motion::CancellationState(air)),"actual airborne mapping cancels support");
    feed.psmDetailed=adapter::kPsmCrouch;
    const auto crouch=adapter::MapToChooser(feed,config,state,context);
    Check(!candidate(running,crouch),"actual crouch mapping remains excluded");
}
int main(int argc, char** argv)
{
    if (argc < 2) return 2;
    const bool baseline = argc > 2 && std::string_view(argv[2]) == "--baseline";
    motion::MotionDatabase db; std::string error;
    if (!db.LoadFromFile(argv[1], motion::kRigIdentityWoman, error)) { std::puts(error.c_str()); return 2; }
    gRunStart = FindClip(db, "M_Neutral_Run_Start_F_Lfoot");
    gIdle = FindClip(db, "M_Neutral_Stand_Idle_Loop");
    gStop = FindClip(db, "M_Neutral_Walk_Stop_F_Lfoot");
    std::printf("INFO clips (by name) run-start=%d idle=%d walk-stop=%d of %u\n", gRunStart, gIdle, gStop, db.NumClips());
    if (gRunStart < 0 || gIdle < 0 || gStop < 0)
    {
        std::puts("FAIL required clip missing in the database (M_Neutral_Run_Start_F_Lfoot / M_Neutral_Stand_Idle_Loop / M_Neutral_Walk_Stop_F_Lfoot)");
        return 1;
    }
    MappedLifecycle();
    cancelstart::State before{0,1,0,1,98.6f,true,false}, after{0,0,0,1,98.6f,false,false};
    const auto candidate = [&](cancelstart::State a, cancelstart::State b, float t = .2f,
                               int source = steer::kClassStart, int dest = steer::kClassStop, bool forced = false) {
        return cancelstart::Candidate(a,b,true,source,t,dest,forced);
    };
    Check(candidate(before,after), "observed early run cancellation admitted");
    Check(!candidate(before,after,.36f), "later start unaffected");
    Check(!candidate(before,after,.2f,steer::kClassReface), "reface preserved");
    Check(!candidate(before,after,.2f,steer::kClassStart,steer::kClassStop,true), "force interrupt preserved");
    Check(!candidate(before,after,.2f,steer::kClassStart,steer::kClassTurnInPlace), "TIP destination preserved");
    Check(!candidate(before,after,.2f,steer::kClassStartSprint), "sprint class preserved");
    Check(!cancelstart::Candidate(before,after,false,steer::kClassStart,.2f,steer::kClassStop,false), "loops preserved");
    for (int mutation = 0; mutation < 9; ++mutation)
    {
        auto a = before, b = after;
        if (mutation == 0) a.mode = 1;
        if (mutation == 1) b.mode = 1;
        if (mutation == 2) a.stance = b.stance = 1;
        if (mutation == 3) a.gait = b.gait = 2;
        if (mutation == 4) a.gait = b.gait = 0;
        if (mutation == 5) a.special = true;
        if (mutation == 6) b.special = true;
        if (mutation == 7) b.movement = 1;
        if (mutation == 8) b.speed = 151;
        Check(!candidate(a,b), "domain boundary rejected");
    }
    const auto plan = cancelstart::Resolve(db,gRunStart,.2f,1,gStop,1.367f,1,.648f,1,.5f);
    Check(plan.support == 1 && plan.duration >= .12f && plan.duration <= .2f, "shared right support determines bounded fade");
    Check(cancelstart::Resolve(db,gRunStart,.2f,1,gStop,1.367f,1,1,1,.5f).support == -1, "ambiguous displayed double support not invented");
    Check(cancelstart::Resolve(db,gRunStart,.2f,1,gStop,1.367f,1,.8f,.4f,.5f).support == -1, "weak displayed contact excluded");
    Check(cancelstart::Resolve(db,gRunStart,.2f,1,gStop,1.6f,1,.648f,1,.5f).support == -1, "unshared incoming support excluded");

    motionstack::BlendStack stack; Prepare(stack,db);
    const auto displayed = stack.OutputSignals();
    const auto previous = Feet(db, stack.Output());
    auto stop = Match(gStop,1.367f,true);
    stop.blend.cancelledGroundStart = !baseline;
    Check(stack.Step(stop,0), "stop inserted");
    const auto inserted = Feet(db, stack.Output());
    Check(Distance(previous[0],inserted[0]) < .00001f && Distance(previous[1],inserted[1]) < .00001f,
          "zero time cancellation preserves displayed feet exactly");
    std::printf("INFO mode=%s before contacts=%.5f,%.5f fade=%.6f support=%d\n", baseline?"baseline":"fixed",
                displayed.left,displayed.right,stack.At(0).duration,stack.CancelledStartSupport());
    Check(stack.At(0).duration <= .2f, "cancelled start outgoing removed within 0.2 seconds");
    Check(stack.CancelledStartSupport() == 1, "hold reports previous displayed right support");
    if (!baseline)
        Check(stack.OutputSignals().left < .3f && stack.OutputSignals().right >= .85f,
              "cancelled source swing releases stale idle contact without dropping the support foot");
    auto previousFeet = inserted; float maxJump = 0, quarterWeight = -1;
    bool sawSourceSwing = false, sawSharedContact = false, sawIncomingOnly = false;
    for (int step = 1; step <= 30; ++step)
    {
        stop.jumped = false; stop.clipTime = 1.367f + float(step)/60;
        Check(stack.Step(stop,1.f/60), "cancellation frame valid");
        const auto current = Feet(db,stack.Output());
        for (int side=0;side<2;++side) maxJump=std::max(maxJump,Distance(previousFeet[side],current[side]));
        previousFeet=current;
        if(step==15)
        {
            quarterWeight=0;
            for(uint32_t layer=0;layer<stack.Count();++layer)
                if(stack.At(layer).clip==gRunStart) quarterWeight+=stack.Weight(layer);
        }
        const int support=stack.CancelledStartSupport();
        Check(support<0 || (support==1 && stack.OutputSignals().right>=.7f), "never holds swing or untrusted support");
        if (!baseline && stack.Count()>1 && stack.At(0).clock<stack.At(0).duration)
        {
            const auto& source=stack.At(1);const auto& destination=stack.At(0);
            const float from=cancelstart::Contact(db,source.clip,source.time,0);
            const float to=cancelstart::Contact(db,destination.clip,destination.time,0);
            const float expected=std::min(from,to);
            Check(std::fabs(stack.OutputSignals().left-expected)<.00001f,
                  "non-support contact follows both actual sampled layer phases");
            if(from<.3f && to>.5f)
            {
                sawSourceSwing=true;
                Check(stack.OutputSignals().left<.3f,"incoming plant cannot reacquire while outgoing source still swings");
            }
            if(stack.OutputSignals().left>.5f)
            {sawSharedContact=true;Check(from>.5f&&to>.5f,"reacquisition requires support in both live layers");}
        }
        else if(!baseline && stack.Count()==1)
        {
            sawIncomingOnly=true;
            Check(std::fabs(stack.OutputSignals().left-cancelstart::Contact(db,stack.At(0).clip,stack.At(0).time,0))<.00001f,
                  "fade completion restores ordinary incoming contact");
        }
    }
    if(!baseline)
        Check(sawSourceSwing&&sawIncomingOnly,"test traverses source swing and completed fade domains");
    (void)sawSharedContact; // Some sample rates reach fade end before a shared-contact sample.
    std::printf("INFO quarter-start-weight=%.6f max-local-ankle-frame-step=%.6fm\n",quarterWeight,maxJump);
    Check(quarterWeight==0, "no accelerating start contribution after 0.25 seconds");
    Check(maxJump<.06f, "60 Hz cancellation has no ankle jump over 6 cm");
    Check(stack.CancelledStartSupport()==-1, "hold expires at fade end");

    Prepare(stack,db); stop=Match(gStop,1.367f,true);stop.blend.cancelledGroundStart=true;
    stack.Step(stop,0);const auto tinyBefore=Feet(db,stack.Output());
    stop.jumped=false;stop.clipTime+=.00001f;stack.Step(stop,.00001f);
    const auto tinyAfter=Feet(db,stack.Output());
    const float tinyDelta=std::max(Distance(tinyBefore[0],tinyAfter[0]),Distance(tinyBefore[1],tinyAfter[1]));
    std::printf("INFO 10us cancellation boundary ankle displacement=%.9fm\n",tinyDelta);
    Check(tinyDelta<.0001f, "fade boundary converges continuously to the prior pose");

    Prepare(stack,db); stop=Match(gStop,1.367f,true);stop.blend.cancelledGroundStart=true;
    stack.Step(stop,0);stop.jumped=false;stop.cancelledStartSupportAllowed=false;
    stack.Step(stop,0);
    Check(stack.CancelledStartSupport()==-1, "new input or air cancels hold even at dt0");
    Check(stack.OutputSignals().left>.3f,"scope cancellation restores ordinary contact mixing at dt0");
    Prepare(stack,db); stop=Match(gStop,1.367f,true);stop.blend.cancelledGroundStart=true;
    stack.Step(stop,0);stack.Step(Match(gIdle,1,true),0);
    Check(stack.CancelledStartSupport()==-1, "unrelated selection cancels hold");
    stack.Reset();Check(stack.CancelledStartSupport()==-1, "reset cancels hold");
    std::printf("CancelledStart: %d checks %d failures\n",checks,failures);
    return failures?1:0;
}
