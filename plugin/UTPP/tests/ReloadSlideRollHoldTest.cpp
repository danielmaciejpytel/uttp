// Offline test of the slide roll hold (ReloadContactFade.hpp SlideRollHold + UpperEnvelope). No game process.
// Fixtures: 21.10 03:37 reloads started in the rolled phase of a slide (k10 03:39:50.93 Into_Rfoot, k11 03:39:58.65
// Into_Lfoot), 92 frames from the reload edge - 2 frames. Per frame: dt, slide clip flag, torso roll proxy (shoulder-line
// elevation of the published MM pose, deg), weapon position in the Spine3 frame (cm) with the arms fully native / frozen
// MM aim arms / as recorded (stage 2), and weapon height minus head height (cm) for native / MM arms on the MM torso.
// Mutation 2 (UTPP_RELOAD_ENVELOPE_MUTATION=2) disables the hold: the test must fail.
// Section 3: torso counter-roll of a pistol reload on a slide frame (SlideDeRoll weight + slide_deroll::Apply on a synthetic
// rolled CP-like skeleton): the shoulder line is levelled, the weight ramps without steps at 30 / 60 / 144 Hz, closed gates leave
// the pose bit-identical, and with the residual roll the hold stays ~0 so the arms are handed back as in a standing reload.
// Mutation 4 (UTPP_RELOAD_ENVELOPE_MUTATION=4) disables the de-roll (weight), mutation 5 only the de-pitch: the test must fail.
// The torso is also pitched about the levelled shoulder line to the standing reload lean (slide_deroll::kDePitchTargetDeg).
// Section 4: rifle reloads on a slide frame (SlideTorsoReload, RifleSlideExitLatch). Mutation 6 keeps the uprighting pistol-only,
// mutation 7 disables the rifle exit latch: the test must fail.
// Section 5: rifle CARRY (no reload) on a slide frame (SlideCarryRifle, early ramp, SlideCarryAim): the chest-carried barrel is
// levelled, aimed slides untouched, the reload / pistol weight bit-identical, steps bounded at 30 / 60 / 144 Hz, the exit latch holds.
// Mutation 8 disables the carry path: the test must fail.
#include "../src/mm/ReloadContactFade.hpp"
#include "../src/mm/SlideDeRoll.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
using namespace utpp::mm::reload_contacts_runtime;
static int g_fail=0,g_pass=0;
static void Check(bool ok,const char* what){(ok?g_pass:g_fail)++;std::printf("%s %s\n",ok?"PASS":"FAIL",what);}

// N37 03:39:50.93 k10 (M_Neutral_Slide_FootOut_Into_Rfoot) edge at index 2
static const float kSlideR_dt[92]={0.01537f,0.01840f,0.01537f,0.01827f,0.01513f,0.01627f,0.01715f,0.01735f,0.01524f,0.01700f,0.01888f,0.01524f,0.01650f,0.01608f,0.01684f,0.01738f,0.01792f,0.01533f,0.01709f,0.01627f,0.01604f,0.01655f,0.01875f,0.01470f,0.01672f,0.01679f,0.01684f,0.01629f,0.01893f,0.01493f,0.01649f,0.01825f,0.01590f,0.01638f,0.01574f,0.01932f,0.01468f,0.01675f,0.01783f,0.01604f,0.01602f,0.01659f,0.01830f,0.02103f,0.01084f,0.01652f,0.01671f,0.01673f,0.01612f,0.01991f,0.01441f,0.01675f,0.01730f,0.01638f,0.01579f,0.02263f,0.01241f,0.01627f,0.01669f,0.01649f,0.01675f,0.01525f,0.01928f,0.01412f,0.01637f,0.01736f,0.01639f,0.01715f,0.01600f,0.01754f,0.01598f,0.01685f,0.01696f,0.01654f,0.02498f,0.01103f,0.01393f,0.01602f,0.01701f,0.01628f,0.01686f,0.01691f,0.01587f,0.01700f,0.01685f,0.01626f,0.01668f,0.01645f,0.01697f,0.01859f,0.01486f,0.01659f};
static const unsigned char kSlideR_slide[92]={1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
static const float kSlideR_roll[92]={33.86f,34.33f,34.51f,37.27f,37.35f,37.44f,37.44f,37.44f,37.52f,37.59f,37.33f,37.12f,36.92f,36.72f,36.36f,36.00f,35.77f,35.57f,35.13f,34.72f,34.68f,34.64f,34.54f,34.45f,34.57f,34.69f,34.89f,35.08f,35.44f,35.72f,35.67f,35.62f,35.74f,35.85f,35.89f,35.97f,35.56f,33.67f,30.05f,25.93f,21.43f,16.39f,10.84f,5.53f,2.86f,0.00f,1.78f,2.34f,1.99f,1.59f,1.26f,0.82f,0.35f,0.15f,0.65f,1.43f,1.93f,2.67f,3.24f,3.78f,4.05f,4.10f,4.06f,3.76f,3.44f,3.04f,2.63f,2.34f,2.19f,2.05f,1.94f,1.84f,1.59f,1.29f,0.87f,0.61f,0.32f,0.08f,0.47f,0.92f,1.38f,1.87f,2.37f,2.91f,3.43f,3.95f,4.40f,4.81f,5.17f,5.38f,5.58f,5.54f};
static const float kSlideR_native[92][3]={{22.49f,-39.67f,0.97f},{26.45f,-44.07f,0.61f},{32.29f,-51.16f,0.24f},{37.81f,-58.64f,-0.26f},{40.91f,-63.17f,-0.99f},{41.89f,-64.40f,-1.43f},{41.74f,-63.59f,-1.45f},{41.31f,-62.13f,-1.39f},{40.91f,-60.76f,-1.27f},{40.56f,-59.37f,-1.02f},{40.40f,-58.04f,-0.71f},{40.84f,-57.67f,-0.27f},{41.04f,-57.30f,-0.17f},{41.02f,-57.28f,0.09f},{41.35f,-57.13f,0.14f},{41.50f,-57.31f,1.10f},{41.58f,-57.50f,2.50f},{41.60f,-57.09f,3.45f},{41.28f,-56.79f,3.85f},{40.77f,-56.87f,3.80f},{40.57f,-56.75f,4.13f},{40.36f,-56.57f,4.38f},{40.11f,-56.42f,4.69f},{39.96f,-56.38f,5.04f},{39.77f,-56.32f,5.43f},{39.52f,-56.21f,5.76f},{39.25f,-56.08f,6.10f},{39.01f,-55.92f,6.42f},{38.73f,-55.64f,6.75f},{38.46f,-55.36f,7.02f},{38.10f,-54.96f,7.29f},{37.70f,-54.49f,7.72f},{37.31f,-54.06f,8.31f},{36.84f,-53.52f,8.91f},{36.40f,-52.96f,9.49f},{35.83f,-52.20f,10.18f},{35.38f,-51.57f,10.69f},{34.84f,-50.79f,11.26f},{34.27f,-49.92f,11.80f},{33.76f,-49.09f,12.24f},{33.48f,-48.53f,12.53f},{33.36f,-48.13f,12.64f},{33.42f,-47.79f,12.51f},{33.53f,-47.30f,12.13f},{33.59f,-46.98f,11.82f},{33.70f,-46.48f,11.30f},{33.82f,-45.95f,10.74f},{33.94f,-45.41f,10.14f},{34.04f,-44.91f,9.59f},{34.17f,-44.41f,9.08f},{34.24f,-44.13f,8.72f},{34.23f,-43.92f,8.35f},{34.18f,-43.74f,8.02f},{34.09f,-43.58f,7.76f},{34.06f,-43.35f,7.64f},{34.13f,-43.44f,7.77f},{34.20f,-43.43f,7.87f},{34.31f,-43.43f,7.89f},{34.51f,-43.32f,7.72f},{34.73f,-43.18f,7.48f},{34.95f,-43.02f,7.23f},{35.16f,-42.85f,6.99f},{35.37f,-42.69f,6.77f},{35.49f,-42.56f,6.64f},{35.58f,-42.42f,6.55f},{35.67f,-42.29f,6.52f},{35.74f,-42.13f,6.48f},{35.80f,-41.99f,6.44f},{35.83f,-41.82f,6.32f},{35.85f,-41.67f,6.15f},{35.85f,-41.53f,5.95f},{35.81f,-41.36f,5.71f},{35.75f,-41.20f,5.47f},{35.67f,-41.02f,5.19f},{35.52f,-40.77f,4.80f},{35.41f,-40.61f,4.57f},{35.28f,-40.43f,4.31f},{35.12f,-40.23f,4.02f},{34.93f,-39.99f,3.73f},{34.71f,-39.68f,3.45f},{34.46f,-39.35f,3.19f},{34.22f,-38.98f,2.94f},{34.00f,-38.77f,2.69f},{33.80f,-38.68f,2.38f},{33.89f,-38.56f,2.72f},{34.12f,-38.42f,3.38f},{34.36f,-38.27f,4.19f},{34.30f,-38.01f,4.63f},{34.06f,-37.66f,4.89f},{33.71f,-37.27f,4.97f},{33.52f,-37.07f,4.93f},{32.21f,-37.84f,4.06f}};
static const float kSlideR_hold[92][3]={{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f},{-20.78f,-48.60f,-35.72f}};
static const float kSlideR_rec[92][3]={{-19.36f,-49.82f,-35.73f},{-20.78f,-48.60f,-35.72f},{-19.19f,-49.76f,-35.10f},{0.14f,-57.25f,-36.84f},{12.04f,-63.04f,-31.05f},{24.42f,-66.45f,-22.22f},{33.83f,-66.03f,-12.29f},{39.29f,-63.16f,-4.49f},{40.82f,-60.82f,-1.43f},{40.56f,-59.37f,-1.02f},{40.40f,-58.04f,-0.71f},{40.84f,-57.67f,-0.27f},{41.04f,-57.30f,-0.17f},{41.02f,-57.28f,0.09f},{41.35f,-57.13f,0.14f},{41.50f,-57.31f,1.10f},{41.58f,-57.50f,2.50f},{41.60f,-57.09f,3.45f},{41.28f,-56.79f,3.85f},{40.77f,-56.87f,3.80f},{40.57f,-56.75f,4.13f},{40.36f,-56.57f,4.38f},{40.11f,-56.42f,4.69f},{39.96f,-56.38f,5.04f},{39.77f,-56.32f,5.43f},{39.52f,-56.21f,5.76f},{39.25f,-56.08f,6.10f},{39.01f,-55.92f,6.42f},{38.73f,-55.64f,6.75f},{38.46f,-55.36f,7.02f},{38.10f,-54.96f,7.29f},{37.70f,-54.49f,7.72f},{37.31f,-54.06f,8.31f},{36.84f,-53.52f,8.91f},{36.40f,-52.96f,9.49f},{35.83f,-52.20f,10.18f},{35.38f,-51.57f,10.69f},{34.84f,-50.79f,11.26f},{34.27f,-49.92f,11.80f},{33.76f,-49.09f,12.24f},{33.48f,-48.53f,12.53f},{33.36f,-48.13f,12.64f},{33.42f,-47.79f,12.51f},{33.53f,-47.30f,12.13f},{33.59f,-46.98f,11.82f},{33.70f,-46.48f,11.30f},{33.82f,-45.95f,10.74f},{33.94f,-45.41f,10.14f},{34.04f,-44.91f,9.59f},{34.17f,-44.41f,9.08f},{34.24f,-44.13f,8.72f},{34.23f,-43.92f,8.35f},{34.18f,-43.74f,8.02f},{34.09f,-43.58f,7.76f},{34.06f,-43.35f,7.64f},{34.13f,-43.44f,7.77f},{34.20f,-43.43f,7.87f},{34.31f,-43.43f,7.89f},{34.51f,-43.32f,7.72f},{34.73f,-43.18f,7.48f},{34.95f,-43.02f,7.23f},{35.16f,-42.85f,6.99f},{35.37f,-42.69f,6.77f},{35.49f,-42.56f,6.64f},{35.58f,-42.42f,6.55f},{35.67f,-42.29f,6.52f},{35.74f,-42.13f,6.48f},{35.80f,-41.99f,6.44f},{35.83f,-41.82f,6.32f},{35.85f,-41.67f,6.15f},{35.85f,-41.53f,5.95f},{35.81f,-41.36f,5.71f},{35.75f,-41.20f,5.47f},{35.67f,-41.02f,5.19f},{35.52f,-40.77f,4.80f},{35.41f,-40.61f,4.57f},{35.28f,-40.43f,4.31f},{35.12f,-40.23f,4.02f},{34.93f,-39.99f,3.73f},{34.71f,-39.68f,3.45f},{34.46f,-39.35f,3.19f},{34.22f,-38.98f,2.94f},{34.00f,-38.77f,2.69f},{33.80f,-38.68f,2.38f},{33.89f,-38.56f,2.72f},{34.12f,-38.42f,3.38f},{34.36f,-38.27f,4.19f},{34.30f,-38.01f,4.63f},{34.06f,-37.66f,4.89f},{33.71f,-37.27f,4.97f},{33.52f,-37.07f,4.93f},{32.27f,-37.82f,4.31f}};
static const float kSlideR_whd_native[92]={-7.8f,-3.8f,1.5f,7.4f,11.0f,12.7f,13.5f,13.9f,14.0f,13.9f,14.1f,14.5f,14.9f,14.9f,15.4f,15.2f,14.6f,14.1f,13.6f,13.2f,12.8f,12.4f,11.6f,11.0f,10.1f,9.1f,8.2f,7.3f,5.9f,4.7f,3.6f,2.3f,0.8f,-0.7f,-2.0f,-3.6f,-5.1f,-7.8f,-11.5f,-15.5f,-19.8f,-24.5f,-29.5f,-34.1f,-36.2f,-38.5f,-39.7f,-40.0f,-39.8f,-39.4f,-39.2f,-38.8f,-38.4f,-37.9f,-37.3f,-36.7f,-36.3f,-35.8f,-35.3f,-34.9f,-34.8f,-34.9f,-35.2f,-35.4f,-35.7f,-36.0f,-36.3f,-36.6f,-36.9f,-37.2f,-37.5f,-37.7f,-37.9f,-38.0f,-38.0f,-38.0f,-38.0f,-37.8f,-37.7f,-37.4f,-37.1f,-36.7f,-36.4f,-36.3f,-35.9f,-35.4f,-35.0f,-34.5f,-34.1f,-33.8f,-33.7f,-35.0f};
static const float kSlideR_whd_hold[92]={-23.7f,-22.5f,-21.7f,-17.9f,-17.4f,-16.9f,-16.1f,-15.3f,-14.8f,-14.2f,-13.9f,-13.7f,-13.7f,-13.6f,-13.8f,-14.0f,-14.1f,-14.2f,-14.8f,-15.2f,-15.3f,-15.5f,-16.0f,-16.4f,-16.8f,-17.3f,-17.5f,-17.8f,-18.2f,-18.7f,-19.4f,-20.1f,-20.6f,-21.2f,-21.7f,-22.3f,-23.5f,-27.4f,-33.8f,-40.7f,-47.6f,-54.4f,-60.5f,-64.8f,-66.2f,-67.1f,-67.3f,-67.3f,-67.3f,-67.3f,-67.4f,-67.5f,-67.5f,-67.6f,-67.7f,-67.7f,-67.7f,-67.6f,-67.4f,-67.3f,-67.3f,-67.3f,-67.3f,-67.4f,-67.4f,-67.5f,-67.5f,-67.5f,-67.3f,-67.1f,-66.9f,-66.7f,-66.6f,-66.6f,-66.6f,-66.6f,-66.7f,-66.9f,-67.1f,-67.4f,-67.7f,-68.1f,-68.5f,-68.9f,-69.4f,-69.8f,-70.2f,-70.5f,-70.8f,-71.0f,-71.2f,-71.1f};
// N37 03:39:58.65 k11 (M_Neutral_Slide_FootOut_Into_Lfoot) edge at index 2
static const float kSlideL_dt[92]={0.01793f,0.01889f,0.01793f,0.01675f,0.01467f,0.01567f,0.01630f,0.01647f,0.01699f,0.01791f,0.01552f,0.01639f,0.01736f,0.01639f,0.01624f,0.01868f,0.01500f,0.01622f,0.01687f,0.01723f,0.01605f,0.01912f,0.01429f,0.01670f,0.01683f,0.01676f,0.01640f,0.01642f,0.01813f,0.01480f,0.01691f,0.01682f,0.01717f,0.01602f,0.01669f,0.01806f,0.01597f,0.01614f,0.01703f,0.01710f,0.01652f,0.01797f,0.01466f,0.01695f,0.01660f,0.01726f,0.01643f,0.01640f,0.01778f,0.01528f,0.01675f,0.01649f,0.01678f,0.01677f,0.01797f,0.01519f,0.01697f,0.01624f,0.01735f,0.01609f,0.01910f,0.01456f,0.01651f,0.01686f,0.01721f,0.01658f,0.01815f,0.01514f,0.02235f,0.01081f,0.01810f,0.01512f,0.01665f,0.01666f,0.01612f,0.01688f,0.01734f,0.01868f,0.01558f,0.01650f,0.01617f,0.01677f,0.01688f,0.01610f,0.01846f,0.01555f,0.01554f,0.01722f,0.01650f,0.01629f,0.01658f,0.01699f};
static const unsigned char kSlideL_slide[92]={1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static const float kSlideL_roll[92]={35.15f,34.99f,35.15f,36.49f,36.44f,36.39f,36.37f,36.36f,36.35f,36.21f,36.13f,36.00f,35.60f,35.15f,34.69f,34.14f,33.59f,33.08f,32.77f,32.64f,32.96f,33.48f,34.15f,34.91f,35.39f,35.84f,35.50f,33.69f,30.08f,26.31f,21.41f,14.85f,8.42f,3.51f,0.25f,2.00f,1.85f,1.21f,0.53f,0.00f,0.38f,0.56f,0.53f,0.23f,0.36f,1.28f,2.25f,3.13f,3.41f,3.02f,2.06f,0.82f,0.63f,1.96f,3.34f,4.19f,5.05f,5.68f,6.27f,6.71f,7.05f,7.13f,7.16f,6.99f,6.82f,6.64f,6.44f,6.28f,6.10f,5.88f,5.63f,5.38f,5.10f,4.76f,4.36f,3.90f,3.34f,2.73f,2.11f,1.47f,0.76f,0.05f,0.57f,1.11f,1.43f,1.56f,1.65f,1.70f,1.78f,1.97f,2.16f,2.37f};
static const float kSlideL_native[92][3]={{22.30f,-39.49f,0.74f},{26.29f,-44.12f,0.20f},{32.22f,-51.45f,-0.27f},{37.72f,-58.79f,-0.63f},{40.88f,-63.21f,-1.14f},{41.89f,-64.40f,-1.45f},{41.74f,-63.60f,-1.45f},{41.35f,-62.23f,-1.40f},{40.92f,-60.76f,-1.27f},{40.56f,-59.29f,-0.98f},{40.47f,-58.19f,-0.66f},{40.83f,-57.68f,-0.27f},{41.05f,-57.30f,-0.17f},{41.02f,-57.28f,0.10f},{41.35f,-57.13f,0.13f},{41.51f,-57.35f,1.17f},{41.58f,-57.48f,2.39f},{41.61f,-57.11f,3.44f},{41.30f,-56.80f,3.83f},{40.77f,-56.87f,3.80f},{40.57f,-56.75f,4.13f},{40.34f,-56.54f,4.40f},{40.12f,-56.45f,4.66f},{39.97f,-56.38f,5.04f},{39.77f,-56.32f,5.43f},{39.52f,-56.21f,5.76f},{39.26f,-56.09f,6.09f},{39.01f,-55.92f,6.41f},{38.74f,-55.65f,6.75f},{38.47f,-55.37f,7.01f},{38.11f,-54.98f,7.29f},{37.73f,-54.53f,7.68f},{37.31f,-54.07f,8.30f},{36.87f,-53.56f,8.87f},{36.40f,-52.96f,9.49f},{35.87f,-52.25f,10.14f},{35.38f,-51.58f,10.69f},{34.86f,-50.81f,11.24f},{34.32f,-49.99f,11.75f},{33.78f,-49.12f,12.23f},{33.48f,-48.53f,12.53f},{33.36f,-48.11f,12.64f},{33.41f,-47.84f,12.54f},{33.50f,-47.45f,12.26f},{33.59f,-47.00f,11.84f},{33.70f,-46.49f,11.31f},{33.82f,-45.96f,10.75f},{33.93f,-45.44f,10.17f},{34.04f,-44.88f,9.56f},{34.16f,-44.47f,9.14f},{34.23f,-44.15f,8.75f},{34.23f,-43.93f,8.38f},{34.19f,-43.76f,8.04f},{34.10f,-43.59f,7.78f},{34.06f,-43.36f,7.65f},{34.10f,-43.44f,7.74f},{34.19f,-43.43f,7.86f},{34.29f,-43.45f,7.89f},{34.51f,-43.32f,7.72f},{34.71f,-43.19f,7.50f},{34.96f,-43.02f,7.23f},{35.16f,-42.86f,7.00f},{35.34f,-42.72f,6.80f},{35.49f,-42.56f,6.65f},{35.58f,-42.42f,6.55f},{35.66f,-42.29f,6.52f},{35.74f,-42.12f,6.48f},{35.80f,-41.99f,6.45f},{35.83f,-41.82f,6.32f},{35.85f,-41.68f,6.16f},{35.85f,-41.52f,5.94f},{35.81f,-41.37f,5.72f},{35.75f,-41.20f,5.47f},{35.67f,-41.03f,5.20f},{35.56f,-40.85f,4.92f},{35.43f,-40.65f,4.63f},{35.28f,-40.44f,4.32f},{35.11f,-40.22f,4.00f},{34.92f,-39.98f,3.72f},{34.70f,-39.67f,3.44f},{34.45f,-39.34f,3.18f},{34.22f,-38.99f,2.95f},{33.99f,-38.76f,2.67f},{33.80f,-38.69f,2.38f},{33.90f,-38.55f,2.74f},{34.13f,-38.42f,3.41f},{34.36f,-38.27f,4.19f},{34.30f,-38.01f,4.63f},{34.06f,-37.65f,4.90f},{33.71f,-37.28f,4.97f},{33.53f,-37.07f,4.93f},{33.51f,-37.18f,4.65f}};
static const float kSlideL_hold[92][3]={{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f},{-11.73f,-49.87f,-41.23f}};
static const float kSlideL_rec[92][3]={{-12.26f,-49.74f,-41.04f},{-11.73f,-49.87f,-41.23f},{-9.29f,-51.89f,-39.97f},{2.41f,-56.71f,-38.47f},{13.24f,-61.91f,-33.19f},{25.13f,-65.44f,-24.47f},{34.56f,-65.52f,-13.68f},{39.41f,-63.24f,-5.41f},{40.82f,-60.83f,-1.48f},{40.56f,-59.29f,-0.98f},{40.47f,-58.19f,-0.66f},{40.83f,-57.68f,-0.27f},{41.05f,-57.30f,-0.17f},{41.02f,-57.28f,0.10f},{41.35f,-57.13f,0.13f},{41.51f,-57.35f,1.17f},{41.58f,-57.48f,2.39f},{41.61f,-57.11f,3.44f},{41.30f,-56.80f,3.83f},{40.77f,-56.87f,3.80f},{40.57f,-56.75f,4.13f},{40.34f,-56.54f,4.40f},{40.12f,-56.45f,4.66f},{39.97f,-56.38f,5.04f},{39.77f,-56.32f,5.43f},{39.52f,-56.21f,5.76f},{39.26f,-56.09f,6.09f},{39.01f,-55.92f,6.41f},{38.74f,-55.65f,6.75f},{38.47f,-55.37f,7.01f},{38.11f,-54.98f,7.29f},{37.73f,-54.53f,7.68f},{37.31f,-54.07f,8.30f},{36.87f,-53.56f,8.87f},{36.40f,-52.96f,9.49f},{35.87f,-52.25f,10.14f},{35.38f,-51.58f,10.69f},{34.86f,-50.81f,11.24f},{34.32f,-49.99f,11.75f},{33.78f,-49.12f,12.23f},{33.48f,-48.53f,12.53f},{33.36f,-48.11f,12.64f},{33.41f,-47.84f,12.54f},{33.50f,-47.45f,12.26f},{33.59f,-47.00f,11.84f},{33.70f,-46.49f,11.31f},{33.82f,-45.96f,10.75f},{33.93f,-45.44f,10.17f},{34.04f,-44.88f,9.56f},{34.16f,-44.47f,9.14f},{34.23f,-44.15f,8.75f},{34.23f,-43.93f,8.38f},{34.19f,-43.76f,8.04f},{34.10f,-43.59f,7.78f},{34.06f,-43.36f,7.65f},{34.10f,-43.44f,7.74f},{34.19f,-43.43f,7.86f},{34.29f,-43.45f,7.89f},{34.51f,-43.32f,7.72f},{34.71f,-43.19f,7.50f},{34.96f,-43.02f,7.23f},{35.16f,-42.86f,7.00f},{35.34f,-42.72f,6.80f},{35.49f,-42.56f,6.65f},{35.58f,-42.42f,6.55f},{35.66f,-42.29f,6.52f},{35.74f,-42.12f,6.48f},{35.80f,-41.99f,6.45f},{35.83f,-41.82f,6.32f},{35.85f,-41.68f,6.16f},{35.85f,-41.52f,5.94f},{35.81f,-41.37f,5.72f},{35.75f,-41.20f,5.47f},{35.67f,-41.03f,5.20f},{35.56f,-40.85f,4.92f},{35.43f,-40.65f,4.63f},{35.28f,-40.44f,4.32f},{35.11f,-40.22f,4.00f},{34.92f,-39.98f,3.72f},{34.70f,-39.67f,3.44f},{34.45f,-39.34f,3.18f},{34.22f,-38.99f,2.95f},{33.99f,-38.76f,2.67f},{33.80f,-38.69f,2.38f},{33.90f,-38.55f,2.74f},{34.13f,-38.42f,3.41f},{34.36f,-38.27f,4.19f},{34.30f,-38.01f,4.63f},{34.06f,-37.65f,4.90f},{33.71f,-37.28f,4.97f},{33.53f,-37.07f,4.93f},{33.54f,-37.16f,4.90f}};
static const float kSlideL_whd_native[92]={-8.9f,-6.1f,-1.9f,2.3f,4.5f,5.1f,4.8f,4.3f,4.3f,4.2f,3.7f,3.2f,2.8f,2.2f,1.9f,0.8f,-0.1f,-0.7f,-1.0f,-1.1f,-0.7f,-0.1f,0.2f,0.7f,1.0f,1.2f,0.9f,-0.6f,-3.8f,-7.7f,-13.3f,-21.5f,-29.8f,-36.1f,-40.3f,-42.2f,-41.7f,-40.4f,-38.8f,-37.2f,-36.1f,-35.3f,-35.3f,-35.7f,-36.7f,-37.9f,-39.0f,-39.8f,-40.2f,-40.0f,-39.5f,-39.3f,-39.2f,-39.2f,-39.1f,-39.2f,-39.3f,-39.2f,-39.0f,-38.8f,-38.6f,-38.5f,-38.4f,-38.4f,-38.4f,-38.3f,-38.2f,-38.2f,-38.1f,-37.9f,-37.8f,-37.7f,-37.4f,-37.2f,-37.0f,-36.7f,-36.4f,-36.0f,-35.7f,-35.3f,-35.0f,-34.6f,-34.5f,-34.5f,-34.3f,-34.2f,-34.0f,-33.8f,-33.6f,-33.5f,-33.4f,-33.7f};
static const float kSlideL_whd_hold[92]={-13.3f,-13.8f,-13.8f,-12.4f,-12.7f,-12.9f,-13.1f,-13.3f,-13.1f,-13.2f,-13.5f,-14.1f,-14.9f,-15.8f,-16.8f,-17.8f,-18.6f,-19.3f,-19.6f,-19.5f,-18.6f,-17.4f,-16.1f,-14.7f,-13.6f,-12.5f,-12.9f,-15.8f,-22.1f,-28.7f,-37.1f,-47.6f,-56.3f,-61.6f,-64.5f,-65.5f,-65.7f,-65.7f,-65.6f,-65.4f,-65.3f,-65.2f,-65.3f,-65.6f,-66.1f,-66.7f,-67.0f,-67.2f,-67.0f,-66.7f,-66.2f,-65.5f,-64.6f,-63.6f,-62.6f,-61.9f,-61.2f,-60.7f,-60.2f,-59.9f,-59.7f,-59.7f,-59.7f,-59.8f,-59.9f,-60.0f,-60.1f,-60.2f,-60.3f,-60.4f,-60.6f,-60.8f,-61.1f,-61.4f,-61.7f,-62.2f,-62.6f,-63.2f,-63.7f,-64.2f,-64.8f,-65.3f,-65.8f,-66.2f,-66.4f,-66.4f,-66.4f,-66.4f,-66.3f,-66.4f,-66.5f,-66.6f};


struct Fixture{const char* name;const float* dt;const unsigned char* slide;const float* roll;const float (*native)[3];const float (*hold)[3];
               const float (*rec)[3];const float* whdNative;const float* whdHold;};
static const int kN=92;

struct Out{float d05=0,d10=0,whdMax=-1e9f,holdAtEdge=0,holdMaxStep=0;float u[kN];float h[kN];};
static Out Simulate(const Fixture& f)
{
    Out o;UpperEnvelope env;SlideRollHold roll;float t[kN],pos[kN][3],whd[kN];float clock=0,prevHold=0;
    for(int i=0;i<kN;++i)
    {
        clock+=f.dt[i];t[i]=clock;
        const bool reload=i>=2;
        env.Step(f.dt[i],reload,true);
        const float h=roll.Step(f.dt[i],reload,f.slide[i]!=0,f.roll[i]);
        const float u=std::fmax(1.0f-AuthoredFadeWeight(env.weight),h); // MotionDriver reloadScale
        const float w=1.0f-u;                                          // native share
        o.u[i]=u;o.h[i]=h;
        for(int k=0;k<3;++k)pos[i][k]=f.hold[i][k]+(f.native[i][k]-f.hold[i][k])*w;
        whd[i]=f.whdHold[i]+(f.whdNative[i]-f.whdHold[i])*w;
        if(i==2)o.holdAtEdge=h;
        if(i>2)o.holdMaxStep=std::fmax(o.holdMaxStep,std::fabs(h-prevHold)/f.dt[i]);
        prevHold=h;
    }
    const float edge=t[2];
    for(int a=1;a<kN;++a)
    {
        if(t[a]-edge<-.05f||t[a]-edge>1.1f)continue;
        o.whdMax=std::fmax(o.whdMax,whd[a]);
        for(int b=a+1;b<kN;++b)
        {
            const float dx=pos[b][0]-pos[a][0],dy=pos[b][1]-pos[a][1],dz=pos[b][2]-pos[a][2];
            const float d=std::sqrt(dx*dx+dy*dy+dz*dz);
            if(t[b]-t[a]<=.05f+1e-6f)o.d05=std::fmax(o.d05,d);
            if(t[b]-t[a]<=.10f+1e-6f)o.d10=std::fmax(o.d10,d);
        }
    }
    return o;
}
// The recorded (pre-fix) trajectory: stage 2 weapon in the Spine3 frame, and weapon above the head for native arms.
static void Recorded(const Fixture& f,float& d05,float& whdNativeMax)
{
    float t[kN],clock=0;d05=0;whdNativeMax=-1e9f;
    for(int i=0;i<kN;++i){clock+=f.dt[i];t[i]=clock;}
    for(int a=1;a<kN;++a)
    {
        if(t[a]-t[2]<-.05f||t[a]-t[2]>1.1f)continue;
        if(t[a]-t[2]>=.05f&&t[a]-t[2]<=.60f)whdNativeMax=std::fmax(whdNativeMax,f.whdNative[a]); // rolled phase
        for(int b=a+1;b<kN&&t[b]-t[a]<=.05f+1e-6f;++b)
        {
            const float dx=f.rec[b][0]-f.rec[a][0],dy=f.rec[b][1]-f.rec[a][1],dz=f.rec[b][2]-f.rec[a][2];
            d05=std::fmax(d05,std::sqrt(dx*dx+dy*dy+dz*dz));
        }
    }
}

// ---- 3. torso counter-roll ---------------------------------------------------------------------------------------------
namespace sd=utpp::mm::slide_deroll;
namespace om=utpp::mm::math;
namespace ovn=utpp::mm::ov;
enum{kRoot,kHips,kSpine,kSpine1,kSpine2,kSpine3,kLSh,kRSh,kNeck,kHead,kLArm,kRArm,kLUpLeg,kBones};
static om::Quat AxisAngleDeg(float x,float y,float z,float deg)
{
    const float l=std::sqrt(x*x+y*y+z*z),h=.5f*deg*.01745329f,s=std::sin(h)/l;
    return {x*s,y*s,z*s,std::cos(h)};
}
// CP-like torso (Z up, forward +Y, V's left = -X), the whole model yawed 55deg and the spine rolled (+ a little pitch) by rollDeg.
static void RolledRig(ovn::Skeleton& sk,ovn::Pose& p,sd::Bones& b,float rollDeg)
{
    const int16_t parent[kBones]={-1,kRoot,kHips,kSpine,kSpine1,kSpine2,kSpine3,kSpine3,kSpine3,kNeck,kLSh,kRSh,kHips};
    const float t[kBones][3]={{0,0,0},{0,0,1.0f},{0,0,.08f},{0,0,.10f},{0,0,.10f},{0,0,.12f},{-.017f,.03f,.14f},{.017f,.03f,.14f},
                              {0,0,.18f},{0,0,.10f},{-.15f,0,0},{.15f,0,0},{-.1f,0,-.05f}};
    sk.n=kBones;p.n=kBones;
    for(int i=0;i<kBones;++i){sk.parent[i]=parent[i];p.b[i]=ovn::Xf{};p.b[i].t={t[i][0],t[i][1],t[i][2]};}
    p.b[kHips].r=AxisAngleDeg(0,0,1,55.0f);
    p.b[kSpine1].r=om::QuatMul(AxisAngleDeg(0,1,0,rollDeg*.40f),AxisAngleDeg(1,0,0,6.0f));
    p.b[kSpine2].r=om::QuatMul(AxisAngleDeg(0,1,0,rollDeg*.35f),AxisAngleDeg(1,0,0,4.0f));
    p.b[kSpine3].r=AxisAngleDeg(0,1,0,rollDeg*.25f);
    p.b[kHead].r=AxisAngleDeg(0,0,1,20.0f);
    b.spine[0]=kSpine1;b.spine[1]=kSpine2;b.spine[2]=kSpine3;b.lsh=kLSh;b.rsh=kRSh;b.neck=kNeck;
}
static float LineYawDeg(const ovn::Skeleton& sk,const ovn::Pose& p,const sd::Bones& b)
{
    om::Vec3 l,r;sd::ShoulderLine(sk,p,b,l,r);return std::atan2(r.y-l.y,r.x-l.x)*57.29578f;
}
static float TiltFromVerticalDeg(const ovn::Skeleton& sk,const ovn::Pose& p,int from,int to)
{
    om::Quat q;om::Vec3 a,c;sd::ModelOf(sk,p,from,q,a);sd::ModelOf(sk,p,to,q,c);
    const om::Vec3 d=c-a;return std::acos(std::clamp(d.z/std::sqrt(om::Dot(d,d)),-1.0f,1.0f))*57.29578f;
}
static bool SameBones(const ovn::Pose& a,const ovn::Pose& b,int skip0=-1,int skip1=-1,int skip2=-1)
{
    for(int i=0;i<a.n;++i)
        if(i!=skip0&&i!=skip1&&i!=skip2&&std::memcmp(&a.b[i],&b.b[i],sizeof(ovn::Xf))!=0)return false;
    return true;
}
static float Lean(const ovn::Skeleton& sk,const ovn::Pose& p,const sd::Bones& b){float l=-999.0f;sd::LeanDeg(sk,p,b,l);return l;}
static void DeRollGeometry()
{
    ovn::Skeleton sk;ovn::Pose raw;sd::Bones b;RolledRig(sk,raw,b,36.0f);
    const float before=sd::RollDeg(sk,raw,b);
    std::printf("  synthetic rig: shoulder roll %.2fdeg, line yaw %.2fdeg, lean %.2fdeg\n",before,LineYawDeg(sk,raw,b),Lean(sk,raw,b));
    Check(b.Valid(sk)&&b.NeckValid(sk)&&before>30.0f&&before<42.0f,"synthetic slide torso: shoulder line rolled 30..42deg");
    {
        om::Vec3 l,r;sd::ShoulderLine(sk,raw,b,l,r);
        Check(sd::RollDeg(sk,raw,b)==ShoulderLineRollDeg(l.x,l.y,l.z,r.x,r.y,r.z),"RollDeg is the hold's shoulder-line proxy");
    }
    // the lean of the levelled (roll only) torso, far from the standing target like the recorded slides (-2 / +0.3deg)
    ovn::Pose rollOnly=raw;
    {
        om::Vec3 l,r;sd::ShoulderLine(sk,raw,b,l,r);const om::Vec3 d=r-l,dh{d.x,d.y,0.0f},ax=om::Cross(d,dh);
        sd::RotateChain(sk,rollOnly,b,ax*(1.0f/std::sqrt(om::Dot(ax,ax))),std::asin(std::fabs(d.z)/std::sqrt(om::Dot(d,d))));
    }
    const float leanLevel=Lean(sk,rollOnly,b);
    Check(sd::RollDeg(sk,rollOnly,b)<.01f&&std::fabs(leanLevel-sd::kDePitchTargetDeg)>10.0f,"levelled synthetic torso leans >10deg off the standing target");
    ovn::Pose p=raw;
    const sd::Result res=sd::Apply(sk,p,b,1.0f);const float after=sd::RollDeg(sk,p,b),leanAfter=Lean(sk,p,b);
    std::printf("  upright w=1: roll removed %.2fdeg (left %.3fdeg), lean %.2f -> %.2fdeg (target %.1f, pitch removed %.2fdeg), yaw %.2f -> %.2fdeg,"
                " Spine3->Head tilt %.1f -> %.1fdeg\n",res.rollDeg,after,leanLevel,leanAfter,sd::kDePitchTargetDeg,res.pitchDeg,LineYawDeg(sk,raw,b),
                LineYawDeg(sk,p,b),TiltFromVerticalDeg(sk,raw,kSpine3,kHead),TiltFromVerticalDeg(sk,p,kSpine3,kHead));
    Check(after<2.0f&&std::fabs(res.rollDeg-before)<.01f,"full de-roll levels the shoulder line (< 2deg left)");
    Check(std::fabs(leanAfter-sd::kDePitchTargetDeg)<1.0f,"de-pitch: Spine3->Neck lean reaches the standing target within 1deg");
    Check(after<.05f,"de-pitch keeps the shoulder line level (pitch about the line itself)");
    Check(std::fabs(LineYawDeg(sk,p,b)-LineYawDeg(sk,raw,b))<.5f,"uprighting keeps the yaw of the shoulder line");
    Check(SameBones(raw,p,kSpine1,kSpine2,kSpine3),"only Spine1..Spine3 change (Hips, Spine, legs, clavicles, neck, head, arms bit-identical)");
    bool tOk=true;for(int k=0;k<3;++k)tOk=tOk&&std::memcmp(&raw.b[b.spine[k]].t,&p.b[b.spine[k]].t,sizeof(om::Vec3))==0&&
                                         std::memcmp(&raw.b[b.spine[k]].s,&p.b[b.spine[k]].s,sizeof(om::Vec3))==0;
    Check(tOk,"spine translations / scales untouched (rotation only)");
    Check(TiltFromVerticalDeg(sk,p,kSpine3,kHead)<TiltFromVerticalDeg(sk,raw,kSpine3,kHead)-10.0f,"neck / head follow the uprighted torso as children");
    for(const float w:{.25f,.5f,.8f})
    {
        ovn::Pose q=raw;sd::Apply(sk,q,b,w);const float lq=Lean(sk,q,b);
        char text[160];std::snprintf(text,sizeof text,"partial w=%.2f leaves (1-w) of the roll, lean moves towards the target (%.2fdeg)",w,lq);
        Check(std::fabs(sd::RollDeg(sk,q,b)-before*(1.0f-w))<.05f&&std::fabs(lq-sd::kDePitchTargetDeg)<std::fabs(leanLevel-sd::kDePitchTargetDeg),text);
    }
    // split evenly: each of Spine1..Spine3 gains a third of a chain rotation
    {
        om::Vec3 l,r;sd::ShoulderLine(sk,raw,b,l,r);const om::Vec3 d=r-l,dh{d.x,d.y,0.0f},ax=om::Cross(d,dh);
        ovn::Pose q=raw;sd::RotateChain(sk,q,b,ax*(1.0f/std::sqrt(om::Dot(ax,ax))),before*.01745329f);
        ovn::Pose s1=raw;s1.b[kSpine1]=q.b[kSpine1];
        ovn::Pose s12=s1;s12.b[kSpine2]=q.b[kSpine2];
        Check(std::fabs(sd::RollDeg(sk,s1,b)-before*2.0f/3.0f)<.05f&&std::fabs(sd::RollDeg(sk,s12,b)-before/3.0f)<.05f,
              "the rotation is split evenly over Spine1 / Spine2 / Spine3 (a third each)");
    }
    {
        ovn::Pose q=raw;
        const bool c0=sd::Apply(sk,q,b,0.0f).Changed(),c1=sd::Apply(sk,q,b,std::numeric_limits<float>::quiet_NaN()).Changed(),
                   c2=sd::Apply(sk,q,b,-1.0f).Changed();
        Check(!c0&&!c1&&!c2&&SameBones(raw,q),"weight 0 / NaN / negative: pose bit-identical");
        sd::Bones bad=b;bad.spine[1]=kNeck;
        Check(!sd::Apply(sk,q,bad,1.0f).Changed()&&SameBones(raw,q)&&sd::RollDeg(sk,q,bad)==0.0f,"broken Spine chain: no edit, roll 0");
        sd::Bones noNeck=b;noNeck.neck=-1;ovn::Pose nn=raw;const sd::Result rn=sd::Apply(sk,nn,noNeck,1.0f);
        Check(rn.pitchDeg==0.0f&&std::fabs(rn.rollDeg-before)<.01f,"no Neck bone: de-roll only");
        ovn::Skeleton lvlSk;ovn::Pose lvl;sd::Bones lb;RolledRig(lvlSk,lvl,lb,0.0f);
        lvl.b[kSpine1].r=lvl.b[kSpine2].r=om::Quat{};
        const sd::Result rl=sd::Apply(lvlSk,lvl,lb,1.0f);
        Check(rl.rollDeg==0.0f&&sd::RollDeg(lvlSk,lvl,lb)<.01f&&std::fabs(Lean(lvlSk,lvl,lb)-sd::kDePitchTargetDeg)<1.0f,
              "already level line: no roll edit, only the lean is set");
    }
}
static void DeRollWeightRates()
{
    for(const float hz:{30.0f,60.0f,144.0f})
    {
        const float dt=1.0f/hz;char text[200];
        SlideDeRoll d;float prev=0,maxUp=0,tFull=-1,t=0;bool mono=true;
        for(int i=0;i<int(hz)&&tFull<0;++i){t+=dt;const float w=d.Step(dt,true,36.0f);maxUp=std::fmax(maxUp,w-prev);mono=mono&&w>=prev;prev=w;if(w>=1.0f)tFull=t;}
        std::snprintf(text,sizeof text,"%.0f Hz: de-roll weight ramps in over %.2fs (%.3fs) monotone, max step %.3f <= %.3f",hz,kDeRollAttackSeconds,tFull,
                      maxUp,1.5f*dt/kDeRollAttackSeconds);
        Check(mono&&tFull>=kDeRollAttackSeconds-1e-4f&&tFull<=kDeRollAttackSeconds+dt+1e-4f&&maxUp<=1.5f*dt/kDeRollAttackSeconds+1e-5f,text);
        // slide exit during the reload: the roll falls back to 0, the latched peak keeps the torso level (no roll-back bump)
        bool held=true;for(int i=0;i<int(.3f*hz);++i){const float roll=std::fmax(0.0f,36.0f-36.0f*(i+1)*dt/.15f);held=held&&d.Step(dt,true,roll)==1.0f;}
        std::snprintf(text,sizeof text,"%.0f Hz: torso rising out of the slide keeps the full weight (roll left = raw roll * 0)",hz);
        Check(held,text);
        float maxDown=0;prev=d.weight;float tZero=-1;t=0;mono=true;
        for(int i=0;i<int(hz)&&tZero<0;++i){t+=dt;const float w=d.Step(dt,false,36.0f);maxDown=std::fmax(maxDown,prev-w);mono=mono&&w<=prev;prev=w;if(w==0.0f)tZero=t;}
        std::snprintf(text,sizeof text,"%.0f Hz: release over %.2fs (%.3fs) monotone, max step %.3f <= %.3f, idle after",hz,kDeRollReleaseSeconds,tZero,maxDown,
                      1.5f*dt/kDeRollReleaseSeconds);
        Check(mono&&tZero>=kDeRollReleaseSeconds-1e-4f&&tZero<=kDeRollReleaseSeconds+dt+1e-4f&&maxDown<=1.5f*dt/kDeRollReleaseSeconds+1e-5f&&
              d.Idle()&&d.peak==0.0f,text);
        // the roll range scales the weight (same 22..32 as the hold): 27deg -> half, below 22 -> nothing
        SlideDeRoll h;for(int i=0;i<int(hz);++i)h.Step(dt,true,27.0f);
        SlideDeRoll lo;bool loZero=true;for(int i=0;i<int(hz);++i)loZero=loZero&&lo.Step(dt,true,21.9f)==0.0f&&lo.Idle();
        std::snprintf(text,sizeof text,"%.0f Hz: weight follows the roll range (27deg -> %.3f, < 22deg -> 0, pose untouched)",hz,h.weight);
        Check(std::fabs(h.weight-.5f)<1e-5f&&loZero,text);
        // closed gate (no reload, no slide, rifle, invalid rig: the driver passes gate=false): never any weight, whatever the roll
        SlideDeRoll off;bool offZero=true;for(int i=0;i<int(2*hz);++i)offZero=offZero&&off.Step(dt,false,60.0f)==0.0f&&off.Idle()&&off.peak==0.0f;
        std::snprintf(text,sizeof text,"%.0f Hz: non-slide / non-reload frames: weight exactly 0, idle (the driver never edits the pose)",hz);
        Check(offZero&&!SlideDeRoll::Engaged(false),text);
    }
    SlideDeRoll d;d.Step(1.f/60,true,36.0f);const float w=d.weight;
    d.Step(std::numeric_limits<float>::quiet_NaN(),true,36.0f);d.Step(0.0f,true,36.0f);d.Step(-1.0f,false,36.0f);
    Check(d.weight==w,"de-roll: non-finite / non-positive dt ignored");
    SlideDeRoll n;n.Step(1.f/60,true,std::numeric_limits<float>::quiet_NaN());
    Check(n.weight==0.0f&&n.Idle(),"de-roll: NaN roll gives no weight");
    Check(SlideDeRoll::Engaged(true),"de-roll engaged for a pistol reload on a slide frame");
}
// Driver composition on the synthetic rig (MotionDriver SlideReloadTorsoStepLocked + ApplyLocked reloadScale): every frame the raw MM
// pose is rolled by the slide, de-rolled, the hold sees the roll left. Compared with the same reload standing (no slide).
struct HandBack{float holdMax=0,uMaxOverStand=0,tHoldZero=-1,tNative=-1,tNativeStand=-1,residualAfterAttack=0,leanErrAfterAttack=0,
                leanStepMax=0,leanStepAllowed=0;};
static HandBack SimulateSlideReload(float hz,bool fromAim,float rawRoll)
{
    const float dt=1.0f/hz;ovn::Skeleton sk;ovn::Pose raw;sd::Bones b;RolledRig(sk,raw,b,rawRoll);
    UpperEnvelope env,stand;SlideRollHold hold;SlideDeRoll der;HandBack o;float t=0;
    const float lean0=Lean(sk,raw,b);float prevLean=lean0;
    // the lean is driven by the shared weight: per frame at most the smoothstep slope 1.5 * dt / attack of the whole correction
    // (+ the lean the partial de-roll itself moves; 2deg covers it on this rig)
    o.leanStepAllowed=1.5f*dt/kDeRollAttackSeconds*(std::fabs(lean0-sd::kDePitchTargetDeg)+2.0f);
    for(int i=0;i<int(1.2f*hz);++i)
    {
        t+=dt;ovn::Pose p=raw;
        const float before=sd::RollDeg(sk,p,b);
        const float w=der.Step(dt,true,before);
        if(w>0)sd::Apply(sk,p,b,w);
        const float left=sd::RollDeg(sk,p,b),lean=Lean(sk,p,b);
        o.leanStepMax=std::fmax(o.leanStepMax,std::fabs(lean-prevLean));prevLean=lean;
        if(t>kDeRollAttackSeconds+dt)o.leanErrAfterAttack=std::fmax(o.leanErrAfterAttack,std::fabs(lean-sd::kDePitchTargetDeg));
        const float h=hold.Step(dt,true,true,left,SlideDeRoll::Engaged(true));
        env.Step(dt,true,fromAim,true);stand.Step(dt,true,fromAim,true);
        const float u=std::fmax(1.0f-AuthoredFadeWeight(env.weight),h),uStand=1.0f-AuthoredFadeWeight(stand.weight);
        o.holdMax=std::fmax(o.holdMax,h);o.uMaxOverStand=std::fmax(o.uMaxOverStand,u-uStand);
        if(t>kDeRollAttackSeconds+dt)o.residualAfterAttack=std::fmax(o.residualAfterAttack,left);
        if(h==0.0f&&o.tHoldZero<0)o.tHoldZero=t;
        if(u==0.0f&&o.tNative<0)o.tNative=t;
        if(uStand==0.0f&&o.tNativeStand<0)o.tNativeStand=t;
    }
    return o;
}
static void DeRollHandBack()
{
    for(const float hz:{30.0f,60.0f,144.0f})
        for(const bool fromAim:{false,true})
        {
            const HandBack o=SimulateSlideReload(hz,fromAim,36.0f);char text[220];
            std::printf("  %.0f Hz %s: hold max %.3f (0 at %.3fs), native arms full at %.3fs (standing %.3fs), max extra MM share %.3f, roll left %.2fdeg\n",
                        hz,fromAim?"from ADS":"hip     ",o.holdMax,o.tHoldZero,o.tNative,o.tNativeStand,o.uMaxOverStand,o.residualAfterAttack);
            std::snprintf(text,sizeof text,"%.0f Hz %s: slide reload hands the arms back as a standing one (hold gone by .30s, extra MM share <= .05)",
                          hz,fromAim?"ADS":"hip");
            Check(o.tHoldZero>0&&o.tHoldZero<=.30f&&o.uMaxOverStand<=.05f&&o.tNative>0&&o.tNative<=o.tNativeStand+1.0f/hz+1e-4f,text);
            std::snprintf(text,sizeof text,"%.0f Hz %s: roll left < 2deg once the de-roll is in",hz,fromAim?"ADS":"hip");
            Check(o.residualAfterAttack<2.0f,text);
            std::printf("  %.0f Hz %s: lean error after the attack %.3fdeg, max lean step %.2f <= %.2fdeg/frame\n",hz,fromAim?"ADS":"hip",
                        o.leanErrAfterAttack,o.leanStepMax,o.leanStepAllowed);
            std::snprintf(text,sizeof text,"%.0f Hz %s: lean at the standing target (< 1deg) once the weight is in, ramped without steps",hz,fromAim?"ADS":"hip");
            Check(o.leanErrAfterAttack<1.0f&&o.leanStepMax<=o.leanStepAllowed,text);
        }
    // Recorded roll series (03:37 k10 / k11, raw MM roll before any de-roll): roll left = (1 - w) * roll (Apply is exact, see above).
    struct Series{const char* name;const float* dt;const unsigned char* slide;const float* roll;};
    const Series series[]={{"03:37 k10 Into_Rfoot",kSlideR_dt,kSlideR_slide,kSlideR_roll},{"03:37 k11 Into_Lfoot",kSlideL_dt,kSlideL_slide,kSlideL_roll}};
    for(const Series& s:series)
    {
        UpperEnvelope env,stand;SlideRollHold hold;SlideDeRoll der;float holdMax=0,extra=0,leftMax=0,clock=0,edge=0,tHoldZero=-1;
        for(int i=0;i<kN;++i)
        {
            clock+=s.dt[i];const bool reload=i>=2;if(i==2)edge=clock;
            const bool gate=reload&&s.slide[i]!=0;
            const float w=der.Step(s.dt[i],gate,s.roll[i]);
            const float left=s.roll[i]*(1.0f-w);
            const float h=hold.Step(s.dt[i],reload,s.slide[i]!=0,left,SlideDeRoll::Engaged(gate));
            env.Step(s.dt[i],reload,true);stand.Step(s.dt[i],reload,true);
            holdMax=std::fmax(holdMax,h);
            if(reload&&h==0.0f&&tHoldZero<0)tHoldZero=clock-edge;
            extra=std::fmax(extra,std::fmax(1.0f-AuthoredFadeWeight(env.weight),h)-(1.0f-AuthoredFadeWeight(stand.weight)));
            if(clock-edge>kDeRollAttackSeconds+.02f&&reload&&s.slide[i])leftMax=std::fmax(leftMax,left);
        }
        std::printf("  %s with de-roll: hold max %.3f (0 at +%.3fs), extra MM share %.3f, roll left after the attack %.2fdeg\n",s.name,holdMax,
                    tHoldZero,extra,leftMax);
        char text[200];std::snprintf(text,sizeof text,"%s: recorded slide roll, de-rolled: hold gone by .30s, arms back as standing (extra <= .05)",s.name);
        Check(tHoldZero>=0&&tHoldZero<=.30f&&extra<=.05f&&leftMax<2.0f,text);
    }
}

// ---- 4. rifle reload on a slide frame (recording 2026-10-26 11:28, 8 rifle slide reloads) -------------------------------------
// The torso uprighting gate takes rifles too (SlideTorsoReload; the roll hold stays pistol-only in the driver). A rifle reload that
// goes on after the slide frames while the MM slide exit is still rolled keeps the de-roll (RifleSlideExitLatch) instead of rolling
// the uprighted chest back (replay ep 17: roll left 3 -> 29 -> 0 deg within .5 s). Mutation 6 keeps the gate pistol-only, mutation 7
// disables the latch: both MUST fail.
static void RifleSlideReload()
{
    Check(SlideTorsoReload(true,true,false),"slide torso gate: pistol reload (unchanged)");
    Check(SlideTorsoReload(true,false,true),"slide torso gate: rifle reload is uprighted too");
    Check(!SlideTorsoReload(true,false,false)&&!SlideTorsoReload(false,true,false)&&!SlideTorsoReload(false,false,true),
          "slide torso gate: other overlays / no reload: closed");
    // latch semantics
    {
        RifleSlideExitLatch l;
        const bool a=l.Step(true,false,36.0f);                 // ground frame before any slide frame: never
        const bool b=l.Step(true,true,36.0f);                  // slide frame: arms the latch, the slide frame itself opens the gate
        const bool c=l.Step(true,false,30.0f);                 // ground, still rolled: kept open
        const bool d=l.Step(true,false,21.0f);                 // roll below the range: released
        const bool e=l.Step(true,false,30.0f);                 // roll rises again: no re-latch without a slide frame
        l.Step(true,true,36.0f);const bool f=l.Step(true,false,std::numeric_limits<float>::quiet_NaN()); // NaN roll: released
        l.Step(true,true,36.0f);const bool g=l.Step(false,false,36.0f);const bool h=l.Step(true,false,36.0f); // reload end resets
        RifleSlideExitLatch p;p.Step(false,true,36.0f);const bool i=p.Step(false,false,36.0f); // pistol (not a rifle reload): never
        Check(!a&&!b&&c&&!d&&!e&&!f&&!g&&!h&&!i&&!p.latched,
              "rifle exit latch: only after a slide frame of the same reload, while the MM roll is >= 22 deg; no re-latch, reset on reload end");
    }
    // composition at 30 / 60 / 144 Hz: 10 slide frames at 37 deg (reload edge on the first), then ground frames whose MM slide exit
    // un-rolls 34 -> 0 deg over .5 s (ep 17 of the recording); roll left = (1 - w) * roll (Apply is exact on the rig, section 3)
    for(const float hz:{30.0f,60.0f,144.0f})
    {
        const float dt=1.0f/hz;
        SlideDeRoll der;RifleSlideExitLatch latch;float leftMax=0,prevW=0,stepMax=0,t=0,wEnd=0;
        const int slideN=int(std::lround(.15f*hz)),n=int(1.2f*hz);
        for(int k=0;k<n;++k)
        {
            t+=dt;
            const bool slide=k<slideN;
            const float roll=slide ? 37.0f : std::fmax(0.0f,34.0f*(1.0f-(t-.15f)/.5f));
            const bool torso=SlideTorsoReload(true,false,true);
            const bool exitLatch=latch.Step(torso,slide,roll);
            const float w=der.Step(dt,torso&&(slide||exitLatch),roll);
            if(t>kDeRollAttackSeconds+dt)leftMax=std::fmax(leftMax,roll*(1.0f-w));
            stepMax=std::fmax(stepMax,std::fabs(w-prevW));prevW=w;wEnd=w;
        }
        char text[220];
        std::printf("  %.0f Hz rifle slide exit: roll left max after the attack %.2f deg, max weight step %.4f, weight at the end %.3f\n",hz,leftMax,stepMax,wEnd);
        std::snprintf(text,sizeof text,"%.0f Hz: rifle reload through a rolled slide exit keeps the chest within 10 deg of level (no roll-back bump)",hz);
        Check(leftMax<10.0f,text);
        std::snprintf(text,sizeof text,"%.0f Hz: weight moves without steps (<= 1.5 dt / attack) and is released once the roll has left the range",hz);
        Check(stepMax<=1.5f*dt/kDeRollAttackSeconds+1e-5f&&wEnd==0.0f&&der.Idle(),text);
    }
}

// ---- 5. rifle CARRY on a slide frame (no reload; recording 2026-10-27 15:14): the MM-owned Rifle arms ride the rolled slide torso,
// the barrel pitched to +33..35 deg (Ready -23.6). Driver composition (MotionDriver SlideReloadTorsoStepLocked): carry gate on slide frames
// and while the shared exit latch holds, early ramp, applied weight = w * SlideCarryAim (1 - AimingWeight). Mutation 8 disables it.
static float BarrelPitchDeg(const ovn::Skeleton& sk,const ovn::Pose& p,om::Vec3 aInArm)
{
    om::Quat q;om::Vec3 t;sd::ModelOf(sk,p,kRArm,q,t); // the weapon rides the right arm (identity locals under Spine3: the chest frame)
    const om::Vec3 d=om::QuatRotate(q,aInArm);return std::asin(std::clamp(d.z/std::sqrt(om::Dot(d,d)),-1.0f,1.0f))*57.29578f;
}
struct CarryRun{float entryLeftMax=0,leftAfterAttack=0,exitLeftMax=0,stepExcess=-1,riseExcess=-1,wEnd=-1,appliedMax=0;bool idleEnd=false,closedRise=false;
                int limitedFrames=0;float tClip=-1;};
// slide .6 s (roll 3 -> 32 deg over .16 s, then 34), then ground frames whose exit clip un-rolls 34 -> 0 over .5 s, until 1.4 s.
// aimAt(t): AimingWeight; reloadFrom: reload gate open from that time on slide frames (carry closed), < 0 = never; carryUntil: the
// carry admission (MM owns the arms) ends at that time (e.g. ADS with the game upper body) while the slide goes on; jump > 0: the entry roll reaches 22 deg at
// that time (ramp already up) and then jumps to 34 deg at 600 deg/s (the entry blend of the recording reaches ~300 deg/s).
template<class AimFn>
static CarryRun SimulateCarry(float hz,bool early,AimFn aimAt,float reloadFrom=-1.0f,bool carry=true,float carryUntil=1e9f,float jump=-1.0f)
{
    const float dt=1.0f/hz;CarryRun o;SlideDeRoll der;RifleSlideExitLatch latch;SlideCarryAim aim;float t=0,prevApplied=0,prevPeak=0,prevAim=aimAt(0.0f),prevW=0;
    const float bound=1.5f*dt/kDeRollAttackSeconds;
    for(int k=0;k<int(1.4f*hz);++k)
    {
        t+=dt;
        const bool slide=t<=.6f;
        const float rollIn=jump>0.0f ? (t<jump ? 3.0f+19.0f*t/jump : std::fmin(34.0f,22.0f+600.0f*(t-jump))) : (t<.16f ? 3.0f+29.0f*t/.16f : 34.0f);
        const float roll=slide ? rollIn : std::fmax(0.0f,34.0f*(1.0f-(t-.6f)/.5f));
        const bool reload=reloadFrom>=0.0f && t>=reloadFrom;
        const bool rifleCarry=SlideCarryRifle(carry,t<carryUntil,reload);
        const bool exitLatch=latch.Step(reload||rifleCarry,slide,roll);
        const bool gate=reload&&(slide||exitLatch);
        const bool carryGate=rifleCarry&&(slide||exitLatch);
        const bool wasLimited=der.riseLimited;
        const float w=der.Step(dt,gate||carryGate,roll,early&&carryGate);
        const float a=aimAt(t);
        const float applied=w*aim.Step(dt,carryGate,gate,a);
        if(der.Idle())aim.Reset();
        const float left=roll*(1.0f-applied); // Apply is exact on the rig (section 3)
        if(slide&&t<=.30f)o.entryLeftMax=std::fmax(o.entryLeftMax,left);
        if(slide&&t>=.25f)o.leftAfterAttack=std::fmax(o.leftAfterAttack,left);
        if(!slide)o.exitLeftMax=std::fmax(o.exitLeftMax,left);
        // w = smoothstep(ramp) * peak, scaled by 1 - aim: |dw| <= 1.5 dt / attack + dpeak + daim (+ dt / attack for a reload hand-over)
        // (a carry weight that lagged behind its target catches up at most at the rise limit dt / kCarryRiseSeconds)
        const float allowed=std::fmax(bound+std::fmax(0.0f,der.peak-prevPeak),dt/kCarryRiseSeconds)+std::fabs(a-prevAim)+
                            dt/kDeRollAttackSeconds*(reload?1.0f:0.0f);
        o.stepExcess=std::fmax(o.stepExcess,std::fabs(applied-prevApplied)-allowed-1e-5f);
        // carry frames and every frame after a clipped carry weight: the weight never rises faster than 1 / kCarryRiseSeconds (the roll
        // of the entry blend can be faster; a reload taking over / a gate closing in the entry must not jump to the target), and it never
        // rises on a frame whose gate is closed
        if(carryGate||wasLimited)o.riseExcess=std::fmax(o.riseExcess,(w-prevW)-dt/kCarryRiseSeconds-1e-6f);
        if(!(gate||carryGate)&&w>prevW)o.closedRise=true;
        o.limitedFrames+=der.riseLimited?1:0;
        if(der.riseLimited&&o.tClip<0.0f)o.tClip=t;
        prevW=w;
        o.appliedMax=std::fmax(o.appliedMax,applied);
        prevApplied=applied;prevPeak=der.peak;prevAim=a;o.wEnd=applied;
    }
    o.idleEnd=der.Idle()&&aim.scale==1.0f&&!aim.carry;
    return o;
}
static void RifleCarry()
{
    Check(SlideCarryRifle(true,true,false),"carry gate: drawn rifle, MM owns the arms, no reload -> open");
    Check(!SlideCarryRifle(false,true,false)&&!SlideCarryRifle(true,false,false)&&!SlideCarryRifle(true,true,true),
          "carry gate: pistol / other overlay, arms not MM-owned, reload (the reload path owns it): closed");
    // the reload / pistol calls keep the former policy bit for bit (earlyRamp false; SlideCarryAim without a carry frame is exactly 1):
    // reference = the SlideDeRoll::Step body before the carry path (git HEAD ca02a4b), over hitches, invalid dt, NaN rolls, gate flips
    {
        struct Former
        {
            float ramp=0,peak=0,weight=0;
            float Step(float dt,bool gate,float rollDeg)
            {
                if(!std::isfinite(dt) || dt<=0)return weight;
                const bool on=SlideDeRoll::Engaged(gate);
                if(on && std::isfinite(rollDeg))
                {
                    const float x=std::clamp((rollDeg-kRollHoldStartDeg)/(kRollHoldFullDeg-kRollHoldStartDeg),0.0f,1.0f);
                    peak=std::max(peak,x*x*(3.0f-2.0f*x));
                }
                const float d=std::min(dt,.25f);
                ramp=(on && peak>0.0f) ? std::min(1.0f,ramp+d/kDeRollAttackSeconds) : std::max(0.0f,ramp-d/kDeRollReleaseSeconds);
                if(ramp==0.0f && !on)peak=0.0f;
                weight=ramp*ramp*(3.0f-2.0f*ramp)*peak;
                return weight;
            }
        };
        Former a;SlideDeRoll b;SlideCarryAim s;bool same=true,one=true;
        const float nan=std::numeric_limits<float>::quiet_NaN();
        for(int i=0;i<2000;++i)
        {
            const float roll=(i%53==0)?nan:float((i*13)%47);
            const float dt=(i%97==0)?nan:(i%89==0)?-1.0f:(i%7==0)?.4f:1.0f/float(30+(i*11)%115);
            const bool g=((i/9)%3!=0)!=((i/131)%2!=0);
            const float wa=a.Step(dt,g,roll),wb=b.Step(dt,g,roll,false);
            const float sc=s.Step(dt,false,g,float((i*5)%11)/10.0f);
            same=same&&std::memcmp(&wa,&wb,sizeof wa)==0&&std::memcmp(&a.ramp,&b.ramp,sizeof a.ramp)==0&&
                 std::memcmp(&a.peak,&b.peak,sizeof a.peak)==0&&!b.riseLimited;
            const float wapp=wb*sc;one=one&&sc==1.0f&&std::memcmp(&wapp,&wb,sizeof wb)==0;
        }
        Check(same&&one,"reload / pistol path: Step(..., earlyRamp=false) == the former Step bit for bit, aim scale 1 (w * 1 == w)");
    }
    // geometry: the upright carry torso (the unrolled synthetic rig, lean ~ the standing target) is rolled by 35 deg about the horizontal
    // axis normal to its shoulder line (as the slide clip does, Spine1..3 in thirds); the Ready barrel carried in the chest frame (aimed
    // 30 deg left of the facing across the body, pitched to the recorded Ready -23.6 deg) then pitches up; the full de-roll brings it back.
    {
        ovn::Skeleton sk;ovn::Pose up;sd::Bones b;RolledRig(sk,up,b,0.0f);
        (void)sd::Apply(sk,up,b,1.0f); // level line: only the lean is set to the standing target (the upright carry torso)
        om::Vec3 l,r;sd::ShoulderLine(sk,up,b,l,r);
        const om::Vec3 line=r-l,axisRaw=om::Cross(line,om::Vec3{0.0f,0.0f,1.0f});
        const om::Vec3 axis=axisRaw*(1.0f/std::sqrt(om::Dot(axisRaw,axisRaw)));
        ovn::Pose rolled=up;const bool rotOk=sd::RotateChain(sk,rolled,b,axis,-35.0f*.01745329f);
        om::Quat qa;om::Vec3 ta;sd::ModelOf(sk,up,kRArm,qa,ta);
        const float yaw=(55.0f+30.0f)*.01745329f,pit=-23.6f*.01745329f; // facing = Hips yaw 55 deg (+Y yawed), barrel 30 deg to the left
        const om::Vec3 dir{-std::sin(yaw)*std::cos(pit),std::cos(yaw)*std::cos(pit),std::sin(pit)};
        const om::Vec3 inArm=om::QuatRotate(om::QuatInv(qa),dir);
        const float pUp=BarrelPitchDeg(sk,up,inArm),pRolled=BarrelPitchDeg(sk,rolled,inArm);
        ovn::Pose fixedP=rolled;sd::Apply(sk,fixedP,b,1.0f);
        const float pFixed=BarrelPitchDeg(sk,fixedP,inArm);
        ovn::Pose aimed=rolled;const float wAimed=1.0f*(1.0f-1.0f);const sd::Result ra=sd::Apply(sk,aimed,b,wAimed);
        std::printf("  carry barrel pitch: upright %.2f (lean %.2f), rolled %.2f (roll %.2f), de-rolled %.2f deg (roll left %.2f, lean %.2f)\n",pUp,
                    Lean(sk,up,b),pRolled,sd::RollDeg(sk,rolled,b),pFixed,sd::RollDeg(sk,fixedP,b),Lean(sk,fixedP,b));
        Check(rotOk&&pRolled-pUp>15.0f&&std::fabs(pFixed-pUp)<3.0f&&sd::RollDeg(sk,fixedP,b)<.05f,
              "carry: rolled slide torso pitches the barrel up >15 deg off Ready, the de-roll levels the line and brings it within 3 deg");
        Check(!ra.Changed()&&SameBones(aimed,rolled),"carry, fully aimed (AimingWeight 1): weight 0, pose bit-identical");
    }
    for(const float hz:{30.0f,60.0f,144.0f})
    {
        char text[240];
        const auto noAim=[](float){return 0.0f;};
        const CarryRun e=SimulateCarry(hz,true,noAim),l=SimulateCarry(hz,false,noAim);
        std::printf("  %.0f Hz carry: entry roll left max early %.2f / late %.2f deg, after attack %.2f, exit %.2f, step excess %.5f, end w %.3f\n",
                    hz,e.entryLeftMax,l.entryLeftMax,e.leftAfterAttack,e.exitLeftMax,e.stepExcess,e.wEnd);
        std::snprintf(text,sizeof text,"%.0f Hz carry: early ramp lowers the entry roll (%.1f < %.1f - 2 deg of the roll-range ramp)",hz,e.entryLeftMax,l.entryLeftMax);
        Check(e.entryLeftMax<l.entryLeftMax-2.0f,text);
        std::snprintf(text,sizeof text,"%.0f Hz carry: shoulder line within 2 deg of level on the slide after the attack, < 10 deg through the rolled exit (latch)",hz);
        Check(e.leftAfterAttack<2.0f&&e.exitLeftMax<10.0f,text);
        std::snprintf(text,sizeof text,"%.0f Hz carry: weight steps bounded (1.5 dt / attack + peak rise; rise <= dt / %.2fs), released to 0 and idle after the exit",
                      hz,kCarryRiseSeconds);
        Check(e.stepExcess<=0.0f&&e.riseExcess<=0.0f&&e.wEnd==0.0f&&e.idleEnd,text);
        // aimed slide (AimingWeight 1 through the hold): never an edit
        const CarryRun aimed=SimulateCarry(hz,true,[](float){return 1.0f;});
        std::snprintf(text,sizeof text,"%.0f Hz carry: aimed slide (AimingWeight 1) -> applied weight exactly 0 on every frame",hz);
        Check(aimed.appliedMax==0.0f&&aimed.idleEnd,text);
        // aim released mid-slide (AimingWeight 1 -> 0 over .2 s from .3 s): the uprighting follows without steps
        const CarryRun rel=SimulateCarry(hz,true,[](float t){return std::clamp(1.0f-(t-.3f)/.2f,0.0f,1.0f);});
        std::snprintf(text,sizeof text,"%.0f Hz carry: aim released mid-slide -> continuous (step <= ramp + peak + aim change), levelled by the exit",hz);
        Check(rel.stepExcess<=0.0f&&rel.exitLeftMax<10.0f&&rel.idleEnd,text);
        // aimed carry, then a reload starts mid-slide (.35 s): the reload path takes the torso over without a jump (scale back to 1 at
        // 1 / attack), the former reload uprighting holds through the rolled exit
        const CarryRun hand=SimulateCarry(hz,true,[](float){return 1.0f;},.35f);
        std::snprintf(text,sizeof text,"%.0f Hz carry -> reload hand-over while aimed: no step (<= dt / attack more), levelled after the attack",hz);
        Check(hand.stepExcess<=0.0f&&hand.exitLeftMax<10.0f&&hand.idleEnd,text);
        // the carry episode ends inside the entry: a reload takes over / the carry admission ends (ADS with the game upper body), on slide
        // frame 2 and right after the first frame whose weight the rise limit clipped (entry roll jumping 22 -> 34 deg at 600 deg/s from .12 s,
        // so every rate clips): no jump to the target, no rise on a closed gate
        const auto noAim0=[](float){return 0.0f;};
        const float tClip=SimulateCarry(hz,true,noAim0,-1.0f,true,1e9f,.12f).tClip;
        Check(tClip>0.0f,"carry: the fast entry roll is clipped by the rise limit (test precondition)");
        for(const float at:{2.0f/hz,tClip+.5f/hz})
        {
            const CarryRun rl=SimulateCarry(hz,true,noAim0,at,true,1e9f,.12f);
            const CarryRun cl=SimulateCarry(hz,true,noAim0,-1.0f,true,at,.12f);
            std::printf("  %.0f Hz carry ends at %.3fs: reload take-over rise excess %.5f (clipped frames %d), gate close rise excess %.5f (clipped %d)%s\n",
                        hz,at,rl.riseExcess,rl.limitedFrames,cl.riseExcess,cl.limitedFrames,cl.closedRise?" ROSE ON A CLOSED GATE":"");
            std::snprintf(text,sizeof text,"%.0f Hz carry -> reload at %.3fs: the weight never rises faster than dt / %.2fs, levelled, idle at the end",hz,at,kCarryRiseSeconds);
            Check(rl.riseExcess<=0.0f&&rl.stepExcess<=0.0f&&rl.leftAfterAttack<2.0f&&rl.idleEnd,text);
            std::snprintf(text,sizeof text,"%.0f Hz carry gate closes at %.3fs: no rise on the closed gate, released to 0",hz,at);
            Check(cl.riseExcess<=0.0f&&!cl.closedRise&&cl.wEnd==0.0f&&cl.idleEnd,text);
        }
    }
    // mutation guard: without the carry gate the rolled slide stays rolled (what mutation 8 does)
    const CarryRun off=SimulateCarry(60.0f,true,[](float){return 0.0f;},-1.0f,false);
    Check(off.appliedMax==0.0f&&off.leftAfterAttack>30.0f,"carry gate closed (pistol / not owned): no edit, the slide roll stays");
}

int main()
{
    // 1. Pure helper checks.
    Check(std::fabs(ShoulderLineRollDeg(-.17f,0,0,.17f,0,0))<1e-4f,"level shoulder line: roll 0");
    Check(std::fabs(ShoulderLineRollDeg(0,0,0,.3f,0,.3f)-45.0f)<.01f,"shoulder line at 45deg: roll 45");
    Check(ShoulderLineRollDeg(0,0,0,0,0,0)==0.0f&&ShoulderLineRollDeg(std::numeric_limits<float>::quiet_NaN(),0,0,1,0,0)==0.0f,"degenerate / NaN line: roll 0");
    {
        SlideRollHold h;
        for(int i=0;i<120;++i)h.Step(1.f/60,true,false,60.0f); // not a slide frame: never holds, whatever the roll
        Check(h.hold==0.0f,"non-slide frames never create a hold (even at 60deg shoulder roll)");
        h.Reset();
        for(int i=0;i<120;++i)h.Step(1.f/60,false,true,60.0f); // slide but no reload
        Check(h.hold==0.0f,"slide without reload never creates a hold");
        h.Reset();
        const float first=h.Step(1.f/60,true,true,36.0f);
        Check(first==1.0f,"reload that starts fully rolled in a slide owns the hold at once");
        float prev=first;bool slewOk=true;
        for(int i=0;i<60;++i){const float v=h.Step(1.f/60,true,true,0.0f);slewOk=slewOk&&(prev-v)<=1.f/60/kRollHoldReleaseSeconds+1e-6f&&v<=prev;prev=v;}
        Check(slewOk&&h.hold==0.0f,"hold releases at the limited rate once the torso is level");
        h.Reset();h.Step(1.f/60,true,true,0.0f);
        float v=0;int frames=0;while(v<1.0f&&frames<200){v=h.Step(1.f/60,true,true,40.0f);++frames;}
        Check(frames>=8&&frames<=11,"slide that rolls the torso mid-reload ramps the hold in over ~.15s");
        h.Reset();h.Step(1.f/60,true,true,36.0f);
        const float bad=h.hold;h.Step(std::numeric_limits<float>::quiet_NaN(),true,true,0.0f);h.Step(-1.f,true,true,0.0f);
        Check(h.hold==bad,"non-finite / non-positive dt ignored");
        SlideRollHold nan;nan.Step(1.f/60,true,true,std::numeric_limits<float>::quiet_NaN());
        Check(nan.hold==0.0f,"NaN roll gives no hold");
    }
    // 2. Recorded slide reloads.
    const Fixture fixtures[]={
        {"03:37 k10 Into_Rfoot",kSlideR_dt,kSlideR_slide,kSlideR_roll,kSlideR_native,kSlideR_hold,kSlideR_rec,kSlideR_whd_native,kSlideR_whd_hold},
        {"03:37 k11 Into_Lfoot",kSlideL_dt,kSlideL_slide,kSlideL_roll,kSlideL_native,kSlideL_hold,kSlideL_rec,kSlideL_whd_native,kSlideL_whd_hold}};
    for(const Fixture& f:fixtures)
    {
        float recD05,recWhd;Recorded(f,recD05,recWhd);
        const Out o=Simulate(f);
        std::printf("  %s: recorded d05=%.1fcm weaponAboveHead(native arms)=%.1fcm | new d05=%.1fcm d10=%.1fcm W-Hd max=%.1fcm hold@edge=%.2f\n",
                    f.name,recD05,recWhd,o.d05,o.d10,o.whdMax,o.holdAtEdge);
        char text[200];
        std::snprintf(text,sizeof text,"%s: recorded pose reproduces the bug (weapon >30cm per .05s, native arms above the head)",f.name);
        Check(recD05>30.0f&&recWhd>3.0f,text);
        std::snprintf(text,sizeof text,"%s: hold owns the arms from the first reload frame",f.name);
        Check(o.holdAtEdge>.99f,text);
        std::snprintf(text,sizeof text,"%s: weapon moves less than 10cm per .05s",f.name);
        Check(o.d05<10.0f,text);
        std::snprintf(text,sizeof text,"%s: weapon stays below the head (>=8cm) for the whole rolled phase",f.name);
        Check(o.whdMax<-8.0f,text);
        std::snprintf(text,sizeof text,"%s: hold changes at most 1/%.2fs",f.name,kRollHoldReleaseSeconds);
        Check(o.holdMaxStep<=1.0f/kRollHoldReleaseSeconds+1e-3f,text);
        std::snprintf(text,sizeof text,"%s: native arms own the reload once the torso is level (hold 0, upper scale 0 by the end)",f.name);
        Check(o.h[kN-1]==0.0f&&o.u[kN-1]==0.0f,text);
    }
    // 3. Torso counter-roll (de-roll).
    {
        SlideRollHold a,b2;bool same=true;
        for(int i=0;i<200;++i){const float roll=float((i*7)%45);const bool rl=i>3&&i<150,sl=i>20;
                               same=same&&a.Step(1.f/60,rl,sl,roll)==b2.Step(1.f/60,rl,sl,roll,false);}
        Check(same,"hold: deRolled=false is the former policy bit for bit");
    }
    DeRollGeometry();
    DeRollWeightRates();
    DeRollHandBack();
    // 4. Rifle reload on a slide frame.
    RifleSlideReload();
    // 5. Rifle carry (no reload) on a slide frame.
    RifleCarry();
    std::printf("%d passed, %d failed\n",g_pass,g_fail);
    return g_fail==0?0:1;
}
