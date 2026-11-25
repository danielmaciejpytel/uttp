# Changelog

Changes of Ultimate Third Person Project, newest version first.

## 0.2.0 - 2026-11-25

For Cyberpunk 2077 2.5 with Phantom Liberty. The requirements are the same as for 0.1.2: RED4ext 1.31.1, redscript 0.5.32, Cyber Engine Tweaks 1.38.1, Codeware 1.21.4, ArchiveXL 1.28.8, Input Loader 0.2.6 and Mod Settings 0.2.30.

**Updating:** install 0.2.0 over the version you have and let it replace every file. No file of an older version has to be removed. Coming from any 0.1.x version, the settings of the mod are reset to their defaults once at the first start (see Changed); a fresh install shows no message.

### Added

- **Feet on uneven ground** (on): V's feet are placed on slopes, steps, stairs and curbs, the legs bend and the hips lower to match.
- **Firm feet in turns and stops**, **Planted feet keep their direction** and **Firm pivot foot in turns on the spot** (all on).
- **Stride matching** (on): V's steps are shortened to her speed in the walking, running and crouched movement loops.
- **Fit steps to stairs** (off): shorter, quicker steps on stairs at the same speed. At walking and running pace a step still spans two to four stairs. It needs Feet on uneven ground and Stride matching.
- **Head follows the camera** (on): in Shooter mode and while strafing without aiming, the head and neck turn and tilt towards the camera, up to 45 degrees to the side, then the upper body follows.
- **Natural body turn while aiming on the move** (on): after V has turned to the crosshair, her legs and hips keep their own direction and turn with her steps; the upper body and the weapon stay on the crosshair.
- **Gamepad stick speed** (Advanced): Walk or run (default) or Follows the stick.
- **Body lean** and **Head lean** are now options in Mod Settings.

### Changed

- **Animation selection** behaves more like a full motion-matching reference, with more protection against animations flickering between each other.
- **Playback speed:** on the ground the movement animations follow V's actual speed, including crouched movement. The movement predicted for picking animations is shaped for starts, stops, gait changes, turns, strafing, crouching, aiming, stairs, jumps and landings.
- **Turns:** in 180-degree turns on the move the body turns the same way as the turn animation, the turn steering follows only the active animation, and the knees follow the direction of the hips.
- **Stops and crouch:** less forward overshoot and a firmer foot plant when stopping; a different choice of the crouched stop, and a floating foot is lowered at the end of it.
- **Dodges:** the direction is taken relative to the character, the dodge animation is turned towards the travel direction (up to 90 degrees), and the animation is kept while V faces away from the travel direction.
- **Lean:** much lighter by default (body and head 0.2; before 0.85 and 0.9). It now also works with a weapon drawn, while aiming (body only) and while crouched.
- **Walking:** the walk toggle speed is 1.9 m/s by default (was 1.6).
- **Gamepad:** by default a stick pushed less than about two thirds of the way gives a steady walk, further gives a full run, and crouching uses the full crouch speed. This changes how fast V moves; Follows the stick restores the analog speed.
- **Aiming:** the barrel is also corrected sideways towards the crosshair; the upper body is steadier while V turns and during camera jerks; a change of direction while aiming and moving can play a turn animation from 10 degrees; quick left and right taps stay on turn-step animations with firmer feet.
- Moving forward while aiming is faster (3.85 m/s, was 3.3), with sideways and backwards movement scaled to it.
- The supporting hand stays on two-handed weapons and the pistol, for example in an aimed slide or while crouched.
- If the mod pauses itself to protect the frame rate while a ranged weapon is drawn, the weapon pose is kept.
- **Shooter mode** (still off by default) holds each direction animation for a minimum time.
- The body height is smoothed when V moves over bumpy ground.
- Male V's drawn body is no longer turned 4 degrees against the camera by default; female V keeps 4 degrees.
- **All settings of the mod are reset to their defaults once**, at the first start after updating from 0.1.x, because 0.2.0 changes the movement and its defaults. A message on the screen tells you when it happened: "The default settings of Ultimate Third Person Project were restored because of major changes in the mod. Have a good game!" The two "Swap shoulder" controls are not changed. This happens once: afterwards your settings are kept as before.

### Fixed

- **Lean:** in Shooter mode and while facing the aim, V no longer rolls along the wrong axis.
- **Feet:** smaller foot correction peaks on reversals and run stops.
- **Walls:** sprinting into a wall, or being pinned against it, holds the idle pose instead of flickering between turn animations.
- **Dodge:** the direction warping no longer twists V's body during a dodge; this mattered most at 30 FPS.
- **Weapons:** drawing, holstering or changing the weapon type blends into the new pose without overshooting.
- **Camera:** curbs and stair steps should no longer switch the camera into its fast-fall mode.
- **Elevators and moving platforms:** V should no longer play jump or fall animations during the ride, and the camera and body height should ride with the cabin.
- **Mod Settings:** the lists should now show a readable label for every value.

### Notes

- The motion-matching databases and the lean and traversal data are the same as in 0.1.2. No new climbing or vaulting behaviour is switched on in this version.

### Known issues

See [Known issues](README.md#known-issues-in-020) in the readme.

## 0.1.2 - 2026-11-15

For Cyberpunk 2077 2.5 with Phantom Liberty. Tested with RED4ext 1.31.1, redscript 0.5.32, Cyber Engine Tweaks 1.38.1, Codeware 1.21.4, ArchiveXL 1.28.8, Input Loader 0.2.6 and Mod Settings 0.2.30.

**Updating:** install 0.1.2 over the version you have and let it replace every file. No file of an older version has to be removed. Settings are kept when you come from 0.1.1; coming from 0.1.0 they are reset to their defaults once (see 0.1.1).

### Added

- **Feminine figure for female V:** while V stands and moves without a weapon her shoulders slope down and sit further back, her arms hang closer to the body and her head sits about 2 cm lower, so the neck looks shorter and the shoulder line softer. Hips, chest and legs are not changed.
- New option **"Feminine figure (female V)"** in the Camera section of the mod menu (next to "Shoulder width"), on by default. Switch it off to get the figure of 0.1.1 back.

### Notes

- The figure is applied to the poses the mod sets for unarmed standing and movement. With a weapon drawn, while climbing or vaulting and in the animations the game plays itself (scenes, vehicles, finishers) the figure of 0.1.1 is used. The change between the two takes a quarter of a second.
- Male V is not changed.

## 0.1.1 - 2026-11-15

For Cyberpunk 2077 2.5 with Phantom Liberty. Tested with RED4ext 1.31.1, redscript 0.5.32, Cyber Engine Tweaks 1.38.1, Codeware 1.21.4, ArchiveXL 1.28.8, Input Loader 0.2.6 and Mod Settings 0.2.30.

**Updating from 0.1.0:** install 0.1.1 over 0.1.0 and let it replace every file. No file of 0.1.0 has to be removed. At the first start of 0.1.1 the settings of the mod are reset to their defaults once (see Changed).

### Added

- **Melee weapons in third person:** knives, katanas, one-handed blunt weapons, batons and two-handed blunt weapons - light combos, strong attacks, block, strikes while walking, sprint attacks and crouched attacks.
- **Mantis Blades:** light combo, strong attack, safe attack, guard, block and strikes while moving.
- **Gorilla Arms:** the third-person fist strikes and the guard.
- **Fists:** a guard stance between punches, a new heavy third punch, standing strikes that move the hips and legs too, and a turn towards the enemy the game tracks when V strikes.
- **Dual wield (first version):** third-person ready pose, aim and hip-fire stance for two one-handed weapons; the left weapon has its own recoil.
- **Ground dodges** with their own animations. The dodge now goes in the direction you press.
- New file `utpp_melee_parried_graph.archive`: it replaces the base-game graph `player_melee.animgraph`, so that the game picks the parried animation that belongs to each attack. It conflicts with other mods that replace that graph.

### Fixed

- **Animation pack "Game Animations":** V no longer stands frozen in one pose. The game's own animations now play on the third-person body.
- **Shadow:** the whole body casts its shadow again, also with ray-traced shadows, and the shadow follows the third-person pose. A small arc of shadow next to male V's head is gone.
- **Movement with keyboard and mouse:** far fewer abrupt animation changes on quick direction changes, strafing and 180-degree turns; smoother starts from a crouch; much smaller hip jumps when an animation changes.
- **Movement:** the mod's movement no longer stays switched off for the rest of a session after a single slow frame.
- **Slide:** the body no longer twists sharply when a slide changes to another variant.
- **Small steps and drops** in the terrain (up to about 30 cm) no longer interrupt the run or sprint animation.
- **Reloads:** pistol and revolver reloads while walking or starting to move, and submachine gun and rifle reloads, no longer twist the arms or pull the shoulders in.
- **Hip fire** with a light machine gun no longer shows the first-person arm pose before the first shot.
- **Two-handed weapons:** the shoulders stay close to their normal width when V stops, and when V aims a two-handed pistol while crouched.
- **Legs** no longer twist when V stops or starts with a weapon drawn.
- **Melee:** the pose after a strike and after putting a melee weapon away is no longer shrunk; no single frame with wide arms at the start of a punch; blocking with a blunt weapon keeps the elbow in place.
- **Vaulting over a vehicle:** the shoulders keep their width.
- **Head look:** the neck no longer twists too far against the chest when V starts to run.
- **Skin and muscles:** the body skin is recalculated in more situations (arm cyberware, entering and leaving vehicles, forearms and lower legs).
- **Settings:** stored values now reach every option.

### Changed

- The fist guard is on by default.
- "Reload animation during Shinobi bonus" is on by default.
- Scenes and dialogues stay in third person by default: "First person in scenes and dialogues" is off.
- The camera sits further to the side of V by default: the shoulder offset is 1.0 m (was 0.5 m).
- V's body is turned less against the camera by default: the body yaw offset is 4 degrees (was 8) and the run hip trim 2 degrees (was 5).
- "Hide first-person shadow proxy" is off by default (it removed part of the shadow with ray-traced shadows).
- **All settings of the mod are reset to their defaults once**, at the first start of 0.1.1, because the defaults and the way several options work changed. A message on the screen tells you when it happened. The two "Swap shoulder" controls (keyboard key and controller button) are not changed. This happens once: afterwards your settings are kept as before.
- Three options that had no effect are no longer shown in the menu: "Head follows turns", "Transition time (s)" and "Start in the last used view". No option was renamed.

### Known issues

See [Known issues](README.md#known-issues-in-020) in the readme (the list of the current version).

## 0.1.0 - 2026-11-13

First public release: Motion Matching locomotion, body rotation, foot locking, third-person aiming and shooting with recoil and weapon grip, fists, entering and leaving vehicles, shoulder swap, the animation pack choice, skin recalculation for the mod's poses and built-in diagnostic logs.
