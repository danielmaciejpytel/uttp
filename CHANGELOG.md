# Changelog

Changes of Ultimate Third Person Project, newest version first.

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
- The UTPP scripts compile with redscript 1.0.0. redscript 1.0.0 is still not supported, because the frameworks UTPP needs were not ready for it when this version was built: keep using redscript 0.5.32.

### Known issues

See [Known issues in 0.1.1](README.md#known-issues-in-011) in the readme.

## 0.1.0 - 2026-11-13

First public release: Motion Matching locomotion, body rotation, foot locking, third-person aiming and shooting with recoil and weapon grip, fists, entering and leaving vehicles, shoulder swap, the animation pack choice, skin recalculation for the mod's poses and built-in diagnostic logs.
