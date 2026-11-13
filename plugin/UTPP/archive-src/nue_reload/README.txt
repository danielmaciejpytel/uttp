utpp_nue_reload archive - build notes
=====================================

The archive OVERRIDES two whole base graphs of the game:
  base\gameplay\anim_graphs\player_ranged.animgraph
  base\gameplay\anim_graphs\weapon.animgraph
plus two own clip sets (utpp\reload\*.anims, registered by utpp_nue_reload.archive.xl).
The UTPP part is only a delta (bool gates UTPPNueReload / UTPPNueReloadAim / UTPPNueReload2H around the native
EMPTY and NORMAL reload leaves). Everything else in those graphs is the game's own content of ONE game version.

RULE: after EVERY game patch rerun build_nue_reload_250.ps1 (offline, read-only on the game folder).
  - It checks Cyberpunk2077.exe (ProductVersion + SHA256) and freshly extracts both base graphs from all game
    archives. A hash mismatch ("GAME CHANGED") means the INSTALLED archive is STALE: it replaces the new graphs
    with old ones (in 2.5 this broke every reload: RightReload/EndRightReload were never received).
  - Until rebuilt, disable it: Source\tools\install\disable_utpp_archive.ps1 -ArchiveName utpp_nue_reload
    (dry run first; -Apply only with the game closed).
  - Then review the new graphs (tools\graph_inspect.py --leaves / --names, tools\graph_conditions.py), make sure
    the patcher still finds 6 EMPTY + 3 NORMAL SkSpeedAnim leaves, update the pinned hashes in $expected and
    rebuild. tools\graph_delta_check.py must report pass=true (same delta as the previous version).

Build:   powershell -NoProfile -ExecutionPolicy Bypass -File build_nue_reload_250.ps1
Output:  Build\stages\nue-reload-250\archive\pc\mod\utpp_nue_reload.archive(.xl), manifest.json, reports\,
         Research\inventory\utpp-nue-reload-250-stage-20261023.json
Install: Source\tools\install\install_verified_stage.ps1 -StageName nue-reload-250
         -ManifestName utpp-nue-reload-250-stage-20261023.json   (dry run; -Apply only with the game closed)
Runtime behaviour (reload animation, magazine, gate) is never proven by this build; test in game.

Male V (2026-11-06, offline, not installed, gates still female-only)
  Male body clip set: utpp\reload\utpp_nue_tpp_reload_man.anims (rig player_man_skeleton), same 8 clip names, same timeline
  (81 / 101 frames, same events/float tracks) as the female set; the magazine set stays shared on weapon_root.ent.
  Source/retarget/verify: Source\tools\analysis\retarget_nue_reload_man.py, verify_nue_reload_man_roundtrip.py,
  render_nue_reload_man.py; pma source = 2.5 basegame_1_engine copy (Extracted\game\glb\weapon-grip-vanilla-man).
  Build (one archive + one combined .xl, replaces the female-only utpp_nue_reload):
    powershell -NoProfile -ExecutionPolicy Bypass -File build_nue_reload_250.ps1 -StageName nue-reload-male-20261106
      -XlSource Source\plugin\UTPP\archive-src\nue_reload\utpp_nue_reload_male.archive.xl -XlSha256 <pin>
      -MaleAnims Animations\Blender\Pistol1H\nue-male-20261106\utpp_nue_tpp_reload_man.anims -MaleAnimsSha256 <pin>
      -InventoryPath Research\inventory\nue-reload-male-20261106.json
  Without the new parameters the script builds the female-only archive exactly as before.
