graph-patch — per-attack Parried clips in player_melee.animgraph (Cyberpunk 2077 2.5), offline only
=====================================================================================================
What: the melee graph's Parried state chooses parried_left / parried_right only by hit_direction. The patch puts a nested state
machine under Parried's Output (vanilla pattern of the Ready/Idle nested SM: default state + conditional entries with HasAnimation):
  state 0 = the untouched vanilla Switch (default, FPP and every set without new clips),
  states 1..17 = clones of that Switch with parried_<left|right>_<suffix> leaves,
  entry (multi-clip types) = WrapperValue utpp_foot_tpp set AND MeleeSlotData.attackType == T AND MeleeSlotData.comboNumber == N
  AND HasAnimation of both clips. Table: Combo 0..3 -> combo_01..04, Final 0..4 -> final_01..05, Strong even/odd -> heavy_01/02.
  entry (single-clip types, since run5) = the same WITHOUT comboNumber: Block 3 -> block, Safe 4 -> safe, Crouch 6 -> crouch,
  Jump 7 -> jump, Sprint 8 -> sprint, Equip 12 -> equip (game anim database player_melee_attacks.csv 2.5: "<type>;*;<x>_attack").
Clip names for weapon sets (always both sides; a missing pair = generic parried_left/right for that attack only):
  parried_left|right_combo_01..04, _final_01..05, _heavy_01..02, _block, _safe, _crouch, _jump, _sprint, _equip   (34 names)
  Each per-type clip starts from the pose of its attack clip at the parry cut (block_attack, safe_attack, crouch_attack,
  jump_attack, sprint_attack, equip_attack) and ends like parried_<side>. Not covered (always generic): Slide 5, Fall 9,
  Throw 10, Dodge 11, deflect attack (sends Combo/0 -> picks combo_01 when present), ground slam (sends no MeleeSlotData).
  Exit: graph Parried -> Ready only on MeleeData.isParried == 0 (0.15 s blend), not on clip end; the clip is not looped.
A weapon opts in by putting both clip names of an entry into its UTPP .anims set (no .xl change).

Files
  patch_parried_combo.py   in.json out.json [--report r.json] [--no-type-entries]   refuses non-vanilla input (Parried shape,
                           UTPP names present); --no-type-entries = run3 graph (byte-identical JSON, graph SHA 7C78A1DF...)
  parried_graph_check.py   inspect <patched.json> | diff <vanilla.json> <patched.json> | simulate <patched.json>
                           | compare <older patched.json> <patched.json>  (old states/entries unchanged + selection sweep)
  pack_parried_graph.ps1   full build: pinned vanilla SHA, control round trip, patch, checks, CR2W round trips, pack, unbundle;
                           optional -CompareWith <older patched JSON> adds the compare check
  json_first_diff.py       diagnostic: first differences of two WolvenKit JSON files (canonical handle ids)

Build (Python = Tools\ai-motion\envs\kimodo\Scripts\python.exe, WolvenKit = Tools\WolvenKit.CLI):
  powershell -NoProfile -ExecutionPolicy Bypass -File pack_parried_graph.ps1 -RawGraph <extracted player_melee.animgraph> -OutRoot <dir> -Run <name>
  Output <OutRoot>\<Run>\packed\utpp_melee_parried_graph.archive + manifest.json, inspect/diff/simulate reports. Never installs.

RULE: after EVERY game patch re-extract player_melee.animgraph (content\basegame_1_engine.archive in 2.5). A SHA mismatch stops the
build; an installed archive is then STALE (it replaces the new game graph with the old one). Review the Parried state, update the
pinned SHA deliberately, rebuild. Runtime behaviour is not proven by this build (see Build\work\melee-parried-graph-20261110\report.txt).
