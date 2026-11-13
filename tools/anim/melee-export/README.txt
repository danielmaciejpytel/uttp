UTPP melee export (2026-11-09): re-solved melee clips -> third-person, full-body melee sets in game.

What it makes
  utpp_melee.archive with utpp\melee\utpp_<set>.anims for the fourteen vanilla first-person melee sets (default_melee_weapon,
  pma_default_melee_weapon, p{w,m}a_{fists,katana,knife,one_handed_blunt,two_handed_blunt,two_handed_hammer}; hammer added 2026-11-10; also known
  since 2026-11-10 but not in "-Sets all": p{w,m}a_baton and p{w,m}a_strongarms_melee_fists = melee_export_lib.EXTRA_SETS, the default vanilla-frozen
  dir has no json for them, pass -Vanilla with them). Each UTPP set holds only the replaced
  NON-ADDITIVE clips, under the vanilla clip names; the registration is Source\plugin\UTPP\archive\utpp_melee_tpp.archive.xl
  (vanilla vars + utpp_foot_tpp, priority vanilla - 1: third person only, first person unchanged). Nothing is installed by these tools.

One command (PowerShell; every step, gates, pack, unbundle + hash compare, manifest):
  powershell -ExecutionPolicy Bypass -File run_melee_export.ps1 -Input <clips dir> -Rig woman|man|both -Sets <a,b|all> -Out <new dir under Build\work>
             [-Suffix v4] [-Vanilla <frozen vanilla dir>] [-Registration <registration.json>] [-Identity] [-NoPack]
             [-ExtraStaged <dir>] [-Only clip,clip] [-NoSimdCanon]
  -ExtraStaged: the *.anims of <dir> are copied byte for byte into the staging folder (sets built elsewhere, e.g. the installed fists sets: one
             utpp_melee.archive carries them; SHA-256 in run.log). Replaces the npc_blunt_export.ps1 / export_run.ps1 wrapper copies of the earlier per-weapon builds.
  -Only: build only these clips of every selected set (test runs). -NoSimdCanon: TEST ONLY (negative control of test_simd_roundtrip.ps1).
  <clips dir>: <set>__<clip>.<Suffix>.npz (W [T,73,4,4] world matrices of the 73 body joints, GAME axes z up / +Y forward, 30 fps, the dump
  layout of Source\tools\melee\melee_dump.py; Hips under Root) + optional .<Suffix>.info.json (identical_to_v3 -> coverage 'fallback').
  The files of the selected sets are copied to <Out>\input first (input-sha256.txt); only that copy is read.
  -Identity: the vanilla clips themselves go through the pipeline (must reproduce vanilla; run it after any tool change). Expected exception
             since 2026-11-09: vanilla hand helper frames farther than 1.5 cm from the hand are put on the hand (listed per clip in
             reports\build-<set>.json hand_helpers_snapped; pma_fists: 13 tracks / 196 frames in 9 clips); arm_*_ik_weapon follows the
             moved arm_*_ik_offset there (vanilla local).
  Then:  python -I make_melee_xl.py --registration <registration.json> --manifest <Out>\manifest.json --out Source\plugin\UTPP\archive\utpp_melee_tpp.archive.xl --copy-to <Out>\packed
         python -I coverage.py --run <Out> --registration <registration.json> --dumps Build\work\melee-anim-20261107\npz --out <Out>\coverage
  Python = Tools\ai-motion\envs\kimodo\Scripts\python.exe -I (numpy; reuses vehicle-entry skel.py / glbanim.py / exit_steps.py).

One-time inputs (made 2026-11-09 in Build\work\melee-export-20261109)
  evidence\  extract_retail.ps1 (read-only unbundle of every set the four 2.5 player entities register + the entities, serialize) and
             registration_evidence.py (hashes, registrations, clip-name overlap per priority window, exclusions, plan) -> registration.json/.txt.
  vanilla-frozen\  anims\ (retail 2.5 .anims), json\ (serialized), glb\ (export WITH root motion), glb-nort\ (export WITHOUT root motion =
             anim buffer Root). The re-solved clips were made from Extracted\game\vanilla; those GLB exports are byte-identical to the retail
             exports for 10 sets; the two one_handed_blunt sets differ only in the events / tags of the equip clips (data chunks equal).

Per set (run_melee_export.ps1)
  1 build_melee_set.py   GLB with one animation per replaced clip (152 joints, all tracks in seconds):
                         73 body joints from the clip file (except Root / Trajectory / reference_joint = vanilla), the other 79 joints = vanilla
                         locals, limb IK helpers (feet / hands targets, knee / elbow poles) re-anchored on the new limbs on EVERY frame with the
                         vanilla per-frame offset in the anchor frame (exit_steps.reanchor_helpers; helper-to-bone distance = vanilla on every
                         frame), arm_*_ik_offset / _weapon = vanilla locals; then (2026-11-09) every HAND helper (arm_*_ik, arm_*_ik_offset,
                         Torso_(World_)*Arm_Hand_IK_JNT) farther than 1.5 cm from the final hand on a frame is put on the hand on that frame
                         (snap_hand_helpers: hand world pose, arm_*_ik_offset identity local; the game's arm IK follows these helpers, a stale
                         one pulls the hand to it; vanilla first-person frames carry up to 41.8 cm in pma_fists); frames within 1.5 cm are not
                         touched (a set without far helpers builds bit-identical); hard gate: hand helpers <= 1.5 cm on every frame incl. the
                         last (exit 3); report per clip: hand_helpers (max / last / rotation, arm_*_ik_weapon info), hand_helpers_snapped;
                         FPP camera chain untouched (hangs from Root), extras (events,
                         float tracks, types; fallback frames: see below) = vanilla. Built in the ANIM BUFFER frame: the vanilla clip is sampled from the
                         no-root-motion export (Root = buffer Root = identity; every other local is bit-identical to the root-motion export), the
                         GLB Root track is that buffer Root (a Root carrying the root motion is stored in the buffer by the importer and would apply
                         it twice). The clip files have Root = identity on every frame (pose at the frame-0 character frame, character motion
                         removed by the solver) = the buffer Root; hard gate input Root vs buffer Root <= 0.1 cm (else not built). The vanilla
                         root motion (7 fists clips, <= 1.5 cm) stays in motionExtraction and moves the entity with the new body in game.
                         fallbackFrameIndices = [] in every built clip (the set-level fallback frames are the vanilla first-person poses;
                         same as the vehicle-entry builder). SIMD (uncompressed) vanilla buffers: last key time + 1 float32 step, so that the
                         importer's floor(t_last*30)+1 equals numFrames. SIMD clips also get every key quaternion with w >= 0 (q and -q
                         are the same rotation; L.canonical_w_nonneg): the importer returned some w < 0 keys of uncompressed clips with
                         another rotation (Male pma_knife throw_start: LeftUpLeg 18 deg, arm_left_ik 38 deg, left foot helper 24.6 cm);
                         keys with w >= 0 never were. Compressed clips are untouched; report field simd_w_neg_keys. The sign flip puts
                         sign jumps between neighbouring keys where w crosses 0 (the game's own SIMD clips have them too: pwa 118, pma 27;
                         the verifier samples keys only and treats q / -q as equal; playback in game not verified).
                         Not built: additive clips, clips without input, exclusions of registration.json, degenerate inputs
                         (pma_katana/finisher_katana_02), clips whose duration != (numFrames-1)/30, --reject-json clips (retry, below).
  2 animset_subset.py    vanilla set reduced to the built clips (base of the import; all data chunks kept).
  3 WkitBridge import    (Build\work\vehicle-entry-car\bridge-bin\WkitBridge.exe; WolvenKit import onto the base, clips with matching names).
  4 patch_melee_json.py  motionExtraction verbatim from vanilla (importer bug: keys for all joints, no axis conversion), every other differing
                         field of the clip entry (events of 10 / 74 fists clips were rewritten by the importer) back to vanilla, numFrames back
                         to vanilla (importer: floor(duration*30)+1 is one short when duration*30 = 53.9999985; keys are time based),
                         fallbackFrameIndices = []. A SIMD buffer whose imported frame count differs is NOT restored (its frame data
                         would be read past the end: WolvenKit 'Unable to read beyond the end of the stream', pwa/pma_knife throw_start in
                         run full-v4); it is listed as unpatchable, exit 4.
  5 deserialize -> <Out>\utpp_melee\utpp\melee\utpp_<set>.anims, serialize again (final-json), export with and without root motion.
  6 verify_melee_set.py  per clip, every frame: (a) local transform of every joint round trip vs built <= 1 mm / 0.15 deg (Root excluded,
                         see b / f); (b) root motion (motion extraction) == vanilla; (c) HAND IK helpers vs hand <= 1.5 cm + 0.05 cm
                         (round-trip compression) on every frame incl. the last, no vanilla allowance (2026-11-09); FOOT helpers <= 1.5 cm or
                         the vanilla distance of that frame + 0.1 cm (inherited frames listed); (d) FPP camera chain == vanilla; (e) events, float
                         tracks (key time tolerance 1 ms: the importer re-quantises constant-key times by ~1e-5 s), types ==
                         vanilla; clip set / rig / numFrames / duration / every json field except animBuffer == vanilla; (f) anim buffer Root ==
                         vanilla; fallback frames empty (round-trip extras and final json).
  Retry (per clip): the clips that failed (verify, or patch exit 4) are collected and the set is rebuilt ONCE without them
                         (reports\reject-<set>.json -> build --reject-json; the clips stay vanilla; gates unchanged); attempt 1 is kept in
                         <Out>\attempt1\, manifest set_details lists attempts and rejected clips. A set that still fails is moved out of the
                         pack folder (rejected-utpp_<set>.anims) and stays vanilla.
  Pack: WolvenKit pack <Out>\utpp_melee -> <Out>\packed\utpp_melee.archive, unbundle -> <Out>\unpacked, sha256 of every file == staged,
  <Out>\manifest.json (archive sha256 / size, staged files sha256 / size, per-set result, tool hashes, registration.json sha256).

Weapon IK helpers (2026-11-10): vanilla melee sets keep arm_right_ik_weapon on WeaponRight and arm_left_ik_weapon on WeaponLeft (0-1.4 cm).
  For the sets in melee_export_lib.WEAPON_HELPER_SETS build_melee_set.snap_weapon_helpers puts arm_*_ik_weapon on its weapon joint on every
  frame farther than 1.5 cm (frames within the limit are not touched) and verify_melee_set gates it (<= 1.5 cm + 0.05 cm, every frame):
    p?a_two_handed_hammer (sliding two-hand grip), p?a_one_handed_blunt and p?a_baton (NPC grip: helpers up to 7 cm, on throws 46 cm off the joint),
    default_melee_weapon and pma_default_melee_weapon (game convention <= 1.4 cm; the fists-based default sets
    reached 99.8 cm), p?a_knife and p?a_katana (the b1 builds measured 7.2 / 8.1 cm off the weapon joint,
    vanilla ~1 cm).
  NOT in the list: p?a_fists and p?a_strongarms_melee_fists (accepted in game; their exports are byte-identical to the installed ones: the build GLBs
    of the six combined-fists sets, rebuilt with these tools, equal the recorded run; see "Merge 2026-11-10" below), two_handed_blunt. There the distance is only reported (ik_weapon_vs_weapon_joint_cm). Adding a set = one name in WEAPON_HELPER_SETS;
    that changes the bytes of that set's export (the installed default sets of the combined fists archive are therefore rebuilt by design).

UTPP-only clip names (2026-11-10; replaces the earlier per-run vanilla-aug folders): the pipeline works per clip name of the vanilla set, a name
  the vanilla set does not have was skipped. melee_export_lib.UTPP_CLIP_TEMPLATES is the allowlist: parried_(left|right)_(combo_NN | final_NN |
  heavy_NN | block | safe | crouch | jump | sprint | equip) -> template clip parried_<side> (the graph patch melee-parried-graph-20261110 picks
  parried_<side>_<attack> per attack when the set holds both sides). Before the sets run, run_melee_export.ps1 calls aug_vanilla.py: for every
  input clip <set>__<name>.<Suffix>.npz that the vanilla set lacks and the allowlist accepts, a clone of the template clip (same buffer, events,
  duration, motion extraction; fresh HandleId / BufferId) is added to a copy of the vanilla json / glb / glb-nort in <Out>\vanilla-aug, which is then
  used as -Vanilla (manifest: vanilla = used, vanilla_given = as passed). The clone is only the clip's identity in the set: animset_subset keeps it,
  the importer takes the input's animation, patch_melee_json copies the template's metadata, verify_melee_set checks against it. Already augmented
  -Vanilla dirs (the earlier vanilla-aug) still work: nothing is cloned. Unknown names that are not allowlisted stay unbuilt, as before (AUG_IGNORED
  in the log, build report input_not_in_vanilla; e.g. the NPC-only guard_idle_loop); an allowlisted name whose template is missing in the set is an
  error (aug_vanilla exit 2); patch_melee_json / verify_melee_set still refuse any clip without a vanilla entry. New patterns: add one line to
  UTPP_CLIP_TEMPLATES (+ test_melee_export.py cases). The .archive.xl is unchanged: it registers sets, not clips.

Tests (2026-11-10)
  python -I test_melee_export.py [--vanilla <frozen vanilla dir>]   offline, ~10 s: canonical_w_nonneg, allowlist accept / refuse, aug_vanilla plan and
      end-to-end clone check on pma_knife (json / glb / glb-nort, unique ids, vanilla untouched), WEAPON_HELPER_SETS content, SETS table.
  powershell -File test_simd_roundtrip.ps1 -Out <new dir under Build\work>   real WolvenKit import round trip of pma_knife / throw_start (Male, SIMD,
      keys with w < 0; fixture = the knife build's pack-input, sha256 checked): canonical signs must pass the verifier, -NoSimdCanon (the old
      handling) must fail it with a local rotation error > 1 deg. Run it after any change of the build / import / patch / verify steps.
  Identity: run_melee_export.ps1 -Identity (vanilla through the pipeline) as before; the fists identity of the merge: see below.

Registration (registration_evidence.py; details in the .xl comment)
  Mirrored: default sets 200 [] -> 199 [utpp_foot_tpp]; weapon sets 128 [w_melee_*|w_two_hand_blunt|w_melee_hammer] -> 127 [same, utpp_foot_tpp]; base and
  ep1 entity of each rig, component root. Not mirrored: [StrongArms] (fists), [additives] (one-handed blunt), [knife_thumb_override] (knife).
  Excluded clip names (stay vanilla): melee_first_equip (the ten weapon sets: the 127 [dildo] set holds it), vault (male default set:
  pma_default_weapon 200 []). Held pose clips (< 6 frames) have no re-solved version (redo_batch.py skips them) and stay vanilla too.
  Classes in registration.txt: tie / always-same -> excluded; modifier-same -> listed risk; exclusive (another weapon variable); below / above.

Re-run for new clips (e.g. a new animation round in a new folder):
  run_melee_export.ps1 -Input <new clips dir> -Suffix <their suffix> -Rig both -Sets all -Out Build\work\<new run>;
  then make_melee_xl.py (only packed sets are registered) and coverage.py. Run registration_evidence.py --clips-dir <new dir> --suffix <s> again
  when the set of clips changes (the exclusions are clip names, not files).

Merge 2026-11-10 (knife / blunt+baton / katana b1 builds; proofs in Build\work\melee-export-merge-20261110)
  Merged here: SIMD w >= 0 (knife); UTPP-only clip names (allowlist + aug_vanilla.py in the run, knife / blunt / katana); WEAPON_HELPER_SETS += one-handed
  blunt, baton (blunt) and both default sets; baton / StrongArms sets natively in melee_export_lib (blunt + combined-fists shims);
  w_melee_baton in registration_evidence.PRIMARY; -ExtraStaged / -Only / -NoSimdCanon in run_melee_export.ps1; coverage.py skips sets without dumps.
  Identity of the accepted fists exports (combined fists archive, 6 sets, npc1 clips-in of melee-fists-combined-20261109):
    build GLBs of pwa_fists, pma_fists, pwa/pma_strongarms_melee_fists = the recorded run's out_glb_sha256 (the SIMD fix and the helper list do not touch them);
    full run_melee_export.ps1 re-export: the .anims of those four sets are byte-identical to the installed ones (464FBB01.., A420426D.., 2C4249B2.., C854BBD6..);
    make_melee_xl.py on that manifest + the same registration.json = utpp_melee_tpp.archive.xl F871E232.. (identical).
    The utpp_melee.archive hash itself is not reproducible: WolvenKit pack of the very same six .anims gives another SHA-256 on every run (38 bytes differ,
    size equal 4 530 176 B); compare the unbundled .anims, not the archive.
  Changed by design: utpp_default_melee_weapon / utpp_pma_default_melee_weapon (installed 95E8A38B.. / BE6FA1E9..): arm_*_ik_weapon was up to 99.8 cm
    (right 20.4 cm) off its weapon joint, now <= 0.61 cm (6 of 9 clips snapped on 34-64 frames); the new files are 774871B4.. / FB334C4D.. (npc1 inputs).
  Not merged: the b1 builders (b1_build.py / b1k_build.py / kbuild.py: --parried-new, --pnew-donor, --parried-donor-order, hold2, block-k, ...) are
    wrappers of Source\tools\melee\npc_blade_build.py / npc_blunt_build.py and live in the earlier per-weapon scripts folders; they are not exporter code.
    The wrappers Source\tools\melee\npc_blunt_export.ps1 (+ .py shim) patch run_melee_export.ps1 text by exact match and have been broken since the hammer
    commit; they are replaced by the options above (-ExtraStaged, native baton / StrongArms, native clip names) and should be retired.
