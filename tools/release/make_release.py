#!/usr/bin/env python
"""UTPP release packer (Nexus Mods; the version is VERSION below).

Usage (always `python -I`):
  python -I Source\\tools\\release\\make_release.py --code-tree <repo or worktree with plugin\\UTPP>
         --dll <UTPP.dll> --out <Build\\work\\release-0.1.0> [--dry-run]

What it does
  * builds the stage from an explicit ship list (data in this script), never from a copy of the game folder;
  * data files (animation databases etc.) are COPIED (shutil.copy2, never moved, never hard-linked) and every SHA-256
    is checked against the value pinned below. The source of a data file is found by CONTENT, not by name (find_source):
    first any file of the pinned size under the project's Data folder, then the same package path in the game folder (a game
    that already has this layout), then the game's red4ext\\plugins\\UTPP and archive\\pc\\mod folders (only read), whose
    SHA-256 equals the pin. A file whose hash is not the pinned one is never used;
  * skin helper data (skin_helpers\\mode.txt + man/woman program.ir and binding.dat) is copied from Build\\test-data\\body_helper_<body>
    (--skin-data-root); its hashes are not pinned here: the audit compares them with the table kApproved parsed from
    <code-tree>\\plugin\\UTPP\\src\\mm\\BodyHelperAdmission.hpp and checks that the shipped DLL contains those hash strings;
  * code files (.lua, .reds, .xl) come from --code-tree (plugin\\UTPP\\...), the DLL from --dll; their hashes are
    computed from the input and recorded in the manifest, together with the git commit of --code-tree;
  * C10: the licence/generator comments in motion chooser JSON and style-weights copies are replaced with neutral text
    (the loaders read none of those fields; see the notes at TRANSFORMS);
  * writes README.txt and THIRD_PARTY_NOTICES.txt (full MIT texts copied unchanged from Tools\\repos);
  * scans the stage (personal data, tool names, probes, development paths, licence leftovers, player-visible text,
    diagnostic defaults) and fails on any hit;
  * writes <out>\\UTPP-<version>.zip (ZIP_DEFLATED, root = game folder), unpacks it to a scratch dir and compares hashes;
    --stage-dir puts the stage (the game-folder tree that install_verified_stage.ps1 installs) elsewhere than <out>\\stage,
    e.g. Build\\stages\\utpp-0.1.0 (inside the project only);
  * writes the manifest (default Research\\inventory\\utpp-release-010-game250.json: no dots, so that install_verified_stage.ps1
    -ManifestName accepts it). The file
    utpp-release-0.1.0.json (record of the unpublished 2.31 package) is never overwritten; any other existing manifest
    is refused unless --overwrite-manifest is given.
Day-of-build groups (melee fists, car entry/exit): pinned, but a final run needs an explicit `--day-of-build
melee=ship|skip,car=ship|skip` (what did not pass the in-game test that day is left out); see DAYOF_GROUPS.
Parried graph (optional, melee group): utpp_melee_parried_graph.archive (modified copy of the game's player_melee.animgraph, pinned
SHA below) ships only when a Parried-bit (32) line of the shipped utpp_melee.reds belongs to a weapon with per-strike parried clips
(PARRIED_CLIPS, keyed by the pinned melee archive SHA); fists = generic only, so 0.1.0 does not include it. See parried_graph_rule.
--early-pose FILE --early-pose-sha HEX (optional): also packs the pose owner switch file red4ext\\plugins\\UTPP\\early-pose.txt
(copy of FILE, pinned, parsed as the plugin parses it); without it no early pose file of any name may ship.
--dry-run: nothing is staged or zipped; every source is hashed, transformed in memory and scanned in place, and the
report/manifest go to <out>\\dryrun\\. Exit code 0 = no FAIL finding, 1 = at least one FAIL, 2 = bad input.
"""
import argparse
import datetime
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path

VERSION = "0.1.1"
GAME_VERSION = "2.50"
AUTHOR = "Daniel Maciej Pytel"
# NVIDIA attribution in THIRD_PARTY_NOTICES (L3). MotionBricks clips are in the motion databases, in
# utpp_vehicle_car.archive (day-of-build group "car", see DAYOF below) and in utpp_melee.archive (group "melee"). Set
# NOTICE_MOTIONBRICKS = False only when none of those shipped files contains a MotionBricks clip (e.g. the motion databases
# without them and --day-of-build car=skip,melee=skip).
# Kimodo: named together with MotionBricks in the licence texts (Source\LICENSE, THIRD_PARTY_NOTICES.txt, README.md).
NOTICE_MOTIONBRICKS = True
NOTICE_KIMODO = True
# <project>\Source\tools\release\make_release.py, or a worktree <project>\Build\<wt>\tools\release\make_release.py
ROOT = next((p for p in Path(__file__).resolve().parents if (p / "Tools" / "repos").is_dir()), Path(__file__).resolve().parents[3])
TOOLS_REPOS = ROOT / "Tools" / "repos"


def _default_game():
    """Game folder: environment variable UTPP_GAME_DIR, else "game_dir" in the untracked tools/local_paths.json (see tools/utpp_paths.py;
    a worktree also looks in <project>/Source/tools). Empty string when it is not configured: pass --game."""
    v = os.environ.get("UTPP_GAME_DIR")
    for tools in (Path(__file__).resolve().parents[1], ROOT / "Source" / "tools"):
        if v:
            break
        try:
            v = json.loads((tools / "local_paths.json").read_text(encoding="utf-8")).get("game_dir")
        except (OSError, ValueError, AttributeError):
            v = None
    return v or ""


DEFAULT_GAME = _default_game()
OLD_DLL_SHA = "5a291a881071d5c2828519ea7026f6af5559f448ae9af5c286d283806137ab88"  # 0.6.5 build, PDB path + old version

P_DLL = "red4ext/plugins/UTPP/"
P_DATA = P_DLL + "data/"
P_REC = P_DLL + "data/recoil/"
P_REDS = "r6/scripts/UTPP/"
P_INPUT = "r6/input/"
P_CET = "bin/x64/plugins/cyber_engine_tweaks/mods/"
P_ARC = "archive/pc/mod/"
# Skin helper data: the DLL recomputes the helper bones of the player body from these files (one bundle per body). The bytes of each
# bundle file are NOT pinned here: the approved SHA-256 values are compiled into the DLL from BodyHelperAdmission.hpp (table kApproved)
# and the audit (check_skin_helpers) compares the shipped files with the table parsed from the code tree at packaging time.
P_SKIN = P_DLL + "skin_helpers/"
SKIN_MODE_REL = P_SKIN + "mode.txt"
SKIN_MODE_BYTES = b"APPLY\n"
SKIN_BODIES = ("man", "woman")
SKIN_BUNDLE_FILES = ("program.ir", "binding.dat")
SKIN_HEADER_SUB = "src/mm/BodyHelperAdmission.hpp"   # under <code-tree>\plugin\UTPP
SKIN_SOURCE_DIR = {b: "body_helper_" + b for b in SKIN_BODIES}   # under <skin data root> (default <project>\Build\test-data)
SKIN_RELS = [SKIN_MODE_REL] + [P_SKIN + b + "/" + f for b in SKIN_BODIES for f in SKIN_BUNDLE_FILES]
OLD_SKIN_FOLDER = "helper_candidate"   # test-install name of the folder; refused everywhere in the package

# ---------------------------------------------------------------------------------------------- ship list: data
# (path in the package, size, sha256). Measured 2026-10-22; re-pinned 2026-11-10 to the game copy
# = stage Build\stages\combined-test-20261110 (installed and tested 2026-11-10): utpp_base.utppmm/.airz, both motion .utppmm, both
# .utpptrav, utpp_female_tpp_nails_plane.archive and the TRANSFORMS sources. Except
# utpp_weapon_magazine_fix.archive (2026-11-01, game copy = install manifest utpp-install-weapon-decal-fix-20261028-025658.json).
# The 2026-11-11 renames changed the package layout (data\ folder, new file names) but not the content of any pinned data file except
# the style weights (TRANSFORMS): the pins below are the content that was tested in game, found by hash (find_source).
# No early pose file ships (round 2, 2026-11-11): the tested behaviour is "experiment off" (the DLL reads only early-pose.txt).
# Explicit opt-in only (--early-pose FILE --early-pose-sha HEX, see EARLY_POSE_REL below): the switch file that selects the early
# mapping pose owner, copied from FILE, pinned to HEX and parsed as the DLL parses it. Without the option nothing changes.
DATA = [
    (P_DLL + "data/utpp_base.utppmm", 176048992, "c4886150a1eb774cc3eed8c30e629ace20194d5fc101e6d6f4cd6f4b77ecb784"),
    (P_DLL + "data/utpp_base.airz", 29252, "89873c49674a31b33757db42c272427b8f33816e492037d189eac526d3cf82aa"),
    (P_DATA + "utpp_motion_woman.utppmm", 187894912, "3e91a026f59a19a8af9734b94ed89f02a14e1a62ebfab0d8131e64e040d8d88a"),
    (P_DATA + "utpp_motion_man.utppmm", 187894928, "932c2452d6a03957b1d275e3ea60503e18a521e01fbb749cd29adbf7c04f5e15"),
    (P_DATA + "utpp_overlays_woman.utppov", 3829312, "e7e5e88ecf4770ff964a70505f79008c65d3e9f9033e741f6ccb5bf4ae8f3a23"),
    (P_DATA + "utpp_overlays_man.utppov", 3829312, "8f2c1d085e9793424c0c7127ac82163b6265921d14286284a99fd83a8e8e5545"),
    (P_DATA + "utpp_lean_woman.bin", 98384, "408691ec73f9e7fe3cee3b69bb1f7bc2ecbf6d0e610400413afd78c0b32eb193"),
    (P_DATA + "utpp_lean_man.bin", 98384, "5b0281a21749ac2442e171efd2c409159eb57169872c783f7f531ec910619f50"),
    (P_DATA + "utpp_traversal_woman.utpptrav", 16071474, "473871a0fded917c133b3521de9cb0dc153658049ddfa8263c71b39ea8c25688"),
    (P_DATA + "utpp_traversal_man.utpptrav", 16074696, "c7620f82a795bca6125229eeefa91174921418bef3598609a6630163ebce79fb"),
    (P_DATA + "utpp_traversal_table.json", 15013, "bbd5e0d32928fe91bc770bfdb1f2cc6509529f3e5474cf1183c80b4ea416ec9b"),
    # ground dodge clips, four per body (tools/mmbake/dodgebake.py); the driver fails closed without them
    (P_DATA + "utpp_dodge_woman.utpptrav", 376690, "415add6b929d5c7b33df424a05b30767ed29bda5eb1676bb280bc6327a3a8901"),
    (P_DATA + "utpp_dodge_man.utpptrav", 376069, "4816f3b2546e81cb6f349f894abf080518cba7ba00ccbb736805fd612f0d0703"),
    (P_ARC + "utpp_camera.archive", 8192, "3f85260e895fe8aedf738aa3030c60372b82c973ef0802b04cd5ffbaba66dbe6"),
    (P_ARC + "utpp_female_tpp_nails_plane.archive", 131072, "7475da5a7101fe46876c2acf41301909ae45968c679db7b4a73b052e2c1ec4ff"),
    (P_ARC + "utpp_weapon_magazine_fix.archive", 1089536, "b52cc3233adc52843b28483107579d575959aee4f2ea5ede1c91f65633ea8663"),
    # Vehicle archive utpp_vehicle_bike.archive (release build: motorcycle workspot override + 12 enter clips, no .xl),
    # checked in game on 2026-11-10 and pinned to that game copy.
    (P_ARC + "utpp_vehicle_bike.archive", 2105344, "9f6a1d1bdcbb89de14805dc9f4246601dc19f9f66d86912f0c3d65acb756746a"),
]
# The car entry archive is a day-of-build item (DAYOF below), pinned to the game copy of 2026-11-09.
# Files that ship only once pinned: (path, note). A dry run reports them as pending (WARN); a final run FAILS while any is
# listed, so a release can neither ship an unpinned file nor silently leave it out. Empty while nothing is pending.
DATA_PENDING = []
# Nue authored reload (female V + Nue pistol). The archive replaces the base-game player_ranged / weapon animgraphs.
# Pin = game copy of 2026-10-25 14:42 (install manifest utpp-install-nue-reload-250-20261025-144327.json).
DATA_NUE = [
    (P_ARC + "utpp_nue_reload.archive", 2445312, "e68522e65c247af3bb30e1696d5c7946a1593693deb0ac7d749660746623344f"),
]
# motion files whose licence/generator comments are neutralised (C10): (path, size, sha256 of the SOURCE);
# the sources of 2026-11-10 are already neutral (choosers: same tables, licence / generator / source / database_file fields neutral).
# Style weights: re-pinned 2026-11-11 to the files regenerated for the renamed keywords (Neutral / primary / other, every weight 1;
# Data\overlay\utpp_style_weights_<body>.txt, 263 B, identical copy in Data\motion); the 398 B file of the earlier install has the old
# keywords and is not read by the current parser.
TRANSFORMS = [
    (P_DATA + "utpp_motion_woman.chooser.json", 48985, "fe19292bf7df6dba560a43de6013d6caf8d7c688c4899eb69cd2bd520ad76d85"),
    (P_DATA + "utpp_motion_man.chooser.json", 48980, "2e04252569ffd423c0476ffd685b34854987d70e24da547e7d6fafceb4d67cca"),
    (P_DATA + "utpp_style_weights_woman.txt", 263, "5e01144a44cb2a5f20abb42c5e6872503fa764084fc23d6dcac72e2828990a92"),
    (P_DATA + "utpp_style_weights_man.txt", 263, "5e01144a44cb2a5f20abb42c5e6872503fa764084fc23d6dcac72e2828990a92"),
]
# path in the stage -> project file used when the game folder has no copy at the ship path (pin checked all the same)
TRANSFORM_PROJECT_SRC = {
    P_DATA + "utpp_style_weights_woman.txt": ROOT / "Data" / "motion" / "utpp_style_weights_woman.txt",
    P_DATA + "utpp_style_weights_man.txt": ROOT / "Data" / "motion" / "utpp_style_weights_man.txt",
}
# 24 weapon families x {shoulder, ironsight}; recoil profiles extracted from the game's own weapon animations (2.5 additions: recoil-missing-20261109).
RECOIL = """\
add_recoil_single_shot_ironsight.utpprecoil 11435 66cfe9f967edb800e8179648a33c725b04c63bcd52b85528428216f6534a352a
add_recoil_single_shot_shoulder.utpprecoil 11304 06c82dd6ed1dd879b41a4a2217b5787bc7289dd82a6c283f9f2c56e289ee2b0e
ajax_single_shot_ironsight.utpprecoil 11587 4298f6a595c9b4c558ea11459fe012511644c857de06a0ffc1f16bb7415f4d82
ajax_single_shot_shoulder.utpprecoil 11449 e676436ba48c8ff8e65c85b258d638de299fd970a750152735b0be9bb2e7cbc9
borg4a_single_shot_ironsight.utpprecoil 11481 90a352e41394294b53101dba26a7a46960b228ab92fd8c9bb6a7dfdc4ed57cba
borg4a_single_shot_shoulder.utpprecoil 11388 6c150b3dedbe8756eee605d94de1c8e0ad34365444636e52733c75011964d6fc
chao_single_shot_ironsight.utpprecoil 10983 89dc33277a6fd1c4eb65f0a53f962642b3b41a830917eb0da45d21d1f8aac227
chao_single_shot_shoulder.utpprecoil 11695 9341416264fbb99b35f27a417bf9a629c8453fbc2663a411df5e77ff7a41c973
copperhead_single_shot_ironsight.utpprecoil 11855 289c774668412ab0733cd40ad8c50f07b5fcd6c89839d574c576bf8b95d24056
copperhead_single_shot_shoulder.utpprecoil 11635 24408e7b3c321ffc4a578f8e80a08b8575c5839903743006c3487165f3766477
defender_single_shot_ironsight.utpprecoil 11605 09c1557491828fe410f5a1cab1d0a3d42d7363acd597c8d4104d3d01c6986c56
defender_single_shot_shoulder.utpprecoil 11364 d3a0d277b7dbd171cad9c699ecf2f6c4ba58393faf951cd431f6792339ebee89
dian_single_shot_ironsight.utpprecoil 11711 e292b155387a7f9b6e4517018c1c5788b7834fb541e02e07a7100d853e65cfe6
dian_single_shot_shoulder.utpprecoil 11889 00f74d7a05d6b5f8f44c2e6672616a817126e122d92511dd27fcad906c63d9b6
hercules_single_shot_ironsight.utpprecoil 11643 3ec594bff38961f552750f044db1860518bd12b71e69687d1d4637e4dcc3c587
hercules_single_shot_shoulder.utpprecoil 11551 db44aefd22fabfdb6617b8bdd686cca2f640511a6e3b26ac7bf0d05aa0214e78
kenshin_single_shot_ironsight.utpprecoil 11665 115879819926884b15fb9794ccb95b8238a0bd2b4f766f57a787bf991c1848bd
kenshin_single_shot_shoulder.utpprecoil 11691 92e2a1feab46f43ea2ab27aaced8605ca2c6babf96dabae541627a37239911bb
lexington_single_shot_ironsight.utpprecoil 11846 f9bb3e36aabaf53784b541d9b391f7c6ac89bb94828871bf61eddd3bdc312f59
lexington_single_shot_shoulder.utpprecoil 11785 9611d236b7547d1af298fd140def9d3dcde120526749c2d7a0b6a8a42c1189bf
liberty_single_shot_ironsight.utpprecoil 11580 62bc73a89656b80aa1b41e9473cf37801156d0d7b574a571ea72e704bcd9145c
liberty_single_shot_shoulder.utpprecoil 11508 b380a599ce7dd44e35065a853f6f72472f90d873b28464bd602060b21b4ccf5d
masamune_single_shot_ironsight.utpprecoil 11298 83ac4f13be27533147f00ea0d9b2d6f0fbb728a3711cf4a396abe4ec1aa83ae3
masamune_single_shot_shoulder.utpprecoil 11368 9366fc7c50ad11299afafcdb5e783a9891b9baf6c12abd86dd2e37f6c156f50d
omaha_single_shot_ironsight.utpprecoil 11679 5a0d2d8cc4a1ec380afd79606a1bbfe876e20ab77629c8b6b006b90059b01a65
omaha_single_shot_shoulder.utpprecoil 11571 3b854fed1c281be3ec0cd1356ae1a5a0e992b2e059264c270aad29f3f1568d02
pulsar_single_shot_ironsight.utpprecoil 11653 f4af3e135f13080118ee33a2ce0eb40b4c5d1b29a7ddcfeaf78ab97fac44a540
pulsar_single_shot_shoulder.utpprecoil 11826 4345ccb8465f6acc3915e166089e140f521757e985e6745d20f4e4e32ce0136e
quasar_single_shot_ironsight.utpprecoil 11691 4706ed08a9466f12c6352add7b5691cfeccd58025fe7628e057a6711ca136f83
quasar_single_shot_shoulder.utpprecoil 11416 16cf1dd847ac1549efc9d62cf27ce97a49cae5b7b7ce0ae126051499a9a58fc4
saratoga_single_shot_ironsight.utpprecoil 11676 c1825dcbea9638ea4ad2c69eb75f13ca0387bf29f47d22ec41c586e17fd64b90
saratoga_single_shot_shoulder.utpprecoil 11775 4808c5275f6e34a4efcf0f84590fd108bdab87ed5b36de7bb6710adc3a8f48bf
senkoh_single_shot_ironsight.utpprecoil 11679 ac5e9309d53956cfcc57ab1bff6a0fffb752121622755af4c80c14629cee5e72
senkoh_single_shot_shoulder.utpprecoil 11776 f43e9cc4100925998dbe19917635935897f26dfa90dc0260a2ec02e295b610d0
shingen_single_shot_ironsight.utpprecoil 11640 8ed6436b5faac31fa40e985f8a777eecaf1da5f3114b2db747dbefdc456c8fa3
shingen_single_shot_shoulder.utpprecoil 11705 9b1f9a196cc4d90023998f30238ca55dd85c9b6d626514d6f1f7a58cbcf93222
sidewinder_single_shot_ironsight.utpprecoil 11774 be465db04e4e893f1146fc56772fd6b5034a59d843a76037a393aae2206f69c8
sidewinder_single_shot_shoulder.utpprecoil 11731 2689b9a2162684b13305c8ab235115f1182978ad9757c72bfa24b5bcb0dea542
slaughtomatic_single_shot_ironsight.utpprecoil 11597 d9dccf9d9e23c270b3b91b47b139f13109289a017d26f290c0c166ea3bdb96d6
slaughtomatic_single_shot_shoulder.utpprecoil 11739 b71170cea2c7212e9ba5b30cdc60fdbe59723d57a0f57398b94709b86fee6559
ticon_single_shot_ironsight.utpprecoil 11770 99b0aa031a615a34a2bbd883ef919b951b9621798752f5e1a8b231879a427bb0
ticon_single_shot_shoulder.utpprecoil 11684 511e608c7f02d9d0bb4ae03c1b6f02dea5c4a6a2a3fd1ea9b35604af73da89d7
unity_single_shot_ironsight.utpprecoil 11580 62bc73a89656b80aa1b41e9473cf37801156d0d7b574a571ea72e704bcd9145c
unity_single_shot_shoulder.utpprecoil 11508 b380a599ce7dd44e35065a853f6f72472f90d873b28464bd602060b21b4ccf5d
warden_single_shot_ironsight.utpprecoil 11544 f87a614eae19dc94170447f7641c34c97a3b40702a747a5a3bc0b20acae24bfe
warden_single_shot_shoulder.utpprecoil 11720 da45229d80740f889c16c690556036fbc967533e95134bb4df51c3d37e6b3d91
yukimura_single_shot_ironsight.utpprecoil 11374 38b7a82567cc783805737870375a2ad96fa40adc18d9a7352e1f6bdc5572f088
yukimura_single_shot_shoulder.utpprecoil 11884 d671c1f86001820ab3a0f3ebd6999e6a950cb7703cee0fdc1a3ef6a4e29a9536
"""

# ---------------------------------------------------------------------------------------------- ship list: code
# (path in the stage, path under <code-tree>\plugin\UTPP, group)
CODE = [
    (P_REDS + n, "scripts/" + n, "core")
    for n in ["utpp.reds", "utpp_motion.reds", "utpp_authored_recoil.reds", "utpp_body.reds", "utpp_bodyturn.reds",
              "utpp_crosshair.reds", "utpp_reticle.reds", "utpp_itemplane.reds", "utpp_loothead.reds", "utpp_locomotion.reds",
              "utpp_weapon_restriction.reds", "utpp_vehicle_exit.reds", "utpp_vehicle_anim.reds",
              "utpp_gamelook_exit.reds", "utpp_modsettings.reds", "utpp_modsettings_bridge.reds", "utpp_shoulder_input.reds",
              "utpp_ragdoll.reds"]   # utpp.reds / utpp_bodyturn.reds call UTPPRgBusy / UTPPRgKnockdownFeed (2026-11-03)
] + [
    (P_CET + m + "/" + f, "cet/" + m + "/" + f, "core")
    for m, f in [("utpp_core", "init.lua"), ("utpp_camera", "init.lua"), ("utpp_camera", "camera_math.lua"),
                 ("utpp_body", "init.lua"), ("utpp_body", "body_parts.lua"), ("utpp_bodyturn", "init.lua"),
                 ("utpp_perspective", "init.lua"), ("utpp_motion", "init.lua"), ("utpp_vehiclecam", "init.lua"),
                 ("utpp_loothead", "init.lua")]
] + [
    (P_ARC + "utpp_camera.archive.xl", "archive/utpp_camera.archive.xl", "core"),
    (P_ARC + "utpp_vehicle_tpp.archive.xl", "archive/utpp_vehicle_tpp.archive.xl", "core"),
    (P_ARC + "utpp_foot_tpp.archive.xl", "archive/utpp_foot_tpp.archive.xl", "core"),
    (P_INPUT + "utpp_shoulder.xml", "input/utpp_shoulder.xml", "core"),   # shoulder-swap key (Input Loader)
]
CODE_NUE = [
    (P_REDS + "utpp_reload_gate.reds", "scripts/utpp_reload_gate.reds", "nue"),
    (P_CET + "utpp_reload_gate/init.lua", "cet/utpp_reload_gate/init.lua", "nue"),
    (P_CET + "utpp_reload_gate/policy.lua", "cet/utpp_reload_gate/policy.lua", "nue"),
    (P_ARC + "utpp_nue_reload.archive.xl", "archive/utpp_nue_reload.archive.xl", "nue"),
]
# Pinned expectations for code files (checked, FAIL if different). Re-pinned 2026-11-11 after the renames: the file differs from the
# tested game copy (083ab875...) only in one comment line (checked with a line diff of both files).
CODE_EXPECT = {
    P_ARC + "utpp_camera.archive.xl": "6ccb18fe2d0a610be4b1a653ca0e46c9ac78d071737fdca7438469a553b44100",
}

# ---------------------------------------------------------------------------------------------- day-of-build groups
# Scope rule: the package carries what passes the in-game test on the day of the build. Each group below is therefore pinned
# (the pins equal the tested game copy / stage) but is NOT decided silently: a final run needs an explicit decision for every
# group, `--day-of-build melee=ship,car=ship` (or =skip), otherwise it stops with exit 2 and prints the checklist. A dry run
# assumes "ship" for scanning, warns, and writes day-of-build-checklist.txt. Skipping a group leaves out ALL its files (the
# scripts of a group are self-contained: no other shipped script references them).
# group -> (title, what to check in game on the day of the build)
DAYOF_GROUPS = {
    "melee": ("Third-person melee: fists, knife, katana, one-handed blunt, baton, two-handed blunt",
              "fist combat in third person (idle, punches, movement, equip/unequip, guard) with the UTPP body; knife, katana, "
              "one-handed blunt, baton and two-handed blunt: combo, heavy, block (not knife), parried from left / right; "
              "Male V only in the single short pre-release test"),
    "car": ("Third-person car entry/exit",
            "entering and leaving cars as driver and passenger (door, approach, exit clips, camera transitions), "
            "sport / standard / utility body types; no .xl file belongs to this archive"),
}
# (group, path in the stage, size, sha256) - data copied from the game folder
DAYOF_DATA = [
    # combined fists pack (pwa/pma fists, StrongArms, default sets): Build\work\melee-fists-combined-20261109\runs\combined\packed,
    # stage melee-fists-combined-20261109; game copy of 2026-11-10 is identical (re-pinned 2026-11-10, was 827a8841... of npc1-fists)
    # Re-pinned 2026-11-14 (was d683748f..., 4530176 bytes): new final_attack_03 (+ _hit, _deflect) of both fists sets; the other fists
    # clips and the default / StrongArms sets are unchanged.
    # Re-pinned 2026-11-14 (was 844d00f6..., 4534272 bytes): knife, katana, one-handed blunt and baton sets for both bodies added; the
    # six fists / default / StrongArms sets are byte-identical to 844d00f6...
    # Re-pinned 2026-11-14 (was 770bb02a..., 22454272 bytes): knife, katana, one-handed blunt and baton final attacks start from the
    # attack before them; knife combo attacks hold their pose through the combo window; two-handed blunt sets for both bodies added.
    # The six fists / default / StrongArms sets are byte-identical to 770bb02a...
    ("melee", P_ARC + "utpp_melee.archive", 26632192, "b978453a91c3240c5b2e999f61ccee19710cfe439ffeff662dc25ebb6f7fbb4b"),
    # Melee full pose tables (hips, spine and legs of every packed melee clip, one file per body; generated from the clip inputs of
    # the archive above by tools\melee\melee_pose_table.py): fists, knife, katana, one-handed blunt, baton and two-handed blunt,
    # 368 clips each.
    ("melee", P_DATA + "utpp_melee_pose_female.bin", 2127612, "882ba64ae582f829b5a990a3b4989b922b9940d6a2c9f2af43caa24fb3f5227b"),
    ("melee", P_DATA + "utpp_melee_pose_male.bin", 2127612, "abb21addbf3d25f4186aa50eedd67f12e7f59a80175ac6768b06f9d44665aa2a"),
    # stage Build\stages\vehexit6-20261109 (game copy of 2026-11-09 15:30 is identical)
    ("car", P_ARC + "utpp_vehicle_car.archive", 7102464, "3843163c100787fc0669c6f5091e58597e83a312cc1f21da1542fc09771119d5"),
]
# (group, path in the stage, path under <code-tree>\plugin\UTPP, sha256 pin of the code tree file). Pins were the tested game copies of
# 2026-11-09 / 2026-11-10; re-pinned 2026-11-11 (each new pin checked against the tree file, old vs new compared line by line):
#   utpp_melee.reds           6336006092... -> 0be6a0b6...  comments only (the UTPP_STATES_TABLE lines are unchanged)
#   utpp_melee_tpp.archive.xl f871e23238... -> c30ed152...  FUNCTIONAL: the wrapper variable the six entries gate on is utpp_melee_tpp
#                                                           instead of utpp_foot_tpp (Animation pack: off with "Game Animations")
#   utpp_vehicle_approach.reds f3c9cab968... -> 70773283... comments only
#   utpp_vehicle_door.reds    f05fb69325... -> f4b0a821...  comments only
#   utpp_vehicle_approach_table.reds, utpp_vehicle_door_timing.reds: unchanged
# Re-pinned 2026-11-13 (redscript 1.0 compatibility, release 0.1.1; the tested game copies of 0.1.0 differ from these files):
#   utpp_melee.reds           0be6a0b6... -> b18f9b9f...  `new Vector4(` -> `Vector4(` (1 site) and `public static func` -> `public func`
#                                                           on 11 file-scope functions; redscript 0.5.32 compiles both to identical decompiled code;
#                                                           the UTPP_STATES_TABLE lines are unchanged
#   utpp_vehicle_approach.reds 70773283... -> 2dd3b901... `new Vector4(` -> `Vector4(` (1 site)
# Re-pinned 2026-11-14 (knife, katana, one-handed blunt and baton sets):
#   utpp_melee.reds           411d1dbe... -> 6c0dd8f3...  FUNCTIONAL: four covered names and state lines added; the strike-turn signal
#                                                           keys on the guard list (fists only, unchanged behaviour for fists)
#   utpp_melee_tpp.archive.xl c30ed152... -> 31a76cca...  FUNCTIONAL: eight entries added; the six existing entries are unchanged
# Re-pinned 2026-11-14 (melee full pose):
#   utpp_melee.reds           6c0dd8f3... -> 9018f613...  FUNCTIONAL: two native declarations and one call that sends the drawn
#                                                           weapon's anim variable with the state mask; the UTPP_STATES_TABLE lines
#                                                           are unchanged
# Re-pinned 2026-11-14 (two-handed blunt sets):
#   utpp_melee.reds           9018f613... -> e18d48f0...  FUNCTIONAL: one covered name and one state line added (w_two_hand_blunt)
#   utpp_melee_tpp.archive.xl 31a76cca... -> b3340155...  FUNCTIONAL: two entries added; the existing entries are unchanged
# Re-pinned 2026-11-14 (strong arms play the third-person fist clips):
#   utpp_melee.reds           e18d48f0... -> a1aec221...  FUNCTIONAL: the strong arms leave the excluded arm cyberware group and map
#                                                           to the fists clip variable (UTPPMeleeClipVariable: covered list, state
#                                                           mask, guard and full pose section of w_melee_fists); the
#                                                           UTPP_STATES_TABLE lines are unchanged
#   utpp_melee_tpp.archive.xl b3340155... -> 1e696565...  FUNCTIONAL: four entries added (fists and default sets of both bodies
#                                                           behind [w_strong_arms, utpp_melee_tpp]); the existing entries are
#                                                           unchanged
DAYOF_CODE = [
    ("melee", P_REDS + "utpp_melee.reds", "scripts/utpp_melee.reds",
     "a1aec221ff9a87c78a885bd8d9ac4efad58fdaae44f738c32e02e0fb965ca339"),
    ("melee", P_ARC + "utpp_melee_tpp.archive.xl", "archive/utpp_melee_tpp.archive.xl",
     "1e6965658dea413426f6b40b2ff8b2a515c6fbae192bb9ef15570d245f70a996"),
    ("car", P_REDS + "utpp_vehicle_approach.reds", "scripts/utpp_vehicle_approach.reds",
     "2dd3b9017b668236a21e6e7cb2b604fbf2a3f199a57f413856fecbae8f0f905e"),
    ("car", P_REDS + "utpp_vehicle_approach_table.reds", "scripts/utpp_vehicle_approach_table.reds",
     "09ccead1654ed2cad752bee455606970eec645ef372463f4e976e63f154c9f38"),
    ("car", P_REDS + "utpp_vehicle_door.reds", "scripts/utpp_vehicle_door.reds",
     "f4b0a82142d8275391db1efb9867d8cc9808942764b14a0606655a2c024f0db8"),
    ("car", P_REDS + "utpp_vehicle_door_timing.reds", "scripts/utpp_vehicle_door_timing.reds",
     "a3273dfc15c4b63528ae8b811b6bcb529f25376d658382cef3b4a2ec7681a5ca"),
]


# Parried graph (optional, belongs to the melee group): utpp_melee_parried_graph.archive = patched copy of the game's
# base\gameplay\anim_graphs\player_melee.animgraph (pinned to the 2.5 vanilla graph BFB4D765...; source
# Build\work\melee-parried-graph-20261110\run5, Source\tools\anim\graph-patch\README.txt). It is copied from the game folder like the other
# data files, but ONLY when the shipped melee pack needs it. Rule (same as Source\tools\install\stage_melee_npc.ps1): a line of the
# UTPP_STATES_TABLE of the shipped utpp_melee.reds with the Parried bit (32) whose weapon has per-strike parried clips
# (parried_<left|right>_<suffix>, not just the plain parried_left / parried_right) needs the graph.
# The release tool cannot read clip names from an archive, so the pinned melee archive SHA-256 maps to the parried clip kind of each
# weapon anim variable: "generic" (plain parried_left / parried_right only) or "per-strike". A Parried-bit line of a weapon that is not
# listed for the shipped archive is a PLAN ERROR (classify it here when the pack is pinned). Fists: generic only.
GRAPH_REL = P_ARC + "utpp_melee_parried_graph.archive"
GRAPH_SIZE = 53248
GRAPH_SHA = "d5786bb159cec4ad8a334b15b6762785a5b587e1df0664a79de91f0cfdc87449"
PARRIED_BIT = 1 << 5
PARRIED_CLIPS = {
    # melee archive SHA-256 -> {anim variable: "generic" | "per-strike"}
    "827a8841b8a5f9d7471d4c8ff0ecf6d0c28593456659c32569835e0fcc4b1f94": {"w_melee_fists": "generic"},  # earlier npc1-fists pack
    # combined fists pack, pinned in DAYOF_DATA (game copy of 2026-11-10 = Build\work\melee-fists-combined-20261109\runs\combined\packed; xl f871e232...):
    # its reports\build-*.json and coverage.json hold only the plain parried_left / parried_right (checked 2026-11-10)
    "d683748f5339f7b6955f7f79c9a39a5f1a7602016967881dc0df64d4258656cf": {"w_melee_fists": "generic"},
    # the same pack with the new final_attack_03 clips (parried clips unchanged: generic)
    "844d00f643fcb36da421cf0ee2e4d1747ddac3622fca0dd8ed13495d689e82fe": {"w_melee_fists": "generic"},
    # the same fists sets + knife, katana, one-handed blunt and baton (parried_<left|right>_<strike> per strike: the graph is needed)
    "770bb02a4c952ff55bbcee589d410c9e81f03ae550e115d9d472dcf373a75c6d": {
        "w_melee_fists": "generic", "w_melee_knife": "per-strike", "w_melee_katana": "per-strike",
        "w_melee_one_hand_blunt": "per-strike", "w_melee_baton": "per-strike"},
    # the same sets with rebuilt final attacks (per-strike parried clips kept) + two-handed blunt sets (parried_<left|right>_<strike>
    # for every strike type of the set)
    "b978453a91c3240c5b2e999f61ccee19710cfe439ffeff662dc25ebb6f7fbb4b": {
        "w_melee_fists": "generic", "w_melee_knife": "per-strike", "w_melee_katana": "per-strike",
        "w_melee_one_hand_blunt": "per-strike", "w_melee_baton": "per-strike", "w_two_hand_blunt": "per-strike"},
}
STATES_RX = re.compile(r'^if\s+Equals\(n,\s*n"([A-Za-z0-9_]+)"\)\s*\{\s*return\s+(\d+);\s*\}$')


def parse_states_table(text):
    """UTPP_STATES_TABLE of utpp_melee.reds -> ({anim variable: mask}, [problems]); every non-blank, non-comment line must parse."""
    m = re.search(r"(?s)// UTPP_STATES_TABLE begin(.*?)// UTPP_STATES_TABLE end", text)
    if not m:
        return {}, ["UTPP_STATES_TABLE markers not found in utpp_melee.reds"]
    states, problems = {}, []
    for raw in m.group(1).splitlines():
        line = raw.strip()
        if not line or line.startswith("//"):
            continue
        t = STATES_RX.match(line)
        if not t:
            problems.append("UTPP_STATES_TABLE line not understood: %s" % line)
        elif t.group(1) in states:
            problems.append("UTPP_STATES_TABLE lists %s twice" % t.group(1))
        else:
            states[t.group(1)] = int(t.group(2))
    return states, problems


def parried_graph_rule(reds_text, melee_archive_sha):
    """-> (graph needed, [problems]) for the shipped utpp_melee.reds and the shipped melee archive (its pinned SHA-256)."""
    states, problems = parse_states_table(reds_text)
    kinds = PARRIED_CLIPS.get(melee_archive_sha.lower(), {})
    needed = False
    for var, mask in sorted(states.items()):
        if not mask & PARRIED_BIT:
            continue
        kind = kinds.get(var)
        if kind == "per-strike":
            needed = True
        elif kind != "generic":
            problems.append("%s has the Parried bit (32) but the parried clip kind of that weapon in the melee archive %s... is not "
                            "classified in PARRIED_CLIPS (generic / per-strike): cannot tell whether the parried graph is needed"
                            % (var, melee_archive_sha[:8]))
    return needed, problems


def parse_dayof(spec):
    """'melee=ship,car=skip' -> ({group: 'ship'|'skip'}, error text or None)."""
    out = {}
    for part in [p.strip() for p in (spec or "").split(",") if p.strip()]:
        g, _, v = part.partition("=")
        if g not in DAYOF_GROUPS or v not in ("ship", "skip"):
            return {}, "bad --day-of-build item %r (use <group>=ship|skip, groups: %s)" % (part, ", ".join(DAYOF_GROUPS))
        if g in out:
            return {}, "--day-of-build lists %r twice" % g
        out[g] = v
    return out, None


def dayof_checklist(decisions):
    lines = ["Day-of-build checklist (ship what passes the in-game test on the build day)"]
    for g, (title, what) in DAYOF_GROUPS.items():
        files = [p for gg, p, _, _ in DAYOF_DATA if gg == g] + [p for gg, p, _, _ in DAYOF_CODE if gg == g]
        lines.append("  [%s] %s - decision: %s" % (g, title, decisions.get(g, "UNDECIDED")))
        lines.append("      test: " + what)
        for p in files:
            lines.append("      file: " + p.replace("/", "\\"))
    for rel, _ in DATA_PENDING:
        lines.append("  [pending] %s stays pending (DATA_PENDING): pin it in DATA after the in-game check" % rel.replace("/", "\\"))
    lines.append("  final run: --day-of-build %s" % ",".join("%s=ship|skip" % g for g in DAYOF_GROUPS))
    return "\n".join(lines)


def dayof_shipped(args):
    return {g for g in DAYOF_GROUPS if args.dayof.get(g, "ship") == "ship"}


DLL_REL = P_DLL + "UTPP.dll"
# Pose owner switch (optional, --early-pose): the plugin reads this file once at load. Present = the pose is written in the early
# mapping stage (first line UTPP_EARLY_POSE_HELPERS, then one 16-digit hex bone name hash per line); absent (the default package) = the
# pose is written in the FK stage. A file whose list does not parse still selects the early owner and then writes no pose at all,
# so the packer checks the content exactly as the plugin parses it (check_early_pose_bytes).
EARLY_POSE_REL = P_DLL + "early-pose.txt"
EARLY_POSE_HEADER = "UTPP_EARLY_POSE_HELPERS"   # a packed file carries this header (the plugin also reads the earlier one)
EARLY_POSE_MAX_HASHES = 1024   # the plugin's rig bone limit
EARLY_POSE_README = ("\nPose write stage: red4ext\\plugins\\UTPP\\early-pose.txt selects the earlier of the two pose write stages of the\n"
                     "plugin. Delete it with the game closed to use the later stage; the change applies at the next game start.")
README_REL = P_DLL + "README.txt"
NOTICES_REL = P_DLL + "THIRD_PARTY_NOTICES.txt"
LICENSE_REL = P_DLL + "LICENSE.txt"   # copy of <code tree>\LICENSE

# ---------------------------------------------------------------------------------------------- generated texts
README_TXT = """\
Ultimate Third Person Project (UTPP) {version} - by {author}

Requires Cyberpunk 2077 2.50 exactly (exe 3.0.88.46235). On any other game version the native plugin disables itself.
Requires RED4ext, redscript, Cyber Engine Tweaks, Codeware, ArchiveXL and Input Loader (Input Loader provides the
shoulder-switch key). Tested with RED4ext 1.31.1, redscript 0.5.32, Cyber Engine Tweaks 1.38.1, Codeware 1.21.4,
ArchiveXL 1.28.8, Input Loader 0.2.6 and Mod Settings 0.2.30 (versions installed in the test game, 2.50).
Older versions will not work with game version 2.5.
Use redscript 0.5.32. The scripts of UTPP compile with redscript 1.0.0 since version 0.1.1, but the frameworks UTPP
needs were not ready for redscript 1.0.0 when this version was built (2026-11-13), so it is not supported yet.
Mod Settings is recommended (the Ultimate Third Person page of the game's Mod Settings screen); without it
Native Settings UI 1.96 provides the in-game settings tab. The Microsoft Visual C++ 2015-2022 x64 runtime is required.
Requires the Phantom Liberty expansion (the weapon magazine fix archive uses its weapon resources). Tested with the
Steam version. TweakXL is not needed.

Install: extract into the Cyberpunk 2077 folder (the one with bin, archive, r6 and red4ext), or install with Vortex.
Settings: Mod Settings > Ultimate Third Person (or the UTPP tab of Native Settings UI when Mod Settings is not
installed), or the CET console: GetMod('utpp_core').Set('key', value). Values set in either place are kept in sync.
Logs: red4ext\\logs\\utpp-*.log. The mod always writes a small basic log there; the detailed "Diagnostic logs" are off
by default (see "How to report a problem" below).
Shoulder switch: one key that the game itself does not use - L on the keyboard, right stick click held for 0.5 s on a
controller; rebind it in Mod Settings. It is provided through Input Loader (r6\\input\\utpp_shoulder.xml), so Input Loader
is required for this key; everything else works without it.
Emergency switches: an empty file next to UTPP.dll named utpp_locomotion.disable, utpp_lootrange.disable,
utpp_headguard.disable or utpp_interactrange.disable switches the matching native feature off (the last one stops
the third-person interaction range / interaction descriptor changes).{early_pose_line}

Animation pack: Mod Settings > Ultimate Third Person > General > Animation pack. "Ultimate Third Person Project" (default) = the
animations of this mod. "Game Animations" = the animations of the game itself on the third-person body: the camera, the full body,
the head, the crosshair and the shoulder key stay; the mod's movement, weapon poses, recoil, hand grip, shoulder and lean corrections,
traversal, the authored reload, the melee clips, the vehicle entry and exit clips and Shooter mode are off, and the options that depend
on them have no effect. V turns with the camera, as in the game. Arm and shoulder deformations of the game's first-person
animations can come back with this pack. The change works at once while V stands idle; melee, vehicle and reload animations change at
the next weapon draw, vehicle entry or reload, and a change waits while V reloads, aims, fights, climbs, drives or is in a scene.
Your other settings are kept. The archives of the mod stay installed and replace some game resources also with this pack; their UTPP
entries are inactive then.

How to report a problem: (1) Mod Settings > Ultimate Third Person > General > Diagnostic logs > "On for 5 minutes"; (2) repeat the
problem in the game: the logs run for 5 minutes of gameplay (not in menus or on pause), then switch themselves off, or set the list to
"Off" earlier; (3) open red4ext\\logs: the file utpp-report-(date).txt lists the files to send; zip them and send them to the author.
The logs contain no personal data; file names in them are relative to the game folder. "On until switched off" keeps the logs on
also after a game restart: only for long test sessions.

{melee_line}
Recoil: UTPP recoil profiles ship for the weapon families in data\\recoil; shotguns, sniper and precision rifles
and heavy machine guns have no UTPP recoil profile in this version.

Optional authored reload (female V with the Nue pistol only): archive\\pc\\mod\\utpp_nue_reload.archive (+ .archive.xl),
r6\\scripts\\UTPP\\utpp_reload_gate.reds and the CET mod utpp_reload_gate. The archive replaces the base-game animation graphs
player_ranged.animgraph and weapon.animgraph, so it conflicts with other mods that replace them. To go back to the game's
own reload, delete all of these files/folders together. Or choose the Game Animations pack, which also uses the game's own reload.

Animation data. The motion-matching databases, overlays, traversal and lean tables shipped with UTPP are original work. They are based on licensed GASPALS 1.2 content by Polygon Hive, then heavily modified, extended, tuned and blended with the game's own animations for Cyberpunk 2077. The GASPALS content was used under a purchased license.
Some animation, recoil and skin helper data is derived from Cyberpunk 2077 content (CD PROJEKT RED) and only works with a copy of the game.
Skin helper data: red4ext\\plugins\\UTPP\\skin_helpers holds the data with which the plugin recomputes the muscle and shoulder helper bones
of the player body (one set per body, produced by the mod's tools from the game's own deformation data). The plugin uses a set only when it
is exactly the one built into UTPP.dll. mode.txt in that folder contains APPLY; write OFF into it to switch the recomputation off.
Licence: see LICENSE.txt. Third-party code and notices: see THIRD_PARTY_NOTICES.txt.
Not affiliated with or endorsed by CD PROJEKT RED, Epic Games, Polygon Hive{nvidia_unaffiliated}.
"""

NOTICES_HEAD = """\
UTPP {version} - third-party notices

Ultimate Third Person Project is (c) {author}. All rights reserved except where a third-party licence below says otherwise.

== Animation data (not open source) ==
Animation data. The motion-matching databases, overlays, traversal and lean tables shipped with UTPP are original work. They are based on licensed GASPALS 1.2 content by Polygon Hive, then heavily modified, extended, tuned and blended with the game's own animations for Cyberpunk 2077. The GASPALS content was used under a purchased license.
{nvidia}
== Content derived from Cyberpunk 2077 ==
Weapon recoil profiles, skin helper data (the muscle and shoulder helper bone data of the player body), and animation clips,
animation graphs and resource patches built from or for the animations and
resources of Cyberpunk 2077 and its Phantom Liberty expansion (c) CD PROJEKT RED, are derived from content of the game.
They do not contain the game, work only together with a legitimate copy of it, and are not endorsed by CD PROJEKT RED.{graph}

== Open-source code used in UTPP.dll ==
The three libraries below are licensed under the MIT licence. Each full licence text follows unchanged.

"""
NOTICE_ITEMS = [
    ("RED4ext.SDK", "Copyright (c) 2020 - present Octavian Dima",
     "Used for the plugin API headers (headers only).", TOOLS_REPOS / "RED4ext.SDK" / "LICENSE.md"),
    ("cyberpunk-vr-port", "Copyright (c) 2026 dariulone",
     "Logic and memory offsets for arming the player animation were adapted from this project.",
     TOOLS_REPOS / "cyberpunk-vr-port" / "LICENSE"),
    ("Motion-Matching", "Copyright (c) 2021 Daniel Holden",
     "The inertialization and motion matching algorithms were re-implemented from this project.",
     TOOLS_REPOS / "Motion-Matching" / "LICENSE"),
]

# ---------------------------------------------------------------------------------------------- local rules
# Machine-local rules of the packer live in the untracked file scan_needles.local.json next to this script: extra word lists of
# the package scan ("needles", "text_rules", "visible_text_fail", "visible_text_warn") and exact text replacements for the header
# fields of chooser files ("chooser_replacements": [[old, new], ...], "chooser_prior_licence"). Without the file only the
# generic checks run and chooser files must already carry the neutral header fields.
LOCAL_RULES_FILE = Path(__file__).with_name("scan_needles.local.json")


def load_local_rules(path=LOCAL_RULES_FILE):
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


LOCAL_RULES = load_local_rules()

# ---------------------------------------------------------------------------------------------- C10 transforms
LICENCE_NEUTRAL = "Licensed animation data"
CHOOSER_REPL = [(old, new) for old, new in LOCAL_RULES.get("chooser_replacements", [])]
CHOOSER_PRIOR = LOCAL_RULES.get("chooser_prior_licence", "")
STYLE_COMMENT = ["# UTPP style weights.", "# %s." % LICENCE_NEUTRAL]


def xform_chooser(data, rel=""):
    # Chooser.hpp:701-706 (+ LoadSlide/mm_rules/blend_profile) read only format, version, inputs, tables, slide,
    # mm_rules and blend_profile; licence / generator / source.* are never read.
    # A header line that is already neutral (new text exactly once, old text absent) is accepted as is; anything else fails.
    text = data.decode("utf-8")
    if CHOOSER_PRIOR and text.count(CHOOSER_PRIOR) == 1:
        text = text.replace(CHOOSER_PRIOR, '"licence": "%s",' % LICENCE_NEUTRAL)
    for old, new in CHOOSER_REPL:
        if text.count(old) == 0 and text.count(new) == 1:
            continue
        if text.count(old) != 1:
            raise ValueError("chooser: expected exactly one occurrence of %r" % old[:60])
        text = text.replace(old, new)
    # "database_file" is a label nobody reads; it carried the work name of the bake (naming rule: no round numbers) -> the shipped
    # database of this chooser (<name>.chooser.json -> <name>.utppmm)
    base = os.path.basename(rel.replace("\\", "/"))
    if base.endswith(".chooser.json"):
        text, n = re.subn(r'"database_file": "[^"]*"', '"database_file": "%s.utppmm"' % base[:-len(".chooser.json")], text)
        if n != 1:
            raise ValueError("chooser: expected exactly one database_file, found %d" % n)
    return text.encode("utf-8")


def xform_style(data):
    # Overlay.hpp:454 skips every line starting with '#'; only the leading comment block is replaced.
    text = data.decode("utf-8")
    eol = "\r\n" if "\r\n" in text else "\n"
    lines = text.split(eol)
    if not lines or lines[0] != "utpp-style-weights 1":
        raise ValueError("style weights: unexpected header %r" % (lines[:1],))
    i = 1
    while i < len(lines) and lines[i].startswith("#"):
        i += 1
    if i == 1 or i >= len(lines) or lines[i] != "[weights]":
        raise ValueError("style weights: comment block not followed by [weights]")
    out = [lines[0]] + STYLE_COMMENT + lines[i:]
    old_data = [l for l in lines if not l.startswith("#")]
    new_data = [l for l in out if not l.startswith("#")]
    if old_data != new_data:
        raise ValueError("style weights: data lines changed")
    return eol.join(out).encode("utf-8")


# ---------------------------------------------------------------------------------------------- helpers
def sha256_of(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while True:
            b = f.read(8 << 20)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


def sha256_bytes(b):
    return hashlib.sha256(b).hexdigest()


def native(rel):
    return rel.replace("/", os.sep)


def run_git(tree, *args):
    try:
        r = subprocess.run(["git", "-C", str(tree)] + list(args), capture_output=True, text=True, timeout=60)
        return r.returncode, r.stdout.strip()
    except Exception as e:  # git missing
        return 1, str(e)


def fmt_mib(n):
    return "%.1f MiB" % (n / 1048576.0)


# ---------------------------------------------------------------------------------------------- finding data sources by content
# Folders searched (besides the same package path in the game folder) for a file with the pinned size and SHA-256: these game-relative
# folders (the game may still hold an older layout of the same files under other names) and the project's Data folder.
SEARCH_SUBDIRS = [P_DLL, P_ARC]
_HASH_CACHE = {}
_INDEX_CACHE = {}


def cached_sha(path):
    st = Path(path).stat()
    key = (str(path), st.st_size, st.st_mtime_ns)
    if key not in _HASH_CACHE:
        _HASH_CACHE[key] = sha256_of(path)
    return _HASH_CACHE[key]


def _size_index(roots):
    key = tuple(str(r) for r in roots)
    if key not in _INDEX_CACHE:
        idx = {}
        for r in roots:
            if not Path(r).is_dir():
                continue
            for dirpath, _, names in os.walk(r):
                for n in names:
                    p = Path(dirpath) / n
                    try:
                        idx.setdefault(p.stat().st_size, []).append(p)
                    except OSError:
                        pass
        _INDEX_CACHE[key] = idx
    return _INDEX_CACHE[key]


def find_source(game, rel, size, sha, extra_roots=()):
    """A file whose size and SHA-256 equal the pin: any file of that size under the data roots (the project's Data folder) first, then
    the package path in the game folder, then the game's search folders. None when no file has the pinned content (a file with
    another hash is never used)."""
    game = Path(game)

    def match(paths):
        for p in paths:
            try:
                if cached_sha(p) == sha:
                    return p
            except OSError:  # vanished after the index was built
                continue
        return None

    found = match(_size_index([Path(r) for r in extra_roots]).get(size, []))  # the project's Data folder first
    if found:
        return found
    direct = game / native(rel)
    if direct.is_file() and direct.stat().st_size == size and cached_sha(direct) == sha:
        return direct
    return match(_size_index([game / native(s) for s in SEARCH_SUBDIRS]).get(size, []))


class Entry:
    def __init__(self, rel, kind, group="core", src=None, expect_sha=None, expect_size=None, note=""):
        self.rel = rel
        self.kind = kind  # data | code | dll | xform | gen | skin (bundle file, hash checked against the DLL's approved table)
        self.group = group
        self.src = src
        self.expect_sha = expect_sha
        self.expect_size = expect_size
        self.note = note
        self.data = None  # bytes for code/xform/gen/dll when small enough
        self.sha = None
        self.size = None
        self.src_sha = None
        self.hits = []  # binary needle hits

    @property
    def ext(self):
        return os.path.splitext(self.rel)[1].lower()


# ---------------------------------------------------------------------------------------------- plan
def build_plan(args, errors):
    game = Path(args.game)
    tree = Path(args.code_tree) / "plugin" / "UTPP"
    entries = []

    soft = args.dry_run
    args.soft_missing = []  # dry run: game files that are not there yet (not installed) become FAIL findings, not plan errors

    def need(p, what):
        if not p.is_file():
            if soft and what.startswith("game"):
                args.soft_missing.append((what, p))
            else:
                errors.append("missing %s: %s%s" % (what, p, " (no file with the pinned size and SHA-256 under the game folder "
                                                    "or the data roots)" if what.startswith("game") else ""))
            return False
        return True

    extra_roots = [Path(r) for r in getattr(args, "data_roots", None) or [ROOT / "Data"]]

    def src_for(rel, size, sha):
        """The file with the pinned content (find_source); when there is none, the package path in the game folder (reported missing)."""
        return find_source(game, rel, size, sha, extra_roots) or (game / native(rel))

    dll = Path(args.dll)
    need(dll, "--dll")
    entries.append(Entry(DLL_REL, "dll", src=dll, note="RED4ext plugin (built from --code-tree commit)"))
    data = list(DATA) + (DATA_NUE if args.nue else [])
    for rel, size, sha in data:
        p = src_for(rel, size, sha)
        if not need(p, "game data file") and soft:
            continue
        entries.append(Entry(rel, "data", src=p, expect_sha=sha, expect_size=size, note="read only copy, found by SHA-256"))
    for line in RECOIL.strip().splitlines():
        name, size, sha = line.split()
        p = src_for(P_REC + name, int(size), sha)
        if not need(p, "game recoil file") and soft:
            continue
        entries.append(Entry(P_REC + name, "data", src=p, expect_sha=sha, expect_size=int(size),
                             note="read only copy, found by SHA-256"))
    for rel, size, sha in TRANSFORMS:
        p = src_for(rel, size, sha)
        if not need(p, "game data file (C10 transform)") and soft:
            continue
        entries.append(Entry(rel, "xform", src=p, expect_sha=sha, expect_size=size,
                             note="copy with neutral licence text (C10), found by SHA-256"))
    for rel, sub, grp in CODE + (CODE_NUE if args.nue else []):
        p = tree / native(sub)
        need(p, "code file")
        entries.append(Entry(rel, "code", group=grp, src=p, expect_sha=CODE_EXPECT.get(rel),
                             note="code tree %s" % sub))
    shipped = dayof_shipped(args)
    for grp, rel, size, sha in DAYOF_DATA:
        if grp not in shipped:
            continue
        p = src_for(rel, size, sha)
        if not need(p, "game data file") and soft:
            continue
        entries.append(Entry(rel, "data", group=grp, src=p, expect_sha=sha, expect_size=size,
                             note="read only copy, found by SHA-256, day-of-build group %s" % grp))
    for grp, rel, sub, sha in DAYOF_CODE:
        if grp not in shipped:
            continue
        p = tree / native(sub)
        need(p, "code file")
        entries.append(Entry(rel, "code", group=grp, src=p, expect_sha=sha,
                             note="code tree %s, day-of-build group %s" % (sub, grp)))
    if "melee" in shipped:  # optional pinned file: the parried graph, only when the shipped melee pack needs it
        reds = tree / native("scripts/utpp_melee.reds")
        if reds.is_file():
            melee_sha = next(s for g, r, _, s in DAYOF_DATA if r == P_ARC + "utpp_melee.archive")
            needed, problems = parried_graph_rule(reds.read_text(encoding="utf-8", errors="replace"), melee_sha)
            errors.extend("parried graph rule: " + x for x in problems)
            if needed:
                p = src_for(GRAPH_REL, GRAPH_SIZE, GRAPH_SHA)
                if need(p, "game data file (parried graph)") or not soft:
                    entries.append(Entry(GRAPH_REL, "data", group="melee", src=p, expect_sha=GRAPH_SHA, expect_size=GRAPH_SIZE,
                                         note="game folder (read only), modified copy of the game's player_melee.animgraph "
                                              "(needed by the shipped melee pack)"))
    skin_root = Path(getattr(args, "skin_data_root", None) or ROOT / "Build" / "test-data")
    for body in SKIN_BODIES:
        for name in SKIN_BUNDLE_FILES:
            p = skin_root / SKIN_SOURCE_DIR[body] / name
            need(p, "skin helper data file")
            entries.append(Entry(P_SKIN + body + "/" + name, "skin", src=p,
                                 note="copy of %s\\%s\\%s (approved hash = BodyHelperAdmission.hpp)" % (skin_root.name, SKIN_SOURCE_DIR[body], name)))
    mode = Entry(SKIN_MODE_REL, "gen", note="generated: APPLY + LF")
    mode.data = SKIN_MODE_BYTES
    entries.append(mode)
    early_src = getattr(args, "early_pose", None)
    if early_src:  # explicit opt-in only (main checks that the pin is given); default packages never contain the switch file
        p = Path(early_src)
        if need(p, "early pose file (--early-pose)"):
            entries.append(Entry(EARLY_POSE_REL, "data", src=p, expect_sha=(getattr(args, "early_pose_sha", None) or "").lower(),
                                 expect_size=p.stat().st_size, note="pose owner switch, copy of --early-pose, pinned by --early-pose-sha"))
    entries.append(Entry(README_REL, "gen", note="generated"))
    lic =Path(args.code_tree) / "LICENSE"
    need(lic, "licence file")
    entries.append(Entry(LICENSE_REL, "code", src=lic, note="code tree LICENSE"))
    entries.append(Entry(NOTICES_REL, "gen", note="generated (MIT texts from Tools\\repos)"))
    return entries


def nvidia_tools():
    """[(tool, licence)] of the NVIDIA generators whose clips are in the package (flags NOTICE_*)."""
    t = []
    if NOTICE_MOTIONBRICKS:
        t.append(("NVIDIA MotionBricks", "NVIDIA Open Model Agreement"))
    if NOTICE_KIMODO:
        t.append(("NVIDIA Kimodo", "NVIDIA Open Model License"))
    return t


def nvidia_notice():
    tools = nvidia_tools()
    if not tools:
        return ""
    names = " and ".join(n for n, _ in tools)
    lic = " and ".join("%s (%s)" % (l, n) if len(tools) > 1 else l for n, l in tools)
    return ("\n== NVIDIA ==\nSome transition, vehicle-exit and melee animation clips were generated locally with %s (models licensed by "
            "NVIDIA Corporation under the %s). No NVIDIA model is included. UTPP is not affiliated with or endorsed by "
            "NVIDIA.\n" % (names, lic))


GRAPH_NOTICE = ("\nutpp_melee_parried_graph.archive is a modified copy of the game's own base\\gameplay\\anim_graphs\\player_melee.animgraph\n"
                "(c) CD PROJEKT RED, patched to choose the parried clip per attack. It contains no third-party content and works\n"
                "only together with a legitimate copy of the game.")


def gen_notices(errors, graph=False):
    parts = [NOTICES_HEAD.format(version=VERSION, author=AUTHOR, nvidia=nvidia_notice(),
                                 graph=GRAPH_NOTICE if graph else "").replace("\n", "\r\n")]
    for name, copyright_line, usage, lic in NOTICE_ITEMS:
        if not lic.is_file():
            errors.append("missing licence text: %s" % lic)
            continue
        parts.append(("-" * 78 + "\r\n%s - %s\r\n%s\r\n" + "-" * 78 + "\r\n\r\n") % (name, copyright_line, usage))
        body = lic.read_bytes().decode("utf-8")
        if "\r\n" not in body:
            body = body.replace("\n", "\r\n")
        parts.append(body.rstrip("\r\n") + "\r\n\r\n")
    return "".join(parts).encode("utf-8")


MELEE_LINE = {
    True: "Melee in third person: fists, knives, katanas, one-handed blunt weapons, batons and two-handed blunt weapons (built\n"
          "from the game's own NPC combat animations); other melee weapons keep the game's animations.",
    False: "Melee in third person: not part of this version; melee keeps the game's animations.",
}


GRAPH_README = ("\nMelee parrying: archive\\pc\\mod\\utpp_melee_parried_graph.archive replaces the base-game animation graph\n"
                "player_melee.animgraph (it picks the parried clip per attack), so it conflicts with other mods that replace that\n"
                "graph. To go back to the game's own parry selection, delete this file.")


def gen_readme(melee=True, graph=False, early_pose=False):
    nv = " or NVIDIA" if nvidia_tools() else ""
    melee_line = MELEE_LINE[bool(melee)] + (GRAPH_README if (melee and graph) else "")
    return README_TXT.format(version=VERSION, author=AUTHOR, nvidia_unaffiliated=nv, melee_line=melee_line,
                             early_pose_line=EARLY_POSE_README if early_pose else "").replace("\n", "\r\n").encode("utf-8")


# ---------------------------------------------------------------------------------------------- scanning
SCAN_NEEDLES_FILE = Path(__file__).with_name("scan_needles.local.json")
AUTHOR_BYTES = AUTHOR.split()[-1].lower().encode()   # the author's surname: allowed in the licence texts and the DLL author field only


def load_private_scan_rules(path=SCAN_NEEDLES_FILE):
    """Words that must not reach a package but are not listed in the public sources (personal data, tool names, internal document
    names) come from the untracked scan_needles.local.json: {"needles": {category: [word, ...]}, "text_rules": [{"check": category,
    "regex": pattern, "ignore_case": true}, ...]}. Without the file only the generic checks run.
    Returns ({category: [lower-case bytes]}, [(category, compiled regex, exempt files)])."""
    try:
        with open(path, encoding="utf-8") as f:
            d = json.load(f)
    except (OSError, ValueError):
        return {}, []
    needles = {cat: [w.lower().encode() for w in words] for cat, words in d.get("needles", {}).items()}
    rules = [(r["check"], re.compile(r["regex"], re.I if r.get("ignore_case", True) else 0),
              {README_REL, NOTICES_REL, LICENSE_REL} if r["check"] == "personal" else set()) for r in d.get("text_rules", [])]
    return needles, rules


PRIVATE_NEEDLES, PRIVATE_TEXT_RULES = load_private_scan_rules()
BIN_NEEDLES = {  # lower-case; name -> needle
    "personal": PRIVATE_NEEDLES.get("personal", []) + [AUTHOR_BYTES],
    "tool-names": PRIVATE_NEEDLES.get("tool-names", []),
    "document-name": PRIVATE_NEEDLES.get("document-name", []),
    "dev-path": [b"ultimate third person project\\", b"ultimate third person project/", b"steam library", b"appdata",
                 b"\\users\\"],
    "licence-leftover": PRIVATE_NEEDLES.get("licence-leftover", []),
    "legacy-name": [b"gasp", b"m360", b"fortnite", OLD_SKIN_FOLDER.encode()],  # (OLD_SKIN_FOLDER also catches a DLL built before the rename) # names retired by the 2026-11-11 renames (licence prose lives in README / NOTICES only)
}
EXTRACT_WARN = [b"extracted/", b"extracted\\", b"research\\"]


def pass_file(path, scan_bin, utf16=False, allow=()):
    """One read: sha256 + (optional) case-insensitive needle search. Returns (sha, size, hits)."""
    needles = []
    if scan_bin:
        for cat, lst in BIN_NEEDLES.items():
            for n in lst:
                if n.decode() in allow:
                    continue
                needles.append((cat, n))
                if utf16:
                    needles.append((cat, n.decode().encode("utf-16le")))
        for n in EXTRACT_WARN:
            needles.append(("extract-ref", n))
    maxlen = max([len(n) for _, n in needles] or [1])
    h = hashlib.sha256()
    hits = []
    seen = {}
    size = 0
    with open(path, "rb") as f:
        tail = b""
        base = 0
        while True:
            chunk = f.read(16 << 20)
            if not chunk:
                break
            h.update(chunk)
            size += len(chunk)
            if needles:
                buf = (tail + chunk).lower()
                off0 = base - len(tail)
                for cat, n in needles:
                    start = 0
                    while True:
                        i = buf.find(n, start)
                        if i < 0:
                            break
                        pos = off0 + i
                        key = (cat, n)
                        # a hit inside the carried tail was already counted in the previous chunk
                        if i + len(n) > len(tail):
                            seen[key] = seen.get(key, 0) + 1
                            if seen[key] <= 3:
                                ctx = re.sub(rb"[^\x20-\x7e]", b".", buf[max(0, i - 30):i + len(n) + 50]).decode()
                                hits.append((cat, n.decode("latin1"), pos, ctx))
                        start = i + 1
                tail = chunk[-(maxlen - 1):] if maxlen > 1 else b""
            base += len(chunk)
    return h.hexdigest(), size, hits


class Findings:
    def __init__(self):
        self.items = []

    def add(self, level, check, rel, detail, line=None):
        self.items.append({"level": level, "check": check, "file": rel, "line": line, "detail": detail[:260]})

    def count(self, level):
        return sum(1 for i in self.items if i["level"] == level)


TEXT_EXT = {".lua", ".reds", ".xl", ".txt", ".json", ".md", ".cfg", ".ini", ".yaml", ".yml", ".xml"}
RX = lambda p, f=re.I: re.compile(p, f)
TEXT_FAIL = [
    ("personal", RX(r"\b%s\b" % re.escape(AUTHOR_BYTES.decode())), {README_REL, NOTICES_REL, LICENSE_REL}),
] + PRIVATE_TEXT_RULES + [
    # a backslash before a quote is a string escape (the mod's display name inside a redscript string), not a path separator
    ("dev-path", RX(r"ultimate third person project(?:/|\\(?!\"))|steam library|appdata|\\users\\"), set()),
    # retired names; the licence texts (README, THIRD_PARTY_NOTICES) name the GASPALS source by law and are exempt
    ("legacy-name", RX(r"gasp|m360|fortnite"), {README_REL, NOTICES_REL, LICENSE_REL}),
    # the test-install folder of the skin helper data (now skin_helpers): no exemption, comments included
    ("legacy-name", RX(re.escape(OLD_SKIN_FOLDER)), set()),
]
# Version / round suffixes in names (utpp_v2, base_r13c, ...), checked in code and menu text only (not in .json tables, which hold game
# and source clip names such as ..._F_V2_Lfoot). One to three digits: the game's own resource names end in four-digit numbers
# (..._tattoo_casius_r3758).
LEGACY_VERSION_RX = RX(r"[A-Za-z0-9]_v[0-9]+(?![A-Za-z0-9])|[A-Za-z0-9]_r[0-9]{1,3}[a-z]?(?![A-Za-z0-9])")
LEGACY_VERSION_EXEMPT = {README_REL, NOTICES_REL, LICENSE_REL}
PROBE_RX = RX(r"(?:deform|mount|reload|helper)_probe|utpp_probe|utpp_testcmd|local_test_profile|reload_phase"
              r"|utpptesthost|utpptestbackgroundinput")
GATE_RX = RX(r"reload_gate")
TEXT_WARN = [
    ("research-ref", RX(r"Research[\\/]|Backups[\\/]|Tools[\\/]repos|Archives[\\/]|handoff")),
    ("abs-path", RX(r"\b[A-Za-z]:[\\/]")),
    ("todo", RX(r"\bTODO\b|FIXME|\bHACK\b|\bXXX\b")),
    ("placeholder", RX(r"placeholder")),
]
COMMENT_RX = re.compile(r"(--|//|^\s*#)")


def lua_visible_strings(src):
    """(line, field, text) for label/desc/choices/section-label strings of Register({...}) tables at depth 1."""
    out = []

    def str_at(i):  # -> (text, end) of a Lua string literal starting at i, or None
        q = src[i]
        if q not in "\"'":
            return None
        j = i + 1
        buf = []
        while j < len(src) and src[j] != q:
            if src[j] == "\\" and j + 1 < len(src):
                buf.append({"n": "\n", "t": "\t"}.get(src[j + 1], src[j + 1]))
                j += 2
                continue
            buf.append(src[j])
            j += 1
        return "".join(buf), j + 1

    def chain(i):  # "a" .. "b" ..
        text = ""
        while True:
            while i < len(src) and src[i] in " \t\r\n":
                i += 1
            r = str_at(i) if i < len(src) else None
            if r is None:
                return text, i
            text += r[0]
            i = r[1]
            m = re.compile(r"\s*\.\.").match(src, i)
            if not m:
                return text, i
            i = m.end()

    def line_of(i):
        return src.count("\n", 0, i) + 1

    for m in re.finditer(r"\bRegister\(\s*\{", src):
        i = m.end() - 1
        depth = 0
        j = i
        while j < len(src):
            c = src[j]
            if c in "\"'":
                r = str_at(j)
                j = r[1]
                continue
            if c == "-" and src.startswith("--", j):
                nl = src.find("\n", j)
                j = len(src) if nl < 0 else nl
                continue
            if c == "{":
                depth += 1
            elif c == "}":
                depth -= 1
                if depth == 0:
                    break
            elif depth == 1:
                fm = re.compile(r"(label|desc|description|tooltip|choices)\s*=\s*").match(src, j)
                if fm and (j == 0 or not (src[j - 1].isalnum() or src[j - 1] == "_")):
                    k = fm.end()
                    if fm.group(1) == "choices":
                        d = 0
                        e = k
                        while e < len(src):
                            if src[e] in "\"'":
                                s2 = str_at(e)
                                out.append((line_of(e), "choices", s2[0]))
                                e = s2[1]
                                continue
                            if src[e] == "{":
                                d += 1
                            elif src[e] == "}":
                                d -= 1
                                if d == 0:
                                    break
                            e += 1
                        j = e
                    else:
                        txt, e = chain(k)
                        if txt:
                            out.append((line_of(j), fm.group(1), txt))
                        j = e
                        continue
            j += 1
    for m in re.finditer(r"RegisterSection\(\s*[\"'][^\"']*[\"']\s*,\s*", src):
        txt, _ = chain(m.end())
        if txt:
            out.append((line_of(m.start()), "section", txt))
    return out


DEFAULT_OFF = ["mm.feedDump", "mm.trace", "mm.debug", "camera.trace"]


def check_defaults(rel_to_text, F):
    """Diagnostics must default to off; the Shooter-mode label must be the neutral one."""
    cet = P_CET
    cands = {
        "mm.feedDump": cet + "utpp_core/init.lua", "mm.trace": cet + "utpp_core/init.lua",
        "mm.debug": cet + "utpp_motion/init.lua", "camera.trace": cet + "utpp_camera/init.lua",
    }
    for key in DEFAULT_OFF:
        rel = cands[key]
        t = rel_to_text.get(rel)
        if t is None:
            continue
        m = re.search(r'key\s*=\s*"%s"(.{0,700}?)\bdefault\s*=\s*(true|false)' % re.escape(key), t, re.S)
        if not m:
            F.add("FAIL", "default-off", rel, "cannot find default of %s" % key)
        elif m.group(2) != "false":
            F.add("FAIL", "default-off", rel, "%s defaults to %s (diagnostics must be off in the release)" % (key, m.group(2)))
    vc = rel_to_text.get(cet + "utpp_vehiclecam/init.lua")
    if vc is not None:
        m = re.search(r"\bcamTrace\s*=\s*(true|false)", vc)
        if not m or m.group(1) != "false":
            F.add("FAIL", "default-off", cet + "utpp_vehiclecam/init.lua", "camTrace default is not false")
    bt = rel_to_text.get(cet + "utpp_bodyturn/init.lua")
    if bt is not None:
        m = re.search(r'key\s*=\s*"bodyturn\.shooterMode"(.{0,300}?)label\s*=\s*"([^"]*)"', bt, re.S)
        if not m or m.group(2) != "Shooter mode":
            F.add("FAIL", "label", cet + "utpp_bodyturn/init.lua",
                  'bodyturn.shooterMode label is %r, expected "Shooter mode"' % (m.group(2) if m else None))
    pv = rel_to_text.get(cet + "utpp_perspective/init.lua")
    if pv is not None and re.search(r"\(PL:", pv):
        F.add("WARN", "polish-text", cet + "utpp_perspective/init.lua", 'Polish "(PL: ...)" inserts remain in descriptions (C9, optional)')


REQUIRED_TEXT = {  # shipped file -> strings that must be in it (menu entries and the user texts of the release)
    P_REDS + "utpp_modsettings.reds": ['displayName", "Animation pack"', 'displayName", "Diagnostic logs"'],
    P_CET + "utpp_core/init.lua": ['label = "Animation pack"', '"Game Animations"', '"Ultimate Third Person Project"',
                                   'label = "Diagnostic logs"', '"On for 5 minutes"', '"On until switched off"'],
    README_REL: ["Animation pack", "Diagnostic logs", "How to report a problem", "utpp-report-"],
    NOTICES_REL: ["GASPALS", "CD PROJEKT RED", "Cyberpunk 2077"],
    LICENSE_REL: ["All rights reserved", "GASPALS", "NVIDIA", "CD PROJEKT RED"],
}


def check_early_pose_bytes(data):
    """Problems of an early pose switch file, parsed as the plugin parses it ([] = valid): first line exactly UTPP_EARLY_POSE_HELPERS, then
    1..1024 lines of exactly 16 hex digits (empty lines skipped), each a non-zero hash that occurs once."""
    try:
        text = data.decode("ascii")
    except UnicodeDecodeError:
        return ["not plain ASCII text (a byte order mark or other bytes break the header)"]
    lines = text.replace("\r\n", "\n").split("\n")
    if lines[0] != EARLY_POSE_HEADER:
        return ["first line is %r, expected %r" % (lines[0][:40], EARLY_POSE_HEADER)]
    problems, seen = [], set()
    for n, line in enumerate(lines[1:], 2):
        if not line:
            continue
        if len(line) != 16 or any(c not in "0123456789abcdefABCDEF" for c in line):
            problems.append("line %d is not a 16-digit hex hash: %r" % (n, line[:40]))
            continue
        value = int(line, 16)
        if not value or value in seen:
            problems.append("line %d: %s hash %s" % (n, "zero" if not value else "repeated", line))
        seen.add(value)
    if not seen:
        problems.append("no hash listed")
    if len(seen) > EARLY_POSE_MAX_HASHES:
        problems.append("%d hashes, the plugin accepts at most %d" % (len(seen), EARLY_POSE_MAX_HASHES))
    return problems


def check_early_pose(data, F, expected):
    """data: bytes of EARLY_POSE_REL in the package (None = not shipped). Without --early-pose (expected False) nothing is checked here
    (check_required refuses every early pose name). With it the file must ship and parse; it is then reported as expected (INFO)."""
    if not expected:
        return
    if data is None:
        F.add("FAIL", "early-pose", EARLY_POSE_REL, "--early-pose was given but the switch file is not in the package")
        return
    problems = check_early_pose_bytes(data)
    for x in problems:
        F.add("FAIL", "early-pose", EARLY_POSE_REL, x)
    if not problems:
        F.add("INFO", "early-pose", EARLY_POSE_REL, "expected: pose owner switch (early mapping stage), %d hashes" %
              len([l for l in data.decode("ascii").splitlines()[1:] if l]))


def early_pose_args_error(path, sha):
    """--early-pose and --early-pose-sha come together (a pinned file or none); the pin is a SHA-256. '' = fine."""
    if bool(path) != bool(sha):
        return "--early-pose and --early-pose-sha must be given together"
    if sha and not re.fullmatch(r"[0-9a-fA-F]{64}", sha):
        return "--early-pose-sha must be a SHA-256 (64 hex digits)"
    return ""


def check_required(rel_to_text, F, names, early_pose=False):
    """The Animation pack and Diagnostic logs menu entries, the diagnostic defaults and the README / licence texts must be staged.
    early_pose (--early-pose): EARLY_POSE_REL is expected; every other early pose name still fails."""
    for rel, needles in REQUIRED_TEXT.items():
        t = rel_to_text.get(rel)
        if t is None:
            F.add("FAIL", "required", rel, "required file is not in the package")
            continue
        for n in needles:
            if n not in t:
                F.add("FAIL", "required", rel, "required text not found: %s" % n)
    if nvidia_tools() and "NVIDIA" not in (rel_to_text.get(NOTICES_REL) or ""):
        F.add("FAIL", "required", NOTICES_REL, "NVIDIA notice missing")
    core = rel_to_text.get(P_CET + "utpp_core/init.lua")
    if core is not None:
        for key, want in (("M.DIAG_KEY", "off"), ("M.PACK_KEY", "utpp")):
            m = re.search(r"key\s*=\s*%s\b[^}]{0,160}?default\s*=\s*\"(\w+)\"" % re.escape(key), core, re.S)
            if not m or m.group(1) != want:
                F.add("FAIL", "default-off" if want == "off" else "required", P_CET + "utpp_core/init.lua",
                      "%s default is %r, expected %r" % (key, m.group(1) if m else None, want))
    for rel in sorted(names):  # the early pose experiment stays off: no early pose file of any name (unless explicitly packed)
        if early_pose and rel == EARLY_POSE_REL:
            continue
        if "early-pose" in rel.lower() or "early_pose" in rel.lower():
            F.add("FAIL", "forbidden-file", rel, "early pose files do not ship (the tested behaviour is: experiment off)")


def check_forbidden_names(rels, nue, F):
    """File names that must not be in the package (logs, databases, probes, ragdoll archives, the old skin helper folder name)."""
    for rel in rels:
        low = rel.lower()
        if re.search(r"\.(log|sqlite3|pdb|exp|lib)$", low) or low.endswith("utpp_settings.txt") or OLD_SKIN_FOLDER in low:
            F.add("FAIL", "forbidden-file", rel, "file name is not allowed in the package")
        if not nue and ("reload_gate" in low or "nue_reload" in low):
            F.add("FAIL", "forbidden-file", rel, "Nue reload file with --no-nue")
        if re.search(r"(deform|mount|reload|helper|ragdoll)_probe|utpp_probe|utpp_testcmd|utpp_bisect|local_test_profile|reload_phase"
                     r"|utpptesthost", low):
            F.add("FAIL", "forbidden-file", rel, "probe / bisect / test-profile file")
        if re.search(r"utpp_ragdoll_(poc|graph)|ragdoll.*\.archive", low):
            F.add("FAIL", "forbidden-file", rel, "ragdoll archive (not in 0.1.0, next version)")


def scan_text(rel, text, F, nue):
    lines = text.splitlines()
    for i, line in enumerate(lines, 1):
        for name, rx, exempt in TEXT_FAIL:
            if rel in exempt:
                continue
            m = rx.search(line)
            if m:
                F.add("FAIL", name, rel, line.strip(), i)
        if not rel.endswith(".json") and rel not in LEGACY_VERSION_EXEMPT:
            m = LEGACY_VERSION_RX.search(line)
            if m:
                F.add("FAIL", "legacy-name", rel, line.strip(), i)
        m = PROBE_RX.search(line)
        if m:
            cm = COMMENT_RX.search(line)
            lvl = "WARN" if (cm and cm.start() < m.start()) else "FAIL"
            F.add(lvl, "probe-name", rel, ("[comment] " if lvl == "WARN" else "") + line.strip(), i)
        if not nue and GATE_RX.search(line):
            F.add("FAIL", "reload-gate-name", rel, line.strip(), i)
        for name, rx in TEXT_WARN:
            if rx.search(line) and not (name == "abs-path" and rel.endswith(".json")):
                F.add("WARN", name, rel, line.strip(), i)
    if rel.endswith(".lua"):
        for line_no, field, txt in lua_visible_strings(text):
            for pat, label in [(r"shooterMode", "Shooter")] + [tuple(r) for r in LOCAL_RULES.get("visible_text_fail", [])]:
                if re.search(pat, txt, re.I):
                    F.add("FAIL", "visible-text", rel, "%s in %s: %s" % (label, field, txt.replace("\n", " ")), line_no)
            for pat, label in [(r"\(PL:", "Polish insert")] + [tuple(r) for r in LOCAL_RULES.get("visible_text_warn", [])]:
                if re.search(pat, txt, re.I):
                    F.add("WARN", "visible-text", rel, "%s in %s: %s" % (label, field, txt.replace("\n", " ")), line_no)


def scan_dll(path, F, rel, strict):
    data = Path(path).read_bytes()
    for m in re.finditer(rb"RSDS", data):
        end = data.find(b"\x00", m.start() + 24)
        pdb = data[m.start() + 24:end].decode("latin1", "replace")
        if end > 0 and 3 < len(pdb) < 400 and pdb.lower().endswith(".pdb"):
            if re.search(r"[\\/:]", pdb):
                F.add("FAIL", "pdb-path", rel, "debug directory embeds a full PDB path: %s (C12: /PDBALTPATH:%%_PDB%%)" % pdb)
            else:
                F.add("INFO", "pdb-path", rel, "PDB reference is the bare file name: %s" % pdb)
    low = data.lower()
    for cat, lst in BIN_NEEDLES.items():
        for n in lst:
            if n == AUTHOR_BYTES:
                continue  # author field of the plugin info
            for enc, needle in (("ascii", n), ("utf16", n.decode().encode("utf-16le"))):
                i = low.find(needle)
                if i >= 0:
                    ctx = re.sub(rb"[^\x20-\x7e]", b".", low[max(0, i - 30):i + len(needle) + 50]).decode()
                    F.add("FAIL", cat, rel, "%s string %r @0x%x: %s" % (enc, n.decode(), i, ctx))
    if AUTHOR_BYTES not in low and AUTHOR_BYTES.decode().encode("utf-16le") not in low:
        F.add("FAIL" if strict else "WARN", "author", rel, 'author string "%s" not found in the DLL (C13 not applied?)' % AUTHOR)
    # version: the DLL log line "UTPP <project version> state=..." (Main.cpp, UTPP_VERSION_STRING); the RED4ext plugin info carries the same
    if ("UTPP %s " % VERSION).encode() not in data:
        F.add("FAIL" if strict else "WARN", "version", rel, 'version text "UTPP %s " not found in the DLL (built with another project VERSION?)' % VERSION)
    return sha256_bytes(data)


# ---------------------------------------------------------------------------------------------- skin helper data audit
HEX64 = r"[0-9a-f]{64}"


def parse_approved_table(text):
    """Parse the table kApproved of BodyHelperAdmission.hpp -> ({body: (programSha256, bindingSha256)}, error or None).
    Exactly one entry per body (man, woman); anything else (missing block, malformed entry, duplicate or missing body) is an error."""
    m = re.search(r"\bkApproved\s*\{\{(.*?)\}\}\s*;", text, re.S)
    if not m:
        return {}, "table kApproved not found"
    block = re.sub(r"//[^\n]*", "", m.group(1))
    found = re.findall(r'\{\s*kBody(\w+)\s*,\s*"(%s)"\s*,\s*"(%s)"' % (HEX64, HEX64), block)
    if len(found) != len(re.findall(r"\bkBody\w+", block)):
        return {}, "an entry of kApproved is not {kBody<Name>, \"<64 hex>\", \"<64 hex>\", ...}"
    table = {}
    for name, prog, bind in found:
        body = name.lower()
        if body not in SKIN_BODIES:
            return {}, "kApproved has an entry for an unknown body kBody%s" % name
        if body in table:
            return {}, "kApproved has more than one entry for body %s" % body
        table[body] = (prog, bind)
    missing = [b for b in SKIN_BODIES if b not in table]
    if missing:
        return {}, "kApproved has no entry for body %s" % ", ".join(missing)
    return table, None


def load_approved_table(code_tree):
    p = Path(code_tree) / "plugin" / "UTPP" / native(SKIN_HEADER_SUB)
    try:
        text = p.read_text(encoding="utf-8")
    except OSError as ex:
        return {}, "cannot read %s: %s" % (p, ex)
    return parse_approved_table(text)


def check_skin_helpers(shipped, mode_bytes, table, table_err, dll_bytes, F):
    """Audit of the skin helper data of the package. shipped: {rel: sha256} of every file of the package; mode_bytes: content of
    skin_helpers/mode.txt (None when absent); table/table_err: parse_approved_table of the code tree header; dll_bytes: the DLL to ship.
    (a) exactly the five files under skin_helpers; (b) mode.txt is exactly APPLY + LF; (c) the SHA-256 of each bundle file equals the
    entry of its body in the table; (d) the DLL contains the four approved hash strings (a DLL older than the header lacks them)."""
    under = {r for r in shipped if r.lower().startswith(P_SKIN.lower())}
    want = set(SKIN_RELS)
    for r in sorted(want - under):
        F.add("FAIL", "skin-helpers-set", r, "required skin helper file is missing from the package")
    for r in sorted(under - want):
        F.add("FAIL", "skin-helpers-set", r, "unexpected file under skin_helpers (only mode.txt and man/woman program.ir + binding.dat ship)")
    if mode_bytes is not None and mode_bytes != SKIN_MODE_BYTES:
        F.add("FAIL", "skin-helpers-mode", SKIN_MODE_REL, "mode.txt must be exactly %r, found %r" % (SKIN_MODE_BYTES, mode_bytes[:20]))
    if table_err:
        F.add("FAIL", "skin-helpers-hash", SKIN_HEADER_SUB, "cannot read the approved hashes: %s" % table_err)
        return
    for body in SKIN_BODIES:
        for name, approved in zip(SKIN_BUNDLE_FILES, table[body]):
            rel = P_SKIN + body + "/" + name
            have = shipped.get(rel)
            if have is not None and have != approved:
                F.add("FAIL", "skin-helpers-hash", rel, "SHA-256 %s differs from the approved %s entry of BodyHelperAdmission.hpp (%s)"
                      % (have, body, approved))
    if dll_bytes is None:
        F.add("FAIL", "skin-helpers-dll", DLL_REL, "no DLL to check the approved hash strings against")
        return
    for body in SKIN_BODIES:
        for name, approved in zip(SKIN_BUNDLE_FILES, table[body]):
            if approved.encode("ascii") not in dll_bytes:
                F.add("FAIL", "skin-helpers-dll", DLL_REL, "the DLL does not contain the approved %s %s hash %s (DLL older than the "
                      "header or the bundles: rebuild it)" % (body, name, approved))


# ---------------------------------------------------------------------------------------------- manifest guard
# Record of the local, never published 2.31 package of 0.1.0. It must survive: never overwritten, not even with the flag.
PROTECTED_MANIFEST = ROOT / "Research" / "inventory" / "utpp-release-0.1.0.json"   # fixed name: not tied to VERSION
DEFAULT_MANIFEST = ROOT / "Research" / "inventory" / ("utpp-release-%s-game%s.json" % (VERSION.replace(".", ""), GAME_VERSION.replace(".", "")))


def _norm(p):
    return os.path.normcase(str(Path(p).resolve()))


def check_manifest_target(path, overwrite=False):
    """Returns an error text when `path` must not be written as the release manifest, else None."""
    if _norm(path) == _norm(PROTECTED_MANIFEST):
        return ("--manifest %s is the record of the unpublished 2.31 package and is never overwritten "
                "(default for this release: %s)" % (path, DEFAULT_MANIFEST.name))
    if Path(path).exists() and not overwrite:
        return "--manifest %s already exists (add --overwrite-manifest to replace it)" % path
    return None


# ---------------------------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description="UTPP %s release packer" % VERSION)
    ap.add_argument("--code-tree", required=True, help="repo/worktree root containing plugin\\UTPP (Source or Build\\wt-release)")
    ap.add_argument("--dll", required=True, help="UTPP.dll to ship")
    ap.add_argument("--out", required=True, help="output directory (Build\\release-%s)" % VERSION)
    ap.add_argument("--game", default=DEFAULT_GAME, help="game folder, data source (read only; default: UTPP_GAME_DIR or tools/local_paths.json)")
    ap.add_argument("--data-root", dest="data_roots", action="append", metavar="DIR",
                    help="extra folder searched for pinned data files by content (repeatable; default: <project>\\Data)")
    ap.add_argument("--skin-data-root", default=None, metavar="DIR",
                    help="folder with body_helper_man and body_helper_woman (default <project>\\Build\\test-data)")
    ap.add_argument("--stage-dir", default=None, metavar="DIR",
                    help="where the stage is built (default <out>\\stage); must be inside the project, e.g. Build\\stages\\utpp-0.1.0")
    ap.add_argument("--manifest", default=str(DEFAULT_MANIFEST),
                    help="final manifest (default %s); an existing file is refused "
                         "without --overwrite-manifest, %s (2.31 record) always" % (DEFAULT_MANIFEST.name, PROTECTED_MANIFEST.name))
    ap.add_argument("--overwrite-manifest", action="store_true", help="allow replacing an existing --manifest file")
    ap.add_argument("--dry-run", action="store_true", help="hash + scan in place, no stage/zip; report to <out>\\dryrun")
    ap.add_argument("--day-of-build", default="", metavar="GROUP=ship|skip,...",
                    help="decision for each in-game-tested group (%s); a final run requires all of them, a dry run assumes "
                         "ship for undecided groups" % ", ".join(DAYOF_GROUPS))
    ap.add_argument("--no-nue", dest="nue", action="store_false", help="leave the Nue authored reload out (default: included)")
    ap.add_argument("--allow-dirty", action="store_true", help="allow uncommitted shipped code files (never for the final run)")
    ap.add_argument("--allow-old-dll", action="store_true", help="allow the 0.6.5 DLL (5a291a88...)")
    ap.add_argument("--clean", action="store_true", help="delete the stage folder and the old zip first")
    ap.add_argument("--early-pose", default=None, metavar="FILE",
                    help="pack FILE as %s (pose owner switch: early mapping stage); default: no such file, FK stage. "
                         "Needs --early-pose-sha" % EARLY_POSE_REL.replace("/", "\\"))
    ap.add_argument("--early-pose-sha", default=None, metavar="HEX", help="pinned SHA-256 of the --early-pose file")
    args = ap.parse_args()
    early_err = early_pose_args_error(args.early_pose, args.early_pose_sha)
    if early_err:
        print("ERROR: " + early_err)
        return 2
    if not args.game:
        print("ERROR: the game folder is not configured: pass --game or set UTPP_GAME_DIR (or game_dir in tools/local_paths.json)")
        return 2

    out = Path(args.out).resolve()
    try:
        out.relative_to(ROOT)
    except ValueError:
        print("ERROR: --out must be inside %s (project rule: everything stays in the project folder)" % ROOT)
        return 2
    stage = Path(args.stage_dir).resolve() if args.stage_dir else out / "stage"
    if stage.parent not in (out, ROOT / "Build" / "stages"):  # --clean deletes it: only a direct child of <out> or Build\stages
        print("ERROR: --stage-dir must be a direct subfolder of --out or of %s" % (ROOT / "Build" / "stages"))
        return 2
    if stage == out or stage in out.parents:
        print("ERROR: --stage-dir must not contain --out")
        return 2
    mf_err = check_manifest_target(args.manifest, args.overwrite_manifest)
    if mf_err and not args.dry_run:  # before anything is staged or zipped
        print("ERROR: " + mf_err)
        return 2
    args.dayof, dayof_err = parse_dayof(args.day_of_build)
    if dayof_err:
        print("ERROR: " + dayof_err)
        return 2
    undecided = [g for g in DAYOF_GROUPS if g not in args.dayof]
    if undecided and not args.dry_run:  # before anything is staged or zipped
        print("ERROR: day-of-build decision missing for: %s\n" % ", ".join(undecided))
        print(dayof_checklist(args.dayof))
        return 2
    errors = []
    entries = build_plan(args, errors)
    graph = any(e.rel == GRAPH_REL for e in entries)  # the parried graph ships only when the melee pack needs it
    notices = gen_notices(errors, graph)
    for e in entries:
        if e.rel == README_REL:
            e.data = gen_readme("melee" in dayof_shipped(args), graph, bool(args.early_pose))
        elif e.rel == NOTICES_REL:
            e.data = notices
    if errors:
        print("PLAN ERRORS:")
        for x in errors:
            print("  -", x)
        return 2
    dup = {}
    for e in entries:
        dup[e.rel.lower()] = dup.get(e.rel.lower(), 0) + 1
    bad = [k for k, v in dup.items() if v > 1]
    if bad:
        print("ERROR: duplicate ship paths:", bad)
        return 2

    tree_root = Path(args.code_tree)
    rc, commit = run_git(tree_root, "rev-parse", "HEAD")
    commit = commit if rc == 0 else "unknown"
    rc, branch = run_git(tree_root, "rev-parse", "--abbrev-ref", "HEAD")
    branch = branch if rc == 0 else "unknown"
    code_paths = ["plugin/UTPP/" + sub for _, sub, _ in CODE + (CODE_NUE if args.nue else [])]
    code_paths += ["plugin/UTPP/" + sub for g, _, sub, _ in DAYOF_CODE if g in dayof_shipped(args)]
    code_paths.append("LICENSE")
    code_paths.append("plugin/UTPP/" + SKIN_HEADER_SUB)   # source of the approved skin helper hashes the DLL was built from
    rc, dirty_out = run_git(tree_root, "status", "--porcelain", "--", *code_paths)
    dirty = [l for l in dirty_out.splitlines() if l.strip()] if rc == 0 else ["git status failed"]
    print("UTPP %s release packer | mode=%s | nue=%s | pose owner=%s" % (VERSION, "DRY-RUN" if args.dry_run else "FINAL", args.nue,
                                                                   "Mapping (--early-pose)" if args.early_pose else "FK"))
    print("code tree : %s  commit %s (%s)  dirty shipped files: %d" % (tree_root, commit[:12], branch, len(dirty)))
    for l in dirty:
        print("            dirty:", l)
    F = Findings()
    if not LOCAL_RULES:
        F.add("WARN", "local-rules", str(LOCAL_RULES_FILE), "local rules file missing or unreadable: only the generic checks run")
    for what, p in getattr(args, "soft_missing", []):  # dry run only; a final run stops with PLAN ERRORS
        F.add("FAIL", "missing-in-game", str(p), "%s not in the game folder (not installed yet?)" % what)
    if mf_err:  # dry run only (a final run returned above)
        F.add("WARN", "manifest-target", args.manifest, mf_err)
    if dirty:
        F.add("WARN" if (args.dry_run or args.allow_dirty) else "FAIL", "dirty-tree", str(tree_root),
              "%d shipped code files are not committed (commit before the final run)" % len(dirty))

    for g, (title, _) in DAYOF_GROUPS.items():
        if args.dry_run:
            F.add("WARN", "day-of-build", g, "%s: decision %s - confirm after the in-game test on the build day "
                  "(final run needs --day-of-build %s=ship|skip)" % (title, args.dayof.get(g, "undecided (assumed ship)"), g))
    pending = []
    for rel, why in DATA_PENDING:
        gp = Path(args.game) / native(rel)
        if gp.is_file():
            psha, psize, _ = pass_file(gp, False)
            info = "game copy %d B sha256 %s (not pinned, not shipped)" % (psize, psha)
        else:
            info = "no game copy"
        pending.append({"path": rel.replace("/", "\\"), "reason": why, "gameCopy": info})
        F.add("WARN" if args.dry_run else "FAIL", "pending-in-game-check", rel,
              "pending in-game check (%s): %s; pin it in DATA after the check" % (why, info))

    zip_path = out / ("UTPP-%s.zip" % VERSION)
    work = out / "dryrun" if args.dry_run else out
    work.mkdir(parents=True, exist_ok=True)
    if not args.dry_run:
        if args.clean:
            if stage.exists():
                shutil.rmtree(stage)
            if zip_path.exists():
                zip_path.unlink()
        if stage.exists() and any(stage.iterdir()):
            print("ERROR: %s is not empty (use --clean)" % stage)
            return 2
        need = sum(os.path.getsize(e.src) for e in entries if e.src and e.src.is_file()) * 2.6
        free = shutil.disk_usage(out).free
        if free < need:
            print("ERROR: free space %s < needed ~%s" % (fmt_mib(free), fmt_mib(need)))
            return 2

    # ---- 1. sources: hash, verify pins, stage (or scan in place)
    texts = {}  # rel -> text for text scans / defaults
    total = 0
    for n, e in enumerate(entries, 1):
        if e.kind == "gen":
            e.sha, e.size = sha256_bytes(e.data), len(e.data)
        elif e.kind in ("code", "dll", "xform", "skin"):
            raw = e.src.read_bytes()
            e.src_sha = sha256_bytes(raw)
            if e.kind == "xform":
                if e.src_sha != e.expect_sha:
                    F.add("FAIL", "sha-mismatch", e.rel, "source sha %s != pinned %s" % (e.src_sha, e.expect_sha))
                try:
                    e.data = xform_chooser(raw, e.rel) if e.rel.endswith(".json") else xform_style(raw)
                except ValueError as ex:
                    F.add("FAIL", "transform", e.rel, str(ex))
                    e.data = raw
            else:
                e.data = raw
                if e.kind == "code" and e.expect_sha and e.src_sha != e.expect_sha:
                    F.add("FAIL", "sha-mismatch", e.rel, "code file sha %s != tested game copy %s" % (e.src_sha, e.expect_sha))
            e.sha, e.size = sha256_bytes(e.data), len(e.data)
        else:  # data
            if not args.dry_run:
                src_sha, src_size, _ = pass_file(e.src, False)
                e.src_sha = src_sha
                if src_sha != e.expect_sha or src_size != e.expect_size:
                    F.add("FAIL", "sha-mismatch", e.rel, "game file sha/size %s/%d != pinned %s/%d" %
                          (src_sha, src_size, e.expect_sha, e.expect_size))
        total += 1

    if args.dry_run:
        pass
    else:
        for e in entries:
            dst = stage / native(e.rel)
            dst.parent.mkdir(parents=True, exist_ok=True)
            if e.data is not None:
                dst.write_bytes(e.data)
            else:
                shutil.copy2(e.src, dst)  # copy only: never move, never hard-link
            if dst.stat().st_nlink != 1:
                F.add("FAIL", "hardlink", e.rel, "stage file has %d links" % dst.stat().st_nlink)

    # ---- 2. verify + scan every file (data: one pass; small: from memory)
    for e in entries:
        if e.kind == "data":
            target = e.src if args.dry_run else stage / native(e.rel)
            sha, size, hits = pass_file(target, True)
            e.sha, e.size, e.hits = sha, size, hits
            if sha != e.expect_sha or size != e.expect_size:
                F.add("FAIL", "sha-mismatch", e.rel, "%s sha/size %s/%d != pinned %s/%d" %
                      ("game" if args.dry_run else "stage copy", sha, size, e.expect_sha, e.expect_size))
            for cat, needle, pos, ctx in hits:
                F.add("WARN" if cat == "extract-ref" else "FAIL", cat, e.rel, "binary hit %r @0x%x: %s" % (needle, pos, ctx))
        elif not args.dry_run:
            st = (stage / native(e.rel)).read_bytes()
            if sha256_bytes(st) != e.sha:
                F.add("FAIL", "sha-mismatch", e.rel, "stage copy differs from source")
        if e.kind == "skin":  # binary needle scan (personal data, dev paths, names) of the bundle file
            for cat, needle, pos, ctx in pass_file(e.src if args.dry_run else stage / native(e.rel), True)[2]:
                F.add("WARN" if cat == "extract-ref" else "FAIL", cat, e.rel, "binary hit %r @0x%x: %s" % (needle, pos, ctx))
        if e.kind != "data" and e.ext in TEXT_EXT:
            t = e.data.decode("utf-8", "replace")
            texts[e.rel] = t
            scan_text(e.rel, t, F, args.nue)
        if e.kind == "data" and e.ext in TEXT_EXT:
            t = (e.src if args.dry_run else stage / native(e.rel)).read_bytes().decode("utf-8", "replace")
            scan_text(e.rel, t, F, args.nue)
        if e.kind == "dll":
            scan_dll(e.src, F, e.rel, not args.dry_run)
            if e.sha == OLD_DLL_SHA:
                F.add("WARN" if (args.dry_run or args.allow_old_dll) else "FAIL", "old-dll", e.rel,
                      "DLL is the 0.6.5 build (5a291a88...): full PDB path, old version/author (rebuild for %s)" % VERSION)
    names = {e.rel for e in entries}
    check_defaults(texts, F)
    check_required(texts, F, names, bool(args.early_pose))
    early_e = next((e for e in entries if e.rel == EARLY_POSE_REL), None)
    check_early_pose((early_e.src if args.dry_run else stage / native(EARLY_POSE_REL)).read_bytes() if early_e else None, F,
                     bool(args.early_pose))
    approved, approved_err = load_approved_table(tree_root)
    mode_e = next((e for e in entries if e.rel == SKIN_MODE_REL), None)
    dll_e = next((e for e in entries if e.kind == "dll"), None)
    check_skin_helpers({e.rel: e.sha for e in entries}, mode_e.data if mode_e else None, approved, approved_err,
                       dll_e.data if dll_e else None, F)
    check_forbidden_names([e.rel for e in entries], args.nue, F)
    if not args.dry_run:
        on_disk = {p.relative_to(stage).as_posix() for p in stage.rglob("*") if p.is_file()}
        if on_disk != names:
            F.add("FAIL", "stage-set", str(stage), "stage differs from ship list: extra=%s missing=%s" %
                  (sorted(on_disk - names)[:5], sorted(names - on_disk)[:5]))

    # ---- 3. manifest
    data_bytes = sum(e.size for e in entries if e.kind in ("data", "skin"))
    files = []
    for e in sorted(entries, key=lambda x: x.rel.lower()):
        d = {"path": e.rel.replace("/", "\\"), "size": e.size, "sha256": e.sha, "kind": e.kind, "group": e.group,
             "source": (str(e.src) if e.src else "generated by make_release.py")}
        if e.kind in ("code", "xform", "dll") and e.src_sha:
            d["sourceSha256"] = e.src_sha
        if e.kind == "code":
            d["commit"] = commit
        if e.kind == "xform":
            d["transform"] = "C10 neutral licence text"
        if e.kind == "data":
            d["pinnedSha256"] = e.expect_sha
        if e.kind == "skin":  # the hash is not pinned in this tool: it is the DLL's approved entry (checked above)
            d["sourceSha256"] = e.src_sha
            body, fname = e.rel.split("/")[-2:]
            d["approvedSha256"] = None if approved_err else approved[body][SKIN_BUNDLE_FILES.index(fname)]
            d["approvedIn"] = SKIN_HEADER_SUB
        files.append(d)
    manifest = {
        "release": "UTPP " + VERSION, "gameVersion": GAME_VERSION, "author": AUTHOR,
        "nvidiaNotice": {"motionbricks": NOTICE_MOTIONBRICKS, "kimodo": NOTICE_KIMODO},"createdAt": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "mode": "dry-run" if args.dry_run else "final", "nueReload": args.nue,
        "poseOwner": ("Mapping (%s, pinned %s)" % (EARLY_POSE_REL.replace("/", "\\"), args.early_pose_sha.lower())
                      if args.early_pose else "FK (no early pose file)"),
        "dayOfBuild": {g: args.dayof.get(g, "undecided (assumed ship)") for g in DAYOF_GROUPS},
        "pendingFiles": pending, "codeTree": str(tree_root), "commit": commit, "branch": branch, "dirtyShippedFiles": dirty,
        "dll": {"path": str(args.dll), "sha256": next(e.sha for e in entries if e.kind == "dll")},
        "gameFolder": args.game, "fileCount": len(entries), "dataBytes": data_bytes,
        "totalBytes": sum(e.size for e in entries), "files": files,
    }

    # ---- 4. zip + extraction check (final only)
    if not args.dry_run:
        print("zipping %d files ..." % len(entries))
        with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED, compresslevel=6) as zf:
            for e in sorted(entries, key=lambda x: x.rel.lower()):
                zf.write(stage / native(e.rel), e.rel)  # arcname = game-relative path, no parent folder
        zsha = sha256_of(zip_path)
        manifest["zip"] = {"name": zip_path.name, "size": zip_path.stat().st_size, "sha256": zsha}
        tmp = Path(tempfile.mkdtemp(prefix="verify-", dir=str(out)))
        ok = True
        with zipfile.ZipFile(zip_path) as zf:
            members = [i for i in zf.infolist() if not i.is_dir()]
            names_z = {i.filename for i in members}
            if names_z != names:
                F.add("FAIL", "zip-set", zip_path.name, "zip members differ from ship list")
                ok = False
            if any(n.startswith("/") or ".." in n.split("/") or n.startswith("UTPP") for n in names_z):
                F.add("FAIL", "zip-root", zip_path.name, "zip has a parent folder or unsafe names")
            if zf.testzip() is not None:
                F.add("FAIL", "zip-crc", zip_path.name, "CRC error in " + zf.testzip())
            zf.extractall(tmp)
        by_rel = {e.rel: e for e in entries}
        for rel in sorted(names_z if ok else names):
            p = tmp / native(rel)
            if not p.is_file() or sha256_of(p) != by_rel[rel].sha:
                F.add("FAIL", "extract-hash", rel, "extracted file hash differs from the manifest")
                ok = False
        if ok:
            shutil.rmtree(tmp)
            print("zip extraction check: %d files, all hashes match the manifest" % len(names_z))
        else:
            print("zip extraction check FAILED; scratch kept in", tmp)
    manifest["scan"] = {"fail": F.count("FAIL"), "warn": F.count("WARN"), "info": F.count("INFO")}

    # ---- 5. report
    checklist = dayof_checklist({g: args.dayof.get(g, "undecided (assumed ship)") for g in DAYOF_GROUPS})
    (work / "day-of-build-checklist.txt").write_text(checklist + "\n", encoding="utf-8")
    print("\n" + checklist)
    report_path =(work / "report.json") if args.dry_run else (out / "scan-report.json")
    report_path.write_text(json.dumps({"findings": F.items, "summary": manifest["scan"]}, indent=1), encoding="utf-8")
    if args.dry_run:
        mpath = work / "manifest-dryrun.json"
    elif F.count("FAIL"):
        mpath = out / "manifest-FAILED.json"  # never overwrite the official manifest with a failed run
    else:
        mpath = Path(args.manifest)
    mpath.parent.mkdir(parents=True, exist_ok=True)
    if mpath == Path(args.manifest):  # the official manifest: re-check right before writing, never replace silently
        err = check_manifest_target(mpath, args.overwrite_manifest)
        if err:
            print("ERROR: " + err)
            return 2
    with open(mpath, "w" if (args.dry_run or mpath.name == "manifest-FAILED.json" or args.overwrite_manifest) else "x",
              encoding="utf-8") as mf:
        mf.write(json.dumps(manifest, indent=1))

    print("\nfiles: %d | data (without generated/code): %d B (%s) | total payload: %d B (%s)" %
          (len(entries), data_bytes, fmt_mib(data_bytes), manifest["totalBytes"], fmt_mib(manifest["totalBytes"])))
    groups = {}
    for i in F.items:
        groups.setdefault((i["level"], i["check"]), []).append(i)
    order = {"FAIL": 0, "WARN": 1, "INFO": 2}
    print("\nscan: %d FAIL, %d WARN, %d INFO" % (F.count("FAIL"), F.count("WARN"), F.count("INFO")))
    for (lvl, chk), items in sorted(groups.items(), key=lambda kv: (order[kv[0][0]], kv[0][1])):
        limit = 12 if lvl == "FAIL" else 3
        print("  [%s] %s: %d" % (lvl, chk, len(items)))
        for i in items[:limit]:
            print("      %s%s: %s" % (i["file"], (":%s" % i["line"]) if i["line"] else "", i["detail"][:170]))
        if len(items) > limit:
            print("      ... %d more (see %s)" % (len(items) - limit, report_path))
    if not args.dry_run:
        print("\nzip: %s  %d B (%s)  sha256 %s" % (zip_path, zip_path.stat().st_size, fmt_mib(zip_path.stat().st_size), zsha))
    if not args.dry_run and F.count("FAIL"):
        print("*** SCAN FAILED - DO NOT UPLOAD THIS ZIP ***")
    print("manifest:", mpath)
    print("report  :", report_path)
    return 1 if F.count("FAIL") else 0


if __name__ == "__main__":
    sys.exit(main())
