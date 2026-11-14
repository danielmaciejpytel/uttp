"""Offline tests for make_release.py (no game access, no staging). Run: python -I -B Source\\tools\\release\\test_make_release.py"""
import importlib.util
import os
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("make_release", HERE / "make_release.py")
mr = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mr)


class ManifestGuard(unittest.TestCase):
    def test_default_is_not_the_protected_file(self):
        self.assertNotEqual(mr._norm(mr.DEFAULT_MANIFEST), mr._norm(mr.PROTECTED_MANIFEST))
        self.assertTrue(mr.DEFAULT_MANIFEST.name.endswith("-game250.json"))
        self.assertRegex(mr.DEFAULT_MANIFEST.name, r"^[A-Za-z0-9_-]+\.json$")  # the installer's -ManifestName rule

    def test_protected_refused_even_with_overwrite(self):
        for ow in (False, True):
            self.assertIsNotNone(mr.check_manifest_target(mr.PROTECTED_MANIFEST, ow))
        # different spelling of the same file (case, redundant segments)
        alt = str(mr.PROTECTED_MANIFEST.parent / ".." / "inventory" / mr.PROTECTED_MANIFEST.name.upper())
        self.assertIsNotNone(mr.check_manifest_target(alt, True))

    def test_existing_needs_flag_new_is_free(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "m.json"
            self.assertIsNone(mr.check_manifest_target(p, False))  # new path
            p.write_text("{}")
            self.assertIsNotNone(mr.check_manifest_target(p, False))
            self.assertIsNone(mr.check_manifest_target(p, True))


class GeneratedTexts(unittest.TestCase):
    def setUp(self):
        self.old = (mr.NOTICE_MOTIONBRICKS, mr.NOTICE_KIMODO)

    def tearDown(self):
        mr.NOTICE_MOTIONBRICKS, mr.NOTICE_KIMODO = self.old

    def texts(self):
        errs = []
        return mr.gen_readme().decode(), mr.gen_notices(errs).decode(), errs

    def test_readme_content(self):
        r, _, _ = self.texts()
        for s in ("utpp_interactrange.disable", "utpp_locomotion.disable", "Input Loader", "two-handed blunt weapons", "2.50"):
            self.assertIn(s, r)
        self.assertNotIn("free-look", r.lower())
        self.assertNotIn("{", r)

    def test_nvidia_conditional(self):
        mr.NOTICE_MOTIONBRICKS, mr.NOTICE_KIMODO = True, False
        r, n, errs = self.texts()
        self.assertEqual(errs, [])
        self.assertIn("NVIDIA MotionBricks", n)
        self.assertIn("NVIDIA Open Model Agreement", n)
        self.assertNotIn("Kimodo", n)
        self.assertIn("or NVIDIA", r)
        mr.NOTICE_KIMODO = True
        _, n, _ = self.texts()
        self.assertIn("NVIDIA Kimodo", n)
        self.assertIn("NVIDIA Open Model License", n)
        mr.NOTICE_MOTIONBRICKS = mr.NOTICE_KIMODO = False
        r, n, _ = self.texts()
        self.assertNotIn("NVIDIA", n)
        self.assertNotIn("NVIDIA", r)

    def test_texts_pass_own_scanner(self):
        for mb, kim in ((True, False), (True, True), (False, False)):
            mr.NOTICE_MOTIONBRICKS, mr.NOTICE_KIMODO = mb, kim
            F = mr.Findings()
            r, n, _ = self.texts()
            mr.scan_text(mr.README_REL, r, F, True)
            mr.scan_text(mr.NOTICES_REL, n, F, True)
            bad = [i for i in F.items if i["level"] in ("FAIL", "WARN")]
            self.assertEqual(bad, [], bad)

    def test_derived_and_sourcepack_wording(self):
        _, n, _ = self.texts()
        self.assertIn("GASPALS 1.2", n)
        self.assertIn("Polygon Hive", n)
        self.assertIn("CD PROJEKT RED", n)
        self.assertNotIn("mocap", n.lower())


class ReadmeRequirements(unittest.TestCase):
    def test_tested_with_versions_and_input_loader(self):
        r = mr.gen_readme().decode()
        for s in ("RED4ext 1.31.1", "redscript 0.5.32", "Cyber Engine Tweaks 1.38.1", "Codeware 1.21.4", "ArchiveXL 1.28.8",
                  "Input Loader 0.2.6", "Mod Settings 0.2.30"):
            self.assertIn(s, r)
        tested = r.split("Tested with", 1)[1].split("(versions installed", 1)[0]
        self.assertIn("Mod Settings 0.2.30", tested)
        self.assertIn("Requires RED4ext, redscript, Cyber Engine Tweaks, Codeware, ArchiveXL and Input Loader", r)
        for old in ("1.30.0", "0.5.31", "1.37.1", "1.20.6", "1.27.5"):
            self.assertNotIn(old, r)

    def test_melee_line_follows_decision(self):
        self.assertIn("two-handed blunt weapons", mr.gen_readme(True).decode())
        off = mr.gen_readme(False).decode()
        self.assertNotIn("two-handed blunt weapons", off)
        self.assertIn("not part of this version", off)
        self.assertNotIn("{", off)


class DayOfBuild(unittest.TestCase):
    def test_pins_and_files(self):
        data = {p: (g, s, h) for g, p, s, h in mr.DAYOF_DATA}
        code = {p: (g, h) for g, p, _, h in mr.DAYOF_CODE}
        for p, (g, s, h) in data.items():
            self.assertEqual(len(h), 64)
            self.assertGreater(s, 0)
        for p, (g, h) in code.items():
            self.assertEqual(len(h), 64)
        want = {
            "melee": {mr.P_ARC + "utpp_melee.archive", mr.P_REDS + "utpp_melee.reds", mr.P_ARC + "utpp_melee_tpp.archive.xl",
                      mr.P_DATA + "utpp_melee_pose_female.bin", mr.P_DATA + "utpp_melee_pose_male.bin"},
            "car": {mr.P_ARC + "utpp_vehicle_car.archive"} | {mr.P_REDS + n + ".reds" for n in (
                "utpp_vehicle_approach", "utpp_vehicle_approach_table", "utpp_vehicle_door", "utpp_vehicle_door_timing")},
        }
        for g, files in want.items():
            have = {p for p, v in data.items() if v[0] == g} | {p for p, v in code.items() if v[0] == g}
            self.assertEqual(have, files)
        self.assertEqual(data[mr.P_ARC + "utpp_melee.archive"][1:], (26632192,
                         "b978453a91c3240c5b2e999f61ccee19710cfe439ffeff662dc25ebb6f7fbb4b"))  # fists + knife, katana, blunt, baton, two-handed blunt
        # the bike archive was checked in game (2026-11-10) and is pinned in the core DATA list, not in a day-of group
        self.assertNotIn(mr.P_ARC + "utpp_vehicle_bike.archive", [r for r, _ in mr.DATA_PENDING])
        self.assertNotIn(mr.P_ARC + "utpp_vehicle_bike.archive", data)
        self.assertEqual({r: (n, s) for r, n, s in mr.DATA}[mr.P_ARC + "utpp_vehicle_bike.archive"],
                         (2105344, "9f6a1d1bdcbb89de14805dc9f4246601dc19f9f66d86912f0c3d65acb756746a"))

    def test_no_duplicates_with_core_lists(self):
        core = {r for r, _, _ in mr.DATA} | {r for r, _, _ in mr.CODE} | {r for r, _, _ in mr.DATA_NUE}
        core |= {r for r, _, _ in mr.CODE_NUE} | {r for r, _, _ in mr.TRANSFORMS}
        for _, p, *_ in mr.DAYOF_DATA + mr.DAYOF_CODE:
            self.assertNotIn(p, core)

    def test_parse(self):
        self.assertEqual(mr.parse_dayof("melee=ship, car=skip"), ({"melee": "ship", "car": "skip"}, None))
        self.assertEqual(mr.parse_dayof(""), ({}, None))
        for bad in ("melee", "melee=yes", "bike=ship", "melee=ship,melee=skip"):
            self.assertIsNotNone(mr.parse_dayof(bad)[1], bad)

    def test_plan_follows_decisions(self):
        import argparse
        for dec, melee, car in (({"melee": "ship", "car": "ship"}, True, True), ({"melee": "skip", "car": "ship"}, False, True),
                                ({"melee": "ship", "car": "skip"}, True, False), ({}, True, True)):
            a = argparse.Namespace(game=r"X:\nogame", code_tree=r"X:\notree", dll=r"X:\none.dll", dry_run=False, nue=True, dayof=dec,
                                   data_roots=[r"X:\nodata"])
            ents = mr.build_plan(a, [])
            rels = {e.rel for e in ents}
            self.assertEqual(mr.P_ARC + "utpp_melee.archive" in rels, melee)
            self.assertEqual(mr.P_REDS + "utpp_melee.reds" in rels, melee)
            self.assertEqual(mr.P_ARC + "utpp_melee_tpp.archive.xl" in rels, melee)
            self.assertEqual(mr.P_REDS + "utpp_vehicle_door.reds" in rels, car)
            for e in ents:
                if e.rel in (mr.P_REDS + "utpp_melee.reds", mr.P_REDS + "utpp_vehicle_door.reds"):
                    self.assertTrue(e.expect_sha)

    def test_final_run_needs_decisions(self):
        import contextlib
        import io
        out = mr.ROOT / "Build" / "work" / "release-tool-20261109" / "test-undecided-out"
        self.assertFalse(out.exists())
        for spec in ("", "melee=ship"):
            argv = ["make_release.py", "--code-tree", str(mr.ROOT / "Source"), "--dll", "x.dll", "--out", str(out),
                    "--manifest", str(out / "m.json"), "--day-of-build", spec]
            old = sys.argv
            sys.argv = argv
            buf = io.StringIO()
            try:
                with contextlib.redirect_stdout(buf):
                    rc = mr.main()
            finally:
                sys.argv = old
            self.assertEqual(rc, 2)
            self.assertIn("day-of-build decision missing", buf.getvalue())
            self.assertIn("Day-of-build checklist", buf.getvalue())
            self.assertFalse(out.exists())


MELEE_SHA = next(s for _, p, _, s in mr.DAYOF_DATA if p == mr.P_ARC + "utpp_melee.archive")


def states_text(*lines):
    return "x\n// UTPP_STATES_TABLE begin\n" + "".join("  %s\n" % l for l in lines) + "  // UTPP_STATES_TABLE end\n"


def state_line(var, mask):
    return 'if Equals(n, n"%s") { return %d; }' % (var, mask)


class ParriedGraph(unittest.TestCase):
    KNIFE = {"w_melee_fists": "generic", "w_melee_knife": "per-strike", "w_melee_katana": "generic"}

    def plan(self, reds_lines, dec, kinds=None):
        """build_plan against a scratch code tree whose utpp_melee.reds holds the given state lines (no game, no staging)."""
        import argparse
        from unittest import mock
        with tempfile.TemporaryDirectory() as d:
            sc = Path(d) / "plugin" / "UTPP" / "scripts"
            sc.mkdir(parents=True)
            (sc / "utpp_melee.reds").write_text(states_text(*reds_lines), encoding="utf-8")
            a = argparse.Namespace(game=r"X:\nogame", code_tree=d, dll=r"X:\none.dll", dry_run=False, nue=True, dayof=dec,
                                   data_roots=[r"X:\nodata"])
            errs = []
            with mock.patch.dict(mr.PARRIED_CLIPS, {MELEE_SHA: kinds or self.KNIFE}):
                ents = mr.build_plan(a, errs)
            return ents, [e for e in errs if "parried graph rule" in e]

    def test_pin(self):
        self.assertEqual(mr.GRAPH_SHA, "D5786BB159CEC4AD8A334B15B6762785A5B587E1DF0664A79DE91F0CFDC87449".lower())
        self.assertEqual(mr.GRAPH_REL, mr.P_ARC + "utpp_melee_parried_graph.archive")
        self.assertEqual(mr.GRAPH_SIZE, 53248)
        self.assertNotIn(mr.GRAPH_REL, [p for _, p, _, _ in mr.DAYOF_DATA])  # optional: not one of the melee group's pinned files

    def test_rule(self):
        t = lambda *l: states_text(*[state_line(v, m) for v, m in l])
        # fists only: generic parried clips, no graph (0.1.0 configuration)
        self.assertEqual(mr.parried_graph_rule(t(("w_melee_fists", 3669472)), MELEE_SHA), (False, []))
        # the combined fists archive of the game copy (d683748f..., to be pinned): fists generic too
        combined = "d683748f5339f7b6955f7f79c9a39a5f1a7602016967881dc0df64d4258656cf"
        self.assertEqual(mr.parried_graph_rule(t(("w_melee_fists", 3669472)), combined), (False, []))
        # the real script of the tree with the pinned pack: knife, katana, blunt, baton and two-handed blunt have per-strike parried
        # clips -> graph
        real =(mr.ROOT / "Source" / "plugin" / "UTPP" / "scripts" / "utpp_melee.reds").read_text(encoding="utf-8")
        self.assertEqual(mr.parried_graph_rule(real, MELEE_SHA), (True, []))
        self.assertEqual(mr.parse_states_table(real), ({"w_melee_fists": 3669472, "w_melee_knife": 3668192, "w_melee_katana": 3669472,
                                                        "w_melee_one_hand_blunt": 3669472, "w_melee_baton": 3669472,
                                                        "w_two_hand_blunt": 3669472}, []))
        # the same lines without the Parried bit (graph-less fallback): not needed
        self.assertEqual(mr.parried_graph_rule(t(("w_melee_knife", 3668160), ("w_melee_katana", 3669440),
                                                 ("w_melee_one_hand_blunt", 3669440), ("w_melee_baton", 3669440),
                                                 ("w_two_hand_blunt", 3669440)), MELEE_SHA), (False, []))
        from unittest import mock
        with mock.patch.dict(mr.PARRIED_CLIPS, {MELEE_SHA: self.KNIFE}):
            self.assertEqual(mr.parried_graph_rule(t(("w_melee_knife", 3668192)), MELEE_SHA), (True, []))
            self.assertEqual(mr.parried_graph_rule(t(("w_melee_fists", 3669472), ("w_melee_knife", 3668192)), MELEE_SHA), (True, []))
            # Parried bit off (graph-less knife 3668160 / katana 3669440): not needed
            self.assertEqual(mr.parried_graph_rule(t(("w_melee_knife", 3668160), ("w_melee_katana", 3669440)), MELEE_SHA), (False, []))
            # katana with the bit but only generic clips (kind says so): not needed
            self.assertEqual(mr.parried_graph_rule(t(("w_melee_katana", 3669472)), MELEE_SHA), (False, []))
            # a weapon with the bit that is not classified: plan error
            needed, problems = mr.parried_graph_rule(t(("w_melee_hammer", 3669472)), MELEE_SHA)
            self.assertTrue(problems and "w_melee_hammer" in problems[0])
        # an unknown melee archive with a Parried line: plan error (fists too: the pin and the table move together)
        needed, problems = mr.parried_graph_rule(t(("w_melee_fists", 3669472)), "0" * 64)
        self.assertTrue(problems)
        # unparseable / duplicate lines and missing markers are problems, never skipped
        for text in (states_text('if (Equals(n, n"w_melee_fists")) { return 3669472; }'),
                     states_text(state_line("w_melee_fists", 3669472), state_line("w_melee_fists", 3669472)), "no markers"):
            self.assertTrue(mr.parried_graph_rule(text, MELEE_SHA)[1], text)
        # comments and blank lines between the markers are fine
        self.assertEqual(mr.parse_states_table(states_text("// note", "", state_line("w_melee_fists", 5)))[0], {"w_melee_fists": 5})

    def test_plan_includes_graph_only_when_needed(self):
        shipped = {"melee": "ship", "car": "skip"}
        ents, errs = self.plan([state_line("w_melee_fists", 3669472)], shipped)
        self.assertEqual((errs, mr.GRAPH_REL in {e.rel for e in ents}), ([], False))
        ents, errs = self.plan([state_line("w_melee_fists", 3669472), state_line("w_melee_knife", 3668192)], shipped)
        self.assertEqual(errs, [])
        g = [e for e in ents if e.rel == mr.GRAPH_REL]
        self.assertEqual(len(g), 1)
        self.assertEqual((g[0].kind, g[0].group, g[0].expect_sha, g[0].expect_size), ("data", "melee", mr.GRAPH_SHA, 53248))
        ents, errs = self.plan([state_line("w_melee_knife", 3668160)], shipped)
        self.assertEqual((errs, mr.GRAPH_REL in {e.rel for e in ents}), ([], False))
        # melee skipped: the group's files, the graph included, stay out even if the script were shipped
        ents, errs = self.plan([state_line("w_melee_knife", 3668192)], {"melee": "skip", "car": "skip"})
        self.assertEqual((errs, mr.GRAPH_REL in {e.rel for e in ents}), ([], False))
        # unclassified Parried-bit weapon: plan error, no graph guessed
        ents, errs = self.plan([state_line("w_melee_hammer", 3669472)], shipped)
        self.assertEqual(len(errs), 1)
        self.assertNotIn(mr.GRAPH_REL, {e.rel for e in ents})

    def test_current_configuration_includes_graph(self):
        import argparse
        a = argparse.Namespace(game=r"X:\nogame", code_tree=str(mr.ROOT / "Source"), dll=r"X:\none.dll", dry_run=False, nue=True,
                               dayof={"melee": "ship", "car": "ship"}, data_roots=[r"X:\nodata"])
        errs = []
        ents = mr.build_plan(a, errs)
        g = [e for e in ents if e.rel == mr.GRAPH_REL]
        self.assertEqual([(e.group, e.expect_sha, e.expect_size) for e in g], [("melee", mr.GRAPH_SHA, mr.GRAPH_SIZE)])
        self.assertFalse([e for e in errs if e.startswith("parried graph rule:")], errs)
        # melee skipped: no graph
        a.dayof = {"melee": "skip", "car": "ship"}
        self.assertNotIn(mr.GRAPH_REL, {e.rel for e in mr.build_plan(a, [])})

    def test_texts_unchanged_without_graph_and_licence_with_it(self):
        self.assertEqual(mr.gen_readme(True), mr.gen_readme(True, False))
        errs = []
        self.assertEqual(mr.gen_notices(errs), mr.gen_notices(errs, False))
        off_r, off_n = mr.gen_readme(True).decode(), mr.gen_notices(errs).decode()
        on_r, on_n = mr.gen_readme(True, True).decode(), mr.gen_notices(errs, True).decode()
        self.assertNotIn("player_melee", off_r + off_n)
        self.assertIn("player_melee.animgraph", on_r)
        self.assertIn("conflicts with other mods", on_r)
        self.assertIn("modified copy of the game's own", on_n)
        self.assertIn("CD PROJEKT RED", on_n)
        self.assertIn("no third-party content", on_n)
        # the graph note sits inside the game-derived section, before the open-source section
        self.assertLess(on_n.index("player_melee.animgraph"), on_n.index("== Open-source code"))
        self.assertGreater(on_n.index("player_melee.animgraph"), on_n.index("== Content derived from Cyberpunk 2077 =="))
        # melee skipped: no graph text even if graph were requested
        self.assertNotIn("player_melee", mr.gen_readme(False, True).decode())
        F = mr.Findings()
        mr.scan_text(mr.README_REL, on_r, F, True)
        mr.scan_text(mr.NOTICES_REL, on_n, F, True)
        self.assertEqual([i for i in F.items if i["level"] in ("FAIL", "WARN")], [])


class ShipLists(unittest.TestCase):
    def all_paths(self):
        p = [r for r, _, _ in mr.DATA] + [r for r, _, _ in mr.DATA_NUE] + [r for r, _ in mr.DATA_PENDING]
        p += [r for r, _, _ in mr.TRANSFORMS] + [r for r, _, _ in mr.CODE] + [r for r, _, _ in mr.CODE_NUE]
        p += [r for _, r, _, _ in mr.DAYOF_DATA] + [r for _, r, _, _ in mr.DAYOF_CODE]
        return [x.lower() for x in p]

    def test_excluded_and_included(self):
        for p in self.all_paths():
            self.assertNotIn("_probe", p)
            self.assertNotIn("utpp_testcmd", p)
            self.assertNotIn("utpp_bisect", p)
            self.assertNotIn("ragdoll_poc", p)
            self.assertNotIn("ragdoll_graph", p)
            self.assertFalse(p.endswith("ragdoll.archive") or p.endswith("ragdoll.archive.xl"), p)
        # the tested behaviour is "early pose experiment off": no early pose file of any name ships
        for p in self.all_paths():
            self.assertNotIn("early-pose", p)
            self.assertNotIn("early_pose", p)


class FindSource(unittest.TestCase):
    def setUp(self):
        mr._INDEX_CACHE.clear()  # the folder index is built once per run; these tests change the folders between calls
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.game = self.root / "game"
        self.data = self.root / "Data"
        (self.game / "red4ext" / "plugins" / "UTPP" / "older").mkdir(parents=True)
        self.data.mkdir()

    def tearDown(self):
        self.tmp.cleanup()

    @staticmethod
    def pin(content):
        import hashlib
        return len(content), hashlib.sha256(content).hexdigest()

    def test_data_folder_first_then_the_package_path_in_the_game(self):
        rel = mr.P_DATA + "a.bin"
        (self.game / mr.native(rel)).parent.mkdir(parents=True)
        (self.game / mr.native(rel)).write_bytes(b"tested")
        size, sha = self.pin(b"tested")
        self.assertEqual(mr.find_source(self.game, rel, size, sha, [self.data]), self.game / mr.native(rel))
        mr._INDEX_CACHE.clear()
        (self.data / "other_name.bin").write_bytes(b"tested")
        self.assertEqual(mr.find_source(self.game, rel, size, sha, [self.data]), self.data / "other_name.bin")

    def test_found_by_content_under_any_name(self):
        rel = mr.P_DATA + "new_name.bin"
        (self.game / "red4ext" / "plugins" / "UTPP" / "older" / "old_name.bin").write_bytes(b"tested")
        size, sha = self.pin(b"tested")
        self.assertEqual(mr.find_source(self.game, rel, size, sha, [self.data]).name, "old_name.bin")
        (self.game / "red4ext" / "plugins" / "UTPP" / "older" / "old_name.bin").unlink()
        (self.data / "elsewhere.bin").write_bytes(b"tested")
        mr._INDEX_CACHE.clear()
        self.assertEqual(mr.find_source(self.game, rel, size, sha, [self.data]).name, "elsewhere.bin")

    def test_same_size_other_content_is_never_used(self):
        rel = mr.P_DATA + "x.bin"
        (self.data / "x.bin").write_bytes(b"other!")
        (self.game / mr.native(rel)).parent.mkdir(parents=True)
        (self.game / mr.native(rel)).write_bytes(b"differ")
        size, sha = self.pin(b"tested")
        self.assertEqual(len(b"other!"), size)
        self.assertIsNone(mr.find_source(self.game, rel, size, sha, [self.data]))

    def test_missing_source_is_a_plan_error_and_package_path_is_reported(self):
        import argparse
        a = argparse.Namespace(game=str(self.game), code_tree=str(self.root / "tree"), dll=str(self.root / "none.dll"),
                               dry_run=False, nue=True, dayof={"melee": "skip", "car": "skip"}, data_roots=[str(self.data)])
        errs = []
        ents = mr.build_plan(a, errs)
        self.assertTrue([e for e in errs if "no file with the pinned size and SHA-256" in e])
        self.assertIn(mr.P_DATA + "utpp_base.utppmm", {e.rel for e in ents})


class CodePins(unittest.TestCase):
    def test_code_pins_equal_the_code_tree(self):
        import hashlib
        tree = mr.ROOT / "Source" / "plugin" / "UTPP"
        if not tree.is_dir():
            self.skipTest("no code tree")
        want = {sub: sha for _, _, sub, sha in mr.DAYOF_CODE}
        want.update({"archive/utpp_camera.archive.xl": mr.CODE_EXPECT[mr.P_ARC + "utpp_camera.archive.xl"]})
        for sub, sha in want.items():
            self.assertEqual(hashlib.sha256((tree / sub).read_bytes()).hexdigest(), sha, sub)

    def test_every_code_file_exists(self):
        tree = mr.ROOT / "Source" / "plugin" / "UTPP"
        if not tree.is_dir():
            self.skipTest("no code tree")
        for _, sub, _ in mr.CODE + mr.CODE_NUE:
            self.assertTrue((tree / sub).is_file(), sub)
        for _, _, sub, _ in mr.DAYOF_CODE:
            self.assertTrue((tree / sub).is_file(), sub)


class ReleaseTexts(unittest.TestCase):
    def test_menu_and_report_sections_in_the_readme(self):
        r = mr.gen_readme().decode()
        for s in ("Animation pack: Mod Settings > Ultimate Third Person > General > Animation pack", "Game Animations",
                  "How to report a problem", "Diagnostic logs", "utpp-report-(date).txt", "data\\recoil"):
            self.assertIn(s, r)
        self.assertNotIn("v2\\recoil", r)
        self.assertNotIn("contain your game install path", r)

    APPROVED = ("Animation data. The motion-matching databases, overlays, traversal and lean tables shipped with UTPP are original "
                "work. They are based on licensed GASPALS 1.2 content by Polygon Hive, then heavily modified, extended, tuned and "
                "blended with the game's own animations for Cyberpunk 2077. The GASPALS content was used under a purchased license.")

    def test_user_approved_animation_data_wording(self):
        r = mr.gen_readme().decode().replace("\r\n", "\n")
        n = mr.gen_notices([]).decode().replace("\r\n", "\n")
        self.assertIn(self.APPROVED, r)
        self.assertIn(self.APPROVED, n)
        self.assertNotIn("used with permission", r + n)

    def test_older_versions_sentence_follows_the_tested_frameworks(self):
        r = mr.gen_readme().decode().replace("\r\n", "\n")
        self.assertIn("(versions installed in the test game, 2.50).\nOlder versions will not work with game version 2.5.\n", r)

    def test_legacy_names_are_flagged_outside_the_licence_texts(self):
        F = mr.Findings()
        mr.scan_text(mr.P_REDS + "x.reds", "let a = utpp_v2; // gasp m360 Fortnite base_r13c", F, True)
        self.assertGreaterEqual(len([i for i in F.items if i["check"] == "legacy-name"]), 2)
        F = mr.Findings()
        mr.scan_text(mr.README_REL, "GASPALS 1.2 licence", F, True)
        self.assertEqual([i for i in F.items if i["level"] == "FAIL"], [])
        F = mr.Findings()  # json tables hold game / source clip names; a real project path is still a dev path
        mr.scan_text(mr.P_DATA + "t.json", '"asset": "AM_M_Neutral_Hurdle_F_V2_Lfoot"', F, True)
        self.assertEqual([i for i in F.items if i["level"] == "FAIL"], [])
        mr.scan_text(mr.P_REDS + "p.reds", 'x = "D:\\\\Ultimate Third Person Project\\\\Data"', F, True)
        self.assertEqual([i["check"] for i in F.items if i["level"] == "FAIL"], ["dev-path"])
        F = mr.Findings()  # the display name inside a redscript string ends in an escaped quote, not a path separator
        mr.scan_text(mr.P_REDS + "q.reds", r'"\"Ultimate Third Person Project\", \"Game Animations\""', F, True)
        self.assertEqual([i for i in F.items if i["level"] == "FAIL"], [])
        F = mr.Findings()  # ordinary game paths are not version suffixes
        mr.scan_text(mr.P_REDS + "y.reds", 'r6\\scripts\\UTPP; x64; player_wa_fpp_ep1.ent; w_melee_fists', F, True)
        self.assertEqual([i for i in F.items if i["level"] == "FAIL"], [])

    def test_required_checks(self):
        F = mr.Findings()
        mr.check_required({}, F, set())
        self.assertGreaterEqual(len([i for i in F.items if i["check"] == "required"]), 4)
        texts = {
            mr.P_REDS + "utpp_modsettings.reds": '@runtimeProperty("ModSettings.displayName", "Animation pack") '
                                                  '@runtimeProperty("ModSettings.displayName", "Diagnostic logs")',
            mr.P_CET + "utpp_core/init.lua": 'M.Register({ key = M.PACK_KEY, section = "general", type = "choice", default = "utpp",\n'
                'choices = { { "utpp", "Ultimate Third Person Project" }, { "game", "Game Animations" } },\n label = "Animation pack" })\n'
                'M.Register({ key = M.DIAG_KEY, section = "general", type = "choice", default = "off",\n'
                'choices = { { "timed", "On for 5 minutes" }, { "until", "On until switched off" } },\n label = "Diagnostic logs" })',
            mr.README_REL: mr.gen_readme().decode(),
            mr.NOTICES_REL: mr.gen_notices([]).decode(),
            mr.LICENSE_REL: open(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'LICENSE'), encoding='utf-8').read(),
        }
        F = mr.Findings()
        mr.check_required(texts, F, set())
        self.assertEqual([i for i in F.items if i["level"] in ("FAIL", "WARN")], [], F.items)
        F = mr.Findings()
        mr.check_required(texts, F, {mr.P_DLL + "early-pose-experiment.txt"})
        self.assertTrue([i for i in F.items if i["check"] == "forbidden-file"])
        texts[mr.P_CET + "utpp_core/init.lua"] = texts[mr.P_CET + "utpp_core/init.lua"].replace('default = "off"', 'default = "timed"')
        F = mr.Findings()
        mr.check_required(texts, F, set())
        self.assertTrue([i for i in F.items if i["check"] == "default-off"])

    def test_test_command_module_is_never_shipped(self):
        F = mr.Findings()
        mr.check_forbidden_names([mr.P_CET + "utpp_testcmd/init.lua"], True, F)
        self.assertEqual([i["check"] for i in F.items if i["level"] == "FAIL"], ["forbidden-file"])
        F = mr.Findings()
        mr.scan_text(mr.P_CET + "utpp_core/init.lua", 'local t = GetMod("utpp_testcmd")', F, True)
        self.assertTrue([i for i in F.items if i["check"] == "probe-name" and i["level"] == "FAIL"])

    def test_test_host_plugin_is_never_shipped(self):
        for rel in ("red4ext/plugins/UTPPTestHost/UTPPTestHost.dll", mr.P_DLL + "UTPPTestHost.dll"):
            F = mr.Findings()
            mr.check_forbidden_names([rel], True, F)
            self.assertEqual([i["check"] for i in F.items if i["level"] == "FAIL"], ["forbidden-file"], rel)
        for line in ('Game.UTPPTestBackgroundInput(true)', 'local s = Game.UTPPTestBackgroundInputStatus()'):
            F = mr.Findings()
            mr.scan_text(mr.P_CET + "utpp_core/init.lua", line, F, True)
            self.assertTrue([i for i in F.items if i["check"] == "probe-name" and i["level"] == "FAIL"], line)
        for p in ShipLists().all_paths():
            self.assertNotIn("utpptesthost", p)
        F = mr.Findings()   # the release plugin itself stays allowed
        mr.check_forbidden_names([mr.P_DLL + "UTPP.dll"], True, F)
        self.assertEqual([i for i in F.items if i["level"] == "FAIL"], [])

    def test_stage_dir_must_be_inside_the_project_stages(self):
        import contextlib
        import io
        out = mr.ROOT / "Build" / "work" / "release-tool-20261109" / "test-stage-dir-out"
        for stage in (str(mr.ROOT), str(mr.ROOT / "Build"), str(mr.ROOT / "Build" / "stages" / "a" / "b"), r"X:\elsewhere"):
            old = sys.argv
            sys.argv = ["make_release.py", "--code-tree", str(mr.ROOT / "Source"), "--dll", "x.dll", "--out", str(out),
                        "--stage-dir", stage, "--day-of-build", "melee=ship,car=ship"]
            buf = io.StringIO()
            try:
                with contextlib.redirect_stdout(buf):
                    rc = mr.main()
            finally:
                sys.argv = old
            self.assertEqual(rc, 2, stage)
            self.assertIn("--stage-dir", buf.getvalue())
            self.assertFalse(out.exists())


class ChooserTransform(unittest.TestCase):
    PAIRS = [('"licence": "old header text",', '"licence": "%s",' % mr.LICENCE_NEUTRAL),
             ('"generator": "old generator text",', '"generator": "UTPP data tools",')]

    def setUp(self):
        self._saved = (mr.CHOOSER_REPL, mr.CHOOSER_PRIOR)
        mr.CHOOSER_REPL, mr.CHOOSER_PRIOR = list(self.PAIRS), '"licence": "earlier neutral text",'

    def tearDown(self):
        mr.CHOOSER_REPL, mr.CHOOSER_PRIOR = self._saved

    def doc(self, pairs):
        return ("{\n" + "\n".join(" " + s for s in pairs) + '\n "format": 1\n}').encode("utf-8")

    def test_old_text_replaced(self):
        out = mr.xform_chooser(self.doc([o for o, _ in mr.CHOOSER_REPL])).decode("utf-8")
        for old, new in mr.CHOOSER_REPL:
            self.assertEqual(out.count(new), 1)
            self.assertNotIn(old, out)

    def test_already_neutral_is_kept(self):
        neutral = self.doc([n for _, n in mr.CHOOSER_REPL])
        self.assertEqual(mr.xform_chooser(neutral), neutral)
        # the database label follows the shipped file name (no work names of bakes)
        lab = mr.xform_chooser(neutral.replace(b'"format"', b'"database_file": "work9.utppmm", "format"', 1), 'x/utpp_motion_woman.chooser.json')
        self.assertIn(b'"database_file": "utpp_motion_woman.utppmm"', lab)
        self.assertNotIn(b'work9', lab)

    def test_unknown_text_fails(self):
        with self.assertRaises(ValueError):
            mr.xform_chooser(self.doc(['"licence": "something else",']))


class SkinHelperAudit(unittest.TestCase):
    """Skin helper data: five files under skin_helpers, mode.txt, hashes = the DLL's approved table, DLL carries the hashes."""

    @staticmethod
    def h(seed):
        import hashlib
        return hashlib.sha256(seed.encode()).hexdigest()

    def setUp(self):
        self.hash = {(b, f): self.h(b + f) for b in mr.SKIN_BODIES for f in mr.SKIN_BUNDLE_FILES}
        self.table = {b: (self.hash[(b, "program.ir")], self.hash[(b, "binding.dat")]) for b in mr.SKIN_BODIES}
        self.shipped = {mr.P_SKIN + b + "/" + f: self.hash[(b, f)] for b, f in self.hash}
        self.shipped[mr.SKIN_MODE_REL] = self.h("mode")
        self.shipped[mr.DLL_REL] = self.h("dll")
        self.dll = b"MZ..." + b"".join(x.encode() + b"\0" for x in self.hash.values())

    def audit(self, shipped=None, mode=mr.SKIN_MODE_BYTES, table=None, err=None, dll=None):
        F = mr.Findings()
        mr.check_skin_helpers(self.shipped if shipped is None else shipped, mode, self.table if table is None else table, err,
                              self.dll if dll is None else dll, F)
        return F

    def fails(self, F, check=None):
        return [i for i in F.items if i["level"] == "FAIL" and (check is None or i["check"] == check)]

    def header(self, *entries):
        return "namespace x {\ninline constexpr std::array<Approved,%d> kApproved{{\n%s}};\n}\n" % (len(entries), "".join(entries))

    @staticmethod
    def entry(body, prog, bind, trailer="40, 0, 619, 113, 0x9623ba2b1bcfdbbdull"):
        return '    {kBody%s,\n     "%s",\n     "%s",\n     %s},\n' % (body, prog, bind, trailer)

    def test_good_package_passes(self):
        self.assertEqual(self.fails(self.audit()), [])
        self.assertEqual(len(mr.SKIN_RELS), 5)
        self.assertEqual(mr.SKIN_MODE_BYTES, b"APPLY\n")

    def test_missing_and_extra_files(self):
        s = dict(self.shipped)
        del s[mr.P_SKIN + "woman/binding.dat"]
        F = self.audit(s)
        self.assertEqual([i["file"] for i in self.fails(F, "skin-helpers-set")], [mr.P_SKIN + "woman/binding.dat"])
        s = dict(self.shipped)
        del s[mr.SKIN_MODE_REL]
        self.assertEqual(len(self.fails(self.audit(s, mode=None), "skin-helpers-set")), 1)
        for extra in (mr.P_SKIN + "notes.txt", mr.P_SKIN + "man/expected.txt", mr.P_SKIN + "child/program.ir", mr.P_SKIN.upper() + "MAN/EXTRA.IR"):
            s = dict(self.shipped)
            s[extra] = self.h(extra)
            self.assertEqual([i["file"] for i in self.fails(self.audit(s), "skin-helpers-set")], [extra], extra)

    def test_mode_must_be_exact(self):
        for bad in (b"APPLY\r\n", b"APPLY", b"OFF\n", b"\xef\xbb\xbfAPPLY\n", b"APPLY\n\n", b"", b"apply\n"):
            F = self.audit(mode=bad)
            self.assertEqual(len(self.fails(F, "skin-helpers-mode")), 1, bad)

    def test_wrong_hash_and_body_swap(self):
        s = dict(self.shipped)
        s[mr.P_SKIN + "man/program.ir"] = self.h("other")
        F = self.audit(s)
        self.assertEqual([i["file"] for i in self.fails(F, "skin-helpers-hash")], [mr.P_SKIN + "man/program.ir"])
        self.assertIn(self.hash[("man", "program.ir")], self.fails(F, "skin-helpers-hash")[0]["detail"])
        # the man bundle in the woman folder (and vice versa) is approved for the other body only: not "any approved hash"
        s = dict(self.shipped)
        for f in mr.SKIN_BUNDLE_FILES:
            s[mr.P_SKIN + "man/" + f], s[mr.P_SKIN + "woman/" + f] = self.shipped[mr.P_SKIN + "woman/" + f], self.shipped[mr.P_SKIN + "man/" + f]
        self.assertEqual(len(self.fails(self.audit(s), "skin-helpers-hash")), 4)

    def test_dll_must_contain_every_approved_hash(self):
        for key, hx in self.hash.items():
            F = self.audit(dll=self.dll.replace(hx.encode(), b"0" * 64))
            self.assertEqual(len(self.fails(F, "skin-helpers-dll")), 1, key)
            self.assertIn(hx, self.fails(F, "skin-helpers-dll")[0]["detail"])
        F = self.audit(dll=b"MZ old dll")
        self.assertEqual(len(self.fails(F, "skin-helpers-dll")), 4)
        # upper-case spelling is not the compiled literal
        self.assertEqual(len(self.fails(self.audit(dll=self.dll.upper()), "skin-helpers-dll")), 4)

    def test_unreadable_table_fails_clearly(self):
        F = self.audit(table={}, err="table kApproved not found")
        f = self.fails(F, "skin-helpers-hash")
        self.assertEqual(len(f), 1)
        self.assertIn("cannot read the approved hashes", f[0]["detail"])
        self.assertIn("kApproved", f[0]["detail"])

    def test_header_parser(self):
        p, b = self.table["man"], self.table["woman"]
        good = self.header(self.entry("Man", *p), self.entry("Woman", *b))
        self.assertEqual(mr.parse_approved_table(good), (self.table, None))
        # comments inside the block (even with hashes) and order do not matter
        noisy = self.header("    // {kBodyMan, \"%s\", \"%s\"},\n" % (self.h("c1"), self.h("c2")), self.entry("Woman", *b), self.entry("Man", *p))
        self.assertEqual(mr.parse_approved_table(noisy), (self.table, None))
        bad = {
            "empty": "",
            "no table": "int x = 1;",
            "empty table": self.header(),
            "missing woman": self.header(self.entry("Man", *p)),
            "duplicate man": self.header(self.entry("Man", *p), self.entry("Man", *b), self.entry("Woman", *b)),
            "unknown body": self.header(self.entry("Man", *p), self.entry("Woman", *b), self.entry("Child", *b)),
            "short hash": self.header(self.entry("Man", p[0][:60], p[1]), self.entry("Woman", *b)),
            "upper-case hash": self.header(self.entry("Man", p[0].upper(), p[1]), self.entry("Woman", *b)),
            "garbled": "constexpr x kApproved{{ {kBodyMan, 1, 2}, {kBodyWoman} }};",
        }
        for name, text in bad.items():
            table, err = mr.parse_approved_table(text)
            self.assertTrue(err, name)
            self.assertEqual(table, {}, name)

    def test_real_header_parses_into_both_bodies(self):
        hpp = mr.ROOT / "Source" / "plugin" / "UTPP" / mr.native(mr.SKIN_HEADER_SUB)
        if not hpp.is_file():
            self.skipTest("no code tree")
        table, err = mr.load_approved_table(mr.ROOT / "Source")
        self.assertIsNone(err)
        self.assertEqual(sorted(table), sorted(mr.SKIN_BODIES))
        for pair in table.values():
            self.assertTrue(all(len(x) == 64 for x in pair))
        table, err = mr.load_approved_table(mr.ROOT / "Source" / "nope")
        self.assertTrue(err)

    def test_plan_ships_exactly_the_five_files(self):
        import argparse
        a = argparse.Namespace(game=r"X:\nogame", code_tree=r"X:\notree", dll=r"X:\none.dll", dry_run=False, nue=True,
                               dayof={"melee": "skip", "car": "skip"}, data_roots=[r"X:\nodata"], skin_data_root=r"X:\noskin")
        ents = mr.build_plan(a, [])
        skin = {e.rel: e for e in ents if e.rel.startswith(mr.P_SKIN)}
        self.assertEqual(set(skin), set(mr.SKIN_RELS))
        self.assertEqual(skin[mr.SKIN_MODE_REL].data, b"APPLY\n")
        self.assertEqual(skin[mr.SKIN_MODE_REL].kind, "gen")
        for rel, e in skin.items():
            if rel != mr.SKIN_MODE_REL:
                self.assertEqual(e.kind, "skin")
                self.assertIsNone(e.expect_sha)   # not pinned here: the DLL's table is the pin
        self.assertEqual(skin[mr.P_SKIN + "woman/program.ir"].src.parent.name, "body_helper_woman")
        self.assertEqual(skin[mr.P_SKIN + "man/binding.dat"].src.parent.name, "body_helper_man")
        # the default source folder is the project's Build\test-data
        a.skin_data_root = None
        ents = mr.build_plan(a, [])
        self.assertEqual(next(e for e in ents if e.rel == mr.P_SKIN + "man/program.ir").src,
                         mr.ROOT / "Build" / "test-data" / "body_helper_man" / "program.ir")
        del a.skin_data_root   # tests of other classes build the plan without the option
        mr.build_plan(a, [])

    def test_old_folder_name_is_refused_everywhere(self):
        old = mr.P_DLL + mr.OLD_SKIN_FOLDER + "/man/program.ir"
        for rel in (old, old.upper(), mr.P_DLL + "x/" + mr.OLD_SKIN_FOLDER + ".txt"):
            F = mr.Findings()
            mr.check_forbidden_names([rel], True, F)
            self.assertEqual([i["check"] for i in F.items if i["level"] == "FAIL"], ["forbidden-file"], rel)
        F = mr.Findings()
        mr.check_forbidden_names(mr.SKIN_RELS, True, F)
        self.assertEqual(F.items, [])   # the new layout passes the name rules
        for text in ("local p = 'helper_candidate/mode.txt'", "-- was helper_candidate", "// helper_candidate"):
            F = mr.Findings()
            mr.scan_text(mr.P_REDS + "x.reds", text, F, True)
            self.assertEqual([i["check"] for i in F.items if i["level"] == "FAIL"], ["legacy-name"], text)   # comments fail too
        F = mr.Findings()
        mr.scan_text(mr.P_REDS + "x.reds", "let dir = skin_helpers;", F, True)
        self.assertEqual([i for i in F.items if i["level"] in ("FAIL", "WARN")], [])
        # binary scan: the data files and the DLL are checked for the old name in ASCII and UTF-16
        with tempfile.TemporaryDirectory() as d:
            for enc in ("ascii", "utf-16le"):
                p = Path(d) / ("f_" + enc)
                p.write_bytes(b"..." + mr.OLD_SKIN_FOLDER.encode(enc) + b"...")
                hits = mr.pass_file(p, True, utf16=True)[2]
                self.assertEqual({h[0] for h in hits}, {"legacy-name"}, enc)
            F = mr.Findings()
            p = Path(d) / "UTPP.dll"
            p.write_bytes(b"MZ" + mr.OLD_SKIN_FOLDER.encode() + b"\0")
            mr.scan_dll(p, F, mr.DLL_REL, False)
            self.assertTrue([i for i in F.items if i["check"] == "legacy-name"])

    def test_texts_mention_the_data_and_stay_truthful(self):
        r = mr.gen_readme().decode()
        n = mr.gen_notices([]).decode()
        self.assertIn("skin_helpers", r)
        self.assertIn("mode.txt", r)
        self.assertIn("skin helper data", n)
        self.assertLess(n.index("skin helper data"), n.index("== Open-source code"))
        self.assertGreater(n.index("skin helper data"), n.index("== Content derived from Cyberpunk 2077 =="))
        self.assertNotIn(mr.OLD_SKIN_FOLDER, r + n)
        self.assertNotIn("{", r)


if __name__ == "__main__":
    unittest.main(verbosity=2)
