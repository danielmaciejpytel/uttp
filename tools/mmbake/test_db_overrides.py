#!/usr/bin/env python3
"""Tests for mmbake_motion.apply_db_overrides (--db-overrides): bias, added entries, DisableReselection, refusals."""
from __future__ import annotations

import copy
import json
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import mmbake_motion as mm  # noqa: E402

PATH = "/Pack/Animations/Sprint"


def entry(name, dr=False):
    return {"index": 0, "asset": f"{PATH}/{name}.{name}", "asset_class": "AnimSequence", "play_length_s": 2.0,
            "enabled": True, "disable_reselection": dr, "mirror_option": "UnmirroredOnly", "sampling_range": [0.0, 0.0],
            "looping": False, "estimated_poses": 60, "sampled_seconds": 2.0}


def dbs():
    return {"Starts": {"continuing_pose_cost_bias": -0.05, "entries": [entry("Start_F_L"), entry("Start_F_R")]},
            "Stops": {"continuing_pose_cost_bias": -0.2, "entries": [entry("Stop_F_L", True)]}}


class DbOverridesTest(unittest.TestCase):
    def test_empty_spec_changes_nothing(self):
        d = dbs()
        ref = copy.deepcopy(d)
        self.assertEqual(mm.apply_db_overrides(d, {}), [])
        self.assertEqual(d, ref)

    def test_bias_entries_and_reselection(self):
        d = dbs()
        mm.apply_db_overrides(d, {"databases": {
            "Stops": {"continuing_pose_cost_bias": -0.3, "add_entries": [{"clip": "Stop_FL_L", "disable_reselection": True}]},
            "Starts": {"add_entries": [{"clip": "Start_FL_L"}], "disable_reselection_all": True}}})
        self.assertEqual(d["Stops"]["continuing_pose_cost_bias"], -0.3)
        self.assertEqual(d["Starts"]["continuing_pose_cost_bias"], -0.05)
        new = d["Stops"]["entries"][-1]
        self.assertEqual(mm.short_name(new["asset"]), "Stop_FL_L")
        self.assertEqual(new["index"], 1)
        self.assertTrue(new["disable_reselection"])
        self.assertEqual(new["sampling_range"], [0.0, 0.0])
        self.assertNotIn("play_length_s", new)
        self.assertEqual([mm.short_name(e["asset"]) for e in d["Starts"]["entries"]], ["Start_F_L", "Start_F_R", "Start_FL_L"])
        self.assertTrue(all(e["disable_reselection"] for e in d["Starts"]["entries"]))

    def test_refusals(self):
        for spec in ({"databases": {"Nope": {"continuing_pose_cost_bias": 0.0}}},
                     {"databases": {"Stops": {"base_cost_bias": 0.0}}},
                     {"databases": {"Stops": {"add_entries": [{"clip": "Stop_F_L"}]}}}):
            with self.assertRaises(SystemExit):
                mm.apply_db_overrides(dbs(), spec)

    def test_shipped_specs_are_valid(self):
        for f in ("database_overrides.json", "database_overrides_sprint.json"):
            with open(os.path.join(HERE, "config", f), encoding="utf-8") as fh:
                spec = json.load(fh)
            for name, ov in spec["databases"].items():
                self.assertFalse(set(ov) - mm.DB_OVERRIDE_KEYS, (f, name))


if __name__ == "__main__":
    unittest.main()
