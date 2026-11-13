"""Offline tests for car_cases_nav.py (no game, no input).

  python Source\\tools\\gametest\\test_car_cases_nav.py -v
"""
from __future__ import annotations

import json
import os
import sys
import unittest
from datetime import datetime, timedelta

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import car_cases_nav as nav  # noqa: E402
import gt_scenario as gs  # noqa: E402

SCEN = os.path.join(HERE, 'scenarios_car_cases.json')


def pos(px, py, cf=(0.0, 1.0), car=(0.0, 0.0), right=(1.0, 0.0), mounted=False):
    """POS dict as gt_night.parse_pos returns it: car at `car`, facing +y (right = +x)."""
    return {'p': (px, py, 0.0), 'cf': cf, 'v': (car[0], car[1], 0.0), 'vr': right, 'src': 'tag', 'weapon': False,
            'mounted': mounted, 'age': 0.2}


class Presets(unittest.TestCase):
    def test_names_both_forms_and_exact(self):
        for rec in ('Vehicle.v_standard2_archer_hella_player', 'Vehicle.v_standard2_archer_hella'):
            self.assertEqual(nav.preset(rec)['key'], 'v_standard2_archer_hella')
        self.assertEqual(nav.preset('Vehicle.v_sport1_quadra_turbo_r')['key'], 'v_sport1_quadra_turbo')
        self.assertEqual(nav.preset('Vehicle.v_sport1_quadra_turbo_player')['key'], 'v_sport1_quadra_turbo')
        self.assertTrue(nav.preset('Vehicle.v_utility4_kaukaz_bratsk_player')['known'])
        self.assertFalse(nav.preset('Vehicle.v_utility4_kaukaz_bratsk_extended')['known'])   # no prefix match

    def test_spawn_corrections_match_the_formula_or_the_verified_run(self):
        h = nav.preset('Vehicle.v_standard2_archer_hella_player')
        self.assertEqual((h['fwd'], h['away']), (0.0, 0.0))
        self.assertEqual((h['fwd_formula'], h['away_formula']), (0.0, 0.0))
        b = nav.preset('Vehicle.v_utility4_kaukaz_bratsk')
        self.assertAlmostEqual(b['fwd'], b['fwd_formula'], delta=0.01)       # -3.49
        self.assertAlmostEqual(b['away'], b['away_formula'], delta=0.01)     # 0.56
        t = nav.preset('Vehicle.v_sport1_quadra_turbo_r')
        self.assertAlmostEqual(t['fwd'], t['fwd_formula'], delta=0.01)       # 0.55


class Decide(unittest.TestCase):
    hella = nav.preset('Vehicle.v_standard2_archer_hella_player')

    def test_hella_parallel_start_is_inside_the_window(self):
        # the working parallel start: V 2.6 m to the car's LEFT (world -x), lon 0 (car at V's right)
        r = nav.decide(pos(-2.6, 0.0), self.hella, 'driver')
        self.assertEqual(r['state'], 'ok')
        self.assertAlmostEqual(r['lat'], 2.6, places=2)

    def test_behind_start_is_far_and_walks_forward_right(self):
        # 4 m behind the seat line, 1.9 m left of the car
        r = nav.decide(pos(-1.9, -4.14), self.hella, 'driver')
        self.assertEqual(r['state'], 'move')
        sx, sy = r['stick']
        self.assertGreater(sy, 0.3)           # camera looks +y: the door is ahead
        self.assertGreaterEqual(sx, 0.0)      # and slightly to the car (right)
        self.assertLessEqual(r['dur'], nav.CHUNK_MAX_S)
        ls = r['step']['ls']
        self.assertEqual(ls[-1][1:], [0.0, 0.0])                 # the stick returns to neutral

    def test_camera_turned_90_deg_rotates_the_stick(self):
        r = nav.decide(pos(-1.9, -4.14, cf=(1.0, 0.0)), self.hella, 'driver')   # camera looks +x (the car's right)
        sx, sy = r['stick']
        self.assertLess(sy, 0.2)              # forward of the camera is no longer the way
        self.assertLess(sx, -0.3)             # the door is on the camera's LEFT (+y is left of +x)

    def test_door_case_pose_needs_its_lon_bias(self):
        # 'door' case: car turned 90 deg, V 0.5 m ahead of the parallel pose; mounted on all five cars of 2026-11-01
        self.assertEqual(nav.decide(pos(-2.6, 0.5), self.hella, 'driver')['state'], 'move')
        r = nav.decide(pos(-2.6, 0.5), self.hella, 'driver', lon_bias=0.5)
        self.assertEqual(r['state'], 'ok')
        self.assertEqual(r['lon_target'], 0.5)

    def test_bratsk_parallel_start_is_walked_in(self):
        b = nav.preset('Vehicle.v_utility4_kaukaz_bratsk')
        far = nav.decide(pos(-3.1, b['lon_target']), b, 'driver')          # V 3.1 m from the truck's centre line (case start)
        self.assertEqual(far['state'], 'move')
        near = nav.decide(pos(-2.35, b['lon_target']), b, 'driver')        # ~0.8 m from the cab door (passenger case X frame)
        self.assertEqual(near['state'], 'ok')
        # review 2026-11-01: V just outside lat_max (2.65) is never 'ok' and the walk target stays inside the window
        for lat in (2.7, 2.8, 2.9, 3.1):
            self.assertEqual(nav.decide(pos(-lat, b['lon_target']), b, 'driver')['state'], 'move', lat)
        self.assertEqual(nav.decide(pos(-(b['lat_max'] - 0.05), b['lon_target']), b, 'driver')['state'], 'ok')
        # the small cars keep the wide window (their parallel start at lat 2.6 mounted)
        self.assertEqual(nav.decide(pos(-2.6, 0.0), nav.preset('Vehicle.v_standard2_archer_hella_player'), 'driver')['state'], 'ok')

    def test_generated_step_loads_in_the_harness_parser(self):
        r = nav.decide(pos(-1.9, -4.14), nav.preset('Vehicle.v_standard2_archer_hella_player'), 'driver')
        raw = {'defaults': {'rate_hz': 60, 'interp': 'smooth'}, 'scenarios': {'car_nav': {'description': 'x', 'steps': [r['step']]}}}
        tmp = os.path.join(os.environ.get('TEMP', HERE), 'car_cases_nav_step_test.json')
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump(raw, f)
        try:
            sc = gs.load_scenarios(tmp)['car_nav']
            self.assertAlmostEqual(sc.duration, r['dur'] + 0.7, places=1)
        finally:
            os.remove(tmp)

    def test_passenger_side_mirrors_the_driver_side(self):
        r = nav.decide(pos(2.6, 0.0), self.hella, 'passenger')
        self.assertEqual(r['state'], 'ok')
        self.assertAlmostEqual(r['lat_door_side'], 2.6, places=2)
        r = nav.decide(pos(2.6, 0.0), self.hella, 'driver')                       # wrong side for a driver case
        self.assertNotEqual(r['state'], 'ok')

    def test_lon_tolerance_from_the_turbo_runs(self):
        t = nav.preset('Vehicle.v_sport1_quadra_turbo_r')          # seat -0.41 -> lon target -0.55
        self.assertEqual(nav.decide(pos(-2.6, -0.55), t, 'driver')['state'], 'ok')       # 20261101b (mounted)
        self.assertEqual(nav.decide(pos(-2.6, 0.0), t, 'driver')['state'], 'move')       # 20261101 (never mounted)

    def test_mounted_and_missing_car(self):
        self.assertEqual(nav.decide(pos(0, 0, mounted=True), self.hella, 'driver')['state'], 'mounted')
        p = pos(0, 0)
        p['v'] = None
        self.assertEqual(nav.decide(p, self.hella, 'driver')['state'], 'nocar')

    def test_retry_nudges_forward_when_already_at_the_door(self):
        p = self.hella
        at = pos(-(p['door_x'] + nav.LAT_TARGET), p['lon_target'])
        r = nav.decide(at, p, 'driver', 'retry')
        self.assertEqual(r['state'], 'move')
        self.assertGreater(r['stick'][1], 0.2)          # forward along the car (camera +y)

    def test_log_reader_takes_the_newest_line_after_the_marker(self):
        ts = datetime(2026, 11, 1, 15, 0, 0)

        def line(t, x):
            return ('[%s] [info    ] [1] [UTPP] UTPP CET [night] POS p=%s,0.00,0.00 cf=0.000,1.000 v=0.00,0.00,0.00,1.000,0.000'
                    ' src=tag w=0 m=0\n' % (t.strftime('%Y-%m-%d %H:%M:%S.%f')[:23], x))
        text = line(ts, '-9.00') + '[car] SPAWN case=a ok=true\n' + line(ts + timedelta(seconds=1), '-1.00') + \
            line(ts + timedelta(seconds=2), '-2.00')
        got, why = nav._pos_from_log(text, '[car] SPAWN case=a ', 8.0, now=ts + timedelta(seconds=3))
        self.assertIsNotNone(got, why)
        self.assertAlmostEqual(got['p'][0], -2.0)
        got, why = nav._pos_from_log(text, '[car] SPAWN case=a ', 8.0, now=ts + timedelta(seconds=30))
        self.assertIsNone(got)
        got, why = nav._pos_from_log(text, '[car] SPAWN case=zzz ', 8.0, now=ts)
        self.assertIsNone(got)


class Split(unittest.TestCase):
    def test_split_parses_and_keeps_the_originals(self):
        with open(SCEN, encoding='utf-8') as f:
            raw = json.load(f)
        out = nav.split_scenarios(raw)
        for k in raw['scenarios']:
            self.assertIn(k, out['scenarios'])
        d = out['scenarios']
        self.assertEqual([s['name'] for s in d['car_cycle_behind__x']['steps']], ['enter'])
        self.assertEqual([s['name'] for s in d['car_cycle_behind__tail']['steps']], ['drive', 'brake', 'exit'])
        self.assertEqual([s['name'] for s in d['car_cycle_behind__pre']['steps']], ['idle', 'approach_behind'])
        self.assertNotIn('car_cycle__pre', d)                    # the enter step is the first one
        self.assertNotIn('car_idle__x', d)
        # the file loads through the harness parser (all scenarios, including the derived ones)
        tmp = os.path.join(os.environ.get('TEMP', HERE), 'car_cases_split_test.json')
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump(out, f)
        try:
            self.assertGreater(len(gs.load_scenarios(tmp)), len(raw['scenarios']))
        finally:
            os.remove(tmp)
        # no drive / brake / exit input in any __x or __pre part
        for k, v in d.items():
            if k.endswith(('__pre', '__x')):
                for st in v['steps']:
                    self.assertNotIn(st['name'], ('drive', 'brake', 'exit'), k)


if __name__ == '__main__':
    unittest.main()
