"""Offline tests for bike_cases_nav.py and scenarios_bike_cases.json (no game, no input).

  python Source\\tools\\gametest\\test_bike_cases_nav.py -v
"""
from __future__ import annotations

import json
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import bike_cases_nav as bn  # noqa: E402
import car_cases_nav as car  # noqa: E402
import gt_scenario as gs  # noqa: E402

SCEN = os.path.join(HERE, 'scenarios_bike_cases.json')
ARCH = 'Vehicle.v_sportbike2_arch_player'


def pos(px, py, car_xy=(0.0, 0.0), right=(1.0, 0.0), cf=(0.0, 1.0), mounted=False):
    """Bike at car_xy facing +y (right = +x); V at (px, py). lat > 0 = bike LEFT = world -x."""
    return {'p': (px, py, 0.0), 'cf': cf, 'v': (car_xy[0], car_xy[1], 0.0), 'vr': right, 'src': 'tag', 'weapon': False,
            'mounted': mounted, 'age': 0.2}


class Presets(unittest.TestCase):
    def test_all_three_bikes_and_the_kusanagi_alias(self):
        for rec, key in (('Vehicle.v_sportbike1_yaiba_kusanagi_player', 'v_sportbike1_yaiba_kusanagi'),
                         ('Vehicle.v_sportbike1_yaiba_kusanagi_player_02', 'v_sportbike1_yaiba_kusanagi'),
                         (ARCH, 'v_sportbike2_arch'),
                         ('Vehicle.v_sportbike3_brennan_apollo_player', 'v_sportbike3_brennan_apollo')):
            p = bn.preset(rec)
            self.assertEqual(p['key'], key)
            self.assertTrue(p['known'])
            self.assertLessEqual(p['lat_max'], 1.0)          # inside the seat sphere r 1.0 .. 1.3 m

    def test_unknown_bike_gets_bike_geometry_but_the_car_module_keeps_its_own(self):
        self.assertLessEqual(bn.preset('Vehicle.v_foo_bike')['lat_max'], 1.0)
        self.assertEqual(car.preset('Vehicle.v_foo_bike')['lat_max'], 2.7)           # untouched car generic
        self.assertFalse(car.preset(ARCH)['known'])                                  # no bike preset leaked into the car module
        self.assertTrue(car.preset('Vehicle.v_standard2_archer_hella_player')['known'])


class Decide(unittest.TestCase):
    p = bn.preset(ARCH)

    def test_the_failed_run_pose_is_not_ok_any_more(self):
        # 2026-11-07 run: V at lat 1.49 .. 1.59 m (the bike's left = world -x), lon ~0: the old car window accepted it
        r = bn.cn.decide(pos(-1.5, 0.0), self.p, 'driver')
        self.assertEqual(r['state'], 'move')
        old = car.decide(pos(-1.5, 0.0), car.preset(ARCH), 'driver')
        self.assertEqual(old['state'], 'ok')                                         # the car window: the root cause

    def test_close_pose_is_ok_and_the_walk_target_is_inside_the_sphere(self):
        self.assertEqual(bn.cn.decide(pos(-0.8, 0.0), self.p, 'driver')['state'], 'ok')
        far = bn.cn.decide(pos(-2.5, 0.0), self.p, 'driver')
        self.assertEqual(far['state'], 'move')
        retry = bn.cn.decide(pos(-1.5, 0.0), self.p, 'driver', mode='retry')
        self.assertEqual(retry['state'], 'move')
        self.assertLessEqual(abs(retry['lat_target']), 1.0)

    def test_right_side_uses_the_passenger_sign(self):
        self.assertEqual(bn.cn.decide(pos(0.8, 0.0), self.p, 'passenger')['state'], 'ok')       # bike right = world +x
        self.assertEqual(bn.cn.decide(pos(-0.8, 0.0), self.p, 'passenger')['state'], 'move')    # wrong side

    def test_lon_bias_shifts_the_window(self):
        self.assertEqual(bn.cn.decide(pos(-0.8, 0.45), self.p, 'driver', lon_bias=0.45)['state'], 'ok')
        self.assertEqual(bn.cn.decide(pos(-0.8, 0.45), self.p, 'driver')['state'], 'move')


class Side(unittest.TestCase):
    def test_sides_and_mounted(self):
        self.assertEqual(bn.side_of(pos(-1.0, 0.0))['side'], 'left')
        self.assertEqual(bn.side_of(pos(1.0, 0.0))['side'], 'right')
        self.assertEqual(bn.side_of(pos(1.0, 0.0, mounted=True))['state'], 'mounted')
        self.assertEqual(bn.side_of({'p': (0, 0, 0), 'cf': (0, 1), 'v': None, 'vr': None})['state'], 'nocar')


class Scenarios(unittest.TestCase):
    def setUp(self):
        with open(SCEN, encoding='utf-8') as f:
            self.raw = json.load(f)

    def test_file_loads_and_every_scenario_is_valid(self):
        names = ('bike_idle', 'bike_exit_only', 'bike_walk_away', 'bike_mount', 'bike_ride', 'bike_dismount',
                 'bike_dismount_stick_l', 'bike_dismount_stick_r')
        for n in names:
            self.assertIn(n, self.raw['scenarios'])
        scn = gs.load_scenarios(SCEN) if hasattr(gs, 'load_scenarios') else None
        if scn is not None:
            for n in names:
                self.assertIn(n, scn)

    def test_pad_bindings(self):
        s = self.raw['scenarios']
        self.assertEqual(s['bike_mount']['steps'][0]['buttons'][0]['button'], 'X')
        self.assertEqual(s['bike_dismount']['steps'][0]['buttons'][0]['button'], 'B')
        self.assertGreaterEqual(s['bike_dismount']['steps'][0]['buttons'][0]['hold'], 1.0)
        self.assertLess(s['bike_dismount_stick_l']['steps'][0]['ls'][0][1], 0)       # left = negative x
        self.assertGreater(s['bike_dismount_stick_r']['steps'][0]['ls'][0][1], 0)


if __name__ == '__main__':
    unittest.main()
