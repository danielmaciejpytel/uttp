"""Offline tests for the traversal set (gt_traversal.py): vault and climb at the summoned car.

The log lines are shaped like the ones the plugin writes ("UTPP trav:", the vehicle vault line, the release line). FakeNight plays the
parts of the night runner the macro uses; the script says which lines the game writes for a pad part. No input is sent.

  python Source\\tools\\gametest\\test_gt_traversal.py -v
"""
from __future__ import annotations

import os
import sys
import tempfile
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gt_night as gn  # noqa: E402
import gt_settings as gs  # noqa: E402
import gt_traversal as gt  # noqa: E402
from gt_scenario import load_scenarios  # noqa: E402

TRAV_OK = lambda action: (f'UTPP trav: action={action} h=0.90 ext=1.20 depth=1.00 back=0.00 v=3.0 kind=1 decision=Vault '  # noqa: E731
                          'result=Ok(-) montage=Vault_Low')
TRAV_REJECTED = ('UTPP trav: action=9 h=0.90 ext=1.20 depth=1.00 back=0.00 v=0.4 kind=1 decision=- '
                 'result=Rejected(too slow) montage=-')
VEHICLE_NO = '[utpp_vehicle_vault] no vault over a car (vanilla: jump pressed too far from the edge; assist: speed) top=0.9'
VEHICLE_ASSIST = '[utpp_vehicle_vault] assisted vault over a car (vanilla: jump pressed too far from the edge) top=0.9'
VEHICLE_NOT_VALID = ('[utpp_vehicle_vault] no vault over a car (vanilla: vaultValid=false (game obstacle probe); assist: no game vault data) '
                     'valid=false/true top=1.38 ext=2.31 behind=0.10 reach=0.50 max=0.79/1.15 v=0.8 cross=0.70 car=0 driver=false')
RELEASED = lambda d: f'UTPP trav: released to game animation (detail={d})'  # noqa: E731


class FakeGuard:
    def wait(self, s):
        pass


V_XY, V_R = (5.0, 0.0), (0.0, 1.0)     # the fake vehicle


class FakeNight:
    """V stands where the approach was sent (plus `stand_errors[i]` of the i-th approach, car frame); after camera_align the camera
    looks at its target turned by `aim_errors[i]` deg of the i-th alignment (the list repeats its last value)."""
    def __init__(self, tmp, script, car=True, approach_dist=0.5, stand_errors=((0.0, 0.0),), aim_errors=(0.0,)):
        self.utpp_log = os.path.join(tmp, 'session.log')
        open(self.utpp_log, 'w').close()
        self.args = types.SimpleNamespace(log_dir=tmp)
        self.settings = gs.Ledger(None, log=lambda m: None)
        self.script, self.car, self.approach_dist = script, car, approach_dist
        self.stand_errors, self.aim_errors = list(stand_errors), list(aim_errors)
        self.door_spot_override = self.door_radius_override = None
        self.veh_xy, self.veh_r = V_XY, V_R
        self.at = self.look = None
        self.n_approach = self.n_align = 0
        self.calls, self.logs, self.plays = [], [], {}

    def log(self, msg):
        self.logs.append(msg)

    def append(self, lines):
        with open(self.utpp_log, 'a', encoding='utf-8') as f:
            for ln in lines:
                f.write('[2026-11-14 19:00:00.000] [info    ] [ 1] [UTPP] ' + ln + '\n')

    def check_continue(self):
        pass

    def console_verified(self, pad, guard, cmd, echo, label, timeout=3.0):
        self.calls.append(('console', label))
        self.append(['CET [night] TRAVLOG ok=true'])
        return '[night] TRAVLOG ok=true'

    def poll_pos(self, guard, pad=None, timeout=None, newer_than=None):
        self.newer = getattr(self, 'newer', 0) + (newer_than is not None)
        if not self.car:
            return None
        veh = (self.veh_xy[0], self.veh_xy[1], 0.0)
        if self.at is None:
            return {'p': (self.veh_xy[0] - 5.0, self.veh_xy[1], 0.0), 'cf': (1.0, 0.0), 'v': veh, 'vr': self.veh_r, 'src': 'summon'}
        p = gn.from_car_frame(self.at[0], self.at[1], self.veh_xy, self.veh_r)
        cf = (1.0, 0.0)
        if self.look is not None:
            dx, dy = self.look[0] - p[0], self.look[1] - p[1]
            d = (dx * dx + dy * dy) ** 0.5
            cf = gn.rotate_stick(dx / d, dy / d, self.aim_error)
        return {'p': (p[0], p[1], 0.0), 'cf': cf, 'v': veh, 'vr': self.veh_r, 'src': 'summon'}

    def locate_vehicle(self, pad, guard, plan, ep):
        self.calls.append(('locate',))
        return None

    def approach(self, pad, guard, plan, pos, ep):
        self.calls.append(('approach', self.door_spot_override))
        self.radii = getattr(self, 'radii', []) + [self.door_radius_override]
        err = self.stand_errors[min(self.n_approach, len(self.stand_errors) - 1)]
        self.n_approach += 1
        spot = self.door_spot_override
        self.at, self.look = (spot[0] + err[0], spot[1] + err[1]), None
        return self.approach_dist

    def wait_car_stopped(self, pad, guard, ep, pos):
        self.calls.append(('stopped',))
        return pos, True

    def camera_align(self, pad, guard, plan, known, target, ep, tol=None, why='align', fine=False, pulses=None):
        self.calls.append(('aim', why))
        self.aims = getattr(self, 'aims', []) + [(tuple(round(c, 3) for c in target), tol)]
        self.fine = getattr(self, 'fine', []) + [fine]
        self.aim_error = self.aim_errors[min(self.n_align, len(self.aim_errors) - 1)]
        self.n_align += 1
        self.look = target
        return known

    def play(self, pad, guard, name, plan, sc=None):
        self.calls.append(('play', name))
        self.plays[name] = self.plays.get(name, 0) + 1
        self.append(self.script(name, self.plays[name]))
        return {}


def success(part, n):
    return [TRAV_OK(9 if part == 'trav_vault_jump' else 8)]


class TraversalSetTests(unittest.TestCase):
    def test_the_set_is_known_and_not_part_of_all(self):
        self.assertEqual(gn.resolve_sets('traversal'), ['traversal'])
        self.assertNotIn('traversal', gn.resolve_sets('all'))
        scen = load_scenarios(os.path.join(HERE, 'scenarios.json'))
        self.assertEqual(gn.validate_sets(['traversal'], scen), [])
        for part in gt.PARTS:
            self.assertGreater(scen[part].duration, 2.0)
            self.assertFalse(any(st.metrics for st in scen[part].steps), part)      # judged from the log, not by limits

    def test_a_spot_can_come_from_the_environment(self):
        name = 'UTPP_NIGHT_TEST_SPOT'
        old = os.environ.pop(name, None)
        try:
            self.assertEqual(gt.env_spot(name, (1.4, 1.3)), (1.4, 1.3))            # unset: the default
            os.environ[name] = '0.3,1.1'
            self.assertEqual(gt.env_spot(name, (1.4, 1.3)), (0.3, 1.1))
            for bad in ('', '0.3', 'a,b', '1,2,3'):
                os.environ[name] = bad
                self.assertEqual(gt.env_spot(name, (1.4, 1.3)), (1.4, 1.3), bad)   # not two numbers: the default
        finally:
            os.environ.pop(name, None)
            if old is not None:
                os.environ[name] = old

    def test_each_part_taps_jump_while_the_stick_pushes_into_the_car(self):
        scen = load_scenarios(os.path.join(HERE, 'scenarios.json'))
        for part in gt.PARTS:
            jump = [st for st in scen[part].steps if st.name == 'jump'][0]
            self.assertTrue([b for b in jump.buttons if b.button == 'A'], part)
            self.assertTrue(any(f.ls[1] > 0.9 for _, f in scen[part].sample(0.1)), part)


class PathTests(unittest.TestCase):
    def test_crossing_of_the_vehicle_axis(self):
        self.assertAlmostEqual(gt.crossing_lon((1.0, 1.0), (-1.0, -1.0)), 0.0)
        self.assertAlmostEqual(gt.crossing_lon((1.26, 1.30), (-0.2, -0.2)), 1.26 - 1.46 * 1.30 / 1.50)
        self.assertIsNone(gt.crossing_lon((1.8, 1.2), (-2.3, 0.9)))      # ran along the side (2026-11-18 Female vault #2)
        self.assertIsNone(gt.crossing_lon(None, (0.0, -1.0)))
        self.assertIsNone(gt.crossing_lon((0.0, -0.5), (0.0, -1.0)))     # not on the near side first
        self.assertIn('crossed the vehicle axis at lon -1.00', gt.path_note((1.0, 1.0), (-3.0, -1.0)))
        self.assertIn('did not cross', gt.path_note((1.8, 1.2), (-2.3, 0.9)))
        self.assertEqual(gt.path_note(None, (0.0, 0.0)), '')

    def test_car_frame_point(self):
        pos = {'p': (5.0, 1.0, 0.0), 'v': (5.0, 0.0, 0.0), 'vr': (0.0, 1.0)}   # forward (-1, 0), left (0, -1)
        self.assertEqual(gt.car_frame_point(pos), (0.0, -1.0))
        self.assertIsNone(gt.car_frame_point({'p': (0.0, 0.0, 0.0)}))
        self.assertEqual(gt.car_frame_point({'p': (4.0, 0.0, 0.0)}, pos), (1.0, 0.0))

    def test_vault_aim_from_the_environment(self):
        name = 'UTPP_NIGHT_TEST_AIM'
        old = os.environ.pop(name, None)
        try:
            self.assertEqual(gt.env_aim(name, (-0.1, 0.0)), (-0.1, 0.0))
            os.environ[name] = 'off'
            self.assertIsNone(gt.env_aim(name, (-0.1, 0.0)))
            os.environ[name] = '-0.2,0'
            self.assertEqual(gt.env_aim(name, (-0.1, 0.0)), (-0.2, 0.0))
        finally:
            os.environ.pop(name, None)
            if old is not None:
                os.environ[name] = old


class EvidenceTests(unittest.TestCase):
    def test_lines_are_read(self):
        text = '\n'.join(['noise', TRAV_REJECTED, TRAV_OK(8), RELEASED(9), VEHICLE_NO, VEHICLE_ASSIST,
                          'UTPP MMTRAV travRelease on detail=9 gate=0x0 motionOwns=1 w=1.00'])
        ev = gt.evidence(text)
        self.assertEqual([(e['action'], e['result']) for e in ev['mm']], [(9, 'Rejected'), (8, 'Ok')])
        self.assertEqual(ev['mm'][0]['reason'], 'too slow')
        self.assertEqual(ev['released'], [9])
        self.assertEqual([v['kind'] for v in ev['vehicle_vault']], ['no', 'assisted'])
        self.assertEqual(ev['mmtrav'], 1)

    def test_judge_vault_and_climb(self):
        self.assertEqual(gt.judge('vault', gt.evidence(TRAV_OK(9)))['by'], ['trav line Ok'])
        self.assertTrue(gt.judge('vault', gt.evidence(RELEASED(9)))['ok'])
        self.assertTrue(gt.judge('vault', gt.evidence(VEHICLE_ASSIST))['ok'])
        self.assertTrue(gt.judge('climb', gt.evidence(TRAV_OK(8)))['ok'])
        self.assertFalse(gt.judge('climb', gt.evidence(VEHICLE_ASSIST))['ok'])       # the vehicle line is a vault only
        self.assertFalse(gt.judge('climb', gt.evidence(TRAV_OK(9)))['ok'])           # the other action does not count
        rej = gt.judge('vault', gt.evidence(TRAV_REJECTED + '\n' + VEHICLE_NO))
        self.assertFalse(rej['ok'])
        self.assertIn('Rejected(too slow)', rej['why'])
        self.assertIn('too far from the edge', rej['why'])
        self.assertIn('no traversal line at all', gt.judge('vault', gt.evidence(''))['why'])

    def test_a_vault_the_game_reports_as_not_valid_is_flagged(self):
        ev = gt.evidence(VEHICLE_NOT_VALID)
        self.assertEqual(ev['vault_not_valid'], 1)
        res = gt.judge('vault', ev)
        self.assertFalse(res['ok'])
        self.assertTrue(res['invalid'])
        self.assertIn('vaultValid=false', res['why'])
        self.assertEqual(gt.evidence(VEHICLE_NO)['vault_not_valid'], 0)
        self.assertFalse(gt.judge('vault', gt.evidence(VEHICLE_NO))['invalid'])
        self.assertFalse(gt.judge('climb', ev)['invalid'])                          # only a vault is "not valid" here
        # a vault that happened anyway is not invalid
        done = gt.judge('vault', gt.evidence(VEHICLE_NOT_VALID + '\n' + TRAV_OK(9)))
        self.assertTrue(done['ok'])
        self.assertFalse(done['invalid'])

    def test_feed_column_detailed(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, 'utpp-feed-2026-11-14-19-00-00.txt')
            rows = ['# cols t dt vx detailed onGround']
            for i in range(40):                                  # 0.05 s per row; vault (9) in the last second only
                rows.append(f'{i * 0.05:.3f} 0.05 0 {9 if i >= 25 else 0} 1')
            with open(path, 'w') as f:
                f.write('\n'.join(rows) + '\n')
            self.assertEqual(gt.feed_detailed(path, 0.5), {9})
            self.assertEqual(gt.feed_detailed(path, 5.0), {0, 9})
            self.assertEqual(gt.newest_feed(d), path)
            self.assertTrue(gt.judge('vault', gt.evidence(''), {9})['ok'])
            self.assertFalse(gt.judge('climb', gt.evidence(''), {9})['ok'])
            self.assertIsNone(gt.feed_detailed(os.path.join(d, 'none.txt'), 1.0))
            bad = os.path.join(d, 'utpp-feed-x.txt')
            with open(bad, 'w') as f:
                f.write('# cols t dt\n0 0.1\n')
            self.assertIsNone(gt.feed_detailed(bad, 1.0))


class MacroTests(unittest.TestCase):
    def run_macro(self, script, **kw):
        with tempfile.TemporaryDirectory() as d:
            night = FakeNight(d, script, **kw)
            out = gt.run_traversal_macro(night, None, FakeGuard(), {}, {})
        return night, out

    def test_both_traversals_happen_at_the_first_attempt(self):
        night, out = self.run_macro(success)
        self.assertEqual(out['verdict'], 'PASS')
        self.assertEqual(out['results'], {'vault': True, 'climb': True})
        self.assertEqual([c[1] for c in night.calls if c[0] == 'approach'], [gt.KINDS['vault']['spot'], (-0.2, 1.3)])   # each kind's spot
        self.assertEqual([c[1] for c in night.calls if c[0] == 'aim'], ['trav-aim'])  # only the vault turns to its crossing point
        self.assertEqual([c[1] for c in night.calls if c[0] == 'play'], ['trav_vault_jump', 'trav_climb_jump'])
        self.assertEqual([w['label'] for w in out['phase_windows']], ['trav_vault_1', 'trav_climb_1'])
        self.assertTrue(all(w['start'] <= w['end'] for w in out['phase_windows']))
        self.assertIsNone(night.door_spot_override)                                  # the vehicle macro's door is back
        self.assertEqual(night.calls[0], ('console', 'traversal/log_on'))
        self.assertEqual(night.calls[-1], ('console', 'traversal/log_off'))
        self.assertEqual([e['state'] for e in night.settings.entries], ['restored'])  # the vault log is switched off again
        # phase windows are what the pose meter and the sheets read
        self.assertEqual([w['name'] for w in gn.phase_windows_of([dict(out, name='traversal', run=1)])],
                         ['trav_vault_1_r1', 'trav_climb_1_r1'])

    def test_the_vault_aims_at_its_crossing_point_and_logs_the_path(self):
        night, out = self.run_macro(lambda part, n: [])
        aim = gt.KINDS['vault']['aim']
        want = tuple(round(c, 3) for c in gn.from_car_frame(aim[0], aim[1], (5.0, 0.0), (0.0, 1.0)))
        self.assertEqual(night.aims[0], (want, gt.AIM_TOL_DEG))
        self.assertEqual(night.fine[0], True)                            # the fine pulses
        self.assertGreaterEqual(night.newer, 1)                          # the stand / aim check reads a POS after the pulses
        vault = [a for a in out['attempts'] if a['kind'] == 'vault']
        self.assertEqual(vault[0]['stand'], list(gt.KINDS['vault']['spot']))   # the fake V stands still at the spot: no crossing
        self.assertEqual((vault[0]['stand_offset_m'], vault[0]['aim_bearing_deg']), (0.0, 0.0))
        self.assertIsNone(vault[0]['crossing_lon'])
        self.assertIn('no traversal line at all', vault[0]['why'])
        self.assertTrue(any('ready to jump [stand lon +1.30 lat 1.20, 0.00 m from the spot, camera ' in m
                            and 'deg off the aim]' in m for m in night.logs), night.logs)

    def test_the_vault_stands_tight_and_the_climb_keeps_the_approach_radius(self):
        night, out = self.run_macro(success)
        self.assertEqual(night.radii, [gt.STAND_RADIUS_M, None])
        self.assertIsNone(night.door_radius_override)
        self.assertLess(gt.STAND_RADIUS_M, gt.STAND_TOL_M)

    def test_a_vault_stand_off_the_spot_is_approached_once_more(self):
        night, out = self.run_macro(success, stand_errors=[(0.0, 0.45), (0.05, 0.0)])
        self.assertEqual(out['verdict'], 'PASS')
        vault = [a for a in out['attempts'] if a['kind'] == 'vault']
        self.assertEqual(len(vault), 1)
        self.assertTrue(vault[0]['reapproached'])
        self.assertEqual(vault[0]['stand_offset_m'], 0.05)
        self.assertEqual([c[1] for c in night.calls if c[0] == 'approach'][:2], [gt.KINDS['vault']['spot']] * 2)
        self.assertTrue(any('0.45 m from the spot' in m and 'approaching once more' in m for m in night.logs))

    def test_a_vault_that_cannot_stand_at_the_spot_does_not_jump(self):
        night, out = self.run_macro(success, stand_errors=[(0.4, 0.0)])
        self.assertEqual(out['verdict'], 'FAIL')
        self.assertNotIn('trav_vault_jump', night.plays)
        vault = [a for a in out['attempts'] if a['kind'] == 'vault']
        self.assertEqual([a['played'] for a in vault], [False] * gt.ATTEMPTS)
        self.assertTrue(all(a['reapproached'] and a['stand_offset_m'] == 0.4 for a in vault))
        self.assertIn('0.40 m from the spot', out['fails'][0])
        self.assertIn('after a second approach; no jump', out['fails'][0])
        self.assertTrue(out['fails'][0].startswith('vault: '))
        self.assertEqual(out['results']['climb'], True)                    # the climb is not held to the vault's stand

    def test_a_vault_aim_off_is_aligned_once_more(self):
        night, out = self.run_macro(success, aim_errors=[6.0, 1.0])
        self.assertEqual(out['verdict'], 'PASS')
        vault = [a for a in out['attempts'] if a['kind'] == 'vault']
        self.assertTrue(vault[0]['realigned'])
        self.assertAlmostEqual(abs(vault[0]['aim_bearing_deg']), 1.0, places=3)
        self.assertEqual([c[1] for c in night.calls if c[0] == 'aim'], ['trav-aim', 'trav-aim'])

    def test_a_vault_whose_aim_is_not_reached_does_not_jump(self):
        night, out = self.run_macro(success, aim_errors=[5.0])
        self.assertEqual(out['verdict'], 'FAIL')
        self.assertNotIn('trav_vault_jump', night.plays)
        self.assertIn('aim not reached', out['fails'][0])
        self.assertIn('5.0 deg off the aim point after two alignments (tol 3 deg); no jump', out['fails'][0])
        vault = [a for a in out['attempts'] if a['kind'] == 'vault']
        self.assertTrue(all(not a['played'] and a['realigned'] for a in vault))

    def test_stand_and_aim(self):
        spot, aim = (0.9, 1.2), (-0.1, 0.0)
        p = gn.from_car_frame(1.2, 1.6, V_XY, V_R)
        target = gn.from_car_frame(aim[0], aim[1], V_XY, V_R)
        cf = ((target[0] - p[0]) / 1.0, (target[1] - p[1]) / 1.0)
        pos = {'p': (p[0], p[1], 0.0), 'cf': cf}
        res = gt.stand_and_aim(pos, {'v': (5.0, 0.0, 0.0), 'vr': V_R}, spot, aim)   # the vehicle from the other record
        self.assertAlmostEqual(res['offset_m'], 0.5)
        self.assertAlmostEqual(res['bearing_deg'], 0.0, places=6)
        self.assertEqual(gt.stand_and_aim(None, None, spot, aim), {'stand': None, 'offset_m': None, 'bearing_deg': None})

    def test_a_second_attempt_can_succeed(self):
        def script(part, n):
            return [TRAV_REJECTED] if part == 'trav_vault_jump' and n == 1 else success(part, n)
        night, out = self.run_macro(script)
        self.assertEqual(out['verdict'], 'PASS')
        vault = [a for a in out['attempts'] if a['kind'] == 'vault']
        self.assertEqual([a['ok'] for a in vault], [False, True])
        self.assertEqual(night.plays['trav_vault_jump'], 2)

    def test_a_traversal_that_never_happens_fails_with_the_reason(self):
        def script(part, n):
            return success(part, n) if part == 'trav_vault_jump' else [VEHICLE_NO]
        night, out = self.run_macro(script)
        self.assertEqual(out['verdict'], 'FAIL')
        self.assertEqual(out['results'], {'vault': True, 'climb': False})
        self.assertEqual(night.plays['trav_climb_jump'], gt.ATTEMPTS)
        self.assertEqual(len(out['fails']), 1)
        self.assertTrue(out['fails'][0].startswith('climb: '))
        self.assertIn('too far from the edge', out['fails'][0])

    def test_a_vault_the_game_reports_as_not_valid_is_invalid_not_a_failure(self):
        def script(part, n):
            return [VEHICLE_NOT_VALID] if part == 'trav_vault_jump' else success(part, n)
        night, out = self.run_macro(script)
        self.assertEqual(out['verdict'], 'INVALID')
        self.assertEqual(out['fails'], [])
        self.assertEqual(out['results'], {'vault': 'invalid', 'climb': True})
        self.assertEqual(len(out['invalid_fails']), 1)
        self.assertTrue(out['invalid_fails'][0].startswith('vault: '))
        self.assertIn('vaultValid=false', out['invalid_cause'])
        self.assertTrue(all(a['invalid'] for a in out['attempts'] if a['kind'] == 'vault'))
        self.assertTrue(any('INVALID' in m for m in night.logs))
        # the night verdict counts it as an invalid macro, not as a failure of the mod
        rec = dict(out, name='traversal', run=1)
        self.assertEqual(gn.macro_failures([rec]), [])
        self.assertEqual([i['scenario'] for i in gn.macro_invalid([rec])], ['traversal#1'])

    def test_one_attempt_without_the_not_valid_line_keeps_the_failure(self):
        def script(part, n):
            if part == 'trav_vault_jump':
                return [VEHICLE_NOT_VALID] if n == 1 else [TRAV_REJECTED]
            return success(part, n)
        night, out = self.run_macro(script)
        self.assertEqual(out['verdict'], 'FAIL')
        self.assertTrue(out['fails'][0].startswith('vault: '))
        self.assertEqual(out['invalid_fails'], [])

    def test_a_failed_climb_still_fails_next_to_an_invalid_vault(self):
        def script(part, n):
            return [VEHICLE_NOT_VALID] if part == 'trav_vault_jump' else [VEHICLE_NO]
        night, out = self.run_macro(script)
        self.assertEqual(out['verdict'], 'FAIL')
        self.assertEqual([f.split(':')[0] for f in out['fails']], ['climb'])
        self.assertEqual([f.split(':')[0] for f in out['invalid_fails']], ['vault'])

    def test_a_not_valid_line_does_not_excuse_a_climb(self):
        def script(part, n):
            return success(part, n) if part == 'trav_vault_jump' else [VEHICLE_NOT_VALID]
        night, out = self.run_macro(script)
        self.assertEqual(out['verdict'], 'FAIL')
        self.assertEqual(out['invalid_fails'], [])

    def test_no_car_is_a_skip(self):
        night, out = self.run_macro(success, car=False)
        self.assertEqual(out['verdict'], 'SKIP')
        self.assertIn(('locate',), night.calls)                                    # the vehicle macro's summon was tried first
        self.assertEqual([c for c in night.calls if c[0] == 'play'], [])
        self.assertEqual(night.calls[-1], ('console', 'traversal/log_off'))

    def test_v_that_cannot_reach_the_spot_does_not_play_the_part(self):
        night, out = self.run_macro(success, approach_dist=6.0)
        self.assertEqual(out['verdict'], 'FAIL')
        self.assertEqual([c for c in night.calls if c[0] == 'play'], [])
        self.assertTrue(all(not a['played'] for a in out['attempts']))
        self.assertIn('could not stand at the car', out['fails'][0])

    def test_the_macro_failure_counts_in_the_night_verdict(self):
        night, out = self.run_macro(lambda part, n: [])
        rec = dict(out, name='traversal', run=1)
        fails = gn.macro_failures([rec])
        self.assertEqual([f['scenario'] for f in fails], ['traversal#1'])
        self.assertTrue(gn.macro_result_text(fails, []).startswith('macro FAIL: traversal#1'))


# bike positions read from the POS lines after the bike stopped (V at the start point 1588.78, 2164.07)
GOOD_BIKE = {'p': (1588.78, 2164.07, 177.10), 'v': (1586.90, 2161.24, 177.54), 'vr': (-0.875, -0.411)}   # 2026-11-18, vault + climb 1st try
OFF_BIKE = {'p': (1588.78, 2164.07, 177.10), 'v': (1586.01, 2162.02, 177.47), 'vr': (-0.983, -0.092)}    # 2026-11-24, h 0.89
TRAV_H = lambda h, result='Rejected(too slow)': (f'UTPP trav: action=9 h={h:.2f} ext=0.35 depth=0.34 back=0.89 v=1.7 kind=1 '  # noqa: E731
                                                 f'decision=- result={result} montage=-')


class SiteNight(FakeNight):
    """FakeNight with the bike at the save's start point; the placed spawn moves it to the placement (or leaves it when
    place_ok is False)."""
    def __init__(self, tmp, script, bike, place_ok=True, summon_polls=0, spawn_ok=True, **kw):
        super().__init__(tmp, script, **kw)
        self.veh_xy, self.veh_r = bike['v'][:2], bike['vr']
        self.place_ok, self.placed, self.summon_polls, self.spawn_ok = place_ok, 0, summon_polls, spawn_ok

    def console_verified(self, pad, guard, cmd, echo, label, timeout=3.0):
        if label != 'traversal/place':
            return super().console_verified(pad, guard, cmd, echo, label, timeout)
        self.calls.append(('console', label))
        self.placed += 1
        if self.place_ok:
            self.veh_xy, self.veh_r = gt.BIKE_PLACE['v'], gt.BIKE_PLACE['vr']
        return '[night] SPAWN placed ok=%s userdata' % ('true' if self.spawn_ok else 'false')

    def poll_pos(self, guard, pad=None, timeout=None, newer_than=None):
        pos = super().poll_pos(guard, pad, timeout, newer_than)
        if pos and self.placed and self.summon_polls > 0:
            # the new entity is not there yet: the position line reports the save's own bike, elsewhere
            self.summon_polls -= 1
            return dict(pos, v=(1584.6, 2167.9, 177.0), vr=(0.0, 1.0), src='summon')
        return dict(pos, src='tag') if pos else pos


class PlacementTests(unittest.TestCase):
    def test_the_recorded_placements_are_judged(self):
        good, off = gt.placement_check(GOOD_BIKE), gt.placement_check(OFF_BIKE)
        self.assertTrue(good['applies'] and good['ok'])
        self.assertTrue(off['applies'])
        self.assertFalse(off['ok'])
        self.assertAlmostEqual(off['off_m'], 1.18, delta=0.01)
        self.assertAlmostEqual(off['off_deg'], 19.9, delta=0.5)
        self.assertIn('1.18 m and 20 deg off', off['why'])
        far = dict(OFF_BIKE, v=(1619.5, 2132.1, 178.6))                     # another place: not judged
        self.assertEqual(gt.placement_check(far)['applies'], False)
        self.assertFalse(gt.placement_check({'p': (0, 0, 0)})['applies'])
        self.assertFalse(gt.placement_check(OFF_BIKE, place={})['applies'])  # check off

    def test_the_spawn_convention_reproduces_both_recorded_bikes(self):
        # the relative spawn puts the bike 3 m right / 2 m ahead of V with V's facing; the forward vector car_axes derives from the
        # recorded right vector must give back the place each bike settled at (so the placed spawn's heading uses the same axes)
        for bike in (GOOD_BIKE, OFF_BIKE):
            (fx, fy), (lx, ly) = gn.car_axes(bike['vr'])
            rx, ry = -lx, -ly
            x = bike['p'][0] + rx * 3 + fx * 2
            y = bike['p'][1] + ry * 3 + fy * 2
            self.assertLess(((x - bike['v'][0]) ** 2 + (y - bike['v'][1]) ** 2) ** 0.5, 0.3)   # settle drift; a wrong axis is metres off

    def test_the_placed_spawn_command(self):
        cmd = gt.placed_spawn_cmd(gt.BIKE_PLACE_DEFAULT)
        self.assertIn('Vector4.new(1586.90,2161.24,177.40,1)', cmd)
        self.assertIn('Vector4.new(0.4251,-0.9051,0,0):ToRotation().yaw', cmd)    # forward from the right vector (unit)
        self.assertIn("DeleteTagged('" + gn.NIGHT_TAG + "')", cmd)
        self.assertIn("[night] SPAWN placed ok=", cmd)

    def test_place_from_the_environment(self):
        name = 'UTPP_NIGHT_TEST_PLACE'
        old = os.environ.pop(name, None)
        try:
            self.assertEqual(gt.env_place(name, {'x': 1}), {'x': 1})
            os.environ[name] = 'off'
            self.assertIsNone(gt.env_place(name, {'x': 1}))
            os.environ[name] = '1,2,3,0,1'
            self.assertEqual(gt.env_place(name, None), {'v': (1.0, 2.0), 'z': 3.0, 'vr': (0.0, 1.0)})
            os.environ[name] = '1,2'
            self.assertEqual(gt.env_place(name, {'x': 1}), {'x': 1})
        finally:
            os.environ.pop(name, None)
            if old is not None:
                os.environ[name] = old

    def test_vault_height(self):
        self.assertEqual(gt.vault_height(gt.evidence(TRAV_OK(8) + '\n' + TRAV_H(0.89) + '\n' + TRAV_H(1.10))), 0.89)
        self.assertIsNone(gt.vault_height(gt.evidence(TRAV_OK(8))))
        self.assertEqual([gt.height_off(h) for h in (None, 0.89, 1.0, 1.08, 1.15, 1.27)], [False, True, False, False, False, True])

    def run_site(self, script, bike, **kw):
        with tempfile.TemporaryDirectory() as d:
            night = SiteNight(d, script, bike, **kw)
            out = gt.run_traversal_macro(night, None, FakeGuard(), {}, {})
        return night, out

    def test_a_bike_off_the_placement_is_spawned_there_once_before_the_jumps(self):
        night, out = self.run_site(success, OFF_BIKE)
        self.assertEqual(night.placed, 1)
        self.assertEqual(out['verdict'], 'PASS')
        labels = [c for c in night.calls if c[0] in ('console', 'play', 'stopped')]
        self.assertLess(labels.index(('console', 'traversal/place')), labels.index(('play', 'trav_vault_jump')))
        self.assertIn(('stopped',), labels)                                    # waits for the new bike to stand
        self.assertTrue(out['placement']['replaced'])
        self.assertFalse(out['placement']['before']['ok'])
        self.assertTrue(out['placement']['after']['ok'])
        self.assertTrue(any('spawning the bike at the level placement once' in m for m in night.logs), night.logs)

    def test_a_bike_at_the_placement_is_left_alone(self):
        night, out = self.run_site(success, GOOD_BIKE)
        self.assertEqual(night.placed, 0)
        self.assertEqual(out['verdict'], 'PASS')
        self.assertTrue(out['placement']['before']['ok'])
        self.assertNotIn('replaced', out['placement'])

    def test_a_bike_that_stays_off_means_no_jump_and_an_invalid_setup(self):
        night, out = self.run_site(success, OFF_BIKE, place_ok=False)
        self.assertEqual(night.placed, 1)                                      # one try only
        self.assertFalse(out['placement']['after']['ok'])
        self.assertEqual(night.plays, {})                                      # V never jumps from other ground
        self.assertEqual(out['verdict'], 'INVALID')
        self.assertEqual(out['fails'], [])
        self.assertIn('still not at the level placement', out['invalid_cause'])
        self.assertEqual(night.calls[-1], ('console', 'traversal/log_off'))    # the vault log is still switched off
        rec = dict(out, name='traversal', run=1)
        self.assertEqual(gn.macro_failures([rec]), [])                         # not counted against the mod

    def test_an_unconfirmed_placed_spawn_means_no_jump(self):
        night, out = self.run_site(success, OFF_BIKE, spawn_ok=False)
        self.assertEqual((out['verdict'], night.plays), ('INVALID', {}))
        self.assertIn('not confirmed', out['invalid_cause'])

    def test_the_save_bike_reported_before_the_new_one_is_not_taken(self):
        night, out = self.run_site(success, OFF_BIKE, summon_polls=2)
        self.assertEqual(out['verdict'], 'PASS')
        self.assertTrue(out['placement']['after']['ok'])                       # judged on the new tagged bike
        night, out = self.run_site(success, OFF_BIKE, summon_polls=gt.PLACE_POLLS)
        self.assertEqual((out['verdict'], night.plays), ('INVALID', {}))      # never reported: no jump
        self.assertIn('no position of the placed bike', out['invalid_cause'])

    def test_a_failed_vault_at_an_odd_height_places_the_bike_once(self):
        def script(part, n):
            if part == 'trav_vault_jump':
                return [TRAV_H(0.89)] if n < 3 else [TRAV_H(1.08, 'Ok(-)')]
            return success(part, n)
        night, out = self.run_site(script, GOOD_BIKE)
        self.assertEqual(night.placed, 1)                                      # after the first failure, not after the second
        self.assertIn('vault obstacle height 0.89 m', out['placement']['replace_why'])
        vault = [a for a in out['attempts'] if a['kind'] == 'vault']
        self.assertEqual([a['obstacle_h_m'] for a in vault], [0.89, 0.89])
        self.assertEqual(out['verdict'], 'FAIL')
        self.assertEqual(sum('obstacle height 0.89 m' in w for w in out['warnings']), 2)

    def test_a_vault_that_worked_at_an_odd_height_is_kept_with_a_warning(self):
        def script(part, n):
            return [TRAV_H(0.89, 'Unsupported(vehicle)'), RELEASED(9)] if part == 'trav_vault_jump' else success(part, n)
        night, out = self.run_site(script, GOOD_BIKE)
        self.assertEqual(out['verdict'], 'PASS')
        self.assertEqual(night.placed, 0)
        self.assertTrue(any('not comparable' in w for w in out['warnings']))

    def test_the_check_does_not_touch_a_vehicle_elsewhere(self):
        night, out = self.run_macro_elsewhere()
        self.assertEqual(out['placement']['before']['applies'], False)
        self.assertNotIn(('console', 'traversal/place'), night.calls)

    def run_macro_elsewhere(self):
        with tempfile.TemporaryDirectory() as d:
            night = FakeNight(d, success)
            out = gt.run_traversal_macro(night, None, FakeGuard(), {}, {})
        return night, out

    def test_the_dry_run_names_the_placement(self):
        self.assertTrue(any(l.strip().startswith('place') and '1586.90, 2161.24' in l for l in gt.dry_run_lines()))


if __name__ == '__main__':
    unittest.main()
