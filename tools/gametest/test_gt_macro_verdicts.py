"""Offline tests for the macro verdicts of the night runner: a failed macro counts in OVERALL, the result line and the exit status;
a weapon failure caused by the game's forced holstering (ForceEmptyHands) is INVALID, with one bounded second try.

The records are shaped like the ones of a real weapon session (katana failed at the crouch attack while the game blocked the
weapon, blunt / baton / bat found nothing in the weapon slot, the firearm macro did not confirm the draw); the log lines are
copies of the two kinds of block line the plugin wrote (vehicle scene, no combat). No input is sent.

  python Source\\tools\\gametest\\test_gt_macro_verdicts.py -v
"""
from __future__ import annotations

import contextlib
import io
import os
import sys
import tempfile
import types
import unittest
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gt_night as gn  # noqa: E402
import night  # noqa: E402
import run as gt_run  # noqa: E402
import test_gt_dodge as td  # noqa: E402

BLOCK_VEHICLE = ('[2026-11-14 18:21:21.717] [info    ] [ 35796] [UTPP] UTPP CET [utpp_weapon_restriction] ForceEmptyHands enter;'
                 'forceParam=false;zoneParam=false;forceFact=0;noCombat=false;fastForward=false;vehicleScene=true;zones=1;'
                 'sceneAim=false;sceneSafe=false')
BLOCK_NO_COMBAT = BLOCK_VEHICLE.replace('18:21:21.717', '17:48:44.160').replace('noCombat=false', 'noCombat=true') \
    .replace('vehicleScene=true', 'vehicleScene=false')
POS_BLOCKED = ('[2026-11-14 18:21:30.511] [info    ] [ 71648] [UTPP] UTPP CET [night] POS p=1554.33,2112.03,180.79 cf=-0.984,-0.165 '
               'v=1638.70,2136.15,177.43,-0.743,-0.594 src=summon w=0 m=0 freeMove=1/0x0 psmW=0 ub=5 wi=none n=794')
POS_FREE = POS_BLOCKED.replace('ub=5', 'ub=1').replace('18:21:30.511', '18:22:10.511')


def view(tag, verdict, fails=(), a='18:20:40.000', b='18:20:50.000'):
    return {'weapon': tag, 'verdict': verdict, 'fails': list(fails), 'warnings': [],
            'phases': [{'phase': 'x', 'start': f'2026-11-14 {a}', 'end': f'2026-11-14 {b}'}], 'phase_windows': []}


def weapons_record(parts, start='18:19:47.000', end='18:22:00.000'):
    fails = [f'{t}: {f}' for t, v in parts.items() for f in v['fails']]
    vs = [v['verdict'] for v in parts.values()]
    return {'name': 'melee_weapons', 'run': 1, 'start': datetime.strptime('2026-11-14 ' + start, '%Y-%m-%d %H:%M:%S.%f'),
            'end': datetime.strptime('2026-11-14 ' + end, '%Y-%m-%d %H:%M:%S.%f'), 'weapons': parts, 'fails': fails,
            'verdict': 'FAIL' if 'FAIL' in vs else 'PASS'}


def session_record():
    """knife passed, katana failed after the block started in its sprint attack, blunt / baton / bat lost the weapon."""
    lost = 'nothing in the weapon slot after the draw request'
    return weapons_record({
        'knife': view('knife', 'PASS', a='18:20:00.000', b='18:20:39.000'),
        'katana': view('katana', 'FAIL', ['crouch_attack: the weapon could not be put back in the hand; phase skipped'],
                       a='18:20:41.000', b='18:21:29.000'),
        'blunt': view('blunt', 'FAIL', [lost], a='18:21:31.000', b='18:21:38.000'),
        'baton': view('baton', 'FAIL', [lost], a='18:21:39.000', b='18:21:46.000'),
        'bat': view('bat', 'FAIL', [lost], a='18:21:47.000', b='18:21:54.000')})


def firearm_record(fails=('draw not confirmed (NightPos w=0)',)):
    return {'name': 'weapon_fire_reload', 'run': 1, 'start': datetime(2026, 11, 14, 18, 23, 10),
            'end': datetime(2026, 11, 14, 18, 23, 20), 'verdict': 'FAIL', 'fails': list(fails)}


class MacroFailureTests(unittest.TestCase):
    def test_names_of_failed_macros_and_parts(self):
        recs = [{'name': 'melee_fists', 'run': 1, 'verdict': 'PASS', 'fails': []}, session_record(), firearm_record()]
        fails = gn.macro_failures(recs)
        self.assertEqual([f['scenario'] for f in fails], ['melee_weapons#1', 'weapon_fire_reload#1'])
        self.assertEqual(fails[0]['macros'], ['katana', 'blunt', 'baton', 'bat'])
        self.assertEqual(gn.macro_result_text(fails, []),
                         'macro FAIL: melee_weapons#1 (katana, blunt, baton, bat); weapon_fire_reload#1')

    def test_all_pass_gives_no_text_and_timeout_counts(self):
        self.assertEqual(gn.macro_result_text([], []), '')
        recs = [{'name': 'melee_fists_target', 'run': 1, 'verdict': 'TIMEOUT', 'fails': ['too slow']}]
        self.assertEqual([f['scenario'] for f in gn.macro_failures(recs)], ['melee_fists_target#1'])

    def test_a_frozen_game_scenario_is_not_judged(self):
        self.assertEqual(gn.macro_failures([firearm_record()], not_judged=['weapon_fire_reload#1']), [])

    def test_overall_is_fail_with_a_macro_failure_although_the_steps_pass(self):
        fails = gn.macro_failures([firearm_record()])
        self.assertEqual(gn.overall_with_macros('PASS', [], []), 'PASS')
        self.assertEqual(gn.overall_with_macros('PASS', fails, []), 'FAIL')
        self.assertEqual(gn.overall_with_macros('PASS', [], [{'scenario': 'x#1', 'macros': [], 'cause': ''}]), 'FAIL')
        self.assertEqual(gn.overall_with_macros(None, [], []), 'FAIL')

    def test_write_outputs_and_the_overall_line_carry_the_macro_failures(self):
        rows, lines = td.analyze(td.read(td.FIXED_A), td.FIXED_A)
        plan = dict(td.plan_of(td.FIXED_A), macro_failures=gn.macro_failures([session_record()]), macro_invalid=[])
        with tempfile.TemporaryDirectory() as d:
            summary = gt_run.write_outputs(d, plan, list(rows.values()), lines, td.FIXED_A)
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                gt_run.print_table(list(rows.values()), summary)
        self.assertEqual(summary['overall'], 'FAIL')
        self.assertEqual(summary['macro_failures'][0]['macros'], ['katana', 'blunt', 'baton', 'bat'])
        over = [ln for ln in buf.getvalue().splitlines() if ln.startswith('OVERALL')][0]
        self.assertIn('OVERALL FAIL', over)
        self.assertIn('macro FAIL: melee_weapons#1 (katana, blunt, baton, bat)', over)
        # without macro failures the same steps keep their verdict
        with tempfile.TemporaryDirectory() as d:
            plain = gt_run.write_outputs(d, td.plan_of(td.FIXED_A), list(rows.values()), lines, td.FIXED_A)
        self.assertEqual(plain['macro_failures'], [])
        self.assertEqual(plain['overall'], 'PASS' if not [r for r in rows.values() if r['verdict'] != 'PASS'] else 'FAIL')


class BlockedWeaponTests(unittest.TestCase):
    def test_parser_reads_both_kinds_of_line(self):
        ev = gn.blocked_weapon_events(BLOCK_VEHICLE + '\n' + 'noise\n' + BLOCK_NO_COMBAT)
        self.assertEqual(len(ev), 2)
        self.assertEqual(ev[0]['t'], datetime(2026, 11, 14, 18, 21, 21, 717000))
        self.assertEqual(ev[0]['cause'], 'vehicle scene')
        self.assertEqual(ev[1]['cause'], 'no combat')
        self.assertEqual(ev[0]['fields']['zones'], '1')

    def test_upper_body_reader(self):
        self.assertEqual(gn.latest_upper_body(POS_BLOCKED), 5)
        self.assertEqual(gn.latest_upper_body(POS_BLOCKED + '\n' + POS_FREE), 1)
        self.assertIsNone(gn.latest_upper_body('nothing'))

    def test_weapon_failures_after_the_block_become_invalid_and_earlier_passes_stay(self):
        recs = [session_record(), firearm_record()]
        marked = gn.mark_blocked(recs, gn.blocked_weapon_events(BLOCK_VEHICLE))
        w = recs[0]['weapons']
        self.assertEqual([w[t]['verdict'] for t in ('knife', 'katana', 'blunt', 'baton', 'bat')],
                         ['PASS', 'INVALID', 'INVALID', 'INVALID', 'INVALID'])
        self.assertEqual(w['katana']['invalid_cause'], 'vehicle scene')
        self.assertEqual(recs[0]['verdict'], 'INVALID')
        self.assertEqual(recs[0]['fails'], [])
        self.assertEqual(len(recs[0]['invalid_fails']), 4)
        self.assertEqual(recs[1]['verdict'], 'INVALID')          # the firearm macro came later, the block still held
        self.assertEqual([m['scenario'] for m in marked], ['melee_weapons#1', 'weapon_fire_reload#1'])
        self.assertEqual(gn.macro_failures(recs), [])
        inv = gn.macro_invalid(recs)
        self.assertEqual(inv[0]['macros'], ['katana', 'blunt', 'baton', 'bat'])
        self.assertIn('INVALID (weapon blocked by the game, not scored)', gn.macro_result_text([], inv))

    def test_without_a_block_line_the_failures_stay(self):
        recs = [session_record(), firearm_record()]
        self.assertEqual(gn.mark_blocked(recs, []), [])
        self.assertEqual(recs[0]['verdict'], 'FAIL')
        self.assertEqual(len(gn.macro_failures(recs)), 2)

    def test_a_failure_of_another_kind_stays_a_failure(self):
        rec = weapons_record({'katana': view('katana', 'FAIL', ['light_combo: mask never came on'],
                                             a='18:21:41.000', b='18:21:50.000')})
        gn.mark_blocked([rec], gn.blocked_weapon_events(BLOCK_VEHICLE))
        self.assertEqual(rec['weapons']['katana']['verdict'], 'FAIL')
        self.assertEqual(rec['verdict'], 'FAIL')

    def test_a_weapon_that_worked_ends_the_blocked_state(self):
        ok = view('knife', 'PASS', a='18:22:00.000', b='18:22:30.000')
        later = view('katana', 'FAIL', ['nothing in the weapon slot after the draw request'], a='18:23:00.000', b='18:23:20.000')
        rec = weapons_record({'knife': ok, 'katana': later}, end='18:23:30.000')
        gn.mark_blocked([rec], gn.blocked_weapon_events(BLOCK_VEHICLE))
        self.assertEqual(rec['weapons']['katana']['verdict'], 'FAIL')

    def test_recompute_after_a_second_try(self):
        rec = session_record()
        gn.mark_blocked([rec], gn.blocked_weapon_events(BLOCK_VEHICLE))
        for tag in ('katana', 'blunt', 'baton'):
            rec['weapons'][tag] = view(tag, 'PASS', a='18:25:00.000', b='18:25:30.000')
        self.assertEqual(gn.recompute_verdict(rec), 'INVALID')          # bat is still invalid
        rec['weapons']['bat'] = view('bat', 'PASS', a='18:26:00.000', b='18:26:30.000')
        self.assertEqual(gn.recompute_verdict(rec), 'PASS')
        self.assertEqual(rec['fails'], [])
        rec['weapons']['bat'] = view('bat', 'FAIL', ['light_combo: no strike'], a='18:26:00.000', b='18:26:30.000')
        self.assertEqual(gn.recompute_verdict(rec), 'FAIL')
        self.assertEqual(rec['fails'], ['bat: light_combo: no strike'])


class NightMacroTests(unittest.TestCase):
    def make(self):
        tmp = tempfile.mkdtemp()
        args = night.build_parser().parse_args(['--out-root', 'unused', '--log-dir', tmp])
        n = night.Night(args, runner=lambda c, **k: None)
        n.settings.pending_path = None
        n.utpp_log = os.path.join(tmp, 'utpp-2026-11-14-18-14-37.log')
        with open(n.utpp_log, 'w', encoding='utf-8') as f:
            f.write(BLOCK_VEHICLE + '\n' + POS_BLOCKED + '\n')
        n.scen_log_off = 0
        n.cycle = {'name': 't', 'steps': [], 'scenarios': [], 'weapon_blocks': []}
        n.weapon_blocks = []
        n.log = lambda msg: None
        n.scenario_deadline = None
        return n

    def test_result_line_names_the_failed_macros(self):
        n = self.make()
        n.cycle['scenarios'] = [session_record(), firearm_record()]
        n.cycle['result'] = 'ok'
        n.apply_macro_verdicts()
        self.assertEqual(n.cycle['result'],
                         'macro FAIL: melee_weapons#1 (katana, blunt, baton, bat); weapon_fire_reload#1')
        n.cycle['scenarios'] = [{'name': 'melee_fists', 'run': 1, 'verdict': 'PASS', 'fails': []}]
        n.cycle['result'] = 'ok'
        n.apply_macro_verdicts()
        self.assertEqual(n.cycle['result'], 'ok')
        self.assertEqual(n.macro_summary('failures'), [])

    def test_block_lifts_and_the_second_try_passes(self):
        n = self.make()
        rec = firearm_record()
        recs = [rec]
        n.wait_block_lift = lambda pad, guard: True
        tries = []

        def retry(pad, guard, plan, name, r, inv):
            tries.append(name)
            r.update(verdict='PASS', fails=[])
        n.retry_blocked = retry
        n.weapon_block_check(None, None, {}, 'weapon_fire_reload', 1, rec, recs)
        self.assertEqual(tries, ['weapon_fire_reload'])
        self.assertEqual(rec['verdict'], 'PASS')
        info = n.cycle['weapon_blocks'][0]
        self.assertTrue(info['lifted'] and info['retried'])
        self.assertEqual(info['cause'], 'vehicle scene')
        self.assertEqual(info['verdict_after'], 'PASS')

    def test_block_stays_one_second_try_and_the_macro_is_invalid_again(self):
        n = self.make()
        rec = session_record()
        n.cycle['scenarios'] = [rec]
        n.wait_block_lift = lambda pad, guard: False
        tries = []

        def retry(pad, guard, plan, name, r, inv):
            tries.append(inv['macros'])
            # the second try finds the weapon slot empty again: the same failure texts, later in the same blocked state
            for tag in inv['macros']:
                r['weapons'][tag] = view(tag, 'FAIL', ['nothing in the weapon slot after the draw request'],
                                         a='18:30:00.000', b='18:30:08.000')
            with open(n.utpp_log, 'a', encoding='utf-8') as f:
                f.write(BLOCK_VEHICLE.replace('18:21:21.717', '18:30:01.000') + '\n')
        n.retry_blocked = retry
        n.weapon_block_check(None, None, {}, 'melee_weapons', 1, rec, [rec])
        self.assertEqual(tries, [['katana', 'blunt', 'baton', 'bat']])                # exactly one second try
        self.assertEqual(rec['verdict'], 'INVALID')
        info = n.cycle['weapon_blocks'][0]
        self.assertEqual(info['parts'], ['katana', 'blunt', 'baton', 'bat'])
        self.assertFalse(info['lifted'])
        self.assertTrue(info['retried'])
        self.assertEqual(info['verdict_after'], 'INVALID')
        self.assertEqual(n.macro_summary('failures'), [])
        self.assertEqual(n.macro_summary('invalid')[0]['macros'], ['katana', 'blunt', 'baton', 'bat'])
        n.cycle['result'] = 'ok'
        n.apply_macro_verdicts()
        self.assertTrue(n.cycle['result'].startswith('INVALID (weapon blocked by the game'))

    def test_an_error_in_the_second_try_does_not_end_the_cycle(self):
        n = self.make()
        rec = firearm_record()
        n.wait_block_lift = lambda pad, guard: None
        pad = types.SimpleNamespace(release=lambda: None)

        def boom(*a):
            raise RuntimeError('console lost')
        n.retry_blocked = boom
        n.weapon_block_check(pad, None, {}, 'weapon_fire_reload', 1, rec, [rec])
        self.assertEqual(rec['verdict'], 'INVALID')
        self.assertIn('RuntimeError', n.cycle['weapon_blocks'][0]['retry_error'])

    def test_a_block_that_began_in_an_earlier_scenario_is_known_at_the_next_weapon_macro(self):
        n = self.make()
        n.scen_log_off = 0
        n.scan_weapon_blocks()                                   # called after the walking scenario that saw the line
        self.assertEqual(len(n.weapon_blocks), 1)
        with open(n.utpp_log, 'w', encoding='utf-8') as f:       # the next scenario's log part has no block line of its own
            f.write(POS_BLOCKED + '\n')
        rec = firearm_record()
        n.wait_block_lift = lambda pad, guard: False
        n.retry_blocked = lambda *a: None
        n.weapon_block_check(None, None, {}, 'weapon_fire_reload', 1, rec, [rec])
        self.assertEqual(rec['verdict'], 'INVALID')
        n.scan_weapon_blocks()
        self.assertEqual(len(n.weapon_blocks), 1)                # the same entry is not added twice

    def test_no_block_line_leaves_the_failure(self):
        n = self.make()
        with open(n.utpp_log, 'w', encoding='utf-8') as f:
            f.write(POS_FREE + '\n')
        rec = firearm_record()
        n.wait_block_lift = lambda pad, guard: self.fail('nothing to wait for')
        n.weapon_block_check(None, None, {}, 'weapon_fire_reload', 1, rec, [rec])
        self.assertEqual(rec['verdict'], 'FAIL')
        self.assertEqual(n.cycle['weapon_blocks'], [])


if __name__ == '__main__':
    unittest.main()
