"""The runner keeps V near the place where the session began on foot: gt_night.home_cmd / home_distance and
night.Night.note_home / keep_near_home (the scenarios only move V forward, so a long session drifts away from its start)."""
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gt_night as gn  # noqa: E402
import night  # noqa: E402

HOME = (1574.85, 2258.16, 182.53)


def pos(x, y, z=182.5, mounted=False):
    return {'p': (x, y, z), 'cf': (0.0, 1.0), 'v': None, 'vr': None, 'src': None, 'weapon': False, 'mounted': mounted}


class HomeHelpers(unittest.TestCase):
    def test_command_teleports_to_the_point_a_little_above_and_echoes(self):
        cmd = gn.home_cmd(HOME)
        self.assertIn('GetTeleportationFacility():Teleport(p,Vector4.new(1574.85,2258.16,182.83,1)', cmd)
        self.assertIn('p:GetWorldYaw()', cmd)                      # the facing stays
        self.assertIn("'[night] HOME ok='", cmd)
        self.assertNotIn('\n', cmd)

    def test_distance_is_horizontal_and_none_when_unknown(self):
        self.assertAlmostEqual(gn.home_distance(pos(1577.85, 2262.16, 250.0), HOME), 5.0, places=4)
        self.assertIsNone(gn.home_distance(None, HOME))
        self.assertIsNone(gn.home_distance(pos(1.0, 2.0), None))


class KeepNearHome(unittest.TestCase):
    def make(self, radius='60'):
        tmp = tempfile.mkdtemp()
        args = night.build_parser().parse_args(['--out-root', 'unused', '--log-dir', tmp, '--home-radius', radius])
        n = night.Night(args, runner=lambda c, **k: None)
        n.settings.pending_path = None
        n.cycle = {'name': 't', 'steps': [], 'scenarios': []}
        n.lines = []
        n.log = n.lines.append
        n.scenario_deadline = None
        n.auto_pos = True       # positions come from the log lines of the game
        n.sent = []
        n.answers = []          # positions returned by poll_pos, in order
        n.poll_pos = lambda guard, pad=None, timeout=None: n.answers.pop(0) if n.answers else None

        def console_verified(pad, guard, cmd, echo, label, timeout=3.0):
            n.sent.append(cmd)
            return n.reply
        n.console_verified = console_verified
        n.reply = '[night] HOME ok=true nil'
        return n

    class Guard:
        def __init__(self):
            self.waited = 0.0

        def wait(self, s):
            self.waited += s

    def test_start_point_is_taken_on_foot_only(self):
        n = self.make()
        n.answers = [pos(HOME[0], HOME[1], HOME[2], mounted=True)]
        n.note_home(None, self.Guard())
        self.assertIsNone(n.home)
        n.answers = [pos(*HOME)]
        n.note_home(None, self.Guard())
        self.assertEqual(n.home, HOME)
        self.assertEqual(n.cycle['home'], [1574.85, 2258.16, 182.53])

    def test_without_logged_positions_nothing_is_done(self):
        n = self.make()
        n.auto_pos = False
        n.answers = [pos(*HOME)]
        n.note_home(None, self.Guard())
        self.assertIsNone(n.home)
        self.assertEqual(len(n.answers), 1)                       # no position was asked for
        n.home = HOME
        n.answers = [pos(HOME[0] + 500.0, HOME[1])]
        self.assertFalse(n.keep_near_home(None, self.Guard(), {}, 'before x'))
        self.assertEqual((n.sent, len(n.answers)), ([], 1))

    def test_the_start_point_is_taken_later_when_the_first_read_saw_v_mounted(self):
        n = self.make()
        n.answers = [pos(HOME[0], HOME[1], HOME[2], mounted=True)]
        n.note_home(None, self.Guard())
        self.assertIsNone(n.home)
        n.answers = [pos(*HOME)]                                  # the first check before a weapon: on foot now
        self.assertFalse(n.keep_near_home(None, self.Guard(), {}, 'before handgun'))
        self.assertEqual(n.home, HOME)
        n.answers = [pos(HOME[0] + 100.0, HOME[1]), pos(*HOME)]
        self.assertTrue(n.keep_near_home(None, self.Guard(), {}, 'before revolver'))

    def test_radius_zero_switches_it_off(self):
        n = self.make('0')
        n.answers = [pos(*HOME)]
        n.note_home(None, self.Guard())
        self.assertIsNone(n.home)
        n.home = HOME
        n.answers = [pos(HOME[0] + 500.0, HOME[1])]
        self.assertFalse(n.keep_near_home(None, self.Guard(), {}, 'before x'))
        self.assertEqual(n.sent, [])

    def test_near_the_start_point_nothing_is_sent(self):
        n = self.make()
        n.home = HOME
        n.answers = [pos(HOME[0] + 59.0, HOME[1])]
        self.assertFalse(n.keep_near_home(None, self.Guard(), {}, 'before x'))
        self.assertEqual(n.sent, [])
        self.assertNotIn('home_returns', n.cycle)

    def test_far_from_the_start_point_v_is_put_back(self):
        n = self.make()
        n.home = HOME
        n.last_known = 'stale'
        n.answers = [pos(HOME[0] + 20.0, HOME[1] - 177.0), pos(HOME[0] + 0.2, HOME[1])]
        g = self.Guard()
        self.assertTrue(n.keep_near_home(None, g, {}, 'before smg'))
        self.assertEqual(n.sent, [gn.home_cmd(HOME)])
        self.assertEqual(g.waited, gn.HOME_SETTLE_S)
        info = n.cycle['home_returns'][0]
        self.assertEqual((info['why'], info['ok'], info['dist_m'], info['after_m']), ('before smg', True, 178.1, 0.2))
        self.assertIsNone(n.last_known)
        self.assertTrue(any('put back' in line for line in n.lines))

    def test_mounted_or_unknown_position_is_left_alone(self):
        n = self.make()
        n.home = HOME
        n.answers = [pos(HOME[0] + 300.0, HOME[1], mounted=True)]
        self.assertFalse(n.keep_near_home(None, self.Guard(), {}, 'before x'))
        n.answers = [None]
        self.assertFalse(n.keep_near_home(None, self.Guard(), {}, 'before x'))
        self.assertEqual(n.sent, [])

    def test_force_uses_five_metres(self):
        n = self.make()
        n.home = HOME
        n.answers = [pos(HOME[0] + 4.0, HOME[1])]
        self.assertFalse(n.keep_near_home(None, self.Guard(), {}, 'weapon blocked', force=True))
        n.answers = [pos(HOME[0] + 30.0, HOME[1]), pos(*HOME)]
        self.assertTrue(n.keep_near_home(None, self.Guard(), {}, 'weapon blocked', force=True))
        self.assertEqual(len(n.sent), 1)

    def test_a_command_that_did_not_run_is_reported(self):
        n = self.make()
        n.home = HOME
        for reply in (None, '[night] HOME ok=false some error'):
            n.reply = reply
            n.answers = [pos(HOME[0] + 100.0, HOME[1])]
            self.assertFalse(n.keep_near_home(None, self.Guard(), {}, 'before x'))
        self.assertEqual([r['ok'] for r in n.cycle['home_returns']], [False, False])
        self.assertTrue(all('did not run' in line for line in n.lines))

    def test_still_far_after_the_teleport_is_not_ok(self):
        n = self.make()
        n.home = HOME
        n.answers = [pos(HOME[0] + 100.0, HOME[1]), pos(HOME[0] + 100.0, HOME[1])]
        self.assertFalse(n.keep_near_home(None, self.Guard(), {}, 'before x'))
        self.assertEqual(n.cycle['home_returns'][0]['after_m'], 100.0)


class HomeAt(unittest.TestCase):
    """--home-at X,Y,Z: V is put at a known spot before the start point is taken (a scenario that needs a curb, a step)."""
    make = KeepNearHome.make
    Guard = KeepNearHome.Guard

    def make_at(self, text):
        n = self.make()
        n.args.home_at = text
        return n

    def test_parse_and_validation(self):
        self.assertIsNone(night.parse_home_at(''))
        self.assertEqual(night.parse_home_at(' 1583.5, 2251,181.7 '), (1583.5, 2251.0, 181.7))
        for bad in ('1,2', '1,2,3,4', 'a,b,c', 'nan,1,2'):
            with self.assertRaises(ValueError):
                night.parse_home_at(bad)
        args = night.build_parser().parse_args(['--home-at', '1,2'])
        self.assertTrue(any('--home-at' in e for e in night.validate_args(args)))
        args = night.build_parser().parse_args(['--home-at', '1583.5,2251,181.7'])
        self.assertFalse(any('--home-at' in e for e in night.validate_args(args)))

    def test_without_the_option_nothing_is_sent(self):
        n = self.make_at('')
        self.assertFalse(n.move_to_home_at(None, self.Guard()))
        self.assertEqual(n.sent, [])
        self.assertNotIn('home_at', n.cycle)

    def test_v_is_teleported_there_and_that_place_becomes_the_start_point(self):
        n = self.make_at('1583.5,2251,181.7')
        n.last_known = 'stale'
        g = self.Guard()
        self.assertTrue(n.move_to_home_at(None, g))
        self.assertEqual(n.sent, [gn.home_cmd((1583.5, 2251.0, 181.7))])
        self.assertEqual(n.cycle['home_at'], {'target': [1583.5, 2251.0, 181.7], 'ok': True})
        self.assertIsNone(n.last_known)
        self.assertGreaterEqual(g.waited, gn.HOME_SETTLE_S)
        n.answers = [pos(1583.6, 2251.1, 181.7)]
        n.note_home(None, g)
        self.assertEqual(n.home, (1583.6, 2251.1, 181.7))

    def test_a_lost_teleport_is_reported_not_retried(self):
        n = self.make_at('1583.5,2251,181.7')
        n.reply = None
        self.assertFalse(n.move_to_home_at(None, self.Guard()))
        self.assertEqual(len(n.sent), 1)
        self.assertFalse(n.cycle['home_at']['ok'])
        self.assertTrue(any('did not run' in ln for ln in n.lines))


class SessionOverrides(unittest.TestCase):
    def test_shooter_mode_and_stride_warp_replace_their_pinned_values(self):
        p = night.build_parser()
        self.assertEqual(night.session_overrides(p.parse_args([])), {})
        self.assertEqual(night.session_overrides(p.parse_args(['--shooter-mode', 'on'])), {'bodyturn.shooterMode': True})
        self.assertEqual(night.session_overrides(p.parse_args(['--shooter-mode', 'off', '--stride-warp', 'off'])),
                         {'bodyturn.shooterMode': False, 'mm.strideWarp': False})
        import gt_settings as gs
        plan = dict(gs.session_plan('shipped', night.session_overrides(p.parse_args(['--shooter-mode', 'on']))))
        self.assertIs(plan['bodyturn.shooterMode'], True)
        self.assertIs(plan['mm.firmFeet'], True)

    def test_curb_set_resolves_to_a_scenario_of_the_file(self):
        scen = night.load_scenarios(os.path.join(HERE, 'scenarios.json'))
        names = gn.resolve_sets('curb')
        self.assertEqual(names, ['curb_spoke'])
        self.assertEqual(gn.validate_sets(names, scen), [])
        self.assertNotIn('curb_spoke', gn.SETS['all'])


if __name__ == '__main__':
    unittest.main()
