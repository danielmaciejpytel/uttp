"""Unit tests of graphics_profile.py (no game, temp files only)."""
import copy
import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import graphics_profile as gp  # noqa: E402

DOC = {
    'version': 1,
    'data': [
        {'group_name': '/graphics/advanced', 'options': [
            {'name': 'AmbientOcclusion', 'type': 'string_list', 'value': 'High', 'values': ['Off', 'Low', 'Medium', 'High'],
             'index': 3, 'default_index': 1},
            {'name': 'ContactShadows', 'type': 'bool', 'value': True, 'default_value': True},
            {'name': 'Anisotropy', 'type': 'int_list', 'value': 16, 'values': [1, 4, 8, 16], 'index': 3},
            {'name': 'LODPreset', 'type': 'string_list', 'value': 'Low', 'values': ['Low', 'Medium', 'High'], 'index': 0},
            {'name': 'AntiAliasing', 'type': 'string_list', 'value': '', 'index': -1}]},
        {'group_name': '/graphics/presets', 'options': [
            {'name': 'TextureQuality', 'type': 'string_list', 'value': 'High', 'values': ['Low', 'Medium', 'High'], 'index': 2},
            {'name': 'DLSS', 'type': 'string_list', 'value': 'Auto', 'index': 0}]},
        {'group_name': '/audio/volume', 'options': [{'name': 'MasterVolume', 'type': 'int', 'value': 70}]},
    ],
}


class ProfileTests(unittest.TestCase):
    def test_apply_sets_lowest_and_remembers_previous(self):
        doc = copy.deepcopy(DOC)
        prev, lines = gp.apply_profile(doc)
        o = gp.options(doc)
        self.assertEqual((o[('/graphics/advanced', 'AmbientOcclusion')]['value'], o[('/graphics/advanced', 'AmbientOcclusion')]['index']),
                         ('Off', 0))
        self.assertIs(o[('/graphics/advanced', 'ContactShadows')]['value'], False)
        self.assertEqual(o[('/graphics/advanced', 'Anisotropy')]['value'], 1)
        self.assertEqual(o[('/graphics/presets', 'TextureQuality')]['value'], 'Low')
        self.assertEqual(prev['/graphics/advanced|AmbientOcclusion'], {'value': 'High', 'index': 3})
        self.assertNotIn('/graphics/advanced|LODPreset', prev)                  # already lowest: not remembered
        self.assertEqual(o[('/graphics/presets', 'DLSS')]['value'], 'Auto')      # not in the profile
        self.assertEqual(o[('/audio/volume', 'MasterVolume')]['value'], 70)

    def test_restore_writes_back_only_the_changed_options(self):
        doc = copy.deepcopy(DOC)
        prev, _ = gp.apply_profile(doc)
        gp.options(doc)[('/audio/volume', 'MasterVolume')]['value'] = 40        # changed meanwhile by the user / game
        gp.restore_profile(doc, prev)
        want = copy.deepcopy(DOC)
        gp.options(want)[('/audio/volume', 'MasterVolume')]['value'] = 40
        self.assertEqual(doc, want)

    def test_cli_apply_twice_then_restore(self):
        with tempfile.TemporaryDirectory() as d:
            settings = os.path.join(d, 'UserSettings.json')
            with open(settings, 'w', encoding='utf-8') as f:
                json.dump(DOC, f, indent=4)
            old = gp.game_running
            gp.game_running = lambda: False
            try:
                args = ['--settings', settings, '--state-dir', os.path.join(d, 'state')]
                self.assertEqual(gp.main(['--apply'] + args), 0)
                self.assertEqual(gp.main(['--apply'] + args), 0)                 # second apply keeps the first "previous"
                self.assertEqual(gp.load(settings)['data'][0]['options'][0]['value'], 'Off')
                self.assertEqual(gp.main(['--restore'] + args), 0)
                self.assertEqual(gp.load(settings), DOC)
                self.assertEqual(gp.main(['--restore'] + args), 0)               # nothing to restore: no change
                self.assertEqual(gp.load(settings), DOC)
                self.assertTrue(os.path.exists(os.path.join(d, 'state', 'UserSettings.before.json')))
            finally:
                gp.game_running = old

    def test_refuses_while_the_game_runs(self):
        with tempfile.TemporaryDirectory() as d:
            settings = os.path.join(d, 'UserSettings.json')
            with open(settings, 'w', encoding='utf-8') as f:
                json.dump(DOC, f)
            old = gp.game_running
            gp.game_running = lambda: True
            try:
                self.assertEqual(gp.main(['--apply', '--settings', settings, '--state-dir', os.path.join(d, 's')]), 3)
                self.assertEqual(gp.load(settings), DOC)
            finally:
                gp.game_running = old


if __name__ == '__main__':
    unittest.main()
