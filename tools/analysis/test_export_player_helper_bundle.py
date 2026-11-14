"""Checks of the helper bundle exporter and the rig signature tool: closure rule for descendants, signature consistency and refusal
of a bundle whose rig signature differs from the expected one. The tests that need the player deformer graph or a recorded merged rig
are skipped when that data is absent."""
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import helper_rig_signature as S  # noqa: E402
import export_player_helper_bundle as X  # noqa: E402

ROOT = HERE.parents[2]
MERGED_MAN = ROOT / 'Build' / 'test-data' / 'body_helper_man' / 'merged_rig.json'
WRIST = ['l_Wrist_2_JNT', 'l_wrist_A_mscl_JNT', 'l_wrist_B_mscl_JNT', 'l_wrist_C_mscl_JNT']


def chain(n):
    """Synthetic rig: bone i has hash 1000+i and parent i-1."""
    return [1000 + i for i in range(n)], [i - 1 for i in range(n)]


def have_graph(body='man'):
    try:
        return all(Path(p).exists() for p in X.E.BODY_FILES[body])
    except Exception:
        return False


class SignatureTest(unittest.TestCase):
    def test_wrong_fingerprint_is_refused(self):
        hashes, parents = chain(8)
        bones, fp = S.closure_signature(hashes, parents, [1003])
        self.assertEqual(bones, 4)
        self.assertTrue(S.check(hashes, parents, [1003], bones, fp))
        self.assertFalse(S.check(hashes, parents, [1003], bones, fp ^ 1))
        self.assertFalse(S.check(hashes, parents, [1003], bones + 1, fp))

    def test_layout_change_of_a_bundle_bone_changes_the_fingerprint(self):
        hashes, parents = chain(8)
        bones, fp = S.closure_signature(hashes, parents, [1003])
        moved = list(parents)
        moved[3] = 0
        self.assertFalse(S.check(hashes, moved, [1003], bones, fp))
        other = list(parents)
        other[6] = 2  # a bone outside the closure: no effect
        self.assertTrue(S.check(hashes, other, [1003], bones, fp))

    def test_cli_expect_refuses_a_wrong_fingerprint(self):
        hashes, parents = chain(8)
        bones, fp = S.closure_signature(hashes, parents, [1003])
        with tempfile.TemporaryDirectory() as d:
            rig = Path(d) / 'rig.json'
            rig.write_text(json.dumps({'hashes': ['%016X' % h for h in hashes], 'parents': parents}))
            binding = Path(d) / 'binding.dat'
            binding.write_text('UTPP_HELPER_BINDING 1\nOWNED %016X 0\nEND\n' % 1003)
            run = lambda *extra: subprocess.run([sys.executable, '-I', str(HERE / 'helper_rig_signature.py'), str(rig), str(binding), *extra],  # noqa: E731
                                                capture_output=True, text=True)
            ok = run('--expect', str(bones), '%x' % fp)
            self.assertEqual(ok.returncode, 0, ok.stdout + ok.stderr)
            bad = run('--expect', str(bones), '%x' % (fp ^ 0x10))
            self.assertEqual(bad.returncode, 1)
            self.assertIn('bundle refused', bad.stdout)


@unittest.skipUnless(have_graph('man'), 'player deformer graph not available')
class ClosureTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ev = X.E.Evaluator(body='man')

    def test_owned_parent_without_its_helper_children_is_refused(self):
        with self.assertRaises(ValueError):
            X.check_closure(self.ev, ['l_Wrist_2_JNT'])
        with self.assertRaises(ValueError):
            X.check_closure(self.ev, WRIST[:3])
        self.assertEqual(X.check_closure(self.ev, WRIST), [])
        self.assertEqual(X.check_closure(self.ev, WRIST[1:]), [])  # children may be owned without their parent

    def test_core_bone_is_refused(self):
        with self.assertRaises(ValueError):
            X.check_closure(self.ev, ['LeftHand'])

    def test_merged_rig_helper_child_must_be_owned(self):
        # merged rig: Spine3 <- a helper of the graph <- another helper of the graph (not a child in the deformer rig)
        a, b = 'l_lowLeg_back_A_mscl_JNT', 'l_lowLeg_back_B_mscl_JNT'
        hashes = [X.fnv64('Spine3'), X.fnv64(a), X.fnv64(b), 0x1234]
        with self.assertRaises(ValueError):
            X.check_closure(self.ev, [a], (hashes, [-1, 0, 1, 1]))
        # a merged-rig child the graph does not drive is compensated
        self.assertEqual(X.check_closure(self.ev, [a, b], (hashes, [-1, 0, 1, 1])), [0x1234])

    @unittest.skipUnless(MERGED_MAN.exists(), 'recorded merged rig not available')
    def test_export_signature_matches_the_tool_and_a_wrong_expectation_refuses(self):
        exporter = HERE / 'export_player_helper_bundle.py'
        with tempfile.TemporaryDirectory() as d:
            owned = Path(d) / 'owned.txt'
            owned.write_text('\n'.join(WRIST) + '\n')
            out = Path(d) / 'bundle'
            r = subprocess.run([sys.executable, '-I', str(exporter), '--body', 'man', '--owned', str(owned), '--merged-rig', str(MERGED_MAN),
                                '--out-dir', str(out), '--manifest', str(Path(d) / 'm.json')], capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, r.stderr)
            sig = json.loads((Path(d) / 'm.json').read_text())['rig_signature']
            rig = json.loads(MERGED_MAN.read_text())
            bones, fp = S.closure_signature([int(h, 16) for h in rig['hashes']], rig['parents'], S.binding_roots(out / 'binding.dat'))
            self.assertEqual((sig['closure_bones'], sig['closure_fingerprint']), (bones, '0x%016x' % fp))
            refused = Path(d) / 'refused'
            r = subprocess.run([sys.executable, '-I', str(exporter), '--body', 'man', '--owned', str(owned), '--merged-rig', str(MERGED_MAN),
                                '--out-dir', str(refused), '--expect-closure', str(bones), '0x%016x' % (fp ^ 1)], capture_output=True, text=True)
            self.assertNotEqual(r.returncode, 0)
            self.assertIn('Bundle refused', r.stderr)
            self.assertFalse(refused.exists(), 'a refused bundle writes no files')
            r = subprocess.run([sys.executable, '-I', str(exporter), '--body', 'man', '--owned', str(owned), '--merged-rig', str(MERGED_MAN),
                                '--out-dir', str(refused), '--expect-closure', str(bones), '0x%016x' % fp], capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, r.stderr)
            self.assertEqual((refused / 'binding.dat').read_bytes(), (out / 'binding.dat').read_bytes())


if __name__ == '__main__':
    unittest.main()
