"""Unit tests of foot_pin.pin_offsets: the whole-span contact (known bug) and the other contact branches (no GPU, no data).

  Tools\\ai-motion\\envs\\kimodo\\Scripts\\python.exe -I Source\\tools\\ai_motion\\test_foot_pin_span.py [-v]

Whole-span contact: for a contact over the WHOLE span pin_offsets must not multiply the whole offset by (1 - w), because the
pinned pivot would still drift mid-span (a planted foot slides on; a uniform slide gets an ease-in and ends almost twice as
fast). Intended behaviour (foot_pin docstring: in-contact motion cancelled, frames 0 and T-1 exact): integrate forward, then
remove only the END residual linearly, i.e. the pivot moves at the constant residual speed (P[T-1] - P[0]) / (T - 1).

With an older foot_pin that scales the whole offset, the two WholeSpan tests SKIP, but only when pin_offsets returns exactly
that result; any other mismatch FAILs. The Branch tests guard the other branches (start / end / interior contacts) on both versions.
Environment:
  UTPP_FOOT_PIN_STRICT=1      the known-bug skip becomes a failure
  UTPP_FOOT_PIN_PATH=<file>   test another copy of foot_pin.py (e.g. a patched copy) instead of the sibling file
"""
import importlib.util
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
FOOT_PIN = os.environ.get('UTPP_FOOT_PIN_PATH') or os.path.join(HERE, 'foot_pin.py')
STRICT = os.environ.get('UTPP_FOOT_PIN_STRICT', '').strip() not in ('', '0')
ATOL = 1e-12
CM = 0.01


def _load(path):
    spec = importlib.util.spec_from_file_location('foot_pin_under_test', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


FP = _load(FOOT_PIN)
if os.environ.get('UTPP_FOOT_PIN_PATH'):
    print('foot_pin under test: %s' % FOOT_PIN, file=sys.stderr)

PIV = 1  # pivot joint of every track; joints 0 and 2 are far-away sentinels that pin_offsets must not read


def track(x_cm, z_cm=None, bob_cm=0.0):
    """P [T, 3, 3] (y up): the pivot at horizontal (x, z) in cm per frame, an optional vertical bob; piv = PIV."""
    x = np.asarray(x_cm, dtype=float) * CM
    z = np.zeros_like(x) if z_cm is None else np.asarray(z_cm, dtype=float) * CM
    P = np.full((len(x), 3, 3), 7.0)
    P[:, PIV, 0] = x
    P[:, PIV, 1] = bob_cm * CM * np.sin(np.arange(len(x)))
    P[:, PIV, 2] = z
    return P, np.full(len(x), PIV)


def offsets(x_cm, z_cm=None):
    """Expected horizontal offsets (cm per frame for x and z) -> [T, 3] in m with y = 0."""
    x = np.asarray(x_cm, dtype=float)
    z = np.zeros_like(x) if z_cm is None else np.asarray(z_cm, dtype=float)
    return np.stack([x, np.zeros_like(x), z], 1) * CM


class WholeSpan(unittest.TestCase):
    """A contact over the whole span (mask all True): forward integration, end residual removed linearly."""

    def check(self, got, fixed, buggy, what):
        if np.allclose(got, fixed, rtol=0, atol=ATOL):
            return
        if not np.allclose(fixed, buggy, rtol=0, atol=ATOL) and np.allclose(got, buggy, rtol=0, atol=ATOL):
            msg = ('KNOWN BUG (%s): foot_pin.pin_offsets scales the whole-span offset by (1 - w), mid-span drift is left; '
                   'foot_pin under test: %s' % (what, FOOT_PIN))
            if STRICT:
                self.fail(msg)
            self.skipTest(msg)
        np.testing.assert_allclose(got, fixed, rtol=0, atol=ATOL, err_msg='%s (foot_pin under test: %s)' % (what, FOOT_PIN))

    def test_planted_after_slide_keeps_constant_residual_speed(self):
        # the pivot slides 4 cm (x) / -2 cm (z) in frames 0..4, then stays planted to frame 8; vertical bob ignored
        x = [0, 1, 2, 3, 4, 4, 4, 4, 4]
        z = [0, -0.5, -1, -1.5, -2, -2, -2, -2, -2]
        P, piv = track(x, z, bob_cm=0.3)
        T = len(x)
        got = FP.pin_offsets(P, piv, np.ones(T, bool))
        # forward offset c = -(x - x0); fixed: c - w * c[-1] (w = f / 8) -> the pivot moves 0.5 cm (x) per frame
        fixed = offsets([0, -0.5, -1, -1.5, -2, -1.5, -1, -0.5, 0], [0, 0.25, 0.5, 0.75, 1, 0.75, 0.5, 0.25, 0])
        w = np.linspace(0, 1, T)[:, None]
        buggy = offsets(-np.array(x, float), -np.array(z, float)) * (1 - w)
        self.check(got, fixed, buggy, 'slide then planted')
        pinned = P[:, PIV] + got
        steps = np.diff(pinned[:, [0, 2]], axis=0)
        np.testing.assert_allclose(steps, np.tile([[0.5 * CM, -0.25 * CM]], (T - 1, 1)), rtol=0, atol=ATOL)
        np.testing.assert_allclose(got[[0, -1]], 0.0, rtol=0, atol=ATOL)  # the span's context/target frames stay exact
        np.testing.assert_allclose(got[:, 1], 0.0, rtol=0, atol=ATOL)     # only horizontal offsets

    def test_uniform_slide_is_left_unchanged(self):
        # a uniform slide over the whole span cannot be reduced with both ends fixed: the offset must stay 0
        x = [0.5 * f for f in range(9)]
        P, piv = track(x, [-0.25 * f for f in range(9)])
        T = len(x)
        got = FP.pin_offsets(P, piv, np.ones(T, bool))
        fixed = np.zeros((T, 3))
        w = np.linspace(0, 1, T)[:, None]
        buggy = offsets(-np.array(x), [0.25 * f for f in range(9)]) * (1 - w)
        self.check(got, fixed, buggy, 'uniform slide')


class Branch(unittest.TestCase):
    """The other contact branches: unchanged by the whole-span fix (expected on the current and on the patched file)."""

    def run_case(self, x, mask, want):
        P, piv = track(x)
        got = FP.pin_offsets(P, piv, np.array(mask, bool))
        np.testing.assert_allclose(got, offsets(want), rtol=0, atol=ATOL, err_msg='foot_pin under test: %s' % FOOT_PIN)

    def test_contact_touching_start(self):
        # forward from frame 0 inside 0..3, then linear to 0 at the last frame
        self.run_case([0, 1, 2, 3, 3.5, 4, 5, 6, 7], [1, 1, 1, 1, 0, 0, 0, 0, 0],
                      [0, -1, -2, -3, -2.4, -1.8, -1.2, -0.6, 0])

    def test_contact_touching_end(self):
        # backward from the last frame inside 5..8, linear from 0 at frame 0
        self.run_case([0, 0, 0, 0, 0, 1, 2, 3, 4], [0, 0, 0, 0, 0, 1, 1, 1, 1],
                      [0, 0.6, 1.2, 1.8, 2.4, 3, 2, 1, 0])

    def test_interior_contact(self):
        # forward from the contact start (offset 0 there), both span ends 0, gaps linear
        self.run_case(list(range(9)), [0, 0, 1, 1, 1, 1, 0, 0, 0],
                      [0, 0, 0, -1, -2, -3, -2, -1, 0])

    def test_contacts_at_both_ends(self):
        # start contact forward, end contact backward, the gap between them linear
        self.run_case(list(range(9)), [1, 1, 1, 0, 0, 0, 1, 1, 1],
                      [0, -1, -2, -1, 0, 1, 2, 1, 0])

    def test_no_contact(self):
        self.run_case(list(range(9)), [0] * 9, [0] * 9)


if __name__ == '__main__':
    unittest.main()
