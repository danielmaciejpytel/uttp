"""Offline tests of gt_collect.py (copying the session's pose recording, input dump and log into the run folder).

  python Source\\tools\\gametest\\test_gt_collect.py -v

A fake game log folder in a temp dir: segments appear, grow, are deleted by the ring, belong to an earlier session, or cannot be
read; nothing outside the temp dirs is touched.
"""
from __future__ import annotations

import os
import shutil
import sys
import tempfile
import unittest
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gt_collect as gc  # noqa: E402

SINCE = datetime(2026, 11, 14, 1, 43, 0)


def seg(minute: int, second: int = 37, ring: bool = True) -> str:
    return f'utpp-pose{"-ring" if ring else ""}-2026-11-14-01-{minute:02d}-{second:02d}.utpppose'


class CollectTests(unittest.TestCase):
    def setUp(self):
        self.src = tempfile.mkdtemp(prefix='gc_src_')
        self.dst = os.path.join(tempfile.mkdtemp(prefix='gc_dst_'), 'logs')
        self.msgs = []
        self.c = gc.SessionCollector(self.src, self.dst, SINCE, log=self.msgs.append)

    def tearDown(self):
        shutil.rmtree(self.src, ignore_errors=True)
        shutil.rmtree(os.path.dirname(self.dst), ignore_errors=True)

    def put(self, name, data=b'x' * 100):
        with open(os.path.join(self.src, name), 'wb') as f:
            f.write(data)

    def got(self, name):
        p = os.path.join(self.dst, name)
        if not os.path.isfile(p):
            return None
        with open(p, 'rb') as f:
            return f.read()

    def test_between_scenarios_only_finished_ring_segments(self):
        self.put(seg(43), b'a' * 10)
        self.put(seg(44), b'b' * 20)
        self.put(seg(45), b'c' * 30)                     # the newest one: the game still writes it
        out = self.c.collect('between scenarios')
        self.assertEqual(sorted(out['new']), sorted([seg(43), seg(44)]))
        self.assertEqual(self.got(seg(43)), b'a' * 10)
        self.assertIsNone(self.got(seg(45)))
        out = self.c.collect('between scenarios')       # nothing changed: nothing is copied again
        self.assertEqual((out['new'], out['refreshed']), ([], []))
        self.put(seg(46), b'd' * 5)                      # the next minute: the 45 segment is finished now
        out = self.c.collect('between scenarios')
        self.assertEqual(out['new'], [seg(45)])

    def test_copy_survives_the_ring_deleting_the_segment(self):
        self.put(seg(43), b'a' * 10)
        self.put(seg(44), b'b' * 10)
        self.c.collect('between scenarios')
        os.remove(os.path.join(self.src, seg(43)))       # the ring drops it after ten minutes
        self.c.collect('between scenarios')
        self.assertEqual(self.got(seg(43)), b'a' * 10)
        self.put(seg(45))
        self.c.collect('scenarios end', final=True)
        gone = self.c.summary()['source_gone_after_copy']
        self.assertIn(seg(43), gone)
        self.assertEqual(self.c.summary()['lost'], [])

    def test_final_copies_everything_and_the_last_copy_after_quit_is_longer(self):
        self.put(seg(43), b'a' * 10)
        self.put(seg(44), b'b' * 10)                     # the open segment, a block not flushed yet
        self.put('utpp-feed-2026-11-14-01-43-37.txt', b'f' * 7)
        out = self.c.collect('scenarios end', final=True)
        self.assertEqual(len(out['new']), 3)
        self.assertEqual(self.got(seg(44)), b'b' * 10)
        self.put(seg(44), b'b' * 10 + b'tail')           # the writer flushed its last block on close
        out = self.c.collect('collect', final=True)
        self.assertEqual(out['refreshed'], [seg(44)])
        self.assertEqual(self.got(seg(44)), b'b' * 10 + b'tail')
        ent = {e['name']: e for e in self.c.manifest()}
        self.assertEqual(ent[seg(44)]['bytes'], 14)
        self.assertEqual(ent[seg(44)]['first_phase'], 'scenarios end')
        self.assertEqual(ent[seg(44)]['phase'], 'collect')
        self.assertEqual(ent[seg(43)]['phase'], 'scenarios end')      # unchanged: not copied twice
        self.assertFalse([f for f in os.listdir(self.dst) if f.endswith('.part')])

    def test_earlier_session_segments_are_ignored(self):
        self.put(seg(38))                                # a segment of the previous session, still within the ten minutes
        self.put(seg(41, 12))
        self.put(seg(43))
        self.put('utpp-feed-2026-11-14-01-30-00.txt')
        self.c.collect('x', final=True)
        names = {e['name'] for e in self.c.manifest()}
        self.assertEqual(names, {seg(43)})

    def test_single_file_form_is_copied_at_the_end_only(self):
        self.put(seg(43, ring=False), b's' * 40)
        out = self.c.collect('between scenarios')
        self.assertEqual(out['new'], [])
        out = self.c.collect('scenarios end', final=True)
        self.assertEqual(out['new'], [seg(43, ring=False)])
        self.assertEqual(self.c.manifest()[0]['kind'], 'pose')
        self.assertEqual(gc.pose_files(self.dst), [os.path.join(self.dst, seg(43, ring=False))])

    def test_a_source_that_disappears_is_reported_not_raised(self):
        self.put(seg(43))
        self.put(seg(44))
        orig = self.c._candidates

        def stale():                                       # the listing saw the files, the game deleted one before the copy
            c = orig()
            os.remove(os.path.join(self.src, seg(43)))
            return c
        self.c._candidates = stale
        out = self.c.collect('between scenarios')
        self.c._candidates = orig
        self.assertIsNone(self.got(seg(43)))
        self.assertEqual(out['new'], [])               # stat failed: it is simply not there any more
        out = self.c.collect('final', final=True)
        self.assertEqual(out['lost'], [])

    def test_a_file_that_vanishes_during_the_copy_is_lost(self):
        self.put(seg(43))
        real_open = open

        def vanishing(path, mode='r', *a, **k):
            if str(path).endswith(seg(43)) and 'b' in mode and 'r' in mode:
                raise FileNotFoundError(path)
            return real_open(path, mode, *a, **k)
        gc.open = vanishing                                  # module-level shadow of the builtin
        try:
            self.put(seg(44))
            out = self.c.collect('final', final=True)
        finally:
            del gc.open
        self.assertEqual(out['lost'], [seg(43)])
        self.assertIn(seg(43), self.c.summary()['lost'])

    def test_a_file_the_game_holds_open_is_busy_not_an_error_and_is_copied_after_it_closed(self):
        self.put(seg(43, ring=False), b'open')
        self.put(seg(44), b'ring')
        real_open = open

        def sharing_violation(path, mode='r', *a, **k):
            if str(path).endswith(seg(43, ring=False)) and mode == 'rb':
                raise PermissionError(13, 'The process cannot access the file because it is being used by another process', None, 32)
            return real_open(path, mode, *a, **k)
        gc.open = sharing_violation
        try:
            out = self.c.collect('scenarios end', final=True)
        finally:
            del gc.open
        self.assertEqual(out['busy'], [seg(43, ring=False)])
        self.assertEqual(out['errors'], [])
        s = self.c.summary()
        self.assertEqual((s['errors'], s['busy']), ([], [seg(43, ring=False)]))
        self.assertTrue(any(e['status'] == 'busy' for e in self.c.manifest()))
        self.assertTrue(any('still open in the game' in m for m in self.msgs))
        out = self.c.collect('collect', final=True)                    # after the quit: the game closed it
        self.assertEqual(out['new'], [seg(43, ring=False)])
        self.assertEqual(self.c.summary()['busy'], [])
        self.assertEqual(self.got(seg(43, ring=False)), b'open')
        # a refresh that hits the sharing violation keeps the earlier copy and says it is stale
        self.put(seg(43, ring=False), b'open and more')
        gc.open = sharing_violation
        try:
            self.c.collect('again', final=True)
        finally:
            del gc.open
        self.assertEqual(self.c.summary()['stale_copies'], [seg(43, ring=False)])
        self.assertEqual(self.got(seg(43, ring=False)), b'open')

    def test_a_copy_error_does_not_raise_and_is_retried(self):
        os.mkdir(os.path.join(self.src, seg(43)))          # unreadable as a file
        out = self.c.collect('final', final=True)
        self.assertEqual(out['errors'], [seg(43)])
        self.assertEqual(self.c.summary()['errors'], [seg(43)])
        shutil.rmtree(os.path.join(self.src, seg(43)))
        self.put(seg(43), b'ok')
        out = self.c.collect('final', final=True)
        self.assertEqual(out['new'], [seg(43)])
        self.assertEqual(self.c.summary()['errors'], [])

    def test_gaps_between_copied_ring_segments_are_reported(self):
        for m in (43, 44, 47, 48):                           # 45 and 46 were dropped by the ring before they were copied
            self.put(seg(m))
        self.c.collect('final', final=True)
        gaps = self.c.gaps()
        self.assertEqual(len(gaps), 1)
        self.assertEqual((gaps[0]['after'], gaps[0]['before']), (seg(44), seg(47)))
        self.assertEqual(gaps[0]['missing_s'], 120.0)

    def test_log_files_come_from_the_extra_listing(self):
        log = os.path.join(self.src, 'utpp-2026-11-14-01-42-41.log')
        with open(log, 'w') as f:
            f.write('log')
        c = gc.SessionCollector(self.src, self.dst, SINCE, extra_files=lambda: [log, os.path.join(self.src, 'missing.log')], log=self.msgs.append)
        c.collect('between scenarios')
        self.assertEqual(c.manifest(), [])                   # not between scenarios
        c.collect('final', final=True)
        self.assertEqual([e['name'] for e in c.manifest()], ['utpp-2026-11-14-01-42-41.log'])
        self.assertEqual(gc.session_since(log, None), datetime(2026, 11, 14, 1, 42, 41))
        self.assertEqual(gc.session_since(None, SINCE), SINCE)

    def test_cli_collects_once(self):
        self.put(seg(43))
        out = os.path.join(os.path.dirname(self.dst), 'cli')
        self.assertEqual(gc.main([self.src, out, '--since', '2026-11-14 01:43:00']), 0)
        self.assertTrue(os.path.isfile(os.path.join(out, seg(43))))


if __name__ == '__main__':
    unittest.main()
