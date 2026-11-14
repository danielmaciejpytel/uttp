"""Offline tests for the window capture (gt_capture.py) and its use in the night runner. No game, no capture, no
ffmpeg is started: commands are built and fake processes stand in for ffmpeg.

  python Source\\tools\\gametest\\test_gt_capture.py -v
"""
from __future__ import annotations

import io
import json
import os
import sys
import tempfile
import unittest
from datetime import datetime, timedelta
from types import SimpleNamespace
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gt_capture as gc  # noqa: E402
import gt_night as gn  # noqa: E402
import night  # noqa: E402

FILTER_HELP = 'Filter gfxcapture\n  Capture graphics/screen content as a video source\ngfxcapture AVOptions:\n'


def fake_runner(has_filter: bool = True, nvenc: bool = True, calls: list | None = None):
    """Stands in for subprocess.run: answers the filter probe and the encoder probe of the night runner."""
    def run(cmd, **kw):
        if calls is not None:
            calls.append(list(cmd))
        text = ' '.join(map(str, cmd))
        if 'filter=gfxcapture' in text:
            out = FILTER_HELP if has_filter else "Unknown filter 'gfxcapture'.\n"
            return SimpleNamespace(returncode=0 if has_filter else 1, stdout=out, stderr='')
        if '-c:v h264_nvenc' in text:
            return SimpleNamespace(returncode=0 if nvenc else 1, stdout='', stderr='')
        return SimpleNamespace(returncode=0, stdout='', stderr='')
    return run


class CommandTests(unittest.TestCase):
    def test_output_size(self):
        self.assertEqual(gc.output_size(3840, 2160), (1920, 1080))
        self.assertEqual(gc.output_size(1920, 1080), (1920, 1080))
        self.assertEqual(gc.output_size(1280, 720), (1280, 720))                 # never enlarged
        self.assertEqual(gc.output_size(3440, 1440), (1920, 802))                # aspect kept
        self.assertEqual(gc.output_size(1001, 563), (1000, 562))                 # even (yuv420p)
        self.assertEqual(gc.output_size(2560, 1600), (1728, 1080))
        with self.assertRaises(ValueError):
            gc.output_size(0, 100)

    def test_nvenc_command(self):
        c = gc.build_cmd('ff8.exe', 'o.mp4', 12345, (3840, 2160), 30)
        i = c[c.index('-i') + 1]
        self.assertEqual(c[c.index('-f') + 1], 'lavfi')
        self.assertTrue(i.startswith('gfxcapture=hwnd=12345:'))                  # by handle, never by title regex
        for part in ('capture_cursor=0', 'display_border=0', 'capture_border=0', 'max_framerate=120', 'width=1920',
                     'height=1080', 'resize_mode=scale'):
            self.assertIn(part, i)
        self.assertIn('setparams=range=tv:colorspace=bt470bg', i)
        self.assertNotIn('window_title', i)
        self.assertEqual(c[c.index('-c:v') + 1], 'h264_nvenc')
        self.assertNotIn('-pix_fmt', c)                                           # GPU frames go to the encoder as they are
        self.assertNotIn('hwdownload', i)
        self.assertEqual(c[c.index('-bf') + 1], '0')
        self.assertEqual(c[c.index('-r') + 1], '30')
        self.assertEqual(c[c.index('-progress') + 1], 'pipe:1')
        self.assertEqual(c[c.index('-movflags') + 1], '+frag_keyframe+empty_moov+default_base_moof')
        self.assertEqual(c[-1], 'o.mp4')
        self.assertEqual(gc.expected_size(c), (1920, 1080))

    def test_libx264_command_downloads_the_frames(self):
        c = gc.build_cmd('ff8.exe', 'o.mp4', 7, (1600, 900), 60, 'libx264')
        i = c[c.index('-i') + 1]
        self.assertTrue(i.endswith(',hwdownload,format=bgra'))
        self.assertIn('max_framerate=240', i)                                   # source limit = 4 x the output rate
        self.assertEqual(c[c.index('-pix_fmt') + 1], 'yuv420p')
        self.assertEqual(c[c.index('-g') + 1], '120')
        self.assertEqual(gc.expected_size(c), (1600, 900))

    def test_errors(self):
        with self.assertRaises(ValueError):
            gc.build_cmd('ff8.exe', 'o.mp4', 0, (1920, 1080))
        with self.assertRaises(ValueError):
            gc.build_cmd('ff8.exe', 'o.mp4', 5, (1920, 1080), encoder='x265')
        with self.assertRaises(ValueError):            # the desktop builder keeps rejecting what it does not know
            gn.build_ffmpeg_cmd('ff.exe', 'o.mp4', 'title')
        with self.assertRaises(ValueError):
            gn.build_ffmpeg_cmd('ff.exe', 'o.mp4', gc.MODE)


class FindFfmpegTests(unittest.TestCase):
    def setUp(self):
        gc._filter_cache.clear()

    def tearDown(self):
        gc._filter_cache.clear()

    def test_filter_probe(self):
        self.assertTrue(gc.has_gfxcapture('a.exe', fake_runner(True)))
        self.assertFalse(gc.has_gfxcapture('b.exe', fake_runner(False)))

        def boom(*a, **k):
            raise OSError('no such file')
        self.assertFalse(gc.has_gfxcapture('c.exe', boom))

    def test_explicit_path(self):
        ff, why = gc.find_ffmpeg(os.path.join(tempfile.gettempdir(), 'missing-dir', 'ffmpeg.exe'))
        self.assertIsNone(ff)
        self.assertIn('does not exist', why)
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, 'ffmpeg.exe')
            open(p, 'w').close()
            self.assertEqual(gc.find_ffmpeg(p, runner=fake_runner(True)), (p, ''))
            gc._filter_cache.clear()
            ff, why = gc.find_ffmpeg(p, runner=fake_runner(False))
            self.assertIsNone(ff)
            self.assertIn('no gfxcapture filter', why)

    def test_tools_folder_newest_first(self):
        with tempfile.TemporaryDirectory() as root:
            for v in ('7.1', '8.1', '8.10', '8.9'):
                os.makedirs(os.path.join(root, 'Tools', f'ffmpeg-{v}'))
                open(os.path.join(root, 'Tools', f'ffmpeg-{v}', 'ffmpeg.exe'), 'w').close()
            names = [os.path.basename(os.path.dirname(p)) for p in gc.candidate_ffmpegs(root)]
            self.assertEqual(names, ['ffmpeg-8.10', 'ffmpeg-8.9', 'ffmpeg-8.1', 'ffmpeg-7.1'])
            ff, _ = gc.find_ffmpeg('', root, fake_runner(True))
            self.assertTrue(ff.endswith(os.path.join('ffmpeg-8.10', 'ffmpeg.exe')))
            gc._filter_cache.clear()
            ff, why = gc.find_ffmpeg('', root, fake_runner(False))
            self.assertIsNone(ff)
            self.assertIn('gfxcapture', why)
        with tempfile.TemporaryDirectory() as empty:
            ff, why = gc.find_ffmpeg('', empty, fake_runner(True))
            self.assertIsNone(ff)
            self.assertIn('Tools', why)


class ModeTests(unittest.TestCase):
    def test_default_is_desktop_and_background_switches_to_window(self):
        self.assertEqual(gc.mode_order('desktop', False), ['desktop', 'ddagrab'])
        self.assertEqual(gc.mode_order('desktop', True), ['window', 'desktop', 'ddagrab'])
        self.assertEqual(gc.mode_order('desktop', True, explicit=True), ['desktop', 'ddagrab'])
        self.assertEqual(gc.mode_order('ddagrab', True), ['ddagrab', 'desktop'])
        self.assertEqual(gc.mode_order('window', False), ['window', 'desktop', 'ddagrab'])
        self.assertEqual(gc.mode_order('window', True, explicit=True), ['window', 'desktop', 'ddagrab'])

    def test_warning_text(self):
        self.assertIn('whatever is in front of the game', gc.fallback_warning('desktop', True, 'no build'))
        self.assertIn('no build', gc.fallback_warning('desktop', True, 'no build'))
        self.assertNotIn('in front', gc.fallback_warning('desktop', False))

    def test_parser(self):
        p = night.build_parser()
        a = p.parse_args([])
        self.assertEqual((a.capture, a.capture_explicit, a.capture_ffmpeg), ('desktop', False, ''))
        a = p.parse_args(['--capture', 'desktop'])
        self.assertEqual((a.capture, a.capture_explicit), ('desktop', True))
        a = p.parse_args(['--capture', 'window'])
        self.assertEqual((a.capture, a.capture_explicit), ('window', True))
        with self.assertRaises(SystemExit):
            p.parse_args(['--capture', 'title'])


class ClockTests(unittest.TestCase):
    def test_progress_lines(self):
        r = gc.ProgressReader(None)
        for t, line in ((100.0, 'frame=2'), (100.0, 'out_time_us=-33333'), (100.6, 'frame=17'),
                        (100.6, 'out_time_us=466667'), (101.1, 'frame=33'), (101.1, 'out_time_us=1000000'),
                        (101.1, 'out_time_us=N/A'), (101.1, 'progress=continue')):
            r.feed(line, t)
        self.assertEqual(r.samples, [(100.6, 0.466667), (101.1, 1.0)])             # negative / unknown times dropped
        self.assertTrue(r.delivering())
        self.assertFalse(r.ended)
        name, start = r.input_start()
        self.assertEqual(name, 'gfxcapture')
        self.assertAlmostEqual(start, min(100.6 - 0.466667, 101.1 - 1.0), places=6)   # smallest pipeline delay
        r.feed('progress=end', 102.0)
        self.assertTrue(r.ended)

    def test_not_enough_data(self):
        r = gc.ProgressReader(None)
        self.assertIsNone(r.input_start())
        self.assertFalse(r.delivering())
        r.feed('out_time_us=500000', 10.0)
        self.assertIsNone(r.input_start())               # one update is no estimate
        self.assertFalse(r.delivering())                 # frames still 0

    def test_long_recording_estimates_from_the_start_and_reports_drift(self):
        r = gc.ProgressReader(None)
        for v in range(1, 301):                                  # 5 minutes, updates every second
            drift = 0.0 if v <= 120 else 0.0005 * (v - 120)      # the wall clock gets ahead of the video later on
            r.feed(f'out_time_us={v * 1000000}', 1000.0 + v + 0.05 + drift)
        r.feed('frame=9000', 1301.0)
        self.assertAlmostEqual(r.input_start()[1], 1000.05, places=6)      # from the first minute only
        self.assertAlmostEqual(r.drift_s(), 0.0005 * 120, places=3)       # last minute starts at 240 s
        short = gc.ProgressReader(None)
        for v in range(1, 30):
            short.feed(f'out_time_us={v * 1000000}', 50.0 + v)
        self.assertIsNone(short.drift_s())

    def test_reader_thread_reads_a_stream(self):
        data = b'frame=3\nout_time_us=100000\nprogress=continue\nframe=9\nout_time_us=300000\nprogress=end\n'
        r = gc.ProgressReader(io.BytesIO(data))
        r.join(5)
        self.assertEqual(r.summary(), {'updates': 2, 'frames': 9, 'ended': True, 'start_drift_s': None})

    def test_recording_clock_takes_the_estimate(self):
        spawn = datetime(2026, 11, 14, 1, 0, 0)
        start = (spawn + timedelta(seconds=0.4)).timestamp()
        ck = gn.recording_clock(spawn, spawn + timedelta(seconds=30), 29.5, ('gfxcapture', start))
        self.assertEqual(ck['start_method'], 'gfxcapture_epoch_start')
        self.assertAlmostEqual((ck['video_start'] - spawn).total_seconds(), 0.4, places=3)
        ck = gn.recording_clock(spawn, spawn + timedelta(seconds=30), 29.5, ('lavfi', 0.0))   # the lavfi banner
        self.assertEqual(ck['start_method'], 'spawn_time')

    def test_short_recording_is_flagged(self):
        self.assertEqual(gc.short_check(598.0, 600.5), [])
        self.assertEqual(gc.short_check(None, 600.0), [])
        self.assertEqual(gc.short_check(8.0, 10.0), [])                  # short runs: the slack covers the start delay
        self.assertTrue(gc.short_check(120.0, 600.0)[0].startswith('window capture is 120.0 s long for a 600.0 s'))

    def test_frozen_window_is_flagged(self):
        busy = 'frame=  300 fps= 30 q=24.0 Lsize=2000KiB time=00:00:10.00 bitrate=1.0kbits/s dup=12 drop=0 speed=1x\n'
        still = 'frame=  300 fps= 30 q=24.0 Lsize=300KiB time=00:00:10.00 bitrate=0.2kbits/s dup=250 drop=0 speed=1x\n'
        self.assertEqual(gc.parse_dup_stats(busy), (300, 12, 0))
        self.assertEqual(gc.frozen_check(busy), [])
        self.assertTrue(gc.frozen_check(still)[0].startswith('window capture delivered few new frames: 250 of 300'))
        self.assertEqual(gc.frozen_check('killed, no statistics'), [])


class FakeProc:
    """Stands in for the ffmpeg process."""
    def __init__(self, stdout: bytes = b'', alive: bool = True):
        self.stdin = io.BytesIO()
        self.stdout = io.BytesIO(stdout)
        self.rc = None if alive else 1
        self.returncode = None
        self.killed = False

    def poll(self):
        return self.rc

    def kill(self):
        self.killed, self.rc, self.returncode = True, -9, -9

    def terminate(self):
        self.kill()

    def wait(self, timeout=None):
        return self.rc


class SyncReader(gc.ProgressReader):
    """ProgressReader that consumes its stream at once (no thread: tests run on a fake clock)."""
    def __init__(self, stream=None, clock=None):
        super().__init__(None)
        for raw in (stream or io.BytesIO()):
            self.feed(raw.decode())


DELIVERING = b'frame=20\nout_time_us=600000\nprogress=continue\nframe=40\nout_time_us=1300000\nprogress=continue\n'


class FakeTime:
    def __init__(self):
        self.t = 1000.0

    def monotonic(self):
        return self.t

    def time(self):
        return self.t

    def sleep(self, s):
        self.t += s


class StartRecordingTests(unittest.TestCase):
    """night.Night.start_recording / finalize_recording with fake processes and a fake ffmpeg runner."""

    def setUp(self):
        gc._filter_cache.clear()
        self._time = night.time
        night.time = FakeTime()
        self._rects = night.physical_rects
        night.physical_rects = lambda hwnd: ((0, 0, 3840, 2160), (0, 0, 3840, 2160))
        self.started: list[tuple[list, FakeProc]] = []
        self.tmp = tempfile.TemporaryDirectory()

    def tearDown(self):
        night.time, night.physical_rects = self._time, self._rects
        gc._filter_cache.clear()
        self.tmp.cleanup()

    def make(self, *flags, runner=None):
        from test_night import CommandChannelNightTests
        n, g, api, mod = CommandChannelNightTests.make(CommandChannelNightTests(), '--cmd-channel', 'file', *flags)
        n.runner = runner or fake_runner()
        n.cycle_dir = self.tmp.name
        n.cycle = {'name': 't', 'steps': []}
        return n

    def popen(self, stdout=DELIVERING, window_alive=True):
        """Popen replacement: window capture processes (gfxcapture in the command) get `stdout`, the others die."""
        def factory(cmd, **kw):
            window = 'gfxcapture' in ' '.join(cmd)
            p = FakeProc(stdout if window else b'', alive=window_alive if window else True)
            self.started.append((cmd, p))
            return p
        return factory

    def record(self, n, **kw):
        with mock.patch.object(night.subprocess, 'Popen', self.popen(**kw)), \
                mock.patch.object(night.gt_capture, 'ProgressReader', SyncReader):
            return n.start_recording()

    def test_window_capture_of_the_game_window(self):
        n = self.make('--capture', 'window', '--capture-ffmpeg', self.ffmpeg8())
        rec = self.record(n)
        self.assertEqual((rec['mode'], rec['encoder']), ('window', 'h264_nvenc'))
        cmd = self.started[0][0]
        self.assertEqual(cmd[0], n.args.capture_ffmpeg)
        self.assertIn('gfxcapture=hwnd=11:', cmd[cmd.index('-i') + 1])
        self.assertEqual(rec['expected_wh'], (1920, 1080))                       # 3840x2160 client scaled down
        self.assertNotIn('warning', rec)
        self.assertIsNotNone(rec['progress'])
        data = json.load(open(rec['json'], encoding='utf-8'))
        self.assertEqual(data['mode'], 'window')
        self.assertNotIn('progress', data)
        self.assertEqual(len(self.started), 1)

    def test_background_run_chooses_the_window_capture(self):
        n = self.make('--background', '--capture-ffmpeg', self.ffmpeg8())
        self.assertEqual(self.record(n)['mode'], 'window')

    def test_explicit_desktop_is_kept_in_background(self):
        n = self.make('--background', '--capture', 'desktop', '--capture-ffmpeg', self.ffmpeg8())
        rec = self.record(n)
        self.assertEqual(rec['mode'], 'desktop')
        self.assertNotIn('warning', rec)
        self.assertNotIn('gfxcapture', ' '.join(self.started[0][0]))

    def test_default_run_is_unchanged(self):
        n = self.make()
        rec = self.record(n)
        self.assertEqual(rec['mode'], 'desktop')
        self.assertIsNone(rec['progress'])
        self.assertEqual(self.started[0][0][0], n.args.ffmpeg)
        self.assertIn('gdigrab', self.started[0][0])

    def test_no_capable_ffmpeg_falls_back_with_a_warning(self):
        n = self.make('--background', runner=fake_runner(has_filter=False))
        rec = self.record(n)
        self.assertEqual(rec['mode'], 'desktop')
        self.assertIn('whatever is in front of the game', rec['warning'])
        self.assertIn('gfxcapture', rec['warning'])
        self.assertTrue(any('!!! WARNING' in l for l in open(os.path.join(self.tmp.name, 'night.log'), encoding='utf-8')))
        step = n.cycle['steps'][-1]
        self.assertEqual(step['mode'], 'desktop')
        self.assertEqual(step['warning'], rec['warning'])
        data = json.load(open(rec['json'], encoding='utf-8'))
        self.assertEqual(data['warning'], rec['warning'])

    def test_window_without_frames_falls_back(self):
        n = self.make('--background', '--encoder', 'libx264', '--capture-ffmpeg', self.ffmpeg8())
        rec = self.record(n, stdout=b'')
        self.assertEqual(rec['mode'], 'desktop')
        self.assertIn('delivered no frames', rec['warning'])
        window_proc = self.started[0][1]
        self.assertTrue(window_proc.killed)                                       # the silent window recorder is stopped

    def test_dead_window_recorder_falls_back(self):
        n = self.make('--capture', 'window', '--encoder', 'libx264', '--capture-ffmpeg', self.ffmpeg8())
        rec = self.record(n, window_alive=False)
        self.assertEqual(rec['mode'], 'desktop')
        self.assertIn('warning', rec)

    def test_minimised_window_falls_back(self):
        n = self.make('--background', '--capture-ffmpeg', self.ffmpeg8())
        night.physical_rects = lambda hwnd: ((-32000, -32000, -32000, -32000), (0, 0, 3840, 2160))   # client area 0 x 0
        rec = self.record(n)
        self.assertEqual(rec['mode'], 'desktop')
        self.assertIn('not recordable', rec['warning'])
        self.assertEqual([c for c, _ in self.started if 'gfxcapture' in ' '.join(c)], [])

    def ffmpeg8(self):
        p = os.path.join(self.tmp.name, 'ffmpeg8.exe')
        open(p, 'w').close()
        return p

    def test_finalize_uses_the_progress_clock_and_flags_repeats(self):
        n = self.make('--capture', 'window', '--capture-ffmpeg', self.ffmpeg8())
        rec = self.record(n)
        rec['proc'].rc = 0
        with open(rec['path'], 'wb') as f:
            f.write(b'0' * 100)
        with open(rec['errp'], 'w', encoding='utf-8') as f:
            f.write('frame=  300 fps= 30 q=24.0 Lsize=300KiB time=00:00:10.00 bitrate=0.2kbits/s dup=250 drop=0 speed=1x\n')
        spawn = rec['spawn_wall']
        rec['progress'] = gc.ProgressReader(None)
        for dt, v in ((0.9, 0.5), (1.4, 1.0), (1.9, 1.5)):
            rec['progress'].feed(f'out_time_us={int(v * 1e6)}', (spawn + timedelta(seconds=dt)).timestamp())
        rec['stop'], rec['stop_wall'] = 'q', spawn + timedelta(seconds=10.4)
        n.runner = lambda cmd, **kw: SimpleNamespace(
            returncode=0, stdout='', stderr='  Stream #0:0: Video: h264, yuv420p, 1920x1080, 30 fps\n'
                                            'frame=  300 time=00:00:10.00 signalstats.YAVG=60.0\n')
        with mock.patch.object(night.video_sheets, 'probe_duration', lambda *a, **k: 10.0):
            info = n.finalize_recording(rec)
        self.assertEqual(rec['input_start'][0], 'gfxcapture')
        self.assertAlmostEqual(rec['input_start'][1], spawn.timestamp() + 0.4, places=2)
        self.assertEqual(rec['clock']['start_method'], 'gfxcapture_epoch_start')
        self.assertTrue(any('delivered few new frames' in p for p in info['problems']), info['problems'])
        self.assertFalse(any('frame size' in p for p in info['problems']), info['problems'])   # 1920x1080 expected
        self.assertEqual(rec['progress_info']['updates'], 3)


if __name__ == '__main__':
    unittest.main()
