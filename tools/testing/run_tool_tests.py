"""Common runner for the offline Python tool tests (Source\\tools\\**\\test_*.py, plus test_*.ps1 run with powershell -File) that live outside ctest.

  python -I Source\\tools\\testing\\run_tool_tests.py [FILTER ...] [options]

FILTER      folder name (gametest, mmbake, ...) or a substring of the path relative to Source\\tools
            (mmbake/test_mmbake_base.py, test_night); several filters = union; none = every discovered file.
--list      print the plan (markers, reasons, command) and run nothing
--include M[,M]   also run files with these markers ('all' = every marker); default-skipped markers:
                    melee       Source\\tools\\melee (heavy data in its work root)
                    gpu         needs a CUDA GPU
                    game        needs the running game
                    blender     needs Blender's Python (bpy)
                    needs-args  CLI tool named test_*: needs input paths / a built exe, no self-contained test
--exclude M[,M]   skip files with these markers too; run by default but excludable:
                    slow        long runtime (the Lua replay)
                    data        bakes or reads large project data (Data DBs, Animations exports, packs)
--quick     = --exclude slow,data (the fast unit tests only)
--strict    known failures (XFAIL) and unexpected passes (XPASS) count as failures
--python EXE      interpreter for every file (default: Tools\\ai-motion\\envs\\kimodo\\Scripts\\python.exe, it has
                  numpy / scipy / PIL / torch; the system Python lacks scipy)
--timeout S       per-file timeout (default 600; the registry can raise it); the process tree is killed
--jobs N          parallel folder groups (default 4); the files of one folder always run one after another
--log-dir DIR     one log per file + summary.json (default Research\\logs\\build\\tool-tests\\<stamp>)
--json PATH       copy of the JSON summary  --tail N  log lines shown per failing file (default 12)

Every file runs in its own process: <python> -I -c <bootstrap> <file> [args], cwd = the file's folder. The bootstrap
puts the file's folder first on sys.path and runs it as __main__ (= `python test_x.py` as the files document it,
while -I keeps PYTHON* variables and the user site out; Python 3.11+ -I alone drops the script folder from sys.path).
Verdict per file: exit code 0 = PASS, else FAIL (unittest failures / a non-zero exit), ERROR (unittest errors only,
a traceback or a usage error), TIMEOUT. Unittest counts are parsed from the log; for plain scripts the last summary
line is shown, and a WARN is printed when a script exits 0 but its log reports failures.
Registry (REGISTRY below): markers, extra args, timeout, data the file needs (missing -> SKIP with the path) and known
failures (xfail + xfail_sig: the file runs and is reported as XFAIL with the reason only while its failures are
exactly the known signature - the [FAIL] labels / failing unittest ids and the exception; any extra, missing or
different failure is a FAIL "new failure in a known-failing file"). Files not in the registry run with defaults and
are reported as 'unregistered'.
Exit code: 0 = no FAIL / ERROR / TIMEOUT (with --strict also no XFAIL / XPASS); 1 = failures; 2 = usage error or
no file matched the filters.
Examples:
  run_tool_tests.py                       default set (melee / gpu / game / blender / needs-args skipped)
  run_tool_tests.py gametest -j 1         only Source\\tools\\gametest, serially
  run_tool_tests.py --quick --strict      fast unit tests only (no slow / data), fail on known failures too
  run_tool_tests.py melee --include melee,gpu
"""
from __future__ import annotations

import argparse
import concurrent.futures as cf
import json
import os
import re
import shutil
import subprocess
import sys
import threading
import time
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)                                   # Source\tools
ROOT = os.path.abspath(os.path.join(TOOLS, '..', '..'))         # project root
KIMODO = os.path.join(ROOT, 'Tools', 'ai-motion', 'envs', 'kimodo', 'Scripts', 'python.exe')
DEFAULT_SKIP = ('melee', 'gpu', 'game', 'blender', 'needs-args')
KNOWN_MARKERS = set(DEFAULT_SKIP) | {'slow', 'data'}
SKIP_DIRS = {'__pycache__', '.git', 'testdata', 'fixtures', 'node_modules'}
BOOT = ("import os, runpy, sys; f = sys.argv[1]; sys.argv = sys.argv[1:]; "
        "sys.path.insert(0, os.path.dirname(f)); runpy.run_path(f, run_name='__main__')")

# Markers for every file of a folder (also for files added later).
FOLDER_MARKERS = {'melee': {'melee'}}

# Per-file registry, paths relative to Source\tools. Keys: markers, args, timeout, requires (project-relative paths
# that must exist, else SKIP), xfail (reason) + xfail_sig {'fails': [regex fullmatched one-to-one against the failure
# labels: '[FAIL] <label> -- ...' / '[FAILED ] <label>' script lines or unittest 'FAIL:/ERROR: <id>' lines],
# 'exception': regex searched in the last exception line, None = no traceback allowed}, note. State of 2026-11-09 (T5).
REGISTRY = {
    'ai_motion/test_bike_glide.py': {},
    'ai_motion/test_foot_pin_span.py': {
        'note': 'WholeSpan tests SKIP on the known (1 - w) bug until Build\\work\\tool-tests-20261109\\patch\\'
                'foot_pin.py.diff is applied; UTPP_FOOT_PIN_STRICT=1 makes them fail'},
    'anim/melee-export/test_melee_export.py': {},
    'anim/melee-export/test_simd_roundtrip.ps1': {
        'markers': {'needs-args'}, 'reason': 'needs -Out <new dir under Build\\work> and WolvenKit.CLI / WkitBridge'},
    'films/test_run_compare_film.py': {'note': 'one case cuts two generated clips with the project ffmpeg and SKIPs without it'},
    'install/test_stage_melee_npc.ps1': {
        'note': 'offline dry-run self-check of stage_melee_npc.ps1; exit 3 = inputs missing under Build\\work'},
    'release/test_make_release.py': {},
    'airz/test_make_airz.py': {'markers': {'data'}, 'note': 'reads the real Data .airz, recreates Build\\test-data\\airz_test'},
    'analysis/test_calibrate_nue_mount_capture.py': {},
    'analysis/test_export_player_helper_bundle.py': {},
    'analysis/test_helper_coverage_report.py': {},
    'analysis/test_helper_graph_batch.py': {},
    'analysis/test_helper_residual_report.py': {},
    'analysis/test_skin_displacement_report.py': {},
    'analysis/test_post_helper_skin_capture.py': {},
    'analysis/test_pose_capture_to_csv.py': {
        'requires': ['Build/test-data/posecapture/utpp-bodytrace-slice.csv'],
        'note': 'compact pose capture -> CSV; the C++ writer cross-check runs in ctest (utpp_pose_capture_converter_test, UTPP_POSE_GOLDEN)'},
    'analysis/test_melee_pose_log.py': {},
    'analysis/test_pose_clip_match.py': {},
    'analysis/test_pose_helper_report.py': {
        'note': 'helper-bone report of compact pose recordings; synthetic recording from the encoder of test_pose_capture_to_csv.py'},
    'analysis/test_punch_direction_report.py': {},
    'analysis/test_pose_stage_pops.py': {'note': 'stage pop attribution on a synthetic stage file and synthetic clip frames'},
    'analysis/test_start_stop_recorded.py': {
        'markers': {'needs-args'}, 'reason': 'analysis CLI: positional baseline candidate feed output'},
    'gametest/test_bike_cases_nav.py': {},
    'gametest/test_bt_gate.py': {},
    'gametest/test_car_cases_nav.py': {},
    'gametest/test_gametest.py': {},
    'gametest/test_graphics_profile.py': {},
    'gametest/test_gt_capture.py': {},
    'gametest/test_gt_collect.py': {},
    'gametest/test_gt_dodge.py': {},
    'gametest/test_gt_flow.py': {},
    'gametest/test_gt_freeze.py': {'note': 'writes real process dumps of a throw-away child process of the test (a few MB, a few seconds)'},
    'gametest/test_gt_melee_weapons.py': {},
    'gametest/test_gt_ranged_families.py': {},
    'gametest/test_gt_states.py': {},
    'gametest/test_gt_post.py': {'note': 'one case cuts an 8 s test video with the project ffmpeg and SKIPs without it'},
    'gametest/test_gt_posemeter.py': {'note': 'runs the pose meter worker processes on the small slice in testdata (a few seconds)'},
    'gametest/test_gt_session.py': {},
    'gametest/test_gt_session_end.py': {},
    'gametest/test_gt_settings.py': {'note': 'the Lua cases SKIP without Tools\\lua-5.1.5\\lua51.dll'},
    'gametest/test_log_rotation.py': {},
    'gametest/test_night.py': {'note': 'test_real_frames_cycle5 SKIPs: the night cycle-5 frame PNGs are gone'},
    'gametest/test_night_data.py': {},
    'gametest/test_night_speed.py': {},
    'gametest/test_pose_metrics.py': {'note': 'the helper mismatch case SKIPs without the extracted deformer graph'},
    'gametest/test_select_save.py': {},
    'lua-tests/test_utpp_camera_run.py': {'markers': {'slow'}, 'note': 'Lua 5.1 DLL replay, ~140 s'},
    'melee/test_anatomy_gate.py': {},
    'melee/test_apply_keyposes.py': {'note': 'needs the melee dumps (melee_dump.py all)'},
    'melee/test_contact.py': {},
    'melee/test_fist_base_fixture.py': {},
    'melee/test_redo.py': {'note': 'needs base_body.py build once'},
    'melee/test_melee_regenerate.py': {'markers': {'gpu'}, 'note': 'MotionBricks on CUDA; writes into the melee MB4\\_tmp'},
    'mmbake/test_extra_assets.py': {'note': 'pure functions only without --off/--on'},
    'mmbake/test_motion_chooser_ref.py': {'markers': {'data'}},
    'mmbake/test_getupbake.py': {'markers': {'data'}},
    'mmbake/test_dodgebake.py': {'markers': {'data'}},
    'mmbake/test_leanbake.py': {
        'markers': {'data'}, 'requires': ['Research/logs/sourcepack-lean-data-2026-10.json'],
        'note': 'leanbake.LEAN_DATA; the file was not found in the project folders (2026-11-09)'},
    'mmbake/test_mmbake_base.py': {
        'markers': {'data'},
        'note': 'bakes the clips the Data subset file lists (the clip folder holds more); the current meta adds a database the '
                'file has no tag for, reported in the detail of the determinism and tag checks'},
    'mmbake/test_mmbake_motion.py': {
        'markers': {'data'}, 'requires': ['Data/utpp_motion_woman.summary.json'],
        'note': 'validates the baker against a woman v2 database WITH its bake summary (bake_args -> retarget inputs); the only such pair '
                'was removed 2026-11-11; the package database has no summary'},
    'mmbake/test_overlaybake.py': {'markers': {'data'}},
    'mmbake/test_slide_spec.py': {'note': 'the C++ mutation guard needs --exe --slide --base --oracle'},
    'mmbake/test_travbake.py': {'markers': {'data'}},
    'mmbake/test_travbake_motion.py': {'markers': {'data'}},
    'ragdoll/test_getup_metric.py': {},
    'replay/test_slide_replay.py': {
        'markers': {'needs-args'}, 'reason': 'needs --exe <utpp_motion_replay.exe> --db <slide bundle> --feeds <dir>'},
    'retarget/test_ankle_offset_yaw.py': {
        'markers': {'blender'}, 'reason': 'imports retarget_batch, which imports bpy (Blender Python only)'},
}

RAN_RE = re.compile(r'^Ran (\d+) tests? in [0-9.]+s', re.M)
UT_STATUS_RE = re.compile(r'^(OK|FAILED)(?: \(([^)]*)\))?\s*$', re.M)
SCRIPT_SUMMARY_RE = re.compile(r'(?i)\b(pass(?:ed)?|fail(?:ed|s|ures)?|result|summary)\b.*\d')
# a script log line that reports a failure: [FAIL] / FAIL: at the line start, "FAIL 3", "FAILS: 2", "5 failed" or
# "2 failures" (not "passed 16 failed 0" / "done=1 failed=0"), "failed 1" (count after the word), "failures=1"
SCRIPT_FAIL_RE = re.compile(r'^\s*\[FAIL(?:ED)?\s*\]|^\s*FAIL(?:ED)?\b[:\s]|\bFAILS?:?\s+[1-9]'
                            r'|(?<!passed )(?<![=\d])\b[1-9]\d*\s+(?:failed|failures?)\b(?!=)|\bfailed\s+[1-9]'
                            r'|\bfailures=[1-9]', re.M)
EXC_RE = re.compile(r'^(\w+(?:Error|Exception|Exit)\b.*)$', re.M)


def discover():
    out = []
    for d, dirs, files in os.walk(TOOLS):
        dirs[:] = sorted(x for x in dirs if x not in SKIP_DIRS and not x.startswith('.'))
        for f in sorted(files):
            if f.startswith('test_') and f.endswith(('.py', '.ps1')):
                out.append(os.path.relpath(os.path.join(d, f), TOOLS).replace(os.sep, '/'))
    return out


def entry(rel):
    e = dict(REGISTRY.get(rel, {}))
    e['registered'] = rel in REGISTRY
    e['markers'] = set(e.get('markers', set())) | FOLDER_MARKERS.get(rel.split('/')[0], set())
    return e


def plan(files, filters, include, exclude):
    """[(rel, entry, skip_reason or None)] for the files matching the filters."""
    out = []
    for rel in files:
        if filters and not any(rel.split('/')[0] == f or f.replace('\\', '/') in rel for f in filters):
            continue
        e = entry(rel)
        skip = None
        blocked = sorted(m for m in e['markers'] if (m in DEFAULT_SKIP and m not in include and 'all' not in include)
                         or m in exclude)
        if blocked:
            skip = 'marker %s%s' % (','.join(blocked), (': ' + e['reason']) if e.get('reason') else '')
        else:
            missing = [p for p in e.get('requires', []) if not os.path.exists(os.path.join(ROOT, *p.split('/')))]
            if missing:
                skip = 'missing data: ' + ', '.join(missing)
        out.append((rel, e, skip))
    return out


def parse_log(text):
    r = {}
    text = re.sub(r'\A(?:# [^\n]*\n){1,2}', '', text)  # the runner's own header (command, cwd)
    ran = RAN_RE.findall(text)
    st = UT_STATUS_RE.findall(text)
    if ran and st:
        r['unittest'] = {'ran': int(ran[-1]), 'status': st[-1][0]}
        for kv in (st[-1][1] or '').split(','):
            if '=' in kv:
                k, v = kv.strip().split('=', 1)
                r['unittest'][k] = int(v)
    else:
        lines = [l.strip() for l in text.splitlines() if l.strip()]
        summ = [l for l in lines if SCRIPT_SUMMARY_RE.search(l)]
        r['summary_line'] = (summ[-1] if summ else (lines[-1] if lines else ''))[:160]
        r['fail_lines'] = [l[:160] for l in lines if SCRIPT_FAIL_RE.search(l)][:5]
    r['traceback'] = 'Traceback (most recent call last)' in text
    r['usage_error'] = bool(re.search(r'^usage: .*\n.*error: ', text, re.M))
    exc = EXC_RE.findall(text) if r['traceback'] else []
    if exc:
        r['exception'] = exc[-1][:300]
    # every failure label (for the xfail signature): script '[FAIL] label -- detail' lines + unittest FAIL/ERROR ids
    labels = [m.split(' -- ')[0].strip() for m in re.findall(r'^\s*\[FAIL(?:ED)?\s*\]\s*(.*?)\s*$', text, re.M)]
    labels += re.findall(r'^(?:FAIL|ERROR): (\S+ \([^)]*\))', text, re.M)
    r['fail_labels'] = labels
    return r


def xfail_matches(sig, info):
    """(True, '') when the failures of a known-failing file are exactly its signature, else (False, what differs)."""
    if not sig:
        return False, 'xfail entry without xfail_sig'
    left = list(info.get('fail_labels', []))
    for pat in sig.get('fails', []):
        hit = next((i for i, l in enumerate(left) if re.fullmatch(pat, l)), None)
        if hit is None:
            return False, 'expected failure missing: %s' % pat[:100]
        left.pop(hit)
    if left:
        return False, 'new failure: %s' % left[0][:120]
    exc = info.get('exception') if info.get('traceback') else None
    if sig.get('exception') and not (exc and re.search(sig['exception'], exc)):
        return False, 'exception differs: %s' % ((exc or 'none')[:120])
    if not sig.get('exception') and info.get('traceback'):
        return False, 'new exception: %s' % ((exc or 'traceback')[:120])
    return True, ''


def kill_tree(p):
    if os.name == 'nt':
        subprocess.run(['taskkill', '/T', '/F', '/PID', str(p.pid)], capture_output=True)
    else:
        p.kill()
    try:
        p.wait(timeout=30)
    except subprocess.TimeoutExpired:
        pass


def run_file(rel, e, python, timeout, log_dir):
    path = os.path.join(TOOLS, *rel.split('/'))
    log = os.path.join(log_dir, rel.replace('/', '__') + '.log')
    if rel.endswith('.ps1'):   # PowerShell tool test: exit code = result
        cmd = ['powershell', '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', path] + list(e.get('args', []))
    else:
        cmd = [python, '-I', '-c', BOOT, path] + list(e.get('args', []))
    t0 = time.time()
    with open(log, 'w', encoding='utf-8', errors='replace') as fh:
        fh.write('# %s\n# cwd %s\n' % (subprocess.list2cmdline(cmd), os.path.dirname(path)))
        fh.flush()
        p = subprocess.Popen(cmd, cwd=os.path.dirname(path), stdout=fh, stderr=subprocess.STDOUT,
                             stdin=subprocess.DEVNULL)
        try:
            rc = p.wait(timeout=e.get('timeout', timeout))
        except subprocess.TimeoutExpired:
            kill_tree(p)
            rc = None
    secs = time.time() - t0
    with open(log, encoding='utf-8', errors='replace') as fh:
        info = parse_log(fh.read())
    ut = info.get('unittest')
    if rc is None:
        status = 'TIMEOUT'
    elif rc == 0:
        status = 'PASS'
    elif ut and ut.get('failures', 0) == 0 and ut.get('errors', 0) > 0:
        status = 'ERROR'
    elif not ut and (info['traceback'] or info['usage_error']):
        status = 'ERROR'
    else:
        status = 'FAIL'
    sig_diff = None
    if e.get('xfail') and status in ('FAIL', 'ERROR'):
        ok, sig_diff = xfail_matches(e.get('xfail_sig'), info)
        status = 'XFAIL' if ok else 'FAIL'
    elif e.get('xfail') and status == 'PASS':
        status = 'XPASS'
    if ut:
        detail = 'ran %d %s' % (ut['ran'], ut['status'])
        extra = ', '.join('%s=%d' % (k, ut[k]) for k in ('failures', 'errors', 'skipped', 'expected failures',
                                                           'unexpected successes') if ut.get(k))
        detail += (' (%s)' % extra) if extra else ''
    elif rc != 0 and info.get('exception'):
        detail = info['exception'][:160]
    else:
        detail = info.get('summary_line', '')
    if sig_diff:
        detail = 'new failure in a known-failing file (%s); %s' % (sig_diff, detail)
    warn = None
    if rc == 0 and not ut and info.get('fail_lines'):
        warn = 'exit 0 but the log reports failures: ' + ' | '.join(info['fail_lines'][:2])
    return {'file': rel, 'status': status, 'rc': rc, 'seconds': round(secs, 1), 'detail': detail, 'log': log,
            'warn': warn, 'registered': e['registered'], 'xfail': e.get('xfail'), 'note': e.get('note')}


def tail(path, n):
    try:
        with open(path, encoding='utf-8', errors='replace') as fh:
            lines = [l.rstrip() for l in fh if l.strip()]
    except OSError:
        return []
    return lines[-n:]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0], formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog='See the module docstring for markers and verdicts.')
    ap.add_argument('filters', nargs='*')
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--include', default='')
    ap.add_argument('--exclude', default='')
    ap.add_argument('--quick', action='store_true')
    ap.add_argument('--strict', action='store_true')
    ap.add_argument('--python', default=KIMODO if os.path.isfile(KIMODO) else sys.executable)
    ap.add_argument('--timeout', type=float, default=600.0)
    ap.add_argument('-j', '--jobs', type=int, default=4)
    ap.add_argument('--log-dir', default='')
    ap.add_argument('--json', default='')
    ap.add_argument('--tail', type=int, default=12)
    a = ap.parse_args(argv)
    include = {m.strip() for m in a.include.split(',') if m.strip()}
    exclude = {m.strip() for m in a.exclude.split(',') if m.strip()} | ({'slow', 'data'} if a.quick else set())
    unknown = (include | exclude) - KNOWN_MARKERS - {'all'}
    if unknown:
        print('unknown marker(s): %s (known: %s)' % (', '.join(sorted(unknown)), ', '.join(sorted(KNOWN_MARKERS))))
        return 2
    a.python = a.python if os.path.isfile(a.python) else (shutil.which(a.python) or '')
    if not a.python:
        print('python not found (--python)')
        return 2
    files = discover()
    stale = sorted(set(REGISTRY) - set(files))
    sel = plan(files, a.filters, include, exclude)
    if not sel:
        print('no test file matches %s (discovered %d under %s)' % (a.filters, len(files), TOOLS))
        return 2
    to_run = [(rel, e) for rel, e, skip in sel if skip is None]
    if a.list:
        for rel, e, skip in sel:
            flags = ','.join(sorted(e['markers'])) or '-'
            state = ('SKIP (%s)' % skip) if skip else ('run, known failure: %s' % e['xfail'] if e.get('xfail') else 'run')
            print('%-48s [%s]%s %s' % (rel, flags, '' if e['registered'] else ' UNREGISTERED', state))
            if e.get('note'):
                print('%48s  note: %s' % ('', e['note']))
        print('%d discovered, %d selected, %d would run; python %s' % (len(files), len(sel), len(to_run), a.python))
        for s in stale:
            print('registry entry without a file: %s' % s)
        return 0

    log_dir = a.log_dir or os.path.join(ROOT, 'Research', 'logs', 'build', 'tool-tests',
                                        datetime.now().strftime('%Y%m%d-%H%M%S'))
    log_dir = os.path.abspath(log_dir)
    os.makedirs(log_dir, exist_ok=True)
    print('UTPP tool tests: %d discovered, %d selected, %d to run; python %s; jobs %d; logs %s'
          % (len(files), len(sel), len(to_run), a.python, a.jobs, log_dir), flush=True)
    groups = {}
    for rel, e in to_run:
        groups.setdefault(rel.split('/')[0], []).append((rel, e))
    lock = threading.Lock()
    results = []

    def run_group(items):
        for rel, e in items:
            r = run_file(rel, e, a.python, a.timeout, log_dir)
            with lock:
                results.append(r)
                print('  %-7s %6.1fs  %-48s %s' % (r['status'], r['seconds'], rel, r['detail']), flush=True)

    t0 = time.time()
    with cf.ThreadPoolExecutor(max(1, a.jobs)) as ex:
        list(ex.map(run_group, sorted(groups.values(), key=lambda g: -len(g))))
    wall = time.time() - t0
    results.sort(key=lambda r: r['file'])
    skipped = [{'file': rel, 'status': 'SKIP', 'reason': skip, 'registered': e['registered']}
               for rel, e, skip in sel if skip is not None]
    count = {k: sum(r['status'] == k for r in results) for k in ('PASS', 'FAIL', 'ERROR', 'TIMEOUT', 'XFAIL', 'XPASS')}
    count['SKIP'] = len(skipped)
    bad = count['FAIL'] + count['ERROR'] + count['TIMEOUT'] + ((count['XFAIL'] + count['XPASS']) if a.strict else 0)
    rc = 1 if bad else 0

    print('\nResults (%d run, wall %.0f s):' % (len(results), wall))
    for r in results:
        print('  %-7s %6.1fs  %-48s %s' % (r['status'], r['seconds'], r['file'], r['detail']))
    failing = [r for r in results if r['status'] in ('FAIL', 'ERROR', 'TIMEOUT')]
    if failing:
        print('\nFailing files (log tail):')
        for r in failing:
            print('  %s %s  (log %s)' % (r['status'], r['file'], r['log']))
            for l in tail(r['log'], a.tail):
                print('      | ' + l[:200])
    xs = [r for r in results if r['status'] in ('XFAIL', 'XPASS')]
    if xs:
        print('\nKnown failures (registry xfail%s):' % ('; --strict: counted as failures' if a.strict else '; not counted, --strict counts them'))
        for r in xs:
            hint = '  -> passes now: remove the xfail entry' if r['status'] == 'XPASS' else ''
            print('  %s %s: %s%s' % (r['status'], r['file'], r['xfail'], hint))
    if skipped:
        print('\nSkipped:')
        for s in skipped:
            print('  SKIP %s: %s' % (s['file'], s['reason']))
    warns = [r for r in results if r['warn']] + [r for r in results if not r['registered']]
    if warns:
        print('\nWarnings:')
        for r in results:
            if r['warn']:
                print('  WARN %s: %s' % (r['file'], r['warn']))
            if not r['registered']:
                print('  WARN %s: unregistered (ran with defaults; add it to REGISTRY)' % r['file'])
    for s in stale:
        print('  WARN registry entry without a file: %s' % s)
    print('\nSummary: ' + '  '.join('%s %d' % (k, v) for k, v in count.items()) +
          '  (wall %.0f s) -> exit %d' % (wall, rc))
    summary = {'python': a.python, 'filters': a.filters, 'include': sorted(include), 'exclude': sorted(exclude),
               'strict': a.strict, 'jobs': a.jobs, 'wall_s': round(wall, 1), 'counts': count, 'exit': rc,
               'results': results, 'skipped': skipped, 'stale_registry': stale}
    with open(os.path.join(log_dir, 'summary.json'), 'w', encoding='utf-8') as fh:
        json.dump(summary, fh, indent=1)
    if a.json:
        with open(a.json, 'w', encoding='utf-8') as fh:
            json.dump(summary, fh, indent=1)
    return rc


if __name__ == '__main__':
    sys.exit(main())
