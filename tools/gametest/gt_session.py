"""Session-level steps around the cycles of night.py: which body the game starts with, the graphics profile, the background
requirements, and the side-by-side summary of several bodies.

--body female|male|both   SaveSession makes the body's save the newest one (select_save.py: only modification times change, with a
                          backup copy) while the game is closed, and puts the times back exactly after the run, on every exit
                          path. The runner never takes over a game that somebody else started. A leftover selection of an earlier
                          runner session (body_session.json next to its pending_restore.json) is restored first on the next start.
--graphics low            the lowest quality profile (graphics_profile.py) before the launch; the player's values are put back when the
                          session ends (restore_graphics), unless --graphics-keep.
--background              requirements (test command module, background pad plugin, window recorder) and the defaults it implies.
"""
from __future__ import annotations

import contextlib
import io
import json
import os
import re
import sys
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import graphics_profile  # noqa: E402
import gt_log  # noqa: E402
import select_save  # noqa: E402

BODY_SAVES = {'female': 'ManualSave-115', 'male': 'ManualSave-106'}
POSE_BODY = {'female': 'woman', 'male': 'man'}          # the name of the body in the pose recording
BODY_CHOICES = ('keep', 'female', 'male', 'both')
MARKER_NAME = 'body_session.json'
DEFAULT_MIN_IDLE_S = 120.0


class SessionError(RuntimeError):
    """A session step cannot be carried out; nothing was changed (or what was changed is stated in the message)."""


def bodies_of(spec: str) -> list[str]:
    """--body value -> the bodies to run in order ([] = keep the game's own save)."""
    if spec in (None, '', 'keep'):
        return []
    if spec == 'both':
        return ['female', 'male']
    if spec in BODY_SAVES:
        return [spec]
    raise SessionError(f'--body {spec!r}: expected one of {", ".join(BODY_CHOICES)}')


def effective_min_idle(args) -> float:
    """--min-idle when given; otherwise 120 s, and 0 for --background (nobody has to keep away from the PC: the game runs
    behind other windows, only Pause stops the run)."""
    v = getattr(args, 'min_idle', None)
    if v is not None:
        return float(v)
    return 0.0 if getattr(args, 'background', False) else DEFAULT_MIN_IDLE_S


# ---------------------------------------------------------------------------------------------- saves
class SaveSession:
    """select_save with the guarantees of a run: never over a running game, the state restored on every exit path, a leftover of
    an earlier runner session restored first. All folders can be given (tests use temporary ones)."""

    def __init__(self, out_root: str, saves_dir: str | None = None, state_file: str | None = None, backup_dir: str | None = None,
                 running=None, out=print, saves: dict | None = None):
        self.out_root = out_root
        self.saves_dir = saves_dir or select_save.default_saves_dir()
        self.state_file = state_file or select_save.default_state_file()
        self.backup_dir = backup_dir or select_save.backups_dir()
        self.running = running or select_save.game_pids
        self.out = out
        self.saves = dict(BODY_SAVES, **(saves or {}))
        self.marker = os.path.join(out_root, MARKER_NAME)
        self.selected: str | None = None

    # -- helpers
    def _lines(self):
        buf: list[str] = []
        return buf, buf.append

    def game_running(self) -> list[int]:
        return list(self.running() or [])

    def state_present(self) -> bool:
        return os.path.isfile(self.state_file)

    def leftover(self) -> dict | None:
        """The marker of an earlier runner session that did not restore, else None."""
        try:
            with open(self.marker, encoding='utf-8') as f:
                return json.load(f)
        except (OSError, ValueError):
            return None

    def _write_marker(self, body: str) -> None:
        os.makedirs(self.out_root, exist_ok=True)
        with open(self.marker, 'w', encoding='utf-8') as f:
            json.dump({'body': body, 'save': self.saves[body], 'state_file': self.state_file,
                       'selected_at': datetime.now().strftime('%Y-%m-%d %H:%M:%S')}, f, indent=1)

    def _drop_marker(self) -> None:
        try:
            os.remove(self.marker)
        except FileNotFoundError:
            pass

    # -- steps
    def restore_leftover(self) -> str | None:
        """First thing at a start: put back the selection an earlier runner session left. Returns what was done, None = nothing."""
        mark = self.leftover()
        if mark is None:
            return None
        text = f'a leftover save selection ({mark.get("save")}, {mark.get("selected_at")}) is restored first'
        self.out(f'[body] {text}')
        self.restore()
        return text

    def select(self, body: str) -> dict:
        """Makes the save of `body` the newest. SessionError (nothing changed) when the game is running, when a selection of
        something else is still in place, or when the save does not exist."""
        if body not in self.saves:
            raise SessionError(f'unknown body {body!r}')
        pids = self.game_running()
        if pids:
            raise SessionError(f'the game is running (process {pids[0]}): it was not started by this run, so it is not closed '
                               'and no save is selected; close it first')
        name = self.saves[body]
        if self.state_present() and self.leftover() is None:
            raise SessionError('a save selection made by select_save.py is still in place (state file '
                               f'{os.path.basename(self.state_file)}): run select_save.py --restore first, nothing was changed')
        buf, out = self._lines()
        self._write_marker(body)
        try:
            select_save.do_select(self.saves_dir, name, self.state_file, self.backup_dir, running=self.running, out=out)
        except select_save.SaveError as e:
            if not self.state_present():
                self._drop_marker()
            raise SessionError(str(e)) from e
        except BaseException:
            # the state file was written before any time was touched: put everything back
            with contextlib.suppress(Exception):
                self.restore()
            raise
        self.selected = name
        for ln in buf:
            self.out(f'[body] {ln}')
        return {'body': body, 'save': name, 'lines': buf}

    def restore(self) -> dict:
        """Puts the modification times back exactly. A game that is still running keeps the state file (and the marker) for the
        next start. Returns {'code', 'lines', 'restored'}."""
        buf, out = self._lines()
        if not self.state_present():
            self._drop_marker()
            self.selected = None
            return {'code': 0, 'lines': ['nothing to restore'], 'restored': False}
        try:
            code = select_save.do_restore(self.saves_dir, self.state_file, running=self.running, out=out)
        except select_save.SaveError as e:
            self.out(f'[body] RESTORE PENDING: {e}; the save order stays changed until select_save.py --restore runs '
                     '(the next start of night.py does it first)')
            return {'code': select_save.EXIT_ERROR, 'lines': [str(e)], 'restored': False}
        for ln in buf:
            self.out(f'[body] {ln}')
        if code == select_save.EXIT_GAME_CHANGED:
            self.out('[body] the game changed saves while the run was going (see above): those were left alone')
        self._drop_marker()
        self.selected = None
        return {'code': code, 'lines': buf, 'restored': True}


# ---------------------------------------------------------------------------------------------- graphics
def apply_graphics(mode: str, settings: str | None = None, state_dir: str | None = None) -> dict:
    """--graphics low: apply the lowest quality profile (the game must be closed). Returns {'mode', 'code', 'status' (one line),
    'lines'}. keep: nothing is done. restore_graphics puts the player's values back at the end of the session."""
    if mode != 'low':
        return {'mode': mode, 'code': 0, 'status': 'graphics profile: not touched (--graphics keep)', 'lines': []}
    argv_extra = (['--settings', settings] if settings else []) + (['--state-dir', state_dir] if state_dir else [])
    buf = io.StringIO()
    err = io.StringIO()
    with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(err):
        code = graphics_profile.main(['--apply'] + argv_extra)
    lines = [ln for ln in (buf.getvalue() + err.getvalue()).splitlines() if ln.strip()]
    if code != 0:
        return {'mode': mode, 'code': code, 'status': f'graphics profile: NOT applied (code {code}): {lines[-1] if lines else ""}',
                'lines': lines}
    tail = lines[-1] if lines else ''
    return {'mode': mode, 'code': 0, 'lines': lines,
            'status': f'graphics profile: low ({tail}); it is put back when the session ends (--graphics-keep keeps it; '
                      'python tools\\gametest\\graphics_profile.py --restore does it by hand)'}


def restore_graphics(settings: str | None = None, state_dir: str | None = None) -> dict:
    """End of a --graphics low session: the player's values go back (the game must be closed; graphics_profile refuses otherwise
    and nothing is written). Returns {'code', 'restored' (options put back), 'status' (one line), 'lines'}."""
    argv_extra = (['--settings', settings] if settings else []) + (['--state-dir', state_dir] if state_dir else [])
    buf = io.StringIO()
    err = io.StringIO()
    with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(err):
        code = graphics_profile.main(['--restore'] + argv_extra)
    lines = [ln for ln in (buf.getvalue() + err.getvalue()).splitlines() if ln.strip()]
    if code != 0:
        return {'code': code, 'restored': 0, 'lines': lines,
                'status': f'graphics profile: NOT restored (code {code}): {lines[-1] if lines else ""}; '
                          'python tools\\gametest\\graphics_profile.py --restore puts the previous values back'}
    m = re.search(r'user settings restored: (\d+) options', '\n'.join(lines))
    if not m:
        return {'code': 0, 'restored': 0, 'lines': lines, 'status': 'graphics profile: nothing to restore (the test profile is not applied)'}
    return {'code': 0, 'restored': int(m.group(1)), 'lines': lines,
            'status': f'graphics profile restored: {m.group(1)} options (the player\'s values are back)'}


def graphics_status(settings: str | None = None, state_dir: str | None = None) -> str:
    argv_extra = (['--settings', settings] if settings else []) + (['--state-dir', state_dir] if state_dir else [])
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        graphics_profile.main(['--status'] + argv_extra)
    return ' | '.join(ln.strip() for ln in buf.getvalue().splitlines() if ln.strip())


# ---------------------------------------------------------------------------------------------- background
def background_requirements(game_dir: str, testcmd_dir: str, args, ffmpeg_with_gfxcapture: str | None, why_no_ffmpeg: str = '',
                            ready_line=None) -> list[tuple[str, bool, str]]:
    """[(requirement, met, detail)] of a --background run: the test command module (file channel), the background pad plugin,
    the window recorder. `ready_line` = the module's READY line of the newest session log, None = none seen (informative only:
    the module logs it at the start of a session)."""
    host = os.path.join(game_dir, 'red4ext', 'plugins', 'UTPPTestHost', 'UTPPTestHost.dll')
    init = os.path.join(testcmd_dir, 'init.lua')
    chan = getattr(args, 'cmd_channel', 'console')
    out = [
        ('test command module (utpp_testcmd, file command channel) installed', os.path.isfile(init), init),
        ('background pad plugin (UTPPTestHost) installed', os.path.isfile(host), host),
        ('window recorder (ffmpeg with the gfxcapture filter)', bool(ffmpeg_with_gfxcapture),
         ffmpeg_with_gfxcapture or why_no_ffmpeg or 'not found'),
        ('command channel is file or auto', chan in ('file', 'auto'), f'--cmd-channel {chan}'),
    ]
    return out


# ---------------------------------------------------------------------------------------------- several bodies side by side
def _load(path: str):
    try:
        with open(path, encoding='utf-8') as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def run_digest(run_dir: str) -> dict:
    """What one run folder says per scenario: the night verdict, the harness step verdicts and the pose meter's columns."""
    summ = _load(os.path.join(run_dir, 'summary.json')) or {}
    gt = _load(os.path.join(run_dir, 'gametest', 'summary.json')) or {}
    pose = _load(os.path.join(run_dir, 'pose_metrics.json')) or {}
    cyc = _load(os.path.join(run_dir, 'cycle.json')) or {}
    scen: dict[str, dict] = {}
    for s in cyc.get('scenarios', []):
        k = f'{s["name"]}#{s["run"]}'
        scen.setdefault(k, {})['night'] = s.get('verdict') or '-'
    for st in gt.get('steps', []):
        k = f'{st["scenario"]}#{st.get("run", 1)}'
        d = scen.setdefault(k, {}).setdefault('steps', {})
        d[st['verdict']] = d.get(st['verdict'], 0) + 1
    for r in pose.get('rows', []):
        if '/' in str(r.get('label', '')):
            continue
        scen.setdefault(str(r['label']), {})['pose'] = {k: r.get(k) for k in (
            'duration_s', 'head_speed_p95', 'chest_speed_p95', 'spikes_per_s', 'helper_p95_cm', 'anatomy_share')}
    return {'run': os.path.basename(os.path.abspath(run_dir)), 'result': cyc.get('result'), 'pose_body': pose.get('body'),
            'perf': cyc.get('perf_status'), 'scenarios': scen, 'harness': summ.get('harness'),
            'skipped': summ.get('skipped_scenarios'), 'body_mismatch': cyc.get('body_mismatch'), 'pose_owner': summ.get('pose_owner')}


def _cell(v, nd=1) -> str:
    if v is None:
        return '-'
    return f'{v:.{nd}f}' if isinstance(v, float) else str(v)


def side_by_side_lines(digests: dict[str, dict]) -> list[str]:
    """One table: a row per scenario, the columns of every body next to each other."""
    bodies = list(digests)
    names: list[str] = []
    for b in bodies:
        for k in digests[b]['scenarios']:
            if k not in names:
                names.append(k)
    head = ['scenario'] + [f'{b}: {c}' for b in bodies for c in ('night', 'steps', 'head p95', 'chest p95', 'spikes/s', 'helper cm', 'anat %')]
    rows = []
    for n in names:
        row = [n]
        for b in bodies:
            d = digests[b]['scenarios'].get(n)
            if not d:
                row += ['-'] * 7
                continue
            steps = ' '.join(f'{k[0]}{v}' for k, v in sorted((d.get('steps') or {}).items())) or '-'
            p = d.get('pose') or {}
            share = p.get('anatomy_share')
            row += [d.get('night') or '-', steps, _cell(p.get('head_speed_p95'), 0), _cell(p.get('chest_speed_p95'), 0),
                    _cell(p.get('spikes_per_s'), 2), _cell(p.get('helper_p95_cm'), 2),
                    '-' if share is None else f'{share * 100:.1f}']
        rows.append(row)
    w = [max(len(head[i]), *(len(r[i]) for r in rows)) if rows else len(head[i]) for i in range(len(head))]
    lines = ['  '.join(h.ljust(w[i]) for i, h in enumerate(head)), '  '.join('-' * w[i] for i in range(len(head)))]
    lines += ['  '.join(c.ljust(w[i]) for i, c in enumerate(r)) for r in rows]
    for b in bodies:
        d = digests[b]
        lines.append(f'{b}: run {d["run"]}, result {d["result"]}, pose body {d["pose_body"]}'
                     + f', pose owner {gt_log.pose_owner_label(d.get("pose_owner"))}'
                     + (f', {len(d["skipped"])} scenarios not played' if d.get('skipped') else '')
                     + (f', perf {d["perf"]}' if d.get('perf') else ''))
        want = POSE_BODY.get(b)
        if d.get('body_mismatch') or (want and d.get('pose_body') and d['pose_body'] != want):
            lines.append(f'{b}: !!! BODY MISMATCH: ' + (d.get('body_mismatch') or
                                                       f'the pose recording is of the {d["pose_body"]} body, not the {want} body')
                         + ' - this run does not show the requested body')
        sv = d.get('saves') or {}
        if sv.get('select_error'):
            lines.append(f'{b}: save not selected: {sv["select_error"]}')
        elif sv:
            sel, res = sv.get('select') or {}, sv.get('restore') or {}
            lines.append(f'{b}: save {sel.get("save")} selected; restore code {res.get("code")}, restored={res.get("restored")}'
                         + ('' if res.get('code') == 0 and res.get('restored') else
                            ' !!! ' + '; '.join(res.get('lines') or ['not restored'])))
    owners = {b: (digests[b].get('pose_owner') or {}).get('owner') or 'unknown' for b in bodies}
    if len(set(owners.values())) > 1:
        lines.append('!!! POSE OWNER DIFFERS: ' + ', '.join(f'{b} {o}' for b, o in owners.items())
                     + ' - these runs are not comparable')
    return lines
