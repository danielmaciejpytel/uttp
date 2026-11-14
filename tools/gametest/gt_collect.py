"""Copies the files a game session writes into the run folder while the game is still running (read-only on the source).

A session writes, into the game's red4ext log folder,
  utpp-pose-ring-<stamp>.utpppose  one-minute pose segments; the mod keeps only the last ten minutes, an older one is deleted
                                   as soon as the next one is closed
  utpp-pose-<stamp>.utpppose       the single-file form of the pose recording (nothing is deleted)
  utpp-feed-<stamp>.txt            the input dump
  utpp-<stamp>.log (+ .N.log)      the mod log of the session
SessionCollector.collect() copies what is new or changed since the last call into <dest>, never raising on a file problem:
  between = True   only ring segments that are finished (every ring segment except the newest one, which the game still writes)
  final   = True   everything; run it before the game is quit and again after it ended - the pose writer flushes its last
                   block on close, so the first copy is shorter than the final one
Files are picked by the time stamp in their NAME (at or after the session's own stamp), not by their modification time, so the
ring segments of an earlier session that are still on disk are left alone.
The manifest (name, bytes, kind, phase, status, note) goes into the cycle record. Gaps between copied ring segments are reported.

  python -I gt_collect.py SOURCE_DIR DEST_DIR [--since "YYYY-MM-DD HH:MM:SS"] [--session-log PATH]     one final collect
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import re
import shutil
import sys
from datetime import datetime, timedelta

STAMP_RE = re.compile(r'(\d{4})-(\d\d)-(\d\d)-(\d\d)-(\d\d)-(\d\d)')
RING_PREFIX = 'utpp-pose-ring-'
POSE_GLOB = 'utpp-pose-*.utpppose'
FEED_GLOB = 'utpp-feed-*.txt'
SEGMENT_S = 60.0                  # nominal length of a ring segment
GAP_TOLERANCE_S = 20.0            # a stamp distance above SEGMENT_S + this is a gap (a segment the ring dropped)
CHUNK = 1 << 20
SHARING_ERRORS = (32, 33)         # Windows: sharing violation, lock violation


def name_stamp(name: str) -> datetime | None:
    """The local date-time in a file name (utpp-pose-ring-2026-11-14-01-43-37.utpppose), else None."""
    m = STAMP_RE.search(os.path.basename(name))
    if not m:
        return None
    try:
        return datetime(*[int(x) for x in m.groups()])
    except ValueError:
        return None


def is_ring(name: str) -> bool:
    return os.path.basename(name).startswith(RING_PREFIX)


def pose_files(folder: str) -> list[str]:
    """Pose recording files of a folder in time order (ring segments and single files; one session per folder)."""
    files = glob.glob(os.path.join(folder, POSE_GLOB))
    return sorted(files, key=lambda p: (name_stamp(p) or datetime.min, os.path.basename(p)))


def first_with(folder: str, pattern: str) -> str | None:
    files = sorted(glob.glob(os.path.join(folder, pattern)))
    return files[0] if files else None


def session_log(folder: str) -> str | None:
    """The plugin log of a folder as the readers want it: the CURRENT utpp-<stamp>.log (readers add the rotated .N.log parts
    themselves). Plain sorting would put utpp-<stamp>.1.log, the older part of a rotated session, first."""
    import gt_log
    files = sorted(glob.glob(os.path.join(folder, 'utpp-2*.log')))
    current = [f for f in files if gt_log.is_session_log(f)]
    return (current or files or [None])[0]


class SessionCollector:
    def __init__(self, src_dir: str, dest_dir: str, since: datetime | None, extra_files=None, log=print,
                 now=datetime.now):
        """since: files whose name stamp is earlier are not part of the session. extra_files: () -> [path] of further
        files to copy (the mod log of the session and its rotated siblings)."""
        self.src_dir = src_dir
        self.dest_dir = dest_dir
        self.since = since
        self.extra_files = extra_files
        self.log = log
        self.now = now
        self.entries: dict[str, dict] = {}
        self.signatures: dict[str, tuple] = {}         # name -> (size, mtime_ns) of the source before the last copy

    # ---- selection
    def _candidates(self) -> list[tuple[str, str]]:
        found: list[tuple[str, str]] = []
        for p in glob.glob(os.path.join(self.src_dir, POSE_GLOB)):
            found.append((p, 'pose-ring' if is_ring(p) else 'pose'))
        found += [(p, 'feed') for p in glob.glob(os.path.join(self.src_dir, FEED_GLOB))]
        picked = []
        for p, kind in found:
            st = name_stamp(p)
            if self.since is not None and (st is None or st < self.since - timedelta(seconds=2)):
                continue
            picked.append((p, kind))
        if self.extra_files:
            try:
                picked += [(p, 'log') for p in self.extra_files() if p and os.path.isfile(p)]
            except Exception as e:  # noqa: BLE001 - a listing problem never stops the collection
                self.log(f'[collect] extra file listing failed: {e!r}')
        return picked

    # ---- copy
    def _copy_one(self, src: str, kind: str, phase: str) -> str:
        """-> 'new' | 'refreshed' | 'same' | 'lost' | 'error'. Updates self.entries."""
        name = os.path.basename(src)
        ent = self.entries.get(name)
        try:
            st = os.stat(src)
        except OSError:
            return 'gone'
        sig = (st.st_size, st.st_mtime_ns)
        if ent and ent.get('status') == 'copied' and self.signatures.get(name) == sig:
            return 'same'
        os.makedirs(self.dest_dir, exist_ok=True)
        dst = os.path.join(self.dest_dir, name)
        tmp = dst + '.part'
        try:
            n = 0
            with open(src, 'rb') as fi, open(tmp, 'wb') as fo:
                while True:
                    buf = fi.read(CHUNK)
                    if not buf:
                        break
                    fo.write(buf)
                    n += len(buf)
            os.replace(tmp, dst)
        except FileNotFoundError:
            self._drop_part(tmp)
            if ent and ent.get('status') == 'copied':
                return 'same'                          # copied earlier, the game removed it since: the copy stays
            self.entries[name] = dict(name=name, kind=kind, bytes=0, phase=phase, status='lost',
                                      note='the source disappeared before it could be copied')
            return 'lost'
        except OSError as e:
            self._drop_part(tmp)
            if getattr(e, 'winerror', None) in SHARING_ERRORS:
                # the game holds this file open for writing and does not share it (a pose segment or the single pose file
                # while it is written): it can only be copied after the game closed it. Not an error.
                if ent and ent.get('status') == 'copied':
                    ent['note'] = f'newer content not copied in phase {phase}: the file is open in the game'
                    ent['stale_copy'] = True
                else:
                    self.entries[name] = dict(name=name, kind=kind, bytes=0, phase=phase, status='busy',
                                              note='open in the game: copied after it closed the file')
                return 'busy'
            if ent and ent.get('status') == 'copied':
                ent['note'] = f'refresh failed in phase {phase}: {e!r}'
                return 'error'
            self.entries[name] = dict(name=name, kind=kind, bytes=0, phase=phase, status='error', note=repr(e))
            return 'error'
        refreshed = bool(ent and ent.get('status') == 'copied')
        self.entries[name] = dict(name=name, kind=kind, bytes=n, phase=phase, status='copied',
                                  first_phase=(ent or {}).get('first_phase', phase) if refreshed else phase,
                                  copied_at=self.now().strftime('%H:%M:%S'), note='')
        self.signatures[name] = sig
        return 'refreshed' if refreshed else 'new'

    @staticmethod
    def _drop_part(tmp: str) -> None:
        try:
            os.remove(tmp)
        except OSError:
            pass

    def collect(self, phase: str, final: bool = False) -> dict:
        """One pass. Returns {'new': [...], 'refreshed': [...], 'lost': [...], 'errors': [...]} (names)."""
        out = {'new': [], 'refreshed': [], 'lost': [], 'errors': [], 'busy': []}
        cands = self._candidates()
        rings = sorted((p for p, k in cands if k == 'pose-ring'), key=lambda p: name_stamp(p) or datetime.min)
        newest_ring = rings[-1] if rings else None
        seen = set()
        for p, kind in cands:
            name = os.path.basename(p)
            seen.add(name)
            if not final:
                if kind != 'pose-ring' or p == newest_ring:
                    continue                           # between scenarios: only the finished ring segments
            res = self._copy_one(p, kind, phase)
            if res in ('new', 'refreshed'):
                out[res].append(name)
            elif res == 'lost':
                out['lost'].append(name)
            elif res == 'error':
                out['errors'].append(name)
            elif res == 'busy':
                out['busy'].append(name)
        for name, ent in self.entries.items():         # the source dropped it after we copied it
            if name not in seen and ent.get('status') == 'copied' and not ent.get('source_gone'):
                ent['source_gone'] = phase
        if out['new'] or out['refreshed'] or out['lost'] or out['errors'] or (final and out['busy']):
            self.log(f'[collect] {phase}: copied {len(out["new"])} new, {len(out["refreshed"])} refreshed'
                     + (f', LOST {out["lost"]}' if out['lost'] else '')
                     + (f', copy errors {out["errors"]}' if out['errors'] else '')
                     + (f', still open in the game (copied after it closes them): {out["busy"]}' if out['busy'] else ''))
        return out

    # ---- report
    def manifest(self) -> list[dict]:
        return sorted((dict(e) for e in self.entries.values()), key=lambda e: (e['kind'], e['name']))

    def gaps(self) -> list[dict]:
        """Missing ring minutes between the copied ring segments (the ring dropped a segment before it was copied)."""
        stamps = sorted((name_stamp(n), n) for n, e in self.entries.items()
                        if e['kind'] == 'pose-ring' and e.get('status') == 'copied' and name_stamp(n))
        out = []
        for (a, na), (b, nb) in zip(stamps, stamps[1:]):
            d = (b - a).total_seconds()
            if d > SEGMENT_S + GAP_TOLERANCE_S:
                out.append({'after': na, 'before': nb, 'missing_s': round(d - SEGMENT_S, 1)})
        return out

    def summary(self) -> dict:
        m = self.manifest()
        copied = [e for e in m if e.get('status') == 'copied']
        return {'files': m, 'bytes': sum(e['bytes'] for e in copied), 'gaps': self.gaps(),
                'lost': [e['name'] for e in m if e.get('status') == 'lost'],
                'errors': [e['name'] for e in m if e.get('status') == 'error'],
                'busy': [e['name'] for e in m if e.get('status') == 'busy'],
                'stale_copies': [e['name'] for e in m if e.get('stale_copy')],
                'source_gone_after_copy': [e['name'] for e in m if e.get('source_gone')]}


def session_since(session_log: str | None, fallback: datetime | None = None) -> datetime | None:
    """The stamp of the session log's name (the session start), else the fallback."""
    return name_stamp(session_log) if session_log and name_stamp(session_log) else fallback


def format_manifest(summary: dict) -> list[str]:
    lines = []
    for e in summary['files']:
        lines.append(f'  {e["name"]:<48} {e["bytes"]:>10} B  {e["kind"]:<9} {e["status"]}'
                     + (f' (copied {e.get("first_phase")}' + (f', refreshed {e["phase"]}' if e.get('first_phase') != e['phase'] else '') + ')'
                        if e.get('status') == 'copied' else f' {e.get("note", "")}')
                     + (f' [source gone after {e["source_gone"]}]' if e.get('source_gone') else ''))
    for g in summary['gaps']:
        lines.append(f'  GAP: {g["missing_s"]:.0f} s missing between {g["after"]} and {g["before"]}')
    return lines


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('src')
    ap.add_argument('dest')
    ap.add_argument('--since', help='"YYYY-MM-DD HH:MM:SS": skip files whose name stamp is earlier')
    ap.add_argument('--session-log', help='the session mod log: its stamp is the default --since; it and its rotated siblings are copied')
    ap.add_argument('--json', help='write the manifest here')
    a = ap.parse_args(argv)
    since = datetime.fromisoformat(a.since) if a.since else session_since(a.session_log)
    extra = None
    if a.session_log:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import gt_log
        extra = lambda: gt_log.session_files(a.session_log)  # noqa: E731
    c = SessionCollector(a.src, a.dest, since, extra)
    c.collect('final', final=True)
    s = c.summary()
    print('\n'.join(format_manifest(s)))
    if a.json:
        with open(a.json, 'w', encoding='utf-8') as f:
            json.dump(s, f, indent=1)
    return 0 if not s['lost'] and not s['errors'] and not s['busy'] else 1


if __name__ == '__main__':
    sys.exit(main())
