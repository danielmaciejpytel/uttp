"""Make one named save the newest one for the game's "continue" auto-load, without changing any save content.

    python select_save.py                 list the saves with their times (newest first)
    python select_save.py NAME            back up the save NAME, remember every save's modification times, set the
                                          modification time of the files of NAME (and of its folder) to now
    python select_save.py --restore       put the remembered times back exactly and report what the game did meanwhile

Only modification times are ever written. Nothing is deleted, renamed or edited; the save content is byte-identical before
and after (the backup copy proves it). Both modes refuse to run while the game is running. A repeated select of the same
save changes nothing (the remembered originals are never overwritten by the changed times).

The saves folder is the game's folder under "Saved Games" (override: environment variable UTPP_SAVES_DIR or "saves_dir"
in local_paths.json). The state file (default Research/inventory/select_save_state.json) lives outside the saves folder
and holds save names, sizes and times only. Backups go to Backups/<date>/saves/<name> (an identical copy found in any
dated folder is reused; a different one is never overwritten).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import sys
import time
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..'))

from utpp_paths import local_path, project_root  # noqa: E402

SAVE_MARKER = 'sav.dat'          # a folder with this file is a save
STATE_VERSION = 1
GAME_SAVES_SUBDIR = os.path.join('CD Projekt Red', 'Cyberpunk 2077')
EXIT_OK, EXIT_ERROR, EXIT_GAME_CHANGED = 0, 1, 3


class SaveError(RuntimeError):
    """The request cannot be carried out; nothing was changed."""


# ------------------------------------------------------------------------------------------------ locations
def known_saved_games() -> str | None:
    """The user's "Saved Games" folder (the Windows known folder), else ~/Saved Games."""
    if sys.platform == 'win32':
        try:
            import ctypes
            from ctypes import wintypes

            class GUID(ctypes.Structure):
                _fields_ = [('a', wintypes.DWORD), ('b', wintypes.WORD), ('c', wintypes.WORD), ('d', ctypes.c_ubyte * 8)]

            # FOLDERID_SavedGames {4C5C32FF-BB9D-43B0-B5B4-2D72E54EAAA4}
            fid = GUID(0x4C5C32FF, 0xBB9D, 0x43B0, (ctypes.c_ubyte * 8)(0xB5, 0xB4, 0x2D, 0x72, 0xE5, 0x4E, 0xAA, 0xA4))
            out = ctypes.c_wchar_p()
            if ctypes.windll.shell32.SHGetKnownFolderPath(ctypes.byref(fid), 0, None, ctypes.byref(out)) == 0:
                path = out.value
                ctypes.windll.ole32.CoTaskMemFree(out)
                if path:
                    return path
        except Exception:  # noqa: BLE001 - fall back to the conventional folder
            pass
    return os.path.join(os.path.expanduser('~'), 'Saved Games')


def default_saves_dir() -> str:
    explicit = local_path('saves_dir')
    if explicit:
        return explicit
    return os.path.join(known_saved_games(), GAME_SAVES_SUBDIR)


def backups_dir() -> str:
    """The project's Backups folder; a working copy whose root has none (its Research is a link to the real project) is
    followed to the real one."""
    explicit = local_path('backups_dir')
    if explicit:
        return explicit
    root = project_root()
    direct = os.path.join(root, 'Backups')
    if os.path.isdir(direct):
        return os.path.realpath(direct)
    research = os.path.realpath(os.path.join(root, 'Research'))
    return os.path.join(os.path.dirname(research), 'Backups')


def default_state_file() -> str:
    return os.path.join(project_root(), 'Research', 'inventory', 'select_save_state.json')


def game_pids() -> list[int]:
    """Process ids of the running game (the same query the night runner uses)."""
    import gt_win32
    if not gt_win32.IS_WINDOWS:
        return []
    return gt_win32.Win32Api().list_pids(gt_win32.GAME_EXE)


# ------------------------------------------------------------------------------------------------ snapshots
def _rel(path: str, base: str) -> str:
    return os.path.relpath(path, base).replace(os.sep, '/')


def snapshot(save_dir: str) -> dict:
    """{'dir_ns': folder mtime, 'files': {relative path: [size, mtime_ns]}} of one save folder."""
    files = {}
    for dp, _dn, fns in os.walk(save_dir):
        for fn in fns:
            p = os.path.join(dp, fn)
            st = os.stat(p)
            files[_rel(p, save_dir)] = [st.st_size, st.st_mtime_ns]
    return {'dir_ns': os.stat(save_dir).st_mtime_ns, 'files': files}


def newest_ns(snap: dict) -> int:
    return max([v[1] for v in snap['files'].values()] or [snap['dir_ns']])


def scan_saves(saves_dir: str) -> dict[str, dict]:
    """Every save folder (a folder with SAVE_MARKER) -> its snapshot."""
    out = {}
    if not os.path.isdir(saves_dir):
        raise SaveError('the saves folder does not exist')
    for name in sorted(os.listdir(saves_dir)):
        d = os.path.join(saves_dir, name)
        if os.path.isdir(d) and os.path.isfile(os.path.join(d, SAVE_MARKER)):
            out[name] = snapshot(d)
    return out


def fmt_ns(ns: int) -> str:
    return datetime.fromtimestamp(ns / 1e9).strftime('%Y-%m-%d %H:%M:%S')


# ------------------------------------------------------------------------------------------------ state file
def load_state(path: str) -> dict | None:
    try:
        with open(path, encoding='utf-8') as f:
            st = json.load(f)
    except FileNotFoundError:
        return None
    except (OSError, ValueError) as e:
        raise SaveError(f'the state file is unreadable ({e.__class__.__name__}): nothing was changed') from e
    if not isinstance(st, dict) or st.get('version') != STATE_VERSION or 'chosen' not in st:
        raise SaveError('the state file has an unknown format: nothing was changed')
    return st


def write_state(path: str, state: dict) -> None:
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(state, f, indent=1, sort_keys=True)
    os.replace(tmp, path)


# ------------------------------------------------------------------------------------------------ backup
def _sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def same_tree(a: str, b: str) -> bool:
    """The same relative files with the same content (size first, then SHA-256)."""
    sa, sb = snapshot(a)['files'], snapshot(b)['files']
    if set(sa) != set(sb) or any(sa[k][0] != sb[k][0] for k in sa):
        return False
    return all(_sha256(os.path.join(a, k)) == _sha256(os.path.join(b, k)) for k in sa)


def backup_save(src: str, name: str, backups: str, today: str) -> tuple[str, str]:
    """Copy the save folder to <backups>/<today>/saves/<name> unless an identical copy exists in any dated folder.
    -> ('exists' | 'copied', path relative to the backups folder's parent). The copy keeps the file times."""
    if os.path.isdir(backups):
        for day in sorted(os.listdir(backups)):
            cand = os.path.join(backups, day, 'saves', name)
            if os.path.isdir(cand) and same_tree(src, cand):
                return 'exists', os.path.join(os.path.basename(backups), day, 'saves', name)
    base = os.path.join(backups, today, 'saves', name)
    dest, n = base, 1
    while os.path.exists(dest):                  # a different copy is never overwritten
        n += 1
        dest = f'{base}-{n}'
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    shutil.copytree(src, dest, copy_function=shutil.copy2)
    if not same_tree(src, dest):
        raise SaveError('the backup copy differs from the save: nothing was changed')
    return 'copied', os.path.join(os.path.basename(backups), os.path.relpath(dest, backups))


# ------------------------------------------------------------------------------------------------ operations
def _set_times(save_dir: str, files: dict, mtime_ns: int | dict, dir_ns: int) -> None:
    """Set the modification time of the listed files and of the folder (the access time stays as it is)."""
    for rel in files:
        p = os.path.join(save_dir, *rel.split('/'))
        ns = mtime_ns[rel] if isinstance(mtime_ns, dict) else mtime_ns
        os.utime(p, ns=(os.stat(p).st_atime_ns, ns))
    os.utime(save_dir, ns=(os.stat(save_dir).st_atime_ns, dir_ns))


def list_saves(saves_dir: str, out=print) -> list[str]:
    saves = scan_saves(saves_dir)
    rows = sorted(saves.items(), key=lambda kv: newest_ns(kv[1]), reverse=True)
    lines = []
    for i, (name, snap) in enumerate(rows):
        lines.append(f'{name}  {fmt_ns(newest_ns(snap))}  {len(snap["files"])} files' + ('  <- newest' if i == 0 else ''))
    for line in lines or ['no saves found']:
        out(line)
    return lines


def do_select(saves_dir: str, name: str, state_path: str, backups: str, running=game_pids, now_ns: int | None = None,
              today: str | None = None, out=print) -> int:
    pids = running()
    if pids:
        raise SaveError(f'the game is running (process {pids[0]}): close it first, nothing was changed')
    saves = scan_saves(saves_dir)
    if name not in saves:
        raise SaveError(f'no save named {name}: nothing was changed (known: {", ".join(saves) or "none"})')
    state = load_state(state_path)
    if state is not None:
        if state['chosen'] != name:
            raise SaveError(f'{state["chosen"]} is still selected: run --restore first, nothing was changed')
        cur = saves[name]
        applied = state['applied_ns']
        if cur['files'].keys() == state['saves'][name]['files'].keys() and all(
                v[1] == applied for v in cur['files'].values()):
            for n, snap in sorted(saves.items()):
                out(f'{n}  {fmt_ns(newest_ns(snap))}  ' + ('already selected, nothing changed' if n == name else 'unchanged'))
            return EXIT_OK
        raise SaveError(f'{name} was changed since it was selected: run --restore first, nothing was changed')
    now = now_ns if now_ns is not None else time.time_ns()
    others = [newest_ns(s) for n, s in saves.items() if n != name]
    now = max(now, max(others) + 1000 if others else now)     # strictly newer than every other save
    now -= now % 100                                          # the file system keeps 100 ns steps
    how, where = backup_save(os.path.join(saves_dir, name), name, backups, today or datetime.now().strftime('%Y-%m-%d'))
    new_state = {'version': STATE_VERSION, 'chosen': name, 'applied_ns': now, 'backup': where,
                 'saves': {n: s for n, s in saves.items()}}
    write_state(state_path, new_state)                         # the originals are on disk before any time is touched
    chosen = saves[name]
    _set_times(os.path.join(saves_dir, name), chosen['files'], now, now)
    for n, snap in sorted(saves.items()):
        if n == name:
            out(f'{n}  {fmt_ns(newest_ns(snap))} -> {fmt_ns(now)}  selected (backup {how}: {where})')
        else:
            out(f'{n}  {fmt_ns(newest_ns(snap))}  unchanged')
    return EXIT_OK


def do_restore(saves_dir: str, state_path: str, running=game_pids, out=print) -> int:
    pids = running()
    if pids:
        raise SaveError(f'the game is running (process {pids[0]}): close it first, nothing was changed')
    state = load_state(state_path)
    if state is None:
        out('nothing to restore (no state file)')
        return EXIT_OK
    saves = scan_saves(saves_dir)
    name, applied, orig = state['chosen'], state['applied_ns'], state['saves']
    code = EXIT_OK
    lines = {}
    for n, snap in saves.items():
        if n not in orig:
            lines[n] = f'{n}  {fmt_ns(newest_ns(snap))}  created meanwhile (by the game), left alone'
            code = EXIT_GAME_CHANGED
            continue
        before = orig[n]['files']
        if n == name:
            same = snap['files'].keys() == before.keys() and all(
                snap['files'][k][0] == before[k][0] and snap['files'][k][1] == applied for k in before)
            if same:
                _set_times(os.path.join(saves_dir, n), before, {k: v[1] for k, v in before.items()}, orig[n]['dir_ns'])
                lines[n] = f'{n}  {fmt_ns(applied)} -> {fmt_ns(newest_ns(orig[n]))}  restored'
            else:
                lines[n] = f'{n}  {fmt_ns(newest_ns(snap))}  changed by the game since it was selected, time left alone'
                code = EXIT_GAME_CHANGED
        elif snap['files'] != before:
            lines[n] = f'{n}  {fmt_ns(newest_ns(snap))}  changed by the game meanwhile, left alone'
            code = EXIT_GAME_CHANGED
        else:
            lines[n] = f'{n}  {fmt_ns(newest_ns(snap))}  unchanged'
    for n in orig:
        if n not in saves:
            lines[n] = f'{n}  missing now (removed meanwhile), nothing to restore'
            code = EXIT_GAME_CHANGED
    for n in sorted(lines):
        out(lines[n])
    os.remove(state_path)                                     # everything that could be restored has been
    return code


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('name', nargs='?', help='the save folder to make the newest (without it: list the saves)')
    p.add_argument('--restore', action='store_true', help='put the remembered modification times back')
    p.add_argument('--saves-dir', default=None, help='the game\'s saves folder (default: under Saved Games)')
    p.add_argument('--state-file', default=None, help='where the original times are kept')
    p.add_argument('--backup-dir', default=None, help='the Backups folder (default: the project\'s)')
    return p


def main(argv=None, running=game_pids, out=print, now_ns: int | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.restore and args.name:
        out('--restore takes no save name')
        return EXIT_ERROR
    try:
        saves_dir = args.saves_dir or default_saves_dir()
        state = args.state_file or default_state_file()
        if args.restore:
            return do_restore(saves_dir, state, running, out)
        if args.name:
            return do_select(saves_dir, args.name, state, args.backup_dir or backups_dir(), running, now_ns, out=out)
        list_saves(saves_dir, out)
        return EXIT_OK
    except SaveError as e:
        out(f'error: {e}')
        return EXIT_ERROR


if __name__ == '__main__':
    sys.exit(main())
