"""Replace identical files inside Backups with NTFS hardlinks (project rule: identical large files may be hardlinked
only between frozen locations; Backups is one, see Backups\\README.md - restore by copying, never edit in place).

Dry run by default: prints how many files and bytes would be linked. --apply does it.
Only files under <project>\\Backups are read or changed. Files are grouped by size, then by SHA-256; files that already
share a file id are left alone. A copy is replaced like this: a hardlink to the group's first file is created next to
the copy under a temporary name, then renamed over the copy (one atomic step), so a failure never loses the file.
After --apply every changed path is checked again (same file id as the first file, SHA-256 unchanged).

  python -I Source\\tools\\maintenance\\hardlink_backups.py [--apply] [--min-size BYTES] [--report FILE]
"""
import argparse, collections, concurrent.futures, hashlib, json, os, stat, sys, time

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..'))
BACKUPS = os.path.join(ROOT, 'Backups')
TMP_SUFFIX = '.hardlink_tmp'


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        while True:
            b = f.read(1 << 22)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


def main():
    ap = argparse.ArgumentParser(description='Hardlink identical files inside Backups')
    ap.add_argument('--apply', action='store_true')
    ap.add_argument('--min-size', type=int, default=1 << 20, help='ignore smaller files (default 1 MiB)')
    ap.add_argument('--report', default='', help='write a json report here')
    ap.add_argument('--jobs', type=int, default=8)
    a = ap.parse_args()
    if not os.path.isdir(BACKUPS):
        sys.exit('no Backups folder: ' + BACKUPS)
    t0 = time.time()
    by_size = collections.defaultdict(list)       # size -> [(path, file id)]
    stack = [BACKUPS]
    while stack:
        d = stack.pop()
        try:
            entries = list(os.scandir(d))
        except OSError:
            continue
        for e in entries:
            st = os.lstat(e.path)                 # os.lstat gives the file id and link count on Windows; DirEntry.stat does not
            if st.st_file_attributes & stat.FILE_ATTRIBUTE_REPARSE_POINT:
                continue
            if stat.S_ISDIR(st.st_mode):
                stack.append(e.path)
            elif st.st_size >= a.min_size and not e.name.endswith(TMP_SUFFIX):
                by_size[st.st_size].append((e.path, (st.st_dev, st.st_ino)))
    cand = {s: v for s, v in by_size.items() if len({i for _, i in v}) > 1}
    # one hash per distinct file id
    todo = {}
    for s, v in cand.items():
        for p, i in v:
            todo.setdefault(i, p)
    with concurrent.futures.ThreadPoolExecutor(a.jobs) as ex:
        hashes = dict(zip(todo.keys(), ex.map(sha256, todo.values())))
    groups = collections.defaultdict(list)        # (size, sha) -> [(path, id)]
    for s, v in cand.items():
        for p, i in v:
            groups[(s, hashes[i])].append((p, i))
    plan = []                                     # (master path, copy path, size, sha)
    for (s, h), v in groups.items():
        v.sort(key=lambda x: x[0].lower())
        master_path, master_id = v[0]
        for p, i in v[1:]:
            if i != master_id:
                plan.append((master_path, p, s, h))
    saved = 0
    seen = set()
    for m, p, s, h in plan:                       # bytes freed = one copy per distinct file id that disappears
        i = os.lstat(p)
        k = (i.st_dev, i.st_ino)
        if k not in seen:
            seen.add(k)
            saved += s
    print('%s: %d files in Backups >= %d bytes, %d identical groups, %d files to link, about %.2f GB logical freed (%.0f s)'
          % ('APPLY' if a.apply else 'DRY RUN', sum(len(v) for v in by_size.values()), a.min_size,
             sum(1 for v in groups.values() if len({i for _, i in v}) > 1), len(plan), saved / 2**30, time.time() - t0))
    done, failed = [], []
    if a.apply:
        for m, p, s, h in plan:
            tmp = p + TMP_SUFFIX
            try:
                if os.path.lexists(tmp):
                    os.unlink(tmp)
                os.link(m, tmp)
                try:
                    os.replace(tmp, p)
                except PermissionError:
                    os.chmod(p, stat.S_IWRITE)
                    os.replace(tmp, p)
                done.append((m, p, s, h))
            except OSError as e:
                failed.append((p, repr(e)))
                if os.path.lexists(tmp):
                    try:
                        os.unlink(tmp)
                    except OSError:
                        pass
        # verification: same file id as the first file of the group, content hash unchanged
        bad = 0
        masters = {}
        for m, p, s, h in done:
            sm, sp = os.lstat(m), os.lstat(p)
            if (sm.st_dev, sm.st_ino) != (sp.st_dev, sp.st_ino):
                bad += 1
            masters[m] = h
        with concurrent.futures.ThreadPoolExecutor(a.jobs) as ex:
            for (m, h), got in zip(masters.items(), ex.map(sha256, masters.keys())):
                if got != h:
                    bad += 1
        print('linked %d, failed %d, verification problems %d (%.0f s)' % (len(done), len(failed), bad, time.time() - t0))
        for p, e in failed[:20]:
            print('  failed', os.path.relpath(p, ROOT), e)
    if a.report:
        json.dump({'date': time.strftime('%Y-%m-%d %H:%M'), 'applied': a.apply, 'min_size': a.min_size,
                   'logical_bytes_freed': saved,
                   'links': [{'first': os.path.relpath(m, ROOT), 'copy': os.path.relpath(p, ROOT), 'size': s, 'sha256': h}
                             for m, p, s, h in (done if a.apply else plan)],
                   'failed': [{'path': os.path.relpath(p, ROOT), 'error': e} for p, e in failed]},
                  open(a.report, 'w', encoding='utf-8'), indent=1)
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
