"""Find byte-identical image files that exist in more than one folder of Media\\png and Media\\jpg and (with --apply)
remove the extra copies, keeping exactly one copy of every image.

Media are never deleted without an explicit decision. Run --apply only after such a decision.

What is and is not touched:
  * only .png / .jpg / .jpeg under Media\\png and Media\\jpg; never Media\\logo, never video or gif;
  * only copies in DIFFERENT folders - identical files inside one folder (e.g. a still part of a frame sequence)
    are left alone, so no sequence gets holes because of its own repeats;
  * the copy that stays: a file cited by path in the project documents first, then the copy outside a folder whose
    name contains "copies" or "scratch", then the folder that sorts first;
  * folders given with --skip-folder are ignored completely (neither kept from nor deleted from).
Writes a json report (every removed file with the copy that stays) and updates the file count and size of the affected
rows in Media\\png\\README.md and Media\\jpg\\README.md.

  python -I Source\\tools\\maintenance\\media_duplicates.py [--apply] [--report FILE] [--skip-folder NAME ...]
"""
import argparse, collections, concurrent.futures, hashlib, json, os, re, stat, sys, time

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..'))
MEDIA = os.path.join(ROOT, 'Media')
EXT = ('.png', '.jpg', '.jpeg')
DOC_DIRS = ['', 'Research']   # documents directly in the project root and in Research
DOC_EXT = ('.md', '.txt')


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        while True:
            b = f.read(1 << 22)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


def cited_paths():
    """Lower-case relative paths 'media\\png\\folder\\file' that a project document or a file in Source mentions."""
    rx = re.compile(r'Media[\\/](?:png|jpg)[\\/]([^\\/\s|`"\'<>]+)[\\/]([^\\/\s|`"\'<>)]+\.(?:png|jpe?g))', re.I)
    out = set()
    files = [os.path.join(ROOT, base, f) for base in DOC_DIRS for f in os.listdir(os.path.join(ROOT, base))
             if f.lower().endswith(DOC_EXT)]
    for base in (os.path.join(ROOT, 'Research', 'analyses'), os.path.join(ROOT, 'Source')):
        for dp, dn, fn in os.walk(base):
            if '.git' in dn:
                dn.remove('.git')
            for f in fn:
                if f.lower().endswith(('.txt', '.md', '.py', '.ps1', '.cmd', '.json', '.html')):
                    files.append(os.path.join(dp, f))
    for p in files:
        try:
            if os.path.getsize(p) > 64 << 20:
                continue
            t = open(p, encoding='utf-8', errors='replace').read()
        except OSError:
            continue
        for m in rx.finditer(t):
            out.add((m.group(1) + '\\' + m.group(2)).lower())
    return out


def main():
    ap = argparse.ArgumentParser(description='Remove identical image copies across Media folders (keeps one)')
    ap.add_argument('--apply', action='store_true')
    ap.add_argument('--report', default='')
    ap.add_argument('--skip-folder', action='append', default=[])
    ap.add_argument('--jobs', type=int, default=12)
    a = ap.parse_args()
    skip = {s.lower() for s in a.skip_folder}
    t0 = time.time()
    by_size = collections.defaultdict(list)          # size -> [(ext dir, folder, file name)]
    for ext in ('png', 'jpg'):
        base = os.path.join(MEDIA, ext)
        for folder in sorted(os.listdir(base)):
            fp = os.path.join(base, folder)
            st = os.lstat(fp)
            if not stat.S_ISDIR(st.st_mode) or st.st_file_attributes & stat.FILE_ATTRIBUTE_REPARSE_POINT or folder.lower() in skip:
                continue
            for e in os.scandir(fp):
                if e.is_file(follow_symlinks=False) and e.name.lower().endswith(EXT):
                    by_size[e.stat(follow_symlinks=False).st_size].append((ext, folder, e.name))
    cand = [(s, v) for s, v in by_size.items() if s > 0 and len({(x[0], x[1]) for x in v}) > 1]
    paths = [os.path.join(MEDIA, *x) for s, v in cand for x in v]
    with concurrent.futures.ThreadPoolExecutor(a.jobs) as ex:
        hashes = dict(zip(paths, ex.map(sha256, paths)))
    cited = cited_paths()
    groups = collections.defaultdict(list)
    for s, v in cand:
        for x in v:
            groups[(s, hashes[os.path.join(MEDIA, *x)])].append(x)

    def rank(x):
        ext, folder, name = x
        low = folder.lower()
        return (0 if (folder + '\\' + name).lower() in cited else 1,
                1 if ('copies' in low or 'scratch' in low) else 0, low, ext, name.lower())

    plan = []                                         # (keeper, removed, size, sha)
    for (s, h), v in groups.items():
        folders = {(x[0], x[1]) for x in v}
        if len(folders) < 2:
            continue
        v.sort(key=rank)
        keeper = v[0]
        kept_folders = {(keeper[0], keeper[1])}
        for x in v[1:]:
            if (x[0], x[1]) in kept_folders:
                continue                              # a repeat inside the keeper's own folder stays
            if (x[1] + '\\' + x[2]).lower() in cited:
                kept_folders.add((x[0], x[1]))        # a cited copy stays as well
                continue
            plan.append((keeper, x, s, h))
    # a repeat inside one folder: if several copies in the same non-keeper folder are listed, all go (the image stays elsewhere)
    total = sum(p[2] for p in plan)
    per_folder = collections.Counter()
    for k, x, s, h in plan:
        per_folder[(x[0], x[1])] += s
    print('%s: %d images checked by hash, %d copies in other folders to remove, %.2f GB (%.0f s)'
          % ('APPLY' if a.apply else 'DRY RUN', len(paths), len(plan), total / 2**30, time.time() - t0))
    for (ext, folder), b in per_folder.most_common(15):
        print('  %8.1f MB  Media\\%s\\%s' % (b / 2**20, ext, folder))
    removed, failed = [], []
    if a.apply:
        for k, x, s, h in plan:
            kp, xp = os.path.join(MEDIA, *k), os.path.join(MEDIA, *x)
            try:
                if not os.path.isfile(kp) or os.path.getsize(kp) != s:
                    raise OSError('the copy that stays is missing or changed')
                os.unlink(xp)
                removed.append((k, x, s, h))
            except OSError as e:
                failed.append((xp, repr(e)))
        # the copy that stays must still hash the same
        keepers = {}
        for k, x, s, h in removed:
            keepers[os.path.join(MEDIA, *k)] = h
        bad = 0
        with concurrent.futures.ThreadPoolExecutor(a.jobs) as ex:
            for (p, h), got in zip(keepers.items(), ex.map(sha256, keepers.keys())):
                bad += int(got != h)
        # README rows: | folder | files | MB |
        touched = collections.defaultdict(set)
        for k, x, s, h in removed:
            touched[x[0]].add(x[1])
        for ext, folders in touched.items():
            rp = os.path.join(MEDIA, ext, 'README.md')
            raw = open(rp, 'rb').read()
            text = raw.decode('utf-8')
            for folder in folders:
                fp = os.path.join(MEDIA, ext, folder)
                names = [e for e in os.scandir(fp) if e.is_file(follow_symlinks=False)]
                n = len(names)
                mb = sum(e.stat(follow_symlinks=False).st_size for e in names) / 1e6
                rx = re.compile(r'^\| ' + re.escape(folder) + r' \| \d+ \| [0-9.]+ \|', re.M)
                if rx.search(text):
                    text = rx.sub('| %s | %d | %.1f |' % (folder, n, mb), text, count=1)
                else:
                    print('  README row not found:', ext, folder)
            open(rp, 'wb').write(text.encode('utf-8'))
        print('removed %d, failed %d, kept copies changed %d (%.0f s)' % (len(removed), len(failed), bad, time.time() - t0))
    if a.report:
        src = removed if a.apply else plan
        json.dump({'date': time.strftime('%Y-%m-%d %H:%M'), 'applied': a.apply, 'bytes': sum(p[2] for p in src),
                   'removed': [{'removed': os.path.join('Media', *x), 'kept': os.path.join('Media', *k), 'size': s, 'sha256': h}
                               for k, x, s, h in src],
                   'failed': [{'path': os.path.relpath(p, ROOT), 'error': e} for p, e in failed]},
                  open(a.report, 'w', encoding='utf-8'), indent=1)
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
