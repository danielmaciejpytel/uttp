"""Space report (read only): where the project's disk space is, how much transparent compression and
hardlink de-duplication could save. Samples files per group/extension (zlib level 1 on the first 8 MB as a
conservative stand-in for NTFS compression) and hashes equal-size files >= 16 MB to find duplicates."""
import hashlib, json, os, random, stat, sys, time, zlib, collections

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))   # the tools folder
from utpp_paths import project_root  # noqa: E402

ROOT = project_root()
GROUPS = [r"Extracted\game", r"Extracted\reverse", r"Extracted", "Media", r"Tools\ai-motion", "Tools", "Backups",
          r"Build\work", r"Build\UTPP", r"Build\stages", "Build", "Animations", "Archives", r"Research\logs", "Research",
          "Data", r"Source\.git", "Source"]
random.seed(7)
t0 = time.time()


def group_of(rel):
    low = rel.lower()
    for g in GROUPS:
        if low == g.lower() or low.startswith(g.lower() + "\\"):
            return g
    return "(root)"


files = collections.defaultdict(list)   # group -> [(path, size, ext, ino)]
stack = [ROOT]
while stack:
    d = stack.pop()
    try:
        it = list(os.scandir(d))
    except OSError:
        continue
    for e in it:
        try:
            st = e.stat(follow_symlinks=False)
        except OSError:
            continue
        if st.st_file_attributes & stat.FILE_ATTRIBUTE_REPARSE_POINT:
            continue
        if e.is_dir(follow_symlinks=False):
            stack.append(e.path)
        else:
            rel = os.path.relpath(e.path, ROOT)
            files[group_of(rel)].append((e.path, st.st_size, os.path.splitext(e.name)[1].lower(), bool(st.st_file_attributes & stat.FILE_ATTRIBUTE_COMPRESSED)))
print("scan %.0f s, files %d" % (time.time() - t0, sum(len(v) for v in files.values())), flush=True)

out = {}
for g, lst in files.items():
    total = sum(s for _, s, _, _ in lst)
    already = sum(s for _, s, _, c in lst if c)
    by_ext = collections.defaultdict(list)
    for p, s, x, c in lst:
        by_ext[x].append((p, s))
    est_saved = 0.0
    ext_rows = []
    for x, fl in by_ext.items():
        tb = sum(s for _, s in fl)
        if tb < 0.01 * total and tb < 2e8:
            continue
        sample = random.sample(fl, min(len(fl), 25))
        raw = comp = 0
        for p, s in sample:
            try:
                with open(p, "rb") as f:
                    b = f.read(8 << 20)
            except OSError:
                continue
            raw += len(b)
            comp += len(zlib.compress(b, 1))
        ratio = comp / raw if raw else 1.0
        est_saved += tb * (1 - min(ratio, 1.0))
        ext_rows.append((x or "(none)", round(tb / 1e9, 2), round(ratio, 2)))
    ext_rows.sort(key=lambda r: -r[1])
    out[g] = {"gb": round(total / 1e9, 2), "files": len(lst), "ntfs_compressed_gb": round(already / 1e9, 2), "est_saved_gb": round(est_saved / 1e9, 2), "ext": ext_rows[:8]}
    print(g, json.dumps(out[g]), flush=True)

# duplicates (not in Media): equal size >= 16 MB, same sha1, different inode
cand = collections.defaultdict(list)
for g, lst in files.items():
    for p, s, x, c in lst:
        if s >= 16 << 20:
            cand[s].append(p)
dup_bytes = 0
dup_groups = []
for s, ps in cand.items():
    if len(ps) < 2:
        continue
    by_hash = collections.defaultdict(list)
    for p in ps:
        try:
            st = os.stat(p)
            h = hashlib.sha1()
            with open(p, "rb") as f:
                for chunk in iter(lambda: f.read(1 << 22), b""):
                    h.update(chunk)
            by_hash[h.hexdigest()].append((p, st.st_ino))
        except OSError:
            pass
    for h, lst in by_hash.items():
        inos = {i for _, i in lst}
        if len(inos) > 1:
            dup_bytes += s * (len(inos) - 1)
            dup_groups.append((s * (len(inos) - 1), s, [os.path.relpath(p, ROOT) for p, _ in lst][:6]))
dup_groups.sort(reverse=True)
print("DUP reclaimable GB: %.2f in %d groups" % (dup_bytes / 1e9, len(dup_groups)))
agg = collections.Counter()
for rb, s, ps in dup_groups:
    key = " + ".join(sorted({"\\".join(p.split("\\")[:2]) for p in ps}))
    agg[key] += rb
for k, v in agg.most_common(15):
    print("  %6.2f GB  %s" % (v / 1e9, k))
for rb, s, ps in dup_groups[:12]:
    print("  dup %.2f GB (file %.2f GB): %s" % (rb / 1e9, s / 1e9, " | ".join(ps[:3])))
json.dump({"groups": out, "dup_gb": dup_bytes / 1e9}, open(os.path.join(ROOT, r"Research\inventory\space-report.json"), "w"), indent=1)
print("done %.0f s" % (time.time() - t0))
