"""Transparent NTFS compression for project folders (space saving, no path or content change).

usage: python -I compress_folders.py [--threads N] [--dry-run] FOLDER [FOLDER ...]      (folders relative to the project root)

Every directory gets the compression attribute (new files are then compressed automatically); existing files are
compressed in place. Never follows reparse points (junctions / symlinks), never leaves the project folder, skips formats
that do not compress (video, images, archives), files below 8 KiB and files that are in use. Runs at background
priority. Safe to re-run: compressed files are skipped. Totals go to Research\\inventory\\compress-<time>.csv.
"""
import argparse, concurrent.futures as cf, csv, ctypes, os, stat, sys, time
from ctypes import wintypes as wt

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
SKIP_EXT = {".mp4", ".png", ".jpg", ".jpeg", ".gif", ".mkv", ".webp", ".zip", ".7z", ".gz", ".xz", ".zst", ".whl", ".pack",
            ".xbm", ".bk2", ".mlmask", ".archive", ".anims", ".blend", ".blend1", ".wem", ".opus", ".ogg"}
NEVER = ("media", "source", "data", r"build\utpp", r"build\stages", r"build\test-data")

k32 = ctypes.WinDLL("kernel32", use_last_error=True)
k32.CreateFileW.restype = wt.HANDLE
k32.CreateFileW.argtypes = [wt.LPCWSTR, wt.DWORD, wt.DWORD, ctypes.c_void_p, wt.DWORD, wt.DWORD, wt.HANDLE]
k32.DeviceIoControl.argtypes = [wt.HANDLE, wt.DWORD, ctypes.c_void_p, wt.DWORD, ctypes.c_void_p, wt.DWORD, ctypes.POINTER(wt.DWORD), ctypes.c_void_p]
k32.GetCompressedFileSizeW.restype = wt.DWORD
k32.GetCompressedFileSizeW.argtypes = [wt.LPCWSTR, ctypes.POINTER(wt.DWORD)]
k32.CloseHandle.argtypes = [wt.HANDLE]
INVALID = wt.HANDLE(-1).value
FSCTL_SET_COMPRESSION = 0x9C040
A_COMPRESSED, A_REPARSE, A_ENCRYPTED, A_READONLY = 0x800, 0x400, 0x4000, 0x1


def lp(p):
    return "\\\\?\\" + p if not p.startswith("\\\\?\\") else p


def set_compression(path, is_dir):
    h = k32.CreateFileW(lp(path), 0xC0000000, 0x7, None, 3, 0x02000000, None)
    if h == INVALID or h is None:
        return ctypes.get_last_error() or 1
    try:
        fmt = ctypes.c_ushort(1)
        n = wt.DWORD(0)
        ok = k32.DeviceIoControl(h, FSCTL_SET_COMPRESSION, ctypes.byref(fmt), 2, None, 0, ctypes.byref(n), None)
        return 0 if ok else (ctypes.get_last_error() or 1)
    finally:
        k32.CloseHandle(h)


def disk_size(path):
    hi = wt.DWORD(0)
    lo = k32.GetCompressedFileSizeW(lp(path), ctypes.byref(hi))
    if lo == 0xFFFFFFFF and ctypes.get_last_error():
        return None
    return (hi.value << 32) | lo


def one_file(item):
    path, size, attrs = item
    before = disk_size(path)
    ro = bool(attrs & A_READONLY)
    try:
        if ro:
            os.chmod(path, stat.S_IWRITE | stat.S_IREAD)
        err = set_compression(path, False)
    finally:
        if ro:
            try:
                os.chmod(path, stat.S_IREAD)
            except OSError:
                pass
    after = disk_size(path)
    return path, size, before, after, err


def walk(top):
    """Yield (dirs, files) without ever entering a reparse point."""
    stack = [top]
    while stack:
        d = stack.pop()
        try:
            entries = list(os.scandir(d))
        except OSError:
            continue
        for e in entries:
            try:
                st = e.stat(follow_symlinks=False)
            except OSError:
                continue
            a = st.st_file_attributes
            if a & A_REPARSE:
                continue
            if e.is_dir(follow_symlinks=False):
                stack.append(e.path)
                yield "d", e.path, 0, a
            else:
                yield "f", e.path, st.st_size, a


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--threads", type=int, default=6)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("folders", nargs="+")
    a = ap.parse_args()
    k32.SetPriorityClass(k32.GetCurrentProcess(), 0x00100000)   # PROCESS_MODE_BACKGROUND_BEGIN: low CPU and I/O priority
    rows = []
    for rel in a.folders:
        rel = rel.strip("\\/")
        top = os.path.normpath(os.path.join(ROOT, rel))
        low = os.path.relpath(top, ROOT).lower()
        if not top.lower().startswith(ROOT.lower() + "\\") or any(low == n or low.startswith(n + "\\") for n in NEVER):
            print("refused (outside the project or excluded):", rel)
            continue
        if not os.path.isdir(top) or (os.lstat(top).st_file_attributes & A_REPARSE):
            print("skipped (missing or a link):", rel)
            continue
        t0 = time.time()
        todo, ndir, nskip, total = [], 0, 0, 0
        if not a.dry_run and not (os.lstat(top).st_file_attributes & A_COMPRESSED):
            set_compression(top, True)
        for kind, p, size, attrs in walk(top):
            if kind == "d":
                ndir += 1
                if not a.dry_run and not (attrs & A_COMPRESSED):
                    set_compression(p, True)
                continue
            total += size
            if attrs & (A_COMPRESSED | A_ENCRYPTED) or size < 8192 or os.path.splitext(p)[1].lower() in SKIP_EXT:
                nskip += 1
                continue
            todo.append((p, size, attrs))
        print("%s: %d dirs, %d files to compress (%.1f GB), %d skipped, folder total %.1f GB" % (
            rel, ndir, len(todo), sum(s for _, s, _ in todo) / 1e9, nskip, total / 1e9), flush=True)
        saved = done = failed = 0
        errs = []
        if not a.dry_run:
            with cf.ThreadPoolExecutor(a.threads) as ex:
                for i, (p, size, before, after, err) in enumerate(ex.map(one_file, todo, chunksize=64)):
                    if err:
                        failed += 1
                        if len(errs) < 50:
                            errs.append((os.path.relpath(p, ROOT), err))
                    else:
                        done += 1
                        if before is not None and after is not None:
                            saved += before - after
                    if i and i % 50000 == 0:
                        print("  %s: %d/%d files, saved %.1f GB, %.0f s" % (rel, i, len(todo), saved / 1e9, time.time() - t0), flush=True)
        rows.append((rel, ndir, len(todo), done, failed, nskip, total, saved, round(time.time() - t0)))
        print("%s: compressed %d, failed %d, saved %.2f GB in %.0f s" % (rel, done, failed, saved / 1e9, time.time() - t0), flush=True)
        for p, e in errs[:8]:
            print("   not compressed (error %d): %s" % (e, p))
    if rows and not a.dry_run:
        out = os.path.join(ROOT, r"Research\inventory", "compress-%s.csv" % time.strftime("%Y%m%d-%H%M%S"))
        with open(out, "w", encoding="utf-8", newline="") as f:
            w = csv.writer(f, delimiter=";")
            w.writerow(["folder", "dirs", "files_selected", "compressed", "failed", "skipped", "folder_bytes", "saved_bytes", "seconds"])
            w.writerows(rows)
        print("log:", out)


main()
