"""Recording intake for UTPP game tests.

Finds the newest Game Bar capture (file name = recording START time), the UTPP log session that covers it, copies the
logs into Research\\logs\\rec-<date>-<time>\\, lists the events (reloads, gate decisions, jumps, vehicle exits) with
their nominal video time and writes events.tsv + summary.txt. Warns when the newest game session has no capture.

Video time = log wall time - capture start. Measured 2026-10-25: the real offset is 0.5..2.6 s later than this nominal
value (Game Bar start latency); refine it from a visible event before frame-exact work.

usage: python -I recording_intake.py [--video PATH] [--sheets N] [--no-copy-pose]
"""
import argparse
import datetime as dt
import os
import re
import shutil
import subprocess
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from utpp_paths import game_dir, local_path, project_root  # noqa: E402

ROOT = project_root()
CAPTURES = local_path("captures_dir", os.path.join(os.path.expanduser("~"), "Videos", "Captures"))
GAME_LOGS = os.path.join(game_dir(), "red4ext", "logs")
FFMPEG = os.path.join(ROOT, r"Tools\pylibs\imageio_ffmpeg\binaries\ffmpeg-win-x86_64-v7.1.exe")
SHEETS = os.path.join(ROOT, r"Source\tools\gametest\video_sheets.py")

CAP_RE = re.compile(r"(\d{4}-\d{2}-\d{2}) (\d{2})-(\d{2})-(\d{2})\.mp4$")
LOG_RE = re.compile(r"^utpp-(\d{4}-\d{2}-\d{2})-(\d{2})-(\d{2})-(\d{2})\.log$")
LINE_TIME = re.compile(r"^\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3})\]")
EVENTS = [
    ("reload", re.compile(r"RELOAD episode .*?dur=([\d.]+) rec=Items\.(\S+).*?start:loco=(\S+) upper=(\d+)")),
    ("gate", re.compile(r"\[utpp_reload_gate\] enter on=(\w+).*?why=(\S+)")),
    ("refill", re.compile(r"RELOAD refill-no-state .*?rec=Items\.(\S+) mag=(\S+) loco=(\S+)")),
    ("jump", re.compile(r"UTPP MMAIR phase=TakeOff")),
    ("land", re.compile(r"UTPP MMAIR phase=Recover")),
    ("unmount", re.compile(r"utpp_vehiclecam\].*unmount")),
]


def capture_start(path):
    m = CAP_RE.search(os.path.basename(path))
    if not m:
        return None
    return dt.datetime.strptime(f"{m.group(1)} {m.group(2)}:{m.group(3)}:{m.group(4)}", "%Y-%m-%d %H:%M:%S")


def video_seconds(path):
    try:
        out = subprocess.run([FFMPEG, "-i", path], capture_output=True, text=True, timeout=60).stderr
    except Exception:
        return None
    m = re.search(r"Duration: (\d+):(\d+):([\d.]+)", out)
    return int(m.group(1)) * 3600 + int(m.group(2)) * 60 + float(m.group(3)) if m else None


def sessions():
    """[(start, end, log path)] of the UTPP logs, newest last; end = the log file's last write."""
    out = []
    for name in os.listdir(GAME_LOGS):
        m = LOG_RE.match(name)
        if not m:
            continue
        p = os.path.join(GAME_LOGS, name)
        start = dt.datetime.strptime(f"{m.group(1)} {m.group(2)}:{m.group(3)}:{m.group(4)}", "%Y-%m-%d %H:%M:%S")
        out.append((start, dt.datetime.fromtimestamp(os.path.getmtime(p)), p))
    return sorted(out)


def session_files(start, end):
    """Pose / feed / bodytrace files written during the session."""
    lo, hi = start.timestamp() - 5, end.timestamp() + 30
    picked = []
    for name in os.listdir(GAME_LOGS):
        if not name.startswith(("utpp-pose-", "utpp-feed-", "utpp-bodytrace-")):
            continue
        p = os.path.join(GAME_LOGS, name)
        if lo <= os.path.getmtime(p) <= hi:
            picked.append(p)
    return sorted(picked)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video")
    ap.add_argument("--sheets", type=int, default=0, help="contact sheets for the first N reload episodes")
    ap.add_argument("--no-copy-pose", action="store_true")
    a = ap.parse_args()

    lines = []
    caps = sorted((p for p in (os.path.join(CAPTURES, n) for n in os.listdir(CAPTURES)) if p.endswith(".mp4") and os.path.basename(p).startswith("Cyberpunk 2077")),
                  key=os.path.getmtime)
    video = a.video or (caps[-1] if caps else None)
    sess = sessions()
    if not sess:
        print("No UTPP logs in " + GAME_LOGS)
        return 2
    newest = sess[-1]
    if video is None:
        print("No recordings in " + CAPTURES)
        return 2
    vstart = capture_start(video)
    vlen = video_seconds(video)
    if vstart is None:
        print("Cannot read the start time from the file name: " + video)
        return 2
    vend = vstart + dt.timedelta(seconds=vlen or 0)

    covering = [s for s in sess if s[0] <= vstart + dt.timedelta(seconds=5) and s[1] >= vstart]
    if not covering:
        print(f"WARNING: recording {os.path.basename(video)} (start {vstart:%H:%M:%S}) matches no log session.")
        return 2
    s_start, s_end, log = covering[-1]
    if newest[0] > s_start and newest[1] > vend + dt.timedelta(minutes=1):
        lines.append(f"WARNING: the newest game session ({newest[0]:%H:%M}-{newest[1]:%H:%M}) has NO recording - "
                     f"Game Bar may not have saved it. Using the older recording {vstart:%H:%M:%S}.")

    out_dir = os.path.join(ROOT, "Research", "logs", f"rec-{vstart:%Y%m%d-%H%M%S}")
    os.makedirs(out_dir, exist_ok=True)
    copied, skipped = [], []
    # spdlog rotates the UTPP log at 10 MB: utpp-<ts>.N.log hold the older parts (higher N = older).
    stem = log[:-4]
    parts = sorted((q for q in (os.path.join(GAME_LOGS, n) for n in os.listdir(GAME_LOGS))
                    if q.startswith(stem + ".") and re.match(r"^\d+\.log$", q[len(stem) + 1:])),
                   key=lambda q: -int(q[len(stem) + 1:-4]))
    logs = parts + [log]
    for p in logs + session_files(s_start, s_end):
        name = os.path.basename(p)
        if name.startswith("utpp-bodytrace-") or (a.no_copy_pose and name.startswith("utpp-pose-")):
            skipped.append(name)
            continue
        try:
            shutil.copy2(p, os.path.join(out_dir, name))
            copied.append(name)
        except OSError as ex:
            skipped.append(f"{name} ({ex.strerror}; is the game still writing?)")

    rows = []
    def log_lines():
        for part in logs:
            with open(part, encoding="utf-8", errors="replace") as f:
                yield from f
    if True:
        for line in log_lines():
            m = LINE_TIME.match(line)
            if not m:
                continue
            for kind, rx in EVENTS:
                e = rx.search(line)
                if not e:
                    continue
                t = dt.datetime.strptime(m.group(1), "%Y-%m-%d %H:%M:%S.%f")
                vt = (t - vstart).total_seconds()
                if kind == "reload":  # the episode line is written at the END of the reload
                    vt -= float(e.group(1))
                detail = " ".join(g for g in e.groups() if g) if e.groups() else ""
                rows.append((vt, t, kind, detail))
    rows.sort()
    inside = [r for r in rows if vlen is None or -2 <= r[0] <= vlen + 2]
    with open(os.path.join(out_dir, "events.tsv"), "w", encoding="utf-8") as f:
        f.write("video_s\tlog_time\tkind\tdetail\n")
        for vt, t, kind, detail in rows:
            f.write(f"{vt:.2f}\t{t:%H:%M:%S.%f}\t{kind}\t{detail}\n")

    def mmss(s):
        return f"{int(s // 60):02d}:{s % 60:04.1f}"

    lines.insert(0, f"Recording: {os.path.basename(video)}  start {vstart:%H:%M:%S}  length {mmss(vlen or 0)}")
    lines.append(f"Log: {os.path.basename(log)}{f' (+{len(parts)} older parts)' if parts else ''}  ({s_start:%H:%M:%S}-{s_end:%H:%M:%S})")
    lines.append(f"Folder: {out_dir}")
    lines.append(f"Copied: {', '.join(copied)}")
    if skipped:
        lines.append(f"Skipped: {', '.join(skipped)}")
    lines.append("Video time = log time - recording start (nominal; the real offset is usually +0.5..2.6 s).")
    counts = {}
    for r in inside:
        counts[r[2]] = counts.get(r[2], 0) + 1
    lines.append("Events in the recording: " + (", ".join(f"{k} {v}" for k, v in counts.items()) or "none"))
    for vt, t, kind, detail in inside:
        if kind in ("reload", "gate", "unmount", "refill"):
            lines.append(f"  {mmss(max(vt, 0))}  {t:%H:%M:%S}  {kind:8s} {detail}")

    if a.sheets:
        made = 0
        for vt, t, kind, detail in inside:
            if kind != "reload" or made >= a.sheets:
                continue
            name = f"reload{made + 1:02d}"
            subprocess.run([sys.executable, "-I", SHEETS, video, os.path.join(out_dir, "sheets"), name,
                            f"{max(vt - 0.5, 0):.2f}", f"{vt + 3.0:.2f}", "6", "--cols", "6", "--rows", "4",
                            "--tile-w", "480"], capture_output=True, timeout=300)
            made += 1
        lines.append(f"Frame sheets: {made} in {os.path.join(out_dir, 'sheets')}")

    text = "\n".join(lines)
    with open(os.path.join(out_dir, "summary.txt"), "w", encoding="utf-8") as f:
        f.write(text + "\n")
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
