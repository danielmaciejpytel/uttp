"""Cuts a short slice out of a continuous helper-bone CSV capture (utpp-bodytrace-*.csv) for the compact pose capture tests.

  python -I make_pose_capture_fixture.py INPUT.csv --out OUT.csv [--frames 120] [--skip-frames 0]

The slice keeps the header lines (qpc / `# rig_bone` topology / column names), then `--frames` whole frames (185 consecutive rows with the
same ticks) after `--skip-frames` frames, then the `# frames= dropped= failed=` trailer line of a finished capture. Streaming: only the
slice is read, so the multi-gigabyte recordings are fine as input. The default output is Build\\test-data\\posecapture\\utpp-bodytrace-slice.csv.
"""
import argparse
import pathlib
import sys

ROWS_PER_FRAME = 185
DEFAULT_OUT = pathlib.Path(__file__).resolve().parents[3] / 'Build' / 'test-data' / 'posecapture' / 'utpp-bodytrace-slice.csv'


def cut(src, dst, frames, skip_frames):
    head, rows = [], []
    wanted_rows = (skip_frames + frames) * ROWS_PER_FRAME
    with open(src, 'r', encoding='utf-8', newline='') as f:
        for line in f:
            if not rows and (line.startswith('#') or line.startswith('ticks,')):
                head.append(line)
                continue
            rows.append(line)
            if len(rows) >= wanted_rows:
                break
    rows = rows[skip_frames * ROWS_PER_FRAME:]
    if len(rows) != frames * ROWS_PER_FRAME:
        raise SystemExit(f'only {len(rows) // ROWS_PER_FRAME} whole frames available after the skip')
    for i in range(0, len(rows), ROWS_PER_FRAME):
        if len({r.split(',', 1)[0] for r in rows[i:i + ROWS_PER_FRAME]}) != 1:
            raise SystemExit(f'rows {i}..{i + ROWS_PER_FRAME - 1} are not one frame')
    dst = pathlib.Path(dst)
    dst.parent.mkdir(parents=True, exist_ok=True)
    with open(dst, 'w', encoding='utf-8', newline='') as o:
        o.writelines(head)
        o.writelines(rows)
        o.write(f'# frames={frames} dropped=0 failed=0\n')
    return dst


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('input')
    ap.add_argument('--out', default=str(DEFAULT_OUT))
    ap.add_argument('--frames', type=int, default=120)
    ap.add_argument('--skip-frames', type=int, default=0)
    a = ap.parse_args(argv)
    out = cut(a.input, a.out, a.frames, a.skip_frames)
    print(f'wrote {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
