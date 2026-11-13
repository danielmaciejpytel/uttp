"""Passenger seated-idle sets of the UTPP car pipeline: utpp_car_p?a_<class>_passenger_r_idle.anims (clip 'tpp_idle').

Why (2026-11-01, body trace 15:25 sport1 passenger): ~0.4 s after a TPP passenger sits down the game asks for the clip 'tpp_idle' by name.
No passenger set carries one, so the lookup falls through the workspot's animset list to player_default_vehicle.anims (priority 254, no
variables), a fixed driver-like pose (mean 0.14 deg against the traced pose, Hips identical) that is 6-14 cm off the passenger's seated
pose. The class' own passenger 'idle' (3.67 s breathing loop, Hips <= 1.2 cm, closed) is the pose the passenger belongs in.

What this builds: for every class and gender the vanilla player set base\\animations\\vehicle\\<dir>\\character\\p?a_<class>_passenger_r.anims
reduced to its 'idle' clip and renamed 'tpp_idle' (animset_subset.py --keep idle --rename idle=tpp_idle). A pure data copy: nothing is
re-encoded, so the clip equals the vanilla idle exactly. make_car_workspot.py registers the sets with the passenger 'enter' variables.
The passenger 'enter' clips must END on frame 0 of this idle (build_entry_clip.py --end-pose <set glb>:idle, checked by
check_car_entry.py --tpp-idle <set glb> --tpp-idle-clip idle): the vanilla enter's last frame is 0 - 10.5 cm off it (standard3 10.5,
sport1 foot 11 cm, male sets of sport2 / utility too).

Checks (exit code != 0 otherwise): the subset JSON keeps exactly the clip 'tpp_idle' with the duration / frame count of the vanilla idle; the
built .anims serialized back equals the subset JSON (Header ignored); the WolvenKit export of the built set equals the export of the
vanilla idle clip (max local translation / rotation difference of every frame, tolerance 1e-5); --selftest mutates the built set's export
(one bone, one frame) and the comparison must reject it.

Usage: python -I make_passenger_idle.py --out <new dir under Build\\work\\vehicle-entry-car> [--classes standard2,sport1,...] [--selftest]
Output: <out>\\utpp_vehicle_car\\utpp\\vehicle\\utpp_car_p?a_<class>_passenger_r_idle.anims (the layout pack_car_workspot.ps1 -AnimsRun reads),
        <out>\\json, <out>\\export, <out>\\report.json
"""
import argparse
import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import skel as S  # noqa: E402
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from utpp_paths import project_root  # noqa: E402

ROOT = Path(project_root())
WKIT = ROOT / "Tools" / "WolvenKit.CLI" / "WolvenKit.CLI.exe"
EX = ROOT / "Extracted" / "game" / "vehicle-entry-car"
CLASSES = ("standard2", "standard25", "standard3", "sport1", "sport2", "utility")
TOL = 1e-5


def run(args, cwd=None):
    r = subprocess.run([str(a) for a in args], capture_output=True, text=True, cwd=cwd)
    out = (r.stdout or "") + (r.stderr or "")
    if r.returncode != 0:
        raise SystemExit("command failed (%d): %s\n%s" % (r.returncode, " ".join(str(a) for a in args), out[-1500:]))
    return out


def body(doc):
    """the document without Header, buffer / handle ids renumbered by order of appearance (WolvenKit renumbers them on a round trip)."""
    bids, hids = {}, {}

    def walk(v):
        if isinstance(v, dict):
            out = {}
            for k, x in v.items():
                if k in ("BufferId", "BufferRefId") and isinstance(x, str):
                    out[k] = bids.setdefault(x, str(len(bids)))
                elif k in ("HandleId", "HandleRefId") and isinstance(x, str):
                    out[k] = hids.setdefault(x, str(len(hids)))
                else:
                    out[k] = walk(x)
            return out
        if isinstance(v, list):
            return [walk(x) for x in v]
        return v
    return walk({k: v for k, v in doc.items() if k != "Header"})


def clips_of(doc):
    return [(e["Data"]["animation"]["Data"]["name"]["$value"], e["Data"]["animation"]["Data"].get("duration")) for e in doc["Data"]["RootChunk"]["animations"]]


def clip_arrays(glb, clip):
    sk = S.Skeleton.from_glb(str(glb))
    c = sk.clip(clip)
    ch = S.G.channels(sk.gj, sk.binb, c)
    dur = max(float(tr[0][-1]) for b, d in ch.items() if b != "Root" for tr in d.values())
    t = np.arange(int(round(dur * 30)) + 1) / 30.0
    lt, lr = sk.sample_local(c, t)
    return sk, dur, lt, S.qcontinuous(lr)


def compare_arrays(lta, lra, ltb, lrb):
    """max local translation (m) and rotation (deg) difference over all frames and bones of two sampled clips of the same rig."""
    if lta.shape != ltb.shape:
        return dict(ok=False, why="frame count differs: %s vs %s" % (lta.shape, ltb.shape))
    dt = float(np.abs(lta - ltb).max())
    # angle from the chord (4 asin(|qa - qb| / 2), sign-folded): arccos of a dot product near 1 reports 0.03 deg for identical float32 quaternions
    chord = np.minimum(np.linalg.norm(lra - lrb, axis=-1), np.linalg.norm(lra + lrb, axis=-1))
    dr = float(np.degrees(4 * np.arcsin(np.clip(chord / 2, 0, 1))).max())
    return dict(ok=bool(dt <= TOL and dr <= 1e-3), max_dt_m=dt, max_dr_deg=dr, frames=int(lta.shape[0]))


def compare(glb_a, clip_a, glb_b, clip_b):
    ska, da, lta, lra = clip_arrays(glb_a, clip_a)
    skb, db, ltb, lrb = clip_arrays(glb_b, clip_b)
    if ska.names != skb.names:
        return dict(ok=False, why="rig differs")
    r = compare_arrays(lta, lra, ltb, lrb)
    r["duration_s"] = da
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--classes", default=",".join(CLASSES))
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    out = Path(a.out).resolve()
    if not str(out).startswith(str(ROOT) + "\\"):
        raise SystemExit("--out outside the project")
    if out.exists():
        raise SystemExit("output exists, choose a new --out: %s" % out)
    depot = out / "utpp_vehicle_car" / "utpp" / "vehicle"
    jdir, edir, rdir = out / "json", out / "export", out / "roundtrip"
    for d in (depot, jdir, edir, rdir):
        d.mkdir(parents=True)
    report = {}
    bad = 0
    for cls in a.classes.split(","):
        for g in "wm":
            van = "p%sa_%s_passenger_r" % (g, cls)
            name = "utpp_car_%s_idle" % van
            vjson = EX / "json" / (van + ".anims.json")
            vglb = EX / "glb" / (van + ".anims.glb")
            if not vjson.is_file() or not vglb.is_file():
                raise SystemExit("missing extraction: %s / %s" % (vjson, vglb))
            sub_json = jdir / (name + ".anims.json")
            run([sys.executable, "-I", HERE / "animset_subset.py", vjson, sub_json, "--keep", "idle", "--rename", "idle=tpp_idle"])
            doc = json.loads(sub_json.read_text(encoding="utf-8"))
            cl = clips_of(doc)
            vcl = dict(clips_of(json.loads(vjson.read_text(encoding="utf-8"))))
            ok_clip = [c[0] for c in cl] == ["tpp_idle"] and cl[0][1] == vcl["idle"]
            run([WKIT, "convert", "deserialize", sub_json, "-o", depot])
            built = depot / (name + ".anims")
            if not built.is_file():
                raise SystemExit("deserialize produced no " + str(built))
            run([WKIT, "convert", "serialize", built, "-o", rdir])
            rt = json.loads((rdir / (name + ".anims.json")).read_text(encoding="utf-8"))
            rt_equal = body(rt) == body(doc)
            run(["powershell", "-NoProfile", "-File", HERE / "export_glb.ps1", "-Files", built, "-OutDir", edir, "-RootMotion"])
            egl = edir / (name + ".anims.glb")
            cmp_ = compare(vglb, "idle", egl, "tpp_idle")
            sha = hashlib.sha256(built.read_bytes()).hexdigest()
            ok = bool(ok_clip and rt_equal and cmp_["ok"])
            st = ""
            if a.selftest and cl:
                # mutation: the export's tpp_idle with one bone moved by 1 cm on one frame must be rejected by compare()
                sk, dur, lt, lr = clip_arrays(egl, "tpp_idle")
                lt2 = lt.copy()
                lt2[7, sk.index["Hips"], 1] += 0.01
                rej = not compare_arrays(lt, lr, lt2, lr)["ok"]
                lr2 = lr.copy()
                lr2[9, sk.index["Head"]] = lr2[9, sk.index["Spine3"]]  # a different rotation on one frame
                rej2 = not compare_arrays(lt, lr, lt, lr2)["ok"] and compare_arrays(lt, lr, lt, lr)["ok"]
                st = "selftest: translation mutation %s, rotation mutation %s" % ("rejected" if rej else "NOT CAUGHT", "rejected" if rej2 else "NOT CAUGHT")
                bad += (not rej) + (not rej2)
            report[name] = dict(ok=ok, clip_ok=ok_clip, roundtrip_equal=rt_equal, export_vs_vanilla_idle=cmp_, size=built.stat().st_size, sha256=sha)
            print("%-52s %s clip=%s roundtrip=%s export==vanilla idle: %s (dt %.2g m, dr %.2g deg) %s" % (
                name, "OK" if ok else "FAIL", ok_clip, rt_equal, cmp_.get("ok"), cmp_.get("max_dt_m", -1), cmp_.get("max_dr_deg", -1), st))
            bad += not ok
    (out / "report.json").write_text(json.dumps(report, indent=1) + "\n", encoding="utf-8")
    print("passenger idle sets: %d built, %d problem(s) -> %s" % (len(report), bad, depot))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
