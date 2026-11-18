#!/usr/bin/env python3
"""Bake the shipped motion databases (utpp_motion_<body>.utppmm) from their recorded recipe in one command.

  bake_motion_db.py --inputs INPUTS.json --out-dir DIR [--body woman|man|both] [--db-overrides SPEC] [--keep-staging]

Steps per body:
  1. retarget/launch_posture.py apply: base_clips -> DIR/_staging/clips-<body> (run / sprint start launch-lean cap);
  2. mmbake_base.py --format v2 with config/motion_db_recipe.json (common + body) and the body's local inputs;
  3. DIR/utpp_motion_<body>.utppmm (+ .chooser.json, .summary.json) and DIR/<body>.sha256 with its SHA-256.
Without --db-overrides the result is byte-identical to the shipped databases when the inputs are the recorded ones.

INPUTS.json (local bake inputs, paths relative to the project root or absolute):
  {"woman": {"base_clips": ..., "slide_clips": ..., "rtz_clips": ..., "v2_config": ..., "slide_spec": ..., "extra_assets": ...},
   "man": {...}}
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import mmbake_base  # noqa: E402

RECIPE = os.path.join(HERE, "config", "motion_db_recipe.json")
LAUNCH = os.path.join(HERE, "..", "retarget", "launch_posture.py")


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def run(cmd, log_path):
    with open(log_path, "w", encoding="utf-8") as lf:
        p = subprocess.run(cmd, stdout=lf, stderr=subprocess.STDOUT, env=dict(os.environ, PYTHONDONTWRITEBYTECODE="1"))
    if p.returncode != 0:
        raise SystemExit(f"step failed ({p.returncode}): see {log_path}")


def tool_cmd(script):
    # run a sibling tool with its own folder importable, also under an isolated interpreter
    boot = ("import runpy, sys, os; s = sys.argv[1]; sys.path.insert(0, os.path.dirname(os.path.abspath(s))); "
            "sys.argv = sys.argv[1:]; runpy.run_path(s, run_name='__main__')")
    return [sys.executable, "-I", "-B", "-c", boot, os.path.abspath(script)]


def bake_body(body, recipe, inputs, root, out_dir, overrides, keep):
    rb = recipe["bodies"][body]
    c = recipe["common"]
    loc = inputs[body]
    missing = [k for k in recipe["input_keys"] if k not in loc]
    if missing:
        raise SystemExit(f"--inputs: {body} lacks {missing}")
    p = lambda v: v if os.path.isabs(v) else os.path.join(root, v)  # noqa: E731
    for k in recipe["input_keys"]:
        if not os.path.exists(p(loc[k])):
            raise SystemExit(f"--inputs: {body}.{k} not found: {loc[k]}")
    stage = os.path.join(out_dir, "_staging", f"clips-{body}")
    if os.path.isdir(stage):
        shutil.rmtree(stage)
    run(tool_cmd(LAUNCH) + ["apply", "--src", p(loc["base_clips"]), "--dst", stage, "--rig", rb["launch_rig"]],
        os.path.join(out_dir, f"{body}.launch.log"))
    out = os.path.join(out_dir, f"utpp_motion_{body}.utppmm")
    cmd = tool_cmd(os.path.join(HERE, "mmbake_base.py")) + [
        "--format", "v2", "--clips", f"{stage};{p(loc['slide_clips'])}", "--rtz-clips", p(loc["rtz_clips"]),
        "--meta", p(c["meta"]), "--config", p(c["config"]), "--v2-config", p(loc["v2_config"]),
        "--sourcepack-dir", p(c["sourcepack_dir"]), "--clip-map", p(c["clip_map"]), "--ue-tracks", p(c["ue_tracks"]),
        "--rig", p(rb["rig"]), "--bonemap", p(rb["bonemap"]), "--motion-src", c["motion_src"],
        "--slide-spec", p(loc["slide_spec"]), "--extra-assets", p(loc["extra_assets"]),
        "--out", out, "--log", os.path.join(out_dir, f"{body}.log")]
    if c.get("sourcepack_only"):
        cmd.append("--sourcepack-only")
    if overrides:
        cmd += ["--db-overrides", os.path.abspath(overrides)]
    run(cmd, os.path.join(out_dir, f"{body}.stdout"))
    sha = sha256_file(out)
    with open(os.path.join(out_dir, f"{body}.sha256"), "w", encoding="utf-8", newline="\n") as f:
        f.write(f"{sha}  {os.path.basename(out)}\n")
    if not keep:
        shutil.rmtree(stage)
    return out, sha


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--inputs", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--body", choices=("woman", "man", "both"), default="both")
    ap.add_argument("--db-overrides", default=None)
    ap.add_argument("--keep-staging", action="store_true")
    a = ap.parse_args()
    root = mmbake_base.project_root()
    with open(RECIPE, encoding="utf-8") as f:
        recipe = json.load(f)
    with open(a.inputs, encoding="utf-8") as f:
        inputs = json.load(f)
    os.makedirs(a.out_dir, exist_ok=True)
    for body in (("woman", "man") if a.body == "both" else (a.body,)):
        out, sha = bake_body(body, recipe, inputs, root, os.path.abspath(a.out_dir), a.db_overrides, a.keep_staging)
        print(f"{body}: {out} sha256 {sha}")
    shutil.rmtree(os.path.join(a.out_dir, "_staging"), ignore_errors=True) if not a.keep_staging else None
    print("RECIPE_OK")


if __name__ == "__main__":
    main()
