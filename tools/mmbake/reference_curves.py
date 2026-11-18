"""Curves, contacts and loop flags of added clips read from a reference export (optional slide spec section).

Clips added through the slide spec come from clip files only: without this section they get contacts from the clip
file (often absent), footstep events derived from those contacts and modifier curves only through the family
majority rule. The section names a reference export (anims.json + anim_curves.json, same layout as the source pack
extraction) and, for each listed clip, takes the authored curves from it. A listed clip whose record or required
curve is missing stops the bake.

Slide spec section (all keys required):
  "reference_export": {
    "dir": "<export dir, relative to the project root>",
    "clips": ["<clip name>", ...],
    "contacts": {"contact_l": 0, "contact_r": 1},            # reference curve -> contact column (CONT)
    "curves": {"MoveData_Speed": "movedata_speed", ...},       # bundle curve -> reference curve
    "recompute_modifiers": ["AM_FootSpeed_L_C", ...]          # modifiers recomputed on the retargeted motion
  }
A curve the asset does not author (absent from its record's curve list) is left unset for that clip, as in the
asset (absent contacts: zero contact, no derived footsteps).
Units: curves whose bundle unit is m/s or m are converted from UE cm with the clip's hip ratio (as source pack
clips); "phase" curves keep the step key evaluation of the source pack curves.
"""
from __future__ import annotations

import json
import os

import numpy as np

REQUIRED_KEYS = ("dir", "clips", "contacts", "curves", "recompute_modifiers")


def _load_json(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


class ReferenceExport:
    def __init__(self, section, root):
        miss = [k for k in REQUIRED_KEYS if k not in section]
        if miss:
            raise SystemExit(f"slide spec reference_export: missing keys {miss}")
        d = section["dir"] if os.path.isabs(section["dir"]) else os.path.join(root, section["dir"])
        self.dir = d
        self.contacts = {str(k): int(v) for k, v in section["contacts"].items()}
        self.curves = {str(k): str(v) for k, v in section["curves"].items()}
        self.recompute = set(section["recompute_modifiers"])
        names = list(section["clips"])
        if len(set(names)) != len(names):
            raise SystemExit("slide spec reference_export: duplicate clip names")
        anims = {r["name"]: r for r in _load_json(os.path.join(d, "anims.json"))["anims"]}
        acurves = _load_json(os.path.join(d, "anim_curves.json"))["curves"]
        self.recs, self.keys, self.absent, errors = {}, {}, {}, []
        for n in names:
            rec = anims.get(n)
            if rec is None:
                errors.append(f"{n}: no record")
                continue
            cv = acurves.get(rec["path"])
            if cv is None:
                errors.append(f"{n}: no curves for {rec['path']}")
                continue
            # a curve may be absent only when the asset itself does not author it (its record does not list it): the
            # bundle then leaves that curve unset (contacts: zero, no derived footsteps), as for the asset
            listed = rec.get("curves") or []
            if isinstance(listed, str):
                raise SystemExit(f"{n}: record curve list is not a list")
            need = sorted(c for c in set(self.contacts) | set(self.curves.values()) if c in listed)
            lack = [c for c in need if c not in cv or not cv[c].get("t")]
            if lack:
                errors.append(f"{n}: missing curves {lack}")
                continue
            self.absent[n] = sorted(c for c in set(self.contacts) | set(self.curves.values()) if c not in listed)
            self.recs[n] = rec
            self.keys[n] = {c: (np.asarray(cv[c]["t"], float), np.asarray(cv[c]["v"], float)) for c in need}
        if errors:
            raise SystemExit("slide spec reference_export: " + "; ".join(errors[:20])
                             + (f" (+{len(errors) - 20} more)" if len(errors) > 20 else ""))

    def __contains__(self, name):
        return name in self.recs

    def num_frames(self, name):
        return int(float(self.recs[name]["num_frames"]))

    def loop(self, name):
        v = self.recs[name]["loop"]
        return v if isinstance(v, bool) else str(v).strip().lower() == "true"

    def length(self, name):
        return float(self.recs[name]["length_s"])

    def evaluate(self, name, curve, t, phase=False):
        kt, kv = self.keys[name][curve]
        out = np.interp(t, kt, kv)
        if phase:     # step keys (-1 -> 1 within one 1/120 s) stay a step, as the source pack phase evaluation
            for i in range(len(kt) - 1):
                if kv[i] == -1.0 and kv[i + 1] == 1.0 and kt[i + 1] - kt[i] < 0.0115:
                    out[(t >= kt[i]) & (t < kt[i + 1])] = -1.0
        return out

    def contact_matrix(self, name, n, fps):
        t = np.arange(n) / fps
        m = np.zeros((n, 2))
        for cn, k in self.contacts.items():
            if cn in self.absent[name]:
                continue
            m[:, k] = np.clip(self.evaluate(name, cn, t), 0.0, 1.0)
        return m


def load(slide, root):
    """ReferenceExport of the slide spec, or None when the spec has no reference_export section."""
    if not slide or "reference_export" not in slide:
        return None
    return ReferenceExport(slide["reference_export"], root)
