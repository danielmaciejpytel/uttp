r"""getup_mesh.py - linear-blend-skin the player body meshes with get-up poses and measure floor clearance of the MESH.

Offline measurement helper for getupbake.py / test_getupbake.py (the joint-radius clamp set alone does not prove that the
skin stays above the floor: buttocks, breasts, shoulder blades and the head sit well below their joints).

Mesh source: WolvenKit.CLI exports of the player base body meshes, read from the game archives (the game folder is never written):
  Tools\WolvenKit.CLI\WolvenKit.CLI.exe unbundle <game>\archive\pc\content\basegame_4_appearance.archive -o <mesh root>\raw
      -w "*t0_000_p??_base__full.mesh"      (and -w "*a0_00?_p??_base_hq__*.mesh" for the arms)
  Tools\WolvenKit.CLI\WolvenKit.CLI.exe unbundle ... -o <mesh root>\rawh -w "*h0_000_p??_c__basehead.mesh"
  set MeshExportArgs__ExportMaterials=false and MeshExportArgs__MaterialRepo=<mesh root>\matrepo, then
  WolvenKit.CLI.exe export <mesh root>\raw -o <mesh root>\glb -gp <game>     (and rawh -> glbh)
Resulting layout (what mesh_files() reads):
  <mesh root>/glb/base/characters/common/player_base_bodies/player_{female,man}_average/t0_000_p{w,m}a_base__full.glb
  <mesh root>/glb/.../arms_hq/a0_000_p{w,m}a_base_hq__{l,r}.glb
  <mesh root>/glbb/base/characters/common/player_base_bodies/player_female_average/t0_000_pwa_base__full_breast_big.glb  (woman only,
      unbundled with -w "*pwa_base__full_breast_*.mesh" into <mesh root>awb and exported to glbb; morph targets are not evaluated)
  <mesh root>/glbh/base/characters/head/p{wa,ma}/h0_000_p{wa,ma}_c__basehead/h0_000_p{wa,ma}_c__basehead.glb
(mesh root: UTPP_GETUP_MESH_ROOT, else <project root>/Build/test-data/getup-mesh.) Clothing is NOT part of the measurement: only the nude base body, arms and head.

Skinning (game frame, Z up): v' = sum_i w_i * Wpose(j_i) * Tc * IBM_j_i * v_glb. The glb joint frames are the game bind joint
frames post-multiplied by the constant rotation Tc = Rx(+90 deg) (verified against the rig: spine / legs differ by <= 7 deg
= A-pose vs the rig T-pose), the glb vertices are Y up, A = glb<-game axis map; the result is already in the game frame.
Mesh-only helper joints (deltoids, muscle joints) follow the nearest animated joint rigidly (bind-position distance).
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "anim", "vehicle-entry"))
import glbanim as G  # noqa: E402

PB = "base/characters/common/player_base_bodies/%s/"
TC = np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1.0]])
ANIM_SKIP = ("Root", "Trajectory", "reference_joint")


def default_mesh_root(project_root):
    """UTPP_GETUP_MESH_ROOT, else <project>/Build/test-data/getup-mesh (or <project>/test-data/getup-mesh when the project root is the Build folder)."""
    env = os.environ.get("UTPP_GETUP_MESH_ROOT")
    if env:
        return env
    cands = [os.path.join(project_root, "Build", "test-data", "getup-mesh"), os.path.join(project_root, "test-data", "getup-mesh")]
    for c in cands:
        if os.path.isdir(c):
            return c
    return cands[0]


def mesh_files(mesh_root, body):
    pfx, rig = ("pwa", "player_female_average") if body == "woman" else ("pma", "player_man_average")
    d = os.path.join(mesh_root, "glb", PB % rig)
    head_dir = os.path.join(mesh_root, "glbh", "base", "characters", "head", pfx, "h0_000_%s_c__basehead" % pfx)
    files = [os.path.join(d, "t0_000_%s_base__full.glb" % pfx),
             os.path.join(d, "arms_hq", "a0_000_%s_base_hq__l.glb" % pfx),
             os.path.join(d, "arms_hq", "a0_000_%s_base_hq__r.glb" % pfx),
             os.path.join(head_dir, "h0_000_%s_c__basehead.glb" % pfx)]
    if body == "woman":
        # the player-selectable big-breast body variant is measured as an extra part (union): its chest is the first to reach the floor
        files.append(os.path.join(mesh_root, "glbb", PB % rig, "t0_000_pwa_base__full_breast_big.glb"))
    return files


def available(mesh_root, body):
    return all(os.path.isfile(p) and os.path.getsize(p) > 100000 for p in mesh_files(mesh_root, body))


def _local(n):
    m = np.eye(4)
    if "matrix" in n:
        return np.array(n["matrix"]).reshape(4, 4).T
    x, y, z, w = n.get("rotation", [0, 0, 0, 1])
    r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    m[:3, :3] = r * np.array(n.get("scale", [1, 1, 1]))[None, :]
    m[:3, 3] = n.get("translation", [0, 0, 0])
    return m


def _node_world(gj):
    par = G.parents(gj)
    cache = {}

    def get(i):
        if i not in cache:
            cache[i] = (get(par[i]) if i in par else np.eye(4)) @ _local(gj["nodes"][i])
        return cache[i]
    return [get(i) for i in range(len(gj["nodes"]))]


def load_parts(paths):
    """Skinned primitives of the glb files: list of dicts (pos (V,3) glb frame, js/ws (V,8), joint names, ibm, node world)."""
    parts = []
    for p in paths:
        gj, binb = G.read_glb(p)
        names = G.node_names(gj)
        nw = _node_world(gj)
        node_of = {n: i for i, n in enumerate(names)}
        for node in gj["nodes"]:
            if "mesh" not in node or "skin" not in node:
                continue
            sk = gj["skins"][node["skin"]]
            jn = [names[j] for j in sk["joints"]]
            ibm = G.accessor(gj, binb, sk["inverseBindMatrices"]).reshape(-1, 4, 4).transpose(0, 2, 1)
            for prim in gj["meshes"][node["mesh"]]["primitives"]:
                at = prim["attributes"]
                if "JOINTS_0" not in at:
                    continue
                pos = G.accessor(gj, binb, at["POSITION"]).astype(np.float64)
                js = G.accessor(gj, binb, at["JOINTS_0"]).astype(np.int64)
                ws = G.accessor(gj, binb, at["WEIGHTS_0"]).astype(np.float64)
                if "JOINTS_1" in at:
                    js = np.concatenate([js, G.accessor(gj, binb, at["JOINTS_1"]).astype(np.int64)], 1)
                    ws = np.concatenate([ws, G.accessor(gj, binb, at["WEIGHTS_1"]).astype(np.float64)], 1)
                parts.append({"file": os.path.basename(p), "pos": pos, "js": js, "ws": ws, "jn": jn, "ibm": ibm,
                              "nw": nw, "node_of": node_of})
    return parts


class MeshSet:
    def __init__(self, mesh_root, body, anatomy=True):
        self.body = body
        self.anatomy = anatomy      # False = nearest-joint binding of all helper joints (comparison only)
        self.parts = load_parts(mesh_files(mesh_root, body))
        self.nv = sum(len(p["pos"]) for p in self.parts)
        # dominant joint (name) and source file per vertex
        self.dom = []
        self.src = []
        for p in self.parts:
            a = p["js"][np.arange(len(p["js"])), np.argmax(p["ws"], axis=1)]
            self.dom += [p["jn"][j] for j in a]
            self.src += [p["file"]] * len(a)

    @staticmethod
    def anatomical_parent(jn):
        """Animated bone that a mesh-only helper joint (the glb hierarchy is flat) follows, from its anatomical name; None =
        no rule (nearest animated joint by bind position)."""
        low = jn.lower()
        side = "Left" if low.startswith("l_") else ("Right" if low.startswith("r_") else None)
        if side is None:
            return None
        if any(k in low for k in ("chest", "scapula", "latissimus", "trapezius", "sterno", "butterfly")):
            return "Spine3"
        if any(k in low for k in ("shl_", "deltoid", "biceps", "triceps")):
            return side + "Arm"
        if "elbow" in low or "wrist_0" in low:
            return side + "ForeArm"
        if any(k in low for k in ("lowleg", "calf", "knee")):
            return side + "Leg"
        if any(k in low for k in ("thigh", "leg_back", "groin", "buttock")):
            return side + "UpLeg"
        return None

    def bind_matrices(self, p, anim_idx):
        """Per mesh joint: (animated clip bone index, constant 4x4 Tc * IBM_of_that_bone) so that skin = Wpose[anim] @ pre."""
        animated = [n for n in p["node_of"] if n in anim_idx and n not in ANIM_SKIP]
        apos = np.array([p["nw"][p["node_of"][n]][:3, 3] for n in animated])
        out = []
        for j, jn in enumerate(p["jn"]):
            if jn in anim_idx and jn not in ANIM_SKIP:
                out.append((anim_idx[jn], TC @ p["ibm"][j]))
            else:
                a = None
                if self.anatomy:
                    a = self.anatomical_parent(jn)
                    if a is not None and (a not in anim_idx or a not in p["node_of"]):
                        a = None
                if a is None:
                    ni = p["node_of"][jn]
                    a = animated[int(np.argmin(np.linalg.norm(apos - p["nw"][ni][:3, 3], axis=1)))]
                out.append((anim_idx[a], TC @ np.linalg.inv(p["nw"][p["node_of"][a]])))
        return out

    def skin(self, anim_names, wp, wq):
        """wp (F,B,3), wq (F,B,4 xyzw) world pose of the bones `anim_names` -> vertices (F, V, 3)."""
        idx = {n: i for i, n in enumerate(anim_names)}
        nf = wp.shape[0]
        x, y, z, w = wq[..., 0], wq[..., 1], wq[..., 2], wq[..., 3]
        R = np.stack([np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
                      np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
                      np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1)], -2)  # (F,B,3,3)
        Wm = np.zeros((nf, len(anim_names), 4, 4))
        Wm[..., :3, :3] = R
        Wm[..., :3, 3] = wp
        Wm[..., 3, 3] = 1.0
        outs = []
        for p in self.parts:
            bm = self.bind_matrices(p, idx)
            ai = np.array([a for a, _ in bm])
            pre = np.stack([m for _, m in bm])
            skin = Wm[:, ai] @ pre[None]                       # (F,J,4,4)
            ph = np.c_[p["pos"], np.ones(len(p["pos"]))]
            v = np.zeros((nf, len(ph), 3))
            for c in range(p["js"].shape[1]):
                m = skin[:, p["js"][:, c]]                     # (F,V,4,4)
                v += p["ws"][None, :, c:c + 1] * np.einsum("fvij,vj->fvi", m, ph)[..., :3]
            outs.append(v)
        return np.concatenate(outs, 1)

    def region(self, vi):
        """Coarse body region of vertex vi from its dominant joint / source file."""
        j = self.dom[vi].lower()
        s = self.src[vi]
        if s.startswith("h0_"):
            return "head"
        for key, name in (("chest", "breast/chest"), ("scapula", "scapula"), ("buttock", "glute"), ("groin", "groin/pelvis"),
                          ("thigh", "thigh"), ("calf", "calf"), ("knee", "knee"), ("lowleg", "calf"), ("leg_back", "thigh"),
                          ("deltoid", "shoulder"), ("bicep", "upper arm"), ("tricep", "upper arm"), ("latissimus", "back"),
                          ("trapezius", "back/neck"), ("stern", "neck"), ("neck", "neck"), ("wrist", "forearm"),
                          ("forearm", "forearm"), ("hand", "hand"), ("finger", "hand"), ("toe", "foot"), ("foot", "foot"),
                          ("upleg", "thigh"), ("leg", "calf"), ("arm", "upper arm"), ("shoulder", "shoulder"),
                          ("spine", "trunk"), ("hips", "pelvis")):
            if key in j:
                return name
        return j


def solver_bone(name):
    """Solver bone (a clamp-set / trunk bone of getup_fixup) that receives the floor violation of vertices whose dominant
    animated owner is bone `name`: fingers -> hand (wrist target lift), toes -> foot (ankle target lift), clavicle -> arm
    (shrug), upper arm / forearm / knee keep their own bone (swivel), the rest lifts the pelvis (Hips / Spine3 / Head / UpLeg)."""
    for side in ("Left", "Right"):
        if name.startswith(side):
            r = name[len(side):]
            if r.startswith("Hand") or r.startswith("InHand"):
                return side + "Hand"
            if r in ("Foot", "ToeBase", "Heel"):
                return side + "Foot"
            if r == "Shoulder":
                return side + "Arm"
            if r in ("Arm", "ForeArm", "Leg", "UpLeg"):
                return name
            return "Hips"
    if name in ("Spine2", "Spine3"):
        return "Spine3"
    if name.startswith("Neck") or name in ("Head",) or name.endswith("Eye"):
        return "Head"
    return "Hips"


class ZSkin:
    """Vertex heights only (third row of the skinning matrices): about 16x cheaper than MeshSet.skin; used inside the solver."""

    def __init__(self, ms, anim_names):
        self.ms = ms
        self.anim_names = list(anim_names)
        idx = {n: i for i, n in enumerate(self.anim_names)}
        self.parts = []
        sb = []
        self.owner = []
        for p in ms.parts:
            bm = ms.bind_matrices(p, idx)
            ai = np.array([a for a, _ in bm])
            pre = np.stack([m for _, m in bm])
            ph = np.c_[p["pos"], np.ones(len(p["pos"]))]
            self.parts.append((p, ai, pre, ph))
            dom = p["js"][np.arange(len(p["js"])), np.argmax(p["ws"], axis=1)]
            for j in dom:
                own = self.anim_names[ai[j]]
                self.owner.append(own)
                sb.append(solver_bone(own))
        self.sbone = np.array(sb)
        self.masks = {g: self.sbone == g for g in sorted(set(sb))}

    def z(self, wp, wq):
        """(F,B,3) / (F,B,4 xyzw) -> vertex heights (F, V)."""
        x, y, z, w = wq[..., 0], wq[..., 1], wq[..., 2], wq[..., 3]
        Wm3 = np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), wp[..., 2]], -1)  # (F,B,4)
        outs = []
        for p, ai, pre, ph in self.parts:
            skz = np.einsum("fjk,jkl->fjl", Wm3[:, ai], pre)   # (F,J,4)
            v = np.zeros((wp.shape[0], len(ph)))
            for c in range(p["js"].shape[1]):
                v += p["ws"][None, :, c] * np.einsum("fvl,vl->fv", skz[:, p["js"][:, c]], ph)
            outs.append(v)
        return np.concatenate(outs, 1)

    def group_min(self, zv):
        """(F,V) -> {solver bone name: (F,) lowest height of the vertices it owns}."""
        return {g: zv[:, m].min(1) for g, m in self.masks.items()}
