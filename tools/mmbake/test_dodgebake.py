"""Tests of dodgebake.py: helpers always; the full bake (both bodies, temporary folder) when the player animation set is unbundled.

usage: python -I test_dodgebake.py
"""
import json
import os
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dodgebake as D  # noqa: E402
import getupbake as G  # noqa: E402

FAILS = []


def expect(ok, what):
    print(("ok   " if ok else "FAIL ") + what)
    if not ok:
        FAILS.append(what)


def test_helpers():
    path = np.zeros((10, 3))
    path[:, 1] = [0, 0.01, 0.03, 0.10, 0.20, 0.27, 0.305, 0.31, 0.31, 0.31]
    t_push, t_rec, peak, _ = D.push_off(path, 30.0)
    expect(abs(t_push - 4 / 30.0) < 1e-9 and abs(peak - 3.0) < 1e-6, "push-off = frame of the peak hips speed")
    expect(abs(t_rec - 7 / 30.0) < 1e-9, "recover = first frame after the push-off below 30 % of the peak")
    expect(abs(D.mat_yaw(np.eye(3))) < 1e-12, "yaw of the identity = 0")


def test_bake():
    try:
        root = D.project_root(None)
    except SystemExit:
        print("skip bake: project root not found")
        return
    src = os.path.join(root, "Extracted", "game", "player-anims", "glb", "base", "animations", "player", "female", "body",
                       "player_exploration.anims.glb")
    if not os.path.isfile(src):
        print("skip bake: player animation set not unbundled")
        return
    with tempfile.TemporaryDirectory() as tmp:
        texts = []
        for body in ("woman", "man"):
            rep = D.bake_body(root, body, tmp)
            p = os.path.join(tmp, "utpp_dodge_%s.utpptrav" % body)
            data = open(p, "rb").read()
            texts.append(data)
            parsed = G.parse_trav(data.decode("ascii"))
            names = [m["name"] for m in parsed["montages"]]
            expect(names == ["Dodge_F", "Dodge_B", "Dodge_L", "Dodge_R"], "%s: four montages in order" % body)
            expect(len(parsed["pose_bones"]) == 73 and parsed["pose_bones"][:3] == ["Root", "Trajectory", "Hips"],
                   "%s: 73 pose bones, database order" % body)
            hips = parsed["pose_bones"].index("Hips")
            ok_off = True
            for m in parsed["montages"]:
                P = np.array(m["pose"], dtype=float).reshape(m["frames"], 73, 7)
                ok_off &= bool(abs(P[:, hips, 1].mean()) < 1e-5)
                ok_off &= bool(np.allclose(P[:, 0, :3], 0.0) and np.allclose(P[:, 0, 6], 1.0))
            expect(ok_off, "%s: hips mean Y offset removed, Root identity (in place)" % body)
            expect(rep["left_source"] == "mirrored fpp_dodge_right", "%s: left = mirrored right by default" % body)
            r = rep["clips"]["R"]
            expect(abs(r["hips_xy_first_last"][0][0]) < 0.02 and abs(r["hips_xy_first_last"][1][0]) < 0.03,
                   "%s: Dodge_R hips X at the first / last frame unchanged (lateral push-off kept, no sideways start)" % body)
            expect(all(not c["errors"] and c["anatomy_gate"]["passed"] for c in rep["clips"].values()), "%s: every gate passed" % body)
            expect(json.loads(open(os.path.join(tmp, "dodge_report_%s.json" % body), "rb").read())["file"]["bytes"] == len(data),
                   "%s: report lists the file size" % body)
        rep2 = D.bake_body(root, "woman", tmp)
        expect(open(os.path.join(tmp, "utpp_dodge_woman.utpptrav"), "rb").read() == texts[0] and rep2["file"]["sha256"],
               "deterministic: the same bytes twice")


def test_bake_npc():
    try:
        root = D.project_root(None)
    except SystemExit:
        print("skip npc bake: project root not found")
        return
    if not all(os.path.isfile(os.path.join(root, *D.NPC_DIR.split("/"), D.NPC_PREFIX[b] + s + ".anims.glb"))
               for b in ("woman", "man") for _, s, _, _, _, _ in D.NPC_CLIPS):
        print("skip npc bake: NPC combat sets not extracted")
        return
    with tempfile.TemporaryDirectory() as tmp:
        texts = []
        for body in ("woman", "man"):
            rep = D.bake_body(root, body, tmp, source="npc")
            data = open(os.path.join(tmp, "utpp_dodge_%s.utpptrav" % body), "rb").read()
            texts.append(data)
            parsed = G.parse_trav(data.decode("ascii"))
            expect([m["name"] for m in parsed["montages"]] == ["Dodge_F", "Dodge_B", "Dodge_L", "Dodge_R"], "%s npc: four montages in order" % body)
            ok_root = True
            for m in parsed["montages"]:
                P = np.array(m["pose"], dtype=float).reshape(m["frames"], 73, 7)
                ok_root &= bool(np.allclose(P[:, 0, :3], 0.0) and np.allclose(P[:, 0, 6], 1.0))
            expect(ok_root, "%s npc: Root identity on every frame (in place)" % body)
            c = rep["clips"]
            expect(rep["source"] == "npc" and all(not x["errors"] and x["anatomy_gate"]["passed"] for x in c.values()),
                   "%s npc: every gate passed (anatomy, bone lengths, joint steps, soles)" % body)
            expect(all(x["frames"] == x["window"][1] - x["window"][0] + 1 for x in c.values()), "%s npc: each clip is its source window" % body)
            # Dodge_L is the mirrored Dodge_R: lateral hips and root travel opposite, the rest equal
            l, r = c["L"], c["R"]
            expect(l["mirror"] and not r["mirror"] and abs(l["root_disp"][0] + r["root_disp"][0]) < 1e-3 and r["root_disp"][0] > 0.5
                   and abs(l["hips_xy_first_last"][0][0] + r["hips_xy_first_last"][0][0]) < 1e-3, "%s npc: Dodge_L = mirrored Dodge_R" % body)
            expect(c["B"]["root_disp"][1] < -1.0 and c["F"]["root_disp"][1] > 1.0, "%s npc: back / forward clips travel back / forward" % body)
        rep2 = D.bake_body(root, "woman", tmp, source="npc")
        expect(open(os.path.join(tmp, "utpp_dodge_woman.utpptrav"), "rb").read() == texts[0] and rep2["file"]["sha256"],
               "npc deterministic: the same bytes twice")
        # optional fifth montage Dodge_F_Run: written after the four, same gates; the four montages are byte-identical to the file without it
        fr = ("gang_unarmed_action_combat", "melee_combo_01_p1_dash", False, 0, 15)
        if os.path.isfile(os.path.join(root, *D.NPC_DIR.split("/"), D.NPC_PREFIX["woman"] + fr[0] + ".anims.glb")):
            for body, four in (("woman", texts[0]), ("man", texts[1])):
                rep5 = D.bake_body(root, body, tmp, source="npc", forward_run=fr)
                data5 = open(os.path.join(tmp, "utpp_dodge_%s.utpptrav" % body), "rb").read().decode("ascii")
                parsed5 = G.parse_trav(data5)
                expect([m["name"] for m in parsed5["montages"]] == ["Dodge_F", "Dodge_B", "Dodge_L", "Dodge_R", "Dodge_F_Run"] and "\nCOUNT 5\n" in data5,
                       "%s npc + forward run: five montages, Dodge_F_Run last" % body)
                c5 = rep5["clips"][D.FORWARD_RUN]
                expect(not c5["errors"] and c5["anatomy_gate"]["passed"] and c5["frames"] == 16 and c5["window"] == [0, 15],
                       "%s npc + forward run: Dodge_F_Run passes the gates, frames = its window" % body)
                body4 = four.decode("ascii").split("\nCOUNT 4\n", 1)[1].rsplit("END\n", 1)[0]
                body5 = data5.split("\nCOUNT 5\n", 1)[1]
                expect(body5.startswith(body4) and body5[len(body4):].startswith("M Dodge_F_Run "),
                       "%s npc + forward run: the four montages unchanged, Dodge_F_Run appended" % body)
            bad = None
            try:
                D.bake_body(root, "woman", tmp, source="npc", forward_run=(fr[0], fr[1], False, 0, 999))
            except SystemExit as e:
                bad = str(e)
            expect(bad is not None and "outside the clip" in bad, "forward run window outside the clip: no file (SystemExit)")
        else:
            print("skip forward run bake: NPC set not extracted")


def test_forward_run_option():
    expect(D.parse_forward_run("gang_unarmed_action_combat:dash_foward_long:3:16") == ("gang_unarmed_action_combat", "dash_foward_long", False, 3, 16),
           "--forward-run SET:CLIP:FIRST:LAST")
    expect(D.parse_forward_run("s:c:0:5:mirror")[2] is True, "--forward-run ...:mirror")
    for bad in ("s:c:3", "s:c:5:3", "s:c:a:5", "s:c:0:5:flip", ":c:0:5"):
        try:
            D.parse_forward_run(bad)
            expect(False, "--forward-run rejects %r" % bad)
        except SystemExit:
            expect(True, "--forward-run rejects %r" % bad)
    try:
        D.bake_body(".", "woman", ".", source="player", forward_run=("s", "c", False, 0, 5))
        expect(False, "--forward-run with the player source is refused")
    except SystemExit as e:
        expect("needs --source npc" in str(e), "--forward-run with the player source is refused")


def main():
    test_helpers()
    test_forward_run_option()
    test_bake()
    test_bake_npc()
    print("FAILED" if FAILS else "PASSED", len(FAILS))
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
