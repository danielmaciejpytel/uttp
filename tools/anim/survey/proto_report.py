"""Stage 5 report block: GO / NO-GO of the guard prototype per clip and per state class (proto_<rig>.json -> report.txt).
  python -I proto_report.py --rig woman"""
import collections
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import survey_common as SC  # noqa: E402

RIG = sys.argv[sys.argv.index('--rig') + 1] if '--rig' in sys.argv else 'woman'
p = json.load(open(os.path.join(SC.WORK, 'proto_%s.json' % RIG), encoding='utf-8'))
L = ['criteria: GO = hand residual after IK <= 2 cm on every frame AND skinned strain of the worst frames (+-1) inside skin_ref + margin',
     '(non-hand regions) AND no guard pop (extra one-frame rotation of any body bone vs vanilla <= 10 deg). Scored AFTER clamp + translation',
     'reset + hinge IK + wrist clamp. fail% = frames failing the survey deformation gate (vanilla -> guard).',
     '%-62s %s %-5s %13s %15s %9s %7s %7s %7s %6s %s' % ('clip', 'c', 'shown', 'fail% v->g', 'skin v->g', 'hand cm', 'noIK', 'elbow', 'handrot', 'pop', 'verdict')]
cls_v = collections.defaultdict(list)
for k, v in p.items():
    L.append('%-62s %s %-5s %5.1f->%5.1f %7.3f->%6.3f %9.2f %7.1f %7.1f %7.1f %6.1f %s  (guard worst: %s %.1f)' % (
        k[:62], v['cls'], v['mode'][:5], v['fail_pct_vanilla'], v['fail_pct_guard'], v['skin_excess_vanilla'], v['skin_excess_guard'],
        v['hand_residual_cm_max'], v['hand_disp_noik_cm_max'], v['elbow_disp_cm_max'], v.get('hand_rot_err_deg_max', -1), v['pop_deg'],
        v['verdict'], v['worst_guard'][0], v['worst_guard'][1]))
    st = v['state']
    if 'exploration' in st:
        c = k.split('/')[1]
        st = 'ladder' if c.startswith('ladder') else 'knockdown' if 'knockdown' in c else 'death' if c.startswith('death') else 'swim' if 'swim' in c else 'vault/climb'
    cls_v[(v['cls'], st)].append(v['verdict'])
L.append('per state class (female, one representative = the most severe clip without glitch frames):')
for (c, st), vs in sorted(cls_v.items()):
    L.append('  %s %-40s %s' % (c, st, ' '.join(vs)))
g = sum(1 for v in p.values() if v['verdict'] == 'GO')
L.append('GO %d / %d clips. NO-GO causes: hand target out of rigid reach after the clavicle translation reset (FPP clavicles shifted 20-40 cm:'
         ' reload, finishers, car body_out / switch_seat, sniper nest, vault, doors, hammer, blueprint) and IK pops (hinge plane / wrist branch'
         ' cut) where the clamped elbow plane differs strongly from the vanilla one.' % (g, len(p)))
txt = '\n'.join(L)
print(txt)
SC.report('STAGE 5 GO / NO-GO PROTOTYPE %s (guard_proto.py, proto_%s.json; limits guard_limits_%s.json)' % (RIG.upper(), RIG, RIG), txt)
