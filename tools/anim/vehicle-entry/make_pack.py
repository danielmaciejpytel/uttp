"""Build sources-final-<TAG>.txt (a base set of clips with the accepted rows replaced), the staged anims dir for pack_car_workspot.ps1, the pack DECISION file and the pack MANIFEST.

  python -I make_pack.py [--base <sources.txt>] [--out-root <dir>] [--verdict-dir <dir> ...] [--conditional ROW:TYPE[,TYPE] --reason ROW=text] TAG [row=rundir ...]
  python -I make_pack.py --stamp-archive --archive <packed .archive> [--idle-dir <dir with the passenger idle .anims>] [--out-root <dir>] TAG
  python -I make_pack.py --selftest

--base      the starting set ('<clip>  <run dir>' lines; default: the round-2 CONDITIONAL set roof-pack2\\sources-final-C2.txt);
--out-root  where sources-final-<TAG>.txt, pack-decision-<TAG>.json, pack-manifest-<TAG>.json and stage-anims-<TAG>\\utpp_vehicle_car\\utpp\\vehicle are written;
row=rundir  a clip (utpp_car_ prefix optional) taken from another build_car.ps1 run (relative paths are relative to --out-root): a NEW row. Rows of the base set are INHERITED.
The passenger idle sets (utpp_car_p?a_<class>_passenger_r_idle.anims, make_passenger_idle.py run) are NOT in the sources: pass their run dir next to the staged dir
(pack_car_workspot.ps1 -AnimsRun <out-root>\\stage-anims-<TAG>,<idle run>).

PACKING CHECK (every row, never skippable). Problems are typed; a row with any problem is REFUSED unless --conditional ROW:TYPE names exactly that type (repeatable, per problem type):
  verdict   door-close-<clip>.json (--verdict-dir first, then the run dir) missing / unreadable / no body phase / body_tol_cm or roof_tol_cm key missing / no start_gate record / not feasible /
            written for another clip file (sha256 of roundtrip/<clip>.anims.glb must equal clip_sha256 - mandatory, a missing GLB or a missing clip_sha256 is a problem) /
            the packed .anims is OLDER than ... NEWER than the round-trip GLB (the GLB is exported from the .anims: a .anims replaced afterwards no longer matches the verdict)   [never conditional]
  door:<phase>   the verdict fails ONE door phase at the solved close; one type per phase of the verdict (r10): door:closed  door:opening  door:open  door:closing  door:closed_after  door:rear  door:body
            door:jamb  door:roof, plus door:not_feasible (no close solves) and door:other (door_clearance_ok false without a failing phase record). --conditional names exactly the failing phase(s)
            (a plain 'door' is refused: too coarse)
  start     start_gate: start position gate (a)          rear   start_gate: rearward motion gate (e)          joint   joint rule (b)          waiver   waiver audit (c)
  approach  the run's approach gate (approach-<clip>.txt 'RESULT approach': relative FAIL > 0); a NEW row without a record is a problem too (inherited rows: noted in the decision file)
  checks    check_car_entry (check-<clip>.json ok false)
Every --conditional row needs a --reason ROW=text (why the open problem is accepted); it is written to the decision file (conditional_reason).
The decision file lists, per row: new / inherited, run dir, the verdict file path it was judged by (verdict_path), sha256 of the packed .anims and of the round-trip GLB, problems found, the conditional
types accepted - nothing is only on stdout.
The manifest (pack-manifest-<TAG>.json) pairs the archive with the runtime tables: row -> clip sha256s, approach spot (entry_approach.json), seat variable, door close delta (door_timing.json) and the
sha256 of utpp_vehicle_door_timing.reds / utpp_vehicle_approach_table.reds; check_pack_manifest.py verifies the plugin scripts against it. r10: the manifest also holds, per row, the source-clip sha256 and the solved close of its door_timing.json entry and the verdict's close /
clip sha256 (check_pack_manifest.py ties them to the packed clip), and --stamp-archive adds the sha256 of the packed .archive and of the passenger idle .anims.
2026-11-04 r11: a rerun of the pack step keeps an existing archive / idle stamp only while the staged .anims are byte-identical to the ones the stamp was made for (same rows, same anims_sha256);
otherwise the stamp is dropped and a WARNING says so (repack, then run --stamp-archive again). The idle clips are not staged here: check_pack_manifest.py --idle-dir catches a changed idle set.
"""
import argparse
import hashlib
import json
import os
import re
import shutil
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from utpp_paths import project_root  # noqa: E402

ROOT = os.path.join(project_root(), r'Build\work\vehicle-entry-car')
HERE = os.path.dirname(os.path.abspath(__file__))
DOOR_PHASES = ('closed', 'opening', 'open', 'closing', 'closed_after', 'rear', 'body', 'jamb', 'roof')
DOOR_TYPES = tuple('door:' + p for p in DOOR_PHASES) + ('door:not_feasible', 'door:other')
TYPES = DOOR_TYPES + ('start', 'rear', 'joint', 'waiver', 'approach', 'checks', 'feel')


def sha256_of(path):
    return hashlib.sha256(open(path, 'rb').read()).hexdigest().upper()


def approach_relative_fails(run_dir, name):
    """Relative FAIL count of the run's approach gate (None = no record)."""
    p = os.path.join(run_dir, 'approach-%s.txt' % name)
    if not os.path.exists(p):
        return None
    b = open(p, 'rb').read()
    t = b.decode('utf-16') if b[:2] in (bytes([255, 254]), bytes([254, 255])) else b.decode('utf-8', 'replace')
    m = re.findall(r'RESULT approach[^\n]*?(\d+) starts, (\d+) FAIL \(hold / entity\) RELATIVE', t)
    return int(m[-1][1]) if m else None


def find_verdict(name, run_dir, verdict_dirs):
    for vd in list(verdict_dirs) + [run_dir]:
        p = os.path.join(vd, 'door-close-%s.json' % name)
        if os.path.exists(p):
            return p
    return None


def feel_problems(name, rt):
    """r13: entry feel gates of a NEW driver row (entry_feel.py E1..E5: start as far forward as the door swing allows, along-car travel, rearward slide of the entity root, hips / knees vs the
    aperture rear edge, head + hair clearance under the roof during the transition). E6 (mount jump) is a runtime property of the native arrival and is checked from game logs."""
    try:
        sys.path.insert(0, HERE)
        import check_door_clearance as _cd
        import entry_feel as _ef
        g, cls, seat = _cd.parse_name(rt)
        o = _ef.evaluate(rt, cls, seat)
    except Exception as e:  # noqa: BLE001
        return [('feel', 'entry_feel could not evaluate %s (%s)' % (name, e))]
    bad = [k for k in ('E1', 'E2', 'E3', 'E4', 'E5') if not o[k]]
    if not bad:
        return []
    return [('feel', 'entry_feel %s: %s' % (','.join(bad), '; '.join(
        {'E1': 'start %.2f m behind the most forward spot the door swing allows (limit %.2f)' % (o['start_gap_m'] or 0.0, _ef.START_FWD_TOL_M),
         'E2': 'along-car travel %.2f m (limit %.2f)' % (o['along_car_m'] or 0.0, _ef.ALONG_CAR_MAX_M),
         'E3': 'rearward root %.3f / sink %.3f m (limit %.2f)' % (o['rear_m']['root'], o['rear_m']['sink'], _ef.REAR_MAX_M),
         'E4': 'limb behind the aperture rear edge %s' % o['legs_margin_m'],
         'E5': 'roof transition max penetration %.1f cm (needs <= %.1f)' % (o['roof_transition_max_pen_cm'], -_ef.HAIR_MARGIN_CM)}[k] for k in bad)))]


def verdict_problems(name, run_dir, verdict_dirs=(), is_new=False):
    """[(type, text)] of one row. type 'verdict' is never conditional."""
    out = []
    frag = find_verdict(name, run_dir, verdict_dirs)
    if frag is None:
        return [('verdict', 'no door verdict door-close-%s.json in %s' % (name, ', '.join(list(verdict_dirs) + [run_dir])))]
    try:
        j = json.load(open(frag, encoding='utf-8'))
    except Exception as e:  # noqa: BLE001
        return [('verdict', 'unreadable door verdict %s (%s)' % (frag, e))]
    if j.get('body') is not True:
        out.append(('verdict', 'verdict has no body phase (body=%r)' % (j.get('body'),)))
    for k in ('body_tol_cm', 'roof_tol_cm'):
        if k not in j:
            out.append(('verdict', 'verdict lacks %s' % k))
    sg = j.get('start_gate')
    if not isinstance(sg, dict):
        out.append(('verdict', 'verdict lacks the start_gate record'))
    else:
        for typ, key in (('start', 'start'), ('rear', 'rearward'), ('joint', 'joint_rule'), ('waiver', 'waivers')):
            sub = sg.get(key)
            if not isinstance(sub, dict):
                out.append(('verdict', 'start_gate record lacks %s (verdict written by an older check_door_clearance.py)' % key))
            elif not sub.get('ok'):
                out.append((typ, 'start_gate: %s gate failed' % key))
    if not j.get('feasible'):
        out.append(('door:not_feasible', 'door verdict not feasible'))
    else:
        ac = j.get('at_close_after') or {}
        failing = [(p, v) for p, v in (ac.get('phases') or {}).items() if isinstance(v, dict) and v.get('ok') is False]
        for p, v in failing:
            out.append(('door:' + p if ('door:' + p) in DOOR_TYPES else 'door:other',
                        'door phase %s fails at the solved close (clearance %s cm vs gate %s cm, capsule %s)' % (p, v.get('clearance_cm'), v.get('gate_cm'), v.get('capsule'))))
        if not ac.get('door_clearance_ok') and not failing:
            out.append(('door:other', 'door verdict FAILS its gates at the solved close (door_clearance_ok false, no failing phase recorded)'))
    # the verdict must belong to the file that is packed: GLB sha mandatory, packed .anims no older than ... not newer than its GLB export
    rt = os.path.join(run_dir, 'roundtrip', name + '.anims.glb')
    packed = os.path.join(run_dir, 'utpp_vehicle_car', 'utpp', 'vehicle', name + '.anims')
    if not os.path.exists(rt):
        out.append(('verdict', 'round-trip GLB %s missing (the verdict cannot be tied to the clip)' % rt))
    elif not j.get('clip_sha256'):
        out.append(('verdict', 'verdict has no clip_sha256'))
    elif sha256_of(rt) != str(j['clip_sha256']).upper():
        out.append(('verdict', 'verdict was written for another clip file (sha256 differs from %s)' % rt))
    if not os.path.exists(packed):
        out.append(('verdict', 'packed clip %s missing' % packed))
    elif os.path.exists(rt) and os.path.getmtime(packed) > os.path.getmtime(rt) + 2.0:
        out.append(('verdict', 'packed %s is NEWER than its round-trip GLB export (replaced after the verdict)' % packed))
    af = approach_relative_fails(run_dir, name)
    if af is None:
        if is_new:
            out.append(('approach', 'new row without an approach gate record (approach-%s.txt)' % name))
    elif af > 0:
        out.append(('approach', 'approach gate: %d start(s) FAIL relative to the baseline' % af))
    if is_new and re.match(r'^utpp_car_p[wm]a_[a-z0-9]+_driver$', name) and os.path.exists(rt):
        out += feel_problems(name, rt)
    ck = os.path.join(run_dir, 'check-%s.json' % name)
    if os.path.exists(ck):
        try:
            if not json.load(open(ck, encoding='utf-8')).get('ok'):
                out.append(('checks', 'check_car_entry: failing checks (check-%s.json ok false)' % name))
        except Exception:  # noqa: BLE001
            out.append(('checks', 'check-%s.json unreadable' % name))
    elif is_new:
        out.append(('checks', 'new row without check_car_entry record (check-%s.json)' % name))
    return out


def selftest_verdict():
    """Mutations of the packing check: each broken run must be refused with the right problem type, the good one accepted."""
    import tempfile
    import time
    d = tempfile.mkdtemp(prefix='verdict_selftest_')
    os.makedirs(os.path.join(d, 'roundtrip'))
    os.makedirs(os.path.join(d, 'utpp_vehicle_car', 'utpp', 'vehicle'))
    glb = os.path.join(d, 'roundtrip', 'c.anims.glb')
    anims = os.path.join(d, 'utpp_vehicle_car', 'utpp', 'vehicle', 'c.anims')
    open(anims, 'wb').write(b'anims bytes')
    time.sleep(0.05)
    open(glb, 'wb').write(b'clip bytes')
    sha = hashlib.sha256(b'clip bytes').hexdigest().upper()
    sg_ok = dict(ok=True, start=dict(ok=True), rearward=dict(ok=True), joint_rule=dict(ok=True), waivers=dict(ok=True))
    ph_ok = {p: dict(ok=True, clearance_cm=3.0, gate_cm=1.0) for p in DOOR_PHASES}

    def ph_bad(p):
        x = json.loads(json.dumps(ph_ok))
        x[p] = dict(ok=False, clearance_cm=-2.0, gate_cm=1.0, capsule='chest')
        return dict(door_clearance_ok=False, phases=x)
    good = dict(feasible=True, at_close_after=dict(door_clearance_ok=True, phases=ph_ok), body=True, body_tol_cm=None, roof_tol_cm=None, start_gate=sg_ok, clip_sha256=sha, close_after_s=1.7)
    open(os.path.join(d, 'approach-c.txt'), 'w').write('RESULT approach cap on alignments T: 4 starts, 0 FAIL (hold / entity) RELATIVE to the baseline, 0 FAIL ABSOLUTE\n')
    json.dump(dict(ok=True), open(os.path.join(d, 'check-c.json'), 'w'))

    def sg_bad(key):
        x = json.loads(json.dumps(sg_ok))
        x[key] = dict(ok=False)
        return x
    cases = [('good verdict', good, None, True, ()),
             ('no body phase (body false)', dict(good, body=False), None, False, ('verdict',)),
             ('body_tol_cm key missing', {k: v for k, v in good.items() if k != 'body_tol_cm'}, None, False, ('verdict',)),
             ('roof_tol_cm key missing', {k: v for k, v in good.items() if k != 'roof_tol_cm'}, None, False, ('verdict',)),
             ('old verdict without body / waiver keys', dict(feasible=True, close_after_s=1.7, clip_sha256=sha), None, False, ('verdict',)),
             ('no start_gate record', {k: v for k, v in good.items() if k != 'start_gate'}, None, False, ('verdict',)),
             ('start_gate record without the rearward part', dict(good, start_gate={k: v for k, v in sg_ok.items() if k != 'rearward'}), None, False, ('verdict',)),
             ('start gate failed', dict(good, start_gate=sg_bad('start')), None, False, ('start',)),
             ('rearward gate failed', dict(good, start_gate=sg_bad('rearward')), None, False, ('rear',)),
             ('joint rule failed', dict(good, start_gate=sg_bad('joint_rule')), None, False, ('joint',)),
             ('waiver audit failed', dict(good, start_gate=sg_bad('waivers')), None, False, ('waiver',)),
             ('verdict of another clip file', dict(good, clip_sha256='0' * 64), None, False, ('verdict',)),
             ('verdict without clip_sha256', {k: v for k, v in good.items() if k != 'clip_sha256'}, None, False, ('verdict',)),
             ('infeasible door verdict', dict(good, feasible=False), None, False, ('door:not_feasible',)),
             ('door gates fail, no phase record', dict(good, at_close_after=dict(door_clearance_ok=False)), None, False, ('door:other',)),
             ('door phase closing fails', dict(good, at_close_after=ph_bad('closing')), None, False, ('door:closing',)),
             ('door phase opening fails', dict(good, at_close_after=ph_bad('opening')), None, False, ('door:opening',)),
             ('door phase open fails', dict(good, at_close_after=ph_bad('open')), None, False, ('door:open',)),
             ('door phase rear fails', dict(good, at_close_after=ph_bad('rear')), None, False, ('door:rear',)),
             ('door phase body fails', dict(good, at_close_after=ph_bad('body')), None, False, ('door:body',)),
             ('door phase jamb fails', dict(good, at_close_after=ph_bad('jamb')), None, False, ('door:jamb',)),
             ('door phase roof fails', dict(good, at_close_after=ph_bad('roof')), None, False, ('door:roof',)),
             ('approach gate fails (relative)', good, ('approach', 'RESULT approach cap on alignments T: 4 starts, 3 FAIL (hold / entity) RELATIVE to the baseline, 3 FAIL ABSOLUTE\n'), False, ('approach',)),
             ('check_car_entry fails', good, ('checks', None), False, ('checks',))]
    ok = True
    for name, frag, extra, want_ok, want_types in cases:
        json.dump(frag, open(os.path.join(d, 'door-close-c.json'), 'w'))
        open(os.path.join(d, 'approach-c.txt'), 'w').write('RESULT approach cap on alignments T: 4 starts, 0 FAIL (hold / entity) RELATIVE to the baseline, 0 FAIL ABSOLUTE\n')
        json.dump(dict(ok=True), open(os.path.join(d, 'check-c.json'), 'w'))
        if extra and extra[0] == 'approach':
            open(os.path.join(d, 'approach-c.txt'), 'w').write(extra[1])
        if extra and extra[0] == 'checks':
            json.dump(dict(ok=False), open(os.path.join(d, 'check-c.json'), 'w'))
        probs = verdict_problems('c', d, (), True)
        types = tuple(sorted(set(t for t, _ in probs)))
        res = (not probs) == want_ok and (want_ok or (set(want_types) <= set(types) and set(t for t in types if t.startswith('door:')) == set(w for w in want_types if w.startswith('door:')) or not any(w.startswith('door:') for w in want_types) and set(want_types) <= set(types)))  # a door phase yields exactly its own door:<phase> type
        ok &= res
        print('selftest packing %-46s -> %s %s (%s)' % (name, 'accepted' if not probs else 'REFUSED', ','.join(types), 'as expected' if res else 'WRONG'))
    # the packed .anims replaced after the GLB export
    json.dump(good, open(os.path.join(d, 'door-close-c.json'), 'w'))
    open(os.path.join(d, 'approach-c.txt'), 'w').write('RESULT approach cap on alignments T: 4 starts, 0 FAIL (hold / entity) RELATIVE to the baseline\n')
    json.dump(dict(ok=True), open(os.path.join(d, 'check-c.json'), 'w'))
    os.utime(anims, (time.time() + 60, time.time() + 60))
    probs = verdict_problems('c', d, (), True)
    res = any(t == 'verdict' and 'NEWER' in x for t, x in probs)
    ok &= res
    print('selftest packing %-46s -> %s (%s)' % ('.anims replaced after the GLB export', 'REFUSED' if probs else 'accepted', 'as expected' if res else 'WRONG'))
    os.utime(anims, (os.path.getmtime(glb) - 5, os.path.getmtime(glb) - 5))
    os.remove(glb)
    probs = verdict_problems('c', d, (), True)
    res = any(t == 'verdict' and 'missing' in x for t, x in probs)
    ok &= res
    print('selftest packing %-46s -> %s (%s)' % ('round-trip GLB missing (no sha skip)', 'REFUSED' if probs else 'accepted', 'as expected' if res else 'WRONG'))
    os.remove(os.path.join(d, 'door-close-c.json'))
    probs = verdict_problems('c', d, (), True)
    res = bool(probs)
    ok &= res
    print('selftest packing %-46s -> %s (%s)' % ('no verdict file', 'REFUSED' if probs else 'accepted', 'as expected' if res else 'WRONG'))
    # a conditional naming the wrong type (or the wrong door phase) must not release a row; the exact phase releases it
    for probs, cond, want_left, lbl in (([('door:jamb', 'x')], {'c': {'approach'}}, True, '--conditional of ANOTHER type does not release'),
                                        ([('door:jamb', 'x')], {'c': {'door:body'}}, True, '--conditional door:body does not release door:jamb'),
                                        ([('door:jamb', 'x')], {'c': {'door:jamb'}}, False, '--conditional door:jamb releases door:jamb'),
                                        ([('verdict', 'x')], {'c': {'verdict'}}, True, 'verdict problems are never released')):
        left = [p for p in probs if p[0] == 'verdict' or p[0] not in cond.get('c', ())]
        res = bool(left) == want_left
        ok &= res
        print('selftest packing %-46s -> %s (%s)' % (lbl, 'REFUSED' if left else 'released', 'as expected' if res else 'WRONG'))
    # the --conditional parser refuses a plain 'door' and unknown types
    for c in ('x:door', 'x:door:nonsense', 'x:bogus'):
        try:
            parse_conditional([c])
            res = False
        except SystemExit:
            res = True
        ok &= res
        print('selftest packing %-46s -> %s (%s)' % ('--conditional ' + c, 'refused' if res else 'ACCEPTED', 'as expected' if res else 'WRONG'))
    return ok


def carry_stamp(old, rows):
    """(archive_file, archive_sha256, idle_anims_sha256, warning) of the existing manifest `old` that stay valid for the new rows: the stamp is kept only if the staged .anims
    (row -> anims_sha256) are identical to the ones of `old`; a differing / unknown staging drops it (warning text). `old` None / unstamped -> nothing to carry, no warning."""
    if not old or not (old.get('archive_sha256') or old.get('idle_anims_sha256')):
        return None, None, {}, None
    old_map = {r: v.get('anims_sha256') for r, v in (old.get('rows') or {}).items()}
    new_map = {r: v.get('anims_sha256') for r, v in rows.items()}
    if old_map == new_map and all(new_map.values()):
        return old.get('archive_file'), old.get('archive_sha256'), old.get('idle_anims_sha256') or {}, None
    changed = sorted(r for r in set(old_map) | set(new_map) if old_map.get(r) != new_map.get(r))
    return None, None, {}, 'WARNING: the staged clips changed since the archive / idle stamp (%d row(s): %s%s) - archive_sha256 / idle_anims_sha256 dropped; repack and run --stamp-archive again' % (
        len(changed), ', '.join(changed[:4]), ', ...' if len(changed) > 4 else '')


def selftest_stamp():
    old = dict(archive_file='x.archive', archive_sha256='AA', idle_anims_sha256={'i.anims': 'BB'}, rows={'r1': dict(anims_sha256='11'), 'r2': dict(anims_sha256='22')})
    same = {'r1': dict(anims_sha256='11'), 'r2': dict(anims_sha256='22')}
    ok = True
    for lbl, o, rows, want_kept, want_warn in (('stamp kept: staged clips identical', old, same, True, False),
                                               ('stamp dropped + warning: one clip changed', old, dict(same, r2=dict(anims_sha256='23')), False, True),
                                               ('stamp dropped + warning: row added', old, dict(same, r3=dict(anims_sha256='33')), False, True),
                                               ('stamp dropped + warning: clip hash unknown', old, dict(same, r2=dict(anims_sha256=None)), False, True),
                                               ('unstamped manifest: nothing to carry, no warning', dict(old, archive_sha256=None, idle_anims_sha256={}), dict(same, r2=dict(anims_sha256='23')), False, False),
                                               ('no previous manifest', None, same, False, False)):
        af, ah, idle, warn = carry_stamp(o, rows)
        res = (bool(ah) == want_kept) and (bool(idle) == want_kept) and (bool(warn) == want_warn)
        ok &= res
        print('selftest packing %-46s -> %s (%s)' % (lbl, 'kept' if ah else 'none', 'as expected' if res else 'WRONG'))
    return ok


def parse_conditional(items):
    cond = {}
    for c in items:
        row, types = c.split(':', 1)
        ts = set(t.strip() for t in types.split(','))
        bad = ts - set(TYPES)
        if bad:
            hint = " (a plain 'door' is too coarse: name the phase, one of %s)" % ', '.join(DOOR_TYPES) if 'door' in bad else ''
            raise SystemExit('--conditional %s: unknown / non-conditional type(s) %s%s' % (c, sorted(bad), hint))
        cond.setdefault(row, set()).update(ts)
    return cond


if '--selftest' in sys.argv:
    sys.exit(0 if (selftest_verdict() & selftest_stamp()) else 1)

ap = argparse.ArgumentParser()
ap.add_argument('--base', default=os.path.join(ROOT, 'roof-pack2', 'sources-final-C2.txt'))
ap.add_argument('--out-root', default=os.path.join(ROOT, 'roof-pack3'))
ap.add_argument('--verdict-dir', action='append', default=[], help='directory with door-close-<clip>.json verdicts (re-solved with the current check_door_clearance.py); searched before the clip run dir')
ap.add_argument('--conditional', action='append', default=[], help='ROW:TYPE[,TYPE] (row = p?a_<class>_<seat>, types: %s): the row is packed although it has an open problem of exactly these types; written to the decision file' % ','.join(TYPES))
ap.add_argument('--reason', action='append', default=[], metavar='ROW=text', help='why the conditional row is accepted (mandatory for every --conditional row); written to the decision file')
ap.add_argument('--stamp-archive', action='store_true', help='add the sha256 of the packed --archive and of the --idle-dir passenger idle .anims to the existing pack-manifest-TAG.json (run after pack_car_workspot.ps1)')
ap.add_argument('--archive', default=None, help='with --stamp-archive: the packed utpp_vehicle_car.archive')
ap.add_argument('--idle-dir', default=None, help='with --stamp-archive: directory with the passenger idle .anims (utpp_car_p?a_<class>_passenger_r_idle.anims)')
ap.add_argument('tag')
ap.add_argument('rows', nargs='*')
a = ap.parse_args()
if a.stamp_archive:
    mp = os.path.join(a.out_root, 'pack-manifest-%s.json' % a.tag)
    if not (a.archive and os.path.exists(a.archive)):
        raise SystemExit('--stamp-archive needs an existing --archive')
    man = json.load(open(mp, encoding='utf-8'))
    man['archive_file'] = os.path.basename(a.archive)
    man['archive_sha256'] = sha256_of(a.archive)
    idle = {}
    if a.idle_dir:
        for fn in sorted(os.listdir(a.idle_dir)):
            if re.match(r'^utpp_car_p[wm]a_[a-z0-9]+_passenger_r_idle\.anims$', fn):
                idle[fn] = sha256_of(os.path.join(a.idle_dir, fn))
        if not idle:
            raise SystemExit('--idle-dir %s holds no utpp_car_p?a_*_passenger_r_idle.anims' % a.idle_dir)
    man['idle_anims_sha256'] = idle
    json.dump(man, open(mp, 'w', encoding='utf-8'), indent=1)
    print('stamped %s: archive %s sha256 %s, %d passenger idle clips' % (mp, man['archive_file'], man['archive_sha256'], len(idle)))
    sys.exit(0)
base = {}
for ln in open(a.base, encoding='utf-8').read().splitlines():
    if ln.strip():
        n, d = ln.split('  ', 1)
        base[n.strip()] = d.strip()
new = dict(base)
for r in a.rows:
    n, d = r.split('=', 1)
    n = n if n.startswith('utpp_car_') else 'utpp_car_' + n
    assert n in new, n
    new[n] = d if os.path.isabs(d) else os.path.join(a.out_root, d)
cond = parse_conditional(a.conditional)
reasons = {}
for s in a.reason:
    if '=' not in s or not s.split('=', 1)[1].strip():
        raise SystemExit('--reason needs ROW=text with a non-empty text: %r' % s)
    reasons[s.split('=', 1)[0].strip()] = s.split('=', 1)[1].strip()
for r_ in cond:
    if r_ not in reasons:
        raise SystemExit('--conditional %s needs --reason %s=<why the problem is accepted>' % (r_, r_))
decision, refused = {}, []
for n, d in sorted(new.items()):
    is_new = new[n] != base[n]
    row = n.replace('utpp_car_', '')
    probs = verdict_problems(n, d, a.verdict_dir, is_new)
    left = [p for p in probs if p[0] == 'verdict' or p[0] not in cond.get(row, ())]
    accepted = [p for p in probs if p not in left]
    packed = os.path.join(d, 'utpp_vehicle_car', 'utpp', 'vehicle', n + '.anims')
    rt = os.path.join(d, 'roundtrip', n + '.anims.glb')
    vp = find_verdict(n, d, a.verdict_dir)
    vj = json.load(open(vp, encoding='utf-8')) if vp else {}
    decision[row] = dict(status='NEW' if is_new else 'INHERITED', run=d, conditional_reason=reasons.get(row), verdict_path=vp, verdict_close_after_s=vj.get('close_after_s'), verdict_clip_sha256=vj.get('clip_sha256'),
                         anims_sha256=sha256_of(packed) if os.path.exists(packed) else None, glb_sha256=sha256_of(rt) if os.path.exists(rt) else None,
                         problems=[dict(type=t, text=x) for t, x in probs], conditional_accepted=[dict(type=t, text=x) for t, x in accepted], refused=[dict(type=t, text=x) for t, x in left],
                         inherited_without_approach_record=bool((not is_new) and approach_relative_fails(d, n) is None))
    if left:
        refused += ['%s [%s]: %s' % (row, t, x) for t, x in left]
os.makedirs(a.out_root, exist_ok=True)
json.dump(decision, open(os.path.join(a.out_root, 'pack-decision-%s.json' % a.tag), 'w', encoding='utf-8'), indent=1)
for row, dd in decision.items():
    for p in dd['conditional_accepted']:
        print('CONDITIONAL %s [%s]: %s' % (row, p['type'], p['text']))
if refused:
    print(chr(10).join(refused))
    raise SystemExit('PACKING REFUSED: %d problem(s) (decision file written)' % len(refused))
open(os.path.join(a.out_root, 'sources-final-%s.txt' % a.tag), 'w', encoding='utf-8', newline='\n').write(''.join('%s  %s\n' % (n, new[n]) for n in sorted(new)))
stage_root = os.path.join(a.out_root, 'stage-anims-%s' % a.tag)
stage = os.path.join(stage_root, 'utpp_vehicle_car', 'utpp', 'vehicle')
if os.path.exists(stage_root):
    shutil.rmtree(stage_root)
os.makedirs(stage)
for n, d in sorted(new.items()):
    src = os.path.join(d, 'utpp_vehicle_car', 'utpp', 'vehicle', n + '.anims')
    assert os.path.exists(src), src
    shutil.copy2(src, stage)
# MANIFEST: row -> clip sha256s, approach spot, seat variable, door close delta; the sha256 of the two generated runtime tables
sys.path.insert(0, HERE)
import gen_door_timing as GD  # noqa: E402
import gen_entry_approach as GA  # noqa: E402
tj = json.load(open(GD.TIMING, encoding='utf-8'))
dt = GD.table(tj)
ea = json.load(open(GA.JSON_PATH, encoding='utf-8'))
man = dict(tag=a.tag, archive_file=None, archive_sha256=None, idle_anims_sha256={}, rows={}, door_timing_reds_sha256=sha256_of(GD.REDS), approach_table_reds_sha256=sha256_of(GA.REDS), entry_approach_json_sha256=sha256_of(GA.JSON_PATH), door_timing_json_sha256=sha256_of(GD.TIMING))
for n in sorted(new):
    m = re.match(r'^utpp_car_p([wm])a_([a-z0-9]+)_(driver|passenger_r)$', n)
    g, cls, seat = m.groups()
    body = 'female' if g == 'w' else 'male'
    body_clip = 'pwa' if g == 'w' else 'pma'
    sv = GA.seat_var(cls, seat)
    spot = ea['classes'][cls]['seats'][seat]['spots'][body]
    man['rows'][n.replace('utpp_car_', '')] = dict(seat_var=sv, anims_sha256=decision[n.replace('utpp_car_', '')]['anims_sha256'], glb_sha256=decision[n.replace('utpp_car_', '')]['glb_sha256'],
                                                   approach_clip=spot.get('clip'), approach_clip_sha256=spot.get('clip_sha256'), spot=[spot['x'], spot['y'], spot['yaw_deg']],
                                                   door_close_delta_s=(dt[sv][1] if sv in dt else None), status=decision[n.replace('utpp_car_', '')]['status'],
                                                   door_timing_clip_sha256=(tj['clips'][cls][seat][body_clip].get('clip_sha256') if body_clip in tj['clips'].get(cls, {}).get(seat, {}) else None),
                                                   door_timing_close_after_s=(tj['clips'][cls][seat][body_clip].get('close_after_s') if body_clip in tj['clips'].get(cls, {}).get(seat, {}) else None),
                                                   verdict_path=decision[n.replace('utpp_car_', '')]['verdict_path'], verdict_close_after_s=decision[n.replace('utpp_car_', '')]['verdict_close_after_s'],
                                                   verdict_clip_sha256=decision[n.replace('utpp_car_', '')]['verdict_clip_sha256'])
mp_ = os.path.join(a.out_root, 'pack-manifest-%s.json' % a.tag)
try:
    old_man = json.load(open(mp_, encoding='utf-8')) if os.path.exists(mp_) else None
except (OSError, ValueError):
    old_man = None
man['archive_file'], man['archive_sha256'], man['idle_anims_sha256'], stamp_warning = carry_stamp(old_man, man['rows'])
if stamp_warning:
    print(stamp_warning)
elif man['archive_sha256']:
    print('kept the archive / idle stamp of the previous manifest (staged clips identical)')
json.dump(man, open(mp_, 'w', encoding='utf-8'), indent=1)
print('staged', len(os.listdir(stage)), 'clips;', sum(1 for n in new if new[n] != base[n]), 'replaced; decision + manifest written')
