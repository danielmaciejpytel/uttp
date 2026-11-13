"""Pairing check of an entry-clip archive with the runtime tables (round 4b, r10: not circular any more).

  python -I check_pack_manifest.py --manifest <pack-manifest-TAG.json> [--approach-reds <file>] [--door-reds <file>] [--door-timing <door_timing.json>] [--entry-approach <entry_approach.json>]
                                   [--archive-dir <dir with the staged .anims>] [--archive <packed .archive>] [--idle-dir <dir with the passenger idle .anims>] [--selftest]

make_pack.py writes the manifest next to the pack: per row the sha256 of the packed clip and of the round-tripped GLB it was exported from, the approach spot and its source-clip sha256, the
door close delta, the source-clip sha256 and the solved close of its door_timing.json entry, and the sha256 of the two generated runtime tables. This tool parses the reds files (default: the
plugin copies of this worktree) and the two json tables (default: this directory) and fails when
  - a row's approach spot (x, y, yaw) in utpp_vehicle_approach_table.reds differs from the manifest by more than 0.0015 m / 0.015 deg,
  - a seat's door close delta in utpp_vehicle_door_timing.reds differs from the manifest by more than 0.0005 s,
  - a table has a spot / delta for a row that is not in the manifest or vice versa,
  - the sha256 of a reds file differs from the manifest (any other change of the generated tables),
  - a row's approach_clip_sha256 (entry_approach.json, live file AND manifest) differs from the row's glb_sha256 (the spot was measured on another clip than the one that is packed),
  - a door_timing.json entry's clip_sha256 (live file AND manifest) differs from the row's glb_sha256 or the entry carries no sha (every entry must be stamped: gen_door_timing.py --merge /
    --stamp-sha); a seat with an entry for one gender only (single-gender merge) has none for the other row - that row is tied by the next rule only,
  - the SHIPPED close of the row's seat is earlier than the close the verdict solved for the packed clip (0.5 ms; a later shipped close is conservative). 2026-11-04 r11: the shipped close is the
    one of the reds file that ships (vanilla close of door_timing.json + the seat delta PARSED from utpp_vehicle_door_timing.reds), not the one the live json would generate,
  - the live door_timing.json / entry_approach.json differ from the sha256 the pack recorded (door_timing_json_sha256 / entry_approach_json_sha256: a json edited after the pack, reds not regenerated),
  - (with --archive-dir) a staged .anims differs from the manifest's anims_sha256 or is missing,
  - (with --archive) the packed .archive differs from the manifest's archive_sha256 (make_pack.py --stamp-archive records it),
  - (with --idle-dir) a passenger idle .anims differs from the manifest's idle_anims_sha256 or is missing.
A WARNING line is printed (not a mismatch) when --archive-dir / --archive / --idle-dir is not given or the manifest carries no stamp for it: make_pack.py keeps a stamp only while the staged contents
are unchanged, so an unstamped or unchecked archive / idle set is not tied to this manifest.
--selftest: a changed spot, a changed delta, an extra row, a changed reds file, an approach / door-timing sha that is not the packed clip, a changed close, a json edited after the pack with stale
reds (r11), a changed json file (sha256), and (with temp dirs) a tampered / missing staged clip, archive and idle clip must each be rejected, the unmodified inputs accepted.
"""
import argparse
import hashlib
import json
import os
import re
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gen_door_timing as GD  # noqa: E402
PLUGIN = os.path.normpath(os.path.join(HERE, '..', '..', '..', 'plugin', 'UTPP', 'scripts'))


def sha(path):
    return hashlib.sha256(open(path, 'rb').read()).hexdigest().upper()


def parse_spots(text):
    out = {}
    for m in re.finditer(r'if Equals\(seatVar, n"([a-z0-9_]+)"\) && (!?)female \{\s*a\.valid = true; a\.x = ([-0-9.]+); a\.y = ([-0-9.]+); a\.yawDeg = ([-0-9.]+);', text):
        out[(m.group(1), 'male' if m.group(2) == '!' else 'female')] = (float(m.group(3)), float(m.group(4)), float(m.group(5)))
    return out


def parse_deltas(text):
    return {m.group(1): float(m.group(2)) for m in re.finditer(r'if Equals\(seatVar, n"([a-z0-9_]+)"\) \{ return ([-0-9.]+); \}', text)}


def row_key(row):
    m = re.match(r'^p([wm])a_([a-z0-9]+)_(driver|passenger_r)$', row)
    g, cls, seat = m.groups()
    return g, cls, seat


def approach_entry(ea, row):
    g, cls, seat = row_key(row)
    return ea['classes'][cls]['seats'][seat]['spots']['female' if g == 'w' else 'male']


def timing_entry(tj, row):
    g, cls, seat = row_key(row)
    return tj['clips'][cls][seat]['pwa' if g == 'w' else 'pma']


def shipped_closes(tj, deltas):
    """{seat variable: close_after_s that SHIPS} = vanilla close of the class / door side (door_timing.json) + the seat delta parsed from the shipped utpp_vehicle_door_timing.reds
    (2026-11-04 r11: not GD.table(tj), which recomputes the delta from the live json and so cannot see a json edited after the reds were generated)."""
    rows = {}
    for cls, seats in tj.get('clips', {}).items():
        for seat in seats:
            if GD.seat_delta(tj, cls, seat) is None:
                continue
            var = '%s_vehicle_seat_%s' % (GD.VAR_CLASS.get(cls, cls), GD.SEAT_SIDE[seat])
            if var in deltas:
                rows[var] = GD.vanilla_close_after(tj, cls, seat) + deltas[var]
    return rows


def check_tables(man, tj, ea, deltas, tj_path=None, ea_path=None):
    """Sha / close pairing of the live door_timing.json + entry_approach.json and of the manifest copies with the packed clip; the close is the one of the shipped reds (deltas)."""
    bad = []
    shipped = shipped_closes(tj, deltas)
    for key, path in (('door_timing_json_sha256', tj_path), ('entry_approach_json_sha256', ea_path)):
        if path is None:
            continue
        if not man.get(key):
            bad.append('manifest has no %s (make_pack.py records it)' % key)
        elif sha(path) != str(man[key]).upper():
            bad.append('%s changed after the pack was made: sha256 %s... != manifest %s %s... (regenerate the reds from the json and repack, or restore the json)' % (os.path.basename(path), sha(path)[:12], key, str(man[key])[:12]))
    for row, r in sorted(man['rows'].items()):
        glb = str(r.get('glb_sha256') or '').upper()
        if not glb:
            bad.append('%s: manifest has no glb_sha256' % row)
            continue
        # approach: the live table AND the manifest copy must name the packed (round-tripped) clip
        try:
            sp = approach_entry(ea, row)
        except KeyError:
            bad.append('%s: no entry in entry_approach.json' % row)
            sp = None
        if sp is not None and str(sp.get('clip_sha256', '')).upper() != glb:
            bad.append('%s: entry_approach.json clip_sha256 %s... is not the packed clip (glb_sha256 %s...)' % (row, str(sp.get('clip_sha256', ''))[:12], glb[:12]))
        if str(r.get('approach_clip_sha256') or '').upper() != glb:
            bad.append('%s: manifest approach_clip_sha256 differs from glb_sha256' % row)
        # door timing: the entry's source clip and its solved close
        try:
            te = timing_entry(tj, row)
        except KeyError:
            te = None  # single-gender seat: the other gender's entry carries the seat close (checked below)
        if te is not None:
            ts = str(te.get('clip_sha256') or '').upper()
            if not ts:
                bad.append('%s: door_timing.json entry has no clip_sha256 (gen_door_timing.py --merge / --stamp-sha)' % row)
            elif ts != glb:
                bad.append('%s: door_timing.json clip_sha256 %s... is not the packed clip (glb_sha256 %s...)' % (row, ts[:12], glb[:12]))
            if str(r.get('door_timing_clip_sha256') or '').upper() != glb:
                bad.append('%s: manifest door_timing_clip_sha256 differs from glb_sha256' % row)
        vc = r.get('verdict_close_after_s')
        sc = shipped.get(r['seat_var'])
        if vc is None:
            bad.append('%s: manifest has no verdict_close_after_s' % row)
        elif sc is None:
            bad.append('%s: no shipped door close for %s (no delta in the door-timing reds or no door_timing.json entry)' % (row, r['seat_var']))
        elif sc < float(vc) - 0.0005:
            bad.append('%s: shipped door close %.3f s of %s (reds delta + vanilla close) is EARLIER than the %.3f s the verdict solved for the packed clip' % (row, sc, r['seat_var'], float(vc)))
        if r.get('verdict_clip_sha256') and str(r['verdict_clip_sha256']).upper() != glb:
            bad.append('%s: manifest verdict_clip_sha256 differs from glb_sha256' % row)
    return bad


def check(man, approach_text, door_text, approach_path, door_path, archive_dir=None, tj=None, ea=None, archive=None, idle_dir=None, tj_path=None, ea_path=None):
    bad = []
    spots, deltas = parse_spots(approach_text), parse_deltas(door_text)
    seen_s, seen_d = set(), set()
    for row, r in sorted(man['rows'].items()):
        body = 'female' if row.startswith('pwa') else 'male'
        k = (r['seat_var'], body)
        seen_s.add(k)
        if k not in spots:
            bad.append('%s: no approach spot for %s %s in the table' % (row, *k))
        else:
            x, y, yaw = spots[k]
            mx, my, myaw = r['spot']
            if abs(x - mx) > 0.0015 or abs(y - my) > 0.0015 or abs(((yaw - myaw + 180) % 360) - 180) > 0.015:
                bad.append('%s: approach spot table (%.3f, %.3f, %.2f) != manifest (%.3f, %.3f, %.2f)' % (row, x, y, yaw, mx, my, myaw))
        if r.get('door_close_delta_s') is not None:
            seen_d.add(r['seat_var'])
            d = deltas.get(r['seat_var'])
            if d is None:
                bad.append('%s: no door close delta for %s in the table' % (row, r['seat_var']))
            elif abs(d - r['door_close_delta_s']) > 0.0005:
                bad.append('%s: door close delta table %.3f != manifest %.3f (%s)' % (row, d, r['door_close_delta_s'], r['seat_var']))
        if archive_dir:
            f = os.path.join(archive_dir, 'utpp_car_%s.anims' % row)
            if not os.path.exists(f):
                bad.append('%s: staged clip missing (%s)' % (row, f))
            elif sha(f) != r['anims_sha256']:
                bad.append('%s: staged clip sha256 differs from the manifest' % row)
    for k in sorted(set(spots) - seen_s):
        bad.append('approach table has a spot for %s %s that is not a manifest row' % k)
    for k in sorted(set(deltas) - seen_d):
        if deltas[k] >= 0:
            bad.append('door timing table has a delta for %s that is no manifest seat' % k)
    for key, path in (('approach_table_reds_sha256', approach_path), ('door_timing_reds_sha256', door_path)):
        if path and os.path.exists(path) and sha(path) != man[key]:
            bad.append('%s differs from the manifest (%s)' % (os.path.basename(path), key))
    if tj is not None and ea is not None:
        bad += check_tables(man, tj, ea, deltas, tj_path, ea_path)
    if archive is not None:
        if not man.get('archive_sha256'):
            bad.append('manifest has no archive_sha256 (make_pack.py --stamp-archive)')
        elif not os.path.exists(archive):
            bad.append('archive missing: %s' % archive)
        elif sha(archive) != str(man['archive_sha256']).upper():
            bad.append('archive %s differs from the manifest archive_sha256' % os.path.basename(archive))
    if idle_dir is not None:
        idle = man.get('idle_anims_sha256') or {}
        if not idle:
            bad.append('manifest has no idle_anims_sha256 (make_pack.py --stamp-archive --idle-dir)')
        for name, h in sorted(idle.items()):
            f = os.path.join(idle_dir, name)
            if not os.path.exists(f):
                bad.append('passenger idle clip missing: %s' % f)
            elif sha(f) != str(h).upper():
                bad.append('passenger idle clip %s differs from the manifest' % name)
    return bad


def tie_warnings(man, archive_dir=None, archive=None, idle_dir=None):
    """WARNING lines for the optional ties that were not checked or have no stamp (a normal make_pack.py rerun resets the archive / idle stamp; --stamp-archive writes it again)."""
    out = []
    if archive_dir is None:
        out.append('--archive-dir not given: the staged .anims are not tied to the manifest anims_sha256')
    if archive is None:
        out.append('--archive not given: the packed .archive is not tied to the manifest' + ('' if man.get('archive_sha256') else ' (and the manifest has no archive_sha256 stamp: make_pack.py --stamp-archive)'))
    if idle_dir is None:
        out.append('--idle-dir not given: the passenger idle .anims are not tied to the manifest' + ('' if man.get('idle_anims_sha256') else ' (and the manifest has no idle_anims_sha256 stamp: make_pack.py --stamp-archive --idle-dir)'))
    return ['WARNING: ' + w for w in out]


def selftest(man, approach_path, door_path, tj=None, ea=None, tj_path=None, ea_path=None):
    at, dt = open(approach_path, encoding='utf-8').read(), open(door_path, encoding='utf-8').read()
    ok = True
    base_bad = check(man, at, dt, approach_path, door_path, None, tj, ea, None, None, tj_path, ea_path)
    ok &= not base_bad
    print('selftest manifest: tables as written -> %s' % ('PASS' if not base_bad else 'FAIL: ' + '; '.join(base_bad[:3])))
    row, r = next(iter(sorted(man['rows'].items())))
    body = 'female' if row.startswith('pwa') else 'male'
    sv = r['seat_var']
    m = re.search(r'(if Equals\(seatVar, n"%s"\) && %sfemale \{\s*a\.valid = true; a\.x = )([-0-9.]+)' % (sv, '' if body == 'female' else '!'), at)
    tests = [('spot x moved by 0.05 m', at[:m.start(2)] + ('%.3f' % (float(m.group(2)) + 0.05)) + at[m.end(2):], dt)]
    d_row = next(((k, v) for k, v in sorted(man['rows'].items()) if v.get('door_close_delta_s') is not None), None)
    if d_row:
        m2 = re.search(r'(if Equals\(seatVar, n"%s"\) \{ return )([-0-9.]+)' % d_row[1]['seat_var'], dt)
        tests.append(('door close delta changed by 0.05 s', at, dt[:m2.start(2)] + ('%.3f' % (float(m2.group(2)) + 0.05)) + dt[m2.end(2):]))
    tests.append(('extra approach spot row', at + '\n  if Equals(seatVar, n"zz_vehicle_seat_front_left") && female {\n    a.valid = true; a.x = 1.000; a.y = 1.000; a.yawDeg = 1.00;\n', dt))
    for name, a2, d2 in tests:
        rej = bool(check(man, a2, d2, None, None))
        print('selftest manifest: %-52s -> %s' % (name, 'REJECTED' if rej else 'NOT rejected'))
        ok &= rej
    # a reds FILE that is not the one of the manifest (sha comparison; the parsed values stay identical: a trailing comment)
    tmp = tempfile.mkdtemp(prefix='cpm_selftest_')
    try:
        for lbl, src, kw in (('approach reds file changed (sha256)', approach_path, 'approach'), ('door-timing reds file changed (sha256)', door_path, 'door')):
            p = os.path.join(tmp, os.path.basename(src))
            open(p, 'w', encoding='utf-8', newline='').write(open(src, encoding='utf-8', newline='').read() + '\n// tampered\n')
            args = (p, door_path) if kw == 'approach' else (approach_path, p)
            rej = bool(check(man, at, dt, args[0], args[1]))
            print('selftest manifest: %-52s -> %s' % (lbl, 'REJECTED' if rej else 'NOT rejected'))
            ok &= rej
        deltas0 = parse_deltas(dt)
        if tj is not None and ea is not None:
            # table pairing: the approach spot / door-timing entry belong to another clip than the packed one
            def mut(fn):
                m2_ = json.loads(json.dumps(man))
                t2, e2 = json.loads(json.dumps(tj)), json.loads(json.dumps(ea))
                fn(m2_, t2, e2)
                return bool(check_tables(m2_, t2, e2, deltas0))
            sp = approach_entry(ea, row)  # noqa: F841
            tests2 = [('entry_approach clip_sha256 != packed clip', lambda m_, t_, e_: approach_entry(e_, row).__setitem__('clip_sha256', '0' * 64)),
                      ('manifest approach_clip_sha256 != glb_sha256', lambda m_, t_, e_: m_['rows'][row].__setitem__('approach_clip_sha256', '0' * 64)),
                      ('door_timing clip_sha256 != packed clip', lambda m_, t_, e_: timing_entry(t_, row).__setitem__('clip_sha256', '0' * 64)),
                      ('door_timing entry without clip_sha256', lambda m_, t_, e_: timing_entry(t_, row).pop('clip_sha256', None)),
                      ('verdict close later than the shipped seat close', lambda m_, t_, e_: m_['rows'][row].__setitem__('verdict_close_after_s', float(m_['rows'][row]['verdict_close_after_s']) + 1.0)),
                      ('manifest door_timing_clip_sha256 != glb_sha256', lambda m_, t_, e_: m_['rows'][row].__setitem__('door_timing_clip_sha256', '0' * 64))]
            for lbl, fn in tests2:
                rej = mut(fn)
                print('selftest manifest: %-52s -> %s' % (lbl, 'REJECTED' if rej else 'NOT rejected'))
                ok &= rej
            # r11: json edited after the pack (close C1 >= verdict) while the shipped reds are stale (C0 < verdict): the old check read C1 from the live json and passed
            sv_ = man['rows'][row]['seat_var']
            c0 = shipped_closes(tj, deltas0).get(sv_)
            if c0 is None:
                print('selftest manifest: %-52s -> FAIL (no shipped close for %s)' % ('json edited after the pack, reds stale', sv_))
                ok = False
            else:
                m3, t3 = json.loads(json.dumps(man)), json.loads(json.dumps(tj))
                v_ = c0 + 0.3
                m3['rows'][row]['verdict_close_after_s'] = v_
                timing_entry(t3, row)['close_after_s'] = v_ + 0.2
                legacy_ok = GD.table(t3)[sv_][0] >= v_ - 0.0005  # what the previous check compared: the live json passes
                bad3 = check_tables(m3, t3, ea, deltas0)
                rej = any('EARLIER' in b_ for b_ in bad3)
                print('selftest manifest: %-52s -> %s' % ('json edited after the pack (C1>=verdict), reds stale (C0<)', 'REJECTED' if (rej and legacy_ok) else 'NOT rejected / scenario invalid (legacy_ok=%s)' % legacy_ok))
                ok &= rej and legacy_ok
            # r11: the live json differs from the sha256 the pack recorded (written to a temp file; same for entry_approach.json)
            for lbl, obj, key, kw in (('door_timing.json changed (recorded sha256)', tj, 'door_timing_json_sha256', 'tj'), ('entry_approach.json changed (recorded sha256)', ea, 'entry_approach_json_sha256', 'ea')):
                pj = os.path.join(tmp, key + '.json')
                open(pj, 'w', encoding='utf-8').write(json.dumps(obj))
                m4 = json.loads(json.dumps(man))
                m4[key] = sha(pj)
                paths = (pj if kw == 'tj' else None, pj if kw == 'ea' else None)
                good_ = not check_tables(m4, tj, ea, deltas0, *paths)
                open(pj, 'a', encoding='utf-8').write(' ')
                rej = bool(check_tables(m4, tj, ea, deltas0, *paths))
                m4.pop(key)
                rej_missing = bool(check_tables(m4, tj, ea, deltas0, *paths))
                print('selftest manifest: %-52s -> %s' % (lbl, 'REJECTED' if (good_ and rej and rej_missing) else 'FAIL (as recorded accepted=%s, changed rejected=%s, no key rejected=%s)' % (good_, rej, rej_missing)))
                ok &= good_ and rej and rej_missing
        # --archive-dir / --archive / --idle-dir on temp files
        stage = os.path.join(tmp, 'stage')
        os.makedirs(stage)
        man2 = json.loads(json.dumps(man))
        for rw, rr in man2['rows'].items():
            data = ('clip ' + rw).encode()
            open(os.path.join(stage, 'utpp_car_%s.anims' % rw), 'wb').write(data)
            rr['anims_sha256'] = hashlib.sha256(data).hexdigest().upper()
        arc = os.path.join(tmp, 'x.archive')
        open(arc, 'wb').write(b'archive bytes')
        man2['archive_sha256'] = hashlib.sha256(b'archive bytes').hexdigest().upper()
        idle = os.path.join(tmp, 'idle')
        os.makedirs(idle)
        open(os.path.join(idle, 'utpp_car_pwa_x_passenger_r_idle.anims'), 'wb').write(b'idle bytes')
        man2['idle_anims_sha256'] = {'utpp_car_pwa_x_passenger_r_idle.anims': hashlib.sha256(b'idle bytes').hexdigest().upper()}
        good = not check(man2, at, dt, None, None, stage, None, None, arc, idle)
        print('selftest manifest: %-52s -> %s' % ('staged dir + archive + idle clips as recorded', 'PASS' if good else 'FAIL'))
        ok &= good
        first = os.path.join(stage, 'utpp_car_%s.anims' % row)
        open(first, 'ab').write(b'x')
        rej = bool(check(man2, at, dt, None, None, stage))
        print('selftest manifest: %-52s -> %s' % ('--archive-dir: staged clip tampered', 'REJECTED' if rej else 'NOT rejected'))
        ok &= rej
        os.remove(first)
        rej = bool(check(man2, at, dt, None, None, stage))
        print('selftest manifest: %-52s -> %s' % ('--archive-dir: staged clip missing', 'REJECTED' if rej else 'NOT rejected'))
        ok &= rej
        open(arc, 'ab').write(b'x')
        rej = bool(check(man2, at, dt, None, None, None, None, None, arc))
        print('selftest manifest: %-52s -> %s' % ('--archive: .archive changed', 'REJECTED' if rej else 'NOT rejected'))
        ok &= rej
        os.remove(os.path.join(idle, 'utpp_car_pwa_x_passenger_r_idle.anims'))
        rej = bool(check(man2, at, dt, None, None, None, None, None, None, idle))
        print('selftest manifest: %-52s -> %s' % ('--idle-dir: idle clip missing', 'REJECTED' if rej else 'NOT rejected'))
        ok &= rej
        w_all = tie_warnings(man2, stage, arc, idle)
        w_none = tie_warnings(dict(man2, archive_sha256=None, idle_anims_sha256={}))
        w_ok = not w_all and len(w_none) == 3 and 'no archive_sha256 stamp' in w_none[1] and 'no idle_anims_sha256 stamp' in w_none[2]
        print('selftest manifest: %-52s -> %s' % ('WARNING lines for ties not given / not stamped', 'PASS' if w_ok else 'FAIL: %r / %r' % (w_all, w_none)))
        ok &= w_ok
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--manifest', required=True)
    ap.add_argument('--approach-reds', default=os.path.join(PLUGIN, 'utpp_vehicle_approach_table.reds'))
    ap.add_argument('--door-reds', default=os.path.join(PLUGIN, 'utpp_vehicle_door_timing.reds'))
    ap.add_argument('--door-timing', default=os.path.join(HERE, 'door_timing.json'))
    ap.add_argument('--entry-approach', default=os.path.join(HERE, 'entry_approach.json'))
    ap.add_argument('--archive-dir', default=None)
    ap.add_argument('--archive', default=None, help='the packed .archive (sha256 against the manifest archive_sha256)')
    ap.add_argument('--idle-dir', default=None, help='directory with the passenger idle .anims (sha256 against the manifest idle_anims_sha256)')
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args()
    man = json.load(open(a.manifest, encoding='utf-8'))
    tj = json.load(open(a.door_timing, encoding='utf-8'))
    ea = json.load(open(a.entry_approach, encoding='utf-8'))
    if a.selftest and not selftest(man, a.approach_reds, a.door_reds, tj, ea, a.door_timing, a.entry_approach):
        sys.exit(2)
    bad = check(man, open(a.approach_reds, encoding='utf-8').read(), open(a.door_reds, encoding='utf-8').read(), a.approach_reds, a.door_reds, a.archive_dir, tj, ea, a.archive, a.idle_dir, a.door_timing, a.entry_approach)
    for b in bad:
        print('MISMATCH: ' + b)
    for w in tie_warnings(man, a.archive_dir, a.archive, a.idle_dir):
        print(w)
    print('check_pack_manifest: %d row(s), %d mismatch(es)' % (len(man['rows']), len(bad)))
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
