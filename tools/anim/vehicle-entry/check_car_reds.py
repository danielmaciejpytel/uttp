"""Static check of utpp_vehicle_anim.reds for the car entry (2026-11-01): the SetAnimsetOverrideForPassenger wrap must admit every VehicleObject
mount (not only bikes) in UTPP third person, and keep value 0 / non-player / already-tagged calls alone.

  python -I check_car_reds.py <utpp_vehicle_anim.reds> [--old-reds <pre-car version>] [--selftest]

Checks on the wrap body (between '@wrapMethod(VehicleComponent)' and the next top-level '}'):
  1. the wrap resolves a VehicleObject (not only a BikeObject) from vehicleID;
  2. EntryWantsTpp is called with (IsDefined(bike), IsDefined(vehicle));
  3. EntryWantsTpp(isBike, isVehicle) returns false when !isVehicle and does NOT reject non-bikes (no '!isBike' gate);
  4. state.NoteEntryKind(IsDefined(bike)) comes before ArrayPush(utpp_vehicle_tpp) and before state.OnEntryVars(player, vars);
  5. value <= 0.0 returns early (exit / unmount untouched), the player-only cast and the ArrayContains guard are still there;
  6. the per-ride log line carries kind=.
--old-reds: must FAIL (the pre-car file); --selftest: mutations of the current file must each FAIL.
Exit 0 = pass (or, with --old-reds / --selftest, every negative case fails).
"""
import argparse
import re
import sys


def wrap_body(t):
    i = t.find('@wrapMethod(VehicleComponent)\npublic final static func SetAnimsetOverrideForPassenger')
    if i < 0:
        return None
    j = t.find('\n}\n', i)
    return t[i:j]


def entry_wants(t):
    m = re.search(r'public func EntryWantsTpp\((.*?)\) -> Bool \{(.*?)\n  \}\n', t, re.S)
    return (m.group(1), m.group(2)) if m else (None, None)


def check(t):
    errs = []
    w = wrap_body(t)
    if w is None:
        return ['no @wrapMethod(VehicleComponent) SetAnimsetOverrideForPassenger wrap']
    if 'as VehicleObject' not in w:
        errs.append('1: the wrap does not resolve a VehicleObject')
    if 'state.EntryWantsTpp(IsDefined(bike), IsDefined(vehicle))' not in w:
        errs.append('2: EntryWantsTpp is not called with (IsDefined(bike), IsDefined(vehicle))')
    sig, body = entry_wants(t)
    if sig is None or 'isVehicle' not in sig:
        errs.append('3: EntryWantsTpp has no isVehicle parameter')
    else:
        if not re.search(r'if !this\.m_enabled \|\| !isVehicle \{\s*return false;', body):
            errs.append('3: EntryWantsTpp does not return false for !isVehicle')
        if '!isBike' in body.split('isBike &&')[0] or re.search(r'!isBike\s*\{', body):
            errs.append('3: EntryWantsTpp still rejects non-bikes')
    k, p, o = w.find('state.NoteEntryKind(IsDefined(bike));'), w.find('ArrayPush(vars, n"utpp_vehicle_tpp");'), w.find('state.OnEntryVars(player, vars);')
    if k < 0 or p < 0 or o < 0 or not (k < p < o):
        errs.append('4: NoteEntryKind / push / OnEntryVars order (%d, %d, %d)' % (k, p, o))
    if 'value <= 0.0' not in w or 'passenger as PlayerPuppet' not in w or 'ArrayContains(vars, n"utpp_vehicle_tpp")' not in w:
        errs.append('5: value 0 / player-only / already-tagged guards missing')
    if 'kind=' not in t:
        errs.append('6: the ride log line has no kind=')
    return errs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('reds')
    ap.add_argument('--old-reds')
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args()
    t = open(a.reds, encoding='utf-8').read()
    errs = check(t)
    for e in errs:
        print('FAIL', e)
    ok = not errs
    if a.old_reds:
        old_errs = check(open(a.old_reds, encoding='utf-8').read())
        print('old reds: %d violations (must be > 0)' % len(old_errs))
        ok = ok and bool(old_errs)
    if a.selftest:
        muts = {
            'gate back to bike only': t.replace('EntryWantsTpp(IsDefined(bike), IsDefined(vehicle))', 'EntryWantsTpp(IsDefined(bike), IsDefined(bike))'),
            'no VehicleObject': t.replace('as VehicleObject;', 'as BikeObject;', 1),
            'isVehicle gate removed': t.replace('|| !isVehicle {', '|| !isBike {', 1),
            'kind noted after the push': t.replace('state.NoteEntryKind(IsDefined(bike));\n  ArrayPush(vars, n"utpp_vehicle_tpp");',
                                                  'ArrayPush(vars, n"utpp_vehicle_tpp");\n  state.NoteEntryKind(IsDefined(bike));', 1),
            'value 0 not guarded': t.replace('value <= 0.0 ||', '', 1),
            'no kind in the log': t.replace('kind="', 'k="'),
        }
        for name, m in muts.items():
            if m == t:
                print('SELFTEST FAIL: mutation did not change the text:', name)
                ok = False
            elif not check(m):
                print('SELFTEST FAIL: mutation not detected:', name)
                ok = False
            else:
                print('selftest %-28s rejected' % name)
    print('RESULT', 'PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
