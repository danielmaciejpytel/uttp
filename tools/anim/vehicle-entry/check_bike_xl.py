"""Validate utpp_vehicle_bike.archive.xl before it is written / packed.

  python -I check_bike_xl.py <bike.archive.xl> --tpp-xl <utpp_vehicle_tpp.archive.xl> --depot <dir with the .anims> [--selftest]

Rules (exit 0 = all pass):
  * 12 entries (sportbike 1/2/3 x female/male x enter_l/enter_r), each with exactly the vars
    [utpp_vehicle_tpp, utpp_bike_route_root, sportbike<1|2|3>_vehicle_seat_front_left], priority 130, component root
    (route 'root' of the seat pop A/B, 2026-10-28: at 94 these root-merged entries never played; 130 tests 'higher number wins'
    against the vanilla MotorcycleAnimsets entries FPP 119 / TPP 100);
  * the seat variable of every entry also appears in utpp_vehicle_tpp.archive.xl (the TPP riding set it complements) with the
    same sportbike number, and the set path is utpp\\vehicle\\utpp_tpp_p<w|m>a_sportbike[2|3]_enter_<l|r>.anims, an existing file
    in the depot; the female / male entity list matches the set name;
  * the number in the var matches the number in the set name (no number = 1).
--selftest runs the checks on mutated copies (each must fail): a clobbered variable (the PowerShell $n / $N clash), a wrong
set name, a missing depot file, a var absent from utpp_vehicle_tpp.archive.xl.

--reds <utpp_vehicle_anim.reds> [--game-dump <decompiled 2.50 scripts>] (2026-10-28, the FPP enter played in TPP): an entry can only
be picked by the entry workspot when every var is in the animVariables list the entering state hands to
WorkspotGameSystem.MountToVehicle, i.e. the list returned by VehicleComponent.SetAnimsetOverrideForPassenger. Checks: the reds wraps
that static function with the game's exact 4-argument signature, calls the original and pushes utpp_vehicle_tpp into the returned
list only for value > 0; the vars of every bike / TPP riding entry are exactly {utpp_vehicle_tpp, sportbikeN_vehicle_seat_front_left};
with --game-dump the dump still declares that signature, EnteringEvents.OnEnter assigns animVariables from it and passes
animVariables to MountToVehicle. --selftest adds: a wrap without the push, a wrong signature, a third var in an entry, a dump without
the hand-over (each must fail). With the route variables the wrap must also push utpp_bike_route_ext / utpp_bike_route_root, each
behind its route test (state.RouteExt() / state.RouteRoot()), before OnEntryVars.

--ext-json <patch-json dir> [--vanilla-json <dir with player_*_fpp*.ent.json>] [--ext-xl <file>] (route 'ext' / 'ride'): the
`resource: patch:` section at the end of the bike xl (or --ext-xl) adds, per gender, utpp\\vehicle\\utpp_bike_animsets_{wa,ma}.ent with props [components] to
exactly the two player FPP entities of that gender; the patch's compiled package holds one entAnimationSetupExtensionComponent
'UTPPMotorcycleAnimsets' (CRUID listed in CruidDict, not used by any vanilla component of the targets, so ArchiveXL appends it
and never replaces MotorcycleAnimsets), control binding to 'root' (enabled, NoBinding excluded) like the vanilla component,
isOverrideContainer 0, the same component in the components list (binding by handle reference); its 9 entries are the 6 UTPP
enter sets of that gender (priority 94, file in the depot, vars [utpp_vehicle_tpp, utpp_bike_route_ext, sportbikeN_vehicle_seat_
front_left]) and the 3 game TPP riding sets tpp_p?a_sportbikeN (priority 95, vars [utpp_vehicle_tpp, utpp_bike_route_ride, seat]);
no first-person set. The reds wrap also pushes utpp_bike_route_ride behind state.RouteRide(). --selftest adds mutations
(MotorcycleAnimsets name, a vanilla CRUID, binding to another component, a missing route var, an FPP set, a wrong-gender target,
props missing).
"""
import argparse
import json
import os
import re
import sys

VAR = re.compile(r'^sportbike([123])_vehicle_seat_front_left$')
ROUTE_ROOT = 'utpp_bike_route_root'
ROUTE_EXT = 'utpp_bike_route_ext'
ROUTE_RIDE = 'utpp_bike_route_ride'
ROOT_PRIORITY = '130'
SET = re.compile(r'^utpp\\vehicle\\utpp_tpp_p([wm])a_sportbike([23]?)_enter_([lr])\.anims$')


def split_xl(text):
    """(animations part, resource part) of the bike xl; the resource section starts at a top-level 'resource:' line and the
    comment block right above it."""
    lines = text.splitlines(keepends=True)
    for i, line in enumerate(lines):
        if line.rstrip() == 'resource:':
            j = i
            while j > 0 and lines[j - 1].lstrip().startswith('#'):
                j -= 1
            return ''.join(lines[:j]), ''.join(lines[j:])
    return text, ''


def parse(text):
    entries, cur = [], None
    for raw in text.splitlines():
        line = raw.rstrip()
        if not line or line.lstrip().startswith('#') or line.strip() == 'animations:':
            continue
        if re.match(r'^  - entity:$', line):
            cur = {'entity': [], 'component': None, 'set': None, 'vars': None, 'priority': None}
            entries.append(cur)
            continue
        m = re.match(r'^      - (.+)$', line)
        if m and cur is not None:
            cur['entity'].append(m.group(1))
            continue
        m = re.match(r'^    (component|set|priority|vars): (.+)$', line)
        if m and cur is not None:
            cur[m.group(1)] = m.group(2)
            continue
        raise ValueError('unparsed line: ' + line)
    return entries


def check(xl_text, tpp_text, depot):
    errs = []
    try:
        entries = parse(xl_text)
    except ValueError as e:
        return [str(e)]
    if len(entries) != 12:
        errs.append('expected 12 entries, got %d' % len(entries))
    tpp_vars = set(re.findall(r'sportbike[123]_vehicle_seat_front_left', tpp_text))
    seen = set()
    for i, e in enumerate(entries):
        tag = 'entry %d' % i
        if e['component'] != 'root':
            errs.append(tag + ': component ' + str(e['component']))
        if e['priority'] != ROOT_PRIORITY:
            errs.append(tag + ': priority ' + str(e['priority']))
        m = re.match(r'^\[utpp_vehicle_tpp, ' + ROUTE_ROOT + r', ([^\]]+)\]$', e['vars'] or '')
        if not m or not VAR.match(m.group(1)):
            errs.append(tag + ': bad vars ' + str(e['vars']))
            continue
        var = m.group(1)
        num = VAR.match(var).group(1)
        if var not in tpp_vars:
            errs.append(tag + ': ' + var + ' not in utpp_vehicle_tpp.archive.xl')
        sm = SET.match(e['set'] or '')
        if not sm:
            errs.append(tag + ': bad set ' + str(e['set']))
            continue
        g, b, side = sm.groups()
        if (b or '1') != num:
            errs.append(tag + ': var number %s != set number %s' % (num, b or '1'))
        if not os.path.isfile(os.path.join(depot, os.path.basename(e['set'].replace('\\', '/')))):
            errs.append(tag + ': set file missing in the depot: ' + e['set'])
        want = 'player_%sa_fpp' % g
        if not any(want in x for x in e['entity']) or len(e['entity']) != 2:
            errs.append(tag + ': entity list does not match the gender of ' + e['set'])
        key = (g, b, side)
        if key in seen:
            errs.append(tag + ': duplicate ' + str(key))
        seen.add(key)
    return errs


WRAP_SIG = ('public final static func SetAnimsetOverrideForPassenger(passenger: wref<GameObject>, vehicleID: EntityID, '
            'slotName: CName, value: Float) -> array<CName> {')
DUMP_SIG = ('public final static func SetAnimsetOverrideForPassenger(passenger: wref<GameObject>, vehicleID: EntityID, '
            'slotName: CName, value: Float) -> [CName] {')


def check_workspot_vars(reds_text, dump_text, xl_text, tpp_text):
    errs = []
    i = reds_text.find('@wrapMethod(VehicleComponent)\n' + WRAP_SIG)
    if i < 0:
        errs.append('reds: no @wrapMethod(VehicleComponent) with the 4-argument SetAnimsetOverrideForPassenger signature')
    else:
        body = reds_text[i:reds_text.find('\n}\n', i)]
        if 'wrappedMethod(passenger, vehicleID, slotName, value)' not in body:
            errs.append('reds: the wrap does not call wrappedMethod with the original arguments')
        if 'ArrayPush(vars, n"utpp_vehicle_tpp")' not in body or 'return vars;' not in body:
            errs.append('reds: the wrap does not push utpp_vehicle_tpp into the returned list')
        for test, var in (('state.RouteExt()', ROUTE_EXT), ('state.RouteRoot()', ROUTE_ROOT), ('state.RouteRide()', ROUTE_RIDE)):
            if not re.search(r'if ' + re.escape(test) + r' \{\s*ArrayPush\(vars, n"' + var + r'"\);\s*\}', body):
                errs.append('reds: the wrap does not push ' + var + ' behind ' + test)
            elif body.find('n"' + var + '"') > body.find('state.OnEntryVars(player, vars);'):
                errs.append('reds: ' + var + ' is pushed after OnEntryVars')
        if 'value <= 0.0' not in body:
            errs.append('reds: the wrap does not leave value 0 (exit / unmount) calls alone')
    for name, text, mid in (('bike xl', xl_text, [ROUTE_ROOT]), ('tpp xl', tpp_text, [])):
        for v in re.findall(r'vars: \[([^\]]*)\]', text):
            parts = [x.strip() for x in v.split(',')]
            if len(parts) != 2 + len(mid) or parts[0] != 'utpp_vehicle_tpp' or parts[1:-1] != mid or not VAR.match(parts[-1]):
                errs.append(name + ': vars [' + v + '] are not [utpp_vehicle_tpp, ' + ''.join(x + ', ' for x in mid)
                            + 'sportbikeN_vehicle_seat_front_left]')
    if dump_text is not None:
        if DUMP_SIG not in dump_text:
            errs.append('game dump: the 4-argument SetAnimsetOverrideForPassenger signature changed')
        j = dump_text.find('public class EnteringEvents extends VehicleEventsTransition {')
        k = dump_text.find('\n  public final func OnExit(', j) if j >= 0 else -1
        ent = dump_text[j:k] if j >= 0 and k > j else ''
        if 'animVariables = VehicleComponent.SetAnimsetOverrideForPassenger(scriptInterface.executionOwner, 1.00);' not in ent:
            errs.append('game dump: EnteringEvents.OnEnter no longer takes animVariables from SetAnimsetOverrideForPassenger')
        if 'entrySlotName, animVariables);' not in ent or 'workspotSystem.MountToVehicle(' not in ent:
            errs.append('game dump: EnteringEvents.OnEnter no longer passes animVariables to MountToVehicle')
    return errs


EXT_NAME = 'UTPPMotorcycleAnimsets'
EXT_PATCH = re.compile(r'^utpp\\vehicle\\utpp_bike_animsets_([wm])a\.ent$')
EXT_ENTER = re.compile(r'^utpp\\vehicle\\utpp_tpp_p([wm])a_sportbike([23]?)_enter_([lr])\.anims$')
EXT_RIDE = re.compile(r'^base\\animations\\vehicle\\sportbike([23]?)\\character\\tpp_p([wm])a_sportbike([23]?)\.anims$')
ENT = {g: ['base\\characters\\entities\\player\\player_%sa_fpp.ent' % g, 'ep1\\characters\\entities\\player\\player_%sa_fpp_ep1.ent' % g]
       for g in 'wm'}


def parse_ext_xl(text):
    """The `resource: patch:` map form only: {patch path: {'props': [...], 'targets': [...]}}."""
    out, cur, key = {}, None, None
    for raw in text.splitlines():
        line = raw.rstrip()
        if not line or line.lstrip().startswith('#'):
            continue
        if line in ('resource:', '  patch:'):
            continue
        m = re.match(r'^    (\S.*):$', line)
        if m:
            cur = out.setdefault(m.group(1), {'props': [], 'targets': []})
            key = None
            continue
        m = re.match(r'^      (props|targets):$', line)
        if m and cur is not None:
            key = m.group(1)
            continue
        m = re.match(r'^        - (.+)$', line)
        if m and cur is not None and key:
            cur[key].append(m.group(1))
            continue
        raise ValueError('ext xl: unparsed line: ' + line)
    return out


def vanilla_components(vanilla_dir):
    """{entity file stem: [(name, id), ...]} of the compiled components of the player FPP entities (2.31 extraction)."""
    out = {}
    if not vanilla_dir or not os.path.isdir(vanilla_dir):
        return out
    for g in 'wm':
        for stem in ('player_%sa_fpp' % g, 'player_%sa_fpp_ep1' % g):
            p = os.path.join(vanilla_dir, stem + '.ent.json')
            if os.path.isfile(p):
                root = json.load(open(p, encoding='utf-8'))['Data']['RootChunk']
                out[stem] = [((c.get('name') or {}).get('$value'), c.get('id'))
                             for c in root['compiledData']['Data']['Chunks'] if isinstance(c, dict) and 'id' in c]
    return out


def check_ext_component(c, g, depot, tag):
    errs = []
    if c.get('$type') != 'entAnimationSetupExtensionComponent':
        return [tag + ': not an entAnimationSetupExtensionComponent: ' + str(c.get('$type'))]
    if (c.get('name') or {}).get('$value') != EXT_NAME:
        errs.append(tag + ': name ' + str((c.get('name') or {}).get('$value')) + ' != ' + EXT_NAME)
    if c.get('isOverrideContainer') != 0:
        errs.append(tag + ': isOverrideContainer != 0')
    gp = ((c.get('animations') or {}).get('gameplay')) or []
    if len(gp) != 9 or (c.get('animations') or {}).get('cinematics') != []:
        errs.append(tag + ': expected 9 gameplay entries and no cinematics, got %d' % len(gp))
    seen = set()
    for i, e in enumerate(gp):
        et = '%s entry %d' % (tag, i)
        path = (((e.get('animSet') or {}).get('DepotPath')) or {}).get('$value', '')
        vars_ = [v.get('$value') for v in e.get('variableNames') or []]
        prio = e.get('priority')
        me, mr = EXT_ENTER.match(path), EXT_RIDE.match(path)
        if me:
            eg, b, side = me.groups()
            want_prio, num, key, route = 94, b or '1', ('enter', b, side), ROUTE_EXT
            if not os.path.isfile(os.path.join(depot, os.path.basename(path.replace('\\', '/')))):
                errs.append(et + ': set file missing in the depot: ' + path)
        elif mr:
            d, eg, b = mr.groups()
            if d != b:
                errs.append(et + ': folder sportbike%s != set sportbike%s' % (d, b))
            want_prio, num, key, route = 95, b or '1', ('ride', b, ''), ROUTE_RIDE
        else:
            errs.append(et + ': not a UTPP enter set or a game TPP riding set: ' + path)
            continue
        if eg != g:
            errs.append(et + ': %s set in the %sa patch' % (eg, g))
        if prio != want_prio:
            errs.append(et + ': priority %s != %d' % (prio, want_prio))
        if vars_ != ['utpp_vehicle_tpp', route, 'sportbike%s_vehicle_seat_front_left' % num]:
            errs.append(et + ': vars %s != [utpp_vehicle_tpp, %s, sportbike%s_vehicle_seat_front_left]' % (vars_, route, num))
        if key in seen:
            errs.append(et + ': duplicate ' + str(key))
        seen.add(key)
    return errs


def check_ext(ext_xl_text, json_dir, depot, vanilla):
    errs = []
    try:
        patches = parse_ext_xl(ext_xl_text)
    except ValueError as e:
        return [str(e)]
    genders = set()
    for ppath, d in patches.items():
        m = EXT_PATCH.match(ppath)
        if not m:
            errs.append('ext xl: patch path ' + ppath + ' is not utpp\\vehicle\\utpp_bike_animsets_{wa,ma}.ent')
            continue
        g = m.group(1)
        genders.add(g)
        if d['props'] != ['components']:
            errs.append('ext xl: ' + ppath + ' props ' + str(d['props']) + ' != [components]')
        if sorted(d['targets']) != sorted(ENT[g]):
            errs.append('ext xl: ' + ppath + ' targets ' + str(d['targets']) + ' != ' + str(ENT[g]))
        jp = os.path.join(json_dir, ppath.replace('\\', os.sep) + '.json')
        if not os.path.isfile(jp):
            errs.append('ext json missing: ' + jp)
            continue
        root = json.load(open(jp, encoding='utf-8'))['Data']['RootChunk']
        pkg = root['compiledData']['Data']
        chunks = pkg['Chunks']
        tag = os.path.basename(jp)
        if len(chunks) != 2 or chunks[0].get('$type') != 'entEntity':
            errs.append(tag + ': compiled package is not [entEntity, component]')
            continue
        c = chunks[1]
        errs += check_ext_component(c, g, depot, tag)
        cid = c.get('id')
        if pkg.get('CruidDict', {}).get('1') != cid:
            errs.append(tag + ': CruidDict does not list the component id')
        bind = (c.get('controlBinding') or {}).get('Data') or {}
        excl = [t.get('$value') for t in (((bind.get('enableMask') or {}).get('excludedTags') or {}).get('tags') or [])]
        if bind.get('$type') != 'entAnimationControlBinding' or (bind.get('bindName') or {}).get('$value') != 'root' \
                or bind.get('enabled') != 1 or excl != ['NoBinding']:
            errs.append(tag + ': control binding is not the vanilla one (root, enabled, NoBinding excluded)')
        listed = root.get('components') or []
        if len(listed) != 1 or {k: v for k, v in listed[0].items() if k != 'controlBinding'} != \
                {k: v for k, v in c.items() if k != 'controlBinding'} or \
                listed[0].get('controlBinding') != {'HandleRefId': (c.get('controlBinding') or {}).get('HandleId')}:
            errs.append(tag + ': components list does not hold the compiled component (binding by handle reference)')
        for stem in ('player_%sa_fpp' % g, 'player_%sa_fpp_ep1' % g):
            for name, vid in vanilla.get(stem, []):
                if vid == cid:
                    errs.append(tag + ': CRUID %s is used by %s:%s (ArchiveXL would replace / clash)' % (cid, stem, name))
                if name == EXT_NAME:
                    errs.append(tag + ': ' + stem + ' already has a component named ' + EXT_NAME)
    if genders != {'w', 'm'}:
        errs.append('ext xl: patches for genders %s, expected both' % sorted(genders))
    return errs


def ext_selftest(ext_xl_text, json_dir, depot, vanilla):
    """Mutations of the ext xl / patch JSON must fail (works on in-memory copies of the JSON)."""
    import copy
    import tempfile
    ok = True
    xmuts = {
        'props missing': ext_xl_text.replace('        - components\n', '', 1),
        'female patch on a male entity': ext_xl_text.replace('player_wa_fpp_ep1.ent', 'player_ma_fpp_ep1.ent', 1),
    }
    for name, t in xmuts.items():
        if t == ext_xl_text or not check_ext(t, json_dir, depot, vanilla):
            print('SELFTEST FAIL: ext mutation not detected:', name)
            ok = False
    src = os.path.join(json_dir, 'utpp', 'vehicle', 'utpp_bike_animsets_wa.ent.json')
    base = json.load(open(src, encoding='utf-8'))

    def comp(d):
        return d['Data']['RootChunk']['compiledData']['Data']['Chunks'][1]

    def m_name(d):
        comp(d)['name']['$value'] = 'MotorcycleAnimsets'

    def m_id(d):
        comp(d)['id'] = '2120206934485733376'
        d['Data']['RootChunk']['compiledData']['Data']['CruidDict']['1'] = '2120206934485733376'
        d['Data']['RootChunk']['components'][0]['id'] = '2120206934485733376'

    def m_bind(d):
        comp(d)['controlBinding']['Data']['bindName']['$value'] = 'deformations'

    def m_route(d):
        comp(d)['animations']['gameplay'][0]['variableNames'].pop(1)
        d['Data']['RootChunk']['components'][0]['animations']['gameplay'][0]['variableNames'].pop(1)

    def m_ridevar(d):
        for c in (comp(d), d['Data']['RootChunk']['components'][0]):
            c['animations']['gameplay'][7]['variableNames'][1]['$value'] = ROUTE_EXT

    def m_fpp(d):
        p = 'base\\animations\\vehicle\\sportbike\\character\\pwa_sportbike.anims'
        comp(d)['animations']['gameplay'][6]['animSet']['DepotPath']['$value'] = p

    jmuts = {'MotorcycleAnimsets name': m_name, 'vanilla CRUID': m_id, 'binding to another component': m_bind,
             'missing route var': m_route, 'first-person set': m_fpp, 'riding set on the enter route var': m_ridevar}
    for name, f in jmuts.items():
        d = copy.deepcopy(base)
        f(d)
        with tempfile.TemporaryDirectory() as td:
            for g in 'wm':
                rel = os.path.join('utpp', 'vehicle', 'utpp_bike_animsets_%sa.ent.json' % g)
                os.makedirs(os.path.join(td, 'utpp', 'vehicle'), exist_ok=True)
                body = d if g == 'w' else json.load(open(os.path.join(json_dir, rel), encoding='utf-8'))
                json.dump(body, open(os.path.join(td, rel), 'w', encoding='utf-8'))
            if not check_ext(ext_xl_text, td, depot, vanilla):
                print('SELFTEST FAIL: ext mutation not detected:', name)
                ok = False
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('xl')
    ap.add_argument('--tpp-xl', required=True)
    ap.add_argument('--depot', required=True)
    ap.add_argument('--selftest', action='store_true')
    ap.add_argument('--reds')
    ap.add_argument('--game-dump')
    ap.add_argument('--ext-xl')
    ap.add_argument('--ext-json')
    ap.add_argument('--vanilla-json')
    a = ap.parse_args()
    xl_full = open(a.xl, encoding='utf-8').read()
    xl, ext_in_xl = split_xl(xl_full)
    tpp = open(a.tpp_xl, encoding='utf-8').read()
    errs = check(xl, tpp, a.depot)
    reds = open(a.reds, encoding='utf-8').read() if a.reds else None
    dump = open(a.game_dump, encoding='utf-8', errors='replace').read() if a.game_dump else None
    if reds is not None:
        errs += check_workspot_vars(reds, dump, xl, tpp)
    ext = open(a.ext_xl, encoding='utf-8').read() if a.ext_xl else (ext_in_xl or None)
    vanilla = vanilla_components(a.vanilla_json)
    if a.ext_json and ext is None:
        errs.append('--ext-json given but the bike xl has no resource section (and no --ext-xl)')
    if ext is not None:
        if not a.ext_json:
            errs.append('the resource section needs --ext-json to be checked')
        else:
            errs += check_ext(ext, a.ext_json, a.depot, vanilla)
            print('ext: %d vanilla player entities read for the CRUID / name clash check' % len(vanilla))
    for e in errs:
        print('FAIL', e)
    ok = not errs
    if a.selftest:
        muts = {
            'clobbered var': xl.replace('sportbike1_vehicle_seat_front_left', 'sportbikeutpp_tpp_pwa_sportbike_enter_l_vehicle_seat_front_left', 1),
            'wrong set name': xl.replace('utpp_tpp_pwa_sportbike_enter_l', 'utpp_tpp_pwa_sportbike9_enter_l', 1),
            'var number != set number': xl.replace('sportbike2_vehicle_seat_front_left', 'sportbike3_vehicle_seat_front_left', 1),
            'old priority 94': xl.replace('priority: 130', 'priority: 94', 1),
            'route var missing': xl.replace('[utpp_vehicle_tpp, utpp_bike_route_root, ', '[utpp_vehicle_tpp, ', 1),
        }
        for name, t in muts.items():
            if not check(t, tpp, a.depot):
                print('SELFTEST FAIL: mutation not detected:', name)
                ok = False
        if not check(xl, tpp.replace('sportbike2_vehicle_seat_front_left', 'x'), a.depot):
            print('SELFTEST FAIL: var absent from the TPP xl not detected')
            ok = False
        if not check(xl, tpp, a.depot + '_missing'):
            print('SELFTEST FAIL: missing depot not detected')
            ok = False
        if reds is not None:
            rmuts = {
                'wrap without the push': (reds.replace('ArrayPush(vars, n"utpp_vehicle_tpp");', ''), dump, xl),
                'wrong wrap signature': (reds.replace('value: Float) -> array<CName> {', 'value: Int32) -> array<CName> {', 1), dump, xl),
                'fourth var in an entry': (reds, dump, xl.replace('vars: [utpp_vehicle_tpp, utpp_bike_route_root, sportbike1_vehicle_seat_front_left]',
                                                                    'vars: [utpp_vehicle_tpp, utpp_bike_route_root, player_tpp_head, '
                                                                    'sportbike1_vehicle_seat_front_left]', 1)),
                'wrap without the ext route push': (reds.replace('ArrayPush(vars, n"utpp_bike_route_ext");', ''), dump, xl),
                'root route push not behind its test': (reds.replace('if state.RouteRoot() {', 'if true {', 1), dump, xl),
            }
            if dump is not None:
                rmuts['dump without the hand-over'] = (reds, dump.replace('entrySlotName, animVariables);', 'entrySlotName);'), xl)
            for name, (r, d2, x2) in rmuts.items():
                if not check_workspot_vars(r, d2, x2, tpp):
                    print('SELFTEST FAIL: mutation not detected:', name)
                    ok = False
        if ext is not None and a.ext_json:
            ok &= ext_selftest(ext, a.ext_json, a.depot, vanilla)
    print('RESULT', 'PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
