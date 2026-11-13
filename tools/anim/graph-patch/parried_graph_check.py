"""Offline checks for the per-attack Parried patch of player_melee.animgraph (WolvenKit JSON). Read only.

  inspect  <patched.json>                   Parried subtree after the patch: states, leaves per switch index, conditional
                                            entries with every condition; checks that the default state holds the vanilla
                                            Switch object and that each new switch maps indices exactly like vanilla.
  diff     <vanilla.json> <patched.json>    shallow per-handle diff (child handles replaced by ids): lists every changed
                                            vanilla object and every new object; fails if anything outside the Parried
                                            state/output, the appended nodesToInit tail or the new objects differs.
  simulate <patched.json>                   evaluates the conditional entries + direction switch for fixed cases (FPP,
                                            TPP with / without the new clips, partial sets, other weapons, stale data,
                                            single-clip attack types block/safe/crouch/jump/sprint/equip with per-type fallback).
  compare  <old_patched.json> <new.json>    regression proof of an extended patch against an older one (run3): graph
                                            outside the nested SM identical, old states/entries canonically identical at
                                            the same indices, exhaustive selection sweep differs only for single-clip
                                            types whose pair is present in TPP.
Exit code 0 = all checks of the mode passed.
usage: python -I parried_graph_check.py <mode> <files...>
"""
import copy
import json
import sys

STATE = 'Parried'
COMBO_TYPES = (0, 1, 2)  # EMeleeAttackType Combo, Strong, Final: entry also checks comboNumber
SINGLE_TYPES = {3: 'block', 4: 'safe', 6: 'crouch', 7: 'jump', 8: 'sprint', 12: 'equip'}  # one clip per type, any comboNumber


def cn(v):
    return v.get('$value') if isinstance(v, dict) else v


def load(p):
    return json.load(open(p, encoding='utf-8'))


def index(d):
    H = {}

    def reg(o):
        if isinstance(o, dict):
            if 'HandleId' in o and 'Data' in o:
                H[o['HandleId']] = o['Data']
            for v in o.values():
                reg(v)
        elif isinstance(o, list):
            for v in o:
                reg(v)
    reg(d)
    return H


def res(H, link):
    if link is None:
        return None, None
    if 'HandleRefId' in link:
        return link['HandleRefId'], H[link['HandleRefId']]
    return link['HandleId'], link['Data']


def parried(d, H):
    sm = [n for n in H.values() if n.get('$type') == 'animAnimNode_StateMachine'
          and any(cn(res(H, s)[1]['name']) == STATE for s in n['states'])]
    assert len(sm) == 1, 'Parried state machine count %d' % len(sm)
    sm = sm[0]
    for s in sm['states']:
        h, n = res(H, s)
        if cn(n['name']) == STATE:
            return sm, h, n


def leaves_of_switch(H, sw):
    return [cn(res(H, i['node'])[1]['animation']) for i in sw['inputNodes']]


def direction_chain(H, sw):
    lh, latch = res(H, sw['weightNode']['node'])
    eh, expr = res(H, latch['input']['node'])
    vh, var = res(H, expr['expressionData']['floatSockets'][0]['link']['node'])
    return dict(latch=lh, latch_type=latch['$type'], expr=eh, expression=cn(expr['expressionString']),
                tokens=expr['expressionData']['expression']['Data']['tokenData'],
                values=expr['expressionData']['expression']['Data']['valuesData'],
                variable=cn(var['variableName']), var=vh)


def cond_text(c):
    t = c['$type'].replace('animAnimStateTransitionCondition_', '')
    if t == 'WrapperValue':
        return '%s %s %s' % (t, cn(c['wrapperName']), 'set' if c['checkIfWrapperIsSet'] else 'NOT set')
    if t == 'IntFeature':
        return '%s %s.%s %s %s' % (t, cn(c['featureName']), cn(c['featurePropertyName']), c['compareFunc'], c['compareValue'])
    if t == 'HasAnimation':
        return '%s %s' % (t, cn(c['animationName']))
    if t == 'BoolFeature':
        return '%s %s.%s == %s' % (t, cn(c['featureName']), cn(c['featurePropertyName']), c['compareValue'])
    return t


def entry_conditions(H, e):
    comp = res(H, e['condition'])[1]
    assert comp['$type'] == 'animAnimStateTransitionCondition_CompositeSimultaneous'
    return [res(H, c)[1] for c in comp['conditions']]


def inspect(p, quiet=False):
    d = load(p)
    H = index(d)
    sm, ph, ps = parried(d, H)
    out_h, out = res(H, ps['nodes'][0])
    nh, nsm = res(H, out['node']['node'])
    ok = True
    lines = ['Parried state #%s nodes=%s' % (ph, [x.get('HandleId') or x.get('HandleRefId') for x in ps['nodes']]),
             '  Output #%s -> %s #%s' % (out_h, nsm['$type'].replace('animAnimNode_', ''), nh)]
    if nsm['$type'] != 'animAnimNode_StateMachine':
        lines.append('  NOT PATCHED (vanilla Switch directly under Output)')
        print('\n'.join(lines))
        return d, H, None
    vanilla_map = None
    states = []
    for i, s in enumerate(nsm['states']):
        sh, st = res(H, s)
        o_h, o = res(H, st['nodes'][0])
        sw_h, sw = res(H, o['node']['node'])
        lv = leaves_of_switch(H, sw)
        dc = direction_chain(H, sw)
        if i == 0:
            vanilla_map = [x.replace('parried_right', 'R').replace('parried_left', 'L') for x in lv]
        m = ['R' if x.startswith('parried_right') else 'L' if x.startswith('parried_left') else '?' for x in lv]
        same = m == vanilla_map and dc['expression'] == 'IN / 90' and dc['variable'] == 'hit_direction'
        ok &= same
        states.append(dict(index=i, name=cn(st['name']), switch=sw_h, leaves=lv, chain=dc, nodes=len(st['nodes']),
                           outTransitions=st['outTransitionIndices']))
        lines.append('  state %2d %-26s Switch #%-5s blend=%s inputs=%s  weight: FloatLatch #%s <- "%s" <- %s #%s  %s'
                     % (i, cn(st['name']), sw_h, sw['blendTime'], lv, dc['latch'], dc['expression'], dc['variable'],
                        dc['var'], 'mapping=vanilla' if same else 'MAPPING DIFFERS'))
    lines.append('  defaultStateIndex=%d transitions=%d globalTransitions=%d' %
                 (nsm['defaultStateIndex'], len(nsm['transitions']), len(nsm['globalTransitions'])))
    ok &= nsm['defaultStateIndex'] == 0 and not nsm['transitions'] and not nsm['globalTransitions']
    entries = []
    for e in nsm['conditionalEntries']:
        ed = res(H, e)[1]
        cs = entry_conditions(H, ed)
        entries.append(dict(target=ed['targetStateIndex'], priority=ed['priority'], enabled=ed['isEnabled'],
                            forced=ed['isForcedToTrue'], conds=cs))
        lines.append('  entry -> state %2d prio %d enabled %d: %s' % (ed['targetStateIndex'], ed['priority'], ed['isEnabled'],
                                                                     ' AND '.join(cond_text(c) for c in cs)))
        # every new-clip state must be guarded by the wrapper and both of its own clip names
        tgt = states[ed['targetStateIndex']]
        names = {cn(c['animationName']) for c in cs if c['$type'].endswith('HasAnimation')}
        need = {x for x in tgt['leaves']}
        wr = [c for c in cs if c['$type'].endswith('WrapperValue')]
        # exactly one MeleeSlotData.attackType condition; comboNumber only for the multi-clip types (Combo/Strong/Final),
        # never for the single-clip types (Block/Safe/Crouch/Jump/Sprint/Equip, anim database key '*')
        ints = [c for c in cs if c['$type'].endswith('IntFeature')]
        at = [c for c in ints if cn(c['featurePropertyName']) == 'attackType']
        cb = [c for c in ints if cn(c['featurePropertyName']) == 'comboNumber']
        shape = len(at) == 1 and len(ints) == len(at) + len(cb) and all(cn(c['featureName']) == 'MeleeSlotData' for c in ints) \
            and ((at[0]['compareValue'] in COMBO_TYPES and len(cb) == 1) or
                 (at[0]['compareValue'] in SINGLE_TYPES and not cb and tgt['leaves'][0] == 'parried_right_' + SINGLE_TYPES[at[0]['compareValue']]))
        good = names == need and len(wr) == 1 and cn(wr[0]['wrapperName']) == 'utpp_foot_tpp' and wr[0]['checkIfWrapperIsSet'] == 1 \
            and ed['isEnabled'] == 1 and ed['isForcedToTrue'] == 0 and ed['targetStateIndex'] != 0 and shape \
            and len(cs) == len(wr) + len(ints) + len(names)
        ok &= good
        if not good:
            lines.append('    ^ GUARD INCOMPLETE')
    # default state = the vanilla switch object with parried_right/left
    ok &= states[0]['leaves'] == ['parried_right', 'parried_right', 'parried_left', 'parried_left', 'parried_left']
    lines.append('RESULT inspect: %s' % ('PASS' if ok else 'FAIL'))
    if not quiet:
        print('\n'.join(lines))
    return d, H, dict(states=states, entries=entries, ok=ok, default_switch=states[0]['switch'])


def shallow(o):
    if isinstance(o, dict):
        if 'HandleId' in o and 'Data' in o:
            return '@' + o['HandleId']
        if 'HandleRefId' in o:
            return '@' + o['HandleRefId']
        return {k: shallow(v) for k, v in o.items()}
    if isinstance(o, list):
        return [shallow(v) for v in o]
    return o


def shallow_data(data):
    return {k: shallow(v) for k, v in data.items()}


def diff(pv, pp):
    dv, dp = load(pv), load(pp)
    Hv, Hp = index(dv), index(dp)
    _, ph, ps = parried(dv, Hv)
    out_h = ps['nodes'][0]['HandleId']
    allowed = {ph, out_h}
    changed, missing = [], []
    for h, n in Hv.items():
        if h not in Hp:
            missing.append(h)
            continue
        if shallow_data(n) != shallow_data(Hp[h]):
            changed.append(h)
    new = sorted((h for h in Hp if h not in Hv), key=int)
    rv, rp = dv['Data']['RootChunk'], dp['Data']['RootChunk']
    init_v = [shallow(x) for x in rv['nodesToInit']]
    init_p = [shallow(x) for x in rp['nodesToInit']]
    init_prefix = init_p[:len(init_v)] == init_v
    tail = init_p[len(init_v):]
    tail_new = all(x[1:] in new for x in tail)
    rest_v = {k: shallow(v) for k, v in rv.items() if k != 'nodesToInit'}
    rest_p = {k: shallow(v) for k, v in rp.items() if k != 'nodesToInit'}
    types = {}
    for h in new:
        t = Hp[h].get('$type', '?')
        types[t] = types.get(t, 0) + 1
    print('vanilla handles %d, patched handles %d, new %d, missing %d' % (len(Hv), len(Hp), len(new), len(missing)))
    print('changed vanilla objects (shallow, child handles as ids):')
    for h in changed:
        a, b = shallow_data(Hv[h]), shallow_data(Hp[h])
        keys = [k for k in set(a) | set(b) if a.get(k) != b.get(k)]
        print('  #%s %s fields %s' % (h, Hv[h].get('$type'), keys))
        for k in keys:
            print('      vanilla: %s' % json.dumps(a.get(k))[:300])
            print('      patched: %s' % json.dumps(b.get(k))[:300])
    print('new objects by type:')
    for t, c in sorted(types.items()):
        print('  %4d %s' % (c, t))
    print('nodesToInit: vanilla %d, patched %d, vanilla prefix unchanged: %s, appended %d (all new objects: %s)'
          % (len(init_v), len(init_p), init_prefix, len(tail), tail_new))
    print('RootChunk fields other than nodesToInit unchanged: %s; animFeatures unchanged: %s; variables unchanged: %s'
          % (rest_v == rest_p, rv['animFeatures'] == rp['animFeatures'], rv['variables'] == rp['variables']))
    ok = not missing and set(changed) <= allowed and init_prefix and tail_new and rest_v == rest_p
    print('allowed changed set: %s; actual: %s' % (sorted(allowed), changed))
    print('RESULT diff: %s' % ('PASS' if ok else 'FAIL'))
    return ok


# ------------------------------------------------------------------ simulation
def switch_index(hit_direction, rule):
    x = hit_direction / 90.0
    i = int(x) if rule == 'floor' else int(x + 0.5)
    return max(0, min(4, i))


def evaluate(entries, case):
    """Index of the state the nested SM enters for a case (conditional entries by priority, else default state 0)."""
    for e in sorted(entries, key=lambda e: -e['priority']):
        if not e['enabled']:
            continue
        good = True
        for c in e['conds']:
            t = c['$type']
            if t.endswith('WrapperValue'):
                good &= (cn(c['wrapperName']) in case['wrappers']) == bool(c['checkIfWrapperIsSet'])
            elif t.endswith('IntFeature'):
                assert cn(c['featureName']) == 'MeleeSlotData' and c['compareFunc'] == 'Equal'
                good &= case[cn(c['featurePropertyName'])] == c['compareValue']
            elif t.endswith('HasAnimation'):
                good &= cn(c['animationName']) in case['clips']
            else:
                raise SystemExit('unknown condition ' + t)
        if good:
            return e['target']
    return 0


def pair(suffix):
    return {'parried_left_' + suffix, 'parried_right_' + suffix}


VAN = {'parried_left', 'parried_right'}
OLD_SUFFIXES = ['combo_%02d' % i for i in range(1, 5)] + ['final_%02d' % i for i in range(1, 6)] + ['heavy_01', 'heavy_02']
TYPE_SUFFIXES = [SINGLE_TYPES[t] for t in sorted(SINGLE_TYPES)]


def simulate(pp):
    d, H, info = inspect(pp, quiet=True)
    if info is None or not info['ok']:
        print('inspect failed')
        return False
    states, entries = info['states'], info['entries']
    van = VAN
    knife_new = van | {'parried_%s_combo_%02d' % (s, i) for s in ('left', 'right') for i in range(1, 5)}
    all_types = van.union(*(pair(s) for s in TYPE_SUFFIXES))
    everything = all_types.union(*(pair(s) for s in OLD_SUFFIXES))
    tpp = {'w_melee_knife', 'utpp_foot_tpp'}
    cases = [
        # ---- run3 cases (unchanged expectations)
        ('FPP knife, combo 2 (wrapper unset, vanilla set)', dict(wrappers={'w_melee_knife'}, clips=van, attackType=0, comboNumber=1), 'vanilla'),
        ('FPP knife, UTPP set active by mistake (no wrapper)', dict(wrappers={'w_melee_knife'}, clips=knife_new, attackType=0, comboNumber=1), 'vanilla'),
        ('TPP knife today (b1 r2 set, no new clips), combo 3', dict(wrappers=tpp, clips=van, attackType=0, comboNumber=2), 'vanilla'),
        ('TPP knife with new clips, combo 1', dict(wrappers=tpp, clips=knife_new, attackType=0, comboNumber=0), 'combo_01'),
        ('TPP knife with new clips, combo 2', dict(wrappers=tpp, clips=knife_new, attackType=0, comboNumber=1), 'combo_02'),
        ('TPP knife with new clips, combo 3', dict(wrappers=tpp, clips=knife_new, attackType=0, comboNumber=2), 'combo_03'),
        ('TPP knife with new clips, combo 4', dict(wrappers=tpp, clips=knife_new, attackType=0, comboNumber=3), 'combo_04'),
        ('TPP knife with new clips, final 4 (no final clips)', dict(wrappers=tpp, clips=knife_new, attackType=2, comboNumber=3), 'vanilla'),
        ('TPP knife with new clips, strong (no heavy clips)', dict(wrappers=tpp, clips=knife_new, attackType=1, comboNumber=0), 'vanilla'),
        ('TPP knife with new clips, safe attack (type 4)', dict(wrappers=tpp, clips=knife_new, attackType=4, comboNumber=0), 'vanilla'),
        ('TPP knife with new clips, comboNumber 4 (out of table)', dict(wrappers=tpp, clips=knife_new, attackType=0, comboNumber=4), 'vanilla'),
        ('TPP knife, only parried_left_combo_02 present', dict(wrappers=tpp, clips=van | {'parried_left_combo_02'}, attackType=0, comboNumber=1), 'vanilla'),
        ('TPP katana today, combo 2', dict(wrappers={'w_melee_katana', 'utpp_foot_tpp'}, clips=van, attackType=0, comboNumber=1), 'vanilla'),
        ('TPP hammer with final+heavy clips, heavy idx 3', dict(wrappers={'w_melee_hammer', 'utpp_foot_tpp'}, clips=van | pair('heavy_02'), attackType=1, comboNumber=3), 'heavy_02'),
        ('TPP hammer with final clips, final 5', dict(wrappers={'w_melee_hammer', 'utpp_foot_tpp'}, clips=van | pair('final_05'), attackType=2, comboNumber=4), 'final_05'),
        ('TPP knife new clips, MeleeSlotData never sent (defaults 0/0)', dict(wrappers=tpp, clips=knife_new, attackType=0, comboNumber=0), 'combo_01'),
    ]
    # ---- single-clip attack types (run4)
    for t in sorted(SINGLE_TYPES):
        s = SINGLE_TYPES[t]
        cases += [
            ('TPP knife, %s attack (type %d) with its pair' % (s, t), dict(wrappers=tpp, clips=van | pair(s), attackType=t, comboNumber=0), s),
            ('TPP knife, %s attack, stale comboNumber 3' % s, dict(wrappers=tpp, clips=van | pair(s), attackType=t, comboNumber=3), s),
            ('TPP knife, %s attack, every clip present' % s, dict(wrappers=tpp, clips=everything, attackType=t, comboNumber=0), s),
            ('TPP knife, %s attack, no %s clips (combo set)' % (s, s), dict(wrappers=tpp, clips=knife_new, attackType=t, comboNumber=0), 'vanilla'),
            ('TPP knife, %s attack, only parried_left_%s' % (s, s), dict(wrappers=tpp, clips=van | {'parried_left_' + s}, attackType=t, comboNumber=0), 'vanilla'),
            ('TPP knife, %s attack, only parried_right_%s' % (s, s), dict(wrappers=tpp, clips=van | {'parried_right_' + s}, attackType=t, comboNumber=0), 'vanilla'),
            ('FPP knife, %s attack, every clip present (no wrapper)' % s, dict(wrappers={'w_melee_knife'}, clips=everything, attackType=t, comboNumber=0), 'vanilla'),
        ]
    other = [x for x in TYPE_SUFFIXES if x != 'safe']
    cases += [
        ('TPP knife, safe attack, only the other type pairs', dict(wrappers=tpp, clips=van.union(*(pair(x) for x in other)), attackType=4, comboNumber=0), 'vanilla'),
        ('TPP knife, sprint attack, only safe pair', dict(wrappers=tpp, clips=van | pair('safe'), attackType=8, comboNumber=0), 'vanilla'),
        ('TPP knife, combo 1, only type pairs present', dict(wrappers=tpp, clips=all_types, attackType=0, comboNumber=0), 'vanilla'),
        ('TPP knife, combo 2, every clip present', dict(wrappers=tpp, clips=everything, attackType=0, comboNumber=1), 'combo_02'),
        ('TPP knife, final 5, every clip present', dict(wrappers=tpp, clips=everything, attackType=2, comboNumber=4), 'final_05'),
        ('TPP knife, strong idx 4, every clip present', dict(wrappers=tpp, clips=everything, attackType=1, comboNumber=4), 'heavy_01'),
    ]
    for t, nm in ((5, 'slide'), (9, 'fall'), (10, 'throw'), (11, 'dodge')):
        cases.append(('TPP katana, %s attack (type %d), every clip present' % (nm, t),
                      dict(wrappers={'w_melee_katana', 'utpp_foot_tpp'}, clips=everything, attackType=t, comboNumber=0), 'vanilla'))
    ok = True
    print('%-62s %-10s %-24s %s' % ('case', 'expected', 'state', 'leaf by hit_direction 0/45/90/135/180/225/270/315/359 (floor | round)'))
    for name, c, want in cases:
        si = evaluate(entries, c)
        st = states[si]
        got = 'vanilla' if si == 0 else st['name'].replace('Parried utpp ', '')
        dirs = [0, 45, 90, 135, 180, 225, 270, 315, 359]
        fl = [st['leaves'][switch_index(h, 'floor')].replace('parried_', '') for h in dirs]
        rd = [st['leaves'][switch_index(h, 'round')].replace('parried_', '') for h in dirs]
        # direction side must equal vanilla's for both rounding rules
        vs = states[0]['leaves']
        side_ok = all(st['leaves'][switch_index(h, r)].split('_')[1] == vs[switch_index(h, r)].split('_')[1]
                      for h in dirs for r in ('floor', 'round'))
        # the selected leaves must be exactly the clips of the expected suffix
        leaf_ok = set(st['leaves']) == (VAN if want == 'vanilla' else pair(want))
        good = got == want and side_ok and leaf_ok
        ok &= good
        print('%-62s %-10s %-24s %s | %s %s' % (name, want, got, ','.join(fl), ','.join(rd), '' if good else '  <-- FAIL'))
    print('RESULT simulate: %s (%d cases)' % ('PASS' if ok else 'FAIL', len(cases)))
    return ok


# ------------------------------------------------------------------ compare (regression proof against an older patch)
def expand(H, o, ids, stack=()):
    """Canonical deep form: references inlined, handle ids renumbered by first appearance inside the subtree."""
    if isinstance(o, dict):
        if 'HandleId' in o and 'Data' in o or 'HandleRefId' in o:
            h = o.get('HandleId', o.get('HandleRefId'))
            k = ids.setdefault(h, len(ids))
            if h in stack:
                return {'@cycle': k}
            return {'@': k, 'Data': expand(H, H[h], ids, stack + (h,))}
        return {key: expand(H, v, ids, stack) for key, v in o.items()}
    if isinstance(o, list):
        return [expand(H, v, ids, stack) for v in o]
    return o


def nested_sm(d, H):
    _, ph, ps = parried(d, H)
    out = res(H, ps['nodes'][0])[1]
    nh, nsm = res(H, out['node']['node'])
    assert nsm['$type'] == 'animAnimNode_StateMachine', 'not patched'
    return ps, out, nh, nsm


def outside_parried(d, H, n_init):
    """Canonical document with the nested SM cut out and nodesToInit truncated to the vanilla length."""
    d = copy.deepcopy(d)
    H2 = index(d)
    ps, out, _, _ = nested_sm(d, H2)
    out['node']['node'] = None
    ps['nodes'] = ps['nodes'][:1]
    d['Data']['RootChunk']['nodesToInit'] = d['Data']['RootChunk']['nodesToInit'][:n_init]
    ids = {}

    def rec(o):
        if isinstance(o, dict):
            return {k: (ids.setdefault(v, len(ids)) if k in ('HandleId', 'HandleRefId') else rec(v)) for k, v in o.items()}
        if isinstance(o, list):
            return [rec(v) for v in o]
        return o
    return rec(d['Data'])


def vanilla_init_len(d, H):
    """Length of the vanilla nodesToInit prefix = position of the nested SM (the patcher appends it first)."""
    _, _, nh, _ = nested_sm(d, H)
    refs = [x.get('HandleRefId') for x in d['Data']['RootChunk']['nodesToInit']]
    return refs.index(nh)


def compare(p_old, p_new):
    """p_old = an earlier patched graph (run3: combo/final/heavy only), p_new = the extended graph. PASS when
    (a) the graph outside the nested Parried SM is identical, (b) every old state / conditional entry is canonically
    identical at the same index, (c) an exhaustive selection sweep differs only where a new single-clip type has its pair."""
    do, dn = load(p_old), load(p_new)
    Ho, Hn = index(do), index(dn)
    n_init = vanilla_init_len(do, Ho)
    ok = n_init == vanilla_init_len(dn, Hn)
    a = outside_parried(do, Ho, n_init) == outside_parried(dn, Hn, n_init)
    print('graph outside the nested Parried SM (nodesToInit first %d) identical: %s' % (n_init, a))
    ok &= a
    _, _, _, so = nested_sm(do, Ho)
    _, _, _, sn = nested_sm(dn, Hn)
    for key in ('states', 'conditionalEntries'):
        lo, ln = so[key], sn[key]
        same = [expand(Ho, lo[i], {}) == expand(Hn, ln[i], {}) for i in range(len(lo))]
        print('%s: old %d, new %d; old indices canonically identical: %d/%d; added: %s' % (
            key, len(lo), len(ln), sum(same), len(lo),
            [cn(res(Hn, x)[1]['name']) if key == 'states' else 'entry->%d' % res(Hn, x)[1]['targetStateIndex'] for x in ln[len(lo):]]))
        ok &= all(same) and len(ln) >= len(lo)
    rest_o = {k: v for k, v in so.items() if k not in ('states', 'conditionalEntries')}
    rest_n = {k: v for k, v in sn.items() if k not in ('states', 'conditionalEntries')}
    r = expand(Ho, rest_o, {}) == expand(Hn, rest_n, {})
    print('state machine shell (default state, frozen state, interpolator, flags) identical: %s' % r)
    ok &= r
    # exhaustive selection sweep
    _, _, io = inspect(p_old, quiet=True)
    _, _, inn = inspect(p_new, quiet=True)
    ok &= io['ok'] and inn['ok']
    clip_sets = [('vanilla', VAN), ('all old', VAN.union(*(pair(s) for s in OLD_SUFFIXES))),
                 ('all old+new', VAN.union(*(pair(s) for s in OLD_SUFFIXES + TYPE_SUFFIXES))),
                 ('all new types', VAN.union(*(pair(s) for s in TYPE_SUFFIXES)))]
    for s in TYPE_SUFFIXES:
        clip_sets += [('pair ' + s, VAN | pair(s)), ('old + pair ' + s, VAN.union(*(pair(x) for x in OLD_SUFFIXES)) | pair(s)),
                      ('left only ' + s, VAN | {'parried_left_' + s}), ('right only ' + s, VAN | {'parried_right_' + s})]
    for s in OLD_SUFFIXES:
        clip_sets.append(('pair ' + s, VAN | pair(s)))
    total = diffs = bad = 0
    for wr in ({'w_melee_knife'}, {'w_melee_knife', 'utpp_foot_tpp'}):
        for cname_, clips in clip_sets:
            for t in range(13):
                for n in range(10):
                    case = dict(wrappers=wr, clips=clips, attackType=t, comboNumber=n)
                    lo = io['states'][evaluate(io['entries'], case)]['leaves']
                    ln = inn['states'][evaluate(inn['entries'], case)]['leaves']
                    total += 1
                    expect_new = 'utpp_foot_tpp' in wr and t in SINGLE_TYPES and pair(SINGLE_TYPES[t]) <= clips
                    if lo != ln:
                        diffs += 1
                    if expect_new:
                        good = lo == io['states'][0]['leaves'] and set(ln) == pair(SINGLE_TYPES[t])
                    else:
                        good = lo == ln
                    if not good:
                        bad += 1
                        if bad <= 10:
                            print('  MISMATCH wrappers=%s clips=%s type=%d combo=%d old=%s new=%s' % (sorted(wr), cname_, t, n, lo, ln))
    print('selection sweep: %d cases, %d differ old->new, all differences = single-clip type with its pair in TPP: %s'
          % (total, diffs, bad == 0))
    ok &= bad == 0
    print('RESULT compare: %s' % ('PASS' if ok else 'FAIL'))
    return ok


if __name__ == '__main__':
    mode = sys.argv[1]
    if mode == 'inspect':
        _, _, info = inspect(sys.argv[2])
        sys.exit(0 if info and info['ok'] else 1)
    if mode == 'diff':
        sys.exit(0 if diff(sys.argv[2], sys.argv[3]) else 1)
    if mode == 'simulate':
        sys.exit(0 if simulate(sys.argv[2]) else 1)
    if mode == 'compare':
        sys.exit(0 if compare(sys.argv[2], sys.argv[3]) else 1)
    raise SystemExit(__doc__)
