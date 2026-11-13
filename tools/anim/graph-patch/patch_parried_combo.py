"""Per-attack Parried clips for player_melee.animgraph (Cyberpunk 2077 2.5, WolvenKit JSON in -> JSON out).

Vanilla: state Parried (top melee state machine) = Output -> Switch(5 inputs: parried_right, parried_right, parried_left,
parried_left, parried_left) weighted by FloatLatch(MathExpression "IN / 90" of graph variable hit_direction). The clip only
depends on the hit direction, never on the parried attack.

Patch (copies the vanilla nested-state-machine pattern of the Ready state: default state + conditional entries with
HasAnimation, e.g. HasAnimation melee_idle_to_safe):
  Parried Output -> NEW StateMachine
    state 0 "Parried vanilla" (default): Output -> the SAME vanilla Switch object (unchanged subtree)
    state k "Parried utpp <dir-pair> <attack>": Output -> clone of the vanilla Switch (own FloatLatch / MathExpression /
      FloatVariable hit_direction copies, same 5-input direction mapping) with leaves parried_right_<suffix> /
      parried_left_<suffix> (clones of the vanilla SkAnim leaves: same parried_end loop event and flags)
    conditional entry per (attackType, comboNumber): CompositeSimultaneous of
      WrapperValue utpp_foot_tpp is set, IntFeature MeleeSlotData.attackType == T, IntFeature MeleeSlotData.comboNumber == N,
      HasAnimation parried_left_<suffix>, HasAnimation parried_right_<suffix>
    conditional entry per single-clip attack type (Block 3, Safe 4, Crouch 6, Jump 7, Sprint 8, Equip 12): the same without the
      comboNumber condition (the game's anim database player_melee_attacks.csv keys these types with ComboNumber '*', and
      their PSM states reset meleeAttackNumber before SendAnimationSlotData) -> parried_<left|right>_<block|safe|crouch|jump|sprint|equip>
  No transitions inside the new state machine; the Parried state's own transitions (to Ready / Deflect) are untouched.
Selection data: MeleeSlotData is sent only when an attack starts (MeleeAttackGenericEvents.OnEnter -> SendAnimationSlotData,
comboNumber = meleeAttackNumber before the increment, the same index the Attack state switches use); MeleeParriedEvents
resets meleeAttackNumber (MeleeData.attackNumber = 0 in Parried) but does not touch MeleeSlotData.
Fallback: without the wrapper (FPP) or without both clips in the active anim sets the entry is false -> default state =
vanilla behaviour. A weapon set opts in by containing both clip names of an entry.

Refuses any input that is not the vanilla 2.5 shape (Parried state, Switch layout, no UTPP names already present).
usage: python -I patch_parried_combo.py in.json out.json [--report report.json] [--no-type-entries]
  --no-type-entries  emit only the combo/final/heavy entries (the run3 graph, byte-identical JSON; regression proof)
"""
import copy
import json
import sys

WRAPPER = 'utpp_foot_tpp'
STATE_NAME = 'Parried'
# EMeleeAttackType: Combo 0, Strong 1, Final 2 (orphans.script; attackType sent as EnumInt).
# (attackType, comboNumber) -> clip suffix. Combo/Final follow the Attack state switches on attackNumber (index n ->
# combo_attack_0{n+1} / final_attack_0{n+1}); Strong follows Switch #1123 (even -> heavy_attack_01, odd -> heavy_attack_02).
ENTRIES = []
for n in range(4):
    ENTRIES.append((0, n, 'combo_%02d' % (n + 1)))
for n in range(5):
    ENTRIES.append((2, n, 'final_%02d' % (n + 1)))
for n in range(6):
    ENTRIES.append((1, n, 'heavy_%02d' % (n % 2 + 1)))
# Single-clip attack types (EMeleeAttackType Block 3, Safe 4, Crouch 6, Jump 7, Sprint 8, Equip 12): one attack clip each
# (player_melee_attacks.csv 2.5: "3;*;block_attack", "4;*;safe_attack", "6;*;crouch_attack", "7;*;jump_attack",
# "8;*;sprint_attack", "12;*;equip_attack"), so the entry ignores comboNumber (None = no comboNumber condition). Appended AFTER the legacy
# entries so the legacy states / entries keep their indices and handle allocation order.
TYPE_ENTRIES = [(3, None, 'block'), (4, None, 'safe'), (6, None, 'crouch'), (7, None, 'jump'), (8, None, 'sprint'),
                (12, None, 'equip')]
ENTRY_PRIORITY = 10


def cname(v):
    return {'$type': 'CName', '$storage': 'string', '$value': v}


def cn(v):
    return v.get('$value') if isinstance(v, dict) else v


class Graph:
    def __init__(self, d):
        self.d = d
        self.rc = d['Data']['RootChunk']
        self.H = {}
        self._reg(d)
        self.next_id = max(int(k) for k in self.H) + 1
        for k in self._all_ids(d):
            self.next_id = max(self.next_id, int(k) + 1)

    def _reg(self, o):
        if isinstance(o, dict):
            if 'HandleId' in o and 'Data' in o:
                self.H[o['HandleId']] = o['Data']
            for v in o.values():
                self._reg(v)
        elif isinstance(o, list):
            for v in o:
                self._reg(v)

    def _all_ids(self, o):
        if isinstance(o, dict):
            for k in ('HandleId', 'HandleRefId'):
                if k in o:
                    yield o[k]
            for v in o.values():
                yield from self._all_ids(v)
        elif isinstance(o, list):
            for v in o:
                yield from self._all_ids(v)

    def new_id(self):
        h = str(self.next_id)
        self.next_id += 1
        return h

    def res(self, link_node):
        """link_node = {'HandleId', 'Data'} or {'HandleRefId'} -> (id, data)."""
        if link_node is None:
            return None, None
        if 'HandleRefId' in link_node:
            return link_node['HandleRefId'], self.H[link_node['HandleRefId']]
        return link_node['HandleId'], link_node['Data']

    def first(self, pred):
        for h, n in self.H.items():
            if pred(n):
                return h, n
        raise SystemExit('template not found')


def strip_handles(o, idmap, g):
    """Deep copy of a JSON subtree; every inline HandleId gets a new id (references to ids defined inside the copied
    subtree are remapped, references to outside objects are kept)."""
    if isinstance(o, dict):
        if 'HandleId' in o and 'Data' in o:
            nid = idmap.setdefault(o['HandleId'], g.new_id())
            return {'HandleId': nid, 'Data': strip_handles(o['Data'], idmap, g)}
        if 'HandleRefId' in o:
            return {'HandleRefId': o['HandleRefId']}  # remapped in a second pass
        return {k: strip_handles(v, idmap, g) for k, v in o.items()}
    if isinstance(o, list):
        return [strip_handles(v, idmap, g) for v in o]
    return copy.deepcopy(o)


def remap_refs(o, idmap):
    if isinstance(o, dict):
        if 'HandleRefId' in o and o['HandleRefId'] in idmap:
            o['HandleRefId'] = idmap[o['HandleRefId']]
        for v in o.values():
            remap_refs(v, idmap)
    elif isinstance(o, list):
        for v in o:
            remap_refs(v, idmap)


def clone(o, g):
    idmap = {}
    c = strip_handles(o, idmap, g)
    remap_refs(c, idmap)
    return c


def ordered(new, template):
    """Key order of the vanilla template (WolvenKit writes RTTI order; handle ids are numbered in that order)."""
    r = {k: new[k] for k in template if k in new}
    r.update({k: v for k, v in new.items() if k not in r})
    return r


def bare(data, g):
    """New handle wrapper around a deep copy of node data (data must not contain inline handles)."""
    return {'HandleId': g.new_id(), 'Data': copy.deepcopy(data)}


def main():
    src, dst = sys.argv[1], sys.argv[2]
    report_path = sys.argv[sys.argv.index('--report') + 1] if '--report' in sys.argv else None
    entries_table = ENTRIES if '--no-type-entries' in sys.argv else ENTRIES + TYPE_ENTRIES
    d = json.load(open(src, encoding='utf-8'))
    raw = json.dumps(d)
    for marker in ('utpp', 'UTPP'):
        if marker in raw:
            raise SystemExit('input already contains %r - refusing (input must be the vanilla graph)' % marker)
    g = Graph(d)
    rc = g.rc

    # ---- locate the top melee state machine and its Parried state (vanilla shape checks)
    sms = [(h, n) for h, n in g.H.items() if n.get('$type') == 'animAnimNode_StateMachine'
           and any(cn(g.res(s)[1]['name']) == STATE_NAME for s in n['states'])]
    if len(sms) != 1:
        raise SystemExit('expected exactly one state machine with a %s state, found %d' % (STATE_NAME, len(sms)))
    sm_id, sm = sms[0]
    pidx = [cn(g.res(s)[1]['name']) for s in sm['states']].index(STATE_NAME)
    pstate_id, pstate = g.res(sm['states'][pidx])
    out_link = pstate['nodes'][0]
    out_id, out = g.res(out_link)
    if out['$type'] != 'animAnimNode_Output' or 'HandleId' not in out_link:
        raise SystemExit('Parried nodes[0] is not an inline Output')
    sw_link = out['node']['node']
    if 'HandleId' not in sw_link:
        raise SystemExit('expected the inline definition of the Parried switch under its Output')
    sw_id, sw = g.res(sw_link)
    if sw['$type'] != 'animAnimNode_Switch' or sw['numInputs'] != 5 or len(sw['inputNodes']) != 5:
        raise SystemExit('Parried switch shape changed: %s numInputs %s' % (sw['$type'], sw.get('numInputs')))
    leaves = [g.res(i['node']) for i in sw['inputNodes']]
    leaf_names = [cn(n['animation']) for _, n in leaves]
    if leaf_names != ['parried_right', 'parried_right', 'parried_left', 'parried_left', 'parried_left']:
        raise SystemExit('Parried switch inputs changed: %s' % leaf_names)
    right_id, right = leaves[0]
    left_id, left = leaves[2]
    latch_link = sw['weightNode']['node']
    latch_id, latch = g.res(latch_link)
    expr_id, expr = g.res(latch['input']['node'])
    if latch['$type'] != 'animAnimNode_FloatLatch' or expr['$type'] != 'animAnimNode_MathExpressionFloat' \
            or cn(expr['expressionString']) != 'IN / 90':
        raise SystemExit('Parried direction weight chain changed')
    var_id, var = g.res(expr['expressionData']['floatSockets'][0]['link']['node'])
    if var['$type'] != 'animAnimNode_FloatVariable' or cn(var['variableName']) != 'hit_direction':
        raise SystemExit('Parried direction variable changed')
    feats = {cn(f['name']): cn(f['className']) for f in rc['animFeatures']}
    if feats.get('MeleeSlotData') != 'AnimFeature_MeleeSlotData':
        raise SystemExit('MeleeSlotData feature not declared in the graph')
    old_nodes = [x.get('HandleId') or x.get('HandleRefId') for x in pstate['nodes']]
    expect_nodes = {out_id, sw_id, right_id, left_id, latch_id, expr_id, var_id}
    if set(old_nodes) != expect_nodes or len(old_nodes) != 7:
        raise SystemExit('Parried state node list changed: %s' % old_nodes)

    # ---- templates (vanilla objects of this graph)
    def has_cond_entry_with_hasanim(n):
        return n.get('$type') == 'animAnimNode_StateMachine' and 'HasAnimation' in json.dumps(n.get('conditionalEntries', []))
    tsm_id, tsm = g.first(has_cond_entry_with_hasanim)
    t_entry = tsm['conditionalEntries'][0]['Data']
    t_comp = t_entry['condition']['Data']
    t_hasanim = g.first(lambda n: n.get('$type') == 'animAnimStateTransitionCondition_HasAnimation')[1]
    t_wrap = g.first(lambda n: n.get('$type') == 'animAnimStateTransitionCondition_WrapperValue' and n.get('checkIfWrapperIsSet') == 1)[1]
    t_int = g.first(lambda n: n.get('$type') == 'animAnimStateTransitionCondition_IntFeature' and n.get('compareFunc') == 'Equal')[1]
    t_state = g.res(tsm['states'][3])[1]  # ToSafe: Output + one leaf
    if cn(t_state['name']) != 'ToSafe':
        raise SystemExit('template state changed')

    added = []  # (id, type) of every new node object (for nodesToInit and the report)

    def node(data):
        w = bare(data, g)
        added.append((w['HandleId'], data['$type']))
        return w

    def output_to(target_link_node):
        o = {k: copy.deepcopy(v) for k, v in out.items() if k != 'node'}
        o['node'] = {'$type': 'animPoseLink', 'node': target_link_node}
        return node(ordered(o, out))

    def state(name, nodes):
        s = {k: copy.deepcopy(v) for k, v in t_state.items() if k not in ('name', 'nodes', 'outTransitionIndices', 'tags')}
        s['name'] = cname(name)
        s['nodes'] = nodes
        s['outTransitionIndices'] = []
        s['tags'] = []
        w = bare(ordered(s, t_state), g)
        added.append((w['HandleId'], 'animAnimNode_State'))
        return w

    def leaf(src_leaf, anim):
        x = {k: copy.deepcopy(v) for k, v in src_leaf.items()}
        x['animation'] = cname(anim)
        return node(x)

    def cond(data):
        return {'HandleId': g.new_id(), 'Data': data}

    # ---- default state: the very same vanilla Switch object
    pstate['nodes'] = [out_link]  # rebuilt below: [Output, new SM]
    out['node']['node'] = None
    v_out = output_to(sw_link)  # inline definition of the vanilla Switch moves under the default state's Output
    v_state = state('Parried vanilla', [v_out] + [{'HandleRefId': h} for h in old_nodes if h != out_id])
    states = [v_state]
    combo_states = {}
    entries = []
    report_states = [{'index': 0, 'name': 'Parried vanilla', 'leaves': leaf_names, 'switch': sw_id, 'default': True}]
    for atype, num, suffix in entries_table:
        if suffix not in combo_states:
            r_leaf = leaf(right, 'parried_right_' + suffix)
            l_leaf = leaf(left, 'parried_left_' + suffix)
            # own direction chain FloatLatch <- MathExpression "IN / 90" <- FloatVariable hit_direction, defined inline in
            # the same nesting as vanilla (the expression object gets a new handle through clone())
            c_var = node({k: copy.deepcopy(v) for k, v in var.items()})
            c_expr_data = clone({k: v for k, v in expr.items() if k != 'expressionData'}, g)
            c_expr_data['expressionData'] = clone({k: v for k, v in expr['expressionData'].items() if k != 'floatSockets'}, g)
            sock = {k: copy.deepcopy(v) for k, v in expr['expressionData']['floatSockets'][0].items() if k != 'link'}
            sock['link'] = {'$type': 'animFloatLink', 'node': c_var}
            c_expr_data['expressionData']['floatSockets'] = [ordered(sock, expr['expressionData']['floatSockets'][0])]
            c_expr_data['expressionData'] = ordered(c_expr_data['expressionData'], expr['expressionData'])
            c_expr = node(ordered(c_expr_data, expr))
            c_latch_data = {k: copy.deepcopy(v) for k, v in latch.items() if k != 'input'}
            c_latch_data['input'] = {'$type': 'animFloatLink', 'node': c_expr}
            c_latch = node(ordered(c_latch_data, latch))
            c_sw_data = {k: copy.deepcopy(v) for k, v in sw.items() if k not in ('inputNodes', 'weightNode')}
            c_sw_data['inputNodes'] = [
                {'$type': 'animPoseLink', 'node': r_leaf},
                {'$type': 'animPoseLink', 'node': {'HandleRefId': r_leaf['HandleId']}},
                {'$type': 'animPoseLink', 'node': l_leaf},
                {'$type': 'animPoseLink', 'node': {'HandleRefId': l_leaf['HandleId']}},
                {'$type': 'animPoseLink', 'node': {'HandleRefId': l_leaf['HandleId']}},
            ]
            c_sw_data['weightNode'] = {'$type': 'animFloatLink', 'node': c_latch}
            c_sw = node(ordered(c_sw_data, sw))
            c_out = output_to(c_sw)
            refs = [{'HandleRefId': x['HandleId']} for x in (c_var, c_sw, c_expr, c_latch, l_leaf, r_leaf)]
            st = state('Parried utpp ' + suffix, [c_out] + refs)
            combo_states[suffix] = len(states)
            states.append(st)
            report_states.append({'index': combo_states[suffix], 'name': 'Parried utpp ' + suffix, 'switch': c_sw['HandleId'],
                                  'leaves': ['parried_right_' + suffix] * 2 + ['parried_left_' + suffix] * 3})
        # conditional entry
        conds = []
        w = copy.deepcopy(t_wrap)
        w['wrapperName'] = cname(WRAPPER)
        w['checkIfWrapperIsSet'] = 1
        conds.append(cond(w))
        for prop, val in (('attackType', atype), ('comboNumber', num)):
            if val is None:  # single-clip attack type: any comboNumber
                continue
            c = copy.deepcopy(t_int)
            c['featureName'] = cname('MeleeSlotData')
            c['featurePropertyName'] = cname(prop)
            c['compareFunc'] = 'Equal'
            c['compareValue'] = val
            c['debugInput'] = 0
            conds.append(cond(c))
        for side in ('left', 'right'):
            c = copy.deepcopy(t_hasanim)
            c['animationName'] = cname('parried_%s_%s' % (side, suffix))
            conds.append(cond(c))
        comp = {k: copy.deepcopy(v) for k, v in t_comp.items() if k != 'conditions'}
        comp['conditions'] = conds
        comp = ordered(comp, t_comp)
        e = {k: copy.deepcopy(v) for k, v in t_entry.items() if k not in ('condition',)}
        e['condition'] = cond(comp)
        e['isEnabled'] = 1
        e['isForcedToTrue'] = 0
        e['priority'] = ENTRY_PRIORITY
        e['targetStateIndex'] = combo_states[suffix]
        entries.append(cond(ordered(e, t_entry)))

    # ---- the new state machine (shell cloned from the vanilla nested SM, own frozen state / interpolator)
    shell = {k: v for k, v in tsm.items() if k not in ('states', 'transitions', 'globalTransitions', 'conditionalEntries')}
    nsm_data = clone(shell, g)
    nsm_data['states'] = states
    nsm_data['transitions'] = []
    nsm_data['globalTransitions'] = []
    nsm_data['conditionalEntries'] = entries
    nsm_data['defaultStateIndex'] = 0
    nsm_data = ordered(nsm_data, tsm)
    nsm = {'HandleId': g.new_id(), 'Data': nsm_data}
    added.insert(0, (nsm['HandleId'], 'animAnimNode_StateMachine'))
    fz = nsm_data['frozenState']
    added.append((fz['HandleId'], 'animAnimNode_StateFrozen'))
    added.append((fz['Data']['nodes'][0]['HandleId'], 'animAnimNode_Output'))
    out['node']['node'] = nsm
    pstate['nodes'] = [out_link, {'HandleRefId': nsm['HandleId']}]

    # ---- nodesToInit: append every new anim node (NueGraphPatch / ragdoll precedent)
    before = len(rc['nodesToInit'])
    for h, t in added:
        rc['nodesToInit'].append({'HandleRefId': h})
    json.dump(d, open(dst, 'w', encoding='utf-8'), indent=2)

    rep = {
        'input': src, 'output': dst, 'stateMachine': sm_id, 'parriedState': pstate_id, 'parriedOutput': out_id,
        'vanillaSwitch': sw_id, 'vanillaSwitchSameObject': True, 'newStateMachine': nsm['HandleId'],
        'templateStateMachine': tsm_id, 'wrapper': WRAPPER, 'states': report_states,
        'entries': [{'attackType': a, 'comboNumber': n, 'suffix': s, 'targetState': combo_states[s],
                     'clips': ['parried_left_' + s, 'parried_right_' + s]} for a, n, s in entries_table],
        'clipNames': sorted({'parried_%s_%s' % (side, s) for _, _, s in entries_table for side in ('left', 'right')}),
        'typeEntries': entries_table is not ENTRIES,
        'nodesToInitBefore': before, 'nodesToInitAfter': len(rc['nodesToInit']), 'addedNodes': len(added),
        'runtimeVerified': False,
    }
    if report_path:
        json.dump(rep, open(report_path, 'w', encoding='utf-8'), indent=2)
    print('Parried: vanilla Switch #%s kept as default state; new SM #%s with %d states, %d conditional entries; '
          'nodesToInit %d -> %d' % (sw_id, nsm['HandleId'], len(states), len(entries), before, len(rc['nodesToInit'])))


if __name__ == '__main__':
    main()
