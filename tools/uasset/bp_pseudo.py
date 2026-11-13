"""Offline Blueprint graph -> pseudo-code renderer for the source pack extraction.

Reads one per-Blueprint JSON produced by sourcepack_post.py (Extracted\\sourcepack\\sourcepack\\blueprints\\*.json)
and prints readable pseudo-code:
  * variables with CDO defaults,
  * every authored K2 graph (event graph, functions, macros) as structured statements
    following exec pins, with data pins rendered as nested expressions,
  * every AnimGraph / blend-stack / state graph as a pose tree from its root,
  * every state machine as states + transitions (+ rule expression from the transition graph).

Read-only: never touches the source project. Usage:
  python -I bp_pseudo.py <blueprint.json> [<out.txt>]
  python -I bp_pseudo.py --all <blueprints_dir> <out_dir>
"""
import json
import os
import re
import sys

GUID_RE = re.compile(r'_\d+_[0-9A-F]{32}$')
OPS = {
    'Add': '+', 'Subtract': '-', 'Multiply': '*', 'Divide': '/', 'Greater': '>',
    'GreaterEqual': '>=', 'Less': '<', 'LessEqual': '<=', 'EqualEqual': '==',
    'NotEqual': '!=', 'BooleanAND': '&&', 'BooleanOR': '||', 'BooleanXOR': '^^',
    'Percent': '%', 'BooleanNAND': 'nand', 'Concat': '++',
}
SKIP_ARG_PINS = {'self', 'execute', 'then', 'exec', 'WorldContextObject', '__WorldContext',
                 'ErrorTolerance'}
POSE_TYPES = ('/Script/Engine.PoseLink', '/Script/Engine.ComponentSpacePoseLink')
ENUMS = {}  # enum short name -> {internal_name: display_name}
SPLIT_RE = re.compile(r'^[A-Za-z]+_(X|Y|Z|W|Pitch|Yaw|Roll|R|G|B|A)$')
ENUM_TOKEN_RE = re.compile(r'(E_\w+|Enum_\w+)::(NewEnumerator\d+)')


def load_enums(bp_json_path):
    base = os.path.dirname(os.path.dirname(os.path.abspath(bp_json_path)))
    f = os.path.join(base, 'blueprints.json')
    if not os.path.exists(f) or ENUMS:
        return
    d = json.load(open(f, encoding='utf-8'))
    for e in d.get('enums', []):
        ENUMS[e['name']] = {x['internal_name']: (x.get('display_name') or x['internal_name'])
                            for x in e.get('entries', [])}


def enum_tr(enum_name, internal):
    m = ENUMS.get(enum_name)
    if m and internal in m:
        return m[internal].replace(' ', '')
    return internal


def tr_text(txt):
    return ENUM_TOKEN_RE.sub(lambda m: '%s::%s' % (m.group(1), enum_tr(m.group(1), m.group(2))), txt)


def clean(n):
    return GUID_RE.sub('', n or '')


def short_path(v):
    if not isinstance(v, str):
        return json.dumps(v, ensure_ascii=False)
    m = re.match(r"^[\w./]+'(.+)'$", v)
    if m:
        v = m.group(1)
    if v.startswith('/') and '.' in v:
        v = v.rsplit('.', 1)[-1]
    return v


def fmt_default(p):
    t = p.get('type')
    d = p.get('default')
    if d is None:
        if t == 'bool':
            return 'false'
        if t in ('real', 'int', 'int64', 'byte'):
            return '0'
        if t in ('object', 'class', 'softobject'):
            return 'None'
        if t == 'struct':
            to = p.get('type_object') or ''
            return short_path(to) + '()'
        return '<%s>' % t
    if t == 'bool':
        return d.lower()
    if t in ('object', 'class', 'softobject', 'softclass'):
        return short_path(d)
    if t == 'real':
        try:
            f = float(d)
            return ('%g' % f)
        except ValueError:
            return d
    if t in ('string', 'name', 'text'):
        return json.dumps(d, ensure_ascii=False) if not isinstance(d, str) else '"%s"' % d
    if t == 'byte' and p.get('type_object'):
        return '%s::%s' % (short_path(p['type_object']), d)
    return str(d)


GRAPHS_BY_PATH = {}
BINDINGS = {}  # 'AnimGraph.AnimGraphNode_X' -> [(prop, path_text, kind)]
BEGIN_RE = re.compile(r'^(\s*)Begin Object .*?ExportPath="[^"]*?:([^"\']+)\'?"')
BIND_RE = re.compile(r'\("([^"]+)", \(PinType=.*?PropertyName="[^"]*"(?:,ArrayIndex=\d+)?,PathAsText="([^"]*)"(.*?)bIsBound=(True|False)')


def load_bindings(t3d_path):
    BINDINGS.clear()
    if not t3d_path or not os.path.exists(t3d_path):
        return
    stack = []  # (indent, path)
    with open(t3d_path, encoding='utf-8-sig', errors='replace') as fh:
        for line in fh:
            m = BEGIN_RE.match(line)
            if m:
                ind = len(m.group(1))
                while stack and stack[-1][0] >= ind:
                    stack.pop()
                stack.append((ind, m.group(2)))
                continue
            if 'PropertyBindings=' in line and stack:
                ind = len(line) - len(line.lstrip())
                while stack and stack[-1][0] >= ind:
                    stack.pop()
                if not stack:
                    continue
                owner = stack[-1][1]
                if owner.rsplit('.', 1)[-1].startswith('AnimGraphNodeBinding'):
                    owner = owner.rsplit('.', 1)[0]
                for bm in BIND_RE.finditer(line):
                    kind = 'fn' if 'Type=Function' in bm.group(3) else 'prop'
                    if bm.group(4) == 'True':
                        BINDINGS.setdefault(owner, []).append((bm.group(1), bm.group(2), kind))


def quoted_tail(ref):
    m = re.search(r"'([^']*)'$", ref or '')
    return m.group(1) if m else (ref or '')


class Graph:
    def __init__(self, g, bp, parent=None, parent_node=None):
        self.g = g
        self.bp = bp
        self.nodes = {n['name']: n for n in g['nodes']}
        self.lines = []
        self.labels = {}
        self.parent = parent
        self.parent_node = parent_node
        self.input_sink = None
        if parent_node is not None:
            self.input_sink = quoted_tail(parent_node['props'].get('InputSinkNode', '')).rsplit('.', 1)[-1]

    def composite_child(self, node):
        bg = quoted_tail(node['props'].get('BoundGraph', '')).rsplit('.', 1)[-1]
        sub = GRAPHS_BY_PATH.get('%s.%s.%s' % (self.g.get('object_path'), node['name'], bg))
        if sub is None:
            return None
        return Graph(sub, self.bp, parent=self, parent_node=node)

    # ---------- pins ----------
    def pin(self, node, name, direction):
        for p in node['pins']:
            if p['name'] == name and p['dir'] == direction:
                return p
        return None

    def link_target(self, link):
        nname, _, pname = link.partition('.')
        return nname, pname

    def in_expr(self, node, pname, depth=0):
        p = self.pin(node, pname, 'in')
        if p is None:
            return '<?%s>' % pname
        return self.pin_value(p, depth)

    def pin_value(self, p, depth=0):
        links = p.get('links') or []
        if links:
            parts = [self.expr(*self.link_target(l), depth=depth + 1) for l in links]
            return parts[0] if len(parts) == 1 else '[' + ', '.join(parts) + ']'
        return fmt_default(p)

    # ---------- expressions ----------
    def call_name(self, node):
        props = node.get('props', {})
        fr = props.get('FunctionReference') or props.get('EventReference') or {}
        name = fr.get('MemberName') or node.get('label', '')
        parent = fr.get('MemberParent')
        if parent:
            cls = short_path(parent)
            if cls not in ('KismetMathLibrary', 'KismetSystemLibrary', 'KismetArrayLibrary',
                           'KismetStringLibrary', 'KismetTextLibrary'):
                return '%s::%s' % (cls, name)
        return name

    def is_pure(self, node):
        return not any(p['type'] == 'exec' for p in node['pins'])

    def args(self, node, depth):
        out = []
        for p in node['pins']:
            if p['dir'] != 'in' or p['type'] == 'exec' or p['name'] in SKIP_ARG_PINS:
                if p['name'] == 'self' and p.get('links'):
                    out.insert(0, 'target=' + self.pin_value(p, depth))
                continue
            if p.get('hidden') and not p.get('links'):
                continue
            out.append('%s=%s' % (clean(p['name']), self.pin_value(p, depth)))
        return out

    def expr(self, nname, pname, depth=0):
        if depth > 40:
            return '<deep>'
        node = self.nodes.get(nname)
        if node is None:
            return '<missing %s.%s>' % (nname, pname)
        c = node['class']
        props = node.get('props', {})
        if c == 'K2Node_Knot':
            return self.in_expr(node, 'InputPin', depth)
        if c in ('K2Node_VariableGet', 'K2Node_VariableSet'):
            vr = props.get('VariableReference', {})
            name = vr.get('MemberName', pname)
            selfp = self.pin(node, 'self', 'in')
            if selfp and selfp.get('links'):
                return '%s.%s' % (self.pin_value(selfp, depth), name)
            if vr.get('MemberParent') and not vr.get('bSelfContext'):
                return '%s.%s' % (short_path(vr['MemberParent']), name)
            if c == 'K2Node_VariableGet' and pname not in (name, 'Output_Get') and pname:
                return '%s.%s' % (name, pname)
            return name
        if c == 'K2Node_PropertyAccess':
            return '@' + '.'.join(props.get('Path', []))
        if c == 'K2Node_Self':
            return 'self'
        if c == 'K2Node_Tunnel' and self.parent is not None and nname == self.input_sink:
            return self.parent.in_expr(self.parent_node, pname, depth)
        if c == 'K2Node_Composite':
            child = self.composite_child(node)
            if child is not None:
                osrc = quoted_tail(props.get('OutputSourceNode', '')).rsplit('.', 1)[-1]
                onode = child.nodes.get(osrc)
                if onode is not None:
                    return child.in_expr(onode, pname, depth)
            return '<composite %s.%s>' % (nname, pname)
        if c in ('K2Node_FunctionEntry', 'K2Node_Tunnel'):
            return 'arg:' + clean(pname)
        if c in ('K2Node_Event', 'K2Node_CustomEvent', 'K2Node_EnhancedInputAction',
                 'K2Node_InputKey', 'K2Node_InputAction', 'K2Node_ComponentBoundEvent'):
            return 'evt:' + clean(pname)
        if c == 'K2Node_TemporaryVariable':
            return 'tmp_' + nname.rsplit('_', 1)[-1]
        if c in ('K2Node_CallFunction', 'K2Node_CommutativeAssociativeBinaryOperator',
                 'K2Node_PromotableOperator', 'K2Node_CallArrayFunction', 'K2Node_Message',
                 'K2Node_CallParentFunction', 'K2Node_CallMaterialParameterCollectionFunction'):
            if not self.is_pure(node):
                return 'r_%s.%s' % (nname.rsplit('_', 1)[-1], clean(pname)) if pname != 'ReturnValue' \
                    else 'r_%s' % nname.rsplit('_', 1)[-1]
            fname = self.call_name(node)
            base = fname.split('::')[-1]
            op = OPS.get(base.split('_')[0])
            a = self.args(node, depth)
            vals = [x.split('=', 1)[1] if '=' in x and not x.startswith('target=') else x for x in a]
            if base.startswith('Not_PreBool'):
                s = '!(%s)' % vals[0]
            elif op and len(vals) >= 2 and pname == 'ReturnValue' and not any(
                    SPLIT_RE.match(x.split('=', 1)[0]) for x in a if '=' in x):
                s = '(' + (' %s ' % op).join(vals) + ')'
                tp = self.pin(node, 'ErrorTolerance', 'in')
                if tp is not None and tp.get('default') not in (None, ''):
                    s = s[:-1] + ' ~tol=%s)' % fmt_default(tp)
            else:
                s = '%s(%s)' % (fname, ', '.join(a))
            if pname != 'ReturnValue':
                s += '.' + clean(pname)
            return s
        if c == 'K2Node_BreakStruct':
            ins = [p for p in node['pins'] if p['dir'] == 'in']
            src = self.pin_value(ins[0], depth) if ins else '?'
            return '%s.%s' % (src, clean(pname))
        if c == 'K2Node_MakeStruct':
            st = short_path(props.get('StructType', ''))
            return '%s{%s}' % (st, ', '.join(self.args(node, depth)))
        if c == 'K2Node_Select':
            idx = self.in_expr(node, 'Index', depth)
            ip = self.pin(node, 'Index', 'in') or {}
            en = short_path(ip.get('type_object', '')) if ip.get('type_object') else ''
            opts = [(enum_tr(en, p['name']) if en else p['name'], self.pin_value(p, depth)) for p in node['pins']
                    if p['dir'] == 'in' and p['name'] != 'Index']
            suffix = ''
            if pname and pname != 'ReturnValue' and pname.startswith('ReturnValue'):
                suffix = '.' + pname[len('ReturnValue_'):]
            if len(opts) == 2 and (props.get('IndexPinType', {}).get('PinCategory') == 'bool'):
                return '(%s ? %s : %s)%s' % (idx, opts[1][1], opts[0][1], suffix)
            return 'select(%s, {%s})%s' % (idx, ', '.join('%s: %s' % (k, v) for k, v in opts), suffix)
        if c in ('K2Node_EnumEquality', 'K2Node_EnumInequality'):
            op = '==' if c == 'K2Node_EnumEquality' else '!='
            return '(%s %s %s)' % (self.in_expr(node, 'A', depth), op, self.in_expr(node, 'B', depth))
        if c == 'K2Node_DynamicCast':
            t = short_path(props.get('TargetType', ''))
            o = self.in_expr(node, 'Object', depth)
            if props.get('PureState') == 'Pure' or self.is_pure(node):
                return ('IsA<%s>(%s)' % (t, o)) if pname == 'bSuccess' else 'Cast<%s>(%s)' % (t, o)
            return 'r_cast%s' % nname.rsplit('_', 1)[-1]
        if c == 'K2Node_AnimGetter':
            lab = node.get('label', '').replace('Call self::', '')
            src = short_path(props.get('SourceNode', ''))
            return '%s(%s%s)' % (lab, src, (', ' + ', '.join(self.args(node, depth))) if self.args(node, depth) else '')
        if c == 'K2Node_AnimNodeReference':
            return 'AnimNodeRef("%s")' % props.get('Tag', '')
        if c == 'K2Node_GetArrayItem':
            return '%s[%s]' % (self.in_expr(node, 'Array', depth), self.in_expr(node, 'Dimension 1', depth))
        if c == 'K2Node_MakeArray':
            return '[' + ', '.join(self.pin_value(p, depth) for p in node['pins'] if p['dir'] == 'in') + ']'
        if c == 'K2Node_GetEnumeratorNameAsString':
            return 'EnumName(%s)' % self.in_expr(node, 'Enumerator', depth)
        if c == 'K2Node_GetSubsystemFromPC':
            return 'GetSubsystem<%s>(%s)' % (short_path(props.get('CustomClass', '')),
                                             self.in_expr(node, 'PlayerController', depth))
        if c == 'K2Node_GetInputActionValue':
            return 'GetInputActionValue(%s)' % short_path(props.get('InputAction', ''))
        if c == 'K2Node_MacroInstance':
            mg = props.get('MacroGraphReference', {}).get('MacroGraph', '')
            mname = mg.rsplit(':', 1)[-1].rstrip("'")
            if self.is_pure(node):
                return '%s(%s).%s' % (mname, ', '.join(self.args(node, depth)), clean(pname))
            return 'r_%s.%s' % (mname, clean(pname))
        if c in ('K2Node_EvaluateChooser2', 'K2Node_EvaluateChooser'):
            if self.is_pure(node):
                return 'EvaluateChooser(%s, %s)' % (short_path(props.get('Chooser', '')),
                                                    ', '.join(self.args(node, depth)))
            return 'r_chooser%s.%s' % (nname.rsplit('_', 1)[-1], pname)
        if c == 'K2Node_SetFieldsInStruct':
            return self.in_expr(node, 'StructRef', depth)
        if c == 'K2Node_CreateDelegate':
            return '&' + props.get('SelectedFunctionName', '?')
        if c in ('K2Node_SpawnActorFromClass', 'K2Node_CreateWidget', 'K2Node_PlayMontage'):
            return 'r_%s.%s' % (nname, pname)
        if c == 'K2Node_ClassDynamicCast':
            return 'Cast<%s>(%s)' % (short_path(props.get('TargetType', '')), self.in_expr(node, 'Class', depth))
        if c == 'K2Node_StructMemberGet':
            return '%s.%s' % (short_path(props.get('StructType', '')), pname)
        if c == 'K2Node_GetClassDefaults':
            return 'CDO(%s).%s' % (self.in_expr(node, 'Class', depth), pname)
        if c.startswith('AnimGraphNode'):
            return '<pose %s>' % nname
        return '<%s:%s.%s>' % (c, nname, pname)

    # ---------- statements ----------
    def emit(self, text, ind):
        self.lines.append('    ' * ind + text)

    def exec_targets(self, node, pname):
        p = self.pin(node, pname, 'out')
        if not p:
            return []
        return [self.link_target(l)[0] for l in (p.get('links') or [])]

    def walk(self, start, ind, visited, depth=0):
        cur = start
        while cur:
            if depth > 60:
                self.emit('<too deep>', ind)
                return
            node = self.nodes.get(cur)
            if node is None:
                self.emit('<missing %s>' % cur, ind)
                return
            if cur in visited:
                self.emit('-> continue at [%s]' % cur, ind)
                return
            visited.add(cur)
            c = node['class']
            props = node.get('props', {})
            nxt = None
            cm = props.get('NodeComment')
            if cm:
                self.emit('// ' + cm.replace('\r', '').replace('\n', ' / '), ind)
            if c == 'K2Node_Knot':
                t = self.exec_targets(node, 'OutputPin')
                nxt = t[0] if t else None
            elif c == 'K2Node_IfThenElse':
                self.emit('if %s:   [%s]' % (self.in_expr(node, 'Condition'), cur), ind)
                for t in self.exec_targets(node, 'then'):
                    self.walk(t, ind + 1, visited, depth + 1)
                el = self.exec_targets(node, 'else')
                if el:
                    self.emit('else:', ind)
                    for t in el:
                        self.walk(t, ind + 1, visited, depth + 1)
                return
            elif c == 'K2Node_ExecutionSequence':
                outs = [p for p in node['pins'] if p['dir'] == 'out' and p['type'] == 'exec']
                for i, p in enumerate(outs):
                    tg = [self.link_target(l)[0] for l in (p.get('links') or [])]
                    if not tg:
                        continue
                    self.emit('sequence[%d]:' % i, ind)
                    for t in tg:
                        self.walk(t, ind + 1, visited, depth + 1)
                return
            elif c.startswith('K2Node_Switch'):
                sel = self.in_expr(node, 'Selection')
                en = short_path(props.get('Enum', '')) if props.get('Enum') else ''
                self.emit('switch %s %s:   [%s]' % (en, sel, cur), ind)
                for p in node['pins']:
                    if p['dir'] == 'out' and p['type'] == 'exec' and p.get('links'):
                        self.emit('case %s:' % (enum_tr(en, p['name']) if en else p['name']), ind)
                        for l in p['links']:
                            self.walk(self.link_target(l)[0], ind + 1, visited, depth + 1)
                return
            elif c in ('K2Node_VariableSet', 'K2Node_SetVariableOnPersistentFrame'):
                if c == 'K2Node_VariableSet':
                    name = self.expr(cur, '')
                    vr = props.get('VariableReference', {}).get('MemberName')
                    mainp = self.pin(node, vr, 'in') if vr else None
                    splits = [x for x in node['pins'] if x['dir'] == 'in' and vr and x['name'].startswith(vr + '_')]
                    if mainp is not None and not mainp.get('links') and any(x.get('links') for x in splits):
                        parts = ['%s=%s' % (x['name'][len(vr) + 1:], self.pin_value(x)) for x in splits]
                        self.emit('%s = {%s}' % (name, ', '.join(parts)), ind)
                    else:
                        val = self.in_expr(node, vr) if vr else '?'
                        self.emit('%s = %s' % (name, val), ind)
                else:
                    for p in node['pins']:
                        if p['dir'] == 'in' and p['type'] != 'exec':
                            self.emit('persist %s = %s' % (p['name'], self.pin_value(p)), ind)
                nxt = (self.exec_targets(node, 'then') or [None])[0]
            elif c == 'K2Node_AssignmentStatement':
                self.emit('%s = %s' % (self.in_expr(node, 'Variable'), self.in_expr(node, 'Value')), ind)
                nxt = (self.exec_targets(node, 'then') or [None])[0]
            elif c == 'K2Node_SetFieldsInStruct':
                st = self.in_expr(node, 'StructRef')
                fs = [('%s.%s = %s' % (st, clean(p['name']), self.pin_value(p))) for p in node['pins']
                      if p['dir'] == 'in' and p['type'] != 'exec' and p['name'] != 'StructRef']
                for f in fs:
                    self.emit(f, ind)
                nxt = (self.exec_targets(node, 'then') or [None])[0]
            elif c == 'K2Node_FunctionResult':
                outs = ['%s=%s' % (clean(p['name']), self.pin_value(p)) for p in node['pins']
                        if p['dir'] == 'in' and p['type'] != 'exec']
                self.emit('return ' + ', '.join(outs), ind)
                return
            else:
                exec_outs = [p for p in node['pins'] if p['dir'] == 'out' and p['type'] == 'exec']
                if c in ('K2Node_CallFunction', 'K2Node_CallArrayFunction', 'K2Node_Message',
                         'K2Node_CallParentFunction', 'K2Node_CommutativeAssociativeBinaryOperator'):
                    call = '%s(%s)' % (self.call_name(node), ', '.join(self.args(node, 0)))
                elif c == 'K2Node_MacroInstance':
                    mg = props.get('MacroGraphReference', {}).get('MacroGraph', '')
                    call = 'macro %s(%s)' % (mg.rsplit(':', 1)[-1].rstrip("'"), ', '.join(self.args(node, 0)))
                elif c in ('K2Node_EvaluateChooser2', 'K2Node_EvaluateChooser'):
                    call = 'EvaluateChooser(%s, mode=%s, %s)' % (short_path(props.get('Chooser', '')),
                                                                props.get('Mode', 'FirstResult'),
                                                                ', '.join(self.args(node, 0)))
                elif c == 'K2Node_DynamicCast':
                    call = 'Cast<%s>(%s)' % (short_path(props.get('TargetType', '')), self.in_expr(node, 'Object'))
                elif c == 'K2Node_AssignDelegate':
                    call = 'bind %s += %s' % (props.get('DelegateReference', {}).get('MemberName'),
                                              self.in_expr(node, 'Delegate'))
                else:
                    call = '%s %s(%s)' % (c.replace('K2Node_', ''), node.get('label', ''), ', '.join(self.args(node, 0)))
                outs_data = [p['name'] for p in node['pins'] if p['dir'] == 'out' and p['type'] != 'exec'
                             and p.get('links')]
                prefix = ''
                if outs_data:
                    prefix = 'r_%s{%s} = ' % (cur.rsplit('_', 1)[-1] if c != 'K2Node_MacroInstance' else
                                              props.get('MacroGraphReference', {}).get('MacroGraph', '').rsplit(':', 1)[-1].rstrip("'"),
                                              ','.join(clean(x) for x in outs_data))
                linked_outs = [p for p in exec_outs if p.get('links')]
                if len(linked_outs) <= 1 and (not linked_outs or linked_outs[0]['name'] in ('then', 'Then', 'execute', 'Exec', 'OutputPin')):
                    self.emit(prefix + call, ind)
                    nxt = self.link_target(linked_outs[0]['links'][0])[0] if linked_outs else None
                else:
                    self.emit(prefix + call + ':   [%s]' % cur, ind)
                    for p in linked_outs:
                        self.emit('on %s:' % p['name'], ind)
                        for l in p['links']:
                            self.walk(self.link_target(l)[0], ind + 1, visited, depth + 1)
                    return
            cur = nxt

    ENTRY_CLASSES = ('K2Node_FunctionEntry', 'K2Node_Event', 'K2Node_CustomEvent',
                     'K2Node_EnhancedInputAction', 'K2Node_InputKey', 'K2Node_InputAction',
                     'K2Node_ComponentBoundEvent', 'K2Node_Tunnel', 'K2Node_InputAxisEvent')

    def render_k2(self):
        entries = [n for n in self.g['nodes'] if n['class'] in self.ENTRY_CLASSES]
        visited = set()
        for e in entries:
            outs = [p for p in e['pins'] if p['dir'] == 'out' and p['type'] == 'exec' and p.get('links')]
            params = [clean(p['name']) for p in e['pins'] if p['dir'] == 'out' and p['type'] != 'exec'
                      and p['type'] != 'delegate']
            hdr = e.get('label', e['name'])
            if e['class'] == 'K2Node_InputKey':
                hdr = 'InputKey %s' % e['props'].get('InputKey')
            if not outs:
                if e['class'] == 'K2Node_FunctionEntry':
                    # pure function: render results directly
                    for r in [n for n in self.g['nodes'] if n['class'] == 'K2Node_FunctionResult']:
                        self.emit('%s(%s):' % (hdr, ', '.join(params)), 0)
                        self.walk(r['name'], 1, visited)
                continue
            self.emit('%s(%s):' % (hdr, ', '.join(params)), 0)
            for p in outs:
                if len(outs) > 1 or p['name'] not in ('then',):
                    self.emit('on %s:' % p['name'], 1)
                for l in p['links']:
                    self.walk(self.link_target(l)[0], 2 if (len(outs) > 1 or p['name'] != 'then') else 1, visited)
        # pure functions with result but no exec link from entry
        results = [n for n in self.g['nodes'] if n['class'] == 'K2Node_FunctionResult' and n['name'] not in visited]
        for r in results:
            self.emit('[unreached result %s]' % r['name'], 0)
            self.walk(r['name'], 1, visited)

    # ---------- anim graphs ----------
    def anim_node_desc(self, node):
        props = dict(node.get('props', {}))
        for k in ('NodeGuid', 'bHasCompilerMessage', 'ErrorType', 'ErrorMsg', 'NodeComment'):
            props.pop(k, None)
        s = '%s [%s]' % (node['class'].replace('AnimGraphNode_', ''), node['name'])
        if props:
            s += ' ' + json.dumps(props, ensure_ascii=False)
        return s

    def render_pose(self, nname, ind, visited):
        node = self.nodes.get(nname)
        if node is None:
            self.emit('<missing %s>' % nname, ind)
            return
        if nname in visited:
            self.emit('-> (shared) %s' % nname, ind)
            return
        visited.add(nname)
        cm = node.get('props', {}).get('NodeComment')
        if cm:
            self.emit('// ' + cm.replace('\r', '').replace('\n', ' / '), ind)
        self.emit(self.anim_node_desc(node), ind)
        gpath = (self.g.get('object_path') or '').split('.', 1)
        key = (gpath[1] + '.' if len(gpath) > 1 else '') + nname
        for prop, ptxt, kind in BINDINGS.get(key, []):
            self.emit('  .%s <= bound %s %s' % (prop, 'function' if kind == 'fn' else 'property', ptxt), ind)
        shown = set(node.get('pins_shown_for_properties') or [])
        for p in node['pins']:
            if p['dir'] != 'in':
                continue
            if p.get('type_object') in POSE_TYPES:
                continue
            if p.get('links') or p['name'] in shown:
                self.emit('  .%s = %s' % (clean(p['name']), self.pin_value(p)), ind)
        for p in node['pins']:
            if p['dir'] == 'in' and p.get('type_object') in POSE_TYPES:
                links = p.get('links') or []
                self.emit('  <%s>:' % p['name'], ind)
                for l in links:
                    self.render_pose(self.link_target(l)[0], ind + 1, visited)
                if not links:
                    self.emit('(unconnected)', ind + 1)

    def render_anim(self):
        roots = [n for n in self.g['nodes'] if n['class'] in (
            'AnimGraphNode_Root', 'AnimGraphNode_StateResult', 'AnimGraphNode_BlendStackResult',
            'AnimGraphNode_LinkedInputPose_Result')]
        visited = set()
        for r in roots:
            self.render_pose(r['name'], 0, visited)
        # disconnected anim nodes
        others = [n for n in self.g['nodes'] if n['class'].startswith('AnimGraphNode') and n['name'] not in visited]
        for o in others:
            # only print top-most of disconnected chains
            outp = [p for p in o['pins'] if p['dir'] == 'out' and p.get('type_object') in POSE_TYPES]
            if outp and any(p.get('links') for p in outp):
                continue
            self.emit('[not connected to root]', 0)
            self.render_pose(o['name'], 1, visited)
        # pure K2 helpers in anim graphs (transition rules etc.)
        tr = [n for n in self.g['nodes'] if n['class'] == 'AnimGraphNode_TransitionResult']
        for t in tr:
            self.emit('CAN_ENTER = %s' % self.in_expr(t, 'bCanEnterTransition'), 0)


def render_state_machine(bp, gobj, graphs_by_path, out):
    g = Graph(gobj, bp)
    path = gobj['object_path']
    nodes = gobj['nodes']
    out.append('  states:')
    for n in nodes:
        if n['class'] in ('AnimStateNode', 'AnimStateConduitNode', 'AnimStateAliasNode', 'AnimStateEntryNode'):
            props = {k: v for k, v in n.get('props', {}).items() if k not in ('NodeGuid',)}
            out.append('    %s %s "%s" %s' % (n['class'], n['name'], n.get('label', ''),
                                             json.dumps(props, ensure_ascii=False)))
            if n['class'] == 'AnimStateEntryNode':
                for p in n['pins']:
                    for l in p.get('links') or []:
                        out.append('      entry -> %s' % l.split('.')[0])
    out.append('  transitions:')
    lbl = {n['name']: (n.get('label') or n['name']) for n in nodes}
    for n in nodes:
        if n['class'] != 'AnimStateTransitionNode':
            continue
        src = dst = '?'
        for p in n['pins']:
            if p['dir'] == 'in' and p.get('links'):
                src = ','.join(l.split('.')[0] for l in p['links'])
            if p['dir'] == 'out' and p.get('links'):
                dst = ','.join(l.split('.')[0] for l in p['links'])
        props = {k: v for k, v in n.get('props', {}).items() if k not in ('NodeGuid', 'BoundGraph', 'CustomTransitionGraph')}
        out.append('    %s: %s -> %s %s' % (n['name'], src, dst, json.dumps(props, ensure_ascii=False)))
        sub = graphs_by_path.get(path + '.' + n['name'] + '.Transition')
        if sub is None:
            pref = path + '.' + n['name'] + '.'
            cands = [g for k, g in graphs_by_path.items() if k and k.startswith(pref)
                     and g.get('class') == 'AnimationTransitionGraph']
            sub = cands[0] if cands else None
        if sub:
            sg = Graph(sub, bp)
            for r in [x for x in sub['nodes'] if x['class'] == 'AnimGraphNode_TransitionResult']:
                out.append('        rule: %s' % sg.in_expr(r, 'bCanEnterTransition'))
    for n in nodes:
        if n['class'] in ('AnimStateNode', 'AnimStateConduitNode'):
            for p in n['pins']:
                pass


def is_intermediate(g):
    nm = g['name']
    op = g.get('object_path') or ''
    return (nm.endswith('_MERGED') or nm.startswith('ExecuteUbergraph') or 'ExecuteUbergraph' in op
            or nm.startswith('EvaluateGraphExposedInputs') or nm.startswith('AnimNotify_Transition'))


def render_bp(path):
    load_enums(path)
    bp = json.load(open(path, encoding='utf-8'))
    out = []
    out.append('### %s  (class %s, parent %s)' % (bp['path'], bp.get('class'), bp.get('parent_class')))
    if bp.get('target_skeleton'):
        out.append('target_skeleton: %s' % bp['target_skeleton'])
    if bp.get('interfaces'):
        out.append('interfaces: %s' % ', '.join(short_path(i) for i in bp['interfaces']))
    out.append('')
    out.append('## VARIABLES (name : type = CDO default) [category]')
    for v in bp.get('variables', []):
        t = v.get('type')
        if v.get('type_object'):
            t += '<%s>' % short_path(v['type_object'])
        if v.get('container'):
            t = '%s<%s>' % (v['container'], t)
        val = v.get('cdo_value', v.get('default_value'))
        if v.get('type_object') and isinstance(val, str) and val.startswith('NewEnumerator'):
            val = enum_tr(short_path(v['type_object']), val)
        out.append('  %s : %s = %s   [%s]' % (v['name'], t, val, v.get('category', '')))
    if bp.get('cdo_other_overrides'):
        out.append('')
        out.append('## CDO OTHER OVERRIDES')
        out.append(json.dumps(bp['cdo_other_overrides'], ensure_ascii=False, indent=1)[:20000])
    if bp.get('components_and_templates'):
        out.append('')
        out.append('## COMPONENTS / TEMPLATES')
        out.append(json.dumps(bp['components_and_templates'], ensure_ascii=False, indent=1)[:60000])
    if bp.get('cdo_default_subobjects'):
        out.append('')
        out.append('## CDO DEFAULT SUBOBJECTS')
        out.append(json.dumps(bp['cdo_default_subobjects'], ensure_ascii=False, indent=1)[:60000])
    graphs_by_path = {g.get('object_path'): g for g in bp.get('graphs', [])}
    GRAPHS_BY_PATH.clear()
    GRAPHS_BY_PATH.update(graphs_by_path)
    t3d_name = (bp.get('sources') or {}).get('t3d')
    t3d_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(path))), 't3d')
    load_bindings(os.path.join(t3d_dir, t3d_name) if t3d_name else None)
    if BINDINGS:
        out.append('')
        out.append('## ANIM NODE PROPERTY BINDINGS (from T3D)')
        for k in sorted(BINDINGS):
            out.append('  %s: %s' % (k, '; '.join('%s<=%s%s' % (a, b, '()' if c == 'fn' else '') for a, b, c in BINDINGS[k])))
    for gobj in bp.get('graphs', []):
        if is_intermediate(gobj):
            continue
        cls = gobj.get('class')
        out.append('')
        out.append('## GRAPH "%s" (%s) path=%s nodes=%d' % (gobj['name'], cls, gobj.get('object_path'),
                                                            gobj.get('node_count', 0)))
        comments = [n for n in gobj['nodes'] if n['class'] == 'EdGraphNode_Comment']
        for cmt in comments:
            txt = cmt.get('props', {}).get('NodeComment') or cmt.get('label')
            if txt:
                out.append('  /* comment box: %s */' % txt.replace('\r', '').replace('\n', ' / ')[:400])
        if cls == 'AnimationStateMachineGraph':
            render_state_machine(bp, gobj, graphs_by_path, out)
            continue
        if cls == 'AnimationTransitionGraph':
            continue  # rendered with its state machine
        g = Graph(gobj, bp)
        if cls in ('AnimationGraph', 'AnimationStateGraph', 'AnimationBlendStackGraph',
                   'AnimationCustomTransitionGraph', 'AnimationConduitGraph'):
            g.render_anim()
        # K2 parts (also inside anim graphs, e.g. none usually)
        if cls not in ('AnimationGraph', 'AnimationStateGraph', 'AnimationBlendStackGraph'):
            g.render_k2()
        out.extend('  ' + l for l in g.lines)
    return tr_text('\n'.join(out))


def main():
    if len(sys.argv) >= 2 and sys.argv[1] == '--all':
        src, dst = sys.argv[2], sys.argv[3]
        os.makedirs(dst, exist_ok=True)
        for f in sorted(os.listdir(src)):
            if not f.endswith('.json'):
                continue
            try:
                txt = render_bp(os.path.join(src, f))
            except Exception as e:  # keep going, report
                txt = 'RENDER ERROR: %r' % e
                print('ERROR', f, repr(e))
            with open(os.path.join(dst, f[:-5] + '.txt'), 'w', encoding='utf-8') as fh:
                fh.write(txt)
        return
    txt = render_bp(sys.argv[1])
    if len(sys.argv) > 2:
        with open(sys.argv[2], 'w', encoding='utf-8') as fh:
            fh.write(txt)
    else:
        sys.stdout.write(txt)


if __name__ == '__main__':
    main()
