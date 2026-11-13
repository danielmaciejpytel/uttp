"""Minimal, dependency-free parser for Unreal T3D text exports (UObjectExporterT3D output).

parse_t3d(text) -> root T3DObject (class, name, export_path, props, custom, children)
parse_value(str) -> python value for UE ExportText syntax:
  (A=1,B="x")        -> dict            (a,b,c)  -> list            ()  -> []
  /Script/M.Struct(A=1) (typed struct, e.g. FInstancedStruct) -> {'_type': '/Script/M.Struct', 'A': '1'}
  "text"             -> str (unescaped)  Class'/Path.Obj' -> str as written
  NSLOCTEXT("ns","key","src") / INVTEXT("x") / LOCGEN... -> {'_text': 'src', '_macro': 'NSLOCTEXT', '_args': [...]}
Scalars stay strings (T3D is text; numbers/bools are not converted) - use as_float/as_bool helpers.
Keys with array index (Name(3)=.. or Name[3]=..) are collected into lists.
Note: T3D is a DELTA export - properties equal to the archetype/class default are omitted.
"""
import re

_KEY_RE = re.compile(r'([A-Za-z_][\w\.]*)(?:\((\d+)\)|\[(\d+)\])?\s*=(?!=)')
_LINE_RE = re.compile(r'^([^=\s\"(][^=\"]*?)(?:\((\d+)\)|\[(\d+)\])?=(.*)$', re.S)  # keys may contain spaces (BP vars)
_ATTR_RE = re.compile(r'(\w+)=("(?:[^"\\]|\\.)*"|\S+)')
_MACROS = ('NSLOCTEXT', 'LOCTEXT', 'INVTEXT', 'LOCTABLE', 'LOCGEN_NUMBER', 'LOCGEN_PERCENT', 'LOCGEN_FORMAT_NAMED',
           'LOCGEN_FORMAT_ORDERED', 'LOCGEN_TOUPPER', 'LOCGEN_TOLOWER', 'LOCGEN_CURRENCY', 'LOCGEN_DATE_UTC',
           'LOCGEN_TIME_UTC', 'LOCGEN_DATETIME_UTC', 'LOCGEN_NUMBER_GROUPED', 'LOCGEN_NUMBER_UNGROUPED',
           'LOCGEN_NUMBER_CUSTOM', 'LOCGEN_PERCENT_GROUPED', 'LOCGEN_PERCENT_UNGROUPED', 'LOCGEN_PERCENT_CUSTOM')


class T3DObject:
    __slots__ = ('cls', 'name', 'export_path', 'archetype', 'props', 'custom', 'children', 'parent')

    def __init__(self, name, parent=None):
        self.cls = None
        self.name = name
        self.export_path = None
        self.archetype = None
        self.props = {}
        self.custom = []
        self.children = {}
        self.parent = parent

    @property
    def short_class(self):
        c = self.cls or ''
        c = c.rsplit('.', 1)[-1]
        return c

    def path(self):
        parts = []
        o = self
        while o is not None and o.name is not None:
            parts.append(o.name)
            o = o.parent
        return '.'.join(reversed(parts))

    def walk(self):
        yield self
        for c in self.children.values():
            yield from c.walk()

    def to_dict(self, recursive=True):
        d = {'name': self.name, 'class': self.cls, 'props': self.props}
        if self.custom:
            d['custom'] = [{'kind': k, 'value': v} for k, v in self.custom]
        if recursive and self.children:
            d['children'] = [c.to_dict() for c in self.children.values()]
        return d


def _ws(s, i):
    n = len(s)
    while i < n and s[i] in ' \t\r\n':
        i += 1
    return i


def _str(s, i):
    # s[i] == '"'
    i += 1
    out = []
    n = len(s)
    while i < n:
        c = s[i]
        if c == '\\' and i + 1 < n:
            nx = s[i + 1]
            out.append({'n': '\n', 't': '\t', 'r': '\r'}.get(nx, nx))
            i += 2
            continue
        if c == '"':
            return ''.join(out), i + 1
        out.append(c)
        i += 1
    return ''.join(out), i


def _match_paren(s, i):
    # s[i] == '(' -> index of the matching ')'
    depth = 0
    n = len(s)
    while i < n:
        c = s[i]
        if c == '"':
            _, i = _str(s, i)
            continue
        if c == '(':
            depth += 1
        elif c == ')':
            depth -= 1
            if depth == 0:
                return i
        i += 1
    return n - 1


def _set_key(d, key, idx, val):
    if idx is None:
        if key in d and isinstance(d[key], list) and isinstance(val, list):
            d[key] = val
        else:
            d[key] = val
        return
    idx = int(idx)
    cur = d.get(key)
    if not isinstance(cur, list):
        cur = [] if cur is None else [cur]
        d[key] = cur
    while len(cur) <= idx:
        cur.append(None)
    cur[idx] = val


def _paren(s, i):
    # s[i] == '('
    i += 1
    items, keyed = [], {}
    n = len(s)
    while i < n:
        i = _ws(s, i)
        if i >= n:
            break
        if s[i] == ')':
            i += 1
            break
        m = _KEY_RE.match(s, i)
        if m:
            v, i = _val(s, m.end())
            _set_key(keyed, m.group(1), m.group(2) or m.group(3), v)
        else:
            v, i = _val(s, i)
            items.append(v)
        i = _ws(s, i)
        if i < n and s[i] == ',':
            i += 1
            continue
        if i < n and s[i] == ')':
            i += 1
            break
        if i < n:
            i += 1  # tolerate garbage
    if keyed and items:
        keyed['_items'] = items
        return keyed, i
    if keyed:
        return keyed, i
    return items, i


def _val(s, i):
    i = _ws(s, i)
    n = len(s)
    if i >= n:
        return '', i
    c = s[i]
    if c == '(':
        return _paren(s, i)
    if c == '"':
        v, j = _str(s, i)
        return v, j
    buf = []
    while i < n:
        c = s[i]
        if c in ',)':
            break
        if c == "'":
            j = s.find("'", i + 1)
            if j < 0:
                j = n - 1
            buf.append(s[i:j + 1])
            i = j + 1
            continue
        if c == '"':
            v, i = _str(s, i)
            buf.append('"' + v + '"')
            continue
        if c == '(':
            tok = ''.join(buf).strip()
            if tok in _MACROS or tok.startswith('LOCGEN'):
                j = _match_paren(s, i)
                inner = s[i + 1:j]
                args = []
                k = 0
                while k < len(inner):
                    k = _ws(inner, k)
                    if k < len(inner) and inner[k] == '"':
                        a, k = _str(inner, k)
                        args.append(a)
                    else:
                        k2 = inner.find(',', k)
                        if k2 < 0:
                            args.append(inner[k:].strip())
                            break
                        args.append(inner[k:k2].strip())
                        k = k2
                    k = _ws(inner, k)
                    if k < len(inner) and inner[k] == ',':
                        k += 1
                return {'_macro': tok, '_args': args, '_text': args[-1] if args else ''}, j + 1
            inner, i = _paren(s, i)
            if isinstance(inner, dict):
                d = {'_type': tok}
                d.update(inner)
                return d, i
            return {'_type': tok, '_items': inner}, i
        buf.append(c)
        i += 1
    return ''.join(buf).strip(), i


def parse_value(s):
    v, _ = _val(s, 0)
    return v


def parse_t3d(text):
    if text.startswith('﻿'):
        text = text[1:]
    root = T3DObject(None)
    stack = [root]
    for raw in text.splitlines():
        line = raw.strip()
        if not line:
            continue
        if line.startswith('Begin Object'):
            attrs = {k: (v[1:-1] if v.startswith('"') and v.endswith('"') else v) for k, v in _ATTR_RE.findall(line[12:])}
            parent = stack[-1]
            name = attrs.get('Name', '?')
            obj = parent.children.get(name)
            if obj is None:
                obj = T3DObject(name, parent if parent is not root else None)
                parent.children[name] = obj
            if attrs.get('Class'):
                obj.cls = attrs['Class']
            if attrs.get('ExportPath'):
                obj.export_path = attrs['ExportPath']
            if attrs.get('Archetype'):
                obj.archetype = attrs['Archetype']
            stack.append(obj)
            continue
        if line.startswith('End Object'):
            if len(stack) > 1:
                stack.pop()
            continue
        cur = stack[-1]
        if line.startswith('CustomProperties '):
            rest = line[len('CustomProperties '):]
            kind, _, data = rest.partition(' ')
            cur.custom.append((kind, parse_value(data) if data.strip() else None))
            continue
        m = _LINE_RE.match(line)
        if m:
            name, idx = m.group(1), m.group(2) or m.group(3)
            raw_val = m.group(4)
            val = parse_value(raw_val) if raw_val != '' else ''
            _set_key(cur.props, name, idx, val)
    tops = list(root.children.values())
    if len(tops) == 1:
        tops[0].parent = None
        return tops[0]
    return root


def objref(v):
    """'/Script/Engine.AnimSequence'/GASPALS/A.A'' or "Class'Path'" -> '/GASPALS/A.A' ; None/'None' -> None."""
    if v is None:
        return None
    if isinstance(v, dict):
        return None
    s = str(v).strip().strip('"')
    if s in ('', 'None'):
        return None
    if "'" in s:
        a = s.find("'")
        b = s.rfind("'")
        if b > a:
            return s[a + 1:b]
    return s


def objclass(v):
    s = str(v or '').strip().strip('"')
    if "'" in s:
        return s[:s.find("'")]
    return None


def as_float(v, default=None):
    try:
        return float(v)
    except Exception:
        return default


def as_bool(v, default=None):
    if isinstance(v, bool):
        return v
    if v is None:
        return default
    s = str(v).strip().lower()
    if s in ('true', '1'):
        return True
    if s in ('false', '0'):
        return False
    return default


def text_of(v):
    if isinstance(v, dict) and '_text' in v:
        return v['_text']
    return v
