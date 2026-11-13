"""Pre-install gate: every redscript `native func` must agree with its RED4ext registration in the UTPP plugin.

A `native func X(...)` declared in plugin/UTPP/scripts/**/*.reds is bound at script init to the function the DLL
registers with CGlobalFunction::Create("X", ...) + AddParam(type, name, isOut, isOptional) + SetReturnType(type) +
rtti->RegisterFunction(f). A declaration without a registration, or one whose parameter count / types / out / opt /
return type disagree with the registration, can fail script init ("Failed to initialize scripts data!") or corrupt the
script stack at the first call.

The DLL side is read STATICALLY (no build, no game). Registration shapes understood (anything else is reported as
UNRESOLVED and fails the gate - it never passes silently):
  * `auto* f = CGlobalFunction::Create("X", "X", &fn);` or `f = Wrapper("X", &fn);` (a wrapper is any function
    `CGlobalFunction* W(...)` defined in the same file, e.g. Make() in Locomotion.cpp / LootHead.cpp)
  * `f->AddParam("T", name[, isOut[, isOptional]])`, `f->SetReturnType("T")`, `rtti->RegisterFunction(f)`
  * `for (const char* p : {a, b, c}) f->AddParam("Float", p);` (also with a braced body): the call repeats once per element
  * struct-array tables: `const Row rows[] = {{"X", &fn}, ...}; for (const Row& b : rows) { Create(b.name, b.name, b.fn);
    f->AddParam(...); ...; RegisterFunction(f); }` - one registration per table row, sharing the loop body's signature
Type names are compared canonically on both sides: redscript `ref<X>`/`wref<X>`/`array<T>`/`script_ref<T>` vs the RTTI
spellings `handle:X`/`whandle:X`/`array:T`/`script_ref:T`, and script aliases (ItemObject, IComponent, ...) vs native
class names (gameItemObject, entIComponent) via the NAME/ALIAS pairs of the RED4ext.SDK generated headers.

usage: python -I tools/install/check_reds_natives.py [repo_root] [--scripts DIR] [--src DIR] [--cmake FILE]
                                                      [--sdk-dir DIR] [-v]
Exit 1 on a mismatch, a declared native with no registration, an unresolved registration pattern or a duplicate
declaration/registration with a differing signature.
"""
import argparse
import bisect
import os
import pathlib
import re
import sys

MANUAL_ALIASES = {  # script name -> native name; in case no SDK checkout is reachable (the generated headers win)
    "ItemObject": "gameItemObject",
    "IComponent": "entIComponent",
    "ItemID": "gameItemID",
    "CallbackSystemHandler": "gameCallbackSystemHandler",
    "CallbackSystemEvent": "gameCallbackSystemEvent",
}


# ---------------------------------------------------------------- generic text helpers
def strip_comments(t):
    """Blank // and /* */ comments (newlines kept, so offsets and line numbers are unchanged); keep string literals."""
    out = []
    i, n = 0, len(t)
    while i < n:
        c = t[i]
        if c == "R" and t.startswith('R"', i) and (i == 0 or not (t[i - 1].isalnum() or t[i - 1] == "_")):
            m = re.compile(r'R"([^()\\\s]{0,16})\(').match(t, i)
            if m:
                end = t.find(")" + m.group(1) + '"', m.end())
                end = n if end < 0 else end + len(m.group(1)) + 2
                out.append(t[i:end])
                i = end
                continue
        if c == '"':
            j = i + 1
            while j < n and t[j] != '"' and t[j] != "\n":
                j += 2 if t[j] == "\\" else 1
            out.append(t[i : j + 1])
            i = j + 1
        elif c == "'":
            m = re.compile(r"'(?:\\.[^']*|[^\\'])'").match(t, i)
            if m:
                out.append(m.group(0))
                i = m.end()
            else:
                out.append(c)
                i += 1
        elif t.startswith("//", i):
            j = t.find("\n", i)
            j = n if j < 0 else j
            out.append(" " * (j - i))
            i = j
        elif t.startswith("/*", i):
            j = t.find("*/", i + 2)
            j = n if j < 0 else j + 2
            out.append(re.sub(r"[^\n]", " ", t[i:j]))
            i = j
        else:
            out.append(c)
            i += 1
    return "".join(out)


def match_close(t, i):
    """Index of the bracket closing the one at t[i] ( ( [ { ), skipping string literals; -1 when unbalanced."""
    pairs = {"(": ")", "[": "]", "{": "}"}
    depth, n = 0, len(t)
    j = i
    while j < n:
        c = t[j]
        if c == '"':
            j += 1
            while j < n and t[j] != '"':
                j += 2 if t[j] == "\\" else 1
        elif c in pairs:
            depth += 1
        elif c in ")]}":
            depth -= 1
            if depth == 0:
                return j
        j += 1
    return -1


def split_spans(t, a, b, angle=False):
    """Top-level comma separated (start, end) spans of t[a:b] (string literals and brackets respected)."""
    spans, depth, start, j = [], 0, a, a
    while j < b:
        c = t[j]
        if c == '"':
            j += 1
            while j < b and t[j] != '"':
                j += 2 if t[j] == "\\" else 1
        elif c in "([{" or (angle and c == "<"):
            depth += 1
        elif c in ")]}" or (angle and c == ">"):
            depth -= 1
        elif c == "," and depth == 0:
            spans.append((start, j))
            start = j + 1
        j += 1
    spans.append((start, b))
    return [(s, e) for s, e in spans if t[s:e].strip()]


def stmt_end(t, i):
    """End offset (after the `;`) of the statement starting at i (string literals respected)."""
    n, j = len(t), i
    while j < n:
        if t[j] == '"':
            j += 1
            while j < n and t[j] != '"':
                j += 2 if t[j] == "\\" else 1
        elif t[j] in "({[":
            j = match_close(t, j)
            if j < 0:
                return n
        elif t[j] == ";":
            return j + 1
        j += 1
    return n


class Src:
    def __init__(self, path, root):
        self.path = path
        self.rel = pathlib.Path(os.path.relpath(path, root)).as_posix()
        self.raw = path.read_text(encoding="utf-8", errors="replace")
        self.t = strip_comments(self.raw)
        self.nl = [m.start() for m in re.finditer("\n", self.t)]

    def line(self, pos):
        return bisect.bisect_left(self.nl, pos) + 1

    def loc(self, pos):
        return f"{self.rel}:{self.line(pos)}"


# ---------------------------------------------------------------- type canonicalization
class Types:
    def __init__(self, sdk_dir):
        self.alias = dict(MANUAL_ALIASES)
        self.sdk_dir = sdk_dir
        self.sdk_pairs = 0
        if sdk_dir:
            gen = sdk_dir / "include/RED4ext/Scripting/Natives/Generated"
            for p in gen.rglob("*.hpp"):
                txt = p.read_text(encoding="utf-8", errors="replace")
                n = re.search(r'NAME = "([^"]+)"', txt)
                a = re.search(r'ALIAS = "([^"]+)"', txt)
                if n and a and a.group(1) != "None":
                    self.alias[a.group(1)] = n.group(1)
                    self.sdk_pairs += 1

    def canon(self, t):
        t = re.sub(r"\s+", "", t)
        m = re.fullmatch(r"(ref|wref|handle|whandle|array|script_ref)[<:](.+?)>?", t)
        if m and (t[len(m.group(1))] == ":" or t.endswith(">")):
            kind = {"ref": "handle", "wref": "whandle"}.get(m.group(1), m.group(1))
            return kind + ":" + self.canon(m.group(2))
        m = re.fullmatch(r"\[(.+);(\d+)\]", t)  # static array [T;N]
        if m:
            return "static:" + m.group(2) + ":" + self.canon(m.group(1))
        return self.alias.get(t, t)


def find_sdk(root, explicit):
    cands = []
    if explicit:
        cands.append(pathlib.Path(explicit))
    if os.environ.get("UTPP_RED4EXT_SDK_DIR"):
        cands.append(pathlib.Path(os.environ["UTPP_RED4EXT_SDK_DIR"]))
    for base in [root, *root.parents]:  # the plugin's own default is <repo>/Tools/repos/RED4ext.SDK
        cands.append(base / "Tools/repos/RED4ext.SDK")
        cands.append(base / "Source/compat-2.5/codeware/vendor/RED4ext.SDK")
    for c in cands:
        if (c / "include/RED4ext/Scripting/Natives/Generated").is_dir():
            return c
    return None


# ---------------------------------------------------------------- redscript side
class Decl:
    def __init__(self, name, params, ret, loc, in_class):
        self.name, self.params, self.ret, self.loc, self.in_class = name, params, ret, loc, in_class


REDS_NATIVE = re.compile(r"\bnative\b\s*(?:(?:static|final|public|private|protected)\s+)*func\s+(\w+)\s*\(")
REDS_TOKEN = re.compile(
    r'(?P<str>"(?:[^"\\\n]|\\.)*")|(?P<cls>\b(?:class|struct|enum)\b[^{};]*\{)|(?P<ob>\{)|(?P<cb>\})|(?P<nat>\bnative\b)'
)


def parse_reds(src, types, unresolved):
    t = src.t
    decls, stack = [], []
    for m in REDS_TOKEN.finditer(t):
        if m.group("str"):
            continue
        if m.group("cls"):
            stack.append("class")
        elif m.group("ob"):
            stack.append("block")
        elif m.group("cb"):
            if stack:
                stack.pop()
        else:
            nm = REDS_NATIVE.match(t, m.start())
            if not nm:
                continue
            op = nm.end() - 1
            cl = match_close(t, op)
            if cl < 0:
                unresolved.append(f"{src.loc(m.start())} native func {nm.group(1)}: unbalanced parameter list")
                continue
            params = []
            for s, e in split_spans(t, op + 1, cl, angle=True):
                txt = re.sub(r"\s+", " ", t[s:e].strip())
                mods = set()
                while True:
                    mm = re.match(r"(out|opt|const)\s+", txt)
                    if not mm:
                        break
                    mods.add(mm.group(1))
                    txt = txt[mm.end() :]
                pm = re.match(r"(\w+)\s*:\s*([^=]+?)\s*(?:=.*)?$", txt)
                if not pm:
                    unresolved.append(f"{src.loc(m.start())} native func {nm.group(1)}: cannot parse parameter '{txt}'")
                    continue
                params.append((types.canon(pm.group(2)), "out" in mods, "opt" in mods))
            rm = re.compile(r"\s*->\s*([^{;\n]+)").match(t, cl + 1)
            ret = types.canon(rm.group(1)) if rm else "Void"
            decls.append(Decl(nm.group(1), params, ret, src.loc(nm.start(1)), "class" in stack))
    return decls


# ---------------------------------------------------------------- DLL side
class Fn:
    """One CGlobalFunction::Create() object; a table loop makes one object that carries several names."""

    def __init__(self, names, handler, loc):
        self.names = names  # [(name, loc)]
        self.handler = handler
        self.loc = loc
        self.params, self.ret, self.registered, self.ret_loc = [], "Void", False, None
        self.short_differs = None
        self.alias_spelled = []  # (loc, raw spelling, canonical) where the DLL spells a script alias


def parse_tables(src):
    structs = {}
    for m in re.finditer(r"\bstruct\s+(\w+)\s*\{", src.t):
        cl = match_close(src.t, m.end() - 1)
        fields = []
        for part in src.t[m.end() : cl].split(";"):
            part = re.sub(r"=.*", "", part).strip()
            fm = re.search(r"(\w+)\s*(?:\[[^\]]*\])?$", part)
            if fm:
                fields.append(fm.group(1))
        structs[m.group(1)] = fields
    tables = {}
    for m in re.finditer(r"\b(\w+)\s+(\w+)\s*\[\s*\]\s*=\s*\{", src.t):
        cl = match_close(src.t, m.end() - 1)
        rows = []
        for s, e in split_spans(src.t, m.end(), cl):
            if src.t[s:e].strip().startswith("{"):
                o = src.t.index("{", s)
                rows.append((o, split_spans(src.t, o + 1, match_close(src.t, o))))
            else:
                rows.append((s, [(s, e)]))
        tables[m.group(2)] = (m.group(1), rows)
    return structs, tables


def parse_cpp(src, objs, unresolved, types):
    t = src.t
    wrappers = set(re.findall(r"\bCGlobalFunction\s*\*\s*(\w+)\s*\(", t))
    excluded = []  # wrapper definitions: the Create() inside them is the template, not a registration
    for m in re.finditer(r"\bCGlobalFunction\s*\*\s*(\w+)\s*\(", t):
        cl = match_close(t, m.end() - 1)
        ob = re.compile(r"\s*\{").match(t, cl + 1) if cl > 0 else None
        if ob:
            excluded.append((m.start(), match_close(t, ob.end() - 1)))
    structs, tables = parse_tables(src)
    wr = ("|" + "|".join(re.escape(w) for w in sorted(wrappers))) if wrappers else ""
    ev = re.compile(
        r"(?P<cls>\bCClass(?:Static)?Function\s*::\s*Create\b)"
        r"|(?P<create>(?:\b(?P<var>\w+)\s*=\s*)?(?:\b(?P<cfn>(?:\w+::)*CGlobalFunction::Create)"
        + (r"|(?<![\w:.>])(?P<wfn>" + wr[1:] + r")" if wrappers else r"|(?P<wfn>(?!))")
        + r")\s*\()"
        r"|(?P<add>\b(?P<av>\w+)\s*->\s*AddParam\s*\()"
        r"|(?P<ret>\b(?P<rv>\w+)\s*->\s*SetReturnType\s*\()"
        r"|(?P<reg>\bRegisterFunction\s*\()"
        r"|(?P<for>\bfor\s*\()"
    )
    bind, scopes = {}, []  # scopes: dicts(end, kind, mult|table, var)

    def mult_of(pos, what):
        while scopes and scopes[-1]["end"] <= pos:
            scopes.pop()
        mult = 1
        for sc in scopes:
            if sc["kind"] == "unknown":
                unresolved.append(f"{src.loc(pos)} {what} inside an unrecognized loop")
                return None
            if sc["kind"] == "list":
                mult *= sc["mult"]
        return mult

    def args_of(open_paren):
        cl = match_close(t, open_paren)
        return cl, split_spans(t, open_paren + 1, cl)

    def lit(s, e):
        m = re.fullmatch(r'\s*"((?:[^"\\]|\\.)*)"\s*', t[s:e])
        return m.group(1) if m else None

    def boolean(s, e):
        v = t[s:e].strip()
        return {"true": True, "false": False}.get(v)

    for m in ev.finditer(t):
        pos = m.start()
        if any(a <= pos < b for a, b in excluded):
            continue
        what = m.lastgroup
        if m.group("cls"):
            unresolved.append(f"{src.loc(pos)} class-function registration (CClass*Function::Create) is not covered by this gate")
        elif m.group("for"):
            cl = match_close(t, m.end() - 1)
            body = re.compile(r"\s*").match(t, cl + 1).end()
            end = match_close(t, body) + 1 if t[body : body + 1] == "{" else stmt_end(t, body)
            hdr = t[m.end() : cl]
            rm = re.fullmatch(r"\s*(?:const\s+)?([\w:]+(?:<[^>]*>)?)\s*[&*]*\s*(\w+)\s*:\s*(.+?)\s*", hdr, re.S)
            sc = {"end": end, "kind": "unknown"}
            if rm:
                srcx = rm.group(3)
                bm = re.fullmatch(r"\{(.*)\}", srcx, re.S)
                if bm:
                    sc.update(kind="list", mult=len(split_spans(bm.group(1), 0, len(bm.group(1)))))
                elif srcx in tables:
                    sc.update(kind="table", table=srcx, var=rm.group(2), type=rm.group(1))
            while scopes and scopes[-1]["end"] <= pos:
                scopes.pop()
            scopes.append(sc)
        elif m.group("create"):
            if mult_of(pos, "Create()") is None:
                continue
            if any(s["kind"] == "list" for s in scopes):
                unresolved.append(f"{src.loc(pos)} Create() inside a repeated loop")
                continue
            cl, args = args_of(m.end() - 1)
            wrapper = m.group("wfn") is not None
            if not args or not m.group("var"):
                unresolved.append(f"{src.loc(pos)} Create() result not bound to a variable (or no arguments)")
                continue
            handler = t[args[1 if wrapper else 2][0] : args[1 if wrapper else 2][1]].strip() if len(args) > (1 if wrapper else 2) else "?"
            name0 = lit(*args[0])
            fn = None
            if name0 is not None:
                fn = Fn([(name0, src.loc(pos))], handler, src.loc(pos))
                if not wrapper and len(args) > 1 and lit(*args[1]) != name0:
                    fn.short_differs = (name0, t[args[1][0] : args[1][1]].strip())
            else:
                tm = re.fullmatch(r"\s*(\w+)\.(\w+)\s*", t[args[0][0] : args[0][1]])
                tsc = next((s for s in scopes if s["kind"] == "table" and tm and s["var"] == tm.group(1)), None)
                if tsc:
                    fields = structs.get(tsc["type"].split("::")[-1], [])
                    if tm.group(2) not in fields:
                        unresolved.append(f"{src.loc(pos)} table field '{tm.group(2)}' not found in struct {tsc['type']}")
                        continue
                    idx = fields.index(tm.group(2))
                    names = []
                    for rpos, cells in tables[tsc["table"]][1]:
                        nm = lit(*cells[idx]) if idx < len(cells) else None
                        if nm is None:
                            unresolved.append(f"{src.loc(rpos)} table row has no string literal in field '{tm.group(2)}'")
                        else:
                            names.append((nm, src.loc(rpos)))
                    fn = Fn(names, "<table>", src.loc(pos))
                else:
                    unresolved.append(f"{src.loc(pos)} Create() name is not a string literal or a table field: '{t[args[0][0]:args[0][1]].strip()}'")
                    continue
            bind[m.group("var")] = fn
            objs.append(fn)
        elif m.group("add") or m.group("ret"):
            var = m.group("av") or m.group("rv")
            fn = bind.get(var)
            if fn is None:
                unresolved.append(f"{src.loc(pos)} {'AddParam' if m.group('add') else 'SetReturnType'} on unknown variable '{var}'")
                continue
            mult = mult_of(pos, "AddParam" if m.group("add") else "SetReturnType")
            if mult is None:
                continue
            cl, args = args_of(m.end() - 1)
            ty = lit(*args[0]) if args else None
            if ty is None:
                unresolved.append(f"{src.loc(pos)} type argument is not a string literal")
                continue
            if types.canon(ty) != re.sub(r"\s+", "", ty):
                fn.alias_spelled.append((src.loc(pos), ty, types.canon(ty)))
            if m.group("ret"):
                fn.ret, fn.ret_loc = types.canon(ty), src.loc(pos)
            else:
                out = boolean(*args[2]) if len(args) > 2 else False
                opt = boolean(*args[3]) if len(args) > 3 else False
                if out is None or opt is None:
                    unresolved.append(f"{src.loc(pos)} isOut/isOptional argument is not true/false")
                    continue
                fn.params += [(types.canon(ty), out, opt)] * mult
        elif m.group("reg"):
            cl, args = args_of(m.end() - 1)
            var = t[args[0][0] : args[0][1]].strip() if len(args) == 1 else ""
            if var not in bind:
                if re.match(r"^\s*(\w+)\s*\)", t[m.end() :]) or not var:
                    unresolved.append(f"{src.loc(pos)} RegisterFunction() argument '{var}' is not a tracked Create() variable")
                continue
            if mult_of(pos, "RegisterFunction") is None:
                continue
            bind[var].registered = True
    return bool(objs)


def cmake_sources(cmake_path):
    """Set of `src/...` paths named in add_library(UTPP ...) / target_sources(UTPP ...), or None when unavailable."""
    if not cmake_path.is_file():
        return None
    t = strip_comments_cmake(cmake_path.read_text(encoding="utf-8", errors="replace"))
    found, any_stmt = set(), False
    for m in re.finditer(r"\b(?:add_library|target_sources)\s*\(\s*UTPP\b", t):
        cl = match_close(t, t.index("(", m.start()))
        any_stmt = True
        found.update(re.findall(r"[\w./-]+\.(?:cpp|cc|cxx)", t[m.end() : cl]))
    return found if any_stmt else None


def strip_comments_cmake(t):
    return re.sub(r"(?m)#.*$", "", t)


# ---------------------------------------------------------------- driver
def sig(params, ret):
    ps = ",".join(("out " if o else "") + ("opt " if p else "") + ty for ty, o, p in params)
    return f"[{ps}]->{ret}"


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("root", nargs="?", default=str(pathlib.Path(__file__).resolve().parents[2]))
    ap.add_argument("--scripts")
    ap.add_argument("--src")
    ap.add_argument("--cmake")
    ap.add_argument("--sdk-dir")
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()
    root = pathlib.Path(a.root).resolve()
    scripts = pathlib.Path(a.scripts) if a.scripts else root / "plugin/UTPP/scripts"
    srcdir = pathlib.Path(a.src) if a.src else root / "plugin/UTPP/src"
    cmake = pathlib.Path(a.cmake) if a.cmake else root / "plugin/UTPP/CMakeLists.txt"

    sdk = find_sdk(root, a.sdk_dir)
    types = Types(sdk)
    print(f"alias source: {sdk if sdk else 'NONE (built-in table only)'} ({types.sdk_pairs} NAME/ALIAS pairs)")

    unresolved, warns = [], []
    decls = []
    for p in sorted(scripts.rglob("*.reds")):
        decls += parse_reds(Src(p, root), types, unresolved)

    compiled = cmake_sources(cmake)
    if compiled is None:
        warns.append(f"{cmake}: no add_library/target_sources(UTPP ...) found; compiled-file check skipped")
    regs, stray = {}, []  # name -> [(Fn, loc, file_rel)]
    for p in sorted(srcdir.rglob("*.cpp")):
        s = Src(p, root)
        objs = []
        parse_cpp(s, objs, unresolved, types)
        rel_in_plugin = pathlib.Path(os.path.relpath(p, cmake.parent)).as_posix() if cmake.parent in p.parents else s.rel
        for fn in objs:
            if fn.short_differs:
                warns.append(f"{fn.loc} {fn.names[0][0]}: Create() full name != short name ({fn.short_differs[1]})")
            for aloc, raw, can in fn.alias_spelled:
                warns.append(f"{aloc} {fn.names[0][0]}: DLL spells the script alias '{raw}' (native name '{can}'); compared as equal, "
                             "the engine must resolve the alias at AddParam time")
            if not fn.registered:
                stray.append(fn)
                continue
            for name, loc in fn.names:
                if compiled is not None and rel_in_plugin not in compiled:
                    stray.append(fn)
                    warns.append(f"{loc} {name}: registered in {rel_in_plugin}, which is NOT a source of the UTPP target")
                    continue
                regs.setdefault(name, []).append((fn, loc))

    mism, unreg, dups = [], [], []
    seen = {}
    matched = set()
    for d in decls:
        seen.setdefault(d.name, []).append(d)
        if d.in_class:
            unresolved.append(f"{d.loc} class-scope native '{d.name}' is not covered by this gate (global natives only)")
            continue
        rs = regs.get(d.name)
        if not rs:
            created = [f for f in stray if any(n == d.name for n, _ in f.names)]
            why = "created but never passed to RegisterFunction() / not compiled" if created else "no registration found"
            unreg.append(f"{d.loc} {d.name} reds={sig(d.params, d.ret)} dll=NONE ({why})")
            continue
        for fn, loc in rs:
            if sig(fn.params, fn.ret) != sig(d.params, d.ret):
                mism.append(f"{d.loc} {d.name} reds={sig(d.params, d.ret)} dll={sig(fn.params, fn.ret)} ({loc})")
            else:
                matched.add(d.name)
        # alias spelled on the DLL side: report (the canonical compare above already passed)
    for name, ds in seen.items():
        if len(ds) > 1:
            same = len({sig(d.params, d.ret) for d in ds}) == 1
            dups.append(f"{', '.join(d.loc for d in ds)} {name} declared {len(ds)}x ({'same' if same else 'DIFFERENT'} signature)")
            if not same:
                mism.append(f"{ds[0].loc} {name} reds-declarations differ: " + " vs ".join(f"{d.loc}={sig(d.params, d.ret)}" for d in ds))
    for name, rs in regs.items():
        if len(rs) > 1:
            same = len({sig(f.params, f.ret) for f, _ in rs}) == 1
            hs = {f.handler for f, _ in rs}
            dups.append(f"{', '.join(l for _, l in rs)} {name} registered {len(rs)}x ({'same' if same else 'DIFFERENT'} signature, "
                        f"{'same' if len(hs) == 1 else 'different'} handler)")
            if not same:
                mism.append(f"{rs[0][1]} {name} registrations differ: " + " vs ".join(f"{l}={sig(f.params, f.ret)}" for f, l in rs))
    undeclared = sorted(n for n in regs if n not in seen)

    for line in mism:
        print("MISMATCH", line)
    for line in unreg:
        print("UNREGISTERED", line)
    for line in unresolved:
        print("UNRESOLVED", line)
    for line in dups:
        print("DUP", line)
    for line in warns:
        print("WARN", line)
    if a.verbose:
        for n in undeclared:
            print("INFO registered without declaration:", n, sig(regs[n][0][0].params, regs[n][0][0].ret), regs[n][0][1])
    print(f"totals: declared {len(decls)} ({len(seen)} names), registered {len(regs)} names, matched {len(matched)}, "
          f"registered without declaration {len(undeclared)}; created-not-registered {len(stray)}")
    bad_dup = any("DIFFERENT" in d for d in dups)
    print(f"unresolved {len(unresolved)}, duplicates {len(dups)}, warnings {len(warns)}")
    fail = bool(mism or unreg or unresolved or bad_dup)
    print(f"natives gate: {'FAIL' if fail else 'ok'} ({len(mism)} mismatches, {len(unreg)} unregistered)")
    sys.exit(1 if fail else 0)


if __name__ == "__main__":
    main()
