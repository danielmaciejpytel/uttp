#!/usr/bin/env python3
"""Reference evaluator for source pack Chooser tables.

Faithful Python port of the UE 5.8 Chooser plugin runtime evaluation, applied to the tables extracted
into Extracted\\sourcepack\\sourcepack\\choosers.json (root CHT_PoseSearchDatabases and every chooser it reaches:
CHT_PoseSearchDatabases_Dense / _Sparse via EvaluateChooser and their embedded NestedChooser tables).

The script is data driven: it contains no Neutral thresholds, database names or row logic. Tables, enum
definitions and bound-property types are loaded at run time from the json paths given on the command
line; the default root chooser name (when --root is not given) is chooser.root of
Research\\inventory\\sourcepack-names.json (Neutral names stay outside the git repo). Everything it writes
(vectors, dumps) is source-pack-derived data and stays outside the Source\\ git repo.

UE semantics reproduced (C = Engine\\Plugins\\Chooser\\Source\\Chooser):
  * UChooserTable::EvaluateChooser, C\\Private\\Chooser.cpp:592-825
      - rows start as every non-disabled row, in row order (633-639); disabled rows are also stripped
        at cook time (271-300), so both builds agree;
      - every enabled column with HasFilters() narrows the list, columns in order (673-701); disabled
        columns are skipped (684-687); Output* columns have HasFilters()==false (Output*Column.h);
      - sorting happens only when a column has costs (705-708); Float/Bool/Enum columns have none
        (IChooserColumn.h:50), unsupported cost columns raise here;
      - each surviving row calls Result.ChooseMulti(callback) (713-761); a row "succeeds" when the
        status is not Failed; Stop ends the whole evaluation;
      - FallbackResult is evaluated only when no row succeeded (763-797); the table returns
        Continue if a row succeeded, otherwise Failed, even when the fallback produced results (824).
  * Result types: FAssetChooser -> FObjectChooserBase::ChooseMulti fallback (IObjectChooser.h:155-163:
    null asset = Failed, else callback); FEvaluateChooser / FNestedChooser -> evaluate the referenced
    table (Chooser.cpp:928-931, 978-981); a null table is Failed (597-600).
  * Modes: AllResults = UChooserFunctionLibrary::EvaluateObjectChooserBaseMulti, callback returns
    Continue (ChooserFunctionLibrary.cpp:330-377); FirstResult = EvaluateObjectChooserBase, callback
    returns Stop (270-309). The K2 node maps its Mode to these (ChooserUncooked\\Private\\
    EvaluateChooserNode.cpp:646-667). source pack calls CHT_PoseSearchDatabases with mode=AllResults
    (Extracted\\sourcepack\\sourcepack\\pseudo\\GASPALS__Blueprints__ABP_SandboxCharacter.txt:1790), hence the default.
  * FFloatRangeColumn::Filter, C\\Private\\FloatRangeColumn.cpp:55-114: closed range
    (bNoMin || v >= Min) && (bNoMax || v <= Max); wrap mode uses FMath::Wrap and the reversed-range
    OR test (70-91). Row Min/Max are C++ float (FloatRangeColumn.h:47-51), the input is read as
    double, so bounds are rounded to float32 and compared in double. Input precision follows the bound
    property type (ChooserPropertyAccess.cpp:245-293: float, double or int32 read as double).
  * FBoolColumn::Filter, C\\Private\\BoolColumn.cpp:29-61: MatchAny or value == (cell == MatchTrue).
  * FEnumColumn::Filter / FChooserEnumRowData::Evaluate, C\\Private\\EnumColumn.cpp:71-121:
    MatchEqual / MatchNotEqual / MatchAny on the uint8 value; in the editor the cell value is
    re-resolved from ValueName (PostLoad, 39-67) - done here when enum definitions are available.
  * A row index beyond a column's RowValues array is filtered out (IsValidIndex checks); an input that
    cannot be read makes the column pass every row through ("passthrough fallback", e.g.
    FloatRangeColumn.cpp:109-113). Here a missing input is an error unless its value is given as null
    (then the passthrough is emulated).

Usage (paths are examples):
  python motion_chooser_ref.py --choosers Extracted\\sourcepack\\sourcepack\\choosers.json eval --set MovementMode=OnGround ...
  python motion_chooser_ref.py --choosers Extracted\\sourcepack\\sourcepack\\choosers.json grid --out Research\\inventory\\chooser_vectors.json
  python motion_chooser_ref.py --choosers Extracted\\sourcepack\\sourcepack\\choosers.json rules
"""
import argparse
import hashlib
import json
import math
import os
import re
import struct
import sys

CONTINUE = "Continue"
STOP = "Stop"
FAILED = "Failed"

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
NAMES_JSON = os.path.join(PROJECT_ROOT, "Research", "inventory", "sourcepack-names.json")

# Member types of engine (C++) structs that a binding chain can pass through. These are UE engine facts,
# not source pack data: UE 5.8 Engine\Plugins\Animation\PoseSearch\Source\Runtime\Public\PoseSearch\
# PoseSearchTrajectoryLibrary.h:94-103 declares "float TimeToLand" and "float LandSpeed".
ENGINE_STRUCT_MEMBER_TYPES = {
    "/Script/PoseSearch.PoseSearchTrajectory_WorldCollisionResults": {"TimeToLand": "float", "LandSpeed": "float"},
}

FLOAT_TYPES = ("float", "double", "int")


class ChooserRefError(Exception):
    pass


class MissingInput(ChooserRefError):
    def __init__(self, key):
        super().__init__("missing chooser input: %s" % key)
        self.key = key


def default_root(names_json=None):
    """Root chooser name = chooser.root of the Neutral names data file (kept outside the git repo)."""
    p = names_json or NAMES_JSON
    if not os.path.isfile(p):
        raise ChooserRefError("no root chooser given and the names file %s is missing (pass --root)" % p)
    with open(p, encoding="utf-8") as f:
        return json.load(f)["chooser"]["root"]


# ----------------------------------------------------------------------------- numeric helpers

def f32(x):
    """Round a Python float (double) to IEEE float32, returned as a double (C++ float storage)."""
    return struct.unpack("<f", struct.pack("<f", float(x)))[0]


def f32_next(x, toward):
    """nextafter for float32."""
    x = f32(x)
    if x == toward or math.isnan(x):
        return x
    if x == 0.0:
        bits = 0x00000001 if toward > 0 else 0x80000001
    else:
        bits = struct.unpack("<I", struct.pack("<f", x))[0]
        if (x > 0) == (toward > x):
            bits += 1
        else:
            bits -= 1
    return struct.unpack("<f", struct.pack("<I", bits))[0]


def ue_wrap(x, mn, mx):
    """FMath::Wrap for floating point (Core\\Public\\Math\\UnrealMathUtility.h:610-634, Modulo = Fmod 671-681)."""
    size = mx - mn
    if size == 0:
        return mx
    end = x
    if end < mn:
        mod = math.fmod(mn - end, size)
        end = (mx - mod) if mod != 0 else mn
    elif end > mx:
        mod = math.fmod(end - mx, size)
        end = (mn + mod) if mod != 0 else mx
    return end


def _bool_str(s, default=False):
    if s is None:
        return default
    if isinstance(s, bool):
        return s
    return str(s).strip().lower() == "true"


def object_path(ref):
    """'/Script/Chooser.ChooserTable'/A/B.C'' or 'Class'path'' or plain path -> plain object path."""
    if ref is None:
        return None
    ref = str(ref).strip()
    m = re.search(r"'([^']*)'\s*$", ref)
    if m:
        return m.group(1)
    return ref


def short_name(path):
    if path is None:
        return None
    p = object_path(path)
    tail = p.rsplit("/", 1)[-1]
    return tail.split(".", 1)[-1] if "." in tail else tail


def norm_key(s):
    return re.sub(r"[\s_]", "", str(s)).lower()


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


# ----------------------------------------------------------------------------- enums

class EnumDef:
    def __init__(self, path, name, entries):
        self.path = path
        self.name = name
        # entries: list of (value, internal_name, display_name)
        self.entries = [e for e in entries if not str(e[1]).endswith("_MAX")]
        self.by_internal = {e[1]: e[0] for e in self.entries}

    def value_of(self, token):
        if isinstance(token, bool):
            raise ChooserRefError("enum %s: bool is not an enum value" % self.name)
        if isinstance(token, int):
            if not 0 <= token <= 255:
                raise ChooserRefError("enum %s: value %r outside uint8" % (self.name, token))
            return token
        if isinstance(token, float) and token.is_integer():
            return self.value_of(int(token))
        s = str(token).strip()
        if re.fullmatch(r"-?\d+", s):
            return self.value_of(int(s))
        m = re.fullmatch(r"(?:[\w]+::)?([\w]+)(?:\s*\((.*)\))?", s)
        cand = [s]
        if m:
            cand += [m.group(1)] + ([m.group(2)] if m.group(2) else [])
        for c in cand:
            if c in self.by_internal:
                return self.by_internal[c]
        for c in cand:
            for v, internal, disp in self.entries:
                if disp is not None and norm_key(disp) == norm_key(c):
                    return v
        raise ChooserRefError("enum %s: unknown value %r (known: %s)" % (
            self.name, token, ", ".join("%s=%d" % (d or i, v) for v, i, d in self.entries)))

    def display(self, value):
        for v, internal, disp in self.entries:
            if v == value:
                return disp or internal
        return int(value)


def _split_value_name(vn):
    """'E_X::NewEnumerator4 (OnGround)' -> ('E_X', 'NewEnumerator4', 'OnGround')."""
    if not vn:
        return None, None, None
    m = re.fullmatch(r"\s*(?:([\w]+)::)?([\w]+)(?:\s*\((.*)\))?\s*", vn)
    if not m:
        return None, vn, None
    return m.group(1), m.group(2), m.group(3)


# ----------------------------------------------------------------------------- model

class Column:
    def __init__(self, table, raw):
        self.table = table
        self.index = int(raw.get("index", 0))
        self.type = raw["type"]
        self.input_type = raw.get("input_type")
        self.binding = tuple(raw.get("input_binding") or ())
        self.key = ".".join(self.binding)
        self.display = raw.get("input_display") or ""
        self.disabled = bool(raw.get("disabled", False))
        self.enum_path = raw.get("input_enum")
        self.enum = None
        self.settings = raw.get("settings") or {}
        self.raw_rows = raw.get("row_values") or []
        self.prop_type = None  # filled by ChooserRef (float/double/int for float columns)
        self.rows = []
        if self.type.startswith("Output"):
            self.kind = "output"
        elif self.type == "FloatRangeColumn":
            self.kind = "float"
            self.wrap = _bool_str(self.settings.get("bWrapInput"), False)
            self.min_value = float(self.settings.get("MinValue", -180.0))   # double, FloatRangeColumn.h:78-81
            self.max_value = float(self.settings.get("MaxValue", 180.0))
            for cell in self.raw_rows:
                cell = cell if isinstance(cell, dict) else {}
                # missing field = C++ default of FChooserFloatRangeRowData (FloatRangeColumn.h:47-59)
                self.rows.append((f32(float(cell.get("Min", 0.0))), f32(float(cell.get("Max", 0.0))),
                                  _bool_str(cell.get("bNoMin"), False), _bool_str(cell.get("bNoMax"), False)))
        elif self.type == "BoolColumn":
            self.kind = "bool"
            for cell in self.raw_rows:
                if cell not in ("MatchFalse", "MatchTrue", "MatchAny"):
                    raise ChooserRefError("%s: bad bool cell %r" % (self.where(), cell))
                self.rows.append(cell)
        elif self.type == "EnumColumn":
            self.kind = "enum"
            # rows resolved once the enum definition is known (ChooserRef._resolve_enums)
        else:
            self.kind = "unsupported"

    def where(self):
        return "%s col %d (%s %s)" % (self.table.label, self.index, self.type, self.key)

    @property
    def has_filters(self):
        return self.kind != "output"

    def resolve_enum_rows(self, enum_def, warnings):
        self.enum = enum_def
        self.rows = []
        for i, cell in enumerate(self.raw_rows):
            cell = cell if isinstance(cell, dict) else {}
            comparison = cell.get("Comparison", "MatchEqual")
            value = int(cell.get("Value", 0))
            _, internal, _ = _split_value_name(cell.get("ValueName"))
            if enum_def is not None and internal in enum_def.by_internal:
                resolved = enum_def.by_internal[internal]     # EnumColumn.cpp:55-60 (editor PostLoad)
                if resolved != value:
                    warnings.append("%s row %d: ValueName %s -> %d but stored Value %d (editor uses ValueName)"
                                    % (self.where(), i, internal, resolved, value))
                value = resolved
            self.rows.append((comparison, value))

    # -- filters (each returns the surviving row indices, keeps order)

    def filter(self, ctx, indices):
        if self.kind == "float":
            ok, v = ctx.read(self)
            if not ok:
                return list(indices)                     # passthrough, FloatRangeColumn.cpp:109-113
            out = []
            if self.wrap:
                v = ue_wrap(v, self.min_value, self.max_value)
                for i in indices:
                    if i < len(self.rows):
                        mn, mx, no_min, no_max = self.rows[i]
                        if mx < mn:
                            hit = (no_min or v >= mn) or (no_max or v <= mx)
                        else:
                            hit = (no_min or v >= mn) and (no_max or v <= mx)
                        if hit:
                            out.append(i)
            else:
                for i in indices:
                    if i < len(self.rows):
                        mn, mx, no_min, no_max = self.rows[i]
                        if (no_min or v >= mn) and (no_max or v <= mx):
                            out.append(i)
            return out
        if self.kind == "bool":
            ok, v = ctx.read(self)
            if not ok:
                return list(indices)                     # BoolColumn.cpp:56-60
            return [i for i in indices if i < len(self.rows)
                    and (self.rows[i] == "MatchAny" or v == (self.rows[i] == "MatchTrue"))]
        if self.kind == "enum":
            ok, v = ctx.read(self)
            if not ok:
                return list(indices)                     # EnumColumn.cpp:116-120
            out = []
            for i in indices:
                if i < len(self.rows):
                    comparison, value = self.rows[i]
                    if comparison == "MatchEqual":
                        hit = v == value
                    elif comparison == "MatchNotEqual":
                        hit = v != value
                    elif comparison == "MatchAny":
                        hit = True
                    else:
                        hit = False                      # EnumColumn.cpp:84-85
                    if hit:
                        out.append(i)
            return out
        raise ChooserRefError("%s: column type not implemented by the reference" % self.where())


class Result:
    def __init__(self, kind, target, owner_asset):
        self.kind = kind          # asset | evaluate | nested | invalid
        self.target = target      # asset path / table path / nested table name
        self.owner_asset = owner_asset

    @staticmethod
    def parse(raw, owner_asset):
        if not raw or not isinstance(raw, dict):
            return Result("invalid", None, owner_asset)
        t = raw.get("type") or raw.get("_type") or ""
        t = t.rsplit(".", 1)[-1]
        if "raw" in raw and not t:
            return Result("invalid", None, owner_asset)
        if t in ("AssetChooser", "SoftAssetChooser"):
            return Result("asset", object_path(raw.get("Asset")) or None, owner_asset)
        if t == "EvaluateChooser":
            return Result("evaluate", object_path(raw.get("Chooser")) or None, owner_asset)
        if t == "NestedChooser":
            ref = raw.get("Chooser")
            return Result("nested", object_path(ref) if ref else None, owner_asset)
        raise ChooserRefError("result type %r not implemented by the reference" % t)

    def describe(self):
        if self.kind == "asset":
            return short_name(self.target)
        if self.kind == "evaluate":
            return "EvaluateChooser(%s)" % short_name(self.target)
        if self.kind == "nested":
            return "Nested(%s)" % self.target
        return "<invalid>"


class Table:
    def __init__(self, raw, asset_path, nested_name=None):
        self.asset_path = asset_path
        self.nested_name = nested_name
        self.name = raw.get("name")
        self.label = short_name(asset_path) + ("/" + nested_name if nested_name else "")
        self.columns = [Column(self, c) for c in raw.get("columns", [])]
        rows = raw.get("rows", [])
        results = raw.get("results")
        if results is None:
            results = [r.get("result") for r in rows]
        if len(results) != len(rows):
            raise ChooserRefError("%s: %d results but %d rows" % (self.label, len(results), len(rows)))
        self.results = [Result.parse(r, asset_path) for r in results]
        self.disabled = [bool(r.get("disabled", False)) for r in rows]
        fb = raw.get("fallback_result")
        self.fallback = Result.parse(fb, asset_path) if fb else None
        self.nested_raw = raw.get("nested", [])


# ----------------------------------------------------------------------------- evaluation context

class Context:
    def __init__(self, ref, values, missing="error"):
        self.ref = ref
        self.values = values      # canonical key -> value (None = unreadable, passthrough)
        self.missing = missing
        self.reads = []

    def read(self, column):
        key = column.key
        if key not in self.reads:
            self.reads.append(key)
        if key not in self.values:
            if self.missing == "passthrough":
                return False, None
            raise MissingInput(key)
        raw = self.values[key]
        if raw is None:
            return False, None
        return True, self.ref.convert(column, raw)


class EvalResult:
    def __init__(self):
        self.assets = []      # full object paths, in callback order
        self.trace = []       # per result: chain of table[row] / table[fallback]
        self.status = None
        self.reads = []
        self.fallback_used = False

    @property
    def names(self):
        return [short_name(a) for a in self.assets]

    def as_dict(self):
        return {"databases": self.names, "status": self.status, "fallback_used": self.fallback_used,
                "trace": self.trace, "reads": self.reads}


# ----------------------------------------------------------------------------- reference

class ChooserRef:
    def __init__(self, choosers_path, context_json=None, enums_json=None, prop_types=None, data=None, root=None,
                 names_json=None):
        """choosers_path: choosers.json; data: already loaded choosers.json content (path then only
        locates the sibling blueprint jsons); root: default root chooser name (None = chooser.root of
        names_json, read when first needed)."""
        self.choosers_path = os.path.abspath(choosers_path)
        self.default_root = root
        self.names_json = names_json
        if data is None:
            with open(self.choosers_path, encoding="utf-8") as f:
                data = json.load(f)
        self.warnings = []
        self.assets = {}         # asset path -> Table
        self.nested = {}         # (asset path, nested name) -> Table
        self.context_classes = {}
        for top in data.get("tables", []):
            path = top["path"]
            tab = Table(top["table"], path)
            self.assets[path] = tab
            self._add_nested(tab, path)
            for c in top["table"].get("context") or []:
                if c.get("Class"):
                    self.context_classes.setdefault(path, c["Class"])
        if data.get("proxy_tables") or data.get("proxy_assets"):
            self.warnings.append("choosers.json lists proxy tables/assets; the reference does not evaluate them")
        base = os.path.dirname(self.choosers_path)
        self.enums_json = enums_json or os.path.join(base, "blueprints.json")
        self.enums = {}
        self.structs = {}
        if os.path.isfile(self.enums_json):
            with open(self.enums_json, encoding="utf-8") as f:
                bp = json.load(f)
            for e in bp.get("enums", []):
                self.enums[e["path"]] = EnumDef(e["path"], e["name"], [
                    (int(x["value"]), x["internal_name"], x.get("display_name")) for x in e.get("entries", [])])
            for s in bp.get("structs", []):
                self.structs[s["path"]] = {fld["name"]: fld for fld in s.get("fields", [])}
        else:
            self.enums_json = None
        self.context_json = context_json
        self._ctx_cache = {}
        self._bind_cache = {}
        self.prop_type_overrides = dict(prop_types or {})
        self._resolve_enums()

    # -- loading helpers

    def _add_nested(self, table, asset_path):
        for n in table.nested_raw:
            t = Table(n, asset_path, nested_name=n.get("name"))
            key = (asset_path, t.name)
            if key in self.nested:
                raise ChooserRefError("duplicate nested chooser name %r in %s" % (t.name, asset_path))
            self.nested[key] = t
            self._add_nested(t, asset_path)

    def all_tables(self):
        return list(self.assets.values()) + list(self.nested.values())

    def _resolve_enums(self):
        for t in self.all_tables():
            for c in t.columns:
                if c.kind != "enum":
                    continue
                ed = self.enums.get(c.enum_path)
                if ed is None:
                    ed = self._enum_from_cells(c)
                    self.enums.setdefault(c.enum_path, ed)
                c.resolve_enum_rows(ed, self.warnings)

    def _enum_from_cells(self, column):
        """Fallback when blueprints.json is absent: enumerators seen in the table cells (ValueName + Value)."""
        entries = {}
        for t in self.all_tables():
            for c in t.columns:
                if c.kind == "enum" and c.enum_path == column.enum_path:
                    for cell in c.raw_rows:
                        if isinstance(cell, dict) and cell.get("ValueName"):
                            _, internal, disp = _split_value_name(cell["ValueName"])
                            entries[internal] = (int(cell.get("Value", 0)), internal, disp)
        self.warnings.append("enum %s not found in %s; using %d enumerators seen in the table cells"
                             % (column.enum_path, self.enums_json, len(entries)))
        return EnumDef(column.enum_path, short_name(column.enum_path), sorted(entries.values()))

    def find_root(self, name=None):
        if not name and not self.default_root:
            self.default_root = default_root(self.names_json)
        name = name or self.default_root
        hits = [p for p in self.assets if p == name or short_name(p) == name]
        if len(hits) != 1:
            raise ChooserRefError("root chooser %r: %d matches" % (name, len(hits)))
        return self.assets[hits[0]]

    def resolve(self, result):
        if result.kind == "evaluate":
            return self.assets.get(result.target) if result.target else None
        if result.kind == "nested":
            return self.nested.get((result.owner_asset, result.target)) if result.target else None
        return None

    def reachable(self, root):
        """Tables reachable from root through results and fallbacks (DFS order, root first)."""
        seen, order, stack = set(), [], [root]
        while stack:
            t = stack.pop()
            if t is None or id(t) in seen:
                continue
            seen.add(id(t))
            order.append(t)
            nxt = [self.resolve(r) for r in t.results] + ([self.resolve(t.fallback)] if t.fallback else [])
            for n in reversed(nxt):
                if n is not None and id(n) not in seen:
                    stack.append(n)
        return order

    # -- bound property types

    def _context_info(self, asset_path):
        cls = self.context_classes.get(asset_path)
        if cls is None:
            return None
        if cls in self._ctx_cache:
            return self._ctx_cache[cls]
        path = self.context_json
        if path is None:
            pkg = cls.split(".", 1)[0].strip("/")
            path = os.path.join(os.path.dirname(self.choosers_path), "blueprints", pkg.replace("/", "__") + ".json")
        info = None
        if os.path.isfile(path):
            with open(path, encoding="utf-8") as f:
                bp = json.load(f)
            info = {"path": os.path.abspath(path),
                    "variables": {v["name"]: v for v in bp.get("variables", [])},
                    "functions": {fn["name"]: fn for fn in bp.get("functions", [])}}
        self._ctx_cache[cls] = info
        return info

    @staticmethod
    def _type_of(desc):
        t, sub = desc.get("type"), desc.get("subtype")
        if t == "real":
            return sub if sub in ("float", "double") else "double"
        if t in ("float", "double", "int", "bool"):
            return t
        if t == "byte" and desc.get("type_object"):
            return "enum"
        if t == "struct":
            return "struct:" + str(desc.get("type_object"))
        return t

    def property_type(self, column):
        if column.key in self.prop_type_overrides:
            return self.prop_type_overrides[column.key]
        info = self._context_info(column.table.asset_path)
        if info is None or not column.binding:
            return None
        head = column.binding[0]
        if head in info["variables"]:
            cur = self._type_of(info["variables"][head])
        elif head in info["functions"]:
            outs = [o for o in info["functions"][head].get("outputs", []) if o.get("name") == "ReturnValue"]
            cur = self._type_of(outs[0]) if outs else None
        else:
            return None
        for member in column.binding[1:]:
            if not (cur and cur.startswith("struct:")):
                return None
            sp = cur[len("struct:"):]
            if sp in ENGINE_STRUCT_MEMBER_TYPES:
                cur = ENGINE_STRUCT_MEMBER_TYPES[sp].get(member)
            elif sp in self.structs and member in self.structs[sp]:
                cur = self._type_of(self.structs[sp][member])
            else:
                return None
        return cur

    def bindings(self, root):
        """key -> {kind, prop_type, enum, columns} for every filter column reachable from root."""
        if id(root) in self._bind_cache:
            return self._bind_cache[id(root)]
        out = {}
        for t in self.reachable(root):
            for c in t.columns:
                if c.disabled or not c.has_filters:
                    continue
                if c.kind == "unsupported":
                    raise ChooserRefError("%s: column type not implemented by the reference" % c.where())
                b = out.setdefault(c.key, {"kind": c.kind, "columns": [], "display": set()})
                if b["kind"] != c.kind:
                    raise ChooserRefError("input %s bound by %s and %s columns" % (c.key, b["kind"], c.kind))
                b["columns"].append(c)
                if c.display:
                    b["display"].add(c.display)
        for key, b in out.items():
            c0 = b["columns"][0]
            if b["kind"] == "float":
                pt = self.property_type(c0)
                if pt not in FLOAT_TYPES:
                    raise ChooserRefError(
                        "input %s: bound property type %r unknown/unsupported; pass --prop-type %s=float|double|int"
                        % (key, pt, key))
                for c in b["columns"]:
                    c.prop_type = pt
                b["prop_type"] = pt
            elif b["kind"] == "enum":
                b["enum"] = c0.enum
                b["prop_type"] = "enum"
            else:
                b["prop_type"] = "bool"
        self._bind_cache[id(root)] = out
        return out

    # -- input conversion (value as stored in the bound property, then read as the column type)

    def convert(self, column, raw):
        if column.kind == "bool":
            if isinstance(raw, bool):
                return raw
            if isinstance(raw, (int, float)) and raw in (0, 1):
                return bool(raw)
            if isinstance(raw, str) and raw.strip().lower() in ("true", "false", "1", "0"):
                return raw.strip().lower() in ("true", "1")
            raise ChooserRefError("%s: %r is not a bool" % (column.where(), raw))
        if column.kind == "enum":
            return column.enum.value_of(raw)
        if column.kind == "float":
            if isinstance(raw, bool):
                raise ChooserRefError("%s: %r is not a number" % (column.where(), raw))
            v = float(raw)
            pt = column.prop_type or self.property_type(column)
            if pt == "float":
                return f32(v)
            if pt == "int":
                if not v.is_integer() or not -2 ** 31 <= v < 2 ** 31:
                    raise ChooserRefError("%s: %r is not an int32 (bound property is int)" % (column.where(), raw))
                return float(int(v))
            if pt == "double":
                return v
            raise ChooserRefError("%s: unknown bound property type %r" % (column.where(), pt))
        raise ChooserRefError("%s: column type not implemented by the reference" % column.where())

    def canonical_inputs(self, root, values):
        """Map user keys (binding chain 'A.B', last segment 'B', column display name) to binding keys.
        Inputs bound only by disabled or output columns are accepted and ignored (never read)."""
        b = self.bindings(root)
        alias = {}
        for key, info in b.items():
            names = {key, key.rsplit(".", 1)[-1]} | info["display"]
            for n in names:
                alias.setdefault(norm_key(n), set()).add(key)
        unread = set()
        for t in self.reachable(root):
            for c in t.columns:
                if c.disabled or not c.has_filters:
                    unread.update(norm_key(n) for n in (c.key, c.key.rsplit(".", 1)[-1], c.display) if n)
        out = {}
        for k, v in values.items():
            hit = alias.get(norm_key(k))
            if not hit and norm_key(k) in unread:
                continue
            if not hit:
                raise ChooserRefError("unknown input %r (known: %s)" % (k, ", ".join(sorted(b))))
            if len(hit) > 1:
                raise ChooserRefError("ambiguous input %r -> %s" % (k, sorted(hit)))
            key = next(iter(hit))
            if key in out:
                raise ChooserRefError("input %s given twice" % key)
            out[key] = v
        return out

    # -- evaluation (port of UChooserTable::EvaluateChooser)

    def _eval_table(self, table, ctx, cb, path, res):
        if table is None:
            return FAILED                                                    # Chooser.cpp:597-600
        indices = [i for i in range(len(table.results)) if not table.disabled[i]]   # 633-639
        for col in table.columns:                                             # 673-701
            if col.disabled or not col.has_filters:
                continue
            indices = col.filter(ctx, indices)
        any_ok = False
        for i in indices:                                                     # 713-761
            r = table.results[i]
            if r.kind == "invalid":
                continue
            st = self._choose_multi(r, ctx, cb, path + ["%s[%d]" % (table.label, i)], res)
            if st != FAILED:
                any_ok = True
            if st == STOP:
                return STOP
        if not any_ok and table.fallback is not None and table.fallback.kind != "invalid":   # 763-797
            res.fallback_used = True
            st = self._choose_multi(table.fallback, ctx, cb, path + ["%s[fallback]" % table.label], res)
            if st == STOP:
                return STOP
        return CONTINUE if any_ok else FAILED                                 # 824

    def _choose_multi(self, r, ctx, cb, path, res):
        if r.kind == "asset":
            if not r.target:
                return FAILED                                                 # IObjectChooser.h:155-163
            return cb(r.target, path)
        if r.kind in ("evaluate", "nested"):
            return self._eval_table(self.resolve(r), ctx, cb, path, res)
        return FAILED

    def evaluate(self, values, root=None, mode="all", missing="error", canonical=False):
        """Evaluate the root chooser for one input vector.

        values: dict input -> value (bool / number / enum display or internal name); None = unreadable.
        mode:   'all' (AllResults, source pack) or 'first' (FirstResult).
        """
        root_t = root if isinstance(root, Table) else self.find_root(root)
        if not canonical:
            values = self.canonical_inputs(root_t, values)
        else:
            self.bindings(root_t)   # makes sure property types are resolved
        ctx = Context(self, values, missing=missing)
        res = EvalResult()

        def cb(asset, path):
            res.assets.append(asset)
            res.trace.append(" > ".join(path))
            if mode == "all":
                return CONTINUE        # ChooserFunctionLibrary.cpp:358-367
            if mode == "first":
                return STOP            # ChooserFunctionLibrary.cpp:298-302
            raise ChooserRefError("mode must be 'all' or 'first'")

        res.status = self._eval_table(root_t, ctx, cb, [], res)
        res.reads = list(ctx.reads)
        return res

    # -- grid of input vectors

    def candidate_values(self, root, delta=0.01):
        """Per input: every enumerator / both bools / for each effective float bound b: b-delta, the
        representable neighbour below, b, the neighbour above, b+delta (int properties: b-1, b, b+1)."""
        cands = {}
        for key, info in self.bindings(root).items():
            if info["kind"] == "bool":
                cands[key] = [False, True]
            elif info["kind"] == "enum":
                cands[key] = [info["enum"].display(v) for v, _, _ in info["enum"].entries]
            else:
                bounds = set()
                for c in info["columns"]:
                    for mn, mx, no_min, no_max in c.rows:
                        if not no_min:
                            bounds.add(mn)
                        if not no_max:
                            bounds.add(mx)
                pt = info["prop_type"]
                vals = set()
                for b in bounds:
                    if pt == "int":
                        for v in (math.floor(b) - 1, math.floor(b), math.ceil(b), math.ceil(b) + 1):
                            vals.add(int(v))
                    elif pt == "float":
                        for v in (b - delta, f32_next(b, -math.inf), b, f32_next(b, math.inf), b + delta):
                            vals.add(f32(v))
                    else:
                        for v in (b - delta, math.nextafter(b, -math.inf), b, math.nextafter(b, math.inf), b + delta):
                            vals.add(v)
                cands[key] = sorted(vals)
        return cands

    def grid(self, root=None, delta=0.01):
        """Lazy cartesian product: an input is branched only when the evaluation actually reads it,
        so every leaf is a complete equivalence class of the AllResults evaluation."""
        root_t = self.find_root(root)
        binds = self.bindings(root_t)
        cands = self.candidate_values(root_t, delta)
        leaves = []
        stack = [{}]
        while stack:
            assign = stack.pop()
            try:
                res = self.evaluate(assign, root_t, mode="all", canonical=True)
            except MissingInput as e:
                for v in reversed(cands[e.key]):
                    nxt = dict(assign)
                    nxt[e.key] = v
                    stack.append(nxt)
                continue
            leaves.append((assign, res))
        defaults = {}
        for key, info in binds.items():
            defaults[key] = cands[key][0] if info["kind"] == "enum" else (False if info["kind"] == "bool" else 0)
        vectors = []
        for assign, res in leaves:
            full = dict(defaults)
            full.update(assign)
            check = self.evaluate(full, root_t, mode="all", canonical=True)
            if check.assets != res.assets or check.trace != res.trace:
                raise ChooserRefError("don't-care fill changed the result for %r" % assign)
            first = self.evaluate(full, root_t, mode="first", canonical=True)
            vectors.append({
                "inputs": full,
                "read": [k for k in binds if k in assign],
                "databases": res.names,
                "first_result": first.names[0] if first.names else None,
                "status": res.status,
                "fallback_used": res.fallback_used,
                "trace": res.trace,
            })
        return root_t, binds, cands, vectors

    # -- human readable rules

    def describe_cell(self, c, i):
        if i >= len(c.rows):
            return "%s: <no cell, row filtered out>" % c.key
        if c.kind == "float":
            mn, mx, no_min, no_max = c.rows[i]
            if no_min and no_max:
                return "%s: any" % c.key
            lo = "-inf" if no_min else repr(mn)
            hi = "+inf" if no_max else repr(mx)
            if not no_min and not no_max and mx < mn and not c.wrap:
                return "%s in [%s, %s] (empty)" % (c.key, lo, hi)
            return "%s in [%s, %s]%s" % (c.key, lo, hi, " (wrapped)" if c.wrap else "")
        if c.kind == "bool":
            return {"MatchAny": "%s: any", "MatchTrue": "%s", "MatchFalse": "!%s"}[c.rows[i]] % c.key
        if c.kind == "enum":
            comparison, value = c.rows[i]
            if comparison == "MatchAny":
                return "%s: any" % c.key
            op = "==" if comparison == "MatchEqual" else ("!=" if comparison == "MatchNotEqual" else comparison)
            return "%s %s %s" % (c.key, op, c.enum.display(value) if c.enum else value)
        return "%s: <%s>" % (c.key, c.type)

    def rules(self, root=None):
        root_t = self.find_root(root)
        self.bindings(root_t)
        lines = []
        for t in self.reachable(root_t):
            lines.append("TABLE %s" % t.label)
            for i, r in enumerate(t.results):
                conds = [self.describe_cell(c, i) for c in t.columns
                         if not c.disabled and c.has_filters and not self.describe_cell(c, i).endswith(": any")]
                lines.append("  row %d%s: %s => %s" % (i, " (DISABLED)" if t.disabled[i] else "",
                                                       " && ".join(conds) or "always", r.describe()))
            if t.fallback is not None:
                lines.append("  fallback (only if no row succeeded) => %s" % t.fallback.describe())
        return lines


# ----------------------------------------------------------------------------- CLI

def _parse_set(items):
    out = {}
    for it in items or []:
        if "=" not in it:
            raise ChooserRefError("--set expects KEY=VALUE, got %r" % it)
        k, v = it.split("=", 1)
        v = v.strip()
        if v.lower() in ("true", "false"):
            out[k.strip()] = v.lower() == "true"
        elif v.lower() in ("null", "none"):
            out[k.strip()] = None
        else:
            try:
                out[k.strip()] = int(v) if re.fullmatch(r"-?\d+", v) else float(v)
            except ValueError:
                out[k.strip()] = v
    return out


def _write_vectors(path, header, vectors):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8", newline="\n") as f:
        head = json.dumps(header, indent=1, ensure_ascii=False)
        f.write(head[:-2] + ',\n "vectors": [\n')
        for i, v in enumerate(vectors):
            f.write("  " + json.dumps(v, ensure_ascii=False) + (",\n" if i + 1 < len(vectors) else "\n"))
        f.write(" ]\n}\n")
    os.replace(tmp, path)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--choosers", required=True, help="choosers.json from the source pack extraction")
    ap.add_argument("--root", default=None,
                    help="root chooser asset (name or object path); default: chooser.root of --names")
    ap.add_argument("--names", default=NAMES_JSON, help="Neutral names data file (default %(default)s)")
    ap.add_argument("--context-json", default=None,
                    help="blueprint json of the chooser context class (default: <choosers dir>/blueprints/<pkg>.json)")
    ap.add_argument("--enums-json", default=None, help="blueprints.json with enums/structs (default: next to choosers.json)")
    ap.add_argument("--prop-type", action="append", default=[], metavar="KEY=float|double|int",
                    help="override the bound property type of a float input")
    sub = ap.add_subparsers(dest="cmd", required=True)
    pe = sub.add_parser("eval", help="evaluate one input vector")
    pe.add_argument("--set", action="append", default=[], metavar="KEY=VALUE")
    pe.add_argument("--vector-json", help="json object with the inputs")
    pe.add_argument("--mode", choices=("all", "first"), default="all")
    pg = sub.add_parser("grid", help="write the grid of input vectors -> database lists")
    pg.add_argument("--out", required=True)
    pg.add_argument("--delta", type=float, default=0.01, help="offset below/above every float bound")
    sub.add_parser("rules", help="print the reachable tables as rules")
    sub.add_parser("inputs", help="print the inputs, their types and candidate grid values")
    a = ap.parse_args(argv)

    prop_types = {}
    for it in a.prop_type:
        k, _, v = it.partition("=")
        if v not in FLOAT_TYPES:
            ap.error("--prop-type %s: type must be float, double or int" % it)
        prop_types[k] = v
    ref = ChooserRef(a.choosers, context_json=a.context_json, enums_json=a.enums_json, prop_types=prop_types,
                     names_json=a.names)
    root = ref.find_root(a.root)
    for w in ref.warnings:
        print("WARNING:", w, file=sys.stderr)

    if a.cmd == "rules":
        print("\n".join(ref.rules(a.root)))
        return 0
    if a.cmd == "inputs":
        binds = ref.bindings(root)
        cands = ref.candidate_values(root)
        for k, b in binds.items():
            print("%-32s %-6s %-7s %s" % (k, b["kind"], b["prop_type"], cands[k]))
        return 0
    if a.cmd == "eval":
        values = {}
        if a.vector_json:
            with open(a.vector_json, encoding="utf-8") as f:
                values.update(json.load(f))
        values.update(_parse_set(a.set))
        res = ref.evaluate(values, root, mode=a.mode)
        print(json.dumps(res.as_dict(), indent=1, ensure_ascii=False))
        return 0
    if a.cmd == "grid":
        root_t, binds, cands, vectors = ref.grid(a.root, a.delta)
        coverage = {}
        for t in ref.reachable(root_t):
            for i in range(len(t.results)):
                coverage["%s[%d]" % (t.label, i)] = 0
            if t.fallback is not None:
                coverage["%s[fallback]" % t.label] = 0
        for v in vectors:
            hit = set()
            for tr in v["trace"]:
                hit.update(p.strip() for p in tr.split(">"))
            for h in hit:
                coverage[h] = coverage.get(h, 0) + 1
        sets = {}
        for v in vectors:
            sets.setdefault(tuple(sorted(set(v["databases"]))), 0)
            sets[tuple(sorted(set(v["databases"])))] += 1
        header = {
            "generator": "Source/tools/mmbake/motion_chooser_ref.py",
            "format": "utpp-chooser-vectors-1",
            "licence": "Licensed animation data",
            "sources": {
                "choosers_json": ref.choosers_path,
                "choosers_json_sha256": sha256_file(ref.choosers_path),
                "context_json": (ref._context_info(root_t.asset_path) or {}).get("path"),
                "enums_json": ref.enums_json,
                "root": root_t.asset_path,
            },
            "semantics": {
                "mode": "AllResults (databases = union of every matching row, in UE callback order); "
                        "first_result = FirstResult mode",
                "ranges": "closed [Min, Max]; bNoMin/bNoMax open the side; bounds stored as float32",
                "fallback": "evaluated only when no row of that table succeeded",
                "read": "inputs actually read by the AllResults evaluation; the others are don't-care and "
                        "were filled with a neutral value (re-evaluated, same result)",
            },
            "inputs": {k: {"kind": b["kind"], "property_type": b["prop_type"],
                           "display": sorted(b["display"]), "grid_values": cands[k]} for k, b in binds.items()},
            "count": len(vectors),
            "distinct_database_sets": len(sets),
            "empty_results": sum(1 for v in vectors if not v["databases"]),
            "fallback_used": sum(1 for v in vectors if v["fallback_used"]),
            "row_coverage": coverage,
            "warnings": ref.warnings,
        }
        _write_vectors(a.out, header, vectors)
        never = [k for k, n in coverage.items() if n == 0]
        print("vectors: %d, distinct database sets: %d, empty: %d, fallback used: %d"
              % (len(vectors), len(sets), header["empty_results"], header["fallback_used"]))
        print("rows never selected: %s" % (", ".join(never) if never else "none"))
        print("written:", os.path.abspath(a.out))
        return 0
    return 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except ChooserRefError as e:
        print("ERROR:", e, file=sys.stderr)
        sys.exit(2)
