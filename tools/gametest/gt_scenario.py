"""Scenario data model for the UTPP in-game test harness (stdlib only, no input sending here).

File format (JSON, see scenarios.json):
{
  "defaults": {"rate_hz": 60, "interp": "smooth"},
  "scenarios": {
    "<name>": {
      "description": "...",
      "cet": ["Game.UTPPSetMMTrace(true)"],          # optional commands typed before the run (with --cet)
      "thresholds": {"<metric>": {"max": 2.0}},       # optional per-scenario overrides of thresholds.json
      "steps": [
        {"name": "walk", "duration": 6.0, "interp": "smooth",
         "ls": [[0, 0, 0], [0.8, 0, 0.5], [6, 0, 0.5]],            # [t, x, y]  x right, y forward
         "ls_polar": {"keys": [[0, 0, 1], [3, 180, 1]], "loop": false},  # [t, deg, mag]  0=fwd, +=clockwise
         "rs": [[t, x, y], ...], "rs_polar": ...,
         "lt": [[t, v], ...], "rt": [[t, v], ...],
         "buttons": [{"t": 0.5, "button": "B", "hold": 0.12}],
         "metrics": false                                           # exclude the step from pass/fail
        }
      ]
    }
  }
}
Keys are relative to the step start. A track may be a plain key list or {"keys": [...], "loop": true, "interp": "linear"}.
With loop=true the key pattern repeats with period = last key time (e.g. slalom). Polar tracks interpolate the angle
linearly in degrees (no wrapping), so [0,0,1] -> [1,90,1] is a 90 deg/s rotation clockwise.
Channels a step does not define keep the final value of the previous step (continuity of ramps); every scenario
implicitly ends with all channels neutral (the runner releases everything).
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field

BUTTONS = {
    'A': 0x1000, 'B': 0x2000, 'X': 0x4000, 'Y': 0x8000,
    'LB': 0x0100, 'RB': 0x0200, 'LS': 0x0040, 'RS': 0x0080, 'L3': 0x0040, 'R3': 0x0080,
    'START': 0x0010, 'BACK': 0x0020,
    'DPAD_UP': 0x0001, 'DPAD_DOWN': 0x0002, 'DPAD_LEFT': 0x0004, 'DPAD_RIGHT': 0x0008,
}
STICK_CHANNELS = ('ls', 'rs')
SCALAR_CHANNELS = ('lt', 'rt')
INTERPS = ('linear', 'smooth', 'step')


class ScenarioError(ValueError):
    pass


def _ease(u: float, mode: str) -> float:
    if mode == 'smooth':
        return u * u * (3.0 - 2.0 * u)
    if mode == 'step':
        return 0.0
    return u


@dataclass
class Track:
    keys: list            # list of tuples (t, v0[, v1])
    loop: bool = False
    interp: str = 'smooth'
    polar: bool = False

    def value(self, t: float) -> tuple:
        keys = self.keys
        if self.loop and len(keys) > 1:
            period = keys[-1][0] - keys[0][0]
            if period > 0 and t > keys[-1][0]:
                t = keys[0][0] + math.fmod(t - keys[0][0], period)
        if t <= keys[0][0]:
            return tuple(keys[0][1:])
        if t >= keys[-1][0]:
            return tuple(keys[-1][1:])
        for a, b in zip(keys, keys[1:]):
            if a[0] <= t <= b[0]:
                span = b[0] - a[0]
                u = 0.0 if span <= 0 else (t - a[0]) / span
                if self.polar:
                    # angle always linear (constant deg/s); magnitude eased
                    ang = a[1] + (b[1] - a[1]) * u
                    mag = a[2] + (b[2] - a[2]) * _ease(u, self.interp)
                    return ang, mag
                e = _ease(u, self.interp)
                return tuple(av + (bv - av) * e for av, bv in zip(a[1:], b[1:]))
        return tuple(keys[-1][1:])  # pragma: no cover

    def end_value(self, duration: float) -> tuple:
        return self.value(duration)


def polar_to_xy(deg: float, mag: float) -> tuple[float, float]:
    r = math.radians(deg)
    return mag * math.sin(r), mag * math.cos(r)


def clamp_stick(x: float, y: float) -> tuple[float, float]:
    x = max(-1.0, min(1.0, x))
    y = max(-1.0, min(1.0, y))
    m = math.hypot(x, y)
    if m > 1.0:
        x, y = x / m, y / m
    return x, y


@dataclass
class ButtonEvent:
    t: float
    button: str
    hold: float = 0.1

    @property
    def mask(self) -> int:
        return BUTTONS[self.button]


@dataclass
class Step:
    name: str
    duration: float
    tracks: dict = field(default_factory=dict)   # channel -> Track ('ls', 'ls_polar', 'rs', 'rs_polar', 'lt', 'rt')
    buttons: list = field(default_factory=list)
    metrics: bool = True


@dataclass
class Frame:
    t: float
    ls: tuple
    rs: tuple
    lt: float
    rt: float
    buttons: int


@dataclass
class Scenario:
    name: str
    description: str
    steps: list
    cet: list = field(default_factory=list)
    thresholds: dict = field(default_factory=dict)
    rate_hz: float = 60.0

    @property
    def duration(self) -> float:
        return sum(s.duration for s in self.steps)

    def step_bounds(self) -> list[tuple[str, float, float]]:
        out, t = [], 0.0
        for s in self.steps:
            out.append((s.name, t, t + s.duration))
            t += s.duration
        return out

    def step_stick_stats(self, dt: float = 1.0 / 30) -> dict:
        """{step name: (min |ls|, max |ls|)} rounded to 3 decimals."""
        out: dict = {}
        for i, fr in self.sample(dt):
            m = math.hypot(*fr.ls)
            lo, hi = out.get(i, (m, m))
            out[i] = (min(lo, m), max(hi, m))
        return {self.steps[i].name: (round(lo, 3), round(hi, 3)) for i, (lo, hi) in out.items()}

    def sample(self, dt: float | None = None):
        """Yield (step_index, Frame) at fixed rate; time is absolute from scenario start."""
        dt = dt or 1.0 / self.rate_hz
        carry = {'ls': (0.0, 0.0), 'rs': (0.0, 0.0), 'lt': 0.0, 'rt': 0.0}
        t0 = 0.0
        for i, step in enumerate(self.steps):
            n = max(1, int(math.ceil(step.duration / dt - 1e-9)))
            for k in range(n + (1 if i == len(self.steps) - 1 else 0)):
                lt = min(k * dt, step.duration)
                yield i, self._frame(step, lt, t0 + lt, carry)
            carry = {c: self._channel(step, c, step.duration, carry) for c in carry}
            t0 += step.duration

    @staticmethod
    def _channel(step: Step, ch: str, t: float, carry: dict):
        if ch in STICK_CHANNELS:
            if ch in step.tracks:
                return clamp_stick(*step.tracks[ch].value(t))
            if ch + '_polar' in step.tracks:
                return clamp_stick(*polar_to_xy(*step.tracks[ch + '_polar'].value(t)))
            return carry[ch]
        if ch in step.tracks:
            return max(0.0, min(1.0, step.tracks[ch].value(t)[0]))
        return carry[ch]

    def _frame(self, step: Step, lt: float, at: float, carry: dict) -> Frame:
        mask = 0
        for b in step.buttons:
            if b.t <= lt < b.t + b.hold:
                mask |= b.mask
        return Frame(at, self._channel(step, 'ls', lt, carry), self._channel(step, 'rs', lt, carry),
                     self._channel(step, 'lt', lt, carry), self._channel(step, 'rt', lt, carry), mask)


# ---------------------------------------------------------------------------------------------- parsing

def _parse_track(raw, ch: str, step_interp: str, where: str) -> Track:
    polar = ch.endswith('_polar')
    loop = False
    interp = step_interp
    keys = raw
    if isinstance(raw, dict):
        keys = raw.get('keys')
        loop = bool(raw.get('loop', False))
        interp = raw.get('interp', step_interp)
    if interp not in INTERPS:
        raise ScenarioError(f'{where}: unknown interp {interp!r}')
    if not isinstance(keys, list) or not keys:
        raise ScenarioError(f'{where}: track {ch} needs a non-empty key list')
    width = 2 if ch in SCALAR_CHANNELS else 3
    out = []
    last_t = -math.inf
    for k in keys:
        if not isinstance(k, (list, tuple)) or len(k) != width:
            raise ScenarioError(f'{where}: track {ch} key {k!r} must have {width} numbers')
        try:
            k = tuple(float(v) for v in k)
        except (TypeError, ValueError):
            raise ScenarioError(f'{where}: track {ch} key {k!r} is not numeric') from None
        if k[0] < last_t:
            raise ScenarioError(f'{where}: track {ch} key times must be ascending')
        last_t = k[0]
        out.append(k)
    return Track(out, loop=loop, interp=interp, polar=polar)


def parse_scenario(name: str, raw: dict, defaults: dict | None = None) -> Scenario:
    defaults = defaults or {}
    step_interp_default = defaults.get('interp', 'smooth')
    steps = []
    if not raw.get('steps'):
        raise ScenarioError(f'{name}: no steps')
    for i, rs in enumerate(raw['steps']):
        where = f'{name}.steps[{i}]'
        sname = rs.get('name') or f'step{i}'
        try:
            dur = float(rs['duration'])
        except (KeyError, TypeError, ValueError):
            raise ScenarioError(f'{where}: duration missing/invalid') from None
        if dur <= 0:
            raise ScenarioError(f'{where}: duration must be > 0')
        interp = rs.get('interp', step_interp_default)
        tracks = {}
        for ch in ('ls', 'ls_polar', 'rs', 'rs_polar', 'lt', 'rt'):
            if ch in rs:
                tracks[ch] = _parse_track(rs[ch], ch, interp, where)
        for st in STICK_CHANNELS:
            if st in tracks and st + '_polar' in tracks:
                raise ScenarioError(f'{where}: both {st} and {st}_polar given')
        buttons = []
        for b in rs.get('buttons', []):
            bn = str(b.get('button', '')).upper()
            if bn not in BUTTONS:
                raise ScenarioError(f'{where}: unknown button {bn!r}')
            ev = ButtonEvent(float(b.get('t', 0.0)), bn, float(b.get('hold', 0.1)))
            if ev.t < 0 or ev.hold <= 0 or ev.t >= dur:
                raise ScenarioError(f'{where}: button event {b!r} outside step')
            buttons.append(ev)
        unknown = set(rs) - {'name', 'duration', 'interp', 'ls', 'ls_polar', 'rs', 'rs_polar', 'lt', 'rt',
                             'buttons', 'metrics', 'note'}
        if unknown:
            raise ScenarioError(f'{where}: unknown keys {sorted(unknown)}')
        steps.append(Step(sname, dur, tracks, buttons, bool(rs.get('metrics', True))))
    names = [s.name for s in steps]
    if len(set(names)) != len(names):
        raise ScenarioError(f'{name}: duplicate step names')
    return Scenario(name, raw.get('description', ''), steps, list(raw.get('cet', [])),
                    dict(raw.get('thresholds', {})), float(raw.get('rate_hz', defaults.get('rate_hz', 60.0))))


def load_scenarios(path: str) -> dict[str, Scenario]:
    with open(path, encoding='utf-8') as f:
        data = json.load(f)
    defaults = data.get('defaults', {})
    return {n: parse_scenario(n, r, defaults) for n, r in data.get('scenarios', {}).items()}


def describe(sc: Scenario, dt: float = 0.5) -> str:
    """Human-readable plan (dry run)."""
    lines = [f'{sc.name}: {sc.description} ({sc.duration:.1f} s, {sc.rate_hz:.0f} Hz)']
    if sc.cet:
        lines.append('  cet: ' + ' ; '.join(sc.cet))
    bounds = {i: b for i, b in enumerate(sc.step_bounds())}
    last_step = -1
    next_t = 0.0
    for i, fr in sc.sample():
        if i != last_step:
            name, a, b = bounds[i]
            st = sc.steps[i]
            btn = ', '.join(f'{e.button}@{e.t:.2f}s/{e.hold:.2f}' for e in st.buttons)
            lines.append(f'  [{a:6.2f}-{b:6.2f}] step {name}' + (f'  buttons: {btn}' if btn else '') +
                         ('' if st.metrics else '  (no metrics)'))
            last_step = i
            next_t = fr.t
        if fr.t + 1e-9 >= next_t:
            lines.append(f'      t={fr.t:6.2f} ls=({fr.ls[0]:+.2f},{fr.ls[1]:+.2f}) |ls|={math.hypot(*fr.ls):.2f} '
                         f'rs=({fr.rs[0]:+.2f},{fr.rs[1]:+.2f}) lt={fr.lt:.2f} rt={fr.rt:.2f} btn=0x{fr.buttons:04X}')
            next_t = fr.t + dt
    return '\n'.join(lines)
