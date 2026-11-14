"""Time budget of one night cycle: where the wall time went, printed at the end of the run.

Budget collects per scenario: the total, the look-ahead probe time, the waits, the copy of the session files, the retries (count
and seconds), the number of probes that were skipped, and the rest = play time. It also collects named post-processing phases.
table() puts them next to the step totals of the cycle (launch, arm, scenarios, recorder stop, quit, collect ...).
"""
from __future__ import annotations

import time
from contextlib import contextmanager
from datetime import datetime

STEP_GROUPS = (('close old game', ('ensure_closed',)), ('install', ('install',)), ('launch', ('launch#',)), ('arm', ('arm',)),
               ('record start', ('record_start',)), ('scenarios', ('scenarios',)), ('recorder stop', ('record_stop',)),
               ('quit', ('quit', 'quit_after')), ('collect', ('collect',)))


def _secs(st: dict) -> float | None:
    a, b = st.get('start'), st.get('end')
    if isinstance(a, datetime) and isinstance(b, datetime):
        return (b - a).total_seconds()
    return None


class Budget:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.scen: dict[str, dict] = {}
        self.order: list[str] = []
        self.current: str | None = None
        self.phases: dict[str, float] = {}
        self.notes: dict[str, str] = {}

    # ---- scenarios
    def begin(self, key: str) -> None:
        self.current = key
        if key not in self.scen:
            self.scen[key] = {'total': 0.0, 'probe': 0.0, 'wait': 0.0, 'copy': 0.0, 'retry_n': 0, 'retry_s': 0.0, 'skipped': 0}
            self.order.append(key)

    def end(self, total_s: float) -> None:
        if self.current:
            self.scen[self.current]['total'] += total_s
        self.current = None

    def add(self, field: str, seconds: float) -> None:
        if self.current:
            self.scen[self.current][field] += seconds

    def count(self, field: str, n: int = 1) -> None:
        if self.current:
            self.scen[self.current][field] += n

    @contextmanager
    def timed(self, field: str):
        """Seconds spent inside go to `field` of the current scenario."""
        t0 = self.clock()
        try:
            yield
        finally:
            self.add(field, self.clock() - t0)

    # ---- phases
    @contextmanager
    def phase(self, name: str, note: str = ''):
        t0 = self.clock()
        try:
            yield
        finally:
            self.phases[name] = round(self.phases.get(name, 0.0) + self.clock() - t0, 1)
            if note:
                self.notes[name] = note

    def set_phase(self, name: str, seconds: float, note: str = '') -> None:
        self.phases[name] = round(seconds, 1)
        if note:
            self.notes[name] = note

    # ---- report
    def rows(self) -> list[dict]:
        out = []
        for k in self.order:
            s = self.scen[k]
            play = max(0.0, s['total'] - s['probe'] - s['wait'] - s['copy'] - s['retry_s'])
            out.append({'scenario': k, 'total': round(s['total'], 1), 'play': round(play, 1), 'probe': round(s['probe'], 1),
                        'retry': f'{s["retry_n"]}/{s["retry_s"]:.1f}', 'wait': round(s['wait'], 1), 'copy': round(s['copy'], 1),
                        'probes_skipped': s['skipped']})
        return out

    def phase_totals(self, steps: list[dict]) -> dict[str, float]:
        res: dict[str, float] = {}
        for label, prefixes in STEP_GROUPS:
            tot = [(_secs(st) or 0.0) for st in steps if any(st.get('name', '').startswith(p) for p in prefixes)]
            if tot:
                res[label] = round(sum(tot), 1)
        return res

    def table(self, steps: list[dict], post_names: tuple = ()) -> list[str]:
        lines = []
        rows = self.rows()
        if rows:
            w = max(len(r['scenario']) for r in rows + [{'scenario': 'scenario'}])
            lines.append(f'{"scenario":<{w}}  {"total":>6} {"play":>6} {"probe":>6} {"retry n/s":>10} {"wait":>6} {"copy":>6} '
                         f'{"probes skipped":>14}')
            for r in rows:
                lines.append(f'{r["scenario"]:<{w}}  {r["total"]:>6} {r["play"]:>6} {r["probe"]:>6} {r["retry"]:>10} '
                             f'{r["wait"]:>6} {r["copy"]:>6} {r["probes_skipped"]:>14}')
            tot = {k: round(sum(r[k] for r in rows), 1) for k in ('total', 'play', 'probe', 'wait', 'copy')}
            lines.append(f'{"sum":<{w}}  {tot["total"]:>6} {tot["play"]:>6} {tot["probe"]:>6} {"":>10} {tot["wait"]:>6} '
                         f'{tot["copy"]:>6}')
        ph = self.phase_totals(steps)
        if ph:
            lines.append('phases (s): ' + ' | '.join(f'{k} {v}' for k, v in ph.items()) + f' | total {round(sum(ph.values()), 1)}')
        post = [(k, v) for k, v in self.phases.items()]
        if post:
            lines.append('post-processing (s): ' + ' | '.join(f'{k} {v}' + (f' ({self.notes[k]})' if k in self.notes else '')
                                                              for k, v in post))
        return lines
