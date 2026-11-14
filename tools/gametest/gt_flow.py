"""Flow control of the night runner that needs no game: the look-ahead probe cache and the early stop.

ProbeCache   The look-ahead walk before a walking scenario (night.clear_path) costs 6-8 s. When the previous probe started from
             the same spot (within SPOT_M), along the same direction (camera heading, route heading and their sum each within
             HEADING_DEG), covered at least REACH_SLACK of the distance the new probe would cover (the speed that walk measured over
             the new duration), came back clear, and V stands at that spot again, the ground ahead is known to be open: the probe
             is skipped. The short "face" push that turns V along the route stays, so the scenario starts in the same state as
             after a probe.
EarlyStop    The first walking scenarios of a cycle that are all invalid for one cause (V cannot move, no motion lines in the
             log ...) show that the rest of the cycle would be invalid for the same cause: the cycle stops with that cause in
             one line instead of playing everything else.
"""
from __future__ import annotations

import math

SPOT_M = 1.0                 # same spot: V within this distance of where the earlier probe started
HEADING_DEG = 15.0           # same direction: camera heading, route heading and their sum each within this many degrees
REACH_SLACK = 0.9            # the earlier walk covered at least this share of the distance needed now (position noise)


def _wrap(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def heading_deg(cf) -> float:
    """Heading of a camera forward vector (x, y) in degrees, clockwise from +y (the same sense as gt_night.bearing_deg)."""
    return math.degrees(math.atan2(cf[0], cf[1]))


class ProbeCache:
    """Look-ahead probes of this cycle that came back clear."""

    def __init__(self, spot_m: float = SPOT_M, heading_deg_tol: float = HEADING_DEG, reach_slack: float = REACH_SLACK):
        self.spot_m, self.heading_tol, self.reach_slack = spot_m, heading_deg_tol, reach_slack
        self.entries: list[dict] = []
        self.skipped = 0
        self.invalidated: list[str] = []

    def invalidate(self, why: str) -> None:
        """Forgets every probe: the camera was turned, an attempt was blocked, a vehicle moved, a new cycle started."""
        if self.entries:
            self.invalidated.append(why)
        self.entries = []

    def remember(self, where: str, p, cf, la: dict, moved_m: float) -> None:
        """A probe along `la` from position p (x, y[, z]) with camera forward cf that came back clear after `moved_m` m."""
        self.entries = [e for e in self.entries if not self._same_probe(e, p, cf, la)]
        dur = max(float(la['duration']), 0.1)
        self.entries.append({'where': where, 'p': (float(p[0]), float(p[1])), 'cam_deg': heading_deg(cf),
                             'route_deg': float(la['heading_deg']), 'covered_m': float(moved_m), 'speed_mps': float(moved_m) / dur})

    def _same_probe(self, e: dict, p, cf, la: dict) -> bool:
        return (math.hypot(e['p'][0] - p[0], e['p'][1] - p[1]) <= self.spot_m
                and abs(_wrap(e['cam_deg'] - heading_deg(cf))) <= self.heading_tol
                and abs(_wrap(e['route_deg'] - la['heading_deg'])) <= self.heading_tol)

    def lookup(self, p, cf, la: dict) -> dict | None:
        """The earlier probe that makes a new one unnecessary, with the measured differences, or None."""
        cam = heading_deg(cf)
        for e in reversed(self.entries):
            dist = math.hypot(e['p'][0] - p[0], e['p'][1] - p[1])
            d_cam = _wrap(cam - e['cam_deg'])
            d_route = _wrap(la['heading_deg'] - e['route_deg'])
            d_world = _wrap((cam + la['heading_deg']) - (e['cam_deg'] + e['route_deg']))
            # the distance the new probe would really cover: the speed the earlier walk measured over the new duration (the planned
            # reach_m rests on a speed table that is above what V does at the probe's stick)
            needed = e['speed_mps'] * float(la['duration'])
            if (dist <= self.spot_m and abs(d_cam) <= self.heading_tol and abs(d_route) <= self.heading_tol
                    and abs(d_world) <= self.heading_tol and e['covered_m'] >= self.reach_slack * needed):
                return {'where': e['where'], 'dist_m': round(dist, 2), 'cam_diff_deg': round(d_cam, 1),
                        'route_diff_deg': round(d_route, 1), 'covered_m': round(e['covered_m'], 1),
                        'needed_m': round(needed, 1)}
        return None


class EarlyStop:
    """The first `count` walking scenarios of a cycle: when every one of them is invalid for the same cause, the cycle stops.
    A valid scenario among them, or different causes, end the watch (the cycle goes on)."""

    def __init__(self, count: int = 2):
        self.count = count
        self.seen: list[tuple[str, str | None]] = []
        self.decided = count <= 0

    @property
    def watching(self) -> bool:
        return not self.decided

    def note(self, label: str, cause: str | None) -> str | None:
        """Report the next walking scenario (`cause` None = it was valid). Returns the one-line reason when the cycle must
        stop, else None."""
        if self.decided:
            return None
        self.seen.append((label, cause))
        if cause is None:
            self.decided = True
            return None
        if len(self.seen) < self.count:
            return None
        self.decided = True
        causes = {c for _, c in self.seen}
        if len(causes) != 1:
            return None
        names = ', '.join(n for n, _ in self.seen)
        return f'the first {self.count} walking scenarios ({names}) are invalid for one cause: {cause}'
