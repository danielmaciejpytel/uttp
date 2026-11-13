"""Motorcycle geometry for bike_cases.ps1 (offline helper, no game input, no game launch). Wraps car_cases_nav.py.

Why the car cases end in NOT MOUNTED on a bike (2026-11-07):
car_cases_nav.py has no preset for the sportbike records, so the generic CAR window applied (door x 0.99, V allowed
1.5 .. 2.7 m to the side). The log shows V standing at lat 1.49 .. 1.59 m of the bike centre line (parallel / door / behind)
when X was pressed. The bike mount prompt (bike_interaction 'Mount' / motorcycle_device_interaction seat_front_left,
InteractRangeCore.hpp) is gated by a seat sphere of r 1.0 .. 1.3 m around V, and the UTPP approach planner does not handle
bikes at all (utpp_vehicle_approach.reds skips BikeObject: the engine slides V to the clip start itself). A pose 1.5 m beside
the bike is outside the sphere: no prompt, the X tap does nothing, both tries fail, NOT MOUNTED.

This module adds BIKE presets WITHOUT touching car_cases_nav.py (the car cases stay as they are): it loads a private copy of that
module, extends the copy's tables and replaces its GENERIC fallback by a bike geometry, then delegates the CLI to the copy.
The numbers are a HYPOTHESIS (the seat offset from the bike origin is not extracted): lat window 0.30 .. 0.90 m, target 0.85,
lon target 0 (a --lon-bias ladder of the caller covers the unknown seat offset).

Extra CLI (one JSON line; everything else = car_cases_nav.py):
  bike_cases_nav.py side --log <utpp log> --marker "<text>" --since <mark>    which side of the bike V stands on now
                    -> {'state': 'ok'|'mounted'|'nopos'|'nocar', 'side': 'left'|'right', 'lat': .., 'lon': ..}
"""
from __future__ import annotations

import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import importlib.util  # noqa: E402

# A PRIVATE copy of the car_cases_nav module (own PRESETS / ALIASES / GENERIC): the bike tables never leak into car_cases_nav in
# the same process (test_car_cases_nav.py, car_cases.ps1 keep their own module object and the car geometry).
_spec = importlib.util.spec_from_file_location('car_cases_nav_bike', os.path.join(HERE, 'car_cases_nav.py'))
cn = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(cn)

# rig = (seat y, door x, door y): only seat y - 0.14 (= lon target 0) and door x (window start / walk target) are used by
# car_cases_nav; lat_max narrows the 2.7 m car default to the seat sphere.
BIKE = dict(rig=(0.14, 0.40, 0.0), fwd=0.0, away=0.0, settle=4.0, lat_max=0.90)

BIKE_PRESETS = {
    'v_sportbike1_yaiba_kusanagi': dict(label='Yaiba Kusanagi (sportbike1)', **BIKE),
    'v_sportbike2_arch': dict(label='Arch (sportbike2)', **BIKE),
    'v_sportbike3_brennan_apollo': dict(label='Brennan Apollo (sportbike3)', **BIKE),
}
BIKE_ALIASES = {
    # the project data names the Kusanagi player skin '..._player_02' (tweakxl.json); same body / rig as the base record
    'v_sportbike1_yaiba_kusanagi_player_02': 'v_sportbike1_yaiba_kusanagi',
}
BIKE_GENERIC = dict(label='unknown bike record (generic bike geometry)', **BIKE)

cn.PRESETS.update(BIKE_PRESETS)
cn.ALIASES.update(BIKE_ALIASES)
cn.GENERIC = BIKE_GENERIC          # preset() reads the module global: an unknown record gets bike (not car) geometry

# Mount ladder (V lon shift [m] relative to the seat-line guess, tried in this order until a mount line appears).
LON_LADDER = (0.0, 0.45, -0.45, 0.9)


def preset(record: str) -> dict:
    return cn.preset(record)


def side_of(pos: dict) -> dict:
    """Which side of the bike V stands on: lat > 0 = bike LEFT (gt_night.car_frame)."""
    if pos.get('mounted') or pos.get('src') == 'mounted':
        return {'state': 'mounted'}
    if not pos.get('v') or not pos.get('vr'):
        return {'state': 'nocar'}
    lon, lat = cn.gn.car_frame(pos['p'][:2], pos['v'][:2], pos['vr'])
    return {'state': 'ok', 'side': 'left' if lat > 0 else 'right', 'lat': round(lat, 2), 'lon': round(lon, 2)}


def main(argv=None) -> int:
    a = list(sys.argv[1:] if argv is None else argv)
    if a and a[0] == 'side':
        import argparse
        ap = argparse.ArgumentParser()
        ap.add_argument('cmd')
        ap.add_argument('--log', required=True)
        ap.add_argument('--marker', default='')
        ap.add_argument('--since', default='')
        ap.add_argument('--max-age', type=float, default=8.0)
        o = ap.parse_args(a)
        text = cn.read_log_text(o.log, o.since, o.marker)
        pos, why = cn._pos_from_log(text, o.marker, o.max_age)
        if pos is None:
            print(json.dumps({'state': 'nopos', 'why': why}))
        else:
            print(json.dumps(side_of(pos)))
        return 0
    return cn.main(a)


if __name__ == '__main__':
    sys.exit(main())
