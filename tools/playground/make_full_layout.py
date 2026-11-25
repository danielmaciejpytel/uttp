"""Full playground layout from the converted Unreal level boxes.

Drops the level's endless floor planes, trigger volumes and hidden helper meshes, then adds one floor
sized to the remaining boxes (plus a margin) and a low railing around it, so the layout can stand on
its own anywhere (e.g. above the city) without covering streets or buildings.
"""
import argparse
import json
import os

DROP_PREFIXES = ('Floor', 'BP_ZeroGZone')
MARGIN_M = 6.0
RAIL_H_M = 1.1     # default; --wall-height raises the railing into walls that hide the surroundings
RAIL_T_M = 0.3


def build(src, rail_h=RAIL_H_M, ceiling=False, light_step=0.0, light_h=20.0, light_lm=200000.0):
    boxes = [b for b in src['boxes']
             if b.get('visible', True) and not b['name'].startswith(DROP_PREFIXES)]
    lo_x = min(b['center'][0] - b['half'][0] for b in boxes) - MARGIN_M
    hi_x = max(b['center'][0] + b['half'][0] for b in boxes) + MARGIN_M
    lo_y = min(b['center'][1] - b['half'][1] for b in boxes) - MARGIN_M
    hi_y = max(b['center'][1] + b['half'][1] for b in boxes) + MARGIN_M
    cx, cy = (lo_x + hi_x) / 2, (lo_y + hi_y) / 2
    hx, hy = (hi_x - lo_x) / 2, (hi_y - lo_y) / 2
    out = [{'name': 'floor', 'center': [cx, cy, -0.5], 'half': [hx, hy, 0.5], 'yaw_deg': 0.0}]
    r = rail_h / 2
    out += [
        {'name': 'rail_south', 'center': [cx, lo_y + RAIL_T_M / 2, r], 'half': [hx, RAIL_T_M / 2, r], 'yaw_deg': 0.0},
        {'name': 'rail_north', 'center': [cx, hi_y - RAIL_T_M / 2, r], 'half': [hx, RAIL_T_M / 2, r], 'yaw_deg': 0.0},
        {'name': 'rail_west', 'center': [lo_x + RAIL_T_M / 2, cy, r], 'half': [RAIL_T_M / 2, hy, r], 'yaw_deg': 0.0},
        {'name': 'rail_east', 'center': [hi_x - RAIL_T_M / 2, cy, r], 'half': [RAIL_T_M / 2, hy, r], 'yaw_deg': 0.0},
    ]
    if ceiling:
        out.append({'name': 'ceiling', 'center': [cx, cy, rail_h + 0.5], 'half': [hx, hy, 0.5], 'yaw_deg': 0.0})
    lights = []
    if light_step > 0:
        nx, ny = max(1, round(2 * hx / light_step)), max(1, round(2 * hy / light_step))
        for i in range(nx):
            for j in range(ny):
                lights.append({'pos': [lo_x + (i + 0.5) * 2 * hx / nx, lo_y + (j + 0.5) * 2 * hy / ny, light_h],
                               'radius': light_step * 2.0, 'intensity': light_lm})
    for b in boxes:
        e = {'name': b['name'], 'center': b['center'], 'half': b['half']}
        e['quat' if 'quat' in b else 'yaw_deg'] = b.get('quat', b.get('yaw_deg', 0.0))
        out.append(e)
    return {'name': 'full', 'note': 'Full playground: level boxes + own floor and railing.',
            'anchor': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw_deg': 0.0}, 'burial_m': 0.0, 'boxes': out, 'lights': lights}, (lo_x, hi_x, lo_y, hi_y)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--input', required=True, help='playground_boxes.json from ue_to_red.py')
    ap.add_argument('--output', required=True)
    ap.add_argument('--wall-height', type=float, default=RAIL_H_M, help='height of the border (m)')
    ap.add_argument('--ceiling', action='store_true', help='close the top at the wall height')
    ap.add_argument('--light-step', type=float, default=0.0, help='grid spacing of point lights (m); 0 = none')
    ap.add_argument('--light-height', type=float, default=20.0)
    ap.add_argument('--light-lumen', type=float, default=200000.0)
    a = ap.parse_args()
    with open(a.input, encoding='utf-8') as f:
        layout, ext = build(json.load(f), a.wall_height, a.ceiling, a.light_step, a.light_height, a.light_lumen)
    os.makedirs(os.path.dirname(os.path.abspath(a.output)), exist_ok=True)
    with open(a.output, 'w', encoding='utf-8') as f:
        json.dump(layout, f, indent=1)
    print(f'lights={len(layout["lights"])} boxes={len(layout["boxes"])} floor x {ext[0]:.1f}..{ext[1]:.1f} y {ext[2]:.1f}..{ext[3]:.1f}')


if __name__ == '__main__':
    main()
