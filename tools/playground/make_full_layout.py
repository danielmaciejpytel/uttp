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
RAIL_H_M = 1.1
RAIL_T_M = 0.3


def build(src):
    boxes = [b for b in src['boxes']
             if b.get('visible', True) and not b['name'].startswith(DROP_PREFIXES)]
    lo_x = min(b['center'][0] - b['half'][0] for b in boxes) - MARGIN_M
    hi_x = max(b['center'][0] + b['half'][0] for b in boxes) + MARGIN_M
    lo_y = min(b['center'][1] - b['half'][1] for b in boxes) - MARGIN_M
    hi_y = max(b['center'][1] + b['half'][1] for b in boxes) + MARGIN_M
    cx, cy = (lo_x + hi_x) / 2, (lo_y + hi_y) / 2
    hx, hy = (hi_x - lo_x) / 2, (hi_y - lo_y) / 2
    out = [{'name': 'floor', 'center': [cx, cy, -0.5], 'half': [hx, hy, 0.5], 'yaw_deg': 0.0}]
    r = RAIL_H_M / 2
    out += [
        {'name': 'rail_south', 'center': [cx, lo_y + RAIL_T_M / 2, r], 'half': [hx, RAIL_T_M / 2, r], 'yaw_deg': 0.0},
        {'name': 'rail_north', 'center': [cx, hi_y - RAIL_T_M / 2, r], 'half': [hx, RAIL_T_M / 2, r], 'yaw_deg': 0.0},
        {'name': 'rail_west', 'center': [lo_x + RAIL_T_M / 2, cy, r], 'half': [RAIL_T_M / 2, hy, r], 'yaw_deg': 0.0},
        {'name': 'rail_east', 'center': [hi_x - RAIL_T_M / 2, cy, r], 'half': [RAIL_T_M / 2, hy, r], 'yaw_deg': 0.0},
    ]
    for b in boxes:
        e = {'name': b['name'], 'center': b['center'], 'half': b['half']}
        e['quat' if 'quat' in b else 'yaw_deg'] = b.get('quat', b.get('yaw_deg', 0.0))
        out.append(e)
    return {'name': 'full', 'note': 'Full playground: level boxes + own floor and railing.',
            'anchor': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw_deg': 0.0}, 'burial_m': 0.0, 'boxes': out}, (lo_x, hi_x, lo_y, hi_y)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--input', required=True, help='playground_boxes.json from ue_to_red.py')
    ap.add_argument('--output', required=True)
    a = ap.parse_args()
    with open(a.input, encoding='utf-8') as f:
        layout, ext = build(json.load(f))
    os.makedirs(os.path.dirname(os.path.abspath(a.output)), exist_ok=True)
    with open(a.output, 'w', encoding='utf-8') as f:
        json.dump(layout, f, indent=1)
    print(f'boxes={len(layout["boxes"])} floor x {ext[0]:.1f}..{ext[1]:.1f} y {ext[2]:.1f}..{ext[3]:.1f}')


if __name__ == '__main__':
    main()
