"""Test stations for the playground: a standing point and facing for every obstacle worth testing.

Input: the full layout (make_full_layout.py) and the anchor it was built with.
Output: stations.json (world positions) and a CET mod (init.lua) with one teleport button per station.

Station kinds
  open_run      the floor point with the most free space around it (locomotion, turns, stops)
  vault_<h>     3 m in front of a traversable block of height h (vault / mantle / climb)
  block_<h>     3 m in front of a plain level block up to 2.5 m (step-up, ledge)
  platforms     next to the lowest platform of the platform field (jumps between platforms)
  bumpers       next to the bumper field
"""
import argparse
import json
import math
import os

CLEAR_M = 0.7          # free radius required around a standing point
APPROACH_M = 3.0       # distance from the box face
CLUSTER_M = 4.0        # blocks closer than this count as one obstacle


def yaw_of(b):
    if 'quat' in b:
        i, j, k, r = b['quat']
        return math.degrees(math.atan2(2 * (r * k + i * j), 1 - 2 * (j * j + k * k)))
    return b.get('yaw_deg', 0.0)


def local(b, x, y):
    a = math.radians(-yaw_of(b))
    dx, dy = x - b['center'][0], y - b['center'][1]
    return dx * math.cos(a) - dy * math.sin(a), dx * math.sin(a) + dy * math.cos(a)


def inside(b, x, y, pad):
    lx, ly = local(b, x, y)
    return abs(lx) <= b['half'][0] + pad and abs(ly) <= b['half'][1] + pad


def top(b):
    return b['center'][2] + b['half'][2]


def free(solid, x, y, pad=CLEAR_M):
    return not any(inside(b, x, y, pad) for b in solid)


def clearance(solid, x, y, step=0.5, limit=15.0):
    r = 0.0
    while r < limit and free(solid, x, y, r):
        r += step
    return r


def face_yaw(dx, dy):
    """Game yaw (deg) that looks along (dx, dy); yaw 0 = +Y, counter-clockwise."""
    return math.degrees(math.atan2(-dx, dy))


def front_point(b, solid, floor):
    best = None
    a = math.radians(yaw_of(b))
    for k in range(8):
        t = k * math.pi / 4
        lx, ly = math.cos(t), math.sin(t)
        # distance from the centre to the box surface along this local direction
        s = min(b['half'][0] / abs(lx) if abs(lx) > 1e-6 else 1e9, b['half'][1] / abs(ly) if abs(ly) > 1e-6 else 1e9)
        d = s + APPROACH_M
        dx, dy = lx * math.cos(a) - ly * math.sin(a), lx * math.sin(a) + ly * math.cos(a)
        x, y = b['center'][0] + dx * d, b['center'][1] + dy * d
        if not inside(floor, x, y, -1.0) or not free(solid, x, y):
            continue
        c = clearance(solid, x, y)
        if best is None or c > best[0]:
            best = (c, x, y, face_yaw(-dx, -dy))
    return best


def cluster(boxes):
    groups = []
    for b in boxes:
        for g in groups:
            if any(math.dist(b['center'][:2], o['center'][:2]) < CLUSTER_M for o in g):
                g.append(b)
                break
        else:
            groups.append([b])
    return groups


def build(layout, anchor):
    boxes = layout['boxes']
    floor = next(b for b in boxes if b['name'] == 'floor')
    solid = [b for b in boxes if b['name'] != 'floor' and top(b) > 0.15]
    st = []

    # open run: floor grid point with the most clearance
    best = None
    x0, y0 = floor['center'][0], floor['center'][1]
    hx, hy = floor['half'][0] - 3, floor['half'][1] - 3
    for i in range(-int(hx), int(hx) + 1, 2):
        for j in range(-int(hy), int(hy) + 1, 2):
            c = clearance(solid, x0 + i, y0 + j)
            if best is None or c > best[0]:
                best = (c, x0 + i, y0 + j)
    st.append({'name': 'open_run', 'x': best[1], 'y': best[2], 'z': 0.0, 'yaw': 0.0,
               'note': f'free radius {best[0]:.1f} m'})

    def obstacle_stations(prefix, cand):
        for g in sorted(cluster(cand), key=lambda g: max(top(b) for b in g)):
            b = max(g, key=top)
            p = front_point(b, solid, floor)
            if p:
                st.append({'name': f'{prefix}_{top(b):.1f}m', 'x': p[1], 'y': p[2], 'z': 0.0, 'yaw': p[3],
                           'note': f'{len(g)} block(s), top {top(b):.2f} m'})

    obstacle_stations('vault', [b for b in solid if 'Traversable' in b['name']])
    obstacle_stations('block', [b for b in solid if b['name'].startswith('LevelBlock.') or
                                (b['name'].startswith('LevelBlock') and 'Traversable' not in b['name'] and top(b) <= 2.5)])
    for kind, key in (('platforms', 'TransparentPlatform'), ('bumpers', 'BP_SimpleObstacle_Bumper')):
        grp = [b for b in solid if b['name'].startswith(key)] or [b for b in boxes if b['name'].startswith(key)]
        if grp:
            b = min(grp, key=top)
            p = front_point(b, solid, floor)
            if p:
                st.append({'name': kind, 'x': p[1], 'y': p[2], 'z': 0.0, 'yaw': p[3], 'note': f'{len(grp)} pieces'})

    # unique names, world coordinates (anchor yaw is 0 for this playground)
    seen = {}
    for s in st:
        n = seen.get(s['name'], 0)
        seen[s['name']] = n + 1
        if n:
            s['name'] += f'_{n + 1}'
        s['x'] = round(anchor[0] + s['x'], 2)
        s['y'] = round(anchor[1] + s['y'], 2)
        s['z'] = round(anchor[2] + s['z'] + 1.0, 2)
        s['yaw'] = round(s['yaw'], 1)
    return st


LUA = """-- Playground stations: teleport buttons in the CET overlay (test-only, generated by make_stations.py).
local stations = {
%s
}
local open = false
registerForEvent('onOverlayOpen', function() open = true end)
registerForEvent('onOverlayClose', function() open = false end)

local function go(s)
  local p = Game.GetPlayer()
  if p then Game.GetTeleportationFacility():Teleport(p, Vector4.new(s[2], s[3], s[4], 1), EulerAngles.new(0, 0, s[5])) end
end

function UTPPPlaygroundGo(name)
  for _, s in ipairs(stations) do if s[1] == name then go(s); return true end end
  return false
end

registerForEvent('onDraw', function()
  if not open then return end
  if ImGui.Begin('UTPP playground') then
    for _, s in ipairs(stations) do
      if ImGui.Button(s[1]) then go(s) end
      ImGui.SameLine(); ImGui.Text(s[6])
    end
  end
  ImGui.End()
end)
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--layout', required=True)
    ap.add_argument('--anchor', nargs=3, type=float, required=True, metavar=('X', 'Y', 'Z'))
    ap.add_argument('--out-json', required=True)
    ap.add_argument('--out-lua', required=True)
    a = ap.parse_args()
    with open(a.layout, encoding='utf-8') as f:
        st = build(json.load(f), a.anchor)
    for p in (a.out_json, a.out_lua):
        os.makedirs(os.path.dirname(os.path.abspath(p)), exist_ok=True)
    with open(a.out_json, 'w', encoding='utf-8') as f:
        json.dump(st, f, indent=1)
    rows = ',\n'.join("  {'%s', %.2f, %.2f, %.2f, %.1f, '%s'}" % (s['name'], s['x'], s['y'], s['z'], s['yaw'], s['note'])
                      for s in st)
    with open(a.out_lua, 'w', encoding='utf-8', newline='\n') as f:
        f.write(LUA % rows)
    for s in st:
        print(f"{s['name']:<16} {s['x']:9.2f} {s['y']:9.2f} {s['z']:7.2f} yaw {s['yaw']:6.1f}  {s['note']}")


if __name__ == '__main__':
    main()
