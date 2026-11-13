"""Synthetic .feed scenarios for the foot-lock replay (same 60 Hz format as tools/replay/extract_feeds.py).
Columns: t dt vx vy vz mvx mvy yawDeg camDeg loco psm trans baseClip baseFlags baseCost baseMicros step
Capsule model: velocity chases the stick target with a bounded acceleration, the body yaw follows the velocity heading (360 deg/s)."""
import math, os, sys

OUT = sys.argv[1]
os.makedirs(OUT, exist_ok=True)
DT = 1.0 / 60.0


def scenario(name, loco, phases, accel=14.0, yaw_rate=360.0):
    """phases: list of (seconds, stick_angle_deg or None, stick_len, speed, label). stick angle in camera space: 0 = forward(+y), +90 = right."""
    x = y = 0.0
    vx = vy = 0.0
    yaw = 0.0  # deg, forward(yaw) = (-sin, cos)
    t = 0.0
    rows = []
    for secs, ang, ln, spd, label in phases:
        n = int(round(secs / DT))
        for _ in range(n):
            if ang is None or ln < 0.01:
                mvx = mvy = 0.0
                tvx = tvy = 0.0
            else:
                a = math.radians(ang)
                mvx = math.sin(a) * ln
                mvy = math.cos(a) * ln
                tvx = math.sin(a) * spd
                tvy = math.cos(a) * spd
            dvx, dvy = tvx - vx, tvy - vy
            d = math.hypot(dvx, dvy)
            lim = accel * DT
            if d > lim:
                dvx, dvy = dvx / d * lim, dvy / d * lim
            vx += dvx
            vy += dvy
            sp = math.hypot(vx, vy)
            if sp > 0.5 and ang is not None:
                heading = math.degrees(math.atan2(-vx, vy))
                dy = (heading - yaw + 180.0) % 360.0 - 180.0
                step = yaw_rate * DT
                yaw += max(-step, min(step, dy))
            x += vx * DT
            y += vy * DT
            t += DT
            rows.append((t, vx, vy, mvx, mvy, yaw, label))
    with open(os.path.join(OUT, '20261106-000000_%s.feed' % name), 'w') as f:
        f.write('# feed v1 session=20261106-000000 scenario=%s log=synthetic hz=60\n' % name)
        f.write('# cols t dt vx vy vz mvx mvy yawDeg camDeg loco psm trans baseClip baseFlags baseCost baseMicros step\n')
        for (tt, vx_, vy_, mvx, mvy, yw, lab) in rows:
            f.write('%.4f %.5f %.3f %.3f 0.000 %.2f %.2f %.1f 0.0 %d 0 0 x 0 0.000 3 %s\n' % (tt, DT, vx_, vy_, mvx, mvy, yw, loco, lab))
    print(name, len(rows), 'frames')


# run (3.8 m/s) with 180 / 90 turns
scenario('run_turns_180_90', 1, [
    (1.0, None, 0, 0, 'idle'), (2.2, 0, 1.0, 3.8, 'run_f'), (2.2, 180, 1.0, 3.8, 'flick_180'), (2.0, 90, 1.0, 3.8, 'turn_r90'),
    (2.0, -90, 1.0, 3.8, 'turn_l90'), (2.0, 135, 1.0, 3.8, 'turn_r135'), (1.5, None, 0, 0, 'stop'), (1.0, None, 0, 0, 'idle')])
# sprint (7.5 m/s) with 180 turns
scenario('sprint_turns_180', 3, [
    (1.0, None, 0, 0, 'idle'), (3.5, 0, 1.0, 7.5, 'sprint_f'), (2.5, 180, 1.0, 7.5, 'flick_180'), (2.5, 90, 1.0, 7.5, 'turn_r90'),
    (2.0, None, 0, 0, 'stop'), (1.0, None, 0, 0, 'idle')])
# walk start / stop / turn
scenario('walk_start_stop_turn', 1, [
    (1.0, None, 0, 0, 'idle'), (2.5, 0, 0.45, 1.6, 'walk_f'), (1.2, None, 0, 0, 'stop'), (1.0, 90, 0.45, 1.6, 'walk_r'),
    (2.0, 180, 0.45, 1.6, 'walk_flick'), (1.5, None, 0, 0, 'stop'), (1.0, None, 0, 0, 'idle')])
# crouch walk with 180 turn
scenario('crouch_start_turn_stop', 2, [
    (1.0, None, 0, 0, 'idle'), (2.2, 0, 0.6, 1.5, 'crouch_f'), (2.2, 180, 0.6, 1.5, 'crouch_flick'), (1.5, None, 0, 0, 'stop'),
    (1.0, None, 0, 0, 'idle')])
# starts in all directions from idle, run
scenario('run_start_dirs', 1, [
    (1.0, None, 0, 0, 'idle'), (1.8, 0, 1.0, 3.8, 'start_f'), (1.2, None, 0, 0, 'stop'), (1.0, None, 0, 0, 'idle'),
    (1.8, 135, 1.0, 3.8, 'start_br'), (1.2, None, 0, 0, 'stop'), (1.0, None, 0, 0, 'idle'),
    (1.8, 180, 1.0, 3.8, 'start_b'), (1.2, None, 0, 0, 'stop'), (1.0, None, 0, 0, 'idle'),
    (1.8, -90, 1.0, 3.8, 'start_l'), (1.2, None, 0, 0, 'stop'), (1.0, None, 0, 0, 'idle')])
