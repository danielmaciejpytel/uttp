"""Synthetic 60 Hz input feeds of the locomotion comparison films (2026-11-09), .feed format of tools/replay/extract_feeds.py /
tools/replay/footlock/gen_synth_feeds.py (columns t dt vx vy vz mvx mvy yawDeg camDeg loco psm trans baseClip baseFlags baseCost baseMicros step).

  python -I loco_feeds.py      -> Build\\work\\loco-films-20261109\\feeds\\loco_film1.feed, loco_film3.feed (+ .json: segments)
Capsule model (parameters measured in the recorded in-game feed dumps, loco_feed_stats.py over utpp-feed-*.txt):
  full-stick speeds stand 3.84 m/s, crouch 2.73 m/s, sprint 8.22 m/s; walk = UTPP walk speed 1.6 m/s (mm.walkSpeed) at stick 0.42;
  velocity chases the stick target: speeding up at most ACCEL (p50 of the dumps 3.5 m/s2), slowing down / turning at most BRAKE
  (p50 9.3 m/s2); the actor yaw follows the velocity heading at YAW_RATE deg/s once the speed is above 0.5 m/s (as gen_synth_feeds.py;
  dumps: p90 325 deg/s while turning).  Camera yaw 0, stick in camera space (x right, y forward).
Slide (Film 3): the rows of a REAL in-game slide episode are copied (utpp-feed-2026-10-15-05-32-55.txt, t 482.61..486.40: sprint 8.25 m/s ->
  slide (PSM LocomotionDetailed 5, loco 0) 3.15 s, speed impulse to 10.8 m/s then braking ~2.7 m/s2 -> crouch (loco 2) at 2.5 m/s):
  per row the speed along the current heading, loco and the PSM detail field + kPsmDetailed bit (raw & 0x00FF0040; the weapon bits of that
  session are cleared: V is unarmed in the whole feed), resampled to 60 Hz.  Entry from sprint (all recorded slides start from sprint).
Writes only under Build\\work\\loco-films-20261109\\feeds.
"""
import json
import math
import os

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..'))
OUT = os.path.join(ROOT, 'Build', 'work', 'loco-films-20261109', 'feeds')
DT = 1.0 / 60.0
ACCEL, BRAKE, YAW_RATE = 3.5, 9.3, 360.0
WALK, RUN, CROUCH, SPRINT = 1.6, 3.84, 2.73, 8.22
WALK_STICK = 0.42
SLIDE_SRC = os.path.join(ROOT, 'Research', 'logs', 'game', 'utpp-feed-2026-10-15-05-32-55.txt')
SLIDE_T0, SLIDE_T1 = 482.61, 486.40          # first slide row .. 0.6 s of the crouch after it


def real_slide():
    """rows of the recorded slide: (t rel, speed, loco, psm detail bits)."""
    out = []
    for ln in open(SLIDE_SRC, encoding='utf-8', errors='replace'):
        if not ln.strip() or ln[0] == '#':
            continue
        p = ln.split()
        t = float(p[0])
        if SLIDE_T0 - 1e-3 <= t <= SLIDE_T1:
            out.append((t, math.hypot(float(p[2]), float(p[3])), int(p[9]), int(p[10]) & 0x00FF0040))
    a = np.array(out, float)
    a[:, 0] -= a[0, 0]
    return a


class Capsule:
    def __init__(self):
        self.x = self.y = self.vx = self.vy = 0.0
        self.yaw = 0.0          # deg, forward(yaw) = (-sin, cos)
        self.t = 0.0
        self.rows = []
        self.segs = []

    def heading(self):
        return math.radians(self.yaw)

    def emit(self, mvx, mvy, loco, psm, label):
        self.x += self.vx * DT
        self.y += self.vy * DT
        self.t += DT
        self.rows.append((self.t, self.vx, self.vy, mvx, mvy, self.yaw, loco, psm, label))

    def turn_to_velocity(self):
        sp = math.hypot(self.vx, self.vy)
        if sp > 0.5:
            hd = math.degrees(math.atan2(-self.vx, self.vy))
            d = (hd - self.yaw + 180.0) % 360.0 - 180.0
            st = YAW_RATE * DT
            self.yaw += max(-st, min(st, d))

    def move(self, secs, ang, stick, speed, loco, label):
        """ang: stick angle in camera space (deg, 0 = forward +y, 90 = right +x) or None (released)."""
        self.segs.append(dict(label=label, t0=round(self.t, 4), secs=secs, ang=ang, stick=stick, speed=speed, loco=loco))
        for _ in range(int(round(secs / DT))):
            if ang is None or stick < 0.01:
                mvx = mvy = tvx = tvy = 0.0
            else:
                a = math.radians(ang)
                mvx, mvy = math.sin(a) * stick, math.cos(a) * stick
                tvx, tvy = math.sin(a) * speed, math.cos(a) * speed
            dvx, dvy = tvx - self.vx, tvy - self.vy
            d = math.hypot(dvx, dvy)
            slowing = dvx * self.vx + dvy * self.vy < 0.0 or math.hypot(tvx, tvy) < math.hypot(self.vx, self.vy)
            lim = (BRAKE if slowing else ACCEL) * DT
            if d > lim:
                dvx, dvy = dvx / d * lim, dvy / d * lim
            self.vx += dvx
            self.vy += dvy
            if ang is not None:
                self.turn_to_velocity()
            self.emit(mvx, mvy, loco, 0, label)

    def slide(self, label='slide'):
        """the recorded slide rows, speed along the current heading, stick forward (as recorded: mv = 0,1)."""
        a = real_slide()
        n = int(round(a[-1, 0] / DT)) + 1
        self.segs.append(dict(label=label, t0=round(self.t, 4), secs=round(n * DT, 4), src=os.path.basename(SLIDE_SRC), src_t=[SLIDE_T0, SLIDE_T1]))
        h = self.heading()
        fx, fy = -math.sin(h), math.cos(h)
        for k in range(n):
            tt = k * DT
            j = int(np.searchsorted(a[:, 0], tt, side='right') - 1)
            j = max(0, min(j, len(a) - 1))
            sp = float(np.interp(tt, a[:, 0], a[:, 1]))
            loco, psm = int(a[j, 2]), int(a[j, 3])
            self.vx, self.vy = fx * sp, fy * sp
            ang = math.degrees(math.atan2(fx, fy))       # stick along the heading (camera yaw 0)
            self.emit(math.sin(math.radians(ang)), math.cos(math.radians(ang)), loco, psm, label if psm else label + '_crouch')

    def write(self, name, desc):
        os.makedirs(OUT, exist_ok=True)
        p = os.path.join(OUT, name + '.feed')
        with open(p, 'w', newline='\n') as f:
            f.write('# feed v1 session=20261109-000000 scenario=%s log=synthetic hz=60 (Source/tools/films/loco_feeds.py)\n' % name)
            f.write('# cols t dt vx vy vz mvx mvy yawDeg camDeg loco psm trans baseClip baseFlags baseCost baseMicros step\n')
            for (tt, vx, vy, mvx, mvy, yw, loco, psm, lab) in self.rows:
                f.write('%.4f %.5f %.3f %.3f 0.000 %.3f %.3f %.2f 0.0 %d %d 0 x 0 0.000 3 %s\n' % (tt, DT, vx, vy, mvx, mvy, yw, loco, psm, lab))
        json.dump(dict(name=name, desc=desc, frames=len(self.rows), seconds=round(self.t, 3), dt=DT, accel=ACCEL, brake=BRAKE,
                       yaw_rate=YAW_RATE, speeds=dict(walk=WALK, run=RUN, crouch=CROUCH, sprint=SPRINT), segments=self.segs),
                  open(os.path.join(OUT, name + '.json'), 'w'), indent=1)
        print(name, len(self.rows), 'frames', round(self.t, 2), 's ->', p)


def film1():
    c = Capsule()
    c.move(1.2, None, 0, 0, 1, 'idle')
    c.move(2.6, 0, WALK_STICK, WALK, 1, 'walk_start')
    c.move(2.0, 90, WALK_STICK, WALK, 1, 'walk_turn90')
    c.move(2.2, -90, WALK_STICK, WALK, 1, 'walk_turn180')
    c.move(1.4, None, 0, 0, 1, 'walk_stop')
    c.move(2.4, 90, 1.0, RUN, 1, 'run_start180')
    c.move(2.0, 0, 1.0, RUN, 1, 'run_turn90')
    c.move(2.2, 180, 1.0, RUN, 1, 'run_turn180')
    c.move(1.5, None, 0, 0, 1, 'run_stop')
    c.move(3.0, 180, 1.0, SPRINT, 3, 'sprint_start')
    c.move(2.4, 90, 1.0, SPRINT, 3, 'sprint_turn90')
    c.move(2.6, -90, 1.0, SPRINT, 3, 'sprint_turn180')
    c.move(2.0, None, 0, 0, 1, 'sprint_stop')
    c.move(1.2, None, 0, 0, 1, 'idle_end')
    c.write('loco_film1', 'walk / run / sprint: starts, 90 and 180 deg turns, stops (stand, unarmed)')


def film3():
    c = Capsule()
    c.move(1.0, None, 0, 0, 1, 'idle')
    c.move(2.0, 0, 1.0, RUN, 1, 'run_start')
    c.move(2.2, 0, 1.0, SPRINT, 3, 'sprint')
    c.slide('slide')
    c.move(1.6, 0, 1.0, RUN, 1, 'slide_exit_run')
    c.move(1.5, None, 0, 0, 1, 'run_stop')
    c.move(1.0, None, 0, 0, 1, 'idle2')
    c.move(2.0, 90, 1.0, RUN, 1, 'run_start2')
    c.move(2.4, 90, 1.0, CROUCH, 2, 'run_to_crouch')
    c.move(2.0, 0, 1.0, CROUCH, 2, 'crouch_turn90')
    c.move(1.2, None, 0, 0, 2, 'crouch_stop')
    c.move(1.2, None, 0, 0, 2, 'crouch_idle')
    c.move(1.6, None, 0, 0, 1, 'stand_up')
    c.write('loco_film3', 'run -> sprint -> slide (recorded episode) -> exit; run -> crouch; crouch walk + 90 turn; crouch stop; stand up')


if __name__ == '__main__':
    film1()
    film3()
