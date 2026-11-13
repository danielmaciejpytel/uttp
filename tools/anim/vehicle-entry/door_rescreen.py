"""Re-evaluate prescreen GLBs (door_prescreen.py output, name ps_p?a_<class>_<seat>_o<out>_b<back>.anims.glb) with the current door model
(no rebuild): python -I door_rescreen.py <scratch dir> <class> [<class> ...]. Prints the same line format as door_prescreen.py."""
import glob
import os
import re
import sys
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import check_door_clearance as C  # noqa: E402


def one(f):
    m = re.match(r'ps_p([wm])a_(\w+?)_(driver|passenger_r)_o(\d+)_b(n?)(\d+)\.anims\.glb', os.path.basename(f))
    g, cls, seat, out, neg, back = m.groups()
    out, back = int(out) / 100.0, (-1 if neg else 1) * int(back) / 100.0
    mod = C.Model(f, cls, seat)  # door_timing.json v_lag_s
    sv = C.solve(mod)
    ca = sv.get('close_after_s')
    ra = C.analyse(mod, ca) if ca else None
    rb = C.analyse(C.Model(f, cls, seat, lag=0.0), ca) if ca else None  # robust: also with V on the clip clock (lag 0)
    ph, allok = {}, bool(ra and rb and ra['door_clearance_ok'] and rb['door_clearance_ok'])
    for p in (ra['phases'] if ra else {}):
        va, vb = ra['phases'][p], rb['phases'][p] if p in rb['phases'] else ra['phases'][p]
        ph[p] = dict(clearance_cm=min(va['clearance_cm'], vb['clearance_cm']), ok=va['ok'] and vb['ok'])
    r = dict(door_clearance_ok=allok)
    s = ' '.join('%s %.1f%s' % (p[:6], v['clearance_cm'], '' if v['ok'] else '!') for p, v in ph.items())
    return '%s p%sa %s out %.2f back %.2f | %s | close_after %s | start err 0.00 m | ok=%s | start []' % (cls, g, seat, out, back, s, ca, bool(r and r['door_clearance_ok']))


def main():
    d = sys.argv[1]
    files = []
    for c in sys.argv[2:]:
        files += sorted(glob.glob(os.path.join(d, 'ps_p?a_%s_*.anims.glb' % c)))
    with ThreadPoolExecutor(6) as ex:
        for line in ex.map(one, files):
            print(line, flush=True)


if __name__ == '__main__':
    main()
