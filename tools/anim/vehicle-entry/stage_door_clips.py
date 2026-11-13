"""Assemble the 24 door-timing clips of a retuned set from several build_car.ps1 runs into one staging run dir (pack_car_workspot.ps1 -AnimsRun
accepts it like a run) and collect the door-close fragments next to it.

  python -I stage_door_clips.py <out dir> <spec> [<spec> ...]
    <spec> = <clip>=<run dir>                  one clip (clip = utpp_car_p?a_<class>_<seat>)
           | <class>=<run dir>                 the 4 clips of that class from that run
           | <run dir>:<class>[,<class>...]    the 4 clips of each listed class from that run (run dirs may contain 'D:' - only the LAST ':' splits)
  --frag <dir>:<class|clip>[,...]             take the door-close fragments of those classes from <dir> (door-close-<clip>.json) instead of the run dir - the unchanged classes of a
                                               partial rebuild keep their shipped fragments (pk-door4-anims/door-fragments-v2); the .anims still come from the run dirs
Refuses an existing <out dir>, a missing .anims / fragment, a clip or class given twice, an unknown class, anything but exactly 24 clips (6 classes x
2 seats x 2 genders) and any fragment that gen_door_timing.py --merge would reject (not feasible, door verdict not PASS, LATE, wrong clip / class / seat /
gender, stale vanilla close, solved with a weaker lag set / margin / window / tolerance or without the rear gate) and any fragment whose clip run dir (or recorded
clip sha256) is not the staged run: nothing is copied before every fragment passed.
"""
import hashlib
import json
import os
import shutil
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gen_door_timing as G  # noqa: E402

CLASSES = ['standard2', 'standard25', 'standard3', 'sport1', 'sport2', 'utility']


def parse(args):
    """-> {clip name: run dir}; refuses a clip / class given twice."""
    clips, given = {}, {}

    def add(clip, run, origin):
        if clip in clips:
            raise SystemExit('twice: %s (%s and %s)' % (clip, given[clip], origin))
        clips[clip] = run
        given[clip] = origin

    for arg in args:
        if '=' in arg:
            k, run = arg.split('=', 1)
            specs = [(k, run)]
        else:
            run, _, cs = arg.rpartition(':')
            if not run or not cs:
                raise SystemExit('bad spec (expected <clip>=<run>, <class>=<run> or <run>:<class>[,<class>]): ' + arg)
            specs = [(c, run) for c in cs.split(',')]
            seen = set()
            for c, _r in specs:
                if c in seen:
                    raise SystemExit('class twice in one spec: %s (%s)' % (c, arg))
                seen.add(c)
        for k, run in specs:
            if k.startswith('utpp_car_'):
                add(k, run, arg)
            elif k in CLASSES:
                for g in ('pwa', 'pma'):
                    for s in ('driver', 'passenger_r'):
                        add('utpp_car_%s_%s_%s' % (g, k, s), run, arg)
            else:
                raise SystemExit('unknown class / clip: %s (%s)' % (k, arg))
    return clips


def main():
    out = sys.argv[1]
    if os.path.exists(out):
        raise SystemExit('exists: ' + out)
    args, fragdir = [], {}
    it = iter(sys.argv[2:])
    for x in it:
        if x == '--frag':
            d, _, cs = next(it).rpartition(':')
            for c in cs.split(','):
                if c not in CLASSES and not c.startswith('utpp_car_'):
                    raise SystemExit('--frag: unknown class / clip ' + c)
                fragdir[c] = d
        else:
            args.append(x)
    clips = parse(args)
    exp = ['utpp_car_%s_%s_%s' % (g, c, s) for c in CLASSES for g in ('pwa', 'pma') for s in ('driver', 'passenger_r')]
    if sorted(clips) != sorted(exp):
        raise SystemExit('clip set differs: missing %s extra %s' % (sorted(set(exp) - set(clips)), sorted(set(clips) - set(exp))))
    tj = json.load(open(G.TIMING, encoding='utf-8'))
    jobs = []
    for n, run in sorted(clips.items()):
        a = os.path.join(run, 'utpp_vehicle_car', 'utpp', 'vehicle', n + '.anims')
        cls_of = n.split('_')[3]
        f = os.path.join(fragdir.get(n, fragdir.get(cls_of, run)), 'door-close-%s.json' % n)
        for p in (a, f):
            if not os.path.exists(p):
                raise SystemExit('missing ' + p)
        fr = json.load(open(f, encoding='utf-8'))
        c, s, g = G.validate_fragment(fr, f, tj)  # feasible, verdict PASS, not LATE, class / seat / gender, vanilla close
        if 'utpp_car_%s_%s_%s' % (g, c, s) != n or os.path.basename(fr.get('clip', '')) != n + '.anims.glb':
            raise SystemExit('fragment %s belongs to %s/%s/%s (clip %s), not to %s' % (f, c, s, g, os.path.basename(fr.get('clip', '')), n))
        # the fragment must have been solved on the clip that is staged: its clip path is <run>/roundtrip/<clip>.anims.glb, the run dir must be the staged one
        # (also with --frag); where the fragment stores the sha256 of its clip and the staged run still has that roundtrip glb, the hash must match as well
        frun = os.path.normcase(os.path.normpath(os.path.dirname(os.path.dirname(fr.get('clip', '')))))
        if frun != os.path.normcase(os.path.normpath(run)):
            raise SystemExit('fragment %s was solved on %s, but the staged .anims comes from the run %s (re-solve on the staged clip)' % (f, fr.get('clip'), run))
        rt = os.path.join(run, 'roundtrip', n + '.anims.glb')
        if fr.get('clip_sha256') and os.path.exists(rt):
            h = hashlib.sha256(open(rt, 'rb').read()).hexdigest().upper()
            if h != fr['clip_sha256'].upper():
                raise SystemExit('fragment %s: clip sha256 %s differs from %s (%s)' % (f, fr['clip_sha256'], rt, h))
        jobs.append((n, run, a, f))
    dep = os.path.join(out, 'utpp_vehicle_car', 'utpp', 'vehicle')
    fr_dir = os.path.join(out, 'door-fragments')
    os.makedirs(dep)
    os.makedirs(fr_dir)
    for n, run, a, f in jobs:
        shutil.copy2(a, dep)
        shutil.copy2(f, fr_dir)
        with open(os.path.join(out, 'sources.txt'), 'a', encoding='ascii') as t:
            t.write('%s  %s\n' % (n, run))
    print('staged %d clips into %s' % (len(clips), out))


if __name__ == '__main__':
    main()
