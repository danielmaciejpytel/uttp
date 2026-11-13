"""Stage 9: LISTING ONLY (no extraction) of NPC animation sets that could serve as third-person sources for the player states the
survey found deformed in TPP: takedowns / grapples, body carry, ladders, consumables, grenades, cyberware, hit / stagger reactions.

  python -I npc_sets.py [--game "<game folder>"]
WolvenKit.CLI `archive -l -r \\.anims$` over archive\\pc\\content and archive\\pc\\ep1 (read only; never archive\\pc\\mod)
-> Build\\work\\anim-survey-20261109\\npc_sets.txt (+ raw listing npc_sets_raw.txt) and the STAGE 9 block of report.txt.
"""
import collections
import os
import re
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import survey_common as SC  # noqa: E402
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from utpp_paths import game_dir  # noqa: E402

GAME = sys.argv[sys.argv.index('--game') + 1] if '--game' in sys.argv else game_dir()
WKIT = os.path.join(SC.ROOT, 'Tools', 'WolvenKit.CLI', 'WolvenKit.CLI.exe')
CATS = [('takedown / grapple / finisher', r'takedown|grapple|finisher|_sync|stealth_kill|choke'),
        ('body carry', r'carry|corpse|body_dump|bodydump|pickup_body|drop_body'),
        ('ladder', r'ladder'),
        ('consumable', r'consum|inhal|inject|drink|smok|heal|stim|food|(^|[\\_])eat'),
        ('grenade', r'grenade|throw'),
        ('cyberware', r'cyberware|mantis|strongarm|gorilla|monowire|launcher|sandevistan|berserk|optical|cw_'),
        ('hit / stagger / knockdown reaction', r'hit_?react|stagger|knockdown|impact|pain|hit_|_hit|death|dying|wounded|stun')]
PLAYER = re.compile(r'(\\|^)(pwa|pma|player)_|\\player\\|fpp', re.I)
SKIP = re.compile(r'\\(lipsync|localization|quest|scenes?|open_world|crowd_reactions)\\', re.I)   # scene / quest / lipsync specific


def main():
    raw = []
    for sub in ('content', 'ep1'):
        p = os.path.join(GAME, 'archive', 'pc', sub)
        r = subprocess.run([WKIT, 'archive', p, '-l', '-r', r'\.anims$'], capture_output=True, text=True, encoding='utf-8', errors='replace')
        raw += [(sub, ln) for ln in r.stdout.splitlines()]
        print(sub, 'exit', r.returncode, 'lines', len(r.stdout.splitlines()))
    with open(os.path.join(SC.WORK, 'npc_sets_raw.txt'), 'w', encoding='utf-8') as f:
        f.write('\n'.join('%s\t%s' % x for x in raw) + '\n')
    paths = collections.OrderedDict()
    for sub, ln in raw:
        m = re.search(r'([A-Za-z0-9_\\/.\-]+\.anims)\b', ln)
        if m:
            paths.setdefault(m.group(1).replace('/', '\\').lower(), sub)
    npc = {p: s for p, s in paths.items() if not PLAYER.search(p) and not SKIP.search(p)}
    rows, cnt = [], collections.Counter()
    for p, s in npc.items():
        name = p.rsplit('\\', 1)[-1]
        rigm = re.search(r'(man|woman)_(average|big|massive|child|fat|small)|\b(ma|wa|mb|mm|mf|wf)_', p)
        rig = rigm.group(0).strip('_') if rigm else '?'
        for c, rx in CATS:
            if re.search(rx, p):
                rows.append((c, rig, s, p))
                cnt[(c, rig)] += 1
                break
    rows.sort()
    with open(os.path.join(SC.WORK, 'npc_sets.txt'), 'w', encoding='utf-8') as f:
        f.write('category\trig_hint\tarchive_dir\tdepot_path\n')
        f.write('\n'.join('\t'.join(r) for r in rows) + '\n')
    L = ['listing only (WolvenKit archive -l over archive\\pc\\content + ep1, read only, nothing extracted): %d .anims in the archives, '
         '%d non-player, %d in the categories below -> %s\\npc_sets.txt (raw: npc_sets_raw.txt)' % (len(paths), len(npc), len(rows), SC.WORK),
         'rig hint = path token (man_average / woman_average = the NPC rigs closest to V; big / massive / child need retargeting).',
         'excluded: player sets, lipsync / localization / quest / scene / crowd-reaction folders. Per SET names only: NPC grenade throws,',
         'consumable uses and hit reactions also sit as CLIPS inside NPC combat / action sets (not visible in a set listing); NPC ladder',
         'sets were not found by name (only the sliding_ladder item set) - check clip names in the gameplay sets before planning.']
    for c, _ in CATS:
        sub = [r for r in rows if r[0] == c]
        by = collections.Counter(r[1] for r in sub)
        L.append('%-36s %4d sets  rigs: %s' % (c, len(sub), ', '.join('%s %d' % kv for kv in by.most_common(6))))
        pref = [r for r in sub if r[1] in ('man_average', 'woman_average', 'ma', 'wa')] or sub
        for r in pref[:6]:
            L.append('    %s  (%s)' % (r[3], r[2]))
    txt = '\n'.join(L)
    print(txt)
    rep = open(SC.REPORT, encoding='utf-8').read()          # replace an earlier STAGE 9 block of this script (re-run), keep the rest
    i = rep.find('\n==== STAGE 9 ')
    if i >= 0:
        j = rep.find('\n==== ', i + 1)
        with open(SC.REPORT, 'w', encoding='utf-8') as f:
            f.write(rep[:i] + (rep[j:] if j >= 0 else '\n'))
    SC.report('STAGE 9 NPC ANIMATION SETS (listing only) as third-person sources', txt)


if __name__ == '__main__':
    main()
