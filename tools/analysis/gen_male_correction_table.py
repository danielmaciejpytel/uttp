"""Writes the kMaleCorrectionTable of src/mm/MaleBodyCorrection.hpp from a corrections JSON (male_body_correction_eval.py format).

python -I gen_male_correction_table.py <corr.json> [<MaleBodyCorrection.hpp>]
Family order = enum MaleCorrFamily. Families missing in the JSON get zeros (no correction).
"""
import json
import os
import re
import sys

ORDER = [('StandRifle', 'stand_Rifle'), ('StandP1H', 'stand_P1H'), ('StandP2H', 'stand_P2H'), ('CrouchRifle', 'crouch_Rifle'),
         ('CrouchP1H', 'crouch_P1H'), ('CrouchP2H', 'crouch_P2H'), ('Loco', 'loco')]
corr = json.load(open(sys.argv[1], encoding='utf-8'))
path = sys.argv[2] if len(sys.argv) > 2 else os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'plugin', 'UTPP', 'src', 'mm', 'MaleBodyCorrection.hpp')
f = lambda v: "%.1ff" % v  # noqa: E731
rows = []
for cpp, key in ORDER:
    d = corr['families'].get(key) or {'spine3': [0, 0, 0], 'clavL': [0, 0, 0], 'clavR': [0, 0, 0], 'rollL': 0, 'rollR': 0}
    rows.append('    /*%-11s*/ {{%s}, {{%s}, {%s}}, {%s, %s}},' % (
        cpp, ', '.join(f(x) for x in d['spine3']), ', '.join(f(x) for x in d['clavL']), ', '.join(f(x) for x in d['clavR']), f(d['rollL']), f(d['rollR'])))
s = open(path, encoding='utf-8').read()
new = re.sub(r'(inline constexpr MaleCorrectionSet kMaleCorrectionTable\[kMaleCorrFamilyCount\] = \{\n).*?(\n\};)', lambda m: m.group(1) + '\n'.join(rows) + m.group(2), s, flags=re.S)
assert new != s or all('0.0f, 0.0f, 0.0f' in r for r in rows)
open(path, 'w', encoding='utf-8').write(new)
print('\n'.join(rows))
