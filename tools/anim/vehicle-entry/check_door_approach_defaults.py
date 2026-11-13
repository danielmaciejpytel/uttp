"""Consistency check: the built-in fallback constants of door_approach.py (_DEFAULTS, used only when VehicleTransition.hpp cannot be opened) equal the header.

Pure text comparison (no numpy / gen_door_timing import). Exit code 1 on any mismatch or a missing constant.
    python -I check_door_approach_defaults.py
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
HPP = os.path.normpath(os.path.join(HERE, '..', '..', '..', 'plugin', 'UTPP', 'src', 'mm', 'VehicleTransition.hpp'))
SRC = os.path.join(HERE, 'door_approach.py')


def main():
    src = open(SRC, encoding='utf-8').read()
    hpp = open(HPP, encoding='utf-8').read()
    names_block = re.search(r'_NAMES = \((.*?)\)\n_DEFAULTS', src, re.S)
    defaults_block = re.search(r'_DEFAULTS = dict\((.*?)\)(?:\s*#[^\n]*)?\n\n', src, re.S)
    if not names_block or not defaults_block:
        print('FAIL cannot find _NAMES / _DEFAULTS in door_approach.py')
        return 1
    names = re.findall(r"'(k\w+)'", names_block.group(1))
    defaults = {k: float(v) for k, v in re.findall(r'(k\w+)=([-0-9.eE+]+)', defaults_block.group(1))}
    bad = 0
    for n in names:
        m = re.search(r'constexpr float %s\s*=\s*([-0-9.eE+]+)f' % n, hpp)
        if not m:
            print('FAIL %s: not found in VehicleTransition.hpp' % n)
            bad += 1
        elif n not in defaults:
            print('FAIL %s: no fallback default in door_approach.py' % n)
            bad += 1
        elif abs(float(m.group(1)) - defaults[n]) > 1e-9:
            print('FAIL %s: fallback %s, header %s' % (n, defaults[n], m.group(1)))
            bad += 1
    print('check_door_approach_defaults: %d constants, %d mismatched' % (len(names), bad))
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
