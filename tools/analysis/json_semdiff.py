"""Semantic diff of two WolvenKit JSON files: lists every differing leaf path (a vs b).
usage: json_semdiff.py a.json b.json [--max N]   (exit 0 always; prints diff count)"""
import json, sys


def walk(a, b, path, out):
    if type(a) != type(b):
        out.append((path, a, b)); return
    if isinstance(a, dict):
        for k in sorted(set(a) | set(b)):
            if k not in a: out.append((path + '/' + k, '<missing>', b[k]))
            elif k not in b: out.append((path + '/' + k, a[k], '<missing>'))
            else: walk(a[k], b[k], path + '/' + k, out)
    elif isinstance(a, list):
        if len(a) != len(b): out.append((path + '[len]', len(a), len(b)))
        for i, (x, y) in enumerate(zip(a, b)): walk(x, y, '%s[%d]' % (path, i), out)
    elif a != b:
        out.append((path, a, b))


def main():
    a = json.load(open(sys.argv[1], encoding='utf-8'))
    b = json.load(open(sys.argv[2], encoding='utf-8'))
    mx = int(sys.argv[sys.argv.index('--max') + 1]) if '--max' in sys.argv else 40
    out = []; walk(a, b, '', out)
    print('DIFFS', len(out))
    for p, x, y in out[:mx]: print(p, '|', str(x)[:80], '->', str(y)[:80])


if __name__ == '__main__': main()
