"""Rig signature of a helper bundle, as compiled into the DLL (src/mm/BodyHelperAdmission.hpp, ComputeRigSignature).

The signature covers only what the bundle's plan reads and writes: the bones named in binding.dat (CORE, OWNED, CHILD, ANCHOR) plus all of
their ancestors, as (name hash, parent name hash) pairs sorted by name hash and hashed with FNV-1a 64 (each hash as 8 little-endian bytes,
root parent = 0). Bone indices, bone order and every other bone of the rig (hair, accessories, garment dangle bones) do not matter.

usage: python -I helper_rig_signature.py <merged_rig.json> <binding.dat> [--expect <closure bones> <fingerprint>]
  merged_rig.json: {"hashes": ["<hex>", ...], "parents": [int, ...]} (rig header of a utpppose recording, see make_helper_bundle_fixture.py)
prints: rig bones, closure bones, closure fingerprint (the values for the Approved table entry)
--expect: exit code 1 (bundle refused) when the closure differs from the given bone count and fingerprint (hex, 0x prefix optional)
"""
import json
import sys

FNV_OFFSET, FNV_PRIME, MASK = 0xcbf29ce484222325, 0x100000001b3, (1 << 64) - 1


def closure_signature(hashes, parents, roots):
    index = {}
    for i, h in enumerate(hashes):
        if h in index:
            raise SystemExit('duplicate bone name hash %016X' % h)
        index[h] = i
    marked = set()
    for root in roots:
        if root not in index:
            raise SystemExit('bundle bone %016X is not in the rig' % root)
        i = index[root]
        while i >= 0 and i not in marked:
            marked.add(i)
            i = parents[i]
    pairs = sorted((hashes[i], hashes[parents[i]] if parents[i] >= 0 else 0) for i in marked)
    x = FNV_OFFSET
    for name, parent in pairs:
        for value in (name, parent):
            for b in range(8):
                x ^= (value >> (8 * b)) & 0xFF
                x = (x * FNV_PRIME) & MASK
    return len(pairs), x


def binding_roots(path):
    roots = []
    with open(path, encoding='ascii') as f:
        text = f.read()
    for line in text.splitlines():
        parts = line.split()
        if parts and parts[0] in ('CORE', 'OWNED', 'CHILD', 'ANCHOR'):
            roots.append(int(parts[1], 16))
    return roots


def check(hashes, parents, roots, bones, fingerprint):
    """True when the bundle's closure in the rig has exactly the expected bone count and fingerprint (the DLL's rig check)."""
    count, value = closure_signature(hashes, parents, roots)
    return count == bones and value == fingerprint


def main():
    args = sys.argv[1:]
    expect = None
    if len(args) == 5 and args[2] == '--expect':
        expect = (int(args[3]), int(args[4], 16))
        args = args[:2]
    if len(args) != 2:
        raise SystemExit(__doc__)
    rig = json.load(open(args[0], encoding='utf-8'))
    hashes = [int(h, 16) for h in rig['hashes']]
    parents = [int(p) for p in rig['parents']]
    count, fingerprint = closure_signature(hashes, parents, binding_roots(args[1]))
    print('rig bones %d, closure bones %d, closure fingerprint 0x%016xull' % (len(hashes), count, fingerprint))
    if expect and (count, fingerprint) != expect:
        print('bundle refused: expected closure bones %d, fingerprint 0x%016xull' % expect)
        raise SystemExit(1)


if __name__ == '__main__':
    main()
