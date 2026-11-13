"""Rig signature of a helper bundle, as compiled into the DLL (src/mm/BodyHelperAdmission.hpp, ComputeRigSignature).

The signature covers only what the bundle's plan reads and writes: the bones named in binding.dat (CORE, OWNED, CHILD, ANCHOR) plus all of
their ancestors, as (name hash, parent name hash) pairs sorted by name hash and hashed with FNV-1a 64 (each hash as 8 little-endian bytes,
root parent = 0). Bone indices, bone order and every other bone of the rig (hair, accessories, garment dangle bones) do not matter.

usage: python -I helper_rig_signature.py <merged_rig.json> <binding.dat>
  merged_rig.json: {"hashes": ["<hex>", ...], "parents": [int, ...]} (rig header of a utpppose recording, see make_helper_bundle_fixture.py)
prints: rig bones, closure bones, closure fingerprint (the values for the Approved table entry)
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
    for line in open(path, encoding='ascii').read().splitlines():
        parts = line.split()
        if parts and parts[0] in ('CORE', 'OWNED', 'CHILD', 'ANCHOR'):
            roots.append(int(parts[1], 16))
    return roots


def main():
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    rig = json.load(open(sys.argv[1], encoding='utf-8'))
    hashes = [int(h, 16) for h in rig['hashes']]
    parents = [int(p) for p in rig['parents']]
    count, fingerprint = closure_signature(hashes, parents, binding_roots(sys.argv[2]))
    print('rig bones %d, closure bones %d, closure fingerprint 0x%016xull' % (len(hashes), count, fingerprint))


if __name__ == '__main__':
    main()
