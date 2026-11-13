"""Read flat values of the retail TweakDB binary (read only), by record property name.

Layout (probed on game 2.5 r6\\cache\\tweakdb.bin): header u32[9]; type table at 0x24 with 20-byte entries
(u64 fnv1a-64 of the type name, u32 valueCount, u32 keyCount, u32 offset); block at offset:
u32 nvalues, values (fixed size per scalar type), u32 nkeys, (u64 id, u32 valueIndex)[nkeys].
A flat id is (crc32(name) | len(name) << 32) as in TweakDBID, stored as 40-bit in the key table.
Array types use a different block layout and are not decoded here.

Usage:  python tweakdb_flat_reader.py [--db PATH] NAME [NAME ...]
        NAME is a full flat name, e.g. FistsAttacks.FistsComboAttack1.attackEffectDelay;
        a trailing '*' is not supported (names are hashed). Prints type and value, or <absent>.
"""
from __future__ import annotations

import argparse
import struct
import sys
import zlib
from pathlib import Path

GAME = Path(r'E:\Steam Library\steamapps\common\Cyberpunk 2077')
DEFAULT_DB = GAME / 'r6' / 'cache' / 'tweakdb.bin'
EP1_DB = GAME / 'r6' / 'cache' / 'tweakdb_ep1.bin'

# scalar type name -> (struct format, element size)
SCALARS = {
    'Float': ('<f', 4), 'Int32': ('<i', 4), 'Bool': ('<?', 1), 'Uint8': ('<B', 1), 'Int16': ('<h', 2),
    'Uint16': ('<H', 2), 'Uint32': ('<I', 4), 'Uint64': ('<Q', 8), 'Int64': ('<q', 8),
    'TweakDBID': ('<Q', 8), 'Vector3': ('<3f', 12), 'Vector2': ('<2f', 8),
    'Quaternion': ('<4f', 16), 'EulerAngles': ('<3f', 12),
}


def fnv1a64(text: str) -> int:
    h = 0xCBF29CE484222325
    for c in text.encode():
        h ^= c
        h = (h * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return h


def tweakdb_id(name: str) -> int:
    return (zlib.crc32(name.encode()) & 0xFFFFFFFF) | (len(name) << 32)


class FlatDb:
    def __init__(self, path: Path = DEFAULT_DB):
        self.data = Path(path).read_bytes()
        self.types = {}
        count = struct.unpack_from('<I', self.data, 0x20)[0]
        by_hash = {fnv1a64(n): n for n in list(SCALARS) + ['String', 'LocKey', 'raRef:CResource']}
        for i in range(count):
            h, nv, nk, off = struct.unpack_from('<QIII', self.data, 0x24 + 20 * i)
            self.types[by_hash.get(h, 'unknown_%016x' % h)] = (nv, nk, off)
        self._maps = {}

    def _scalar_map(self, tname):
        if tname in self._maps:
            return self._maps[tname]
        nv, nk, off = self.types[tname]
        fmt, size = SCALARS[tname]
        p = off
        n = struct.unpack_from('<I', self.data, p)[0]
        p += 4
        if n != nv:
            raise ValueError('value count mismatch for %s: %d != %d' % (tname, n, nv))
        values_at = p
        p += size * nv
        k = struct.unpack_from('<I', self.data, p)[0]
        p += 4
        if k != nk:
            raise ValueError('key count mismatch for %s' % tname)
        keys = {}
        for j in range(nk):
            tid, idx = struct.unpack_from('<QI', self.data, p + 12 * j)
            keys[tid & 0xFFFFFFFFFF] = idx
        self._maps[tname] = (values_at, fmt, size, keys)
        return self._maps[tname]

    def get(self, name: str):
        """Return (typeName, value) or None. Searches all decoded scalar types."""
        tid = tweakdb_id(name)
        for tname in SCALARS:
            if tname not in self.types:
                continue
            values_at, fmt, size, keys = self._scalar_map(tname)
            idx = keys.get(tid)
            if idx is not None:
                v = struct.unpack_from(fmt, self.data, values_at + size * idx)
                return tname, (v[0] if len(v) == 1 else v)
        return None


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--db', default=str(DEFAULT_DB))
    ap.add_argument('names', nargs='+')
    args = ap.parse_args(argv)
    db = FlatDb(Path(args.db))
    for n in args.names:
        r = db.get(n)
        print('%s = %s' % (n, '<absent>' if r is None else '%s %r' % (r[1], r[0]) if False else '%s (%s)' % (r[1], r[0])))
    return 0


if __name__ == '__main__':
    sys.exit(main())
