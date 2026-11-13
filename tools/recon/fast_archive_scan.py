"""Fast read-only scan of Cyberpunk 2077 .archive (RDAR) file tables for given depot paths.

Each file record is identified by FNV-1a 64 of the lower-case depot path with backslashes,
so we only read the archive header and index (no decompression, no WolvenKit start-up per file).
Usage: python fast_archive_scan.py <paths.txt> <out.json> <archive_dir_or_file> [...]
"""
import json
import os
import struct
import sys

FNV_OFFSET, FNV_PRIME, MASK = 0xCBF29CE484222325, 0x100000001B3, (1 << 64) - 1


def fnv1a64(text):
    h = FNV_OFFSET
    for b in text.encode('utf-8'):
        h = ((h ^ b) * FNV_PRIME) & MASK
    return h


def depot_hash(path):
    return fnv1a64(path.strip().replace('/', '\\').lower())


def archive_hashes(path):
    with open(path, 'rb') as f:
        head = f.read(40)
        magic, version, index_pos, index_size = struct.unpack_from('<4sIQI', head, 0)
        if magic != b'RDAR':
            raise ValueError(f'not RDAR: {magic!r}')
        f.seek(index_pos)
        index = f.read(index_size)
    # Index: fileTableOffset u32, fileTableSize u32, crc u64, fileCount u32, segmentCount u32, depCount u32
    _, _, _, file_count, _, _ = struct.unpack_from('<IIQIII', index, 0)
    base = 28
    return {struct.unpack_from('<Q', index, base + i * 56)[0] for i in range(file_count)}, version, file_count


def main():
    paths = [l.strip() for l in open(sys.argv[1], encoding='utf-8') if l.strip()]
    want = {depot_hash(p): p for p in paths}
    archives = []
    for target in sys.argv[3:]:
        if os.path.isfile(target):
            archives.append(target)
        for root, _, files in os.walk(target):
            archives += [os.path.join(root, f) for f in files if f.lower().endswith('.archive')]
    result = {'paths': paths, 'archives': {}, 'errors': {}, 'scanned': len(archives)}
    for a in sorted(set(archives)):
        try:
            hashes, _, _ = archive_hashes(a)
        except Exception as e:  # report and continue; never stop the whole scan
            result['errors'][a] = str(e)
            continue
        hits = sorted(want[h] for h in hashes & set(want))
        if hits:
            result['archives'][a] = hits
    json.dump(result, open(sys.argv[2], 'w', encoding='utf-8'), indent=2)
    by_path = {}
    for a, hs in result['archives'].items():
        for h in hs:
            by_path.setdefault(h, []).append(os.path.basename(a))
    print(f'scanned {len(archives)} archives, {len(result["archives"])} with hits, {len(result["errors"])} errors')
    for p in paths:
        hits = by_path.get(p, [])
        print(f'{len(hits)}\t{p}\t{" | ".join(hits)}')


if __name__ == '__main__':
    main()
