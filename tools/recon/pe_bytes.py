"""Read raw bytes at RVAs from a PE file on disk (read-only), for hook signature checks.

Usage: python pe_bytes.py <exe> <count> <rva> [rva ...]
"""
import struct
import sys

exe, count = sys.argv[1], int(sys.argv[2])
data = open(exe, 'rb').read()
pe = struct.unpack_from('<I', data, 0x3C)[0]
nsec = struct.unpack_from('<H', data, pe + 6)[0]
opt_size = struct.unpack_from('<H', data, pe + 20)[0]
image_base = struct.unpack_from('<Q', data, pe + 24 + 24)[0]
sec = pe + 24 + opt_size
sections = []
for i in range(nsec):
    o = sec + i * 40
    name = data[o:o + 8].rstrip(b'\0').decode()
    vsize, va, rsize, raw = struct.unpack_from('<IIII', data, o + 8)
    sections.append((name, va, max(vsize, rsize), raw))
print(f'imageBase=0x{image_base:X}')
for r in sys.argv[3:]:
    rva = int(r, 16)
    for name, va, size, raw in sections:
        if va <= rva < va + size:
            off = raw + rva - va
            b = data[off:off + count]
            print(f'RVA 0x{rva:X} ({name}, file 0x{off:X}): {b.hex(" ").upper()}')
            break
    else:
        print(f'RVA 0x{rva:X}: not in any section')
