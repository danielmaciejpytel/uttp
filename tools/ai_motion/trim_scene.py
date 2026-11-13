import sys, numpy as np
s = np.load(sys.argv[1], allow_pickle=True); a, b = int(sys.argv[3]), int(sys.argv[4])
keep = {k: s[k] for k in s.files if k not in ('G', 'hips', 'pos')}
np.savez(sys.argv[2], G=s['G'][a:b], hips=s['hips'][a:b], **keep)
print('TRIM', sys.argv[2], b - a)
