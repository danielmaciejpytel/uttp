"""python -I fs_stage.py <src clip dir> <fixed clip dir> <dest dir>: dest = hard-link copy of src with the files of <fixed> swapped in as NEW files
(the source dir inodes are never written)."""
import os, shutil, sys
src, fixed, dest = sys.argv[1:4]
os.makedirs(dest, exist_ok=True)
n = 0
for f in os.listdir(src):
    d = os.path.join(dest, f)
    if os.path.exists(d):
        os.remove(d)
    os.link(os.path.join(src, f), d)
for f in os.listdir(fixed):
    if not f.endswith('.utppclip.json'):
        continue
    tmp = os.path.join(dest, f + '.tmp')
    shutil.copyfile(os.path.join(fixed, f), tmp)
    os.replace(tmp, os.path.join(dest, f))   # replaces the directory entry (hard link) with a new inode
    n += 1
print('linked', len(os.listdir(src)), 'replaced', n)
