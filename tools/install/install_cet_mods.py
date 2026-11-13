"""Install selected UTPP CET mod folders (our own files) into the game with backup of existing files."""
import hashlib, json, os, shutil, subprocess, sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))   # the tools folder
from utpp_paths import game_dir

R = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
SRC = os.path.join(R, r'Source\plugin\UTPP\cet')
DST = os.path.join(game_dir(), r'bin\x64\plugins\cyber_engine_tweaks\mods')
mods = sys.argv[1:]
h = lambda p: hashlib.sha256(open(p, 'rb').read()).hexdigest()
assert 'Cyberpunk2077.exe' not in subprocess.run(['tasklist', '/FI', 'IMAGENAME eq Cyberpunk2077.exe'],
                                                  capture_output=True, text=True).stdout, 'game is running'
out = []
for m in mods:
    for n in sorted(os.listdir(os.path.join(SRC, m))):
        if not n.endswith('.lua'):
            continue
        src, dst = os.path.join(SRC, m, n), os.path.join(DST, m, n)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        if os.path.exists(dst):
            if h(dst) == h(src):
                out.append({'path': f'{m}/{n}', 'sha256': h(dst), 'unchanged': True})
                continue
            bk = os.path.join(R, r'Backups\2026-10-07', f'cet-{m}-before', n)
            os.makedirs(os.path.dirname(bk), exist_ok=True)
            shutil.copy2(dst, bk)
            os.remove(dst)
        shutil.copyfile(src, dst)
        out.append({'path': f'{m}/{n}', 'sha256': h(dst)})
p = os.path.join(R, r'Research\inventory', f'utpp-cet-{"-".join(mods)}-install-2026-10-07.json')
json.dump(out, open(p, 'w'), indent=1)
for o in out:
    print(('=' if o.get('unchanged') else '+'), o['sha256'][:8], o['path'])
