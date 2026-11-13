import ctypes, subprocess, sys, time

class LII(ctypes.Structure):
    _fields_ = [("cbSize", ctypes.c_uint), ("dwTime", ctypes.c_uint)]


def idle_s():
    l = LII(); l.cbSize = ctypes.sizeof(LII)
    ctypes.windll.user32.GetLastInputInfo(ctypes.byref(l))
    return (ctypes.windll.kernel32.GetTickCount() - l.dwTime) / 1000.0


need = float(sys.argv[1])
t0 = time.time()
while idle_s() < need:
    if time.time() - t0 > 3 * 3600:
        print("gave up waiting for idle"); sys.exit(2)
    time.sleep(5)
print("idle %.0f s, starting night.py" % idle_s(), flush=True)
sys.exit(subprocess.call([sys.executable, "-I", "-B"] + sys.argv[2:]))
