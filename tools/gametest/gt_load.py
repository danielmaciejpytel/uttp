"""Machine load during a run, for the run record: a number measured in a game that shares the machine with compile jobs is
not comparable with one measured on a quiet machine (frame budget lines such as "feedP95us=371.5 budget=250").

LoadSampler runs in a background thread from start() to stop(): total CPU load every `period` s and, every `proc_every`
samples, the compiler / build processes that are running (names in COMPILER_NAMES). Method: psutil when it can be imported, else
`typeperf` for the CPU load and `tasklist` for the processes. Report only: HIGH_CPU_PERCENT is the mean load above which the
record carries a warning, it is printed with the result and never changes a verdict.
"""
from __future__ import annotations

import subprocess
import threading
import time

COMPILER_NAMES = ('cl.exe', 'clang-cl.exe', 'clang.exe', 'clang++.exe', 'link.exe', 'lld-link.exe', 'ninja.exe', 'msbuild.exe',
                  'cc1plus.exe', 'c1xx.exe', 'rustc.exe', 'cmake.exe', 'ld.exe')
HIGH_CPU_PERCENT = 60.0          # mean total CPU load over the scenario phase above which the run record warns
PERIOD_S = 2.0
PROC_EVERY = 5                   # process list every 5th sample (10 s)


def _have_psutil():
    try:
        import psutil  # noqa: F401
        return True
    except Exception:  # noqa: BLE001
        return False


def cpu_percent_typeperf(runner=subprocess.run) -> float | None:
    """Total CPU load through `typeperf` (one sample): the fallback when psutil is not importable."""
    try:
        out = runner(['typeperf', r'\Processor(_Total)\% Processor Time', '-sc', '1'], capture_output=True, text=True, timeout=15)
        for line in (out.stdout or '').splitlines():
            parts = [p.strip().strip('"') for p in line.split('","')]
            if len(parts) >= 2:
                try:
                    return float(parts[-1].strip('"'))
                except ValueError:
                    continue
    except Exception:  # noqa: BLE001
        pass
    return None


def compilers_tasklist(runner=subprocess.run) -> list[str]:
    try:
        out = runner(['tasklist', '/FO', 'CSV', '/NH'], capture_output=True, text=True, timeout=20)
    except Exception:  # noqa: BLE001
        return []
    names = []
    for line in (out.stdout or '').splitlines():
        n = line.split('","')[0].strip('"').lower()
        if n in COMPILER_NAMES:
            names.append(n)
    return names


class LoadSampler:
    def __init__(self, period: float = PERIOD_S, proc_every: int = PROC_EVERY, high_cpu: float = HIGH_CPU_PERCENT,
                 use_psutil: bool | None = None, cpu_probe=None, proc_probe=None):
        """cpu_probe() -> percent | None and proc_probe() -> [process names] replace the real probes (tests)."""
        self.period = period
        self.proc_every = max(1, proc_every)
        self.high_cpu = high_cpu
        self.use_psutil = _have_psutil() if use_psutil is None else use_psutil
        self.cpu_probe = cpu_probe
        self.proc_probe = proc_probe
        self.cpu: list[float] = []
        self.compilers_max = 0
        self.compiler_names: set[str] = set()
        self.cores = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.t0 = self.t1 = None

    # ---- probes
    def _cpu(self) -> float | None:
        if self.cpu_probe:
            return self.cpu_probe()
        if self.use_psutil:
            import psutil
            return float(psutil.cpu_percent(interval=None))
        return cpu_percent_typeperf()

    def _procs(self) -> list[str]:
        if self.proc_probe:
            return list(self.proc_probe())
        if self.use_psutil:
            import psutil
            return [p.info['name'].lower() for p in psutil.process_iter(['name']) if p.info.get('name')
                    and p.info['name'].lower() in COMPILER_NAMES]
        return compilers_tasklist()

    def sample(self, n: int) -> None:
        v = self._cpu()
        if v is not None:
            self.cpu.append(v)
        if n % self.proc_every == 0:
            names = self._procs()
            self.compilers_max = max(self.compilers_max, len(names))
            self.compiler_names.update(names)

    # ---- thread
    def start(self) -> 'LoadSampler':
        if self.use_psutil and not self.cpu_probe:
            import psutil
            psutil.cpu_percent(interval=None)     # primes the counter
            self.cores = psutil.cpu_count()
        self.t0 = time.time()
        self._stop.clear()

        def loop():
            n = 0
            while not self._stop.wait(self.period):      # the first reading comes after one period (psutil needs an interval)
                try:
                    self.sample(n)
                except Exception:  # noqa: BLE001 - a probe problem never touches the run
                    pass
                n += 1
        self._thread = threading.Thread(target=loop, name='load-sampler', daemon=True)
        self._thread.start()
        return self

    def stop(self) -> dict:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=5.0)
        self.t1 = time.time()
        return self.result()

    def result(self) -> dict:
        cpu = sorted(self.cpu)
        mean = sum(cpu) / len(cpu) if cpu else None
        res = {'method': 'psutil' if self.use_psutil and not self.cpu_probe else ('typeperf/tasklist' if not self.cpu_probe else 'probe'),
               'samples': len(cpu), 'cpu_mean_pct': None if mean is None else round(mean, 1),
               'cpu_p95_pct': None if not cpu else round(cpu[min(len(cpu) - 1, int(0.95 * len(cpu)))], 1),
               'cpu_max_pct': None if not cpu else round(cpu[-1], 1), 'cores': self.cores,
               'compiler_processes_max': self.compilers_max, 'compiler_names': sorted(self.compiler_names),
               'high_cpu_threshold_pct': self.high_cpu,
               'seconds': None if self.t0 is None or self.t1 is None else round(self.t1 - self.t0, 1)}
        res['high'] = bool((mean is not None and mean >= self.high_cpu) or self.compilers_max > 0)
        return res


def format_load(res: dict) -> str:
    if not res or not res.get('samples'):
        return 'machine load: no samples'
    s = (f'machine load over the scenarios: CPU mean {res["cpu_mean_pct"]} % (p95 {res["cpu_p95_pct"]}, max {res["cpu_max_pct"]}; '
         f'{res["samples"]} samples, {res["method"]}), compiler processes seen: max {res["compiler_processes_max"]}'
         + (f' ({", ".join(res["compiler_names"])})' if res['compiler_names'] else ''))
    if res.get('high'):
        s += (f' - WARNING: busy machine (CPU mean >= {res["high_cpu_threshold_pct"]:.0f} % or a compiler running): '
              'frame budget and timing numbers of this run are not comparable with a quiet machine')
    return s
