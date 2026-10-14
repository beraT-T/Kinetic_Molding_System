"""
Test yardimcilari: backend import yolu, Qt cekirdek uygulamasi, sahte kopru + sahte saat
ile kurulu Controller ("Rig").

Calistirma (UI_v2.0_QML klasorunden):
    QT_QPA_PLATFORM=offscreen venv/bin/python3 -m unittest discover -s tests -v
"""
import os
import sys
import tempfile

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BACKEND = os.path.join(ROOT, "backend")
QML_DIR = os.path.join(ROOT, "qml")
if BACKEND not in sys.path:
    sys.path.insert(0, BACKEND)

from PySide6.QtCore import QCoreApplication  # noqa: E402

APP = QCoreApplication.instance() or QCoreApplication([])

from timers import FakeTimerService  # noqa: E402
from sim_bridge import SimBridge  # noqa: E402
from controller import Controller  # noqa: E402

MOTION_VERBS = ("MOV", "ALL", "ARR", "HOME")


def verb(cmd):
    return cmd.split(":", 1)[0]


def motion_cmds(cmds):
    return [c for c in cmds if verb(c) in MOTION_VERBS]


def pump_events(cond, timeout_s=15.0):
    """Gercek thread'li isler icin (STL yukleme): olay dongusunu kosul saglanana
    kadar isle."""
    import time
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        APP.processEvents()
        if cond():
            return True
        time.sleep(0.01)
    APP.processEvents()
    return cond()


class Rig:
    """Controller + SimBridge + FakeTimerService. Zaman yalniz run() ile ilerler."""

    def __init__(self, modules=(1,), speed=100.0, config=None, **bridge_kw):
        self.ts = FakeTimerService()
        self.br = SimBridge(self.ts, modules=modules, speed_mm_s=speed, **bridge_kw)
        kw = {"transport": self.br, "timers": self.ts}
        if config is not None:
            kw["config"] = config
        try:
            self.c = Controller(**kw)
        except TypeError:
            kw.pop("config", None)
            self.c = Controller(**kw)
        self.results = []
        self.logs = []
        self.c.operationFinished.connect(
            lambda ok, t, m: self.results.append((bool(ok), t, m)))
        self.c.logMessage.connect(self.logs.append)

    # ---- zaman ----
    def run(self, ms):
        self.ts.advance(ms)

    def run_s(self, s):
        self.ts.advance_s(s)

    def run_until(self, cond, max_s=600.0, step_ms=100):
        t = 0.0
        while t < max_s * 1000.0:
            if cond():
                return True
            self.run(step_ms)
            t += step_ms
        return cond()

    # ---- kurulum ----
    def connect(self):
        self.c.connectPort("SIM")
        self.run(10)

    def discover(self):
        if hasattr(self.c, "_link"):
            self.c.scan()
            self.run_s(8)
        else:   # v2.0 tarama thread'i gercek uyur; testte PONG enjekte edilir
            for sid in sorted(self.br.mods):
                self.br.inject(f"PONG:{sid:02d}")
            self.run(10)

    def settle(self, s=None):
        """Gozetim poll'unun her modulu en az iki kez okumasini bekle."""
        n = max(1, len(self.br.mods))
        self.run_s(s if s is not None else 2 * n * 2.6 + 1)

    def set_grid(self, grid):
        grid = [int(v) for v in grid]
        if hasattr(self.c, "_set_model_for_test"):
            self.c._set_model_for_test("test.stl", grid)
        else:
            self.c._grid = grid
            self.c.gridChanged.emit()

    def setup(self, grid=300):
        """Bagla + modulleri bul + model + durum uzlasmasi."""
        self.connect()
        self.discover()
        if grid is not None:
            self.set_grid([grid] * 144)
        self.settle()

    def home_all(self):
        """Tum bulunan modullerde referans al ve bitmesini bekle."""
        n0 = len(self.results)
        self.c.startHomeAll()
        self.run_until(lambda: len(self.results) > n0, max_s=400)
        self.settle()
        return self.results[n0:] if len(self.results) > n0 else []

    # ---- gozlem ----
    def mark(self):
        return len(self.br.sent)

    def sent_since(self, mark):
        return self.br.sent[mark:]

    def successes(self):
        return [r for r in self.results if r[0]]

    def failures(self):
        return [r for r in self.results if not r[0]]

    def alarms(self):
        return list(getattr(self.c, "alarms", []) or [])

    def active_alarms(self):
        return [a for a in self.alarms() if a.get("active")]


def write_box_stl(path, height=50.0):
    """Kucuk, gecerli bir STL (kutu) yaz."""
    import trimesh
    box = trimesh.creation.box(extents=(100.0, 100.0, height))
    box.export(path)
    return path


def tmpdir():
    return tempfile.mkdtemp(prefix="v22test_")
