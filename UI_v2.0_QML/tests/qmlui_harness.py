"""
Gercek QML etkilesim testleri icin yardimci (ayri surecte calisir).

QGuiApplication + QQmlApplicationEngine ile Main.qml yuklenir; Controller sahte kopru
(SimBridge) + sahte saat (FakeTimerService) ile kurulur. Dokunuslar QTest.mouseClick ile
pencereye GERCEK fare olayi olarak gonderilir (z-sirasi, modal ortu, enabled durumu
gercek QML'deki gibi isler). Zaman yalniz Rig.run() ile ilerler.

Bu modul test kesfi (test*.py) disindadir: QGuiApplication, testlerdeki
QCoreApplication ile ayni surecte olusturulamaz. test_qml_interaction.py bu modulleri
alt surecte calistirir.
"""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "backend"))
sys.path.insert(0, HERE)

from PySide6.QtCore import QUrl, QPointF, QPoint, Qt, qInstallMessageHandler  # noqa: E402
from PySide6.QtGui import QGuiApplication  # noqa: E402
from PySide6.QtQml import QQmlApplicationEngine  # noqa: E402
from PySide6.QtQuickControls2 import QQuickStyle  # noqa: E402
from PySide6.QtTest import QTest  # noqa: E402

QQuickStyle.setStyle("Basic")
APP = QGuiApplication.instance() or QGuiApplication(sys.argv[:1])

from support import Rig  # noqa: E402  (QGuiApplication'dan SONRA)

ALLOWED = ("isApiRhiBased", "Qt Quick 3D is not functional", "Populating font family aliases")
QT_MESSAGES = []


def _handler(mode, ctx, msg):
    if not any(a in msg for a in ALLOWED):
        QT_MESSAGES.append(msg)


qInstallMessageHandler(_handler)


class UiRig:
    """Rig + gercek Main.qml penceresi (1920x1080, hedef panel cozunurlugu)."""

    def __init__(self, modules=(1, 2), demo=False, size=(1920, 1080), **kw):
        self.r = Rig(modules=modules, demo=demo, **kw)
        self.c = self.r.c
        self.engine = QQmlApplicationEngine()
        self.engine.warnings.connect(lambda ws: QT_MESSAGES.extend(w.toString() for w in ws))
        ctx = self.engine.rootContext()
        ctx.setContextProperty("app", self.c)
        ctx.setContextProperty("startFullscreen", False)
        self.engine.load(QUrl.fromLocalFile(os.path.join(ROOT, "qml", "Main.qml")))
        roots = self.engine.rootObjects()
        if not roots:
            raise RuntimeError("Main.qml yuklenemedi: " + "\n".join(QT_MESSAGES))
        self.win = roots[0]
        self.win.setProperty("width", size[0])
        self.win.setProperty("height", size[1])
        self.pump()

    # ---- olay dongusu ----
    def pump(self, n=6):
        for _ in range(n):
            APP.processEvents()

    def render(self):
        """Yerlesim (polish) + bir kare cizimi zorla: offscreen'de olay isleme tek basina
        GridLayout'u yerlestirmez. Olcumden once cagrilir."""
        self.pump(2)
        self.win.grabWindow()
        self.pump(2)

    def run_s(self, s):
        self.r.run_s(s)
        self.pump()

    # ---- oge bulma (gorsel agac; Repeater delegeleri dahil) ----
    def find(self, name, under=None):
        root = under if under is not None else self.win.contentItem()
        stack = [root]
        while stack:
            it = stack.pop()
            if it.objectName() == name:
                return it
            stack.extend(reversed(it.childItems()))
        return None

    def must(self, name, under=None):
        it = self.find(name, under)
        if it is None:
            raise AssertionError(f"QML ogesi bulunamadi: {name}")
        return it

    @staticmethod
    def shown(item):
        """Oge ve butun atalari gorunur mu (StackLayout gizli sekmeyi gizler)."""
        it = item
        while it is not None:
            if not it.isVisible():
                return False
            it = it.parentItem()
        return True

    def scene_rect(self, item):
        p = item.mapToScene(QPointF(0, 0))
        return (p.x(), p.y(), item.width(), item.height())

    # ---- dokunma (gercek fare olayi) ----
    def tap_at(self, x, y):
        """Parmak dokunusu benzetimi: imleci getir, bas, kisa bekle, birak.
        (Hareketsiz tek mouseClick offscreen'de Button'a ulasmiyor.)"""
        p = QPoint(int(x), int(y))
        QTest.mouseMove(self.win, p)
        self.pump(2)
        QTest.mousePress(self.win, Qt.LeftButton, Qt.NoModifier, p)
        self.pump(2)
        QTest.qWait(30)
        QTest.mouseRelease(self.win, Qt.LeftButton, Qt.NoModifier, p)
        self.pump()

    def tap(self, item_or_name, fx=0.5, fy=0.5):
        self.render()
        it = self.must(item_or_name) if isinstance(item_or_name, str) else item_or_name
        p = it.mapToScene(QPointF(it.width() * fx, it.height() * fy))
        self.tap_at(p.x(), p.y())

    def tap_heatmap_module(self, sid):
        hm = self.must("heatmap")
        r0, c0 = (sid - 1) // 4 * 3, (sid - 1) % 4 * 3
        self.tap(hm, (c0 + 1.5) / 12.0, (r0 + 1.5) / 12.0)

    def close_info(self, max_n=10):
        """Sonuc penceresi(leri) acik ise operator gibi 'Kapat'a dokun."""
        for _ in range(max_n):
            dlg = self.find("infoDialog")
            if dlg is None or not self.shown(dlg):
                return
            self.tap("infoDialog_accept")

    def tab(self, i):
        self.tap(f"tab_{i}")
        self.render()

    def grab(self, path):
        img = self.win.grabWindow()
        img.save(path)
        return path
