"""
QML yukleme kontrolu (ayri surecte calisir; test_qml_load.py cagirir).

Main.qml'i offscreen QQmlApplicationEngine ile yukler, kisa sure olay isler ve
beklenmeyen her Qt/QML uyarisini raporlar. Cikis kodu 0 = temiz.

Beklenen ve zararsiz (filtrelenir):
  - Quick3D offscreen cizmez: "isApiRhiBased" / "Qt Quick 3D is not functional"
  - Font takma ad taramasi bilgisi ("Populating font family aliases")

Istege bagli: --shots <klasor>  her sekmenin 1920x1080 ekran goruntusunu kaydeder.
"""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "backend"))

from PySide6.QtCore import QUrl, QTimer, qInstallMessageHandler  # noqa: E402
from PySide6.QtGui import QGuiApplication  # noqa: E402
from PySide6.QtQml import QQmlApplicationEngine  # noqa: E402
from PySide6.QtQuickControls2 import QQuickStyle  # noqa: E402

ALLOWED = (
    "isApiRhiBased",
    "Qt Quick 3D is not functional",
    "Populating font family aliases",
)

messages = []


def handler(mode, ctx, msg):
    if any(a in msg for a in ALLOWED):
        return
    messages.append(msg)


def main():
    shots = None
    if "--shots" in sys.argv:
        shots = sys.argv[sys.argv.index("--shots") + 1]
    run_ms = 2500 if shots else 1500

    qInstallMessageHandler(handler)
    QQuickStyle.setStyle("Basic")
    app = QGuiApplication(sys.argv)

    from controller import Controller
    controller = Controller(demo=True)

    engine = QQmlApplicationEngine()
    engine.warnings.connect(lambda ws: messages.extend(w.toString() for w in ws))
    engine.rootContext().setContextProperty("app", controller)
    engine.rootContext().setContextProperty("startFullscreen", False)
    engine.load(QUrl.fromLocalFile(os.path.join(ROOT, "qml", "Main.qml")))
    roots = engine.rootObjects()
    if not roots:
        print("QML yuklenemedi")
        for m in messages:
            print("UYARI:", m)
        return 2

    if shots:
        import shot_tour
        shot_tour.start(app, engine, controller, roots[0], shots)
    else:
        QTimer.singleShot(run_ms, app.quit)
    app.exec()

    for m in messages:
        print("UYARI:", m)
    return 1 if messages else 0


if __name__ == "__main__":
    sys.exit(main())
