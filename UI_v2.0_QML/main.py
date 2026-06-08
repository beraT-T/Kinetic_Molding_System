#!/usr/bin/env python3
"""
Adaptif Kalip Kontrol - UI v2.0 (PySide6 + QML)
Tek entegre uygulama: GPU hizlandirmali QML arayuz + seri port + STL,
hepsi tek surecte. Flask/tarayici yok -> Raspberry Pi'de akici.

Calistirma:
    python3 main.py
Tam ekran (kiosk) icin:
    python3 main.py --fullscreen
"""
import os
import sys

# backend modullerini import yoluna ekle
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "backend"))

from PySide6.QtCore import QUrl
from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlApplicationEngine
from PySide6.QtQuickControls2 import QQuickStyle

from controller import Controller


def main():
    # Hafif, GPU-dostu kontrol stili
    QQuickStyle.setStyle("Basic")

    app = QGuiApplication(sys.argv)
    app.setApplicationName("Adaptif Kalip UI v2.0")

    engine = QQmlApplicationEngine()
    controller = Controller()
    engine.rootContext().setContextProperty("app", controller)
    engine.rootContext().setContextProperty(
        "startFullscreen", "--fullscreen" in sys.argv
    )

    qml_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "qml", "Main.qml")
    engine.load(QUrl.fromLocalFile(qml_path))

    if not engine.rootObjects():
        print("QML yuklenemedi.")
        sys.exit(-1)

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
