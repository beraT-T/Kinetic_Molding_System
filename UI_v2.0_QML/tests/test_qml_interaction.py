"""Gercek QML etkilesim testleri (qmlui_tests.py) ayri surecte calistirilir.

QGuiApplication, test surecindeki QCoreApplication ile ayni surecte kurulamaz; QML
cokmesi de test surecini dusurmesin. Alt surecin unittest ciktisi hata mesajina eklenir.
"""
import os
import subprocess
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))


class QmlInteractionTest(unittest.TestCase):
    def test_qml_interactions(self):
        env = dict(os.environ, QT_QPA_PLATFORM="offscreen")
        p = subprocess.run([sys.executable, "-m", "unittest", "-v", "qmlui_tests"],
                           cwd=HERE, env=env, capture_output=True, text=True, timeout=600)
        out = (p.stdout or "") + (p.stderr or "")
        self.assertEqual(p.returncode, 0, "QML etkilesim testleri basarisiz:\n" + out[-12000:])


if __name__ == "__main__":
    unittest.main()
