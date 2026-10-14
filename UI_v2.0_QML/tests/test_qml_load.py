"""QML yukleme testi: Main.qml offscreen yuklenir, beklenmeyen QML/Qt uyarisi olmamali.

Ayri surecte calisir (QGuiApplication ile testlerdeki QCoreApplication cakismasin,
QML cokmesi test surecini dusurmesin).
"""
import os
import subprocess
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))


class QmlLoadTest(unittest.TestCase):
    def test_main_qml_loads_without_warnings(self):
        env = dict(os.environ, QT_QPA_PLATFORM="offscreen")
        p = subprocess.run([sys.executable, os.path.join(HERE, "qml_load_check.py")],
                           env=env, capture_output=True, text=True, timeout=120)
        out = (p.stdout or "") + (p.stderr or "")
        self.assertEqual(p.returncode, 0, "QML yukleme basarisiz:\n" + out)


if __name__ == "__main__":
    unittest.main()
