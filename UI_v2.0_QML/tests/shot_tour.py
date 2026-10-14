"""
Offscreen ekran goruntusu turu (sabah inceleme icin). qml_load_check.py --shots <klasor>
ile cagrilir. Pencere 1920x1080, demo (SIMULASYON) baglantisi; her durum icin PNG.

NOT: Quick3D offscreen cizilmez (3D alan bos cikar - beklenen). Bu goruntuler gercek
panel olcumu DEGILDIR; okunabilirlik/dokunma isabeti PANELDE TEST edilmelidir.
"""
import os

from PySide6.QtCore import QTimer, QObject


def start(app, engine, controller, root, outdir):
    os.makedirs(outdir, exist_ok=True)
    root.setProperty("width", 1920)
    root.setProperty("height", 1080)
    tabs = root.findChild(QObject, "tabStack")
    alarms = root.findChild(QObject, "alarmPanel")
    c = controller
    steps = []

    def shot(name):
        def f():
            img = root.grabWindow()
            img.save(os.path.join(outdir, name + ".png"))
            print("goruntu:", name)
        return f

    def tab(i):
        return lambda: tabs.setProperty("currentIndex", i)

    def model():
        g = [int(150 + 250 * (((r - 5.5) ** 2 + (k - 5.5) ** 2) / 60.5))
             for r in range(12) for k in range(12)]
        c._set_model_for_test("ornek_kubbe.stl", [min(590, max(10, v)) for v in g])

    detail = root.findChild(QObject, "moduleDetail")
    pad = root.findChild(QObject, "numPad")

    def show_pad():
        pad.setProperty("motor", 3)
        pad.setProperty("title", "Modül 1 / Motor 3 hedefi")
        pad.setProperty("info", "Ölçülen: 0 mm (1 sn önce)")
        pad.setProperty("actionPrefix", "Motor 3'ü")
        pad.setProperty("entry", "250")
        pad.setProperty("visible", True)

    steps += [(500, shot("01_baslangic_baglanti_yok"))]
    steps += [(100, lambda: c.connectPort("SIM")), (100, c.scan), (100, model)]
    steps += [(12000, shot("02_stl_kontrol_bagli_referans_yok"))]
    steps += [(100, lambda: (detail.setProperty("sid", 6), detail.setProperty("visible", True))),
              (500, shot("03_harita_modul_ayrintisi")),
              (100, lambda: detail.setProperty("visible", False))]
    steps += [(100, tab(1)), (800, shot("04_bakim_motor_testi"))]
    steps += [(100, show_pad), (500, shot("05_sayisal_tus_takimi")),
              (100, lambda: pad.setProperty("visible", False))]
    steps += [(100, tab(0)), (100, c.startProduction), (6000, shot("06_islem_suruyor"))]
    steps += [(100, lambda: c._transport.fail_motor(2, 5)), (6000, shot("07_ariza_sonuc"))]
    steps += [(100, lambda: alarms.setProperty("visible", True)), (600, shot("08_alarm_listesi"))]
    steps += [(100, lambda: alarms.setProperty("visible", False)), (100, tab(1)),
              (600, shot("09_bakim_ariza_sonrasi"))]
    steps += [(100, app.quit)]

    t = 0
    for delay, fn in steps:
        t += delay
        QTimer.singleShot(t, fn)
