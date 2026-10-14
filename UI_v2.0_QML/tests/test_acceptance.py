"""
INCELEME.md "Kisa kabul testi matrisi" - otomatiklestirilebilen satirlar.

Her test gercek Controller'i sahte seri kopru (SimBridge) + sahte saat ile calistirir.
Fiziksel donanim, panel, gercek USB ve RS485 zamanlamasi BURADA DOGRULANMAZ.
"""
import os
import unittest

from support import (Rig, motion_cmds, verb, pump_events, write_box_stl, tmpdir,
                     QML_DIR)

S9_0 = ",".join(["S0"] * 9)
S9_300 = ",".join(["S300"] * 9)


class M01_WriteFailStaleS(unittest.TestCase):
    """Yazma basarisiz + eski S -> basari yok; kabul belirsizligi gorunur."""

    def test_write_fail_with_stale_S_never_succeeds(self):
        r = Rig(modules=(1,))
        r.setup()
        r.br.write_ok = False
        r.c.startProduction()
        # onceki islemden kalma S0 cevaplari hatta dolasiyor
        for k in range(40):
            r.br.inject("STAT:01:" + S9_0, delay_ms=10)
            r.run(2000)
        self.assertEqual(r.successes(), [], "yazma basarisizken basari bildirildi")
        self.assertTrue(r.failures(), "kabul edilmeyen komut operatore bildirilmedi")


class M02_StaleS300DuringHome(unittest.TestCase):
    """HOME sirasinda eski S300 -> referans tamamlanmaz, ARR baslamaz."""

    def test_stale_S300_does_not_complete_home(self):
        r = Rig(modules=(1,))
        r.setup()
        m = r.mark()
        r.c.startProduction()
        r.run(50)
        r.br.stat_override[1] = S9_300     # her STAT cevabi eski S300
        r.run_s(30)
        sent = r.sent_since(m)
        self.assertIn("HOME:01", sent)
        self.assertNotIn("ARR", [verb(c) for c in sent], "HOME bitmeden ARR gonderildi")
        self.assertFalse(r.c.allHomed)


class M03_PartialStat(unittest.TestCase):
    """Eksik/bozuk STAT, U2ERR -> bilinmeyen motorlar ayrilir; mevcut F kaybolmaz."""

    def test_u2err_partial_stat_keeps_fault(self):
        r = Rig(modules=(1,))
        r.setup()
        r.c.startHomeAll()
        r.run(500)
        r.br.stat_override[1] = "F0,S0,S0,S0,U2ERR"
        r.run_s(10)
        self.assertEqual(r.successes(), [])
        self.assertTrue(r.c.faultText, "kismi STAT'taki F islenmedi")
        info = r.c.moduleInfo(1)
        self.assertTrue(info["u2Error"])
        self.assertTrue(any(not m["known"] for m in info["motors"]))

    def test_garbage_tokens_are_not_success(self):
        r = Rig(modules=(1,))
        r.setup()
        r.c.startHomeAll()
        r.run(500)
        r.br.stat_override[1] = ",".join(["Sgarbage"] * 9)
        r.run_s(20)
        self.assertEqual(r.successes(), [])
        self.assertFalse(r.c.allHomed)


class M04_ConflictingMotionBlocked(unittest.TestCase):
    """HOME sirasinda MOV/ALL/terminal -> celiskili hareket butun girislerde engellenir."""

    def test_all_motion_paths_blocked_during_home(self):
        r = Rig(modules=(1,))
        r.setup()
        r.c.startHomeAll()
        r.run(300)
        m = r.mark()
        r.c.moveMotor(1, 1, 300)
        r.c.allToValue(1, 300)
        r.c.homeSlave(1)
        r.c.homeMotor(1, 1)
        r.c.sendArrayToSlave(1)
        r.c.startSendSelected(1)
        for raw in ("MOV:01:01:300", "ALL:01:0", "HOME:01", "HOME:01:02",
                    "ARR:01:300:300:300:300:300:300:300:300:300"):
            r.c.sendRaw(raw)
        r.run(3000)
        self.assertEqual(motion_cmds(r.sent_since(m)), [])


class M05_SecondStart(unittest.TestCase):
    """Ikinci BASLA -> ikinci hareket gonderilmez (backend)."""

    def test_second_start_sends_nothing(self):
        r = Rig(modules=(1,))
        r.setup()
        m = r.mark()
        r.c.startProduction()
        r.c.startProduction()
        r.run(3000)
        self.assertEqual(r.sent_since(m).count("HOME:01"), 1)


class M06_SupervisionAfterFault(unittest.TestCase):
    """F veya timeout sonrasi -> gozetim surer; yeni hareket ve alarm yasam dongusu kontrollu."""

    def _homed_rig(self, speed=100.0):
        r = Rig(modules=(1,), speed=speed)
        r.setup()
        res = r.home_all()
        self.assertTrue(res and res[-1][0], f"home basarisiz: {res}")
        return r

    def test_fault_keeps_polling_and_locks_motion(self):
        r = self._homed_rig(speed=4.0)
        r.c.startProduction()
        r.run_s(4)
        r.br.fail_motor(1, 1)            # motor 1 ariza, digerleri hareket halinde
        r.run_until(lambda: r.failures(), max_s=30)
        self.assertTrue(r.failures())
        self.assertFalse(r.c.allHomed, "arizada eski allHomed=True kaldi")
        m = r.mark()
        r.c.allToValue(1, 300)
        r.c.startProduction()
        r.c.homeSlave(1)
        r.run_s(10)
        sent = r.sent_since(m)
        self.assertEqual(motion_cmds(sent), [], "ariza sonrasi hareket kilidi yok")
        self.assertGreaterEqual(sent.count("STAT:01"), 3, "ariza sonrasi gozetim durdu")

    def test_fault_alarm_survives_acknowledge(self):
        r = self._homed_rig()
        r.c.startProduction()
        r.run_s(2)
        r.br.fail_motor(1, 3)
        r.run_until(lambda: r.failures(), max_s=30)
        r.c.clearFault()                  # "Uyarıyı gördüm"
        r.run_s(3)
        self.assertTrue(r.c.faultText, "onay arizayi sildi")
        self.assertTrue(r.active_alarms())

    def test_timeout_creates_persistent_alarm(self):
        r = self._homed_rig()
        r.br.stat_override[1] = ",".join(["M100"] * 9)   # hic bitmiyor
        r.c.startProduction()
        r.run_until(lambda: r.failures(), max_s=400)
        self.assertTrue(r.failures())
        self.assertTrue(r.active_alarms(), "zaman asimi kalici kayit uretmedi")

    def test_completed_module_rearm_is_supervised(self):
        r = self._homed_rig(speed=4.0)
        n0 = len(r.results)
        r.c.startProduction()
        r.run_until(lambda: len(r.results) > n0, max_s=200)
        self.assertTrue(r.results[-1][0])
        r.br.disturb(1, 2, 50.0)         # firmware REARM: S -> M (4 mm/s ~12 sn)
        r.run_s(4)
        self.assertTrue(r.c.motionLocked, "yeniden harekete gecen modul izlenmiyor")


class M07_StaleDispatch(unittest.TestCase):
    """Eski dispatch callback -> yeni ise komut sizmaz."""

    def test_old_dispatch_does_not_leak(self):
        r = Rig(modules=(1, 2))
        r.setup()
        m = r.mark()
        r.c.startHomeAll()
        r.run(1)
        # birinci islem modul 1 arizasiyla sonlaniyor
        r.br.inject("STAT:01:" + ",".join(["F0"] + ["H0"] * 8))
        r.run(1)
        r.c.startSendSelected(1)
        r.run(3000)
        sent = r.sent_since(m)
        self.assertNotIn("HOME:02", sent, "biten islemin HOME:02'si sonradan gonderildi")


class M08_AtomicStl(unittest.TestCase):
    """Bozuk yeni STL -> eski hedef yeni ad altinda uygulanamaz."""

    def test_failed_load_does_not_rename_old_grid(self):
        d = tmpdir()
        a = write_box_stl(os.path.join(d, "A.stl"), height=40)
        b = os.path.join(d, "B.stl")
        with open(b, "w") as f:
            f.write("bu bir stl degil\n")
        r = Rig(modules=(1,))
        r.setup(grid=None)

        def name():
            n = getattr(r.c, "modelName", None)
            if n is not None:
                return n
            return os.path.basename(r.c.stlFileUrl.toLocalFile())

        r.c.loadStl(a)
        self.assertTrue(pump_events(lambda: len(r.c.gridData) == 144))
        grid_a = list(r.c.gridData)
        r.c.loadStl(b)
        pump_events(lambda: getattr(r.c, "modelState", "") in ("error", "ready"), 5)
        self.assertFalse(name() == "B.stl" and list(r.c.gridData) == grid_a,
                         "B adiyla A'nin grid'i gosteriliyor")
        self.assertTrue(r.c.productionBlockReason, "bozuk yuklemeden sonra BASLA acik")


class M09_UsbDisconnect(unittest.TestCase):
    """USB kopma / yeniden baglanma -> guncellik ve referans duser; otomatik hareket yok."""

    def test_disconnect_invalidates_state(self):
        r = Rig(modules=(1,))
        r.setup()
        r.home_all()
        r.br.disconnect("USB koptu")
        r.run(500)
        self.assertFalse(r.c.connected)
        self.assertEqual(list(r.c.activeSlaves), [])
        self.assertFalse(r.c.allHomed)
        self.assertTrue(r.active_alarms(), "kopma alarmi yok")

    def test_manual_disconnect_clears_active_list(self):
        r = Rig(modules=(1,))
        r.setup()
        r.c.disconnectPort()
        r.run(100)
        self.assertEqual(list(r.c.activeSlaves), [])

    def test_reconnect_does_not_resume_motion(self):
        r = Rig(modules=(1,), speed=10.0)
        r.setup()
        r.c.startProduction()
        r.run_s(3)
        r.br.disconnect("USB koptu")
        r.run_s(2)
        r.connect()
        m = r.mark()
        r.discover()
        r.run_s(60)
        self.assertEqual(motion_cmds(r.sent_since(m)), [], "yeniden baglaninca hareket surdu")


class M10_SlaveReset(unittest.TestCase):
    """Slave reset -> once guncel durum uzlastirilir; referans varsayilmaz."""

    def test_nine_I0_invalidates_reference(self):
        r = Rig(modules=(1,))
        r.setup()
        r.home_all()
        self.assertTrue(r.c.allHomed)
        r.br.reset_module(1)
        r.c.requestStatus(1)
        r.run_s(5)
        self.assertFalse(r.c.allHomed, "dokuz I0 sonrasi referans gecerli kaldi")


class M11_SingleSerialManager(unittest.TestCase):
    """16 modul + gecikme + canli poll -> tek seri islem duzeni (hatta tek bekleyen istek)."""

    def test_no_overlapping_requests(self):
        r = Rig(modules=range(1, 17), speed=50.0)
        r.br.u2_slow_ms[5] = 1450            # modul 5: U2 gec cevap veriyor (~1,5 sn)
        r.setup()
        m = r.mark()
        r.c.startProduction()
        for k in range(80):                  # Tester "canli durum" + operator sorgulari
            r.c.requestStatus(k % 16 + 1)
            r.run(400)
        self.assertIn("HOME:16", r.sent_since(m), "uretim baslamadi")
        self.assertEqual(r.br.overlaps, 0, "cevap beklenirken hatta yeni komut yazildi")


class M12_TargetValidation(unittest.TestCase):
    """Bos, metin, eksi, >590 hedef -> hareket gonderilmez; neden aciklanir."""

    def test_invalid_targets_not_sent(self):
        r = Rig(modules=(1,))
        r.setup()
        r.home_all()
        m = r.mark()
        r.c.moveMotor(1, 1, -5)
        r.c.moveMotor(1, 1, 700)
        r.c.moveMotor(1, 1, 0)
        r.c.moveMotor(1, 1, 600)
        r.c.allToValue(1, 0)
        r.c.allToValue(1, 595)
        for raw in ("MOV:01:01:abc", "MOV:01:01:", "ALL:01:-1", "ARR:01:1:2:3",
                    "MOV:01:10:300", "ALL:01:600"):
            r.c.sendRaw(raw)
        r.run_s(3)
        self.assertEqual(motion_cmds(r.sent_since(m)), [])
        for bad in ("", "abc", "-5", "0", "9", "591", "600", "700", "30.5x"):
            self.assertTrue(r.c.checkTarget(bad), f"gecersiz hedef kabul edildi: {bad!r}")
        for good in ("10", "300", "590"):
            self.assertEqual(r.c.checkTarget(good), "", good)


class M13_BoundedSerialStream(unittest.TestCase):
    """Uzun/bozuk seri akis -> tampon sinirli."""

    def test_line_assembler_bounded(self):
        from serial_manager import LineAssembler
        asm = LineAssembler(max_len=256)
        lines = asm.feed("X" * 100000)
        self.assertLessEqual(asm.pending_len(), 256)
        lines += asm.feed("\nSTAT:01:" + S9_0 + "\n")
        self.assertIn("STAT:01:" + S9_0, lines)


class M14_StopText(unittest.TestCase):
    """Fiziksel durdurma: yazilim dogrulanmamis durdurma talimati vermemeli."""

    def test_busy_overlay_has_no_unverified_stop_instruction(self):
        with open(os.path.join(QML_DIR, "BusyOverlay.qml"), encoding="utf-8") as f:
            src = f.read()
        self.assertNotIn("GUCUNU KESIN", src)
        self.assertIn("Bu arayüzden hareket durdurulamıyor.", src)


if __name__ == "__main__":
    unittest.main()
