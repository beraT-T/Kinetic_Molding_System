"""Faz 2: baglanti/erisilebilirlik, referans gecerliligi, veri yasi, alarm yasam dongusu,
kismi STAT (U2ERR + surum), beklenen moduller, en kotu alarm gecikmesi (hat simulasyonu)."""
import os
import time
import unittest

from support import Rig, motion_cmds
from hmi_config import (worst_alarm_latency_s, typical_alarm_latency_s, data_stale_s,
                        load_machine_config)


def cfg(expected=None, variant=None):
    return {"expected_modules": expected, "firmware_variant": variant, "errors": [],
            "path": "(test)"}


class ReachabilityTest(unittest.TestCase):
    def test_silent_module_drops_from_active_and_alarms(self):
        r = Rig(modules=(1, 2))
        r.setup()
        self.assertEqual(list(r.c.activeSlaves), [1, 2])
        r.br.mods[2].u3_down = True
        r.run_s(20)
        self.assertEqual(list(r.c.activeSlaves), [1])
        keys = [a["key"] for a in r.active_alarms()]
        self.assertIn("COMM:2", keys)
        self.assertTrue(r.c.productionBlockReason)
        # geri gelince taze gecerli STAT ile cozulur (onay bekler)
        r.br.mods[2].u3_down = False
        r.run_s(10)
        self.assertEqual(list(r.c.activeSlaves), [1, 2])
        self.assertNotIn("COMM:2", [a["key"] for a in r.active_alarms()])

    def test_reconnect_requires_reconciliation(self):
        r = Rig(modules=(1,), speed=10.0)
        r.setup()
        r.c.startHomeAll()
        r.run_s(2)
        r.br.disconnect("USB koptu")
        r.run(100)
        self.assertIn("LINK", [a["key"] for a in r.active_alarms()])
        self.assertIn("SYNC:1", [a["key"] for a in r.active_alarms()])
        r.connect()
        self.assertNotIn("LINK", [a["key"] for a in r.active_alarms()])
        r.discover()
        self.assertFalse(r.c.allHomed, "yeniden baglaninca referans varsayildi")
        self.assertTrue(r.c.productionBlockReason)
        r.run_s(60)                    # modul durur, 2 tutarli STAT -> SYNC cozulur
        self.assertNotIn("SYNC:1", [a["key"] for a in r.active_alarms()])


class ReferenceValidityTest(unittest.TestCase):
    def test_home_timeout_invalidates_reference(self):
        r = Rig(modules=(1,))
        r.setup()
        r.br.stat_override[1] = ",".join(["H50"] * 9)
        r.c.startHomeAll()
        r.run_until(lambda: r.failures(), max_s=400)
        self.assertEqual(r.failures()[-1][1], "Zaman aşımı")
        self.assertFalse(r.c.allHomed)

    def test_fault_invalidates_reference(self):
        r = Rig(modules=(1,), speed=4.0)
        r.setup()
        r.home_all()
        self.assertTrue(r.c.allHomed)
        r.c.allToValue(1, 100)
        r.run_s(3)
        r.br.fail_motor(1, 2)
        r.run_s(3)
        self.assertFalse(r.c.allHomed)


class DataAgeTest(unittest.TestCase):
    def test_status_unknown_when_data_old(self):
        r = Rig(modules=(1,))
        r.setup()
        r.home_all()
        self.assertFalse(r.c.dataStale)
        r.br.mods[1].u3_down = True
        r.run_s(data_stale_s(1) + 5)
        self.assertTrue(r.c.dataStale)
        info = r.c.moduleInfo(1)
        self.assertTrue(info["stale"])
        self.assertGreater(info["ageS"], data_stale_s(1))

    def test_status_text_unknown_without_alarm(self):
        """Alarm uretmeyen bir eskime: veri yasi esigi asilinca serit 'bilinmiyor' der."""
        r = Rig(modules=(1,))
        r.setup()
        r.c.acknowledgeAlarms()
        # poll'u durdur: tick zamanlayicisini kapat (ornegin olay dongusu takildi)
        r.c._tick_timer.stop()
        r.run_s(data_stale_s(1) + 2)
        r.c._changed()
        self.assertEqual(r.c.statusInfo["text"], "Güncel makine durumu bilinmiyor.")
        self.assertNotIn("Anormal durum yok", r.c.statusInfo["text"])


class AlarmLifecycleTest(unittest.TestCase):
    def _faulted(self):
        r = Rig(modules=(1,))
        r.setup()
        r.home_all()
        r.br.fail_motor(1, 5)          # islem disi ariza
        r.run_s(4)
        return r

    def test_fault_outside_operation_creates_alarm(self):
        r = self._faulted()
        self.assertIn("F:1:5", [a["key"] for a in r.active_alarms()])
        self.assertIn("MOTOR 5", r.c.faultText)

    def test_ack_is_not_resolve(self):
        r = self._faulted()
        r.c.acknowledgeAlarms()
        a = [x for x in r.alarms() if x["key"] == "F:1:5"][0]
        self.assertTrue(a["active"] and a["acked"])
        self.assertTrue(r.c.faultText)
        self.assertEqual(r.c.unackedAlarms, 0)

    def test_resolves_only_with_fresh_valid_stat(self):
        r = self._faulted()
        r.c.acknowledgeAlarms()
        r.br.reset_module(1)           # modul enerjisi kesilip verildi -> I0
        r.run_s(6)
        keys = [a["key"] for a in r.alarms()]
        self.assertNotIn("F:1:5", keys, "onayli + normale donmus alarm listeden kalkmali")
        self.assertIn("SYNC:1", keys)  # reset tespiti ayri kayit
        self.assertFalse(r.c.allHomed)

    def test_resolved_but_unacked_stays_visible(self):
        r = self._faulted()
        r.br.reset_module(1)
        r.run_s(6)
        a = [x for x in r.alarms() if x["key"] == "F:1:5"][0]
        self.assertFalse(a["active"])
        self.assertFalse(a["acked"])

    def test_comm_ambiguity_does_not_resolve(self):
        r = self._faulted()
        r.br.mods[1].u3_down = True
        r.run_s(20)
        self.assertIn("F:1:5", [a["key"] for a in r.active_alarms()])

    def test_new_operation_does_not_clear_alarms(self):
        r = Rig(modules=(1,))
        r.setup()
        r.br.inject("STAT:01:" + ",".join(["F0"] + ["S0"] * 8))
        r.run(100)
        n = len(r.active_alarms())
        r.c.startHomeAll()             # aktif ariza -> reddedilir, kayit kalir
        r.run(100)
        self.assertGreaterEqual(len(r.active_alarms()), n)


class PartialStatTest(unittest.TestCase):
    def test_variant_configured_maps_local_motors(self):
        r = Rig(modules=(1,), config=cfg(variant="v5f411"))
        r.setup()
        r.br.stat_override[1] = "F0,S0,S0,S0,U2ERR"
        r.run_s(3)
        keys = [a["key"] for a in r.active_alarms()]
        self.assertIn("F:1:1", keys)
        self.assertIn("U2:1", keys)
        info = r.c.moduleInfo(1)
        self.assertTrue(info["motors"][0]["known"])
        self.assertTrue(all(not m["known"] for m in info["motors"][4:]))
        self.assertTrue(info["u2Error"])

    def test_u2_alarm_persists_until_full_stat(self):
        r = Rig(modules=(1,), config=cfg(variant="v5f411"))
        r.setup()
        r.br.mods[1].u2_down = True
        r.run_s(6)
        self.assertIn("U2:1", [a["key"] for a in r.active_alarms()])
        self.assertIn("motor 5–9", [a for a in r.active_alarms() if a["key"] == "U2:1"][0]["text"])
        r.c.acknowledgeAlarms()
        r.run_s(6)
        self.assertIn("U2:1", [a["key"] for a in r.active_alarms()])
        r.br.mods[1].u2_down = False
        r.run_s(6)
        self.assertNotIn("U2:1", [a["key"] for a in r.active_alarms()])

    def test_v51_format_with_v5f411_config_is_mismatch(self):
        r = Rig(modules=(1,), config=cfg(variant="v5f411"))
        r.setup()
        r.br.stat_override[1] = "U2ERR,S0,S0,S0,F0"
        r.run_s(3)
        a = [x for x in r.active_alarms() if x["key"] == "F:1:?"]
        self.assertTrue(a and "çelişiyor" in a[0]["text"])


class ExpectedModulesTest(unittest.TestCase):
    def test_missing_expected_module_blocks_start(self):
        r = Rig(modules=(1,), config=cfg(expected=[1, 2]))
        r.setup()
        self.assertIn("eksik: 2", r.c.productionBlockReason)

    def test_unexpected_module_blocks_start(self):
        r = Rig(modules=(1, 3), config=cfg(expected=[1]))
        r.setup()
        self.assertIn("Beklenmeyen modül bulundu: 3", r.c.productionBlockReason)

    def test_matching_set_allows_start(self):
        r = Rig(modules=(1, 2), config=cfg(expected=[1, 2]))
        r.setup()
        self.assertEqual(r.c.productionBlockReason, "")
        self.assertEqual(r.c.productionPlan["modules"], [1, 2])
        self.assertEqual(r.c.productionPlan["motors"], 18)

    def test_send_selected_requires_active_and_homed(self):
        r = Rig(modules=(1,))
        r.setup()
        r.c.currentSlaveId = 2
        self.assertIn("erişilemiyor", r.c.sendSelectedBlockReason)
        r.c.currentSlaveId = 1
        self.assertIn("referanssız", r.c.sendSelectedBlockReason)


class MachineConfigTest(unittest.TestCase):
    def test_invalid_config_is_conservative(self):
        import tempfile
        d = tempfile.mkdtemp()
        p = os.path.join(d, "m.json")
        with open(p, "w") as f:
            f.write('{"expected_modules": [0, 99], "firmware_variant": "v9"}')
        c = load_machine_config(p)
        self.assertIsNone(c["expected_modules"])
        self.assertIsNone(c["firmware_variant"])
        self.assertEqual(len(c["errors"]), 2)

    def test_missing_config(self):
        c = load_machine_config("/nonexistent/machine.json")
        self.assertIsNone(c["expected_modules"])
        self.assertTrue(c["errors"])


class AlarmLatencyTest(unittest.TestCase):
    """Hat simulasyonu: 16 modul, biri ~1,5 sn gec cevap veriyor, Bakim sorgulari da var.
    Bir motor F'ye dusunce operatorun gormesi hesaplanan en kotu gecikmeyi asmamali."""

    def test_fault_visible_within_worst_case(self):
        r = Rig(modules=range(1, 17), speed=50.0)
        r.br.u2_slow_ms[7] = 1450
        r.setup()
        r.home_all()
        r.c.acknowledgeAlarms()
        t0 = r.ts.now()
        r.br.fail_motor(9, 4)
        ok = r.run_until(lambda: "MOTOR 4" in r.c.faultText, max_s=120, step_ms=50)
        dt = r.ts.now() - t0
        self.assertTrue(ok)
        self.assertLessEqual(dt, worst_alarm_latency_s(16))
        print(f"\n  [olcum] 16 modul, ariza gorunme suresi {dt:.1f} sn "
              f"(tipik hesap {typical_alarm_latency_s(16):.1f}, en kotu {worst_alarm_latency_s(16):.1f})")


class SerialManagerLoopTest(unittest.TestCase):
    """Gercek SerialManager thread'leri, pyserial 'loop://' ile (donanimsiz)."""

    def test_async_write_read_and_disconnect_signal(self):
        from serial_manager import SerialManager
        got, disc, res = [], [], []
        m = SerialManager(on_line=got.append, on_disconnect=disc.append)
        m.open("loop://", 9600)
        m.write_async("STAT:01", res.append)
        end = time.time() + 2
        while time.time() < end and not got:
            time.sleep(0.01)
        self.assertEqual(got, ["STAT:01"])
        self.assertEqual(res, [True])
        # portu arkadan kapat (USB cikti benzetimi): okuyucu hatayi YUTMAMALI
        m._s.ser.close()
        end = time.time() + 2
        while time.time() < end and not disc:
            time.sleep(0.01)
        self.assertTrue(disc, "kopma bildirilmedi")
        m.close()

    def test_close_does_not_block(self):
        from serial_manager import SerialManager
        m = SerialManager()
        m.open("loop://", 9600)
        t = time.time()
        m.close()
        self.assertLess(time.time() - t, 0.2)
        res = []
        m.write_async("PING:01", res.append)
        self.assertEqual(res, [False])


if __name__ == "__main__":
    unittest.main()
