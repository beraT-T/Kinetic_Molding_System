"""Bagimsiz inceleme (2026-10-19) regresyonlari - backend (bulgu 2-6).

Her test, inceleme sirasinda yeniden uretilen HATALI davranisi tetikler ve dogru davranisi
ister. 2c5cf2c uzerinde bu testler KIRMIZI idi (docs/v2.2/DUZELTME_TURU.md "Kirmizi sonuc").
"""
import unittest

from support import Rig, motion_cmds
import hmi_config
from sim_bridge import SimBridge


def _ack_received(r, sid):
    op = r.c._op
    return op is not None and sid in op["ack"]


# ---------------------------------------------------------------------------
# Bulgu 5: yazma tamamlanmasi hic gelmezse islem sonsuza kadar "writing" kaliyordu
# ---------------------------------------------------------------------------
class HoldingBridge(SimBridge):
    """Yazma tamamlanma callback'ini (istege bagli) HIC cagirmayan sahte tasiyici.
    Gercekte: yazici thread'i flush()/tcdrain'de takilir, USB surucusu askida kalir."""

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.hold = None            # None | callable(cmd) -> bool (True = callback'i tut)
        self.held = []              # (cmd, done_cb)

    def write_async(self, text, done_cb):
        cmd = text.strip()
        if self.hold is not None and self.hold(cmd):
            self.attempted.append(cmd)
            self.held.append((cmd, done_cb))
            return
        super().write_async(text, done_cb)


def holding_rig(modules=(1,), speed=100.0):
    return Rig(modules=modules, speed=speed, bridge_cls=HoldingBridge)


class WriteCompletionSupervisionTest(unittest.TestCase):
    def test_missing_write_callback_does_not_hang_forever(self):
        r = holding_rig()
        r.setup()
        r.br.hold = lambda cmd: cmd.startswith("STAT")      # gozetim sorgusu takilir
        r.run_s(100)
        out = r.c._link.outstanding
        self.assertFalse(out is not None and out.state == "writing",
                         f"100 sn sonra istek hala yazma asamasinda: {out}")
        self.assertFalse(r.c.connected, "takilan yazmadan sonra baglanti gecerli sayildi")
        keys = [a["key"] for a in r.active_alarms()]
        self.assertIn("LINK", keys)

    def test_missing_write_callback_during_motion_ends_op_without_retry(self):
        r = holding_rig()
        r.setup()
        r.br.hold = lambda cmd: cmd.startswith("HOME")
        r.c.startHomeAll()
        r.run_s(100)
        self.assertFalse(r.c.busy, "islem yazma tamamlanmasini sonsuza kadar bekliyor")
        self.assertTrue(r.failures(), "operatore sonuc bildirilmedi")
        self.assertEqual(sum(1 for c in r.br.attempted if c.startswith("HOME")), 1,
                         "belirsiz yazmadan sonra hareket komutu tekrarlandi")
        keys = [a["key"] for a in r.alarms()]
        self.assertIn("SYNC:1", keys)
        self.assertFalse(r.c.allHomed)
        # gec gelen callback eski oturuma aittir: hicbir sey degismez, hareket gonderilmez
        n_att = len(r.br.attempted)
        for _cmd, cb in r.br.held:
            cb(True)
        r.run_s(10)
        self.assertFalse(r.c.busy)
        self.assertEqual(motion_cmds(r.br.attempted[n_att:]), [])

    def test_late_callback_after_reconnect_is_ignored(self):
        r = holding_rig()
        r.setup()
        r.br.hold = lambda cmd: cmd.startswith("STAT")
        r.run_s(20)
        self.assertFalse(r.c.connected)
        r.br.hold = None
        r.connect()
        r.discover()
        r.settle()
        self.assertTrue(r.c.connected)
        before = (r.c._link.gen, r.c.connected, list(r.c.activeSlaves))
        for _cmd, cb in r.br.held:      # onceki oturumun callback'leri simdi geliyor
            cb(True)
            cb(False)
        r.run_s(5)
        self.assertEqual((r.c._link.gen, r.c.connected, list(r.c.activeSlaves)), before)
        self.assertNotIn("LINK", [a["key"] for a in r.active_alarms()])


# ---------------------------------------------------------------------------
# Bulgu 2: STAT zaman asimindan sonra eski cevap (9 x S0) sahte HOME basarisi uretiyordu
# ---------------------------------------------------------------------------
class StaleReplyAfterTimeoutTest(unittest.TestCase):
    def _homed_at_300(self):
        r = Rig(modules=(1,), speed=4.0)
        r.setup()
        r.home_all()
        n0 = len(r.results)
        r.c.allToValue(1, 300)
        r.run_until(lambda: len(r.results) > n0, max_s=400)
        self.assertTrue(r.results[-1][0], r.results[-1])
        r.settle()
        self.assertTrue(r.c.moduleInfo(1)["homed"])
        return r

    def test_stale_S0_after_stat_timeout_does_not_complete_home(self):
        r = self._homed_at_300()
        stale = "STAT:01:" + ",".join(["S0"] * 9)     # HOME'dan ONCEKI donemin cevabi
        states_at_success = []
        r.c.operationFinished.connect(
            lambda ok, t, m: states_at_success.append(r.br.states(1)) if ok else None)
        n0 = len(r.results)
        r.c.homeSlave(1)
        self.assertTrue(r.run_until(lambda: _ack_received(r, 1), max_s=10, step_ms=10))
        phase = {"p": "drop"}

        def filt(cmd, line):
            if phase["p"] == "drop":
                return None if cmd.startswith("STAT") else line
            if phase["p"] == "stale_next":
                # Modul gec kalan eski cevabi gonderirken sonraki komut kaybolur
                # (firmware cevaptan sonra RS485 girisini bosaltir): o komuta gelen
                # satir ESKI STAT cevabidir.
                phase["p"] = "normal"
                return stale
            return line
        r.br.reply_filter = filt
        self.assertTrue(r.run_until(lambda: any("Cevap yok: STAT:01" in l for l in r.logs),
                                    max_s=20, step_ms=20))
        phase["p"] = "stale_next"
        r.run_until(lambda: len(r.results) > n0, max_s=400, step_ms=100)
        for st in states_at_success:
            self.assertNotIn("H", st, "motorlar H iken 'Referans alma tamamlandı' uretildi")
        self.assertTrue(r.results and r.results[-1][0],
                        f"gercek referans tamamlaninca da basari gelmedi: {r.results[-1:]}")
        self.assertEqual(r.br.states(1), "S" * 9)

    def test_unsolicited_stat_forces_line_order_check(self):
        """Istenmemis (gec) STAT cevabi gorulurse, o modulun sonraki cevabi tazelik kaniti
        sayilmaz: once hat sirasi PING/PONG ile yeniden kurulur."""
        r = Rig(modules=(1,))
        r.setup()
        r.run_until(lambda: r.c._link.idle(), max_s=5, step_ms=10)
        r.br.inject("STAT:01:" + ",".join(["S0"] * 9))
        r.run(5)
        m = r.mark()
        r.run_s(3)
        first = [c for c in r.sent_since(m) if c.endswith(":01") or ":01:" in c]
        self.assertTrue(first, r.sent_since(m))
        self.assertEqual(first[0], "PING:01")

    def test_timeout_makes_module_data_unverified_until_realigned(self):
        r = Rig(modules=(1,))
        r.setup()
        r.home_all()
        r.br.reply_filter = lambda cmd, line: None if cmd.startswith("STAT") else line
        r.run_until(lambda: any("Cevap yok: STAT:01" in l for l in r.logs), max_s=20, step_ms=20)
        self.assertIn("doğrulan", r.c.testerMoveBlockReason)
        r.br.reply_filter = None
        r.run_s(8)
        self.assertEqual(r.c.testerMoveBlockReason, "")


# ---------------------------------------------------------------------------
# Bulgu 4: terminal sorgulari ariza gozetimini 84,8 sn geciktiriyordu
# ---------------------------------------------------------------------------
def loaded_bound(n):
    f = getattr(hmi_config, "worst_alarm_latency_loaded_s", None)
    return f(n) if f else hmi_config.worst_alarm_latency_s(n)


class SupervisionUnderQueryLoadTest(unittest.TestCase):
    def _run(self, n, fail_sid, queries=30, offset_ms=0):
        r = Rig(modules=range(1, n + 1), speed=50.0)
        r.setup()
        r.home_all()
        r.c.acknowledgeAlarms()
        r.br.reply_filter = lambda cmd, line: None if cmd.startswith("GETPOS") else line
        rejected = [x for x in (r.c.sendRaw(f"GETPOS:{fail_sid:02d}:{1 + i % 9}")
                                for i in range(queries)) if x]
        r.run(offset_ms)
        t0 = r.ts.now()
        r.br.fail_motor(fail_sid, 4)
        ok = r.run_until(lambda: "MOTOR 4" in r.c.faultText, max_s=300, step_ms=20)
        return ok, r.ts.now() - t0, rejected, r

    def _check(self, n, fail_sid):
        worst = 0.0
        for off in (0, 330, 770, 1500, 2600):
            ok, dt, rejected, r = self._run(n, fail_sid, offset_ms=off)
            self.assertTrue(ok)
            worst = max(worst, dt)
            self.assertLessEqual(dt, loaded_bound(n),
                                 f"{n} modul, ofset {off} ms: ariza {dt:.1f} sn sonra gorundu "
                                 f"(sinir {loaded_bound(n):.1f})")
            self.assertTrue(rejected, "30 yanitsiz sorgunun hepsi kuyruga alindi (yuk kontrolu yok)")
            self.assertIn("sorgu", rejected[0].lower())
        print(f"\n  [olcum] {n} modul + 30 yanitsiz GETPOS: en kotu {worst:.1f} sn "
              f"(sinir {loaded_bound(n):.2f})")

    def test_one_module(self):
        self._check(1, 1)

    def test_four_modules(self):
        self._check(4, 3)

    def test_sixteen_modules(self):
        self._check(16, 9)

    def test_double_tap_same_query_sent_once(self):
        r = Rig(modules=(1,))
        r.setup()
        self.assertEqual(r.c.sendRaw("GETPOS:01:3"), "")
        self.assertTrue(r.c.sendRaw("GETPOS:01:3"), "ayni sorgu ikinci kez kuyruga girdi")
        m = r.mark()
        r.c.requestStatus(1)
        r.c.requestStatus(1)
        r.run_s(4)
        self.assertLessEqual(sum(1 for c in r.sent_since(m) if c == "GETPOS:01:3"), 1)

    def test_long_run_supervision_does_not_starve_or_grow(self):
        """2 saat (sahte saat): 4 modul, her 7 sn'de bir terminal sorgusu (bazilari
        cevapsiz). Hicbir modulun verisi esigi asmaz, kuyruk/zamanlayici/alarm sayisi
        sinirli kalir, sahte alarm olusmaz."""
        r = Rig(modules=(1, 2, 3, 4), speed=50.0)
        r.setup()
        r.home_all()
        r.c.acknowledgeAlarms()
        drop = {"on": False}
        r.br.reply_filter = lambda cmd, line: None if (drop["on"] and cmd.startswith("GETPOS")) else line
        thr = hmi_config.data_stale_s(4)
        max_q = max_timers = 0
        max_age = 0.0
        for k in range(2 * 3600):
            if k % 7 == 0:
                drop["on"] = (k // 7) % 3 == 0
                r.c.sendRaw(f"GETPOS:{1 + k % 4:02d}:{1 + k % 9}")
                r.c.requestStatus(1 + (k // 7) % 4)
            r.run(1000)
            max_q = max(max_q, len(r.c._link.queued()))
            max_timers = max(max_timers, r.ts.pending())
            for sid in (1, 2, 3, 4):
                a = r.c._age_s(r.c._mods[sid])
                max_age = max(max_age, a if a is not None else 1e9)
        self.assertLessEqual(max_age, thr, f"gozetim aksadi: en eski veri {max_age:.1f} sn")
        self.assertLessEqual(max_q, 8)
        self.assertLessEqual(max_timers, 20)
        self.assertEqual(r.active_alarms(), [])
        self.assertLessEqual(len(r.alarms()), 4)


# ---------------------------------------------------------------------------
# Bulgu 3: yeniden baglanmada kayip modul unutuluyordu
# ---------------------------------------------------------------------------
class LostModuleAcrossReconnectTest(unittest.TestCase):
    def _two_homed(self):
        r = Rig(modules=(1, 2))
        r.setup()
        r.home_all()
        self.assertTrue(r.c.allHomed)
        return r

    def test_silent_module_not_forgotten_after_reconnect(self):
        r = self._two_homed()
        r.br.mods[2].u3_down = True
        r.c.disconnectPort()            # COMM alarmi olusmadan once kes
        r.connect()
        r.discover()
        r.settle()
        self.assertEqual(list(r.c.activeSlaves), [1])
        for reason in (r.c.productionBlockReason, r.c.homeAllBlockReason):
            self.assertTrue(reason, "kapsam sessizce Modul 1'e daraldi")
            self.assertIn("2", reason)
        m = r.mark()
        r.c.startProduction()
        r.c.startHomeAll()
        r.run_s(5)
        self.assertEqual(motion_cmds(r.sent_since(m)), [])
        self.assertIn("COMM:2", [a["key"] for a in r.active_alarms()])

    def test_module_returning_clears_lost_state(self):
        r = self._two_homed()
        r.br.mods[2].u3_down = True
        r.c.disconnectPort()
        r.connect()
        r.discover()
        r.settle()
        r.br.mods[2].u3_down = False
        r.run_s(12)
        self.assertEqual(list(r.c.activeSlaves), [1, 2])
        self.assertNotIn("COMM:2", [a["key"] for a in r.active_alarms()])
        self.assertEqual(r.c.homeAllBlockReason, "")

    def test_restart_with_unconfigured_machine_does_not_invent_modules(self):
        """Uygulama yeniden baslatildi (yeni Controller), machine.json null: onceki oturum
        bilinemez. Arayuz modul listesi UYDURMAZ ve kapsamin yalniz taramaya dayandigini
        acikca soyler (BASLA onayi bulunan modulleri listeler)."""
        r = Rig(modules=(1,))
        r.setup()
        r.home_all()
        self.assertEqual(list(r.c.expectedModules), [])
        plan = r.c.productionPlan
        self.assertFalse(plan["expectedConfigured"])
        self.assertEqual(plan["modules"], [1])
        self.assertEqual(plan.get("lost", []), [])
        self.assertIn("yapılandırılmadı", r.c.statusInfo["text"])


# ---------------------------------------------------------------------------
# Turkce / operator dili: sonuc mesajlarinda ham protokol kodu yok
# ---------------------------------------------------------------------------
class OperatorMessageTest(unittest.TestCase):
    RAW = ("ARRERR", "HOMEERR", "ALLERR", "U2ERR", "HOMEOK", "ARROK", "STAT:", "ACK")

    def test_u2_error_result_is_plain_language(self):
        r = Rig(modules=(1,), config={"expected_modules": None, "firmware_variant": "v5f411",
                                      "errors": [], "path": "(test)"})
        r.setup()
        r.br.mods[1].u2_down = True
        r.c.homeSlave(1)
        r.run_s(5)
        self.assertTrue(r.failures())
        title, msg = r.failures()[-1][1], r.failures()[-1][2]
        for code in self.RAW:
            self.assertNotIn(code, title + msg, f"ham kod operatore gosterildi: {code}")
        # mesaj yapisi: ne oldu / etkisi / sonraki adim
        self.assertIn("Etki:", msg)
        self.assertIn("Adım:", msg)
        for a in r.active_alarms():
            for code in self.RAW:
                self.assertNotIn(code, a["text"])


class FlappingAlarmTest(unittest.TestCase):
    def test_flapping_fault_is_one_record_and_no_popup_storm(self):
        """Ariza 1 dakika boyunca gelip gidiyor: kayit anahtar basina TEK satir kalir,
        islem yokken sonuc penceresi acilmaz, onay kaydi silmez."""
        r = Rig(modules=(1,))
        r.setup()
        r.home_all()
        n_res = len(r.results)
        for k in range(20):
            r.br.stat_override[1] = ",".join(["F0"] + ["S0"] * 8) if k % 2 == 0 else \
                ",".join(["S0"] * 9)
            r.run_s(3)
            if k % 5 == 0:
                r.c.acknowledgeAlarms()
        keys = [a["key"] for a in r.alarms()]
        self.assertEqual(len(keys), len(set(keys)), keys)
        self.assertLessEqual(keys.count("F:1:1"), 1)
        self.assertEqual(len(r.results), n_res, "islem yokken ariza icin pencere acildi")


# ---------------------------------------------------------------------------
# Bulgu 6: tek motor HOME sonrasi ayni motor hareketi - politika acik soylenmeli
# ---------------------------------------------------------------------------
class SingleMotorReferencePolicyTest(unittest.TestCase):
    def test_single_motor_home_then_move_is_explained(self):
        r = Rig(modules=(1,))
        r.setup()
        n0 = len(r.results)
        r.c.homeMotor(1, 3)
        r.run_until(lambda: len(r.results) > n0, max_s=60)
        self.assertTrue(r.results[-1][0], r.results[-1])
        self.assertIn("modül referansını tamamlamaz", r.results[-1][2])
        reason = r.c.testerMoveBlockReason
        self.assertIn("tek motor referansı yeterli değil", reason)
        m = r.mark()
        r.c.moveMotor(1, 3, 100)
        r.run_s(2)
        self.assertEqual(motion_cmds(r.sent_since(m)), [])
        self.assertIn("tek motor referansı yeterli değil", r.failures()[-1][2])
        info = r.c.moduleInfo(1)
        self.assertTrue(info["motors"][2]["referenced"])
        self.assertFalse(info["homed"])

    def test_full_module_home_allows_move(self):
        r = Rig(modules=(1,))
        r.setup()
        n0 = len(r.results)
        r.c.homeSlave(1)
        r.run_until(lambda: len(r.results) > n0, max_s=60)
        r.settle()
        m = r.mark()
        r.c.moveMotor(1, 3, 100)
        r.run_s(1)
        self.assertEqual(motion_cmds(r.sent_since(m)), ["MOV:01:03:100"])


if __name__ == "__main__":
    unittest.main()
