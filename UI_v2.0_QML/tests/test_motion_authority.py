"""Faz 1: tek hareket yetkisi, komut kabulu/sonuc ayrimi, arizada gozetim, eski nesil."""
import unittest

from support import Rig, motion_cmds, verb


class TerminalAuthorityTest(unittest.TestCase):
    def test_setid_and_unknown_never_sent(self):
        r = Rig(modules=(1,))
        r.setup()
        m = r.mark()
        self.assertIn("SETID", r.c.sendRaw("SETID:01:02"))
        self.assertTrue(r.c.sendRaw("INTERNAL_HOMEALL"))
        self.assertTrue(r.c.sendRaw("XYZ:01"))
        r.run_s(2)
        sent = r.sent_since(m)
        self.assertFalse([c for c in sent if verb(c) in ("SETID", "INTERNAL_HOMEALL", "XYZ")])

    def test_free_verbs_allowed_during_operation(self):
        r = Rig(modules=(1,))
        r.setup()
        r.c.startHomeAll()
        r.run(300)
        m = r.mark()
        for t in ("PING:01", "GETID:01", "GETPOS:01:03", "STAT:01"):
            self.assertEqual(r.c.sendRaw(t), "", t)
        r.run_s(3)
        sent = r.sent_since(m)
        for v in ("PING", "GETID", "GETPOS"):
            self.assertIn(v, [verb(c) for c in sent])

    def test_terminal_motion_goes_through_authority(self):
        r = Rig(modules=(1,))
        r.setup()
        # referanssiz: MOV/ALL/ARR reddedilir, HOME izinli (K8)
        self.assertIn("referanssız", r.c.sendRaw("MOV:01:01:300"))
        self.assertIn("referanssız", r.c.sendRaw("ALL:01:300"))
        m = r.mark()
        self.assertEqual(r.c.sendRaw("HOME:01"), "")
        r.run(500)
        self.assertEqual(motion_cmds(r.sent_since(m)), ["HOME:01"])
        self.assertTrue(r.c.busy)
        self.assertTrue(r.c.sendRaw("HOME:01"))       # islem suruyor


class AckVsCompletionTest(unittest.TestCase):
    def test_ack_alone_is_not_completion(self):
        r = Rig(modules=(1,), speed=4.0)
        r.setup()
        r.c.startHomeAll()
        r.run(200)                       # HOMEOK geldi, taze STAT henuz yok/H
        self.assertTrue(r.c.busy)
        self.assertEqual(r.successes(), [])

    def test_err_ack_fails_op_keeps_supervision(self):
        r = Rig(modules=(1, 2))
        r.setup()
        r.br.mods[1].u2_down = True      # HOMEERR, ama U3 yerelleri yine zamanlar
        m = r.mark()
        r.c.startHomeAll()
        r.run_until(lambda: r.failures(), max_s=10)
        self.assertEqual(r.failures()[-1][1], "Komut kısmen kabul edildi")
        self.assertNotIn("HOME:02", r.sent_since(m), "ERR sonrasi kalan modullere gonderildi")
        self.assertFalse(r.c.busy)
        self.assertTrue(r.c.motionLocked, "ERR sonrasi hareket olasi ama kilit yok")
        self.assertTrue(r.c.productionBlockReason)
        m2 = r.mark()
        r.run_s(5)
        self.assertIn("STAT:01", r.sent_since(m2))

    def test_lost_ack_no_retry(self):
        r = Rig(modules=(1,))
        r.setup()
        r.br.reply_filter = lambda cmd, line: None if cmd.startswith("HOME") else line
        m = r.mark()
        r.c.startHomeAll()
        r.run_until(lambda: r.failures(), max_s=10)
        self.assertEqual(r.failures()[-1][1], "Kabul doğrulanamadı")
        self.assertTrue(any(a["kind"] == "SYNC" for a in r.active_alarms()))
        self.assertTrue(r.c.motionLocked)
        r.run_s(30)
        self.assertEqual(r.sent_since(m).count("HOME:01"), 1, "kor otomatik tekrar")
        # komut aslinda ulasmisti: modul durunca 2 tutarli STAT ile SYNC normale doner,
        # ama onaylanana kadar listede "giderildi, onay bekliyor" olarak kalir
        sync = [a for a in r.alarms() if a["kind"] == "SYNC"]
        self.assertTrue(sync and not sync[0]["active"] and not sync[0]["acked"])
        self.assertFalse(r.c.allHomed, "kabulu dogrulanmamis HOME referans sayildi")

    def test_write_failure_is_not_accepted(self):
        r = Rig(modules=(1,))
        r.setup()
        r.br.write_ok = lambda cmd: not cmd.startswith("HOME")
        r.c.startHomeAll()
        r.run_until(lambda: r.failures(), max_s=10)
        self.assertEqual(r.failures()[-1][1], "Komut gönderilemedi")
        self.assertEqual(r.successes(), [])

    def test_dispatch_waits_for_ack(self):
        r = Rig(modules=(1, 2, 3))
        r.setup()
        m = r.mark()
        r.c.startHomeAll()
        r.run(50)                        # cevap ~85 ms: henuz ACK yok
        self.assertEqual(motion_cmds(r.sent_since(m)), ["HOME:01"])
        r.run(2000)
        self.assertEqual(motion_cmds(r.sent_since(m)), ["HOME:01", "HOME:02", "HOME:03"])

    def test_targets_are_immutable_copy(self):
        r = Rig(modules=(1,))
        r.setup()
        r.home_all()
        m = r.mark()
        r.c.startProduction()
        r.set_grid([400] * 144)          # islem sirasinda model degisse bile
        r.run_s(2)
        arr = [c for c in r.sent_since(m) if c.startswith("ARR")]
        self.assertEqual(arr, ["ARR:01:" + ":".join(["300"] * 9)])


class MaintenanceTest(unittest.TestCase):
    def test_maintenance_motion_tracked_and_visible(self):
        r = Rig(modules=(1,), speed=10.0)
        r.setup()
        r.home_all()
        r.c.allToValue(1, 300)
        r.run(300)
        self.assertTrue(r.c.busy)
        self.assertTrue(r.c.motionActive)
        r.run_until(lambda: r.results and r.results[-1][0] and not r.c.busy, max_s=200)
        self.assertIn("hedefe ulaştı", r.results[-1][2])

    def test_mov_completion_checks_target(self):
        r = Rig(modules=(1,))
        r.setup()
        r.home_all()
        n0 = len(r.results)
        r.c.moveMotor(1, 4, 120)
        r.run_until(lambda: len(r.results) > n0, max_s=60)
        self.assertTrue(r.results[-1][0])
        self.assertIn("Motor 4", r.results[-1][2])

    def test_reference_required_for_move_not_for_home(self):
        r = Rig(modules=(1,))
        r.setup()
        self.assertIn("referanssız", r.c.testerMoveBlockReason)
        self.assertEqual(r.c.testerHomeBlockReason, "")


class GenerationTest(unittest.TestCase):
    def test_disconnect_mid_dispatch_no_leak_after_reconnect(self):
        r = Rig(modules=(1, 2))
        r.setup()
        m = r.mark()
        r.c.startHomeAll()
        r.run(1)
        r.c.disconnectPort()
        r.connect()
        r.discover()
        r.run_s(20)
        self.assertEqual(motion_cmds(r.sent_since(m)), ["HOME:01"])


if __name__ == "__main__":
    unittest.main()
