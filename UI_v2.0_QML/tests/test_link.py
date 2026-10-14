"""SerialLink (tek seri islem yoneticisi) birim testleri."""
import unittest

import support  # noqa: F401
from timers import FakeTimerService
from link import SerialLink, Request
from hmi_config import POST_TIMEOUT_GUARD_MS


class FakeWire:
    """Yazmayi kaydeder; tamamlanmayi 1 ms sonra bildirir."""

    def __init__(self, ts):
        self.ts = ts
        self.written = []
        self.ok = True

    def write_async(self, text, done):
        self.written.append(text)
        ok = self.ok
        self.ts.call_later(1, lambda: done(ok))


class LinkTest(unittest.TestCase):
    def setUp(self):
        self.ts = FakeTimerService()
        self.wire = FakeWire(self.ts)
        self.unsol = []
        self.link = SerialLink(self.ts, self.wire.write_async,
                               lambda rep, line: self.unsol.append(line), max_queue=4)
        self.link.start()
        self.done = []

    def req(self, cmd, sid, kind="query", timeout=500, motor=None):
        return Request(cmd, kind, timeout, sid=sid, motor=motor,
                       on_done=lambda r, st, rep: self.done.append((r.cmd, st,
                                                                    rep.raw if rep else None)))

    def test_single_outstanding_request(self):
        for s in (1, 2, 3):
            self.assertTrue(self.link.submit(self.req(f"STAT:{s:02d}", s)))
        self.ts.advance(100)
        self.assertEqual(self.wire.written, ["STAT:01"])
        self.link.on_line("STAT:01:S0")
        self.ts.advance(100)
        self.assertEqual(self.wire.written, ["STAT:01", "STAT:02"])
        self.assertEqual(self.done[0], ("STAT:01", "ok", "STAT:01:S0"))

    def test_timeout_then_guard_then_next(self):
        self.link.submit(self.req("STAT:01", 1, timeout=500))
        self.link.submit(self.req("STAT:02", 2))
        self.ts.advance(510)
        self.assertEqual(self.done, [("STAT:01", "timeout", None)])
        self.assertEqual(self.wire.written, ["STAT:01"])      # koruma suresi
        self.ts.advance(POST_TIMEOUT_GUARD_MS + 10)
        self.assertEqual(self.wire.written, ["STAT:01", "STAT:02"])

    def test_write_failure_reported_not_retried(self):
        self.wire.ok = False
        self.link.submit(self.req("ARR:01:1:2:3:4:5:6:7:8:9", 1, kind="motion"))
        self.ts.advance(100)
        self.assertEqual(self.done[0][1], "write_failed")
        self.ts.advance(5000)
        self.assertEqual(len(self.wire.written), 1)

    def test_motion_timeout_not_retried(self):
        self.link.submit(self.req("HOME:01", 1, kind="motion", motor=0))
        self.ts.advance(10000)
        self.assertEqual(self.done, [("HOME:01", "timeout", None)])
        self.assertEqual(self.wire.written, ["HOME:01"])

    def test_bounded_queue(self):
        self.link.submit(self.req("STAT:01", 1))
        self.ts.advance(5)                   # 1. istek hatta
        for s in range(2, 6):
            self.assertTrue(self.link.submit(self.req(f"STAT:{s:02d}", s)))
        self.assertFalse(self.link.submit(self.req("STAT:06", 6)), "kuyruk siniri yok")

    def test_reset_fails_everything_and_ignores_late_reply(self):
        self.link.submit(self.req("STAT:01", 1))
        self.link.submit(self.req("STAT:02", 2))
        self.ts.advance(5)
        self.link.reset()
        self.assertEqual(sorted(st for _c, st, _r in self.done), ["link_lost", "link_lost"])
        self.link.start()
        self.link.on_line("STAT:01:S0")      # eski baglantinin gec cevabi
        self.assertEqual(self.unsol, ["STAT:01:S0"])

    def test_unsolicited_line_not_matched(self):
        self.link.submit(self.req("STAT:01", 1))
        self.ts.advance(5)
        self.link.on_line("STAT:02:S0")
        self.assertEqual(self.unsol, ["STAT:02:S0"])
        self.assertEqual(self.done, [])

    def test_residual_risk_late_reply_matches_next_same_request(self):
        """BILINEN SINIR (protokolde islem kimligi yok): zaman asimina ugramis
        STAT:01'in gec cevabi, bir SONRAKI STAT:01 istegine eslesir. Azaltma
        Controller'da: zaman asimi sonrasi modul 'senkron disi' sayilir ve ardisik
        2 tutarli taze STAT istenir (K11). Oneri: PROTOKOL_ONERI_v5.2.md."""
        self.link.submit(self.req("STAT:01", 1, timeout=500))
        self.ts.advance(510 + POST_TIMEOUT_GUARD_MS)
        self.link.submit(self.req("STAT:01", 1, timeout=500))
        self.ts.advance(5)
        self.link.on_line("STAT:01:OLD")     # aslinda ilk istegin cevabi
        self.assertEqual(self.done[-1], ("STAT:01", "ok", "STAT:01:OLD"))


if __name__ == "__main__":
    unittest.main()
