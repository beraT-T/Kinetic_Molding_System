"""
Gercek QML etkilesim testleri (alt surecte calisir: test_qml_interaction.py).

Dokunuslar QTest ile pencereye gercek fare olayi olarak gider. Pencere 1920x1080
(Pi paneli). Zaman sahte saatle ilerler.
"""
import os
import unittest

from qmlui_harness import UiRig, QT_MESSAGES

MOTION = ("MOV", "ALL", "ARR", "HOME")


def motion(cmds):
    return [c for c in cmds if c.split(":", 1)[0] in MOTION]


def ready(modules=(1, 2), **kw):
    u = UiRig(modules=modules, **kw)
    u.r.setup()
    u.r.home_all()
    u.pump()
    u.close_info()
    return u


# ---------------------------------------------------------------------------
# Bulgu 1: onay penceresi acilis modulunu gosterirken canli secime komut gonderiyordu
# ---------------------------------------------------------------------------
class DialogContextTest(unittest.TestCase):
    def _select_module_via_map(self, u, sid):
        u.tab(0)
        u.tap_heatmap_module(sid)
        self.assertTrue(u.shown(u.must("detailMakeTarget")))
        u.tap("detailMakeTarget")
        self.assertEqual(u.c.currentSlaveId, sid)

    def test_target_pad_does_not_send_to_other_module(self):
        u = ready()
        self.assertEqual(u.c.testerMoveBlockReason, "")
        u.tab(1)
        u.tap("motorCell_3")             # motor hucresine dokun -> motor paneli
        pad = u.must("numPad")
        self.assertTrue(u.shown(pad))
        self.assertIn("Modül 1 / Motor 3", pad.property("title"))
        for k in "100":
            u.tap("padKey_" + k)
        self.assertEqual(pad.property("entry"), "100")
        self._select_module_via_map(u, 2)
        u.tab(1)
        m = u.r.mark()
        acc = u.must("padAccept")
        if u.shown(acc) and acc.isEnabled():
            # pencere hala acik: baslik ve gidecek komut AYNI modulu gostermeli
            self.assertIn("Modül 1 / Motor 3", pad.property("title"))
            u.tap(acc)
        u.run_s(1)
        sent = u.r.sent_since(m)
        self.assertNotIn("MOV:02:03:100", sent, "baslik Modul 1 iken Modul 2'ye komut gitti")
        self.assertFalse([c for c in motion(sent) if not c.startswith("MOV:01:03:")], sent)

    def test_motor_home_confirm_does_not_go_to_other_module(self):
        u = ready()
        u.tab(1)
        u.tap("motorCell_3")
        u.tap("padHome")                 # panelde "Motor 3'e referans al" -> onay
        dlg = u.must("testerConfirm")
        self.assertTrue(u.shown(dlg))
        title = dlg.property("title")
        self._select_module_via_map(u, 2)
        u.tab(1)
        m = u.r.mark()
        acc = u.must("testerConfirm_accept")
        if u.shown(acc):
            self.assertEqual(dlg.property("title"), title)
            u.tap(acc)
        u.run_s(1)
        sent = motion(u.r.sent_since(m))
        self.assertNotIn("HOME:02:03", sent)
        self.assertFalse([c for c in sent if c != "HOME:01:03"], sent)

    def test_send_selected_confirm_bound_to_shown_module(self):
        u = ready()
        self.assertEqual(u.c.currentSlaveId, 1)
        u.tab(0)
        u.tap("btnSendSelected")
        dlg = u.must("mainConfirm")
        self.assertTrue(u.shown(dlg))
        self.assertIn("Modül: 1", dlg.property("message"))
        u.tab(1)
        u.tap("modTile_2")
        self.assertEqual(u.c.currentSlaveId, 2)
        u.tab(0)
        m = u.r.mark()
        acc = u.must("mainConfirm_accept")
        if u.shown(acc):
            self.assertIn("Modül: 1", dlg.property("message"))
            u.tap(acc)
        u.run_s(1)
        sent = motion(u.r.sent_since(m))
        self.assertFalse([c for c in sent if c.startswith("ARR:02")], sent)

    def test_start_confirm_invalidated_when_module_list_changes(self):
        u = UiRig(modules=(1, 2, 3))
        u.r.br.mods[3].u3_down = True
        u.r.setup()
        u.r.home_all()
        u.pump()
        u.close_info()
        self.assertEqual(list(u.c.activeSlaves), [1, 2])
        u.tab(0)
        u.tap("btnStart")
        dlg = u.must("mainConfirm")
        self.assertTrue(u.shown(dlg))
        self.assertIn("1, 2", dlg.property("message"))
        u.r.br.mods[3].u3_down = False
        u.tap("btnScan")                 # baslik cubugu onay ortusunun disinda
        u.run_s(8)
        u.r.settle()
        u.pump()
        self.assertEqual(list(u.c.activeSlaves), [1, 2, 3])
        m = u.r.mark()
        acc = u.must("mainConfirm_accept")
        if u.shown(acc):
            u.tap(acc)
        u.run_s(2)
        sent = motion(u.r.sent_since(m))
        self.assertEqual(sent, [], "onayda listelenmeyen modul kapsama girdi")


# ---------------------------------------------------------------------------
# Bakim ekrani geometrisi (1920x1080): butonlar hucre icinde, kilavuz kesilmiyor
# ---------------------------------------------------------------------------
class TesterGeometryTest(unittest.TestCase):
    def _check(self, demo):
        u = UiRig(modules=(1, 2, 3, 4), demo=demo)
        u.r.setup()                      # referans yok -> sebep satirlari gorunur (en kotu)
        u.tab(1)
        u.pump(10)
        page = u.must("testerPage")
        W, H = 1920, 1080

        def inside(inner, outer, tol=0.5):
            ix, iy, iw, ih = u.scene_rect(inner)
            ox, oy, ow, oh = u.scene_rect(outer)
            return (ix >= ox - tol and iy >= oy - tol and ix + iw <= ox + ow + tol and
                    iy + ih <= oy + oh + tol)
        # Pi'de yazi tipi (DejaVu Sans) Mac'tekinden genis/yuksek olabilir: hucre ve
        # kilavuz icin en az %12 bos yer istenir (yalniz "sigiyor" yetmez).
        for i in range(1, 10):
            cell = u.must(f"motorCell_{i}", page)
            self.assertGreaterEqual(cell.height(), 80, f"motor hucresi dokunma hedefi < 80 px")
            for name in (f"motorTitle_{i}", f"motorState_{i}", f"measLabel_{i}",
                         f"measValue_{i}", f"targetValue_{i}", f"motorHint_{i}"):
                it = u.must(name, page)
                self.assertTrue(inside(it, cell), f"demo={demo}: {name} hucre disina tasiyor "
                                f"{u.scene_rect(it)} / {u.scene_rect(cell)}")
            # en kucuk icerik: baslik satiri + bosluk + (etiket + deger) + kenar paylari
            row = max(u.must(f"motorTitle_{i}", page).height(),
                      u.must(f"motorState_{i}", page).height())
            vals = u.must(f"measLabel_{i}", page).height() + u.must(f"measValue_{i}", page).height()
            need = row + 4 + vals + 2 * 8
            self.assertGreaterEqual(cell.height(), need * 1.12,
                                    f"demo={demo}: motor hucresinde yazi tipi payi yok")
        for k in range(1, 17):
            tag = u.must(f"modTag_{k}", page)
            self.assertFalse(tag.property("truncated"), f"modul karosu etiketi kesiliyor")
            self.assertTrue(inside(tag, u.must(f"modTile_{k}", page)))
        box, txt = u.must("guideBox", page), u.must("guideText", page)
        bx, by, bw, bh = u.scene_rect(box)
        tx, ty, tw, th = u.scene_rect(txt)
        self.assertLessEqual(ty + txt.property("implicitHeight") * 1.12, by + bh,
                             f"demo={demo}: kilavuz metni kesiliyor / pay yok")
        inp = u.must("terminalInput", page)
        x, y, w, h = u.scene_rect(inp)
        self.assertLessEqual(y + h, H)
        self.assertLessEqual(x + w, W)
        reason = u.must("testerReason", page)
        self.assertTrue(u.shown(reason))
        self.assertFalse(reason.property("truncated"), "devre disi sebebi kesiliyor")
        # motor paneli pencereye sigiyor (tus takimi + eylemler + sebepler)
        u.tap("motorCell_5")
        padbox = u.must("padBox")
        x, y, w, h = u.scene_rect(padbox)
        self.assertTrue(x >= 0 and y >= 0 and x + w <= W and y + h <= H, (x, y, w, h))
        self.assertGreaterEqual(h, u.must("padCancel").mapToItem(padbox, 0, 0).y()
                                + u.must("padCancel").height())
        for name in ("padAccept", "padHome", "padCancel", "padMoveReason", "padKey_0"):
            self.assertTrue(inside(u.must(name), padbox), f"{name} panel disinda")
        return u

    def test_geometry_real_panel(self):
        u = self._check(demo=False)
        out = os.environ.get("QMLUI_SHOTS")
        if out:
            u.grab(os.path.join(out, "bakim_1920x1080.png"))

    def test_geometry_with_simulation_band(self):
        u = self._check(demo=True)
        out = os.environ.get("QMLUI_SHOTS")
        if out:
            u.grab(os.path.join(out, "bakim_1920x1080_simulasyon.png"))


# ---------------------------------------------------------------------------
# Metin / durum davranisi gercek QML'de
# ---------------------------------------------------------------------------
class OperatorFeedbackTest(unittest.TestCase):
    def test_disabled_motor_buttons_show_reason(self):
        u = UiRig(modules=(1,))
        u.r.setup()
        u.tab(1)
        u.tap("motorCell_1")
        for k in "100":
            u.tap("padKey_" + k)
        b = u.must("padAccept")
        self.assertFalse(b.isEnabled())
        self.assertTrue(b.property("disabledReason"), "devre disi butonun sebebi yok")
        self.assertTrue(b.property("showReason"))
        full = u.must("padMoveReason")
        self.assertTrue(u.shown(full))
        self.assertIn("referans", full.property("text"))
        h = u.must("padHome")
        self.assertNotEqual(h.property("text"), "Ref.")
        self.assertIn("referans al", h.property("text"))

    def test_stale_measurement_labelled_last_known(self):
        u = ready(modules=(1,))
        u.tab(1)
        lbl = u.must("measLabel_1")
        self.assertEqual(lbl.property("text"), "Ölçülen")
        u.r.br.mods[1].u3_down = True
        u.run_s(40)
        self.assertTrue(u.c.dataStale)
        self.assertIn("Son bilinen", lbl.property("text"))

    def test_result_dialogs_do_not_storm_or_hide_fault(self):
        u = UiRig(modules=(1,))
        dlg = u.must("infoDialog")
        u.c.operationFinished.emit(False, "ARIZA", "Motor arızası bildirildi: Modül 1 / Motor 2.")
        for _ in range(10):
            u.c.operationFinished.emit(False, "Tarama başlatılmadı", "Bağlantı yok.")
        u.pump()
        self.assertTrue(u.shown(dlg))
        self.assertEqual(dlg.property("title"), "ARIZA", "ariza penceresi sonraki mesajla ezildi")
        u.tap("infoDialog_accept")
        self.assertTrue(u.shown(dlg))
        self.assertEqual(dlg.property("title"), "Tarama başlatılmadı")
        u.tap("infoDialog_accept")
        self.assertFalse(u.shown(dlg), "ayni mesaj icin pencere tekrar tekrar acildi")

    def test_alarm_ack_in_strip_does_not_clear_fault(self):
        u = ready(modules=(1,))
        u.r.br.fail_motor(1, 5)
        u.run_s(4)
        self.assertIn("F:1:5", [a["key"] for a in u.c.alarms if a["active"]])
        u.tap("stripAck")
        u.run_s(1)
        self.assertIn("F:1:5", [a["key"] for a in u.c.alarms if a["active"]])
        self.assertEqual(u.c.statusInfo["level"], 2)


class NoQmlWarningsTest(unittest.TestCase):
    def test_zz_no_unexpected_qt_messages(self):
        self.assertEqual(QT_MESSAGES, [])


if __name__ == "__main__":
    unittest.main()
