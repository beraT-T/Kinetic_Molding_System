"""
Controller - QML ile Python arasindaki kopru (tek QObject). QML 'app' olarak erisir.

v2.2 ana fikir: ARAYUZ BILMEDIGI BIR SEYI BILDIGINI IDDIA ETMEZ.
  - Tek hareket yetkilendirme noktasi: _block_reason() / _authorize_motion().
    Uretim, Bakim butonlari, dogrudan slotlar ve terminal hep buradan gecer.
  - Tek seri islem yoneticisi (link.SerialLink): hatta tek bekleyen istek.
  - Komut kabulu (ACK/ERR) ile tamamlanma ayri. Tamamlanma = yazma basarili + bu
    isleme ait ACK + ACK'ten SONRA istenmis taze STAT + 9 gecerli token + tolerans.
  - Islem nesli (op id) ve baglanti nesli (link gen): eski callback/cevap yok sayilir.
  - Gozetim her zaman surer (round-robin STAT); is sonucu != makine gozetimi.
  - Kalici alarm modeli: "Uyarıyı gördüm" (onay) != "giderildi" (taze STAT ile normale donus).
  - Model yukleme atomik (dosya adi + grid birlikte, yalniz basarida).

Protokol v5.1: ACK = "basladi", bitis STAT ile anlasilir. STOP/ABORT komutu YOK:
bu arayuzden hareket durdurulamaz (docs/v2.2/GUVENLIK_ACIK_KONULAR.md).
"""
import os
import threading
import time
from collections import deque

from PySide6.QtCore import QObject, Signal, Slot, Property, QUrl

import protocol as P
from hmi_config import (TARGET_TOL_MM, HOME_ZERO_TOL_MM, MOTORS_PER_MODULE, MAX_MODULES,
                        POLL_TICK_MS, STAT_TIMEOUT_MS, MOTION_ACK_TIMEOUT_MS,
                        PING_TIMEOUT_MS, PING_GUARD_MS, QUERY_TIMEOUT_MS, COMM_FAIL_LIMIT, SYNC_CONFIRM,
                        OP_PHASE_TIMEOUT_S, SERIAL_LINE_QUEUE_MAX, SAFE_MIN_MM, SAFE_MAX_MM,
                        data_stale_s, load_machine_config)
from link import SerialLink, Request
from serial_manager import SerialManager
from stl_interpolator import interpolate_stl_to_grid_info, grid_stats
from timers import QtTimerService


def slave_grid_origin(slave_id):
    """slave_id (1..16) -> 12x12 grid icindeki 3x3 baslangic (row, col)."""
    row = (slave_id - 1) // 4
    col = (slave_id - 1) % 4
    return row * 3, col * 3


def slave_values(grid, slave_id):
    """slave'in 9 motor hedefini (M1..M9, satir oncelikli) grid'den cikar."""
    sr, sc = slave_grid_origin(slave_id)
    return [int(grid[(sr + i) * 12 + (sc + j)]) for i in range(3) for j in range(3)]


def _ids_text(ids):
    return ", ".join(str(s) for s in sorted(ids))


STATE_TEXT = {"S": "hedefe ulaştı", "M": "hareket", "H": "referans alıyor",
              "F": "ARIZA", "I": "boşta"}


# Alarm turu -> operatore neden / etki / adim (v2.2, INCELEME 4b). Uydurma talimat yok:
# fiziksel durdurma/enerji ayirma proseduru tanimli DEGIL (GUVENLIK_ACIK_KONULAR.md).
ALARM_HELP = {
    "F": ("Neden: firmware motoru ARIZA durumuna aldı (ör. hareket komutunda 1,5 sn içinde "
          "yeterli encoder darbesi yok: sıkışma, encoder kablosu, mekanik dayanma).\n"
          "Etki: firmware bildirimine göre o motorun sürüşü kesildi; diğer motorların "
          "durduğu doğrulanmadı. Bütün hareket kilitli.\n"
          "Adım: Kayıt, modül arızasız taze durum bildirince kapanır (firmware ARIZA'dan yalnız "
          "yeni komut veya modülün yeniden başlaması ile çıkar). Müdahale yetkin bakım "
          "personelince yapılmalı."),
    "U2": ("Neden: modülün ikinci kartı (U2) birinci karta (U3) cevap vermedi.\n"
           "Etki: U2'ye bağlı motorların durumu bilinmiyor; bütün hareket kilitli.\n"
           "Adım: Kartlar arası bağlantı bakım personelince incelenmeli; tam durum cevabı "
           "gelince kayıt kapanır."),
    "COMM": ("Neden: modül art arda sorgulara cevap vermedi (kablo, enerji, adres).\n"
             "Etki: modülün hareket edip etmediği bilinmiyor; bütün hareket kilitli.\n"
             "Adım: Haberleşme düzelip geçerli durum gelince kayıt kapanır. Modül bilerek "
             "çıkarıldıysa uygulama yeniden başlatılmalı."),
    "SYNC": ("Neden: komutun kabulü/tamamlanması doğrulanamadı, bağlantı kesildi veya modül "
             "yeniden başladı.\n"
             "Etki: modülün durumu belirsiz; bütün hareket kilitli. Komut otomatik tekrar "
             "gönderilmez.\n"
             "Adım: Modül iki ardışık sorguda tutarlı ve hareketsiz durum bildirince kayıt "
             "kendiliğinden kapanır."),
    "LINK": ("Neden: seri bağlantı koptu veya kesildi.\n"
             "Etki: hareketin durduğu doğrulanamıyor; referans bilgisi geçersiz.\n"
             "Adım: Yeniden bağlanın ve 'Modülleri bul'. Hareket otomatik sürdürülmez."),
    "DATA": ("Neden: modülden şema dışı veya eksik durum cevabı geldi.\n"
             "Etki: motor durumları bilinmiyor; bütün hareket kilitli.\n"
             "Adım: Geçerli tam cevap gelince kayıt kapanır."),
}


class _RepeatTimer:
    """Tekrarlayan zamanlayici (enjekte edilen timer servisi uzerinde)."""

    def __init__(self, ts, ms, fn):
        self._ts, self._ms, self._fn = ts, ms, fn
        self._h = None

    def start(self):
        self.stop()
        self._h = self._ts.call_later(self._ms, self._tick)

    def stop(self):
        if self._h is not None:
            self._h.cancel()
            self._h = None

    def _tick(self):
        self._h = self._ts.call_later(self._ms, self._tick)
        self._fn()


class ModState:
    """Bir modulun BU BAGLANTIDAKI bilinen durumu (kimlik + zaman damgasi)."""

    def __init__(self, sid):
        self.sid = sid
        self.known = False          # bu baglantida en az bir gecerli cevap verdi
        self.reachable = False
        self.fails = 0              # ardisik cevapsiz istek
        self.last_reply = None      # monoton zaman
        self.motors = [None] * MOTORS_PER_MODULE   # (harf, mm) veya None (bilinmiyor)
        self.stat_t = None          # son STAT'in ISTENME zamani (veri yasi buradan)
        self.complete = False       # son STAT 9 gecerli token mi
        self.u2err = False
        self.homed = False          # referans alma tamamlandi (firmware bildirimi)
        self.seen_non_I = False     # reset tespiti
        self.watch_t = None         # son hareket komutunun yazilmaya basladigi an
        self.motion_pending = False  # hareket komutu kuyrukta/hatta
        self.sync_need = False      # K11: senkron disi
        self.sync_prev = None
        self.sync_count = 0
        self.last_body = None       # gunluk sadelestirme
        # son KABUL EDILEN komut hedefi (motor basina mm; None = yok/belirsiz).
        # Olcum (motors) ile karistirilmaz; STAT bunu degistirmez.
        self.targets = [None] * MOTORS_PER_MODULE

    def letters(self):
        return [m[0] if m else None for m in self.motors]

    def moving(self):
        return any(l in ("M", "H") for l in self.letters())

    def motion_possible(self):
        """Hareket suruyor olabilir mi (komut yolda, M/H, veya komut sonrasi henuz
        durgunluk dogrulanmadi)."""
        return self.motion_pending or self.moving() or self.watch_t is not None


class Controller(QObject):
    # ---- sinyaller ----
    logMessage        = Signal(str)
    connectedChanged  = Signal()
    demoModeChanged   = Signal()
    portsChanged      = Signal()
    activeSlavesChanged = Signal()
    gridChanged       = Signal()
    statsChanged      = Signal()
    modelChanged      = Signal()
    currentSlaveIdChanged = Signal()
    stlUrlChanged     = Signal()
    busyChanged       = Signal()
    homedChanged      = Signal()
    faultChanged      = Signal()
    alarmsChanged     = Signal()
    gateChanged       = Signal()
    statusChanged     = Signal()
    motionChanged     = Signal()
    modulesChanged    = Signal()
    configChanged     = Signal()
    scanningChanged   = Signal()
    # faz metni, biten MOTOR, toplam motor, gecen sn, tahmini kalan sn (-1 bilinmiyor)
    opProgress        = Signal(str, int, int, int, int)
    operationFinished = Signal(bool, str, str)        # ok, baslik, mesaj
    # thread -> ana thread kopruleri
    _linesReady       = Signal()
    _callSig          = Signal(object)
    _jobDone          = Signal(int, bool, object)

    def __init__(self, demo=False, transport=None, timers=None, config=None):
        super().__init__()
        self._ts = timers if timers is not None else QtTimerService()
        self._demo = bool(demo)
        self._cfg = config if config is not None else load_machine_config()

        if transport is None:
            if self._demo:
                from sim_bridge import SimBridge
                transport = SimBridge(self._ts, modules=(1, 2, 3, 4), speed_mm_s=60.0)
            else:
                transport = SerialManager()
        self._transport = transport
        self._transport.on_line = self._on_transport_line          # herhangi thread
        self._transport.on_disconnect = self._on_transport_disconnect
        if hasattr(self._transport, "on_noise"):
            self._transport.on_noise = lambda m: self._post(lambda: self._log("UYARI: " + m))

        self._rx = deque()
        self._rx_lock = threading.Lock()
        self._rx_pending = False
        self._rx_dropped = 0
        self._linesReady.connect(self._drain_lines)
        self._callSig.connect(self._run_posted)
        self._jobDone.connect(self._on_job_done)

        self._link = SerialLink(self._ts, self._write_async, self._on_unsolicited)

        self._connected = False
        self._ports = []
        self._current_slave = 1
        self._mods = {}              # sid -> ModState (bu baglanti)
        self._scanning = False
        self._scan_left = 0
        self._poll_i = 0

        # --- model ---
        self._model = None           # {"name","path","grid"(tuple),"stats","info"}
        self._model_state = "idle"   # idle | loading | ready | error
        self._model_error = ""
        self._loading_name = ""
        self._load_seq = 0

        # --- islem ---
        self._op = None
        self._op_seq = 0

        # --- alarmlar ---
        self._alarms = {}            # key -> dict
        self._alarm_seq = 0

        # --- turetilmis durum (degisince sinyal) ---
        self._derived = {}
        self._mod_rev = 0

        self._tick_timer = _RepeatTimer(self._ts, POLL_TICK_MS, self._tick)

        for e in self._cfg.get("errors", []):
            self._log("YAPILANDIRMA: " + e)
        self.refreshPorts()
        self._changed()

    # ================= thread kopruleri =================
    def _post(self, fn):
        self._callSig.emit(fn)

    @Slot(object)
    def _run_posted(self, fn):
        fn()

    def _write_async(self, text, done_cb):
        out = self._link.outstanding
        quiet = out is not None and out.cmd == text and out.kind == "poll"
        self._log(f"→ {text}", quiet=quiet)
        self._transport.write_async(text, lambda ok: self._post(lambda: done_cb(ok)))

    def _on_transport_line(self, line):
        """Herhangi thread'den: sinirli kuyruga ekle, ana thread'e haber ver."""
        with self._rx_lock:
            if len(self._rx) >= SERIAL_LINE_QUEUE_MAX:
                self._rx.popleft()
                self._rx_dropped += 1
            self._rx.append(line)
            if self._rx_pending:
                return
            self._rx_pending = True
        self._linesReady.emit()

    @Slot()
    def _drain_lines(self):
        while True:
            with self._rx_lock:
                if not self._rx:
                    self._rx_pending = False
                    dropped, self._rx_dropped = self._rx_dropped, 0
                    break
                line = self._rx.popleft()
            if self._connected:
                self._link.on_line(line)
        if dropped:
            self._log(f"UYARI: seri satır kuyruğu taştı, {dropped} satır atıldı")

    def _on_transport_disconnect(self, reason):
        self._post(lambda: self._drop_link(manual=False, reason=reason))

    # ================= PROPERTIES =================
    def _get_connected(self): return self._connected
    connected = Property(bool, _get_connected, notify=connectedChanged)

    def _get_demo(self): return self._demo
    demoMode = Property(bool, _get_demo, notify=demoModeChanged)

    def _get_ports(self): return self._ports
    ports = Property("QVariantList", _get_ports, notify=portsChanged)

    def _get_active(self):
        return sorted(s for s, m in self._mods.items() if m.known and m.reachable)
    activeSlaves = Property("QVariantList", _get_active, notify=activeSlavesChanged)

    def _get_known(self):
        return sorted(s for s, m in self._mods.items() if m.known)
    knownSlaves = Property("QVariantList", _get_known, notify=activeSlavesChanged)

    def _get_grid(self):
        return list(self._model["grid"]) if self._model else []
    gridData = Property("QVariantList", _get_grid, notify=gridChanged)

    def _get_stats(self):
        return dict(self._model["stats"]) if self._model else {}
    stats = Property("QVariant", _get_stats, notify=statsChanged)

    def _get_model_name(self): return self._model["name"] if self._model else ""
    modelName = Property(str, _get_model_name, notify=modelChanged)

    def _get_model_state(self): return self._model_state
    modelState = Property(str, _get_model_state, notify=modelChanged)

    def _get_model_error(self): return self._model_error
    modelError = Property(str, _get_model_error, notify=modelChanged)

    def _get_loading_name(self): return self._loading_name
    loadingName = Property(str, _get_loading_name, notify=modelChanged)

    def _get_model_info(self):
        return dict(self._model["info"]) if self._model else {}
    modelInfo = Property("QVariant", _get_model_info, notify=modelChanged)

    def _get_stl_url(self):
        if self._model and self._model.get("path"):
            return QUrl.fromLocalFile(self._model["path"])
        return QUrl()
    stlFileUrl = Property(QUrl, _get_stl_url, notify=stlUrlChanged)

    def _get_current(self): return self._current_slave

    def _set_current(self, v):
        v = int(v)
        if v != self._current_slave and 1 <= v <= MAX_MODULES:
            self._current_slave = v
            self.currentSlaveIdChanged.emit()
            self._changed()
    currentSlaveId = Property(int, _get_current, _set_current, notify=currentSlaveIdChanged)

    def _get_busy(self): return self._op is not None
    busy = Property(bool, _get_busy, notify=busyChanged)

    def _get_scanning(self): return self._scanning
    scanning = Property(bool, _get_scanning, notify=scanningChanged)

    def _get_all_homed(self): return self._derived.get("allHomed", False)
    allHomed = Property(bool, _get_all_homed, notify=homedChanged)

    def _get_fault(self): return self._derived.get("faultText", "")
    faultText = Property(str, _get_fault, notify=faultChanged)

    def _get_alarms(self): return self._derived.get("alarms", [])
    alarms = Property("QVariantList", _get_alarms, notify=alarmsChanged)

    def _get_unacked(self): return self._derived.get("unacked", 0)
    unackedAlarms = Property(int, _get_unacked, notify=alarmsChanged)

    def _get_status(self): return self._derived.get("status", {})
    statusInfo = Property("QVariant", _get_status, notify=statusChanged)

    def _get_stale(self): return self._derived.get("stale", False)
    dataStale = Property(bool, _get_stale, notify=statusChanged)

    def _get_motion_active(self): return self._derived.get("motionActive", False)
    motionActive = Property(bool, _get_motion_active, notify=motionChanged)

    def _get_motion_locked(self):
        return self._op is not None or self._derived.get("motionActive", False)
    motionLocked = Property(bool, _get_motion_locked, notify=motionChanged)

    def _get_motion_text(self): return self._derived.get("motionText", "")
    motionText = Property(str, _get_motion_text, notify=motionChanged)

    # buton kapilari ("" = izinli, aksi halde operatore gosterilen sebep)
    def _g_prod(self): return self._derived.get("productionBlockReason", "")
    productionBlockReason = Property(str, _g_prod, notify=gateChanged)

    def _g_home(self): return self._derived.get("homeAllBlockReason", "")
    homeAllBlockReason = Property(str, _g_home, notify=gateChanged)

    def _g_sel(self): return self._derived.get("sendSelectedBlockReason", "")
    sendSelectedBlockReason = Property(str, _g_sel, notify=gateChanged)

    def _g_tmove(self): return self._derived.get("testerMoveBlockReason", "")
    testerMoveBlockReason = Property(str, _g_tmove, notify=gateChanged)

    def _g_thome(self): return self._derived.get("testerHomeBlockReason", "")
    testerHomeBlockReason = Property(str, _g_thome, notify=gateChanged)

    def _g_scan(self): return self._derived.get("scanBlockReason", "")
    scanBlockReason = Property(str, _g_scan, notify=gateChanged)

    def _g_link(self): return self._derived.get("linkBlockReason", "")
    linkBlockReason = Property(str, _g_link, notify=gateChanged)

    def _g_model(self): return self._derived.get("modelBlockReason", "")
    modelBlockReason = Property(str, _g_model, notify=gateChanged)

    def _get_plan(self): return self._derived.get("plan", {})
    productionPlan = Property("QVariant", _get_plan, notify=gateChanged)

    def _get_mod_rev(self): return self._mod_rev
    moduleRevision = Property(int, _get_mod_rev, notify=modulesChanged)

    def _get_expected(self):
        e = self._cfg.get("expected_modules")
        return list(e) if e else []
    expectedModules = Property("QVariantList", _get_expected, notify=configChanged)

    def _get_variant(self): return self._cfg.get("firmware_variant") or ""
    firmwareVariant = Property(str, _get_variant, notify=configChanged)

    def _get_cfg_errors(self): return list(self._cfg.get("errors", []))
    configErrors = Property("QVariantList", _get_cfg_errors, notify=configChanged)

    def _get_safe_min(self): return SAFE_MIN_MM
    safeMinMm = Property(int, _get_safe_min, constant=True)

    def _get_safe_max(self): return SAFE_MAX_MM
    safeMaxMm = Property(int, _get_safe_max, constant=True)

    def _get_tol(self): return TARGET_TOL_MM
    targetTolMm = Property(int, _get_tol, constant=True)

    # ================= LOG =================
    def _log(self, msg, quiet=False):
        if quiet:
            return
        ts = time.strftime("%H:%M:%S", time.localtime(self._ts.wall()))
        self.logMessage.emit(f"[{ts}] {msg}")

    # ================= PORTS =================
    @Slot()
    def refreshPorts(self):
        try:
            self._ports = [p["device"] for p in self._transport.list_ports()]
        except Exception as e:
            self._ports = []
            self._log(f"HATA: port listesi alınamadı: {e}")
        self.portsChanged.emit()
        self._log(f"Port listesi yenilendi: {len(self._ports)} port")

    # ================= BAGLAN / KES =================
    @Slot(str, int)
    @Slot(str)
    def connectPort(self, device, baud=115200):
        if self._connected:
            return
        if self._demo:
            device = "SIM"
        try:
            self._transport.open(device, baud)
        except Exception as e:
            self._log(f"HATA bağlantı: {e}")
            self._reject("Bağlanamadı",
                         f"{device or 'Port'} açılamadı: {e}\n"
                         "Adım: Portları yenileyip doğru portu seçin.")
            return
        self._mods = {}
        self._poll_i = 0
        self._link.start()
        self._connected = True
        self._resolve("LINK")
        self._tick_timer.start()
        self.connectedChanged.emit()
        self.activeSlavesChanged.emit()
        self._log(f"Bağlandı: {device} @ {baud}" + (" (SİMÜLASYON)" if self._demo else ""))
        self._changed()

    @Slot()
    def disconnectPort(self):
        self._drop_link(manual=True, reason="operatör kesti")

    def _drop_link(self, manual, reason):
        if not self._connected:
            return
        at_risk = {s for s, m in self._mods.items()
                   if m.known and (m.motion_possible() or not m.complete)}
        if self._op:
            at_risk |= set(self._op["modules"])
        at_risk = sorted(at_risk)
        if self._op:
            self._end_op(False, "Bağlantı kesildi",
                         "Bağlantı kesildi. Hareketin durduğu doğrulanamıyor.\n"
                         "Hareket otomatik sürdürülmez. Yeniden bağlandıktan sonra "
                         "'Modülleri bul' ile durum okunur; referans yeniden alınmalıdır.")
        self._tick_timer.stop()
        self._link.reset("link_lost")
        try:
            self._transport.close()
        except Exception:
            pass
        self._connected = False
        self._scanning = False
        self._mods = {}
        for sid in at_risk:
            self._raise(f"SYNC:{sid}", "SYNC", 1, sid, None,
                        f"Modül {sid}: bağlantı kesildi, hareketin durduğu doğrulanamıyor")
        if not manual:
            self._raise("LINK", "LINK", 2, None, None,
                        f"Bağlantı koptu ({reason}). Hareketin durduğu doğrulanamıyor")
        elif at_risk:
            self._raise("LINK", "LINK", 1, None, None,
                        "Bağlantı kesildi. Hareketin durduğu doğrulanamıyor")
        self._log("Bağlantı kesildi" if manual else f"HATA: bağlantı koptu: {reason}")
        self.connectedChanged.emit()
        self.activeSlavesChanged.emit()
        self.scanningChanged.emit()
        self._mod_rev += 1
        self.modulesChanged.emit()
        self._changed()

    # ================= TARAMA (PING) =================
    @Slot()
    def scan(self):
        reason = self._scan_block()
        if reason:
            self._reject("Tarama başlatılmadı", reason)
            return
        self._scanning = True
        self._scan_left = MAX_MODULES
        self.scanningChanged.emit()
        self._log("Modül taraması başladı (PING 01–16)")
        for sid in range(1, MAX_MODULES + 1):
            req = Request(P.cmd_ping(sid), "scan", PING_TIMEOUT_MS,
                          on_done=self._on_scan_done, sid=sid, guard_ms=PING_GUARD_MS)
            if not self._link.submit(req):
                self._scan_left -= 1
        self._changed()

    def _on_scan_done(self, req, status, reply):
        if status in ("link_lost", "cancelled") or req.gen != self._link.gen:
            return
        if status == "ok":
            self._log(f"← {reply.raw}")
            self._mark_reply(req.sid)
        elif status == "timeout":
            self._note_no_reply(req.sid)
        self._scan_left -= 1
        if self._scan_left <= 0 and self._scanning:
            self._scanning = False
            self.scanningChanged.emit()
            act = self._get_active()
            self._log(f"Tarama bitti: {len(act)} modül [{_ids_text(act)}]")
        self._changed()

    def _scan_block(self):
        if not self._connected:
            return "Bağlantı yok."
        if self._scanning:
            return "Modül taraması zaten sürüyor."
        if self._op:
            return "İşlem sürüyor."
        return ""

    # ================= MODUL DURUMU =================
    def _mod(self, sid):
        m = self._mods.get(sid)
        if m is None:
            m = ModState(sid)
            a = self._alarms.get(f"SYNC:{sid}")
            if a and a["active"]:
                m.sync_need = True        # baglanti/reset oncesi belirsizlik surer
            self._mods[sid] = m
        return m

    def _mark_reply(self, sid):
        m = self._mod(sid)
        newly = not (m.known and m.reachable)
        m.known = True
        m.reachable = True
        m.fails = 0
        m.last_reply = self._ts.now()
        if newly:
            self.activeSlavesChanged.emit()
        return m

    def _note_no_reply(self, sid):
        m = self._mods.get(sid)
        if m is None or not m.known:
            return
        m.fails += 1
        if m.fails >= COMM_FAIL_LIMIT and m.reachable:
            m.reachable = False
            self.activeSlavesChanged.emit()
            self._raise(f"COMM:{sid}", "COMM", 2, sid, None,
                        f"Modül {sid} cevap vermiyor (son geçerli durum {self._age_text(m)})")

    def _age_s(self, m):
        if m is None or m.stat_t is None:
            return None
        return max(0.0, self._ts.now() - m.stat_t)

    def _age_text(self, m):
        a = self._age_s(m)
        return "yok" if a is None else f"{int(a)} sn önce"

    def _variant(self):
        return self._cfg.get("firmware_variant")

    # ================= POLL / STAT =================
    def _tick(self):
        if not self._connected:
            return
        self._poll_next()
        self._check_op_timeout()
        self._emit_progress()
        self._mod_rev += 1
        self.modulesChanged.emit()
        self._changed()

    def _poll_next(self):
        """Gozetim: link bostayken tick basina TEK modul (round-robin, bilinen BUTUN
        moduller - erisilemeyenler dahil, geri gelirlerse gorulsun)."""
        if not self._link.idle() or self._scanning:
            return
        if self._op and self._op.get("dispatching"):
            return
        known = self._get_known()
        if not known:
            return
        self._poll_i %= len(known)
        sid = known[self._poll_i]
        self._poll_i = (self._poll_i + 1) % len(known)
        self._submit_stat(sid, "poll")

    def _submit_stat(self, sid, kind):
        if self._link.has(lambda r: r.verb == "STAT" and r.sid == sid):
            return True
        return self._link.submit(Request(P.cmd_stat(sid), kind, STAT_TIMEOUT_MS,
                                         on_done=self._on_stat_done, sid=sid))

    @Slot(int)
    def requestStatus(self, slave_id):
        """Secili modulun durumunu sorgula (kuyruga girer; hatta tek istek)."""
        if not self._connected:
            return
        if not self._submit_stat(int(slave_id), "query"):
            self._log("UYARI: seri kuyruk dolu, sorgu eklenmedi")

    def _on_stat_done(self, req, status, reply):
        if req.gen != self._link.gen or status in ("link_lost", "cancelled"):
            return
        if status == "ok":
            self._apply_stat(req.sid, reply, req)
        elif status == "timeout":
            self._log(f"Cevap yok: {req.cmd}")
            self._note_no_reply(req.sid)
        elif status == "write_failed":
            self._log(f"HATA: yazılamadı: {req.cmd}")
        self._changed()

    def _apply_stat(self, sid, reply, req):
        body = reply.fields[0] if reply.fields else ""
        prev = self._mods.get(sid)
        quiet = req.kind == "poll" and prev is not None and prev.last_body == body
        self._log(f"← {reply.raw}", quiet=quiet)
        info = P.interpret_stat(body, self._variant())
        m = self._mod(sid)
        m.last_body = body
        if info["complete"] or info["u2err"]:
            self._mark_reply(sid)
        else:
            m.last_reply = self._ts.now()     # cevap var ama icerik gecersiz
        m.motors = list(info["motors"])
        m.stat_t = req.t_sent if req.t_sent is not None else self._ts.now()
        m.complete = info["complete"]
        m.u2err = info["u2err"]
        letters = m.letters()

        # --- reset tespiti: firmware I'ya yalniz yeniden baslayinca doner ---
        if "I" in letters and (m.seen_non_I or m.homed):
            m.seen_non_I = False
            m.homed = False
            m.targets = [None] * MOTORS_PER_MODULE
            self._raise(f"SYNC:{sid}", "SYNC", 1, sid, None,
                        f"Modül {sid} yeniden başlamış olabilir (durum I). Referans geçersiz; "
                        "sayaçlar sıfırlandı, fiziksel konum bilinmiyor")
            m.sync_need = True
            m.sync_prev, m.sync_count = None, 0
            if self._op and sid in self._op["modules"]:
                self._end_op(False, "Modül yeniden başladı",
                             f"Modül {sid} yeniden başlamış olabilir (durum I).\n"
                             "Etki: Referans ve konum bilgisi geçersiz; işlem sonlandırıldı. "
                             "Diğer modüller izlenmeye devam ediyor.\n"
                             "Adım: Durum doğrulandıktan sonra referans alın.")
        if any(l in ("M", "H", "S", "F") for l in letters if l):
            m.seen_non_I = True

        # --- ariza (F) ---
        for i, mt in enumerate(m.motors):
            key = f"F:{sid}:{i + 1}"
            if mt is not None and mt[0] == "F":
                m.homed = False
                self._raise(key, "F", 2, sid, i + 1, f"MODÜL {sid} / MOTOR {i + 1} ARIZA")
            elif mt is not None:
                self._resolve(key)
        if info["unmapped_faults"]:
            m.homed = False
            why = ("firmware sürümü yapılandırılmadı" if not self._variant()
                   else "U2ERR biçimi yapılandırılan sürümle çelişiyor")
            self._raise(f"F:{sid}:?", "F", 2, sid, None,
                        f"MODÜL {sid} ARIZA (motor numarası belirlenemedi: {why})")
        elif m.complete and "F" not in letters:
            self._resolve(f"F:{sid}:?")

        # --- U2 / iletisim / gecersiz cevap ---
        if info["u2err"]:
            m.homed = False
            self._raise(f"U2:{sid}", "U2", 2, sid, None,
                        f"Modül {sid}: U2 kartı cevap vermiyor; "
                        f"{P.u2_motor_text(self._variant())} durumu bilinmiyor")
        elif m.complete:
            self._resolve(f"U2:{sid}")
        if m.complete:
            self._resolve(f"COMM:{sid}")
            self._resolve(f"DATA:{sid}")
        elif not info["u2err"]:
            self._raise(f"DATA:{sid}", "DATA", 1, sid, None,
                        f"Modül {sid}: geçersiz/eksik durum cevabı (motor durumu bilinmiyor)")

        # --- komut sonrasi durgunluk dogrulamasi ---
        if m.watch_t is not None and m.stat_t > m.watch_t and m.complete and not m.moving():
            m.watch_t = None

        # --- K11 senkron (ardisik tutarli, hareketsiz, tam STAT) ---
        if m.sync_need:
            stable = m.complete and not m.moving()
            if stable and m.sync_prev is not None and self._consistent(m.sync_prev, m.motors):
                m.sync_count += 1
            elif stable:
                m.sync_count = 1
            else:
                m.sync_count = 0
            m.sync_prev = list(m.motors) if stable else None
            if m.sync_count >= SYNC_CONFIRM:
                m.sync_need = False
                m.sync_prev, m.sync_count = None, 0
                self._resolve(f"SYNC:{sid}")

        self._op_on_stat(sid, m, req)
        self._mod_rev += 1
        self.modulesChanged.emit()

    @staticmethod
    def _consistent(a, b):
        for x, y in zip(a, b):
            if x is None or y is None or x[0] != y[0] or abs(x[1] - y[1]) > TARGET_TOL_MM:
                return False
        return True

    def _on_unsolicited(self, reply, line):
        """Bekleyen istege ait olmayan satir: veri olarak KULLANILMAZ (eski olabilir).
        Ama icindeki ariza (F) bilgisi kaybolmaz: alarm uretir (muhafazakar)."""
        self._log(f"← {line}  (beklenmeyen cevap; durum için kullanılmadı)")
        if reply is None or reply.verb != "STAT" or reply.sid is None:
            return
        info = P.interpret_stat(reply.fields[0] if reply.fields else "", self._variant())
        sid = reply.sid
        for i, mt in enumerate(info["motors"]):
            if mt is not None and mt[0] == "F":
                if sid in self._mods:
                    self._mods[sid].homed = False
                self._raise(f"F:{sid}:{i + 1}", "F", 2, sid, i + 1,
                            f"MODÜL {sid} / MOTOR {i + 1} ARIZA")
        if info["unmapped_faults"]:
            self._raise(f"F:{sid}:?", "F", 2, sid, None,
                        f"MODÜL {sid} ARIZA (motor numarası belirlenemedi)")
        self._changed()

    # ================= ALARMLAR =================
    def _raise(self, key, kind, level, sid, motor, text):
        a = self._alarms.get(key)
        if a is not None and a["active"]:
            if a["text"] != text or a["level"] < level:
                a["text"], a["level"] = text, max(a["level"], level)
            return
        self._alarm_seq += 1
        self._alarms[key] = {
            "key": key, "kind": kind, "level": int(level), "sid": sid or 0,
            "motor": motor or 0, "text": text, "active": True, "acked": False,
            "since": time.strftime("%H:%M:%S", time.localtime(self._ts.wall())),
            "resolvedAt": "", "seq": self._alarm_seq, "help": ALARM_HELP.get(kind, ""),
        }
        self._log(("ALARM: " if level >= 2 else "UYARI: ") + text)
        if kind == "F" and self._op is not None:
            faulty = sorted((x["sid"], x["motor"]) for x in self._alarms.values()
                            if x["active"] and x["kind"] == "F")
            lst = ", ".join(f"Modül {s} / Motor {mo}" if mo else f"Modül {s} (motor ?)"
                            for s, mo in faulty)
            self._end_op(False, "ARIZA",
                         f"Motor arızası bildirildi: {lst}.\n"
                         "Diğer motorların durduğu doğrulanmadı; makine izlenmeye devam ediyor.\n"
                         "Arıza kaydı, modülden arızasız taze durum gelince kapanır; "
                         "'Uyarıyı gördüm' kaydı silmez.")

    def _resolve(self, key):
        a = self._alarms.get(key)
        if a is None or not a["active"]:
            return
        a["active"] = False
        a["resolvedAt"] = time.strftime("%H:%M:%S", time.localtime(self._ts.wall()))
        self._log(f"Normale döndü (taze durum ile): {a['text']}")
        if a["acked"]:
            del self._alarms[key]

    @Slot()
    def acknowledgeAlarms(self):
        """'Uyarıyı gördüm': onay KAYDI SILMEZ. Yalniz normale donmus ve onaylanmis
        alarmlar listeden kalkar; aktif alarm onayli olarak gorunur kalir."""
        changed = False
        for key in list(self._alarms):
            a = self._alarms[key]
            if not a["acked"]:
                a["acked"] = True
                changed = True
            if not a["active"]:
                del self._alarms[key]
                changed = True
        if changed:
            self._log("Operatör uyarıları gördüğünü onayladı (aktif kayıtlar korunur)")
            self._changed()

    @Slot()
    def clearFault(self):
        """v2.0 uyumlulugu icin eski slot adi. v2.2: kaydi silmez, yalniz onaylar."""
        self.acknowledgeAlarms()

    def _active_alarms(self):
        return [a for a in self._alarms.values() if a["active"]]

    # ================= HAREKET YETKISI (TEK NOKTA) =================
    def _block_reason(self, kind, scope):
        """Hareket baslatilabilir mi? "" = evet; aksi halde operatore sebep.
        kind: production | home_all | send_selected | maint_move | maint_home"""
        if not self._connected:
            return "Bağlantı yok."
        if self._scanning:
            return "Modül taraması sürüyor."
        if self._op is not None:
            return "İşlem sürüyor."
        if self._link.has(lambda r: r.kind == "motion"):
            return "Önceki hareket komutunun kabulü bekleniyor."
        if kind in ("production", "send_selected"):
            if self._model_state == "loading":
                return "Model yükleniyor; hesaplama bitmeden başlatılamaz."
            if not self._model:
                return ("Model yok. STL Seç ile bir model yükleyin ve hesaplamanın "
                        "tamamlanmasını bekleyin.")
        if not scope:
            return "Modül bulunamadı. Önce 'Modülleri bul'."
        if kind == "production":
            exp = self._cfg.get("expected_modules")
            if exp:
                missing = sorted(set(exp) - set(scope))
                extra = sorted(set(scope) - set(exp))
                if missing:
                    return f"Beklenen modüller eksik: {_ids_text(missing)}."
                if extra:
                    return f"Beklenmeyen modül bulundu: {_ids_text(extra)} (machine.json)."
        for sid in scope:
            m = self._mods.get(sid)
            if m is None or not m.known or not m.reachable:
                return f"Modül {sid} erişilemiyor."
        act = self._active_alarms()
        faults = [a for a in act if a["kind"] == "F"]
        if faults:
            a = min(faults, key=lambda x: x["seq"])
            return (f"Aktif arıza: {a['text']}. Kayıt, modülden arızasız taze durum "
                    "gelince kapanır.")
        other = [a for a in act if a["kind"] in ("U2", "COMM", "SYNC", "LINK", "DATA")]
        if other:
            a = min(other, key=lambda x: x["seq"])
            return f"Çözülmemiş uyarı: {a['text']}."
        issue = self._motion_issue()
        if issue:
            return issue
        if kind in ("send_selected", "maint_move"):
            for sid in scope:
                if not self._mods[sid].homed:
                    return f"Modül {sid} referanssız. Önce referans alın."
        return ""

    def _motion_issue(self):
        """Makine durgun ve durumu guncel mi? Bilinen BUTUN modullere bakar
        (tek parca membran: bir bolge hareket ederken baska bolge baslatilmaz)."""
        known = [self._mods[s] for s in self._get_known()]
        thr = data_stale_s(len(known))
        for m in known:
            if m.motion_pending:
                return f"Modül {m.sid}: hareket komutu gönderiliyor."
            if m.moving():
                return f"Modül {m.sid}: hareket sürüyor."
            if m.watch_t is not None:
                return f"Modül {m.sid}: hareket sonrası durgunluk henüz doğrulanmadı."
        for m in known:
            if m.stat_t is None:
                return f"Modül {m.sid}: durum henüz okunmadı."
            if not m.complete:
                return f"Modül {m.sid}: motor durumu kısmen bilinmiyor."
            if self._age_s(m) > thr:
                return f"Modül {m.sid}: durum güncel değil ({self._age_text(m)})."
            if m.sync_need:
                return f"Modül {m.sid}: durum doğrulanıyor."
        return ""

    def _authorize_motion(self, kind, scope, title):
        """TEK hareket yetkilendirme noktasi. "" = izinli."""
        reason = self._block_reason(kind, list(scope))
        if reason:
            self._log(f"REDDEDİLDİ ({title}): {reason}")
        return reason

    def _reject(self, title, reason):
        self.operationFinished.emit(False, title, reason)

    # ================= URETIM / HOME / SECILI =================
    @Slot()
    def startProduction(self):
        """Ana uretim akisi. Referans alinmamis modul varsa ONCE butun modullerde
        referans alinir, sonra model sekli uygulanir."""
        scope = self._get_active()
        reason = self._authorize_motion("production", scope, "Üretim")
        if reason:
            self._reject("Üretim başlatılmadı", reason)
            return
        grid = self._model["grid"]           # degismez kopya (tuple)
        phases = []
        if any(not self._mods[s].homed for s in scope):
            phases.append(self._phase_home(scope))
        phases.append(self._phase_arr({s: slave_values(grid, s) for s in scope}))
        self._start_op("production", phases, scope, self._model["name"])

    @Slot()
    def startHomeAll(self):
        scope = self._get_active()
        reason = self._authorize_motion("home_all", scope, "Referans")
        if reason:
            self._reject("Referans alma başlatılmadı", reason)
            return
        self._start_op("home_all", [self._phase_home(scope)], scope, "")

    @Slot(int)
    def startSendSelected(self, slave_id):
        sid = int(slave_id)
        reason = self._authorize_motion("send_selected", [sid], "Seçili modül")
        if reason:
            self._reject("Seçili modüle uygulanmadı", reason)
            return
        vals = slave_values(self._model["grid"], sid)
        self._start_op("send_selected", [self._phase_arr({sid: vals})], [sid],
                       self._model["name"])

    @Slot(int)
    def sendArrayToSlave(self, slave_id):
        self.startSendSelected(slave_id)

    # ================= BAKIM (tek modul) =================
    def _maint(self, kind, sid, phase, title):
        reason = self._authorize_motion(kind, [sid], title)
        if reason:
            return reason
        self._start_op("maint", [phase], [sid], "")
        return ""

    def _maint_mov(self, sid, motor, mm):
        err = P.check_target_value(mm)
        if err:
            return err
        if not 1 <= motor <= MOTORS_PER_MODULE:
            return "Motor numarası 1–9 olmalı."
        exp = [None] * MOTORS_PER_MODULE
        exp[motor - 1] = (int(mm), TARGET_TOL_MM)
        ph = self._phase("MOV", {sid: (P.cmd_mov(sid, motor, mm), motor)}, {sid: exp},
                         f"Modül {sid} / Motor {motor} → {int(mm)} mm", others=("S", "I"))
        return self._maint("maint_move", sid, ph, "Bakım: tek motor")

    def _maint_all(self, sid, mm):
        err = P.check_target_value(mm)
        if err:
            return err
        exp = [(int(mm), TARGET_TOL_MM)] * MOTORS_PER_MODULE
        ph = self._phase("ALL", {sid: (P.cmd_all(sid, mm), None)}, {sid: exp},
                         f"Modül {sid}: 9 motor → {int(mm)} mm")
        return self._maint("maint_move", sid, ph, "Bakım: 9 motor")

    def _maint_arr(self, sid, vals):
        for v in vals:
            err = P.check_target_value(v)
            if err:
                return err
        ph = self._phase_arr({sid: list(vals)}, label=f"Modül {sid}: 9 ayrı hedef")
        return self._maint("maint_move", sid, ph, "Bakım: ARR")

    def _maint_home(self, sid, motor=0):
        if motor:
            if not 1 <= motor <= MOTORS_PER_MODULE:
                return "Motor numarası 1–9 olmalı."
            exp = [None] * MOTORS_PER_MODULE
            exp[motor - 1] = (0, HOME_ZERO_TOL_MM)
            ph = self._phase("HOMEM", {sid: (P.cmd_home(sid, motor), motor)}, {sid: exp},
                             f"Modül {sid} / Motor {motor}: referans alınıyor",
                             others=("S", "I"))
        else:
            ph = self._phase_home([sid], label=f"Modül {sid}: referans alınıyor")
        return self._maint("maint_home", sid, ph, "Bakım: referans")

    @Slot(int, int, int)
    def moveMotor(self, slave_id, motor_id, mm):
        r = self._maint_mov(int(slave_id), int(motor_id), mm)
        if r:
            self._reject("Hareket başlatılmadı", r)

    @Slot(int, int)
    def allToValue(self, slave_id, mm):
        r = self._maint_all(int(slave_id), mm)
        if r:
            self._reject("Hareket başlatılmadı", r)

    @Slot(int)
    def homeSlave(self, slave_id):
        r = self._maint_home(int(slave_id))
        if r:
            self._reject("Referans alma başlatılmadı", r)

    @Slot(int, int)
    def homeMotor(self, slave_id, motor_id):
        r = self._maint_home(int(slave_id), int(motor_id))
        if r:
            self._reject("Referans alma başlatılmadı", r)

    @Slot(str, result=str)
    def checkTarget(self, text):
        """Bakim hedef girisi dogrulamasi ("" = gecerli)."""
        return P.check_target(text)

    # ================= TERMINAL =================
    @Slot(str, result=str)
    def sendRaw(self, text):
        """Terminal. Fiil ayristirilir: PING/STAT/GETPOS/GETID serbest (kuyruktan);
        ARR/ALL/MOV/HOME hareket yetkisinden gecer; SETID kilitli; digerleri red.
        Donus: "" = kabul, aksi halde sebep."""
        c = P.parse_terminal(text)
        if c.error:
            self._log(f"REDDEDİLDİ (terminal) {c.text or '(boş)'}: {c.error}")
            return c.error
        if not self._connected:
            self._log("REDDEDİLDİ (terminal): bağlantı yok")
            return "Bağlantı yok."
        v = c.verb
        if v == "STAT":
            return "" if self._submit_stat(c.sid, "raw") else "Seri kuyruk dolu."
        if v in ("PING", "GETID", "GETPOS"):
            timeout = PING_TIMEOUT_MS if v == "PING" else QUERY_TIMEOUT_MS
            ok = self._link.submit(Request(c.text, "raw", timeout, on_done=self._on_raw_done,
                                           sid=c.sid, motor=c.motor))
            return "" if ok else "Seri kuyruk dolu."
        if v == "MOV":
            r = self._maint_mov(c.sid, c.motor, c.values[0])
        elif v == "ALL":
            r = self._maint_all(c.sid, c.values[0])
        elif v == "ARR":
            r = self._maint_arr(c.sid, c.values)
        else:  # HOME
            r = self._maint_home(c.sid, c.motor or 0)
        if r:
            self._log(f"REDDEDİLDİ (terminal) {c.text}: {r}")
        return r

    def _on_raw_done(self, req, status, reply):
        if req.gen != self._link.gen or status in ("link_lost", "cancelled"):
            return
        if status == "ok":
            self._log(f"← {reply.raw}")
            if reply.verb == "PONG":
                self._mark_reply(req.sid)
        elif status == "timeout":
            self._log(f"Cevap yok: {req.cmd}")
        elif status == "write_failed":
            self._log(f"HATA: yazılamadı: {req.cmd}")
        self._changed()

    # ================= ISLEM (OP) =================
    def _phase(self, verb, cmds, expect, label, others=("S",)):
        return {"verb": verb, "cmds": dict(cmds), "expect": dict(expect), "label": label,
                "others": tuple(others), "order": sorted(cmds)}

    def _phase_home(self, scope, label="Referans alınıyor"):
        exp = [(0, HOME_ZERO_TOL_MM)] * MOTORS_PER_MODULE
        return self._phase("HOME", {s: (P.cmd_home(s), 0) for s in scope},
                           {s: list(exp) for s in scope}, label)

    def _phase_arr(self, targets, label="Şekil uygulanıyor"):
        cmds = {s: (P.cmd_arr(s, v), None) for s, v in targets.items()}
        exp = {s: [(int(x), TARGET_TOL_MM) for x in v] for s, v in targets.items()}
        return self._phase("ARR", cmds, exp, label)

    def _start_op(self, kind, phases, modules, model_name):
        self._op_seq += 1
        self._op = {
            "id": self._op_seq, "kind": kind, "phases": list(phases), "pi": -1,
            "modules": sorted(modules), "model": model_name,
            "t0": 0.0, "di": 0, "dispatching": False, "ack": {}, "done": set(),
            "progress": {},
        }
        self._log(f"İşlem başladı #{self._op_seq}: {kind} — modül {_ids_text(modules)}"
                  + (f" — model {model_name}" if model_name else ""))
        self.busyChanged.emit()
        self._next_phase()
        self._changed()

    def _next_phase(self):
        op = self._op
        if op is None:
            return
        op["pi"] += 1
        if op["pi"] >= len(op["phases"]):
            self._finish_success()
            return
        ph = op["phases"][op["pi"]]
        op["t0"] = self._ts.now()
        op["di"] = 0
        op["ack"] = {}
        op["done"] = set()
        op["progress"] = {}
        op["dispatching"] = True
        self._log(f"{ph['label']} ({op['pi'] + 1}/{len(op['phases'])}) — "
                  f"{len(ph['order'])} modül")
        self._emit_progress()
        self._dispatch_next(op["id"], op["pi"])

    def _dispatch_next(self, op_id, pi):
        """Moduller SIRAYLA: bir modulun komutu kabul edilmeden (ACK) sonrakine
        gecilmez (80 ms kor aralik yok). Callback islem+faz nesli kontrol eder."""
        op = self._op
        if op is None or op["id"] != op_id or op["pi"] != pi:
            return                       # eski islem/faz: SIZMA YOK
        ph = op["phases"][pi]
        if op["di"] >= len(ph["order"]):
            op["dispatching"] = False
            return
        sid = ph["order"][op["di"]]
        cmd, motor = ph["cmds"][sid]
        m = self._mod(sid)
        m.motion_pending = True
        req = Request(cmd, "motion", MOTION_ACK_TIMEOUT_MS, sid=sid, motor=motor,
                      tag=(op_id, pi), on_done=self._on_motion_done)
        if not self._link.submit(req):
            m.motion_pending = False
            self._end_op(False, "Komut gönderilemedi",
                         "Seri kuyruk dolu veya bağlantı yok; komut gönderilmedi.\n"
                         f"Etki: Modül {_ids_text(ph['order'][op['di']:])} için hareket "
                         "başlatılmadı.")

    def _on_motion_done(self, req, status, reply):
        sid = req.sid
        m = self._mods.get(sid)
        stale_gen = req.gen != self._link.gen
        # ---- modul seviyesinde etki (islem bitmis olsa bile) ----
        if m is not None and not stale_gen:
            m.motion_pending = False
            if req.t_sent is not None and status != "cancelled":
                m.watch_t = req.t_sent     # hareket baslamis olabilir -> izle
        if status == "ok":
            self._log(f"← {reply.raw}")
        ok = status == "ok" and P.is_ack_ok(reply)
        if not stale_gen and m is not None and not ok and \
                status not in ("cancelled", "link_lost"):
            if status == "ok":         # *ERR
                why = (f"Modül {sid}: {reply.verb} — U2 komutu onaylamadı; "
                       "hareketin başlamadığı varsayılamaz")
            elif status == "timeout":
                why = (f"Modül {sid}: {req.verb} kabulü doğrulanamadı (cevap yok); "
                       "komut ulaşmış olabilir")
            else:
                why = f"Modül {sid}: {req.verb} yazılamadı; komutun ulaşıp ulaşmadığı belirsiz"
            m.sync_need = True
            m.sync_prev, m.sync_count = None, 0
            if req.verb == "HOME":
                m.homed = False
            # kabul belirsiz: hangi motorun hangi hedefe gittigi bilinmiyor
            if req.verb == "MOV" and req.motor:
                m.targets[req.motor - 1] = None
            else:
                m.targets = [None] * MOTORS_PER_MODULE
            self._raise(f"SYNC:{sid}", "SYNC", 1, sid, None, why)
        # ---- islem seviyesinde etki ----
        op = self._op
        if op is None or stale_gen or req.tag is None or req.tag[0] != op["id"] \
                or req.tag[1] != op["pi"]:
            self._changed()
            return
        if ok:
            op["ack"][sid] = self._ts.now()
            if m is not None:
                exp = op["phases"][op["pi"]]["expect"][sid]
                for i, e in enumerate(exp):
                    if e is not None:
                        m.targets[i] = e[0]
            op["di"] += 1
            self._dispatch_next(op["id"], op["pi"])
            self._changed()
            return
        u2txt = P.u2_motor_text(self._variant())
        if status == "ok":
            self._end_op(False, "Komut kısmen kabul edildi",
                         f"Modül {sid}: {reply.raw}. Neden: modülün ikinci kartı (U2) "
                         "komutu onaylamadı.\n"
                         f"Etki: {u2txt} komutu almamış olabilir; diğer motorlar hareket "
                         "etmiş olabilir. Hareketin başlamadığı varsayılamaz. Kalan "
                         "modüllere komut gönderilmedi.\n"
                         "Adım: Durum izleniyor; hareket bitip durum doğrulanana kadar "
                         "yeni hareket kilitli. Sorun sürerse kart bağlantısı bakım "
                         "personelince incelenmeli.")
        elif status == "timeout":
            self._end_op(False, "Kabul doğrulanamadı",
                         f"Modül {sid} komuta {MOTION_ACK_TIMEOUT_MS / 1000:.1f} sn içinde "
                         "cevap vermedi.\n"
                         "Etki: Komut ulaşmış ve hareket başlamış olabilir. Komut otomatik "
                         "olarak tekrar gönderilmez; kalan modüllere komut gönderilmedi.\n"
                         "Adım: Modül izleniyor; durum iki ardışık sorguda tutarlı gelene "
                         "kadar hareket kilitli.")
        elif status == "write_failed":
            self._end_op(False, "Komut gönderilemedi",
                         f"Seri porta yazılamadı ({req.cmd}).\n"
                         "Etki: Komutun modüle ulaşıp ulaşmadığı belirsiz; kalan modüllere "
                         "komut gönderilmedi.\n"
                         "Adım: Bağlantıyı kontrol edin; durum doğrulanana kadar hareket kilitli.")
        self._changed()

    def _module_meets(self, ph, sid, m):
        exp = ph["expect"][sid]
        for i, mt in enumerate(m.motors):
            if mt is None:
                return False
            letter, mm = mt
            e = exp[i]
            if e is None:
                if letter not in ph["others"]:
                    return False
                continue
            target, tol = e
            if letter != "S" or abs(mm - target) > tol:
                return False
        return True

    def _op_on_stat(self, sid, m, req):
        op = self._op
        if op is None or op["pi"] < 0:
            return
        ph = op["phases"][op["pi"]]
        if sid not in ph["cmds"]:
            return
        ack_t = op["ack"].get(sid)
        if ack_t is None or req.t_sent is None or req.t_sent <= ack_t:
            return                       # ACK'ten once istenmis STAT tamamlanma sayilmaz
        if not m.complete:
            op["done"].discard(sid)
            op["progress"][sid] = 0
            self._emit_progress()
            return
        cnt = 0
        for i, mt in enumerate(m.motors):
            e = ph["expect"][sid][i]
            if e is None or (mt[0] == "S" and abs(mt[1] - e[0]) <= e[1]):
                cnt += 1
        op["progress"][sid] = cnt
        if self._module_meets(ph, sid, m):
            op["done"].add(sid)
            if ph["verb"] == "HOME":
                m.homed = True
        else:
            op["done"].discard(sid)
        self._emit_progress()
        if not op["dispatching"] and len(op["done"]) == len(ph["cmds"]):
            self._next_phase()

    def _check_op_timeout(self):
        op = self._op
        if op is None or op["pi"] < 0:
            return
        if self._ts.now() - op["t0"] <= OP_PHASE_TIMEOUT_S:
            return
        ph = op["phases"][op["pi"]]
        pend = [s for s in ph["order"] if s not in op["done"]]
        parts = []
        for sid in pend:
            m = self._mods.get(sid)
            last = "bilinmiyor"
            if m is not None and m.stat_t is not None:
                ls = [l for l in m.letters() if l]
                last = ", ".join(sorted({STATE_TEXT.get(l, l) for l in ls})) or "bilinmiyor"
            parts.append(f"Modül {sid}: son durum {last}, {self._age_text(m)}")
            if m is not None:
                m.sync_need = True
                m.sync_prev, m.sync_count = None, 0
                if ph["verb"] in ("HOME", "HOMEM"):
                    m.homed = False
            self._raise(f"SYNC:{sid}", "SYNC", 1, sid, None,
                        f"Modül {sid}: süre içinde tamamlanma doğrulanamadı")
        self._end_op(False, "Zaman aşımı",
                     "Süre içinde tamamlanma doğrulanamadı.\n" + "\n".join(parts) +
                     "\nHareket sürüyor olabilir; izleme devam ediyor.")

    def _emit_progress(self):
        op = self._op
        if op is None or op["pi"] < 0:
            return
        ph = op["phases"][op["pi"]]
        total = len(ph["order"]) * MOTORS_PER_MODULE
        done = 0
        for sid in ph["order"]:
            if sid in op["done"]:
                done += MOTORS_PER_MODULE
            else:
                done += op["progress"].get(sid, 0)
        elapsed = int(self._ts.now() - op["t0"])
        remaining = -1
        if done > 0 and elapsed > 3:
            frac = done / float(total)
            if frac >= 0.05:
                remaining = max(0, int(elapsed * (1.0 - frac) / frac))
        self.opProgress.emit(f"{ph['label']} ({op['pi'] + 1}/{len(op['phases'])})",
                             done, total, elapsed, remaining)

    def _finish_success(self):
        op = self._op
        last = op["phases"][-1]
        mods = op["modules"]
        n = len(mods)
        if last["verb"] == "HOME":
            msg = (f"Referans alma tamamlandı (firmware bildirimi): Modül {_ids_text(mods)}.\n"
                   "Referansın fiziksel doğruluğu bu arayüzden doğrulanamaz.")
        elif last["verb"] == "HOMEM":
            sid = mods[0]
            motor = last["cmds"][sid][1]
            msg = (f"Modül {sid} / Motor {motor}: referans alma tamamlandı "
                   "(firmware bildirimi).")
        elif last["verb"] == "MOV":
            sid = mods[0]
            motor = last["cmds"][sid][1]
            tgt = last["expect"][sid][motor - 1][0]
            msg = (f"Modül {sid} / Motor {motor} hedefe ulaştı ({tgt} mm; son durum "
                   f"sorgusu, ±{TARGET_TOL_MM} mm).")
        else:
            scope = (f"Modül {_ids_text(mods)} ({n * MOTORS_PER_MODULE} motor)"
                     if op["kind"] == "production" else
                     f"Seçili {n} modülün {n * MOTORS_PER_MODULE} motoru")
            msg = (f"{scope} hedefe ulaştı (son durum sorgusu, ±{TARGET_TOL_MM} mm).\n"
                   "Bu, kalıp alanına güvenli erişim anlamına gelmez.")
        self._end_op(True, "Tamamlandı", msg)

    def _end_op(self, ok, title, msg):
        op = self._op
        if op is None:
            return
        oid = op["id"]
        self._op = None
        self._link.cancel(lambda r: r.kind == "motion" and r.tag and r.tag[0] == oid)
        for m in self._mods.values():
            if m.motion_pending and not self._link.has(
                    lambda r, s=m.sid: r.kind == "motion" and r.sid == s):
                m.motion_pending = False
        self.busyChanged.emit()
        self._log(("TAMAM: " if ok else "HATA: ") + title + " — " + msg.replace("\n", " "))
        self.operationFinished.emit(bool(ok), title, msg)
        self._changed()

    # ================= MODEL (STL) =================
    @Slot(QUrl)
    @Slot(str)
    def loadStl(self, file):
        path = file.toLocalFile() if isinstance(file, QUrl) else str(file)
        if path.startswith("file:"):
            path = QUrl(path).toLocalFile()
        reason = self._model_block()
        if reason:
            self._reject("Model yüklenmedi", reason)
            return
        name = os.path.basename(path) if path else ""
        self._load_seq += 1
        lid = self._load_seq
        self._model = None                 # eski hedefler YENI adla gosterilmez
        self._model_state = "loading"
        self._model_error = ""
        self._loading_name = name
        self._emit_model()
        self._log(f"STL hesaplanıyor: {name}")
        if not path or not os.path.exists(path):
            self._on_job_done(lid, False, f"Dosya bulunamadı: {path}")
            return

        def worker():
            try:
                grid, _mesh, info = interpolate_stl_to_grid_info(path)
                g = tuple(int(v) for v in grid.tolist())
                if len(g) != 144:
                    raise ValueError("grid 144 hücre değil")
                self._jobDone.emit(lid, True, {"name": name, "path": path, "grid": g,
                                               "stats": grid_stats(g), "info": info})
            except Exception as e:
                self._jobDone.emit(lid, False, f"{e.__class__.__name__}: {e}")
        threading.Thread(target=worker, daemon=True).start()

    def _model_block(self):
        if self._model_state == "loading":
            return "Model yükleniyor; bitmesini bekleyin."
        if self._op is not None:
            return "İşlem sürüyor; model değiştirilemez."
        return ""

    @Slot(int, bool, object)
    def _on_job_done(self, lid, ok, payload):
        if lid != self._load_seq:
            self._log("Eski model hesaplaması yok sayıldı")
            return
        if ok:
            self._model = payload
            self._model_state = "ready"
            self._model_error = ""
            s, i = payload["stats"], payload["info"]
            self._log(f"Model hazır: {payload['name']} — {s['min']}–{s['max']} mm "
                      f"(ort {s['mean']}); doldurulan {i['filled']}, kırpılan "
                      f"{i['clipped_low'] + i['clipped_high']} hücre")
        else:
            self._model = None
            self._model_state = "error"
            self._model_error = str(payload)
            self._log(f"HATA: STL hesaplanamadı: {self._loading_name}: {payload}")
            self.operationFinished.emit(
                False, "Model yüklenemedi",
                f"{self._loading_name}: dosya STL olarak okunamadı veya hesaplanamadı.\n"
                f"Ayrıntı: {payload}\n"
                "Etki: Yüklü model yok; BAŞLA kapalı.\n"
                "Adım: STL Seç ile geçerli bir dosya seçin.")
        self._emit_model()

    def _emit_model(self):
        self.modelChanged.emit()
        self.gridChanged.emit()
        self.statsChanged.emit()
        self.stlUrlChanged.emit()
        self._changed()

    def _set_model_for_test(self, name, grid):
        g = tuple(int(v) for v in grid)
        self._load_seq += 1
        self._model = {"name": name, "path": "", "grid": g, "stats": grid_stats(g),
                       "info": {"cells": 144, "filled": 0, "clipped_low": 0,
                                "clipped_high": 0, "offset_mm": 0.0, "max_neighbor": 0}}
        self._model_state = "ready"
        self._emit_model()

    # ================= QML YARDIMCILARI =================
    @Slot(int, result="QVariant")
    def moduleInfo(self, slave_id):
        sid = int(slave_id)
        m = self._mods.get(sid)
        motors = []
        for i in range(MOTORS_PER_MODULE):
            mt = m.motors[i] if m else None
            tgt = m.targets[i] if m else None
            motors.append({"known": mt is not None, "letter": mt[0] if mt else "",
                           "mm": mt[1] if mt else -1,
                           "target": -1 if tgt is None else int(tgt),
                           "fault": bool(mt and mt[0] == "F")})
        age = self._age_s(m)
        thr = data_stale_s(max(1, len(self._get_known())))
        return {
            "sid": sid, "known": bool(m and m.known), "reachable": bool(m and m.reachable),
            "homed": bool(m and m.homed), "u2Error": bool(m and m.u2err),
            "complete": bool(m and m.complete), "ageS": -1 if age is None else int(age),
            "stale": age is None or age > thr, "syncing": bool(m and m.sync_need),
            "moving": bool(m and m.moving()), "motors": motors,
            "alarm": any(a["active"] and a["sid"] == sid for a in self._alarms.values()),
        }

    @Slot(int, result="QVariantList")
    def moduleTargets(self, slave_id):
        """Yuklu modelin bu modul icin hedefleri ([] = model yok)."""
        if not self._model:
            return []
        return slave_values(self._model["grid"], int(slave_id))

    # ================= TURETILMIS DURUM =================
    def _changed(self):
        d = {}
        known = self._get_known()
        active = self._get_active()
        cur = [self._current_slave]
        d["allHomed"] = bool(active) and all(self._mods[s].homed for s in active)

        act = self._active_alarms()
        alarms = sorted(self._alarms.values(),
                        key=lambda a: (not a["active"], -a["level"], a["seq"]))
        d["alarms"] = [dict(a) for a in alarms]
        d["unacked"] = sum(1 for a in self._alarms.values() if not a["acked"])
        faults = sorted([a for a in act if a["level"] >= 2], key=lambda a: a["seq"])
        if faults:
            t = f"{faults[0]['text']}  ·  {faults[0]['since']}"
            if len(faults) > 1:
                t += f"  (+{len(faults) - 1})"
            d["faultText"] = t
        else:
            d["faultText"] = ""

        # veri yasi
        thr = data_stale_s(len(known))
        stale = False
        oldest = None
        if self._connected and known:
            for s in known:
                a = self._age_s(self._mods[s])
                if a is None or a > thr:
                    stale = True
                if a is not None:
                    oldest = a if oldest is None else max(oldest, a)
        d["stale"] = stale

        # hareket (olasi)
        mtext = ""
        if self._connected:
            for s in known:
                if self._mods[s].motion_possible():
                    mtext = f"Modül {s}: hareket sürüyor veya durgunluk doğrulanmadı"
                    break
        d["motionActive"] = bool(mtext) or self._op is not None
        d["motionText"] = mtext
        d["busyFlag"] = self._op is not None

        # kapilar
        d["productionBlockReason"] = self._block_reason("production", active)
        d["homeAllBlockReason"] = self._block_reason("home_all", active)
        d["sendSelectedBlockReason"] = self._block_reason("send_selected", cur)
        d["testerMoveBlockReason"] = self._block_reason("maint_move", cur)
        d["testerHomeBlockReason"] = self._block_reason("maint_home", cur)
        d["scanBlockReason"] = self._scan_block()
        # Baglantiyi kesmek yalniz islem surerken kapali: hareket olasiyken kesmek
        # gozetimi kaybettirir ama cevapsiz modul yuzunden port degisimi kilitlenmesin.
        d["linkBlockReason"] = "İşlem sürüyor." if self._op is not None else ""
        d["modelBlockReason"] = self._model_block()
        exp = self._cfg.get("expected_modules")
        d["plan"] = {
            "model": self._model["name"] if self._model else "",
            "modules": list(active), "motors": len(active) * MOTORS_PER_MODULE,
            "needHome": [s for s in active if not self._mods[s].homed],
            "expected": list(exp) if exp else [], "expectedConfigured": bool(exp),
            "missing": sorted(set(exp) - set(active)) if exp else [],
        }

        d["status"] = self._status(act, known, stale, oldest, d)

        old = self._derived
        self._derived = d

        def ch(*keys):
            return any(old.get(k) != d.get(k) for k in keys)
        if ch("allHomed"):
            self.homedChanged.emit()
        if ch("faultText"):
            self.faultChanged.emit()
        if ch("alarms", "unacked"):
            self.alarmsChanged.emit()
        if ch("status", "stale"):
            self.statusChanged.emit()
        if ch("motionActive", "motionText", "busyFlag"):
            self.motionChanged.emit()
        if ch("productionBlockReason", "homeAllBlockReason", "sendSelectedBlockReason",
              "testerMoveBlockReason", "testerHomeBlockReason", "scanBlockReason",
              "linkBlockReason", "modelBlockReason", "plan"):
            self.gateChanged.emit()

    def _status(self, act, known, stale, oldest, d):
        """Durum seridi: oncelik sirali TEK metin (ISA-101: anormal durum tek yerde)."""
        lvl2 = sorted([a for a in act if a["level"] >= 2], key=lambda a: a["seq"])
        lvl1 = sorted([a for a in act if a["level"] == 1], key=lambda a: a["seq"])
        rtn = [a for a in self._alarms.values() if not a["active"] and not a["acked"]]
        n_act = len(act)

        def more(n):
            return f"  (+{n - 1})" if n > 1 else ""
        if lvl2:
            return {"level": 2, "text": lvl2[0]["text"] + more(n_act)}
        if lvl1:
            return {"level": 1, "text": lvl1[0]["text"] + more(n_act)}
        if not self._connected:
            return {"level": 1, "text": "BAĞLANTI YOK — cihaza bağlanın"}
        if rtn:
            return {"level": 1, "text": "Giderildi, onay bekliyor: " + rtn[0]["text"]}
        if self._scanning:
            return {"level": 0, "text": "Modüller aranıyor…"}
        if not known:
            return {"level": 1, "text": "MODÜL BULUNAMADI — 'Modülleri bul'"}
        if stale:
            return {"level": 1, "text": "Güncel makine durumu bilinmiyor."}
        if d["motionActive"]:
            return {"level": 0, "text": "Hareket sürüyor — bu arayüzden durdurulamıyor"}
        if not d["allHomed"]:
            return {"level": 1, "text": "Referans alınmadı — hareketten önce referans "
                                        "alınmalı (BAŞLA bunu kendisi yapar)"}
        if not self._cfg.get("expected_modules"):
            return {"level": 1, "text": "Beklenen modül listesi yapılandırılmadı "
                                        "(config/machine.json)"}
        age = int(oldest) if oldest is not None else 0
        return {"level": 0, "text": f"Anormal durum yok  ·  en eski durum {age} sn önce"}
