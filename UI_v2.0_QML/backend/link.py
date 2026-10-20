"""
Tek seri islem yoneticisi (SerialLink).

Butun seri trafik buradan gecer: gozetim poll'u, tarama, hareket dagitimi, Bakim
ekrani sorgulari ve terminal. Kurallar:
  - Hatta AYNI ANDA TEK bekleyen istek (yari cift yonlu RS485; cevap beklenirken yazma yok).
  - Istek basina zaman asimi; zaman asiminda hat kisa sure dinlenir (gec cevap).
  - YAZMA DAHIL sure gozetimi: yazma tamamlanma bildirimi WRITE_DONE_TIMEOUT_MS icinde
    gelmezse istek "write_stalled" ile sonuclanir ve on_stall cagrilir (Controller
    baglantiyi dusurur). Gec gelen bildirim (eski nesil/istek) yok sayilir.
  - Kuyruk sinirli (QUEUE_MAX). Dolu ise istek reddedilir (cagiran bildirir).
    Gozetim istegi kuyrugun ONUNE girebilir (front=True); boylece bir gozetim sorgusu
    en fazla hatta o an bulunan TEK istegi bekler (hmi_config.worst_alarm_latency_loaded_s).
  - Hareket komutlarinda KOR OTOMATIK TEKRAR YOK. Zaman asimi = "kabul belirsiz";
    karar cagirana (Controller) aittir.
  - Cevap <-> istek eslesmesi fiil + modul (+ motor) ile yapilir (protokolde islem
    kimligi yok). Eslesmeyen satir "istenmemis" olarak bildirilir.
  - Her yazmaya artan bir yazma sira numarasi (wseq) verilir. Controller bunu, zaman
    asimi/istenmemis cevap sonrasi "hat sirasi belirsiz" donemini ve PING citini
    karsilastirmak icin kullanir (ayrinti: controller._aligned).

Thread modeli: Butun metotlar ana (Qt) thread'de cagrilir. Transport.write_async'in
tamamlanma callback'i baska thread'den gelebilir; Controller onu ana thread'e tasir.
"""
import itertools

import protocol as P
from hmi_config import (QUEUE_MAX, INTER_FRAME_MS, POST_TIMEOUT_GUARD_MS,
                        WRITE_DONE_TIMEOUT_MS)


class Request:
    __slots__ = ("rid", "cmd", "verb", "sid", "motor", "kind", "timeout_ms", "on_done",
                 "state", "t_queued", "t_sent", "t_written", "gen", "tag", "guard_ms",
                 "wseq")

    def __init__(self, cmd, kind, timeout_ms, on_done=None, sid=None, motor=None, tag=None,
                 guard_ms=None):
        self.rid = 0
        self.cmd = cmd
        self.verb = cmd.split(":", 1)[0]
        self.sid = sid
        self.motor = motor
        self.kind = kind            # "poll" | "fence" | "query" | "scan" | "motion" | "raw"
        self.timeout_ms = timeout_ms
        # fn(req, status, reply); status: ok | timeout | write_failed | write_stalled |
        #                                  cancelled | link_lost
        self.on_done = on_done
        self.state = "queued"       # queued | writing | waiting | done
        self.t_queued = None
        self.t_sent = None          # yazmanin BASLADIGI an (tazelik alt siniri)
        self.t_written = None       # yazmanin bittigi an (cevap zaman asimi buradan)
        self.gen = None
        self.tag = tag
        self.guard_ms = guard_ms    # zaman asimi sonrasi hat dinleme (None = varsayilan)
        self.wseq = None            # yazma sira numarasi (yazilmadiysa None)

    def __repr__(self):
        return f"Request({self.cmd!r}, {self.kind}, {self.state})"


class SerialLink:
    def __init__(self, timers, write_async, on_unsolicited, max_queue=QUEUE_MAX,
                 on_stall=None):
        self._ts = timers
        self._write_async = write_async      # fn(text, done_cb(ok))
        self._on_unsolicited = on_unsolicited  # fn(reply_or_None, raw_line)
        self._on_stall = on_stall            # fn(req): yazma tamamlanmadi (hat askida)
        self._max = max_queue
        self._q = []
        self._out = None                      # bekleyen (yazilan) istek
        self._timeout_h = None                # yazma suresi VEYA cevap suresi zamanlayicisi
        self._pump_h = None
        self._quiet_until = 0.0
        self._ids = itertools.count(1)
        self._wseq = 0
        self.gen = 0                          # baglanti nesli (reset ile artar)
        self.enabled = False

    # ---------------- durum ----------------
    @property
    def outstanding(self):
        return self._out

    @property
    def writes(self):
        """Simdiye kadar baslatilan yazma sayisi (= son yazilan istegin wseq'i)."""
        return self._wseq

    def idle(self):
        return self._out is None and not self._q

    def queued(self):
        return list(self._q)

    def has(self, pred):
        if self._out is not None and pred(self._out):
            return True
        return any(pred(r) for r in self._q)

    def count(self, pred):
        n = 1 if (self._out is not None and pred(self._out)) else 0
        return n + sum(1 for r in self._q if pred(r))

    # ---------------- yonetim ----------------
    def start(self):
        self.gen += 1
        self.enabled = True
        self._quiet_until = 0.0

    def reset(self, status="link_lost"):
        """Baglanti degisti: butun istekler sonuclanir (callback'ler eski nesil
        olarak isaretlenir), yeni nesil baslar."""
        self.enabled = False
        self.gen += 1
        pending = ([self._out] if self._out is not None else []) + self._q
        self._out = None
        self._q = []
        self._cancel_timers()
        for r in pending:
            self._finish(r, status, None)

    def submit(self, req, front=False):
        """Istegi kuyruga ekle. False: baglanti yok veya kuyruk dolu.
        front=True: kuyrugun ONUNE (gozetim). Gozetim istegi tek oldugundan kuyruk
        sinirini en fazla 1 asar."""
        if not self.enabled:
            return False
        if len(self._q) >= self._max + (1 if front else 0):
            return False
        req.rid = next(self._ids)
        req.gen = self.gen
        req.t_queued = self._ts.now()
        if front:
            self._q.insert(0, req)
        else:
            self._q.append(req)
        self._schedule_pump(0)
        return True

    def promote(self, pred):
        """Kuyruktaki ilk eslesen istegi one al (yazilmamis olmali). True: bulundu."""
        for i, r in enumerate(self._q):
            if pred(r):
                if i:
                    self._q.insert(0, self._q.pop(i))
                return True
        return False

    def cancel(self, pred, status="cancelled"):
        """Kuyruktaki (henuz yazilmamis) eslesen istekleri iptal et.
        Yazilmis/bekleyen istek iptal EDILMEZ (komut hatta cikmis olabilir)."""
        keep, drop = [], []
        for r in self._q:
            (drop if pred(r) else keep).append(r)
        self._q = keep
        for r in drop:
            self._finish(r, status, None)
        return len(drop)

    # ---------------- gelen satir ----------------
    def on_line(self, line):
        """Ana thread'de cagrilir. Bekleyen istege aitse onu tamamlar."""
        reply = P.parse_reply(line)
        out = self._out
        if out is not None and out.state in ("writing", "waiting") and \
                P.request_matches(out.verb, out.sid, out.motor, reply):
            self._out = None
            self._cancel_timeout()
            self._quiet(INTER_FRAME_MS)
            self._finish(out, "ok", reply)
            self._schedule_pump(INTER_FRAME_MS)
            return
        self._on_unsolicited(reply, line)

    # ---------------- ic ----------------
    def _quiet(self, ms):
        """Hat bu sure boyunca yeni yazma yapmaz (cerceve arasi / gec cevap payi)."""
        self._quiet_until = max(self._quiet_until, self._ts.now() + ms / 1000.0)

    def _finish(self, req, status, reply):
        req.state = "done"
        if req.on_done:
            req.on_done(req, status, reply)

    def _cancel_timeout(self):
        if self._timeout_h is not None:
            self._timeout_h.cancel()
            self._timeout_h = None

    def _cancel_timers(self):
        self._cancel_timeout()
        if self._pump_h is not None:
            self._pump_h.cancel()
            self._pump_h = None

    def _schedule_pump(self, ms):
        if self._pump_h is not None and self._pump_h.active:
            return
        self._pump_h = self._ts.call_later(ms, self._pump)

    def _pump(self):
        self._pump_h = None
        if not self.enabled or self._out is not None or not self._q:
            return
        now = self._ts.now()
        if now < self._quiet_until:
            self._schedule_pump(int((self._quiet_until - now) * 1000) + 1)
            return
        req = self._q.pop(0)
        req.state = "writing"
        req.t_sent = now
        self._wseq += 1
        req.wseq = self._wseq
        self._out = req
        gen, rid = self.gen, req.rid
        # yazma suresi gozetimi: tamamlanma bildirimi gelmezse istek askida kalmaz
        self._timeout_h = self._ts.call_later(WRITE_DONE_TIMEOUT_MS,
                                              lambda: self._write_stalled(gen, rid))
        self._write_async(req.cmd, lambda ok: self._write_done(gen, rid, ok))

    def _write_done(self, gen, rid, ok):
        out = self._out
        if gen != self.gen or out is None or out.rid != rid or out.state != "writing":
            return                      # eski nesil / zaten sonuclanmis / gec bildirim
        self._cancel_timeout()
        if not ok:
            self._out = None
            self._quiet(INTER_FRAME_MS)
            self._finish(out, "write_failed", None)
            self._schedule_pump(INTER_FRAME_MS)
            return
        out.state = "waiting"
        out.t_written = self._ts.now()
        self._timeout_h = self._ts.call_later(out.timeout_ms,
                                              lambda: self._timed_out(gen, rid))

    def _write_stalled(self, gen, rid):
        self._timeout_h = None
        out = self._out
        if gen != self.gen or out is None or out.rid != rid or out.state != "writing":
            return
        # Bayt'larin hatta cikip cikmadigi BILINMEZ. Hat askida: yeni yazma yapilmaz.
        self._out = None
        self._finish(out, "write_stalled", None)
        if self._on_stall is not None and gen == self.gen:
            self._on_stall(out)
        if self.enabled and gen == self.gen:
            self._quiet(POST_TIMEOUT_GUARD_MS)
            self._schedule_pump(POST_TIMEOUT_GUARD_MS)

    def _timed_out(self, gen, rid):
        self._timeout_h = None
        out = self._out
        if gen != self.gen or out is None or out.rid != rid:
            return
        self._out = None
        guard = out.guard_ms if out.guard_ms is not None else POST_TIMEOUT_GUARD_MS
        self._quiet(guard)
        self._finish(out, "timeout", None)
        self._schedule_pump(guard)
