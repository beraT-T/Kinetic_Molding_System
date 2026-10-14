"""
Enjekte edilebilir saat / zamanlayici.

Gercek uygulama QtTimerService kullanir (QTimer.singleShot + iptal bayragi).
Testler FakeTimerService ile zamani ELLE ilerletir: callback sirasi ve
zaman asimlari deterministik olur (donanimsiz, beklemesiz test).

Arayuz (ikisi de ayni):
    now()                  monoton saniye (sure / yas hesabi)
    wall()                 duvar saati saniye (ekranda saat gostermek icin)
    call_later(ms, fn)     -> handle; handle.cancel() ile iptal
"""
import heapq
import itertools
import time

from PySide6.QtCore import QTimer


class TimerHandle:
    """Zamanlanmis callback tutamaci. cancel() sonrasi callback calismaz."""
    __slots__ = ("done",)

    def __init__(self):
        self.done = False

    def cancel(self):
        self.done = True

    @property
    def active(self):
        return not self.done


class QtTimerService:
    """Gercek zamanlayici: Qt olay dongusunde (ana thread) calisir."""

    def now(self):
        return time.monotonic()

    def wall(self):
        return time.time()

    def call_later(self, ms, fn):
        h = TimerHandle()

        def fire():
            if h.done:
                return
            h.done = True
            fn()

        QTimer.singleShot(max(0, int(ms)), fire)
        return h


class FakeTimerService:
    """Test zamanlayicisi: advance(ms) ile zaman ilerler, vadesi gelen
    callback'ler zaman sirasina gore (esitlikte ekleme sirasina gore) calisir."""

    def __init__(self, start=1000.0, wall0=1_760_000_000.0):
        self._t = float(start)
        self._wall0 = float(wall0) - float(start)
        self._q = []
        self._seq = itertools.count()

    def now(self):
        return self._t

    def wall(self):
        return self._wall0 + self._t

    def call_later(self, ms, fn):
        h = TimerHandle()
        due = self._t + max(0.0, float(ms)) / 1000.0
        heapq.heappush(self._q, (due, next(self._seq), h, fn))
        return h

    def pending(self):
        return sum(1 for (_d, _s, h, _f) in self._q if not h.done)

    def advance(self, ms):
        """Zamani ms kadar ilerlet; arada vadesi gelen her callback'i calistir."""
        end = self._t + float(ms) / 1000.0
        while self._q and self._q[0][0] <= end:
            due, _s, h, fn = heapq.heappop(self._q)
            if due > self._t:
                self._t = due
            if h.done:
                continue
            h.done = True
            fn()
        self._t = end

    def advance_s(self, s):
        self.advance(float(s) * 1000.0)
