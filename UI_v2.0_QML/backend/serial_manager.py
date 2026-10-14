"""
Seri port yoneticisi - pyserial + arka plan OKUYUCU ve YAZICI thread'leri.

v2.2 degisiklikleri:
  - Yazma UI thread'inde DEGIL: write_async() kuyruga koyar, yazici thread yazar,
    sonucu callback ile bildirir. write_timeout tanimli (takilan port UI'yi dondurmaz).
  - Okuma tamponu sinirli (LineAssembler): satir sonu gelmeyen akis SERIAL_MAX_LINE'i
    asinca atilir ve bildirilir.
  - Okuyucu istisnalari YUTULMAZ: port hatasi (USB cikti vb.) on_disconnect ile bildirilir.
  - serial_for_url: gercek cihaz yolu veya pyserial URL'si (testte "loop://").

Cerceveden bagimsiz; callback'ler okuyucu/yazici thread'inden cagrilir.
"""
import queue
import threading
import time

import serial
import serial.tools.list_ports

from hmi_config import SERIAL_WRITE_TIMEOUT_S, SERIAL_MAX_LINE


class LineAssembler:
    """Gelen bayt akisini satirlara boler; tamponu sinirlar."""

    def __init__(self, max_len=SERIAL_MAX_LINE):
        self.max_len = int(max_len)
        self._buf = ""
        self.dropped = 0           # sinir asimi nedeniyle atilan karakter

    def pending_len(self):
        return len(self._buf)

    def feed(self, text):
        out = []
        self._buf += text
        while "\n" in self._buf:
            line, self._buf = self._buf.split("\n", 1)
            line = line.strip().replace("\r", "")
            if line:
                if len(line) > self.max_len:
                    self.dropped += len(line)
                    continue
                out.append(line)
        if len(self._buf) > self.max_len:
            self.dropped += len(self._buf)
            self._buf = ""
        return out


class _Session:
    """Bir acik baglanti. Thread'ler kendi oturumlarina bakar: kapatilip hemen
    yeniden acilan port eski thread'in hatasiyla yeni baglantiyi dusurmez."""

    def __init__(self, ser):
        self.ser = ser
        self.running = True
        self.wq = queue.Queue()
        self.lock = threading.Lock()


class SerialManager:
    def __init__(self, on_line=None, on_disconnect=None, on_noise=None):
        self._s = None
        self.on_line = on_line            # callback(str)        - okuyucu thread'i
        self.on_disconnect = on_disconnect  # callback(str)      - okuyucu thread'i
        self.on_noise = on_noise          # callback(str)        - okuyucu thread'i

    # ---- port listesi ----
    @staticmethod
    def list_ports():
        out = []
        for p in serial.tools.list_ports.comports():
            out.append({"device": p.device, "description": p.description or ""})
        return out

    @property
    def is_open(self):
        s = self._s
        return s is not None and s.running and s.ser.is_open

    # ---- baglan / kes ----
    def open(self, port, baud=115200):
        self.close()
        ser = serial.serial_for_url(port, baudrate=baud, timeout=0.1,
                                    write_timeout=SERIAL_WRITE_TIMEOUT_S)
        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        except Exception:
            pass
        s = _Session(ser)
        self._s = s
        threading.Thread(target=self._read_loop, args=(s,), daemon=True).start()
        threading.Thread(target=self._write_loop, args=(s,), daemon=True).start()
        return True

    def close(self):
        """UI thread'ini bekletmez: thread'lere dur sinyali verilir, port ayri
        thread'de kapatilir (yazici write_timeout kadar mesgul olabilir)."""
        s, self._s = self._s, None
        if s is None:
            return
        s.running = False
        s.wq.put(None)

        def _close():
            with s.lock:
                try:
                    s.ser.close()
                except Exception:
                    pass
        threading.Thread(target=_close, daemon=True).start()

    # ---- gonder ----
    def write_async(self, text, done_cb):
        """Komutu yazici thread'ine ver. done_cb(ok) yazici thread'inden cagrilir."""
        s = self._s
        if s is None or not s.running:
            done_cb(False)
            return
        if not text.endswith("\n"):
            text = text + "\r\n"
        s.wq.put((text, done_cb))

    def _write_loop(self, s):
        while True:
            item = s.wq.get()
            if item is None:
                return
            text, cb = item
            ok = False
            if s.running:
                data = text.encode("utf-8")
                with s.lock:
                    try:
                        n = s.ser.write(data)
                        s.ser.flush()
                        ok = (n == len(data))
                    except Exception:
                        ok = False
            try:
                cb(ok)
            except Exception:
                pass

    # ---- arka plan okuma ----
    def _read_loop(self, s):
        asm = LineAssembler()
        while s.running:
            try:
                n = s.ser.in_waiting
                if n:
                    raw = s.ser.read(n).decode("utf-8", errors="replace")
                    before = asm.dropped
                    for line in asm.feed(raw):
                        if s.running and self.on_line:
                            self.on_line(line)
                    if asm.dropped != before and self.on_noise:
                        self.on_noise("Satır sonu gelmeyen veri atıldı "
                                      f"({asm.dropped - before} karakter)")
                else:
                    time.sleep(0.005)
            except Exception as e:
                # kopma (USB cikti, port kapandi): YUTULMAZ, bildirilir
                if s.running:
                    s.running = False
                    if self.on_disconnect:
                        self.on_disconnect(str(e) or e.__class__.__name__)
                return
