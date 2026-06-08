"""
Seri port yoneticisi - pyserial + arka plan okuma thread'i.
Cerceveden bagimsiz; satir geldikce verilen callback'i cagirir.
"""
import threading
import time

import serial
import serial.tools.list_ports


class SerialManager:
    def __init__(self, on_line=None):
        self._ser = None
        self._lock = threading.Lock()
        self._reader = None
        self._running = False
        self.on_line = on_line          # callback(str)

    # ---- port listesi ----
    @staticmethod
    def list_ports():
        out = []
        for p in serial.tools.list_ports.comports():
            out.append({"device": p.device, "description": p.description or ""})
        return out

    @property
    def is_open(self):
        return self._ser is not None and self._ser.is_open

    # ---- baglan / kes ----
    def open(self, port, baud=115200):
        self.close()
        self._ser = serial.Serial(port, baud, timeout=0.1)
        try:
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
        except Exception:
            pass
        self._running = True
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        return True

    def close(self):
        self._running = False
        if self._reader and self._reader.is_alive():
            self._reader.join(timeout=0.5)
        self._reader = None
        with self._lock:
            if self._ser:
                try:
                    self._ser.close()
                except Exception:
                    pass
            self._ser = None

    # ---- gonder ----
    def write_line(self, text):
        """Komutu \r\n ile gonder."""
        if not self.is_open:
            return False
        if not text.endswith("\n"):
            text = text + "\r\n"
        with self._lock:
            try:
                self._ser.write(text.encode("utf-8"))
                self._ser.flush()
                return True
            except Exception:
                return False

    # ---- arka plan okuma ----
    def _read_loop(self):
        buf = ""
        while self._running:
            try:
                if not self.is_open:
                    time.sleep(0.2)
                    continue
                n = self._ser.in_waiting
                if n:
                    raw = self._ser.read(n).decode("utf-8", errors="ignore")
                    buf += raw
                    if "\n" in buf:
                        parts = buf.split("\n")
                        buf = parts[-1]
                        for line in parts[:-1]:
                            line = line.strip().replace("\r", "")
                            if line and self.on_line:
                                self.on_line(line)
                else:
                    time.sleep(0.005)
            except Exception:
                buf = ""
                time.sleep(0.1)
