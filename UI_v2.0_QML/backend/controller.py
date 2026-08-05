"""
Controller - QML ile Python arasindaki kopru (tek QObject).
QML 'app' olarak erisir. Seri portu, STL'i ve v5.1 protokolunu yonetir.
"""
import threading
import time
import os

from PySide6.QtCore import QObject, Signal, Slot, Property, QUrl, QTimer

from serial_manager import SerialManager
from stl_interpolator import interpolate_stl_to_grid, grid_stats


def slave_grid_origin(slave_id):
    """slave_id (1..16) -> 12x12 grid icindeki 3x3 baslangic (row, col)."""
    row = (slave_id - 1) // 4
    col = (slave_id - 1) % 4
    return row * 3, col * 3


# Islem (op) sabitleri
OP_POLL_MS      = 1500     # STAT poll araligi
OP_STAGGER_MS   = 80       # modul modul komut gonderim araligi (UI bloklamaz)
OP_TIMEOUT_S    = 300.0    # home ~200s, hareket ~150s -> pay birakildi
DEMO_MOVE_S     = 6.0      # demo modda sahte hareket suresi


class Controller(QObject):
    # ---- sinyaller ----
    logMessage        = Signal(str)
    connectedChanged  = Signal()
    demoModeChanged   = Signal()
    portsChanged      = Signal()
    activeSlavesChanged = Signal()
    gridChanged       = Signal()
    statsChanged      = Signal()
    currentSlaveIdChanged = Signal()
    statusReceived    = Signal(int, "QVariantList")   # slaveId, [ "S120", "M300", ... ]
    stlUrlChanged     = Signal()
    busyChanged       = Signal()
    homedChanged      = Signal()
    opProgress        = Signal(str, int, int)         # faz metni, biten modul, toplam
    operationFinished = Signal(bool, str, str)        # ok, baslik, mesaj
    _lineSig          = Signal(str)                   # serial thread -> main thread kopru

    def __init__(self, demo=False):
        super().__init__()
        self._serial = SerialManager(on_line=lambda l: self._lineSig.emit(l))
        self._lineSig.connect(self._handle_line)      # queued: ana thread'de calisir

        self._connected = False
        self._demo = bool(demo)    # main.py --demo bayragindan gelir (arayuzde dugme yok)
        self._ports = []
        self._active = []
        self._grid = []            # 144 int
        self._stats = {}
        self._current_slave = 1
        self._demo_targets = {}    # demo modu icin son hedefler
        self._stl_url = QUrl()     # yuklu ham STL yolu (3D STL gorunumu icin)

        # --- islem (op) durum makinesi ---
        self._homed = set()        # bu oturumda referans alinmis modul id'leri
        self._op = None            # aktif islem sozlugu (asagida _start_op)
        self._op_timer = QTimer(self)
        self._op_timer.setInterval(OP_POLL_MS)
        self._op_timer.timeout.connect(self._poll_op)
        self._demo_done_at = {}    # demo: sid -> sahte hareketin bitecegi zaman
        self._demo_letter = {}     # demo: sid -> hareket sirasindaki durum harfi

        self.refreshPorts()

    # ================= PROPERTIES =================
    def _get_connected(self): return self._connected
    connected = Property(bool, _get_connected, notify=connectedChanged)

    def _get_demo(self): return self._demo
    demoMode = Property(bool, _get_demo, notify=demoModeChanged)

    def _get_ports(self): return self._ports
    ports = Property("QVariantList", _get_ports, notify=portsChanged)

    def _get_active(self): return self._active
    activeSlaves = Property("QVariantList", _get_active, notify=activeSlavesChanged)

    def _get_grid(self): return self._grid
    gridData = Property("QVariantList", _get_grid, notify=gridChanged)

    def _get_stats(self): return self._stats
    stats = Property("QVariant", _get_stats, notify=statsChanged)

    def _get_current(self): return self._current_slave
    def _set_current(self, v):
        v = int(v)
        if v != self._current_slave:
            self._current_slave = v
            self.currentSlaveIdChanged.emit()
    currentSlaveId = Property(int, _get_current, _set_current, notify=currentSlaveIdChanged)

    def _get_stl_url(self): return self._stl_url
    stlFileUrl = Property(QUrl, _get_stl_url, notify=stlUrlChanged)

    # islem suruyor mu (butonlar kilitlenir; home bitmeden hareket gonderilmez)
    def _get_busy(self): return self._op is not None
    busy = Property(bool, _get_busy, notify=busyChanged)

    # bagli modullerin hepsi bu oturumda referans aldi mi
    def _get_all_homed(self):
        if not self._active:
            return False
        return all(s in self._homed for s in self._active)
    allHomed = Property(bool, _get_all_homed, notify=homedChanged)

    def _reset_homed(self):
        """Baglanti degisince referans bilgisi gecersiz (guc durumu bilinmez)."""
        self._homed = set()
        self.homedChanged.emit()

    # ================= LOG =================
    def _log(self, msg):
        ts = time.strftime("%H:%M:%S")
        self.logMessage.emit(f"[{ts}] {msg}")

    # ================= PORTS =================
    @Slot()
    def refreshPorts(self):
        self._ports = [p["device"] for p in SerialManager.list_ports()]
        self.portsChanged.emit()
        self._log(f"Port tarandi: {len(self._ports)} port")

    # ================= BAGLAN =================
    @Slot(str, int)
    @Slot(str)
    def connectPort(self, device, baud=115200):
        if self._demo:
            self._connected = True
            self.connectedChanged.emit()
            self._log("DEMO MODE: sahte baglanti kuruldu")
            return
        try:
            self._serial.open(device, baud)
            self._connected = True
            self._reset_homed()
            self.connectedChanged.emit()
            self._log(f"Baglandi: {device} @ {baud}")
        except Exception as e:
            self._log(f"HATA baglanti: {e}")

    @Slot()
    def disconnectPort(self):
        self._abort_op("Baglanti kesildi")
        self._serial.close()
        self._connected = False
        self._reset_homed()
        self.connectedChanged.emit()
        self._log("Baglanti kesildi")

    @Slot(bool)
    def setDemoMode(self, on):
        self._demo = bool(on)
        self._connected = False
        self.connectedChanged.emit()
        self.demoModeChanged.emit()
        self._log(f"Mod: {'DEMO' if on else 'Gercek donanim'}")

    # ================= TARAMA (PING) =================
    @Slot()
    def scan(self):
        self._active = []
        self.activeSlavesChanged.emit()
        self._log("Ag taramasi basladi...")
        if self._demo:
            for s in range(1, 5):
                self._active.append(s)
            self.activeSlavesChanged.emit()
            self.homedChanged.emit()   # allHomed aktif listeye bagli
            self._log(f"DEMO: {len(self._active)} modul bulundu {self._active}")
            return

        def worker():
            for sid in range(1, 17):
                self._serial.write_line(f"PING:{sid:02d}")
                self._lineSig.emit(f"@TX PING:{sid:02d}")
                time.sleep(0.15)
        threading.Thread(target=worker, daemon=True).start()

    # ================= STL =================
    @Slot(QUrl)
    @Slot(str)
    def loadStl(self, file):
        path = file.toLocalFile() if isinstance(file, QUrl) else str(file)
        if not path or not os.path.exists(path):
            self._log(f"HATA: STL bulunamadi: {path}")
            return
        self._log(f"STL interpolasyonu: {os.path.basename(path)}")
        # ham STL yolunu 3D STL gorunumu icin ac (RuntimeLoader source)
        self._stl_url = QUrl.fromLocalFile(path)
        self.stlUrlChanged.emit()

        def worker():
            try:
                grid, _mesh = interpolate_stl_to_grid(path)
                self._grid = [int(v) for v in grid.tolist()]
                self._stats = grid_stats(self._grid)
                # ana thread'e bildir
                self._lineSig.emit("@GRID_READY")
            except Exception as e:
                self._lineSig.emit(f"@ERR STL: {e}")
        threading.Thread(target=worker, daemon=True).start()

    # ================= HAREKET (v5.1) =================
    def _slave_values(self, slave_id):
        """slave'in 9 motor hedefini (1..9) grid'den cikar."""
        sr, sc = slave_grid_origin(slave_id)
        vals = []
        for i in range(3):
            for j in range(3):
                idx = (sr + i) * 12 + (sc + j)
                vals.append(int(self._grid[idx]) if idx < len(self._grid) else 0)
        return vals

    # ---------- ISLEM (OP) DURUM MAKINESI ----------
    # Faz zinciri: ["home", "send"] gibi. Her faz: komutlar kademeli gonderilir,
    # sonra STAT poll ile tum modullerin tokenlari "S" olana kadar beklenir.
    # Protokol v5.1 aynen: ARR/HOME ack doner, bitis STAT poll ile anlasilir.

    @Slot()
    def startProduction(self):
        """Ana uretim akisi: ONCE HOME (tum bagli moduller) SONRA sekli uygula."""
        if not self._guard_op():
            return
        if not self._grid:
            self.operationFinished.emit(False, "Model yok", "Once bir STL yukleyip hesaplayin.")
            return
        self._start_op(["home", "send"], list(self._active))

    @Slot()
    def startHomeAll(self):
        """Sadece referans alma (tum bagli moduller)."""
        if not self._guard_op():
            return
        self._start_op(["home"], list(self._active))

    @Slot(int)
    def startSendSelected(self, slave_id):
        """Test: yalnizca secili module sekli uygula (home zorunlu degil)."""
        if not self._guard_op():
            return
        if not self._grid:
            self.operationFinished.emit(False, "Model yok", "Once bir STL yukleyip hesaplayin.")
            return
        self._start_op(["send"], [int(slave_id)])

    def _guard_op(self):
        """Islem baslatilabilir mi (bagli, bos, modul var)."""
        if self._op:
            self.operationFinished.emit(False, "Islem suruyor",
                                        "Mevcut hareket bitmeden yeni islem baslatilamaz.")
            return False
        if not self._connected:
            self.operationFinished.emit(False, "Baglanti yok", "Once cihaza baglanin.")
            return False
        if not self._active:
            self.operationFinished.emit(False, "Modul yok",
                                        "Bagli modul bulunamadi. Once 'Modulleri Bul'.")
            return False
        return True

    def _start_op(self, seq, targets):
        self._op = {
            "seq": list(seq), "phase": None, "targets": list(targets),
            "pending": set(), "t0": 0.0, "step": 0, "nsteps": len(seq), "label": "",
        }
        self._next_phase()

    def _next_phase(self):
        op = self._op
        if not op:
            return
        if not op["seq"]:
            self._finish_op(True, "Tamamlandi", "Tum moduller hedef pozisyonda.")
            return
        phase = op["seq"].pop(0)
        op["phase"] = phase
        op["step"] += 1
        op["label"] = "Referans aliniyor" if phase == "home" else "Sekil uygulaniyor"
        op["pending"] = set(op["targets"])
        op["t0"] = time.time()
        self._log(f"{op['label']} ({op['step']}/{op['nsteps']}) - {len(op['targets'])} modul")
        self.busyChanged.emit()
        self._emit_progress()
        self._dispatch(phase, list(op["targets"]), 0)
        self._op_timer.start()

    def _dispatch(self, phase, targets, i):
        """Komutlari modul modul kademeli gonder (QTimer - UI bloklamaz)."""
        if not self._op or i >= len(targets):
            return
        sid = targets[i]
        if phase == "home":
            self._send(f"HOME:{sid:02d}")
            self._demo_begin(sid, "H", [0] * 9)
        else:
            vals = self._slave_values(sid)
            self._send(f"ARR:{sid:02d}:" + ":".join(str(v) for v in vals))
            self._demo_begin(sid, "M", vals)
        QTimer.singleShot(OP_STAGGER_MS, lambda: self._dispatch(phase, targets, i + 1))

    def _poll_op(self):
        op = self._op
        if not op:
            self._op_timer.stop()
            return
        if time.time() - op["t0"] > OP_TIMEOUT_S:
            kalan = ", ".join(str(s) for s in sorted(op["pending"]))
            self._finish_op(False, "Zaman asimi",
                            f"{op['label']} tamamlanmadi. Yanit vermeyen modul: {kalan}")
            return
        for sid in sorted(op["pending"]):
            self.requestStatus(sid)

    def _op_status(self, sid, toks):
        """STAT cevabini aktif isleme isle (tum tokenlar S -> o modul bitti)."""
        op = self._op
        if not op or sid not in op["pending"]:
            return
        letters = [t[:1] for t in toks if t]
        if len(letters) < 9:
            return
        if "F" in letters:
            motor = letters.index("F") + 1
            self._finish_op(False, "ARIZA",
                            f"Modul {sid} / Motor {motor} ariza verdi. "
                            "Encoder kablosunu ve mekanigi kontrol edin.")
            return
        if all(l == "S" for l in letters):
            op["pending"].discard(sid)
            self._emit_progress()
            if not op["pending"]:
                self._op_timer.stop()
                if op["phase"] == "home":
                    self._homed.update(op["targets"])
                    self.homedChanged.emit()
                self._next_phase()

    def _emit_progress(self):
        op = self._op
        if not op:
            return
        total = len(op["targets"])
        done = total - len(op["pending"])
        self.opProgress.emit(f"{op['label']} ({op['step']}/{op['nsteps']})", done, total)

    def _finish_op(self, ok, title, msg):
        self._op_timer.stop()
        self._op = None
        self.busyChanged.emit()
        self._log(("TAMAM: " if ok else "HATA: ") + title + " - " + msg)
        self.operationFinished.emit(bool(ok), title, msg)

    def _abort_op(self, reason):
        """Islem varsa sessizce bitir (baglanti kesilmesi vb.).
        NOT: protokol v5.1'de STOP/ABORT komutu YOK - motorlar fiziksel olarak durmaz."""
        if self._op:
            self._finish_op(False, "Islem kesildi", reason)

    # ---------- tekil komutlar (Tester / test butonlari - gate yok) ----------
    @Slot(int)
    def sendArrayToSlave(self, slave_id):
        if not self._grid:
            self._log("HATA: once STL hesapla")
            return
        vals = self._slave_values(slave_id)
        cmd = f"ARR:{slave_id:02d}:" + ":".join(str(v) for v in vals)
        self._send(cmd)
        self._demo_begin(slave_id, "M", vals)

    @Slot(int)
    def homeSlave(self, slave_id):
        self._send(f"HOME:{slave_id:02d}")
        self._demo_begin(slave_id, "H", [0] * 9)

    @Slot(int, int, int)
    def moveMotor(self, slave_id, motor_id, mm):
        self._send(f"MOV:{slave_id:02d}:{motor_id:02d}:{int(mm)}")

    @Slot(int, int)
    def homeMotor(self, slave_id, motor_id):
        self._send(f"HOME:{slave_id:02d}:{motor_id:02d}")

    @Slot(int, int)
    def allToValue(self, slave_id, mm):
        self._send(f"ALL:{slave_id:02d}:{int(mm)}")
        self._demo_begin(slave_id, "M", [int(mm)] * 9)

    # ---------- demo simulasyonu (donanimsiz gelistirme) ----------
    def _demo_begin(self, slave_id, letter, vals):
        """Demo modda sahte hareket baslat: DEMO_MOVE_S boyunca M/H, sonra S."""
        self._demo_targets[slave_id] = vals
        if not self._demo:
            return
        self._demo_letter[slave_id] = letter
        self._demo_done_at[slave_id] = time.time() + DEMO_MOVE_S

    @Slot(int)
    def requestStatus(self, slave_id):
        if self._demo:
            vals = self._demo_targets.get(slave_id, [0] * 9)
            if time.time() < self._demo_done_at.get(slave_id, 0.0):
                letter = self._demo_letter.get(slave_id, "M")
            else:
                letter = "S"
            self._on_status(slave_id, [f"{letter}{v}" for v in vals])
            return
        self._send(f"STAT:{slave_id:02d}")

    def _on_status(self, slave_id, toks):
        """Tek giris noktasi: STAT hem UI'ya hem aktif isleme dagitilir."""
        self.statusReceived.emit(slave_id, toks)
        self._op_status(slave_id, toks)

    @Slot(str)
    def sendRaw(self, text):
        text = text.strip()
        if text:
            self._send(text)

    def _send(self, cmd):
        self._log(f"→ {cmd}")
        if self._demo:
            return
        if not self._serial.is_open:
            self._log("HATA: baglanti yok")
            return
        self._serial.write_line(cmd)

    # ================= GELEN SATIR ISLEME =================
    @Slot(str)
    def _handle_line(self, line):
        # ic mesajlar
        if line == "@GRID_READY":
            self.gridChanged.emit()
            self.statsChanged.emit()
            s = self._stats
            self._log(f"Grid hazir: {s.get('min')}-{s.get('max')}mm (ort {s.get('mean')})")
            return
        if line.startswith("@ERR"):
            self._log(line[1:])
            return
        if line.startswith("@TX "):
            self._log("→ " + line[4:])
            return

        # gercek donanim cevabi
        self._log("← " + line)

        if line.startswith("PONG"):
            try:
                sid = int(line.split(":")[1])
                if sid not in self._active:
                    self._active.append(sid)
                    self._active.sort()
                    self.activeSlavesChanged.emit()
                    self.homedChanged.emit()   # allHomed aktif listeye bagli
            except Exception:
                pass
        elif line.startswith("STAT:"):
            # STAT:01:S120,M300,...  (9 token)
            try:
                parts = line.split(":", 2)
                sid = int(parts[1])
                toks = parts[2].split(",") if len(parts) > 2 else []
                self._on_status(sid, toks)
            except Exception:
                pass
