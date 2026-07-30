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

    # ================= STL DOSYA GEZGINI (StlPicker) =================
    @Slot(result="QVariantList")
    def stlSearchDirs(self):
        """Hizli erisim klasorleri: ev, ~/stl, uygulama dizini."""
        home = os.path.expanduser("~")
        app_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        cand = [("Ev", home), ("STL", os.path.join(home, "stl")), ("Uygulama", app_dir)]
        out = []
        for name, p in cand:
            if os.path.isdir(p):
                out.append({"name": name, "path": p})
        return out

    @Slot(str, result=str)
    def parentDir(self, directory):
        return os.path.dirname(directory.rstrip("/")) or "/"

    @Slot(str, result="QVariantList")
    def listStlFiles(self, directory):
        """Verilen klasordeki alt klasorleri + .stl dosyalarini dondur.
        Her oge: {name, path, isDir, size, mtime}. QML tarafinda FolderListModel yok."""
        if not directory or not os.path.isdir(directory):
            directory = os.path.expanduser("~")
        dirs, files = [], []
        try:
            for entry in os.scandir(directory):
                try:
                    if entry.name.startswith("."):
                        continue
                    if entry.is_dir():
                        dirs.append({"name": entry.name, "path": entry.path,
                                     "isDir": True, "size": "", "mtime": ""})
                    elif entry.is_file() and entry.name.lower().endswith(".stl"):
                        st = entry.stat()
                        kb = st.st_size / 1024.0
                        size = f"{kb/1024.0:.1f} MB" if kb >= 1024 else f"{kb:.0f} KB"
                        mtime = time.strftime("%Y-%m-%d %H:%M", time.localtime(st.st_mtime))
                        files.append({"name": entry.name, "path": entry.path,
                                      "isDir": False, "size": size, "mtime": mtime})
                except OSError:
                    continue
        except OSError as e:
            self._log(f"HATA klasor okunamadi: {e}")
        dirs.sort(key=lambda d: d["name"].lower())
        files.sort(key=lambda f: f["name"].lower())
        return dirs + files

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
            self.connectedChanged.emit()
            self._log(f"Baglandi: {device} @ {baud}")
        except Exception as e:
            self._log(f"HATA baglanti: {e}")

    @Slot()
    def disconnectPort(self):
        self._serial.close()
        self._connected = False
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
            self._log(f"DEMO: aktif slave {self._active}")
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

    @Slot(int)
    def sendArrayToSlave(self, slave_id):
        if not self._grid:
            self._log("HATA: once STL hesapla")
            return
        vals = self._slave_values(slave_id)
        cmd = f"ARR:{slave_id:02d}:" + ":".join(str(v) for v in vals)
        self._send(cmd)
        self._demo_targets[slave_id] = vals

    @Slot()
    def sendArrayActive(self):
        if not self._grid:
            self._log("HATA: once STL hesapla")
            return
        if not self._active:
            self._log("HATA: aktif slave yok, once tara")
            return
        for sid in self._active:
            self.sendArrayToSlave(sid)
            time.sleep(0.05)

    @Slot(int)
    def homeSlave(self, slave_id):
        self._send(f"HOME:{slave_id:02d}")
        self._demo_targets[slave_id] = [0] * 9

    @Slot()
    def homeAllActive(self):
        if not self._active:
            self._log("HATA: aktif slave yok")
            return
        for sid in self._active:
            self.homeSlave(sid)
            time.sleep(0.05)

    @Slot(int, int, int)
    def moveMotor(self, slave_id, motor_id, mm):
        self._send(f"MOV:{slave_id:02d}:{motor_id:02d}:{int(mm)}")

    @Slot(int, int)
    def homeMotor(self, slave_id, motor_id):
        self._send(f"HOME:{slave_id:02d}:{motor_id:02d}")

    @Slot(int, int)
    def allToValue(self, slave_id, mm):
        self._send(f"ALL:{slave_id:02d}:{int(mm)}")
        self._demo_targets[slave_id] = [int(mm)] * 9

    @Slot(int)
    def requestStatus(self, slave_id):
        if self._demo:
            vals = self._demo_targets.get(slave_id, [0] * 9)
            toks = [f"S{v}" for v in vals]
            self.statusReceived.emit(slave_id, toks)
            return
        self._send(f"STAT:{slave_id:02d}")

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
            except Exception:
                pass
        elif line.startswith("STAT:"):
            # STAT:01:S120,M300,...  (9 token)
            try:
                parts = line.split(":", 2)
                sid = int(parts[1])
                toks = parts[2].split(",") if len(parts) > 2 else []
                self.statusReceived.emit(sid, toks)
            except Exception:
                pass
