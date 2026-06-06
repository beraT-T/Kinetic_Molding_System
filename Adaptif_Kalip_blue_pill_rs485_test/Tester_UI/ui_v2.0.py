"""
Kinetik Yüzey Kontrol Paneli — v2.0
=====================================
Değişiklikler (v1.0 → v2.0):
  [1] SETID formatı güncellendi: SETID:{currentID}:{newID}
      → 16 slave aynı hatta olduğunda çakışma olmaz
  [2] GETID formatı güncellendi: GETID:{id} (adresli)
      → Sadece hedef slave cevap verir: IDVAL:{id}
  [3] Ağ taraması düzeltildi:
      - Önceki: 4.5 sn/slave × 16 = 72 sn → şimdi 150ms/slave × 16 = 2.4 sn
      - PING:{id} → PONG:{id} handshake (adresli, çakışmasız)
  [4] Yeni response handler'lar: IDVAL, ALLOK, HOMEOK
  [5] Inspector sekmesi: SETID için dialog eklendi
  [6] Tüm komutlar protokol formatına uygun (id zero-padded)
"""

import customtkinter as ctk
import serial
import serial.tools.list_ports
import threading
import time

ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("blue")

# ── Protokol Sabitleri ──────────────────────────────────────
SCAN_DELAY_PER_SLAVE = 0.15   # 150 ms — PONG için yeterli
SLAVE_COUNT          = 16
MOTOR_COUNT          = 9      # Her slave'de 9 motor (U2:5 + U3:4)

class KineticApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Kinetik Yüzey Kontrol Paneli v2.0")
        self.geometry("1150x720")

        self.ser             = None
        self.is_connected    = False
        self.selected_slave  = 1
        self.slider_timers   = {}

        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        self.create_sidebar()

        self.tabview = ctk.CTkTabview(self, width=850)
        self.tabview.grid(row=0, column=1, padx=20, pady=20, sticky="nsew")

        self.tab_dashboard = self.tabview.add("Genel Bakış (16 Slave)")
        self.tab_inspector = self.tabview.add("Modül Test (9 Motor)")
        self.tab_terminal  = self.tabview.add("Terminal / Log")

        self.create_dashboard_tab()
        self.create_inspector_tab()
        self.create_terminal_tab()

        # Arka plan okuma thread'i
        self._rx_thread = threading.Thread(target=self._read_serial_loop, daemon=True)
        self._rx_thread.start()

    # ── SIDEBAR ─────────────────────────────────────────────
    def create_sidebar(self):
        f = ctk.CTkFrame(self, width=210, corner_radius=0)
        f.grid(row=0, column=0, sticky="nsew")
        f.grid_rowconfigure(6, weight=1)

        ctk.CTkLabel(f, text="BAĞLANTI", font=ctk.CTkFont(size=18, weight="bold")).grid(
            row=0, column=0, padx=20, pady=(20, 8))

        ports = [p.device for p in serial.tools.list_ports.comports()] or ["Port Yok"]
        self.opt_port = ctk.CTkOptionMenu(f, values=ports)
        self.opt_port.grid(row=1, column=0, padx=20, pady=6)

        self.opt_baud = ctk.CTkOptionMenu(f, values=["9600", "115200", "250000"])
        self.opt_baud.set("115200")
        self.opt_baud.grid(row=2, column=0, padx=20, pady=6)

        self.btn_connect = ctk.CTkButton(f, text="BAĞLAN", command=self._toggle_connection,
                                         fg_color="green")
        self.btn_connect.grid(row=3, column=0, padx=20, pady=8)

        ctk.CTkLabel(f, text="AĞ İŞLEMLERİ", font=ctk.CTkFont(size=14, weight="bold")).grid(
            row=4, column=0, padx=20, pady=(16, 4))

        self.btn_scan = ctk.CTkButton(f, text="AĞI TARA (PING)", command=self._scan_network,
                                      state="disabled")
        self.btn_scan.grid(row=5, column=0, padx=20, pady=6)

        # ID Atama butonu — SETID:{currentID}:{newID}
        self.btn_setid = ctk.CTkButton(f, text="ID ATA (SETID)", command=self._open_setid_dialog,
                                       state="disabled", fg_color="#555500")
        self.btn_setid.grid(row=6, column=0, padx=20, pady=6)

    # ── DASHBOARD — 4×4 Slave Grid ──────────────────────────
    def create_dashboard_tab(self):
        self.slave_buttons = {}
        for i in range(4):
            self.tab_dashboard.grid_columnconfigure(i, weight=1)
            self.tab_dashboard.grid_rowconfigure(i, weight=1)

        sid = 1
        for row in range(4):
            for col in range(4):
                btn = ctk.CTkButton(
                    self.tab_dashboard,
                    text=f"SLAVE {sid:02d}\n(Offline)",
                    font=ctk.CTkFont(size=15, weight="bold"),
                    fg_color="#333333", height=110,
                    command=lambda s=sid: self._select_slave(s)
                )
                btn.grid(row=row, column=col, padx=8, pady=8, sticky="nsew")
                self.slave_buttons[sid] = btn
                sid += 1

    # ── INSPECTOR — 3×3 Motor Grid ──────────────────────────
    def create_inspector_tab(self):
        self.lbl_module = ctk.CTkLabel(self.tab_inspector,
                                       text="Seçili Slave: ID 01",
                                       font=ctk.CTkFont(size=22))
        self.lbl_module.pack(pady=8)

        motor_frame = ctk.CTkFrame(self.tab_inspector)
        motor_frame.pack(pady=6, padx=10, fill="both", expand=True)

        self.motor_sliders = []
        self.motor_labels  = []

        for row in range(3):
            for col in range(3):
                mid = row * 3 + col + 1
                f = ctk.CTkFrame(motor_frame)
                f.grid(row=row, column=col, padx=4, pady=4, sticky="nsew")
                motor_frame.grid_columnconfigure(col, weight=1)
                motor_frame.grid_rowconfigure(row, weight=1)

                ctk.CTkLabel(f, text=f"Motor {mid}", font=ctk.CTkFont(weight="bold")).pack(pady=4)

                lbl_val = ctk.CTkLabel(f, text="0 mm", font=ctk.CTkFont(size=12))
                lbl_val.pack()
                self.motor_labels.append(lbl_val)

                slider = ctk.CTkSlider(f, from_=0, to=600, orientation="vertical", height=130,
                                       command=lambda v, m=mid: self._on_slider(m, v))
                slider.set(0)
                slider.pack(pady=4)
                self.motor_sliders.append(slider)

                ctk.CTkButton(f, text="HOME", width=80, height=28, fg_color="orange",
                              command=lambda m=mid: self._send_home(m)).pack(pady=4)

        ctrl = ctk.CTkFrame(self.tab_inspector)
        ctrl.pack(fill="x", pady=8)
        ctk.CTkButton(ctrl, text="Tümünü HOME (0mm)",
                      command=self._all_home).pack(side="left", padx=10, pady=8, expand=True)
        ctk.CTkButton(ctrl, text="Test (300mm)",
                      command=self._all_test).pack(side="left", padx=10, pady=8, expand=True)

    # ── TERMINAL ─────────────────────────────────────────────
    def create_terminal_tab(self):
        self.txt_log = ctk.CTkTextbox(self.tab_terminal, font=("Consolas", 12))
        self.txt_log.pack(fill="both", expand=True, padx=10, pady=(10, 4))

        bot = ctk.CTkFrame(self.tab_terminal)
        bot.pack(fill="x", padx=10, pady=4)

        self.entry_cmd = ctk.CTkEntry(bot,
            placeholder_text="Örn: PING:01  |  GETID:01  |  SETID:01:03  |  MOV:01:06:150",
            font=("Consolas", 12))
        self.entry_cmd.pack(side="left", fill="x", expand=True, padx=(0, 4))
        self.entry_cmd.bind("<Return>", lambda e: self._send_terminal_cmd())

        ctk.CTkButton(bot, text="Gönder", command=self._send_terminal_cmd, width=90).pack(side="left", padx=(0, 4))
        ctk.CTkButton(bot, text="Temizle",
                      command=lambda: self.txt_log.delete("0.0", "end"), width=90).pack(side="left")

    # ── BAĞLANTI ─────────────────────────────────────────────
    def _toggle_connection(self):
        if not self.is_connected:
            try:
                port = self.opt_port.get()
                baud = int(self.opt_baud.get())
                self.ser = serial.Serial(port, baud, timeout=0.1)
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                self.is_connected = True
                self.btn_connect.configure(text="BAĞLANTIYI KES", fg_color="red")
                self.btn_scan.configure(state="normal")
                self.btn_setid.configure(state="normal")
                self._log(f"✓ Bağlandı: {port} @ {baud}")
            except Exception as e:
                self._log(f"✗ Hata: {e}")
        else:
            if self.ser:
                self.ser.close()
            self.is_connected = False
            self.btn_connect.configure(text="BAĞLAN", fg_color="green")
            self.btn_scan.configure(state="disabled")
            self.btn_setid.configure(state="disabled")
            self._log("Bağlantı kesildi.")

    # ── VERİ GÖNDER ──────────────────────────────────────────
    def _send(self, data: str, log_it: bool = True):
        """USB üzerinden veri gönder — \n otomatik eklenir."""
        if not self.is_connected or not self.ser:
            return
        try:
            msg = data.strip() + "\n"
            self.ser.write(msg.encode("utf-8"))
            if log_it:
                self.after(0, self._log, f"→ {data.strip()}")
        except Exception as e:
            self.after(0, self._log, f"✗ Gönderim Hatası: {e}")

    def _send_terminal_cmd(self):
        if not self.is_connected:
            self._log("Hata: Önce bağlantı kurun!")
            return
        cmd = self.entry_cmd.get().strip()
        if cmd:
            self._send(cmd)
            self.entry_cmd.delete(0, "end")

    # ── ARKA PLAN OKUMA ──────────────────────────────────────
    def _read_serial_loop(self):
        buf = ""
        while True:
            if self.is_connected and self.ser:
                try:
                    if self.ser.in_waiting > 0:
                        chunk = self.ser.read(self.ser.in_waiting).decode("utf-8", errors="ignore")
                        buf += chunk
                        while "\n" in buf:
                            line, buf = buf.split("\n", 1)
                            line = line.strip()
                            if line:
                                self.after(0, self._log, f"← {line}")
                                self.after(0, self._process_response, line)
                except Exception as e:
                    print(f"[RX Hata] {e}")
                    buf = ""
            else:
                buf = ""
            time.sleep(0.03)

    # ── GELEN VERİ İŞLEME ───────────────────────────────────
    def _process_response(self, line: str):
        parts = line.split(":")

        # PONG:{id} — PING cevabı, slave online
        if line.startswith("PONG") and len(parts) >= 2:
            try:
                sid = int(parts[1])
                if sid in self.slave_buttons:
                    self.slave_buttons[sid].configure(
                        fg_color="green",
                        text=f"SLAVE {sid:02d}\n(Online)"
                    )
            except ValueError:
                pass

        # IDVAL:{id} — GETID cevabı
        elif line.startswith("IDVAL") and len(parts) >= 2:
            try:
                sid = int(parts[1])
                self._log(f"  → Slave ID doğrulandı: {sid:02d}")
            except ValueError:
                pass

        # IDSET:{newID} — SETID başarılı
        elif line.startswith("IDSET") and len(parts) >= 2:
            try:
                sid = int(parts[1])
                self._log(f"  → ID başarıyla atandı: {sid:02d}")
                # Yeni ID'yi online göster
                if sid in self.slave_buttons:
                    self.slave_buttons[sid].configure(
                        fg_color="#006600",
                        text=f"SLAVE {sid:02d}\n(ID Atandı)"
                    )
            except ValueError:
                pass

        # IDERR:{reason} — SETID hatası
        elif line.startswith("IDERR"):
            reason = parts[1] if len(parts) >= 2 else "?"
            self._log(f"  ✗ ID atama hatası: {reason}")

        # MOVOK:{id}:{mid}:{val}
        elif line.startswith("MOVOK") and len(parts) >= 4:
            try:
                sid = int(parts[1]); mid = int(parts[2]); val = int(parts[3])
                # Seçili slave ise slider etiketini güncelle
                if sid == self.selected_slave and 1 <= mid <= 9:
                    self.motor_labels[mid - 1].configure(text=f"{val} mm")
            except (ValueError, IndexError):
                pass

        # ALLOK:{id}:{val}
        elif line.startswith("ALLOK") and len(parts) >= 3:
            try:
                sid = int(parts[1]); val = int(parts[2])
                if sid == self.selected_slave:
                    for lbl in self.motor_labels:
                        lbl.configure(text=f"{val} mm")
            except (ValueError, IndexError):
                pass

        # HOMEOK:{id}:{mid}
        elif line.startswith("HOMEOK") and len(parts) >= 3:
            try:
                sid = int(parts[1]); mid = int(parts[2])
                if sid == self.selected_slave and 1 <= mid <= 9:
                    self.motor_labels[mid - 1].configure(text="0 mm")
                    self.motor_sliders[mid - 1].set(0)
            except (ValueError, IndexError):
                pass

    # ── AĞ TARAMASI ──────────────────────────────────────────
    def _scan_network(self):
        """
        Tüm 16 slave'e PING:{id} gönder, cevap bekle.
        Toplam süre: 16 × 150ms = 2.4 sn
        PONG:{id} gelince ilgili slave kutusu yeşil olur.
        """
        self.btn_scan.configure(state="disabled")
        threading.Thread(target=self._scan_thread, daemon=True).start()

    def _scan_thread(self):
        self.after(0, self._log, "─── Ağ Taraması Başladı ───")

        # Önce hepsini offline yap
        for sid in range(1, SLAVE_COUNT + 1):
            self.after(0, lambda s=sid: self.slave_buttons[s].configure(
                fg_color="#333333",
                text=f"SLAVE {s:02d}\n(Offline)"
            ))

        # Her slave'e adresli PING at
        for sid in range(1, SLAVE_COUNT + 1):
            if self.is_connected and self.ser:
                try:
                    msg = f"PING:{sid:02d}\n"
                    self.ser.write(msg.encode("utf-8"))
                    self.after(0, self._log, f"→ PING:{sid:02d}")
                except Exception as e:
                    self.after(0, self._log, f"✗ {e}")
            # PONG için 150ms bekle (_process_response asenkron olarak işler)
            time.sleep(SCAN_DELAY_PER_SLAVE)

        self.after(0, self._log, "─── Ağ Taraması Tamamlandı ───")
        self.after(0, lambda: self.btn_scan.configure(state="normal"))

    # ── ID ATAMA DİALOG ──────────────────────────────────────
    def _open_setid_dialog(self):
        """
        SETID:{currentID}:{newID}
        Mevcut ID'yi ve yeni ID'yi kullanıcıdan al.
        """
        dlg = ctk.CTkToplevel(self)
        dlg.title("ID Ata (SETID)")
        dlg.geometry("320x200")
        dlg.grab_set()

        ctk.CTkLabel(dlg, text="Mevcut ID (1-16):").pack(pady=(16, 2))
        entry_old = ctk.CTkEntry(dlg, width=100)
        entry_old.insert(0, "1")
        entry_old.pack()

        ctk.CTkLabel(dlg, text="Yeni ID (1-16):").pack(pady=(8, 2))
        entry_new = ctk.CTkEntry(dlg, width=100)
        entry_new.pack()

        def do_setid():
            try:
                old_id = int(entry_old.get())
                new_id = int(entry_new.get())
                if not (1 <= old_id <= 16 and 1 <= new_id <= 16):
                    raise ValueError
                self._send(f"SETID:{old_id:02d}:{new_id:02d}")
                dlg.destroy()
            except ValueError:
                ctk.CTkLabel(dlg, text="✗ Geçersiz değer!", text_color="red").pack()

        ctk.CTkButton(dlg, text="Gönder", command=do_setid).pack(pady=12)

    # ── SLAVE SEÇİMİ ─────────────────────────────────────────
    def _select_slave(self, slave_id: int):
        self.selected_slave = slave_id
        self.lbl_module.configure(text=f"Seçili Slave: ID {slave_id:02d}")
        self.tabview.set("Modül Test (9 Motor)")
        self._log(f"Slave {slave_id:02d} seçildi.")
        # Slider'ları sıfırla
        for s in self.motor_sliders: s.set(0)
        for lbl in self.motor_labels: lbl.configure(text="0 mm")

    # ── MOTOR KONTROL ────────────────────────────────────────
    def _on_slider(self, motor_idx: int, value: float):
        """Slider hareket edince debounce ile komut gönder."""
        if motor_idx in self.slider_timers:
            self.after_cancel(self.slider_timers[motor_idx])
        timer = self.after(300, lambda: self._send_mov(motor_idx, value))
        self.slider_timers[motor_idx] = timer

    def _send_mov(self, motor_idx: int, value: float):
        val = int(value)
        self._send(f"MOV:{self.selected_slave:02d}:{motor_idx:02d}:{val}")
        self.motor_labels[motor_idx - 1].configure(text=f"{val} mm")
        self.slider_timers.pop(motor_idx, None)

    def _send_home(self, motor_idx: int):
        self._send(f"HOME:{self.selected_slave:02d}:{motor_idx:02d}")

    def _all_home(self):
        self._send(f"ALL:{self.selected_slave:02d}:0")
        for s in self.motor_sliders: s.set(0)
        for lbl in self.motor_labels: lbl.configure(text="0 mm")

    def _all_test(self):
        self._send(f"ALL:{self.selected_slave:02d}:300")
        for s in self.motor_sliders: s.set(300)
        for lbl in self.motor_labels: lbl.configure(text="300 mm")

    # ── LOG ──────────────────────────────────────────────────
    def _log(self, msg: str):
        self.txt_log.insert("end", msg + "\n")
        self.txt_log.see("end")


if __name__ == "__main__":
    app = KineticApp()
    app.mainloop()
