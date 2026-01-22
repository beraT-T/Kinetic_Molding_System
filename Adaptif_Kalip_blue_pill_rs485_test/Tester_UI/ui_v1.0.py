import customtkinter as ctk
import serial
import serial.tools.list_ports
import threading
import time

# --- AYARLAR ---
ctk.set_appearance_mode("Dark")  # Modes: "System" (standard), "Dark", "Light"
ctk.set_default_color_theme("blue")  # Themes: "blue" (standard), "green", "dark-blue"

class KineticApp(ctk.CTk):
    def __init__(self):
        super().__init__()

        # Pencere Ayarları
        self.title("Kinetik Yüzey Kontrol Paneli v1.0")
        self.geometry("1100x700")

        # Serial Değişkenleri
        self.ser = None
        self.is_connected = False
        self.selected_slave_id = 1 # Varsayılan seçili slave
        
        # Slider debounce için timer'lar
        self.slider_timers = {}  # Her motor için timer sakla

        # --- ARAYÜZ DÜZENİ (GRID) ---
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        # 1. SOL MENÜ (SIDEBAR)
        self.create_sidebar()

        # 2. ANA ALAN (SEKMELER)
        self.tabview = ctk.CTkTabview(self, width=800)
        self.tabview.grid(row=0, column=1, padx=20, pady=20, sticky="nsew")
        
        self.tab_dashboard = self.tabview.add("Genel Bakış (16 Node)")
        self.tab_inspector = self.tabview.add("Modül Test (3x3)")
        self.tab_terminal = self.tabview.add("Terminal / Log")

        # Sekme İçerikleri
        self.create_dashboard_tab()
        self.create_inspector_tab()
        self.create_terminal_tab()

        # Serial Okuma Thread'i Başlat
        self.thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.thread.start()

    def create_sidebar(self):
        self.sidebar_frame = ctk.CTkFrame(self, width=200, corner_radius=0)
        self.sidebar_frame.grid(row=0, column=0, sticky="nsew")
        self.sidebar_frame.grid_rowconfigure(4, weight=1)

        ctk.CTkLabel(self.sidebar_frame, text="BAĞLANTI AYARLARI", font=ctk.CTkFont(size=20, weight="bold")).grid(row=0, column=0, padx=20, pady=(20, 10))

        # COM Port Seçimi
        self.com_ports = self.get_com_ports()
        self.option_port = ctk.CTkOptionMenu(self.sidebar_frame, values=self.com_ports)
        self.option_port.grid(row=1, column=0, padx=20, pady=10)

        # Baud Rate (BluePill USB hızı 115200'dür genelde)
        self.option_baud = ctk.CTkOptionMenu(self.sidebar_frame, values=["9600", "115200", "250000"])
        self.option_baud.set("115200")
        self.option_baud.grid(row=2, column=0, padx=20, pady=10)

        # Bağlan Butonu
        self.btn_connect = ctk.CTkButton(self.sidebar_frame, text="BAĞLAN", command=self.toggle_connection, fg_color="green")
        self.btn_connect.grid(row=3, column=0, padx=20, pady=10)

        # Alt Butonlar
        self.btn_scan = ctk.CTkButton(self.sidebar_frame, text="AĞI TARA (SCAN)", command=self.scan_network, state="disabled")
        self.btn_scan.grid(row=5, column=0, padx=20, pady=10)

    def create_dashboard_tab(self):
        """16 Slave'in durumunu gösteren 4x4 Grid"""
        self.slave_buttons = {} # Butonları saklamak için

        # Grid Ayarı
        for i in range(4): self.tab_dashboard.grid_columnconfigure(i, weight=1)
        for i in range(4): self.tab_dashboard.grid_rowconfigure(i, weight=1)

        slave_id = 1
        for row in range(4):
            for col in range(4):
                # Her Slave için bir kart/buton
                btn = ctk.CTkButton(
                    self.tab_dashboard,
                    text=f"SLAVE ID: {slave_id}\n(Offline)",
                    font=ctk.CTkFont(size=16, weight="bold"),
                    fg_color="#333333", # Gri (Offline)
                    height=100,
                    command=lambda i=slave_id: self.select_slave(i) # Tıklayınca detay aç
                )
                btn.grid(row=row, column=col, padx=10, pady=10, sticky="nsew")
                self.slave_buttons[slave_id] = btn
                slave_id += 1

    def create_inspector_tab(self):
        """Seçili Slave'in 3x3 Motorlarını Kontrol Etme"""
        self.lbl_inspector_title = ctk.CTkLabel(self.tab_inspector, text="Seçili Modül: ID 1", font=ctk.CTkFont(size=24))
        self.lbl_inspector_title.pack(pady=10)

        # 3x3 Motor Grid Frame
        self.motor_frame = ctk.CTkFrame(self.tab_inspector)
        self.motor_frame.pack(pady=10, padx=10, fill="both", expand=True)

        self.motor_sliders = []
        self.motor_home_buttons = []

        for row in range(3):
            for col in range(3):
                motor_idx = row * 3 + col + 1 # 1'den 9'a
                
                # Her motor için ufak bir panel
                frame = ctk.CTkFrame(self.motor_frame)
                frame.grid(row=row, column=col, padx=5, pady=5, sticky="nsew")
                self.motor_frame.grid_columnconfigure(col, weight=1)
                self.motor_frame.grid_rowconfigure(row, weight=1)

                ctk.CTkLabel(frame, text=f"Motor {motor_idx}").pack(pady=5)
                
                # Slider (0-600mm) - Debounce ile komut gönder (slider durduğunda)
                slider = ctk.CTkSlider(frame, from_=0, to=600, orientation="vertical", height=150, 
                                      command=lambda val, m=motor_idx: self.on_slider_change(m, val))
                slider.set(0)
                slider.pack(pady=5)
                self.motor_sliders.append(slider)
                
                # HOME butonu (slider'ın yanında)
                home_btn = ctk.CTkButton(frame, text="HOME", command=lambda m=motor_idx: self.send_home_command(m), 
                                       width=80, height=30, fg_color="orange")
                home_btn.pack(pady=5)
                self.motor_home_buttons.append(home_btn)

        # Toplu Kontrol Butonları
        ctrl_frame = ctk.CTkFrame(self.tab_inspector)
        ctrl_frame.pack(fill="x", pady=10)
        
        ctk.CTkButton(ctrl_frame, text="Tümünü Sıfırla (HOME)", command=self.reset_all_motors).pack(side="left", padx=10, pady=10, expand=True)
        ctk.CTkButton(ctrl_frame, text="Test Pozisyonuna Git (300mm)", command=self.test_position).pack(side="left", padx=10, pady=10, expand=True)

    def create_terminal_tab(self):
        # Terminal çıktı alanı
        self.txt_terminal = ctk.CTkTextbox(self.tab_terminal, font=("Consolas", 12))
        self.txt_terminal.pack(fill="both", expand=True, padx=10, pady=(10, 5))
        
        # Alt panel: Input ve butonlar
        bottom_frame = ctk.CTkFrame(self.tab_terminal)
        bottom_frame.pack(fill="x", padx=10, pady=5)
        
        # Input alanı
        self.entry_terminal = ctk.CTkEntry(bottom_frame, placeholder_text="Komut girin (örn: SETID:01, PING:05)", font=("Consolas", 12))
        self.entry_terminal.pack(side="left", fill="x", expand=True, padx=(0, 5))
        self.entry_terminal.bind("<Return>", lambda e: self.send_terminal_command())
        
        # Gönder butonu
        btn_send = ctk.CTkButton(bottom_frame, text="Gönder", command=self.send_terminal_command, width=100)
        btn_send.pack(side="left", padx=(0, 5))
        
        # Temizle butonu
        btn_clear = ctk.CTkButton(bottom_frame, text="Temizle", command=lambda: self.txt_terminal.delete("0.0", "end"), width=100)
        btn_clear.pack(side="left")

    # --- LOJİK VE FONKSİYONLAR ---

    def get_com_ports(self):
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports] if ports else ["Port Yok"]

    def toggle_connection(self):
        if not self.is_connected:
            try:
                port = self.option_port.get()
                baud = int(self.option_baud.get())
                # Serial bağlantısı - timeout kısa tut (non-blocking okuma için)
                self.ser = serial.Serial(port, baud, timeout=0.1)
                # Buffer'ı temizle
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                self.is_connected = True
                self.btn_connect.configure(text="BAĞLANTIYI KES", fg_color="red")
                self.btn_scan.configure(state="normal")
                self.log(f"Bağlandı: {port} @ {baud}")
            except Exception as e:
                self.log(f"Hata: {e}")
        else:
            if self.ser:
                self.ser.close()
            self.is_connected = False
            self.btn_connect.configure(text="BAĞLAN", fg_color="green")
            self.btn_scan.configure(state="disabled")
            self.log("Bağlantı kesildi.")

    def send_data(self, data, log_message=True):
        """USB Üzerinden Veri Yollar - Thread-safe"""
        if self.is_connected and self.ser:
            try:
                # Satır sonu karakteri \n ekle (BluePill bunu bekliyor)
                msg = f"{data}\n"
                self.ser.write(msg.encode('utf-8'))
                if log_message:
                    self.after(0, self.log, f"Giden -> {msg.strip()}")
            except Exception as e:
                self.after(0, self.log, f"Gönderim Hatası: {e}")
    
    def send_terminal_command(self):
        """Terminal input alanından komut gönder"""
        if not self.is_connected:
            self.log("Hata: Önce bağlantı kurun!")
            return
        
        command = self.entry_terminal.get().strip()
        if command:
            self.send_data(command)
            self.entry_terminal.delete(0, "end")  # Input'u temizle
        else:
            self.log("Uyarı: Boş komut gönderilemez!")

    def read_serial_loop(self):
        """Arka planda sürekli dinleyen fonksiyon - BASİT VE ÇALIŞAN VERSİYON"""
        buffer = ""
        while True:
            if self.is_connected and self.ser:
                try:
                    # Mevcut tüm veriyi oku
                    if self.ser.in_waiting > 0:
                        data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                        buffer += data
                        
                        # Satır sonu karakterlerine göre ayır
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            line = line.strip()
                            if line:
                                # Ana thread'e güvenli şekilde gönder
                                self.after(0, self.log, f"Gelen <- {line}")
                                self.after(0, self.process_incoming_data, line)
                except Exception as e:
                    print(f"Okuma Hatası: {e}")
                    buffer = ""  # Hata durumunda buffer'ı temizle
            else:
                buffer = ""  # Bağlantı yoksa buffer'ı temizle
            
            time.sleep(0.05)  # CPU kullanımını azalt

    def process_incoming_data(self, line):
        """Gelen veriyi işle (PONG, IDSET cevabı vb.) - Ana thread'de çalışır"""
        # PONG:01 (Slave 1 cevap verdi)
        if line.startswith("PONG"):
            try:
                parts = line.split(":")
                if len(parts) >= 2:
                    slave_id = int(parts[1])
                    # İlgili kutuyu YEŞİL yap
                    if slave_id in self.slave_buttons:
                        self.slave_buttons[slave_id].configure(
                            fg_color="green", 
                            text=f"SLAVE ID: {slave_id}\n(Online)"
                        )
            except (ValueError, IndexError) as e:
                print(f"PONG parse hatası: {e}, Line: {line}")
        
        # IDSET:01 (Slave ID başarıyla ayarlandı)
        elif line.startswith("IDSET"):
            try:
                parts = line.split(":")
                if len(parts) >= 2:
                    slave_id = int(parts[1])
                    # İlgili kutuyu YEŞİL yap (ID atandı, online sayılır)
                    if slave_id in self.slave_buttons:
                        self.slave_buttons[slave_id].configure(
                            fg_color="green", 
                            text=f"SLAVE ID: {slave_id}\n(Online - ID Set)"
                        )
            except (ValueError, IndexError) as e:
                print(f"IDSET parse hatası: {e}, Line: {line}")
        
        # HOMEDONE:01:02 (Motor 2 home tamamlandı)
        elif line.startswith("HOMEDONE"):
            try:
                parts = line.split(":")
                if len(parts) >= 3:
                    slave_id = int(parts[1])
                    motor_id = int(parts[2])
                    # Terminal'de zaten görünecek (log fonksiyonu ile)
                    # İsterseniz burada ek işlemler yapabilirsiniz (örn: slider'ı sıfırla)
                    print(f"HOMEDONE: Slave {slave_id}, Motor {motor_id}")
            except (ValueError, IndexError) as e:
                print(f"HOMEDONE parse hatası: {e}, Line: {line}")
        
        # MOVOK:01:02:156 (Motor hareket komutu başarılı)
        elif line.startswith("MOVOK"):
            try:
                parts = line.split(":")
                if len(parts) >= 4:
                    slave_id = int(parts[1])
                    motor_id = int(parts[2])
                    position = int(parts[3])
                    # Terminal'de zaten görünecek
                    print(f"MOVOK: Slave {slave_id}, Motor {motor_id}, Position {position}mm")
            except (ValueError, IndexError) as e:
                print(f"MOVOK parse hatası: {e}, Line: {line}")
        
        # DEBUG:... (Slave'den gelen debug mesajları - Serial Monitor çıktısı)
        elif line.startswith("DEBUG:"):
            # Debug mesajını terminale yazdır (Serial Monitor çıktısı gibi)
            debug_msg = line[6:]  # "DEBUG:" kısmını atla
            self.after(0, self.log, f"[DEBUG] {debug_msg}")
        
        # IDERR:... (ID ayarlama hatası)
        elif line.startswith("IDERR"):
            # Hata mesajını log'da göster (zaten log'da görünüyor)
            pass

    def log(self, message):
        self.txt_terminal.insert("end", message + "\n")
        self.txt_terminal.see("end")

    # --- BUTON AKSİYONLARI ---

    def scan_network(self):
        """Tüm adreslere PING atar - Thread'de çalışır"""
        # Butonu devre dışı bırak (çift tıklamayı önle)
        self.btn_scan.configure(state="disabled")
        
        # Thread'de çalıştır (UI donmasın)
        scan_thread = threading.Thread(target=self._scan_network_thread, daemon=True)
        scan_thread.start()
    
    def _scan_network_thread(self):
        """Ağ taraması thread fonksiyonu"""
        self.after(0, self.log, "--- Ağ Taraması Başladı ---")
        
        # Önce hepsini Gri (Offline) yap
        for i in range(1, 17):
            idx = i  # Closure için
            self.after(0, lambda idx=idx: self.slave_buttons[idx].configure(
                fg_color="#333333", 
                text=f"SLAVE ID: {idx}\n(Offline)"
            ))
        
        # Her slave için PING gönder ve yanıt bekle
        for i in range(1, 17):
            # PING gönder
            if self.is_connected and self.ser:
                try:
                    msg = f"PING:{i:02d}\n"
                    self.ser.write(msg.encode('utf-8'))
                    self.after(0, self.log, f"Giden -> {msg.strip()}")
                except Exception as e:
                    self.after(0, self.log, f"Gönderim Hatası: {e}")
            
            # Slave'in 4 saniye beklemesi + yanıt göndermesi için toplam 4.5 saniye bekle
            time.sleep(0.4)
        
        # Tarama bitti
        self.after(0, self.log, "--- Ağ Taraması Tamamlandı ---")
        self.after(0, lambda: self.btn_scan.configure(state="normal"))

    def select_slave(self, slave_id):
        """Dashboard'da bir kutuya tıklayınca Detay sekmesine git"""
        self.selected_slave_id = slave_id
        self.lbl_inspector_title.configure(text=f"Seçili Modül: ID {slave_id}")
        self.tabview.set("Modül Test (3x3)") # Sekmeyi değiştir
        self.log(f"Modül {slave_id} seçildi.")

    def on_slider_change(self, motor_idx, value):
        """Slider değiştiğinde debounce ile komut gönder (spam önleme)"""
        # Önceki timer'ı iptal et (eğer varsa)
        if motor_idx in self.slider_timers:
            self.after_cancel(self.slider_timers[motor_idx])
        
        # Yeni timer başlat - 300ms sonra komut gönder (slider durduğunda)
        timer_id = self.after(300, lambda: self._send_motor_command(motor_idx, value))
        self.slider_timers[motor_idx] = timer_id
    
    def _send_motor_command(self, motor_idx, value):
        """Motor komutunu gönder (debounce sonrası)"""
        # Protokol: MOV:SlaveID:MotorID:Value
        cmd = f"MOV:{self.selected_slave_id:02d}:{motor_idx:02d}:{int(value)}"
        self.send_data(cmd)
        # Timer'ı temizle
        if motor_idx in self.slider_timers:
            del self.slider_timers[motor_idx]

    def reset_all_motors(self):
        cmd = f"ALL:{self.selected_slave_id:02d}:0"
        self.send_data(cmd)
        # Sliderları da sıfırla
        for slider in self.motor_sliders: slider.set(0)

    def test_position(self):
        cmd = f"ALL:{self.selected_slave_id:02d}:300"
        self.send_data(cmd)
        for slider in self.motor_sliders: slider.set(300)
    
    def send_home_command(self, motor_idx):
        """HOME komutu gönder"""
        cmd = f"HOME:{self.selected_slave_id:02d}:{motor_idx:02d}"
        self.send_data(cmd)
        self.log(f"Giden -> {cmd}")

if __name__ == "__main__":
    app = KineticApp()
    app.mainloop()