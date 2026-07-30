# CLAUDE.md — Adaptif Kalıp / Kinetic Molding System

> Bu dosya Claude Code tarafından her oturumda otomatik okunur. Projenin kalıcı belleğidir.
> Kod yorumları ASCII (Türkçe karaktersiz) tutulur; doküman ve UI metinleri Türkçe olabilir.

## Proje amacı

12×12 = **144 lineer aktüatörlü** adaptif bir kalıp yüzeyi. Üstte **tek parça (yekpare) silikon**
membran var; bu yüzden motorlar **eş zamanlı** hareket etmeli. Bir STL modeli yüklenir, yüzeyi
12×12 Z-pozisyon grid'ine interpolasyonla çevrilir, her motor kendi Z'sine gider ve membran o şekli alır.

- Aktüatörler: Hall-effect encoder'lı, 0–600 mm strok, **çok yavaş (~4 mm/s)**. Tam strok ≈ 150 sn.
- Demeraj (inrush) akımı yüksek → motorlar **1'er saniye kademeli** başlatılır (eş zamanlı sayılır,
  çünkü 1 sn kademe 150 sn'lik harekette ihmal edilebilir).
- Modüler: her **3×3 = 9 motor** bir "slave" modülü; 16 slave × 9 = 144. Adaptif → kaç modül
  configlenmişse o kadar motor.

## Donanım topolojisi

```
Host (PC / Raspberry Pi 5)
   │ USB seri
Blue Pill (STM32F103)  — ŞEFFAF RS485 köprüsü, 9600 baud
   │ RS485 (adresli)
Slave modülü = 2× Black Pill (STM32F401RC*, HSE 25MHz)
   ├─ U3 "yönetici": RS485 dinler + motor 6-9 (yerel) + flash slave ID
   │     └─ USART2 115200 ──> U2
   └─ U2 "işçi": sadece USART2 + motor 1-5
```

\* `platformio.ini` board = `genericSTM32F401RC`. Kart fiziksel olarak F411 ise board satırı
güncellenmeli (kod aynı kalır; F411'i 84 MHz'de sürer, zararsız).

### Pin / timer haritası (encoder mode)
- **U2 (motor 1-5):** TIM1(PA8/PA9,16bit), TIM2(PA5/PB3,32bit), TIM3(PA6/PA7,16bit),
  TIM4(PB6/PB7,16bit), TIM5(PA0/PA1,32bit). Motor F/R: M1 PB14/PB15, M2 PB4/PB5,
  M3 PB0/PB1, M4 PB8/PB9, M5 PC14/PC15.
- **U3 (motor 6-9):** TIM2,TIM3,TIM4,TIM5 (TIM1 yok; PA8/9/10 RS485'te). Motor F/R: M6 PB4/PB5,
  M7 PB0/PB1, M8 PB8/PB9, M9 PC14/PC15. RS485 DE/RE: PA8, USART1 PA9/PA10. USART2 PA2/PA3 → U2.
- LED: PC13 (her iki kart).

## Kalibrasyon gerçekleri (DOĞRULANDI — donanımdan ölçüldü)

- **9 motorun hepsi tutarlı:** ileri sürüş encoder'ı AZALTIR →
  `MOTOR_ENCODER_REVERSED = true` (hepsi).
- `MOTOR_HOME_RETRACT_FWD = true` (hepsi): "ileri" sürüş aktüatörü GERİ çeker (home yönü).
- `CAL_HARDWARE ≈ 79.93 puls/mm`. v5.1'de motor başına `cal` alanı var (gerekirse tek tek ayarlanır).
- Geçmiş arıza: **Motor 5** ara sıra puls kaçırıyordu → fiziksel aşma (300 yerine 540 mm).
  Sebep encoder **bağlantı sorunuydu**, çözüldü. Ders: bir motor hedefi aşıyorsa önce encoder
  kablosu/konektörünü (ilgili TIM kanalları) kontrol et.
- Yön/ölçüm testi için `Calibration_U2_5motor` / `Calibration_U3_4motor` (USB seri menü: `h`, `t<n>`,
  `f<n>`, `r<n>`, `z`, `l`, `s`). Rehber: `KALIBRASYON_REHBERI.md`.

## Firmware sürümleri

- **v4.4 (çalışan temel, DOKUNMA):** `Adaptif_kalip_v4_slave1_rs485_4motor`, `..._slave2_5motor`,
  `Adaptif_Kalip_blue_pill_rs485_test`. Bloklayıcı, bang-bang. Yedek/referans olarak korunur.
- **v5.1 (GÜNCEL):** `Adaptif_kalip_v5_U3_4motor`, `Adaptif_kalip_v5_U2_5motor`.
  - Tek `Motor` struct + durum makinesi: `IDLE → MOVING/HOMING → SETTLED` (veya `FAULT`). Tamamen NON-BLOCKING.
  - **Histerezis:** `STOP_BAND=20` puls (~0.25mm) içinde dur; `REARM_BAND=60` aşılmadan tekrar hareket etme → salınım biter.
  - **Stall/encoder-kopma koruması:** MOV'da `STALL_WINDOW_MS=1500` içinde `STALL_MIN_PULSES=10` hareket yoksa FAULT.
  - **Encoder giriş filtresi** (IC1F=IC2F=0xF) — hall gürültüsü.
  - **Home (entegre limit switch):** retract yönüne sür; `HOME_GRACE_MS=3000` kalkış payı sonrası
    encoder `HOME_SETTLE_MS=1200` ms değişmezse fiziksel 0 → sayacı sıfırla. `HOME_MAX_MS=200000`
    (tam strok 150s + pay). **Eski 70s timeout bug'ı buydu** (motor dibe inmeden HOMEOK dönüyordu).
  - **1 sn kademeli eş zamanlı:** `STAGGER_MS=1000`. Slot: motor 1-5 → 0..4, motor 6-9 → 5..8.
  - U2 beklenirken U3 yerel motorları servis edilir (kilitlenme yok).
  - Detay: `V5_MIMARI_NOTU.md`.

## Protokol v5.1 (host → U3, RS485)

| Komut | Açıklama | Cevap (ack = "başladı", async) |
|-------|----------|-------------------------------|
| `PING:id` | haberleşme | `PONG:id` |
| `GETID:id` / `SETID:id:yeni` | ID oku/yaz (flash) | `IDVAL` / `IDSET` |
| `MOV:id:motor:mm` | tek motor (test) | `MOVOK:id:motor:mm` |
| `ALL:id:mm` | 9 motor aynı hedef, kademeli | `ALLOK:id:mm` |
| `ARR:id:p1:..:p9` | 9 motora ayrı hedef (STL array) | `ARROK:id` |
| `HOME:id` | 9 motor home, kademeli | `HOMEOK:id:00` |
| `HOME:id:motor` | tek motor home | `HOMEOK:id:motor` |
| `GETPOS:id:motor` | pozisyon | `POS:id:motor:mm:puls` |
| `STAT:id` | 9 motor durumu | `STAT:id:<t1>,..,<t9>` (token=`<durum><mm>`) |

Durum harfleri: `I`=idle, `M`=moving, `H`=homing, `S`=settled (tamam), `F`=fault.
U3↔U2 arası: `INTERNAL_MOV/ALL/ARR/HOME/HOMEALL/GETPOS/STAT`.

**Async kural:** komut hemen ack döner; bitiş **`STAT` poll** ile anlaşılır (hareket ~150s).
Bir slave'e ARR/HOME gönderdikten sonra tüm tokenlar `S` olana kadar `STAT` ile beklenmeli;
home bitmeden hareket komutu göndermek sıfır referansını bozar.

## Arayüz

- **`Web_UI/`** (legacy): Flask backend + React/Vite frontend + Socket.IO. Chromium kiosk'ta Pi'de kasıyor.
- **`UI_v2.0_QML/`** (GÜNCEL, Pi optimize): PySide6 + QML, tek süreç (Flask/tarayıcı yok), GPU
  hızlandırmalı. `main.py` → `backend/controller.py` (QML köprüsü) + `serial_manager.py` +
  `stl_interpolator.py`; `qml/` arayüz. Protokol v5.1. Çalıştırma: `python3 main.py [--fullscreen]`.
  Ayrıntı: `UI_v2.0_QML/README.md`. (Genelde `ui_v2.0` git branch'inde tutulur.)
- STL interpolasyonu: trimesh + scipy `LinearNDInterpolator`, merkez 300mm'ye offset, 0–600 clip.

## Raspberry Pi dağıtımı (UI_v2.0_QML)

Arayüz gerçek panelde çalışıyor: **Pi 5, Raspberry Pi OS Lite 64-bit (Trixie), PySide6 6.11.1,
15.6" 1080p dokunmatik**, `cage` (Wayland kiosk compositor) altında fullscreen.

- **Pi:** `berat@192.168.88.244` (parolasız SSH anahtarı). Repo: `/home/berat/Kinetic_Molding_System`,
  branch `ui_v2.0`. venv: `UI_v2.0_QML/venv`. Log: `/home/berat/kalip.log`.
- **Kiosk:** tty1 otomatik giriş → `/usr/local/bin/kalip` → `cage -- python3 main.py --fullscreen`.
  Yeniden başlat: `sudo systemctl restart getty@tty1` (parolasız izinli).
- **Deploy akışı:** push → Pi'de `git pull --ff-only` → **offscreen QML kontrolu**
  (`QT_QPA_PLATFORM=offscreen venv/bin/python3 main.py`, kiosk'a dokunmadan sozdizimi/binding/
  traceback yakalar; Quick3D "isApiRhiBased ... not functional" BEKLENEN) → kiosk restart → `kalip.log`.
  Kolaylik: `UI_v2.0_QML/deploy_pi.sh` (host'u `PI_HOST` env'den okur).

### Kritik ortam bulguları (kaybolmasın)
1. pip PySide6 wheel'inde `libQt6EglFsKmsGbmSupport.so.6` YOK → **eglfs kullanılamaz**;
   çözüm `cage` + Wayland.
2. systemd + `PAMName=login` ekran verir ama **giriş cihazlarını vermez** (dokunmatik ölü) →
   çalışan yol tty1 otomatik giriş + başlatıcıdan çağırma.
3. `QT_QPA_PLATFORM` **elle ayarlanmaz** (cage `WAYLAND_DISPLAY` verir, Qt eklentiyi kendi seçer);
   "wayland" zorlanınca dokunmatik çalışmadı.
4. Raspberry Pi OS **Bookworm kullanılamaz** (glibc 2.36; PySide6 6.8.1+ `manylinux_2_39`/glibc≥2.39).
   **Trixie şart.**
5. **Hover yok** (parmak) → hoverEnabled/ToolTip/onEntered ölü; tap tabanlı eşdeğer kullan.
6. **Dokunmatik hot-swap "touch sıçraması":** SiS HID Touch Controller (`0457:0819`) ilk
   bağlantıda güvenilir enumerate olmuyor; cage başlarken cihaz hazir degilse eslemeyi kaciriyor
   → dokunma yanlis koordinata dusuyor. **Cozum (otomatik):** cihaz eklenince kiosk'u tazeleyen
   udev+oneshot (Pi'de, repoda degil):
   - `/etc/systemd/system/kalip-touch-reset.service` (Type=oneshot, ExecStartPre sleep 2,
     ExecStart `systemctl restart getty@tty1`)
   - `/etc/udev/rules.d/99-kalip-touch.rules`:
     `ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="0457", ATTR{idProduct}=="0819",
     TAG+="systemd", ENV{SYSTEMD_WANTS}+="kalip-touch-reset.service"`
   Ayni model panel takasinda VID:PID ayni oldugu icin tek kural ikisini de kapsar; her takasta
   kiosk otomatik yenilenip touch dogru eslenir. (udev'den dogrudan uzun systemctl yerine
   oneshot service kullanildi.)

### Uzaktan doğrulama sınırı (donanım-döngüsü gibi)
SSH ile DOĞRULANABİLİR: QML yükleniyor mu, import/traceback, binding uyarısı, uygulama ayakta mı.
DOĞRULANAMAZ (kullanıcıya bırak, "PANELDE TEST" listesi ver): görsel yerleşim/taşma, dokunma isabeti,
tap davranışı, 3D render, akıcılık, punto okunabilirliği.
**GÜVENLİK:** donanıma hareket komutu (ARR/ALL/MOV/HOME) gönderme; seri gerekiyorsa yalnız PING/STAT,
o da kullanıcıya sorarak. config.txt/sudoers/systemd/venv/PySide6 sürümüne dokunma. Kiosk'u ayakta bırak.

## Derleme / yükleme

- PlatformIO: `pio run -e genericSTM32F401RC` (build), `pio run -t upload` (ST-Link ile yükle),
  `pio device monitor -b 115200` (seri).
- Sandbox/bu makinede PlatformIO ve donanım YOK → **derleme ve test fiziksel olarak kullanıcıda yapılır.**

## DONANIM-DÖNGÜSÜ (çok önemli)

Claude kartı flash'layamaz/test edemez. Akış:
1. Claude kodu yazar/düzeltir, mantığı ve pin/timer haritasını mevcut çalışan kodla karşılaştırarak doğrular.
2. **Kullanıcı** PlatformIO ile yükler ve donanımda test eder.
3. Kullanıcı `STAT`/seri çıktısını, hangi motor/komut olduğunu ve gözlemi geri bildirir.
4. Claude buna göre iterasyon yapar. Test prosedürü: `V5_MIMARI_NOTU.md` "Test sırası" bölümü.

Test ederken: önce `HOME:01` → `STAT:01` hepsi `S`/≈0 → sonra `ARR`/`ALL`. Bir encoder kablosunu
gevşetip `STAT`'ta `F` (fault) görülmeli (koruma testi).

## Git

- Remote: `https://github.com/beraT-T/Kinetic_Molding_System.git`. Branch'ler: `main` (firmware + legacy UI),
  `ui_v2.0` (QML arayüz, ayrı tutuluyor).
- Anlamlı, gruplu commit; v4.4'e dokunma. Push GitHub token isteyebilir.
- **Gotcha:** bazen `.git/index.lock` takılı kalır → `rm -f .git/index.lock`.
- `.gitignore`: `.claude/worktrees/` ve `.claude/settings.local.json` ignore edilmeli; ama
  `.claude/commands/`, `.claude/settings.json` **commit edilmeli** (Claude Code yapılandırması).
  (Not: tüm `.claude/` ignore EDİLMEMELİ.)

## Konvansiyonlar

- Firmware: C++ (Arduino framework), register-seviye encoder kurulumu çalışan koddan birebir
  korunur. Yorumlar ASCII.
- Yön/kalibrasyon değerleri motor struct'ında; yeni bir motor ters davranırsa sadece o satır değişir.
- Yeni firmware varyantını v4.4'ün üstüne yazma; yeni `v5/v6` klasörü aç, eski sürümü yedek bırak.
- Büyük/çok-dosyalı değişiklikten önce plan modu kullan (Shift+Tab); değişiklik sonrası `/review`.
```
