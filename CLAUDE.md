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
Slave modülü = 2× Black Pill (STM32F401RC veya F411CEU6*, HSE 25MHz)
   ├─ U3 "yönetici": RS485 dinler + motor 6-9 (yerel) + flash slave ID
   │     └─ USART2 115200 ──> U2
   └─ U2 "işçi": sadece USART2 + motor 1-5
```

\* İki ayrı firmware seti var: F401RC için `Adaptif_kalip_v5_*`, F411CEU6 (WeAct BlackPill V3.0)
için `Adaptif_kalip_v5f411_*`. **Sadece board satırını değiştirmek yetmez** — F411'de flash
sektör haritası (SETID) ve saat/voltaj ölçeği farklı. Ayrıntı: `V5F411_NOTU.md`.

### Pin / timer haritası (encoder mode)

**Pinler PCB'ye sabittir — kodda değiştirilemez.** Sürümler arasında değişen tek şey
hangi pin çiftine hangi motor numarasının verildiğidir.

Encoder timer'ları: TIM1(PA8/PA9,16bit), TIM2(PA5/PB3,32bit), TIM3(PA6/PA7,16bit),
TIM4(PB6/PB7,16bit), TIM5(PA0/PA1,32bit). U3'te TIM1 yok (PA8/9/10 RS485'te).
U3: RS485 DE/RE PA8, USART1 PA9/PA10. USART2 PA2/PA3 (U3↔U2). LED PC13 (her iki kart).

| Kart | Sürüş F/R | Encoder | v5.1 no | **v5f411 no** |
|---|---|---|---|---|
| U2 | PB14/PB15 | TIM1 | M1 | **M5** |
| U2 | PB4/PB5 | TIM2 | M2 | **M6** |
| U2 | PB0/PB1 | TIM3 | M3 | **M9** |
| U2 | PB8/PB9 | TIM4 | M4 | **M8** |
| U2 | PC14/PC15 | TIM5 | M5 | **M7** |
| U3 | PB4/PB5 | TIM2 | M6 | **M4** |
| U3 | PB0/PB1 | TIM3 | M7 | **M1** |
| U3 | PB8/PB9 | TIM4 | M8 | **M2** |
| U3 | PC14/PC15 | TIM5 | M9 | **M3** |

- **v5f411 numaralandırması kasadaki fiziksel soket sırasına göredir** (soket 1 = M1 …
  soket 9 = M9). v5.1 sırasına göre soketler 7-8-9-6-1-2-5-4-3 diye gidiyordu, düzeltildi.
- Sonuç: **v5f411'de U3 motor 1-4'ü, U2 motor 5-9'u tutar** (v5.1'de tersiydi).
  Kademe slotları: U3 → 0..3, U2 → 4..8. Host protokolü değişmedi (hep 1-9).
- **PC14/PC15 tuzağı** (v5.1'de M5/M9, v5f411'de M7/M3): Bunlar OSC32_IN/OUT ayakları;
  düz GPIO olarak çalışmalarının tek sebebi firmware'in LSE'yi hiç açmaması (core LSE'yi
  yalnız RTC istendiğinde açar). Projeye RTC / `LSE_CLOCK` eklenirse bu iki motor
  **sessizce ölür** — hata vermez, sadece sürülmez.

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
- **v5f411 (F411CEU6 portu):** `Adaptif_kalip_v5f411_U3_4motor`, `Adaptif_kalip_v5f411_U2_5motor`.
  v5.1 ile **mantık/protokol birebir aynı**; sadece 4 donanım farkı:
  board `blackpill_f411ce`; saat 84→96 MHz (VOS Scale1, 3 wait state, PLL M25/N192/P2/Q4);
  `AddrToSector()` 512 KB sektör haritası (**SETID bunsuz F411'de çalışmaz** — eski kod
  sektör 5 döndürüyordu, ID ise sektör 7'de). Pinler v5.1 ile aynı, ama **motor numaraları
  fiziksel soket sırasına göre yeniden atandı** → U3 = motor 1-4, U2 = motor 5-9.
  Detay: `V5F411_NOTU.md`.

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
İç bağlantıda da host motor numarası kullanılır (çeviri yok): v5.1'de 1-5, v5f411'de 5-9.
`STAT` token sırası her zaman host numarasına göre M1→M9'dur.

**Async kural:** komut hemen ack döner; bitiş **`STAT` poll** ile anlaşılır (hareket ~150s).
Bir slave'e ARR/HOME gönderdikten sonra tüm tokenlar `S` olana kadar `STAT` ile beklenmeli;
home bitmeden hareket komutu göndermek sıfır referansını bozar.

## Arayüz

- **`Web_UI/`** (legacy): Flask backend + React/Vite frontend + Socket.IO. Chromium kiosk'ta Pi'de kasıyor.
- **`UI_v2.0_QML/`** (GÜNCEL, Pi optimize): PySide6 + QML, tek süreç (Flask/tarayıcı yok), GPU
  hızlandırmalı. `main.py` → `backend/controller.py` (QML köprüsü) + `serial_manager.py` +
  `stl_interpolator.py`; `qml/` arayüz. Protokol v5.1. Çalıştırma: `python3 main.py [--fullscreen]`.
  Ayrıntı: `UI_v2.0_QML/README.md`. (Genelde `ui_v2.0` git branch'inde tutulur.)
- STL interpolasyonu: trimesh + scipy `LinearNDInterpolator`, merkez 300mm'ye offset,
  **10–590 mm clip** (güvenlik marjı — aşağıya bak).

### HMI stil rehberi (ISA-101 / IEC 60073 / ISO 9241-303)

Arayüz endüstriyel HMI konvansiyonlarına göre tasarlandı. **Kurallar `qml/Theme.js`
başındaki yorum bloğundadır** (ISA-101 hex dayatmaz, kuruluşun kendi stil rehberini
yazmasını ister — o dosya bu rehberdir). Özet:

- **Renk bütçesi:** normal işletimde ekran nötr gri ("report by exception").
  Kırmızı **yalnız arıza**; amber = anormal/ön-koşul eksik (referans yok, bağlantı yok);
  normal durum **nötr** (yeşil dekoratif kullanılmaz).
- **İstisna:** IEC 60204-1 uyarınca start **aktüatörü** yeşil olabilir → `BAŞLA`
  butonunun yeşili standarda uygundur, nötrlenmemeli.
- **Veri rengi ≠ durum rengi:** ısı haritası **cividis** (algısal düzgün, renk körü
  uyumlu, **kırmızı içermez**) → alarm kırmızısıyla karışmaz. Rainbow/jet kullanma.
- **Ölçüler:** dokunma hedefi ≥80 px (~14 mm), birincil buton 92 px, tehlikeli
  hareketi başlatan tek kontrol (`BAŞLA`) 132 px (~24 mm). Bakım (Tester) ekranında
  yoğunluk için `touchDense` 64 px — bilinçli sapma.
- **Punto:** ISO 9241-303 min 16 yay-dk → gövde 20 px, buton 22-26, sayısal 34.
  Sayısal alanlar monospace (sütun hizası).
- **Uzun işlem:** ekranı kapatan opak modal YOK; ilerleme şeridi aksiyon çubuğunun
  üstüne biner, harita/durum canlı kalır. İlerleme **motor bazlı** (modül bazlı olursa
  1-2 modülde çubuk 150 sn kıpırdamaz), geçen + tahmini kalan süre gösterilir.
- **Durum şeridi** (`StatusStrip.qml`) her sekmede sabit: arıza/bağlantı/referans.
- `font.families` bu Qt sürümünde **yok**, yalnız `font.family` (bkz. `Theme.fontMono`).

### Grid ↔ motor eşlemesi (kablolama için)

`controller._slave_values()` satır-öncelikli dizer, `ARR:id:p1..p9` sırası budur:

```
Modülün 3x3 alanı           12x12 içinde modüller (4x4)
 M1  M2  M3                   1   2   3   4
 M4  M5  M6                   5   6   7   8
 M7  M8  M9                   9  10  11  12
                             13  14  15  16
```
Yani blok içi (satır 0, sütun 0) = **Motor 1**; Modül 1 = satır 0-2 / sütun 0-2.
Kasadaki soket sırası = motor numarası (soket 1 = M1 … soket 9 = M9, v5f411).
Bu **mantıksal** eşleme; modülün fiziksel yönü ters monte edilmişse Tester'dan tek motor
sürerek bir kez doğrula.

### Güvenlik marjı: tam stroka komut verme (sahte FAULT sebebi)

Hedef **0 veya 600 mm** (tam strok) verilirse aktüatör fiziksel dayanmaya oturur, encoder
durur ve v5.1 stall koruması (1500 ms'de <10 puls) bunu **FAULT** sanar → sahte arıza.
Belirti: STL'de 600'e clip'lenen hücrelerin motorları (ör. modül 1 / motor 1-2) sürekli
arıza verir. Çözüm firmware'de değil, **interpolasyonda**: `stl_interpolator.SAFE_MARGIN_MM`
(=10) ile aralık 10–590'a kırpılır.

### STAT poll'u sıralı olmalı (9600 baud)

RS485 köprüsü 9600 baud ≈ 960 bayt/sn; bir STAT sorgu+cevap ≈ 63 bayt ≈ 66 ms.
Tüm bekleyen modüllere aynı anda STAT atmak 16 modülde ≈ 1 sn hat trafiği (%70+ doluluk)
→ çakışma, bozuk satır, yanlış/gecikmeli bitiş teyidi. UI bu yüzden **round-robin** poll eder
(tick başına tek modül, `OP_POLL_MS`) ve komut dağıtımı sürerken poll yapmaz.

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

- PlatformIO **bu makinede kurulu**: `~/.platformio/penv/bin/pio` (PATH'te değil, tam yol gerekir).
  `pio run -d <proje>` (build), `pio run -d <proje> -t upload` (ST-Link), `pio device monitor -b 115200`.
  Env adları: F401 projelerinde `genericSTM32F401RC`, v5f411'de `blackpill_f411ce`.
- **Derleme burada doğrulanabilir; donanım YOK → flash'lama ve gerçek test kullanıcıda.**

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
