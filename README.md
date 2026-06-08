# Adaptif Kalıp / Kinetic Molding System

12×12 = **144 lineer aktüatörlü** adaptif kalıp yüzeyi. Üstte tek parça (yekpare) silikon membran
olduğu için motorlar eş zamanlı hareket eder. Bir STL modeli yüklenir, yüzeyi 12×12 Z-pozisyon
grid'ine interpolasyonla çevrilir, her motor kendi Z'sine gider ve membran o şekli alır.

- Aktüatörler: Hall-effect encoder'lı, 0–600 mm strok, ~4 mm/s (tam strok ≈ 150 sn).
- Demeraj akımı yüksek → motorlar 1'er saniye kademeli başlatılır (eş zamanlı sayılır).
- Modüler: her 3×3 = 9 motor bir "slave" modülü; 16 slave × 9 = 144. Adaptif (kaç modül configliyse).

> Projenin tam teknik belleği **`CLAUDE.md`** dosyasındadır (mimari, pin/timer haritası, kalibrasyon,
> protokol, test döngüsü). Claude Code ile çalışırken otomatik okunur.

## Donanım topolojisi

```
Host (PC / Raspberry Pi 5)
   │ USB seri
Blue Pill (STM32F103)  — şeffaf RS485 köprüsü, 9600 baud
   │ RS485 (adresli)
Slave modülü = 2× Black Pill (STM32F401RC, HSE 25 MHz)
   ├─ U3 "yönetici": RS485 + motor 6-9 + flash slave ID
   │     └─ USART2 115200 ──> U2
   └─ U2 "işçi": sadece USART2 + motor 1-5
```

## Firmware

- **v4.4** (`Adaptif_kalip_v4_slave1_rs485_4motor`, `..._slave2_5motor`,
  `Adaptif_Kalip_blue_pill_rs485_test`): çalışan temel, bloklayıcı. Yedek/referans olarak korunur.
- **v5.1** (`Adaptif_kalip_v5_U3_4motor`, `Adaptif_kalip_v5_U2_5motor`): güncel. Non-blocking durum
  makinesi, histerezis (salınım yok), stall/encoder-kopma koruması, encoder giriş filtresi, doğru
  home (entegre limit switch + 200 sn timeout), 1 sn kademeli eş zamanlı hareket, `ARR`/`STAT`/`HOME`
  komutları. Detay: **`V5_MIMARI_NOTU.md`**.
- **Kalibrasyon araçları** (`Calibration_U2_5motor`, `Calibration_U3_4motor`): her motor/encoder
  yönünü ve puls/mm'sini ölçmek için. Rehber: **`KALIBRASYON_REHBERI.md`**.

### Kalibrasyon gerçekleri (doğrulandı)
9 motorun hepsi tutarlı: `MOTOR_ENCODER_REVERSED = true`, `MOTOR_HOME_RETRACT_FWD = true`.
`CAL_HARDWARE ≈ 79.93 puls/mm` (v5.1'de motor başına `cal` alanı var).

## Protokol v5.1 (host → U3, RS485, async)

`PING` · `GETID`/`SETID` · `MOV:id:motor:mm` · `ALL:id:mm` · `ARR:id:p1..p9` · `HOME:id` (hepsi) ·
`HOME:id:motor` · `GETPOS:id:motor` · `STAT:id`. Komut hemen ack döner; bitiş `STAT` poll ile
anlaşılır (token = `<durum harfi><mm>`, durum: I/M/H/S/F). Tam tablo `CLAUDE.md` ve `V5_MIMARI_NOTU.md`.

## Arayüz

- **`UI_v2.0_QML/`** (güncel, `ui_v2.0` branch'inde): PySide6 + QML, tek süreç, GPU hızlandırmalı,
  Raspberry Pi optimize. Flask/tarayıcı yok. Çalıştırma: `python3 main.py [--fullscreen]`.
  Detay: `UI_v2.0_QML/README.md`.
- **`Web_UI/`** (legacy): Flask + React/Vite + Socket.IO. Referans/yedek olarak tutuluyor.

## Derleme / yükleme (PlatformIO)

```bash
pio run -e genericSTM32F401RC      # derle
pio run -t upload                  # ST-Link ile yukle
pio device monitor -b 115200       # seri monitor
```

Kart fiziksel olarak F411 ise `platformio.ini` board satırını güncelle (kod aynı kalır).

## Depo yapısı

```
Adaptif_kalip_v4_*            v4.4 firmware (yedek)
Adaptif_Kalip_blue_pill_*    F103 RS485 koprusu (+ Tester_UI/ui_v2.0.py)
Adaptif_kalip_v5_U2/U3       v5.1 firmware (guncel)
Calibration_U2/U3            donanim test/kalibrasyon firmware'i
UI_v2.0_QML/                 PySide6 + QML arayuz (ui_v2.0 branch)
Web_UI/                      legacy Flask + React arayuz
CLAUDE.md / REVIEW.md        Claude Code bellegi + review kriterleri
CLAUDE_CODE_REHBERI.md       yonetim/kurulum rehberi
V5_MIMARI_NOTU.md            v5.1 mimari + protokol
KALIBRASYON_REHBERI.md       kalibrasyon kullanim kilavuzu
```

## Git

Remote: `https://github.com/beraT-T/Kinetic_Molding_System.git`. Branch'ler: `main` (firmware + legacy
UI + dokümanlar), `ui_v2.0` (QML arayüz). v4.4'e dokunulmaz. Takılı kalırsa: `rm -f .git/index.lock`.
