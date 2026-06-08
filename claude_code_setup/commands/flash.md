---
description: Bir PlatformIO projesini derle ve ST-Link ile yukle (U2/U3/cal-u2/cal-u3)
---
Hedef: `$ARGUMENTS`

Asagidaki eslemeye gore DOGRU klasoru sec ve PlatformIO komutlarini calistir.
ST-Link'in bagli oldugunu varsay; degilse kullaniciyi uyar.

Eslesme:
- `U2`      -> `Adaptif_kalip_v5_U2_5motor`   (v5.1 isci, motor 1-5)
- `U3`      -> `Adaptif_kalip_v5_U3_4motor`   (v5.1 yonetici, motor 6-9)
- `cal-u2`  -> `Calibration_U2_5motor`        (kalibrasyon, motor 1-5)
- `cal-u3`  -> `Calibration_U3_4motor`        (kalibrasyon, motor 6-9)

Adimlar:
1. Klasore gir, `pio run -e genericSTM32F401RC` ile DERLE. Hata varsa dur ve raporla.
2. Derleme temizse `pio run -t upload` ile YUKLE.
3. Istenirse `pio device monitor -b 115200` ile seri monitoru ac.

Notlar:
- Kart fiziksel olarak F411 ise once `platformio.ini` board satirini guncellemeyi oner (kod aynidir).
- Bu makinede PlatformIO/donanim yoksa kullaniciya tam komutlari ver, o calistirsin.
- v4.4 klasorlerine ASLA yukleme yapma (onlar yedek).
