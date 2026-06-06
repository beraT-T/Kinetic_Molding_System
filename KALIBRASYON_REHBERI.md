# Donanım Test / Kalibrasyon Rehberi

Bu iki proje (`Calibration_U3_4motor`, `Calibration_U2_5motor`) sadece **donanım testi** içindir.
Çalışan v4.4 firmware'ine dokunmaz — aynı pin/timer haritasını kullanır ama hiçbir RS485/U2 haberleşmesi yapmaz, böylece her motoru ve encoder'ı tek başına, güvenle test edebilirsin.

Amaç üç sorunun kesin cevabını almak:

1. Motor fiziksel olarak dönüyor mu?
2. İleri sürdüğümde encoder artıyor mu (DÜZ) yoksa azalıyor mu (TERS)? → `MOTOR_ENCODER_REVERSED`
3. Hangi yön aktüatörü fiziksel olarak geri çekiyor? → home yönü

## Adımlar

1. Test edeceğin kartı ST-Link ile bağla. İlgili klasörde:
   - `Calibration_U3_4motor` → U3 kartı (motor 6,7,8,9)
   - `Calibration_U2_5motor` → U2 kartı (motor 1,2,3,4,5)
2. PlatformIO'da `Upload` ile yükle (`pio run -t upload`).
3. Kartın USB'sini bilgisayara tak, seri monitörü **115200**'de aç (`pio device monitor`).
4. `h` yazıp Enter → komut listesi gelir.

> Kartta USB CDC çalışmıyorsa: `src/main.cpp` başındaki `#define DBG Serial` satırını `#define DBG Serial1` yap ve PA9 (TX)/PA10 (RX) üzerinden bir USB-TTL adaptörüyle bağlan.

## Komutlar

| Komut | Ne yapar |
|-------|----------|
| `h`   | Yardım / komut listesi |
| `l`   | Tüm encoder sayaçlarını gösterir |
| `z`   | Sayaçları sıfırlar |
| `s`   | **ACİL** — tüm motorları durdurur (Enter beklemeden) |
| `t<n>`| Motor n'i otomatik test: ileri sürer, ölçer, geri sürer, sonucu yazar |
| `f<n>`| Motor n'i kısa süre **ileri** sürer (fiziksel yönü gözle gör) |
| `r<n>`| Motor n'i kısa süre **geri** sürer |
| `ta`  | Tüm motorları sırayla otomatik test eder |

`n` = U3 için 6/7/8/9, U2 için 1/2/3/4/5. Örnek: `t6`, `f1`, `r3`.

## Nasıl yorumlanır

`t<n>` çıktısında:

- **İLERİ sürüş → encoder delta pozitif** ise encoder DÜZ → `MOTOR_ENCODER_REVERSED = false`
- **İLERİ sürüş → encoder delta negatif** ise encoder TERS → `MOTOR_ENCODER_REVERSED = true`
- Her iki yönde de delta ~0 ise motor dönmüyor ya da encoder okumuyor → kablo/güç/sürücü kontrol et.
- Tek yönde hareket varsa motor sınırdadır; önce `f`/`r` ile biraz ortaya getir, sonra tekrar `t`.

Home yönü için: `f<n>` ve `r<n>` ile dene, **aktüatörü fiziksel olarak geri çeken (kısaltan)** yön hangisiyse onu not et. İleri (`f`) geri çekiyorsa `MOTOR_HOME_RETRACT_FWD = true`, geri (`r`) çekiyorsa `false`.

## Sonuç tablosu — doldur

Bu değerleri buraya yazarsan, sıfırdan temiz mimariyi bunlarla doğru kurarız.

### U2 (motor 1-5)

| Motor | Dönüyor? | İleri→encoder işareti | ENCODER_REVERSED | Geri çeken yön (f/r) | HOME_RETRACT_FWD |
|-------|----------|----------------------|------------------|----------------------|------------------|
| M1 |  |  |  |  |  |
| M2 |  |  |  |  |  |
| M3 |  |  |  |  |  |
| M4 |  |  |  |  |  |
| M5 |  |  |  |  |  |

### U3 (motor 6-9)

| Motor | Dönüyor? | İleri→encoder işareti | ENCODER_REVERSED | Geri çeken yön (f/r) | HOME_RETRACT_FWD |
|-------|----------|----------------------|------------------|----------------------|------------------|
| M6 |  |  |  |  |  |
| M7 |  |  |  |  |  |
| M8 |  |  |  |  |  |
| M9 |  |  |  |  |  |

### Kalibrasyon sabiti (puls/mm)

`l` ile sayacı sıfırla (`z`), bir motoru bilinen bir mesafe (örn. cetvelle 100 mm) hareket ettir, encoder sayacını oku. `puls / mm` = senin `CAL_HARDWARE` değerin. Şu an kodda **79.93** sabit. Her motor için ölçüp not et — farklılık varsa motor başına ayrı sabit gerekebilir.

| Motor | Ölçülen mm | Encoder puls | puls/mm |
|-------|-----------|--------------|---------|
| | | | |
