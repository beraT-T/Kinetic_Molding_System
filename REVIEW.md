# REVIEW.md — Bu projede kod gözden geçirme kriterleri

`/review` ve GitHub PR review bu dosyayı en yüksek öncelikle kullanır. Aşağıdakileri kontrol et:

## Firmware (STM32 / v5)
- **v4.4 klasörlerine dokunulmamış olmalı** (`Adaptif_kalip_v4_*`, `Adaptif_Kalip_blue_pill_*`).
  Bunlar çalışan yedek; değişiklik yeni `v5/v6` klasöründe olur.
- Pin ve timer haritası `CLAUDE.md`'deki haritayla birebir tutarlı mı? (U2: TIM1-5; U3: TIM2-5,
  RS485 PA8/9/10; motor F/R pinleri.)
- Encoder mode register kurulumu ve 16/32-bit (TIM2/TIM5 32-bit) okuma/wrap mantığı korunmuş mu?
- Yön config'i: `encReversed=true`, `homeRetractFwd=true` (kalibrasyonla doğrulandı). Değişmişse gerekçe?
- Home: `HOME_MAX_MS` tam stroktan (≈150s) büyük mü (≥200000)? Kalkış payı + "encoder durdu" mantığı
  bozulmamış mı? (Eski 70s timeout bug'ına dönülmemeli.)
- Hareket NON-BLOCKING mi (loop kilitlenmiyor)? Histerezis (STOP_BAND/REARM_BAND) ve stall koruması yerinde mi?
- Protokol v5.1 komutları (`ARR`/`HOME`/`STAT`/`INTERNAL_*`) ve ack/async sözleşmesi korunuyor mu?
- Buffer boyutları (LINE_LEN) ve `snprintf` hedef boyutları taşmaya karşı yeterli mi?
- Yorumlar ASCII mi (Türkçe karakter yok)?

## UI (PySide6 / QML — UI_v2.0_QML)
- Seri okuma ayrı thread'de; QML'e veri Qt signal ile (ana thread) aktarılıyor mu? (Doğrudan UI'dan
  bloklayan seri çağrı olmamalı.)
- Protokol v5.1 kullanılıyor mu (ARR/STAT/HOME); async poll mantığı korunuyor mu?
- QML'de değer atama vs binding çatışması yok mu? (slider'lar `applyVals` sinyaliyle güncelleniyor.)
- Pi performansı: gereksiz ağır binding/animasyon, sürekli `requestPaint` döngüsü yok mu?

## Genel
- Donanımda test edilmesi gereken bir değişiklik mi? Öyleyse PR/özet "DONANIMDA TEST GEREKLİ" notu
  ve test adımlarını içermeli (HOME→STAT→ARR).
- Güvenlik: bir motorun sınıra/zorlamaya koşmasına yol açacak yön/stall regresyonu var mı?
- Yeni kalıcı bilgi (karar, sabit, bulgu) `CLAUDE.md`'ye işlenmiş mi?
```
