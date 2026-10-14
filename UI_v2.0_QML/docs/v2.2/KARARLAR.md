# v2.2 Kararlar

Gece çalışması sırasında soru sorulamadığı için her karara **varsayılan** verildi.
Hepsi `ONAY BEKLİYOR`. Kullanıcı sabah onaylar veya değiştirir.
Yeni bir karar çıkarsa: en muhafazakâr seçenek + aşağıya ekle.

| # | Konu | Varsayılan (bu gece kullanılacak) | Gerekçe | Durum |
|---|---|---|---|---|
| K1 | ARR/ALL hedef toleransı | `TARGET_TOL_MM = 2` | STAT mm tamsayı; firmware STOP_BAND ≈0.25 mm; yuvarlama payı | ONAY BEKLİYOR |
| K2 | HOME sıfır toleransı | `HOME_ZERO_TOL_MM = 2` | aynı; "eski S300" tamamlanma sayılmasın | ONAY BEKLİYOR |
| K3 | Beklenen modüller (`machine.json`) | `expected_modules: null` = **yapılandırılmadı** → amber "Beklenen modül yapılandırılmadı" + BAŞLA öncesi bulunan modülleri **tek tek listeleyen** açık onay | Takılı modülleri bilmiyoruz; "16 zorunlu" varsaymıyoruz; uydurmuyoruz | ONAY BEKLİYOR |
| K4 | Firmware sürümü (`machine.json`) | `firmware_variant: null` → U2ERR gelince yerel tokenların hangi motora ait olduğu **belirlenemez**: 9 motor "bilinmiyor", ama mevcut `F` "modülde arıza (motor no belirlenemedi)" olarak **işlenir** | v5.1'de U2=1-5, v5f411'de U2=5-9; yanlış tahmin yanlış motoru işaretler | ONAY BEKLİYOR |
| K5 | En kötü alarm gecikmesi | Hesaplanıp belgelenir, ekranda veri yaşı gösterilir; politika değişmez | İnceleme: 16 modülde ~16 sn | ONAY BEKLİYOR |
| K6 | "Tümünü Sıfırla" (`ALL:id:0`) | **Kaldır.** 0 mm tam stroka gider → sahte arıza (CLAUDE.md güvenlik marjı) ve referans alma değildir | Daha az hareket yüzeyi | ONAY BEKLİYOR |
| K7 | Tester hedef aralığı | 10–590 mm (STL ile aynı `SAFE_MARGIN_MM`) | Tek doğrulama politikası | ONAY BEKLİYOR |
| K8 | Tester hareket yetkisi | MOV/ALL: işlem yok + aktif arıza yok + modül referanslı. HOME (modül/motor): işlem yok + aktif arıza yok | Giriş/rol sistemi yok; muhafazakâr | ONAY BEKLİYOR |
| K9 | Terminal fiilleri | `PING/STAT/GETPOS/GETID` serbest; `ARR/ALL/MOV/HOME` hareket yetkisinden geçer; `SETID` (flash yazar) **kilitli**; tanınmayan fiil reddedilir | Terminal kilitleri aşmasın | ONAY BEKLİYOR |
| K10 | Veri yaşı eşiği | Poll döngüsünden türetilir (en kötü tur süresi × 2), adlandırılmış sabit | Gereksiz "bilinmiyor" alarmı olmasın, eski veri de normal görünmesin | ONAY BEKLİYOR |
| K11 | Zaman aşımı sonrası | Modül "senkron dışı"; ardışık 2 tutarlı taze STAT gelene kadar tamamlanma/normale dönüş sayılmaz | Protokolde işlem kimliği yok | ONAY BEKLİYOR |
| K12 | HOME bildirimi metni | "Referans alma tamamlandı (firmware bildirimi)" — "doğrulandı" kullanılmaz | Bulgu 12 | ONAY BEKLİYOR |
| K13 | Sayısal giriş | Uygulama içi QML sayısal tuş takımı; Qt VirtualKeyboard eklenmez | Yeni bağımlılık yok | ONAY BEKLİYOR |
| K14 | Komşu yükseklik farkı | Kontrol/limit yok; yalnız rapor | Membran limiti bilinmiyor | ONAY BEKLİYOR |

## Gece eklenen kararlar
<!-- Claude Code buraya ekler: | K15 | ... | ... | ... | ONAY BEKLİYOR | -->
