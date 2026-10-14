# UI v2.2 — PySide6 + QML (Raspberry Pi optimize)

Tek Python süreci: QML arayüz + seri port + STL interpolasyonu. Flask/tarayıcı yok →
Pi 5'te akıcı (Qt Quick scene-graph'ı GPU ile çizer). Protokol **v5.1** (firmware v5 / v5f411).

> **v2.2 ana fikir:** arayüz bilmediği bir şeyi bildiğini iddia etmez. Ayrıntı ve gerekçeler:
> `docs/v2.2/` (GOREV, INCELEME, KARARLAR, ILERLEME, PROTOKOL_ONERI_v5.2, GUVENLIK_ACIK_KONULAR).
> **Bu arayüzden hareket durdurulamaz** (protokolde STOP yok; fiziksel durdurma değerlendirilmedi).

## Özellikler

- Bağlan / port seç / modül tara (PING 01–16)
- STL yükle → 12×12 grid (algoritma v2.0 ile aynı; doldurulan/kırpılan hücre ve en büyük komşu
  farkı gösterilir) — yükleme **atomik**: dosya adı ve hedefler yalnız başarılı hesaplamada birlikte
- Model hedef haritası + 3D önizleme (**model hedefleri — ölçülen konum değildir**), haritaya
  dokununca modül ayrıntısı (hedef + son ölçüm)
- Üretim (BAŞLA: gerekirse önce referans, sonra şekil), Referans al, Seçili modüle uygula
- Bakım / Motor testi: Ölçülen ≠ Hedef, veri yaşı, uygulama içi sayısal tuş takımı, onaylı hareket
- Kalıcı alarm listesi ("Uyarıyı gördüm" kaydı silmez), durum şeridinde veri yaşı
- Haberleşme günlüğü + terminal (aynı hareket yetkisinden geçer; SETID kilitli)

## Mimari (backend/)

| Dosya | Görev |
|---|---|
| `controller.py` | QML köprüsü. **Tek hareket yetkisi** (`_authorize_motion`), işlem (op) ve bağlantı nesli, katı tamamlanma, sürekli gözetim, alarm modeli, model durumu |
| `link.py` | **Tek seri işlem yöneticisi**: hatta tek bekleyen istek, istek başına zaman aşımı, sınırlı kuyruk, hareket komutlarında kör tekrar yok |
| `protocol.py` | Saf fonksiyonlar: katı STAT şeması (`^[IMHSF]\d+$`, 9 token, `U2ERR`), ACK eşleme, terminal fiilleri, hedef doğrulama (10–590 mm) |
| `serial_manager.py` | pyserial; yazma ayrı thread'de (`write_timeout`), okuma tamponu sınırlı, kopma bildirilir |
| `hmi_config.py` | Bütün sabitler (tolerans, zaman aşımları, veri yaşı, alarm gecikmesi) + `config/machine.json` yükleyici |
| `sim_bridge.py` | Donanımsız U3 simülatörü (`--demo` ve testler; hata enjeksiyonu) |
| `timers.py` | Enjekte edilebilir saat (gerçek: QTimer; test: elle ilerletilen sahte saat) |
| `stl_interpolator.py` | STL → 12×12 (trimesh + scipy), 10–590 mm güvenlik marjı |

`config/machine.json`: `expected_modules` (takılı modüller) ve `firmware_variant` (`"v5.1"` /
`"v5f411"`). `null` = yapılandırılmadı → arayüz varsayım yapmaz (BAŞLA onayında bulunan modüller
tek tek listelenir; `U2ERR`'de motor numarası belirlenmez). Bkz. `docs/v2.2/KARARLAR.md` K3/K4.

## Çalıştırma

```bash
python3 main.py                 # pencere
python3 main.py --fullscreen    # tam ekran (kiosk)
python3 main.py --demo          # donanımsız SİMÜLASYON
```

Pi'de kiosk `cage` (Wayland) altında çalışır; `QT_QPA_PLATFORM` elle **ayarlanmaz**, eglfs pip
wheel'inde yok. Kurulum/dağıtım: kök `CLAUDE.md` → "Raspberry Pi dağıtımı".

## Testler (donanım gerekmez)

```bash
cd UI_v2.0_QML
QT_QPA_PLATFORM=offscreen venv/bin/python3 -m unittest discover -s tests -v
```

- `test_acceptance.py` — INCELEME kabul matrisinin otomatikleştirilebilen satırları
- `test_motion_authority.py`, `test_supervision.py`, `test_model.py` — faz testleri
- `test_protocol.py`, `test_link.py` — birim testleri (bilinen sınır: geç cevap eşleşmesi)
- `test_texts.py` — operatör metinleri (eski metin yok, doğrulanmamış iddia yok, Türkçe karakter)
- `test_qml_load.py` — `Main.qml` ayrı süreçte offscreen yüklenir; beklenmeyen QML/Qt uyarısı
  testi kırar ("Qt Quick 3D is not functional" offscreen'de beklenir ve filtrelenir)

Ekran görüntüleri (offscreen 1920×1080, demo; 3D alan boş çıkar):

```bash
QT_QPA_PLATFORM=offscreen venv/bin/python3 tests/qml_load_check.py --shots docs/v2.2/ekran
```

Testler sahte köprü + sahte saatle çalışır: gerçek RS485 zamanlaması, USB kopması, motor hareketi,
panel okunabilirliği ve dokunma isabeti **burada doğrulanmaz** (PANELDE / DONANIMDA TEST).

## `--demo` sınırları

- Demo, gerçek protokol yolunu (seri kuyruk, ACK, STAT, alarm, yetki) **simülasyon köprüsü** ile
  çalıştırır: 4 modül, ~60 mm/sn (gerçek ~4 mm/sn), firmware'e benzer durum harfleri.
- Ekranda kalıcı **SİMÜLASYON** bandı vardır; sonuçlar gerçek makine hakkında hiçbir şey söylemez.
- Simülatör firmware'in birebir kopyası değildir: RS485 elektriksel zamanlaması, U2 haberleşmesi,
  encoder gürültüsü, stall/HOME algılaması, güç davranışı yaklaşık modellenir veya hiç modellenmez.
- Demo, kabul testi matrisinin yerine geçmez; hata senaryoları `tests/` altındadır.
