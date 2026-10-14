# v2.2 İlerleme

> Oturum yeniden başlarsa veya bağlam sıkıştırılırsa: önce bu dosya, sonra `KARARLAR.md`,
> sonra `GOREV.md`. Kaldığın yerden devam et.

## SABAH ÖZETİ
_(gece sonunda doldurulacak — biçim: GOREV.md bölüm E)_

## Durum
- Branch: `ui_v2.2` (ui_v2.0 @ 0733c9c'den açıldı)
- Aktif faz: Faz 4
- Son commit: (Faz 0 commit'i)

## Başlangıç çalışma ağacı (dokunulmayacak)
Oturum başında zaten vardı: ` D Adaptif_Kalip_blue_pill_rs485_test/Tester_UI/ui_v2.0.py`,
untracked `.claude/` (içinde `settings.local.json.yedek-v22-oncesi`), `AGENTS.md`,
`Adaptif_Kalip_blue_pill_rs485_bridge_usart1/`, `Adaptif_kalip_v5f411_*`, `PMS_GOREVLERI.md`,
`RS485_DENETIM_NOTU.md`, `RS485_Modul_Tester_f411/`, `Tester_UI/`, `V5F411_NOTU.md`,
`hakem_sunumu/`, `makale/`, `photos/`, `UI_v2.0_QML/docs/screenshots/`.

## Plan
**Mimari (Faz 0-3 boyunca kurulacak):**
- `backend/hmi_config.py` — bütün sabitler (tolerans, zaman aşımları, güvenli aralık, veri yaşı)
  + `config/machine.json` yükleyici (expected_modules, firmware_variant).
- `backend/protocol.py` — saf fonksiyonlar: katı STAT ayrıştırma (`^[IMHSF]\d+$`, 9 token,
  `U2ERR` konumu), ACK/ERR ayrıştırma, terminal fiil ayrıştırma, hedef doğrulama (10–590).
- `backend/timers.py` — enjekte edilebilir saat/zamanlayıcı (`QtTimerService`; testte `FakeTimerService`).
- `backend/link.py` — **tek seri işlem yöneticisi**: tek kuyruk, hatta tek bekleyen istek,
  istek başına timeout, sınırlı kuyruk, motion için kör retry yok; cevap ↔ bekleyen istek eşleme.
- `backend/serial_manager.py` — yazma ayrı thread'de (`write_timeout`), okuma tamponu sınırlı,
  kopma sinyali (istisna yutulmaz).
- `backend/sim_bridge.py` — donanımsız simülasyon köprüsü (demo + testlerdeki hata enjeksiyonu
  bunun üstüne kurulur). Demo artık gerçek protokol yolunu kullanır.
- `controller.py` — modül durum deposu (token+zaman), işlem/bağlantı nesli, tek
  `_authorize_motion()`, katı tamamlanma, gözetim poll'u (her zaman, round-robin), kalıcı alarm
  modeli (aktif/onaylı/çözüldü ayrı), atomik model yükleme.
- QML: alarm listesi + "Uyarıyı gördüm", veri yaşı, hedef/ölçüm ayrımı, sayısal tuş takımı,
  SİMÜLASYON bandı, Türkçe metinler, devre dışı nedenleri.

**Faz sırası:** GOREV.md D bölümü (0 → 5). Her faz: test + QML yükleme testi + commit + bu dosya.

## Günlük
<!-- tarih saat — faz — ne yapıldı / sonuç -->
- 2026-10-14 04:13 — başlangıç — branch `ui_v2.2` açıldı, görev dosyaları commit (6fadd42).
- 04:40 — Faz 0 — `backend/timers.py` (QtTimerService / FakeTimerService), `backend/sim_bridge.py`
  (U3 protokol simülatörü + hata enjeksiyonu), Controller'a `transport=`/`timers=` enjeksiyonu
  (varsayılan davranış aynı: SerialManager + QTimer). `tests/support.py` (Rig),
  `tests/test_acceptance.py` (kabul matrisi), `tests/test_qml_load.py` (+ `qml_load_check.py`, ayrı süreç).
  **Kırmızı sonuç (v2.0 kodu, 0733c9c mantığı):** 21 test → 19 başarısız (beklenen), 2 yeşil.

### Faz 0 kırmızı test sonucu (v2.0 davranışı, düzeltme öncesi)
| Matris satırı | Test | v2.0 sonucu |
|---|---|---|
| Yazma başarısız + eski S | M01 `test_write_fail_with_stale_S_never_succeeds` | KIRMIZI: `(True, 'Tamamlandi', 'Tum moduller hedef pozisyonda.')` |
| HOME sırasında eski S300 | M02 `test_stale_S300_does_not_complete_home` | KIRMIZI: gönderilen `HOME, STAT, ARR` — HOME bitmeden ARR |
| Eksik/bozuk STAT, U2ERR | M03 `test_u2err_partial_stat_keeps_fault` | KIRMIZI: `faultText=''` |
| 〃 | M03 `test_garbage_tokens_are_not_success` | KIRMIZI: `Sgarbage`×9 başarı sayıldı |
| HOME sırasında MOV/ALL/terminal | M04 `test_all_motion_paths_blocked_during_home` | KIRMIZI: MOV, ALL, HOME, ARR… hepsi gönderildi |
| İkinci BAŞLA | M05 `test_second_start_sends_nothing` | YEŞİL (backend); fiziksel çift dokunma → panel |
| F/timeout sonrası | M06 `test_fault_keeps_polling_and_locks_motion` | KIRMIZI: arızada `allHomed=True` kaldı |
| 〃 | M06 `test_fault_alarm_survives_acknowledge` | KIRMIZI: onay arızayı sildi |
| 〃 | M06 `test_timeout_creates_persistent_alarm` | KIRMIZI: kalıcı kayıt yok |
| 〃 | M06 `test_completed_module_rearm_is_supervised` | KIRMIZI: REARM izlenmiyor (özellik yok) |
| Eski dispatch callback | M07 `test_old_dispatch_does_not_leak` | KIRMIZI: `HOME:01 → ARR:01 → HOME:02` (inceleme ile aynı) |
| Bozuk yeni STL | M08 `test_failed_load_does_not_rename_old_grid` | KIRMIZI: B adı + A grid'i |
| USB kopma / yeniden bağlanma | M09 `test_disconnect_invalidates_state` | KIRMIZI: kopma sonrası `connected=True` |
| 〃 | M09 `test_manual_disconnect_clears_active_list` | KIRMIZI: aktif liste `[1]` kaldı |
| 〃 | M09 `test_reconnect_does_not_resume_motion` | KIRMIZI: yeniden bağlanınca **ARR kendiliğinden gönderildi** |
| Slave reset | M10 `test_nine_I0_invalidates_reference` | KIRMIZI: 9×I0 sonrası `allHomed=True` |
| 16 modül + gecikme + canlı poll | M11 `test_no_overlapping_requests` | KIRMIZI: 95 hat çakışması (cevap beklenirken yazma) |
| Boş/metin/eksi/>590 hedef | M12 `test_invalid_targets_not_sent` | KIRMIZI: `MOV:01:01:-5`, `MOV:01:01:700`… gönderildi |
| Uzun/bozuk seri akış | M13 `test_line_assembler_bounded` | KIRMIZI: sınırlı tampon yok |
| Fiziksel durdurma (metin) | M14 `test_busy_overlay_has_no_unverified_stop_instruction` | KIRMIZI: "GUCUNU KESIN" metni |
| (QML yükleme) | `test_main_qml_loads_without_warnings` | YEŞİL |
- 05:05 — Faz 1 — Controller yeniden yazıldı (`backend/controller.py`), yeni: `protocol.py`
  (katı STAT, ACK eşleme, terminal fiil, hedef doğrulama), `link.py` (tek seri işlem yöneticisi),
  `hmi_config.py` (sabitler + `config/machine.json`), `serial_manager.py` (yazıcı thread,
  write_timeout, sınırlı tampon, kopma bildirimi), `stl_interpolator.interpolate_stl_to_grid_info`
  (aynı algoritma, bilgi döner — eski fonksiyonla birebir aynı sonuç doğrulandı).
  **Not:** 1b'nin "bu işleme ait ACK + ACK'ten sonra istenmiş STAT" şartı tek-istek seri yönetici
  olmadan yazılamadığı için 2a (link), 2e (alarm modeli) ve 3a (atomik model) çekirdekleri bu
  commit'te kuruldu; QML tarafı (alarm listesi, veri yaşı, Tester yeniden tasarımı) sonraki fazlarda.
  QML (asgari): Tester butonları backend kapısına bağlandı, "Tümünü Sıfırla" kaldırıldı (K6),
  BusyOverlay metni "Bu arayüzden hareket durdurulamıyor." + nötr/amber + `motionActive`'te görünür,
  terminal red sebebini gösterir.
  **Sonuç:** 19 kırmızı testin hepsi yeşil, expectedFailure kalmadı. Ek testler: `test_protocol.py`,
  `test_link.py`, `test_motion_authority.py` → toplam 57 test OK. Demo (gerçek QTimer) duman testi:
  bağlan → tara → durum uzlaşması (9 sn) → üretim (HOME+ARR) 32 sn'de "Tamamlandı".
- 05:45 — Faz 2 — Backend çekirdeği Faz 1'de kurulmuştu; bu fazda: PING zaman aşımı 300 ms +
  100 ms koruma (tarama ~5 sn), bağlantıyı kesme yalnız işlem sürerken kapalı, QML: `StatusStrip`
  (backend `statusInfo`; "Uyarıyı gördüm" 80 px; "Alarmlar (n)"), yeni `AlarmPanel.qml`
  (aktif/görüldü/giderildi), `TouchButton` devre dışı sebep satırı, `Header` (Portları yenile,
  Bağlantıyı kes kapısı, Modülleri bul kapısı), `MainPage` butonları backend kapılarına bağlı,
  `ActionButton` sebep satırı kaydırma, `Theme.stateLabel` (S="Hedefe ulaştı", ""="Durum bilinmiyor"),
  sonuç penceresi "Kapat" (başarı nötr renk). `tests/test_supervision.py` (24 test) +
  `tests/shot_tour.py` (offscreen 1920×1080 ekran görüntüleri). **81 test OK.**

### En kötü durum alarm gecikmesi (Faz 2a, K5)
Kaynak: `backend/hmi_config.py` (`typical_alarm_latency_s`, `worst_alarm_latency_s`, `data_stale_s`).
Gözetim poll'u tick (1 sn) başına TEK modül, hat boşsa; bilinen bütün modüller round-robin.

| Modül (N) | Tipik (hepsi cevap veriyor) | En kötü (turdaki her istek zaman aşımı) | Veri yaşı eşiği |
|---|---|---|---|
| 1 | 1,1 sn | 5,3 sn | 10 sn |
| 4 | 4,1 sn | 13,7 sn | 22,4 sn |
| 9 | 9,1 sn | 27,7 sn | 50,4 sn |
| 16 | 16,1 sn | **47,3 sn** | 89,6 sn |

Formül: tipik = N×1 sn + 0,1 sn; en kötü = N×max(1 sn, 2,5 sn STAT zaman aşımı + 0,3 sn hat koruması)
+ 2,5 sn. Ek gecikmeler: hareket dağıtımı sırasında poll durur (modül başına ACK, normalde ~0,1 sn,
U2 yavaşsa ~1,6 sn, en çok 2,5 sn); terminal/Bakım sorgusu başına ≤2,8 sn. Simülasyonda ölçülen
(16 modül, biri U2-yavaş, test `AlarmLatencyTest`): **10,8 sn**. Gerçek RS485 hattında ölçülmedi.

- 06:20 — Faz 3 — 3a: atomik model (Faz 1'de backend) + MainPage model durumu (Hesaplanıyor /
  Yüklenemedi / ad), hata kalıcı satırda. 3b: `modelInfo` (doldurulan / kırpılan hücre, en büyük
  komşu farkı — yalnız rapor, K14) STL panelinde. 3c: backend modül deposuna **son kabul edilen
  komut hedefi** (`targets`, ACK OK'te yazılır; ERR/zaman aşımında "—"; STAT ezmez);
  `moduleInfo().motors[].target`. Bakım ekranı yeniden tasarlandı: "Ölçülen" / "Hedef" ayrı,
  "Son ölçüm N sn önce", modül karolarında durum etiketi (yok/cevapsız/ALARM/hareket/ref. yok),
  "Canlı Durum" anahtarı yerine "Seçili modülün durumunu sorgula" (gözetim zaten sürekli),
  yeni `NumPad.qml` (uygulama içi sayısal tuş takımı, K13 — Faz 4c maddesi burada yapıldı),
  "9 motoru 300 mm'ye götür" / "Modül X'e referans al" / "Motor X'e referans al" onaylı,
  terminal "HABERLEŞME GÜNLÜĞÜ" + "Günlüğü temizle" + 1080p'de kesilmiyor. Harita/3D başlığı
  "Model hedefleri — ölçülen konum değildir". Onay pencerelerinde model adı, modül listesi, motor
  sayısı, beklenen liste (Faz 4b maddesi). `Theme.dative/accusative` (Modül 3'e, Motor 6'yı).
  BusyOverlay içerik yüksekliğinde. `tests/test_model.py` (12). **93 test OK.**

## ENGELLENDİ / STASH
<!-- ENGELLENDİ: <komut> — <sebep>  |  STASH: v22-wip-<konu> — <dosyalar> — <neden> -->

## Kapsama tablosu

| İnceleme maddesi | Faz | Yapılan | Test | Kalan / gerekçe |
|---|---|---|---|---|
| 1 Eski STAT → başarı | 1b | | | protokol kimliği yok → öneri |
| 2 STOP yok / metin | 1e, 5 | | | donanım değerlendirmesi |
| 3 Tester/terminal kilit aşımı | 1a | | | çift dokunma → panel |
| 4 Arızada gözetim bırakılıyor | 1c | | | |
| 5 USB/slave reset | 2b, 2c, 2d | | | gerçek USB → donanım |
| 6 Kısmi STAT / U2ERR | 2f | | | U3 davranışı → öneri |
| 7 Alarm onayı siliyor | 2e | | | |
| 8 Eski dispatch callback | 1d | | | |
| 9 Atomik STL + şeffaflık | 3a, 3b | | | komşu fark limiti → kullanıcı |
| 10 Hedef/ölçüm/modül karışması | 3c | | | canlı ölçüm haritası → öneri |
| 11 Tek seri yönetici | 2a | | | hat simülasyonu |
| 12 HOME referansı kanıt değil | metin + 5 | | | firmware/donanım |
| 13 Eksik modül / referanssız test / Tester aralığı | 2g | | | config değerleri → kullanıcı |
| 14 UI thread yazma / sınırsız tampon | 2a | | | Pi ölçümü yapılmadı |
| Metin tablosu (16 satır) | 4a | | | |
| Ek dil/akış (7 madde) | 4b | | | |
| Dokunmatik/görsel (6 madde) | 4c | | | panel |
| Kabul matrisi (14 satır) | 0, 5 | | | |
| Demo yetersizliği | 0, 4b | | | |
