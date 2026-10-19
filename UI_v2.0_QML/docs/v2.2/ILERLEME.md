# v2.2 İlerleme

> **2026-10-19:** bağımsız inceleme bulguları ayrı dalda düzeltildi
> (`claude/ui-v2.2-bulgu-duzeltme`). Plan, kırmızı/yeşil kanıtlar ve teslim raporu:
> **`DUZELTME_TURU.md`**. Aşağıdaki "SABAH ÖZETİ" ilk gece turunu anlatır; oradaki "102 OK",
> K5 gecikme tablosu ve Bakım ekranı tarifinin güncel hali `DUZELTME_TURU.md`'dedir.

> Oturum yeniden başlarsa veya bağlam sıkıştırılırsa: önce bu dosya, sonra `KARARLAR.md`,
> sonra `GOREV.md`. Kaldığın yerden devam et.

## SABAH ÖZETİ

**1. Fazlar ve commit'ler** — Faz 0–5 **bitti**. Branch `ui_v2.2` (push YOK). Son kod/doküman
commit'i `b513f3c`; bu özet bir sonraki `docs(v2.2)` commit'inde (`git log -1 ui_v2.2`).
`6fadd42` görev dosyaları → `c40d0dd` Faz 0 → `eaefe6c` Faz 1 → `991d470` Faz 2 → `3312495` Faz 3 →
`7a5df5b` Faz 4 → `b513f3c` düzeltme + Faz 5 dokümanları. Testler: **102 OK**
(`cd UI_v2.0_QML && QT_QPA_PLATFORM=offscreen venv/bin/python3 -m unittest discover -s tests -v`).
Not: Faz 4 commit'i ilk denemede (`e17bfe0`) 1 kırmızı metin testiyle oluştu; test düzeltilip
**yerelde amend** edildi (`7a5df5b`, push yapılmadığı için geçmiş paylaşılmamıştı).

**2. Kırmızı → yeşil** — Faz 0'da v2.0 davranışını yeniden üreten **19 kabul testinin hepsi
yeşil** (tablo aşağıda "Faz 0 kırmızı test sonucu"). `expectedFailure` ile kalan test **yok**.
Yeşil kalan: M05 ikinci BAŞLA (backend), QML yükleme.

**3. Operatörün göreceği davranış değişiklikleri**
- Hareket yalnız tek yetki noktasından: üretim, Bakım butonları ve **terminal** aynı kilitlere
  bağlı. HOME sürerken Bakım/terminal hareketi reddedilir; reddin sebebi gösterilir.
- Kilitler: işlem sürüyor, herhangi bir modülde hareket/durgunluk doğrulanmadı, veri eski/eksik,
  çözülmemiş alarm (arıza, U2, cevapsız modül, senkron dışı, bağlantı), referanssız (MOV/ALL/ARR).
  **Aktif motor arızasında referans alma da kilitli** (K15) — kurtarma modülün yeniden başlatılmasını
  gerektirir.
- Başarı yalnız ACK + ACK'ten sonra istenmiş taze STAT + 9 geçerli token + ±2 mm ile; mesaj kapsamı
  söyler ve "güvenli erişim anlamına gelmez" der. ERR/ACK zaman aşımı/yazma hatasında işlem durur,
  **kalan modüllere gönderilmez, otomatik tekrar yok**.
- Arıza/zaman aşımından sonra izleme sürer; hareket bandı "Hareket izleniyor" der. Bant metni:
  **"Bu arayüzden hareket durdurulamıyor."** (kırmızı değil; talimat yok).
- Durum şeridi: veri eskiyse "Güncel makine durumu bilinmiyor."; "Uyarıyı gördüm" (80 px) kaydı
  **silmez**; "Alarmlar (n)" listesi: aktif / görüldü / giderildi + neden/etki/adım.
- Bağlantı kopması/kesilmesi: bulunan liste boşalır, referans düşer, yeniden bağlanınca
  **otomatik hareket yok**; önce durum okunur.
- Model: yükleme sırasında BAŞLA kapalı; bozuk dosyada "Yüklenemedi", **yüklü model yok** (K18);
  doldurulan/kırpılan hücre ve en büyük komşu farkı gösterilir.
- Bakım ekranı: "Ölçülen" ve "Hedef" ayrı, veri yaşı, sayısal tuş takımı (10–590 mm), "Tümünü
  Sıfırla" **kaldırıldı**, "Canlı Durum" yerine "Seçili modülün durumunu sorgula", her hareket onaylı.
- Haritaya dokunma: modül ayrıntısı penceresi. `--demo`'da kalıcı SİMÜLASYON bandı.
- Tarama ~5 sn (PING 0,3 sn); bağlandıktan sonra BAŞLA, bütün modüllerin durumu okunana kadar
  (~N sn) kapalı.

**4. ONAY BEKLİYOR** — `KARARLAR.md` **K1–K29 hepsi** (K1–K14 görevden varsayılan, K15–K29 gece
eklendi). Öncelikle bakılması önerilen: **K15** (arızada referans kilidi), **K16** (global kilit),
**K17** (cevapsız modül kilidi), **K18** (bozuk STL'de model yok), **K19** (eksi token), **K3/K4**
(`config/machine.json` değerleri — şu an `null`), **K26** (arıza metni, prosedür yok).

**5. ENGELLENDİ / stash** — Yok. Hiçbir işlem engellenmedi; stash kullanılmadı.

**6. PANELDE TEST** (Pi, 15,6" 1080p dokunmatik; doğrulanamadı → kullanıcı)
1. Durum şeridi 50–70 cm'den okunuyor mu (amber üstü koyu metin, 24 px); "Alarmlar (n)" ve
   "Uyarıyı gördüm" eldivenle tek dokunuşta isabet ediyor mu.
2. Bakım ekranında terminal giriş satırı, "Gönder" ve "Günlüğü temizle" 1080p'de tamamen görünüyor;
   "Kılavuz" metni kesilmiyor; motor hücrelerinde "Ölçülen/Hedef" okunuyor.
3. "Hedef gir" → tuş takımı: tuş isabeti, Sil/Temizle, 5 veya 600 girince amber uyarı ve eylem
   butonunun kapalı olması.
4. Haritaya dokun → modül ayrıntısı açılıyor; dışına dokununca kapanıyor; "Hedef modül yap".
5. Çift dokunma: BAŞLA'ya ve onay butonuna hızlı iki kez → günlükte tek "İşlem başladı".
6. Devre dışı butonlardaki sebep satırı (16 px amber) okunuyor mu; uzun sebep kısaltılıyorsa tam
   metin Bakım ekranında "Hareket kapalı: …" satırında görünüyor mu.
7. Hareket bandı aksiyon çubuğunu kapatıyor, harita ve durum şeridi görünür kalıyor mu.
8. Parlama/güneş altında sönük metin (`textFaint`) okunabilirliği.
9. Terminal TextField: Pi'de dokunmatik klavye yok → fiziksel klavye gerekli (beklenen).
10. STL Seç dosya penceresi dokunmatikte kullanılabiliyor mu.
11. 3D önizleme çiziliyor mu (offscreen'de doğrulanamadı).
12. Akıcılık: saniyede bir durum yenilemesi (modül karoları, motor hücreleri) takılma yaratıyor mu.

**7. DONANIMDA TEST GEREKLİ** (komutları kullanıcı gönderir; önce `config/machine.json`'a
`firmware_variant` ve `expected_modules` yazılmalı; kalıp alanı boş; güç kesme noktası bilinmeli)
1. Bağlan → Modülleri bul: bulunan liste doğru; şerit önce "Güncel makine durumu bilinmiyor",
   birkaç saniye sonra "Referans alınmadı". Günlükte "beklenmeyen cevap" satırı çıkmamalı.
2. Bakım → "Modül 1'e referans al" → `HOME:01` → STAT'larda H → S0 → "Referans alma tamamlandı
   (firmware bildirimi)"; karo "bağlı".
3. `STAT:01` (terminal): 9 token `S0`; Bakım ekranında Ölçülen 0, Hedef 0.
4. Bakım → Motor 1 → Hedef gir 300 → `MOV:01:01:300`; Ölçülen artıyor, Hedef 300; sonuç "hedefe
   ulaştı (300 mm; ±2 mm)". ±2 mm toleransı (K1) gerçek değerlerle uyumlu mu?
5. "9 motoru 300 mm'ye götür" (`ALL:01:300`) → tamamlanma; sonra ana sayfa "Referans al".
6. Model yükle → BAŞLA → onayda model adı/modül listesi/motor sayısı → (gerekirse HOME) → `ARR` →
   "Modül … hedefe ulaştı".
7. Koruma: hareket sırasında bir encoder kablosu gevşetilir (yetkin kişi, güvenli biçimde) → F
   alarmı, "Uyarıyı gördüm" sonrası kayıt "AKTİF · görüldü" kalır, bütün hareket kilitli, diğer
   modüller izleniyor (günlükte STAT sürüyor).
8. U3–U2 kablosu çıkarılır → STAT'ta `U2ERR` → U2 alarmı, U2 motorları "—"; takılınca tam STAT ile
   "giderildi". `firmware_variant` doğru motorları "—" yapıyor mu?
9. Hareket yokken USB çekilir → kırmızı "Bağlantı koptu…", liste boş, referans düştü; takılıp
   Bağlan + Modülleri bul → **hiçbir hareket komutu gitmiyor**.
10. Modül enerjisi kesilip verilir (yetkin kişi) → STAT `I0` → "yeniden başlamış olabilir" uyarısı,
    referans düştü; K15 kurtarma yolu bu.
11. Terminal: `SETID:01:02` reddedilir; `MOV:01:01:0` reddedilir (10–590); HOME sürerken `MOV`
    reddedilir.
12. Zamanlama: günlükte `→ ARR` ile `← ARROK` arası (U2 beklemesi dahil) < 2,5 sn mi (K22); STAT
    cevabı < 2,5 sn mi.
13. Alarm gecikmesi: bir motor F'ye düşünce ekranda görünme süresi (beklenen ~N×1 sn; en kötü
    16 modülde 47,3 sn — K5).
14. Eksi konum tokenı (`S-1`) görülürse not edilmeli (K19: şu an "bilinmiyor" + kilit).

**8. Pi'ye deploy önerisi** (kullanıcı yapacak; kiosk `ui_v2.0`'da kalıyor)
1. Mac'te inceleme sonrası: `git push -u origin ui_v2.2`.
2. Pi'de kiosk'a dokunmadan ayrı çalışma ağacı: `git fetch` → `git worktree add
   ../kalip_v22 ui_v2.2`; ortak venv ile offscreen test:
   `cd ../kalip_v22/UI_v2.0_QML && QT_QPA_PLATFORM=offscreen
   /home/berat/Kinetic_Molding_System/UI_v2.0_QML/venv/bin/python3 -m unittest discover -s tests`
   (Quick3D "not functional" beklenen).
3. `config/machine.json`'u doldur (K3/K4), PANELDE TEST listesini `--demo` ile panelde dene.
4. Kiosk'u ancak bundan sonra `ui_v2.2`'ye al (`git checkout ui_v2.2` + `sudo systemctl restart
   getty@tty1`, `kalip.log` izle). Geri dönüş: `git checkout ui_v2.0` + restart.
5. DONANIMDA TEST listesini sırayla uygula.

## Durum
- Branch: `ui_v2.2` (ui_v2.0 @ 0733c9c'den açıldı)
- Aktif faz: **bitti** (Faz 0–5)
- Son kod commit'i: b513f3c

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
+ 2,5 sn. **Düzeltme (2026-10-19):** bu tablo yalnız gözetim trafiği varken geçerliydi; terminal
sorguları varken 1 modülde 84,8 sn ölçüldü. Güncel sınırlar ve varsayımlar: `DUZELTME_TURU.md` →
Bulgu 4 ve `hmi_config.worst_alarm_latency_loaded_s`. Ek gecikmeler: hareket dağıtımı sırasında poll durur (modül başına ACK, normalde ~0,1 sn,
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
- 06:55 — Faz 4 — Metin tablosu (16 satır) ve ek dil/akış (7 madde) aşağıdaki kapsama tablosunda
  satır satır. Yeni: alarm kayıtlarında **neden / etki / adım** (`ALARM_HELP`, AlarmPanel'de),
  `--demo` için kalıcı **SİMÜLASYON** bandı, `AppDialog` tek butonda "Kapat", haritada modül
  çerçevesi + dokununca `ModuleDetail.qml` (büyük hücreli hedef/ölçüm), harita ipucu Türkçe
  (Satır/Sütun/Modül/Motor), F yardım metni "sürüşü kesildi (firmware bildirimi)" — "durdu"
  iddiası yok. `tests/test_texts.py` (eski metinler yok, gerekli metinler var, "doğrulandı"/"durdu"
  pozitif iddiası yok, "güvenli" yalnız olumsuz, ASCII'leştirilmiş Türkçe kelime yok).
  Ekran görüntüleri (offscreen 1920×1080, demo): `docs/v2.2/ekran/01…09` (3D alan boş — beklenen).
  **99 test OK.**

- 07:30 — Faz 5 — Kendi gözden geçirmemde hata bulundu ve düzeltildi: reset tespiti modül
  bazlıydı → tek motor referansından sonra diğer motorların `I` kalması yanlış "Modül yeniden
  başladı" üretiyordu; motor bazlıya çevrildi (+2 test). Gerçek `SerialManager(loop://)` + QTimer
  ile Controller entegrasyon testi eklendi (thread'ler arası taşıma + kopma). Dokümanlar:
  `PROTOKOL_ONERI_v5.2.md`, `GUVENLIK_ACIK_KONULAR.md`, `CLAUDE.md` (v2.2 çekirdek kuralları; Pi
  bölümüne dokunulmadı), `README.md`. **102 test OK** (b513f3c). Kapsama tablosu + SABAH ÖZETİ yazıldı.

## ENGELLENDİ / STASH
Yok — hiçbir komut engellenmedi, stash kullanılmadı.

## Kapsama tablosu

### İnceleme bulguları (14 madde)
| İnceleme maddesi | Faz | Yapılan | Test | Kalan / gerekçe |
|---|---|---|---|---|
| 1 Eski STAT → başarı | 1b | Yazma sonucu işlenir; ACK/ERR işleme bağlanır; tamamlanma = ACK + ACK'ten sonra istenmiş taze STAT + katı şema + ±2 mm (HOME ≈0); işlem/bağlantı nesli; zaman aşımı → senkron dışı + 2 tutarlı STAT (K11) | M01, M02, M03b, `AckVsCompletionTest`, `test_link` | Protokolde işlem kimliği yok → geç cevap eşleşmesi kalan risk (`test_residual_risk_…`), PROTOKOL_ONERI §2 |
| 2 STOP yok / metin | 1e, 5 | "Bu arayüzden hareket durdurulamıyor." (talimatsız, nötr/amber); her hareket komutunda (Bakım dahil) ve hareket doğrulanana kadar görünür | M14, `test_texts` | Fiziksel durdurma/enerji ayırma → yetkin değerlendirme (GUVENLIK_ACIK_KONULAR §1); yazılımsal STOP önerisi §1 |
| 3 Tester/terminal kilit aşımı | 1a | `_authorize_motion` tek nokta; bütün slotlar + terminal (SETID kilitli, tanınmayan red); Tester butonları backend kapısına bağlı; Enter yolu yerine tuş takımı | M04, M05, `TerminalAuthorityTest` | Fiziksel çift dokunma → PANELDE TEST 5 |
| 4 Arızada gözetim bırakılıyor | 1c | İş sonucu ≠ gözetim; sürekli round-robin STAT (tamamlanan/arızalı modüller dahil); M/H/durgunluk doğrulanmadı → kilit; F → referans düşer; REARM izlenir | M06 (4 test), `test_err_ack_fails_op_keeps_supervision` | — |
| 5 USB/slave reset | 2b, 2c, 2d | Okuyucu istisnası → kopma sinyali; bağlantı nesli; erişilemez modül listeden düşer, kopmada liste temizlenir; SYNC ile uzlaştırma; I → referans düşer (motor bazlı); veri yaşı | M09 (3), M10, `ReachabilityTest`, `ResetDetectionTest`, `RealThreadPathTest`, `SerialManagerLoopTest` | Gerçek USB çekme / Pi yeniden başlatma → DONANIMDA TEST 9–10 |
| 6 Kısmi STAT / U2ERR | 2f | Katı yorum; U2ERR konumu + `machine.json` sürümü (K4); F kaybolmaz; bilinmeyen "—"/"Durum bilinmiyor"; U2 alarmı kalıcı | M03a, `PartialStatTest`, `test_protocol` | U3'ün U2 hatasında yerel motorları zamanlaması → PROTOKOL_ONERI §4 |
| 7 Alarm onayı siliyor | 2e | Kalıcı alarm modeli: onay ≠ giderildi; işlem dışı F alarm; zaman aşımı kaydı; yeni işlem temizlemez; iletişim belirsizliğinde çözülmez | M06b/c, `AlarmLifecycleTest` | Alarm geçmişi bellek içi (öneri: dosya, PROTOKOL_ONERI Ek A.3) |
| 8 Eski dispatch callback | 1d | Dağıtım ACK zinciri; her callback op id + faz + link nesli kontrolü; işlem bitince kuyruktaki komut iptal | M07, `GenerationTest` | — |
| 9 Atomik STL + şeffaflık | 3a, 3b | Model durum makinesi + yükleme kimliği; ad+grid birlikte; hata operatöre; yüklemede hareket ve ikinci yükleme yok; işlem başında değişmez kopya; doldurulan/kırpılan/komşu fark | M08, `AtomicLoadTest`, `InterpolationInfoTest`, `test_targets_are_immutable_copy` | Komşu fark limiti bilinmiyor (K14); birim/yön/tek katman varsayımı doğrulanmadı (README, stl_interpolator) |
| 10 Hedef/ölçüm/modül karışması | 3c | Modül deposu (kimlik + zaman); Ölçülen/Hedef ayrı; seçim değişince yalnız o modül; STAT hedefi ezmez; harita/3D başlığı | `TargetVsMeasurementTest` | Canlı ölçüm haritası → öneri (PROTOKOL_ONERI Ek A.1) |
| 11 Tek seri yönetici | 2a | `link.py`; Tester bağımsız timer'ı kaldırıldı; tarama yeniden girişe kapalı; dağıtım ACK bekler; en kötü alarm gecikmesi hesaplandı (K5) | M11, `test_link`, `AlarmLatencyTest` | Gerçek hat ölçümü yok (DONANIMDA TEST 12–13) |
| 12 HOME referansı kanıt değil | metin + 5 | "Referans alma tamamlandı (firmware bildirimi)", "fiziksel doğruluğu doğrulanamaz" | `test_texts` | Firmware/donanım (PROTOKOL_ONERI §3) |
| 13 Eksik modül / referanssız test / aralık | 2g | `machine.json`; beklenen ≠ bulunan kilidi (K3, K23); onayda liste; Seçili modül aktif + referanslı; Bakım 10–590; Tümünü Sıfırla kaldırıldı (K6) | `ExpectedModulesTest`, M12, `MachineConfigTest` | Gerçek değerler kullanıcıda (K3/K4) |
| 14 UI thread yazma / tampon | 2a | Yazıcı thread + `write_timeout`; satır tamponu, satır kuyruğu ve istek kuyruğu sınırlı; tek model işi; kapatma bloklamaz | M13, `SerialManagerLoopTest`, `test_bounded_queue` | Pi'de donma/bellek/FPS ölçülmedi |

### Metin tablosu (16 satır, 4a)
| # | Eski metin | Yeni metin / davranış | Durum |
|---|---|---|---|
| 1 | DURDURMAK ICIN CIHAZIN GUCUNU KESIN | "Bu arayüzden hareket durdurulamıyor." Talimat yok (prosedür doğrulanmadı) | Yapıldı (talimat kısmı → GUVENLIK §1) |
| 2 | Tum moduller hedef pozisyonda. | "Modül 1, 2 (18 motor)…" / "Seçili 1 modülün 9 motoru hedefe ulaştı (son durum sorgusu, ±2 mm). Bu, kalıp alanına güvenli erişim anlamına gelmez." | Yapıldı |
| 3 | Anormal durum yok | Veri eskiyse "Güncel makine durumu bilinmiyor."; normalde "…· en eski durum N sn önce" | Yapıldı |
| 4 | Islem kesildi / Baglanti kesildi | "Bağlantı kesildi. Hareketin durduğu doğrulanamıyor." | Yapıldı |
| 5 | Yanit vermeyen modul | "Süre içinde tamamlanma doğrulanamadı. Modül X: son durum …, N sn önce." | Yapıldı |
| 6 | ONAYLA | "Uyarıyı gördüm" (kayıt silinmez) | Yapıldı |
| 7 | Tumunu Sifirla | Kaldırıldı (K6) | Yapıldı |
| 8 | Home Hepsi, Home | "Modül X'e referans al" / onayda "Motor X'e referans al"; hücre butonu "Ref." (K27) | Yapıldı (etiket kısaltması K27) |
| 9 | Test 300 mm | "9 motoru 300 mm'ye götür" + onay (modül, süre) | Yapıldı |
| 10 | POZISYON HARITASI | "MODEL HEDEFLERİ — ölçülen konum değildir" (3D başlığı da) | Yapıldı |
| 11 | Hazir / Bosta | S = "Hedefe ulaştı"; geçersiz/eksik = "Durum bilinmiyor"; I = "Boşta" | Yapıldı |
| 12 | REFERANS ALINMADI - islem oncesi home yapilacak | "Referans alınmadı — hareketten önce referans alınmalı (BAŞLA bunu kendisi yapar)"; hareketler gerçekten engelli | Yapıldı (metin K25) |
| 13 | STL yukleyip 'Hesapla' deyince… | "STL Seç ile dosya seçin. Hesaplama tamamlanınca önizleme burada görünür." | Yapıldı |
| 14 | Once bir STL yukleyip hesaplayin. | "Model yok. STL Seç ile bir model yükleyin ve hesaplamanın tamamlanmasını bekleyin."; BAŞLA hesaplama bitmeden kapalı | Yapıldı |
| 15 | Encoder kablosunu ve mekanigi kontrol edin. | "Motor arızası bildirildi: … Diğer motorların durduğu doğrulanmadı; makine izlenmeye devam ediyor." | Kısmen: prosedür cümlesi eklenmedi (K26, prosedür tanımlı değil) |
| 16 | Canli Durum surekli izler | "Seçili modülün durumunu sorgula" + "Son ölçüm N sn önce" (gözetim zaten sürekli) | Yapıldı |

### Ek dil/akış (7 madde, 4b)
| # | Madde | Durum |
|---|---|---|
| 1 | Türkçe karakterler; Kılavuz; "Bakım / Motor testi"; "HABERLEŞME GÜNLÜĞÜ" | Yapıldı (`test_texts` denetler) |
| 2 | ARRERR/U2ERR/istisnalar → neden/etki/adım | Yapıldı: işlem sonuç pencereleri + alarm listesinde `ALARM_HELP` + model/bağlantı hata pencereleri |
| 3 | "Portları yenile", "Günlüğü temizle" | Yapıldı |
| 4 | İptal/Tamam → Vazgeç/Kapat; "Tekrar dene" eklenmedi | Yapıldı |
| 5 | Devre dışı nedenleri bütün butonlarda; modül yoksa BAŞLA kapalı | Yapıldı (TouchButton/ActionButton sebep satırı; Bakım ekranında tam sebep satırı) |
| 6 | Onaylarda model adı, modül listesi, motor sayısı | Yapıldı (+ beklenen liste / yapılandırılmadı uyarısı, K3) |
| 7 | `--demo` kalıcı SİMÜLASYON bandı | Yapıldı (K29) |

### Dokunmatik ve görsel (6 madde, 4c)
| # | Madde | Durum |
|---|---|---|
| 1 | Alarm onayı ≥80 px | Yapıldı (şerit 88 px, buton 80 px) |
| 2 | Tester terminal/gönder 1080p'de kesilmesin; yardım satırları taşmasın | Yapıldı (ekran görüntüsü 04); PANELDE TEST 2 |
| 3 | Harita hücreleri <80 px → ayrıntı | Yapıldı: modül ayrıntısı penceresi (K28) |
| 4 | Uygulama içi sayısal tuş takımı | Yapıldı (`NumPad.qml`, K13) |
| 5 | Hareket bandı alarm kırmızısı değil | Yapıldı (nötr + amber) |
| 6 | Okunabilirlik/eldiven/parlama | PANELDE TEST 1, 6, 8 |
| — | Offscreen ekran görüntüleri | `docs/v2.2/ekran/01…09` (3D alan boş, beklenen) |

### Kabul testi matrisi (14 satır)
| Test | Durum (v2.2) |
|---|---|
| Yazma başarısız + eski S | Otomatik, YEŞİL (M01, `test_write_failure_is_not_accepted`) |
| HOME sırasında eski S300 | Otomatik, YEŞİL (M02) |
| Eksik/bozuk STAT, U2ERR | Otomatik, YEŞİL (M03, `PartialStatTest`) |
| HOME sırasında MOV/ALL/terminal | Otomatik, YEŞİL (M04, `TerminalAuthorityTest`) |
| İkinci BAŞLA | Backend YEŞİL (M05); fiziksel çift dokunma → PANELDE TEST 5 |
| F veya timeout sonrası | Otomatik, YEŞİL (M06, `AlarmLifecycleTest`) |
| Eski dispatch callback | Otomatik, YEŞİL (M07, `GenerationTest`) |
| Bozuk yeni STL | Otomatik, YEŞİL (M08, `AtomicLoadTest`) |
| USB kopma / yeniden bağlanma | Sahte köprü + gerçek SerialManager(loop://) YEŞİL (M09, `RealThreadPathTest`); gerçek USB → DONANIMDA TEST 9 |
| Slave reset / Pi restart | Otomatik YEŞİL (M10, `ResetDetectionTest`); Pi restart → DONANIMDA TEST |
| 16 modül + gecikme + canlı poll | Hat simülasyonu YEŞİL (M11, `AlarmLatencyTest`: 10,8 sn ölçüldü; en kötü 47,3 sn hesap) |
| Boş, metin, eksi, >590 hedef | Backend YEŞİL (M12, `test_protocol`); Qt giriş olayları (tuş takımı) → PANELDE TEST 3 |
| Uzun/bozuk seri akış + büyük STL | Tampon/kuyruk sınırları YEŞİL (M13, `test_bounded_queue`); uzun süreli Pi testi yapılmadı |
| Fiziksel durdurma ve enerji dönüşü | Yazılımla doğrulanamaz → GUVENLIK_ACIK_KONULAR §1, DONANIMDA TEST 10 |

### Demo yetersizliği
Demo artık gerçek protokol yolunu `SimBridge` ile çalıştırır (kuyruk, ACK, STAT, alarm, yetki) ve
kalıcı SİMÜLASYON bandı gösterir; matris demo ile değil `tests/` altındaki hata enjeksiyonlu
testlerle karşılanır. Sınırları README "`--demo` sınırları" bölümünde.
