# v2.2 bağımsız inceleme — düzeltme turu (2026-10-19)

> Kaynak: kullanıcının 2026-10-19 tarihli bağımsız inceleme bulguları (6 regresyon + QML/Türkçe +
> belge/repo denetimi + deploy script). Bu belge plan, kanıt ve teslim raporudur.
> **Testlerin yeşil olması makinenin güvenli olduğu anlamına gelmez** (§10, `GUVENLIK_ACIK_KONULAR.md` §11).

## 0. Başlangıç durumu (kayıt)

- İncelenen başlangıç: `ui_v2.2` @ `2c5cf2c7856b1a36fff5d8e985b48ecd9c857756`. Başlangıç testleri `102 OK`
  (bulgular test kapsamı dışındaydı).
- Çalışma dalı: `claude/ui-v2.2-bulgu-duzeltme` (`ui_v2.2`'den). **Push yok.** Donanıma, seri porta, SSH'a,
  Pi'ye bağlanılmadı; deploy/restart yapılmadı; firmware yürütme koduna dokunulmadı; `machine.json`
  değiştirilmedi.
- Başlangıç çalışma ağacı korundu, commit'e eklenmedi: ` D Adaptif_Kalip_blue_pill_rs485_test/Tester_UI/ui_v2.0.py`,
  takip edilmeyen `.claude/`, `AGENTS.md`, `Adaptif_Kalip_blue_pill_rs485_bridge_usart1/`,
  `Adaptif_kalip_v5f411_U2_5motor/`, `Adaptif_kalip_v5f411_U3_4motor/`, `PMS_GOREVLERI.md`,
  `RS485_DENETIM_NOTU.md`, `RS485_Modul_Tester_f411/`, `Tester_UI/`, `UI_v2.0_QML/docs/screenshots/`,
  `V5F411_NOTU.md`, `hakem_sunumu/`, `makale/`, `photos/`. Var olan worktree
  `.claude/worktrees/determined-nightingale` dokunulmadı.
- Takip edilmeyen dosyalarda yapılan (commit'e girmeyen) düzenlemeler §7'de ayrıca listelendi.

## 1. Plan (uygulandı)

1. Her regresyon için önce mevcut kodda başarısız olan davranış testi (QML için gerçek etkileşim);
   sonra düzeltme, aynı test yeşil. Assert zayıflatma / skip / expectedFailure yok.
2. Sıra: seri yönetici çekirdeği (5 → 2 → 4) → 3 → 6 → QML (1 + Türkçe + geometri) → deploy → belgeler.

## 2. Commit'ler

| Commit | Kapsam |
|---|---|
| `22fd9ee` | `fix(v2.2)`: link.py / hmi_config.py / controller.py — bulgu 2–6 + backend onay doğrulaması; `tests/test_review_regressions.py`, `tests/support.py` |
| `75d683c` | `fix(v2.2)`: QML onay bağlamı, Bakım motor paneli, sonuç kuyruğu; `tests/qmlui_*.py`, `test_qml_interaction.py`, `test_texts.py` |
| `0d82c6b` | `fix(deploy)`: `deploy_pi.sh` + `tests/test_deploy_script.py` |
| `558dd82` | `chore(repo)`: Web_UI donanım araçları içe aktarmada çalışmaz; F401 `platformio.ini` F411 yönlendirmesi; `tests/test_repo_safety.py` |
| (sonraki) | `docs`: bu rapor, KARARLAR K5/K11/K27 + K30–K35, GÜVENLİK §5/§9–§11, PROTOKOL_ONERI §2/§4, README'ler, CLAUDE.md, REPO_HARITASI, ARSIVLEME_ONERISI, ekran görüntüleri |

## 3. Regresyonlar — kanıt

Kırmızı kanıt iki yoldan alındı: (a) düzeltme yapılmadan önce yeni testler çalıştırıldı (yalnız test
desteği ve davranışsız `objectName` eklenmişti); (b) yeni backend + deploy testleri `git archive 2c5cf2c`
ile çıkarılan **değiştirilmemiş** ağaçta çalıştırıldı: 25 testten **20 başarısız + 1 hata**; orijinalde
de yeşil olan 4 test koruma amaçlıdır (yeniden başlatma/null yapılandırma, tam modül referansıyla
hareket, 2 saatlik uzun koşu, alarm dalgalanması) ve bu raporda "düzeltildi" sayılmadı.

### Bulgu 1 — QML onay penceresinde yanlış modüle komut

- **Dosya/satır (2c5cf2c):** `qml/TesterPage.qml:27` (`NumPad.onAccepted: app.moveMotor(page.sid, …)`),
  `:54–57` (`confirm.onAccepted` canlı `page.sid`), `qml/MainPage.qml:65–69` (`startSendSelected(app.currentSlaveId)`,
  `startProduction()` canlı liste). Pencereler yalnız kendi sayfasını örtüyordu; sekme çubuğu ve başlık
  (Modülleri bul) dokunulabilir kalıyordu.
- **Tetikleme (gerçek QML, QTest fare olayları, 1920×1080):** Bakım → Motor 3 hedef paneli (Modül 1) → 100 →
  STL sekmesi → haritada Modül 2 → "Hedef modül yap" → Bakım → kabul.
- **Önce:** başlık "Modül 1 / Motor 3" iken `MOV:02:03:100` gönderildi. Aynı açıdan: motor referans
  onayı → `HOME:02:03`; "Seçili modüle uygula" (mesaj "Modül: 1") → `ARR:02:…`; BAŞLA onayı "1, 2" listelerken
  yeniden tarama Modül 3'ü bulunca `HOME:01, HOME:02, HOME:03`.
- **Sonra:** panel/onay açılıştaki modül, motor, modül listesi, model ve bağlantı oturumunu saklar
  (`TesterPage.qml` `openMotor`/`confirm.ask`, `MainPage.qml` `confirmDialog.ask`); bağlam değişirse
  (`closeDialogs`, `recheck`, sayfa gizlenince) pencere **kapanır**, komut gitmez, günlüğe
  "ARAYÜZ: … kapatıldı (komut gönderilmedi)" yazılır. Kabul anında yeniden denetlenir; backend
  `startProductionConfirmed(liste, modelId)` / `startHomeAllConfirmed(liste)` /
  `startSendSelectedConfirmed(modül, modelId)` farklı listeyi/modeli reddeder (K32).
- **Test:** `qmlui_tests.DialogContextTest` (4 test) — önce 4/4 kırmızı (`MOV:02:03:100`, `HOME:02:03`,
  `ARR:02…`, `HOME:01–03`), sonra yeşil.

### Bulgu 2 — STAT zaman aşımından sonra eski yanıtla yanlış HOME başarısı

- **Dosya/satır (2c5cf2c):** `backend/controller.py:584–593` `_note_no_reply` yalnız sayaç artırıyordu;
  `:647–657` `_on_stat_done` zaman aşımında modülü "senkron dışı" yapmıyordu; `:1255–1284` `_op_on_stat`
  senkron durumuna bakmıyordu; `backend/link.py:121–133` eşleşme yalnız fiil+modül. K11 belgede vardı,
  kodda STAT için yoktu.
- **Tetikleme:** referanslı modülde (300 mm) yeni `HOME:01`, ACK sonrası STAT cevapları düşürülür →
  zaman aşımı; zaman aşımından sonraki ilk komuta gelen satır HOME öncesi dönemin `STAT:01:S0×9` cevabıdır
  (firmware cevaptan sonra RS485 girişini boşaltır → sonraki komut kaybolur; fiziksel olarak tutarlı).
- **Önce:** simülatörde motorlar `HHHHHHHHH` iken "Referans alma tamamlandı (firmware bildirimi)".
- **Sonra (`controller.py` `_note_misaligned` 682, `_aligned` 699, `_poll_next` 734, `_on_fence_done` 762,
  `_stat_unverified` 832, `_motion_issue` 1100):** zaman aşımı / yazma hatası / yazma takılması / modül
  kimlikli istenmemiş cevap → modülün **cevap sırası belirsiz**; sonraki cevaplar veri değildir (yalnız F
  alarmı üretebilir); gözetim `PING` gönderir; `PONG`'dan sonra istenen STAT kullanılır; bu sürede hareket
  kilitli ("cevap sırası yeniden doğrulanıyor"). Günlük izi: eski satır PING beklenirken "beklenmeyen cevap"
  olarak düştü → PING zaman aşımı → PING/PONG → gerçek `H277…` okundu → gerçek bitişte başarı.
- **Kesin çözülemeyen sınır:** protokolde istek kimliği yok. Çit, firmware'in komutları sırayla işleyip
  cevapladığı varsayımına dayanır (kaynaktan okundu, ölçülmedi); çitten sonra aynı biçimde gelen yabancı
  bir satır ayırt edilemez. İki ardışık benzer cevap **kesin tazelik kanıtı olarak sunulmadı**: SYNC
  yardım metni ve K11 bunu açıkça söyler; "doğrulandı" kullanılmaz. K11 belge ↔ kod uyumu sağlandı.
- **Test:** `StaleReplyAfterTimeoutTest` (3 test) — orijinalde 3/3 kırmızı, sonra yeşil.
  Bilinen sınır testi (`test_link.test_residual_risk_late_reply_matches_next_same_request`) korunur.

### Bulgu 3 — Yeniden bağlanmada kayıp modül unutuluyor

- **Dosya/satır (2c5cf2c):** `controller.py:465, 501` bağlan/kes `self._mods = {}`; kapsam
  `_get_active` (298–300) yalnız bu bağlantının cevap verenleri; `_block_reason` null yapılandırmada
  beklenen listeyle karşılaştıramıyordu.
- **Tetikleme:** Modül 1 ve 2 keşfedilip referanslanır; Modül 2 susturulur; COMM alarmı oluşmadan
  bağlantı kesilip açılır, yeniden keşif.
- **Önce:** yalnız Modül 1 bulundu; `productionBlockReason == ""`, BAŞLA ve Referans al Modül 1'e
  daraltılmış kapsamla serbest.
- **Sonra:** `self._session_seen` (oturumda geçerli cevap veren modüller) bağlantıdan bağımsız; `_lost()`
  (363) = görülmüş ama şimdi erişilemeyen; tarama sonunda COMM alarmı (633); `_block_reason` (1068)
  bütün hareketi kilitler ("Bu oturumda görülen Modül 2 şimdi cevap vermiyor; kapsam daraltılmaz…");
  gözetim kayıp modülü PING ile yoklamaya devam eder, dönünce kayıt kapanır. Canlı keşif (`activeSlaves`),
  oturum hafızası (`seenModules`, `lostModules`) ve beklenen liste (`expectedModules`) ayrı; Ana ekranda
  "CEVAP VERMEYEN (bu oturumda görülmüş)" satırı.
- **Yeniden başlatma / eksik yapılandırma:** hafıza diske yazılmaz; yeniden başlatınca sıfırlanır (K30).
  Null yapılandırmada arayüz modül listesi uydurmaz (`expectedModules` boş, plan `expectedConfigured=false`,
  şerit "Beklenen modül listesi yapılandırılmadı", BAŞLA onayı bulunan modülleri listeler) — bu, yeniden
  başlatmadan sonra eksik modülün **fark edilemeyeceği** anlamına gelir (GÜVENLİK §10).
- **Test:** `LostModuleAcrossReconnectTest` — 2 test orijinalde kırmızı, 1 koruma testi (null
  yapılandırmada uydurma yok) orijinalde de yeşil.

### Bulgu 4 — Terminal sorguları arıza izlemesini geciktiriyor

- **Dosya/satır (2c5cf2c):** `controller.py:618–631` `_poll_next` yalnız `link.idle()` iken; tarama ve
  dağıtım sırasında hiç; terminal sorgularında sınır yok (`sendRaw` 1058–1065).
- **Tetikleme:** tek modülde 30 cevapsız `GETPOS`, ardından motor F.
- **Önce:** F görünme süresi 1 modül **84,8 sn** (incelemedeki değer birebir), 4 modül 85,2 sn, 16 modül 97,8 sn.
- **Sonra:** gözetim isteği kuyruğun önüne girer (`link.submit(front=True)`), en fazla 1 bekler, tarama
  ve hareket dağıtımı sırasında da sürer; aynı modülün bekleyen operatör STAT'ı öne alınır; operatör
  sorguları ≤ 4 (`USER_QUERY_MAX`), aynı sorgu ikinci kez kuyruğa girmez (`_user_query_block` 780) (K34).
- **Ölçülen (sahte saat, 5 farklı arıza zamanı, 30 cevapsız sorgu):** 1 modül **2,9 sn** (sınır 5,32),
  4 modül 15,2 sn (22,24), 16 modül 34,8 sn (89,92). Mevcut 16 modül testi: 10,8 → 6,8 sn.
- **Belgelenen sınırlar ve varsayımları (`hmi_config.py`):**
  - `worst_alarm_latency_s(N)` = N×2,8 + 2,5 sn — **yalnız gözetim trafiği** (5,3 / 13,7 / 47,3 sn).
  - `worst_alarm_latency_loaded_s(N)` = (N−1)×2×2,82 + 2,82 + 2,5 sn — başka trafik varken
    (5,32 / 22,24 / 89,92 sn). Varsayımlar: gözetim isteği önde ve tek; yazılmış istek kesilemez, en fazla
    bir istek beklenir; modülün cevap sırası belirsiz değil (değilse +1 tur PING); her istek zaman aşımına
    uğrar (en kötü); seri yazma takılmaz (takılırsa bağlantı düşer). 16 modülde yük sınırı veri yaşı eşiğini
    (89,6 sn) aşar: o durumda şerit önce "Güncel makine durumu bilinmiyor" der ve hareket kilitlenir.
  - Gerçek RS-485 hattında ölçülmedi.
- **Test:** `SupervisionUnderQueryLoadTest` — 1/4/16 modül, dolu kuyruk (30 deneme → 4 kabul), cevapsız
  sorgular, çift dokunma (orijinalde 4/4 kırmızı), 2 saatlik sahte saat (kuyruk ≤ 8, zamanlayıcı ≤ 20,
  en eski veri ≤ eşik, sahte alarm yok — koruma testi).

### Bulgu 5 — Yazma tamamlanması gelmezse işlem sonsuza kadar bekliyor

- **Dosya/satır (2c5cf2c):** `backend/link.py:176–189` cevap zaman aşımı `_write_done` içinde
  başlıyordu; `_pump` (161–174) yazma için süre koymuyordu.
- **Tetikleme:** yazma callback'ini hiç çağırmayan sahte taşıyıcı (`HoldingBridge`).
- **Önce:** 100 sn sonra istek `writing`, işlem sürüyor, hat kilitli (gözetim de durmuş).
- **Sonra (`link.py` `_pump` 202–220, `_write_stalled` 238):** yazma `WRITE_DONE_TIMEOUT_MS` = 1,5 sn
  içinde bitmezse istek `write_stalled`, işlem "Yazma tamamlanmadı" ile biter, kalan modüllere komut yok,
  **komut tekrar edilmez**, `controller._on_link_stall` (593) bağlantıyı düşürür (LINK + SYNC). Gecikmiş
  callback ve eski oturum callback'leri (nesil + istek + durum kontrolü) yok sayılır.
- **Test:** `WriteCompletionSupervisionTest` (3 test: takılma, hareket sırasında takılma + gecikmiş
  callback + tekrar yok, yeniden bağlanma sonrası eski callback) — orijinalde 3/3 kırmızı. Gerçek thread'li
  sahte seri yol: mevcut `RealThreadPathTest`/`SerialManagerLoopTest` (`loop://`) yeşil.

### Bulgu 6 — Tek motor HOME sonrası aynı motor hareketi

- **Dosya/satır (2c5cf2c):** `controller.py:886–889` hareket için `m.homed` (modül) gerekir; tek motor
  HOME (HOMEM) hiçbir referans durumu tutmuyordu; ret metni "Modül X referanssız. Önce referans alın."
- **Karar (K31, ONAY BEKLİYOR):** modül kilidi **korundu** (K8): tek parça membran; diğer 8 motorun
  konumu bilinmeden tek motor hareketi önerilmez. Motor bazlı izin ayrı karar olarak bırakıldı.
- **Sonra:** tek motor referansı motor bazında izlenir (`moduleInfo().motors[i].referenced`, hücrede
  "· motor referanslı"); HOMEM başarı mesajı "Bu, modül referansını tamamlamaz: …"; ret metni
  "Modül 1: yalnız motor 3 referanslı; tek motor referansı yeterli değil. Hareket için önce modülün
  tamamına referans alın ('Modül 1'e referans al')."; tek motor referans onayında da aynı not.
- **Test:** `SingleMotorReferencePolicyTest` — açıklama testi orijinalde kırmızı; "tam modül referansıyla
  hareket serbest" koruma testi orijinalde de yeşil.

## 4. QML ve Türkçe

### Bakım ekranı geometrisi (1920×1080)

- **Önce:** `docs/v2.2/ekran/04_bakim_motor_testi.png` — SİMÜLASYON bandıyla motor hücresi 104 px;
  "Hedef gir"/"Ref." butonları hücre dışına taşıyor, Kılavuz kesik. Geometri testi orijinalde kırmızı
  (`motorTarget_1 (708,529,125,64) / hücre (450,483,472,104)`).
- **Sonra (K33):** hücre butonsuz, tek dokunma hedefi (≥ 80 px; ölçülen 123–138 px); dokununca **motor
  paneli** (tuş takımı + "Motor X'e referans al" + kapalı eylemlerin tam sebebi). Kılavuz kısaldı; modül
  karosu etiketleri iki satıra sığıyor. Test, Pi yazı tipi farkı için hücre ve kılavuzda **%12 boş yer**
  ister (`TesterGeometryTest`, SİMÜLASYON bandı dahil/değil). Ekran görüntüleri: `docs/v2.2/ekran_duzeltme/`
  (`04_bakim_motor_testi.png`, `05_motor_paneli.png` …).
- **Bilinen fark:** offscreen Mac yazı tipi (Helvetica) ≠ Pi (DejaVu Sans); panelde doğrulanmalı (§9).

### Metin değişiklikleri — mevcut → sorun → öneri → gereken davranış

| # | Mevcut | Sorun | Öneri (uygulanan) | Gereken davranış |
|---|---|---|---|---|
| 1 | Motor hücresinde "Ref." | Kısaltma/jargon; hücreden taşıyor | Motor panelinde "Motor X'e referans al" | Eylem adı tam; onay penceresi açılır |
| 2 | Modül karosu "ref. yok" | Kısaltma | "referans yok" (iki satır) | Durum kısaltmasız okunur |
| 3 | "Modül X referanssız. Önce referans alın." (tek motor referansından sonra da) | Neden reddedildiği belirsiz | "Modül 1: yalnız motor 3 referanslı; tek motor referansı yeterli değil. Hareket için önce modülün tamamına referans alın ('Modül 1'e referans al')." | Politika + sonraki adım |
| 4 | "Modül 1 / Motor 3: referans alma tamamlandı (firmware bildirimi)." | Modülün hâlâ referanssız olduğu söylenmiyor | + "Bu, modül referansını tamamlamaz: tek motor hareketi için modülün tamamına referans alınmalıdır…" | Başarı mesajı kapsamı söyler |
| 5 | "Modül 1: ARRERR:01. Neden: …" | Ham protokol kodu | "Modül 1: şekil (9 ayrı hedef) komutunu modülün ikinci kartı (U2) onaylamadı." + Etki + Adım | Ne oldu / etki / adım; kod yok (`OperatorMessageTest`) |
| 6 | SYNC nedeni "Modül 1: ARRERR — U2 komutu onaylamadı" | Ham kod | "…komutunu ikinci kart (U2) onaylamadı; hareketin başlamadığı varsayılamaz" | Alarm listesinde kod yok |
| 7 | "Seri porta yazılamadı (ARR:01:300:…)." | Ham komut | "Modül 1: seri porta yazılamadı." + etki + adım | — |
| 8 | "U2ERR biçimi yapılandırılan sürümle çelişiyor" | Ham kod | "U2 hata bildiriminin biçimi yapılandırılan firmware sürümüyle çelişiyor" | — |
| 9 | Bakımda her zaman "Ölçülen" | Son bilinen ile güncel doğrulanmış karışıyor | Güncel değilse "Son bilinen" (amber), başlıkta "Son bilinen durum: N sn önce — güncel değil"; Modül ayrıntısında aynı | `moduleInfo().current` = bağlı + erişilebilir + cevap sırası kurulu + tam + taze |
| 10 | SYNC adımı "iki ardışık sorguda tutarlı … kendiliğinden kapanır" | İki benzer cevap tazelik kanıtı gibi | "…haberleşme sırası yeniden kurulduktan sonra… Bu, cevapların tazeliğinin kesin kanıtı değildir." | Kanıtsız iddia yok |
| 11 | COMM yardımı | Kayıp modül ve kapsam daraltma söylenmiyor | "…bu oturumda görülmüş olduğu hâlde taramada bulunamadı … Kapsam bulunan modüllere sessizce daraltılmaz … machine.json güncellenmeli" | — |
| 12 | (yoktu) kayıp modülde ret | Sessiz kapsam daralması | "Bu oturumda görülen Modül 2 şimdi cevap vermiyor; kapsam daraltılmaz, hareket kilitli. Modül bilerek çıkarıldıysa uygulamayı yeniden başlatın." | Bütün hareket kilitli |
| 13 | (yoktu) zaman aşımı sonrası | Eski veri güncel sanılıyor | "Modül X: zaman aşımı/beklenmeyen cevaptan sonra cevap sırası yeniden doğrulanıyor; güncel durum bilinmiyor." | Hareket kilitli |
| 14 | "Seri kuyruk dolu." (terminal) | Neden/adım yok | "Bekleyen sorgu sınırı (4) dolu; önceki sorgular bitince tekrar deneyin. (Makine gözetimi bundan etkilenmez.)" / "Aynı sorgu zaten sırada: …" | Çift dokunma tek sorgu |
| 15 | (yoktu) onay bağlamı değişti | Yanlış modüle komut | Pencere kapanır; backend reddi: "Onaydan sonra modül listesi (1, 2 → 1, 2, 3) değişti; işlem başlatılmadı. Adım: …" | Komut yok |
| 16 | (yoktu) yazma takıldı | Sonsuz bekleme | "Yazma tamamlanmadı" — etki (ulaşıp ulaşmadığı belirsiz, hareket başlamış olabilir, tekrar yok, bağlantı kesildi) + adım | Bağlantı düşer |
| 17 | Kapalı "Hedef gir" butonunda sebep yok | Devre dışı neden görünmüyor | Motor panelinde buton altında kısa sebep + "Hareket kapalı: <tam sebep>" | `disabledReason` + tam metin (`test_disabled_motor_buttons_show_reason`) |
| 18 | Tuş takımı "Hedef girin" | Eylem değil durum | "Önce hedef girin" | — |
| 19 | Sonuç penceresi her mesajda yeniden açılıyor; ARIZA penceresi sonraki "Tarama başlatılmadı" ile eziliyordu | Pencere yağmuru / arıza gizleniyor | Tek pencere + kısa kuyruk: aynı mesaj tekrar açılmaz, ARIZA ezilmez ve öne geçer | `test_result_dialogs_do_not_storm_or_hide_fault` (orijinalde kırmızı) |

Değişmeyen ilkeler: "Bu arayüzden hareket durdurulamıyor." (talimat/anahtar adı yok), "Referans alma
tamamlandı (firmware bildirimi)", "doğrulandı/durdu" yok, "güvenli" yalnız olumsuz (`test_texts`).
Alarm onayı arızayı silmez (gerçek QML dokunuşu ile: `test_alarm_ack_in_strip_does_not_clear_fault`);
dalgalanan arıza tek kayıt, işlem yokken pencere yok (`FlappingAlarmTest`, koruma).

## 5. Deploy script

- **Önce (`deploy_pi.sh:21–23`):** `… main.py 2>&1 | grep … | head -40 || true` uzak komutun içinde →
  QML hatası olsa da uzak komut 0 döner, restart yapılır. Sahte ssh (uzak komut yerel kabukta, sahte
  python3 hata veriyor) ile eski script: "UYARI: …TypeError" yazdı, **kiosk restart çalıştı**, "Deploy tamam.", çıkış 0.
- **Sonra:** `DEPLOY_BRANCH` zorunlu (Pi farklı daldaysa durur, dal değiştirmez); izlenen dosyalarda yerel
  değişiklik → durur; `fetch` + `merge --ff-only FETCH_HEAD`; Pi HEAD ≠ beklenen (`DEPLOY_COMMIT` veya
  yerel `origin/<dal>`) → durur; QML kontrolü `tests/qml_load_check.py` çıkış koduyla → başarısızsa
  **restart yok**; uzak komuta giren değerler doğrulanır; logda Traceback → çıkış 3.
- **Test:** `test_deploy_script.py` (7 test) — orijinal script ile 6 başarısız + 1 hata; yenisiyle yeşil.
  Gerçek deploy çalıştırılmadı.

## 6. Belge ve repo denetimi

- `PROTOKOL_ONERI_v5.2.md` §4: "hep ya da hiç" iddiası düzeltildi (U2 komutu alıp ACK'i kaybolabilir →
  yarım hareket yine mümkün); öneri (a)/(b)/(c) ile **uygulanmış** arayüz davranışı ayrıldı. §2'ye
  uygulanan PING/PONG azaltması ve sınırı eklendi.
- `KARARLAR.md`: K5 (yük altında sınırlar), K11 (kod uyumu), K27 → K33; yeni K30–K35.
- `GUVENLIK_ACIK_KONULAR.md`: §5 yeniden yazıldı; §9 yazma takılması, §10 kayıp modül/yeniden başlatma,
  §11 testlerin söyledikleri/söylemedikleri.
- Kök `README.md`: "F411 için yalnız board değiştir, kod aynı" kaldırıldı → v5f411 projeleri; topoloji
  F401/F411 motor numaraları; UI dalları; kör test keşfi uyarısı; depo yapısı → `REPO_HARITASI.md`.
- `CLAUDE.md`: çekirdek kurallar (cevap sırası, yazma takılması, gözetim önceliği, oturum hafızası, onay
  bağlamı, tek motor referansı), STAT poll bölümü, deploy, topoloji; mevcut güvenlik kuralları korundu.
- `REPO_HARITASI.md` (yeni): her klasörün amacı, kart/firmware, giriş noktası, durum, Git takibi, test türü;
  takip edilmeyen önemli dosyalar listesi; Tester_UI taşıma doğrulaması; donanıma dokunan araçlar.
- `ARSIVLEME_ONERISI.md` (yeni): yalnız öneri; worktree ve donanım varyantları gereksiz sayılmadı.
- `Web_UI/test_mov.py`, `test.master.py`: içe aktarılınca COM3'e PING/MOV yazıyordu → `main()` +
  `--donanim`; `test_repo_safety.py` (AST) bunu ve paketin gerçek port açmadığını denetler.
- Tester_UI: eski izlenen yol silinmiş, kök kopya takip edilmiyor; **birebir aynı** (SHA-256
  `7cea21fb…817aa`, 18 950 bayt). Taşıma yarım; dokunulmadı (`REPO_HARITASI.md`).

## 7. Takip edilmeyen dosyalarda yapılan düzenlemeler (commit'e EKLENMEDİ)

| Dosya | Değişiklik |
|---|---|
| `Adaptif_kalip_v5f411_U2_5motor/platformio.ini` | Başlık yorumu "motor 1-5" → "motor 5-9" (+ soket sırası notu). Yalnız yorum. |
| `Adaptif_kalip_v5f411_U3_4motor/platformio.ini` | Başlık yorumu "motor 6-9" → "motor 1-4" (+ not). Yalnız yorum. |
| `V5F411_NOTU.md` | Baştaki tabloda motor aralıkları (U3 1-4, U2 5-9) — aynı belgenin §4'üyle çelişiyordu. |
| `AGENTS.md` | Güncel `CLAUDE.md`'den yeniden üretildi; eski kopyadaki `.Codex/` yol hataları giderildi; eski kopyada olup yenide olmayan satırların hepsi eskimiş sürümlerdi (kural kaybı yok). |

`configparser` ile dört `platformio.ini` dosyasının ayrıştığı doğrulandı (PlatformIO çağrısı izin
verilmediği için çalıştırılmadı; derleme yapılmadı). Orijinal kopyalar oturum scratchpad'inde.

## 8. Test sonuçları (donanımsız)

```
cd UI_v2.0_QML && QT_QPA_PLATFORM=offscreen venv/bin/python3 -m unittest discover -s tests -v
Ran 130 tests … OK          (başlangıç: 102)
  └ test_qml_interaction → qmlui_tests (ayrı süreç): Ran 11 tests … OK
```

Dahil: gerçek thread'li sahte seri (`SerialManager` + `loop://` + `QtTimerService`), gerçek QML etkileşimi
(QTest fare olayları, 1920×1080, SİMÜLASYON bandı dahil/değil), QML yükleme, sahte ssh deploy, depo
güvenliği. Repo genelinde kör test keşfi yapılmadı.

## 9. Kapsam dışı kalan protokol / donanım sınırları

1. **Fiziksel/yazılımsal durdurma yok:** protokolde STOP yok, E-stop doğrulanmadı; arayüz yalnız "Bu
   arayüzden hareket durdurulamıyor." der (GÜVENLİK §1).
2. **İstek kimliği yok:** PING/PONG çiti bir çıkarımdır; kesin tazelik için istek kimliği gerekir (PROTOKOL §2).
3. **U2 hatasında yarım modül hareketi:** firmware U2 onayı olmadan U3 motorlarını zamanlar; arayüz
   önlemez, görünür kılar ve kilitler (PROTOKOL §4).
4. **HOME referansı bağımsız doğrulanmaz** (encoder durması = "oturdu"; GÜVENLİK §3).
5. **Gecikme sınırları hesap + simülasyon;** gerçek RS-485 zamanlaması, U2 bekleme süreleri, USB askıda
   kalma ölçülmedi.
6. **Kayıp modül hafızası yeniden başlatmada kaybolur** (K30); `machine.json` boş.
7. **Firmware sıralı işleme ve RS485 girişini boşaltma** varsayımı kaynaktan okundu, ölçülmedi.

## 10. Demo kabul testi (kullanıcı; donanımsız `--demo`)

`cd UI_v2.0_QML && venv/bin/python3 main.py --demo` (SİMÜLASYON bandı görünmeli; simülatör ~60 mm/sn):

1. Bağlan → Modülleri bul → 4 modül; şerit "Referans alınmadı…"; Bakım'da karolar "referans yok".
2. Bakım → Motor 2 hücresine dokun → panel "Modül 1 / Motor 2" → "Motor 2'ye referans al" → onay →
   bitince mesajda "Bu, modül referansını tamamlamaz"; hücrede "· motor referanslı". Tekrar Motor 2 →
   100 → panelde "Hareket kapalı: … tek motor referansı yeterli değil"; götür butonu kapalı.
3. "Modül 1'e referans al" → onay → bitince Motor 2 → 100 → "Motor 2'yi 100 mm'ye götür" →
   günlükte `→ MOV:01:02:100`.
4. Motor 3 hücresine dokun → 1-0-0 gir → **STL Kontrol** sekmesine geç → haritada Modül 2'ye dokun →
   "Hedef modül yap" → Bakım'a dön: panel **kapalı**, günlükte "ARAYÜZ: Motor paneli kapatıldı: …
   (komut gönderilmedi)", `MOV` yok.
5. STL Kontrol → bir STL yükle → "Referans al" (bütün modüller) bitsin → Hedef modül 1 →
   "Seçili modüle uygula" onayı açıkken Bakım'da Modül 2 karosuna dokun → STL Kontrol'e dön: onay
   kapalı, `ARR` yok.
6. Terminalde aynı `GETPOS:01:3` iki kez hızla → ikincisi "Aynı sorgu zaten sırada"; art arda 5 farklı
   sorgu → 5.'si "Bekleyen sorgu sınırı (4) dolu…".
7. "Bağlantıyı kes" → Bakım'da etiket "Son bilinen", değerler "—" (bağlantı yokken ölçüm iddiası yok) →
   "Bağlan" → "Modülleri bul" → durum okununca "Ölçülen"; şerit yine "Referans alınmadı" (referans düştü,
   hareket kendiliğinden sürdürülmedi).
8. Bakım ekranında hiçbir buton/metin hücre veya panel dışına taşmıyor; Kılavuz tam okunuyor.
9. Arıza/pencere kuyruğu görsel turu (donanımsız, ekran görüntüsü):
   `QT_QPA_PLATFORM=offscreen venv/bin/python3 tests/qml_load_check.py --shots /tmp/tur` → `07_ariza_sonuc`,
   `09_bakim_ariza_sonrasi`: ARIZA penceresi sonraki mesajla ezilmiyor.

## 11. Fiziksel panelde doğrulanacaklar (Pi, 15,6" 1080p dokunmatik)

1. Bakım ekranı DejaVu Sans ile: motor hücreleri, modül karosu "referans yok" iki satır, Kılavuz,
   "Son bilinen" etiketi taşmıyor (offscreen Mac ölçümü %12 pay bıraktı).
2. Motor hücresine eldivenle tek dokunuşta panel açılıyor; panel 1080p'ye sığıyor; tuşlar ≥ 80 px.
3. Sekme değişince panel/onay kapanıyor; günlükte "ARAYÜZ: … kapatıldı" satırı görünüyor.
4. Sonuç penceresi kuyruğu: art arda gelen mesajlarda dokunmatikte "Kapat" akışı anlaşılır mı.
5. "Son bilinen" amber etiketi parlamada okunuyor mu.
6. Ana ekran "CEVAP VERMEYEN (bu oturumda görülmüş)" satırı uzun listede kesiliyorsa tam metin
   durum şeridinde/alarm listesinde var mı.
7. Önceki PANELDE TEST listesi (`ILERLEME.md` §6) geçerliliğini korur.

## 12. Gerçek donanım öncesi açık kararlar

1. **K30** kayıp modül hafızası kalıcı olsun mu (yeniden başlatmada da fark edilsin mi)?
2. **K31** tek motor referansıyla motor bazlı hareket izni (önerilmez; membran sınırı bilinmiyor).
3. **K32–K35** onay bağlamı, motor paneli, gözetim önceliği/sorgu sınırı (4), yazma takılmasında
   bağlantı düşürme (1,5 sn) — değerler donanımda ölçülmeli.
4. **K3/K4** `config/machine.json` `expected_modules` ve `firmware_variant` — gerçek kurulumdan
   kullanıcı yazmalı (arayüz uydurmaz; boşken yeniden başlatmada eksik modül fark edilemez).
5. Fiziksel durdurma / enerji ayırma düzeni ve prosedürü (yetkin değerlendirme; GÜVENLİK §1–§2).
6. Firmware: istek kimliği, U2 hatasında davranış (PROTOKOL §2, §4) — atomiklik garanti edilemez.
7. Sahadaki RS-485 köprüsünün hangi sürüm olduğu (USART2 v4.x mi, USART1 v4.2 mi) ve F411
   klasörlerinin Git'e eklenmesi (`REPO_HARITASI.md`).
8. `deploy_pi.sh` ilk kullanımda `DEPLOY_BRANCH=ui_v2.0` ve beklenen commit elle verilmeli; kiosk'un
   `ui_v2.2`'ye ne zaman alınacağı kullanıcı kararı.
