# v2.2 Görev Tanımı — güvenilir durum, tek hareket yetkisi, dürüst operatör dili

> **GECE MODU:** Bu görev kullanıcı uyurken, **gözetimsiz** çalıştırılacak.
> Kimse soru cevaplamayacak, izin vermeyecek, plan onaylamayacak.
> Bu dosyadaki her "karar" için varsayılan değer verilmiştir; onu kullan ve
> `KARARLAR.md`'ye yaz. Kullanıcı sabah okuyup onaylayacak.

Kanıtlar ve satır referansları: **`INCELEME.md`** (bağımsız inceleme raporu, tam metin).
İlerleme ve devam noktası: **`ILERLEME.md`**. Kararlar: **`KARARLAR.md`**.
Hepsi bu klasörde: `UI_v2.0_QML/docs/v2.2/`.

---

## A. GECE MODU KURALLARI — bunlar CLAUDE.md'deki "önce planı göster / onay bekle"
## kuralının yerine geçer (kullanıcı bu çalıştırma için önceden izin verdi)

1. **Soru sorma, onay bekleme, turu bitirme.** Fazlar arasında durup rapor verme;
   bir faz bitince doğrudan sonrakine geç. Turu yalnız (a) bütün fazlar bittiğinde
   veya (b) aşağıdaki durma koşullarından biri oluştuğunda bitir.
2. **Plan modu kullanma.** Planı `ILERLEME.md`'ye yaz, sonra uygula.
3. **Karar gerekirse:** `KARARLAR.md`'deki varsayılanı kullan. Orada olmayan bir
   karar çıkarsa en **muhafazakâr** (hareketi daha az mümkün kılan, daha az iddia
   eden) seçeneği seç, `KARARLAR.md`'ye `ONAY BEKLİYOR` etiketiyle ekle, devam et.
4. **Bir işlem engellenirse (izin/sınıflandırıcı/deny) TEKRAR DENEME.**
   Aynı veya benzer komutu yeniden çalıştırmak art arda engellemeye yol açar ve
   oturumu kullanıcı onayı bekler hale getirir — gece boyu kilitlenir. Engellenen
   işlemi `ILERLEME.md`'ye "ENGELLENDİ: <komut> — <sebep>" diye yaz, o işi atla,
   bağımsız bir sonraki işe geç.
5. **Hiç denemeyeceğin işlemler** (bunlar zaten yasak; denemek bile engelleme sayacını
   doldurur): `git push`, `ssh`, `scp`, `rsync`, `pip install`, `pio`/PlatformIO,
   ağdan indirme, `git add -A` / `git add .`, `git reset --hard`, `git clean`,
   `git checkout -- .`, `git restore .`, `git stash drop/clear`, `rm -rf`,
   `.claude/` altını düzenleme, firmware/legacy klasörlerini düzenleme,
   seri port açma, uygulamayı gerçek bir port'a bağlama.
6. **Başarısız denemeyi silme.** Bir alt değişiklik testleri kırıyor ve düzeltemiyorsan:
   `git stash push -m "v22-wip-<konu>" -- <dosyalar>` ile kenara koy, `ILERLEME.md`'ye yaz,
   devam et. Kırık kodu commit'leme.
7. **Her commit'te uygulama yüklenebilir olmalı** (QML yükleme testi geçer) ve
   o ana kadarki testler yeşil olmalı. Kullanıcı sabah hangi commit'te durursa
   dursun çalışan bir uygulama bulmalı.
8. **İlerleme dosyasını sık güncelle.** Her faz başında/sonunda ve her önemli adımda
   `ILERLEME.md`'yi güncelle ve commit'le. Bağlam sıkıştırılırsa veya oturum yeniden
   başlarsa: **önce `ILERLEME.md`, `KARARLAR.md`, bu dosya** — kaldığın yerden devam.
9. **Donanım yok.** Bütün doğrulama sahte seri köprüyle ve offscreen Qt ile yapılır.
   Donanım/panel gerektiren her şey "PANELDE TEST" / "DONANIMDA TEST GEREKLİ"
   listelerine gider; asla "test edildi" diye işaretlenmez.

### Durma koşulları
- Bütün fazlar bitti → sabah özetini yaz, son commit, dur.
- Aynı faz içinde 3 bağımsız alt iş art arda engellendi/başarısız oldu ve ilerlenebilecek
  bağımsız iş kalmadı → durumu yaz, dur.
- Çalışma ağacında beklemediğin, senin yapmadığın bir değişiklik görürsen (başka bir
  oturum/agent aynı repoda çalışıyor olabilir) → hiçbir şeye dokunma, yaz, dur.

---

## B. KAPSAM VE GENEL KURALLAR

Bu tur bir makyaj değil: backend'in hareket, durum ve seri haberleşme çekirdeğine dokunan
**davranış değişikliği**. Ana fikir: **arayüz bilmediği bir şeyi bildiğini iddia etmeyecek.**

`INCELEME.md`'deki **hiçbir madde atlanmayacak.** Her madde ya çözülecek ya da gerekçesiyle
"yapılmadı / yazılımla çözülemez" olarak `ILERLEME.md`'deki kapsama tablosuna yazılacak.

1. Yalnız `UI_v2.0_QML/` ve `CLAUDE.md` değişir. Firmware/protokol gerektiren maddeler
   için yalnız öneri dokümanı (Faz 5). v4.4, v5, v5f411, Calibration, Web_UI,
   RS485_Modul_Tester_f411 klasörlerine dokunma (okumak serbest).
2. **Sahte STOP / sahte E-stop yok.** Yazılımda "durdur" düğmesi uydurma, var olmayan
   şalter/anahtar adı yazma.
3. **Hareket komutlarında kör otomatik tekrar (retry) yok.** Zaman aşımında önce kabul
   durumu uzlaştırılır; tekrar gönderim yalnız operatörün açık kararıyla.
4. **Metin davranışı aşamaz.** Metni ancak anlattığı davranış gerçekten doğruysa değiştir.
   "Doğrulandı", "güvenli", "durdu" kelimelerini kanıt olmadan kullanma.
5. Kod yorumları ASCII; **operatöre görünen metinler doğru Türkçe karakterlerle**
   (ı ş ğ ü ö ç İ). Sabitler `Theme.js` / tek bir config'te merkezî.
6. Mevcut dokümanlardaki "doğrulandı" ifadelerini bağımsız kanıt sayma.

### Git
- Mevcut branch `ui_v2.0`'dan **`ui_v2.2`** aç (`git checkout -b ui_v2.2`). Push yok.
- Çalışma ağacı kirli; bunlara **dokunma, stage'leme, commit'leme**:
  - `Adaptif_Kalip_blue_pill_rs485_test/Tester_UI/ui_v2.0.py` silinmiş görünüyor (korumalı klasör)
  - untracked: `Adaptif_kalip_v5f411_*`, `Adaptif_Kalip_blue_pill_rs485_bridge_usart1/`,
    `RS485_Modul_Tester_f411/`, `Tester_UI/`, `AGENTS.md`, `PMS_GOREVLERI.md`,
    `RS485_DENETIM_NOTU.md`, `V5F411_NOTU.md`, `hakem_sunumu/`, `makale/`, `photos/`,
    `UI_v2.0_QML/docs/screenshots/`
- Yalnız ilgili yolları **tek tek** stage'le: `git add UI_v2.0_QML/backend/...`,
  `git add UI_v2.0_QML/qml/...`, `git add UI_v2.0_QML/tests/...`,
  `git add UI_v2.0_QML/docs/v2.2/...`, `git add CLAUDE.md` vb.
- **İlk commit:** bu klasördeki görev dosyaları (`GOREV.md`, `INCELEME.md`,
  `ILERLEME.md`, `KARARLAR.md`, `BASLAT.txt`) — ilerleme kalıcı olsun.

### Komutlar (macOS, yerel venv)
- Python: `UI_v2.0_QML/venv/bin/python3` (PySide6 6.11.1 kurulu). Yeni paket kurma.
- Testler: `cd UI_v2.0_QML && QT_QPA_PLATFORM=offscreen venv/bin/python3 -m unittest discover -s tests -v`
- macOS'ta GNU `timeout` olmayabilir. Uygulamayı süreli çalıştırmak yerine QML yükleme
  kontrolünü bir **test** olarak yaz (`tests/test_qml_load.py`: offscreen
  `QQmlApplicationEngine` ile `Main.qml` yükle, kısa süre event işle, `rootObjects()`
  boş değil ve QML uyarısı yok mu kontrol et). Quick3D offscreen çizilmez;
  "isApiRhiBased ... not functional" beklenen ve zararsızdır.

---

## C. YAZILIMLA ÇÖZÜLEMEYENLER — sınırı net tut

- **Fiziksel durdurma / enerji ayırma (Bulgu 2).** Host STOP/ABORT yok; `mStopAll()` iç
  fonksiyon; fiziksel E-stop yok; güç devresi incelenmedi.
  → HMI metni: **"Bu arayüzden hareket durdurulamıyor."** Sonrasına talimat yazma.
  → `docs/v2.2/GUVENLIK_ACIK_KONULAR.md`: yetkin donanım/makine güvenliği değerlendirmesi
  gerektiğini, doğrulanmamış iddiaların kaldırıldığını yaz. Gelecekteki yazılımsal
  durdurma asla "E-stop" diye adlandırılmayacak; cevap vermeyen aktüatör için "durdu" denmeyecek.
- **HOME referansının bağımsız doğrulanması (Bulgu 12).** "Referans doğrulandı" deme;
  "Referans alma tamamlandı (firmware bildirimi)" gibi kanıtın sınırını söyleyen metin.
- **İşlem kimliği (Bulgu 1'in protokol kısmı).** v2.2'de azalt, kalan riski dokümante et,
  protokol desteğini öner.
- **U3'ün U2 hatasında kendi motorlarını yine zamanlaması (Bulgu 6 eki).**
  `ALLERR/ARRERR/HOMEERR` = "belirsiz, izlemeye devam". Firmware tarafı öneriye.
- **Membran komşu yükseklik farkı sınırı (Bulgu 9 eki).** Değer bilinmiyor; yalnız raporla.

---

## D. FAZLAR (sırayla; her faz sonunda: testler yeşil + QML yükleme testi + commit +
## ILERLEME.md güncelle → SONRAKİ FAZA GEÇ)

Zaman yetmezse öncelik sırası budur; P0'lar (Faz 0-1) en değerli kısım.

### Faz 0 — Test altyapısı ve kırmızı testler
- `UI_v2.0_QML/tests/` — stdlib `unittest` + `QCoreApplication`.
- `Controller`'a seri katman **enjekte edilebilsin** (şu an `SerialManager`'ı içeride kuruyor).
  Varsayılan davranış değişmesin.
- **Hata enjeksiyonlu sahte köprü**: `write_line=False`, gecikmeli cevap, kayıp ACK, bozuk
  satır, kısmi STAT (`U2ERR`), eski/sıra dışı cevap, slave reset (`I0`), kopma.
- Zaman/timer kontrol edilebilir (enjekte edilebilir saat/zamanlayıcı) — dağıtım callback
  sırası ve timeout'lar deterministik.
- `INCELEME.md`'deki **kabul testi matrisinin** otomatikleştirilebilen her satırı bir test.
  Önce **kırmızı** olduklarını göster (sonucu `ILERLEME.md`'ye yaz), sonra düzeltmelerle yeşile çevir.
  Kırmızı testleri commit'lemek için `@unittest.expectedFailure` kullan ve ilgili faz
  düzeltince kaldır (kural A7: commit'lerde test paketi yeşil kalsın).
- `tests/test_qml_load.py` (bkz. B/Komutlar).
- Demo modu matrisin yerine geçmez.

### Faz 1 — P0: hareket yetkisi, katı tamamlanma, arızada gözetim
**1a. Tek hareket yetkilendirme noktası (Bulgu 3).** Backend'de tek fonksiyon
(ör. `_authorize_motion(kind, scope)`); **bütün** hareket yolları buradan geçer: üretim
(`startProduction/startHomeAll/startSendSelected`), Tester butonları ("Test 300 mm",
"Tümünü Sıfırla", "Home Hepsi", tek motor Home), sayı alanında Enter, doğrudan
`moveMotor/homeMotor/allToValue/sendArrayToSlave/homeSlave`, ve **terminal**.
Aktif işlem / belirsiz durum / aktif arıza / geçersiz referans varken çelişkili hareket
reddedilir; sebep operatöre gösterilir. Terminal: fiil ayrıştırılır; `ARR/ALL/MOV/HOME`
aynı yetkiden geçer; `SETID` kilitli (KARARLAR); `PING/STAT/GETPOS/GETID` serbest;
tanınmayan fiil reddedilir.

**1b. Komut kabulü ve sonuç ayrımı (Bulgu 1).**
- `_send()` yazma sonucunu döndürsün; başarısızsa işlem "kabul edilmedi", **asla başarı yok**.
- ACK/ERR (`ARROK/HOMEOK/ALLOK/MOVOK`, `*ERR`) işlem durumuna işlensin. ACK ≠ tamamlanma;
  ERR ≠ "hareket yok".
- **Katı STAT şeması:** tam 9 token, her biri `^[IMHSF]\d+$`; uymayan token "bilinmiyor".
- **Tamamlanma:** yazma başarılı + bu işleme ait ACK + ACK'ten **sonra istenmiş** taze STAT
  + tüm tokenlar `S` + ARR/ALL'da her motor hedefin toleransında, HOME'da her motor ≈0
  (toleranslar KARARLAR'da). "HOME sırasında eski S300" ve "yazma başarısız + eski S0"
  kapanmalı.
- **İşlem ve bağlantı nesli:** her işleme/bağlantıya artan kimlik; eski nesle ait cevap ve
  callback yok sayılır. Kalan riski (zaman aşımına uğramış isteğin geç cevabı) dokümante et;
  azaltma: zaman aşımı sonrası modülü "senkron dışı" işaretle, ardışık 2 tutarlı taze STAT iste.

**1c. Arızada iş sonucu ≠ makine gözetimi (Bulgu 4).** İlk `F` işi başarısız yapar ama
gözetimi bitirmez: poll sürer, işleme dahil bütün modüller (tamamlananlar ve arızalılar dahil)
izlenir, hiçbir modülde `M/H` kalmayana ve durum kararlı olana kadar yeni hareket kilitli.
Firmware `SETTLED`'dan `REARM_BAND` aşılınca `MOVING`'e dönebiliyor → tamamlanan modül
izlemeden düşmez. "Hedefe ulaştı" ≠ "güvenli erişim". `allHomed` arızada eski `True` kalmasın.

**1d. Eski zamanlanmış komut sızması (Bulgu 8).** `_dispatch`'in `QTimer.singleShot` zinciri
iptal edilebilir; her callback işlem+bağlantı neslini kontrol eder. `HOME:01 → yeni ARR:01 →
eski HOME:02` oluşamamalı.

**1e. Durdurma metni (Bulgu 2).** `BusyOverlay.qml` metni bölüm C'deki gibi. Uyarı **herhangi
bir hareket komutu kabul edilmişken** (Tester'ın MOV/ALL/HOME'u dahil) görünsün. Normal hareket
bandı alarm kırmızısını kullanmasın (`Theme.js` ISA-101 kuralları: nötr/amber).

### Faz 2 — P1: bağlantı, veri yaşı, referans, alarm, tek seri yönetici
**2a. Tek seri işlem yöneticisi (Bulgu 11, 14).** Bütün seri trafik tek kuyruktan: üretim
poll'u, Tester'ın bağımsız "Canlı Durum" timer'ı, tarama, dağıtım, terminal. Hatta tek bekleyen
istek; istek başına timeout ve backoff; sınırlı kuyruk. Tarama yeniden girişe kapalı. Dağıtım
ACK'i bekler (80 ms kör aralık yerine); U2 gecikmesi (~1,5 sn) hesaba katılır. Yazma UI
thread'inde değil; `write_timeout` tanımlı; okuyucu tamponu sınırlı. **En kötü durum alarm
gecikmesini hesapla**, `ILERLEME.md` ve `CLAUDE.md`'ye yaz, `KARARLAR.md`'ye `ONAY BEKLİYOR`.

**2b. Bağlantı ve modül erişilebilirliği (Bulgu 5).** Okuyucu istisnaları yutulmaz; kopma
sinyalle bildirilir. Bağlantı nesli; modül başına son cevap zamanı; `activeSlaves` zaman
aşımıyla düşer ve bağlantı kesilince temizlenir. Yeniden bağlanma / slave reset sonrası
**uzlaştırma**: önce güncel durum, referans varsayılmaz, otomatik hareket tekrarı yok.

**2c. Referans geçerliliği (Bulgu 5).** `_homed` düşer: bağlantı kopması, beklenmeyen `I`
durumları (slave reset), HOME sırasında `F`, HOME zaman aşımı. Dokuz `I0` → `allHomed=False`.

**2d. Veri yaşı (Bulgu 5, 7).** Her modülün son geçerli durumunun yaşı görünür; eşik aşılınca
`StatusStrip` "Anormal durum yok" demez → **"Güncel makine durumu bilinmiyor."**

**2e. Kalıcı alarm modeli (Bulgu 7).** Modül/motor bazlı aktif alarm listesi. "Uyarıyı gördüm"
(onay) ≠ "giderildi": onay silmez; alarm yalnız taze geçerli STAT ile normale dönüş
doğrulanınca kapanır; iletişim belirsizliğinde çözüldü sayılmaz. İşlem dışı `F` de alarm
üretir. Zaman aşımı kalıcı kayıt üretir. Yeni işlem alarmı temizlemez.

**2f. Kısmi cevap (Bulgu 6).** `STAT:01:F0,S0,S0,S0,U2ERR` → mevcut `F` işlenir; eksik
motorlar "bilinmiyor", pozisyon `—`; U2 iletişim arızası ayrı ve kalıcı alarm. U2'deki motorlar
firmware sürümüne bağlı (v5.1: U2=1-5, v5f411: U2=5-9) → **sabit kodlama**, config'ten oku
(KARARLAR: sürüm yapılandırılmamışsa muhafazakâr davranış). `U` "Boşta"ya düşmez; sayıya
çevrilemeyen değer `0` olmaz. Token sırasını firmware kaynağından doğrula (okuma serbest).

**2g. Beklenen modül kümesi ve kapsam (Bulgu 13).** `UI_v2.0_QML/config/machine.json`
(beklenen modüller + firmware sürümü; varsayılanlar KARARLAR'da). Bulunan ≠ beklenen
ise BAŞLA kilitli veya eksikleri listeleyen açık onay (KARARLAR). "Seçili modüle uygula" için
modül aktif **ve** referanslı. Tester girişleri STL ile aynı politikaya bağlı (10–590);
boş/metin/eksi/>590 gönderilmez, sebep gösterilir. "Tümünü Sıfırla" (KARARLAR).

### Faz 3 — P1: model yükleme ve veri sunumu
**3a. Atomik STL yükleme (Bulgu 9).** Durum makinesi (boşta/yükleniyor/hazır/hata) + yükleme
kimliği. Dosya adı ve grid **birlikte**, yalnız başarılı yüklemede kabul. Yükleme sürerken
hareket başlatılamaz. Hata operatöre gösterilir. Eski yükleme sonuçları iptal; ters sırada biten
thread yeniyi ezemez. İşlem başında hedeflerin **değişmez kopyası**.

**3b. İnterpolasyon şeffaflığı.** Algoritmayı değiştirme; **kaç hücre dolduruldu/kırpıldı**
bilgisini ve uyarıyı göster. Varsayımları dokümante et. Komşu fark kontrolü → öneri.

**3c. Hedef ≠ ölçüm (Bulgu 10).** Modül başına durum deposu (kimlik + zaman damgası).
Tester'da seçim değişince önceki modülün değerleri temizlenir ("bilinmiyor"); STAT hedef
alanlarını **ezmez**; HOME/ALL butonları kutuları hedefe çekmez. "Hedef" ve "Son ölçüm
(… sn önce)" ayrı. Harita/3D başlığı: **"Model hedefleri — ölçülen konum değildir"**.
Canlı ölçüm haritası → öneri listesi.

### Faz 4 — Türkçe metin, operatör akışı, dokunmatik
**4a.** `INCELEME.md` metin tablosunun **her satırı** (16 satır): önerilen metin + "doğru olması
için gereken davranış" sağlanmadan metni değiştirme. Uygulanamayanı gerekçesiyle listele.

**4b. Ek dil/akış maddeleri (7 madde, hepsi):** doğru Türkçe karakterler; "Klavuz" →
**Kılavuz**, "Tester" → **Bakım / Motor testi**, "SERIAL MONITOR" → **Haberleşme günlüğü**;
`ARRERR`/`U2ERR`/istisnalar operatöre **neden / etki / adım** olarak; "Yenile" → **Portları
yenile**, "Temizle" → **Günlüğü temizle**; "İptal"/"Tamam" yalnız kapatıyorsa **Vazgeç/Kapat**,
"Tekrar dene" akışı **ekleme**; devre dışı nedenleri **bütün** butonlarda, modül yoksa BAŞLA
önceden kapalı; onaylarda **model adı, hedef modül listesi, motor sayısı**; `--demo` için
kalıcı **SİMÜLASYON** bandı.

**4c. Dokunmatik ve görsel (6 madde, hepsi):** alarm onayı 40 px → **≥80 px**; Tester
terminal/gönder 1080p'de kesilmesin, yardım satırları taşmasın; harita hücreleri <80 px →
ayrıntı/büyütme kontrolü; uygulama içi **sayısal tuş takımı** (QML, yeni bağımlılık yok);
hareket bandı alarm kırmızısı değil; okunabilirlik/eldiven/parlama → PANELDE TEST.
Sabah inceleme için offscreen ekran görüntüleri (1920×1080, her sayfa) →
`docs/v2.2/ekran/` (Quick3D boş çıkar, beklenen).

### Faz 5 — Doküman ve teslim
- `docs/v2.2/PROTOKOL_ONERI_v5.2.md` (yalnız öneri): host STOP/ABORT (kapsam, cevap, sınırlar),
  ACK/STAT'ta işlem/sıra numarası yankısı, HOME referans doğrulama/arıza ayrımı, U2 başarısızken
  U3 davranışı.
- `docs/v2.2/GUVENLIK_ACIK_KONULAR.md` (bölüm C).
- `CLAUDE.md`: hareket yetkisi tek nokta, işlem/bağlantı nesli, tek seri yönetici, alarm yaşam
  döngüsü, veri yaşı eşiği, en kötü alarm gecikmesi, `machine.json`, test komutu.
  Pi dağıtım bölümüne dokunma.
- `UI_v2.0_QML/README.md`: testlerin çalıştırılması, `--demo` sınırları.
- `ILERLEME.md`'de **kapsama tablosu** doldurulmuş + **SABAH ÖZETİ** (en üstte).

---

## E. SABAH ÖZETİ BİÇİMİ (`ILERLEME.md` en üstü)

1. Hangi fazlar bitti, son commit hash'i
2. Kırmızıdan yeşile dönen testler / hâlâ kırmızı (expectedFailure) olanlar
3. Operatörün göreceği davranış değişiklikleri
4. `KARARLAR.md`'de `ONAY BEKLİYOR` olanlar
5. ENGELLENDİ / stash'e alınan işler
6. **PANELDE TEST** — numaralı, somut adımlar
7. **DONANIMDA TEST GEREKLİ** — numaralı adımlar (HOME→STAT→ARR; komutu kullanıcı gönderir)
8. Pi'ye deploy için öneri (kullanıcı yapacak; kiosk `ui_v2.0`'da kalıyor)
