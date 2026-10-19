# Protokol v5.2 — öneri (UYGULANMADI)

> **Durum:** yalnız öneri. v2.2 turunda firmware'e ve protokole dokunulmadı (GOREV B1).
> Firmware kaynağı yalnız okundu: `Adaptif_kalip_v5f411_U3_4motor/src/main.cpp`,
> `Adaptif_kalip_v5f411_U2_5motor/src/main.cpp`, `Adaptif_kalip_v5_U3_4motor/src/main.cpp`.
> Buradaki her madde donanımda tasarlanıp **test edilmeden** güvenlik iddiası taşımaz.

Arayüz v2.2, protokolün sınırlarını **azaltır ama kapatamaz**. Aşağıdaki maddeler, v2.2'de
"yazılımla çözülemez / kalan risk" olarak işaretlenen konuların firmware/protokol tarafı içindir.

---

## 1. Host'tan erişilebilir STOP / ABORT

**Bugün:** v5.1/v5f411 komut işleyicilerinde host STOP/ABORT yok. `mStopAll()` yalnız açılışta
çağrılan iç fonksiyon (`mStop` + `ST_IDLE`). "Bağlantıyı kes" yalnız takibi bırakır.
HMI metni: **"Bu arayüzden hareket durdurulamıyor."**

**Öneri:**
- `STOP:id` (modül) ve `STOP:00` (yayın, bütün modüller). U3 alınca **yerel motorları hemen**
  `mStop` + yeni durum `ST_STOPPED` (`P` harfi gibi ayrı bir harf; `I` ile karışmasın) yapar,
  U2'ye `INTERNAL_STOP` iletir.
- Cevap: `STOPOK:id:<u3>:<u2>` — `u3`/`u2` = `1` durdurma uygulandı / `0` uygulanamadı (U2
  cevap vermedi). Yayın komutuna cevap verilmez (çakışma); host ardından her modüle `STAT` sorar.
- **Sınırlar (metne yansımalı):** STOP bir **yazılımsal** durdurmadır; RS485/USB/köprü/firmware
  çalışıyorsa etkilidir. Cevap vermeyen aktüatör için "durdu" denmez; durma ancak sonraki taze
  `STAT`'ta bütün tokenlar `P`/`S` ise "durduğu bildirildi (firmware)" diye yazılır. Bu komut
  **hiçbir zaman "E-stop" / "acil durdurma" diye adlandırılmaz** (bkz. `GUVENLIK_ACIK_KONULAR.md`).
- `ST_STOPPED` sonrası referans korunur mu? Sayaç sürüşü kesince kaybolmaz; ama yük altında
  kayma varsa REARM davranışı tanımlanmalı (bkz. 5).
- U2, U3'ten belirli süre (ör. 2 sn) hiçbir şey duymazsa kendi motorlarını durdurmalı mı?
  (Bugün U2'de iletişim gözetimi yok; U3 ile bağı kopan U2 hareketi bitirene/200 sn zaman
  aşımına kadar sürer.) Karar donanım güvenlik değerlendirmesine bağlı.

## 2. İşlem / sıra numarası yankısı (Bulgu 1)

**Bugün:** cevaplar yalnız fiil + modül (+ motor) taşır. Zaman aşımına uğramış bir isteğin geç
cevabı bir sonraki aynı istekle eşleşebilir (v2.2 testi:
`test_link.test_residual_risk_late_reply_matches_next_same_request`).

**v2.2 azaltması (2026-10-19 düzeltme turunda UYGULANDI, arayüz tarafı):** zaman aşımı, yazma
hatası/takılması veya istenmemiş cevaptan sonra o modülün "cevap sırası belirsiz" sayılır; gözetim
STAT yerine `PING` gönderir, `PONG`'dan **sonra** istenen STAT veri olarak kullanılır (ondan önceki
cevaplar yalnız arıza alarmı üretebilir). Dayanak: firmware komutları sırayla işler ve cevaptan
sonra RS485 girişini boşaltır (`RS485_RX_Mode`), köprü sırayı korur — **kaynaktan okundu, donanımda
ölçülmedi.** Bu bir çıkarımdır, kanıt değildir: iki ardışık benzer cevap tazeliğin kesin kanıtı
sayılmaz (K11, `GUVENLIK_ACIK_KONULAR.md` §5). Kesin çözüm aşağıdaki istek kimliğidir.

**Öneri:** isteğe isteğe bağlı sıra numarası: `STAT:01#1234`, `ARR:01:…#1235`.
Cevap aynı numarayı yankılar: `STAT:01#1234:S300,…`, `ARROK:01#1235`. Numarasız istek eski
biçimde cevaplanır (geriye uyumlu). Host yalnız beklediği numaralı cevabı tamamlanma için kullanır.
Ek: `STAT` cevabına modülün **son kabul ettiği hareket komutunun numarası** (`L1235`) eklenirse
"bu STAT hangi işten sonra" sorusu kesin cevaplanır.

## 3. HOME referans doğrulama / arıza ayrımı (Bulgu 12)

**Bugün (v5f411 `Motor_Update_All`, `ST_HOMING`):** kalkış payı `HOME_GRACE_MS=3000` sonrası
encoder `HOME_SETTLE_MS=1200` ms boyunca `HOME_MOVE_PULSES=3`'ten az değişirse sayaç sıfırlanır,
`ST_SETTLED`. Bağımsız limit konumu okunmaz → kopuk encoder, enerjisiz motor veya sıkışma da
"referans alındı" üretebilir. HMI metni: **"Referans alma tamamlandı (firmware bildirimi)"**.

**Öneri:**
- Referans anında **hareket kanıtı**: HOME başladıktan sonra en az N puls geri hareket görülmeden
  oturma kabul edilmesin; görülmezse `ST_FAULT` + ayrı sebep kodu (`F` yerine `R` = referans
  doğrulanamadı, ya da `F` + sebep alanı).
- Mümkünse limit anahtarı/akım artışı gibi **bağımsız** bir sinyal okunsun (donanım değişikliği).
- `STAT` token'ına sebep: `F0` yerine `F0/STALL`, `F0/TIMEOUT`, `F0/HOME` (geriye uyum için ayrı
  `WHY:id` komutu da olabilir).

## 4. U2 başarısızken U3 davranışı (Bulgu 6 eki)

**Bugün (`ALL`/`ARR`/`HOME` işleyicileri):** U3 önce U2'ye `INTERNAL_*` gönderir; U2 1,5 sn
içinde onaylamasa bile **kendi yerel motorlarını yine zamanlar**, sonra `*ERR` döner. Yani
`ALLERR/ARRERR/HOMEERR` "hiç hareket başlamadı" demek değildir. v2.2 bunu "belirsiz, izlemeye
devam" diye işler (SYNC uyarısı, hareket kilitli, kalan modüllere komut yok).

**Uygulanmış davranış (arayüz v2.2; firmware DEĞİŞMEDİ):** `*ERR`, ACK zaman aşımı, yazma
hatası/takılması = "kabul belirsiz": işlem biter, kalan modüllere komut gönderilmez, modül "senkron
dışı" (SYNC), izleme sürer, komut **otomatik tekrar edilmez**. Arayüz yarım modül hareketini
**önlemez**; yalnız görünür kılar ve sonraki hareketi kilitler.

**Öneriler (donanım/firmware ekibi seçmeli; hiçbiri atomiklik garantisi vermez):**
- (a) **U2 onayı gelmeden U3 yerel motorları başlatmaz**, `ARRERR:id:U2` döner. Bu, "U2 komutu hiç
  almadı" durumunda yarım hareketi önler; ama **hep ya da hiç DEĞİLDİR**: U2 komutu almış ve kendi
  motorlarını zamanlamış, yalnız `INTERNAL_*OK` onayı USART2'de kaybolmuş/gecikmiş olabilir → U2
  motorları hareket eder, U3 motorları etmez; yarım modül hareketi yine mümkündür. (Önceki sürümde
  bu seçenek yanlışlıkla "hep ya da hiç" diye anlatılmıştı.)
- (b) Atomikliğe **yaklaşmak** için iki aşamalı protokol: U3 → U2 `PREPARE` (hedefler) + onay →
  U3 her iki tarafa `GO`; U2 belirli süre `GO` almazsa hazırlığı düşürür. Bu da `GO` kaybı ve
  zamanlaması için kendi belirsizliğini taşır (U3 `GO` gönderdi, U2 almadı → yine yarım hareket);
  bu yüzden her durumda U3, U2'nin gerçek durumunu `STAT`'ta (yaşıyla) raporlamalı, host da
  sonucu yalnız `STAT` ile değerlendirmelidir.
- (c) Mevcut davranış korunur ama cevap ne olduğunu söyler: `ARRPART:id:U3` (U3 motorları
  zamanlandı, U2 belirsiz).
- `ARRERR:FMT` gibi id'siz cevaplar `ARRERR:id:FMT` biçimine getirilmeli.

## 5. Diğer

- **STAT'ta U2 yaşı:** `U2ERR` yerine son başarılı U2 durumunun yaşı + `U2?` işaretli tokenlar.
- **REARM:** `SETTLED`'dan `REARM_BAND=60` puls aşılınca firmware kendiliğinden `MOVING`'e döner.
  Bu, "tamamlanan" modülün komutsuz hareketi demektir. v2.2 bunu sürekli gözetimle görür (test
  `test_completed_module_rearm_is_supervised`); firmware'de REARM sayısı/zamanı `STAT`'a
  eklenmeli, istenirse kapatılabilir olmalı.
- **Negatif konum:** `%ld` eksi değer basabilir; v2.2 katı şeması (`^[IMHSF]\d+$`) bunu
  "bilinmiyor" sayar (K19). Ya firmware 0'a kırpmalı (bilgi kaybı) ya da şema `-?\d+` olarak
  genişletilmeli — karar verilmeli.
- **Yeniden başlama bildirimi:** açılışta `BOOT:id:<sebep>` yayını, host'un reset tespitini
  (`I` harfi) kesinleştirir.

---

## Ek A — Protokol değişikliği gerektirmeyen arayüz önerileri (sonraki tur)

1. **Canlı ölçüm haritası** (3c): 12×12 haritanın ikinci modu olarak modül deposundaki son
   ölçümler + yaşları (eski/bilinmeyen hücre gri tarama). v2.2'de harita yalnız MODEL hedefidir
   ve öyle etiketlenir; ölçüm Bakım ekranında ve "Modül ayrıntısı" penceresinde.
2. **Komşu yükseklik farkı denetimi** (3b, K14): membran için izin verilen komşu fark (mm)
   belirlenince, model yüklemede bu değeri aşan hücre çiftleri işaretlensin ve BAŞLA ek onay
   istesin. v2.2 yalnız en büyük komşu farkını raporlar.
3. **Alarm geçmişi kalıcılığı:** alarm kayıtları uygulama yeniden başlayınca kaybolur (bellek
   içi). Dosyaya (ör. `~/kalip_alarm.log`) satır satır yazılması önerilir.
4. **Gözetim önceliği:** 2026-10-19'da gözetimin **diğer trafiğe** önceliği uygulandı (kuyruğun
   önü, operatör sorgu sınırı — K34). Hâlâ öneri: işlem modüllerinin daha sık sorgulanması
   (round-robin ağırlığı); şu an bütün bilinen modüller eşit (en kötü gecikme hesabı bu eşitliğe
   dayanır).
