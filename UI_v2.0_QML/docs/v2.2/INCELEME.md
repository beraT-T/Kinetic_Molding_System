# v2.2 — Bağımsız inceleme raporu (tam metin, değiştirilmeden)

> Kaynak: kullanıcının ilettiği operatör-güvenilirliği incelemesi. İnceleme 0733c9c HEAD + commit edilmemiş dosyalar üzerinde yapıldı. Satır numaraları o sürüme aittir; kod değiştikçe kayabilir.

Mevcut arayüzü güvenilir operatör kullanımı için kabul etmem. En kritik nedenler: eski STAT'ın yeni işlemi tamamlanmış gösterebilmesi, HOME kilidinin Tester üzerinden aşılması, bağlantı kaybının güncel durumdan ayrılmaması ve arıza sonrası izlemenin bırakılması. Bunların bir bölümünü gerçek Controller kodunda, donanımsız testlerle yeniden ürettim.

İnceleme yerel çalışma kopyasına aittir: 0733c9c… HEAD'i ve commit edilmemiş mevcut dosyalar. Pi'de aynı sürümün çalıştığını doğrulamadım. Kod değiştirmedim; port açmadım, SSH/deploy/restart yapmadım, donanıma komut göndermedim.

Aşağıda P0 yanlış/güvenilir biçimde denetlenemeyen hareket açısından öncelikli, P1 ciddi işletim hatası, P2 kullanılabilirlik ve bakım sorunudur. "Kesin hata" kod veya donanımsız yeniden üretim; "risk" ise fiziksel sonucu ölçülmemiş durum anlamındadır.

## 1. P0 — Eski STAT, gönderilememiş komutu bile başarıya dönüştürüyor. Kesin hata.

Kaynak: `UI_v2.0_QML/backend/controller.py:320`, özellikle 359–391, 491–538.

- Senaryo: ARR gönderimi başarısız; önceki işlemden kalma `STAT:01:S0,…,S0` geliyor. Diğer senaryoda HOME başlatıldıktan sonra eski `S300` cevabı geliyor.
- Beklenen: Komutun kabulü ve bu işleme ait güncel tamamlanma ayrı doğrulanmalı; belirsiz cevap başarı üretmemeli.
- Gerçek: `_send()` yazma sonucunu kullanmıyor. ACK/ERR cevapları işlem durumuna işlenmiyor. STAT'ta yalnız ilk harflerin S olması yeterli; hedef, sayısal değer, cevap yaşı veya işlem ilişkisi kontrol edilmiyor.
- Kanıt: Sahte seri bağlantıda `write_line=False` ve dokuz `S0`, `(True, "Tamamlandi", "Tum moduller hedef pozisyonda.")` üretti. HOME sırasında dokuz `S300`, `_homed` bilgisini doldurup hemen ARR gönderdi. Dokuz `Sgarbage` de başarı sayıldı.
- Etki: Hareket yapılmadan başarı; HOME bitmeden şekil uygulama; yanlış referansla hareket.
- Düzeltme yönü: Katı STAT şeması, komut kabulü/sonuç ayrımı, işlem ve bağlantı nesli, güncellik kontrolü, hedef toleransı. Protokolde işlem kimliği bulunmadığı için yalnız arayüze zaman damgası eklemek gecikmiş cevabın hangi komuta ait olduğunu tamamen çözmez; protokol desteği değerlendirilmelidir. ACK tek başına yine tamamlanma değildir.

## 2. P0 — Durdurma yönlendirmesi belirsiz; kullanılabilir yazılımsal STOP yok. Donanım güvenliği eksikliği metin hatasından ayrıdır.

Tam metin:

```
DURDURMAK ICIN CIHAZIN GUCUNU KESIN
```

Kaynak: `UI_v2.0_QML/qml/BusyOverlay.qml:92`, `UI_v2.0_QML/backend/controller.py:109`, `controller.py:424`.

- Koşul: `app.busy`, yani `_op` mevcut olduğu sürece görünür. BAŞLA, toplu HOME veya seçili modüle model uygulama bunu açar. Gerçek hareketin görülmesi gerekmez; komut gönderilemese de çıkar. Tester'ın doğrudan MOV/ALL/HOME işlemleri `_op` oluşturmadığından bu uyarı o hareketlerde görünmeyebilir.
- Gerçek durdurma: İncelenen v5/v5f411 komut işleyicilerinde hosttan erişilebilir STOP/ABORT yok. Firmware'deki `mStopAll()` iç fonksiyonu bir kullanıcı durdurma komutu değildir. "Bağlantıyı Kes" yalnız bağlantıyı ve uygulamanın takibini keser.
- Etki: Kullanıcı Pi/USB/kontrol elektroniğini kapatmayı motor enerjisini kesmekle karıştırabilir. Kod hangi güç kaynağını veya anahtarı kastettiğini belirlemiyor.
- Donanım sınırı: Fiziksel E-stop bulunmadığı bilgisi sizden geliyor. Motor besleme devresi, kesme elemanı, depolanmış enerji ve enerji kesilince mekanik davranış incelenmedi. Bu eksiklik metin değiştirerek giderilemez.
- Düzeltme yönü: Önce gerçek durdurma ve enerji ayırma düzeni, erişilebilirliği ve tüm aktüatörlere etkisi yetkin donanım/makine güvenliği değerlendirmesiyle belirlenmeli. Gelecekte eklenecek yazılımsal durdurma fiziksel E-stop olarak adlandırılmamalı; cevap vermeyen aktüatör için "durdu" denmemeli.

Mevcut koddan doğru biçimde söylenebilecek metin: "Bu arayüzden hareket durdurulamıyor." Bunun ardından uygulanabilir bir durdurma talimatı yazabilmek için gerçek güç kesme noktası ve kapsamı doğrulanmalıdır. Şu an anahtar adı uydurmak doğru olmaz.

## 3. P0 — Tester ve terminal, HOME ve hareket kilitlerini aşıyor. Kesin hata.

Kaynak: `UI_v2.0_QML/qml/TesterPage.qml:156`, 248–251; `UI_v2.0_QML/qml/Terminal.qml:58`; `UI_v2.0_QML/backend/controller.py:430`.

- Senaryo: HOME sürerken Tester'a geçip "Test 300 mm", "Tümünü Sıfırla", "Home Hepsi" veya sayı alanında Enter kullanmak. Terminalden hareket komutu göndermek.
- Beklenen: Aynı motoru etkileyen çelişkili işlem, bütün giriş yollarında engellenmeli.
- Gerçek: Bu üç toplu Tester butonu yalnız `connected` kontrol ediyor. Enter yolu da yalnız bağlantıyı kontrol ediyor. Backend doğrudan komut fonksiyonlarında busy/referans/arıza kilidi yok. Terminalde daha da az kısıtlama var.
- Kanıt: Aktif HOME sırasında gerçek Controller şu sırayı üretti: `HOME:01`, `MOV:01:01:300`, `ALL:01:0`.
- Etki: Referans alma kesilebilir, hedef değişebilir; tek parça membranın farklı bölgeleri beklenmedik biçimde hareket edebilir.
- Düzeltme yönü: Backend'de tek hareket yetkilendirme noktası; bakım işlemlerinde açık kapsam ve yetki; aynı hareket kilidinin bütün UI yollarına uygulanması.

Ana üretim fonksiyonunu iki kez çağırdığımda ikinci hareket gönderilmedi. Ancak bu koruma Tester'a genellenmemiş. Fiziksel çift dokunma davranışını test etmedim.

## 4. P0 — Arızada diğer motorların izlemesi bırakılıyor; "tamamlanan" motor yeniden hareket edebilir. Kesin kod davranışı, fiziksel sonuç risk.

Kaynak: `UI_v2.0_QML/backend/controller.py:359`, 417–428; `Adaptif_kalip_v5f411_U3_4motor/src/main.cpp:325`.

- Senaryo: Modül 1'de bir motor F olurken diğer motorlar hareket ediyor.
- Beklenen: İş başarısız sayılsa bile makinenin gözlemi sürmeli; devam eden ve bilinmeyen hareket açıkça gösterilmeli.
- Gerçek: İlk F, `_finish_op()` ile poll zamanlayıcısını durduruyor, `_op=None` yapıyor, busy kilidini kaldırıyor. Diğer motorlara durdurma gönderilmiyor.
- Kanıt: F + sekiz M testinde `busy=False`, işlem timer'ı kapalı, önceki `allHomed=True` kaldı.
- Ek durum: Firmware SETTLED durumunda hata `REARM_BAND` değerini aşınca kendiliğinden MOVING'e dönebiliyor. Controller ise tamamlanmış modülü pending listesinden çıkarıp işlem poll'undan düşürüyor.
- Etki: Arıza penceresi kapanınca devam eden veya yeniden başlayan hareket izlenmeyebilir; yeni komutlara izin verilebilir.
- Düzeltme yönü: İş sonucu ile makine gözetimini ayırmak; tamamlanan/arıza veren modülleri de izlemek; belirsiz durum çözülene kadar yeni hareketi kilitlemek. "Hedefe ulaştı" ile "hareket edemez/güvenli erişim" aynı durum değildir.

## 5. P1 — USB kopması ve slave yeniden başlaması durum bilgisini geçersizleştirmiyor. Kesin hata.

Kaynak: `UI_v2.0_QML/backend/serial_manager.py:75`, `UI_v2.0_QML/backend/controller.py:147`, 520–538; `UI_v2.0_QML/qml/StatusStrip.qml:47`.

- Senaryo: USB çıkıyor veya Pi açıkken slave enerjisi kesilip geri geliyor.
- Beklenen: Bağlantı, konum ve referans geçerliliği ayrı ayrı düşmeli; son bilinen verinin yaşı görünmeli.
- Gerçek: Okuma istisnaları yutuluyor; Controller'a kopma olayı iletilmiyor. `activeSlaves` PONG ile ekleniyor ama zaman aşımıyla düşmüyor. Açık bağlantıyı elle kapatınca bile liste temizlenmiyor. Yeni `I0` durumları `_homed` bilgisini geçersizleştirmiyor.
- Kanıt: Referanslı modüle dokuz `I0` enjekte edildiğinde `allHomed=True` kaldı. Disconnect sonrası aktif liste `[1]` kaldı.
- Etki: "bağlı", "referans alınmış" ve "Anormal durum yok" bilgileri eski olabilir. Slave sayacını sıfırlamışken üretim HOME'u atlayabilir.
- Düzeltme yönü: Bağlantı nesli, son cevap zamanı, modül bazlı erişilebilirlik ve referans geçerliliği; reset/yeniden bağlanma sonrası uzlaştırma.

Uygulama yeniden açılınca `_homed` boş başlıyor; bu kısmı doğru. Ancak önceki hareketin sürüp sürmediği geri kazanılmıyor. Slave yeniden başlatma kodu sayaçları sıfır/IDLE yapıyor; bu fiziksel olarak sıfıra ulaşıldığı anlamına gelmiyor.

## 6. P1 — Kısmi modül arızası yanlışlıkla "boşta/0" gösteriliyor; yerel F bile kayboluyor. Kesin hata.

Kaynak: `Adaptif_kalip_v5f411_U3_4motor/src/main.cpp:541`, `UI_v2.0_QML/backend/controller.py:364`, `UI_v2.0_QML/qml/TesterPage.qml:17`.

- Senaryo: U2 cevap vermiyor; U3 `STAT:01:F0,S0,S0,S0,U2ERR` gönderiyor.
- Beklenen: Motor 1 arızası ve motor 5–9 iletişim kaybı ayrı görünmeli.
- Gerçek: Controller dokuz token yoksa F kontrolüne ulaşmadan dönüyor. Tester eksik tokenları `I0`, sayıya çevrilemeyen değerleri `0` yapıyor. `U` da durum etiketi fonksiyonunda "Boşta"ya düşüyor.
- Kanıt: Bu gerçek firmware cevap biçimini enjekte ettiğimde `faultText=""`, `busy=True`, sonuç yoktu.
- Etki: Motor arızası gecikir; veri alınamayan motorlar duruyor ve sıfırdaymış gibi görünür.
- Düzeltme yönü: Eksik/geçersiz durum için "bilinmiyor"; pozisyon için —; kısmi cevapta bile alınabilen arızaları işle. U2 iletişim arızasını ayrı ve kalıcı göster.

Ayrıca U3, U2 komut cevabı başarısız olsa da kendi motorlarını zamanlayabiliyor: `Adaptif_kalip_v5f411_U3_4motor/src/main.cpp:467`. Dolayısıyla `ALLERR/ARRERR/HOMEERR`, "hiç hareket başlamadı" anlamına gelmez.

## 7. P1 — Alarmı onaylamak arızayı siliyor; işlem dışındaki arızalar kalıcı alarma dönüşmüyor. Kesin hata.

Kaynak: `UI_v2.0_QML/qml/StatusStrip.qml:54`, `UI_v2.0_QML/backend/controller.py:129`, 288–289, 359–375.

- Senaryo: Arıza bildirimi sonrası ONAYLA; veya Tester'da F alınması.
- Beklenen: "Gördüm" ile "arıza giderildi" ayrılmalı; çözülmemiş arıza görünür kalmalı.
- Gerçek: ONAYLA doğrudan metni siliyor. Yeni işlem başlatmak da metni temizliyor. `_op` yokken F, kalıcı alarm üretmiyor. Timeout yalnız sonuç penceresinde; kalıcı fault kaydı yok.
- Kanıt: Arıza testinden sonra `clearFault()` ile kayıt boşaldı. 301 saniyeye taşınmış işlem timeout verdiğinde de kalıcı alarm boş kaldı.
- Etki: Çözülmemiş arıza "Anormal durum yok" görünümüne dönüşebilir.
- Düzeltme yönü: Modül/motor bazlı aktif alarm listesi, ayrı onay bilgisi, doğrulanmış normale dönüş; iletişim belirsizliğinde alarmı çözüldü saymamak.

Aynı F'nin sürekli modal açtığı bir döngü bulmadım; mevcut sorun tersine, ilk arızadan sonra gözetimin ve alarm kapsamının daralması.

## 8. P1 — Eski zamanlanmış komut, yeni işlemin içine sızabiliyor. Kesin hata.

Kaynak: `UI_v2.0_QML/backend/controller.py:320`.

- Senaryo: Çok modüllü HOME dağıtımı kesiliyor; 80 ms'lik eski callback çalışmadan yeni işlem başlıyor.
- Beklenen: Eski işe ait bütün gönderimler geçersiz olmalı.
- Gerçek: Callback yalnız herhangi bir `_op` var mı diye bakıyor. İşlem kimliği yok; eski phase/targets yeni işlem sırasında kullanılabiliyor.
- Kanıt: Timer callback'lerini kontrollü çalıştıran testte sıra `HOME:01` → yeni `ARR:01` → eski `HOME:02` oldu.
- Etki: Yeni şekil uygulaması sırasında beklenmedik referans hareketi.
- Düzeltme yönü: İptal edilebilir dağıtım timer'ı ve callback başına işlem/bağlantı nesli kontrolü.

Bu bir gerçek panel zamanlama testi değil; gerçek `_dispatch` kodunun kontrollü callback sırasıyla yeniden üretimidir.

## 9. P1 — Yeni STL başarısız olsa da eski hedefler yeni dosya adı altında gönderilebilir. Kesin hata; eşzamanlı yüklemede ek risk.

Kaynak: `UI_v2.0_QML/backend/controller.py:203`, `UI_v2.0_QML/qml/MainPage.qml:127`.

- Senaryo: A modeli hazırken B seçiliyor; B hesaplanamıyor veya hâlâ hesaplanıyor.
- Beklenen: Dosya adı ve uygulanabilir hedef grid'i aynı başarılı yüklemeye ait olmalı; yükleme boyunca hareket başlatılamamalı.
- Gerçek: Dosya URL'si hesaplama öncesinde değişiyor; eski grid tutuluyor. Hesaplama busy durumuna dahil değil. Hata yalnız terminale gidiyor.
- Kanıt: Kontrollü interpolasyon hatasında gösterilen dosya değişti, eski 123 mm grid'i kaldı, `busy=False` oldu.
- Etki: Operatör B'yi uyguladığını düşünürken A uygulanabilir. Birden fazla yükleme thread'i sonucu ters sırada tamamlayabilir; hareket dağıtımı sırasında grid de değişebilir.
- Düzeltme yönü: Model yükleme durum makinesi, yükleme kimliği, atomik dosya+grid kabulü ve işlem başında değişmez hedef kopyası.

İnterpolasyon ayrıca sınır dışı bölgeleri `maxZ` ile doldurup 10–590 mm'ye kırpıyor: `UI_v2.0_QML/backend/stl_interpolator.py:32`. Bu dönüşümler operatöre açıklanmıyor. Birim, yön, çok katmanlı yüzey ve membran için izin verilen komşu yükseklik farkları doğrulanmadan fiziksel uygunluk sonucu çıkarılamaz.

## 10. P1 — Gösterilen konumlar hedef/ölçüm/modül kimliğini karıştırıyor. Kesin veri sunum hataları.

Kaynak: `UI_v2.0_QML/qml/TesterPage.qml:17`, 91, 229–256; `UI_v2.0_QML/qml/Heatmap.qml:22`; `UI_v2.0_QML/qml/Surface3D.qml:40`.

- Senaryo: Modül 1'in STAT'ı görüldükten sonra cevap vermeyen Modül 2 seçiliyor; veya operatör hedef yazarken canlı STAT geliyor.
- Beklenen: Her modülün verisi kimliği ve yaşıyla saklanmalı; ölçülen konum ile girilen hedef farklı alanlar olmalı.
- Gerçek: Seçim değişince motor kutularındaki önceki değer/durum sıfırlanmıyor. Gelen STAT doğrudan hedef girişlerini değiştiriyor. HOME/ALL butonları da aynı kutuları henüz hareket bitmeden hedef değere ayarlıyor.
- Etki: Modül 1'in "Hazır" bilgisi Modül 2 altında kalabilir; hedef girişi ölçümle ezilebilir.
- Ek: "POZİSYON HARİTASI" ve 3D görünüm yalnız `gridData`, yani STL hedeflerini gösteriyor; canlı motor konumlarını göstermiyor.
- Düzeltme yönü: "Hedef" ve "Son ölçüm" ayrımı; modül bazlı durum deposu; bilinmeyen/eskimiş görünümü; haritayı açıkça hedef haritası olarak adlandırmak.

Mantıksal grid eşlemesi testte tutarlıydı: Modül 1 `[0,1,2,12,13,14,24,25,26]`, Modül 16 `[117,118,119,129,130,131,141,142,143]`. Fiziksel montaj yönü ve sahadaki firmware/soket eşlemesi doğrulanmadı.

## 11. P1 — Round-robin tek başına seri işlemleri düzenlemiyor. Kesin mimari boşluk, hat çakışması risk.

Kaynak: `UI_v2.0_QML/backend/controller.py:184`, 320–357; `UI_v2.0_QML/qml/TesterPage.qml:30`; `Adaptif_kalip_v5f411_U3_4motor/src/main.cpp:357`.

- Senaryo: Canlı Durum açıkken üretim; taramaya çift dokunma; U2 cevabının 1,5 saniye gecikmesi.
- Beklenen: Bütün seri işlemler tek zamanlayıcı/istek-cevap düzeniyle yönetilmeli.
- Gerçek: Üretim poll'u 1 sn'de tek modül, fakat Tester'ın bağımsız 1,2 sn timer'ı busy/sayfa kontrolü olmadan çalışıyor. Tarama ayrı thread; tekrarlı tarama engeli yok. Dağıtım 80 ms aralıkla, ACK beklemeden ilerliyor.
- Etki: Aynı yarı çift yönlü hatta örtüşen trafik ve gecikmiş cevap riski. U2 timeout'u 80 ms dağıtım veya 1 sn poll aralığını aşabiliyor.
- Düzeltme yönü: Tek seri işlem yöneticisi, bekleyen cevap takibi, tanımlı timeout/backoff ve sınırlı kuyruk. Hareket komutlarında kör otomatik retry kullanılmamalı; önce kabul durumu uzlaştırılmalı.

Mevcut kodda ACK timeout/retry mekanizması bulmadım; yalnız faz başına 300 sn var. 16 bekleyen modülde her modül yaklaşık 16 saniyede bir sorulur. Bu, arıza görünürlüğü için açıkça kabul edilmesi gereken bir gecikmedir.

## 12. P1 — HOME tamamlandı bilgisi fiziksel referansın bağımsız kanıtı değil. Kesin algoritma, donanımda doğrulanacak risk.

Kaynak: `Adaptif_kalip_v5f411_U3_4motor/src/main.cpp:313`, `Adaptif_kalip_v5f411_U2_5motor/src/main.cpp:279`.

- Senaryo: HOME sırasında encoder değişmiyor; neden fiziksel son konum, kopuk encoder, enerjisiz motor veya sıkışma olabilir.
- Beklenen: Geçerli referans ile arızadan kaynaklanan hareketsizlik ayrılmalı.
- Gerçek: Kalkış payından sonra yeterince uzun süre puls değişmemesi sayaç sıfırlama ve SETTLED üretir. Kod bu aşamada bağımsız limit konumu okumuyor.
- Etki: HMI "referans alındı" derken fiziksel sıfır yanlış olabilir; sonraki hedefler güvenilir olmayabilir.
- Düzeltme yönü: Referans doğrulama ve arıza ayrımı firmware/donanım seviyesinde ele alınmalı. HMI'de daha kesin bir metin kullanmak bu eksikliği gidermez.

## 13. P1 — Eksik modüllerle üretim ve referanssız test kolayca başlatılabiliyor. Kesin kontrol eksikliği, mekanik etki risk.

Kaynak: `UI_v2.0_QML/backend/controller.py:243`, 263–286; `UI_v2.0_QML/qml/MainPage.qml:35`.

- Senaryo: Beklenen 16 modülden yalnız birkaçı taramada cevap veriyor; BAŞLA seçiliyor.
- Beklenen: İş için beklenen modül kümesi ile bulunan küme karşılaştırılmalı; eksik kapsam anlaşılır olmalı.
- Gerçek: Boş olmayan `_active` listesi yeterli. İş yalnız bulunan modüllere uygulanıyor. Seçili modüle uygulamada o ID'nin aktif listede olması veya referanslı olması gerekmiyor.
- Etki: 144 motorluk hedef gösterilirken yüzeyin yalnız bir kısmı hareket edebilir.
- Düzeltme yönü: Yapılandırılmış beklenen modüller, eksik modül kilidi ve onayda kesin hedef listesi. Her zaman 16 zorunlu demiyorum; izin verilen yapılandırma açık olmalı.

Tester ayrıca 0–600 girişini ve `ALL:…:0` komutunu sunuyor; STL yolundaki 10–590 marjı burada uygulanmıyor. "Tümünü Sıfırla" referans alma değildir. Fiziksel sınıra gitme ve arıza riski aynı doğrulama politikasına bağlanmalıdır.

## 14. P2 — Seri yazma UI thread'ini bekletebilir; tamponlar ve işler sınırlanmamış. Risk, ölçülmüş performans sonucu değil.

Kaynak: `UI_v2.0_QML/backend/serial_manager.py:33`, 60–97; `UI_v2.0_QML/backend/controller.py:216`.

- UI'dan çağrılan `_send()` senkron `write()` ve `flush()` yapıyor; açık `write_timeout` yok. Port kapatma da join ve kilit bekleyebilir.
- STL interpolasyonu arka plan thread'inde; doğrudan UI slotunda hesaplanmıyor. Ancak paralel yüklemeler sınırlanmıyor, ortak grid worker'dan değiştiriliyor.
- Seri okuyucuda newline gelmezse `buf` büyümeye devam ediyor.
- Log modeli 800 kayıtla sınırlı; burada sınırsız kayıt birikimi iddiası doğru olmaz. Buna karşılık sinyal trafiği ve yinelenen worker'lar için geri basınç yok.

Beklenen/düzeltme: Süre sınırlı seri worker, sınırlı satır/tampon/kuyruk, tek model yükleme işi ve eski sonucu iptal etme. Pi'de donma süresi, bellek artışı veya FPS ölçmedim.

## Türkçe metin ve operatör akışı bulguları

Bütün QML dosyalarındaki görünür metinleri ve Controller'ın mesajlarını taradım. Aşağıdakiler yalnız yazım düzeltmesi değil, davranışla birlikte ele alınmalıdır:

| Mevcut metin | Neden sorunlu | Önerilen Türkçe metin | Doğru olması için gereken davranış |
|---|---|---|---|
| DURDURMAK ICIN CIHAZIN GUCUNU KESIN | Hangi enerji/anahtar belirsiz. | "Bu arayüzden hareket durdurulamıyor." Ardından doğrulanmış durdurma talimatı. | Gerçek enerji kesme kapsamı belirlenmeli; hayalî düğme eklenmemeli. |
| Tum moduller hedef pozisyonda. | Tek modül işlemi için de kullanılıyor; hedef doğrulanmıyor. | "Seçili 1 modülün 9 motoru hedefe ulaştı." | Kapsam, güncel cevap ve hedef toleransı doğrulanmalı. Güvenli erişim anlamına gelmemeli. |
| Anormal durum yok | Veri eski veya hiç alınmamış olabilir. | Belirsizlikte: "Güncel makine durumu bilinmiyor." | Veri yaşı gösterilmeli; güncellik süresi aşılınca durum değişmeli. |
| Islem kesildi / Baglanti kesildi | Fiziksel hareketin kesildiği sanılabilir. | "Bağlantı kesildi. Hareketin durduğu doğrulanamıyor." | Takibin kesilmesi ile fiziksel durma ayrı tutulmalı. |
| Yanit vermeyen modul: … | Modül M/H yanıtı verdiği halde de timeout olabilir. | "Süre içinde tamamlanma doğrulanamadı. Modül …: son durum …, … sn önce." | Cevapsızlık ile uzun hareket ayrılmalı. |
| ONAYLA | Arızayı siler; kullanıcı neyi onayladığını bilmiyor. | "Uyarıyı gördüm" | Aktif arıza kaydı görünür kalmalı. |
| Tumunu Sifirla | Gerçekte seçili modüle ALL 0; HOME değil. | "Modül X'in 9 motorunu 0 mm'ye götür" | Önce 0 mm hareketinin izinli olup olmadığı çözülmeli; gerekirse işlem kaldırılmalı. |
| Home Hepsi, Home | İngilizce ve kapsam belirsiz. | "Modül X'e referans al" / "Motor X'e referans al" | İlgili kapsam, kilit ve tamamlanma takibi bulunmalı. |
| Test 300 mm | Bir kontrol testi gibi; dokununca hareket başlatıyor. | "9 motoru 300 mm'ye götür" | Seçili modül ve hareket onayı açık olmalı. |
| POZISYON HARITASI | Ölçülen konum izlenimi veriyor. | "Model hedefleri — ölçülen konum değildir" | Mevcut grid gösterimiyle uyumlu; canlı ölçüm ayrı sunulmalı. |
| Hazir, varsayılan Bosta | S genel hazırlık değil; tanınmayan token boşta sayılıyor. | "Hedefe ulaştı"; geçersizde "Durum bilinmiyor" | Güncellik ve katı token doğrulaması. |
| REFERANS ALINMADI - islem oncesi home yapilacak | Tester ve seçili uygulama yollarında doğru değil. | "Referans doğrulanmadı. Hareketten önce referans alın." | İlgili hareketler gerçekten engellenmeli. |
| STL yukleyip 'Hesapla' deyince… | Hesapla düğmesi yok. | "STL Seç ile dosya seçin. Hesaplama tamamlanınca önizleme burada görünür." | Yükleme/başarı/hata durumu açıkça gösterilmeli. |
| Once bir STL yukleyip hesaplayin. | Aynı olmayan işlem yönlendirmesi. | "STL Seç ile bir model yükleyin ve hesaplamanın tamamlanmasını bekleyin." | Başlat düğmesi hesaplama bitmeden açılmamalı. |
| Encoder kablosunu ve mekanigi kontrol edin. | Diğer motorlar hareket edebilirken müdahaleye yönlendiriyor. | "Motor arızası bildirildi. Diğer motorların durduğu doğrulanmadı. Müdahaleden önce doğrulanmış güvenli durdurma/enerji ayırma prosedürünü uygulayın." | Böyle bir prosedür gerçekten tanımlanmış ve erişilebilir olmalı. |
| Canli Durum surekli izler | Yalnız seçili modülü sorguluyor; güncellik garantisi yok. | "Seçili modülün durumunu sorgula" | Son cevap yaşı ve sorgulama durumu görünmeli. |

Ek dil/akış sorunları:

- Klavuz → Kılavuz, Tester → Bakım / Motor testi, SERIAL MONITOR → Haberleşme günlüğü; görünür Türkçe metinlerde eksik ı/ş/ğ/ü/ö/ç karakterleri yaygın. Kaynak: `UI_v2.0_QML/qml/TesterPage.qml:110`.
- Bağlantı ve STL hataları çoğunlukla yalnız teknik logda. `ARRERR`, `U2ERR`, exception metni operatöre neden/etki/uygulanabilir adım olarak çevrilmiyor.
- "Yenile" port listesini, "Temizle" yalnız günlüğü temizliyor; "Portları yenile" ve "Günlüğü temizle" daha açık.
- "İptal" onay penceresini kapatıyor; hareket iptali yapmıyor. Sonuç penceresindeki "Tamam" da yalnız kapatıyor. Mevcut UI'da "Tekrar dene/Devam et" akışı bulmadım.
- Ana ActionButton'larda devre dışı nedenleri var; diğer butonlarda yok. Modül bulunmamış olması ana BAŞLA düğmesini önceden engellemiyor.
- Onaylarda model adı, hedef modül listesi ve motor sayısı yok. "Tüm bağlı modüller" eski keşif listesine dayanıyor.
- `--demo` için kalıcı, belirgin simülasyon bandı yok; sahte başarı gerçek işlem görünümüyle karışabilir.

## Dokunmatik ve görsel gözlemler

Mevcut Tester görüntüsünü (`UI_v2.0_QML/docs/screenshots/02_tester.png`) ve STL Kontrol görüntüsünü (`UI_v2.0_QML/docs/screenshots/01_stl_kontrol.png`) inceledim. Bunlar güncel fiziksel panel ölçümü değildir.

- Tester görüntüsünde alttaki terminal/gönder bölümü ekran sınırında kesiliyor; motor yardım satırları hücre altına taşıyor. Gerçek 1080p panelde tekrar kontrol edilmeli.
- Alarm onayı 40 px yüksekliğinde; projenin 80 px genel hedefinden küçük: `UI_v2.0_QML/qml/StatusStrip.qml:58`.
- Harita hücreleri 80 px hedefin altında; motor seçimi için büyütme veya ayrı ayrıntı kontrolü gerekli olabilir.
- "Değer yaz + Enter" dokunmatik klavyeye bağımlı. Uygulama içinde sayısal klavye/InputPanel kurulumu görmedim. Pi'nin dış klavye desteği doğrulanmalı; kesin "çalışmıyor" demiyorum.
- Normal hareket sırasında kırmızı durdurma bandı, gerçek arıza kırmızısıyla aynı dikkat kanalını kullanıyor.
- Sönük metinlerin okunabilirliği, eldivenli isabet, güneş/parlama, alarm taşması ve iki parmak hareketi panelde ölçülmedi. Renklerin standartlara uygun olduğunu kod yorumundan kabul etmedim.

## Kısa kabul testi matrisi

| Test | Kabul koşulu | Bu incelemedeki durum |
|---|---|---|
| Yazma başarısız + eski S | Başarı yok; kabul belirsizliği görünür | Başarısız, yeniden üretildi |
| HOME sırasında eski S300 | Referans tamamlanmaz, ARR başlamaz | Başarısız, yeniden üretildi |
| Eksik/bozuk STAT, U2ERR | Bilinmeyen motorlar doğru ayrılır; mevcut F kaybolmaz | Başarısız, yeniden üretildi |
| HOME sırasında MOV/ALL/terminal | Çelişkili hareket bütün girişlerde engellenir | Başarısız, backend'de üretildi |
| İkinci BAŞLA | İkinci hareket gönderilmez | Backend testi geçti; fiziksel çift dokunma bekliyor |
| F veya timeout sonrası | Gözetim sürer; yeni hareket ve alarm yaşam döngüsü kontrollüdür | Başarısız, yeniden üretildi |
| Eski dispatch callback | Yeni işe komut sızmaz | Başarısız, kontrollü timer testi |
| Bozuk yeni STL | Eski hedef yeni ad altında uygulanamaz | Başarısız, kontrollü hata testi |
| USB kopma / yeniden bağlanma | Güncellik ve referans düşer; otomatik hareket tekrarı olmaz | Kopma kodu yetersiz; gerçek USB testi yapılmadı |
| Slave reset / Pi restart | Önce güncel durum uzlaştırılır; referans varsayılmaz | Slave I0 testi başarısız; Pi testi yapılmadı |
| 16 modül + gecikme + canlı poll | Tek seri işlem düzeni, tanımlı en kötü alarm gecikmesi | Hat simülasyonu gerekli |
| Boş, metin, eksi, >600 hedef | Hareket gönderilmez; neden ve düzeltme açıklanır | Backend doğrulaması eksik; Qt giriş olayları sınanmadı |
| Uzun/bozuk seri akış + büyük STL | UI cevap verir; bellek/kuyruk sınırlı | Uzun süreli test yapılmadı |
| Fiziksel durdurma ve enerji dönüşü | Gerçek durdurma kapsamı ve referans kaybı doğrulanır | Donanım değerlendirmesi gerekli |

Demo modu bu matrisi tek başına karşılamaz: dört sahte modül, yaklaşık altı saniye sonra tek tip S üretir; MOV/HOME-motor/ham komut davranışlarını tam simüle etmez. Hata enjeksiyonlu seri mock veya zamanlamalı sahte köprü ile gecikme, kayıp ACK, bozuk satır, kısmi modül, reset ve sıra dışı cevaplar donanımsız sınanabilir.

## Düzeltme sırası

1. Fiziksel durdurma/enerji ayırma eksikliğini ayrı güvenlik işi olarak çözmek; arayüzde durma hakkında doğrulanmamış iddiaları kaldırmak.
2. Eski STAT ile başarı ve HOME geçişini kapatmak; bütün hareket yollarına ortak backend kilidi getirmek.
3. Bağlantı, veri yaşı, referans ve kalıcı alarm modelini düzeltmek; arızada gözetimi sürdürmek.
4. Tek seri işlem yöneticisi, işlem nesli ve kısmi modül yönetimi.
5. Atomik STL yükleme, sabit işlem hedefleri, kapsam/giriş doğrulaması.
6. Türkçe eylem adları, gerçek davranışı açıklayan onaylar, hedef/ölçüm ayrımı ve panel kullanılabilirlik testleri.

İnceleme; tüm QML bileşenlerini, Controller/seri/STL backend'ini, ilgili v5/v5f411 komut ve motor durum akışlarını kapsıyor. Gerçek Qt Controller üzerinde sahte seri bağlantıyla mantık testleri yaptım; tam QML etkileşim testi, Pi performansı, RS-485 elektriksel zamanlaması, motor hareketi, güç devresi ve fiziksel güvenlik doğrulaması yapmadım. Mevcut dokümanlardaki "doğrulandı" ifadelerini bu alanlar için bağımsız kanıt saymadım.
