# Güvenlik — açık konular (yazılımla çözülemeyenler)

> Bu belge bir güvenlik değerlendirmesi **değildir**. Makine/donanım güvenliği konusunda yetkin
> bir kişinin değerlendirmesi gereken konuları ve arayüzün bu konularda **ne iddia etmediğini**
> listeler. v2.2 arayüzü, doğrulanmamış iddiaları kaldırmak için değiştirildi; eksikliklerin
> kendisini gidermez.

## 1. Fiziksel durdurma ve enerji ayırma (Bulgu 2)

- **Bilinenler:** Protokol v5.1'de (v5 ve v5f411 firmware) host'tan erişilebilir STOP/ABORT
  komutu yok. Firmware'deki `mStopAll()` yalnız açılışta çağrılan iç fonksiyondur. Kullanıcıdan
  gelen bilgiye göre fiziksel acil durdurma (E-stop) yok.
- **Bilinmeyenler (incelenmedi):** motor besleme devresi, kesme elemanı ve yeri, depolanmış
  enerji, enerji kesildiğinde aktüatörlerin mekanik davranışı (geri kayma, yük altında konum),
  enerji dönüşünde motorların davranışı, bu işlemlerin bütün 144 aktüatöre etkisi.
- **Arayüzde ne değişti:** v2.0'daki "DURDURMAK İÇİN CİHAZIN GÜCÜNÜ KESİN" talimatı kaldırıldı
  (hangi enerjinin/anahtarın kastedildiği belirsizdi; Pi/USB/kontrol elektroniğini kapatmak motor
  enerjisini kesmekle karıştırılabilirdi). Yerine yalnız doğru olan yazılır:
  **"Bu arayüzden hareket durdurulamıyor."** — sonrasında talimat **yok**; anahtar adı uydurulmadı.
  Uyarı, herhangi bir hareket komutu yoldayken/kabul edilmişken ve hareket sürüyor ya da
  durgunluğu doğrulanmamışken (Bakım hareketleri dahil) görünür; normal hareket alarm kırmızısı
  kullanmaz.
- **Gerekli:** gerçek durdurma ve enerji ayırma düzeninin, erişilebilirliğinin ve bütün
  aktüatörlere etkisinin yetkin donanım/makine güvenliği değerlendirmesiyle belirlenmesi.
  Bir prosedür tanımlanıp doğrulanınca arayüz metnine **o prosedür** eklenebilir.
- **Gelecekteki yazılımsal durdurma** (öneri: `PROTOKOL_ONERI_v5.2.md` §1) **asla "E-stop" veya
  "acil durdurma" diye adlandırılmayacak**; cevap vermeyen aktüatör için "durdu" denmeyecek.

## 2. Arıza sonrası müdahale

- Sonuç penceresindeki eski "Encoder kablosunu ve mekaniği kontrol edin" yönlendirmesi kaldırıldı:
  diğer motorlar hareket ederken müdahaleye yönlendiriyordu.
- İncelemede önerilen "…doğrulanmış güvenli durdurma/enerji ayırma prosedürünü uygulayın"
  cümlesi **eklenmedi**, çünkü böyle bir prosedür tanımlı ve erişilebilir değil (KARARLAR K26).
- Bugünkü metin: "Motor arızası bildirildi: … Diğer motorların durduğu doğrulanmadı; makine
  izlenmeye devam ediyor."
- Firmware bir motoru `ARIZA` (F) durumundan yalnız yeni hareket komutu veya modülün yeniden
  başlaması ile çıkarır. v2.2'de aktif arıza varken referans alma dahil bütün hareket kilitlidir
  (K15); pratikte kurtarma, modülün enerjisinin yetkin kişi tarafından güvenli biçimde kesilip
  verilmesini gerektirir. **Bu adımın nasıl güvenli yapılacağı tanımlı değildir** — bu belge onu
  tanımlamaz.

## 3. HOME referansının bağımsız doğrulanmaması (Bulgu 12)

Firmware, kalkış payından sonra encoder belirli süre değişmezse sayacı sıfırlar ve "oturdu"
bildirir. Kopuk encoder, enerjisiz motor veya sıkışma da aynı sonucu verebilir. Arayüz bu yüzden
**"Referans alma tamamlandı (firmware bildirimi)"** der; "referans doğrulandı" demez. Bağımsız
doğrulama firmware/donanım işidir (`PROTOKOL_ONERI_v5.2.md` §3).

## 4. Hedefe ulaşmak ≠ güvenli erişim

"Hedefe ulaştı" yalnız firmware'in o motor için `SETTLED` bildirdiği ve konumun ±2 mm içinde
olduğu anlamına gelir. Kalıp alanına güvenli erişim anlamına **gelmez** (REARM ile kendiliğinden
yeniden hareket mümkün; enerji açık). Başarı mesajı bunu açıkça söyler.

## 5. İşlem kimliği yok (Bulgu 1, protokol kısmı; 2026-10-19 inceleme bulgu 2)

Cevaplarda işlem/sıra numarası olmadığından, zaman aşımına uğramış bir isteğin geç cevabı sonraki
aynı isteğe eşlenebilir. İlk v2.2 sürümü bunu "senkron dışı + 2 tutarlı STAT" (K11) ile azalttığını
söylüyordu, ama STAT zaman aşımında bunu **uygulamıyordu**: bağımsız incelemede, HOME sürerken
(motorlar `H`) zaman aşımından sonraki STAT'a eşleşen eski `9×S0` cevabı "Referans alma tamamlandı"
üretti. Düzeltme (`controller._aligned`, `_note_misaligned`, `_poll_next`):

- Zaman aşımı, yazma hatası/takılması veya modül kimliği taşıyan **istenmemiş** cevaptan sonra o
  modülün **cevap sırası belirsiz** sayılır. Sonraki cevaplar veri olarak kullanılmaz (ölçüm,
  tamamlanma, alarm çözme, durgunluk doğrulaması yok); içlerindeki arıza (`F`) yine alarm üretir.
- Gözetim o modüle STAT yerine `PING` gönderir. `PONG` geldiğinde, ondan **önce** gönderilmiş
  isteklerin geç cevaplarının ondan önce gelmiş olması beklenir; ondan sonra istenen STAT veri olarak
  kullanılır. Bu sürede hareket kilitlidir ("cevap sırası yeniden doğrulanıyor").
- Komut kabulü belirsizse (ACK zaman aşımı, `*ERR`, yazma sorunu) ayrıca SYNC kaydı açılır; kayıt,
  sıra yeniden kurulduktan sonra iki ardışık tutarlı ve hareketsiz durumla kapanır (K11).

**Kalan sınır (yazılımla kapatılamaz):** bu bir **çıkarımdır, kanıt değildir.** Dayandığı
varsayımlar: (1) firmware komutları sırayla işler ve cevaptan sonra RS485 girişini boşaltır
(`Adaptif_kalip_v5f411_U3_4motor/src/main.cpp` `loop`/`Send_Response`/`RS485_RX_Mode` — kaynaktan
okundu, **donanımda ölçülmedi**); (2) köprü ve USB tamponu satır sırasını korur; (3) çitten sonra
gelen, gerçekte başka bir isteğe ait ama aynı biçimde bir satır ayırt edilemez. **İki ardışık
benzer cevap tazeliğin kesin kanıtı değildir**; arayüz bu yüzden "doğrulandı" demez. Kesin çözüm:
protokolde istek kimliği (`PROTOKOL_ONERI_v5.2.md` §2). Bilinen sınırı gösteren testler:
`test_link.test_residual_risk_late_reply_matches_next_same_request` (link düzeyi) ve
`test_review_regressions.StaleReplyAfterTimeoutTest` (davranış).

## 6. U2 hatasında U3'ün kendi motorlarını yine zamanlaması (Bulgu 6 eki)

`ALLERR/ARRERR/HOMEERR`, "hiç hareket başlamadı" demek değildir. v2.2 bunları "belirsiz,
izlemeye devam" olarak işler; firmware seçenekleri `PROTOKOL_ONERI_v5.2.md` §4'te.

## 7. Membran komşu yükseklik farkı (Bulgu 9 eki)

Tek parça silikon membranın dayanabileceği komşu motor yükseklik farkı bilinmiyor. v2.2 yalnız
modeldeki en büyük komşu farkını **raporlar**; sınır uygulamaz (K14).

## 8. Doğrulanamayanlar (bu turda hiç ölçülmedi)

Panel okunabilirliği, eldivenli dokunma isabeti, parlama, Pi performansı/bellek, RS-485 elektriksel
zamanlaması, gerçek USB kopma davranışı, motor hareketi, güç devresi. Ayrıntılı test adımları:
`ILERLEME.md` → SABAH ÖZETİ → PANELDE TEST / DONANIMDA TEST GEREKLİ.

## 9. Seri yazmanın takılması (2026-10-19 inceleme bulgu 5)

Yazma tamamlanma bildirimi 1,5 sn içinde gelmezse (`WRITE_DONE_TIMEOUT_MS`; ör. USB sürücüsü veya
`flush()` askıda) komutun modüle ulaşıp ulaşmadığı **bilinemez** ve hareket başlamış olabilir.
Arayüz: isteği "yazma tamamlanmadı" ile bitirir, işlemi sonlandırır, kalan modüllere komut
göndermez, **komutu tekrar etmez**, bağlantıyı düşürür (LINK + SYNC). Gecikmiş/eski oturum
bildirimleri yok sayılır. Yeniden bağlanınca durum okunana kadar hareket kilitlidir. Gerçek USB
askıda kalma davranışı ölçülmedi.

## 10. Kayıp modül ve yeniden başlatma (2026-10-19 inceleme bulgu 3)

Bu oturumda görülmüş bir modül cevap vermezse (bağlantı kesilip yeniden kurulsa da) kapsam
sessizce daraltılmaz: COMM kaydı açılır, bütün hareket kilitli kalır. **Sınır:** bu hafıza
uygulama yeniden başlayınca sıfırlanır (kalıcılık kararı K30, ONAY BEKLİYOR). `config/machine.json`
`expected_modules` doldurulmadan, uygulama yeniden başlatıldıktan sonra eksik bir modülü arayüz
**fark edemez**; yalnız BAŞLA onayında "beklenen liste yapılandırılmadı" uyarısı ve bulunan
modüllerin listesi gösterilir. Gerçek kurulum listesi kullanıcı tarafından yazılmalıdır (arayüz
uydurmaz).

## 11. Testlerin söyledikleri ve söylemedikleri

Donanımsız testlerin (sahte köprü, sahte saat, `loop://` seri, offscreen QML) yeşil olması
**makinenin güvenli olduğu anlamına gelmez.** Testler yalnız arayüzün kendi kurallarını
(yetki, kilit, tamamlanma ölçütü, metin, yerleşim) sahte ortamda doğrular. RS-485 elektriksel
davranışı, firmware'in gerçek zamanlaması, motor/encoder davranışı, enerji kesme ve fiziksel
durdurma, dokunmatik panelde okunabilirlik ve isabet ayrıca doğrulanmalıdır (`DUZELTME_TURU.md`
→ "Fiziksel panelde doğrulanacaklar" ve "Gerçek donanım öncesi açık kararlar").

