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

## 5. İşlem kimliği yok (Bulgu 1, protokol kısmı)

Cevaplarda işlem/sıra numarası olmadığından, zaman aşımına uğramış bir isteğin geç cevabı sonraki
aynı isteğe eşlenebilir. v2.2 azaltması: tek seri yönetici (hatta tek istek), zaman aşımı sonrası
hat koruması, "senkron dışı" + ardışık 2 tutarlı taze STAT (K11). Kalan risk belgelenmiştir
(`tests/test_link.py`).

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
