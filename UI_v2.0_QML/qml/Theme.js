.pragma library

// =====================================================================
// ADAPTIF KALIP - HMI STIL KURALLARI (proje stil rehberi)
// ISA-101 / High Performance HMI + IEC 60073 + ISO 9241-303 esas alindi.
// ISA-101 hex deger dayatmaz; kurulusun kendi stil rehberini yazmasini ister.
// Bu dosya o rehberdir. Bilincli sapmalar asagida gerekcesiyle yazilidir.
//
// 1) RENK BUTCESI: normal isletimde ekran NOTR GRI kalir. Renk yalnizca
//    anormal durum icin harcanir ("report by exception").
// 2) SEMANTIK (IEC 60073 - GOSTERGELER icin):
//       kirmizi (alarm) = ariza / tehlike, hemen mudahale
//       amber  (warn)   = anormal, on-kosul eksik (referans yok, baglanti yok)
//       notr            = normal (yesil DEKORATIF olarak kullanilmaz)
// 3) ISTISNA (IEC 60204-1 - AKTUATORLER icin): start/baslat BUTONU yesil
//    olabilir. "BASLA" butonunun yesili STANDARDA UYGUNDUR, notrlenmemeli.
// 4) VERI rengi != DURUM rengi. Isi haritasi bir veri gosterimidir; rampasi
//    algisal duzgun ve KIRMIZISIZ (cividis) secildi ki alarm kirmizisiyla
//    karismasin.
// 5) SAPMA: ISA-101 acik-orta gri arka plan onerir (7/24 kontrol odasi,
//    parlama). Bu panel atolyede, kisa sureli kullanimda ve koyu ortamda;
//    koyu tema korundu, ama DOYGUNLUK sifirlandi (eski lacivert #0a1628 ->
//    notr gri). Asil kural "doygun olmasin", "acik olsun" degil.
// =====================================================================

// ---------- notr govde (doygunlugu sifir) ----------
var bg        = "#1b1e22";   // uygulama zemini
var bg2       = "#22262b";   // sekme cubugu / ikincil zemin
var panel     = "#2a2f36";   // panel yuzeyi
var panelHi   = "#343a43";   // yukseltilmis yuzey / girinti
var border    = "#454c56";
var text      = "#e8eaed";
var textDim   = "#9aa0a6";
var textFaint = "#6b7178";

// ---------- semantik durum renkleri (GOSTERGE) ----------
var alarm     = "#e5484d";   // ariza / tehlike - SADECE gercek ariza
var warn      = "#e8a33d";   // anormal / on-kosul eksik
var normal    = "#9aa0a6";   // normal durum = notr (yesil DEGIL)
var selection = "#4a6fa5";   // secim/odak - dusuk doygunluklu mavi

// ---------- aktuator (buton) renkleri - IEC 60204-1 ----------
var actionStart   = "#2f7d4f";  // BASLA (start aktuatoru - yesil serbest)
var actionPrimary = "#3d5a80";  // birincil ama tehlikesiz islem
var actionNeutral = "#3b424b";  // ikincil / notr buton
var actionWarn    = "#8a6420";  // dikkat isteyen islem (home vb.)

// ---------- eski isimler (geriye donuk uyum; yeni kodda KULLANMA) ----------
var accent  = actionPrimary;
var accent2 = text;
var green   = actionStart;
var orange  = warn;
var red     = alarm;
var purple  = actionNeutral;

// ---------- tipografi ----------
// ISO 9241-303 / ANSI-HFES 100: min 16 yay-dakikasi, hedef 20-22'.
// 15.6" 1080p (5.56 px/mm), 500-700 mm bakis -> asagidaki degerler.
// NOT: bu QML surumunde font.families YOK (yalniz font.family). Govde metni
// icin sistem varsayilani birakilir (Pi'de DejaVu Sans, Mac'te Helvetica);
// sayisal alanlarda generic "monospace" kullanilir (sutun hizasi -> hizli okuma).
var fontMono = "monospace";

var fsSmall    = 16;   // alt metin / ipucu (mutlak taban)
var fsBody     = 20;   // govde
var fsButton   = 22;   // ikincil buton
var fsButtonLg = 26;   // birincil buton
var fsTitle    = 28;   // baslik
var fsValue    = 34;   // uzaktan okunacak sayi
var fsTip      = 20;   // harita etiketi
var fsAlarm    = 24;   // alarm seridi

// ---------- dokunma olculeri ----------
// Endustriyel (ince eldiven) hedef ~15 mm = 84 px. Operator ekraninda bu
// hedeflenir; BAKIM (Tester) ekraninda yogunluk icin touchDense kullanilir
// (bilincli sapma: Seviye-4 teknisyen ekrani, cift elle ve yakindan kullanilir).
var touchMin    = 80;   // ~14.4 mm - genel dokunma hedefi
var touchDense  = 64;   // ~11.5 mm - yalniz bakim/Tester ekrani
var btnHeight   = 92;   // birincil aksiyon butonu
var btnHeightXl = 132;  // ~24 mm - tehlikeli hareketi baslatan tek kontrol
var inputHeight = 80;
var stepperSize = 80;
var gapMin      = 16;   // ~2.9 mm - ANSI-HFES 3.2 mm'ye yakin
var radius      = 6;    // endustriyel: keskin, dekoratif yuvarlaklik yok
var radiusSm    = 4;
var stripHeight = 52;   // alarm/durum seridi

// =====================================================================
// ISI HARITASI RAMPASI - cividis (algisal duzgun, renk koru uyumlu,
// KIRMIZI ICERMEZ -> alarm kirmizisiyla cakismaz).
// Rainbow/jet rampalar veride olmayan sahte kenarlar uretir (Crameri 2020).
// =====================================================================
var _cividis = [
    [  0,  32,  76], [  0,  53, 108], [ 34,  72, 107], [ 66,  90, 110],
    [ 99, 107, 113], [134, 125, 111], [173, 145, 103], [215, 166,  86],
    [255, 233,  69]
];

function heat(mm, maxMm) {
    if (maxMm === undefined) maxMm = 600;
    var t = Math.max(0, Math.min(1, mm / maxMm));
    var n = _cividis.length - 1;
    var f = t * n;
    var i = Math.min(n - 1, Math.floor(f));
    var u = f - i;
    var a = _cividis[i], b = _cividis[i + 1];
    return Qt.rgba((a[0] + (b[0] - a[0]) * u) / 255,
                   (a[1] + (b[1] - a[1]) * u) / 255,
                   (a[2] + (b[2] - a[2]) * u) / 255, 1);
}

// durum harfi -> renk. Normal durumlar NOTR; yalniz ariza renklidir.
function stateColor(letter) {
    switch (letter) {
        case "F": return alarm;    // ariza
        case "H": return warn;     // referans aliyor (gecici anormal sayilir)
        case "M": return textDim;  // hareket - normal
        case "S": return normal;   // hazir - normal (yesil DEGIL)
        default:  return textFaint;
    }
}

// durum harfi -> operator dili
function stateLabel(letter) {
    switch (letter) {
        case "S": return "Hazir";
        case "M": return "Hareket";
        case "H": return "Referans";
        case "F": return "ARIZA";
        default:  return "Bosta";
    }
}

// saniye -> "mm:ss"
function clock(sec) {
    if (sec === undefined || sec < 0) return "--:--";
    var m = Math.floor(sec / 60), s = Math.floor(sec % 60);
    return (m < 10 ? "0" : "") + m + ":" + (s < 10 ? "0" : "") + s;
}
