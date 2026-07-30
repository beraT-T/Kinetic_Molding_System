.pragma library

// Renk paleti (web arayuzune yakin koyu tema)
var bg        = "#0a1628";
var bg2       = "#0c1e3a";
var panel     = "#13243f";
var panelHi   = "#1a2f52";
var border    = "#24395e";
var text      = "#e6edf6";
var textDim   = "#8aa0bd";
var accent    = "#3b82f6";
var accent2   = "#22d3ee";
var green      = "#22c55e";
var orange    = "#f59e0b";
var red        = "#ef4444";
var purple     = "#a855f7";

// ================= DOKUNMATIK OLCU SABITLERI =================
// 15.6" 1080p dokunmatik panel, parmakla kullanim. Material Design min 48px;
// burada eldivenli/kalin parmak icin 56-64px. Hicbir dokunulabilir oge <56px olmamali.
var touchMin    = 56;   // minimum dokunma hedefi
var btnHeight   = 64;   // birincil aksiyon buton yuksekligi
var inputHeight = 56;   // spinbox / metin girisi yuksekligi
var stepperSize = 56;   // spinbox +/- stepper min kenar
var gapMin      = 12;   // etkilesimli ogeler arasi min bosluk
var radius      = 12;   // panel kose yaricapi
var radiusSm    = 8;    // kucuk oge kose yaricapi

// ================= PUNTO (mevcut +~%20) =================
var fsSmall   = 12;   // ipuclari / alt metin
var fsBody    = 14;   // govde
var fsButton  = 16;   // ikincil buton
var fsButtonLg = 18;  // birincil buton
var fsTitle   = 20;   // baslik
var fsValue   = 22;   // spinbox degeri
var fsTip     = 15;   // heatmap etiket (fsBody +~%15)

// Isi haritasi: mm degerini renge cevir (0..600 -> mavi->yesil->kirmizi)
function heat(mm, maxMm) {
    if (maxMm === undefined) maxMm = 600;
    var t = Math.max(0, Math.min(1, mm / maxMm));
    var r, g, b;
    if (t < 0.5) {            // mavi -> yesil
        var u = t / 0.5;
        r = Math.round(30 + u * 20);
        g = Math.round(80 + u * 140);
        b = Math.round(220 - u * 120);
    } else {                  // yesil -> kirmizi
        var u2 = (t - 0.5) / 0.5;
        r = Math.round(50 + u2 * 200);
        g = Math.round(220 - u2 * 150);
        b = Math.round(100 - u2 * 80);
    }
    return Qt.rgba(r / 255, g / 255, b / 255, 1);
}

// durum harfi -> renk (F = ariza, kirmizi ve belirgin)
function stateColor(letter) {
    switch (letter) {
        case "S": return green;
        case "M": return accent;
        case "H": return orange;
        case "F": return red;
        default:  return textDim;   // I
    }
}

// durum harfi -> operator dili (STAT token: <harf><mm>)
function stateLabel(letter) {
    switch (letter) {
        case "S": return "Hazir";
        case "M": return "Hareket";
        case "H": return "Referans";
        case "F": return "ARIZA";
        default:  return "Bosta";   // I
    }
}
