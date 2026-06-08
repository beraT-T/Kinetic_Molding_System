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

// durum harfi -> renk
function stateColor(letter) {
    switch (letter) {
        case "S": return green;
        case "M": return accent;
        case "H": return orange;
        case "F": return red;
        default:  return textDim;   // I
    }
}
