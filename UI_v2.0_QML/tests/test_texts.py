"""Faz 4: operator metinleri. Metin davranisi asamaz: dogrulanmamis iddia yok,
dogru Turkce karakterler, INCELEME metin tablosundaki eski metinler kalmadi."""
import glob
import os
import re
import unittest

from support import QML_DIR, BACKEND


def qml_sources():
    out = {}
    for p in glob.glob(os.path.join(QML_DIR, "*.qml")) + [os.path.join(QML_DIR, "Theme.js")]:
        with open(p, encoding="utf-8") as f:
            out[os.path.basename(p)] = f.read()
    return out


def string_literals(src):
    """Yorum satirlari haric cift tirnakli literaller."""
    lits = []
    for line in src.splitlines():
        code = line.split("//", 1)[0] if not line.strip().startswith("#") else ""
        lits += re.findall(r'"([^"\\]*(?:\\.[^"\\]*)*)"', code)
    return lits


class OldTextsGoneTest(unittest.TestCase):
    FORBIDDEN = ["GUCUNU KESIN", "ONAYLA", "Tumunu Sifirla", "Home Hepsi", "Test 300 mm",
                 "POZISYON HARITASI", "Klavuz", "SERIAL MONITOR", "Canli Durum", "'Hesapla'",
                 "Hazir", "Bosta", "REFERANS ALINMADI - islem", "Yanit vermeyen",
                 "Tum moduller hedef pozisyonda", "Encoder kablosunu ve mekanigi",
                 "Once bir STL yukleyip", "Islem kesildi"]

    def test_forbidden_texts_absent(self):
        lits = []
        for src in qml_sources().values():
            lits += string_literals(src)
        with open(os.path.join(BACKEND, "controller.py"), encoding="utf-8") as f:
            lits += string_literals(f.read())
        joined = "\n".join(lits)
        for t in self.FORBIDDEN:
            self.assertFalse(t in joined, t)

    def test_close_and_cancel_wording(self):
        srcs = qml_sources()
        for name, src in srcs.items():
            lits = string_literals(src)
            self.assertNotIn("Tamam", lits, name)
            self.assertNotIn("Iptal", lits, name)
            self.assertNotIn("İptal", lits, name)
            self.assertNotIn("Yenile", lits, name)
        self.assertNotIn("Temizle", string_literals(srcs["Terminal.qml"]))


class RequiredTextsTest(unittest.TestCase):
    REQUIRED = {
        "BusyOverlay.qml": ["Bu arayüzden hareket durdurulamıyor."],
        "StatusStrip.qml": ["Uyarıyı gördüm"],
        "TesterPage.qml": ["Kılavuz", "Seçili modülün durumunu sorgula", "9 motoru 300 mm'ye götür",
                           "Ölçülen", "Hedef"],
        "Main.qml": ["Bakım / Motor testi", "SİMÜLASYON"],
        "Terminal.qml": ["HABERLEŞME GÜNLÜĞÜ", "Günlüğü temizle"],
        "Header.qml": ["Portları yenile"],
        "MainPage.qml": ["MODEL HEDEFLERİ — ölçülen konum değildir", "Vazgeç"],
        "Surface3D.qml": ["STL Seç ile dosya seçin."],
        "Theme.js": ["Hedefe ulaştı", "Durum bilinmiyor"],
    }

    def test_required(self):
        srcs = qml_sources()
        for name, items in self.REQUIRED.items():
            for t in items:
                self.assertTrue(t in srcs[name], f"{name}: {t}")


class NoUnverifiedClaimsTest(unittest.TestCase):
    """'doğrulandı', 'güvenli', 'durdu' kanit olmadan kullanilmaz."""

    def _operator_strings(self):
        lits = []
        for src in qml_sources().values():
            lits += string_literals(src)
        with open(os.path.join(BACKEND, "controller.py"), encoding="utf-8") as f:
            lits += string_literals(f.read())
        return lits

    def test_no_positive_verification_claims(self):
        for s in self._operator_strings():
            self.assertFalse(re.search(r"doğrulandı\b", s), s[:80])
            self.assertFalse(re.search(r"\bdurdu\b", s), s[:80])

    def test_safe_only_in_negation(self):
        for s in self._operator_strings():
            if "güvenli" in s.lower():
                self.assertRegex(s, r"güvenli erişim anlamına gelmez", s)


class TurkishCharactersTest(unittest.TestCase):
    """Gorunen metinlerde ASCII'lestirilmis yaygin Turkce kelimeler kalmamali."""
    ASCII_WORDS = r"\b(Modul|modul|Baglanti|baglanti|Islem|islem|Gonder|secin|Sec\b|" \
                  r"yukle|Yukle|Satir|Sutun|Gecen|surukle|Surukle|yakinlas|degil|bulunamadi|" \
                  r"Basla\b|BASLA\b|Secili|Kalip|ONIZLEME|MODELI\b)"

    def test_visible_strings(self):
        for name, src in qml_sources().items():
            for s in string_literals(src):
                if s.startswith(("#", "qrc", "file")) or "Theme." in s:
                    continue
                if re.fullmatch(r"[a-z][A-Za-z0-9]*", s):
                    continue        # objectName / tanimlayici, gorunen metin degil
                self.assertFalse(re.search(self.ASCII_WORDS, s), f"{name}: {s[:80]!r}")


if __name__ == "__main__":
    unittest.main()
