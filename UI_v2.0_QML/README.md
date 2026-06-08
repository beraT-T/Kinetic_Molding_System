# UI v2.0 — PySide6 + QML (Raspberry Pi optimize)

Web arayüzünün (Flask + React) native, GPU hızlandırmalı karşılığı. Tek Python süreci:
QML arayüz + seri port + STL interpolasyonu hepsi içeride. **Flask yok, tarayıcı yok** →
Pi 5'te Chromium kiosk'un aksine akıcı (Qt Quick scene-graph'ı GPU ile çizer).

## Özellikler (web arayüzüyle parite)

- Bağlan / port seç / demo mod / ağ tara (PING 1–16, aktif slave takibi)
- STL yükle → 12×12 grid interpolasyon (mevcut `stl_interpolator.py` mantığı)
- 3D önizleme (144 pin yükseklik alanı, orbit/zoom)
- 12×12 ısı haritası + hover bilgisi + istatistik (min/max/ort/std)
- Gönder: bu slave'e (ARR), tüm aktiflere; HOME (slave/motor); STAT ile canlı durum
- Tester sekmesi: 16-slave panosu + 3×3 motor slider grid + Home/Test + seri terminal
- Terminalden ham protokol komutu

Protokol **v5.1**'e hizalı (ARR ile dizi gönderimi, STAT ile durum, HOME hepsi). Eski tekli
`MOV`/`HOME:id:motor` Tester sekmesinde mevcut.

## Klasör yapısı

```
UI_v2.0_QML/
├── main.py                  # giriş noktası
├── requirements.txt
├── backend/
│   ├── serial_manager.py    # pyserial + arka plan okuma thread'i
│   ├── controller.py        # QML köprüsü (slot/signal/property, protokol)
│   └── stl_interpolator.py  # STL -> 12x12 grid (yeniden kullanım)
└── qml/
    ├── Main.qml  Header.qml  MainPage.qml  TesterPage.qml
    ├── Heatmap.qml  Surface3D.qml  Terminal.qml  ActionButton.qml
    └── Theme.js
```

## Kurulum (Raspberry Pi 5, 64-bit)

```bash
sudo apt update
sudo apt install -y python3-pip libgl1-mesa-dri
cd UI_v2.0_QML
pip install -r requirements.txt        # gerekiyorsa: pip install --break-system-packages -r requirements.txt
```

> PySide6 ARM64 (aarch64) tekerlekleri pip'te mevcut. trimesh/scipy ilk kurulumda biraz
> uzun sürebilir. Qt Quick 3D için GPU sürücüsü (Pi 5 varsayılan KMS/V3D) yeterli.

## Çalıştırma

```bash
python3 main.py                 # pencere
python3 main.py --fullscreen    # tam ekran (kiosk)
```

Masaüstü olmadan, doğrudan ekrana (en hafif) çalıştırmak için EGLFS:

```bash
QT_QPA_PLATFORM=eglfs python3 main.py --fullscreen
```

Wayland masaüstünde:

```bash
QT_QPA_PLATFORM=wayland python3 main.py --fullscreen
```

Kontrol stili performans için `Basic`'e ayarlı (kod içinde). Kiosk için otomatik başlatma
istersen systemd servisi ya da `~/.config/wayfire.ini`/autostart ile `python3 main.py --fullscreen`
çağırman yeterli.

## Yeni branch'te tutmak (ui_v2.0)

Bu klasör main'e karışmasın istiyorsan ayrı branch'te tut:

```bash
# proje kökünde
git checkout -b ui_v2.0
git add UI_v2.0_QML
git commit -m "UI v2.0: PySide6 + QML native arayuz (Pi optimize)"
git push -u origin ui_v2.0
```

main'e geri dönmek: `git checkout main`. Branch'ler arası geçişte bu klasör görünüp kaybolur,
bu normal.

## Notlar / sonraki adımlar

- 3D önizleme şu an **interpolasyon sonucu mold yüzeyini** 144 pin olarak gösterir (web'deki
  ham STL mesh'i yerine; bu makinenin yapacağı şekli daha net gösterir). Ham STL mesh
  render'ı istenirse eklenebilir.
- Demo mod donanımsız UI testine izin verir (sahte bağlan/tara/STAT).
- Gerçek donanımda STAT poll'u "Canlı durum" anahtarıyla açılır (Tester sekmesi).
- İlk çalıştırmada bir hata/uyumsuzluk olursa (QML modülü, Quick3D vb.) çıktıyı ilet,
  birlikte ayıklayalım.
