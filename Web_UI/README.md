# Adaptif Kalıp Güç Test Sistemi-Donanım Tarafı

## Proje Genel Bakış

Bu proje, **12x12 = 144 noktalı** adaptif kalıp yüzey kontrol sistemi için tasarlanmıştır. Sistem, bilgisayardan gelen pozisyon verilerini alarak, 16 adet slave modül üzerinden toplam 144 adet lineer aktüatörü kontrol eder.

## Sistem Mimarisi

### 1. Master (Blue Pill - STM32F103C6)
- **Görev**: Bilgisayar ↔ RS485 köprü görevi
- **Bağlantı**: 
  - USB Serial (115200 baud) ↔ Bilgisayar
  - RS485 (9600 baud) ↔ 16 Slave
- **Fonksiyon**: 
  - Bilgisayardan gelen 12x12 pozisyon array'ini alır
  - Array'i 16 parçaya böler (her parça 3x3)
  - Her parçayı ilgili slave'e gönderir
  - Slave'lerden gelen yanıtları bilgisayara iletir

### 2. Slave Modüller (16 Adet - STM32F407VETX)
- **Her Slave**: 3x3 = 9 adet motor kontrolü
- **Toplam**: 16 slave × 9 motor = 144 motor
- **Motor Tipi**: 600mm Hall Effect'li Lineer Aktüatör
- **Bağlantı**: RS485 üzerinden master ile haberleşir
- **Fonksiyon**:
  - Master'dan 3x3 pozisyon array'i alır
  - Her motoru ilgili pozisyona sürer
  - Encoder okuma ve pozisyon kontrolü yapar

### 3. Bilgisayar Arayüzü (Python UI)
- **Görev**: Test ve kontrol arayüzü
- **Özellikler**:
  - 16 slave'in durumunu görselleştirme (4x4 grid)
  - Ağ taraması (PING/PONG)
  - Motor kontrolü (slider'lar)
  - Terminal/log görüntüleme

## Veri Akışı

```
Bilgisayar (12x12 Array)
    ↓
Master (Blue Pill)
    ↓ (Array'i 16 parçaya böler)
16 x Slave (Her biri 3x3 array alır)
    ↓
144 Motor (Her motor kendi pozisyonuna gider)
```

## Array Dağıtım Mantığı

### Giriş: 12x12 Pozisyon Array'i
```
[P00, P01, P02, ..., P0,11]
[P10, P11, P12, ..., P1,11]
...
[P11,0, P11,1, ..., P11,11]
```

### Çıkış: 16 Adet 3x3 Array (Her Slave'e)
- **Slave 1**: [P00, P01, P02] [P10, P11, P12] [P20, P21, P22]
- **Slave 2**: [P03, P04, P05] [P13, P14, P15] [P23, P24, P25]
- **Slave 3**: [P06, P07, P08] [P16, P17, P18] [P26, P27, P28]
- **Slave 4**: [P09, P0A, P0B] [P19, P1A, P1B] [P29, P2A, P2B]
- ... (devam eder)
- **Slave 16**: [P99, P9A, P9B] [PA9, PAA, PAB] [PB9, PBA, PBB]

**Not**: Array indeksleme 0'dan başlar. Her slave 3x3 = 9 pozisyon alır.

## Protokol

### Master ↔ Slave Haberleşme (RS485 - 9600 baud)

#### PING/PONG (Haberleşme Testi)
- **Gönder**: `PING:01\n` (Slave ID 1'e ping)
- **Yanıt**: `PONG:01\n`

#### Slave ID Ayarlama (Flash Memory'de Saklanır)
- **Komut**: `SETID:ID\n` (Broadcast, tüm slave'ler dinler)
  - Örnek: `SETID:05\n` (Slave ID'yi 5 yap)
  - **Yanıt**: `IDSET:05\n` (Başarılı) veya `IDERR:...\n` (Hata)
  - **Not**: ID Flash memory'de saklanır, power cycle'dan sonra da kalır
  - **Geçerli Aralık**: 1-16

#### Pozisyon Komutları
- **Tek Motor**: `MOV:SlaveID:MotorID:Value\n`
  - Örnek: `MOV:01:05:300\n` (Slave 1, Motor 5, 300mm)
  
- **Tüm Motorlar**: `ALL:SlaveID:Value\n`
  - Örnek: `ALL:01:300\n` (Slave 1, tüm motorlar 300mm)

#### 3x3 Array Gönderimi (İleride eklenecek)
- **Format**: `ARR:SlaveID:P00:P01:P02:P10:P11:P12:P20:P21:P22\n`
  - Örnek: `ARR:01:100:150:200:120:170:220:140:190:240\n`

## Donanım Özellikleri

### Master (Blue Pill)
- **MCU**: STM32F103C6
- **RS485**: PA2/PA3, DE/RE: PA4
- **USB Serial**: Virtual COM Port

### Slave (STM32F407)
- **MCU**: STM32F407VETX
- **RS485**: USART2 (PA2/PA3), DE/RE: PA8
- **Motor Kontrol**: 9 adet Forward/Reverse pin çifti
- **Encoder**: 9 adet (TIM1-5, TIM8-12)
- **LED**: PE4 (Durum göstergesi)

### Motor
- **Tip**: Hall Effect'li Lineer Aktüatör
- **Menzil**: 0-600mm
- **Kontrol**: Forward/Reverse pin çifti ile

## Klasör Yapısı

```
adaptif_kalip_guc_test/
├── README.md (Bu dosya)
├── Adaptif_Kalip_blue_pill_rs485_test/  (Master - PlatformIO)
│   ├── src/main.cpp
│   └── Tester_UI/ui_v1.0.py
├── Core/                                 (Slave - STM32CubeIDE)
│   ├── Inc/main.h
│   └── Src/main.c
└── Drivers/                              (HAL Driver)
```

## Geliştirme Durumu

### ✅ Tamamlanan
- Master-Slave RS485 haberleşme altyapısı
- PING/PONG test protokolü
- Python UI temel arayüzü
- Slave donanım konfigürasyonu (motor pinleri, encoder timer'ları)
- **Dinamik Slave ID sistemi (Flash memory'de saklama)**
- **SETID protokolü (Master üzerinden ID atama)**

### 🚧 Yapılacaklar
- [ ] Master: 12x12 array'i alma ve 16 parçaya bölme algoritması
- [ ] Master: Array parçalarını slave'lere gönderme protokolü
- [ ] Slave: 3x3 array alma ve parse etme
- [ ] Slave: Motor kontrol algoritması (PID/pozisyon kontrolü)
- [ ] Slave: Encoder okuma ve pozisyon geri bildirimi
- [ ] Python UI: 12x12 array girişi ve görselleştirme

## Kullanım Senaryosu

1. **Başlangıç**:
   - Bilgisayar 12x12 pozisyon array'i hazırlar (mm cinsinden integer değerler)
   - Master'a array gönderilir

2. **Dağıtım**:
   - Master array'i 16 parçaya böler
   - Her parça (3x3) ilgili slave'e gönderilir

3. **Kontrol**:
   - Her slave kendi 3x3 array'ini alır
   - 9 motoru sırayla hedef pozisyonlara sürer
   - Encoder ile pozisyon kontrolü yapar

4. **Geri Bildirim**:
   - Slave'ler pozisyon durumunu master'a bildirir
   - Master bilgisayara durum raporu gönderir

## Notlar

- Tüm sistemde RS485 baud rate **9600** olmalıdır
- Slave ID'ler 1-16 arası olmalıdır (Flash memory'de saklanır)
- Motor pozisyonları 0-600mm arası olmalıdır
- Array indeksleme 0'dan başlar (0-11 arası)
- **Slave ID Flash Memory**: Son sector (Sector 11) kullanılır
- **ID Atama**: `SETID:XX` komutu ile master üzerinden yapılır
- **ID Kalıcılık**: Power cycle'dan sonra da korunur

## Lisans

Bu proje özel bir endüstriyel uygulama için geliştirilmiştir.


# 🎯 Adaptif Kalıp Kontrol Sistemi - Web Arayüzü-Yazılım Tarafı

Modern web tabanlı kontrol sistemi: **React + Flask + WebSocket**

> 144 Aktüatör (16 Slave × 9 Motor) kontrolü için STL tabanlı pozisyon hesaplama ve gerçek zamanlı izleme

---

## 📋 Gereksinimler

### Sistem Gereksinimleri
- **Python**: 3.8 veya üzeri
- **Node.js**: 18.0 veya üzeri
- **npm**: 9.0 veya üzeri
- **İşletim Sistemi**: Windows/Linux/macOS

### Donanım (Gerçek Test İçin)
- Master STM32 (USB-Serial dönüştürücü ile)
- En az 1 Slave (RS485 bağlantılı)
- USB kablosu

---

## 🚀 Kurulum

### 1️⃣ Projeyi İndir

```bash
# Git ile klonla (veya ZIP indir)
git clone https://github.com/your-repo/Kinetic_Molding_System.git
cd Kinetic_Molding_System/Web_UI
```

### 2️⃣ Backend Kurulumu (Python/Flask)

```bash
# Backend klasörüne git
cd backend

# Virtual environment oluştur (önerilen)
python -m venv venv

# Virtual environment'ı aktifleştir
# Windows:
venv\Scripts\activate
# Linux/Mac:
source venv/bin/activate

# Gerekli paketleri yükle
pip install -r requirements.txt
```

### 3️⃣ Frontend Kurulumu (React/Vite)

**Yeni terminal aç** ve:

```bash
# Frontend klasörüne git
cd frontend

# Node paketlerini yükle
npm install
```

---

## ▶️ Çalıştırma

### Backend'i Başlat

```bash
cd backend
python app.py
```

**Çıktı:**
```
Backend başlatılıyor: http://localhost:5000
 * Running on http://127.0.0.1:5000
 * Debugger is active!
```

### Frontend'i Başlat

**Yeni terminal aç:**

```bash
cd frontend
npm run dev
```

**Çıktı:**
```
  VITE v5.4.21  ready in 234 ms
  
  ➜  Local:   http://localhost:3000/
  ➜  Network: use --host to expose
```

### Tarayıcıda Aç

`http://localhost:3000` adresine git

---

## 🎮 Demo Mode vs Gerçek Donanım

### 🎭 Demo Mode (Varsayılan)

**Donanım olmadan test için!**

- Sahte Master bağlantısı
- 144 MOV komutu terminal'de loglanır
- Sahte PONG yanıtları
- Slave tarama simülasyonu

**Kullanım:**
1. Backend ve frontend'i başlat
2. "Bağlan (Demo)" butonuna tıkla
3. STL yükle, hesapla, gönder

### 🔌 Gerçek Donanım Modu

**Backend'de değişiklik:**

`backend/app.py` dosyasını aç:

```python
# Satır 15-16
DEMO_MODE = False  # ← True'dan False'a değiştir
```

**Gereksinimler:**
- Master USB'ye bağlı
- En az 1 slave bağlı
- Doğru COM port seçili

**Kullanım:**
1. Master'ı tak
2. Backend'i yeniden başlat
3. COM port seç (dropdown'dan)
4. "Bağlan" tıkla
5. STL yükle → gönder

---

## 📖 Kullanım Kılavuzu

### Adım 1: Master'a Bağlan

**Demo Mode:**
- "Bağlan (Demo)" butonuna tıkla
- Hazır! ✅

**Gerçek Mod:**
1. COM port seç (dropdown)
2. 🔍 ile portları tara (yeni port bağlandıysa)
3. "Bağlan" tıkla
4. Yeşil "Bağlı" göreceksin

### Adım 2: Slave'leri Tara (Opsiyonel)

- 🌐 "Ağı Tara" butonuna tıkla
- Terminal'de PING/PONG mesajları
- Aktif slave sayısı gösterilir (ör. "5/16")

### Adım 3: STL Dosyası Yükle

1. "STL Yükle" alanına dosya sürükle
2. Veya tıklayıp dosya seç
3. Otomatik olarak 12×12 grid hesaplanır
4. İstatistikler gösterilir:
   - **Min:** En düşük Z pozisyonu
   - **Ort:** Ortalama Z pozisyonu
   - **Max:** En yüksek Z pozisyonu

### Adım 4: Heatmap'i İncele

- **12×12 ızgara** renkli gösterilir
- Turuncu/kırmızı = Yüksek pozisyon
- Mavi/yeşil = Düşük pozisyon
- Üzerine gel → Pozisyon değeri göster

### Adım 5: Master'a Gönder

1. "Pozisyon verilerini Master'a Gönder" butonuna tıkla
2. Terminal'de **144 MOV komutu** akacak:
   ```
   → MOV:01:01:300
   → MOV:01:02:305
   ...
   ← PONG:01:OK
   ```
3. Gerçek modda: **Aktüatörler hareket edecek!**

---

## 🎨 Arayüz Özellikleri

### Header (Üst Bar)
- **🎭 Demo Mode** göstergesi
- **COM Port** seçici (gerçek modda)
- **🔍 Port Tara** butonu
- **🌐 Ağı Tara** butonu (slave keşfi)
- **Bağlan/Bağlı** durumu
- **🔄 Sıfırla** butonu

### Sol Panel
- **STL Yükleme** (drag & drop)
- **3D Önizleme** (Three.js)

### Orta Panel
- **Heatmap** (12×12 grid)
- **Min/Ort/Max** istatistikler
- **Gönder** butonu

### Sağ Panel (Terminal
)
- **Gerçek zamanlı log** (WebSocket)
- **PING/PONG** mesajları
- **MOV** komutları

---

## 🐛 Sorun Giderme

### Backend Başlamıyor

```bash
# Virtual environment aktif mi kontrol et
pip list | grep flask

# Paketleri yeniden yükle
pip install -r requirements.txt --force-reinstall
```

### Frontend Başlamıyor

```bash
# Node modüllerini temizle
rm -rf node_modules package-lock.json
npm install

# Port 3000 kullanımda mı? Başka port kullan:
npm run dev -- --port 3001
```

### Port Bulunamıyor

- Sürücü kurulu mu kontrol et (CP210x, FTDI, vb.)
- Device Manager'da COM port görünüyor mu?
- 🔍 butonu ile portları yeniden tara

### "Bağlantı yok" Hatası

- Backend çalışıyor mu? (`http://localhost:5000` kontrol et)
- `DEMO_MODE = True` ise port seçmeye gerek yok
- `DEMO_MODE = False` ise COM port seçilmiş mi?

---

## 📊 Teknik Detaylar

### Backend (Flask)
- **Port:** 5000
- **Framework:** Flask 3.0
- **Real-time:** Flask-SocketIO
- **Serial:** PySerial
- **STL:** Trimesh + NumPy

### Frontend (React)
- **Port:** 3000
- **Framework:** React 18 + Vite
- **Styling:** Tailwind CSS
- **Icons:** Lucide React
- **3D:** Three.js
- **Real-time:** Socket.IO Client

### API Endpoints
```
GET  /api/ports          → Mevcut COM portları
POST /api/connect        → Serial bağlantı kur
POST /api/disconnect     → Bağlantıyı kes
POST /api/scan           → Slave'leri tara
POST /api/upload-stl     → STL yükle ve hesapla
POST /api/send-to-master → 144 MOV gönder
```

---

## 🔒 Güvenlik Notları

- **CORS:** Development için açık (`cors_allowed_origins="*"`)
- **Production'da:** CORS'u sınırla
- **USB Sürücüler:** Güvenli kaynaklardan indir

---

## 📞 Destek

Sorun yaşarsan:
1. Terminal loglarını kontrol et
2. Browser console'a bak (F12)
3. Demo mode ile test et
4. Issue aç veya ekibe ulaş

---

## ✅ Hızlı Başlangıç Checklist

- [ ] Python 3.8+ kurulu
- [ ] Node.js 18+ kurulu
- [ ] Backend paketleri yüklendi (`pip install -r requirements.txt`)
- [ ] Frontend paketleri yüklendi (`npm install`)
- [ ] Backend çalışıyor (`python app.py`)
- [ ] Frontend çalışıyor (`npm run dev`)
- [ ] Tarayıcıda açıldı (`http://localhost:3000`)
- [ ] Demo mode test edildi
- [ ] (Opsiyonel) Gerçek donanımla test edildi

---

**🎉 Sistem hazır! İyi çalışmalar!**
