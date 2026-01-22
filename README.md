# Adaptif Kalıp Güç Test Sistemi

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

