# STM32F407 Slave PlatformIO Projesi

STM32F407VETX tabanlı slave kartı için PlatformIO projesi. 9 motor kontrolü ve encoder okuma özellikleri içerir.

## Özellikler

- **9 Motor Kontrolü**: 3x3 array halinde 9 adet 600mm Hall effect linear actuator
- **Encoder Mode**: TIM1, TIM2, TIM3, TIM4, TIM5, TIM8 (6 motor)
- **Input Capture (Interrupt)**: TIM9, TIM10, TIM11, TIM12 (3 motor + ekstra)
- **RS485 Haberleşme**: USART2 üzerinden master ile iletişim
- **Flash Memory ID Storage**: Slave ID'yi Flash'ta saklar
- **Komutlar**: PING, SETID, MOV, ALL

## Klasör Yapısı

```
STM32F407_Slave_PlatformIO/
├── platformio.ini      # PlatformIO yapılandırması
├── src/
│   └── main.ino        # Ana kod (Arduino formatı)
├── test_motors.py      # Motor test scripti
└── README.md           # Bu dosya
```

## Kurulum

1. **PlatformIO CLI kurulumu** (eğer yoksa):
   ```bash
   pip install platformio
   ```

2. **Projeyi derle**:
   ```bash
   cd STM32F407_Slave_PlatformIO
   pio run
   ```

3. **Yükle** (ST-Link ile):
   ```bash
   pio run -t upload
   ```

4. **Serial Monitor**:
   ```bash
   pio device monitor
   ```

## Motor Pinout

| Motor | Forward Pin | Reverse Pin | Encoder Timer |
|-------|-------------|-------------|---------------|
| 1     | PE12        | PE13        | TIM1          |
| 2     | PB4         | PB5         | TIM2          |
| 3     | PC4         | PC5         | TIM3          |
| 4     | PD10        | PD11        | TIM4          |
| 5     | PC2         | PC3         | TIM5          |
| 6     | PC8         | PC9         | TIM8          |
| 7     | PE0         | PE1         | TIM9 (IC)     |
| 8     | PB13        | PB12        | TIM10 (IC)    |
| 9     | PB7         | PB6         | TIM11 (IC)    |

## Encoder Yapılandırması

### Encoder Mode (TIM1, TIM2, TIM3, TIM4, TIM5, TIM8)
- **Mod**: Encoder Mode TI1
- **Polarity**: RISING
- **Filter**: 0
- **Prescaler**: DIV1
- **Period**: 
  - TIM1, TIM3, TIM4, TIM8: 65535 (16-bit)
  - TIM2, TIM5: 4294967295 (32-bit)

### Input Capture Mode (TIM9, TIM10, TIM11, TIM12)
- **Mod**: Input Capture
- **Polarity**: RISING
- **Filter**: 0
- **Prescaler**: DIV1
- **Period**: 65535

## Komutlar

### PING
```
PING:01
```
Yanıt: `PONG:01`

### SETID
```
SETID:01
```
Yanıt: `IDSET:01` veya `IDERR:...`

### MOV (Tek Motor)
```
MOV:01:05:300
```
- Slave ID: 01
- Motor ID: 05
- Pozisyon: 300mm

Yanıt: `MOVOK:01:05:300`

### ALL (Tüm Motorlar)
```
ALL:01:300
```
- Slave ID: 01
- Pozisyon: 300mm (tüm motorlar)

Yanıt: `ALLOK:01:300`

## Test Scripti

Motorları test etmek için Python test scripti kullanılabilir:

```bash
python3 test_motors.py
```

Test scripti özellikleri:
- PING testi
- SETID testi
- Tek motor testi
- Tüm motorlar testi
- Sıralı motor testi (1'den 9'a)

## Notlar

1. **Encoder Kalibrasyonu**: Encoder değerlerini mm'ye çevirmek için pulse/mm oranı belirlenmelidir. Bu kalibrasyon yapılmadan pozisyon kontrolü tam çalışmayacaktır.

2. **HAL Framework**: Bu proje STM32 HAL framework kullanır. Timer yapılandırmaları HAL fonksiyonları ile yapılmalıdır.

3. **Flash Memory**: Slave ID Flash'ın son 4 byte'ında saklanır (0x0807FFFC). Magic number: 0xABCD1234.

4. **RS485**: USART2 (PA2/PA3) üzerinden RS485 haberleşme. DE/RE kontrolü PA8 pininden yapılır.

## Geliştirme

### Encoder Okuma
Encoder değerleri interrupt handler'lar veya periyodik okuma ile güncellenir. Encoder Mode timer'lar için counter değeri direkt okunabilir.

### Motor Kontrolü
Motor kontrolü şu anda basit bir yapıdadır. Encoder kalibrasyonu yapıldıktan sonra PID kontrolü veya daha gelişmiş pozisyon kontrolü eklenebilir.

## Sorun Giderme

1. **Derleme Hatası**: PlatformIO'nun güncel olduğundan emin olun: `pio update`
2. **Yükleme Hatası**: ST-Link bağlantısını kontrol edin
3. **Serial Bağlantı Hatası**: COM port numarasını ve baud rate'i kontrol edin
4. **Motor Çalışmıyor**: Pin yapılandırmasını ve encoder bağlantılarını kontrol edin

