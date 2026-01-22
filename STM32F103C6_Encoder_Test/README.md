# STM32F103C8 Encoder Test - Motor Test Ünitesi

STM32F103C8 (Blue Pill) tabanlı encoder test projesi. Motor encoder'larını test etmek ve encoder mode'u denemek için tasarlanmıştır.

## Özellikler

- **Encoder Mode Testi**: Timer'ları encoder mode'a ayarlama
- **Encoder Pulse Okuma**: Encoder pulse'larını gerçek zamanlı okuma
- **Motor Kontrolü**: Forward/Reverse motor kontrolü ile encoder testi
- **Serial Monitor**: Encoder değerlerini Serial üzerinden görüntüleme
- **Hız ve Yön Tespiti**: Encoder hızı ve yönü hesaplama

## Klasör Yapısı

```
STM32F103C6_Encoder_Test/
├── platformio.ini      # PlatformIO yapılandırması
├── src/
│   └── main.cpp        # Ana kod
├── test_encoder.py      # Encoder test scripti
└── README.md           # Bu dosya
```

## Kurulum

1. **PlatformIO CLI kurulumu** (eğer yoksa):
   ```bash
   pip install platformio
   ```

2. **Projeyi derle**:
   ```bash
   cd STM32F103C6_Encoder_Test
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

## Pin Yapılandırması

### Motor Kontrol Pinleri
- **Forward**: PB4 (GPIOB, Pin 4)
- **Reverse**: PB5 (GPIOB, Pin 5)

### Encoder Pinleri (TIM2)
- **CH1**: PA0 (GPIOA, Pin 0) - TIM2_CH1
- **CH2**: PA1 (GPIOA, Pin 1) - TIM2_CH2

### I2C1 Pinleri (OLED 0.96")
- **SCL**: PB6 (GPIOB, Pin 6) - I2C1_SCL
- **SDA**: PB7 (GPIOB, Pin 7) - I2C1_SDA

**NOT**: Tüm pinler ayrı - çakışma yok!

### Alternatif Encoder Pinleri (Kullanılmıyor - Referans için)

#### TIM1
- CH1: PA8
- CH2: PA9

#### TIM2 (Kullanılıyor)
- CH1: PA0 (Encoder CH1)
- CH2: PA1 (Encoder CH2)

#### TIM3
- CH1: PA6 (veya PB4 - Motor Forward ile çakışır!)
- CH2: PA7 (veya PB5 - Motor Reverse ile çakışır!)

#### TIM4
- CH1: PB6 (I2C1 SCL ile çakışır!)
- CH2: PB7 (I2C1 SDA ile çakışır!)

### I2C1 Pinleri (OLED 0.96")
- **SCL**: PB6 (GPIOB, Pin 6) - I2C1_SCL
- **SDA**: PB7 (GPIOB, Pin 7) - I2C1_SDA

### LED
- **Status LED**: PC13 (Blue Pill onboard LED)

## Pin Özeti

| Fonksiyon | Pin | GPIO | Not |
|-----------|-----|------|-----|
| Motor Forward | PB4 | GPIOB, Pin 4 | |
| Motor Reverse | PB5 | GPIOB, Pin 5 | |
| Encoder CH1 | PA0 | GPIOA, Pin 0 | TIM2_CH1 |
| Encoder CH2 | PA1 | GPIOA, Pin 1 | TIM2_CH2 |
| I2C1 SCL | PB6 | GPIOB, Pin 6 | OLED ekran |
| I2C1 SDA | PB7 | GPIOB, Pin 7 | OLED ekran |
| LED | PC13 | GPIOC, Pin 13 | Onboard LED |

**Önemli**: Tüm pinler ayrı - çakışma yok!

## Kullanım

### Serial Komutları

- **F** - Motor Forward başlat
- **R** - Motor Reverse başlat
- **S** - Motor durdur
- **T** - Encoder testi (Forward 2s, Reverse 2s)
- **?** - Encoder bilgisi göster
- **H** - Yardım mesajını göster

### Encoder Çıktısı

Encoder değerleri otomatik olarak her 500ms'de bir Serial'e yazdırılır:

```
Encoder: 1234 pulse | Hız: 100.50 pulse/s | Yön: FORWARD | Motor: FORWARD
```

### Test Scripti

Python test scripti ile encoder verilerini görüntüleyebilirsiniz:

```bash
python3 test_encoder.py
```

## Encoder Mode Yapılandırması

### HAL Framework ile

```c
TIM_Encoder_InitTypeDef sConfig = {0};
sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
sConfig.IC1Filter = 0;
sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
sConfig.IC2Filter = 0;

HAL_TIM_Encoder_Init(&htim2, &sConfig);
HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
```

### Encoder Okuma

```c
int32_t encoder_value = __HAL_TIM_GET_COUNTER(&htim2);
```

## Test Senaryoları

### 1. Basit Encoder Testi
1. Motoru Forward başlat (F)
2. Encoder pulse'larını gözlemle
3. Motoru durdur (S)
4. Encoder değerinin sabit kaldığını kontrol et

### 2. Yön Testi
1. Motoru Forward başlat (F)
2. Encoder değerinin arttığını kontrol et
3. Motoru Reverse başlat (R)
4. Encoder değerinin azaldığını kontrol et

### 3. Otomatik Test
1. Test komutunu çalıştır (T)
2. Forward 2 saniye, Reverse 2 saniye otomatik test
3. Encoder değerlerini gözlemle

## Sorun Giderme

### Encoder Değeri Değişmiyor
- Encoder pinlerinin doğru bağlandığından emin olun
- Timer yapılandırmasını kontrol edin
- Encoder mode'un aktif olduğunu kontrol edin

### Motor ve Encoder Pin Çakışması
- Motor pinleri (PA0/PA1) ve encoder pinleri çakışabilir
- Farklı timer/pin kombinasyonları kullanın (TIM1, TIM3, TIM4)

### Serial Bağlantı Sorunu
- COM port numarasını kontrol edin
- Baud rate'in 115200 olduğundan emin olun
- USB kablosunun data kablosu olduğundan emin olun

## Geliştirme Notları

1. **HAL Framework**: Encoder timer yapılandırması HAL framework ile yapılmalıdır
2. **Pin Çakışması**: Motor pinleri ve encoder pinleri çakışmamalıdır
3. **Filter**: Encoder gürültüsü için filter değeri ayarlanabilir
4. **Prescaler**: Encoder hızına göre prescaler ayarlanabilir

## Gelecek Geliştirmeler

- [ ] PID kontrolü ile pozisyon kontrolü
- [ ] Encoder kalibrasyonu (pulse/mm)
- [ ] Çoklu encoder desteği
- [ ] Web arayüzü (WiFi modülü ile)
- [ ] Veri kaydı (SD kart)

