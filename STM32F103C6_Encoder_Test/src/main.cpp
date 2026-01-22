#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/* --- AYARLAR --- */
#define ENCODER_UPDATE_INTERVAL_MS 10  // Tepkiyi hızlandırmak için 100ms'den 10ms'ye düşürdüm
#define WAIT_TIME_MS 1000              // Her 30mm sonrası bekleme süresi

// Encoder kalibrasyon (TI12 modu için 4x çözünürlük)
#define BASE_CALIBRATION 79.93f 
#define PULSES_PER_MM (BASE_CALIBRATION *1.0f) 

// Homing ayarları
#define HOMING_STALL_THRESHOLD 5        // Pulse cinsinden eşik (limit switch tespiti)
#define HOMING_CHECK_INTERVAL_MS 250    // Homing kontrol aralığı
#define TARGET_DISTANCE_MM 600          // Hedef mesafe (600mm)
#define POSITION_TOLERANCE 20           // Pulse cinsinden tolerans

/* --- PIN TANIMLARI --- */
// Motor Kontrol Pinleri
#define MOTOR_F_PIN PB4   // Forward
#define MOTOR_R_PIN PB5   // Reverse

// Encoder Pinleri (TIM2)
#define ENCODER_CH1_PIN PA0   // TIM2_CH1
#define ENCODER_CH2_PIN PA1   // TIM2_CH2

// I2C Pinleri (OLED 0.96" 128x64)
#define OLED_SDA PB7   // I2C1 SDA
#define OLED_SCL PB6   // I2C1 SCL

// LED (Status)
#define LED_PIN PC13

// OLED Display (128x64 I2C)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
uint8_t oled_address = SCREEN_ADDRESS;

/* --- GLOBAL DEĞİŞKENLER --- */
// Encoder değerleri (Overflow korumalı)
volatile int32_t totalEncoderCount = 0;  // Toplam encoder sayacı
int16_t lastEncoderRaw = 0;     // Son okunan raw timer değeri
float encoder_speed = 0.0;
int8_t encoder_direction = 0;  // 1=Forward, -1=Reverse, 0=Stop

// Homing değişkenleri
int32_t lastHomingPos = 0;
unsigned long lastHomingCheckTime = 0;
int32_t targetPos = 0;  // Hedef pozisyon (pulse cinsinden)

// Zamanlama
unsigned long last_encoder_read = 0;
unsigned long last_display_update = 0;

// Motor durumu
bool motor_running = false;
bool motor_forward = true;

// Kalibrasyon durumu
enum CalibrationState {
  CALIB_HOME,      // Home'a git (reverse)
  CALIB_RESET,     // Pulse sıfırla
  CALIB_TEST_30MM, // 30mm ilerlet, 1 saniye dur (test)
  CALIB_MOVE_600,  // (Eski koddan kalan, kullanılmıyor ama silinmedi)
  CALIB_DONE       // Tamamlandı
};
CalibrationState calib_state = CALIB_HOME;
int32_t pulse_at_600mm = 0;  // 600mm'deki pulse sayısı

// Test için değişkenler
int32_t test_target_mm = 30;   // Hedef test pozisyonu (30mm ile başlar)
unsigned long test_wait_start_time = 0;  // Bekleme başlangıç zamanı
bool is_waiting_at_step = false;  // Şu an 1 saniyelik bekleme modunda mı?

/* --- FONKSİYON PROTOTİPLERİ --- */
uint8_t Scan_I2C_Devices(void);
void Setup_OLED(void);
void Setup_Motor_Pins(void);
void Setup_Encoder_Timer(void);
float Get_Position_MM(void);
void Read_Encoder(void);
void Reset_Encoder(void);
void Update_OLED_Display(void);
void Motor_Start(bool forward);
void Motor_Stop(void);

/* --- I2C SCANNER --- */
uint8_t Scan_I2C_Devices(void) {
  uint8_t found_address = 0;
  uint8_t device_count = 0;
  
  for(uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    
    if (error == 0) {
      device_count++;
      if (address == 0x3C || address == 0x3D) {
        found_address = address;
      }
    }
  }
  
  if (found_address == 0) {
    return SCREEN_ADDRESS;
  }
  
  return found_address;
}

/* --- OLED SETUP --- */
void Setup_OLED(void) {
  Wire.setSDA(OLED_SDA);
  Wire.setSCL(OLED_SCL);
  Wire.begin();
  
  oled_address = Scan_I2C_Devices();
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, oled_address)) {
    while(true) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }
  
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.display();
}

/* --- MOTOR PIN SETUP --- */
void Setup_Motor_Pins(void) {
  pinMode(MOTOR_F_PIN, OUTPUT);
  pinMode(MOTOR_R_PIN, OUTPUT);
  digitalWrite(MOTOR_F_PIN, LOW);
  digitalWrite(MOTOR_R_PIN, LOW);
}

/* --- ENCODER TIMER SETUP (Register Seviyesinde) --- */
void Setup_Encoder_Timer(void) {
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;  
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;

  GPIOA->CRL &= ~(0xFF);
  GPIOA->CRL |= 0x44;

  TIM2->CR1 = 0;
  TIM2->CNT = 0;
  
  TIM2->CCMR1 = 0; 
  TIM2->CCMR1 |= (1 << 0) | (1 << 8); 
  TIM2->CCMR1 |= (0xF << 4) | (0xF << 12); 
  
  // Polarity ayarı (Gerekirse burayı değiştirebilirsin ama yazılımsal düzelttik)
  TIM2->CCER |= (TIM_CCER_CC1P | TIM_CCER_CC2P);  
  
  TIM2->SMCR &= ~TIM_SMCR_SMS;  
  TIM2->SMCR |= 3;              
  
  TIM2->ARR = 65535;
  TIM2->PSC = 0;
  
  TIM2->CR1 |= TIM_CR1_CEN;
  
  delay(10);
  lastEncoderRaw = (int16_t)TIM2->CNT;
  totalEncoderCount = 0;
}

/* --- ENCODER OKUMA (DÜZELTİLDİ) --- */
void Read_Encoder(void) {
  int16_t current_cnt = (int16_t)TIM2->CNT;
  int16_t diff = current_cnt - lastEncoderRaw;
  
  if (diff != 0) {
    if (diff > 30000) diff -= 65536;
    if (diff < -30000) diff += 65536;
    
    // YÖN DÜZELTME: 
    // Raw count eksi yöne gidiyorsa (diff negatifse), totalEncoderCount azalır.
    // Ancak motor ileri giderken (mutlak değer artmalı).
    // Eğer raw eksiye gidiyorsa, biz bunu eksiyle çarparak (çıkartarak) artıya çeviriyoruz.
    // Şuan kodda: totalEncoderCount -= diff; var. 
    // Eğer diff -5 ise: total -= (-5) => total += 5 olur (ARTAR). 
    // Yani bu mantık DOĞRU olmalı. Eğer hala ters sayıyorsa aşağıdaki satırı "+= diff" yap.
    
    totalEncoderCount -= diff; 
    
    lastEncoderRaw = current_cnt;
  }
  
  // Hız hesapla
  static int32_t last_counter_for_speed = 0;
  static unsigned long last_speed_calc = 0;
  unsigned long now = millis();
  
  if (now - last_speed_calc >= 200) {
    int32_t pulse_diff = totalEncoderCount - last_counter_for_speed;
    float time_diff = (now - last_speed_calc) / 1000.0;
    
    if (time_diff > 0) {
      encoder_speed = (float)pulse_diff / time_diff;
      if (encoder_speed > 10) encoder_direction = 1;
      else if (encoder_speed < -10) encoder_direction = -1;
      else encoder_direction = 0;
    }
    
    last_counter_for_speed = totalEncoderCount;
    last_speed_calc = now;
  }
}

/* --- KONUM OKUMA (MM cinsinden) --- */
float Get_Position_MM(void) {
  // Mutlak değer almayalım, negatifse negatif dönsün ki hatayı görelim
  // Ama ekranda temiz görünsün diye 'Read_Encoder' düzgün çalışmalı.
  return (float)totalEncoderCount / PULSES_PER_MM;
}

/* --- ENCODER SIFIRLA --- */
void Reset_Encoder(void) {
  totalEncoderCount = 0;
  lastEncoderRaw = (int16_t)TIM2->CNT; // Mevcut counter'ı yeni referans al
  // TIM2->CNT = 0; // Donanımsal sayıcıyı sıfırlamaya gerek yok, delta kullanıyoruz
}

/* --- MOTOR KONTROL --- */
void Motor_Start(bool forward) {
  motor_running = true;
  motor_forward = forward;
  
  if (forward) {
    digitalWrite(MOTOR_F_PIN, LOW);   
    digitalWrite(MOTOR_R_PIN, HIGH);  
  } else {
    digitalWrite(MOTOR_F_PIN, HIGH);   
    digitalWrite(MOTOR_R_PIN, LOW);    
  }
}

void Motor_Stop(void) {
  motor_running = false;
  digitalWrite(MOTOR_F_PIN, LOW);
  digitalWrite(MOTOR_R_PIN, LOW);
}

/* --- OLED GÜNCELLEME --- */
void Update_OLED_Display(void) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Encoder Test 3.0"); // Versiyon güncelledim
  
  display.setCursor(80, 0);
  if (motor_running) display.print(motor_forward ? "FWD" : "REV");
  else display.print("STP");
  
  // RAW Timer değeri
  display.setCursor(0, 12);
  display.print("R:"); 
  display.print((int16_t)TIM2->CNT);
  
  // Toplam pulse
  display.setCursor(64, 12);
  display.print("T:"); 
  display.print(totalEncoderCount);
  
  // MM Cinsinden Konum
  display.setTextSize(2); // Büyük font
  display.setCursor(0, 25);
  display.print(Get_Position_MM(), 1); // 1 ondalık
  display.setTextSize(1);
  display.print(" mm");

  // Durum Bilgisi
  display.setCursor(0, 45);
  switch(calib_state) {
    case CALIB_HOME: display.print("Durum: HOMING"); break;
    case CALIB_RESET: display.print("Durum: RESET"); break;
    case CALIB_TEST_30MM: 
      display.print("Hedef: ");
      display.print(test_target_mm);
      display.print("mm");
      if(is_waiting_at_step) display.print(" [W]");
      break;
    case CALIB_DONE: display.print("BITTI. 600mm OK"); break;
    default: break;
  }
  
  // Progress bar
  float currentMM = Get_Position_MM();
  int barWidth = map(currentMM, 0, 600, 0, 128);
  if (barWidth < 0) barWidth = 0;
  if (barWidth > 128) barWidth = 128;
  display.fillRect(0, 58, barWidth, 6, SSD1306_WHITE);
  display.drawRect(0, 58, 128, 6, SSD1306_WHITE);
  
  display.display();
}

/* --- SETUP --- */
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  Setup_Motor_Pins();
  Setup_Encoder_Timer();
  Setup_OLED();
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.print("Sistem Basliyor...");
  display.display();
  delay(1000);
  
  // Kalibrasyon başlat
  calib_state = CALIB_HOME;
  lastHomingCheckTime = millis();
  lastHomingPos = (int16_t)TIM2->CNT;
  Motor_Start(false); // Geriye (Home) git
}

/* --- LOOP (DÜZELTİLMİŞ) --- */
void loop() {
  // 1. Encoder'ı sürekli oku
  Read_Encoder();
  
  unsigned long now = millis();
  
  // 2. Durum Makinesi
  switch(calib_state) {
    
    // --- ADIM 1: HOMING ---
    case CALIB_HOME:
      Motor_Start(false); // Geri git
      
      // Motor durdu mu kontrol et (Limit switch veya mekanik sınır)
      if (now - lastHomingCheckTime >= HOMING_CHECK_INTERVAL_MS) {
        // Hem raw timer hem de total encoder count'u kontrol et
        int16_t currentTimerVal = (int16_t)TIM2->CNT;
        int32_t currentTotal = totalEncoderCount;
        
        // Raw timer değişikliği kontrolü
        int16_t raw_diff = abs(currentTimerVal - (int16_t)lastHomingPos);
        // Total encoder count değişikliği kontrolü (daha güvenilir)
        static int32_t lastHomingTotal = 0;
        int32_t total_diff = abs(currentTotal - lastHomingTotal);
        
        // Eğer hem raw hem total değişmediyse (stall)
        if (raw_diff < HOMING_STALL_THRESHOLD && total_diff < HOMING_STALL_THRESHOLD) {
          Motor_Stop();
          delay(500); // Home'da kısa bir duraklama
          calib_state = CALIB_RESET;
        } else {
          // Hala hareket ediyor, son konumları güncelle
          lastHomingPos = currentTimerVal;
          lastHomingTotal = currentTotal;
          lastHomingCheckTime = now;
        }
      }
      break;
      
    // --- ADIM 2: RESET & HAZIRLIK ---
    case CALIB_RESET:
      Reset_Encoder();
      delay(500);
      test_target_mm = 30; // İlk hedef 30mm
      is_waiting_at_step = false;
      // Buradan TEST moduna geçiyoruz, eski MOVE_600 moduna değil
      calib_state = CALIB_TEST_30MM; 
      break;
      
    // --- ADIM 3: 30MM ADIM TESTİ (Millis ile) ---
    case CALIB_TEST_30MM: {
      
      // Eğer bekleme modundaysak (30mm'ye ulaştı ve bekliyorsa)
      if (is_waiting_at_step) {
        Motor_Stop();
        // 1 saniye (WAIT_TIME_MS) doldu mu?
        if (now - test_wait_start_time >= WAIT_TIME_MS) {
          // Bekleme bitti, yeni hedef belirle
          test_target_mm += 30;
          is_waiting_at_step = false;
          
          // Eğer 600mm'yi geçtiysek bitir
          if (test_target_mm > TARGET_DISTANCE_MM) {
            calib_state = CALIB_DONE;
          }
        }
      } 
      // Eğer hareket halindeysek
      else {
        int32_t target_pulse = (int32_t)(test_target_mm * PULSES_PER_MM);
        int32_t error = target_pulse - totalEncoderCount;
        
        // Tolerans kontrolü
        if (abs(error) <= POSITION_TOLERANCE) {
          // Hedefe ulaştık!
          Motor_Stop();
          is_waiting_at_step = true;
          test_wait_start_time = now; // Zamanı başlat
        } 
        else {
          // Hedefe git
          if (error > 0) {
            Motor_Start(true); // İleri
          } else {
            Motor_Start(false); // Geri (Aşma olursa düzelt)
          }
        }
      }
      break;
    }
    
    // --- ESKİ KOD (Kullanılmıyor ama silinmedi) ---
    case CALIB_MOVE_600:
       // Burayı atladık
      break;
      
    // --- ADIM 4: BITIS ---
    case CALIB_DONE:
      Motor_Stop();
      // Burada LED yakabilirsin veya ekranda bitti yazabilirsin
      break;
  }
  
  // OLED'i güncelle (her 100ms yerine daha sık da olabilir ama 100ms iyidir)
  if (now - last_display_update >= 100) {
    Update_OLED_Display();
    last_display_update = now;
  }
  
  // Ana döngü delay'ini kaldırdım veya çok azalttım ki millis hassas çalışsın
  delay(1); 
}