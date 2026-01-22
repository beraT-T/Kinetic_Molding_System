/**
 * STM32F407VETX Slave - Motor Control Test
 * PlatformIO Project - Arduino Format (.ino)
 * 
 * Özellikler:
 * - 9 Motor Kontrolü (3x3 Array)
 * - Encoder Mode: TIM1, TIM2, TIM3, TIM4, TIM5, TIM8
 * - Input Capture (Interrupt): TIM9, TIM10, TIM11, TIM12
 * - RS485 Haberleşme (USART2)
 * - Flash Memory ID Storage
 * - Komutlar: PING, SETID, MOV, ALL
 */

/**
 * STM32F407VETX Slave - Motor Control Test
 * PlatformIO Project - HAL Framework
 * 
 * NOT: Bu dosya .ino uzantılıdır ama HAL framework kullanır.
 * PlatformIO HAL framework ile çalışır.
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// HAL Framework için gerekli include'lar
// PlatformIO HAL framework kullanıldığında bu include'lar otomatik gelir
#ifdef HAL_TIM_MODULE_ENABLED
  // HAL Timer kütüphanesi aktif
#endif

// HAL Flash kütüphanesi için include'lar
// PlatformIO HAL framework'ünde bu fonksiyonlar genellikle otomatik gelir
// Ancak explicit include eklemek daha güvenli
#ifdef HAL_FLASH_MODULE_ENABLED
  // HAL Flash kütüphanesi aktif
  // HAL_FLASH_Unlock, HAL_FLASH_Lock, HAL_FLASH_Program, HAL_FLASHEx_Erase
  // FLASH_EraseInitTypeDef, FLASH_SECTOR_7, FLASH_VOLTAGE_RANGE_3
  // FLASH_TYPEPROGRAM_WORD gibi tanımlar HAL framework'ünde mevcut
#endif

/* --- AYARLAR --- */
#define LINE_BUFFER_LEN 64
#define SLAVE_ID_FLASH_ADDRESS 0x0807FFFC  /* Last 4 bytes of Flash */
#define SLAVE_ID_MAGIC         0xABCD1234   /* Magic number to verify valid ID */
#define SLAVE_ID_DEFAULT       0            /* Default ID if Flash is empty */

/* --- HARDWARE TEST MODU --- */
#define HARDWARE_TEST_MODE 0  // 1 = Test modu (encoder yok, basit motor kontrol), 0 = Normal mod

/* --- HOMING AYARLARI --- */
#define HOMING_STALL_THRESHOLD 5        // Pulse cinsinden eşik (limit switch tespiti)
#define HOMING_CHECK_INTERVAL_MS 250    // Homing kontrol aralığı

/* --- PIN TANIMLARI --- */
// Motor Pinleri
#define MOTOR1_F_PIN PE12
#define MOTOR1_R_PIN PE13
#define MOTOR2_F_PIN PB4
#define MOTOR2_R_PIN PB5
#define MOTOR3_F_PIN PC4
#define MOTOR3_R_PIN PC5
#define MOTOR4_F_PIN PD10
#define MOTOR4_R_PIN PD11
#define MOTOR5_F_PIN PC2
#define MOTOR5_R_PIN PC3
#define MOTOR6_F_PIN PC8
#define MOTOR6_R_PIN PC9
#define MOTOR7_F_PIN PE0   // Motor7 Forward
#define MOTOR7_R_PIN PE1   // Motor7 Reverse
#define MOTOR8_F_PIN PB13
#define MOTOR8_R_PIN PB12
#define MOTOR9_F_PIN PB7   // Motor9 Forward
#define MOTOR9_R_PIN PB6   // Motor9 Reverse

// RS485
#define RS485_DE_RE_PIN PA8

// LED
#define LED_PIN PE4

// USART2 (RS485)
HardwareSerial SerialRS485(PA3, PA2); // RX, TX

/* --- GLOBAL DEĞİŞKENLER --- */
char rx_buffer[LINE_BUFFER_LEN];
uint8_t rx_index = 0;
uint8_t slave_id = SLAVE_ID_DEFAULT;

/* Motor pozisyonları (0-600mm, -1 = hedef set edilmemiş) */
int16_t motor_positions[9] = {-1, -1, -1, -1, -1, -1, -1, -1, -1};

/* Homing durumu (0 = homing yok, 1 = homing yapılıyor) */
uint8_t motor_homing[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
int32_t last_homing_pos[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
unsigned long last_homing_check_time[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

/* Encoder counter değerleri (overflow korumalı) */
volatile int32_t encoder_counters[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
int32_t last_encoder_raw[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; // Son okunan raw timer değerleri (Motor 2 ve 5 için int32_t, diğerleri int16_t)

/* Motor 9 için yön tespiti (TIM10 ve TIM11 birlikte) */
volatile uint8_t motor9_last_interrupt = 0; // 0=hiç, 1=TIM10, 2=TIM11

/* Encoder kalibrasyon (STM32F103 test kodundan - TI12 modu için 4x çözünürlük) */
#define BASE_CALIBRATION 79.93f 
#define PULSES_PER_MM (BASE_CALIBRATION * 1.0f)

/* --- FONKSİYON PROTOTİPLERİ --- */
void RS485_TX_Mode(void);
void RS485_RX_Mode(void);
void Send_Response(const char* msg);
void Process_Packet(char* line);
bool ReadLine_NonBlocking(char *dst, uint8_t *pos, size_t max_len);
uint8_t Flash_Read_SlaveID(void);
bool Flash_Write_SlaveID(uint8_t id);
void Motor_Control(uint8_t motor_id, int16_t target_position);
void Motor_Stop(uint8_t motor_id);
int32_t Encoder_Read(uint8_t motor_id);
void Read_Encoder(uint8_t motor_id);
void Reset_Encoder(uint8_t motor_id);
void Setup_Encoders(void);
void Setup_InputCapture(void);
void Setup_Encoder_Timer(uint8_t timer_num);
void Motor_Home(uint8_t motor_id);
void Process_Homing(void);
void Send_Debug(const char* msg);

/* --- ENCODER INTERRUPT HANDLERS --- */
// NOT: Bu interrupt handler'lar HAL framework ile yapılandırılmalıdır.
// PlatformIO HAL framework kullanıldığında bu handler'lar HAL tarafından otomatik çağrılır.
// Şimdilik placeholder - HAL timer yapılandırması gerekli.

// TIM1 Encoder (Motor 1) - Encoder Mode
// Counter değeri direkt okunur, interrupt gerekmez

// TIM2 Encoder (Motor 2) - 32-bit Encoder Mode
// Counter değeri direkt okunur

// TIM3 Encoder (Motor 3) - Encoder Mode
// Counter değeri direkt okunur

// TIM4 Encoder (Motor 4) - Encoder Mode
// Counter değeri direkt okunur

// TIM5 Encoder (Motor 5) - 32-bit Encoder Mode
// Counter değeri direkt okunur

// TIM8 Encoder (Motor 6) - Encoder Mode
// Counter değeri direkt okunur

// TIM9 Input Capture (Motor 7) - Interrupt Mode
// Input Capture interrupt ile pulse sayılır

// TIM10 Input Capture (Motor 8) - Interrupt Mode
// Input Capture interrupt ile pulse sayılır

// TIM11 Input Capture (Motor 9) - Interrupt Mode
// Input Capture interrupt ile pulse sayılır

/* --- SETUP --- */
void setup() {
  // Serial başlat (USB - Debug için)
  Serial.begin(115200);
  while (!Serial) {
    ; // USB Serial bağlantısını bekle
  }
  
  // RS485 Serial başlat
  SerialRS485.begin(9600);
  
  // Pin ayarları
  pinMode(RS485_DE_RE_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  // Motor pinleri
  pinMode(MOTOR1_F_PIN, OUTPUT);
  pinMode(MOTOR1_R_PIN, OUTPUT);
  pinMode(MOTOR2_F_PIN, OUTPUT);
  pinMode(MOTOR2_R_PIN, OUTPUT);
  pinMode(MOTOR3_F_PIN, OUTPUT);
  pinMode(MOTOR3_R_PIN, OUTPUT);
  pinMode(MOTOR4_F_PIN, OUTPUT);
  pinMode(MOTOR4_R_PIN, OUTPUT);
  pinMode(MOTOR5_F_PIN, OUTPUT);
  pinMode(MOTOR5_R_PIN, OUTPUT);
  pinMode(MOTOR6_F_PIN, OUTPUT);
  pinMode(MOTOR6_R_PIN, OUTPUT);
  pinMode(MOTOR7_F_PIN, OUTPUT);
  pinMode(MOTOR7_R_PIN, OUTPUT);
  pinMode(MOTOR8_F_PIN, OUTPUT);
  pinMode(MOTOR8_R_PIN, OUTPUT);
  pinMode(MOTOR9_F_PIN, OUTPUT);
  pinMode(MOTOR9_R_PIN, OUTPUT);
  
  // Tüm motorları durdur
  for (uint8_t i = 1; i <= 9; i++) {
    Motor_Stop(i);
  }
  
  // Flash'tan Slave ID oku
  slave_id = Flash_Read_SlaveID();
  if (slave_id == 0 || slave_id > 16) {
    slave_id = SLAVE_ID_DEFAULT;
  }
  
  // Encoder'ları başlat (sadece normal modda)
  if (!HARDWARE_TEST_MODE) {
    Setup_Encoders();
    Setup_InputCapture();
  } else {
    Serial.println("HARDWARE TEST MODE: Encoders disabled");
  }
  
  // Boot testi (LED 5 kere yanıp sön)
  for(int i = 0; i < 5; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
    delay(50);
  }
  
  // RS485 RX moduna geç
  RS485_RX_Mode();
  
  Serial.println("STM32F407 Slave Ready");
  Serial.print("Slave ID: ");
  Serial.println(slave_id);
  
  if (HARDWARE_TEST_MODE) {
    Serial.println("*** HARDWARE TEST MODE ACTIVE ***");
    Serial.println("Encoders: DISABLED");
    Serial.println("Motor Control: SIMPLE (0=REVERSE, >0=FORWARD)");
    Serial.println("No position feedback - motors run continuously");
  }
}

/* --- LOOP --- */
void loop() {
  // Veri geldi mi?
  if (ReadLine_NonBlocking(rx_buffer, &rx_index, LINE_BUFFER_LEN)) {
    Process_Packet(rx_buffer);
  }
  
  // Encoder değerlerini periyodik olarak oku (Encoder Mode için) - Sadece normal modda
  if (!HARDWARE_TEST_MODE) {
    static unsigned long last_encoder_read = 0;
    if (millis() - last_encoder_read > 10) { // 10ms'de bir (hızlı tepki için)
      // Motor 1-6: Encoder Mode (register seviyesinde okuma)
      for (uint8_t i = 1; i <= 6; i++) {
        Read_Encoder(i);
      }
      
      last_encoder_read = millis();
    }
    
    // Debug çıktıları kapatıldı (performans için)
  }
  
  // Homing işlemini kontrol et
  Process_Homing();
  
  // Motor kontrolünü sürekli çalıştır (hedef pozisyona gitmek için) - Sadece normal modda
  // TEST MODU: Motor kontrolü direkt MOV komutunda yapılıyor, loop'ta tekrar kontrol etme
  if (!HARDWARE_TEST_MODE) {
    static unsigned long last_motor_control = 0;
    if (millis() - last_motor_control > 50) { // 50ms'de bir motor kontrolü
      for (uint8_t i = 1; i <= 9; i++) {
        // Eğer homing yapılmıyorsa ve hedef pozisyon set edilmişse (>= 0), motoru kontrol et
        if (motor_homing[i - 1] == 0 && motor_positions[i - 1] >= 0) {
          Motor_Control(i, motor_positions[i - 1]);
        }
      }
      
      last_motor_control = millis();
    }
  }
}

/* --- ENCODER SETUP (Register Seviyesinde - F407 için) --- */
void Setup_Encoders(void) {
  // Motor 1-6: Encoder Mode (TIM1, TIM2, TIM3, TIM4, TIM5, TIM8)
  Setup_Encoder_Timer(1); // TIM1 - Motor 1 (PE9, PE11)
  Setup_Encoder_Timer(2); // TIM2 - Motor 2 (PA5, PB3) - 32-bit
  Setup_Encoder_Timer(3); // TIM3 - Motor 3 (PA6, PA7)
  Setup_Encoder_Timer(4); // TIM4 - Motor 4 (PD12, PD13)
  Setup_Encoder_Timer(5); // TIM5 - Motor 5 (PA0, PA1) - 32-bit
  Setup_Encoder_Timer(8); // TIM8 - Motor 6 (PC6, PC7)
  
  delay(10); // Timer'ların stabilize olması için
  
  // İlk değerleri kaydet
  for (uint8_t i = 0; i < 6; i++) {
    last_encoder_raw[i] = 0;
    encoder_counters[i] = 0;
  }
  
  Serial.println("Encoder Mode setup complete (6 motors)");
}

/* --- ENCODER TIMER SETUP (Register Seviyesinde - F407) --- */
void Setup_Encoder_Timer(uint8_t timer_num) {
  TIM_TypeDef* TIMx = NULL;
  
  // Timer seçimi ve clock enable
  switch(timer_num) {
    case 1: // TIM1 - Motor 1 (PE9, PE11)
      RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
      RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
      TIMx = TIM1;
      // PE9: TIM1_CH1, PE11: TIM1_CH2
      // GPIOE MODER: PE9=Pin 9 (bit 18-19), PE11=Pin 11 (bit 22-23)
      GPIOE->MODER &= ~(GPIO_MODER_MODER9 | GPIO_MODER_MODER11);
      GPIOE->MODER |= (GPIO_MODER_MODER9_1 | GPIO_MODER_MODER11_1); // Alternate Function
      GPIOE->AFR[1] |= (2 << ((9-8)*4)) | (2 << ((11-8)*4)); // AF2 = TIM1
      break;
      
    case 2: // TIM2 - Motor 2 (PA5, PB3) - 32-bit
      RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
      RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
      TIMx = TIM2;
      // PA5: TIM2_CH1, PB3: TIM2_CH2
      GPIOA->MODER &= ~GPIO_MODER_MODER5;
      GPIOA->MODER |= GPIO_MODER_MODER5_1; // Alternate Function
      GPIOA->AFR[0] |= (1 << (5*4)); // AF1 = TIM2
      GPIOB->MODER &= ~GPIO_MODER_MODER3;
      GPIOB->MODER |= GPIO_MODER_MODER3_1; // Alternate Function
      GPIOB->AFR[0] |= (1 << (3*4)); // AF1 = TIM2
      break;
      
    case 3: // TIM3 - Motor 3 (PA6, PA7)
      RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
      RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
      TIMx = TIM3;
      // PA6: TIM3_CH1, PA7: TIM3_CH2
      GPIOA->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
      GPIOA->MODER |= (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1); // Alternate Function
      GPIOA->AFR[0] |= (2 << (6*4)) | (2 << (7*4)); // AF2 = TIM3
      break;
      
    case 4: // TIM4 - Motor 4 (PD12, PD13)
      RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
      RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
      TIMx = TIM4;
      // PD12: TIM4_CH1, PD13: TIM4_CH2
      GPIOD->MODER &= ~(GPIO_MODER_MODER12 | GPIO_MODER_MODER13);
      GPIOD->MODER |= (GPIO_MODER_MODER12_1 | GPIO_MODER_MODER13_1); // Alternate Function
      GPIOD->AFR[1] |= (2 << ((12-8)*4)) | (2 << ((13-8)*4)); // AF2 = TIM4
      break;
      
    case 5: // TIM5 - Motor 5 (PA0, PA1) - 32-bit
      RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
      RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
      TIMx = TIM5;
      // PA0: TIM5_CH1, PA1: TIM5_CH2
      GPIOA->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1);
      GPIOA->MODER |= (GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1); // Alternate Function
      GPIOA->AFR[0] |= (2 << (0*4)) | (2 << (1*4)); // AF2 = TIM5
      break;
      
    case 8: // TIM8 - Motor 6 (PC6, PC7)
      RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
      RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
      TIMx = TIM8;
      // PC6: TIM8_CH1, PC7: TIM8_CH2
      GPIOC->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
      GPIOC->MODER |= (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1); // Alternate Function
      GPIOC->AFR[0] |= (3 << (6*4)) | (3 << (7*4)); // AF3 = TIM8
      break;
      
    default:
      return;
  }
  
  // Timer yapılandırması (Encoder Mode 3 - TI12, 4x çözünürlük)
  TIMx->CR1 = 0;
  TIMx->CNT = 0;
  
  // CCMR1: Capture/Compare Mode Register
  // CC1S = 01 (Input, mapped to TI1)
  // CC2S = 01 (Input, mapped to TI2)
  // IC1F ve IC2F = 1111 (0xF) -> En güçlü Filtre
  TIMx->CCMR1 = 0;
  TIMx->CCMR1 |= (1 << 0) | (1 << 8); // CC1S ve CC2S
  TIMx->CCMR1 |= (0xF << 4) | (0xF << 12); // Filtreler
  
  // CCER: Polarity (Rising edge)
  TIMx->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P); // Non-inverted (Motor 2 eski haline döndü)
  
  // SMCR: Slave Mode Control Register
  // SMS = 011 (Encoder Mode 3 - Hem TI1 hem TI2 kenarlarını say)
  TIMx->SMCR &= ~TIM_SMCR_SMS;
  TIMx->SMCR |= 3; // Encoder Mode 3
  
  // ARR ve PSC
  if (timer_num == 2 || timer_num == 5) {
    // 32-bit timer'lar için ARR = 0xFFFFFFFF
    TIMx->ARR = 0xFFFFFFFF;
  } else {
    // 16-bit timer'lar için ARR = 65535
    TIMx->ARR = 65535;
  }
  TIMx->PSC = 0;
  
  // Counter Enable
  TIMx->CR1 |= TIM_CR1_CEN;
  
  // Advanced Timer'lar için (TIM1, TIM8) Main Output Enable
  if (timer_num == 1 || timer_num == 8) {
    TIMx->BDTR |= TIM_BDTR_MOE;
  }
}

void Setup_InputCapture(void) {
  // Motor 7: TIM9 - İki kanal (PE5, PE6) - Encoder Mode benzeri
  // Motor 8: TIM12 - İki kanal (PB14, PB15) - Encoder Mode benzeri
  // Motor 9: TIM10 CH1 (PB8) + TIM11 CH1 (PB9) - İki timer, yön tespiti ile
  
  // TIM9 - Motor 7 (PE5, PE6) - İki kanal
  RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
  
  // PE5: TIM9_CH1, PE6: TIM9_CH2
  GPIOE->MODER &= ~(GPIO_MODER_MODER5 | GPIO_MODER_MODER6);
  GPIOE->MODER |= (GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1); // Alternate Function
  GPIOE->AFR[0] |= (3 << (5*4)) | (3 << (6*4)); // AF3 = TIM9
  
  TIM9->CR1 = 0;
  TIM9->CNT = 0;
  TIM9->PSC = 0;
  TIM9->ARR = 65535;
  
  // Input Capture Mode - Both Channels
  TIM9->CCMR1 = 0;
  TIM9->CCMR1 |= (1 << 0); // CC1S = 01 (Input, TI1)
  TIM9->CCMR1 |= (1 << 8); // CC2S = 01 (Input, TI2)
  TIM9->CCMR1 |= (0xF << 4) | (0xF << 12); // Filtreler
  
  TIM9->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P); // Rising edge
  TIM9->DIER |= TIM_DIER_CC1IE | TIM_DIER_CC2IE; // Interrupt enable
  TIM9->CR1 |= TIM_CR1_CEN;
  
  // TIM12 - Motor 8 (PB14, PB15) - İki kanal
  RCC->APB1ENR |= RCC_APB1ENR_TIM12EN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  
  // PB14: TIM12_CH1, PB15: TIM12_CH2
  GPIOB->MODER &= ~(GPIO_MODER_MODER14 | GPIO_MODER_MODER15);
  GPIOB->MODER |= (GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1); // Alternate Function
  GPIOB->AFR[1] |= (9 << ((14-8)*4)) | (9 << ((15-8)*4)); // AF9 = TIM12
  
  TIM12->CR1 = 0;
  TIM12->CNT = 0;
  TIM12->PSC = 0;
  TIM12->ARR = 65535;
  
  // Input Capture Mode - Both Channels
  TIM12->CCMR1 = 0;
  TIM12->CCMR1 |= (1 << 0); // CC1S = 01 (Input, TI1)
  TIM12->CCMR1 |= (1 << 8); // CC2S = 01 (Input, TI2)
  TIM12->CCMR1 |= (0xF << 4) | (0xF << 12); // Filtreler
  
  TIM12->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P); // Rising edge
  TIM12->DIER |= TIM_DIER_CC1IE | TIM_DIER_CC2IE; // Interrupt enable
  TIM12->CR1 |= TIM_CR1_CEN;
  
  // TIM10 - Motor 9 CH1 (PB8)
  RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
  
  // PB8: TIM10_CH1
  GPIOB->MODER &= ~GPIO_MODER_MODER8;
  GPIOB->MODER |= GPIO_MODER_MODER8_1; // Alternate Function
  GPIOB->AFR[1] |= (3 << ((8-8)*4)); // AF3 = TIM10
  
  TIM10->CR1 = 0;
  TIM10->CNT = 0;
  TIM10->PSC = 0;
  TIM10->ARR = 65535;
  
  TIM10->CCMR1 = 0;
  TIM10->CCMR1 |= (1 << 0); // CC1S = 01 (Input, TI1)
  TIM10->CCMR1 |= (0xF << 4); // Filtre
  TIM10->CCER &= ~TIM_CCER_CC1P; // Rising edge
  TIM10->DIER |= TIM_DIER_CC1IE; // Interrupt enable
  TIM10->CR1 |= TIM_CR1_CEN;
  
  // TIM11 - Motor 9 CH2 (PB9)
  RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
  
  // PB9: TIM11_CH1
  GPIOB->MODER &= ~GPIO_MODER_MODER9;
  GPIOB->MODER |= GPIO_MODER_MODER9_1; // Alternate Function
  GPIOB->AFR[1] |= (3 << ((9-8)*4)); // AF3 = TIM11
  
  TIM11->CR1 = 0;
  TIM11->CNT = 0;
  TIM11->PSC = 0;
  TIM11->ARR = 65535;
  
  TIM11->CCMR1 = 0;
  TIM11->CCMR1 |= (1 << 0); // CC1S = 01 (Input, TI1)
  TIM11->CCMR1 |= (0xF << 4); // Filtre
  TIM11->CCER &= ~TIM_CCER_CC1P; // Rising edge
  TIM11->DIER |= TIM_DIER_CC1IE; // Interrupt enable
  TIM11->CR1 |= TIM_CR1_CEN;
  
  // NVIC Interrupt Enable
  NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn); // TIM9 interrupt
  NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn); // TIM12 interrupt
  NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn); // TIM10 interrupt (Motor 9 CH1)
  NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn); // TIM11 interrupt (Motor 9 CH2)
  
  Serial.println("Input Capture setup complete (Motor 7: TIM9, Motor 8: TIM12, Motor 9: TIM10+11)");
}

/* --- INTERRUPT HANDLERS --- */
// Arduino framework'ün handler'larını override etmek için
// Handler'ları __real_ prefix'i ile tanımlıyoruz (linker flag ile yönlendirilecek)
extern "C" {
  // TIM9 Interrupt Handler (Motor 7) - İki kanal (PE5, PE6)
  void __real_TIM1_BRK_TIM9_IRQHandler(void) {
    if (TIM9->SR & TIM_SR_CC1IF) {
      encoder_counters[6]++; // Motor 7 - CH1
      TIM9->SR &= ~TIM_SR_CC1IF;
    }
    if (TIM9->SR & TIM_SR_CC2IF) {
      encoder_counters[6]++; // Motor 7 - CH2
      TIM9->SR &= ~TIM_SR_CC2IF;
    }
  }
  
  // TIM12 Interrupt Handler (Motor 8) - İki kanal (PB14, PB15)
  void __real_TIM8_BRK_TIM12_IRQHandler(void) {
    if (TIM12->SR & TIM_SR_CC1IF) {
      encoder_counters[7]++; // Motor 8 - CH1
      TIM12->SR &= ~TIM_SR_CC1IF;
    }
    if (TIM12->SR & TIM_SR_CC2IF) {
      encoder_counters[7]++; // Motor 8 - CH2
      TIM12->SR &= ~TIM_SR_CC2IF;
    }
  }
  
  // TIM10 Interrupt Handler (Motor 9 CH1 - PB8)
  void __real_TIM1_UP_TIM10_IRQHandler(void) {
    if (TIM10->SR & TIM_SR_CC1IF) {
      // Quadrature mantığı: TIM10 (CH1) önce gelirse forward
      if (motor9_last_interrupt == 0 || motor9_last_interrupt == 1) {
        // İlk interrupt veya TIM10 tekrar geldi -> Forward
        encoder_counters[8]++; // Motor 9 - Forward
      } else if (motor9_last_interrupt == 2) {
        // TIM11 önce geldi, şimdi TIM10 geldi -> Forward (normal quadrature)
        encoder_counters[8]++; // Motor 9 - Forward
      }
      motor9_last_interrupt = 1; // TIM10 geldi
      TIM10->SR &= ~TIM_SR_CC1IF;
    }
  }
  
  // TIM11 Interrupt Handler (Motor 9 CH2 - PB9)
  void __real_TIM1_TRG_COM_TIM11_IRQHandler(void) {
    if (TIM11->SR & TIM_SR_CC1IF) {
      // Quadrature mantığı: TIM11 (CH2) önce gelirse reverse
      if (motor9_last_interrupt == 0 || motor9_last_interrupt == 2) {
        // İlk interrupt veya TIM11 tekrar geldi -> Reverse
        encoder_counters[8]--; // Motor 9 - Reverse
      } else if (motor9_last_interrupt == 1) {
        // TIM10 önce geldi, şimdi TIM11 geldi -> Forward (normal quadrature)
        encoder_counters[8]++; // Motor 9 - Forward
      }
      motor9_last_interrupt = 2; // TIM11 geldi
      TIM11->SR &= ~TIM_SR_CC1IF;
    }
  }
}

/* --- RS485 FONKSİYONLARI --- */
void RS485_TX_Mode(void) {
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  delayMicroseconds(50);
}

void RS485_RX_Mode(void) {
  digitalWrite(RS485_DE_RE_PIN, LOW);
}

void Send_Response(const char* msg) {
  delay(20); // Master dinlemeye geçsin
  RS485_TX_Mode();
  SerialRS485.print(msg);
  SerialRS485.flush();
  delay(5);
  RS485_RX_Mode();
}

// Debug mesajlarını RS485 üzerinden gönder (Python UI terminalinde görünsün)
void Send_Debug(const char* msg) {
  // Debug mesajlarını RS485'e gönder (master forward edecek)
  char debug_msg[128];
  snprintf(debug_msg, sizeof(debug_msg), "DEBUG:%s\n", msg);
  Send_Response(debug_msg);
}

bool ReadLine_NonBlocking(char *dst, uint8_t *pos, size_t max_len) {
  if (SerialRS485.available()) {
    char ch = SerialRS485.read();
    if (ch == '\n') {
      dst[*pos] = '\0';
      *pos = 0;
      return true;
    } else if (ch != '\r') {
      if (*pos < max_len - 1) {
        dst[(*pos)++] = ch;
      }
    }
  }
  return false;
}

/* --- FLASH MEMORY FONKSİYONLARI --- */
uint8_t Flash_Read_SlaveID(void) {
  // Flash okuma - HAL kütüphanesi ile
  // Şimdilik basit implementasyon
  uint32_t flash_data = *((volatile uint32_t*)SLAVE_ID_FLASH_ADDRESS);
  
  if (flash_data == 0xFFFFFFFF || flash_data == 0x00000000) {
    return SLAVE_ID_DEFAULT;
  }
  
  uint32_t magic = (flash_data >> 16) & 0xFFFF;
  uint32_t id = flash_data & 0xFFFF;
  
  if (magic == (SLAVE_ID_MAGIC & 0xFFFF) && id >= 1 && id <= 16) {
    return (uint8_t)id;
  }
  
  return SLAVE_ID_DEFAULT;
}

bool Flash_Write_SlaveID(uint8_t id) {
  // Flash yazma - HAL kütüphanesi ile tam implementasyon
  
  // ID kontrolü
  if (id < 1 || id > 16) {
    Serial.println("Flash write error: Invalid ID (1-16)");
    return false;
  }
  
  // HAL Flash fonksiyonları için gerekli include'lar
  // PlatformIO HAL framework'ünde bu fonksiyonlar otomatik gelir
  
  // Flash Unlock
  if (HAL_FLASH_Unlock() != HAL_OK) {
    Serial.println("Flash write error: Unlock failed");
    return false;
  }
  
  // Flash'taki mevcut değeri kontrol et
  uint32_t current_data = *((volatile uint32_t*)SLAVE_ID_FLASH_ADDRESS);
  uint32_t new_data = ((SLAVE_ID_MAGIC & 0xFFFF) << 16) | id;
  
  // Eğer aynı değer yazılmak isteniyorsa, erase etmeye gerek yok
  if (current_data == new_data) {
    HAL_FLASH_Lock();
    Serial.println("Flash write: Same ID, no write needed");
    return true;
  }
  
  // Son sector'ü erase et (STM32F407VETX: Sector 7, 128KB)
  // Flash: 512KB (0x08000000 - 0x0807FFFF)
  // Sector yapısı:
  //   Sector 0-3: 16KB each (0x08000000 - 0x0800FFFF)
  //   Sector 4: 64KB (0x08010000 - 0x0801FFFF)
  //   Sector 5: 128KB (0x08020000 - 0x0803FFFF)
  //   Sector 6: 128KB (0x08040000 - 0x0805FFFF)
  //   Sector 7: 128KB (0x08060000 - 0x0807FFFF) <- SON SECTOR!
  // Son 4 byte: 0x0807FFFC, bu Sector 7'de
  
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SectorError = 0;
  
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3; // 2.7V - 3.6V
  EraseInitStruct.Sector = FLASH_SECTOR_7;  // Son sector (512KB Flash için)
  EraseInitStruct.NbSectors = 1;
  
  HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
  if (status != HAL_OK) {
    HAL_FLASH_Lock();
    Serial.print("Flash write error: Erase failed, SectorError=");
    Serial.println(SectorError, HEX);
    return false;
  }
  
  // Yeni değeri yaz
  status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, SLAVE_ID_FLASH_ADDRESS, new_data);
  
  // Flash Lock
  HAL_FLASH_Lock();
  
  if (status != HAL_OK) {
    Serial.println("Flash write error: Program failed");
    return false;
  }
  
  // Yazılan değeri doğrula
  uint32_t verify_data = *((volatile uint32_t*)SLAVE_ID_FLASH_ADDRESS);
  if (verify_data != new_data) {
    Serial.print("Flash write error: Verification failed. Written=");
    Serial.print(new_data, HEX);
    Serial.print(", Read=");
    Serial.println(verify_data, HEX);
    return false;
  }
  
  Serial.print("Flash write success: ID=");
  Serial.println(id);
  return true;
}

/* --- MOTOR KONTROL FONKSİYONLARI --- */
void Motor_Stop(uint8_t motor_id) {
  if (motor_id < 1 || motor_id > 9) return;
  
  switch(motor_id) {
    case 1: digitalWrite(MOTOR1_F_PIN, LOW); digitalWrite(MOTOR1_R_PIN, LOW); break;
    case 2: digitalWrite(MOTOR2_F_PIN, LOW); digitalWrite(MOTOR2_R_PIN, LOW); break;
    case 3: digitalWrite(MOTOR3_F_PIN, LOW); digitalWrite(MOTOR3_R_PIN, LOW); break;
    case 4: digitalWrite(MOTOR4_F_PIN, LOW); digitalWrite(MOTOR4_R_PIN, LOW); break;
    case 5: digitalWrite(MOTOR5_F_PIN, LOW); digitalWrite(MOTOR5_R_PIN, LOW); break;
    case 6: digitalWrite(MOTOR6_F_PIN, LOW); digitalWrite(MOTOR6_R_PIN, LOW); break;
    case 7: digitalWrite(MOTOR7_F_PIN, LOW); digitalWrite(MOTOR7_R_PIN, LOW); break;
    case 8: digitalWrite(MOTOR8_F_PIN, LOW); digitalWrite(MOTOR8_R_PIN, LOW); break;
    case 9: digitalWrite(MOTOR9_F_PIN, LOW); digitalWrite(MOTOR9_R_PIN, LOW); break;
  }
}

/* --- ENCODER OKUMA (STM32F103 Test Kodundaki Mantık - Birebir) --- */
void Read_Encoder(uint8_t motor_id) {
  if (motor_id < 1 || motor_id > 6) return; // Sadece Encoder Mode motorlar (1-6)
  
  TIM_TypeDef* TIMx = NULL;
  uint8_t idx = motor_id - 1;
  
  // Timer seçimi
  switch(motor_id) {
    case 1: TIMx = TIM1; break;
    case 2: TIMx = TIM2; break;
    case 3: TIMx = TIM3; break;
    case 4: TIMx = TIM4; break;
    case 5: TIMx = TIM5; break;
    case 6: TIMx = TIM8; break;
    default: return;
  }
  
  // Motor 2 ve 5: 32-bit timer (TIM2, TIM5) -> int32_t kullan
  // Diğerleri: 16-bit timer -> int16_t kullan
  int32_t current_cnt;
  int32_t diff;
  
  if (motor_id == 2 || motor_id == 5) {
    // 32-bit timer: int32_t olarak oku
    current_cnt = (int32_t)TIMx->CNT;
    diff = current_cnt - last_encoder_raw[idx];
    
    // 32-bit overflow/underflow kontrolü (2^31 = 2147483648, ama pratikte daha küçük değerler için)
    if (diff > 2000000000) diff -= 4294967296UL;
    if (diff < -2000000000) diff += 4294967296UL;
  } else {
    // 16-bit timer: int16_t olarak oku
    current_cnt = (int16_t)TIMx->CNT;
    diff = current_cnt - (int16_t)last_encoder_raw[idx];
    
    // 16-bit overflow/underflow kontrolü
    if (diff > 30000) diff -= 65536;
    if (diff < -30000) diff += 65536;
  }
  
  if (diff != 0) {
    // Motor 2 için encoder yönü ters - diff'i ters çevir
    if (motor_id == 2) {
      // Motor 2 için: encoder_counters += diff (ters yön)
      encoder_counters[idx] += diff;
    } else {
      // Diğer motorlar için: encoder_counters -= diff (normal yön)
      encoder_counters[idx] -= diff;
    }
    
    last_encoder_raw[idx] = current_cnt;
  }
}

int32_t Encoder_Read(uint8_t motor_id) {
  if (motor_id < 1 || motor_id > 9) return 0;
  
  // Motor 1-6: Encoder Mode (sürekli okunur)
  // Motor 7-9: Input Capture (interrupt ile sayılır)
  
  return encoder_counters[motor_id - 1];
}

/* --- ENCODER SIFIRLA --- */
void Reset_Encoder(uint8_t motor_id) {
  if (motor_id < 1 || motor_id > 6) return;
  
  uint8_t idx = motor_id - 1;
  encoder_counters[idx] = 0;
  
  TIM_TypeDef* TIMx = NULL;
  switch(motor_id) {
    case 1: TIMx = TIM1; break;
    case 2: TIMx = TIM2; break;
    case 3: TIMx = TIM3; break;
    case 4: TIMx = TIM4; break;
    case 5: TIMx = TIM5; break;
    case 6: TIMx = TIM8; break;
    default: return;
  }
  
  // Motor 2 ve 5: 32-bit timer, diğerleri: 16-bit timer
  if (motor_id == 2 || motor_id == 5) {
    last_encoder_raw[idx] = (int32_t)TIMx->CNT;
  } else {
    last_encoder_raw[idx] = (int16_t)TIMx->CNT;
  }
  
}

void Motor_Control(uint8_t motor_id, int16_t target_position) {
  if (motor_id < 1 || motor_id > 9) return;
  if (target_position < 0 || target_position > 600) return;
  
  // HARDWARE TEST MODU: Basit motor kontrolü (encoder yok)
  if (HARDWARE_TEST_MODE) {
    // Hedef pozisyonu kaydet
    motor_positions[motor_id - 1] = target_position;
    
    // Basit kontrol: 0 = Reverse (home), >0 = Forward
    if (target_position == 0) {
      // Home pozisyonu: Reverse (YÖN TERSİNE ÇEVRİLDİ)
      if (motor_id == 1) { digitalWrite(MOTOR1_F_PIN, HIGH); digitalWrite(MOTOR1_R_PIN, LOW); }
      else if (motor_id == 2) { digitalWrite(MOTOR2_F_PIN, HIGH); digitalWrite(MOTOR2_R_PIN, LOW); }
      else if (motor_id == 3) { digitalWrite(MOTOR3_F_PIN, HIGH); digitalWrite(MOTOR3_R_PIN, LOW); }
      else if (motor_id == 4) { digitalWrite(MOTOR4_F_PIN, HIGH); digitalWrite(MOTOR4_R_PIN, LOW); }
      else if (motor_id == 5) { digitalWrite(MOTOR5_F_PIN, HIGH); digitalWrite(MOTOR5_R_PIN, LOW); }
      else if (motor_id == 6) { digitalWrite(MOTOR6_F_PIN, HIGH); digitalWrite(MOTOR6_R_PIN, LOW); }
      else if (motor_id == 7) { digitalWrite(MOTOR7_F_PIN, HIGH); digitalWrite(MOTOR7_R_PIN, LOW); }
      else if (motor_id == 8) { digitalWrite(MOTOR8_F_PIN, HIGH); digitalWrite(MOTOR8_R_PIN, LOW); }
      else if (motor_id == 9) { digitalWrite(MOTOR9_F_PIN, HIGH); digitalWrite(MOTOR9_R_PIN, LOW); }
    } else {
      // Forward pozisyonu (YÖN TERSİNE ÇEVRİLDİ)
      if (motor_id == 1) { digitalWrite(MOTOR1_F_PIN, LOW); digitalWrite(MOTOR1_R_PIN, HIGH); }
      else if (motor_id == 2) { digitalWrite(MOTOR2_F_PIN, LOW); digitalWrite(MOTOR2_R_PIN, HIGH); }
      else if (motor_id == 3) { digitalWrite(MOTOR3_F_PIN, LOW); digitalWrite(MOTOR3_R_PIN, HIGH); }
      else if (motor_id == 4) { digitalWrite(MOTOR4_F_PIN, LOW); digitalWrite(MOTOR4_R_PIN, HIGH); }
      else if (motor_id == 5) { digitalWrite(MOTOR5_F_PIN, LOW); digitalWrite(MOTOR5_R_PIN, HIGH); }
      else if (motor_id == 6) { digitalWrite(MOTOR6_F_PIN, LOW); digitalWrite(MOTOR6_R_PIN, HIGH); }
      else if (motor_id == 7) { digitalWrite(MOTOR7_F_PIN, LOW); digitalWrite(MOTOR7_R_PIN, HIGH); }
      else if (motor_id == 8) { digitalWrite(MOTOR8_F_PIN, LOW); digitalWrite(MOTOR8_R_PIN, HIGH); }
      else if (motor_id == 9) { digitalWrite(MOTOR9_F_PIN, LOW); digitalWrite(MOTOR9_R_PIN, HIGH); }
    }
    
    Serial.print("TEST MODE: Motor ");
    Serial.print(motor_id);
    Serial.print(" -> ");
    Serial.print(target_position == 0 ? "REVERSE" : "FORWARD");
    Serial.print(" (");
    Serial.print(target_position);
    Serial.println("mm)");
    return;
  }
  
  // NORMAL MOD: Encoder ile pozisyon kontrolü
  // Hedef pozisyonu kaydet
  motor_positions[motor_id - 1] = target_position;
  
  // Encoder değerini oku
  int32_t current_encoder = Encoder_Read(motor_id);
  
  // Hedef pozisyonu pulse'a çevir (mm -> pulse)
  int32_t target_pulse = (int32_t)(target_position * PULSES_PER_MM);
  
  // Hata hesapla
  int32_t error = target_pulse - current_encoder;
  
  
  // Tolerans (20 pulse)
  #define POSITION_TOLERANCE 20
  
  // Bang-Bang Kontrol
  if (abs(error) > POSITION_TOLERANCE) {
    if (error > 0) {
      // İleri git (YÖN TERSİNE ÇEVRİLDİ)
      if (motor_id == 1) { digitalWrite(MOTOR1_F_PIN, LOW); digitalWrite(MOTOR1_R_PIN, HIGH); }
      else if (motor_id == 2) { digitalWrite(MOTOR2_F_PIN, LOW); digitalWrite(MOTOR2_R_PIN, HIGH); }
      else if (motor_id == 3) { digitalWrite(MOTOR3_F_PIN, LOW); digitalWrite(MOTOR3_R_PIN, HIGH); }
      else if (motor_id == 4) { digitalWrite(MOTOR4_F_PIN, LOW); digitalWrite(MOTOR4_R_PIN, HIGH); }
      else if (motor_id == 5) { digitalWrite(MOTOR5_F_PIN, LOW); digitalWrite(MOTOR5_R_PIN, HIGH); }
      else if (motor_id == 6) { digitalWrite(MOTOR6_F_PIN, LOW); digitalWrite(MOTOR6_R_PIN, HIGH); }
      else if (motor_id == 7) { digitalWrite(MOTOR7_F_PIN, LOW); digitalWrite(MOTOR7_R_PIN, HIGH); }
      else if (motor_id == 8) { digitalWrite(MOTOR8_F_PIN, LOW); digitalWrite(MOTOR8_R_PIN, HIGH); }
      else if (motor_id == 9) { digitalWrite(MOTOR9_F_PIN, LOW); digitalWrite(MOTOR9_R_PIN, HIGH); }
    } else {
      // Geri git (YÖN TERSİNE ÇEVRİLDİ)
      if (motor_id == 1) { digitalWrite(MOTOR1_F_PIN, HIGH); digitalWrite(MOTOR1_R_PIN, LOW); }
      else if (motor_id == 2) { digitalWrite(MOTOR2_F_PIN, HIGH); digitalWrite(MOTOR2_R_PIN, LOW); }
      else if (motor_id == 3) { digitalWrite(MOTOR3_F_PIN, HIGH); digitalWrite(MOTOR3_R_PIN, LOW); }
      else if (motor_id == 4) { digitalWrite(MOTOR4_F_PIN, HIGH); digitalWrite(MOTOR4_R_PIN, LOW); }
      else if (motor_id == 5) { digitalWrite(MOTOR5_F_PIN, HIGH); digitalWrite(MOTOR5_R_PIN, LOW); }
      else if (motor_id == 6) { digitalWrite(MOTOR6_F_PIN, HIGH); digitalWrite(MOTOR6_R_PIN, LOW); }
      else if (motor_id == 7) { digitalWrite(MOTOR7_F_PIN, HIGH); digitalWrite(MOTOR7_R_PIN, LOW); }
      else if (motor_id == 8) { digitalWrite(MOTOR8_F_PIN, HIGH); digitalWrite(MOTOR8_R_PIN, LOW); }
      else if (motor_id == 9) { digitalWrite(MOTOR9_F_PIN, HIGH); digitalWrite(MOTOR9_R_PIN, LOW); }
    }
  } else {
    // Hedefe ulaşıldı, motoru durdur
    Motor_Stop(motor_id);
  }
}

/* --- HOMING FONKSİYONLARI --- */
void Motor_Home(uint8_t motor_id) {
  if (motor_id < 1 || motor_id > 9) return;
  
  uint8_t idx = motor_id - 1;
  
  // Homing başlat
  motor_homing[idx] = 1;
  last_homing_check_time[idx] = millis();
  
  // Encoder değerini oku (başlangıç pozisyonu)
  if (motor_id >= 1 && motor_id <= 6) {
    // Encoder Mode motorlar için
    TIM_TypeDef* TIMx = NULL;
    switch(motor_id) {
      case 1: TIMx = TIM1; break;
      case 2: TIMx = TIM2; break;
      case 3: TIMx = TIM3; break;
      case 4: TIMx = TIM4; break;
      case 5: TIMx = TIM5; break;
      case 6: TIMx = TIM8; break;
      default: return;
    }
    
    // Motor 2 ve 5: 32-bit timer, diğerleri: 16-bit timer
    if (motor_id == 2 || motor_id == 5) {
      last_homing_pos[idx] = (int32_t)TIMx->CNT;
    } else {
      last_homing_pos[idx] = (int16_t)TIMx->CNT;
    }
  } else {
    // Input Capture motorlar için
    last_homing_pos[idx] = encoder_counters[idx];
  }
  
  // Motoru reverse'a al (home'a git)
  if (motor_id == 1) { digitalWrite(MOTOR1_F_PIN, HIGH); digitalWrite(MOTOR1_R_PIN, LOW); }
  else if (motor_id == 2) { digitalWrite(MOTOR2_F_PIN, HIGH); digitalWrite(MOTOR2_R_PIN, LOW); }
  else if (motor_id == 3) { digitalWrite(MOTOR3_F_PIN, HIGH); digitalWrite(MOTOR3_R_PIN, LOW); }
  else if (motor_id == 4) { digitalWrite(MOTOR4_F_PIN, HIGH); digitalWrite(MOTOR4_R_PIN, LOW); }
  else if (motor_id == 5) { digitalWrite(MOTOR5_F_PIN, HIGH); digitalWrite(MOTOR5_R_PIN, LOW); }
  else if (motor_id == 6) { digitalWrite(MOTOR6_F_PIN, HIGH); digitalWrite(MOTOR6_R_PIN, LOW); }
  else if (motor_id == 7) { digitalWrite(MOTOR7_F_PIN, HIGH); digitalWrite(MOTOR7_R_PIN, LOW); }
  else if (motor_id == 8) { digitalWrite(MOTOR8_F_PIN, HIGH); digitalWrite(MOTOR8_R_PIN, LOW); }
  else if (motor_id == 9) { digitalWrite(MOTOR9_F_PIN, HIGH); digitalWrite(MOTOR9_R_PIN, LOW); }
  
  Serial.print("HOMING: Motor ");
  Serial.print(motor_id);
  Serial.println(" started");
}

void Process_Homing(void) {
  unsigned long now = millis();
  
  for (uint8_t i = 0; i < 9; i++) {
    uint8_t motor_id = i + 1;
    
    // Eğer bu motor homing yapıyorsa
    if (motor_homing[i] == 1) {
      // Kontrol zamanı geldi mi?
      if (now - last_homing_check_time[i] >= HOMING_CHECK_INTERVAL_MS) {
        int32_t current_pos = 0;
        
        // Mevcut encoder pozisyonunu oku - STM32F103 mantığı (int16_t)
        if (motor_id >= 1 && motor_id <= 6) {
          // Encoder Mode motorlar için
          TIM_TypeDef* TIMx = NULL;
          switch(motor_id) {
            case 1: TIMx = TIM1; break;
            case 2: TIMx = TIM2; break;
            case 3: TIMx = TIM3; break;
            case 4: TIMx = TIM4; break;
            case 5: TIMx = TIM5; break;
            case 6: TIMx = TIM8; break;
            default: continue;
          }
          
          // Motor 2 ve 5: 32-bit timer, diğerleri: 16-bit timer
          if (motor_id == 2 || motor_id == 5) {
            current_pos = (int32_t)TIMx->CNT;
          } else {
            current_pos = (int16_t)TIMx->CNT;
          }
        } else {
          // Input Capture motorlar için
          current_pos = encoder_counters[i];
        }
        
        // Pozisyon değişikliğini kontrol et
        int32_t pos_diff = abs(current_pos - last_homing_pos[i]);
        
        // Eğer pozisyon değişmediyse (limit switch'e ulaşıldı)
        // Motor 2 için threshold'u artır (32-bit timer, daha hassas olabilir)
        uint8_t threshold = (motor_id == 2) ? 10 : HOMING_STALL_THRESHOLD;
        if (pos_diff < threshold) {
          // Motoru durdur
          Motor_Stop(motor_id);
          
          // Encoder'ı sıfırla
          Reset_Encoder(motor_id);
          encoder_counters[i] = 0; // Ekstra güvenlik için tekrar sıfırla
          
          // Homing tamamlandı
          motor_homing[i] = 0;
          
          // RS485 üzerinden home tamamlandı mesajı gönder
          char response[32];
          snprintf(response, sizeof(response), "HOMEDONE:%02d:%02d\n", slave_id, motor_id);
          Send_Response(response);
          
          Serial.print("HOMING: Motor ");
          Serial.print(motor_id);
          Serial.println(" completed");
        } else {
          // Hala hareket ediyor, pozisyonu güncelle
          last_homing_pos[i] = current_pos;
          last_homing_check_time[i] = now;
        }
      }
    }
  }
}

/* --- PACKET PROCESSING --- */
void Process_Packet(char* line) {
  char tempLine[LINE_BUFFER_LEN];
  strncpy(tempLine, line, LINE_BUFFER_LEN);
  tempLine[LINE_BUFFER_LEN - 1] = '\0';
  
  char *cmd = strtok(tempLine, ":");
  char *param1 = strtok(NULL, ":");
  char *param2 = strtok(NULL, ":");
  
  if (cmd == NULL) return;
  
  // SETID komutu
  if (strcmp(cmd, "SETID") == 0) {
    if (param1 != NULL) {
      int newID = atoi(param1);
      if (newID >= 1 && newID <= 16) {
        if (Flash_Write_SlaveID((uint8_t)newID)) {
          slave_id = (uint8_t)newID;
          char response[32];
          snprintf(response, sizeof(response), "IDSET:%02d\n", slave_id);
          Send_Response(response);
          
          // LED 3 kere yanıp sön
          for(int i = 0; i < 3; i++) {
            digitalWrite(LED_PIN, HIGH);
            delay(100);
            digitalWrite(LED_PIN, LOW);
            delay(100);
          }
        } else {
          Send_Response("IDERR:Flash write failed\n");
        }
      } else {
        Send_Response("IDERR:Invalid ID (1-16)\n");
      }
    }
    return;
  }
  
  // Diğer komutlar için target ID kontrolü
  if (param1 == NULL) return;
  
  int targetID = atoi(param1);
  
  if (targetID == slave_id) {
    digitalWrite(LED_PIN, HIGH);
    
    if (strcmp(cmd, "PING") == 0) {
      delay(4000); // 4 saniye bekle
      char response[32];
      snprintf(response, sizeof(response), "PONG:%02d\n", slave_id);
      Send_Response(response);
    }
    else if (strcmp(cmd, "MOV") == 0) {
      if (param2 != NULL) {
        int motorID = atoi(param2);
        int targetValue = atoi(strtok(NULL, ":"));
        
        if (motorID >= 1 && motorID <= 9 && targetValue >= 0 && targetValue <= 600) {
          Motor_Control((uint8_t)motorID, (int16_t)targetValue);
          char response[32];
          snprintf(response, sizeof(response), "MOVOK:%02d:%02d:%d\n", slave_id, motorID, targetValue);
          Send_Response(response);
        } else {
          Send_Response("MOVERR:Invalid params\n");
        }
      }
    }
    else if (strcmp(cmd, "ALL") == 0) {
      int targetValue = atoi(param2);
      
      if (targetValue >= 0 && targetValue <= 600) {
        for (uint8_t m = 1; m <= 9; m++) {
          Motor_Control(m, (int16_t)targetValue);
        }
        char response[32];
        snprintf(response, sizeof(response), "ALLOK:%02d:%d\n", slave_id, targetValue);
        Send_Response(response);
      } else {
        Send_Response("ALLERR:Invalid value (0-600)\n");
      }
    }
    else if (strcmp(cmd, "HOME") == 0) {
      if (param2 != NULL) {
        int motorID = atoi(param2);
        
        if (motorID >= 1 && motorID <= 9) {
          Motor_Home((uint8_t)motorID);
          char response[32];
          snprintf(response, sizeof(response), "HOME:%02d:%02d\n", slave_id, motorID);
          Send_Response(response);
        } else {
          Send_Response("HOMEERR:Invalid motor ID (1-9)\n");
        }
      }
    }
    
    delay(100);
    digitalWrite(LED_PIN, LOW);
  }
}

