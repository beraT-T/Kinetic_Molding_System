/**
 * STM32F401 KART 1 (U2) - 5 MOTORLU İŞÇİ DÜĞÜM
 * - RS485 Yok. İletişim USART2 (PA2/PA3) üzerinden U3 ile.
 * - Sadece Motor 1, 2, 3, 4, 5.
 * - Tümü Hardware Encoder (TIM1, TIM2, TIM3, TIM4, TIM5).
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include <string.h>

#define LINE_BUFFER_LEN 64
#define CAL_HARDWARE 79.93f  // Tüm motorlar hardware olduğu için tek kalibrasyon

/* --- PIN TANIMLARI (U2) --- */
// Sürüş Pinleri
#define MOTOR1_F_PIN PB14
#define MOTOR1_R_PIN PB15
#define MOTOR2_F_PIN PB4
#define MOTOR2_R_PIN PB5
#define MOTOR3_F_PIN PB0
#define MOTOR3_R_PIN PB1
#define MOTOR4_F_PIN PB8
#define MOTOR4_R_PIN PB9
#define MOTOR5_F_PIN PC14
#define MOTOR5_R_PIN PC15

// Haberleşme (U3 ile)
HardwareSerial SerialU3(USART2); // PA3 (RX), PA2 (TX)

/* --- GLOBAL DEĞİŞKENLER --- */
char rx_buffer[LINE_BUFFER_LEN];
uint8_t rx_index = 0;
int16_t motor_positions[5] = {-1, -1, -1, -1, -1};
volatile int32_t encoder_counters[5] = {0, 0, 0, 0, 0};
int32_t last_encoder_raw[5] = {0, 0, 0, 0, 0};

/* --- FONKSİYON PROTOTİPLERİ --- */
void Process_Packet(char* line);
bool ReadLine_NonBlocking(char *dst, uint8_t *pos, size_t max_len);
void Setup_Encoders(void);
void Read_Encoder(uint8_t motor_id);
void Motor_Control(uint8_t motor_id, int16_t target_position);
void Motor_Stop(uint8_t motor_id);

void setup() {
  SerialU3.begin(115200); // U3 ile yüksek hızda konuşabilir
  
  pinMode(MOTOR1_F_PIN, OUTPUT); pinMode(MOTOR1_R_PIN, OUTPUT);
  pinMode(MOTOR2_F_PIN, OUTPUT); pinMode(MOTOR2_R_PIN, OUTPUT);
  pinMode(MOTOR3_F_PIN, OUTPUT); pinMode(MOTOR3_R_PIN, OUTPUT);
  pinMode(MOTOR4_F_PIN, OUTPUT); pinMode(MOTOR4_R_PIN, OUTPUT);
  pinMode(MOTOR5_F_PIN, OUTPUT); pinMode(MOTOR5_R_PIN, OUTPUT);
  
  for (uint8_t i = 1; i <= 5; i++) Motor_Stop(i);
  
  Setup_Encoders();
}

void loop() {
  if (ReadLine_NonBlocking(rx_buffer, &rx_index, LINE_BUFFER_LEN)) {
    Process_Packet(rx_buffer);
  }
  
  static unsigned long last_encoder_read = 0;
  if (millis() - last_encoder_read > 10) {
    for (uint8_t i = 1; i <= 5; i++) Read_Encoder(i);
    last_encoder_read = millis();
  }
  
  static unsigned long last_motor_control = 0;
  if (millis() - last_motor_control > 50) {
    for (uint8_t i = 1; i <= 5; i++) {
      if (motor_positions[i - 1] >= 0) Motor_Control(i, motor_positions[i - 1]);
    }
    last_motor_control = millis();
  }
}

// STM32F401 için Timer Register Ayarları
void Setup_Encoders(void) {
  // TIM1 (PA8, PA9) - AF1
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  GPIOA->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9); GPIOA->MODER |= (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1);
  GPIOA->AFR[1] |= (1 << ((8-8)*4)) | (1 << ((9-8)*4));
  TIM1->SMCR |= 3; TIM1->CCER |= (TIM_CCER_CC1P | TIM_CCER_CC2P); TIM1->CR1 |= TIM_CR1_CEN;

  // TIM2 (PA15, PB3) - AF1
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
  GPIOA->MODER &= ~GPIO_MODER_MODER15; GPIOA->MODER |= GPIO_MODER_MODER15_1; GPIOA->AFR[1] |= (1 << ((15-8)*4));
  GPIOB->MODER &= ~GPIO_MODER_MODER3; GPIOB->MODER |= GPIO_MODER_MODER3_1; GPIOB->AFR[0] |= (1 << (3*4));
  TIM2->SMCR |= 3; TIM2->CCER |= (TIM_CCER_CC1P | TIM_CCER_CC2P); TIM2->CR1 |= TIM_CR1_CEN;

  // TIM3 (PA6, PA7) - AF2
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  GPIOA->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7); GPIOA->MODER |= (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);
  GPIOA->AFR[0] |= (2 << (6*4)) | (2 << (7*4));
  TIM3->SMCR |= 3; TIM3->CCER |= (TIM_CCER_CC1P | TIM_CCER_CC2P); TIM3->CR1 |= TIM_CR1_CEN;

  // TIM4 (PB6, PB7) - AF2
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  GPIOB->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7); GPIOB->MODER |= (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);
  GPIOB->AFR[0] |= (2 << (6*4)) | (2 << (7*4));
  TIM4->SMCR |= 3; TIM4->CCER |= (TIM_CCER_CC1P | TIM_CCER_CC2P); TIM4->CR1 |= TIM_CR1_CEN;

  // TIM5 (PA0, PA1) - AF2
  RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
  GPIOA->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1); GPIOA->MODER |= (GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1);
  GPIOA->AFR[0] |= (2 << (0*4)) | (2 << (1*4));
  TIM5->SMCR |= 3; TIM5->CCER |= (TIM_CCER_CC1P | TIM_CCER_CC2P); TIM5->CR1 |= TIM_CR1_CEN;
}

void Read_Encoder(uint8_t motor_id) {
  TIM_TypeDef* TIMx = NULL; uint8_t idx = motor_id - 1;
  switch(motor_id) { case 1: TIMx=TIM1; break; case 2: TIMx=TIM2; break; case 3: TIMx=TIM3; break; case 4: TIMx=TIM4; break; case 5: TIMx=TIM5; break; }
  int32_t current_cnt = (TIMx == TIM2 || TIMx == TIM5) ? (int32_t)TIMx->CNT : (int16_t)TIMx->CNT;
  int32_t diff = current_cnt - last_encoder_raw[idx];
  if (TIMx == TIM2 || TIMx == TIM5) { if(diff > 2000000000) diff -= 4294967296UL; else if(diff < -2000000000) diff += 4294967296UL; } 
  else { if(diff > 30000) diff -= 65536; else if(diff < -30000) diff += 65536; }
  if (diff != 0) { encoder_counters[idx] += diff; last_encoder_raw[idx] = current_cnt; }
}

void Motor_Stop(uint8_t motor_id) {
  switch(motor_id) {
    case 1: digitalWrite(MOTOR1_F_PIN, LOW); digitalWrite(MOTOR1_R_PIN, LOW); break;
    case 2: digitalWrite(MOTOR2_F_PIN, LOW); digitalWrite(MOTOR2_R_PIN, LOW); break;
    case 3: digitalWrite(MOTOR3_F_PIN, LOW); digitalWrite(MOTOR3_R_PIN, LOW); break;
    case 4: digitalWrite(MOTOR4_F_PIN, LOW); digitalWrite(MOTOR4_R_PIN, LOW); break;
    case 5: digitalWrite(MOTOR5_F_PIN, LOW); digitalWrite(MOTOR5_R_PIN, LOW); break;
  }
}

void Motor_Control(uint8_t motor_id, int16_t target_position) {
  int32_t target_pulse = (int32_t)(target_position * CAL_HARDWARE);
  int32_t error = target_pulse - encoder_counters[motor_id - 1];
  
  if (abs(error) > 20) {
    bool forward = (error < 0); // Yönler projene göre terslenebilir
    if (motor_id == 1) { digitalWrite(MOTOR1_F_PIN, forward); digitalWrite(MOTOR1_R_PIN, !forward); }
    else if (motor_id == 2) { digitalWrite(MOTOR2_F_PIN, forward); digitalWrite(MOTOR2_R_PIN, !forward); }
    else if (motor_id == 3) { digitalWrite(MOTOR3_F_PIN, forward); digitalWrite(MOTOR3_R_PIN, !forward); }
    else if (motor_id == 4) { digitalWrite(MOTOR4_F_PIN, forward); digitalWrite(MOTOR4_R_PIN, !forward); }
    else if (motor_id == 5) { digitalWrite(MOTOR5_F_PIN, forward); digitalWrite(MOTOR5_R_PIN, !forward); }
  } else { Motor_Stop(motor_id); }
}

bool ReadLine_NonBlocking(char *dst, uint8_t *pos, size_t max_len) {
  if (SerialU3.available()) { char ch = SerialU3.read(); if (ch == '\n') { dst[*pos] = '\0'; *pos = 0; return true; } else if (ch != '\r') { if (*pos < max_len - 1) dst[(*pos)++] = ch; } }
  return false;
}

void Process_Packet(char* line) {
  char tempLine[LINE_BUFFER_LEN]; strncpy(tempLine, line, LINE_BUFFER_LEN); tempLine[LINE_BUFFER_LEN - 1] = '\0';
  char *cmd = strtok(tempLine, ":"); char *param1 = strtok(NULL, ":"); char *param2 = strtok(NULL, ":"); char *param3 = strtok(NULL, ":");
  if (cmd == NULL) return;
  // Format: INTERNAL_MOV:MotorID:Value
  if (strcmp(cmd, "INTERNAL_MOV") == 0 && param1 != NULL && param2 != NULL) {
    int mID = atoi(param1); int val = atoi(param2);
    if (mID >= 1 && mID <= 5) motor_positions[mID - 1] = val;
  }
}