/**
 * STM32F401 KART 2 (U3) - 4 MOTORLU YÖNETİCİ DÜĞÜM
 * - RS485 ile ana hattan komut alır. (ID mantığı burada çalışır).
 * - Motor 6, 7, 8, 9'u kendi sürer. (Kendi içinde 1,2,3,4 olarak indekslenir)
 * - Motor 1, 2, 3, 4, 5 komutlarını USART2 ile U2'ye paslar.
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include <string.h>

#define LINE_BUFFER_LEN 64
#define CAL_HARDWARE 79.93f
#define SLAVE_ID 1 // Flaş okuma yerine sabit ID (veya EEPROM kütüphanesi eklenebilir)

/* --- PIN TANIMLARI (U3) --- */
// Haberleşme
#define RS485_DE_RE_PIN PA8
HardwareSerial SerialRS485(USART1); // PA10 (RX), PA9 (TX)
HardwareSerial SerialU2(USART2);    // PA3 (RX), PA2 (TX)

// Sürüş Pinleri (Sistemdeki Motor 6, 7, 8, 9)
#define MOTOR6_F_PIN PB4
#define MOTOR6_R_PIN PB5
#define MOTOR7_F_PIN PB0
#define MOTOR7_R_PIN PB1
#define MOTOR8_F_PIN PB8
#define MOTOR8_R_PIN PB9
#define MOTOR9_F_PIN PC14
#define MOTOR9_R_PIN PC15

char rx_buffer[LINE_BUFFER_LEN];
uint8_t rx_index = 0;
int16_t motor_positions[4] = {-1, -1, -1, -1};
volatile int32_t encoder_counters[4] = {0, 0, 0, 0};
int32_t last_encoder_raw[4] = {0, 0, 0, 0};

/* --- FONKSİYON PROTOTİPLERİ --- */
void RS485_TX_Mode(void); void RS485_RX_Mode(void); void Send_Response(const char* msg);
void Process_Packet(char* line); bool ReadLine_NonBlocking(char *dst, uint8_t *pos, size_t max_len);
void Setup_Encoders(void); void Read_Encoder(uint8_t local_motor_id);
void Motor_Control(uint8_t local_motor_id, int16_t target_position); void Motor_Stop(uint8_t local_motor_id);

void setup() {
  SerialRS485.begin(9600);
  SerialU2.begin(115200); // U2 ile iletişim hızı
  pinMode(RS485_DE_RE_PIN, OUTPUT);
  
  pinMode(MOTOR6_F_PIN, OUTPUT); pinMode(MOTOR6_R_PIN, OUTPUT);
  pinMode(MOTOR7_F_PIN, OUTPUT); pinMode(MOTOR7_R_PIN, OUTPUT);
  pinMode(MOTOR8_F_PIN, OUTPUT); pinMode(MOTOR8_R_PIN, OUTPUT);
  pinMode(MOTOR9_F_PIN, OUTPUT); pinMode(MOTOR9_R_PIN, OUTPUT);
  
  for (uint8_t i = 1; i <= 4; i++) Motor_Stop(i);
  Setup_Encoders();
  RS485_RX_Mode();
}

void loop() {
  if (ReadLine_NonBlocking(rx_buffer, &rx_index, LINE_BUFFER_LEN)) {
    Process_Packet(rx_buffer);
  }
  
  static unsigned long last_encoder_read = 0;
  if (millis() - last_encoder_read > 10) {
    for (uint8_t i = 1; i <= 4; i++) Read_Encoder(i);
    last_encoder_read = millis();
  }
  
  static unsigned long last_motor_control = 0;
  if (millis() - last_motor_control > 50) {
    for (uint8_t i = 1; i <= 4; i++) {
      if (motor_positions[i - 1] >= 0) Motor_Control(i, motor_positions[i - 1]);
    }
    last_motor_control = millis();
  }
}

// U3 için Timer Ayarları (Sadece TIM2, TIM3, TIM4, TIM5 var)
void Setup_Encoders(void) {
  // Local Motor 1 (Sistem 6) -> TIM2 (PA15, PB3)
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
  GPIOA->MODER &= ~GPIO_MODER_MODER15; GPIOA->MODER |= GPIO_MODER_MODER15_1; GPIOA->AFR[1] |= (1 << ((15-8)*4));
  GPIOB->MODER &= ~GPIO_MODER_MODER3; GPIOB->MODER |= GPIO_MODER_MODER3_1; GPIOB->AFR[0] |= (1 << (3*4));
  TIM2->SMCR |= 3; TIM2->CCER |= (TIM_CCER_CC1P | TIM_CCER_CC2P); TIM2->CR1 |= TIM_CR1_CEN;

  // Local Motor 2 (Sistem 7) -> TIM3 (PA6, PA7)
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  GPIOA->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7); GPIOA->MODER |= (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);
  GPIOA->AFR[0] |= (2 << (6*4)) | (2 << (7*4));
  TIM3->SMCR |= 3; TIM3->CCER |= (TIM_CCER_CC1P | TIM_CCER_CC2P); TIM3->CR1 |= TIM_CR1_CEN;

  // Local Motor 3 (Sistem 8) -> TIM4 (PB6, PB7)
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  GPIOB->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7); GPIOB->MODER |= (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);
  GPIOB->AFR[0] |= (2 << (6*4)) | (2 << (7*4));
  TIM4->SMCR |= 3; TIM4->CCER |= (TIM_CCER_CC1P | TIM_CCER_CC2P); TIM4->CR1 |= TIM_CR1_CEN;

  // Local Motor 4 (Sistem 9) -> TIM5 (PA0, PA1)
  RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
  GPIOA->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1); GPIOA->MODER |= (GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1);
  GPIOA->AFR[0] |= (2 << (0*4)) | (2 << (1*4));
  TIM5->SMCR |= 3; TIM5->CCER |= (TIM_CCER_CC1P | TIM_CCER_CC2P); TIM5->CR1 |= TIM_CR1_CEN;
}

void Read_Encoder(uint8_t local_motor_id) {
  TIM_TypeDef* TIMx = NULL; uint8_t idx = local_motor_id - 1;
  switch(local_motor_id) { case 1: TIMx=TIM2; break; case 2: TIMx=TIM3; break; case 3: TIMx=TIM4; break; case 4: TIMx=TIM5; break; }
  int32_t current_cnt = (TIMx == TIM2 || TIMx == TIM5) ? (int32_t)TIMx->CNT : (int16_t)TIMx->CNT;
  int32_t diff = current_cnt - last_encoder_raw[idx];
  if (TIMx == TIM2 || TIMx == TIM5) { if(diff > 2000000000) diff -= 4294967296UL; else if(diff < -2000000000) diff += 4294967296UL; } 
  else { if(diff > 30000) diff -= 65536; else if(diff < -30000) diff += 65536; }
  if (diff != 0) { encoder_counters[idx] += diff; last_encoder_raw[idx] = current_cnt; }
}

void Motor_Stop(uint8_t local_motor_id) {
  switch(local_motor_id) {
    case 1: digitalWrite(MOTOR6_F_PIN, LOW); digitalWrite(MOTOR6_R_PIN, LOW); break;
    case 2: digitalWrite(MOTOR7_F_PIN, LOW); digitalWrite(MOTOR7_R_PIN, LOW); break;
    case 3: digitalWrite(MOTOR8_F_PIN, LOW); digitalWrite(MOTOR8_R_PIN, LOW); break;
    case 4: digitalWrite(MOTOR9_F_PIN, LOW); digitalWrite(MOTOR9_R_PIN, LOW); break;
  }
}

void Motor_Control(uint8_t local_motor_id, int16_t target_position) {
  int32_t target_pulse = (int32_t)(target_position * CAL_HARDWARE);
  int32_t error = target_pulse - encoder_counters[local_motor_id - 1];
  
  if (abs(error) > 20) {
    bool forward = (error < 0);
    if (local_motor_id == 1) { digitalWrite(MOTOR6_F_PIN, forward); digitalWrite(MOTOR6_R_PIN, !forward); }
    else if (local_motor_id == 2) { digitalWrite(MOTOR7_F_PIN, forward); digitalWrite(MOTOR7_R_PIN, !forward); }
    else if (local_motor_id == 3) { digitalWrite(MOTOR8_F_PIN, forward); digitalWrite(MOTOR8_R_PIN, !forward); }
    else if (local_motor_id == 4) { digitalWrite(MOTOR9_F_PIN, forward); digitalWrite(MOTOR9_R_PIN, !forward); }
  } else { Motor_Stop(local_motor_id); }
}

void RS485_TX_Mode(void) { digitalWrite(RS485_DE_RE_PIN, HIGH); delayMicroseconds(50); }
void RS485_RX_Mode(void) { digitalWrite(RS485_DE_RE_PIN, LOW); }
void Send_Response(const char* msg) { delay(20); RS485_TX_Mode(); SerialRS485.print(msg); SerialRS485.flush(); delay(5); RS485_RX_Mode(); }

bool ReadLine_NonBlocking(char *dst, uint8_t *pos, size_t max_len) {
  if (SerialRS485.available()) { char ch = SerialRS485.read(); if (ch == '\n') { dst[*pos] = '\0'; *pos = 0; return true; } else if (ch != '\r') { if (*pos < max_len - 1) dst[(*pos)++] = ch; } }
  return false;
}

void Process_Packet(char* line) {
  char tempLine[LINE_BUFFER_LEN]; strncpy(tempLine, line, LINE_BUFFER_LEN); tempLine[LINE_BUFFER_LEN - 1] = '\0';
  char *cmd = strtok(tempLine, ":"); char *param1 = strtok(NULL, ":"); char *param2 = strtok(NULL, ":"); char *param3 = strtok(NULL, ":");
  if (cmd == NULL || param1 == NULL) return;
  
  int targetID = atoi(param1);
  if (targetID == SLAVE_ID) {
    if (strcmp(cmd, "MOV") == 0 && param2 != NULL && param3 != NULL) {
      int mID = atoi(param2); int val = atoi(param3);
      if (val >= 0 && val <= 600) {
        if (mID >= 1 && mID <= 5) {
          // U2'ye (Slave MCU) pasla
          char u2_cmd[32]; snprintf(u2_cmd, 32, "INTERNAL_MOV:%d:%d\n", mID, val);
          SerialU2.print(u2_cmd);
        } else if (mID >= 6 && mID <= 9) {
          // Kendi motorları (Lokal 1-4 olarak haritala)
          motor_positions[mID - 5] = val;
        }
        char r[32]; snprintf(r,32,"MOVOK:%02d:%02d:%d\n", SLAVE_ID, mID, val); Send_Response(r);
      }
    }
  }
}