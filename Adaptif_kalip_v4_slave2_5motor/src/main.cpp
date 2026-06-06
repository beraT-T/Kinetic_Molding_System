/**
 * STM32F401 KART 1 (U2) - 5 MOTORLU İŞÇİ DÜĞÜM - v4.4 STABLE
 *
 * - RS485 YOK. Sadece U3 ile USART2 (PA2/PA3) üzerinden haberleşir.
 * - Motor 1-5, TIM1-TIM5 hardware encoder.
 * - Komutlar bloklayıcı çalışır: hareket/home biter, sonra U3'e cevap döner.
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include <string.h>
#include <stdlib.h>

// --- SABITLER ---
#define LINE_BUFFER_LEN        64
#define CAL_HARDWARE           79.93f
#define MOTOR_DEADBAND         20
#define MOTOR_MAX_MM           600
#define LED_PIN                PC13
#define ENCODER_SERVICE_MS     10
#define MOTOR_STAGGER_MS       100
#define MOVE_TIMEOUT_MS        90000
#define HOME_START_GRACE_MS    1200
#define HOME_STALL_TIMEOUT_MS  1800
#define HOME_MAX_TIME_MS       70000
#define HOME_STALL_PULSES      2

// Motor başına encoder yönü — true = encoder ters, false = encoder düz
// Bir motor MOV sırasında sınırına koşarsa ilgili değeri flip et.
const bool MOTOR_ENCODER_REVERSED[5] = {
    true,   // Motor 1
    true,   // Motor 2
    true,   // Motor 3
    true,   // Motor 4
    true    // Motor 5
};

// HOME sırasında aktüatörü fiziksel geri yöne süren çıkış yönü
// Ters yönde home yaparsa ilgili motoru false yap.
const bool MOTOR_HOME_RETRACT_FWD[5] = {
    true,   // Motor 1
    true,   // Motor 2
    true,   // Motor 3
    true,   // Motor 4
    true    // Motor 5
};

// ============================================================
// PIN TANIMLARI (U2)
// ============================================================
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

HardwareSerial SerialU3(USART2); // PA3(RX), PA2(TX)

// ============================================================
// GLOBAL DEĞİŞKENLER
// ============================================================
char     rx_buffer[LINE_BUFFER_LEN];
uint8_t  rx_index = 0;
volatile int32_t encoder_counters[5] = {0, 0, 0, 0, 0};
int32_t  last_encoder_raw[5]         = {0, 0, 0, 0, 0};
unsigned long last_encoder_service   = 0;
int16_t  motor_targets[5]            = {0, 0, 0, 0, 0};
bool     motor_active[5]             = {false, false, false, false, false};
unsigned long motor_start_at[5]      = {0, 0, 0, 0, 0};

// ============================================================
// FONKSİYON PROTOTİPLERİ
// ============================================================
void LED_Blink(uint8_t times); void LED_Blink_Slow(uint8_t times);
bool ReadLine_NonBlocking(char *dst, uint8_t *pos, size_t max_len);
void Process_Packet(char* line);
void Setup_Encoders(void);
void Service_Encoders(void);
void Read_Encoder(uint8_t motor_id);
int32_t Encoder_Raw(uint8_t motor_id);
void Motor_Stop(uint8_t motor_id);
void Motor_Stop_All(void);
void Motor_Drive(uint8_t motor_id, bool fwd);
void Motor_Move_Drive(uint8_t motor_id, int32_t error);
bool Motor_Move_To(uint8_t motor_id, int16_t target_mm);
bool Motor_Home(uint8_t motor_id);
void Motor_Schedule(uint8_t motor_id, int16_t target_mm, unsigned long delay_ms);
void Motor_Update_All(void);

// ============================================================
// SETUP & LOOP
// ============================================================
void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    LED_Blink(5);

    SerialU3.begin(115200);

    pinMode(MOTOR1_F_PIN, OUTPUT); pinMode(MOTOR1_R_PIN, OUTPUT);
    pinMode(MOTOR2_F_PIN, OUTPUT); pinMode(MOTOR2_R_PIN, OUTPUT);
    pinMode(MOTOR3_F_PIN, OUTPUT); pinMode(MOTOR3_R_PIN, OUTPUT);
    pinMode(MOTOR4_F_PIN, OUTPUT); pinMode(MOTOR4_R_PIN, OUTPUT);
    pinMode(MOTOR5_F_PIN, OUTPUT); pinMode(MOTOR5_R_PIN, OUTPUT);

    Motor_Stop_All();
    Setup_Encoders();
    for (uint8_t i = 1; i <= 5; i++) last_encoder_raw[i - 1] = Encoder_Raw(i);
    LED_Blink_Slow(3);
}

void loop() {
    Service_Encoders();
    Motor_Update_All();

    if (ReadLine_NonBlocking(rx_buffer, &rx_index, LINE_BUFFER_LEN)) {
        LED_Blink(1);
        Process_Packet(rx_buffer);
    }
}

// ============================================================
// ENCODER — TIM1/3/4: 16-bit signed, TIM2/5: 32-bit
// ============================================================
void Setup_Encoders(void) {
    // TIM1 (PA8, PA9) AF1 — 16-bit
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
    GPIOA->MODER |=  (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1);
    GPIOA->AFR[1] &= ~((0xFu << ((8-8)*4)) | (0xFu << ((9-8)*4)));
    GPIOA->AFR[1] |=  (1u << ((8-8)*4)) | (1u << ((9-8)*4));
    TIM1->CR1 = 0;
    TIM1->CNT = 0;
    TIM1->ARR = 0xFFFF;
    TIM1->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
    TIM1->CCMR1 |=  (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);
    TIM1->SMCR &= ~TIM_SMCR_SMS;
    TIM1->SMCR |=  (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1);
    TIM1->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC1NP | TIM_CCER_CC2NP);
    TIM1->CCER |=  (TIM_CCER_CC1P | TIM_CCER_CC2P);
    TIM1->CR1 |= TIM_CR1_CEN;

    // TIM2 (PA5, PB3) AF1 — PA15 boşta, gerçek encoder pinleri bunlar
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    GPIOA->MODER &= ~GPIO_MODER_MODER5;   GPIOA->MODER |= GPIO_MODER_MODER5_1;
    GPIOA->AFR[0] &= ~(0xFu << (5*4));    GPIOA->AFR[0] |= (1u << (5*4));
    GPIOB->MODER &= ~GPIO_MODER_MODER3;   GPIOB->MODER |= GPIO_MODER_MODER3_1;
    GPIOB->AFR[0] &= ~(0xFu << (3*4));    GPIOB->AFR[0] |= (1u << (3*4));
    TIM2->CR1 = 0;
    TIM2->CNT = 0;
    TIM2->ARR = 0xFFFFFFFFu;
    TIM2->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
    TIM2->CCMR1 |=  (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);
    TIM2->SMCR &= ~TIM_SMCR_SMS;
    TIM2->SMCR |=  (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1);
    TIM2->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC1NP | TIM_CCER_CC2NP);
    TIM2->CCER |=  (TIM_CCER_CC1P | TIM_CCER_CC2P);
    TIM2->CR1 |= TIM_CR1_CEN;

    // TIM3 (PA6, PA7) AF2 — 16-bit
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    GPIOA->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    GPIOA->MODER |=  (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);
    GPIOA->AFR[0] &= ~((0xFu << (6*4)) | (0xFu << (7*4)));
    GPIOA->AFR[0] |=  (2u << (6*4)) | (2u << (7*4));
    TIM3->CR1 = 0;
    TIM3->CNT = 0;
    TIM3->ARR = 0xFFFF;
    TIM3->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
    TIM3->CCMR1 |=  (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);
    TIM3->SMCR &= ~TIM_SMCR_SMS;
    TIM3->SMCR |=  (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1);
    TIM3->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC1NP | TIM_CCER_CC2NP);
    TIM3->CCER |=  (TIM_CCER_CC1P | TIM_CCER_CC2P);
    TIM3->CR1 |= TIM_CR1_CEN;

    // TIM4 (PB6, PB7) AF2 — 16-bit
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    GPIOB->MODER |=  (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);
    GPIOB->AFR[0] &= ~((0xFu << (6*4)) | (0xFu << (7*4)));
    GPIOB->AFR[0] |=  (2u << (6*4)) | (2u << (7*4));
    TIM4->CR1 = 0;
    TIM4->CNT = 0;
    TIM4->ARR = 0xFFFF;
    TIM4->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
    TIM4->CCMR1 |=  (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);
    TIM4->SMCR &= ~TIM_SMCR_SMS;
    TIM4->SMCR |=  (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1);
    TIM4->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC1NP | TIM_CCER_CC2NP);
    TIM4->CCER |=  (TIM_CCER_CC1P | TIM_CCER_CC2P);
    TIM4->CR1 |= TIM_CR1_CEN;

    // TIM5 (PA0, PA1) AF2 — 32-bit
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    GPIOA->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1);
    GPIOA->MODER |=  (GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1);
    GPIOA->AFR[0] &= ~((0xFu << (0*4)) | (0xFu << (1*4)));
    GPIOA->AFR[0] |=  (2u << (0*4)) | (2u << (1*4));
    TIM5->CR1 = 0;
    TIM5->CNT = 0;
    TIM5->ARR = 0xFFFFFFFFu;
    TIM5->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
    TIM5->CCMR1 |=  (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);
    TIM5->SMCR &= ~TIM_SMCR_SMS;
    TIM5->SMCR |=  (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1);
    TIM5->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC1NP | TIM_CCER_CC2NP);
    TIM5->CCER |=  (TIM_CCER_CC1P | TIM_CCER_CC2P);
    TIM5->CR1 |= TIM_CR1_CEN;
}

int32_t Encoder_Raw(uint8_t motor_id) {
    TIM_TypeDef* T = NULL;
    switch (motor_id) {
        case 1: T = TIM1; break; case 2: T = TIM2; break;
        case 3: T = TIM3; break; case 4: T = TIM4; break;
        case 5: T = TIM5; break; default: return 0;
    }
    return (T == TIM2 || T == TIM5) ? (int32_t)T->CNT : (int32_t)(int16_t)T->CNT;
}

void Service_Encoders(void) {
    unsigned long now = millis();
    if (now - last_encoder_service < ENCODER_SERVICE_MS) return;
    for (uint8_t i = 1; i <= 5; i++) Read_Encoder(i);
    last_encoder_service = now;
}

void Read_Encoder(uint8_t motor_id) {
    TIM_TypeDef* T = NULL; uint8_t idx = motor_id - 1;
    switch (motor_id) {
        case 1: T = TIM1; break; case 2: T = TIM2; break;
        case 3: T = TIM3; break; case 4: T = TIM4; break;
        case 5: T = TIM5; break; default: return;
    }
    int32_t cur = (T == TIM2 || T == TIM5) ? (int32_t)T->CNT : (int32_t)(int16_t)T->CNT;
    int32_t dif = cur - last_encoder_raw[idx];
    if (T == TIM2 || T == TIM5) {
        if (dif >  2000000000) dif -= (int32_t)4294967296LL;
        else if (dif < -2000000000) dif += (int32_t)4294967296LL;
    } else {
        if (dif >  30000) dif -= 65536;
        else if (dif < -30000) dif += 65536;
    }
    if (dif) { encoder_counters[idx] += dif; last_encoder_raw[idx] = cur; }
}

// ============================================================
// MOTOR KONTROL
// ============================================================
void Motor_Stop(uint8_t motor_id) {
    switch (motor_id) {
        case 1: digitalWrite(MOTOR1_F_PIN, LOW); digitalWrite(MOTOR1_R_PIN, LOW); break;
        case 2: digitalWrite(MOTOR2_F_PIN, LOW); digitalWrite(MOTOR2_R_PIN, LOW); break;
        case 3: digitalWrite(MOTOR3_F_PIN, LOW); digitalWrite(MOTOR3_R_PIN, LOW); break;
        case 4: digitalWrite(MOTOR4_F_PIN, LOW); digitalWrite(MOTOR4_R_PIN, LOW); break;
        case 5: digitalWrite(MOTOR5_F_PIN, LOW); digitalWrite(MOTOR5_R_PIN, LOW); break;
    }
}

void Motor_Stop_All(void) {
    for (uint8_t i = 1; i <= 5; i++) Motor_Stop(i);
}

void Motor_Drive(uint8_t motor_id, bool fwd) {
    switch (motor_id) {
        case 1: digitalWrite(MOTOR1_F_PIN, fwd); digitalWrite(MOTOR1_R_PIN, !fwd); break;
        case 2: digitalWrite(MOTOR2_F_PIN, fwd); digitalWrite(MOTOR2_R_PIN, !fwd); break;
        case 3: digitalWrite(MOTOR3_F_PIN, fwd); digitalWrite(MOTOR3_R_PIN, !fwd); break;
        case 4: digitalWrite(MOTOR4_F_PIN, fwd); digitalWrite(MOTOR4_R_PIN, !fwd); break;
        case 5: digitalWrite(MOTOR5_F_PIN, fwd); digitalWrite(MOTOR5_R_PIN, !fwd); break;
    }
}

void Motor_Move_Drive(uint8_t motor_id, int32_t error) {
    bool fwd = MOTOR_ENCODER_REVERSED[motor_id - 1] ? (error < 0) : (error > 0);
    Motor_Drive(motor_id, fwd);
}

bool Motor_Move_To(uint8_t motor_id, int16_t target_mm) {
    uint8_t idx = motor_id - 1;
    int32_t target_pulse = (int32_t)((float)target_mm * CAL_HARDWARE);
    unsigned long start = millis();

    while (millis() - start < MOVE_TIMEOUT_MS) {
        Service_Encoders();
        int32_t error = target_pulse - encoder_counters[idx];
        if (labs(error) <= MOTOR_DEADBAND) {
            Motor_Stop(motor_id);
            return true;
        }
        Motor_Move_Drive(motor_id, error);
        delay(2);
    }

    Motor_Stop(motor_id);
    return false;
}

void Motor_Schedule(uint8_t motor_id, int16_t target_mm, unsigned long delay_ms) {
    if (motor_id < 1 || motor_id > 5) return;
    uint8_t idx = motor_id - 1;
    motor_targets[idx] = target_mm;
    motor_start_at[idx] = millis() + delay_ms;
    motor_active[idx] = true;
}

void Motor_Update_All(void) {
    unsigned long now = millis();

    for (uint8_t motor_id = 1; motor_id <= 5; motor_id++) {
        uint8_t idx = motor_id - 1;
        if (!motor_active[idx]) continue;
        if ((long)(now - motor_start_at[idx]) < 0) continue;

        int32_t target_pulse = (int32_t)((float)motor_targets[idx] * CAL_HARDWARE);
        int32_t error = target_pulse - encoder_counters[idx];
        if (labs(error) <= MOTOR_DEADBAND) {
            Motor_Stop(motor_id);
        } else {
            Motor_Move_Drive(motor_id, error);
        }
    }
}

bool Motor_Home(uint8_t motor_id) {
    uint8_t idx = motor_id - 1;
    unsigned long start = millis();
    unsigned long last_motion = start;
    int32_t last_count = encoder_counters[idx];
    bool seen_motion = false;

    Motor_Drive(motor_id, MOTOR_HOME_RETRACT_FWD[idx]);

    while (millis() - start < HOME_MAX_TIME_MS) {
        Service_Encoders();
        unsigned long now = millis();
        int32_t cur = encoder_counters[idx];

        if (labs(cur - last_count) > HOME_STALL_PULSES) {
            seen_motion = true;
            last_count = cur;
            last_motion = now;
        }

        if (!seen_motion && now - start >= HOME_START_GRACE_MS) break;
        if (seen_motion && now - last_motion >= HOME_STALL_TIMEOUT_MS) break;

        Motor_Drive(motor_id, MOTOR_HOME_RETRACT_FWD[idx]);
        delay(2);
    }

    Motor_Stop(motor_id);
    delay(50);
    Service_Encoders();
    encoder_counters[idx] = 0;
    last_encoder_raw[idx] = Encoder_Raw(motor_id);
    return true;
}

// ============================================================
// HABERLEŞME
// ============================================================
void LED_Blink(uint8_t times) {
    for (uint8_t i = 0; i < times; i++) {
        digitalWrite(LED_PIN, LOW);  delay(40);
        digitalWrite(LED_PIN, HIGH); delay(40);
    }
}

void LED_Blink_Slow(uint8_t times) {
    for (uint8_t i = 0; i < times; i++) {
        digitalWrite(LED_PIN, LOW);  delay(300);
        digitalWrite(LED_PIN, HIGH); delay(300);
    }
}

bool ReadLine_NonBlocking(char *dst, uint8_t *pos, size_t max_len) {
    while (SerialU3.available()) {
        char ch = (char)SerialU3.read();
        if (ch == '\n') { dst[*pos] = '\0'; *pos = 0; return true; }
        else if (ch != '\r') { if (*pos < max_len - 1) dst[(*pos)++] = ch; }
    }
    return false;
}

// ============================================================
// PAKET İŞLEME — Sadece INTERNAL_ komutları kabul eder
// ============================================================
void Process_Packet(char* line) {
    char tmp[LINE_BUFFER_LEN];
    strncpy(tmp, line, LINE_BUFFER_LEN);
    tmp[LINE_BUFFER_LEN - 1] = '\0';

    char *cmd = strtok(tmp, ":");
    char *p1  = strtok(NULL, ":");
    char *p2  = strtok(NULL, ":");
    if (!cmd) return;

    // ── INTERNAL_MOV:{motorID}:{val} ──────────────────────
    if (strcmp(cmd, "INTERNAL_MOV") == 0 && p1 && p2) {
        int mID = atoi(p1); int val = atoi(p2);
        if (mID >= 1 && mID <= 5 && val >= 0 && val <= MOTOR_MAX_MM) {
            Motor_Schedule((uint8_t)mID, (int16_t)val, 0);
            char r[40];
            snprintf(r, sizeof(r), "INTERNAL_MOVOK:%d:%d\n", mID, val);
            SerialU3.print(r);
        }
        return;
    }

    // ── INTERNAL_ALL:{val} ────────────────────────────────
    if (strcmp(cmd, "INTERNAL_ALL") == 0 && p1) {
        int val = atoi(p1);
        if (val >= 0 && val <= MOTOR_MAX_MM) {
            for (uint8_t i = 1; i <= 5; i++) {
                Motor_Schedule(i, (int16_t)val, (unsigned long)(i - 1) * MOTOR_STAGGER_MS);
            }
            char r[32];
            snprintf(r, sizeof(r), "INTERNAL_ALLOK:%d\n", val);
            SerialU3.print(r);
        }
        return;
    }

    // ── INTERNAL_HOME:{motorID} ───────────────────────────
    if (strcmp(cmd, "INTERNAL_HOME") == 0 && p1) {
        int mID = atoi(p1);
        if (mID >= 1 && mID <= 5) {
            motor_active[mID - 1] = false;
            Motor_Stop((uint8_t)mID);
            Motor_Home((uint8_t)mID);
            char r[32]; snprintf(r, sizeof(r), "INTERNAL_HOMEOK:%d\n", mID);
            SerialU3.print(r);
        }
        return;
    }

    // ── INTERNAL_GETPOS:{motorID} ─────────────────────────
    if (strcmp(cmd, "INTERNAL_GETPOS") == 0 && p1) {
        int mID = atoi(p1);
        if (mID >= 1 && mID <= 5) {
            Service_Encoders();
            int32_t pulse = encoder_counters[mID - 1];
            int32_t mm    = (int32_t)((float)pulse / CAL_HARDWARE);
            char r[48];
            snprintf(r, sizeof(r), "INTERNAL_POS:%d:%ld:%ld\n",
                     mID, (long)mm, (long)pulse);
            SerialU3.print(r);
        }
        return;
    }
}
