/**
 * STM32F401 KART 2 (U3) - 4 MOTORLU YÖNETİCİ DÜĞÜM - v4.4 STABLE
 *
 * MİMARİ:
 * PC -> F103 RS485 köprüsü -> RS485 -> U3 -> USART2 -> U2.
 * Her komut adreslidir. Hareket komutları bloklayıcı çalışır:
 * motor işi bitirir, durur, sonra cevap döner.
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include <string.h>
#include <stdlib.h>
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"

// ============================================================
// SAAT YAPILANDIRMASI - HSE 25MHz öncelikli, HSI fallback ile
// ============================================================
extern "C" void SystemClock_Config(void) {
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM       = 25;
    osc.PLL.PLLN       = 336;
    osc.PLL.PLLP       = RCC_PLLP_DIV4;
    osc.PLL.PLLQ       = 7;

    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        RCC_OscInitTypeDef hsi = {0};
        hsi.OscillatorType        = RCC_OSCILLATORTYPE_HSI;
        hsi.HSIState              = RCC_HSI_ON;
        hsi.HSICalibrationValue   = RCC_HSICALIBRATION_DEFAULT;
        hsi.PLL.PLLState          = RCC_PLL_ON;
        hsi.PLL.PLLSource         = RCC_PLLSOURCE_HSI;
        hsi.PLL.PLLM              = 8;
        hsi.PLL.PLLN              = 168;
        hsi.PLL.PLLP              = RCC_PLLP_DIV4;
        hsi.PLL.PLLQ              = 7;
        if (HAL_RCC_OscConfig(&hsi) != HAL_OK) return;
    }

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                         RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;

    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2);
}

// --- SABITLER ---
#define LINE_BUFFER_LEN        64
#define CAL_HARDWARE           79.93f
#define SLAVE_ID_DEFAULT       1
#define SLAVE_ID_MAGIC         0xABCD1234u
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

// Motor başına encoder yönü (Motor 6-9)
// Bir motor MOV sırasında sınırına koşarsa ilgili değeri flip et.
const bool MOTOR_ENCODER_REVERSED[4] = {
    true,   // Motor 6
    true,   // Motor 7
    true,   // Motor 8
    true    // Motor 9
};

// HOME sırasında aktüatörü fiziksel geri yöne süren çıkış yönü
// Ters yönde home yaparsa ilgili motoru false yap.
const bool MOTOR_HOME_RETRACT_FWD[4] = {
    true,   // Motor 6
    true,   // Motor 7
    true,   // Motor 8
    true    // Motor 9
};

uint8_t slave_id = SLAVE_ID_DEFAULT;

// ============================================================
// FLASH — Slave ID kalıcı depolama
// ============================================================
static uint32_t Flash_SizeKB(void) {
    uint32_t kb = (uint32_t)(*(volatile uint16_t*)0x1FFF7A22u);
    if (kb == 0u || kb > 512u) kb = 256u;
    return kb;
}

static uint32_t SlaveID_FlashAddr(void) {
    return 0x08000000u + Flash_SizeKB() * 1024u - 4u;
}

static uint32_t AddrToSector(uint32_t addr) {
    if (addr < 0x08004000u) return FLASH_SECTOR_0;
    if (addr < 0x08008000u) return FLASH_SECTOR_1;
    if (addr < 0x0800C000u) return FLASH_SECTOR_2;
    if (addr < 0x08010000u) return FLASH_SECTOR_3;
    if (addr < 0x08020000u) return FLASH_SECTOR_4;
    return FLASH_SECTOR_5;
}

static uint8_t Flash_Read_SlaveID(void) {
    uint32_t w = *(volatile uint32_t*)SlaveID_FlashAddr();
    if (w == 0xFFFFFFFFu || w == 0u) return SLAVE_ID_DEFAULT;
    uint32_t magic = (w >> 16) & 0xFFFFu;
    uint32_t id    = w & 0xFFFFu;
    if (magic == (SLAVE_ID_MAGIC & 0xFFFFu) && id >= 1u && id <= 16u)
        return (uint8_t)id;
    return SLAVE_ID_DEFAULT;
}

static bool Flash_Write_SlaveID(uint8_t id) {
    if (id < 1u || id > 16u) return false;
    uint32_t a    = SlaveID_FlashAddr();
    uint32_t neww = ((SLAVE_ID_MAGIC & 0xFFFFu) << 16) | (uint32_t)id;
    if (*(volatile uint32_t*)a == neww) return true;
    if (HAL_FLASH_Unlock() != HAL_OK) return false;
    FLASH_EraseInitTypeDef ei; memset(&ei, 0, sizeof(ei));
    uint32_t err = 0;
    ei.TypeErase    = FLASH_TYPEERASE_SECTORS;
    ei.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    ei.Sector       = AddrToSector(a);
    ei.NbSectors    = 1;
    if (HAL_FLASHEx_Erase(&ei, &err) != HAL_OK) { HAL_FLASH_Lock(); return false; }
    HAL_StatusTypeDef st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, a, neww);
    HAL_FLASH_Lock();
    return st == HAL_OK;
}

// ============================================================
// PIN TANIMLARI
// ============================================================
#define RS485_DE_RE_PIN PA8
HardwareSerial SerialRS485(PA10, PA9); // RX, TX
HardwareSerial SerialU2(PA3, PA2);     // RX, TX

#define MOTOR6_F_PIN PB4
#define MOTOR6_R_PIN PB5
#define MOTOR7_F_PIN PB0
#define MOTOR7_R_PIN PB1
#define MOTOR8_F_PIN PB8
#define MOTOR8_R_PIN PB9
#define MOTOR9_F_PIN PC14
#define MOTOR9_R_PIN PC15

// ============================================================
// GLOBAL DEĞİŞKENLER
// ============================================================
char     rx_buffer[LINE_BUFFER_LEN];
uint8_t  rx_index = 0;
volatile int32_t encoder_counters[4] = {0, 0, 0, 0};
int32_t  last_encoder_raw[4]         = {0, 0, 0, 0};
unsigned long last_encoder_service   = 0;
int16_t  motor_targets[4]            = {0, 0, 0, 0};
bool     motor_active[4]             = {false, false, false, false};
unsigned long motor_start_at[4]      = {0, 0, 0, 0};

// ============================================================
// FONKSİYON PROTOTİPLERİ
// ============================================================
void RS485_TX_Mode(void); void RS485_RX_Mode(void);
void Send_Response(const char* msg);
void LED_Blink(uint8_t times); void LED_Blink_Slow(uint8_t times);
bool ReadLine_NonBlocking(char *dst, uint8_t *pos, size_t max_len);
void Process_Packet(char* line);
void Setup_Encoders(void);
void Service_Encoders(void);
void Read_Encoder(uint8_t local_id);
int32_t Encoder_Raw(uint8_t local_id);
void Motor_Stop(uint8_t local_id);
void Motor_Stop_All(void);
void Motor_Drive(uint8_t local_id, bool fwd);
void Motor_Move_Drive(uint8_t local_id, int32_t error);
bool Motor_Move_To(uint8_t local_id, int16_t target_mm);
bool Motor_Home(uint8_t local_id);
void Motor_Schedule(uint8_t local_id, int16_t target_mm, unsigned long delay_ms);
void Motor_Update_All(void);
void U2_Drain(void);
bool Wait_U2_Response(const char* expected, int expected_motor, char* out, size_t out_len, unsigned long timeout_ms);
bool U2_Command_Wait(const char* msg, const char* expected, int expected_motor, char* out, size_t out_len, unsigned long timeout_ms);

// ============================================================
// SETUP & LOOP
// ============================================================
void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    LED_Blink(5);

    SerialRS485.begin(9600);
    SerialU2.begin(115200);
    pinMode(RS485_DE_RE_PIN, OUTPUT);

    pinMode(MOTOR6_F_PIN, OUTPUT); pinMode(MOTOR6_R_PIN, OUTPUT);
    pinMode(MOTOR7_F_PIN, OUTPUT); pinMode(MOTOR7_R_PIN, OUTPUT);
    pinMode(MOTOR8_F_PIN, OUTPUT); pinMode(MOTOR8_R_PIN, OUTPUT);
    pinMode(MOTOR9_F_PIN, OUTPUT); pinMode(MOTOR9_R_PIN, OUTPUT);

    Motor_Stop_All();
    Setup_Encoders();
    for (uint8_t i = 1; i <= 4; i++) last_encoder_raw[i - 1] = Encoder_Raw(i);
    slave_id = Flash_Read_SlaveID();
    RS485_RX_Mode();

    LED_Blink_Slow(slave_id);
}

void loop() {
    Service_Encoders();
    Motor_Update_All();

    if (ReadLine_NonBlocking(rx_buffer, &rx_index, LINE_BUFFER_LEN)) {
        digitalWrite(LED_PIN, LOW);
        Process_Packet(rx_buffer);
        digitalWrite(LED_PIN, HIGH);
    }
}

// ============================================================
// ENCODER & MOTOR FONKSİYONLARI
// ============================================================
void Setup_Encoders(void) {
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

    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
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

int32_t Encoder_Raw(uint8_t local_id) {
    TIM_TypeDef* T = NULL;
    switch (local_id) {
        case 1: T = TIM2; break; case 2: T = TIM3; break;
        case 3: T = TIM4; break; case 4: T = TIM5; break;
        default: return 0;
    }
    return (T == TIM2 || T == TIM5) ? (int32_t)T->CNT : (int32_t)(int16_t)T->CNT;
}

void Service_Encoders(void) {
    unsigned long now = millis();
    if (now - last_encoder_service < ENCODER_SERVICE_MS) return;
    for (uint8_t i = 1; i <= 4; i++) Read_Encoder(i);
    last_encoder_service = now;
}

void Read_Encoder(uint8_t local_id) {
    TIM_TypeDef* T = NULL; uint8_t idx = local_id - 1;
    switch (local_id) {
        case 1: T = TIM2; break; case 2: T = TIM3; break;
        case 3: T = TIM4; break; case 4: T = TIM5; break;
        default: return;
    }
    int32_t cur = (T == TIM2 || T == TIM5) ? (int32_t)T->CNT : (int32_t)(int16_t)T->CNT;
    int32_t dif = cur - last_encoder_raw[idx];
    if (T == TIM2 || T == TIM5) {
        if (dif > 2000000000) dif -= (int32_t)4294967296LL;
        else if (dif < -2000000000) dif += (int32_t)4294967296LL;
    } else {
        if (dif > 30000) dif -= 65536;
        else if (dif < -30000) dif += 65536;
    }
    if (dif) { encoder_counters[idx] += dif; last_encoder_raw[idx] = cur; }
}

void Motor_Stop(uint8_t local_id) {
    switch (local_id) {
        case 1: digitalWrite(MOTOR6_F_PIN, LOW); digitalWrite(MOTOR6_R_PIN, LOW); break;
        case 2: digitalWrite(MOTOR7_F_PIN, LOW); digitalWrite(MOTOR7_R_PIN, LOW); break;
        case 3: digitalWrite(MOTOR8_F_PIN, LOW); digitalWrite(MOTOR8_R_PIN, LOW); break;
        case 4: digitalWrite(MOTOR9_F_PIN, LOW); digitalWrite(MOTOR9_R_PIN, LOW); break;
    }
}

void Motor_Stop_All(void) {
    for (uint8_t i = 1; i <= 4; i++) Motor_Stop(i);
}

void Motor_Drive(uint8_t local_id, bool fwd) {
    switch (local_id) {
        case 1: digitalWrite(MOTOR6_F_PIN, fwd); digitalWrite(MOTOR6_R_PIN, !fwd); break;
        case 2: digitalWrite(MOTOR7_F_PIN, fwd); digitalWrite(MOTOR7_R_PIN, !fwd); break;
        case 3: digitalWrite(MOTOR8_F_PIN, fwd); digitalWrite(MOTOR8_R_PIN, !fwd); break;
        case 4: digitalWrite(MOTOR9_F_PIN, fwd); digitalWrite(MOTOR9_R_PIN, !fwd); break;
    }
}

void Motor_Move_Drive(uint8_t local_id, int32_t error) {
    bool fwd = MOTOR_ENCODER_REVERSED[local_id - 1] ? (error < 0) : (error > 0);
    Motor_Drive(local_id, fwd);
}

bool Motor_Move_To(uint8_t local_id, int16_t target_mm) {
    uint8_t idx = local_id - 1;
    int32_t target_pulse = (int32_t)((float)target_mm * CAL_HARDWARE);
    unsigned long start = millis();

    while (millis() - start < MOVE_TIMEOUT_MS) {
        Service_Encoders();
        int32_t error = target_pulse - encoder_counters[idx];
        if (labs(error) <= MOTOR_DEADBAND) {
            Motor_Stop(local_id);
            return true;
        }
        Motor_Move_Drive(local_id, error);
        delay(2);
    }

    Motor_Stop(local_id);
    return false;
}

void Motor_Schedule(uint8_t local_id, int16_t target_mm, unsigned long delay_ms) {
    if (local_id < 1 || local_id > 4) return;
    uint8_t idx = local_id - 1;
    motor_targets[idx] = target_mm;
    motor_start_at[idx] = millis() + delay_ms;
    motor_active[idx] = true;
}

void Motor_Update_All(void) {
    unsigned long now = millis();

    for (uint8_t local_id = 1; local_id <= 4; local_id++) {
        uint8_t idx = local_id - 1;
        if (!motor_active[idx]) continue;
        if ((long)(now - motor_start_at[idx]) < 0) continue;

        int32_t target_pulse = (int32_t)((float)motor_targets[idx] * CAL_HARDWARE);
        int32_t error = target_pulse - encoder_counters[idx];
        if (labs(error) <= MOTOR_DEADBAND) {
            Motor_Stop(local_id);
        } else {
            Motor_Move_Drive(local_id, error);
        }
    }
}

bool Motor_Home(uint8_t local_id) {
    uint8_t idx = local_id - 1;
    unsigned long start = millis();
    unsigned long last_motion = start;
    int32_t last_count = encoder_counters[idx];
    bool seen_motion = false;

    Motor_Drive(local_id, MOTOR_HOME_RETRACT_FWD[idx]);

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

        Motor_Drive(local_id, MOTOR_HOME_RETRACT_FWD[idx]);
        delay(2);
    }

    Motor_Stop(local_id);
    delay(50);
    Service_Encoders();
    encoder_counters[idx] = 0;
    last_encoder_raw[idx] = Encoder_Raw(local_id);
    return true;
}

// ============================================================
// U2 HABERLEŞME
// ============================================================
void U2_Drain(void) {
    while (SerialU2.available()) (void)SerialU2.read();
}

bool Wait_U2_Response(const char* expected, int expected_motor, char* out, size_t out_len, unsigned long timeout_ms) {
    char buf[LINE_BUFFER_LEN];
    uint8_t bi = 0;
    unsigned long deadline = millis() + timeout_ms;

    while ((long)(deadline - millis()) > 0) {
        Service_Encoders();
        while (SerialU2.available()) {
            char ch = (char)SerialU2.read();
            if (ch == '\n') {
                buf[bi] = '\0';

                char tmp[LINE_BUFFER_LEN];
                strncpy(tmp, buf, sizeof(tmp));
                tmp[sizeof(tmp) - 1] = '\0';

                char *cmd = strtok(tmp, ":");
                char *p1  = strtok(NULL, ":");
                if (cmd && strcmp(cmd, expected) == 0 &&
                    (expected_motor < 0 || (p1 && atoi(p1) == expected_motor))) {
                    if (out && out_len > 0) {
                        strncpy(out, buf, out_len);
                        out[out_len - 1] = '\0';
                    }
                    return true;
                }
                bi = 0;
            } else if (ch != '\r' && bi < LINE_BUFFER_LEN - 1) {
                buf[bi++] = ch;
            }
        }
        delay(1);
    }

    return false;
}

bool U2_Command_Wait(const char* msg, const char* expected, int expected_motor, char* out, size_t out_len, unsigned long timeout_ms) {
    U2_Drain();
    SerialU2.print(msg);
    return Wait_U2_Response(expected, expected_motor, out, out_len, timeout_ms);
}

// ============================================================
// RS485 & KOMUT İŞLEME
// ============================================================
void RS485_TX_Mode(void) {
    digitalWrite(RS485_DE_RE_PIN, HIGH);
    delayMicroseconds(200);
}

void RS485_RX_Mode(void) {
    SerialRS485.flush();
    delay(3);
    digitalWrite(RS485_DE_RE_PIN, LOW);
    delayMicroseconds(500);
    while (SerialRS485.available()) SerialRS485.read();
}

void Send_Response(const char* msg) {
    delay(5);
    RS485_TX_Mode();
    SerialRS485.print(msg);
    RS485_RX_Mode();
}

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
    while (SerialRS485.available()) {
        char ch = (char)SerialRS485.read();
        if (ch == '\n') { dst[*pos] = '\0'; *pos = 0; return true; }
        else if (ch != '\r') { if (*pos < max_len - 1) dst[(*pos)++] = ch; }
    }
    return false;
}

void Process_Packet(char* line) {
    char tmp[LINE_BUFFER_LEN];
    strncpy(tmp, line, LINE_BUFFER_LEN);
    tmp[LINE_BUFFER_LEN - 1] = '\0';

    char *cmd = strtok(tmp, ":");
    char *p1  = strtok(NULL, ":");
    char *p2  = strtok(NULL, ":");
    char *p3  = strtok(NULL, ":");
    if (!cmd) return;

    if (strcmp(cmd, "PING") == 0 && p1) {
        if (atoi(p1) != (int)slave_id) return;
        char r[24]; snprintf(r, sizeof(r), "PONG:%02d\n", slave_id);
        Send_Response(r); return;
    }

    if (strcmp(cmd, "GETID") == 0 && p1) {
        if (atoi(p1) != (int)slave_id) return;
        char r[24]; snprintf(r, sizeof(r), "IDVAL:%02d\n", slave_id);
        Send_Response(r); return;
    }

    if (strcmp(cmd, "SETID") == 0 && p1 && p2) {
        if (atoi(p1) != (int)slave_id) return;
        int newID = atoi(p2);
        if (newID >= 1 && newID <= 16) {
            if (Flash_Write_SlaveID((uint8_t)newID)) {
                slave_id = (uint8_t)newID;
                char r[24]; snprintf(r, sizeof(r), "IDSET:%02d\n", slave_id);
                Send_Response(r);
            } else {
                Send_Response("IDERR:FLASH\n");
            }
        }
        return;
    }

    if (!p1 || atoi(p1) != (int)slave_id) return;

    if (strcmp(cmd, "MOV") == 0 && p2 && p3) {
        int mID = atoi(p2); int val = atoi(p3);
        if (val < 0 || val > MOTOR_MAX_MM) return;

        bool ok = false;
        if (mID >= 1 && mID <= 5) {
            char q[40];
            snprintf(q, sizeof(q), "INTERNAL_MOV:%d:%d\n", mID, val);
            ok = U2_Command_Wait(q, "INTERNAL_MOVOK", mID, NULL, 0, 1000);
        } else if (mID >= 6 && mID <= 9) {
            Motor_Schedule((uint8_t)(mID - 5), (int16_t)val, 0);
            ok = true;
        } else {
            return;
        }

        char r[40];
        snprintf(r, sizeof(r), ok ? "MOVOK:%02d:%02d:%d\n" : "MOVERR:%02d:%02d:%d\n",
                 slave_id, mID, val);
        Send_Response(r);
        return;
    }

    if (strcmp(cmd, "ALL") == 0 && p2) {
        int val = atoi(p2);
        if (val < 0 || val > MOTOR_MAX_MM) return;

        bool ok = true;
        char q[32];
        snprintf(q, sizeof(q), "INTERNAL_ALL:%d\n", val);
        if (!U2_Command_Wait(q, "INTERNAL_ALLOK", -1, NULL, 0, 1000))
            ok = false;

        for (uint8_t local = 1; local <= 4; local++) {
            Motor_Schedule(local, (int16_t)val, (unsigned long)(local - 1) * MOTOR_STAGGER_MS);
        }

        char r[40];
        snprintf(r, sizeof(r), ok ? "ALLOK:%02d:%d\n" : "ALLERR:%02d:%d\n", slave_id, val);
        Send_Response(r);
        return;
    }

    if (strcmp(cmd, "HOME") == 0 && p2) {
        int mID = atoi(p2);
        bool ok = false;

        if (mID >= 1 && mID <= 5) {
            char q[32];
            snprintf(q, sizeof(q), "INTERNAL_HOME:%d\n", mID);
            ok = U2_Command_Wait(q, "INTERNAL_HOMEOK", mID, NULL, 0, HOME_MAX_TIME_MS + 5000);
        } else if (mID >= 6 && mID <= 9) {
            motor_active[mID - 6] = false;
            Motor_Stop((uint8_t)(mID - 5));
            ok = Motor_Home((uint8_t)(mID - 5));
        } else {
            return;
        }

        char r[32];
        snprintf(r, sizeof(r), ok ? "HOMEOK:%02d:%02d\n" : "HOMEERR:%02d:%02d\n", slave_id, mID);
        Send_Response(r);
        return;
    }

    if (strcmp(cmd, "GETPOS") == 0 && p2) {
        int mID = atoi(p2);
        int32_t mm = 0, pulse = 0;
        bool got = false;

        if (mID >= 1 && mID <= 5) {
            char q[32];
            char resp[LINE_BUFFER_LEN];
            snprintf(q, sizeof(q), "INTERNAL_GETPOS:%d\n", mID);
            got = U2_Command_Wait(q, "INTERNAL_POS", mID, resp, sizeof(resp), 1000);
            if (got) {
                char parse[LINE_BUFFER_LEN];
                strncpy(parse, resp, sizeof(parse));
                parse[sizeof(parse) - 1] = '\0';
                (void)strtok(parse, ":");
                (void)strtok(NULL, ":");
                char *pm = strtok(NULL, ":");
                char *pp = strtok(NULL, ":");
                if (pm && pp) {
                    mm = atol(pm);
                    pulse = atol(pp);
                } else {
                    got = false;
                }
            }
        } else if (mID >= 6 && mID <= 9) {
            Service_Encoders();
            uint8_t idx = (uint8_t)(mID - 6);
            pulse = encoder_counters[idx];
            mm    = (int32_t)((float)pulse / CAL_HARDWARE);
            got   = true;
        } else {
            return;
        }

        char r[48];
        if (got)
            snprintf(r, sizeof(r), "POS:%02d:%02d:%ld:%ld\n",
                     slave_id, mID, (long)mm, (long)pulse);
        else
            snprintf(r, sizeof(r), "POSERR:%02d:%02d\n", slave_id, mID);
        Send_Response(r);
        return;
    }
}
