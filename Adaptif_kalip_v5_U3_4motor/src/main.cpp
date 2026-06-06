/**
 * ============================================================
 *  ADAPTIF KALIP  v5.1  --  U3 KARTI (4 MOTOR: 6..9)  "YONETICI"
 * ------------------------------------------------------------
 *  MIMARI:  PC/Pi -> F103 RS485 koprusu -> RS485 -> U3 -> USART2 -> U2
 *   - Motor 6-9 yerel (TIM2/3/4/5), motor 1-5 U2'de.
 *   - TUM hareket NON-BLOCKING, es zamanli, 1 sn kademeli.
 *   - Komut gelince HEMEN ack doner; bitis STAT ile sorulur (async).
 *   - 9 motorun kademe slotu: 1-5 -> slot 0..4 (U2), 6-9 -> slot 5..8 (U3).
 *
 *  RS485 KOMUTLARI (host -> U3):
 *   PING:id / GETID:id / SETID:id:newid
 *   MOV:id:motor:mm           tek motor (test)
 *   ALL:id:mm                 9 motor ayni hedef (kademeli)
 *   ARR:id:p1:..:p9           9 motora ayri hedef (STL array'i)
 *   HOME:id                   9 motor home (kademeli)
 *   HOME:id:motor             tek motor home (test)
 *   GETPOS:id:motor           tek motor pozisyon
 *   STAT:id                   9 motorun durum+pozisyonu
 *
 *  Ack'ler "BASLADI" anlamindadir. Bitis icin STAT poll edilir.
 *  STAT cevabi:  STAT:id:<L mm>,<L mm>,... (9 token; L = I/M/H/S/F)
 * ============================================================
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include <string.h>
#include <stdlib.h>
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"

// ------------------------- AYARLAR --------------------------
#define NUM_MOTORS        4          // yerel: motor 6,7,8,9
#define LOCAL_SLOT_BASE   5          // 1-5 U2'de -> yerel motorlar slot 5'ten baslar
#define CAL_HARDWARE      79.93f
#define MOTOR_MAX_MM      600

#define STOP_BAND         20
#define REARM_BAND        60
#define MOVE_TIMEOUT_MS   200000
#define STAGGER_MS        1000

#define STALL_WINDOW_MS   1500
#define STALL_MIN_PULSES  10

#define HOME_MAX_MS       200000
#define HOME_GRACE_MS     3000
#define HOME_SETTLE_MS    1200
#define HOME_MOVE_PULSES  3

#define ENC_SERVICE_MS    5
#define LINE_LEN          96
#define LED_PIN           PC13

#define SLAVE_ID_DEFAULT  1
#define SLAVE_ID_MAGIC    0xABCD1234u
#define RS485_DE_RE_PIN   PA8

enum { ST_IDLE = 0, ST_MOVING, ST_HOMING, ST_SETTLED, ST_FAULT };

struct Motor {
    uint32_t      pinF, pinR;
    TIM_TypeDef*  tim;
    bool          enc32;
    bool          encReversed;
    bool          homeRetractFwd;
    float         cal;            // puls/mm (motor basina ayri ayarlanabilir)
    int32_t       count;
    int32_t       lastRaw;
    int32_t       targetPulse;
    uint8_t       state;
    unsigned long startAt;
    int32_t       refCount;
    unsigned long refTime;
};

// Yerel index 0..3 -> fiziksel motor 6,7,8,9
Motor M[NUM_MOTORS] = {
    // pinF , pinR , tim , enc32, encReversed, homeRetractFwd, cal(puls/mm)
    { PB4,  PB5,  TIM2, true,  true, true, CAL_HARDWARE },   // M6
    { PB0,  PB1,  TIM3, false, true, true, CAL_HARDWARE },   // M7
    { PB8,  PB9,  TIM4, false, true, true, CAL_HARDWARE },   // M8
    { PC14, PC15, TIM5, true,  true, true, CAL_HARDWARE },   // M9
};
const uint8_t LABEL_BASE = 6;

HardwareSerial SerialRS485(PA10, PA9);
HardwareSerial SerialU2(PA3, PA2);

uint8_t  slave_id = SLAVE_ID_DEFAULT;
char     rxBuf[LINE_LEN];
uint8_t  rxIdx = 0;
unsigned long lastEncService = 0;

// ============================================================
//  SAAT
// ============================================================
extern "C" void SystemClock_Config(void) {
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON; osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 25; osc.PLL.PLLN = 336;
    osc.PLL.PLLP = RCC_PLLP_DIV4; osc.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        RCC_OscInitTypeDef hsi = {0};
        hsi.OscillatorType = RCC_OSCILLATORTYPE_HSI;
        hsi.HSIState = RCC_HSI_ON;
        hsi.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
        hsi.PLL.PLLState = RCC_PLL_ON; hsi.PLL.PLLSource = RCC_PLLSOURCE_HSI;
        hsi.PLL.PLLM = 8; hsi.PLL.PLLN = 168;
        hsi.PLL.PLLP = RCC_PLLP_DIV4; hsi.PLL.PLLQ = 7;
        if (HAL_RCC_OscConfig(&hsi) != HAL_OK) return;
    }
    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2);
}

// ============================================================
//  FLASH - Slave ID
// ============================================================
static uint32_t Flash_SizeKB(void) {
    uint32_t kb = (uint32_t)(*(volatile uint16_t*)0x1FFF7A22u);
    if (kb == 0u || kb > 512u) kb = 256u;
    return kb;
}
static uint32_t SlaveID_FlashAddr(void) { return 0x08000000u + Flash_SizeKB()*1024u - 4u; }
static uint32_t AddrToSector(uint32_t a) {
    if (a < 0x08004000u) return FLASH_SECTOR_0;
    if (a < 0x08008000u) return FLASH_SECTOR_1;
    if (a < 0x0800C000u) return FLASH_SECTOR_2;
    if (a < 0x08010000u) return FLASH_SECTOR_3;
    if (a < 0x08020000u) return FLASH_SECTOR_4;
    return FLASH_SECTOR_5;
}
static uint8_t Flash_Read_SlaveID(void) {
    uint32_t w = *(volatile uint32_t*)SlaveID_FlashAddr();
    if (w == 0xFFFFFFFFu || w == 0u) return SLAVE_ID_DEFAULT;
    uint32_t magic = (w >> 16) & 0xFFFFu, id = w & 0xFFFFu;
    if (magic == (SLAVE_ID_MAGIC & 0xFFFFu) && id >= 1u && id <= 16u) return (uint8_t)id;
    return SLAVE_ID_DEFAULT;
}
static bool Flash_Write_SlaveID(uint8_t id) {
    if (id < 1u || id > 16u) return false;
    uint32_t a = SlaveID_FlashAddr();
    uint32_t neww = ((SLAVE_ID_MAGIC & 0xFFFFu) << 16) | (uint32_t)id;
    if (*(volatile uint32_t*)a == neww) return true;
    if (HAL_FLASH_Unlock() != HAL_OK) return false;
    FLASH_EraseInitTypeDef ei; memset(&ei, 0, sizeof(ei));
    uint32_t err = 0;
    ei.TypeErase = FLASH_TYPEERASE_SECTORS; ei.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    ei.Sector = AddrToSector(a); ei.NbSectors = 1;
    if (HAL_FLASHEx_Erase(&ei, &err) != HAL_OK) { HAL_FLASH_Lock(); return false; }
    HAL_StatusTypeDef st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, a, neww);
    HAL_FLASH_Lock();
    return st == HAL_OK;
}

// ============================================================
//  ENCODER  (TIM2/3/4/5) + giris filtresi
// ============================================================
static void encConfigCommon(TIM_TypeDef* T, bool is32) {
    T->CR1  = 0; T->CNT = 0; T->ARR = is32 ? 0xFFFFFFFFu : 0xFFFFu;
    T->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
    T->CCMR1 |=  (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);
    T->CCMR1 |=  (0xFu << 4) | (0xFu << 12);
    T->SMCR  &= ~TIM_SMCR_SMS;
    T->SMCR  |=  (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1);
    T->CCER  &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC1NP | TIM_CCER_CC2NP);
    T->CCER  |=  (TIM_CCER_CC1P | TIM_CCER_CC2P);
    T->CR1   |=  TIM_CR1_CEN;
}

void Setup_Encoders(void) {
    // TIM2 (PA5, PB3) AF1
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    GPIOA->MODER &= ~GPIO_MODER_MODER5;   GPIOA->MODER |= GPIO_MODER_MODER5_1;
    GPIOA->AFR[0] &= ~(0xFu << (5*4));    GPIOA->AFR[0] |= (1u << (5*4));
    GPIOB->MODER &= ~GPIO_MODER_MODER3;   GPIOB->MODER |= GPIO_MODER_MODER3_1;
    GPIOB->AFR[0] &= ~(0xFu << (3*4));    GPIOB->AFR[0] |= (1u << (3*4));
    encConfigCommon(TIM2, true);

    // TIM3 (PA6, PA7) AF2
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    GPIOA->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    GPIOA->MODER |=  (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);
    GPIOA->AFR[0] &= ~((0xFu << (6*4)) | (0xFu << (7*4)));
    GPIOA->AFR[0] |=  (2u << (6*4)) | (2u << (7*4));
    encConfigCommon(TIM3, false);

    // TIM4 (PB6, PB7) AF2
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    GPIOB->MODER |=  (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);
    GPIOB->AFR[0] &= ~((0xFu << (6*4)) | (0xFu << (7*4)));
    GPIOB->AFR[0] |=  (2u << (6*4)) | (2u << (7*4));
    encConfigCommon(TIM4, false);

    // TIM5 (PA0, PA1) AF2
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    GPIOA->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1);
    GPIOA->MODER |=  (GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1);
    GPIOA->AFR[0] &= ~((0xFu << (0*4)) | (0xFu << (1*4)));
    GPIOA->AFR[0] |=  (2u << (0*4)) | (2u << (1*4));
    encConfigCommon(TIM5, true);
}

static inline int32_t encRaw(uint8_t i) {
    TIM_TypeDef* T = M[i].tim;
    return M[i].enc32 ? (int32_t)T->CNT : (int32_t)(int16_t)T->CNT;
}

void Service_Encoders(void) {
    unsigned long now = millis();
    if (now - lastEncService < ENC_SERVICE_MS) return;
    lastEncService = now;
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        int32_t cur = encRaw(i);
        int32_t dif = cur - M[i].lastRaw;
        if (M[i].enc32) {
            if (dif >  2000000000) dif -= (int32_t)4294967296LL;
            else if (dif < -2000000000) dif += (int32_t)4294967296LL;
        } else {
            if (dif >  30000) dif -= 65536;
            else if (dif < -30000) dif += 65536;
        }
        if (dif) { M[i].count += dif; M[i].lastRaw = cur; }
    }
}

// ============================================================
//  MOTOR KONTROL  (U2 ile ayni cekirdek)
// ============================================================
static inline void mStop(uint8_t i) { digitalWrite(M[i].pinF, LOW); digitalWrite(M[i].pinR, LOW); }
void mStopAll(void) { for (uint8_t i = 0; i < NUM_MOTORS; i++) { mStop(i); M[i].state = ST_IDLE; } }

static inline void mDriveToward(uint8_t i, int32_t error) {
    bool fwd = M[i].encReversed ? (error < 0) : (error > 0);
    digitalWrite(M[i].pinF, fwd); digitalWrite(M[i].pinR, !fwd);
}
static inline void mDriveHome(uint8_t i) {
    bool fwd = M[i].homeRetractFwd;
    digitalWrite(M[i].pinF, fwd); digitalWrite(M[i].pinR, !fwd);
}

void Motor_ScheduleMove(uint8_t i, int16_t mm, unsigned long delay_ms) {
    if (i >= NUM_MOTORS) return;
    if (mm < 0) mm = 0; if (mm > MOTOR_MAX_MM) mm = MOTOR_MAX_MM;
    M[i].targetPulse = (int32_t)((float)mm * M[i].cal);
    M[i].startAt = millis() + delay_ms;
    M[i].state = ST_MOVING;
    M[i].refCount = M[i].count; M[i].refTime = M[i].startAt;
}
void Motor_ScheduleHome(uint8_t i, unsigned long delay_ms) {
    if (i >= NUM_MOTORS) return;
    M[i].startAt = millis() + delay_ms;
    M[i].state = ST_HOMING;
    M[i].refCount = M[i].count; M[i].refTime = M[i].startAt;
}

void Motor_Update_All(void) {
    unsigned long now = millis();
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        Motor& m = M[i];
        if ((m.state == ST_MOVING || m.state == ST_HOMING) && (long)(now - m.startAt) < 0) {
            mStop(i); continue;
        }
        switch (m.state) {
        case ST_MOVING: {
            int32_t error = m.targetPulse - m.count;
            if (labs(error) <= STOP_BAND) { mStop(i); m.state = ST_SETTLED; break; }
            if (now - m.startAt > MOVE_TIMEOUT_MS) { mStop(i); m.state = ST_FAULT; break; }
            if (labs(m.count - m.refCount) >= STALL_MIN_PULSES) { m.refCount = m.count; m.refTime = now; }
            else if (now - m.refTime > STALL_WINDOW_MS) { mStop(i); m.state = ST_FAULT; break; }
            mDriveToward(i, error);
            break;
        }
        case ST_HOMING: {
            if (labs(m.count - m.refCount) > HOME_MOVE_PULSES) { m.refCount = m.count; m.refTime = now; }
            else if (now - m.startAt <= HOME_GRACE_MS) { m.refTime = now; }  // grace: settle sayaci baslamasin
            if ((now - m.startAt > HOME_GRACE_MS) && (now - m.refTime > HOME_SETTLE_MS)) {
                mStop(i); delay(2); Service_Encoders();
                m.count = 0; m.lastRaw = encRaw(i); m.targetPulse = 0;
                m.state = ST_SETTLED; break;
            }
            if (now - m.startAt > HOME_MAX_MS) { mStop(i); m.state = ST_FAULT; break; }
            mDriveHome(i);
            break;
        }
        case ST_SETTLED: {
            int32_t error = m.targetPulse - m.count;
            if (labs(error) > REARM_BAND) {
                m.state = ST_MOVING; m.startAt = now;
                m.refCount = m.count; m.refTime = now;
            } else mStop(i);
            break;
        }
        default: mStop(i); break;
        }
    }
}

char stateLetter(uint8_t s) {
    switch (s) { case ST_MOVING: return 'M'; case ST_HOMING: return 'H';
                 case ST_SETTLED: return 'S'; case ST_FAULT: return 'F'; default: return 'I'; }
}
void buildLocalStatus(char* dst, size_t n) {   // 4 token: M6..M9
    dst[0] = '\0';
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        int32_t mm = (int32_t)((float)M[i].count / M[i].cal);
        char tok[16];
        snprintf(tok, sizeof(tok), "%s%c%ld", (i ? "," : ""), stateLetter(M[i].state), (long)mm);
        strncat(dst, tok, n - strlen(dst) - 1);
    }
}

// ============================================================
//  U2 HABERLESME (beklerken yerel motorlar servis edilir)
// ============================================================
void U2_Drain(void) { while (SerialU2.available()) (void)SerialU2.read(); }

bool Wait_U2(const char* expected, char* out, size_t outLen, unsigned long timeout_ms) {
    char buf[LINE_LEN]; uint8_t bi = 0;
    unsigned long deadline = millis() + timeout_ms;
    while ((long)(deadline - millis()) > 0) {
        Service_Encoders();
        Motor_Update_All();
        while (SerialU2.available()) {
            char ch = (char)SerialU2.read();
            if (ch == '\n') {
                buf[bi] = '\0';
                char tmp[LINE_LEN]; strncpy(tmp, buf, sizeof(tmp)); tmp[sizeof(tmp)-1] = '\0';
                char* cmd = strtok(tmp, ":");
                if (cmd && strcmp(cmd, expected) == 0) {
                    if (out && outLen > 0) { strncpy(out, buf, outLen); out[outLen-1] = '\0'; }
                    return true;
                }
                bi = 0;
            } else if (ch != '\r' && bi < LINE_LEN - 1) buf[bi++] = ch;
        }
        delay(1);
    }
    return false;
}
bool U2_Cmd(const char* msg, const char* expected, char* out, size_t outLen, unsigned long timeout_ms) {
    U2_Drain(); SerialU2.print(msg);
    return Wait_U2(expected, out, outLen, timeout_ms);
}

// ============================================================
//  RS485
// ============================================================
void RS485_TX_Mode(void) { digitalWrite(RS485_DE_RE_PIN, HIGH); delayMicroseconds(200); }
void RS485_RX_Mode(void) {
    SerialRS485.flush(); delay(3);
    digitalWrite(RS485_DE_RE_PIN, LOW); delayMicroseconds(500);
    while (SerialRS485.available()) SerialRS485.read();
}
void Send_Response(const char* msg) { delay(5); RS485_TX_Mode(); SerialRS485.print(msg); RS485_RX_Mode(); }

void LED_Blink(uint8_t n)      { for (uint8_t i=0;i<n;i++){digitalWrite(LED_PIN,LOW);delay(40);digitalWrite(LED_PIN,HIGH);delay(40);} }
void LED_Blink_Slow(uint8_t n) { for (uint8_t i=0;i<n;i++){digitalWrite(LED_PIN,LOW);delay(300);digitalWrite(LED_PIN,HIGH);delay(300);} }

bool ReadLine(char* dst, uint8_t* pos, size_t maxlen) {
    while (SerialRS485.available()) {
        char ch = (char)SerialRS485.read();
        if (ch == '\n') { dst[*pos] = '\0'; *pos = 0; return true; }
        else if (ch != '\r') { if (*pos < maxlen - 1) dst[(*pos)++] = ch; }
    }
    return false;
}

// ============================================================
//  KOMUT ISLEME
// ============================================================
void Process_Packet(char* line) {
    char tmp[LINE_LEN]; strncpy(tmp, line, LINE_LEN); tmp[LINE_LEN-1] = '\0';
    char* cmd = strtok(tmp, ":");
    char* p1  = strtok(NULL, ":");
    if (!cmd) return;

    // --- adressiz / id kontrollu temel komutlar ---
    if (strcmp(cmd, "PING") == 0 && p1) {
        if (atoi(p1) != (int)slave_id) return;
        char r[24]; snprintf(r, sizeof(r), "PONG:%02d\n", slave_id); Send_Response(r); return;
    }
    if (strcmp(cmd, "GETID") == 0 && p1) {
        if (atoi(p1) != (int)slave_id) return;
        char r[24]; snprintf(r, sizeof(r), "IDVAL:%02d\n", slave_id); Send_Response(r); return;
    }
    if (strcmp(cmd, "SETID") == 0 && p1) {
        if (atoi(p1) != (int)slave_id) return;
        char* p2 = strtok(NULL, ":");
        if (p2) {
            int nid = atoi(p2);
            if (nid >= 1 && nid <= 16) {
                if (Flash_Write_SlaveID((uint8_t)nid)) {
                    slave_id = (uint8_t)nid;
                    char r[24]; snprintf(r, sizeof(r), "IDSET:%02d\n", slave_id); Send_Response(r);
                } else Send_Response("IDERR:FLASH\n");
            }
        }
        return;
    }

    if (!p1 || atoi(p1) != (int)slave_id) return;

    // --- MOV:id:motor:mm  (tek motor) ---
    if (strcmp(cmd, "MOV") == 0) {
        char* p2 = strtok(NULL, ":"); char* p3 = strtok(NULL, ":");
        if (!p2 || !p3) return;
        int mID = atoi(p2), val = atoi(p3);
        if (val < 0 || val > MOTOR_MAX_MM) return;
        bool ok = false;
        if (mID >= 1 && mID <= 5) {
            char q[40]; snprintf(q, sizeof(q), "INTERNAL_MOV:%d:%d\n", mID, val);
            ok = U2_Cmd(q, "INTERNAL_MOVOK", NULL, 0, 1500);
        } else if (mID >= 6 && mID <= 9) {
            Motor_ScheduleMove((uint8_t)(mID - LABEL_BASE), (int16_t)val, 0); ok = true;
        } else return;
        char r[40]; snprintf(r, sizeof(r), ok ? "MOVOK:%02d:%02d:%d\n" : "MOVERR:%02d:%02d:%d\n", slave_id, mID, val);
        Send_Response(r); return;
    }

    // --- ALL:id:mm  (9 motor ayni hedef, kademeli) ---
    if (strcmp(cmd, "ALL") == 0) {
        char* p2 = strtok(NULL, ":");
        if (!p2) return;
        int val = atoi(p2);
        if (val < 0 || val > MOTOR_MAX_MM) return;
        char q[32]; snprintf(q, sizeof(q), "INTERNAL_ALL:%d\n", val);
        bool ok = U2_Cmd(q, "INTERNAL_ALLOK", NULL, 0, 1500);
        for (uint8_t i = 0; i < NUM_MOTORS; i++)
            Motor_ScheduleMove(i, (int16_t)val, (unsigned long)(LOCAL_SLOT_BASE + i) * STAGGER_MS);
        char r[40]; snprintf(r, sizeof(r), ok ? "ALLOK:%02d:%d\n" : "ALLERR:%02d:%d\n", slave_id, val);
        Send_Response(r); return;
    }

    // --- ARR:id:p1:..:p9  (STL array'i) ---
    if (strcmp(cmd, "ARR") == 0) {
        int vals[9]; uint8_t cnt = 0;
        for (uint8_t i = 0; i < 9; i++) {
            char* p = strtok(NULL, ":");
            if (!p) break;
            vals[i] = atoi(p); cnt++;
        }
        if (cnt != 9) { Send_Response("ARRERR:FMT\n"); return; }
        // 1-5 -> U2
        char q[64];
        snprintf(q, sizeof(q), "INTERNAL_ARR:%d:%d:%d:%d:%d\n", vals[0], vals[1], vals[2], vals[3], vals[4]);
        bool ok = U2_Cmd(q, "INTERNAL_ARROK", NULL, 0, 1500);
        // 6-9 -> yerel (slot 5..8)
        for (uint8_t i = 0; i < NUM_MOTORS; i++)
            Motor_ScheduleMove(i, (int16_t)vals[5 + i], (unsigned long)(LOCAL_SLOT_BASE + i) * STAGGER_MS);
        char r[32]; snprintf(r, sizeof(r), ok ? "ARROK:%02d\n" : "ARRERR:%02d\n", slave_id);
        Send_Response(r); return;
    }

    // --- HOME:id  (hepsi)  /  HOME:id:motor  (tek) ---
    if (strcmp(cmd, "HOME") == 0) {
        char* p2 = strtok(NULL, ":");
        int mID = p2 ? atoi(p2) : 0;
        if (mID == 0) {                 // hepsi
            bool ok = U2_Cmd("INTERNAL_HOMEALL\n", "INTERNAL_HOMEALLOK", NULL, 0, 1500);
            for (uint8_t i = 0; i < NUM_MOTORS; i++)
                Motor_ScheduleHome(i, (unsigned long)(LOCAL_SLOT_BASE + i) * STAGGER_MS);
            char r[32]; snprintf(r, sizeof(r), ok ? "HOMEOK:%02d:00\n" : "HOMEERR:%02d:00\n", slave_id);
            Send_Response(r); return;
        }
        bool ok = false;
        if (mID >= 1 && mID <= 5) {
            char q[32]; snprintf(q, sizeof(q), "INTERNAL_HOME:%d\n", mID);
            ok = U2_Cmd(q, "INTERNAL_HOMEOK", NULL, 0, 1500);
        } else if (mID >= 6 && mID <= 9) {
            Motor_ScheduleHome((uint8_t)(mID - LABEL_BASE), 0); ok = true;
        } else return;
        char r[32]; snprintf(r, sizeof(r), ok ? "HOMEOK:%02d:%02d\n" : "HOMEERR:%02d:%02d\n", slave_id, mID);
        Send_Response(r); return;
    }

    // --- GETPOS:id:motor ---
    if (strcmp(cmd, "GETPOS") == 0) {
        char* p2 = strtok(NULL, ":");
        if (!p2) return;
        int mID = atoi(p2); int32_t mm = 0, pulse = 0; bool got = false;
        if (mID >= 1 && mID <= 5) {
            char q[32], resp[LINE_LEN];
            snprintf(q, sizeof(q), "INTERNAL_GETPOS:%d\n", mID);
            got = U2_Cmd(q, "INTERNAL_POS", resp, sizeof(resp), 1500);
            if (got) {
                char parse[LINE_LEN]; strncpy(parse, resp, sizeof(parse)); parse[sizeof(parse)-1] = '\0';
                (void)strtok(parse, ":"); (void)strtok(NULL, ":");
                char* pm = strtok(NULL, ":"); char* pp = strtok(NULL, ":");
                if (pm && pp) { mm = atol(pm); pulse = atol(pp); } else got = false;
            }
        } else if (mID >= 6 && mID <= 9) {
            uint8_t idx = (uint8_t)(mID - LABEL_BASE);
            pulse = M[idx].count; mm = (int32_t)((float)pulse / M[idx].cal); got = true;
        } else return;
        char r[48];
        if (got) snprintf(r, sizeof(r), "POS:%02d:%02d:%ld:%ld\n", slave_id, mID, (long)mm, (long)pulse);
        else     snprintf(r, sizeof(r), "POSERR:%02d:%02d\n", slave_id, mID);
        Send_Response(r); return;
    }

    // --- STAT:id  (9 motorun durum+pozisyonu) ---
    if (strcmp(cmd, "STAT") == 0) {
        char u2[80] = {0};
        char resp[LINE_LEN];
        bool got = U2_Cmd("INTERNAL_STAT\n", "INTERNAL_STAT", resp, sizeof(resp), 1500);
        if (got) {
            // resp = "INTERNAL_STAT:<5 token>"
            char* c = strchr(resp, ':');
            if (c && *(c+1)) strncpy(u2, c + 1, sizeof(u2) - 1);
        }
        char loc[48]; buildLocalStatus(loc, sizeof(loc));
        char r[LINE_LEN];
        if (got) snprintf(r, sizeof(r), "STAT:%02d:%s,%s\n", slave_id, u2, loc);
        else     snprintf(r, sizeof(r), "STAT:%02d:U2ERR,%s\n", slave_id, loc);
        Send_Response(r); return;
    }
}

// ============================================================
//  SETUP / LOOP
// ============================================================
void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    LED_Blink(5);

    SerialRS485.begin(9600);
    SerialU2.begin(115200);
    pinMode(RS485_DE_RE_PIN, OUTPUT);

    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        pinMode(M[i].pinF, OUTPUT); pinMode(M[i].pinR, OUTPUT);
        M[i].count = 0; M[i].targetPulse = 0; M[i].state = ST_IDLE;
    }
    mStopAll();

    Setup_Encoders();
    for (uint8_t i = 0; i < NUM_MOTORS; i++) M[i].lastRaw = encRaw(i);

    slave_id = Flash_Read_SlaveID();
    RS485_RX_Mode();
    LED_Blink_Slow(slave_id);
}

void loop() {
    Service_Encoders();
    Motor_Update_All();
    if (ReadLine(rxBuf, &rxIdx, LINE_LEN)) {
        digitalWrite(LED_PIN, LOW);
        Process_Packet(rxBuf);
        digitalWrite(LED_PIN, HIGH);
    }
}
