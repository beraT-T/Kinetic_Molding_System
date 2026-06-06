/**
 * ============================================================
 *  ADAPTIF KALIP  v5.1  --  U2 KARTI (5 MOTOR: 1..5)  "ISCI DUGUM"
 * ------------------------------------------------------------
 *  - RS485 YOK. Sadece U3 ile USART2 (PA2/PA3).
 *  - TUM hareket NON-BLOCKING ve es zamanli (1 sn kademeli baslar).
 *  - Komut gelince HEMEN ack doner; bitis STAT ile sorulur (async).
 *
 *  DURUM MAKINESI: IDLE -> MOVING/HOMING -> SETTLED (veya FAULT)
 *
 *  HOME mantigi (entegre limit switch'li aktuator):
 *   Motor retract yonune surulur. Aktuator fiziksel 0'da kendi gucunu
 *   keser -> encoder degismez. Kalkis payi sonrasi encoder HOME_SETTLE_MS
 *   boyunca degismezse "fiziksel 0'a varildi" kabul edip sayaci sifirlar.
 *   Timeout tam strok suresinden (600mm/4mm/s=150s) buyuk: 200s.
 *
 *  INTERNAL komutlar (U3 -> U2):
 *   INTERNAL_MOV:id:mm        tek motor hedef
 *   INTERNAL_ALL:mm           5 motor ayni hedef (kademeli)
 *   INTERNAL_ARR:p1:..:p5     5 motora ayri hedef (kademeli)
 *   INTERNAL_HOME:id          tek motor home
 *   INTERNAL_HOMEALL          5 motor home (kademeli)
 *   INTERNAL_GETPOS:id        tek motor pozisyon
 *   INTERNAL_STAT             5 motorun durum+pozisyonu
 * ============================================================
 */

#include <Arduino.h>
#include <string.h>
#include <stdlib.h>

// ------------------------- AYARLAR --------------------------
#define NUM_MOTORS        5
#define CAL_HARDWARE      79.93f     // puls/mm (cetvelle dogrula)
#define MOTOR_MAX_MM      600

#define STOP_BAND         20         // hedef tolerans (puls) ~0.25mm
#define REARM_BAND        60         // histerezis yeniden-tetik (puls)
#define MOVE_TIMEOUT_MS   200000     // bir hareket en fazla (150s strok + pay)
#define STAGGER_MS        1000       // motorlar arasi baslama (demeraj icin)

#define STALL_WINDOW_MS   1500       // MOV'da bu surede hareket yoksa FAULT
#define STALL_MIN_PULSES  10

#define HOME_MAX_MS       200000     // home timeout (tam strok + pay)
#define HOME_GRACE_MS     3000       // kalkis payi: bu sureden once "durdu" sayilmaz
#define HOME_SETTLE_MS    1200       // encoder bu kadar degismezse fiziksel 0
#define HOME_MOVE_PULSES  3          // hareket esigi

#define ENC_SERVICE_MS    5
#define LINE_LEN          96
#define LED_PIN           PC13

enum { ST_IDLE = 0, ST_MOVING, ST_HOMING, ST_SETTLED, ST_FAULT };

// ------------------------ MOTOR TIPI ------------------------
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
    int32_t       refCount;       // stall/home hareket referansi
    unsigned long refTime;
};

Motor M[NUM_MOTORS] = {
    // pinF , pinR , tim , enc32, encReversed, homeRetractFwd, cal(puls/mm)
    { PB14, PB15, TIM1, false, true, true, CAL_HARDWARE },   // M1
    { PB4,  PB5,  TIM2, true,  true, true, CAL_HARDWARE },   // M2
    { PB0,  PB1,  TIM3, false, true, true, CAL_HARDWARE },   // M3
    { PB8,  PB9,  TIM4, false, true, true, CAL_HARDWARE },   // M4
    { PC14, PC15, TIM5, true,  true, true, CAL_HARDWARE },   // M5  <- olcup duzelt
};

HardwareSerial SerialU3(USART2);

char    rxBuf[LINE_LEN];
uint8_t rxIdx = 0;
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
//  ENCODER  (TIM1/2/3/4/5) + giris filtresi
// ============================================================
static void encConfigCommon(TIM_TypeDef* T, bool is32) {
    T->CR1  = 0; T->CNT = 0; T->ARR = is32 ? 0xFFFFFFFFu : 0xFFFFu;
    T->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
    T->CCMR1 |=  (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);
    T->CCMR1 |=  (0xFu << 4) | (0xFu << 12);          // IC1F=IC2F=0xF
    T->SMCR  &= ~TIM_SMCR_SMS;
    T->SMCR  |=  (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1);
    T->CCER  &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC1NP | TIM_CCER_CC2NP);
    T->CCER  |=  (TIM_CCER_CC1P | TIM_CCER_CC2P);
    T->CR1   |=  TIM_CR1_CEN;
}

void Setup_Encoders(void) {
    // TIM1 (PA8, PA9) AF1
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
    GPIOA->MODER |=  (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1);
    GPIOA->AFR[1] &= ~((0xFu << 0) | (0xFu << 4));
    GPIOA->AFR[1] |=  (1u << 0) | (1u << 4);
    encConfigCommon(TIM1, false);

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
//  MOTOR KONTROL
// ============================================================
static inline void mStop(uint8_t i)  { digitalWrite(M[i].pinF, LOW); digitalWrite(M[i].pinR, LOW); }
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
    M[i].refCount = M[i].count;
    M[i].refTime = M[i].startAt;
}

void Motor_ScheduleHome(uint8_t i, unsigned long delay_ms) {
    if (i >= NUM_MOTORS) return;
    M[i].startAt = millis() + delay_ms;
    M[i].state = ST_HOMING;
    M[i].refCount = M[i].count;
    M[i].refTime = M[i].startAt;
}

void Motor_Update_All(void) {
    unsigned long now = millis();
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        Motor& m = M[i];

        if ((m.state == ST_MOVING || m.state == ST_HOMING) && (long)(now - m.startAt) < 0) {
            mStop(i); continue;   // kademeli baslama: zamani gelmedi
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
            // hareket var mi?
            if (labs(m.count - m.refCount) > HOME_MOVE_PULSES) { m.refCount = m.count; m.refTime = now; }
            else if (now - m.startAt <= HOME_GRACE_MS) { m.refTime = now; }  // grace: settle sayaci baslamasin
            // kalkis payi gecti VE encoder bir suredir degismiyor -> fiziksel 0
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

// ============================================================
//  DURUM RAPORU (STAT)
// ============================================================
char stateLetter(uint8_t s) {
    switch (s) { case ST_MOVING: return 'M'; case ST_HOMING: return 'H';
                 case ST_SETTLED: return 'S'; case ST_FAULT: return 'F'; default: return 'I'; }
}
// "<L><mm>,<L><mm>,..." 5 motor -> dst
void buildStatus(char* dst, size_t n) {
    dst[0] = '\0';
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        int32_t mm = (int32_t)((float)M[i].count / M[i].cal);
        char tok[16];
        snprintf(tok, sizeof(tok), "%s%c%ld", (i ? "," : ""), stateLetter(M[i].state), (long)mm);
        strncat(dst, tok, n - strlen(dst) - 1);
    }
}

// ============================================================
//  HABERLESME
// ============================================================
void LED_Blink(uint8_t n)      { for (uint8_t i=0;i<n;i++){digitalWrite(LED_PIN,LOW);delay(40);digitalWrite(LED_PIN,HIGH);delay(40);} }
void LED_Blink_Slow(uint8_t n) { for (uint8_t i=0;i<n;i++){digitalWrite(LED_PIN,LOW);delay(300);digitalWrite(LED_PIN,HIGH);delay(300);} }

bool ReadLine(char* dst, uint8_t* pos, size_t maxlen) {
    while (SerialU3.available()) {
        char ch = (char)SerialU3.read();
        if (ch == '\n') { dst[*pos] = '\0'; *pos = 0; return true; }
        else if (ch != '\r') { if (*pos < maxlen - 1) dst[(*pos)++] = ch; }
    }
    return false;
}

void Process_Packet(char* line) {
    char tmp[LINE_LEN];
    strncpy(tmp, line, LINE_LEN); tmp[LINE_LEN-1] = '\0';
    char* cmd = strtok(tmp, ":");
    if (!cmd) return;

    if (strcmp(cmd, "INTERNAL_MOV") == 0) {
        char* a = strtok(NULL, ":"); char* b = strtok(NULL, ":");
        if (a && b) {
            int id = atoi(a), val = atoi(b);
            if (id >= 1 && id <= NUM_MOTORS && val >= 0 && val <= MOTOR_MAX_MM) {
                Motor_ScheduleMove((uint8_t)(id-1), (int16_t)val, 0);
                char r[40]; snprintf(r, sizeof(r), "INTERNAL_MOVOK:%d:%d\n", id, val);
                SerialU3.print(r);
            }
        }
        return;
    }
    if (strcmp(cmd, "INTERNAL_ALL") == 0) {
        char* a = strtok(NULL, ":");
        if (a) {
            int val = atoi(a);
            if (val >= 0 && val <= MOTOR_MAX_MM) {
                for (uint8_t i = 0; i < NUM_MOTORS; i++)
                    Motor_ScheduleMove(i, (int16_t)val, (unsigned long)i * STAGGER_MS);
                char r[32]; snprintf(r, sizeof(r), "INTERNAL_ALLOK:%d\n", val);
                SerialU3.print(r);
            }
        }
        return;
    }
    if (strcmp(cmd, "INTERNAL_ARR") == 0) {
        // 5 hedef, kademeli
        int vals[NUM_MOTORS]; uint8_t cnt = 0;
        for (uint8_t i = 0; i < NUM_MOTORS; i++) {
            char* p = strtok(NULL, ":");
            if (!p) break;
            vals[i] = atoi(p); cnt++;
        }
        if (cnt == NUM_MOTORS) {
            for (uint8_t i = 0; i < NUM_MOTORS; i++)
                Motor_ScheduleMove(i, (int16_t)vals[i], (unsigned long)i * STAGGER_MS);
            SerialU3.print("INTERNAL_ARROK\n");
        }
        return;
    }
    if (strcmp(cmd, "INTERNAL_HOME") == 0) {
        char* a = strtok(NULL, ":");
        if (a) {
            int id = atoi(a);
            if (id >= 1 && id <= NUM_MOTORS) {
                Motor_ScheduleHome((uint8_t)(id-1), 0);
                char r[32]; snprintf(r, sizeof(r), "INTERNAL_HOMEOK:%d\n", id);
                SerialU3.print(r);
            }
        }
        return;
    }
    if (strcmp(cmd, "INTERNAL_HOMEALL") == 0) {
        for (uint8_t i = 0; i < NUM_MOTORS; i++)
            Motor_ScheduleHome(i, (unsigned long)i * STAGGER_MS);
        SerialU3.print("INTERNAL_HOMEALLOK\n");
        return;
    }
    if (strcmp(cmd, "INTERNAL_GETPOS") == 0) {
        char* a = strtok(NULL, ":");
        if (a) {
            int id = atoi(a);
            if (id >= 1 && id <= NUM_MOTORS) {
                int32_t pulse = M[id-1].count;
                int32_t mm = (int32_t)((float)pulse / M[id-1].cal);
                char r[48]; snprintf(r, sizeof(r), "INTERNAL_POS:%d:%ld:%ld\n", id, (long)mm, (long)pulse);
                SerialU3.print(r);
            }
        }
        return;
    }
    if (strcmp(cmd, "INTERNAL_STAT") == 0) {
        char st[80]; buildStatus(st, sizeof(st));
        char r[96]; snprintf(r, sizeof(r), "INTERNAL_STAT:%s\n", st);
        SerialU3.print(r);
        return;
    }
}

// ============================================================
//  SETUP / LOOP
// ============================================================
void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    LED_Blink(5);

    SerialU3.begin(115200);

    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        pinMode(M[i].pinF, OUTPUT); pinMode(M[i].pinR, OUTPUT);
        M[i].count = 0; M[i].targetPulse = 0; M[i].state = ST_IDLE;
    }
    mStopAll();

    Setup_Encoders();
    for (uint8_t i = 0; i < NUM_MOTORS; i++) M[i].lastRaw = encRaw(i);

    LED_Blink_Slow(3);
}

void loop() {
    Service_Encoders();
    Motor_Update_All();
    if (ReadLine(rxBuf, &rxIdx, LINE_LEN)) {
        LED_Blink(1);
        Process_Packet(rxBuf);
    }
}
