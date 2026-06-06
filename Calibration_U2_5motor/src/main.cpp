/**
 * ============================================================
 *  KALIBRASYON / DONANIM TESTI  --  U2 KARTI (5 MOTOR: 1..5)
 * ------------------------------------------------------------
 *  AMAC: Her motorun ve her encoder'in yonunu TEK TEK, kesin
 *  olarak olcmek. Calisan v4.4 firmware'i ile AYNI pin ve timer
 *  haritasini kullanir, ama hicbir U3 haberlesmesi yapmaz.
 *
 *  NASIL KULLANILIR:
 *   1) ST-Link ile bu projeyi yukle.
 *   2) Kartin USB'sini bilgisayara tak, seri monitoru 115200'de ac.
 *   3) 'h' yaz -> komut listesi gelir.
 *
 *  KOMUTLAR:
 *   h        -> yardim
 *   l        -> tum encoder sayaclarini goster
 *   z        -> tum sayaclari sifirla
 *   s        -> TUM motorlari durdur (acil)
 *   t<n>     -> Motor n'i OTOMATIK test et (ileri sur, olc, geri sur)
 *   f<n>     -> Motor n'i kisa sure ILERI sur
 *   r<n>     -> Motor n'i kisa sure GERI sur
 *   ta       -> tum motorlari sirayla test et
 *   n = 1,2,3,4,5  (ornek: t1, f5, r3)
 * ============================================================
 */

#include <Arduino.h>
#include <stdlib.h>

#define DBG Serial          // USB yoksa Serial1 yap

#define TEST_DRIVE_MS   1200
#define JOG_MS          700
#define MOVE_NOISE      8
#define LED_PIN         PC13

// ============================================================
//  SAAT YAPILANDIRMASI  (slave1 ile ayni, HSE 25MHz + HSI fallback)
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
    osc.PLL.PLLM = 25; osc.PLL.PLLN = 336;
    osc.PLL.PLLP = RCC_PLLP_DIV4; osc.PLL.PLLQ = 7;

    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        RCC_OscInitTypeDef hsi = {0};
        hsi.OscillatorType = RCC_OSCILLATORTYPE_HSI;
        hsi.HSIState = RCC_HSI_ON;
        hsi.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
        hsi.PLL.PLLState = RCC_PLL_ON;
        hsi.PLL.PLLSource = RCC_PLLSOURCE_HSI;
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
//  MOTOR & ENCODER HARITASI  (slave2 / U2 ile birebir ayni)
//  Yerel index 0..4  ->  fiziksel motor 1,2,3,4,5
// ============================================================
const uint8_t  NUM = 5;
const uint8_t  MOTOR_LABEL[NUM] = { 1, 2, 3, 4, 5 };

const uint32_t MOTOR_F[NUM] = { PB14, PB4, PB0, PB8, PC14 };
const uint32_t MOTOR_R[NUM] = { PB15, PB5, PB1, PB9, PC15 };

TIM_TypeDef*   ENC_TIM[NUM]   = { TIM1, TIM2, TIM3, TIM4, TIM5 };
const bool     ENC_32BIT[NUM] = { false, true, false, false, true };  // TIM2/TIM5 = 32-bit

volatile int32_t enc_count[NUM] = {0,0,0,0,0};
int32_t          enc_last[NUM]  = {0,0,0,0,0};

// ============================================================
//  ENCODER KURULUMU  (slave2'den birebir kopya - kanitli)
// ============================================================
void Setup_Encoders(void) {
    // TIM1 (PA8, PA9) AF1 - 16-bit
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
    GPIOA->MODER |=  (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1);
    GPIOA->AFR[1] &= ~((0xFu << ((8-8)*4)) | (0xFu << ((9-8)*4)));
    GPIOA->AFR[1] |=  (1u << ((8-8)*4)) | (1u << ((9-8)*4));
    TIM1->CR1 = 0; TIM1->CNT = 0; TIM1->ARR = 0xFFFF;
    TIM1->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
    TIM1->CCMR1 |=  (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);
    TIM1->SMCR &= ~TIM_SMCR_SMS; TIM1->SMCR |= (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1);
    TIM1->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC1NP | TIM_CCER_CC2NP);
    TIM1->CCER |=  (TIM_CCER_CC1P | TIM_CCER_CC2P);
    TIM1->CR1 |= TIM_CR1_CEN;

    // TIM2 (PA5, PB3) AF1 - 32-bit
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    GPIOA->MODER &= ~GPIO_MODER_MODER5;   GPIOA->MODER |= GPIO_MODER_MODER5_1;
    GPIOA->AFR[0] &= ~(0xFu << (5*4));    GPIOA->AFR[0] |= (1u << (5*4));
    GPIOB->MODER &= ~GPIO_MODER_MODER3;   GPIOB->MODER |= GPIO_MODER_MODER3_1;
    GPIOB->AFR[0] &= ~(0xFu << (3*4));    GPIOB->AFR[0] |= (1u << (3*4));
    TIM2->CR1 = 0; TIM2->CNT = 0; TIM2->ARR = 0xFFFFFFFFu;
    TIM2->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
    TIM2->CCMR1 |=  (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);
    TIM2->SMCR &= ~TIM_SMCR_SMS; TIM2->SMCR |= (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1);
    TIM2->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC1NP | TIM_CCER_CC2NP);
    TIM2->CCER |=  (TIM_CCER_CC1P | TIM_CCER_CC2P);
    TIM2->CR1 |= TIM_CR1_CEN;

    // TIM3 (PA6, PA7) AF2 - 16-bit
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    GPIOA->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    GPIOA->MODER |=  (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);
    GPIOA->AFR[0] &= ~((0xFu << (6*4)) | (0xFu << (7*4)));
    GPIOA->AFR[0] |=  (2u << (6*4)) | (2u << (7*4));
    TIM3->CR1 = 0; TIM3->CNT = 0; TIM3->ARR = 0xFFFF;
    TIM3->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
    TIM3->CCMR1 |=  (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);
    TIM3->SMCR &= ~TIM_SMCR_SMS; TIM3->SMCR |= (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1);
    TIM3->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC1NP | TIM_CCER_CC2NP);
    TIM3->CCER |=  (TIM_CCER_CC1P | TIM_CCER_CC2P);
    TIM3->CR1 |= TIM_CR1_CEN;

    // TIM4 (PB6, PB7) AF2 - 16-bit
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    GPIOB->MODER |=  (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);
    GPIOB->AFR[0] &= ~((0xFu << (6*4)) | (0xFu << (7*4)));
    GPIOB->AFR[0] |=  (2u << (6*4)) | (2u << (7*4));
    TIM4->CR1 = 0; TIM4->CNT = 0; TIM4->ARR = 0xFFFF;
    TIM4->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
    TIM4->CCMR1 |=  (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);
    TIM4->SMCR &= ~TIM_SMCR_SMS; TIM4->SMCR |= (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1);
    TIM4->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC1NP | TIM_CCER_CC2NP);
    TIM4->CCER |=  (TIM_CCER_CC1P | TIM_CCER_CC2P);
    TIM4->CR1 |= TIM_CR1_CEN;

    // TIM5 (PA0, PA1) AF2 - 32-bit
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    GPIOA->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1);
    GPIOA->MODER |=  (GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1);
    GPIOA->AFR[0] &= ~((0xFu << (0*4)) | (0xFu << (1*4)));
    GPIOA->AFR[0] |=  (2u << (0*4)) | (2u << (1*4));
    TIM5->CR1 = 0; TIM5->CNT = 0; TIM5->ARR = 0xFFFFFFFFu;
    TIM5->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
    TIM5->CCMR1 |=  (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);
    TIM5->SMCR &= ~TIM_SMCR_SMS; TIM5->SMCR |= (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1);
    TIM5->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC1NP | TIM_CCER_CC2NP);
    TIM5->CCER |=  (TIM_CCER_CC1P | TIM_CCER_CC2P);
    TIM5->CR1 |= TIM_CR1_CEN;
}

int32_t encRaw(uint8_t i) {
    TIM_TypeDef* T = ENC_TIM[i];
    return ENC_32BIT[i] ? (int32_t)T->CNT : (int32_t)(int16_t)T->CNT;
}

void serviceEncoders(void) {
    for (uint8_t i = 0; i < NUM; i++) {
        int32_t cur = encRaw(i);
        int32_t dif = cur - enc_last[i];
        if (ENC_32BIT[i]) {
            if (dif >  2000000000) dif -= (int32_t)4294967296LL;
            else if (dif < -2000000000) dif += (int32_t)4294967296LL;
        } else {
            if (dif >  30000) dif -= 65536;
            else if (dif < -30000) dif += 65536;
        }
        if (dif) { enc_count[i] += dif; enc_last[i] = cur; }
    }
}

// ============================================================
//  MOTOR KONTROL
// ============================================================
void motorStop(uint8_t i) {
    digitalWrite(MOTOR_F[i], LOW);
    digitalWrite(MOTOR_R[i], LOW);
}
void motorStopAll(void) { for (uint8_t i = 0; i < NUM; i++) motorStop(i); }

void motorDrive(uint8_t i, bool fwd) {
    digitalWrite(MOTOR_F[i],  fwd);
    digitalWrite(MOTOR_R[i], !fwd);
}

int32_t driveFor(uint8_t i, bool fwd, unsigned long ms) {
    serviceEncoders();
    int32_t start = enc_count[i];
    unsigned long t0 = millis();
    digitalWrite(LED_PIN, LOW);
    motorDrive(i, fwd);
    while (millis() - t0 < ms) {
        serviceEncoders();
        if (DBG.available() && DBG.read() == 's') break;
        delay(2);
    }
    motorStop(i);
    digitalWrite(LED_PIN, HIGH);
    serviceEncoders();
    return enc_count[i] - start;
}

void autoTest(uint8_t i) {
    DBG.print("\n--- Motor M"); DBG.print(MOTOR_LABEL[i]); DBG.println(" testi ---");
    DBG.println("Ileri suruluyor...");
    int32_t dF = driveFor(i, true, TEST_DRIVE_MS);
    delay(300);
    DBG.println("Geri suruluyor...");
    int32_t dR = driveFor(i, false, TEST_DRIVE_MS);

    DBG.print("  ILERI surus -> encoder delta = "); DBG.println(dF);
    DBG.print("  GERI  surus -> encoder delta = "); DBG.println(dR);

    if (labs(dF) < MOVE_NOISE && labs(dR) < MOVE_NOISE) {
        DBG.println("  !! MOTOR HAREKET ETMEDI veya ENCODER OKUMUYOR.");
        DBG.println("     Kontrol et: surucu guc, F/R kablolari, encoder kablo/besleme.");
        return;
    }
    if (labs(dF) < MOVE_NOISE || labs(dR) < MOVE_NOISE) {
        DBG.println("  ! Tek yonde hareket var. Motor sinirda olabilir; once r/f ile ortala.");
    }

    if (dF > 0) {
        DBG.print("  SONUC: encoder DUZ  -> MOTOR_ENCODER_REVERSED[M");
        DBG.print(MOTOR_LABEL[i]); DBG.println("] = false");
    } else {
        DBG.print("  SONUC: encoder TERS -> MOTOR_ENCODER_REVERSED[M");
        DBG.print(MOTOR_LABEL[i]); DBG.println("] = true");
    }
    DBG.println("  (Home yonu icin f/r ile hangi yonun fiziksel GERI cektigine bak.)");
}

// ============================================================
//  SERI ARAYUZ
// ============================================================
void printHelp(void) {
    DBG.println("\n==================================================");
    DBG.println(" U2 KALIBRASYON - 5 MOTOR (M1..M5)");
    DBG.println("==================================================");
    DBG.println(" h      : yardim");
    DBG.println(" l      : encoder sayaclarini goster");
    DBG.println(" z      : sayaclari sifirla");
    DBG.println(" s      : TUM motorlari durdur (acil)");
    DBG.println(" t<n>   : Motor n otomatik test  (orn: t1)");
    DBG.println(" f<n>   : Motor n kisa ILERI sur (orn: f1)");
    DBG.println(" r<n>   : Motor n kisa GERI sur  (orn: r1)");
    DBG.println(" ta     : tum motorlari sirayla test et");
    DBG.println(" n = 1,2,3,4,5");
    DBG.println("==================================================\n");
}

void listEncoders(void) {
    serviceEncoders();
    for (uint8_t i = 0; i < NUM; i++) {
        DBG.print("  M"); DBG.print(MOTOR_LABEL[i]);
        DBG.print("  enc = "); DBG.println(enc_count[i]);
    }
}

uint8_t labelToIndex(int lbl) {
    for (uint8_t i = 0; i < NUM; i++) if (MOTOR_LABEL[i] == lbl) return i;
    return 255;
}

void handleCommand(char* s) {
    if (s[0] == '\0') return;
    char c = s[0];

    if (c == 'h') { printHelp(); return; }
    if (c == 'l') { listEncoders(); return; }
    if (c == 'z') { for (uint8_t i=0;i<NUM;i++){enc_count[i]=0; enc_last[i]=encRaw(i);} DBG.println("Sayaclar sifirlandi."); return; }
    if (c == 's') { motorStopAll(); DBG.println("Tum motorlar durduruldu."); return; }

    if (c == 't' && s[1] == 'a') {
        for (uint8_t i = 0; i < NUM; i++) { autoTest(i); delay(400); }
        return;
    }

    if (c == 't' || c == 'f' || c == 'r') {
        int lbl = atoi(s + 1);
        uint8_t i = labelToIndex(lbl);
        if (i == 255) { DBG.println("Gecersiz motor. (1..5)"); return; }
        if (c == 't') { autoTest(i); }
        else {
            bool fwd = (c == 'f');
            DBG.print("M"); DBG.print(lbl);
            DBG.println(fwd ? " ILERI..." : " GERI...");
            int32_t d = driveFor(i, fwd, JOG_MS);
            DBG.print("  encoder delta = "); DBG.println(d);
        }
        return;
    }
    DBG.println("Bilinmeyen komut. 'h' yaz.");
}

// ============================================================
//  SETUP / LOOP
// ============================================================
char  cmd[16];
uint8_t ci = 0;

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    for (uint8_t i = 0; i < NUM; i++) {
        pinMode(MOTOR_F[i], OUTPUT);
        pinMode(MOTOR_R[i], OUTPUT);
    }
    motorStopAll();

    Setup_Encoders();
    for (uint8_t i = 0; i < NUM; i++) enc_last[i] = encRaw(i);

    DBG.begin(115200);
    delay(800);
    printHelp();
}

void loop() {
    serviceEncoders();

    while (DBG.available()) {
        char ch = (char)DBG.read();
        if (ch == '\n' || ch == '\r') {
            cmd[ci] = '\0';
            handleCommand(cmd);
            ci = 0;
        } else if (ch == 's' && ci == 0) {
            motorStopAll();
            DBG.println("[ACIL] Durduruldu.");
        } else if (ci < sizeof(cmd) - 1) {
            cmd[ci++] = ch;
        }
    }
}
