/**
 * STM32F103 MASTER — Şeffaf RS485 Köprüsü - v4.1
 *
 * GÖREV: PC (USB) ↔ RS485 hattı arasında şeffaf köprü.
 *   - PC'den gelen her satır (\n ile biten) RS485'e iletilir.
 *   - RS485'ten gelen her byte USB'ye iletilir.
 *   - Tüm zamanlama ve handshake mantığı Python UI'da.
 *
 * DÜZELTİLENLER (v4.0 → v4.1):
 *   [1] Sabit delay(4500), delay(5000) kaldırıldı — UI zamanlıyor
 *   [2] String sınıfı yerine char dizisi — heap fragmantasyon yok
 *   [3] DE/RE geçişi flush() sonrası — son bit garanti çıkıyor
 *   [4] LED aktivite göstergesi iyileştirildi
 */

#include <Arduino.h>

// --- PIN TANIMLARI ---
#define RS485_DE_RE_PIN PA4
#define RS485_TX_PIN    PA2
#define RS485_RX_PIN    PA3
#define BAUD_RATE       9600
#define LED_PIN         PC13   // BluePill onboard LED — active LOW

HardwareSerial Serial485(PA3, PA2);

// Satır tamponu
#define USB_BUF_LEN 80
char    usb_buf[USB_BUF_LEN];
uint8_t usb_idx = 0;

void rs485_tx_mode(void) {
    digitalWrite(RS485_DE_RE_PIN, HIGH);
    delayMicroseconds(200); // DE oturması
}
void rs485_rx_mode(void) {
    Serial485.flush();    // Son bitin yazılım FIFO'dan çıktığından emin ol
    // 9600 baud'da 1 byte = ~1.04ms. Son byte'ın HW shift register'dan
    // çıkması için ekstra bekleme. Çok kritik: az olursa son karakter bozulur.
    delay(3);
    digitalWrite(RS485_DE_RE_PIN, LOW);

    // === ÇOK KRİTİK: TX sırasında RX hattı yüzer durumdaydı, UART sahte
    // start bit algılayıp RX buffer'a çöp byte'lar koymuş olabilir. Slave
    // cevabı vermeden önce 2ms boyunca SÜREKLİ drain ediyoruz.
    // (Slave Send_Response'da 5ms bekliyor, yani 2ms drain güvenli.)
    unsigned long drain_until = millis() + 2;
    while ((long)(drain_until - millis()) > 0) {
        while (Serial485.available()) (void)Serial485.read();
    }
}

void setup() {
    Serial.begin(115200);     // USB (PC ile)
    Serial485.begin(BAUD_RATE); // RS485 (Slave'lerle)

    pinMode(RS485_DE_RE_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); // Kapalı (active LOW)

    rs485_rx_mode();

    // NOT: while(!Serial) KALDIRILDI — DTR sinyali olmadan sonsuza kadar
    // beklemesini önler. Serial.println başarısız olsa bile sorun değil.
    Serial.println("MASTER_READY");
    // Hazır sinyali: 2 blink
    for (uint8_t i = 0; i < 2; i++) {
        digitalWrite(LED_PIN, LOW);  delay(100);
        digitalWrite(LED_PIN, HIGH); delay(100);
    }
}

void loop() {
    // ── A: USB → RS485 ─────────────────────────────────────
    // Satır tamponlama: \n gelince hattı gönder
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (usb_idx < USB_BUF_LEN - 1) usb_buf[usb_idx++] = c;

        if (c == '\n') {
            usb_buf[usb_idx] = '\0';
            usb_idx = 0;

            digitalWrite(LED_PIN, LOW);   // TX sinyali
            rs485_tx_mode();
            Serial485.write((uint8_t*)usb_buf, strlen(usb_buf));
            rs485_rx_mode();
            digitalWrite(LED_PIN, HIGH);
        }
    }

    // ── B: RS485 → USB ─────────────────────────────────────
    // Slave'den gelen her byte direkt USB'ye aktar
    while (Serial485.available()) {
        digitalWrite(LED_PIN, LOW);
        Serial.write((uint8_t)Serial485.read());
        digitalWrite(LED_PIN, HIGH);
    }
}
