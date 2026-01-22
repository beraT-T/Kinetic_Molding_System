#include <Arduino.h>

/* --- DONANIM AYARLARI --- */
// RS485 Modül Pinleri
#define RS485_DE_RE_PIN PA4  // Kontrol Pini
#define RS485_TX_PIN    PA2
#define RS485_RX_PIN    PA3

// Haberleşme Hızı (Tüm sistemde AYNI olmalı!)
#define BAUD_RATE       9600 

// LED (PC13) - Veri akışını görmek için
#define LED_PIN         PC13

// Serial Nesnesi (PA2/PA3)
HardwareSerial Serial485(PA3, PA2); 

// Veri Tamponları
String usbBuffer = "";
bool stringComplete = false;

void rs485_tx_mode() {
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  delayMicroseconds(50); // Modülün uyanması için güvenli süre
}

void rs485_rx_mode() {
  Serial485.flush(); // Son bitin gittiğinden emin ol
  digitalWrite(RS485_DE_RE_PIN, LOW);
}

void setup() {
  // 1. USB Serial (PC ile)
  Serial.begin(115200); // USB hızı sanaldır, yüksek kalabilir.
  
  // 2. RS485 Serial (Slaves ile)
  Serial485.begin(BAUD_RATE);
  
  // 3. Pin Ayarları
  pinMode(RS485_DE_RE_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  // Başlangıç durumu: Dinleme (RX)
  rs485_rx_mode();
  
  // Hazır sinyali (Sadece PC görür)
  while(!Serial); 
  Serial.println("MASTER_READY");
}

void loop() {
  
  // --- A. PC'DEN GELENİ DİNLE (USB -> RS485) ---
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    // Veriyi tampona ekle
    usbBuffer += inChar;
    
    // Satır sonu karakteri geldiyse paket tamamdır
    if (inChar == '\n') {
      stringComplete = true;
    }
  }

  // Paket tamamlandıysa RS485'e bas
  if (stringComplete) {
    // Komut tipini kontrol et (temizlenmeden önce)
    bool isPing = (usbBuffer.indexOf("PING") >= 0);
    bool isHome = (usbBuffer.indexOf("HOME") >= 0);
    bool isSetId = (usbBuffer.indexOf("SETID") >= 0);
    
    // Görsel geri bildirim (LED Yan)
    digitalWrite(LED_PIN, LOW); 
    
    // 1. Gönderim Moduna Geç
    rs485_tx_mode();
    
    // 2. Veriyi Bas
    Serial485.print(usbBuffer);
    Serial485.flush(); // Tüm verinin gönderildiğinden emin ol
    
    // 3. Dinleme Moduna Dön (Slave'in yanıt vermesi için bekle)
    // Flash yazma işlemi zaman alabilir, bu yüzden daha uzun bekle
    if (isSetId) {
      delay(200); // SETID için Flash yazma süresi
    } else {
      delay(50); // Diğer komutlar için kısa bekleme
    }
    rs485_rx_mode();
    
    // Slave'in yanıtını almak için bekle
    if (isPing) {
      delay(4500); // PING için 4 saniye + buffer (slave'in 4 saniye beklemesi var)
    } else if (isHome) {
      delay(5000); // HOME için 5 saniye (homing işlemi zaman alabilir)
    } else {
      delay(200); // Diğer komutlar için (MOV, ALL vb.)
    }
    
    // 4. Temizlik
    usbBuffer = "";
    stringComplete = false;
    
    // LED Sön
    digitalWrite(LED_PIN, HIGH); 
  }

  // --- B. SLAVE'LERDEN GELENİ DİNLE (RS485 -> USB) ---
  // Karakter karakter oku ve direkt PC'ye aktar
  if (Serial485.available()) {
    // LED Yan (Veri Geliyor)
    digitalWrite(LED_PIN, LOW);
    
    // Tüm mevcut veriyi oku ve direkt PC'ye aktar
    while (Serial485.available()) {
      char c = Serial485.read();
      Serial.write(c);
    }
    
    // LED Sön
    digitalWrite(LED_PIN, HIGH);
  }
}