#!/usr/bin/env python3
"""
STM32F103C8 Encoder Test Script
Encoder pulse'larını test etmek için kullanılır
"""

import serial
import serial.tools.list_ports
import time
import sys
import re

# --- AYARLAR ---
BAUD_RATE = 115200

def get_com_port():
    """COM port listesini göster ve seç"""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("Hata: Hiçbir COM port bulunamadı!")
        return None
    
    print("\nMevcut COM Portlar:")
    for i, port in enumerate(ports):
        print(f"  {i+1}. {port.device} - {port.description}")
    
    if len(ports) == 1:
        print(f"\nOtomatik seçildi: {ports[0].device}")
        return ports[0].device
    
    try:
        choice = int(input("\nPort numarasını seçin (1-{}): ".format(len(ports))))
        if 1 <= choice <= len(ports):
            return ports[choice - 1].device
        else:
            print("Geçersiz seçim!")
            return None
    except ValueError:
        print("Geçersiz giriş!")
        return None

def parse_encoder_data(line):
    """Encoder verisini parse et"""
    # Format: "Encoder: 1234 pulse | Hız: 100.50 pulse/s | Yön: FORWARD | Motor: FORWARD"
    pattern = r'Encoder: (-?\d+) pulse \| Hız: ([\d.]+) pulse/s \| Yön: (\w+) \| Motor: (\w+)'
    match = re.match(pattern, line)
    if match:
        counter = int(match.group(1))
        speed = float(match.group(2))
        direction = match.group(3)
        motor_state = match.group(4)
        return {
            'counter': counter,
            'speed': speed,
            'direction': direction,
            'motor_state': motor_state
        }
    return None

def main():
    print("=" * 60)
    print("STM32F103C8 Encoder Test Script")
    print("=" * 60)
    
    # COM port seç
    port = get_com_port()
    if not port:
        print("COM port seçilemedi, çıkılıyor...")
        sys.exit(1)
    
    # Serial bağlantısı
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        time.sleep(2)  # Bağlantı için bekle
        print(f"\n✓ {port} portuna bağlandı @ {BAUD_RATE} baud")
    except Exception as e:
        print(f"✗ Bağlantı hatası: {e}")
        sys.exit(1)
    
    # Buffer'ı temizle
    ser.reset_input_buffer()
    
    print("\nEncoder verileri okunuyor... (Ctrl+C ile çıkış)")
    print("Komut göndermek için: F (Forward), R (Reverse), S (Stop), T (Test), ? (Info), H (Help)\n")
    
    encoder_data_history = []
    last_counter = 0
    
    try:
        while True:
            # Gelen veriyi oku
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                # Encoder verisi mi?
                if "Encoder:" in line:
                    data = parse_encoder_data(line)
                    if data:
                        encoder_data_history.append(data)
                        
                        # Son 10 veriyi göster
                        if len(encoder_data_history) > 10:
                            encoder_data_history.pop(0)
                        
                        # Counter değişimini hesapla
                        counter_diff = data['counter'] - last_counter
                        last_counter = data['counter']
                        
                        # Konsola yazdır
                        print(f"Counter: {data['counter']:8d} | "
                              f"Hız: {data['speed']:7.2f} pulse/s | "
                              f"Yön: {data['direction']:8s} | "
                              f"Motor: {data['motor_state']:8s} | "
                              f"Δ: {counter_diff:+6d}")
                else:
                    # Diğer mesajları direkt göster
                    print(line)
            
            # Kullanıcı komutu göndermek için threading kullan
            # (Windows'ta select çalışmaz)
            time.sleep(0.01)  # CPU kullanımını azalt
    
    except KeyboardInterrupt:
        print("\n\nKullanıcı tarafından durduruldu...")
        print("\nEncoder Test Özeti:")
        if encoder_data_history:
            print(f"  Toplam Okuma: {len(encoder_data_history)}")
            print(f"  Son Counter: {encoder_data_history[-1]['counter']}")
            print(f"  Ortalama Hız: {sum(d['speed'] for d in encoder_data_history) / len(encoder_data_history):.2f} pulse/s")
    except Exception as e:
        print(f"\n✗ Hata: {e}")
    finally:
        ser.close()
        print("\nSerial bağlantısı kapatıldı.")

if __name__ == "__main__":
    print("NOT: Komut göndermek için Serial Monitor kullanın veya")
    print("     scripti çalıştırırken komutları manuel olarak gönderin.\n")
    
    main()

