#!/usr/bin/env python3
"""
STM32F407 Slave Motor Test Script
Tüm motorları test etmek için kullanılır
"""

import serial
import serial.tools.list_ports
import time
import sys

# --- AYARLAR ---
BAUD_RATE = 115200
SLAVE_ID = 1  # Test edilecek slave ID

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

def send_command(ser, command):
    """Komut gönder ve yanıtı bekle"""
    ser.write(f"{command}\n".encode('utf-8'))
    ser.flush()
    time.sleep(0.1)  # Yanıt için bekle
    
    # Yanıtı oku
    if ser.in_waiting > 0:
        response = ser.readline().decode('utf-8').strip()
        return response
    return None

def test_ping(ser):
    """PING komutunu test et"""
    print("\n=== PING Testi ===")
    print(f"PING:{SLAVE_ID:02d} gönderiliyor...")
    response = send_command(ser, f"PING:{SLAVE_ID:02d}")
    if response:
        print(f"Yanıt: {response}")
        if response.startswith("PONG"):
            print("✓ PING başarılı!")
            return True
        else:
            print("✗ PING başarısız!")
            return False
    else:
        print("✗ Yanıt alınamadı!")
        return False

def test_setid(ser, new_id):
    """SETID komutunu test et"""
    print(f"\n=== SETID Testi (ID: {new_id}) ===")
    print(f"SETID:{new_id:02d} gönderiliyor...")
    response = send_command(ser, f"SETID:{new_id:02d}")
    if response:
        print(f"Yanıt: {response}")
        if response.startswith("IDSET"):
            print("✓ SETID başarılı!")
            return True
        else:
            print("✗ SETID başarısız!")
            return False
    else:
        print("✗ Yanıt alınamadı!")
        return False

def test_motor(ser, motor_id, position):
    """Tek bir motoru test et"""
    print(f"\n=== Motor {motor_id} Testi (Pozisyon: {position}mm) ===")
    print(f"MOV:{SLAVE_ID:02d}:{motor_id:02d}:{position} gönderiliyor...")
    response = send_command(ser, f"MOV:{SLAVE_ID:02d}:{motor_id:02d}:{position}")
    if response:
        print(f"Yanıt: {response}")
        if response.startswith("MOVOK"):
            print(f"✓ Motor {motor_id} başarılı!")
            return True
        else:
            print(f"✗ Motor {motor_id} başarısız!")
            return False
    else:
        print(f"✗ Motor {motor_id} yanıt alınamadı!")
        return False

def test_all_motors(ser, position):
    """Tüm motorları aynı pozisyona sür"""
    print(f"\n=== Tüm Motorlar Testi (Pozisyon: {position}mm) ===")
    print(f"ALL:{SLAVE_ID:02d}:{position} gönderiliyor...")
    response = send_command(ser, f"ALL:{SLAVE_ID:02d}:{position}")
    if response:
        print(f"Yanıt: {response}")
        if response.startswith("ALLOK"):
            print("✓ Tüm motorlar başarılı!")
            return True
        else:
            print("✗ Tüm motorlar başarısız!")
            return False
    else:
        print("✗ Yanıt alınamadı!")
        return False

def test_encoder_read(ser):
    """Encoder okuma testi (şimdilik sadece bilgi)"""
    print("\n=== Encoder Okuma Testi ===")
    print("NOT: Encoder değerleri slave tarafından otomatik okunur.")
    print("Motor hareket ederken encoder değerleri güncellenir.")
    print("Encoder kalibrasyonu yapıldıktan sonra pozisyon kontrolü aktif olacak.")

def main():
    print("=" * 60)
    print("STM32F407 Slave Motor Test Script")
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
    
    try:
        # Test menüsü
        while True:
            print("\n" + "=" * 60)
            print("TEST MENÜSÜ")
            print("=" * 60)
            print("1. PING Testi")
            print("2. SETID Testi")
            print("3. Tek Motor Testi (1-9)")
            print("4. Tüm Motorlar Testi")
            print("5. Sıralı Motor Testi (1'den 9'a)")
            print("6. Encoder Okuma Bilgisi")
            print("0. Çıkış")
            
            choice = input("\nSeçiminiz (0-6): ").strip()
            
            if choice == "0":
                print("\nÇıkılıyor...")
                break
            elif choice == "1":
                test_ping(ser)
            elif choice == "2":
                try:
                    new_id = int(input("Yeni Slave ID (1-16): "))
                    if 1 <= new_id <= 16:
                        test_setid(ser, new_id)
                    else:
                        print("Geçersiz ID! (1-16 arası olmalı)")
                except ValueError:
                    print("Geçersiz giriş!")
            elif choice == "3":
                try:
                    motor_id = int(input("Motor ID (1-9): "))
                    position = int(input("Pozisyon (0-600mm): "))
                    if 1 <= motor_id <= 9 and 0 <= position <= 600:
                        test_motor(ser, motor_id, position)
                    else:
                        print("Geçersiz parametreler!")
                except ValueError:
                    print("Geçersiz giriş!")
            elif choice == "4":
                try:
                    position = int(input("Pozisyon (0-600mm): "))
                    if 0 <= position <= 600:
                        test_all_motors(ser, position)
                    else:
                        print("Geçersiz pozisyon! (0-600mm)")
                except ValueError:
                    print("Geçersiz giriş!")
            elif choice == "5":
                print("\n=== Sıralı Motor Testi ===")
                print("Her motor 100mm pozisyona gidecek, 1 saniye bekleyecek...")
                for motor_id in range(1, 10):
                    test_motor(ser, motor_id, 100)
                    time.sleep(1)
                print("\n✓ Tüm motorlar test edildi!")
            elif choice == "6":
                test_encoder_read(ser)
            else:
                print("Geçersiz seçim!")
            
            time.sleep(0.5)  # Komutlar arası bekleme
    
    except KeyboardInterrupt:
        print("\n\nKullanıcı tarafından durduruldu...")
    except Exception as e:
        print(f"\n✗ Hata: {e}")
    finally:
        ser.close()
        print("\nSerial bağlantısı kapatıldı.")

if __name__ == "__main__":
    main()

