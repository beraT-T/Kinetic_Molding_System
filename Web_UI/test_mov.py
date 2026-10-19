"""
Elle seri test araci: PING + MOV komutlari gonderir.

!!! FIZIKSEL DONANIM ARACI - otomatik test DEGILDIR !!!
Seri porta gercek komut yazar (MOV:01:01:300, MOV:03:05:250 -> MOTOR HAREKET EDER). Ice aktarilinca (pytest / unittest toplama)
HICBIR SEY yapmaz; yalniz acikca calistirilinca ve --donanim bayragi verilince calisir:
    python test_mov.py --donanim
Donanimsiz testler: UI_v2.0_QML/tests (bkz. UI_v2.0_QML/README.md).
"""
import sys
import serial
import time

PORT = 'COM3'
BAUD = 115200



def main():
    try:
        print(f"🔌 {PORT} portuna bağlanılıyor...")
        ser = serial.Serial(PORT, BAUD, timeout=2)
        time.sleep(2)
    
        print("✅ Bağlantı başarılı!")
    
        # MASTER_READY mesajını oku
        if ser.in_waiting > 0:
            ready_msg = ser.readline().decode('utf-8').strip()
            print(f"📥 Master: {ready_msg}")
    
        # Test komutları
        test_commands = [
            "PING:01",
            "MOV:01:01:300",
            "MOV:03:05:250"  # Slave 3, Motor 5
        ]
    
        for cmd in test_commands:
            print(f"\n📤 Gönderiliyor: {cmd}")
            ser.write(f"{cmd}\n".encode())
            time.sleep(0.5)
        
            # Cevapları oku
            while ser.in_waiting > 0:
                response = ser.readline().decode('utf-8', errors='ignore').strip()
                if response:
                    print(f"📥 Cevap: {response}")
    
        print("\n⏳ 2 saniye daha bekleniyor (geç cevaplar için)...")
        time.sleep(4)
    
        while ser.in_waiting > 0:
            response = ser.readline().decode('utf-8', errors='ignore').strip()
            if response:
                print(f"📥 Geç cevap: {response}")
    
        ser.close()
        print("\n✅ Test tamamlandı!")
    
    except Exception as e:
        print(f"❌ Hata: {e}")


if __name__ == "__main__":
    if "--donanim" not in sys.argv:
        print("Bu arac GERCEK donanima seri komut gonderir. Calistirmak icin: "
              "python test_mov.py --donanim")
        sys.exit(2)
    main()
