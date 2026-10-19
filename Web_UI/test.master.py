"""
Elle seri test araci: PING:01 gonderip PONG bekler.

!!! FIZIKSEL DONANIM ARACI - otomatik test DEGILDIR !!!
Seri porta gercek komut yazar (PING:01). Ice aktarilinca (pytest / unittest toplama)
HICBIR SEY yapmaz; yalniz acikca calistirilinca ve --donanim bayragi verilince calisir:
    python test.master.py --donanim
Donanimsiz testler: UI_v2.0_QML/tests (bkz. UI_v2.0_QML/README.md).
"""
import sys
import serial
import time

# COM portunu değiştir (Device Manager'dan gelen)
PORT = 'COM3'  # ← BU NUMARAYI DEĞİŞTİR
BAUD = 115200



def main():
    try:
        print(f"🔌 {PORT} portuna bağlanılıyor...")
        ser = serial.Serial(PORT, BAUD, timeout=2)
        time.sleep(2)  # Bağlantı stabilize olsun
    
        print("✅ Bağlantı başarılı!")
        print("📤 PING:01 gönderiliyor...")
    
        # PING gönder
        ser.write(b"PING:01\n")
        time.sleep(0.5)
    
        # Cevap bekle
        if ser.in_waiting > 0:
            response = ser.readline().decode('utf-8').strip()
            print(f"📥 Cevap: {response}")
        
            if "PONG" in response:
                print("🎉 BAŞARILI! Master cevap veriyor!")
            else:
                print("⚠️ Beklenmeyen cevap!")
        else:
            print("❌ Cevap yok! Sorunlar:")
            print("   1. Firmware yüklü mü?")
            print("   2. Baud rate doğru mu? (115200)")
            print("   3. USB kablo veri destekliyor mu?")
    
        ser.close()
    
    except serial.SerialException as e:
        print(f"❌ HATA: {e}")
        print("\nÇözümler:")
        print("  1. COM port numarasını kontrol et (Device Manager)")
        print("  2. Başka program portu kullanıyor mu? (Arduino IDE, PuTTY)")
        print("  3. USB sürücüsü kurulu mu?")
    except Exception as e:
        print(f"❌ Beklenmeyen hata: {e}")


if __name__ == "__main__":
    if "--donanim" not in sys.argv:
        print("Bu arac GERCEK donanima seri komut gonderir. Calistirmak icin: "
              "python test.master.py --donanim")
        sys.exit(2)
    main()
