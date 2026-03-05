from flask import Flask, request, jsonify, send_from_directory
from flask_cors import CORS
from flask_socketio import SocketIO, emit
import serial
import serial.tools.list_ports
import threading
import time
import os
from stl_interpolator import interpolate_stl_to_grid

app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*", ping_timeout=120, ping_interval=20)

# 🎭 DEMO MODE - donanım olmadan test için
DEMO_MODE = False  # False yapınca gerçek donanım gerekir

# Uploads klasörü
UPLOAD_FOLDER = 'uploads'
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

# ✅ GLOBAL DEĞİŞKENLER
ser = None
serial_buffer = []
is_connected = False
active_slaves = []  # ← YENİ

# ✅ SERIAL VERİ İŞLEME FONKSİYONU
def process_serial_data(line):
    """Gelen seri veriyi filtrele ve işle (WebSocket/Buffer yönetimi burada)"""
    global active_slaves, serial_buffer
    
    # Zararlı/Gereksiz mesajları filtrele (Kullanıcı istemiyor)
    filter_tags = ["MOVOK", "DEBUG:"]
    if any(tag in line for tag in filter_tags):
        return

    # Konsola bas (Sadece filtrelenmemiş/anlamlı veriler)
    print(f"[DEBUG Serial] ← {line}")

    # Önemli yanıtlar için özel formatlama
    msg = line
    if line.startswith("PONG"):
        try:
            parts = line.split(":")
            slave_id = int(parts[1])
            if slave_id not in active_slaves:
                active_slaves.append(slave_id)
            msg = f"✅ Slave {slave_id} AKTİF (PONG)"
        except: pass
    elif line.startswith("IDSET"):
        msg = f"✅ ID Atandı: {line}"
    elif any(tag in line for tag in ["HOMEDONE", "EDOHOME", "HHOMED"]):
        msg = f"🏠 {line}"

    # Log Kaydı ve WebSocket Gönderimi (TEK NOKTADAN)
    log_entry = {
        'timestamp': time.time(),
        'message': f"← {msg}"
    }
    serial_buffer.append(log_entry)
    
    # Buffer koruması (Gereksiz şişmeyi önle)
    if len(serial_buffer) > 500:
        serial_buffer = serial_buffer[-250:]
    
    socketio.emit('serial_data', log_entry)

# ✅ YENİ: Serial Okuma Thread'i (process_serial_data ile entegre)
def read_serial_loop():
    """Arka planda sürekli serial port'u dinle - Flask context ile"""
    global ser, is_connected, serial_buffer
    buffer = ""
    
    print("[DEBUG] read_serial_loop başlatıldı")
    
    with app.app_context():
        while True:
            if is_connected and ser and ser.is_open:
                try:
                    if ser.in_waiting > 0:
                        # Tüm veriyi ham olarak oku
                        raw_data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                        buffer += raw_data
                        
                        # Hem \r\n hem \n desteği için normalize et
                        if '\n' in buffer:
                            lines = buffer.split('\n')
                            # Son parça yarım kalmış olabilir, buffer'da tut
                            buffer = lines[-1]
                            
                            # Diğer tam satırları işle
                            for line in lines[:-1]:
                                clean_line = line.strip().replace('\r', '')
                                if clean_line:
                                    # Veriyi işle ve logla (Merkezi yönetim + Filtreleme)
                                    process_serial_data(clean_line)
                
                except Exception as e:
                    print(f"[DEBUG Serial Hatası] {e}")
                    buffer = ""
            else:
                buffer = ""
                time.sleep(0.5) # Bağlantı yoksa daha seyrek kontrol et
            
            time.sleep(0.01) # CPU'yu yorma

# ✅ SONRA (doğru - Flask context ile)
def start_serial_thread():
    """Serial okuma thread'ini Flask context ile başlat"""
    serial_thread = threading.Thread(target=read_serial_loop, daemon=True)
    serial_thread.start()

# API Endpoints
@app.route('/api/ports', methods=['GET'])
def get_ports():
    """Mevcut COM portlarını listele"""
    ports = serial.tools.list_ports.comports()
    port_list = [{'device': p.device, 'description': p.description} for p in ports]
    print(f"🔍 Port tarama isteği: {len(port_list)} port bulundu.")
    return jsonify({
        'ports': port_list,
        'count': len(port_list)
    })

@app.route('/api/config', methods=['GET', 'POST'])
def handle_config():
    """Sistem konfigürasyonunu yönet (Demo Mode vb.)"""
    global DEMO_MODE
    if request.method == 'POST':
        data = request.json
        if 'demo_mode' in data:
            DEMO_MODE = data['demo_mode']
            status = "AÇIK" if DEMO_MODE else "KAPALI"
            print(f"🔄 Demo Mode {status} olarak değiştirildi.")
            socketio.emit('serial_data', {
                'timestamp': time.time(), 
                'message': f'🔄 Sistem Durumu: {"Demo Mode Aktif" if DEMO_MODE else "Gerçek Donanım Modu"}'
            })
        return jsonify({'success': True, 'demo_mode': DEMO_MODE})
    
    return jsonify({
        'demo_mode': DEMO_MODE,
        'is_connected': is_connected
    })

@app.route('/api/connect', methods=['POST'])
def connect_serial():
    """Serial porta bağlan (veya DEMO MODE'da sahte bağlantı)"""
    global ser, is_connected
    try:
        if DEMO_MODE:
            # 🎭 DEMO MODE: Sahte bağlantı
            is_connected = True
            print("\n🎭 [DEMO MODE] Sahte Master bağlantısı kuruldu!")
            socketio.emit('serial_data', {'timestamp': time.time(), 'message': '🎭 DEMO MODE: Master simüle ediliyor'})
            socketio.emit('serial_data', {'timestamp': time.time(), 'message': '✅ Bağlantı başarılı (sahte)'})
            return jsonify({'success': True, 'message': 'Demo modunda bağlandı'})
        
        # Gerçek bağlantı
        data = request.json
        port = data['port']
        baudrate = data.get('baudrate', 115200)
        
        print(f"🔌 Gerçek bağlantı deneniyor: {port} @ {baudrate} baud")
        
        if ser and ser.is_open:
            ser.close()
        
        try:
            ser = serial.Serial(port, baudrate, timeout=1)
            print(f"✅ {port} AÇILDI - {baudrate} baud")
        except Exception as e:
            print(f"❌ PORT HATASI: {e}")
            return jsonify({'success': False, 'error': str(e)}), 400
        
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        is_connected = True
        
        # Okuma thread'ini başlat
        threading.Thread(target= read_serial_loop, daemon=True).start()
        
        return jsonify({'success': True, 'message': f'Bağlandı: {port}'})
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 400
    
    

@app.route('/api/disconnect', methods=['POST'])
def disconnect_serial():
    """Serial bağlantısını kes"""
    global ser
    try:
        if ser and ser.is_open:
            ser.close()
        ser = None
        return jsonify({'success': True, 'message': 'Bağlantı kesildi'})
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 400



@app.route('/api/upload-stl', methods=['POST'])
def upload_stl():
    """STL dosyası yükle ve interpolasyon yap"""
    try:
        if 'file' not in request.files:
            return jsonify({'success': False, 'error': 'Dosya yok'}), 400
        
        file = request.files['file']
        if file.filename == '':
            return jsonify({'success': False, 'error': 'Dosya seçilmedi'}), 400
        
        # Dosyayı kaydet
        filepath = os.path.join(UPLOAD_FOLDER, file.filename)
        file.save(filepath)
        
        # İnterpolasyon yap
        positions = interpolate_stl_to_grid(filepath)
        
        return jsonify({
            'success': True,
            'filename': file.filename,
            'positions': positions.tolist(),
            'stats': {
                'min': int(positions.min()),
                'max': int(positions.max()),
                'mean': int(positions.mean()),
                'std': int(positions.std())
            }
        })
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 400
    
        
@app.route('/api/scan', methods=['POST'])
def scan_slaves():
    """16 Slave'e PING at (UI_v1.0 mantığıyla - hızlı gönder, arka planda dinle)"""
    global ser, is_connected, serial_buffer, active_slaves, DEMO_MODE
    try:
        if (not ser or not ser.is_open) and not DEMO_MODE:
            return jsonify({'success': False, 'error': 'Bağlantı yok'}), 400
        
        print("\n" + "="*60)
        print(f"🔍 AĞ TARAMASI BAŞLADI {'(DEMO MODE)' if DEMO_MODE else ''}")
        print("="*60)
        
        socketio.emit('serial_data', {
            'timestamp': time.time(),
            'message': f"🔍 Ağ Taraması Başladı... {'(DEMO MODE)' if DEMO_MODE else ''}"
        })

        if DEMO_MODE:
            # DEMO MODU: Tüm 16 slave aktif gibi simüle et
            active_slaves = [] # Reset
            simulated_slaves = list(range(1, 17))
            for s_id in simulated_slaves:
                time.sleep(0.5) # Simüle gecikme
                if s_id not in active_slaves:
                    active_slaves.append(s_id)
                
                log_msg = {
                    'timestamp': time.time(),
                    'message': f"✅ Slave {s_id} AKTİF (PONG - DEMO)"
                }
                serial_buffer.append(log_msg)
                socketio.emit('serial_data', log_msg)
                print(f"   🎭 DEMO: Slave {s_id} aktif edildi.")
            
            return jsonify({
                'success': True,
                'active_slaves': active_slaves,
                'count': len(active_slaves),
                'message': f'Demo tarama tamamlandı ({len(active_slaves)} slave bulundu)'
            })
        
        # TÜM PING'LERİ HIZLICA GÖNDER
        for slave_id in range(1, 17):
            cmd = f"PING:{slave_id:02d}\n"
            
            try:
                ser.write(cmd.encode())
                
                log_entry = {
                    'timestamp': time.time(),
                    'message': f'→ PING:{slave_id:02d}'
                }
                serial_buffer.append(log_entry)
                socketio.emit('serial_data', log_entry)
                
                print(f"   → PING:{slave_id:02d}")
                
                time.sleep(0.4) 
                
            except Exception as e:
                print(f"   ❌ Slave {slave_id:02d} hatası: {e}")
        
        print("="*60)
        print("✅ Tüm PING'ler gönderildi")
        print(f"   Aktif Slave'ler: {active_slaves}")  # ← DEBUG
        print("   PONG'lar arka planda alınacak (read_serial_loop)")
        print("="*60 + "\n")
        
        socketio.emit('serial_data', {
            'timestamp': time.time(),
            'message': '✅ PING\'ler gönderildi, PONG\'lar bekleniyor...'
        })
        
        # ✅ ACTIVE_SLAVES DÖNDÜR
        return jsonify({
            'success': True,
            'active_slaves': active_slaves,  # ← ÇOK ÖNEMLİ! EKLE
            'count': len(active_slaves),     # ← Bonus: Kaç slave aktif
            'message': 'PING\'ler gönderildi',
            'info': 'PONG yanıtları WebSocket üzerinden gelecek'
        })
        
    except Exception as e:
        print(f"❌ TARAMA HATASI: {str(e)}")
        import traceback
        traceback.print_exc()
        return jsonify({'success': False, 'error': str(e)}), 400

        

@app.route('/api/send-to-slave', methods=['POST'])
def send_to_slave():
    """STL'den gelen 12×12 grid'in BELİRLİ BİR SLAVE bölümünü gönder - SENKRON HAREKET"""
    global ser, serial_buffer, is_connected

    try:
        if (not ser or not ser.is_open) and not DEMO_MODE:
            return jsonify({'success': False, 'error': 'Serial bağlantısı yok'}), 400
        
        data = request.json
        positions = data['positions']  # 144 elemanlı FLAT array
        slave_id = data.get('slave_id', 3) # Varsayılan 3, ama artık dinamik
        
        # 12×12 flat array'i 2D array'e çevir
        grid_2d = []
        for i in range(12):
            row = []
            for j in range(12):
                row.append(positions[i * 12 + j])
            grid_2d.append(row)
        
        # SLAVE KONUMUNU HESAPLA
        slave_row = (slave_id - 1) // 4
        slave_col = (slave_id - 1) % 4
        start_row = slave_row * 3
        start_col = slave_col * 3
        
        print("\n" + "="*80)
        print(f"🎯 SLAVE {slave_id}'E ÖZEL 9 MOV KOMUTU - SENKRON HAREKET")
        print("="*80)
        print(f"   Slave ID: {slave_id}")
        print(f"   Grid pozisyonu: Row [{start_row}-{start_row+2}], Col [{start_col}-{start_col+2}]")
        print("="*80 + "\n")
        
        socketio.emit('serial_data', {
            'timestamp': time.time(),
            'message': f'🎯 Slave {slave_id}\'e 9 MOV komutu gönderiliyor (SENKRON)...'
        })
        
        # ============================================================
        # 4. POZISYONLARI HAZIRLA
        # ============================================================
        motor_positions = []
        motor_id = 1
        
        for i in range(3):
            for j in range(3):
                row = start_row + i
                col = start_col + j
                z_pos = grid_2d[row][col]
                
                motor_positions.append({
                    'motor_id': motor_id,
                    'grid_row': row,
                    'grid_col': col,
                    'z_pos': int(z_pos),
                    'command': f"MOV:{slave_id:02d}:{motor_id:02d}:{int(z_pos)}",
                    'movok_received': False,
                    'response_time': None
                })
                
                motor_id += 1
        
        # ============================================================
        # 5. TÜM KOMUTLARI HIZLICA GÖNDER (SENKRON)
        # ============================================================
        print("📤 9 MOV komutunu hızlıca gönderiyorum (SENKRON hareket)...\n")
        
        start_time = time.time()
        sent_count = 0
        
        for mp in motor_positions:
            try:
                full_cmd = f"{mp['command']}\r\n"
                if ser and ser.is_open:
                    ser.write(full_cmd.encode('utf-8'))
                    ser.flush()
                
                sent_count += 1
                
                print(f"   [{mp['motor_id']}/9] → {mp['command']} (Raw: {full_cmd.encode()})")
                
                socketio.emit('serial_data', {
                    'timestamp': time.time(),
                    'message': f"[{mp['motor_id']}/9] → {mp['command']}"
                })
                
                # 🛠 ÖNEMLİ: Bekleme süresini 50ms'den 200ms'ye çıkardım (Donanım yanıt kapasitesi için)
                socketio.sleep(0.2) # Yield to WebSocket heartbeat
            
            except Exception as e:
                print(f"   ❌ Motor {mp['motor_id']} hatası: {e}")
        
        send_duration = (time.time() - start_time) * 1000
        print(f"\n✅ {sent_count}/9 komut gönderildi ({send_duration:.0f}ms)")
        print("⏳ Tüm motorlar AYNI ANDA hareket ediyor...\n")
        
        socketio.emit('serial_data', {
            'timestamp': time.time(),
            'message': f'✅ {sent_count}/9 komut gönderildi - Motorlar senkron hareket ediyor... Yanıtlar asenkron olarak loglara yansıyacak.'
        })
        
        # ============================================================
        # 6. RETURN (Okuma artık asenkron, arka plan thread'i hallediyor)
        # ============================================================
        return jsonify({
            'success': True,
            'slave_id': 3,
            'sent_count': sent_count,
            'message': f'Slave 3: {sent_count} komut gönderildi (SENKRON). Yanıtlar WebSocket üzerinden takip edilecek.',
            'info': 'Asenkron okuma aktif - Çakışma engellendi.'
        })
    
    # ============================================================
    # EXCEPT BLOĞU
    # ============================================================
    except Exception as e:
        print(f"\n❌ FATAL HATA: {str(e)}")
        import traceback
        traceback.print_exc()
        
        socketio.emit('serial_data', {
            'timestamp': time.time(),
            'message': f'❌ Hata: {str(e)}'
        })
        
        return jsonify({'success': False, 'error': str(e)}), 400

@app.route('/api/home-slave3', methods=['POST'])
def home_slave3():
    """Slave 3'ün tüm motorlarını 0 pozisyonuna gönder - SENKRON HAREKET"""
    global ser, is_connected
    
    try:
        if (not ser or not ser.is_open) and not DEMO_MODE:
            return jsonify({'success': False, 'error': 'Serial bağlantısı yok'}), 400
        
        print("\n" + "="*80)
        print("🏠 SLAVE 3 HOME - SENKRON HAREKET")
        print("="*80)
        
        socketio.emit('serial_data', {
            'timestamp': time.time(),
            'message': '🏠 Slave 3 motorları senkron sıfırlanıyor...'
        })
        
        sent_count = 0
        for motor_id in range(1, 10):
            cmd = f"MOV:03:{motor_id:02d}:0"
            try:
                if ser and ser.is_open:
                    ser.write(f"{cmd}\n".encode())
                sent_count += 1
                print(f"   [{motor_id}/9] → {cmd}")
                socketio.emit('serial_data', {
                    'timestamp': time.time(),
                    'message': f'[{motor_id}/9] → {cmd}'
                })
                time.sleep(0.02)  # 20ms - Buffer taşmasını önle
            except Exception as e:
                print(f"   ❌ Motor {motor_id} gönderim hatası: {e}")
        
        return jsonify({
            'success': True,
            'sent_count': sent_count,
            'message': f'{sent_count} motor home pozisyonuna gönderildi. Yanıtlar arka planda alınıyor.'
        })
    except Exception as e:
        print(f"\n❌ HOME HATASI: {str(e)}")
        return jsonify({'success': False, 'error': str(e)}), 400

    

    


@app.route('/api/send-to-master', methods=['POST'])
def send_to_master():
    """STL'den gelen pozisyonları Master'a gönder (UI_v1.0 mantığıyla)"""
    global ser, serial_buffer, is_connected
    try:
        if (not ser or not ser.is_open) and not DEMO_MODE:
            return jsonify({'success': False, 'error': 'Serial bağlantısı yok'}), 400
        
        data = request.json
        positions = data['positions']  # 144 elemanlı liste
        
        # 12×12 flat array'i 2D array'e çevir
        grid_2d = []
        for i in range(12):
            row = []
            for j in range(12):
                row.append(positions[i * 12 + j])
            grid_2d.append(row)
        
        print("\n" + "="*80)
        print(f"📤 MOV KOMUTLARI GÖNDERİLİYOR ({len(active_slaves) * 9} Komut)")
        print("="*80)
        
        if not active_slaves:
            print("⚠️ Aktif slave bulunamadı! Gönderim iptal edildi.")
            socketio.emit('serial_data', {
                'timestamp': time.time(),
                'message': '⚠️ Aktif slave bulunamadı! Lütfen önce tarama (Scan) yapın.'
            })
            return jsonify({'success': False, 'error': 'Aktif slave bulunamadı. Lütfen önce tarama yapın.'}), 400

        socketio.emit('serial_data', {
            'timestamp': time.time(),
            'message': f'📤 {len(active_slaves) * 9} MOV komutu {len(active_slaves)} slave için gönderiliyor...'
        })
        
        sent_count = 0
        
        # SADECE AKTİF SLAVE'LERE HIZLICA GÖNDER (tek tek MOVOK bekleme)
        for slave_id in active_slaves:
            # Bu Slave'in 3×3 grid koordinatları
            slave_row = (slave_id - 1) // 4
            slave_col = (slave_id - 1) % 4
            
            start_row = slave_row * 3
            start_col = slave_col * 3
            
            motor_id = 1
            for i in range(3):
                for j in range(3):
                    row = start_row + i
                    col = start_col + j
                    z_pos = grid_2d[row][col]
                    
                    cmd_str = f"MOV:{slave_id:02d}:{motor_id:02d}:{int(z_pos)}"
                    full_cmd = f"{cmd_str}\r\n"
                    
                    try:
                        # Komutu gönder
                        if ser and ser.is_open:
                            ser.write(full_cmd.encode('utf-8'))
                            ser.flush()
                        
                        # Log ekle
                        log_entry = {
                            'timestamp': time.time(),
                            'message': f"→ {cmd_str}"
                        }
                        serial_buffer.append(log_entry)
                        socketio.emit('serial_data', log_entry)
                        
                        sent_count += 1
                        
                        # İlk ve son 3 komutu göster
                        if sent_count <= 3 or sent_count > (len(active_slaves) * 9 - 3):
                            print(f"   [{sent_count:03d}/{len(active_slaves) * 9}] {cmd_str} (Raw: {full_cmd.encode()})")
                        elif sent_count == 4:
                            print(f"   ... ({len(active_slaves) * 9 - 6} komut daha) ...")
                        
                        # Spam önleme (Master buffer'ı için gecikme artırıldı)
                        socketio.sleep(0.1)  # Yield for heartbeat
                    except Exception as send_error:
                        print(f"   ❌ HATA: {cmd_str.strip()} - {send_error}")
                    
                    motor_id += 1
        
        print("="*80)
        print(f"✅ {sent_count}/{len(active_slaves) * 9} komut gönderildi")
        print("="*80 + "\n")
        
        socketio.emit('serial_data', {
            'timestamp': time.time(),
            'message': f'✅ {sent_count} komut gönderildi - Motorlar hareket ediyor...'
        })
        
        return jsonify({
            'success': True,
            'sent_count': sent_count,
            'message': f'{sent_count} komut gönderildi',
            'info': 'MOVOK yanıtları arka planda alınıyor'
        })
        
    except Exception as e:
        print(f"\n❌ FATAL HATA: {str(e)}")
        import traceback
        traceback.print_exc()
        return jsonify({'success': False, 'error': str(e)}), 400
    

    

@app.route('/api/send-command', methods=['POST'])
def send_command():
    """Tek komut gönder (manuel test için)"""
    global ser, DEMO_MODE
    try:
        if (not ser or not ser.is_open) and not DEMO_MODE:
            return jsonify({'success': False, 'error': 'Serial bağlantısı yok'}), 400
        
        cmd = request.json['command']
        
        # Gerçek gönderim
        if not DEMO_MODE and ser and ser.is_open:
            full_cmd = f"{cmd}\r\n"
            ser.write(full_cmd.encode('utf-8'))
            ser.flush()
            socketio.sleep(0.01)
            print(f"[DEBUG Terminal] Gönderildi: {full_cmd.strip()} (Raw: {full_cmd.encode()})")
        
        # WebSocket'e bas
        log_entry = {
            'timestamp': time.time(),
            'message': f"→ {cmd}" + (" (DEMO)" if DEMO_MODE else "")
        }
        serial_buffer.append(log_entry)
        socketio.emit('serial_data', log_entry)

        return jsonify({'success': True, 'demo': DEMO_MODE})
    except Exception as e:
        print(f"❌ send_command hatası: {e}")
        return jsonify({'success': False, 'error': str(e)}), 400

@app.route('/api/serial-log', methods=['GET'])
def get_serial_log():
    """Serial log'u al"""
    global serial_buffer
    return jsonify({'logs': serial_buffer[-200:]})  # Son 200 mesaj

@app.route('/api/test-ping', methods=['POST'])
def test_ping():
    """Master'a PING komutu gönder (Yanıt asenkron loglanacak)"""
    global ser, DEMO_MODE
    try:
        if (not ser or not ser.is_open) and not DEMO_MODE:
            return jsonify({'success': False, 'error': 'Serial port açık değil'}), 400
        
        cmd = "PING:01" # \n kaldırıldı, \r\n aşağıda eklenecek
        print(f"\n📤 TEST: {cmd.strip()} gönderiliyor... {'(DEMO)' if DEMO_MODE else ''}")
        # Gerçek gönderim
        if not DEMO_MODE and ser and ser.is_open:
            full_cmd = f"{cmd}\r\n"
            ser.write(full_cmd.encode('utf-8'))
            ser.flush()  # Tamponu hemen boşalt
            time.sleep(0.005)  # 5ms bekle (donanım işlemesi için)
            print(f"[DEBUG] Terminalden gönderildi: {full_cmd.strip()} (Raw: {full_cmd.encode()})")
        
        socketio.emit('serial_data', {
            'timestamp': time.time(),
            'message': f'→ TEST: {cmd.strip()} gönderildi.' + (' (DEMO)' if DEMO_MODE else '')
        })

        if DEMO_MODE:
             socketio.emit('serial_data', {
                'timestamp': time.time() + 0.5,
                'message': '← PONG:01 (DEMO)'
            })
        
        return jsonify({
            'success': True,
            'message': 'PING gönderildi.'
        })
        
    except Exception as e:
        print(f"❌ Test hatası: {e}")
        return jsonify({'success': False, 'error': str(e)}), 400

@app.route('/api/send-all-force', methods=['POST'])
def send_all_force():
    """Ping/Pong'dan bağımsız olarak TÜM 16 slave'e 144 MOV komutu gönder"""
    global ser, serial_buffer, is_connected
    try:
        if (not ser or not ser.is_open) and not DEMO_MODE:
            return jsonify({'success': False, 'error': 'Serial bağlantısı yok'}), 400

        data = request.json
        positions = data['positions']  # 144 elemanlı FLAT array

        # 12×12 flat array'i 2D array'e çevir
        grid_2d = []
        for i in range(12):
            row = []
            for j in range(12):
                row.append(positions[i * 12 + j])
            grid_2d.append(row)

        all_slaves = list(range(1, 17))  # 1..16 arası tüm slave'ler

        print("\n" + "="*80)
        print(f"🚀 FORCE SEND ALL - TÜM 16 SLAVE'E {16 * 9} KOMUT GÖNDERİLİYOR")
        print("="*80)

        socketio.emit('serial_data', {
            'timestamp': time.time(),
            'message': f'🚀 FORCE SEND ALL: {16 * 9} MOV komutu 16 slave için gönderiliyor (Ping kontrolü yok)...'
        })

        sent_count = 0

        for slave_id in all_slaves:
            slave_row = (slave_id - 1) // 4
            slave_col = (slave_id - 1) % 4
            start_row = slave_row * 3
            start_col = slave_col * 3

            motor_id = 1
            for i in range(3):
                for j in range(3):
                    row = start_row + i
                    col = start_col + j
                    z_pos = grid_2d[row][col]

                    cmd_str = f"MOV:{slave_id:02d}:{motor_id:02d}:{int(z_pos)}"
                    full_cmd = f"{cmd_str}\r\n"

                    try:
                        if ser and ser.is_open:
                            ser.write(full_cmd.encode('utf-8'))
                            ser.flush()

                        log_entry = {
                            'timestamp': time.time(),
                            'message': f"→ {cmd_str}"
                        }
                        serial_buffer.append(log_entry)
                        socketio.emit('serial_data', log_entry)

                        sent_count += 1

                        if sent_count <= 3 or sent_count > (16 * 9 - 3):
                            print(f"   [{sent_count:03d}/144] {cmd_str}")
                        elif sent_count == 4:
                            print(f"   ... (138 komut daha) ...")

                        socketio.sleep(0.05)
                    except Exception as send_error:
                        print(f"   ❌ HATA: {cmd_str} - {send_error}")

                    motor_id += 1

        print("="*80)
        print(f"✅ {sent_count}/144 komut gönderildi (FORCE ALL)")
        print("="*80 + "\n")

        socketio.emit('serial_data', {
            'timestamp': time.time(),
            'message': f'✅ FORCE ALL: {sent_count}/144 komut gönderildi - Tüm motorlar hareket ediyor...'
        })

        return jsonify({
            'success': True,
            'sent_count': sent_count,
            'message': f'{sent_count} komut gönderildi (tüm 16 slave, ping kontrolü yok)',
            'info': 'MOVOK yanıtları arka planda alınıyor'
        })

    except Exception as e:
        print(f"\n❌ FORCE SEND ALL HATASI: {str(e)}")
        import traceback
        traceback.print_exc()
        return jsonify({'success': False, 'error': str(e)}), 400


if __name__ == '__main__':
    print("Backend başlatılıyor: http://localhost:5000")
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)