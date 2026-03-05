"""
STL'den 12x12 Motor Grid'e İnterpolasyon
Kaynak: aktuator_pozisyon.m

STL dosyasını 144 motor pozisyonuna dönüştürür (adaptif kalıp sistemi için).
"""

import numpy as np
from scipy.interpolate import LinearNDInterpolator
import trimesh


def interpolate_stl_to_grid(stl_file):
    """
    STL dosyasını yükle ve 12x12 motor grid'ine interpolasyon yap.
    
    Parametreler:
    ------------
    stl_file : str
        STL dosya yolu
    
    Döndürür:
    ---------
    np.ndarray
        144 elemanlı Z pozisyon dizisi (mm), aralık: 0-600mm
        Motorlar satır-satır indekslenir (motor 1-12: satır 1, motor 13-24: satır 2, vb.)
    """
    
    # 1. STL mesh'ini yükle
    print(f"STL yükleniyor: {stl_file}")
    mesh = trimesh.load(stl_file)
    points = mesh.vertices
    
    print(f"  Vertex sayısı: {len(points)}")
    print(f"  Üçgen sayısı: {len(mesh.faces)}")
    
    # 2. STL sınırlarını al (kendi sınırlarını kullan, zorla ölçekleme yok)
    minX, minY, minZ = points.min(axis=0)
    maxX, maxY, maxZ = points.max(axis=0)
    
    print(f"  X sınırları: [{minX:.1f}, {maxX:.1f}] mm")
    print(f"  Y sınırları: [{minY:.1f}, {maxY:.1f}] mm")
    print(f"  Z sınırları: [{minZ:.1f}, {maxZ:.1f}] mm")
    
    # 3. Interpolant oluştur (MATLAB scatteredInterpolant'a eşdeğer)
    # LinearNDInterpolator kullanılıyor ('natural' metoduna benzer)
    F = LinearNDInterpolator(points[:, :2], points[:, 2], fill_value=maxZ)
    
    # 4. 12x12 motor grid'i oluştur
    rows, cols = 12, 12
    x_spacing = (maxX - minX) / (cols - 1)
    y_spacing = (maxY - minY) / (rows - 1)
    
    print(f"  Motor aralığı: X={x_spacing:.2f}mm, Y={y_spacing:.2f}mm")
    
    # Grid noktaları (MATLAB meshgrid ile aynı)
    X, Y = np.meshgrid(
        np.linspace(minX, maxX, cols),
        np.linspace(minY, maxY, rows)
    )
    
    # 5. Her motor pozisyonu için Z değerini interpolasyon yap
    X_flat = X.ravel()
    Y_flat = Y.ravel()
    Z_top = F(X_flat, Y_flat)
    
    # NaN değerleri kontrol et (bazı noktalar interpolasyon aralığı dışında kalabilir)
    nan_count = np.isnan(Z_top).sum()
    if nan_count > 0:
        print(f"  Uyarı: {nan_count} NaN değer bulundu, max Z ile dolduruluyor")
        Z_top[np.isnan(Z_top)] = maxZ
    
    # 6. 300mm offset uygula (merkez 300mm'de olsun)
    Z_mean = np.mean(Z_top)
    offset = 300 - Z_mean
    Z_motor = Z_top + offset
    
    print(f"  Panel Z ortalaması: {Z_mean:.1f}mm → Offset: {offset:.1f}mm")
    
    # 7. Motor aralığına sınırla (0-600mm)
    Z_motor = np.clip(Z_motor, 0, 600)
    
    # 8. İstatistikler
    print(f"\n  Motor Pozisyonları:")
    print(f"    Min: {Z_motor.min():.1f}mm")
    print(f"    Max: {Z_motor.max():.1f}mm")
    print(f"    Ortalama: {Z_motor.mean():.1f}mm")
    print(f"    Std Sapma: {Z_motor.std():.1f}mm")
    
    return Z_motor.astype(int)


def grid_to_motor_commands(grid_values):
    """
    144 elemanlı grid'i MOV komutlarına dönüştür.
    
    Parametreler:
    ------------
    grid_values : np.ndarray
        144 Z pozisyonu
    
    Döndürür:
    ---------
    list of str
        MOV komutları, format: "MOV:SlaveID:MotorID:Pozisyon"
    """
    commands = []
    
    for idx, z_pos in enumerate(grid_values):
        slave_id = (idx // 9) + 1   # 1-16
        motor_id = (idx % 9) + 1    # 1-9
        cmd = f"MOV:{slave_id:02d}:{motor_id:02d}:{int(z_pos)}"
        commands.append(cmd)
    
    return commands


# Test fonksiyonu
if __name__ == "__main__":
    import sys
    
    if len(sys.argv) < 2:
        print("Kullanım: python stl_interpolator.py <stl_dosyası>")
        sys.exit(1)
    
    stl_file = sys.argv[1]
    
    # İnterpolasyon yap
    positions = interpolate_stl_to_grid(stl_file)
    
    # Komutları oluştur
    commands = grid_to_motor_commands(positions)
    
    # İlk 10 komutu göster
    print("\nİlk 10 MOV komutu:")
    for cmd in commands[:10]:
        print(f"  {cmd}")
    
    print(f"\n... (toplam {len(commands)} komut)")
    
    # Dosyaya kaydet
    output_file = "motor_commands.txt"
    with open(output_file, 'w') as f:
        f.write('\n'.join(commands))
    
    print(f"\nTüm komutlar kaydedildi: {output_file}")
