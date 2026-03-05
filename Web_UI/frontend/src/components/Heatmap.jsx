import React from 'react';

const Heatmap = ({ gridData, onHover }) => {
    // gridData undefined veya boş ise bekleme ekranı göster
    if (!gridData || gridData.length === 0) {
        return (
            <div className="w-full h-full flex items-center justify-center bg-black/20 rounded-xl border border-white/5">
                <p className="text-gray-500 text-xs animate-pulse uppercase tracking-widest font-bold">
                    Veri Bekleniyor
                </p>
            </div>
        );
    }

    // ✅ 1D array (144) → 2D array (12x12) dönüşümü
    const grid2D = [];
    for (let i = 0; i < 12; i++) {
        const row = [];
        for (let j = 0; j < 12; j++) {
            row.push(gridData[i * 12 + j] || 0);
        }
        grid2D.push(row);
    }

    // ✅ Min/Max hesapla (dinamik renk için)
    const min = Math.min(...gridData);
    const max = Math.max(...gridData);

    // ✅ Yumuşak Geçişli Renk Skalası (Parlamasız)
    const getCellColor = (val) => {
        if (max === min) return 'hsl(220, 50%, 35%)';

        const normalized = (val - min) / (max - min); // 0-1 arası

        // Mavi → Cyan → Yeşil → Sarı → Turuncu → Kırmızı
        const hue = (1 - normalized) * 240; // 240 (mavi) → 0 (kırmızı)
        const saturation = 65; // Sabit doygunluk (mat)
        const lightness = 40; // Sabit parlaklık (koyu, mat)

        return `hsl(${hue}, ${saturation}%, ${lightness}%)`;
    };

    // ✅ Motor numarası hesapla (1-144)
    const getMotorNumber = (row, col) => {
        return row * 12 + col + 1;
    };

    return (
        <div className="w-full h-full flex items-center justify-center">
            <div className="w-full h-full max-w-full max-h-full p-2">
                <div className="grid grid-cols-12 gap-[2px] w-full h-full">
                    {grid2D.map((row, rowIndex) =>
                        row.map((val, colIndex) => {
                            const motorNum = getMotorNumber(rowIndex, colIndex);

                            return (
                                <div
                                    key={`${rowIndex}-${colIndex}`}
                                    className="relative group cursor-crosshair transition-all duration-200 rounded-sm overflow-hidden hover:scale-110 hover:z-20"
                                    onMouseEnter={() => onHover({ row: rowIndex, col: colIndex, val })}
                                    onMouseLeave={() => onHover(null)}
                                    style={{
                                        backgroundColor: getCellColor(val)
                                        // ✅ boxShadow kaldırıldı (parlama yok)
                                    }}
                                >
                                    {/* Motor Numarası + Değer */}
                                    <div className="absolute inset-0 flex flex-col items-center justify-center text-center pointer-events-none">
                                        <span className="text-[8px] font-bold text-white/90 drop-shadow-lg leading-none">
                                            M{motorNum}
                                        </span>
                                        <span className="text-[7px] font-bold text-white/70 drop-shadow-md mt-0.5">
                                            {Math.round(val)}
                                        </span>
                                    </div>

                                    {/* Hover Overlay */}
                                    <div className="absolute inset-0 opacity-0 group-hover:opacity-100 transition-opacity bg-black/80 pointer-events-none z-10 flex flex-col items-center justify-center">
                                        <span className="text-[9px] font-black text-cyan-400 drop-shadow-lg">
                                            Motor {motorNum}
                                        </span>
                                        <span className="text-[8px] font-bold text-white mt-1">
                                            {val.toFixed(2)} mm
                                        </span>
                                        <span className="text-[7px] text-gray-400 mt-0.5">
                                            [{rowIndex}, {colIndex}]
                                        </span>
                                    </div>

                                    {/* Hover Border */}
                                    <div className="absolute inset-0 border-2 border-cyan-400 opacity-0 group-hover:opacity-100 transition-opacity pointer-events-none rounded-sm"></div>
                                </div>
                            );
                        })
                    )}
                </div>
            </div>
        </div>
    );
};

export default Heatmap;