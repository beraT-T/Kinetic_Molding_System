import React, { useState, useEffect } from 'react';
import { Activity, Wifi, WifiOff, RotateCcw, Search, Network } from 'lucide-react';
import { api } from '../utils/api';

export default function Header({ isConnected, onConnect, onDisconnect, onReset, selectedPort, onPortChange, isDemoMode, onDemoModeChange, addLog, onScanNetwork, isScanning, activeSlaves }) {
    const [ports, setPorts] = useState([]);
    const [isScanningPorts, setIsScanningPorts] = useState(false);

    useEffect(() => {
        if (!isDemoMode) {
            scanPorts();
        }
    }, [isDemoMode]);

    const scanPorts = async () => {
        setIsScanningPorts(true);
        try {
            await new Promise(resolve => setTimeout(resolve, 600));
            const data = await api.getPorts();
            setPorts(data.ports || []);

            if (addLog) {
                addLog(`🔍 Port taraması bitti: ${data.ports?.length || 0} cihaz bulundu.`);
            }
        } catch (error) {
            console.error('Port tarama hatası:', error);
            if (addLog) addLog('❌ Portlar taranamadı.');
        }
        setIsScanningPorts(false);
    };

    return (
        <div className="bg-gradient-to-b from-[#1a2942] to-[#0f172a] border-b border-blue-900/20">
            <div className="max-w-[1600px] mx-auto px-6 py-5">
                <div className="flex items-center justify-between">
                    {/* Sol Taraf - Logo + Başlık */}
                    <div className="flex items-center gap-4">
                        {/* ✅ YENİ LOGO - Grid + Motor Hareketi */}
                        <div className="w-12 h-12 rounded-xl bg-gradient-to-br from-blue-500 to-indigo-600 flex items-center justify-center shadow-lg shadow-blue-900/30 relative overflow-hidden">
                            {/* Arka plan grid pattern */}
                            <div className="absolute inset-0 opacity-20">
                                <svg viewBox="0 0 48 48" className="w-full h-full">
                                    <defs>
                                        <pattern id="gridPattern" width="8" height="8" patternUnits="userSpaceOnUse">
                                            <path d="M 8 0 L 0 0 0 8" fill="none" stroke="white" strokeWidth="0.5" />
                                        </pattern>
                                    </defs>
                                    <rect width="48" height="48" fill="url(#gridPattern)" />
                                </svg>
                            </div>

                            {/* Aktüatör hareketi ikonu (yukarı-aşağı oklar) */}
                            <div className="relative z-10">
                                <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="white" strokeWidth="2.5" strokeLinecap="round" strokeLinejoin="round">
                                    <path d="M12 5v14M8 9l4-4 4 4M8 15l4 4 4-4" />
                                </svg>
                            </div>
                        </div>

                        <div>
                            <h1 className="text-2xl font-bold text-white drop-shadow-lg">
                                Adaptif Kalıp Kontrol
                            </h1>
                            <div className="flex items-center gap-3 mt-1">
                                <p className="text-sm text-slate-300 font-medium">
                                    144 Aktüatör Grid Sistem
                                </p>
                                <div className="h-4 w-[1px] bg-slate-600"></div>
                                <button
                                    onClick={() => onDemoModeChange(!isDemoMode)}
                                    className={`text-xs px-2.5 py-1 rounded-full border font-semibold transition-all ${isDemoMode
                                        ? 'bg-amber-500/20 text-amber-300 border-amber-400/40 backdrop-blur-sm'
                                        : 'bg-blue-500/20 text-blue-300 border-blue-400/40 backdrop-blur-sm hover:bg-blue-500/30'
                                        }`}
                                >
                                    {isDemoMode ? '🎭 Demo Mode' : '🔌 Hardware Mode'}
                                </button>
                            </div>
                        </div>
                    </div>

                    {/* Sağ Taraf - Butonlar */}
                    <div className="flex items-center gap-3">
                        {/* Port Seçimi (Sadece Hardware modda) */}
                        {!isDemoMode && (
                            <div className="flex items-center gap-2">
                                <select
                                    value={selectedPort}
                                    onChange={(e) => onPortChange(e.target.value)}
                                    disabled={isConnected}
                                    className="px-4 py-2.5 rounded-lg bg-slate-800/50 border border-slate-700/50 text-white disabled:opacity-50 backdrop-blur-sm text-sm font-medium min-w-[150px] focus:outline-none focus:ring-2 focus:ring-blue-500/50"
                                >
                                    <option value="" className="bg-slate-900">Port Seç ({ports.length})</option>
                                    {ports.map(p => (
                                        <option key={p.device} value={p.device} className="bg-slate-900">
                                            {p.device} - {p.description}
                                        </option>
                                    ))}
                                </select>
                                <button
                                    onClick={scanPorts}
                                    disabled={isScanningPorts || isConnected}
                                    className="px-4 py-2.5 rounded-lg bg-slate-800/50 hover:bg-slate-700/50 border border-slate-700/50 backdrop-blur-sm transition-all disabled:opacity-50"
                                    title="Portları Tara"
                                >
                                    <Search size={18} className={`text-slate-300 ${isScanningPorts ? 'animate-pulse' : ''}`} />
                                </button>
                            </div>
                        )}

                        {/* Ağı Tara Butonu */}
                        <button
                            onClick={onScanNetwork}
                            disabled={!isConnected || isScanning}
                            className={`flex items-center gap-2 px-4 py-2.5 rounded-lg font-semibold transition-all text-sm backdrop-blur-sm ${!isConnected || isScanning
                                ? 'bg-slate-800/30 text-slate-500 cursor-not-allowed border border-slate-700/30'
                                : 'bg-slate-700/70 hover:bg-slate-600/70 text-white border border-slate-600/50'
                                }`}
                            title="Slave'leri Tara"
                        >
                            <Network size={18} className={isScanning ? 'animate-pulse' : ''} />
                            {activeSlaves?.length > 0 && (
                                <span className="text-xs font-bold bg-emerald-600 px-2 py-0.5 rounded-full">
                                    {activeSlaves.length}/16
                                </span>
                            )}
                            <span className="hidden sm:inline">Ağı Tara</span>
                        </button>

                        {/* Bağlan/Kes Butonu */}
                        <button
                            onClick={isConnected ? onDisconnect : onConnect}
                            disabled={!isConnected && (!isDemoMode && !selectedPort)}
                            className={`flex items-center gap-2 px-6 py-2.5 rounded-lg font-bold transition-all backdrop-blur-sm ${isConnected
                                ? 'bg-red-600/90 hover:bg-red-600 text-white border border-red-500/30 shadow-lg shadow-red-900/20'
                                : !isDemoMode && !selectedPort
                                    ? 'bg-slate-800/30 text-slate-500 cursor-not-allowed border border-slate-700/30'
                                    : 'bg-blue-600/90 hover:bg-blue-600 text-white border border-blue-500/30 shadow-lg shadow-blue-900/30'
                                }`}
                        >
                            {isConnected ? <Wifi size={20} /> : <WifiOff size={20} />}
                            {isConnected ? 'Bağlantıyı Kes' : 'Bağlan'}
                        </button>

                        {/* Reset Butonu */}
                        <button
                            onClick={onReset}
                            className="px-4 py-2.5 rounded-lg font-semibold bg-slate-800/50 hover:bg-slate-700/50 text-slate-300 border border-slate-700/50 backdrop-blur-sm transition-all"
                            title="Sistemi Sıfırla"
                        >
                            <RotateCcw size={20} />
                        </button>
                    </div>
                </div>
            </div>
        </div>
    );
}