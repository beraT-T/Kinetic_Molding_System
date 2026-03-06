import React, { useState, useEffect } from 'react';
import io from 'socket.io-client';
import Header from './components/Header';
import FileUpload from './components/FileUpload';
import STLViewer3D from './components/STLViewer3D';
import HeatmapPanel from './components/HeatmapPanel';
import Terminal from './components/Terminal';
import { api } from './utils/api';
import TesterPanel from './components/TesterPanel';

export default function App() {
    const [activeTab, setActiveTab] = useState('main');
    const [stlFile, setStlFile] = useState(null);
    const [gridData, setGridData] = useState(null);
    const [stats, setStats] = useState(null);
    const [logs, setLogs] = useState([]);
    const [isConnected, setIsConnected] = useState(false);
    const [isProcessing, setIsProcessing] = useState(false);
    const [isSending, setIsSending] = useState(false);
    const [isSendingSlave3, setIsSendingSlave3] = useState(false);
    const [isHoming, setIsHoming] = useState(false);
    const [isHomingAll, setIsHomingAll] = useState(false);
    const [hoverInfo, setHoverInfo] = useState(null);
    const [viewerKey, setViewerKey] = useState(0);
    const [selectedPort, setSelectedPort] = useState('');
    const [isDemoMode, setIsDemoMode] = useState(true);
    const [isScanning, setIsScanning] = useState(false);
    const [activeSlaves, setActiveSlaves] = useState([]);
    const [currentSlaveId, setCurrentSlaveId] = useState(1);

    useEffect(() => {
        const fetchConfig = async () => {
            try {
                const data = await api.getConfig();
                setIsDemoMode(data.demo_mode);
            } catch (error) {
                console.error('Konfigürasyon alınamadı:', error);
            }
        };
        fetchConfig();
    }, []);

    useEffect(() => {
        const socket = io('http://localhost:5000', {
            transports: ['websocket', 'polling'],
            reconnectionAttempts: 10,
            reconnectionDelay: 2000,
            timeout: 20000
        });

        socket.on('connect', () => {
            console.log('✅ WebSocket bağlandı');
            addLog('✅ WebSocket bağlandı');
        });

        socket.on('disconnect', () => {
            console.log('❌ WebSocket bağlantısı kesildi');
            addLog('❌ WebSocket bağlantısı kesildi');
        });

        socket.on('serial_data', (data) => {
            console.log('📩 Mesaj geldi:', data);
            const timestamp = new Date(data.timestamp * 1000).toLocaleTimeString();
            setLogs(prev => [...prev.slice(-1000), { time: timestamp, text: data.message }]);
        });

        return () => socket.disconnect();
    }, []);

    const addLog = (message) => {
        const timestamp = new Date().toLocaleTimeString();
        setLogs(prev => [...prev.slice(-1000), { time: timestamp, text: message }]);
    };

    const handleConnect = async () => {
        try {
            addLog('🔌 Bağlanılıyor...');
            const data = await api.connect(isDemoMode ? 'DEMO' : (selectedPort || 'COM3'));
            if (data.success) {
                setIsConnected(true);
                addLog(`✅ ${isDemoMode ? 'Demo' : 'Master'} bağlantısı kuruldu`);
            } else {
                addLog(`❌ Bağlantı hatası: ${data.error}`);
            }
        } catch (error) {
            addLog('❌ Backend çalışmıyor! python app.py');
        }
    };

    const handleDisconnect = async () => {
        try {
            addLog('🔌 Bağlantı kesiliyorum...');
            const data = await api.disconnect();
            if (data.success) {
                setIsConnected(false);
                addLog('✅ Bağlantı kesildi');
            } else {
                addLog(`❌ Bağlantı kesme hatası: ${data.error}`);
            }
        } catch (error) {
            addLog('❌ Backend bağlantısı yok');
        }
    };

    const handleDemoModeToggle = async (val) => {
        try {
            const data = await api.updateConfig({ demo_mode: val });
            if (data.success) {
                setIsDemoMode(val);
                setIsConnected(false);
                addLog(`🔄 Mod değiştirildi: ${val ? 'Demo Mode' : 'Gerçek Donanım'}`);
            }
        } catch (error) {
            addLog('❌ Mod değiştirilemedi');
        }
    };

    const handleFileChange = (file) => {
        if (!file) return;
        setStlFile(file);
        setViewerKey(prev => prev + 1);
        addLog(`✅ ${file.name} (${(file.size / 1024 / 1024).toFixed(2)} MB)`);
    };

    const handleDrop = (e) => {
        e.preventDefault();
        const file = e.dataTransfer.files[0];
        if (file && file.name.endsWith('.stl')) {
            handleFileChange(file);
        }
    };

    const handleCalculate = async () => {
        if (!stlFile) {
            addLog('❌ STL dosyası seçin');
            return;
        }

        setIsProcessing(true);
        addLog('⚙️ İnterpolasyon başlatılıyor...');

        try {
            const data = await api.uploadSTL(stlFile);

            if (data.success) {
                setGridData(data.positions);
                setStats(data.stats);
                addLog('✅ 12x12 grid hesaplandı');
                addLog(`📊 ${data.stats.min}mm - ${data.stats.max}mm`);
            } else {
                addLog(`❌ ${data.error}`);
            }
        } catch (error) {
            addLog('❌ Backend bağlantısı yok');
        } finally {
            setIsProcessing(false);
        }
    };

    const handleScanNetwork = async () => {
        if (!isConnected) {
            addLog('❌ Önce Master\'a bağlanın');
            return;
        }

        setIsScanning(true);
        addLog('🔍 Ağ taraması başlatılıyor...');

        try {
            const data = await api.scanSlaves();
            if (data.success) {
                setActiveSlaves(data.active_slaves || []);
                addLog(`✅ ${data.active_slaves?.length || 0} aktif Slave bulundu: [${data.active_slaves?.join(', ') || ''}]`);
            } else {
                addLog(`❌ Tarama hatası: ${data.error}`);
            }
        } catch (error) {
            addLog('❌ Backend bağlantısı yok');
            console.error(error);
        } finally {
            setIsScanning(false);
        }
    };

    const handleSendToSlave = async () => {
        if (!gridData) {
            addLog('❌ Önce STL hesaplaması yapın');
            return;
        }
        if (!isConnected) {
            addLog('❌ Master bağlantısı yok');
            return;
        }

        setIsSendingSlave3(true);
        addLog(`🎯 Slave ${currentSlaveId}'e 9 MOV komutu gönderiliyor...`);

        try {
            const data = await api.sendToSlave(currentSlaveId, gridData);

            if (data.success) {
                addLog(`✅ Slave ${currentSlaveId}: ${data.sent_count} komut gönderildi.`);
            } else {
                addLog(`❌ ${data.error}`);
            }
        } catch (error) {
            addLog('❌ Gönderim başarısız');
            console.error(error);
        } finally {
            setIsSendingSlave3(false);
        }
    };

    const handleHomeSlave = async () => {
        if (!isConnected) {
            addLog('❌ Master bağlantısı yok');
            return;
        }

        setIsHoming(true);
        addLog(`🏠 Slave ${currentSlaveId} tüm motorlar HOME'a gönderiliyor...`);

        try {
            const data = await api.homeAll(currentSlaveId);

            if (data.success) {
                addLog(`✅ Slave ${currentSlaveId}: ${data.sent_count} HOME komutu gönderildi.`);
            } else {
                addLog(`❌ ${data.error}`);
            }
        } catch (error) {
            addLog('❌ Home başarısız');
            console.error(error);
        } finally {
            setIsHoming(false);
        }
    };

    const handleHomeAllSlaves = async () => {
        if (!isConnected) {
            addLog('❌ Master bağlantısı yok');
            return;
        }
        if (activeSlaves.length === 0) {
            addLog('❌ Aktif slave yok. Önce ağ taraması yapın.');
            return;
        }

        setIsHomingAll(true);
        addLog(`🏠 HOME ALL: ${activeSlaves.length} aktif slave HOME'a gönderiliyor...`);

        try {
            const data = await api.homeActiveSlaves();

            if (data.success) {
                addLog(`✅ HOME ALL: ${data.sent_count} komut gönderildi.`);
            } else {
                addLog(`❌ ${data.error}`);
            }
        } catch (error) {
            addLog('❌ HOME ALL başarısız');
            console.error(error);
        } finally {
            setIsHomingAll(false);
        }
    };

    const handleSendToMaster = async () => {
        if (!gridData) {
            addLog('❌ Önce hesaplama yapın');
            return;
        }
        if (!isConnected) {
            addLog('❌ Master bağlantısı yok');
            return;
        }

        setIsSending(true);
        addLog('📤 144 komut gönderiliyor...');

        try {
            const data = await api.sendToMaster(gridData);
            if (data.success) {
                addLog('✅ Komutlar gönderildi');
                addLog('⏳ Motorlar hareket ediyor...');
            } else {
                addLog(`❌ ${data.error}`);
            }
        } catch (error) {
            addLog('❌ Gönderim başarısız');
        } finally {
            setIsSending(false);
        }
    };

    const handleReset = () => {
        setStlFile(null);
        setGridData(null);
        setStats(null);
        setLogs([]);
        setViewerKey(prev => prev + 1);
        addLog('🔄 Sistem sıfırlandı');
    };

    return (
        // App.jsx - Ana container
        <div className="min-h-screen bg-gradient-to-br from-[#0a1628] via-[#0c1e3a] to-[#0a1628] text-white">
            <div className="fixed inset-0 overflow-hidden pointer-events-none">
                <div className="absolute top-0 left-1/4 w-[500px] h-[500px] bg-blue-600/15 rounded-full blur-3xl"></div>
                <div className="absolute bottom-0 right-1/4 w-[500px] h-[500px] bg-cyan-600/15 rounded-full blur-3xl"></div>
            </div>

            {/* Header - Az Padding */}
            <div className="relative z-10">
                <Header
                    isConnected={isConnected}
                    onConnect={handleConnect}
                    onReset={handleReset}
                    onDisconnect={handleDisconnect}
                    onScanNetwork={handleScanNetwork}
                    selectedPort={selectedPort}
                    onPortChange={setSelectedPort}
                    isDemoMode={isDemoMode}
                    onDemoModeChange={handleDemoModeToggle}
                    isScanning={isScanning}
                    activeSlaves={activeSlaves}
                    addLog={addLog}
                />
            </div>

            {/* Tab Navigation - Kenardan Az Mesafe */}
            <div className="max-w-[1600px] mx-auto px-6 mt-4">
                <div className="flex p-1 bg-white/5 backdrop-blur-md rounded-xl border border-white/10 w-fit">
                    <button
                        onClick={() => setActiveTab('main')}
                        className={`px-6 py-2 rounded-lg text-sm font-bold transition-all ${activeTab === 'main'
                            ? 'bg-blue-600 text-white shadow-lg'
                            : 'text-gray-400 hover:text-white'
                            }`}
                    >
                        STL Kontrol
                    </button>
                    <button
                        onClick={() => setActiveTab('tester')}
                        className={`px-6 py-2 rounded-lg text-sm font-bold transition-all ${activeTab === 'tester'
                            ? 'bg-purple-600 text-white shadow-lg'
                            : 'text-gray-400 hover:text-white'
                            }`}
                    >
                        Tester UI
                    </button>
                </div>
            </div>

            {/* Main Content - Kenarlardan Az Mesafe */}
            <div className="max-w-[1600px] mx-auto px-6 py-6">
                {activeTab === 'main' ? (
                    <div className="grid grid-cols-12 gap-6">
                        {/* Sol Panel - 5/12 (Genişletildi) */}
                        <div className="col-span-5 space-y-6">
                            {/* File Upload */}
                            <FileUpload
                                file={stlFile}
                                onFileChange={handleFileChange}
                                onCalculate={handleCalculate}
                                isProcessing={isProcessing}
                                onDrop={handleDrop}
                            />

                            {/* 3D Preview - Daha Büyük */}
                            <div className="bg-white/5 backdrop-blur-xl rounded-2xl p-6 border border-white/10">
                                <h3 className="text-sm font-bold mb-4 flex items-center gap-2 text-gray-300">
                                    <div className="w-2 h-2 rounded-full bg-blue-400 animate-pulse"></div>
                                    3D Önizleme
                                </h3>
                                <div className="h-[450px] rounded-xl overflow-hidden bg-[#111827]/80 border border-white/10 relative">
                                    <STLViewer3D key={viewerKey} file={stlFile} />
                                    {!stlFile && (
                                        <div className="absolute inset-0 flex items-center justify-center pointer-events-none">
                                            <div className="text-center">
                                                <p className="text-[10px] uppercase tracking-[0.3em] text-gray-400 font-black opacity-40">
                                                    MODEL SEÇİLMEDİ
                                                </p>
                                            </div>
                                        </div>
                                    )}
                                </div>
                                <div className="flex justify-center gap-6 mt-4 text-[10px] text-gray-500 font-bold tracking-wider uppercase">
                                    <span>🔄 Orbit</span>
                                    <span>🔍 Zoom</span>
                                    <span>👆 Pan</span>
                                </div>
                            </div>

                            {/* Terminal */}
                            <Terminal logs={logs} />
                        </div>

                        {/* Sağ Panel - 7/12 (Aynı Kaldı) */}
                        <div className="col-span-7">
                            <HeatmapPanel
                                stats={stats}
                                gridData={gridData}
                                hoverInfo={hoverInfo}
                                onHover={setHoverInfo}
                                isConnected={isConnected}
                                isSending={isSending}
                                onSendToMaster={handleSendToMaster}
                                onSendToSlave={handleSendToSlave}
                                onScanNetwork={handleScanNetwork}
                                isScanning={isScanning}
                                onHomeSlave={handleHomeSlave}
                                isHoming={isHoming}
                                isSendingSlave={isSendingSlave3}
                                activeSlaves={activeSlaves}
                                currentSlaveId={currentSlaveId}
                                onSlaveChange={setCurrentSlaveId}
                                onHomeAllSlaves={handleHomeAllSlaves}
                                isHomingAll={isHomingAll}
                            />
                        </div>
                    </div>
                ) : (
                    <TesterPanel
                        isConnected={isConnected}
                        activeSlaves={activeSlaves}
                        logs={logs}
                        addLog={addLog}
                    />
                )}
            </div>

            {/* Footer */}
            <div className="relative z-10 text-center py-6 text-gray-500 text-sm">
                <p>Adaptif Kalıp Kontrol Sistemi</p>
            </div>
        </div>
    );
}