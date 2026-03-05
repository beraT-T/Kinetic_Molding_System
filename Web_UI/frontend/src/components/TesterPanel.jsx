import React, { useState, useEffect, useRef } from 'react';
import { api } from '../utils/api';

const TesterPanel = ({ isConnected, activeSlaves, logs, addLog }) => {
    const [selectedSlave, setSelectedSlave] = useState(1);
    const [motorPositions, setMotorPositions] = useState(Array(9).fill(0));
    const [command, setCommand] = useState('');
    const terminalRef = useRef(null);

    // Terminal logları değiştiğinde sadece iç alanı kaydır (Sayfa zıplamasını önler)
    useEffect(() => {
        if (terminalRef.current) {
            terminalRef.current.scrollTop = terminalRef.current.scrollHeight;
        }
    }, [logs]);

    const handleSlaveSelect = (id) => {
        setSelectedSlave(id);
        addLog(`📍 Slave ID ${id} seçildi.`);
    };

    const sliderTimers = useRef({});

    const handleMotorMove = (motorIdx, value) => {
        const newPositions = [...motorPositions];
        newPositions[motorIdx] = value;
        setMotorPositions(newPositions);

        // Debounce: 200ms bekle (ui_v1.0.py'da 300ms idi, biraz daha hızlı olsun)
        if (sliderTimers.current[motorIdx]) {
            clearTimeout(sliderTimers.current[motorIdx]);
        }

        sliderTimers.current[motorIdx] = setTimeout(async () => {
            if (!isConnected) return;

            const cmd = `MOV:${selectedSlave.toString().padStart(2, '0')}:${(motorIdx + 1).toString().padStart(2, '0')}:${parseInt(value)}`;
            try {
                await api.sendCommand(cmd);
                delete sliderTimers.current[motorIdx];
            } catch (error) {
                console.error('Komut gönderilemedi:', error);
            }
        }, 200);
    };

    const handleHomeMotor = async (motorIdx) => {
        if (!isConnected) return;

        // UI'da slider'ı da sıfıra çek
        const newPositions = [...motorPositions];
        newPositions[motorIdx] = 0;
        setMotorPositions(newPositions);

        // HOME:SlaveID:MotorID
        const cmd = `HOME:${selectedSlave.toString().padStart(2, '0')}:${(motorIdx + 1).toString().padStart(2, '0')}`;
        try {
            await api.sendCommand(cmd);
            addLog(`🏠 Motor ${motorIdx + 1} HOME komutu gönderildi.`);
        } catch (error) {
            console.error('Home komutu gönderilemedi:', error);
        }
    };

    const handleSendCommand = async (e) => {
        e.preventDefault();
        const cmdText = command.trim();

        if (!cmdText) return;

        // Tetiklendiğini anlamak için konsola bas
        console.log("Terminal komutu tetiklendi:", cmdText);

        if (!isConnected) {
            addLog(`⚠️ Bağlantı yok! Komut gönderilemedi: ${cmdText}`);
            return;
        }

        try {
            // Komutu önce yerel logda göster
            addLog(`→ ${cmdText}`);

            const data = await api.sendCommand(cmdText);
            if (data.success) {
                setCommand('');
            } else {
                addLog(`❌ Hata: ${data.error}`);
            }
        } catch (error) {
            addLog(`❌ Bağlantı hatası: ${error.message}`);
        }
    };

    const handleHomeAll = async () => {
        if (!isConnected) return;
        const cmd = `ALL:${selectedSlave.toString().padStart(2, '0')}:0`;
        try {
            await api.sendCommand(cmd);
            setMotorPositions(Array(9).fill(0));
            addLog(`🏠 Slave ${selectedSlave} tüm motorlar sıfırlanıyor...`);
        } catch (error) {
            console.error('Toplu home hatası:', error);
        }
    };

    const handleTestPos = async () => {
        if (!isConnected) return;
        const cmd = `ALL:${selectedSlave.toString().padStart(2, '0')}:300`;
        try {
            await api.sendCommand(cmd);
            setMotorPositions(Array(9).fill(300));
            addLog(`🎯 Slave ${selectedSlave} 300mm test pozisyonuna gidiyor...`);
        } catch (error) {
            console.error('Test pozisyonu hatası:', error);
        }
    };

    return (
        <div className="grid grid-cols-1 lg:grid-cols-12 gap-6 animate-in fade-in duration-500">
            {/* Dashboard (16 Slave) - Left Column (3 units) */}
            <div className="lg:col-span-4 space-y-4">
                <div className="bg-white/5 backdrop-blur-xl rounded-2xl p-4 border border-white/10">
                    <h3 className="text-lg font-bold mb-4 flex items-center gap-2">
                        <i className="fas fa-network-wired text-blue-400"></i>
                        Dashboard (16 Slaves)
                    </h3>
                    <div className="grid grid-cols-4 gap-2">
                        {Array.from({ length: 16 }, (_, i) => i + 1).map((id) => (
                            <button
                                key={id}
                                onClick={() => handleSlaveSelect(id)}
                                className={`h-16 rounded-lg font-bold text-xs flex flex-col items-center justify-center transition-all duration-200 border ${selectedSlave === id
                                    ? 'bg-blue-600/40 border-blue-400 border-2 shadow-[0_0_15px_rgba(59,130,246,0.3)]'
                                    : activeSlaves.includes(id)
                                        ? 'bg-green-600/20 border-green-500/50 hover:bg-green-600/30'
                                        : 'bg-white/5 border-white/10 hover:bg-white/10'
                                    }`}
                            >
                                <span>ID: {id}</span>
                                <span className={`text-[10px] mt-1 ${activeSlaves.includes(id) ? 'text-green-400' : 'text-gray-500'}`}>
                                    {activeSlaves.includes(id) ? 'Online' : 'Offline'}
                                </span>
                            </button>
                        ))}
                    </div>
                </div>

                {/* Info Panel */}
                <div className="bg-gradient-to-br from-blue-600/20 to-purple-600/20 backdrop-blur-xl rounded-2xl p-4 border border-white/10">
                    <h4 className="font-bold text-sm mb-2">Tester UI Klavuzu</h4>
                    <ul className="text-xs text-gray-400 space-y-1">
                        <li>• Sol panelden hedef modülü seçin.</li>
                        <li>• 3x3 grid üzerinden motorları kontrol edin.</li>
                        <li>• Terminalden direkt protokol komutları girebilirsiniz.</li>
                    </ul>
                </div>
            </div>

            {/* Motor Tester & Terminal - Right Column (9 units) */}
            <div className="lg:col-span-8 space-y-6">
                {/* 3x3 Motor Grid */}
                <div className="bg-white/5 backdrop-blur-xl rounded-2xl p-6 border border-white/10">
                    <div className="flex justify-between items-center mb-6">
                        <h3 className="text-xl font-bold flex items-center gap-2">
                            <i className="fas fa-microchip text-purple-400"></i>
                            Modül Kontrol: ID {selectedSlave}
                        </h3>
                        <div className="flex gap-2">
                            <button
                                onClick={handleHomeAll}
                                className="px-4 py-2 bg-orange-600/20 hover:bg-orange-600/40 border border-orange-500/50 rounded-lg text-sm font-bold transition-all"
                            >
                                Tümünü Sıfırla
                            </button>
                            <button
                                onClick={handleTestPos}
                                className="px-4 py-2 bg-blue-600/20 hover:bg-blue-600/40 border border-blue-500/50 rounded-lg text-sm font-bold transition-all"
                            >
                                Test (300mm)
                            </button>
                        </div>
                    </div>

                    <div className="grid grid-cols-3 gap-4">
                        {motorPositions.map((pos, idx) => (
                            <div key={idx} className="bg-white/5 rounded-xl p-4 border border-white/5 flex flex-col items-center gap-3">
                                <span className="text-xs font-bold text-gray-400">Motor {idx + 1}</span>
                                <div className="relative h-48 w-full flex justify-center py-2">
                                    <input
                                        type="range"
                                        min="0"
                                        max="600"
                                        value={pos}
                                        onChange={(e) => handleMotorMove(idx, e.target.value)}
                                        className="h-full appearance-none bg-white/10 rounded-lg overflow-hidden w-2 cursor-pointer
                                                   [&::-webkit-slider-thumb]:appearance-none [&::-webkit-slider-thumb]:h-4 [&::-webkit-slider-thumb]:w-4 
                                                   [&::-webkit-slider-thumb]:rounded-full [&::-webkit-slider-thumb]:bg-blue-500
                                                   [&::-webkit-slider-thumb]:shadow-[0_0_10px_rgba(59,130,246,0.8)]"
                                        style={{ writingMode: 'bt-lr', WebkitAppearance: 'slider-vertical' }}
                                    />
                                </div>
                                <div className="flex flex-col items-center gap-2 w-full">
                                    <span className="text-sm font-mono font-bold text-blue-400">{pos}mm</span>
                                    <button
                                        onClick={() => handleHomeMotor(idx)}
                                        className="w-full py-1 text-[10px] bg-orange-600/30 hover:bg-orange-600/50 border border-orange-500/30 rounded font-bold uppercase"
                                    >
                                        Home
                                    </button>
                                </div>
                            </div>
                        ))}
                    </div>
                </div>

                {/* Serial Terminal */}
                <div className="bg-[#050510] rounded-2xl border border-white/10 overflow-hidden flex flex-col h-[350px]">
                    <div className="bg-white/5 px-4 py-2 border-bottom border-white/10 flex items-center justify-between">
                        <span className="text-xs font-bold text-gray-400 flex items-center gap-2">
                            <div className="w-2 h-2 rounded-full bg-green-500 animate-pulse"></div>
                            SERIAL MONITOR / TERMINAL
                        </span>
                        <div className="flex gap-2 text-[10px]">
                            <span className="text-gray-600">Baud: 115200</span>
                        </div>
                    </div>

                    <div
                        ref={terminalRef}
                        className="flex-1 overflow-y-auto p-4 font-mono text-sm space-y-1 scrollbar-thin scrollbar-thumb-white/10"
                    >
                        {logs.map((log, i) => (
                            <div key={i} className="flex gap-3">
                                <span className="text-blue-500/50 shrink-0">[{log.time}]</span>
                                <span className={log.text.startsWith('→') ? 'text-blue-400' : log.text.startsWith('←') ? 'text-green-400' : 'text-gray-300'}>
                                    {log.text}
                                </span>
                            </div>
                        ))}
                    </div>

                    <form onSubmit={handleSendCommand} className="p-2 bg-white/5 flex gap-2">
                        <input
                            type="text"
                            value={command}
                            onChange={(e) => setCommand(e.target.value)}
                            placeholder="Komut girin (örn: PING:01, MOV:03:01:100)..."
                            className="flex-1 bg-black/40 border border-white/10 rounded-lg px-4 py-2 text-sm focus:outline-none focus:border-blue-500 transition-all font-mono"
                        />
                        <button
                            type="submit"
                            className="bg-blue-600 hover:bg-blue-500 px-6 rounded-lg text-sm font-bold transition-all"
                        >
                            Gönder
                        </button>
                    </form>
                </div>
            </div>
        </div>
    );
};

export default TesterPanel;
