import React from 'react';
import { Play, RotateCw, Activity, Zap, Target } from 'lucide-react';
import Heatmap from './Heatmap';

export default function HeatmapPanel({
    stats,
    gridData,
    hoverInfo,
    onHover,
    isConnected,
    isSending,
    onSendToMaster,
    onSendToSlave,
    onScanNetwork,
    isScanning,
    onHomeSlave,
    isHoming,
    isSendingSlave,
    activeSlaves,
    currentSlaveId,
    onSlaveChange,
    onSendAllForce,
    isSendingForce
}) {
    return (
        <div className="bg-[#0f172a]/40 backdrop-blur-2xl rounded-2xl p-6 border border-blue-900/20 shadow-2xl">
            {/* Stats Header */}
            <div className="flex items-center justify-between mb-6 bg-[#1a2942]/30 p-4 rounded-xl border border-blue-800/20">
                <div className="flex items-center gap-8">
                    <div className="flex items-center gap-3">
                        <Activity size={14} className="text-slate-400" />
                        <div>
                            <span className="text-[9px] uppercase tracking-wider text-slate-500 font-semibold block">Min</span>
                            <span className="text-base font-mono font-bold text-slate-200">
                                {stats ? stats.min.toFixed(1) : '0.0'}
                            </span>
                        </div>
                    </div>
                    <div className="flex items-center gap-3">
                        <Zap size={14} className="text-slate-400" />
                        <div>
                            <span className="text-[9px] uppercase tracking-wider text-slate-500 font-semibold block">Ort</span>
                            <span className="text-base font-mono font-bold text-slate-200">
                                {stats ? stats.mean.toFixed(1) : '0.0'}
                            </span>
                        </div>
                    </div>
                    <div className="flex items-center gap-3">
                        <Target size={14} className="text-slate-400" />
                        <div>
                            <span className="text-[9px] uppercase tracking-wider text-slate-500 font-semibold block">Max</span>
                            <span className="text-base font-mono font-bold text-slate-200">
                                {stats ? stats.max.toFixed(1) : '0.0'}
                            </span>
                        </div>
                    </div>
                </div>

                <div className="flex items-center gap-4">
                    <div className="flex items-center gap-2">
                        <div className={`w-2 h-2 rounded-full ${isConnected ? 'bg-emerald-500' : 'bg-slate-600'}`}></div>
                        <span className="text-xs font-semibold uppercase tracking-tight text-slate-400">
                            {isConnected ? 'Bağlı' : 'Bağlı Değil'}
                        </span>
                    </div>
                </div>
            </div>

            {/* View Area */}
            <div className="rounded-xl p-4 border border-blue-900/20 mb-6">
                <div className="h-[500px]">
                    <Heatmap gridData={gridData} onHover={onHover} />
                </div>

                {hoverInfo && (
                    <div className="mt-4 text-center">
                        <span className="text-xs font-semibold text-slate-300 bg-blue-900/30 px-4 py-2 rounded-lg border border-blue-800/30">
                            Hücre [{hoverInfo.row}, {hoverInfo.col}]: {hoverInfo.val.toFixed(2)} mm
                        </span>
                    </div>
                )}
            </div>

            {/* Controls */}
            <div className="grid grid-cols-5 gap-3">
                {/* Slave Selector */}
                <div className="flex items-center gap-2 px-3 py-2.5 bg-[#1a2942]/30 rounded-lg border border-blue-800/20">
                    <span className="text-[10px] text-slate-500 font-semibold whitespace-nowrap">Modül:</span>
                    <select
                        value={currentSlaveId}
                        onChange={(e) => onSlaveChange(parseInt(e.target.value))}
                        className="w-full bg-transparent text-sm text-slate-200 focus:outline-none cursor-pointer font-semibold"
                    >
                        {Array.from({ length: 16 }, (_, i) => i + 1).map(id => (
                            <option key={id} value={id} className="bg-[#0f172a]">Slave {id}</option>
                        ))}
                    </select>
                </div>

                {/* Home */}
                <button
                    onClick={onHomeSlave}
                    disabled={!isConnected || isHoming}
                    className={`h-11 rounded-lg text-xs font-bold transition-all ${!isConnected || isHoming
                            ? 'bg-slate-800/50 text-slate-600 cursor-not-allowed border border-slate-700/50'
                            : 'bg-orange-600/90 hover:bg-orange-600 text-white border border-orange-500/30 shadow-lg shadow-orange-900/20'
                        }`}
                >
                    {isHoming ? <RotateCw className="animate-spin mx-auto" size={16} /> : `HOME ${currentSlaveId}`}
                </button>

                {/* Send Single */}
                <button
                    onClick={onSendToSlave}
                    disabled={!gridData || !isConnected || isSendingSlave}
                    className={`h-11 rounded-lg text-xs font-bold transition-all ${!gridData || !isConnected || isSendingSlave
                            ? 'bg-slate-800/50 text-slate-600 cursor-not-allowed border border-slate-700/50'
                            : 'bg-slate-700 hover:bg-slate-600 text-white border border-slate-600/50'
                        }`}
                >
                    {isSendingSlave ? <RotateCw className="animate-spin mx-auto" size={16} /> : `SEND ${currentSlaveId}`}
                </button>

                {/* Send All (sadece aktif slave'ler) */}
                <button
                    onClick={onSendToMaster}
                    disabled={!gridData || !isConnected || isSending}
                    className={`h-11 rounded-lg text-xs font-bold transition-all ${!gridData || !isConnected || isSending
                            ? 'bg-slate-800/50 text-slate-600 cursor-not-allowed border border-slate-700/50'
                            : 'bg-emerald-600/90 hover:bg-emerald-600 text-white border border-emerald-500/30 shadow-lg shadow-emerald-900/20'
                        }`}
                >
                    {isSending ? (
                        <RotateCw className="animate-spin mx-auto" size={16} />
                    ) : (
                        <div className="flex items-center justify-center gap-2">
                            <Play size={12} fill="currentColor" />
                            <span>SEND ALL</span>
                        </div>
                    )}
                </button>

                {/* Force All (ping kontrolü yok, 16 slave) */}
                <button
                    onClick={onSendAllForce}
                    disabled={!gridData || !isConnected || isSendingForce}
                    className={`h-11 rounded-lg text-xs font-bold transition-all ${!gridData || !isConnected || isSendingForce
                            ? 'bg-slate-800/50 text-slate-600 cursor-not-allowed border border-slate-700/50'
                            : 'bg-violet-600/90 hover:bg-violet-600 text-white border border-violet-500/30 shadow-lg shadow-violet-900/20'
                        }`}
                >
                    {isSendingForce ? (
                        <RotateCw className="animate-spin mx-auto" size={16} />
                    ) : (
                        <div className="flex items-center justify-center gap-2">
                            <Zap size={12} fill="currentColor" />
                            <span>FORCE 16</span>
                        </div>
                    )}
                </button>
            </div>
        </div>
    );
}