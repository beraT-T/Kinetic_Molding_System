import React, { useEffect, useRef } from 'react';

export default function Terminal({ logs }) {
    const scrollRef = useRef(null);

    useEffect(() => {
        if (scrollRef.current) {
            scrollRef.current.scrollTop = scrollRef.current.scrollHeight;
        }
    }, [logs]);

    return (
        <div className="bg-[#0f172a]/40 backdrop-blur-xl rounded-2xl border border-blue-900/20 shadow-inner flex flex-col h-[500px] overflow-hidden">
            {/* Header */}
            <div className="flex items-center gap-2 px-3 py-2 border-b border-blue-800/20 bg-[#1a2942]/30">
                <div className="flex gap-1.5">
                    <div className="w-2 h-2 rounded-full bg-red-500"></div>
                    <div className="w-2 h-2 rounded-full bg-yellow-500"></div>
                    <div className="w-2 h-2 rounded-full bg-green-500"></div>
                </div>
                <span className="text-[10px] font-bold text-slate-400 uppercase tracking-[0.2em] ml-2">
                    Sistem Konsolu
                </span>
            </div>

            {/* Log Area */}
            <div
                ref={scrollRef}
                className="flex-1 overflow-y-auto p-3 font-mono text-[10px] space-y-1.5 scrollbar-thin scrollbar-thumb-blue-800/30 scrollbar-track-transparent"
            >
                {logs.length === 0 ? (
                    <div className="text-slate-500 italic opacity-50">
                        Sistem hazır, olay bekleniyor...
                    </div>
                ) : (
                    logs.map((log, i) => (
                        <div key={i} className="flex gap-2 leading-relaxed group">
                            <span className="text-slate-600 shrink-0 font-bold">
                                [{log.time}]
                            </span>
                            <span className="text-slate-300 group-hover:text-blue-400 transition-colors break-all">
                                {log.text}
                            </span>
                        </div>
                    ))
                )}
            </div>
        </div>
    );
}