import React from 'react';
import { Upload, FileCode, CheckCircle2, Loader2, Play } from 'lucide-react';

export default function FileUpload({ file, onFileChange, onCalculate, isProcessing, onDrop }) {
    return (
        <div className="bg-white/5 backdrop-blur-3xl rounded-xl p-3 border border-white/10 shadow-lg relative overflow-hidden">
            <div className="flex flex-col gap-3">
                {/* Upload Zone - Compact Bar */}
                <div
                    onClick={() => document.getElementById('fileInput').click()}
                    className={`flex items-center gap-3 px-3 py-2 rounded-lg border border-dashed transition-all duration-300 cursor-pointer ${file
                            ? 'border-emerald-500/40 bg-emerald-500/5 hover:bg-emerald-500/10'
                            : 'border-white/10 hover:border-blue-500/40 hover:bg-white/5'
                        }`}
                >
                    <input
                        id="fileInput"
                        type="file"
                        className="hidden"
                        accept=".stl"
                        onChange={(e) => onFileChange(e.target.files[0])}
                    />

                    <div className={`${file ? 'text-emerald-400' : 'text-blue-400'}`}>
                        {file ? <FileCode size={16} /> : <Upload size={16} />}
                    </div>

                    <div className="flex-1 overflow-hidden">
                        <p className="text-[11px] font-bold truncate text-gray-200">
                            {file ? file.name : 'STL Dosyası Seç'}
                        </p>
                    </div>

                    {file && <CheckCircle2 className="text-emerald-500 animate-in zoom-in" size={14} />}
                </div>

                {/* Calculate Button - Solid Green */}
                <button
                    onClick={onCalculate}
                    disabled={!file || isProcessing}
                    className={`h-[36px] w-full rounded-lg font-bold flex items-center justify-center gap-2 transition-all duration-300 ${!file || isProcessing
                            ? 'bg-white/5 text-gray-500 cursor-not-allowed'
                            : 'bg-emerald-600 hover:bg-emerald-500 text-white shadow-md active:scale-95'
                        }`}
                >
                    {isProcessing ? (
                        <Loader2 className="animate-spin" size={14} />
                    ) : (
                        <>
                            <Play size={12} fill="currentColor" />
                            <span className="text-[10px] uppercase tracking-widest">Hesapla</span>
                        </>
                    )}
                </button>
            </div>
        </div>
    );
}
