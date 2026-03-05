const API_BASE = 'http://localhost:5000';

export const api = {
    // Serial bağlantı
    connect: async (port = 'COM3', baudrate = 115200) => {
        const response = await fetch(`${API_BASE}/api/connect`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ port, baudrate })
        });
        return response.json();
    },


    // Modüllere (Slave) veri gönder
    sendToSlave: async (slave_id, positions) => {
        const response = await fetch(`${API_BASE}/api/send-to-slave`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ slave_id, positions })
        });
        return response.json();
    },

    homeSlave3: async () => {
        const response = await fetch(`${API_BASE}/api/home-slave3`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' }
        });
        return response.json();
    },



    // STL yükle ve interpolasyon
    uploadSTL: async (file) => {
        const formData = new FormData();
        formData.append('file', file);

        const response = await fetch(`${API_BASE}/api/upload-stl`, {
            method: 'POST',
            body: formData
        });
        return response.json();
    },

    // Master'a gönder
    sendToMaster: async (positions) => {
        const response = await fetch(`${API_BASE}/api/send-to-master`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ positions })
        });
        return response.json();
    },

    // Manuel komut (test için)
    sendCommand: async (command) => {
        const response = await fetch(`${API_BASE}/api/send-command`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ command })
        });
        return response.json();
    },

    // Serial log al
    getSerialLog: async () => {
        const response = await fetch(`${API_BASE}/api/serial-log`);
        return response.json();
    },


    // Mevcut portları listele
    getPorts: async () => {
        const response = await fetch(`${API_BASE}/api/ports`);
        const data = await response.json();
        return data;
    },
    // Sistem konfigürasyonunu al
    getConfig: async () => {
        const response = await fetch(`${API_BASE}/api/config`);
        return response.json();
    },

    // Sistem konfigürasyonunu güncelle (Demo Mode vb.)
    updateConfig: async (config) => {
        const response = await fetch(`${API_BASE}/api/config`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(config)
        });
        return response.json();
    },

    // Ağı tara (16 slave PING)
    scanSlaves: async () => {
        const response = await fetch(`${API_BASE}/api/scan`, {
            method: 'POST'
        });
        return response.json();
    },

    // Ping kontrolü olmadan tüm 16 slave'e gönder
    sendAllForce: async (positions) => {
        const response = await fetch(`${API_BASE}/api/send-all-force`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ positions })
        });
        return response.json();
    }
};