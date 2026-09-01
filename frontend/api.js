class BackendClient {
    constructor(baseUrl) {
        this.baseUrl = baseUrl;
        this.wsUrl = baseUrl.replace("http", "ws") + "/ws";
        this.ws = null;
        this.onTelemetry = null;
    }

    async get(path) {
        const r = await fetch(`${this.baseUrl}${path}`);
        return r.json();
    }

    async post(path, body) {
        const r = await fetch(`${this.baseUrl}${path}`, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: body ? JSON.stringify(body) : undefined,
        });
        const data = await r.json();
        if (!r.ok) throw new Error(data.error || `Error ${r.status}`);
        return data;
    }

    async health() {
        try {
            return await this.get("/health");
        } catch {
            return null;
        }
    }

    async scanPorts() {
        return this.get("/ports");
    }

    async connectSerial(port, baudrate) {
        return this.post("/connect", { port, baudrate });
    }

    async disconnectSerial() {
        return this.post("/disconnect");
    }

    async startMonitor() {
        return this.post("/monitor/start");
    }

    async stopMonitor() {
        return this.post("/monitor/stop");
    }

    async startControl() {
        return this.post("/start");
    }

    async stopControl() {
        return this.post("/stop");
    }

    async setController(controller) {
        return this.post("/controller", { controller });
    }

    async setGains(gains) {
        return this.post("/gains", { gains });
    }

    async calibrateLeft() {
        return this.post("/calibrate/left");
    }

    async calibrateRight() {
        return this.post("/calibrate/right");
    }

    async computeCenter() {
        return this.post("/calibrate/compute_center");
    }

    async moveToCenter() {
        return this.post("/calibrate/move_to_center");
    }

    async applyCalibration() {
        return this.post("/calibrate/apply");
    }

    async resetCalibration() {
        return this.post("/calibrate/reset");
    }

    async moveStart(direction, voltage) {
        return this.post("/move/start", { direction, voltage });
    }

    async moveStop() {
        return this.post("/move/stop");
    }

    async exportData() {
        const r = await fetch(`${this.baseUrl}/data/export`);
        if (!r.ok) throw new Error("Error al exportar");
        return r.blob();
    }

    connectWs(onMessage) {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) return;
        try {
            this.ws = new WebSocket(this.wsUrl);
            this.ws.onopen = () => {
                console.log("[WS] Conectado");
                if (onMessage) onMessage({ _connected: true });
            };
            this.ws.onmessage = (e) => {
                try {
                    const data = JSON.parse(e.data);
                    if (onMessage) onMessage(data);
                } catch { /* ignore */ }
            };
            this.ws.onclose = () => {
                console.log("[WS] Desconectado");
                setTimeout(() => this.connectWs(onMessage), 3000);
            };
            this.ws.onerror = () => { this.ws.close(); };
        } catch (e) {
            console.error("[WS] Error:", e);
        }
    }

    wsSend(cmd) {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(JSON.stringify(cmd));
        } else {
            if (cmd.controller) {
                this.setController(cmd.controller).catch(() => {});
            }
            if (cmd.gains) {
                this.setGains(cmd.gains).catch(() => {});
            }
        }
    }

    disconnectWs() {
        if (this.ws) {
            this.ws.close();
            this.ws = null;
        }
    }
}
