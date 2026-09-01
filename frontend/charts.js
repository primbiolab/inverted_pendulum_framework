class RealtimeChart {
  constructor(canvas, opts = {}) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.dpr = Math.min(window.devicePixelRatio || 1, 2);
    this.maxPoints = opts.maxPoints || 300;
    this.color = opts.color || "#89b4fa";
    this.minY = opts.minY || -10;
    this.maxY = opts.maxY || 10;
    this.fixedHeight = opts.height || 180;
    this.compact = opts.compact || false;
    this.data = [];
    this.paused = false;
    this.setupCanvas();
  }

  setupCanvas() {
    const rect = this.canvas.parentElement.getBoundingClientRect();
    const w = this.compact ? Math.max(rect.width - 20, 80) : Math.max(rect.width - 28, 200);
    const h = this.compact ? Math.max(rect.height - 24, 50) : this.fixedHeight;
    this.canvas.style.width = w + "px";
    this.canvas.style.height = h + "px";
    this.canvas.width = w * this.dpr;
    this.canvas.height = h * this.dpr;
    this.ctx.scale(this.dpr, this.dpr);
    this.W = w;
    this.H = h;
  }

  push(value) {
    if (this.paused) return;
    this.data.push(value);
    if (this.data.length > this.maxPoints) this.data.shift();
  }

  clear() { this.data = []; this.draw(); }

  draw() {
    const ctx = this.ctx, w = this.W, h = this.H;
    ctx.clearRect(0, 0, w, h);

    const colors = { bg: "#1e1e2e", surface: "#313244", overlay: "#45475a", muted: "#6c7086", fg: "#cdd6f4" };
    ctx.fillStyle = colors.bg;
    ctx.fillRect(0, 0, w, h);

    const pad = { top: 10, bottom: 20, left: 8, right: 8 };
    const plotW = w - pad.left - pad.right;
    const plotH = h - pad.top - pad.bottom;

    ctx.strokeStyle = colors.surface;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(pad.left, pad.top);
    ctx.lineTo(pad.left, h - pad.bottom);
    ctx.lineTo(w - pad.right, h - pad.bottom);
    ctx.stroke();

    const yRange = this.maxY - this.minY;
    const zeroY = pad.top + plotH * (this.maxY / yRange);
    ctx.strokeStyle = colors.overlay;
    ctx.setLineDash([3, 3]);
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(pad.left, zeroY);
    ctx.lineTo(w - pad.right, zeroY);
    ctx.stroke();
    ctx.setLineDash([]);

    if (this.data.length < 2) { this.drawLabels(); return; }

    const stepX = plotW / (this.maxPoints - 1);
    const offsetX = plotW - this.data.length * stepX;

    ctx.strokeStyle = this.color;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    for (let i = 0; i < this.data.length; i++) {
      const x = pad.left + Math.max(0, offsetX + i * stepX);
      const y = pad.top + plotH * (1 - (this.data[i] - this.minY) / yRange);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();
    this.drawLabels();

    const fs = this.compact ? '8px monospace' : '10px "JetBrains Mono", monospace';
    ctx.fillStyle = colors.overlay;
    ctx.font = fs;
    ctx.fillText(`n=${this.data.length}`, w - pad.right - 40, h - pad.bottom - 2);
  }

  drawLabels() {
    const ctx = this.ctx, w = this.W, h = this.H;
    const colors = { muted: "#6c7086" };
    const fs = this.compact ? '8px monospace' : '10px "JetBrains Mono", monospace';
    ctx.font = fs;
    ctx.fillStyle = colors.muted;
    ctx.fillText(this.maxY.toFixed(this.compact ? 0 : 1), 6, 14);
    ctx.fillText(this.minY.toFixed(this.compact ? 0 : 1), 6, h - 14);
  }
}
