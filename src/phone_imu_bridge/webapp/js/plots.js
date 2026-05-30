/**
 * plots.js — Lightweight Canvas-based real-time strip chart & spectrum plot.
 */

class StripChart {
    /**
     * @param {string} canvasId — HTML canvas element ID
     * @param {object} opts — { maxPoints, lineColors, yRange, yLabel }
     */
    constructor(canvasId, opts = {}) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas ? this.canvas.getContext('2d') : null;
        this.maxPoints = opts.maxPoints || 200;
        this.lineColors = opts.lineColors || ['#6366f1', '#34d399', '#f87171'];
        this.yRange = opts.yRange || [-20, 20];
        this.yLabel = opts.yLabel || '';
        this.series = this.lineColors.map(() => []);
    }

    push(values) {
        for (let i = 0; i < values.length && i < this.series.length; i++) {
            this.series[i].push(values[i]);
            if (this.series[i].length > this.maxPoints) this.series[i].shift();
        }
        this._draw();
    }

    _draw() {
        if (!this.ctx) return;
        const W = this.canvas.width;
        const H = this.canvas.height;
        const ctx = this.ctx;
        const pad = { t: 8, b: 20, l: 40, r: 10 };
        const plotW = W - pad.l - pad.r;
        const plotH = H - pad.t - pad.b;

        ctx.clearRect(0, 0, W, H);

        // Grid
        ctx.strokeStyle = 'rgba(255,255,255,0.06)';
        ctx.lineWidth = 1;
        const gridLines = 5;
        for (let i = 0; i <= gridLines; i++) {
            const y = pad.t + (plotH / gridLines) * i;
            ctx.beginPath(); ctx.moveTo(pad.l, y); ctx.lineTo(W - pad.r, y); ctx.stroke();
        }

        // Y-axis labels
        ctx.fillStyle = 'rgba(156, 163, 175, 0.6)';
        ctx.font = '10px JetBrains Mono, monospace';
        ctx.textAlign = 'right';
        for (let i = 0; i <= gridLines; i++) {
            const y = pad.t + (plotH / gridLines) * i;
            const val = this.yRange[1] - (this.yRange[1] - this.yRange[0]) * (i / gridLines);
            ctx.fillText(val.toFixed(1), pad.l - 4, y + 3);
        }

        // Zero line
        const zeroY = pad.t + plotH * (this.yRange[1] / (this.yRange[1] - this.yRange[0]));
        if (zeroY >= pad.t && zeroY <= pad.t + plotH) {
            ctx.strokeStyle = 'rgba(255,255,255,0.12)';
            ctx.lineWidth = 1;
            ctx.setLineDash([4, 4]);
            ctx.beginPath(); ctx.moveTo(pad.l, zeroY); ctx.lineTo(W - pad.r, zeroY); ctx.stroke();
            ctx.setLineDash([]);
        }

        // Data lines
        for (let s = 0; s < this.series.length; s++) {
            const data = this.series[s];
            if (data.length < 2) continue;
            ctx.strokeStyle = this.lineColors[s];
            ctx.lineWidth = 1.5;
            ctx.beginPath();
            for (let i = 0; i < data.length; i++) {
                const x = pad.l + (i / (this.maxPoints - 1)) * plotW;
                const norm = (data[i] - this.yRange[0]) / (this.yRange[1] - this.yRange[0]);
                const y = pad.t + plotH * (1 - Math.max(0, Math.min(1, norm)));
                if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
            }
            ctx.stroke();
        }

        // Legend
        ctx.font = '9px Inter, sans-serif';
        ctx.textAlign = 'left';
        const labels = ['X', 'Y', 'Z'];
        for (let s = 0; s < Math.min(this.series.length, 3); s++) {
            const lx = pad.l + 8 + s * 35;
            ctx.fillStyle = this.lineColors[s];
            ctx.fillRect(lx, H - 14, 12, 3);
            ctx.fillText(labels[s], lx + 15, H - 10);
        }
    }

    clear() {
        this.series = this.lineColors.map(() => []);
        if (this.ctx) this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
    }
}


class SpectrumPlot {
    constructor(canvasId) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas ? this.canvas.getContext('2d') : null;
        this.freqs = [];
        this.psd_db = [];
    }

    update(freqs, psd_db) {
        this.freqs = freqs;
        this.psd_db = psd_db;
        this._draw();
    }

    _draw() {
        if (!this.ctx || this.freqs.length === 0) return;
        const W = this.canvas.width;
        const H = this.canvas.height;
        const ctx = this.ctx;
        const pad = { t: 8, b: 24, l: 45, r: 10 };
        const plotW = W - pad.l - pad.r;
        const plotH = H - pad.t - pad.b;

        ctx.clearRect(0, 0, W, H);

        const maxFreq = this.freqs[this.freqs.length - 1] || 50;
        const minDb = Math.min(...this.psd_db) - 5;
        const maxDb = Math.max(...this.psd_db) + 5;

        // Grid
        ctx.strokeStyle = 'rgba(255,255,255,0.06)';
        ctx.lineWidth = 1;
        for (let i = 0; i <= 4; i++) {
            const y = pad.t + (plotH / 4) * i;
            ctx.beginPath(); ctx.moveTo(pad.l, y); ctx.lineTo(W - pad.r, y); ctx.stroke();
            ctx.fillStyle = 'rgba(156,163,175,0.6)';
            ctx.font = '9px JetBrains Mono, monospace';
            ctx.textAlign = 'right';
            const val = maxDb - (maxDb - minDb) * (i / 4);
            ctx.fillText(val.toFixed(0), pad.l - 4, y + 3);
        }

        // X-axis labels
        ctx.textAlign = 'center';
        for (let f = 0; f <= maxFreq; f += 10) {
            const x = pad.l + (f / maxFreq) * plotW;
            ctx.fillText(f + '', x, H - 6);
        }

        // Spectrum line with gradient fill
        const grad = ctx.createLinearGradient(0, pad.t, 0, pad.t + plotH);
        grad.addColorStop(0, 'rgba(99, 102, 241, 0.3)');
        grad.addColorStop(1, 'rgba(99, 102, 241, 0.02)');

        ctx.beginPath();
        ctx.moveTo(pad.l, pad.t + plotH);
        for (let i = 0; i < this.psd_db.length; i++) {
            const x = pad.l + (this.freqs[i] / maxFreq) * plotW;
            const norm = (this.psd_db[i] - minDb) / (maxDb - minDb);
            const y = pad.t + plotH * (1 - Math.max(0, Math.min(1, norm)));
            ctx.lineTo(x, y);
        }
        ctx.lineTo(pad.l + plotW, pad.t + plotH);
        ctx.closePath();
        ctx.fillStyle = grad;
        ctx.fill();

        // Spectrum line on top
        ctx.strokeStyle = '#6366f1';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        for (let i = 0; i < this.psd_db.length; i++) {
            const x = pad.l + (this.freqs[i] / maxFreq) * plotW;
            const norm = (this.psd_db[i] - minDb) / (maxDb - minDb);
            const y = pad.t + plotH * (1 - Math.max(0, Math.min(1, norm)));
            if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
        }
        ctx.stroke();

        // Peak marker
        const peakIdx = this.psd_db.indexOf(Math.max(...this.psd_db));
        if (peakIdx >= 0) {
            const px = pad.l + (this.freqs[peakIdx] / maxFreq) * plotW;
            const pNorm = (this.psd_db[peakIdx] - minDb) / (maxDb - minDb);
            const py = pad.t + plotH * (1 - pNorm);
            ctx.fillStyle = '#f87171';
            ctx.beginPath(); ctx.arc(px, py, 4, 0, Math.PI * 2); ctx.fill();
            ctx.fillStyle = 'rgba(248,113,113,0.8)';
            ctx.font = '9px JetBrains Mono, monospace';
            ctx.textAlign = 'left';
            ctx.fillText(this.freqs[peakIdx].toFixed(1) + ' Hz', px + 6, py - 4);
        }
    }
}
