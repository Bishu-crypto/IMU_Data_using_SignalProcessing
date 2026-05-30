/**
 * app.js — Dashboard controller.
 *
 * Manages WebSocket connection (via settings modal), dispatches incoming
 * JSON frames to the chart / 3D-orientation renderers defined in
 * plots.js and orientation3d.js, and updates all metric readouts.
 */

/* ── Chart instances (from plots.js) ─────────────────────────────── */
const accelChart = new StripChart('canvas-accel', {
    maxPoints: 200, yRange: [-20, 20], lineColors: ['#6366f1', '#34d399', '#f87171'],
});
const velChart = new StripChart('canvas-velocity', {
    maxPoints: 200, yRange: [-2, 2], lineColors: ['#6366f1', '#34d399', '#f87171'],
});
const dispChart = new StripChart('canvas-displacement', {
    maxPoints: 200, yRange: [-1, 1], lineColors: ['#6366f1', '#34d399', '#f87171'],
});
const spectrumPlot = new SpectrumPlot('canvas-spectrum');

/* ── State ───────────────────────────────────────────────────────── */
let ws = null;
let msgCount = 0;
let rateCount = 0;
let startTime = Date.now();

/* ── DOM refs ────────────────────────────────────────────────────── */
const statusBadge  = document.getElementById('connection-status');
const statusDot    = statusBadge ? statusBadge.querySelector('.status-dot') : null;
const statusText   = statusBadge ? statusBadge.querySelector('.status-text') : null;
const btnConnect   = document.getElementById('btn-connect');
const btnSettings  = document.getElementById('btn-settings');
const settingsModal = document.getElementById('settings-modal');
const btnCancel    = document.getElementById('btn-cancel-settings');
const btnSave      = document.getElementById('btn-save-settings');
const wsHostInput  = document.getElementById('ws-host');
const wsPortInput  = document.getElementById('ws-port');

/* ── Metric elements ─────────────────────────────────────────────── */
const valRoll  = document.getElementById('val-roll');
const valPitch = document.getElementById('val-pitch');
const valYaw   = document.getElementById('val-yaw');
const valVx    = document.getElementById('val-vx');
const valVy    = document.getElementById('val-vy');
const valVz    = document.getElementById('val-vz');
const valPx    = document.getElementById('val-px');
const valPy    = document.getElementById('val-py');
const valPz    = document.getElementById('val-pz');
const valPtotal = document.getElementById('val-ptotal');
const valNoiseFloor = document.getElementById('val-noise-floor');
const valPeakFreq   = document.getElementById('val-peak-freq');
const valSnr        = document.getElementById('val-snr');
const valMsgCount   = document.getElementById('val-msg-count');
const valMsgRate    = document.getElementById('val-msg-rate');
const valUptime     = document.getElementById('val-uptime');

/* Pipeline step elements */
const pipeRecv     = document.getElementById('pipe-recv');
const pipeMadgwick = document.getElementById('pipe-madgwick');
const pipeFilter   = document.getElementById('pipe-filter');
const pipeNav      = document.getElementById('pipe-nav');

/* ── Quaternion → Euler helper ───────────────────────────────────── */
function quatToEuler(w, x, y, z) {
    const sinr_cosp = 2 * (w * x + y * z);
    const cosr_cosp = 1 - 2 * (x * x + y * y);
    const roll = Math.atan2(sinr_cosp, cosr_cosp);

    let sinp = 2 * (w * y - z * x);
    sinp = Math.max(-1, Math.min(1, sinp));
    const pitch = Math.asin(sinp);

    const siny_cosp = 2 * (w * z + x * y);
    const cosy_cosp = 1 - 2 * (y * y + z * z);
    const yaw = Math.atan2(siny_cosp, cosy_cosp);

    const deg = 180 / Math.PI;
    return { roll: roll * deg, pitch: pitch * deg, yaw: yaw * deg };
}

/* ── Connection logic ────────────────────────────────────────────── */
function setStatus(connected) {
    if (statusBadge) statusBadge.className = 'status-badge ' + (connected ? 'connected' : '');
    if (statusText) statusText.textContent = connected ? 'Connected' : 'Disconnected';
}

function connect(host, port) {
    if (ws) { ws.close(); ws = null; }

    const url = `ws://${host}:${port}`;
    console.log('Connecting to', url);
    ws = new WebSocket(url);

    ws.onopen = () => {
        setStatus(true);
        msgCount = 0;
        rateCount = 0;
        startTime = Date.now();
        console.log('WS connected');
    };

    ws.onclose = () => { setStatus(false); console.log('WS disconnected'); };
    ws.onerror = (e) => { setStatus(false); console.error('WS error', e); };

    ws.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);
            msgCount++;
            rateCount++;
            handleMessage(data);
        } catch (e) {
            console.warn('Parse error', e);
        }
    };
}

/* ── Message dispatch ────────────────────────────────────────────── */
function handleMessage(data) {
    switch (data.type) {
        case 'imu':
            // Orientation 3D
            Orientation3D.setQuaternion(data.qw, data.qx, data.qy, data.qz);
            const euler = quatToEuler(data.qw, data.qx, data.qy, data.qz);
            if (valRoll)  valRoll.textContent  = euler.roll.toFixed(1)  + '°';
            if (valPitch) valPitch.textContent = euler.pitch.toFixed(1) + '°';
            if (valYaw)   valYaw.textContent   = euler.yaw.toFixed(1)   + '°';

            // Acceleration chart
            accelChart.push([data.ax, data.ay, data.az]);

            // Light up pipeline
            if (pipeRecv) pipeRecv.classList.add('active');
            if (pipeMadgwick) pipeMadgwick.classList.add('active');
            break;

        case 'velocity':
            velChart.push([data.vx, data.vy, data.vz]);
            if (valVx) valVx.textContent = data.vx.toFixed(2);
            if (valVy) valVy.textContent = data.vy.toFixed(2);
            if (valVz) valVz.textContent = data.vz.toFixed(2);
            if (pipeFilter) pipeFilter.classList.add('active');
            break;

        case 'position':
            dispChart.push([data.px, data.py, data.pz]);
            if (valPx) valPx.textContent = data.px.toFixed(2);
            if (valPy) valPy.textContent = data.py.toFixed(2);
            if (valPz) valPz.textContent = data.pz.toFixed(2);
            const total = Math.sqrt(data.px ** 2 + data.py ** 2 + data.pz ** 2);
            if (valPtotal) valPtotal.textContent = total.toFixed(2);
            if (pipeNav) pipeNav.classList.add('active');
            break;

        case 'psd':
            if (data.freqs && data.psd_db) {
                spectrumPlot.update(data.freqs, data.psd_db);
                // Noise floor: mean of bottom 50%
                const sorted = [...data.psd_db].sort((a, b) => a - b);
                const nf = sorted.slice(0, Math.floor(sorted.length / 2))
                    .reduce((s, v) => s + v, 0) / Math.max(1, Math.floor(sorted.length / 2));
                const peak = Math.max(...data.psd_db);
                const peakIdx = data.psd_db.indexOf(peak);
                if (valNoiseFloor) valNoiseFloor.textContent = nf.toFixed(1) + ' dB';
                if (valPeakFreq) valPeakFreq.textContent = data.freqs[peakIdx].toFixed(1) + ' Hz';
                if (valSnr) valSnr.textContent = (peak - nf).toFixed(1) + ' dB';
            }
            break;
    }
}

/* ── Rate / uptime counter ───────────────────────────────────────── */
setInterval(() => {
    if (valMsgCount) valMsgCount.textContent = msgCount;
    if (valMsgRate) valMsgRate.textContent = rateCount + ' Hz';
    rateCount = 0;

    if (valUptime) {
        const secs = Math.floor((Date.now() - startTime) / 1000);
        const m = Math.floor(secs / 60);
        const s = secs % 60;
        valUptime.textContent = m > 0 ? `${m}m ${s}s` : `${s}s`;
    }
}, 1000);

/* ── Settings modal wiring ───────────────────────────────────────── */
if (btnSettings) btnSettings.addEventListener('click', () => {
    if (settingsModal) settingsModal.style.display = 'flex';
});
if (btnCancel) btnCancel.addEventListener('click', () => {
    if (settingsModal) settingsModal.style.display = 'none';
});
if (btnSave) btnSave.addEventListener('click', () => {
    const host = wsHostInput ? wsHostInput.value.trim() : 'localhost';
    const port = wsPortInput ? wsPortInput.value.trim() : '8765';
    localStorage.setItem('ws_host', host);
    localStorage.setItem('ws_port', port);
    if (settingsModal) settingsModal.style.display = 'none';
    connect(host, port);
});

/* ── Quick-connect button ────────────────────────────────────────── */
if (btnConnect) btnConnect.addEventListener('click', () => {
    const host = wsHostInput ? wsHostInput.value.trim() : 'localhost';
    const port = wsPortInput ? wsPortInput.value.trim() : '8765';
    connect(host, port);
});

/* ── Init 3D orientation ─────────────────────────────────────────── */
Orientation3D.init('canvas-orientation');

/* ── Auto-fill host from page URL & restore saved settings ───────── */
(function init() {
    const savedHost = localStorage.getItem('ws_host');
    const savedPort = localStorage.getItem('ws_port');

    if (savedHost && wsHostInput) wsHostInput.value = savedHost;
    else if (wsHostInput) {
        const h = window.location.hostname;
        if (h && h !== '' && h !== 'localhost' && h !== '127.0.0.1') {
            wsHostInput.value = h;
        }
    }
    if (savedPort && wsPortInput) wsPortInput.value = savedPort;

    // Auto-connect on load
    const host = wsHostInput ? wsHostInput.value.trim() : 'localhost';
    const port = wsPortInput ? wsPortInput.value.trim() : '8765';
    connect(host, port);
})();
