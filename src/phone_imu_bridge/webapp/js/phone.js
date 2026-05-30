/**
 * Phone IMU Transmitter — reads device motion sensors and streams
 * accelerometer + gyroscope data to the ROS 2 HTTP receiver as JSON.
 *
 * Wired to the phone.html UI: reads endpoint & interval from config
 * inputs, drives the btn-start / btn-stop buttons, and updates the
 * live sensor value grid + sample counter.
 */

/* ── State ─────────────────────────────────────────────────────── */
let streaming = false;
let sampleCount = 0;
let rateCount = 0;
let rateTimer = null;
let sendTimer = null;

// Latest sensor readings (updated on every devicemotion event)
let latestAccel = { x: 0, y: 0, z: 0 };
let latestGyro  = { x: 0, y: 0, z: 0 };

/* ── DOM refs ──────────────────────────────────────────────────── */
const btnStart        = document.getElementById('btn-start');
const btnStop         = document.getElementById('btn-stop');
const streamIndicator = document.getElementById('stream-indicator');
const streamText      = document.getElementById('stream-text');
const serverUrlInput  = document.getElementById('server-url');
const intervalInput   = document.getElementById('send-interval');
const sampleCountEl   = document.getElementById('sample-count');
const sendRateEl      = document.getElementById('send-rate');

// Sensor value display elements
const sAx = document.getElementById('s-ax');
const sAy = document.getElementById('s-ay');
const sAz = document.getElementById('s-az');
const sGx = document.getElementById('s-gx');
const sGy = document.getElementById('s-gy');
const sGz = document.getElementById('s-gz');

/* ── Sensor listener ───────────────────────────────────────────── */

function onDeviceMotion(event) {
    const a = event.accelerationIncludingGravity || {};
    const r = event.rotationRate || {};

    latestAccel.x = a.x || 0;
    latestAccel.y = a.y || 0;
    latestAccel.z = a.z || 0;

    // rotationRate is in deg/s — convert to rad/s for ROS
    latestGyro.x = ((r.alpha || 0) * Math.PI) / 180;
    latestGyro.y = ((r.beta  || 0) * Math.PI) / 180;
    latestGyro.z = ((r.gamma || 0) * Math.PI) / 180;

    // Update the live display
    if (sAx) sAx.textContent = latestAccel.x.toFixed(2);
    if (sAy) sAy.textContent = latestAccel.y.toFixed(2);
    if (sAz) sAz.textContent = latestAccel.z.toFixed(2);
    if (sGx) sGx.textContent = latestGyro.x.toFixed(2);
    if (sGy) sGy.textContent = latestGyro.y.toFixed(2);
    if (sGz) sGz.textContent = latestGyro.z.toFixed(2);
}

/* ── HTTP sender (runs on a timer at the configured interval) ──── */

function sendSample() {
    const url = serverUrlInput ? serverUrlInput.value.trim() : '';
    if (!url) return;

    const payload = [{
        payload: [
            {
                name: 'accelerometer',
                values: { x: latestAccel.x, y: latestAccel.y, z: latestAccel.z }
            },
            {
                name: 'gyroscope',
                values: { x: latestGyro.x, y: latestGyro.y, z: latestGyro.z }
            }
        ]
    }];

    fetch(url, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload),
    }).then(() => {
        sampleCount++;
        rateCount++;
        if (sampleCountEl) sampleCountEl.textContent = sampleCount;
    }).catch(err => {
        console.warn('Send failed:', err.message);
    });
}

/* ── Start / Stop controls ─────────────────────────────────────── */

function startStreaming() {
    // Request sensor permission (required on iOS 13+, some Android)
    if (typeof DeviceMotionEvent !== 'undefined' &&
        typeof DeviceMotionEvent.requestPermission === 'function') {
        DeviceMotionEvent.requestPermission()
            .then(state => {
                if (state === 'granted') {
                    beginStream();
                } else {
                    alert('Motion sensor permission denied.');
                }
            })
            .catch(err => {
                console.error('Permission error:', err);
                alert('Could not request motion permission. Try HTTPS.');
            });
    } else {
        beginStream();
    }
}

function beginStream() {
    streaming = true;
    sampleCount = 0;
    rateCount = 0;

    // Attach the devicemotion listener
    window.addEventListener('devicemotion', onDeviceMotion);

    // Start the send timer
    const intervalMs = parseInt(intervalInput ? intervalInput.value : '50', 10) || 50;
    sendTimer = setInterval(sendSample, intervalMs);

    // Start the rate counter (updates once per second)
    rateTimer = setInterval(() => {
        if (sendRateEl) sendRateEl.textContent = rateCount;
        rateCount = 0;
    }, 1000);

    // Update UI
    if (btnStart) btnStart.style.display = 'none';
    if (btnStop)  btnStop.style.display  = 'block';
    if (streamIndicator) {
        streamIndicator.classList.remove('idle');
        streamIndicator.classList.add('streaming');
    }
    if (streamText) streamText.textContent = 'Streaming to server…';

    console.log(`Streaming started → ${serverUrlInput.value} @ ${intervalMs}ms`);
}

function stopStreaming() {
    streaming = false;

    window.removeEventListener('devicemotion', onDeviceMotion);

    if (sendTimer) { clearInterval(sendTimer); sendTimer = null; }
    if (rateTimer) { clearInterval(rateTimer); rateTimer = null; }

    // Update UI
    if (btnStart) btnStart.style.display = 'block';
    if (btnStop)  btnStop.style.display  = 'none';
    if (streamIndicator) {
        streamIndicator.classList.remove('streaming');
        streamIndicator.classList.add('idle');
    }
    if (streamText) streamText.textContent = 'Stopped';
    if (sendRateEl) sendRateEl.textContent = '0';

    console.log('Streaming stopped');
}

/* ── Wire up buttons ───────────────────────────────────────────── */
if (btnStart) btnStart.addEventListener('click', startStreaming);
if (btnStop)  btnStop.addEventListener('click', stopStreaming);

/* ── Auto-fill server URL from current page host ──────────────── */
(function autoFillHost() {
    if (!serverUrlInput) return;
    const host = window.location.hostname;
    if (host && host !== 'localhost' && host !== '127.0.0.1') {
        serverUrlInput.value = `http://${host}:5555/data`;
    }
})();
