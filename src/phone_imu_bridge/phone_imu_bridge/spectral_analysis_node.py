"""Spectral analysis node for IMU noise characterisation.

Subscribes to ``/phone/imu/data_raw``, accumulates a window of
accelerometer samples (all 3 axes), then computes a Hann-windowed FFT
to estimate the one-sided power spectral density (PSD).  Results are
logged and published as JSON on ``/phone/imu/psd`` for the web dashboard.

Metrics computed per analysis window:
- **Median noise floor** (dB/Hz) — overall sensor noise level
- **Peak frequency** (Hz) — dominant vibration component
- **SNR** (dB) — signal-to-noise ratio relative to noise floor
- **Dominant frequency confidence** — ratio of peak to mean power

DSP Concepts
------------
1. **Hann window**: Reduces spectral leakage by tapering the signal
   edges to zero.  Side-lobe level ≈ -31 dB vs. 0 dB for rectangular.

2. **PSD via periodogram**: ``PSD[k] = |X[k]|² / (fs · Σ w²[n])``
   where the denominator normalises for the window's power.

3. **One-sided spectrum**: We use ``rfft`` (real FFT) which returns
   only the positive-frequency bins [0, fs/2], halving computation.

Reference: Heinzel, Rüdiger & Schilling, "Spectrum and spectral density
estimation by the Discrete Fourier transform (DFT)", 2002.
"""

import json

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String


class SpectralAnalysisNode(Node):
    def __init__(self):
        super().__init__('spectral_analysis_node')

        self.declare_parameter('buffer_size', 256)
        self.declare_parameter('sample_rate', 100.0)

        self.N  = self.get_parameter('buffer_size').value
        self.fs = self.get_parameter('sample_rate').value

        # Buffers for all 3 axes
        self.buf_ax = []
        self.buf_ay = []
        self.buf_az = []

        self.sub = self.create_subscription(
            Imu, '/phone/imu/data_raw', self.imu_cb, 10)

        # Publish PSD as JSON for the web dashboard
        self.pub_psd = self.create_publisher(
            String, '/phone/imu/psd', 10)

        self.get_logger().info(
            f'Spectral node ready — will analyse every {self.N} samples')

    def imu_cb(self, msg):
        self.buf_ax.append(msg.linear_acceleration.x)
        self.buf_ay.append(msg.linear_acceleration.y)
        self.buf_az.append(msg.linear_acceleration.z)

        if len(self.buf_ax) >= self.N:
            self._analyse(
                np.array(self.buf_ax[:self.N]),
                np.array(self.buf_ay[:self.N]),
                np.array(self.buf_az[:self.N]),
            )
            self.buf_ax = []
            self.buf_ay = []
            self.buf_az = []

    def _analyse(self, x, y, z):
        """Compute PSD for all 3 axes and publish results."""
        # Hann window — reduces spectral leakage
        window = np.hanning(self.N)
        window_power = np.sum(window ** 2)
        freqs = np.fft.rfftfreq(self.N, d=1.0 / self.fs)

        results = {}
        for axis_name, signal in [('x', x), ('y', y), ('z', z)]:
            x_win = signal * window
            fft_vals = np.fft.rfft(x_win)
            psd = (np.abs(fft_vals) ** 2) / (self.fs * window_power)
            psd_db = 10.0 * np.log10(psd + 1e-12)

            # ── Metrics ───────────────────────────────────────────
            noise_floor_db = float(np.median(psd_db))
            peak_idx = int(np.argmax(psd_db))
            peak_freq = float(freqs[peak_idx])
            peak_db = float(psd_db[peak_idx])

            # SNR: peak power relative to noise floor
            snr_db = peak_db - noise_floor_db

            # Confidence: peak / mean (linear domain)
            mean_psd = float(np.mean(psd))
            confidence = float(psd[peak_idx] / mean_psd) if mean_psd > 0 else 0.0

            results[axis_name] = {
                'noise_floor_db': round(noise_floor_db, 2),
                'peak_freq': round(peak_freq, 2),
                'peak_db': round(peak_db, 2),
                'snr_db': round(snr_db, 2),
                'confidence': round(confidence, 2),
            }

        # Log summary (X-axis for backward compatibility)
        rx = results['x']
        self.get_logger().info(
            f'[PSD] noise_floor={rx["noise_floor_db"]:.1f} dB/Hz  '
            f'peak={rx["peak_freq"]:.1f} Hz @ {rx["peak_db"]:.1f} dB  '
            f'SNR={rx["snr_db"]:.1f} dB')

        # Publish full PSD data as JSON for web dashboard
        psd_msg = String()
        # Include X-axis PSD array for plotting (keep payload manageable)
        x_win = x * window
        fft_vals = np.fft.rfft(x_win)
        psd_x = (np.abs(fft_vals) ** 2) / (self.fs * window_power)
        psd_x_db = 10.0 * np.log10(psd_x + 1e-12)

        psd_msg.data = json.dumps({
            'freqs': freqs.tolist(),
            'psd_db': psd_x_db.tolist(),
            'axes': results,
        })
        self.pub_psd.publish(psd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SpectralAnalysisNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
