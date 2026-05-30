"""Unit tests for spectral_analysis_node.py — FFT / PSD computation."""
import math
from unittest.mock import MagicMock

import numpy as np
import pytest

from phone_imu_bridge.spectral_analysis_node import SpectralAnalysisNode


def _make_node(buffer_size=256, sample_rate=100.0):
    """Return a SpectralAnalysisNode with default attributes."""
    node = SpectralAnalysisNode.__new__(SpectralAnalysisNode)
    node.N = buffer_size
    node.fs = sample_rate
    node.buf_ax = []
    node.get_logger = MagicMock(return_value=MagicMock())
    return node


def _sine_signal(freq, fs, n_samples, amplitude=1.0):
    """Generate a pure sine wave."""
    t = np.arange(n_samples) / fs
    return amplitude * np.sin(2.0 * np.pi * freq * t)


# == Peak detection ========================================================

class TestPeakDetection:

    def test_pure_sine_peak(self):
        """A 10 Hz sine at 100 Hz sample rate -> peak near 10 Hz."""
        node = _make_node(buffer_size=256, sample_rate=100.0)
        signal = _sine_signal(10.0, 100.0, 256, amplitude=2.0)
        node._analyse(signal)

        log_call = node.get_logger().info.call_args[0][0]
        peak_str = log_call.split('peak=')[1].split(' Hz')[0]
        peak_freq = float(peak_str)
        assert abs(peak_freq - 10.0) < 1.0

    def test_dc_signal_peak_at_zero(self):
        """A constant (DC) signal -> peak at 0 Hz."""
        node = _make_node(buffer_size=256, sample_rate=100.0)
        signal = np.ones(256) * 5.0
        node._analyse(signal)

        log_call = node.get_logger().info.call_args[0][0]
        peak_str = log_call.split('peak=')[1].split(' Hz')[0]
        peak_freq = float(peak_str)
        assert peak_freq == pytest.approx(0.0, abs=0.5)


# == Buffer management =====================================================

class TestBufferManagement:

    def test_accumulation_before_threshold(self, make_imu_msg):
        """Messages accumulate until buffer_size is reached."""
        node = _make_node(buffer_size=64, sample_rate=100.0)
        for i in range(63):
            node.imu_cb(make_imu_msg(ax=float(i)))
        assert len(node.buf_ax) == 63
        node.get_logger().info.assert_not_called()

    def test_analysis_triggers_at_threshold(self, make_imu_msg):
        """Reaching buffer_size triggers _analyse."""
        node = _make_node(buffer_size=64, sample_rate=100.0)
        for i in range(64):
            node.imu_cb(make_imu_msg(ax=1.0))
        node.get_logger().info.assert_called_once()

    def test_buffer_reset_after_analysis(self, make_imu_msg):
        """Buffer is empty after analysis completes."""
        node = _make_node(buffer_size=64, sample_rate=100.0)
        for i in range(64):
            node.imu_cb(make_imu_msg(ax=1.0))
        assert len(node.buf_ax) == 0


# == PSD sanity =============================================================

class TestPsdSanity:

    def test_psd_values_finite_and_nonnegative(self):
        """PSD values should be finite and >= 0."""
        signal = _sine_signal(25.0, 100.0, 128) + np.random.randn(128) * 0.1
        window = np.hanning(128)
        x_win = signal * window
        fft_vals = np.fft.rfft(x_win)
        psd = (np.abs(fft_vals) ** 2) / (100.0 * np.sum(window ** 2))
        assert np.all(np.isfinite(psd))
        assert np.all(psd >= 0)

    def test_hann_window_reduces_leakage(self):
        """Hann-windowed FFT has lower max sidelobe relative to peak.

        We use a frequency that does NOT fall on an exact FFT bin to
        ensure spectral leakage is present, then show Hann suppresses
        the *worst-case* sidelobe better than a rectangular window.
        """
        N = 256
        fs = 100.0
        freq = 10.3  # deliberately off-bin
        signal = _sine_signal(freq, fs, N)

        # Rectangular window PSD
        mag_rect = np.abs(np.fft.rfft(signal)) ** 2

        # Hann window PSD
        window = np.hanning(N)
        mag_hann = np.abs(np.fft.rfft(signal * window)) ** 2

        # For each, find ratio of 3rd-largest bin to 1st-largest bin.
        # (2nd-largest is typically the adjacent main-lobe bin, which
        #  Hann also widens, so we compare sidelobes further out.)
        sorted_rect = np.sort(mag_rect)[::-1]
        sorted_hann = np.sort(mag_hann)[::-1]

        # Third-highest bin / peak gives a robust sidelobe metric
        ratio_rect = sorted_rect[3] / sorted_rect[0]
        ratio_hann = sorted_hann[3] / sorted_hann[0]

        assert ratio_hann < ratio_rect, (
            f'Hann sidelobe ratio ({ratio_hann:.4f}) should be less '
            f'than rectangular ({ratio_rect:.4f})'
        )
