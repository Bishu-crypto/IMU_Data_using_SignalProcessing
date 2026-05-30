"""Unit tests for dsp_filters.py — Butterworth IIR filter design & filtering."""
import math

import numpy as np
import pytest

from phone_imu_bridge.dsp_filters import ButterworthLPF, HighPassFromLP


# == Coefficient Design =====================================================

class TestCoefficientDesign:

    def test_coefficients_are_finite(self):
        """All filter coefficients should be finite real numbers."""
        lpf = ButterworthLPF(cutoff_hz=5.0, sample_rate=100.0)
        for c in lpf.b + lpf.a:
            assert math.isfinite(c)

    def test_dc_gain_is_unity(self):
        """At DC (z=1), H(z) should equal 1.0 for a low-pass filter.

        H(z=1) = (b0+b1+b2) / (a0+a1+a2) = 1.0
        """
        lpf = ButterworthLPF(cutoff_hz=10.0, sample_rate=100.0)
        num_sum = sum(lpf.b)
        den_sum = sum(lpf.a)
        assert num_sum / den_sum == pytest.approx(1.0, abs=1e-10)

    def test_nyquist_gain_is_zero(self):
        """At Nyquist (z=-1), low-pass gain should approach 0.

        H(z=-1) = (b0-b1+b2) / (a0-a1+a2)
        """
        lpf = ButterworthLPF(cutoff_hz=5.0, sample_rate=100.0)
        b0, b1, b2 = lpf.b
        a0, a1, a2 = lpf.a
        gain_nyquist = abs((b0 - b1 + b2) / (a0 - a1 + a2))
        assert gain_nyquist < 0.1  # Strong attenuation at Nyquist

    def test_cutoff_above_nyquist_raises(self):
        """Cutoff >= Nyquist should raise ValueError."""
        with pytest.raises(ValueError, match='Nyquist'):
            ButterworthLPF(cutoff_hz=50.0, sample_rate=100.0)

    def test_negative_cutoff_raises(self):
        """Negative cutoff should raise ValueError."""
        with pytest.raises(ValueError, match='positive'):
            ButterworthLPF(cutoff_hz=-1.0, sample_rate=100.0)

    def test_coefficients_symmetric_b(self):
        """For Butterworth LPF, b0 == b2 (numerator symmetry)."""
        lpf = ButterworthLPF(cutoff_hz=10.0, sample_rate=200.0)
        assert lpf.b[0] == pytest.approx(lpf.b[2], abs=1e-12)

    def test_b1_equals_2b0(self):
        """For 2nd-order Butterworth, b1 = 2·b0."""
        lpf = ButterworthLPF(cutoff_hz=15.0, sample_rate=100.0)
        assert lpf.b[1] == pytest.approx(2.0 * lpf.b[0], abs=1e-12)


# == Frequency Response =====================================================

class TestFrequencyResponse:

    def _measure_gain(self, lpf, freq_hz, n_samples=10000):
        """Measure filter gain at a specific frequency by feeding a sine."""
        fs = lpf.sample_rate
        t = np.arange(n_samples) / fs
        x = np.sin(2 * np.pi * freq_hz * t)
        y = np.array([lpf.filter(xi) for xi in x])

        # Discard transient (first 20%)
        y_ss = y[n_samples // 5:]
        x_ss = x[n_samples // 5:]

        gain = np.max(np.abs(y_ss)) / max(np.max(np.abs(x_ss)), 1e-12)
        return gain

    def test_passband_gain_near_unity(self):
        """Signal well below cutoff should pass with ~unity gain."""
        lpf = ButterworthLPF(cutoff_hz=20.0, sample_rate=200.0)
        gain = self._measure_gain(lpf, 2.0)
        assert gain == pytest.approx(1.0, abs=0.05)

    def test_cutoff_gain_is_minus_3db(self):
        """At the cutoff frequency, gain should be ~0.707 (-3 dB)."""
        lpf = ButterworthLPF(cutoff_hz=20.0, sample_rate=200.0)
        gain = self._measure_gain(lpf, 20.0)
        expected = 10 ** (-3.0 / 20.0)  # 0.7079
        assert gain == pytest.approx(expected, abs=0.05)

    def test_stopband_attenuation(self):
        """Signal well above cutoff should be heavily attenuated."""
        lpf = ButterworthLPF(cutoff_hz=10.0, sample_rate=200.0)
        gain = self._measure_gain(lpf, 80.0)
        assert gain < 0.02  # > 34 dB attenuation

    def test_monotonic_rolloff(self):
        """Gain should decrease monotonically above cutoff."""
        lpf = ButterworthLPF(cutoff_hz=10.0, sample_rate=200.0)
        gains = []
        for f in [10, 20, 30, 40, 50]:
            lpf.reset()
            gains.append(self._measure_gain(lpf, f))
        for i in range(len(gains) - 1):
            assert gains[i] >= gains[i + 1]


# == Filtering Behaviour =====================================================

class TestFiltering:

    def test_constant_input_passes_through(self):
        """DC signal should pass through LPF unchanged (after transient)."""
        lpf = ButterworthLPF(cutoff_hz=5.0, sample_rate=100.0)
        dc_value = 3.14
        outputs = [lpf.filter(dc_value) for _ in range(500)]
        # After settling, output should equal input
        assert outputs[-1] == pytest.approx(dc_value, abs=0.01)

    def test_reset_clears_state(self):
        """After reset, filter should behave as freshly constructed."""
        lpf = ButterworthLPF(cutoff_hz=5.0, sample_rate=100.0)
        for _ in range(100):
            lpf.filter(10.0)
        lpf.reset()
        # First output after reset with 0 input should be 0
        assert lpf.filter(0.0) == pytest.approx(0.0, abs=1e-12)

    def test_noise_reduction(self):
        """LPF should reduce RMS of high-frequency noise."""
        lpf = ButterworthLPF(cutoff_hz=5.0, sample_rate=100.0)
        np.random.seed(42)
        noise = np.random.randn(1000) * 2.0
        filtered = np.array([lpf.filter(n) for n in noise])
        assert np.std(filtered[200:]) < np.std(noise[200:])


# == HighPassFromLP ==========================================================

class TestHighPassFromLP:

    def test_removes_dc(self):
        """HP filter should remove DC offset (gravity-like constant)."""
        lpf = ButterworthLPF(cutoff_hz=0.5, sample_rate=100.0)
        hpf = HighPassFromLP(lpf)
        dc = 9.81
        outputs = [hpf.filter(dc) for _ in range(2000)]
        # After settling, DC should be removed
        assert abs(outputs[-1]) < 0.1

    def test_passes_high_frequency(self):
        """HP filter should pass signals above the cutoff."""
        lpf = ButterworthLPF(cutoff_hz=1.0, sample_rate=100.0)
        hpf = HighPassFromLP(lpf)
        # 10 Hz signal (well above 1 Hz cutoff)
        t = np.arange(5000) / 100.0
        signal = np.sin(2 * np.pi * 10.0 * t)
        output = np.array([hpf.filter(s) for s in signal])
        # After transient, amplitude should be close to 1
        assert np.max(np.abs(output[1000:])) > 0.8
