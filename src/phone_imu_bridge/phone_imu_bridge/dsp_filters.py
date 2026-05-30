"""Digital Signal Processing filters for IMU data.

This module implements IIR (Infinite Impulse Response) digital filters used
throughout the inertial navigation pipeline.  Every filter is implemented
from first principles — no dependency on ``scipy.signal`` — so the DSP
theory is fully transparent for academic review.

Mathematical Background
-----------------------
A 2nd-order Butterworth low-pass filter is designed via the **bilinear
transform** (Tustin's method), which maps the analog prototype transfer
function from the s-domain to the z-domain while preserving stability:

    s  =  (2/T) · (z - 1) / (z + 1)

where *T = 1/fs* is the sampling period.

The analog 2nd-order Butterworth prototype has the transfer function::

    H_a(s) = ω_c² / (s² + √2 · ω_c · s + ω_c²)

After bilinear-transforming and pre-warping the cutoff frequency::

    ω_c' = (2/T) · tan(π · f_c / f_s)

we obtain the discrete transfer function::

    H(z) = (b₀ + b₁·z⁻¹ + b₂·z⁻²) / (1 + a₁·z⁻¹ + a₂·z⁻²)

implemented as the Direct-Form II difference equation::

    y[n] = b₀·x[n] + b₁·x[n-1] + b₂·x[n-2]
                     - a₁·y[n-1] - a₂·y[n-2]

Reference: Oppenheim & Willsky, *Signals and Systems*, 2nd ed., Ch. 11.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass
class ButterworthLPF:
    """2nd-order Butterworth low-pass IIR filter (Direct-Form II).

    Parameters
    ----------
    cutoff_hz : float
        -3 dB cutoff frequency in Hz.
    sample_rate : float
        Sampling frequency in Hz.  Must satisfy Nyquist: ``cutoff_hz < sample_rate / 2``.

    Attributes
    ----------
    b : tuple[float, float, float]
        Numerator (feed-forward) coefficients ``(b0, b1, b2)``.
    a : tuple[float, float, float]
        Denominator (feedback) coefficients ``(1, a1, a2)``.
        ``a[0]`` is always 1 after normalisation.

    Example
    -------
    >>> lpf = ButterworthLPF(cutoff_hz=5.0, sample_rate=100.0)
    >>> filtered = lpf.filter(raw_sample)
    """

    cutoff_hz: float
    sample_rate: float

    # Computed coefficients (set in __post_init__)
    b: tuple[float, float, float] = field(init=False, repr=False)
    a: tuple[float, float, float] = field(init=False, repr=False)

    # Filter state (Direct-Form II transposed)
    _x1: float = field(init=False, default=0.0, repr=False)
    _x2: float = field(init=False, default=0.0, repr=False)
    _y1: float = field(init=False, default=0.0, repr=False)
    _y2: float = field(init=False, default=0.0, repr=False)

    def __post_init__(self) -> None:
        if self.cutoff_hz <= 0:
            raise ValueError(f'cutoff_hz must be positive, got {self.cutoff_hz}')
        if self.cutoff_hz >= self.sample_rate / 2:
            raise ValueError(
                f'cutoff_hz ({self.cutoff_hz}) must be < Nyquist '
                f'({self.sample_rate / 2})')

        self.b, self.a = self._design()

    # ── Coefficient design ────────────────────────────────────────────

    def _design(self) -> tuple[tuple[float, float, float],
                               tuple[float, float, float]]:
        """Compute IIR coefficients via bilinear transform.

        Steps
        -----
        1. **Pre-warp** the cutoff frequency to compensate for the
           frequency warping inherent in the bilinear transform:

               ω_d  = 2π · f_c / f_s       (digital frequency)
               ω_c' = (2·f_s) · tan(ω_d/2) (pre-warped analog freq)

        2. **Analog prototype** — 2nd-order Butterworth::

               H_a(s) = ω_c'² / (s² + √2·ω_c'·s + ω_c'²)

        3. **Bilinear transform** ``s = 2·f_s·(z-1)/(z+1)`` yields the
           discrete transfer function ``H(z) = B(z) / A(z)``.
        """
        fs = self.sample_rate
        fc = self.cutoff_hz

        # Step 1: Pre-warp
        wc = 2.0 * fs * math.tan(math.pi * fc / fs)

        # Step 2 & 3: Bilinear transform of 2nd-order Butterworth
        # Let K = 2·fs (bilinear constant)
        k = 2.0 * fs
        k2 = k * k
        wc2 = wc * wc
        sqrt2_wc_k = math.sqrt(2.0) * wc * k

        # Denominator polynomial: K² + √2·ωc·K + ωc²
        denom = k2 + sqrt2_wc_k + wc2

        # Normalised coefficients
        b0 = wc2 / denom
        b1 = 2.0 * wc2 / denom
        b2 = wc2 / denom

        a1 = (2.0 * wc2 - 2.0 * k2) / denom
        a2 = (k2 - sqrt2_wc_k + wc2) / denom

        return (b0, b1, b2), (1.0, a1, a2)

    # ── Single-sample filtering ───────────────────────────────────────

    def filter(self, x: float) -> float:
        """Filter one sample through the IIR difference equation.

        Implements Direct-Form I::

            y[n] = b0·x[n] + b1·x[n-1] + b2·x[n-2]
                            - a1·y[n-1] - a2·y[n-2]

        Parameters
        ----------
        x : float
            Current input sample.

        Returns
        -------
        float
            Filtered output sample.
        """
        b0, b1, b2 = self.b
        _, a1, a2 = self.a

        y = b0 * x + b1 * self._x1 + b2 * self._x2 \
            - a1 * self._y1 - a2 * self._y2

        # Shift state
        self._x2 = self._x1
        self._x1 = x
        self._y2 = self._y1
        self._y1 = y

        return y

    def reset(self) -> None:
        """Reset filter state to zero (e.g. after a ZUPT event)."""
        self._x1 = self._x2 = 0.0
        self._y1 = self._y2 = 0.0


@dataclass
class HighPassFromLP:
    """High-pass filter derived by spectral inversion of a Butterworth LPF.

    For gravity removal: ``accel_linear = accel_raw - LP(accel_raw)``
    is equivalent to ``accel_linear = HP(accel_raw)``.

    This approach is simpler and avoids phase issues when the LP filter
    is already available.
    """

    lpf: ButterworthLPF

    def filter(self, x: float) -> float:
        """Return ``x - LP(x)`` to remove the low-frequency (gravity) component."""
        return x - self.lpf.filter(x)

    def reset(self) -> None:
        """Reset internal LP filter state."""
        self.lpf.reset()
"""

DSP Filters — Key Interview Points
-----------------------------------
1. Why Butterworth?  Maximally flat passband — no ripple.
2. Why IIR over FIR?  Lower order → less delay, fewer coefficients.
   Trade-off: non-linear phase (acceptable for IMU since we care about
   magnitude, not exact phase alignment).
3. Why bilinear transform?  Maps entire jΩ axis to unit circle,
   preserving stability.  Main artefact: frequency warping, which we
   compensate via pre-warping.
4. Why 2nd order?  -40 dB/decade rolloff is sufficient for MEMS IMU
   noise.  Higher orders risk numerical instability in fixed-point.
"""
