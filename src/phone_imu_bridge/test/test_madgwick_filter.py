"""Unit tests for madgwick_filter_node.py — sensor fusion algorithm."""
import math
from unittest.mock import MagicMock

import numpy as np
import pytest

from phone_imu_bridge.madgwick_filter_node import MadgwickFilterNode


def _make_node(beta=0.033, frequency=100.0):
    """Return a MadgwickFilterNode with default attributes.

    The conftest injects a fake ``rclpy.node.Node`` base class so
    ``MadgwickFilterNode()`` can be constructed without a running ROS 2
    runtime.  We then override the parameters to the values we want.
    """
    node = MadgwickFilterNode.__new__(MadgwickFilterNode)
    node.beta = beta
    node.dt = 1.0 / frequency
    node.q = np.array([1.0, 0.0, 0.0, 0.0])
    node.calibrated = False
    node.calib_buf = []
    node.CALIB_SAMPLES = 50
    node.pub = MagicMock()
    node.get_logger = MagicMock(return_value=MagicMock())
    return node


# == _init_quat_from_accel ==================================================

class TestInitQuatFromAccel:

    def test_identity_gravity(self):
        """Gravity along +Z -> identity quaternion [1, 0, 0, 0]."""
        node = _make_node()
        q = node._init_quat_from_accel(np.array([0.0, 0.0, 9.81]))
        np.testing.assert_allclose(q, [1.0, 0.0, 0.0, 0.0], atol=1e-6)

    def test_tilted_forward(self):
        """45-degree forward pitch -> correct pitch quaternion."""
        node = _make_node()
        g = 9.81
        ax = -g * math.sin(math.radians(45))
        az = g * math.cos(math.radians(45))
        q = node._init_quat_from_accel(np.array([ax, 0.0, az]))
        # Verify unit norm
        assert np.linalg.norm(q) == pytest.approx(1.0, abs=1e-6)
        # Verify pitch angle: pitch = 2*arcsin(q[2]) for small roll
        # More precisely: use the Euler extraction
        # pitch = arcsin(2*(q0*q2 - q3*q1))
        pitch = math.asin(2.0 * (q[0] * q[2] - q[3] * q[1]))
        assert abs(pitch - math.radians(45)) < 0.01

    def test_normalization(self):
        """Output quaternion always has unit norm."""
        node = _make_node()
        q = node._init_quat_from_accel(np.array([100.0, 200.0, 300.0]))
        assert np.linalg.norm(q) == pytest.approx(1.0, abs=1e-6)


# == _madgwick_update =======================================================

class TestMadgwickUpdate:

    def test_stationary_stays_near_identity(self):
        """Zero gyro + gravity along Z -> quaternion stays near identity."""
        node = _make_node()
        q = np.array([1.0, 0.0, 0.0, 0.0])
        for _ in range(100):
            q = node._madgwick_update(q, 0.0, 0.0, 0.0, 0.0, 0.0, 9.81)
        np.testing.assert_allclose(q, [1.0, 0.0, 0.0, 0.0], atol=0.05)

    def test_pure_z_rotation(self):
        """Constant gyro about Z -> yaw grows over time."""
        node = _make_node(beta=0.0)  # disable accel correction
        q = np.array([1.0, 0.0, 0.0, 0.0])
        gz = 1.0  # 1 rad/s
        for _ in range(100):  # 1 second at 100 Hz
            q = node._madgwick_update(q, 0.0, 0.0, gz, 0.0, 0.0, 9.81)
        yaw = 2.0 * math.atan2(q[3], q[0])
        assert abs(yaw - 1.0) < 0.15

    def test_zero_accel_guard(self):
        """Near-zero acceleration -> quaternion returned unchanged."""
        node = _make_node()
        q_in = np.array([0.7071, 0.7071, 0.0, 0.0])
        q_out = node._madgwick_update(q_in, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        np.testing.assert_array_equal(q_out, q_in)

    def test_preserves_unit_norm(self):
        """After many noisy updates the quaternion stays unit-norm."""
        node = _make_node()
        q = np.array([1.0, 0.0, 0.0, 0.0])
        for _ in range(500):
            q = node._madgwick_update(q, 0.3, -0.1, 0.5, 2.0, -1.0, 8.5)
        assert np.linalg.norm(q) == pytest.approx(1.0, abs=1e-6)


# == Calibration phase =====================================================

class TestCalibrationPhase:

    def test_buffering_before_threshold(self, make_imu_msg):
        """First 49 callbacks do not trigger calibration."""
        node = _make_node()
        for _ in range(49):
            node.imu_cb(make_imu_msg(az=9.81))
        assert not node.calibrated
        node.pub.publish.assert_not_called()

    def test_calibration_triggers_at_threshold(self, make_imu_msg):
        """The 50th callback triggers calibration."""
        node = _make_node()
        for _ in range(50):
            node.imu_cb(make_imu_msg(az=9.81))
        assert node.calibrated

    def test_publishes_after_calibration(self, make_imu_msg):
        """After calibration, the next callback publishes fused data."""
        node = _make_node()
        for _ in range(50):
            node.imu_cb(make_imu_msg(az=9.81))
        node.imu_cb(make_imu_msg(az=9.81))
        assert node.pub.publish.call_count == 1

    def test_output_copies_header(self, make_imu_msg):
        """Fused message carries the same header as input."""
        node = _make_node()
        for _ in range(50):
            node.imu_cb(make_imu_msg(az=9.81))
        in_msg = make_imu_msg(az=9.81)
        sentinel = object()
        in_msg.header = sentinel
        node.imu_cb(in_msg)
        out_msg = node.pub.publish.call_args[0][0]
        assert out_msg.header is sentinel

    def test_output_orientation_covariance(self, make_imu_msg):
        """Fused output sets orientation_covariance[0] = 0.01."""
        node = _make_node()
        for _ in range(50):
            node.imu_cb(make_imu_msg(az=9.81))
        node.imu_cb(make_imu_msg(az=9.81))
        out_msg = node.pub.publish.call_args[0][0]
        assert out_msg.orientation_covariance[0] == pytest.approx(0.01)
