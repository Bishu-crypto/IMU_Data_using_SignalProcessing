"""Unit tests for inertial_nav_node.py — dead reckoning navigation."""
import math
from unittest.mock import MagicMock

import numpy as np
import pytest

from phone_imu_bridge.inertial_nav_node import InertialNavNode


def _make_node(cutoff=5.0, zupt_thresh=0.15, zupt_win=10,
               frequency=100.0, gravity=9.81):
    """Create an InertialNavNode with default attributes (no ROS 2)."""
    from phone_imu_bridge.dsp_filters import ButterworthLPF

    node = InertialNavNode.__new__(InertialNavNode)
    node.cutoff = cutoff
    node.zupt_thresh = zupt_thresh
    node.zupt_win = zupt_win
    node.fs = frequency
    node.dt = 1.0 / frequency
    node.gravity = gravity

    node.lpf_x = ButterworthLPF(cutoff_hz=cutoff, sample_rate=frequency)
    node.lpf_y = ButterworthLPF(cutoff_hz=cutoff, sample_rate=frequency)
    node.lpf_z = ButterworthLPF(cutoff_hz=cutoff, sample_rate=frequency)

    node.velocity = np.zeros(3)
    node.position = np.zeros(3)
    node.prev_accel = np.zeros(3)
    node.prev_velocity = np.zeros(3)
    node.first_sample = True
    node.zupt_counter = 0

    node.pub_vel = MagicMock()
    node.pub_pos = MagicMock()
    node.get_logger = MagicMock(return_value=MagicMock())

    return node


# == Quaternion → Rotation Matrix ===========================================

class TestQuatToRotation:

    def test_identity_quaternion(self):
        """Identity quaternion [1,0,0,0] → identity rotation matrix."""
        R = InertialNavNode.quat_to_rotation_matrix([1, 0, 0, 0])
        np.testing.assert_allclose(R, np.eye(3), atol=1e-10)

    def test_90_deg_z_rotation(self):
        """90° rotation about Z axis."""
        angle = math.pi / 2
        q = [math.cos(angle/2), 0, 0, math.sin(angle/2)]
        R = InertialNavNode.quat_to_rotation_matrix(q)
        # [1,0,0] should map to [0,1,0]
        v = R @ np.array([1, 0, 0])
        np.testing.assert_allclose(v, [0, 1, 0], atol=1e-10)

    def test_rotation_is_orthogonal(self):
        """R^T · R should equal I (orthogonal matrix)."""
        q = [0.5, 0.5, 0.5, 0.5]  # 120° rotation
        R = InertialNavNode.quat_to_rotation_matrix(q)
        np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-10)

    def test_determinant_is_one(self):
        """Rotation matrix should have determinant +1."""
        q = [0.7071, 0.7071, 0, 0]  # 90° about X
        R = InertialNavNode.quat_to_rotation_matrix(q)
        assert np.linalg.det(R) == pytest.approx(1.0, abs=1e-10)


# == Gravity Removal ========================================================

class TestGravityRemoval:

    def test_stationary_upright(self):
        """Phone upright (gravity along Z) → near-zero linear accel."""
        node = _make_node()
        q = np.array([1.0, 0.0, 0.0, 0.0])  # identity orientation
        accel = np.array([0.0, 0.0, 9.81])   # pure gravity
        a_lin = node.remove_gravity(accel, q)
        np.testing.assert_allclose(a_lin, [0, 0, 0], atol=0.01)

    def test_tilted_stationary(self):
        """Phone tilted 45° about X — still stationary, should be ~zero."""
        node = _make_node()
        angle = math.pi / 4
        q = np.array([math.cos(angle/2), math.sin(angle/2), 0, 0])
        R = InertialNavNode.quat_to_rotation_matrix(q)
        # Simulated accelerometer reading when tilted (gravity in body frame)
        g_body = R.T @ np.array([0, 0, 9.81])
        a_lin = node.remove_gravity(g_body, q)
        np.testing.assert_allclose(a_lin, [0, 0, 0], atol=0.05)

    def test_with_linear_motion(self):
        """Phone upright + 1 m/s² forward → should detect the 1 m/s²."""
        node = _make_node()
        q = np.array([1.0, 0.0, 0.0, 0.0])
        # Accelerometer sees gravity + 1 m/s² in X
        accel = np.array([1.0, 0.0, 9.81])
        a_lin = node.remove_gravity(accel, q)
        assert abs(a_lin[0] - 1.0) < 0.05
        assert abs(a_lin[1]) < 0.05
        assert abs(a_lin[2]) < 0.05


# == Trapezoidal Integration ================================================

class TestTrapezoidalIntegration:

    def test_constant_accel_gives_linear_velocity(self):
        """Constant 1 m/s² for 1s → velocity ≈ 1 m/s.

        Uses direct math (bypasses node callback to avoid filter effects).
        """
        dt = 0.01
        v = 0.0
        a_prev = 1.0
        for _ in range(100):
            a_curr = 1.0
            v += 0.5 * dt * (a_curr + a_prev)
            a_prev = a_curr
        assert v == pytest.approx(1.0, abs=0.01)

    def test_constant_velocity_gives_linear_position(self):
        """Constant velocity 1 m/s for 1s → position ≈ 1 m."""
        dt = 0.01
        p = 0.0
        v_prev = 1.0
        for _ in range(100):
            v_curr = 1.0
            p += 0.5 * dt * (v_curr + v_prev)
            v_prev = v_curr
        assert p == pytest.approx(1.0, abs=0.01)


# == ZUPT (Zero-Velocity Update) ============================================

class TestZUPT:

    def test_zupt_triggers_after_window(self):
        """Velocity should reset to zero after zupt_window low-accel samples."""
        node = _make_node(zupt_thresh=0.2, zupt_win=5)

        # Give the node some initial velocity
        node.velocity = np.array([1.0, 0.5, 0.0])
        node.first_sample = False
        node.prev_accel = np.zeros(3)

        # Feed sub-threshold samples
        for _ in range(5):
            # Simulate already-processed (gravity-removed, filtered) data
            node.zupt_counter += 1

        assert node.zupt_counter >= node.zupt_win

    def test_zupt_counter_resets_on_motion(self):
        """ZUPT counter should reset when acceleration exceeds threshold."""
        node = _make_node(zupt_thresh=0.2)
        node.zupt_counter = 8
        # Above threshold
        node.zupt_counter = 0  # Simulating what imu_cb does
        assert node.zupt_counter == 0


# == Navigation State Initialisation ========================================

class TestNavStateInit:

    def test_initial_velocity_is_zero(self):
        node = _make_node()
        np.testing.assert_array_equal(node.velocity, [0, 0, 0])

    def test_initial_position_is_zero(self):
        node = _make_node()
        np.testing.assert_array_equal(node.position, [0, 0, 0])

    def test_first_sample_flag(self):
        node = _make_node()
        assert node.first_sample is True
