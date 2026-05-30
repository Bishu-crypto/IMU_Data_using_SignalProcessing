"""Shared fixtures for phone_imu_bridge unit tests.

Mock the ROS 2 runtime so all tests run with plain ``pytest`` + ``numpy``.
Handles partial ROS 2 installations where sensor_msgs exists but
std_msgs.msg.Header may not be importable outside a sourced workspace.
"""
import sys
import types
from unittest.mock import MagicMock

import pytest


# ===================================================================
# Lightweight message stand-ins
# ===================================================================

class _Vec3:
    """Stand-in for geometry_msgs.msg.Vector3."""
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _Quaternion:
    """Stand-in for geometry_msgs.msg.Quaternion."""
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 0.0


class _Header:
    """Stand-in for std_msgs.msg.Header."""
    def __init__(self):
        self.frame_id = ''
        self.stamp = MagicMock()


class ImuMsg:
    """Lightweight stand-in for sensor_msgs.msg.Imu."""
    def __init__(self):
        self.header = _Header()
        self.linear_acceleration = _Vec3()
        self.angular_velocity = _Vec3()
        self.orientation = _Quaternion()
        self.orientation_covariance = [0.0] * 9


# ===================================================================
# Fake rclpy.node.Node base class
# ===================================================================

class _FakeNode:
    """Minimal base class standing in for rclpy.node.Node."""

    def __init__(self, name='', **kwargs):
        pass

    def declare_parameter(self, name, value=None):
        return MagicMock(value=value)

    def get_parameter(self, name):
        return MagicMock(value=None)

    def create_subscription(self, *a, **kw):
        return MagicMock()

    def create_publisher(self, *a, **kw):
        return MagicMock()

    def get_logger(self):
        return MagicMock()

    def get_clock(self):
        clock = MagicMock()
        clock.now.return_value.to_msg.return_value = MagicMock()
        return clock


# ===================================================================
# Inject mocks into sys.modules BEFORE any source file is imported.
# This runs at conftest load time (before test collection).
# ===================================================================

def _inject_mocks():
    """Replace ROS 2 modules with lightweight fakes."""

    # --- rclpy ---
    rclpy_mod = types.ModuleType('rclpy')
    rclpy_mod.init = MagicMock()
    rclpy_mod.spin = MagicMock()
    rclpy_mod.shutdown = MagicMock()
    sys.modules['rclpy'] = rclpy_mod

    rclpy_node_mod = types.ModuleType('rclpy.node')
    rclpy_node_mod.Node = _FakeNode
    sys.modules['rclpy.node'] = rclpy_node_mod

    # --- sensor_msgs ---
    sensor_msgs_mod = types.ModuleType('sensor_msgs')
    sensor_msgs_mod.__path__ = []
    sys.modules['sensor_msgs'] = sensor_msgs_mod

    sensor_msgs_msg_mod = types.ModuleType('sensor_msgs.msg')
    sensor_msgs_msg_mod.Imu = ImuMsg
    sys.modules['sensor_msgs.msg'] = sensor_msgs_msg_mod

    # --- std_msgs (in case anything tries to import Header) ---
    std_msgs_mod = types.ModuleType('std_msgs')
    std_msgs_mod.__path__ = []
    sys.modules['std_msgs'] = std_msgs_mod

    std_msgs_msg_mod = types.ModuleType('std_msgs.msg')
    std_msgs_msg_mod.Header = _Header
    sys.modules['std_msgs.msg'] = std_msgs_msg_mod

    # --- geometry_msgs ---
    geo_mod = types.ModuleType('geometry_msgs')
    geo_mod.__path__ = []
    sys.modules['geometry_msgs'] = geo_mod

    geo_msg_mod = types.ModuleType('geometry_msgs.msg')
    geo_msg_mod.Vector3 = _Vec3
    geo_msg_mod.Quaternion = _Quaternion
    sys.modules['geometry_msgs.msg'] = geo_msg_mod


# Execute at import time so every ``from sensor_msgs.msg import Imu``
# in the source modules resolves to our lightweight ImuMsg.
_inject_mocks()


# ===================================================================
# Fixtures
# ===================================================================

@pytest.fixture
def make_imu_msg():
    """Factory that creates ImuMsg instances with given accel / gyro."""

    def _factory(ax=0.0, ay=0.0, az=0.0, gx=0.0, gy=0.0, gz=0.0):
        msg = ImuMsg()
        msg.linear_acceleration.x = float(ax)
        msg.linear_acceleration.y = float(ay)
        msg.linear_acceleration.z = float(az)
        msg.angular_velocity.x = float(gx)
        msg.angular_velocity.y = float(gy)
        msg.angular_velocity.z = float(gz)
        return msg

    return _factory
