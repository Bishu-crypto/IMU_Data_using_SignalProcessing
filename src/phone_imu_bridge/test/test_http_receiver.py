"""Unit tests for http_receiver_node.py — HTTP/JSON -> IMU message parsing."""
import io
import json
from unittest.mock import MagicMock

import pytest

from phone_imu_bridge.http_receiver_node import HTTPHandler


def _make_handler(body_bytes):
    """Create an HTTPHandler wired to a mock publisher, pre-loaded with body.

    Returns (handler, publisher_mock).
    """
    publisher = MagicMock()
    ros_node = MagicMock()
    ros_node.get_clock.return_value.now.return_value.to_msg.return_value = (
        MagicMock()
    )

    HTTPHandler.publisher = publisher
    HTTPHandler.ros_node = ros_node

    handler = HTTPHandler.__new__(HTTPHandler)
    handler.headers = {'Content-Length': str(len(body_bytes))}
    handler.rfile = io.BytesIO(body_bytes)
    handler.wfile = io.BytesIO()
    handler.send_response = MagicMock()
    handler.end_headers = MagicMock()
    handler.send_header = MagicMock()

    return handler, publisher


# -- Single accelerometer payload ------------------------------------------

class TestSingleAccelPayload:
    def test_accel_values_populated(self):
        payload = [
            {
                'payload': [
                    {
                        'name': 'accelerometer',
                        'values': {'x': 1.1, 'y': 2.2, 'z': 9.8},
                    }
                ]
            }
        ]
        handler, pub = _make_handler(json.dumps(payload).encode())
        handler.do_POST()

        assert pub.publish.call_count == 1
        msg = pub.publish.call_args[0][0]
        assert abs(msg.linear_acceleration.x - 1.1) < 1e-6
        assert abs(msg.linear_acceleration.y - 2.2) < 1e-6
        assert abs(msg.linear_acceleration.z - 9.8) < 1e-6


# -- Single gyroscope payload ----------------------------------------------

class TestSingleGyroPayload:
    def test_gyro_values_populated(self):
        payload = [
            {
                'payload': [
                    {
                        'name': 'gyroscope',
                        'values': {'x': 0.01, 'y': -0.02, 'z': 0.03},
                    }
                ]
            }
        ]
        handler, pub = _make_handler(json.dumps(payload).encode())
        handler.do_POST()

        assert pub.publish.call_count == 1
        msg = pub.publish.call_args[0][0]
        assert abs(msg.angular_velocity.x - 0.01) < 1e-6
        assert abs(msg.angular_velocity.y - (-0.02)) < 1e-6
        assert abs(msg.angular_velocity.z - 0.03) < 1e-6


# -- Batch payload ----------------------------------------------------------

class TestBatchPayload:
    def test_multiple_entries_published(self):
        payload = [
            {
                'payload': [
                    {
                        'name': 'accelerometer',
                        'values': {'x': 1, 'y': 2, 'z': 3},
                    }
                ]
            },
            {
                'payload': [
                    {
                        'name': 'accelerometer',
                        'values': {'x': 4, 'y': 5, 'z': 6},
                    }
                ]
            },
        ]
        handler, pub = _make_handler(json.dumps(payload).encode())
        handler.do_POST()
        assert pub.publish.call_count == 2


# -- Alternate key format (ax/ay/az) ----------------------------------------

class TestAlternateKeyFormat:
    def test_ax_ay_az_keys(self):
        payload = [
            {
                'payload': [
                    {
                        'name': '',
                        'values': {'ax': 3.0, 'ay': 4.0, 'az': 5.0},
                    }
                ]
            }
        ]
        handler, pub = _make_handler(json.dumps(payload).encode())
        handler.do_POST()

        msg = pub.publish.call_args[0][0]
        assert abs(msg.linear_acceleration.x - 3.0) < 1e-6
        assert abs(msg.linear_acceleration.y - 4.0) < 1e-6
        assert abs(msg.linear_acceleration.z - 5.0) < 1e-6


# -- Malformed JSON ---------------------------------------------------------

class TestMalformedJson:
    def test_no_crash_on_garbage(self):
        handler, pub = _make_handler(b'{{{not json!!!')
        handler.do_POST()
        assert pub.publish.call_count == 0
        HTTPHandler.ros_node.get_logger.return_value.warn.assert_called_once()


# -- Empty payload -----------------------------------------------------------

class TestEmptyPayload:
    def test_empty_list_no_publish(self):
        handler, pub = _make_handler(json.dumps([]).encode())
        handler.do_POST()
        assert pub.publish.call_count == 0


# -- Orientation covariance flag ---------------------------------------------

class TestOrientationCovarianceFlag:
    def test_covariance_set_to_minus_one(self):
        payload = [
            {
                'payload': [
                    {
                        'name': 'accelerometer',
                        'values': {'x': 0, 'y': 0, 'z': 9.8},
                    }
                ]
            }
        ]
        handler, pub = _make_handler(json.dumps(payload).encode())
        handler.do_POST()

        msg = pub.publish.call_args[0][0]
        assert msg.orientation_covariance[0] == -1.0


# -- CORS headers -----------------------------------------------------------

class TestCorsHeaders:
    def test_post_sends_cors_headers(self):
        """POST responses should include Access-Control-Allow-Origin."""
        handler, pub = _make_handler(json.dumps([]).encode())
        handler.do_POST()
        cors_calls = [
            call for call in handler.send_header.call_args_list
            if call[0][0] == 'Access-Control-Allow-Origin'
        ]
        assert len(cors_calls) == 1
        assert cors_calls[0][0][1] == '*'

    def test_options_returns_200(self):
        """OPTIONS preflight should return 200 with CORS headers."""
        handler, pub = _make_handler(b'')
        handler.do_OPTIONS()
        handler.send_response.assert_called_with(200)
