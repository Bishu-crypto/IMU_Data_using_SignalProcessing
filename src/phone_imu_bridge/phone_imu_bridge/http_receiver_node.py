"""HTTP receiver node for phone IMU data.

Listens for HTTP POST requests containing JSON-encoded IMU sensor data
from the companion phone app (or any compatible sender such as Android
Sensor Logger).  Each valid reading is re-published as a standard
``sensor_msgs/Imu`` message on ``/phone/imu/data_raw``.

JSON payload format (array of batches)::

    [
      {
        "payload": [
          {"name": "accelerometer", "values": {"x": 0.1, "y": 0.2, "z": 9.8}},
          {"name": "gyroscope",     "values": {"x": 0.01, "y": -0.02, "z": 0.03}}
        ]
      }
    ]

Alternate flat format (``ax/ay/az/gx/gy/gz`` keys) is also supported.
"""

import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class HTTPHandler(BaseHTTPRequestHandler):
    """Minimal HTTP handler that converts POST JSON bodies to ROS Imu msgs."""

    publisher = None
    ros_node = None

    # ── CORS helpers ──────────────────────────────────────────────────

    _CORS_HEADERS = {
        'Access-Control-Allow-Origin': '*',
        'Access-Control-Allow-Methods': 'POST, OPTIONS',
        'Access-Control-Allow-Headers': 'Content-Type',
    }

    def _send_cors_headers(self):
        for key, value in self._CORS_HEADERS.items():
            self.send_header(key, value)

    # ── HTTP verbs ────────────────────────────────────────────────────

    def do_OPTIONS(self):  # noqa: N802 – required by BaseHTTPRequestHandler
        """Handle CORS preflight requests from the phone PWA."""
        self.send_response(200)
        self._send_cors_headers()
        self.end_headers()

    def do_POST(self):  # noqa: N802
        """Parse incoming JSON IMU data and publish to ROS topic."""
        length = int(self.headers.get('Content-Length', 0))
        raw = self.rfile.read(length)
        self.send_response(200)
        self._send_cors_headers()
        self.end_headers()

        try:
            payload = json.loads(raw.decode())
            # Sensor Logger / phone app sends a JSON array of batches
            if isinstance(payload, list):
                entries = payload
            else:
                entries = [payload]

            for entry in entries:
                # Each entry has a 'payload' list of sensor readings
                sensors = entry.get('payload', [entry])
                for s in sensors:
                    name = s.get('name', '')
                    vals = s.get('values', s)

                    if name in ('accelerometer', 'gyroscope', ''):
                        msg = Imu()
                        msg.header.frame_id = 'phone_imu_link'
                        msg.header.stamp = (
                            HTTPHandler.ros_node.get_clock()
                            .now().to_msg()
                        )

                        if name == 'accelerometer' or 'ax' in vals:
                            msg.linear_acceleration.x = float(
                                vals.get('x', vals.get('ax', 0)))
                            msg.linear_acceleration.y = float(
                                vals.get('y', vals.get('ay', 0)))
                            msg.linear_acceleration.z = float(
                                vals.get('z', vals.get('az', 0)))

                        if name == 'gyroscope' or 'gx' in vals:
                            msg.angular_velocity.x = float(
                                vals.get('x', vals.get('gx', 0)))
                            msg.angular_velocity.y = float(
                                vals.get('y', vals.get('gy', 0)))
                            msg.angular_velocity.z = float(
                                vals.get('z', vals.get('gz', 0)))

                        msg.orientation_covariance[0] = -1.0
                        HTTPHandler.publisher.publish(msg)

        except Exception as e:
            HTTPHandler.ros_node.get_logger().warn(
                f'Parse error: {e}  raw={raw[:200]}')

    def log_message(self, format, *args):
        """Suppress default HTTP access logging."""
        pass


class HTTPReceiverNode(Node):
    """ROS 2 node that runs an HTTP server to receive phone IMU data.

    Parameters
    ----------
    http_port : int, default 5555
        TCP port the HTTP server listens on.
    frame_id : str, default 'phone_imu_link'
        TF frame ID stamped on every outgoing Imu message.
    """

    def __init__(self):
        super().__init__('http_receiver_node')

        self.declare_parameter('http_port', 5555)
        self.declare_parameter('frame_id', 'phone_imu_link')

        port = self.get_parameter('http_port').value

        self.pub = self.create_publisher(Imu, '/phone/imu/data_raw', 10)

        HTTPHandler.publisher = self.pub
        HTTPHandler.ros_node = self

        server = HTTPServer(('0.0.0.0', port), HTTPHandler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()

        self.get_logger().info(f'HTTP receiver listening on port {port}')


def main(args=None):
    rclpy.init(args=args)
    node = HTTPReceiverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
