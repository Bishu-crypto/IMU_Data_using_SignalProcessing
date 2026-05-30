"""WebSocket bridge node — streams ROS 2 topics to the web dashboard.

Subscribes to fused IMU, navigation state, and PSD topics, then broadcasts
JSON-encoded messages to all connected WebSocket clients.  The web dashboard
(``webapp/index.html``) connects to this node for real-time visualisation.

Protocol
--------
Each WebSocket frame is a JSON object with a ``type`` field::

    {"type": "imu",  "qw": …, "qx": …, "qy": …, "qz": …,
     "ax": …, "ay": …, "az": …, "gx": …, "gy": …, "gz": …}

    {"type": "nav",  "vx": …, "vy": …, "vz": …,
                     "px": …, "py": …, "pz": …}

    {"type": "psd",  "freqs": […], "psd_db": […]}
"""

import asyncio
import json
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import String


class WSBridgeNode(Node):
    """ROS 2 node that bridges topics to WebSocket clients.

    Parameters
    ----------
    ws_port : int, default 8765
        TCP port for the WebSocket server.
    broadcast_rate : float, default 30.0
        Maximum broadcast rate to clients (Hz).
    """

    def __init__(self):
        super().__init__('ws_bridge_node')

        self.declare_parameter('ws_port', 8765)
        self.declare_parameter('broadcast_rate', 30.0)

        self.ws_port = self.get_parameter('ws_port').value
        self.broadcast_rate = self.get_parameter('broadcast_rate').value

        # Latest data (thread-safe via GIL for simple assignments)
        self._imu_data = None
        self._vel_data = None
        self._pos_data = None
        self._psd_data = None

        # Connected clients
        self._clients = set()

        # ── Subscriptions ─────────────────────────────────────────────
        self.create_subscription(
            Imu, '/phone/imu/data_fused', self._imu_cb, 10)
        self.create_subscription(
            Vector3Stamped, '/phone/nav/velocity', self._vel_cb, 10)
        self.create_subscription(
            Vector3Stamped, '/phone/nav/position', self._pos_cb, 10)
        self.create_subscription(
            String, '/phone/imu/psd', self._psd_cb, 10)

        # ── Start WebSocket server in background thread ───────────────
        self._ws_thread = threading.Thread(
            target=self._run_ws_server, daemon=True)
        self._ws_thread.start()

        self.get_logger().info(
            f'WebSocket bridge ready on ws://0.0.0.0:{self.ws_port}')

    # ── ROS callbacks ─────────────────────────────────────────────────

    def _imu_cb(self, msg):
        self._imu_data = {
            'type': 'imu',
            'qw': msg.orientation.w,
            'qx': msg.orientation.x,
            'qy': msg.orientation.y,
            'qz': msg.orientation.z,
            'ax': msg.linear_acceleration.x,
            'ay': msg.linear_acceleration.y,
            'az': msg.linear_acceleration.z,
            'gx': msg.angular_velocity.x,
            'gy': msg.angular_velocity.y,
            'gz': msg.angular_velocity.z,
        }

    def _vel_cb(self, msg):
        self._vel_data = {
            'type': 'velocity',
            'vx': msg.vector.x,
            'vy': msg.vector.y,
            'vz': msg.vector.z,
        }

    def _pos_cb(self, msg):
        self._pos_data = {
            'type': 'position',
            'px': msg.vector.x,
            'py': msg.vector.y,
            'pz': msg.vector.z,
        }

    def _psd_cb(self, msg):
        try:
            self._psd_data = json.loads(msg.data)
            self._psd_data['type'] = 'psd'
        except json.JSONDecodeError:
            pass

    # ── WebSocket server ──────────────────────────────────────────────

    def _run_ws_server(self):
        """Run asyncio event loop for the WebSocket server."""
        try:
            import websockets
            import websockets.server
        except ImportError:
            self.get_logger().warn(
                'websockets package not installed — WS bridge disabled. '
                'Install with: pip install websockets')
            return

        async def handler(websocket):
            self._clients.add(websocket)
            self.get_logger().info(
                f'WS client connected ({len(self._clients)} total)')
            try:
                async for _ in websocket:
                    pass  # We only broadcast; ignore incoming
            finally:
                self._clients.discard(websocket)
                self.get_logger().info(
                    f'WS client disconnected ({len(self._clients)} total)')

        async def broadcast_loop():
            """Periodically send latest data to all clients."""
            interval = 1.0 / self.broadcast_rate
            while True:
                if self._clients:
                    messages = []
                    if self._imu_data:
                        messages.append(json.dumps(self._imu_data))
                    if self._vel_data:
                        messages.append(json.dumps(self._vel_data))
                    if self._pos_data:
                        messages.append(json.dumps(self._pos_data))
                    if self._psd_data:
                        messages.append(json.dumps(self._psd_data))

                    for msg_str in messages:
                        dead = set()
                        for client in self._clients.copy():
                            try:
                                await client.send(msg_str)
                            except Exception:
                                dead.add(client)
                        self._clients -= dead

                await asyncio.sleep(interval)

        async def main_server():
            async with websockets.server.serve(
                handler, '0.0.0.0', self.ws_port
            ):
                await broadcast_loop()

        asyncio.run(main_server())


def main(args=None):
    rclpy.init(args=args)
    node = WSBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
