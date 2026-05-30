"""Inertial navigation node — dead reckoning via double integration.

Subscribes to the Madgwick-fused IMU data on ``/phone/imu/data_fused``,
removes gravity using the fused quaternion orientation, applies a 2nd-order
Butterworth low-pass filter to suppress MEMS accelerometer noise, and
performs trapezoidal (Tustin) numerical integration to estimate velocity
and displacement.

DSP Pipeline
------------
::

    ┌─────────┐    ┌──────────────┐    ┌────────────┐    ┌──────────┐
    │ Fused   │───▶│ Gravity      │───▶│ Butterworth│───▶│Trapezoidal│
    │ IMU Msg │    │ Removal      │    │ LPF (IIR)  │    │Integration│
    └─────────┘    │ (Quaternion) │    │ 2nd order  │    │  ×2       │
                   └──────────────┘    └────────────┘    └──────────┘
                                                              │
                                                              ▼
                                                     ┌──────────────┐
                                                     │  ZUPT Check  │
                                                     │  (zero-vel   │
                                                     │   update)    │
                                                     └──────────────┘
                                                              │
                                                              ▼
                                                     Position, Velocity,
                                                     Orientation

Mathematical Background
-----------------------
**Gravity Removal** — The accelerometer measures *specific force*:
    a_meas = a_linear + R^T · g

where R is the rotation matrix from world to body frame (derived from
the Madgwick quaternion), and g = [0, 0, 9.81]^T.  We compute:
    a_linear = a_meas - R^T · g

**Trapezoidal Integration** (more accurate than Euler):
    v[n] = v[n-1] + (dt/2) · (a[n] + a[n-1])
    p[n] = p[n-1] + (dt/2) · (v[n] + v[n-1])

**ZUPT (Zero-Velocity Update)** — When the device is stationary
(||a_linear|| < threshold for N consecutive samples), velocity is
reset to zero.  This bounds the unbounded drift inherent in open-loop
integration.

Reference: Woodman, "An introduction to inertial navigation", 2007.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped

from phone_imu_bridge.dsp_filters import ButterworthLPF


class InertialNavNode(Node):
    """Estimate velocity and displacement from fused IMU data.

    Parameters
    ----------
    butterworth_cutoff : float, default 5.0
        Low-pass filter cutoff frequency (Hz).
    butterworth_order : int, default 2
        Filter order (only 2 supported in current implementation).
    zupt_threshold : float, default 0.15
        Acceleration magnitude threshold for ZUPT (m/s²).
    zupt_window : int, default 10
        Number of consecutive sub-threshold samples to trigger ZUPT.
    frequency : float, default 100.0
        Expected IMU sampling rate (Hz).
    gravity : float, default 9.81
        Local gravitational acceleration (m/s²).
    """

    def __init__(self):
        super().__init__('inertial_nav_node')

        # ── Declare parameters ────────────────────────────────────────
        self.declare_parameter('butterworth_cutoff', 5.0)
        self.declare_parameter('zupt_threshold', 0.15)
        self.declare_parameter('zupt_window', 10)
        self.declare_parameter('frequency', 100.0)
        self.declare_parameter('gravity', 9.81)

        self.cutoff = self.get_parameter('butterworth_cutoff').value
        self.zupt_thresh = self.get_parameter('zupt_threshold').value
        self.zupt_win = self.get_parameter('zupt_window').value
        self.fs = self.get_parameter('frequency').value
        self.gravity = self.get_parameter('gravity').value
        self.dt = 1.0 / self.fs

        # ── DSP filters (one per axis) ────────────────────────────────
        self.lpf_x = ButterworthLPF(cutoff_hz=self.cutoff, sample_rate=self.fs)
        self.lpf_y = ButterworthLPF(cutoff_hz=self.cutoff, sample_rate=self.fs)
        self.lpf_z = ButterworthLPF(cutoff_hz=self.cutoff, sample_rate=self.fs)

        # ── Navigation state ──────────────────────────────────────────
        self.velocity = np.zeros(3)      # m/s  [x, y, z] world frame
        self.position = np.zeros(3)      # m    [x, y, z] world frame
        self.prev_accel = np.zeros(3)    # for trapezoidal integration
        self.prev_velocity = np.zeros(3) # for trapezoidal integration
        self.first_sample = True

        # ZUPT state
        self.zupt_counter = 0

        # ── ROS 2 pub/sub ─────────────────────────────────────────────
        self.sub = self.create_subscription(
            Imu, '/phone/imu/data_fused', self.imu_cb, 10)

        self.pub_vel = self.create_publisher(
            Vector3Stamped, '/phone/nav/velocity', 10)
        self.pub_pos = self.create_publisher(
            Vector3Stamped, '/phone/nav/position', 10)

        self.get_logger().info(
            f'Inertial nav ready — LPF cutoff={self.cutoff} Hz, '
            f'ZUPT thresh={self.zupt_thresh} m/s²')

    # ── Quaternion → Rotation Matrix ──────────────────────────────────

    @staticmethod
    def quat_to_rotation_matrix(q):
        """Convert quaternion [w, x, y, z] to a 3×3 rotation matrix.

        The rotation matrix R transforms vectors from the world frame
        to the body frame:  v_body = R · v_world.

        Derivation: R = I + 2w·[e]× + 2·[e]×²
        where [e]× is the skew-symmetric matrix of the vector part.
        """
        w, x, y, z = q

        r00 = 1 - 2*(y*y + z*z)
        r01 = 2*(x*y - w*z)
        r02 = 2*(x*z + w*y)

        r10 = 2*(x*y + w*z)
        r11 = 1 - 2*(x*x + z*z)
        r12 = 2*(y*z - w*x)

        r20 = 2*(x*z - w*y)
        r21 = 2*(y*z + w*x)
        r22 = 1 - 2*(x*x + y*y)

        return np.array([
            [r00, r01, r02],
            [r10, r11, r12],
            [r20, r21, r22]
        ])

    # ── Gravity removal ───────────────────────────────────────────────

    def remove_gravity(self, accel_body, quaternion):
        """Remove gravitational acceleration from body-frame measurement.

        Parameters
        ----------
        accel_body : ndarray, shape (3,)
            Measured acceleration in body (sensor) frame.
        quaternion : ndarray, shape (4,)
            Orientation quaternion [w, x, y, z] from Madgwick filter.

        Returns
        -------
        ndarray, shape (3,)
            Linear acceleration in the world frame (gravity-free).

        Notes
        -----
        The accelerometer measures:  a_meas = a_linear + R^T · g
        where R rotates world→body.  So:
            a_linear_body = a_meas - R^T · g
            a_linear_world = R · a_linear_body

        We first rotate gravity into body frame, subtract it, then
        rotate the result back to world frame.
        """
        R = self.quat_to_rotation_matrix(quaternion)
        g_world = np.array([0.0, 0.0, self.gravity])

        # Gravity in body frame: R^T · g  (R is world→body, so R^T·g)
        g_body = R.T @ g_world

        # Remove gravity in body frame
        a_linear_body = accel_body - g_body

        # Transform to world frame
        a_linear_world = R @ a_linear_body

        return a_linear_world

    # ── IMU callback ──────────────────────────────────────────────────

    def imu_cb(self, msg):
        """Process fused IMU message: filter → integrate → publish."""
        # Extract quaternion (Madgwick output)
        q = np.array([
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
        ])

        # Skip if orientation is not yet valid
        qnorm = np.linalg.norm(q)
        if qnorm < 0.5:
            return
        q = q / qnorm  # re-normalise for safety

        # Extract body-frame accelerometer data
        accel_body = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        # ── Step 1: Remove gravity ────────────────────────────────
        a_world = self.remove_gravity(accel_body, q)

        # ── Step 2: Butterworth low-pass filter ───────────────────
        a_filtered = np.array([
            self.lpf_x.filter(a_world[0]),
            self.lpf_y.filter(a_world[1]),
            self.lpf_z.filter(a_world[2]),
        ])

        # ── Step 3: ZUPT check ───────────────────────────────────
        accel_mag = np.linalg.norm(a_filtered)
        if accel_mag < self.zupt_thresh:
            self.zupt_counter += 1
        else:
            self.zupt_counter = 0

        zupt_active = self.zupt_counter >= self.zupt_win

        # ── Step 4: Trapezoidal integration ───────────────────────
        if self.first_sample:
            self.prev_accel = a_filtered.copy()
            self.first_sample = False
            return

        if zupt_active:
            # Zero-velocity update: reset velocity to bound drift
            self.velocity = np.zeros(3)
            self.lpf_x.reset()
            self.lpf_y.reset()
            self.lpf_z.reset()
        else:
            # Velocity: v[n] = v[n-1] + (dt/2)·(a[n] + a[n-1])
            new_velocity = self.velocity + \
                0.5 * self.dt * (a_filtered + self.prev_accel)

            # Position: p[n] = p[n-1] + (dt/2)·(v[n] + v[n-1])
            self.position += 0.5 * self.dt * (new_velocity + self.velocity)
            self.velocity = new_velocity

        self.prev_accel = a_filtered.copy()

        # ── Step 5: Publish ───────────────────────────────────────
        stamp = msg.header.stamp

        vel_msg = Vector3Stamped()
        vel_msg.header.stamp = stamp
        vel_msg.header.frame_id = 'world'
        vel_msg.vector.x = float(self.velocity[0])
        vel_msg.vector.y = float(self.velocity[1])
        vel_msg.vector.z = float(self.velocity[2])
        self.pub_vel.publish(vel_msg)

        pos_msg = Vector3Stamped()
        pos_msg.header.stamp = stamp
        pos_msg.header.frame_id = 'world'
        pos_msg.vector.x = float(self.position[0])
        pos_msg.vector.y = float(self.position[1])
        pos_msg.vector.z = float(self.position[2])
        self.pub_pos.publish(pos_msg)


def main(args=None):
    rclpy.init(args=args)
    node = InertialNavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
