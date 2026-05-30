"""Madgwick AHRS orientation filter node.

Subscribes to raw 6-axis IMU data on ``/phone/imu/data_raw``, applies the
Madgwick gradient-descent sensor fusion algorithm, and re-publishes the
result (with a valid quaternion orientation) on ``/phone/imu/data_fused``.

Reference
---------
S. O. H. Madgwick, "An efficient orientation filter for inertial and
inertial/magnetic sensor arrays," Report x-io and University of Bristol,
2010.  https://courses.cs.washington.edu/courses/cse466/14au/labs/l4/
madgwick_internal_report.pdf
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class MadgwickFilterNode(Node):
    def __init__(self):
        super().__init__('madgwick_filter_node')

        self.declare_parameter('beta', 0.033)
        self.declare_parameter('frequency', 100.0)

        self.beta = self.get_parameter('beta').value
        self.dt   = 1.0 / self.get_parameter('frequency').value

        # Quaternion state [w, x, y, z]
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.calibrated   = False
        self.calib_buf    = []
        self.CALIB_SAMPLES = 5   # 5 samples (~5s at 1 Hz)

        self.sub = self.create_subscription(
            Imu, '/phone/imu/data_raw', self.imu_cb, 10)
        self.pub = self.create_publisher(
            Imu, '/phone/imu/data_fused', 10)

        self.get_logger().info('Madgwick filter node ready')

    def imu_cb(self, msg):
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        # Startup calibration: compute initial quaternion from gravity
        if not self.calibrated:
            self.calib_buf.append([ax, ay, az])
            if len(self.calib_buf) >= self.CALIB_SAMPLES:
                avg = np.mean(self.calib_buf, axis=0)
                self.q = self._init_quat_from_accel(avg)
                self.calibrated = True
                self.get_logger().info('Calibration done — filter running')
            return

        self.q = self._madgwick_update(self.q, gx, gy, gz, ax, ay, az)

        out = Imu()
        out.header = msg.header
        out.orientation.w = float(self.q[0])
        out.orientation.x = float(self.q[1])
        out.orientation.y = float(self.q[2])
        out.orientation.z = float(self.q[3])
        out.linear_acceleration = msg.linear_acceleration
        out.angular_velocity    = msg.angular_velocity
        out.orientation_covariance[0] = 0.01
        self.pub.publish(out)

    def _init_quat_from_accel(self, a):
        a = a / np.linalg.norm(a)
        ax, ay, az = a
        pitch = np.arcsin(-ax)
        roll  = np.arctan2(ay, az)
        cp, sp = np.cos(pitch/2), np.sin(pitch/2)
        cr, sr = np.cos(roll/2),  np.sin(roll/2)
        return np.array([cp*cr, cp*sr, sp*cr, -sp*sr])

    def _madgwick_update(self, q, gx, gy, gz, ax, ay, az):
        q0, q1, q2, q3 = q

        # Normalise accelerometer
        norm = np.sqrt(ax*ax + ay*ay + az*az)
        if norm < 1e-10:
            return q
        ax /= norm; ay /= norm; az /= norm

        # Gradient descent step
        f1 = 2*(q1*q3 - q0*q2) - ax
        f2 = 2*(q0*q1 + q2*q3) - ay
        f3 = 2*(0.5 - q1*q1 - q2*q2) - az

        j11 = -2*q2; j12 =  2*q3; j13 = -2*q0; j14 =  2*q1
        j21 =  2*q1; j22 =  2*q0; j23 =  2*q3; j24 =  2*q2
        j31 =  0.0;  j32 = -4*q1; j33 = -4*q2; j34 =  0.0

        grad0 = j11*f1 + j21*f2 + j31*f3
        grad1 = j12*f1 + j22*f2 + j32*f3
        grad2 = j13*f1 + j23*f2 + j33*f3
        grad3 = j14*f1 + j24*f2 + j34*f3

        gnorm = np.sqrt(grad0**2 + grad1**2 + grad2**2 + grad3**2)
        if gnorm > 1e-10:
            grad0 /= gnorm; grad1 /= gnorm
            grad2 /= gnorm; grad3 /= gnorm

        # Quaternion rate from gyroscope
        qdot0 = 0.5*(-q1*gx - q2*gy - q3*gz) - self.beta*grad0
        qdot1 = 0.5*( q0*gx + q2*gz - q3*gy) - self.beta*grad1
        qdot2 = 0.5*( q0*gy - q1*gz + q3*gx) - self.beta*grad2
        qdot3 = 0.5*( q0*gz + q1*gy - q2*gx) - self.beta*grad3

        q0 += qdot0 * self.dt
        q1 += qdot1 * self.dt
        q2 += qdot2 * self.dt
        q3 += qdot3 * self.dt

        norm = np.sqrt(q0**2 + q1**2 + q2**2 + q3**2)
        return np.array([q0, q1, q2, q3]) / norm


def main(args=None):
    rclpy.init(args=args)
    node = MadgwickFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
