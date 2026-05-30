#!/usr/bin/env python3
"""Record & plot live IMU data from ROS 2 topics.

Usage
-----
    # Source workspace first:
    source ~/phone_imu_ws/install/setup.bash

    # Record for 60 seconds then plot:
    python3 plot_imu_live.py --duration 60

    # Record indefinitely, press Ctrl+C to stop and plot:
    python3 plot_imu_live.py

    # Just plot previously saved data:
    python3 plot_imu_live.py --load imu_recording.npz

The script subscribes to:
    /phone/imu/data_raw      — raw accelerometer + gyroscope
    /phone/imu/data_fused    — Madgwick-fused orientation + accel
    /phone/nav/velocity      — integrated velocity
    /phone/nav/position      — integrated displacement

After recording, it generates a multi-panel plot and saves:
    - A PNG screenshot (imu_plot_<timestamp>.png)
    - Raw data as NumPy archive (imu_recording.npz)
"""

import argparse
import signal
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped

# ── We import matplotlib lazily to avoid display issues ──────────
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend — saves to file
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec


class IMURecorder(Node):
    """ROS 2 node that records IMU data into numpy arrays."""

    def __init__(self, duration=None):
        super().__init__('imu_recorder')
        self.duration = duration
        self.start_time = time.time()
        self.running = True

        # Storage buffers
        self.raw_time = []
        self.raw_accel = []   # [ax, ay, az]
        self.raw_gyro = []    # [gx, gy, gz]

        self.fused_time = []
        self.fused_quat = []  # [w, x, y, z]
        self.fused_accel = [] # [ax, ay, az]

        self.vel_time = []
        self.vel_data = []    # [vx, vy, vz]

        self.pos_time = []
        self.pos_data = []    # [px, py, pz]

        # Subscriptions
        self.create_subscription(
            Imu, '/phone/imu/data_raw', self._raw_cb, 10)
        self.create_subscription(
            Imu, '/phone/imu/data_fused', self._fused_cb, 10)
        self.create_subscription(
            Vector3Stamped, '/phone/nav/velocity', self._vel_cb, 10)
        self.create_subscription(
            Vector3Stamped, '/phone/nav/position', self._pos_cb, 10)

        self.get_logger().info(
            f'Recording IMU data'
            + (f' for {duration}s...' if duration else ' (Ctrl+C to stop)...'))

    def _elapsed(self):
        return time.time() - self.start_time

    def _raw_cb(self, msg):
        if not self.running:
            return
        t = self._elapsed()
        self.raw_time.append(t)
        self.raw_accel.append([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])
        self.raw_gyro.append([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ])
        n = len(self.raw_time)
        if n % 5 == 0:
            self.get_logger().info(f'  Raw samples: {n}  (t={t:.1f}s)')

        if self.duration and t >= self.duration:
            self.running = False

    def _fused_cb(self, msg):
        if not self.running:
            return
        t = self._elapsed()
        self.fused_time.append(t)
        self.fused_quat.append([
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
        ])
        self.fused_accel.append([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])

    def _vel_cb(self, msg):
        if not self.running:
            return
        self.vel_time.append(self._elapsed())
        self.vel_data.append([msg.vector.x, msg.vector.y, msg.vector.z])

    def _pos_cb(self, msg):
        if not self.running:
            return
        self.pos_time.append(self._elapsed())
        self.pos_data.append([msg.vector.x, msg.vector.y, msg.vector.z])

    def save_data(self, filepath):
        """Save all recorded data to a .npz file."""
        np.savez(filepath,
                 raw_time=np.array(self.raw_time),
                 raw_accel=np.array(self.raw_accel) if self.raw_accel else np.empty((0, 3)),
                 raw_gyro=np.array(self.raw_gyro) if self.raw_gyro else np.empty((0, 3)),
                 fused_time=np.array(self.fused_time),
                 fused_quat=np.array(self.fused_quat) if self.fused_quat else np.empty((0, 4)),
                 fused_accel=np.array(self.fused_accel) if self.fused_accel else np.empty((0, 3)),
                 vel_time=np.array(self.vel_time),
                 vel_data=np.array(self.vel_data) if self.vel_data else np.empty((0, 3)),
                 pos_time=np.array(self.pos_time),
                 pos_data=np.array(self.pos_data) if self.pos_data else np.empty((0, 3)))
        print(f'Data saved → {filepath}')


def quat_to_euler(quats):
    """Convert Nx4 quaternion array [w,x,y,z] → Nx3 Euler [roll,pitch,yaw] in degrees."""
    w, x, y, z = quats[:, 0], quats[:, 1], quats[:, 2], quats[:, 3]
    roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    sinp = np.clip(2*(w*y - z*x), -1, 1)
    pitch = np.arcsin(sinp)
    yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return np.degrees(np.column_stack([roll, pitch, yaw]))


def generate_plot(data, output_path):
    """Create a comprehensive multi-panel IMU analysis plot."""

    # ── Style ────────────────────────────────────────────────────
    plt.style.use('dark_background')
    colors = {
        'x': '#6366f1',   # indigo
        'y': '#34d399',   # emerald
        'z': '#f87171',   # red
        'w': '#facc15',   # yellow
        'grid': '#1e293b',
        'text': '#94a3b8',
        'title': '#e2e8f0',
        'bg': '#0f172a',
        'card': '#1e293b',
    }

    fig = plt.figure(figsize=(18, 14), facecolor=colors['bg'])
    fig.suptitle('Phone IMU — Inertial Navigation Analysis',
                 fontsize=18, fontweight='bold', color=colors['title'], y=0.98)

    gs = GridSpec(3, 2, figure=fig, hspace=0.35, wspace=0.25,
                  left=0.06, right=0.97, top=0.93, bottom=0.05)

    axes_kwargs = dict(facecolor=colors['card'])

    # ── Panel 1: Raw Acceleration ────────────────────────────────
    raw_t = data['raw_time']
    raw_a = data['raw_accel']

    if len(raw_t) > 0 and raw_a.shape[0] > 0:
        ax1 = fig.add_subplot(gs[0, 0], **axes_kwargs)
        ax1.plot(raw_t, raw_a[:, 0], color=colors['x'], lw=1.2, label='X', alpha=0.9)
        ax1.plot(raw_t, raw_a[:, 1], color=colors['y'], lw=1.2, label='Y', alpha=0.9)
        ax1.plot(raw_t, raw_a[:, 2], color=colors['z'], lw=1.2, label='Z', alpha=0.9)
        ax1.set_title('Raw Accelerometer', color=colors['title'], fontweight='bold')
        ax1.set_ylabel('m/s²', color=colors['text'])
        ax1.set_xlabel('Time (s)', color=colors['text'])
        ax1.legend(loc='upper right', framealpha=0.3)
        ax1.grid(True, alpha=0.15, color=colors['grid'])
        ax1.tick_params(colors=colors['text'])

    # ── Panel 2: Raw Gyroscope ───────────────────────────────────
    raw_g = data['raw_gyro']

    if len(raw_t) > 0 and raw_g.shape[0] > 0:
        ax2 = fig.add_subplot(gs[0, 1], **axes_kwargs)
        ax2.plot(raw_t, raw_g[:, 0], color=colors['x'], lw=1.2, label='X', alpha=0.9)
        ax2.plot(raw_t, raw_g[:, 1], color=colors['y'], lw=1.2, label='Y', alpha=0.9)
        ax2.plot(raw_t, raw_g[:, 2], color=colors['z'], lw=1.2, label='Z', alpha=0.9)
        ax2.set_title('Raw Gyroscope', color=colors['title'], fontweight='bold')
        ax2.set_ylabel('rad/s', color=colors['text'])
        ax2.set_xlabel('Time (s)', color=colors['text'])
        ax2.legend(loc='upper right', framealpha=0.3)
        ax2.grid(True, alpha=0.15, color=colors['grid'])
        ax2.tick_params(colors=colors['text'])

    # ── Panel 3: Euler Angles (from Madgwick) ────────────────────
    fused_t = data['fused_time']
    fused_q = data['fused_quat']

    if len(fused_t) > 0 and fused_q.shape[0] > 0:
        euler = quat_to_euler(fused_q)
        ax3 = fig.add_subplot(gs[1, 0], **axes_kwargs)
        ax3.plot(fused_t, euler[:, 0], color=colors['x'], lw=1.2, label='Roll', alpha=0.9)
        ax3.plot(fused_t, euler[:, 1], color=colors['y'], lw=1.2, label='Pitch', alpha=0.9)
        ax3.plot(fused_t, euler[:, 2], color=colors['z'], lw=1.2, label='Yaw', alpha=0.9)
        ax3.set_title('Orientation (Madgwick Fusion)', color=colors['title'], fontweight='bold')
        ax3.set_ylabel('Degrees', color=colors['text'])
        ax3.set_xlabel('Time (s)', color=colors['text'])
        ax3.legend(loc='upper right', framealpha=0.3)
        ax3.grid(True, alpha=0.15, color=colors['grid'])
        ax3.tick_params(colors=colors['text'])

    # ── Panel 4: Velocity ────────────────────────────────────────
    vel_t = data['vel_time']
    vel_d = data['vel_data']

    if len(vel_t) > 0 and vel_d.shape[0] > 0:
        ax4 = fig.add_subplot(gs[1, 1], **axes_kwargs)
        ax4.plot(vel_t, vel_d[:, 0], color=colors['x'], lw=1.2, label='Vx', alpha=0.9)
        ax4.plot(vel_t, vel_d[:, 1], color=colors['y'], lw=1.2, label='Vy', alpha=0.9)
        ax4.plot(vel_t, vel_d[:, 2], color=colors['z'], lw=1.2, label='Vz', alpha=0.9)
        ax4.set_title('Estimated Velocity', color=colors['title'], fontweight='bold')
        ax4.set_ylabel('m/s', color=colors['text'])
        ax4.set_xlabel('Time (s)', color=colors['text'])
        ax4.legend(loc='upper right', framealpha=0.3)
        ax4.grid(True, alpha=0.15, color=colors['grid'])
        ax4.tick_params(colors=colors['text'])

    # ── Panel 5: Displacement ────────────────────────────────────
    pos_t = data['pos_time']
    pos_d = data['pos_data']

    if len(pos_t) > 0 and pos_d.shape[0] > 0:
        ax5 = fig.add_subplot(gs[2, 0], **axes_kwargs)
        ax5.plot(pos_t, pos_d[:, 0], color=colors['x'], lw=1.2, label='X', alpha=0.9)
        ax5.plot(pos_t, pos_d[:, 1], color=colors['y'], lw=1.2, label='Y', alpha=0.9)
        ax5.plot(pos_t, pos_d[:, 2], color=colors['z'], lw=1.2, label='Z', alpha=0.9)
        total = np.sqrt(np.sum(pos_d**2, axis=1))
        ax5.plot(pos_t, total, color=colors['w'], lw=1.5, label='Total', alpha=0.8, ls='--')
        ax5.set_title('Displacement (Dead Reckoning)', color=colors['title'], fontweight='bold')
        ax5.set_ylabel('meters', color=colors['text'])
        ax5.set_xlabel('Time (s)', color=colors['text'])
        ax5.legend(loc='upper right', framealpha=0.3)
        ax5.grid(True, alpha=0.15, color=colors['grid'])
        ax5.tick_params(colors=colors['text'])

    # ── Panel 6: FFT of raw accel ────────────────────────────────
    if len(raw_t) > 2 and raw_a.shape[0] > 2:
        dt_avg = np.mean(np.diff(raw_t)) if len(raw_t) > 1 else 1.0
        fs = 1.0 / dt_avg if dt_avg > 0 else 1.0
        accel_mag = np.sqrt(np.sum(raw_a**2, axis=1))
        N = len(accel_mag)
        freqs = np.fft.rfftfreq(N, d=1.0/fs)
        fft_vals = np.abs(np.fft.rfft(accel_mag - np.mean(accel_mag)))
        psd_db = 20 * np.log10(fft_vals + 1e-12)

        ax6 = fig.add_subplot(gs[2, 1], **axes_kwargs)
        ax6.fill_between(freqs, psd_db, alpha=0.3, color=colors['x'])
        ax6.plot(freqs, psd_db, color=colors['x'], lw=1.2, alpha=0.9)
        if len(psd_db) > 0:
            peak_idx = np.argmax(psd_db)
            ax6.plot(freqs[peak_idx], psd_db[peak_idx], 'o',
                     color=colors['z'], ms=6, label=f'Peak: {freqs[peak_idx]:.2f} Hz')
        ax6.set_title('FFT Spectrum (Accel Magnitude)', color=colors['title'], fontweight='bold')
        ax6.set_ylabel('PSD (dB)', color=colors['text'])
        ax6.set_xlabel('Frequency (Hz)', color=colors['text'])
        ax6.legend(loc='upper right', framealpha=0.3)
        ax6.grid(True, alpha=0.15, color=colors['grid'])
        ax6.tick_params(colors=colors['text'])

    # ── Info text ────────────────────────────────────────────────
    n_raw = len(raw_t)
    dur = raw_t[-1] - raw_t[0] if n_raw > 1 else 0
    rate = n_raw / dur if dur > 0 else 0
    info = (f'Samples: {n_raw}  |  Duration: {dur:.1f}s  |  '
            f'Rate: {rate:.1f} Hz  |  Recorded: {time.strftime("%Y-%m-%d %H:%M")}')
    fig.text(0.5, 0.01, info, ha='center', fontsize=10, color=colors['text'])

    plt.savefig(str(output_path), dpi=150, facecolor=colors['bg'])
    print(f'\nPlot saved → {output_path}')
    print(f'  Samples: {n_raw}, Duration: {dur:.1f}s, Rate: {rate:.1f} Hz')

    # Also try to show interactively
    try:
        matplotlib.use('TkAgg')
        plt.show()
    except Exception:
        print('  (Non-interactive display — see saved PNG)')


def main():
    parser = argparse.ArgumentParser(
        description='Record & plot live IMU data from ROS 2 topics')
    parser.add_argument('--duration', type=int, default=None,
                        help='Recording duration in seconds (default: until Ctrl+C)')
    parser.add_argument('--load', type=str, default=None,
                        help='Load previously saved .npz file and plot it')
    parser.add_argument('--output', type=str, default=None,
                        help='Output PNG file path')
    args = parser.parse_args()

    output_dir = Path(__file__).parent.parent / 'plots'
    output_dir.mkdir(exist_ok=True)

    timestamp = time.strftime('%Y%m%d_%H%M%S')
    png_path = Path(args.output) if args.output else output_dir / f'imu_plot_{timestamp}.png'
    npz_path = output_dir / f'imu_recording_{timestamp}.npz'

    # ── Load mode ────────────────────────────────────────────────
    if args.load:
        print(f'Loading data from {args.load}...')
        data = dict(np.load(args.load, allow_pickle=True))
        generate_plot(data, png_path)
        return

    # ── Record mode ──────────────────────────────────────────────
    rclpy.init()
    recorder = IMURecorder(duration=args.duration)

    # Handle Ctrl+C gracefully
    def sigint_handler(sig, frame):
        print('\n\nStopping recording...')
        recorder.running = False

    signal.signal(signal.SIGINT, sigint_handler)

    print('Waiting for IMU data on /phone/imu/data_raw ...')
    print('(Make sure the Sensor Logger app is streaming!)\n')

    try:
        while rclpy.ok() and recorder.running:
            rclpy.spin_once(recorder, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass

    print(f'\nRecording complete. {len(recorder.raw_time)} samples collected.')

    if len(recorder.raw_time) == 0:
        print('No data received! Check:')
        print('  1. ros2 launch phone_imu_bridge imu_bridge.launch.py  (running?)')
        print('  2. Sensor Logger HTTP Push URL = http://10.157.16.75:5555/data')
        print('  3. ros2 topic echo /phone/imu/data_raw --once')
        rclpy.shutdown()
        return

    # Save raw data
    recorder.save_data(str(npz_path))

    # Generate plot
    data = dict(np.load(str(npz_path)))
    generate_plot(data, png_path)

    recorder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
