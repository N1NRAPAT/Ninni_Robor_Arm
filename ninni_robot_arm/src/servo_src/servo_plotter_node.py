#!/usr/bin/env python3
"""Live matplotlib plotter for servo voltage and current via /servo_telemetry topic.
Two windows: 5 subplots each for voltage and current, one per servo."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
from collections import defaultdict
import time


class ServoPlotterNode(Node):
    def __init__(self):
        super().__init__('servo_plotter_node')

        self.declare_parameter('max_samples', 120)
        self.max_samples = self.get_parameter('max_samples').value

        self.timestamps = []
        self.voltage_data = defaultdict(list)
        self.current_data = defaultdict(list)
        self.start_time = time.time()

        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        self.colors = {
            'joint1': '#e74c3c',
            'joint2': '#3498db',
            'joint3': '#2ecc71',
            'joint4': '#e67e22',
            'joint5': '#9b59b6',
        }

        plt.ion()

        # Window 1: Voltage (5 subplots)
        self.fig_v, self.axes_v = plt.subplots(5, 1, figsize=(10, 8), sharex=True)
        self.fig_v.suptitle('Servo Voltage Monitor', fontsize=14, fontweight='bold')
        self.fig_v.canvas.manager.set_window_title('Voltage Monitor')
        for i, name in enumerate(self.joint_names):
            self.axes_v[i].set_ylabel(f'{name}\n(V)', fontsize=9)
            self.axes_v[i].grid(True, alpha=0.3)
        self.axes_v[-1].set_xlabel('Time (s)')

        # Window 2: Current (5 subplots)
        self.fig_c, self.axes_c = plt.subplots(5, 1, figsize=(10, 8), sharex=True)
        self.fig_c.suptitle('Servo Current Monitor', fontsize=14, fontweight='bold')
        self.fig_c.canvas.manager.set_window_title('Current Monitor')
        for i, name in enumerate(self.joint_names):
            self.axes_c[i].set_ylabel(f'{name}\n(mA)', fontsize=9)
            self.axes_c[i].grid(True, alpha=0.3)
        self.axes_c[-1].set_xlabel('Time (s)')

        self.subscription = self.create_subscription(
            JointState,
            '/servo_telemetry',
            self.telemetry_callback,
            10
        )

        self.plot_timer = self.create_timer(0.1, self.update_plot)

        self.get_logger().info('Servo plotter ready - subscribing to /servo_telemetry')

    def telemetry_callback(self, msg):
        t = time.time() - self.start_time
        self.timestamps.append(t)

        for i, name in enumerate(msg.name):
            voltage = msg.position[i] if i < len(msg.position) else 0.0
            current = msg.velocity[i] if i < len(msg.velocity) else 0.0
            self.voltage_data[name].append(voltage)
            self.current_data[name].append(current)

        # trim to max samples
        if len(self.timestamps) > self.max_samples:
            self.timestamps.pop(0)
            for name in list(self.voltage_data.keys()):
                if len(self.voltage_data[name]) > self.max_samples:
                    self.voltage_data[name].pop(0)
                if len(self.current_data[name]) > self.max_samples:
                    self.current_data[name].pop(0)

    def update_plot(self):
        if not self.timestamps:
            return

        for i, name in enumerate(self.joint_names):
            color = self.colors[name]

            # Voltage subplot
            self.axes_v[i].clear()
            self.axes_v[i].set_ylabel(f'{name}\n(V)', fontsize=9)
            self.axes_v[i].grid(True, alpha=0.3)
            if name in self.voltage_data and self.voltage_data[name]:
                data_len = len(self.voltage_data[name])
                ts = self.timestamps[-data_len:]
                self.axes_v[i].plot(ts, self.voltage_data[name], color=color, linewidth=1.5)
                latest_v = self.voltage_data[name][-1]
                self.axes_v[i].set_title(f'{name}: {latest_v:.1f}V', fontsize=9, loc='right')

            # Current subplot
            self.axes_c[i].clear()
            self.axes_c[i].set_ylabel(f'{name}\n(mA)', fontsize=9)
            self.axes_c[i].grid(True, alpha=0.3)
            if name in self.current_data and self.current_data[name]:
                data_len = len(self.current_data[name])
                ts = self.timestamps[-data_len:]
                self.axes_c[i].plot(ts, self.current_data[name], color=color, linewidth=1.5)
                latest_c = self.current_data[name][-1]
                self.axes_c[i].set_title(f'{name}: {latest_c:.0f}mA', fontsize=9, loc='right')

        self.axes_v[-1].set_xlabel('Time (s)')
        self.axes_c[-1].set_xlabel('Time (s)')

        self.fig_v.tight_layout()
        self.fig_c.tight_layout()
        self.fig_v.canvas.draw_idle()
        self.fig_v.canvas.flush_events()
        self.fig_c.canvas.draw_idle()
        self.fig_c.canvas.flush_events()

    def destroy_node(self):
        plt.ioff()
        plt.close('all')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoPlotterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
