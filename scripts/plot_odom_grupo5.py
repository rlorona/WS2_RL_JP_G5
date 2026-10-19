#!/usr/bin/env python3
"""Grafica en vivo la trayectoria real del robot.

Lee /pose_ground_truth, la pose fisica del modelo publicada por el nodo de
trayectoria a partir de Gazebo (sin error de odometria). Guarda una imagen
PNG en ~/Desktop/WS2_grupo5_trayectoria.png y muestra la ventana de
matplotlib mientras el robot dibuja el numero 5.
"""

import math
import os
import sys
import time

import matplotlib.pyplot as plt
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

SAVE_PATH = os.path.expanduser('~/Desktop/WS2_grupo5_trayectoria.png')
MIN_STEP = 0.002
PLOT_PERIOD = 0.10
SAVE_PERIOD = 2.0


class PlotOdom(Node):
    def __init__(self):
        super().__init__('plot_odom_grupo5')
        self.xs = []
        self.ys = []
        self.last_save = 0.0

        self.sub = self.create_subscription(
            Odometry, '/pose_ground_truth', self.pose_cb, 200)
        self.timer = self.create_timer(PLOT_PERIOD, self.update_plot)

        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.line, = self.ax.plot([], [], color='tab:blue', linewidth=2,
                                  label='Recorrido')
        self.start_dot, = self.ax.plot([], [], 'o', color='tab:green',
                                       markersize=9, label='Inicio')
        self.robot_dot, = self.ax.plot([], [], 'o', color='tab:red',
                                       markersize=9, label='Robot')
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.grid(True, linestyle='--', alpha=0.5)
        self.ax.set_xlabel('x [m]')
        self.ax.set_ylabel('y [m]')
        self.ax.set_title('Trayectoria real (Gazebo) - numero 5 (grupo 5)')
        self.ax.legend(loc='upper left')
        self.fig.tight_layout()
        plt.show(block=False)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        self.get_logger().info(f'Ventana lista. Se guarda en {SAVE_PATH}')

    def pose_cb(self, msg):
        p = msg.pose.pose.position
        if not self.xs or math.hypot(p.x - self.xs[-1],
                                     p.y - self.ys[-1]) >= MIN_STEP:
            self.xs.append(p.x)
            self.ys.append(p.y)

    def update_plot(self):
        if len(self.xs) < 2:
            return
        self.line.set_data(self.xs, self.ys)
        self.start_dot.set_data([self.xs[0]], [self.ys[0]])
        self.robot_dot.set_data([self.xs[-1]], [self.ys[-1]])

        xmin, xmax = min(self.xs), max(self.xs)
        ymin, ymax = min(self.ys), max(self.ys)
        cx, cy = 0.5 * (xmin + xmax), 0.5 * (ymin + ymax)
        span = max(xmax - xmin, ymax - ymin, 0.8) + 0.4
        self.ax.set_xlim(cx - 0.5 * span, cx + 0.5 * span)
        self.ax.set_ylim(cy - 0.5 * span, cy + 0.5 * span)

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

        now = time.time()
        if now - self.last_save > SAVE_PERIOD:
            self.save()
            self.last_save = now

    def save(self):
        if len(self.xs) < 2:
            return
        try:
            self.fig.savefig(SAVE_PATH, dpi=150, bbox_inches='tight')
        except Exception as exc:
            self.get_logger().warn(f'No se pudo guardar la grafica: {exc}')


def main():
    rclpy.init()
    node = PlotOdom()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(f'Grafica guardada en {SAVE_PATH}')


if __name__ == '__main__':
    sys.exit(main())
