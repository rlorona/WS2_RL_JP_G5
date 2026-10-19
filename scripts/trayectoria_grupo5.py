#!/usr/bin/env python3
"""Dibuja el numero 5 (grupo 5) con el robot diferencial en Gazebo.

Control en lazo cerrado usando la odometria publicada por el plugin
gz-sim-diff-drive-system en /odom.
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

LINEAR_SPEED = 0.15
ANGULAR_SPEED = 0.60
LINEAR_SIGN = 1.0
ANGULAR_SIGN = 1.0
CONTROL_PERIOD = 0.02
STOP_PAUSE = 0.5

SEGMENT = 0.60
HEIGHT = 0.30


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class Trayectoria(Node):
    def __init__(self):
        super().__init__('trayectoria_grupo5')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.have_odom = False

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.x = p.x
        self.y = p.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.have_odom = True

    def wait_for_odom(self, timeout=10.0):
        start = time.time()
        while rclpy.ok() and not self.have_odom:
            rclpy.spin_once(self, timeout_sec=CONTROL_PERIOD)
            if time.time() - start > timeout:
                self.get_logger().error('No se recibio /odom')
                return False
        return True

    def send(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.pub.publish(msg)

    def detener(self):
        self.send(0.0, 0.0)
        end = time.time() + STOP_PAUSE
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=CONTROL_PERIOD)

    def avanzar(self, distancia):
        self.wait_for_odom()
        start_x, start_y = self.x, self.y
        timeout = abs(distancia) / LINEAR_SPEED * 3.0 + 5.0
        start = time.time()
        self.get_logger().info(f'Avanzando {distancia:.2f} m')
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=CONTROL_PERIOD)
            recorrido = math.hypot(self.x - start_x, self.y - start_y)
            if recorrido >= abs(distancia) - 0.002:
                break
            if time.time() - start > timeout:
                self.get_logger().warn('Timeout avanzando')
                break
            self.send(math.copysign(LINEAR_SPEED, distancia) * LINEAR_SIGN, 0.0)
        self.detener()

    def girar(self, grados):
        self.wait_for_odom()
        objetivo = math.radians(grados)
        start_yaw = self.yaw
        timeout = abs(objetivo) / ANGULAR_SPEED * 3.0 + 5.0
        start = time.time()
        self.get_logger().info(f'Girando {grados:.0f} grados')
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=CONTROL_PERIOD)
            girado = normalize_angle(self.yaw - start_yaw)
            if abs(girado) >= abs(objetivo) - math.radians(0.5):
                break
            if time.time() - start > timeout:
                self.get_logger().warn('Timeout girando')
                break
            self.send(0.0, math.copysign(ANGULAR_SPEED, objetivo) * ANGULAR_SIGN)
        self.detener()


def main():
    rclpy.init()
    nodo = Trayectoria()
    try:
        if not nodo.wait_for_odom():
            return 1
        nodo.get_logger().info('Dibujando el numero 5 (grupo 5)')
        nodo.avanzar(SEGMENT)
        nodo.girar(+90)
        nodo.avanzar(HEIGHT)
        nodo.girar(+90)
        nodo.avanzar(SEGMENT)
        nodo.girar(-90)
        nodo.avanzar(HEIGHT)
        nodo.girar(-90)
        nodo.avanzar(SEGMENT)
        nodo.get_logger().info('Trayectoria terminada')
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        try:
            nodo.detener()
        except Exception:
            pass
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
