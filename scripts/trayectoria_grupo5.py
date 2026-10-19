#!/usr/bin/env python3
"""Dibuja el numero 5 (grupo 5) con el robot diferencial en Gazebo.

Usa la pose fisica real del modelo (/world/empty/pose/info). Los giros son
rapidos para vencer la friccion y se frena antes de cada esquina para
compensar la inercia. Cada tramo sigue la linea ideal horizontal/vertical
con correccion de desvio lateral. Publica la pose real en
/pose_ground_truth para la grafica en vivo.
"""

import math
import sys
import time

import gz.transport13 as gz_transport
from gz.msgs10.pose_v_pb2 import Pose_V

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

LINEAR_SPEED = 0.06
ANGULAR_SPEED = 0.80
CONTROL_PERIOD = 0.02
STOP_PAUSE = 0.60

POS_TOL = 0.015
YAW_TOL = math.radians(4.0)
K_HEADING = 3.0
K_CROSS = 4.0
MAX_CROSS_ANGLE = math.radians(20.0)

SEGMENT = 0.60
HEIGHT = 0.30

MODEL_NAME = 'Robot-Pogyo_Lorona-WS2'
POSE_TOPIC = '/world/empty/pose/info'

# El centro instantaneo de giro de un diferencial esta en el eje de las
# ruedas motrices. Ese punto (y no el origen de base_link, 8 cm adelante)
# es el que se mantiene fijo al girar en el lugar y el que debe dibujar el 5.
AXLE_OFFSET = -0.08


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


def clamp(value, low, high):
    return max(low, min(high, value))


class Trayectoria(Node):
    def __init__(self):
        super().__init__('trayectoria_grupo5')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gt_pub = self.create_publisher(Odometry, '/pose_ground_truth', 10)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.have_pose = False

        self.gz_node = gz_transport.Node()
        self.gz_node.subscribe(Pose_V, POSE_TOPIC, self.gz_pose_cb)
        self.create_timer(CONTROL_PERIOD, self.publish_ground_truth)

    @property
    def draw_x(self):
        return self.x + AXLE_OFFSET * math.cos(self.yaw)

    @property
    def draw_y(self):
        return self.y + AXLE_OFFSET * math.sin(self.yaw)

    def gz_pose_cb(self, msg):
        for pose in msg.pose:
            if pose.name == MODEL_NAME:
                self.x = pose.position.x
                self.y = pose.position.y
                self.yaw = yaw_from_quaternion(pose.orientation)
                self.have_pose = True
                return

    def publish_ground_truth(self):
        if not self.have_pose or not rclpy.ok():
            return
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self.draw_x
        msg.pose.pose.position.y = self.draw_y
        msg.pose.pose.orientation.z = math.sin(0.5 * self.yaw)
        msg.pose.pose.orientation.w = math.cos(0.5 * self.yaw)
        try:
            self.gt_pub.publish(msg)
        except Exception:
            pass

    def wait_for_pose(self, timeout=10.0):
        start = time.time()
        while rclpy.ok() and not self.have_pose:
            rclpy.spin_once(self, timeout_sec=CONTROL_PERIOD)
            if time.time() - start > timeout:
                self.get_logger().error('No se recibio la pose real de Gazebo')
                return False
        return True

    def send(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.pub.publish(msg)

    def detener(self, seconds=STOP_PAUSE):
        self.send(0.0, 0.0)
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=CONTROL_PERIOD)

    def girar_a(self, objetivo):
        self.wait_for_pose()
        start = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=CONTROL_PERIOD)
            error = normalize_angle(objetivo - self.yaw)
            if abs(error) <= YAW_TOL:
                break
            if time.time() - start > 10.0:
                self.get_logger().warn(
                    f'Timeout girando: error {math.degrees(error):.1f} deg')
                break
            self.send(0.0, math.copysign(ANGULAR_SPEED, error))
        self.detener()
        self.get_logger().info(
            f'Giro: yaw {math.degrees(self.yaw):.1f} deg '
            f'(objetivo {math.degrees(objetivo):.1f} deg)')

    def avanzar_a(self, x_destino, y_destino, x_linea, y_linea):
        self.wait_for_pose()
        dx = x_destino - x_linea
        dy = y_destino - y_linea
        longitud = math.hypot(dx, dy)
        if longitud < POS_TOL:
            return

        rumbo = math.atan2(dy, dx)
        self.girar_a(rumbo)

        cos_r = math.cos(rumbo)
        sin_r = math.sin(rumbo)
        start = time.time()
        self.get_logger().info(f'Avanzando a ({x_destino:.2f}, {y_destino:.2f})')
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=CONTROL_PERIOD)
            px = self.draw_x - x_linea
            py = self.draw_y - y_linea
            avance = px * cos_r + py * sin_r
            cruce = -px * sin_r + py * cos_r
            restante = longitud - avance

            if restante <= POS_TOL:
                break
            if time.time() - start > 45.0:
                self.get_logger().warn('Timeout avanzando')
                break

            correccion = clamp(-K_CROSS * cruce,
                               -MAX_CROSS_ANGLE, MAX_CROSS_ANGLE)
            deseado = rumbo + correccion
            error = normalize_angle(deseado - self.yaw)
            if abs(error) > math.radians(1.0):
                magnitud = clamp(3.0 * abs(error), 0.5, ANGULAR_SPEED)
                angular = math.copysign(magnitud, error)
            else:
                angular = 0.0
            if abs(error) > math.radians(20.0):
                lineal = 0.0
            else:
                lineal = min(LINEAR_SPEED, max(0.01, 1.0 * restante))
            self.send(lineal, angular)
        self.detener()


def main():
    rclpy.init()
    nodo = Trayectoria()
    try:
        if not nodo.wait_for_pose():
            return 1
        x0, y0 = nodo.draw_x, nodo.draw_y
        puntos = [
            (x0 + SEGMENT, y0),
            (x0 + SEGMENT, y0 + HEIGHT),
            (x0, y0 + HEIGHT),
            (x0, y0 + 2.0 * HEIGHT),
            (x0 + SEGMENT, y0 + 2.0 * HEIGHT),
        ]
        nodo.get_logger().info('Dibujando el numero 5 (grupo 5)')
        previo = (x0, y0)
        for i, (x, y) in enumerate(puntos, start=1):
            nodo.get_logger().info(
                f'Tramo {i}/5 -> ({x - x0:.2f}, {y - y0:.2f}) m')
            nodo.avanzar_a(x, y, previo[0], previo[1])
            previo = (x, y)
            nodo.get_logger().info(
                f'Fin tramo {i}: pose ({nodo.draw_x:.3f}, {nodo.draw_y:.3f})')
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
