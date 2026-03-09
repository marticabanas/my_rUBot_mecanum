#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from numpy import sign
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class RobotSelfControl(Node):

    def __init__(self):
        super().__init__('robot_selfcontrol_node')

        # Parámetros configurables
        self.declare_parameter('distance_laser', 0.3)
        self.declare_parameter('speed_factor', 1.0)
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('rotation_speed', 0.3)
        self.declare_parameter('time_to_stop', 5.0)

        # Obtener valores de los parámetros
        self._distanceLaser = self.get_parameter('distance_laser').value
        self._speedFactor = self.get_parameter('speed_factor').value
        self._forwardSpeed = self.get_parameter('forward_speed').value
        self._rotationSpeed = self.get_parameter('rotation_speed').value
        self._time_to_stop = self.get_parameter('time_to_stop').value

        # Mensaje inicial de movimiento hacia delante
        self._msg = Twist()
        self._msg.linear.x = self._forwardSpeed * self._speedFactor
        self._msg.angular.z = 0.0

        self._cmdVel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.Laser_callback,
            qos_profile
        )

        self.__isScanRangesLengthCorrectionFactorCalculated = False
        self.__scanRangesLengthCorrectionFactor = 2

        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self._shutting_down = False

        self._last_info_time = self.start_time  # Para mostrar info de distancia/ángulo cada segundo
        self._last_speed_time = self.start_time  # Para mostrar velocidad/tiempo cada segundo

    def timer_callback(self):
        if self._shutting_down:
            return

        now_sec = self.get_clock().now().seconds_nanoseconds()[0]
        elapsed_time = now_sec - self.start_time

        self._cmdVel.publish(self._msg)

        if now_sec - self._last_speed_time >= 1:
            self.get_logger().info(f"Vx: {self._msg.linear.x:.2f} m/s, w: {self._msg.angular.z:.2f} rad/s | Tiempo: {elapsed_time:.1f}s")
            self._last_speed_time = now_sec

        if elapsed_time >= self._time_to_stop:
            self.stop()
            self.timer.cancel()
            self.get_logger().info("Robot detenido")
            rclpy.try_shutdown()

    def Laser_callback(self, scan):
        if self._shutting_down:
            return

        # Cálculo de relación puntos/grado
        if not self.__isScanRangesLengthCorrectionFactorCalculated:
            self.__scanRangesLengthCorrectionFactor = int(len(scan.ranges) / 360)
            self.__isScanRangesLengthCorrectionFactorCalculated = True
            self.get_logger().info(f"Lidar beams/degree factor: {self.__scanRangesLengthCorrectionFactor:.0f}")

        # Buscar distancia más cercana válida
        closestDistance, elementIndex = min(
            (val, idx) for (idx, val) in enumerate(scan.ranges)
            if scan.range_min < val < scan.range_max
        )

        # Convertir índice a ángulo en grados
        angleClosestDistance_deg = (scan.angle_min + elementIndex * scan.angle_increment) * 180.0 / 3.14159

        # Determinar zona del objeto más cercano
        if -45 <= angleClosestDistance_deg <= 45:
            zona = "DELANTE"
        elif 45 < angleClosestDistance_deg <= 110:
            zona = "IZQUIERDA"
        elif -110 <= angleClosestDistance_deg < -45:
            zona = "DERECHA"
        else:
            zona = "FUERA DEL FOV"

        now = self.get_clock().now().seconds_nanoseconds()[0]
        if now - self._last_info_time >= 1:
            self.get_logger().info(f"[DETECCIÓN] Objeto más cercano a {closestDistance:.2f} m | Ángulo: {angleClosestDistance_deg:.1f}° | Zona: {zona}")
            self._last_info_time = now

        # Reaccionar si el obstáculo está dentro del umbral de distancia
        if closestDistance < self._distanceLaser:
            if zona == "DELANTE":
                self._msg.linear.x = 0.0
                self._msg.angular.z = 0.0
            elif zona == "IZQUIERDA":
                self._msg.linear.x = 0.0
                self._msg.angular.z = -self._rotationSpeed * self._speedFactor
            elif zona == "DERECHA":
                self._msg.linear.x = 0.0
                self._msg.angular.z = self._rotationSpeed * self._speedFactor
            else:
                self._msg.linear.x = self._forwardSpeed * self._speedFactor
                self._msg.angular.z = 0.0
        else:
            self._msg.linear.x = self._forwardSpeed * self._speedFactor
            self._msg.angular.z = 0.0

    def stop(self):
        self._shutting_down = True
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self._cmdVel.publish(stop_msg)
        rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    rubot = RobotSelfControl()
    try:
        rclpy.spin(rubot)
    except KeyboardInterrupt:
        pass
    finally:
        rubot.destroy_node()


if __name__ == '__main__':
    main()
