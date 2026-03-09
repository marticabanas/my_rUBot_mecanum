import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class LidarTest(Node):

    def __init__(self):
        super().__init__('lidar_test_node')
        # Define Quality of Service to receive scan values faster. Its is not important if some of them are lost and only keep the last 10 values
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile
        )
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.info_mostrada = False
        self.rangos_mostrados = False

    def listener_callback(self, msg):
        current_time = self.get_clock().now().seconds_nanoseconds()[0]

        fov = msg.angle_max - msg.angle_min
        min_angle = msg.angle_min * 180.0 / 3.14159
        max_angle = msg.angle_max * 180.0 / 3.14159
        fov_deg = fov * 180.0 / 3.14159
        angle_increment_deg = msg.angle_increment * 180.0 / 3.14159
        total_points = len(msg.ranges)

        if not self.info_mostrada:
            self.get_logger().info(f"Mín angle: {min_angle:.2f} grados")
            self.get_logger().info(f"Máx angle: {max_angle:.2f} grados")
            self.get_logger().info(f"Campo de visión (FOV): {fov_deg:.2f} grados")
            self.get_logger().info(f"Incremento angular por punto: {angle_increment_deg:.2f} grados")
            self.get_logger().info(f"Número total de puntos del escaneo: {total_points}")
            self.info_mostrada = True

        if current_time - self.start_time < 10:
            if not self.rangos_mostrados:
                self.get_logger().info("Rangos angulares definidos para cada zona:")
                self.get_logger().info(" - DELANTE:            [-30°, 30°]")
                self.get_logger().info(" - SUP. IZQUIERDA:     (30°, 70°]")
                self.get_logger().info(" - IZQUIERDA:          (70°, 110°]")
                self.get_logger().info(" - SUP. DERECHA:       [-70°, -30°)")
                self.get_logger().info(" - DERECHA:            [-110°, -70°)")
                self.get_logger().info(" - ATRÁS:              (resto, fuera de visión habitual)")
                self.rangos_mostrados = True
            return

        ranges = msg.ranges
        angle_min_deg = msg.angle_min * 180.0 / 3.14159

        sections = {
            'delante': [],
            'sup_izquierda': [],
            'izquierda': [],
            'sup_derecha': [],
            'derecha': [],
            'atras': [],
        }

        for i, distance in enumerate(ranges):
            angle = angle_min_deg + i * angle_increment_deg
            if distance == float('inf') or distance == 0.0:
                continue
            if -30 <= angle <= 30:
                sections['delante'].append(distance)
            elif 30 < angle <= 70:
                sections['sup_izquierda'].append(distance)
            elif 70 < angle <= 110:
                sections['izquierda'].append(distance)
            elif -70 <= angle < -30:
                sections['sup_derecha'].append(distance)
            elif -110 <= angle < -70:
                sections['derecha'].append(distance)
            else:
                sections['atras'].append(distance)

        min_distances = {}
        for zona, valores in sections.items():
            if valores:
                min_dist = min(valores)
                min_distances[zona] = min_dist
                self.get_logger().info(f"Distancia mínima en {zona}: {min_dist:.2f} m")
            else:
                self.get_logger().info(f"Sin detecciones en {zona}")

        if min_distances:
            zona_mas_cercana = min(min_distances, key=min_distances.get)
            self.get_logger().info(f"--Objeto más cercano está en {zona_mas_cercana.upper()} ({min_distances[zona_mas_cercana]:.2f} m)")


def main(args=None):
    rclpy.init(args=args)
    scan_values = LidarTest()
    rclpy.spin(scan_values)
    scan_values.destroy_node()
    rclpy.shutdown()
