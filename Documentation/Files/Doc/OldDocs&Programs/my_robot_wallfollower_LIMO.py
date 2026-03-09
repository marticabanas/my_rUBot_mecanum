import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower_node')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.declare_parameter('distance_laser', 0.5)
        self.base_distance = self.get_parameter('distance_laser').get_parameter_value().double_value
        self.d = 0.05

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            qos_profile
        )

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.log_info)

        self.closest = (float('inf'), 'unknown')

    def laser_callback(self, msg):
        ranges = msg.ranges
        angle_min = msg.angle_min * 180.0 / 3.14159
        angle_increment = msg.angle_increment * 180.0 / 3.14159

        delante = []
        derecha = []

        for i, distance in enumerate(ranges):
            angle = angle_min + i * angle_increment
            if distance == float('inf') or distance == 0.0:
                continue
            if -15 <= angle <= 15:
                delante.append(distance)
            elif -110 <= angle < -15:
                derecha.append(distance)

        min_delante = min(delante) if delante else float('inf')
        min_derecha = min(derecha) if derecha else float('inf')

        self.closest = (min(min_delante, min_derecha), 'delante' if min_delante < min_derecha else 'derecha')

        twist = Twist()
        action_log = ""

        if min_delante < self.base_distance:
            twist.angular.z = 0.3
            action_log = "Giro por obstáculo delante"
        elif min_derecha < self.base_distance - self.d:
            twist.angular.z = 0.3
            action_log = "Demasiado cerca - girar izquierda para alejarse"
        elif min_derecha > self.base_distance + self.d:
            twist.angular.z = -0.3
            action_log = "Demasiado lejos - girar derecha para acercarse"
        else:
            twist.linear.x = 0.2
            action_log = "Siguiendo pared"

        self.get_logger().info(f"{action_log}")
        # self.get_logger().info(f"Lecturas válidas -> Delante: {len(delante)}, Derecha: {len(derecha)}")

        self.publisher.publish(twist)

    def log_info(self):
        dist, region = self.closest
        if isinstance(dist, (float, int)) and isinstance(region, str):
            self.get_logger().info(f"Objeto más cercano a {dist:.2f} m en región: {region.upper()}")
        else:
            self.get_logger().warn(f"Distancia inválida ({dist}) de tipo {type(dist)} recibida, no se puede formatear.")

        # self.get_logger().info(f"Distancia base: {self.base_distance:.2f} m | Umbral: ±{self.d:.2f} m")


def main(args=None):
    rclpy.init(args=args)
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()
