import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import math


class RobotSelfControlHolonomic(Node):

    def __init__(self):
        super().__init__('robot_selfcontrol_holonomic_node')

        self.declare_parameter('distance_limit', 0.3)
        self.declare_parameter('forward_speed',  0.2)
        self.declare_parameter('lateral_speed',  0.2)
        self.declare_parameter('time_to_stop',   30.0)

        self._dist = self.get_parameter('distance_limit').value
        self._fwd  = self.get_parameter('forward_speed').value
        self._lat  = self.get_parameter('lateral_speed').value
        self._tmax = self.get_parameter('time_to_stop').value

        self._msg  = Twist()
        self._msg.linear.x = self._fwd
        self._shutting_down = False
        self._last_log = 0.0

        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self._laser_cb, scan_qos)
        self._timer = self.create_timer(0.05, self._ctrl_cb)
        self._t0 = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(
            f"Selfcontrol holonomic | dist={self._dist} fwd={self._fwd} lat={self._lat} t={self._tmax}s"
        )

    def _ctrl_cb(self):
        if self._shutting_down:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        self._pub.publish(self._msg)
        if now - self._last_log >= 1.0:
            self.get_logger().info(
                f"t={now-self._t0:.1f}s | vx={self._msg.linear.x:.2f} vy={self._msg.linear.y:.2f}"
            )
            self._last_log = now
        if now - self._t0 >= self._tmax:
            self._timer.cancel()
            self._shutting_down = True
            self._pub.publish(Twist())
            self.get_logger().info("Stopped.")

    def _laser_cb(self, scan):
        if self._shutting_down:
            return

        # Single-pass minimum in [-150°, 150°]
        min_d, min_a = float('inf'), 0.0
        for i, d in enumerate(scan.ranges):
            if not math.isfinite(d) or d < scan.range_min or d > scan.range_max:
                continue
            a = math.degrees(scan.angle_min + i * scan.angle_increment)
            if a > 180.0: a -= 360.0
            if -150.0 < a < 150.0 and d < min_d:
                min_d, min_a = d, a

        if min_d == float('inf'):
            return

        # Zone of closest obstacle
        if   -45  <= min_a <=  45:  zone = "FRONT"
        elif  45  <  min_a <= 110:  zone = "LEFT"
        elif -110 <= min_a <  -45:  zone = "RIGHT"
        elif  110 <  min_a <= 150:  zone = "BACK_LEFT"
        else:                       zone = "BACK_RIGHT"

        # Holonomic reaction: slide away from obstacle zone, no rotation
        if min_d < self._dist:
            if zone == "FRONT":
                # obstacle ahead → slide left
                self._msg.linear.x, self._msg.linear.y = -self._fwd, self._lat
            elif zone == "LEFT":
                # obstacle left → slide right
                self._msg.linear.x, self._msg.linear.y = 0.0, -self._lat
            elif zone == "RIGHT":
                # obstacle right → slide left
                self._msg.linear.x, self._msg.linear.y = 0.0, self._lat
            elif zone == "BACK_LEFT":
                # obstacle back-left → move forward-right
                self._msg.linear.x, self._msg.linear.y = self._fwd, -self._lat
            else:  # BACK_RIGHT
                # obstacle back-right → move forward-left
                self._msg.linear.x, self._msg.linear.y = self._fwd, self._lat
            self._msg.angular.z = 0.0
        else:
            # Free → go straight
            self._msg.linear.x, self._msg.linear.y, self._msg.angular.z = self._fwd, 0.0, 0.0


def main(args=None):
    rclpy.init(args=args)
    node = RobotSelfControlHolonomic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()