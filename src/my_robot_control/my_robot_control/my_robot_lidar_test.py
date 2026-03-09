#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class TwistLidarStop(Node):
    def __init__(self):
        super().__init__("twist_lidar_stop")

        # Parameters (match your launch)
        self.declare_parameter("vx", 0.3)
        self.declare_parameter("vy", 0.0)
        self.declare_parameter("w", 0.0)
        self.declare_parameter("stop_distance", 0.3)

        # Optional: limit FOV for detection
        self.declare_parameter("fov_min_deg", -150.0)
        self.declare_parameter("fov_max_deg", 150.0)

        self.vx = float(self.get_parameter("vx").value)
        self.vy = float(self.get_parameter("vy").value)
        self.w = float(self.get_parameter("w").value)
        self.stop_distance = float(self.get_parameter("stop_distance").value)

        self.fov_min_deg = float(self.get_parameter("fov_min_deg").value)
        self.fov_max_deg = float(self.get_parameter("fov_max_deg").value)

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self._moving_msg = Twist()
        self._moving_msg.linear.x = self.vx
        self._moving_msg.linear.y = self.vy
        self._moving_msg.angular.z = self.w

        self._stop_msg = Twist()  # all zeros
        self._stopped = False

        # Publish at fixed rate
        self.timer = self.create_timer(0.05, self.timer_cb)

        # Lidar subscription
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_cb,
            10,
        )

        self.get_logger().info(
            f"Publishing /cmd_vel: vx={self.vx:.2f}, vy={self.vy:.2f}, w={self.w:.2f} | "
            f"stop_distance={self.stop_distance:.2f} m | FOV=[{self.fov_min_deg:.0f}°, {self.fov_max_deg:.0f}°]"
        )

    def timer_cb(self):
        if self._stopped:
            self.cmd_pub.publish(self._stop_msg)
        else:
            self.cmd_pub.publish(self._moving_msg)

    def scan_cb(self, scan: LaserScan):
        if self._stopped:
            return

        angle_min_deg = math.degrees(scan.angle_min)
        angle_inc_deg = math.degrees(scan.angle_increment)

        candidates = []
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r <= 0.0:
                continue
            if r < scan.range_min or r > scan.range_max:
                continue

            ang_deg = angle_min_deg + i * angle_inc_deg
            # Normalize to [-180, 180)
            ang_deg = (ang_deg + 180.0) % 360.0 - 180.0

            if self.fov_min_deg <= ang_deg <= self.fov_max_deg:
                candidates.append((r, ang_deg))

        if not candidates:
            return

        closest_r, closest_ang = min(candidates, key=lambda x: x[0])

        if closest_r < self.stop_distance:
            self._stopped = True
            self.get_logger().warn(
                f"STOP: obstacle at {closest_r:.2f} m, angle {closest_ang:.0f}° "
                f"(threshold {self.stop_distance:.2f} m)"
            )


def main(args=None):
    rclpy.init(args=args)
    node = TwistLidarStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()