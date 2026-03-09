import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import math


class RobotSelfControlHolonomic(Node):

    def __init__(self):
        super().__init__('robot_selfcontrol_holonomic_node')

        # Configurable parameters
        self.declare_parameter('distance_limit', 0.3)
        self.declare_parameter('speed_factor', 1.0)
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('lateral_speed', 0.2)
        self.declare_parameter('time_to_stop', 5.0)

        self._distanceLimit = self.get_parameter('distance_limit').value
        self._speedFactor   = self.get_parameter('speed_factor').value
        self._forwardSpeed  = self.get_parameter('forward_speed').value
        self._lateralSpeed  = self.get_parameter('lateral_speed').value
        self._time_to_stop  = self.get_parameter('time_to_stop').value

        # Working Twist — updated by laser, published by timer
        self._msg = Twist()
        self._msg.linear.x = self._forwardSpeed * self._speedFactor

        # Latest laser reading (set atomically in callback)
        self._closest_distance = float('inf')
        self._closest_angle    = 0.0
        self._closest_zone     = "NONE"

        # Publisher
        self._cmdVel = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer: 20 Hz control loop
        self.timer = self.create_timer(0.05, self.timer_callback)

        # QoS compatible with real LIDAR (BEST_EFFORT) and Gazebo (RELIABLE)
        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.create_subscription(LaserScan, '/scan', self.laser_callback, scan_qos)

        self._start_time      = self.get_clock().now().nanoseconds * 1e-9
        self._last_log_time   = self._start_time
        self._shutting_down   = False

        self.get_logger().info(
            f"Holonomic selfcontrol started | "
            f"dist_limit={self._distanceLimit} m | "
            f"fwd={self._forwardSpeed} m/s | lat={self._lateralSpeed} m/s | "
            f"stop_after={self._time_to_stop} s"
        )

    # ------------------------------------------------------------------
    # Timer: publish current command + check timeout
    # ------------------------------------------------------------------
    def timer_callback(self):
        if self._shutting_down:
            return

        now   = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now - self._start_time

        self._cmdVel.publish(self._msg)

        # Periodic log every second
        if now - self._last_log_time >= 1.0:
            self.get_logger().info(
                f"t={elapsed:.1f}s | "
                f"vx={self._msg.linear.x:.2f} vy={self._msg.linear.y:.2f} | "
                f"zone={self._closest_zone} dist={self._closest_distance:.2f} m angle={self._closest_angle:.0f}°"
            )
            self._last_log_time = now

        if elapsed >= self._time_to_stop:
            self.timer.cancel()
            self.stop()
            self.get_logger().info("Time limit reached — robot stopped.")

    # ------------------------------------------------------------------
    # Laser: find closest valid beam in one pass, then decide command
    # ------------------------------------------------------------------
    def laser_callback(self, scan):
        if self._shutting_down:
            return

        angle_min_rad = scan.angle_min
        angle_inc_rad = scan.angle_increment
        range_min     = scan.range_min
        range_max     = scan.range_max

        min_dist  = float('inf')
        min_angle = 0.0

        # Single-pass minimum — no list allocation
        for i, d in enumerate(scan.ranges):
            if not math.isfinite(d) or d < range_min or d > range_max:
                continue
            angle_deg = math.degrees(angle_min_rad + i * angle_inc_rad)
            if angle_deg > 180.0:
                angle_deg -= 360.0
            if not (-150.0 < angle_deg < 150.0):
                continue
            if d < min_dist:
                min_dist  = d
                min_angle = angle_deg

        if min_dist == float('inf'):
            return  # no valid beam

        # Classify zone
        a = min_angle
        if   -45  <= a <=  45:   zone = "FRONT"
        elif  45  <  a <= 110:   zone = "LEFT"
        elif -110 <= a < -45:    zone = "RIGHT"
        elif  110 <  a <= 150:   zone = "BACK_LEFT"
        else:                    zone = "BACK_RIGHT"   # -150 to -110

        # Store for logger
        self._closest_distance = min_dist
        self._closest_angle    = min_angle
        self._closest_zone     = zone

        # --- HOLONOMIC REACTION ---
        # Key idea: instead of rotating away, slide laterally away.
        # vx  = forward/backward
        # vy  = left (+) / right (-)   ← holonomic axis
        # wz  = kept at 0 (pure translation avoidance)

        fwd = self._forwardSpeed  * self._speedFactor
        lat = self._lateralSpeed  * self._speedFactor

        if min_dist < self._distanceLimit:
            if zone == "FRONT":
                # Obstacle ahead → slide left
                self._msg.linear.x  =  0.0
                self._msg.linear.y  =  lat
                self._msg.angular.z =  0.0

            elif zone == "LEFT":
                # Obstacle on the left → slide right
                self._msg.linear.x  =  0.0
                self._msg.linear.y  = -lat
                self._msg.angular.z =  0.0

            elif zone == "RIGHT":
                # Obstacle on the right → slide left
                self._msg.linear.x  =  0.0
                self._msg.linear.y  =  lat
                self._msg.angular.z =  0.0

            elif zone == "BACK_LEFT":
                # Behind-left → move forward-right
                self._msg.linear.x  =  fwd
                self._msg.linear.y  = -lat
                self._msg.angular.z =  0.0

            elif zone == "BACK_RIGHT":
                # Behind-right → move forward-left
                self._msg.linear.x  =  fwd
                self._msg.linear.y  =  lat
                self._msg.angular.z =  0.0

        else:
            # Free path → go straight
            self._msg.linear.x  =  fwd
            self._msg.linear.y  =  0.0
            self._msg.angular.z =  0.0

    # ------------------------------------------------------------------
    def stop(self):
        self._shutting_down = True
        self._cmdVel.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    robot = RobotSelfControlHolonomic()
    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            robot._cmdVel.publish(Twist())
        except Exception:
            pass
        robot.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()