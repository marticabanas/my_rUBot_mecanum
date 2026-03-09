import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallFollowerHolonomic(Node):
    def __init__(self):
        super().__init__('wall_follower_holonomic_node')

        self.declare_parameter('distance_limit', 0.5)
        self.declare_parameter('forward_speed',  0.20)
        self.declare_parameter('lateral_speed',  0.20)
        self.declare_parameter('time_to_stop',   30.0)
        self.declare_parameter('tolerance',      0.05)

        self.base_distance = float(self.get_parameter('distance_limit').value)
        self.v_lin         = float(self.get_parameter('forward_speed').value)
        self.v_lat         = float(self.get_parameter('lateral_speed').value)
        self.time_to_stop  = float(self.get_parameter('time_to_stop').value)
        self.tol           = float(self.get_parameter('tolerance').value)

        self.cmd = Twist()
        self._state_action      = "Idle"
        self._last_action_logged = None
        self._shutting_down     = False
        self.start_time_s       = self.get_clock().now().nanoseconds * 1e-9

        self.create_subscription(LaserScan, '/scan', self.laser_callback, qos_profile_sensor_data)
        self.publisher  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_timer  = self.create_timer(0.05,  self.cmd_publish_cb)   # 20 Hz
        self.stop_timer = self.create_timer(0.05,  self.stop_watchdog)
        self.log_timer  = self.create_timer(1.0,   self.log_info)

        self.get_logger().info(
            f"WallFollower HOLONOMIC | target={self.base_distance} m "
            f"tol=±{self.tol} | fwd={self.v_lin} lat={self.v_lat} m/s"
        )

    # ------------------------------------------------------------------
    def stop_watchdog(self):
        if self._shutting_down:
            return
        if self.get_clock().now().nanoseconds * 1e-9 - self.start_time_s >= self.time_to_stop:
            self.get_logger().info("Timeout — stopping.")
            self.stop()

    def stop(self):
        self._shutting_down = True
        self.cmd = Twist()
        try:
            self.publisher.publish(self.cmd)
        except Exception:
            pass
        for t in [self.cmd_timer, self.stop_timer, self.log_timer]:
            try:
                t.cancel()
            except Exception:
                pass

    def cmd_publish_cb(self):
        if not self._shutting_down:
            try:
                self.publisher.publish(self.cmd)
            except Exception:
                pass

    def log_info(self):
        if not self._shutting_down:
            self.get_logger().info(self._state_action)

    # ------------------------------------------------------------------
    def laser_callback(self, scan):
        if self._shutting_down:
            return

        angle_min = math.degrees(scan.angle_min)
        angle_inc = math.degrees(scan.angle_increment)

        FRONT, FR_RIGHT, RIGHT, BACK_RIGHT, BACK = [], [], [], [], []

        for i, d in enumerate(scan.ranges):
            if not math.isfinite(d) or d < scan.range_min or d > scan.range_max:
                continue
            ang = angle_min + i * angle_inc
            if   -20  <= ang <=  20:   FRONT.append(d)
            elif -70  <= ang < -20:    FR_RIGHT.append(d)
            elif -110 <= ang < -70:    RIGHT.append(d)
            elif -160 <= ang < -110:   BACK_RIGHT.append(d)
            elif ang < -160 or ang > 160: BACK.append(d)

        min_front      = min(FRONT)      if FRONT      else float('inf')
        min_fr_right   = min(FR_RIGHT)   if FR_RIGHT   else float('inf')
        min_right      = min(RIGHT)      if RIGHT      else float('inf')
        min_back_right = min(BACK_RIGHT) if BACK_RIGHT else float('inf')
        min_back       = min(BACK)       if BACK       else float('inf')

        twist  = Twist()
        action = ""

        # ── RULE 1: obstacle in FRONT → back up + slide LEFT ──
        if min_front < self.base_distance:
            # Very close: back up too. Moderately close: just slide left.
            twist.linear.x  = -self.v_lin * 0.5 if min_front < self.base_distance * 0.5 else 0.0
            twist.linear.y  =  self.v_lat
            twist.angular.z =  0.0
            tag = 'back+slide' if min_front < self.base_distance * 0.5 else 'slide'
            action = f"FRONT {min_front:.2f} m → {tag} LEFT"

        # ── RULE 2: obstacle FRONT-RIGHT → move forward-left (vx + vy) ──
        elif min_fr_right < self.base_distance:
            twist.linear.x  =  self.v_lin * 0.5
            twist.linear.y  =  self.v_lat
            twist.angular.z =  0.0
            action = f"FRONT-RIGHT {min_fr_right:.2f} m → forward-LEFT"

        # ── RULE 3: RIGHT visible → maintain target distance with lateral slide ──
        elif math.isfinite(min_right):
            error = min_right - self.base_distance  # >0 too far, <0 too close

            if abs(error) <= self.tol:
                # Perfect distance → go straight forward, no lateral correction
                twist.linear.x  =  self.v_lin
                twist.linear.y  =  0.0
                twist.angular.z =  0.0
                action = (f"RIGHT OK ({min_right:.2f} m ≈ {self.base_distance:.2f}) → STRAIGHT")

            elif error < 0:
                # Too close to wall → slide LEFT while moving forward
                twist.linear.x  =  self.v_lin
                twist.linear.y  =  self.v_lat * min(2.0, abs(error) / self.tol)
                twist.angular.z =  0.0
                action = (f"RIGHT CLOSE ({min_right:.2f} m) → forward + slide LEFT")

            else:
                # Too far from wall → slide RIGHT while moving forward
                twist.linear.x  =  self.v_lin
                twist.linear.y  = -self.v_lat * min(2.0, abs(error) / self.tol)
                twist.angular.z =  0.0
                action = (f"RIGHT FAR ({min_right:.2f} m) → forward + slide RIGHT")

        # ── RULE 4: BACK-RIGHT → move forward-right to reacquire wall ──
        elif math.isfinite(min_back_right):
            twist.linear.x  =  self.v_lin
            twist.linear.y  = -self.v_lat          # slide right (toward wall)
            twist.angular.z =  0.0
            action = f"BACK-RIGHT {min_back_right:.2f} m → forward-RIGHT"

        # ── RULE 5: obstacle only at BACK → slide RIGHT to find wall ──
        elif math.isfinite(min_back):
            twist.linear.x  =  0.0
            twist.linear.y  = -self.v_lat
            twist.angular.z =  0.0
            action = f"BACK {min_back:.2f} m → slide RIGHT"

        # ── No wall visible → go straight ──
        else:
            twist.linear.x  =  self.v_lin
            twist.linear.y  =  0.0
            twist.angular.z =  0.0
            action = "No wall detected → STRAIGHT"

        self.cmd = twist

        # Log only on state change
        if action != self._last_action_logged:
            self.get_logger().info(action)
            self._last_action_logged = action
        self._state_action = action


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerHolonomic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()