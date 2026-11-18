import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class SecurityPatrolNode(Node):
    def __init__(self):
        super().__init__('security_patrol_node')
        
        # 【新增】宣告可由 Launch 檔或命令列動態調整的 ROS 2 參數
        self.declare_parameter('max_speed', 0.20)
        self.declare_parameter('inspect_duration', 3.0)
        self.declare_parameter('timeout_sec', 25.0)

        # 讀取參數數值
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.inspect_duration_sec = self.get_parameter('inspect_duration').get_parameter_value().double_value
        self.timeout_sec = self.get_parameter('timeout_sec').get_parameter_value().double_value

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        # 四角落安防巡邏路線
        self.waypoints = [
            (1.5, -0.5),   # 航點 1：右下方空地
            (1.5,  1.2),   # 航點 2：右上方空地
            (-1.5, 1.2),   # 航點 3：左上方空地
            (-1.5, -0.5)   # 航點 4：左下方起點附近
        ]
        self.current_wp_idx = 0
        self.goal_tolerance = 0.18
        self.loop_patrol = True

        self.state = 'NAVIGATING'
        self.waypoint_start_time = None
        self.inspect_start_time = None
        self.inspection_triggered = False

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False

        self.min_front = 3.5
        self.min_f_left = 3.5
        self.min_f_right = 3.5
        self.avg_left = 3.5
        self.avg_right = 3.5
        self.turn_direction = 0

        self.get_logger().info(
            f'參數化安防巡檢已啟動！[最大速度: {self.max_speed} m/s | 停留秒數: {self.inspect_duration_sec}s | 超時: {self.timeout_sec}s]'
        )
        self.get_logger().info(f'>>> 前往航點 1: {self.waypoints}')

    def get_now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def euler_from_quaternion(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.odom_received = True

    def get_clean_ranges(self, raw_ranges):
        clean = []
        for d in raw_ranges:
            if 0.05 < d < 3.5:
                clean.append(d)
            elif d >= 3.5 or d == float('inf'):
                clean.append(3.5)
        return clean

    def scan_callback(self, msg):
        front_center = self.get_clean_ranges(list(msg.ranges[0:15]) + list(msg.ranges[345:360]))
        front_left   = self.get_clean_ranges(list(msg.ranges[15:45]))
        front_right  = self.get_clean_ranges(list(msg.ranges[315:345]))
        side_left    = self.get_clean_ranges(list(msg.ranges[45:85]))
        side_right   = self.get_clean_ranges(list(msg.ranges[275:315]))

        self.min_front = min(front_center) if front_center else 3.5
        self.min_f_left = min(front_left) if front_left else 3.5
        self.min_f_right = min(front_right) if front_right else 3.5
        self.avg_left = sum(side_left) / len(side_left) if side_left else 0.0
        self.avg_right = sum(side_right) / len(side_right) if side_right else 0.0

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def switch_to_next_waypoint(self, reason_text):
        wp_num = self.current_wp_idx + 1
        self.get_logger().info(f'[{reason_text}] 結束航點 {wp_num} 任務')

        if not self.loop_patrol and self.current_wp_idx == len(self.waypoints) - 1:
            self.state = 'FINISHED'
            self.cmd_pub.publish(Twist())
            self.get_logger().info('所有巡檢任務已完成，機器人已停車待命！')
            return

        self.current_wp_idx = (self.current_wp_idx + 1) % len(self.waypoints)
        next_wp = self.waypoints[self.current_wp_idx]
        
        self.state = 'NAVIGATING'
        self.waypoint_start_time = self.get_now_sec()
        self.inspection_triggered = False
        self.turn_direction = 0
        self.get_logger().info(f'>>> 切換目標 -> 前往航點 {self.current_wp_idx + 1}: {next_wp}')

    def perform_camera_inspection(self, wp_num):
        self.get_logger().info(f'   [視覺巡檢] 航點 {wp_num} 機身已靜止 -> 啟動相機擷取與辨識...')

    def control_loop(self):
        if not self.odom_received or self.state == 'FINISHED':
            return

        now = self.get_now_sec()
        if self.waypoint_start_time is None:
            self.waypoint_start_time = now

        goal_x, goal_y = self.waypoints[self.current_wp_idx]
        wp_num = self.current_wp_idx + 1

        if self.state == 'INSPECTING':
            self.cmd_pub.publish(Twist())
            if not self.inspection_triggered:
                self.perform_camera_inspection(wp_num)
                self.inspection_triggered = True

            if (now - self.inspect_start_time) >= self.inspect_duration_sec:
                self.switch_to_next_waypoint(f'停留偵測完成 ({self.inspect_duration_sec}s)')
            return

        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        distance_to_goal = math.hypot(dx, dy)

        if distance_to_goal < self.goal_tolerance:
            self.cmd_pub.publish(Twist())
            self.state = 'INSPECTING'
            self.inspect_start_time = now
            self.get_logger().info(f'>>> 已抵達航點 {wp_num} ({goal_x}, {goal_y})！進入定點停留偵測模式...')
            return

        if (now - self.waypoint_start_time) > self.timeout_sec:
            self.get_logger().warn(f'警告：前往航點 {wp_num} 已超過 {self.timeout_sec} 秒，判定路徑受阻！')
            self.switch_to_next_waypoint('超時自動跳過')
            return

        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - self.current_yaw)

        twist = Twist()
        stop_dist = 0.30
        caution_dist = 0.55

        if self.min_front < stop_dist:
            twist.linear.x = 0.0
            if self.turn_direction == 0:
                self.turn_direction = 1 if self.avg_left >= self.avg_right else -1
            twist.angular.z = 0.7 * self.turn_direction

        elif self.min_front < caution_dist or self.min_f_left < caution_dist or self.min_f_right < caution_dist:
            self.turn_direction = 0
            # 動態避障時自動將速度降為最大速度的 60%
            twist.linear.x = self.max_speed * 0.6
            steer_error = self.min_f_left - self.min_f_right
            angular_cmd = max(-0.85, min(0.85, steer_error * 1.8))
            if abs(angular_cmd) < 0.15:
                angular_cmd = 0.55 if self.avg_left >= self.avg_right else -0.55
            twist.angular.z = angular_cmd

        else:
            self.turn_direction = 0
            if abs(angle_error) > 0.5:
                twist.linear.x = 0.06
                twist.angular.z = max(-0.8, min(0.8, angle_error * 1.2))
            else:
                # 使用外部傳入的 max_speed 作為巡航速度上限
                twist.linear.x = min(self.max_speed, max(0.08, distance_to_goal * 0.5))
                twist.angular.z = max(-0.6, min(0.6, angle_error * 1.2))

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SecurityPatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
