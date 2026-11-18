import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class GoToGoalAvoidNode(Node):
    def __init__(self):
        super().__init__('go_to_goal_avoid_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 訂閱光達與里程計兩個感測器話題
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # 建立 10Hz (每 0.1 秒) 的主控制迴圈，整合感測器數據並決策
        self.timer = self.create_timer(0.1, self.control_loop)

        # 設定目標點座標（單位：公尺，可依場地大小自由修改）
        self.goal_x = 1.5
        self.goal_y = 1.5
        self.goal_tolerance = 0.15  # 抵達目標的誤差容許範圍（15公分內視為到達）

        # 機器人即時狀態變數
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False

        # 光達即時狀態變數
        self.min_front = 3.5
        self.min_f_left = 3.5
        self.min_f_right = 3.5
        self.avg_left = 3.5
        self.avg_right = 3.5
        self.turn_direction = 0
        self.goal_reached = False

        self.get_logger().info(f'定點導航＋動態避障節點已啟動！目標座標: ({self.goal_x}, {self.goal_y})')

    def euler_from_quaternion(self, x, y, z, w):
        # 將里程計的四元數 (Quaternion) 轉換為平面偏航角 Yaw（弧度）
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
        # 將角度誤差限制在 -pi 到 +pi 之間，確保車子永遠選擇最近的方向轉彎
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def control_loop(self):
        if not self.odom_received or self.goal_reached:
            return

        # 1. 計算目前位置距離目標點的直線距離與方位角
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance_to_goal = math.hypot(dx, dy)

        # 判斷是否已經抵達目標點
        if distance_to_goal < self.goal_tolerance:
            self.goal_reached = True
            self.cmd_pub.publish(Twist())
            self.get_logger().info(f'已精準抵達目標點 ({self.goal_x}, {self.goal_y})！最終誤差: {distance_to_goal:.2f}m')
            return

        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - self.current_yaw)

        twist = Twist()
        stop_dist = 0.30
        caution_dist = 0.55

        # 【最高優先級：正前方緊急脫困】
        if self.min_front < stop_dist:
            twist.linear.x = 0.0
            if self.turn_direction == 0:
                self.turn_direction = 1 if self.avg_left >= self.avg_right else -1
            twist.angular.z = 0.7 * self.turn_direction
            self.get_logger().info(f'[緊急避障] 距離目標 {distance_to_goal:.2f}m | 正在脫困...')

        # 【第二優先級：邊走邊微調繞開柱子】
        elif self.min_front < caution_dist or self.min_f_left < caution_dist or self.min_f_right < caution_dist:
            self.turn_direction = 0
            twist.linear.x = 0.12
            steer_error = self.min_f_left - self.min_f_right
            angular_cmd = max(-0.85, min(0.85, steer_error * 1.8))
            if abs(angular_cmd) < 0.15:
                angular_cmd = 0.55 if self.avg_left >= self.avg_right else -0.55
            twist.angular.z = angular_cmd
            self.get_logger().info(f'[動態繞柱] 距離目標 {distance_to_goal:.2f}m | 閃避障礙中...')

        # 【第三優先級：前方安全，朝向目標座標前進】
        else:
            self.turn_direction = 0
            # 如果車頭跟目標角度偏差較大（大於約 30 度），先減速轉向對準目標
            if abs(angle_error) > 0.5:
                twist.linear.x = 0.06
                twist.angular.z = max(-0.8, min(0.8, angle_error * 1.2))
            else:
                # 已經大致對準目標，全速前進並微調航向
                twist.linear.x = min(0.20, max(0.08, distance_to_goal * 0.5))
                twist.angular.z = max(-0.6, min(0.6, angle_error * 1.2))

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = GoToGoalAvoidNode()
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
