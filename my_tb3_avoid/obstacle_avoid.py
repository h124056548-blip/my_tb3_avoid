import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class DynamicObstacleAvoidNode(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_avoid_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        self.turn_direction = 0  # 緊急原地旋轉時的鎖定方向（1:左, -1:右）
        self.get_logger().info('動態微調閃避節點已啟動！')

    def get_clean_ranges(self, raw_ranges):
        # 過濾近距離雜訊，並將超過感測器上限的無窮遠視為 3.5 公尺
        clean = []
        for d in raw_ranges:
            if 0.05 < d < 3.5:
                clean.append(d)
            elif d >= 3.5 or d == float('inf'):
                clean.append(3.5)
        return clean

    def scan_callback(self, msg):
        # 1. 將視野切分為五個精細扇區
        front_center = self.get_clean_ranges(list(msg.ranges[0:15]) + list(msg.ranges[345:360]))
        front_left   = self.get_clean_ranges(list(msg.ranges[15:45]))
        front_right  = self.get_clean_ranges(list(msg.ranges[315:345]))
        side_left    = self.get_clean_ranges(list(msg.ranges[45:85]))
        side_right   = self.get_clean_ranges(list(msg.ranges[275:315]))

        # 2. 計算各區域的最近距離與開闊度
        min_front = min(front_center) if front_center else 3.5
        min_f_left = min(front_left) if front_left else 3.5
        min_f_right = min(front_right) if front_right else 3.5
        
        avg_left = sum(side_left) / len(side_left) if side_left else 0.0
        avg_right = sum(side_right) / len(side_right) if side_right else 0.0

        twist = Twist()

        # 設定兩層安全距離門檻
        stop_dist = 0.35     # 緊急停車距離：正前方小於 35 公分才完全煞車
        caution_dist = 0.65  # 動態微調距離：前方或斜前方小於 65 公分開始邊走邊繞

        # 【第一層：緊急煞車原地避障區】正前方太近，必須停下旋轉
        if min_front < stop_dist:
            twist.linear.x = 0.0
            if self.turn_direction == 0:
                self.turn_direction = 1 if avg_left >= avg_right else -1
                side_str = '左側' if self.turn_direction == 1 else '右側'
                self.get_logger().info(f'正前方緊急障礙 ({min_front:.2f}m) -> 原地向{side_str}脫困')
            twist.angular.z = 0.7 * self.turn_direction

        # 【第二層：邊走邊微調動態閃避區】看到斜前方或側邊有柱子，不停車直接劃弧繞開
        elif min_front < caution_dist or min_f_left < caution_dist or min_f_right < caution_dist:
            self.turn_direction = 0
            # 降低前進速度，讓轉向半徑更平滑可控
            twist.linear.x = 0.12

            # 根據左右兩側的距離差（比例控制 P-Control）動態計算轉向力道
            # 右邊比較近就往左偏（正值），左邊比較近就往右偏（負值）
            steer_error = min_f_left - min_f_right
            
            # 限制最大微調角速度在 0.85 rad/s 以內，避免轉向過猛
            angular_cmd = max(-0.85, min(0.85, steer_error * 1.5))
            
            # 若正前方也有障礙（左右距離剛好相等時），優先往較空曠的一側偏轉
            if abs(angular_cmd) < 0.15:
                angular_cmd = 0.55 if avg_left >= avg_right else -0.55

            twist.angular.z = angular_cmd

        # 【第三層：自由巡航區】前方完全開闊，全速前進並貼緊牆壁保持安全距
        else:
            self.turn_direction = 0
            twist.linear.x = 0.20
            
            # 側邊防擦撞微調：如果側面太貼近柱子（小於 0.28m），輕微往反方向推開一點
            min_s_left = min(side_left) if side_left else 3.5
            min_s_right = min(side_right) if side_right else 3.5
            if min_s_left < 0.28:
                twist.angular.z = -0.35
            elif min_s_right < 0.28:
                twist.angular.z = 0.35
            else:
                twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleAvoidNode()
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
