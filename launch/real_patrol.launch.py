import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 取得實體車底層啟動套件 turtlebot3_bringup 的路徑
    tb3_bringup_dir = get_package_share_directory('turtlebot3_bringup')

    # 2. 啟動實體馬達與光達感測器 (Bringup)
    robot_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_bringup_dir, 'launch', 'robot.launch.py')
        )
    )

    # 3. 啟動你的安防自動巡檢節點（實體環境 use_sim_time 設為 False）
    patrol_node_cmd = Node(
        package='my_tb3_avoid',
        executable='patrol_node',
        name='security_patrol_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # 4. 延遲 4 秒等光達開始穩定旋轉後，再啟動巡檢出發
    delayed_patrol_cmd = TimerAction(
        period=4.0,
        actions=[patrol_node_cmd]
    )

    ld = LaunchDescription()
    ld.add_action(robot_bringup_cmd)
    ld.add_action(delayed_patrol_cmd)

    return ld
