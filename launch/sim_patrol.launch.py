import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    tb3_bringup_dir = get_package_share_directory('turtlebot3_bringup')

    max_speed_arg = DeclareLaunchArgument('max_speed', default_value='0.20')
    inspect_duration_arg = DeclareLaunchArgument('inspect_duration', default_value='3.0')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')

    max_speed = LaunchConfiguration('max_speed')
    inspect_duration = LaunchConfiguration('inspect_duration')
    use_rviz = LaunchConfiguration('use_rviz')

    # 1. 第 0 秒：專心啟動 Gazebo 模擬場地與生成 Burger
    gazebo_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # 2. 第 10 秒：等 Gazebo 與車子完全生成後，再開啟 RViz2 視覺化監控
    rviz_config_file = os.path.join(tb3_bringup_dir, 'rviz', 'tb3_gazebo.rviz')
    rviz_node_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
        output='screen'
    )
    delayed_rviz_cmd = TimerAction(period=10.0, actions=[rviz_node_cmd])

    # 3. 第 14 秒：啟動安防自動巡檢節點開始出發
    patrol_node_cmd = Node(
        package='my_tb3_avoid',
        executable='patrol_node',
        name='security_patrol_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'max_speed': max_speed,
            'inspect_duration': inspect_duration
        }]
    )
    delayed_patrol_cmd = TimerAction(period=14.0, actions=[patrol_node_cmd])

    ld = LaunchDescription()
    ld.add_action(max_speed_arg)
    ld.add_action(inspect_duration_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(gazebo_sim_cmd)
    ld.add_action(delayed_rviz_cmd)
    ld.add_action(delayed_patrol_cmd)

    return ld
