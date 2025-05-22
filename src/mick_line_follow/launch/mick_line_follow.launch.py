from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # 启动Astra摄像头
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('astra_camera'),
                'launch',
                'astra_mini.launch.py'
            ]),
        ),
        
        # 启动巡线节点
        Node(
            package='mick_line_follow',
            executable='line_follower_node',
            name='line_follower',
            output='screen',
        ),
    ])
