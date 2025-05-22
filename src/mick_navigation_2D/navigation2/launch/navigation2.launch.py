from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node  
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    navigation_pkg = 'navigation2'
    rviz_config = PathJoinSubstitution([
        FindPackageShare(navigation_pkg),
        'rviz',
        'navigation2.rviz'  
    ])
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock'),
            
        DeclareLaunchArgument(
            'autostart',
            default_value='true', 
            description='Automatically startup nav2'),
            
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'bringup_launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': LaunchConfiguration('autostart'),
                'map': PathJoinSubstitution([
                    FindPackageShare(navigation_pkg),
                    'maps',
                    'map_out.yaml'
                ])
            }.items()
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    ])
