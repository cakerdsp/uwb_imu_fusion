from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # 获取功能包路径
    pkg_share = FindPackageShare('uwb_imu_fusion')
    
    # 声明launch参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            pkg_share,
            'config',
            'default.yaml'
        ]),
        description='配置文件路径'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时间'
    )
    
    # 获取参数值
    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # 创建节点
    uwb_imu_fusion_node = Node(
        package='uwb_imu_fusion',
        executable='uwb_imu_fusion_node',
        name='uwb_imu_fusion',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            # 如果需要重映射话题，可以在这里添加
            # ('/imu/data', '/your_imu_topic'),
            # ('/odometry/filtered', '/your_odom_topic'),
        ]
    )
    
    return LaunchDescription([
        LogInfo(msg=['launch uwb_imu_fusion node']),
        config_file_arg,
        use_sim_time_arg,
        uwb_imu_fusion_node,
    ])

