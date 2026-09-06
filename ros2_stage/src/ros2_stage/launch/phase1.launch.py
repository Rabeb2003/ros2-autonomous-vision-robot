import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = get_package_share_directory('ros2_stage')

    urdf_file = PathJoinSubstitution([
        FindPackageShare('ros2_stage'),
        'urdf',
        'diff_trailer.urdf.xacro',
    ])
    rviz_config = os.path.join(package_share, 'rviz', 'phase1.rviz')

    hitch_offset = LaunchConfiguration('hitch_offset')
    trailer_length = LaunchConfiguration('trailer_length')
    beta_limit_deg = LaunchConfiguration('beta_limit_deg')
    rviz = LaunchConfiguration('rviz')

    robot_description = {
        'robot_description': Command([
            'xacro ',
            urdf_file,
            ' hitch_offset:=', hitch_offset,
            ' trailer_length:=', trailer_length,
        ])
    }

    return LaunchDescription([
        DeclareLaunchArgument('hitch_offset', default_value='0.45'),
        DeclareLaunchArgument('trailer_length', default_value='0.85'),
        DeclareLaunchArgument('beta_limit_deg', default_value='70.0'),
        DeclareLaunchArgument('rviz', default_value='true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
            output='screen'),

        Node(
            package='ros2_stage',
            executable='kinematic_trailer_node',
            parameters=[
                {'hitch_offset': hitch_offset},
                {'trailer_length': trailer_length},
                {'beta_limit_deg': beta_limit_deg},
            ],
            output='screen'),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(rviz),
            output='screen'),
    ])
