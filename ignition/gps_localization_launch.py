from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('diff_robot')  # adapted for diff_robot package

    world_path = os.path.join(pkg_share, 'world', 'silverstone_track.world')
    bridge_config = os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml')
    ekf_config = os.path.join(pkg_share, 'config', 'gps_ekf.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'gps_nav.rviz')

    # 1. Simulateur Gazebo Fortress
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    # 2. Pont ROS 2 <-> Gazebo : doit démarrer tôt, les EKF en dépendent
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    # 3. Spawn du robot dans Gazebo Fortress (remplace spawn_entity.py de Classic)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'tractor_trailer', '-z', '0.1'],
        output='screen'
    )

    # 4. EKF locale (odom -> base_footprint)
    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[ekf_config],
        remappings=[('odometry/filtered', 'odometry/local')]
    )

    # 5. navsat_transform_node : doit démarrer APRÈS que le GPS ait eu le
    #    temps de stabiliser sa covariance (cf. bruit gaussien non nul
    #    ajouté au capteur, fichier 2/8) -> décalage volontaire
    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('gps/fix', 'gps/fix'),
            ('imu', 'imu'),
            ('odometry/filtered', 'odometry/global'),
        ]
    )

    # 6. EKF globale (map -> odom), démarre après navsat_transform
    ekf_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[ekf_config],
        remappings=[('odometry/filtered', 'odometry/global')]
    )

    # 7. RViz, en dernier, une fois la TF map->odom->base_footprint stable
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        TimerAction(period=2.0, actions=[bridge]),
        TimerAction(period=3.0, actions=[spawn_robot]),
        TimerAction(period=6.0, actions=[ekf_local]),
        TimerAction(period=6.5, actions=[navsat_transform]),
        TimerAction(period=7.0, actions=[ekf_global]),
        TimerAction(period=7.5, actions=[rviz]),
    ])
