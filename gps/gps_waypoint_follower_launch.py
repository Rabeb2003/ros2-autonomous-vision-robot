from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('tractor_trailer_nav')  # adapter au nom reel du package
    mapviz_config = os.path.join(pkg_share, 'config', 'mapviz_silverstone.yaml')

    # Mapviz : visualisation GPS + chemin + clic pour waypoint
    mapviz = Node(
        package='mapviz',
        executable='mapviz',
        name='mapviz',
        arguments=['-config', mapviz_config],
        output='screen'
    )

    # Suivi interactif : ecoute les clics Mapviz, envoie le robot au point GPS clique
    interactive_follower = Node(
        package='tractor_trailer_nav',  # adapter au nom reel du package
        executable='interactive_waypoint_follower',
        name='interactive_waypoint_follower',
        output='screen'
    )

    return LaunchDescription([
        # Demarre apres la pile GPS + Nav2 (voir gps_localization_launch.py,
        # deja sequence a 6.0/6.5/7.0/7.5s) pour que /follow_gps_waypoints
        # et /fromLL soient deja disponibles.
        TimerAction(period=10.0, actions=[mapviz]),
        TimerAction(period=10.5, actions=[interactive_follower]),
    ])
