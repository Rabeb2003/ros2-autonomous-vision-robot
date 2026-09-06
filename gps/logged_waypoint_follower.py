#!/usr/bin/env python3
"""
logged_waypoint_follower.py

Charge un fichier YAML de waypoints (format produit par
gps_waypoint_logger.py) et envoie la liste complete au robot via
l'action /follow_gps_waypoints (nav2_simple_commander.BasicNavigator).

Utilisation:
    ros2 run <pkg> logged_waypoint_follower /home/you/gps_waypoints.yaml
"""

import sys
import yaml
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geographic_msgs.msg import GeoPose


class LoggedWaypointFollower(Node):

    def __init__(self, yaml_path: str):
        super().__init__('logged_waypoint_follower')

        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)

        waypoints_data = data.get('waypoints', [])
        if not waypoints_data:
            self.get_logger().error(f'Aucun waypoint trouve dans {yaml_path}')
            return

        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active(localizer='ekf_filter_node_map')

        goals = []
        for wp in waypoints_data:
            goal = GeoPose()
            goal.position.latitude = float(wp['latitude'])
            goal.position.longitude = float(wp['longitude'])
            goal.position.altitude = 0.0
            yaw = float(wp.get('yaw', 0.0))
            # Conversion yaw -> quaternion (rotation autour de Z uniquement)
            import math
            goal.orientation.z = math.sin(yaw / 2.0)
            goal.orientation.w = math.cos(yaw / 2.0)
            goals.append(goal)

        self.get_logger().info(f'{len(goals)} waypoints charges depuis {yaml_path}. Depart...')
        self.navigator.followGpsWaypoints(goals)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(
                    f'Waypoint courant: {feedback.current_waypoint}/{len(goals)}',
                    throttle_duration_sec=5.0
                )

        result = self.navigator.getResult()
        self.get_logger().info(f'Parcours termine. Resultat: {result}')


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print('Usage: logged_waypoint_follower <chemin_vers_waypoints.yaml>')
        return

    node = LoggedWaypointFollower(sys.argv[1])
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
