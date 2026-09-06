#!/usr/bin/env python3
"""
interactive_waypoint_follower.py

Equivalent du noeud de la demo Nav2 (image 2 des captures fournies) :
ecoute les points cliques sur Mapviz (topic /clicked_point, repere wgs84)
et envoie le robot tracteur-remorque a ce point GPS via l'action
/follow_gps_waypoints, deja fournie nativement par nav2_waypoint_follower
(pas besoin de recoder le serveur d'action, seulement le declencheur).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_msgs.srv import GetCostmap


class InteractiveWaypointFollower(Node):

    def __init__(self):
        super().__init__('interactive_waypoint_follower')

        self.navigator = BasicNavigator()

        # Attend que Nav2 (bt_navigator, waypoint_follower, etc.) soit actif
        self.navigator.waitUntilNav2Active(localizer='ekf_filter_node_map')

        self.click_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10
        )

        self.get_logger().info(
            'interactive_waypoint_follower pret. '
            'En attente de clics sur /clicked_point (repere wgs84).'
        )

    def clicked_point_callback(self, msg: PointStamped):
        # Sur Mapviz, quand le point_click_publisher est configure sur le
        # repere wgs84, PointStamped.point.x = longitude, point.y = latitude
        latitude = msg.point.y
        longitude = msg.point.x

        self.get_logger().info(
            f'Point clique recu: lat={latitude:.6f}, lon={longitude:.6f}. '
            f'Envoi du waypoint GPS...'
        )

        from geographic_msgs.msg import GeoPose

        goal = GeoPose()
        goal.position.latitude = latitude
        goal.position.longitude = longitude
        goal.position.altitude = 0.0
        goal.orientation.w = 1.0  # cap non specifie: identite

        self.navigator.followGpsWaypoints([goal])


def main(args=None):
    rclpy.init(args=args)
    node = InteractiveWaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
