#!/usr/bin/env python3
"""
gps_waypoint_logger.py

Equivalent du "gps_waypoint_logger" de la demo Nav2. Ecoute /gps/fix et
/imu, et sur declenchement (topic /log_waypoint, std_msgs/Empty), ajoute
la position et le cap courants a un fichier YAML de waypoints. Simplifie
par rapport a la demo (pas de GUI Tkinter) pour rester portable en
environnement conteneurise / sans affichage.

Utilisation:
    ros2 run <pkg> gps_waypoint_logger --ros-args -p output_path:=/home/you/waypoints.yaml
    # puis, a chaque position a enregistrer :
    ros2 topic pub -1 /log_waypoint std_msgs/msg/Empty {}
"""

import math
import yaml
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Empty


class GpsWaypointLogger(Node):

    def __init__(self):
        super().__init__('gps_waypoint_logger')

        self.declare_parameter('output_path', os.path.expanduser('~/gps_waypoints.yaml'))
        self.output_path = self.get_parameter('output_path').value

        self.last_fix = None
        self.last_yaw = 0.0
        self.waypoints = []

        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Empty, '/log_waypoint', self.log_callback, 10)

        self.get_logger().info(
            f'gps_waypoint_logger pret. Sortie: {self.output_path}. '
            f'Publiez sur /log_waypoint (std_msgs/Empty) pour enregistrer un point.'
        )

    def gps_callback(self, msg: NavSatFix):
        self.last_fix = msg

    def imu_callback(self, msg: Imu):
        # Extraction du cap (yaw) depuis le quaternion, convention ENU
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.last_yaw = math.atan2(siny_cosp, cosy_cosp)

    def log_callback(self, _msg: Empty):
        if self.last_fix is None:
            self.get_logger().warn('Pas encore de fix GPS recu, rien a enregistrer.')
            return

        entry = {
            'latitude': float(self.last_fix.latitude),
            'longitude': float(self.last_fix.longitude),
            'yaw': float(self.last_yaw),
        }
        self.waypoints.append(entry)

        with open(self.output_path, 'w') as f:
            yaml.dump({'waypoints': self.waypoints}, f, default_flow_style=False)

        self.get_logger().info(
            f'Waypoint #{len(self.waypoints)} enregistre: '
            f'lat={entry["latitude"]:.6f}, lon={entry["longitude"]:.6f}, '
            f'yaw={math.degrees(entry["yaw"]):.1f} deg'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GpsWaypointLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
