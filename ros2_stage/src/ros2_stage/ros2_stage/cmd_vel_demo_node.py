import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelDemoNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_demo_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.steps = [
            ('ligne droite', 5.0, 0.35, 0.0),
            ('virage lent gauche', 5.0, 0.30, 0.12),
            ('recuperation ligne droite', 6.0, 0.30, 0.0),
            ('virage lent droite', 5.0, 0.30, -0.12),
            ('recuperation ligne droite', 6.0, 0.30, 0.0),
            ('petite rotation sur place', 1.0, 0.0, 0.20),
            ('recuperation finale', 8.0, 0.25, 0.0),
            ('stop', 2.0, 0.0, 0.0),
        ]
        self.step_index = 0
        self.step_start_time = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.on_timer)

        self.get_logger().info('Demo cmd_vel started')
        self.log_current_step()

    def on_timer(self):
        if self.step_index >= len(self.steps):
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info('Demo finished')
            rclpy.shutdown()
            return

        name, duration, v, omega = self.steps[self.step_index]
        elapsed = (self.get_clock().now() - self.step_start_time).nanoseconds * 1e-9

        if elapsed >= duration:
            self.step_index += 1
            self.step_start_time = self.get_clock().now()
            if self.step_index < len(self.steps):
                self.log_current_step()
            return

        self.publish_cmd(v, omega)

    def publish_cmd(self, v: float, omega: float):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = omega
        self.publisher.publish(msg)

    def log_current_step(self):
        name, duration, v, omega = self.steps[self.step_index]
        self.get_logger().info(
            f'Step: {name}, duration={duration:.1f}s, v={v:.2f}, omega={omega:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelDemoNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish_cmd(0.0, 0.0)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
