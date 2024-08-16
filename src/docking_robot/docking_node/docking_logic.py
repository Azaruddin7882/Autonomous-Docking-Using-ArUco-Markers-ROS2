import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped

class DockingLogic(Node):
    def __init__(self):
        super().__init__('docking_logic')
        self.subscription = self.create_subscription(
            PoseStamped,
            'aruco_marker_pose',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('Docking Logic Node has been started.')

    def listener_callback(self, msg):
        cmd = Twist()

        self.get_logger().info(f'Received marker pose: x={msg.pose.position.x}, y={msg.pose.position.y}, z={msg.pose.position.z}')
        if abs(msg.pose.position.x) > 0.1:
            cmd.linear.x = 0.1
            self.get_logger().info(f'Moving forward: cmd.linear.x={cmd.linear.x}')
        else:
            cmd.linear.x = 0.0
            self.get_logger().info('Stopping forward movement.')

        if abs(msg.pose.position.y) > 0.1:
            cmd.angular.z = 0.1
            self.get_logger().info(f'Rotating: cmd.angular.z={cmd.angular.z}')
        else:
            cmd.angular.z = 0.0
            self.get_logger().info('Stopping rotation.')

        self.publisher.publish(cmd)
        self.get_logger().info(f'Published command: linear.x={cmd.linear.x}, angular.z={cmd.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = DockingLogic()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
