import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SawBladeController(Node):
    def __init__(self):
        super().__init__('saw_blade_controller')

        self.publisher_1 = self.create_publisher(Twist, '/saw_blade_1/cmd_vel', 10)
        self.publisher_2 = self.create_publisher(Twist, '/saw_blade_2/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.speed = 10.0  # rad/s

    def timer_callback(self):
        msg = Twist()
        msg.angular.z = self.speed
        self.publisher_1.publish(msg)
        self.publisher_2.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SawBladeController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
