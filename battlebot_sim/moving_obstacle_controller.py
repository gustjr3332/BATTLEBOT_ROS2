import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MovingObstacleController(Node):
    def __init__(self):
        super().__init__('moving_obstacle_controller')

        self.publisher_ = self.create_publisher(Twist, '/moving_obstacle/cmd_vel', 10)
        self.direction = 1.0
        self.position = 0.0
        self.speed = 0.5  # m/s
        self.max_distance = 3.0  # 🔄 왕복 거리의 절반 (총 10m)
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.direction * self.speed
        self.publisher_.publish(msg)

        self.position += self.direction * self.speed * self.timer_period
        if abs(self.position) >= self.max_distance:
            self.direction *= -1.0
            self.get_logger().info(f'flip direction: {self.direction}')


def main(args=None):
    rclpy.init(args=args)
    node = MovingObstacleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
