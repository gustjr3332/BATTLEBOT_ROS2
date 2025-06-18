import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

class MovingObstacleController(Node):
    def __init__(self):
        super().__init__('moving_obstacle_controller')

        self.publisher_ = self.create_publisher(Twist, '/moving_obstacle/cmd_vel', 10)
        self.shutdown_sub = self.create_subscription(
            Empty,
            '/obstacle_destroyed',
            self.shutdown_callback,
            10)
        self.get_logger().info("Obstacle controller initialized and waiting for shutdown signal.")
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
            
    def shutdown_callback(self, msg):
        self.get_logger().warn("Shutdown signal received, stopping obstacle controller.")
        
        # 움직임을 멈추기 위해 마지막으로 빈 Twist 메시지를 보냄
        stop_msg = Twist()
        self.publisher_.publish(stop_msg)
        
        # 노드를 소멸시키고 rclpy를 종료하여 프로세스를 끝냄
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MovingObstacleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
