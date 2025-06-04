import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty


class DualTeleop(Node):
    def __init__(self):
        super().__init__('dual_teleop_custom')
        self.pub1 = self.create_publisher(Twist, '/battlebot_1/cmd_vel', 10)
        self.pub2 = self.create_publisher(Twist, '/battlebot_2/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.read_key)

    def read_key(self):
        key = get_key()
        twist1 = Twist()
        twist2 = Twist()

        # robot1 control: w/a/s/d
        if key == 'w':
            twist1.linear.x = 0.5
        elif key == 's':
            twist1.linear.x = -0.5
        elif key == 'a':
            twist1.angular.z = 1.0
        elif key == 'd':
            twist1.angular.z = -1.0

        # robot2 control: i/j/k/l
        elif key == 'i':
            twist2.linear.x = 0.5
        elif key == 'k':
            twist2.linear.x = -0.5
        elif key == 'j':
            twist2.angular.z = 1.0
        elif key == 'l':
            twist2.angular.z = -1.0

        elif key == 'q':
            rclpy.shutdown()
            return

        if twist1.linear.x != 0.0 or twist1.angular.z != 0.0:
            self.pub1.publish(twist1)

        if twist2.linear.x != 0.0 or twist2.angular.z != 0.0:
            self.pub2.publish(twist2)


def get_key():
    fd = sys.stdin.fileno()
    settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, settings)
    return key


def main():
    rclpy.init()
    node = DualTeleop()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
