import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import traceback

# 조작키 안내 문구
MSG = """
=====================================================
          Dual Battlebot Teleop Controller
=====================================================
       (Bot 1: BLUE)        (Bot 2: RED)
-----------------------------------------------------
Moving around:       Moving around:
        w                   ▲
   a    s    d         ◀    ▼    ▶
(forward/backward)   (left/right turn)

Emergency Stop:      Emergency Stop:
        f                   ;
-----------------------------------------------------
Hold down a key to move. Release to stop.
Press 'q' or Ctrl+C to quit.
=====================================================
"""


class DualTeleop(Node):
    def __init__(self):
        super().__init__('dual_teleop_custom')
        self.pub1 = self.create_publisher(Twist, '/battlebot_1/cmd_vel', 10)
        self.pub2 = self.create_publisher(Twist, '/battlebot_2/cmd_vel', 10)

        self.linear_speed_1 = 0.5
        self.linear_speed_2 = 0.5
        self.max_linear_speed = 1.5
        self.min_linear_speed = 0.1
        self.fixed_angular_speed = 1.0

        self.timer = self.create_timer(0.1, self.read_key)

    def read_key(self):
        key = get_key()
        if key is None:
            if rclpy.ok():
                self.get_logger().error("Error in get_key() or shutdown initiated from there. Stopping teleop.")
                if self.timer:
                    self.timer.cancel()
                if not rclpy.is_shutdown():
                    rclpy.shutdown()
            return

        twist1 = Twist()
        twist2 = Twist()

        # battlebot_1: w/a/s/d
        if key == 'w':
            twist1.linear.x = self.linear_speed_1
        elif key == 's':
            twist1.linear.x = -self.linear_speed_1
        elif key == 'a':
            twist1.angular.z = self.fixed_angular_speed
        elif key == 'd':
            twist1.angular.z = -self.fixed_angular_speed

        # battlebot_2: i/j/k/l
        elif key == 'i':
            twist2.linear.x = self.linear_speed_2
        elif key == 'k':
            twist2.linear.x = -self.linear_speed_2
        elif key == 'j':
            twist2.angular.z = self.fixed_angular_speed
        elif key == 'l':
            twist2.angular.z = -self.fixed_angular_speed

        # 속도 조절 - battlebot_1
        elif key == 'q':
            if self.linear_speed_1 < self.max_linear_speed:
                self.linear_speed_1 += 0.1
                self.get_logger().info(f"battlebot_1 linear speed increased to {self.linear_speed_1:.2f}")
            else:
                self.get_logger().warn("battlebot_1 linear speed at maximum (1.5 m/s)")
        elif key == 'e':
            self.linear_speed_1 = max(self.min_linear_speed, self.linear_speed_1 - 0.1)
            self.get_logger().info(f"battlebot_1 linear speed decreased to {self.linear_speed_1:.2f}")

        # 속도 조절 - battlebot_2
        elif key == 'u':
            if self.linear_speed_2 < self.max_linear_speed:
                self.linear_speed_2 += 0.1
                self.get_logger().info(f"battlebot_2 linear speed increased to {self.linear_speed_2:.2f}")
            else:
                self.get_logger().warn("battlebot_2 linear speed at maximum (1.5 m/s)")
        elif key == 'o':
            self.linear_speed_2 = max(self.min_linear_speed, self.linear_speed_2 - 0.1)
            self.get_logger().info(f"battlebot_2 linear speed decreased to {self.linear_speed_2:.2f}")

        # 종료
        elif key == 'x':
            self.get_logger().info("'x' pressed, initiating shutdown.")
            if self.timer:
                self.timer.cancel()
            rclpy.shutdown()
            return

        if twist1.linear.x != 0.0 or twist1.angular.z != 0.0:
            self.pub1.publish(twist1)
        if twist2.linear.x != 0.0 or twist2.angular.z != 0.0:
            self.pub2.publish(twist2)


def get_key():
    fd = sys.stdin.fileno()
    settings = None
    try:
        settings = termios.tcgetattr(fd)
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    except termios.error as e:
        print(f"TERMIOS ERROR in get_key: {e}")
        print("Is this script running in a proper terminal?")
        if rclpy.ok() and not rclpy.is_shutdown():
            rclpy.shutdown()
        return None
    except Exception as e:
        print(f"Unexpected error in get_key: {e}")
        traceback.print_exc()
        if rclpy.ok() and not rclpy.is_shutdown():
            rclpy.shutdown()
        return None
    finally:
        if settings:
            termios.tcsetattr(fd, termios.TCSADRAIN, settings)
    return key


def main():
    rclpy.init()
    node = None
    try:
        node = DualTeleop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("KeyboardInterrupt, shutting down.")
    except Exception as e:
        if node:
            node.get_logger().error(f"An unhandled exception occurred: {e}")
        else:
            print(f"An unhandled exception occurred before node initialization: {e}")
        traceback.print_exc()
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok() and not rclpy.is_shutdown():
            rclpy.shutdown()
        print("--- Script execution finished or an error occurred ---")
        print("Press Enter to close this window...")
        input()


if __name__ == '__main__':
    main()