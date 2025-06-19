import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import traceback

# [추가] 터미널에 출력될 조작키 안내 문구
MSG = """
===================================================================
                Dual Battlebot Teleop Controller
===================================================================
            (Bot 1: BLUE)        (Bot 2: RED)
-------------------------------------------------------------------
            Moving:                  Moving:
                 w                      i
            a    s    d            j     k     l

            Speed Control:           Speed Control:
            q/e (UP/DOWN)          u/o (UP/DOWN)

            STOP:                    STOP:
                    f                      ;
------------------------------------------------------------------
IMPORTANT: Press a key once to set velocity.
           Robot will keep moving with the last command.
           Press the STOP key to stop each robot.

Press 'x' to quit.
===================================================================
"""

class DualTeleop(Node):
    def __init__(self):
        super().__init__('dual_teleop_custom')
        self.pub1 = self.create_publisher(Twist, '/battlebot_1/cmd_vel', 10)
        self.pub2 = self.create_publisher(Twist, '/battlebot_2/cmd_vel', 10)

        self.linear_speed_1 = 1.0
        self.linear_speed_2 = 1.0
        self.max_linear_speed = 2.0
        self.min_linear_speed = 0.5
        self.fixed_angular_speed = 1.0
        
        # [추가] 노드 시작 시 조작키 안내 출력
        self.get_logger().info("Teleop node has started. Press keys in this terminal.")
        print(MSG)

        self.timer = self.create_timer(0.1, self.read_key)

    def read_key(self):
        key = get_key()
        if key is None:
            # 이 부분은 에러 발생 시 처리 로직으로, 그대로 둡니다.
            if rclpy.ok():
                self.get_logger().error("Error in get_key(). Stopping teleop.")
                if self.timer:
                    self.timer.cancel()
                rclpy.shutdown()
            return

        twist1 = Twist()
        twist2 = Twist()

        # battlebot_1: w/a/s/d
        if key == 'w':
            twist1.linear.x = self.linear_speed_1
            self.pub1.publish(twist1)
        elif key == 's':
            twist1.linear.x = -self.linear_speed_1
            self.pub1.publish(twist1)
        elif key == 'a':
            twist1.angular.z = self.fixed_angular_speed
            self.pub1.publish(twist1)
        elif key == 'd':
            twist1.angular.z = -self.fixed_angular_speed
            self.pub1.publish(twist1)
        # [추가] battlebot_1 정지
        elif key == 'f':
            self.pub1.publish(twist1) # 속도가 0인 Twist 메시지 발행

        # battlebot_2: i/j/k/l
        elif key == 'i':
            twist2.linear.x = self.linear_speed_2
            self.pub2.publish(twist2)
        elif key == 'k':
            twist2.linear.x = -self.linear_speed_2
            self.pub2.publish(twist2)
        elif key == 'j':
            twist2.angular.z = self.fixed_angular_speed
            self.pub2.publish(twist2)
        elif key == 'l':
            twist2.angular.z = -self.fixed_angular_speed
            self.pub2.publish(twist2)
        # [추가] battlebot_2 정지 (한글 'ㅣ'키 대신 세미콜론)
        elif key == ';':
            self.pub2.publish(twist2) # 속도가 0인 Twist 메시지 발행

        # 속도 조절 - battlebot_1
        elif key == 'q':
            self.linear_speed_1 = min(self.max_linear_speed, self.linear_speed_1 + 0.1)
            self.get_logger().info(f"battlebot_1 linear speed set to {self.linear_speed_1:.2f}")
        elif key == 'e':
            self.linear_speed_1 = max(self.min_linear_speed, self.linear_speed_1 - 0.1)
            self.get_logger().info(f"battlebot_1 linear speed set to {self.linear_speed_1:.2f}")

        # 속도 조절 - battlebot_2
        elif key == 'u':
            self.linear_speed_2 = min(self.max_linear_speed, self.linear_speed_2 + 0.1)
            self.get_logger().info(f"battlebot_2 linear speed set to {self.linear_speed_2:.2f}")
        elif key == 'o':
            self.linear_speed_2 = max(self.min_linear_speed, self.linear_speed_2 - 0.1)
            self.get_logger().info(f"battlebot_2 linear speed set to {self.linear_speed_2:.2f}")

        # 종료
        elif key == 'x':
            self.get_logger().info("'x' pressed, initiating shutdown.")
            self.pub1.publish(Twist()) # 종료 시 두 로봇 모두 정지
            self.pub2.publish(Twist())
            if self.timer:
                self.timer.cancel()
            rclpy.shutdown()
            return
        
        # [수정] 아래 블록은 각 if/elif 문 안으로 이동하여, 키 입력이 있을 때만 발행하도록 변경


def get_key():
    # 이 함수는 그대로 유지하되, 에러 처리 시 rclpy.ok()를 확인하는 것이 좋습니다.
    # 또한 Foxy에는 is_shutdown이 없으므로 해당 부분은 제거해야 합니다.
    fd = sys.stdin.fileno()
    settings = None
    try:
        settings = termios.tcgetattr(fd)
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    except termios.error as e:
        print(f"TERMIOS ERROR in get_key: {e}")
        return None
    except Exception as e:
        print(f"Unexpected error in get_key: {e}")
        traceback.print_exc()
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
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        if node:
            node.get_logger().info("Shutting down.")
    except Exception as e:
        if node:
            node.get_logger().error(f"An unhandled exception occurred: {e}")
        else:
            print(f"An unhandled exception occurred before node initialization: {e}")
        traceback.print_exc()
    finally:
        # 종료 시 두 로봇 모두에게 마지막으로 정지 명령을 보냄
        if node:
            if rclpy.ok(): # 노드가 아직 살아있으면 정지 명령 발행
                node.pub1.publish(Twist())
                node.pub2.publish(Twist())
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        # 터미널 창 유지를 위한 input()은 수동 실행 시 불편할 수 있어 제거했습니다.

if __name__ == '__main__':
    main()