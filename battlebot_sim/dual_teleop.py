import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import traceback # 오류 출력을 위해 추가


class DualTeleop(Node):
    def __init__(self):
        super().__init__('dual_teleop_custom')
        self.pub1 = self.create_publisher(Twist, '/battlebot_1/cmd_vel', 10)
        self.pub2 = self.create_publisher(Twist, '/battlebot_2/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.read_key)

    def read_key(self):
        key = get_key()
        # get_key에서 오류 발생 시 (None 반환 가정)
        if key is None:
            if rclpy.ok(): # rclpy가 아직 실행 중이라면 메시지 출력
                self.get_logger().error("Error in get_key() or shutdown initiated from there. Stopping teleop.")
                # 필요한 경우 타이머 중지 또는 노드 종료 로직 추가
                if self.timer:
                    self.timer.cancel()
                if not rclpy.is_shutdown(): # 명시적으로 shutdown 호출
                    rclpy.shutdown()
            return
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
            self.get_logger().info("'q' pressed, initiating shutdown.")
            if self.timer:
                self.timer.cancel() # 타이머 중지
            rclpy.shutdown()
            return

        #
        if twist1.linear.x != 0.0 or twist1.angular.z != 0.0:
            self.pub1.publish(twist1)

        if twist2.linear.x != 0.0 or twist2.angular.z != 0.0:
            self.pub2.publish(twist2)

def get_key():
    fd = sys.stdin.fileno()
    settings = None # settings 변수 초기화
    try:
        settings = termios.tcgetattr(fd)
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    except termios.error as e: # termios 관련 에러 명시적 처리
        print(f"TERMIOS ERROR in get_key: {e}")
        print("Is this script running in a proper terminal?")
        # 오류 발생 시 None을 반환하거나, rclpy.shutdown() 호출
        if rclpy.ok() and not rclpy.is_shutdown(): # 중복 shutdown 방지
             rclpy.shutdown()
        return None # 오류 발생 시 None 반환
    except Exception as e: # 다른 예기치 않은 에러 처리
        print(f"Unexpected error in get_key: {e}")
        traceback.print_exc()
        if rclpy.ok() and not rclpy.is_shutdown():
            rclpy.shutdown()
        return None
    finally:
        if settings: # settings가 성공적으로 받아와졌을 때만 복원 시도
            termios.tcsetattr(fd, termios.TCSADRAIN, settings)
    return key


def main():
    rclpy.init()
    node = None # node 변수 초기화
    try:
        node = DualTeleop()
        rclpy.spin(node)
    except KeyboardInterrupt: # Ctrl+C 로 종료 시
        if node:
            node.get_logger().info("KeyboardInterrupt, shutting down.")
    except Exception as e: # 그 외 모든 예외 처리
        if node: # node가 생성되었으면 logger 사용, 아니면 print
            node.get_logger().error(f"An unhandled exception occurred: {e}")
        else:
            print(f"An unhandled exception occurred before node initialization: {e}")
        traceback.print_exc() # 전체 에러 트레이스백 출력
    finally:
        if node:
            node.destroy_node() # 노드 소멸
        if rclpy.ok() and not rclpy.is_shutdown(): # 이미 shutdown 되지 않았으면 shutdown
            rclpy.shutdown()
        
        print("--- Script execution finished or an error occurred ---")
        print("Press Enter to close this window...")
        input() # 사용자가 Enter를 누를 때까지 창을 열어둠

if __name__ == '__main__':
    main()
