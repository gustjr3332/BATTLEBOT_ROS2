import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import ApplyJointEffort
from builtin_interfaces.msg import Duration

class SpinSaw(Node):
    def __init__(self):
        super().__init__('spin_saw')
        self.cli = self.create_client(ApplyJointEffort, '/gazebo/apply_joint_effort')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스 대기 중...')

        self.req = ApplyJointEffort.Request()

    def send_effort(self, effort=5.0, duration_sec=0.1):
        self.req.joint_name = 'blade_joint'  # 회전시킬 조인트명
        self.req.effort = effort
        self.req.start_time.sec = 0
        self.req.start_time.nanosec = 0
        self.req.duration.sec = int(duration_sec)
        self.req.duration.nanosec = int((duration_sec - int(duration_sec)) * 1e9)

        self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    node = SpinSaw()

    try:
        while rclpy.ok():
            node.send_effort()
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
