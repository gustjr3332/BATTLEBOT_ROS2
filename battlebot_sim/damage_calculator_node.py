import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
# --- 수정: ROS의 시간 모듈을 사용합니다 ---
from rclpy.time import Time, Duration

from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from transforms3d.euler import quat2euler
import math

class DamageCalculator(Node):
    def __init__(self):
        super().__init__('damage_calculator')

        # --- 수정: 파라미터 선언 부분에 hit_interval 추가 ---
        self.declare_parameter('hit_interval', 1.0 / 3.0) # 1초에 3번 데미지
        self.declare_parameter('initial_health', 100.0)
        self.declare_parameter('contact_damage_per_event', 1.0)
        self.declare_parameter('moving_obstacle_contact_damage', 5.0)
        self.declare_parameter('saw_blade_contact_damage', 5.0)
        self.declare_parameter('flip_damage_per_event', 99.0)
        self.declare_parameter('weapon_damage_saw', 10.0)
        self.declare_parameter('flip_threshold_roll_deg', 90.0)
        self.declare_parameter('flip_threshold_pitch_deg', 90.0)

        # 파라미터 값 가져오기
        self.initial_health = self.get_parameter('initial_health').value
        self.damage_config = {
            'contact_damage': self.get_parameter('contact_damage_per_event').value,
            'moving_obstacle_contact_damage': self.get_parameter('moving_obstacle_contact_damage').value,
            'saw_blade_contact_damage': self.get_parameter('saw_blade_contact_damage').value,
            'flip_damage': self.get_parameter('flip_damage_per_event').value,
            'weapon_damage_saw': self.get_parameter('weapon_damage_saw').value,
        }
        self.flip_threshold_roll_rad = math.radians(self.get_parameter('flip_threshold_roll_deg').value)
        self.flip_threshold_pitch_rad = math.radians(self.get_parameter('flip_threshold_pitch_deg').value)

        # --- 수정: ROS Duration 객체로 쿨다운 시간 설정 및 ROS 시간으로 초기화 ---
        hit_interval_sec = self.get_parameter('hit_interval').value
        self.hit_interval_duration = Duration(seconds=hit_interval_sec)
        
        # Robots
        self.robots = ["battlebot_1", "battlebot_2"]
        now = self.get_clock().now()
        self.robot_health = {name: self.initial_health for name in self.robots}
        self.is_destroyed = {name: False for name in self.robots}
        self.last_hit_time = {name: now for name in self.robots} # ROS 시간으로 초기화
        self.is_flipped = {name: False for name in self.robots}

        # Subscribers (중복 구독 정리)
        self.create_subscription(ContactsState, "/saw_blade_1/contact", self.process_saw_blade_contact, 10)
        self.create_subscription(ContactsState, "/saw_blade_2/contact", self.process_saw_blade_contact, 10)

        self.create_subscription(ContactsState, "/battlebot_1/contact", lambda msg: self.process_contact("battlebot_1", "battlebot_2", msg), 10)
        self.create_subscription(ContactsState, "/battlebot_2/contact", lambda msg: self.process_contact("battlebot_2", "battlebot_1", msg), 10)
        self.create_subscription(ContactsState, "/moving_obstacle/contact", lambda msg: self.process_contact("moving_obstacle", "battlebot_2", msg), 10)

        self.create_subscription(Imu, "/battlebot_1/imu", lambda msg: self.check_flip("battlebot_1", msg), 10)
        self.create_subscription(Imu, "/battlebot_2/imu", lambda msg: self.check_flip("battlebot_2", msg), 10)

        # Publishers
        self.health_pub = {name: self.create_publisher(Float32, f"/{name}/health", 10) for name in self.robots}
        self.timer = self.create_timer(0.5, self.publish_health)

        self.get_logger().info("[INIT] Battlesim started.")
        self.get_logger().info(f"[INIT] Damage Cooldown: {hit_interval_sec:.3f} seconds")
        self.get_logger().info(f"[INIT] battlebot_1 HP: {self.robot_health['battlebot_1']} | battlebot_2 HP: {self.robot_health['battlebot_2']}")

    def process_saw_blade_contact(self, msg):
        """톱날 충돌을 처리하는 별도 콜백 함수 (saw_blade_1, saw_blade_2 모두 처리)"""
        if msg.states:
            for state in msg.states:
                collision1_name = state.collision1_name
                collision2_name = state.collision2_name

                # 공격자와 타겟 식별
                attacker = None
                target = None

                if "saw_blade_1" in collision1_name or "saw_blade_1" in collision2_name:
                    attacker = "saw_blade_1"
                elif "saw_blade_2" in collision1_name or "saw_blade_2" in collision2_name:
                    attacker = "saw_blade_2"

                if "battlebot_1" in collision1_name or "battlebot_1" in collision2_name:
                    target = "battlebot_1"
                elif "battlebot_2" in collision1_name or "battlebot_2" in collision2_name:
                    target = "battlebot_2"
                
                # 유효한 공격자와 타겟이 식별되었을 경우에만 데미지 처리 함수 호출
                if attacker and target:
                    self.process_contact(attacker, target, msg)
    # ---------------------------------------------------------------


    def process_contact(self, attacker, target, msg):
        if self.is_destroyed.get(target, True):
            return

        # --- 수정: ROS 시간을 사용하여 쿨다운을 확인 ---
        now = self.get_clock().now()
        if now - self.last_hit_time[target] < self.hit_interval_duration:
            return

        if msg.states:
            for state in msg.states:
                # collision1_name과 collision2_name에 타겟이 있는지 정확히 확인
                target_in_collision = False
                if target in state.collision1_name and attacker in state.collision2_name:
                    target_in_collision = True
                elif target in state.collision2_name and attacker in state.collision1_name:
                    target_in_collision = True

                if not target_in_collision:
                    continue
                
                damage = 0.0

                if attacker == "moving_obstacle":
                    damage = self.damage_config['moving_obstacle_contact_damage']
                elif attacker.startswith("saw_blade"):
                    damage = self.damage_config['saw_blade_contact_damage']
                elif attacker.startswith("battlebot"):
                    # 무기 이름은 보통 링크 이름의 일부로 들어감
                    attacker_weapon_part = state.collision1_name if attacker in state.collision1_name else state.collision2_name
                    if "saw" in attacker_weapon_part:
                        damage = self.damage_config['weapon_damage_saw']
                    else:
                        damage = self.damage_config['contact_damage']

                if damage == 0.0:
                    continue

                self.robot_health[target] -= damage
                self.last_hit_time[target] = now # 마지막 공격 시간을 ROS 시간으로 업데이트
                
                if self.robot_health[target] <= 0.0:
                    self.robot_health[target] = 0.0
                    self.is_destroyed[target] = True
                    self.get_logger().warn(f"[DESTROYED] {target} has been destroyed!")
                    self.print_result()
                else:
                    self.get_logger().info(f"[HIT] {attacker} → {target} | Damage: {damage} | HP: {self.robot_health[target]:.1f}")
                
                self.publish_health()
                break # 한 번의 콜백에서 한 번의 데미지만 처리

    def check_flip(self, name, msg):
        if self.is_destroyed[name]:
            return

        orientation_q = msg.orientation
        q = [orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z]
        roll, pitch, _ = quat2euler(q, axes='sxyz')

        if abs(roll) > self.flip_threshold_roll_rad or abs(pitch) > self.flip_threshold_pitch_rad:
            if not self.is_flipped[name]:
                # 뒤집힘 데미지에도 쿨다운을 적용하려면 last_hit_time을 확인
                now = self.get_clock().now()
                if now - self.last_hit_time[name] < self.hit_interval_duration:
                    return

                self.robot_health[name] -= self.damage_config['flip_damage']
                self.last_hit_time[name] = now # 뒤집힘 데미지도 쿨다운에 포함

                if self.robot_health[name] <= 0:
                    self.robot_health[name] = 0
                    self.is_destroyed[name] = True
                    self.get_logger().warn(f"[DESTROYED] {name} has been destroyed!")
                    self.print_result()
                else:
                    self.get_logger().warn(f"[HIT] {name} flipped! Damage: {self.damage_config['flip_damage']} | HP: {self.robot_health[name]:.1f}")
                
                self.is_flipped[name] = True
                self.publish_health()
        else:
            self.is_flipped[name] = False

    def publish_health(self):
        for name in self.robots:
            msg = Float32()
            msg.data = self.robot_health[name]
            self.health_pub[name].publish(msg)

    def print_result(self):
        # 두 로봇이 모두 파괴되었는지 확인
        if all(self.is_destroyed.values()):
            winner = None
            if self.robot_health["battlebot_1"] > self.robot_health["battlebot_2"]:
                winner = "battlebot_1"
            elif self.robot_health["battlebot_2"] > self.robot_health["battlebot_1"]:
                winner = "battlebot_2"

            self.get_logger().info("[END] Simulation ended.")
            if winner:
                self.get_logger().info(f"[RESULT] Winner: {winner}")
            else:
                self.get_logger().info("[RESULT] Draw.")
            # rclpy.shutdown() # 필요시 노드 종료

        # 한쪽만 파괴되었을 때
        elif self.is_destroyed["battlebot_1"]:
            self.get_logger().info("[END] Simulation ended.")
            self.get_logger().info("[RESULT] Winner: battlebot_2")
        elif self.is_destroyed["battlebot_2"]:
            self.get_logger().info("[END] Simulation ended.")
            self.get_logger().info("[RESULT] Winner: battlebot_1")


def main(args=None):
    rclpy.init(args=args)
    node = DamageCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()