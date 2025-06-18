import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time, Duration

from gazebo_msgs.msg import ContactsState
from gazebo_msgs.srv import DeleteEntity
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Empty
from transforms3d.euler import quat2euler
import math

class DamageCalculator(Node):
    COLOR_GREEN_BOLD = '\033[1;32m'
    COLOR_RESET = '\033[0m'

    def __init__(self):
        super().__init__('damage_calculator')

        # 파라미터 선언
        self.declare_parameter('hit_interval', 0.5)
        self.declare_parameter('initial_health', 100.0)
        self.declare_parameter('contact_damage', 5.0)
        self.declare_parameter('obstacle_initial_health', 50.0)
        self.declare_parameter('damage_to_obstacle', 5.0)
        self.declare_parameter('obstacle_contact_damage', 1.0)
        self.declare_parameter('flip_damage', 99.0)
        self.declare_parameter('saw_blade_contact_damage', 5.0)
        self.declare_parameter('heal_amount', 20.0)
        self.declare_parameter('flip_threshold_roll_deg', 90.0)
        self.declare_parameter('flip_threshold_pitch_deg', 90.0)
        
        # 파라미터 값 가져오기
        self.initial_health = self.get_parameter('initial_health').value
        self.obstacle_initial_health = self.get_parameter('obstacle_initial_health').value
        self.damage_config = {
            'contact': self.get_parameter('contact_damage').value,
            'to_obstacle': self.get_parameter('damage_to_obstacle').value,
            'from_obstacle': self.get_parameter('obstacle_contact_damage').value,
            'flip': self.get_parameter('flip_damage').value,
            'saw_blade_contact_damage': self.get_parameter('saw_blade_contact_damage').value,
            'heal_amount': self.get_parameter('heal_amount').value,
        }
        self.flip_threshold_roll_rad = math.radians(self.get_parameter('flip_threshold_roll_deg').value)
        self.flip_threshold_pitch_rad = math.radians(self.get_parameter('flip_threshold_pitch_deg').value)
        self.hit_interval_duration = Duration(seconds=self.get_parameter('hit_interval').value)
        
        # 엔티티 상태 초기화
        self.entities = ["battlebot_1", "battlebot_2", "moving_obstacle"]
        now = self.get_clock().now()
        self.health = {
            "battlebot_1": self.initial_health,
            "battlebot_2": self.initial_health,
            "moving_obstacle": self.obstacle_initial_health
        }
        self.is_destroyed = {name: False for name in self.entities}
        self.last_hit_time = {name: now for name in self.entities}
        self.is_flipped = {"battlebot_1": False, "battlebot_2": False}
        self.healing_packs_used = {"healing_pack_1": False, "healing_pack_2": False}

        # 구독자 설정
        self.create_subscription(ContactsState, "/saw_blade_1/contact", self.process_external_saw_contact, 10)
        self.create_subscription(ContactsState, "/saw_blade_2/contact", self.process_external_saw_contact, 10)
        self.create_subscription(ContactsState, "/battlebot_1/contact", lambda msg: self.process_bot_contact("battlebot_1", msg), 10)
        self.create_subscription(ContactsState, "/battlebot_2/contact", lambda msg: self.process_bot_contact("battlebot_2", msg), 10)
        self.create_subscription(ContactsState, "/moving_obstacle/contact", self.process_obstacle_contact, 10)
        self.create_subscription(Imu, "/battlebot_1/imu", lambda msg: self.check_flip("battlebot_1", msg), 10)
        self.create_subscription(Imu, "/battlebot_2/imu", lambda msg: self.check_flip("battlebot_2", msg), 10)
        self.create_subscription(ContactsState, "/healing_pack_1/contact", lambda msg: self.process_heal_contact("healing_pack_1", msg), 10)
        self.create_subscription(ContactsState, "/healing_pack_2/contact", lambda msg: self.process_heal_contact("healing_pack_2", msg), 10)

        # 발행자 설정
        self.health_pubs = {name: self.create_publisher(Float32, f"/{name}/health", 10) for name in self.entities}
        self.obstacle_destroyed_pub = self.create_publisher(Empty, "/obstacle_destroyed", 10)
        self.timer = self.create_timer(0.5, self.publish_health)

        # Gazebo 엔티티 삭제 서비스 클라이언트
        self.delete_entity_client = self.create_client(DeleteEntity, '/delete_entity')
        while not self.delete_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Gazebo /delete_entity service not available, waiting again...')

        self.get_logger().info("Damage Calculator Initialized.")
        self.get_logger().info(f"battlebot_1 HP: {self.health['battlebot_1']} | battlebot_2 HP: {self.health['battlebot_2']} | Obstacle HP: {self.health['moving_obstacle']}")

    def process_external_saw_contact(self, msg):
        if not msg.states: return
        coll_str = str(msg.states)
        attacker = "saw_blade_1" if "saw_blade_1" in coll_str else "saw_blade_2"
        target = "battlebot_1" if "battlebot_1" in coll_str else "battlebot_2"
        if "saw_blade" in attacker and "battlebot" in target:
            self.apply_damage(attacker, target, self.damage_config['saw_blade_contact_damage'])

    def process_bot_contact(self, attacker_name, msg):
        if not msg.states: return
        coll_str = msg.states[0].collision1_name + msg.states[0].collision2_name
        
        target_bot = "battlebot_2" if attacker_name == "battlebot_1" else "battlebot_1"
        if target_bot in coll_str:
            self.apply_damage(attacker_name, target_bot, self.damage_config['contact'])
        elif "moving_obstacle" in coll_str:
            self.apply_damage(attacker_name, "moving_obstacle", self.damage_config['to_obstacle'])

    def process_obstacle_contact(self, msg):
        if not msg.states: return
        coll_str = msg.states[0].collision1_name + msg.states[0].collision2_name
        target = "battlebot_1" if "battlebot_1" in coll_str else "battlebot_2" if "battlebot_2" in coll_str else None
        if target:
            self.apply_damage("moving_obstacle", target, self.damage_config['from_obstacle'])

    # --- [수정된 힐링팩 충돌 처리 함수] ---
    def process_heal_contact(self, pack_name, msg):
        # 1회용이므로 이미 사용되었으면 즉시 반환
        if self.healing_packs_used.get(pack_name, True):
            return
        
        # 메시지 안에 실제 충돌 데이터가 있는지 확인
        if not msg.states: 
            return

        target_bot = None
        # 각 충돌 상태를 순회하며 충돌한 로봇을 찾습니다.
        for state in msg.states:
            coll1 = state.collision1_name
            coll2 = state.collision2_name
            if "battlebot_1" in coll1 or "battlebot_1" in coll2:
                target_bot = "battlebot_1"
                break # 찾았으면 루프 중단
            elif "battlebot_2" in coll1 or "battlebot_2" in coll2:
                target_bot = "battlebot_2"
                break # 찾았으면 루프 중단
        
        # 유효한 로봇과 충돌했을 때만 힐링 적용
        if target_bot:
            self.apply_heal(target_bot, self.damage_config['heal_amount'])
            self.healing_packs_used[pack_name] = True
            
            # 힐링팩 모델 제거 요청
            req = DeleteEntity.Request(name=pack_name)
            self.delete_entity_client.call_async(req)
            self.get_logger().info(f"Healing pack '{pack_name}' used by {target_bot} and removed.")

    def apply_damage(self, attacker, target, damage):
        if self.is_destroyed.get(target, True): return
        now = self.get_clock().now()
        if now - self.last_hit_time.get(target, now) < self.hit_interval_duration: return
        if damage <= 0.0: return

        self.health[target] -= damage
        self.last_hit_time[target] = now
        
        if self.health[target] <= 0.0:
            self.health[target] = 0.0
            self.is_destroyed[target] = True
            self.get_logger().warn(f"[DESTROYED] {target} has been destroyed by {attacker}!")
            if target == "moving_obstacle":
                self.obstacle_destroyed_pub.publish(Empty())
            else:
                self.print_result()
        else:
            self.get_logger().info(f"[HIT] {attacker} -> {target} | Damage: {damage:.1f} | {target} HP: {self.health[target]:.1f}")
        self.publish_health()
    
    def apply_heal(self, target, heal_amount):
        if self.is_destroyed.get(target, True): return
        self.health[target] = min(self.initial_health, self.health[target] + heal_amount)
        self.get_logger().info(f"{self.COLOR_GREEN_BOLD}[HEAL] {target} recovered {heal_amount:.1f} HP! | Current HP: {self.health[target]:.1f}{self.COLOR_RESET}")
        self.publish_health()

    def check_flip(self, name, msg):
        if self.is_destroyed.get(name, True): return
        q = msg.orientation
        roll, pitch, _ = quat2euler([q.w, q.x, q.y, q.z], axes='sxyz')
        if abs(roll) > self.flip_threshold_roll_rad or abs(pitch) > self.flip_threshold_pitch_rad:
            if not self.is_flipped.get(name, False):
                self.get_logger().warn(f"{name} flipped!")
                self.apply_damage(attacker="environment (flip)", target=name, damage=self.damage_config['flip'])
                self.is_flipped[name] = True
        else:
            if self.is_flipped.get(name, False):
                self.is_flipped[name] = False

    def publish_health(self):
        for name, health_val in self.health.items():
            if name in self.health_pubs:
                msg = Float32(data=health_val)
                self.health_pubs[name].publish(msg)

    def print_result(self):
        winner = None
        bots_destroyed = self.is_destroyed["battlebot_1"] or self.is_destroyed["battlebot_2"]
        if not bots_destroyed: return
        if self.is_destroyed["battlebot_1"] and not self.is_destroyed["battlebot_2"]: winner = "battlebot_2"
        elif not self.is_destroyed["battlebot_1"] and self.is_destroyed["battlebot_2"]: winner = "battlebot_1"
        else:
            winner = "Draw"
            if self.health["battlebot_1"] > self.health["battlebot_2"]: winner = "battlebot_1"
            elif self.health["battlebot_2"] > self.health["battlebot_1"]: winner = "battlebot_2"

        self.get_logger().info(f"{self.COLOR_GREEN_BOLD}==================== GAME SET ===================={self.COLOR_RESET}")
        self.get_logger().info(f"{self.COLOR_GREEN_BOLD}                     WINNER: {winner}                  {self.COLOR_RESET}")
        self.get_logger().info(f"{self.COLOR_GREEN_BOLD}================================================{self.COLOR_RESET}")
        self.get_logger().info("Shutting down the damage calculator node.")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = DamageCalculator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node and rclpy.ok():
             node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()