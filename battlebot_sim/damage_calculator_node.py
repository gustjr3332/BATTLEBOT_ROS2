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
        self.declare_parameter('obstacle_initial_health', 50.0)
        self.declare_parameter('heal_amount', 20.0)
        self.declare_parameter('flip_damage', 100.0)
        self.declare_parameter('saw_blade_contact_damage', 5.0)
        self.declare_parameter('obstacle_contact_damage', 1.0)
        self.declare_parameter('min_force_threshold', 100.0)
        self.declare_parameter('max_force_for_scaling', 5000.0)
        self.declare_parameter('min_damage_at_threshold', 1.0)
        self.declare_parameter('max_damage_at_max_force', 15.0)
        self.declare_parameter('flip_threshold_roll_deg', 90.0)
        self.declare_parameter('flip_threshold_pitch_deg', 90.0)

        # 파라미터 값 가져오기
        self.initial_health = self.get_parameter('initial_health').value
        self.obstacle_initial_health = self.get_parameter('obstacle_initial_health').value
        self.damage_config = {
            'flip': self.get_parameter('flip_damage').value,
            'heal_amount': self.get_parameter('heal_amount').value,
            'saw_blade': self.get_parameter('saw_blade_contact_damage').value,
            'from_obstacle': self.get_parameter('obstacle_contact_damage').value,
            'min_force': self.get_parameter('min_force_threshold').value,
            'max_force': self.get_parameter('max_force_for_scaling').value,
            'min_damage': self.get_parameter('min_damage_at_threshold').value,
            'max_damage': self.get_parameter('max_damage_at_max_force').value,
        }
        self.hit_interval_duration = Duration(seconds=self.get_parameter('hit_interval').value)
        self.flip_threshold_roll_rad = math.radians(self.get_parameter('flip_threshold_roll_deg').value)
        self.flip_threshold_pitch_rad = math.radians(self.get_parameter('flip_threshold_pitch_deg').value)
        
        self.entities = ["battlebot_1", "battlebot_2", "moving_obstacle"]
        self.health = { name: (self.initial_health if "bot" in name else self.obstacle_initial_health) for name in self.entities }
        self.is_destroyed = {name: False for name in self.entities}
        self.last_hit_time = {name: self.get_clock().now() for name in self.entities}
        self.is_flipped = {"battlebot_1": False, "battlebot_2": False}
        self.healing_packs_used = {"healing_pack_1": False, "healing_pack_2": False}

        # 구독자 설정
        self.create_subscription(ContactsState, "/battlebot_1/contact", lambda msg: self.process_contact("battlebot_1", msg), 10)
        self.create_subscription(ContactsState, "/battlebot_1/saw_contact", lambda msg: self.process_contact("battlebot_1", msg), 10)
        self.create_subscription(ContactsState, "/battlebot_2/contact", lambda msg: self.process_contact("battlebot_2", msg), 10)
        self.create_subscription(ContactsState, "/battlebot_2/saw_contact", lambda msg: self.process_contact("battlebot_2", msg), 10)
        self.create_subscription(ContactsState, "/moving_obstacle/contact", lambda msg: self.process_contact("moving_obstacle", msg), 10)
        self.create_subscription(ContactsState, "/saw_blade_1/contact", lambda msg: self.process_contact("saw_blade_1", msg), 10)
        self.create_subscription(ContactsState, "/saw_blade_2/contact", lambda msg: self.process_contact("saw_blade_2", msg), 10)
        self.create_subscription(Imu, "/battlebot_1/imu", lambda msg: self.check_flip("battlebot_1", msg), 10)
        self.create_subscription(Imu, "/battlebot_2/imu", lambda msg: self.check_flip("battlebot_2", msg), 10)
        self.create_subscription(ContactsState, "/healing_pack_1/contact", lambda msg: self.process_heal_contact("healing_pack_1", msg), 10)
        self.create_subscription(ContactsState, "/healing_pack_2/contact", lambda msg: self.process_heal_contact("healing_pack_2", msg), 10)

        # 발행자 및 서비스 클라이언트
        self.health_pubs = {name: self.create_publisher(Float32, f"/{name}/health", 10) for name in self.entities}
        self.obstacle_destroyed_pub = self.create_publisher(Empty, "/obstacle_destroyed", 10)
        self.timer = self.create_timer(0.5, self.publish_health)
        self.delete_entity_client = self.create_client(DeleteEntity, '/delete_entity')
        self.get_logger().info("Damage Calculator Initialized (Force-Based Damage Model).")

    def get_max_impact_force(self, msg: ContactsState) -> float:
        max_force = 0.0
        for state in msg.states:
            for wrench in state.wrenches:
                force = wrench.force
                magnitude = math.sqrt(force.x**2 + force.y**2 + force.z**2)
                if magnitude > max_force: max_force = magnitude
        return max_force

    def calculate_force_based_damage(self, force: float) -> float:
        # --- [수정] 데미지 계산 시 1.5배를 적용 ---
        damage_multiplier = 1.5
        min_f = self.damage_config['min_force']
        max_f = self.damage_config['max_force']
        min_d = self.damage_config['min_damage'] * damage_multiplier
        max_d = self.damage_config['max_damage'] * damage_multiplier
        
        if force < min_f: return 0.0
        if force >= max_f: return max_d
        
        ratio = (force - min_f) / (max_f - min_f)
        damage = min_d + ratio * (max_d - min_d)
        return damage
        
    def process_contact(self, sensor_owner_name: str, msg: ContactsState):
        if not msg.states: return
        coll_str = msg.states[0].collision1_name + msg.states[0].collision2_name
        all_possible_entities = self.entities + ["saw_blade_1", "saw_blade_2"]
        involved_entities = [name for name in all_possible_entities if name in coll_str]
        if len(involved_entities) < 2: return
        
        attacker, target = None, None
        if sensor_owner_name in involved_entities:
            attacker = sensor_owner_name
            target = next((name for name in involved_entities if name != attacker), None)
        if not (attacker and target): return

        damage = 0.0
        if "saw_blade" in attacker:
            damage = self.damage_config['saw_blade']
        elif attacker == "moving_obstacle":
            damage = self.damage_config['from_obstacle']
        elif "battlebot" in attacker:
            force = self.get_max_impact_force(msg)
            damage = self.calculate_force_based_damage(force)
            # --- [수정] 톱날 무기 데미지 2배 증폭 로직 제거 ---
        
        self.apply_damage(attacker, target, damage)

    def apply_damage(self, attacker, target, damage):
        if self.is_destroyed.get(target, True): return
        now = self.get_clock().now()
        if attacker != "environment (flip)":
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
                if "battlebot" in attacker:
                    self.get_logger().info(f"{self.COLOR_GREEN_BOLD}[REWARD] {attacker} health restored!{self.COLOR_RESET}")
                    self.health[attacker] = self.initial_health
            else:
                self.print_result()
        else:
            self.get_logger().info(f"[HIT] {attacker} -> {target} | Damage: {damage:.1f} | {target} HP: {self.health[target]:.1f}")
        self.publish_health()
    
    def process_heal_contact(self, pack_name, msg):
        if self.healing_packs_used.get(pack_name, True) or not msg.states: return
        target_bot = None
        for state in msg.states:
            coll1, coll2 = state.collision1_name, state.collision2_name
            if "battlebot_1" in coll1 or "battlebot_1" in coll2: target_bot = "battlebot_1"; break
            elif "battlebot_2" in coll1 or "battlebot_2" in coll2: target_bot = "battlebot_2"; break
        if target_bot:
            self.apply_heal(target_bot, self.damage_config['heal_amount'])
            self.healing_packs_used[pack_name] = True
            req = DeleteEntity.Request(name=pack_name)
            self.delete_entity_client.call_async(req)
            self.get_logger().info(f"Healing pack '{pack_name}' used by {target_bot} and removed.")

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
            if self.is_flipped.get(name, False): self.is_flipped[name] = False

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
        self.get_logger().info(f"{self.COLOR_GREEN_BOLD}--- GAME OVER! WINNER is {winner} ---{self.COLOR_RESET}")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = DamageCalculator()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass
    finally:
        if node and rclpy.ok(): node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()