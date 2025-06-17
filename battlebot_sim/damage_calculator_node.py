import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time, Duration

from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from transforms3d.euler import quat2euler
import math

class DamageCalculator(Node):
    def __init__(self):
        super().__init__('damage_calculator')

        # 파라미터 선언
        self.declare_parameter('hit_interval', 1.0 / 3.0)
        self.declare_parameter('initial_health', 100.0)
        self.declare_parameter('contact_damage_per_event', 1.0)
        self.declare_parameter('moving_obstacle_contact_damage', 1.0)
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

        hit_interval_sec = self.get_parameter('hit_interval').value
        self.hit_interval_duration = Duration(seconds=hit_interval_sec)
        
        # 로봇 상태 초기화
        self.robots = ["battlebot_1", "battlebot_2"]
        now = self.get_clock().now()
        self.robot_health = {name: self.initial_health for name in self.robots}
        self.is_destroyed = {name: False for name in self.robots}
        self.last_hit_time = {name: now for name in self.robots}
        self.is_flipped = {name: False for name in self.robots}

        # 구독자(Subscriber) 설정
        self.create_subscription(ContactsState, "/saw_blade_1/contact", self.process_external_saw_contact, 10)
        self.create_subscription(ContactsState, "/saw_blade_2/contact", self.process_external_saw_contact, 10)
        self.create_subscription(ContactsState, "/battlebot_1/contact", lambda msg: self.process_bot_vs_bot_contact("battlebot_1", "battlebot_2", msg), 10)
        self.create_subscription(ContactsState, "/battlebot_2/contact", lambda msg: self.process_bot_vs_bot_contact("battlebot_2", "battlebot_1", msg), 10)
        self.create_subscription(ContactsState, "/moving_obstacle/contact", self.process_obstacle_contact, 10)
        self.create_subscription(Imu, "/battlebot_1/imu", lambda msg: self.check_flip("battlebot_1", msg), 10)
        self.create_subscription(Imu, "/battlebot_2/imu", lambda msg: self.check_flip("battlebot_2", msg), 10)

        # 발행자(Publisher) 설정
        self.health_pub = {name: self.create_publisher(Float32, f"/{name}/health", 10) for name in self.robots}
        self.timer = self.create_timer(0.5, self.publish_health)

        # 초기화 완료 로그
        self.get_logger().info("Damage Calculator Node Initialized.")
        self.get_logger().info(f"Damage Cooldown: {hit_interval_sec:.3f} seconds")
        self.get_logger().info(f"battlebot_1 HP: {self.robot_health['battlebot_1']} | battlebot_2 HP: {self.robot_health['battlebot_2']}")

    def process_external_saw_contact(self, msg):
        if not msg.states: return
        coll_str = str(msg.states)
        attacker = "saw_blade_1" if "saw_blade_1" in coll_str else "saw_blade_2"
        target = "battlebot_1" if "battlebot_1" in coll_str else "battlebot_2"
        if "saw_blade" in attacker and "battlebot" in target:
            self.apply_damage(attacker, target, self.damage_config['saw_blade_contact_damage'])

    def process_bot_vs_bot_contact(self, attacker, target, msg):
        if not msg.states: return
        coll_str = msg.states[0].collision1_name + msg.states[0].collision2_name
        if attacker not in coll_str or target not in coll_str: return
        
        is_weapon_hit = "saw" in coll_str
        damage = self.damage_config['weapon_damage_saw'] if is_weapon_hit else self.damage_config['contact_damage']
        self.apply_damage(attacker, target, damage)
        
    def process_obstacle_contact(self, msg):
        if not msg.states: return
        attacker = "moving_obstacle"
        coll_str = msg.states[0].collision1_name + msg.states[0].collision2_name
        target = "battlebot_1" if "battlebot_1" in coll_str else "battlebot_2" if "battlebot_2" in coll_str else None
        if target:
            self.apply_damage(attacker, target, self.damage_config['moving_obstacle_contact_damage'])

    def apply_damage(self, attacker, target, damage):
        """데미지를 적용하는 공통 함수"""
        if self.is_destroyed.get(target, True): return
        now = self.get_clock().now()
        if now - self.last_hit_time.get(target, now) < self.hit_interval_duration: return
        if damage <= 0.0: return

        self.robot_health[target] -= damage
        self.last_hit_time[target] = now
        
        if self.robot_health[target] <= 0.0:
            self.robot_health[target] = 0.0
            self.is_destroyed[target] = True
            self.get_logger().warn(f"[DESTROYED] {target} has been destroyed by {attacker}!")
            self.print_result()
        else:
            self.get_logger().info(f"[HIT] {attacker} → {target} | Damage: {damage:.1f} | HP: {self.robot_health[target]:.1f}")
        
        self.publish_health()
        
    def check_flip(self, name, msg):
        if self.is_destroyed.get(name, True): return
        q = msg.orientation
        roll, pitch, _ = quat2euler([q.w, q.x, q.y, q.z], axes='sxyz')
        if abs(roll) > self.flip_threshold_roll_rad or abs(pitch) > self.flip_threshold_pitch_rad:
            if not self.is_flipped[name]:
                self.get_logger().warn(f"{name} flipped!")
                self.apply_damage(attacker="environment (flip)", target=name, damage=self.damage_config['flip_damage'])
                self.is_flipped[name] = True
        else:
            self.is_flipped[name] = False

    def publish_health(self):
        for name in self.robots:
            msg = Float32()
            msg.data = self.robot_health[name]
            self.health_pub[name].publish(msg)

    def print_result(self):
        winner = None
        if all(self.is_destroyed.values()):
            winner = "Draw"
            if self.robot_health["battlebot_1"] > self.robot_health["battlebot_2"]: winner = "battlebot_1"
            elif self.robot_health["battlebot_2"] > self.robot_health["battlebot_1"]: winner = "battlebot_2"
        elif self.is_destroyed["battlebot_1"]: winner = "battlebot_2"
        elif self.is_destroyed["battlebot_2"]: winner = "battlebot_1"
        else: return

        self.get_logger().info("="*20 + " GAME SET " + "="*20)
        self.get_logger().info(f"WINNER: {winner}")
        self.get_logger().info("="*50)
        self.get_logger().info("Shutting down the damage calculator node.")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = DamageCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node and rclpy.ok():
             node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()