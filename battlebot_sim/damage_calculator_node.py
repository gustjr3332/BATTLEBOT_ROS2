import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from tf_transformations import euler_from_quaternion # Quaternion to RPY conversion

import math

class DamageCalculator(Node):
    def __init__(self):
        super().__init__('damage_calculator')

        # Declare and get parameters
        self.declare_parameter('initial_health', 100.0)
        self.declare_parameter('contact_damage_per_event', 1.0)
        self.declare_parameter('moving_obstacle_contact_damage', 5.0)
        self.declare_parameter('saw_blade_contact_damage', 5.0) # <--- 새로 추가된 파라미터
        self.declare_parameter('flip_damage_per_event', 5.0)
        self.declare_parameter('weapon_damage_saw', 10.0)
        self.declare_parameter('weapon_damage_hammer', 15.0)
        self.declare_parameter('flip_threshold_roll_deg', 90.0)  # Degrees
        self.declare_parameter('flip_threshold_pitch_deg', 90.0) # Degrees

        self.initial_health = self.get_parameter('initial_health').get_parameter_value().double_value
        self.damage_config = {
            'contact_damage': self.get_parameter('contact_damage_per_event').get_parameter_value().double_value,
            'moving_obstacle_contact_damage': self.get_parameter('moving_obstacle_contact_damage').get_parameter_value().double_value,
            'saw_blade_contact_damage': self.get_parameter('saw_blade_contact_damage').get_parameter_value().double_value, # <--- 파라미터 값 가져오기
            'flip_damage': self.get_parameter('flip_damage_per_event').get_parameter_value().double_value,
            'weapon_damage_saw': self.get_parameter('weapon_damage_saw').get_parameter_value().double_value,
            'weapon_damage_hammer': self.get_parameter('weapon_damage_hammer').get_parameter_value().double_value,
        }
        self.flip_threshold_roll_rad = math.radians(self.get_parameter('flip_threshold_roll_deg').get_parameter_value().double_value)
        self.flip_threshold_pitch_rad = math.radians(self.get_parameter('flip_threshold_pitch_deg').get_parameter_value().double_value)

        # Initialize health for each bot
        self.robot_health = {
            "battlebot": self.initial_health,
            "moving_obstacle": self.initial_health,
            # 만약 saw_blade_1, saw_blade_2도 체력이 있다면 여기에 추가
            # "saw_blade_1": self.initial_health,
            # "saw_blade_2": self.initial_health,
        }

        # Subscribers for contact sensors
        self.contact_sub_battlebot = self.create_subscription(
            ContactsState,
            "/battlebot/contact", # URDF 플러그인의 bumperTopicName과 일치하는지 확인!
            lambda msg: self.process_contact("battlebot", "moving_obstacle", msg), # battlebot이 moving_obstacle을 공격
            10
        )
        self.contact_sub_obstacle = self.create_subscription(
            ContactsState,
            "/moving_obstacle/contact", # moving_obstacle의 URDF 플러그인 토픽 확인
            lambda msg: self.process_contact("moving_obstacle", "battlebot", msg), # moving_obstacle이 battlebot을 공격
            10
        )
        # saw_blade_1과 saw_blade_2의 접촉 센서 구독 추가
        self.contact_sub_saw_blade_1 = self.create_subscription(
            ContactsState,
            "/saw_blade_1/contact", # saw_blade_1의 URDF 플러그인 토픽 확인 (아마도 /saw_blade_1/contact_sensor 일 수도 있음)
            lambda msg: self.process_contact("saw_blade_1", "battlebot", msg), # saw_blade_1이 battlebot을 공격
            10
        )
        self.contact_sub_saw_blade_2 = self.create_subscription(
            ContactsState,
            "/saw_blade_2/contact", # saw_blade_2의 URDF 플러그인 토픽 확인
            lambda msg: self.process_contact("saw_blade_2", "battlebot", msg), # saw_blade_2가 battlebot을 공격
            10
        )


        # Subscribers for IMU sensors
        self.imu_sub_battlebot = self.create_subscription(
            Imu,
            "/battlebot/imu", # URDF 플러그인의 topicName과 일치하는지 확인!
            lambda msg: self.check_flip("battlebot", msg),
            10
        )
        self.imu_sub_obstacle = self.create_subscription(
            Imu,
            "/moving_obstacle/imu", # moving_obstacle의 URDF 플러그인 토픽 확인
            lambda msg: self.check_flip("moving_obstacle", msg),
            10
        )
        # saw_blade들은 IMU가 없을 수 있으므로 일단 추가하지 않음

        # Publishers for health
        self.health_pub_battlebot = self.create_publisher(Float32, "/battlebot/health", 10)
        self.health_pub_obstacle = self.create_publisher(Float32, "/moving_obstacle/health", 10)
        # 만약 saw_blade_1, saw_blade_2도 체력이 있다면 퍼블리셔 추가
        # self.health_pub_saw_blade_1 = self.create_publisher(Float32, "/saw_blade_1/health", 10)
        # self.health_pub_saw_blade_2 = self.create_publisher(Float32, "/saw_blade_2/health", 10)


        # Timer to periodically publish health (optional, can be done on contact/imu callback)
        self.timer = self.create_timer(0.5, self.publish_health)

        self.get_logger().info("DamageCalculator node started.")

        # To track if a robot is currently flipped
        self.is_flipped = {
            "battlebot": False,
            # "moving_obstacle": False,
            # "saw_blade_1": False,
            # "saw_blade_2": False,
        }

    def process_contact(self, attacker_bot_name, target_bot_name, msg):
        if msg.states:
            for contact_state in msg.states:
                # Check if the contact involves the other robot (or its parts)
                target_hit = (target_bot_name in contact_state.collision1_name or
                              target_bot_name in contact_state.collision2_name)

                if target_hit:
                    self.get_logger().info(f"{attacker_bot_name} detected contact with {target_bot_name}!")

                    damage_amount = 0.0
                    
                    # Case 1: moving_obstacle hits battlebot
                    if attacker_bot_name == "moving_obstacle" and target_bot_name == "battlebot":
                        damage_amount = self.damage_config['moving_obstacle_contact_damage']
                        self.get_logger().info(f"Moving obstacle hit battlebot! Damage: {damage_amount}")
                    
                    # Case 2: saw_blade_1 or saw_blade_2 hits battlebot
                    elif (attacker_bot_name == "saw_blade_1" or attacker_bot_name == "saw_blade_2") \
                         and target_bot_name == "battlebot":
                        damage_amount = self.damage_config['saw_blade_contact_damage'] # <--- 여기에 톱날 데미지 적용
                        self.get_logger().info(f"{attacker_bot_name} hit battlebot! Damage: {damage_amount}")
                    
                    # Case 3: battlebot hits something (moving_obstacle, saw_blade_1, saw_blade_2, etc.)
                    # 이 부분은 battlebot의 무기(saw, hammer)에 의한 공격을 처리합니다.
                    elif attacker_bot_name == "battlebot":
                        saw_hit_by_battlebot = ("saw" in contact_state.collision1_name or
                                                "saw" in contact_state.collision2_name)


                        if saw_hit_by_battlebot:
                            damage_amount = self.damage_config['weapon_damage_saw']
                            self.get_logger().info(f"{attacker_bot_name}'s saw hit {target_bot_name}! Damage: {damage_amount}")
                        else:
                            # General body contact by battlebot
                            damage_amount = self.damage_config['contact_damage']
                            self.get_logger().info(f"{attacker_bot_name} body hit {target_bot_name}! Damage: {damage_amount}")
                    
                    # If damage_amount is still 0.0 (e.g., obstacles hitting each other, or other cases not handled)
                    if damage_amount == 0.0:
                        self.get_logger().debug(f"No specific damage rule for {attacker_bot_name} hitting {target_bot_name}.")
                        continue # 이 충돌 이벤트는 데미지 적용 없이 건너뜀

                    self.robot_health[target_bot_name] -= damage_amount

                    # Ensure health doesn't go below zero
                    if self.robot_health[target_bot_name] < 0:
                        self.robot_health[target_bot_name] = 0
                        self.get_logger().warn(f"{target_bot_name} health reached 0!")

                    self.publish_health() # Publish health immediately after damage
                    break # Only apply damage once per contact event, to avoid multiple hits from one contact event

    def check_flip(self, bot_name, msg):
        # Convert quaternion to RPY
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        # Check if the robot is flipped
        if abs(roll) > self.flip_threshold_roll_rad or abs(pitch) > self.flip_threshold_pitch_rad:
            if not self.is_flipped[bot_name]:
                self.robot_health[bot_name] -= self.damage_config['flip_damage']
                self.get_logger().warn(f"{bot_name} is FLIPPED! Health: {self.robot_health[bot_name]:.2f}")
                self.is_flipped[bot_name] = True
                self.publish_health()
        else:
            # Robot is no longer flipped
            self.is_flipped[bot_name] = False

    def publish_health(self):
        health_msg = Float32()

        # Publish battlebot health
        health_msg.data = self.robot_health["battlebot"]
        self.health_pub_battlebot.publish(health_msg)

        # Publish moving_obstacle health (if it exists)
        if "moving_obstacle" in self.robot_health:
            health_msg.data = self.robot_health["moving_obstacle"]
            self.health_pub_obstacle.publish(health_msg)

        # 만약 saw_blade_1, saw_blade_2도 체력이 있다면 여기에 추가
        # if "saw_blade_1" in self.robot_health:
        #     health_msg.data = self.robot_health["saw_blade_1"]
        #     self.health_pub_saw_blade_1.publish(health_msg)
        # if "saw_blade_2" in self.robot_health:
        #     health_msg.data = self.robot_health["saw_blade_2"]
        #     self.health_pub_saw_blade_2.publish(health_msg)


def main(args=None):
    rclpy.init(args=args)
    damage_calculator = DamageCalculator()
    rclpy.spin(damage_calculator)
    damage_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()