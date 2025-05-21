#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy

class WeaponController(Node):
    """
    무기 컨트롤러 노드: 조이스틱 입력을 받아 무기(톱날) 회전 속도를 제어합니다
    """
    def __init__(self):
        super().__init__('weapon_controller')
        
        # 퍼블리셔 생성: 무기 회전 속도 명령
        self.weapon_pub = self.create_publisher(
            Float64, 
            'weapon_speed', 
            10
        )
        
        # 조이스틱 입력 구독
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        # 키보드 입력을 위한 타이머 (조이스틱이 없는 경우)
        self.keyboard_timer = self.create_timer(0.1, self.keyboard_callback)
        
        # 현재 무기 속도 상태
        self.weapon_speed = 0.0
        self.max_weapon_speed = 20.0  # 최대 회전 속도 (rad/s)
        
        self.get_logger().info('무기 컨트롤러 시작됨')
    
    def joy_callback(self, msg):
        """
        조이스틱 입력 처리: 오른쪽 트리거(R2)가 무기 속도 제어
        """
        # 일반적으로 조이스틱 설정에서 R2 버튼은 axes[5]
        if len(msg.axes) >= 6:
            # R2 트리거는 -1(누름)에서 1(안누름) 범위이므로 변환 필요
            trigger_value = (-msg.axes[5] + 1) / 2
            self.weapon_speed = trigger_value * self.max_weapon_speed
            self.send_weapon_command()
        
        # 무기 비상 정지: B 버튼 (일반적으로 버튼[1])
        if len(msg.buttons) >= 2 and msg.buttons[1] == 1:
            self.weapon_speed = 0.0
            self.send_weapon_command()
    
    def keyboard_callback(self):
        """
        키보드로 무기 속도 제어 (조이스틱이 없는 경우)
        이 함수는 실제 키 입력을 감지하지 않으며, 키 입력 감지는 
        teleop_twist_keyboard 패키지를 통해 이루어집니다.
        
        여기서는 키보드 명령을 시뮬레이션하기 위해 키보드 입력을 처리하는 방법을 설명합니다:
        - 'q' 키: 무기 속도 증가
        - 'z' 키: 무기 속도 감소
        - 'x' 키: 무기 정지
        """
        # 실제 구현에서는 키보드 입력 메시지를 구독하여 처리
        pass
    
    def send_weapon_command(self):
        """무기 회전 속도 명령 전송"""
        msg = Float64()
        msg.data = self.weapon_speed
        self.weapon_pub.publish(msg)
        self.get_logger().debug(f'무기 속도 명령: {self.weapon_speed}')

def main(args=None):
    rclpy.init(args=args)
    controller = WeaponController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
