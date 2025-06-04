import os
from glob import glob
from setuptools import setup


package_name = 'battlebot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'models'), glob('models/**/*.*', recursive=True)), 
        # 데미지 노드 추가 구문
        ('share/' + package_name + '/config', ['config/params.yaml']),
        # 설정 파일 설치 추가
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='honggeun',
    maintainer_email='your@email.com',
    description='Battlebot simulator package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'moving_obstacle_controller = battlebot_sim.moving_obstacle_controller:main',
             # 데미지 계산 노드 실행 스크립트 추가
            'damage_calculator_node = battlebot_sim.damage_calculator_node:main',
             # dual_teleop 키보드 제어 노드
            'dual_teleop = battlebot_sim.dual_teleop:main',
        ],
    },
)
