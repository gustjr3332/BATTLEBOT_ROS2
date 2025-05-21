from setuptools import setup
import os
from glob import glob

package_name = 'battlebot_spinner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
	('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/spawn_spinner_bot.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/spinner_bot.xacro']),
        ('share/' + package_name + '/models/spinner_bot', [
            'models/spinner_bot/model.config',
            'models/spinner_bot/model.sdf'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gustjr3332',
    maintainer_email='gustjr3332@gmail.com',
    description='Battlebot spinner simulation package',
    license='MIT',
    entry_points={
        'console_scripts': [
        ],
    },
)
