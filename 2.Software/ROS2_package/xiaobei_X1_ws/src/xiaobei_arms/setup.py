from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'xiaobei_arms'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=['xiaobei_arms', 'xiaobei_arms.*']),
    data_files=[
        # ROS2 资源索引
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # 包含 package.xml
        ('share/' + package_name, ['package.xml']),
        # 安装 launch 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Beisek Robotics',
    maintainer_email='dev@xiaobei.ai',
    description='ROS2 双臂协同控制系统：主控机器人通过 WiFi 发布舵机姿态，从控机器人实时跟随，实现多机双臂同步运动。',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 主控节点：发布主机舵机角度
            'master_arm_publisher = xiaobei_arms.master_arm_publisher:main',
            # 从控节点：接收主机角度并驱动本机舵机
            'slave_arm_subscriber = xiaobei_arms.slave_arm_subscriber:main',
            # 💥 新增的桥接节点：原始数据 -> /joint_states
            'joint_state_bridge = xiaobei_arms.joint_state_bridge:main',
            
            'trajectory_action_server = xiaobei_arms.trajectory_action_server:main',  # ★ 新增
        ],
    },
)