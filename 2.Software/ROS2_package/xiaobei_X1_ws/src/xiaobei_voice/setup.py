from setuptools import find_packages, setup

package_name = 'xiaobei_voice'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu22',
    maintainer_email='ubuntu22@todo.todo',
    description='Voice control package for Xiaobei Robot using USB serial module',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 1. 负责听：串口读取节点
            'voice_node = xiaobei_voice.voice_serial_node:main',
            # 2. 负责做：语音控制逻辑节点 (新增)
            'voice_control_node = xiaobei_voice.voice_control_node:main',
        ],
    },
)