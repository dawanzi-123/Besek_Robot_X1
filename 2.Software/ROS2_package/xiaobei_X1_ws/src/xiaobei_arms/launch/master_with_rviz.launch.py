import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 找到功能包的共享目录
    pkg_xiaobei_arms = get_package_share_directory('xiaobei_arms')
    pkg_robot_model = get_package_share_directory('pkg_robot_model')

    # 1. 准备 URDF 文件内容
    # 路径: ~/xiaobei_X1_ws/src/pkg_robot_model/urdf/pkg_robot_model.urdf
    urdf_path = os.path.join(pkg_robot_model, 'urdf', 'pkg_robot_model.urdf')
    
    # 假设这是纯 URDF 文件，直接读取内容。
    # 警告：如果您的文件是 Xacro 格式，这里需要使用 Command(['xacro ', urdf_path])
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    # 2. 启动 'master_arm_publisher' 节点 (读取硬件数据)
    master_publisher_node = Node(
        package='xiaobei_arms',
        executable='master_arm_publisher',
        name='master_arm_publisher',
        output='screen',
        parameters=[
            {'port': '/dev/ttyACM0'},
            {'manual_unload': True}, 
        ]
    )
    
    # 3. 启动 'joint_state_bridge' 节点 (桥接数据到 /joint_states)
    joint_bridge_node = Node(
        package='xiaobei_arms',
        executable='joint_state_bridge',
        name='joint_state_bridge',
        output='screen',
    )
    
    # 4. 启动 'robot_state_publisher' 节点 (将 /joint_states 转换为 TF2)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': False} # 针对真实硬件
        ]
    )

    # 5. 启动 RViz 节点
    # 路径: ~/xiaobei_X1_ws/src/pkg_robot_model/config/default_robot_config.rviz
    rviz_config_file = os.path.join(pkg_robot_model, 'config', 'default_robot_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    return LaunchDescription([
        master_publisher_node,
        joint_bridge_node,
        robot_state_publisher_node,
        rviz_node
    ])