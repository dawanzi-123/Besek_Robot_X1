#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 JointState 桥接节点：
订阅原始舵机位置（包括双臂和头部），
转换为弧度，并发布到标准的 /joint_states 话题，供 robot_state_publisher 使用。
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState
import math
from typing import List, Dict

# ---------------------------- ⚠️ 关键配置：请根据您的实际情况修改 ----------------------------

# 1. 舵机ID到关节名的映射 (已添加头部关节 101 和 102)
JOINT_ID_TO_NAME: Dict[int, str] = {
    # 左臂关节 (ID 1-7)
    1: '1', 2: '2', 3: '3', 4: '4',
    5: '5', 6: '6', 7: '7',
    # 右臂关节 (ID 51-57)
    51: '51', 52: '52', 53: '53', 54: '54',
    55: '55', 56: '56', 57: '57',
    # 头部关节 (ID 101, 102)
    101: '101', 102: '102',
}

LEFT_ARM_IDS = [1, 2, 3, 4, 5, 6, 7]
RIGHT_ARM_IDS = [51, 52, 53, 54, 55, 56, 57]
HEAD_IDS = [101, 102] # 新增头部 ID 列表
ALL_JOINT_IDS = LEFT_ARM_IDS + RIGHT_ARM_IDS + HEAD_IDS # 集合所有 ID

# 2. 舵机位置转换参数 (STS 系列通常是 0-4095 计数值)
FULL_RANGE_COUNTS = 4096.0 
# 假设舵机行程是 360 度 (2*pi 弧度)，请根据您的舵机规格修改
FULL_RANGE_RADIANS = 2 * math.pi 
COUNTS_TO_RADIAN_FACTOR = FULL_RANGE_RADIANS / FULL_RANGE_COUNTS

# 3. 舵机零点偏移和方向校正
# 格式: id: (offset_counts, direction_multiplier)
# 默认 2048 计数值为 URDF 的 0 弧度，方向为正 (1.0)
JOINT_CALIBRATION: Dict[int, tuple] = {
    # 默认所有关节
    sid: (2048, 1.0) for sid in ALL_JOINT_IDS
    
    # ⚠️ 特别注意头部关节的方向和零点可能需要修正：
    # 例如，如果 101 舵机反转：
    # 101: (2048, -1.0),
}

# ------------------------------------------------------------------------------------

class JointStateBridge(Node):
    def __init__(self) -> None:
        super().__init__('joint_state_bridge')
        self.get_logger().info('JointState Bridge 启动中...')

        self.joint_positions: Dict[int, float] = {}
        # 初始化所有关节位置为 0.0
        for sid in ALL_JOINT_IDS:
             self.joint_positions[sid] = 0.0 

        # 订阅原始舵机位置话题 (手臂)
        self.create_subscription(Int32MultiArray,
                                 '/xiaobei/left_arm_positions',
                                 self._cb_left, 10)
        self.create_subscription(Int32MultiArray,
                                 '/xiaobei/right_arm_positions',
                                 self._cb_right, 10)
                                 
        # 订阅新增的头部舵机位置话题
        self.create_subscription(Int32MultiArray,
                                 '/xiaobei/head_positions',
                                 self._cb_head, 10)

        # 发布 /joint_states 话题
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

        # 定时器：以 30Hz 发布 /joint_states
        self.timer = self.create_timer(1.0 / 30.0, self._publish_joint_states)
        self.get_logger().info('✅ 订阅原始位置（双臂和头部），准备发布 /joint_states')

    def _convert_and_store(self, ids: List[int], data: List[int]) -> None:
        """ 接收到原始计数值并进行弧度转换和存储 """
        n = min(len(ids), len(data))
        for i in range(n):
            sid = ids[i]
            raw_pos = int(data[i])

            # 从配置中获取校准参数
            offset, multiplier = JOINT_CALIBRATION.get(sid, (2048, 1.0))
            
            # 1. 应用零点偏移
            relative_counts = raw_pos - offset
            
            # 2. 转换为弧度并应用方向校正
            radian_pos = relative_counts * COUNTS_TO_RADIAN_FACTOR * multiplier
            
            # 3. 存储
            self.joint_positions[sid] = radian_pos

    def _cb_left(self, msg: Int32MultiArray) -> None:
        self._convert_and_store(LEFT_ARM_IDS, list(msg.data))

    def _cb_right(self, msg: Int32MultiArray) -> None:
        self._convert_and_store(RIGHT_ARM_IDS, list(msg.data))
        
    def _cb_head(self, msg: Int32MultiArray) -> None:
        """ 处理头部话题的回调函数 """
        self._convert_and_store(HEAD_IDS, list(msg.data))

    def _publish_joint_states(self) -> None:
        """ 定时发布 JointState 消息 """
        
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        
        # 按照预定义的顺序填充名称和位置
        for sid in ALL_JOINT_IDS:
            joint_name = JOINT_ID_TO_NAME.get(sid)
            position = self.joint_positions.get(sid)
            
            if joint_name and position is not None:
                js_msg.name.append(joint_name)
                js_msg.position.append(position)

        self.publisher_.publish(js_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()