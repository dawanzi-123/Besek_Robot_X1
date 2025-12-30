#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
主控节点（纯 STS）：发布主机双臂当前位置
- 发布话题：
  /xiaobei/left_arm_positions   : Int32MultiArray（左臂 1~7）
  /xiaobei/right_arm_positions  : Int32MultiArray（右臂 51~57）
- 默认两臂卸力，便于手拽示教
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from typing import List

from .servo_interface import ServoBusSTS, MID_POS

class MasterArmPublisher(Node):
    def __init__(self) -> None:
        super().__init__('master_arms')

        # 参数
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 1_000_000)
        self.declare_parameter('rate_hz', 30)
        self.declare_parameter('left_ids',  [1, 2, 3, 4, 5, 6, 7])
        self.declare_parameter('right_ids', [51, 52, 53, 54, 55, 56, 57])
        self.declare_parameter('manual_unload', True)  # 是否默认卸力示教

        port      = self.get_parameter('port').get_parameter_value().string_value
        baudrate  = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.rate = int(self.get_parameter('rate_hz').get_parameter_value().integer_value)
        self.left_ids  = list(self.get_parameter('left_ids').get_parameter_value().integer_array_value)
        self.right_ids = list(self.get_parameter('right_ids').get_parameter_value().integer_array_value)
        manual_unload  = bool(self.get_parameter('manual_unload').get_parameter_value().bool_value)

        # 串口
        self.bus = ServoBusSTS(port=port, baudrate=baudrate)
        try:
            self.bus.open()
            self.get_logger().info(f'✅ 串口已打开：{port} @ {baudrate}')
        except Exception as e:
            self.get_logger().fatal(f'❌ 串口打开失败：{e}')
            raise

        # 初始化 + 卸力示教
        try:
            self.bus.init_ids(self.left_ids + self.right_ids, torque_on=not manual_unload)
            if manual_unload:
                self.bus.set_torque(self.left_ids + self.right_ids, enable=False)
            self.get_logger().info('🔧 舵机初始化完成（主机示教状态：卸力）' if manual_unload else '🔧 舵机初始化完成（上力）')
        except Exception as e:
            self.get_logger().warn(f'舵机初始化异常：{e}')

        # 发布者
        self.pub_left  = self.create_publisher(Int32MultiArray, '/xiaobei/left_arm_positions',  10)
        self.pub_right = self.create_publisher(Int32MultiArray, '/xiaobei/right_arm_positions', 10)

        # 定时器
        self.timer = self.create_timer(1.0 / max(1, self.rate), self._on_timer)
        self.get_logger().info(f'🚀 Master（STS）发布中：left={self.left_ids}, right={self.right_ids}, rate={self.rate}Hz')

    def _read_ids(self, ids: List[int]) -> List[int]:
        out = []
        for sid in ids:
            p = self.bus.read_pos(sid)
            out.append(p if p is not None else MID_POS)
        return out

    def _on_timer(self) -> None:
        msg_l = Int32MultiArray()
        msg_r = Int32MultiArray()
        msg_l.data = self._read_ids(self.left_ids)
        msg_r.data = self._read_ids(self.right_ids)
        self.pub_left.publish(msg_l)
        self.pub_right.publish(msg_r)

    def destroy_node(self) -> None:
        try:
            self.bus.close()
        except Exception:
            pass
        super().destroy_node()

def main() -> None:
    rclpy.init()
    node = MasterArmPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 中断，安全退出')
    finally:
        node.destroy_node()
        rclpy.shutdown()
