#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
从控节点（纯 STS）：接收主机双臂目标位置并执行本机跟随
- 订阅：
  /xiaobei/left_arm_positions   : 左臂 1~7
  /xiaobei/right_arm_positions  : 右臂 51~57
- 默认正向控制（左手→左手，右手→右手），无反转
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from typing import List, Set

from .servo_interface import ServoBusSTS, MAX_POS


class SlaveArmSubscriber(Node):
    """从控端：接收主机姿态并下发至本机舵机（纯 STS 版本）"""
    def __init__(self) -> None:
        super().__init__('slave_arms')

        # ---------------- 参数 ----------------
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 1_000_000)
        self.declare_parameter('left_ids',  [1, 2, 3, 4, 5, 6, 7])
        self.declare_parameter('right_ids', [51, 52, 53, 54, 55, 56, 57])
        self.declare_parameter('speed', 1500)
        self.declare_parameter('accel', 0)
        # ✅ 默认不反转
        self.declare_parameter('reverse_ids', [])

        port     = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.left_ids  = list(self.get_parameter('left_ids').get_parameter_value().integer_array_value)
        self.right_ids = list(self.get_parameter('right_ids').get_parameter_value().integer_array_value)
        self.speed     = int(self.get_parameter('speed').get_parameter_value().integer_value)
        self.accel     = int(self.get_parameter('accel').get_parameter_value().integer_value)
        self.reverse: Set[int] = set(self.get_parameter('reverse_ids').get_parameter_value().integer_array_value)

        # ---------------- 串口初始化 ----------------
        self.bus = ServoBusSTS(port=port, baudrate=baudrate)
        try:
            self.bus.open()
            self.get_logger().info(f'✅ 串口已打开：{port} @ {baudrate}')
        except Exception as e:
            self.get_logger().fatal(f'❌ 串口打开失败：{e}')
            raise

        # 上力准备执行
        try:
            self.bus.init_ids(self.left_ids + self.right_ids, torque_on=True)
            self.get_logger().info('🔧 舵机初始化完成（从机上力）')
        except Exception as e:
            self.get_logger().warn(f'舵机初始化异常：{e}')

        # ---------------- 订阅主机话题 ----------------
        self.sub_left  = self.create_subscription(Int32MultiArray,
                                                  '/xiaobei/left_arm_positions',
                                                  self._cb_left, 10)
        self.sub_right = self.create_subscription(Int32MultiArray,
                                                  '/xiaobei/right_arm_positions',
                                                  self._cb_right, 10)

        self.get_logger().info(
            f'🚀 Slave（STS 正向控制）启动完毕：\n'
            f'    left={self.left_ids}\n'
            f'    right={self.right_ids}\n'
            f'    reverse={sorted(self.reverse)}\n'
            f'    speed={self.speed}, accel={self.accel}'
        )

    # ---------------- 执行舵机动作 ----------------
    def _apply_targets(self, ids: List[int], targets: List[int]) -> None:
        """接收到目标位置数组后，逐个舵机下发"""
        n = min(len(ids), len(targets))
        for i in range(n):
            sid = ids[i]
            pos = int(targets[i])

            # 若手动指定某关节反转（一般为空）
            if sid in self.reverse:
                pos = MAX_POS - pos

            ok = self.bus.write_pos(sid, pos, speed=self.speed, accel=self.accel)
            if not ok:
                self.get_logger().debug(f'写入失败：id={sid}, pos={pos}')

    # ---------------- 订阅回调 ----------------
    def _cb_left(self, msg: Int32MultiArray) -> None:
        self._apply_targets(self.left_ids, list(msg.data))

    def _cb_right(self, msg: Int32MultiArray) -> None:
        self._apply_targets(self.right_ids, list(msg.data))

    # ---------------- 清理关闭 ----------------
    def destroy_node(self) -> None:
        try:
            self.bus.close()
        except Exception:
            pass
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = SlaveArmSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 中断，安全退出')
    finally:
        node.destroy_node()
        rclpy.shutdown()
