#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MoveIt 轨迹执行服务器（FollowJointTrajectory → 飞特舵机）

- 匹配当前 moveit_controllers.yaml 配置：
  * left_arm_controller: joints [1..6]
  * right_arm_controller: joints [51..56]

- 在 ROS 中提供两个 ActionServer：
  * /left_arm_controller/follow_joint_trajectory
  * /right_arm_controller/follow_joint_trajectory

- 将 MoveIt 下发的关节轨迹，转换为飞特舵机的位置指令，并通过 scservo_sdk 下发。
  注意：
    1）这里默认“关节名 = 舵机 ID”（'1' → ID=1）
    2）这里暂时假设 MoveIt 传来的关节值已经是 0~4095 的刻度；
        如果你后面改成用弧度，可以在 convert_position() 里改映射关系。
"""

import time
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# 从包内的 scservo_sdk 导入（路径结构按你当前 xiaobei_arms 包）
from .scservo_sdk import PortHandler, sms_sts, COMM_SUCCESS




JOINT_ZERO = {
    "L1": 2048,
    "L2": 2048,
    "L3": 2048,
    "L4": 2048,
    "L5": 2048,
    "L6": 2048,
    "R51": 2048,
    "R52": 2048,
    "R53": 2048,
    "R54": 2048,
    "R55": 2048,
    "R56": 2048,
}



# ==========================
#   工具函数：值转换
# ==========================

def convert_position(joint_name: str, value: float) -> int:
    """
    将 MoveIt 传来的关节值转换为舵机刻度值。

    当前假设：
      - MoveIt 的关节值已经是 0~4095 之间的数（比如你在 URDF 里直接用伺服刻度）
      - 如果后续你改成用“弧度”，可以在这里实现：rad -> 0~4095 的映射

    :param joint_name: 关节名（这里就是字符串形式的 ID，例如 '1', '51'）
    :param value:      MoveIt 传来的关节值（float）
    :return:           整数刻度值（0~4095）
    """
    # TODO：如果以后使用弧度，这里改为真正的转换逻辑
    #       示例：center = 2048, rad ∈ [-pi/2, pi/2] 映射到 [2048-1024, 2048+1024]
    pos = int(round(value))
    if pos < 0:
        pos = 0
    elif pos > 4095:
        pos = 4095
    return pos


# ==========================
#   主类：轨迹执行服务器
# ==========================

class TrajectoryActionServer(Node):
    def __init__(self):
        super().__init__('xiaobei_trajectory_action_server')

        # --------------- 参数配置 ---------------
        # 串口设备和波特率，根据你目前双臂用的实际端口调整
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 1_000_000)

        port_name = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        # --------------- 初始化串口和舵机总线 ---------------
        self.get_logger().info(f'初始化飞特舵机总线：port={port_name}, baudrate={baudrate}')
        self.port_handler = PortHandler(port_name)
        self.sts = sms_sts(self.port_handler)

        if not self.port_handler.openPort():
            self.get_logger().error(f'无法打开串口 {port_name}')
            raise RuntimeError('打开串口失败')

        if not self.port_handler.setBaudRate(baudrate):
            self.get_logger().error(f'无法设置波特率 {baudrate}')
            raise RuntimeError('设置波特率失败')

        self.get_logger().info('✅ 飞特串口初始化成功')

        # 下发时使用的速度和加速度（可根据需要调整）
        self.move_speed = 800
        self.move_acc = 100

        # --------------- 关节列表（严格匹配你当前配置） ---------------
        # 注意：这里的 key 是 MoveIt 关节名（字符串），value 是舵机 ID（int）
        self.left_joint_map: Dict[str, int] = {
            '1': 1,
            '2': 2,
            '3': 3,
            '4': 4,
            '5': 5,
            '6': 6,
        }
        self.right_joint_map: Dict[str, int] = {
            '51': 51,
            '52': 52,
            '53': 53,
            '54': 54,
            '55': 55,
            '56': 56,
        }

        # --------------- Action Server：左臂 & 右臂 ---------------
        # 注意 action 名称要与 MoveIt 中的 action_ns 对齐（建议设置为 follow_joint_trajectory）
        #   /left_arm_controller/follow_joint_trajectory
        #   /right_arm_controller/follow_joint_trajectory
        self.left_server = ActionServer(
            self,
            FollowJointTrajectory,
            'left_arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_left_cb)

        self.right_server = ActionServer(
            self,
            FollowJointTrajectory,
            'right_arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_right_cb)

        self.get_logger().info('✅ TrajectoryActionServer 已启动：等待 MoveIt 轨迹指令')

    # ==========================
    #   舵机写入封装
    # ==========================

    def write_servo_position(self, servo_id: int, pos: int):
        """
        对单个舵机下发位置指令（位置模式）
        """
        # 这里直接用 WritePosEx，与你现有 follow_id 脚本保持一致风格
        try:
            _, err = self.sts.WritePosEx(servo_id, pos, self.move_speed, self.move_acc)
            if err != 0:
                self.get_logger().warn(f'写入舵机 ID {servo_id} 位置 {pos} 出错，err={err}')
        except Exception as e:
            self.get_logger().error(f'写入舵机 ID {servo_id} 失败: {e}')

    def send_trajectory_point(self, joint_map: Dict[str, int],
                              joint_names: List[str],
                              point: JointTrajectoryPoint):
        """
        将一个 JointTrajectoryPoint 下发到对应舵机
        :param joint_map:  关节名 -> 舵机ID 映射
        :param joint_names: 该轨迹中的关节名列表
        :param point:       当前轨迹点
        """
        positions = list(point.positions)
        if len(joint_names) != len(positions):
            self.get_logger().error('JointTrajectoryPoint 中 positions 数量与 joint_names 不一致')
            return

        for name, value in zip(joint_names, positions):
            if name not in joint_map:
                # MoveIt 里有这个 joint，但我们不控制（可以忽略）
                self.get_logger().debug(f'忽略未映射关节: {name}')
                continue

            servo_id = joint_map[name]
            pos_tick = convert_position(name, value)
            self.write_servo_position(servo_id, pos_tick)

    # ==========================
    #   Action 执行回调：左臂
    # ==========================

    def execute_left_cb(self, goal_handle):
        """
        处理 left_arm_controller 的轨迹执行
        """
        self.get_logger().info('📥 收到左臂 FollowJointTrajectory 目标')

        traj: JointTrajectory = goal_handle.request.trajectory
        joint_names = list(traj.joint_names)

        # 简单校验：确保所有关节都在我们的映射表中（或者是子集）
        for name in joint_names:
            if name not in self.left_joint_map:
                self.get_logger().warn(f'左臂轨迹中包含未识别关节: {name}')

        # 逐点执行轨迹（根据 time_from_start 做简单时间控制）
        last_t = 0.0
        start_wall_time = time.time()

        for point in traj.points:
            # 支持 cancel
            if goal_handle.is_cancel_requested:
                self.get_logger().warn('左臂轨迹被取消')
                goal_handle.canceled()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                result.error_string = 'Goal canceled'
                return result

            # 计算与上一点的时间差
            t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            dt = max(0.0, t - last_t)
            last_t = t

            # 按时间差 sleep，尽量贴近 MoveIt 规划的节奏
            if dt > 0.0:
                target_time = start_wall_time + t
                now = time.time()
                sleep_dur = target_time - now
                if sleep_dur > 0.0:
                    time.sleep(sleep_dur)

            # 下发当前点的所有关节位置
            self.send_trajectory_point(self.left_joint_map, joint_names, point)

        # 执行完成
        goal_handle.succeed()
        self.get_logger().info('✅ 左臂轨迹执行完成')

        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = ''
        return result

    # ==========================
    #   Action 执行回调：右臂
    # ==========================

    def execute_right_cb(self, goal_handle):
        """
        处理 right_arm_controller 的轨迹执行
        """
        self.get_logger().info('📥 收到右臂 FollowJointTrajectory 目标')

        traj: JointTrajectory = goal_handle.request.trajectory
        joint_names = list(traj.joint_names)

        for name in joint_names:
            if name not in self.right_joint_map:
                self.get_logger().warn(f'右臂轨迹中包含未识别关节: {name}')

        last_t = 0.0
        start_wall_time = time.time()

        for point in traj.points:
            if goal_handle.is_cancel_requested:
                self.get_logger().warn('右臂轨迹被取消')
                goal_handle.canceled()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                result.error_string = 'Goal canceled'
                return result

            t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            dt = max(0.0, t - last_t)
            last_t = t

            if dt > 0.0:
                target_time = start_wall_time + t
                now = time.time()
                sleep_dur = target_time - now
                if sleep_dur > 0.0:
                    time.sleep(sleep_dur)

            self.send_trajectory_point(self.right_joint_map, joint_names, point)

        goal_handle.succeed()
        self.get_logger().info('✅ 右臂轨迹执行完成')

        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = ''
        return result

    # ==========================
    #   资源清理
    # ==========================

    def destroy_node(self):
        self.get_logger().info('关闭串口并销毁节点')
        try:
            if self.port_handler is not None:
                self.port_handler.closePort()
        except Exception as e:
            self.get_logger().warn(f'关闭串口异常: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到 Ctrl+C，准备退出')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
