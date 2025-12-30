#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import time

class VoiceControlNode(Node):
    def __init__(self):
        super().__init__('voice_control_node')
        
        self.subscription = self.create_subscription(
            String,
            '/voice/text',
            self.listener_callback,
            10)
        
        self.master_process = None
        self.slave_process = None

        self.get_logger().info('语音控制节点已就绪。正在过滤并等待指令...')

    def listener_callback(self, msg):
        raw_data = msg.data.strip()
        
        # === 核心修改：数据清洗逻辑 ===
        
        # 1. 如果这句话里没有 ">>"，说明不是用户说的话（可能是系统日志或机器回复），直接跳过
        if ">>" not in raw_data:
            return

        # 2. 提取 ">>" 后面的内容
        # 比如原句是: "I (117079) Application: >> 打开主机控制。"
        # 分割后变成: ["I (117079) Application: ", " 打开主机控制。"]
        parts = raw_data.split(">>")
        
        # 取最后一部分，并去掉首尾空格
        clean_command = parts[-1].strip()

        # 3. 去掉末尾可能的标点符号 (语音识别通常会加句号)
        clean_command = clean_command.replace("。", "").replace(".", "")

        self.get_logger().info(f'识别到有效指令: "{clean_command}"')

        # === 下面是控制逻辑 (判断 clean_command) ===

        # 1. 开启主机
        if clean_command == "打开主机控制":
            if self.master_process is None or self.master_process.poll() is not None:
                self.get_logger().info(">>> 正在启动主机控制 (Master Arms)...")
                try:
                    self.master_process = subprocess.Popen(
                        ['ros2', 'launch', 'xiaobei_arms', 'master_arms.launch.py']
                    )
                    self.get_logger().info(">>> 主机控制启动成功！")
                except Exception as e:
                    self.get_logger().error(f"启动失败: {e}")
            else:
                self.get_logger().warn("主机控制已经在运行中！")

        # 2. 停止主机
        elif clean_command == "停止主机控制":
            if self.master_process is not None and self.master_process.poll() is None:
                self.get_logger().info("<<< 正在停止主机控制...")
                self.master_process.terminate()
                self.master_process.wait()
                self.master_process = None
                self.get_logger().info("<<< 主机控制已停止。")
            else:
                self.get_logger().warn("主机控制当前未运行。")

        # 3. 开启从机
        elif clean_command == "打开从机跟随":
            if self.slave_process is None or self.slave_process.poll() is not None:
                self.get_logger().info(">>> 正在启动从机跟随 (Slave Arms)...")
                try:
                    self.slave_process = subprocess.Popen(
                        ['ros2', 'launch', 'xiaobei_arms', 'slave_arms.launch.py']
                    )
                    self.get_logger().info(">>> 从机跟随启动成功！")
                except Exception as e:
                    self.get_logger().error(f"启动失败: {e}")
            else:
                self.get_logger().warn("从机跟随已经在运行中！")

        # 4. 停止从机
        elif clean_command == "停止从机跟随":
            if self.slave_process is not None and self.slave_process.poll() is None:
                self.get_logger().info("<<< 正在停止从机跟随...")
                self.slave_process.terminate()
                self.slave_process.wait()
                self.slave_process = None
                self.get_logger().info("<<< 从机跟随已停止。")
            else:
                self.get_logger().warn("从机跟随当前未运行。")

def main(args=None):
    rclpy.init(args=args)
    node = VoiceControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.master_process:
            node.master_process.terminate()
        if node.slave_process:
            node.slave_process.terminate()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()