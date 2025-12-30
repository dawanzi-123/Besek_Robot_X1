#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class VoiceSerialNode(Node):
    def __init__(self):
        super().__init__('voice_serial_node')
        
        # 声明参数，方便在Launch文件中修改端口号和波特率
        self.declare_parameter('port', '/dev/ttyACM1')      # 默认端口，根据实际情况修改
        self.declare_parameter('baudrate', 115200)          # 默认波特率，一定要查你的模块说明书！

        port_name = self.get_parameter('port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baudrate').get_parameter_value().integer_value

        # 创建发布者，话题名为 /voice/text
        self.publisher_ = self.create_publisher(String, '/voice/text', 10)
        
        self.ser = None
        self.connect_serial(port_name, baud_rate)

        # 创建定时器，以一定频率读取串口（例如 0.1秒一次）
        self.timer = self.create_timer(0.1, self.read_from_serial)

    def connect_serial(self, port_name, baud_rate):
        """尝试连接串口"""
        try:
            self.ser = serial.Serial(port_name, baud_rate, timeout=1)
            self.get_logger().info(f'成功连接到语音模块: {port_name} (波特率: {baud_rate})')
        except serial.SerialException as e:
            self.get_logger().error(f'无法打开串口 {port_name}: {e}')
            self.get_logger().warn('请检查USB连接或权限 (sudo chmod 777 /dev/ttyUSB0)')

    def read_from_serial(self):
        """读取串口数据并发布"""
        if self.ser is not None and self.ser.is_open:
            try:
                # 检查缓冲区是否有数据
                if self.ser.in_waiting > 0:
                    # 读取一行数据，解码并去除首尾空格
                    # 注意：如果模块发送的是Hex数据而非字符串，这里需要用 ser.read() 并自行解析
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    if line:
                        self.get_logger().info(f'收到语音指令: "{line}"')
                        
                        # 构造ROS消息并发布
                        msg = String()
                        msg.data = line
                        self.publisher_.publish(msg)
            except Exception as e:
                self.get_logger().error(f'读取数据出错: {e}')
                # 如果出错尝试重连逻辑可以在这里写

def main(args=None):
    rclpy.init(args=args)
    node = VoiceSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser is not None:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()