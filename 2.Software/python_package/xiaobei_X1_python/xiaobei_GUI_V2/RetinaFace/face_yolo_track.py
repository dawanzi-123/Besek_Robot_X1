#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
人脸跟踪控制头部 (muse/face_yolov8n)
作者：BESEK Robotics
功能：
  - 检测人脸 (YOLOv8n)
  - 控制舵机 ID101（摇头） + ID102（点头）
修改：
  - 已优化路径加载，使用相对路径，方便移植
"""

import sys
import os

# 1. 优先使用当前文件夹下的 scservo_sdk
sys.path.append(os.getcwd()) 

import cv2
import time
from ultralytics import YOLO
# 确保这里能引用到 SDK
try:
    from scservo_sdk import *
except ImportError:
    print("❌ 找不到 scservo_sdk，请检查文件夹结构")
    exit()

# === [关键修改] 动态获取串口 ===
if len(sys.argv) > 1:
    PORT = sys.argv[1]   # 如果有参数，就用参数里的 (例如 COM3)
    print(f"🔗 接收到端口参数: {PORT}")
else:
    PORT = "COM3"        # 默认值 (你可以改成你常用的)
    print(f"⚠️ 未接收到参数，使用默认端口: {PORT}")

BAUD = 1000000
YAW_ID, PITCH_ID = 101, 102
SPEED, ACC = 1600, 150
port = PortHandler(PORT)
packet = sms_sts(port)
port.openPort(); port.setBaudRate(BAUD)
yaw_pos, pitch_pos = 2048, 2048

# === 模型路径 (动态获取) ===
# 获取当前脚本所在的绝对目录
current_dir = os.path.dirname(os.path.abspath(__file__))
# 拼接模型文件名 (确保 face_yolov8n.pt 在同级目录下)
model_path = os.path.join(current_dir, "face_yolov8n.pt")

if not os.path.exists(model_path):
    print(f"❌ 错误: 在以下路径找不到模型文件:\n{model_path}")
    print("👉 请确保 'face_yolov8n.pt' 文件和此脚本放在同一个文件夹内！")
    exit()

model = YOLO(model_path)
print(f"✅ 模型加载完成: {model_path}")

# === 摄像头 ===
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)
center_x, center_y = 320, 240

# === PID参数 ===
Kp_yaw, Kp_pitch = 0.25, 0.25
yaw_limit = (1500, 2600)
pitch_limit = (1800, 2600)

print("🚀 启动人脸跟踪 (按 Q 退出)")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 推理
    results = model.predict(frame, imgsz=640, conf=0.5, verbose=False)
    faces = results[0].boxes.xyxy.cpu().numpy() if len(results) else []

    if len(faces) > 0:
        # 取第一个人脸
        x1, y1, x2, y2 = map(int, faces[0][:4])
        cx, cy = (x1 + x2)//2, (y1 + y2)//2
        
        # 画框和中心点
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,255), 2)
        cv2.circle(frame, (cx, cy), 5, (0,0,255), -1)

        # 偏差计算
        dx, dy = cx - center_x, cy - center_y
        
        # P控制更新角度 (注意：Y轴通常是反向的，或者是根据安装方向调整加减)
        yaw_pos += int(Kp_yaw * dx)
        pitch_pos -= int(Kp_pitch * dy)
        
        # 限位保护
        yaw_pos = max(yaw_limit[0], min(yaw_limit[1], yaw_pos))
        pitch_pos = max(pitch_limit[0], min(pitch_limit[1], pitch_pos))

        # 发送指令
        packet.WritePosEx(YAW_ID, yaw_pos, SPEED, ACC)
        packet.WritePosEx(PITCH_ID, pitch_pos, SPEED, ACC)

    else:
        cv2.putText(frame, "No Face", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    cv2.imshow("face_yolov8n - Head Tracking", frame)
    
    # 按 Q 退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 资源释放
cap.release()
cv2.destroyAllWindows()
port.closePort()
print("✅ 程序结束，串口关闭")
