#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2, math
from pose_estimation import detect_people
from servo_controller import ServoController

PORT = "/dev/ttyACM0"
BAUD = 1000000
ID = 54

MID = 2048
ANGLE_REST = 160.0   # 站立时角度
ANGLE_MIN  = 30.0    # 最弯肘角度

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def calc_angle(a, b, c):
    ax, ay = a
    bx, by = b
    cx, cy = c
    v1 = (ax - bx, ay - by)
    v2 = (cx - bx, cy - by)
    dot = v1[0]*v2[0] + v1[1]*v2[1]
    mag1 = math.hypot(v1[0], v1[1])
    mag2 = math.hypot(v2[0], v2[1])
    if mag1 < 1e-6 or mag2 < 1e-6:
        return ANGLE_REST
    cosv = clamp(dot / (mag1 * mag2 + 1e-9), -1.0, 1.0)
    return math.degrees(math.acos(cosv))

sc = ServoController(PORT, BAUD)
print("✅ ID54 模仿模式启动（立正2048，弯肘→0）")

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise RuntimeError("❌ 摄像头打开失败")

# 先给中位
sc.write_pos(ID, MID)

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    out = detect_people(frame)
    ppl = out.get("people", [])

    if ppl:
        k = ppl[0]["kpts"]
        RS, RE, RW = k[6], k[8], k[10]

        if RS[2] > 0.2 and RE[2] > 0.2 and RW[2] > 0.2:
            elbow = calc_angle((RS[0],RS[1]), (RE[0],RE[1]), (RW[0],RW[1]))

            # 角度→舵机映射
            elbow = clamp(elbow, ANGLE_MIN, ANGLE_REST)

            # 完全伸直(160°)=2048
            # 完全弯曲(30°)=0
            ratio = (elbow - ANGLE_MIN) / (ANGLE_REST - ANGLE_MIN)
            target = int(ratio * MID)

            target = clamp(target, 0, 4095)
            sc.write_pos(ID, target)

            print(f"[肘角 {elbow:.1f}°] -> 舵机 {target}")

    cv2.imshow("follow", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
print("✅ 结束")
