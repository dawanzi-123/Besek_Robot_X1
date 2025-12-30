#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import time
import threading
import json
import serial
import serial.tools.list_ports

# ==========================================
# 1. 动态加载 SDK
# ==========================================
current_dir = os.path.dirname(os.path.abspath(__file__))
sdk_path = os.path.join(current_dir, 'scservo_sdk')
sys.path.append(sdk_path)

try:
    from scservo_sdk import PortHandler, sms_sts, COMM_SUCCESS
except ImportError:
    print(f"❌ 错误：在 {sdk_path} 找不到 scservo_sdk！")
    sys.exit(1)

# ==========================================
# 2. 配置参数
# ==========================================
ROBOT_PORT = '/dev/ttyACM0' 
ROBOT_BAUD = 1000000
VOICE_PORT = '/dev/ttyACM1'
VOICE_BAUD = 115200

# 既然全是 STS，那就统一处理
LEFT_ARM_IDS  = list(range(1, 8))
RIGHT_ARM_IDS = list(range(51, 58))
ALL_IDS = LEFT_ARM_IDS + RIGHT_ARM_IDS

class PureRobotControl:
    def __init__(self):
        # === 连接机器人 ===
        print(f"🔌 正在连接机器人串口 {ROBOT_PORT}...")
        self.portHandler = PortHandler(ROBOT_PORT)
        self.sts = sms_sts(self.portHandler)
        
        if self.portHandler.openPort() and self.portHandler.setBaudRate(ROBOT_BAUD):
            print("✅ 机器人连接成功！(全 STS 模式)")
        else:
            print("❌ 机器人串口打开失败！")
            sys.exit(1)

        # === 连接语音 ===
        print(f"🎤 正在连接语音串口 {VOICE_PORT}...")
        try:
            self.voice_ser = serial.Serial(VOICE_PORT, VOICE_BAUD, timeout=0.1)
            print("✅ 语音模块连接成功！")
        except Exception as e:
            print(f"❌ 语音串口打开失败: {e}")
            sys.exit(1)

        self.actions_dir = os.path.join(current_dir, 'actions')
        os.makedirs(self.actions_dir, exist_ok=True)
        
        self.is_recording = False
        self.recording_name = ""
        self.recorded_data = []
        
        self.set_torque_all(False)
        print("\n>>> 🚀 系统就绪！[全STS模式] <<<")
        print(">>> 默认状态：[卸力模式] 可以掰动机器人")

        self.record_thread = threading.Thread(target=self.record_loop, daemon=True)
        self.record_thread.start()

        self.voice_loop()

    # ==========================
    # 底层控制 (全 STS)
    # ==========================
    def set_torque_all(self, enable):
        val = 1 if enable else 0
        for sid in ALL_IDS:
            try:
                # 统一使用 STS 协议开启/关闭扭矩 (地址 0x28)
                self.sts.write1ByteTxRx(sid, 0x28, val)
            except: pass
            time.sleep(0.001)
        status = "🔒 上力 (变硬)" if enable else "🔓 卸力 (变软)"
        print(f"⚙️ {status}")

    def read_pos(self, sid):
        # 统一使用 STS 协议读取位置
        pos, cr, err = self.sts.ReadPos(sid)
        if cr == COMM_SUCCESS and err == 0: 
            return pos
        return 2048 # 读不到返回中间值

    def write_pos(self, sid, pos):
        # 统一使用 STS 协议写入位置
        if pos < 0: pos = 0
        if pos > 4095: pos = 4095
        try:
            # STS 写入：ID, 位置, 速度(1500), 加速度(0)
            self.sts.WritePosEx(sid, int(pos), 1500, 0)
        except: pass

    # ==========================
    # 录制循环
    # ==========================
    def record_loop(self):
        while True:
            if self.is_recording:
                # 记录一帧
                left = [self.read_pos(sid) for sid in LEFT_ARM_IDS]
                right = [self.read_pos(sid) for sid in RIGHT_ARM_IDS]
                
                self.recorded_data.append({
                    "left": left,
                    "right": right,
                    "time": time.time()
                })
                time.sleep(0.033) # 30Hz
            else:
                time.sleep(0.1)

    # ==========================
    # 语音处理
    # ==========================
    def voice_loop(self):
        while True:
            try:
                if self.voice_ser.in_waiting:
                    line = self.voice_ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line: continue
                    if ">>" not in line: continue 
                    
                    cmd = line.split(">>")[-1].strip().replace("。", "").replace(".", "")
                    print(f"👂 收到指令: {cmd}")
                    self.process_command(cmd)
            except KeyboardInterrupt:
                break
            except Exception as e:
                time.sleep(1)

    def process_command(self, cmd):
        if "停" in cmd or "别动" in cmd:
            if self.is_recording: self.stop_recording()
            self.set_torque_all(False)

        elif "开始录制" in cmd and "动作" in cmd:
            name = cmd.replace("开始录制", "").replace("动作", "").strip()
            self.start_recording(name)

        elif "停止录制" in cmd or "结束录制" in cmd or "录制完成" in cmd:
            self.stop_recording()

        else:
            self.check_and_play(cmd)

    def start_recording(self, name):
        print(f"🔴 开始录制: [{name}]")
        self.set_torque_all(False)
        time.sleep(0.5)
        self.recorded_data = []
        self.recording_name = name
        self.is_recording = True

    def stop_recording(self):
        if self.is_recording:
            self.is_recording = False
            time.sleep(0.1) 
            
            filepath = os.path.join(self.actions_dir, f"{self.recording_name}.json")
            if self.recorded_data:
                start_t = self.recorded_data[0]['time']
                for frame in self.recorded_data:
                    if 'time' in frame:
                        frame['rel_time'] = frame['time'] - start_t
                        del frame['time']
            
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(self.recorded_data, f, ensure_ascii=False)
            print(f"💾 动作已保存: {filepath} ({len(self.recorded_data)}帧)")
        else:
            print("⚠️ 当前没有在录制")

    def check_and_play(self, text):
        if not os.path.exists(self.actions_dir): return
        for f in os.listdir(self.actions_dir):
            if f.endswith(".json"):
                name = f.replace(".json", "")
                if name in text:
                    threading.Thread(target=self.play_action, args=(f,)).start()
                    return

    def play_action(self, filename):
        filepath = os.path.join(self.actions_dir, filename)
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except: return

        if not data: return
        print(f"🎞️ 正在回放: [{filename}]")
        
        self.set_torque_all(True) 
        time.sleep(0.5)

        start_time = time.time()
        for frame in data:
            if not self.is_torque_on_check(): 
                print("⚠️ 回放被中断")
                return 

            if 'rel_time' not in frame: continue

            target_time = start_time + frame['rel_time']
            wait = target_time - time.time()
            if wait > 0: time.sleep(wait)

            for i, pos in enumerate(frame['left']):
                self.write_pos(LEFT_ARM_IDS[i], pos)
            for i, pos in enumerate(frame['right']):
                self.write_pos(RIGHT_ARM_IDS[i], pos)
        
        print("🏁 回放结束 -> 自动卸力")
        self.set_torque_all(False)

    def is_torque_on_check(self):
        return not self.is_recording

if __name__ == '__main__':
    PureRobotControl()
