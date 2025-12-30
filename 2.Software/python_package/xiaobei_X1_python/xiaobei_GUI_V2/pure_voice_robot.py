#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import serial
import io

# ==========================================
# 核心修复：强行将标准输出设置为 UTF-8
# 只有加了这两行，Windows下打印 Emoji 才不会报错
# ==========================================
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8')

# 默认端口 (如果主程序没传参数，就用这个)
VOICE_PORT = 'COM17' 
if len(sys.argv) > 1 and sys.argv[1]:
    VOICE_PORT = sys.argv[1]

VOICE_BAUD = 115200

def main():
    # 现在这里打印 Emoji (🎤) 就不会崩了
    print(f"🎤 正在连接语音模块: {VOICE_PORT} ...")
    sys.stdout.flush() # 强制刷新，让主程序立刻看到
    
    try:
        # 只连接语音，不碰舵机！
        voice_ser = serial.Serial(VOICE_PORT, VOICE_BAUD, timeout=0.1)
        print("✅ 语音模块连接成功！正在监听...")
        print(">>> (请对模块说话)...")
        sys.stdout.flush()
        
        while True:
            if voice_ser.in_waiting:
                try:
                    line = voice_ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line: continue
                    
                    # 过滤掉杂讯，只提取指令
                    if len(line) > 1:
                         # 加一个前缀 VOICE_CMD 让主程序好识别
                        print(f"VOICE_CMD::{line}")
                        sys.stdout.flush() # 强制刷新缓冲区，确保主程序立刻收到
                        
                except Exception as e:
                    print(f"⚠️ 读取错误: {e}")
                    sys.stdout.flush()
            
            time.sleep(0.05)

    except Exception as e:
        print(f"❌ 语音模块连接失败: {e}")
        print("请检查端口是否被占用，或可以在上方重新选择端口。")
        sys.stdout.flush()

if __name__ == '__main__':
    main()