#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
贝塞克机器人控制面板 - 旗舰版
1. 核心修复：线程锁、自动扭矩、垃圾回收修复。
2. 界面布局：经典三列布局。
3. 新增功能：启动时的进度条加载画面 (Splash Screen)。
4. 修改记录：ID7和ID57夹爪全部改为STS舵机。
"""

import time
import threading
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import serial.tools.list_ports
import serial
import cv2
from PIL import Image, ImageTk
import sys
import os

# ==========================================
# ⬇️ 核心修改：PyInstaller 资源路径修复
# ==========================================
def get_resource_path(relative_path):
    if hasattr(sys, '_MEIPASS'):
        return os.path.join(sys._MEIPASS, relative_path)
    return os.path.join(os.path.abspath("."), relative_path)

sdk_path = get_resource_path('scservo_sdk')
sys.path.append(sdk_path)

# ==========================================
# ⬇️ 核心修改：全局线程锁
# ==========================================
serial_lock = threading.Lock()


# ===== 参数 =====
BAUDRATE    = 1000000
SPEED_VAL   = 1500
ACC_VAL     = 0
REVERSE_SLAVES = {51, 52, 54, 57}
MAX_POS     = 4095

LEFT_ARM_IDS  = list(range(1, 8))
RIGHT_ARM_IDS = list(range(51, 58))

# ===== 全局状态 =====
teleop_running = True
log_update_running = True
web_target_positions = {i: 2048 for i in RIGHT_ARM_IDS}

left_recording  = False
right_recording = False
left_trajectory  = []
right_trajectory = []

slider_vars_left = {}
slider_vars_right = {}
port_var = None
log_text = None

# ===== 全局串口对象 =====
port = None
sts  = None
scs  = None

try:
    from scservo_sdk import PortHandler, sms_sts, scscl, COMM_SUCCESS
except ImportError:
    messagebox.showerror("错误", "请确认 scservo_sdk 目录存在且包含所有 .py 文件")
    sys.exit(1)


# ============================
# 串口初始化
# ============================
def check_serial_port():
    global port, sts, scs
    if port_var is None: return False
    selected_port = port_var.get()
    
    if not selected_port or not (selected_port.startswith("COM") or selected_port.startswith("/dev/")):
        messagebox.showwarning("警告", "请先选择一个有效的串口！")
        return False

    if port and hasattr(port, 'is_open') and port.is_open:
        return True

    try:
        with serial_lock: 
            port = PortHandler(selected_port)
            sts  = sms_sts(port)
            scs  = scscl(port)
            if not port.openPort():
                messagebox.showerror("串口错误", f"无法打开串口 {selected_port}")
                return False
            if not port.setBaudRate(BAUDRATE):
                messagebox.showerror("串口错误", f"无法设置波特率 {BAUDRATE}")
                return False
        
        print(f"✅ 串口 {selected_port} 初始化成功")
        return True
    except Exception as e:
        messagebox.showerror("串口错误", f"串口初始化失败: {e}")
        return False


# ============================
# 读写舵机
# ============================
def read_pos_sts(servo_id: int):
    global port, sts, serial_lock
    if not port or not port.is_open: return None
    with serial_lock:
        try:
            pos, cr, err = sts.ReadPos(servo_id)
            if cr == COMM_SUCCESS and err == 0: return pos
        except: pass
    return None

def read_pos_scs(servo_id: int):
    global port, scs, serial_lock
    if not port or not port.is_open: return None
    with serial_lock:
        try:
            pos, spd, cr, err = scs.ReadPosSpeed(servo_id)
            if cr == COMM_SUCCESS and err == 0: return pos
        except: pass
    return None

def write_pos_to_id(servo_id: int, pos: int):
    global port, sts, scs, serial_lock
    if not port or not port.is_open: return
    with serial_lock:
        try:
            if pos < 0: pos = 0
            elif pos > MAX_POS: pos = MAX_POS
            sts.WritePosEx(servo_id, int(pos), SPEED_VAL, ACC_VAL)
        except Exception as e:
            print(f"[ERROR] Write ID{servo_id}: {e}")

def set_torque(servo_ids, enable: bool):
    global port, sts, scs, serial_lock
    if not port or not port.is_open: return
    with serial_lock:
        for sid in servo_ids:
            try:
                # 修改：全部统一为 STS 舵机 (ID 7 和 57 均使用 sts 对象)
                sts.write1ByteTxRx(sid, 0x28, 1 if enable else 0)
            except: pass
            time.sleep(0.001)

def init_sts(servo_id: int, torque_on: bool):
    global port, sts, serial_lock
    if not port or not port.is_open: return
    with serial_lock:
        try:
            sts.write1ByteTxRx(servo_id, 0x37, 0)
            sts.write1ByteTxRx(servo_id, 0x21, 0)
            sts.write1ByteTxRx(servo_id, 0x28, 1 if torque_on else 0)
        except: pass
    time.sleep(0.002)

def init_scs(servo_id: int, torque_on: bool):
    global port, scs, serial_lock
    if not port or not port.is_open: return
    with serial_lock:
        try:
            scs.write1ByteTxRx(servo_id, 0x28, 1 if torque_on else 0)
        except: pass
    time.sleep(0.002)


# ============================
# 后台线程
# ============================
def teleop_loop():
    global teleop_running
    pairs = [
        (read_pos_sts, 1, write_pos_to_id, 51),
        (read_pos_sts, 2, write_pos_to_id, 52),
        (read_pos_sts, 3, write_pos_to_id, 53),
        (read_pos_sts, 4, write_pos_to_id, 54),
        (read_pos_sts, 5, write_pos_to_id, 55),
        (read_pos_sts, 6, write_pos_to_id, 56),
        (read_pos_sts, 7, write_pos_to_id, 57),
    ]
    while True:
        if not teleop_running:
            time.sleep(0.1)
            continue
        
        cache = []
        for rf, mid, wf, sid in pairs:
            p = rf(mid)
            cache.append((wf, sid, p))
        
        for wf, sid, p in cache:
            if p is not None:
                if sid in REVERSE_SLAVES:
                    p = MAX_POS - p
                wf(sid, p)
        time.sleep(0.005)

def record_loop():
    global left_recording, right_recording
    global left_trajectory, right_trajectory
    last_left = 0
    last_right = 0
    while True:
        now = time.time()
        if left_recording and now - last_left >= 0.05:
            pos_list = []
            for sid in LEFT_ARM_IDS:
                # 修改：ID 7 改为使用 read_pos_sts (全系STS)
                p = read_pos_sts(sid)
                pos_list.append(p if p is not None else 2048)
            left_trajectory.append((now, pos_list))
            last_left = now

        if right_recording and now - last_right >= 0.05:
            pos_list = [read_pos_sts(sid) or 2048 for sid in RIGHT_ARM_IDS]
            right_trajectory.append((now, pos_list))
            last_right = now
        time.sleep(0.01)

threading.Thread(target=teleop_loop, daemon=True).start()
threading.Thread(target=record_loop, daemon=True).start()


# ============================
# 按钮逻辑
# ============================
def append_log(msg):
    if log_text:
        log_text.insert(tk.END, msg + "\n")
        log_text.see(tk.END)

def set_web_mode():
    if not check_serial_port(): return
    global teleop_running
    teleop_running = False  
    
    append_log("⚙️ 开启全机扭矩，准备执行指令...")
    set_torque(LEFT_ARM_IDS + RIGHT_ARM_IDS, enable=True)
    time.sleep(0.1)

    for sid in LEFT_ARM_IDS:
        val = slider_vars_left[sid].get()
        write_pos_to_id(sid, val)

    for sid in RIGHT_ARM_IDS:
        val = slider_vars_right[sid].get()
        if sid in REVERSE_SLAVES: pos = MAX_POS - val
        else: pos = val
        write_pos_to_id(sid, pos)

    append_log("✅ 指令已发送 (保持位置)")

def reset_to_middle():
    if not check_serial_port(): return
    global teleop_running
    teleop_running = False

    append_log("⚙️ 开启全机扭矩，正在复位...")
    set_torque(LEFT_ARM_IDS + RIGHT_ARM_IDS, enable=True)
    time.sleep(0.1)

    for sid in LEFT_ARM_IDS + RIGHT_ARM_IDS:
        target = 2048
        pos = MAX_POS - target if sid in REVERSE_SLAVES else target
        write_pos_to_id(sid, pos)
        if sid in slider_vars_left: slider_vars_left[sid].set(2048)
        if sid in slider_vars_right: slider_vars_right[sid].set(2048)
        
    append_log("🔄 已复位到中间")

def emergency_stop():
    if not check_serial_port(): return
    global teleop_running
    teleop_running = False

    append_log("⚠️ 急停：全机卸力...")
    for sid in LEFT_ARM_IDS + RIGHT_ARM_IDS:
        # 修改：全部统一为 STS 卸力 (移除 SCS 判断)
        init_sts(sid, torque_on=False)
    
    append_log("🛑 急停完毕")

def resume_teleop():
    if not check_serial_port(): return
    global teleop_running

    append_log("⚙️ 恢复遥操作配置...")
    set_torque(LEFT_ARM_IDS, enable=False)
    set_torque(RIGHT_ARM_IDS, enable=True)
    
    teleop_running = True
    append_log("🔁 遥操作已恢复 (左软右硬)")


# ============================
# 录制逻辑
# ============================
def start_record_left():
    if not check_serial_port(): return
    global left_recording, left_trajectory
    left_trajectory = []
    left_recording = True
    append_log("⏺️ 左臂开始录制...")

def stop_record_left():
    if not check_serial_port(): return
    global left_recording
    left_recording = False
    append_log(f"⏹️ 左臂录制结束 ({len(left_trajectory)}帧)")

def playback_left():
    if not check_serial_port(): return
    global teleop_running
    if not left_trajectory:
        append_log("⚠️ 无数据")
        return
    teleop_running = False
    set_torque(LEFT_ARM_IDS, enable=True)
    
    record_start = left_trajectory[0][0]
    play_start = time.time()
    
    for t, pos_list in left_trajectory:
        target_delay = t - record_start
        current_elapsed = time.time() - play_start
        sleep_dur = target_delay - current_elapsed
        if sleep_dur > 0: time.sleep(sleep_dur)
        for i, sid in enumerate(LEFT_ARM_IDS):
            write_pos_to_id(sid, pos_list[i])
            
    set_torque(LEFT_ARM_IDS, enable=False)
    append_log("▶️ 左臂回放完毕")

def start_record_right():
    if not check_serial_port(): return
    global right_recording, right_trajectory
    right_trajectory = []
    right_recording = True
    set_torque(RIGHT_ARM_IDS, enable=False)
    append_log("⏺️ 右臂开始录制...")

def stop_record_right():
    if not check_serial_port(): return
    global right_recording
    right_recording = False
    set_torque(RIGHT_ARM_IDS, enable=True)
    append_log(f"⏹️ 右臂录制结束 ({len(right_trajectory)}帧)")

def playback_right():
    if not check_serial_port(): return
    global teleop_running
    if not right_trajectory:
        append_log("⚠️ 无数据")
        return
    teleop_running = False
    set_torque(RIGHT_ARM_IDS, enable=True)
    
    record_start = right_trajectory[0][0]
    play_start = time.time()
    
    for t, pos_list in right_trajectory:
        target_delay = t - record_start
        current_elapsed = time.time() - play_start
        sleep_dur = target_delay - current_elapsed
        if sleep_dur > 0: time.sleep(sleep_dur)
        for i, sid in enumerate(RIGHT_ARM_IDS):
            write_pos_to_id(sid, pos_list[i])
            
    set_torque(RIGHT_ARM_IDS, enable=False)
    append_log("▶️ 右臂回放完毕")


# ==========================================================
# 摄像头页
# ==========================================================
class CameraPage(ttk.Frame):
    def __init__(self, master, go_back_callback,
                 head_cam=0, left_cam=1, right_cam=2):
        super().__init__(master)
        self.go_back_callback = go_back_callback
        self.head_cam = head_cam
        self.left_cam = left_cam
        self.right_cam = right_cam

        topbar = ttk.Frame(self)
        topbar.pack(fill='x', pady=5, padx=5)
        ttk.Label(topbar, text="Camera", font=("Arial", 16, "bold")).pack(side='left')
        ttk.Button(topbar, text="⬅ BACK", command=self.on_back_clicked).pack(side='right')

        self.head_label = tk.Label(self, bg="black")
        self.head_label.pack(pady=5)

        bottom_frame = ttk.Frame(self)
        bottom_frame.pack(pady=5)
        self.left_label = tk.Label(bottom_frame, bg="black")
        self.left_label.pack(side='left', padx=5)
        self.right_label = tk.Label(bottom_frame, bg="black")
        self.right_label.pack(side='left', padx=5)

        self.cap_head = None
        self.cap_left = None
        self.cap_right = None
        self.running = False
        self.thread = None
        self.img_head = None
        self.img_left = None
        self.img_right = None

    def start_camera(self):
        if self.running: return
        self.cap_head = cv2.VideoCapture(self.head_cam)
        self.cap_left = cv2.VideoCapture(self.left_cam)
        self.cap_right = cv2.VideoCapture(self.right_cam)
        self.running = True
        self.thread = threading.Thread(target=self._cam_loop, daemon=True)
        self.thread.start()

    def stop_camera(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=0.5)
        for cap in [self.cap_head, self.cap_left, self.cap_right]:
            if cap: cap.release()
        self.cap_head = self.cap_left = self.cap_right = None

    def on_back_clicked(self):
        self.stop_camera()
        self.go_back_callback()

    def _cam_loop(self):
        while self.running:
            ret_h, fh = (self.cap_head.read() if self.cap_head else (False, None))
            ret_l, fl = (self.cap_left.read() if self.cap_left else (False, None))
            ret_r, fr = (self.cap_right.read() if self.cap_right else (False, None))
            if any([ret_h, ret_l, ret_r]):
                self.after(0, self._update_frames, fh, fl, fr)
            time.sleep(0.03)

    def _update_frames(self, fh, fl, fr):
        def make_img(frame, size, tag):
            if frame is None: return None
            frame = cv2.resize(frame, size)
            cv2.putText(frame, tag, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            return ImageTk.PhotoImage(image=Image.fromarray(rgb))
        
        if fh is not None: 
            img = make_img(fh, (960, 540), "HEAD")
            self.head_label.config(image=img)
            self.img_head = img 

        if fl is not None: 
            img = make_img(fl, (480, 270), "L")
            self.left_label.config(image=img)
            self.img_left = img

        if fr is not None: 
            img = make_img(fr, (480, 270), "R")
            self.right_label.config(image=img)
            self.img_right = img


# ==========================================================
# 控制页 Frame
# ==========================================================
class ControlPage(ttk.Frame):
    def __init__(self, master, go_camera_callback):
        super().__init__(master)
        self.go_camera_callback = go_camera_callback
        self.build_ui()

    def build_ui(self):
        global port_var, port_combo, slider_vars_left, slider_vars_right, log_text

        # 顶部标题行
        top_title_frame = ttk.Frame(self)
        top_title_frame.pack(fill='x', pady=5, padx=5)

        top_title_frame.columnconfigure(0, weight=1)
        top_title_frame.columnconfigure(1, weight=1)
        top_title_frame.columnconfigure(2, weight=1)

        tk.Label(top_title_frame, text="").grid(row=0, column=0, sticky="w")
        tk.Label(top_title_frame, text="🤖 贝塞克机器人控制面板", font=("Arial", 16, "bold")).grid(row=0, column=1)
        ttk.Button(top_title_frame, text="📷 CAMERA", command=self.go_camera_callback).grid(row=0, column=2, sticky="e")

        desc_label = tk.Label(self, text="📌 默认：遥操作模式（手拽主臂） | 点击下方按钮切换模式", fg="gray")
        desc_label.pack(anchor='center')

        # 上半部分：串口+控制区
        control_block = ttk.Frame(self)
        control_block.pack(pady=10, fill='x')

        port_select_row = ttk.Frame(control_block)
        port_select_row.pack(fill='x', pady=5, padx=5, anchor='w')

        ttk.Label(port_select_row, text="串口:").pack(side='left')
        port_var = tk.StringVar()
        port_combo = ttk.Combobox(port_select_row, textvariable=port_var, width=10)
        port_combo.pack(side='left', padx=5)

        def refresh_ports():
            ports = [p.device for p in serial.tools.list_ports.comports()]
            port_combo['values'] = ports
            if ports: port_var.set(ports[0])
            else: port_var.set("无串口")
        refresh_ports()

        ttk.Button(port_select_row, text="🔄 刷新", command=refresh_ports).pack(side='left', padx=5)

        triple_area = ttk.Frame(control_block)
        triple_area.pack(pady=10)

        triple_area.columnconfigure(0, weight=1)
        triple_area.columnconfigure(1, weight=1)
        triple_area.columnconfigure(2, weight=1)

        left_frame = ttk.LabelFrame(triple_area, text="左臂 (ID 1~7)")
        left_frame.grid(row=0, column=0, padx=20, sticky='n')

        for sid in LEFT_ARM_IDS:
            rowf = ttk.Frame(left_frame)
            rowf.pack(pady=4, anchor='w')
            tk.Label(rowf, text=f"舵机 {sid}:").pack(side='left')
            v = tk.IntVar(value=2048)
            slider_vars_left[sid] = v
            ttk.Scale(rowf, from_=0, to=MAX_POS, variable=v, orient='horizontal', length=200).pack(side='left', padx=5)
            tk.Entry(rowf, textvariable=v, width=6).pack(side='left')

        btn_frame = ttk.Frame(triple_area)
        btn_frame.grid(row=0, column=1, padx=20, sticky='n')

        ttk.Button(btn_frame, text="🚀 发送指令（进入网页控制）", command=set_web_mode).pack(pady=5, fill='x')
        ttk.Button(btn_frame, text="🔄 复位到中间", command=reset_to_middle).pack(pady=5, fill='x')
        ttk.Button(btn_frame, text="🛑 急停", command=emergency_stop).pack(pady=5, fill='x')
        ttk.Button(btn_frame, text="🔁 恢复遥操作", command=resume_teleop).pack(pady=5, fill='x')

        right_frame = ttk.LabelFrame(triple_area, text="右臂 (ID 51~57)")
        right_frame.grid(row=0, column=2, padx=20, sticky='n')

        for sid in RIGHT_ARM_IDS:
            rowf = ttk.Frame(right_frame)
            rowf.pack(pady=4, anchor='w')
            tk.Label(rowf, text=f"舵机 {sid}:").pack(side='left')
            v = tk.IntVar(value=2048)
            slider_vars_right[sid] = v
            ttk.Scale(rowf, from_=0, to=MAX_POS, variable=v, orient='horizontal', length=200).pack(side='left', padx=5)
            tk.Entry(rowf, textvariable=v, width=6).pack(side='left')

        tk.Label(self, text="系统日志:", font=("Arial", 10)).pack(anchor='w', padx=10)
        log_text = scrolledtext.ScrolledText(self, height=10, width=50, state='normal', bg="black", fg="white", font=("Courier", 10))
        log_text.pack(padx=10, fill='y', expand=True, anchor='w')

        record_block = ttk.Frame(self)
        record_block.pack(pady=10)

        left_rec = ttk.Frame(record_block)
        left_rec.pack(side='left', padx=40)
        ttk.Label(left_rec, text="#### 左臂（ID 1~7）", font=("Arial", 12)).pack()
        ttk.Button(left_rec, text="⏺ 开始录制left", command=start_record_left).pack(pady=3)
        ttk.Button(left_rec, text="⏹ 结束录制left", command=stop_record_left).pack(pady=3)
        ttk.Button(left_rec, text="▶ 回放left",   command=playback_left).pack(pady=3)

        right_rec = ttk.Frame(record_block)
        right_rec.pack(side='left', padx=40)
        ttk.Label(right_rec, text="#### 右臂（ID 51~57）", font=("Arial", 12)).pack()
        ttk.Button(right_rec, text="⏺ 开始录制right", command=start_record_right).pack(pady=3)
        ttk.Button(right_rec, text="⏹ 结束录制right", command=stop_record_right).pack(pady=3)
        ttk.Button(right_rec, text="▶ 回放right",   command=playback_right).pack(pady=3)


# ==========================================================
# 主窗口
# ==========================================================
class MainApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("贝塞克机器人控制面板")
        self.geometry("1000x700")
        
        # 居中显示
        sw = self.winfo_screenwidth()
        sh = self.winfo_screenheight()
        w, h = 1000, 700
        x = (sw - w) // 2
        y = (sh - h) // 2
        self.geometry(f"{w}x{h}+{x}+{y}")

        self.control_page = ControlPage(self, go_camera_callback=self.show_camera_page)
        self.camera_page  = CameraPage(self, go_back_callback=self.show_control_page)
        self.control_page.pack(fill='both', expand=True)

        self.start_log_update_thread()
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def start_log_update_thread(self):
        def log_loop():
            last_positions = {}
            while True:
                if not log_update_running:
                    time.sleep(0.3)
                    continue
                current_positions = {}
                for sid in LEFT_ARM_IDS + RIGHT_ARM_IDS:
                    p = read_pos_sts(sid)
                    if p is not None: current_positions[sid] = p
                
                if current_positions != last_positions:
                    pos_str = ", ".join([f"ID{sid}:{pos}" for sid, pos in sorted(current_positions.items())])
                    self.after(0, lambda s=pos_str: append_log(f"🔄 实时: {s}"))
                    last_positions = current_positions
                time.sleep(0.3)
        thread = threading.Thread(target=log_loop, daemon=True)
        thread.start()

    def show_camera_page(self):
        self.control_page.pack_forget()
        self.camera_page.pack(fill='both', expand=True)
        self.camera_page.start_camera()

    def show_control_page(self):
        self.camera_page.stop_camera()
        self.camera_page.pack_forget()
        self.control_page.pack(fill='both', expand=True)

    def on_close(self):
        global log_update_running, port
        self.camera_page.stop_camera()
        log_update_running = False
        if port and hasattr(port, 'is_open') and port.is_open:
            try: port.closePort()
            except: pass
        self.destroy()


# ==========================================================
# ⬇️ 新增：启动加载页 (Splash Screen) 函数
# ==========================================================
def show_loading_screen():
    splash_root = tk.Tk()
    splash_root.overrideredirect(True) # 无边框
    
    # 设置加载窗口大小和位置
    width, height = 400, 150
    screen_width = splash_root.winfo_screenwidth()
    screen_height = splash_root.winfo_screenheight()
    x = (screen_width - width) // 2
    y = (screen_height - height) // 2
    splash_root.geometry(f"{width}x{height}+{x}+{y}")
    
    # 样式
    splash_root.configure(bg="#2b2b2b")
    
    tk.Label(splash_root, text="贝塞克机器人控制系统", font=("Arial", 16, "bold"), 
             bg="#2b2b2b", fg="white").pack(pady=(20, 10))
    
    status_label = tk.Label(splash_root, text="正在初始化...", font=("Arial", 10), 
                            bg="#2b2b2b", fg="#cccccc")
    status_label.pack(pady=5)
    
    progress = ttk.Progressbar(splash_root, orient="horizontal", length=300, mode="determinate")
    progress.pack(pady=10)
    
    # 模拟加载过程
    steps = [
        (10, "加载核心库..."),
        (30, "初始化 UI 组件..."),
        (50, "检查串口驱动..."),
        (70, "加载视觉模块..."),
        (90, "正在启动主程序..."),
        (100, "准备就绪！")
    ]
    
    for val, text in steps:
        progress["value"] = val
        status_label.config(text=text)
        splash_root.update()
        time.sleep(0.2) # 假装在忙，让用户看清楚
        
    splash_root.destroy() # 关闭加载页

# ==========================================================
# 程序入口
# ==========================================================
if __name__ == "__main__":
    # 1. 先显示加载页
    show_loading_screen()
    
    # 2. 只有加载页关闭后，才会运行主程序
    app = MainApp()
    app.mainloop()
    print("✅ 程序安全退出")