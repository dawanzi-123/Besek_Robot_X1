import sys
import os

# 屏蔽 OpenCV 烦人的控制台报错
os.environ["OPENCV_LOG_LEVEL"] = "OFF"

import time
import threading
import cv2
import json  # JSON 支持
import serial
import serial.tools.list_ports
import numpy as np # 数学库
from stl import mesh # STL模型加载库

# 3D 图形库
import pyqtgraph.opengl as gl

# 引入 PyQt6 组件
from PyQt6.QtWidgets import (QApplication, QMainWindow, QPushButton, QLabel, 
                             QVBoxLayout, QWidget, QHBoxLayout, QFrame, 
                             QStackedWidget, QTextEdit, QComboBox, 
                             QSlider, QGridLayout, QGroupBox, QScrollArea, 
                             QSizePolicy, QLineEdit, QTabWidget, QProgressBar)
# 引入 QMatrix4x4 用于精确计算坐标
from PyQt6.QtGui import QFont, QKeyEvent, QImage, QPixmap, QVector3D, QMatrix4x4
from PyQt6.QtCore import Qt, QTimer, QTime, QProcess, pyqtSignal, QObject, QThread

# ==========================================
# [关键新增] 引入我们改写好的子模块
# ==========================================
import pure_voice_robot

# 注意：如果你也修改了视觉脚本 (例如 face_yolo_track.py)，请在这里 import 它们
# from RetinaFace import face_yolo_track 
# from human_mimic_demo import mimic_human_pose_V2

# ==========================================
# 1. 满血版底层后端 (全 STS 协议 + 头部支持)
# ==========================================
def get_resource_path(relative_path):
    """ 获取资源的绝对路径，兼容开发环境和 PyInstaller 打包后的环境 """
    if hasattr(sys, '_MEIPASS'):
        # PyInstaller 打包后，文件解压到 sys._MEIPASS
        return os.path.join(sys._MEIPASS, relative_path)
    return os.path.join(os.path.abspath("."), relative_path)

# [修改] 使用新的路径获取方式
current_dir = get_resource_path(".")
sdk_path = os.path.join(current_dir, 'scservo_sdk')
sys.path.append(sdk_path)

try:
    from scservo_sdk import PortHandler, sms_sts, COMM_SUCCESS
except ImportError:
    pass 

# 舵机 ID 配置
LEFT_ARM_IDS  = list(range(1, 8))   
RIGHT_ARM_IDS = list(range(51, 58)) 
HEAD_IDS      = [101, 102]          

ALL_IDS = LEFT_ARM_IDS + RIGHT_ARM_IDS + HEAD_IDS
REVERSE_SLAVES = {51, 52, 54, 57}   
MAX_POS = 4095

class ServoBackend(QObject):
    log_signal = pyqtSignal(str) 

    def __init__(self):
        super().__init__()
        self.port = None
        self.sts = None
        self.lock = threading.Lock()
        self.teleop_running = False
        self.recording = False 
        self.left_traj = []
        self.right_traj = []
        self.head_traj = []

        threading.Thread(target=self.teleop_loop, daemon=True).start()
        threading.Thread(target=self.record_loop, daemon=True).start()

    def connect_serial(self, port_name):
        # 连接前先确保断开旧连接
        self.disconnect_serial()
        try:
            with self.lock:
                self.port = PortHandler(port_name)
                self.sts = sms_sts(self.port) 
                if self.port.openPort() and self.port.setBaudRate(1000000):
                    self.log_signal.emit(f"✅ 串口 {port_name} 连接成功 (含头部)")
                    return True
                else:
                    self.log_signal.emit("❌ 串口打开失败")
                    return False
        except Exception as e:
            self.log_signal.emit(f"❌ 连接异常: {e}")
            return False

    # [新增] 主动断开串口，释放资源给外部脚本使用
    def disconnect_serial(self):
        with self.lock:
            if self.port and self.port.is_open:
                try:
                    self.port.closePort()
                    self.log_signal.emit("🔌 串口已断开 (资源释放)")
                except: pass
            self.port = None

    def read_pos(self, sid):
        if not self.port or not self.port.is_open: return None
        with self.lock:
            try:
                pos, cr, err = self.sts.ReadPos(sid)
                if cr == COMM_SUCCESS and err == 0: return pos
            except: pass
        return None

    def write_pos(self, sid, pos, speed=1500):
        if not self.port or not self.port.is_open: return
        if pos < 0: pos = 0
        if pos > MAX_POS: pos = MAX_POS
        with self.lock:
            try:
                self.sts.WritePosEx(sid, int(pos), int(speed), 0)
            except: pass

    def set_torque(self, ids, enable):
        if not self.port or not self.port.is_open: return
        val = 1 if enable else 0
        with self.lock:
            for sid in ids:
                try:
                    self.sts.write1ByteTxRx(sid, 0x28, val)
                except: pass
                time.sleep(0.001)
        status = "上力" if enable else "卸力"
        self.log_signal.emit(f"⚙️ {ids} {status}")

    def teleop_loop(self):
        pairs = [(1, 51), (2, 52), (3, 53), (4, 54), (5, 55), (6, 56), (7, 57)]
        while True:
            if not self.teleop_running:
                time.sleep(0.1)
                continue
            for mid, sid in pairs:
                p = self.read_pos(mid)
                if p is not None:
                    target = MAX_POS - p if sid in REVERSE_SLAVES else p
                    self.write_pos(sid, target)
            time.sleep(0.005)

    def record_loop(self):
        last_time = 0
        while True:
            if self.recording:
                now = time.time()
                if now - last_time > 0.04: 
                    l_row = [self.read_pos(sid) or 2048 for sid in LEFT_ARM_IDS]
                    self.left_traj.append(l_row)
                    r_row = [self.read_pos(sid) or 2048 for sid in RIGHT_ARM_IDS]
                    self.right_traj.append(r_row)
                    h_row = [self.read_pos(sid) or 2048 for sid in HEAD_IDS]
                    self.head_traj.append(h_row)
                    last_time = now
            time.sleep(0.01)

    def play_data(self, data):
        self.teleop_running = False
        self.set_torque(ALL_IDS, True) 
        self.log_signal.emit("▶ 开始回放动作...")
        start_t = time.time()
        for frame in data:
            if 'rel_time' in frame:
                target_time = start_t + frame['rel_time']
                wait = target_time - time.time()
                if wait > 0: time.sleep(wait)
            else:
                time.sleep(0.04) 
            if 'left' in frame:
                for i, pos in enumerate(frame['left']):
                    if i < len(LEFT_ARM_IDS): self.write_pos(LEFT_ARM_IDS[i], pos)
            if 'right' in frame:
                for i, pos in enumerate(frame['right']):
                    if i < len(RIGHT_ARM_IDS): self.write_pos(RIGHT_ARM_IDS[i], pos)
            if 'head' in frame:
                for i, pos in enumerate(frame['head']):
                    if i < len(HEAD_IDS): self.write_pos(HEAD_IDS[i], pos)
        self.set_torque(ALL_IDS, False) 
        self.log_signal.emit("🏁 动作回放结束")

backend = ServoBackend()

# ==========================================
# 2. 摄像头系统 (三摄 + 热插拔)
# ==========================================

class VideoLabel(QLabel):
    def __init__(self, name):
        super().__init__()
        self.name = name
        self.setText(f"{name}\n(No Signal)")
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setStyleSheet("background-color: #000; border: 2px solid #333; color: #555; font-weight: bold;")
        self.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
        self.setScaledContents(False) 

    def update_image(self, qt_img):
        if qt_img.isNull():
            return
        scaled = qt_img.scaled(self.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
        self.setPixmap(QPixmap.fromImage(scaled))

class CameraWorker(QThread):
    frame_signal = pyqtSignal(int, QImage)
    
    def __init__(self):
        super().__init__()
        self.running = True
        self.caps = {0: None, 1: None, 2: None} 
        self.last_scan = 0
    
    def run(self):
        self.last_scan = 0 
        while self.running:
            now = time.time()
            if now - self.last_scan > 1.5:
                self.scan_cameras()
                self.last_scan = now
            
            for idx in list(self.caps.keys()):
                cap = self.caps[idx]
                if cap is not None and cap.isOpened():
                    ret, frame = cap.read()
                    if ret:
                        try:
                            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                            h, w, ch = frame.shape
                            qt_img = QImage(frame.data, w, h, ch * w, QImage.Format.Format_RGB888)
                            self.frame_signal.emit(idx, qt_img.copy())
                        except: pass
                    else:
                        self.close_cap(idx)
                        self.frame_signal.emit(idx, QImage())
                else:
                    self.frame_signal.emit(idx, QImage())
            time.sleep(0.03)

    def scan_cameras(self):
        for idx in [0, 1, 2]:
            if self.caps[idx] is None:
                try:
                    cap = cv2.VideoCapture(idx)
                    if cap.isOpened():
                        self.caps[idx] = cap
                    else:
                        cap.release()
                except: pass

    def close_cap(self, idx):
        if self.caps[idx]:
            try:
                self.caps[idx].release()
            except: pass
        self.caps[idx] = None

    def stop(self):
        self.running = False
        self.wait()
        for idx in self.caps:
            self.close_cap(idx)

class MultiCameraSystem(QWidget):
    def __init__(self):
        super().__init__()
        layout = QHBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(10)
        
        self.screens = {}
        configs = [(0, "HEAD CAM"), (1, "LEFT CAM"), (2, "RIGHT CAM")]
        
        for idx, name in configs:
            lbl = VideoLabel(name)
            layout.addWidget(lbl)
            self.screens[idx] = lbl
            lbl.hide()
        
        self.worker = CameraWorker()
        self.worker.frame_signal.connect(self.update_feed)
        
    def start(self):
        if not self.worker.isRunning():
            self.worker.running = True
            self.worker.start()
            
    def stop(self):
        self.worker.stop()
        
    def update_feed(self, idx, qt_img):
        if idx in self.screens:
            lbl = self.screens[idx]
            if qt_img.isNull():
                lbl.hide()
            else:
                lbl.show()
                lbl.update_image(qt_img)

# ==========================================
# 3D 机器人视图 (矩阵驱动模式 + 白色背景)
# ==========================================
class Robot3DViewer(gl.GLViewWidget):
    def __init__(self):
        super().__init__()
        # 调整相机参数
        self.opts['distance'] = 1.2
        self.opts['fov'] = 60
        self.opts['elevation'] = 15
        self.opts['azimuth'] = -90
        self.opts['center'] = QVector3D(0, 0, 0.4) 
        
        # [修改] 设置为白色背景 (纯白)
        self.setBackgroundColor('w') 
        
        # 地面
        g = gl.GLGridItem()
        g.setSize(x=2, y=2, z=0)
        g.setSpacing(x=0.1, y=0.1, z=0.1)
        # [修改] 设置网格线为深灰色 (50, 50, 50)，确保在白色背景下清晰可见
        g.setColor((50, 50, 50, 255))
        self.addItem(g)

        self.mesh_dir = os.path.join(current_dir, 'meshes')
        
        # 构建机器人 (使用矩阵计算，稳健方案)
        self.build_robot_matrix_style()

    def get_local_matrix(self, xyz, rpy):
        """ 根据 URDF 参数生成局部变换矩阵 """
        m = QMatrix4x4()
        m.translate(xyz[0], xyz[1], xyz[2])
        
        deg_r = np.degrees(rpy[0])
        deg_p = np.degrees(rpy[1])
        deg_y = np.degrees(rpy[2])
        
        m.rotate(deg_y, 0, 0, 1) # Yaw (Z)
        m.rotate(deg_p, 0, 1, 0) # Pitch (Y)
        m.rotate(deg_r, 1, 0, 0) # Roll (X)
        return m

    def add_link(self, filename, color, parent_matrix, xyz, rpy):
        """ 核心函数：加载模型并应用矩阵 """
        # 1. 计算矩阵
        local_m = self.get_local_matrix(xyz, rpy)
        global_m = parent_matrix * local_m 
        
        # 2. 加载模型
        full_path = os.path.join(self.mesh_dir, filename)
        mesh_item = None
        
        if os.path.exists(full_path):
            try:
                your_mesh = mesh.Mesh.from_file(full_path)
                points = your_mesh.vectors.reshape(-1, 3)
                faces = np.arange(points.shape[0]).reshape(-1, 3)
                mesh_data = gl.MeshData(vertexes=points, faces=faces)
                mesh_item = gl.GLMeshItem(meshdata=mesh_data, smooth=True, 
                                          color=color, shader='shaded', glOptions='translucent')
            except: pass
            
        # 备用方块
        if mesh_item is None:
            mesh_item = gl.GLBoxItem(size=gl.QVector3D(0.03, 0.03, 0.03), color=color)
            mesh_item.translate(-0.015, -0.015, -0.015) 

        # 3. 应用变换
        mesh_item.setTransform(global_m)
        self.addItem(mesh_item)
        
        return global_m

    def build_robot_matrix_style(self):
        """ 硬编码 URDF 树状结构 """
        c_silver = (0.8, 0.8, 0.8, 1)
        c_dark   = (0.2, 0.2, 0.2, 1)

        # --- Base ---
        base_tf = QMatrix4x4() # World Origin
        self.add_link('base_link_0.STL', c_silver, base_tf, (0,0,0), (0,0,0))

        # --- Head Chain ---
        h101_tf = self.add_link('head_101.STL', c_dark, base_tf, 
                                (0, 0, 0.5661), (0, 0, 1.5707))
        self.add_link('head_102.STL', c_silver, h101_tf, 
                      (0.018, 0, 0.0764), (-1.5707, 0, -1.5707))

        # --- Right Arm Chain ---
        r51_tf = self.add_link('R51.STL', c_silver, base_tf, 
                               (0, -0.079, 0.517), (1.5708, -1.5708, 0))
        r52_tf = self.add_link('R52.STL', c_silver, r51_tf, 
                               (0, -0.017, 0.042), (1.5708, -1.5708, 0))
        r53_tf = self.add_link('R53.STL', c_silver, r52_tf, 
                               (0, 0.0795, -0.017), (-1.5708, 0, 0))
        r54_tf = self.add_link('R54.STL', c_silver, r53_tf, 
                               (0.017, 0, 0.103), (1.5707, 0, 1.5707))
        r55_tf = self.add_link('R55.STL', c_silver, r54_tf, 
                               (0, 0.0744, -0.017), (0, -1.5708, -1.5708))
        r56_tf = self.add_link('R56.STL', c_silver, r55_tf, 
                               (0, 0.017, 0.077), (1.5708, 0, -3.1416))
        self.add_link('R57.STL', c_silver, r56_tf, 
                      (0.0125, 0.037, 0.001), (0, 0, 0))

        # --- Left Arm Chain ---
        l1_tf = self.add_link('L1.STL', c_silver, base_tf, 
                              (0, 0.079, 0.517), (1.5708, -1.5708, 0))
        l2_tf = self.add_link('L2.STL', c_silver, l1_tf, 
                              (0, -0.017, -0.042), (1.5708, -1.5708, 0))
        l3_tf = self.add_link('L3.STL', c_silver, l2_tf, 
                              (0, 0.0795, -0.017), (-1.5708, 0, 0))
        l4_tf = self.add_link('L4.STL', c_silver, l3_tf, 
                              (-0.017, 0, 0.103), (1.5707, 0, -1.5707))
        l5_tf = self.add_link('L5.STL', c_silver, l4_tf, 
                              (0, 0.0744, -0.017), (0, 1.5707, 1.5707))
        l6_tf = self.add_link('L6.STL', c_silver, l5_tf, 
                              (0, 0.017, 0.077), (1.5708, 0, -3.1415))
        self.add_link('L7.STL', c_silver, l6_tf, 
                      (-0.0125, 0.037, 0.001), (0, 0, 0))

# ==========================================
# 控制面板 (手动录制区域)
# ==========================================
class FullControlPanel(QWidget):
    def __init__(self):
        super().__init__()
        backend.log_signal.connect(self.log)
        self.setup_ui()

    def setup_ui(self):
        main_layout = QHBoxLayout(self)
        
        # === 左栏 ===
        left_group = QGroupBox("Left Arm (ID 1-7)")
        left_group.setStyleSheet("QGroupBox { color: #00E5FF; border: 1px solid #444; }")
        l_layout = QVBoxLayout(left_group)
        self.sliders_left = {}
        for sid in LEFT_ARM_IDS:
            row = QHBoxLayout()
            row.addWidget(QLabel(f"ID {sid}"))
            sl = QSlider(Qt.Orientation.Horizontal)
            sl.setRange(0, 4095); sl.setValue(2048)
            sl.valueChanged.connect(lambda val, s=sid: backend.write_pos(s, val))
            self.sliders_left[sid] = sl
            row.addWidget(sl)
            l_layout.addLayout(row)
        main_layout.addWidget(left_group)

        # === 中栏 ===
        center_panel = QWidget()
        c_layout = QVBoxLayout(center_panel)
        
        # 1. 串口
        hbox_port = QHBoxLayout()
        self.combo_port = QComboBox()
        self.refresh_ports()
        btn_conn = QPushButton("连接")
        btn_conn.clicked.connect(lambda: backend.connect_serial(self.combo_port.currentText()))
        btn_conn.setStyleSheet("background: #333; color: white;")
        hbox_port.addWidget(self.combo_port)
        hbox_port.addWidget(btn_conn)
        c_layout.addLayout(hbox_port)

        # 2. 金刚键
        btn_teleop = QPushButton("🚀 开启遥操作")
        btn_teleop.clicked.connect(self.start_teleop)
        btn_teleop.setStyleSheet("background: #2962FF; color: white; padding: 10px;")
        c_layout.addWidget(btn_teleop)
        
        btn_reset = QPushButton("🔄 全机复位")
        btn_reset.clicked.connect(self.reset_mid)
        btn_reset.setStyleSheet("background: #FF6D00; color: white; padding: 10px;")
        c_layout.addWidget(btn_reset)
        
        btn_stop = QPushButton("🛑 急停 (全机卸力)")
        btn_stop.clicked.connect(self.stop_all)
        btn_stop.setStyleSheet("background: #D50000; color: white; padding: 15px;")
        c_layout.addWidget(btn_stop)

        # 3. 头部滑块
        head_group = QGroupBox("Head (ID 101-102)")
        head_group.setStyleSheet("color: #E040FB; border: 1px solid #444;")
        h_layout = QVBoxLayout(head_group)
        self.sliders_head = {}
        for sid in HEAD_IDS:
            row = QHBoxLayout()
            row.addWidget(QLabel(f"ID {sid}"))
            sl = QSlider(Qt.Orientation.Horizontal)
            sl.setRange(0, 4095); sl.setValue(2048)
            sl.valueChanged.connect(lambda val, s=sid: backend.write_pos(s, val))
            self.sliders_head[sid] = sl
            row.addWidget(sl)
            h_layout.addLayout(row)
        c_layout.addWidget(head_group)

        # 4. Action Manual Control
        action_group = QGroupBox("Action Manual Control")
        action_group.setStyleSheet("color: #00E676; border: 1px solid #444; font-weight: bold;")
        ac_layout = QGridLayout(action_group)
        
        # 录制部分
        self.input_action_name = QLineEdit()
        self.input_action_name.setPlaceholderText("动作名(Action Name)")
        self.input_action_name.setStyleSheet("background: #333; color: white; border: 1px solid #555; padding: 5px;")
        
        btn_rec_start = QPushButton("开始录制")
        btn_rec_start.clicked.connect(self.manual_start_record)
        btn_rec_start.setStyleSheet("background: #D84315; color: white; padding: 5px;")
        
        btn_rec_stop = QPushButton("停止保存")
        btn_rec_stop.clicked.connect(self.manual_stop_record)
        btn_rec_stop.setStyleSheet("background: #2E7D32; color: white; padding: 5px;")
        
        ac_layout.addWidget(QLabel("录制:", styleSheet="color:#AAA"), 0, 0)
        ac_layout.addWidget(self.input_action_name, 0, 1)
        ac_layout.addWidget(btn_rec_start, 0, 2)
        ac_layout.addWidget(btn_rec_stop, 0, 3)
        
        # 回放部分
        self.combo_actions = QComboBox()
        self.combo_actions.setStyleSheet("background: #333; color: white; border: 1px solid #555; padding: 5px;")
        self.refresh_action_list() # 初始化列表
        
        btn_refresh = QPushButton("🔄")
        btn_refresh.setFixedWidth(30)
        btn_refresh.setStyleSheet("background: #444; color: white;")
        btn_refresh.clicked.connect(self.refresh_action_list)
        
        btn_play = QPushButton("回放")
        btn_play.clicked.connect(self.manual_playback)
        btn_play.setStyleSheet("background: #1565C0; color: white; padding: 5px;")
        
        ac_layout.addWidget(QLabel("回放:", styleSheet="color:#AAA"), 1, 0)
        ac_layout.addWidget(self.combo_actions, 1, 1)
        ac_layout.addWidget(btn_refresh, 1, 2)
        ac_layout.addWidget(btn_play, 1, 3)
        
        c_layout.addWidget(action_group)

        # 5. 日志
        self.log_area = QTextEdit()
        self.log_area.setReadOnly(True)
        self.log_area.setFixedHeight(100)
        self.log_area.setStyleSheet("background: #111; color: #AAA;")
        c_layout.addWidget(self.log_area)
        
        main_layout.addWidget(center_panel)

        # === 右栏 ===
        right_group = QGroupBox("Right Arm (ID 51-57)")
        right_group.setStyleSheet("QGroupBox { color: #FFD600; border: 1px solid #444; }")
        r_layout = QVBoxLayout(right_group)
        self.sliders_right = {}
        for sid in RIGHT_ARM_IDS:
            row = QHBoxLayout()
            row.addWidget(QLabel(f"ID {sid}"))
            sl = QSlider(Qt.Orientation.Horizontal)
            sl.setRange(0, 4095); sl.setValue(2048)
            sl.valueChanged.connect(lambda val, s=sid: backend.write_pos(s, val))
            self.sliders_right[sid] = sl
            row.addWidget(sl)
            r_layout.addLayout(row)
        main_layout.addWidget(right_group)

    def log(self, text):
        self.log_area.append(text)

    def refresh_ports(self):
        self.combo_port.clear()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.combo_port.addItems(ports)

    # === 手动控制逻辑 ===
    def refresh_action_list(self):
        self.combo_actions.clear()
        action_dir = os.path.join(current_dir, 'actions')
        if os.path.exists(action_dir):
            files = [f.replace('.json', '') for f in os.listdir(action_dir) if f.endswith('.json')]
            self.combo_actions.addItems(sorted(files))

    def manual_start_record(self):
        name = self.input_action_name.text().strip()
        if not name:
            self.log("⚠️ 请先输入动作名称！")
            return
        
        self.log(f"🔴 开始录制动作: [{name}]")
        self.log(">>> 全机卸力，请示教...")
        
        backend.set_torque(ALL_IDS, False)
        backend.left_traj = []
        backend.right_traj = []
        backend.head_traj = []
        backend.recording = True
        self.current_rec_name = name

    def manual_stop_record(self):
        if not backend.recording: return
        backend.recording = False
        self.log("⏹ 停止录制，保存中...")
        
        saved_data = []
        count = len(backend.left_traj)
        for i in range(count):
            frame = {
                "rel_time": i * 0.04, 
                "left": backend.left_traj[i],
                "right": backend.right_traj[i],
                "head": backend.head_traj[i] if i < len(backend.head_traj) else []
            }
            saved_data.append(frame)
            
        action_dir = os.path.join(current_dir, 'actions')
        os.makedirs(action_dir, exist_ok=True)
        filepath = os.path.join(action_dir, f"{self.current_rec_name}.json")
        
        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(saved_data, f, ensure_ascii=False)
            self.log(f"💾 动作保存成功: {self.current_rec_name}.json")
            self.refresh_action_list() # 刷新下拉列表
        except Exception as e:
            self.log(f"❌ 保存失败: {e}")

    def manual_playback(self):
        name = self.combo_actions.currentText()
        if not name: return
        
        filepath = os.path.join(current_dir, 'actions', f"{name}.json")
        if not os.path.exists(filepath):
            self.log("❌ 文件不存在")
            return
            
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                data = json.load(f)
            threading.Thread(target=backend.play_data, args=(data,)).start()
        except Exception as e:
            self.log(f"❌ 读取失败: {e}")

    def start_teleop(self):
        backend.set_torque(LEFT_ARM_IDS, False)
        backend.set_torque(RIGHT_ARM_IDS, True)
        backend.teleop_running = True
        self.log(">>> 遥操作已开启")

    def reset_mid(self):
        backend.teleop_running = False
        backend.recording = False
        backend.set_torque(ALL_IDS, True)
        for sid in ALL_IDS:
            target = 2048
            if sid in REVERSE_SLAVES: target = MAX_POS - 2048
            backend.write_pos(sid, target)
        
        for sl in self.sliders_left.values():
            sl.blockSignals(True); sl.setValue(2048); sl.blockSignals(False)
        for sl in self.sliders_right.values():
            sl.blockSignals(True); sl.setValue(2048); sl.blockSignals(False)
        for sl in self.sliders_head.values():
            sl.blockSignals(True); sl.setValue(2048); sl.blockSignals(False)

        self.log(">>> 全机已复位 (UI已同步)")

    def stop_all(self):
        backend.teleop_running = False
        backend.recording = False
        backend.set_torque(ALL_IDS, False)
        self.log("!!! 急停执行完毕")

# ==========================================
# [修改版] 设置页面 (无 Python 环境兼容)
# ==========================================
class SettingsPage(QWidget):
    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window 
        self.ext_process = QProcess()
        # [关键] 绑定标准输出(正常信息) 和 标准错误(报错信息)
        self.ext_process.readyReadStandardOutput.connect(self.read_output)
        self.ext_process.readyReadStandardError.connect(self.read_error)
        self.ext_process.finished.connect(self.process_finished)
        self.setup_ui()

    def setup_ui(self):
        # 移除 QTabWidget，直接使用垂直布局
        layout = QVBoxLayout(self)
        layout.setContentsMargins(30, 30, 30, 30)
        layout.setSpacing(25)
        
        # 1. 顶部标题 (为了保持视觉统一，加上这个大标题)
        layout.addWidget(QLabel("🚀 拓展视觉功能", styleSheet="font-size: 28px; font-weight: bold; color: #FFD600;"))
        
        # 2. 说明文字框
        info = QLabel("⚠️ 说明：启动功能时，OS会自动断开主系统串口，并将当前端口号传给子程序。\n功能结束后，请手动重新连接串口。")
        info.setStyleSheet("""
            background: #332200; 
            color: #AAA; 
            padding: 15px; 
            border: 1px solid #FFD600; 
            border-radius: 8px; 
            font-size: 14px;
        """)
        layout.addWidget(info)

        # 3. 两个大功能按钮 (并排)
        grid = QGridLayout()
        grid.setSpacing(40) 
        
        btn_pose = QPushButton("🤖 骨架跟随 (Mimic Pose)")
        btn_pose.setFixedSize(300, 180) 
        btn_pose.setStyleSheet("""
            QPushButton { background: #00C853; color: white; font-size: 20px; border-radius: 15px; font-weight: bold; }
            QPushButton:hover { background: #00E676; border: 2px solid white; }
        """)
        # 注意：这里我们传入 script_name 作为标记
        btn_pose.clicked.connect(lambda: self.launch_script("human_mimic_demo", "mimic_human_pose_V2.py"))
        grid.addWidget(btn_pose, 0, 0)
        
        btn_face = QPushButton("🙂 人脸跟踪 (Face Track)")
        btn_face.setFixedSize(300, 180)
        btn_face.setStyleSheet("""
            QPushButton { background: #2962FF; color: white; font-size: 20px; border-radius: 15px; font-weight: bold; }
            QPushButton:hover { background: #448AFF; border: 2px solid white; }
        """)
        # 注意：这里我们传入 script_name 作为标记
        btn_face.clicked.connect(lambda: self.launch_script("RetinaFace", "face_yolo_track.py"))
        grid.addWidget(btn_face, 0, 1)

        layout.addLayout(grid)
        
        # 4. 底部黑色控制台
        layout.addWidget(QLabel("🖥️ 运行日志:", styleSheet="color: #0f0; font-weight: bold; margin-top: 10px;"))
        self.console = QTextEdit()
        self.console.setReadOnly(True)
        self.console.setStyleSheet("background: #000; color: #0f0; font-family: Consolas; font-size: 13px; border: 1px solid #333; border-radius: 5px;")
        layout.addWidget(self.console)
        
    def launch_script(self, folder_name, script_name):
        if self.ext_process.state() == QProcess.ProcessState.Running:
            self.console.append("⚠️ 已有一个任务在运行，请先关闭！")
            return

        current_port = self.main_window.page_control.combo_port.currentText()
        if not current_port:
            self.console.append("❌ 错误：请先在【上位机】页面选择正确的串口！")
            return

        self.console.append(">>> 正在释放摄像头资源...")
        self.main_window.page_camera.stop()
        self.console.append(f">>> 正在释放串口资源 ({current_port})...")
        backend.disconnect_serial()
        
        time.sleep(0.5)

        # ==================================================
        # [核心修改] 兼容单文件 EXE 的启动方式
        # ==================================================
        worker_type = "unknown"
        if "mimic" in script_name: 
            worker_type = "pose"
        elif "face" in script_name: 
            worker_type = "face"

        self.console.append(f"🚀 正在启动子进程: {worker_type}")
        
        # 默认：生产环境 (打包后)，调用自身 (sys.executable) 并传入参数
        program = sys.executable
        args = ["--worker", worker_type, current_port]

        # 如果是开发环境 (未打包)，则退回使用 python 启动脚本
        if not getattr(sys, 'frozen', False):
            program = "python"
            base_path = os.path.dirname(os.path.abspath(__file__))
            work_dir = os.path.join(base_path, folder_name)
            self.ext_process.setWorkingDirectory(work_dir)
            args = ["-u", script_name, current_port]

        self.ext_process.start(program, args)

    def read_output(self):
        data = self.ext_process.readAllStandardOutput().data().decode('utf-8', errors='ignore')
        self.console.append(data.strip())

    def read_error(self):
        data = self.ext_process.readAllStandardError().data().decode('utf-8', errors='ignore')
        self.console.append(f"<font color='red'>{data.strip()}</font>")

    def process_finished(self):
        self.console.append("⏹ 外部程序已退出。")

# ==========================================
# 5. 主系统框架 (OS Shell)
# ==========================================
class RobotDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("BEISAIKE OS")
        self.resize(1280, 800) 
        self.setStyleSheet("background-color: #121212; color: white;")
        
        self.actions_dir = os.path.join(current_dir, 'actions')
        os.makedirs(self.actions_dir, exist_ok=True)
        self.recording_name = "" 

        self.process_voice = QProcess()
        self.process_voice.readyReadStandardOutput.connect(self.handle_voice_output)
        self.process_voice.readyReadStandardError.connect(self.handle_voice_error)

        self.setup_ui()

    def setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(10, 10, 10, 0)
        main_layout.setSpacing(0)

        top_bar = QHBoxLayout()
        logo = QLabel("BEISAIKE")
        logo.setFont(QFont("Arial", 16, QFont.Weight.Bold))
        logo.setStyleSheet("color: #00E5FF; letter-spacing: 2px;")
        self.time_label = QLabel("12:00 PM")
        top_bar.addWidget(logo); top_bar.addStretch(); top_bar.addWidget(QLabel("📶 5G  🔋 85%  ")); top_bar.addWidget(self.time_label)
        main_layout.addLayout(top_bar)

        self.center_stack = QStackedWidget()
        self.page_home = Robot3DViewer(); self.center_stack.addWidget(self.page_home) # 0

        self.page_voice = QWidget(); v_layout = QVBoxLayout(self.page_voice)
        voice_port_layout = QHBoxLayout()
        voice_port_layout.addWidget(QLabel("语音模块端口:", styleSheet="color: #AAA; font-weight: bold;"))
        self.combo_voice_port = QComboBox(); self.combo_voice_port.setStyleSheet("background: #333; color: white; padding: 5px;")
        self.combo_voice_port.setMinimumWidth(150)
        self.combo_voice_port.addItems([p.device for p in serial.tools.list_ports.comports()])
        voice_port_layout.addWidget(self.combo_voice_port)
        btn_restart_voice = QPushButton("启动/重启语音"); btn_restart_voice.setStyleSheet("background: #2962FF; color: white; padding: 5px 15px;")
        btn_restart_voice.clicked.connect(self.restart_voice_process)
        voice_port_layout.addWidget(btn_restart_voice); voice_port_layout.addStretch() 
        v_layout.addLayout(voice_port_layout); v_layout.addWidget(QLabel("🎤 语音控制台", styleSheet="color:#0f0; font-weight:bold;"))
        self.voice_console = QTextEdit(); self.voice_console.setReadOnly(True); self.voice_console.setStyleSheet("background:#000; color:#0f0; font-family: Consolas;")
        v_layout.addWidget(self.voice_console)
        self.center_stack.addWidget(self.page_voice) # 1

        self.page_control = FullControlPanel(); self.center_stack.addWidget(self.page_control) # 2
        self.page_camera = MultiCameraSystem(); self.center_stack.addWidget(self.page_camera) # 3
        
        # [修改] 直接载入新的 SettingsPage (无Tab版)
        self.page_settings = SettingsPage(self); self.center_stack.addWidget(self.page_settings) # 4

        main_layout.addWidget(self.center_stack)
        self.setup_dock(main_layout)
        self.timer = QTimer(); self.timer.timeout.connect(lambda: self.time_label.setText(QTime.currentTime().toString("hh:mm AP"))); self.timer.start(1000)

    def setup_dock(self, parent_layout):
        dock = QFrame(); dock.setFixedHeight(90); dock.setStyleSheet("background-color: rgba(30, 30, 30, 0.95); border-top: 1px solid #444;")
        layout = QHBoxLayout(dock)
        buttons = [
            ("主界面", lambda: self.switch_tab(0)),
            ("上位机", lambda: self.switch_tab(2)),
            ("语音助手", lambda: self.switch_tab(1) or self.start_voice()), 
            ("摄像头", lambda: self.switch_tab(3)), 
            ("拓展功能", lambda: self.switch_tab(4)), # [修改] 名字改为 拓展功能
        ]
        for name, func in buttons:
            btn = QPushButton(name)
            btn.setFixedHeight(50)
            btn.setStyleSheet("""
                QPushButton { background: transparent; color: #AAA; border: none; border-radius: 8px; font-size: 14px; }
                QPushButton:hover { background: rgba(255,255,255,0.1); color: white; }
                QPushButton:pressed { color: #00E5FF; }
            """)
            btn.clicked.connect(func)
            layout.addWidget(btn)
        parent_layout.addWidget(dock)

    def switch_tab(self, index):
        self.center_stack.setCurrentIndex(index)
        if index == 3: self.page_camera.start()
        else: self.page_camera.stop()

    def start_voice(self):
        if self.process_voice.state() == QProcess.ProcessState.NotRunning:
            port = self.combo_voice_port.currentText()
            self.voice_console.append(f">>> 正在启动语音内核 (Target: {port})...")
            
            # [核心修改] 分身启动模式
            program = sys.executable
            args = ["--worker", "voice", port]
            
            # 开发模式下回退到 python script
            if not getattr(sys, 'frozen', False):
                program = "python"
                args = ["-u", "pure_voice_robot.py", port]

            self.process_voice.start(program, args)
    
    def restart_voice_process(self):
        self.voice_console.append(">>> 正在重启语音服务...")
        if self.process_voice.state() != QProcess.ProcessState.NotRunning: self.process_voice.kill(); self.process_voice.waitForFinished(1000)
        self.start_voice()

    def handle_voice_output(self):
        data = self.process_voice.readAllStandardOutput().data().decode('utf-8', errors='ignore')
        if not data: return
        for line in data.split('\n'):
            line = line.strip(); 
            if not line: continue
            self.voice_console.append(line)
            if "VOICE_CMD::" in line:
                raw_cmd = line.split("VOICE_CMD::")[1].strip()
                cmd = raw_cmd.split(">>")[1].strip() if ">>" in raw_cmd else raw_cmd
                cmd = cmd.replace("。", "").replace(".", "")
                if cmd: self.process_logic(cmd)

    def handle_voice_error(self):
        err = self.process_voice.readAllStandardError().data().decode('utf-8', errors='ignore')
        if err: self.voice_console.append(f"<font color='red'>{err}</font>")

    def process_logic(self, cmd):
        self.voice_console.append(f"<font color='yellow'>识别指令: {cmd}</font>")
        if "停" in cmd or "别动" in cmd:
            if backend.recording: self.stop_recording_action()
            self.page_control.stop_all()
            self.voice_console.append(">>> 执行：急停 / 停止录制")
        elif "复位" in cmd: self.page_control.reset_mid()
        elif "开始录制" in cmd and "动作" in cmd:
            name = cmd.replace("开始录制", "").replace("动作", "").strip()
            if name: self.start_recording_action(name)
            else: self.voice_console.append("⚠️ 没听清动作名字")
        elif "停止录制" in cmd or "结束录制" in cmd or "录制完成" in cmd: self.stop_recording_action()
        else: self.check_and_play(cmd)

    def start_recording_action(self, name):
        self.voice_console.append(f"🔴 开始录制动作: [{name}]"); self.voice_console.append(">>> 机器人已全机卸力，请手动掰动示教...")
        backend.set_torque(ALL_IDS, False); backend.left_traj = []; backend.right_traj = []; backend.head_traj = []; backend.recording = True; self.recording_name = name

    def stop_recording_action(self):
        if not backend.recording: return
        backend.recording = False; self.voice_console.append("⏹ 停止录制，正在保存数据...")
        saved_data = []
        for i in range(len(backend.left_traj)):
            saved_data.append({"rel_time": i * 0.04, "left": backend.left_traj[i], "right": backend.right_traj[i], "head": backend.head_traj[i] if i < len(backend.head_traj) else []})
        filename = f"{self.recording_name}.json"; filepath = os.path.join(self.actions_dir, filename)
        try:
            with open(filepath, 'w', encoding='utf-8') as f: json.dump(saved_data, f, ensure_ascii=False)
            self.voice_console.append(f"💾 动作已保存: {filename} (共 {len(backend.left_traj)} 帧)"); self.page_control.refresh_action_list()
        except Exception as e: self.voice_console.append(f"❌ 保存失败: {e}")

    def check_and_play(self, text):
        if not os.path.exists(self.actions_dir): return
        files = [f for f in os.listdir(self.actions_dir) if f.endswith(".json")]
        matched = False
        for f in files:
            name_no_ext = f.replace(".json", "")
            if name_no_ext in text:
                self.voice_console.append(f"🎞️ 匹配到动作库: [{name_no_ext}]"); self.play_action_file(f); matched = True; break
        if not matched and "回放" in text: self.voice_console.append("⚠️ 未找到对应动作文件，请先录制。")

    def play_action_file(self, filename):
        filepath = os.path.join(self.actions_dir, filename)
        try:
            with open(filepath, 'r', encoding='utf-8') as f: threading.Thread(target=backend.play_data, args=(json.load(f),)).start()
        except Exception as e: self.voice_console.append(f"❌ 读取动作文件失败: {e}")

    def keyPressEvent(self, event: QKeyEvent):
        if event.key() == Qt.Key.Key_Escape:
            if (event.modifiers() & Qt.KeyboardModifier.ControlModifier) and (event.modifiers() & Qt.KeyboardModifier.AltModifier):
                self.process_voice.kill(); self.page_camera.stop(); self.close()

# ==========================================
# 6. [新增] 启动加载页面 (Splash Screen)
# ==========================================
class SplashScreen(QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedSize(600, 350)
        self.setWindowFlags(Qt.WindowType.FramelessWindowHint | Qt.WindowType.WindowStaysOnTopHint)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)

        layout = QVBoxLayout(self)
        self.setStyleSheet("""
            QWidget { background-color: #121212; border: 1px solid #333; border-radius: 12px; }
            QLabel { color: white; border: none; }
            QProgressBar { border: none; background-color: #222; height: 6px; border-radius: 3px; text-align: center; }
            QProgressBar::chunk { background-color: #00E5FF; border-radius: 3px; }
        """)

        # Logo 区域
        logo_label = QLabel("BEISAIKE OS")
        logo_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        logo_label.setStyleSheet("color: #00E5FF; font-size: 48px; font-weight: bold; letter-spacing: 3px; border: none;")
        layout.addStretch()
        layout.addWidget(logo_label)
        layout.addStretch()

        # 状态文字
        self.status_label = QLabel("Initializing System...")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.status_label.setStyleSheet("color: #888; font-size: 14px; margin-bottom: 10px; border: none;")
        layout.addWidget(self.status_label)

        # 进度条
        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setTextVisible(False)
        layout.addWidget(self.progress)

        layout.setContentsMargins(50, 60, 50, 50)

    def update_progress(self, val, text):
        self.progress.setValue(val)
        self.status_label.setText(text)
        QApplication.processEvents()

# ==========================================
# [核心修改] 主程序入口 + 多进程分发中心
# ==========================================
if __name__ == '__main__':
    # 1. 必须调用 freeze_support 以支持打包后的多进程
    from multiprocessing import freeze_support
    freeze_support()

    # 2. 检查这是否是一个“分身”进程 (Worker)
    if len(sys.argv) > 1 and sys.argv[1] == '--worker':
        worker_type = sys.argv[2]
        port_arg = sys.argv[3]
        
        if worker_type == 'voice':
            # 启动语音模块
            pure_voice_robot.run_voice(port_arg)
            
        elif worker_type == 'pose':
            # 启动骨架跟随
            # [警告] 你必须修改 mimic_human_pose_V2.py 像 pure_voice_robot 那样封装成 run_pose(port)
            # from human_mimic_demo import mimic_human_pose_V2
            # mimic_human_pose_V2.run_pose(port_arg)
            pass
            
        elif worker_type == 'face':
            # 启动人脸跟踪
            # [警告] 你必须修改 face_yolo_track.py 封装成 run_face(port)
            # from RetinaFace import face_yolo_track
            # face_yolo_track.run_face(port_arg)
            pass
            
        # 任务结束直接退出，不显示 GUI
        sys.exit(0)

    # ==========================================
    # 3. 如果没有参数，说明是用户双击打开的主界面
    # ==========================================
    app = QApplication(sys.argv)

    # --- 显示加载窗口 ---
    splash = SplashScreen()
    splash.show()

    # 模拟加载过程
    loading_steps = [
        (10, "Initializing Kernel..."),
        (25, "Loading 3D Environment (OpenGL)..."),
        (40, "Checking Hardware Interfaces..."),
        (60, "Loading Neural Networks Config..."),
        (80, "Starting User Interface..."),
        (95, "Welcome to BEISAIKE OS."),
        (100, "Done.")
    ]

    for val, text in loading_steps:
        splash.update_progress(val, text)
        time.sleep(0.15) 

    # --- 启动主程序 ---
    window = RobotDashboard()
    window.show()
    
    # 关闭加载页
    splash.close()
    
    sys.exit(app.exec())