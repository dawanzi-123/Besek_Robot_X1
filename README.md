# 🤖 Besek X1: Desktop Dual-Arm Humanoid Robot for Embodied AI
# 贝赛克 X1：基于 ROS 2 & Python 的桌面级具身智能双臂机器人

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10-yellow.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)]()
[![Hardware](https://img.shields.io/badge/Hardware-Low%20Cost-orange.svg)]()

> **[Open Source] A Low-Cost Platform for Embodied AI Research.**
> Features: **Teleoperation**, **Visual Servoing**, **Imitation Learning**, and **Sim2Real**.
>
> **[开源]** 专为**具身智能 (Embodied AI)** 教育与研究打造的低成本开发平台。支持**遥操作**、**视觉抓取**、**动作克隆**。

<div align="center">
  <img src="https://github.com/user-attachments/assets/c2b167b9-d7fa-4ccb-af13-801b3cb17824" width="80%" alt="Besek X1 Dual Arm Robot ROS2 Python">
  <br>
  <em>Besek Robot X1 桌面级双臂机器人真机展示</em>
</div>

---

## 📺 视频演示 (Demo)

👀 **[【硬核】贝塞克机器人 X1 演示 - 抖音](https://www.douyin.com/user/MS4wLjABAAAALJjPB01CHBNe-ILyPQk7zOEjp2iVMOuy6y-uWWRwRGMZzpzafSwUnlrpVfqK1Zw?from_tab_name=main&modal_id=7572126951415601266)**

---

## 📂 项目概览 (Overview)

**Update 2025-12-30**: 本项目已开源所有代码部分，旨在降低双臂机器人的开发门槛。

### 1. 🛒 硬件获取 (Hardware)
为了让大家能低成本复现，X1 没有采用昂贵的谐波减速器，而是选择了 **Feetech STS 系列总线舵机**。
* **获取链接**: [淘宝硬件套件 (Taobao)](https://item.taobao.com/item.htm?id=963573481713&mi_id=00000SG52yAfml-7lI-NROiuXbjteY14U2qNXjdzCW5R_Ck&spm=a21xtw.29178619.0.0)
* **设计理念 (青春版)**:
    * **成本极低**：整机成本控制在消费级范围内。
    * **制造简单**：全机结构件均可使用 **FDM 3D 打印**制造。
    * **极简电子**：摒弃复杂驱动板，采用 PC 直连架构。

### 2. 💻 软件架构 (Software)
基于 **Ubuntu 22.04** + **ROS 2 Humble** 生态：
* ✅ **ROS 2 Workspace**: 包含核心功能包 `xiaobei_X1_ws`。
* ✅ **Python AI Package**: 集成 **YOLOv8** 姿态模仿、人脸追踪、语音控制。
* ✅ **Simulation**: `pkg_robot_model` 支持 Gazebo 仿真与数字孪生 (Digital Twin)。

<div align="center">
  <img src="https://github.com/user-attachments/assets/c83f3c66-3442-4306-a570-145378c2baa8" width="80%" alt="Besek X1 ROS2 Simulation and Control Software">
</div>

### 3. 📚 文档 (Documentation)
* 已添加保姆级环境配置文档 (`3.Docs/01_Environment.docx`)。
* 包含双臂控制原理及仿真教程。

---

## ⚙️ 机械与电子设计 (Design)

### 结构设计 (Mechanical)
整机采用拟人化设计，拥有 **14+2 个自由度**（双臂各 7 DOF + 头部 2 DOF）。
* **高集成度方案**：使用 **Feetech STS3215** 磁编码舵机，单级减速，自带位置/速度/负载反馈。
* **直驱技术**：关节直接驱动，消除减速箱间隙，大幅降低装配难度。
* **末端执行器**：通用二指夹爪设计，支持更换指尖以适应不同物体。

### 电路模块 (Electronics)
为了实现极致的简洁，X1 采用了 **PC 直连 (PC-Direct)** 的电气架构：
1.  **主控大脑**：运行 Ubuntu 22.04 的 PC（推荐 NVIDIA 显卡加速 AI）。
2.  **通信链路**：USB 转 TTL 模块直接连接舵机总线。
3.  **电源管理**：12V DC (≥5A) 集中供电。
4.  **工业级走线**：左右臂与头部总线分路设计，内部走线槽避免缠绕。

---

## 💻 核心功能 (Core Features)

这是本项目的灵魂所在。我们基于 **ROS 2 Humble** 构建了完整的控制系统。

### 1. 数字孪生 (Digital Twin)
* **功能包**: `pkg_robot_model`
* **描述**: 负责 URDF 加载与状态发布。通过 `robot_state_publisher`，在 RViz 中实时同步真实机器人的姿态。

### 2. 主从遥操作 (Teleoperation)
* **功能包**: `xiaobei_arms`
* **描述**: 利用 ROS 2 DDS 通信机制，实现毫秒级双机同步。
    * `master_arm_publisher`: 读取示教臂角度并广播。
    * `slave_arm_subscriber`: 从机订阅并跟随。

### 3. MoveIt 运动规划
* 配置完整的 **KDL 运动学求解器**。
* 支持笛卡尔空间拖拽规划，并自动避障。

---

## 🧠 AI 与智能交互 (AI & Algorithms)

位于 `python_package` 目录，完全使用 Python 编写，降低 AI 开发门槛。

### 1. 姿态模仿 (Human Mimicry) [YOLOv8]
* **原理**: 通过摄像头实时提取人体骨骼关键点 (Keypoints)。
* **算法**: 将 2D 图像向量映射到 3D 关节空间，配合平滑滤波算法，实现自然动作复现。

### 2. 人脸追踪 (Face Tracking) [RetinaFace]
* 视觉节点解算人脸中心坐标，通过 **PID 算法** 驱动头部云台自动锁定目标。

### 3. 语音控制 (Voice Control)
* 唤醒词：“你好小贝”。
* 支持指令：“握手”、“跳舞”、“打招呼”等预设动作库 (Action Group)。

---

## 🕹️ 控制模式说明 (Control Modes)

| 模式名称 | 核心技术 | 适用场景 | 说明 |
| :--- | :--- | :--- | :--- |
| **Master-Slave (主从)** | ROS 2 Topic | 遥操作、数据采集 | 需两台设备，适合**模仿学习数据收集**，延迟极低 |
| **MoveIt Planning** | Inverse Kinematics | 自动化抓取 | 在 RViz 中拖动滑块，规划无碰撞路径 |
| **AI Follow** | Computer Vision | 交互演示 | 启动 Python 脚本，机器人自动跟随人脸或模仿动作 |
| **Voice CMD** | Speech Recognition | 娱乐互动 | 语音触发预存的动作序列 |

---

## 🚀 快速开始 (Quick Start)

### 1. 硬件准备
* 准备一台 Ubuntu 22.04 电脑。
* 连接 Feetech 舵机与 USB 串口模块。
* **权限设置**: `sudo usermod -a -G dialout $USER`

### 2. 编译代码
```bash
cd ~/xiaobei_X1_ws
colcon build
source install/setup.bash
