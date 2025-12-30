
# Besek Robot X1: Desktop Dual-Arm Humanoid Robot

**我的桌面级双臂人形机器人项目。**

这是一个基于 ROS 2 Humble 和 Python AI 生态构建的双臂人形机器人平台。

视频演示：[【硬核】贝塞克机器人 X1演示 - 抖音](https://www.douyin.com/user/MS4wLjABAAAALJjPB01CHBNe-ILyPQk7zOEjp2iVMOuy6y-uWWRwRGMZzpzafSwUnlrpVfqK1Zw?from_tab_name=main&modal_id=7572126951415601266)

硬件获取：[淘宝硬件](https://item.taobao.com/item.htm?id=963573481713&mi_id=00000SG52yAfml-7lI-NROiuXbjteY14U2qNXjdzCW5R_Ck&spm=a21xtw.29178619.0.0)

----------

## 📂 资料说明 (Update 2025-12-30)

本项目已开源所有代码部分，旨在降低双臂机器人的开发门槛：

-   **1.Hardware**: [淘宝硬件](https://item.taobao.com/item.htm?id=963573481713&mi_id=00000SG52yAfml-7lI-NROiuXbjteY14U2qNXjdzCW5R_Ck&spm=a21xtw.29178619.0.0)
    
-   **2.Software**:
    
    -   已添加完整的 ROS 2 工作空间 (`xiaobei_X1_ws`)。
        
    -   已添加 Python AI 功能包 (YOLOv8 姿态模仿、人脸追踪、语音控制)。
        
    -   已添加 `pkg_robot_model` 仿真与数字孪生配置。
        
-   **3.Docs**: 已添加保姆级环境配置文档、双臂控制原理及仿真教程。
    

关于“青春版”设计理念：

为了让大家能低成本复现，X1 没有采用昂贵的谐波减速器，而是选择了 Feetech STS 系列总线舵机。

-   **成本极低**：整机成本控制在消费级范围内。
    
-   **制造简单**：全机结构件均可使用 FDM 3D 打印制造。
    
-   **生态兼容**：完全适配 Ubuntu 22.04 + ROS 2 Humble，支持 MoveIt 运动规划。
    

----------

## ⚙️ 关于结构设计

整机采用拟人化设计，拥有 **14+2 个自由度**（双臂各 7 DOF + 头部 2 DOF）。

不同于 其它人形机器人的步进电机+谐波减速方案，Besek X1 选择了**高集成度的串行总线舵机方案**：

-   **关节模组**：使用 Feetech STS3215 磁编码舵机，单级减速，自带位置/速度/负载反馈。
    
-   **传动方式**：关节直接驱动，消除了复杂的减速箱设计，大幅降低了装配难度。
    
-   **末端执行器**：设计了通用的二指夹爪，支持更换不同的指尖以适应不同物体的抓取。
    

详细的 ID 分配与安装规范，请参考 `3.Docs/01_Environment.docx`。

----------

## 🔌 关于电路模块

为了实现极致的简洁，X1 摒弃了复杂的自研驱动板，采用了**PC 直连**的电气架构。

系统的核心电气拓扑非常简单：

1.  **主控大脑**：一台运行 Ubuntu 22.04 的 PC（推荐配置 NVIDIA 显卡以加速 AI）。
    
2.  **通信链路**：通过 USB 转 TTL 模块直接连接舵机总线。
    
3.  **电源管理**：使用 12V DC (≥5A) 集中供电。
    

布线改进：

参考了工业机器人的走线方式，我们将左右臂和头部的总线进行了分路设计，所有线缆通过内部走线槽，避免了运动中的缠绕问题。

----------

## 💻 关于核心软件架构 (ROS 2)

这是本项目的灵魂所在。我们基于 **ROS 2 Humble** 构建了完整的控制系统。

**核心功能模块：**

1.  pkg_robot_model (数字孪生)：
    
    负责 URDF 模型的加载与状态发布。通过 robot_state_publisher 和 joint_state_publisher，我们可以在 RViz 中实时看到与真实机器人一模一样的姿态 2。
    
2.  xiaobei_arms (主从控制)：
    
    利用 ROS 2 的 DDS 通信机制，实现了双机遥操作。
    
    -   `master_arm_publisher`: 读取示教臂角度并广播。
        
    -   `slave_arm_subscriber`: 从机订阅并实时跟随，支持毫秒级同步 3。
        
3.  MoveIt 运动规划：
    
    配置了完整的 KDL 运动学求解器，支持在笛卡尔空间进行拖拽规划，并自动避障。
    

----------

## 🧠 关于 AI 与控制算法

除了传统的 ROS 控制，我们还在 Python 层实现了多种 AI 交互功能（位于 `python_package` 目录）。

### 1. 姿态模仿 (Human Mimicry)

基于 **YOLOv8-Pose** 算法。

-   **原理**：通过摄像头实时提取人体骨骼关键点（手腕、手肘、肩膀）。
    
-   **映射**：将 2D 图像平面的向量映射到机器人的 3D 关节空间，计算出目标角度。
    
-   **滤波**：为了防止抖动，我们加入了平滑滤波算法，让模仿动作更加自然。
    

### 2. 人脸追踪 (Face Tracking)

基于 **RetinaFace** + PID 控制。

-   视觉节点实时解算人脸中心坐标 (x, y)。
    
-   头部云台根据偏差值进行 PID 计算，驱动颈部舵机自动锁定目标。
    

### 3. 语音控制 (Voice Control)

实现了一个轻量级的语音交互系统。

-   唤醒词：“你好小贝”。
    
-   指令集：支持“握手”、“跳舞”、“打招呼”等预设动作库的调用。
    

----------

## 🕹️ 指令模式

Besek X1 支持多种控制模式，适用于不同的场景：

**模式名称**

**核心技术**

**适用场景**

**说明**

**Master-Slave (主从)**

ROS 2 Topic

遥操作、示教记录

需要两台设备或示教臂，延迟极低 4

**MoveIt Planning**

Inverse Kinematics

自动化抓取

在 RViz 中拖动滑块，规划无碰撞路径 5

**AI Follow**

Computer Vision

交互演示

启动 Python 脚本，机器人自动跟随人脸或模仿动作

**Voice CMD**

Speech Recognition

娱乐互动

语音触发预存的动作序列 (Action Group)

----------

## 🚀 快速开始

**1. 硬件准备**

-   准备一台 Ubuntu 22.04 电脑。
    
-   连接 Feetech 舵机与 USB 串口模块。
    
-   确保获得串口权限：`sudo usermod -a -G dialout $USER`。
    

**2. 软件编译**

Bash

```
cd ~/xiaobei_X1_ws
colcon build
source install/setup.bash
```

**3. 启动数字孪生**

Bash

```
ros2 launch pkg_robot_model display.launch.py
```

更详细的教程请查阅 `3.Docs` 文件夹。

----------

## 🤝 关于作者

dawanzi-123

硬核机器人爱好者。

如果你觉得这个项目对你有帮助，请给个 Star ⭐️！
