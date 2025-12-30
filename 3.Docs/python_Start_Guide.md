
# python 使用说明

### 一、先执行配置环境的命令

### 1.重装正确的 OpenCV 包（最推荐）

通常只需要卸载当前的 headless 版本，并安装完整的 opencv-python 版本即可。

请在终端中依次执行以下命令：

卸载现有的 OpenCV 包（为了防止冲突，建议把相关的都卸载一遍）：

pip uninstall opencv-python opencv-python-headless opencv-contrib-python

安装支持 GUI 的完整版：

pip install opencv-python_(如果你需要 contrib 库中的额外功能，也可以改为安装_ _pip install opencv-contrib-python__，它同样包含 GUI 支持)_

###   
2.缺少 ultralytics 库（这是 YOLOv8 的核心库），你需要安装它。

请在终端执行以下命令进行安装：

pip install ultralytics

  

### 二、功能启动

1.照镜子模仿人的动作：

先进入xiaobei_X1_python中的 human_mimic_demo文件夹，然后执行 python3 mimic_human_pose_V2.py

终端命令

cd xiaobei_X1_python/

cd human_mimic_demo/

python3 mimic_human_pose_V2.py


2.头部人脸跟踪：

先进入xiaobei_X1_python中的RetinaFace文件夹，然后执行python3 face_yolo_track.py

终端命令

cd xiaobei_X1_python/

cd RetinaFace/

python3 face_yolo_track.py

  

3.上位机

先进入xiaobei_X1_python中的xiaobei_GUI文件夹，然后执行python3 robot_control_main.py

终端命令

cd xiaobei_X1_python/xiaobei_GUI/

python3 robot_control_main.py

  

4.语音控制

先进入xiaobei_X1_python中的xiaobei_GUI文件夹，然后执行python3 pure_voice_robot.py

终端命令

cd xiaobei_X1_python/

cd xiaobei_voice/

python3 pure_voice_robot.py

使用方法：先唤醒小贝，说你好小贝，然后看到屏幕上面出现聆听中，再说“跳舞”，“展示自由度”，“握手”，“打招呼”。每个词代表一个动作，要等当前动作做完再说别的动作词。

录制方法：先唤醒小贝，说你好小贝，然后看到屏幕上面出现聆听中，再说开始录制XXX动作，然后录制结束说停止录制，录制好的动作就会以XXX的名字保存好。