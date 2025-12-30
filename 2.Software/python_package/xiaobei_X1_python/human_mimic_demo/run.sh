#!/bin/bash
echo "正在启动机器人照镜子程序..."
# 确保在当前目录
cd "$(dirname "$0")"
# 运行
python3 mimic_human_pose_V2.py
read -p "程序已结束，按回车键退出..."