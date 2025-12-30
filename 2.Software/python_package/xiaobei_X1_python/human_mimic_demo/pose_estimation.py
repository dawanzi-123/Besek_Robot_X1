#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO Pose 人体关键点检测（兼容 Python 3.13）
返回：dict
{
  'people': [  # 只取第一人也行
     {
        'kpts': [(x,y,conf), ... 17个COCO关键点],
        'conf': score
     }, ...
  ]
}
关键点索引(COCO):
0鼻,1左眼,2右眼,3左耳,4右耳,5左肩,6右肩,7左肘,8右肘,9左腕,10右腕,
11左髋,12右髋,13左膝,14右膝,15左踝,16右踝
"""
import numpy as np
from ultralytics import YOLO

# 首次会自动下载  yolov8n-pose.pt  模型
_model = YOLO("yolov8n-pose.pt")

def detect_people(frame_bgr):
    results = _model(frame_bgr, verbose=False)
    out = {'people': []}
    if not results:
        return out
    r = results[0]
    if r.keypoints is None or len(r.keypoints) == 0:
        return out

    kpts_xy = r.keypoints.xy  # [num, 17, 2]
    kpts_cf = r.keypoints.conf  # [num,17]
    boxes = r.boxes
    n = len(kpts_xy)
    for i in range(n):
        pts = kpts_xy[i].cpu().numpy()   # (17,2)
        cfs = kpts_cf[i].cpu().numpy()   # (17,)
        klist = [(float(pts[j,0]), float(pts[j,1]), float(cfs[j])) for j in range(pts.shape[0])]
        score = float(boxes.conf[i].cpu().numpy()) if boxes is not None and len(boxes) == n else 0.0
        out['people'].append({'kpts': klist, 'conf': score})
    return out
