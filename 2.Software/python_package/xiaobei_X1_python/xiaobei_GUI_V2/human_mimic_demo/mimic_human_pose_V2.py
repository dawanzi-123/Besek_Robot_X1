#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os

# 1. 优先使用当前文件夹下的 scservo_sdk
sys.path.append(os.getcwd())

import cv2, time, numpy as np, math
from pose_estimation import detect_people
# 确保这里能引用到 SDK (该文件夹下应该有 servo_controller.py)
try:
    from servo_controller import ServoController, POS_MID
except ImportError as e:
    print(f"❌ 导入错误: {e}")
    print("请确保 scservo_sdk 和 servo_controller.py 在同一目录下")
    exit()

# === [关键修改] 动态获取串口 ===
if len(sys.argv) > 1:
    PORT = sys.argv[1]
    print(f"🔗 接收到端口参数: {PORT}")
else:
    PORT = "COM3" # Windows 默认
    print(f"⚠️ 未接收到参数，使用默认端口: {PORT}")

BAUD = 1000000
SPEED, ACC = 1200, 80
FPS = 20.0

SAFE_MIN_1, SAFE_MAX_1 = 1800, 4095
SAFE_MIN_2, SAFE_MAX_2 = 1750, 2250
SAFE_MIN_4, SAFE_MAX_4 = 1900, 3200
SAFE_MIN_51, SAFE_MAX_51 = 0, 2048
SAFE_MIN_52, SAFE_MAX_52 = 1750, 2250
SAFE_MIN_54, SAFE_MAX_54 = 1900, 3200

REST_1 = 2048
REST_51 = 2000

CENTER_X_RATIO = 0.25
CENTER_Y_RATIO = 0.30

# ========== 来自 id54_follow.py 的右肘参数 ==========
MID = 2048
ANGLE_REST = 160.0
ANGLE_MIN = 30.0

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def calc_angle(a, b, c):
    ax, ay = a; bx, by = b; cx, cy = c
    v1 = (ax-bx, ay-by); v2 = (cx-bx, cy-by)
    dot = v1[0]*v2[0] + v1[1]*v2[1]
    mag1 = math.hypot(v1[0],v1[1]); mag2 = math.hypot(v2[0],v2[1])
    if mag1 < 1e-6 or mag2 < 1e-6: return ANGLE_REST
    cosv = clamp(dot/(mag1*mag2+1e-9), -1, 1)
    return math.degrees(math.acos(cosv))

SKELETON = [
    (5,6),(5,7),(7,9),(6,8),(8,10),
    (5,11),(6,12),(11,12),
    (11,13),(13,15),(12,14),(14,16)
]

def extract_one(det):
    if not det['people']: return None
    return max(det['people'], key=lambda p: p['conf'])['kpts']

def is_person_centered(k,w,h):
    xs=[p[0] for p in k if p[2]>0.3]; ys=[p[1] for p in k if p[2]>0.3]
    if not xs or not ys: return False
    px,py=np.mean(xs),np.mean(ys); cx,cy=w/2,h/2
    return (cx-w*CENTER_X_RATIO < px < cx+w*CENTER_X_RATIO and
            cy-h*CENTER_Y_RATIO < py < cy+h*CENTER_Y_RATIO)

def compute_pitch(k,h,side):
    SHO_y=k[5 if side=='L' else 6][1]
    ELB_y=k[7 if side=='L' else 8][1]
    dy=SHO_y-ELB_y
    return np.clip((dy/(h/2))*180*2,-90,180)

def compute_roll(k,side):
    SHO_x=k[5 if side=='L' else 6][0]
    ELB_x=k[7 if side=='L' else 8][0]
    dx=(SHO_x-ELB_x)*12.0
    dx=np.clip(dx,-250,250)
    return np.clip(np.interp(dx/250,[-1,1],[90,-60]),-60,90)

def compute_elbow(k,side):
    SHO=np.array(k[5 if side=='L' else 6][:2])
    ELB=np.array(k[7 if side=='L' else 8][:2])
    WRI=np.array(k[9 if side=='L' else 10][:2])
    u,f=SHO-ELB,WRI-ELB
    cosang=np.dot(u,f)/(np.linalg.norm(u)*np.linalg.norm(f)+1e-6)
    return np.clip(np.degrees(np.arccos(np.clip(cosang,-1,1))),0,160)

def deg_to_pos_pitch(deg,side):
    if side=='L': lo,hi=-90,90; smin,smax=SAFE_MIN_1,SAFE_MAX_1
    else: lo,hi=90,-90; smin,smax=SAFE_MIN_51,SAFE_MAX_51
    pos = smin+((deg-lo)/(hi-lo))*(smax-smin)
    return int(np.clip(pos,smin,smax))

def deg_to_pos_roll(deg,side):
    lo,hi=-60,90; t=(deg-lo)/(hi-lo)
    if side=='L': return int(SAFE_MAX_2 - t*(SAFE_MAX_2-SAFE_MIN_2))
    return int(SAFE_MAX_52 - t*(SAFE_MAX_52-SAFE_MIN_52))

def draw_full_skeleton(d,k,center=False):
    col=(0,255,255) if center else (100,100,100); blk=(0,0,0)
    if len(k)>6:
        ls=np.array(k[5][:2]); rs=np.array(k[6][:2])
        cc=((ls[0]+rs[0])/2,(ls[1]+rs[1])/2)
        hc=(int(cc[0]),int(cc[1]-80))
        cv2.circle(d,hc,32,col,6)

        # ✅ 脖子竖线 (仅新增这 3 行)
        neck_top = (int(cc[0]), int(cc[1]-40))
        neck_bottom = (int(cc[0]), int(cc[1]))
        cv2.line(d, neck_top, neck_bottom, col, 6)

    for a,b in SKELETON:
        if a<len(k) and b<len(k):
            cv2.line(d,tuple(map(int,k[a][:2])),tuple(map(int,k[b][:2])),col,6)
    for i in range(5,len(k)):
        if k[i][2]>0.3: cv2.circle(d,(int(k[i][0]),int(k[i][1])),6,blk,-1)

# ========== 主程序 ==========
def main():
    cap=cv2.VideoCapture(0)
    if not cap.isOpened(): print("摄像头错误"); return

    ctrl=ServoController(PORT,BAUD,speed=SPEED,acc=ACC)
    for sid in [1,2,4,51,52,54]: ctrl.write_pos(sid,2048)

    last=0
    while True:
        ok,frame=cap.read(); disp=cv2.flip(frame,1)
        if not ok: break
        h,w=disp.shape[:2]
        if time.time()-last<1/FPS:
            cv2.imshow("DualArm",disp)
            if cv2.waitKey(1)==27: break
            continue
        last=time.time()

        det=detect_people(disp)
        kpts=extract_one(det)
        if not kpts:
            ctrl.write_pos(51,REST_51)
            for sid in [1,2,4,52,54]: ctrl.write_pos(sid,2048)
            cv2.putText(disp,"No Person",(20,50),1,2,(0,0,255),2)
            cv2.imshow("DualArm",disp)
            continue

        centered=is_person_centered(kpts,w,h)

        if centered:
            # 左臂
            ctrl.write_pos(1,deg_to_pos_pitch(compute_pitch(kpts,h,'L'),'L'))
            ctrl.write_pos(2,deg_to_pos_roll(compute_roll(kpts,'L'),'L'))
            ctrl.write_pos(4,int(SAFE_MAX_4 - (compute_elbow(kpts,'L')/160)*(SAFE_MAX_4-SAFE_MIN_4)))

            # 右肩
            ctrl.write_pos(51,deg_to_pos_pitch(compute_pitch(kpts,h,'R'),'R'))
            ctrl.write_pos(52,deg_to_pos_roll(compute_roll(kpts,'R'),'R'))

            # ✅ 右肘你校准的逻辑
            RS,RE,RW = kpts[6],kpts[8],kpts[10]
            elbow = calc_angle((RS[0],RS[1]),(RE[0],RE[1]),(RW[0],RW[1]))
            elbow = clamp(elbow,ANGLE_MIN,ANGLE_REST)
            ratio = (elbow-ANGLE_MIN)/(ANGLE_REST-ANGLE_MIN)
            pos54 = int(ratio*MID)
            pos54 = clamp(pos54,0,4095)
            ctrl.write_pos(54,pos54,speed=SPEED,acc=ACC)

            draw_full_skeleton(disp,kpts,True)
            cv2.putText(disp,"CENTER LOCKED",(20,50),1,2,(0,255,0),2)

        else:
            ctrl.write_pos(51,REST_51)
            for sid in [1,2,4,52,54]: ctrl.write_pos(sid,2048)
            draw_full_skeleton(disp,kpts,False)
            cv2.putText(disp,"OUT OF CENTER",(20,50),1,2,(128,128,128),2)

        cv2.imshow("DualArm",disp)
        if cv2.waitKey(1)==27: break

    cap.release(); cv2.destroyAllWindows(); ctrl.safe_close()

if __name__=="__main__":
    main()
