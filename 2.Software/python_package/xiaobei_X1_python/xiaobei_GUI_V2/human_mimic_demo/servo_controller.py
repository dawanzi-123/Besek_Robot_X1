#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Feetech 舵机控制封装（支持 STS + SCS）
- 左臂:  ID 1~7   （7 默认按 SCS 处理）
- 右臂:  ID 51~57 （57 默认按 SCS 处理）
- 位置范围: 0~4095 （≈ 0~360°，1步≈0.088°）
- 提供：上/卸力、写位置（带速度/加速度）、安全夹紧等工具函数
"""

import os, sys, time

# 优先在当前目录找 scservo_sdk，其次在上级目录找
_here = os.path.dirname(os.path.abspath(__file__))
_sdk_local = os.path.join(_here, "scservo_sdk")
_sdk_parent = os.path.join(os.path.dirname(_here), "scservo_sdk")
for _p in (_sdk_local, _sdk_parent):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.append(_p)

try:
    from scservo_sdk import PortHandler, sms_sts, scscl, COMM_SUCCESS
except Exception as e:
    raise RuntimeError("❌ 未找到 scservo_sdk，请把 scservo_sdk 目录放到 human_mimic_demo/ 或其上级目录") from e


BAUD = 1000000
POS_MIN, POS_MAX, POS_MID = 0, 4095, 2048

# 7/57 默认是 SCS（若你的项目不是，写入时会自动降级到 STS 写法）
SCS_GRIPPER_IDS = {7, 57}

# 某些关节可能需要反向（按你的机构装配改这里）
REVERSE_IDS = set()  # 例如：{2, 54, 56}


class ServoController:
    def __init__(self, port="/dev/ttyACM0", baud=BAUD, speed=1200, acc=80):
        self.port_name = port
        self.port = PortHandler(port)
        self.sts = sms_sts(self.port)  # STS/SMS 协议
        self.scs = scscl(self.port)    # SCS 协议
        self.default_speed = int(speed)
        self.default_acc = int(acc)

        if not self.port.openPort():
            raise RuntimeError(f"❌ 无法打开串口 {port}")
        if not self.port.setBaudRate(baud):
            raise RuntimeError(f"❌ 无法设置波特率 {baud}")
        print(f"✅ 串口 {port} 初始化成功，波特率 {baud}")

        # 默认：给所有关节上力（夹爪保持上力，避免下垂），可按需修改
        self.torque_enable(range(1, 8), True)
        self.torque_enable(range(51, 58), True)

    # ========== 低层 ==========
    def _write_pos_sts(self, sid: int, pos: int, speed: int = None, acc: int = None):
        spd = self.default_speed if speed is None else int(speed)
        ac  = self.default_acc   if acc   is None else int(acc)
        pos = max(POS_MIN, min(POS_MAX, int(pos)))
        cr, err = self.sts.WritePosEx(sid, pos, spd, ac)
        return cr == COMM_SUCCESS and err == 0

    def _write_pos_scs(self, sid: int, pos: int, time_ms: int = 0, spd: int = 0):
        # SCS 没有加速度，time/spd 二选一，这里用 WritePos(位置, 速度, 加速度/时间)
        pos = max(POS_MIN, min(POS_MAX, int(pos)))
        try:
            self.scs.WritePos(sid, pos, spd, time_ms)
            return True
        except Exception:
            return False

    def _write_torque(self, sid: int, enable: bool):
        val = 1 if enable else 0
        # 0x28 扭矩开关地址：STS/SCS 均通用
        try:
            self.sts.write1ByteTxRx(sid, 0x28, val)
            return True
        except Exception:
            try:
                self.scs.write1ByteTxRx(sid, 0x28, val)
                return True
            except Exception:
                return False

    # ========== 公共接口 ==========
    def torque_enable(self, ids, enable: bool):
        for sid in ids:
            ok = self._write_torque(int(sid), enable)
            if not ok:
                print(f"[WARN] 设置扭矩失败 ID={sid}")

    def write_pos(self, sid: int, pos: int, speed: int = None, acc: int = None):
        """自动区分 SCS/STS；7/57 优先按 SCS 写，失败再走 STS。"""
        sid = int(sid)
        if sid in REVERSE_IDS:
            pos = POS_MAX - pos

        if sid in SCS_GRIPPER_IDS:
            if not self._write_pos_scs(sid, pos, time_ms=0, spd=0):
                # 兜底：部分项目 7/57 也用 STS 协议
                self._write_pos_sts(sid, pos, speed, acc)
            return

        # 其他默认 STS
        if not self._write_pos_sts(sid, pos, speed, acc):
            # 兜底：万一实际是 SCS
            self._write_pos_scs(sid, pos, time_ms=0, spd=0)

    def safe_close(self):
        try:
            # 默认收回到中位并卸力
            for sid in list(range(1, 8)) + list(range(51, 58)):
                self.write_pos(sid, POS_MID)
                time.sleep(0.005)
            self.torque_enable(range(1, 8), False)
            self.torque_enable(range(51, 58), False)
            time.sleep(0.05)
        finally:
            try:
                self.port.closePort()
            except Exception:
                pass
            print("✅ 串口关闭，舵机已安全退出")


# ======= 角度↔步数工具 =======
STEP_PER_DEG = 4096.0 / 360.0  # ≈11.3778 steps/deg

def deg_to_pos(center_deg: float, target_deg: float, center_pos: int = POS_MID,
               limit_min: int = POS_MIN, limit_max: int = POS_MAX) -> int:
    """
    将相对中心角度(°)映射为舵机步数。
    center_deg: 定义 center_pos 对应的角度（通常 0°）
    target_deg: 目标绝对角度（相对 center_deg 偏转）
    """
    delta = (target_deg - center_deg) * STEP_PER_DEG
    pos = int(round(center_pos + delta))
    return max(limit_min, min(limit_max, pos))
