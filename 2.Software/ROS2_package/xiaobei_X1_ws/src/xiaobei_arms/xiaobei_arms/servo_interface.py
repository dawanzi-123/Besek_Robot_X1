#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Feetech STS 串口舵机总线封装（纯 STS/ SMS 飞特协议）
- 统一对上层暴露：open/close、read_pos(id)、write_pos(id,pos,speed,acc)
- 只使用 sms_sts（飞特协议），适配所有 ID（包括 7/57）
"""

import time
from typing import Iterable, Optional

try:
    from .scservo_sdk import PortHandler, sms_sts, COMM_SUCCESS  # 包内 SDK
except Exception:
    from scservo_sdk import PortHandler, sms_sts, COMM_SUCCESS   # 兼容被单独运行

MAX_POS = 4095
MID_POS = 2048

# 常用寄存器地址（STS/SMS 飞特协议）
ADDR_LOCK_FLAG = 0x37     # 55 锁标志：0=解锁，1=上锁
ADDR_RUN_MODE  = 0x21     # 33 运行模式：0 伺服位置模式
ADDR_TORQUE_EN = 0x28     # 40 扭矩开关：0 关；1 开；128 当前位置设为 2048
ADDR_SPEED     = 0x2E     # 46 运行速度
ADDR_ACCEL     = 0x29     # 41 加速度

class ServoBusSTS:
    """ 纯 STS 总线封装 """
    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 1_000_000) -> None:
        self._port_name = port
        self._baudrate = baudrate
        self._port = PortHandler(self._port_name)
        self._sts = sms_sts(self._port)
        self._opened = False

    # -------- 基础 --------
    def open(self) -> None:
        if not self._port.openPort():
            raise RuntimeError(f"无法打开串口: {self._port_name}")
        if not self._port.setBaudRate(self._baudrate):
            raise RuntimeError(f"无法设置波特率: {self._baudrate}")
        self._opened = True

    def close(self) -> None:
        if self._opened:
            try:
                self._port.closePort()
            except Exception:
                pass
            self._opened = False

    @property
    def is_open(self) -> bool:
        return self._opened

    # -------- 初始化 / 扭矩 --------
    def init_ids(self, ids: Iterable[int], torque_on: bool = True) -> None:
        """ 解锁→位置模式→扭矩 """
        for sid in ids:
            try:
                self._sts.write1ByteTxRx(sid, ADDR_LOCK_FLAG, 0)
                self._sts.write1ByteTxRx(sid, ADDR_RUN_MODE, 0)
                self._sts.write1ByteTxRx(sid, ADDR_TORQUE_EN, 1 if torque_on else 0)
                time.sleep(0.002)
            except Exception:
                continue

    def set_torque(self, ids: Iterable[int], enable: bool) -> None:
        for sid in ids:
            try:
                self._sts.write1ByteTxRx(sid, ADDR_TORQUE_EN, 1 if enable else 0)
                time.sleep(0.001)
            except Exception:
                continue

    # -------- 读 / 写位置 --------
    def read_pos(self, sid: int) -> Optional[int]:
        try:
            pos, cr, err = self._sts.ReadPos(sid)
            if cr == COMM_SUCCESS and err == 0:
                return int(pos)
            return None
        except Exception:
            return None

    def write_pos(self, sid: int, pos: int, speed: int = 1500, accel: int = 0) -> bool:
        if pos < 0: pos = 0
        elif pos > MAX_POS: pos = MAX_POS
        try:
            cr, err = self._sts.WritePosEx(sid, int(pos), int(speed), int(accel))
            return (cr == COMM_SUCCESS and err == 0)
        except Exception:
            return False
