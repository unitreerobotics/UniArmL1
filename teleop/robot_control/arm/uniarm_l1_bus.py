# unitree.py 核心部分备份

import struct
import time
import serial
import math
import numpy as np
import logging
import threading
from dataclasses import dataclass
import sys
import time
from .constants import (
    BAUDRATE, HEAD_TX, HEAD_RX, RX_LEN, GEAR_RATIO,
    K_POS, K_SPEED, K_TORQUE, K_KP, K_KD, K_VOLTAGE,
    OUT_POS_RES, ERROR_MAP,CRC32_TABLE
)
from .utiles import (
    OutPosRaw2OutPos_rad, rad2deg, deg2rad, OutPos_rad2OutPosRaw,
)

logger = logging.getLogger(__name__)

@dataclass
class MotorState:
    id: int
    mode: int = 0
    timeout: int = 0
    temp_case: int = 0
    temp_winding: int = 0
    voltage: float = 0.0
    torque_nm: float = 0.0
    speed_rads: float = 0.0
    pos_rad: float = 0.0
    pos_raw_rad: int = 0
    MError: int = 0
    ExFlag: int = 0
    OutPos: int = 0
    OutPos_rad: float = 0.0
    OutPos_deg: float = 0.0
    CRC32: int = 0
    freq: float = 0.0
    last_update: float = time.perf_counter()
    pos_rad_threshold: float = 20.0


class UnitreeMotorSDK:
    """Unitree 电机 SDK - 单总线半双工通信，支持位置、速度、力矩控制模式，带 CRC 校验和连续位置跟踪"""

    def __init__(self, motor_ids, port: str):
        self.ser = serial.Serial(
            port,
            BAUDRATE,
            timeout=0.002,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        self.motor_states = {motor_id: MotorState(id=motor_id) for motor_id in motor_ids}
        self.verbose_errors = False
        self._serial_lock = threading.RLock()
        self.last_valid_time = {}
        self.pos_rad_unwrapped = {motor_id: 0.0 for motor_id in motor_ids}  # 追踪 unwrapped 位置
        self.output_zero_offset = {motor_id: None for motor_id in motor_ids}
    @staticmethod
    def _wrap_0_2pi(x):
        two_pi = 2.0 * math.pi
        return x % two_pi

    def _ensure_output_offset(self, motor_id: int):
        st = self.motor_states[motor_id]
        if self.output_zero_offset[motor_id] is None:
            raw_cont = st.pos_rad
            raw_mod = self._wrap_0_2pi(raw_cont)
            self.output_zero_offset[motor_id] = self._wrap_0_2pi(st.OutPos_rad - raw_mod)
    def get_output_pos_cont_aligned(self, motor_id: int) -> float:
        self._ensure_output_offset(motor_id)
        st = self.motor_states[motor_id]
        offset = self.output_zero_offset[motor_id]
        if offset is None:
            offset = 0.0
        return st.pos_rad + offset
    @staticmethod
    def crc32_lookup(data: bytes) -> int:
        crc = 0xFFFFFFFF
        n = len(data)
        i = 0
        while i + 3 < n:
            b0 = data[i + 3]
            b1 = data[i + 2]
            b2 = data[i + 1]
            b3 = data[i]
            crc = CRC32_TABLE[(crc >> 24) ^ b0] ^ ((crc << 8) & 0xFFFFFFFF)
            crc = CRC32_TABLE[(crc >> 24) ^ b1] ^ ((crc << 8) & 0xFFFFFFFF)
            crc = CRC32_TABLE[(crc >> 24) ^ b2] ^ ((crc << 8) & 0xFFFFFFFF)
            crc = CRC32_TABLE[(crc >> 24) ^ b3] ^ ((crc << 8) & 0xFFFFFFFF)
            i += 4
        return crc

    def write_control_packet(self, motor_id: int, pos: float = 0.0, kp: float = 1.8e-4,
            kd: float = 4.1e-5, torque: float = 0.0, speed: float = 0.5e-8,
            status: int = 1, timeout_enable: int = 0):
        """
        这里把传入的 pos 统一解释为：输出端单圈目标角(rad)
        内部自动转换成：最近连续目标角
        """

        mode_byte = (motor_id & 0x0F) | ((status & 0x07) << 4) | ((timeout_enable & 0x01) << 7)
        st = self.motor_states[motor_id]

        two_pi = 2.0 * math.pi

        # 当前连续输出端位置（由返回的 pos 换算）
        cur_pos_cont = st.pos_rad

        # 当前输出端单圈角 [0, 2π)
        cur_pos_mod = st.OutPos_rad

        # 目标也强制解释成单圈角
        pos_mod = pos % two_pi

        # 单圈最短路径误差
        delta = pos_mod - cur_pos_mod
        delta = (delta + math.pi) % two_pi - math.pi

        # 最近连续目标
        pos_cont = cur_pos_cont + delta

        raw_tor = max(-32768, min(32767, int((torque / GEAR_RATIO) * K_TORQUE)))
        raw_spd = max(-32768, min(32767, int((speed * GEAR_RATIO) * K_SPEED)))
        raw_pos = int((pos_cont * GEAR_RATIO) * K_POS)
        raw_kp = max(0, min(32767, int(kp * K_KP)))
        raw_kd = max(0, min(32767, int(kd * K_KD)))

        self._ensure_output_offset(motor_id)
        aligned_cont = self.get_output_pos_cont_aligned(motor_id)

        # if motor_id == 3:
        #     print(
        #         f"[mid={motor_id}] "
        #         f"cmd_in={pos:.4f}, pos_mod={pos_mod:.4f}, "
        #         f"cur_mod={cur_pos_mod:.4f}, raw_cont={cur_pos_cont:.4f}, "
        #         f"aligned_cont={aligned_cont:.4f}, "
        #         f"delta={delta:.4f}, pos_cont={pos_cont:.4f}, raw_pos={raw_pos}"
        #     )
        payload = struct.pack("<BBhhihh", mode_byte, 0, raw_tor, raw_spd, raw_pos, raw_kp, raw_kd)
        packet_without_crc = HEAD_TX + payload
        crc = self.crc32_lookup(packet_without_crc)
        full_packet = packet_without_crc + struct.pack('<I', crc)
        assert len(full_packet) == 20
        self.ser.write(full_packet)
        

    def read(self):
        """接收反馈包，并在串口流错位时自动按帧头重同步。"""
        data = self._read_fast_frame()
        if data is None:
            data = self._read_feedback_frame()
        if data is None:
            return None
        mode_byte = data[2]
        motor_id = mode_byte & 0x0F
        mode = (mode_byte >> 4) & 0x07
        timeout = (mode_byte >> 7) & 0x01
        # 提取 19 字节的 fbk 数据
        fbk = data[3:22]  # 22 - 3 = 19
        if len(fbk) != 19:
            return None

        try:
            temp, sensor, vol, torque, speed, pos, MError, outpos_exflag, exsensor2, excom = \
                struct.unpack('<bbb hh i I H BB', fbk)
        except struct.error as e:
            print(f"[解析错误] {e}, fbk length={len(fbk)}")
            return None

        # 分离位域
        OutPos = outpos_exflag & 0x1FFF      # 低13位
        ExFlag = (outpos_exflag >> 13) & 0x07  # 高3位

        # CRC 校验：从 mode_byte 开始共 20 字节（data[2:22]）
        crc_received = struct.unpack('<I', data[22:26])[0]
        crc_computed = self.crc32_lookup(data[2:22])
        if crc_received != crc_computed:
            return None

        # 转换为物理量
        vol_f = vol / 2.0
        ExPos = 2 * math.pi * OutPos / 8192.0
        outputSpd = (speed / 2.56) * 2 * math.pi / GEAR_RATIO
        outputTor = (torque / 256000.0) * GEAR_RATIO
        outputPos = 2 * math.pi * pos / 32768.0 / GEAR_RATIO
        

        now = time.perf_counter()
        st = self.motor_states[motor_id]
        st.mode = mode
        st.timeout = 0
        st.temp_case = temp   # 电机外壳温度
        st.temp_winding = sensor  # 电机绕组温度
        st.voltage = vol_f   # 电压
        st.torque_nm = outputTor
        st.speed_rads = outputSpd
        st.pos_rad = outputPos
        st.MError = MError
        st.ExFlag = ExFlag
        st.OutPos = OutPos
        st.OutPos_rad = ExPos 
        st.OutPos_deg = rad2deg(ExPos)
        st.CRC32 = crc_received
        
        # 频率计算
        last_time = self.last_valid_time.get(motor_id, now - 0.01)
        dt = now - last_time
        st.freq = (1.0 / dt) if dt > 0 else 0.0
        self.last_valid_time[motor_id] = now

        return st

    def _is_valid_feedback_frame(self, data: bytes) -> bool:
        if len(data) != RX_LEN:
            return False
        if data[:2] != HEAD_RX:
            return False
        crc_received = struct.unpack('<I', data[22:26])[0]
        crc_computed = self.crc32_lookup(data[2:22])
        return crc_received == crc_computed

    def _read_fast_frame(self) -> bytes | None:
        try:
            data = self.ser.read(RX_LEN)
        except serial.SerialException as exc:
            logger.warning("Serial read failed on %s: %s", self.ser.port, exc)
            return None
        if self._is_valid_feedback_frame(data):
            return data
        return None

    def _read_feedback_frame(self) -> bytes | None:
        deadline = time.perf_counter() + 0.003
        frame = bytearray()

        while time.perf_counter() < deadline:
            try:
                byte = self.ser.read(1)
            except serial.SerialException as exc:
                logger.warning("Serial read failed on %s: %s", self.ser.port, exc)
                return None
            if not byte:
                continue

            if not frame:
                if byte == HEAD_RX[:1]:
                    frame.extend(byte)
                continue

            if len(frame) == 1:
                if byte == HEAD_RX[1:2]:
                    frame.extend(byte)
                    rest = self.ser.read(RX_LEN - 2)
                    if len(rest) != RX_LEN - 2:
                        return None
                    frame.extend(rest)
                    data = bytes(frame)
                    if self._is_valid_feedback_frame(data):
                        return data
                    frame.clear()
                elif byte == HEAD_RX[:1]:
                    frame[:] = byte
                else:
                    frame.clear()

        return None


    def parse_feedback_packet(self,data: bytes):
        if len(data) != 26:
            return None
        if data[0] != 0xFC or data[1] != 0xEE:
            return None

        mode_byte = data[2]
        motor_id = mode_byte & 0x0F
        mode = (mode_byte >> 4) & 0x07
        timeout = (mode_byte >> 7) & 0x01

        # 提取 19 字节的 fbk 数据
        fbk = data[3:22]  # 22 - 3 = 19
        if len(fbk) != 19:
            return None

        try:
            temp, sensor, vol, torque, speed, pos, MError, outpos_exflag, exsensor2, excom = \
                struct.unpack('<bbb hh i I H BB', fbk)
        except struct.error as e:
            print(f"[解析错误] {e}, fbk length={len(fbk)}")
            return None

        # 分离位域
        OutPos = outpos_exflag & 0x1FFF      # 低13位
        ExFlag = (outpos_exflag >> 13) & 0x07  # 高3位

        # CRC 校验：从 mode_byte 开始共 20 字节（data[2:22]）
        crc_received = struct.unpack('<I', data[22:26])[0]
        crc_computed = self.crc32_lookup(data[2:22])
        if crc_received != crc_computed:
            return None

        # 转换为物理量
        vol_f = vol / 2.0
        ExPos = 2 * math.pi * OutPos / 8192.0
        outputSpd = (speed / 2.56) * 2 * math.pi / GEAR_RATIO
        outputTor = (torque / 256000.0) * GEAR_RATIO
        outputPos = 2 * math.pi * pos / 32768.0 / GEAR_RATIO

        return {
            'motor_id': motor_id,
            'mode': mode,
            'timeout': timeout,
            'Temp': temp,
            'sensor': sensor,
            'vol': vol_f,
            'MError': MError,
            'MWarn': ExFlag,
            'ExPos': ExPos,
            'outputTor': outputTor,
            'outputSpd': outputSpd,
            'outputPos': outputPos
        }

    def send_and_receive(self, motor_id, mode, timeout_flag,
                         outputTor, outputSpd, outputPos, Kp, Kd):
        # 1. 发送控制包
        self.write_control_packet(motor_id, mode, timeout_flag,
                                   outputTor, outputSpd, outputPos, Kp, Kd)
        

        # 2. 立即读取 26 字节反馈（阻塞，带超时）
        feedback = self.ser.read(26)
        if len(feedback) != 26:
            print(f"[错误] 未收到完整反馈包（收到 {len(feedback)} 字节）")
            return None

        # 3. 解析反馈
        parsed = self.parse_feedback_packet(feedback)
        if parsed is None:
            print("[错误] 反馈包校验失败或格式错误")
            return None

        return parsed
    def close(self):
        if self.ser.is_open:
            self.ser.close()


class UnitreeMotorsBus:
    """Unitree 半双工单总线"""

    def __init__(self, port: str, motor_ids: list[int],
                 kp_default: list | None = None,
                 kd_default: list | None = None,
                 torque_default: list | None = None,
                 speed_default: list | None = None,
                 kp_loop: list | None = None,
                 kd_loop: list | None = None,
                 handshake_timeout: float = 1.0):
        self.port = port
        self.motor_ids = motor_ids
        n_motors = len(motor_ids)

        # 每个电机独立参数（8个电机，按电机ID索引）
        # 默认值：M0-M7
        if kp_default is None:
            kp_default = [5.4755e-5, 5.6755e-5, 5.0755e-5, 6.0755e-5, 3.4755e-5, 5.6755e-5, 5.6755e-5, 8.0755e-5]
        if kd_default is None:
            kd_default = [9.6889e-6, 1.889e-5, 9.0889e-6, 6.6889e-6, 9.6889e-6, 0.9889e-5, 0.6889e-5, 8.0889e-6]
        if torque_default is None:
            torque_default = [0.0] * n_motors
        if speed_default is None:
            speed_default = [0.0] * n_motors
        if kp_loop is None:
            kp_loop = [1.0, 1.2, 0.9, 1.0, 1.0, 1.8, 0.9, 0.8]
        if kd_loop is None:
            kd_loop = [0.01, 0.04, 0.01, 0.01, 0.01, 0.01, 0.03, 0.01]

        self.kp_default = kp_default
        self.kd_default = kd_default
        self.torque_default = torque_default
        self.speed_default = speed_default
        self.kp_loop = kp_loop
        self.kd_loop = kd_loop

        self.handshake_timeout = handshake_timeout

        self.motors = UnitreeMotorSDK(motor_ids=motor_ids, port=port)
        self._is_connected = False

        self.con_mod = np.zeros(n_motors)
        self.tgt_pos_rad = np.zeros(n_motors)
        self.tgt_torque = np.zeros(n_motors)
        self.cur_pos_rad = np.zeros(n_motors)

        self.control_mode = 'zero_damping'  # 默认控制模式
        self.kp_torque_pos = 3.21
        self.kd_torque_vel = 0.17
        self.max_torque = 2.0
        self.range_min_rad = None
        self.range_max_rad = None

        # 连续 None 计数器（用于检测电机未连接）
        self._consecutive_none_count = 0
        self._max_consecutive_none = 200
        self._connection_error = None  # 存储连接错误信息

    def close_control(self):
        """发送控制指令"""
        debug_counter = 0

        tgt_pos_rad = np.zeros_like(self.tgt_pos_rad) if self.control_mode == 'zero_damping' else self.tgt_pos_rad
        kp= [0.0]*len(self.kp_default) if self.control_mode == 'zero_damping' else self.kp_default
        kd = [0.0]*len(self.kd_default) if self.control_mode == 'zero_damping' else self.kd_default
        torque = [0.0]*len(self.torque_default) if self.control_mode == 'zero_damping' else self.tgt_torque
        speed = [0.0]*len(self.speed_default) if self.control_mode == 'zero_damping' else self.speed_default
        if self.control_mode == 'zero_damping':
            status = np.ones(len(self.motor_ids))
        else:
            status = self.con_mod.copy()

        # # 极限位置安全裕度（弧度），在到达极限前提前停止
        # SAFETY_MARGIN = 0.05  # 约 3 度

        debug_counter += 1
        for i, mid in enumerate(self.motor_ids):
            cur_pos = self.motors.motor_states[mid].OutPos_rad
            freq = self.motors.motor_states[mid].freq


            target = tgt_pos_rad[i]

            delta = target - cur_pos
            if delta > math.pi:
                delta -= 2 * math.pi
            elif delta < -math.pi:
                delta += 2 * math.pi

            adjusted_target = cur_pos + self.kp_loop[i]*delta + self.kd_loop[i]*(0.0 - self.motors.motor_states[mid].speed_rads)

            
            self.motors.write_control_packet(
                    motor_id=mid,
                    pos=adjusted_target,
                    kp=kp[mid],
                    kd=kd[mid],
                    torque=torque[mid],
                    speed=speed[mid],
                    status=int(status[i]),
                    timeout_enable=0,
                )
            
            result = self.motors.read()

            # 检测连续 None（电机未连接）
            if result is None:
                self._consecutive_none_count += 1
                if self._consecutive_none_count == self._max_consecutive_none:
                    print(f"   电机反馈连续丢失，请检查电机电源和串口连接 ({self.port}, last motor id={mid})")
            else:
                self._consecutive_none_count = 0  # 重置计数器


    def set_all_zero(self):
        """一问一答模式：发一条 -> 收一条"""
        
        out = {}
        
        # 对每个电机：发送 -> 立刻读取
        for mid in self.motor_ids:
            self.motors.write_control_packet(
                motor_id=mid,
                status=1,
                kp=0.0,
                kd=0.0,
                pos=0.0,
                timeout_enable=0
            )
            
            # 立刻读取这条电机的回包
            fb = self.motors.read()
            
        # 收集所有电机的最新状态
        for mid in self.motor_ids:
            st = self.motors.motor_states.get(mid)
            if st is not None:
                out[mid] = {
                    "pos_raw_rad": st.pos_rad,
                    "out_pos_deg": st.OutPos_deg,
                    "OutPos_raw": st.OutPos,
                }
        
        return out

 



    def set_zero_damping(self):
        """设置所有电机为0阻尼模式（采集数据专用）"""
        results = []
        
        for mid in self.motor_ids:
            self.motors.write_control_packet(
                motor_id=mid,
                status=1,
                kp=0.0,
                kd=0.0,
                pos=0.0,
                timeout_enable=0,
            )
            try:
                fb = self.motors.read()
            except serial.SerialException as exc:
                logger.warning("Serial read failed on %s while setting zero damping: %s", self.port, exc)
                break
            if fb is None:
                results.append({
                    "motor_id": mid,
                    "OutPos": None,
                    "pos_rad": None,
                    "pos_deg": None,
                    "freq": 0.0,
                })
                continue
            
            st = self.motors.motor_states.get(mid)
            if st:
                results.append({
                    "motor_id": mid,
                    "OutPos": st.OutPos,
                    "pos_rad": st.OutPos_rad,
                    "pos_deg": st.OutPos_deg,
                    "freq": st.freq,
                })
            else:
                results.append({
                    "motor_id": mid,
                    "OutPos": None,
                    "pos_rad": None,
                    "pos_deg": None,
                    "freq": 0.0,
                })
            
        
        return results

    def poll_zero_damping(self, motor_ids: list[int]):
        for mid in motor_ids:
            self.motors.write_control_packet(
                motor_id=mid,
                status=1,
                kp=0.0,
                kd=0.0,
                pos=0.0,
                timeout_enable=0,
            )
            self.motors.read()
