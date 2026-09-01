import logging
import time
import threading
from pathlib import Path
from dataclasses import dataclass

import pinocchio as pin
import numpy as np
from scipy.spatial.transform import Rotation as R

from image_server.utils import make_cameras_from_configs

from .config_uniarm_l1 import UniArmL1RobotConfig
from .uniarm_l1_bus import UnitreeMotorsBus
from .constants import OUT_POS_RES
from .utiles import OutPosRaw2OutPos_rad
from .kinematics import RobotKinematics

logger = logging.getLogger(__name__)
@dataclass
class MotorCalibration:
    motor_id: int
    joint_name: str
    homing_offset: float
    range_min: float
    range_max: float

class UniArmL1:
    """UniArmL1 robot controller."""

    name = "uniarm_l1_follower"

    def __init__(self, config: UniArmL1RobotConfig):
        self.config = config

        # Motor configuration: supports dual-motor joints
        self.joint_motor_ids = config.joint_motor_ids
        self.motor_names = list(self.joint_motor_ids.keys())
        self.n_joints = len(self.motor_names)

        # Get all motor IDs list
        motor_ids = []
        self.motor_to_joint = {}  # motor_id -> joint_name reverse mapping
        for joint_name, ids in self.joint_motor_ids.items():
            for mid in ids:
                motor_ids.append(mid)
                self.motor_to_joint[mid] = joint_name
        self.motor_ids = sorted(motor_ids)
        self.n_motors = len(self.motor_ids)
    
        

        self.cameras = make_cameras_from_configs(config.cameras)
        self.urdf_path = config.urdf_path
        self.mesh_dir = Path(self.urdf_path).parent if self.urdf_path else None
        self.pin_model = pin.buildModelFromUrdf(self.urdf_path)
        self.pin_data = self.pin_model.createData()

        # IK configuration
        if self.urdf_path is None:
            raise ValueError("URDF path must not be None for IK solver initialization.")

        self.ik_solver = RobotKinematics(
            urdf_path=self.urdf_path,
            target_frame_name="Link_5",
            joint_names=self.motor_names[:5],
        )

        self.ik_solver.set_joint_preferences({
            "shoulder_pan": 0.5,
            "shoulder_lift": 0.3,
            "elbow_flex": 0.3,
            "wrist_flex": 0.3,
            "wrist_roll": 0.5,
        })

        self.calibration = {}
        self._running = False
        self._control_thread = None

        # Joint-level data (n_joints = 6)
        self.homing_rad = np.zeros(self.n_joints)
        self.range_min_rad = np.zeros(self.n_joints)
        self.range_max_rad = np.zeros(self.n_joints)
        self.q_current_sim = np.zeros(self.n_joints)
        self.tgt_pos_rad = np.zeros(self.n_motors)
        self.ratio_sim_phys = np.zeros(self.n_joints)
        self.phys_spans = np.zeros(self.n_joints)
        self.sim_spans = np.zeros(self.n_joints)
        self.home_set = np.zeros(self.n_joints)

        # Motor-level data (n_motors = 8)
        self.cur_pos_rad = np.zeros(self.n_motors)
        self.tgt_tau = np.zeros(self.n_motors)
        self.v_current_sim = np.zeros(self.n_joints)
        self.a_current_sim = np.zeros(self.n_joints)
        self.v_last_sim = np.zeros(self.n_joints)
        self.last_dyn_time = time.time()

        self.con_mode = np.zeros(self.n_motors)
        self.calibration_fpath = Path.home() / f".cache/unitree/calibration/{config.id}.json"

        # Default values
        self.kp_torque = [0.3, 1.6, 0.55, 0.6, 0.2, 0.1, 1.2, 0.45]
        self.kd_torque = [0.001, 0.004, 0.02, 0.038, 0.012, 0.001, 0.0005, 0.016]
        self.max_torque = [1.5, 3.0, 2.0, 1.5, 1.5, 1.5, 3.0, 2.0]

        # Individual homing and range for each motor (8 motors)
        self.motor_homing_rad = np.zeros(self.n_motors)
        self.motor_range_min_rad = np.zeros(self.n_motors)
        self.motor_range_max_rad = np.zeros(self.n_motors)

        # Try loading calibration from file
        self._load_calibration_from_file()

        # Initialize only when calibration has data
        if self.calibration:
            self.init_rate_phys_sim_spans()
            self.init_homing_rad()

        # home_set: home position in sim space
        # joint 0: middle, 1: large, 2: small, 3: small, 4: middle, 5: middle
        self.home_set = np.zeros(self.n_joints)
        for i, joint_name in enumerate(self.motor_names):
            q_min, q_max = self._get_joint_limits(joint_name)
            if q_min is None or q_max is None:
                continue
            if i in [1]:  # large
                self.home_set[i] = q_max
            elif i in [2, 3]:  # small
                self.home_set[i] = q_min
            else:  # middle (0, 4, 5)
                self.home_set[i] = (q_min + q_max) / 2
        self.sim_last = np.zeros(self.n_joints)
        self.t_start = time.time()
        self.init_finish = False
        self.last = 6.0

        # Try connecting hardware, fallback to simulation mode if failed
        if self.config.no_real_robot:
            print("⚠️ no_real_robot=True, using simulation mode")
            self.bus = None
            self.sim_mode = True
        else:
            try:
                self.bus = UnitreeMotorsBus(port=self.config.port, motor_ids=self.motor_ids)
                self.bus.range_max_rad = self.motor_range_max_rad.copy()
                self.bus.range_min_rad = self.motor_range_min_rad.copy()
                self.sim_mode = False
            except Exception as e:
                print(f"⚠️ Failed to connect to hardware ({e}), using simulation mode")
                self.bus = None
                self.sim_mode = True
        self.ee_state = 0

        self.teleop_enabled = True
        self._step_first_active = True
        # Initial pose (sim space)
        self.q_init_sim_rad = np.array([0.0, 0.7, -0.5, -0.9, 0.0, 0.0])
        self.q_init_rad = self.map_sim_to_output_rad_all(self.q_init_sim_rad)
        self.last_sim_q_deg = np.rad2deg(self.q_init_sim_rad)
        self.ee_pose = self.ik_solver.forward_kinematics(self.last_sim_q_deg[:5])
        self.target_pos_base = self.ee_pose[:3, 3]
        self.target_ori = self.ee_pose[:3, :3]


    def init_homing_rad(self):
        """Initialize individual homing and range for each motor."""
        # First initialize individual data for each motor
        for motor_id in self.motor_ids:
            cal = self.calibration.get(motor_id)
            if cal is None:
                print(f"Warning: No calibration for motor {motor_id}")
                continue

            self.motor_homing_rad[motor_id] = OutPosRaw2OutPos_rad(int(cal.homing_offset))
            self.motor_range_min_rad[motor_id] = self.motor_homing_rad[motor_id] + OutPosRaw2OutPos_rad(int(cal.range_min))
            self.motor_range_max_rad[motor_id] = self.motor_homing_rad[motor_id] + OutPosRaw2OutPos_rad(int(cal.range_max))
            print(f"Motor {motor_id} ({cal.joint_name}): homing={self.motor_homing_rad[motor_id]:.4f} rad, "
                  f"range=[{self.motor_range_min_rad[motor_id]:.4f}, {self.motor_range_max_rad[motor_id]:.4f}] rad")

        # Joint-level data: used for single-motor joints and sim->output primary mapping (using first motor)
        for i in range(len(self.motor_names)):
            joint_name = self.motor_names[i]
            motor_ids = self.joint_motor_ids[joint_name]
            motor_id = motor_ids[0]

            self.homing_rad[i] = self.motor_homing_rad[motor_id]
            self.range_min_rad[i] = self.motor_range_min_rad[motor_id]
            self.range_max_rad[i] = self.motor_range_max_rad[motor_id]

        print("Joint-level homing_rad:", self.homing_rad)
        print("Joint-level range_min_rad:", self.range_min_rad)
        print("Joint-level range_max_rad:", self.range_max_rad)

    

    def _load_calibration_from_file(self) -> bool:
        """Load calibration data from file.
        Returns: True if loaded successfully
        """
        import json

        calibration_file = self.calibration_fpath

        if not calibration_file.exists():
            logger.warning(f"Calibration file not found: {calibration_file}")
            return False

        try:
            with open(calibration_file, 'r') as f:
                data = json.load(f)

            # Store by motor ID: {"motor_0": {"motor_id": 0, "joint_name": "...", ...}, ...}
            for key, cal_data in data.items():
                motor_id = cal_data.get("motor_id")
                if motor_id is not None and motor_id in self.motor_ids:
                    self.calibration[motor_id] = MotorCalibration(
                        motor_id=motor_id,
                        joint_name=cal_data.get("joint_name", ""),
                        homing_offset=float(cal_data.get("homing_offset", 0)),
                        range_min=float(cal_data.get("range_min", 0)),
                        range_max=float(cal_data.get("range_max", 0)),
                    )

            if self.calibration:
                logger.info(f"Successfully loaded calibration data for {len(self.calibration)} motors")
                return True
            else:
                logger.warning("Calibration file is empty or no matching motors")
                return False

        except Exception as e:
            logger.error(f"Failed to load calibration file: {e}")
            return False


    def _save_calibration(self) -> None:
        """Save calibration data to JSON file."""
        import json

        # Ensure directory exists
        self.calibration_fpath.parent.mkdir(parents=True, exist_ok=True)

        # Convert calibration to serializable dict
        data = {}
        for motor_id, cal in self.calibration.items():
            data[f"motor_{motor_id}"] = {
                "motor_id": cal.motor_id,
                "joint_name": cal.joint_name,
                "homing_offset": cal.homing_offset,
                "range_min": cal.range_min,
                "range_max": cal.range_max,
            }

        with open(self.calibration_fpath, 'w') as f:
            json.dump(data, f, indent=4)

        logger.info(f"Calibration data saved to: {self.calibration_fpath}")

    def calibrate(self) -> None:
        """Calibration based on Unwrapping principle: each motor calibrated independently.
        - range_min/max store unwrapped offset values (can be negative)
        - homing_offset stores zero point raw position
        """
        import select
        import sys

        logger.info("\n--- Starting UniArmL1 calibration (per-motor independent) ---")

        # ---------- Phase 1: Establish zero point origin ----------
        print("\n[Phase 1/2] Please move arm to [zero position]. Press Enter to lock zero...")
        zero_offsets = {}  # motor_id -> raw value
        while True:
            fresh_states = self.bus.set_zero_damping()
            time.sleep(0.05)

            # Read raw data for all motors
            motor_raw_values = {}
            for st in fresh_states:
                if st.get("OutPos") is not None:
                    motor_raw_values[st["motor_id"]] = st["OutPos"]

            # Display all motor data (sorted by motor ID)
            line_parts = []
            for mid in sorted(motor_raw_values.keys()):
                joint_name = self.motor_to_joint.get(mid, "?")
                line_parts.append(f"M{mid}({joint_name[:4]}):{motor_raw_values[mid]:6.1f}")
            print("\r" + " | ".join(line_parts), end="", flush=True)

            if select.select([sys.stdin], [], [], 0.05)[0]:
                sys.stdin.readline()
                if len(motor_raw_values) == self.n_motors:
                    # Save zero point for each motor independently
                    for motor_id in self.motor_ids:
                        zero_offsets[motor_id] = motor_raw_values[motor_id]
                    break
                print(f"\n❌ Still have {len(motor_raw_values)}/{self.n_motors} motors offline, continuing...")

        print(f"\nZero point locked: {zero_offsets}")

        # Build calibration dict: each motor independently
        self.calibration = {}
        for motor_id in self.motor_ids:
            joint_name = self.motor_to_joint[motor_id]
            self.calibration[motor_id] = MotorCalibration(
                motor_id=motor_id,
                joint_name=joint_name,
                homing_offset=float(zero_offsets[motor_id]),
                range_min=0.0,
                range_max=0.0,
            )

        # Temporary ranges (each motor independently)
        temp_ranges = {motor_id: {"min": 0.0, "max": 0.0} for motor_id in self.motor_ids}

        # ---------- Phase 2: Record ranges in unwrapped space ----------
        print("\n[Phase 2/2] Please move to each joint's max/min limits. Press Enter to finish.")
        print("Tip: The program will automatically handle wrap-around cases, you just need to move to physical limits.\n")

        last_print = 0.0
        while True:
            fresh_states = self.bus.set_zero_damping()
            time.sleep(0.05)

            # Read raw data for all motors
            motor_raw_values = {}
            for st in fresh_states:
                if st.get("OutPos") is not None:
                    motor_raw_values[st["motor_id"]] = st["OutPos"]

            # Update range for each motor
            for motor_id in self.motor_ids:
                if motor_id not in motor_raw_values:
                    continue

                cal = self.calibration[motor_id]
                P_raw = motor_raw_values[motor_id]
                P_zero = cal.homing_offset

                # Calculate signed shortest distance (unwrapped offset)
                delta_P = (P_raw - P_zero + OUT_POS_RES/2) % OUT_POS_RES - OUT_POS_RES/2

                # Update range
                temp_ranges[motor_id]["min"] = min(temp_ranges[motor_id]["min"], delta_P)
                temp_ranges[motor_id]["max"] = max(temp_ranges[motor_id]["max"], delta_P)

            # 10Hz clear table refresh
            now = time.time()
            if now - last_print > 0.1:
                last_print = now
                sys.stdout.write("\x1b[2J\x1b[H")
                print(f"[Phase 2/2] Recording range  Press Enter to finish  (total {self.n_motors} motors)\n")

                # Display all motor raw data
                print("Motor raw data:")
                motor_line = "  ".join([f"M{mid}:{motor_raw_values.get(mid, 'OFF'):>7.1f}" if mid in motor_raw_values else f"M{mid}:   OFF" for mid in sorted(self.motor_ids)])
                print(motor_line)
                print()

                # Display calibration data for each motor
                print(f"{'Motor ID':>6} {'Joint':<14} {'Current raw':>8} {'Zero raw':>8} {'ΔP':>8} {'min_Δ':>8} {'max_Δ':>8} {'Span':>8}")
                print("-" * 88)

                for motor_id in sorted(self.motor_ids):
                    cal = self.calibration[motor_id]
                    if motor_id in motor_raw_values:
                        P_raw = motor_raw_values[motor_id]
                        P_zero = cal.homing_offset
                        delta_P = (P_raw - P_zero + OUT_POS_RES/2) % OUT_POS_RES - OUT_POS_RES/2
                        span = temp_ranges[motor_id]["max"] - temp_ranges[motor_id]["min"]

                        print(f"M{motor_id:>5} {cal.joint_name:<14} {P_raw:>8.1f} {P_zero:>8.1f} {delta_P:>8.1f} "
                              f"{temp_ranges[motor_id]['min']:>8.1f} {temp_ranges[motor_id]['max']:>8.1f} "
                              f"{span:>8.1f}")
                    else:
                        print(f"M{motor_id:>5} {cal.joint_name:<14} {'OFF':>8} {'-':>8} {'-':>8} {'-':>8} {'-':>8} {'-':>8}")

                sys.stdout.flush()

            if select.select([sys.stdin], [], [], 0.01)[0]:
                sys.stdin.readline()
                break

        # ---------- Save unwrapped values ----------
        print("\nSaving unwrapped ranges...")
        for motor_id in sorted(self.motor_ids):
            cal = self.calibration[motor_id]
            cal.range_min = float(temp_ranges[motor_id]["min"])
            cal.range_max = float(temp_ranges[motor_id]["max"])

            span = cal.range_max - cal.range_min
            print(f"M{motor_id} ({cal.joint_name:<12}): Zero raw={cal.homing_offset:>7.1f}, "
                  f"range=[{cal.range_min:>7.1f}, {cal.range_max:>7.1f}] (span={span:>7.1f})")

        self._save_calibration()
        logger.info(f"Calibration saved successfully: {self.calibration_fpath}")

    def map_output_delta_to_sim(self, id, delta_data):
        q_min, q_max = self._get_joint_limits(self.motor_names[id])
        if q_min is None:
            return 0.0  # gripper 等不在 URDF 中的关节
        # delta_data 是 ticks，需要先转换为 rad
        delta_rad = OutPosRaw2OutPos_rad(int(delta_data))
        delta_sim = delta_rad * self.ratio_sim_phys[id]
        q_sim = q_min + delta_sim
        return q_sim

    def map_sim_to_output_delta(self, id, q_sim):
        q_min, q_max = self._get_joint_limits(self.motor_names[id])
        if q_min is None:
            return 0.0  # gripper 等不在 URDF 中的关节
        delta_q_sim = q_sim - q_min
        delta_output_from_min = delta_q_sim * (1/self.ratio_sim_phys[id])
        motor_ids = self.joint_motor_ids[self.motor_names[id]]
        cal = self.calibration.get(motor_ids[0])
        if cal:
            delta_output_from_homing = cal.range_min + delta_output_from_min
        else:
            delta_output_from_homing = delta_output_from_min
        return delta_output_from_homing

    def map_sim_to_output_rad(self, id, q_sim):
        """将 sim 空间角度映射到电机输出角度

        使用线性插值：
        - URDF q_min → 电机 range_min
        - URDF q_max → 电机 range_max
        """
        q_min, q_max = self._get_joint_limits(self.motor_names[id])
        if id == 5:
            # 夹爪特殊处理：方向需要根据 leader/follower 区分
            q_min, q_max = 0.0, 0.0285
            s = (q_sim - q_min) / (q_max - q_min)  # 归一化 [0, 1]

            is_leader = self.config.id == "leader"
            if is_leader:
                # leader: URDF min → 电机 range_min, URDF max → 电机 range_max
                output_rad = self.motor_range_min_rad[id] + s * (self.motor_range_max_rad[id] - self.motor_range_min_rad[id])
            else:
                # follower: URDF max → 电机 range_min, URDF min → 电机 range_max
                output_rad = self.motor_range_max_rad[id] - s * (self.motor_range_max_rad[id] - self.motor_range_min_rad[id])

            output_rad = np.clip(output_rad, self.motor_range_min_rad[id], self.motor_range_max_rad[id])
            return output_rad

        # 线性插值：归一化到 [0, 1]
        if q_max != q_min:
            s = (q_sim - q_min) / (q_max - q_min)
        else:
            s = 0.0

        # 映射到电机物理范围
        output_rad = self.range_min_rad[id] + s * (self.range_max_rad[id] - self.range_min_rad[id])

        # 限制在电机物理范围内
        output_rad = np.clip(output_rad, self.range_min_rad[id], self.range_max_rad[id])

        return output_rad

    def map_sim_to_output_rad_all(self, q_sim):
        """将sim空间关节角度映射到所有电机角度（包括双电机反向）

        双电机逻辑：
        - M1 目标由 sim 映射计算
        - M6 目标 = M6零点 - (M1目标 - M1零点) = M6零点 + M1零点 - M1目标
        - 这样保证两个电机偏差完全对称
        """
        motor_pos_rad = np.zeros(self.n_motors)

        for i in range(self.n_joints):
            motor_ids = self.joint_motor_ids[self.motor_names[i]]

            if len(motor_ids) == 1:
                # 单电机：直接映射
                output = self.map_sim_to_output_rad(i, q_sim[i])
                motor_pos_rad[motor_ids[0]] = output
            else:
                # 双电机：M1正常映射，M6反向偏差
                m1_target = self.map_sim_to_output_rad(i, q_sim[i])
                motor_pos_rad[motor_ids[0]] = m1_target

                # M6 = M6_homing - (M1_target - M1_homing)
                m1_homing = self.motor_homing_rad[motor_ids[0]]
                m6_homing = self.motor_homing_rad[motor_ids[1]]
                m6_target = m6_homing - (m1_target - m1_homing)
                motor_pos_rad[motor_ids[1]] = m6_target

        return motor_pos_rad
    # 计算目标力矩
    def cal_target_torque(self):
        '''
        根据当前位置，计算重力补偿所需力矩
        '''
        q_now = self.get_cur_q_sim()
        # pin_model 需要 7 维输入
        nq = self.pin_model.nq
        q_full = np.zeros(nq)
        q_full[:len(q_now)] = q_now
        tau = pin.computeGeneralizedGravity(self.pin_model, self.pin_data, q_full)
        # 只取前 n_joints 个关节的力矩
        self.dynamics_tau = tau[:self.n_joints]
        return self.dynamics_tau

    def get_cur_v_sim(self):
        '''获取当前关节速度（sim空间），从电机速度读取'''
        if self.sim_mode or not self.bus:
            return self.v_current_sim
        for i, joint_name in enumerate(self.motor_names):
            motor_ids = self.joint_motor_ids[joint_name]
            # 用第一个电机的速度
            motor_idx = motor_ids[0]
            if motor_idx < len(self.bus.motor_ids):
                st = self.bus.motors.motor_states.get(motor_idx)
                if st:
                    # 电机速度 (rad/s) 需要通过 ratio 映射到 sim 空间
                    # 注意：速度方向可能需要根据关节映射调整
                    self.v_current_sim[i] = st.speed_rads * self.ratio_sim_phys[i]
        return self.v_current_sim

    def cal_full_torque(self, q=None, v=None, a=None, use_target_acc=False):
        '''完整逆动力学计算：惯性 + 科氏力 + 重力

        参数:
            q: 关节位置，默认使用当前位置
            v: 关节速度，默认从电机读取
            a: 关节加速度，默认通过数值微分计算或设为0
            use_target_acc: 是否使用目标加速度（需要轨迹规划）

        返回: 各关节所需扭矩
        '''
        if q is None:
            q = self.get_cur_q_sim()
        if v is None:
            v = self.get_cur_v_sim()

        if a is None:
            if use_target_acc:
                # 目标加速度（需要外部设置，例如轨迹规划器）
                a = self.a_current_sim
            else:
                # 通过数值微分计算加速度
                now = time.time()
                dt = now - self.last_dyn_time
                if dt > 0.001:  # 防止 dt 太小导致数值不稳定
                    a = (self.v_current_sim - self.v_last_sim) / dt
                    self.v_last_sim = self.v_current_sim.copy()
                    self.last_dyn_time = now
                else:
                    a = np.zeros(self.n_joints)

        # 使用 Pinocchio RNEA 算法计算完整逆动力学
        # q, v, a 是 6 维，但 pin_model 需要 7 维
        nq = self.pin_model.nq
        q_full = np.zeros(nq)
        v_full = np.zeros(nq)
        a_full = np.zeros(nq)
        q_full[:len(q)] = q
        v_full[:len(v)] = v
        a_full[:len(a)] = a

        tau = pin.rnea(self.pin_model, self.pin_data, q_full, v_full, a_full)

        self.dynamics_tau_full = tau
        return tau*0.05

    def map_raw_to_q_sim(self, joint_name, p_raw):
        motor_ids = self.joint_motor_ids[joint_name]
        cal = self.calibration.get(motor_ids[0])
        if cal is None:
            return 0.0

        q_min, q_max = self._get_joint_limits(joint_name)

        span_ticks = cal.range_max - cal.range_min
        # 物理行程跨度（ticks）
        span_ticks = (cal.range_max - cal.range_min + OUT_POS_RES) % OUT_POS_RES
        if span_ticks <= 0:
            return q_min

        # 相对位移（ticks）
        p_offset = (p_raw - cal.range_min + OUT_POS_RES) % OUT_POS_RES
        s = p_offset / span_ticks  # 归一化 [0,1]

        # 线性映射到仿真区间
        q_sim = q_min + s * (q_max - q_min)


        return np.clip(q_sim, min(q_min, q_max), max(q_min, q_max))

    def _get_joint_limits(self, joint_name):
        joint_id = self.pin_model.getJointId(joint_name)
        if joint_id >= len(self.pin_model.joints):
            return None, None  # joint 不存在
        idx_q = self.pin_model.joints[joint_id].idx_q
        q_min = self.pin_model.lowerPositionLimit[idx_q]
        q_max = self.pin_model.upperPositionLimit[idx_q]
        return q_min, q_max


    def init_rate_phys_sim_spans(self):

        for id, joint_name in enumerate(self.motor_names):
            q_min, q_max = self._get_joint_limits(joint_name)
            if q_min is None:
                # 跳过 URDF 中不存在的 joint（如 gripper）
                print(f"  {joint_name}: not in URDF, skipping")
                continue
            self.sim_spans[id] = q_max - q_min
            motor_ids = self.joint_motor_ids[joint_name]
            cal = self.calibration.get(motor_ids[0])
            if cal:
                # 将 ticks 转换为 rad
                phys_span_ticks = cal.range_max - cal.range_min
                self.phys_spans[id] = OutPosRaw2OutPos_rad(int(phys_span_ticks))
                if self.phys_spans[id] > 0:
                    self.ratio_sim_phys[id] = self.sim_spans[id] / self.phys_spans[id]
                else:
                    self.ratio_sim_phys[id] = 1.0
                print(f"  {joint_name}: sim_span={self.sim_spans[id]:.4f} rad, phys_span={self.phys_spans[id]:.4f} rad, ratio={self.ratio_sim_phys[id]:.6f}")



    def get_cur_q_sim(self):
        if self.sim_mode or not self.bus:
            return self.q_current_sim
        # 夹爪 URDF 限位（gripper 不在 URDF 中）
        GRIPPER_Q_MIN, GRIPPER_Q_MAX = 0.0, 0.0285
        # 判断是否为 leader（主臂夹爪方向与从臂相反）
        is_leader = self.config.id == "leader"

        for i, joint_name in enumerate(self.motor_names):
            motor_ids = self.joint_motor_ids[joint_name]
            motor_id = motor_ids[0]  # 用第一个电机
            cal = self.calibration.get(motor_id)
            if cal is None:
                continue
            motor_state = self.bus.motors.motor_states.get(motor_id) if self.bus else None
            if motor_state is None:
                continue
            delta_follower = (motor_state.OutPos - cal.homing_offset + OUT_POS_RES/2) % OUT_POS_RES - OUT_POS_RES/2 - cal.range_min

            # 夹爪特殊处理
            if i == 5:
                # 将电机位置线性映射到 URDF 限位 [0, 0.0285]
                delta_rad = OutPosRaw2OutPos_rad(int(delta_follower))
                # delta_rad 范围: [0, motor_range_span]
                motor_range_span = self.motor_range_max_rad[5] - self.motor_range_min_rad[5]
                if motor_range_span > 0:
                    s = delta_rad / motor_range_span  # 归一化 [0, 1]
                else:
                    s = 0.0
                if is_leader:
                    # leader: 电机 range_min → URDF min (闭合), 电机 range_max → URDF max (打开)
                    self.q_current_sim[i] = GRIPPER_Q_MIN + s * (GRIPPER_Q_MAX - GRIPPER_Q_MIN)
                else:
                    # follower: 电机 range_min → URDF max (打开), 电机 range_max → URDF min (闭合)
                    self.q_current_sim[i] = GRIPPER_Q_MAX - s * (GRIPPER_Q_MAX - GRIPPER_Q_MIN)
            else:
                self.q_current_sim[i] = self.map_output_delta_to_sim(i, delta_follower)
        return self.q_current_sim

    def start_control_loop(self):
        """启动控制循环线程"""
        if self._control_thread is not None and self._control_thread.is_alive():
            logger.info("control loop already running, skip.")
            return

        self._running = True
        self._control_thread = threading.Thread(
            target=self._control_loop_worker,
            daemon=True,
            name="uniarm_l1_control_loop",
        )
        self._control_thread.start()

    def move_to_init_position(self, duration: float | None = None):
        """平滑移动到初始位置

        Args:
            duration: 移动时间（秒），默认使用配置中的 init_move_duration
        """
        if not self.bus:
            return
        if duration is None:
            duration = self.config.init_move_duration

        if duration <= 0:
            # 立即跳转
            self.tgt_pos_rad = self.q_init_rad.copy()
            self.con_mode[:] = 1
            print("⚠️ init_move_duration=0，立即跳转到初始位置")
            return

        # 获取当前位置
        start_pos = np.array([
            self.bus.motors.motor_states[mid].OutPos_rad
            for mid in self.motor_ids
        ])

        end_pos = self.q_init_rad.copy()

        # 检查是否已经在目标位置
        max_error = np.max(np.abs(end_pos - start_pos))
        if max_error < 0.01:  # 误差小于0.01rad，认为已经在目标位置
            print(f"✅ 已在初始位置附近 (max_error={max_error:.4f} rad)")
            return

        print(f"平滑移动到初始位置: duration={duration}s, max_error={max_error:.3f} rad")

        # 使能所有电机
        self.con_mode[:] = 1
        # 临时切换到位置控制模式（非 zero_damping）
        original_control_mode = self.bus.control_mode
        self.bus.control_mode = "position_control"

        # 使用梯形速度曲线（加速-匀速-减速）
        control_dt = 0.01
        steps = int(duration / control_dt)

        for step in range(steps + 1):
            t = step / steps  # 归一化时间 [0, 1]

            # 梯形速度曲线：smoothstep函数
            # 前1/4加速，中间1/2匀速，后1/4减速
            if t < 0.25:
                # 加速阶段
                s = 2 * t * t  # 0 -> 0.125
            elif t < 0.75:
                # 匀速阶段
                s = 0.125 + (t - 0.25)  # 0.125 -> 0.625
            else:
                # 减速阶段
                t_rel = (t - 0.75) * 4  # 0 -> 1
                s = 0.625 + 0.375 * (2 * t_rel - t_rel * t_rel)  # 0.625 -> 1

            # 线性插值
            self.tgt_pos_rad = start_pos + s * (end_pos - start_pos)

            # 等待控制循环执行
            time.sleep(control_dt)

        # 确保到达目标位置
        self.tgt_pos_rad = end_pos.copy()

        # 恢复原来的控制模式和增益参数
        self.bus.control_mode = original_control_mode

        print("✅ 已到达初始位置")

    def stop_control_loop(self):
        """停止控制循环"""
        for cam in self.cameras.values():
            try:
                cam.disconnect()
            except Exception:
                pass

        if not self.bus:
            self._running = False
            if self._control_thread:
                self._control_thread.join(timeout=1)
            return

        # 位置控制模式：移动到home位置后切换零阻尼
        self.tgt_pos_rad = self.map_sim_to_output_rad_all(self.home_set)
        time.sleep(0.1)
        self.bus.control_mode = "zero_damping"
        time.sleep(0.1)  # 确保命令发送出去
        self._running = False
        if self._control_thread:
            self._control_thread.join(timeout=1)
    def _control_loop_worker(self):
        '''控制循环线程函数'''
        failed_cameras = []
        for name, cam in self.cameras.items():
            try:
                cam.connect(warmup=False)
            except Exception as e:
                logger.warning(f"Camera connect failed: {e}")
                failed_cameras.append(name)
        for name in failed_cameras:
            self.cameras.pop(name, None)
        while self._running:
            self._position_control_step()
            time.sleep(0.01)

    def _position_control_step(self):
        '''位置控制模式（默认）'''
        if not self.bus:
            return
        self.tgt_tau = self.cal_full_torque()

        # 直接发送电机级别命令
        self.bus.tgt_pos_rad[:] = self.tgt_pos_rad

        # 将关节力矩映射到电机力矩
        motor_tau = np.zeros(self.n_motors)
        for i in range(self.n_joints):
            motor_ids = self.joint_motor_ids[self.motor_names[i]]
            if len(motor_ids) == 1:
                motor_tau[motor_ids[0]] = self.tgt_tau[i]
            else:
                # 双电机：第一个正向，第二个反向（与位置映射一致）
                motor_tau[motor_ids[0]] = self.tgt_tau[i] / 2
                motor_tau[motor_ids[1]] = -self.tgt_tau[i] / 2

        self.bus.tgt_torque[:] = motor_tau
        self.bus.con_mod = self.con_mode.copy()

        self.bus.close_control()

        # 更新当前位置
        for mid in self.bus.motor_ids:
            self.cur_pos_rad[mid] = self.bus.motors.motor_states[mid].OutPos_rad

        # 读取电机实际输出扭矩
        self.actual_tau = np.array([
            self.bus.motors.motor_states[mid].torque_nm
            for mid in self.motor_ids
        ])

    def step(self, active, dxyz, drot, trigger):
        """计算目标关节角度 (sim空间)

        返回: 6个关节的sim角度
        """
        q_sim = np.zeros(self.n_joints)  # 6个关节（含gripper）
        n_ik_joints = 5  # IK关节数量（不含gripper）

        if not active:
            # 未激活：保持当前位置
            q_sim[:n_ik_joints] = np.deg2rad(self.last_sim_q_deg[:n_ik_joints])
            # 更新基准位置为当前末端位置，下次激活时从这里继续
            self.ee_pose = self.ik_solver.forward_kinematics(self.last_sim_q_deg[:n_ik_joints])
            self.target_pos_base = self.ee_pose[:3, 3].copy()
            self.target_ori = self.ee_pose[:3, :3].copy()
            self._step_first_active = True  # 下次激活时平滑过渡

        else:
            # 激活：IK计算
            # 第一次激活时，限制姿态变化幅度（避免wrist_roll跳变）
            if self._step_first_active:
                if drot is not None:
                    # 限制第一次激活时的旋转幅度
                    drot = np.clip(drot, -0.05, 0.05)  # 只允许很小的初始偏移
                self._step_first_active = False

            # print(dxyz)
            if dxyz is not None:
                target_pos = self.target_pos_base + dxyz
            else:
                target_pos = self.target_pos_base

            if drot is not None and np.linalg.norm(drot) > 1e-6:
                drot_mat = R.from_rotvec(drot).as_matrix()
                target_ori_new = self.target_ori @ drot_mat
            else:
                target_ori_new = self.target_ori

            t_des = np.eye(4, dtype=float)
            t_des[:3, :3] = target_ori_new
            t_des[:3, 3] = target_pos
            # print(f"{t_des}")

            q_target_deg = self.ik_solver.inverse_kinematics(
                current_joint_pos=self.last_sim_q_deg,
                desired_ee_pose=t_des,
                position_weight=1.0,
                orientation_weight=0.01,
                joint_regularization_weight=0.001,  # 大幅降低正则化，让IK能自由求解
                smoothing_weight=0.001,
            )
            self.ik_solver.forward_kinematics(q_target_deg)


            # 滤波
            dead_zone = 0.5
            jump_threshold = 50.0
            max_delta_deg = 5.0

            delta_deg = q_target_deg - self.last_sim_q_deg
            max_change = np.max(np.abs(delta_deg))

            if max_change < dead_zone:
                q_target_deg_final = self.last_sim_q_deg
            elif max_change > jump_threshold:
                print(f"⚠️ 跳变: {max_change:.1f} deg")
                delta_deg = np.clip(delta_deg, -max_delta_deg, max_delta_deg)
                q_target_deg_final = self.last_sim_q_deg + delta_deg
            else:
                q_target_deg_final = q_target_deg

            self.last_sim_q_deg = q_target_deg_final.copy()
            q_sim[:n_ik_joints] = np.deg2rad(q_target_deg_final[:n_ik_joints])

        gripper_idx = 5
        q_min_gripper, q_max_gripper = 0.0, 0.0285 
        if trigger > 0.8:
            q_sim[gripper_idx] = q_max_gripper  # 0 -> 闭合
        else:
            q_sim[gripper_idx] = q_min_gripper  # 0.0285 -> 打开

        return q_sim

    def reset_to_home(self):
        """Reset to initial position and update target base position."""
        n_ik_joints = 5
        self.last_sim_q_deg = np.rad2deg(self.q_init_sim_rad).copy()
        self.ee_pose = self.ik_solver.forward_kinematics(self.last_sim_q_deg[:n_ik_joints])
        self.target_pos_base = self.ee_pose[:3, 3].copy()
        self.target_ori = self.ee_pose[:3, :3].copy()
        self._step_first_active = True
        return self.q_init_sim_rad.copy()
