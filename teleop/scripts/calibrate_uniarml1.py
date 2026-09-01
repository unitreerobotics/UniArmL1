#!/usr/bin/env python3
"""
UniArmL1 机械臂校准脚本

用法:
    # 从臂 (8电机，双电机驱动 shoulder_lift 和 elbow_flex)
    python teleop/scripts/calibrate_uniarml1.py --port /dev/ttyACM0 --id follower

    # 主臂 (8电机，双电机驱动 shoulder_lift 和 elbow_flex)
    python teleop/scripts/calibrate_uniarml1.py --port /dev/ttyACM4 --id leader

校准文件保存位置:
    teleop/robot_control/calibration/{id}.json
"""

import argparse
import logging
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from robot_control.arm.uniarm_l1 import UniArmL1
from robot_control.arm.config_uniarm_l1 import UniArmL1RobotConfig

logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')

# 默认电机配置
DEFAULT_JOINT_MOTOR_IDS = {
    "leader": {
        # 从臂: 8电机，双电机驱动 shoulder_lift 和 elbow_flex
        "shoulder_pan": [0],
        "shoulder_lift": [1, 6],
        "elbow_flex": [2, 7],
        "wrist_flex": [3],
        "wrist_roll": [4],
        "gripper": [5],
    },
    "follower": {
        # 从臂: 8电机，双电机驱动 shoulder_lift 和 elbow_flex
        "shoulder_pan": [0],
        "shoulder_lift": [1, 6],
        "elbow_flex": [2, 7],
        "wrist_flex": [3],
        "wrist_roll": [4],
        "gripper": [5],
    },
}


def main():
    parser = argparse.ArgumentParser(description="UniArmL1 机械臂校准工具")
    parser.add_argument(
        "--port",
        type=str,
        default="/dev/ttyACM3",
        help="串口设备路径，例如 /dev/ttyACM1 或 /dev/ttyUSB2"
    )
    parser.add_argument(
        "--id",
        type=str,
        default="leader",
        choices=["leader","follower"],
    )

    args = parser.parse_args()

    # 根据 id 获取默认电机配置
    joint_motor_ids = DEFAULT_JOINT_MOTOR_IDS[args.id]
    n_motors = sum(len(ids) for ids in joint_motor_ids.values())

    print("=" * 60)
    print(f"UniArmL1 校准工具")
    print(f"  串口: {args.port}")
    print(f"  ID: {args.id}")
    print(f"  电机数: {n_motors}")
    print("=" * 60)

    # 创建配置 - 使用项目根目录的绝对路径
    project_root = Path(__file__).parent.parent.parent
    urdf_path = project_root / "assets/uniarml1/urdf/UniArmL1.urdf"

    config = UniArmL1RobotConfig(
        port=args.port,
        cameras={},
        id=args.id,
        urdf_path=str(urdf_path),
        joint_motor_ids=joint_motor_ids,
    )

    # 创建机械臂实例
    robot = UniArmL1(config)

    # 显示校准文件路径
    print(f"\n校准文件将保存到:")
    print(f"  {robot.calibration_fpath}")
    print()

    # 开始校准
    try:
        robot.calibrate()
    except KeyboardInterrupt:
        print("\n用户中断校准")
        sys.exit(1)

    # 验证校准结果
    if robot.calibration:
        print("\n" + "=" * 60)
        print("校准成功！数据已保存。")
        print("=" * 60)
        print("\n校准结果:")
        for name, cal in robot.calibration.items():
            print(f"  {name}:")
            print(f"    零点 (homing_offset): {cal.homing_offset:.1f}")
            print(f"    范围: [{cal.range_min:.1f}, {cal.range_max:.1f}]")
            print(f"    跨度: {cal.range_max - cal.range_min:.1f}")
    else:
        print("\n校准失败，请重试。")
        sys.exit(1)


if __name__ == "__main__":
    main()
