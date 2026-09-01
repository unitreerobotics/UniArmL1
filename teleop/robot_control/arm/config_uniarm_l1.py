"""UniArmL1 robot configuration module."""

from dataclasses import dataclass, field
from typing import Callable, ClassVar, TypeAlias, TypeVar

from image_server.camera import CameraConfig


# ── Robot config base ──────────────────────────────────────────

T = TypeVar("T", bound="RobotConfig")


@dataclass
class RobotConfig:
    """Base class for robot configurations with subclass registry."""

    _registry: ClassVar[dict[str, type["RobotConfig"]]] = {}

    id: str | None = None

    @classmethod
    def register_subclass(cls, key: str) -> Callable[[type[T]], type[T]]:
        """Decorator to register a subclass with a key."""
        def decorator(subclass: type[T]) -> type[T]:
            cls._registry[key] = subclass
            return subclass
        return decorator

    @classmethod
    def get_subclass(cls, key: str) -> type["RobotConfig"] | None:
        return cls._registry.get(key)


# ── UniArmL1 hardware config ────────────────────────────────────

@dataclass
class UniArmL1Config:
    """UniArmL1 hardware configuration."""

    # Serial port
    port: str

    disable_torque_on_disconnect: bool = True

    # Safety limit: maximum single-step target change
    max_relative_target: float | dict[str, float] | None = None

    # Camera configuration
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    # Angle unit
    use_degrees: bool = False

    urdf_path: str | None = None

    use_vr: bool = False

    # Initialization move duration (seconds): smooth move from current position to initial position
    # Set to 0 for immediate jump (may cause jitter)
    init_move_duration: float = 2.0

    # Motor configuration: joint name -> motor ID list (supports dual motors)
    # Dual-motor joints: two motors drive the same joint synchronously
    joint_motor_ids: dict[str, list[int]] = field(default_factory=lambda: {
        "shoulder_pan": [0],
        "shoulder_lift": [1, 6],  # dual motor
        "elbow_flex": [2, 7],     # dual motor
        "wrist_flex": [3],
        "wrist_roll": [4],
        "gripper": [5],
    })

    # Motor PD control parameters (by motor ID order, index equals motor ID)
    kp_loop: list[float] | None = None
    kd_loop: list[float] | None = None

    # Motor default kp/kd (for motor control mode)
    kp_default: list[float] | None = None
    kd_default: list[float] | None = None

    # Run without real robot (simulation mode)
    no_real_robot: bool = False


@RobotConfig.register_subclass("uniarm_l1_follower")
@dataclass
class UniArmL1RobotConfig(RobotConfig, UniArmL1Config):
    pass


UniArmL1ConfigType: TypeAlias = UniArmL1RobotConfig


# ── Teleop flow config ─────────────────────────────────────────

@dataclass
class TeleopConfig:
    """Teleoperation flow configuration."""

    input: str = "vr"  # vr | keyboard | leader
    port: str = "/dev/ttyACM0"  # Follower arm serial port
    leader_port: str = "/dev/ttyACM3"  # Leader arm serial port
    urdf_path: str = "../assets/uniarml1/urdf/UniArmL1.urdf"
    cameras: list[str] = field(default_factory=lambda: ["head:1", "wrist:2"])
    no_camera: bool = False
    record: bool = False
    task_dir: str = "./data/teleop"
    task_goal: str = ""
    record_hz: int = 50
    meshcat: bool = False
    no_real_robot: bool = False
