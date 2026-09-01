"""Unified teleoperation runner that merges VR, keyboard, and leader-arm modes."""

import logging
import time

import cv2
import numpy as np

from .input.input_source import InputSource
from .input.input_keyboard import KeyboardInput
from .input.input_leader import LeaderArmInput
from .recorder import Recorder
from .arm.uniarm_l1 import UniArmL1
from robot_control.input.input_vr import VRInput

logger = logging.getLogger(__name__)


class TeleopRunner:
    """Unified teleoperation runner.

    Supports three input modes:
    - vr: VR controller with IK-based delta control
    - keyboard: Terminal keyboard with IK-based delta control
    - leader: Leader arm with direct joint control (no IK)

    Supports two recording modes:
    - episode: EpisodeWriter (JSON + images)
    - none: No recording
    """

    def __init__(
        self,
        robot: UniArmL1,
        input_source: InputSource,
        recorder: Recorder,
        leader: UniArmL1 | None = None,
        control_dt: float = 0.01,
        record_hz: int = 50,
        show_camera: bool = True,
        use_meshcat: bool = False,
        debug_rate: bool = False,
    ):
        self.robot = robot
        self.input = input_source
        self.recorder = recorder
        self.leader = leader
        self.control_dt = control_dt
        self.record_hz = record_hz
        self.show_camera = show_camera
        self.use_meshcat = use_meshcat
        self.debug_rate = debug_rate

        self._viz = None
        self._viz_model = None
        self._running = False
        self._last_button = {"record": False, "delete": False}
        self._next_record_t = 0.0
        self._loop_count = 0
        self._last_rate_t = 0.0
        self._max_loop_dt = 0.0
        self._debug_max_parts = {
            "input": 0.0,
            "map": 0.0,
            "camera": 0.0,
            "buttons": 0.0,
            "record": 0.0,
        }

    def setup(self) -> None:
        """Initialize robot control mode."""
        q_sim_direct = self.input.get_q_sim()

        if q_sim_direct is not None:
            # Leader arm mode: follower uses position control
            if self.robot.bus is not None:
                for _ in range(3):
                    self.robot.bus.set_zero_damping()
                self.robot.bus.control_mode = "control_mode"
                self.robot.tgt_pos_rad = self.robot.map_sim_to_output_rad_all(q_sim_direct)
            if self.leader is not None:
                if self.leader.bus is not None:
                    self.leader.bus.control_mode = "zero_damping"
        else:
            # IK/delta mode: position control
            if self.robot.bus is not None:
                self.robot.bus.control_mode = "control_mode"

        if self.robot.bus is not None:
            self.robot.start_control_loop()
        if self.leader is not None:
            time.sleep(1.0)

        # Meshcat visualization
        if self.use_meshcat and self.robot.urdf_path and self.robot.mesh_dir:
            import pinocchio as pin
            from pinocchio.visualize import MeshcatVisualizer

            model, collision_model, visual_model = pin.buildModelsFromUrdf(
                self.robot.urdf_path, self.robot.mesh_dir
            )
            self._viz = MeshcatVisualizer(model, collision_model, visual_model)
            self._viz.initViewer(open=True)
            self._viz.loadViewerModel("uniarm_l1")
            self._viz.display(pin.neutral(model))
            self._viz_model = model
            logger.info("Meshcat viewer started")

    def run(self) -> None:
        """Main teleoperation loop."""
        self._running = True
        self.robot.con_mode[:] = 1

        record_period = 1.0 / self.record_hz
        self._print_controls()
        self._next_record_t = time.monotonic()
        self._last_rate_t = self._next_record_t

        

        try:
            while self._running:
                loop_t0 = time.monotonic()
                if self.input.should_quit:
                    break

                # Compute target joint angles
                part_t = time.monotonic()
                q_sim = self._compute_q_sim()
                self._debug_max_parts["input"] = max(self._debug_max_parts["input"], time.monotonic() - part_t)

                # Visualization
                if self._viz is not None:
                    self._display_viz(q_sim)

                # Map sim -> motor and send
                part_t = time.monotonic()
                motor_pos_rad = self.robot.map_sim_to_output_rad_all(q_sim)
                self.robot.tgt_pos_rad = motor_pos_rad
                self._debug_max_parts["map"] = max(self._debug_max_parts["map"], time.monotonic() - part_t)

                # Camera frames
                part_t = time.monotonic()
                colors = {}
                if self.robot.cameras:
                    for name, cam in self.robot.cameras.items():
                        try:
                            colors[name] = cam.async_read()
                        except Exception:
                            colors[name] = None
                    if self.show_camera:
                        self._show_frames(colors)
                        key = cv2.waitKey(1) & 0xFF
                        if key == ord("q"):
                            break
                self._debug_max_parts["camera"] = max(self._debug_max_parts["camera"], time.monotonic() - part_t)

                # Handle buttons
                part_t = time.monotonic()
                self._handle_buttons()
                self._debug_max_parts["buttons"] = max(self._debug_max_parts["buttons"], time.monotonic() - part_t)

                # Record data at fixed frequency
                part_t = time.monotonic()
                self._record_tick(colors, q_sim, record_period)
                self._debug_max_parts["record"] = max(self._debug_max_parts["record"], time.monotonic() - part_t)

                # Reset to home
                self._handle_reset_button()

                self._log_loop_rate()
                self._max_loop_dt = max(self._max_loop_dt, time.monotonic() - loop_t0)
                time.sleep(self.control_dt)

        except KeyboardInterrupt:
            logger.info("User interrupted")
        finally:
            self._cleanup()

    def _compute_q_sim(self) -> np.ndarray:
        """Compute target joint angles based on input mode."""
        q_sim_direct = self.input.get_q_sim()

        if q_sim_direct is not None:
            # Leader arm: direct joint angles, no IK
            return q_sim_direct

        # IK/delta mode
        active, dxyz, drot, trigger = self.input.get_delta()
        # print(f"Delta input: active={active}, dxyz={dxyz}, drot={drot}, trigger={trigger}")
        return self.robot.step(active, dxyz, drot, trigger)

    def _display_viz(self, q_sim: np.ndarray) -> None:
        if self._viz_model is None:
            return

        q_viz = np.zeros(self._viz_model.nq)
        q_viz[:5] = q_sim[:5]
        q_viz[5] = q_sim[5]  # gripper_l
        q_viz[6] = q_sim[5]  # gripper_r mirrors gripper_l
        self._viz.display(q_viz)

    def _show_frames(self, colors: dict) -> None:
        for name, frame in colors.items():
            if frame is None:
                continue
            cv2.imshow(name, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))

    def _handle_buttons(self) -> None:
        # Toggle recording: VR "A" / keyboard "s" / leader "s"
        record_pressed = self.input.get_button("A")

        if record_pressed and not self._last_button["record"]:
            if not self.recorder.is_recording:
                logger.info("Starting recording")
                if self.recorder.start_episode():
                    self._next_record_t = time.monotonic()
            else:
                logger.info("Stopping recording")
                self.recorder.stop_episode()
        self._last_button["record"] = record_pressed

        # Delete last episode: VR "B" button
        delete_pressed = self.input.get_button("B")
        if delete_pressed and not self._last_button["delete"] and not self.recorder.is_recording:
            if self.recorder.delete_last_episode():
                logger.info("Deleted last episode")
        self._last_button["delete"] = delete_pressed

    def _record_tick(self, colors: dict, q_sim: np.ndarray,
                     record_period: float) -> None:
        if not self.recorder.is_recording:
            return
        now = time.monotonic()
        if now >= self._next_record_t:
            self.recorder.add_item(q_sim, colors)
            while self._next_record_t <= now:
                self._next_record_t += record_period

    def _log_loop_rate(self) -> None:
        if not self.debug_rate:
            return
        self._loop_count += 1
        now = time.monotonic()
        dt = now - self._last_rate_t
        if dt >= 2.0:
            logger.info(
                "teleop loop rate: %.1f Hz, max loop dt: %.1f ms "
                "(input %.1f, map %.1f, camera %.1f, buttons %.1f, record %.1f ms)",
                self._loop_count / dt,
                self._max_loop_dt * 1000.0,
                self._debug_max_parts["input"] * 1000.0,
                self._debug_max_parts["map"] * 1000.0,
                self._debug_max_parts["camera"] * 1000.0,
                self._debug_max_parts["buttons"] * 1000.0,
                self._debug_max_parts["record"] * 1000.0,
            )
            self._loop_count = 0
            self._last_rate_t = now
            self._max_loop_dt = 0.0
            for key in self._debug_max_parts:
                self._debug_max_parts[key] = 0.0

    def _handle_reset_button(self) -> None:
        if self.input.get_button("X"):
            q_home = self.robot.reset_to_home()
            motor_pos = self.robot.map_sim_to_output_rad_all(q_home)
            self.robot.tgt_pos_rad = motor_pos
            # 清除键盘累积的 delta
            if isinstance(self.input, KeyboardInput):
                self.input.dxyz[:] = 0
                self.input.drot[:] = 0

    def _print_controls(self) -> None:
        lines = ["=" * 50, "Teleoperation Controls"]
        # Check VR input by name to avoid import
        if isinstance(self.input, VRInput):
            lines.append("  VR Controls:")
            lines.append("    A button : Toggle recording")
            lines.append("    B button : Delete last episode")
            lines.append("    Grip     : Activate tracking")
            lines.append("    Trigger  : Close gripper")
            lines.append("    X button : Reset to home")
        elif isinstance(self.input, KeyboardInput):
            lines.append("  Keyboard Controls:")
            lines.append("    i/k : X axis +/- (forward/backward)")
            lines.append("    j/l : Y axis +/- (left/right)")
            lines.append("    u/o : Z axis +/- (up/down)")
            lines.append("    r/f : Rotate around Z +/-")
            lines.append("    q/e : Rotate around Y +/- (yaw)")
            lines.append("    w/n : Rotate around X +/- (pitch)")
            lines.append("    g   : Toggle gripper")
            lines.append("    s   : Toggle recording")
            lines.append("    d   : Delete last episode")
            lines.append("    h   : Reset to home")
            lines.append("    x   : Quit")
        elif isinstance(self.input, LeaderArmInput):
            lines.append("  Leader Arm Controls:")
            lines.append("    s   : Toggle recording")
            lines.append("    d   : Delete last episode")
            lines.append("    h   : Reset to home")
            lines.append("    q   : Quit")
        print("\n" + "\n".join(lines) + "\n")

    def _cleanup(self) -> None:
        if self.recorder.is_recording:
            self.recorder.stop_episode()
        self.recorder.close()
        self.robot.stop_control_loop()
        if self.leader is not None:
            self.leader.stop_control_loop()
        if self.show_camera:
            cv2.destroyAllWindows()
        self.input.close()
        logger.info("Disconnected")
