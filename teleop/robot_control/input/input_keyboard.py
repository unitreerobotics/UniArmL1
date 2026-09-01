"""Keyboard input source for teleoperation.

Uses terminal raw mode to capture keypresses without waiting for Enter.
All controls are through the terminal — no cv2 dependency for keyboard mode.

Controls:
    i/k : X axis +/- (forward/backward)
    j/l : Y axis +/- (left/right)
    u/o : Z axis +/- (up/down)
    r/f : Rotate around Z +/-
    q/e : Rotate around Y +/- (yaw)
    w/z : Rotate around X +/- (pitch)
    g   : Toggle gripper
    s   : Toggle recording
    d   : Delete last episode
    h   : Reset to home
    x   : Quit
"""

from __future__ import annotations

import sys
import select
import termios
import tty

import numpy as np

from .input_source import InputSource


class KeyboardInput(InputSource):
    """Input from terminal keyboard for teleoperation."""

    def __init__(
        self,
        pos_step: float = 0.005,
        rot_step: float = 0.05,
    ):
        self.pos_step = pos_step
        self.rot_step = rot_step

        self.dxyz = np.zeros(3)
        self.drot = np.zeros(3)
        self.gripper_open = True
        self.trigger = 0.0

        self._old_settings = None
        self._started = False
        self._quit = False

        # Button state for teleop_runner edge detection
        self._buttons: dict[str, bool] = {
            "A": False,  # record toggle
            "B": False,  # delete episode
            "X": False,  # reset home
        }

    def start(self) -> None:
        """Start keyboard input (set terminal to raw mode)."""
        if self._started:
            return
        self._old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin)
        self._started = True

    def get_delta(self) -> tuple[bool, np.ndarray | None, np.ndarray | None, float]:
        if not self._started:
            self.start()

        # Reset one-shot buttons each frame
        self._buttons = {"A": False, "B": False, "X": False}

        key = self._read_key()
        if key is None:
            return True, self.dxyz.copy(), self.drot.copy(), self.trigger

        if key == "x":
            self._quit = True
            return False, None, None, self.trigger
        elif key == "i":
            self.dxyz[0] += self.pos_step
        elif key == "k":
            self.dxyz[0] -= self.pos_step
        elif key == "j":
            self.dxyz[1] += self.pos_step
        elif key == "l":
            self.dxyz[1] -= self.pos_step
        elif key == "u":
            self.dxyz[2] += self.pos_step
        elif key == "o":
            self.dxyz[2] -= self.pos_step
        elif key == "r":
            self.drot[2] += self.rot_step
        elif key == "f":
            self.drot[2] -= self.rot_step
        elif key == "q":
            self.drot[1] += self.rot_step
        elif key == "e":
            self.drot[1] -= self.rot_step
        elif key == "w":
            self.drot[0] += self.rot_step
        elif key == "n":
            self.drot[0] -= self.rot_step
        elif key == "g":
            self.gripper_open = not self.gripper_open
            self.trigger = 0.0 if self.gripper_open else 1.0
        elif key == "s":
            self._buttons["A"] = True
        elif key == "d":
            self._buttons["B"] = True
        elif key == "h":
            self._buttons["X"] = True

        return True, self.dxyz.copy(), self.drot.copy(), self.trigger

    def get_button(self, name: str) -> bool:
        return self._buttons.get(name, False)

    @property
    def should_quit(self) -> bool:
        return self._quit

    def close(self) -> None:
        if self._started and self._old_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)
            self._started = False

    @staticmethod
    def _read_key() -> str | None:
        if select.select([sys.stdin], [], [], 0.0)[0]:
            ch = sys.stdin.read(1)
            # Consume ESC sequences (arrow keys, function keys, etc.)
            if ch == "\x1b":
                # Read and discard the rest of the escape sequence
                while select.select([sys.stdin], [], [], 0.01)[0]:
                    sys.stdin.read(1)
                return None
            return ch
        return None
