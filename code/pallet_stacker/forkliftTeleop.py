"""ROS 2 keyboard teleoperation for the simplified forklift.

Controls
--------
W / S
    Forward / reverse.
A / D
    Turn left / right.
Up / Down arrows
    Raise / lower both forks. The node publishes a position target, so the
    target remains active when the key is released.
Space
    Stop chassis motion immediately.
Q
    Stop and exit.

Run with the ROS 2 system Python, not an isolated virtual environment that
cannot see apt-installed ``rclpy``::

    source /opt/ros/jazzy/setup.bash
    PYTHONPATH=code /usr/bin/python3 -m pallet_stacker.forkliftTeleop
"""

from __future__ import annotations

import os
import select
import sys
import termios
import time
import tty
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64

from . import settings


UP = "UP"
DOWN = "DOWN"


@dataclass(slots=True)
class _AxisCommand:
    target: float = 0.0
    current: float = 0.0
    expires_at: float = 0.0


class _RawKeyboard:
    """Non-blocking terminal reader with ANSI arrow-sequence parsing."""

    def __init__(self) -> None:
        if not sys.stdin.isatty():
            raise RuntimeError("Forklift teleop requires an interactive terminal.")
        self._fd = sys.stdin.fileno()
        self._original = termios.tcgetattr(self._fd)
        self._buffer = b""

    def __enter__(self) -> "_RawKeyboard":
        tty.setcbreak(self._fd)
        os.set_blocking(self._fd, False)
        return self

    def __exit__(self, *_: object) -> None:
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._original)
        os.set_blocking(self._fd, True)

    def read_keys(self) -> list[str]:
        keys: list[str] = []
        while select.select([self._fd], [], [], 0.0)[0]:
            try:
                chunk = os.read(self._fd, 64)
            except BlockingIOError:
                break
            if not chunk:
                break
            self._buffer += chunk

        while self._buffer:
            if self._buffer.startswith(b"\x1b[A"):
                keys.append(UP)
                self._buffer = self._buffer[3:]
                continue
            if self._buffer.startswith(b"\x1b[B"):
                keys.append(DOWN)
                self._buffer = self._buffer[3:]
                continue
            if self._buffer.startswith(b"\x1b") and len(self._buffer) < 3:
                break
            if self._buffer.startswith(b"\x1b["):
                # Ignore unsupported ANSI cursor/function key sequences.
                self._buffer = self._buffer[3:]
                continue

            byte = self._buffer[0]
            self._buffer = self._buffer[1:]
            if byte == 3:  # Ctrl-C
                raise KeyboardInterrupt
            try:
                keys.append(chr(byte).lower())
            except ValueError:
                continue
        return keys


class ForkliftTeleop(Node):
    """Publish ramp-limited Twist commands and persistent fork targets."""

    def __init__(self) -> None:
        super().__init__("forklift_teleop")
        self._twist_publisher = self.create_publisher(
            Twist, settings.FORKLIFT_CMD_VEL_TOPIC, 10
        )
        self._fork_publisher = self.create_publisher(
            Float64, settings.FORKLIFT_FORK_POSITION_TOPIC, 10
        )

        self._linear = _AxisCommand()
        self._angular = _AxisCommand()
        self._fork_target = settings.FORKLIFT_FORK_INITIAL_POSITION_M
        self._last_tick = time.monotonic()
        self._stop_requested = False

        period = 1.0 / settings.FORKLIFT_TELEOP_RATE_HZ
        self._timer = self.create_timer(period, self._tick)
        self._publish_fork_target()
        self.get_logger().info(
            "W/S drive, A/D turn, arrows lift/lower, SPACE stop, Q quit"
        )

    @property
    def stop_requested(self) -> bool:
        return self._stop_requested

    def handle_key(self, key: str) -> None:
        now = time.monotonic()
        expiry = now + settings.FORKLIFT_KEY_HOLD_TIMEOUT_S

        if key == "w":
            self._linear.target = settings.FORKLIFT_MAX_LINEAR_VELOCITY_MPS
            self._linear.expires_at = expiry
        elif key == "s":
            self._linear.target = -settings.FORKLIFT_MAX_LINEAR_VELOCITY_MPS
            self._linear.expires_at = expiry
        elif key == "a":
            self._angular.target = settings.FORKLIFT_MAX_TURN_VELOCITY_RAD_S
            self._angular.expires_at = expiry
        elif key == "d":
            self._angular.target = -settings.FORKLIFT_MAX_TURN_VELOCITY_RAD_S
            self._angular.expires_at = expiry
        elif key == UP:
            self._set_fork_target(
                self._fork_target + settings.FORKLIFT_FORK_KEY_STEP_M
            )
        elif key == DOWN:
            self._set_fork_target(
                self._fork_target - settings.FORKLIFT_FORK_KEY_STEP_M
            )
        elif key == " ":
            self._linear.target = 0.0
            self._linear.current = 0.0
            self._linear.expires_at = 0.0
            self._angular.target = 0.0
            self._angular.current = 0.0
            self._angular.expires_at = 0.0
            self._publish_twist()
        elif key == "q":
            self._stop_requested = True

    def stop(self) -> None:
        self._linear.target = self._linear.current = 0.0
        self._angular.target = self._angular.current = 0.0
        self._publish_twist()
        self._publish_fork_target()

    def _tick(self) -> None:
        now = time.monotonic()
        dt = max(0.0, min(now - self._last_tick, 0.25))
        self._last_tick = now

        if now >= self._linear.expires_at:
            self._linear.target = 0.0
        if now >= self._angular.expires_at:
            self._angular.target = 0.0
        self._linear.current = _approach(
            self._linear.current,
            self._linear.target,
            settings.FORKLIFT_MAX_LINEAR_ACCELERATION_MPS2 * dt,
        )
        self._angular.current = _approach(
            self._angular.current,
            self._angular.target,
            settings.FORKLIFT_MAX_TURN_ACCELERATION_RAD_S2 * dt,
        )

        self._publish_twist()

    def _publish_twist(self) -> None:
        message = Twist()
        message.linear.x = float(self._linear.current)
        message.angular.z = float(self._angular.current)
        self._twist_publisher.publish(message)

    def _set_fork_target(self, target: float) -> None:
        clamped = min(
            settings.FORKLIFT_FORK_MAX_POSITION_M,
            max(settings.FORKLIFT_FORK_MIN_POSITION_M, target),
        )
        if clamped != self._fork_target:
            self._fork_target = clamped
            self._publish_fork_target()

    def _publish_fork_target(self) -> None:
        message = Float64()
        message.data = float(self._fork_target)
        self._fork_publisher.publish(message)


def _approach(current: float, target: float, maximum_delta: float) -> float:
    if current < target:
        return min(current + maximum_delta, target)
    if current > target:
        return max(current - maximum_delta, target)
    return target


def main() -> int:
    rclpy.init()
    node = ForkliftTeleop()
    try:
        with _RawKeyboard() as keyboard:
            while rclpy.ok() and not node.stop_requested:
                for key in keyboard.read_keys():
                    node.handle_key(key)
                rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
