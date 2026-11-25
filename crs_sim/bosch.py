"""
Simulated Bosch SR450 SCARA, drop-in replacement of
``ctu_bosch_sr450.RobotBosch``.

The interface, the kinematics (``fk``, ``ik``, ``ik_xyz``), joint limits and
IRC conversion are taken from ``ctu_bosch_sr450`` 1.0.7 (BSD 3-Clause,
Copyright (c) 2023, CTURobotics, see LICENSE-ctu_bosch_sr450), so the
simulator returns exactly the same poses and IK solutions as the real robot.

Joints: q[0], q[1] revolute (q[1] relative to arm 1), q[2] prismatic height
[m], q[3] tool rotation measured from the base x axis (absolute, not relative
to arm 2). ``fk`` returns (x, y, z, phi), :meth:`RobotBosch.flange_pose` the
same as a 4x4 pose with z along the tool (pointing down), so that the hoop
sits at ``flange_pose(q) @ hoop_in_flange`` like on the other robots.
"""
from __future__ import annotations

import time
from pathlib import Path

import numpy as np
from numpy.typing import ArrayLike

from .sim_robot import SimRobot
from .transforms import pose, ry

URDF_PATH_SR450 = Path(__file__).parent / "urdf" / "SR450.urdf"


def circle_circle_intersection(
    c0: ArrayLike, r0: float, c1: ArrayLike, r1: float, tol: float = 1e-9
) -> list[np.ndarray]:
    """Intersection(s) of two circles, as in ctu_bosch_sr450: two points (equal
    when tangent), none if they do not intersect, two random points of the
    circle if they coincide."""
    c0 = np.asarray(c0)
    c1 = np.asarray(c1)
    d = np.linalg.norm(c1 - c0)

    if np.isclose(d, 0) and np.isclose(r0, r1):
        return [
            c0 + [r0 * np.cos(a), r0 * np.sin(a)]
            for a in np.random.uniform(-np.pi, np.pi, 2)
        ]
    if d > r0 + r1 + tol or d < np.abs(r0 - r1) - tol:
        return []

    a = (r0**2 - r1**2 + d**2) / (2 * d)
    h_squared = ((r0 + r1) ** 2 - d**2) * (d**2 - (r0 - r1) ** 2) / (4 * d**2)
    h = np.sqrt(max(0.0, h_squared))

    x0, y0 = c0
    x1, y1 = c1
    x2 = x0 + a * (x1 - x0) / d
    y2 = y0 + a * (y1 - y0) / d
    x3 = x2 + h * (y1 - y0) / d
    y3 = y2 - h * (x1 - x0) / d
    x4 = x2 - h * (y1 - y0) / d
    y4 = y2 + h * (x1 - x0) / d

    out = [np.array([x3, y3]), np.array([x4, y4])]
    np.random.shuffle(out)
    return out


class RobotBosch(SimRobot):
    """
    Simulated Bosch SR450. Accepts the same arguments as
    ``ctu_bosch_sr450.RobotBosch`` plus the keyword-only simulator options of
    :class:`crs_sim.sim_robot.SimRobot` (``visualize``, ``speedup``,
    ``camera``, ``on_contact``, ...). ``tty_dev`` is ignored, also ``None``
    gives a working simulated robot.
    """
    #: Table top height in the base frame the launcher uses by default. The
    #: base frame is on the floor plate under axis 1 and the flange only goes
    #: down to z = 0.17 m, so the puzzle has to stand higher for the hoop to
    #: reach the base plate. An assumption, measure it on the real cell.
    default_table_z = 0.15

    def __init__(self, tty_dev: str | None = "/dev/mars", baudrate: int = 19200,
                 **sim_kwargs) -> None:
        self.link_lengths = np.array([0.25, 0.2])
        self._z_offset = 0.5  # what is the height of the end-effector when homed

        self._motors_ids = "ABCD"
        self._q_to_q_irc = np.array(
            [54286.97186604, 35061.99184485, 444000.0, 22686.85977431]
        )
        lower_bound_irc = np.array([-80536, -73434, -146520, -7919])
        upper_bound_irc = np.array([80536, 73434, 2220, 427637])
        self.q_min = self._irc_to_joint_values(lower_bound_irc)
        self.q_max = self._irc_to_joint_values(upper_bound_irc)
        self.q_home = np.array([0.0, 0.0, 0.0, 0.0])

        self._min_speed_irc256_per_ms = np.array([1000, 1000, 1000, 1000])
        self._max_speed_irc256_per_ms = np.array([10000, 10000, 5000, 5000])
        self._default_speed_irc256_per_ms = np.array([10000, 10000, 5000, 5000])
        self._min_acceleration_irc_per_ms = np.array([0, 0, 1, 1])
        self._max_acceleration_irc_per_ms = np.array([100, 100, 100, 100])
        self._default_acceleration_irc_per_ms = np.array([30, 30, 20, 30])

        self._initialized = False
        self._max_velocity = np.zeros(4)
        self._set_speed_no_check(self._default_speed_irc256_per_ms)
        self._acceleration = self._default_acceleration_irc_per_ms.astype(float)
        sim_kwargs.setdefault("urdf_path", URDF_PATH_SR450)
        super().__init__(**sim_kwargs)

    # ------------------------------------------------------------------
    # Interface of ctu_bosch_sr450.RobotBosch
    # ------------------------------------------------------------------
    def release(self):
        """Release errors and reset the control unit (no-op)."""

    def reset_motors(self):
        """Reset the robot's motors (no-op)."""

    def close(self):
        """Close the connection to the robot (no-op)."""

    def initialize(self, home: bool = True):
        """
        Initialize the robot: reset speed and acceleration to defaults and
        perform hard and soft homing if ``home`` is True. The real robot asks
        you to press the ARM POWER button here, the simulator does not wait.
        """
        self.set_speed(self._default_speed_irc256_per_ms)
        self.set_acceleration(self._default_acceleration_irc_per_ms)
        if home:
            self.hard_home()
            self.soft_home()
        self._initialized = True

    def _joint_values_to_irc(self, joint_values: ArrayLike) -> np.ndarray:
        """Converts joint values [rad, m] to IRC."""
        j = np.asarray(joint_values)
        assert j.shape == (len(self._motors_ids),), "Incorrect number of joints."
        return np.rint(joint_values * self._q_to_q_irc)

    def _irc_to_joint_values(self, irc: ArrayLike) -> np.ndarray:
        """Converts IRC to joint values [rad, m]."""
        irc = np.asarray(irc)
        assert irc.shape == (len(self._motors_ids),), "Incorrect number of joints."
        return irc / self._q_to_q_irc

    def set_speed(self, speed_irc256_ms: ArrayLike):
        """Sets the speed for each motor in IRC*256/msec."""
        assert len(speed_irc256_ms) == len(self._motors_ids)
        self._set_speed_no_check(speed_irc256_ms)

    def set_speed_relative(self, fraction: float):
        """Sets the speed for each motor as a fraction (0-1) of the maximum speed."""
        assert 0 <= fraction <= 1, "Fraction must be in [0,1]."
        s = self._min_speed_irc256_per_ms + fraction * (
            self._max_speed_irc256_per_ms - self._min_speed_irc256_per_ms
        )
        self.set_speed(s)

    def set_acceleration(self, acceleration_irc_ms: ArrayLike):
        """Sets the acceleration for each motor in IRC/msec. Stored only, the
        simulator uses a smooth velocity profile instead."""
        assert len(acceleration_irc_ms) == len(self._motors_ids)
        self._acceleration = np.asarray(acceleration_irc_ms, dtype=float)

    def set_acceleration_relative(self, fraction: float):
        """Sets the acceleration for each motor as a fraction (0-1) of the
        maximum acceleration."""
        assert 0 <= fraction <= 1, "Fraction must be in [0,1]."
        a = self._min_acceleration_irc_per_ms + fraction * (
            self._max_acceleration_irc_per_ms - self._min_acceleration_irc_per_ms
        )
        self.set_acceleration(a)

    def hard_home(self):
        """Simulated hard home: the joints are driven to q = 0 one by one, the
        prismatic joint first, then A, B and D (the real order)."""
        self.set_speed(self._default_speed_irc256_per_ms)
        self.set_acceleration(self._default_acceleration_irc_per_ms)
        for axis in "CABD":
            q = self.get_q()
            q[self._motors_ids.index(axis)] = 0.0
            self._start_motion(q)
            self.wait_for_motion_stop()

    def soft_home(self):
        """Moves the robot to the home position using a coordinated movement."""
        self._start_motion(self.q_home)
        self.wait_for_motion_stop()

    def move_to_q(self, q: ArrayLike):
        """
        Moves the robot to the given joint configuration [rad] using a coordinated
        movement. Non-blocking, use wait_for_motion_stop() to wait.
        """
        assert self._initialized, "You need to initialize the robot before moving it."
        assert np.all(q >= self.q_min) and np.all(
            q <= self.q_max
        ), "Joint limits violated."
        self._raise_contacts()
        self._start_motion(q)

    def get_q(self) -> np.ndarray:
        """Gets the current joint configuration [rad]."""
        return self._current_q()

    def in_motion(self) -> bool:
        """Returns whether the robot is in motion."""
        return not self._motion.finished(time.monotonic())

    def wait_for_motion_stop(self):
        """Waits until the robot stops moving."""
        self._wait_for_motion()

    def in_limits(self, q: ArrayLike) -> bool:
        """Returns whether the given joint configuration is within the joint limits."""
        return np.all(q >= self.q_min) and np.all(q <= self.q_max)

    def fk(self, q: ArrayLike) -> tuple[float, float, float, float]:
        """
        Forward kinematics: (x, y, z, phi), the end-effector position in the
        base frame and its rotation phi about the vertical axis.
        """
        q = np.asarray(q)
        assert q.shape == (len(self._motors_ids),), "Incorrect number of joints."
        l1, l2 = self.link_lengths
        x = l1 * np.cos(q[0]) + l2 * np.cos(q[0] + q[1])
        y = l1 * np.sin(q[0]) + l2 * np.sin(q[0] + q[1])
        z = self._z_offset + q[2]
        phi = np.arctan2(np.sin(q[3]), np.cos(q[3]))
        return x, y, z, phi

    def ik_xyz(
        self, x: float, y: float, z: float = 0, q3: float = 0
    ) -> list[np.ndarray]:
        """
        Inverse kinematics for an end-effector position, the last joint is set
        to q3. Returns all solutions within the joint limits.
        """
        sols = []
        bs = circle_circle_intersection(
            np.zeros(2), self.link_lengths[0], [x, y], self.link_lengths[1]
        )
        for b in bs:
            q = np.zeros(4)
            q[0] = np.arctan2(*b[::-1])
            rot = np.array(
                [[np.cos(q[0]), -np.sin(q[0])], [np.sin(q[0]), np.cos(q[0])]]
            )
            d = rot.T @ (np.asarray([x, y]) - b)
            q[1] = np.arctan2(*d[::-1])
            q[2] = z - self._z_offset
            q[3] = q3
            if self.in_limits(q):
                sols.append(q)
        np.random.shuffle(sols)
        return sols

    def ik(
        self, x: float, y: float, z: float = 0.0, phi: float = 0
    ) -> list[np.ndarray]:
        """
        Inverse kinematics for an end-effector pose (x, y, z, phi), all tool
        rotations q[3] = phi + k 2 pi within the limits. Returns all solutions
        within the joint limits, an empty list if there is none.
        """
        phi = np.arctan2(np.sin(phi), np.cos(phi))  # normalize to [-pi,pi]
        sols = self.ik_xyz(x, y, z, phi)
        for k in range(1, int((self.q_max[-1] - self.q_min[-1]) / (2 * np.pi)) + 1):
            for plus_minus in [-1, 1]:
                q3 = phi + plus_minus * k * 2 * np.pi
                if self.q_min[-1] <= q3 <= self.q_max[-1]:
                    sols.extend(self.ik_xyz(x, y, z, q3))
        np.random.shuffle(sols)
        return sols

    # ------------------------------------------------------------------
    # Simulator only
    # ------------------------------------------------------------------
    def flange_pose(self, q: ArrayLike) -> np.ndarray:
        """``fk`` as a 4x4 pose: origin at (x, y, z), z axis along the tool
        (down), x axis opposite to phi, so the hoop points along phi."""
        x, y, z, phi = self.fk(q)
        return pose([x, y, z], phi) @ ry(np.pi)

    def model_q(self, q: ArrayLike) -> np.ndarray:
        """URDF joint values: the URDF's last joint is relative to arm 2."""
        m = np.asarray(q, dtype=float).copy()
        m[3] = q[3] - q[0] - q[1]
        return m

    def _set_speed_no_check(self, speed_irc256_ms: ArrayLike) -> None:
        irc_per_s = np.asarray(speed_irc256_ms, dtype=float) * 1000 / 256
        self._max_velocity = irc_per_s / self._q_to_q_irc

    def _quantize(self, q: ArrayLike) -> np.ndarray:
        """The controller works in IRC, so the reached q is quantized."""
        return self._irc_to_joint_values(self._joint_values_to_irc(np.asarray(q, dtype=float)))
