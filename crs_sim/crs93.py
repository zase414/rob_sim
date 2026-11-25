"""
Simulated CRS93 robot, drop-in replacement of ``ctu_crs.CRS93``.

The CRS93 is the same arm as the CRS97 (same kinematics and URDF), it
differs in its controller parameters: home switch positions, joint limits,
speeds and gripper.
"""
from __future__ import annotations

from .crs97 import URDF_PATH
from .crs_robot import CRSRobot

# Same values as ctu_crs/params_crs93.yaml
CRS93_PARAMS = {
    "hh_irc": [-181650, -349, -62200, 99200, 8300, -96500],
    "hh_sequence": ["B", "AC", "DEF"],
    "lower_bound_irc": [-370000, -100000, 63000, 203500, 67000, 4600],
    "upper_bound_irc": [10000, 100000, -190000, -5000, -50000, -199500],
    "default_speed_irc256_per_ms": [7680, 2048, 5120, 7680, 7680, 19200],
    "default_acceleration_irc_per_ms": [51, 10, 20, 38, 128, 768],
    "gripper": {"bounds": [840, 103], "axis": "G"},
    "camera_name": "camera-crs93",
}


class CRS93(CRSRobot):
    """
    Simulated CRS93. Accepts the same arguments as ``ctu_crs.CRS93`` plus the
    keyword-only simulator options of :class:`CRSRobot` (``visualize``,
    ``open_browser``, ``speedup``, ``q0``, ``render_fps``, ``camera``).
    """
    def __init__(self, tty_dev: str | None = "/dev/mars", baudrate: int = 19200,
                 **sim_kwargs) -> None:
        sim_kwargs.setdefault("urdf_path", URDF_PATH)
        super().__init__(tty_dev, baudrate, **sim_kwargs, **CRS93_PARAMS)
