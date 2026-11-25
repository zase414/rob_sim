"""
Standalone simulator of the CRS93 / CRS97, Mitsubishi RV-6S and Bosch SR450
robotic arms with the interfaces of ``ctu_crs``, ``ctu_mitsubishi`` and
``ctu_bosch_sr450``.

    from crs_sim import CRS97      # instead of: from ctu_crs import CRS97
    from crs_sim import CRS93      # instead of: from ctu_crs import CRS93
    from crs_sim import Rv6s       # instead of: from ctu_mitsubishi import Rv6s
    from crs_sim import RobotBosch # instead of: from ctu_bosch_sr450 import RobotBosch
"""
from .bosch import URDF_PATH_SR450, RobotBosch
from .camera import Camera
from .contact import ContactChecker, ContactError
from .crs93 import CRS93, CRS93_PARAMS
from .crs97 import CRS97, CRS97_PARAMS, URDF_PATH
from .crs_robot import CRSRobot
from .puzzles import Puzzle
from .rv6s import URDF_PATH_RV6S, Rv6s, Rv6sGripper
from .sim_robot import SimRobot
from .visualizer import Visualizer, load_mesh

__all__ = ["CRS93", "CRS93_PARAMS", "CRS97", "CRS97_PARAMS", "CRSRobot",
           "Camera", "ContactChecker", "ContactError", "Puzzle", "RobotBosch",
           "Rv6s", "Rv6sGripper", "SimRobot", "URDF_PATH", "URDF_PATH_RV6S",
           "URDF_PATH_SR450", "Visualizer", "load_mesh"]
